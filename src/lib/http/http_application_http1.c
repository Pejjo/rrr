/*

Read Route Record

Copyright (C) 2020 Atle Solbakken atle@goliathdns.no

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <string.h>
#include <stdlib.h>
#include <limits.h>

#include "../log.h"

#include "http_application.h"
#include "http_application_http1.h"
#ifdef RRR_WITH_NGHTTP2
#	include "http_application_http2.h"
#endif
#include "http_application_internals.h"

#include "http_transaction.h"
#include "http_part.h"
#include "http_part_parse.h"
#include "http_part_multipart.h"
#include "http_util.h"
#include "http_common.h"
#include "../random.h"
#include "../string_builder.h"
#include "../websocket/websocket.h"
#include "../net_transport/net_transport.h"
#include "../util/gnu.h"
#include "../sha1/sha1.h"
#include "../util/base64.h"
#include "../http2/http2.h"

struct rrr_http_application_http1 {
	RRR_HTTP_APPLICATION_HEAD;
	enum rrr_http_upgrade_mode upgrade_active;
	struct rrr_websocket_state ws_state;

	// HTTP1 only has one active transaction at a time
	struct rrr_http_transaction *active_transaction;
};

static void __rrr_http_application_http1_destroy (struct rrr_http_application *app) {
	struct rrr_http_application_http1 *http1 = (struct rrr_http_application_http1 *) app;
	rrr_websocket_state_clear_all(&http1->ws_state);
	rrr_http_transaction_decref_if_not_null(http1->active_transaction);
	free(http1);
}

static void __rrr_http_application_http1_transaction_set (
		struct rrr_http_application_http1 *http1,
		struct rrr_http_transaction *transaction
) {
	rrr_http_transaction_decref_if_not_null(http1->active_transaction);
	rrr_http_transaction_incref(transaction);
	http1->active_transaction = transaction;
}

static int __rrr_http_application_http1_request_send_make_headers_callback (
		struct rrr_http_header_field *field,
		void *arg
) {
	struct rrr_string_builder *builder = arg;

	// Note : Only plain values supported
	if (!rrr_nullsafe_str_isset(field->value)) {
		return 0;
	}

	int ret = 0;

	RRR_HTTP_UTIL_SET_TMP_NAME_FROM_NULLSAFE(name,field->name);
	RRR_HTTP_UTIL_SET_TMP_NAME_FROM_NULLSAFE(value,field->value);

	ret |= rrr_string_builder_append(builder, name);
	ret |= rrr_string_builder_append(builder, ": ");
	ret |= rrr_string_builder_append(builder, value);
	ret |= rrr_string_builder_append(builder, "\r\n");

	return ret;
}

static void __rrr_http_application_http1_free_dbl_ptr (void *ptr) {
	void *to_free = *((void **) ptr);
	RRR_FREE_IF_NOT_NULL(to_free);
}

static int __rrr_http_application_http1_multipart_form_data_body_send_wrap_chunk (
		struct rrr_net_transport_handle *handle,
		const void *data,
		ssize_t size
) {
	if (size < 0) {
		RRR_BUG("Size was < 0 in __rrr_http_application_http1_multipart_form_data_body_send_wrap_chunk\n");
	}
	if (size == 0) {
		return RRR_HTTP_OK;
	}

	int ret = 0;

	char buf[128];
	sprintf(buf, "%x\r\n", (unsigned int) size);

	if ((ret = rrr_net_transport_ctx_send_blocking (handle, buf, strlen(buf))) != RRR_NET_TRANSPORT_SEND_OK) {
		goto out;
	}

	if ((ret = rrr_net_transport_ctx_send_blocking (handle, data, size)) != RRR_NET_TRANSPORT_SEND_OK) {
		goto out;
	}

	if ((ret = rrr_net_transport_ctx_send_blocking (handle, "\r\n", 2)) != RRR_NET_TRANSPORT_SEND_OK) {
		goto out;
	}

	out:
	return ret;
}

static int __rrr_http_application_http1_multipart_field_send (
		struct rrr_net_transport_handle *handle,
		const char *boundary,
		struct rrr_http_field *node,
		int is_first
) {
	int ret = 0;

	char *name_buf = NULL;
	char *name_buf_full = NULL;
	char *content_type_buf = NULL;
	char *body_buf = NULL;

	if (rrr_nullsafe_str_isset(node->name)) {
		if ((name_buf = rrr_http_util_quote_header_value_nullsafe(node->name, '"', '"')) == NULL) {
			RRR_MSG_0("Could not quote field name_buf in __rrr_http_application_http1_multipart_field_send\n");
			ret = 1;
			goto out;
		}

		if ((ret = rrr_asprintf (&name_buf_full, "; name=%s", name_buf)) <= 0) {
			RRR_MSG_0("Could not create name_buf_full in __rrr_http_application_http1_multipart_field_send return was %i\n", ret);
			ret = 1;
			goto out;
		}
	}

	if (rrr_nullsafe_str_isset(node->content_type)) {
		RRR_HTTP_UTIL_SET_TMP_NAME_FROM_NULLSAFE(value,node->content_type);
		if ((ret = rrr_asprintf (&content_type_buf, "Content-Type: %s\r\n", value)) <= 0) {
			RRR_MSG_0("Could not create content_type_buf in __rrr_http_application_http1_multipart_field_send return was %i\n", ret);
			ret = 1;
			goto out;
		}
	}

	RRR_FREE_IF_NOT_NULL(body_buf);
	if ((ret = rrr_asprintf (
			&body_buf,
			"%s--%s\r\n"
			"Content-Disposition: form-data%s\r\n"
			"%s\r\n",
			(is_first ? "" : "\r\n"),
			boundary,
			(name_buf_full != NULL ? name_buf_full : ""),
			(content_type_buf != NULL ? content_type_buf : "")
	)) < 0) {
		RRR_MSG_0("Could not create content type string and body  in __rrr_http_application_http1_multipart_field_send return was %i\n", ret);
		ret = 1;
		goto out;
	}

	if ((ret = __rrr_http_application_http1_multipart_form_data_body_send_wrap_chunk(handle, body_buf, strlen(body_buf))) != 0) {
		RRR_MSG_0("Could not send form part of HTTP request in __rrr_http_application_http1_multipart_field_send A\n");
		goto out;
	}

	if (rrr_nullsafe_str_isset(node->value)) {
		if ((ret = __rrr_http_application_http1_multipart_form_data_body_send_wrap_chunk(
				handle,
				node->value->str,
				node->value->len
		)) != 0) {
			RRR_MSG_0("Could not send form part of HTTP request in __rrr_http_application_http1_multipart_field_send B\n");
			goto out;
		}
	}

	out:
	RRR_FREE_IF_NOT_NULL(name_buf);
	RRR_FREE_IF_NOT_NULL(name_buf_full);
	RRR_FREE_IF_NOT_NULL(content_type_buf);
	RRR_FREE_IF_NOT_NULL(body_buf);
	return ret;
}

static int __rrr_http_application_http1_multipart_form_data_body_send (
		struct rrr_net_transport_handle *handle,
		struct rrr_http_part *request_part
) {
	int ret = 0;

	char *body_buf = NULL;
	char *boundary_buf = NULL;

	pthread_cleanup_push(__rrr_http_application_http1_free_dbl_ptr, &body_buf);
	pthread_cleanup_push(__rrr_http_application_http1_free_dbl_ptr, &boundary_buf);

	// RFC7578

	if ((ret = rrr_asprintf (&boundary_buf, "RRR%u", (unsigned int) rrr_rand())) < 0) {
		RRR_MSG_0("Could not create boundary_buf string in __rrr_http_application_http1_multipart_form_data_body_send return was %i\n", ret);
		ret = 1;
		goto out;
	}

	{
		RRR_FREE_IF_NOT_NULL(body_buf);
		if ((ret = rrr_asprintf (
				&body_buf,
				"Content-Type: multipart/form-data; boundary=%s\r\n"
				"Transfer-Encoding: chunked\r\n\r\n",
				boundary_buf
		)) < 0) {
			RRR_MSG_0("Could not create content type string in __rrr_http_application_http1_multipart_form_data_body_send return was %i\n", ret);
			ret = 1;
			goto out;
		}

		if ((ret = rrr_net_transport_ctx_send_blocking(handle, body_buf, strlen(body_buf))) != 0) {
			RRR_DBG_1("Could not send first part of HTTP request in __rrr_http_application_http1_multipart_form_data_body_send\n");
			goto out;
		}
	}

	// All sends below this point must be wrapped inside chunk sender

	int is_first = 1;
	RRR_LL_ITERATE_BEGIN(&request_part->fields, struct rrr_http_field);
		if ((ret = __rrr_http_application_http1_multipart_field_send(handle, boundary_buf, node, is_first)) != 0) {
			goto out;
		}
		is_first = 0;
	RRR_LL_ITERATE_END();

	{
		RRR_FREE_IF_NOT_NULL(body_buf);
		if ((ret = rrr_asprintf (
				&body_buf,
				"\r\n--%s--\r\n",  // <-- ONE CRLF AFTER BODY AND ONE AT THE VERY END
				boundary_buf
		)) < 0) {
			RRR_MSG_0("Could not create last boundary in __rrr_http_application_http1_multipart_form_data_body_send return was %i\n", ret);
			ret = 1;
			goto out;
		}

		if ((ret = __rrr_http_application_http1_multipart_form_data_body_send_wrap_chunk(handle, body_buf, strlen(body_buf))) != 0) {
			RRR_MSG_0("Could not send last part of HTTP request in __rrr_http_application_http1_multipart_form_data_body_send\n");
			goto out;
		}
	}

	if ((ret = rrr_net_transport_ctx_send_blocking(handle, "0\r\n\r\n", 5)) != 0) {
		RRR_DBG_1("Could not send terminating chunk of HTTP request in __rrr_http_application_http1_multipart_form_data_body_send\n");
		goto out;
	}

	out:
	pthread_cleanup_pop(1);
	pthread_cleanup_pop(1);

	return ret;
}

static int __rrr_http_application_http1_post_x_www_form_body_send (
		struct rrr_net_transport_handle *handle,
		struct rrr_http_part *request_part,
		int no_urlencoding
) {
	int ret = 0;
	char *body_buf = NULL;
	char *header_buf = NULL;

	pthread_cleanup_push(__rrr_http_application_http1_free_dbl_ptr, &body_buf);
	pthread_cleanup_push(__rrr_http_application_http1_free_dbl_ptr, &header_buf);

	rrr_length body_size = 0;
	if (no_urlencoding == 0) {
		body_buf = rrr_http_field_collection_to_urlencoded_form_data(&body_size, &request_part->fields);
	}
	else {
		body_buf = rrr_http_field_collection_to_raw_form_data(&body_size, &request_part->fields);
	}

	if (body_buf == NULL) {
		RRR_MSG_0("Could not create body in __rrr_http_application_http1_send_post_urlencoded_body\n");
		ret = 1;
		goto out;
	}

	if ((ret = rrr_asprintf (
			&header_buf,
			"Content-Type: application/x-www-form-urlencoded\r\n"
			"Content-Length: %" PRIrrrl "\r\n\r\n",
			body_size
	)) < 0) {
		RRR_MSG_0("Could not create content type string in __rrr_http_application_http1_send_get_body return was %i\n", ret);
		ret = 1;
		goto out;
	}

	if ((ret = rrr_net_transport_ctx_send_blocking (handle, header_buf, strlen(header_buf))) != 0) {
		RRR_DBG_1("Could not send GET body header in __rrr_http_application_http1_send_get_body\n");
		goto out;
	}

	if ((ret = rrr_net_transport_ctx_send_blocking (handle, body_buf, body_size)) != 0) {
		RRR_DBG_1("Could not send GET body in __rrr_http_application_http1_send_get_body\n");
		goto out;
	}

	out:
	pthread_cleanup_pop(1);
	pthread_cleanup_pop(1);
	return ret;
}

int __rrr_http_application_http1_request_send (
		RRR_HTTP_APPLICATION_REQUEST_SEND_ARGS
) {
	int ret = 0;

	struct rrr_http_application_http1 *http1 = (struct rrr_http_application_http1 *) application;
	struct rrr_http_part *request_part = transaction->request_part;

	char *request_buf = NULL;
	char *host_buf = NULL;
	char *user_agent_buf = NULL;
	char *uri_tmp = NULL;
	char *extra_uri_tmp = NULL;
	char *websocket_key_tmp = NULL;
	char *http2_upgrade_settings_tmp = NULL;

	pthread_cleanup_push(__rrr_http_application_http1_free_dbl_ptr, &request_buf);
	pthread_cleanup_push(__rrr_http_application_http1_free_dbl_ptr, &host_buf);
	pthread_cleanup_push(__rrr_http_application_http1_free_dbl_ptr, &user_agent_buf);
	pthread_cleanup_push(__rrr_http_application_http1_free_dbl_ptr, &uri_tmp);
	pthread_cleanup_push(__rrr_http_application_http1_free_dbl_ptr, &extra_uri_tmp);
	pthread_cleanup_push(__rrr_http_application_http1_free_dbl_ptr, &websocket_key_tmp);
	pthread_cleanup_push(__rrr_http_application_http1_free_dbl_ptr, &http2_upgrade_settings_tmp);

	rrr_length extra_uri_size = 0;
	const char *extra_uri_separator = "";

	const char *uri_to_use = transaction->uri_str;

	struct rrr_string_builder *header_builder = NULL;

	__rrr_http_application_http1_transaction_set(http1, transaction);

	if (rrr_string_builder_new(&header_builder) != 0) {
		RRR_MSG_0("Failed to create string builder in __rrr_http_application_http1_request_send\n");
		ret = 1;
		goto out_final;
	}

	pthread_cleanup_push(rrr_string_builder_destroy_void, header_builder);

	host_buf = rrr_http_util_quote_header_value(host, strlen(host), '"', '"');
	if (host_buf == NULL) {
		RRR_MSG_0("Invalid host '%s' in __rrr_http_application_http1_request_send\n", host);
		ret = 1;
		goto out;
	}

	user_agent_buf = rrr_http_util_quote_header_value(user_agent, strlen(user_agent), '"', '"');
	if (user_agent_buf == NULL) {
		RRR_MSG_0("Invalid user agent '%s' in __rrr_http_application_http1_request_send\n", user_agent);
		ret = 1;
		goto out;
	}

	if (transaction->method == RRR_HTTP_METHOD_GET && RRR_LL_COUNT(&request_part->fields) > 0) {
		extra_uri_tmp  = rrr_http_field_collection_to_urlencoded_form_data(&extra_uri_size, &request_part->fields);

		if (strchr(transaction->uri_str, '?') != NULL) {
			// Append to existing ?-query string in GET URI
			extra_uri_separator = "&";
		}
		else {
			extra_uri_separator = "?";
		}

		rrr_biglength uri_orig_len = strlen(uri_to_use);
		RRR_TYPES_BUG_IF_LENGTH_EXCEEDED(uri_orig_len,"rrr_http_application_http1_request_send");

		if ((uri_tmp = malloc(uri_orig_len + extra_uri_size + 1 + 1)) == NULL) { // + separator + 0
			RRR_MSG_0("Could not allocate memory for new URI in __rrr_http_application_http1_request_send\n");
			ret = 1;
			goto out;
		}

		char *wpos = uri_tmp;

		memcpy(wpos, uri_to_use, uri_orig_len);
		wpos += uri_orig_len;

		*wpos = *extra_uri_separator;
		wpos++;

		memcpy(wpos, extra_uri_tmp, extra_uri_size);
		wpos += extra_uri_size;

		*wpos = '\0';

		uri_to_use = uri_tmp;
	}

	if (upgrade_mode == RRR_HTTP_UPGRADE_MODE_WEBSOCKET) {
		if (transaction->method != RRR_HTTP_METHOD_GET) {
			RRR_BUG("BUG: HTTP method was not GET while upgrade mode was WebSocket\n");
		}
		if ((ret = rrr_http_part_header_field_push(request_part, "connection", "Upgrade")) != 0) {
			goto out;
		}
		if ((ret = rrr_http_part_header_field_push(request_part, "upgrade", "websocket")) != 0) {
			goto out;
		}
		if ((ret = rrr_websocket_state_get_key_base64 (&websocket_key_tmp, &http1->ws_state)) != 0) {
			goto out;
		}
		if ((ret = rrr_http_part_header_field_push(request_part, "sec-websocket-key", websocket_key_tmp)) != 0) {
			goto out;
		}
		if ((ret = rrr_http_part_header_field_push(request_part, "sec-websocket-version", "13")) != 0) {
			goto out;
		}

		rrr_websocket_state_set_client_mode(&http1->ws_state);
	}
	else if (upgrade_mode == RRR_HTTP_UPGRADE_MODE_HTTP2) {
		if (transaction->method != RRR_HTTP_METHOD_GET && transaction->method != RRR_HTTP_METHOD_HEAD) {
			RRR_BUG("BUG: HTTP method was not GET or HEAD while upgrade mode was HTTP2\n");
		}
#ifdef RRR_WITH_NGHTTP2
		if ((ret = rrr_http_part_header_field_push(request_part, "connection", "Upgrade, HTTP2-Settings")) != 0) {
			goto out;
		}
		if ((ret = rrr_http_part_header_field_push(request_part, "upgrade", "h2c")) != 0) {
			goto out;
		}
		if (rrr_http2_pack_upgrade_request_settings(&http2_upgrade_settings_tmp) != 0) {
			ret = RRR_HTTP_HARD_ERROR;
			goto out;
		}
		if ((ret = rrr_http_part_header_field_push(request_part, "http2-settings", http2_upgrade_settings_tmp)) != 0) {
			goto out;
		}
#else
		RRR_MSG_0("Warning: HTTP client attempted to send GET request with upgrade to HTTP2, but RRR is not built with NGHTTP2. Proceeding using HTTP/1.1.\n");
#endif /* RRR_WITH_NGHTTP2 */
	}

	if ((ret = rrr_asprintf (
			&request_buf,
			"%s %s HTTP/1.1\r\n"
			"Host: %s\r\n"
			"User-Agent: %s\r\n"
			"Accept-Charset: UTF-8\r\n",
			RRR_HTTP_METHOD_TO_STR_CONFORMING(transaction->method),
			uri_to_use,
			host_buf,
			user_agent_buf
	)) < 0) {
		RRR_MSG_0("Error while making request string in rrr_http_application_http1_request_send return was %i\n", ret);
		ret = 1;
		goto out;
	}

	if ((ret = rrr_net_transport_ctx_send_blocking (handle, request_buf, strlen(request_buf))) != 0) {
		RRR_DBG_1("Could not send first part of HTTP request header in __rrr_http_application_http1_request_send\n");
		goto out;
	}

	rrr_string_builder_clear(header_builder);

	if (rrr_http_part_header_fields_iterate (
			request_part,
			__rrr_http_application_http1_request_send_make_headers_callback,
			header_builder
	) != 0) {
		RRR_MSG_0("Failed to make header fields in __rrr_http_application_http1_request_send\n");
		ret = 1;
		goto out;
	}

	ssize_t header_builder_length = rrr_string_builder_length(header_builder);
	if (header_builder_length > 0) {
		RRR_FREE_IF_NOT_NULL(request_buf);
		request_buf = rrr_string_builder_buffer_takeover(header_builder);
		if ((ret = rrr_net_transport_ctx_send_blocking (handle, request_buf, header_builder_length)) != 0) {
			RRR_MSG_0("Could not send second part of HTTP request header in __rrr_http_application_http1_request_send\n");
			goto out;
		}
	}

	if (transaction->method != RRR_HTTP_METHOD_GET && RRR_LL_COUNT(&request_part->fields) > 0) {
		if (transaction->method == RRR_HTTP_METHOD_POST_MULTIPART_FORM_DATA) {
			if ((ret = __rrr_http_application_http1_multipart_form_data_body_send (handle, request_part)) != 0) {
				RRR_MSG_0("Could not send POST multipart body in __rrr_http_application_http1_request_send\n");
				goto out;
			}
		}
		else if (transaction->method == RRR_HTTP_METHOD_POST_URLENCODED) {
			if ((ret = __rrr_http_application_http1_post_x_www_form_body_send (handle, request_part, 0)) != 0) {
				RRR_MSG_0("Could not send POST urlencoded body in __rrr_http_application_http1_request_send\n");
				goto out;
			}
		}
		else if (transaction->method == RRR_HTTP_METHOD_POST_URLENCODED_NO_QUOTING) {
			// Application may choose to quote itself (influxdb has special quoting)
			if ((ret = __rrr_http_application_http1_post_x_www_form_body_send (handle, request_part, 1)) != 0) {
				RRR_MSG_0("Could not send POST urlencoded body in __rrr_http_application_http1_request_send\n");
				goto out;
			}
		}

		// TODO : If we use plain text or octet stream method, simply concatenate and encode all fields

		else {
			RRR_MSG_0("Unknown request method %s for request with fields set\n", RRR_HTTP_METHOD_TO_STR(transaction->method));
			ret = 1;
			goto out;
		}
	}
	else if ((ret = rrr_net_transport_ctx_send_blocking (handle, "\r\n", strlen("\r\n"))) != 0) {
		RRR_MSG_0("Could not send last \\r\\n in __rrr_http_application_http1_request_send\n");
		goto out;
	}

	out:
		pthread_cleanup_pop(1);
	out_final:
		pthread_cleanup_pop(1);
		pthread_cleanup_pop(1);
		pthread_cleanup_pop(1);
		pthread_cleanup_pop(1);
		pthread_cleanup_pop(1);
		pthread_cleanup_pop(1);
		pthread_cleanup_pop(1);
		return ret;
}

struct rrr_http_application_http1_send_header_field_callback_data {
	struct rrr_net_transport_handle *handle;
};

static int __rrr_http_application_http1_send_header_field_callback (struct rrr_http_header_field *field, void *arg) {
	struct rrr_http_application_http1_send_header_field_callback_data *callback_data = arg;

	int ret = 0;

	char *send_data = NULL;
	size_t send_data_length = 0;

	if (!rrr_nullsafe_str_isset(field->name) || !rrr_nullsafe_str_isset(field->value)) {
		RRR_BUG("BUG: Name or value was NULL in __rrr_http_application_http1_send_header_field_callback\n");
	}
	if (RRR_LL_COUNT(&field->fields) > 0) {
		RRR_BUG("BUG: Subvalues were present in __rrr_http_application_http1_send_header_field_callback, this is not supported\n");
	}

	pthread_cleanup_push(__rrr_http_application_http1_free_dbl_ptr, &send_data);

	RRR_HTTP_UTIL_SET_TMP_NAME_FROM_NULLSAFE(name, field->name);
	RRR_HTTP_UTIL_SET_TMP_NAME_FROM_NULLSAFE(value, field->value);

	if ((send_data_length = rrr_asprintf(&send_data, "%s: %s\r\n", name, value)) <= 0) {
		RRR_MSG_0("Could not allocate memory for header line in __rrr_http_application_http1_send_header_field_callback\n");
		ret = 1;
		goto out;
	}

	// Hack to create Camel-Case header names (before : only)
	int next_to_upper = 1;
	for (size_t i = 0; i < send_data_length; i++) {
		if (send_data[i] == ':' || send_data[i] == '\0') {
			break;
		}

		if (next_to_upper) {
			if (send_data[i] >= 'a' && send_data[i] <= 'z') {
				send_data[i] -= ('a' - 'A');
			}
		}

		next_to_upper = (send_data[i] == '-' ? 1 : 0);
	}

	if ((ret = rrr_net_transport_ctx_send_blocking(callback_data->handle, send_data, send_data_length)) != 0) {
		RRR_DBG_1("Error: Send failed in __rrr_http_application_http1_send_header_field_callback\n");
		goto out;
	}

	out:
	pthread_cleanup_pop(1);
	return ret;
}

int __rrr_http_application_http1_response_send (
		RRR_HTTP_APPLICATION_RESPONSE_SEND_ARGS
) {
	int ret = 0;

	(void)(application);

	struct rrr_http_part *response_part = transaction->response_part;

	if (response_part->response_raw_data_nullsafe != NULL) {
		if ((ret = rrr_net_transport_ctx_send_blocking (
				handle,
				response_part->response_raw_data_nullsafe->str,
				response_part->response_raw_data_nullsafe->len
		)) != 0 ) {
			goto out_err;
		}
		goto out;
	}

	if (response_part->response_code == 0) {
		RRR_BUG("BUG: Response code was not set in rrr_http_application_http1_send_response\n");
	}

	const char *response_str = NULL;

	switch (response_part->response_code) {
		case RRR_HTTP_RESPONSE_CODE_SWITCHING_PROTOCOLS:
			response_str = "HTTP/1.1 101 Switching Protocols\r\n";
			break;
		case RRR_HTTP_RESPONSE_CODE_OK:
			response_str = "HTTP/1.1 200 OK\r\n";
			break;
		case RRR_HTTP_RESPONSE_CODE_OK_NO_CONTENT:
			response_str = "HTTP/1.1 204 No Content\r\n";
			break;
		case RRR_HTTP_RESPONSE_CODE_ERROR_BAD_REQUEST:
			response_str = "HTTP/1.1 400 Bad Request\r\n";
			break;
		case RRR_HTTP_RESPONSE_CODE_ERROR_NOT_FOUND:
			response_str = "HTTP/1.1 404 Not Found\r\n";
			break;
		case RRR_HTTP_RESPONSE_CODE_INTERNAL_SERVER_ERROR:
			response_str = "HTTP/1.1 500 Internal Server Error\r\n";
			break;
		case RRR_HTTP_RESPONSE_CODE_GATEWAY_TIMEOUT:
			response_str = "HTTP/1.1 504 Gateway Timeout\r\n";
			break;
		case RRR_HTTP_RESPONSE_CODE_VERSION_NOT_SUPPORTED:
			response_str = "HTTP/1.1 504 Version Not Supported\r\n";
			break;
		default:
			RRR_BUG("BUG: Response code %i not implemented in rrr_http_application_http1_send_response\n",
					response_part->response_code);
	}

	if ((ret = rrr_net_transport_ctx_send_blocking(handle, response_str, strlen(response_str))) != 0) {
		goto out_err;
	}

	struct rrr_http_application_http1_send_header_field_callback_data callback_data = {
			handle
	};

	if ((ret = rrr_http_part_header_fields_iterate(response_part, __rrr_http_application_http1_send_header_field_callback, &callback_data)) != 0) {
		goto out_err;
	}

	if ((ret = rrr_net_transport_ctx_send_blocking(handle, "\r\n", 2)) != 0 ) {
		goto out_err;
	}

	goto out;
	out_err:
		RRR_MSG_0("Error while sending headers for HTTP client %i in rrr_http_application_http1_transport_ctx_send_response\n",
				handle->handle);
	out:
		return ret;

}

struct rrr_http_application_http1_receive_data {
	struct rrr_net_transport_handle *handle;
	struct rrr_http_application_http1 *http1;
	ssize_t received_bytes; // Used only for stall timeout and sleeping
	rrr_http_unique_id unique_id;
	int is_client;
	struct rrr_http_application *upgraded_application;
	int (*websocket_callback)(RRR_HTTP_APPLICATION_WEBSOCKET_HANDSHAKE_CALLBACK_ARGS);
	void *websocket_callback_arg;
	int (*callback)(RRR_HTTP_APPLICATION_RECEIVE_CALLBACK_ARGS);
	void *callback_arg;
	int (*raw_callback)(RRR_HTTP_APPLICATION_RAW_RECEIVE_CALLBACK_ARGS);
	void *raw_callback_arg;
};

static int __rrr_http_application_http1_websocket_make_accept_string (
		char **accept_str,
		const char *sec_websocket_key
) {
	int ret = 0;

	char *accept_str_tmp = NULL;
	char *accept_base64_tmp = NULL;

	if (rrr_asprintf(&accept_str_tmp, "%s%s", sec_websocket_key, RRR_HTTP_WEBSOCKET_GUID) <= 0) {
		RRR_MSG_0("Failed to concatenate accept-string in __rrr_http_session_make_websocket_accept_string\n");
		ret = RRR_HTTP_HARD_ERROR;
		goto out;
	}

	rrr_SHA1Context sha1_ctx = {0};
	rrr_SHA1Reset(&sha1_ctx);
	rrr_SHA1Input(&sha1_ctx, (const unsigned char *) accept_str_tmp, strlen(accept_str_tmp));

	if (!rrr_SHA1Result(&sha1_ctx) || sha1_ctx.Corrupted != 0 || sha1_ctx.Computed != 1) {
		RRR_MSG_0("Computation of SHA1 failed in __rrr_http_session_websocket_make_accept_string (Corrupt: %i - Computed: %i)\n",
				sha1_ctx.Corrupted, sha1_ctx.Computed);
		ret = RRR_HTTP_SOFT_ERROR;
		goto out;
	}

	rrr_SHA1toBE(&sha1_ctx);

	size_t accept_base64_length = 0;
	if ((accept_base64_tmp = (char *) rrr_base64_encode (
			(const unsigned char *) sha1_ctx.Message_Digest,
			sizeof(sha1_ctx.Message_Digest),
			&accept_base64_length
	)) == NULL) {
		RRR_MSG_0("Base64 encoding failed in __rrr_http_session_websocket_make_accept_string\n");
		ret = RRR_HTTP_SOFT_ERROR;
		goto out;
	}

	char *newline = strchr(accept_base64_tmp, '\n');
	if (newline) {
		*newline = '\0';
	}

	*accept_str = accept_base64_tmp;
	accept_base64_tmp = NULL;

	out:
	RRR_FREE_IF_NOT_NULL(accept_base64_tmp);
	RRR_FREE_IF_NOT_NULL(accept_str_tmp);
	return ret;
}

static int __rrr_http_application_http1_websocket_response_check_headers (
		struct rrr_http_part *response_part,
		struct rrr_websocket_state *ws_state
) {
	int ret = 0;

	char *sec_websocket_key_tmp = NULL;
	char *accept_str_tmp = NULL;

	const struct rrr_http_header_field *field_connection = rrr_http_part_header_field_get_with_value_case(response_part, "connection", "upgrade");
	const struct rrr_http_header_field *field_accept = rrr_http_part_header_field_get(response_part, "sec-websocket-accept");

	if (field_connection == NULL) {
		RRR_MSG_0("Missing 'Connection: upgrade' field in HTTP server WebSocket upgrade response\n");
		ret = RRR_HTTP_SOFT_ERROR;
		goto out;
	}

	if (field_accept == NULL) {
		RRR_MSG_0("Missing 'Sec-Websocket-Accept' field in HTTP server WebSocket upgrade response\n");
		ret = RRR_HTTP_SOFT_ERROR;
		goto out;
	}

	if ((ret = rrr_websocket_state_get_key_base64(&sec_websocket_key_tmp, ws_state)) != 0) {
		RRR_MSG_0("Failed to get key from WebSocket state in __rrr_http_session_request_receive_try_websocket\n");
		goto out;
	}

	if ((ret = __rrr_http_application_http1_websocket_make_accept_string(&accept_str_tmp, sec_websocket_key_tmp)) != 0) {
		RRR_MSG_0("Failed to make accept-string in __rrr_http_session_request_receive_try_websocket\n");
		goto out;
	}

	RRR_HTTP_UTIL_SET_TMP_NAME_FROM_NULLSAFE(response_accept_str,field_accept->value);
	if (strcmp(accept_str_tmp, response_accept_str) != 0) {
		RRR_MSG_0("WebSocket accept string from server mismatch (got '%s' but expected  '%s')\n",
				response_accept_str, accept_str_tmp);
		ret = RRR_HTTP_SOFT_ERROR;
		goto out;
	}

	out:
	RRR_FREE_IF_NOT_NULL(sec_websocket_key_tmp);
	RRR_FREE_IF_NOT_NULL(accept_str_tmp);
	return ret;
}

static int __rrr_application_http1_response_receive_callback (
		struct rrr_read_session *read_session,
		void *arg
) {
	struct rrr_http_application_http1_receive_data *receive_data = arg;
	struct rrr_http_transaction *transaction = receive_data->http1->active_transaction;

	char *orig_http2_settings_tmp = NULL;

	int ret = 0;

	if (RRR_DEBUGLEVEL_3) {
		rrr_http_part_header_dump(transaction->response_part);
	}

	RRR_DBG_3("HTTP reading complete, data length is %li response length is %li header length is %li\n",
			transaction->response_part->data_length,
			transaction->response_part->headroom_length,
			transaction->response_part->header_length
	);

	if (receive_data->raw_callback != NULL) {
		if ((ret = receive_data->raw_callback (
				read_session->rx_buf_ptr,
				read_session->rx_buf_wpos,
				0,
				receive_data->raw_callback_arg
		)) != 0) {
			RRR_MSG_0("Error %i from raw callback in __rrr_application_http1_response_receive_callback\n", ret);
			goto out;
		}
	}

	enum rrr_http_upgrade_mode upgrade_mode = RRR_HTTP_UPGRADE_MODE_NONE;

	if (transaction->response_part->response_code == RRR_HTTP_RESPONSE_CODE_SWITCHING_PROTOCOLS) {
		if (receive_data->websocket_callback == NULL) {
			RRR_MSG_0("Unexpected HTTP 101 Switching Protocols response from server, websocket upgrade was not requested\n");
			ret = RRR_HTTP_SOFT_ERROR;
			goto out;
		}

		const struct rrr_http_header_field *field_upgrade_websocket = rrr_http_part_header_field_get_with_value_case(transaction->response_part, "upgrade", "websocket");
		const struct rrr_http_header_field *field_upgrade_h2c = rrr_http_part_header_field_get_with_value_case(transaction->response_part, "upgrade", "h2c");

		if (field_upgrade_websocket != NULL) {
			if (rrr_http_part_header_field_get_with_value_case(transaction->request_part, "upgrade", "websocket") == NULL) {
				RRR_MSG_0("Unexpected 101 Switching Protocols response with Upgrade: websocket set\n");
				ret = RRR_HTTP_SOFT_ERROR;
				goto out;
			}

			if ((ret = __rrr_http_application_http1_websocket_response_check_headers(transaction->response_part, &receive_data->http1->ws_state)) != 0) {
				goto out;
			}

			int do_websocket = 0;
			if ((ret = receive_data->websocket_callback (
					&do_websocket,
					receive_data->handle,
					transaction,
					read_session->rx_buf_ptr,
					(const struct sockaddr *) &read_session->src_addr,
					read_session->src_addr_len,
					read_session->rx_overshoot_size,
					0,
					receive_data->websocket_callback_arg
			))) {
				goto out;
			}

			if (do_websocket != 1) {
				// Application regrets websocket upgrade, close connection
				goto out;
			}

			upgrade_mode = RRR_HTTP_UPGRADE_MODE_WEBSOCKET;
		}
		else if (field_upgrade_h2c != NULL) {
			if (rrr_http_part_header_field_get_with_value_case(transaction->request_part, "upgrade", "h2c") == NULL) {
				RRR_MSG_0("Unexpected 101 Switching Protocols response with Upgrade: h2c set\n");
				ret = RRR_HTTP_SOFT_ERROR;
				goto out;
			}
#ifdef RRR_WITH_NGHTTP2
			RRR_DBG_3("Upgrade to HTTP2 size is %li overshoot is %li\n", read_session->rx_buf_wpos, read_session->rx_overshoot_size);
			// Pass any extra data received to http2 session for processing there. Overshoot pointer will
			// be set to NULL if http2_session takes control of the pointer.
			ret = rrr_http_application_http2_new_from_upgrade (
					&receive_data->upgraded_application,
					(void **) &read_session->rx_overshoot,
					read_session->rx_overshoot_size,
					transaction
			);

			// Make sure these to variables are always both either 0 or set
			RRR_FREE_IF_NOT_NULL(read_session->rx_overshoot);
			read_session->rx_overshoot_size = 0;

			if (ret != 0) {
				RRR_MSG_0("Failed to initialize HTTP2 application in __rrr_application_http1_response_receive_callback\n");
				ret = RRR_HTTP_HARD_ERROR;
				goto out;
			}

			upgrade_mode = RRR_HTTP_UPGRADE_MODE_HTTP2;
#else
			RRR_BUG("HTTP Client sent a Upgrade: h2c request to which the server responded correctly, but NGHTTP2 support is not built in __rrr_application_http1_response_receive_callback\n");
#endif /* RRR_WITH_NGHTTP2 */
		}
		else {
			RRR_MSG_0("Missing Upgrade: field in HTTP server 101 Switcing Protocols response or value was not h2c or websocket\n");
			ret = RRR_HTTP_SOFT_ERROR;
			goto out;
		}
	}

	if ((ret = receive_data->callback (
			receive_data->handle,
			transaction,
			read_session->rx_buf_ptr,
			(const struct sockaddr *) &read_session->src_addr,
			read_session->src_addr_len,
			read_session->rx_overshoot_size,
			0,
			upgrade_mode,
			receive_data->callback_arg
	)) != 0) {
		goto out;
	}

	receive_data->http1->upgrade_active = upgrade_mode;

	out:
	RRR_FREE_IF_NOT_NULL(orig_http2_settings_tmp);
	return ret;
}

static int __rrr_application_http1_websocket_request_check_version (
		struct rrr_http_part *request_part
) {
	const struct rrr_http_header_field *sec_websocket_version = rrr_http_part_header_field_get(request_part, "sec-websocket-version");
	if (sec_websocket_version == NULL) {
		RRR_DBG_1("Field Sec-WebSocket-Version missing in HTTP request with Connection: Upgrade and Upgrade: websocket headers set\n");
		return 1;
	}

	if (rrr_nullsafe_str_cmpto(sec_websocket_version->value, "13") != 0) {
		RRR_HTTP_UTIL_SET_TMP_NAME_FROM_NULLSAFE(value,sec_websocket_version->value);
		RRR_DBG_1("Received HTTP request with WebSocket upgrade and version '%s' set, but only version '13' is supported\n",
				value);
		return 1;
	}
	return 0;
}

static int __rrr_application_http1_request_upgrade_try_websocket (
		int *do_websocket,
		struct rrr_http_application_http1_receive_data *receive_data,
		struct rrr_read_session *read_session,
		const char *data_to_use
) {
	*do_websocket = 0;

	struct rrr_http_transaction *transaction = receive_data->http1->active_transaction;

	int ret = 0;

	char *accept_base64_tmp = NULL;

	*do_websocket = 1;

	if (transaction->request_part->request_method != RRR_HTTP_METHOD_GET) {
		RRR_HTTP_UTIL_SET_TMP_NAME_FROM_NULLSAFE(method_name,transaction->request_part->request_method_str_nullsafe);
		RRR_DBG_1("Received websocket upgrade request which was not a GET request but '%s'\n", method_name);
		goto out_bad_request;
	}

	if (read_session->rx_overshoot_size) {
		RRR_DBG_1("Extra data received from client after websocket HTTP request\n");
		goto out_bad_request;
	}

	if (__rrr_application_http1_websocket_request_check_version(transaction->request_part) != 0) {
		goto out_bad_request;
	}

	const struct rrr_http_header_field *sec_websocket_key = rrr_http_part_header_field_get(transaction->request_part, "sec-websocket-key");
	if (sec_websocket_key == NULL) {
		RRR_DBG_1("HTTP request with WebSocket upgrade missing field Sec-WebSocket-Key\n");
		goto out_bad_request;
	}

	if (!rrr_nullsafe_str_isset(sec_websocket_key->binary_value_nullsafe)) {
		RRR_BUG("BUG: Binary value was not set for sec-websocket-key header field in __rrr_application_http1_request_receive_try_websocket\n");
	}

	if (rrr_nullsafe_str_len(sec_websocket_key->binary_value_nullsafe) != 16) {
		RRR_DBG_1("Incorrect length for Sec-WebSocket-Key header field in HTTP request with WebSocket upgrade. 16 bytes are required but got %" PRIrrrl "\n",
				rrr_nullsafe_str_len(sec_websocket_key->binary_value_nullsafe));
		goto out_bad_request;
	}

	if ((ret = receive_data->websocket_callback (
			do_websocket,
			receive_data->handle,
			transaction,
			data_to_use,
			(const struct sockaddr *) &read_session->src_addr,
			read_session->src_addr_len,
			read_session->rx_overshoot_size,
			receive_data->unique_id,
			receive_data->websocket_callback_arg
	)) != RRR_HTTP_OK || transaction->response_part->response_code != 0) {
		goto out;
	}

	if (*do_websocket != 1) {
		// Application refuses websocket
		goto out_bad_request;
	}

	if ((ret = rrr_http_part_header_field_push(transaction->response_part, "connection", "upgrade")) != 0) {
		goto out;
	}

	if ((ret = rrr_http_part_header_field_push(transaction->response_part, "upgrade", "websocket")) != 0) {
		goto out;
	}

	RRR_HTTP_UTIL_SET_TMP_NAME_FROM_NULLSAFE(sec_websocket_key_str, sec_websocket_key->value);
	if ((ret = __rrr_http_application_http1_websocket_make_accept_string(&accept_base64_tmp, sec_websocket_key_str)) != 0) {
		RRR_MSG_0("Failed to make accept-string in __rrr_application_http1_request_receive_try_websocket\n");
		goto out;
	}

	if ((ret = rrr_http_part_header_field_push(transaction->response_part, "sec-websocket-accept", accept_base64_tmp)) != 0) {
		goto out;
	}

	transaction->response_part->response_code = RRR_HTTP_RESPONSE_CODE_SWITCHING_PROTOCOLS;

	goto out;
	out_bad_request:
		transaction->response_part->response_code = RRR_HTTP_RESPONSE_CODE_ERROR_BAD_REQUEST;
	out:
		RRR_FREE_IF_NOT_NULL(accept_base64_tmp);
		return ret;
}

static int __rrr_application_http1_request_upgrade_try_http2 (
		int *do_http2,
		struct rrr_http_part *request_part,
		struct rrr_http_part *response_part
) {
	*do_http2 = 0;

	int ret = 0;

	const struct rrr_http_header_field *connection_http2_settings = rrr_http_part_header_field_get_with_value_case(request_part, "connection", "http2-settings");
	const struct rrr_http_header_field *http2_settings = rrr_http_part_header_field_get(request_part, "sec-websocket-version");

	if (connection_http2_settings == NULL) {
		RRR_DBG_1("Value HTTP2-Settings was missing in Connection: header field while upgrade was requested\n");
		goto out_bad_request;
	}

	if (http2_settings == NULL) {
		RRR_DBG_1("Field HTTP2-Settings: was missing in header while upgrade was requested\n");
		goto out_bad_request;
	}

	if (request_part->request_method != RRR_HTTP_METHOD_GET && request_part->request_method != RRR_HTTP_METHOD_OPTIONS) {
		RRR_HTTP_UTIL_SET_TMP_NAME_FROM_NULLSAFE(method_name,request_part->request_method_str_nullsafe);
		RRR_DBG_1("Received HTTP2 upgrade request which was not a GET or OPTION request but '%s'\n", method_name);
		goto out_bad_request;
	}

#ifdef RRR_WITH_NGHTTP2
	if ((ret = rrr_http_part_header_field_push(response_part, "connection", "upgrade")) != 0) {
		goto out;
	}

	if ((ret = rrr_http_part_header_field_push(response_part, "upgrade", "h2c")) != 0) {
		goto out;
	}

	*do_http2 = 1;
#else
	RRR_DBG_3("Upgrade to HTTP2 was requested by client, but this RRR is not built with NGHTTP2 bindings. Proceeding with HTTP/1.1\n");
#endif /* RRR_WITH_NGHTTP2 */

	goto out;
	out_bad_request:
		response_part->response_code = RRR_HTTP_RESPONSE_CODE_ERROR_BAD_REQUEST;
	out:
		return ret;
}

static int __rrr_application_http1_request_upgrade_try (
		enum rrr_http_upgrade_mode *upgrade_mode,
		struct rrr_http_application_http1_receive_data *receive_data,
		struct rrr_read_session *read_session,
		const char *data_to_use
) {
	*upgrade_mode = RRR_HTTP_UPGRADE_MODE_NONE;

	int ret = 0;

	struct rrr_http_transaction *transaction = receive_data->http1->active_transaction;

	const struct rrr_http_header_field *connection = rrr_http_part_header_field_get_with_value_case(transaction->request_part, "connection", "upgrade");
	const struct rrr_http_header_field *upgrade_websocket = rrr_http_part_header_field_get_with_value_case(transaction->request_part, "upgrade", "websocket");
	const struct rrr_http_header_field *upgrade_h2c = rrr_http_part_header_field_get_with_value_case(transaction->request_part, "upgrade", "h2c");

	if (connection == NULL) {
		goto out;
	}

	if (upgrade_websocket != NULL && upgrade_h2c != NULL) {
		goto out_bad_request;
	}

	if (upgrade_websocket != NULL) {
		int do_websocket = 0;
		if ((ret = __rrr_application_http1_request_upgrade_try_websocket (
				&do_websocket,
				receive_data,
				read_session,
				data_to_use
		)) == 0 && do_websocket) {
			*upgrade_mode = RRR_HTTP_UPGRADE_MODE_WEBSOCKET;
		}
	}
	else if (upgrade_h2c != NULL) {
		int do_http2 = 0;
		if ((ret = __rrr_application_http1_request_upgrade_try_http2 (
				&do_http2,
				transaction->request_part,
				transaction->response_part
		)) == 0 && do_http2) {
			*upgrade_mode = RRR_HTTP_UPGRADE_MODE_HTTP2;
		}
	}

	goto out;
	out_bad_request:
		transaction->response_part->response_code = RRR_HTTP_RESPONSE_CODE_ERROR_BAD_REQUEST;
	out:
		return ret;
}

static int __rrr_application_http1_request_receive_callback (
		struct rrr_read_session *read_session,
		void *arg
) {
	struct rrr_http_application_http1_receive_data *receive_data = arg;
	struct rrr_http_transaction *transaction = receive_data->http1->active_transaction;

	int ret = 0;

	char *merged_chunks = NULL;

//	const struct rrr_http_header_field *content_type = rrr_http_part_get_header_field(part, "content-type");

	if (RRR_DEBUGLEVEL_3) {
		rrr_http_part_header_dump(transaction->request_part);
	}

	RRR_DBG_3("HTTP reading complete, data length is %li response length is %li header length is %li\n",
			transaction->request_part->data_length,
			transaction->request_part->headroom_length,
			transaction->request_part->header_length
	);

	if (receive_data->raw_callback != NULL) {
		if ((ret = receive_data->raw_callback (
				read_session->rx_buf_ptr,
				read_session->rx_buf_wpos,
				receive_data->unique_id,
				receive_data->raw_callback_arg
		)) != 0) {
			RRR_MSG_0("Error %i from raw callback in __rrr_application_http1_request_receive_callback\n", ret);
			goto out;
		}
	}

	if ((ret = rrr_http_part_chunks_merge(&merged_chunks, transaction->request_part, read_session->rx_buf_ptr)) != 0) {
		goto out;
	}

	const char *data_to_use = (merged_chunks != NULL ? merged_chunks : read_session->rx_buf_ptr);

	if ((ret = rrr_http_part_multipart_process(transaction->request_part, data_to_use)) != 0) {
		goto out;
	}

	if ((ret = rrr_http_part_post_and_query_fields_extract(transaction->request_part, data_to_use)) != 0) {
		goto out;
	}

	if (RRR_DEBUGLEVEL_3) {
		rrr_http_field_collection_dump (&transaction->request_part->fields);
	}

	enum rrr_http_upgrade_mode upgrade_mode = RRR_HTTP_UPGRADE_MODE_NONE;
	if (receive_data->websocket_callback != NULL && (ret = __rrr_application_http1_request_upgrade_try (
			&upgrade_mode,
			receive_data,
			read_session,
			data_to_use
	)) != 0) {
		goto out;
	}

	if (upgrade_mode != RRR_HTTP_UPGRADE_MODE_NONE) {
		if (transaction->response_part->response_code == RRR_HTTP_RESPONSE_CODE_SWITCHING_PROTOCOLS) {
			RRR_DBG_3("Upgrading HTTP connection to %s\n", RRR_HTTP_UPGRADE_MODE_TO_STR(upgrade_mode));
			receive_data->http1->upgrade_active = upgrade_mode;
		}
		else {
			RRR_DBG_1("Upgrade HTTP connection to %s failed\n", RRR_HTTP_UPGRADE_MODE_TO_STR(upgrade_mode));
		}
	}

	if ((ret = receive_data->callback (
			receive_data->handle,
			transaction,
			data_to_use,
			(const struct sockaddr *) &read_session->src_addr,
			read_session->src_addr_len,
			read_session->rx_overshoot_size,
			receive_data->unique_id,
			upgrade_mode,
			receive_data->callback_arg
	)) != RRR_HTTP_OK) {
		goto out;
	}

	if ((ret = __rrr_http_application_http1_response_send((struct rrr_http_application *) receive_data->http1, receive_data->handle, transaction)) != 0) {
		goto out;
	}

	out:
	RRR_FREE_IF_NOT_NULL(merged_chunks);
	return ret;
}

static int __rrr_application_http1_receive_get_target_size (
		struct rrr_read_session *read_session,
		void *arg
) {
	struct rrr_http_application_http1_receive_data *receive_data = arg;

	int ret = RRR_NET_TRANSPORT_READ_COMPLETE_METHOD_TARGET_LENGTH;

	const char *end = read_session->rx_buf_ptr + read_session->rx_buf_wpos;

	// ASCII validation
	int rnrn_counter = 4;
	for (const unsigned char *pos = (const unsigned char *) read_session->rx_buf_ptr; pos < (const unsigned char *) end; pos++) {
//		printf("pos: %02x\n", *pos);
		if (*pos == '\r' && (rnrn_counter == 4 || rnrn_counter == 2)) {
			--rnrn_counter;
		}
		else if (*pos == '\n' && (rnrn_counter == 3 || rnrn_counter == 1)) {
			if (--rnrn_counter == 0) {
				break; // Header complete
			}
		}
		else {
			rnrn_counter = 4;

			// TODO : Why do this? We should be OK with non-ASCII
			if (*pos > 0x7f) {
				RRR_MSG_0("Received non-ASCII character %02x in HTTP request\n", *pos);
				ret = RRR_READ_SOFT_ERROR;
				goto out;
			}
		}
	}

	if (rnrn_counter != 0) {
		ret = RRR_READ_INCOMPLETE;
		goto out;
	}

	size_t target_size;
	size_t parsed_bytes = 0;

	struct rrr_http_part *part_to_use = NULL;
	enum rrr_http_parse_type parse_type = 0;

	if (receive_data->is_client == 1) {
		part_to_use = receive_data->http1->active_transaction->response_part;
		parse_type = RRR_HTTP_PARSE_RESPONSE;
	}
	else {
		if (read_session->parse_pos == 0) {
			struct rrr_http_transaction *transaction = NULL;

			// HTTP1 only supports one active transaction. Make a new and delete any old one. Method
			// does not matter.
			if ((ret = rrr_http_transaction_new(&transaction, RRR_HTTP_METHOD_GET, 1)) != 0) {
				RRR_MSG_0("Could not create transaction for request in __rrr_application_http1_receive_get_target_size\n");
				goto out;
			}

			__rrr_http_application_http1_transaction_set(receive_data->http1, transaction);
			rrr_http_transaction_decref_if_not_null(transaction);
		}
		part_to_use = receive_data->http1->active_transaction->request_part;
		parse_type = RRR_HTTP_PARSE_REQUEST;
	}

	// There might be more than one chunk in each read cycle, we have to
	// go through all of them in a loop here. The parser will always return
	// after a chunk is found.
	do {
		ret = rrr_http_part_parse (
				part_to_use,
				&target_size,
				&parsed_bytes,
				read_session->rx_buf_ptr,
				read_session->parse_pos,
				end,
				parse_type
		);

		read_session->parse_pos += parsed_bytes;
	} while (parsed_bytes != 0 && ret == RRR_HTTP_PARSE_INCOMPLETE);

	if (target_size > SSIZE_MAX) {
		RRR_MSG_0("Target size %lu exceeds maximum value of %li while parsing HTTP part\n",
				target_size, SSIZE_MAX);
		ret = RRR_NET_TRANSPORT_READ_SOFT_ERROR;
		goto out;
	}

	// Used only for stall timeout
	receive_data->received_bytes = read_session->rx_buf_wpos;

	if (ret == RRR_HTTP_PARSE_OK) {
		read_session->target_size = target_size;
	}
	else if (ret == RRR_HTTP_PARSE_INCOMPLETE) {
		if (part_to_use->data_length_unknown) {
			read_session->read_complete_method = RRR_NET_TRANSPORT_READ_COMPLETE_METHOD_CONN_CLOSE;
			ret = RRR_NET_TRANSPORT_READ_OK;
		}
	}
	else {
		ret = RRR_NET_TRANSPORT_READ_SOFT_ERROR;
	}

	out:
	if (ret != RRR_HTTP_PARSE_INCOMPLETE) {
		read_session->parse_pos = 0;
	}
	return ret;
}

struct rrr_http_application_http1_frame_callback_data {
	rrr_http_unique_id unique_id;
	int (*callback)(RRR_HTTP_APPLICATION_WEBSOCKET_FRAME_CALLBACK_ARGS);
	void *callback_arg;
};

static int __rrr_http_application_http1_websocket_frame_callback (
		RRR_WEBSOCKET_FRAME_CALLBACK_ARGS
) {
	struct rrr_http_application_http1_frame_callback_data *callback_data = arg;

	if (opcode == RRR_WEBSOCKET_OPCODE_BINARY || opcode == RRR_WEBSOCKET_OPCODE_TEXT) {
		return callback_data->callback (
				payload,
				payload_size,
				(opcode == RRR_WEBSOCKET_OPCODE_BINARY ? 1 : 0),
				callback_data->unique_id,
				callback_data->callback_arg
		);
	}

	return RRR_HTTP_OK;
}

static int __rrr_http_application_http1_websocket_get_responses (
		struct rrr_websocket_state *ws_state,
		int (*get_response_callback)(RRR_HTTP_APPLICATION_WEBSOCKET_GET_RESPONSE_CALLBACK_ARGS),
		void *get_response_callback_arg
) {
	int ret = 0;

	void *response_data = NULL;
	ssize_t response_data_len = 0;
	int response_is_binary = 0;

	do {
		RRR_FREE_IF_NOT_NULL(response_data);
		if ((ret = get_response_callback (
				&response_data,
				&response_data_len,
				&response_is_binary,
				get_response_callback_arg
		)) != 0) {
			goto out;
		}
		if (response_data) {
			if ((ret = rrr_websocket_frame_enqueue (
					ws_state,
					(response_is_binary ? RRR_WEBSOCKET_OPCODE_BINARY : RRR_WEBSOCKET_OPCODE_TEXT),
					(char**) &response_data,
					response_data_len
			)) != 0) {
				goto out;
			}
		}
	} while (response_data != NULL);

	out:
	RRR_FREE_IF_NOT_NULL(response_data);
	return ret;
}

static int __rrr_http_application_http1_transport_ctx_websocket_tick (
		struct rrr_http_application_http1 *http1,
		struct rrr_net_transport_handle *handle,
		ssize_t read_max_size,
		rrr_http_unique_id unique_id,
		int ping_interval_s,
		int timeout_s,
		int (*get_response_callback)(RRR_HTTP_APPLICATION_WEBSOCKET_GET_RESPONSE_CALLBACK_ARGS),
		void *get_response_callback_arg,
		int (*frame_callback)(RRR_HTTP_APPLICATION_WEBSOCKET_FRAME_CALLBACK_ARGS),
		void *frame_callback_arg
) {
	int ret = 0;

	struct rrr_http_application_http1_frame_callback_data callback_data = {
			unique_id,
			frame_callback,
			frame_callback_arg
	};

	if (rrr_websocket_check_timeout(&http1->ws_state, timeout_s) != 0) {
		RRR_DBG_2("HTTP websocket session timed out after %i seconds of inactivity\n", timeout_s);
		ret = RRR_READ_EOF;
		goto out;
	}

	if ((ret = rrr_websocket_enqueue_ping_if_needed(&http1->ws_state, ping_interval_s)) != 0) {
		goto out;
	}

	if ((ret = __rrr_http_application_http1_websocket_get_responses (
			&http1->ws_state,
			get_response_callback,
			get_response_callback_arg
	)) != 0) {
		goto out;
	}

	if ((ret = rrr_websocket_transport_ctx_send_frames (
			handle,
			&http1->ws_state
	)) != 0) {
		goto out;
	}

	if ((ret = (rrr_websocket_transport_ctx_read_frames (
			handle,
			&http1->ws_state,
			100,
			4096,
			65535,
			read_max_size,
			__rrr_http_application_http1_websocket_frame_callback,
			&callback_data
	)) & ~(RRR_NET_TRANSPORT_READ_INCOMPLETE)) != 0) {
		goto out;
	}

	out:
	return ret;
}

int __rrr_http_application_http1_tick (
		RRR_HTTP_APPLICATION_TICK_ARGS
) {
	struct rrr_http_application_http1 *http1 = (struct rrr_http_application_http1 *) app;

	int ret = RRR_HTTP_OK;

	*upgraded_app = NULL;

	if (http1->upgrade_active == RRR_HTTP_UPGRADE_MODE_WEBSOCKET) {
		ret = __rrr_http_application_http1_transport_ctx_websocket_tick (
				http1,
				handle,
				read_max_size,
				unique_id,
				10,
				15,
				get_response_callback,
				get_response_callback_arg,
				frame_callback,
				frame_callback_arg
		);
	}
	else if (http1->upgrade_active == RRR_HTTP_UPGRADE_MODE_NONE) {
		struct rrr_http_application_http1_receive_data callback_data = {
				handle,
				http1,
				*received_bytes,
				unique_id,
				is_client,
				NULL,
				websocket_callback,
				websocket_callback_arg,
				callback,
				callback_arg,
				raw_callback,
				raw_callback_arg
		};

		ret = rrr_net_transport_ctx_read_message (
					handle,
					100,
					4096,
					65535,
					read_max_size,
					__rrr_application_http1_receive_get_target_size,
					&callback_data,
					is_client
						? __rrr_application_http1_response_receive_callback
						: __rrr_application_http1_request_receive_callback,
					&callback_data
		);

		*received_bytes = callback_data.received_bytes;
		*upgraded_app = callback_data.upgraded_application;
	}
	else {
		RRR_BUG("__rrr_http_application_http1_tick called while active upgrade was not NONE or WEBSOCKET but %i, maybe caller forgot to switch to HTTP2?\n", http1->upgrade_active);
	}

	return ret;
}

static void __rrr_http_application_http1_alpn_protos_get (
		RRR_HTTP_APPLICATION_ALPN_PROTOS_GET_ARGS
) {
	*target = NULL;
	*length = 0;
}

static void __rrr_http_application_http1_polite_close (
		RRR_HTTP_APPLICATION_POLITE_CLOSE_ARGS
) {
	(void)(app);
	(void)(handle);
	return;
}

static const struct rrr_http_application_constants rrr_http_application_http1_constants = {
	RRR_HTTP_APPLICATION_HTTP1,
	__rrr_http_application_http1_destroy,
	__rrr_http_application_http1_request_send,
	__rrr_http_application_http1_response_send,
	__rrr_http_application_http1_tick,
	__rrr_http_application_http1_alpn_protos_get,
	__rrr_http_application_http1_polite_close
};

int rrr_http_application_http1_new (struct rrr_http_application **target) {
	int ret = 0;

	struct rrr_http_application_http1 *result = NULL;

	if ((result = malloc(sizeof(*result))) == NULL) {
		RRR_MSG_0("Could not allocate memory in __rrr_http_application_http1_new\n");
		ret = 1;
		goto out;
	}

	memset(result, '\0', sizeof(*result));

	result->constants = &rrr_http_application_http1_constants;

	*target = (struct rrr_http_application *) result;

	out:
	return ret;
}
