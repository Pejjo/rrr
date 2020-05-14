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

#include <sys/types.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <openssl/bio.h>

#define RRR_NET_TRANSPORT_H_ENABLE_INTERNALS

#include "../global.h"
#include "net_transport.h"
#include "net_transport_tls.h"
#include "rrr_socket.h"
#include "rrr_openssl.h"
#include "rrr_strerror.h"
#include "gnu.h"
#include "read.h"
#include "read_constants.h"
#include "ip.h"
#include "ip_accept_data.h"

struct rrr_net_transport_tls_ssl_data {
	SSL_CTX *ctx;
	BIO *web;
	struct rrr_ip_data ip_data;
	struct rrr_sockaddr sockaddr;
	socklen_t socklen;
	int handshake_complete;
};

static void __rrr_net_transport_tls_ssl_data_destroy (struct rrr_net_transport_tls_ssl_data *ssl_data) {
	if (ssl_data != NULL) {
/*		if (ssl_data->out != NULL) {
			BIO_free(ssl_data->out);
		}*/
		if (ssl_data->web != NULL) {
			BIO_free_all(ssl_data->web);
		}
		if (ssl_data->ctx != NULL) {
			SSL_CTX_free(ssl_data->ctx);
		}
		if (ssl_data->ip_data.fd != 0) {
			rrr_ip_close(&ssl_data->ip_data);
		}

		free(ssl_data);
	}
}

static int __rrr_net_transport_tls_ssl_data_close (struct rrr_net_transport_handle *handle) {
	__rrr_net_transport_tls_ssl_data_destroy (handle->submodule_private_ptr);

	return 0;
}

static void __rrr_net_transport_tls_destroy (struct rrr_net_transport *transport) {
	// This will call back into our close() function for each handle
	rrr_net_transport_common_cleanup(transport);

	rrr_openssl_global_unregister_user();

	// Do not free here, upstream does that after destroying lock
}

static void __rrr_net_transport_tls_dump_enabled_ciphers(SSL *ssl) {
	STACK_OF(SSL_CIPHER) *sk = SSL_get1_supported_ciphers(ssl);

	RRR_MSG("Enabled ciphers: ");

	for (int i = 0; i < sk_SSL_CIPHER_num(sk); i++) {
		const SSL_CIPHER *c = sk_SSL_CIPHER_value(sk, i);

		const char *name = SSL_CIPHER_get_name(c);
		if (name == NULL) {
			break;
		}

		RRR_MSG("%s%s", (i == 0 ? "" : ":"), name);
	}

	RRR_MSG("\n");

	sk_SSL_CIPHER_free(sk);
}

static int __rrr_net_transport_tls_verify_always_ok (X509_STORE_CTX *x509, void *arg) {
	(void)(x509);
	(void)(arg);
	return 1;
}

struct rrr_net_transport_tls_ssl_data *__rrr_net_transport_tls_ssl_data_new (void) {
	struct rrr_net_transport_tls_ssl_data *ssl_data = NULL;

	if ((ssl_data = malloc(sizeof(*ssl_data))) == NULL) {
		RRR_MSG_ERR("Could not allocate memory for SSL data in __rrr_net_transport_ssl_data_new \n");
		return NULL;
	}
	memset (ssl_data, '\0', sizeof(*ssl_data));

	return ssl_data;
}

static int __rrr_net_transport_tls_new_ctx (
		SSL_CTX **target,
		const SSL_METHOD *method,
		int flags,
		const char *certificate_file,
		const char *private_key_file
) {
	int ret = 0;

	*target = NULL;

	SSL_CTX *ctx = NULL;

	if (((certificate_file == NULL || *certificate_file == '\0') && (private_key_file != NULL && *private_key_file != '\0')) ||
		((private_key_file == NULL || *private_key_file == '\0') && (certificate_file != NULL && *certificate_file != '\0'))
	) {
		RRR_BUG("BUG: Certificate file and private key file must both be either set or unset in __rrr_net_transport_tls_new_ctx\n");
	}

	if ((ctx = SSL_CTX_new(method)) == NULL) {
		RRR_SSL_ERR("Could not get SSL CTX in __rrr_net_transport_tls_new_ctx ");
		ret = 1;
		goto out;
	}

	// NULL callback causes verification failure to cancel further processing
	SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER, NULL);
	SSL_CTX_set_verify_depth(ctx, 4);

	// Unused flag: SSL_OP_NO_TLSv1_2, we need to support 1.2
	// TODO : Apparently the version restrictions with set_options are deprecated
	SSL_CTX_set_options(ctx, SSL_OP_NO_SSLv2 | SSL_OP_NO_SSLv3 | SSL_OP_NO_TLSv1_1 | SSL_OP_NO_COMPRESSION);

	if (SSL_CTX_set_min_proto_version(ctx, TLS1_2_VERSION) != 1) {
		RRR_SSL_ERR("Could not set minimum protocol version to TLSv1.2");
		ret = 1;
		goto out_destroy;
	}

	// TODO : Add user-configurable cerfificates and paths
	if ((ret = rrr_openssl_load_verify_locations(ctx)) != 0) {
		ret = 1;
		goto out_destroy;
	}

	// Disable verification if required
	if ((flags & RRR_NET_TRANSPORT_F_TLS_NO_CERT_VERIFY) != 0) {
		SSL_CTX_set_cert_verify_callback (ctx, __rrr_net_transport_tls_verify_always_ok, NULL);
	}

	if (certificate_file != NULL && *certificate_file != '\0') {
		if (SSL_CTX_use_certificate_file(ctx, certificate_file, SSL_FILETYPE_PEM) <= 0) {
			RRR_SSL_ERR("Could not set certificate file while starting TLS");
			ret = 1;
			goto out_destroy;
		}
	}

	if (private_key_file != NULL && *private_key_file != '\0') {
		if (SSL_CTX_use_PrivateKey_file(ctx, private_key_file, SSL_FILETYPE_PEM) <= 0 ) {
			RRR_SSL_ERR("Could not set private key file while starting TLS");
			ret = 1;
			goto out_destroy;
		}

		if (SSL_CTX_check_private_key(ctx) != 1) {
			RRR_SSL_ERR("Error encoutered while checking private key while starting TLS");
			ret = 1;
			goto out_destroy;
		}
	}

	*target = ctx;

	goto out;
	out_destroy:
		SSL_CTX_free(ctx);
	out:
		return ret;
}

static int __rrr_net_transport_tls_connect (
		struct rrr_net_transport_handle **handle,
		struct rrr_net_transport *transport,
		unsigned int port,
		const char *host
) {
	struct rrr_net_transport_tls *tls = (struct rrr_net_transport_tls *) transport;
	struct rrr_ip_accept_data *accept_data = NULL;

	*handle = NULL;

	int ret = 0;

	struct rrr_net_transport_tls_ssl_data *ssl_data = NULL;
	struct rrr_net_transport_handle *new_handle = NULL;

	if (rrr_ip_network_connect_tcp_ipv4_or_ipv6(&accept_data, port, host) != 0) {
		RRR_MSG_ERR("Could not create TLS connection to %s:%u\n", host, port);
		ret = 1;
		goto out;
	}

	if ((ssl_data = __rrr_net_transport_tls_ssl_data_new()) == NULL) {
		RRR_MSG_ERR("Could not allocate memory for SSL data in __rrr_net_transport_tls_connect\n");
		ret = 1;
		goto out_destroy_ip;
	}

	ssl_data->ip_data = accept_data->ip_data;

	if ((ret = rrr_net_transport_handle_allocate_and_add_return_locked (
			&new_handle,
			transport,
			RRR_NET_TRANSPORT_SOCKET_MODE_CONNECTION,
			ssl_data,
			0
	)) != 0) {
		RRR_MSG_ERR("Could not get handle in __rrr_net_transport_tls_connect\n");
		ret = 1;
		goto out_destroy_ssl_data;
	}

	if (__rrr_net_transport_tls_new_ctx (
			&ssl_data->ctx,
			tls->ssl_client_method,
			tls->flags,
			tls->certificate_file,
			tls->private_key_file
	) != 0) {
		RRR_SSL_ERR("Could not get SSL CTX in __rrr_net_transport_tls_connect");
		ret = 1;
		goto out_destroy_ip;
	}

	if ((ssl_data->web = BIO_new_ssl(ssl_data->ctx, 1)) == NULL) {
		RRR_SSL_ERR("Could not get BIO in __rrr_net_transport_tls_connect");
		ret = 1;
		goto out_unregister_handle;
	}

	SSL *ssl = NULL;
	BIO_get_ssl(ssl_data->web, &ssl);

	if (SSL_set_fd(ssl, ssl_data->ip_data.fd) != 1) {
		RRR_SSL_ERR("Could not set FD for SSL in __rrr_net_transport_tls_accept\n");
		ret = 1;
		goto out_unregister_handle;
	}

	// Not used for TLSv1.3
	//const char* const PREFERRED_CIPHERS = "HIGH:!aNULL:!kRSA:!PSK:!SRP:!MD5:!RC4";
	//res = SSL_set_cipher_list(ssl, PREFERRED_CIPHERS);

	if (SSL_set_tlsext_host_name(ssl, host) != 1) {
		RRR_SSL_ERR("Could not set TLS hostname");
		ret = 1;
		goto out_unregister_handle;
	}

	if (RRR_DEBUGLEVEL_1) {
		__rrr_net_transport_tls_dump_enabled_ciphers(ssl);
	}

	// Set non-blocking I/O
	BIO_set_nbio(ssl_data->web, 1); // Always returns 1

	retry_handshake:
	if (BIO_do_handshake(ssl_data->web) != 1) {
		if (BIO_should_retry(ssl_data->web)) {
			usleep(1000);
			goto retry_handshake;
		}
		RRR_SSL_ERR("Could not do TLS handshake");
		ret = 1;
		goto out_unregister_handle;
	}
	
	ssl_data->handshake_complete = 1;

	X509 *cert = SSL_get_peer_certificate(ssl);
	if (cert != NULL) {
		X509_free(cert);
	}
	else {
		RRR_MSG_ERR("No certificate received in TLS handshake with %s:%u\n", host, port);
		ret = 1;
		goto out_unregister_handle;
	}

	long verify_result = 0;
	if ((verify_result = SSL_get_verify_result(ssl)) != X509_V_OK) {
		RRR_MSG_ERR("Certificate verification failed for %s:%u with reason %li\n", host, port, verify_result);
		ret = 1;
		goto out_unregister_handle;
	}

	// TODO : Hostname verification

	// Return locked handle
	*handle = new_handle;

	goto out;

	out_unregister_handle:
		// Will also destroy ssl_data (which in turn destroys ip)
		// Will unlock and destroy
		rrr_net_transport_ctx_handle_close(new_handle);
		// Freed when handle is unregistered, don't double-free
		goto out;
	out_destroy_ssl_data:
		__rrr_net_transport_tls_ssl_data_destroy(ssl_data);
		goto out; // ssl_data_destroy calls ip_close
	out_destroy_ip:
		rrr_ip_close(&accept_data->ip_data);
	out:
		RRR_FREE_IF_NOT_NULL(accept_data);
		return ret;
}

static int __rrr_net_transport_tls_bind_and_listen (
		struct rrr_net_transport *transport,
		unsigned int port,
		void (*callback)(struct rrr_net_transport_handle *handle, void *arg),
		void *arg
) {
	struct rrr_net_transport_tls *tls = (struct rrr_net_transport_tls *) transport;
	struct rrr_net_transport_tls_ssl_data *ssl_data = NULL;
	struct rrr_net_transport_handle *new_handle = NULL;

	int ret = 0;

	if (tls->certificate_file == NULL || tls->private_key_file == NULL) {
		RRR_MSG_ERR("Certificate file and/or private key file not set while attempting to start TLS listening server\n");
		ret = 1;
		goto out;
	}

	if ((ssl_data = __rrr_net_transport_tls_ssl_data_new()) == NULL) {
		RRR_MSG_ERR("Could not allocate memory for SSL data in __rrr_net_transport_tls_bind_and_listen\n");
		ret = 1;
		goto out;
	}

	if ((ret = rrr_net_transport_handle_allocate_and_add_return_locked (
			&new_handle,
			transport,
			RRR_NET_TRANSPORT_SOCKET_MODE_LISTEN,
			ssl_data,
			0
	)) != 0) {
		RRR_MSG_ERR("Could not get handle in __rrr_net_transport_tls_bind_and_listen\n");
		ret = 1;
		goto out_free_ssl_data;
	}

	// Do all initialization inside memory fence
	memset(ssl_data, '\0', sizeof(*ssl_data));

	ssl_data->ip_data.port = port;

	if (rrr_ip_network_start_tcp_ipv4_and_ipv6 (&ssl_data->ip_data, 10) != 0) {
		RRR_MSG_ERR("Could not start IP listening in __rrr_net_transport_tls_bind_and_listen\n");
		ret = 1;
		goto out_unregister_handle;
	}

	if (__rrr_net_transport_tls_new_ctx (
			&ssl_data->ctx,
			tls->ssl_server_method,
			tls->flags,
			tls->certificate_file,
			tls->private_key_file
	) != 0) {
		RRR_SSL_ERR("Could not get SSL CTX in __rrr_net_transport_tls_bind_and_listen");
		ret = 1;
		goto out_destroy_ip;
	}

	callback(new_handle, arg);

	pthread_mutex_unlock(&new_handle->lock);

	goto out;
//	out_destroy_ctx:
//		SSL_CTX_free(ssl_data->ctx);
	out_destroy_ip:
		rrr_ip_close(&ssl_data->ip_data);
	out_unregister_handle:
		// Will also destroy ssl_data (which in turn destroys ip)
		// Will unlock and destroy
		rrr_net_transport_ctx_handle_close (new_handle);
		// Freed when handle is unregistered, don't double-free
		ssl_data = NULL;
	out_free_ssl_data:
		RRR_FREE_IF_NOT_NULL(ssl_data);
	out:
		return ret;
}

int __rrr_net_transport_tls_accept (
		struct rrr_net_transport_handle *listen_handle,
		void (*callback)(struct rrr_net_transport_handle *handle, const struct sockaddr *sockaddr, socklen_t socklen, void *arg),
		void *arg
) {
	struct rrr_ip_accept_data *accept_data = NULL;
	struct rrr_net_transport_tls_ssl_data *new_ssl_data = NULL;
	struct rrr_net_transport_handle *new_handle = NULL;

	int ret = 0;

	struct rrr_net_transport_tls_ssl_data *listen_ssl_data = listen_handle->submodule_private_ptr;

	if ((ret = rrr_ip_accept(&accept_data, &listen_ssl_data->ip_data, "net_transport_tls", 0)) != 0) {
		RRR_MSG_ERR("Error while accepting connection in TLS server\n");
		ret = 1;
		goto out;
	}

	if (accept_data == NULL) {
		goto out;
	}

	if ((new_ssl_data = __rrr_net_transport_tls_ssl_data_new()) == NULL) {
		RRR_MSG_ERR("Could not allocate memory for SSL data in __rrr_net_transport_tls_accept\n");
		ret = 1;
		goto out_destroy_ip;
	}

	// Run this before populating SSL data to provide memory fence
	if ((ret = rrr_net_transport_handle_allocate_and_add_return_locked(
			&new_handle,
			listen_handle->transport,
			RRR_NET_TRANSPORT_SOCKET_MODE_CONNECTION,
			new_ssl_data,
			0
	)) != 0) {
		RRR_MSG_ERR("Could not get handle in __rrr_net_transport_tls_accept\n");
		ret = 1;
		goto out_destroy_ssl_data;
	}

	// Do all initialization inside memory fence
	memset (new_ssl_data, '\0', sizeof(*new_ssl_data));

	new_ssl_data->sockaddr = accept_data->addr;
	new_ssl_data->socklen = accept_data->len;
	new_ssl_data->ip_data = accept_data->ip_data;
	new_ssl_data = NULL;

	if ((new_ssl_data->web = BIO_new_ssl(new_ssl_data->ctx, 0)) == NULL) {
		RRR_SSL_ERR("Could not allocate BIO in __rrr_net_transport_tls_accept\n");
		ret = 1;
		goto out_unregister_handle;
	}

	SSL *ssl;
	BIO_get_ssl(new_ssl_data->web, &ssl);

	if (SSL_set_fd(ssl, new_ssl_data->ip_data.fd) != 1) {
		RRR_SSL_ERR("Could not set FD for SSL in __rrr_net_transport_tls_accept\n");
		ret = 1;
		goto out_unregister_handle;
	}

	BIO_set_nbio(new_ssl_data->web, 1);

	// SSL handshake is done in read function

	callback(new_handle, (struct sockaddr *) &accept_data->addr, accept_data->len, arg);

	pthread_mutex_unlock(&new_handle->lock);

	goto out;
	out_unregister_handle:
		// Will also destroy ssl_data (which in turn destroys ip)
		// Will unlock and destroy
		rrr_net_transport_ctx_handle_close (new_handle);
		goto out;
	out_destroy_ssl_data:
		__rrr_net_transport_tls_ssl_data_destroy(new_ssl_data);
		goto out; // ssl_data_cleanup will call rrr_ip_close
	out_destroy_ip:
		rrr_ip_close(&accept_data->ip_data);
	out:
		RRR_FREE_IF_NOT_NULL(accept_data);
		return ret;
}

static int __rrr_net_transport_tls_read_poll(int read_flags, void *private_arg) {
	(void)(private_arg);
	(void)(read_flags);
	return RRR_READ_OK;
}

static struct rrr_read_session *__rrr_net_transport_tls_read_get_read_session_with_overshoot(void *private_arg) {
	struct rrr_net_transport_read_callback_data *callback_data = private_arg;

	return rrr_read_session_collection_get_session_with_overshoot (
			&callback_data->handle->read_sessions
	);
}

static struct rrr_read_session *__rrr_net_transport_tls_read_get_read_session(void *private_arg) {
	struct rrr_net_transport_read_callback_data *callback_data = private_arg;
	struct rrr_net_transport_tls_ssl_data *ssl_data = callback_data->handle->submodule_private_ptr;

	return rrr_read_session_collection_maintain_and_find_or_create (
			&callback_data->handle->read_sessions,
			(struct sockaddr *) &ssl_data->sockaddr,
			ssl_data->socklen
	);
}

static void __rrr_net_transport_tls_read_remove_read_session(struct rrr_read_session *read_session, void *private_arg) {
	struct rrr_net_transport_read_callback_data *callback_data = private_arg;

	rrr_read_session_collection_remove_session(&callback_data->handle->read_sessions, read_session);
}

static int __rrr_net_transport_tls_read_get_target_size(struct rrr_read_session *read_session, void *private_arg) {
	struct rrr_net_transport_read_callback_data *callback_data = private_arg;
	return callback_data->get_target_size(read_session, callback_data->get_target_size_arg);
}

static int __rrr_net_transport_tls_read_complete_callback(struct rrr_read_session *read_session, void *private_arg) {
	struct rrr_net_transport_read_callback_data *callback_data = private_arg;
	return callback_data->complete_callback(read_session, callback_data->complete_callback_arg);
}

static int __rrr_net_transport_tls_read_read (
		char *buf,
		ssize_t *read_bytes,
		ssize_t read_step_max_size,
		void *private_arg
) {
	struct rrr_net_transport_read_callback_data *callback_data = private_arg;
	struct rrr_net_transport_tls_ssl_data *ssl_data = callback_data->handle->submodule_private_ptr;

	int ret = RRR_READ_OK;

	ssize_t result = BIO_read(ssl_data->web, buf, read_step_max_size);
	if (result <= 0) {
		if (BIO_should_retry(ssl_data->web) == 0) {
			int reason = BIO_get_retry_reason(ssl_data->web);
			RRR_SSL_ERR("Error while reading from TLS connection");
			RRR_MSG_ERR("Reason: %s\n", rrr_strerror(reason));
			// Possible close of connection
			goto out;
		}
		else {
			// Retry later
			return RRR_READ_INCOMPLETE;
		}
	}
	else if (ERR_peek_error() != 0) {
		RRR_SSL_ERR("Error while reading in __rrr_net_transport_tls_read_read");
		return RRR_READ_HARD_ERROR;
	}

	out:
	ERR_clear_error();
	*read_bytes = (result >= 0 ? result : 0);
	return ret;
}

static int __rrr_net_transport_tls_read_message (
	struct rrr_net_transport_handle *handle,
	int read_attempts,
	ssize_t read_step_initial,
	ssize_t read_step_max_size,
	int (*get_target_size)(struct rrr_read_session *read_callback_data, void *arg),
	void *get_target_size_arg,
	int (*complete_callback)(struct rrr_read_session *read_callback_data, void *arg),
	void *complete_callback_arg
) {
	int ret = 0;

	struct rrr_net_transport_tls_ssl_data *ssl_data = handle->submodule_private_ptr;

	// Try only once to avoid blocking on bad clients
	while (ssl_data->handshake_complete == 0) {
		if (BIO_do_handshake(ssl_data->web) != 1) {
			if (BIO_should_retry(ssl_data->web) != 1) {
				RRR_SSL_ERR("Could not do handshake with client in TLS server\n");
				ret = 1;
				goto out;
			}
			else if (--read_attempts == 0) {
				ret = RRR_NET_TRANSPORT_READ_INCOMPLETE;
				goto out;
			}
		}
		else {
			ssl_data->handshake_complete = 1;
		}
	}

	struct rrr_net_transport_read_callback_data read_callback_data = {
		handle,
		get_target_size,
		get_target_size_arg,
		complete_callback,
		complete_callback_arg
	};

	while (--read_attempts > 0) {
		ret = rrr_read_message_using_callbacks (
				read_step_initial,
				read_step_max_size,
				0,
				__rrr_net_transport_tls_read_get_target_size,
				__rrr_net_transport_tls_read_complete_callback,
				__rrr_net_transport_tls_read_poll,
				__rrr_net_transport_tls_read_read,
				__rrr_net_transport_tls_read_get_read_session_with_overshoot,
				__rrr_net_transport_tls_read_get_read_session,
				__rrr_net_transport_tls_read_remove_read_session,
				NULL,
				&read_callback_data
		);

		if (ret == RRR_NET_TRANSPORT_READ_INCOMPLETE) {
			continue;
		}
		else if (ret == RRR_NET_TRANSPORT_READ_OK) {
			ret = 0;
			break;
		}
		else {
			RRR_MSG_ERR("Error while reading from remote\n");
			ret = 1;
			goto out;
		}
	}

	out:
	return ret;
}

static int __rrr_net_transport_tls_send (
	ssize_t *sent_bytes,
	struct rrr_net_transport_handle *handle,
	const void *data,
	ssize_t size
) {
	struct rrr_net_transport_tls_ssl_data *ssl_data = handle->submodule_private_ptr;

	*sent_bytes = 0;

	if (BIO_write(ssl_data->web, data, size) <= 0) {
		if (BIO_should_retry(ssl_data->web)) {
			return 0;
		}
		RRR_MSG_ERR("Write failure in __rrr_net_transport_tls_send\n");
		return 1;
	}
	else {
		*sent_bytes = size;
	}

	return 0;
}

static const struct rrr_net_transport_methods tls_methods = {
	__rrr_net_transport_tls_destroy,
	__rrr_net_transport_tls_connect,
	__rrr_net_transport_tls_bind_and_listen,
	__rrr_net_transport_tls_accept,
	__rrr_net_transport_tls_ssl_data_close,
	__rrr_net_transport_tls_read_message,
	__rrr_net_transport_tls_send
};

int rrr_net_transport_tls_new (
		struct rrr_net_transport_tls **target,
		int flags,
		const char *certificate_file,
		const char *private_key_file
) {
	struct rrr_net_transport_tls *result = NULL;

	*target = NULL;

	int ret = 0;

	int flags_checked = 0;
	if ((flags & RRR_NET_TRANSPORT_F_TLS_NO_CERT_VERIFY) != 0) {
		flags_checked |= RRR_NET_TRANSPORT_F_TLS_NO_CERT_VERIFY;
		flags &= ~(RRR_NET_TRANSPORT_F_TLS_NO_CERT_VERIFY);
	}

	if (flags != 0) {
		RRR_BUG("BUG: Unknown flags %i given to rrr_net_transport_tls_new\n", flags);
	}

	if ((result = malloc(sizeof(*result))) == NULL) {
		RRR_MSG_ERR("Could not allocate memory in rrr_net_transport_tls_new\n");
		ret = 1;
		goto out;
	}

	rrr_openssl_global_register_user();

	memset(result, '\0', sizeof(*result));

	if (certificate_file != NULL && *certificate_file != '\0') {
		if ((result->certificate_file = strdup(certificate_file)) == NULL) {
			RRR_MSG_ERR("Could not allocate memory for certificate file in rrr_net_transport_tls_new\n");
			ret = 1;
			goto out_free;
		}
	}

	if (private_key_file != NULL && *private_key_file != '\0') {
		if ((result->private_key_file = strdup(private_key_file)) == NULL) {
			RRR_MSG_ERR("Could not allocate memory for private key file in rrr_net_transport_tls_new\n");
			ret = 1;
			goto out_free;
		}
	}

	result->methods = &tls_methods;
	result->ssl_client_method = TLS_client_method();
	result->ssl_server_method = TLS_server_method();
	result->flags = flags_checked;

	*target = result;

	goto out;
	out_free:
		RRR_FREE_IF_NOT_NULL(result->certificate_file);
		RRR_FREE_IF_NOT_NULL(result->private_key_file);
		free(result);
	out:
		return ret;
}
