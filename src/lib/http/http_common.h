/*

Read Route Record

Copyright (C) 2019-2020 Atle Solbakken atle@goliathdns.no

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

#ifndef RRR_HTTP_COMMON_H
#define RRR_HTTP_COMMON_H

#include "../read_constants.h"

#define RRR_HTTP_OK				RRR_READ_OK
#define RRR_HTTP_HARD_ERROR		RRR_READ_HARD_ERROR
#define RRR_HTTP_SOFT_ERROR		RRR_READ_SOFT_ERROR

enum rrr_http_transport {
	RRR_HTTP_TRANSPORT_ANY,
	RRR_HTTP_TRANSPORT_HTTP,
	RRR_HTTP_TRANSPORT_HTTPS
};

enum rrr_http_method {
	RRR_HTTP_METHOD_GET,
	RRR_HTTP_METHOD_POST_MULTIPART_FORM_DATA,
	RRR_HTTP_METHOD_POST_URLENCODED,
	RRR_HTTP_METHOD_POST_URLENCODED_NO_QUOTING,
	RRR_HTTP_METHOD_POST_APPLICATON_OCTET_STREAM,
	RRR_HTTP_METHOD_POST_TEXT_PLAIN
};

extern const char *rrr_http_transport_str_any;
extern const char *rrr_http_transport_str_http;
extern const char *rrr_http_transport_str_https;

#define RRR_HTTP_TRANSPORT_TO_STR(transport)												\
	(transport == RRR_HTTP_TRANSPORT_ANY ? rrr_http_transport_str_any :						\
	(transport == RRR_HTTP_TRANSPORT_HTTP ? rrr_http_transport_str_http :					\
	(transport == RRR_HTTP_TRANSPORT_HTTPS ? rrr_http_transport_str_https : ("unknown")		\
	)))

#endif /* RRR_HTTP_COMMON_H */
