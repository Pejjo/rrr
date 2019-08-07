/*

Read Route Record

Copyright (C) 2018-2019 Atle Solbakken atle@goliathdns.no

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

#ifndef RRR_MQTT_COMMON_H
#define RRR_MQTT_COMMON_H

#include <stdio.h>

#include "mqtt_connection.h"

#define RRR_MQTT_DATA_CLIENT_NAME_LENGTH 64
#define RRR_MQTT_SYNCHRONIZED_READ_STEP_MAX_SIZE 4096

struct ip_accept_data;
struct rrr_mqtt_packet_internal;
struct rrr_mqtt_data;

#define RRR_MQTT_TYPE_HANDLER_DEFINITION \
		struct rrr_mqtt_data *data, struct rrr_mqtt_packet_internal *packet

struct rrr_mqtt_type_handler_properties {
	int (*handler)(RRR_MQTT_TYPE_HANDLER_DEFINITION);
};

struct rrr_mqtt_data {
	struct rrr_mqtt_connection_collection connections;
	char client_name[RRR_MQTT_DATA_CLIENT_NAME_LENGTH + 1];
	const struct rrr_mqtt_type_handler_properties *handler_properties;
};

void rrr_mqtt_common_data_destroy (struct rrr_mqtt_data *data);
int rrr_mqtt_common_data_init (struct rrr_mqtt_data *data,
		const char *client_name,
		const struct rrr_mqtt_type_handler_properties *handler_properties
);
int rrr_mqtt_common_data_register_connection (
		struct rrr_mqtt_data *data,
		const struct ip_accept_data *accept_data
);
int rrr_mqtt_common_read_and_parse (struct rrr_mqtt_data *data);

#endif /* RRR_MQTT_COMMON_H */
