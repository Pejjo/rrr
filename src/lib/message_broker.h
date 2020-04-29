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

#ifndef RRR_MESSAGE_BROKER_H
#define RRR_MESSAGE_BROKER_H

#include <pthread.h>
#include <sys/socket.h>

#include "linked_list.h"
#include "buffer.h"

#define RRR_MESSAGE_BROKER_OK		0
#define RRR_MESSAGE_BROKER_POST		RRR_MESSAGE_BROKER_OK
#define RRR_MESSAGE_BROKER_ERR		1
#define RRR_MESSAGE_BROKER_DROP		2
#define RRR_MESSAGE_BROKER_AGAIN	3

struct rrr_message_broker_costumer {
	RRR_LL_NODE(struct rrr_message_broker_costumer);
	char *name;
	pthread_mutex_t lock;
	struct rrr_fifo_buffer queue;
};

struct rrr_message_broker {
	RRR_LL_HEAD(struct rrr_message_broker_costumer);
	pthread_mutex_t lock;
};

struct rrr_ip_buffer_entry;

// Do not cast this to struct rrr_message_broker_costumer except from
// inside this framework, memory might become freed up at any time
typedef void rrr_message_broker_costumer_handle;

void rrr_message_broker_cleanup (
		struct rrr_message_broker *broker
);
int rrr_message_broker_init (
		struct rrr_message_broker *broker
);
void rrr_message_broker_costumer_unregister (
		struct rrr_message_broker *broker,
		rrr_message_broker_costumer_handle *handle
);
int rrr_message_broker_costumer_register (
		rrr_message_broker_costumer_handle **result,
		struct rrr_message_broker *broker,
		const char *name_unique
);
int rrr_message_broker_write_entry (
		struct rrr_message_broker *broker,
		rrr_message_broker_costumer_handle *handle,
		ssize_t data_length,
		const struct sockaddr *addr,
		socklen_t socklen,
		int protocol,
		int (*callback)(struct rrr_ip_buffer_entry *new_entry, void *arg),
		void *callback_arg
);

#endif /* RRR_MESSAGE_BROKER_H */
