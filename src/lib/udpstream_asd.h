/*

Read Route Record

Copyright (C) 2019 Atle Solbakken atle@goliathdns.no

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

#ifndef RRR_UDPSTREAM_ASD_H
#define RRR_UDPSTREAM_ASD_H

#include <pthread.h>
#include "udpstream.h"
#include "buffer.h"
#include "linked_list.h"

#define RRR_UDPSTREAM_ASD_CONNECT_TIMEOUT_MS 5000
#define RRR_UDPSTREAM_ASD_BUFFER_MAX 500
#define RRR_UDPSTREAM_ASD_MESSAGE_ID_MAX 0xffffffff
#define RRR_UDPSTREAM_ASD_RESEND_INTERVAL_MS (RRR_UDPSTREAM_RESEND_INTERVAL_FRAME_MS * 4) // Milliseconds before resending a packet
#define RRR_UDPSTREAM_ASD_RELEASE_QUEUE_MAX 50 // Max unreleased messages awaiting release ACK

// This many delivered messages must follow a message before it is deleted from release queue
#define RRR_UDPSTREAM_ASD_DELIVERY_GRACE_COUNTER (RRR_UDPSTREAM_ASD_RELEASE_QUEUE_MAX/2)

#define RRR_UDPSTREAM_ASD_OK			0
#define RRR_UDPSTREAM_ASD_ERR			RRR_UDPSTREAM_ERR
#define RRR_UDPSTREAM_ASD_NOT_READY		RRR_UDPSTREAM_NOT_READY
#define RRR_UDPSTREAM_ASD_BUFFER_FULL	RRR_UDPSTREAM_BUFFER_FULL

#define RRR_UDPSTREAM_ASD_ACK_FLAGS_MSG			(1<<0)
#define RRR_UDPSTREAM_ASD_ACK_FLAGS_DACK		(1<<1)
#define RRR_UDPSTREAM_ASD_ACK_FLAGS_RACK		(1<<2)
#define RRR_UDPSTREAM_ASD_ACK_FLAGS_CACK		(1<<3)
#define RRR_UDPSTREAM_ASD_ACK_FLAGS_DELIVERED	(1<<15)

// The following three packets resembles functionality of MQTT QoS2, for this
// purpose called "assured single delivery". This type of management of whole
// messages is not performed by the udpstream API and must be implemented by API user.

// Used for assured single delivery messages (with non-zero boundary id) to notify
// about delivery to application. After delivery ACK is received by the sender,
// it must not re-send the message.
// After sending delivery ACK, a client must not release the message before
// release ACK is received. The sender sends release ACK once it receives
// the delivery ACK. Upon receival of release ACK, the client may release
// the message, but it must not yet be deleted as it must reserve the boundary
// ID to ensure there are no duplicates.
//#define RRR_UDPSTREAM_FRAME_TYPE_RELEASE_ACK		06

// After receiving release ACK, a client must delete the message and send
// complete ACK. After the sender receives this, it may also delete the
// message.
//#define RRR_UDPSTREAM_FRAME_TYPE_COMPLETE_ACK		07

struct ip_buffer_entry;

struct rrr_udpstream_asd_queue_entry {
	RRR_LL_NODE(struct rrr_udpstream_asd_queue_entry);
	struct ip_buffer_entry *message;
	uint32_t message_id;
	uint64_t send_time;
	int delivered_grace_counter;
	int ack_status_flags;
};

struct rrr_udpstream_asd_queue {
	RRR_LL_HEAD(struct rrr_udpstream_asd_queue_entry);
};

struct rrr_udpstream_asd_control_queue_entry {
	RRR_LL_NODE(struct rrr_udpstream_asd_control_queue_entry);
	uint32_t message_id;
	uint32_t ack_flags;
};

struct rrr_udpstream_asd_control_queue {
	RRR_LL_HEAD(struct rrr_udpstream_asd_control_queue_entry);
};

struct rrr_udpstream_asd {
	struct rrr_udpstream udpstream;

	struct rrr_udpstream_asd_queue release_queue;
	struct rrr_udpstream_asd_queue send_queue;
	struct rrr_udpstream_asd_control_queue control_send_queue;

	char *remote_host;
	char *remote_port;

	pthread_mutex_t connect_lock;
	int is_connected;
	uint64_t connection_attempt_time;
	uint32_t connect_handle;

	uint32_t client_id;

	pthread_mutex_t message_id_lock;
	uint32_t message_id_pos;

	pthread_mutex_t queue_lock;
};

struct rrr_udpstream_asd_control_msg {
	uint32_t flags;
	uint32_t message_id;
} __attribute((packed));

void rrr_udpstream_asd_destroy (
		struct rrr_udpstream_asd *session
);
int rrr_udpstream_asd_new (
		struct rrr_udpstream_asd **target,
		unsigned int local_port,
		const char *remote_host,
		const char *remote_port,
		uint32_t client_id
);
int rrr_udpstream_asd_queue_message (
		struct rrr_udpstream_asd *session,
		struct ip_buffer_entry **message
);
int rrr_udpstream_asd_deliver_messages (
		struct rrr_udpstream_asd *session,
		int (*receive_callback)(struct ip_buffer_entry *message, void *arg),
		void *receive_callback_arg
);
int rrr_udpstream_asd_buffer_tick (
		int *receive_count,
		int *send_count,
		struct rrr_udpstream_asd *session
);

#endif /* RRR_UDPSTREAM_ASD_H */