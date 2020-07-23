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

#ifndef RRR_CMODULE_CHANNEL_H
#define RRR_CMODULE_CHANNEL_H

#include <sys/types.h>

#include "../linked_list.h"

struct rrr_message;
struct rrr_message_addr;
struct rrr_setting_packed;
struct rrr_mmap_channel;
struct rrr_cmodule_deferred_message_collection;

int rrr_cmodule_channel_send_message (
		int *sent_total,
		int *retries,
		struct rrr_mmap_channel *channel,
		struct rrr_cmodule_deferred_message_collection *deferred_queue,
		struct rrr_message *message,
		const struct rrr_message_addr *message_addr,
		unsigned int wait_time_us
);
int rrr_cmodule_channel_receive_messages (
		struct rrr_mmap_channel *channel,
		unsigned int empty_wait_time_us,
		int (*callback)(const void *data, size_t data_size, void *arg),
		void *callback_arg
);
void rrr_cmodule_channel_bubblesort (
		struct rrr_mmap_channel *channel
);

#endif /* RRR_CMODULE_CHANNEL_H */
