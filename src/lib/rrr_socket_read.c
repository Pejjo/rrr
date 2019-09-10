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

#include <stdint.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "../global.h"
#include "linked_list.h"
#include "rrr_socket.h"
#include "rrr_socket_read.h"
#include "rrr_socket_msg.h"
#include "vl_time.h"

static struct rrr_socket_read_session *__rrr_socket_read_session_new (
		struct sockaddr *src_addr,
		socklen_t src_addr_len
) {
	struct rrr_socket_read_session *read_session = malloc(sizeof(*read_session));
	if (read_session == NULL) {
		VL_MSG_ERR("Could not allocate memory in __rrr_socket_read_session_new\n");
		return NULL;
	}
	memset(read_session, '\0', sizeof(*read_session));

	read_session->last_read_time = time_get_64();
	read_session->src_addr = *src_addr;
	read_session->src_addr_len = src_addr_len;

	return read_session;
}

static int __rrr_socket_read_session_destroy (
		struct rrr_socket_read_session *read_session
) {
	RRR_FREE_IF_NOT_NULL(read_session->rx_buf_ptr);
	RRR_FREE_IF_NOT_NULL(read_session->rx_overshoot);
	return 0;
}

void rrr_socket_read_session_collection_init (
		struct rrr_socket_read_session_collection *collection
) {
	memset(collection, '\0', sizeof(*collection));
}

void rrr_socket_read_session_collection_destroy (
		struct rrr_socket_read_session_collection *collection
) {
	RRR_LINKED_LIST_DESTROY(collection,struct rrr_socket_read_session,__rrr_socket_read_session_destroy(node));
}

static struct rrr_socket_read_session *__rrr_socket_read_session_collection_maintain_and_find_or_create (
		struct rrr_socket_read_session_collection *collection,
		struct sockaddr *src_addr,
		socklen_t src_addr_len
) {
	struct rrr_socket_read_session *res = NULL;

	uint64_t time_now = time_get_64();
	uint64_t time_limit = time_now - RRR_SOCKET_READ_TIMEOUT * 1000 * 1000;

	RRR_LINKED_LIST_ITERATE_BEGIN(collection,struct rrr_socket_read_session);
		if (node->last_read_time < time_limit) {
			RRR_LINKED_LIST_SET_DESTROY();
		}
		else if (memcmp(src_addr, &node->src_addr, sizeof(*src_addr)) == 0) {
			if (res != NULL) {
				VL_BUG("Two equal src_addr in rrr_socket_read_session_collection_maintain_and_find\n");
			}
			res = node;
		}
	RRR_LINKED_LIST_ITERATE_END_CHECK_DESTROY(collection,__rrr_socket_read_session_destroy(node));

	if (res == NULL) {
		res = __rrr_socket_read_session_new(src_addr, src_addr_len);
		if (res == NULL) {
			VL_MSG_ERR("Could not allocate memory for read session in rrr_socket_read_message\n");
			goto out;
		}

		RRR_LINKED_LIST_PUSH(collection,res);
	}

	out:
	return res;
}

int rrr_socket_read_message (
		struct rrr_socket_read_session_collection *read_session_collection,
		int fd,
		ssize_t read_step_initial,
		ssize_t read_step_max_size,
		int (*get_target_size)(struct rrr_socket_read_session *read_session, void *arg),
		void *get_target_size_arg,
		int (*complete_callback)(struct rrr_socket_read_session *read_session, void *arg),
		void *complete_callback_arg
) {
	int ret = RRR_SOCKET_OK;

	char buf[read_step_max_size];
	struct rrr_socket_read_session *read_session = NULL;

	struct pollfd pollfd = { fd, POLLIN, 0 };
	ssize_t bytes = 0;
	ssize_t items = 0;
	int bytes_int = 0;

	poll_retry:
	items = poll(&pollfd, 1, 0);
	if (items == -1) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			ret = RRR_SOCKET_READ_INCOMPLETE;
			goto out;
		}
		else if (errno == EINTR) {
			goto poll_retry;
		}
		VL_MSG_ERR("Poll error in rrr_socket_read_message\n");
		ret = RRR_SOCKET_SOFT_ERROR;
		goto out;
	}
	else if ((pollfd.revents & (POLLERR|POLLNVAL)) != 0) {
		VL_MSG_ERR("Poll error in rrr_socket_read_message\n");
		ret = RRR_SOCKET_SOFT_ERROR;
		goto out;
	}
	else if (items == 0) {
		ret = RRR_SOCKET_READ_INCOMPLETE;
		goto out;
	}

	if (ioctl (fd, FIONREAD, &bytes_int) != 0) {
		VL_MSG_ERR("Error from ioctl in rrr_socket_read_message: %s\n", strerror(errno));
		ret = RRR_SOCKET_SOFT_ERROR;
		goto out;
	}

	bytes = bytes_int;

	if (bytes == 0) {
		goto out;
	}

	struct sockaddr src_addr;
	socklen_t src_addr_len = sizeof(src_addr);

	/* Read */
	read_retry:
	bytes = recvfrom (
			fd,
			buf,
			read_step_max_size,
			0,
			&src_addr,
			&src_addr_len
	);

	if (bytes == -1) {
		if (errno == EINTR) {
			goto read_retry;
		}
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			goto out;
		}
		VL_MSG_ERR("Error from read in rrr_socket_read_message: %s\n", strerror(errno));
		ret = RRR_SOCKET_SOFT_ERROR;
		goto out;
	}

	if (bytes == 0) {
		VL_MSG_ERR("Bytes was 0 after read in rrr_socket_read_message, despite polling first\n");
		ret = RRR_SOCKET_SOFT_ERROR;
		goto out;
	}

	read_session = __rrr_socket_read_session_collection_maintain_and_find_or_create (
			read_session_collection,
			&src_addr,
			src_addr_len
	);

	if (read_session == NULL) {
		ret = RRR_SOCKET_HARD_ERROR;
		goto out;
	}

	/* Check for new read session */
	if (read_session->rx_buf_ptr == NULL) {
		if (read_session->rx_overshoot != NULL) {
			read_session->rx_buf_ptr = read_session->rx_overshoot;
			read_session->rx_buf_size = read_session->rx_overshoot_size;
			read_session->rx_buf_wpos = read_session->rx_overshoot_size;

			read_session->rx_overshoot = NULL;
			read_session->rx_overshoot_size = 0;
		}
		else {
			read_session->rx_buf_ptr = malloc(bytes > read_step_max_size ? bytes : read_step_max_size);
			if (read_session->rx_buf_ptr == NULL) {
				VL_MSG_ERR("Could not allocate memory in rrr_socket_read_message\n");
				ret = RRR_SOCKET_HARD_ERROR;
				goto out;
			}
			read_session->rx_buf_size = read_step_max_size;
			read_session->rx_buf_wpos = 0;
		}

		/* This number will change after the fixed header is parsed. The first round we can
		 * only read 2 bytes to make sure we don't read in many packets at a time. */
		read_session->target_size = 0;
	}

	if (read_session->read_complete != 0) {
		VL_BUG("Read complete was non-zero in rrr_socket_read_message, read session must be cleared prior to reading more data\n");
	}

	/* Check for expansion of buffer */
	if (bytes + read_session->rx_buf_wpos > read_session->rx_buf_size) {
		ssize_t new_size = read_session->rx_buf_size + (bytes > read_step_max_size ? bytes : read_step_max_size);
		char *new_buf = realloc(read_session->rx_buf_ptr, new_size);
		if (new_buf == NULL) {
			VL_MSG_ERR("Could not re-allocate memory in rrr_socket_read_message\n");
			ret = RRR_SOCKET_HARD_ERROR;
			goto out;
		}
		read_session->rx_buf_ptr = new_buf;
		read_session->rx_buf_size = new_size;
	}

	memcpy (read_session->rx_buf_ptr + read_session->rx_buf_wpos, buf, bytes);
	read_session->rx_buf_wpos += bytes;
	read_session->last_read_time = time_get_64();

	if (get_target_size == NULL) {
		read_session->target_size = read_step_initial;
	}
	else if (read_session->target_size == 0) {
		// In the first read, we take a sneak peak at the first bytes to find a length field
		// if it is present. If there is not target size function, the target size becomes
		// the initial bytes parameter (set at the top of the function).
		if (read_session->rx_buf_wpos < read_session->target_size) {
			ret = RRR_SOCKET_READ_INCOMPLETE;
			goto out;
		}

		if ((ret = get_target_size(read_session, get_target_size_arg)) != RRR_SOCKET_OK) {
			goto out;
		}

		if (read_session->target_size == 0) {
			VL_BUG("target_size was still zero after get_target_size in rrr_socket_read_message\n");
		}
	}

	if (read_session->rx_buf_wpos > read_session->target_size) {
			if (read_session->rx_overshoot != NULL) {
				VL_BUG("overshoot was not NULL in rrr_socket_read_message\n");
			}

			read_session->rx_overshoot_size = read_session->rx_buf_wpos - read_session->target_size;
			read_session->rx_buf_wpos -= read_session->rx_overshoot_size;

			read_session->rx_overshoot = malloc(read_session->rx_overshoot_size);
			if (read_session->rx_overshoot == NULL) {
				VL_MSG_ERR("Could not allocate memory for overshoot in rrr_socket_read_message\n");
				ret = RRR_SOCKET_HARD_ERROR;
				goto out;
			}

			memcpy(read_session->rx_overshoot, read_session->rx_buf_ptr + read_session->rx_buf_wpos, read_session->rx_overshoot_size);
	}

	if (read_session->rx_buf_wpos == read_session->target_size && read_session->target_size > 0) {
		read_session->read_complete = 1;
		if (complete_callback != NULL) {
			ret = complete_callback (read_session, complete_callback_arg);
			if (ret != 0) {
				VL_MSG_ERR("Error from callback in rrr_socket_read_message\n");
				goto out;
			}
			RRR_FREE_IF_NOT_NULL(read_session->rx_buf_ptr);
			read_session->read_complete = 0;
		}
	}

	out:
	if (ret != RRR_SOCKET_OK && ret != RRR_SOCKET_READ_INCOMPLETE && read_session != NULL) {
		RRR_LINKED_LIST_REMOVE_NODE(
				read_session_collection,
				struct rrr_socket_read_session,
				read_session,
				__rrr_socket_read_session_destroy(node)
		);
	}
	return ret;
}
