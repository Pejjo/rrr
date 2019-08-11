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

#include <poll.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>

#include "ip.h"
#include "buffer.h"
#include "vl_time.h"
#include "../global.h"
#include "mqtt_common.h"
#include "mqtt_connection.h"
#include "mqtt_packet.h"
#include "mqtt_parse.h"

int __rrr_mqtt_connection_collection_read_lock (struct rrr_mqtt_connection_collection *connections) {
	int ret = 0;

	pthread_mutex_lock(&connections->lock);
	if (connections->invalid != 0) {
		pthread_mutex_unlock(&connections->lock);
		ret = 1;
		goto out;
	}
	pthread_mutex_unlock(&connections->lock);

	int pass = 0;
	while (pass != 1) {
		pthread_mutex_lock(&connections->lock);
		if (connections->writers_waiting == 0 && connections->write_locked == 0) {
			connections->readers++;
			pass = 1;
		}
		pthread_mutex_unlock(&connections->lock);
	}

	out:
	return ret;
}

int __rrr_mqtt_connection_collection_read_unlock (struct rrr_mqtt_connection_collection *connections) {
	int ret = 0;

	pthread_mutex_lock(&connections->lock);
	if (connections->invalid != 0) {
		pthread_mutex_unlock(&connections->lock);
		ret = 1;
		goto out;
	}
	if (connections->readers == 0) {
		VL_BUG("__rrr_mqtt_connection_collection_read_unlock double-called, no read lock held\n");
	}
	connections->readers--;
	pthread_mutex_unlock(&connections->lock);

	out:
	return ret;
}

int __rrr_mqtt_connection_collection_write_lock (struct rrr_mqtt_connection_collection *connections) {
	int ret = 0;

	pthread_mutex_lock(&connections->lock);
	if (connections->invalid != 0) {
		pthread_mutex_unlock(&connections->lock);
		ret = 1;
		goto out;
	}

	/* This blocks new readers */
	connections->writers_waiting++;

	pthread_mutex_unlock(&connections->lock);

	int pass = 0;
	while (pass != 1) {
		pthread_mutex_lock(&connections->lock);
		if (connections->readers == 0 && connections->write_locked == 0) {
			connections->write_locked = 1;
			connections->writers_waiting--;
			pass = 1;
		}
		pthread_mutex_unlock(&connections->lock);
	}

	out:
	return ret;
}

int __rrr_mqtt_connection_collection_write_unlock (struct rrr_mqtt_connection_collection *connections) {
	int ret = 0;

	pthread_mutex_lock(&connections->lock);
	if (connections->invalid != 0) {
		pthread_mutex_unlock(&connections->lock);
		ret = 1;
		goto out;
	}
	if (connections->write_locked != 1) {
		VL_BUG("__rrr_mqtt_connection_collection_write_unlock double-called, no write lock held\n");
	}
	connections->write_locked = 0;
	pthread_mutex_unlock(&connections->lock);

	out:
	return ret;
}

/* Reader which converts to write lock has priority over other writers */
int __rrr_mqtt_connection_collection_read_to_write_lock (struct rrr_mqtt_connection_collection *connections) {
	int ret = 0;

	pthread_mutex_lock(&connections->lock);
	if (connections->invalid != 0) {
		pthread_mutex_unlock(&connections->lock);
		ret = 1;
		goto out;
	}

	if (connections->readers == 0) {
		VL_BUG("__rrr_mqtt_connection_collection_read_write_to_lock called with no read lock held\n");
	}
	if (connections->write_locked != 0) {
		VL_BUG("write_locked was not 0 in __rrr_mqtt_connection_collection_read_write_to_lock\n");
	}

	/* This blocks new readers */
	connections->writers_waiting++;

	pthread_mutex_unlock(&connections->lock);

	int pass = 0;
	while (pass != 1) {
		pthread_mutex_lock(&connections->lock);
		if (connections->readers == 1) {
			connections->write_locked = 1;
			connections->readers--;
			connections->writers_waiting--;
			pass = 1;
		}
		pthread_mutex_unlock(&connections->lock);
	}

	out:
	return ret;
}

int __rrr_mqtt_connection_collection_write_to_read_lock (struct rrr_mqtt_connection_collection *connections) {
	int ret = 0;

	pthread_mutex_lock(&connections->lock);
	if (connections->invalid != 0) {
		pthread_mutex_unlock(&connections->lock);
		ret = 1;
		goto out;
	}

	if (connections->readers != 0) {
		VL_BUG("__rrr_mqtt_connection_collection_read_write_to_lock readers was not zero\n");
	}
	if (connections->write_locked != 1) {
		VL_BUG("write_locked was not 1 in __rrr_mqtt_connection_collection_write_to_read_lock\n");
	}

	connections->readers++;
	connections->write_locked = 0;

	pthread_mutex_unlock(&connections->lock);

	out:
	return ret;
}

int rrr_mqtt_connection_send_disconnect_and_close (struct rrr_mqtt_connection *connection) {
	pthread_mutex_lock(&connection->lock);
	if (connection->state == RRR_MQTT_CONNECTION_STATE_CLOSED) {
		VL_BUG("State of connection was already CLOSED in __rrr_mqtt_connection_destroy\n");
	}
	if (connection->ip_data.fd == 0) {
		VL_BUG("FD was zero in __rrr_mqtt_connection_destroy\n");
	}

	// TODO : Send close packet

	printf ("mqtt connection close connection fd %i\n", connection->ip_data.fd);

	ip_close(&connection->ip_data);
	connection->state = RRR_MQTT_CONNECTION_STATE_CLOSED;

	pthread_mutex_unlock(&connection->lock);

	return 0;
}

static void __rrr_mqtt_connection_reset_sessions (struct rrr_mqtt_connection *connection) {
	RRR_FREE_IF_NOT_NULL(connection->read_session.rx_buf);
	rrr_mqtt_parse_session_destroy(&connection->parse_session);
	connection->read_complete = 0;
	connection->parse_complete = 0;
}

static void __rrr_mqtt_connection_destroy (struct rrr_mqtt_connection *connection) {
	if (connection == NULL) {
		VL_BUG("NULL pointer in __rrr_mqtt_connection_destroy\n");
	}

	pthread_mutex_lock(&connection->lock);
	if (connection->state != RRR_MQTT_CONNECTION_STATE_CLOSED) {
		if (connection->ip_data.fd == 0) {
			VL_BUG("Connection was not closed but FD was zero in __rrr_mqtt_connection_destroy\n");
		}
		pthread_mutex_unlock(&connection->lock);
		if (rrr_mqtt_connection_send_disconnect_and_close(connection) != 0) {
			VL_MSG_ERR("Warning: Error while sending disconnect packet while destroying connection\n");
		}
	}
	else {
		pthread_mutex_unlock(&connection->lock);
	}

	fifo_buffer_invalidate(&connection->receive_queue.buffer);
	fifo_buffer_invalidate(&connection->send_queue.buffer);

	__rrr_mqtt_connection_reset_sessions (connection);

	if (connection->client_id != NULL) {
		free(connection->client_id);
	}

	pthread_mutex_destroy (&connection->lock);

	free(connection);
}

static int __rrr_mqtt_connection_new (
		struct rrr_mqtt_connection **connection,
		const struct ip_data *ip_data,
		const struct sockaddr *remote_addr
) {
	int ret = 0;

	*connection = NULL;
	struct rrr_mqtt_connection *res = NULL;

	res = malloc(sizeof(*res));
	if (res == NULL) {
		VL_MSG_ERR("Could not allocate memory in rrr_mqtt_connection_new\n");
		ret = 1;
		goto out;
	}

	memset (res, '\0', sizeof(*res));

	if ((ret = pthread_mutex_init (&res->lock, 0)) != 0) {
		VL_MSG_ERR("Could not initialize mutex in __rrr_mqtt_connection_new\n");
		goto out;
	}

	ret |= fifo_buffer_init_custom_free(&res->receive_queue.buffer,		rrr_mqtt_p_decref);
	ret |= fifo_buffer_init_custom_free(&res->send_queue.buffer,		rrr_mqtt_p_decref);

	if (ret != 0) {
		VL_MSG_ERR("Could not initialize buffers in __rrr_mqtt_connection_new\n");
		goto out;
	}

	res->ip_data = *ip_data;
	res->connect_time = res->last_seen_time = time_get_64();

	switch (remote_addr->sa_family) {
		case AF_INET: {
			res->type = RRR_MQTT_CONNECTION_TYPE_IPV4;
			res->remote_in = *((const struct sockaddr_in *) remote_addr);
			inet_ntop(AF_INET, &res->remote_in.sin_addr, res->ip, sizeof(res->ip));
			break;
		}
		case AF_INET6: {
			res->type = RRR_MQTT_CONNECTION_TYPE_IPV6;
			res->remote_in6 = *((const struct sockaddr_in6 *) remote_addr);
			inet_ntop(AF_INET6, &res->remote_in6.sin6_addr, res->ip, sizeof(res->ip));
			break;
		}
		default: {
			VL_BUG("Received non INET/INET6 sockaddr struct in __rrr_mqtt_connection_new\n");
		}
	}

	out:
	if (ret == 0) {
		*connection = res;
	}
	else if (res != NULL) {
		__rrr_mqtt_connection_destroy(res);
	}

	return ret;
}

void rrr_mqtt_connection_collection_destroy (struct rrr_mqtt_connection_collection *connections) {
	if (connections == NULL) {
		return;
	}

	pthread_mutex_lock (&connections->lock);
	if (connections->readers != 0 || connections->write_locked != 0 || connections->writers_waiting != 0) {
		VL_BUG("rrr_mqtt_connection_collection_destroy called while users were active\n");
	}
	pthread_mutex_unlock (&connections->lock);

	struct rrr_mqtt_connection *cur = connections->first;
	while (cur) {
		struct rrr_mqtt_connection *next = cur->next;
		__rrr_mqtt_connection_destroy (cur);
		cur = next;
	}

	connections->first = NULL;
	connections->invalid = 1;

	pthread_mutex_destroy (&connections->lock);
}

int rrr_mqtt_connection_collection_init (struct rrr_mqtt_connection_collection *connections) {
	int ret = 0;

	memset (connections, '\0', sizeof(*connections));

	connections->invalid = 1;
	connections->writers_waiting = 0;
	connections->readers = 0;
	connections->write_locked = 0;

	if ((ret = pthread_mutex_init (&connections->lock, 0)) != 0) {
		VL_MSG_ERR("Could not initialize mutex in __rrr_mqtt_connection_collection_new\n");
		goto out;
	}

	out:
	if (ret != 0) {
		rrr_mqtt_connection_collection_destroy(connections);
	}
	else {
		connections->invalid = 0;
	}

	return ret;
}

int rrr_mqtt_connection_collection_new_connection (
		struct rrr_mqtt_connection **connection,
		struct rrr_mqtt_connection_collection *connections,
		const struct ip_data *ip_data,
		const struct sockaddr *remote_addr
) {
	int ret = 0;
	struct rrr_mqtt_connection *res = NULL;

	*connection = NULL;

	if (connections->invalid == 1) {
		VL_BUG("rrr_mqtt_connection_collection_new_connection called with invalid set to 1\n");
	}

	if (ip_data->fd < 1) {
		VL_BUG("FD was < 1 in rrr_mqtt_connection_collection_new_connection\n");
	}

	if ((ret = __rrr_mqtt_connection_new(&res, ip_data, remote_addr)) != 0) {
		VL_MSG_ERR("Could not create new connection in rrr_mqtt_connection_collection_new_connection\n");
		goto out_nolock;
	}

	if ((ret = __rrr_mqtt_connection_collection_write_lock(connections)) != 0) {
		VL_MSG_ERR("Lock error in rrr_mqtt_connection_collection_new_connection\n");
		goto out_nolock;
	}

	res->next = connections->first;
	connections->first = res;

	if ((ret = __rrr_mqtt_connection_collection_write_unlock(connections)) != 0) {
		VL_MSG_ERR("Lock error in rrr_mqtt_connection_collection_new_connection\n");
		goto out_nolock;
	}

	*connection = res;

	out_nolock:
	return ret;
}

int rrr_mqtt_connection_collection_iterate_reenter_read_to_write (
		struct rrr_mqtt_connection_collection *connections,
		int (*callback)(struct rrr_mqtt_connection *connection, void *callback_arg),
		void *callback_arg
) {
	int ret = 0;
	int callback_ret = 0;

	if ((ret = __rrr_mqtt_connection_collection_read_to_write_lock(connections)) != 0) {
		VL_MSG_ERR("Lock error in rrr_mqtt_connection_collection_iterate_reenter_read_to_write\n");
		goto out;
	}

	struct rrr_mqtt_connection *cur = connections->first;
	while (cur) {
		int ret_tmp = callback(cur, callback_arg);
		if (ret_tmp != RRR_MQTT_CONNECTION_OK) {
			if ((ret_tmp & RRR_MQTT_CONNECTION_DESTROY_CONNECTION) != 0) {
				VL_BUG("Destroy connection flag not allowed in rrr_mqtt_connection_collection_iterate_reenter_read_to_write\n");
			}
			if ((ret_tmp & RRR_MQTT_CONNECTION_INTERNAL_ERROR) != 0) {
				VL_MSG_ERR("Internal error returned from callback in rrr_mqtt_connection_collection_iterate\n");
				callback_ret |= ret_tmp;
				break;
			}
			if ((ret_tmp & RRR_MQTT_CONNECTION_ITERATE_STOP) != 0) {
				callback_ret |= ret_tmp;
				break;
			}

			VL_MSG_ERR("Soft error returned from callback in rrr_mqtt_connection_collection_iterate\n");
		}

		cur = cur->next;
	}

	if ((ret = __rrr_mqtt_connection_collection_write_to_read_lock(connections)) != 0) {
		VL_MSG_ERR("Lock error in rrr_mqtt_connection_collection_iterate_reenter_read_to_write\n");
		goto out;
	}

	out:
	return (ret | callback_ret);
}

int rrr_mqtt_connection_collection_iterate (
	struct rrr_mqtt_connection_collection *connections,
	int (*callback)(struct rrr_mqtt_connection *connection, void *callback_arg),
	void *callback_arg
) {
	int ret = 0;
	int callback_ret = 0;

	if ((ret = __rrr_mqtt_connection_collection_read_lock(connections)) != 0) {
		VL_MSG_ERR("Lock error in rrr_mqtt_connection_collection_iterate\n");
		goto out;
	}

	struct rrr_mqtt_connection *cur = connections->first;
	struct rrr_mqtt_connection *prev = NULL;
	while (cur) {
		int ret_tmp = callback(cur, callback_arg);
		if (ret_tmp != RRR_MQTT_CONNECTION_OK) {
			VL_MSG_ERR("Error returned from callback in rrr_mqtt_connection_collection_iterate\n");

			if ((ret_tmp & RRR_MQTT_CONNECTION_DESTROY_CONNECTION) != 0) {
				VL_MSG_ERR("Destroying connection in rrr_mqtt_connection_collection_iterate\n");

				if ((ret = __rrr_mqtt_connection_collection_read_to_write_lock(connections)) != 0) {
					VL_MSG_ERR("Lock error in rrr_mqtt_connection_collection_iterate\n");
					goto out;
				}

				struct rrr_mqtt_connection *next = cur->next;

				__rrr_mqtt_connection_destroy(cur);

				if (prev != NULL) {
					prev->next = next;
				}
				else {
					connections->first = next;
				}

				// Will not be processed again, it is set to prev->next at the end of the while loop
				if (prev != NULL) {
					cur = prev;
				}
				else {
					cur = next;
				}

				if ((ret = __rrr_mqtt_connection_collection_write_to_read_lock(connections)) != 0) {
					VL_MSG_ERR("Lock error in rrr_mqtt_connection_collection_iterate\n");
					goto out;
				}

				callback_ret |= RRR_MQTT_CONNECTION_SOFT_ERROR;
			}
			if ((ret_tmp & RRR_MQTT_CONNECTION_INTERNAL_ERROR) != 0) {
				VL_MSG_ERR("Internal error returned from callback in rrr_mqtt_connection_collection_iterate\n");
				callback_ret |= ret_tmp;
				break;
			}
			if ((ret_tmp & RRR_MQTT_CONNECTION_ITERATE_STOP) != 0) {
				callback_ret |= ret_tmp;
				break;
			}

			VL_MSG_ERR("Soft error returned from callback in rrr_mqtt_connection_collection_iterate\n");
		}

		/* If the current connection was last in the list and then destroyed, cur will be NULL */
		if (cur != NULL) {
			prev = cur;
			cur = cur->next;
		}
	}

	if ((ret = __rrr_mqtt_connection_collection_read_unlock(connections)) != 0) {
		VL_MSG_ERR("Lock error in rrr_mqtt_connection_collection_iterate\n");
		goto out;
	}

	out:
	return (ret | callback_ret);
}

int rrr_mqtt_connection_read (
		struct rrr_mqtt_connection *connection,
		int read_step_max_size
) {
	int ret = RRR_MQTT_CONNECTION_OK;

	/* There can be multiple read threads, make sure we do not block */
	if (pthread_mutex_trylock(&connection->lock) != 0) {
		ret = RRR_MQTT_CONNECTION_BUSY;
		goto out_nolock;
	}

	struct rrr_mqtt_connection_read_session *read_session = &connection->read_session;

	if (connection->read_complete == 1) {
		if (read_session->rx_buf_wpos != read_session->target_size) {
			VL_BUG("packet complete was 1 but read size was not target size in rrr_mqtt_connection_read\n");
		}
		ret = RRR_MQTT_CONNECTION_BUSY;
		goto out_unlock;
	}

	if (read_session->rx_buf_wpos > read_session->target_size) {
		VL_MSG_ERR("Invalid message: Actual size of message exceeds stated size in rrr_mqtt_connection_read\n");
		ret = RRR_MQTT_CONNECTION_INTERNAL_ERROR;
		goto out_unlock;
	}

	struct pollfd pollfd = { connection->ip_data.fd, POLLIN, 0 };
	ssize_t bytes = 0;
	ssize_t items = 0;
	int bytes_int = 0;

	poll_retry:

	items = poll(&pollfd, 1, 0);
	if (items == -1) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			ret = RRR_MQTT_CONNECTION_BUSY;
			goto out_unlock;
		}
		else if (errno == EINTR) {
			goto poll_retry;
		}
		VL_MSG_ERR("Poll error in rrr_mqtt_connection_read\n");
		ret = RRR_MQTT_CONNECTION_SOFT_ERROR | RRR_MQTT_CONNECTION_DESTROY_CONNECTION;
		goto out_unlock;
	}
	else if ((pollfd.revents & (POLLERR|POLLNVAL)) != 0) {
		VL_MSG_ERR("Poll error in rrr_mqtt_connection_read\n");
		ret = RRR_MQTT_CONNECTION_SOFT_ERROR | RRR_MQTT_CONNECTION_DESTROY_CONNECTION;
		goto out_unlock;
	}
	else if (items == 0) {
		ret = RRR_MQTT_CONNECTION_BUSY;
		goto out_unlock;
	}

	if (ioctl (connection->ip_data.fd, FIONREAD, &bytes_int) != 0) {
		VL_MSG_ERR("Error from ioctl in rrr_mqtt_connection_read: %s\n", strerror(errno));
		ret = RRR_MQTT_CONNECTION_SOFT_ERROR | RRR_MQTT_CONNECTION_DESTROY_CONNECTION;
		goto out_unlock;
	}

	bytes = bytes_int;

	/* Check for new read session */
	if (read_session->rx_buf == NULL) {
		if (bytes < 2) {
			VL_MSG_ERR("Received less than 2 bytes in first packet on connection\n");
			ret = RRR_MQTT_CONNECTION_SOFT_ERROR | RRR_MQTT_CONNECTION_DESTROY_CONNECTION;
			goto out_unlock;
		}
		read_session->rx_buf = malloc(bytes > read_step_max_size ? bytes : read_step_max_size);
		if (read_session->rx_buf == NULL) {
			VL_MSG_ERR("Could not allocate memory in rrr_mqtt_connection_read\n");
			ret = RRR_MQTT_CONNECTION_INTERNAL_ERROR;
			goto out_unlock;
		}
		read_session->rx_buf_size = read_step_max_size;
		read_session->rx_buf_wpos = 0;
		read_session->step_size_limit = read_step_max_size;

		/* This number will change after the fixed header is parsed */
		read_session->target_size = read_step_max_size;
	}

	/* Check for expansion of buffer */
	if (bytes + read_session->rx_buf_wpos > read_session->rx_buf_size) {
		ssize_t new_size = read_session->rx_buf_size + (bytes > read_step_max_size ? bytes : read_step_max_size);
		char *new_buf = realloc(read_session->rx_buf, new_size);
		if (new_buf == NULL) {
			VL_MSG_ERR("Could not re-allocate memory in rrr_mqtt_connection_read\n");
			ret = RRR_MQTT_CONNECTION_INTERNAL_ERROR;
			goto out_unlock;
		}
		read_session->rx_buf = new_buf;
		read_session->rx_buf_size = new_size;
	}

	/* Make sure we do not read past the current message */
	int to_read_bytes = (read_session->target_size < read_session->rx_buf_size
			? read_session->target_size - read_session->rx_buf_wpos
			: read_session->rx_buf_size - read_session->rx_buf_wpos
	);

	if (to_read_bytes < 0) {
		VL_BUG("to_read_bytes was < 0 in rrr_mqtt_connection_read\n");
	}

	if (connection->read_complete == 1 && to_read_bytes != 0) {
		VL_BUG("packet_complete was 1 but to_read_bytes was not zero\n");
	}

	/*
	 * When a message is completely received, we do not read any more data
	 * until somebody else has reset the receive buffer
	 */
	if (to_read_bytes == 0) {
		connection->read_complete = 1;
		ret = RRR_MQTT_CONNECTION_BUSY;
		goto out_unlock;
	}

	/* Read */
	read_retry:
	bytes = read (
			connection->ip_data.fd,
			read_session->rx_buf + read_session->rx_buf_wpos,
			to_read_bytes
	);

	if (bytes == -1) {
		if (errno == EINTR) {
			goto read_retry;
		}
		VL_MSG_ERR("Error from read in rrr_mqtt_connection_read: %s\n", strerror(errno));
		ret = RRR_MQTT_CONNECTION_INTERNAL_ERROR;
		goto out_unlock;
	}

	if (bytes == 0) {
		VL_MSG_ERR("Bytes was 0 after read in rrr_mqtt_connection_read, despite polling first\n");
		ret = RRR_MQTT_CONNECTION_SOFT_ERROR | RRR_MQTT_CONNECTION_DESTROY_CONNECTION;
		goto out_unlock;
	}

	read_session->rx_buf_wpos += bytes;
	read_session->step_size_limit -= bytes;

	if (read_session->rx_buf_wpos > read_session->target_size) {
		VL_BUG("rx_buf_wpos was > target_size in rrr_mqtt_connection_read\n");
	}

	if (read_session->rx_buf_wpos == read_session->target_size) {
		connection->read_complete = 1;
	}

	if (read_session->step_size_limit < 0) {
		ret = RRR_MQTT_CONNECTION_STEP_LIMIT;
		read_session->step_size_limit = read_step_max_size;
	}

	out_unlock:
	pthread_mutex_unlock(&connection->lock);

	out_nolock:
	return ret;
}

int rrr_mqtt_connection_parse (
		struct rrr_mqtt_connection *connection
) {
	int ret = 0;

	/* There can be multiple parse threads, make sure we do not block */
	if (pthread_mutex_trylock(&connection->lock) != 0) {
		ret = RRR_MQTT_CONNECTION_BUSY;
		goto out_nolock;
	}

	if (connection->read_session.rx_buf != NULL) {
		if (connection->parse_session.buf == NULL) {
			rrr_mqtt_parse_session_init (
					&connection->parse_session,
					connection->read_session.rx_buf,
					connection->read_session.rx_buf_wpos
			);
		}

		ret = rrr_mqtt_packet_parse (&connection->parse_session);
		if (RRR_MQTT_PARSE_IS_ERR(&connection->parse_session)) {
			/* Error which was the remote's fault, close connection */
			ret = RRR_MQTT_CONNECTION_SOFT_ERROR|RRR_MQTT_CONNECTION_DESTROY_CONNECTION;
			goto out_unlock;
		}
		if (RRR_MQTT_PARSE_FIXED_HEADER_IS_DONE(&connection->parse_session)) {
			connection->read_session.target_size = connection->parse_session.target_size;
			if (connection->read_session.rx_buf_wpos == connection->read_session.target_size) {
				connection->read_complete = 1;
			}
			else if (connection->read_session.rx_buf_wpos > connection->read_session.target_size) {
				VL_MSG_ERR("Invalid message: Actual size of message exceeds stated size in rrr_mqtt_connection_parse\n");
				ret = RRR_MQTT_CONNECTION_SOFT_ERROR;
				goto out_unlock;
			}
		}
		if (RRR_MQTT_PARSE_IS_COMPLETE(&connection->parse_session)) {
			connection->parse_complete = 1;
		}
	}

	out_unlock:
	pthread_mutex_unlock(&connection->lock);

	out_nolock:
	return ret;
}

int rrr_mqtt_connection_check_finalize (
		struct rrr_mqtt_connection *connection
) {
	int ret = 0;

	/* There can be multiple parse threads, make sure we do not block */
	if (pthread_mutex_trylock(&connection->lock) != 0) {
		ret = RRR_MQTT_CONNECTION_BUSY;
		goto out_nolock;
	}

	if (connection->read_complete == 1) {
		if (connection->parse_complete != 1) {
			VL_MSG_ERR("Reading is done for a packet but parsing did not complete. Closing connection.\n");
			ret = RRR_MQTT_CONNECTION_DESTROY_CONNECTION|RRR_MQTT_CONNECTION_SOFT_ERROR;
			goto out_unlock;
		}

		struct rrr_mqtt_p_packet *packet;
		ret = rrr_mqtt_packet_parse_finalize(&packet, &connection->parse_session);
		if (rrr_mqtt_p_get_refcount(packet) != 1) {
			VL_BUG("Refcount was not 1 while finalizing mqtt packet and adding to receive buffer\n");
		}

		fifo_buffer_write(&connection->receive_queue.buffer, (char*) packet, RRR_MQTT_P_GET_SIZE(packet));

		__rrr_mqtt_connection_reset_sessions(connection);
	}

	out_unlock:
		pthread_mutex_unlock(&connection->lock);

	out_nolock:
		return ret;
}

int rrr_mqtt_connection_read_and_parse (
		struct rrr_mqtt_connection *connection,
		void *arg
) {
	int ret = RRR_MQTT_CONNECTION_OK;

	if (arg != NULL) {
		VL_BUG("rrr_mqtt_connection_read_and_parse received non-null custom argument\n");
	}

	// Do not block while reading a large message, read only 4K each time. This also
	// goes for threaded reading, the connection lock must be released often to allow
	// for other iterators to check stuff.
	ret = rrr_mqtt_connection_read (connection, RRR_MQTT_SYNCHRONIZED_READ_STEP_MAX_SIZE);

	if ((ret & RRR_MQTT_CONNECTION_INTERNAL_ERROR) != 0) {
		VL_MSG_ERR("Internal error while reading data from mqtt client. Closing down server.\n");
		ret =  RRR_MQTT_CONNECTION_INTERNAL_ERROR;
		goto out;
	}

	if ((ret & (RRR_MQTT_CONNECTION_DESTROY_CONNECTION|RRR_MQTT_CONNECTION_SOFT_ERROR)) != 0) {
		VL_MSG_ERR("Error while reading data from mqtt client, destroying connection.\n");
		ret = RRR_MQTT_CONNECTION_DESTROY_CONNECTION|RRR_MQTT_CONNECTION_SOFT_ERROR;
		goto out;
	}

	ret = rrr_mqtt_connection_parse (connection);

	if ((ret & RRR_MQTT_CONNECTION_INTERNAL_ERROR) != 0) {
		VL_MSG_ERR("Internal error while parsing data from mqtt client. Closing down server.\n");
		ret =  RRR_MQTT_CONNECTION_INTERNAL_ERROR;
		goto out;
	}

	if ((ret & (RRR_MQTT_CONNECTION_DESTROY_CONNECTION|RRR_MQTT_CONNECTION_SOFT_ERROR)) != 0) {
		VL_MSG_ERR("Error while parsing data from mqtt client, destroying connection.\n");
		ret = RRR_MQTT_CONNECTION_DESTROY_CONNECTION|RRR_MQTT_CONNECTION_SOFT_ERROR;
		goto out;
	}

	ret = rrr_mqtt_connection_check_finalize (connection);

	if ((ret & RRR_MQTT_CONNECTION_INTERNAL_ERROR) != 0) {
		VL_MSG_ERR("Internal error while finalizing data from mqtt client. Closing down server.\n");
		ret =  RRR_MQTT_CONNECTION_INTERNAL_ERROR;
		goto out;
	}

	if ((ret & (RRR_MQTT_CONNECTION_DESTROY_CONNECTION|RRR_MQTT_CONNECTION_SOFT_ERROR)) != 0) {
		VL_MSG_ERR("Error while finalizing data from mqtt client, destroying connection.\n");
		ret = RRR_MQTT_CONNECTION_DESTROY_CONNECTION|RRR_MQTT_CONNECTION_SOFT_ERROR;
		goto out;
	}

	out:
	return ret;
}

struct handle_packets_callback {
	struct rrr_mqtt_data *data;
	struct rrr_mqtt_connection *connection;
	int handler_return;
};

static int __rrr_mqtt_connection_handle_packets_callback (struct fifo_callback_args *callback_data, char *data, unsigned long int size) {
	int ret = FIFO_OK;

	(void)(size);

	struct handle_packets_callback *handle_packets_data = callback_data->private_data;
	struct rrr_mqtt_data *mqtt_data = handle_packets_data->data;
	struct rrr_mqtt_connection *connection = handle_packets_data->connection;
	struct rrr_mqtt_p_packet *packet = (struct rrr_mqtt_p_packet *) data;

	if (mqtt_data->handler_properties[RRR_MQTT_P_GET_TYPE(packet)].handler == NULL) {
		VL_MSG_ERR("No handler specified for packet type %i\n", RRR_MQTT_P_GET_TYPE(packet));
		return FIFO_CALLBACK_ERR|FIFO_SEARCH_STOP;
	}

	int tmp = mqtt_data->handler_properties[RRR_MQTT_P_GET_TYPE(packet)].handler(mqtt_data, connection, packet);

	printf ("handler return: %i\n", handle_packets_data->handler_return);

	if (tmp != RRR_MQTT_CONNECTION_OK) {
		tmp = tmp & ~(RRR_MQTT_CONNECTION_SOFT_ERROR|RRR_MQTT_CONNECTION_DESTROY_CONNECTION);
		if (tmp == 0) {
			handle_packets_data->handler_return = RRR_MQTT_CONNECTION_SOFT_ERROR|RRR_MQTT_CONNECTION_DESTROY_CONNECTION;
		}
		else {
			handle_packets_data->handler_return = RRR_MQTT_CONNECTION_INTERNAL_ERROR;
		}

		ret = FIFO_CALLBACK_ERR|FIFO_SEARCH_STOP;
	}

	return ret;
}

int rrr_mqtt_connection_handle_packets (
		struct rrr_mqtt_connection *connection,
		void *arg
) {
	int ret = RRR_MQTT_CONNECTION_OK;

	/* There can be multiple parse threads, make sure we do not block */
	if (pthread_mutex_trylock(&connection->lock) != 0) {
		ret = RRR_MQTT_CONNECTION_BUSY;
		goto out_nolock;
	}

	struct rrr_mqtt_data *data = arg;

	struct handle_packets_callback callback_data = {
			data, connection, RRR_MQTT_CONNECTION_OK
	};

	struct fifo_callback_args fifo_callback_data = {
			NULL, &callback_data, 0
	};

	ret = fifo_read_clear_forward (
			&connection->receive_queue.buffer,
			NULL,
			__rrr_mqtt_connection_handle_packets_callback,
			&fifo_callback_data,
			0
	);

	if (ret == FIFO_GLOBAL_ERR) {
		VL_MSG_ERR("Buffer error while handling mqtt packets from client, must exit.\n");
		ret = RRR_MQTT_CONNECTION_INTERNAL_ERROR;
		goto out;
	}
	else if (ret != FIFO_OK) {
		ret = callback_data.handler_return;
		ret = ret & ~(RRR_MQTT_CONNECTION_SOFT_ERROR|RRR_MQTT_CONNECTION_DESTROY_CONNECTION);
		if (ret != 0) {
			VL_MSG_ERR("Internal error while handling packets from mqtt client, must exit. Return is %i.\n", ret);
			ret = RRR_MQTT_CONNECTION_INTERNAL_ERROR;
			goto out;
		}
		VL_MSG_ERR("Soft error while handling packets from mqtt client, destroying connection.\n");
		ret |= (RRR_MQTT_CONNECTION_SOFT_ERROR|RRR_MQTT_CONNECTION_DESTROY_CONNECTION);
	}

	out:
	pthread_mutex_unlock(&connection->lock);

	out_nolock:
	return ret;
}

int rrr_mqtt_connection_collection_read_parse_handle (
		struct rrr_mqtt_connection_collection *connections,
		struct rrr_mqtt_data *mqtt_data
) {
	int ret = 0;

	ret = rrr_mqtt_connection_collection_iterate(connections, rrr_mqtt_connection_read_and_parse, NULL);

	if ((ret & (RRR_MQTT_CONNECTION_SOFT_ERROR|RRR_MQTT_CONNECTION_DESTROY_CONNECTION)) != 0) {
		VL_MSG_ERR("Soft error in rrr_mqtt_connection_collection_read_parse_handle  (one or more connections had to be closed)\n");
		ret = 0;
	}
	if ((ret & RRR_MQTT_CONNECTION_INTERNAL_ERROR) != 0) {
		VL_MSG_ERR("Internal error received in rrr_mqtt_connection_collection_read_parse_handle \n");
		ret = 1;
	}

	ret = rrr_mqtt_connection_collection_iterate(connections, rrr_mqtt_connection_handle_packets, mqtt_data);

	if ((ret & (RRR_MQTT_CONNECTION_SOFT_ERROR|RRR_MQTT_CONNECTION_DESTROY_CONNECTION)) != 0) {
		VL_MSG_ERR("Soft error in rrr_mqtt_connection_collection_read_parse_handle  while handling packets (one or more connections had to be closed)\n");
		ret = 0;
	}
	if ((ret & RRR_MQTT_CONNECTION_INTERNAL_ERROR) != 0) {
		VL_MSG_ERR("Internal error received in rrr_mqtt_connection_collection_read_parse_handle  while handling packets\n");
		ret = 1;
	}

	return ret;
}
