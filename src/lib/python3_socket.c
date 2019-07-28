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

#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <poll.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <Python.h>

#include "vl_time.h"
#include "python3_common.h"
#include "python3_module_common.h"
#include "python3_vl_message.h"
#include "python3_module.h"
#include "python3_socket.h"
#include "python3_setting.h"
#include "rrr_socket.h"
#include "rrr_socket_msg.h"
#include "messages.h"
#include "settings.h"
#include "../../config.h"

// TODO : Many of these functions may be used by other modules like ipclient/ipserver. Migrate functionality to rrr_socket.

#define RRR_PYTHON3_IN_FLIGHT_ACK_INTERVAL 10
#define RRR_PYTHON3_MAX_IN_FLIGHT 50

struct rrr_python3_socket_data {
	PyObject_HEAD
	int socket_fd;
	int connected_fd;
	char *filename;
	int send_stats;
	uint64_t time_start;
	pthread_mutex_t stats_lock;
	pthread_mutex_t send_lock;
};

static void __rrr_python3_socket_dealloc_internals (PyObject *self) {
	struct rrr_python3_socket_data *socket_data = (struct rrr_python3_socket_data *) self;

	if (socket_data->connected_fd > 0) {
		rrr_socket_close(socket_data->connected_fd);
		socket_data->connected_fd = 0;
	}
	if (socket_data->socket_fd > 0) {
		rrr_socket_close(socket_data->socket_fd);
		socket_data->socket_fd = 0;
	}
	if (socket_data->filename != NULL) {
		unlink (socket_data->filename);
		free(socket_data->filename);
		socket_data->filename = NULL;
	}
}

static void rrr_python3_socket_f_dealloc (PyObject *self) {
	__rrr_python3_socket_dealloc_internals(self);
	PyObject_Del(self);
}

static PyObject *rrr_python3_socket_f_new (PyTypeObject *type, PyObject *args, PyObject *kwds) {
	PyObject *self = PyType_GenericNew(type, args, kwds);
	if (self == NULL) {
		return NULL;
	}

	return self;
}

static int rrr_python3_socket_f_init (PyObject *self, PyObject *args, PyObject *kwds) {
	__rrr_python3_socket_dealloc_internals(self);
	(void)(args);
	(void)(kwds);
	return 0;
}

static PyObject *rrr_python3_socket_f_accept (PyObject *self, PyObject *args) {
	struct rrr_python3_socket_data *socket_data = (struct rrr_python3_socket_data *) self;
	int ret = 0;

	(void)(args);

	if (socket_data->socket_fd == 0) {
		VL_MSG_ERR("Cannot accept connection in python3 socket, no listen socket created\n");
		ret = 1;
		goto out;
	}

	struct sockaddr addr;
	socklen_t len = sizeof(addr);
	int new_fd = rrr_socket_accept(socket_data->socket_fd, &addr, &len, "rrr_python3_socket_f_accept");

	if (new_fd == -1) {
		VL_MSG_ERR("Could not accept connection on python3 socket: %s\n", strerror(errno));
		ret = 1;
		goto out;
	}

	socket_data->connected_fd = new_fd;

	out:
	if (ret != 0) {
		Py_RETURN_FALSE;
	}
	Py_RETURN_TRUE;
}

static PyObject *rrr_python3_socket_f_start (PyObject *self, PyObject *args, PyObject *kwds) {
	struct rrr_python3_socket_data *socket_data = (struct rrr_python3_socket_data *) self;
	int ret = 0;

	char *arg_filename = "";
	char *valid_keys[] = {"filename", NULL};

	__rrr_python3_socket_dealloc_internals(self);

	if (args != NULL) {
		if (!PyArg_ParseTupleAndKeywords(args, kwds, "|s", valid_keys, &arg_filename)) {
			VL_MSG_ERR("Could not parse arguments to socket __init__ python3 module: %s\n", strerror(errno));
			ret = 1;
			goto out;
		}
	}

	int new_socket = rrr_socket(AF_UNIX, SOCK_SEQPACKET/*|O_NONBLOCK*/, 0, "rrr_python3_socket_f_start - socket");

	if (new_socket == -1) {
		VL_MSG_ERR("Could not create UNIX socket in python3 module socket __init__: %s\n", strerror(errno));
		ret = 1;
		goto out;
	}

	char *filename = NULL;
	if (*arg_filename != '\0') {
		// Connect to existing socket
		socket_data->connected_fd = new_socket;

		filename = arg_filename;
	}
	else {
		// Create new socket for listening
		socket_data->socket_fd = new_socket;

		char filename_template[128];
		sprintf(filename_template, "%s%s", RRR_TMP_PATH, "/rrr-py-socket-XXXXXX");

		filename = filename_template;

		int fd = rrr_socket_mkstemp (filename, "rrr_python3_socket_f_start - mkstemp");
		if (fd == -1) {
			VL_MSG_ERR("Could not create temporary filename for UNIX socket in python3 module in socket __init_: %s\n", strerror(errno));
			ret = 1;
			goto out;
		}
		rrr_socket_close(fd);

		if (unlink (filename) != 0) {
			VL_MSG_ERR("Could not unlink file to be used for socket in python3 socket __init__ (%s): %s\n", arg_filename, strerror(errno));
			ret = 1;
			goto out;
		}
	}

	socket_data->filename = malloc(strlen(filename)+1);
	strcpy(socket_data->filename, filename);

	struct sockaddr_un addr;
	memset(&addr, '\0', sizeof(addr));
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, socket_data->filename, sizeof(addr.sun_path)-1);

	if (socket_data->socket_fd > 0) {
		if (bind(socket_data->socket_fd, (struct sockaddr *) &addr, sizeof(addr)) != 0) {
			VL_MSG_ERR("Could not bind to socket %s in python3 module in socket __init_: %s\n", socket_data->filename, strerror(errno));
			ret = 1;
			goto out;
		}

		if (listen(socket_data->socket_fd, 1) != 0) {
			VL_MSG_ERR("Could not listen on socket %s in python3 module in socket __init_: %s\n", socket_data->filename, strerror(errno));
			ret = 1;
			goto out;
		}
	}
	else {
		if (connect(socket_data->connected_fd, (struct sockaddr *) &addr, sizeof(addr)) != 0) {
			VL_MSG_ERR("Could not connect to existing socket %s in python3 module in socket __init_: %s\n", socket_data->filename, strerror(errno));
			ret = 1;
			goto out;
		}
	}

	VL_DEBUG_MSG_7 ("New python3 AF_UNIX socket filename is %s pid %i\n", socket_data->filename, getpid());

	out:
	if (ret != 0) {
		__rrr_python3_socket_dealloc_internals(self);
		Py_RETURN_FALSE;
	}
	Py_RETURN_TRUE;
}

static PyObject *rrr_python3_socket_f_get_filename(PyObject *self, PyObject *args) {
	struct rrr_python3_socket_data *socket_data = (struct rrr_python3_socket_data *) self;

	(void)(args);

	if (socket_data->filename == NULL) {
		VL_MSG_ERR("Could not get filename as socket is not initialized in python3 module\n");
		return NULL;
	}
	return (PyUnicode_FromString(socket_data->filename));
}

static PyObject *rrr_python3_socket_f_send (PyObject *self, PyObject *arg) {
	int ret = 0;

	struct rrr_socket_msg *message = NULL;
	if (rrr_python3_vl_message_check(arg)) {
		message = rrr_vl_message_safe_cast(rrr_python3_vl_message_get_message (arg));
	}
	else if (rrr_python3_setting_check(arg)) {
		message = rrr_setting_safe_cast(rrr_python3_setting_get_setting (arg));
	}
	else {
		VL_MSG_ERR("Received unknown object type in python3 socket send\n");
		ret = 1;
		goto out;
	}

	if ((ret = rrr_python3_socket_send(self, message)) != 0) {
		VL_MSG_ERR("Received error in python3 socket send function\n");
		ret = 1;
		goto out;
	}

	out:
	if (ret != 0) {
		Py_RETURN_FALSE;
	}
	Py_RETURN_TRUE;
}

static PyMethodDef socket_methods[] = {
		{
				ml_name:	"get_filename",
				ml_meth:	(PyCFunction) rrr_python3_socket_f_get_filename,
				ml_flags:	METH_NOARGS,
				ml_doc:		"Get filename of socket"
		},
		{
				ml_name:	"send",
				ml_meth:	(PyCFunction) rrr_python3_socket_f_send,
				ml_flags:	METH_O,
				ml_doc:		"Send a vl_message object on the socket"
		},
		{
				ml_name:	"start",
				ml_meth:	(PyCFunction) rrr_python3_socket_f_start,
				ml_flags:	METH_O,
				ml_doc:		"Start a new socket or connect to existing if filename provided"
		},
		{
				ml_name:	"accept",
				ml_meth:	(PyCFunction) rrr_python3_socket_f_accept,
				ml_flags:	METH_NOARGS,
				ml_doc:		"Accept a connection on the socket"
		},
		{ NULL, NULL, 0, NULL }
};

PyTypeObject rrr_python3_socket_type = {
		ob_base:			PyVarObject_HEAD_INIT(NULL, 0) // Comma is inside macro
	    tp_name:			RRR_PYTHON3_MODULE_NAME	"." RRR_PYTHON3_SOCKET_TYPE_NAME,
	    tp_basicsize:		sizeof(struct rrr_python3_socket_data),
		tp_itemsize:		0,
	    tp_dealloc:			(destructor) rrr_python3_socket_f_dealloc,
	    tp_print:			NULL,
	    tp_getattr:			NULL,
	    tp_setattr:			NULL,
	    tp_as_async:		NULL,
	    tp_repr:			NULL,
	    tp_as_number:		NULL,
	    tp_as_sequence:		NULL,
	    tp_as_mapping:		NULL,
	    tp_hash:			NULL,
	    tp_call:			NULL,
	    tp_str:				NULL,
	    tp_getattro:		NULL,
	    tp_setattro:		NULL,
	    tp_as_buffer:		NULL,
	    tp_flags:			Py_TPFLAGS_DEFAULT,
	    tp_doc:				"ReadRouteRecord type for UNIX socket IPC",
	    tp_traverse:		NULL,
	    tp_clear:			NULL,
	    tp_richcompare:		NULL,
	    tp_weaklistoffset:	0,
	    tp_iter:			NULL,
	    tp_iternext:		NULL,
	    tp_methods:			socket_methods,
	    tp_members:			NULL,
	    tp_getset:			NULL,
	    tp_base:			NULL,
	    tp_dict:			NULL,
	    tp_descr_get:		NULL,
	    tp_descr_set:		NULL,
	    tp_dictoffset:		0,
	    tp_init:			rrr_python3_socket_f_init,
	    tp_alloc:			PyType_GenericAlloc,
	    tp_new:				rrr_python3_socket_f_new,
	    tp_free:			NULL,
	    tp_is_gc:			NULL,
	    tp_bases:			NULL,
	    tp_mro:				NULL,
	    tp_cache:			NULL,
	    tp_subclasses:		NULL,
	    tp_weaklist:		NULL,
	    tp_del:				NULL,
	    tp_version_tag:		0,
	    tp_finalize:		NULL
};

const char *rrr_python3_socket_get_filename(PyObject *self) {
	struct rrr_python3_socket_data *socket_data = (struct rrr_python3_socket_data *) self;
	if (socket_data->filename == NULL) {
		VL_BUG("rrr_python3_socket_get_filename called with filename being NULL, socket it probably not initialized\n");
	}
	return socket_data->filename;
}

int rrr_python3_socket_get_fd (PyObject *self) {
	struct rrr_python3_socket_data *socket_data = (struct rrr_python3_socket_data *) self;
	return socket_data->socket_fd;
}

int rrr_python3_socket_get_connected_fd (PyObject *self) {
	struct rrr_python3_socket_data *socket_data = (struct rrr_python3_socket_data *) self;
	return socket_data->connected_fd;
}

PyObject *rrr_python3_socket_new (const char *filename) {
	struct rrr_python3_socket_data *new_socket = NULL;
	PyObject *args = NULL;
	PyObject *res = NULL;

	int ret = 0;

	new_socket = PyObject_New(struct rrr_python3_socket_data, &rrr_python3_socket_type);
	if (new_socket == NULL) {
		VL_MSG_ERR("Could not create new socket:\n");
		PyErr_Print();
		ret = 1;
		goto out;
	}

	new_socket->filename = NULL;
	new_socket->socket_fd = 0;
	new_socket->connected_fd = 0;
	new_socket->send_stats = 0;
	pthread_mutex_init(&new_socket->stats_lock, 0);
	pthread_mutex_init(&new_socket->send_lock, 0);

	new_socket->time_start = time_get_64();

	args = PyTuple_New(1);
	if (args == NULL) {
		VL_MSG_ERR("Could not create tuple in rrr_python3_socket_new:\n");
		PyErr_Print();
		ret = 1;
		goto out;
	}

	if (filename != NULL && *filename != '\0') {
		PyObject *str = PyUnicode_FromString(filename);
		if (str == NULL) {
			VL_MSG_ERR("Could not create unicode object from filename '%s' while creating new python3 socket:\n",
					filename);
			PyErr_Print();
			ret = 1;
			goto out;
		}
		PyTuple_SET_ITEM(args, 0, str);
	}

	res = rrr_python3_socket_f_start((PyObject *) new_socket, args, NULL);
	if (!PyObject_IsTrue(res)) {
		VL_MSG_ERR("Could not start socket with filename '%s' while creating new python3 socket:\n",
				(filename != NULL ? filename : ""));
		ret = 1;
		goto out;
	}

	out:
	if (ret != 0) {
		RRR_Py_XDECREF((PyObject *) new_socket);
		new_socket = NULL;
	}
	RRR_Py_XDECREF(args);
	RRR_Py_XDECREF(res);
	return (PyObject *) new_socket;
}

int rrr_python3_socket_poll (PyObject *socket, int timeout) {
	struct rrr_python3_socket_data *socket_data = (struct rrr_python3_socket_data *) socket;
	int ret = 0;

	if (socket_data->connected_fd == 0) {
		VL_MSG_ERR("Cannot poll in python3 socket: Not connected\n");
		ret = -1;
		goto out;
	}

	struct pollfd poll_data = {
			socket_data->connected_fd,
			POLLIN,
			0
	};

	int max_retries = 100;

	retry:
	if ((ret = poll(&poll_data, 1, timeout)) == -1) {
		if (--max_retries == 100) {
			VL_MSG_ERR("Max retries reached in rrr_python3_socket_poll for socket %i pid %i\n",
					socket_data->connected_fd, getpid());
			ret = -1;
			goto out;
		}
		else if (errno == EAGAIN || errno == EWOULDBLOCK) {
			goto retry;
		}
		else if (errno == EINTR) {
			goto retry;
		}
		VL_MSG_ERR("Error from poll function in python3 unix socket fd %i pid %i: %s\n",
				socket_data->connected_fd, getpid(), strerror(errno));
		ret = -1;
	}

	if (ret != 0) {
		VL_DEBUG_MSG_7 ("python3 socket poll on socket %s fd %i pid %i result %i\n",
				rrr_python3_socket_get_filename(socket),
				rrr_python3_socket_get_fd(socket),
				getpid(), ret
		);
	}

	out:
	return ret;
}

union merged_buf {
	struct rrr_setting_packed setting; // The two others have this struct at the top
	struct rrr_socket_msg msg;
	struct vl_message vl_message;
};

int rrr_python3_socket_send (PyObject *socket, struct rrr_socket_msg *message) {
	struct rrr_python3_socket_data *socket_data = (struct rrr_python3_socket_data *) socket;
	int ret = 0;

	VL_DEBUG_MSG_7 ("python3 socket send on socket %s fd %i pid %i size %u\n",
			rrr_python3_socket_get_filename(socket),
			rrr_python3_socket_get_fd(socket),
			getpid(), message->msg_size
	);

	if (message->msg_size < sizeof(struct rrr_socket_msg) ||  message->msg_size > sizeof(union merged_buf)) {
		VL_BUG("Received a socket message of wrong size in rrr_python3_socket_send (it says %u bytes)\n", message->msg_size);
	}

	char buf[message->msg_size];

	if (socket_data->connected_fd == 0) {
		VL_MSG_ERR("Cannot send in python3 socket: Not connected\n");
		ret = 1;
		goto out;
	}

	memcpy(buf, message, sizeof(buf));
	struct rrr_socket_msg *msg_new = (struct rrr_socket_msg *) buf;

	if (RRR_SOCKET_MSG_IS_VL_MESSAGE(msg_new)) {
		if (message->msg_size != sizeof(struct vl_message)) {
			VL_BUG("Received a vl_message with wrong size parameter %u in  rrr_python3_socket_send\n", message->msg_size);
		}
		if (message_validate((struct vl_message *) msg_new) != 0) {
			VL_BUG("Received an invalid vl_message in  rrr_python3_socket_send\n");
		}
		message_prepare_for_network((struct vl_message *) msg_new);
	}
	else if (RRR_SOCKET_MSG_IS_SETTING(msg_new)) {
		if (message->msg_size != sizeof(struct rrr_setting_packed)) {
			VL_BUG("Received a setting with wrong size parameter %u in  rrr_python3_socket_send\n", message->msg_size);
		}
		rrr_settings_packed_prepare_for_network((struct rrr_setting_packed*) msg_new);
	}
	else if (RRR_SOCKET_MSG_IS_CTRL(msg_new)) {
		rrr_socket_msg_populate_head(msg_new, msg_new->msg_type, msg_new->msg_size, message->msg_value);
		rrr_socket_msg_checksum(msg_new, sizeof(*msg_new));
		rrr_socket_msg_head_to_network(msg_new);
	}
	else {
		VL_MSG_ERR("Received socket message of unkown type %u in rrr_python3_socket_send\n", message->msg_type);
		ret = 1;
		goto out;
	}


	pthread_mutex_lock(&socket_data->stats_lock);
	uint64_t time_send_start = time_get_64();
	if (time_send_start - socket_data->time_start > 1000000) {
		VL_DEBUG_MSG_1 ("python3 socket send on socket %s fd %i pid %i messages %i\n",
				rrr_python3_socket_get_filename(socket),
				rrr_python3_socket_get_fd(socket),
				getpid(),
				socket_data->send_stats
		);
		socket_data->time_start = time_send_start;
		socket_data->send_stats = 0;
	}
	pthread_mutex_unlock(&socket_data->stats_lock);

	int max_retries = 1000000;

	retry:
	pthread_mutex_lock(&socket_data->send_lock);
	ret = sendto(socket_data->connected_fd, &buf, sizeof(buf), MSG_EOR|MSG_DONTWAIT, NULL, 0);
	pthread_mutex_unlock(&socket_data->send_lock);
	if (ret != (int) sizeof(buf)) {
		if (ret == -1) {
			if (--max_retries == 0) {
				VL_MSG_ERR("Max retries reached in rrr_python3_socket_send for socket %i pid %i\n",
						socket_data->connected_fd, getpid()
				);
				ret = 1;
				goto out;
			}
			else if (errno == EAGAIN || errno == EWOULDBLOCK) {
				usleep(10);
				goto retry;
			}
			else if (errno == EINTR) {
				usleep(10);
				goto retry;
			}
			else {
				VL_MSG_ERR("Error from send function in python3 unix socket fd %i pid %i: %s\n",
						socket_data->connected_fd, getpid(), strerror(errno));
				ret = 1;
				goto out;
			}
		}
		else {
			VL_MSG_ERR("Error while sending message in python3 socket, sent %i of %lu bytes\n",
					ret, sizeof(buf));
			ret = 1;
			goto out;
		}
	}

	pthread_mutex_lock(&socket_data->stats_lock);
	socket_data->send_stats++;
	pthread_mutex_unlock(&socket_data->stats_lock);
	ret = 0;

	out:
	return ret;
}

int rrr_python3_socket_recv (struct rrr_socket_msg **result, PyObject *socket, int timeout) {
	struct rrr_python3_socket_data *socket_data = (struct rrr_python3_socket_data *) socket;
	int ret = 0;

	*result = NULL;

	union merged_buf *buf = NULL;

	if (socket_data->connected_fd == 0) {
		VL_MSG_ERR("Cannot receive in python3 socket: Not connected\n");
		ret = 1;
		goto out;
	}

	int numitems = rrr_python3_socket_poll(socket, timeout);
	if (numitems == -1) {
		VL_MSG_ERR("Could not poll in python3 socket recv function\n");
		ret = 1;
		goto out;
	}
	else if (numitems == 0) {
		goto out;
	}

	VL_DEBUG_MSG_7 ("python3 socket recv on socket %s fd %i pid %i poll result %i\n",
			rrr_python3_socket_get_filename(socket),
			rrr_python3_socket_get_fd(socket),
			getpid(), numitems
	);

	// For now we only support receiving one at a time
	numitems = 1;

	buf = malloc(sizeof(*buf));

	if (buf == NULL) {
		VL_MSG_ERR("Could not allocate memory in rrr_python3_socket_recv\n");
		ret = 1;
		goto out;
	}

	int max_retries = 100;

	retry:
	ret = recvfrom(socket_data->connected_fd, buf, sizeof(*buf), /*MSG_DONTWAIT*/0, NULL, NULL);

	if (ret >= (int) sizeof(buf->msg)) {
		VL_DEBUG_MSG_7 ("python3 socket recv on socket %s fd %i pid %i size %u\n",
				rrr_python3_socket_get_filename(socket),
				rrr_python3_socket_get_fd(socket),
				getpid(), ret
		);

		// We need to duplicate the head as the downstream structs do no convert
		// if they find the head already having correct endianess
		struct rrr_socket_msg tmp_head = buf->msg;
		rrr_socket_msg_head_to_host(&tmp_head);

		if (ret != (int)tmp_head.msg_size) {
			VL_MSG_ERR("Received a message in python3 socket receive function which says it has %u bytes, but we have read %i\n", tmp_head.msg_size, ret);
			ret = 1;
			goto out;
		}

		// The type specified in the header is validated implicitly in the if's below, and
		// the actual size is checked within each type. Only check for min/max size here.
		if (tmp_head.msg_size < sizeof(struct rrr_socket_msg) || tmp_head.msg_size > sizeof(union merged_buf)) {
			VL_MSG_ERR("Received a message in python3 socket receive function with invalid size %u\n", tmp_head.msg_size);
			ret = 1;
			goto out;
		}

		if (RRR_SOCKET_MSG_IS_VL_MESSAGE(&tmp_head)) {
			if (message_convert_endianess(&(buf->vl_message)) != 0) {
				VL_MSG_ERR("Received vl_message in python3 socket receive function with unknown endianess\n");
				ret = 1;
				goto out;
			}
			if (message_validate (&(buf->vl_message))) {
				VL_MSG_ERR("Received vl_message in python3 socket receive function which could not be validated\n");
				ret = 1;
				goto out;
			}
		}
		else if (RRR_SOCKET_MSG_IS_SETTING(&tmp_head)) {
			if (rrr_settings_packed_convert_endianess (&buf->setting) != 0) {
				VL_MSG_ERR("Received setting message in python3 socket receive function with unknown endianess\n");
				ret = 1;
				goto out;
			}
			if (rrr_settings_packed_validate(&(buf->setting))) {
				VL_MSG_ERR("Received setting message in python3 socket receive function which could not be validated\n");
				ret = 1;
				goto out;
			}
		}
		else if (RRR_SOCKET_MSG_IS_CTRL(&tmp_head)) {
			buf->msg = tmp_head;
		}
		else {
			VL_MSG_ERR("Received a message in python3 socket receive function  of unknown type %u\n", buf->msg.msg_type);
			ret = 1;
			goto out;
		}

		ret = rrr_socket_msg_checksum_check(&buf->msg, buf->msg.msg_size);
		if (ret != 0) {
			VL_MSG_ERR("Received message in python3 socket receive function with wrong checksum\n");
			ret = 1;
			goto out;
		}
		if (RRR_SOCKET_MSG_IS_CTRL(&buf->msg)) {
			if (rrr_socket_msg_head_validate(&buf->msg) != 0) {
				VL_MSG_ERR("Received control message was invalid in python3 socket receive\n");
				ret = 1;
				goto out;
			}

			// Above validate function should catch invalid flags
			VL_BUG("Unknown flags in control message in python3 socket receive\n");

			// Do not return control messages to application
			goto out_free;
		}

		ret = 0;
	}
	else if (ret == 0) {
		goto out_free;
	}
	else if (ret == -1) {
		if (--max_retries == 0) {
			VL_MSG_ERR("Max retries reached in rrr_python3_socket_recv for socket %i pid %i\n",
					socket_data->connected_fd, getpid());
			ret = 1;
			goto out;
		}
		else if (errno == EAGAIN || errno == EWOULDBLOCK) {
			goto retry;
		}
		else if (errno == EINTR) {
			goto retry;
		}
		else {
			VL_MSG_ERR("Error from recvfrom function in python3 unix socket fd %i pid %i: %s\n",
					socket_data->connected_fd, getpid(), strerror(errno));
			ret = 1;
			goto out;
		}
	}
	else {
		VL_MSG_ERR("Warning: Received message of unknown length %i in python3 socket receive function\n", ret);
		ret = 0;
		goto out_free;
	}

	out:
		if (ret == 0) {
			*result = &buf->msg;
		}
		else {
			RRR_FREE_IF_NOT_NULL(buf);
		}
		return ret;

	out_free:
		free(buf);
		buf = NULL;
		return ret;
}

int rrr_python3_socket_accept (PyObject *self) {
	PyObject *res = NULL;
	int ret = 0;

	res = rrr_python3_socket_f_accept(self, NULL);

	ret = (PyObject_IsTrue(res) ? 0 : 1);
	RRR_Py_XDECREF(res);

	return ret;
}

void rrr_python3_socket_close (PyObject *self) {
	struct rrr_python3_socket_data *socket_data = (struct rrr_python3_socket_data *) self;

	if (socket_data->connected_fd > 0) {
		rrr_socket_close(socket_data->connected_fd);
		socket_data->connected_fd = 0;
	}
	if (socket_data->socket_fd > 0) {
		rrr_socket_close(socket_data->socket_fd);
		socket_data->socket_fd = 0;
	}
}