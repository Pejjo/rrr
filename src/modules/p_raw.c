/*

Voltage Logger

Copyright (C) 2018 Atle Solbakken atle@goliathdns.no

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


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

#include "../modules.h"
#include "../measurement.h"
#include "../lib/threads.h"
#include "p_raw.h"

struct raw_private_data {
	struct module_dynamic_data *sender;
};

static void *thread_entry(void *arg) {
	struct module_thread_data *thread_data = arg;

	thread_set_state(thread_data->thread, VL_THREAD_STATE_RUNNING);

	while (thread_check_encourage_stop(thread_data->thread) != 1) {
		update_watchdog_time(thread_data->thread);
		usleep (5000);
	}

	printf ("Thread raw %p exiting\n", thread_data->thread);

	pthread_exit(0);
	return NULL;
}

static struct module_operations module_operations = {
		thread_entry,
		NULL,
		NULL
};

static const char *module_name = "raw";

__attribute__((constructor)) void load(struct module_dynamic_data *data) {
	struct raw_private_data *private_data = malloc(sizeof(*private_data));
	memset(private_data, '\0', sizeof(*private_data));
	data->private_data = private_data;
	data->name = module_name;
	data->type = VL_MODULE_TYPE_PROCESSOR;
	data->operations = module_operations;
	data->dl_ptr = NULL;
	data->private_data = NULL;
}

__attribute__((constructor)) void unload(struct module_dynamic_data *data) {
	printf ("Destroy raw module\n");

	free(data->private_data);
}

__attribute__((constructor)) void set_sender (struct module_dynamic_data *data, struct module_dynamic_data *sender) {
	struct raw_private_data *private_data = (struct raw_private_data *) data->private_data;
	private_data->sender = sender;
}
