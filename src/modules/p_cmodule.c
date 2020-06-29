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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <inttypes.h>
#include <dlfcn.h>
#include <sys/stat.h>

#include "../lib/ip.h"
#include "../lib/poll_helper.h"
#include "../lib/instance_config.h"
#include "../lib/instances.h"
#include "../lib/messages.h"
#include "../lib/threads.h"
#include "../lib/message_broker.h"
#include "../lib/log.h"
#include "../lib/cmodule_common.h"
#include "../lib/cmodule_native.h"
#include "../lib/stats/stats_instance.h"

const char *cmodule_library_paths[] = {
		"/usr/lib/rrr/cmodules",
		"/lib/rrr/cmodules",
		"/usr/local/lib/rrr/cmodules",
		"./src/cmodules/.libs",
		"./src/cmodules",
		"../cmodules/.libs", // <!-- For test suite
		"../cmodules", // <!-- For test suite
		"./cmodules",
		"./",
		""
};

struct cmodule_data {
	struct rrr_instance_thread_data *thread_data;

	int do_drop_on_error;

	char *cmodule_name;

	char *config_function;
	char *source_function;
	char *process_function;
	char *cleanup_function;

	rrr_setting_uint source_interval_ms;

	char *log_prefix;
};

static void cmodule_data_cleanup(void *arg) {
	struct cmodule_data *data = arg;

	RRR_FREE_IF_NOT_NULL(data->cmodule_name);

	RRR_FREE_IF_NOT_NULL(data->config_function);
	RRR_FREE_IF_NOT_NULL(data->source_function);
	RRR_FREE_IF_NOT_NULL(data->process_function);
	RRR_FREE_IF_NOT_NULL(data->cleanup_function);

	RRR_FREE_IF_NOT_NULL(data->log_prefix);
}

static int cmodule_data_init(struct cmodule_data *data, struct rrr_instance_thread_data *thread_data) {
	int ret = 0;
	data->thread_data = thread_data;
	if (ret != 0) {
		cmodule_data_cleanup(data);
	}
	return ret;
}

static int cmodule_parse_config (struct cmodule_data *data, struct rrr_instance_config *config) {
	int ret = 0;

	RRR_SETTINGS_PARSE_OPTIONAL_UTF8_DEFAULT_NULL("cmodule_name", cmodule_name);

	if (data->cmodule_name == NULL || *(data->cmodule_name) == '\0') {
		RRR_MSG_0("cmodule_name configuration parameter missing for cmodule instance %s\n", config->name);
		ret = 1;
		goto out;
	}

	RRR_SETTINGS_PARSE_OPTIONAL_YESNO("cmodule_drop_on_error", do_drop_on_error, 0);
	RRR_SETTINGS_PARSE_OPTIONAL_UNSIGNED("cmodule_source_interval_ms", source_interval_ms, 1000);

	RRR_SETTINGS_PARSE_OPTIONAL_UTF8_DEFAULT_NULL("cmodule_config_function", config_function);
	RRR_SETTINGS_PARSE_OPTIONAL_UTF8_DEFAULT_NULL("cmodule_source_function", source_function);
	RRR_SETTINGS_PARSE_OPTIONAL_UTF8_DEFAULT_NULL("cmodule_process_function", process_function);
	RRR_SETTINGS_PARSE_OPTIONAL_UTF8_DEFAULT_NULL("cmodule_cleanup_function", cleanup_function);

	if ((data->process_function == NULL || *(data->process_function) == '\0') &&
		(data->source_function == NULL || *(data->source_function) == '\0')
	) {
		RRR_MSG_0("No process or source function defined in configuration for cmodule instance %s\n", config->name);
		ret = 1;
		goto out;
	}

	RRR_SETTINGS_PARSE_OPTIONAL_UTF8_DEFAULT_NULL("cmodule_log_prefix", log_prefix);

	out:
	return ret;
}

struct cmodule_run_data {
	struct rrr_cmodule_ctx ctx;
	void *dl_ptr;

	struct cmodule_data *data;

	int (*config_function)(RRR_CONFIG_ARGS);
	int (*source_function)(RRR_SOURCE_ARGS);
	int (*process_function)(RRR_PROCESS_ARGS);
	int (*cleanup_function)(RRR_CLEANUP_ARGS);
};

#define GET_FUNCTION(name)																\
	do { if (data->name != NULL && *(data->name) != '\0') {								\
		if ((run_data->name = dlsym(handle, data->name)) == NULL) {						\
			RRR_MSG_0("Could not load function '%s' from cmodule instance %s: %s\n",	\
					data->name, INSTANCE_D_NAME(data->thread_data), dlerror());			\
			function_err = 1;															\
		}																				\
	} } while(0)

static void __cmodule_dl_unload (
		void *handle
) {
	if (handle == NULL) {
		return;
	}
	if (dlclose(handle) != 0) {
		RRR_MSG_0 ("Warning: Error while unloading cmodule: %s\n", dlerror());
	}
}

static int __cmodule_load (
		struct cmodule_run_data *run_data,
		struct cmodule_data *data
) {
	int ret = 1; // Default is NOT OK

	void *handle = NULL;

	for (int i = 0; *(cmodule_library_paths[i]) != '\0'; i++) {
		char path[256 + strlen(data->cmodule_name) + 1];
		sprintf(path, "%s/%s.so", cmodule_library_paths[i], data->cmodule_name);


		printf("check path %s\n", path);
		struct stat buf;
		if (stat(path, &buf) != 0) {
			if (errno == ENOENT) {
				continue;
			}
			RRR_MSG_0 ("Could not stat %s while loading module: %s\n", path, rrr_strerror(errno));
			continue;
		}

		__cmodule_dl_unload(handle);
		handle = dlopen(path, RTLD_LAZY);

		RRR_DBG_1 ("dlopen handle for %s: %p\n", data->cmodule_name, handle);

		if (handle == NULL) {
			RRR_MSG_0 ("Error while opening module %s: %s\n", path, dlerror());
			continue;
		}

		int function_err = 0;

		GET_FUNCTION(config_function);
		GET_FUNCTION(source_function);
		GET_FUNCTION(process_function);
		GET_FUNCTION(cleanup_function);

		if (function_err != 0) {
			continue;
		}

		ret = 0; // OK
		break;
	}

	if (ret != 0) {
		__cmodule_dl_unload(handle);
	}
	else {
		run_data->dl_ptr = handle;
	}

	return ret;
}

static void __cmodule_application_cleanup (void *arg) {
	struct cmodule_run_data *run_data = arg;

	if (run_data->cleanup_function == NULL) {
		RRR_DBG_1("Note: No cleanup function set for cmodule instance %s\n",
				INSTANCE_D_NAME(run_data->data->thread_data));
		goto out;
	}

	int ret = 0;
	if ((ret = run_data->cleanup_function(&run_data->ctx)) != 0) {
		RRR_MSG_0("Warning: Error %i from cleanup function in cmodule instance %s\n",
				ret, INSTANCE_D_NAME(run_data->data->thread_data));
		goto out;
	}

	out:
	return;
}

static int cmodule_init_wrapper_callback (RRR_CMODULE_INIT_WRAPPER_CALLBACK_ARGS) {
	struct cmodule_data *data = private_arg;

	(void)(configuration_callback_arg);
	(void)(process_callback_arg);

	int ret = 0;

	struct cmodule_run_data run_data = {0};

	if (__cmodule_load(&run_data, data) != 0) {
		RRR_MSG_0("Loading failed in cmodule instance %s\n",
				INSTANCE_D_NAME(data->thread_data));
		ret = 1;
		goto out;
	}

	// It's safe to use the char * from cmodule_data. It will never
	// get freed by the fork.
	if (data->log_prefix != NULL && *(data->log_prefix) != '\0') {
		rrr_global_config_set_log_prefix(data->log_prefix);
	}

	run_data.data = data;
	run_data.ctx.worker = worker;

	pthread_cleanup_push(__cmodule_dl_unload, run_data.dl_ptr);
	pthread_cleanup_push(__cmodule_application_cleanup, &run_data);

	if ((ret = rrr_cmodule_worker_loop_start (
			worker,
			configuration_callback,
			&run_data,
			process_callback,
			&run_data
	)) != 0) {
		RRR_MSG_0("Error from worker loop in __rrr_cmodule_worker_loop_init_wrapper_default\n");
		// Don't goto out, run cleanup functions
	}

	pthread_cleanup_pop(1);
	pthread_cleanup_pop(1);

	if (run_data.ctx.application_ptr != NULL) {
		RRR_MSG_0("Warning: application_ptr in ctx for cmodule instance %s was not NULL upon exit\n",
				INSTANCE_D_NAME(data->thread_data));
	}

	out:
	return ret;
}

static int cmodule_configuration_callback (RRR_CMODULE_CONFIGURATION_CALLBACK_ARGS) {
	struct cmodule_run_data *run_data = private_arg;

	(void)(worker);

	int ret = 0;

	if (run_data->config_function == NULL) {
		RRR_DBG_1("Note: No configuration function set for cmodule instance %s\n",
				INSTANCE_D_NAME(run_data->data->thread_data));
		goto out;
	}

	if ((ret = run_data->config_function(&run_data->ctx, INSTANCE_D_CONFIG(run_data->data->thread_data))) != 0) {
		RRR_MSG_0("Error %i from configuration function in cmodule instance %s\n",
				ret, INSTANCE_D_NAME(run_data->data->thread_data));
		ret = 1;
		goto out;
	}

	out:
	return ret;
}

static int cmodule_process_callback (RRR_CMODULE_PROCESS_CALLBACK_ARGS) {
	struct cmodule_run_data *run_data = private_arg;

	(void)(worker);

	int ret = 0;

	struct rrr_message *message_copy = rrr_message_duplicate(message);
	if (message_copy == NULL) {
		RRR_MSG_0("Could not allocate message in cmodule_process_callback\n");
		ret = 1;
		goto out;
	}

	if (is_spawn_ctx) {
		if (run_data->source_function == NULL) {
			RRR_BUG("BUG: Source function was NULL but we tried to source anyway in cmodule_process_callback\n");
		}
		ret = run_data->source_function(&run_data->ctx, message_copy, message_addr);
		// Don't goto out here, print error further down
	}
	else {
		if (run_data->process_function == NULL) {
			RRR_BUG("BUG: Process function was NULL but we tried to source anyway in cmodule_process_callback\n");
		}
		ret = run_data->process_function(&run_data->ctx, message_copy, message_addr);
		// Don't goto out here, print error further down
	}

	if (ret != 0) {
		RRR_MSG_0("Error %i returned from application in cmodule instance %s. Mode was %s.\n",
				ret, INSTANCE_D_NAME(run_data->data->thread_data), (is_spawn_ctx ? "sourcing" : "processing"));
		ret = 1;
		goto out;
	}

	out:
	return ret;
}

static void *thread_entry_cmodule (struct rrr_thread *thread) {
	struct rrr_instance_thread_data *thread_data = thread->private_data;
	struct cmodule_data *data = thread_data->private_data = thread_data->private_memory;
	struct rrr_poll_collection poll;

	if (cmodule_data_init(data, thread_data) != 0) {
		RRR_MSG_0("Could not initalize thread_data in cmodule instance %s\n", INSTANCE_D_NAME(thread_data));
		pthread_exit(0);
	}

	RRR_DBG_1 ("cmodule thread thread_data is %p\n", thread_data);

	rrr_poll_collection_init(&poll);
	RRR_STATS_INSTANCE_INIT_WITH_PTHREAD_CLEANUP_PUSH;
	pthread_cleanup_push(rrr_poll_collection_clear_void, &poll);
	pthread_cleanup_push(cmodule_data_cleanup, data);
//	pthread_cleanup_push(rrr_thread_set_stopping, thread);

	rrr_thread_set_state(thread, RRR_THREAD_STATE_INITIALIZED);
	rrr_thread_signal_wait(thread_data->thread, RRR_THREAD_SIGNAL_START);
	rrr_thread_set_state(thread, RRR_THREAD_STATE_RUNNING);

	if (cmodule_parse_config(data, thread_data->init_data.instance_config) != 0) {
		goto out_message;
	}

	rrr_instance_config_check_all_settings_used(thread_data->init_data.instance_config);

	rrr_poll_add_from_thread_senders (&poll, thread_data);

	if (rrr_poll_collection_count(&poll) > 0 && (data->process_function == NULL || *(data->process_function) == '\0')) {
		RRR_MSG_0("Senders are specified for cmodule instance %s, but there is no process function defined. This is an invalid configuration.\n",
				INSTANCE_D_NAME(thread_data));
		goto out_message;
	}

	pid_t fork_pid;

	if (rrr_cmodule_start_worker_fork (
			&fork_pid,
			thread_data->cmodule,
			INSTANCE_D_FORK(thread_data),
			data->source_interval_ms * 1000,
			10 * 1000,
			INSTANCE_D_NAME(thread_data),
			(data->source_function == NULL || *(data->source_function) == '\0' ? 0 : 1),
			(data->process_function == NULL || *(data->process_function) == '\0' ? 0 : 1),
			data->do_drop_on_error,
			INSTANCE_D_SETTINGS(thread_data),
			cmodule_init_wrapper_callback,
			data,
			cmodule_configuration_callback,
			NULL, // <-- in the init wrapper, this callback arg is set to child_data
			cmodule_process_callback,
			NULL  // <-- in the init wrapper, this callback is set to child_data
	) != 0) {
		RRR_MSG_0("Error while starting cmodule worker fork for instance %s\n", INSTANCE_D_NAME(thread_data));
		goto out_message;
	}

	rrr_thread_set_state(thread, RRR_THREAD_STATE_RUNNING_FORKED);

	RRR_DBG_1 ("cmodule instance %s started thread %p\n", INSTANCE_D_NAME(thread_data), thread_data);

	rrr_cmodule_common_loop (
			thread_data,
			stats,
			&poll,
			fork_pid,
			rrr_poll_collection_count(&poll) == 0
	);

	out_message:
	RRR_DBG_1 ("cmodule instance %s started thread %p\n", INSTANCE_D_NAME(thread_data), thread_data);

	//pthread_cleanup_pop(1);
	pthread_cleanup_pop(1);
	pthread_cleanup_pop(1);
	RRR_STATS_INSTANCE_CLEANUP_WITH_PTHREAD_CLEANUP_POP;
	pthread_exit(0);
}

static struct rrr_module_operations module_operations = {
		NULL,
		thread_entry_cmodule,
		NULL,
		NULL,
		NULL,
		NULL
};

static const char *module_name = "cmodule";

__attribute__((constructor)) void load(void) {
}

void init(struct rrr_instance_dynamic_data *data) {
	data->private_data = NULL;
	data->module_name = module_name;
	data->type = RRR_MODULE_TYPE_FLEXIBLE;
	data->operations = module_operations;
	data->start_priority = RRR_THREAD_START_PRIORITY_FORK;
	data->dl_ptr = NULL;
}

void unload(void) {
	RRR_DBG_1 ("Destroy cmodule module\n");
}
