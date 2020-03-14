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

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>

#include "main.h"
#include "global.h"
#include "lib/common.h"
#include "lib/instances.h"
#include "lib/instance_config.h"
#include "lib/cmdlineparser/cmdline.h"
#include "lib/version.h"
#include "lib/configuration.h"
#include "lib/threads.h"
#include "lib/version.h"
#include "lib/rrr_socket.h"

const char *module_library_paths[] = {
		RRR_MODULE_PATH,
		"/usr/lib/rrr",
		"/lib/rrr",
		"/usr/local/lib/rrr",
		"/usr/lib/",
		"/lib/",
		"/usr/local/lib/",
		"./src/modules/.libs",
		"./src/modules",
		"./modules",
		"./",
		""
};

#ifndef RRR_BUILD_TIMESTAMP
#define RRR_BUILD_TIMESTAMP 1
#endif

// Used so that debugger output at program exit can show function names
// on the stack correctly
// #define RRR_NO_MODULE_UNLOAD

static volatile int main_running = 1;

int main_signal_handler(int s, void *arg) {
	(void)(arg);

	if (s == SIGCHLD) {
		RRR_DBG_1("Received SIGCHLD\n");
	}
	else if (s == SIGUSR1) {
		main_running = 0;
		return RRR_SIGNAL_HANDLED;
	}
	else if (s == SIGPIPE) {
		RRR_MSG_ERR("Received SIGPIPE, ignoring\n");
	}
	else if (s == SIGTERM) {
		exit(EXIT_FAILURE);
	}
	else if (s == SIGINT) {
		// Allow double ctrl+c to close program
		if (s == SIGINT) {
			RRR_MSG_ERR("Received SIGINT\n");
			signal(SIGINT, SIG_DFL);
		}

		main_running = 0;
		return RRR_SIGNAL_HANDLED;
	}

	return RRR_SIGNAL_NOT_HANDLED;
}

static const struct cmd_arg_rule cmd_rules[] = {
		{CMD_ARG_FLAG_NO_FLAG,		'\0',	"config",				"{CONFIGURATION FILE}"},
		{CMD_ARG_FLAG_HAS_ARGUMENT,	'd',	"debuglevel",			"[-d|--debuglevel[=]DEBUG FLAGS]"},
		{CMD_ARG_FLAG_HAS_ARGUMENT,	'D',	"debuglevel_on_exit",	"[-D|--debuglevel_on_exit[=]DEBUG FLAGS]"},
		{0,							'W',	"no_watchdog_timers",	"[-W|--no_watchdog_timers]"},
		{0,							'T',	"no_thread_restart",	"[-T|--no_thread_restart]"},
		{0,							'h',	"help",					"[-h|--help]"},
		{0,							'v',	"version",				"[-v|--version]"},
		{0,							'\0',	NULL,					NULL}
};

int main (int argc, const char *argv[]) {
	if (!rrr_verify_library_build_timestamp(RRR_BUILD_TIMESTAMP)) {
		RRR_MSG_ERR("Library build version mismatch.\n");
		exit(EXIT_FAILURE);
	}

	struct rrr_signal_handler *signal_handler = NULL;
	struct rrr_thread_collection *collection = NULL;
	struct instance_metadata_collection *instances = NULL;
	const char *config_string = NULL;
	struct rrr_config *config = NULL;
	int ret = EXIT_SUCCESS;
	int count = 0;

	struct cmd_data cmd;
	cmd_init(&cmd, cmd_rules, argc, argv);

	struct rrr_signal_functions signal_functions = {
			rrr_signal_handler_set_active,
			rrr_signal_handler_push,
			rrr_signal_handler_remove
	};

	signal_handler = signal_functions.push_handler(main_signal_handler, NULL);

	if (rrr_instance_metadata_collection_new (&instances, &signal_functions) != 0) {
		ret = EXIT_FAILURE;
		goto out_cleanup_signal;
	}

	if ((ret = main_parse_cmd_arguments(&cmd, CMD_CONFIG_DEFAULTS)) != 0) {
		goto out_destroy_metadata_collection;
	}

	if (rrr_print_help_and_version(&cmd) != 0) {
		goto out_destroy_metadata_collection;
	}

	RRR_DBG_1("ReadRouteRecord debuglevel is: %u\n", RRR_DEBUGLEVEL);

	config_string = cmd_get_value(&cmd, "config", 0);
	if (config_string != NULL && *config_string != '\0') {
		config = rrr_config_parse_file(config_string);

		if (config == NULL) {
			ret = EXIT_FAILURE;
			RRR_MSG_ERR("Configuration file parsing failed\n");
			goto out_unload_modules;
		}

		RRR_DBG_1("found %d instances\n", config->module_count);

		ret = rrr_instance_process_from_config(instances, config, module_library_paths);

		if (ret != 0) {
			goto out_unload_modules;
		}
	}

	if (RRR_DEBUGLEVEL_1) {
		if (config != NULL && rrr_config_dump(config) != 0) {
			RRR_MSG_ERR("Error occured while dumping configuration\n");
		}
	}

	// Initialzie dynamic_data thread data
	struct sigaction action;
	action.sa_handler = rrr_signal;
	sigemptyset (&action.sa_mask);
	action.sa_flags = 0;

	threads_restart:

	rrr_socket_close_all();

	// During preload stage, signals are temporarily deactivated.
	instances->signal_functions->set_active(RRR_SIGNALS_ACTIVE);

	// Handle forked children exiting
	sigaction (SIGCHLD, &action, NULL);
	// We generally ignore sigpipe and use NONBLOCK on all sockets
	sigaction (SIGPIPE, &action, NULL);
	// Used to set main_running = 0. The signal is set to default afterwards
	// so that a second SIGINT will terminate the process
	sigaction (SIGINT, &action, NULL);
	// Used to set main_running = 0;
	sigaction (SIGUSR1, &action, NULL);
	// Exit immediately with EXIT_FAILURE
	sigaction (SIGTERM, &action, NULL);

	rrr_set_debuglevel_orig();
	if ((ret = main_start_threads(&collection, instances, config, &cmd)) != 0) {
		goto out_stop_threads;
	}

	while (main_running) {
		usleep (100000);

		if (rrr_instance_check_threads_stopped(instances) == 1) {
			RRR_DBG_1 ("One or more threads have finished or do hard restart. Restart.\n");

			rrr_set_debuglevel_on_exit();
			main_threads_stop(collection, instances);
			rrr_thread_destroy_collection (collection);

			if (main_running && rrr_global_config.no_thread_restart == 0) {
				usleep(1000000);
				goto threads_restart;
			}
			else {
				goto out_unload_modules;
			}
		}

		rrr_thread_run_ghost_cleanup(&count);
		if (count > 0) {
			RRR_MSG_ERR("Main cleaned up after %i ghost(s) (in loop)\n", count);
		}
	}

	RRR_DBG_1 ("Main loop finished\n");

	out_stop_threads:
		rrr_set_debuglevel_on_exit();
		RRR_DBG_1("Debuglevel on exit is: %i\n", rrr_global_config.debuglevel);
		main_threads_stop(collection, instances);
		rrr_thread_destroy_collection (collection);
		rrr_thread_run_ghost_cleanup(&count);
		if (count > 0) {
			RRR_MSG_ERR("Main cleaned up after %i ghost(s) (after loop)\n", count);
		}
		rrr_socket_close_all();

	out_unload_modules:
#ifndef RRR_NO_MODULE_UNLOAD
		rrr_instance_unload_all(instances);
#endif
		if (config != NULL) {
			rrr_config_destroy(config);
		}

	out_destroy_metadata_collection:
		rrr_instance_metadata_collection_destroy(instances);

	out_cleanup_signal:
		rrr_signal_handler_remove(signal_handler);
		rrr_exit_cleanup_methods_run_and_free();
		if (ret == 0) {
			RRR_DBG_1("Exiting program without errors\n");
		}
		else {
			RRR_DBG_1("Exiting program with errors\n");
		}
		cmd_destroy(&cmd);
		return ret;
}
