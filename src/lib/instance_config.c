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

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "log.h"
#include "settings.h"
#include "instance_config.h"
#include "map.h"
#include "array.h"
#include "array_tree.h"
#include "parse.h"
#include "util/gnu.h"
#include "util/linked_list.h"

int rrr_instance_config_string_set (
		char **target,
		const char *prefix,
		const char *name,
		const char *suffix
) {
	RRR_FREE_IF_NOT_NULL(*target);
	if (rrr_asprintf(target, "%s%s%s", prefix, name, (suffix != NULL ? suffix : "")) < 0) {
		RRR_MSG_0("Could not allocate memory in rrr_instance_config_string_set\n");
		return 1;
	}
	return 0;
}

void rrr_instance_config_destroy(struct rrr_instance_config_data *config) {
	rrr_settings_destroy(config->settings);
	free(config->name);
	free(config);
}

struct rrr_instance_config_data *rrr_instance_config_new (
		const char *name_begin,
		const int name_length,
		const int max_settings,
		const struct rrr_array_tree_list *global_array_trees
) {
	struct rrr_instance_config_data *ret = NULL;

	char *name = malloc(name_length + 1);
	if (name == NULL) {
		RRR_MSG_0("Could not allocate memory for name in __rrr_config_new_instance_config");
		goto out;
	}

	ret = malloc(sizeof(*ret));
	if (ret == NULL) {
		RRR_MSG_0("Could not allocate memory for name in __rrr_config_new_instance_config");
		goto out_free_name;
	}

	memcpy(name, name_begin, name_length);
	name[name_length] = '\0';

	ret->name = name;
	ret->settings = rrr_settings_new(max_settings);
	if (ret->settings == NULL) {
		RRR_MSG_0("Could not create settings structure in __rrr_config_new_instance_config");
		goto out_free_config;
	}
	ret->global_array_trees = global_array_trees;

	goto out;

	out_free_config:
	free(ret);
	ret = NULL;

	out_free_name:
	free(name);

	out:
	return ret;
}

int rrr_instance_config_read_port_number (rrr_setting_uint *target, struct rrr_instance_config_data *source, const char *name) {
	int ret = 0;

	*target = 0;

	rrr_setting_uint tmp_uint = 0;
	ret = rrr_settings_read_unsigned_integer (&tmp_uint, source->settings, name);

	if (ret != 0) {
		if (ret == RRR_SETTING_PARSE_ERROR) {
			char *tmp_string;

			rrr_settings_read_string (&tmp_string, source->settings, name); // Ignore error
			RRR_MSG_0 (
					"Syntax error in port setting %s. Could not parse '%s' as number.\n",
					name, (tmp_string != NULL ? tmp_string : "")
			);

			RRR_FREE_IF_NOT_NULL(tmp_string);

			ret = 1;
			goto out;
		}
	}
	else {
		if (tmp_uint < 1 || tmp_uint > 65535) {
			RRR_MSG_0 (
					"port setting %s out of range, must be 1-65535 but was %" PRIrrrbl ".\n",
					name, tmp_uint
			);
			ret = 1;
			goto out;
		}
	}

	*target = tmp_uint;

	out:
	return ret;
}

int rrr_instance_config_check_all_settings_used (struct rrr_instance_config_data *config) {
	int ret = rrr_settings_check_all_used (config->settings);

	if (ret != 0) {
		RRR_MSG_0("Warning: Not all settings of instance %s were used, possible typo in configuration file\n",
				config->name);
	}

	return ret;
}

int rrr_instance_config_parse_array_tree_definition_from_config_silent_fail (
		struct rrr_array_tree **target_array_tree,
		struct rrr_instance_config_data *config,
		const char *cmd_key
) {
	int ret = 0;

	*target_array_tree = NULL;

	struct rrr_array_tree *new_tree = NULL;

	char *target_str_tmp = NULL;

	if ((ret = rrr_settings_get_string_noconvert_silent(&target_str_tmp, config->settings, cmd_key)) != 0) {
		if (ret == RRR_SETTING_NOT_FOUND) {
			goto out;
		}
		else {
			RRR_MSG_0("Error while parsing setting %s in instance %s\n", cmd_key, config->name);
			ret = 1;
			goto out;
		}
	}

	char *curly_start_pos = strchr(target_str_tmp, '{');
	char *curly_end_pos = strchr(target_str_tmp, '}');

	if (curly_start_pos == NULL || curly_end_pos == NULL || !(curly_end_pos > curly_start_pos)) {
		size_t definition_length = strlen(target_str_tmp);

		// Replace terminating \0 with semicolon. We don't actually use the \0 to
		// figure out where the end is when parsing the array. This adding of ;
		// allows simple array definition to be specified without ; at the end.
		target_str_tmp[definition_length] = ';';

		if (rrr_array_tree_interpret_raw (
				&new_tree,
				target_str_tmp,
				definition_length + 1, // DO NOT use strlen here, string no longer has \0
				"-"
		)) {
			RRR_MSG_0("Error while parsing array tree in setting %s in instance %s\n", cmd_key, config->name);
			ret = 1;
			goto out;
		}
	}
	else {
		const char *name_pos = curly_start_pos + 1;
		*curly_end_pos = '\0';

		const struct rrr_array_tree *array_tree = rrr_array_tree_list_get_tree_by_name (
				config->global_array_trees,
				name_pos
		);

		if (array_tree == NULL) {
			RRR_MSG_0("Array tree with name '%s' not found, check spelling\n", name_pos);
			ret = 1;
			goto out;
		}

		if ((ret = rrr_array_tree_clone_without_data(&new_tree, array_tree)) != 0) {
			goto out;
		}
	}

	if (RRR_DEBUGLEVEL_1) {
		rrr_array_tree_dump(new_tree);
	}

	*target_array_tree = new_tree;
	new_tree = NULL;

	out:
		RRR_FREE_IF_NOT_NULL(target_str_tmp);
		if (new_tree != NULL) {
			rrr_array_tree_destroy(new_tree);
		}
		return ret;
}

struct parse_associative_list_to_map_callback_data {
	struct rrr_map *target;
	const char *delimeter;
};

static int __parse_associative_list_to_map_callback (
		const char *value,
		void *arg
) {
	struct parse_associative_list_to_map_callback_data *data = arg;
	return rrr_map_parse_pair(value, data->target, data->delimeter);
}

int rrr_instance_config_parse_comma_separated_associative_to_map (
		struct rrr_map *target,
		struct rrr_instance_config_data *config,
		const char *cmd_key,
		const char *delimeter
) {
	struct parse_associative_list_to_map_callback_data callback_data = {
			target, delimeter
	};

	return rrr_instance_config_traverse_split_commas_silent_fail (
			config,
			cmd_key,
			__parse_associative_list_to_map_callback,
			&callback_data
	);
}

int rrr_instance_config_parse_comma_separated_to_map (
		struct rrr_map *target,
		struct rrr_instance_config_data *config,
		const char *cmd_key
) {
	struct parse_associative_list_to_map_callback_data callback_data = {
			target, NULL
	};

	return rrr_instance_config_traverse_split_commas_silent_fail (
			config,
			cmd_key,
			__parse_associative_list_to_map_callback,
			&callback_data
	);
}

int rrr_instance_config_parse_optional_utf8 (
		char **target,
		struct rrr_instance_config_data *config,
		const char *string,
		const char *def
) {
	int ret = 0;

	if ((ret = rrr_settings_get_string_noconvert_silent(target, config->settings, string)) != 0) {
		if (ret == RRR_SETTING_NOT_FOUND) {
			if (def != NULL && (*target = strdup(def)) == NULL) {
				RRR_MSG_0("Could not allocate memory for default value of setting %s in instance %s\n",
					string, config->name);
				ret = 1;
				goto out;
			}
		}
		else {
			RRR_MSG_0("Error while parsing setting %s in instance %s\n", string, config->name);
			ret = 1;
			goto out;
		}
		ret = 0;
	}
	else {
		if (rrr_utf8_validate(*target, strlen(*target)) != 0) {
			RRR_MSG_0("Setting %s in instance %s was not valid UTF-8\n", string, config->name);
			ret = 1;
			goto out;
		}
	}

	out:
	return ret;
}
