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

#ifndef RRR_ARRAY_H
#define RRR_ARRAY_H

#include <stdint.h>

#include "cmdlineparser/cmdline.h"
#include "instance_config.h"
#include "linked_list.h"
#include "type.h"

#define RRR_ARRAY_VERSION 6

#define RRR_ARRAY_OK 				RRR_TYPE_PARSE_OK
#define RRR_ARRAY_PARSE_HARD_ERR	RRR_TYPE_PARSE_HARD_ERR
#define RRR_ARRAY_PARSE_SOFT_ERR	RRR_TYPE_PARSE_SOFT_ERR
#define RRR_ARRAY_PARSE_INCOMPLETE	RRR_TYPE_PARSE_INCOMPLETE

struct vl_message;

struct rrr_array_value_packed {
	rrr_type type;
	rrr_type_flags flags;
	rrr_type_length tag_length;
	rrr_type_length total_length;
	rrr_type_length elements;
	char data[1];
} __attribute((packed));

struct rrr_array {
		RRR_LINKED_LIST_HEAD(struct rrr_type_value);
		uint16_t version;
};

static inline int rrr_array_count(struct rrr_array *array) {
	return RRR_LINKED_LIST_COUNT(array);
}

int rrr_array_parse_single_definition (
		struct rrr_array *target,
		const char *start,
		const char *end
);

struct rrr_array_parse_single_definition_callback_data {
	struct rrr_array *target;
	int parse_ret;
};

int rrr_array_parse_single_definition_callback (
		const char *value,
		void *arg
);
int rrr_array_validate_definition (
		const struct rrr_array *target
);
int rrr_array_parse_data_from_definition (
		struct rrr_array *target,
		ssize_t *parsed_bytes,
		const char *data,
		const rrr_type_length length
);
int rrr_array_definition_collection_clone (
		struct rrr_array *target,
		const struct rrr_array *source
) ;
void rrr_array_clear (
		struct rrr_array *collection
);
struct rrr_type_value *rrr_array_value_get_by_index (
		struct rrr_array *definition,
		int idx
);
struct rrr_type_value *rrr_array_value_get_by_tag (
		struct rrr_array *definition,
		const char *tag
);
int rrr_array_get_packed_length_from_buffer (
		ssize_t *import_length,
		const struct rrr_array *definition,
		const char *buf,
		ssize_t buf_length
);
int rrr_array_new_message_from_buffer (
		struct vl_message **target,
		ssize_t *parsed_bytes,
		const char *buf,
		ssize_t buf_len,
		const char *topic,
		ssize_t topic_length,
		const struct rrr_array *definition
);
int rrr_array_new_message_from_buffer_with_callback (
		const char *buf,
		ssize_t buf_len,
		const char *topic,
		ssize_t topic_length,
		const struct rrr_array *definition,
		int (*callback)(struct vl_message *message, void *arg),
		void *callback_arg
);
int rrr_array_new_message_from_collection (
		struct vl_message **final_message,
		const struct rrr_array *definition,
		uint64_t time,
		const char *topic,
		ssize_t topic_length
);
int rrr_array_message_to_collection (
		struct rrr_array *target,
		const struct vl_message *message_orig
);

#endif /* RRR_ARRAY_H */
