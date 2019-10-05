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

#ifndef RRR_TYPE_HEADER
#define RRR_TYPE_HEADER

#include <stdint.h>

#include "linked_list.h"

static const union type_system_endian {
	uint16_t two;
	uint8_t one;
} type_system_endian = {0x1};

#define RRR_TYPE_SYSTEM_ENDIAN_IS_LE (type_system_endian.one == 1)
#define RRR_TYPE_SYSTEM_ENDIAN_IS_BE (type_system_endian.one == 0)

#define RRR_TYPE_PARSE_OK			0
#define RRR_TYPE_PARSE_HARD_ERR		1
#define RRR_TYPE_PARSE_SOFT_ERR		2
#define RRR_TYPE_PARSE_INCOMPLETE	3

// Remember to update convert function pointers in types.c
// Highest possible ID is 255 (uint8_t)
#define RRR_TYPE_MIN		2
#define RRR_TYPE_LE			2 // Little endian number
#define RRR_TYPE_BE			3 // Big endian number
#define RRR_TYPE_H			4 // Host endian number (can be both)
#define RRR_TYPE_BLOB		5 // Type which holds arbitary data
#define RRR_TYPE_USTR		6 // Unsigned int given as a string
#define RRR_TYPE_ISTR		7 // Signed int given as a string
#define RRR_TYPE_SEP		8 // Separator character ;,.-_*+\/=$@%#!|§ etc. No brackets.
#define RRR_TYPE_MSG		9 // Type which holds an RRR message
#define RRR_TYPE_FIXP		10 // Signed 64 type of which 24 bits are fraction given as string in base10 or base16
#define RRR_TYPE_STR		11 // Dynamic length string quoted with "
#define RRR_TYPE_MAX		11

#define RRR_TYPE_NAME_LE	"le"
#define RRR_TYPE_NAME_BE	"be"
#define RRR_TYPE_NAME_H		"h"
#define RRR_TYPE_NAME_BLOB	"blob"
#define RRR_TYPE_NAME_USTR	"ustr"
#define RRR_TYPE_NAME_ISTR	"istr"
#define RRR_TYPE_NAME_SEP	"sep"
#define RRR_TYPE_NAME_MSG	"msg"
#define RRR_TYPE_NAME_FIXP	"fixp"
#define RRR_TYPE_NAME_STR	"str"

#define RRR_TYPE_MAX_LE		sizeof(rrr_type_le)
#define RRR_TYPE_MAX_BE		sizeof(rrr_type_be)
#define RRR_TYPE_MAX_H		sizeof(rrr_type_h)
#define RRR_TYPE_MAX_BLOB	RRR_TYPE_MAX_BLOB_LENGTH
#define RRR_TYPE_MAX_USTR	0
#define RRR_TYPE_MAX_ISTR	0
#define RRR_TYPE_MAX_SEP	64
#define RRR_TYPE_MAX_MSG	0
#define RRR_TYPE_MAX_FIXP	0
#define RRR_TYPE_MAX_STR	0

#define RRR_TYPE_IS_64(type) 	(														\
			(type) == RRR_TYPE_LE || (type) == RRR_TYPE_BE || (type) == RRR_TYPE_H ||	\
			(type) == RRR_TYPE_USTR || (type) == RRR_TYPE_ISTR							\
		)
#define RRR_TYPE_IS_BLOB(type)		((type) == RRR_TYPE_BLOB || (type) == RRR_TYPE_SEP || (type) == RRR_TYPE_MSG || (type) == RRR_TYPE_STR)
#define RRR_TYPE_IS_FIXP(type)		((type) == RRR_TYPE_FIXP)
#define RRR_TYPE_IS_MSG(type)		((type) == RRR_TYPE_MSG)
#define RRR_TYPE_IS_STR(type)		((type) == RRR_TYPE_STR || (type) == RRR_TYPE_SEP)
#define RRR_TYPE_IS_SEP(type)		((type) == RRR_TYPE_SEP)
#define RRR_TYPE_ALLOWS_SIGN(type)	((type) == RRR_TYPE_LE || (type) == RRR_TYPE_BE || (type) == RRR_TYPE_H)
#define RRR_TYPE_OK(type)			((type) >= RRR_TYPE_MIN && (type) <= RRR_TYPE_MAX)

#define RRR_TYPE_FLAG_SIGNED 1<<0

#define RRR_TYPE_FLAG_IS_SIGNED(flags)		(((flags) & RRR_TYPE_FLAG_SIGNED) == 1)
#define RRR_TYPE_FLAG_IS_UNSIGNED(flags)	(((flags) & RRR_TYPE_FLAG_SIGNED) == 0)

#define RRR_TYPE_FLAG_SET_SIGNED(flags)		(flags) |= (RRR_TYPE_FLAG_SIGNED)
#define RRR_TYPE_FLAG_SET_UNSIGNED(flags)	(flags) &= ~(RRR_TYPE_FLAG_SIGNED)

#define RRR_TYPE_GET_IMPORT_LENGTH_ARGS		\
		ssize_t *import_length,				\
		const struct rrr_type_value *node,	\
		const char *buf,					\
		ssize_t buf_size

#define RRR_TYPE_IMPORT_ARGS				\
		struct rrr_type_value *node,		\
		ssize_t *parsed_bytes,				\
		const char *start,					\
		const char *end

#define RRR_TYPE_UNPACK_ARGS				\
		struct rrr_type_value *node

#define RRR_TYPE_PACK_ARGS					\
		char *target,						\
		ssize_t *written_bytes,				\
		uint8_t *new_type_id,				\
		const struct rrr_type_value *node

typedef uint8_t rrr_type;
typedef uint8_t rrr_type_flags;
typedef uint32_t rrr_type_length;
typedef uint32_t rrr_def_count;
typedef uint32_t rrr_type_array_size;
typedef uint32_t rrr_size;
typedef uint64_t rrr_type_le;
typedef uint64_t rrr_type_be;
typedef uint64_t rrr_type_h;
typedef uint64_t rrr_type_istr;
typedef uint64_t rrr_type_ustr;

struct rrr_type_value;

struct rrr_type_definition {
	rrr_type type;
	rrr_type_length max_length;
	int (*get_import_length)(RRR_TYPE_GET_IMPORT_LENGTH_ARGS);
	int (*import)(RRR_TYPE_IMPORT_ARGS);
	int (*unpack)(RRR_TYPE_UNPACK_ARGS);
	int (*pack)(RRR_TYPE_PACK_ARGS);
	const char *identifier;
};

struct rrr_type_value {
	RRR_LINKED_LIST_NODE(struct rrr_type_value);
	const struct rrr_type_definition *definition;
	rrr_type_flags flags;
	rrr_type_length tag_length;
	rrr_type_length import_length;
	rrr_type_length import_elements;
	rrr_type_length total_stored_length;
	rrr_type_array_size element_count; // 1 = no array, 0 = auto
	char *tag;
	char *data;
};

const struct rrr_type_definition *rrr_type_parse_from_string (
		ssize_t *parsed_bytes,
		const char *start,
		const char *end
);
const struct rrr_type_definition *rrr_type_get_from_id (
		uint8_t type_in
);
void rrr_type_value_destroy (
		struct rrr_type_value *template
);
int rrr_type_value_new (
		struct rrr_type_value **result,
		const struct rrr_type_definition *type,
		rrr_type_flags flags,
		rrr_type_length tag_length,
		const char *tag,
		rrr_type_length import_length,
		rrr_type_array_size element_count,
		rrr_type_length stored_length
);

#endif /* RRR_TYPE_HEADER */
