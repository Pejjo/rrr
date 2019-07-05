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

#ifndef VL_MESSAGES_H
#define VL_MESSAGES_H

#define MSG_TYPE_MSG 1
#define MSG_TYPE_ACK 2
#define MSG_TYPE_TAG 3

#define MSG_CLASS_POINT 1
#define MSG_CLASS_AVG 2
#define MSG_CLASS_MAX 3
#define MSG_CLASS_MIN 4
#define MSG_CLASS_INFO 10
#define MSG_CLASS_ARRAY 11

#define MSG_TYPE_MSG_STRING "MSG"
#define MSG_TYPE_ACK_STRING "ACK"
#define MSG_TYPE_TAG_STRING "TAG"

#define MSG_CLASS_POINT_STRING "POINT"
#define MSG_CLASS_AVG_STRING "AVG"
#define MSG_CLASS_MAX_STRING "MAX"
#define MSG_CLASS_MIN_STRING "MIN"
#define MSG_CLASS_INFO_STRING "INFO"
#define MSG_CLASS_ARRAY_STRING "ARRAY"

#define MSG_DATA_MAX_LENGTH 1024

#define MSG_SEND_MAX_LENGTH (6 + 10*2 + 32*5 + MSG_DATA_MAX_LENGTH + 1)

#define MSG_TMP_SIZE 64

#define MSG_IS_MSG(message)			(message->type == MSG_TYPE_MSG)
#define MSG_IS_ACK(message)			(message->type == MSG_TYPE_ACK)
#define MSG_IS_TAG(message)			(message->type == MSG_TYPE_TAG)

#define MSG_IS_POINT(message)		(message->class == MSG_CLASS_POINT)
#define MSG_IS_INFO(message)		(message->class == MSG_CLASS_INFO)
#define MSG_IS_ARRAY(message)		(message->class == MSG_CLASS_ARRAY)

#define MSG_IS_MSG_POINT(message)	(MSG_IS_MSG(message) && MSG_IS_POINT(message))
#define MSG_IS_MSG_INFO(message)	(MSG_IS_MSG(message) && MSG_IS_INFO(message))
#define MSG_IS_MSG_ARRAY(message)	(MSG_IS_MSG(message) && MSG_IS_ARRAY(message))

#define MSG_ENDIAN_BYTES	0x0102
#define MSG_ENDIAN_LE		0x02
#define MSG_ENDIAN_BE		0x01

#define MSG_IS_LE(msg)		(msg->endian_one == MSG_ENDIAN_LE)
#define MSG_IS_BE(msg)		(msg->endian_one == MSG_ENDIAN_BE)

#include <stdint.h>

struct vl_message {
	// Used by ipclient and ipserver for network transfer. CRC must be first
	// as we skip the first 4 bytes of the message when calculating.
	uint32_t crc32;
	union {
		uint16_t endian_two;
		uint8_t endian_one;
	};
	uint16_t reserved;

	uint32_t type;
	uint32_t class;
	uint64_t timestamp_from;
	uint64_t timestamp_to;
	uint64_t data_numeric;

	uint32_t length;
	char data[MSG_DATA_MAX_LENGTH+2];
} __attribute__((packed));

struct vl_message *message_new_reading (
	uint64_t reading_millis,
	uint64_t time
);
struct vl_message *message_new_info (
	uint64_t time,
	const char *msg_terminated
);
struct vl_message *message_new_array (
	uint64_t time,
	uint32_t length
);
int init_empty_message (
	unsigned long int type,
	unsigned long int class,
	uint64_t timestamp_from,
	uint64_t timestamp_to,
	uint64_t data_numeric,
	unsigned long int data_size,
	struct vl_message *result
);
int init_message (
	unsigned long int type,
	unsigned long int class,
	uint64_t timestamp_from,
	uint64_t timestamp_to,
	uint64_t data_numeric,
	const char *data,
	unsigned long int data_size,
	struct vl_message *result
);
/*int parse_message (
	const char *msg,
	unsigned long int size,
	struct vl_message *result
);
int message_to_string (
	struct vl_message *message,
	char *target,
	unsigned long int target_size
);
int message_fix_endianess (
	struct vl_message *message
);*/
void message_checksum (
	struct vl_message *message
);
int message_checksum_check (
	struct vl_message *message
);
int message_convert_endianess (
	struct vl_message *message
);
void message_prepare_for_network (
	struct vl_message *message
);
struct vl_message *message_duplicate (
	struct vl_message *message
);

#endif
