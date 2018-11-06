#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ubx_protocol.h"
#include "ubx_tools.h"

char * ubx_message_generate(ubx_message_class_t message_class, ubx_message_id_t message_id, uint8_t *payload, size_t payload_length, size_t *message_length) {
	*message_length = sizeof(ubx_header_t) + payload_length + sizeof(ubx_checksum_t);
	char *message = calloc(1, *message_length);

	ubx_header_t *header = (ubx_header_t *) message;
	ubx_checksum_t *checksum = (ubx_checksum_t *) (message + sizeof(ubx_header_t) + payload_length);

	header->sync_char_1 = UBX_HEADER_SYNC_CHAR_1;
	header->sync_char_2 = UBX_HEADER_SYNC_CHAR_2;
	header->message_class = message_class;
	header->message_id = message_id;
	header->payload_length = (uint16_t) payload_length;

	if (payload_length > 0) {
		memcpy(message + sizeof(ubx_header_t), payload, payload_length);
	}

	ubx_message_calculate_checksum(&header->message_class, (uint32_t) checksum - (uint32_t) &header->message_class,
			checksum);

	return message;
}

void ubx_message_calculate_checksum(const uint8_t *data, size_t length, ubx_checksum_t *checksum) {
	checksum->ck_a = 0;
	checksum->ck_b = 0;

	for (int i = 0; i < length; i++) {
		checksum->ck_a += *(data + i);
		checksum->ck_b += checksum->ck_a;
	}
}