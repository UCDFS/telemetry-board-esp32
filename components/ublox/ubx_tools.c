#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

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

char * ubx_message_cfg_val_del_generate(size_t count, ubx_cfg_val_key_t *vals, struct ubx_cfg_layers_t layers, size_t *message_length) {
	size_t payload_length = sizeof(ubx_cfg_valdel_header_t) + count * 4;
	uint8_t *payload = calloc(1, *message_length);

	ubx_cfg_valdel_header_t *header = (ubx_cfg_valdel_header_t *) payload;
	header->layers = layers;
	if (!layers.ram && !layers.flash && !layers.bbr) {
		header->layers.ram = true;
	}

	memcpy(payload + sizeof(ubx_cfg_valdel_header_t), vals, count * 4);

	return ubx_message_generate(UBX_MESSAGE_CLASS_CFG, UBX_MESSAGE_ID_CFG_VALDEL, (uint8_t *) vals, count * 4, message_length);
}

char * ubx_message_cfg_val_set_generate(size_t count, const ubx_cfg_val_val_t *vals, struct ubx_cfg_layers_t layers, size_t *message_length) {
	size_t payload_length = sizeof(ubx_cfg_valset_header_t);
	for (int i = 0; i < count; i++) {
		payload_length += 4 + vals[i].size;
	}
	uint8_t *payload = calloc(1, payload_length);

	ubx_cfg_valset_header_t *header = (ubx_cfg_valset_header_t *) payload;
	header->layers = layers;
	if (!layers.ram && !layers.flash && !layers.bbr) {
		header->layers.ram = true;
	}

	int payload_offset = sizeof(ubx_cfg_valset_header_t);
	for (int i = 0; i < count; i++) {
		memcpy(payload + payload_offset, &vals[i].key, 4);
		payload_offset += 4;
		switch (vals[i].size) {
			case 1: {
				*(payload + payload_offset) = (uint8_t) vals[i].val;
				break;
			}
			case 2: {
				uint16_t val = (uint16_t) vals[i].val;
				memcpy(payload + payload_offset, &val, 2);
				break;
			}
			case 4: {
				uint32_t val = (uint16_t) vals[i].val;
				memcpy(payload + payload_offset, &val, 4);
				break;
			}
			case 8: {
				memcpy(payload + payload_offset, &vals[i].val, 8);
				break;
			}
			default:
				return NULL;
		}
	}

	return ubx_message_generate(UBX_MESSAGE_CLASS_CFG, UBX_MESSAGE_ID_CFG_VALSET, payload, payload_length, message_length);
}

void ubx_message_calculate_checksum(const uint8_t *data, size_t length, ubx_checksum_t *checksum) {
	checksum->ck_a = 0;
	checksum->ck_b = 0;

	for (int i = 0; i < length; i++) {
		checksum->ck_a += *(data + i);
		checksum->ck_b += checksum->ck_a;
	}
}