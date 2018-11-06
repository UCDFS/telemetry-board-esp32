#ifndef UBX_TOOLS_H
#define UBX_TOOLS_H

#include <stdlib.h>

#include "ubx_protocol.h"

char * ubx_message_generate(ubx_message_class_t message_class, ubx_message_id_t message_id, uint8_t *payload, size_t payload_length, size_t *message_length);

bool ubx_message_verify_checksum(const uint8_t *data, size_t length, ubx_checksum_t *checksum);
void ubx_message_calculate_checksum(const uint8_t *data, size_t length, ubx_checksum_t *checksum);

#endif //UBX_TOOLS_H
