#ifndef UBX_TOOLS_H
#define UBX_TOOLS_H

#include <stdlib.h>

#include "ubx_protocol.h"

#define UBX_CFG_LAYERS_RAM ((struct ubx_cfg_layers_t) {})

char * ubx_message_generate(ubx_message_class_t message_class, ubx_message_id_t message_id, uint8_t *payload, size_t payload_length, size_t *message_length);
char * ubx_message_cfg_val_del_generate(size_t count, ubx_cfg_val_key_t *vals, struct ubx_cfg_layers_t layers, size_t *message_length);
char * ubx_message_cfg_val_set_generate(size_t count, const ubx_cfg_val_val_t *vals, struct ubx_cfg_layers_t layers, size_t *message_length);
void ubx_message_calculate_checksum(const uint8_t *data, size_t length, ubx_checksum_t *checksum);

#endif //UBX_TOOLS_H
