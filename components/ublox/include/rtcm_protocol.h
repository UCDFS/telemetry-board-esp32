#ifndef UCDFS_TELEMETRY_RTCM_PROTOCOL_H
#define UCDFS_TELEMETRY_RTCM_PROTOCOL_H

#include <stdint.h>

#define RTCM_HEADER_SYNC_CHAR 0xD3

#define RTCM_MESSAGE_MAX_LENGTH 1023

/**
 * RTCM Header
 */
typedef struct {
	uint8_t sync_char :8;
	uint8_t reserved :6;
	uint16_t message_length :10;
} rtcm_header_t;

#endif //UCDFS_TELEMETRY_RTCM_PROTOCOL_H
