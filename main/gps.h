#ifndef UCDFS_TELEMETRY_GPS_H
#define UCDFS_TELEMETRY_GPS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void gps_init();

bool gps_send_message(uint8_t *data, size_t length);

#endif //UCDFS_TELEMETRY_GPS_H