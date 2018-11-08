#ifndef UCDFS_TELEMETRY_STATUS_DISPLAY_H
#define UCDFS_TELEMETRY_STATUS_DISPLAY_H

#include <ssd1306.h>

void status_display_init();
void status_display_task();
ssd1306_handle_t status_display_get();
void status_display_http_server_init();
void status_display_http_server_stop();

#endif //UCDFS_TELEMETRY_STATUS_DISPLAY_H
