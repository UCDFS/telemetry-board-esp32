#ifndef GPS_H
#define GPS_H

#define GPS_UART UART_NUM_2
#define GPS_TXD GPIO_NUM_16
#define GPS_RXD GPIO_NUM_17
#define GPS_BUFFER_SIZE 256

void gps_read_task();

#endif