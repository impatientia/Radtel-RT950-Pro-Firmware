/*
 * gps.h - GPS module interface for the RT-950 Pro
 *
 * Connected via USART3 at 9600 baud (PB10/PB11 default, no remap).
 * Receives NMEA sentences.
 */

#ifndef APP_GPS_H
#define APP_GPS_H

#include <stdint.h>

typedef struct {
    float latitude;
    float longitude;
    float altitude;
    uint8_t fix_quality;
    uint8_t num_satellites;
    uint8_t hour, minute, second;
} gps_data_t;

void gps_init(void);
void gps_process(void);
const gps_data_t *gps_get_data(void);

/* Raw ring buffer access (for diagnostics/hw_test) */
int      gps_rx_available(void);
uint8_t  gps_rx_read(void);

#endif /* APP_GPS_H */
