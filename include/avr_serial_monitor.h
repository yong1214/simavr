/*
 * avr_serial_monitor.h
 *
 * Serial interface monitoring for SimAVR - Header file
 * Monitors SPI, I2C, and UART transactions
 */

#ifndef __AVR_SERIAL_MONITOR_H__
#define __AVR_SERIAL_MONITOR_H__

#include "sim_avr.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize serial monitor (structure setup and callback registration)
// pipe_path: Path to named pipe for serial transaction updates
void avr_serial_monitor_init(avr_t *avr, const char *pipe_path);

// Reset serial monitor (start pipe thread)
// Called during avr_reset() to start the pipe thread
void avr_serial_monitor_reset(void);

// Cleanup serial monitor
void avr_serial_monitor_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* __AVR_SERIAL_MONITOR_H__ */

