/*
 * avr_gpio_monitor.h
 *
 * GPIO monitoring for SimAVR - Header file
 */

#ifndef __AVR_GPIO_MONITOR_H__
#define __AVR_GPIO_MONITOR_H__

#include "sim_avr.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize GPIO monitor (structure setup and callback registration)
// pipe_path: Path to named pipe for GPIO state updates
void avr_gpio_monitor_init(avr_t *avr, const char *pipe_path);

// Reset GPIO monitor (start pipe thread, like UART DMA)
// Called during avr_reset() to start the pipe thread
void avr_gpio_monitor_reset(void);

// Cleanup GPIO monitor
void avr_gpio_monitor_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* __AVR_GPIO_MONITOR_H__ */

