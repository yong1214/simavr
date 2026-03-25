/*
 * avr_gpio_inject.h
 *
 * GPIO input injection for SimAVR - Header file
 */

#ifndef AVR_GPIO_INJECT_H
#define AVR_GPIO_INJECT_H

#include "sim_avr.h"
#include "gpio_timing_executor.h"

// Initialize GPIO injector
void avr_gpio_inject_init(avr_t *avr, const char *pipe_path);

// Start GPIO inject thread (called from avr_reset)
void avr_gpio_inject_reset(void);

// Register GTPE devices for runtime param updates via inject pipe
void avr_gpio_inject_register_gte(GteDevice *devices, int count);

// Cleanup GPIO injector
void avr_gpio_inject_cleanup(void);

#endif

