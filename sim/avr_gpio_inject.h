/*
 * avr_gpio_inject.h
 *
 * GPIO input injection for SimAVR - Header file
 */

#ifndef AVR_GPIO_INJECT_H
#define AVR_GPIO_INJECT_H

#include "sim_avr.h"

// Initialize GPIO injector
void avr_gpio_inject_init(avr_t *avr, const char *pipe_path);

// Start GPIO inject thread (called from avr_reset)
void avr_gpio_inject_reset(void);

// Cleanup GPIO injector
void avr_gpio_inject_cleanup(void);

#endif

