/*
 * avr_pwm_monitor.h
 *
 * PWM Monitor for SimAVR
 * Hooks into timer registers to report PWM configuration changes via named pipe.
 */

#ifndef __AVR_PWM_MONITOR_H__
#define __AVR_PWM_MONITOR_H__

#include "sim_avr.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the PWM monitor
// Opens /tmp/simavr_pwm.pipe and hooks into all timer instances
void avr_pwm_monitor_init(avr_t * avr);

#ifdef __cplusplus
};
#endif

#endif /* __AVR_PWM_MONITOR_H__ */
