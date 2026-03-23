#ifndef HC_SR04_VIRT_H
#define HC_SR04_VIRT_H

#include "sim_avr.h"
#include "sim_irq.h"

/**
 * HC-SR04 Ultrasonic Distance Sensor — SimAVR Virtual Device
 *
 * Uses avr_cycle_timer_register_usec() for cycle-accurate echo timing.
 * Firmware's pulseIn() measures exact virtual microseconds.
 */
typedef struct hc_sr04_virt_t {
    avr_t *avr;
    avr_irq_t *irq_trigger;    /* IRQ to watch trigger pin output */
    avr_irq_t *irq_echo;       /* IRQ to set echo pin input */
    float distance_cm;
    int trigger_was_high;
    int verbose;
} hc_sr04_virt_t;

void hc_sr04_virt_init(avr_t *avr, hc_sr04_virt_t *dev, float distance_cm);

/* Attach to AVR GPIO pins. port/bit are AVR port (e.g., 'D',4 for Arduino D4) */
void hc_sr04_virt_attach(hc_sr04_virt_t *dev,
                          char trig_port, int trig_bit,
                          char echo_port, int echo_bit);

#endif /* HC_SR04_VIRT_H */
