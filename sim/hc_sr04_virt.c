/*
 * HC-SR04 Ultrasonic Distance Sensor — SimAVR Virtual Device
 *
 * Watches TRIGGER pin output via IRQ. On rising edge, schedules
 * ECHO pin HIGH/LOW using avr_cycle_timer_register_usec() for
 * cycle-accurate virtual-time timing.
 *
 * Echo duration = distance_cm * 58 µs (speed of sound round-trip)
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "sim_avr.h"
#include "sim_time.h"
#include "avr_ioport.h"
#include "hc_sr04_virt.h"

static avr_cycle_count_t
hc_sr04_echo_end(avr_t *avr, avr_cycle_count_t when, void *param)
{
    hc_sr04_virt_t *dev = (hc_sr04_virt_t*)param;
    (void)when;
    /* Set ECHO pin LOW — firmware's pulseIn() captures this edge */
    avr_raise_irq(dev->irq_echo, 0);
    return 0;
}

static avr_cycle_count_t
hc_sr04_echo_start(avr_t *avr, avr_cycle_count_t when, void *param)
{
    hc_sr04_virt_t *dev = (hc_sr04_virt_t*)param;
    (void)when;

    /* Set ECHO pin HIGH */
    avr_raise_irq(dev->irq_echo, 1);

    /* Schedule ECHO LOW after distance-proportional delay */
    uint32_t echo_us = (uint32_t)(dev->distance_cm * 58.0f);
    if (echo_us < 116) echo_us = 116;   /* min 2cm */
    if (echo_us > 23200) echo_us = 23200; /* max 400cm */

    avr_cycle_timer_register_usec(avr, echo_us, hc_sr04_echo_end, dev);
    return 0;
}

static void
hc_sr04_trigger_notify(struct avr_irq_t *irq, uint32_t value, void *param)
{
    hc_sr04_virt_t *dev = (hc_sr04_virt_t*)param;
    (void)irq;

    int high = (value & 1);

    if (high && !dev->trigger_was_high) {
        /* Rising edge on TRIGGER — schedule echo after 2µs propagation delay */
        avr_cycle_timer_register_usec(dev->avr, 2, hc_sr04_echo_start, dev);
    }
    dev->trigger_was_high = high;
}

void hc_sr04_virt_init(avr_t *avr, hc_sr04_virt_t *dev, float distance_cm)
{
    memset(dev, 0, sizeof(*dev));
    dev->avr = avr;
    dev->distance_cm = (distance_cm > 0) ? distance_cm : 15.0f;
    dev->trigger_was_high = 0;
    dev->verbose = 0;
}

void hc_sr04_virt_attach(hc_sr04_virt_t *dev,
                          char trig_port, int trig_bit,
                          char echo_port, int echo_bit)
{
    /* Get trigger pin output IRQ — watch for firmware setting this pin */
    dev->irq_trigger = avr_io_getirq(dev->avr,
        AVR_IOCTL_IOPORT_GETIRQ(trig_port), trig_bit);

    /* Get echo pin input IRQ — we raise this to simulate sensor response */
    dev->irq_echo = avr_io_getirq(dev->avr,
        AVR_IOCTL_IOPORT_GETIRQ(echo_port), echo_bit);

    if (dev->irq_trigger) {
        avr_irq_register_notify(dev->irq_trigger, hc_sr04_trigger_notify, dev);
    }

    fprintf(stderr, "✅ HC-SR04: trig=PORT%c%d, echo=PORT%c%d, dist=%.1fcm\n",
            trig_port, trig_bit, echo_port, echo_bit, dev->distance_cm);
}
