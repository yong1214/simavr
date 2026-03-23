/*
 * DHT11 Temperature/Humidity Sensor — SimAVR Virtual Device
 *
 * State machine driven by avr_cycle_timer_register_usec().
 * Each pulse is scheduled at exact virtual-time, so firmware's
 * timing measurements are cycle-accurate.
 *
 * Protocol: 80µs LOW + 80µs HIGH (start), then 40 bits:
 *   each bit = 50µs LOW + (26µs HIGH for 0, 70µs HIGH for 1)
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "sim_avr.h"
#include "sim_time.h"
#include "avr_ioport.h"
#include "dht11_virt.h"

static avr_cycle_count_t dht11_virt_step(avr_t *avr, avr_cycle_count_t when, void *param);

static void dht11_encode_data(dht11_virt_t *dev)
{
    uint8_t hum  = (uint8_t)(dev->humidity > 90 ? 90 : (dev->humidity < 0 ? 0 : dev->humidity));
    uint8_t temp = (uint8_t)(dev->temperature > 50 ? 50 : (dev->temperature < 0 ? 0 : dev->temperature));
    dev->data_bytes[0] = hum;
    dev->data_bytes[1] = 0;
    dev->data_bytes[2] = temp;
    dev->data_bytes[3] = 0;
    dev->data_bytes[4] = (hum + temp) & 0xFF;
}

static avr_cycle_count_t
dht11_virt_step(avr_t *avr, avr_cycle_count_t when, void *param)
{
    dht11_virt_t *dev = (dht11_virt_t*)param;
    (void)when;

    switch (dev->state) {
    case DHT11_VIRT_RESPONSE_LOW:
        avr_raise_irq(dev->irq_data, 0);
        dev->state = DHT11_VIRT_RESPONSE_HIGH;
        avr_cycle_timer_register_usec(avr, 80, dht11_virt_step, dev);
        break;

    case DHT11_VIRT_RESPONSE_HIGH:
        avr_raise_irq(dev->irq_data, 1);
        dev->state = DHT11_VIRT_BIT_LOW;
        dev->current_bit = 0;
        avr_cycle_timer_register_usec(avr, 80, dht11_virt_step, dev);
        break;

    case DHT11_VIRT_BIT_LOW:
        avr_raise_irq(dev->irq_data, 0);
        dev->state = DHT11_VIRT_BIT_HIGH;
        avr_cycle_timer_register_usec(avr, 50, dht11_virt_step, dev);
        break;

    case DHT11_VIRT_BIT_HIGH: {
        avr_raise_irq(dev->irq_data, 1);
        int byte_idx = dev->current_bit / 8;
        int bit_idx  = 7 - (dev->current_bit % 8);
        int bit_val  = (dev->data_bytes[byte_idx] >> bit_idx) & 1;
        uint32_t high_us = bit_val ? 70 : 26;

        dev->current_bit++;
        if (dev->current_bit >= DHT11_VIRT_DATA_BITS) {
            dev->state = DHT11_VIRT_DONE;
        } else {
            dev->state = DHT11_VIRT_BIT_LOW;
        }
        avr_cycle_timer_register_usec(avr, high_us, dht11_virt_step, dev);
        break;
    }

    case DHT11_VIRT_DONE:
        avr_raise_irq(dev->irq_data, 1);
        dev->state = DHT11_VIRT_IDLE;
        break;

    default:
        break;
    }
    return 0;
}

static void
dht11_data_pin_notify(struct avr_irq_t *irq, uint32_t value, void *param)
{
    dht11_virt_t *dev = (dht11_virt_t*)param;
    (void)irq;

    int high = (value & 1);

    if (!high && !dev->pin_was_low && dev->state == DHT11_VIRT_IDLE) {
        /* Falling edge — firmware pulls LOW (start signal) */
        dev->pin_was_low = 1;
    } else if (high && dev->pin_was_low && dev->state == DHT11_VIRT_IDLE) {
        /* Rising edge — firmware released after LOW period.
         * Start response after 20µs delay. */
        dev->pin_was_low = 0;
        dht11_encode_data(dev);
        dev->state = DHT11_VIRT_RESPONSE_LOW;
        avr_cycle_timer_register_usec(dev->avr, 20, dht11_virt_step, dev);
    }
    dev->pin_was_low = !high;
}

void dht11_virt_init(avr_t *avr, dht11_virt_t *dev,
                      float temperature, float humidity)
{
    memset(dev, 0, sizeof(*dev));
    dev->avr = avr;
    dev->temperature = temperature;
    dev->humidity = humidity;
    dev->state = DHT11_VIRT_IDLE;
    dev->pin_was_low = 0;
    dev->verbose = 0;
}

void dht11_virt_attach(dht11_virt_t *dev, char port, int bit)
{
    dev->irq_data = avr_io_getirq(dev->avr,
        AVR_IOCTL_IOPORT_GETIRQ(port), bit);

    if (dev->irq_data) {
        avr_irq_register_notify(dev->irq_data, dht11_data_pin_notify, dev);
    }

    fprintf(stderr, "✅ DHT11: data=PORT%c%d, temp=%.0f°C, hum=%.0f%%\n",
            port, bit, dev->temperature, dev->humidity);
}
