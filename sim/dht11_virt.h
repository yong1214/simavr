#ifndef DHT11_VIRT_H
#define DHT11_VIRT_H

#include "sim_avr.h"
#include "sim_irq.h"

/**
 * DHT11 Temperature/Humidity Sensor — SimAVR Virtual Device
 *
 * Single-wire protocol using avr_cycle_timer_register_usec()
 * for cycle-accurate bit timing.
 */

#define DHT11_VIRT_DATA_BITS 40

typedef enum {
    DHT11_VIRT_IDLE,
    DHT11_VIRT_RESPONSE_LOW,
    DHT11_VIRT_RESPONSE_HIGH,
    DHT11_VIRT_BIT_LOW,
    DHT11_VIRT_BIT_HIGH,
    DHT11_VIRT_DONE
} dht11_virt_state_t;

typedef struct dht11_virt_t {
    avr_t *avr;
    avr_irq_t *irq_data;      /* IRQ for the DATA pin */
    float temperature;          /* °C (0-50) */
    float humidity;             /* % (20-90) */
    dht11_virt_state_t state;
    int current_bit;
    uint8_t data_bytes[5];
    int pin_was_low;
    int verbose;
} dht11_virt_t;

void dht11_virt_init(avr_t *avr, dht11_virt_t *dev,
                      float temperature, float humidity);

/* Attach to AVR GPIO pin (port='D', bit=4 for Arduino D4) */
void dht11_virt_attach(dht11_virt_t *dev, char port, int bit);

#endif /* DHT11_VIRT_H */
