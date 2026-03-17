/*
	bh1750_virt.h

	Virtual BH1750 Ambient Light Sensor for SimAVR
	Digital 16-bit ambient light sensor

	Based on tmp105_virt
*/

#ifndef BH1750_VIRT_H_
#define BH1750_VIRT_H_

#include "sim_irq.h"
#include "sim_avr.h"

// BH1750 I2C address (7-bit)
#define BH1750_VIRT_TWI_ADDR		0x23

// Command bytes
#define BH1750_CMD_POWER_DOWN		0x00
#define BH1750_CMD_POWER_ON		0x01
#define BH1750_CMD_RESET		0x07
#define BH1750_CMD_CONT_H_RES		0x10
#define BH1750_CMD_CONT_H_RES2		0x11
#define BH1750_CMD_CONT_L_RES		0x13
#define BH1750_CMD_ONE_H_RES		0x20
#define BH1750_CMD_ONE_H_RES2		0x21
#define BH1750_CMD_ONE_L_RES		0x23

enum {
	BH1750_TWI_IRQ_OUTPUT = 0,
	BH1750_TWI_IRQ_INPUT,
	BH1750_IRQ_COUNT
};

/*
 * BH1750 virtual device structure
 */
typedef struct bh1750_virt_t {
	struct avr_t * avr;
	avr_irq_t * irq;		// irq list
	uint8_t verbose;
	uint8_t selected;		// selected address (0 if not addressed)

	// BH1750 state
	uint8_t powered;		// Power state (0=off, 1=on)
	uint8_t mode;			// Current measurement mode
	uint8_t read_byte_pos;		// Read byte position (0=MSB, 1=LSB)

	// Measurement data
	uint16_t lux_raw;		// Raw light measurement value
} bh1750_virt_t;

void
bh1750_virt_init(struct avr_t * avr,
                 bh1750_virt_t * p);

/*
 * Attach the BH1750 to the AVR's TWI master code,
 * pass AVR_IOCTL_TWI_GETIRQ(0) for example as i2c_irq_base
 */
void
bh1750_virt_attach_twi(bh1750_virt_t * p,
                       uint32_t i2c_irq_base);

/*
 * Set light level in lux
 */
void
bh1750_virt_set_lux(bh1750_virt_t * p, float lux);

#endif /* BH1750_VIRT_H_ */
