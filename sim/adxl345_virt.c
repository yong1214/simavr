/*
	adxl345_virt.c

	Virtual ADXL345 3-Axis Accelerometer for SimAVR
	Digital accelerometer with 13-bit resolution

	Based on tmp105_virt
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "avr_twi.h"
#include "adxl345_virt.h"

static const char * _adxl345_irq_names[2] = {
	[ADXL345_TWI_IRQ_INPUT] = "8>adxl345.out",
	[ADXL345_TWI_IRQ_OUTPUT] = "32<adxl345.in",
};

/*
 * Read a single register byte at the current register address
 * Auto-increments the register address after each read
 */
static uint8_t
adxl345_virt_read_reg(adxl345_virt_t * p)
{
	uint8_t data = 0;

	switch (p->reg_addr) {
		case ADXL345_VIRT_DEVID:
			data = ADXL345_VIRT_DEVID_VAL;
			break;
		case ADXL345_VIRT_BW_RATE:
			data = p->bw_rate;
			break;
		case ADXL345_VIRT_POWER_CTL:
			data = p->power_ctl;
			break;
		case ADXL345_VIRT_INT_ENABLE:
			data = p->int_enable;
			break;
		case ADXL345_VIRT_DATA_FORMAT:
			data = p->data_format;
			break;
		case ADXL345_VIRT_DATAX0:
			data = p->accel_x & 0xFF;		// X LSB
			break;
		case ADXL345_VIRT_DATAX1:
			data = (p->accel_x >> 8) & 0xFF;	// X MSB
			break;
		case ADXL345_VIRT_DATAY0:
			data = p->accel_y & 0xFF;		// Y LSB
			break;
		case ADXL345_VIRT_DATAY1:
			data = (p->accel_y >> 8) & 0xFF;	// Y MSB
			break;
		case ADXL345_VIRT_DATAZ0:
			data = p->accel_z & 0xFF;		// Z LSB
			break;
		case ADXL345_VIRT_DATAZ1:
			data = (p->accel_z >> 8) & 0xFF;	// Z MSB
			break;
		case ADXL345_VIRT_INT_SOURCE:
			data = 0x00; // No interrupts active
			break;
		case ADXL345_VIRT_FIFO_STATUS:
			data = 0x00; // FIFO empty
			break;
		default:
			if (p->verbose)
				printf("adxl345 READ unknown register 0x%02X\n", p->reg_addr);
			data = 0;
			break;
	}

	if (p->verbose)
		printf("adxl345 READ reg 0x%02X = 0x%02X\n", p->reg_addr, data);

	// Auto-increment register address
	p->reg_addr++;

	return data;
}

/*
 * I2C transaction hook - handles START, STOP, and data transactions
 */
static void
adxl345_virt_in_hook(
		struct avr_irq_t * irq,
		uint32_t value,
		void * param)
{
	adxl345_virt_t * p = (adxl345_virt_t*)param;
	avr_twi_msg_irq_t v;
	v.u.v = value;

	/*
	 * If we receive a STOP, reset the transaction
	 */
	if (v.u.twi.msg & TWI_COND_STOP) {
		if (p->selected) {
			if (p->verbose)
				printf("adxl345 received stop\n");
		}
		p->selected = 0;
		p->reg_selected = 0;
		p->reg_byte_pos = 0;
	}

	/*
	 * If we receive a START, check if the slave address is meant for us,
	 * and if so reply with an ACK bit
	 */
	if (v.u.twi.msg & TWI_COND_START) {
		p->selected = 0;
		if ((v.u.twi.addr >> 1) == ADXL345_VIRT_TWI_ADDR) {
			if (p->verbose)
				printf("adxl345 received start (addr=0x%02X)\n", v.u.twi.addr);
			p->selected = v.u.twi.addr;
			avr_raise_irq(p->irq + ADXL345_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_ACK, p->selected, 1));
		}
	}

	/*
	 * If it's a data transaction and we're selected, handle it
	 */
	if (p->selected) {
		if (v.u.twi.msg & TWI_COND_WRITE) {
			avr_raise_irq(p->irq + ADXL345_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_ACK, p->selected, 1));

			if (p->verbose)
				printf("adxl345 WRITE data 0x%02X\n", v.u.twi.data);

			if (!p->reg_selected) {
				// First byte sets register pointer
				p->reg_addr = v.u.twi.data;
				p->reg_selected = 1;
				p->reg_byte_pos = 0;
				if (p->verbose)
					printf("adxl345 register pointer set to 0x%02X\n", p->reg_addr);
			} else {
				// Write to the selected register
				switch (p->reg_addr) {
					case ADXL345_VIRT_POWER_CTL:
						p->power_ctl = v.u.twi.data;
						if (p->verbose)
							printf("adxl345: power_ctl set to 0x%02X\n", p->power_ctl);
						break;
					case ADXL345_VIRT_DATA_FORMAT:
						p->data_format = v.u.twi.data;
						if (p->verbose)
							printf("adxl345: data_format set to 0x%02X\n", p->data_format);
						break;
					case ADXL345_VIRT_BW_RATE:
						p->bw_rate = v.u.twi.data;
						if (p->verbose)
							printf("adxl345: bw_rate set to 0x%02X\n", p->bw_rate);
						break;
					case ADXL345_VIRT_INT_ENABLE:
						p->int_enable = v.u.twi.data;
						if (p->verbose)
							printf("adxl345: int_enable set to 0x%02X\n", p->int_enable);
						break;
					case ADXL345_VIRT_THRESH_TAP:
					case ADXL345_VIRT_OFSX:
					case ADXL345_VIRT_OFSY:
					case ADXL345_VIRT_OFSZ:
					case ADXL345_VIRT_INT_MAP:
					case ADXL345_VIRT_FIFO_CTL:
						// Writable registers we don't track
						if (p->verbose)
							printf("adxl345: write 0x%02X to reg 0x%02X (ignored)\n",
								v.u.twi.data, p->reg_addr);
						break;
					default:
						if (p->verbose)
							printf("adxl345: write to unknown/read-only register 0x%02X\n",
								p->reg_addr);
						break;
				}
				p->reg_addr++;
				p->reg_byte_pos = 0;
			}
		}

		if (v.u.twi.msg & TWI_COND_READ) {
			// If register not selected, default to device ID
			if (!p->reg_selected) {
				p->reg_addr = ADXL345_VIRT_DEVID;
				p->reg_selected = 1;
			}

			uint8_t data = adxl345_virt_read_reg(p);

			avr_raise_irq(p->irq + ADXL345_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_READ, p->selected, data));
		}
	}
}

void
adxl345_virt_init(
		struct avr_t * avr,
		adxl345_virt_t * p)
{
	memset(p, 0, sizeof(*p));

	p->avr = avr;
	p->irq = avr_alloc_irq(&avr->irq_pool, 0, ADXL345_IRQ_COUNT, _adxl345_irq_names);
	avr_irq_register_notify(p->irq + ADXL345_TWI_IRQ_OUTPUT, adxl345_virt_in_hook, p);

	// Initialize registers with default values
	p->power_ctl = 0x00;
	p->data_format = 0x00;
	p->bw_rate = 0x0A;	// 100 Hz default
	p->int_enable = 0x00;

	// Default acceleration: 0g X, 0g Y, 1g Z
	adxl345_virt_set_accel_x(p, 0.0f);
	adxl345_virt_set_accel_y(p, 0.0f);
	adxl345_virt_set_accel_z(p, 1.0f);

	p->reg_addr = 0;
	p->reg_selected = 0;
	p->reg_byte_pos = 0;
	p->verbose = 0;
}

void
adxl345_virt_attach_twi(adxl345_virt_t * p,
                        uint32_t i2c_irq_base)
{
	avr_connect_irq(
		p->irq + ADXL345_TWI_IRQ_OUTPUT,
		avr_io_getirq(p->avr, i2c_irq_base, TWI_IRQ_INPUT));
	avr_connect_irq(
		avr_io_getirq(p->avr, i2c_irq_base, TWI_IRQ_OUTPUT),
		p->irq + ADXL345_TWI_IRQ_INPUT);
}

void
adxl345_virt_set_accel_x(adxl345_virt_t * p, float g)
{
	// ADXL345 default range: +-2g, LSB = 3.9mg
	p->accel_x = (int16_t)(g / 0.0039f);
	if (p->verbose)
		printf("adxl345: X acceleration set to %.4fg (raw=%d)\n",
			g, p->accel_x);
}

void
adxl345_virt_set_accel_y(adxl345_virt_t * p, float g)
{
	p->accel_y = (int16_t)(g / 0.0039f);
	if (p->verbose)
		printf("adxl345: Y acceleration set to %.4fg (raw=%d)\n",
			g, p->accel_y);
}

void
adxl345_virt_set_accel_z(adxl345_virt_t * p, float g)
{
	p->accel_z = (int16_t)(g / 0.0039f);
	if (p->verbose)
		printf("adxl345: Z acceleration set to %.4fg (raw=%d)\n",
			g, p->accel_z);
}
