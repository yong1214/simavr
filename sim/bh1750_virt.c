/*
	bh1750_virt.c

	Virtual BH1750 Ambient Light Sensor for SimAVR
	Digital 16-bit ambient light sensor

	Based on tmp105_virt
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "avr_twi.h"
#include "bh1750_virt.h"

static const char * _bh1750_irq_names[2] = {
	[BH1750_TWI_IRQ_INPUT] = "8>bh1750.out",
	[BH1750_TWI_IRQ_OUTPUT] = "32<bh1750.in",
};

/*
 * I2C transaction hook - handles START, STOP, and data transactions
 */
static void
bh1750_virt_in_hook(
		struct avr_irq_t * irq,
		uint32_t value,
		void * param)
{
	bh1750_virt_t * p = (bh1750_virt_t*)param;
	avr_twi_msg_irq_t v;
	v.u.v = value;

	/*
	 * If we receive a STOP, reset the transaction
	 */
	if (v.u.twi.msg & TWI_COND_STOP) {
		if (p->selected) {
			if (p->verbose)
				printf("bh1750 received stop\n");
		}
		p->selected = 0;
		p->read_byte_pos = 0;
	}

	/*
	 * If we receive a START, check if the slave address is meant for us,
	 * and if so reply with an ACK bit
	 */
	if (v.u.twi.msg & TWI_COND_START) {
		p->selected = 0;
		if ((v.u.twi.addr >> 1) == BH1750_VIRT_TWI_ADDR) {
			if (p->verbose)
				printf("bh1750 received start (addr=0x%02X)\n", v.u.twi.addr);
			p->selected = v.u.twi.addr;
			p->read_byte_pos = 0;
			avr_raise_irq(p->irq + BH1750_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_ACK, p->selected, 1));
		}
	}

	/*
	 * If it's a data transaction and we're selected, handle it
	 */
	if (p->selected) {
		if (v.u.twi.msg & TWI_COND_WRITE) {
			// Command-based: each write byte is a command
			avr_raise_irq(p->irq + BH1750_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_ACK, p->selected, 1));

			uint8_t cmd = v.u.twi.data;
			if (p->verbose)
				printf("bh1750 WRITE command 0x%02X\n", cmd);

			switch (cmd) {
				case BH1750_CMD_POWER_ON:
					p->powered = 1;
					if (p->verbose)
						printf("bh1750: powered on\n");
					break;
				case BH1750_CMD_POWER_DOWN:
					p->powered = 0;
					if (p->verbose)
						printf("bh1750: powered down\n");
					break;
				case BH1750_CMD_RESET:
					// Reset data register (only works when powered on)
					if (p->powered) {
						p->lux_raw = 0;
						if (p->verbose)
							printf("bh1750: reset\n");
					}
					break;
				case BH1750_CMD_CONT_H_RES:
				case BH1750_CMD_CONT_H_RES2:
				case BH1750_CMD_CONT_L_RES:
				case BH1750_CMD_ONE_H_RES:
				case BH1750_CMD_ONE_H_RES2:
				case BH1750_CMD_ONE_L_RES:
					p->mode = cmd;
					if (p->verbose)
						printf("bh1750: measurement mode set to 0x%02X\n", cmd);
					break;
				default:
					if (p->verbose)
						printf("bh1750: unknown command 0x%02X\n", cmd);
					break;
			}
		}

		if (v.u.twi.msg & TWI_COND_READ) {
			// Read returns 2 bytes: MSB then LSB of light measurement
			uint8_t data = 0;

			if (p->read_byte_pos == 0) {
				data = (p->lux_raw >> 8) & 0xFF;
				p->read_byte_pos = 1;
			} else {
				data = p->lux_raw & 0xFF;
				p->read_byte_pos = 0;
			}

			if (p->verbose)
				printf("bh1750 READ: 0x%02X (raw=0x%04X)\n", data, p->lux_raw);

			avr_raise_irq(p->irq + BH1750_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_READ, p->selected, data));
		}
	}
}

void
bh1750_virt_init(
		struct avr_t * avr,
		bh1750_virt_t * p)
{
	memset(p, 0, sizeof(*p));

	p->avr = avr;
	p->irq = avr_alloc_irq(&avr->irq_pool, 0, BH1750_IRQ_COUNT, _bh1750_irq_names);
	avr_irq_register_notify(p->irq + BH1750_TWI_IRQ_OUTPUT, bh1750_virt_in_hook, p);

	// Initialize with default values
	p->powered = 0;
	p->mode = 0;
	p->read_byte_pos = 0;

	// Default: 500 lux
	bh1750_virt_set_lux(p, 500.0f);

	p->verbose = 0;
}

void
bh1750_virt_attach_twi(bh1750_virt_t * p,
                       uint32_t i2c_irq_base)
{
	avr_connect_irq(
		p->irq + BH1750_TWI_IRQ_OUTPUT,
		avr_io_getirq(p->avr, i2c_irq_base, TWI_IRQ_INPUT));
	avr_connect_irq(
		avr_io_getirq(p->avr, i2c_irq_base, TWI_IRQ_OUTPUT),
		p->irq + BH1750_TWI_IRQ_INPUT);
}

void
bh1750_virt_set_lux(bh1750_virt_t * p, float lux)
{
	p->lux_raw = (uint16_t)(lux / 1.2f);
	if (p->verbose)
		printf("bh1750: Lux set to %.2f (raw=0x%04X)\n",
			lux, p->lux_raw);
}
