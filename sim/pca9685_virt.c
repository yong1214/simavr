/*
	pca9685_virt.c

	Virtual PCA9685 16-Channel PWM Driver for SimAVR
	I2C-bus controlled 16-channel LED controller with PWM output

	Based on tmp105_virt and ds1338_virt examples
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "avr_twi.h"
#include "pca9685_virt.h"

static const char * _pca9685_irq_names[2] = {
	[PCA9685_TWI_IRQ_INPUT] = "8>pca9685.out",
	[PCA9685_TWI_IRQ_OUTPUT] = "32<pca9685.in",
};

/*
 * I2C transaction hook - handles START, STOP, and data transactions
 */
static void
pca9685_virt_in_hook(
		struct avr_irq_t * irq,
		uint32_t value,
		void * param)
{
	pca9685_virt_t * p = (pca9685_virt_t*)param;
	avr_twi_msg_irq_t v;
	v.u.v = value;

	/*
	 * If we receive a STOP, reset the transaction
	 */
	if (v.u.twi.msg & TWI_COND_STOP) {
		if (p->selected) {
			if (p->verbose)
				printf("pca9685 received stop\n");
		}
		p->selected = 0;
		p->reg_selected = 0;
	}

	/*
	 * If we receive a START, check if the slave address is meant for us,
	 * and if so reply with an ACK bit
	 */
	if (v.u.twi.msg & TWI_COND_START) {
		p->selected = 0;
		if ((v.u.twi.addr >> 1) == PCA9685_VIRT_TWI_ADDR) {
			if (p->verbose)
				printf("pca9685 received start (addr=0x%02X)\n", v.u.twi.addr);
			p->selected = v.u.twi.addr;
			avr_raise_irq(p->irq + PCA9685_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_ACK, p->selected, 1));
		}
	}

	/*
	 * If it's a data transaction and we're selected, handle it
	 */
	if (p->selected) {
		if (v.u.twi.msg & TWI_COND_WRITE) {
			/* Write transaction */
			avr_raise_irq(p->irq + PCA9685_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_ACK, p->selected, 1));

			if (p->verbose)
				printf("pca9685 WRITE data 0x%02X\n", v.u.twi.data);

			/* First byte sets register pointer */
			if (!p->reg_selected) {
				p->reg_addr = v.u.twi.data;
				p->reg_selected = 1;
				if (p->verbose)
					printf("pca9685 register pointer set to 0x%02X\n", p->reg_addr);
			} else {
				/* Write to the selected register */
				p->registers[p->reg_addr] = v.u.twi.data;
				if (p->verbose)
					printf("pca9685 register[0x%02X] = 0x%02X\n",
						p->reg_addr, v.u.twi.data);
				/* Auto-increment if MODE1 bit 5 (AI) is set */
				if (p->registers[0x00] & PCA9685_MODE1_AI) {
					p->reg_addr++;
				}
			}
		}

		if (v.u.twi.msg & TWI_COND_READ) {
			/* Read transaction */
			uint8_t data = p->registers[p->reg_addr];

			if (p->verbose)
				printf("pca9685 READ register[0x%02X] = 0x%02X\n",
					p->reg_addr, data);

			/* Auto-increment if MODE1 bit 5 (AI) is set */
			if (p->registers[0x00] & PCA9685_MODE1_AI) {
				p->reg_addr++;
			}

			avr_raise_irq(p->irq + PCA9685_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_READ, p->selected, data));
		}
	}
}

static void
pca9685_set_defaults(pca9685_virt_t * p)
{
	memset(p->registers, 0, sizeof(p->registers));
	p->registers[0x00] = PCA9685_MODE1_DEFAULT;     /* MODE1 */
	p->registers[0x01] = PCA9685_MODE2_DEFAULT;     /* MODE2 */
	p->registers[0xFE] = PCA9685_PRE_SCALE_DEFAULT; /* PRE_SCALE */
}

void
pca9685_virt_init(
		struct avr_t * avr,
		pca9685_virt_t * p)
{
	memset(p, 0, sizeof(*p));

	p->avr = avr;
	p->irq = avr_alloc_irq(&avr->irq_pool, 0, PCA9685_IRQ_COUNT, _pca9685_irq_names);
	avr_irq_register_notify(p->irq + PCA9685_TWI_IRQ_OUTPUT, pca9685_virt_in_hook, p);

	/* Initialize registers with default values */
	pca9685_set_defaults(p);

	p->reg_addr = 0;
	p->reg_selected = 0;
	p->verbose = 0;
}

void
pca9685_virt_attach_twi(pca9685_virt_t * p,
                         uint32_t i2c_irq_base)
{
	avr_connect_irq(
		p->irq + PCA9685_TWI_IRQ_OUTPUT,
		avr_io_getirq(p->avr, i2c_irq_base, TWI_IRQ_INPUT));
	avr_connect_irq(
		avr_io_getirq(p->avr, i2c_irq_base, TWI_IRQ_OUTPUT),
		p->irq + PCA9685_TWI_IRQ_INPUT);
}
