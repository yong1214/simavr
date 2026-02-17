/*
	ssd1306_virt.c

	Virtual SSD1306 OLED Display for SimAVR
	Minimal implementation that ACKs I2C transactions to allow data capture

	Based on ds1338_virt and i2c_eeprom examples
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "avr_twi.h"
#include "ssd1306_virt.h"

static const char * _ssd1306_irq_names[2] = {
	[TWI_IRQ_INPUT] = "8>ssd1306.out",
	[TWI_IRQ_OUTPUT] = "32<ssd1306.in",
};

/*
 * I2C transaction hook - handles START, STOP, and data transactions
 */
static void
ssd1306_virt_in_hook(
		struct avr_irq_t * irq,
		uint32_t value,
		void * param)
{
	ssd1306_virt_t * p = (ssd1306_virt_t*)param;
	avr_twi_msg_irq_t v;
	v.u.v = value;

	/*
	 * If we receive a STOP, reset the transaction
	 */
	if (v.u.twi.msg & TWI_COND_STOP) {
		if (p->selected) {
			if (p->verbose)
				printf("ssd1306 received stop\n");
		}
		p->selected = 0;
		p->command_mode = 1; // Reset to command mode
		p->current_page = 0;
		p->current_column = 0;
	}
	
	/*
	 * If we receive a START, check if the slave address is meant for us,
	 * and if so reply with an ACK bit
	 */
	if (v.u.twi.msg & TWI_COND_START) {
		p->selected = 0;
		if ((v.u.twi.addr >> 1) == SSD1306_VIRT_TWI_ADDR) {
			// It's us!
			if (p->verbose)
				printf("ssd1306 received start (addr=0x%02X)\n", v.u.twi.addr);
			p->selected = v.u.twi.addr;
			// Send ACK to allow transaction to continue
			avr_raise_irq(p->irq + TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_ACK, p->selected, 1));
		}
	}
	
	/*
	 * If it's a data transaction and we're selected, ACK it
	 */
	if (p->selected) {
		if (v.u.twi.msg & TWI_COND_WRITE) {
			// Write transaction - always ACK to allow data flow
			avr_raise_irq(p->irq + TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_ACK, p->selected, 1));
			
			if (p->verbose)
				printf("ssd1306 WRITE data 0x%02X\n", v.u.twi.data);
		}
		if (v.u.twi.msg & TWI_COND_READ) {
			// Read transaction - SSD1306 doesn't support reads, send 0
			avr_raise_irq(p->irq + TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_READ, p->selected, 0));
		}
	}
}

void
ssd1306_virt_init(
		struct avr_t * avr,
		ssd1306_virt_t * p)
{
	memset(p, 0, sizeof(*p));
	memset(p->display_buffer, 0, sizeof(p->display_buffer));

	p->avr = avr;
	p->irq = avr_alloc_irq(&avr->irq_pool, 0, SSD1306_IRQ_COUNT, _ssd1306_irq_names);
	avr_irq_register_notify(p->irq + TWI_IRQ_OUTPUT, ssd1306_virt_in_hook, p);
	
	p->command_mode = 1; // Start in command mode
	p->current_page = 0;
	p->current_column = 0;
	p->verbose = 0; // Disable verbose logging by default
}

void
ssd1306_virt_attach_twi(
		ssd1306_virt_t * p,
		uint32_t i2c_irq_base)
{
	// Connect BOTH directions (like ds1338_virt):
	// 1. Device INPUT → TWI INPUT (for sending ACKs back to master)
	avr_connect_irq(
		p->irq + TWI_IRQ_INPUT,
		avr_io_getirq(p->avr, i2c_irq_base, TWI_IRQ_INPUT));
	// 2. TWI OUTPUT → Device OUTPUT (for receiving data from master)
	avr_connect_irq(
		avr_io_getirq(p->avr, i2c_irq_base, TWI_IRQ_OUTPUT),
		p->irq + TWI_IRQ_OUTPUT);
}
