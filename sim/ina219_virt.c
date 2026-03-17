/*
	ina219_virt.c

	Virtual INA219 Current/Power Monitor for SimAVR
	High-side current/power monitor with I2C interface

	Based on tmp105_virt
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "avr_twi.h"
#include "ina219_virt.h"

static const char * _ina219_irq_names[2] = {
	[INA219_TWI_IRQ_INPUT] = "8>ina219.out",
	[INA219_TWI_IRQ_OUTPUT] = "32<ina219.in",
};

/*
 * Get 16-bit register value for the current register address
 */
static uint16_t
ina219_virt_get_reg16(ina219_virt_t * p)
{
	switch (p->reg_addr) {
		case INA219_VIRT_CONFIG:
			return p->config;
		case INA219_VIRT_SHUNT_VOLTAGE:
			return (uint16_t)p->shunt_voltage;
		case INA219_VIRT_BUS_VOLTAGE:
			return p->bus_voltage;
		case INA219_VIRT_POWER:
			return p->power;
		case INA219_VIRT_CURRENT:
			return (uint16_t)p->current;
		case INA219_VIRT_CALIBRATION:
			return p->calibration;
		default:
			if (p->verbose)
				printf("ina219 READ unknown register 0x%02X\n", p->reg_addr);
			return 0;
	}
}

/*
 * Write 16-bit value to the current register
 */
static void
ina219_virt_write_reg16(ina219_virt_t * p, uint16_t value)
{
	switch (p->reg_addr) {
		case INA219_VIRT_CONFIG:
			p->config = value;
			if (p->verbose)
				printf("ina219: config set to 0x%04X\n", p->config);
			// Check for reset bit (bit 15)
			if (value & 0x8000) {
				p->config = INA219_VIRT_CONFIG_DEFAULT;
				if (p->verbose)
					printf("ina219: reset triggered, config = 0x%04X\n", p->config);
			}
			break;
		case INA219_VIRT_CALIBRATION:
			p->calibration = value;
			if (p->verbose)
				printf("ina219: calibration set to 0x%04X\n", p->calibration);
			break;
		case INA219_VIRT_SHUNT_VOLTAGE:
		case INA219_VIRT_BUS_VOLTAGE:
		case INA219_VIRT_POWER:
		case INA219_VIRT_CURRENT:
			// Read-only registers
			if (p->verbose)
				printf("ina219: write to read-only register 0x%02X ignored\n", p->reg_addr);
			break;
		default:
			if (p->verbose)
				printf("ina219: write to unknown register 0x%02X\n", p->reg_addr);
			break;
	}
}

/*
 * I2C transaction hook - handles START, STOP, and data transactions
 */
static void
ina219_virt_in_hook(
		struct avr_irq_t * irq,
		uint32_t value,
		void * param)
{
	ina219_virt_t * p = (ina219_virt_t*)param;
	avr_twi_msg_irq_t v;
	v.u.v = value;

	/*
	 * If we receive a STOP, reset the transaction
	 */
	if (v.u.twi.msg & TWI_COND_STOP) {
		if (p->selected) {
			if (p->verbose)
				printf("ina219 received stop\n");
			// If we received a complete 16-bit write, commit it
			if (p->reg_selected && p->write_byte_pos == 2) {
				uint16_t val = (p->write_msb << 8) | 0; // incomplete
				// This case is already handled in write handler
			}
		}
		p->selected = 0;
		p->reg_selected = 0;
		p->reg_byte_pos = 0;
		p->write_byte_pos = 0;
	}

	/*
	 * If we receive a START, check if the slave address is meant for us,
	 * and if so reply with an ACK bit
	 */
	if (v.u.twi.msg & TWI_COND_START) {
		p->selected = 0;
		if ((v.u.twi.addr >> 1) == INA219_VIRT_TWI_ADDR) {
			if (p->verbose)
				printf("ina219 received start (addr=0x%02X)\n", v.u.twi.addr);
			p->selected = v.u.twi.addr;
			p->reg_byte_pos = 0;
			p->write_byte_pos = 0;
			avr_raise_irq(p->irq + INA219_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_ACK, p->selected, 1));
		}
	}

	/*
	 * If it's a data transaction and we're selected, handle it
	 */
	if (p->selected) {
		if (v.u.twi.msg & TWI_COND_WRITE) {
			avr_raise_irq(p->irq + INA219_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_ACK, p->selected, 1));

			if (p->verbose)
				printf("ina219 WRITE data 0x%02X\n", v.u.twi.data);

			if (!p->reg_selected) {
				// First byte sets register pointer
				p->reg_addr = v.u.twi.data & 0x07; // Only 6 registers (0-5)
				p->reg_selected = 1;
				p->reg_byte_pos = 0;
				p->write_byte_pos = 0;
				if (p->verbose)
					printf("ina219 register pointer set to 0x%02X\n", p->reg_addr);
			} else {
				// Write data bytes (16-bit registers, MSB first)
				if (p->write_byte_pos == 0) {
					// MSB
					p->write_msb = v.u.twi.data;
					p->write_byte_pos = 1;
				} else {
					// LSB - commit the 16-bit write
					uint16_t val = (p->write_msb << 8) | v.u.twi.data;
					ina219_virt_write_reg16(p, val);
					p->write_byte_pos = 0;
					p->reg_addr++; // Auto-increment
				}
			}
		}

		if (v.u.twi.msg & TWI_COND_READ) {
			// If register not selected, default to config
			if (!p->reg_selected) {
				p->reg_addr = INA219_VIRT_CONFIG;
				p->reg_selected = 1;
				p->reg_byte_pos = 0;
			}

			uint16_t reg16 = ina219_virt_get_reg16(p);
			uint8_t data;

			if (p->reg_byte_pos == 0) {
				// MSB
				data = (reg16 >> 8) & 0xFF;
				p->reg_byte_pos = 1;
			} else {
				// LSB
				data = reg16 & 0xFF;
				p->reg_byte_pos = 0;
				// Auto-increment to next register after reading both bytes
				p->reg_addr++;
			}

			if (p->verbose)
				printf("ina219 READ reg 0x%02X = 0x%02X (reg16=0x%04X)\n",
					p->reg_addr, data, reg16);

			avr_raise_irq(p->irq + INA219_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_READ, p->selected, data));
		}
	}
}

void
ina219_virt_init(
		struct avr_t * avr,
		ina219_virt_t * p)
{
	memset(p, 0, sizeof(*p));

	p->avr = avr;
	p->irq = avr_alloc_irq(&avr->irq_pool, 0, INA219_IRQ_COUNT, _ina219_irq_names);
	avr_irq_register_notify(p->irq + INA219_TWI_IRQ_OUTPUT, ina219_virt_in_hook, p);

	// Initialize with default values
	p->config = INA219_VIRT_CONFIG_DEFAULT;
	p->calibration = 0x0000;

	// Default: 5V bus, 0.1V shunt, 0.1A current, 0.5W power
	ina219_virt_set_bus_voltage(p, 5.0f);
	ina219_virt_set_shunt_voltage(p, 0.1f);
	ina219_virt_set_current(p, 0.1f);
	ina219_virt_set_power(p, 0.5f);

	p->reg_addr = 0;
	p->reg_selected = 0;
	p->reg_byte_pos = 0;
	p->write_byte_pos = 0;
	p->verbose = 0;
}

void
ina219_virt_attach_twi(ina219_virt_t * p,
                       uint32_t i2c_irq_base)
{
	avr_connect_irq(
		p->irq + INA219_TWI_IRQ_OUTPUT,
		avr_io_getirq(p->avr, i2c_irq_base, TWI_IRQ_INPUT));
	avr_connect_irq(
		avr_io_getirq(p->avr, i2c_irq_base, TWI_IRQ_OUTPUT),
		p->irq + INA219_TWI_IRQ_INPUT);
}

void
ina219_virt_set_bus_voltage(ina219_virt_t * p, float volts)
{
	// Bus voltage register: LSB = 4mV, value in bits [15:3]
	p->bus_voltage = ((uint16_t)(volts / 0.004f)) << 3;
	if (p->verbose)
		printf("ina219: Bus voltage set to %.3fV (raw=0x%04X)\n",
			volts, p->bus_voltage);
}

void
ina219_virt_set_shunt_voltage(ina219_virt_t * p, float volts)
{
	// Shunt voltage register: LSB = 10uV
	p->shunt_voltage = (int16_t)(volts / 0.00001f);
	if (p->verbose)
		printf("ina219: Shunt voltage set to %.6fV (raw=%d)\n",
			volts, p->shunt_voltage);
}

void
ina219_virt_set_current(ina219_virt_t * p, float amps)
{
	// Current register: LSB = 1mA
	p->current = (int16_t)(amps / 0.001f);
	if (p->verbose)
		printf("ina219: Current set to %.4fA (raw=%d)\n",
			amps, p->current);
}

void
ina219_virt_set_power(ina219_virt_t * p, float watts)
{
	// Power register: LSB = 20mW
	p->power = (uint16_t)(watts / 0.02f);
	if (p->verbose)
		printf("ina219: Power set to %.4fW (raw=%d)\n",
			watts, p->power);
}
