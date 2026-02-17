/*
	tmp105_virt.c

	Virtual TMP105 Temperature Sensor for SimAVR
	Digital temperature sensor with 12-bit resolution (0.0625°C per LSB)

	Based on ds1338_virt and ssd1306_virt examples
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "avr_twi.h"
#include "tmp105_virt.h"

static const char * _tmp105_irq_names[2] = {
	[TMP105_TWI_IRQ_INPUT] = "8>tmp105.out",
	[TMP105_TWI_IRQ_OUTPUT] = "32<tmp105.in",
};

/*
 * Convert Celsius to 16-bit temperature register value
 * TMP105 uses 12-bit resolution (bits 15-4), 0.0625°C per LSB
 */
static uint16_t
tmp105_celsius_to_register(float celsius)
{
	// Convert to 12-bit value (0.0625°C per LSB)
	int16_t temp12bit = (int16_t)round(celsius / 0.0625);
	
	// Convert to 16-bit format (bits 15-4)
	// Mask to 12 bits and shift left by 4
	return (temp12bit & 0x0FFF) << 4;
}

/*
 * Convert 16-bit temperature register value to Celsius
 */
static float
tmp105_register_to_celsius(uint16_t reg_value)
{
	// Extract 12-bit value (bits 15-4)
	int16_t temp12bit = (reg_value >> 4) & 0x0FFF;
	
	// Convert to signed 12-bit (two's complement)
	if (temp12bit & 0x0800) {
		temp12bit -= 0x1000;
	}
	
	// Convert to Celsius (0.0625°C per LSB)
	return temp12bit * 0.0625;
}

/*
 * I2C transaction hook - handles START, STOP, and data transactions
 */
static void
tmp105_virt_in_hook(
		struct avr_irq_t * irq,
		uint32_t value,
		void * param)
{
	tmp105_virt_t * p = (tmp105_virt_t*)param;
	avr_twi_msg_irq_t v;
	v.u.v = value;

	/*
	 * If we receive a STOP, reset the transaction
	 */
	if (v.u.twi.msg & TWI_COND_STOP) {
		if (p->selected) {
			if (p->verbose)
				printf("tmp105 received stop\n");
		}
		p->selected = 0;
		p->reg_selected = 0; // Reset register selection
		p->reg_byte_pos = 0; // Reset byte position
	}
	
	/*
	 * If we receive a START, check if the slave address is meant for us,
	 * and if so reply with an ACK bit
	 */
	if (v.u.twi.msg & TWI_COND_START) {
		p->selected = 0;
		if ((v.u.twi.addr >> 1) == TMP105_VIRT_TWI_ADDR) {
			// It's us!
			if (p->verbose)
				printf("tmp105 received start (addr=0x%02X)\n", v.u.twi.addr);
			p->selected = v.u.twi.addr;
			// Send ACK to allow transaction to continue
			avr_raise_irq(p->irq + TMP105_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_ACK, p->selected, 1));
		}
	}
	
	/*
	 * If it's a data transaction and we're selected, handle it
	 */
	if (p->selected) {
		if (v.u.twi.msg & TWI_COND_WRITE) {
			// Write transaction
			avr_raise_irq(p->irq + TMP105_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_ACK, p->selected, 1));
			
			if (p->verbose)
				printf("tmp105 WRITE data 0x%02X\n", v.u.twi.data);
			
			// First byte sets register pointer
			if (!p->reg_selected) {
				// This is the register pointer
				p->reg_addr = v.u.twi.data & 0x03; // Only 4 registers (0-3)
				p->reg_selected = 1;
				p->reg_byte_pos = 0; // Reset byte position
				if (p->verbose)
					printf("tmp105 register pointer set to 0x%02X\n", p->reg_addr);
			} else {
				// Write to the selected register
				switch (p->reg_addr) {
					case TMP105_VIRT_TEMPERATURE:
						// Temperature register is read-only, ignore write
						if (p->verbose)
							printf("tmp105: Temperature register is read-only\n");
						break;
					
					case TMP105_VIRT_CONFIGURATION:
						// Configuration register (8-bit)
						p->configuration = v.u.twi.data;
						if (p->verbose)
							printf("tmp105: Configuration set to 0x%02X\n", p->configuration);
						p->reg_addr++; // Auto-increment
						p->reg_byte_pos = 0; // Reset byte position
						break;
					
					case TMP105_VIRT_T_LOW:
						// T_LOW register (16-bit, MSB first)
						if (p->reg_byte_pos == 0) {
							// First byte (MSB)
							p->t_low = (v.u.twi.data << 8);
							p->reg_byte_pos = 1;
						} else {
							// Second byte (LSB)
							p->t_low |= v.u.twi.data;
							if (p->verbose)
								printf("tmp105: T_LOW set to 0x%04X (%.2f°C)\n",
									p->t_low, tmp105_register_to_celsius(p->t_low));
							p->reg_byte_pos = 0; // Reset for next write
						}
						break;
					
					case TMP105_VIRT_T_HIGH:
						// T_HIGH register (16-bit, MSB first)
						if (p->reg_byte_pos == 0) {
							// First byte (MSB)
							p->t_high = (v.u.twi.data << 8);
							p->reg_byte_pos = 1;
						} else {
							// Second byte (LSB)
							p->t_high |= v.u.twi.data;
							if (p->verbose)
								printf("tmp105: T_HIGH set to 0x%04X (%.2f°C)\n",
									p->t_high, tmp105_register_to_celsius(p->t_high));
							p->reg_byte_pos = 0; // Reset for next write
						}
						break;
					
					default:
						if (p->verbose)
							printf("tmp105: Unknown register 0x%02X\n", p->reg_addr);
						break;
				}
			}
		}
		
		if (v.u.twi.msg & TWI_COND_READ) {
			// Read transaction
			uint8_t data = 0;
			
			// If register not selected, default to temperature (0x00)
			if (!p->reg_selected) {
				p->reg_addr = TMP105_VIRT_TEMPERATURE;
				p->reg_selected = 1;
				p->reg_byte_pos = 0;
			}
			
			switch (p->reg_addr) {
				case TMP105_VIRT_TEMPERATURE:
					// Temperature register (16-bit, MSB first)
					if (p->reg_byte_pos == 0) {
						// First byte (MSB)
						data = (p->temperature >> 8) & 0xFF;
						p->reg_byte_pos = 1;
					} else {
						// Second byte (LSB)
						data = p->temperature & 0xFF;
						p->reg_byte_pos = 0; // Reset for next read
					}
					if (p->verbose)
						printf("tmp105 READ temperature: 0x%02X (reg=0x%04X, %.2f°C)\n",
							data, p->temperature, tmp105_register_to_celsius(p->temperature));
					break;
				
				case TMP105_VIRT_CONFIGURATION:
					// Configuration register (8-bit)
					data = p->configuration;
					if (p->verbose)
						printf("tmp105 READ configuration: 0x%02X\n", data);
					p->reg_addr++; // Auto-increment
					p->reg_byte_pos = 0; // Reset byte position
					break;
				
				case TMP105_VIRT_T_LOW:
					// T_LOW register (16-bit, MSB first)
					if (p->reg_byte_pos == 0) {
						// First byte (MSB)
						data = (p->t_low >> 8) & 0xFF;
						p->reg_byte_pos = 1;
					} else {
						// Second byte (LSB)
						data = p->t_low & 0xFF;
						p->reg_byte_pos = 0; // Reset for next read
					}
					if (p->verbose)
						printf("tmp105 READ T_LOW: 0x%02X\n", data);
					break;
				
				case TMP105_VIRT_T_HIGH:
					// T_HIGH register (16-bit, MSB first)
					if (p->reg_byte_pos == 0) {
						// First byte (MSB)
						data = (p->t_high >> 8) & 0xFF;
						p->reg_byte_pos = 1;
					} else {
						// Second byte (LSB)
						data = p->t_high & 0xFF;
						p->reg_byte_pos = 0; // Reset for next read
					}
					if (p->verbose)
						printf("tmp105 READ T_HIGH: 0x%02X\n", data);
					break;
				
				default:
					if (p->verbose)
						printf("tmp105 READ unknown register 0x%02X\n", p->reg_addr);
					data = 0;
					break;
			}
			
			avr_raise_irq(p->irq + TMP105_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_READ, p->selected, data));
		}
	}
}

void
tmp105_virt_init(
		struct avr_t * avr,
		tmp105_virt_t * p)
{
	memset(p, 0, sizeof(*p));

	p->avr = avr;
	p->irq = avr_alloc_irq(&avr->irq_pool, 0, TMP105_IRQ_COUNT, _tmp105_irq_names);
	avr_irq_register_notify(p->irq + TMP105_TWI_IRQ_OUTPUT, tmp105_virt_in_hook, p);
	
	// Initialize registers with default values
	// Default temperature: 26.0°C (0x1A00)
	p->temperature = tmp105_celsius_to_register(26.0f);
	// Default T_LOW: 20.0°C (0x1400)
	p->t_low = tmp105_celsius_to_register(20.0f);
	// Default T_HIGH: 30.0°C (0x1E00)
	p->t_high = tmp105_celsius_to_register(30.0f);
	// Default configuration: 0x00
	p->configuration = 0x00;
	
	p->reg_addr = 0;
	p->reg_selected = 0;
	p->reg_byte_pos = 0;
	p->verbose = 0;
}

void
tmp105_virt_attach_twi(tmp105_virt_t * p,
                       uint32_t i2c_irq_base)
{
	avr_connect_irq(
		p->irq + TMP105_TWI_IRQ_OUTPUT,
		avr_io_getirq(p->avr, i2c_irq_base, TWI_IRQ_INPUT));
	avr_connect_irq(
		avr_io_getirq(p->avr, i2c_irq_base, TWI_IRQ_OUTPUT),
		p->irq + TMP105_TWI_IRQ_INPUT);
}

void
tmp105_virt_set_temperature(tmp105_virt_t * p, float celsius)
{
	p->temperature = tmp105_celsius_to_register(celsius);
	if (p->verbose)
		printf("tmp105: Temperature set to %.2f°C (0x%04X)\n",
			celsius, p->temperature);
}

