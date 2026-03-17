/*
	bme280_virt.c

	Virtual BME280 Environmental Sensor for SimAVR
	Combined temperature, pressure, and humidity sensor

	Based on tmp105_virt
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "avr_twi.h"
#include "bme280_virt.h"

static const char * _bme280_irq_names[2] = {
	[BME280_TWI_IRQ_INPUT] = "8>bme280.out",
	[BME280_TWI_IRQ_OUTPUT] = "32<bme280.in",
};

/*
 * Read a single register byte at the current register address
 * Auto-increments the register address after each read
 */
static uint8_t
bme280_virt_read_reg(bme280_virt_t * p)
{
	uint8_t data = 0;

	switch (p->reg_addr) {
		case BME280_VIRT_CHIP_ID:
			data = BME280_VIRT_CHIP_ID_VAL;
			break;
		case BME280_VIRT_CTRL_HUM:
			data = p->ctrl_hum;
			break;
		case BME280_VIRT_STATUS:
			data = 0x00; // Not busy, no update in progress
			break;
		case BME280_VIRT_CTRL_MEAS:
			data = p->ctrl_meas;
			break;
		case BME280_VIRT_CONFIG:
			data = p->config;
			break;
		case BME280_VIRT_PRESS_MSB:
			data = (p->pressure_raw >> 12) & 0xFF;
			break;
		case BME280_VIRT_PRESS_LSB:
			data = (p->pressure_raw >> 4) & 0xFF;
			break;
		case BME280_VIRT_PRESS_XLSB:
			data = (p->pressure_raw << 4) & 0xF0;
			break;
		case BME280_VIRT_TEMP_MSB:
			data = (p->temperature_raw >> 12) & 0xFF;
			break;
		case BME280_VIRT_TEMP_LSB:
			data = (p->temperature_raw >> 4) & 0xFF;
			break;
		case BME280_VIRT_TEMP_XLSB:
			data = (p->temperature_raw << 4) & 0xF0;
			break;
		case BME280_VIRT_HUM_MSB:
			data = (p->humidity_raw >> 8) & 0xFF;
			break;
		case BME280_VIRT_HUM_LSB:
			data = p->humidity_raw & 0xFF;
			break;
		default:
			if (p->verbose)
				printf("bme280 READ unknown register 0x%02X\n", p->reg_addr);
			data = 0;
			break;
	}

	if (p->verbose)
		printf("bme280 READ reg 0x%02X = 0x%02X\n", p->reg_addr, data);

	// Auto-increment register address
	p->reg_addr++;

	return data;
}

/*
 * I2C transaction hook - handles START, STOP, and data transactions
 */
static void
bme280_virt_in_hook(
		struct avr_irq_t * irq,
		uint32_t value,
		void * param)
{
	bme280_virt_t * p = (bme280_virt_t*)param;
	avr_twi_msg_irq_t v;
	v.u.v = value;

	/*
	 * If we receive a STOP, reset the transaction
	 */
	if (v.u.twi.msg & TWI_COND_STOP) {
		if (p->selected) {
			if (p->verbose)
				printf("bme280 received stop\n");
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
		if ((v.u.twi.addr >> 1) == BME280_VIRT_TWI_ADDR) {
			if (p->verbose)
				printf("bme280 received start (addr=0x%02X)\n", v.u.twi.addr);
			p->selected = v.u.twi.addr;
			avr_raise_irq(p->irq + BME280_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_ACK, p->selected, 1));
		}
	}

	/*
	 * If it's a data transaction and we're selected, handle it
	 */
	if (p->selected) {
		if (v.u.twi.msg & TWI_COND_WRITE) {
			avr_raise_irq(p->irq + BME280_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_ACK, p->selected, 1));

			if (p->verbose)
				printf("bme280 WRITE data 0x%02X\n", v.u.twi.data);

			if (!p->reg_selected) {
				// First byte sets register pointer
				p->reg_addr = v.u.twi.data;
				p->reg_selected = 1;
				p->reg_byte_pos = 0;
				if (p->verbose)
					printf("bme280 register pointer set to 0x%02X\n", p->reg_addr);
			} else {
				// Write to the selected register
				switch (p->reg_addr) {
					case BME280_VIRT_CTRL_HUM:
						p->ctrl_hum = v.u.twi.data;
						if (p->verbose)
							printf("bme280: ctrl_hum set to 0x%02X\n", p->ctrl_hum);
						break;
					case BME280_VIRT_CTRL_MEAS:
						p->ctrl_meas = v.u.twi.data;
						if (p->verbose)
							printf("bme280: ctrl_meas set to 0x%02X\n", p->ctrl_meas);
						break;
					case BME280_VIRT_CONFIG:
						p->config = v.u.twi.data;
						if (p->verbose)
							printf("bme280: config set to 0x%02X\n", p->config);
						break;
					case BME280_VIRT_RESET:
						if (v.u.twi.data == 0xB6) {
							if (p->verbose)
								printf("bme280: soft reset\n");
							p->ctrl_hum = 0x00;
							p->ctrl_meas = 0x00;
							p->config = 0x00;
						}
						break;
					default:
						if (p->verbose)
							printf("bme280: write to unknown/read-only register 0x%02X\n", p->reg_addr);
						break;
				}
				p->reg_addr++;
				p->reg_byte_pos = 0;
			}
		}

		if (v.u.twi.msg & TWI_COND_READ) {
			// If register not selected, default to chip ID
			if (!p->reg_selected) {
				p->reg_addr = BME280_VIRT_CHIP_ID;
				p->reg_selected = 1;
			}

			uint8_t data = bme280_virt_read_reg(p);

			avr_raise_irq(p->irq + BME280_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_READ, p->selected, data));
		}
	}
}

void
bme280_virt_init(
		struct avr_t * avr,
		bme280_virt_t * p)
{
	memset(p, 0, sizeof(*p));

	p->avr = avr;
	p->irq = avr_alloc_irq(&avr->irq_pool, 0, BME280_IRQ_COUNT, _bme280_irq_names);
	avr_irq_register_notify(p->irq + BME280_TWI_IRQ_OUTPUT, bme280_virt_in_hook, p);

	// Initialize with default values
	p->ctrl_hum = 0x00;
	p->ctrl_meas = 0x00;
	p->config = 0x00;

	// Default: 25.0°C, 101325 Pa, 50% humidity
	bme280_virt_set_temperature(p, 25.0f);
	bme280_virt_set_pressure(p, 101325.0f);
	bme280_virt_set_humidity(p, 50.0f);

	p->reg_addr = 0;
	p->reg_selected = 0;
	p->reg_byte_pos = 0;
	p->verbose = 0;
}

void
bme280_virt_attach_twi(bme280_virt_t * p,
                       uint32_t i2c_irq_base)
{
	avr_connect_irq(
		p->irq + BME280_TWI_IRQ_OUTPUT,
		avr_io_getirq(p->avr, i2c_irq_base, TWI_IRQ_INPUT));
	avr_connect_irq(
		avr_io_getirq(p->avr, i2c_irq_base, TWI_IRQ_OUTPUT),
		p->irq + BME280_TWI_IRQ_INPUT);
}

void
bme280_virt_set_temperature(bme280_virt_t * p, float celsius)
{
	p->temperature_raw = (uint32_t)(celsius * 5120.0f);
	if (p->verbose)
		printf("bme280: Temperature set to %.2f°C (raw=0x%05X)\n",
			celsius, p->temperature_raw);
}

void
bme280_virt_set_pressure(bme280_virt_t * p, float pascals)
{
	p->pressure_raw = (uint32_t)(pascals / 100.0f * 256.0f);
	if (p->verbose)
		printf("bme280: Pressure set to %.2f Pa (raw=0x%05X)\n",
			pascals, p->pressure_raw);
}

void
bme280_virt_set_humidity(bme280_virt_t * p, float percent)
{
	p->humidity_raw = (uint16_t)(percent * 1024.0f);
	if (p->verbose)
		printf("bme280: Humidity set to %.2f%% (raw=0x%04X)\n",
			percent, p->humidity_raw);
}
