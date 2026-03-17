/*
	aht20_virt.c

	Virtual AHT20 Temperature and Humidity Sensor for SimAVR
	I2C temperature and humidity sensor with command-based interface

	Based on tmp105_virt
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "avr_twi.h"
#include "aht20_virt.h"

static const char * _aht20_irq_names[2] = {
	[AHT20_TWI_IRQ_INPUT] = "8>aht20.out",
	[AHT20_TWI_IRQ_OUTPUT] = "32<aht20.in",
};

/*
 * Build the 6-byte read response:
 * [status, hum_h, hum_m, hum_l_temp_h, temp_m, temp_l]
 *
 * Humidity is 20-bit in bits [19:0] spread across bytes 1-3 (upper nibble of byte 3)
 * Temperature is 20-bit in bits [19:0] spread across bytes 3 (lower nibble)-5
 */
static uint8_t
aht20_virt_read_byte(aht20_virt_t * p, uint8_t pos)
{
	switch (pos) {
		case 0:
			// Status byte: bit 3 = calibrated, bit 7 = busy
			return AHT20_STATUS_CALIBRATED; // 0x08 = calibrated, not busy
		case 1:
			// Humidity [19:12]
			return (p->humidity_raw >> 12) & 0xFF;
		case 2:
			// Humidity [11:4]
			return (p->humidity_raw >> 4) & 0xFF;
		case 3:
			// Humidity [3:0] in upper nibble, Temperature [19:16] in lower nibble
			return ((p->humidity_raw & 0x0F) << 4) | ((p->temperature_raw >> 16) & 0x0F);
		case 4:
			// Temperature [15:8]
			return (p->temperature_raw >> 8) & 0xFF;
		case 5:
			// Temperature [7:0]
			return p->temperature_raw & 0xFF;
		default:
			return 0;
	}
}

/*
 * I2C transaction hook - handles START, STOP, and data transactions
 */
static void
aht20_virt_in_hook(
		struct avr_irq_t * irq,
		uint32_t value,
		void * param)
{
	aht20_virt_t * p = (aht20_virt_t*)param;
	avr_twi_msg_irq_t v;
	v.u.v = value;

	/*
	 * If we receive a STOP, reset the transaction
	 */
	if (v.u.twi.msg & TWI_COND_STOP) {
		if (p->selected) {
			if (p->verbose)
				printf("aht20 received stop\n");
		}
		p->selected = 0;
		p->write_byte_pos = 0;
		p->read_byte_pos = 0;
	}

	/*
	 * If we receive a START, check if the slave address is meant for us,
	 * and if so reply with an ACK bit
	 */
	if (v.u.twi.msg & TWI_COND_START) {
		p->selected = 0;
		if ((v.u.twi.addr >> 1) == AHT20_VIRT_TWI_ADDR) {
			if (p->verbose)
				printf("aht20 received start (addr=0x%02X)\n", v.u.twi.addr);
			p->selected = v.u.twi.addr;
			p->read_byte_pos = 0;
			p->write_byte_pos = 0;
			avr_raise_irq(p->irq + AHT20_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_ACK, p->selected, 1));
		}
	}

	/*
	 * If it's a data transaction and we're selected, handle it
	 */
	if (p->selected) {
		if (v.u.twi.msg & TWI_COND_WRITE) {
			avr_raise_irq(p->irq + AHT20_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_ACK, p->selected, 1));

			if (p->verbose)
				printf("aht20 WRITE data 0x%02X (pos=%d)\n",
					v.u.twi.data, p->write_byte_pos);

			if (p->write_byte_pos == 0) {
				// First byte is the command
				p->write_cmd = v.u.twi.data;
				p->write_byte_pos = 1;

				switch (p->write_cmd) {
					case AHT20_CMD_SOFT_RESET:
						if (p->verbose)
							printf("aht20: soft reset\n");
						p->initialized = 0;
						p->measurement_ready = 0;
						break;
					default:
						// Multi-byte commands, wait for more bytes
						break;
				}
			} else {
				// Subsequent bytes for multi-byte commands
				p->write_byte_pos++;

				if (p->write_cmd == AHT20_CMD_INITIALIZE) {
					// Initialize command: 0xBE, 0x08, 0x00
					if (p->write_byte_pos >= 3) {
						p->initialized = 1;
						if (p->verbose)
							printf("aht20: initialized\n");
						p->write_byte_pos = 0;
					}
				} else if (p->write_cmd == AHT20_CMD_TRIGGER) {
					// Trigger measurement: 0xAC, 0x33, 0x00
					if (p->write_byte_pos >= 3) {
						p->measurement_ready = 1;
						if (p->verbose)
							printf("aht20: measurement triggered\n");
						p->write_byte_pos = 0;
					}
				}
			}
		}

		if (v.u.twi.msg & TWI_COND_READ) {
			uint8_t data = aht20_virt_read_byte(p, p->read_byte_pos);

			if (p->verbose)
				printf("aht20 READ byte %d: 0x%02X\n", p->read_byte_pos, data);

			// Advance read position (wraps after 6 bytes)
			p->read_byte_pos++;
			if (p->read_byte_pos > 5)
				p->read_byte_pos = 0;

			avr_raise_irq(p->irq + AHT20_TWI_IRQ_INPUT,
					avr_twi_irq_msg(TWI_COND_READ, p->selected, data));
		}
	}
}

void
aht20_virt_init(
		struct avr_t * avr,
		aht20_virt_t * p)
{
	memset(p, 0, sizeof(*p));

	p->avr = avr;
	p->irq = avr_alloc_irq(&avr->irq_pool, 0, AHT20_IRQ_COUNT, _aht20_irq_names);
	avr_irq_register_notify(p->irq + AHT20_TWI_IRQ_OUTPUT, aht20_virt_in_hook, p);

	// Initialize with default values
	p->initialized = 0;
	p->measurement_ready = 0;
	p->write_byte_pos = 0;
	p->read_byte_pos = 0;

	// Default: 25.0°C, 50% humidity
	aht20_virt_set_temperature(p, 25.0f);
	aht20_virt_set_humidity(p, 50.0f);

	p->verbose = 0;
}

void
aht20_virt_attach_twi(aht20_virt_t * p,
                      uint32_t i2c_irq_base)
{
	avr_connect_irq(
		p->irq + AHT20_TWI_IRQ_OUTPUT,
		avr_io_getirq(p->avr, i2c_irq_base, TWI_IRQ_INPUT));
	avr_connect_irq(
		avr_io_getirq(p->avr, i2c_irq_base, TWI_IRQ_OUTPUT),
		p->irq + AHT20_TWI_IRQ_INPUT);
}

void
aht20_virt_set_temperature(aht20_virt_t * p, float celsius)
{
	// AHT20 formula: raw = (temp + 50) / 200 * 2^20
	p->temperature_raw = (uint32_t)((celsius + 50.0f) / 200.0f * 1048576.0f);
	if (p->verbose)
		printf("aht20: Temperature set to %.2f°C (raw=0x%05X)\n",
			celsius, p->temperature_raw);
}

void
aht20_virt_set_humidity(aht20_virt_t * p, float percent)
{
	// AHT20 formula: raw = hum / 100 * 2^20
	p->humidity_raw = (uint32_t)(percent / 100.0f * 1048576.0f);
	if (p->verbose)
		printf("aht20: Humidity set to %.2f%% (raw=0x%05X)\n",
			percent, p->humidity_raw);
}
