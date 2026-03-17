/*
	bme280_virt.h

	Virtual BME280 Environmental Sensor for SimAVR
	Combined temperature, pressure, and humidity sensor

	Based on tmp105_virt
*/

#ifndef BME280_VIRT_H_
#define BME280_VIRT_H_

#include "sim_irq.h"
#include "sim_avr.h"

// BME280 I2C address (7-bit)
#define BME280_VIRT_TWI_ADDR		0x76

// Register addresses
#define BME280_VIRT_CHIP_ID		0xD0
#define BME280_VIRT_RESET		0xE0
#define BME280_VIRT_CTRL_HUM		0xF2
#define BME280_VIRT_STATUS		0xF3
#define BME280_VIRT_CTRL_MEAS		0xF4
#define BME280_VIRT_CONFIG		0xF5
#define BME280_VIRT_PRESS_MSB		0xF7
#define BME280_VIRT_PRESS_LSB		0xF8
#define BME280_VIRT_PRESS_XLSB		0xF9
#define BME280_VIRT_TEMP_MSB		0xFA
#define BME280_VIRT_TEMP_LSB		0xFB
#define BME280_VIRT_TEMP_XLSB		0xFC
#define BME280_VIRT_HUM_MSB		0xFD
#define BME280_VIRT_HUM_LSB		0xFE

// Chip ID value
#define BME280_VIRT_CHIP_ID_VAL		0x60

enum {
	BME280_TWI_IRQ_OUTPUT = 0,
	BME280_TWI_IRQ_INPUT,
	BME280_IRQ_COUNT
};

/*
 * BME280 virtual device structure
 */
typedef struct bme280_virt_t {
	struct avr_t * avr;
	avr_irq_t * irq;		// irq list
	uint8_t verbose;
	uint8_t selected;		// selected address (0 if not addressed)
	uint8_t reg_selected;		// register pointer has been set
	uint8_t reg_addr;		// current register pointer
	uint8_t reg_byte_pos;		// byte position within current register

	// BME280 registers
	uint8_t ctrl_hum;		// Humidity control register
	uint8_t ctrl_meas;		// Measurement control register
	uint8_t config;			// Configuration register

	// Raw measurement data (20-bit pressure, 20-bit temperature, 16-bit humidity)
	uint32_t pressure_raw;		// 20-bit pressure raw value
	uint32_t temperature_raw;	// 20-bit temperature raw value
	uint16_t humidity_raw;		// 16-bit humidity raw value
} bme280_virt_t;

void
bme280_virt_init(struct avr_t * avr,
                 bme280_virt_t * p);

/*
 * Attach the BME280 to the AVR's TWI master code,
 * pass AVR_IOCTL_TWI_GETIRQ(0) for example as i2c_irq_base
 */
void
bme280_virt_attach_twi(bme280_virt_t * p,
                       uint32_t i2c_irq_base);

/*
 * Set temperature in Celsius
 */
void
bme280_virt_set_temperature(bme280_virt_t * p, float celsius);

/*
 * Set pressure in Pascals
 */
void
bme280_virt_set_pressure(bme280_virt_t * p, float pascals);

/*
 * Set humidity in percent (0-100)
 */
void
bme280_virt_set_humidity(bme280_virt_t * p, float percent);

#endif /* BME280_VIRT_H_ */
