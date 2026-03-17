/*
	aht20_virt.h

	Virtual AHT20 Temperature and Humidity Sensor for SimAVR
	I2C temperature and humidity sensor with command-based interface

	Based on tmp105_virt
*/

#ifndef AHT20_VIRT_H_
#define AHT20_VIRT_H_

#include "sim_irq.h"
#include "sim_avr.h"

// AHT20 I2C address (7-bit)
#define AHT20_VIRT_TWI_ADDR		0x38

// Command bytes
#define AHT20_CMD_INITIALIZE		0xBE
#define AHT20_CMD_TRIGGER		0xAC
#define AHT20_CMD_SOFT_RESET		0xBA

// Status bits
#define AHT20_STATUS_BUSY		0x80
#define AHT20_STATUS_CALIBRATED		0x08

enum {
	AHT20_TWI_IRQ_OUTPUT = 0,
	AHT20_TWI_IRQ_INPUT,
	AHT20_IRQ_COUNT
};

/*
 * AHT20 virtual device structure
 */
typedef struct aht20_virt_t {
	struct avr_t * avr;
	avr_irq_t * irq;		// irq list
	uint8_t verbose;
	uint8_t selected;		// selected address (0 if not addressed)

	// AHT20 state
	uint8_t initialized;		// Sensor has been initialized
	uint8_t measurement_ready;	// Measurement data is available
	uint8_t write_byte_pos;		// Write buffer position (for multi-byte commands)
	uint8_t write_cmd;		// Current command being written
	uint8_t read_byte_pos;		// Read byte position (0-5)

	// Measurement data
	uint32_t humidity_raw;		// 20-bit humidity raw value
	uint32_t temperature_raw;	// 20-bit temperature raw value
} aht20_virt_t;

void
aht20_virt_init(struct avr_t * avr,
                aht20_virt_t * p);

/*
 * Attach the AHT20 to the AVR's TWI master code,
 * pass AVR_IOCTL_TWI_GETIRQ(0) for example as i2c_irq_base
 */
void
aht20_virt_attach_twi(aht20_virt_t * p,
                      uint32_t i2c_irq_base);

/*
 * Set temperature in Celsius (-50 to 150)
 */
void
aht20_virt_set_temperature(aht20_virt_t * p, float celsius);

/*
 * Set humidity in percent (0-100)
 */
void
aht20_virt_set_humidity(aht20_virt_t * p, float percent);

#endif /* AHT20_VIRT_H_ */
