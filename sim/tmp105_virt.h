/*
	tmp105_virt.h

	Virtual TMP105 Temperature Sensor for SimAVR
	Digital temperature sensor with 12-bit resolution (0.0625°C per LSB)

	Based on ds1338_virt and ssd1306_virt examples
*/

#ifndef TMP105_VIRT_H_
#define TMP105_VIRT_H_

#include "sim_irq.h"
#include "sim_avr.h"

// TMP105 I2C address (7-bit)
#define TMP105_VIRT_TWI_ADDR		0x48

// Register addresses
#define TMP105_VIRT_TEMPERATURE		0x00
#define TMP105_VIRT_CONFIGURATION	0x01
#define TMP105_VIRT_T_LOW		0x02
#define TMP105_VIRT_T_HIGH		0x03

enum {
	TMP105_TWI_IRQ_OUTPUT = 0,
	TMP105_TWI_IRQ_INPUT,
	TMP105_IRQ_COUNT
};

/*
 * TMP105 virtual device structure
 */
typedef struct tmp105_virt_t {
	struct avr_t * avr;
	avr_irq_t * irq;		// irq list
	uint8_t verbose;
	uint8_t selected;		// selected address (0 if not addressed)
	uint8_t reg_selected;		// register pointer has been set
	uint8_t reg_addr;		// current register pointer
	uint8_t reg_byte_pos;		// byte position for 16-bit registers (0=MSB, 1=LSB)
	
	// TMP105 registers
	uint16_t temperature;		// Temperature register (12-bit, 0.0625°C per LSB)
	uint8_t configuration;		// Configuration register
	uint16_t t_low;			// Low temperature threshold
	uint16_t t_high;		// High temperature threshold
} tmp105_virt_t;

void
tmp105_virt_init(struct avr_t * avr,
                 tmp105_virt_t * p);

/*
 * Attach the TMP105 to the AVR's TWI master code,
 * pass AVR_IOCTL_TWI_GETIRQ(0) for example as i2c_irq_base
 */
void
tmp105_virt_attach_twi(tmp105_virt_t * p,
                       uint32_t i2c_irq_base);

/*
 * Set temperature in Celsius
 * @param celsius - Temperature in Celsius (will be converted to 12-bit format)
 */
void
tmp105_virt_set_temperature(tmp105_virt_t * p, float celsius);

#endif /* TMP105_VIRT_H_ */

