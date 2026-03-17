/*
	adxl345_virt.h

	Virtual ADXL345 3-Axis Accelerometer for SimAVR
	Digital accelerometer with 13-bit resolution

	Based on tmp105_virt
*/

#ifndef ADXL345_VIRT_H_
#define ADXL345_VIRT_H_

#include "sim_irq.h"
#include "sim_avr.h"

// ADXL345 I2C address (7-bit)
#define ADXL345_VIRT_TWI_ADDR		0x53

// Register addresses
#define ADXL345_VIRT_DEVID		0x00
#define ADXL345_VIRT_THRESH_TAP		0x1D
#define ADXL345_VIRT_OFSX		0x1E
#define ADXL345_VIRT_OFSY		0x1F
#define ADXL345_VIRT_OFSZ		0x20
#define ADXL345_VIRT_BW_RATE		0x2C
#define ADXL345_VIRT_POWER_CTL		0x2D
#define ADXL345_VIRT_INT_ENABLE		0x2E
#define ADXL345_VIRT_INT_MAP		0x2F
#define ADXL345_VIRT_INT_SOURCE		0x30
#define ADXL345_VIRT_DATA_FORMAT	0x31
#define ADXL345_VIRT_DATAX0		0x32
#define ADXL345_VIRT_DATAX1		0x33
#define ADXL345_VIRT_DATAY0		0x34
#define ADXL345_VIRT_DATAY1		0x35
#define ADXL345_VIRT_DATAZ0		0x36
#define ADXL345_VIRT_DATAZ1		0x37
#define ADXL345_VIRT_FIFO_CTL		0x38
#define ADXL345_VIRT_FIFO_STATUS	0x39

// Device ID value
#define ADXL345_VIRT_DEVID_VAL		0xE5

enum {
	ADXL345_TWI_IRQ_OUTPUT = 0,
	ADXL345_TWI_IRQ_INPUT,
	ADXL345_IRQ_COUNT
};

/*
 * ADXL345 virtual device structure
 */
typedef struct adxl345_virt_t {
	struct avr_t * avr;
	avr_irq_t * irq;		// irq list
	uint8_t verbose;
	uint8_t selected;		// selected address (0 if not addressed)
	uint8_t reg_selected;		// register pointer has been set
	uint8_t reg_addr;		// current register pointer
	uint8_t reg_byte_pos;		// byte position for multi-byte reads

	// ADXL345 registers
	uint8_t power_ctl;		// Power control register
	uint8_t data_format;		// Data format register
	uint8_t bw_rate;		// Bandwidth rate register
	uint8_t int_enable;		// Interrupt enable register

	// Acceleration data (16-bit signed, LSB = 3.9mg in default +-2g range)
	int16_t accel_x;		// X-axis raw value
	int16_t accel_y;		// Y-axis raw value
	int16_t accel_z;		// Z-axis raw value
} adxl345_virt_t;

void
adxl345_virt_init(struct avr_t * avr,
                  adxl345_virt_t * p);

/*
 * Attach the ADXL345 to the AVR's TWI master code,
 * pass AVR_IOCTL_TWI_GETIRQ(0) for example as i2c_irq_base
 */
void
adxl345_virt_attach_twi(adxl345_virt_t * p,
                        uint32_t i2c_irq_base);

/*
 * Set X-axis acceleration in g
 */
void
adxl345_virt_set_accel_x(adxl345_virt_t * p, float g);

/*
 * Set Y-axis acceleration in g
 */
void
adxl345_virt_set_accel_y(adxl345_virt_t * p, float g);

/*
 * Set Z-axis acceleration in g
 */
void
adxl345_virt_set_accel_z(adxl345_virt_t * p, float g);

#endif /* ADXL345_VIRT_H_ */
