/*
	ina219_virt.h

	Virtual INA219 Current/Power Monitor for SimAVR
	High-side current/power monitor with I2C interface

	Based on tmp105_virt
*/

#ifndef INA219_VIRT_H_
#define INA219_VIRT_H_

#include "sim_irq.h"
#include "sim_avr.h"

// INA219 I2C address (7-bit)
#define INA219_VIRT_TWI_ADDR		0x40

// Register addresses
#define INA219_VIRT_CONFIG		0x00
#define INA219_VIRT_SHUNT_VOLTAGE	0x01
#define INA219_VIRT_BUS_VOLTAGE		0x02
#define INA219_VIRT_POWER		0x03
#define INA219_VIRT_CURRENT		0x04
#define INA219_VIRT_CALIBRATION		0x05

// Default configuration value
#define INA219_VIRT_CONFIG_DEFAULT	0x399F

enum {
	INA219_TWI_IRQ_OUTPUT = 0,
	INA219_TWI_IRQ_INPUT,
	INA219_IRQ_COUNT
};

/*
 * INA219 virtual device structure
 */
typedef struct ina219_virt_t {
	struct avr_t * avr;
	avr_irq_t * irq;		// irq list
	uint8_t verbose;
	uint8_t selected;		// selected address (0 if not addressed)
	uint8_t reg_selected;		// register pointer has been set
	uint8_t reg_addr;		// current register pointer
	uint8_t reg_byte_pos;		// byte position for 16-bit registers (0=MSB, 1=LSB)
	uint8_t write_byte_pos;		// byte position for 16-bit writes (0=MSB, 1=LSB)

	// INA219 registers (all 16-bit)
	uint16_t config;		// Configuration register
	int16_t shunt_voltage;		// Shunt voltage (LSB = 10uV)
	uint16_t bus_voltage;		// Bus voltage (LSB = 4mV, bits 15-3)
	uint16_t power;			// Power register (LSB = 20mW)
	int16_t current;		// Current register (LSB = 1mA)
	uint16_t calibration;		// Calibration register

	// Temp storage for 16-bit writes
	uint8_t write_msb;		// MSB of 16-bit write
} ina219_virt_t;

void
ina219_virt_init(struct avr_t * avr,
                 ina219_virt_t * p);

/*
 * Attach the INA219 to the AVR's TWI master code,
 * pass AVR_IOCTL_TWI_GETIRQ(0) for example as i2c_irq_base
 */
void
ina219_virt_attach_twi(ina219_virt_t * p,
                       uint32_t i2c_irq_base);

/*
 * Set bus voltage in Volts
 */
void
ina219_virt_set_bus_voltage(ina219_virt_t * p, float volts);

/*
 * Set shunt voltage in Volts
 */
void
ina219_virt_set_shunt_voltage(ina219_virt_t * p, float volts);

/*
 * Set current in Amps
 */
void
ina219_virt_set_current(ina219_virt_t * p, float amps);

/*
 * Set power in Watts
 */
void
ina219_virt_set_power(ina219_virt_t * p, float watts);

#endif /* INA219_VIRT_H_ */
