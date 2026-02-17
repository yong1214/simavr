/*
	ssd1306_virt.h

	Virtual SSD1306 OLED Display for SimAVR
	Minimal implementation that ACKs I2C transactions to allow data capture

	Based on ds1338_virt and i2c_eeprom examples
*/

#ifndef SSD1306_VIRT_H_
#define SSD1306_VIRT_H_

#include "sim_irq.h"
#include "sim_avr.h"

// SSD1306 I2C address (7-bit)
#define SSD1306_VIRT_TWI_ADDR		0x3C

// Display buffer size (128x64 = 1024 bytes)
#define SSD1306_DISPLAY_BUFFER_SIZE	1024

enum {
	SSD1306_TWI_IRQ_OUTPUT = 0,
	SSD1306_TWI_IRQ_INPUT,
	SSD1306_IRQ_COUNT
};

/*
 * SSD1306 virtual device structure
 */
typedef struct ssd1306_virt_t {
	struct avr_t * avr;
	avr_irq_t * irq;		// irq list
	uint8_t verbose;
	uint8_t selected;		// selected address (0 if not addressed)
	uint8_t command_mode;		// true if in command mode (0x00), false if data mode (0x40)
	uint8_t current_page;		// current page (0-7)
	uint8_t current_column;		// current column (0-127)
	uint8_t display_buffer[SSD1306_DISPLAY_BUFFER_SIZE]; // Display buffer
} ssd1306_virt_t;

void
ssd1306_virt_init(struct avr_t * avr,
                  ssd1306_virt_t * p);

/*
 * Attach the SSD1306 to the AVR's TWI master code,
 * pass AVR_IOCTL_TWI_GETIRQ(0) for example as i2c_irq_base
 */
void
ssd1306_virt_attach_twi(ssd1306_virt_t * p,
                        uint32_t i2c_irq_base);

#endif /* SSD1306_VIRT_H_ */
