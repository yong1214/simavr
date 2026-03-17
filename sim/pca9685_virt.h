/*
	pca9685_virt.h

	Virtual PCA9685 16-Channel PWM Driver for SimAVR
	I2C-bus controlled 16-channel LED controller with PWM output

	Based on tmp105_virt and ds1338_virt examples
*/

#ifndef PCA9685_VIRT_H_
#define PCA9685_VIRT_H_

#include "sim_irq.h"
#include "sim_avr.h"

/* PCA9685 I2C address (7-bit) */
#define PCA9685_VIRT_TWI_ADDR		0x41

/* MODE1 bit definitions */
#define PCA9685_MODE1_AI		0x20	/* Auto-Increment (bit 5) */

/* PCA9685 register defaults */
#define PCA9685_MODE1_DEFAULT		0x11	/* SLEEP=1, ALLCALL=1 */
#define PCA9685_MODE2_DEFAULT		0x04	/* OUTDRV=1 */
#define PCA9685_PRE_SCALE_DEFAULT	0x1E	/* ~200Hz at 25MHz internal osc */

enum {
	PCA9685_TWI_IRQ_OUTPUT = 0,
	PCA9685_TWI_IRQ_INPUT,
	PCA9685_IRQ_COUNT
};

/*
 * PCA9685 virtual device structure
 */
typedef struct pca9685_virt_t {
	struct avr_t * avr;
	avr_irq_t * irq;		/* irq list */
	uint8_t verbose;
	uint8_t selected;		/* selected address (0 if not addressed) */
	uint8_t reg_selected;		/* register pointer has been set */
	uint8_t reg_addr;		/* current register pointer */
	uint8_t registers[256];		/* full register space */
} pca9685_virt_t;

void
pca9685_virt_init(struct avr_t * avr,
                  pca9685_virt_t * p);

/*
 * Attach the PCA9685 to the AVR's TWI master code,
 * pass AVR_IOCTL_TWI_GETIRQ(0) for example as i2c_irq_base
 */
void
pca9685_virt_attach_twi(pca9685_virt_t * p,
                         uint32_t i2c_irq_base);

#endif /* PCA9685_VIRT_H_ */
