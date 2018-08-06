#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>

#include "adi_hal.h"

#ifndef IIO_TALISE_LINUX_HAL_H_
#define IIO_TALISE_LINUX_HAL_H_

struct adrv9009_hal {
	struct spi_device 	*spi;
	struct gpio_desc	*reset_gpio;
	unsigned int 		logLevel;
};

#endif /* IIO_TALISE_LINUX_HAL_H_ */
