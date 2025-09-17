/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#ifndef GPIO_ADI_ADSP_PORT_H
#define GPIO_ADI_ADSP_PORT_H

#include <linux/gpio/driver.h>

/* Number of GPIOs per port instance */
#define ADSP_PORT_NGPIO 16

/* PORT memory layout */
#define ADSP_PORT_REG_FER			0x00
#define ADSP_PORT_REG_FER_SET		0x04
#define ADSP_PORT_REG_FER_CLEAR		0x08
#define ADSP_PORT_REG_DATA			0x0c
#define ADSP_PORT_REG_DATA_SET		0x10
#define ADSP_PORT_REG_DATA_CLEAR	0x14
#define ADSP_PORT_REG_DIR			0x18
#define ADSP_PORT_REG_DIR_SET		0x1c
#define ADSP_PORT_REG_DIR_CLEAR		0x20
#define ADSP_PORT_REG_INEN			0x24
#define ADSP_PORT_REG_INEN_SET		0x28
#define ADSP_PORT_REG_INEN_CLEAR	0x2c
#define ADSP_PORT_REG_PORT_MUX		0x30
#define ADSP_PORT_REG_DATA_TGL		0x34
#define ADSP_PORT_REG_POLAR			0x38
#define ADSP_PORT_REG_POLAR_SET		0x3c
#define ADSP_PORT_REG_POLAR_CLEAR	0x40
#define ADSP_PORT_REG_LOCK			0x44
#define ADSP_PORT_REG_TRIG_TGL		0x48

/*
 * One gpio instance per PORT instance in the hardware, provides the per-PORT
 * interface to the hardware. Referenced in GPIO and PINCTRL drivers
 */
struct adsp_gpio_port {
	struct device *dev;
	void __iomem *regs;
	struct gpio_chip gpio;
	struct irq_domain *irq_domain;
	u32 irq_offset;
	u32 open_drain;
	spinlock_t lock;
};

/* may need lock depending on register */
static inline u32 __adsp_gpio_readl(struct adsp_gpio_port *port,
				    size_t offset)
{
	return readl(port->regs + offset);
}

/* may need lock depending on register */
static inline void __adsp_gpio_writel(struct adsp_gpio_port *port, u32 val,
				      size_t offset)
{
	writel(val, port->regs + offset);
}

/* may need lock depending on register */
static inline u16 __adsp_gpio_readw(struct adsp_gpio_port *port,
				    size_t offset)
{
	return readw(port->regs + offset);
}

/* may need lock depending on register */
static inline void __adsp_gpio_writew(struct adsp_gpio_port *port, u16 val,
				      size_t offset)
{
	writew(val, port->regs + offset);
}

static inline struct adsp_gpio_port *to_adsp_gpio_port(struct gpio_chip
						       *chip)
{
	return container_of(chip, struct adsp_gpio_port, gpio);
}

int adsp_attach_pint_to_gpio(struct adsp_gpio_port *port);

#endif
