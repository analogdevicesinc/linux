/*
* Analog Devices AD-FMCDAQ1-EBZ SPI-GPIO expander driver
*
* Copyright 2016 Analog Devices Inc.
* Author: Dragos Bogdan <dragos.bogdan@analog.com>
*
* Licensed under the GPL-2.
*/

#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/module.h>

#define REG_ADC_CONTROL		0x10
#define REG_DAC_CONTROL		0x11
#define REG_CLK_CONTROL		0x12
#define REG_ADC_STATUS		0x20
#define REG_DAC_STATUS		0x21
#define REG_CLK_STATUS		0x22

/* REG_ADC_CONTROL */
#define ADC_CTRL_PWDN_STBY	(1 << 0)

/* REG_DAC_CONTROL */
#define DAC_CTRL_RESETN		(1 << 0)

/* REG_CLK_CONTROL */
#define CLK_CTRL_SYNCN		(1 << 2)
#define CLK_CTRL_RESETN		(1 << 1)
#define CLK_CTRL_PWDNN		(1 << 0)

/* REG_ADC_STATUS */
#define ADC_ST_STATUS		(1 << 2)
#define ADC_ST_FDB			(1 << 1)
#define ADC_ST_FDA			(1 << 0)

/* REG_DAC_STATUS */
#define DAC_ST_IRQN			(1 << 0)

/* REG_CLK_STATUS */
#define CLK_ST_STATUS_1		(1 << 1)
#define CLK_ST_STATUS_0		(1 << 0)

/* Available GPIOs */
#define IN_CLK_STATUS_1		13
#define IN_CLK_STATUS_0		12
#define IN_DAC_IRQN			11
#define IN_ADC_STATUS		10
#define IN_ADC_FDB			9
#define IN_ADC_FDA			8
#define OUT_CLK_SYNCN		4
#define OUT_CLK_RESETN		3
#define OUT_CLK_PWDNN		2
#define OUT_DAC_RESETN		1
#define OUT_ADC_PWDN_STBY	0

#define IN_OUT_NUM			16
#define IN_OFFSET			8

struct daq1_gpio {
	struct mutex lock;
	struct spi_device *spi;
	struct gpio_chip gpio_chip;
	u8 gpio_out;
	u8 gpio_in;
};

static int daq1_gpio_read(struct spi_device *spi,
						  u8 reg, u8 *val)
{
	int ret;

	reg |= 0x80;
	ret = spi_write_then_read(spi, &reg, 1, &val, 1);

	return ret;
}

static int daq1_gpio_write(struct spi_device *spi,
						   u8 reg, u8 val)
{
	u8 buf[2];
	int ret;

	buf[0] = reg;
	buf[1] = val;
	ret = spi_write_then_read(spi, buf, 2, NULL, 0);

	return ret;
}

static void daq1_gpio_update(struct daq1_gpio *daq1)
{
	u8 val;

	/* Update output GPIOs */
	val = (daq1->gpio_out >> OUT_ADC_PWDN_STBY) & ADC_CTRL_PWDN_STBY;
	daq1_gpio_write(daq1->spi, REG_ADC_CONTROL, val);

	val = (daq1->gpio_out >> OUT_DAC_RESETN) & DAC_CTRL_RESETN;
	daq1_gpio_write(daq1->spi, REG_DAC_CONTROL, val);

	val = (daq1->gpio_out >> OUT_CLK_PWDNN) &
				(CLK_CTRL_SYNCN | CLK_CTRL_RESETN | CLK_CTRL_PWDNN);
	daq1_gpio_write(daq1->spi, REG_CLK_CONTROL, val);

	/* Update input GPIOs */
	daq1_gpio_read(daq1->spi, REG_ADC_STATUS, &val);
	daq1->gpio_in &= ~((ADC_ST_STATUS | ADC_ST_FDB | ADC_ST_FDA)
						<< (IN_ADC_FDA - IN_OFFSET));
	daq1->gpio_in |= ((val & (ADC_ST_STATUS | ADC_ST_FDB | ADC_ST_FDA))
						<< (IN_ADC_FDA - IN_OFFSET));

	daq1_gpio_read(daq1->spi, REG_DAC_STATUS, &val);
	daq1->gpio_in &= ~(DAC_ST_IRQN << (IN_DAC_IRQN - IN_OFFSET));
	daq1->gpio_in |= ((val & DAC_ST_IRQN) << (IN_DAC_IRQN - IN_OFFSET));

	daq1_gpio_read(daq1->spi, REG_CLK_STATUS, &val);
	daq1->gpio_in &= ~((CLK_ST_STATUS_1 | CLK_ST_STATUS_0)
						<< (IN_CLK_STATUS_0 - IN_OFFSET));
	daq1->gpio_in |= ((val & (CLK_ST_STATUS_1 | CLK_ST_STATUS_0))
						<< (IN_CLK_STATUS_0 - IN_OFFSET));
}

static void daq1_gpio_set_value(struct gpio_chip *chip,
								unsigned offset, int val)
{
	struct daq1_gpio *daq1 = container_of(chip, struct daq1_gpio, gpio_chip);

	if (offset >= IN_OFFSET)
		return;		// This GPIO is an input

	mutex_lock(&daq1->lock);
	if (val)
		daq1->gpio_out |= (1 << offset);
	else
		daq1->gpio_out &= ~(1 << offset);
	daq1_gpio_update(daq1);
	mutex_unlock(&daq1->lock);
}

static int daq1_gpio_get_value(struct gpio_chip *chip,
							   unsigned offset)
{
	struct daq1_gpio *daq1 = container_of(chip, struct daq1_gpio, gpio_chip);
	int ret;

	mutex_lock(&daq1->lock);
	daq1_gpio_update(daq1);
	if (offset < IN_OFFSET)
		ret = (daq1->gpio_out >> offset) & 0x1;
	else
		ret = (daq1->gpio_in >> (offset - 8)) & 0x1;
	mutex_unlock(&daq1->lock);

	return ret;
}

static int daq1_gpio_get_direction(struct gpio_chip *chip,
								   unsigned offset)
{
	if (offset < IN_OFFSET)
		return 0;	// This GPIO is an output
	else
		return 1;	// This GPIO is an input
}

static int daq1_gpio_direction_input(struct gpio_chip *chip,
									 unsigned offset)
{
	if (offset < IN_OFFSET)
		return -EINVAL;	// This GPIO is an output

	return 0;
}

static int daq1_gpio_direction_output(struct gpio_chip *chip,
									  unsigned offset, int val)
{
	if (offset >= IN_OFFSET)
		return -EINVAL;	// This GPIO is an input

	daq1_gpio_set_value(chip, offset, val);

	return 0;
}

static int daq1_gpio_probe(struct spi_device *spi)
{
	struct daq1_gpio *daq1;
	int ret;

	daq1 = devm_kzalloc(&spi->dev, sizeof(*daq1), GFP_KERNEL);
	if (!daq1)
		return -ENOMEM;

	spi_set_drvdata(spi, daq1);

	daq1->spi = spi;

	daq1->gpio_out = 0x00;
	daq1_gpio_update(daq1);

	daq1->gpio_chip.base = -1;
	daq1->gpio_chip.label = spi->modalias;
	daq1->gpio_chip.get_direction = daq1_gpio_get_direction;
	daq1->gpio_chip.direction_input = daq1_gpio_direction_input;
	daq1->gpio_chip.direction_output = daq1_gpio_direction_output;
	daq1->gpio_chip.get = daq1_gpio_get_value;
	daq1->gpio_chip.set = daq1_gpio_set_value;
	daq1->gpio_chip.ngpio = IN_OUT_NUM;
	daq1->gpio_chip.parent = &spi->dev;
	daq1->gpio_chip.owner = THIS_MODULE;

	mutex_init(&daq1->lock);

	ret = gpiochip_add(&daq1->gpio_chip);
	if (!ret) {
		dev_info(&spi->dev, "ADI-DAQ1-GPIO driver successfully initialized");
		return 0;
	}

	mutex_destroy(&daq1->lock);

	return ret;
}

static int daq1_gpio_remove(struct spi_device *spi)
{
	struct daq1_gpio *daq1 = spi_get_drvdata(spi);

	gpiochip_remove(&daq1->gpio_chip);
	mutex_destroy(&daq1->lock);

	return 0;
}

static const struct spi_device_id daq1_gpio_ids[] = {
	{ "gpio-adi-daq1", 0 },
	{ },
};

MODULE_DEVICE_TABLE(spi, daq1_gpio_ids);

static struct spi_driver gpio_adi_daq1_driver = {
	.driver = {
		.name	= "gpio-adi-daq1",
		.owner	= THIS_MODULE,
	},
	.id_table	= daq1_gpio_ids,
	.probe		= daq1_gpio_probe,
	.remove		= daq1_gpio_remove,
};
module_spi_driver(gpio_adi_daq1_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD-FMCDAQ1-EBZ SPI-GPIO expander driver");
