// SPDX-License-Identifier: GPL-2.0+
/*
 * ADG1408 SPI MUX driver
 *
 * Copyright 2018 Analog Devices Inc.
 *
 */

#include <linux/err.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/mux/driver.h>
#include <linux/spi/spi.h>
#include <linux/property.h>

#define ADGS1408_SW_DATA       (0x01)
#define ADGS1408_REG_READ(reg) ((reg) | 0x80)
#define ADGS1408_DISABLE       (0x00)
#define ADGS1408_MUX(state)    (((state) << 1) | 1)

static int adgs140x_spi_reg_write(struct spi_device *spi,
				u8 reg_addr, u8 reg_data)
{
	u8 tx_buf[2];

	tx_buf[0] = reg_addr;
	tx_buf[1] = reg_data;

	return spi_write_then_read(spi, tx_buf, sizeof(tx_buf), NULL, 0);
}

static int adgs140x_set(struct mux_control *mux, int state)
{
	struct spi_device *spi = to_spi_device(mux->chip->dev.parent);
	u8 reg;

	if (state == MUX_IDLE_DISCONNECT)
		reg = ADGS1408_DISABLE;
	else
		reg = ADGS1408_MUX(state);

	return adgs140x_spi_reg_write(spi, ADGS1408_SW_DATA, reg);
}

static const struct mux_control_ops adgs140x_ops = {
	.set = adgs140x_set,
};

static int adgs140x_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mux_chip *mux_chip;
	u32 idle_state[2];
	u32 cells;
	int ret;
	int i;

	ret = device_property_read_u32(dev, "#mux-control-cells", &cells);
	if (ret < 0)
		return ret;

	if (cells >= 2)
		return -EINVAL;

	mux_chip = devm_mux_chip_alloc(dev, cells ? 2 : 1, 0);
	if (IS_ERR(mux_chip))
		return PTR_ERR(mux_chip);

	mux_chip->ops = &adgs140x_ops;

	ret = adgs140x_spi_reg_write(spi, ADGS1408_SW_DATA, ADGS1408_DISABLE);
	if (ret < 0)
		return ret;

	ret = device_property_read_u32_array(dev, "idle-state",
					     idle_state,
					     mux_chip->controllers);
	if (ret < 0) {
		idle_state[0] = MUX_IDLE_AS_IS;
		idle_state[1] = MUX_IDLE_AS_IS;
	}

	for (i = 0; i < mux_chip->controllers; ++i) {
		struct mux_control *mux = &mux_chip->mux[i];

		if (mux_chip->controllers == 1)
			mux->states = 8;
		else
			mux->states = 4;

		switch (idle_state[i]) {
		case MUX_IDLE_DISCONNECT:
		case MUX_IDLE_AS_IS:
		case 0 ... 8:
			mux->idle_state = idle_state[i];
			break;
		default:
			dev_err(dev, "invalid idle-state %d\n", idle_state[i]);
			return -EINVAL;
		}
	}

	return devm_mux_chip_register(dev, mux_chip);
}

static const struct spi_device_id adgs140x_id[] = {
	{ .name = "adgs1408", },
	{ .name = "adgs1409", },
	{ }
};
MODULE_DEVICE_TABLE(spi, adgs140x_id);

static const struct of_device_id adgs140x_of_match[] = {
	{ .compatible = "adi,adgs1408", },
	{ .compatible = "adi,adgs1409", },
	{ }
};
MODULE_DEVICE_TABLE(of, adgs140x_of_match);

static struct spi_driver adgs140x_driver = {
	.driver = {
		.name = "adgs1408",
		.of_match_table = of_match_ptr(adgs140x_of_match),
	},
	.probe = adgs140x_probe,
	.id_table = adgs140x_id,
};
module_spi_driver(adgs140x_driver);

MODULE_AUTHOR("Mircea Caprioru <mircea.caprioru@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADGS140x MUX driver");
MODULE_LICENSE("GPL v2");
