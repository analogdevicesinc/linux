// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * Analog Devices MAX2035x MFD core driver
 *
 * Provides I2C, regmap and IRQ handling for MAX20355 / MAX20357
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/interrupt.h>

#include "max2035x.h"
#include "max2035x_registers.h"

static struct max2035x *slave_chips[2];
static struct max2035x *master_chip;
static DEFINE_MUTEX(chip_list_lock);

void max2035x_register_device(struct max2035x *chip)
{
    mutex_lock(&chip_list_lock);
    if (chip->type == MAX20355) {
		master_chip = chip;
    } else if (chip->channel_id >= 1 && chip->channel_id <= 2) {
		slave_chips[chip->channel_id - 1] = chip;
    }
    mutex_unlock(&chip_list_lock);
}
EXPORT_SYMBOL_GPL(max2035x_register_device);

void max2035x_unregister_device(struct max2035x *chip)
{
    mutex_lock(&chip_list_lock);
    if (chip->type == MAX20355) {
		if (master_chip == chip)
			master_chip = NULL;
    } else if (chip->channel_id >= 1 && chip->channel_id <= 2) {
		if (slave_chips[chip->channel_id - 1] == chip)
			slave_chips[chip->channel_id - 1] = NULL;
    }
    mutex_unlock(&chip_list_lock);
}
EXPORT_SYMBOL_GPL(max2035x_unregister_device);

struct max2035x *max2035x_get_device(enum max2035x_type type, int channel)
{
	struct max2035x *chip = NULL;

	mutex_lock(&chip_list_lock);

	if (type == MAX20355) {
		chip = master_chip;
	} else if (channel >= 1 && channel <= 2) {
		chip = slave_chips[channel - 1];
	}

	mutex_unlock(&chip_list_lock);

	return chip;
}
EXPORT_SYMBOL_GPL(max2035x_get_device);

/* -------------------------------------------------------------------------- */
/* Regmap configuration                                                       */
/* -------------------------------------------------------------------------- */
static const struct regmap_config max2035x_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
	/* max_register is set per variant in probe */
};

/* -------------------------------------------------------------------------- */
/* MFD cells                                                                  */
/* -------------------------------------------------------------------------- */
static const struct mfd_cell max20355_cells[] = {
	{ .name = "max20355-plc" },
	{ .name = "max20355-fuelgauge" },
	{ .name = "max20355-ram" },
};

static const struct mfd_cell max20357_cells[] = {
	{ .name = "max20357-plc" },
	{ .name = "max20357-fuelgauge" },
	{ .name = "max20357-ram" },
};

/* -------------------------------------------------------------------------- */
/* Device Tree match table                                                    */
/* -------------------------------------------------------------------------- */
static const struct of_device_id max2035x_of_match[] = {
	{ .compatible = "adi,max20355", .data = (void *)MAX20355 },
	{ .compatible = "adi,max20357", .data = (void *)MAX20357 },
	{ }
};
MODULE_DEVICE_TABLE(of, max2035x_of_match);

/* -------------------------------------------------------------------------- */
/* Threaded IRQ handler                                                       */
/* -------------------------------------------------------------------------- */
static irqreturn_t max2035x_irq_thread(int irq, void *data)
{
	struct max2035x *chip = data;
	u8 ints[6], masks[6], status[7];
	unsigned int reg_int, reg_mask, reg_status;
	int cnt_int, cnt_status;
	int ret;

	if (chip->type == MAX20355) {
		reg_int    = MAX20355_REG_INT0;
		reg_mask   = MAX20355_REG_INTMASK0;
		reg_status = MAX20355_REG_STATUS0;
		cnt_int    = 4;
		cnt_status = 4;
	} else {
		reg_int    = MAX20357_REG_INT0;
		reg_mask   = MAX20357_REG_INTMASK0;
		reg_status = MAX20357_REG_STATUS0;
		cnt_int    = 6;
		cnt_status = 7;
	}

	ret = regmap_bulk_read(chip->regmap, reg_int, ints, cnt_int);
	if (ret)
		return IRQ_NONE;

	ret = regmap_bulk_read(chip->regmap, reg_mask, masks, cnt_int);
	if (ret)
		return IRQ_NONE;

	ret = regmap_bulk_read(chip->regmap, reg_status, status, cnt_status);
	if (ret)
		return IRQ_NONE;


	if (chip->type == MAX20355) {
		dev_info(chip->dev, "%s: [MAX20355] INT dump         : %*ph\n", __func__, cnt_int, ints);
		dev_info(chip->dev, "%s: [MAX20355] MASK dump        : %*ph\n", __func__, cnt_int, masks);
		dev_info(chip->dev, "%s: [MAX20355] STATUS dump      : %*ph\n", __func__, cnt_status, status);
	} else {
		dev_info(chip->dev, "%s: [MAX20357_CH%u] INT dump    : %*ph\n", __func__, chip->channel_id, cnt_int, ints);
		dev_info(chip->dev, "%s: [MAX20357_CH%u] MASK dump   : %*ph\n", __func__, chip->channel_id, cnt_int, masks);
		dev_info(chip->dev, "%s: [MAX20357_CH%u] STATUS dump : %*ph\n", __func__, chip->channel_id, cnt_status, status);
	}

	if (chip->type == MAX20355 && chip->plc_notify_20355)
		chip->plc_notify_20355(chip->plc_data, ints);
	else if (chip->type == MAX20357 && chip->plc_notify_20357)
		chip->plc_notify_20357(chip->plc_data, ints);

	return IRQ_HANDLED;
}

static int max2035x_probe(struct i2c_client *client)
{
	struct max2035x *chip;
	struct regmap_config regmap_cfg;
	unsigned int rev, rev_reg;
	const struct mfd_cell *cells;
	int ncells, ret;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return dev_err_probe(&client->dev, -ENOMEM, "%s : Failed to allocate max2035x object\n", __func__);

	chip->dev = &client->dev;
	chip->irq = client->irq;

	chip->type = (enum max2035x_type)(uintptr_t)i2c_get_match_data(client);

	/* Init regmap */
	regmap_cfg = max2035x_regmap_cfg;
	regmap_cfg.max_register = MAX2035X_REG_MAX(chip->type);

	chip->regmap = devm_regmap_init_i2c(client, &regmap_cfg);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(chip->dev, PTR_ERR(chip->regmap), "%s : Failed to initialize regmap\n", __func__);

	switch (chip->type) {
	case MAX20355:
		rev_reg = MAX20355_REG_REVISION_ID;
		cells = max20355_cells;
		ncells = ARRAY_SIZE(max20355_cells);
		break;
	case MAX20357:
		rev_reg = MAX20357_REG_REVISION_ID;
		cells = max20357_cells;
		ncells = ARRAY_SIZE(max20357_cells);
		break;
	}

    ret = regmap_read(chip->regmap, rev_reg, &rev);
    if (ret)
		return dev_err_probe(chip->dev, ret, "%s : No response from device\n", __func__);

    dev_info(chip->dev, "%s : Analog Devices %s detected, rev=0x%02x, channel_id=%d\n",
			__func__, chip->type == MAX20355 ? "MAX20355" : "MAX20357", rev & 0xFF, chip->channel_id);

	i2c_set_clientdata(client, chip);

	chip->fuelgauge = i2c_new_ancillary_device(client, "fuel-gauge",
						    MAX2035X_I2C_ADDR_FG);
	if (IS_ERR(chip->fuelgauge))
		return dev_err_probe(chip->dev, PTR_ERR(chip->fuelgauge), "%s : Failed to create the Fuelgauge device\n", __func__);

	ret = devm_add_action_or_reset(chip->dev,
			(void (*)(void *))i2c_unregister_device, chip->fuelgauge);
	if (ret)
		return ret;

	i2c_set_clientdata(chip->fuelgauge, chip);

	chip->ram = i2c_new_ancillary_device(client, "ram",
					     chip->type == MAX20355 ?
					     MAX20355_I2C_ADDR_RAM :
					     MAX20357_I2C_ADDR_RAM);
	if (IS_ERR(chip->ram))
		return dev_err_probe(chip->dev, PTR_ERR(chip->ram), "%s : Failed to create the RAM device\n", __func__);

	ret = devm_add_action_or_reset(chip->dev,
			(void (*)(void *))i2c_unregister_device, chip->ram);
	if (ret)
		return ret;

	i2c_set_clientdata(chip->ram, chip);

	if (chip->irq > 0) {
		dev_info(chip->dev, "%s : client->irq = %d\n", __func__, client->irq);
		/* IRQ is read-clear, handled in threaded context only */
		ret = devm_request_threaded_irq(
				chip->dev,
				chip->irq,
				NULL,
				max2035x_irq_thread,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				"max2035x-irq",
				chip);
		if (ret)
			return dev_err_probe(chip->dev, ret,
					     "%s : Failed to request IRQ\n", __func__);
	} else
		dev_warn(chip->dev, "%s : No IRQ configured\n", __func__);

	dev_info(chip->dev, "%s : %s Core probed\n",
			__func__, chip->type == MAX20355 ? "MAX20355" : "MAX20357");

	max2035x_register_device(chip);
	ret = devm_add_action_or_reset(chip->dev,
			(void (*)(void *))max2035x_unregister_device, chip);
	if (ret)
		return ret;

	/* Register MFD sub-devices */
	ret = devm_mfd_add_devices(chip->dev, PLATFORM_DEVID_AUTO,
				   cells, ncells, NULL, 0, NULL);

	if (ret)
		return dev_err_probe(chip->dev, ret, "%s : Failed to add MFD devices\n", __func__);

	return 0;
}

/* -------------------------------------------------------------------------- */
/* I2C driver                                                                 */
/* -------------------------------------------------------------------------- */
static const struct i2c_device_id max2035x_id[] = {
	{ "max20355", MAX20355 },
	{ "max20357", MAX20357 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max2035x_id);

static struct i2c_driver max2035x_driver = {
	.driver = {
		.name = "max2035x",
		.of_match_table = max2035x_of_match,
	},
	.probe = max2035x_probe,
	.id_table = max2035x_id,
};

static int __init max2035x_init(void)
{
	int ret;

	/* Register platform drivers for all submodules */
	ret = max2035x_plc_init();
	if (ret)
		return ret;

	ret = max2035x_fuelgauge_init();
	if (ret)
		goto err_fuelgauge;

	ret = max2035x_ram_init();
	if (ret)
		goto err_ram;

	/* Register I2C driver last */
	ret = i2c_add_driver(&max2035x_driver);
	if (ret)
		goto err_i2c;

	return 0;

err_i2c:
	max2035x_ram_exit();
err_ram:
	max2035x_fuelgauge_exit();
err_fuelgauge:
	max2035x_plc_exit();
	return ret;
}
module_init(max2035x_init);

static void __exit max2035x_exit(void)
{
	i2c_del_driver(&max2035x_driver);
	max2035x_ram_exit();
	max2035x_fuelgauge_exit();
	max2035x_plc_exit();
}
module_exit(max2035x_exit);

MODULE_DESCRIPTION("Analog Devices MAX2035x MFD core Driver");
MODULE_AUTHOR("Judy Na <judy.na@analog.com>");
MODULE_LICENSE("GPL");
