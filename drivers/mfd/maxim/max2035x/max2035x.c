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
#include <linux/of.h>
#include <linux/regmap.h>

#include "max2035x.h"
#include "max2035x_registers.h"

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
	{ .name = "max20355-battery" },
	{ .name = "max20355-gpio" },
	{ .name = "max20355-regulator" },
};

static const struct mfd_cell max20357_cells[] = {
	{ .name = "max20357-plc" },
	{ .name = "max20357-fuelgauge" },
	{ .name = "max20357-battery" },
	{ .name = "max20357-charger" },
	{ .name = "max20357-gpio" },
};

/* -------------------------------------------------------------------------- */
/* regmap IRQ chip definitions                                                */
/* -------------------------------------------------------------------------- */
static const struct regmap_irq max20355_irqs[] = {
	/* INT0 (0x05) */
	REGMAP_IRQ_REG(0,  0, MAX20355_INT0_ITF_RDY_STS_BIT),
	REGMAP_IRQ_REG(1,  0, MAX20355_INT0_CH1_CON_BIT),
	REGMAP_IRQ_REG(2,  0, MAX20355_INT0_CH2_CON_BIT),
	REGMAP_IRQ_REG(3,  0, MAX20355_INT0_CH1_IDL_BIT),
	REGMAP_IRQ_REG(4,  0, MAX20355_INT0_CH2_IDL_BIT),
	REGMAP_IRQ_REG(5,  0, MAX20355_INT0_MOI_DNE_BIT),
	REGMAP_IRQ_REG(6,  0, MAX20355_INT0_PLC2_MOI_DET_BIT),
	REGMAP_IRQ_REG(7,  0, MAX20355_INT0_PLC1_MOI_DET_BIT),
	/* INT1 (0x06) */
	REGMAP_IRQ_REG(8,  1, MAX20355_INT1_SYS_ERR_BIT),
	REGMAP_IRQ_REG(9,  1, MAX20355_INT1_BB_FAULT_BIT),
	REGMAP_IRQ_REG(10, 1, MAX20355_INT1_THM_FLT_BIT),
	REGMAP_IRQ_REG(11, 1, MAX20355_INT1_PLC_NEW_DAT_BIT),
	REGMAP_IRQ_REG(12, 1, MAX20355_INT1_PLC2_CMD_DNE_BIT),
	REGMAP_IRQ_REG(13, 1, MAX20355_INT1_PLC1_CMD_DNE_BIT),
	REGMAP_IRQ_REG(14, 1, MAX20355_INT1_PLC2_CMD_ERR_BIT),
	REGMAP_IRQ_REG(15, 1, MAX20355_INT1_PLC1_CMD_ERR_BIT),
	/* INT2 (0x07) */
	REGMAP_IRQ_REG(16, 2, MAX20355_INT2_MOI_DET_BIT),
	REGMAP_IRQ_REG(17, 2, MAX20355_INT2_RES_DET_ABR_BIT),
	REGMAP_IRQ_REG(18, 2, MAX20355_INT2_RES_DET_OPN_BIT),
	REGMAP_IRQ_REG(19, 2, MAX20355_INT2_RES_DET_GND_BIT),
	/* INT3 (0x08) */
	REGMAP_IRQ_REG(20, 3, MAX20355_INT3_URT_TMO_FLT2_BIT),
	REGMAP_IRQ_REG(21, 3, MAX20355_INT3_URT_MODFAIL2_BIT),
	REGMAP_IRQ_REG(22, 3, MAX20355_INT3_URT_MODDONE2_BIT),
	REGMAP_IRQ_REG(23, 3, MAX20355_INT3_URT_TMO_FLT1_BIT),
	REGMAP_IRQ_REG(24, 3, MAX20355_INT3_URT_MODFAIL1_BIT),
	REGMAP_IRQ_REG(25, 3, MAX20355_INT3_URT_MODDONE1_BIT),
};

static const struct regmap_irq_chip max20355_irq_chip = {
	.name = "max20355",
	.irqs = max20355_irqs,
	.num_irqs = ARRAY_SIZE(max20355_irqs),
	.num_regs = 4,
	.status_base = MAX20355_REG_INT0,
	.unmask_base = MAX20355_REG_INTMASK0,
};

static const struct regmap_irq max20357_irqs[] = {
	/* INT0 (0x08) */
	REGMAP_IRQ_REG(0,  0, MAX20357_INT0_PLC_SUMACT_BIT),
	REGMAP_IRQ_REG(1,  0, MAX20357_INT0_PLC_SUMCURR_BIT),
	REGMAP_IRQ_REG(2,  0, MAX20357_INT0_CHG_THRM_REG_BIT),
	REGMAP_IRQ_REG(3,  0, MAX20357_INT0_CC1_TMO_BIT),
	REGMAP_IRQ_REG(4,  0, MAX20357_INT0_CHGSTAT_BIT),
	REGMAP_IRQ_REG(5,  0, MAX20357_INT0_SYSMINREG_BIT),
	REGMAP_IRQ_REG(6,  0, MAX20357_INT0_CHG_RESTA_B_BIT),
	REGMAP_IRQ_REG(7,  0, MAX20357_INT0_THMSTAT_BIT),
	/* INT1 (0x09) */
	REGMAP_IRQ_REG(8,  1, MAX20357_INT1_JEITA_IS_REG_BIT),
	REGMAP_IRQ_REG(9,  1, MAX20357_INT1_CHG_REV_BIT),
	REGMAP_IRQ_REG(10, 1, MAX20357_INT1_CHG_VOLT_MODE_BIT),
	REGMAP_IRQ_REG(11, 1, MAX20357_INT1_CHG_VOLT_STP_BIT),
	REGMAP_IRQ_REG(12, 1, MAX20357_INT1_CHG_GMD_BIT),
	REGMAP_IRQ_REG(13, 1, MAX20357_INT1_LDO_GMD_BIT),
	REGMAP_IRQ_REG(14, 1, MAX20357_INT1_PLCOk_BIT),
	REGMAP_IRQ_REG(15, 1, MAX20357_INT1_SYSREV_BIT),
	/* INT2 (0x0A) */
	REGMAP_IRQ_REG(16, 2, MAX20357_INT2_CHN_CON_BIT),
	REGMAP_IRQ_REG(17, 2, MAX20357_INT2_CHN_WTY_BIT),
	REGMAP_IRQ_REG(18, 2, MAX20357_INT2_CHN_IDL_BIT),
	REGMAP_IRQ_REG(19, 2, MAX20357_INT2_SRT_XFER_RISE_BIT),
	REGMAP_IRQ_REG(20, 2, MAX20357_INT2_SRT_XFER_FALL_BIT),
	REGMAP_IRQ_REG(21, 2, MAX20357_INT2_PLC_NEW_DAT_BIT),
	REGMAP_IRQ_REG(22, 2, MAX20357_INT2_PLC_CMD_DNE_BIT),
	REGMAP_IRQ_REG(23, 2, MAX20357_INT2_PLC_CMD_ERR_BIT),
	/* INT3 (0x0B) */
	REGMAP_IRQ_REG(24, 3, MAX20357_INT3_LNG_XFER_BIT),
	REGMAP_IRQ_REG(25, 3, MAX20357_INT3_BATUVLOB_BIT),
	REGMAP_IRQ_REG(26, 3, MAX20357_INT3_MOI_DNE_BIT),
	REGMAP_IRQ_REG(27, 3, MAX20357_INT3_PLC_MOI_DET_BIT),
	REGMAP_IRQ_REG(28, 3, MAX20357_INT3_MOI_DET_BIT),
	REGMAP_IRQ_REG(29, 3, MAX20357_INT3_RES_DET_ABR_BIT),
	REGMAP_IRQ_REG(30, 3, MAX20357_INT3_RES_DET_OPN_BIT),
	REGMAP_IRQ_REG(31, 3, MAX20357_INT3_RES_DET_GND_BIT),
	/* INT4 (0x0C) */
	REGMAP_IRQ_REG(32, 4, MAX20357_INT4_URT_TMO_FLT_BIT),
	REGMAP_IRQ_REG(33, 4, MAX20357_INT4_URT_MODFAIL_BIT),
	REGMAP_IRQ_REG(34, 4, MAX20357_INT4_URT_MODDONE_BIT),
	REGMAP_IRQ_REG(35, 4, MAX20357_INT4_URT_SWC_OPN_BIT),
	REGMAP_IRQ_REG(36, 4, MAX20357_INT4_DEAD_FOUND_BIT),
	REGMAP_IRQ_REG(37, 4, MAX20357_INT4_SWC_OFF_MOD_BIT),
	REGMAP_IRQ_REG(38, 4, MAX20357_INT4_CHG_PRQ_INP_BIT),
	/* INT5 (0x0D) */
	REGMAP_IRQ_REG(39, 5, MAX20357_INT5_ITF_RDY_STS_BIT),
	REGMAP_IRQ_REG(40, 5, MAX20357_INT5_WD_ITR_CLR_BIT),
};

static const struct regmap_irq_chip max20357_irq_chip = {
	.name = "max20357",
	.irqs = max20357_irqs,
	.num_irqs = ARRAY_SIZE(max20357_irqs),
	.num_regs = 6,
	.status_base = MAX20357_REG_INT0,
	.unmask_base = MAX20357_REG_INTMASK0,
};

/* -------------------------------------------------------------------------- */
/* Variant data                                                               */
/* -------------------------------------------------------------------------- */
static const struct max2035x_variant max20355_variant = {
	.name = "MAX20355",
	.rev_reg = MAX20355_REG_REVISION_ID,
	.max_register = MAX20355_REG_MAX,
	.i2c_addr_ram = MAX20355_I2C_ADDR_RAM,
	.cells = max20355_cells,
	.num_cells = ARRAY_SIZE(max20355_cells),
	.irq_chip = &max20355_irq_chip,
};

static const struct max2035x_variant max20357_variant = {
	.name = "MAX20357",
	.rev_reg = MAX20357_REG_REVISION_ID,
	.max_register = MAX20357_REG_MAX,
	.i2c_addr_ram = MAX20357_I2C_ADDR_RAM,
	.cells = max20357_cells,
	.num_cells = ARRAY_SIZE(max20357_cells),
	.irq_chip = &max20357_irq_chip,
};

/* -------------------------------------------------------------------------- */
/* Device Tree match table                                                    */
/* -------------------------------------------------------------------------- */
static const struct of_device_id max2035x_of_match[] = {
	{ .compatible = "adi,max20355", .data = &max20355_variant },
	{ .compatible = "adi,max20357", .data = &max20357_variant },
	{ }
};
MODULE_DEVICE_TABLE(of, max2035x_of_match);

static int max2035x_probe(struct i2c_client *client)
{
	const struct max2035x_variant *info;
	struct max2035x *chip;
	struct regmap_config regmap_cfg;
	unsigned int rev;
	int ret;

	info = i2c_get_match_data(client);
	if (!info)
		return -ENODEV;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &client->dev;
	chip->irq = client->irq;
	chip->info = info;
	chip->type = (info == &max20355_variant) ? MAX20355 : MAX20357;

	if (chip->type == MAX20357) {
		ret = of_property_read_u32(client->dev.of_node, "adi,plc-channel",
					   &chip->channel_id);
		if (ret)
			return dev_err_probe(&client->dev, ret,
					     "missing adi,plc-channel property\n");
	}

	regmap_cfg = max2035x_regmap_cfg;
	regmap_cfg.max_register = info->max_register;

	chip->regmap = devm_regmap_init_i2c(client, &regmap_cfg);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(chip->dev, PTR_ERR(chip->regmap),
				     "Failed to initialize regmap\n");

	ret = regmap_read(chip->regmap, info->rev_reg, &rev);
	if (ret)
		return dev_err_probe(chip->dev, ret, "No response from device\n");

	dev_info(chip->dev, "Analog Devices %s detected, rev=0x%02x, channel_id=%d\n",
		 info->name, rev & 0xFF, chip->channel_id);

	i2c_set_clientdata(client, chip);

	chip->fuelgauge = i2c_new_ancillary_device(client, "fuel-gauge",
						    MAX2035X_I2C_ADDR_FG);
	if (IS_ERR(chip->fuelgauge))
		return dev_err_probe(chip->dev, PTR_ERR(chip->fuelgauge),
				     "Failed to create fuelgauge device\n");

	ret = devm_add_action_or_reset(chip->dev,
			(void (*)(void *))i2c_unregister_device, chip->fuelgauge);
	if (ret)
		return ret;

	i2c_set_clientdata(chip->fuelgauge, chip);

	chip->ram = i2c_new_ancillary_device(client, "ram", info->i2c_addr_ram);
	if (IS_ERR(chip->ram))
		return dev_err_probe(chip->dev, PTR_ERR(chip->ram),
				     "Failed to create RAM device\n");

	ret = devm_add_action_or_reset(chip->dev,
			(void (*)(void *))i2c_unregister_device, chip->ram);
	if (ret)
		return ret;

	i2c_set_clientdata(chip->ram, chip);

	if (chip->irq > 0) {
		ret = devm_regmap_add_irq_chip(chip->dev, chip->regmap,
					       chip->irq,
					       IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					       0, info->irq_chip,
					       &chip->irq_data);
		if (ret)
			return dev_err_probe(chip->dev, ret,
					     "Failed to add IRQ chip\n");
	} else {
		dev_warn(chip->dev, "No IRQ configured\n");
	}

	dev_info(chip->dev, "%s core probed\n", info->name);

	ret = devm_mfd_add_devices(chip->dev, PLATFORM_DEVID_AUTO,
				   info->cells, info->num_cells, NULL, 0, NULL);
	if (ret)
		return dev_err_probe(chip->dev, ret,
				     "Failed to add MFD devices\n");

	return 0;
}

/* -------------------------------------------------------------------------- */
/* I2C driver                                                                 */
/* -------------------------------------------------------------------------- */
static const struct i2c_device_id max2035x_id[] = {
	{ "max20355", (kernel_ulong_t)&max20355_variant },
	{ "max20357", (kernel_ulong_t)&max20357_variant },
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

	/* Register I2C driver last */
	ret = i2c_add_driver(&max2035x_driver);
	if (ret)
		goto err_i2c;

	return 0;

err_i2c:
	max2035x_fuelgauge_exit();
err_fuelgauge:
	max2035x_plc_exit();
	return ret;
}
module_init(max2035x_init);

static void __exit max2035x_exit(void)
{
	i2c_del_driver(&max2035x_driver);
	max2035x_fuelgauge_exit();
	max2035x_plc_exit();
}
module_exit(max2035x_exit);

MODULE_DESCRIPTION("Analog Devices MAX2035x MFD core Driver");
MODULE_AUTHOR("Judy Na <judy.na@analog.com>");
MODULE_LICENSE("GPL");
