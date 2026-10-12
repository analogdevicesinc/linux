// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * Analog Devices MAX2035x Fuelgauge Driver
 */
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/math64.h>

#include "../../mfd/maxim/max2035x/max2035x.h"
#include "../../mfd/maxim/max2035x/max2035x_registers.h"
#include "../../mfd/maxim/max2035x/max2035x_fuelgauge.h"

static int max2035x_read_fuelgauge_repcap(struct max2035x_fuelgauge *fuelgauge)
{
	unsigned int reg_val;
	int ret, capacity_uah;

	ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_REPCAP, &reg_val);
	if (ret) {
		dev_err(fuelgauge->dev, "%s : Failed to read MAX2035X_FG_REG_REPCAP (ret: %d)\n", __func__, ret);
		return ret;
	}

	capacity_uah = (int)((reg_val * 5000) / fuelgauge->rsense_mohm);
	dev_dbg(fuelgauge->dev, "%s: RepCap: %d.%03d mAh (0x%04x)\n",
		__func__, capacity_uah / 1000, capacity_uah % 1000, reg_val);

	return capacity_uah;
}

static int max2035x_read_fuelgauge_tte(struct max2035x_fuelgauge *fuelgauge)
{
	unsigned int reg_val;
	int ret, tte_seconds;

	ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_TTE, &reg_val);
	if (ret) {
		dev_err(fuelgauge->dev, "%s: Failed to read MAX2035X_FG_REG_TTE (ret: %d)\n", __func__, ret);
		return ret;
	}

	/* TTE: 5.625 seconds per LSB */
	tte_seconds = (int)((reg_val * 5625) / 1000);
	dev_dbg(fuelgauge->dev, "%s: TTE: %d sec (%dh %dm %ds) (0x%04x)\n",
		__func__,
		tte_seconds,
		tte_seconds / 3600, (tte_seconds % 3600) / 60, tte_seconds % 60,
		reg_val);

	return tte_seconds;
}

static int max2035x_read_fuelgauge_repsoc(struct max2035x_fuelgauge *fuelgauge)
{
	unsigned int reg_val;
	int ret, soc_percent_x100;

	ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_REPSOC, &reg_val);
	if (ret) {
		dev_err(fuelgauge->dev, "%s: Failed to read MAX2035X_FG_REG_REPSOC (ret: %d)\n", __func__, ret);
		return ret;
	}

	/* RepSOC: upper 8 bits = percentage, LSB = 1/256% */
	soc_percent_x100 = (reg_val >> 8) * 100 + ((reg_val & 0xFF) * 100) / 256;

	dev_dbg(fuelgauge->dev, "%s: RepSOC: %d.%02d%% (0x%04x)\n",
		__func__, soc_percent_x100 / 100, soc_percent_x100 % 100, reg_val);

	return soc_percent_x100;
}

static int max2035x_write_verify_reg(struct regmap *regmap, unsigned int reg, u16 val)
{
	int attempt = 0;
	unsigned int read_val;
	int ret;

	do {
		ret = regmap_write(regmap, reg, val);
		if (ret < 0)
			continue;

		usleep_range(1000, 1100);

		ret = regmap_read(regmap, reg, &read_val);
		if (ret < 0)
			continue;

		if ((u16)read_val == val)
			return 0;

		pr_debug("%s: Verify failed at 0x%02x: write 0x%04x, read 0x%04x (retry %d)\n",
			 __func__, reg, val, (u16)read_val, attempt + 1);

	} while (++attempt < 3);

	pr_err("%s: Final verify failed at 0x%02x after 3 attempts\n", __func__, reg);
	return -EIO;
}

static int max2035x_read_dt_u16(struct device_node *np, const char *name,
				u16 *val, u16 def)
{
	u32 tmp;

	if (of_property_read_u32(np, name, &tmp) == 0) {
		*val = (u16)tmp;
		return 0;
	}

	*val = def;
	return -EINVAL;
}

static int max2035x_load_battery_config(struct max2035x_fuelgauge *fuelgauge)
{
	struct device_node *np = fuelgauge->dev->of_node;
	struct max2035x_battery_data *bat = fuelgauge->battery_data;
	int ret, i;
	u32 model[32];

	if (!np)
		return -ENODEV;

	max2035x_read_dt_u16(np, "maxim,designcap", &bat->designcap, 0x1194);
	max2035x_read_dt_u16(np, "maxim,dpacc", &bat->dpacc, 0x0C80);
	max2035x_read_dt_u16(np, "maxim,dqacc", &bat->dqacc, 0x08CA);
	max2035x_read_dt_u16(np, "maxim,ichgterm", &bat->ichgterm, 0x03C0);
	max2035x_read_dt_u16(np, "maxim,learncfg", &bat->learncfg, 0x4486);
	max2035x_read_dt_u16(np, "maxim,misccfg", &bat->misccfg, 0x3870);
	max2035x_read_dt_u16(np, "maxim,qrtable00", &bat->qr_table00, 0x2C04);
	max2035x_read_dt_u16(np, "maxim,qrtable10", &bat->qr_table10, 0x1601);
	max2035x_read_dt_u16(np, "maxim,qrtable20", &bat->qr_table20, 0x0B00);
	max2035x_read_dt_u16(np, "maxim,qrtable30", &bat->qr_table30, 0x0A80);
	max2035x_read_dt_u16(np, "maxim,rcomp0", &bat->rcomp0, 0x0023);
	max2035x_read_dt_u16(np, "maxim,relaxcfg", &bat->relaxcfg, 0x2039);
	max2035x_read_dt_u16(np, "maxim,tempco", &bat->tempco, 0x171F);
	max2035x_read_dt_u16(np, "maxim,vempty", &bat->vempty, 0xA561);
	max2035x_read_dt_u16(np, "maxim,rcompseg", &bat->rcompseg, 0x0080);
	max2035x_read_dt_u16(np, "maxim,fullcaprep", &bat->fullcaprep, 0x1194);
	max2035x_read_dt_u16(np, "maxim,fullcapnom", &bat->fullcapnom, 0x1194);
	max2035x_read_dt_u16(np, "maxim,cycles", &bat->cycles, 0x0000);
	max2035x_read_dt_u16(np, "maxim,mixcap", &bat->mixcap, 0x017F);
	max2035x_read_dt_u16(np, "maxim,config", &bat->config, 0x2210);
	max2035x_read_dt_u16(np, "maxim,config2", &bat->config2, 0x0658);
	max2035x_read_dt_u16(np, "maxim,fullsocthr", &bat->fullsocthr, 0x5F00);
	max2035x_read_dt_u16(np, "maxim,tgain", &bat->tgain, 0xEE56);
	max2035x_read_dt_u16(np, "maxim,toff", &bat->toff, 0x1DA4);
	max2035x_read_dt_u16(np, "maxim,curve", &bat->curve, 0x3025);

	ret = of_property_read_u32_array(np, "maxim,model-data", model, 32);
	if (ret) {
		dev_err(fuelgauge->dev, "Failed to read maxim,model-data from DT\n");
		return ret;
	}

	for (i = 0; i < 16; i++) {
		bat->model_data[0][i] = (u16)model[i];
		bat->model_data[1][i] = (u16)model[16 + i];
	}

	return 0;
}

static int max2035x_custom_full_ini(struct max2035x_fuelgauge *fuelgauge)
{
	int i, ret;
	int attempt = 0;
	unsigned int reg_val, r_dqacc, r_dpacc, r_fullcapnom, r_config2;
	u32 r_vfsoc, update_capacity;
	struct max2035x_battery_data *bat = fuelgauge->battery_data;

	/* 2.3.1 Unlock Model Access */
	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_TABLE_UNLOCK1, 0x0059);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_TABLE_UNLOCK2, 0x00C4);
	if (ret)
		return ret;

	/* 2.3.2 Write/Read/Verify the Custom Model (0x80 - 0x9F) */
	for (i = 0; i < 16; i++) {
		ret = regmap_write(fuelgauge->regmap, 0x80 + i, bat->model_data[0][i]);
		if (ret)
			return ret;
	}
	for (i = 0; i < 16; i++) {
		ret = regmap_write(fuelgauge->regmap, 0x90 + i, bat->model_data[1][i]);
		if (ret)
			return ret;
	}

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_RCOMPSEG, bat->rcompseg);
	if (ret)
		return ret;

	/* Read back and Verify */
	for (i = 0; i < 16; i++) {
		ret = regmap_read(fuelgauge->regmap, 0x80 + i, &reg_val);
		if (ret)
			return ret;
		if (reg_val != bat->model_data[0][i])
			dev_warn(fuelgauge->dev, "Model verify failed at 0x%02x: expected 0x%04x, read 0x%04x\n",
				 0x80 + i, bat->model_data[0][i], reg_val);

		ret = regmap_read(fuelgauge->regmap, 0x90 + i, &reg_val);
		if (ret)
			return ret;
		if (reg_val != bat->model_data[1][i])
			dev_warn(fuelgauge->dev, "Model verify failed at 0x%02x: expected 0x%04x, read 0x%04x\n",
				 0x90 + i, bat->model_data[1][i], reg_val);
	}

	ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_RCOMPSEG, &reg_val);
	if (ret)
		return ret;
	if (reg_val != bat->rcompseg)
		dev_warn(fuelgauge->dev, "RCompSeg verify failed: expected 0x%04x, read 0x%04x\n",
			 bat->rcompseg, reg_val);

	/* 2.3.3 Lock Model Access */
	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_TABLE_UNLOCK1, 0x0000);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_TABLE_UNLOCK2, 0x0000);
	if (ret)
		return ret;

	/* 2.3.4 Verify that Model Access is locked */
	for (i = 0x80; i <= 0x9F; i++) {
		ret = regmap_read(fuelgauge->regmap, i, &reg_val);
		if (ret)
			return ret;
		if (reg_val != 0x0000 && reg_val != 0xFFFF)
			dev_warn(fuelgauge->dev, "Model lock verify: 0x%02x reads 0x%04x\n",
				 i, reg_val);
	}

	msleep(100);

	/* 2.3.5 Write Custom Parameters */
	max2035x_write_verify_reg(fuelgauge->regmap, MAX2035X_FG_REG_REPCAP, 0x0000);

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_DESIGNCAP, bat->designcap);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_FULLCAPREP, bat->fullcaprep);
	if (ret)
		return ret;

	do {
		ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_DQACC, bat->fullcapnom / 2);
		if (ret)
			return ret;

		ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_DPACC, 0x0C80);
		if (ret)
			return ret;

		msleep(10);

		ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_FULLCAPNOM, bat->fullcapnom);
		if (ret)
			return ret;

		ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_DQACC, &r_dqacc);
		if (ret)
			return ret;

		ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_DPACC, &r_dpacc);
		if (ret)
			return ret;

		ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_FULLCAPNOM, &r_fullcapnom);
		if (ret)
			return ret;

		if (r_dqacc == bat->fullcapnom / 2 &&
		    r_dpacc == 0x0C80 &&
		    r_fullcapnom == bat->fullcapnom)
			break;

		dev_warn(fuelgauge->dev, "Custom param verify failed, retrying (attempt %d)\n",
			 attempt + 1);
	} while (++attempt < 3);

	ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_VFSOC, &r_vfsoc);
	if (ret)
		return ret;

	update_capacity = (u32)div_u64((u64)r_vfsoc * bat->fullcapnom, 25600);

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_MIXCAP, update_capacity);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_AVCAP, update_capacity);
	if (ret)
		return ret;

	msleep(200);

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_ICHGTERM, bat->ichgterm);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_VEMPTY, bat->vempty);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_RCOMP0, bat->rcomp0);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_TEMPCO, bat->tempco);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_QRTABLE00, bat->qr_table00);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_QRTABLE10, bat->qr_table10);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_QRTABLE20, bat->qr_table20);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_QRTABLE30, bat->qr_table30);
	if (ret)
		return ret;

	/* 2.3.6 Updating optional registers */
	max2035x_write_verify_reg(fuelgauge->regmap, MAX2035X_FG_REG_LEARNCFG, bat->learncfg);

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_RELAXCFG, bat->relaxcfg);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_CONFIG, bat->config);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_CONFIG2, bat->config2);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_FULLSOCTHR, bat->fullsocthr);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_TGAIN, bat->tgain);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_TOFF, bat->toff);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_CURVE, bat->curve);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_MISCCFG, bat->misccfg);
	if (ret)
		return ret;

	/* 2.3.7 Initiate Model Loading */
	ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_CONFIG2, &r_config2);
	if (ret)
		return ret;

	ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_CONFIG2, r_config2 | 0x0020);
	if (ret)
		return ret;

	/* Poll LdMdl bit until it becomes 0 */
	ret = regmap_read_poll_timeout(fuelgauge->regmap, MAX2035X_FG_REG_CONFIG2,
				       r_config2, !(r_config2 & 0x0020),
				       10000, 400000);
	if (ret) {
		dev_err(fuelgauge->dev, "Model loading failed: LdMdl bit stuck at 1\n");
		return ret;
	}

	/* 2.3.8 Update QRTable20, QRTable30 and Cycles */
	max2035x_write_verify_reg(fuelgauge->regmap, MAX2035X_FG_REG_QRTABLE20, bat->qr_table20);
	max2035x_write_verify_reg(fuelgauge->regmap, MAX2035X_FG_REG_QRTABLE30, bat->qr_table30);
	max2035x_write_verify_reg(fuelgauge->regmap, MAX2035X_FG_REG_CYCLES, bat->cycles);

	return 0;
}

static int max2035x_initialize_fuelgauge(struct max2035x_fuelgauge *fuelgauge)
{
	unsigned int r_status, r_fstat, r_hibcfg;
	int ret, retry = 0;

	/* <Step 0> Check for POR */
	for (retry = 0; retry < 5; retry++) {
		ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_STATUS, &r_status);
		if (ret)
			return ret;

		if (!(r_status & MAX2035X_FG_STATUS_POR)) {
			dev_info(fuelgauge->dev, "%s: Fuelgauge already initialized (POR=0)\n", __func__);
			goto step_3_2;
		}

		/* <Step 1> Delay until FSTAT.DNR bit == 0 */
		ret = regmap_read_poll_timeout(fuelgauge->regmap,
					       MAX2035X_FG_REG_FSTAT, r_fstat,
					       !(r_fstat & MAX2035X_FG_FSTAT_DNR),
					       10000, 500000);
		if (ret)
			return ret;

		/* <Step 2> Initialize Configuration */
		ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_HIBCFG, &r_hibcfg);
		if (ret)
			return ret;

		ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_SOFT_WAKEUP, 0x0090);
		if (ret)
			return ret;

		ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_HIBCFG, 0x0000);
		if (ret)
			return ret;

		ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_SOFT_WAKEUP, 0x0000);
		if (ret)
			return ret;

		ret = max2035x_custom_full_ini(fuelgauge);
		if (ret)
			return ret;

		ret = regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_HIBCFG, r_hibcfg);
		if (ret)
			return ret;

		/* <Step 3> Initialize Complete */
		ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_STATUS, &r_status);
		if (ret)
			return ret;

		max2035x_write_verify_reg(fuelgauge->regmap, MAX2035X_FG_REG_STATUS, r_status & 0xFFFD);

		/* 3.1 Check for IC Reset */
		ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_STATUS, &r_status);
		if (ret)
			return ret;

		if ((r_status & 0x0002) == 0)
			goto step_3_2;
	}

step_3_2:
	/* 3.2 Read the RepCap and RepSOC Registers */
	max2035x_read_fuelgauge_repcap(fuelgauge);
	max2035x_read_fuelgauge_repsoc(fuelgauge);

	/* 3.3 : Read the TTE Register */
	max2035x_read_fuelgauge_tte(fuelgauge);

	dev_info(fuelgauge->dev, "%s: Fuelgauge initialization completed \n", __func__);

	return 0;
}

static int max2035x_fuelgauge_probe(struct platform_device *pdev)
{
	struct max2035x *chip = dev_get_drvdata(pdev->dev.parent);
	struct max2035x_fuelgauge *fuelgauge;
	int ret;

	fuelgauge = devm_kzalloc(&pdev->dev, sizeof(*fuelgauge), GFP_KERNEL);
	if (!fuelgauge)
		return -ENOMEM;

	fuelgauge->battery_data = devm_kzalloc(&pdev->dev, sizeof(*fuelgauge->battery_data), GFP_KERNEL);
	if (!fuelgauge->battery_data)
		return -ENOMEM;

	fuelgauge->dev = &pdev->dev;
	fuelgauge->chip = chip;
	fuelgauge->regmap = chip->fg_regmap;

	/* Set rsense to 200 mohm */
	fuelgauge->rsense_mohm = 200;

	platform_set_drvdata(pdev, fuelgauge);

	ret = max2035x_load_battery_config(fuelgauge);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to load battery config from DT\n");

	ret = max2035x_initialize_fuelgauge(fuelgauge);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to initialize fuelgauge\n");

	return 0;
}

static const struct platform_device_id max2035x_fuelgauge_id[] = {
	{ "max20355-fuelgauge" },
	{ "max20357-fuelgauge" },
	{ }
};
MODULE_DEVICE_TABLE(platform, max2035x_fuelgauge_id);

static struct platform_driver max2035x_fuelgauge_driver = {
	.driver = {
		.name = "max2035x-fuelgauge",
	},
	.probe = max2035x_fuelgauge_probe,
	.id_table = max2035x_fuelgauge_id,
};
module_platform_driver(max2035x_fuelgauge_driver);

MODULE_DESCRIPTION("Analog Devices MAX2035x Fuelgauge Driver");
MODULE_AUTHOR("Judy Na <judy.na@analog.com>");
MODULE_LICENSE("GPL");
