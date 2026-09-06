// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * Analog Devices MAX2035x Fuelgauge Driver
 */
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/ctype.h>
#include <linux/math64.h>

#include "max2035x.h"
#include "max2035x_registers.h"
#include "max2035x_fuelgauge.h"

static const struct regmap_config max2035x_fg_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0xFF,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.cache_type = REGCACHE_NONE,
};

static char *_strcasestr(const char *s, const char *find)
{
	size_t len;
	char c, sc;

	c = *find++;
	if (c != 0) {
		c = (char)tolower((unsigned char)c);
		len = strlen(find);
		do {
			do {
				sc = *s++;
				if (sc == 0)
					return NULL;
			} while ((char)tolower((unsigned char)sc) != c);
		} while (strncasecmp(s, find, len) != 0);
		s--;
	}
	return (char *)s;
}

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
	dev_info(fuelgauge->dev, "%s: RepCap: %d.%03d mAh (0x%04x)\n",
		__func__, capacity_uah / 1000, capacity_uah % 1000, reg_val);

	return capacity_uah;
}

static int max2035x_read_fuelgauge_tte(struct max2035x_fuelgauge *fuelgauge)
{
	unsigned int reg_val;
	int ret, tte_seconds;

	if (!fuelgauge) {
		pr_err("%s: fuelgauge is NULL\n", __func__);
		return -EINVAL;
	}

	ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_TTE, &reg_val);
	if (ret) {
		dev_err(fuelgauge->dev, "%s: Failed to read MAX2035X_FG_REG_TTE (ret: %d)\n", __func__, ret);
		return ret;
	}

	/* TTE: 5.625 seconds per LSB */
	tte_seconds = (int)((reg_val * 5625) / 1000);
	dev_info(fuelgauge->dev, "%s: TTE: %d sec (%dh %dm %ds) (0x%04x)\n",
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

	if (!fuelgauge) {
		pr_err("%s: fuelgauge is NULL\n", __func__);
		return -EINVAL;
	}

	ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_REPSOC, &reg_val);
	if (ret) {
		dev_err(fuelgauge->dev, "%s: Failed to read MAX2035X_FG_REG_REPSOC (ret: %d)\n", __func__, ret);
		return ret;
	}

	/* RepSOC: upper 8 bits = percentage, LSB = 1/256% */
	soc_percent_x100 = (reg_val >> 8) * 100 + ((reg_val & 0xFF) * 100) / 256;

	dev_info(fuelgauge->dev, "%s: RepSOC: %d.%02d%% (0x%04x)\n",
		__func__, soc_percent_x100 / 100, soc_percent_x100 % 100, reg_val);

	return soc_percent_x100;
}

static int max2035x_read_fuelgauge_vcell(struct max2035x_fuelgauge *fuelgauge)
{
	unsigned int reg_val;
	int ret, vcell_uv;

	if (!fuelgauge) {
		pr_err("%s: fuelgauge is NULL\n", __func__);
		return -EINVAL;
	}

	ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_VCELL, &reg_val);
	if (ret) {
		dev_err(fuelgauge->dev, "%s: Failed to read MAX2035X_FG_REG_VCELL (ret: %d)\n", __func__, ret);
		return ret;
	}

	/* VCell: LSB = 1.25mV/16 = 78.125μV */
	vcell_uv = (int)div_u64((u64)reg_val * 78125, 1000);

	dev_info(fuelgauge->dev, "%s: VCell: %d.%03d mV (0x%04x)\n",
		__func__, vcell_uv / 1000, vcell_uv % 1000, reg_val);

	return vcell_uv;
}

static int max2035x_read_fuelgauge_avgvcell(struct max2035x_fuelgauge *fuelgauge)
{
	unsigned int reg_val;
	int ret, avgvcell_uv;

	if (!fuelgauge) {
		pr_err("%s: fuelgauge is NULL\n", __func__);
		return -EINVAL;
	}

	ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_AVGVCELL, &reg_val);
	if (ret) {
		dev_err(fuelgauge->dev, "%s: Failed to read MAX2035X_FG_REG_AVGVCELL (ret: %d)\n", __func__, ret);
		return ret;
	}

	/* AvgVCell: LSB = 1.25mV/16 = 78.125μV */
	avgvcell_uv = (int)div_u64((u64)reg_val * 78125, 1000);

	dev_info(fuelgauge->dev, "%s: AvgVCell: %d.%03d mV (0x%04x)\n",
		__func__, avgvcell_uv / 1000, avgvcell_uv % 1000, reg_val);

	return avgvcell_uv;
}

static int max2035x_read_fuelgauge_ttf(struct max2035x_fuelgauge *fuelgauge)
{
	unsigned int reg_val;
	int ret, ttf_sec;

	if (!fuelgauge) {
		pr_err("%s: fuelgauge is NULL\n", __func__);
		return -EINVAL;
	}

	ret = regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_TTF, &reg_val);
	if (ret) {
		dev_err(fuelgauge->dev, "%s: Failed to read MAX2035X_FG_REG_TTF (ret: %d)\n", __func__, ret);
		return ret;
	}

	/* TTF: LSB = 5.625s */
	ttf_sec = (reg_val * 5625) / 1000;

	dev_info(fuelgauge->dev, "%s: TTF: %d sec (%dh %dm %ds) (0x%04x)\n",
		__func__, ttf_sec, ttf_sec / 3600, (ttf_sec % 3600) / 60, ttf_sec % 60, reg_val);

	return ttf_sec;
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

static int max2035x_load_ini_file(struct max2035x_fuelgauge *fuelgauge)
{
    const struct firmware *fw;
    const char *ptr, *section_ptr;
    int ret, i;
    u16 val;
    struct max2035x_battery_data *bat = fuelgauge->battery_data;
	char *fw_name = INI_FILE_NAME;

    ret = request_firmware(&fw, fw_name, fuelgauge->dev);
    if (ret) {
		dev_err(fuelgauge->dev, "%s: Failed to load %s (ret: %d)\n", __func__, fw_name, ret);
		return ret;
    }

    ptr = (const char *)fw->data;

	section_ptr = _strcasestr(ptr, "DesignCap=");
	if (section_ptr) {
		sscanf(section_ptr, "DesignCap=%hx", &bat->designcap);
		dev_info(fuelgauge->dev, "DesingCap found, using DesignCap: 0x%04x\n", bat->designcap);
	} else {
		bat->designcap = 0x1194;
		dev_info(fuelgauge->dev, "DesingCap not found, using DesignCap: 0x%04x\n", bat->designcap);
	}

	section_ptr = _strcasestr(ptr, "dPacc=");
	if (section_ptr) {
		sscanf(section_ptr, "dPacc=%hx", &bat->dpacc);
		dev_info(fuelgauge->dev, "dPacc found, using dPacc: 0x%04x\n", bat->dpacc);
	} else {
		bat->dpacc = 0x0C80;
		dev_info(fuelgauge->dev, "dPacc not found, using dPacc: 0x%04x\n", bat->dpacc);
	}

	section_ptr = _strcasestr(ptr, "dQacc=");
	if (section_ptr) {
		sscanf(section_ptr, "dQacc=%hx", &bat->dqacc);
		dev_info(fuelgauge->dev, "dQacc found, using dQacc: 0x%04x\n", bat->dqacc);
	} else {
		bat->dqacc = 0x08CA;
		dev_info(fuelgauge->dev, "dQacc not found, using dQacc: 0x%04x\n", bat->dqacc);
	}

	section_ptr = _strcasestr(ptr, "ICHGTerm=");
	if (section_ptr) {
		sscanf(section_ptr, "ICHGTerm=%hx", &bat->ichgterm);
		dev_info(fuelgauge->dev, "ICHGTerm found, using ICHGTerm: 0x%04x\n", bat->ichgterm);
	} else {
		bat->ichgterm = 0x03C0;
		dev_info(fuelgauge->dev, "ICHGTerm not found, using ICHGTerm: 0x%04x\n", bat->ichgterm);
	}

	section_ptr = _strcasestr(ptr, "learncfg=");
	if (section_ptr) {
		sscanf(section_ptr, "learncfg=%hx", &bat->learncfg);
		dev_info(fuelgauge->dev, "learncfg found, using learncfg: 0x%04x\n", bat->learncfg);
	} else {
		bat->learncfg = 0x4486;
		dev_info(fuelgauge->dev, "learncfg not found, using learncfg: 0x%04x\n", bat->learncfg);
	}

	section_ptr = _strcasestr(ptr, "misccfg=");
	if (section_ptr) {
		sscanf(section_ptr, "misccfg=%hx", &bat->misccfg);
		dev_info(fuelgauge->dev, "misccfg found, using misccfg: 0x%04x\n", bat->misccfg);
	} else {
		bat->misccfg = 0x3870;
		dev_info(fuelgauge->dev, "misccfg not found, using misccfg: 0x%04x\n", bat->misccfg);
	}

	section_ptr = _strcasestr(ptr, "QRTable00=");
	if (section_ptr) {
		sscanf(section_ptr, "QRTable00=%hx", &bat->qr_table00);
		dev_info(fuelgauge->dev, "QRTable00 found, using QRTable00: 0x%04x\n", bat->qr_table00);
	} else {
		bat->qr_table00 = 0x2C04;
		dev_info(fuelgauge->dev, "QRTable00 not found, using QRTable00: 0x%04x\n", bat->qr_table00);
	}

	section_ptr = _strcasestr(ptr, "QRTable10=");
	if (section_ptr) {
		sscanf(section_ptr, "QRTable10=%hx", &bat->qr_table10);
		dev_info(fuelgauge->dev, "QRTable10 found, using QRTable10: 0x%04x\n", bat->qr_table10);
	} else {
		bat->qr_table10 = 0x1601;
		dev_info(fuelgauge->dev, "QRTable10 not found, using QRTable10: 0x%04x\n", bat->qr_table10);
	}

	section_ptr = _strcasestr(ptr, "QRTable20=");
	if (section_ptr) {
		sscanf(section_ptr, "QRTable20=%hx", &bat->qr_table20);
		dev_info(fuelgauge->dev, "QRTable20 found, using QRTable20: 0x%04x\n", bat->qr_table20);
	} else {
		bat->qr_table20 = 0x0B00;
		dev_info(fuelgauge->dev, "QRTable20 not found, using QRTable20: 0x%04x\n", bat->qr_table20);
	}

	section_ptr = _strcasestr(ptr, "QRTable30=");
	if (section_ptr) {
		sscanf(section_ptr, "QRTable30=%hx", &bat->qr_table30);
		dev_info(fuelgauge->dev, "QRTable30 found, using QRTable30: 0x%04x\n", bat->qr_table30);
	} else {
		bat->qr_table30 = 0x0A80;
		dev_info(fuelgauge->dev, "QRTable30 not found, using QRTable30: 0x%04x\n", bat->qr_table30);
	}

	section_ptr = _strcasestr(ptr, "RCOMP0=");
	if (section_ptr) {
		sscanf(section_ptr, "RCOMP0=%hx", &bat->rcomp0);
		dev_info(fuelgauge->dev, "RCOMP0 found, using RCOMP0: 0x%04x\n", bat->rcomp0);
	} else {
		bat->rcomp0 = 0x0023;
		dev_info(fuelgauge->dev, "RCOMP0 not found, using RCOMP0: 0x%04x\n", bat->rcomp0);
	}

	section_ptr = _strcasestr(ptr, "relaxcfg=");
	if (section_ptr) {
		sscanf(section_ptr, "relaxcfg=%hx", &bat->relaxcfg);
		dev_info(fuelgauge->dev, "relaxcfg found, using relaxcfg: 0x%04x\n", bat->relaxcfg);
	} else {
		bat->relaxcfg = 0x2039;
		dev_info(fuelgauge->dev, "relaxcfg not found, using relaxcfg: 0x%04x\n", bat->relaxcfg);
	}

	section_ptr = _strcasestr(ptr, "TempCo=");
	if (section_ptr) {
		sscanf(section_ptr, "TempCo=%hx", &bat->tempco);
		dev_info(fuelgauge->dev, "TempCo found, using TempCo: 0x%04x\n", bat->tempco);
	} else {
		bat->tempco = 0x171F;
		dev_info(fuelgauge->dev, "TempCo not found, using TempCo: 0x%04x\n", bat->tempco);
	}

	section_ptr = _strcasestr(ptr, "Vempty=");
	if (section_ptr) {
		sscanf(section_ptr, "Vempty=%hx", &bat->vempty);
		dev_info(fuelgauge->dev, "Vempty found, using Vempty: 0x%04x\n", bat->vempty);
	} else {
		bat->vempty = 0xA561;
		dev_info(fuelgauge->dev, "Vempty not found, using Vempty: 0x%04x\n", bat->vempty);
	}

	section_ptr = _strcasestr(ptr, "RCOMPSEG=");
	if (section_ptr) {
		sscanf(section_ptr, "RCOMPSEG=%hx", &bat->rcompseg);
		dev_info(fuelgauge->dev, "RCOMPSEG found, using RCOMPSEG: 0x%04x\n", bat->rcompseg);
	} else {
		bat->rcompseg = 0x0080;
		dev_info(fuelgauge->dev, "RCOMPSEG not found, using RCOMPSEG: 0x%04x\n", bat->rcompseg);
	}

	section_ptr = _strcasestr(ptr, "fullcaprep=");
	if (section_ptr) {
		sscanf(section_ptr, "fullcaprep=%hx", &bat->fullcaprep);
		dev_info(fuelgauge->dev, "fullcaprep found, using fullcaprep: 0x%04x\n", bat->fullcaprep);
	} else {
		bat->fullcaprep = 0x1194;
		dev_info(fuelgauge->dev, "fullcaprep not found, using fullcaprep: 0x%04x\n", bat->fullcaprep);
	}

	section_ptr = _strcasestr(ptr, "fullcapnom=");
	if (section_ptr) {
		sscanf(section_ptr, "fullcapnom=%hx", &bat->fullcapnom);
		dev_info(fuelgauge->dev, "fullcapnom found, using fullcapnom: 0x%04x\n", bat->fullcapnom);
	} else {
		bat->fullcapnom = 0x1194;
		dev_info(fuelgauge->dev, "fullcapnom not found, using fullcapnom: 0x%04x\n", bat->fullcapnom);
	}

	section_ptr = _strcasestr(ptr, "Cycles=");
	if (section_ptr) {
		sscanf(section_ptr, "Cycles=%hx", &bat->cycles);
		dev_info(fuelgauge->dev, "Cycles found, using Cycles: 0x%04x\n", bat->cycles);
	} else {
		bat->cycles = 0x0000;
		dev_info(fuelgauge->dev, "Cycles not found, using Cycles: 0x%04x\n", bat->cycles);
	}

	section_ptr = _strcasestr(ptr, "Mixcap=");
	if (section_ptr) {
		sscanf(section_ptr, "Mixcap=%hx", &bat->mixcap);
		dev_info(fuelgauge->dev, "Mixcap found, using Mixcap: 0x%04x\n", bat->mixcap);
	} else {
		bat->mixcap = 0x017F;
		dev_info(fuelgauge->dev, "Mixcap not found, using Mixcap: 0x%04x\n", bat->mixcap);
	}

	section_ptr = _strcasestr(ptr, "config=");
	if (section_ptr) {
		sscanf(section_ptr, "config=%hx", &bat->config);
		dev_info(fuelgauge->dev, "config found, using config: 0x%04x\n", bat->config);
	} else {
		bat->config = 0x2210;
		dev_info(fuelgauge->dev, "config not found, using config: 0x%04x\n", bat->config);
	}

	section_ptr = _strcasestr(ptr, "config2=");
	if (section_ptr) {
		sscanf(section_ptr, "config2=%hx", &bat->config2);
		dev_info(fuelgauge->dev, "config2 found, using config2: 0x%04x\n", bat->config2);
	} else {
		bat->config2 = 0x0658;
		dev_info(fuelgauge->dev, "config2 not found, using config2: 0x%04x\n", bat->config2);
	}

	section_ptr = _strcasestr(ptr, "FullSOCthr=");
	if (section_ptr) {
		sscanf(section_ptr, "FullSOCthr=%hx", &bat->fullsocthr);
		dev_info(fuelgauge->dev, "FullSOCthr found, using FullSOCthr: 0x%04x\n", bat->fullsocthr);
	} else {
		bat->fullsocthr = 0x5F00;
		dev_info(fuelgauge->dev, "FullSOCthr not found, using FullSOCthr: 0x%04x\n", bat->fullsocthr);
	}

	section_ptr = _strcasestr(ptr, "TGain=");
	if (section_ptr) {
		sscanf(section_ptr, "TGain=%hx", &bat->tgain);
		dev_info(fuelgauge->dev, "TGain found, using TGain: 0x%04x\n", bat->tgain);
	} else {
		bat->tgain = 0xEE56;
		dev_info(fuelgauge->dev, "TGain not found, using TGain: 0x%04x\n", bat->tgain);
	}

	section_ptr = _strcasestr(ptr, "TOff=");
	if (section_ptr) {
		sscanf(section_ptr, "TOff=%hx", &bat->toff);
		dev_info(fuelgauge->dev, "TOff found, using TOff: 0x%04x\n", bat->toff);
	} else {
		bat->toff = 0x1DA4;
		dev_info(fuelgauge->dev, "TOff not found, using TOff: 0x%04x\n", bat->toff);
	}

	section_ptr = _strcasestr(ptr, "Curve=");
	if (section_ptr) {
		sscanf(section_ptr, "Curve=%hx", &bat->curve);
		dev_info(fuelgauge->dev, "Curve found, using Curve: 0x%04x\n", bat->curve);
	} else {
		bat->curve = 0x3025;
		dev_info(fuelgauge->dev, "Curve not found, using Curve: 0x%04x\n", bat->curve);
	}

	/* Parse model data table - find start after 16 consecutive 0x0000 values */
	section_ptr = ptr;
	int zero_count = 0;
	bool found_start = false;

	while ((section_ptr = strstr(section_ptr, "0x"))) {
		if (sscanf(section_ptr, "0x%hx", &val) == 1) {
			if (val == 0x0000) {
				zero_count++;
				section_ptr += 2;
			} else {
				if (zero_count >= 16) {
					/* Found first non-zero after 16 zeros - this is model data start */
					ptr = section_ptr;
					found_start = true;
					break;
				}
				zero_count = 0;
				section_ptr += 2;
			}
		} else {
			section_ptr++;
		}
	}

	if (!found_start) {
		dev_err(fuelgauge->dev, "%s: Failed to find model data in ini file\n", __func__);
		release_firmware(fw);
		return -EINVAL;
	}

	/* Parse 0x80 ~ 0x8F */
	for (i = 0; i < 16; i++) {
		if (ptr && sscanf(ptr, "0x%hx", &val) == 1) {
			bat->model_data[0][i] = val;
			ptr = strstr(ptr + 2, "0x");
		} else {
			dev_err(fuelgauge->dev, "%s: Failed to parse model_data[0][%d]\n", __func__, i);
			release_firmware(fw);
			return -EINVAL;
		}
	}

	/* Parse 0x90 ~ 0x9F */
	for (i = 0; i < 16; i++) {
		if (ptr && sscanf(ptr, "0x%hx", &val) == 1) {
			bat->model_data[1][i] = val;
			ptr = strstr(ptr + 2, "0x");
		} else {
			dev_err(fuelgauge->dev, "%s: Failed to parse model_data[1][%d]\n", __func__, i);
			release_firmware(fw);
			return -EINVAL;
		}
	}

	release_firmware(fw);
	return 0;
}

static void max2035x_custom_full_ini(struct max2035x_fuelgauge *fuelgauge)
{
	int i;
	int attempt = 0;
	unsigned int reg_val, r_dqacc, r_dpacc, r_fullcapnom, r_config2;
	u32 r_vfsoc, update_capacity;
	struct max2035x_battery_data *bat = fuelgauge->battery_data;

	/* 2.3.1 Unlock Model Access */
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_TABLE_UNLOCK1, 0x0059);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_TABLE_UNLOCK2, 0x00C4);

	/* 2.3.2 Write/Read/Verify the Custom Model */
	/* Write the 32-word custom model (0x80 - 0x9F) */
	for (i = 0; i < 16; i++) {
		regmap_write(fuelgauge->regmap, 0x80 + i, bat->model_data[0][i]);
	}
	for (i = 0; i < 16; i++) {
		regmap_write(fuelgauge->regmap, 0x90 + i, bat->model_data[1][i]);
	}

	/* Write RCompSeg (0xAF) */
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_RCOMPSEG, bat->rcompseg);

	/* Read back and Verify */
	for (i = 0; i < 16; i++) {
		/* 0x80 - 0x8F */
		regmap_read(fuelgauge->regmap, 0x80 + i, &reg_val);
		if (reg_val != bat->model_data[0][i]) {
			dev_warn(fuelgauge->dev, "%s: Model verification failed at 0x%02x: expected 0x%04x, read 0x%04x\n",
				__func__, 0x80 + i, bat->model_data[0][i], reg_val);
		}

		/* 0x90 ~ 0x9F */
		regmap_read(fuelgauge->regmap, 0x90 + i, &reg_val);
		if (reg_val != bat->model_data[1][i]) {
			dev_warn(fuelgauge->dev, "%s: Model verification failed at 0x%02x: expected 0x%04x, read 0x%04x\n",
				__func__, 0x90 + i, bat->model_data[1][i], reg_val);
		}
	}

	/* Verify RCompSeg */
	regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_RCOMPSEG, &reg_val);
	if (reg_val != bat->rcompseg) {
		dev_warn(fuelgauge->dev, "%s: RCompSeg verification failed: expected 0x%04x, read 0x%04x\n",
			__func__, bat->rcompseg, reg_val);
	}

	/* 2.3.3 Lock Model Access */
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_TABLE_UNLOCK1, 0x0000);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_TABLE_UNLOCK2, 0x0000);

	/* 2.3.4 Verify that Model Access is locked */
	for (i = 0x80; i <= 0x9F; i++) {
		regmap_read(fuelgauge->regmap, i, &reg_val);
		if (reg_val != 0x0000 && reg_val != 0xFFFF) {
			dev_warn(fuelgauge->dev, "%s: Model Access lock verification: 0x%02x reads 0x%04x (expected 0x0000 or 0xFFFF)\n",
				__func__, i, reg_val);
		}
	}

	/* Step 2.3.5: Wait 100ms before writing custom parameters */
	msleep(100);

	/* 2.3.5 Write Custom Parameters */
	max2035x_write_verify_reg(fuelgauge->regmap, MAX2035X_FG_REG_REPCAP, 0x0000);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_DESIGNCAP, bat->designcap);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_FULLCAPREP, bat->fullcaprep);

	do {
		regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_DQACC, (bat->fullcapnom) / 2);
		regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_DPACC, 0x0C80);
		msleep(10);
		regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_FULLCAPNOM, bat->fullcapnom);

		regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_DQACC, &r_dqacc);
		regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_DPACC, &r_dpacc);
		regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_FULLCAPNOM, &r_fullcapnom);

		if ((r_dqacc == (bat->fullcapnom) / 2)
			&& (r_dpacc == 0x0C80)
			&& (r_fullcapnom == bat->fullcapnom)) {
			break;
		}

		dev_warn(fuelgauge->dev, "%s: Verify failed, retrying... (attempt %d)\n", __func__, attempt + 1);
	} while (++attempt < 3);

	regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_VFSOC, &r_vfsoc);
	update_capacity = (u32)div_u64((u64)r_vfsoc * bat->fullcapnom, 25600);

	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_MIXCAP, update_capacity);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_AVCAP, update_capacity);

	msleep(200);

	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_ICHGTERM, bat->ichgterm);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_VEMPTY, bat->vempty);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_RCOMP0, bat->rcomp0);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_TEMPCO, bat->tempco);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_QRTABLE00, bat->qr_table00);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_QRTABLE10, bat->qr_table10);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_QRTABLE20, bat->qr_table20);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_QRTABLE30, bat->qr_table30);

	/* 2.3.6 Updating optional registers */
	max2035x_write_verify_reg(fuelgauge->regmap, MAX2035X_FG_REG_LEARNCFG, bat->learncfg);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_RELAXCFG, bat->relaxcfg);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_CONFIG, bat->config);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_CONFIG2, bat->config2);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_FULLSOCTHR, bat->fullsocthr);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_TGAIN, bat->tgain);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_TOFF, bat->toff);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_CURVE, bat->curve);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_MISCCFG, bat->misccfg);

	/* 2.3.7 Initiate Model Loading */
	regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_CONFIG2, &r_config2);
	regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_CONFIG2, r_config2 | 0x0020);

	/* Poll LdMdl bit until it becomes 0 */
	for (i = 0; i < 40; i++) {
		regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_CONFIG2, &r_config2);

		if (!(r_config2 & 0x0020))
			break;

		regmap_write(fuelgauge->regmap, 0x0A, 0x0000);
		regmap_write(fuelgauge->regmap, 0x0B, 0x0000);

		msleep(10);
	}

	if (i == 40) {
		dev_err(fuelgauge->dev, "%s: Model loading failed: LdMdl bit remain at 1\n", __func__);
	}

	/* 2.3.8 Update QRTable20, QRTable30 and Cycles */
	max2035x_write_verify_reg(fuelgauge->regmap, MAX2035X_FG_REG_QRTABLE20, bat->qr_table20);
	max2035x_write_verify_reg(fuelgauge->regmap, MAX2035X_FG_REG_QRTABLE30, bat->qr_table30);
	max2035x_write_verify_reg(fuelgauge->regmap, MAX2035X_FG_REG_CYCLES, bat->cycles);
}

static int max2035x_initialize_fuelgauge(struct max2035x_fuelgauge *fuelgauge)
{
	unsigned int r_status, r_fstat, r_hibcfg;
	int count, retry = 0;

	/* <Step 0> Check for POR */
	for (retry = 0; retry < 5; retry++) {
	    regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_STATUS, &r_status);

		if (!(r_status & MAX2035X_FG_STATUS_POR)) {
			dev_info(fuelgauge->dev, "%s: Fuelgauge already initialized (POR=0)\n", __func__);
			goto step_3_2;
		}

		dev_info(fuelgauge->dev, "%s: POR detected (%d/5), starting initialization...\n", __func__, retry + 1);

		count = 0;
		while (count < 50) {
			/* <Step 1> Delay until FSTAT.DNR bit == 0 */
			regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_FSTAT, &r_fstat);

			if (!(r_fstat & MAX2035X_FG_FSTAT_DNR))
				break;

			msleep(10);
			count++;
		}

		/* <Step 2> Initialize Configuration */
		regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_HIBCFG, &r_hibcfg);
		regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_SOFT_WAKEUP, 0x0090);
		regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_HIBCFG, 0x0000);
		regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_SOFT_WAKEUP, 0x0000);

		max2035x_custom_full_ini(fuelgauge);

		regmap_write(fuelgauge->regmap, MAX2035X_FG_REG_HIBCFG, r_hibcfg);

		/* <Step 3> Initialize Complete */
		regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_STATUS, &r_status);
		max2035x_write_verify_reg(fuelgauge->regmap, MAX2035X_FG_REG_STATUS, r_status & 0xFFFD);

		/* 3.1 Check for IC Reset */
		regmap_read(fuelgauge->regmap, MAX2035X_FG_REG_STATUS, &r_status);
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
	fuelgauge->i2c = chip->fuelgauge;
	fuelgauge->chip = chip;

	fuelgauge->regmap = devm_regmap_init_i2c(fuelgauge->i2c, &max2035x_fg_regmap_cfg);
	if (IS_ERR(fuelgauge->regmap))
		return dev_err_probe(chip->dev, PTR_ERR(fuelgauge->regmap), "%s : Failed to initialize regmap\n", __func__);

	/* Set rsense to 200 mohm */
	fuelgauge->rsense_mohm = 200;

	platform_set_drvdata(pdev, fuelgauge);

	ret = max2035x_load_ini_file(fuelgauge);
	if (ret) {
		dev_err(&pdev->dev, "%s : Failed to load INI file (ret: %d), skipping fuelgauge initialization\n", __func__, ret);
	} else {
		ret = max2035x_initialize_fuelgauge(fuelgauge);
		if (ret)
			dev_err(&pdev->dev, "%s : Failed to initialize fuelgauge (ret: %d)\n", __func__, ret);
	}

	dev_info(&pdev->dev, "%s : %s Fuelgauge driver Probed\n",
			__func__, (chip->type == MAX20355) ? "MAX20355" : "MAX20357");

	return 0;
}

static void max2035x_fuelgauge_remove(struct platform_device *pdev)
{
	struct max2035x_fuelgauge *fuelgauge = platform_get_drvdata(pdev);

	if (!fuelgauge)
		return;

	dev_info(&pdev->dev, "%s : Fuelgauge driver removed\n", __func__);
}

static struct platform_driver max20355_fuelgauge_driver = {
	.driver = {
		.name = "max20355-fuelgauge",
	},
	.probe = max2035x_fuelgauge_probe,
	.remove = max2035x_fuelgauge_remove,
};

static struct platform_driver max20357_fuelgauge_driver = {
	.driver = {
		.name = "max20357-fuelgauge",
	},
	.probe = max2035x_fuelgauge_probe,
	.remove = max2035x_fuelgauge_remove,
};

static struct platform_driver * const fuelgauge_drivers[] = {
	&max20355_fuelgauge_driver,
	&max20357_fuelgauge_driver,
};

int __init max2035x_fuelgauge_init(void)
{
	return platform_register_drivers(fuelgauge_drivers, ARRAY_SIZE(fuelgauge_drivers));
}

void max2035x_fuelgauge_exit(void)
{
	platform_unregister_drivers(fuelgauge_drivers, ARRAY_SIZE(fuelgauge_drivers));
}