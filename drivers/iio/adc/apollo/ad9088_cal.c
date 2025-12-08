// SPDX-License-Identifier: GPL-2.0
/*
 * AD9088 Calibration Data Save/Restore
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/iio/iio.h>
#include <linux/sysfs.h>
#include <linux/firmware.h>
#include <linux/of.h>
#include "ad9088.h"
#include "../cf_axi_adc.h"

/*
 * Calibration File Format (Version 2)
 * ====================================
 *
 * The calibration data is organized as follows:
 *
 * +------------------------+
 * | Header (64 bytes)      |  <- Fixed size header with magic, version, metadata
 * +------------------------+
 * | ADC Cal Data           |  <- All ADC channels, both sequential and random modes
 * +------------------------+
 * | SERDES RX Cal Data     |  <- All SERDES RX 12-packs
 * +------------------------+
 * | Clock Cond Cal Data    |  <- Clock conditioning cal for both sides
 * +------------------------+
 * | CRC32 (4 bytes)        |  <- Checksum of entire file excluding this field
 * +------------------------+
 *
 * Note: Header structure and constants are defined in ad9088.h
 */

/**
 * ad9088_cal_save - Save all calibration data to buffer
 * @phy: AD9088 device structure
 * @buf: Pointer to receive allocated buffer
 * @len: Pointer to receive buffer length
 *
 * Returns: 0 on success, negative error code on failure
 */
int ad9088_cal_save(struct ad9088_phy *phy, u8 **buf, size_t *len)
{
	struct ad9088_cal_header *hdr;
	adi_apollo_device_t *device = &phy->ad9088;
	u8 *ptr, *cal_buf;
	u32 adc_len_per_mode = 0, adc_len_per_chan, serdes_rx_len, clk_cond_len;
	u32 total_size, current_offset;
	u32 crc;
	int ret, i, mode;
	u16 adc_cal_chans;
	u8 num_adcs;
	u16 serdes_jrx_enabled_mask = 0;
	adi_apollo_serdes_bgcal_state_t serdes_state[ADI_APOLLO_NUM_JRX_SERDES_12PACKS];
	adi_apollo_mailbox_resp_update_cal_data_crc_t crc_resp;

	dev_dbg(&phy->spi->dev, "Saving calibration data...\n");

	/* Determine device configuration */
	adc_cal_chans = device->dev_info.is_8t8r ? ADI_APOLLO_ADC_ALL : ADI_APOLLO_ADC_ALL_4T4R;
	num_adcs = device->dev_info.is_8t8r ? 8 : 4;

	/* Query which SERDES JRX packs are enabled */
	ret = adi_apollo_serdes_jrx_bgcal_state_get(device, ADI_APOLLO_TXRX_SERDES_12PACK_ALL,
						    serdes_state, ADI_APOLLO_NUM_JRX_SERDES_12PACKS);
	if (ret) {
		dev_err(&phy->spi->dev, "Failed to get SERDES JRX bgcal state: %d\n", ret);
		return -EFAULT;
	}

	/* Build mask of enabled SERDES packs */
	for (i = 0; i < ADI_APOLLO_NUM_JRX_SERDES_12PACKS; i++) {
		if (serdes_state[i].state_valid &&
		    (serdes_state[i].bgcal_state & ADI_APOLLO_SERDES_BGCAL_STATE_ENABLED)) {
			serdes_jrx_enabled_mask |= (ADI_APOLLO_TXRX_SERDES_12PACK_A << i);
			dev_dbg(&phy->spi->dev, "SERDES JRX Pack %d is enabled (state: 0x%x)\n",
				i, serdes_state[i].bgcal_state);
		}
	}

	if (serdes_jrx_enabled_mask == 0)
		dev_dbg(&phy->spi->dev, "No SERDES JRX packs are enabled for bgcal\n");
	else
		dev_dbg(&phy->spi->dev, "SERDES JRX enabled mask: 0x%04x\n", serdes_jrx_enabled_mask);

	/* Get calibration data sizes */
	ret = adi_apollo_cfg_adc_cal_data_len_get(device, ADI_APOLLO_ADC_CAL_SEQUENTIAL_MODE, &adc_len_per_chan);
	if (ret) {
		dev_err(&phy->spi->dev, "Failed to get ADC cal data length: %d\n", ret);
		return -EFAULT;
	}

	adc_len_per_mode = adc_len_per_chan * num_adcs;  /* Total for one mode (seq or random) */

	ret = adi_apollo_cfg_serdes_rx_cal_data_len_get(device, &serdes_rx_len);
	if (ret) {
		dev_err(&phy->spi->dev, "Failed to get SERDES RX cal data length: %d\n", ret);
		return -EFAULT;
	}

	ret = adi_apollo_cfg_clk_cond_cal_data_len_get(device, &clk_cond_len);
	if (ret) {
		dev_err(&phy->spi->dev, "Failed to get clock conditioning cal data length: %d\n", ret);
		return -EFAULT;
	}

	/* Calculate total size:
	 * - Header
	 * - ADC cal data (2 modes * num_adcs * size_per_adc)
	 * - SERDES RX cal data (num_serdes_rx * size_per_serdes)
	 * - Clock conditioning cal data (2 sides * size_per_side)
	 * - CRC32
	 */
	total_size = sizeof(struct ad9088_cal_header) +
		     (adc_len_per_mode * 2) +  /* 2 modes: sequential and random */
		     (serdes_rx_len * ADI_APOLLO_NUM_JTX_SERDES_12PACKS) +
		     (clk_cond_len * ADI_APOLLO_NUM_SIDES) +
		     sizeof(u32);  /* CRC32 */

	/* Allocate buffer */
	cal_buf = kzalloc(total_size, GFP_KERNEL);
	if (!cal_buf)
		return -ENOMEM;

	ptr = cal_buf;

	/* Fill header */
	hdr = (struct ad9088_cal_header *)ptr;
	hdr->magic = AD9088_CAL_MAGIC;
	hdr->version = AD9088_CAL_VERSION;
	hdr->chip_id = phy->chip_id.chip_type;
	hdr->is_8t8r = device->dev_info.is_8t8r;
	hdr->num_adcs = num_adcs;
	hdr->num_serdes_rx = ADI_APOLLO_NUM_JTX_SERDES_12PACKS;
	hdr->num_clk_cond = ADI_APOLLO_NUM_SIDES;
	hdr->total_size = total_size;

	/* Set up section offsets and sizes */
	current_offset = sizeof(struct ad9088_cal_header);

	hdr->adc_cal_offset = current_offset;
	hdr->adc_cal_size = adc_len_per_mode * 2;
	current_offset += hdr->adc_cal_size;

	hdr->serdes_rx_cal_offset = current_offset;
	hdr->serdes_rx_cal_size = serdes_rx_len * ADI_APOLLO_NUM_JTX_SERDES_12PACKS;
	current_offset += hdr->serdes_rx_cal_size;

	hdr->clk_cond_cal_offset = current_offset;
	hdr->clk_cond_cal_size = clk_cond_len * ADI_APOLLO_NUM_SIDES;

	ptr += sizeof(struct ad9088_cal_header);

	/* Read ADC calibration data (both sequential and random modes) */
	/* Freeze ADC background calibration before reading */
	dev_dbg(&phy->spi->dev, "Freezing ADC background calibration...\n");
	ret = adi_apollo_adc_bgcal_freeze(device, adc_cal_chans);
	if (ret) {
		dev_err(&phy->spi->dev, "Failed to freeze ADC bgcal: %d\n", ret);
		kfree(cal_buf);
		return -EFAULT;
	}

	/* Freeze SERDES JRX background calibration (only for enabled packs) */
	if (serdes_jrx_enabled_mask) {
		dev_dbg(&phy->spi->dev, "Freezing SERDES JRX background calibration (mask: 0x%04x)...\n",
			 serdes_jrx_enabled_mask);
		ret = adi_apollo_serdes_jrx_bgcal_freeze(&phy->ad9088, serdes_jrx_enabled_mask);
		if (ret) {
			dev_err(&phy->spi->dev, "Failed to freeze SERDES JRX bgcal: %d\n", ret);
			adi_apollo_adc_bgcal_unfreeze(device, adc_cal_chans);
			kfree(cal_buf);
			return -EFAULT;
		}
	}

	/* Wait for freeze to take effect */
	msleep(5);

	/* Update CRC in firmware before reading data */
	dev_dbg(&phy->spi->dev, "Updating calibration data CRC...\n");
	ret = adi_apollo_mailbox_update_cal_data_crc(device, &crc_resp);
	if (ret) {
		dev_err(&phy->spi->dev, "Failed to update cal data CRC: %d\n", ret);
		/* Unfreeze bgcal before returning */
		adi_apollo_adc_bgcal_unfreeze(device, adc_cal_chans);
		if (serdes_jrx_enabled_mask)
			adi_apollo_serdes_jrx_bgcal_unfreeze(device, serdes_jrx_enabled_mask);
		kfree(cal_buf);
		return -EFAULT;
	}
	dev_dbg(&phy->spi->dev, "CRC update status: %u\n", crc_resp.status);

	/* Read ADC calibration data (both sequential and random modes) */
	dev_dbg(&phy->spi->dev, "Reading ADC calibration data...\n");
	for (mode = 0; mode < 2; mode++) {
		for (i = 0; i < ADI_APOLLO_ADC_NUM; i++) {
			if (adc_cal_chans & (ADI_APOLLO_ADC_A0 << i)) {
				ret = adi_apollo_cfg_adc_cal_data_get(device,
					ADI_APOLLO_ADC_A0 << i, mode, ptr, adc_len_per_chan);
				if (ret) {
					dev_err(&phy->spi->dev,
						"Failed to get ADC%d mode%d cal data: %d\n",
						i, mode, ret);
					/* Unfreeze bgcal before returning */
					adi_apollo_adc_bgcal_unfreeze(device, adc_cal_chans);
					if (serdes_jrx_enabled_mask)
						adi_apollo_serdes_jrx_bgcal_unfreeze(device, serdes_jrx_enabled_mask);
					kfree(cal_buf);
					return -EFAULT;
				}
				ptr += adc_len_per_chan;
			}
		}
	}

	/* Unfreeze ADC background calibration after reading */
	dev_dbg(&phy->spi->dev, "Unfreezing ADC background calibration...\n");
	ret = adi_apollo_adc_bgcal_unfreeze(device, adc_cal_chans);
	if (ret)
		dev_warn(&phy->spi->dev, "Failed to unfreeze ADC bgcal: %d\n", ret);

	/* Unfreeze SERDES JRX background calibration after reading */
	if (serdes_jrx_enabled_mask) {
		dev_dbg(&phy->spi->dev, "Unfreezing SERDES JRX background calibration...\n");
		ret = adi_apollo_serdes_jrx_bgcal_unfreeze(device, serdes_jrx_enabled_mask);
		if (ret)
			dev_warn(&phy->spi->dev, "Failed to unfreeze SERDES JRX bgcal: %d\n", ret);
	}

	/* Read SERDES RX calibration data */
	dev_dbg(&phy->spi->dev, "Reading SERDES RX calibration data...\n");
	for (i = 0; i < ADI_APOLLO_NUM_JTX_SERDES_12PACKS; i++) {
		ret = adi_apollo_cfg_serdes_rx_cal_data_get(device,
			ADI_APOLLO_TXRX_SERDES_12PACK_A << i, ptr, serdes_rx_len);
		if (ret) {
			dev_err(&phy->spi->dev,
				"Failed to get SERDES RX%d cal data: %d\n", i, ret);
			kfree(cal_buf);
			return -EFAULT;
		}
		ptr += serdes_rx_len;
	}

	/* Read clock conditioning calibration data */
	dev_dbg(&phy->spi->dev, "Reading clock conditioning calibration data...\n");
	for (i = 0; i < ADI_APOLLO_NUM_SIDES; i++) {
		ret = adi_apollo_cfg_clk_cond_cal_data_get(device,
			ADI_APOLLO_SIDE_A << i, ptr, clk_cond_len);
		if (ret) {
			dev_err(&phy->spi->dev,
				"Failed to get clock conditioning Side %c cal data: %d\n",
				'A' + i, ret);
			kfree(cal_buf);
			return -EFAULT;
		}
		ptr += clk_cond_len;
	}

	/* Calculate and append CRC32 of everything except the CRC itself */
	crc = crc32_le(~0, cal_buf, total_size - sizeof(u32));
	crc = ~crc;
	memcpy(ptr, &crc, sizeof(u32));

	dev_info(&phy->spi->dev,
		 "Calibration data saved: %u bytes (ADC: %u, SERDES RX: %u, Clk Cond: %u)\n",
		 total_size, hdr->adc_cal_size, hdr->serdes_rx_cal_size,
		 hdr->clk_cond_cal_size);

	*buf = cal_buf;
	*len = total_size;

	return 0;
}

/**
 * ad9088_cal_restore - Restore calibration data from buffer
 * @phy: AD9088 device structure
 * @buf: Buffer containing calibration data
 * @len: Buffer length
 *
 * Returns: 0 on success, negative error code on failure
 */
int ad9088_cal_restore(struct ad9088_phy *phy, const u8 *buf, size_t len)
{
	struct ad9088_cal_header *hdr;
	adi_apollo_device_t *device = &phy->ad9088;
	const u8 *ptr;
	u32 crc, crc_calc;
	u32 adc_len_per_chan, serdes_rx_len, clk_cond_len;
	int ret, i, mode;
	u16 adc_cal_chans;
	u16 serdes_jrx_enabled_mask = 0;
	adi_apollo_serdes_bgcal_state_t serdes_state[ADI_APOLLO_NUM_JRX_SERDES_12PACKS];

	dev_dbg(&phy->spi->dev, "Restoring calibration data...\n");

	/* Validate minimum size */
	if (len < sizeof(struct ad9088_cal_header) + sizeof(u32)) {
		dev_err(&phy->spi->dev, "Calibration buffer too small\n");
		return -EINVAL;
	}

	hdr = (struct ad9088_cal_header *)buf;

	/* Validate magic number */
	if (hdr->magic != AD9088_CAL_MAGIC) {
		dev_err(&phy->spi->dev,
			"Invalid calibration magic: 0x%08x (expected 0x%08x)\n",
			hdr->magic, AD9088_CAL_MAGIC);
		return -EINVAL;
	}

	/* Validate version */
	if (hdr->version != AD9088_CAL_VERSION) {
		dev_err(&phy->spi->dev,
			"Unsupported calibration version: %u (expected %u)\n",
			hdr->version, AD9088_CAL_VERSION);
		return -EINVAL;
	}

	/* Validate total size */
	if (hdr->total_size != len) {
		dev_err(&phy->spi->dev,
			"Calibration size mismatch: %u vs %zu\n",
			hdr->total_size, len);
		return -EINVAL;
	}

	/* Validate chip ID */
	if (hdr->chip_id != phy->chip_id.chip_type) {
		dev_err(&phy->spi->dev,
			"Chip ID mismatch: 0x%04x vs 0x%04x\n",
			hdr->chip_id, phy->chip_id.chip_type);
		return -EINVAL;
	}

	/* Validate device configuration */
	if (hdr->is_8t8r != device->dev_info.is_8t8r) {
		dev_err(&phy->spi->dev,
			"Device configuration mismatch: %s vs %s\n",
			hdr->is_8t8r ? "8T8R" : "4T4R",
			device->dev_info.is_8t8r ? "8T8R" : "4T4R");
		return -EINVAL;
	}

	/* Verify CRC32 */
	memcpy(&crc, buf + len - sizeof(u32), sizeof(u32));
	crc_calc = crc32_le(~0, buf, len - sizeof(u32));
	crc_calc = ~crc_calc;

	if (crc != crc_calc) {
		dev_err(&phy->spi->dev,
			"CRC mismatch: 0x%08x vs 0x%08x\n", crc, crc_calc);
		return -EINVAL;
	}

	/* Determine device configuration */
	adc_cal_chans = device->dev_info.is_8t8r ? ADI_APOLLO_ADC_ALL : ADI_APOLLO_ADC_ALL_4T4R;

	/* Query which SERDES JRX packs are enabled */
	ret = adi_apollo_serdes_jrx_bgcal_state_get(device, ADI_APOLLO_TXRX_SERDES_12PACK_ALL,
						    serdes_state, ADI_APOLLO_NUM_JRX_SERDES_12PACKS);
	if (ret) {
		dev_err(&phy->spi->dev, "Failed to get SERDES JRX bgcal state: %d\n", ret);
		return -EFAULT;
	}

	/* Build mask of enabled SERDES packs */
	for (i = 0; i < ADI_APOLLO_NUM_JRX_SERDES_12PACKS; i++) {
		if (serdes_state[i].state_valid &&
		    (serdes_state[i].bgcal_state & ADI_APOLLO_SERDES_BGCAL_STATE_ENABLED)) {
			serdes_jrx_enabled_mask |= (ADI_APOLLO_TXRX_SERDES_12PACK_A << i);
			dev_dbg(&phy->spi->dev, "SERDES JRX Pack %d is enabled (state: 0x%x)\n",
				i, serdes_state[i].bgcal_state);
		}
	}

	if (serdes_jrx_enabled_mask == 0)
		dev_warn(&phy->spi->dev, "No SERDES JRX packs are enabled for bgcal\n");
	else
		dev_dbg(&phy->spi->dev, "SERDES JRX enabled mask: 0x%04x\n", serdes_jrx_enabled_mask);

	/* Get expected sizes */
	ret = adi_apollo_cfg_adc_cal_data_len_get(device, ADI_APOLLO_ADC_CAL_SEQUENTIAL_MODE, &adc_len_per_chan);
	if (ret) {
		dev_err(&phy->spi->dev, "Failed to get ADC cal data length: %d\n", ret);
		return -EFAULT;
	}

	ret = adi_apollo_cfg_serdes_rx_cal_data_len_get(device, &serdes_rx_len);
	if (ret) {
		dev_err(&phy->spi->dev, "Failed to get SERDES RX cal data length: %d\n", ret);
		return -EFAULT;
	}

	ret = adi_apollo_cfg_clk_cond_cal_data_len_get(device, &clk_cond_len);
	if (ret) {
		dev_err(&phy->spi->dev, "Failed to get clock conditioning cal data length: %d\n", ret);
		return -EFAULT;
	}

	/* Restore ADC calibration data */
	dev_dbg(&phy->spi->dev, "Restoring ADC calibration data...\n");
	ptr = buf + hdr->adc_cal_offset;
	for (mode = 0; mode < 2; mode++) {
		for (i = 0; i < ADI_APOLLO_ADC_NUM; i++) {
			if (adc_cal_chans & (ADI_APOLLO_ADC_A0 << i)) {
				ret = adi_apollo_cfg_adc_cal_data_set(device,
					ADI_APOLLO_ADC_A0 << i, mode, (u8 *)ptr, adc_len_per_chan);
				if (ret) {
					dev_err(&phy->spi->dev,
						"Failed to set ADC%d mode%d cal data: %d\n",
						i, mode, ret);
					return -EFAULT;
				}
				ptr += adc_len_per_chan;
			}
		}
	}

	/* Restore SERDES RX calibration data */
	dev_dbg(&phy->spi->dev, "Restoring SERDES RX calibration data...\n");
	ptr = buf + hdr->serdes_rx_cal_offset;
	for (i = 0; i < ADI_APOLLO_NUM_JTX_SERDES_12PACKS; i++) {
		ret = adi_apollo_cfg_serdes_rx_cal_data_set(device,
			ADI_APOLLO_TXRX_SERDES_12PACK_A << i, (u8 *)ptr, serdes_rx_len);
		if (ret) {
			dev_err(&phy->spi->dev,
				"Failed to set SERDES RX%d cal data: %d\n", i, ret);
			return -EFAULT;
		}
		ptr += serdes_rx_len;
	}

	/* Restore clock conditioning calibration data */
	dev_dbg(&phy->spi->dev, "Restoring clock conditioning calibration data...\n");
	ptr = buf + hdr->clk_cond_cal_offset;
	for (i = 0; i < ADI_APOLLO_NUM_SIDES; i++) {
		ret = adi_apollo_cfg_clk_cond_cal_data_set(device,
			ADI_APOLLO_SIDE_A << i, (u8 *)ptr, clk_cond_len);
		if (ret) {
			dev_err(&phy->spi->dev,
				"Failed to set clock conditioning Side %c cal data: %d\n",
				'A' + i, ret);
			return -EFAULT;
		}
		ptr += clk_cond_len;
	}

	dev_dbg(&phy->spi->dev, "Calibration data restored successfully\n");

	return 0;
}

/**
 * ad9088_cal_load_from_firmware - Load and restore calibration data from firmware file
 * @phy: AD9088 PHY device structure
 *
 * This function reads the calibration firmware filename from the device tree
 * property "adi,device-calibration-data-name" and loads the calibration data
 * from the firmware directory. If the property is not present, this function
 * returns success without doing anything (optional feature).
 *
 * Returns: 0 on success, negative error code on failure
 */
int ad9088_cal_load_from_firmware(struct ad9088_phy *phy)
{
	struct device *dev = &phy->spi->dev;
	struct device_node *node = dev->of_node;
	const struct firmware *fw;
	const char *name;
	int ret;

	phy->cal_data_loaded_from_fw = false;

	/* Check if calibration data firmware name is specified in device tree */
	ret = of_property_read_string(node, "adi,device-calibration-data-name", &name);
	if (ret) {
		/* Property not present - this is optional, so return success */
		dev_dbg(dev, "No calibration firmware specified in device tree\n");
		return 0;
	}

	dev_dbg(dev, "Loading calibration data from firmware: %s\n", name);

	/* Request the firmware file */
	ret = request_firmware(&fw, name, dev);
	if (ret) {
		dev_err(dev, "Failed to load calibration firmware '%s': %d\n", name, ret);
		return ret;
	}

	/* Validate firmware size */
	if (fw->size < sizeof(struct ad9088_cal_header)) {
		dev_err(dev, "Calibration firmware '%s' too small (%zu bytes)\n",
			name, fw->size);
		ret = -EINVAL;
		goto out_release_fw;
	}

	/* Restore calibration data to hardware */
	ret = ad9088_cal_restore(phy, fw->data, fw->size);
	if (ret) {
		dev_err(dev, "Failed to restore calibration data from firmware: %d\n", ret);
		goto out_release_fw;
	}

	dev_dbg(dev, "Calibration data loaded and restored successfully from %s\n", name);

	phy->cal_data_loaded_from_fw = true;

out_release_fw:
	release_firmware(fw);
	return ret;
}

/**
 * ad9088_cal_data_read - Sysfs bin_attribute read for calibration data
 * @filp: File pointer
 * @kobj: Kernel object
 * @bin_attr: Binary attribute
 * @buf: Buffer to read into
 * @off: Offset in file
 * @count: Number of bytes to read
 *
 * Returns: Number of bytes read, or negative error code
 */
ssize_t ad9088_cal_data_read(struct file *filp, struct kobject *kobj,
			     struct bin_attribute *bin_attr,
			     char *buf, loff_t off, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9088_phy *phy = conv->phy;
	u8 *cal_buf = NULL;
	size_t cal_len = 0;
	ssize_t ret_count;
	int ret;

	/* Generate calibration data on first read or if not cached */
	if (off == 0) {
		guard(mutex)(&phy->lock);

		/* Free old buffer if exists */
		kfree(phy->nvm_adc_cal);
		phy->nvm_adc_cal = NULL;

		/* Save current calibration data */
		ret = ad9088_cal_save(phy, &cal_buf, &cal_len);
		if (ret) {
			dev_err(&phy->spi->dev, "Failed to save calibration data: %d\n", ret);
			return ret;
		}

		/* Cache the buffer for subsequent reads */
		phy->nvm_adc_cal = cal_buf;
		phy->adc_cal_len = cal_len;
	}

	/* Validate offset */
	if (off >= phy->adc_cal_len)
		return 0;

	/* Calculate bytes to read */
	ret_count = min_t(size_t, count, phy->adc_cal_len - off);

	/* Copy data to user buffer */
	memcpy(buf, phy->nvm_adc_cal + off, ret_count);

	return ret_count;
}

/**
 * ad9088_cal_data_write - Sysfs bin_attribute write for calibration data
 * @filp: File pointer
 * @kobj: Kernel object
 * @bin_attr: Binary attribute
 * @buf: Buffer to write from
 * @off: Offset in file
 * @count: Number of bytes to write
 *
 * Handles multi-write operations by accumulating data in a buffer until
 * the entire calibration file is received, then restores it.
 *
 * Returns: Number of bytes written, or negative error code
 */
ssize_t ad9088_cal_data_write(struct file *filp, struct kobject *kobj,
			      struct bin_attribute *bin_attr,
			      char *buf, loff_t off, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9088_phy *phy = conv->phy;
	struct ad9088_cal_header *hdr;
	int ret = 0;

	guard(mutex)(&phy->lock);

	/* First write - read header and allocate buffer */
	if (off == 0) {
		/* Free any existing buffer */
		if (phy->cal_restore_buf) {
			vfree(phy->cal_restore_buf);
			phy->cal_restore_buf = NULL;
			phy->cal_restore_size = 0;
			phy->cal_restore_received = 0;
		}

		/* Need at least header to determine size */
		if (count < sizeof(struct ad9088_cal_header)) {
			dev_err(&phy->spi->dev,
				"First write too small: %zu bytes (need at least %zu)\n",
				count, sizeof(struct ad9088_cal_header));
			return -EINVAL;
		}

		/* Read header to get total size */
		hdr = (struct ad9088_cal_header *)buf;

		/* Validate magic */
		if (hdr->magic != AD9088_CAL_MAGIC) {
			dev_err(&phy->spi->dev,
				"Invalid calibration magic: 0x%08x\n", hdr->magic);
			return -EINVAL;
		}

		/* Validate version */
		if (hdr->version != AD9088_CAL_VERSION) {
			dev_err(&phy->spi->dev,
				"Unsupported calibration version: %u\n", hdr->version);
			return -EINVAL;
		}

		/* Allocate buffer for entire file */
		phy->cal_restore_size = hdr->total_size;
		phy->cal_restore_buf = vmalloc(phy->cal_restore_size);
		if (!phy->cal_restore_buf) {
			dev_err(&phy->spi->dev,
				"Failed to allocate %zu bytes for calibration restore\n",
				phy->cal_restore_size);
			phy->cal_restore_size = 0;
			return -ENOMEM;
		}

		phy->cal_restore_received = 0;
		dev_dbg(&phy->spi->dev,
			 "Starting calibration restore: %zu bytes expected\n",
			 phy->cal_restore_size);
	}

	/* Verify we have an active restore in progress */
	if (!phy->cal_restore_buf) {
		dev_err(&phy->spi->dev,
			"No calibration restore in progress (write at offset %lld)\n", off);
		return -EINVAL;
	}

	/* Verify offset is within bounds */
	if (off + count > phy->cal_restore_size) {
		dev_err(&phy->spi->dev,
			"Write exceeds calibration size: offset=%lld count=%zu size=%zu\n",
			off, count, phy->cal_restore_size);
		vfree(phy->cal_restore_buf);
		phy->cal_restore_buf = NULL;
		phy->cal_restore_size = 0;
		phy->cal_restore_received = 0;
		return -EINVAL;
	}

	/* Copy data into buffer */
	memcpy(phy->cal_restore_buf + off, buf, count);
	phy->cal_restore_received = off + count;

	dev_dbg(&phy->spi->dev,
		"Calibration write: offset=%lld count=%zu received=%zu/%zu\n",
		off, count, phy->cal_restore_received, phy->cal_restore_size);

	/* If we've received all data, restore it */
	if (phy->cal_restore_received >= phy->cal_restore_size) {
		dev_dbg(&phy->spi->dev,
			 "All calibration data received, restoring...\n");

		ret = ad9088_cal_restore(phy, phy->cal_restore_buf,
					 phy->cal_restore_size);
		if (ret) {
			dev_err(&phy->spi->dev,
				"Failed to restore calibration data: %d\n", ret);
		} else {
			dev_info(&phy->spi->dev,
				 "Calibration data restored successfully\n");
		}

		/* Free buffer */
		vfree(phy->cal_restore_buf);
		phy->cal_restore_buf = NULL;
		phy->cal_restore_size = 0;
		phy->cal_restore_received = 0;

		if (ret)
			return ret;
	}

	return count;
}
