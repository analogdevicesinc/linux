// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for AD9088 and similar mixed signal front end (MxFE®)
 *
 * Copyright 2025 Analog Devices Inc.
 */
//#define DEBUG

#include "ad9088.h"

static int ad9088_jesd_lane_setup(struct ad9088_phy *phy)
{
	struct device *dev = &phy->spi->dev;
	struct device_node *node = dev->of_node;
	int ret, i;
	u32 lane_xbar[12];
	u32 ser_amplitude = ADI_APOLLO_JESD_DRIVE_SWING_VTT_100;
	u32 ser_pre_emphasis = ADI_APOLLO_JESD_PRE_TAP_LEVEL_6_DB;
	u32 ser_post_emphasis = ADI_APOLLO_JESD_POST_TAP_LEVEL_3_DB;

	ret = of_property_read_variable_u32_array(
		      node, "adi,jtx0-logical-lane-mapping",
		      lane_xbar, 12, 12);

	if (ret == 12) {
		dev_dbg(dev, "found adi,jtx0-logical-lane-mapping\n");
		for (i = 0; i < 12; i++)
			phy->profile.jtx[0].tx_link_cfg[0].lane_xbar[i] = lane_xbar[i];
	}

	ret = of_property_read_variable_u32_array(
		      node, "adi,jtx1-logical-lane-mapping",
		      lane_xbar, 12, 12);

	if (ret == 12) {
		dev_dbg(dev, "found adi,jtx1-logical-lane-mapping\n");
		for (i = 0; i < 12; i++)
			phy->profile.jtx[1].tx_link_cfg[0].lane_xbar[i] = lane_xbar[i];
	}

	ret = of_property_read_variable_u32_array(
		      node, "adi,jrx0-physical-lane-mapping",
		      lane_xbar, 12, 12);

	if (ret == 12) {
		dev_dbg(dev, "found adi,jrx0-logical-lane-mapping\n");
		for (i = 0; i < 12; i++)
			phy->profile.jrx[0].rx_link_cfg[0].lane_xbar[i] = lane_xbar[i];
	}

	ret = of_property_read_variable_u32_array(
		      node, "adi,jrx1-physical-lane-mapping",
		      lane_xbar, 12, 12);

	if (ret == 12) {
		dev_dbg(dev, "found adi,jrx1-logical-lane-mapping\n");
		for (i = 0; i < 12; i++)
			phy->profile.jrx[1].rx_link_cfg[0].lane_xbar[i] = lane_xbar[i];
	}

	of_property_read_u32(node, "adi,jtx-ser-amplitude", &ser_amplitude);
	of_property_read_u32(node, "adi,jtx-ser-pre-emphasis", &ser_pre_emphasis);
	of_property_read_u32(node, "adi,jtx-ser-post-emphasis", &ser_post_emphasis);

	phy->profile.jrx[0].common_link_cfg.lane_enables = 0;
	phy->profile.jrx[1].common_link_cfg.lane_enables = 0;
	phy->profile.jtx[0].common_link_cfg.lane_enables = 0;
	phy->profile.jtx[1].common_link_cfg.lane_enables = 0;

	for (i = 0; i < 12; i++) {
		phy->profile.jtx[0].serializer_lane[i].ser_amplitude = ser_amplitude;
		phy->profile.jtx[0].serializer_lane[i].ser_pre_emphasis = ser_pre_emphasis;
		phy->profile.jtx[0].serializer_lane[i].ser_post_emphasis = ser_post_emphasis;

		phy->profile.jtx[1].serializer_lane[i].ser_amplitude = ser_amplitude;
		phy->profile.jtx[1].serializer_lane[i].ser_pre_emphasis = ser_pre_emphasis;
		phy->profile.jtx[1].serializer_lane[i].ser_post_emphasis = ser_post_emphasis;

		/* JRX */
		if (phy->profile.jrx[0].rx_link_cfg[0].link_in_use)
			if (i <= (phy->profile.jrx[0].rx_link_cfg[0].l_minus1))
				phy->profile.jrx[0].common_link_cfg.lane_enables |= (1 << phy->profile.jrx[0].rx_link_cfg[0].lane_xbar[i]);

		if (phy->profile.jrx[1].rx_link_cfg[0].link_in_use)
			if (i <= (phy->profile.jrx[1].rx_link_cfg[0].l_minus1))
				phy->profile.jrx[1].common_link_cfg.lane_enables |= (1 << phy->profile.jrx[1].rx_link_cfg[0].lane_xbar[i]);

		/* JRX Link2 */

		if (phy->profile.jrx[0].rx_link_cfg[1].link_in_use)
			if (i <= (phy->profile.jrx[0].rx_link_cfg[1].l_minus1))
				phy->profile.jrx[0].common_link_cfg.lane_enables |= (1 << phy->profile.jrx[0].rx_link_cfg[1].lane_xbar[i]);

		if (phy->profile.jrx[1].rx_link_cfg[1].link_in_use)
			if (i <= (phy->profile.jrx[1].rx_link_cfg[1].l_minus1))
				phy->profile.jrx[1].common_link_cfg.lane_enables |= (1 << phy->profile.jrx[1].rx_link_cfg[1].lane_xbar[i]);

		/* JTX */
		if (phy->profile.jtx[0].tx_link_cfg[0].link_in_use)
			if ((u32)phy->profile.jtx[0].tx_link_cfg[0].lane_xbar[i] <= (u32)phy->profile.jtx[0].tx_link_cfg[0].l_minus1)
				phy->profile.jtx[0].common_link_cfg.lane_enables |= (1 << i);

		if (phy->profile.jtx[1].tx_link_cfg[0].link_in_use)
			if ((u32)phy->profile.jtx[1].tx_link_cfg[0].lane_xbar[i] <= (u32)phy->profile.jtx[1].tx_link_cfg[0].l_minus1)
				phy->profile.jtx[1].common_link_cfg.lane_enables |= (1 << i);

		/* JTX Link2 */

		if (phy->profile.jtx[0].tx_link_cfg[1].link_in_use)
			if ((u32)phy->profile.jtx[0].tx_link_cfg[1].lane_xbar[i] <= (u32)phy->profile.jtx[0].tx_link_cfg[1].l_minus1)
				phy->profile.jtx[0].common_link_cfg.lane_enables |= (1 << i);

		if (phy->profile.jtx[1].tx_link_cfg[1].link_in_use)
			if ((u32)phy->profile.jtx[1].tx_link_cfg[1].lane_xbar[i] <= (u32)phy->profile.jtx[1].tx_link_cfg[1].l_minus1)
				phy->profile.jtx[1].common_link_cfg.lane_enables |= (1 << i);
	}

	return 0;
}

int ad9088_parse_dt(struct ad9088_phy *phy)
{
	struct device *dev = &phy->spi->dev;
	struct device_node *node = dev->of_node;
	adi_apollo_top_t *p = &phy->profile;
	int ret;
	u32 val;
	const char *name;
	bool found;

	/* Check if calibration firmware is available - defer probe if not yet accessible */
	ret = of_property_read_string(node, "adi,device-calibration-data-name", &name);
	if (!ret) {
		const struct firmware *fw;

		ret = firmware_request_nowarn(&fw, name, dev);
		if (ret == -ENOENT) {
			dev_dbg(dev, "Calibration firmware '%s' not available yet, deferring probe\n", name);
			return -EPROBE_DEFER;
		} else if (ret) {
			dev_err(dev, "Failed to load calibration firmware '%s': %d\n", name, ret);
			return ret;
		}
		/* Firmware is available, release it - will be loaded later in ad9088_cal_load_from_firmware() */
		release_firmware(fw);
	}

	ret = of_property_read_string(node, "adi,device-profile-fw-name", &name);
	if (!ret) {
		ret = firmware_request_nowarn(&phy->fw, name, dev);
		if (ret == -ENOENT) {
			dev_dbg(dev, "Profile firmware '%s' not available yet, deferring probe\n", name);
			return -EPROBE_DEFER;
		} else if (ret) {
			dev_err(dev, "request_firmware() failed with %d\n", ret);
			return ret;
		}

		if (sizeof(*p) == phy->fw->size) {
			memcpy(p, phy->fw->data, sizeof(*p));
			release_firmware(phy->fw);
			phy->device_profile_firmware_load = true;
		} else {
			dev_err(dev, "request_firmware() incompatible size %zu != %zu\n", sizeof(*p), phy->fw->size);
			release_firmware(phy->fw);
			return -EINVAL;
		}
	}

	phy->spi_3wire_en = of_property_read_bool(node, "adi,spi-3wire-enable");

	/* Parse device label for sub-device naming (bmem, fft-sniffer) */
	ret = of_property_read_string(node, "label", &phy->device_label);
	if (ret)
		phy->device_label = NULL;

	phy->complex_rx = !of_property_read_bool(node, "adi,rx-real-channel-en");
	phy->complex_tx = !of_property_read_bool(node, "adi,tx-real-channel-en");
	phy->aion_background_serial_alignment_en =
		of_property_read_bool(node, "adi,aion-background-serial-alignment-en");

	phy->side_b_use_own_tpl_en = of_property_read_bool(node,
							   "adi,side-b-use-seperate-tpl-en");

	phy->hsci_use_auto_linkup_mode = of_property_read_bool(node, "adi,hsci-auto-linkup-mode-en");

	phy->hsci_disable_after_initial_configuration =
		of_property_read_bool(node, "adi,hsci-disable-after-boot-en");

	phy->multidevice_instance_count = 1;
	of_property_read_u32(node, "adi,multidevice-instance-count",
			     &phy->multidevice_instance_count);

	/*
	 * MCS tracking calibration TDC decimation rate. A larger value improves
	 * precision at the expense of longer TDC measurement time. For gapped
	 * periodic SYSREF, keep below 32768. Default: 1023.
	 */
	phy->mcs_track_decimation = 1023;
	of_property_read_u16(node, "adi,mcs-track-decimation",
			     &phy->mcs_track_decimation);

	phy->trig_sync_en = of_property_read_bool(node, "adi,trigger-sync-en");

	phy->standalone = of_property_read_bool(node, "adi,standalone-enable");

	phy->rx_nyquist_zone = 1;
	of_property_read_u32(node, "adi,nyquist-zone", &phy->rx_nyquist_zone);

	if (phy->rx_nyquist_zone != 1 && phy->rx_nyquist_zone != 2) {
		dev_err(dev, "Invalid Nyquist zone %u\n", phy->rx_nyquist_zone);
		return -EINVAL;
	}

	phy->fnco_dual_modulus_mode_en = of_property_read_bool(node, "adi,fnco-dual-modulus-mode-en");
	phy->cnco_dual_modulus_mode_en = of_property_read_bool(node, "adi,cnco-dual-modulus-mode-en");

	phy->sniffer_en = !of_property_read_bool(node, "adi,sniffer-disable");

	phy->cddc_sample_delay_en = of_property_read_bool(node, "adi,cddc-bmem-sample-delay-en");
	phy->fddc_sample_delay_en = of_property_read_bool(node, "adi,fddc-bmem-sample-delay-en");

	ad9088_jesd_lane_setup(phy);

	/*
	 * IIO channel scan_index remapping for lane swap compensation.
	 * When FPGA lane routing causes DMA buffer positions to not match
	 * the physical channel order, use this array to remap scan_index.
	 * Value at index i specifies which DMA buffer position IIO channel i
	 * should read from. A value of -1 means no remapping (identity).
	 *
	 * Array covers: channelizers * 2 (I/Q) * multidevice_instance_count (max 4)
	 *
	 * Example: If sides are swapped (Side B data in DMA pos 0-3,
	 * Side A data in DMA pos 4-7):
	 *   adi,rx-iio-to-phy-remap = /bits/ 8 <4 5 6 7 0 1 2 3>;
	 */
	memset(phy->rx_iio_to_phy_remap, -1, sizeof(phy->rx_iio_to_phy_remap));

	ret = of_property_read_variable_u8_array(node, "adi,rx-iio-to-phy-remap",
						 (u8 *)phy->rx_iio_to_phy_remap,
						 1, MAX_NUM_REMAP_CHANNELS);
	if (ret > 0)
		dev_info(dev, "RX IIO-to-PHY channel remap: %d entries\n", ret);

	found = of_property_read_bool(node, "adi,dformat-ddc-dither-en");
	if (found) {
		phy->profile.rx_path[0].rx_dformat[0].ddc_dither_en = found;
		phy->profile.rx_path[0].rx_dformat[1].ddc_dither_en = found;
		phy->profile.rx_path[1].rx_dformat[0].ddc_dither_en = found;
		phy->profile.rx_path[1].rx_dformat[1].ddc_dither_en = found;
	}

	ret = of_property_read_u32(node, "adi,subclass", &val);
	if (!ret) {
		phy->profile.jtx[0].common_link_cfg.subclass = val;
		phy->profile.jtx[1].common_link_cfg.subclass = val;
		phy->profile.jrx[0].common_link_cfg.subclass = val;
		phy->profile.jrx[1].common_link_cfg.subclass = val;
	}

	if (phy->profile.profile_cfg.profile_version.major != ADI_APOLLO_PROFILE_VERSION_MAJOR ||
	    phy->profile.profile_cfg.profile_version.minor != ADI_APOLLO_PROFILE_VERSION_MINOR) {
		dev_err(dev, "Incompatible profile version %u.%u != %u.%u\n",
			phy->profile.profile_cfg.profile_version.major,
			phy->profile.profile_cfg.profile_version.minor,
			ADI_APOLLO_PROFILE_VERSION_MAJOR,
			ADI_APOLLO_PROFILE_VERSION_MINOR);

		return -EINVAL;
	}

	/* FIXME ! */
	if (phy->profile.profile_cfg.profile_version.patch< 3) {
		dev_warn(dev, "Old profile version patch %u, updating to %u\n",
			 phy->profile.profile_cfg.profile_version.patch, 3);

		phy->profile.profile_cfg.profile_version.patch = 3;
		phy->profile.reserved_cfg[4] = phy->profile.reserved_cfg[0];
		phy->profile.reserved_cfg[5] = phy->profile.reserved_cfg[1];
		phy->profile.reserved_cfg[0] = 0;
		phy->profile.reserved_cfg[1] = 0;
		phy->profile.mcs_cfg.center_sysref.sysref_present = true;

	}

	dev_dbg(dev, "Profile CRC32 %u\n", phy->profile.profile_checksum);
	phy->profile.profile_checksum = crc32_be(0, (unsigned char const *)p, sizeof(*p) - sizeof(u32));
	dev_dbg(dev, "Profile CRC32 %u\n", phy->profile.profile_checksum);

	return 0;
}
