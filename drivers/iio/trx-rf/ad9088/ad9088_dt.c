// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for AD9088 and similar mixed signal front end (MxFE®)
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include "ad9088.h"

static void ad9088_jrx_lane_set(adi_apollo_jesd_rx_cfg_t *jrx, u8 link, u8 idx)
{
	adi_apollo_jesd_rx_link_cfg_t *link_cfg = &jrx->rx_link_cfg[link];

	if (!link_cfg->link_in_use || idx > link_cfg->l_minus1)
		return;
	jrx->common_link_cfg.lane_enables |= (1 << link_cfg->lane_xbar[idx]);
}

static void ad9088_jtx_lane_set(adi_apollo_jesd_tx_cfg_t *jtx, u8 link, u8 idx)
{
	adi_apollo_jesd_tx_link_cfg_t *link_cfg = &jtx->tx_link_cfg[link];

	if (!link_cfg->link_in_use || (u32)link_cfg->lane_xbar[idx] > (u32)link_cfg->l_minus1)
		return;
	jtx->common_link_cfg.lane_enables |= (1 << idx);
}

static void ad9088_jesd_lane_setup(struct ad9088_phy *phy)
{
	struct device *dev = &phy->spi->dev;
	struct device_node *node = dev->of_node;
	int ret, i;
	u32 lane_xbar[12];
	u32 ser_amplitude = ADI_APOLLO_JESD_DRIVE_SWING_VTT_100;
	u32 ser_pre_emphasis = ADI_APOLLO_JESD_PRE_TAP_LEVEL_6_DB;
	u32 ser_post_emphasis = ADI_APOLLO_JESD_POST_TAP_LEVEL_3_DB;

	ret = of_property_read_u32_array(node, "adi,jtx0-logical-lane-mapping",
					 lane_xbar, ARRAY_SIZE(lane_xbar));
	if (!ret) {
		dev_dbg(dev, "found adi,jtx0-logical-lane-mapping\n");
		for (i = 0; i < ARRAY_SIZE(lane_xbar); i++)
			phy->profile.jtx[0].tx_link_cfg[0].lane_xbar[i] = lane_xbar[i];
	}

	ret = of_property_read_u32_array(node, "adi,jtx1-logical-lane-mapping",
					 lane_xbar, ARRAY_SIZE(lane_xbar));
	if (!ret) {
		dev_dbg(dev, "found adi,jtx1-logical-lane-mapping\n");
		for (i = 0; i < ARRAY_SIZE(lane_xbar); i++)
			phy->profile.jtx[1].tx_link_cfg[0].lane_xbar[i] = lane_xbar[i];
	}

	ret = of_property_read_u32_array(node, "adi,jrx0-physical-lane-mapping",
					 lane_xbar, ARRAY_SIZE(lane_xbar));
	if (!ret) {
		dev_dbg(dev, "found adi,jrx0-logical-lane-mapping\n");
		for (i = 0; i < ARRAY_SIZE(lane_xbar); i++)
			phy->profile.jrx[0].rx_link_cfg[0].lane_xbar[i] = lane_xbar[i];
	}

	ret = of_property_read_u32_array(node, "adi,jrx1-physical-lane-mapping",
					 lane_xbar, ARRAY_SIZE(lane_xbar));
	if (!ret) {
		dev_dbg(dev, "found adi,jrx1-logical-lane-mapping\n");
		for (i = 0; i < ARRAY_SIZE(lane_xbar); i++)
			phy->profile.jrx[1].rx_link_cfg[0].lane_xbar[i] = lane_xbar[i];
	}

	of_property_read_u32(node, "adi,jtx-ser-amplitude", &ser_amplitude);
	of_property_read_u32(node, "adi,jtx-ser-pre-emphasis", &ser_pre_emphasis);
	of_property_read_u32(node, "adi,jtx-ser-post-emphasis", &ser_post_emphasis);

	phy->profile.jrx[0].common_link_cfg.lane_enables = 0;
	phy->profile.jrx[1].common_link_cfg.lane_enables = 0;
	phy->profile.jtx[0].common_link_cfg.lane_enables = 0;
	phy->profile.jtx[1].common_link_cfg.lane_enables = 0;

	for (i = 0; i < ARRAY_SIZE(lane_xbar); i++) {
		phy->profile.jtx[0].serializer_lane[i].ser_amplitude = ser_amplitude;
		phy->profile.jtx[0].serializer_lane[i].ser_pre_emphasis = ser_pre_emphasis;
		phy->profile.jtx[0].serializer_lane[i].ser_post_emphasis = ser_post_emphasis;

		phy->profile.jtx[1].serializer_lane[i].ser_amplitude = ser_amplitude;
		phy->profile.jtx[1].serializer_lane[i].ser_pre_emphasis = ser_pre_emphasis;
		phy->profile.jtx[1].serializer_lane[i].ser_post_emphasis = ser_post_emphasis;

		/* JRX */
		ad9088_jrx_lane_set(&phy->profile.jrx[0], 0, i);
		ad9088_jrx_lane_set(&phy->profile.jrx[1], 0, i);
		/* JRX Link2 */
		ad9088_jrx_lane_set(&phy->profile.jrx[0], 1, i);
		ad9088_jrx_lane_set(&phy->profile.jrx[1], 1, i);
		/* JTX */
		ad9088_jtx_lane_set(&phy->profile.jtx[0], 0, i);
		ad9088_jtx_lane_set(&phy->profile.jtx[1], 0, i);
		/* JTX Link2 */
		ad9088_jtx_lane_set(&phy->profile.jtx[0], 1, i);
		ad9088_jtx_lane_set(&phy->profile.jtx[1], 1, i);
	}
}

static int ad9088_fsrc_setup(struct ad9088_phy *phy)
{
	if (!phy->iio_axi_fsrc)
		return 0;

	dev_info(&phy->spi->dev, "FSRC support enabled\n");

	/*
	 * Default 1x value from python example at
	 * public/inc/adi_apollo_fsrc.h@adi_apollo_fsrc_rate_set
	 **/
	for (u8 i = 0; i < ADI_APOLLO_NUM_SIDES; i++) {
		for (u8 j = 0; j < ADI_APOLLO_FSRCS_PER_SIDE; j++) {
			phy->profile.rx_path[i].rx_fsrc[j].fsrc_rate_int = BIT(48);
			phy->profile.tx_path[i].tx_fsrc[j].fsrc_rate_int = BIT(48);
			phy->profile.rx_path[i].rx_fsrc[j].fsrc_rate_frac_a = 0;
			phy->profile.tx_path[i].tx_fsrc[j].fsrc_rate_frac_a = 0;
			phy->profile.rx_path[i].rx_fsrc[j].fsrc_rate_frac_b = 1;
			phy->profile.tx_path[i].tx_fsrc[j].fsrc_rate_frac_b = 1;
			phy->profile.rx_path[i].rx_fsrc[j].gain_reduction = BIT(12) - 1;
			phy->profile.tx_path[i].tx_fsrc[j].gain_reduction = BIT(12) - 1;
			phy->profile.rx_path[i].rx_fsrc[j].mode_1x = true;
			phy->profile.tx_path[i].tx_fsrc[j].mode_1x = true;
			phy->profile.rx_path[i].rx_fsrc[j].enable = true;
			phy->profile.tx_path[i].tx_fsrc[j].enable = true;
		}
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

	phy->spi_3wire_en = of_property_read_bool(node, "adi,spi-3wire-enable");

	ret = of_property_read_string(node, "adi,device-profile-fw-name", &name);
	if (!ret) {
		ret = firmware_request_nowarn(&phy->fw, name, dev);
		if (ret == -ENOENT)
			return dev_err_probe(dev, -EPROBE_DEFER,
					     "Profile firmware '%s' not available yet, deferring probe\n",
					     name);
		if (ret)
			return dev_err_probe(dev, ret, "request_firmware() failed\n");

		if (sizeof(*p) == phy->fw->size) {
			memcpy(p, phy->fw->data, sizeof(*p));
			release_firmware(phy->fw);
			phy->device_profile_firmware_load = true;
		} else {
			release_firmware(phy->fw);
			return dev_err_probe(dev, -EINVAL,
					     "request_firmware() incompatible size %zu != %zu\n",
					     sizeof(*p), phy->fw->size);
		}
	}

	/* Check if calibration firmware is available - defer probe if not yet accessible */
	ret = of_property_read_string(node, "adi,device-calibration-data-name", &name);
	if (!ret) {
		const struct firmware *fw;

		ret = firmware_request_nowarn(&fw, name, dev);
		if (ret == -ENOENT)
			return dev_err_probe(dev, -EPROBE_DEFER,
					     "Calibration firmware '%s' not available yet, deferring probe\n",
					     name);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to load calibration firmware '%s': %d\n",
					     name, ret);
		/*
		 * Firmware is available, release it - will be loaded later in
		 * ad9088_cal_load_from_firmware()
		 */
		release_firmware(fw);
	}

	phy->complex_rx = !of_property_read_bool(node, "adi,rx-real-channel-en");
	phy->complex_tx = !of_property_read_bool(node, "adi,tx-real-channel-en");

	if (of_property_read_bool(node, "adi,aion-background-serial-alignment-en"))
		phy->aion_background_serial_alignment_en = true;

	phy->side_b_use_own_tpl_en = of_property_read_bool(node,
							   "adi,side-b-use-separate-tpl-en");

	phy->hsci_use_auto_linkup_mode = of_property_read_bool(node,
							       "adi,hsci-auto-linkup-mode-en");

	if (of_property_read_bool(node, "adi,hsci-disable-after-boot-en"))
		phy->hsci_disable_after_initial_configuration = true;

	phy->multidevice_instance_count = 1;
	of_property_read_u32(node, "adi,multidevice-instance-count",
			     &phy->multidevice_instance_count);

	/*
	 * MCS tracking calibration TDC decimation rate. A larger value improves
	 * precision at the expense of longer TDC measurement time. For gapped
	 * periodic SYSREF, keep below 32768. Default: 1023.
	 */
	phy->mcs_track_decimation = 1023;
	of_property_read_u32(node, "adi,mcs-track-decimation", &phy->mcs_track_decimation);

	phy->trig_sync_en = of_property_read_bool(node, "adi,trigger-sync-en");

	phy->standalone = of_property_read_bool(node, "adi,standalone-enable");

	phy->rx_nyquist_zone = 1;
	of_property_read_u32(node, "adi,nyquist-zone", &phy->rx_nyquist_zone);

	if (phy->rx_nyquist_zone != 1 && phy->rx_nyquist_zone != 2)
		return dev_err_probe(dev, -EINVAL, "Invalid Nyquist zone %u\n",
				     phy->rx_nyquist_zone);

	phy->fnco_dual_modulus_mode_en = of_property_read_bool(node,
							       "adi,fnco-dual-modulus-mode-en");
	phy->cnco_dual_modulus_mode_en = of_property_read_bool(node,
							       "adi,cnco-dual-modulus-mode-en");

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

	ad9088_fsrc_setup(phy);

	ret = of_property_read_u32(node, "adi,subclass", &val);
	if (!ret) {
		phy->profile.jtx[0].common_link_cfg.subclass = val;
		phy->profile.jtx[1].common_link_cfg.subclass = val;
		phy->profile.jrx[0].common_link_cfg.subclass = val;
		phy->profile.jrx[1].common_link_cfg.subclass = val;

		if (val) {
			if (!phy->profile.mcs_cfg.side_a_sysref.sysref_present ||
			    !phy->profile.mcs_cfg.side_b_sysref.sysref_present)
				phy->profile.mcs_cfg.center_sysref.sysref_present = true;
		}
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

	/* ADF4382 clock align GPIO configuration - set defaults */
	p->mcs_cfg.adf4382_cfg.clock_align_delay_adjust_gpio[0] = 16;
	p->mcs_cfg.adf4382_cfg.clock_align_delay_adjust_gpio[1] = 0;
	p->mcs_cfg.adf4382_cfg.clock_align_delay_strobe_gpio[0] = 15;
	p->mcs_cfg.adf4382_cfg.clock_align_delay_strobe_gpio[1] = 0;

	/* Allow device tree to override */
	of_property_read_variable_u8_array(node, "adi,clock-align-delay-adjust-gpio-num",
					   p->mcs_cfg.adf4382_cfg.clock_align_delay_adjust_gpio,
					   1, ADI_APOLLO_NUM_ADF4382_GPIOS);

	of_property_read_variable_u8_array(node, "adi,clock-align-delay-strobe-gpio-num",
					   p->mcs_cfg.adf4382_cfg.clock_align_delay_strobe_gpio,
					   1, ADI_APOLLO_NUM_ADF4382_GPIOS);

	/*
	 * MCS tracking window: the amount of deviation (in femtoseconds) the
	 * ADF4382 output clock can drift relative to AD9084's External SysRef
	 * before MCS Tracking attempts to correct it. Overrides the profile
	 * value for both ADF4382 instances when specified.
	 */
	if (!of_property_read_u32(node, "adi,mcs-track-win", &phy->mcs_track_win)) {
		p->mcs_cfg.adf4382_cfg.track_win[0] = phy->mcs_track_win;
		p->mcs_cfg.adf4382_cfg.track_win[1] = phy->mcs_track_win;
	} else {
		phy->mcs_track_win = p->mcs_cfg.adf4382_cfg.track_win[0];
	}

	dev_dbg(dev, "Profile CRC32 %u\n", phy->profile.profile_checksum);
	phy->profile.profile_checksum = crc32_be(0, (unsigned char const *)p,
						 sizeof(*p) - sizeof(u32));
	dev_dbg(dev, "Profile CRC32 %u\n", phy->profile.profile_checksum);

	return 0;
}
