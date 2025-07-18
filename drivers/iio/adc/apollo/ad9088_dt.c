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

	phy->spi_3wire_en = of_property_read_bool(node, "adi,spi-3wire-enable");

	ret = of_property_read_string(node, "adi,device-profile-fw-name", &name);
	if (!ret) {
		ret = request_firmware(&phy->fw, name, dev);
		if (ret) {
			dev_err(dev, "request_firmware() failed with %d\n", ret);
			return ret;
		}

		if (sizeof(*p) == phy->fw->size) {
			memcpy(p, phy->fw->data, sizeof(*p));
			release_firmware(phy->fw);
			phy->device_profile_firmware_load = true;
		} else {
			dev_err(dev, "request_firmware() incompatible size %u != %zu\n", sizeof(*p), phy->fw->size);
			release_firmware(phy->fw);
			return -EINVAL;
		}
	}

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

	phy->trig_sync_en = of_property_read_bool(node, "adi,trigger-sync-en");

	phy->standalone = of_property_read_bool(node, "adi,standalone-enable");

	phy->rx_nyquist_zone = 1;
	of_property_read_u32(node, "adi,nyquist-zone", &phy->rx_nyquist_zone);

	if (phy->rx_nyquist_zone != 1 && phy->rx_nyquist_zone != 2) {
		dev_err(dev, "Invalid Nyquist zone %u\n", phy->rx_nyquist_zone);
		return -EINVAL;
	}

	ad9088_jesd_lane_setup(phy);

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

	dev_dbg(dev, "Profile CRC32 %u\n", phy->profile.profile_checksum);
	phy->profile.profile_checksum = crc32_be(0, (unsigned char const *)p, sizeof(*p) - sizeof(u32));
	dev_dbg(dev, "Profile CRC32 %u\n", phy->profile.profile_checksum);

	return 0;
}
