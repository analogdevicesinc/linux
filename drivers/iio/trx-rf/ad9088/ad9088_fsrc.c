// SPDX-License-Identifier: GPL-2.0
/*
 * FSRC (Fractional Sample Rate Converter) support for AD9088
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/string.h>

#include "ad9088.h"
#include "adi_apollo_bf_jrx_wrapper.h"

static int ad9088_axi_fsrc_enable(struct ad9088_phy *phy, bool enable,
				  adi_apollo_terminal_e terminal)
{
	int ret;

	if (!phy->iio_axi_fsrc) {
		dev_dbg(&phy->spi->dev, "FSRC channel not available\n");
		return -ENODEV;
	}

	ret = ad9088_iio_write_channel_ext_info(phy, phy->iio_axi_fsrc,
						terminal == ADI_APOLLO_TX ? "tx_enable" : "rx_enable", enable);
	if (ret < 0) {
		dev_err(&phy->spi->dev, "Failed to %s %s FSRC: %d\n",
			enable ? "enable" : "disable", terminal == ADI_APOLLO_TX ? "tx_enable" : "rx_enable", ret);
		return ret;
	}

	dev_dbg(&phy->spi->dev, "TX FSRC %s\n", enable ? "enabled" : "disabled");
	return 0;
}

/**
 * ad9088_axi_fsrc_tx_active - Start/stop TX transmission
 * @phy: AD9088 device instance
 * @active: true to start, false to stop (send invalids)
 *
 * Returns: 0 on success, negative error code on failure
 */
static int ad9088_axi_fsrc_tx_active(struct ad9088_phy *phy, bool active)
{
	int ret;

	if (!phy->iio_axi_fsrc) {
		dev_dbg(&phy->spi->dev, "FSRC channel not available\n");
		return -ENODEV;
	}

	ret = ad9088_iio_write_channel_ext_info(phy, phy->iio_axi_fsrc,
						"tx_active", active);
	if (ret < 0) {
		dev_err(&phy->spi->dev, "Failed to set TX FSRC active=%d: %d\n",
			active, ret);
		return ret;
	}

	dev_dbg(&phy->spi->dev, "TX FSRC transmission %s\n",
		active ? "started" : "stopped (-FS stream)");
	return 0;
}

static int ad9088_fsrc_ratio_apply(struct ad9088_phy *phy,
				   adi_apollo_terminal_e terminal, u32 n, u32 m)
{
	int ret;

	if (n == m) {
		ret = adi_apollo_fsrc_rate_set(&phy->ad9088, terminal, ADI_APOLLO_FSRC_ALL,
					       0, 0, 1, AD9088_FSRC_1X_GAIN);
		return ad9088_check_apollo_error(&phy->spi->dev, ret,
						 "adi_apollo_fsrc_rate_set");
	}

	ret = adi_apollo_fsrc_ratio_set(&phy->ad9088, terminal, ADI_APOLLO_FSRC_ALL, n, m);
	return ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_fsrc_ratio_set");
}

/**
 * ad9088_fsrc_rx_configure - Configure RX FSRC and data path
 * @phy: AD9088 device instance
 * @fsrc_n: FSRC N value
 * @fsrc_m: FSRC M value
 *
 * Based on apollo_rx_fsrc_configure() from fullchip_fsrc_dr.c
 *
 * Returns: 0 on success, negative error code on failure
 */
int ad9088_fsrc_rx_configure(struct ad9088_phy *phy, u32 fsrc_n, u32 fsrc_m)
{
	int ret;

	dev_dbg(&phy->spi->dev, "Configuring RX FSRC: N=%u M=%u\n", fsrc_n, fsrc_m);

	ret = adi_apollo_fsrc_mode_1x_enable_set(&phy->ad9088, ADI_APOLLO_RX, ADI_APOLLO_FSRC_ALL,
						 fsrc_m == fsrc_n);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_fsrc_mode_1x_enable_set");
	if (ret)
		return ret;

	ret = ad9088_fsrc_ratio_apply(phy, ADI_APOLLO_RX, fsrc_n, fsrc_m);
	if (ret)
		return ret;

	dev_dbg(&phy->spi->dev, "RX FSRC configuration complete\n");
	return 0;
}

/*
 * The API enables invalid sample handling in JRx rate match FIFO slice i only
 * when JRx link i is in use, but FSRC i feeds slice i whatever the links: with
 * one link per side and both FSRCs enabled, slice 1 takes the -FS holes as data.
 */
static int ad9088_fsrc_tx_invalid_enable(struct ad9088_phy *phy)
{
	static const u32 wrappers[ADI_APOLLO_NUM_SIDES] = {
		JRX_WRAPPER_JRX_TX_DIGITAL0, JRX_WRAPPER_JRX_TX_DIGITAL1
	};
	int ret;

	for (int side = 0; side < ADI_APOLLO_NUM_SIDES; side++) {
		const adi_apollo_jesd_rx_link_cfg_t *link = &phy->profile.jrx[side].rx_link_cfg[0];

		if (!link->link_in_use)
			continue;

		for (int slice = 0; slice < ADI_APOLLO_FSRC_PER_SIDE_NUM; slice++) {
			ret = adi_apollo_hal_bf_set(&phy->ad9088,
						    BF_INVALID_DATA_EN_INFO(wrappers[side], slice), 1);
			if (!ret)
				ret = adi_apollo_hal_bf_set(&phy->ad9088,
							    BF_NUM_OF_INVALID_SAMPLE_INFO(wrappers[side], slice),
							    link->ns_minus1);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
							"JRx invalid sample enable");
			if (ret)
				return ret;
		}
	}

	return 0;
}

/**
 * ad9088_fsrc_tx_configure - Configure TX FSRC and data path
 * @phy: AD9088 device instance
 * @fsrc_n: FSRC N value
 * @fsrc_m: FSRC M value
 *
 * Based on apollo_tx_fsrc_configure() from fullchip_fsrc_dr.c
 *
 * Returns: 0 on success, negative error code on failure
 */
int ad9088_fsrc_tx_configure(struct ad9088_phy *phy, u32 fsrc_n, u32 fsrc_m)
{
	char ratio_str[64];
	int ret;

	dev_dbg(&phy->spi->dev, "Configuring TX FSRC: N=%u M=%u\n", fsrc_n, fsrc_m);

	if (!phy->iio_axi_fsrc) {
		dev_dbg(&phy->spi->dev, "FSRC channel not available\n");
		return -ENODEV;
	}

	if (fsrc_m != 0 && fsrc_n != 0) {
		/* The FPGA hole pattern follows the JRx samples per conv_clk */
		snprintf(ratio_str, sizeof(ratio_str), "%u %u %u", fsrc_n, fsrc_m,
			 phy->profile.jrx[0].rx_link_cfg[0].ns_minus1 + 1);
		ret = iio_write_channel_ext_info(phy->iio_axi_fsrc, "tx_ratio_set",
						  ratio_str, strlen(ratio_str) + 1);
		if (ret < 0) {
			dev_err(&phy->spi->dev, "Failed to set TX FSRC ratio %u/%u: %d\n",
				fsrc_n, fsrc_m, ret);
			return ret;
		}
	}
	ret = ad9088_fsrc_tx_invalid_enable(phy);
	if (ret)
		return ret;

	/* Similar to public/inc/adi_apollo_fsrc.h@adi_apollo_fsrc_rate_set python example */
	ret = adi_apollo_fsrc_mode_1x_enable_set(&phy->ad9088, ADI_APOLLO_TX, ADI_APOLLO_FSRC_ALL,
						 fsrc_m == fsrc_n);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_fsrc_mode_1x_enable_set");
	if (ret)
		return ret;

	ret = ad9088_fsrc_ratio_apply(phy, ADI_APOLLO_TX, fsrc_n, fsrc_m);
	if (ret)
		return ret;

	dev_dbg(&phy->spi->dev, "TX FSRC configuration complete\n");
	return 0;
}

/**
 * ad9088_fsrc_trigger_reconfig_sequence - Execute FSRC dynamic reconfig
 * @phy: AD9088 device instance
 *
 * Sequence:
 * 1. Trigger FPGA sequencer (GPIO only)
 * 2. FPGA sequencer counts SYSREFs and sends trigger to Apollo (GPIO only)
 * 3. Apollo executes reconfig trigger
 * 4. Clear Apollo trigger sync (GPIO only)
 *
 * Returns: 0 on success, negative error code on failure
 */
static int ad9088_fsrc_trigger_reconfig_sequence(struct ad9088_phy *phy)
{
	int ret;

	if (phy->fsrc_gpio_trig_en) {
		/* Trigger FPGA sequencer to start the SYSREF-based sequence */
		ret = ad9088_iio_write_channel_ext_info(phy, phy->iio_axi_fsrc,
							"seq_start", true);
		if (ret < 0) {
			dev_err(&phy->spi->dev,
				"Failed to trigger sequencer via reg: %d\n", ret);
			return ret;
		}

		/*
		 * Wait for sequencer to complete, at most 15 SYSREF periods
		 */
		usleep_range(500, 1000);

		/* Clear Apollo trigger sync */
		ret = adi_apollo_clk_mcs_trig_sync_enable(&phy->ad9088, 0);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
						"adi_apollo_clk_mcs_trig_sync_enable clear");
		if (ret)
			return ret;
	} else {
		/* Execute manual dynamic reconfig - applies new settings */
		ret = adi_apollo_clk_mcs_man_reconfig_sync(&phy->ad9088);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
						"adi_apollo_clk_mcs_man_reconfig_sync");
		if (ret)
			return ret;
	}

	ret = adi_apollo_clk_mcs_trig_reset_disable(&phy->ad9088);
	return ad9088_check_apollo_error(&phy->spi->dev, ret,
					 "adi_apollo_clk_mcs_trig_reset_disable");
}

/**
 * ad9088_fsrc_rx_reconfig_sequence - Execute RX FSRC dynamic reconfig
 * @phy: AD9088 device instance
 * @enable: Enable AXI FSRC side
 *
 * Sequence:
 * 1. Enable trigger sync on Apollo
 * 2. Do trigger procedure regmap or GPIO trigger.
 * 3. Wait for samples to flow through RX path
 * 4. Reset JRX rate-match FIFO
 *
 * RX Path Invalid Sample Flow:
 *   Apollo ADC -> Apollo RX FSRC (adds invalid samples -FS)
 *   FPGA RX FSRC (removes invalid samples)
 *
 * Returns: 0 on success, negative error code on failure
 */
int ad9088_fsrc_rx_reconfig_sequence(struct ad9088_phy *phy, bool enable)
{
	int ret;

	dev_dbg(&phy->spi->dev, "Starting RX FSRC SPI reconfig sequence\n");

	/* Enable RX FPGA, just removes -FS */
	ret = ad9088_axi_fsrc_enable(phy, enable, ADI_APOLLO_RX);
	if (ret)
		return ret;

	/* Enable trigger sync - resync RX digital blocks during reconfig */
	ret = adi_apollo_clk_mcs_trig_reset_dsp_enable(&phy->ad9088);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_clk_mcs_trig_reset_dsp_enable");
	if (ret)
		return ret;

	/*
	 * RX needs no alignment with the FPGA, which deletes the invalid
	 * samples wherever they are, so reconfigure over SPI. Running the
	 * sequencer here would also restart the TX hole pattern.
	 */
	ret = adi_apollo_clk_mcs_man_reconfig_sync(&phy->ad9088);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_clk_mcs_man_reconfig_sync");
	if (ret)
		return ret;

	ret = adi_apollo_clk_mcs_trig_reset_disable(&phy->ad9088);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_clk_mcs_trig_reset_disable");
	if (ret)
		return ret;

	dev_dbg(&phy->spi->dev, "RX FSRC reconfig sequence complete\n");
	return 0;
}

/**
 * ad9088_fsrc_tx_reconfig_sequence - Execute TX FSRC GPIO-triggered reconfig
 * @phy: AD9088 device instance
 * @enable: Enable AXI FSRC side
 *
 * Executes TX-only GPIO-triggered FSRC dynamic reconfiguration sequence:
 * 1. Enable FPGA sequencer external trigger mode
 * 2. Stop FPGA TX (send invalids only)
 * 3. Apollo forces JTx invalids
 * 4. Enable Apollo trigger sync (wait for GPIO trigger)
 * 5. Trigger FPGA sequencer (via register or external GPIO)
 * 6. FPGA sequencer counts SYSREFs and sends trigger to Apollo
 * 7. Apollo executes TX reconfig on trigger
 * 8. FPGA resumes sending valid data
 * 9. Clear Apollo trigger sync
 * 10. Reset rate-match FIFO
 *
 * Based on reconfig_trig() from fullchip_fsrc_sc1_ext_trig.c
 *
 * Returns: 0 on success, negative error code on failure
 */
int ad9088_fsrc_tx_reconfig_sequence(struct ad9088_phy *phy, bool enable)
{
	int ret;

	dev_dbg(&phy->spi->dev, "Starting TX FSRC reconfig sequence\n");

	ret = ad9088_axi_fsrc_enable(phy, enable, ADI_APOLLO_TX);
	if (ret)
		return ret;

	if (enable && phy->fsrc_gpio_trig_en) {
		ret = ad9088_axi_fsrc_tx_active(phy, 0);
		if (ret)
			return ret;
	}

	/* Allow invalids to flow through JESD link */
	usleep_range(10000, 11000);

	if (phy->fsrc_gpio_trig_en) {
		/* Apollo forces JTx invalids - TX will send only invalid samples */
		ret = adi_apollo_jtx_force_invalids_set(&phy->ad9088, ADI_APOLLO_LINK_ALL, 1);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
						"adi_apollo_jtx_force_invalids_set");
		if (ret)
			return ret;

		/* Wait for invalids to propagate through the system */
		usleep_range(1000000, 1100000);  /* 1000ms */

		ret = adi_apollo_jrx_rm_fifo_reset(&phy->ad9088,
						   ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
						"adi_apollo_jrx_rm_fifo_reset");
		if (ret)
			return ret;

		/* The sequencer trigger reaches Apollo on trigger pin A0 */
		ret = adi_apollo_clk_mcs_sync_trig_map(&phy->ad9088, ADI_APOLLO_RX_TX_ALL,
						       ADI_APOLLO_TRIG_PIN_A0);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
						"adi_apollo_clk_mcs_sync_trig_map");
		if (ret)
			return ret;

		ret = adi_apollo_clk_mcs_trig_reset_dsp_enable(&phy->ad9088);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
						"adi_apollo_clk_mcs_trig_reset_dsp_enable");
		if (ret)
			return ret;

		ret = adi_apollo_clk_mcs_trig_sync_enable(&phy->ad9088, 1);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
						"adi_apollo_clk_mcs_trig_sync_enable");
		if (ret)
			return ret;
	} else {
		ret = adi_apollo_clk_mcs_trig_reset_dsp_enable(&phy->ad9088);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
						"adi_apollo_clk_mcs_trig_reset_dsp_enable");
		if (ret)
			return ret;
	}

	ret = ad9088_fsrc_trigger_reconfig_sequence(phy);
	if (ret)
		return ret;

	/* Resume FPGA TX - sends valid and invalid samples at new rate */
	if (enable) {
		ret = ad9088_axi_fsrc_tx_active(phy, 1);
		if (ret)
			return ret;
	}

	if (!phy->fsrc_gpio_trig_en) {
		usleep_range(100, 200);

		ret = adi_apollo_jrx_rm_fifo_reset(&phy->ad9088,
						   ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
						"adi_apollo_jrx_rm_fifo_reset");
		if (ret)
			return ret;
	}

	dev_dbg(&phy->spi->dev, "TX FSRC reconfig sequence complete\n");
	return 0;
}

