// SPDX-License-Identifier: GPL-2.0
/*
 * FSRC (Fractional Sample Rate Converter) support for AD9088
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/string.h>

#include "ad9088.h"

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

	if (fsrc_m != fsrc_n) {

		ret = adi_apollo_fsrc_ratio_set(&phy->ad9088, ADI_APOLLO_RX, ADI_APOLLO_FSRC_ALL,
						fsrc_n, fsrc_m);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
						"adi_apollo_fsrc_ratio_set");
	}

	if (ret)
		return ret;

	dev_dbg(&phy->spi->dev, "RX FSRC configuration complete\n");
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
		snprintf(ratio_str, sizeof(ratio_str), "%u %u", fsrc_n, fsrc_m);
		ret = iio_write_channel_ext_info(phy->iio_axi_fsrc, "tx_ratio_set",
						  ratio_str, strlen(ratio_str) + 1);
		if (ret < 0) {
			dev_err(&phy->spi->dev, "Failed to set TX FSRC ratio %u/%u: %d\n",
				fsrc_n, fsrc_m, ret);
			return ret;
		}
	}
	/* Similar to public/inc/adi_apollo_fsrc.h@adi_apollo_fsrc_rate_set python example */
	ret = adi_apollo_fsrc_mode_1x_enable_set(&phy->ad9088, ADI_APOLLO_TX, ADI_APOLLO_FSRC_ALL,
						 fsrc_m == fsrc_n);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_fsrc_mode_1x_enable_set");
	if (ret)
		return ret;


	if (fsrc_m != fsrc_n) {
		ret = adi_apollo_fsrc_ratio_set(&phy->ad9088, ADI_APOLLO_TX, ADI_APOLLO_FSRC_ALL,
						fsrc_n, fsrc_m);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
						"adi_apollo_fsrc_ratio_set");
	}

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
			dev_warn(&phy->spi->dev,
				 "Failed to trigger sequencer via reg: %d. ", ret);
		}

		/*
		 * Wait for sequencer to complete
		 * Timing: first_trig_cnt (1002) + margin
		 * SYSREF of 4MHz: 1050 * 250ns = 262.5us
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

	return 0;
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

	ret = ad9088_fsrc_trigger_reconfig_sequence(phy);
	if (ret)
		return ret;

	/* Wait for samples to flow through RX path - needed for RMFIFO status to clear */
	usleep_range(100, 200);

	/* Reset the JRX rate-match FIFO to clear any full/empty status from rate change */
	ret = adi_apollo_jrx_rm_fifo_reset(&phy->ad9088,
					   ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_jrx_rm_fifo_reset");
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

	/* FPGA sends all invalid samples (stop sending valid data) */
	ret = ad9088_axi_fsrc_enable(phy, enable, ADI_APOLLO_TX);
	if (ret)
		return ret;

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

	/* Allow samples to flow - needed for RMFIFO status to clear */
	usleep_range(100, 200);

	/* Reset the rate-match FIFO to clear any full/empty status */
	ret = adi_apollo_jrx_rm_fifo_reset(&phy->ad9088,
					   ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_jrx_rm_fifo_reset");
	if (ret)
		return ret;

	dev_dbg(&phy->spi->dev, "TX FSRC reconfig sequence complete\n");
	return 0;
}


static ssize_t ad9088_fsrc_print_block(char *buf, size_t size, ssize_t offset,
				       const char *name, const adi_apollo_fsrc_inspect_t *fsrc)
{
	ssize_t len = offset;

	len += snprintf(buf + len, size - len, "  %s:\n", name);
	len += snprintf(buf + len, size - len, "    enable:           %d\n", fsrc->dp_cfg.enable);
	len += snprintf(buf + len, size - len, "    mode_1x:          %d\n", fsrc->dp_cfg.mode_1x);
	len += snprintf(buf + len, size - len, "    fsrc_bypass:      %d\n", fsrc->fsrc_bypass);
	len += snprintf(buf + len, size - len, "    fsrc_rate_int:    0x%llx\n", fsrc->dp_cfg.fsrc_rate_int);
	len += snprintf(buf + len, size - len, "    fsrc_rate_frac_a: 0x%llx\n", fsrc->dp_cfg.fsrc_rate_frac_a);
	len += snprintf(buf + len, size - len, "    fsrc_rate_frac_b: 0x%llx\n", fsrc->dp_cfg.fsrc_rate_frac_b);
	len += snprintf(buf + len, size - len, "    gain_reduction:   0x%x\n", fsrc->dp_cfg.gain_reduction);
	len += snprintf(buf + len, size - len, "    fsrc_delay:       0x%x\n", fsrc->dp_cfg.fsrc_delay);

	return len;
}

/**
 * ad9088_fsrc_inspect - Inspect FSRC
 * @phy: AD9088 device instance
 *
 * Reads and displays all FSRC block configuration (N, M, rate_int, rate_frac)
 * for both RX and TX paths across all FSRC blocks (A0, A1, B0, B1).
 *
 * Returns: Length on success, 0 otherwise
 */
int ad9088_fsrc_inspect(struct ad9088_phy *phy)
{
	adi_apollo_fsrc_inspect_t fsrc[2][ADI_APOLLO_FSRC_NUM]; /* [0]=RX, [1]=TX */
	const u16 fsrcs[] = { ADI_APOLLO_FSRC_A0, ADI_APOLLO_FSRC_A1,
			      ADI_APOLLO_FSRC_B0, ADI_APOLLO_FSRC_B1 };
	const char *fsrc_names[] = { "A0", "A1", "B0", "B1" };
	const struct {
		adi_apollo_terminal_e terminal;
		const char *name;
	} terminals[] = {
		{ ADI_APOLLO_RX, "Rx" },
		{ ADI_APOLLO_TX, "Tx" }
	};
	ssize_t len = 0;
	int ret, i, t;

	/* Inspect all FSRC blocks for both RX and TX */
	for (t = 0; t < ARRAY_SIZE(terminals); t++) {
		for (i = 0; i < ADI_APOLLO_FSRC_NUM; i++) {
			ret = adi_apollo_fsrc_inspect(&phy->ad9088,
						      terminals[t].terminal,
						      fsrcs[i], &fsrc[t][i]);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
							"adi_apollo_fsrc_inspect");
			if (ret)
				return 0;
		}
	}

	/* Print tree-style output */
	for (t = 0; t < ARRAY_SIZE(terminals); t++) {
		len += snprintf(phy->dbuf + len, sizeof(phy->dbuf) - len,
				"%s FSRC:\n", terminals[t].name);
		for (i = 0; i < ADI_APOLLO_FSRC_NUM; i++) {
			len = ad9088_fsrc_print_block(phy->dbuf, sizeof(phy->dbuf),
						      len, fsrc_names[i], &fsrc[t][i]);
		}
		len += snprintf(phy->dbuf + len, sizeof(phy->dbuf) - len, "\n");
	}

	len += snprintf(phy->dbuf + len, sizeof(phy->dbuf) - len,
			"  FSRC ratio (N/M) = 2^48 / (fsrc_rate_int + fsrc_rate_frac_a/fsrc_rate_frac_b)\n");

	return len;
}
