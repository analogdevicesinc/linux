// SPDX-License-Identifier: GPL-2.0
/*
 * FSRC (Fractional Sample Rate Converter) support for AD9088
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/string.h>

#include "ad9088.h"

static int ad9088_axi_fsrc_enable(struct ad9088_phy *phy, bool enable)
{
	int ret;

	if (!phy->iio_axi_fsrc) {
		dev_dbg(&phy->spi->dev, "FSRC channel not available\n");
		return -ENODEV;
	}

	ret = ad9088_iio_write_channel_ext_info(phy, phy->iio_axi_fsrc,
						"tx_enable", enable ? 1 : 0);
	if (ret < 0) {
		dev_err(&phy->spi->dev, "Failed to %s TX FSRC: %d\n",
			enable ? "enable" : "disable", ret);
		return ret;
	}

	dev_dbg(&phy->spi->dev, "TX FSRC %s\n", enable ? "enabled" : "disabled");
	return 0;
}

/**
 * ad9088_axi_fsrc_active - Start/stop TX FSRC transmission
 * @phy: AD9088 device instance
 * @active: true to start, false to stop (send invalids)
 *
 * Returns: 0 on success, negative error code on failure
 */
static int ad9088_axi_fsrc_active(struct ad9088_phy *phy, bool active)
{
	int ret;

	if (!phy->iio_axi_fsrc) {
		dev_dbg(&phy->spi->dev, "FSRC channel not available\n");
		return -ENODEV;
	}

	ret = ad9088_iio_write_channel_ext_info(phy, phy->iio_axi_fsrc,
						"tx_active", active ? 1 : 0);
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
 * ad9088_fsrc_configure_rx - Configure RX FSRC and data path
 * @phy: AD9088 device instance
 * @fsrc_n: FSRC N value
 * @fsrc_m: FSRC M value
 * @cddc_dcm: CDDC decimation ratio (to drop)
 * @fddc_dcm: FDDC decimation ratio (to drop)
 *
 * Configures the RX FSRC ratio and DDC (Digital Down Converter) settings.
 * Based on apollo_rx_fsrc_configure() from fullchip_fsrc_dr.c
 *
 * Returns: 0 on success, negative error code on failure
 */
int ad9088_fsrc_configure_rx(struct ad9088_phy *phy, u32 fsrc_n, u32 fsrc_m,
			     adi_apollo_coarse_ddc_dcm_e cddc_dcm,
			     adi_apollo_fddc_ratio_e fddc_dcm)
{
	int ret;

	dev_dbg(&phy->spi->dev, "Configuring RX FSRC: N=%u M=%u\n", fsrc_n, fsrc_m);

	ret = adi_apollo_fsrc_ratio_set(&phy->ad9088, ADI_APOLLO_RX, ADI_APOLLO_FSRC_ALL,
					fsrc_n, fsrc_m);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_fsrc_ratio_set");
	if (ret)
		return ret;

	dev_dbg(&phy->spi->dev, "RX FSRC configuration complete\n");
	return 0;
}

/**
 * ad9088_fsrc_configure_tx - Configure TX FSRC and data path
 * @phy: AD9088 device instance
 * @fsrc_n: FSRC N value
 * @fsrc_m: FSRC M value
 * @cduc_interp: CDUC interpolation ratio (to drop)
 * @fduc_interp: FDUC interpolation ratio (to drop)
 *
 * Configures the TX FSRC ratio and DUC (Digital Up Converter) settings.
 * Based on apollo_tx_fsrc_configure() from fullchip_fsrc_dr.c
 *
 * Returns: 0 on success, negative error code on failure
 */
int ad9088_fsrc_configure_tx(struct ad9088_phy *phy, u32 fsrc_n, u32 fsrc_m,
			     adi_apollo_coarse_duc_dcm_e cduc_interp,
			     adi_apollo_fduc_ratio_e fduc_interp)
{
	char ratio_str[64];
	int ret;

	dev_dbg(&phy->spi->dev, "Configuring TX FSRC: N=%u M=%u\n", fsrc_n, fsrc_m);


	if (!phy->iio_axi_fsrc) {
		dev_dbg(&phy->spi->dev, "FSRC channel not available\n");
		return -ENODEV;
	}

	snprintf(ratio_str, sizeof(ratio_str), "%u %u", fsrc_n, fsrc_m);

	ret = iio_write_channel_ext_info(phy->iio_axi_fsrc, "tx_ratio_set",
					  ratio_str, strlen(ratio_str) + 1);
	if (ret < 0) {
		dev_err(&phy->spi->dev, "Failed to set TX FSRC ratio %u/%u: %d\n",
			fsrc_n, fsrc_m, ret);
		return ret;
	}
	/* Similar to public/inc/adi_apollo_fsrc.h@adi_apollo_fsrc_rate_set python example */
	ret = adi_apollo_fsrc_ratio_set(&phy->ad9088, ADI_APOLLO_TX, ADI_APOLLO_FSRC_ALL,
					fsrc_n, fsrc_m);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_fsrc_ratio_set");
	if (ret)
		return ret;

	dev_dbg(&phy->spi->dev, "TX FSRC configuration complete\n");
	return 0;
}

/**
 * ad9088_fsrc_tx_reconfig_sequence_spi - Execute TX FSRC dynamic reconfig via SPI trigger
 * @phy: AD9088 device instance
 *
 * Executes the TX-only FSRC dynamic reconfiguration sequence using SPI/regmap trigger.
 *
 * TX Path Invalid Sample Flow:
 *   FPGA TX FSRC → adds invalid samples (-FS)
 *   Apollo TX FSRC → removes invalid samples
 *   DAC → clean analog output
 *
 * Sequence:
 * 1. Stop FPGA TX (send only invalid samples)
 * 2. Wait for invalids to flow through JESD
 * 3. Enable trigger sync on Apollo
 * 4. Execute manual reconfig (SPI trigger - applies new ratio/CDUC/FDUC)
 * 5. Resume FPGA TX (send valid+invalid samples at new rate)
 * 6. Wait for samples to flow
 * 7. Reset rate-match FIFO
 *
 * Note: Apollo FSRC TX block is already enabled by profile. Reconfig only applies new settings.
 *
 * Based on fpga_hw_fsrc_dr_seq_run() from fullchip_fsrc_dr.c
 *
 * Returns: 0 on success, negative error code on failure
 */
int ad9088_fsrc_tx_reconfig_sequence_spi(struct ad9088_phy *phy)
{
	int ret;

	dev_dbg(&phy->spi->dev, "Starting TX FSRC SPI reconfig sequence\n");

	/* FPGA TX sends all invalid samples (stop sending valid data)
	 * TX Path: FPGA adds invalids → Apollo FSRC TX removes invalids → clean DAC output
	 */
	ret = ad9088_axi_fsrc_enable(phy, true);
	if (ret)
		return ret;

	/* Allow invalids to flow through JESD link */
	usleep_range(10000, 11000);

	/* Enable trigger sync - resync Tx digital blocks during reconfig */
	ret = adi_apollo_clk_mcs_trig_reset_dsp_enable(&phy->ad9088);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_clk_mcs_trig_reset_dsp_enable");
	if (ret)
		return ret;

	/* Execute manual dynamic reconfig. */
	ret = adi_apollo_clk_mcs_man_reconfig_sync(&phy->ad9088);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_clk_mcs_man_reconfig_sync");
	if (ret)
		return ret;

	/* Resume FPGA TX - sends valid and invalid samples at new rate */
	ret = ad9088_axi_fsrc_active(phy, true);
	if (ret)
		return ret;

	/* Allow samples to flow - needed for RMFIFO status to clear */
	usleep_range(100, 200);

	/* Reset the rate-match FIFO to clear any full/empty status */
	ret = adi_apollo_jrx_rm_fifo_reset(&phy->ad9088,
					   ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_jrx_rm_fifo_reset");
	if (ret)
		return ret;

	dev_dbg(&phy->spi->dev, "TX FSRC SPI reconfig sequence complete\n");
	return 0;
}

/**
 * ad9088_fsrc_tx_reconfig_sequence_gpio - Execute TX FSRC GPIO-triggered reconfig
 * @phy: AD9088 device instance
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
int ad9088_fsrc_tx_reconfig_sequence_gpio(struct ad9088_phy *phy)
{
	int ret;

	dev_dbg(&phy->spi->dev, "Starting TX FSRC GPIO reconfig sequence\n");

	/* FPGA sends all invalid samples (stop sending valid data) */
	ret = ad9088_axi_fsrc_enable(phy, true);
	if (ret)
		return ret;

	/* Wait for invalids to flow */
	usleep_range(10000, 11000);

	/* Apollo forces JTx invalids - TX will send only invalid samples */
	ret = adi_apollo_jtx_force_invalids_set(&phy->ad9088, ADI_APOLLO_LINK_ALL, 1);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_jtx_force_invalids_set");
	if (ret)
		return ret;

	/* Wait for invalids to propagate through the system */
	usleep_range(1000000, 1100000);  /* 1000ms */

	/* Enable Apollo trigger sync - Apollo will wait for external GPIO trigger */
	ret = adi_apollo_clk_mcs_trig_sync_enable(&phy->ad9088, 1);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_clk_mcs_trig_sync_enable");
	if (ret)
		return ret;

	/* Trigger FPGA sequencer to start the SYSREF-based sequence */
	ret = ad9088_iio_write_channel_ext_info(phy, phy->iio_axi_fsrc,
						"seq_start", 1);
	if (ret < 0) {
		dev_warn(&phy->spi->dev,
			 "Failed to trigger sequencer via reg: %d. "
			 "Use external GPIO trigger instead.\n", ret);
	}

	/* Wait for sequencer to complete
	 * Timing: first_trig_cnt (1002) + margin = ~1050 SYSREF cycles
	 * At typical SYSREF of 4MHz: 1050 * 250ns = 262.5us
	 * Add margin for safety
	 */
	usleep_range(500, 1000);  /* 500us */

	/* Clear Apollo trigger sync (not self-clearing) */
	ret = adi_apollo_clk_mcs_trig_sync_enable(&phy->ad9088, 0);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_clk_mcs_trig_sync_enable clear");
	if (ret)
		return ret;

	/* Resume FPGA TX - send valid and invalid samples */
	ret = ad9088_axi_fsrc_active(phy, true);
	if (ret)
		return ret;

	/* Wait for samples to flow - needed for rmfifo status to clear */
	usleep_range(100, 200);

	/* Reset the rate-match FIFO */
	ret = adi_apollo_jrx_rm_fifo_reset(&phy->ad9088,
					   ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_jrx_rm_fifo_reset");
	if (ret)
		return ret;

	if (!ret)
		dev_dbg(&phy->spi->dev, "TX FSRC GPIO reconfig sequence complete\n");

	return ret;
}

/**
 * ad9088_fsrc_rx_reconfig_sequence_spi - Execute RX FSRC dynamic reconfig via SPI trigger
 * @phy: AD9088 device instance
 *
 * Executes the RX-only FSRC dynamic reconfiguration sequence using SPI/regmap trigger.
 *
 * RX Path Invalid Sample Flow:
 *   Apollo ADC → Apollo RX FSRC (adds invalid samples -FS)
 *   Apollo JRX → JESD RX Link → FPGA JRX
 *   FPGA RX FSRC (removes invalid samples) → DMA → clean digital data
 *
 * Sequence:
 * 1. Enable trigger sync on Apollo
 * 2. Execute manual reconfig (SPI trigger - applies new RX ratio/CDDC/FDDC)
 * 3. Wait for samples to flow
 * 4. Reset JRX rate-match FIFO
 *
 * Note: Apollo RX FSRC block is already enabled by profile. Reconfig only applies new settings.
 * The RX path is independent - no TX-side manipulation needed.
 *
 * Returns: 0 on success, negative error code on failure
 */
int ad9088_fsrc_rx_reconfig_sequence_spi(struct ad9088_phy *phy)
{
	int ret;

	dev_dbg(&phy->spi->dev, "Starting RX FSRC SPI reconfig sequence\n");

	/* Enable trigger sync - resync RX digital blocks during reconfig */
	ret = adi_apollo_clk_mcs_trig_reset_dsp_enable(&phy->ad9088);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_clk_mcs_trig_reset_dsp_enable");
	if (ret)
		return ret;

	/* Execute manual dynamic reconfig - applies new RX FSRC ratio/CDDC/FDDC configuration
	 * Note: RX FSRC blocks are already enabled by profile. This only applies new settings.
	 */
	ret = adi_apollo_clk_mcs_man_reconfig_sync(&phy->ad9088);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_clk_mcs_man_reconfig_sync");
	if (ret)
		return ret;

	/* Allow samples to flow through RX path - needed for RMFIFO status to clear */
	usleep_range(100, 200);

	/* Reset the JRX rate-match FIFO to clear any full/empty status from rate change */
	ret = adi_apollo_jrx_rm_fifo_reset(&phy->ad9088,
					   ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_jrx_rm_fifo_reset");
	if (ret)
		return ret;

	dev_dbg(&phy->spi->dev, "RX FSRC SPI reconfig sequence complete\n");
	return 0;
}

/**
 * ad9088_fsrc_rx_reconfig_sequence_gpio - Execute RX FSRC GPIO-triggered reconfig
 * @phy: AD9088 device instance
 *
 * Executes RX-only GPIO-triggered FSRC dynamic reconfiguration sequence:
 * 1. Enable Apollo trigger sync (wait for GPIO trigger)
 * 2. Trigger FPGA sequencer (via register or external GPIO)
 * 3. FPGA sequencer counts SYSREFs and sends trigger to Apollo
 * 4. Apollo executes RX reconfig on trigger
 * 5. Clear Apollo trigger sync
 * 6. Wait for samples to flow through RX path
 * 7. Reset JRX rate-match FIFO
 *
 * RX Path Invalid Sample Flow:
 *   Apollo ADC → Apollo RX FSRC (adds invalid samples)
 *   Apollo JRX → JESD RX Link → FPGA JRX
 *   FPGA RX FSRC (removes invalid samples) → DMA → clean digital data
 *
 * Note: The RX path is independent - no TX-side manipulation needed.
 *
 * Returns: 0 on success, negative error code on failure
 */
int ad9088_fsrc_rx_reconfig_sequence_gpio(struct ad9088_phy *phy)
{
	int ret;

	dev_dbg(&phy->spi->dev, "Starting RX FSRC GPIO reconfig sequence\n");

	/* Enable Apollo trigger sync - Apollo will wait for external GPIO trigger */
	ret = adi_apollo_clk_mcs_trig_sync_enable(&phy->ad9088, 1);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_clk_mcs_trig_sync_enable");
	if (ret)
		return ret;

	/* Trigger FPGA sequencer to start the SYSREF-based sequence */
	ret = ad9088_iio_write_channel_ext_info(phy, phy->iio_axi_fsrc,
						"seq_start", 1);
	if (ret < 0) {
		dev_warn(&phy->spi->dev,
			 "Failed to trigger sequencer via reg: %d. "
			 "Use external GPIO trigger instead.\n", ret);
	}

	/*
	 * Wait for sequencer to complete
	 * Timing: first_trig_cnt (1002) + margin = ~1050 SYSREF cycles
	 * At typical SYSREF of 4MHz: 1050 * 250ns = 262.5us
	 */
	usleep_range(500, 1000);  /* 500us */

	/* Clear Apollo trigger sync */
	ret = adi_apollo_clk_mcs_trig_sync_enable(&phy->ad9088, 0);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					"adi_apollo_clk_mcs_trig_sync_enable clear");
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

	dev_dbg(&phy->spi->dev, "RX FSRC GPIO reconfig sequence complete\n");
	return ret;
}
