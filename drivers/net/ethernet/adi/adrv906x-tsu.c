// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include "adrv906x-tsu.h"

void adrv906x_tsu_calculate_phy_delay(struct adrv906x_tsu *tsu, int speed,
				      bool rs_fec_enabled, u32 bit_slip,
				      u32 buf_delay_tx, u32 buf_delay_rx)
{
	u32 rx_pcs, rx_pcs_pipeline, rx_pcs_decode, rx_pcs_gearbox, rx_rs_fec;
	u32 tx_pcs, tx_pcs_pipeline, tx_pcs_decode, tx_pcs_gearbox, tx_rs_fec;
	u32 t_div66, t_div64;
	u32 tod_cdc_delay;

	t_div66 = (speed == SPEED_25000) ? T_DIV66_25G : T_DIV66_10G;
	t_div64 = (speed == SPEED_25000) ? T_DIV64_25G : T_DIV64_10G;

	/* ToD_cdc_delay = 1.5 * 1 / hsdig_clk + (cdc_delay + 3.0) * T_div66 */

	tod_cdc_delay = (1000000000ULL << 16) / tsu->hsdig_clk_rate;
	tod_cdc_delay = tod_cdc_delay * 3 / 2;
	tod_cdc_delay += (adrv906x_tod_cfg_cdc_delay + 3) * t_div66;

	/* Calculate Rx_static_phy_delay */

	/* Rx_pcs = Rx_pcs_pipeline + Rx_pcs_decode + Rx_pcs_gearbox + Rx_rs_fec
	 *
	 * Rx_pcs_pipeline = 2 * T_div66 + 2 * T_div64
	 * Rx_pcs_decode   = 6 * T_div66 - bit_slip / 66 * T_div66
	 * Rx_pcs_gearbox  = floor(fine_buf_receive_delay) * T_div66
	 * if RS-FEC is used
	 *   Rx_rs_fec       = 234 * T_div66
	 */

	rx_pcs_pipeline = 2 * t_div66 + 2 * t_div64;
	rx_pcs_decode = 6 * t_div66 - bit_slip * t_div66 / 66;
	rx_pcs_gearbox = buf_delay_rx * t_div66;
	rx_rs_fec = rs_fec_enabled ? 234 * t_div66 : 0;

	rx_pcs = rx_pcs_pipeline + rx_pcs_decode + rx_pcs_gearbox + rx_rs_fec;

	/* Rx_static_phy_delay = Rx_pcs - ToD_cdc_delay - PCB_delay */

	if (rx_pcs > tod_cdc_delay + tsu->pcb_delay_rx)
		tsu->phy_delay_rx = rx_pcs - tod_cdc_delay - tsu->pcb_delay_rx;
	else
		tsu->phy_delay_rx = 0;

	/* Calculate Tx_static_phy_delay */

	/* Tx_pcs = Tx_pcs_pipeline + Tx_pcs_decode + Tx_pcs_gearbox + Tx_rs_fec
	 *
	 * Tx_pcs_pipeline = 2 * T_div66 + T_div64
	 * Tx_pcs_decode   = 4 * T_div66
	 * Tx_pcs_gearbox  = floor(fine_buf_transmit_delay) * T_div66
	 * if RS-FEC is used
	 *   Tx_rs_fec       = 17 * T_div66
	 */

	tx_pcs_pipeline = 2 * t_div66 + t_div64;
	tx_pcs_decode = 4 * t_div66;
	tx_pcs_gearbox = buf_delay_tx * t_div66;
	tx_rs_fec = rs_fec_enabled ? 17 * t_div66 : 0;

	tx_pcs = tx_pcs_pipeline + tx_pcs_decode + tx_pcs_gearbox + tx_rs_fec;

	/* Tx_static_phy_delay = Tx_pcs + ToD_cdc_delay + PCB_delay */

	tsu->phy_delay_tx = tx_pcs + tod_cdc_delay + tsu->pcb_delay_tx;
}

void adrv906x_tsu_set_phy_delay(struct adrv906x_tsu *tsu)
{
	void __iomem *base = tsu->base;

	iowrite32(tsu->phy_delay_tx, base + ADRV906X_TSU_STATIC_PHY_DELAY_TX);
	iowrite32(tsu->phy_delay_rx, base + ADRV906X_TSU_STATIC_PHY_DELAY_RX);
}

void adrv906x_tsu_set_ptp_timestamping_mode(void __iomem *base)
{
	u32 mode, val;

	mode = ADRV906X_PTP_TIMESTAMPING_MODE_TWO_STEP;

	val = ioread32(base + ADRV906X_TSU_TIMESTAMPING_MODE);
	val &= ~ADRV906X_PTP_TIMESTAMPING_MODE;
	val |= (mode & ADRV906X_PTP_TIMESTAMPING_MODE);

	iowrite32(val, base + ADRV906X_TSU_TIMESTAMPING_MODE);
}

void adrv906x_tsu_set_speed(struct adrv906x_tsu *tsu, int speed)
{
	u32 mode, val;
	void __iomem *base;

	base = tsu->base;
	mode = (speed == SPEED_25000) ? ADRV906X_CORE_SPEED_25G : ADRV906X_CORE_SPEED_10G;

	val = ioread32(base + ADRV906X_TSU_TIMESTAMPING_MODE);
	val &= ~ADRV906X_CORE_SPEED;
	val |= (mode & ADRV906X_CORE_SPEED);

	iowrite32(val, base + ADRV906X_TSU_TIMESTAMPING_MODE);
}

int adrv906x_tsu_setup(struct platform_device *pdev, struct adrv906x_tsu *tsu,
		       struct device_node *eth_port_np)
{
	struct device *dev = &pdev->dev;
	struct clk *hsdig_clk;
	u32 val, frac_val, reg, len;
	int ret;

	hsdig_clk = devm_get_clk_from_child(dev, eth_port_np, "hsdig_clk");
	if (IS_ERR(hsdig_clk)) {
		dev_err(dev, "cannot get 'hsdig_clk'");
		return PTR_ERR(hsdig_clk);
	}
	tsu->hsdig_clk_rate = clk_get_rate(hsdig_clk);

	of_property_read_u32_index(eth_port_np, "reg", 6, &reg);
	of_property_read_u32_index(eth_port_np, "reg", 7, &len);

	tsu->base = devm_ioremap(dev, reg, len);
	if (IS_ERR(tsu->base))
		return PTR_ERR(tsu->base);

	ret = of_property_read_u32(eth_port_np, "adi,pcb-delay-tx-ns", &val);
	if (ret < 0 || val > 0xffff)
		dev_warn(dev, "dt: adi,pcb-delay-tx-ns missing or invalid, using 0");

	ret = of_property_read_u32(eth_port_np, "adi,pcb-delay-tx-frac-ns", &frac_val);
	if (ret < 0 || val > 0xffff) {
		dev_warn(dev, "dt: adi,pcb-delay-tx-frac-ns missing or invalid, using 0");
		frac_val = 0;
	}

	tsu->pcb_delay_tx = (val << 16) | (frac_val & 0xFFFF);
	dev_info(dev, "tsu pcb delay tx 0x%08x", tsu->pcb_delay_tx);

	ret = of_property_read_u32(eth_port_np, "adi,pcb-delay-rx-ns", &val);
	if (ret < 0 || val > 0xffff) {
		dev_warn(dev, "dt: adi,pcb-delay-rx-ns missing or invalid, using 0");
		val = 0;
	}

	ret = of_property_read_u32(eth_port_np, "adi,pcb-delay-rx-frac-ns", &frac_val);
	if (ret < 0 || val > 0xffff) {
		dev_warn(dev, "dt: adi,pcb-delay-rx-frac-ns missing or invalid, using 0");
		frac_val = 0;
	}

	tsu->pcb_delay_rx = (val << 16) | (frac_val & 0xFFFF);
	dev_info(dev, "tsu pcb delay rx 0x%08x", tsu->pcb_delay_rx);

	adrv906x_tsu_set_ptp_timestamping_mode(tsu->base);

	return 0;
}

void adrv906x_tsu_compensate_tx_tstamp(struct adrv906x_tsu *tsu, struct timespec64 *ts)
{
    timespec64_add_ns(ts, tsu->phy_delay_tx >> 16);
}