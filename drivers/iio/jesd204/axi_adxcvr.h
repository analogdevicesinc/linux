/*
 * ADI AXI-ADXCVR Module
 *
 * Copyright 2016-2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * https://wiki.analog.com/resources/fpga/docs/axi_adxcvr
 */

#ifndef AXI_ADXCVR_H_
#define AXI_ADXCVR_H_

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/mutex.h>
#include <linux/fpga/adi-axi-common.h>
#include <linux/jesd204/jesd204.h>

#include "xilinx_transceiver.h"

#define ADXCVR_REG_RESETN		0x0010
#define ADXCVR_RESETN			(1 << 0)
#define ADXCVR_BUFSTATUS_RST	(1 << 1)

#define ADXCVR_REG_STATUS		0x0014
#define ADXCVR_STATUS			(1 << 0)

#define ADXCVR_REG_CONTROL		0x0020
#define ADXCVR_LPM_DFE_N		(1 << 12)
#define ADXCVR_RATE(x)			(((x) & 0x7) << 8)
#define ADXCVR_SYSCLK_SEL(x)		(((x) & 0x3) << 4)
#define ADXCVR_OUTCLK_SEL(x)		(((x) & 0x7) << 0)

#define ADXCVR_REG_SYNTH		0x24

#define ADXCVR_REG_DRP_SEL(x)		(0x0040 + (x))

#define ADXCVR_REG_DRP_CTRL(x)		(0x0044 + (x))
#define ADXCVR_DRP_CTRL_WR		(1 << 28)
#define ADXCVR_DRP_CTRL_ADDR(x)		(((x) & 0xFFF) << 16)
#define ADXCVR_DRP_CTRL_WDATA(x)	(((x) & 0xFFFF) << 0)

#define ADXCVR_REG_DRP_STATUS(x)	(0x0048 + (x))
#define ADXCVR_DRP_STATUS_BUSY		(1 << 16)
#define ADXCVR_DRP_STATUS_RDATA(x)	(((x) & 0xFFFF) << 0)

#define ADXCVR_DRP_PORT_ADDR_COMMON	0x00
#define ADXCVR_DRP_PORT_ADDR_CHANNEL	0x20

#define ADXCVR_DRP_PORT_COMMON(x)	(x)
#define ADXCVR_DRP_PORT_CHANNEL(x)	(0x100 + (x))

#define ADXCVR_REG_REG_PRBS_CNTRL	0x0180
#define ADXCVR_PRBSEL(x)		(((x) & 0xF) << 0)
#define ADXCVR_PRBS_CNT_RESET		BIT(8)
#define ADXCVR_PRBS_FORCE_ERR		BIT(16)

#define ADXCVR_REG_REG_PRBS_STATUS	0x0184
#define ADXCVR_PRBS_ERR(x)		((x) & BIT(8))
#define ADXCVR_PRBS_LOCKED(x)		((x) & BIT(0))

struct adxcvr_state {
	struct device		*dev;
	void __iomem		*regs;
	struct jesd204_dev	*jdev;
	struct clk		*conv_clk;
	struct clk		*conv2_clk;
	struct clk		*lane_rate_div40_clk;
	struct clk_hw		lane_clk_hw;
	struct clk_hw		qpll_clk_hw;
	struct work_struct	work;
	struct delayed_work jesd_fsm_en_work;
	struct mutex		mutex;
	unsigned long		lane_rate;
	bool			tx_enable;
	bool			qpll_enable;
	u32			sys_clk_sel;
	u32			out_clk_sel;
	u32			fsm_enable_delay_ms;
	int			fsm_start_delayed_ret;

	struct clk_hw_onecell_data *clk_lookup;

	struct xilinx_xcvr	xcvr;
	struct adxcvr_eyescan   *eye;

	bool			cpll_enable;
	bool			lpm_enable;
	bool			ref_is_div40;

	unsigned int		num_lanes;
	unsigned int		addr;
};

#endif /* AXI_ADXCVR_H_ */
