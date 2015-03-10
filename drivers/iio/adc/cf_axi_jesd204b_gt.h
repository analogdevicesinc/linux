/*
 * ADI AXI-JESD204B GT Interface Module
 *
 * Copyright 2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * http://wiki.analog.com/resources/fpga/xilinx/
 */

#ifndef ADI_JESD204B_GT_H_
#define ADI_JESD204B_GT_H_

#define ADI_REG_VERSION		0x0000				/*Version and Scratch Registers */
#define ADI_VERSION(x)		(((x) & 0xffffffff) << 0)	/* RO, Version number. */
#define VERSION_IS(x,y,z)	((x) << 16 | (y) << 8 | (z))
#define ADI_REG_ID		0x0004			 	/*Version and Scratch Registers */
#define ADI_ID(x)		(((x) & 0xffffffff) << 0)   	/* RO, Instance identifier number. */
#define ADI_REG_SCRATCH		0x0008			 	/*Version and Scratch Registers */
#define ADI_SCRATCH(x)		(((x) & 0xffffffff) << 0)	/* RW, Scratch register. */

#define PCORE_VERSION(major, minor, letter) ((major << 16) | (minor << 8) | letter)
#define PCORE_VERSION_MAJOR(version) (version >> 16)
#define PCORE_VERSION_MINOR(version) ((version >> 8) & 0xff)
#define PCORE_VERSION_LETTER(version) (version & 0xff)

/* JESD GT */

#define ADI_REG_CPLL_PD		0x0010
#define ADI_CPLL_PD			(1 << 0)
#define ADI_LPM_EN			(1 << 1)

#define ADI_REG_RSTN_1		0x0014
#define ADI_DRP_RSTN			(1 << 1)
#define ADI_GT_PLL_RSTN			(1 << 0)

#define ADI_REG_RX_GT_RSTN		0x0020
#define ADI_RX_GT_RSTN			(1 << 0)

#define ADI_REG_RX_RSTN		0x0024
#define ADI_RX_RSTN			(1 << 0)

#define ADI_REG_RX_CLK_SEL		0x0028
#define ADI_RX_SYS_CLK_SEL(x)		(((x) & 0x3) << 4)
#define ADI_TO_RX_SYS_CLK_SEL(x)		(((x) >> 4) & 0x3)
#define ADI_RX_OUT_CLK_SEL(x)		(((x) & 0x7) << 0)
#define ADI_TO_RX_OUT_CLK_SEL(x)		(((x) >> 0) & 0x7)

#define ADI_REG_RX_SYSREF_CTL		0x002C
#define ADI_RX_SYSREF_SEL			(1 << 1)
#define ADI_RX_SYSREF			(1 << 0)

#define ADI_REG_RX_SYNC_CTL		0x0030
#define ADI_RX_SYNC			(1 << 0)

#define ADI_REG_RX_STATUS		0x0034
#define ADI_RX_STATUS			(1 << 16)
#define ADI_RX_RST_DONE(x)		(((x) & 0xFF) << 8)
#define ADI_TO_RX_RST_DONE(x)		(((x) >> 8) & 0xFF)
#define ADI_RX_PLL_LOCKED(x)		(((x) & 0xFF) << 0)
#define ADI_TO_RX_PLL_LOCKED(x)		(((x) >> 0) & 0xFF)

#define ADI_REG_TX_GT_RSTN		0x0060
#define ADI_TX_GT_RSTN			(1 << 0)

#define ADI_REG_TX_RSTN		0x0064
#define ADI_TX_RSTN			(1 << 0)

#define ADI_REG_TX_CLK_SEL		0x0068
#define ADI_TX_SYS_CLK_SEL(x)		(((x) & 0x3) << 4)
#define ADI_TO_TX_SYS_CLK_SEL(x)		(((x) >> 4) & 0x3)
#define ADI_TX_OUT_CLK_SEL(x)		(((x) & 0x7) << 0)
#define ADI_TO_TX_OUT_CLK_SEL(x)		(((x) >> 0) & 0x7)

#define ADI_REG_TX_SYSREF_CTL		0x006C
#define ADI_TX_SYSREF_SEL			(1 << 1)
#define ADI_TX_SYSREF			(1 << 0)

#define ADI_REG_TX_SYNC_CTL		0x0070
#define ADI_TX_SYNC			(1 << 0)

#define ADI_REG_TX_STATUS		0x0074
#define ADI_TX_STATUS			(1 << 16)
#define ADI_TX_RST_DONE(x)		(((x) & 0xFF) << 8)
#define ADI_TO_TX_RST_DONE(x)		(((x) >> 8) & 0xFF)
#define ADI_TX_PLL_LOCKED(x)		(((x) & 0xFF) << 0)
#define ADI_TO_TX_PLL_LOCKED(x)		(((x) >> 0) & 0xFF)

#define ADI_REG_RX_LANESEL		0x008C
#define ADI_RX_LANESEL(x)		(((x) & 0xFF) << 0)
#define ADI_TO_RX_LANESEL(x)		(((x) >> 0) & 0xFF)

#define ADI_REG_DRP_CNTRL		0x0090
#define ADI_DRP_RWN			(1 << 28)
#define ADI_DRP_ADDRESS(x)		(((x) & 0xFFF) << 16)
#define ADI_TO_DRP_ADDRESS(x)		(((x) >> 16) & 0xFFF)
#define ADI_DRP_WDATA(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_DRP_WDATA(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_DRP_STATUS		0x0094
#define ADI_DRP_STATUS			(1 << 16)
#define ADI_DRP_RDATA(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_DRP_RDATA(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_EYESCAN_CNTRL		0x00A0
#define ADI_EYESCAN_INIT			(1 << 2)
#define ADI_EYESCAN_STOP			(1 << 1)
#define ADI_EYESCAN_START			(1 << 0)

#define ADI_REG_EYESCAN_PRESCALE		0x00A4
#define ADI_EYESCAN_PRESCALE(x)		(((x) & 0x1F) << 0)
#define ADI_TO_EYESCAN_PRESCALE(x)		(((x) >> 0) & 0x1F)

#define ADI_REG_EYESCAN_VOFFSET		0x00A8
#define ADI_EYESCAN_VOFFSET_STEP(x)		(((x) & 0xFF) << 16)
#define ADI_TO_EYESCAN_VOFFSET_STEP(x)		(((x) >> 16) & 0xFF)
#define ADI_EYESCAN_VOFFSET_MAX(x)		(((x) & 0xFF) << 8)
#define ADI_TO_EYESCAN_VOFFSET_MAX(x)		(((x) >> 8) & 0xFF)
#define ADI_EYESCAN_VOFFSET_MIN(x)		(((x) & 0xFF) << 0)
#define ADI_TO_EYESCAN_VOFFSET_MIN(x)		(((x) >> 0) & 0xFF)

#define ADI_REG_EYESCAN_HOFFSET_1		0x00AC
#define ADI_EYESCAN_HOFFSET_MAX(x)		(((x) & 0xFFF) << 16)
#define ADI_TO_EYESCAN_HOFFSET_MAX(x)		(((x) >> 16) & 0xFFF)
#define ADI_EYESCAN_HOFFSET_MIN(x)		(((x) & 0xFFF) << 0)
#define ADI_TO_EYESCAN_HOFFSET_MIN(x)		(((x) >> 0) & 0xFFF)

#define ADI_REG_EYESCAN_HOFFSET_2		0x00B0
#define ADI_EYESCAN_HOFFSET_STEP(x)		(((x) & 0xFFF) << 0)
#define ADI_TO_EYESCAN_HOFFSET_STEP(x)		(((x) >> 0) & 0xFFF)

#define ADI_REG_EYESCAN_DMA_STARTADDR		0x00B4
#define ADI_EYESCAN_DMA_STARTADDR(x)		(((x) & 0xFFFFFFFF) << 0)
#define ADI_TO_EYESCAN_DMA_STARTADDR(x)		(((x) >> 0) & 0xFFFFFFFF)

#define ADI_REG_EYESCAN_SDATA_1_0		0x00B8
#define ADI_EYESCAN_SDATA1(x)		(((x) & 0xFFFF) << 16)
#define ADI_TO_EYESCAN_SDATA1(x)		(((x) >> 16) & 0xFFFF)
#define ADI_EYESCAN_SDATA0(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_EYESCAN_SDATA0(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_EYESCAN_SDATA_3_2		0x00BC
#define ADI_EYESCAN_SDATA3(x)		(((x) & 0xFFFF) << 16)
#define ADI_TO_EYESCAN_SDATA3(x)		(((x) >> 16) & 0xFFFF)
#define ADI_EYESCAN_SDATA2(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_EYESCAN_SDATA2(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_EYESCAN_SDATA_4		0x00C0
#define ADI_EYESCAN_SDATA4(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_EYESCAN_SDATA4(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_EYESCAN_QDATA_1_0		0x00C4
#define ADI_EYESCAN_QDATA1(x)		(((x) & 0xFFFF) << 16)
#define ADI_TO_EYESCAN_QDATA1(x)		(((x) >> 16) & 0xFFFF)
#define ADI_EYESCAN_QDATA0(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_EYESCAN_QDATA0(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_EYESCAN_QDATA_3_2		0x00C8
#define ADI_EYESCAN_QDATA3(x)		(((x) & 0xFFFF) << 16)
#define ADI_TO_EYESCAN_QDATA3(x)		(((x) >> 16) & 0xFFFF)
#define ADI_EYESCAN_QDATA2(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_EYESCAN_QDATA2(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_EYESCAN_QDATA_4		0x00CC
#define ADI_EYESCAN_QDATA4(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_EYESCAN_QDATA4(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_EYESCAN_STATUS		0x00E0
#define ADI_EYESCAN_DMAERR			(1 << 1)
#define ADI_EYESCAN_STATUS			(1 << 0)

#define ADI_REG_EYESCAN_RATE		0x00E4
#define ADI_EYESCAN_RATE			(1 << 1)

#define ADI_REG_TRANSCEIVER_TYPE		0x00E8
#define ADI_TRANSCEIVER_GTH		0x1 /* Ultra-Scale */
#define ADI_TRANSCEIVER_GTX		0x0 /* 7-Series */

#endif /* ADI_JESD204B_GT_H_ */
