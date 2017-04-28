/*
 * ADI AXI-JESD204B GT Interface Module
 *
 * Copyright 2014-2015 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * http://wiki.analog.com/resources/fpga/xilinx/
 */

#ifndef ADI_JESD204B_GT_H_
#define ADI_JESD204B_GT_H_

#define ADI_REG_VERSION		0x0000				/*Version and Scratch Registers	*/
#define ADI_VERSION(x)		(((x) &	0xffffffff) << 0)	/* RO, Version number. */
#define VERSION_IS(x,y,z)	((x) <<	16 | (y) << 8 |	(z))
#define ADI_REG_ID		0x0004				/*Version and Scratch Registers	*/
#define ADI_ID(x)		(((x) &	0xffffffff) << 0)	/* RO, Instance	identifier number. */
#define ADI_REG_SCRATCH		0x0008				/*Version and Scratch Registers	*/
#define ADI_SCRATCH(x)		(((x) &	0xffffffff) << 0)	/* RW, Scratch register. */

#define PCORE_VERSION(major, minor, letter) ((major << 16) | (minor << 8) | letter)
#define PCORE_VERSION_MAJOR(version) (version >> 16)
#define PCORE_VERSION_MINOR(version) ((version >> 8) & 0xff)
#define PCORE_VERSION_LETTER(version) (version & 0xff)

#define JESD204B_GT_REG_VERSION		  0x0000
#define JESD204B_GT_VERSION(x)		  (((x)	& 0xffffffff) << 0)
#define JESD204B_GT_VERSION_IS(x,y,z)	  ((x) << 16 | (y) << 8	| (z))
#define JESD204B_GT_VERSION_MAJOR(x)	  ((x) >> 16)

#define JESD204B_GT_REG_ID		  0x0004
#define JESD204B_GT_REG_SCRATCH		  0x0008
#define JESD204B_GT_REG_DRP_RSTN(c)	  (0x014 + (c) * 0x400)
#define JESD204B_GT_DRP_RSTN		  (1 <<	1)
#define JESD204B_GT_REG_RX_OFFSET	  0x0000
#define JESD204B_GT_REG_TX_OFFSET	  0x0040

#define JESD204B_GT_REG_LPM_CPLL_PD(c)	  (0x010 + (c) * 0x400)
#define JESD204B_GT_LPM_DFE(x)		  (((x)	& 1) <<	1)
#define JESD204B_GT_CPLL_PD		  (1 <<	0)

#define JESD204B_GT_REG_RSTN_1(l)	(0x0014	+ (l) *	0x400)
#define JESD204B_GT_DRP_RSTN		(1 << 1)
#define JESD204B_GT_GT_PLL_RSTN		(1 << 0)


#define JESD204B_GT_REG_GT_RSTN(c)	  (0x020 + (c) * 0x400)
#define JESD204B_GT_GT_RSTN		  (1 <<	0)

#define JESD204B_GT_REG_RSTN(c)		  (0x024 + (c) * 0x400)
#define JESD204B_GT_RSTN		  (1 <<	0)

#define JESD204B_GT_REG_CLK_SEL(c)	  (0x028 + (c) * 0x400)
#define JESD204B_GT_SYS_CLK_SEL(x)	  (((x)	& 0x3) << 4)
#define JESD204B_GT_TO_SYS_CLK_SEL(x)	  (((x)	>> 4) &	0x3)
#define JESD204B_GT_OUT_CLK_SEL(x)	  (((x)	& 0x7) << 0)
#define JESD204B_GT_TO_OUT_CLK_SEL(x)	  (((x)	>> 0) &	0x7)

#define JESD204B_GT_REG_SYSREF_CTL(c)	  (0x02C + (c) * 0x400)
#define JESD204B_GT_SYSREF_EXTERNAL	  (1 <<	1)
#define JESD204B_GT_SYSREF_ON		  (1 <<	0)
#define JESD204B_GT_SYSREF_OFF		  (0 <<	0)

#define JESD204B_GT_REG_SYNC_CTL(c)	  (0x030 + (c) * 0x400)
#define JESD204B_GT_SYNC		  (1 <<	0)

#define JESD204B_GT_REG_STATUS(c)	  (0x034 + (c) * 0x400)
#define JESD204B_GT_STATUS		  (1 <<	16)
#define JESD204B_GT_STATUS_PLL_LOCKED	  0xFF
#define JESD204B_GT_STATUS_RST_DONE	  0xFFFF
#define JESD204B_GT_STATUS_SYNC		  0x1FFFF
#define JESD204B_GT_RST_DONE(x)		  (((x)	& 0xFF)	<< 8)
#define JESD204B_GT_TO_RST_DONE(x)	  (((x)	>> 8) &	0xFF)
#define JESD204B_GT_PLL_LOCKED(x)	  (((x)	& 0xFF)	<< 0)
#define JESD204B_GT_TO_PLL_LOCKED(x)	  (((x)	>> 0) &	0xFF)

#define JESD204B_GT_REG_USER_READY(c)	  (0x038 + (c) * 0x400)
#define JESD204B_GT_USER_READY		  (1 <<	0)

#define JESD204B_GT_REG_PLL_RSTN(c)	  (0x03c + (c) * 0x400)
#define JESD204B_GT_PLL_RSTN		  (1 <<	0)

#define JESD204B_GT_REG_LANESEL(c)     (0x008C + (c) * 0x400)
#define JESD204B_GT_LANESEL(x)	       (((x) & 0xFF) <<	0)
#define JESD204B_GT_TO_LANESEL(x)      (((x) >>	0) & 0xFF)

#define JESD204B_GT_REG_DRP_CNTRL(c)	  (0x0090 + (c)	* 0x400)
#define JESD204B_GT_DRP_RWN		  (1 <<	28)
#define JESD204B_GT_DRP_ADDRESS(x)	  (((x)	& 0xFFF) << 16)
#define JESD204B_GT_TO_DRP_ADDRESS(x)	  (((x)	>> 16) & 0xFFF)
#define JESD204B_GT_DRP_WDATA(x)	  (((x)	& 0xFFFF) << 0)
#define JESD204B_GT_TO_DRP_WDATA(x)	  (((x)	>> 0) &	0xFFFF)

#define JESD204B_GT_REG_DRP_STATUS(c)	  (0x0094 + (c)	* 0x400)
#define JESD204B_GT_DRP_STATUS		  (1 <<	16)
#define JESD204B_GT_DRP_RDATA(x)	  (((x)	& 0xFFFF) << 0)
#define JESD204B_GT_TO_DRP_RDATA(x)	  (((x)	>> 0) &	0xFFFF)

#define JESD204B_GT_REG_EYESCAN_CNTRL(c)  (0x00A0 + (c)	* 0x400)
#define JESD204B_GT_EYESCAN_INIT	  (1 <<	2)
#define JESD204B_GT_EYESCAN_STOP	  (1 <<	1)
#define JESD204B_GT_EYESCAN_START	  (1 <<	0)

#define JESD204B_GT_REG_EYESCAN_PRESCALE(c)  (0x00A4 + (c) * 0x400)
#define JESD204B_GT_EYESCAN_PRESCALE(x)	   (((x) & 0x1F) << 0)
#define JESD204B_GT_TO_EYESCAN_PRESCALE(x)  (((x) >> 0)	& 0x1F)

#define JESD204B_GT_REG_EYESCAN_VOFFSET(c)    (0x00A8 +	(c) * 0x400)
#define JESD204B_GT_EYESCAN_VOFFSET_STEP(x)    (((x) & 0xFF) <<	16)
#define JESD204B_GT_TO_EYESCAN_VOFFSET_STEP(x)	(((x) >> 16) & 0xFF)
#define JESD204B_GT_EYESCAN_VOFFSET_MAX(x)    (((x) & 0xFF) << 8)
#define JESD204B_GT_TO_EYESCAN_VOFFSET_MAX(x)  (((x) >>	8) & 0xFF)
#define JESD204B_GT_EYESCAN_VOFFSET_MIN(x)    (((x) & 0xFF) << 0)
#define JESD204B_GT_TO_EYESCAN_VOFFSET_MIN(x)  (((x) >>	0) & 0xFF)

#define JESD204B_GT_REG_EYESCAN_HOFFSET_1(c)	(0x00AC + (c) * 0x400)
#define JESD204B_GT_EYESCAN_HOFFSET_MAX(x)    (((x) & 0xFFF) <<	16)
#define JESD204B_GT_TO_EYESCAN_HOFFSET_MAX(x)  (((x) >>	16) & 0xFFF)
#define JESD204B_GT_EYESCAN_HOFFSET_MIN(x)    (((x) & 0xFFF) <<	0)
#define JESD204B_GT_TO_EYESCAN_HOFFSET_MIN(x)  (((x) >>	0) & 0xFFF)

#define JESD204B_GT_REG_EYESCAN_HOFFSET_2(c)  (0x00B0 +	(c) * 0x400)
#define JESD204B_GT_EYESCAN_HOFFSET_STEP(x)    (((x) & 0xFFF) << 0)
#define JESD204B_GT_TO_EYESCAN_HOFFSET_STEP(x)	(((x) >> 0) & 0xFFF)

#define JESD204B_GT_REG_EYESCAN_DMA_STARTADDR(c)  (0x00B4 + (c)	* 0x400)
#define JESD204B_GT_EYESCAN_DMA_STARTADDR(x)	(((x) &	0xFFFFFFFF) << 0)
#define JESD204B_GT_TO_EYESCAN_DMA_STARTADDR(x)	   (((x) >> 0) & 0xFFFFFFFF)

#define JESD204B_GT_REG_EYESCAN_SDATA_1_0(c)  (0x00B8 +	(c) * 0x400)
#define JESD204B_GT_EYESCAN_SDATA1(x)	   (((x) & 0xFFFF) << 16)
#define JESD204B_GT_TO_EYESCAN_SDATA1(x)    (((x) >> 16) & 0xFFFF)
#define JESD204B_GT_EYESCAN_SDATA0(x)	   (((x) & 0xFFFF) << 0)
#define JESD204B_GT_TO_EYESCAN_SDATA0(x)    (((x) >> 0)	& 0xFFFF)

#define JESD204B_GT_REG_EYESCAN_SDATA_3_2(c)  (0x00BC +	(c) * 0x400)
#define JESD204B_GT_EYESCAN_SDATA3(x)	   (((x) & 0xFFFF) << 16)
#define JESD204B_GT_TO_EYESCAN_SDATA3(x)    (((x) >> 16) & 0xFFFF)
#define JESD204B_GT_EYESCAN_SDATA2(x)	   (((x) & 0xFFFF) << 0)
#define JESD204B_GT_TO_EYESCAN_SDATA2(x)    (((x) >> 0)	& 0xFFFF)

#define JESD204B_GT_REG_EYESCAN_SDATA_4(c)    (0x00C0 +	(c) * 0x400)
#define JESD204B_GT_EYESCAN_SDATA4(x)	   (((x) & 0xFFFF) << 0)
#define JESD204B_GT_TO_EYESCAN_SDATA4(x)    (((x) >> 0)	& 0xFFFF)

#define JESD204B_GT_REG_EYESCAN_QDATA_1_0(c)  (0x00C4 +	(c) * 0x400)
#define JESD204B_GT_EYESCAN_QDATA1(x)	   (((x) & 0xFFFF) << 16)
#define JESD204B_GT_TO_EYESCAN_QDATA1(x)    (((x) >> 16) & 0xFFFF)
#define JESD204B_GT_EYESCAN_QDATA0(x)	   (((x) & 0xFFFF) << 0)
#define JESD204B_GT_TO_EYESCAN_QDATA0(x)    (((x) >> 0)	& 0xFFFF)

#define JESD204B_GT_REG_EYESCAN_QDATA_3_2(c)  (0x00C8 +	(c) * 0x400)
#define JESD204B_GT_EYESCAN_QDATA3(x)	   (((x) & 0xFFFF) << 16)
#define JESD204B_GT_TO_EYESCAN_QDATA3(x)    (((x) >> 16) & 0xFFFF)
#define JESD204B_GT_EYESCAN_QDATA2(x)	   (((x) & 0xFFFF) << 0)
#define JESD204B_GT_TO_EYESCAN_QDATA2(x)    (((x) >> 0)	& 0xFFFF)

#define JESD204B_GT_REG_EYESCAN_QDATA_4(c)    (0x00CC +	(c) * 0x400)
#define JESD204B_GT_EYESCAN_QDATA4(x)	   (((x) & 0xFFFF) << 0)
#define JESD204B_GT_TO_EYESCAN_QDATA4(x)    (((x) >> 0)	& 0xFFFF)

#define JESD204B_GT_REG_EYESCAN_STATUS(c)  (0x00E0 + (c) * 0x400)
#define JESD204B_GT_EYESCAN_DMAERR	(1 << 1)
#define JESD204B_GT_EYESCAN_STATUS	(1 << 0)

#define JESD204B_GT_REG_EYESCAN_RATE(c)	   (0x00E4 + (c) * 0x400)
#define JESD204B_GT_EYESCAN_RATE      (1 << 1)

#define JESD204B_GT_RX		  0
#define JESD204B_GT_TX		  1
#define JESD204B_GT_CPLL	  0
#define JESD204B_GT_QPLL	  1
#define JESD204B_GT_DFE		  0
#define JESD204B_GT_LPM		  1
#define JESD204B_GT_SYSREF_INT	  0
#define JESD204B_GT_SYSREF_EXT	  1

#define CPLL	0x0000
#define QPLL	0x3FC0

#define ENC_8B10B		810
#define SCAMBLED_PRBS		0

#define JESD204B_GT_REG_TRANSCEIVER_TYPE(l)	(0x00E8	+ (l) *	0x400)
#define JESD204B_GT_TRANSCEIVER_GTH		0x1 /* Ultra-Scale */
#define JESD204B_GT_TRANSCEIVER_GTX		0x0 /* 7-Series	*/

#define JESD204B_GT_REG_TX_OFFSET	  	0x0040

#define TXOUT_DIV_ADDR					0x88
#define TXOUT_DIV_MASK					0x70
#define TXOUT_DIV_OFFSET				0x4
#define TXOUT_DIV_WIDTH					0x3
#define TXOUT_DIV_DEFAULT				0x0

#define RXOUT_DIV_ADDR					0x88
#define RXOUT_DIV_MASK					0x7
#define RXOUT_DIV_OFFSET				0x0
#define RXOUT_DIV_WIDTH					0x3
#define RXOUT_DIV_DEFAULT				0x0

#define RXCDR_CFG0_ADDR					0xa8
#define RXCDR_CFG0_MASK					0xffff
#define RXCDR_CFG0_OFFSET				0x0
#define RXCDR_CFG0_WIDTH				0x10
#define RXCDR_CFG0_DEFAULT				0x0

#define RXCDR_CFG1_ADDR					0xa9
#define RXCDR_CFG1_MASK					0xffff
#define RXCDR_CFG1_OFFSET				0x0
#define RXCDR_CFG1_WIDTH				0x10
#define RXCDR_CFG1_DEFAULT				0x0

#define RXCDR_CFG2_ADDR					0xaa
#define RXCDR_CFG2_MASK					0xffff
#define RXCDR_CFG2_OFFSET				0x0
#define RXCDR_CFG2_WIDTH				0x10
#define RXCDR_CFG2_DEFAULT				0x0

#define RXCDR_CFG3_ADDR					0xab
#define RXCDR_CFG3_MASK					0xffff
#define RXCDR_CFG3_OFFSET				0x0
#define RXCDR_CFG3_WIDTH				0x10
#define RXCDR_CFG3_DEFAULT				0x0

#define RXCDR_CFG4_ADDR					0xac
#define RXCDR_CFG4_MASK					0xff
#define RXCDR_CFG4_OFFSET				0x0
#define RXCDR_CFG4_WIDTH				0x8
#define RXCDR_CFG4_DEFAULT				0x0

#define RX_DFE_LPM_CFG_ADDR				0x29
#define RX_DFE_LPM_CFG_MASK				0xffff
#define RX_DFE_LPM_CFG_OFFSET				0x0
#define RX_DFE_LPM_CFG_WIDTH				0x10
#define RX_DFE_LPM_CFG_DEFAULT				0x0

#define QPLL_CFG0_ADDR					0x32
#define QPLL_CFG0_MASK					0xffff
#define QPLL_CFG0_BAND_MASK				0x40
#define QPLL_CFG0_OFFSET				0x0
#define QPLL_CFG0_WIDTH					0x10
#define QPLL_CFG0_DEFAULT				0x0

#define QPLL_CFG1_ADDR					0x33
#define QPLL_CFG1_MASK					0x7ff
#define QPLL_CFG1_OFFSET				0x0
#define QPLL_CFG1_WIDTH					0xb
#define QPLL_CFG1_DEFAULT				0x0

#define QPLL_REFCLK_DIV_M_ADDR				0x33
#define QPLL_REFCLK_DIV_M_MASK				0xf800
#define QPLL_REFCLK_DIV_M_OFFSET			0xb
#define QPLL_REFCLK_DIV_M_WIDTH				0x5
#define QPLL_REFCLK_DIV_M_DEFAULT			0x0

#define QPLL_FBDIV_N_ADDR				0x36
#define QPLL_FBDIV_N_MASK				0x3ff
#define QPLL_FBDIV_N_OFFSET				0x0
#define QPLL_FBDIV_N_WIDTH				0xa
#define QPLL_FBDIV_N_DEFAULT				0x0

#define QPLL_FBDIV_RATIO_ADDR				0x37
#define QPLL_FBDIV_RATIO_MASK				0x40
#define QPLL_FBDIV_RATIO_OFFSET				0x6
#define QPLL_FBDIV_RATIO_WIDTH				0x1
#define QPLL_FBDIV_RATIO_DEFAULT			0x0

#define CPLL_CFG0_ADDR					0x5c
#define CPLL_CFG0_MASK					0xff00
#define CPLL_CFG0_OFFSET				0x8
#define CPLL_CFG0_WIDTH					0x8
#define CPLL_CFG0_DEFAULT				0x0

#define CPLL_CFG1_ADDR					0x5d
#define CPLL_CFG1_MASK					0xffff
#define CPLL_CFG1_OFFSET				0x0
#define CPLL_CFG1_WIDTH					0x10
#define CPLL_CFG1_DEFAULT				0x0

#define CPLL_REFCLK_DIV_M_ADDR				0x5e
#define CPLL_REFCLK_DIV_M_MASK				0x1f00
#define CPLL_REFCLK_DIV_M_OFFSET			0x8
#define CPLL_REFCLK_DIV_M_WIDTH				0x5
#define CPLL_REFCLK_DIV_M_DEFAULT			0x0

#define CPLL_FB_DIV_45_N1_ADDR				0x5e
#define CPLL_FB_DIV_45_N1_MASK				0x80
#define CPLL_FB_DIV_45_N1_OFFSET			0x7
#define CPLL_FB_DIV_45_N1_WIDTH				0x1
#define CPLL_FB_DIV_45_N1_DEFAULT			0x0

#define CPLL_FBDIV_N2_ADDR				0x5e
#define CPLL_FBDIV_N2_MASK				0x7f
#define CPLL_FBDIV_N2_OFFSET				0x0
#define CPLL_FBDIV_N2_WIDTH				0x7
#define CPLL_FBDIV_N2_DEFAULT				0x0

#endif /* ADI_JESD204B_GT_H_ */
