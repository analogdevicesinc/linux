/*
 * ADI AXI-JESD204B Interface Module
 *
 * Copyright 2012-2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * http://wiki.analog.com/resources/fpga/xilinx/
 */

#ifndef ADI_JESD204B_H_
#define ADI_JESD204B_H_

/* PCORE CoreFPGA register map */
/* Thu Aug 29 12:34:20 2013 */
#define ADI_REG_VERSION		0x0000				/*Version and Scratch Registers */
#define ADI_VERSION(x)		(((x) & 0xffffffff) << 0)	/* RO, Version number. */
#define VERSION_IS(x,y,z)	((x) << 16 | (y) << 8 | (z))
#define ADI_REG_ID		0x0004			 	/*Version and Scratch Registers */
#define ADI_ID(x)		(((x) & 0xffffffff) << 0)   	/* RO, Instance identifier number. */
#define ADI_REG_SCRATCH		0x0008			 	/*Version and Scratch Registers */
#define ADI_SCRATCH(x)		(((x) & 0xffffffff) << 0)	/* RW, Scratch register. */

/* JESD */

#define AXI_JESD204B_REG_RSTN			0x0040
#define AXI_JESD204B_DRP_RSTN			(1 << 3)
#define AXI_JESD204B_RSTN			(1 << 2)
#define AXI_JESD204B_IP_RSTN			(1 << 1)
#define AXI_JESD204B_GT_RSTN			(1 << 0)

#define AXI_JESD204B_REG_SYSREF			0x0044
#define AXI_JESD204B_IP_SYSREF			(1 << 1)
#define AXI_JESD204B_SYSREF			(1 << 0)

#define AXI_JESD204B_REG_SYNC			0x0048
#define AXI_JESD204B_SYNC			(1 << 0)

#define AXI_JESD204B_REG_RX_CNTRL_1		0x0050
#define AXI_JESD204B_RX_LANESYNC_ENB		(1 << 18)
#define AXI_JESD204B_RX_DESCR_ENB		(1 << 17)
#define AXI_JESD204B_RX_SYSREF_ENB		(1 << 16)
#define AXI_JESD204B_RX_MFRM_FRMCNT(x)		(((x) & 0xFF) << 8)
#define AXI_JESD204B_TO_RX_MFRM_FRMCNT(x)	(((x) >> 8) & 0xFF)
#define AXI_JESD204B_RX_FRM_BYTECNT(x)		(((x) & 0xFF) << 0)
#define AXI_JESD204B_TO_RX_FRM_BYTECNT(x)	(((x) >> 0) & 0xFF)

#define AXI_JESD204B_REG_RX_CNTRL_2		0x0054
#define AXI_JESD204B_RX_ERRRPT_DISB		(1 << 20)
#define AXI_JESD204B_RX_TESTMODE(x)		(((x) & 0xF) << 16)
#define AXI_JESD204B_TO_RX_TESTMODE(x)		(((x) >> 16) & 0xF)
#define AXI_JESD204B_RX_BUFDELAY(x)		(((x) & 0xFFFF) << 0)
#define AXI_JESD204B_TO_RX_BUFDELAY(x)		(((x) >> 0) & 0xFFFF)

#define AXI_JESD204B_REG_RX_LANESEL		0x005C
#define AXI_JESD204B_RX_LANESEL(x)		(((x) & 0xFF) << 0)
#define AXI_JESD204B_TO_RX_LANESEL(x)		(((x) >> 0) & 0xFF)

#define AXI_JESD204B_REG_RX_STATUS		0x0060
#define AXI_JESD204B_RX_STATUS			(1 << 0)

#define AXI_JESD204B_REG_RX_INIT_DATA_0		0x0064
#define AXI_JESD204B_RX_INIT_DATA_0(x)		(((x) & 0xFFFFFFFF) << 0)
#define AXI_JESD204B_TO_RX_INIT_DATA_0(x)	(((x) >> 0) & 0xFFFFFFFF)

#define AXI_JESD204B_REG_RX_INIT_DATA_1		0x0068
#define AXI_JESD204B_RX_INIT_DATA_1(x)		(((x) & 0xFFFFFFFF) << 0)
#define AXI_JESD204B_TO_RX_INIT_DATA_1(x)	(((x) >> 0) & 0xFFFFFFFF)

#define AXI_JESD204B_REG_RX_INIT_DATA_2		0x006C
#define AXI_JESD204B_RX_INIT_DATA_2(x)		(((x) & 0xFFFFFFFF) << 0)
#define AXI_JESD204B_TO_RX_INIT_DATA_2(x)	(((x) >> 0) & 0xFFFFFFFF)

#define AXI_JESD204B_REG_RX_INIT_DATA_3		0x0070
#define AXI_JESD204B_RX_INIT_DATA_3(x)		(((x) & 0xFFFFFFFF) << 0)
#define AXI_JESD204B_TO_RX_INIT_DATA_3(x)	(((x) >> 0) & 0xFFFFFFFF)

#define AXI_JESD204B_REG_RX_BUFCNT		0x0074
#define AXI_JESD204B_RX_BUFCNT(x)		(((x) & 0xFF) << 0)
#define AXI_JESD204B_TO_RX_BUFCNT(x)		(((x) >> 0) & 0xFF)

#define AXI_JESD204B_REG_RX_TEST_MFCNT		0x0078
#define AXI_JESD204B_RX_TEST_MFCNT(x)		(((x) & 0xFFFFFFFF) << 0)
#define AXI_JESD204B_TO_RX_TEST_MFCNT(x)	(((x) >> 0) & 0xFFFFFFFF)

#define AXI_JESD204B_REG_RX_TEST_ILACNT		0x007C
#define AXI_JESD204B_RX_TEST_ILACNT(x)		(((x) & 0xFFFFFFFF) << 0)
#define AXI_JESD204B_TO_RX_TEST_ILACNT(x)	(((x) >> 0) & 0xFFFFFFFF)

#define AXI_JESD204B_REG_RX_TEST_ERRCNT		0x0080
#define AXI_JESD204B_RX_TEST_ERRCNT(x)		(((x) & 0xFFFFFFFF) << 0)
#define AXI_JESD204B_TO_RX_TEST_ERRCNT(x)	(((x) >> 0) & 0xFFFFFFFF)

#define AXI_JESD204B_REG_DRP_CNTRL		0x0090
#define AXI_JESD204B_DRP_SEL			(1 << 29)
#define AXI_JESD204B_DRP_RWN			(1 << 28)
#define AXI_JESD204B_DRP_ADDRESS(x)		(((x) & 0xFFF) << 16)
#define AXI_JESD204B_TO_DRP_ADDRESS(x)		(((x) >> 16) & 0xFFF)
#define AXI_JESD204B_DRP_WDATA(x)		(((x) & 0xFFFF) << 0)
#define AXI_JESD204B_TO_DRP_WDATA(x)		(((x) >> 0) & 0xFFFF)

#define AXI_JESD204B_REG_DRP_STATUS		0x0094
#define AXI_JESD204B_DRP_STATUS			(1 << 16)
#define AXI_JESD204B_DRP_RDATA(x)		(((x) & 0xFFFF) << 0)
#define AXI_JESD204B_TO_DRP_RDATA(x)		(((x) >> 0) & 0xFFFF)

#define AXI_JESD204B_REG_EYESCAN_CNTRL		0x00A0
#define AXI_JESD204B_EYESCAN_INIT		(1 << 2)
#define AXI_JESD204B_EYESCAN_STOP		(1 << 1)
#define AXI_JESD204B_EYESCAN_START		(1 << 0)

#define AXI_JESD204B_REG_EYESCAN_PRESCALE	0x00A4
#define AXI_JESD204B_EYESCAN_PRESCALE(x)	(((x) & 0x1F) << 0)
#define AXI_JESD204B_TO_EYESCAN_PRESCALE(x)	(((x) >> 0) & 0x1F)

#define AXI_JESD204B_REG_EYESCAN_VOFFSET	0x00A8
#define AXI_JESD204B_EYESCAN_VOFFSET_STEP(x)	(((x) & 0xFF) << 16)
#define AXI_JESD204B_TO_EYESCAN_VOFFSET_STEP(x)	(((x) >> 16) & 0xFF)
#define AXI_JESD204B_EYESCAN_VOFFSET_MAX(x)	(((x) & 0xFF) << 8)
#define AXI_JESD204B_TO_EYESCAN_VOFFSET_MAX(x)	(((x) >> 8) & 0xFF)
#define AXI_JESD204B_EYESCAN_VOFFSET_MIN(x)	(((x) & 0xFF) << 0)
#define AXI_JESD204B_TO_EYESCAN_VOFFSET_MIN(x)	(((x) >> 0) & 0xFF)

#define AXI_JESD204B_REG_EYESCAN_HOFFSET_1	0x00AC
#define AXI_JESD204B_EYESCAN_HOFFSET_MAX(x)	(((x) & 0xFFF) << 16)
#define AXI_JESD204B_TO_EYESCAN_HOFFSET_MAX(x)	(((x) >> 16) & 0xFFF)
#define AXI_JESD204B_EYESCAN_HOFFSET_MIN(x)	(((x) & 0xFFF) << 0)
#define AXI_JESD204B_TO_EYESCAN_HOFFSET_MIN(x)	(((x) >> 0) & 0xFFF)

#define AXI_JESD204B_REG_EYESCAN_HOFFSET_2	0x00B0
#define AXI_JESD204B_EYESCAN_HOFFSET_STEP(x)	(((x) & 0xFFF) << 0)
#define AXI_JESD204B_TO_EYESCAN_HOFFSET_STEP(x)	(((x) >> 0) & 0xFFF)

#define AXI_JESD204B_REG_EYESCAN_DMA_STARTADDR	0x00B4
#define AXI_JESD204B_EYESCAN_DMA_STARTADDR(x)	(((x) & 0xFFFFFFFF) << 0)
#define AXI_JESD204B_TO_EYESCAN_DMA_STARTADDR(x)(((x) >> 0) & 0xFFFFFFFF)

#define AXI_JESD204B_REG_EYESCAN_SDATA_1_0	0x00B8
#define AXI_JESD204B_EYESCAN_SDATA1(x)		(((x) & 0xFFFF) << 16)
#define AXI_JESD204B_TO_EYESCAN_SDATA1(x)	(((x) >> 16) & 0xFFFF)
#define AXI_JESD204B_EYESCAN_SDATA0(x)		(((x) & 0xFFFF) << 0)
#define AXI_JESD204B_TO_EYESCAN_SDATA0(x)	(((x) >> 0) & 0xFFFF)

#define AXI_JESD204B_REG_EYESCAN_SDATA_3_2	0x00BC
#define AXI_JESD204B_EYESCAN_SDATA3(x)		(((x) & 0xFFFF) << 16)
#define AXI_JESD204B_TO_EYESCAN_SDATA3(x)	(((x) >> 16) & 0xFFFF)
#define AXI_JESD204B_EYESCAN_SDATA2(x)		(((x) & 0xFFFF) << 0)
#define AXI_JESD204B_TO_EYESCAN_SDATA2(x)	(((x) >> 0) & 0xFFFF)

#define AXI_JESD204B_REG_EYESCAN_SDATA_4	0x00C0
#define AXI_JESD204B_EYESCAN_SDATA4(x)		(((x) & 0xFFFF) << 0)
#define AXI_JESD204B_TO_EYESCAN_SDATA4(x)	(((x) >> 0) & 0xFFFF)

#define AXI_JESD204B_REG_EYESCAN_QDATA_1_0	0x00C4
#define AXI_JESD204B_EYESCAN_QDATA1(x)		(((x) & 0xFFFF) << 16)
#define AXI_JESD204B_TO_EYESCAN_QDATA1(x)	(((x) >> 16) & 0xFFFF)
#define AXI_JESD204B_EYESCAN_QDATA0(x)		(((x) & 0xFFFF) << 0)
#define AXI_JESD204B_TO_EYESCAN_QDATA0(x)	(((x) >> 0) & 0xFFFF)

#define AXI_JESD204B_REG_EYESCAN_QDATA_3_2	0x00C8
#define AXI_JESD204B_EYESCAN_QDATA3(x)		(((x) & 0xFFFF) << 16)
#define AXI_JESD204B_TO_EYESCAN_QDATA3(x)	(((x) >> 16) & 0xFFFF)
#define AXI_JESD204B_EYESCAN_QDATA2(x)		(((x) & 0xFFFF) << 0)
#define AXI_JESD204B_TO_EYESCAN_QDATA2(x)	(((x) >> 0) & 0xFFFF)

#define AXI_JESD204B_REG_EYESCAN_QDATA_4	0x00CC
#define AXI_JESD204B_EYESCAN_QDATA4(x)		(((x) & 0xFFFF) << 0)
#define AXI_JESD204B_TO_EYESCAN_QDATA4(x)	(((x) >> 0) & 0xFFFF)

#define AXI_JESD204B_REG_EYESCAN_STATUS		0x00E0
#define AXI_JESD204B_EYESCAN_DMAERR		(1 << 1)
#define AXI_JESD204B_EYESCAN_STATUS		(1 << 0)

#define AXI_JESD204B_REG_EYESCAN_RATE 		0x00E4

/* AXI_JESD204B_REG_INIT_DATA0 */
#define AXI_JESD204B_INIT0_DID(x)	(((x) >> 0) & 0xFF) /* DID Device ID */
#define AXI_JESD204B_INIT0_BID(x)	(((x) >> 8) & 0xFF) /* BID Bank ID */
#define AXI_JESD204B_INIT0_LID(x)	(((x) >> 13) & 0x1F) /* LID Lane ID */
#define AXI_JESD204B_INIT0_L(x)		((((x) >> 18) & 0x1F) + 1) /* Number of Lanes per Device*/
#define AXI_JESD204B_INIT0_SCR(x)	(((x) >> 23) & 0x1) /* SCR Scrambling Enabled */
#define AXI_JESD204B_INIT0_F(x)		((((x) >> 24) & 0xFF) + 1) /* Octets per Frame */

/* AXI_JESD204B_REG_INIT_DATA1 */

#define AXI_JESD204B_INIT1_K(x)		((((x) >> 0) & 0x1F) + 1) /* Frames per Multiframe */
#define AXI_JESD204B_INIT1_M(x)		((((x) >> 5) & 0xFF) + 1) /* Converters per Device */
#define AXI_JESD204B_INIT1_N(x)		((((x) >> 13) & 0x1F) + 1) /* Converter Resolution */
#define AXI_JESD204B_INIT1_CS(x)		(((x) >> 18) & 0x3) /* Control Bits per Sample */
#define AXI_JESD204B_INIT1_ND(x)		((((x) >> 20) & 0x1F) + 1) /* Total Bits per Sample */
#define AXI_JESD204B_INIT1_S(x)		((((x) >> 25) & 0x1F) + 1) /* Samples per Converter per Frame Cycle */
#define AXI_JESD204B_INIT1_HD(x)		(((x) >> 30) & 0x1) /* High Density Format */

/* AXI_JESD204B_REG_INIT_DATA2 */

#define AXI_JESD204B_INIT2_FCHK(x)	(((x) >> 16) & 0xFF) /* Checksum */
#define AXI_JESD204B_INIT2_CF(x)		(((x) >> 24) & 0x1F) /* Control Words per Frame Cycle per Link */

/* AXI_JESD204B_REG_INIT_DATA3 */

#define AXI_JESD204B_INIT3_ADJCNT(x)		(((x) >> 0) & 0xF) /* ADJCNT Adjustment step count */
#define AXI_JESD204B_INIT3_PHYADJ(x)		(((x) >> 4) & 0x1) /* PHYADJ Adjustment request */
#define AXI_JESD204B_INIT3_ADJDIR(x)		(((x) >> 5) & 0x1) /* ADJDIR Adjustment direction */
#define AXI_JESD204B_INIT3_JESDV(x)		(((x) >> 6) & 0x7) /* JESD204 Version */
#define AXI_JESD204B_INIT3_SUBCLASSV(x)		(((x) >> 9) & 0x7) /* JESD204 subclass version */

#endif /* ADI_JESD204B_H_ */
