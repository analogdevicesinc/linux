/*
 * Freescale FlexSPI driver.
 *
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/completion.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/mutex.h>
#include <linux/pm_qos.h>
#include <linux/pci.h>
#include <soc/imx8/sc/sci.h>

/* The registers */
#define FLEXSPI_MCR0			0x00
#define FLEXSPI_MCR0_AHB_TIMEOUT_SHIFT	24
#define FLEXSPI_MCR0_AHB_TIMEOUT_MASK	(0xFF << FLEXSPI_MCR0_AHB_TIMEOUT_SHIFT)
#define FLEXSPI_MCR0_IP_TIMEOUT_SHIFT	16
#define FLEXSPI_MCR0_IP_TIMEOUT_MASK	(0xFF << FLEXSPI_MCR0_IP_TIMEOUT_SHIFT)
#define FLEXSPI_MCR0_LEARN_EN_SHIFT	15
#define FLEXSPI_MCR0_LEARN_EN_MASK	(1 << FLEXSPI_MCR0_LEARN_EN_SHIFT)
#define FLEXSPI_MCR0_SCRFRUN_EN_SHIFT	14
#define FLEXSPI_MCR0_SCRFRUN_EN_MASK	(1 << FLEXSPI_MCR0_SCRFRUN_EN_SHIFT)
#define FLEXSPI_MCR0_OCTCOMB_EN_SHIFT	13
#define FLEXSPI_MCR0_OCTCOMB_EN_MASK	(1 << FLEXSPI_MCR0_OCTCOMB_EN_SHIFT)
#define FLEXSPI_MCR0_DOZE_EN_SHIFT	12
#define FLEXSPI_MCR0_DOZE_EN_MASK	(1 << FLEXSPI_MCR0_DOZE_EN_SHIFT)
#define FLEXSPI_MCR0_HSEN_SHIFT		11
#define FLEXSPI_MCR0_HSEN_MASK		(1 << FLEXSPI_MCR0_HSEN_SHIFT)
#define FLEXSPI_MCR0_SERCLKDIV_SHIFT	8
#define FLEXSPI_MCR0_SERCLKDIV_MASK	(7 << FLEXSPI_MCR0_SERCLKDIV_SHIFT)
#define FLEXSPI_MCR0_ATDF_EN_SHIFT	7
#define FLEXSPI_MCR0_ATDF_EN_MASK	(1 << FLEXSPI_MCR0_ATDF_EN_SHIFT)
#define FLEXSPI_MCR0_ARDF_EN_SHIFT	6
#define FLEXSPI_MCR0_ARDF_EN_MASK	(1 << FLEXSPI_MCR0_ARDF_EN_SHIFT)
#define FLEXSPI_MCR0_RXCLKSRC_SHIFT	4
#define FLEXSPI_MCR0_RXCLKSRC_MASK	(3 << FLEXSPI_MCR0_RXCLKSRC_SHIFT)
#define FLEXSPI_MCR0_END_CFG_SHIFT	2
#define FLEXSPI_MCR0_END_CFG_MASK	(3 << FLEXSPI_MCR0_END_CFG_SHIFT)
#define FLEXSPI_MCR0_MDIS_SHIFT		1
#define FLEXSPI_MCR0_MDIS_MASK		(1 << FLEXSPI_MCR0_MDIS_SHIFT)
#define FLEXSPI_MCR0_SWRST_SHIFT	0
#define FLEXSPI_MCR0_SWRST_MASK		(1 << FLEXSPI_MCR0_SWRST_SHIFT)

#define FLEXSPI_MCR1			0x04
#define FLEXSPI_MCR1_SEQ_TIMEOUT_SHIFT	16
#define FLEXSPI_MCR1_SEQ_TIMEOUT_MASK	\
	(0xFFFF << FLEXSPI_MCR1_SEQ_TIMEOUT_SHIFT)
#define FLEXSPI_MCR1_AHB_TIMEOUT_SHIFT	0
#define FLEXSPI_MCR1_AHB_TIMEOUT_MASK	\
	(0xFFFF << FLEXSPI_MCR1_AHB_TIMEOUT_SHIFT)

#define FLEXSPI_MCR2			0x08
#define FLEXSPI_MCR2_IDLE_WAIT_SHIFT	24
#define FLEXSPI_MCR2_IDLE_WAIT_MASK	(0xFF << FLEXSPI_MCR2_IDLE_WAIT_SHIFT)
#define FLEXSPI_MCR2_SAMEFLASH_SHIFT	15
#define FLEXSPI_MCR2_SAMEFLASH_MASK	(1 << FLEXSPI_MCR2_SAMEFLASH_SHIFT)
#define FLEXSPI_MCR2_CLRLRPHS_SHIFT	14
#define FLEXSPI_MCR2_CLRLRPHS_MASK	(1 << FLEXSPI_MCR2_CLRLRPHS_SHIFT)
#define FLEXSPI_MCR2_ABRDATSZ_SHIFT	8
#define FLEXSPI_MCR2_ABRDATSZ_MASK	(1 << FLEXSPI_MCR2_ABRDATSZ_SHIFT)
#define FLEXSPI_MCR2_ABRLEARN_SHIFT	7
#define FLEXSPI_MCR2_ABRLEARN_MASK	(1 << FLEXSPI_MCR2_ABRLEARN_SHIFT)
#define FLEXSPI_MCR2_ABR_READ_SHIFT	6
#define FLEXSPI_MCR2_ABR_READ_MASK	(1 << FLEXSPI_MCR2_ABR_READ_SHIFT)
#define FLEXSPI_MCR2_ABRWRITE_SHIFT	5
#define FLEXSPI_MCR2_ABRWRITE_MASK	(1 << FLEXSPI_MCR2_ABRWRITE_SHIFT)
#define FLEXSPI_MCR2_ABRDUMMY_SHIFT	4
#define FLEXSPI_MCR2_ABRDUMMY_MASK	(1 << FLEXSPI_MCR2_ABRDUMMY_SHIFT)
#define FLEXSPI_MCR2_ABR_MODE_SHIFT	3
#define FLEXSPI_MCR2_ABR_MODE_MASK	(1 << FLEXSPI_MCR2_ABR_MODE_SHIFT)
#define FLEXSPI_MCR2_ABRCADDR_SHIFT	2
#define FLEXSPI_MCR2_ABRCADDR_MASK	(1 << FLEXSPI_MCR2_ABRCADDR_SHIFT)
#define FLEXSPI_MCR2_ABRRADDR_SHIFT	1
#define FLEXSPI_MCR2_ABRRADDR_MASK	(1 << FLEXSPI_MCR2_ABRRADDR_SHIFT)
#define FLEXSPI_MCR2_ABR_CMD_SHIFT	0
#define FLEXSPI_MCR2_ABR_CMD_MASK	(1 << FLEXSPI_MCR2_ABR_CMD_SHIFT)

#define FLEXSPI_AHBCR			0x0c
#define FLEXSPI_AHBCR_PREF_EN_SHIFT	5
#define FLEXSPI_AHBCR_PREF_EN_MASK	(1 << FLEXSPI_AHBCR_PREF_EN_SHIFT)
#define FLEXSPI_AHBCR_BUFF_EN_SHIFT	4
#define FLEXSPI_AHBCR_BUFF_EN_MASK	(1 << FLEXSPI_AHBCR_BUFF_EN_SHIFT)
#define FLEXSPI_AHBCR_CACH_EN_SHIFT	3
#define FLEXSPI_AHBCR_CACH_EN_MASK	(1 << FLEXSPI_AHBCR_CACH_EN_SHIFT)
#define FLEXSPI_AHBCR_CLRTXBUF_SHIFT	2
#define FLEXSPI_AHBCR_CLRTXBUF_MASK	(1 << FLEXSPI_AHBCR_CLRTXBUF_SHIFT)
#define FLEXSPI_AHBCR_CLRRXBUF_SHIFT	1
#define FLEXSPI_AHBCR_CLRRXBUF_MASK	(1 << FLEXSPI_AHBCR_CLRRXBUF_SHIFT)
#define FLEXSPI_AHBCR_PAR_EN_SHIFT	0
#define FLEXSPI_AHBCR_PAR_EN_MASK	(1 << FLEXSPI_AHBCR_PAR_EN_SHIFT)

#define FLEXSPI_INTEN			0x10
#define FLEXSPI_INTEN_SCLKSBWR_SHIFT	9
#define FLEXSPI_INTEN_SCLKSBWR_MASK	(1 << FLEXSPI_INTEN_SCLKSBWR_SHIFT)
#define FLEXSPI_INTEN_SCLKSBRD_SHIFT	8
#define FLEXSPI_INTEN_SCLKSBRD_MASK	(1 << FLEXSPI_INTEN_SCLKSBRD_SHIFT)
#define FLEXSPI_INTEN_DATALRNFL_SHIFT	7
#define FLEXSPI_INTEN_DATALRNFL_MASK	(1 << FLEXSPI_INTEN_DATALRNFL_SHIFT)
#define FLEXSPI_INTEN_IPTXWE_SHIFT	6
#define FLEXSPI_INTEN_IPTXWE_MASK	(1 << FLEXSPI_INTEN_IPTXWE_SHIFT)
#define FLEXSPI_INTEN_IPRXWA_SHIFT	5
#define FLEXSPI_INTEN_IPRXWA_MASK	(1 << FLEXSPI_INTEN_IPRXWA_SHIFT)
#define FLEXSPI_INTEN_AHBCMDERR_SHIFT	4
#define FLEXSPI_INTEN_AHBCMDERR_MASK	(1 << FLEXSPI_INTEN_AHBCMDERR_SHIFT)
#define FLEXSPI_INTEN_IPCMDERR_SHIFT	3
#define FLEXSPI_INTEN_IPCMDERR_MASK	(1 << FLEXSPI_INTEN_IPCMDERR_SHIFT)
#define FLEXSPI_INTEN_AHBCMDGE_SHIFT	2
#define FLEXSPI_INTEN_AHBCMDGE_MASK	(1 << FLEXSPI_INTEN_AHBCMDGE_SHIFT)
#define FLEXSPI_INTEN_IPCMDGE_SHIFT	1
#define FLEXSPI_INTEN_IPCMDGE_MASK	(1 << FLEXSPI_INTEN_IPCMDGE_SHIFT)
#define FLEXSPI_INTEN_IPCMDDONE_SHIFT	0
#define FLEXSPI_INTEN_IPCMDDONE_MASK	(1 << FLEXSPI_INTEN_IPCMDDONE_SHIFT)

#define FLEXSPI_INTR			0x14
#define FLEXSPI_INTR_SCLKSBWR_SHIFT	9
#define FLEXSPI_INTR_SCLKSBWR_MASK	(1 << FLEXSPI_INTR_SCLKSBWR_SHIFT)
#define FLEXSPI_INTR_SCLKSBRD_SHIFT	8
#define FLEXSPI_INTR_SCLKSBRD_MASK	(1 << FLEXSPI_INTR_SCLKSBRD_SHIFT)
#define FLEXSPI_INTR_DATALRNFL_SHIFT	7
#define FLEXSPI_INTR_DATALRNFL_MASK	(1 << FLEXSPI_INTR_DATALRNFL_SHIFT)
#define FLEXSPI_INTR_IPTXWE_SHIFT	6
#define FLEXSPI_INTR_IPTXWE_MASK	(1 << FLEXSPI_INTR_IPTXWE_SHIFT)
#define FLEXSPI_INTR_IPRXWA_SHIFT	5
#define FLEXSPI_INTR_IPRXWA_MASK	(1 << FLEXSPI_INTR_IPRXWA_SHIFT)
#define FLEXSPI_INTR_AHBCMDERR_SHIFT	4
#define FLEXSPI_INTR_AHBCMDERR_MASK	(1 << FLEXSPI_INTR_AHBCMDERR_SHIFT)
#define FLEXSPI_INTR_IPCMDERR_SHIFT	3
#define FLEXSPI_INTR_IPCMDERR_MASK	(1 << FLEXSPI_INTR_IPCMDERR_SHIFT)
#define FLEXSPI_INTR_AHBCMDGE_SHIFT	2
#define FLEXSPI_INTR_AHBCMDGE_MASK	(1 << FLEXSPI_INTR_AHBCMDGE_SHIFT)
#define FLEXSPI_INTR_IPCMDGE_SHIFT	1
#define FLEXSPI_INTR_IPCMDGE_MASK	(1 << FLEXSPI_INTR_IPCMDGE_SHIFT)
#define FLEXSPI_INTR_IPCMDDONE_SHIFT	0
#define FLEXSPI_INTR_IPCMDDONE_MASK	(1 << FLEXSPI_INTR_IPCMDDONE_SHIFT)

#define FLEXSPI_LUTKEY			0x18
#define FLEXSPI_LUTKEY_VALUE		0x5AF05AF0

#define FLEXSPI_LCKCR			0x1C
#define FLEXSPI_LCKER_LOCK		0x1
#define FLEXSPI_LCKER_UNLOCK		0x2

#define FLEXSPI_BUFXCR_INVALID_MSTRID	0xe
#define FLEXSPI_AHBRX_BUF0CR0		0x20
#define FLEXSPI_AHBRX_BUF1CR0		0x24
#define FLEXSPI_AHBRX_BUF2CR0		0x28
#define FLEXSPI_AHBRX_BUF3CR0		0x2C
#define FLEXSPI_AHBRX_BUF4CR0		0x30
#define FLEXSPI_AHBRX_BUF5CR0		0x34
#define FLEXSPI_AHBRX_BUF6CR0		0x38
#define FLEXSPI_AHBRX_BUF7CR0		0x3C
#define FLEXSPI_AHBRXBUF0CR7_PREF_SHIFT	31
#define FLEXSPI_AHBRXBUF0CR7_PREF_MASK	(1 << FLEXSPI_AHBRXBUF0CR7_PREF_SHIFT)

#define FLEXSPI_AHBRX_BUF0CR1		0x40
#define FLEXSPI_AHBRX_BUF1CR1		0x44
#define FLEXSPI_AHBRX_BUF2CR1		0x48
#define FLEXSPI_AHBRX_BUF3CR1		0x4C
#define FLEXSPI_AHBRX_BUF4CR1		0x50
#define FLEXSPI_AHBRX_BUF5CR1		0x54
#define FLEXSPI_AHBRX_BUF6CR1		0x58
#define FLEXSPI_AHBRX_BUF7CR1		0x5C
#define FLEXSPI_BUFXCR1_MSID_SHIFT	0
#define FLEXSPI_BUFXCR1_MSID_MASK	(0xF << FLEXSPI_BUFXCR1_MSID_SHIFT)
#define FLEXSPI_BUFXCR1_PRIO_SHIFT	8
#define FLEXSPI_BUFXCR1_PRIO_MASK	(0x7 << FLEXSPI_BUFXCR1_PRIO_SHIFT)

#define FLEXSPI_FLSHA1CR0		0x60
#define FLEXSPI_FLSHA2CR0		0x64
#define FLEXSPI_FLSHB1CR0		0x68
#define FLEXSPI_FLSHB2CR0		0x6C
#define FLEXSPI_FLSHXCR0_SZ_SHIFT	10
#define FLEXSPI_FLSHXCR0_SZ_MASK	(0x3FFFFF << FLEXSPI_FLSHXCR0_SZ_SHIFT)

#define FLEXSPI_FLSHA1CR1		0x70
#define FLEXSPI_FLSHA2CR1		0x74
#define FLEXSPI_FLSHB1CR1		0x78
#define FLEXSPI_FLSHB2CR1		0x7C
#define FLEXSPI_FLSHXCR1_CSINTR_SHIFT	16
#define FLEXSPI_FLSHXCR1_CSINTR_MASK	\
	(0xFFFF << FLEXSPI_FLSHXCR1_CSINTR_SHIFT)
#define FLEXSPI_FLSHXCR1_CAS_SHIFT	11
#define FLEXSPI_FLSHXCR1_CAS_MASK	(0xF << FLEXSPI_FLSHXCR1_CAS_SHIFT)
#define FLEXSPI_FLSHXCR1_WA_SHIFT	10
#define FLEXSPI_FLSHXCR1_WA_MASK	(1 << FLEXSPI_FLSHXCR1_WA_SHIFT)
#define FLEXSPI_FLSHXCR1_TCSH_SHIFT	5
#define FLEXSPI_FLSHXCR1_TCSH_MASK	(0x1F << FLEXSPI_FLSHXCR1_TCSH_SHIFT)
#define FLEXSPI_FLSHXCR1_TCSS_SHIFT	0
#define FLEXSPI_FLSHXCR1_TCSS_MASK	(0x1F << FLEXSPI_FLSHXCR1_TCSS_SHIFT)

#define FLEXSPI_FLSHA1CR2		0x80
#define FLEXSPI_FLSHA2CR2		0x84
#define FLEXSPI_FLSHB1CR2		0x88
#define FLEXSPI_FLSHB2CR2		0x8C
#define FLEXSPI_FLSHXCR2_CLRINSP_SHIFT	24
#define FLEXSPI_FLSHXCR2_CLRINSP_MASK	(1 << FLEXSPI_FLSHXCR2_CLRINSP_SHIFT)
#define FLEXSPI_FLSHXCR2_AWRWAIT_SHIFT	16
#define FLEXSPI_FLSHXCR2_AWRWAIT_MASK	(0xFF << FLEXSPI_FLSHXCR2_AWRWAIT_SHIFT)
#define FLEXSPI_FLSHXCR2_AWRSEQN_SHIFT	13
#define FLEXSPI_FLSHXCR2_AWRSEQN_MASK	(0x7 << FLEXSPI_FLSHXCR2_AWRSEQN_SHIFT)
#define FLEXSPI_FLSHXCR2_AWRSEQI_SHIFT	8
#define FLEXSPI_FLSHXCR2_AWRSEQI_MASK	(0xF << FLEXSPI_FLSHXCR2_AWRSEQI_SHIFT)
#define FLEXSPI_FLSHXCR2_ARDSEQN_SHIFT	5
#define FLEXSPI_FLSHXCR2_ARDSEQN_MASK	(0x7 << FLEXSPI_FLSHXCR2_ARDSEQN_SHIFT)
#define FLEXSPI_FLSHXCR2_ARDSEQI_SHIFT	0
#define FLEXSPI_FLSHXCR2_ARDSEQI_MASK	(0xF << FLEXSPI_FLSHXCR2_ARDSEQI_SHIFT)

#define FLEXSPI_IPCR0			0xA0

#define FLEXSPI_IPCR1			0xA4
#define FLEXSPI_IPCR1_IPAREN_SHIFT	31
#define FLEXSPI_IPCR1_IPAREN_MASK	(1 << FLEXSPI_IPCR1_IPAREN_SHIFT)
#define FLEXSPI_IPCR1_SEQNUM_SHIFT	24
#define FLEXSPI_IPCR1_SEQNUM_MASK	(0xF << FLEXSPI_IPCR1_SEQNUM_SHIFT)
#define FLEXSPI_IPCR1_SEQID_SHIFT	16
#define FLEXSPI_IPCR1_SEQID_MASK	(0xF << FLEXSPI_IPCR1_SEQID_SHIFT)
#define FLEXSPI_IPCR1_IDATSZ_SHIFT	0
#define FLEXSPI_IPCR1_IDATSZ_MASK	(0xFFFF << FLEXSPI_IPCR1_IDATSZ_SHIFT)

#define FLEXSPI_IPCMD			0xB0
#define FLEXSPI_IPCMD_TRG_SHIFT		0
#define FLEXSPI_IPCMD_TRG_MASK		(1 << FLEXSPI_IPCMD_TRG_SHIFT)

#define FLEXSPI_DLPR			0xB4

#define FLEXSPI_IPRXFCR			0xB8
#define FLEXSPI_IPRXFCR_CLR_SHIFT	0
#define FLEXSPI_IPRXFCR_CLR_MASK	(1 << FLEXSPI_IPRXFCR_CLR_SHIFT)
#define FLEXSPI_IPRXFCR_DMA_EN_SHIFT	1
#define FLEXSPI_IPRXFCR_DMA_EN_MASK	(1 << FLEXSPI_IPRXFCR_DMA_EN_SHIFT)
#define FLEXSPI_IPRXFCR_WMRK_SHIFT	2
#define FLEXSPI_IPRXFCR_WMRK_MASK	(0x1F << FLEXSPI_IPRXFCR_WMRK_SHIFT)

#define FLEXSPI_IPTXFCR			0xBC
#define FLEXSPI_IPTXFCR_CLR_SHIFT	0
#define FLEXSPI_IPTXFCR_CLR_MASK	(1 << FLEXSPI_IPTXFCR_CLR_SHIFT)
#define FLEXSPI_IPTXFCR_DMA_EN_SHIFT	1
#define FLEXSPI_IPTXFCR_DMA_EN_MASK	(1 << FLEXSPI_IPTXFCR_DMA_EN_SHIFT)
#define FLEXSPI_IPTXFCR_WMRK_SHIFT	2
#define FLEXSPI_IPTXFCR_WMRK_MASK	(0x1F << FLEXSPI_IPTXFCR_WMRK_SHIFT)

#define FLEXSPI_STS0			0xE0
#define FLEXSPI_STS0_DLPHA_SHIFT	9
#define FLEXSPI_STS0_DLPHA_MASK		(0x1F << FLEXSPI_STS0_DLPHA_SHIFT)
#define FLEXSPI_STS0_DLPHB_SHIFT	4
#define FLEXSPI_STS0_DLPHB_MASK		(0x1F << FLEXSPI_STS0_DLPHB_SHIFT)
#define FLEXSPI_STS0_CMD_SRC_SHIFT	2
#define FLEXSPI_STS0_CMD_SRC_MASK	(3 << FLEXSPI_STS0_CMD_SRC_SHIFT)
#define FLEXSPI_STS0_ARB_IDLE_SHIFT	1
#define FLEXSPI_STS0_ARB_IDLE_MASK	(1 << FLEXSPI_STS0_ARB_IDLE_SHIFT)
#define FLEXSPI_STS0_SEQ_IDLE_SHIFT	0
#define FLEXSPI_STS0_SEQ_IDLE_MASK	(1 << FLEXSPI_STS0_SEQ_IDLE_SHIFT)

#define FLEXSPI_STS1			0xE4
#define FLEXSPI_STS1_IP_ERRCD_SHIFT	24
#define FLEXSPI_STS1_IP_ERRCD_MASK	(0xF << FLEXSPI_STS1_IP_ERRCD_SHIFT)
#define FLEXSPI_STS1_IP_ERRID_SHIFT	16
#define FLEXSPI_STS1_IP_ERRID_MASK	(0xF << FLEXSPI_STS1_IP_ERRID_SHIFT)
#define FLEXSPI_STS1_AHB_ERRCD_SHIFT	8
#define FLEXSPI_STS1_AHB_ERRCD_MASK	(0xF << FLEXSPI_STS1_AHB_ERRCD_SHIFT)
#define FLEXSPI_STS1_AHB_ERRID_SHIFT	0
#define FLEXSPI_STS1_AHB_ERRID_MASK	(0xF << FLEXSPI_STS1_AHB_ERRID_SHIFT)

#define FLEXSPI_AHBSPNST		0xEC
#define FLEXSPI_AHBSPNST_DATLFT_SHIFT	16
#define FLEXSPI_AHBSPNST_DATLFT_MASK	\
	(0xFFFF << FLEXSPI_AHBSPNST_DATLFT_SHIFT)
#define FLEXSPI_AHBSPNST_BUFID_SHIFT	1
#define FLEXSPI_AHBSPNST_BUFID_MASK	(7 << FLEXSPI_AHBSPNST_BUFID_SHIFT)
#define FLEXSPI_AHBSPNST_ACTIVE_SHIFT	0
#define FLEXSPI_AHBSPNST_ACTIVE_MASK	(1 << FLEXSPI_AHBSPNST_ACTIVE_SHIFT)

#define FLEXSPI_IPRXFSTS		0xF0
#define FLEXSPI_IPRXFSTS_RDCNTR_SHIFT	16
#define FLEXSPI_IPRXFSTS_RDCNTR_MASK	\
	(0xFFFF << FLEXSPI_IPRXFSTS_RDCNTR_SHIFT)
#define FLEXSPI_IPRXFSTS_FILL_SHIFT	0
#define FLEXSPI_IPRXFSTS_FILL_MASK	(0xFF << FLEXSPI_IPRXFSTS_FILL_SHIFT)

#define FLEXSPI_IPTXFSTS		0xF4
#define FLEXSPI_IPTXFSTS_WRCNTR_SHIFT	16
#define FLEXSPI_IPTXFSTS_WRCNTR_MASK	\
	(0xFFFF << FLEXSPI_IPTXFSTS_WRCNTR_SHIFT)
#define FLEXSPI_IPTXFSTS_FILL_SHIFT	0
#define FLEXSPI_IPTXFSTS_FILL_MASK	(0xFF << FLEXSPI_IPTXFSTS_FILL_SHIFT)

#define FLEXSPI_RFDR			0x100
#define FLEXSPI_TFDR			0x180

#define FLEXSPI_LUT_BASE		0x200

/* register map end */

/*
 * The definition of the LUT register shows below:
 *
 *  ---------------------------------------------------
 *  | INSTR1 | PAD1 | OPRND1 | INSTR0 | PAD0 | OPRND0 |
 *  ---------------------------------------------------
 */
#define OPRND0_SHIFT		0
#define PAD0_SHIFT		8
#define INSTR0_SHIFT		10
#define OPRND1_SHIFT		16

/* Instruction set for the LUT register. */

#define LUT_STOP		0x00
#define LUT_CMD		0x01
#define LUT_ADDR		0x02
#define LUT_CADDR_SDR		0x03
#define LUT_MODE		0x04
#define LUT_MODE2		0x05
#define LUT_MODE4		0x06
#define LUT_MODE8		0x07
#define LUT_FSL_WRITE		0x08
#define LUT_FSL_READ		0x09
#define LUT_LEARN_SDR		0x0A
#define LUT_DATSZ_SDR		0x0B
#define LUT_DUMMY		0x0C
#define LUT_DUMMY_RWDS_SDR	0x0D
#define LUT_JMP_ON_CS		0x1F
#define LUT_CMD_DDR		0x21
#define LUT_ADDR_DDR		0x22
#define LUT_CADDR_DDR		0x23
#define LUT_MODE_DDR		0x24
#define LUT_MODE2_DDR		0x25
#define LUT_MODE4_DDR		0x26
#define LUT_MODE8_DDR		0x27
#define LUT_WRITE_DDR		0x28
#define LUT_READ_DDR		0x29
#define LUT_LEARN_DDR		0x2A
#define LUT_DATSZ_DDR		0x2B
#define LUT_DUMMY_DDR		0x2C
#define LUT_DUMMY_RWDS_DDR	0x2D


/*
 * The PAD definitions for LUT register.
 *
 * The pad stands for the lines number of IO[0:3].
 * For example, the Quad read need four IO lines, so you should
 * set LUT_PAD4 which means we use four IO lines.
 */
#define LUT_PAD1		0
#define LUT_PAD2		1
#define LUT_PAD4		2
#define LUT_PAD8		3

/* Oprands for the LUT register. */
#define ADDR24BIT		0x18
#define ADDR32BIT		0x20

/* Macros for constructing the LUT register. */
#define LUT0(ins, pad, opr)						\
		(((opr) << OPRND0_SHIFT) | ((LUT_##pad) << PAD0_SHIFT) | \
		((LUT_##ins) << INSTR0_SHIFT))

#define LUT1(ins, pad, opr)	(LUT0(ins, pad, opr) << OPRND1_SHIFT)

/* other macros for LUT register. */
#define FLEXSPI_LUT(x)          (FLEXSPI_LUT_BASE + (x) * 4)
#define FLEXSPI_LUT_NUM		64

/* SEQID -- we can have 16 seqids at most. */
#define SEQID_QUAD_READ		0
#define SEQID_WREN		1
#define SEQID_WRDI		2
#define SEQID_RDSR		3
#define SEQID_SE		4
#define SEQID_CHIP_ERASE	5
#define SEQID_PP		6
#define SEQID_RDID		7
#define SEQID_WRSR		8
#define SEQID_RDCR		9
#define SEQID_EN4B		10
#define SEQID_BRWR		11
#define SEQID_RD_EVCR		12
#define SEQID_WD_EVCR		13

#define FLEXSPI_MIN_IOMAP	SZ_4M

enum fsl_flexspi_devtype {
	FSL_FLEXSPI_IMX8QM,
	FSL_FLEXSPI_IMX8QXP,
};

struct fsl_flexspi_devtype_data {
	enum fsl_flexspi_devtype devtype;
	int rxfifo;
	int txfifo;
	int ahb_buf_size;
	int driver_data;
};

static struct fsl_flexspi_devtype_data imx8qm_data = {
	.devtype = FSL_FLEXSPI_IMX8QM,
	/* .rxfifo = 1024, */
	.rxfifo = 128,
	.txfifo = 1024,
	.ahb_buf_size = 2048,
	.driver_data = 0,
};

static struct fsl_flexspi_devtype_data imx8qxp_data = {
	.devtype = FSL_FLEXSPI_IMX8QXP,
	/* .rxfifo = 1024, */
	.rxfifo = 128,
	.txfifo = 1024,
	.ahb_buf_size = 2048,
	.driver_data = 0,
};

#define FSL_FLEXSPI_MAX_CHIP	4
struct fsl_flexspi {
	struct mtd_info mtd[FSL_FLEXSPI_MAX_CHIP];
	struct spi_nor nor[FSL_FLEXSPI_MAX_CHIP];
	void __iomem *iobase;
	void __iomem *ahb_addr;
	u32 memmap_phy;
	u32 memmap_offs;
	u32 memmap_len;
	struct clk *clk, *clk_en;
	struct device *dev;
	struct completion c;
	struct fsl_flexspi_devtype_data *devtype_data;
	u32 nor_size;
	u32 nor_num;
	u32 clk_rate;
	unsigned int chip_base_addr; /* We may support two chips. */
	bool has_second_chip;
	u32 ddr_smp;
	struct mutex lock;
	struct pm_qos_request pm_qos_req;
};

static inline void fsl_flexspi_unlock_lut(struct fsl_flexspi *flex)
{
	writel(FLEXSPI_LUTKEY_VALUE, flex->iobase + FLEXSPI_LUTKEY);
	writel(FLEXSPI_LCKER_UNLOCK, flex->iobase + FLEXSPI_LCKCR);
}

static inline void fsl_flexspi_lock_lut(struct fsl_flexspi *flex)
{
	writel(FLEXSPI_LUTKEY_VALUE, flex->iobase + FLEXSPI_LUTKEY);
	writel(FLEXSPI_LCKER_LOCK, flex->iobase + FLEXSPI_LCKCR);
}

static irqreturn_t fsl_flexspi_irq_handler(int irq, void *dev_id)
{
	struct fsl_flexspi *flex = dev_id;
	u32 reg;

	reg = readl(flex->iobase + FLEXSPI_INTR);
	writel(FLEXSPI_INTR_IPCMDDONE_MASK, flex->iobase + FLEXSPI_INTR);
	if (reg & FLEXSPI_INTR_IPCMDDONE_MASK)
		complete(&flex->c);

	return IRQ_HANDLED;
}

static void fsl_flexspi_init_lut(struct fsl_flexspi *flex)
{
	void __iomem *base = flex->iobase;
	int rxfifo = flex->devtype_data->rxfifo;
	struct spi_nor *nor = &flex->nor[0];
	u8 addrlen = (nor->addr_width == 3) ? ADDR24BIT : ADDR32BIT;
	u32 lut_base;
	u8 op, dm;
	int i;

	fsl_flexspi_unlock_lut(flex);

	/* Clear all the LUT table */
	for (i = 0; i < FLEXSPI_LUT_NUM; i++)
		writel(0, base + FLEXSPI_LUT_BASE + i * 4);

	/* Quad Read and DDR Quad Read*/
	lut_base = SEQID_QUAD_READ * 4;
	op = nor->read_opcode;
	dm = nor->read_dummy;

	/* Octal DDR read */
	if (op == SPINOR_OP_READ_1_1_8_D) {
		writel(LUT0(CMD, PAD1, op) |
		       LUT1(ADDR_DDR, PAD1, addrlen),
		       base + FLEXSPI_LUT(lut_base));

		writel(LUT0(DUMMY_DDR, PAD8, dm * 2)
			| LUT1(READ_DDR, PAD8, rxfifo),
			base + FLEXSPI_LUT(lut_base + 1));

	}

	if (nor->flash_read == SPI_NOR_QUAD) {
		if (op == SPINOR_OP_READ_1_1_4 || op == SPINOR_OP_READ4_1_1_4) {
			/* read mode : 1-1-4 */
			writel(LUT0(CMD, PAD1, op) | LUT1(ADDR, PAD1, addrlen),
				base + FLEXSPI_LUT(lut_base));

			writel(LUT0(DUMMY, PAD1, dm) |
			       LUT1(FSL_READ, PAD4, rxfifo),
			       base + FLEXSPI_LUT(lut_base + 1));
		} else {
			dev_err(nor->dev, "Unsupported opcode : 0x%.2x\n", op);
		}
	} else if (nor->flash_read == SPI_NOR_DDR_QUAD) {
		if (op == SPINOR_OP_READ_1_4_4_D ||
			 op == SPINOR_OP_READ4_1_4_4_D) {
			/* read mode : 1-4-4, such as Spansion s25fl128s. */
			writel(LUT0(CMD_DDR, PAD1, op)
				| LUT1(ADDR_DDR, PAD4, addrlen),
				base + FLEXSPI_LUT(lut_base));

			writel(LUT0(MODE_DDR, PAD4, 0xff)
				| LUT1(DUMMY, PAD1, dm),
				base + FLEXSPI_LUT(lut_base + 1));

			writel(LUT0(READ_DDR, PAD4, rxfifo)
				| LUT1(JMP_ON_CS, PAD1, 0),
				base + FLEXSPI_LUT(lut_base + 2));
		} else if (op == SPINOR_OP_READ_1_1_4_D) {
			/* read mode : 1-1-4, such as Micron N25Q256A. */
			writel(LUT0(CMD_DDR, PAD1, op)
				| LUT1(ADDR_DDR, PAD1, addrlen),
				base + FLEXSPI_LUT(lut_base));

			writel(LUT0(DUMMY, PAD1, dm)
				| LUT1(READ_DDR, PAD4, rxfifo),
				base + FLEXSPI_LUT(lut_base + 1));

			writel(LUT0(JMP_ON_CS, PAD1, 0),
				base + FLEXSPI_LUT(lut_base + 2));
		} else {
			dev_err(nor->dev, "Unsupported opcode : 0x%.2x\n", op);
		}
	}

	/* Write enable */
	lut_base = SEQID_WREN * 4;
	writel(LUT0(CMD, PAD1, SPINOR_OP_WREN), base + FLEXSPI_LUT(lut_base));

	/* Page Program */
	lut_base = SEQID_PP * 4;
	writel(LUT0(CMD, PAD1, nor->program_opcode) | LUT1(ADDR, PAD1, addrlen),
			base + FLEXSPI_LUT(lut_base));
	writel(LUT0(FSL_WRITE, PAD1, 0), base + FLEXSPI_LUT(lut_base + 1));

	/* Read Status */
	lut_base = SEQID_RDSR * 4;
	writel(LUT0(CMD, PAD1, SPINOR_OP_RDSR) | LUT1(FSL_READ, PAD1, 0x1),
			base + FLEXSPI_LUT(lut_base));

	/* Erase a sector */
	lut_base = SEQID_SE * 4;
	writel(LUT0(CMD, PAD1, nor->erase_opcode) | LUT1(ADDR, PAD1, addrlen),
			base + FLEXSPI_LUT(lut_base));

	/* Erase the whole chip */
	lut_base = SEQID_CHIP_ERASE * 4;
	writel(LUT0(CMD, PAD1, SPINOR_OP_CHIP_ERASE),
			base + FLEXSPI_LUT(lut_base));

	/* READ ID */
	lut_base = SEQID_RDID * 4;
	writel(LUT0(CMD, PAD1, SPINOR_OP_RDID) | LUT1(FSL_READ, PAD1, 0x8),
			base + FLEXSPI_LUT(lut_base));

	/* Write Register */
	lut_base = SEQID_WRSR * 4;
	writel(LUT0(CMD, PAD1, SPINOR_OP_WRSR) | LUT1(FSL_WRITE, PAD1, 0x2),
			base + FLEXSPI_LUT(lut_base));

	/* Read Configuration Register */
	lut_base = SEQID_RDCR * 4;
	writel(LUT0(CMD, PAD1, SPINOR_OP_RDCR) | LUT1(FSL_READ, PAD1, 0x1),
			base + FLEXSPI_LUT(lut_base));

	/* Write disable */
	lut_base = SEQID_WRDI * 4;
	writel(LUT0(CMD, PAD1, SPINOR_OP_WRDI), base + FLEXSPI_LUT(lut_base));

	/* Enter 4 Byte Mode (Micron) */
	lut_base = SEQID_EN4B * 4;
	writel(LUT0(CMD, PAD1, SPINOR_OP_EN4B), base + FLEXSPI_LUT(lut_base));

	/* Enter 4 Byte Mode (Spansion) */
	lut_base = SEQID_BRWR * 4;
	writel(LUT0(CMD, PAD1, SPINOR_OP_BRWR), base + FLEXSPI_LUT(lut_base));

	/* Read EVCR register */
	lut_base = SEQID_RD_EVCR * 4;
	writel(LUT0(CMD, PAD1, SPINOR_OP_RD_EVCR),
	       base + FLEXSPI_LUT(lut_base));

	/* Write EVCR register */
	lut_base = SEQID_WD_EVCR * 4;
	writel(LUT0(CMD, PAD1, SPINOR_OP_WD_EVCR),
	       base + FLEXSPI_LUT(lut_base));
	fsl_flexspi_lock_lut(flex);
}

/* Get the SEQID for the command */
static int fsl_flexspi_get_seqid(struct fsl_flexspi *flex, u8 cmd)
{

	switch (cmd) {
	case SPINOR_OP_READ_1_1_4_D:
	case SPINOR_OP_READ_1_1_8_D:
	case SPINOR_OP_READ_1_4_4_D:
	case SPINOR_OP_READ4_1_4_4_D:
	case SPINOR_OP_READ4_1_1_4:
	case SPINOR_OP_READ_1_1_4:
	case SPINOR_OP_READ4:
		return SEQID_QUAD_READ;
	case SPINOR_OP_WREN:
		return SEQID_WREN;
	case SPINOR_OP_WRDI:
		return SEQID_WRDI;
	case SPINOR_OP_RDSR:
		return SEQID_RDSR;
	case SPINOR_OP_BE_4K:
	case SPINOR_OP_SE:
		return SEQID_SE;
	case SPINOR_OP_CHIP_ERASE:
		return SEQID_CHIP_ERASE;
	case SPINOR_OP_PP:
		return SEQID_PP;
	case SPINOR_OP_RDID:
		return SEQID_RDID;
	case SPINOR_OP_WRSR:
		return SEQID_WRSR;
	case SPINOR_OP_RDCR:
		return SEQID_RDCR;
	case SPINOR_OP_EN4B:
		return SEQID_EN4B;
	case SPINOR_OP_BRWR:
		return SEQID_BRWR;
	case SPINOR_OP_RD_EVCR:
		return SEQID_RD_EVCR;
	case SPINOR_OP_WD_EVCR:
		return SEQID_WD_EVCR;
	default:
		dev_err(flex->dev, "Unsupported cmd 0x%.2x\n", cmd);
		break;
	}
	return -EINVAL;
}

static int
fsl_flexspi_runcmd(struct fsl_flexspi *flex, u8 cmd, unsigned int addr, int len)
{
	void __iomem *base = flex->iobase;
	int seqid;
	int seqnum = 0;
	u32 reg;
	int err;

	init_completion(&flex->c);
	dev_dbg(flex->dev, "to 0x%.8x:0x%.8x, len:%d, cmd:%.2x\n",
			flex->chip_base_addr, addr, len, cmd);

	/* write address */
	writel(flex->chip_base_addr + addr, base + FLEXSPI_IPCR0);

	seqid = fsl_flexspi_get_seqid(flex, cmd);

	writel((seqnum << FLEXSPI_IPCR1_SEQNUM_SHIFT) |
			(seqid << FLEXSPI_IPCR1_SEQID_SHIFT) | len,
			base + FLEXSPI_IPCR1);

	/* wait till controller is idle */
	do {
		reg = readl(base + FLEXSPI_STS0);
		if ((reg & FLEXSPI_STS0_ARB_IDLE_MASK) &&
		    (reg & FLEXSPI_STS0_SEQ_IDLE_MASK))
			break;
		udelay(1);
		dev_err(flex->dev, "The controller is busy, 0x%x\n", reg);
	} while (1);

	/* trigger the LUT now */
	writel(1, base + FLEXSPI_IPCMD);

	/* Wait for the interrupt. */
	if (!wait_for_completion_timeout(&flex->c, msecs_to_jiffies(1000))) {
		dev_dbg(flex->dev,
			"cmd 0x%.2x timeout, addr@%.8x, Status0:0x%.8x, Status1:0x%.8x\n",
			cmd, addr, readl(base + FLEXSPI_STS0),
			readl(base + FLEXSPI_STS1));
		err = -ETIMEDOUT;
	} else {
		err = 0;
	}

	return err;
}

/* Read out the data from the FLEXSPI_RBDR buffer registers. */
static void fsl_flexspi_read_data(struct fsl_flexspi *flex, int len, u8 *rxbuf)
{
	/* u64 tmp; */
	int i = 0;
	int size;

	/* invalid RXFIFO first */
	writel(FLEXSPI_IPRXFCR_CLR_MASK, flex->iobase + FLEXSPI_IPRXFCR);
	while (len > 0) {

		size = len / 8;

		for (i = 0; i < size; ++i) {
			/* Wait for RXFIFO available*/
			while (!(readl(flex->iobase + FLEXSPI_INTR)
				 & FLEXSPI_INTR_IPRXWA_MASK))
				;

			/* read 64 bit data once */
			memcpy(rxbuf, flex->iobase + FLEXSPI_RFDR, 8);
			rxbuf += 8;

			/* move the FIFO pointer */
			writel(FLEXSPI_INTR_IPRXWA_MASK,
			       flex->iobase + FLEXSPI_INTR);
			len -= 8;
		}

		size = len % 8;

		if (size) {
			/* Wait for RXFIFO available*/
			while (!(readl(flex->iobase + FLEXSPI_INTR)
				 & FLEXSPI_INTR_IPRXWA_MASK))
				;

			memcpy(rxbuf, flex->iobase + FLEXSPI_RFDR, size);
			len -= size;
		}

		writel(FLEXSPI_INTR_IPRXWA_MASK,
		       flex->iobase + FLEXSPI_INTR);

		/* invalid the RXFIFO */
		writel(FLEXSPI_IPRXFCR_CLR_MASK,
		       flex->iobase + FLEXSPI_IPRXFCR);
	}
}

/*
 * If we have changed the content of the flash by writing or erasing,
 * we need to invalidate the AHB buffer. If we do not do so, we may read out
 * the wrong data. The spec tells us reset the AHB domain and Serial Flash
 * domain at the same time.
 */
static inline void fsl_flexspi_invalid(struct fsl_flexspi *flex)
{
	u32 reg;

	reg = readl(flex->iobase + FLEXSPI_MCR0);
	writel(reg | FLEXSPI_MCR0_SWRST_MASK, flex->iobase + FLEXSPI_MCR0);

	/*
	 * The minimum delay : 1 AHB + 2 SFCK clocks.
	 * Delay 1 us is enough.
	 */
	while (readl(flex->iobase + FLEXSPI_MCR0) & FLEXSPI_MCR0_SWRST_MASK)
		;

}

static ssize_t fsl_flexspi_nor_write(struct fsl_flexspi *flex,
				     struct spi_nor *nor, u8 opcode,
				     unsigned int to, u32 *txbuf,
				     unsigned int count)
{
	int ret, i;
	int size;

	dev_dbg(flex->dev, "nor write to 0x%.8x:0x%.8x, len : %d\n",
		flex->chip_base_addr, to, count);

	/* clear the TX FIFO. */
	writel(FLEXSPI_IPTXFCR_CLR_MASK, flex->iobase + FLEXSPI_IPTXFCR);

	size = count / 8;
	for (i = 0; i < size; i++) {
		/* Wait for TXFIFO empty*/
		while (!(readl(flex->iobase + FLEXSPI_INTR)
			 & FLEXSPI_INTR_IPTXWE_MASK))
			;

		memcpy(flex->iobase + FLEXSPI_TFDR, txbuf, 8);
		txbuf += 2;
		writel(FLEXSPI_INTR_IPTXWE_MASK, flex->iobase + FLEXSPI_INTR);
	}

	size = count % 8;
	if (size) {
		/* Wait for TXFIFO empty*/
		while (!(readl(flex->iobase + FLEXSPI_INTR)
			 & FLEXSPI_INTR_IPTXWE_MASK))
			;

		memcpy(flex->iobase + FLEXSPI_TFDR, txbuf, size);
		writel(FLEXSPI_INTR_IPTXWE_MASK, flex->iobase + FLEXSPI_INTR);
	}

	/* Trigger it */
	ret = fsl_flexspi_runcmd(flex, opcode, to, count);

	if (ret == 0)
		return count;

	return ret;
}

static void fsl_flexspi_set_map_addr(struct fsl_flexspi *flex)
{
	int nor_size = flex->nor_size >> 10;
	void __iomem *base = flex->iobase;

	writel(nor_size, base + FLEXSPI_FLSHA1CR0);
	writel(nor_size * 2, base + FLEXSPI_FLSHA2CR0);
	writel(nor_size * 3, base + FLEXSPI_FLSHB1CR0);
	writel(nor_size * 4, base + FLEXSPI_FLSHB2CR0);
}

/*
 * There are two different ways to read out the data from the flash:
 *  the "IP Command Read" and the "AHB Command Read".
 *
 * The IC guy suggests we use the "AHB Command Read" which is faster
 * then the "IP Command Read". (What's more is that there is a bug in
 * the "IP Command Read" in the Vybrid.)
 *
 * After we set up the registers for the "AHB Command Read", we can use
 * the memcpy to read the data directly. A "missed" access to the buffer
 * causes the controller to clear the buffer, and use the sequence pointed
 * by the FLEXSPI_BFGENCR[SEQID] to initiate a read from the flash.
 */
static void fsl_flexspi_init_ahb_read(struct fsl_flexspi *flex)
{
	void __iomem *base = flex->iobase;
	struct spi_nor *nor = &flex->nor[0];
	/* u32 reg, reg2; */
	int seqid;
	int i;

	/* AHB configuration for access buffer 0/1/2 .*/
	for (i = 0; i < 7; i++)
		writel(0, base + FLEXSPI_AHBRX_BUF0CR0 + 4 * i);
	/*
	 * Set ADATSZ with the maximum AHB buffer size to improve the
	 * read performance.
	 */
	writel((flex->devtype_data->ahb_buf_size / 8 |
		FLEXSPI_AHBRXBUF0CR7_PREF_MASK),
	       base + FLEXSPI_AHBRX_BUF7CR0);

	writel(FLEXSPI_AHBCR_PREF_EN_MASK, base + FLEXSPI_AHBCR);

	/* Set the default lut sequence for AHB Read. */
	seqid = fsl_flexspi_get_seqid(flex, nor->read_opcode);
	writel(seqid, flex->iobase + FLEXSPI_FLSHA1CR2);
}

/* This function was used to prepare and enable QSPI clock */
static int fsl_flexspi_clk_prep_enable(struct fsl_flexspi *flex)
{
	int ret;

	ret = clk_prepare_enable(flex->clk_en);
	if (ret)
		return ret;

	ret = clk_prepare_enable(flex->clk);
	if (ret) {
		clk_disable_unprepare(flex->clk_en);
		return ret;
	}

	return 0;
}

/* This function was used to disable and unprepare QSPI clock */
static void fsl_flexspi_clk_disable_unprep(struct fsl_flexspi *flex)
{
	clk_disable_unprepare(flex->clk);
	clk_disable_unprepare(flex->clk_en);
}

/* We use this function to do some basic init for spi_nor_scan(). */
static int fsl_flexspi_nor_setup(struct fsl_flexspi *flex)
{
	void __iomem *base = flex->iobase;
	u32 reg;

	/* Reset the module */
	writel(FLEXSPI_MCR0_SWRST_MASK, base + FLEXSPI_MCR0);
	do {
		udelay(1);
	} while (0x1 & readl(base + FLEXSPI_MCR0));

	/* Disable the module */
	writel(FLEXSPI_MCR0_MDIS_MASK, base + FLEXSPI_MCR0);

	/* enable module */
	writel(FLEXSPI_MCR0_AHB_TIMEOUT_MASK | FLEXSPI_MCR0_IP_TIMEOUT_MASK |
	       FLEXSPI_MCR0_OCTCOMB_EN_MASK, base + FLEXSPI_MCR0);

	/* Read the register value */
	reg = readl(base + FLEXSPI_MCR0);

	/* Init the LUT table. */
	fsl_flexspi_init_lut(flex);

	/* enable the interrupt */
	writel(FLEXSPI_INTEN_IPCMDDONE_MASK, flex->iobase + FLEXSPI_INTEN);
	return 0;
}

static int fsl_flexspi_nor_setup_last(struct fsl_flexspi *flex)
{
	unsigned long rate = flex->clk_rate;
	int ret;

	/* disable and unprepare clock to avoid glitch pass to controller */
	fsl_flexspi_clk_disable_unprep(flex);

	ret = clk_set_rate(flex->clk, rate);
	if (ret)
		return ret;

	ret = fsl_flexspi_clk_prep_enable(flex);
	if (ret)
		return ret;

	/* Init the LUT table again. */
	fsl_flexspi_init_lut(flex);

	/* Init for AHB read */
	fsl_flexspi_init_ahb_read(flex);

	return 0;
}

static void fsl_flexspi_set_base_addr(struct fsl_flexspi *flex,
				      struct spi_nor *nor)
{
	flex->chip_base_addr = flex->nor_size * (nor - flex->nor);
}

static int fsl_flexspi_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf,
				int len)
{
	int ret;
	struct fsl_flexspi *flex = nor->priv;

	ret = fsl_flexspi_runcmd(flex, opcode, 0, len);
	if (ret)
		return ret;

	fsl_flexspi_read_data(flex, len, buf);
	return 0;
}

static int fsl_flexspi_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf,
				 int len)
{
	struct fsl_flexspi *flex = nor->priv;
	int ret;

	if (!buf) {
		ret = fsl_flexspi_runcmd(flex, opcode, 0, 1);
		if (ret)
			return ret;

		if (opcode == SPINOR_OP_CHIP_ERASE)
			fsl_flexspi_invalid(flex);

	} else if (len > 0) {
		ret = fsl_flexspi_nor_write(flex, nor, opcode, 0,
					(u32 *)buf, len);
	} else {
		dev_err(flex->dev, "invalid cmd %d\n", opcode);
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t fsl_flexspi_write(struct spi_nor *nor, loff_t to,
		size_t len, const u_char *buf)
{
	struct fsl_flexspi *flex = nor->priv;

	ssize_t ret = fsl_flexspi_nor_write(flex, nor, nor->program_opcode, to,
				(u32 *)buf, len);

	/* invalid the data in the AHB buffer. */
	fsl_flexspi_invalid(flex);
	return ret;
}

static ssize_t fsl_flexspi_read(struct spi_nor *nor, loff_t from,
		size_t len, u_char *buf)
{
	struct fsl_flexspi *flex = nor->priv;

	/* if necessary,ioremap buffer before AHB read, */
	if (!flex->ahb_addr) {
		flex->memmap_offs = flex->chip_base_addr + from;
		flex->memmap_len = len > FLEXSPI_MIN_IOMAP ?
			len : FLEXSPI_MIN_IOMAP;

		flex->ahb_addr = ioremap_nocache(
				flex->memmap_phy + flex->memmap_offs,
				flex->memmap_len);
		if (!flex->ahb_addr) {
			dev_err(flex->dev, "ioremap failed\n");
			return -ENOMEM;
		}
	/* ioremap if the data requested is out of range */
	} else if (flex->chip_base_addr + from < flex->memmap_offs
			|| flex->chip_base_addr + from + len >
			flex->memmap_offs + flex->memmap_len) {
		iounmap(flex->ahb_addr);

		flex->memmap_offs = flex->chip_base_addr + from;
		flex->memmap_len = len > FLEXSPI_MIN_IOMAP ?
			len : FLEXSPI_MIN_IOMAP;
		flex->ahb_addr = ioremap_nocache(
				flex->memmap_phy + flex->memmap_offs,
				flex->memmap_len);
		if (!flex->ahb_addr) {
			dev_err(flex->dev, "ioremap failed\n");
			return -ENOMEM;
		}
	}

	/* Read out the data directly from the AHB buffer.*/
	memcpy(buf,
	       flex->ahb_addr + flex->chip_base_addr + from - flex->memmap_offs,
	       len);

	return len;
}

static int fsl_flexspi_erase(struct spi_nor *nor, loff_t offs)
{
	struct fsl_flexspi *flex = nor->priv;
	int ret;

	dev_dbg(nor->dev, "%dKiB at 0x%08x:0x%08x\n",
		nor->mtd.erasesize / 1024, flex->chip_base_addr, (u32)offs);

	ret = fsl_flexspi_runcmd(flex, nor->erase_opcode, offs, 0);
	if (ret)
		return ret;

	fsl_flexspi_invalid(flex);
	return 0;
}

static int fsl_flexspi_prep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct fsl_flexspi *flex = nor->priv;
	int ret;

	mutex_lock(&flex->lock);

	ret = fsl_flexspi_clk_prep_enable(flex);
	if (ret)
		goto err_mutex;

	fsl_flexspi_set_base_addr(flex, nor);
	return 0;

err_mutex:
	mutex_unlock(&flex->lock);
	return ret;
}

static void fsl_flexspi_unprep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct fsl_flexspi *flex = nor->priv;

	fsl_flexspi_clk_disable_unprep(flex);
	mutex_unlock(&flex->lock);
}

static const struct of_device_id fsl_flexspi_dt_ids[] = {
	{ .compatible = "fsl,imx8qm-flexspi", .data = (void *)&imx8qm_data, },
	{ .compatible = "fsl,imx8qxp-flexspi", .data = (void *)&imx8qxp_data, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_flexspi_dt_ids);

static int fsl_flexspi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct fsl_flexspi *flex;
	struct resource *res;
	struct spi_nor *nor;
	struct mtd_info *mtd;
	int ret, i = 0;

	const struct of_device_id *of_id =
			of_match_device(fsl_flexspi_dt_ids, &pdev->dev);

	flex = devm_kzalloc(dev, sizeof(*flex), GFP_KERNEL);
	if (!flex)
		return -ENOMEM;

	flex->nor_num = of_get_child_count(dev->of_node);
	if (!flex->nor_num || flex->nor_num > 4)
		return -ENODEV;

	flex->dev = dev;
	flex->devtype_data = (struct fsl_flexspi_devtype_data *)of_id->data;
	platform_set_drvdata(pdev, flex);

	/* find the resources */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "FlexSPI");
	if (!res) {
		dev_err(dev, "FlexSPI get resource IORESOURCE_MEM failed\n");
		return -ENODEV;
	}

	flex->iobase = devm_ioremap_resource(dev, res);
	if (IS_ERR(flex->iobase))
		return PTR_ERR(flex->iobase);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					"FlexSPI-memory");
	if (!res) {
		dev_err(dev,
			"FlexSPI-memory get resource IORESOURCE_MEM failed\n");
		return -ENODEV;
	}

	if (!devm_request_mem_region(dev, res->start, resource_size(res),
				     res->name)) {
		dev_err(dev, "can't request region for resource %pR\n", res);
		return -EBUSY;
	}

	flex->memmap_phy = res->start;

	/* find the clocks */
	flex->clk_en = devm_clk_get(dev, "qspi_en");
	if (IS_ERR(flex->clk_en))
		return PTR_ERR(flex->clk_en);

	flex->clk = devm_clk_get(dev, "qspi");
	if (IS_ERR(flex->clk))
		return PTR_ERR(flex->clk);

	/* find ddrsmp value */
	ret = of_property_read_u32(dev->of_node, "ddrsmp",
				&flex->ddr_smp);
	if (ret)
		flex->ddr_smp = 0;

	ret = fsl_flexspi_clk_prep_enable(flex);
	if (ret) {
		dev_err(dev, "can not enable the clock\n");
		goto clk_failed;
	}

	/* find the irq */
	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(dev, "failed to get the irq: %d\n", ret);
		goto irq_failed;
	}

	ret = devm_request_irq(dev, ret,
			fsl_flexspi_irq_handler, 0, pdev->name, flex);
	if (ret) {
		dev_err(dev, "failed to request irq: %d\n", ret);
		goto irq_failed;
	}

	ret = fsl_flexspi_nor_setup(flex);
	if (ret)
		goto irq_failed;

	if (of_get_property(np, "fsl,qspi-has-second-chip", NULL))
		flex->has_second_chip = true;

	mutex_init(&flex->lock);

	/* iterate the subnodes. */
	for_each_available_child_of_node(dev->of_node, np) {
		enum read_mode mode = SPI_NOR_DDR_OCTAL;
		u32 dummy = 0;

		/* skip the holes */
		if (!flex->has_second_chip)
			i *= 2;

		nor = &flex->nor[i];
		mtd = &nor->mtd;

		nor->dev = dev;
		spi_nor_set_flash_node(nor, np);
		nor->priv = flex;

		/* fill the hooks */
		nor->read_reg = fsl_flexspi_read_reg;
		nor->write_reg = fsl_flexspi_write_reg;
		nor->read = fsl_flexspi_read;
		nor->write = fsl_flexspi_write;
		nor->erase = fsl_flexspi_erase;

		nor->prepare = fsl_flexspi_prep;
		nor->unprepare = fsl_flexspi_unprep;

		ret = of_property_read_u32(np, "spi-max-frequency",
				&flex->clk_rate);
		if (ret < 0)
			goto mutex_failed;

		/* Can we enable the DDR Quad Read? */
		ret = of_property_read_u32(np, "spi-nor,ddr-quad-read-dummy",
					&dummy);
		if (!ret && dummy > 0)
			mode = SPI_NOR_DDR_OCTAL;

		/* set the chip address for READID */
		fsl_flexspi_set_base_addr(flex, nor);


		ret = spi_nor_scan(nor, NULL, mode);
		if (ret)
			goto mutex_failed;

		ret = mtd_device_register(mtd, NULL, 0);
		if (ret)
			goto mutex_failed;

		/* Set the correct NOR size now. */
		if (flex->nor_size == 0) {
			flex->nor_size = mtd->size;

			/* Map the SPI NOR to accessiable address */
			fsl_flexspi_set_map_addr(flex);
		}

		/*
		 * The TX FIFO is 64 bytes in the Vybrid, but the Page Program
		 * may writes 265 bytes per time. The write is working in the
		 * unit of the TX FIFO, not in the unit of the SPI NOR's page
		 * size.
		 *
		 * So shrink the spi_nor->page_size if it is larger then the
		 * TX FIFO.
		 */
		if (nor->page_size > flex->devtype_data->txfifo)
			nor->page_size = flex->devtype_data->txfifo;

		i++;
	}

	/* finish the rest init. */
	ret = fsl_flexspi_nor_setup_last(flex);
	if (ret)
		goto last_init_failed;

	fsl_flexspi_clk_disable_unprep(flex);
	return 0;

last_init_failed:
	for (i = 0; i < flex->nor_num; i++) {
		/* skip the holes */
		if (!flex->has_second_chip)
			i *= 2;
		mtd_device_unregister(&flex->mtd[i]);
	}
mutex_failed:
	mutex_destroy(&flex->lock);
irq_failed:
	fsl_flexspi_clk_disable_unprep(flex);
clk_failed:
	dev_err(dev, "Freescale FlexSPI probe failed\n");
	return ret;
}

static int fsl_flexspi_remove(struct platform_device *pdev)
{
	struct fsl_flexspi *flex = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < flex->nor_num; i++) {
		/* skip the holes */
		if (!flex->has_second_chip)
			i *= 2;
		mtd_device_unregister(&flex->nor[i].mtd);
	}

	/* disable the hardware */
	writel(FLEXSPI_MCR0_MDIS_MASK, flex->iobase + FLEXSPI_MCR0);

	mutex_destroy(&flex->lock);

	if (flex->ahb_addr)
		iounmap(flex->ahb_addr);

	return 0;
}

static int fsl_flexspi_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int fsl_flexspi_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver fsl_flexspi_driver = {
	.driver = {
		.name	= "fsl-flexspi",
		.bus	= &platform_bus_type,
		.of_match_table = fsl_flexspi_dt_ids,
	},
	.probe          = fsl_flexspi_probe,
	.remove		= fsl_flexspi_remove,
	.suspend	= fsl_flexspi_suspend,
	.resume		= fsl_flexspi_resume,
};
module_platform_driver(fsl_flexspi_driver);


MODULE_DESCRIPTION("Freescale FlexSPI Controller Driver");
MODULE_AUTHOR("Freescale Semiconductor Inc.");
MODULE_LICENSE("GPL v2");
