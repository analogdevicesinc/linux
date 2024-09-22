// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices SPI3 controller driver
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/types.h>

#define SPI2_BASE 0x60000000
#define SPI2_SIZE 0x2000000

/* SPI_CONTROL */
#define SPI_CTL_EN                  0x00000001    /* Enable */
#define SPI_CTL_MSTR                0x00000002    /* Master/Slave */
#define SPI_CTL_PSSE                0x00000004    /* controls modf error in controller mode */
#define SPI_CTL_ODM                 0x00000008    /* Open Drain Mode */
#define SPI_CTL_CPHA                0x00000010    /* Clock Phase */
#define SPI_CTL_CPOL                0x00000020    /* Clock Polarity */
#define SPI_CTL_ASSEL               0x00000040    /* Slave Select Pin Control */
#define SPI_CTL_SELST               0x00000080    /* Slave Select Polarity in-between transfers */
#define SPI_CTL_EMISO               0x00000100    /* Enable MISO */
#define SPI_CTL_SIZE                0x00000600    /* Word Transfer Size */
#define SPI_CTL_SIZE08              0x00000000    /* SIZE: 8 bits */
#define SPI_CTL_SIZE16              0x00000200    /* SIZE: 16 bits */
#define SPI_CTL_SIZE32              0x00000400    /* SIZE: 32 bits */
#define SPI_CTL_LSBF                0x00001000    /* LSB First */
#define SPI_CTL_FCEN                0x00002000    /* Flow-Control Enable */
#define SPI_CTL_FCCH                0x00004000    /* Flow-Control Channel Selection */
#define SPI_CTL_FCPL                0x00008000    /* Flow-Control Polarity */
#define SPI_CTL_FCWM                0x00030000    /* Flow-Control Water-Mark */
#define SPI_CTL_FIFO0               0x00000000    /* FCWM: TFIFO empty or RFIFO Full */
#define SPI_CTL_FIFO1               0x00010000    /* FCWM: TFIFO >= 75% empty, RFIFO >= 75% full */
#define SPI_CTL_FIFO2               0x00020000    /* FCWM: TFIFO >= 50% empty, RFIFO >= 50% full */
#define SPI_CTL_FMODE               0x00040000    /* Fast-mode Enable */
#define SPI_CTL_MIOM                0x00300000    /* Multiple I/O Mode */
#define SPI_CTL_MIO_DIS             0x00000000    /* MIOM: Disable */
#define SPI_CTL_MIO_DUAL            0x00100000    /* MIOM: Enable DIOM (Dual I/O Mode) */
#define SPI_CTL_MIO_QUAD            0x00200000    /* MIOM: Enable QUAD (Quad SPI Mode) */
#define SPI_CTL_SOSI                0x00400000    /* Start on MOSI */
#define SPI_CTL_MMWEM		    0x40000000	  /* Memory Mapped Write Error Mask */
#define SPI_CTL_MMSE		    0x80000000	  /* Enable Memory Mapped Mode */
/* SPI_RX_CONTROL */
#define SPI_RXCTL_REN               0x00000001    /* Receive Channel Enable */
#define SPI_RXCTL_RTI               0x00000004    /* Receive Transfer Initiate */
#define SPI_RXCTL_RWCEN             0x00000008    /* Receive Word Counter Enable */
#define SPI_RXCTL_RDR               0x00000070    /* Receive Data Request */
#define SPI_RXCTL_RDR_DIS           0x00000000    /* RDR: Disabled */
#define SPI_RXCTL_RDR_NE            0x00000010    /* RDR: RFIFO not empty */
#define SPI_RXCTL_RDR_25            0x00000020    /* RDR: RFIFO 25% full */
#define SPI_RXCTL_RDR_50            0x00000030    /* RDR: RFIFO 50% full */
#define SPI_RXCTL_RDR_75            0x00000040    /* RDR: RFIFO 75% full */
#define SPI_RXCTL_RDR_FULL          0x00000050    /* RDR: RFIFO full */
#define SPI_RXCTL_RDO               0x00000100    /* Receive Data Over-Run */
#define SPI_RXCTL_RRWM              0x00003000    /* FIFO Regular Water-Mark */
#define SPI_RXCTL_RWM_0             0x00000000    /* RRWM: RFIFO Empty */
#define SPI_RXCTL_RWM_25            0x00001000    /* RRWM: RFIFO 25% full */
#define SPI_RXCTL_RWM_50            0x00002000    /* RRWM: RFIFO 50% full */
#define SPI_RXCTL_RWM_75            0x00003000    /* RRWM: RFIFO 75% full */
#define SPI_RXCTL_RUWM              0x00070000    /* FIFO Urgent Water-Mark */
#define SPI_RXCTL_UWM_DIS           0x00000000    /* RUWM: Disabled */
#define SPI_RXCTL_UWM_25            0x00010000    /* RUWM: RFIFO 25% full */
#define SPI_RXCTL_UWM_50            0x00020000    /* RUWM: RFIFO 50% full */
#define SPI_RXCTL_UWM_75            0x00030000    /* RUWM: RFIFO 75% full */
#define SPI_RXCTL_UWM_FULL          0x00040000    /* RUWM: RFIFO full */
/* SPI_TX_CONTROL */
#define SPI_TXCTL_TEN               0x00000001    /* Transmit Channel Enable */
#define SPI_TXCTL_TTI               0x00000004    /* Transmit Transfer Initiate */
#define SPI_TXCTL_TWCEN             0x00000008    /* Transmit Word Counter Enable */
#define SPI_TXCTL_TDR               0x00000070    /* Transmit Data Request */
#define SPI_TXCTL_TDR_DIS           0x00000000    /* TDR: Disabled */
#define SPI_TXCTL_TDR_NF            0x00000010    /* TDR: TFIFO not full */
#define SPI_TXCTL_TDR_25            0x00000020    /* TDR: TFIFO 25% empty */
#define SPI_TXCTL_TDR_50            0x00000030    /* TDR: TFIFO 50% empty */
#define SPI_TXCTL_TDR_75            0x00000040    /* TDR: TFIFO 75% empty */
#define SPI_TXCTL_TDR_EMPTY         0x00000050    /* TDR: TFIFO empty */
#define SPI_TXCTL_TDU               0x00000100    /* Transmit Data Under-Run */
#define SPI_TXCTL_TRWM              0x00003000    /* FIFO Regular Water-Mark */
#define SPI_TXCTL_RWM_FULL          0x00000000    /* TRWM: TFIFO full */
#define SPI_TXCTL_RWM_25            0x00001000    /* TRWM: TFIFO 25% empty */
#define SPI_TXCTL_RWM_50            0x00002000    /* TRWM: TFIFO 50% empty */
#define SPI_TXCTL_RWM_75            0x00003000    /* TRWM: TFIFO 75% empty */
#define SPI_TXCTL_TUWM              0x00070000    /* FIFO Urgent Water-Mark */
#define SPI_TXCTL_UWM_DIS           0x00000000    /* TUWM: Disabled */
#define SPI_TXCTL_UWM_25            0x00010000    /* TUWM: TFIFO 25% empty */
#define SPI_TXCTL_UWM_50            0x00020000    /* TUWM: TFIFO 50% empty */
#define SPI_TXCTL_UWM_75            0x00030000    /* TUWM: TFIFO 75% empty */
#define SPI_TXCTL_UWM_EMPTY         0x00040000    /* TUWM: TFIFO empty */
/* SPI_CLOCK */
#define SPI_CLK_BAUD                0x0000FFFF    /* Baud Rate */
/* SPI_DELAY */
#define SPI_DLY_STOP                0x000000FF    /* Transfer delay time */
#define SPI_DLY_STOP_3		    0x00000003    /* Transfer delay time */
#define SPI_DLY_LEADX               0x00000100    /* Extended (1 SCK) LEAD Control */
#define SPI_DLY_LAGX                0x00000200    /* Extended (1 SCK) LAG control */
/* SPI_SSEL */
#define SPI_SLVSEL_SSE1             0x00000002    /* SPISSEL1 Enable */
#define SPI_SLVSEL_SSE2             0x00000004    /* SPISSEL2 Enable */
#define SPI_SLVSEL_SSE3             0x00000008    /* SPISSEL3 Enable */
#define SPI_SLVSEL_SSE4             0x00000010    /* SPISSEL4 Enable */
#define SPI_SLVSEL_SSE5             0x00000020    /* SPISSEL5 Enable */
#define SPI_SLVSEL_SSE6             0x00000040    /* SPISSEL6 Enable */
#define SPI_SLVSEL_SSE7             0x00000080    /* SPISSEL7 Enable */
#define SPI_SLVSEL_SSEL1            0x00000200    /* SPISSEL1 Value */
#define SPI_SLVSEL_SSEL2            0x00000400    /* SPISSEL2 Value */
#define SPI_SLVSEL_SSEL3            0x00000800    /* SPISSEL3 Value */
#define SPI_SLVSEL_SSEL4            0x00001000    /* SPISSEL4 Value */
#define SPI_SLVSEL_SSEL5            0x00002000    /* SPISSEL5 Value */
#define SPI_SLVSEL_SSEL6            0x00004000    /* SPISSEL6 Value */
#define SPI_SLVSEL_SSEL7            0x00008000    /* SPISSEL7 Value */
/* SPI_RWC */
#define SPI_RWC_VALUE               0x0000FFFF    /* Received Word-Count */
/* SPI_RWCR */
#define SPI_RWCR_VALUE              0x0000FFFF    /* Received Word-Count Reload */
/* SPI_TWC */
#define SPI_TWC_VALUE               0x0000FFFF    /* Transmitted Word-Count */
/* SPI_TWCR */
#define SPI_TWCR_VALUE              0x0000FFFF    /* Transmitted Word-Count Reload */
/* SPI_IMASK */
#define SPI_IMSK_RUWM               0x00000002    /* Receive Urgent Water-Mark Interrupt Mask */
#define SPI_IMSK_TUWM               0x00000004    /* Transmit Urgent Water-Mark Interrupt Mask */
#define SPI_IMSK_ROM                0x00000010    /* Receive Over-Run Error Interrupt Mask */
#define SPI_IMSK_TUM                0x00000020    /* Transmit Under-Run Error Interrupt Mask */
#define SPI_IMSK_TCM                0x00000040    /* Transmit Collision Error Interrupt Mask */
#define SPI_IMSK_MFM                0x00000080    /* Mode Fault Error Interrupt Mask */
#define SPI_IMSK_RSM                0x00000100    /* Receive Start Interrupt Mask */
#define SPI_IMSK_TSM                0x00000200    /* Transmit Start Interrupt Mask */
#define SPI_IMSK_RFM                0x00000400    /* Receive Finish Interrupt Mask */
#define SPI_IMSK_TFM                0x00000800    /* Transmit Finish Interrupt Mask */
/* SPI_IMASKCL */
#define SPI_IMSK_CLR_RUW            0x00000002    /* Receive Urgent Water-Mark Interrupt Mask */
#define SPI_IMSK_CLR_TUWM           0x00000004    /* Transmit Urgent Water-Mark Interrupt Mask */
#define SPI_IMSK_CLR_ROM            0x00000010    /* Receive Over-Run Error Interrupt Mask */
#define SPI_IMSK_CLR_TUM            0x00000020    /* Transmit Under-Run Error Interrupt Mask */
#define SPI_IMSK_CLR_TCM            0x00000040    /* Transmit Collision Error Interrupt Mask */
#define SPI_IMSK_CLR_MFM            0x00000080    /* Mode Fault Error Interrupt Mask */
#define SPI_IMSK_CLR_RSM            0x00000100    /* Receive Start Interrupt Mask */
#define SPI_IMSK_CLR_TSM            0x00000200    /* Transmit Start Interrupt Mask */
#define SPI_IMSK_CLR_RFM            0x00000400    /* Receive Finish Interrupt Mask */
#define SPI_IMSK_CLR_TFM            0x00000800    /* Transmit Finish Interrupt Mask */
/* SPI_IMASKST */
#define SPI_IMSK_SET_RUWM           0x00000002    /* Receive Urgent Water-Mark Interrupt Mask */
#define SPI_IMSK_SET_TUWM           0x00000004    /* Transmit Urgent Water-Mark Interrupt Mask */
#define SPI_IMSK_SET_ROM            0x00000010    /* Receive Over-Run Error Interrupt Mask */
#define SPI_IMSK_SET_TUM            0x00000020    /* Transmit Under-Run Error Interrupt Mask */
#define SPI_IMSK_SET_TCM            0x00000040    /* Transmit Collision Error Interrupt Mask */
#define SPI_IMSK_SET_MFM            0x00000080    /* Mode Fault Error Interrupt Mask */
#define SPI_IMSK_SET_RSM            0x00000100    /* Receive Start Interrupt Mask */
#define SPI_IMSK_SET_TSM            0x00000200    /* Transmit Start Interrupt Mask */
#define SPI_IMSK_SET_RFM            0x00000400    /* Receive Finish Interrupt Mask */
#define SPI_IMSK_SET_TFM            0x00000800    /* Transmit Finish Interrupt Mask */
/* SPI_STATUS */
#define SPI_STAT_SPIF               0x00000001    /* SPI Finished */
#define SPI_STAT_RUWM               0x00000002    /* Receive Urgent Water-Mark Breached */
#define SPI_STAT_TUWM               0x00000004    /* Transmit Urgent Water-Mark Breached */
#define SPI_STAT_ROE                0x00000010    /* Receive Over-Run Error Indication */
#define SPI_STAT_TUE                0x00000020    /* Transmit Under-Run Error Indication */
#define SPI_STAT_TCE                0x00000040    /* Transmit Collision Error Indication */
#define SPI_STAT_MODF               0x00000080    /* Mode Fault Error Indication */
#define SPI_STAT_RS                 0x00000100    /* Receive Start Indication */
#define SPI_STAT_TS                 0x00000200    /* Transmit Start Indication */
#define SPI_STAT_RF                 0x00000400    /* Receive Finish Indication */
#define SPI_STAT_TF                 0x00000800    /* Transmit Finish Indication */
#define SPI_STAT_RFS                0x00007000    /* SPI_RFIFO status */
#define SPI_STAT_RFIFO_EMPTY        0x00000000    /* RFS: RFIFO Empty */
#define SPI_STAT_RFIFO_25           0x00001000    /* RFS: RFIFO 25% Full */
#define SPI_STAT_RFIFO_50           0x00002000    /* RFS: RFIFO 50% Full */
#define SPI_STAT_RFIFO_75           0x00003000    /* RFS: RFIFO 75% Full */
#define SPI_STAT_RFIFO_FULL         0x00004000    /* RFS: RFIFO Full */
#define SPI_STAT_TFS                0x00070000    /* SPI_TFIFO status */
#define SPI_STAT_TFIFO_FULL         0x00000000    /* TFS: TFIFO full */
#define SPI_STAT_TFIFO_25           0x00010000    /* TFS: TFIFO 25% empty */
#define SPI_STAT_TFIFO_50           0x00020000    /* TFS: TFIFO 50% empty */
#define SPI_STAT_TFIFO_75           0x00030000    /* TFS: TFIFO 75% empty */
#define SPI_STAT_TFIFO_EMPTY        0x00040000    /* TFS: TFIFO empty */
#define SPI_STAT_FCS                0x00100000    /* Flow-Control Stall Indication */
#define SPI_STAT_RFE                0x00400000    /* SPI_RFIFO Empty */
#define SPI_STAT_TFF                0x00800000    /* SPI_TFIFO Full */
/* SPI_ILAT */
#define SPI_ILAT_RUWMI              0x00000002    /* Receive Urgent Water Mark Interrupt */
#define SPI_ILAT_TUWMI              0x00000004    /* Transmit Urgent Water Mark Interrupt */
#define SPI_ILAT_ROI                0x00000010    /* Receive Over-Run Error Indication */
#define SPI_ILAT_TUI                0x00000020    /* Transmit Under-Run Error Indication */
#define SPI_ILAT_TCI                0x00000040    /* Transmit Collision Error Indication */
#define SPI_ILAT_MFI                0x00000080    /* Mode Fault Error Indication */
#define SPI_ILAT_RSI                0x00000100    /* Receive Start Indication */
#define SPI_ILAT_TSI                0x00000200    /* Transmit Start Indication */
#define SPI_ILAT_RFI                0x00000400    /* Receive Finish Indication */
#define SPI_ILAT_TFI                0x00000800    /* Transmit Finish Indication */
/* SPI_ILATCL */
#define SPI_ILAT_CLR_RUWMI          0x00000002    /* Receive Urgent Water Mark Interrupt */
#define SPI_ILAT_CLR_TUWMI          0x00000004    /* Transmit Urgent Water Mark Interrupt */
#define SPI_ILAT_CLR_ROI            0x00000010    /* Receive Over-Run Error Indication */
#define SPI_ILAT_CLR_TUI            0x00000020    /* Transmit Under-Run Error Indication */
#define SPI_ILAT_CLR_TCI            0x00000040    /* Transmit Collision Error Indication */
#define SPI_ILAT_CLR_MFI            0x00000080    /* Mode Fault Error Indication */
#define SPI_ILAT_CLR_RSI            0x00000100    /* Receive Start Indication */
#define SPI_ILAT_CLR_TSI            0x00000200    /* Transmit Start Indication */
#define SPI_ILAT_CLR_RFI            0x00000400    /* Receive Finish Indication */
#define SPI_ILAT_CLR_TFI            0x00000800    /* Transmit Finish Indication */

/* SPI_MMRDH */
#define SPI_MMRDH_OPCODE_BITM	    0xFF	  /* initial opcode */
#define SPI_MMRDH_ADRSIZE_1	    0x00000100	 /* 1 Byte address */
#define SPI_MMRDH_ADRSIZE_2	    0x00000200	 /* 2 Byte address */
#define SPI_MMRDH_ADRSIZE_3	    0x00000300	 /* 3 Byte address */
#define SPI_MMRDH_ADRSIZE_4	    0x00000400	 /* 4 Byte address */
#define SPI_MMRDH_ADRPINS_MIOM	    0x00000800	 /* 1 Byte address */

#define SPI_MMRDH_DUMMY_SIZE_0	    0x00000000	 /* 1 Byte address */
#define SPI_MMRDH_DUMMY_SIZE_1	    0x00001000	 /* 1 Byte address */
#define SPI_MMRDH_DUMMY_SIZE_2	    0x00002000	 /* 1 Byte address */
#define SPI_MMRDH_DUMMY_SIZE_3	    0x00003000	 /* 1 Byte address */
#define SPI_MMRDH_DUMMY_SIZE_4	    0x00004000	 /* 1 Byte address */
#define SPI_MMRDH_DUMMY_SIZE_5	    0x00005000	 /* 1 Byte address */
#define SPI_MMRDH_DUMMY_SIZE_6	    0x00006000	 /* 1 Byte address */
#define SPI_MMRDH_DUMMY_SIZE_7	    0x00007000	 /* 1 Byte address */
#define SPI_MMRDH_MODE_BITM	    0x00FF0000	 /* 1 Byte address */
#define SPI_MMRDH_MODE_TRIDMY_IMM	    0x00000000
#define SPI_MMRDH_MODE_TRIDMY_4_BITS	    0x01000000
#define SPI_MMRDH_MODE_TRIDMY_8_BITS	    0x02000000
#define SPI_MMRDH_MODE_TRIDMY_NEVER	    0x03000000
#define SPI_MMRDH_MERGE		   0x04000000
#define SPI_MMRDH_WRAP		   0x08000000
#define SPI_MMRDH_CMDSKIP	   0x10000000
#define SPI_MMRDH_CMDPINS_MOSI_ONLY	0x00000000
#define SPI_MMRDH_CMDPINS_MIOM	   0x20000000

/*
 * adi spi3 registers layout
 */
struct adi_spi_regs {
	u32 revid;
	u32 control;
	u32 rx_control;
	u32 tx_control;
	u32 clock;
	u32 delay;
	u32 ssel;
	u32 rwc;
	u32 rwcr;
	u32 twc;
	u32 twcr;
	u32 reserved0;
	u32 emask;
	u32 emaskcl;
	u32 emaskst;
	u32 reserved1;
	u32 status;
	u32 elat;
	u32 elatcl;
	u32 reserved2;
	u32 rfifo;
	u32 reserved3;
	u32 tfifo;
	u32 reserved4;
	u32 mmrdh;
	u32 mmtop;
};

struct adi_spi_controller;

struct adi_spi_transfer_ops {
	void (*write)(struct adi_spi_controller *controller, struct spi_transfer *xfer);
	void (*read)(struct adi_spi_controller *controller, struct spi_transfer *xfer);
	void (*duplex)(struct adi_spi_controller *controller, struct spi_transfer *xfer);
};

/* runtime info for spi controller */
struct adi_spi_controller {
	/* SPI framework hookup */
	struct spi_controller *controller;
	struct device *dev;

	/* Regs base of SPI controller */
	struct adi_spi_regs __iomem *regs;

	/* Current message transfer state info */
	struct spi_transfer *cur_transfer;
	const struct adi_spi_transfer_ops *ops;
	dma_cookie_t tx_cookie;
	dma_cookie_t rx_cookie;

	/* store register value for suspend/resume */
	u32 control;
	u32 ssel;

	struct clk *sclk;
	unsigned long sclk_rate;

	/* memory mapped spi flash */
	void __iomem *mm_spi_flash;
	u8 opcode;
	u32 phy_addr;

};

struct adi_spi_device {
	bool dma;
	void __iomem *mmap;
	u32 control;
};

static void adi_spi_disable(struct adi_spi_controller *drv_data)
{
	u32 ctl;

	ctl = ioread32(&drv_data->regs->control);
	ctl &= ~SPI_CTL_EN;
	iowrite32(ctl, &drv_data->regs->control);
}

static void adi_spi_dma_terminate(struct adi_spi_controller *drv_data)
{
	dmaengine_terminate_sync(drv_data->controller->dma_tx);
	dmaengine_terminate_sync(drv_data->controller->dma_rx);
}

/* Calculate the SPI_CLOCK register value based on input HZ */
static u32 hz_to_spi_clock(u32 sclk, u32 speed_hz)
{
	u32 spi_clock = sclk / speed_hz;

	if (spi_clock)
		spi_clock--;
	return spi_clock;
}

static void adi_spi_u8_write(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		iowrite32(*(u8 *)(xfer->tx_buf + i), &drv->regs->tfifo);
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		ioread32(&drv->regs->rfifo);
	}
}

static void adi_spi_u16_write(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		iowrite32(*(u16 *)(xfer->tx_buf + 2*i), &drv->regs->tfifo);
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		ioread32(&drv->regs->rfifo);
	}
}

static void adi_spi_u32_write(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		iowrite32(*(u32 *)(xfer->tx_buf + 4*i), &drv->regs->tfifo);
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		ioread32(&drv->regs->rfifo);
	}
}

static void adi_spi_u8_read(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u8 *)(xfer->rx_buf + i) = ioread32(&drv->regs->rfifo);
	}
}

static void adi_spi_u16_read(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u16 *)(xfer->rx_buf + 2*i) = ioread32(&drv->regs->rfifo);
	}
}

static void adi_spi_u32_read(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u32 *)(xfer->rx_buf + 4*i) = ioread32(&drv->regs->rfifo);
	}
}

static void adi_spi_u8_duplex(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		iowrite32(*(u8 *)(xfer->tx_buf + i), &drv->regs->tfifo);
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u8 *)(xfer->rx_buf + i) = ioread32(&drv->regs->rfifo);
	}
}

static void adi_spi_u16_duplex(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		iowrite32(*(u16 *)(xfer->tx_buf + 2*i), &drv->regs->tfifo);
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u16 *)(xfer->rx_buf + 2*i) = ioread32(&drv->regs->rfifo);
	}
}

static void adi_spi_u32_duplex(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		iowrite32(*(u32 *)(xfer->tx_buf + 4*i), &drv->regs->tfifo);
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u32 *)(xfer->rx_buf + 4*i) = ioread32(&drv->regs->rfifo);
	}
}

static const struct adi_spi_transfer_ops adi_spi_transfer_ops_u8 = {
	.write  = adi_spi_u8_write,
	.read   = adi_spi_u8_read,
	.duplex = adi_spi_u8_duplex,
};

static const struct adi_spi_transfer_ops adi_spi_transfer_ops_u16 = {
	.write  = adi_spi_u16_write,
	.read   = adi_spi_u16_read,
	.duplex = adi_spi_u16_duplex,
};

static const struct adi_spi_transfer_ops adi_spi_transfer_ops_u32 = {
	.write  = adi_spi_u32_write,
	.read   = adi_spi_u32_read,
	.duplex = adi_spi_u32_duplex,
};

#define PR_ADI_SPI_REGISTER(DEV, REG_NAME) \
	dev_info(DEV, #REG_NAME "\t%08x\n", REG_NAME) 

static void adi_spi_mm_pr_regs(struct adi_spi_controller *mmspi)
{
	u32 control, delay, rx_control, tx_control, mmrdh, mmtop;
	u32 status, rfifo, tfifo, clock, ssel;
	
	control = readl(&mmspi->regs->control);
	delay = readl(&mmspi->regs->delay);
	rx_control = readl(&mmspi->regs->rx_control);
	tx_control = readl(&mmspi->regs->tx_control);
	status = readl(&mmspi->regs->status);
	rfifo = readl(&mmspi->regs->rfifo);
	tfifo = readl(&mmspi->regs->tfifo);
	clock = readl(&mmspi->regs->clock);
	ssel = readl(&mmspi->regs->ssel);

	PR_ADI_SPI_REGISTER(mmspi->dev, control);
	PR_ADI_SPI_REGISTER(mmspi->dev, delay);
	PR_ADI_SPI_REGISTER(mmspi->dev, rx_control);
	PR_ADI_SPI_REGISTER(mmspi->dev, tx_control);
	PR_ADI_SPI_REGISTER(mmspi->dev, status);
	PR_ADI_SPI_REGISTER(mmspi->dev, rfifo);
	PR_ADI_SPI_REGISTER(mmspi->dev, tfifo);
	PR_ADI_SPI_REGISTER(mmspi->dev, clock);
	PR_ADI_SPI_REGISTER(mmspi->dev, ssel);
}

static void check_for_compatible_mm_op(u32 *buf, struct adi_spi_controller *drv)
{
	u32 compatible_ops[] = {0x03, 0x0B, 0x3B, 0x6B, 0xBB, 0xEB};
	u32 opcode = *buf & 0xFF;
	int i;

	if(drv->opcode != 0)
		return;

	for(i=0; i<6; i++)
		if(opcode == compatible_ops[i]) {
			drv->opcode = opcode;
			return;
		}

	return;
}

static void check_for_phy_addr(u32 *buf, struct adi_spi_controller *drv)
{	
	int i;

	if(drv->opcode)
		return;

	/*covers all dummy cases*/
	if ((!*buf) || (*buf > (SPI2_BASE + SPI2_SIZE)))
		return;

	drv->phy_addr = *buf;
	return;
}

static int adi_spi_pio_xfer(struct spi_controller *controller, struct spi_device *spi,
	struct spi_transfer *xfer)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(controller);

	if (!xfer->rx_buf) {
		pr_info("write called! %08x\n", *(u32*)xfer->tx_buf);
		iowrite32(SPI_RXCTL_REN, &drv->regs->rx_control);
		iowrite32(SPI_TXCTL_TEN | SPI_TXCTL_TTI, &drv->regs->tx_control);
		drv->ops->write(drv, xfer);

		check_for_compatible_mm_op((u32*)xfer->tx_buf, drv);
		check_for_phy_addr((u32*)xfer->tx_buf, drv);
	} else if (!xfer->tx_buf) {
		pr_info("read called! %08x\n", *(u32*)xfer->rx_buf);

		if(drv->opcode) {
			if(drv->phy_addr >= SPI2_BASE) {
				pr_info("%08x\n", readl(drv->phy_addr));
			} else {
				pr_info("read request outside mmap region\n");
			}
		}


		iowrite32(0, &drv->regs->tx_control);
		iowrite32(SPI_RXCTL_REN | SPI_RXCTL_RTI, &drv->regs->rx_control);
		drv->ops->read(drv, xfer);
	} else {
		iowrite32(SPI_RXCTL_REN, &drv->regs->rx_control);
		iowrite32(SPI_TXCTL_TEN | SPI_TXCTL_TTI, &drv->regs->tx_control);
		drv->ops->duplex(drv, xfer);
	}

	//adi_spi_mm_pr_regs(drv);

	iowrite32(0, &drv->regs->tx_control);
	iowrite32(0, &drv->regs->rx_control);

	return 0;
}

/*
 * Disable both paths and alert spi core that this transfer is done
 */
static void adi_spi_rx_dma_isr(void *data)
{
	struct adi_spi_controller *drv_data = data;

	struct dma_tx_state state;
	enum dma_status status;

	status = dmaengine_tx_status(drv_data->controller->dma_rx, drv_data->rx_cookie, &state);
	if (status == DMA_ERROR)
		dev_err(&drv_data->controller->dev, "spi rx dma error\n");

	iowrite32(0, &drv_data->regs->tx_control);
	iowrite32(0, &drv_data->regs->rx_control);
	spi_finalize_current_transfer(drv_data->controller);
}

/*
 * Disable tx path and enable rx path for dual/quad modes
 */
static void adi_spi_tx_dma_isr(void *data)
{
	struct adi_spi_controller *drv = data;
	struct dma_tx_state state;
	enum dma_status status;

	status = dmaengine_tx_status(drv->controller->dma_tx, drv->tx_cookie, &state);
	if (status == DMA_ERROR)
		dev_err(&drv->controller->dev, "spi tx dma error\n");

	iowrite32(0, &drv->regs->tx_control);

	if (drv->cur_transfer->rx_buf) {
		iowrite32(SPI_RXCTL_REN | SPI_RXCTL_RTI | SPI_RXCTL_RDR_NE,
				&drv->regs->rx_control);
		dma_async_issue_pending(drv->controller->dma_rx);
	} else {
		spi_finalize_current_transfer(drv->controller);
	}
}

static int adi_spi_dma_xfer(struct spi_controller *controller, struct spi_device *spi,
	struct spi_transfer *xfer)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(controller);
	struct dma_async_tx_descriptor *tx_desc;
	struct dma_async_tx_descriptor *rx_desc;

	if (xfer->tx_buf) {
		tx_desc = dmaengine_prep_slave_sg(controller->dma_tx, xfer->tx_sg.sgl,
			xfer->tx_sg.nents, DMA_MEM_TO_DEV, 0);
		if (!tx_desc) {
			dev_err(drv->dev, "Unable to allocate TX DMA descriptor\n");
			goto error;
		}

		if (!xfer->rx_buf) {
			tx_desc->callback = adi_spi_tx_dma_isr;
			tx_desc->callback_param = drv;
		}
		drv->tx_cookie = dmaengine_submit(tx_desc);

		iowrite32(SPI_TXCTL_TEN | SPI_TXCTL_TTI | SPI_TXCTL_TDR_NF,
			&drv->regs->tx_control);
		dma_async_issue_pending(controller->dma_tx);
	}

	if (xfer->rx_buf) {
		rx_desc = dmaengine_prep_slave_sg(controller->dma_rx, xfer->rx_sg.sgl,
			xfer->rx_sg.nents, DMA_DEV_TO_MEM, 0);
		if (!rx_desc) {
			dev_err(drv->dev, "Unable to allocate RX DMA descriptor\n");
			goto error;
		}

		rx_desc->callback = adi_spi_rx_dma_isr;
		rx_desc->callback_param = drv;
		drv->rx_cookie = dmaengine_submit(rx_desc);
		iowrite32(SPI_RXCTL_REN | SPI_RXCTL_RTI | SPI_RXCTL_RDR_NE,
			&drv->regs->rx_control);
		dma_async_issue_pending(controller->dma_rx);
	}

	return 1;

error:
	adi_spi_dma_terminate(drv);
	return -ENOENT;
}

/*
 * TEST 
 * */
#include <linux/spi/spi.h>

int spi_write_data(struct spi_device *spi)
{
    int ret;
    u8 tx_buf[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    struct spi_transfer t = {
        .tx_buf = tx_buf,
        .len = sizeof(tx_buf),
    };
    struct spi_message m;

    pr_info("init message\n");
    spi_message_init(&m);
    pr_info("adding to tail\n");
    spi_message_add_tail(&t, &m);

    pr_info("executing\n");
    ret = spi_sync(spi, &m);
    if (ret)
        pr_err("SPI write failed: %d\n", ret);

    return ret;
}

static struct spi_mem *test_mem;
int spi_read_data(struct spi_device *spi)
{
    int ret, i;
    int size = 1024;
    u32 *rx_buf = kzalloc(size, GFP_KERNEL);
    struct spi_mem_op op = SPI_MEM_OP(SPI_MEM_OP_CMD(0x03, 1),
                                      SPI_MEM_OP_ADDR(4, SPI2_BASE, 1),
                                      SPI_MEM_OP_NO_DUMMY,
                                      SPI_MEM_OP_DATA_IN(size, rx_buf, 1));

    ret = spi_mem_exec_op(test_mem, &op);
    if (ret)
        pr_err("SPI read failed: %d\n", ret);
    else
	    for(i=0; i<(size/32); i++)
		pr_info("Read data: %08x\n", rx_buf[i]);

    kfree(rx_buf);
    return ret;
}

static void test(struct spi_device *spi)
{
	//pr_info("writing data\n");
	//spi_write_data(spi);
	pr_info("reading data\n");
	spi_read_data(spi);
}

static bool adi_spi_can_dma(struct spi_controller *controller, struct spi_device *spi,
	struct spi_transfer *xfer)
{
	struct adi_spi_device *chip = spi_get_ctldata(spi);

	if (chip->dma)
		return true;
	return false;
}

static int adi_spi_transfer_one(struct spi_controller *controller, struct spi_device *spi,
	struct spi_transfer *xfer)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(controller);
	u32 cr;

	drv->cur_transfer = xfer;

	cr = ioread32(&drv->regs->control) & ~SPI_CTL_MIOM;

	if (xfer->rx_nbits == SPI_NBITS_QUAD || xfer->tx_nbits == SPI_NBITS_QUAD)
		cr |= SPI_CTL_MIO_QUAD;
	else if (xfer->rx_nbits == SPI_NBITS_DUAL || xfer->tx_nbits == SPI_NBITS_DUAL)
		cr |= SPI_CTL_MIO_DUAL;

	iowrite32(cr, &drv->regs->control);

	if (adi_spi_can_dma(controller, spi, xfer))
		return adi_spi_dma_xfer(controller, spi, xfer);
	return adi_spi_pio_xfer(controller, spi, xfer);
}

/*
 * Settings like clock speed and bits per word are assumed to be the same for all
 * transfers in a message. tx_nbits and rx_nbits can change, however
 */
static int adi_spi_prepare_message(struct spi_controller *controller, struct spi_message *msg)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(controller);
	struct adi_spi_device *chip = spi_get_ctldata(msg->spi);
	struct dma_slave_config dma_config = {0};
	struct spi_transfer *xfer;
	int ret;
	u32 cr, cr_width;
	u32 words;

	xfer = list_first_entry(&msg->transfers, struct spi_transfer, transfer_list);
	words = DIV_ROUND_UP(xfer->bits_per_word, 8);
	iowrite32(hz_to_spi_clock(drv->sclk_rate, xfer->speed_hz), &drv->regs->clock);

	switch (words) {
	case 1:
		cr_width = SPI_CTL_SIZE08;
		drv->ops = &adi_spi_transfer_ops_u8;
		break;
	case 2:
		cr_width = SPI_CTL_SIZE16;
		drv->ops = &adi_spi_transfer_ops_u16;
		break;
	case 4:
		cr_width = SPI_CTL_SIZE32;
		drv->ops = &adi_spi_transfer_ops_u32;
		break;
	default:
		dev_err(&controller->dev, "invalid word size in incoming message\n");
		return -EINVAL;
	}

	cr = chip->control;
	cr |= cr_width | SPI_CTL_EN;
	cr &= ~SPI_CTL_SOSI;
	iowrite32(cr, &drv->regs->control);

	dma_config.direction = DMA_MEM_TO_DEV;
	dma_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_config.src_maxburst = words;
	dma_config.dst_maxburst = words;
	ret = dmaengine_slave_config(controller->dma_tx, &dma_config);
	if (ret) {
		dev_err(drv->dev, "tx dma slave config failed: %d\n", ret);
		return ret;
	}

	dma_config.direction = DMA_DEV_TO_MEM;
	ret = dmaengine_slave_config(controller->dma_rx, &dma_config);
	if (ret) {
		dev_err(drv->dev, "rx dma slave config failed: %d\n", ret);
		return ret;
	}

	drv->opcode = 0;
	drv->phy_addr = 0;
	adi_spi_mm_pr_regs(drv);
	return 0;
}

static int adi_spi_unprepare_message(struct spi_controller *controller, struct spi_message *msg)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(controller);

	adi_spi_disable(drv);
	return 0;
}

static void adi_spi_mm_cfg(struct adi_spi_controller *mmspi, 
		const struct spi_mem_op *op)
{
	writel(0x00000000, &mmspi->regs->control);
	writel(0x00000006, &mmspi->regs->clock);
	writel(0x00000005, &mmspi->regs->rx_control);  /*  Receive Control Register */
	writel(0x00000005, &mmspi->regs->tx_control);  /*  Transmit Control Register */
	writel(0x00000301, &mmspi->regs->delay);    /*  Delay Register */
	writel(0x0000FE02, &mmspi->regs->ssel); /*  Slave Select Register */
	writel(0x05003BEB, &mmspi->regs->mmrdh);                    /*  Memory Mapped Read Header */
	writel(0x7FFFFFFF, &mmspi->regs->mmtop);
	writel(0x80240073, &mmspi->regs->control);
}

static void toggle_cs(bool cs_val) 
{
	
}

static int adi_spi_mm_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct adi_spi_controller *drv = 
		spi_controller_get_devdata(mem->spi->controller);
	u32 control;
	int i;
	void __iomem *mmap = ioremap(op->addr.val, op->data.nbytes);

	pr_info("mapping mmmap(%d B)\n", op->data.nbytes);
	if (!mmap) {
		dev_err(&mem->spi->dev, "Could not map spi memory\n");
		return -EINVAL;
	}

	pr_info("exec op called!\n");
	adi_spi_mm_cfg(drv, op);

	pr_info("attempting to read\n");
	memcpy_fromio((void *)op->data.buf.in, mmap, op->data.nbytes);

	pr_info("unmapping\n");
	iounmap(mmap);
	return 0;
}

static bool adi_spi_mm_supports_op(struct spi_mem *mem, 
		const struct spi_mem_op *op)
{
	test_mem = mem;
	if (!spi_mem_default_supports_op(mem, op))
		return false;

	return true;
}

static const char *adi_spi_mm_getname(struct spi_mem *spi_mem)
{
	return dev_name(&spi_mem->spi->dev);
}

static const struct spi_controller_mem_ops adi_spi_mm_ops = {
	.exec_op = adi_spi_mm_exec_op,
	.supports_op = adi_spi_mm_supports_op,
	.get_name = adi_spi_mm_getname
};

static int adi_spi_setup(struct spi_device *spi)
{
	struct adi_spi_device *chip;
	struct device_node *np = spi->dev.of_node;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	spi_set_ctldata(spi, chip);

	chip->dma = false;
	chip->mmap = NULL;
	if (of_property_read_bool(np, "adi,enable-dma"))
		chip->dma = true;

	chip->control = 0;
	if (of_property_read_bool(np, "adi,open-drain-mode"))
		chip->control |= SPI_CTL_ODM;

	if (of_property_read_bool(np, "adi,psse"))
		chip->control |= SPI_CTL_PSSE;

	if (of_property_present(np, "adi,mmap")) {
		//register memops
		spi->controller->mem_ops = &adi_spi_mm_ops;
	} 

	if (spi->mode & SPI_CPOL)
		chip->control |= SPI_CTL_CPOL;
	if (spi->mode & SPI_CPHA)
		chip->control |= SPI_CTL_CPHA;
	if (spi->mode & SPI_LSB_FIRST)
		chip->control |= SPI_CTL_LSBF;
	chip->control |= SPI_CTL_MSTR;
	chip->control &= ~SPI_CTL_ASSEL;

	return 0;
}

static void adi_spi_cleanup(struct spi_device *spi)
{
	struct adi_spi_device *chip = spi_get_ctldata(spi);

	if (!chip)
		return;

	spi_set_ctldata(spi, NULL);
	kfree(chip);
}

static irqreturn_t spi_irq_err(int irq, void *dev_id)
{
	struct adi_spi_controller *drv_data = dev_id;
	u32 status;

	status = ioread32(&drv_data->regs->status);
	dev_err(drv_data->dev, "spi error irq, status = 0x%x\n", status);
	iowrite32(status, &drv_data->regs->status);

	iowrite32(0, &drv_data->regs->tx_control);
	iowrite32(0, &drv_data->regs->rx_control);
	adi_spi_disable(drv_data);
	adi_spi_dma_terminate(drv_data);

	return IRQ_HANDLED;
}

static const struct of_device_id adi_spi_of_match[] = {
	{
		.compatible = "adi,spi3",
	},
	{},
};
MODULE_DEVICE_TABLE(of, adi_spi_of_match);

static int adi_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *controller;
	struct adi_spi_controller *drv_data;
	struct resource *mem;
	struct irq_domain *irq_domain;
	struct device_node *irq_of_node;
	struct irq_fwspec spi_fwspec = {
	    .param_count = 3,
	    .param = {0, 132, 4},
	};
	int virq;
	struct clk *sclk;
	int ret;

	sclk = devm_clk_get(dev, "spi");
	if (IS_ERR(sclk)) {
		dev_err(dev, "can not get spi clock\n");
		return PTR_ERR(sclk);
	}

	controller = devm_spi_alloc_master(dev, sizeof(*drv_data));
	if (!controller) {
		dev_err(dev, "can not alloc spi_controller\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, controller);

	/* the mode bits supported by this driver */
	controller->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST |
				SPI_TX_DUAL | SPI_TX_QUAD |
				SPI_RX_DUAL | SPI_RX_QUAD;

	controller->dev.of_node = dev->of_node;
	controller->bus_num = -1;
	controller->num_chipselect = 4;
	controller->cleanup = adi_spi_cleanup;
	controller->setup = adi_spi_setup;
	controller->prepare_message = adi_spi_prepare_message;
	controller->unprepare_message = adi_spi_unprepare_message;
	controller->transfer_one = adi_spi_transfer_one;
	controller->can_dma = adi_spi_can_dma;
	controller->bits_per_word_mask = BIT(32 - 1) | BIT(16 - 1) | BIT(8 - 1);
	controller->use_gpio_descriptors = true;

	drv_data = spi_controller_get_devdata(controller);
	drv_data->controller = controller;
	drv_data->sclk = sclk;
	drv_data->sclk_rate = clk_get_rate(sclk);
	drv_data->dev = dev;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drv_data->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(drv_data->regs)) {
		dev_err(dev, "Could not map spi memory, check device tree\n");
		return PTR_ERR(drv_data->regs);
	}

	drv_data->mm_spi_flash = ioremap(SPI2_BASE, SPI2_SIZE);
	if (IS_ERR(drv_data->mm_spi_flash)) {
		dev_err(dev, "Could not map spi flash memory\n");
		return PTR_ERR(drv_data->mm_spi_flash);
	}


	irq_of_node = of_find_node_by_name(NULL,"interrupt-controller");
	if (!irq_of_node) {
	    dev_err(dev, "could not find node interrupt-controller\n");
	    return -ENODEV;
	}

	irq_domain = irq_find_host(irq_of_node);
	if(!irq_domain) {
	    dev_err(dev, "could not get irq domain from interrupt-controller\n");
	    return -ENODATA;
	}

	spi_fwspec.fwnode = irq_domain->fwnode;
	virq = irq_create_fwspec_mapping(&spi_fwspec);
	ret = devm_request_irq(dev, virq, spi_irq_err, 0, "SPI ERROR", drv_data);
	if (ret) {
		dev_err(dev, "could not request spi error irq\n");
		return ret;
	}

	iowrite32(SPI_CTL_MSTR | SPI_CTL_CPHA, &drv_data->regs->control);
	iowrite32(0x0000FE00, &drv_data->regs->ssel);
	iowrite32(0x0, &drv_data->regs->delay);
	iowrite32(SPI_IMSK_SET_ROM, &drv_data->regs->emaskst);

	controller->dma_tx = dma_request_chan(dev, "tx");
	if (!controller->dma_tx) {
		dev_err(dev, "Could not get TX DMA channel\n");
		return -ENOENT;
	}

	controller->dma_rx = dma_request_chan(dev, "rx");
	if (!controller->dma_rx) {
		dev_err(dev, "Could not get RX DMA channel\n");
		ret = -ENOENT;
		goto err_free_tx_dma;
	}

	ret = clk_prepare_enable(drv_data->sclk);
	if (ret) {
		dev_err(dev, "Could not enable SPI clock\n");
		goto err_free_rx_dma;
	}

	ret = devm_spi_register_controller(dev, controller);
	if (ret) {
		dev_err(dev, "can not  register spi controller\n");
		goto err_free_rx_dma;
	}

	dev_info(dev, "registered ADI SPI controller %s\n",
					dev_name(&controller->dev));
	adi_spi_mm_pr_regs(drv_data);

	test(test_mem->spi);
	return ret;

err_free_rx_dma:
	dma_release_channel(controller->dma_rx);

err_free_tx_dma:
	dma_release_channel(controller->dma_tx);

	return ret;
}

static int adi_spi_remove(struct platform_device *pdev)
{
	struct spi_controller *controller = platform_get_drvdata(pdev);
	struct adi_spi_controller *drv_data = spi_controller_get_devdata(controller);

	adi_spi_disable(drv_data);
	clk_disable_unprepare(drv_data->sclk);
	dma_release_channel(controller->dma_tx);
	dma_release_channel(controller->dma_rx);
	return 0;
}

static int __maybe_unused adi_spi_suspend(struct device *dev)
{
	struct spi_controller *controller = dev_get_drvdata(dev);

	return spi_controller_suspend(controller);
}

static int __maybe_unused adi_spi_resume(struct device *dev)
{
	struct spi_controller *controller = dev_get_drvdata(dev);

	return spi_controller_resume(controller);
}

static const struct dev_pm_ops adi_spi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(adi_spi_suspend, adi_spi_resume)
};

MODULE_ALIAS("platform:adi-spi3");
static struct platform_driver adi_spi_driver = {
	.driver	= {
		.name	= "adi-spi3",
		.pm     = &adi_spi_pm_ops,
		.of_match_table = adi_spi_of_match,
	},
	.probe      = adi_spi_probe,
	.remove		= adi_spi_remove,
};

module_platform_driver(adi_spi_driver);

MODULE_DESCRIPTION("Analog Devices SPI3 controller driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
