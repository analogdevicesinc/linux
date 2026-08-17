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
#include <linux/math.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <linux/spi/offload/provider.h>
#include <linux/types.h>

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

	/* Optional SPI Offload framework provider state. NULL when the port
	 * has no "trigger-sources" DT property.
	 */
	struct spi_offload *offload;
	u32 offload_caps;
};

/*
 * Per-offload-instance private data.
 *
 * The ADI SPI3 peripheral does not embed a command FIFO like the axi-spi-engine
 * does; the "offload message" is a single SPI transfer whose length tells us
 * how many words must be clocked per hardware-triggered transaction. We stash
 * that word count — and the SPI_CTL mode/size bits derived from the consumer
 * device's spi->mode and the transfer's bits_per_word — here at
 * spi_optimize_message() time so that trigger_enable() can program the word
 * counters and SPI_CTL before the trigger source is armed.
 */
struct adi_spi_offload_priv {
	struct adi_spi_controller *drv;
	unsigned int xfer_words;
	u32 ctl_xfer;
	u32 speed_hz;
	unsigned long assigned;		/* SPI_ENGINE_OFFLOAD_FLAG_ASSIGNED-like */
	/*
	 * The SPI3 peripheral's ATSEL=1 mode drives whichever CS lines are
	 * enabled in SPI_SLVSEL. We stash the offload consumer's chip_select
	 * at get_offload() time and use it in trigger_enable() to program
	 * SLVSEL so that ATSEL toggles the right SPISSELn line for this
	 * particular consumer. Restored to the port-wide default (0xFE00 —
	 * all seven SS lines idle high, none enabled) in trigger_disable().
	 */
	u8 chip_select;
};

#define ADI_SPI_SLVSEL_DEFAULT		0x0000FE00
/*
 * SPI_SLVSEL layout (see the SSE1..SSE7 defines at the top of this file):
 *   bit 0     : reserved
 *   bits 1..7 : SSE1..SSE7 (SPISSELn output enables)
 *   bit 8     : reserved
 *   bits 9..15: SSEL1..SSEL7 polarity/idle value (default 1 = idle-high)
 *
 * So the enable bit for SPISSELn is BIT(n), not BIT(n+1). cs comes from
 * spi_get_chipselect() with the DT reg = 1..7 convention (SPISSEL1..7),
 * matching adi_spi_get_offload()'s range check.
 */
#define ADI_SPI_SLVSEL_SSE(cs)		BIT(cs)

#define ADI_SPI_OFFLOAD_ASSIGNED	0

struct adi_spi_device {
	bool dma;
	/*
	 * Slave uses hardware CS via the peripheral's SSELn pin (as opposed
	 * to cs-gpios). Set for offload-capable slaves that declare no
	 * cs-gpios; drives SPI_SLVSEL manually in prepare/unprepare_message
	 * so a single pinctrl state serves both the non-offload transfer
	 * path (regmap short-frames) and the offload path.
	 */
	bool hw_cs;
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
	u32 spi_clock = DIV_ROUND_UP(sclk, speed_hz);

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
		*(u16 *)(xfer->rx_buf + 2 * i) = ioread32(&drv->regs->rfifo);
	}
}

static void adi_spi_u32_read(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u32 *)(xfer->rx_buf + 4 * i) = ioread32(&drv->regs->rfifo);
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
		iowrite32(*(u16 *)(xfer->tx_buf + 2 * i), &drv->regs->tfifo);
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u16 *)(xfer->rx_buf + 2 * i) = ioread32(&drv->regs->rfifo);
	}
}

static void adi_spi_u32_duplex(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		iowrite32(*(u32 *)(xfer->tx_buf + 4 * i), &drv->regs->tfifo);
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u32 *)(xfer->rx_buf + 4 * i) = ioread32(&drv->regs->rfifo);
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

static int adi_spi_pio_xfer(struct spi_controller *controller, struct spi_device *spi,
	struct spi_transfer *xfer)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(controller);

	if (!xfer->rx_buf) {
		iowrite32(SPI_RXCTL_REN, &drv->regs->rx_control);
		iowrite32(SPI_TXCTL_TEN | SPI_TXCTL_TTI, &drv->regs->tx_control);
		drv->ops->write(drv, xfer);
	} else if (!xfer->tx_buf) {
		iowrite32(0, &drv->regs->tx_control);
		iowrite32(SPI_RXCTL_REN | SPI_RXCTL_RTI, &drv->regs->rx_control);
		drv->ops->read(drv, xfer);
	} else {
		iowrite32(SPI_RXCTL_REN, &drv->regs->rx_control);
		iowrite32(SPI_TXCTL_TEN | SPI_TXCTL_TTI, &drv->regs->tx_control);
		drv->ops->duplex(drv, xfer);
	}

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

static bool adi_spi_can_dma(struct spi_controller *controller, struct spi_device *spi,
	struct spi_transfer *xfer)
{
	struct adi_spi_device *chip = spi_get_ctldata(spi);

	/* No DMA channels claimed (offload-only port) → force PIO. */
	if (!controller->dma_tx || !controller->dma_rx)
		return false;

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

	/*
	 * Software-driven CS via SPI_SLVSEL (see adi_spi_setup() for why):
	 * program the peripheral to drive this slave's SSELn low for the
	 * duration of the message. adi_spi_unprepare_message() releases it.
	 *
	 * Format: SSE bit (bit N) = enable SSELn output; SSEL polarity bit
	 * (bit N+8) = idle value. Assert = SSE=1, POL=0. Deassert = SSE=1,
	 * POL=1 (or clear SSE which tri-states the pin).
	 *
	 * The offload path (adi_spi_offload_trigger_enable) programs SLVSEL
	 * itself with ASSEL/TWCEN semantics; unprepare_message rewrites
	 * SLVSEL to the port-wide default so the two paths coexist.
	 */
	if (chip->hw_cs) {
		u8 cs = spi_get_chipselect(msg->spi, 0);
		u32 ssel;

		/*
		 * Start from the port-wide idle-high default, then keep
		 * this slave's SSE enabled AND clear its polarity bit so
		 * SSELn drives low.
		 */
		ssel = ADI_SPI_SLVSEL_DEFAULT | ADI_SPI_SLVSEL_SSE(cs);
		ssel &= ~(ADI_SPI_SLVSEL_SSE(cs) << 8);
		iowrite32(ssel, &drv->regs->ssel);
	}

	/*
	 * dma_tx/dma_rx may be NULL when the DT declares an offload-only port
	 * (no "tx"/"rx" dma-names) -- non-offload transfers on that port fall
	 * back to PIO via adi_spi_can_dma() returning false, so we don't need
	 * to program the (absent) channels here.
	 */
	if (controller->dma_tx) {
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
	}

	if (controller->dma_rx) {
		dma_config.direction = DMA_DEV_TO_MEM;
		dma_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		dma_config.src_maxburst = words;
		dma_config.dst_maxburst = words;
		ret = dmaengine_slave_config(controller->dma_rx, &dma_config);
		if (ret) {
			dev_err(drv->dev, "rx dma slave config failed: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int adi_spi_unprepare_message(struct spi_controller *controller, struct spi_message *msg)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(controller);
	struct adi_spi_device *chip = spi_get_ctldata(msg->spi);

	adi_spi_disable(drv);
	if (chip->hw_cs)
		iowrite32(ADI_SPI_SLVSEL_DEFAULT, &drv->regs->ssel);
	return 0;
}

static void adi_spi_set_cs(struct spi_device *spi, bool high)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(spi->controller);
	u8 cs = spi_get_chipselect(spi, 0);
	u32 ssel;

	ssel = ioread32(&drv->regs->ssel);
	/* Set SSEL value bit */
	if (high)
		ssel |= BIT(cs + 8);
	else
		ssel &= ~BIT(cs + 8);
	/* Set SSE enable bit */
	ssel |= BIT(cs);
	iowrite32(ssel, &drv->regs->ssel);
}

static int adi_spi_setup(struct spi_device *spi)
{
	struct adi_spi_device *chip;
	struct device_node *np = spi->dev.of_node;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	spi_set_ctldata(spi, chip);

	chip->dma = false;
	if (of_property_read_bool(np, "adi,enable-dma"))
		chip->dma = true;

	chip->control = 0;
	if (of_property_read_bool(np, "adi,open-drain-mode"))
		chip->control |= SPI_CTL_ODM;

	if (of_property_read_bool(np, "adi,psse"))
		chip->control |= SPI_CTL_PSSE;

	if (spi->mode & SPI_CPOL)
		chip->control |= SPI_CTL_CPOL;
	if (spi->mode & SPI_CPHA)
		chip->control |= SPI_CTL_CPHA;
	if (spi->mode & SPI_LSB_FIRST)
		chip->control |= SPI_CTL_LSBF;
	chip->control |= SPI_CTL_MSTR;
	/*
	 * Two chip-select modes: cs-gpios (default, gpiolib toggles the
	 * pin) or, for offload-capable slaves without cs-gpios, hardware
	 * CS through SPI_SLVSEL/ASSEL. Mixing software CS on the spi_sync
	 * path with hardware CS on the offload path would need per-transfer
	 * pinmux swaps, so such slaves use hardware CS for both.
	 * A slave with neither cs-gpios nor offload never sees CS toggle:
	 * warn so the misconfiguration shows up in dmesg.
	 */
	if (!spi_get_csgpiod(spi, 0) && spi->controller->get_offload) {
		u8 cs = spi_get_chipselect(spi, 0);

		if (cs < 1 || cs > 7) {
			dev_err(&spi->dev,
				"hardware CS requires reg=1..7 (SPISSEL1..SPISSEL7), got reg=%u\n",
				cs);
			kfree(chip);
			spi_set_ctldata(spi, NULL);
			return -EINVAL;
		}
		/*
		 * spi_sync traffic drives CS in software through SPI_SLVSEL:
		 * prepare_message() asserts the slave's SSELn, then
		 * unprepare_message() releases it, giving one CS pulse per
		 * message through the peripheral's own SSELn pin (ASSEL=0 so
		 * the hardware doesn't toggle CS per word). The offload path
		 * instead uses ASSEL + word counters and manages CS itself.
		 */
		chip->control &= ~SPI_CTL_ASSEL;
		chip->hw_cs = true;
	} else {
		chip->control &= ~SPI_CTL_ASSEL;
		chip->hw_cs = false;
		if (!spi_get_csgpiod(spi, 0))
			dev_warn_once(&spi->dev,
				      "%s reg=%u has no cs-gpios and controller has no offload; adi-spi3 non-offload path is GPIO-CS only -- hardware SEL will not assert\n",
				      dev_name(&spi->dev),
				      spi_get_chipselect(spi, 0));
	}

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

/*
 * SPI Offload framework provider: hardware-triggered transfers with the
 * TX/RX word counters (TWCR/RWCR) gating each transaction so the
 * peripheral auto-deasserts CS, and dedicated "offload-tx"/"offload-rx"
 * DMA channels handed to the consumer.
 *
 * DT contract on the controller node:
 *   "trigger-sources" — phandle to the trigger provider
 *   "dma-names"       — must contain "offload-rx" (and optionally
 *                       "offload-tx") in addition to "tx"/"rx"
 */

static struct spi_offload *adi_spi_get_offload(struct spi_device *spi,
					       const struct spi_offload_config *config)
{
	struct spi_controller *ctrl = spi->controller;
	struct adi_spi_controller *drv = spi_controller_get_devdata(ctrl);
	struct adi_spi_offload_priv *priv;
	u8 cs = spi_get_chipselect(spi, 0);

	if (!drv->offload)
		return ERR_PTR(-ENODEV);

	if (config->capability_flags & ~drv->offload_caps)
		return ERR_PTR(-EINVAL);

	/*
	 * ATSEL=1 wants exclusive control of SPISSELn — if the framework
	 * has a cs-gpios entry for this slot the software CS path would
	 * fight the peripheral for the pin every transaction. Refuse
	 * offload rather than let both drive the line.
	 */
	if (spi_get_csgpiod(spi, 0)) {
		dev_err(drv->dev,
			"offload requires hardware CS: drop cs-gpios for cs=%u\n",
			cs);
		return ERR_PTR(-EINVAL);
	}

	if (cs < 1 || cs > 7) {
		dev_err(drv->dev,
			"offload cs=%u out of range (SPISSEL1..7)\n", cs);
		return ERR_PTR(-EINVAL);
	}

	priv = drv->offload->priv;
	priv->chip_select = cs;
	if (test_and_set_bit_lock(ADI_SPI_OFFLOAD_ASSIGNED, &priv->assigned))
		return ERR_PTR(-EBUSY);

	return drv->offload;
}

static void adi_spi_put_offload(struct spi_offload *offload)
{
	struct adi_spi_offload_priv *priv = offload->priv;

	clear_bit_unlock(ADI_SPI_OFFLOAD_ASSIGNED, &priv->assigned);
}

static int adi_spi_optimize_message(struct spi_message *msg)
{
	struct spi_controller *ctrl = msg->spi->controller;
	struct adi_spi_controller *drv = spi_controller_get_devdata(ctrl);
	struct adi_spi_offload_priv *priv;
	struct spi_transfer *xfer;

	if (!msg->offload || msg->offload != drv->offload)
		return 0;

	if (!list_is_singular(&msg->transfers)) {
		dev_err(drv->dev, "offload requires a single-transfer message\n");
		return -EOPNOTSUPP;
	}

	xfer = list_first_entry(&msg->transfers, struct spi_transfer, transfer_list);
	if (!xfer->len || xfer->len > SPI_RWC_VALUE) {
		dev_err(drv->dev, "invalid offload xfer length %u\n", xfer->len);
		return -EINVAL;
	}

	priv = drv->offload->priv;

	/*
	 * Derive the SPI_CTL mode/size bits from the consumer device and the
	 * transfer instead of hard-coding them — the spi core has already
	 * validated spi->mode against the controller's mode_bits and filled
	 * in bits_per_word.
	 */
	priv->ctl_xfer = 0;
	if (msg->spi->mode & SPI_CPOL)
		priv->ctl_xfer |= SPI_CTL_CPOL;
	if (msg->spi->mode & SPI_CPHA)
		priv->ctl_xfer |= SPI_CTL_CPHA;
	if (msg->spi->mode & SPI_LSB_FIRST)
		priv->ctl_xfer |= SPI_CTL_LSBF;

	switch (xfer->bits_per_word) {
	case 8:
		priv->ctl_xfer |= SPI_CTL_SIZE08;
		break;
	case 16:
		priv->ctl_xfer |= SPI_CTL_SIZE16;
		break;
	case 32:
		priv->ctl_xfer |= SPI_CTL_SIZE32;
		break;
	default:
		dev_err(drv->dev, "offload: unsupported bits_per_word %u\n",
			xfer->bits_per_word);
		return -EINVAL;
	}

	if (xfer->len % (xfer->bits_per_word / 8)) {
		dev_err(drv->dev,
			"offload xfer length %u not a multiple of the %u-bit word size\n",
			xfer->len, xfer->bits_per_word);
		return -EINVAL;
	}

	/* The SPI3 TWC/RWC counters count words, not bytes. */
	priv->xfer_words = xfer->len / (xfer->bits_per_word / 8);

	/*
	 * The spi core has clamped xfer->speed_hz to the device's
	 * spi-max-frequency; program it in trigger_enable() rather than
	 * inheriting whatever bit rate the last regmap transfer used.
	 */
	priv->speed_hz = xfer->speed_hz;

	return 0;
}

static int adi_spi_offload_trigger_enable(struct spi_offload *offload)
{
	struct adi_spi_offload_priv *priv = offload->priv;
	struct adi_spi_controller *drv = priv->drv;
	u32 ctl, ssel;

	if (!priv->xfer_words) {
		dev_err(drv->dev,
			"trigger_enable called without a prepared offload message\n");
		return -EINVAL;
	}

	/*
	 * Program SPI_SLVSEL first — ATSEL only drives lines whose SSEn bit
	 * is set. Keep every other CS line in its idle-high default so the
	 * ADEMA sees a clean CS while, e.g., an unrelated flash device on
	 * SSEL2 stays deasserted. The consumer's chip_select was stashed
	 * at get_offload() time.
	 */
	ssel = ADI_SPI_SLVSEL_DEFAULT | ADI_SPI_SLVSEL_SSE(priv->chip_select);
	iowrite32(ssel, &drv->regs->ssel);

	/*
	 * Flush the FIFOs before reprogramming: the port has been running
	 * regmap short-frame transfers, and per the HRM the receive FIFO is
	 * only reset "when the SPI is disabled after being enabled" — an
	 * actual EN 1->0 edge. If the port is already disabled, produce the
	 * edge explicitly (enable with no channels armed, then disable);
	 * without it, stale RFIFO bytes lead the offload RX stream and every
	 * DMA'd long-frame lands byte-shifted in its ring slot.
	 */
	iowrite32(0, &drv->regs->tx_control);
	iowrite32(0, &drv->regs->rx_control);
	if (!(ioread32(&drv->regs->control) & SPI_CTL_EN))
		iowrite32(SPI_CTL_MSTR | SPI_CTL_EN, &drv->regs->control);
	iowrite32(0, &drv->regs->control);

	/*
	 * Word counters, programmed while TXCTL/RXCTL are still zero (the
	 * HRM only allows TWC/RWC changes with the counters disabled).
	 * TWC/RWC hold the live count for the first transaction; TWCR/RWCR
	 * are only reload values latched when the count hits zero. Writing
	 * just the reloads leaves TWC=0, and with TWCEN=1 a zero TWC never
	 * requests a TX DMA read — the offload channels sit "Running"
	 * forever with no data movement.
	 */
	iowrite32(priv->xfer_words, &drv->regs->twc);
	iowrite32(priv->xfer_words, &drv->regs->twcr);
	iowrite32(priv->xfer_words, &drv->regs->rwc);
	iowrite32(priv->xfer_words, &drv->regs->rwcr);

	iowrite32(hz_to_spi_clock(drv->sclk_rate, priv->speed_hz),
		  &drv->regs->clock);

	/*
	 * Duplex lock-step initiation (TTI=RTI=1): the bus idles until the
	 * first trigger-gated TX work unit lands. TTI-only initiation is
	 * not an option — it waits for the TFIFO to drain before each
	 * transfer, which stretches one frame past the DREADY period and
	 * collapses the streaming rate (observed ~35 of 250 SPS).
	 */
	iowrite32(SPI_RXCTL_REN | SPI_RXCTL_RTI | SPI_RXCTL_RWCEN | SPI_RXCTL_RDR_NE,
		  &drv->regs->rx_control);
	iowrite32(SPI_TXCTL_TEN | SPI_TXCTL_TTI | SPI_TXCTL_TWCEN | SPI_TXCTL_TDR_NF,
		  &drv->regs->tx_control);

	/*
	 * Master mode with hardware auto-CS: ASSEL=1 asserts/deasserts CS
	 * per triggered transaction of TWC words. Mode/word-size bits were
	 * captured in adi_spi_optimize_message(). Per HRM, SPI_CTL.EN must
	 * be written after both SPI_RXCTL and SPI_TXCTL.
	 */
	ctl = SPI_CTL_MSTR | SPI_CTL_ASSEL | SPI_CTL_EN | priv->ctl_xfer;
	iowrite32(ctl, &drv->regs->control);

	return 0;
}

static void adi_spi_offload_trigger_disable(struct spi_offload *offload)
{
	struct adi_spi_offload_priv *priv = offload->priv;
	struct adi_spi_controller *drv = priv->drv;

	iowrite32(0, &drv->regs->tx_control);
	iowrite32(0, &drv->regs->rx_control);
	iowrite32(0, &drv->regs->twc);
	iowrite32(0, &drv->regs->twcr);
	iowrite32(0, &drv->regs->rwc);
	iowrite32(0, &drv->regs->rwcr);
	adi_spi_disable(drv);

	/* Return SLVSEL to the port-wide idle-high default. */
	iowrite32(ADI_SPI_SLVSEL_DEFAULT, &drv->regs->ssel);
}

static struct dma_chan *
adi_spi_offload_rx_stream_request_dma_chan(struct spi_offload *offload)
{
	struct adi_spi_offload_priv *priv = offload->priv;

	return dma_request_chan(priv->drv->dev, "offload-rx");
}

static struct dma_chan *
adi_spi_offload_tx_stream_request_dma_chan(struct spi_offload *offload)
{
	struct adi_spi_offload_priv *priv = offload->priv;

	return dma_request_chan(priv->drv->dev, "offload-tx");
}

static const struct spi_offload_ops adi_spi_offload_ops = {
	.trigger_enable			= adi_spi_offload_trigger_enable,
	.trigger_disable		= adi_spi_offload_trigger_disable,
	.rx_stream_request_dma_chan	= adi_spi_offload_rx_stream_request_dma_chan,
	.tx_stream_request_dma_chan	= adi_spi_offload_tx_stream_request_dma_chan,
};

static int adi_spi_setup_offload(struct platform_device *pdev,
				 struct adi_spi_controller *drv)
{
	struct device *dev = &pdev->dev;
	struct spi_offload *offload;
	struct adi_spi_offload_priv *priv;

	if (!device_property_present(dev, "trigger-sources"))
		return 0;

	offload = devm_spi_offload_alloc(dev, sizeof(*priv));
	if (IS_ERR(offload))
		return dev_err_probe(dev, PTR_ERR(offload),
				     "failed to allocate SPI offload\n");

	priv = offload->priv;
	priv->drv = drv;

	offload->ops = &adi_spi_offload_ops;
	drv->offload = offload;
	drv->offload_caps = SPI_OFFLOAD_CAP_TRIGGER;

	/*
	 * TX and RX offload streams are advertised only when the DT lists
	 * dedicated "offload-tx"/"offload-rx" DMA channels — the regular "tx"
	 * and "rx" channels are still owned by the normal PIO/DMA transfer
	 * path.
	 */
	if (device_property_match_string(dev, "dma-names", "offload-rx") >= 0) {
		drv->offload_caps |= SPI_OFFLOAD_CAP_RX_STREAM_DMA;
		offload->xfer_flags |= SPI_OFFLOAD_XFER_RX_STREAM;
	}
	if (device_property_match_string(dev, "dma-names", "offload-tx") >= 0) {
		drv->offload_caps |= SPI_OFFLOAD_CAP_TX_STREAM_DMA;
		offload->xfer_flags |= SPI_OFFLOAD_XFER_TX_STREAM;
	}

	drv->controller->get_offload = adi_spi_get_offload;
	drv->controller->put_offload = adi_spi_put_offload;
	drv->controller->optimize_message = adi_spi_optimize_message;

	dev_info(dev, "SPI offload provider enabled (caps=0x%x)\n",
		 drv->offload_caps);
	return 0;
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
	struct clk *sclk;
	int ret, err_irq;

	sclk = devm_clk_get(dev, "spi");
	if (IS_ERR(sclk)) {
		dev_err(dev, "can not get spi clock\n");
		return PTR_ERR(sclk);
	}

	controller = devm_spi_alloc_host(dev, sizeof(*drv_data));
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
	controller->num_chipselect = 8;
	controller->use_gpio_descriptors = true;
	controller->set_cs = adi_spi_set_cs;
	controller->cleanup = adi_spi_cleanup;
	controller->setup = adi_spi_setup;
	controller->prepare_message = adi_spi_prepare_message;
	controller->unprepare_message = adi_spi_unprepare_message;
	controller->transfer_one = adi_spi_transfer_one;
	controller->can_dma = adi_spi_can_dma;
	controller->bits_per_word_mask = BIT(32 - 1) | BIT(16 - 1) | BIT(8 - 1);

	drv_data = spi_controller_get_devdata(controller);
	drv_data->controller = controller;
	drv_data->sclk = sclk;
	drv_data->dev = dev;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drv_data->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(drv_data->regs)) {
		dev_err(dev, "Could not map spi memory, check device tree\n");
		return PTR_ERR(drv_data->regs);
	}

	err_irq = platform_get_irq(pdev, 0);
	if (!err_irq) {
		dev_err(dev, "No SPI error irq resource found\n");
		return -ENODEV;
	}

	ret = devm_request_irq(dev, err_irq, spi_irq_err, 0, "SPI ERROR", drv_data);
	if (ret) {
		dev_err(dev, "could not request spi error irq\n");
		return ret;
	}

	iowrite32(SPI_CTL_MSTR | SPI_CTL_CPHA, &drv_data->regs->control);
	iowrite32(0x0000FE00, &drv_data->regs->ssel);
	iowrite32(0x0, &drv_data->regs->delay);
	iowrite32(SPI_IMSK_SET_ROM, &drv_data->regs->emaskst);

	/*
	 * The regular "tx"/"rx" DMA channels drive adi_spi_dma_xfer for
	 * slaves that opt into DMA via `adi,enable-dma`. When the DT lists
	 * only "offload-tx"/"offload-rx" (single physical DMA channel per
	 * SPI port owned by the offload consumer), skip them and fall back
	 * to PIO for regmap-style short-frames on the non-offload path.
	 * adi_spi_can_dma() gates the DMA path on chip->dma so PIO fallback
	 * is automatic; ENODEV here just means the DT decided this port is
	 * offload-only.
	 */
	controller->dma_tx = dma_request_chan(dev, "tx");
	if (IS_ERR(controller->dma_tx)) {
		ret = PTR_ERR(controller->dma_tx);
		if (ret != -ENODEV) {
			dev_err(dev, "Could not get TX DMA channel: %d\n", ret);
			return ret;
		}
		dev_info(dev, "no \"tx\" DMA channel -- PIO-only for non-offload transfers\n");
		controller->dma_tx = NULL;
	}

	controller->dma_rx = dma_request_chan(dev, "rx");
	if (IS_ERR(controller->dma_rx)) {
		ret = PTR_ERR(controller->dma_rx);
		if (ret != -ENODEV) {
			dev_err(dev, "Could not get RX DMA channel: %d\n", ret);
			goto err_free_tx_dma;
		}
		controller->dma_rx = NULL;
	}

	ret = clk_prepare_enable(drv_data->sclk);
	if (ret) {
		dev_err(dev, "Could not enable SPI clock\n");
		goto err_free_rx_dma;
	}

	drv_data->sclk_rate = clk_get_rate(drv_data->sclk);
	if (!drv_data->sclk_rate) {
		dev_err(dev, "Invalid SPI clock rate: %lu Hz\n", drv_data->sclk_rate);
		ret = -EINVAL;
		goto err_free_rx_dma;
	}

	ret = adi_spi_setup_offload(pdev, drv_data);
	if (ret)
		goto err_free_rx_dma;

	ret = devm_spi_register_controller(dev, controller);
	if (ret) {
		dev_err(dev, "can not  register spi controller\n");
		goto err_free_rx_dma;
	}

	dev_info(dev, "registered ADI SPI controller %s\n",
					dev_name(&controller->dev));
	return ret;

err_free_rx_dma:
	if (controller->dma_rx)
		dma_release_channel(controller->dma_rx);

err_free_tx_dma:
	if (controller->dma_tx)
		dma_release_channel(controller->dma_tx);

	return ret;
}

static void adi_spi_remove(struct platform_device *pdev)
{
	struct spi_controller *controller = platform_get_drvdata(pdev);
	struct adi_spi_controller *drv_data = spi_controller_get_devdata(controller);

	adi_spi_disable(drv_data);
	clk_disable_unprepare(drv_data->sclk);
	if (controller->dma_tx)
		dma_release_channel(controller->dma_tx);
	if (controller->dma_rx)
		dma_release_channel(controller->dma_rx);
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
MODULE_LICENSE("GPL");
