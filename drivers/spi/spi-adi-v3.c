// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices SPI3 controller driver
 *
 * Copyright (c) 2014 - 2024 Analog Devices Inc.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/spi/spi-mem.h>

/*
 * ADI SPI registers layout
 */
#define ADI_SPI_REVID                 0x00

#define ADI_SPI_CTL                   0x04
#define   SPI_CTL_SOSI                BIT(22)           /* Start on MOSI */
#define   SPI_CTL_MIOM                GENMASK(21, 20)   /* Multiple I/O Mode */
#define   SPI_CTL_MIO_DIS             0x00000000        /* MIOM: Disable */
#define   SPI_CTL_MIO_DUAL            0x00100000        /* MIOM: Enable DIOM (Dual I/O Mode) */
#define   SPI_CTL_MIO_QUAD            0x00200000        /* MIOM: Enable QUAD (Quad SPI Mode) */
#define   SPI_CTL_FMODE               BIT(18)           /* Fast-mode Enable */
#define   SPI_CTL_FCWM                GENMASK(17, 16)   /* Flow-Control Watermark */
#define   SPI_CTL_FIFO0               0x00000000        /* FCWM: TFIFO empty or RFIFO Full */
#define   SPI_CTL_FIFO1               0x00010000        /* FCWM: TFIFO 75% or more empty or RFIFO 75% or more full */
#define   SPI_CTL_FIFO2               0x00020000        /* FCWM: TFIFO 50% or more empty or RFIFO 50% or more full */
#define   SPI_CTL_FCPL                BIT(15)           /* Flow-Control Polarity */
#define   SPI_CTL_FCCH                BIT(14)           /* Flow-Control Channel Selection */
#define   SPI_CTL_FCEN                BIT(13)           /* Flow-Control Enable */
#define   SPI_CTL_LSBF                BIT(12)           /* LSB First */
#define   SPI_CTL_SIZE                GENMASK(10, 9)    /* Word Transfer Size */
#define   SPI_CTL_SIZE08              0x00000000        /* SIZE: 8 bits */
#define   SPI_CTL_SIZE16              0x00000200        /* SIZE: 16 bits */
#define   SPI_CTL_SIZE32              0x00000400        /* SIZE: 32 bits */
#define   SPI_CTL_EMISO               BIT(8)            /* Enable MISO */
#define   SPI_CTL_SELST               BIT(7)            /* Slave Select Polarity Between Transfers */
#define   SPI_CTL_ASSEL               BIT(6)            /* Slave Select Pin Control */
#define   SPI_CTL_CPOL                BIT(5)            /* Clock Polarity */
#define   SPI_CTL_CPHA                BIT(4)            /* Clock Phase */
#define   SPI_CTL_ODM                 BIT(3)            /* Open Drain Mode */
#define   SPI_CTL_PSSE                BIT(2)            /* Protected Slave Select Enable */
#define   SPI_CTL_MSTR                BIT(1)            /* Master/Slave */
#define   SPI_CTL_EN                  BIT(0)            /* Enable */

#define ADI_SPI_RXCTL                 0x08
#define   SPI_RXCTL_RUWM              GENMASK(18, 16)   /* FIFO Urgent Watermark */
#define   SPI_RXCTL_UWM_DIS           0x00000000        /* RUWM: Disabled */
#define   SPI_RXCTL_UWM_25            0x00010000        /* RUWM: RFIFO 25% full */
#define   SPI_RXCTL_UWM_50            0x00020000        /* RUWM: RFIFO 50% full */
#define   SPI_RXCTL_UWM_75            0x00030000        /* RUWM: RFIFO 75% full */
#define   SPI_RXCTL_UWM_FULL          0x00040000        /* RUWM: RFIFO full */
#define   SPI_RXCTL_RRWM              GENMASK(13, 12)   /* FIFO Regular Watermark */
#define   SPI_RXCTL_RWM_0             0x00000000        /* RRWM: RFIFO Empty */
#define   SPI_RXCTL_RWM_25            0x00001000        /* RRWM: RFIFO 25% full */
#define   SPI_RXCTL_RWM_50            0x00002000        /* RRWM: RFIFO 50% full */
#define   SPI_RXCTL_RWM_75            0x00003000        /* RRWM: RFIFO 75% full */
#define   SPI_RXCTL_RDO               BIT(8)            /* Receive Data Overrun */
#define   SPI_RXCTL_RDR               GENMASK(6, 4)     /* Receive Data Request */
#define   SPI_RXCTL_RDR_DIS           0x00000000        /* RDR: Disabled */
#define   SPI_RXCTL_RDR_NE            0x00000010        /* RDR: RFIFO not empty */
#define   SPI_RXCTL_RDR_25            0x00000020        /* RDR: RFIFO 25% full */
#define   SPI_RXCTL_RDR_50            0x00000030        /* RDR: RFIFO 50% full */
#define   SPI_RXCTL_RDR_75            0x00000040        /* RDR: RFIFO 75% full */
#define   SPI_RXCTL_RDR_FULL          0x00000050        /* RDR: RFIFO full */
#define   SPI_RXCTL_RWCEN             BIT(3)            /* Receive Word Counter Enable */
#define   SPI_RXCTL_RTI               BIT(2)            /* Receive Transfer Initiate */
#define   SPI_RXCTL_REN               BIT(0)            /* Receive Channel Enable */

#define ADI_SPI_TXCTL                 0x0c
#define   SPI_TXCTL_TUWM              GENMASK(18, 16)   /* FIFO Urgent Watermark */
#define   SPI_TXCTL_UWM_DIS           0x00000000        /* TUWM: Disabled */
#define   SPI_TXCTL_UWM_25            0x00010000        /* TUWM: TFIFO 25% empty */
#define   SPI_TXCTL_UWM_50            0x00020000        /* TUWM: TFIFO 50% empty */
#define   SPI_TXCTL_UWM_75            0x00030000        /* TUWM: TFIFO 75% empty */
#define   SPI_TXCTL_UWM_EMPTY         0x00040000        /* TUWM: TFIFO empty */
#define   SPI_TXCTL_TRWM              GENMASK(13, 12)   /* FIFO Regular Watermark */
#define   SPI_TXCTL_RWM_FULL          0x00000000        /* TRWM: TFIFO full */
#define   SPI_TXCTL_RWM_25            0x00001000        /* TRWM: TFIFO 25% empty */
#define   SPI_TXCTL_RWM_50            0x00002000        /* TRWM: TFIFO 50% empty */
#define   SPI_TXCTL_RWM_75            0x00003000        /* TRWM: TFIFO 75% empty */
#define   SPI_TXCTL_TDU               BIT(8)            /* Transmit Data Under-Run */
#define   SPI_TXCTL_TDR               GENMASK(6, 4)     /* Transmit Data Request */
#define   SPI_TXCTL_TDR_DIS           0x00000000        /* TDR: Disabled */
#define   SPI_TXCTL_TDR_NF            0x00000010        /* TDR: TFIFO not full */
#define   SPI_TXCTL_TDR_25            0x00000020        /* TDR: TFIFO 25% empty */
#define   SPI_TXCTL_TDR_50            0x00000030        /* TDR: TFIFO 50% empty */
#define   SPI_TXCTL_TDR_75            0x00000040        /* TDR: TFIFO 75% empty */
#define   SPI_TXCTL_TDR_EMPTY         0x00000050        /* TDR: TFIFO empty */
#define   SPI_TXCTL_TWCEN             BIT(3)            /* Transmit Word Counter Enable */
#define   SPI_TXCTL_TTI               BIT(2)            /* Transmit Transfer Initiate */
#define   SPI_TXCTL_TEN               BIT(0)            /* Transmit Channel Enable */

#define ADI_SPI_CLK                   0x10
#define   SPI_CLK_BAUD                GENMASK(15, 0)  /* Baud Rate */

#define ADI_SPI_DLY                   0x14
#define   SPI_DLY_LAGX                BIT(9)            /* Extended (1 SCK) LAG control */
#define   SPI_DLY_LEADX               BIT(8)            /* Extended (1 SCK) LEAD Control */
#define   SPI_DLY_STOP                GENMASK(7, 0)     /* Transfer delay time in multiples of SCK period */

#define ADI_SPI_SLVSEL                0x18
#define   SPI_SLVSEL_SSEL7            BIT(15)           /* SPISSEL7 Value */
#define   SPI_SLVSEL_SSEL6            BIT(14)           /* SPISSEL6 Value */
#define   SPI_SLVSEL_SSEL5            BIT(13)           /* SPISSEL5 Value */
#define   SPI_SLVSEL_SSEL4            BIT(12)           /* SPISSEL4 Value */
#define   SPI_SLVSEL_SSEL3            BIT(11)           /* SPISSEL3 Value */
#define   SPI_SLVSEL_SSEL2            BIT(10)           /* SPISSEL2 Value */
#define   SPI_SLVSEL_SSEL1            BIT(9)            /* SPISSEL1 Value */
#define   SPI_SLVSEL_SSE7             BIT(7)            /* SPISSEL7 Enable */
#define   SPI_SLVSEL_SSE6             BIT(6)            /* SPISSEL6 Enable */
#define   SPI_SLVSEL_SSE5             BIT(5)            /* SPISSEL5 Enable */
#define   SPI_SLVSEL_SSE4             BIT(4)            /* SPISSEL4 Enable */
#define   SPI_SLVSEL_SSE3             BIT(3)            /* SPISSEL3 Enable */
#define   SPI_SLVSEL_SSE2             BIT(2)            /* SPISSEL2 Enable */
#define   SPI_SLVSEL_SSE1             BIT(1)            /* SPISSEL1 Enable */

#define ADI_SPI_RWC                   0x1c
#define   SPI_RWC_VALUE               GENMASK(15, 0)  /* Received Word-Count */

#define ADI_SPI_RWCR                  0x20
#define   SPI_RWCR_VALUE              GENMASK(15, 0)  /* Received Word-Count Reload */

#define ADI_SPI_TWC                   0x24
#define   SPI_TWC_VALUE               GENMASK(15, 0)  /* Transmitted Word-Count */

#define ADI_SPI_TWCR                  0x28
#define   SPI_TWCR_VALUE              GENMASK(15, 0)  /* Transmitted Word-Count Reload */

#define ADI_SPI_IMSK                  0x30
#define   SPI_IMSK_TFM                BIT(11)           /* Transmit Finish Interrupt Mask */
#define   SPI_IMSK_RFM                BIT(10)           /* Receive Finish Interrupt Mask */
#define   SPI_IMSK_TSM                BIT(9)            /* Transmit Start Interrupt Mask */
#define   SPI_IMSK_RSM                BIT(8)            /* Receive Start Interrupt Mask */
#define   SPI_IMSK_MFM                BIT(7)            /* Mode Fault Error Interrupt Mask */
#define   SPI_IMSK_TCM                BIT(6)            /* Transmit Collision Error Interrupt Mask */
#define   SPI_IMSK_TUM                BIT(5)            /* Transmit Under-Run Error Interrupt Mask */
#define   SPI_IMSK_ROM                BIT(4)            /* Receive Overrun Error Interrupt Mask */
#define   SPI_IMSK_TUWM               BIT(2)            /* Transmit Urgent Watermark Interrupt Mask */
#define   SPI_IMSK_RUWM               BIT(1)            /* Receive Urgent Watermark Interrupt Mask */

#define ADI_SPI_IMSK_CLR              0x34
#define   SPI_IMSK_CLR_TFM            BIT(11)           /* Clear Transmit Finish Interrupt Mask */
#define   SPI_IMSK_CLR_RFM            BIT(10)           /* Clear Receive Finish Interrupt Mask */
#define   SPI_IMSK_CLR_TSM            BIT(9)            /* Clear Transmit Start Interrupt Mask */
#define   SPI_IMSK_CLR_RSM            BIT(8)            /* Clear Receive Start Interrupt Mask */
#define   SPI_IMSK_CLR_MFM            BIT(7)            /* Clear Mode Fault Interrupt Mask */
#define   SPI_IMSK_CLR_TCM            BIT(6)            /* Clear Transmit Collision Interrupt Mask */
#define   SPI_IMSK_CLR_TUM            BIT(5)            /* Clear Transmit Under-run Interrupt Mask */
#define   SPI_IMSK_CLR_ROM            BIT(4)            /* Clear Receive Overrun Interrupt Mask */
#define   SPI_IMSK_CLR_TUWM           BIT(2)            /* Clear Transmit Urgent Watermark Interrupt Mask */
#define   SPI_IMSK_CLR_RUW            BIT(1)            /* Clear Receive Urgent Watermark Interrupt Mask */

#define ADI_SPI_IMSK_SET              0x38
#define   SPI_IMSK_SET_TFM            BIT(11)           /* Set Transmit Finish Interrupt Mask */
#define   SPI_IMSK_SET_RFM            BIT(10)           /* Set Receive Finish Interrupt Mask */
#define   SPI_IMSK_SET_TSM            BIT(9)            /* Set Transmit Start Interrupt Mask */
#define   SPI_IMSK_SET_RSM            BIT(8)            /* Set Receive Start Interrupt Mask */
#define   SPI_IMSK_SET_MFM            BIT(7)            /* Set Mode Fault Interrupt Mask */
#define   SPI_IMSK_SET_TCM            BIT(6)            /* Set Transmit Collision Interrupt Mask */
#define   SPI_IMSK_SET_TUM            BIT(5)            /* Set Transmit Under-run Interrupt Mask */
#define   SPI_IMSK_SET_ROM            BIT(4)            /* Set Receive Overrun Interrupt Mask */
#define   SPI_IMSK_SET_TUWM           BIT(2)            /* Set Transmit Urgent Watermark Interrupt Mask */
#define   SPI_IMSK_SET_RUWM           BIT(1)            /* Set Receive Urgent Watermark Interrupt Mask */

#define ADI_SPI_STAT                  0x40
#define   SPI_STAT_TFF                BIT(23)           /* SPI_TFIFO Full */
#define   SPI_STAT_RFE                BIT(22)           /* SPI_RFIFO Empty */
#define   SPI_STAT_FCS                BIT(20)           /* Flow-Control Stall Indication */
#define   SPI_STAT_TFS                GENMASK(18, 16)   /* SPI_TFIFO status */
#define   SPI_STAT_TFIFO_FULL         0x00000000        /* TFS: TFIFO full */
#define   SPI_STAT_TFIFO_25           0x00010000        /* TFS: TFIFO 25% empty */
#define   SPI_STAT_TFIFO_50           0x00020000        /* TFS: TFIFO 50% empty */
#define   SPI_STAT_TFIFO_75           0x00030000        /* TFS: TFIFO 75% empty */
#define   SPI_STAT_TFIFO_EMPTY        0x00040000        /* TFS: TFIFO empty */
#define   SPI_STAT_RFS                GENMASK(14, 12)   /* SPI_RFIFO status */
#define   SPI_STAT_RFIFO_EMPTY        0x00000000        /* RFS: RFIFO Empty */
#define   SPI_STAT_RFIFO_25           0x00001000        /* RFS: RFIFO 25% Full */
#define   SPI_STAT_RFIFO_50           0x00002000        /* RFS: RFIFO 50% Full */
#define   SPI_STAT_RFIFO_75           0x00003000        /* RFS: RFIFO 75% Full */
#define   SPI_STAT_RFIFO_FULL         0x00004000        /* RFS: RFIFO Full */
#define   SPI_STAT_TF                 BIT(11)           /* Transmit Finish Indication */
#define   SPI_STAT_RF                 BIT(10)           /* Receive Finish Indication */
#define   SPI_STAT_TS                 BIT(9)            /* Transmit Start */
#define   SPI_STAT_RS                 BIT(8)            /* Receive Start */
#define   SPI_STAT_MODF               BIT(7)            /* Mode Fault Error Indication */
#define   SPI_STAT_TCE                BIT(6)            /* Transmit Collision Error Indication */
#define   SPI_STAT_TUE                BIT(5)            /* Transmit Under-Run Error Indication */
#define   SPI_STAT_ROE                BIT(4)            /* Receive Overrun Error Indication */
#define   SPI_STAT_TUWM               BIT(2)            /* Transmit Urgent Watermark Breached */
#define   SPI_STAT_RUWM               BIT(1)            /* Receive Urgent Watermark Breached */
#define   SPI_STAT_SPIF               BIT(0)            /* SPI Finished */

#define ADI_SPI_ILAT                  0x44
#define   SPI_ILAT_TFI                BIT(11)           /* Transmit Finish Interrupt Latch */
#define   SPI_ILAT_RFI                BIT(10)           /* Receive Finish Interrupt Latch */
#define   SPI_ILAT_TSI                BIT(9)            /* Transmit Start Interrupt Latch */
#define   SPI_ILAT_RSI                BIT(8)            /* Receive Start Interrupt Latch */
#define   SPI_ILAT_MFI                BIT(7)            /* Mode Fault Interrupt Latch */
#define   SPI_ILAT_TCI                BIT(6)            /* Transmit Collision Interrupt Latch */
#define   SPI_ILAT_TUI                BIT(5)            /* Transmit Under-run Interrupt Latch */
#define   SPI_ILAT_ROI                BIT(4)            /* Receive Overrun Interrupt Latch */
#define   SPI_ILAT_TUWMI              BIT(2)            /* Transmit Urgent Watermark Interrupt Latch */
#define   SPI_ILAT_RUWMI              BIT(1)            /* Receive Urgent Watermark Interrupt Latch */

#define ADI_SPI_ILAT_CLR              0x48
#define   SPI_ILAT_CLR_TFI            BIT(11)           /* Clear Transmit Finish Interrupt Latch */
#define   SPI_ILAT_CLR_RFI            BIT(10)           /* Clear Receive Finish Interrupt Latch */
#define   SPI_ILAT_CLR_TSI            BIT(9)            /* Clear Transmit Start Interrupt Latch */
#define   SPI_ILAT_CLR_RSI            BIT(8)            /* Clear Receive Start Interrupt Latch */
#define   SPI_ILAT_CLR_MFI            BIT(7)            /* Clear Mode Fault Interrupt Latch */
#define   SPI_ILAT_CLR_TCI            BIT(6)            /* Clear Transmit Collision Interrupt Latch */
#define   SPI_ILAT_CLR_TUI            BIT(5)            /* Clear Transmit Under-run Interrupt Latch */
#define   SPI_ILAT_CLR_ROI            BIT(4)            /* Clear Receive Overrun Interrupt Latch */
#define   SPI_ILAT_CLR_TUWMI          BIT(2)            /* Clear Transmit Urgent Watermark Interrupt Latch */
#define   SPI_ILAT_CLR_RUWMI          BIT(1)            /* Clear Receive Urgent Watermark Interrupt Latch */

#define ADI_SPI_RFIFO                 0x50
#define ADI_SPI_TFIFO                 0x58

#define SPI_MAX_SS                    7                         /* Maximum number of native slave selects */
#define SPI_SSE(n)                    BIT((n) + 1)              /* Slave Select Enable (SSE-x) Bit Select */
#define SPI_SSEL(n)                   BIT((n) + 9)              /* Slave Select Value (SSEL-x) Bit Select */

#define MAX_SPI_TRANSFER_SIZE         ((64 * 1024) - 4096)      /* Arbitrary value lower than 64K (DDE transfer limit) */

struct adi_spi_controller;

struct adi_spi_transfer_ops {
	void (*write)(struct adi_spi_controller *drv, struct spi_transfer *xfer);
	void (*read)(struct adi_spi_controller *drv, struct spi_transfer *xfer);
	void (*duplex)(struct adi_spi_controller *drv, struct spi_transfer *xfer);
};

/* runtime info for spi controller */
struct adi_spi_controller {
	/* SPI framework hookup */
	struct spi_controller *ctlr;
	struct device *dev;

	/* Regs base of SPI controller */
	void __iomem *regs;

	int irq;

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
};

struct adi_spi_device {
	bool dma;
	u32 control;
};

static void adi_spi_disable(struct adi_spi_controller *drv_data)
{
	u32 ctl;

	ctl = ioread32(drv_data->regs + ADI_SPI_CTL);
	ctl &= ~SPI_CTL_EN;
	iowrite32(ctl, drv_data->regs + ADI_SPI_CTL);
}

static void adi_spi_dma_terminate(struct adi_spi_controller *drv_data)
{
	dmaengine_terminate_sync(drv_data->ctlr->dma_tx);
	dmaengine_terminate_sync(drv_data->ctlr->dma_rx);
}

/* Caculate the SPI_CLOCK register value based on input HZ */
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
		iowrite32(*(u8 *)(xfer->tx_buf + i), drv->regs + ADI_SPI_TFIFO);
		while (ioread32(drv->regs + ADI_SPI_STAT) & SPI_STAT_RFE)
			cpu_relax();
		ioread32(drv->regs + ADI_SPI_RFIFO);
	}
}

static void adi_spi_u16_write(struct adi_spi_controller *drv,
			      struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		iowrite32(*(u16 *)(xfer->tx_buf + 2 * i), drv->regs + ADI_SPI_TFIFO);
		while (ioread32(drv->regs + ADI_SPI_STAT) & SPI_STAT_RFE)
			cpu_relax();
		ioread32(drv->regs + ADI_SPI_RFIFO);
	}
}

static void adi_spi_u32_write(struct adi_spi_controller *drv,
			      struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		iowrite32(*(u32 *)(xfer->tx_buf + 4 * i), drv->regs + ADI_SPI_TFIFO);
		while (ioread32(drv->regs + ADI_SPI_STAT) & SPI_STAT_RFE)
			cpu_relax();
		ioread32(drv->regs + ADI_SPI_RFIFO);
	}
}

static void adi_spi_u8_read(struct adi_spi_controller *drv,
			    struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		while (ioread32(drv->regs + ADI_SPI_STAT) & SPI_STAT_RFE)
			cpu_relax();
		*(u8 *)(xfer->rx_buf + i) = ioread32(drv->regs + ADI_SPI_RFIFO);
	}
}

static void adi_spi_u16_read(struct adi_spi_controller *drv,
			     struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		while (ioread32(drv->regs + ADI_SPI_STAT) & SPI_STAT_RFE)
			cpu_relax();
		*(u16 *)(xfer->rx_buf + 2 * i) = ioread32(drv->regs + ADI_SPI_RFIFO);
	}
}

static void adi_spi_u32_read(struct adi_spi_controller *drv,
			     struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		while (ioread32(drv->regs + ADI_SPI_STAT) & SPI_STAT_RFE)
			cpu_relax();
		*(u32 *)(xfer->rx_buf + 4 * i) = ioread32(drv->regs + ADI_SPI_RFIFO);
	}
}

static void adi_spi_u8_duplex(struct adi_spi_controller *drv,
			      struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		iowrite32(*(u8 *)(xfer->tx_buf + i), drv->regs + ADI_SPI_TFIFO);
		while (ioread32(drv->regs + ADI_SPI_STAT) & SPI_STAT_RFE)
			cpu_relax();
		*(u8 *)(xfer->rx_buf + i) = ioread32(drv->regs + ADI_SPI_RFIFO);
	}
}

static void adi_spi_u16_duplex(struct adi_spi_controller *drv,
			       struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		iowrite32(*(u16 *)(xfer->tx_buf + 2 * i), drv->regs + ADI_SPI_TFIFO);
		while (ioread32(drv->regs + ADI_SPI_STAT) & SPI_STAT_RFE)
			cpu_relax();
		*(u16 *)(xfer->rx_buf + 2 * i) = ioread32(drv->regs + ADI_SPI_RFIFO);
	}
}

static void adi_spi_u32_duplex(struct adi_spi_controller *drv,
			       struct spi_transfer *xfer)
{
	size_t i;

	for (i = 0; i < xfer->len; ++i) {
		iowrite32(*(u32 *)(xfer->tx_buf + 4 * i), drv->regs + ADI_SPI_TFIFO);
		while (ioread32(drv->regs + ADI_SPI_STAT) & SPI_STAT_RFE)
			cpu_relax();
		*(u32 *)(xfer->rx_buf + 4 * i) = ioread32(drv->regs + ADI_SPI_RFIFO);
	}
}

static const struct adi_spi_transfer_ops adi_spi_transfer_ops_u8 = {
	.write	= adi_spi_u8_write,
	.read	= adi_spi_u8_read,
	.duplex = adi_spi_u8_duplex,
};

static const struct adi_spi_transfer_ops adi_spi_transfer_ops_u16 = {
	.write	= adi_spi_u16_write,
	.read	= adi_spi_u16_read,
	.duplex = adi_spi_u16_duplex,
};

static const struct adi_spi_transfer_ops adi_spi_transfer_ops_u32 = {
	.write	= adi_spi_u32_write,
	.read	= adi_spi_u32_read,
	.duplex = adi_spi_u32_duplex,
};

static int adi_spi_pio_xfer(struct spi_controller *ctlr, struct spi_device *spi,
			    struct spi_transfer *xfer)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(ctlr);

	if (!xfer->rx_buf) {
		iowrite32(SPI_RXCTL_REN, drv->regs + ADI_SPI_RXCTL);
		iowrite32(SPI_TXCTL_TEN | SPI_TXCTL_TTI, drv->regs + ADI_SPI_TXCTL);
		drv->ops->write(drv, xfer);
	} else if (!xfer->tx_buf) {
		iowrite32(0, drv->regs + ADI_SPI_TXCTL);
		iowrite32(SPI_RXCTL_REN | SPI_RXCTL_RTI, drv->regs + ADI_SPI_RXCTL);
		drv->ops->read(drv, xfer);
	} else {
		iowrite32(SPI_RXCTL_REN, drv->regs + ADI_SPI_RXCTL);
		iowrite32(SPI_TXCTL_TEN | SPI_TXCTL_TTI, drv->regs + ADI_SPI_TXCTL);
		drv->ops->duplex(drv, xfer);
	}

	iowrite32(0, drv->regs + ADI_SPI_TXCTL);
	iowrite32(0, drv->regs + ADI_SPI_RXCTL);
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

	status = dmaengine_tx_status(drv_data->ctlr->dma_rx, drv_data->rx_cookie, &state);
	if (status == DMA_ERROR)
		dev_err(&drv_data->ctlr->dev, "spi rx dma error\n");

	iowrite32(0, drv_data->regs + ADI_SPI_TXCTL);
	iowrite32(0, drv_data->regs + ADI_SPI_RXCTL);
	spi_finalize_current_transfer(drv_data->ctlr);
}

/*
 * Disable tx path and enable rx path for dual/quad modes
 */
static void adi_spi_tx_dma_isr(void *data)
{
	struct adi_spi_controller *drv = data;
	struct dma_tx_state state;
	enum dma_status status;

	status = dmaengine_tx_status(drv->ctlr->dma_tx, drv->tx_cookie, &state);
	if (status == DMA_ERROR)
		dev_err(&drv->ctlr->dev, "spi tx dma error\n");

	iowrite32(0, drv->regs + ADI_SPI_TXCTL);

	if (drv->cur_transfer->rx_buf) {
		iowrite32(SPI_RXCTL_REN | SPI_RXCTL_RTI | SPI_RXCTL_RDR_NE,
			  drv->regs + ADI_SPI_RXCTL);
		dma_async_issue_pending(drv->ctlr->dma_rx);
	} else {
		spi_finalize_current_transfer(drv->ctlr);
	}
}

static int adi_spi_dma_xfer(struct spi_controller *ctlr, struct spi_device *spi,
			    struct spi_transfer *xfer)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(ctlr);
	struct dma_async_tx_descriptor *tx_desc;
	struct dma_async_tx_descriptor *rx_desc;

	if (xfer->tx_buf) {
		tx_desc = dmaengine_prep_slave_sg(ctlr->dma_tx, xfer->tx_sg.sgl,
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
			  drv->regs + ADI_SPI_TXCTL);
		dma_async_issue_pending(ctlr->dma_tx);
	}

	if (xfer->rx_buf) {
		rx_desc = dmaengine_prep_slave_sg(ctlr->dma_rx, xfer->rx_sg.sgl,
						  xfer->rx_sg.nents, DMA_DEV_TO_MEM, 0);
		if (!rx_desc) {
			dev_err(drv->dev, "Unable to allocate RX DMA descriptor\n");
			goto error;
		}

		rx_desc->callback = adi_spi_rx_dma_isr;
		rx_desc->callback_param = drv;
		drv->rx_cookie = dmaengine_submit(rx_desc);
		iowrite32(SPI_RXCTL_REN | SPI_RXCTL_RTI | SPI_RXCTL_RDR_NE,
			  drv->regs + ADI_SPI_RXCTL);
		dma_async_issue_pending(ctlr->dma_rx);
	}

	return 1;

error:
	adi_spi_dma_terminate(drv);
	return -ENOENT;
}

static bool adi_spi_can_dma(struct spi_controller *ctlr, struct spi_device *spi,
			    struct spi_transfer *xfer)
{
	struct adi_spi_device *chip = spi_get_ctldata(spi);

	if (chip->dma)
		return true;
	return false;
}

static int adi_spi_transfer_one(struct spi_controller *ctlr, struct spi_device *spi,
				struct spi_transfer *xfer)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(ctlr);
	u32 cr;

	drv->cur_transfer = xfer;

	cr = ioread32(drv->regs + ADI_SPI_CTL) & ~SPI_CTL_MIOM;

	if (xfer->rx_nbits == SPI_NBITS_QUAD || xfer->tx_nbits == SPI_NBITS_QUAD)
		cr |= SPI_CTL_MIO_QUAD;
	else if (xfer->rx_nbits == SPI_NBITS_DUAL || xfer->tx_nbits == SPI_NBITS_DUAL)
		cr |= SPI_CTL_MIO_DUAL;

	iowrite32(cr, drv->regs + ADI_SPI_CTL);

	if (adi_spi_can_dma(ctlr, spi, xfer))
		return adi_spi_dma_xfer(ctlr, spi, xfer);
	return adi_spi_pio_xfer(ctlr, spi, xfer);
}

/*
 * Settings like clock speed and bits per word are assumed to be the same for all
 * transfers in a message. tx_nbits and rx_nbits can change, however
 */
static int adi_spi_prepare_message(struct spi_controller *ctlr, struct spi_message *msg)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(ctlr);
	struct adi_spi_device *chip = spi_get_ctldata(msg->spi);
	struct dma_slave_config dma_config = { 0 };
	struct spi_transfer *xfer;
	int ret;
	u32 cr, cr_width;
	u32 words;

	xfer = list_first_entry(&msg->transfers, struct spi_transfer, transfer_list);
	words = DIV_ROUND_UP(xfer->bits_per_word, 8);
	iowrite32(hz_to_spi_clock(drv->sclk_rate, xfer->speed_hz), drv->regs + ADI_SPI_CLK);

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
		dev_err(&ctlr->dev, "invalid word size in incoming message\n");
		return -EINVAL;
	}

	cr = chip->control;
	cr |= cr_width | SPI_CTL_EN;
	cr &= ~SPI_CTL_SOSI;
	iowrite32(cr, drv->regs + ADI_SPI_CTL);

	if (adi_spi_can_dma(ctlr, msg->spi, xfer)) {
		dma_config.direction = DMA_MEM_TO_DEV;
		dma_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		dma_config.src_maxburst = words;
		dma_config.dst_maxburst = words;
		ret = dmaengine_slave_config(ctlr->dma_tx, &dma_config);
		if (ret) {
			dev_err(drv->dev, "tx dma slave config failed: %d\n", ret);
			return ret;
		}

		dma_config.direction = DMA_DEV_TO_MEM;
		ret = dmaengine_slave_config(ctlr->dma_rx, &dma_config);
		if (ret) {
			dev_err(drv->dev, "rx dma slave config failed: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int adi_spi_unprepare_message(struct spi_controller *ctlr, struct spi_message *msg)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(ctlr);

	adi_spi_disable(drv);
	return 0;
}

static void adi_spi_set_cs(struct spi_device *spi, bool enable)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(spi->controller);
	u32 ssel;

	ssel = ioread32(drv->regs + ADI_SPI_SLVSEL);
	ssel |= SPI_SSE(spi_get_chipselect(spi, 0));

	if (enable)
		ssel |= SPI_SSEL(spi_get_chipselect(spi, 0));           /* CS deassert */
	else
		ssel &= ~SPI_SSEL(spi_get_chipselect(spi, 0));          /* CS assert */

	/* Required double write to get result on SLVSEL port */
	iowrite32(ssel, drv->regs + ADI_SPI_SLVSEL);
	iowrite32(ssel, drv->regs + ADI_SPI_SLVSEL);
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
	chip->control &= ~SPI_CTL_ASSEL;

	/* Fast Mode */
	chip->control |= SPI_CTL_FMODE;

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

	status = ioread32(drv_data->regs + ADI_SPI_STAT);
	dev_err(drv_data->dev, "spi error irq, status = 0x%x\n", status);
	iowrite32(status, drv_data->regs + ADI_SPI_STAT);

	iowrite32(0, drv_data->regs + ADI_SPI_TXCTL);
	iowrite32(0, drv_data->regs + ADI_SPI_RXCTL);
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

static int adi_qspi_adjust_op_size(struct spi_mem *mem, struct spi_mem_op *op)
{
	if (op->data.nbytes > MAX_SPI_TRANSFER_SIZE)
		op->data.nbytes = MAX_SPI_TRANSFER_SIZE;

	return 0;
}

static int adi_qspi_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	/* Once ops is defined, this callback needs to be present
	 * Do nothing and return error. Fallback will be the regular core
	 * spi_meme_exec_op implmentation */
	return -ENOTSUPP;
}

static const struct spi_controller_mem_ops adi_qspi_mem_ops = {
	.adjust_op_size = adi_qspi_adjust_op_size,
	.exec_op	= adi_qspi_exec_op,
};

static int adi_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *ctlr;
	struct adi_spi_controller *drv_data;
	struct resource *mem;
	struct clk *sclk;
	int ret;

	sclk = devm_clk_get(dev, "spi");
	if (IS_ERR(sclk)) {
		dev_err(dev, "can not get spi clock\n");
		return PTR_ERR(sclk);
	}

	ctlr = devm_spi_alloc_master(dev, sizeof(*drv_data));
	if (!ctlr) {
		dev_err(dev, "can not alloc spi_controller\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, ctlr);

	/* the mode bits supported by this driver */
	ctlr->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST |
			  SPI_TX_DUAL | SPI_TX_QUAD |
			  SPI_RX_DUAL | SPI_RX_QUAD;

	ctlr->dev.of_node = dev->of_node;
	ctlr->bus_num = -1;
	ctlr->num_chipselect = SPI_MAX_SS;
	ctlr->use_gpio_descriptors = true;
	ctlr->cleanup = adi_spi_cleanup;
	ctlr->setup = adi_spi_setup;
	ctlr->set_cs = adi_spi_set_cs;
	ctlr->prepare_message = adi_spi_prepare_message;
	ctlr->unprepare_message = adi_spi_unprepare_message;
	ctlr->transfer_one = adi_spi_transfer_one;
	ctlr->bits_per_word_mask = BIT(32 - 1) | BIT(16 - 1) | BIT(8 - 1);
	ctlr->max_native_cs = SPI_MAX_SS;
	ctlr->mem_ops = &adi_qspi_mem_ops;

	drv_data = spi_controller_get_devdata(ctlr);
	drv_data->ctlr = ctlr;
	drv_data->sclk = sclk;
	drv_data->sclk_rate = clk_get_rate(sclk);
	drv_data->dev = dev;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drv_data->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(drv_data->regs)) {
		dev_err(dev, "Could not map spiv3 memory, check device tree\n");
		return PTR_ERR(drv_data->regs);
	}

	drv_data->irq = platform_get_irq(pdev, 0);
	if (drv_data->irq <= 0) {
		ret = drv_data->irq ? drv_data->irq : -ENXIO;
		return ret;
	}
	ret = devm_request_irq(dev, drv_data->irq, spi_irq_err,
			       0, "SPI ERROR", drv_data);
	if (ret) {
		dev_err(dev, "can not request spi error irq\n");
		return ret;
	}

	iowrite32(SPI_CTL_MSTR | SPI_CTL_CPHA, drv_data->regs + ADI_SPI_CTL);
	iowrite32(0x0000FE00, drv_data->regs + ADI_SPI_SLVSEL);
	iowrite32(0x0, drv_data->regs + ADI_SPI_DLY);
	iowrite32(SPI_IMSK_SET_ROM, drv_data->regs + ADI_SPI_IMSK_SET);

	ctlr->dma_tx = dma_request_chan(dev, "tx");
	if (IS_ERR(ctlr->dma_tx)) {
		dev_warn(dev, "DMA TX channel not available, SPI unable to use DMA\n");
		ctlr->dma_tx = NULL;
	} else {
		ctlr->dma_rx = dma_request_chan(dev, "rx");
		if (IS_ERR(ctlr->dma_rx)) {
			dev_warn(dev, "DMA RX channel not available, SPI unable to use DMA\n");
			dma_release_channel(ctlr->dma_tx);
			ctlr->dma_tx = NULL;
			ctlr->dma_rx = NULL;
		}
	}

	if (ctlr->dma_rx && ctlr->dma_tx)
		ctlr->can_dma = adi_spi_can_dma;

	ret = clk_prepare_enable(drv_data->sclk);
	if (ret) {
		dev_err(dev, "Could not enable SPI clock\n");
		goto err_free_dma;
	}

	ret = devm_spi_register_controller(dev, ctlr);
	if (ret) {
		dev_err(dev, "cannot register spi controller\n");
		goto err_free_dma;
	}

	dev_info(dev, "registered ADI SPI controller %s\n",
		 dev_name(&ctlr->dev));
	return ret;

err_free_dma:
	if (ctlr->dma_tx) {
		dma_release_channel(ctlr->dma_rx);
		dma_release_channel(ctlr->dma_tx);
		ctlr->dma_tx = NULL;
		ctlr->dma_rx = NULL;
	}

	return ret;
}

static void adi_spi_remove(struct platform_device *pdev)
{
	struct spi_controller *ctlr = platform_get_drvdata(pdev);
	struct adi_spi_controller *drv_data = spi_controller_get_devdata(ctlr);

	adi_spi_disable(drv_data);
	clk_disable_unprepare(drv_data->sclk);

	if (ctlr->dma_tx)
		dma_release_channel(ctlr->dma_tx);
	if (ctlr->dma_rx)
		dma_release_channel(ctlr->dma_rx);
}

static int __maybe_unused adi_spi_suspend(struct device *dev)
{
	struct spi_controller *ctlr = dev_get_drvdata(dev);

	return spi_controller_suspend(ctlr);
}

static int __maybe_unused adi_spi_resume(struct device *dev)
{
	struct spi_controller *ctlr = dev_get_drvdata(dev);

	return spi_controller_resume(ctlr);
}

static const struct dev_pm_ops adi_spi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(adi_spi_suspend, adi_spi_resume)
};

MODULE_ALIAS("platform:adi-spi3");
static struct platform_driver adi_spi_driver = {
	.driver			= {
		.name		= "adi-spi3",
		.pm		= &adi_spi_pm_ops,
		.of_match_table = adi_spi_of_match,
	},
	.probe			= adi_spi_probe,
	.remove			= adi_spi_remove,
};

module_platform_driver(adi_spi_driver);

MODULE_DESCRIPTION("Analog Devices SPI3 controller driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
