// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices SPI-MM controller driver
 *
 * (C) Copyright 2024 - Analog Devices, Inc.
 *
 *
 * Contact: Utsav Agarwal <utsav.agarwal@analog.com> 
 *
 */

#include <linux/clk.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>

#define SPI2_REGS ((void *)0x31030000)
#define SPI2_MMAP ((void *)0x60000000)
#define LINUX_SRC (SPI2_MMAP + 0x01a0000)
#define LINUX_SZ (0x3E60000)
#define LOADADDR ((void *)0x90001000)

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
#define SPI_CTL_MMSE		    0x80000000	  /* Enable Memory Mapped Mode */
#define SPI_CTL_MMWEM		    0x40000000	  /* Memory Mapped Write Error Mask */

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



#define PR_ADI_SPI_REGISTER(DEV, REG_NAME) \
	dev_info(DEV, #REG_NAME "\t%08x\n", REG_NAME)

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

/* runtime info for spi controller */
struct adi_spi_controller {
	struct spi_controller *controller;
	struct device *dev;
	struct adi_spi_regs __iomem *regs;
	void __iomem *mmap;
	u32 control;
	u32 ssel;
	struct clk *sclk;
	unsigned long sclk_rate;
	bool mm_init;
	u32 curr_word_sz;
};

static const struct of_device_id adi_spi_mm_of_match[] = {
	{
		.compatible = "adi,spi-mm",
	},
	{},
};
MODULE_DEVICE_TABLE(of, adi_spi_mm_of_match);

static void adi_spi_mm_pr_regs(struct adi_spi_controller *mmspi)
{
	u32 control, delay, rx_control, tx_control;
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

static void adi_spi_mm_disable(struct adi_spi_controller *mmspi)
{
	u32 control;

	control = readl(&mmspi->regs->control);
	control &= ~(SPI_CTL_MMSE);
	writel(control, &mmspi->regs->control);
	writel(0x0, &mmspi->regs->mmrdh);
	writel(0x0, &mmspi->regs->mmtop);

	writel(0x0, &mmspi->regs->status);
	writel(0x0, &mmspi->regs->tx_control);
	writel(0x0, &mmspi->regs->rx_control);
	writel(0x0, &mmspi->regs->delay);
	writel(0x0000FE00, &mmspi->regs->ssel);
}

static void adi_spi_mm_cleanup(struct spi_device *spi)
{
	return;
}

static irqreturn_t spi_irq_err(int irq, void *dev_id)
{
	struct adi_spi_controller *mmspi = dev_id;

	adi_spi_mm_pr_regs(mmspi);
	writel(0, &mmspi->regs->tx_control);
	writel(0, &mmspi->regs->rx_control);
	adi_spi_mm_disable(mmspi);

	return IRQ_HANDLED;
}

/* Calculate the SPI_CLOCK register value based on input HZ */
static u32 hz_to_spi_clock(u32 sclk, u32 speed_hz)
{
	u32 spi_clock = sclk / speed_hz;

	if (spi_clock)
		spi_clock--;
	return spi_clock;
}

static int adi_spi_mm_dev_setup(struct spi_device *spi)
{

	u32 control;
	struct adi_spi_controller *mmspi = 
		spi_controller_get_devdata(spi->controller);
	/*
	 * Need to setup spi in normal mode here so that
	 * jedec ID can be read
	 * */

	//TODO: dont write to regs here 
	//
	writel(hz_to_spi_clock(mmspi->sclk_rate, spi->max_speed_hz), 
			&mmspi->regs->clock);

	control = readl(&mmspi->regs->control);


	if (spi->mode & SPI_CPOL)
		control |= SPI_CTL_CPOL;
	if (spi->mode & SPI_CPHA)
		control |= SPI_CTL_CPHA;
	if (spi->mode & SPI_LSB_FIRST)
		control |= SPI_CTL_LSBF;
	
	control |= SPI_CTL_MSTR;
	control &= ~SPI_CTL_ASSEL;
	writel(control, &mmspi->regs->control);

	return 0;
}

static void adi_spi_mm_cfg(struct adi_spi_controller *mmspi, struct spi_mem_op *op)
{
	writel(0x00000000, &mmspi->regs->control);
	writel(0x00000006, &mmspi->regs->clock);
	writel(0x00000005, &mmspi->regs->rx_control);  /*  Receive Control Register */
	writel(0x00000005, &mmspi->regs->tx_control);  /*  Transmit Control Register */
	writel(0x00000301, &mmspi->regs->delay);    /*  Delay Register */
	writel(0x0000FE02, &mmspi->regs->ssel); /*  Slave Select Register */
	writel(0x05003B03, &mmspi->regs->mmrdh);                    /*  Memory Mapped Read Header */
	writel(0x10000000, &mmspi->regs->mmtop);
	writel(0x80240073, &mmspi->regs->control);
}

static void adi_spi_set_word_size(u32 word, struct adi_spi_controller *mmspi) 
{
	u32 control;

	control = readl(&mmspi->regs->control);
	control &= ~SPI_CTL_SIZE; 
	switch(word) {
		case 1:
			control |= SPI_CTL_SIZE08;
			break;
		case 2:
			control |= SPI_CTL_SIZE16;
			break;
		case 3:
			control |= SPI_CTL_SIZE32;
			break;
		default:
			dev_err(mmspi->dev, "Word size %dB not supported\n",
					word);
	}

	dev_info(mmspi->dev, "DEBUG: Word size set to %dB \n", word);
	control &= ~SPI_CTL_SOSI;
	writel(control, &mmspi->regs->control);
	mmspi->curr_word_sz = word;
}

static u32 adi_spi_get_word_size(struct adi_spi_controller *mmspi)
{
	dev_info(mmspi->dev, "DEBUG: Current Word size %dB \n", 
			mmspi->curr_word_sz);
	return mmspi->curr_word_sz;
}

static void adi_spi_read(struct adi_spi_controller *mmspi,
		struct spi_mem_op *op)
{
	int i=0;
	u32 word;
	void *buf_ptr;

	if(op->data.dir != SPI_MEM_DATA_IN)
		return;

	word = adi_spi_get_word_size(mmspi);
	for (i = 0; i < op->data.nbytes; i++) {
		buf_ptr = op->data.buf.in + i*(word);
		
	//	while (readl(&mmspi->regs->status) & SPI_STAT_RFE)
	//		cpu_relax();
		
		switch (word) {
			case 1:
				*(u8 *)buf_ptr = readl(&mmspi->regs->rfifo);
				break;
			case 2:
				*(u16 *)buf_ptr = readl(&mmspi->regs->rfifo);
				break;
			default:
				*(u32 *)buf_ptr = readl(&mmspi->regs->rfifo);
		}
		
	}

	dev_info(mmspi->dev,"DEBUG: read buf(%d):%08x\n", op->data.nbytes, 
			*(u32*)op->data.buf.in);

}

static void adi_spi_send(void *buf, struct adi_spi_controller *mmspi, u32 len)
{
	u8 *buf_ptr;
	u32 word;
	int i;

	word = adi_spi_get_word_size(mmspi);

	writel(SPI_TXCTL_TEN | SPI_TXCTL_TTI, &mmspi->regs->tx_control);
	writel(SPI_RXCTL_REN, &mmspi->regs->rx_control);

	
	for (i = 0; i < len; i++) {
		buf_ptr = buf + i*(word);

		dev_info(mmspi->dev,"DEBUG: write buf(%d):%08x\n", len, *(u32*)buf_ptr);
		switch (word) {
			case 1:
				writel(*(u8*)buf_ptr, &mmspi->regs->tfifo);
				break;
			case 2:
				writel(*(u16*)buf_ptr, &mmspi->regs->tfifo);
				break;
			default:
				writel(*(u32*)buf_ptr, &mmspi->regs->tfifo);
		}

		dev_info(mmspi->dev,"DEBUG: tfifo buf(%d):%08x\n", len, readl(&mmspi->regs->tfifo));
		
	//	while (readl(&mmspi->regs->status) & SPI_STAT_RFE)
	//		cpu_relax();
		//readl(&mmspi->regs->rfifo);
	}
}

static void adi_spi_write(struct adi_spi_controller *mmspi, 
		struct spi_mem_op *op)
{
	u8 *tmpbuf;
	u32 tmpbufsize, word_size, buf_len;

	tmpbufsize = op->cmd.nbytes + op->addr.nbytes + op->dummy.nbytes;
	if (op->data.dir == SPI_MEM_DATA_OUT)
		tmpbufsize += op->data.nbytes;
	
	tmpbuf = kzalloc(tmpbufsize, GFP_KERNEL);
	if (!tmpbuf) {
		dev_err(mmspi->dev, "Could not allocate memory\n");
		return;
	}

	dev_info(mmspi->dev,"DEBUG: opcode: %02x\n", op->cmd.opcode);

	tmpbuf[0] = op->cmd.opcode;
	word_size = op->cmd.buswidth;
	buf_len = 1;
	
	/*--------SEND-----------*/
	adi_spi_set_word_size(word_size, mmspi);
	adi_spi_send(tmpbuf, mmspi, buf_len);
	
	/*
	 * ***NOTE:control's word size can be done with op->cmd.buswidth
	 * xfers[xferpos].tx_nbits = op->addr.buswidth;
	 * */
	if (op->addr.nbytes) {
		int i;

		for (i = 0; i < op->addr.nbytes; i++)
			tmpbuf[i + 1] = op->addr.val >>
					(8 * (op->addr.nbytes - i - 1));

		word_size = op->addr.buswidth;
		buf_len = op->addr.nbytes;
	
		/*--------SEND-----------*/
		adi_spi_set_word_size(word_size, mmspi);
		adi_spi_send(tmpbuf + 1, mmspi, buf_len);
	}

	if (op->dummy.nbytes) {
		memset(tmpbuf + op->addr.nbytes + 1, 0xff, op->dummy.nbytes);
		word_size = op->dummy.buswidth;
		buf_len = op->dummy.nbytes;
	
		/*--------SEND-----------*/
		adi_spi_set_word_size(word_size, mmspi);
		adi_spi_send(tmpbuf + 1 + op->addr.nbytes, mmspi, buf_len);
	}


	if ((op->data.nbytes) && (op->data.dir == SPI_MEM_DATA_OUT)) {
		memcpy(tmpbuf + buf_len, op->data.buf.out, op->data.nbytes);
		word_size = op->data.buswidth;
		buf_len += op->data.nbytes;

		/*--------SEND-----------*/
		adi_spi_set_word_size(word_size, mmspi);
		adi_spi_send(tmpbuf + 1 + op->addr.nbytes + op->dummy.nbytes, 
				mmspi, buf_len);
	}

	kfree(tmpbuf);
}

static int adi_spi_mm_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct adi_spi_controller *mmspi = 
		spi_controller_get_devdata(mem->spi->controller);
	

	adi_spi_write(mmspi, op);

	adi_spi_mm_pr_regs(mmspi);
	if(op->data.dir == SPI_MEM_DATA_IN) {
		adi_spi_read(mmspi, op);
	
		adi_spi_mm_pr_regs(mmspi);
	}

	writel(0, &mmspi->regs->tx_control);
	writel(0, &mmspi->regs->rx_control);
	return 0;
}

static bool adi_spi_mm_supports_op(struct spi_mem *mem, 
		const struct spi_mem_op *op)
{
	if (!spi_mem_default_supports_op(mem, op))
		return false;

	return true;
}

static const char *adi_spi_mm_getname(struct spi_mem *spi_mem)
{
	return dev_name(&spi_mem->spi->dev);
}

static const struct spi_controller_mem_ops adi_spi_mm_memops = {
	.exec_op = adi_spi_mm_exec_op,
	.supports_op = adi_spi_mm_supports_op,
	.get_name = adi_spi_mm_getname
};

static int adi_spi_mm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *controller;
	struct adi_spi_controller *mmspi;
	struct resource *mem;
	int ret;
	struct clk *sclk;
	struct irq_domain *irq_domain;
	struct device_node *irq_of_node;
	struct irq_fwspec spi_fwspec = {
	    .param_count = 3,
	    .param = {0, 132, 4},
	};
	int virq;
	u32 control;
	
	sclk = devm_clk_get(dev, "spi");
	if (IS_ERR(sclk)) {
		dev_err(dev, "can not get spi clock\n");
		return PTR_ERR(sclk);
	}

	//controller can only be a master in this mode 
	controller = devm_spi_alloc_master(dev, sizeof(*mmspi));
	if (!controller) {
		dev_err(dev, "can not alloc spi_controller\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, controller);
	controller->dev.of_node = dev->of_node;
	controller->mode_bits =  SPI_LSB_FIRST | SPI_CPHA | SPI_CPOL |
		SPI_RX_DUAL | SPI_RX_QUAD | SPI_TX_DUAL | SPI_TX_QUAD;
	controller->mem_ops = &adi_spi_mm_memops;
	controller->bus_num = -1;
	controller->num_chipselect = 4;
	controller->cleanup = adi_spi_mm_cleanup;
	controller->setup = adi_spi_mm_dev_setup;
	controller->bits_per_word_mask = BIT(32 - 1) | BIT(16 - 1) | BIT(8 - 1);

	mmspi = spi_controller_get_devdata(controller);
	mmspi->controller = controller;
	mmspi->dev = dev;
	mmspi->sclk = sclk;
	mmspi->sclk_rate = clk_get_rate(sclk);

	//get the register io via dts
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mmspi->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(mmspi->regs)) {
		dev_err(dev, "Could not map spi regs, check device tree\n");
		return PTR_ERR(mmspi->regs);
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
	ret = devm_request_irq(dev, virq, spi_irq_err, 0, "SPI ERROR", mmspi);
	if (ret) {
		dev_err(dev, "could not request spi error irq\n");
		return ret;
	}

	ret = clk_prepare_enable(mmspi->sclk);
	if (ret) {
		dev_err(dev, "Could not enable SPI clock\n");\
		return ret;
	}

	writel(0x0000FE02, &mmspi->regs->ssel);
	writel(0x0, &mmspi->regs->delay);
	control = SPI_CTL_MSTR;

	control &= ~SPI_CTL_MIOM;
	control &= ~SPI_CTL_SOSI;
	control |= SPI_CTL_EN;
	writel(control, &mmspi->regs->control);

	/*this only supports reads directly to an address*/
	mmspi->mm_init = false;
	
	adi_spi_mm_pr_regs(mmspi);
	dev_info(mmspi->dev, "SPI init config done\n");

	ret = devm_spi_register_controller(dev, controller);
	if (ret) {
		dev_err(dev, "can not register spi controller\n");
		return ret;
	}

	return 0;
}

static int adi_spi_mm_remove(struct platform_device *pdev) 
{
	struct adi_spi_controller *mmspi = platform_get_drvdata(pdev);
	adi_spi_mm_disable(mmspi);

	return 0;
}

static int __maybe_unused adi_spi_mm_suspend(struct device *dev)
{
	struct spi_controller *controller = dev_get_drvdata(dev);

	return spi_controller_suspend(controller);
}

static int __maybe_unused adi_spi_mm_resume(struct device *dev)
{
	struct spi_controller *controller = dev_get_drvdata(dev);

	return spi_controller_resume(controller);
}

static const struct dev_pm_ops adi_spi_mm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(adi_spi_mm_suspend, adi_spi_mm_resume)
};

MODULE_ALIAS("platform:adi-spi-mm");
static struct platform_driver adi_spi_mm_driver = {
	.driver	= {
		.name	= "adi-spi-mm",
		.pm     = &adi_spi_mm_pm_ops,
		.of_match_table = adi_spi_mm_of_match,
	},
	.probe      = adi_spi_mm_probe,
	.remove		= adi_spi_mm_remove,
};

module_platform_driver(adi_spi_mm_driver);

MODULE_DESCRIPTION("Analog Devices SPI MMAP controller driver");
MODULE_AUTHOR("Utsav Agarwal <utsav.agarwal@analog.com>");
MODULE_LICENSE("GPL v2");
