/*
 * Driver for Cadence QSPI Controller
 *
 * Copyright Altera Corporation (C) 2012-2014. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/timer.h>

#define CQSPI_NAME			"cadence-qspi"
#define CQSPI_MAX_CHIPSELECT		16

struct cqspi_flash_pdata {
	struct mtd_info mtd;
	struct spi_nor nor;
	u32 clk_rate;
	unsigned int page_size;
	unsigned int block_size;
	unsigned int read_delay;
	unsigned int tshsl_ns;
	unsigned int tsd2d_ns;
	unsigned int tchsh_ns;
	unsigned int tslch_ns;
	int quad_mode_set;
};

struct cqspi_st {
	struct platform_device *pdev;

	struct clk *clk;
	unsigned int sclk;

	void __iomem *iobase;
	void __iomem *ahb_base;
	struct completion transfer_complete;
	unsigned int irq_mask;
	int current_cs;
	unsigned int ahb_phy_addr;
	unsigned int master_ref_clk_hz;
	u32 is_decoded_cs;
	unsigned int fifo_depth;
	struct cqspi_flash_pdata f_pdata[CQSPI_MAX_CHIPSELECT];
};

/* Operation timeout value */
#define CQSPI_TIMEOUT_MS			500
#define CQSPI_READ_TIMEOUT_MS			10
#define CQSPI_POLL_IDLE_RETRY			3

#define CQSPI_FIFO_WIDTH			4

#define CQSPI_REG_SRAM_THRESHOLD_BYTES		50

/* Instruction type */
#define CQSPI_INST_TYPE_SINGLE			0
#define CQSPI_INST_TYPE_DUAL			1
#define CQSPI_INST_TYPE_QUAD			2

#define CQSPI_DUMMY_CLKS_PER_BYTE		8
#define CQSPI_DUMMY_BYTES_MAX			4

#define CQSPI_STIG_DATA_LEN_MAX			8

#define CQSPI_INDIRECTTRIGGER_ADDR_MASK		0xFFFFF

/* Register map */
#define CQSPI_REG_CONFIG			0x00
#define CQSPI_REG_CONFIG_ENABLE_MASK		BIT(0)
#define CQSPI_REG_CONFIG_DECODE_MASK		BIT(9)
#define CQSPI_REG_CONFIG_CHIPSELECT_LSB		10
#define CQSPI_REG_CONFIG_DMA_MASK		BIT(15)
#define CQSPI_REG_CONFIG_BAUD_LSB		19
#define CQSPI_REG_CONFIG_IDLE_LSB		31
#define CQSPI_REG_CONFIG_CHIPSELECT_MASK	0xF
#define CQSPI_REG_CONFIG_BAUD_MASK		0xF

#define CQSPI_REG_RD_INSTR			0x04
#define CQSPI_REG_RD_INSTR_OPCODE_LSB		0
#define CQSPI_REG_RD_INSTR_TYPE_INSTR_LSB	8
#define CQSPI_REG_RD_INSTR_TYPE_ADDR_LSB	12
#define CQSPI_REG_RD_INSTR_TYPE_DATA_LSB	16
#define CQSPI_REG_RD_INSTR_MODE_EN_LSB		20
#define CQSPI_REG_RD_INSTR_DUMMY_LSB		24
#define CQSPI_REG_RD_INSTR_TYPE_INSTR_MASK	0x3
#define CQSPI_REG_RD_INSTR_TYPE_ADDR_MASK	0x3
#define CQSPI_REG_RD_INSTR_TYPE_DATA_MASK	0x3
#define CQSPI_REG_RD_INSTR_DUMMY_MASK		0x1F

#define CQSPI_REG_WR_INSTR			0x08
#define CQSPI_REG_WR_INSTR_OPCODE_LSB		0
#define CQSPI_REG_WR_INSTR_TYPE_ADDR_LSB	12
#define CQSPI_REG_WR_INSTR_TYPE_DATA_LSB	16

#define CQSPI_REG_DELAY				0x0C
#define CQSPI_REG_DELAY_TSLCH_LSB		0
#define CQSPI_REG_DELAY_TCHSH_LSB		8
#define CQSPI_REG_DELAY_TSD2D_LSB		16
#define CQSPI_REG_DELAY_TSHSL_LSB		24
#define CQSPI_REG_DELAY_TSLCH_MASK		0xFF
#define CQSPI_REG_DELAY_TCHSH_MASK		0xFF
#define CQSPI_REG_DELAY_TSD2D_MASK		0xFF
#define CQSPI_REG_DELAY_TSHSL_MASK		0xFF

#define CQSPI_REG_READCAPTURE			0x10
#define CQSPI_REG_READCAPTURE_BYPASS_LSB	0
#define CQSPI_REG_READCAPTURE_DELAY_LSB		1
#define CQSPI_REG_READCAPTURE_DELAY_MASK	0xF

#define CQSPI_REG_SIZE				0x14
#define CQSPI_REG_SIZE_ADDRESS_LSB		0
#define CQSPI_REG_SIZE_PAGE_LSB			4
#define CQSPI_REG_SIZE_BLOCK_LSB		16
#define CQSPI_REG_SIZE_ADDRESS_MASK		0xF
#define CQSPI_REG_SIZE_PAGE_MASK		0xFFF
#define CQSPI_REG_SIZE_BLOCK_MASK		0x3F

#define CQSPI_REG_SRAMPARTITION			0x18
#define CQSPI_REG_INDIRECTTRIGGER		0x1C

#define CQSPI_REG_DMA				0x20
#define CQSPI_REG_DMA_SINGLE_LSB		0
#define CQSPI_REG_DMA_BURST_LSB			8
#define CQSPI_REG_DMA_SINGLE_MASK		0xFF
#define CQSPI_REG_DMA_BURST_MASK		0xFF

#define CQSPI_REG_REMAP				0x24
#define CQSPI_REG_MODE_BIT			0x28

#define CQSPI_REG_SDRAMLEVEL			0x2C
#define CQSPI_REG_SDRAMLEVEL_RD_LSB		0
#define CQSPI_REG_SDRAMLEVEL_WR_LSB		16
#define CQSPI_REG_SDRAMLEVEL_RD_MASK		0xFFFF
#define CQSPI_REG_SDRAMLEVEL_WR_MASK		0xFFFF

#define CQSPI_REG_IRQSTATUS			0x40
#define CQSPI_REG_IRQMASK			0x44

#define CQSPI_REG_INDIRECTRD			0x60
#define CQSPI_REG_INDIRECTRD_START_MASK		BIT(0)
#define CQSPI_REG_INDIRECTRD_CANCEL_MASK	BIT(1)
#define CQSPI_REG_INDIRECTRD_DONE_MASK		BIT(5)

#define CQSPI_REG_INDIRECTRDWATERMARK		0x64
#define CQSPI_REG_INDIRECTRDSTARTADDR		0x68
#define CQSPI_REG_INDIRECTRDBYTES		0x6C

#define CQSPI_REG_CMDCTRL			0x90
#define CQSPI_REG_CMDCTRL_EXECUTE_MASK		BIT(0)
#define CQSPI_REG_CMDCTRL_INPROGRESS_MASK	BIT(1)
#define CQSPI_REG_CMDCTRL_WR_BYTES_LSB		12
#define CQSPI_REG_CMDCTRL_WR_EN_LSB		15
#define CQSPI_REG_CMDCTRL_ADD_BYTES_LSB		16
#define CQSPI_REG_CMDCTRL_ADDR_EN_LSB		19
#define CQSPI_REG_CMDCTRL_RD_BYTES_LSB		20
#define CQSPI_REG_CMDCTRL_RD_EN_LSB		23
#define CQSPI_REG_CMDCTRL_OPCODE_LSB		24
#define CQSPI_REG_CMDCTRL_WR_BYTES_MASK		0x7
#define CQSPI_REG_CMDCTRL_ADD_BYTES_MASK	0x3
#define CQSPI_REG_CMDCTRL_RD_BYTES_MASK		0x7

#define CQSPI_REG_INDIRECTWR			0x70
#define CQSPI_REG_INDIRECTWR_START_MASK		BIT(0)
#define CQSPI_REG_INDIRECTWR_CANCEL_MASK	BIT(1)
#define CQSPI_REG_INDIRECTWR_DONE_MASK		BIT(5)

#define CQSPI_REG_INDIRECTWRWATERMARK		0x74
#define CQSPI_REG_INDIRECTWRSTARTADDR		0x78
#define CQSPI_REG_INDIRECTWRBYTES		0x7C

#define CQSPI_REG_CMDADDRESS			0x94
#define CQSPI_REG_CMDREADDATALOWER		0xA0
#define CQSPI_REG_CMDREADDATAUPPER		0xA4
#define CQSPI_REG_CMDWRITEDATALOWER		0xA8
#define CQSPI_REG_CMDWRITEDATAUPPER		0xAC

/* Interrupt status bits */
#define CQSPI_REG_IRQ_MODE_ERR			BIT(0)
#define CQSPI_REG_IRQ_UNDERFLOW			BIT(1)
#define CQSPI_REG_IRQ_IND_COMP			BIT(2)
#define CQSPI_REG_IRQ_IND_RD_REJECT		BIT(3)
#define CQSPI_REG_IRQ_WR_PROTECTED_ERR		BIT(4)
#define CQSPI_REG_IRQ_ILLEGAL_AHB_ERR		BIT(5)
#define CQSPI_REG_IRQ_WATERMARK			BIT(6)
#define CQSPI_REG_IRQ_IND_RD_OVERFLOW		BIT(12)

#define CQSPI_IRQ_STATUS_ERR		(CQSPI_REG_IRQ_MODE_ERR		| \
					 CQSPI_REG_IRQ_IND_RD_REJECT	| \
					 CQSPI_REG_IRQ_WR_PROTECTED_ERR	| \
					 CQSPI_REG_IRQ_ILLEGAL_AHB_ERR)

#define CQSPI_IRQ_MASK_RD		(CQSPI_REG_IRQ_WATERMARK	| \
					 CQSPI_REG_IRQ_IND_RD_OVERFLOW	| \
					 CQSPI_REG_IRQ_IND_COMP)

#define CQSPI_IRQ_MASK_WR		(CQSPI_REG_IRQ_IND_COMP		| \
					 CQSPI_REG_IRQ_WATERMARK	| \
					 CQSPI_REG_IRQ_UNDERFLOW)

#define CQSPI_IRQ_STATUS_MASK		0x1FFFF

#define CQSPI_REG_IS_IDLE(base)						\
		((readl(base + CQSPI_REG_CONFIG) >>		\
			CQSPI_REG_CONFIG_IDLE_LSB) & 0x1)

#define CQSPI_GET_RD_SRAM_LEVEL(reg_base)				\
		(((readl(reg_base + CQSPI_REG_SDRAMLEVEL)) >>	\
		CQSPI_REG_SDRAMLEVEL_RD_LSB) & CQSPI_REG_SDRAMLEVEL_RD_MASK)

static unsigned int cqspi_init_timeout(unsigned long timeout_in_ms)
{
	return jiffies + msecs_to_jiffies(timeout_in_ms);
}

static unsigned int cqspi_check_timeout(unsigned long timeout)
{
	return time_before(jiffies, timeout);
}

static void cqspi_fifo_read(void *dest, const void *src_ahb_addr,
			    unsigned int bytes)
{
	unsigned int temp;
	int remaining = bytes;
	unsigned int *dest_ptr = (unsigned int *)dest;
	unsigned int *src_ptr = (unsigned int *)src_ahb_addr;

	while (remaining > CQSPI_FIFO_WIDTH) {
		*dest_ptr = readl(src_ptr);
		dest_ptr++;
		remaining -= CQSPI_FIFO_WIDTH;
	}
	if (remaining > 0) {
		/* dangling bytes */
		temp = readl(src_ptr);
		memcpy(dest_ptr, &temp, remaining);
	}
}

static void cqspi_fifo_write(void *dest_ahb_addr,
			     const void *src, unsigned int bytes)
{
	unsigned int temp;
	int remaining = bytes;
	unsigned int *dest_ptr = (unsigned int *)dest_ahb_addr;
	unsigned int *src_ptr = (unsigned int *)src;

	while (remaining > CQSPI_FIFO_WIDTH) {
		writel(*src_ptr, dest_ptr);
		src_ptr++;
		remaining -= CQSPI_FIFO_WIDTH;
	}
	if (remaining > 0) {
		/* dangling bytes */
		memcpy(&temp, src_ptr, remaining);
		writel(temp, dest_ptr);
	}
}

static irqreturn_t cqspi_irq_handler(int this_irq, void *dev)
{
	struct cqspi_st *cqspi = dev;
	unsigned int irq_status;

	/* Read interrupt status */
	irq_status = readl(cqspi->iobase + CQSPI_REG_IRQSTATUS);

	/* Clear interrupt */
	writel(irq_status, cqspi->iobase + CQSPI_REG_IRQSTATUS);

	if (irq_status & cqspi->irq_mask)
		complete(&cqspi->transfer_complete);

	return IRQ_HANDLED;
}

static int cqspi_find_chipselect(struct spi_nor *nor)
{
	int cs = -1;
	struct cqspi_st *cqspi = nor->priv;

	for (cs = 0; cs < CQSPI_MAX_CHIPSELECT; cs++)
		if (nor == &cqspi->f_pdata[cs].nor)
			break;
	return cs;
}

static unsigned int cqspi_calc_rdreg(struct spi_nor *nor, u8 opcode)
{
	unsigned int rdreg = 0;
	struct cqspi_st *cqspi = nor->priv;
	struct cqspi_flash_pdata *f_pdata;

	f_pdata = &cqspi->f_pdata[cqspi->current_cs];

	if (f_pdata->quad_mode_set)
		rdreg |= (CQSPI_INST_TYPE_QUAD
			<< CQSPI_REG_RD_INSTR_TYPE_INSTR_LSB);
	else if (nor->flash_read == SPI_NOR_QUAD)
		rdreg |= (CQSPI_INST_TYPE_QUAD
			  << CQSPI_REG_RD_INSTR_TYPE_DATA_LSB);
	return rdreg;
}

static unsigned int cqspi_wait_idle(struct cqspi_st *cqspi)
{
	void __iomem *reg_base = cqspi->iobase;
	unsigned int count = 0;
	unsigned timeout;

	timeout = cqspi_init_timeout(CQSPI_TIMEOUT_MS);
	while (cqspi_check_timeout(timeout)) {
		if (CQSPI_REG_IS_IDLE(reg_base)) {
			/*
			 * Read few times in succession to ensure it does
			 * not transition low again
			 */
			count++;
			if (count >= CQSPI_POLL_IDLE_RETRY)
				return 1;
		} else {
			count = 0;
		}
		cpu_relax();
	}

	/* Timeout, in busy mode. */
	dev_err(&cqspi->pdev->dev, "QSPI is still busy after %dms timeout.\n",
		CQSPI_TIMEOUT_MS);
	return 0;
}

static int cqspi_exec_flash_cmd(struct cqspi_st *cqspi, unsigned int reg)
{
	void __iomem *reg_base = cqspi->iobase;
	unsigned int timeout;

	/* Write the CMDCTRL without start execution. */
	writel(reg, reg_base + CQSPI_REG_CMDCTRL);
	/* Start execute */
	reg |= CQSPI_REG_CMDCTRL_EXECUTE_MASK;
	writel(reg, reg_base + CQSPI_REG_CMDCTRL);

	/* Polling for completion. */
	timeout = cqspi_init_timeout(CQSPI_TIMEOUT_MS);
	while (cqspi_check_timeout(timeout)) {
		reg = readl(reg_base + CQSPI_REG_CMDCTRL) &
		    CQSPI_REG_CMDCTRL_INPROGRESS_MASK;
		if (!reg)
			break;
	}

	if (reg != 0) {
		dev_err(&cqspi->pdev->dev, "flash cmd execute time out\n");
		return -EIO;
	}

	/* Polling QSPI idle status. */
	if (!cqspi_wait_idle(cqspi))
		return -EIO;

	return 0;
}

static int cqspi_command_read(struct spi_nor *nor,
			      const u8 *txbuf, unsigned n_tx,
			      u8 *rxbuf, unsigned n_rx)
{
	unsigned int reg;
	unsigned int read_len;
	int status;
	struct cqspi_st *cqspi = nor->priv;
	void __iomem *reg_base = cqspi->iobase;
	unsigned int rdreg;

	if (!n_rx || n_rx > CQSPI_STIG_DATA_LEN_MAX || !rxbuf) {
		dev_err(nor->dev,
			"Invalid input argument, len %d rxbuf 0x%08x\n", n_rx,
			(unsigned int)rxbuf);
		return -EINVAL;
	}

	reg = txbuf[0] << CQSPI_REG_CMDCTRL_OPCODE_LSB;

	rdreg = cqspi_calc_rdreg(nor, txbuf[0]);
	writel(rdreg, reg_base + CQSPI_REG_RD_INSTR);

	reg |= (0x1 << CQSPI_REG_CMDCTRL_RD_EN_LSB);

	/* 0 means 1 byte. */
	reg |= (((n_rx - 1) & CQSPI_REG_CMDCTRL_RD_BYTES_MASK)
		<< CQSPI_REG_CMDCTRL_RD_BYTES_LSB);
	status = cqspi_exec_flash_cmd(cqspi, reg);
	if (status != 0)
		return status;

	reg = readl(reg_base + CQSPI_REG_CMDREADDATALOWER);

	/* Put the read value into rx_buf */
	read_len = (n_rx > 4) ? 4 : n_rx;
	memcpy(rxbuf, &reg, read_len);
	rxbuf += read_len;

	if (n_rx > 4) {
		reg = readl(reg_base + CQSPI_REG_CMDREADDATAUPPER);

		read_len = n_rx - read_len;
		memcpy(rxbuf, &reg, read_len);
	}

	return 0;
}

static int cqspi_command_write(struct spi_nor *nor,
			       u8 opcode, const u8 *txbuf, unsigned n_tx)
{
	unsigned int reg;
	unsigned int data;
	int ret;
	struct cqspi_st *cqspi = nor->priv;
	void __iomem *reg_base = cqspi->iobase;

	if (n_tx > 4 || (n_tx && !txbuf)) {
		dev_err(nor->dev,
			"Invalid input argument, cmdlen %d txbuf 0x%08x\n",
			n_tx, (unsigned int)txbuf);
		return -EINVAL;
	}

	reg = opcode << CQSPI_REG_CMDCTRL_OPCODE_LSB;
	if (n_tx) {
		reg |= (0x1 << CQSPI_REG_CMDCTRL_WR_EN_LSB);
		reg |= ((n_tx - 1) & CQSPI_REG_CMDCTRL_WR_BYTES_MASK)
		    << CQSPI_REG_CMDCTRL_WR_BYTES_LSB;
		data = 0;
		memcpy(&data, txbuf, n_tx);
		writel(data, reg_base + CQSPI_REG_CMDWRITEDATALOWER);
	}

	ret = cqspi_exec_flash_cmd(cqspi, reg);
	/*
	 * Hack it up, watch for Micron quad mode command so that
	 * we use proper controller setup for next command
	 */
	if (n_tx && opcode == SPINOR_OP_WD_EVCR &&
	    !(txbuf[0] & EVCR_QUAD_EN_MICRON)) {
		struct cqspi_flash_pdata *f_pdata;

		f_pdata = &cqspi->f_pdata[cqspi->current_cs];
		f_pdata->quad_mode_set = 1;
	}
	return ret;
}

static int cqspi_command_write_addr(struct spi_nor *nor,
				    u8 opcode, unsigned int addr)
{
	unsigned int reg;
	struct cqspi_st *cqspi = nor->priv;
	void __iomem *reg_base = cqspi->iobase;

	reg = opcode << CQSPI_REG_CMDCTRL_OPCODE_LSB;
	reg |= (0x1 << CQSPI_REG_CMDCTRL_ADDR_EN_LSB);
	reg |= ((nor->addr_width - 1) & CQSPI_REG_CMDCTRL_ADD_BYTES_MASK)
	    << CQSPI_REG_CMDCTRL_ADD_BYTES_LSB;

	writel(addr, reg_base + CQSPI_REG_CMDADDRESS);

	return cqspi_exec_flash_cmd(cqspi, reg);
}

static int cqspi_indirect_read_setup(struct spi_nor *nor,
				     unsigned int from_addr)
{
	unsigned int reg;
	unsigned int dummy_clk = 0;
	unsigned int dummy_bytes;
	struct cqspi_st *cqspi = nor->priv;
	void __iomem *reg_base = cqspi->iobase;
	unsigned int ahb_phy_addr = cqspi->ahb_phy_addr;

	writel((ahb_phy_addr & CQSPI_INDIRECTTRIGGER_ADDR_MASK),
	       reg_base + CQSPI_REG_INDIRECTTRIGGER);
	writel(from_addr, reg_base + CQSPI_REG_INDIRECTRDSTARTADDR);

	reg = nor->read_opcode << CQSPI_REG_RD_INSTR_OPCODE_LSB;
	reg |= cqspi_calc_rdreg(nor, nor->read_opcode);

	/* Setup dummy clock cycles */
	dummy_bytes = nor->read_dummy / 8;
	if (dummy_bytes) {
		struct cqspi_flash_pdata *f_pdata;

		f_pdata = &cqspi->f_pdata[cqspi->current_cs];

		if (dummy_bytes > CQSPI_DUMMY_BYTES_MAX)
			dummy_bytes = CQSPI_DUMMY_BYTES_MAX;

		reg |= (1 << CQSPI_REG_RD_INSTR_MODE_EN_LSB);
		/* Set mode bits high to ensure chip doesn't enter XIP */
		writel(0xFF, reg_base + CQSPI_REG_MODE_BIT);

		dummy_clk = dummy_bytes * CQSPI_DUMMY_CLKS_PER_BYTE;
		if (!f_pdata->quad_mode_set) {
			/* Need to subtract the mode byte (8 clocks). */
			dummy_clk -= CQSPI_DUMMY_CLKS_PER_BYTE;
		}

		if (dummy_clk)
			reg |= (dummy_clk & CQSPI_REG_RD_INSTR_DUMMY_MASK)
			    << CQSPI_REG_RD_INSTR_DUMMY_LSB;
	}

	writel(reg, reg_base + CQSPI_REG_RD_INSTR);

	/* Set address width */
	reg = readl(reg_base + CQSPI_REG_SIZE);
	reg &= ~CQSPI_REG_SIZE_ADDRESS_MASK;
	reg |= (nor->addr_width - 1);
	writel(reg, reg_base + CQSPI_REG_SIZE);
	return 0;
}

static int cqspi_indirect_read_execute(struct spi_nor *nor,
				       u8 *rxbuf, unsigned n_rx)
{
	int ret = 0;
	unsigned int reg = 0;
	unsigned int bytes_to_read = 0;
	unsigned int timeout;
	unsigned int watermark;
	struct cqspi_st *cqspi = nor->priv;
	void __iomem *reg_base = cqspi->iobase;
	void __iomem *ahb_base = cqspi->ahb_base;
	int remaining = (int)n_rx;

	watermark = cqspi->fifo_depth * CQSPI_FIFO_WIDTH / 2;
	writel(watermark, reg_base + CQSPI_REG_INDIRECTRDWATERMARK);
	writel(remaining, reg_base + CQSPI_REG_INDIRECTRDBYTES);

	/* Clear all interrupts. */
	writel(CQSPI_IRQ_STATUS_MASK, reg_base + CQSPI_REG_IRQSTATUS);

	cqspi->irq_mask = CQSPI_IRQ_MASK_RD;
	writel(cqspi->irq_mask, reg_base + CQSPI_REG_IRQMASK);

	reinit_completion(&cqspi->transfer_complete);
	writel(CQSPI_REG_INDIRECTRD_START_MASK,
	       reg_base + CQSPI_REG_INDIRECTRD);

	while (remaining > 0) {
		ret = wait_for_completion_timeout(&cqspi->transfer_complete,
						  msecs_to_jiffies
						  (CQSPI_READ_TIMEOUT_MS));

		bytes_to_read = CQSPI_GET_RD_SRAM_LEVEL(reg_base);

		if (!ret && bytes_to_read == 0) {
			dev_err(nor->dev, "Indirect read timeout, no bytes\n");
			ret = -ETIMEDOUT;
			goto failrd;
		}

		while (bytes_to_read != 0) {
			bytes_to_read *= CQSPI_FIFO_WIDTH;
			bytes_to_read = bytes_to_read > remaining
			    ? remaining : bytes_to_read;
			cqspi_fifo_read(rxbuf, ahb_base, bytes_to_read);
			rxbuf += bytes_to_read;
			remaining -= bytes_to_read;
			bytes_to_read = CQSPI_GET_RD_SRAM_LEVEL(reg_base);
		}
	}

	/* Check indirect done status */
	timeout = cqspi_init_timeout(CQSPI_TIMEOUT_MS);
	while (cqspi_check_timeout(timeout)) {
		reg = readl(reg_base + CQSPI_REG_INDIRECTRD);
		if (reg & CQSPI_REG_INDIRECTRD_DONE_MASK)
			break;
	}

	if (!(reg & CQSPI_REG_INDIRECTRD_DONE_MASK)) {
		dev_err(nor->dev,
			"Indirect read completion error 0x%08x\n", reg);
		ret = -ETIMEDOUT;
		goto failrd;
	}

	/* Disable interrupt */
	writel(0, reg_base + CQSPI_REG_IRQMASK);

	/* Clear indirect completion status */
	writel(CQSPI_REG_INDIRECTRD_DONE_MASK, reg_base + CQSPI_REG_INDIRECTRD);

	return 0;

 failrd:
	/* Disable interrupt */
	writel(0, reg_base + CQSPI_REG_IRQMASK);

	/* Cancel the indirect read */
	writel(CQSPI_REG_INDIRECTWR_CANCEL_MASK,
	       reg_base + CQSPI_REG_INDIRECTRD);
	return ret;
}

static int cqspi_indirect_write_setup(struct spi_nor *nor, unsigned int to_addr)
{
	unsigned int reg;
	struct cqspi_st *cqspi = nor->priv;
	void __iomem *reg_base = cqspi->iobase;

	/* Set opcode. */
	reg = nor->program_opcode << CQSPI_REG_WR_INSTR_OPCODE_LSB;
	writel(reg, reg_base + CQSPI_REG_WR_INSTR);
	reg = cqspi_calc_rdreg(nor, nor->program_opcode);
	writel(reg, reg_base + CQSPI_REG_RD_INSTR);

	writel(to_addr, reg_base + CQSPI_REG_INDIRECTWRSTARTADDR);

	reg = readl(reg_base + CQSPI_REG_SIZE);
	reg &= ~CQSPI_REG_SIZE_ADDRESS_MASK;
	reg |= (nor->addr_width - 1);
	writel(reg, reg_base + CQSPI_REG_SIZE);
	return 0;
}

static int cqspi_indirect_write_execute(struct spi_nor *nor,
					const u8 *txbuf, unsigned n_tx)
{
	int ret;
	unsigned int timeout;
	unsigned int reg = 0;
	struct cqspi_st *cqspi = nor->priv;
	void __iomem *reg_base = cqspi->iobase;
	void __iomem *ahb_base = cqspi->ahb_base;
	int remaining = (int)n_tx;
	struct cqspi_flash_pdata *f_pdata;
	unsigned int page_size;
	unsigned int write_bytes;

	f_pdata = &cqspi->f_pdata[cqspi->current_cs];
	page_size = f_pdata->page_size;

	writel(remaining, reg_base + CQSPI_REG_INDIRECTWRBYTES);

	writel(CQSPI_REG_SRAM_THRESHOLD_BYTES, reg_base +
	       CQSPI_REG_INDIRECTWRWATERMARK);

	/* Clear all interrupts. */
	writel(CQSPI_IRQ_STATUS_MASK, reg_base + CQSPI_REG_IRQSTATUS);

	cqspi->irq_mask = CQSPI_IRQ_MASK_WR;
	writel(cqspi->irq_mask, reg_base + CQSPI_REG_IRQMASK);

	reinit_completion(&cqspi->transfer_complete);
	writel(CQSPI_REG_INDIRECTWR_START_MASK,
	       reg_base + CQSPI_REG_INDIRECTWR);

	/* Write a page or remaining bytes. */
	write_bytes = remaining > page_size ? page_size : remaining;
	/* Fill up the data at the beginning */
	cqspi_fifo_write(ahb_base, txbuf, write_bytes);
	txbuf += write_bytes;
	remaining -= write_bytes;

	while (remaining > 0) {
		ret = wait_for_completion_timeout(&cqspi->transfer_complete,
						  msecs_to_jiffies
						  (CQSPI_TIMEOUT_MS));
		if (!ret) {
			dev_err(nor->dev, "Indirect write timeout\n");
			ret = -ETIMEDOUT;
			goto failwr;
		}

		write_bytes = remaining > page_size ? page_size : remaining;
		cqspi_fifo_write(ahb_base, txbuf, write_bytes);
		txbuf += write_bytes;
		remaining -= write_bytes;
	}

	ret = wait_for_completion_timeout(&cqspi->transfer_complete,
					  msecs_to_jiffies(CQSPI_TIMEOUT_MS));
	if (!ret) {
		dev_err(nor->dev, "Indirect write timeout\n");
		ret = -ETIMEDOUT;
		goto failwr;
	}

	/* Check indirect done status */
	timeout = cqspi_init_timeout(CQSPI_TIMEOUT_MS);
	while (cqspi_check_timeout(timeout)) {
		reg = readl(reg_base + CQSPI_REG_INDIRECTWR);
		if (reg & CQSPI_REG_INDIRECTWR_DONE_MASK)
			break;
	}

	if (!(reg & CQSPI_REG_INDIRECTWR_DONE_MASK)) {
		dev_err(nor->dev,
			"Indirect write completion error 0x%08x\n", reg);
		ret = -ETIMEDOUT;
		goto failwr;
	}

	/* Disable interrupt. */
	writel(0, reg_base + CQSPI_REG_IRQMASK);

	/* Clear indirect completion status */
	writel(CQSPI_REG_INDIRECTWR_DONE_MASK, reg_base + CQSPI_REG_INDIRECTWR);

	cqspi_wait_idle(cqspi);

	return 0;

 failwr:
	/* Disable interrupt. */
	writel(0, reg_base + CQSPI_REG_IRQMASK);

	/* Cancel the indirect write */
	writel(CQSPI_REG_INDIRECTWR_CANCEL_MASK,
	       reg_base + CQSPI_REG_INDIRECTWR);
	return ret;
}

static void cqspi_write(struct spi_nor *nor, loff_t to,
			size_t len, size_t *retlen, const u_char *buf)
{
	int ret;

	ret = cqspi_indirect_write_setup(nor, to);
	if (ret == 0) {
		ret = cqspi_indirect_write_execute(nor, buf, len);
		if (ret == 0)
			*retlen += len;
	}
}

static int cqspi_read(struct spi_nor *nor, loff_t from,
		      size_t len, size_t *retlen, u_char *buf)
{
	int ret;

	ret = cqspi_indirect_read_setup(nor, from);
	if (ret == 0) {
		ret = cqspi_indirect_read_execute(nor, buf, len);
		if (ret == 0)
			*retlen += len;
	}
	return ret;
}

static int cqspi_erase(struct spi_nor *nor, loff_t offs)
{
	int ret;

	/* Send write enable, then erase commands. */
	ret = nor->write_reg(nor, SPINOR_OP_WREN, NULL, 0, 0);
	if (ret)
		return ret;

	/* Set up command buffer. */
	ret = cqspi_command_write_addr(nor, nor->erase_opcode, offs);
	if (ret)
		return ret;

	return 0;
}

static unsigned int calculate_ticks_for_ns(unsigned int ref_clk_hz,
					   unsigned int ns_val)
{
	unsigned int ticks;

	ticks = ref_clk_hz;
	ticks /= 1000;
	ticks *= ns_val;
	ticks += 999999;
	ticks /= 1000000;
	return ticks;
}

static void cqspi_delay(struct cqspi_st *cqspi,
			unsigned int ref_clk_hz, unsigned int sclk_hz)
{
	void __iomem *iobase = cqspi->iobase;
	struct cqspi_flash_pdata *f_pdata;
	unsigned int ref_clk_ns;
	unsigned int sclk_ns;
	unsigned int tshsl, tchsh, tslch, tsd2d;
	unsigned int reg;
	unsigned int tsclk;

	f_pdata = &cqspi->f_pdata[cqspi->current_cs];

	/* Convert to ns. */
	ref_clk_ns = NSEC_PER_SEC / ref_clk_hz;

	/* Convert to ns. */
	sclk_ns = NSEC_PER_SEC / sclk_hz;

	/* calculate the number of ref ticks for one sclk tick */
	tsclk = (ref_clk_hz + sclk_hz - 1) / sclk_hz;

	tshsl = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tshsl_ns);
	/* this particular value must be at least one sclk */
	if (tshsl < tsclk)
		tshsl = tsclk;

	tchsh = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tchsh_ns);
	tslch = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tslch_ns);
	tsd2d = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tsd2d_ns);

	reg = ((tshsl & CQSPI_REG_DELAY_TSHSL_MASK)
	       << CQSPI_REG_DELAY_TSHSL_LSB);
	reg |= ((tchsh & CQSPI_REG_DELAY_TCHSH_MASK)
		<< CQSPI_REG_DELAY_TCHSH_LSB);
	reg |= ((tslch & CQSPI_REG_DELAY_TSLCH_MASK)
		<< CQSPI_REG_DELAY_TSLCH_LSB);
	reg |= ((tsd2d & CQSPI_REG_DELAY_TSD2D_MASK)
		<< CQSPI_REG_DELAY_TSD2D_LSB);
	writel(reg, iobase + CQSPI_REG_DELAY);
}

static void cqspi_config_baudrate_div(struct cqspi_st *cqspi,
				      unsigned int ref_clk_hz,
				      unsigned int sclk_hz)
{
	void __iomem *reg_base = cqspi->iobase;
	unsigned int reg;
	unsigned int div;

	reg = readl(reg_base + CQSPI_REG_CONFIG);
	reg &= ~(CQSPI_REG_CONFIG_BAUD_MASK << CQSPI_REG_CONFIG_BAUD_LSB);

	div = ref_clk_hz / sclk_hz;

	/* Recalculate the baudrate divisor based on QSPI specification. */
	if (div > 32)
		div = 32;

	/* Check if even number. */
	if (div & 1)
		div = (div / 2);
	else
		div = (div / 2) - 1;

	div = (div & CQSPI_REG_CONFIG_BAUD_MASK) << CQSPI_REG_CONFIG_BAUD_LSB;
	reg |= div;
	writel(reg, reg_base + CQSPI_REG_CONFIG);
}

static void cqspi_readdata_capture(struct cqspi_st *cqspi,
				   unsigned int bypass, unsigned int delay)
{
	void __iomem *reg_base = cqspi->iobase;
	unsigned int reg;

	reg = readl(reg_base + CQSPI_REG_READCAPTURE);

	if (bypass)
		reg |= (1 << CQSPI_REG_READCAPTURE_BYPASS_LSB);
	else
		reg &= ~(1 << CQSPI_REG_READCAPTURE_BYPASS_LSB);

	reg &= ~(CQSPI_REG_READCAPTURE_DELAY_MASK
		 << CQSPI_REG_READCAPTURE_DELAY_LSB);

	reg |= ((delay & CQSPI_REG_READCAPTURE_DELAY_MASK)
		<< CQSPI_REG_READCAPTURE_DELAY_LSB);

	writel(reg, reg_base + CQSPI_REG_READCAPTURE);
}

static void cqspi_chipselect(struct cqspi_st *cqspi,
			     unsigned int chip_select,
			     unsigned int decoder_enable)
{
	void __iomem *reg_base = cqspi->iobase;
	unsigned int reg;

	reg = readl(reg_base + CQSPI_REG_CONFIG);
	if (decoder_enable) {
		reg |= CQSPI_REG_CONFIG_DECODE_MASK;
	} else {
		reg &= ~CQSPI_REG_CONFIG_DECODE_MASK;

		/* Convert CS if without decoder.
		 * CS0 to 4b'1110
		 * CS1 to 4b'1101
		 * CS2 to 4b'1011
		 * CS3 to 4b'0111
		 */
		chip_select = 0xF & ~(1 << chip_select);
	}

	reg &= ~(CQSPI_REG_CONFIG_CHIPSELECT_MASK
		 << CQSPI_REG_CONFIG_CHIPSELECT_LSB);
	reg |= (chip_select & CQSPI_REG_CONFIG_CHIPSELECT_MASK)
	    << CQSPI_REG_CONFIG_CHIPSELECT_LSB;
	writel(reg, reg_base + CQSPI_REG_CONFIG);
}

static void cqspi_controller_enable(struct cqspi_st *cqspi)
{
	void __iomem *reg_base = cqspi->iobase;
	unsigned int reg;

	reg = readl(reg_base + CQSPI_REG_CONFIG);
	reg |= CQSPI_REG_CONFIG_ENABLE_MASK;
	writel(reg, reg_base + CQSPI_REG_CONFIG);
}

static void cqspi_controller_disable(struct cqspi_st *cqspi)
{
	void __iomem *reg_base = cqspi->iobase;
	unsigned int reg;

	reg = readl(reg_base + CQSPI_REG_CONFIG);
	reg &= ~CQSPI_REG_CONFIG_ENABLE_MASK;
	writel(reg, reg_base + CQSPI_REG_CONFIG);
}

static void cqspi_switch_cs(struct cqspi_st *cqspi, unsigned int cs)
{
	unsigned int reg;
	struct cqspi_flash_pdata *f_pdata = &cqspi->f_pdata[cs];
	void __iomem *iobase = cqspi->iobase;
	struct spi_nor *nor = &f_pdata->nor;

	cqspi_controller_disable(cqspi);

	/* configure page size and block size. */
	reg = readl(iobase + CQSPI_REG_SIZE);
	reg &= ~(CQSPI_REG_SIZE_PAGE_MASK << CQSPI_REG_SIZE_PAGE_LSB);
	reg &= ~(CQSPI_REG_SIZE_BLOCK_MASK << CQSPI_REG_SIZE_BLOCK_LSB);
	reg |= (f_pdata->page_size << CQSPI_REG_SIZE_PAGE_LSB);
	reg |= (f_pdata->block_size << CQSPI_REG_SIZE_BLOCK_LSB);
	reg &= ~CQSPI_REG_SIZE_ADDRESS_MASK;
	reg |= (nor->addr_width - 1);
	writel(reg, iobase + CQSPI_REG_SIZE);

	/* configure the chip select */
	cqspi_chipselect(cqspi, cs, cqspi->is_decoded_cs);

	cqspi_controller_enable(cqspi);
}

static int cqspi_prep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct cqspi_st *cqspi = nor->priv;
	int cs = cqspi_find_chipselect(nor);
	struct cqspi_flash_pdata *f_pdata;
	unsigned int sclk;

	/* Switch chip select. */
	if (cqspi->current_cs != cs) {
		cqspi->current_cs = cs;
		cqspi_switch_cs(cqspi, cs);
	}

	/* Setup baudrate divisor and delays */
	f_pdata = &cqspi->f_pdata[cqspi->current_cs];
	sclk = f_pdata->clk_rate;
	if (cqspi->sclk != sclk) {
		cqspi->sclk = sclk;
		cqspi_controller_disable(cqspi);
		cqspi_config_baudrate_div(cqspi,
					  cqspi->master_ref_clk_hz, sclk);
		cqspi_delay(cqspi, cqspi->master_ref_clk_hz, sclk);
		cqspi_readdata_capture(cqspi, 1, f_pdata->read_delay);
		cqspi_controller_enable(cqspi);
	}
	return 0;
}

static void cqspi_unprep(struct spi_nor *nor, enum spi_nor_ops ops)
{
}

static int cqspi_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	int ret;

	cqspi_prep(nor, SPI_NOR_OPS_READ);

	ret = cqspi_command_read(nor, &opcode, 1, buf, len);
	return ret;
}

static int cqspi_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len,
			   int write_enable)
{
	int ret = 0;

	cqspi_prep(nor, SPI_NOR_OPS_WRITE);

	ret = cqspi_command_write(nor, opcode, buf, len);
	return ret;
}

static int cqspi_of_get_flash_pdata(struct platform_device *pdev,
				    struct cqspi_flash_pdata *f_pdata,
				    struct device_node *np)
{
	if (of_property_read_u32(np, "cdns,page-size", &f_pdata->page_size)) {
		dev_err(&pdev->dev, "couldn't determine page-size\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,block-size", &f_pdata->block_size)) {
		dev_err(&pdev->dev, "couldn't determine block-size\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,read-delay", &f_pdata->read_delay)) {
		dev_err(&pdev->dev, "couldn't determine read-delay\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,tshsl-ns", &f_pdata->tshsl_ns)) {
		dev_err(&pdev->dev, "couldn't determine tshsl-ns\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,tsd2d-ns", &f_pdata->tsd2d_ns)) {
		dev_err(&pdev->dev, "couldn't determine tsd2d-ns\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,tchsh-ns", &f_pdata->tchsh_ns)) {
		dev_err(&pdev->dev, "couldn't determine tchsh-ns\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,tslch-ns", &f_pdata->tslch_ns)) {
		dev_err(&pdev->dev, "couldn't determine tslch-ns\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "spi-max-frequency", &f_pdata->clk_rate)) {
		dev_err(&pdev->dev, "couldn't determine spi-max-frequency\n");
		return -ENXIO;
	}

	return 0;
}

static int cqspi_of_get_pdata(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct cqspi_st *cqspi = platform_get_drvdata(pdev);

	if (of_property_read_u32(np, "is-decoded-cs", &cqspi->is_decoded_cs))
		cqspi->is_decoded_cs = 0;

	if (of_property_read_u32(np, "fifo-depth", &cqspi->fifo_depth)) {
		dev_err(&pdev->dev, "couldn't determine fifo-depth\n");
		return -ENXIO;
	}

	return 0;
}

static void cqspi_controller_init(struct cqspi_st *cqspi)
{
	cqspi_controller_disable(cqspi);

	/* Configure the remap address register, no remap */
	writel(0, cqspi->iobase + CQSPI_REG_REMAP);

	/* Disable all interrupts. */
	writel(0, cqspi->iobase + CQSPI_REG_IRQMASK);

	cqspi_controller_enable(cqspi);
}

static int cqspi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mtd_part_parser_data ppdata;
	struct device *dev = &pdev->dev;
	struct cqspi_st *cqspi;
	struct spi_nor *nor;
	struct mtd_info *mtd;
	struct resource *res;
	struct resource *res_ahb;
	int ret;
	int irq;

	cqspi = devm_kzalloc(dev, sizeof(*cqspi), GFP_KERNEL);
	if (!cqspi)
		return -ENOMEM;

	cqspi->pdev = pdev;
	platform_set_drvdata(pdev, cqspi);

	ret = cqspi_of_get_pdata(pdev);
	if (ret) {
		dev_err(dev, "Get platform data failed.\n");
		return -ENODEV;
	}

	cqspi->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(cqspi->clk)) {
		dev_err(dev, "cannot get qspi clk\n");
		ret = PTR_ERR(cqspi->clk);
		goto probe_failed;
	}
	cqspi->master_ref_clk_hz = clk_get_rate(cqspi->clk);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cqspi->iobase = devm_ioremap_resource(dev, res);
	if (IS_ERR(cqspi->iobase)) {
		dev_err(dev, "devm_ioremap_resource res 0 failed\n");
		ret = PTR_ERR(cqspi->iobase);
		goto probe_failed;
	}

	res_ahb = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	cqspi->ahb_phy_addr = res_ahb->start;
	cqspi->ahb_base = devm_ioremap_resource(dev, res_ahb);
	if (IS_ERR(cqspi->ahb_base)) {
		dev_err(dev, "devm_ioremap_resource res 1 failed\n");
		ret = PTR_ERR(cqspi->ahb_base);
		goto probe_failed;
	}

	init_completion(&cqspi->transfer_complete);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "platform_get_irq failed\n");
		ret = -ENXIO;
		goto probe_failed;
	}
	ret = devm_request_irq(dev, irq,
			       cqspi_irq_handler, 0, pdev->name, cqspi);
	if (ret) {
		dev_err(dev, "devm_request_irq failed\n");
		goto probe_failed;
	}

	cqspi_wait_idle(cqspi);
	cqspi_controller_init(cqspi);
	cqspi->current_cs = -1;
	cqspi->sclk = 0;

	/* Get flash device data */
	for_each_available_child_of_node(dev->of_node, np) {
		unsigned int cs;
		struct cqspi_flash_pdata *f_pdata;

		if (of_property_read_u32(np, "reg", &cs)) {
			dev_err(dev, "couldn't determine chip select\n");
			return -ENXIO;
		}
		if (cs > CQSPI_MAX_CHIPSELECT) {
			dev_err(dev, "chip select %d out of range\n", cs);
			return -ENXIO;
		}
		f_pdata = &cqspi->f_pdata[cs];

		ret = cqspi_of_get_flash_pdata(pdev, f_pdata, np);
		if (ret)
			goto probe_failed;

		f_pdata->quad_mode_set = 0;

		nor = &f_pdata->nor;
		mtd = &f_pdata->mtd;

		nor->mtd = mtd;
		nor->dev = dev;
		nor->priv = cqspi;
		mtd->priv = nor;

		nor->read_reg = cqspi_read_reg;
		nor->write_reg = cqspi_write_reg;
		nor->read = cqspi_read;
		nor->write = cqspi_write;
		nor->erase = cqspi_erase;

		nor->prepare = cqspi_prep;
		nor->unprepare = cqspi_unprep;

		ret = spi_nor_scan(nor, NULL, SPI_NOR_QUAD);
		if (ret)
			goto probe_failed;

		ppdata.of_node = np;
		ret = mtd_device_parse_register(mtd, NULL, &ppdata, NULL, 0);
		if (ret)
			goto probe_failed;
	}

	dev_info(dev, "Cadence QSPI NOR flash driver\n");
	return 0;

 probe_failed:
	dev_err(dev, "Cadence QSPI NOR probe failed\n");
	return ret;
}

static int cqspi_remove(struct platform_device *pdev)
{
	struct cqspi_st *cqspi = platform_get_drvdata(pdev);
	int i;

	cqspi_controller_disable(cqspi);

	for (i = 0; i < CQSPI_MAX_CHIPSELECT; i++)
		if (cqspi->f_pdata[i].mtd.name)
			mtd_device_unregister(&cqspi->f_pdata[i].mtd);

	return 0;
}

#ifdef CONFIG_PM
static int cqspi_suspend(struct device *dev)
{
	struct cqspi_st *cqspi = dev_get_drvdata(dev);

	cqspi_controller_disable(cqspi);
	return 0;
}

static int cqspi_resume(struct device *dev)
{
	struct cqspi_st *cqspi = dev_get_drvdata(dev);

	cqspi_controller_enable(cqspi);
	return 0;
}

static const struct dev_pm_ops cqspi__dev_pm_ops = {
	.suspend = cqspi_suspend,
	.resume = cqspi_resume,
};

#define CQSPI_DEV_PM_OPS	(&cqspi__dev_pm_ops)
#else
#define CQSPI_DEV_PM_OPS	NULL
#endif

static struct of_device_id const cqspi_dt_ids[] = {
	{.compatible = "cdns,qspi-nor",},
	{ /* end of table */ }
};

MODULE_DEVICE_TABLE(of, cqspi_dt_ids);

static struct platform_driver cqspi_platform_driver = {
	.probe = cqspi_probe,
	.remove = cqspi_remove,
	.driver = {
		   .name = CQSPI_NAME,
		   .pm = CQSPI_DEV_PM_OPS,
		   .of_match_table = cqspi_dt_ids,
		   },
};

module_platform_driver(cqspi_platform_driver);

MODULE_DESCRIPTION("Cadence QSPI Controller Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" CQSPI_NAME);
MODULE_AUTHOR("Ley Foon Tan <lftan@altera.com>");
MODULE_AUTHOR("Graham Moore <grmoore@opensource.altera.com>");
