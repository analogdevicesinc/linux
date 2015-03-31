/*
 * Analog Devices ADF7242 Low-Power IEEE 802.15.4 Transceiver
 *
 * Copyright 2009-2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/firmware.h>
#include <linux/spi/spi.h>
#include <linux/spi/adf7242.h>
#include <linux/skbuff.h>
#include <linux/of.h>

#include <net/mac802154.h>
#include <net/wpan-phy.h>

#define FIRMWARE       "adf7242_firmware.bin"
#define MAX_POLL_LOOPS 50

/* All Registers */

#define REG_EXT_CTRL   0x100	/* RW External LNA/PA and internal PA control configuration bits */
#define REG_TX_FSK_TEST 0x101	/* RW TX FSK test mode configuration */
#define REG_CCA1       0x105	/* RW RSSI threshold for CCA */
#define REG_CCA2       0x106	/* RW CCA mode configuration */
#define REG_BUFFERCFG  0x107	/* RW RX_BUFFER overwrite control */
#define REG_PKT_CFG    0x108	/* RW FCS evaluation configuration */
#define REG_DELAYCFG0  0x109	/* RW RC_RX command to SFD or sync word search delay */
#define REG_DELAYCFG1  0x10A	/* RW RC_TX command to TX state */
#define REG_DELAYCFG2  0x10B	/* RW Mac delay extention */
#define REG_SYNC_WORD0 0x10C	/* RW sync word bits [7:0] of [23:0]  */
#define REG_SYNC_WORD1 0x10D	/* RW sync word bits [15:8] of [23:0]  */
#define REG_SYNC_WORD2 0x10E	/* RW sync word bits [23:16] of [23:0]  */
#define REG_SYNC_CONFIG        0x10F	/* RW sync word configuration */
#define REG_RC_CFG     0x13E	/* RW RX / TX packet configuration */
#define REG_RC_VAR44   0x13F	/* RW RESERVED */
#define REG_CH_FREQ0   0x300	/* RW Channel Frequency Settings - Low Byte */
#define REG_CH_FREQ1   0x301	/* RW Channel Frequency Settings - Middle Byte */
#define REG_CH_FREQ2   0x302	/* RW Channel Frequency Settings - 2 MSBs */
#define REG_TX_FD      0x304	/* RW TX Frequency Deviation Register */
#define REG_DM_CFG0    0x305	/* RW RX Discriminator BW Register */
#define REG_TX_M       0x306	/* RW TX Mode Register */
#define REG_RX_M       0x307	/* RW RX Mode Register */
#define REG_RRB                0x30C	/* R RSSI Readback Register */
#define REG_LRB                0x30D	/* R Link Quality Readback Register */
#define REG_DR0                0x30E	/* RW bits [15:8] of [15:0] for data rate setting */
#define REG_DR1                0x30F	/* RW bits [7:0] of [15:0] for data rate setting */
#define REG_PRAMPG     0x313	/* RW RESERVED */
#define REG_TXPB       0x314	/* RW TX Packet Storage Base Address */
#define REG_RXPB       0x315	/* RW RX Packet Storage Base Address */
#define REG_TMR_CFG0   0x316	/* RW Wake up Timer Configuration Register - High Byte */
#define REG_TMR_CFG1   0x317	/* RW Wake up Timer Configuration Register - Low Byte */
#define REG_TMR_RLD0   0x318	/* RW Wake up Timer Value Register - High Byte */
#define REG_TMR_RLD1   0x319	/* RW Wake up Timer Value Register - Low Byte */
#define REG_TMR_CTRL   0x31A	/* RW Wake up Timer Timeout flag */
#define REG_PD_AUX     0x31E	/* RW Battmon enable */
#define REG_GP_CFG     0x32C	/* RW GPIO Configuration */
#define REG_GP_OUT     0x32D	/* RW GPIO Configuration */
#define REG_GP_IN      0x32E	/* R GPIO Configuration */
#define REG_SYNT       0x335	/* RW bandwidth calibration timers */
#define REG_CAL_CFG    0x33D	/* RW Calibration Settings */
#define REG_PA_BIAS    0x36E	/* RW PA BIAS */
#define REG_SYNT_CAL   0x371	/* RW Oscillator and Doubler Configuration */
#define REG_IIRF_CFG   0x389	/* RW BB Filter Decimation Rate */
#define REG_CDR_CFG    0x38A	/* RW CDR kVCO */
#define REG_DM_CFG1    0x38B	/* RW Postdemodulator Filter */
#define REG_AGCSTAT    0x38E	/* R RXBB Ref Osc Calibration Engine Readback */
#define REG_RXCAL0     0x395	/* RW RX BB filter tuning, LSB */
#define REG_RXCAL1     0x396	/* RW RX BB filter tuning, MSB */
#define REG_RXFE_CFG   0x39B	/* RW RXBB Ref Osc & RXFE Calibration */
#define REG_PA_RR      0x3A7	/* RW Set PA ramp rate */
#define REG_PA_CFG     0x3A8	/* RW PA enable */
#define REG_EXTPA_CFG  0x3A9	/* RW External PA BIAS DAC */
#define REG_EXTPA_MSC  0x3AA	/* RW PA Bias Mode */
#define REG_ADC_RBK    0x3AE	/* R Readback temp */
#define REG_AGC_CFG1   0x3B2	/* RW GC Parameters */
#define REG_AGC_MAX    0x3B4	/* RW Slew rate  */
#define REG_AGC_CFG2   0x3B6	/* RW RSSI Parameters */
#define REG_AGC_CFG3   0x3B7	/* RW RSSI Parameters */
#define REG_AGC_CFG4   0x3B8	/* RW RSSI Parameters */
#define REG_AGC_CFG5   0x3B9	/* RW RSSI & NDEC Parameters */
#define REG_AGC_CFG6   0x3BA	/* RW NDEC Parameters */
#define REG_OCL_CFG1   0x3C4	/* RW OCL System Parameters */
#define REG_IRQ1_EN0   0x3C7	/* RW Interrupt Mask set bits  [7:0] of [15:0] for IRQ1 */
#define REG_IRQ1_EN1   0x3C8	/* RW Interrupt Mask set bits  [15:8] of [15:0] for IRQ1 */
#define REG_IRQ2_EN0   0x3C9	/* RW Interrupt Mask set bits  [7:0] of [15:0] for IRQ2 */
#define REG_IRQ2_EN1   0x3CA	/* RW Interrupt Mask set bits  [15:8] of [15:0] for IRQ2 */
#define REG_IRQ1_SRC0  0x3CB	/* RW Interrupt Source  bits  [7:0] of [15:0] for IRQ */
#define REG_IRQ1_SRC1  0x3CC	/* RW Interrupt Source bits  [15:8] of [15:0] for IRQ */
#define REG_OCL_BW0    0x3D2	/* RW OCL System Parameters */
#define REG_OCL_BW1    0x3D3	/* RW OCL System Parameters */
#define REG_OCL_BW2    0x3D4	/* RW OCL System Parameters */
#define REG_OCL_BW3    0x3D5	/* RW OCL System Parameters */
#define REG_OCL_BW4    0x3D6	/* RW OCL System Parameters */
#define REG_OCL_BWS    0x3D7	/* RW OCL System Parameters */
#define REG_OCL_CFG13  0x3E0	/* RW OCL System Parameters */
#define REG_GP_DRV     0x3E3	/* RW I/O pads Configuration and bg trim */
#define REG_BM_CFG     0x3E6	/* RW Battery Monitor Threshold Voltage setting */
#define REG_SFD_15_4   0x3F4	/* RW Option to set non standard SFD */
#define REG_AFC_CFG    0x3F7	/* RW AFC mode and polarity */
#define REG_AFC_KI_KP  0x3F8	/* RW AFC ki and kp */
#define REG_AFC_RANGE  0x3F9	/* RW AFC range */
#define REG_AFC_READ   0x3FA	/* RW Readback frequency error */

/* REG_EXTPA_MSC */
#define PA_PWR(x)		(((x) & 0xF) << 4)
#define EXTPA_BIAS_SRC		(1 << 3)
#define EXTPA_BIAS_MODE(x)	(((x) & 0x7) << 0)

/* REG_PA_CFG */
#define PA_BRIDGE_DBIAS(x)	(((x) & 0x1F) << 0)

/* REG_PA_BIAS */
#define PA_BIAS_CTRL(x)		(((x) & 0x1F) << 1)
#define REG_PA_BIAS_DFL		(1 << 0)

#define REG_PAN_ID0            0x112
#define REG_PAN_ID1            0x113
#define REG_SHORT_ADDR_0       0x114
#define REG_SHORT_ADDR_1       0x115
#define REG_IEEE_ADDR_0                0x116
#define REG_IEEE_ADDR_1                0x117
#define REG_IEEE_ADDR_2                0x118
#define REG_IEEE_ADDR_3                0x119
#define REG_IEEE_ADDR_4                0x11A
#define REG_IEEE_ADDR_5                0x11B
#define REG_IEEE_ADDR_6                0x11C
#define REG_IEEE_ADDR_7                0x11D
#define REG_FFILT_CFG          0x11E
#define REG_AUTO_CFG           0x11F
#define REG_AUTO_TX1           0x120
#define REG_AUTO_TX2           0x121
#define REG_AUTO_STATUS                0x122

/* REG_FFILT_CFG */
#define ACCEPT_BEACON_FRAMES   (1 << 0)
#define ACCEPT_DATA_FRAMES     (1 << 1)
#define ACCEPT_ACK_FRAMES      (1 << 2)
#define ACCEPT_MACCMD_FRAMES   (1 << 3)
#define ACCEPT_RESERVED_FRAMES (1 << 4)
#define ACCEPT_ALL_ADDRESS     (1 << 5)

/* REG_AUTO_CFG */
#define AUTO_ACK_FRAMEPEND     (1 << 0)
#define IS_PANCOORD            (1 << 1)
#define RX_AUTO_ACK_EN         (1 << 3)
#define CSMA_CA_RX_TURNAROUND  (1 << 4)

/* REG_AUTO_TX1 */
#define MAX_FRAME_RETRIES(x)   ((x) & 0xF)
#define MAX_CCA_RETRIES(x)     (((x) & 0x7) << 4)

/* REG_AUTO_TX2 */
#define CSMA_MAX_BE(x)         ((x) & 0xF)
#define CSMA_MIN_BE(x)         (((x) & 0xF) << 4)

#define CMD_SPI_NOP            0xFF	/* No operation. Use for dummy writes */
#define CMD_SPI_PKT_WR         0x10	/* Write telegram to the Packet RAM starting from the TX packet base address pointer tx_packet_base */
#define CMD_SPI_PKT_RD         0x30	/* Read telegram from the Packet RAM starting from RX packet base address pointer rxpb.rx_packet_base */
#define CMD_SPI_MEM_WR(x)      (0x18 + (x >> 8))	/* Write data to MCR or Packet RAM sequentially */
#define CMD_SPI_MEM_RD(x)      (0x38 + (x >> 8))	/* Read data from MCR or Packet RAM sequentially */
#define CMD_SPI_MEMR_WR(x)     (0x08 + (x >> 8))	/* Write data to MCR or Packet RAM as random block */
#define CMD_SPI_MEMR_RD(x)     (0x28 + (x >> 8))	/* Read data from MCR or Packet RAM as random block */
#define CMD_SPI_PRAM_WR                0x1E	/* Write data sequentially to current PRAM page selected */
#define CMD_SPI_PRAM_RD                0x3E	/* Read data sequentially from current PRAM page selected */
#define CMD_RC_SLEEP           0xB1	/* Invoke transition of radio controller into SLEEP state */
#define CMD_RC_IDLE            0xB2	/* Invoke transition of radio controller into IDLE state */
#define CMD_RC_PHY_RDY         0xB3	/* Invoke transition of radio controller into PHY_RDY state */
#define CMD_RC_RX              0xB4	/* Invoke transition of radio controller into RX state */
#define CMD_RC_TX              0xB5	/* Invoke transition of radio controller into TX state */
#define CMD_RC_MEAS            0xB6	/* Invoke transition of radio controller into MEAS state */
#define CMD_RC_CCA             0xB7	/* Invoke Clear channel assessment */
#define CMD_RC_CSMACA          0xC1	/* initiates CSMA-CA channel access sequence and frame transmission */
#define CMD_RC_PC_RESET        0xC7	/* Program counter reset */
#define CMD_RC_RESET           0xC8	/* Resets the ADF7242 and puts it in the sleep state */

/* STATUS */

#define STAT_SPI_READY         (1 << 7)
#define STAT_IRQ_STATUS                (1 << 6)
#define STAT_RC_READY          (1 << 5)
#define STAT_CCA_RESULT                (1 << 4)
#define RC_STATUS_IDLE         1
#define RC_STATUS_MEAS         2
#define RC_STATUS_PHY_RDY      3
#define RC_STATUS_RX           4
#define RC_STATUS_TX           5
#define RC_STATUS_MASK         0xF

/* AUTO_STATUS */

#define SUCCESS                        0
#define SUCCESS_DATPEND                1
#define FAILURE_CSMACA         2
#define FAILURE_NOACK          3
#define AUTO_STATUS_MASK       0x3

#define PRAM_PAGESIZE          256

/* IRQ1 */

#define IRQ_CCA_COMPLETE       (1 << 0)
#define IRQ_SFD_RX             (1 << 1)
#define IRQ_SFD_TX             (1 << 2)
#define IRQ_RX_PKT_RCVD                (1 << 3)
#define IRQ_TX_PKT_SENT                (1 << 4)
#define IRQ_FRAME_VALID                (1 << 5)
#define IRQ_ADDRESS_VALID      (1 << 6)
#define IRQ_CSMA_CA            (1 << 7)

#define AUTO_TX_TURNAROUND     (1 << 3)
#define ADDON_EN               (1 << 4)

static struct adf7242_platform_data adf7242_default_pdata = {
	.mode = ADF_IEEE802154_AUTO_CSMA_CA | ADF_IEEE802154_HW_AACK,
	.max_frame_retries = 4,
	.max_cca_retries = 4,
	.max_csma_be = 6,
	.min_csma_be = 1,
};

struct adf7242_local {
	struct spi_device *spi;
	struct adf7242_platform_data *pdata;
	struct completion tx_complete;
	struct ieee802154_dev *dev;
	struct mutex bmux;
	struct spi_message stat_msg;
	struct spi_transfer stat_xfer;
	unsigned mode;
	unsigned tx_irq;
	int tx_stat;
	u8 max_frame_retries;
	u8 max_cca_retries;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */

	u8 buf[3] ____cacheline_aligned;
	u8 buf_reg_tx[3];
	u8 buf_read_tx[4];
	u8 buf_read_rx[4];
	u8 buf_stat_rx;
	u8 buf_stat_tx;
	u8 buf_cmd;

};

static int adf7242_status(struct adf7242_local *lp, u8 * stat)
{
	int status;

	mutex_lock(&lp->bmux);
	status = spi_sync(lp->spi, &lp->stat_msg);
	*stat = lp->buf_stat_rx;
	mutex_unlock(&lp->bmux);

	return status;
}

static int adf7242_wait_ready(struct adf7242_local *lp)
{
	u8 stat;
	int cnt = 0;

	dev_vdbg(&lp->spi->dev, "%s :Enter\n", __func__);

	do {
		adf7242_status(lp, &stat);
		cnt++;
	} while (!(stat & STAT_RC_READY) && (cnt < MAX_POLL_LOOPS));

	if (cnt >= MAX_POLL_LOOPS)
		dev_warn(&lp->spi->dev, "%s Timeout (%d)\n", __func__, cnt);

	dev_vdbg(&lp->spi->dev, "%s :Exit loops=%d\n", __func__, cnt);

	return 0;
}

static int adf7242_wait_status(struct adf7242_local *lp, int status)
{
	u8 stat;
	int cnt = 0;

	dev_vdbg(&lp->spi->dev, "%s :Enter\n", __func__);

	do {
		adf7242_status(lp, &stat);
		stat &= RC_STATUS_MASK;
		cnt++;
	} while ((stat != status) && (cnt < MAX_POLL_LOOPS));

	if (cnt >= MAX_POLL_LOOPS)
		dev_warn(&lp->spi->dev, "%s Timeout (%d)\n", __func__, cnt);

	dev_vdbg(&lp->spi->dev, "%s :Exit loops=%d\n", __func__, cnt);

	return 0;
}

static int adf7242_write_fbuf(struct adf7242_local *lp, u8 * data, u8 len)
{
	u8 *buf = lp->buf;
	int status;
	struct spi_message msg;
	struct spi_transfer xfer_head = {
		.len = 2,
		.tx_buf = buf,

	};
	struct spi_transfer xfer_buf = {
		.len = len,
		.tx_buf = data,
	};

	dev_vdbg(&lp->spi->dev, "%s :Enter\n", __func__);

	spi_message_init(&msg);
	spi_message_add_tail(&xfer_head, &msg);
	spi_message_add_tail(&xfer_buf, &msg);

	adf7242_wait_ready(lp);

	mutex_lock(&lp->bmux);
	buf[0] = CMD_SPI_PKT_WR;
	buf[1] = len + 2;

	status = spi_sync(lp->spi, &msg);
	mutex_unlock(&lp->bmux);

	dev_vdbg(&lp->spi->dev, "%s :Exit\n", __func__);
	return status;
}

static int adf7242_read_fbuf(struct adf7242_local *lp,
			     u8 *data, size_t *len, u8 *lqi)
{
	u8 *buf = lp->buf;
	u8 plen;
	int status;
	struct spi_message msg;
	struct spi_transfer xfer_head = {
		.len = 3,
		.tx_buf = buf,
		.rx_buf = buf,

	};
	struct spi_transfer xfer_buf = {
		.len = *len,
		.rx_buf = data,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer_head, &msg);
	spi_message_add_tail(&xfer_buf, &msg);

	dev_vdbg(&lp->spi->dev, "%s :Enter\n", __func__);
	adf7242_wait_ready(lp);

	mutex_lock(&lp->bmux);
	if (lqi) {
		buf[0] = CMD_SPI_PKT_RD;
		buf[1] = CMD_SPI_NOP;
		buf[2] = 0;		/* PHR */
	} else {
		buf[0] = CMD_SPI_PRAM_RD;
		buf[1] = 0;
		buf[2] = CMD_SPI_NOP;
	}

	status = spi_sync(lp->spi, &msg);

	if (!status && lqi) {
		plen = buf[2] & 0x7F;
		*lqi = data[plen - 1];
		*len = plen;	/* PHR */
	}

	mutex_unlock(&lp->bmux);
	dev_vdbg(&lp->spi->dev, "%s :Exit\n", __func__);
	return status;
}

static int adf7242_read_reg(struct adf7242_local *lp, u16 addr, u8 * data)
{
	int status;
	struct spi_message msg;

	struct spi_transfer xfer = {
		.len = 4,
		.tx_buf = lp->buf_read_tx,
		.rx_buf = lp->buf_read_rx,
	};

	dev_vdbg(&lp->spi->dev, "%s :Enter\n", __func__);
	adf7242_wait_ready(lp);

	mutex_lock(&lp->bmux);
	lp->buf_read_tx[0] = CMD_SPI_MEM_RD(addr);
	lp->buf_read_tx[1] = addr;
	lp->buf_read_tx[2] = CMD_SPI_NOP;
	lp->buf_read_tx[3] = CMD_SPI_NOP;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	status = spi_sync(lp->spi, &msg);
	if (msg.status)
		status = msg.status;

	if (!status)
		*data = lp->buf_read_rx[3];

	mutex_unlock(&lp->bmux);
	dev_vdbg(&lp->spi->dev, "%s :Exit REG 0x%X, VAL 0x%X\n", __func__,
		 addr, *data);

	return status;
}

static int adf7242_write_reg(struct adf7242_local *lp, u16 addr, u8 data)
{
	int status;

	dev_vdbg(&lp->spi->dev, "%s :Enter\n", __func__);

	adf7242_wait_ready(lp);

	mutex_lock(&lp->bmux);
	lp->buf_reg_tx[0] = CMD_SPI_MEM_WR(addr);
	lp->buf_reg_tx[1] = addr;
	lp->buf_reg_tx[2] = data;
	status = spi_write(lp->spi, lp->buf_reg_tx, 3);
	mutex_unlock(&lp->bmux);

	dev_vdbg(&lp->spi->dev, "%s :Exit REG 0x%X, VAL 0x%X\n",
		 __func__, addr, data);

	return status;
}

static int adf7242_cmd(struct adf7242_local *lp, u8 cmd)
{
	int status;

	dev_vdbg(&lp->spi->dev, "%s :Enter CMD=0x%X\n", __func__, cmd);

	adf7242_wait_ready(lp);

	mutex_lock(&lp->bmux);
	lp->buf_cmd = cmd;
	status = spi_write(lp->spi, &lp->buf_cmd, 1);
	mutex_unlock(&lp->bmux);

	dev_vdbg(&lp->spi->dev, "%s :Exit\n", __func__);

	return status;
}

static int adf7242_upload_firmware(struct adf7242_local *lp, u8 * data, u16 len)
{
	struct spi_message msg;
	struct spi_transfer xfer_buf = { };
	int status, i, page = 0;
	u8 *buf = lp->buf;

	struct spi_transfer xfer_head = {
		.len = 2,
		.tx_buf = buf,
	};

	buf[0] = CMD_SPI_PRAM_WR;
	buf[1] = 0;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer_head, &msg);
	spi_message_add_tail(&xfer_buf, &msg);

	for (i = len; i >= 0; i -= PRAM_PAGESIZE) {
		adf7242_write_reg(lp, REG_PRAMPG, page);

		xfer_buf.len = (i >= PRAM_PAGESIZE) ? PRAM_PAGESIZE : i;
		xfer_buf.tx_buf = &data[page * PRAM_PAGESIZE];

		mutex_lock(&lp->bmux);
		status = spi_sync(lp->spi, &msg);
		mutex_unlock(&lp->bmux);
		page++;
	}

	return status;
}

static int adf7242_verify_firmware(struct adf7242_local *lp,
		const u8 *data, size_t len)
{
	int i, j;
	unsigned int page;

	u8 *buf = kmalloc(PRAM_PAGESIZE, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	for (page = 0, i = len; i >= 0; i -= PRAM_PAGESIZE, page++) {

		size_t nb = (i >= PRAM_PAGESIZE)	 ? PRAM_PAGESIZE : i;
		adf7242_write_reg(lp, REG_PRAMPG, page);
		adf7242_read_fbuf(lp, buf, &nb, NULL);

		for (j = 0; j < nb; j++) {
			if (buf[j] != data[page * PRAM_PAGESIZE + j]) {
				kfree(buf);
				return -EIO;

			}
		}
	}
	kfree(buf);

	return 0;
}

static int
adf7242_set_txpower(struct ieee802154_dev *dev, int db)
{
	struct adf7242_local *lp = dev->priv;
	u8 pwr, bias_ctrl, dbias, tmp;
	int ret;

	dev_vdbg(&lp->spi->dev, "%s :Enter Power %d dB\n", __func__, db);

	if (db > 5 || db < -26)
		return -EINVAL;

	db = DIV_ROUND_CLOSEST(db + 29, 2);

	if (db > 15) {
		dbias = 21;
		bias_ctrl = 63;
	} else {
		dbias = 13;
		bias_ctrl = 55;
	}

	pwr = clamp_t(u8, db, 3, 15);

	adf7242_read_reg(lp, REG_PA_CFG, &tmp);
	tmp &= ~PA_BRIDGE_DBIAS(~0);
	tmp |= PA_BRIDGE_DBIAS(dbias);
	adf7242_write_reg(lp, REG_PA_CFG, tmp);

	adf7242_read_reg(lp, REG_PA_BIAS, &tmp);
	tmp &= ~PA_BIAS_CTRL(~0);
	tmp |= PA_BIAS_CTRL(bias_ctrl);
	adf7242_write_reg(lp, REG_PA_BIAS, tmp);

	adf7242_read_reg(lp, REG_EXTPA_MSC, &tmp);
	tmp &= ~PA_PWR(~0);
	tmp |= PA_PWR(pwr);
	ret = adf7242_write_reg(lp, REG_EXTPA_MSC, tmp);

	dev_vdbg(&lp->spi->dev, "%s :Exit\n", __func__);
	return ret;
}

static int
adf7242_set_csma_params(struct ieee802154_dev *dev, u8 min_be, u8 max_be,
			  u8 retries)
{
	struct adf7242_local *lp = dev->priv;
	int ret;
	dev_vdbg(&lp->spi->dev, "%s :Enter min_be=%d max_be=%d retries=%d\n",
		 __func__, min_be, max_be, retries);

	if (min_be > max_be || max_be > 8 || retries > 5)
		return -EINVAL;

	ret = adf7242_write_reg(lp, REG_AUTO_TX1,
				  MAX_FRAME_RETRIES(lp->max_frame_retries) |
				  MAX_CCA_RETRIES(retries));
	if (ret)
		return ret;

	lp->max_cca_retries = retries;

	dev_vdbg(&lp->spi->dev, "%s :Exit\n", __func__);

	return adf7242_write_reg(lp, REG_AUTO_TX2, CSMA_MAX_BE(max_be) |
			CSMA_MIN_BE(min_be));
}

static int
adf7242_set_frame_retries(struct ieee802154_dev *dev, s8 retries)
{
	struct adf7242_local *lp = dev->priv;
	int ret = 0;

	dev_vdbg(&lp->spi->dev, "%s :Enter Retries = %d\n", __func__, retries);

	if (retries < -1 || retries > 15)
		return -EINVAL;

	if (retries >= 0)
		ret = adf7242_write_reg(lp, REG_AUTO_TX1,
				  MAX_FRAME_RETRIES(retries) |
				  MAX_CCA_RETRIES(lp->max_cca_retries));

	lp->max_frame_retries = retries;
	dev_vdbg(&lp->spi->dev, "%s :Exit\n", __func__);
	return ret;
}

static int adf7242_ed(struct ieee802154_dev *dev, u8 *level)
{
	struct adf7242_local *lp = dev->priv;
	int ret;

	dev_vdbg(&lp->spi->dev, "%s :Enter\n", __func__);
#if 0
	adf7242_cmd(lp, CMD_RC_PHY_RDY);
	adf7242_cmd(lp, CMD_RC_CCA);
	adf7242_wait_status(lp, RC_STATUS_PHY_RDY);
#else
	udelay(128);
#endif
	ret = adf7242_read_reg(lp, REG_RRB, level);
	adf7242_cmd(lp, CMD_RC_RX);
	dev_vdbg(&lp->spi->dev, "%s :Exit ret=%d level=%d\n",
		 __func__, ret, *level);

	return ret;
}

static int adf7242_start(struct ieee802154_dev *dev)
{
	struct adf7242_local *lp = dev->priv;
	int ret;

	dev_vdbg(&lp->spi->dev, "%s :Enter\n", __func__);
	ret = adf7242_cmd(lp, CMD_RC_RX);
	dev_vdbg(&lp->spi->dev, "%s :Exit\n", __func__);

	return ret;
}

static void adf7242_stop(struct ieee802154_dev *dev)
{
	struct adf7242_local *lp = dev->priv;

	dev_vdbg(&lp->spi->dev, "%s :Enter\n", __func__);
	adf7242_cmd(lp, CMD_RC_PHY_RDY);
	dev_vdbg(&lp->spi->dev, "%s :Exit\n", __func__);
}

static int adf7242_channel(struct ieee802154_dev *dev, int page, int channel)
{
	struct adf7242_local *lp = dev->priv;
	unsigned long freq;

	dev_vdbg(&lp->spi->dev, "%s :Enter\n", __func__);
	dev_dbg(&lp->spi->dev, "%s :Channel=%d\n", __func__, channel);

	might_sleep();

	BUG_ON(page != 0);
	BUG_ON(channel < 11);
	BUG_ON(channel > 26);

	freq = (2405 + 5 * (channel - 11)) * 100;
	adf7242_cmd(lp, CMD_RC_PHY_RDY);

	adf7242_write_reg(lp, REG_CH_FREQ0, freq);
	adf7242_write_reg(lp, REG_CH_FREQ1, freq >> 8);
	adf7242_write_reg(lp, REG_CH_FREQ2, freq >> 16);

	adf7242_cmd(lp, CMD_RC_RX);

	dev->phy->current_channel = channel;
	dev_vdbg(&lp->spi->dev, "%s :Exit\n", __func__);

	return 0;
}

static int adf7242_set_hw_addr_filt(struct ieee802154_dev *dev,
				    struct ieee802154_hw_addr_filt *filt,
				    unsigned long changed)
{
	struct adf7242_local *lp = dev->priv;
	u8 reg;

	dev_vdbg(&lp->spi->dev, "%s :Enter\n", __func__);
	dev_dbg(&lp->spi->dev, "%s :Changed=0x%lX\n", __func__, changed);

	might_sleep();

	if (changed & IEEE802515_AFILT_IEEEADDR_CHANGED) {
		u8 addr[8], i;
		memcpy(addr, &filt->ieee_addr, 8);

		for (i = 0; i < 8; i++)
			adf7242_write_reg(lp, REG_IEEE_ADDR_0 + i, addr[i]);

	}

	if (changed & IEEE802515_AFILT_SADDR_CHANGED) {
		u16 saddr = le16_to_cpu(filt->short_addr);
		adf7242_write_reg(lp, REG_SHORT_ADDR_0, saddr);
		adf7242_write_reg(lp, REG_SHORT_ADDR_1, saddr >> 8);
	}

	if (changed & IEEE802515_AFILT_PANID_CHANGED) {
		u16 pan_id = le16_to_cpu(filt->pan_id);
		adf7242_write_reg(lp, REG_PAN_ID0, pan_id);
		adf7242_write_reg(lp, REG_PAN_ID1, pan_id >> 8);
	}

	if (changed & IEEE802515_AFILT_PANC_CHANGED) {
		adf7242_read_reg(lp, REG_AUTO_CFG, &reg);
		if (filt->pan_coord)
			reg |= IS_PANCOORD;
		else
			reg &= ~IS_PANCOORD;
		adf7242_write_reg(lp, REG_AUTO_CFG, reg);
	}

	dev_vdbg(&lp->spi->dev, "%s :Exit\n", __func__);
	return 0;
}

static int adf7242_xmit(struct ieee802154_dev *dev, struct sk_buff *skb)
{
	struct adf7242_local *lp = dev->priv;
	int ret;

	dev_vdbg(&lp->spi->dev, "%s :Enter\n", __func__);

	reinit_completion(&lp->tx_complete);

	ret = adf7242_write_fbuf(lp, skb->data, skb->len);
	if (ret)
		goto err;

	if (lp->mode & ADF_IEEE802154_AUTO_CSMA_CA) {
		ret = adf7242_cmd(lp, CMD_RC_PHY_RDY);
		ret |= adf7242_cmd(lp, CMD_RC_CSMACA);
	} else {
		ret = adf7242_cmd(lp, CMD_RC_TX);
	}

	if (ret)
		goto err;

	ret = wait_for_completion_interruptible_timeout(&lp->tx_complete, 3*HZ);
	if (ret == -ERESTARTSYS)
		goto err;
	if (ret == 0) {
		dev_warn(&lp->spi->dev, "Timeout waiting for TX interrupt\n");
		ret = -ETIMEDOUT;
		goto err;
	}

	if (lp->tx_stat != SUCCESS) {
		dev_warn(&lp->spi->dev,
			 "Error xmit: Retry count exceeded Status=0x%x\n",
			 lp->tx_stat);
		ret = -ECOMM;
	} else {
		ret = 0;
	}

	dev_vdbg(&lp->spi->dev, "%s :Exit\n", __func__);

err:
	return ret;
}

static int adf7242_rx(struct adf7242_local *lp)
{
	size_t len = 128;
	u8 lqi = 0;
	int ret;
	struct sk_buff *skb;

	dev_vdbg(&lp->spi->dev, "%s :Enter\n", __func__);

	skb = alloc_skb(len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	ret = adf7242_read_fbuf(lp, skb_put(skb, len), &len, &lqi);

	adf7242_cmd(lp, CMD_RC_RX);

	skb_trim(skb, len - 2);	/* We do not put RSSI/LQI or CRC into the frame */

	if (len < 2) {
		kfree_skb(skb);
		return -EINVAL;
	}

	ieee802154_rx_irqsafe(lp->dev, skb, lqi);

	dev_dbg(&lp->spi->dev, "%s: ret=%d len=%d lqi=%d\n",
		__func__, ret, len, lqi);
	dev_vdbg(&lp->spi->dev, "%s :Exit\n", __func__);

	return 0;
}

static struct ieee802154_ops adf7242_ops = {
	.owner = THIS_MODULE,
	.xmit = adf7242_xmit,
	.ed = adf7242_ed,
	.set_channel = adf7242_channel,
	.set_hw_addr_filt = adf7242_set_hw_addr_filt,
	.start = adf7242_start,
	.stop = adf7242_stop,
	.set_csma_params = adf7242_set_csma_params,
	.set_frame_retries = adf7242_set_frame_retries,
	.set_txpower = adf7242_set_txpower,
};

static irqreturn_t adf7242_isr(int irq, void *data)
{
	struct adf7242_local *lp = data;
	u8 irq1, auto_stat = 0, stat = 0;
	int ret;

	dev_vdbg(&lp->spi->dev, "%s :Enter\n", __func__);

	ret = adf7242_read_reg(lp, REG_IRQ1_SRC1, &irq1);

	dev_dbg(&lp->spi->dev, "%s IRQ1 = %X:\n%s%s%s%s%s%s%s%s\n", __func__, irq1,
	    irq1 & IRQ_CCA_COMPLETE ? "IRQ_CCA_COMPLETE\n" : "",
	    irq1 & IRQ_SFD_RX ? "IRQ_SFD_RX\n" : "",
	    irq1 & IRQ_SFD_TX ? "IRQ_SFD_TX\n" : "",
	    irq1 & IRQ_RX_PKT_RCVD ? "IRQ_RX_PKT_RCVD\n" : "",
	    irq1 & IRQ_TX_PKT_SENT ? "IRQ_TX_PKT_SENT\n" : "",
	    irq1 & IRQ_CSMA_CA ? "IRQ_CSMA_CA\n" : "",
	    irq1 & IRQ_FRAME_VALID ? "IRQ_FRAME_VALID\n" : "",
	    irq1 & IRQ_ADDRESS_VALID ? "IRQ_ADDRESS_VALID\n" : "");

	adf7242_status(lp, &stat);

	dev_dbg(&lp->spi->dev, "%s STATUS = %X:\n%s\n%s%s%s%s%s\n", __func__, stat,
	    stat & STAT_RC_READY ? "RC_READY" : "RC_BUSY",
	    (stat & 0xf) == RC_STATUS_IDLE ? "RC_STATUS_IDLE" : "",
	    (stat & 0xf) == RC_STATUS_MEAS ? "RC_STATUS_MEAS" : "",
	    (stat & 0xf) == RC_STATUS_PHY_RDY ? "RC_STATUS_PHY_RDY" : "",
	    (stat & 0xf) == RC_STATUS_RX ? "RC_STATUS_RX" : "",
	    (stat & 0xf) == RC_STATUS_TX ? "RC_STATUS_TX" : "");

	adf7242_write_reg(lp, REG_IRQ1_SRC1, irq1);

	if (!(irq1 & (IRQ_RX_PKT_RCVD | lp->tx_irq)))
		dev_err(&lp->spi->dev, "%s :ERROR IRQ1 = 0x%X\n", __func__, irq1);

#if 0
	if ((irq1 & IRQ_RX_PKT_RCVD) &&
		((lp->mode & ADF_IEEE802154_REPORT_ACK) ||
		!(irq1 & lp->tx_irq))) {
#else
	if (irq1 & IRQ_RX_PKT_RCVD) {
#endif
		/* Wait until ACK is processed */
		if ((lp->mode & ADF_IEEE802154_HW_AACK) &&
		    ((stat & RC_STATUS_MASK) != RC_STATUS_PHY_RDY))
			adf7242_wait_status(lp, RC_STATUS_PHY_RDY);

		adf7242_rx(lp);
	}

	if (irq1 & lp->tx_irq) {

		if (lp->mode & ADF_IEEE802154_AUTO_CSMA_CA) {
			adf7242_read_reg(lp, REG_AUTO_STATUS, &auto_stat);
			auto_stat &= AUTO_STATUS_MASK;

			dev_dbg(&lp->spi->dev, "%s AUTO_STATUS = %X:\n%s%s%s%s\n",
			    __func__, auto_stat,
			    auto_stat == SUCCESS ? "SUCCESS" : "",
			    auto_stat ==
			    SUCCESS_DATPEND ? "SUCCESS_DATPEND" : "",
			    auto_stat == FAILURE_CSMACA ? "FAILURE_CSMACA" : "",
			    auto_stat == FAILURE_NOACK ? "FAILURE_NOACK" : "");

			/* save CSMA-CA completion status */
			lp->tx_stat = auto_stat;
		}

		complete(&lp->tx_complete);
		adf7242_cmd(lp, CMD_RC_RX);
	}

	dev_vdbg(&lp->spi->dev, "%s :Exit\n", __func__);

	return IRQ_HANDLED;
}

static int adf7242_hw_init(struct adf7242_local *lp)
{
	int ret;
	const struct firmware *fw;

	dev_vdbg(&lp->spi->dev, "%s :Enter\n", __func__);


	adf7242_cmd(lp, CMD_RC_RESET);
	adf7242_cmd(lp, CMD_RC_IDLE);

	if (lp->mode) {
		/* get ADF7242 addon firmware
		 * build this driver as module
		 * and place under /lib/firmware/adf7242_firmware.bin
		 */
		ret = request_firmware(&fw, FIRMWARE, &lp->spi->dev);
		if (ret) {
			dev_err(&lp->spi->dev,
				"request_firmware() failed with %d\n", ret);
			return ret;
		}

		ret = adf7242_upload_firmware(lp, (u8 *) fw->data, fw->size);
		if (ret) {
			dev_err(&lp->spi->dev,
				"upload firmware failed with %d\n", ret);
			return ret;
		}

		ret = adf7242_verify_firmware(lp, (u8 *) fw->data, fw->size);
		if (ret) {
			dev_err(&lp->spi->dev,
				"verify firmware failed with %d\n", ret);
			return ret;
		}

		adf7242_cmd(lp, CMD_RC_PC_RESET);

		release_firmware(fw);

		adf7242_write_reg(lp, REG_FFILT_CFG,
				  ACCEPT_BEACON_FRAMES |
				  ACCEPT_DATA_FRAMES |
				  ACCEPT_MACCMD_FRAMES |
				  (lp->mode & ADF_IEEE802154_PROMISCUOUS_MODE ?
				   ACCEPT_ALL_ADDRESS : 0) |
				  (lp->mode & ADF_IEEE802154_REPORT_ACK ?
				   ACCEPT_ACK_FRAMES : 0) |
				   ACCEPT_RESERVED_FRAMES);

		adf7242_set_csma_params(lp->dev, lp->pdata->min_csma_be,
					lp->pdata->max_csma_be,
					lp->pdata->max_cca_retries);
		adf7242_set_frame_retries(lp->dev, lp->pdata->max_frame_retries);

		adf7242_write_reg(lp, REG_AUTO_CFG,
				  (lp->mode & ADF_IEEE802154_HW_AACK ?
				   RX_AUTO_ACK_EN | CSMA_CA_RX_TURNAROUND : 0));
	}

	adf7242_write_reg(lp, REG_PKT_CFG, (lp->mode ? ADDON_EN : 0) | BIT(2));

	adf7242_write_reg(lp, REG_EXTPA_MSC, 0xF1);
	adf7242_write_reg(lp, REG_RXFE_CFG, 0x1D);
	adf7242_write_reg(lp, REG_IRQ1_EN0, 0);

	adf7242_write_reg(lp, REG_IRQ1_EN1, IRQ_RX_PKT_RCVD | lp->tx_irq);

	adf7242_write_reg(lp, REG_IRQ1_SRC1, 0xFF);
	adf7242_write_reg(lp, REG_IRQ1_SRC0, 0xFF);

	adf7242_cmd(lp, CMD_RC_PHY_RDY);

	dev_vdbg(&lp->spi->dev, "%s :Exit\n", __func__);

	return 0;
}

static int adf7242_suspend(struct spi_device *spi, pm_message_t message)
{
	return 0;
}

static int adf7242_resume(struct spi_device *spi)
{
	return 0;
}

static ssize_t adf7242_show(struct device *dev,
			    struct device_attribute *devattr, char *buf)
{
	struct adf7242_local *lp = dev_get_drvdata(dev);
	u8 stat, irq1;
	size_t len;

	adf7242_status(lp, &stat);

	adf7242_read_reg(lp, REG_IRQ1_SRC1, &irq1);

	len = sprintf(buf, "IRQ1 = %X:\n%s%s%s%s%s%s%s%s\n", irq1,
	    irq1 & IRQ_CCA_COMPLETE ? "IRQ_CCA_COMPLETE\n" : "",
	    irq1 & IRQ_SFD_RX ? "IRQ_SFD_RX\n" : "",
	    irq1 & IRQ_SFD_TX ? "IRQ_SFD_TX\n" : "",
	    irq1 & IRQ_RX_PKT_RCVD ? "IRQ_RX_PKT_RCVD\n" : "",
	    irq1 & IRQ_TX_PKT_SENT ? "IRQ_TX_PKT_SENT\n" : "",
	    irq1 & IRQ_CSMA_CA ? "IRQ_CSMA_CA\n" : "",
	    irq1 & IRQ_FRAME_VALID ? "IRQ_FRAME_VALID\n" : "",
	    irq1 & IRQ_ADDRESS_VALID ? "IRQ_ADDRESS_VALID\n" : "");


	len += sprintf(buf + len, "STATUS = %X:\n%s\n%s%s%s%s%s\n", stat,
		       stat & STAT_RC_READY ? "RC_READY" : "RC_BUSY",
		       (stat & 0xf) == RC_STATUS_IDLE ? "RC_STATUS_IDLE" : "",
		       (stat & 0xf) == RC_STATUS_MEAS ? "RC_STATUS_MEAS" : "",
		       (stat & 0xf) ==
				RC_STATUS_PHY_RDY ? "RC_STATUS_PHY_RDY" : "",
		       (stat & 0xf) == RC_STATUS_RX ? "RC_STATUS_RX" : "",
		       (stat & 0xf) == RC_STATUS_TX ? "RC_STATUS_TX" : "");

	return len;
}

static DEVICE_ATTR(status, 0664, adf7242_show, NULL);

static struct attribute *adf7242_attributes[] = {
	&dev_attr_status.attr,
	NULL
};

static const struct attribute_group adf7242_attr_group = {
	.attrs = adf7242_attributes,
};

#ifdef CONFIG_OF
static struct adf7242_platform_data *adf7242_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct adf7242_platform_data *pdata;
	u32 tmp;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return NULL;
	}

	pdata->mode |= of_property_read_bool(np, "adi,hw-aack-mode-enable") ?
		ADF_IEEE802154_HW_AACK : 0;

	pdata->mode |=
		of_property_read_bool(np, "adi,auto-csma-ca-mode-enable") ?
		ADF_IEEE802154_AUTO_CSMA_CA : 0;

	pdata->mode |=
		of_property_read_bool(np, "adi,promiscuous-mode-enable") ?
		ADF_IEEE802154_PROMISCUOUS_MODE : 0;

	pdata->mode |=
		of_property_read_bool(np, "adi,report-ack-frames-enable") ?
		ADF_IEEE802154_REPORT_ACK : 0;

	tmp = 4;
	of_property_read_u32(np, "adi,max-frame-retries", &tmp);
	pdata->max_frame_retries = tmp;

	tmp = 4;
	of_property_read_u32(np, "adi,max-cca-retries", &tmp);
	pdata->max_cca_retries = tmp;

	tmp = 6;
	of_property_read_u32(np, "adi,max-csma-back-off-exponent", &tmp);
	pdata->max_csma_be = tmp;

	tmp = 1;
	of_property_read_u32(np, "adi,min-csma-back-off-exponent", &tmp);
	pdata->min_csma_be = tmp;

	return pdata;
}
#else
static
struct adf7242_platform_data *adf7242_parse_dt(struct device *dev)
{
	return NULL;
}
#endif

static int adf7242_probe(struct spi_device *spi)
{
	struct adf7242_platform_data *pdata;
	struct ieee802154_dev *dev;
	struct adf7242_local *lp;
	int ret;

	if (!spi->irq) {
		dev_err(&spi->dev, "no IRQ specified\n");
		return -EINVAL;
	}

	if (spi->dev.of_node)
		pdata = adf7242_parse_dt(&spi->dev);
	else
		pdata = spi->dev.platform_data;

	if (!pdata) {
		dev_err(&spi->dev, "no platform data? using default\n");
		pdata = &adf7242_default_pdata;
	}

	dev = ieee802154_alloc_device(sizeof(*lp), &adf7242_ops);
	if (!dev)
		return -ENOMEM;

	lp = dev->priv;
	lp->dev = dev;
	lp->spi = spi;
	lp->pdata = pdata;
	lp->mode = pdata->mode;

	dev->priv = lp;
	dev->parent = &spi->dev;
	dev->extra_tx_headroom = 0;
	/* We do support only 2.4 Ghz */

	dev->phy->channels_supported[0] = 0x7FFF800;

	if (!lp->mode) {
		adf7242_ops.set_hw_addr_filt = NULL;
		lp->tx_irq = IRQ_TX_PKT_SENT;
	} else {
		if ((lp->mode & ADF_IEEE802154_PROMISCUOUS_MODE) &&
		    (lp->mode & ADF_IEEE802154_HW_AACK))
			lp->mode &= ~ADF_IEEE802154_HW_AACK;

		if (lp->mode & ADF_IEEE802154_AUTO_CSMA_CA)
			lp->tx_irq = IRQ_CSMA_CA;
		else
			lp->tx_irq = IRQ_TX_PKT_SENT;
	}

	dev->flags = IEEE802154_HW_OMIT_CKSUM |
		(lp->mode & ADF_IEEE802154_HW_AACK ? IEEE802154_HW_AACK : 0);

	mutex_init(&lp->bmux);
	init_completion(&lp->tx_complete);

	/* Setup Status Message */
	lp->stat_xfer.len = 1;
	lp->stat_xfer.tx_buf = &lp->buf_stat_tx;
	lp->stat_xfer.rx_buf = &lp->buf_stat_rx;
	lp->buf_stat_tx = CMD_SPI_NOP;

	spi_message_init(&lp->stat_msg);
	spi_message_add_tail(&lp->stat_xfer, &lp->stat_msg);

	spi_set_drvdata(spi, lp);

	ret = adf7242_hw_init(lp);
	if (ret)
		goto err_hw_init;

	ret = devm_request_threaded_irq(&spi->dev, spi->irq, NULL,
				adf7242_isr, IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				dev_name(&spi->dev), lp);

	if (ret)
		goto err_hw_init;

	ret = ieee802154_register_device(lp->dev);
	if (ret)
		goto err_hw_init;

	dev_set_drvdata(&spi->dev, lp);

	ret = sysfs_create_group(&spi->dev.kobj, &adf7242_attr_group);
	if (ret)
		goto out;

	dev_info(&spi->dev, "mac802154 IRQ-%d registered\n", spi->irq);

	return ret;

out:
	ieee802154_unregister_device(lp->dev);
err_hw_init:
	mutex_destroy(&lp->bmux);
	ieee802154_free_device(lp->dev);
	return ret;
}

static int adf7242_remove(struct spi_device *spi)
{
	struct adf7242_local *lp = spi_get_drvdata(spi);

	ieee802154_unregister_device(lp->dev);
	mutex_destroy(&lp->bmux);
	ieee802154_free_device(lp->dev);

	return 0;
}

static const struct of_device_id adf7242_of_match[] = {
	{ .compatible = "adi,adf7242", },
	{ },
};
MODULE_DEVICE_TABLE(of, adf7242_of_match);

static const struct spi_device_id adf7242_device_id[] = {
	{ .name = "adf7242", },
	{ },
};
MODULE_DEVICE_TABLE(spi, adf7242_device_id);

static struct spi_driver adf7242_driver = {
	.id_table = adf7242_device_id,
	.driver = {
		   .of_match_table = of_match_ptr(adf7242_of_match),
		   .name = "adf7242",
		   .owner = THIS_MODULE,
		   },
	.probe = adf7242_probe,
	.remove = adf7242_remove,
	.suspend = adf7242_suspend,
	.resume = adf7242_resume,
};

module_spi_driver(adf7242_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("ADF7242 IEEE802.15.4 Transceiver Driver");
MODULE_LICENSE("GPL");
