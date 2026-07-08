// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices ADEMA124 / ADEMA127 poly-phase energy metering ADCs
 *
 * Copyright 2026 Analog Devices Inc.
 *
 * Datasheet: ADEMA124/ADEMA127 (Rev. A, 11/2025)
 *
 * Supported parts:
 *   ADEMA124  – 4-channel, 24-bit sigma-delta, PRODUCT_ID 0x13
 *   ADEMA127  – 7-channel, 24-bit sigma-delta, PRODUCT_ID 0x16
 *
 * Two sample-capture paths are supported:
 *   1) Software-triggered IIO buffer: one long-frame SPI read per DREADY IRQ
 *      (works on any SPI controller; CPU-heavy at 32 kSPS).
 *   2) SPI Offload framework: the SPI controller is programmed to autonomously
 *      shift the STATUS2 long-frame command on every hardware trigger and
 *      stream the response to a DMA channel. Selected automatically when the
 *      underlying SPI controller advertises SPI_OFFLOAD_CAP_TRIGGER +
 *      SPI_OFFLOAD_CAP_RX_STREAM_DMA.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/crc-itu-t.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/units.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include "adema127.h"

/* --- Command word bits [31:0] -------------------------------------------- */
#define ADEMA_CMD_RWB			BIT(31)		/* 1 = read */
#define ADEMA_CMD_LONG			BIT(30)		/* 1 = long frame */
#define ADEMA_CMD_ADDR			GENMASK(29, 16)
#define ADEMA_CMD_DATA			GENMASK(15, 8)
#define ADEMA_CMD_CRC			GENMASK(7, 0)

/* --- MMR registers ------------------------------------------------------- */
#define ADEMA_REG_SWRST			0x001
#define ADEMA_SWRST_KEY			0xD6

#define ADEMA_REG_CONFIG0		0x002
#define ADEMA_CONFIG0_ADC_POWER_MODE	GENMASK(7, 6)
#define ADEMA_CONFIG0_REF_PD_HP_REF	BIT(5)
#define ADEMA_CONFIG0_REF_PD_BUFFER	BIT(4)
#define ADEMA_CONFIG0_STREAM_DBG	GENMASK(3, 2)
#define ADEMA_CONFIG0_CRC_EN_SPI_WRITE	BIT(1)
#define ADEMA_CONFIG0_CLKOUT_EN		BIT(0)

#define ADEMA_REG_ADC_PD		0x004
#define ADEMA_REG_ADC_CMI		0x005
#define ADEMA_REG_ADC_GAIN		0x006
#define ADEMA_REG_ADC_INV		0x007

#define ADEMA_REG_ACCESS_EXTENDED_MMAP	0x012
#define ADEMA_ACCESS_DSP_MEM_ACCESS_REQ	BIT(0)

#define ADEMA_REG_SCRATCH		0x013

#define ADEMA_REG_SYNC_SNAP		0x014
#define ADEMA_SYNC_SNAP_PREP_BROADCAST	BIT(2)
#define ADEMA_SYNC_SNAP_ALIGN		BIT(1)
#define ADEMA_SYNC_SNAP_SNAPSHOT	BIT(0)

#define ADEMA_REG_MASK0			0x019
#define ADEMA_REG_MASK1			0x01A
#define ADEMA_REG_MASK2			0x01B

#define ADEMA_REG_WR_LOCK		0x01F
#define ADEMA_WR_LOCK_UNLOCK		0x5E
#define ADEMA_WR_LOCK_LOCK		0xD4

#define ADEMA_REG_STATUS0		0x020
#define ADEMA_STATUS0_STATUS1X		BIT(7)
#define ADEMA_STATUS0_STATUS2X		BIT(6)
#define ADEMA_STATUS0_RESET_DONE	BIT(5)
#define ADEMA_STATUS0_CRC_CHG_MMR_RET	BIT(4)
#define ADEMA_STATUS0_CRC_CHG_MMR	BIT(3)
#define ADEMA_STATUS0_EFUSE_MEM_ERR	BIT(2)
#define ADEMA_STATUS0_SPI_CRC_ERR	BIT(1)

#define ADEMA_REG_STATUS1		0x021
#define ADEMA_STATUS1_CH_OVRNG(x)	BIT(x)

#define ADEMA_STATUS2_DSP_MEM_READY	BIT(0)

/* Waveform registers: CHx_WAV_HI/MD/LO at 0x026 + 3*x .. */
#define ADEMA_REG_CH_WAV_HI(x)		(0x026 + 3 * (x))

#define ADEMA_REG_DATAPATH_CONFIG_LOCK	0x03B
#define ADEMA_REG_DATARATE		0x03C
#define ADEMA_DATARATE_DSP_DECIMATION_X2	BIT(7)
#define ADEMA_DATARATE_ADC_CLK_PRESCALER	GENMASK(6, 4)
#define ADEMA_DATARATE_DECIMATION_RATE		GENMASK(3, 0)

/* Two channels per register: even channel in [3:0], odd in [7:4]. */
#define ADEMA_REG_DATAPATH_ALPHA(x)	(0x03D + ((x) / 2))
#define ADEMA_ALPHA_MASK(x)		((x) & 1 ? GENMASK(7, 4) : GENMASK(3, 0))
#define ADEMA_REG_DATAPATH_CFG(x)	(0x041 + (x))
#define ADEMA_DATAPATH_CFG_ALLPASS_EN	BIT(6)
#define ADEMA_DATAPATH_CFG_LPF_EN	BIT(5)
#define ADEMA_DATAPATH_CFG_COMP_FLT_CFG	BIT(4)
#define ADEMA_DATAPATH_CFG_COMP_FLT_EN	BIT(3)
#define ADEMA_DATAPATH_CFG_HPF_EN	BIT(2)
#define ADEMA_DATAPATH_CFG_SCF_EN	BIT(1)
#define ADEMA_DATAPATH_CFG_GAIN_OFFSET_XT_EN BIT(0)

#define ADEMA_REG_PHASE_OFFSET_HI(x)	(0x048 + 2 * (x))
#define ADEMA_REG_PHASE_OFFSET_LO(x)	(0x049 + 2 * (x))
#define ADEMA_PHASE_OFFSET_BITS		13
#define ADEMA_PHASE_OFFSET_MAX		((1 << ADEMA_PHASE_OFFSET_BITS) - 1)

#define ADEMA_REG_PRODUCT_ID		0x07E
#define ADEMA_PRODUCT_ID_ADEMA124	0x13
#define ADEMA_PRODUCT_ID_ADEMA127	0x16

/* --- DSP RAM (per-channel calibration) ----------------------------------- */
/* Base per channel: CH0=0x401, CH1=0x441, ... (stride 0x40). */
#define ADEMA_DSP_CH_BASE(x)		(0x401 + 0x40 * (x))
#define ADEMA_DSP_OFF_SHIFT		0x1C	/* 1 byte */
#define ADEMA_DSP_OFF_GAIN_LO		0x20	/* 3 bytes: LO, MD, HI */
#define ADEMA_DSP_OFF_OFFSET_LO		0x24	/* 3 bytes */
#define ADEMA_DSP_OFF_XT_GAIN_LO	0x28	/* 3 bytes */
#define ADEMA_DSP_OFF_XT_AGGRESSOR	0x2C	/* 1 byte */

/* --- CRC parameters ------------------------------------------------------ */
#define ADEMA_CMD_CRC_POLY		0x07
#define ADEMA_CMD_CRC_XOR_OUT		0x55

/* --- Timing / power / reference ------------------------------------------ */
#define ADEMA_STARTUP_US		1000	/* fast start-up ≤ 0.5 ms */
#define ADEMA_DATAPATH_LOCK_US		50	/* ROM→RAM copy time */
#define ADEMA_INTERNAL_REF_MV		1250	/* 1.25 V band-gap */
#define ADEMA_XTAL_HZ_NOMINAL		16384000

/*
 * Datasheet Table 2: at 1× gain the ADC delivers 4,772,275 codes per volt
 * of differential input. The scale attribute is therefore expressed as the
 * exact rational number 1000 / (4,772,275 × input_gain) millivolts per LSB.
 */
#define ADEMA_CODES_PER_VOLT_1X		4772275
#define ADEMA_REALBITS			24
#define ADEMA_STORAGEBITS		32

/* Long-frame response byte offsets (offset of the first transmitted byte,
 * WAV_LO, of Vx_WAV inside the 20/32-byte response). Byte 0 of the
 * response is CMD_ECHO.
 *   ADEMA127: echo(1) V0(3) STAT0(1) V1(3) STAT1(1) V2(3) rsvd(1) V3(3) rsvd(1)
 *             V4(3) rsvd(1) V5(3) rsvd(1) V6(3) RDD1(1) RDD0(1) CRC(2)
 *   ADEMA124: echo(1) V0(3) STAT0(1) V1(3) STAT1(1) V2(3) rsvd(1) V3(3)
 *             RDD1(1) RDD0(1) CRC(2)
 */
static const u8 adema127_wav_off[7] = { 1, 5, 9, 13, 17, 21, 25 };
static const u8 adema124_wav_off[4] = { 1, 5, 9, 13 };

#define ADEMA127_STATUS0_OFF		4
#define ADEMA127_STATUS1_OFF		8

#define ADEMA124_STATUS0_OFF		4
#define ADEMA124_STATUS1_OFF		8

/* ------------------------------------------------------------------------ */
/*  CRC helpers                                                             */
/* ------------------------------------------------------------------------ */

/*
 * Command CRC: polynomial x^8 + x^2 + x + 1 (0x07), seed 0x00, final XOR 0x55.
 * Computed over the top 24 bits of the command word (bits [31:8]).
 */
static u8 adema_cmd_crc(const u8 *data, size_t len)
{
	u8 crc = 0;
	size_t i;
	unsigned int j;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (j = 0; j < 8; j++)
			crc = (crc & 0x80) ? (u8)((crc << 1) ^ ADEMA_CMD_CRC_POLY)
					   : (u8)(crc << 1);
	}
	return crc ^ ADEMA_CMD_CRC_XOR_OUT;
}

/*
 * Response CRC: CRC-CCITT-FALSE (poly 0x1021 non-reflected, seed 0xFFFF,
 * no final XOR). Note: Linux's crc_ccitt() is the KERMIT variant (reflected,
 * seed 0) — do NOT use it here. crc_itu_t() with seed 0xFFFF is CCITT-FALSE.
 *
 * The 16-bit CRC is transmitted little-endian on the wire — the LSB byte
 * arrives first (byte 4 of the response frame), MSB byte second (byte 5).
 */
static u16 adema_resp_crc(const u8 *data, size_t len)
{
	return crc_itu_t(0xFFFF, data, len);
}

static void adema_build_cmd(u8 buf[ADEMA_CMD_LEN], bool read, bool long_frame,
			    u16 addr, u8 data)
{
	buf[0] = (read ? 0x80 : 0x00) | (long_frame ? 0x40 : 0x00) |
		 ((addr >> 8) & 0x3F);
	buf[1] = addr & 0xFF;
	buf[2] = data;
	buf[3] = adema_cmd_crc(buf, 3);
}

/* ------------------------------------------------------------------------ */
/*  regmap bus – short frame register access                                */
/* ------------------------------------------------------------------------ */

/*
 * The ADEMA short frame is 6 bytes on the wire. TX is the 4-byte command
 * followed by 2 don't-care bytes. RX holds the response to the *previous*
 * command: echo, STATUS0, RDDATA1 (reg+1), RDDATA0 (reg), CRC16 (2 bytes).
 *
 * Every regmap access therefore performs two 6-byte transfers:
 *   1) issue the desired command and clock out the previous response (unused)
 *   2) re-send a benign command (READ SCRATCH) and clock in this command's
 *      response.
 */
static bool adema_debug_wire;
module_param_named(debug_wire, adema_debug_wire, bool, 0644);
MODULE_PARM_DESC(debug_wire,
		 "log every SPI short-frame TX/RX at dev_err level (default off)");

static bool adema_ignore_crc;
module_param_named(ignore_crc, adema_ignore_crc, bool, 0644);
MODULE_PARM_DESC(ignore_crc,
		 "do not fail on response CRC mismatch — bring-up debug only");

static int adema_short_xfer(struct adema_state *st, bool read, u16 addr, u8 data)
{
	struct spi_transfer xfer = {
		.tx_buf = st->reg_tx,
		.rx_buf = st->reg_rx,
		.len = ADEMA_SHORT_FRAME_LEN,
	};
	int ret;

	/*
	 * Short-frame TX layout per ADEMA124/127 datasheet Figure 43 and the
	 * ADI NoOS energy-adc-service reference (ADI_ADC_CMD_SHORT_FRAME_ADEMA12X):
	 *   [dummy(2)] [cmd(4)]
	 * The response to the *previous* command is shifted in during the same
	 * 48-bit CS-low window and occupies all 6 RX bytes:
	 *   [echo(1)] [status0(1)] [data(2)] [crc(2)]
	 */
	st->reg_tx[0] = 0;
	st->reg_tx[1] = 0;
	adema_build_cmd(&st->reg_tx[2], read, false, addr, data);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (adema_debug_wire)
		dev_err(&st->spi->dev,
			"short xfer %s addr=%04x data=%02x ret=%d tx=%*ph rx=%*ph\n",
			read ? "RD" : "WR", addr, data, ret,
			ADEMA_SHORT_FRAME_LEN, st->reg_tx,
			ADEMA_SHORT_FRAME_LEN, st->reg_rx);
	return ret;
}

static int adema_verify_resp_crc(struct adema_state *st)
{
	u16 got = get_unaligned_le16(&st->reg_rx[4]);
	u16 want = adema_resp_crc(st->reg_rx, 4);

	if (got != want) {
		/*
		 * Rate-limited: a persistent wire issue can fire this every
		 * SPI transaction (up to hundreds of Hz on the register path
		 * during buffered captures). At 115200 baud the unrate-limited
		 * ~130-byte line takes ~11 ms/msg, saturating the serial
		 * console; that in turn stalls the printk kthread and can
		 * cascade into apparent kernel wedges long after the
		 * underlying SPI problem started.
		 */
		dev_err_ratelimited(&st->spi->dev,
			"short-frame CRC mismatch got=%04x want=%04x rx=%*ph tx=%*ph%s\n",
			got, want,
			ADEMA_SHORT_FRAME_LEN, st->reg_rx,
			ADEMA_SHORT_FRAME_LEN, st->reg_tx,
			adema_ignore_crc ? " (IGNORED)" : "");
		if (!adema_ignore_crc)
			return -EBADMSG;
	}
	return 0;
}

static int adema_regmap_reg_read(void *ctx, unsigned int reg, unsigned int *val)
{
	struct adema_state *st = ctx;
	int ret;

	guard(mutex)(&st->lock);

	/* Phase 1: send READ(reg). RX contains previous frame's payload. */
	ret = adema_short_xfer(st, true, reg, 0);
	if (ret)
		return ret;

	/*
	 * Phase 2: send a harmless READ(SCRATCH). RX now contains RDDATA0
	 * (= reg) and RDDATA1 (= reg + 1) plus STATUS0 and CRC16.
	 */
	ret = adema_short_xfer(st, true, ADEMA_REG_SCRATCH, 0);
	if (ret)
		return ret;

	ret = adema_verify_resp_crc(st);
	if (ret)
		return ret;

	/* RDDATA0 lives at byte 3 of the response. */
	*val = st->reg_rx[3];
	if (adema_debug_wire)
		dev_err(&st->spi->dev,
			"reg_read(0x%03x) = 0x%02x  (echo=%02x status0=%02x rddata1=%02x)\n",
			reg, *val, st->reg_rx[0], st->reg_rx[1], st->reg_rx[2]);
	return 0;
}

static int adema_regmap_reg_write(void *ctx, unsigned int reg, unsigned int val)
{
	struct adema_state *st = ctx;
	int ret;

	guard(mutex)(&st->lock);

	ret = adema_short_xfer(st, false, reg, val);
	if (ret)
		return ret;

	/* Follow-up read to shift out the write's echo/status. */
	ret = adema_short_xfer(st, true, ADEMA_REG_SCRATCH, 0);
	if (ret)
		return ret;

	return adema_verify_resp_crc(st);
}

static bool adema_regmap_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ADEMA_REG_STATUS0:
	case ADEMA_REG_STATUS1:
	case ADEMA_REG_STATUS2:
	case ADEMA_REG_SYNC_SNAP:
	case ADEMA_REG_ACCESS_EXTENDED_MMAP:
	case 0x026 ... 0x03A:	/* Vx_WAV_HI/MD/LO */
		return true;
	default:
		return false;
	}
}

static const struct regmap_config adema_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0x623,
	.reg_read	= adema_regmap_reg_read,
	.reg_write	= adema_regmap_reg_write,
	.volatile_reg	= adema_regmap_volatile,
	.cache_type	= REGCACHE_NONE,
	.can_sleep	= true,
};

/* ------------------------------------------------------------------------ */
/*  DSP RAM window helpers                                                  */
/* ------------------------------------------------------------------------ */

/*
 * DSP RAM (addresses ≥ 0x401) is only visible while DSP_MEM_ACCESS_REQ is
 * asserted and STATUS2.DSP_MEM_ACCESS_READY reads 1.
 */
static int adema_dsp_ram_enter(struct adema_state *st)
{
	unsigned int status;
	int ret;

	ret = regmap_write(st->regmap, ADEMA_REG_ACCESS_EXTENDED_MMAP,
			   ADEMA_ACCESS_DSP_MEM_ACCESS_REQ);
	if (ret)
		return ret;

	/*
	 * The window refuses to open for a few milliseconds while the chip
	 * reloads DSP RAM defaults (triggered by enabling a DSP filter with
	 * the config lock set), so the timeout must ride that out.
	 */
	return regmap_read_poll_timeout(st->regmap, ADEMA_REG_STATUS2, status,
					status & ADEMA_STATUS2_DSP_MEM_READY,
					10, 100000);
}

static int adema_dsp_ram_leave(struct adema_state *st)
{
	return regmap_write(st->regmap, ADEMA_REG_ACCESS_EXTENDED_MMAP, 0);
}

static int adema_dsp_ram_read24(struct adema_state *st, u16 base_lo, u32 *val)
{
	unsigned int lo, md, hi;
	int ret;

	guard(mutex)(&st->dsp_lock);

	ret = adema_dsp_ram_enter(st);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, base_lo,     &lo);
	if (ret)
		goto out;
	ret = regmap_read(st->regmap, base_lo + 1, &md);
	if (ret)
		goto out;
	ret = regmap_read(st->regmap, base_lo + 2, &hi);
	if (ret)
		goto out;

	*val = ((hi & 0xFFu) << 16) | ((md & 0xFFu) << 8) | (lo & 0xFFu);
out:
	adema_dsp_ram_leave(st);
	return ret;
}

static int adema_dsp_ram_write24(struct adema_state *st, u16 base_lo, u32 val)
{
	int ret;

	guard(mutex)(&st->dsp_lock);

	ret = adema_dsp_ram_enter(st);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, base_lo,     val & 0xFF);
	if (ret)
		goto out;
	ret = regmap_write(st->regmap, base_lo + 1, (val >> 8) & 0xFF);
	if (ret)
		goto out;
	ret = regmap_write(st->regmap, base_lo + 2, (val >> 16) & 0xFF);
out:
	adema_dsp_ram_leave(st);
	return ret;
}

static int adema_dsp_ram_read8(struct adema_state *st, u16 addr, unsigned int *val)
{
	int ret;

	guard(mutex)(&st->dsp_lock);

	ret = adema_dsp_ram_enter(st);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, addr, val);
	adema_dsp_ram_leave(st);
	return ret;
}

static int adema_dsp_ram_write8(struct adema_state *st, u16 addr, u8 val)
{
	int ret;

	guard(mutex)(&st->dsp_lock);

	ret = adema_dsp_ram_enter(st);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, addr, val);
	adema_dsp_ram_leave(st);
	return ret;
}

/* ------------------------------------------------------------------------ */
/*  Datapath / config lock helpers                                          */
/* ------------------------------------------------------------------------ */

static int adema_config_unlock(struct adema_state *st)
{
	int ret;

	ret = regmap_write(st->regmap, ADEMA_REG_WR_LOCK, ADEMA_WR_LOCK_UNLOCK);
	if (ret)
		return ret;

	return regmap_write(st->regmap, ADEMA_REG_DATAPATH_CONFIG_LOCK, 0);
}

static int adema_config_lock(struct adema_state *st)
{
	int ret;

	ret = regmap_write(st->regmap, ADEMA_REG_DATAPATH_CONFIG_LOCK, 1);
	if (ret)
		return ret;

	/* Arm delay per §Configuration Procedure. */
	fsleep(ADEMA_DATAPATH_LOCK_US);

	/*
	 * Re-arm the register write protection (the mirror of the
	 * ADEMA_WR_LOCK_UNLOCK write in adema_config_unlock()); without
	 * this the protection stays permanently off after the first
	 * configuration change.
	 */
	return regmap_write(st->regmap, ADEMA_REG_WR_LOCK,
			    ADEMA_WR_LOCK_LOCK);
}

/* ------------------------------------------------------------------------ */
/*  Sampling-rate lookup                                                    */
/* ------------------------------------------------------------------------ */

struct adema_datarate {
	u32 fs;			/* sample rate at 16.384 MHz XTAL */
	u8 datarate;
};

static const struct adema_datarate adema_datarates[] = {
	{ 64000, 0x30 },
	{ 32000, 0x31 },
	{ 16000, 0x32 },
	{  8000, 0x33 },
	{  4000, 0x34 },
	{  2000, 0x35 },
	{  1000, 0x36 },
	{   500, 0x37 },
	{   250, 0x38 },
};

/*
 * Scale a nominal-XTAL sample rate to the actual XTAL. Full-precision
 * rational scaling (rounded to nearest) so non-integer XTAL ratios
 * (e.g. 24.576 MHz = 1.5x nominal) map correctly — the DATARATE
 * divider chain scales linearly with the crystal.
 */
static int adema_scale_fs(const struct adema_state *st, u32 fs_nominal)
{
	return DIV_ROUND_CLOSEST_ULL((u64)fs_nominal * st->xtal_hz,
				     ADEMA_XTAL_HZ_NOMINAL);
}

static void adema_init_sampling_freqs(struct adema_state *st)
{
	unsigned int i, n = ARRAY_SIZE(adema_datarates);

	BUILD_BUG_ON(ARRAY_SIZE(adema_datarates) !=
		     ARRAY_SIZE(st->sampling_freqs));

	/* adema_datarates[] is descending; present the list ascending. */
	for (i = 0; i < n; i++)
		st->sampling_freqs[i] =
			adema_scale_fs(st, adema_datarates[n - 1 - i].fs);
}

static int adema_lookup_datarate(const struct adema_state *st, u32 fs, u8 *reg)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(adema_datarates); i++) {
		if (adema_scale_fs(st, adema_datarates[i].fs) == fs) {
			*reg = adema_datarates[i].datarate;
			return 0;
		}
	}
	return -EINVAL;
}

static int adema_set_sampling_freq(struct adema_state *st, u32 fs)
{
	u8 datarate;
	int ret;

	ret = adema_lookup_datarate(st, fs, &datarate);
	if (ret)
		return ret;

	ret = adema_config_unlock(st);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADEMA_REG_DATARATE, datarate);
	if (ret)
		return ret;

	ret = adema_config_lock(st);
	if (ret)
		return ret;

	st->sampling_freq = fs;
	return 0;
}

/* ------------------------------------------------------------------------ */
/*  Long-frame builders (streaming and single-shot capture)                 */
/* ------------------------------------------------------------------------ */

static void adema_build_long_stream_cmd(struct adema_state *st)
{
	unsigned int cmd_off = st->chip_info->long_frame_len - ADEMA_CMD_LEN;

	/*
	 * Long-frame TX layout per NoOS reference (see
	 * ADI_ADC_CMD_LONG_FRAME_ADEMA127 / ADEMA124): the 4-byte command
	 * lives in the LAST four bytes of the frame (bytes 28-31 for ADEMA127,
	 * 16-19 for ADEMA124). Everything before is transmitted as zeros; the
	 * previous command's response payload streams in on MISO during those
	 * dummy bytes.
	 */
	memset(st->long_tx, 0, st->chip_info->long_frame_len);
	adema_build_cmd(&st->long_tx[cmd_off], true, true, ADEMA_STREAM_CMD_ADDR, 0);
}

static int adema_verify_long_crc(struct adema_state *st)
{
	unsigned int len = st->chip_info->long_frame_len;
	u16 got = get_unaligned_le16(&st->long_rx[len - 2]);
	u16 want = adema_resp_crc(st->long_rx, len - 2);

	if (got != want) {
		dev_dbg_ratelimited(&st->spi->dev,
				    "long-frame CRC mismatch got=%04x want=%04x\n",
				    got, want);
		return -EBADMSG;
	}
	return 0;
}

/* ------------------------------------------------------------------------ */
/*  IIO channel description                                                 */
/* ------------------------------------------------------------------------ */

enum {
	ADEMA_EXT_INFO_XT_GAIN,
	ADEMA_EXT_INFO_XT_AGGRESSOR,
	ADEMA_EXT_INFO_SHIFT,
	ADEMA_EXT_INFO_DC_BLOCK_ALPHA,
	ADEMA_EXT_INFO_HPF_EN,
	ADEMA_EXT_INFO_LPF_EN,
	ADEMA_EXT_INFO_SCF_EN,
	ADEMA_EXT_INFO_COMP_FLT_EN,
	ADEMA_EXT_INFO_COMP_FLT_CFG,
	ADEMA_EXT_INFO_ALLPASS_EN,
	ADEMA_EXT_INFO_GAIN_OFFSET_XT_EN,
	ADEMA_EXT_INFO_INPUT_GAIN,
	ADEMA_EXT_INFO_INPUT_INVERT,
};

static int adema_read_channel_raw(struct adema_state *st, unsigned int ch, s32 *out)
{
	unsigned int hi, md, lo;
	u32 v;
	int ret;

	ret = regmap_read(st->regmap, ADEMA_REG_CH_WAV_HI(ch), &hi);
	if (ret)
		return ret;
	ret = regmap_read(st->regmap, ADEMA_REG_CH_WAV_HI(ch) + 1, &md);
	if (ret)
		return ret;
	ret = regmap_read(st->regmap, ADEMA_REG_CH_WAV_HI(ch) + 2, &lo);
	if (ret)
		return ret;

	v = ((hi & 0xFFu) << 16) | ((md & 0xFFu) << 8) | (lo & 0xFFu);
	*out = sign_extend32(v, 23);
	return 0;
}

/* Fill IIO_VAL_FRACTIONAL scale (mV per LSB) for the given channel. */
static void adema_scale(struct adema_state *st, unsigned int ch,
			int *val, int *val2)
{
	unsigned int gain = st->adc_input_gain[ch] ? st->adc_input_gain[ch] : 1;

	/* scale = 1000 mV / (codes_per_volt × input_gain) */
	*val = 1000;
	*val2 = ADEMA_CODES_PER_VOLT_1X * gain;
}

/* --- gain register (2.22 signed) helpers --- */

static int adema_get_gain(struct adema_state *st, unsigned int ch, s32 *val)
{
	u32 raw;
	int ret;

	ret = adema_dsp_ram_read24(st, ADEMA_DSP_CH_BASE(ch) + ADEMA_DSP_OFF_GAIN_LO,
				   &raw);
	if (ret)
		return ret;

	*val = sign_extend32(raw, 23);
	return 0;
}

static int adema_set_gain(struct adema_state *st, unsigned int ch, s32 val)
{
	if (val > (1 << 23) - 1 || val < -(1 << 23))
		return -ERANGE;

	return adema_dsp_ram_write24(st, ADEMA_DSP_CH_BASE(ch) + ADEMA_DSP_OFF_GAIN_LO,
				     val & 0xFFFFFF);
}

static int adema_get_offset(struct adema_state *st, unsigned int ch, s32 *val)
{
	u32 raw;
	int ret;

	ret = adema_dsp_ram_read24(st, ADEMA_DSP_CH_BASE(ch) + ADEMA_DSP_OFF_OFFSET_LO,
				   &raw);
	if (ret)
		return ret;

	*val = sign_extend32(raw, 23);
	return 0;
}

static int adema_set_offset(struct adema_state *st, unsigned int ch, s32 val)
{
	return adema_dsp_ram_write24(st, ADEMA_DSP_CH_BASE(ch) + ADEMA_DSP_OFF_OFFSET_LO,
				     val & 0xFFFFFF);
}

/*
 * Phase offset: unsigned fixed-point fraction of one sample period,
 * 13 bits, 0..0x1FFF (max one sample period of delay).
 */

static int adema_get_phase(struct adema_state *st, unsigned int ch, s32 *val)
{
	unsigned int hi, lo;
	int ret;

	ret = regmap_read(st->regmap, ADEMA_REG_PHASE_OFFSET_HI(ch), &hi);
	if (ret)
		return ret;
	ret = regmap_read(st->regmap, ADEMA_REG_PHASE_OFFSET_LO(ch), &lo);
	if (ret)
		return ret;

	*val = ((hi & 0x1F) << 8) | (lo & 0xFF);
	return 0;
}

static int adema_set_phase(struct adema_state *st, unsigned int ch, s32 val)
{
	u32 raw;
	int ret;

	if (val < 0 || val > 0x1FFF)
		return -ERANGE;

	/*
	 * PHASE_OFFSET is a datapath configuration register: writes are
	 * discarded while DATAPATH_CONFIG_LOCK is set, so bracket them
	 * with the unlock/lock sequence like the other datapath fields.
	 */
	ret = adema_config_unlock(st);
	if (ret)
		return ret;

	raw = val;
	ret = regmap_write(st->regmap, ADEMA_REG_PHASE_OFFSET_HI(ch),
			   (raw >> 8) & 0x1F);
	if (!ret)
		ret = regmap_write(st->regmap, ADEMA_REG_PHASE_OFFSET_LO(ch),
				   raw & 0xFF);
	if (ret)
		return ret;

	return adema_config_lock(st);
}

/* ------------------------------------------------------------------------ */
/*  IIO ext_info attributes                                                 */
/* ------------------------------------------------------------------------ */

static int adema_read_datapath_bit(struct adema_state *st, unsigned int ch, u8 mask)
{
	unsigned int val;
	int ret;

	ret = regmap_read(st->regmap, ADEMA_REG_DATAPATH_CFG(ch), &val);
	if (ret)
		return ret;
	return (val & mask) ? 1 : 0;
}

static int adema_write_datapath_bit(struct adema_state *st, unsigned int ch,
				    u8 mask, bool enable)
{
	int ret;

	ret = adema_config_unlock(st);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, ADEMA_REG_DATAPATH_CFG(ch),
				 mask, enable ? mask : 0);
	if (ret)
		return ret;

	return adema_config_lock(st);
}

static ssize_t __adema_ext_read(struct iio_dev *indio_dev, uintptr_t priv,
				const struct iio_chan_spec *chan, char *buf)
{
	struct adema_state *st = iio_priv(indio_dev);
	unsigned int ch = chan->channel;
	unsigned int val;
	s32 sval;
	u32 raw;
	int ret;

	switch (priv) {
	case ADEMA_EXT_INFO_XT_GAIN:
		ret = adema_dsp_ram_read24(st, ADEMA_DSP_CH_BASE(ch) +
					   ADEMA_DSP_OFF_XT_GAIN_LO, &raw);
		if (ret)
			return ret;
		sval = sign_extend32(raw, 23);
		return sysfs_emit(buf, "%d\n", sval);
	case ADEMA_EXT_INFO_XT_AGGRESSOR:
		ret = adema_dsp_ram_read8(st, ADEMA_DSP_CH_BASE(ch) +
					  ADEMA_DSP_OFF_XT_AGGRESSOR, &val);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", val & 0x7);
	case ADEMA_EXT_INFO_SHIFT:
		ret = adema_dsp_ram_read8(st, ADEMA_DSP_CH_BASE(ch) +
					  ADEMA_DSP_OFF_SHIFT, &val);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", val & 0x7);
	case ADEMA_EXT_INFO_DC_BLOCK_ALPHA:
		ret = regmap_read(st->regmap, ADEMA_REG_DATAPATH_ALPHA(ch), &val);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n",
				  (ch & 1) ? (val >> 4) & 0xF : val & 0xF);
	case ADEMA_EXT_INFO_HPF_EN:
		ret = adema_read_datapath_bit(st, ch, ADEMA_DATAPATH_CFG_HPF_EN);
		break;
	case ADEMA_EXT_INFO_LPF_EN:
		ret = adema_read_datapath_bit(st, ch, ADEMA_DATAPATH_CFG_LPF_EN);
		break;
	case ADEMA_EXT_INFO_SCF_EN:
		ret = adema_read_datapath_bit(st, ch, ADEMA_DATAPATH_CFG_SCF_EN);
		break;
	case ADEMA_EXT_INFO_COMP_FLT_EN:
		ret = adema_read_datapath_bit(st, ch, ADEMA_DATAPATH_CFG_COMP_FLT_EN);
		break;
	case ADEMA_EXT_INFO_COMP_FLT_CFG:
		ret = adema_read_datapath_bit(st, ch, ADEMA_DATAPATH_CFG_COMP_FLT_CFG);
		break;
	case ADEMA_EXT_INFO_ALLPASS_EN:
		ret = adema_read_datapath_bit(st, ch, ADEMA_DATAPATH_CFG_ALLPASS_EN);
		break;
	case ADEMA_EXT_INFO_GAIN_OFFSET_XT_EN:
		ret = adema_read_datapath_bit(st, ch,
					      ADEMA_DATAPATH_CFG_GAIN_OFFSET_XT_EN);
		break;
	case ADEMA_EXT_INFO_INPUT_GAIN:
		ret = regmap_read(st->regmap, ADEMA_REG_ADC_GAIN, &val);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", (val & BIT(ch)) ? 2 : 1);
	case ADEMA_EXT_INFO_INPUT_INVERT:
		ret = regmap_read(st->regmap, ADEMA_REG_ADC_INV, &val);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", (val & BIT(ch)) ? 1 : 0);
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	return sysfs_emit(buf, "%d\n", ret);
}

static ssize_t adema_ext_read(struct iio_dev *indio_dev, uintptr_t priv,
			      const struct iio_chan_spec *chan, char *buf)
{
	ssize_t ret;

	/*
	 * The ext_info attributes reach the chip over the regmap
	 * short-frame path; keep them off the bus while the buffer
	 * streams (the SPI offload engine owns the controller then).
	 */
	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;
	ret = __adema_ext_read(indio_dev, priv, chan, buf);
	iio_device_release_direct(indio_dev);
	return ret;
}

static ssize_t __adema_ext_write(struct iio_dev *indio_dev, uintptr_t priv,
				 const struct iio_chan_spec *chan,
				 const char *buf, size_t len)
{
	struct adema_state *st = iio_priv(indio_dev);
	unsigned int ch = chan->channel;
	unsigned int uval;
	bool bval;
	s32 sval;
	int ret;

	switch (priv) {
	case ADEMA_EXT_INFO_XT_GAIN:
		ret = kstrtos32(buf, 0, &sval);
		if (ret)
			return ret;
		if (sval > (1 << 23) - 1 || sval < -(1 << 23))
			return -ERANGE;
		ret = adema_dsp_ram_write24(st, ADEMA_DSP_CH_BASE(ch) +
					    ADEMA_DSP_OFF_XT_GAIN_LO,
					    sval & 0xFFFFFF);
		break;
	case ADEMA_EXT_INFO_XT_AGGRESSOR:
		ret = kstrtouint(buf, 0, &uval);
		if (ret)
			return ret;
		if (uval > 6)
			return -EINVAL;
		ret = adema_dsp_ram_write8(st, ADEMA_DSP_CH_BASE(ch) +
					   ADEMA_DSP_OFF_XT_AGGRESSOR, uval);
		break;
	case ADEMA_EXT_INFO_SHIFT:
		ret = kstrtouint(buf, 0, &uval);
		if (ret)
			return ret;
		if (uval > 7)
			return -EINVAL;
		ret = adema_dsp_ram_write8(st, ADEMA_DSP_CH_BASE(ch) +
					   ADEMA_DSP_OFF_SHIFT, uval);
		break;
	case ADEMA_EXT_INFO_DC_BLOCK_ALPHA:
		ret = kstrtouint(buf, 0, &uval);
		if (ret)
			return ret;
		if (uval > 0xF)
			return -EINVAL;
		ret = adema_config_unlock(st);
		if (ret)
			return ret;
		ret = regmap_update_bits(st->regmap,
					 ADEMA_REG_DATAPATH_ALPHA(ch),
					 ADEMA_ALPHA_MASK(ch),
					 (ch & 1) ? uval << 4 : uval);
		if (ret)
			return ret;
		ret = adema_config_lock(st);
		break;
	case ADEMA_EXT_INFO_HPF_EN:
	case ADEMA_EXT_INFO_LPF_EN:
	case ADEMA_EXT_INFO_SCF_EN:
	case ADEMA_EXT_INFO_COMP_FLT_EN:
	case ADEMA_EXT_INFO_COMP_FLT_CFG:
	case ADEMA_EXT_INFO_ALLPASS_EN:
	case ADEMA_EXT_INFO_GAIN_OFFSET_XT_EN: {
		static const u8 mask_map[] = {
			[ADEMA_EXT_INFO_HPF_EN]		 = ADEMA_DATAPATH_CFG_HPF_EN,
			[ADEMA_EXT_INFO_LPF_EN]		 = ADEMA_DATAPATH_CFG_LPF_EN,
			[ADEMA_EXT_INFO_SCF_EN]		 = ADEMA_DATAPATH_CFG_SCF_EN,
			[ADEMA_EXT_INFO_COMP_FLT_EN]	 = ADEMA_DATAPATH_CFG_COMP_FLT_EN,
			[ADEMA_EXT_INFO_COMP_FLT_CFG]	 = ADEMA_DATAPATH_CFG_COMP_FLT_CFG,
			[ADEMA_EXT_INFO_ALLPASS_EN]	 = ADEMA_DATAPATH_CFG_ALLPASS_EN,
			[ADEMA_EXT_INFO_GAIN_OFFSET_XT_EN] = ADEMA_DATAPATH_CFG_GAIN_OFFSET_XT_EN,
		};
		ret = kstrtobool(buf, &bval);
		if (ret)
			return ret;
		ret = adema_write_datapath_bit(st, ch, mask_map[priv], bval);
		break;
	}
	case ADEMA_EXT_INFO_INPUT_GAIN:
		ret = kstrtouint(buf, 0, &uval);
		if (ret)
			return ret;
		if (uval != 1 && uval != 2)
			return -EINVAL;
		ret = adema_config_unlock(st);
		if (ret)
			return ret;
		ret = regmap_update_bits(st->regmap, ADEMA_REG_ADC_GAIN,
					 BIT(ch), uval == 2 ? BIT(ch) : 0);
		if (ret)
			return ret;
		st->adc_input_gain[ch] = uval;
		ret = adema_config_lock(st);
		break;
	case ADEMA_EXT_INFO_INPUT_INVERT:
		ret = kstrtobool(buf, &bval);
		if (ret)
			return ret;
		ret = adema_config_unlock(st);
		if (ret)
			return ret;
		ret = regmap_update_bits(st->regmap, ADEMA_REG_ADC_INV,
					 BIT(ch), bval ? BIT(ch) : 0);
		if (ret)
			return ret;
		ret = adema_config_lock(st);
		break;
	default:
		return -EINVAL;
	}

	return ret ? ret : len;
}

#define ADEMA_EXT_INFO(_name, _priv) {			\
	.name = (_name),				\
	.shared = IIO_SEPARATE,				\
	.read = adema_ext_read,				\
	.write = adema_ext_write,			\
	.private = (_priv),				\
}

static ssize_t adema_ext_write(struct iio_dev *indio_dev, uintptr_t priv,
			       const struct iio_chan_spec *chan,
			       const char *buf, size_t len)
{
	ssize_t ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;
	ret = __adema_ext_write(indio_dev, priv, chan, buf, len);
	iio_device_release_direct(indio_dev);
	return ret;
}

static const struct iio_chan_spec_ext_info adema_ext_info[] = {
	ADEMA_EXT_INFO("xt_gain",		ADEMA_EXT_INFO_XT_GAIN),
	ADEMA_EXT_INFO("xt_aggressor",		ADEMA_EXT_INFO_XT_AGGRESSOR),
	ADEMA_EXT_INFO("shift",			ADEMA_EXT_INFO_SHIFT),
	ADEMA_EXT_INFO("dc_block_alpha",	ADEMA_EXT_INFO_DC_BLOCK_ALPHA),
	ADEMA_EXT_INFO("filter_hpf_en",		ADEMA_EXT_INFO_HPF_EN),
	ADEMA_EXT_INFO("filter_lpf_en",		ADEMA_EXT_INFO_LPF_EN),
	ADEMA_EXT_INFO("filter_scf_en",		ADEMA_EXT_INFO_SCF_EN),
	ADEMA_EXT_INFO("filter_comp_en",	ADEMA_EXT_INFO_COMP_FLT_EN),
	ADEMA_EXT_INFO("filter_comp_cfg",	ADEMA_EXT_INFO_COMP_FLT_CFG),
	ADEMA_EXT_INFO("filter_allpass_en",	ADEMA_EXT_INFO_ALLPASS_EN),
	ADEMA_EXT_INFO("gain_offset_xt_en",	ADEMA_EXT_INFO_GAIN_OFFSET_XT_EN),
	ADEMA_EXT_INFO("input_gain",		ADEMA_EXT_INFO_INPUT_GAIN),
	ADEMA_EXT_INFO("input_invert",		ADEMA_EXT_INFO_INPUT_INVERT),
	{ }
};

#define ADEMA_VOLTAGE_CHAN(_idx) {					\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.channel = (_idx),						\
	.scan_index = (_idx),						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_CALIBBIAS) |		\
			      BIT(IIO_CHAN_INFO_CALIBSCALE) |		\
			      BIT(IIO_CHAN_INFO_PHASE),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.info_mask_shared_by_all_available =				\
		BIT(IIO_CHAN_INFO_SAMP_FREQ),				\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = ADEMA_REALBITS,				\
		.storagebits = ADEMA_STORAGEBITS,			\
		.endianness = IIO_CPU,					\
	},								\
	.ext_info = adema_ext_info,					\
}

/* All channels are always sampled simultaneously — expose that as the only
 * legal scan mask so IIO enforces a fixed frame layout.
 */
static const unsigned long adema127_scan_masks[] = {
	GENMASK(6, 0),
	0,
};

static const unsigned long adema124_scan_masks[] = {
	GENMASK(3, 0),
	0,
};

static const struct iio_chan_spec adema127_channels[] = {
	ADEMA_VOLTAGE_CHAN(0),
	ADEMA_VOLTAGE_CHAN(1),
	ADEMA_VOLTAGE_CHAN(2),
	ADEMA_VOLTAGE_CHAN(3),
	ADEMA_VOLTAGE_CHAN(4),
	ADEMA_VOLTAGE_CHAN(5),
	ADEMA_VOLTAGE_CHAN(6),
	IIO_CHAN_SOFT_TIMESTAMP(7),
};

static const struct iio_chan_spec adema124_channels[] = {
	ADEMA_VOLTAGE_CHAN(0),
	ADEMA_VOLTAGE_CHAN(1),
	ADEMA_VOLTAGE_CHAN(2),
	ADEMA_VOLTAGE_CHAN(3),
	IIO_CHAN_SOFT_TIMESTAMP(4),
};

static const struct adema_chip_info adema127_chip_info = {
	.name			= "adema127",
	.product_id		= ADEMA_PRODUCT_ID_ADEMA127,
	.num_channels		= 7,
	.long_frame_len		= ADEMA127_LONG_FRAME_LEN,
	.channels		= adema127_channels,
	.num_iio_channels	= ARRAY_SIZE(adema127_channels),
	.available_scan_masks	= adema127_scan_masks,
};

static const struct adema_chip_info adema124_chip_info = {
	.name			= "adema124",
	.product_id		= ADEMA_PRODUCT_ID_ADEMA124,
	.num_channels		= 4,
	.long_frame_len		= ADEMA124_LONG_FRAME_LEN,
	.channels		= adema124_channels,
	.num_iio_channels	= ARRAY_SIZE(adema124_channels),
	.available_scan_masks	= adema124_scan_masks,
};

/* ------------------------------------------------------------------------ */
/*  IIO info ops                                                            */
/* ------------------------------------------------------------------------ */

static int adema_read_raw(struct iio_dev *indio_dev,
			  struct iio_chan_spec const *chan,
			  int *val, int *val2, long info)
{
	struct adema_state *st = iio_priv(indio_dev);
	s32 sval;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;
		ret = adema_read_channel_raw(st, chan->channel, &sval);
		iio_device_release_direct(indio_dev);
		if (ret)
			return ret;
		*val = sval;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		adema_scale(st, chan->channel, val, val2);
		return IIO_VAL_FRACTIONAL;
	case IIO_CHAN_INFO_CALIBBIAS:
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;
		ret = adema_get_offset(st, chan->channel, &sval);
		iio_device_release_direct(indio_dev);
		if (ret)
			return ret;
		*val = sval;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;
		ret = adema_get_gain(st, chan->channel, &sval);
		iio_device_release_direct(indio_dev);
		if (ret)
			return ret;
		/*
		 * Register is signed 2.22 fixed point (range ±2). Report as
		 * fractional integer: raw / 2^22.
		 */
		*val = sval;
		*val2 = 22;
		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_PHASE:
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;
		ret = adema_get_phase(st, chan->channel, &sval);
		iio_device_release_direct(indio_dev);
		if (ret)
			return ret;
		*val = sval;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->sampling_freq;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int adema_write_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int val, int val2, long info)
{
	struct adema_state *st = iio_priv(indio_dev);
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	switch (info) {
	case IIO_CHAN_INFO_CALIBBIAS:
		ret = adema_set_offset(st, chan->channel, val);
		break;
	case IIO_CHAN_INFO_CALIBSCALE:
		ret = adema_set_gain(st, chan->channel, val);
		break;
	case IIO_CHAN_INFO_PHASE:
		ret = adema_set_phase(st, chan->channel, val);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = adema_set_sampling_freq(st, val);
		break;
	default:
		ret = -EINVAL;
	}

	iio_device_release_direct(indio_dev);
	return ret;
}

static int adema_read_avail(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    const int **vals, int *type, int *length,
			    long info)
{
	struct adema_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = st->sampling_freqs;
		*length = ARRAY_SIZE(st->sampling_freqs);
		*type = IIO_VAL_INT;
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int adema_debugfs_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				    unsigned int writeval, unsigned int *readval)
{
	struct adema_state *st = iio_priv(indio_dev);
	bool needs_dsp = reg >= 0x400;
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	if (needs_dsp) {
		if (readval)
			ret = adema_dsp_ram_read8(st, reg, readval);
		else
			ret = adema_dsp_ram_write8(st, reg, writeval);
	} else if (readval) {
		ret = regmap_read(st->regmap, reg, readval);
	} else {
		ret = regmap_write(st->regmap, reg, writeval);
	}

	iio_device_release_direct(indio_dev);
	return ret;
}

/* --- device-level sync-align attribute --- */

static ssize_t adema_sync_align_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adema_state *st = iio_priv(indio_dev);
	bool trigger;
	int ret;

	ret = kstrtobool(buf, &trigger);
	if (ret)
		return ret;
	if (!trigger)
		return len;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	ret = regmap_write(st->regmap, ADEMA_REG_SYNC_SNAP,
			   ADEMA_SYNC_SNAP_ALIGN);

	iio_device_release_direct(indio_dev);
	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(sync_align, 0200, NULL, adema_sync_align_store, 0);

/* --- device-level STREAM_DBG test-pattern selector --- */
/*
 * CONFIG0[3:2] selects the source of the WAV shift-out registers:
 *   normal      – real ADC samples
 *   static      – a fixed constant pattern (for SPI-link bring-up)
 *   increment   – a counter that steps once per ADC conversion (for
 *                 detecting dropped or duplicated samples end-to-end)
 * See ADEMA124/127 datasheet §"Debug Streaming Modes".
 */
static const char * const adema_test_pattern_modes[] = {
	[0] = "normal",
	[1] = "static",
	[2] = "increment",
};

static ssize_t adema_test_pattern_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adema_state *st = iio_priv(indio_dev);
	unsigned int val, mode;
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;
	ret = regmap_read(st->regmap, ADEMA_REG_CONFIG0, &val);
	iio_device_release_direct(indio_dev);
	if (ret)
		return ret;

	mode = FIELD_GET(ADEMA_CONFIG0_STREAM_DBG, val);
	if (mode >= ARRAY_SIZE(adema_test_pattern_modes) ||
	    !adema_test_pattern_modes[mode])
		return sysfs_emit(buf, "reserved\n");

	return sysfs_emit(buf, "%s\n", adema_test_pattern_modes[mode]);
}

static ssize_t adema_test_pattern_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adema_state *st = iio_priv(indio_dev);
	int mode, ret;

	mode = sysfs_match_string(adema_test_pattern_modes, buf);
	if (mode < 0)
		return mode;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	ret = adema_config_unlock(st);
	if (ret)
		goto out;

	ret = regmap_update_bits(st->regmap, ADEMA_REG_CONFIG0,
				 ADEMA_CONFIG0_STREAM_DBG,
				 FIELD_PREP(ADEMA_CONFIG0_STREAM_DBG, mode));
	if (ret)
		goto out;

	ret = adema_config_lock(st);
out:
	iio_device_release_direct(indio_dev);
	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(test_pattern, 0644,
		       adema_test_pattern_show,
		       adema_test_pattern_store, 0);

static ssize_t adema_test_pattern_available_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	int len = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(adema_test_pattern_modes); i++)
		len += sysfs_emit_at(buf, len, "%s%s",
				     i ? " " : "",
				     adema_test_pattern_modes[i]);
	len += sysfs_emit_at(buf, len, "\n");
	return len;
}

static IIO_DEVICE_ATTR(test_pattern_available, 0444,
		       adema_test_pattern_available_show, NULL, 0);

static struct attribute *adema_attributes[] = {
	&iio_dev_attr_sync_align.dev_attr.attr,
	&iio_dev_attr_test_pattern.dev_attr.attr,
	&iio_dev_attr_test_pattern_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group adema_attr_group = {
	.attrs = adema_attributes,
};

static const struct iio_info adema_iio_info = {
	.read_raw		= adema_read_raw,
	.write_raw		= adema_write_raw,
	.read_avail		= adema_read_avail,
	.debugfs_reg_access	= adema_debugfs_reg_access,
	.attrs			= &adema_attr_group,
};

/* ------------------------------------------------------------------------ */
/*  Triggered buffer path                                                   */
/* ------------------------------------------------------------------------ */

static void adema_unpack_long_frame(struct adema_state *st)
{
	const u8 *off = st->chip_info->num_channels == 4 ?
			adema124_wav_off : adema127_wav_off;
	s32 *chan = (s32 *)st->scan;
	unsigned int i;

	for (i = 0; i < st->chip_info->num_channels; i++) {
		const u8 *p = &st->long_rx[off[i]];
		/*
		 * Waveform bytes are transmitted WAV_LO, WAV_MD, WAV_HI
		 * (little-endian) per the datasheet's Long Format Operation
		 * section -- p[0] is the LSB.
		 */
		u32 raw = ((u32)p[2] << 16) | ((u32)p[1] << 8) | p[0];

		chan[i] = sign_extend32(raw, 23);
	}
}

static irqreturn_t adema_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct adema_state *st = iio_priv(indio_dev);
	struct spi_transfer xfer = {
		.tx_buf = st->long_tx,
		.rx_buf = st->long_rx,
		.len = st->chip_info->long_frame_len,
	};
	int ret;

	scoped_guard(mutex, &st->lock) {
		ret = spi_sync_transfer(st->spi, &xfer, 1);
		if (ret)
			goto out;

		ret = adema_verify_long_crc(st);
		if (ret)
			goto out;

		adema_unpack_long_frame(st);
	}

	iio_push_to_buffers_with_timestamp(indio_dev, st->scan, pf->timestamp);
out:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int adema_buffer_preenable(struct iio_dev *indio_dev)
{
	struct adema_state *st = iio_priv(indio_dev);

	adema_build_long_stream_cmd(st);
	return 0;
}

static int adema_buffer_postenable(struct iio_dev *indio_dev)
{
	struct adema_state *st = iio_priv(indio_dev);

	if (st->dready_irq)
		enable_irq(st->dready_irq);
	return 0;
}

static int adema_buffer_predisable(struct iio_dev *indio_dev)
{
	struct adema_state *st = iio_priv(indio_dev);

	if (st->dready_irq)
		disable_irq(st->dready_irq);
	return 0;
}

static const struct iio_buffer_setup_ops adema_buffer_setup_ops = {
	.preenable  = adema_buffer_preenable,
	.postenable = adema_buffer_postenable,
	.predisable = adema_buffer_predisable,
};

/* ------------------------------------------------------------------------ */
/*  DREADY interrupt handler (triggered-buffer path)                        */
/* ------------------------------------------------------------------------ */

static irqreturn_t adema_dready_irq(int irq, void *ptr)
{
	struct iio_dev *indio_dev = ptr;

	iio_trigger_poll(indio_dev->trig);
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------------ */
/*  Reset / init                                                            */
/* ------------------------------------------------------------------------ */

static int adema_reset(struct adema_state *st)
{
	unsigned int status;
	int ret;

	if (st->reset_gpio) {
		/* Hardware reset: pulse RESET low (asserted), then release. */
		gpiod_set_value_cansleep(st->reset_gpio, 1);
		fsleep(10);
		gpiod_set_value_cansleep(st->reset_gpio, 0);
	} else {
		/* Software reset: unlock, write the SWRST key, wait. */
		ret = regmap_write(st->regmap, ADEMA_REG_WR_LOCK,
				   ADEMA_WR_LOCK_UNLOCK);
		if (ret)
			return ret;

		ret = regmap_write(st->regmap, ADEMA_REG_SWRST, ADEMA_SWRST_KEY);
		if (ret)
			return ret;
	}

	fsleep(ADEMA_STARTUP_US);

	/* Confirm RESET_DONE and clear it. */
	ret = regmap_read(st->regmap, ADEMA_REG_STATUS0, &status);
	if (ret)
		return ret;
	if (!(status & ADEMA_STATUS0_RESET_DONE))
		dev_warn(&st->spi->dev, "RESET_DONE not asserted after reset\n");

	return regmap_write(st->regmap, ADEMA_REG_STATUS0,
			    ADEMA_STATUS0_RESET_DONE);
}

static int adema_detect(struct adema_state *st)
{
	unsigned int id;
	int ret;

	ret = regmap_read(st->regmap, ADEMA_REG_PRODUCT_ID, &id);
	if (ret)
		return dev_err_probe(&st->spi->dev, ret,
				     "failed to read PRODUCT_ID\n");

	if (id != st->chip_info->product_id)
		return dev_err_probe(&st->spi->dev, -ENODEV,
				     "unexpected PRODUCT_ID 0x%02x (expected 0x%02x)\n",
				     id, st->chip_info->product_id);

	dev_info(&st->spi->dev, "detected %s (PRODUCT_ID 0x%02x)\n",
		 st->chip_info->name, id);
	return 0;
}

/*
 * DSP RAM (calibration gain/offset, crosstalk compensation, shift) powers
 * up with undefined contents: the chip only loads the documented defaults
 * "when a DSP filter is enabled and the DATAPATH_CONFIG_LOCK bit is set
 * to 1" (datasheet, Reset section). Pulse a filter enable once so the
 * DSP-RAM-backed attributes read their defaults instead of die-random
 * SRAM power-up state — which would otherwise enter the datapath the
 * moment a user enables gain/offset/crosstalk processing.
 */
static int adema_dsp_ram_load_defaults(struct adema_state *st)
{
	int ret;

	ret = adema_write_datapath_bit(st, 0, ADEMA_DATAPATH_CFG_HPF_EN, true);
	if (ret)
		return ret;

	/*
	 * The ROM-to-RAM load takes ~40 µs after the config lock is set and
	 * the datasheet forbids SPI transactions while it runs (and the DSP
	 * memory window won't open while a filter is enabled), so wait it
	 * out quietly before disabling the filter again.
	 */
	fsleep(1000);

	ret = adema_write_datapath_bit(st, 0, ADEMA_DATAPATH_CFG_HPF_EN, false);
	if (ret)
		return ret;

	fsleep(1000);

	/* Confirm the window opens again with the defaults in place. */
	scoped_guard(mutex, &st->dsp_lock) {
		ret = adema_dsp_ram_enter(st);
		if (ret == 0)
			ret = adema_dsp_ram_leave(st);
	}
	return ret;
}

static int adema_init(struct adema_state *st)
{
	int ret;

	/* Bring device to a known state. */
	ret = adema_reset(st);
	if (ret)
		return ret;

	ret = adema_detect(st);
	if (ret)
		return ret;

	/*
	 * Configure: mask everything except DREADY-relevant events, set
	 * default streaming (normal) mode, keep SPI-CRC on, use default
	 * 32 kSPS rate.
	 */
	ret = adema_config_unlock(st);
	if (ret)
		return ret;

	/*
	 * DREADY and CLKOUT share the same pin (per §"CLKOUT/DREADY").
	 * Leave CLKOUT_EN cleared so the pin carries the falling-edge DREADY
	 * pulse the driver's IRQ handler needs — setting CLKOUT_EN turns it
	 * into a ~16 MHz clock output which floods the host GPIO IRQ line.
	 */
	ret = regmap_write(st->regmap, ADEMA_REG_CONFIG0,
			   FIELD_PREP(ADEMA_CONFIG0_ADC_POWER_MODE, 3) |
			   ADEMA_CONFIG0_CRC_EN_SPI_WRITE);
	if (ret)
		return ret;

	/* Enable overrange bits in MASK1 for enabled channels. */
	ret = regmap_write(st->regmap, ADEMA_REG_MASK1,
			   GENMASK(st->chip_info->num_channels - 1, 0));
	if (ret)
		return ret;

	/*
	 * Stop here: programming DATARATE / SYNC_SNAP.ALIGN starts DREADY
	 * toggling, and PINT_LATCH accumulates edges regardless of MASK
	 * (HRM §16.6.6.1) — any backlog dumps as an IRQ storm on the first
	 * unmask. adema_start_sampling() finishes init from probe after
	 * adema_request_dready() has installed the DREADY IRQ.
	 */
	ret = adema_config_lock(st);
	if (ret)
		return ret;

	return adema_dsp_ram_load_defaults(st);
}

/*
 * Second-half init: program a low sampling rate and issue SYNC_SNAP.ALIGN
 * so channels start synchronously. Must run only after the DREADY IRQ has
 * been requested + defensively disabled. Userspace can raise the rate via
 * IIO_CHAN_INFO_SAMP_FREQ afterwards.
 */
static int adema_start_sampling(struct adema_state *st)
{
	int ret;

	/*
	 * Start at the lowest supported rate (250 Hz at nominal XTAL,
	 * scaled to the actual crystal). If a spurious unmask slips through
	 * despite IRQF_NO_AUTOEN and the defensive disable_irq(), the low
	 * rate keeps the storm survivable — the shell stays responsive and
	 * we can recover.
	 */
	ret = adema_set_sampling_freq(st, st->sampling_freqs[0]);
	if (ret)
		return ret;

	return regmap_write(st->regmap, ADEMA_REG_SYNC_SNAP,
			    ADEMA_SYNC_SNAP_ALIGN);
}

/* ------------------------------------------------------------------------ */
/*  Probe                                                                   */
/* ------------------------------------------------------------------------ */

static int adema_get_xtal(struct adema_state *st)
{
	struct device *dev = &st->spi->dev;
	struct clk *xtal;
	unsigned long rate;

	xtal = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(xtal))
		return dev_err_probe(dev, PTR_ERR(xtal),
				     "failed to get XTAL clock\n");

	rate = clk_get_rate(xtal);
	if (!rate)
		return dev_err_probe(dev, -EINVAL, "XTAL clock has 0 Hz rate\n");
	if (rate > UINT_MAX)
		return dev_err_probe(dev, -EINVAL,
				     "XTAL clock rate %lu Hz out of range\n", rate);

	st->xtal_hz = rate;
	adema_init_sampling_freqs(st);
	return 0;
}

static int adema_apply_channel_dt(struct adema_state *st)
{
	struct device *dev = &st->spi->dev;
	u8 gain_mask = 0, inv_mask = 0;
	int ret = 0;

	device_for_each_child_node_scoped(dev, node) {
		u32 idx, gain;

		ret = fwnode_property_read_u32(node, "reg", &idx);
		if (ret) {
			dev_err(dev, "channel node without reg property\n");
			return ret;
		}
		if (idx >= st->chip_info->num_channels) {
			dev_err(dev, "channel@%u out of range\n", idx);
			return -EINVAL;
		}

		if (!fwnode_property_read_u32(node, "adi,input-gain", &gain)) {
			if (gain != 1 && gain != 2)
				return -EINVAL;
			st->adc_input_gain[idx] = gain;
			if (gain == 2)
				gain_mask |= BIT(idx);
		}

		if (fwnode_property_read_bool(node, "adi,input-invert"))
			inv_mask |= BIT(idx);
	}

	if (!gain_mask && !inv_mask)
		return 0;

	ret = adema_config_unlock(st);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, ADEMA_REG_ADC_GAIN, gain_mask);
	if (ret)
		return ret;
	ret = regmap_write(st->regmap, ADEMA_REG_ADC_INV, inv_mask);
	if (ret)
		return ret;

	return adema_config_lock(st);
}

static int adema_request_dready(struct iio_dev *indio_dev, struct adema_state *st)
{
	struct device *dev = &st->spi->dev;
	struct gpio_desc *dready_gpio;
	struct iio_trigger *trig;
	int ret, irq;

	/*
	 * Prefer a dready-gpios binding when the board supplies one:
	 * gpiod_direction_input() enables the pad's input buffer
	 * (PORT_INEN, required for the pint to see edges) and
	 * gpiod_to_irq() maps the pin to the PINT-domain virq. Boards
	 * with a proper DT interrupt-controller parent use the plain
	 * interrupts fallback below instead.
	 */
	dready_gpio = devm_gpiod_get_optional(dev, "dready", GPIOD_IN);
	if (IS_ERR(dready_gpio))
		return dev_err_probe(dev, PTR_ERR(dready_gpio),
				     "failed to get dready gpio\n");

	if (dready_gpio) {
		irq = gpiod_to_irq(dready_gpio);
		if (irq < 0)
			return dev_err_probe(dev, irq,
					     "gpiod_to_irq(dready) failed\n");
	} else {
		irq = fwnode_irq_get_byname(dev_fwnode(dev), "dready");
		if (irq < 0)
			irq = st->spi->irq;
		if (irq <= 0)
			return dev_err_probe(dev, -EINVAL,
					     "missing DREADY interrupt\n");
	}

	trig = devm_iio_trigger_alloc(dev, "%s-dready-%d",
				      st->chip_info->name, iio_device_id(indio_dev));
	if (!trig)
		return -ENOMEM;

	iio_trigger_set_drvdata(trig, indio_dev);

	ret = devm_iio_trigger_register(dev, trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(trig);

	/*
	 * Request the IRQ enabled, then disable_irq() — deliberately not
	 * IRQF_NO_AUTOEN: that skips irq_startup(), but applying
	 * IRQF_TRIGGER_FALLING transiently unmasks the pin on the
	 * adsp-pint irqchip, and PINT_LATCH latches edges regardless of
	 * MASK (HRM §16.6.6.1), so a pre-accumulated backlog would dump
	 * as an IRQ storm. Requesting enabled forces one clean
	 * mask/unmask/mask cycle and still ends at depth=1, keeping
	 * postenable/predisable enable_irq()/disable_irq() balanced.
	 */
	ret = devm_request_irq(dev, irq, adema_dready_irq,
			       IRQF_TRIGGER_FALLING,
			       dev_name(dev), indio_dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request DREADY IRQ\n");
	disable_irq(irq);

	st->dready_irq = irq;
	return 0;
}

static int adema_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct adema_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	st->chip_info = spi_get_device_match_data(spi);
	if (!st->chip_info)
		return dev_err_probe(dev, -ENODEV, "missing chip match data\n");

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	ret = devm_mutex_init(dev, &st->dsp_lock);
	if (ret)
		return ret;

	ret = adema_get_xtal(st);
	if (ret)
		return ret;

	/*
	 * Power rails per the binding; boards with always-on rails simply
	 * omit the properties and get the regulator core's dummy supply.
	 */
	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable vdd supply\n");

	ret = devm_regulator_get_enable(dev, "refio");
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable refio supply\n");

	st->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(st->reset_gpio),
				     "failed to get reset gpio\n");

	/* Chip-sized scan layout: N × s32 payload, ALIGN(8) padding, s64 ts. */
	st->scan_push_bytes =
		ALIGN(st->chip_info->num_channels * sizeof(s32), sizeof(s64)) +
		sizeof(s64);

	for (unsigned int i = 0; i < ADEMA_MAX_CHANNELS; i++)
		st->adc_input_gain[i] = 1;

	st->regmap = devm_regmap_init(dev, NULL, st, &adema_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "failed to allocate regmap\n");

	indio_dev->name = st->chip_info->name;
	indio_dev->info = &adema_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_iio_channels;
	indio_dev->available_scan_masks = st->chip_info->available_scan_masks;

	ret = adema_init(st);
	if (ret)
		return ret;

	ret = adema_apply_channel_dt(st);
	if (ret)
		return ret;

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      &iio_pollfunc_store_time,
					      adema_trigger_handler,
					      &adema_buffer_setup_ops);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to set up triggered buffer\n");

	/*
	 * Install (and immediately disable) the DREADY IRQ BEFORE the
	 * chip starts pulsing DREADY. adema_init() only did reset +
	 * detect + CONFIG0 + MASK1 -- it deliberately deferred the
	 * DATARATE + SYNC_SNAP.ALIGN writes to adema_start_sampling()
	 * so PINT_LATCH stays empty while we install the handler.
	 */
	ret = adema_request_dready(indio_dev, st);
	if (ret)
		return ret;

	/*
	 * Start the sample pipeline now that a masked IRQ handler is in
	 * place.
	 */
	ret = adema_start_sampling(st);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

/* ------------------------------------------------------------------------ */
/*  Driver boilerplate                                                      */
/* ------------------------------------------------------------------------ */

static const struct spi_device_id adema_spi_ids[] = {
	{ "adema124", (kernel_ulong_t)&adema124_chip_info },
	{ "adema127", (kernel_ulong_t)&adema127_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(spi, adema_spi_ids);

static const struct of_device_id adema_of_match[] = {
	{ .compatible = "adi,adema124", .data = &adema124_chip_info },
	{ .compatible = "adi,adema127", .data = &adema127_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(of, adema_of_match);

static struct spi_driver adema_driver = {
	.driver = {
		.name = "adema127",
		.of_match_table = adema_of_match,
	},
	.probe = adema_probe,
	.id_table = adema_spi_ids,
};
module_spi_driver(adema_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADEMA124/ADEMA127 IIO driver");
MODULE_LICENSE("GPL");
