/*
 * ADAS1000 ECG driver
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/module.h>

#include <linux/interrupt.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include <linux/platform_data/adas1000.h>

#define ADAS1000_NOP		0x00	/* NOP (No operation) */
#define ADAS1000_ECGCTL		0x01	/* ECG Setting Register */
#define ADAS1000_LOFFCTL	0x02	/* Leads off Control Register */
#define ADAS1000_RESPCTL	0x03	/* Respiration Control Register */
#define ADAS1000_PACECTL	0x04	/* Pace Detection Control Register */
#define ADAS1000_CMREFCTL	0x05	/* Common Mode Reference and Shield Drive Control Register */
#define ADAS1000_GPIOCTL	0x06	/* GPIO Control Register */
#define ADAS1000_PACEAMPTH	0x07	/* Pace Amplitude Threshold2 */
#define ADAS1000_TESTTONE	0x08	/* Test Tone */
#define ADAS1000_CALDAC		0x09	/* Calibration DAC */
#define ADAS1000_FRMCTL		0x0A	/* Frame Control Register */
#define ADAS1000_FILTCTL	0x0B	/* Filter Control Register */
#define ADAS1000_LOFFUTH	0x0C	/* Leads off Upper Threshold */
#define ADAS1000_LOFFLTH	0x0D	/* Leads off Lower Threshold */
#define ADAS1000_PACEEDGETH	0x0E	/* Pace Edge Threshold */
#define ADAS1000_PACELVLTH	0x0F	/* Pace Level Threshold */
#define ADAS1000_REVID		0x10	/* Silicon revision ID */
#define ADAS1000_LADATA		0x11	/* LA or LEAD I Data */
#define ADAS1000_LLDATA		0x12	/* LL or LEAD II Data */
#define ADAS1000_RADATA		0x13	/* RA or LEAD III Data */
#define ADAS1000_V1DATA		0x14	/* V1 or V1' Data */
#define ADAS1000_V2DATA		0x15	/* V2 or V2' Data */
#define ADAS1000_PACEDATA	0x1A	/* Read Pace Detection Data */
#define ADAS1000_RESPMAG	0x1B	/* Read Respiration Data Magnitude */
#define ADAS1000_RESPPH		0x1C	/* Read Respiration Data Phase */
#define ADAS1000_LOFF		0x1D	/* Leads Off Status */
#define ADAS1000_DCLEADSOFF	0x1E	/* DC Leads off Register */
#define ADAS1000_OPSTAT		0x1F	/* Operating state */
#define ADAS1000_EXTENDSW	0x20	/* Extended Switch for respiration inputs */
#define ADAS1000_CALLA		0x21	/* User gain calibration LA */
#define ADAS1000_CALLL		0x22	/* User gain calibration LL */
#define ADAS1000_CALRA		0x23	/* User gain calibration RA */
#define ADAS1000_CALV1		0x24	/* User gain calibration V1 */
#define ADAS1000_CALV2		0x25	/* User gain calibration V2 */
#define ADAS1000_CAL(chan)	(ADAS1000_CALLA + (chan))
#define ADAS1000_LOAMLA		0x31	/* Leads off Amplitude for LA */
#define ADAS1000_LOAMLL		0x32	/* Leads off Amplitude for LL */
#define ADAS1000_LOAMRA		0x33	/* Leads off Amplitude for RA */
#define ADAS1000_LOAMV1		0x34	/* Leads off Amplitude for V1 */
#define ADAS1000_LOAMV2		0x35	/* Leads off Amplitude for V2 */
#define ADAS1000_PACE1_DATA	0x3A	/* Pace1 Width & Amplitude2 */
#define ADAS1000_PACE2_DATA	0x3B	/* Pace2 Width & Amplitude2 */
#define ADAS1000_PACE3_DATA	0x3C	/* Pace3 Width & Amplitude2 */
#define ADAS1000_FRAMES		0x40	/* Frame Header - Read Data Frames */
#define ADAS1000_CRC		0x41	/* Frame CRC */


#define ADAS1000_FRMCTL_CHANNEL_MASK	(0x1f << 19)
#define ADAS1000_FRMCTL_CHANNEL_DIS(ch)	BIT(23 - (ch))
#define ADAS1000_FRMCTL_DATAFMT		BIT(4)
#define ADAS1000_FRMCTL_DECIMATION_1	(0x0 << 2)
#define ADAS1000_FRMCTL_DECIMATION_2	(0x1 << 2)
#define ADAS1000_FRMCTL_DECIMATION_4	(0x2 << 2)
#define ADAS1000_FRMCTL_DECIMATION_MASK	(0x3 << 2)
#define ADAS1000_FRMCTL_FRMRATE_2KHZ	0x0
#define ADAS1000_FRMCTL_FRMRATE_16KHZ	0x1
#define ADAS1000_FRMCTL_FRMRATE_128KHZ	0x2
#define ADAS1000_FRMCTL_FRMRATE_MASK	0x3

#define ADAS1000_TESTTONE_TYPE_1HZ	(0 << 3)
#define ADAS1000_TESTTONE_TYPE_10HZ	(1 << 3)
#define ADAS1000_TESTTONE_TYPE_150HZ	(2 << 3)
#define ADAS1000_TESTTONE_TYPE_MASK	(3 << 3)

#define ADAS1000_TESTTONE_TONV2		BIT(23)
#define ADAS1000_TESTTONE_TONV1		BIT(22)
#define ADAS1000_TESTTONE_TONLL		BIT(21)
#define ADAS1000_TESTTONE_TONRA		BIT(20)
#define ADAS1000_TESTTONE_TONLA		BIT(19)
#define ADAS1000_TESTTONE_TONINT	BIT(2)
#define ADAS1000_TESTTONE_TONOUT	BIT(1)
#define ADAS1000_TESTTONE_TONEN		BIT(0)

#define ADAS1000_ECGCTL_CHANNEL_MASK	(0x1f << 19)
#define ADAS1000_ECGCTL_CHANNEL_EN(ch)	BIT(23 - (ch))
#define ADAS1000_ECGCTL_CHCONFIG	BIT(10)
#define ADAS1000_ECGCTL_GAIN_1_4	(0 << 8)
#define ADAS1000_ECGCTL_GAIN_2_1	(1 << 8)
#define ADAS1000_ECGCTL_GAIN_2_8	(2 << 8)
#define ADAS1000_ECGCTL_GAIN_4_2	(3 << 8)
#define ADAS1000_ECGCTL_GAIN_MASK	(3 << 8)
#define ADAS1000_ECGCTL_VREFBUF		BIT(7)
#define ADAS1000_ECGCTL_CLKEXT		BIT(6)
#define ADAS1000_ECGCTL_GANG		BIT(5)
#define ADAS1000_ECGCTL_MASTER		BIT(4)
#define ADAS1000_ECGCTL_HP		BIT(3)
#define ADAS1000_ECGCTL_CNVEN		BIT(2)
#define ADAS1000_ECGCTL_PWREN		BIT(1)
#define ADAS1000_ECGCTL_RESET		BIT(0)

#define ADAS1000_HEADER_MARKER		BIT(15)
#define ADAS1000_HEADER_READY		BIT(14)

#define ADAS1000_FILTCTL_LPF_40		(0x0 << 2)
#define ADAS1000_FILTCTL_LPF_150	(0x1 << 2)
#define ADAS1000_FILTCTL_LPF_250	(0x2 << 2)
#define ADAS1000_FILTCTL_LPF_450	(0x3 << 2)
#define ADAS1000_FILTCTL_LPF_MASK	(0x3 << 2)

#define ADAS1000_OPSTAT_FUSE_CRC	BIT(3)
#define ADAS1000_OPSTAT_FUSE_STATUS	BIT(2)
#define ADAS1000_OPSTAT_PLL_LOCK	BIT(1)
#define ADAS1000_OPSTAT_PLL_LOCKED	BIT(0)

#define ADAS1000_CAL_USRCAL		BIT(23)

#define ADAS1000_CALDAC_CALDACEN	BIT(10)
#define ADAS1000_CALDAC_CALDATA_MASK	0x3ff

#define ADAS1000_CMREFCTL_LACM		BIT(23)
#define ADAS1000_CMREFCTL_LLCM		BIT(22)
#define ADAS1000_CMREFCTL_RACM		BIT(21)
#define ADAS1000_CMREFCTL_V1CM		BIT(20)
#define ADAS1000_CMREFCTL_V2CM		BIT(19)
#define ADAS1000_CMREFCTL_LARLD		BIT(14)
#define ADAS1000_CMREFCTL_LLRLD		BIT(13)
#define ADAS1000_CMREFCTL_RARLD		BIT(12)
#define ADAS1000_CMREFCTL_V1RLD		BIT(11)
#define ADAS1000_CMREFCTL_V2RLD		BIT(10)
#define ADAS1000_CMREFCTL_CERLD		BIT(9)
#define ADAS1000_CMREFCTL_CEREFEN	BIT(8)
#define ADAS1000_CMREFCTL_RLDSEL_OFFSET	4
#define ADAS1000_CMREFCTL_RLDSEL_MASK	(0xf << 4)
#define ADAS1000_CMREFCTL_DRVCM		BIT(3)
#define ADAS1000_CMREFCTL_EXTCM		BIT(2)
#define ADAS1000_CMREFCTL_RLDEN		BIT(1)
#define ADAS1000_CMREFCTL_SHLDEN	BIT(0)

enum adas1000_channel {
	ADAS1000_LA,
	ADAS1000_LL,
	ADAS1000_RA,
	ADAS1000_V1,
	ADAS1000_V2,
	ADAS1000_VCM,
	ADAS1000_CALIB_DAC,
};

struct adas1000_state {
	struct spi_device	*spi;

	bool sampling;
	bool irq_enabled;

	uint8_t			*frame_data;
	unsigned int		frame_size;
	unsigned int		num_samples;

	unsigned int samplerate;

	struct iio_trigger *trig;

	struct mutex lock;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	__be32 data ____cacheline_aligned;
};

static int adas1000_write_reg(struct iio_dev *indio_dev, unsigned int reg,
	unsigned int val)
{
	struct adas1000_state *st = iio_priv(indio_dev);
	int ret;

	if (iio_buffer_enabled(indio_dev))
		return -EBUSY;

	mutex_lock(&st->lock);

	st->data = cpu_to_be32(BIT(31) | (reg << 24) | val);
	ret = spi_write(st->spi, &st->data, sizeof(st->data));

	mutex_unlock(&st->lock);

	return ret;
}

static int adas1000_read_reg(struct iio_dev *indio_dev, unsigned int reg,
	unsigned int *val)
{
	struct adas1000_state *st = iio_priv(indio_dev);
	struct spi_transfer xfer[2];
	struct spi_message msg;
	int ret;

	if (iio_buffer_enabled(indio_dev))
		return -EBUSY;

	memset(xfer, 0, sizeof(xfer));
	xfer[0].tx_buf = &st->data;
	xfer[0].len = sizeof(st->data);
	xfer[0].bits_per_word = 8;
	xfer[1].rx_buf = &st->data;
	xfer[1].len = sizeof(st->data);
	xfer[1].bits_per_word = 8;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer[0], &msg);
	spi_message_add_tail(&xfer[1], &msg);

	mutex_lock(&st->lock);

	st->data = cpu_to_be32(reg << 24);

	ret = spi_sync(st->spi, &msg);
	if (ret)
		goto out;

	*val = be32_to_cpu(st->data) & 0xffffff;

out:
	mutex_unlock(&st->lock);

	return ret;
}

static int adas1000_update_reg(struct iio_dev *indio_dev, unsigned int reg,
	unsigned int mask, unsigned int val)
{
	unsigned int reg_val;
	int ret;

	ret = adas1000_read_reg(indio_dev, reg, &reg_val);
	if (ret)
		return ret;
	reg_val &= ~mask;
	reg_val |= val;
	return adas1000_write_reg(indio_dev, reg, reg_val);
}

#ifdef CONFIG_DEBUG_FS

struct adas1000_debugfs_reg_data {
	struct iio_dev *indio_dev;
	unsigned int reg;
	unsigned int mask;
};

static int adas1000_debugfs_reg_set(void *data, u64 enable)
{
	struct adas1000_debugfs_reg_data *t = data;
	unsigned int val;

	if (enable)
		val = t->mask;
	else
		val = 0;

	return adas1000_update_reg(t->indio_dev, t->reg, t->mask, val);
}

static int adas1000_debugfs_reg_get(void *data, u64 *enable)
{
	struct adas1000_debugfs_reg_data *t = data;
	unsigned int val;
	int ret;

	ret = adas1000_read_reg(t->indio_dev, t->reg, &val);
	if (ret)
		return ret;

	*enable = (val & t->mask) ? 1 : 0;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(adas1000_testtone_fops,
	adas1000_debugfs_reg_get, adas1000_debugfs_reg_set,
	"%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(adas1000_status_fops,
	adas1000_debugfs_reg_get, NULL,
	"%llu\n");

static int adas1000_testtone_set_frequency(void *data, u64 val)
{
	struct iio_dev *indio_dev = data;
	unsigned int type;

	if (val == 1)
		type = ADAS1000_TESTTONE_TYPE_1HZ;
	else if (val == 10)
		type = ADAS1000_TESTTONE_TYPE_10HZ;
	else if (val == 150)
		type = ADAS1000_TESTTONE_TYPE_150HZ;
	else
		return -EINVAL;

	return adas1000_update_reg(indio_dev, ADAS1000_TESTTONE,
		ADAS1000_TESTTONE_TYPE_MASK, type);
}

static int adas1000_testtone_get_frequency(void *data, u64 *val)
{
	struct iio_dev *indio_dev = data;
	unsigned int type;
	int ret;

	ret = adas1000_read_reg(indio_dev, ADAS1000_TESTTONE, &type);
	if (ret)
		return ret;

	type &= ADAS1000_TESTTONE_TYPE_MASK;

	switch (type) {
	case ADAS1000_TESTTONE_TYPE_1HZ:
		*val = 1;
		break;
	case ADAS1000_TESTTONE_TYPE_10HZ:
		*val = 10;
		break;
	default:
		*val = 150;
		break;
	}

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(adas1000_testtone_frequency_fops,
	adas1000_testtone_get_frequency, adas1000_testtone_set_frequency,
	"%llu\n");

static const struct {
	const char * const name;
	unsigned int reg;
	unsigned int mask;
	const struct file_operations *fops;
} adas1000_debugfs_reg_info[] = {
	{ "testtone_en", ADAS1000_TESTTONE, ADAS1000_TESTTONE_TONEN,
		&adas1000_testtone_fops },
	{ "testtone_out_en", ADAS1000_TESTTONE, ADAS1000_TESTTONE_TONOUT,
		&adas1000_testtone_fops },
	{ "testtone_internal", ADAS1000_TESTTONE, ADAS1000_TESTTONE_TONINT,
		&adas1000_testtone_fops },
	{ "testtone_la_en", ADAS1000_TESTTONE, ADAS1000_TESTTONE_TONLA,
		&adas1000_testtone_fops },
	{ "testtone_ra_en", ADAS1000_TESTTONE, ADAS1000_TESTTONE_TONRA,
		&adas1000_testtone_fops },
	{ "testtone_ll_en", ADAS1000_TESTTONE, ADAS1000_TESTTONE_TONLL,
		&adas1000_testtone_fops },
	{ "testtone_v1_en", ADAS1000_TESTTONE, ADAS1000_TESTTONE_TONV1,
		&adas1000_testtone_fops },
	{ "testtone_v2_en", ADAS1000_TESTTONE, ADAS1000_TESTTONE_TONV2,
		&adas1000_testtone_fops },
	{ "fuse_crc_failure", ADAS1000_OPSTAT, ADAS1000_OPSTAT_FUSE_CRC,
		&adas1000_status_fops },
	{ "fuse_status", ADAS1000_OPSTAT, ADAS1000_OPSTAT_FUSE_STATUS,
		&adas1000_status_fops },
	{ "pll_lock_lost", ADAS1000_OPSTAT, ADAS1000_OPSTAT_PLL_LOCK,
		&adas1000_status_fops },
	{ "pll_locked", ADAS1000_OPSTAT, ADAS1000_OPSTAT_PLL_LOCKED,
		&adas1000_status_fops },
};

static void adaus1000_debugfs_init(struct iio_dev *indio_dev)
{
	struct adas1000_debugfs_reg_data *t;
	unsigned int i;

	t = devm_kzalloc(&indio_dev->dev,
		sizeof(*t) * ARRAY_SIZE(adas1000_debugfs_reg_info),
		GFP_KERNEL);
	if (!t)
		return;

	for (i = 0; i < ARRAY_SIZE(adas1000_debugfs_reg_info); i++) {
		t[i].indio_dev = indio_dev;
		t[i].reg = adas1000_debugfs_reg_info[i].reg;
		t[i].mask = adas1000_debugfs_reg_info[i].mask;
		debugfs_create_file(adas1000_debugfs_reg_info[i].name, 0644,
			indio_dev->debugfs_dentry, &t[i],
			adas1000_debugfs_reg_info[i].fops);
	}

	debugfs_create_file("testtone_frequency", 0644,
		indio_dev->debugfs_dentry, indio_dev,
		&adas1000_testtone_frequency_fops);
}

static int adas1000_debugfs_reg_access(struct iio_dev *indio_dev,
				  unsigned reg, unsigned writeval,
				  unsigned *readval)
{
	if (readval)
		return adas1000_read_reg(indio_dev, reg, readval);
	else
		return adas1000_write_reg(indio_dev, reg, writeval);
}

#else

static inline void adas1000_debugfs_init(struct indio_dev *indio_dev) {};
#define adau1000_debugfs_reg_access NULL

#endif

static int adas1000_startup(struct iio_dev *indio_dev, unsigned long mask)
{
	unsigned int ctrl_mask;
	unsigned int ctrl_value;
	unsigned int i;

	ctrl_mask = ADAS1000_ECGCTL_CNVEN | ADAS1000_ECGCTL_PWREN |
			ADAS1000_ECGCTL_CHCONFIG | ADAS1000_ECGCTL_CHANNEL_MASK;
	ctrl_value = ADAS1000_ECGCTL_CNVEN | ADAS1000_ECGCTL_PWREN;

	/* Channel 1, 3 and 5 */
	if (mask & 0x15)
		ctrl_value |= ADAS1000_ECGCTL_CHCONFIG;

	for_each_set_bit(i, &mask, indio_dev->masklength)
		ctrl_value |= ADAS1000_ECGCTL_CHANNEL_EN(i / 2);

	return adas1000_update_reg(indio_dev, ADAS1000_ECGCTL,
		ctrl_mask, ctrl_value);
}

static int adas1000_shutdown(struct iio_dev *indio_dev)
{
	unsigned int ctrl_mask;
	ctrl_mask = ADAS1000_ECGCTL_CNVEN | ADAS1000_ECGCTL_PWREN;
	ctrl_mask |= ADAS1000_ECGCTL_CHANNEL_MASK;
	return adas1000_update_reg(indio_dev, ADAS1000_ECGCTL, ctrl_mask, 0x00);
}

static int adas1000_preenable(struct iio_dev *indio_dev)
{
	struct adas1000_state *st = iio_priv(indio_dev);
	struct spi_transfer xfer;
	struct spi_message msg;
	unsigned int dummy;
	int ret;

	ret = iio_sw_buffer_preenable(indio_dev);
	if (ret)
			return ret;

	ret = adas1000_startup(indio_dev, *indio_dev->active_scan_mask);
	if (ret)
		return ret;

	/* Start sampling */
	adas1000_read_reg(indio_dev, ADAS1000_FRAMES, &dummy);

	/* We have to make sure to output all zeros on the SDO line */
	memset(st->frame_data, 0x00, st->frame_size);
	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = st->frame_data;
	xfer.rx_buf = st->frame_data;
	xfer.len = st->frame_size - 4;
	xfer.bits_per_word = 8;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(st->spi, &msg);

	st->sampling = true;

	return ret;
}

static int adas1000_postenable(struct iio_dev *indio_dev)
{
	struct adas1000_state *st = iio_priv(indio_dev);

	iio_triggered_buffer_postenable(indio_dev);

	if (st->spi->irq) {
		st->irq_enabled = true;
		enable_irq(st->spi->irq);
	}

	return 0;
}

static int adas1000_postdisable(struct iio_dev *indio_dev)
{
	struct adas1000_state *st = iio_priv(indio_dev);

	st->sampling = false;

	return adas1000_shutdown(indio_dev);
}

static bool adas1000_validate_scan_mask(struct iio_dev *indio_dev,
	const unsigned long *mask)
{
	return !((*mask & 0x15) && (*mask & 0x2a));
}

static const struct iio_buffer_setup_ops iio_triggered_buffer_setup_ops = {
	.preenable = &adas1000_preenable,
	.postenable = &adas1000_postenable,
	.predisable = &iio_triggered_buffer_predisable,
	.postdisable = &adas1000_postdisable,
	.validate_scan_mask = &adas1000_validate_scan_mask,
};

static irqreturn_t adas1000_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct iio_buffer *buffer = indio_dev->buffer;
	struct adas1000_state *st = iio_priv(indio_dev);
	struct spi_transfer xfer;
	struct spi_message msg;
	unsigned int frame_size;
	uint16_t header;
	uint32_t *d;
	int ret;
	int i;

	/* in 128kHz mode each sample is only 16 bit */
	if (st->samplerate == 128000)
		frame_size = st->frame_size / 2;
	else
		frame_size = st->frame_size;

	/* We have to make sure to output all zeros on the SDO line */
	memset(st->frame_data, 0x00, frame_size);
	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = st->frame_data;
	xfer.rx_buf = st->frame_data;
	xfer.len = frame_size;
	xfer.bits_per_word = 8;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(st->spi, &msg);
	if (ret)
		goto err;

	if (st->samplerate == 128000)
		header = be16_to_cpup((__be16 *)st->frame_data);
	else
		header = be32_to_cpup((__be32 *)st->frame_data) >> 16;

	/* Something is not right, discard the frame */
	if (!(header & ADAS1000_HEADER_MARKER) ||
		(header & ADAS1000_HEADER_READY))
		goto err;

	if (st->samplerate == 128000) {
		/* Unpack 16bit samples into 32bit samples */
		d = (uint32_t *)st->frame_data;
		for (i = round_down(st->num_samples, 2); i >= 0 ; i -= 2) {
			uint32_t l;
			l = be32_to_cpu(d[i / 2]);
			d[i] = (l >> 8) & 0x00ffffff;
			d[i+1] = (l << 8) & 0x00ffffff;
			}
	} else {
		/* The upper 8 bits contain the address, mask them out */
		d = (uint32_t *)st->frame_data;
		for (i = 1; i <= st->num_samples; i++)
			d[i] = be32_to_cpu(d[i]) & 0x00ffffff;
	}

	if (indio_dev->scan_timestamp)
		*((s64 *)(st->frame_data + 4 + indio_dev->scan_bytes - sizeof(u64))) = pf->timestamp;
	iio_push_to_buffer(buffer, st->frame_data + 4);

err:
	iio_trigger_notify_done(indio_dev->trig);
	enable_irq(st->spi->irq);

	return IRQ_HANDLED;
}

static int adas1000_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *scan_mask)
{
	struct adas1000_state *st = iio_priv(indio_dev);
	unsigned int ctrl;
	unsigned int i;

	st->frame_size = indio_dev->scan_bytes;
	st->frame_size += 8; /* Four bytes for the header and CRC each */

	st->frame_data = krealloc(st->frame_data, st->frame_size, GFP_KERNEL);
	if (!st->frame_data)
		return -ENOMEM;

	ctrl = ADAS1000_FRMCTL_CHANNEL_MASK;
	st->num_samples = 0;
	for_each_set_bit(i, scan_mask, indio_dev->masklength) {
		ctrl &= ~ADAS1000_FRMCTL_CHANNEL_DIS(i / 2);
		st->num_samples++;
	}

	ctrl |= (0x1f << 10);

	return adas1000_update_reg(indio_dev, ADAS1000_FRMCTL,
		ADAS1000_FRMCTL_CHANNEL_MASK | (0x1f << 10), ctrl);
}

static int adas1000_get_sample_rate(struct iio_dev *indio_dev,
	int *sample_rate)
{
	unsigned int ctrl;
	int ret;

	ret = adas1000_read_reg(indio_dev, ADAS1000_FRMCTL, &ctrl);
	if (ret)
		return ret;

	switch (ctrl & ADAS1000_FRMCTL_FRMRATE_MASK) {
	case ADAS1000_FRMCTL_FRMRATE_128KHZ:
		*sample_rate = 128000;
		break;
	case ADAS1000_FRMCTL_FRMRATE_16KHZ:
		*sample_rate = 16000;
		break;
	case ADAS1000_FRMCTL_FRMRATE_2KHZ:
		*sample_rate = 2000;
		break;
	default:
		return -EIO;
	}

	switch (ctrl & ADAS1000_FRMCTL_DECIMATION_MASK) {
	case ADAS1000_FRMCTL_DECIMATION_1:
		break;
	case ADAS1000_FRMCTL_DECIMATION_2:
		*sample_rate /= 2;
		break;
	default:
		*sample_rate /= 4;
		break;
	}

	return 0;
}

static int adas1000_set_sample_rate(struct iio_dev *indio_dev,
	int sample_rate)
{
	struct adas1000_state *st = iio_priv(indio_dev);
	unsigned int real_sample_rate;
	unsigned int ctrl;

	switch (sample_rate) {
	case 128000:
	case 64000:
	case 32000:
		real_sample_rate = 128000;
		ctrl = ADAS1000_FRMCTL_FRMRATE_128KHZ;
		break;
	case 16000:
	case 8000:
	case 4000:
		real_sample_rate = 16000;
		ctrl = ADAS1000_FRMCTL_FRMRATE_16KHZ;
		break;
	case 2000:
	case 1000:
	case 500:
		real_sample_rate = 2000;
		ctrl = ADAS1000_FRMCTL_FRMRATE_2KHZ;
		break;
	default:
		return -EINVAL;
	}

	switch (real_sample_rate / sample_rate) {
	case 1:
		ctrl |= ADAS1000_FRMCTL_DECIMATION_1;
		break;
	case 2:
		ctrl |= ADAS1000_FRMCTL_DECIMATION_2;
		break;
	case 4:
		ctrl |= ADAS1000_FRMCTL_DECIMATION_4;
		break;
	default: /* Should never happen */
		return -EINVAL;
	}

	st->samplerate = real_sample_rate;

	return adas1000_update_reg(indio_dev, ADAS1000_FRMCTL,
		ADAS1000_FRMCTL_FRMRATE_MASK | ADAS1000_FRMCTL_DECIMATION_MASK,
		ctrl);
}

static int adas1000_get_low_pass_filter(struct iio_dev *indio_dev,
	int *freq)
{
	unsigned int val;
	int ret;

	ret = adas1000_read_reg(indio_dev, ADAS1000_FILTCTL, &val);
	if (ret)
		return ret;

	switch (val & ADAS1000_FILTCTL_LPF_MASK) {
	case ADAS1000_FILTCTL_LPF_40:
		*freq = 40;
		break;
	case ADAS1000_FILTCTL_LPF_150:
		*freq = 150;
		break;
	case ADAS1000_FILTCTL_LPF_250:
		*freq = 250;
		break;
	case ADAS1000_FILTCTL_LPF_450:
		*freq = 450;
		break;
	default:
		return -EIO;
	}

	return 0;
}

static int adas1000_set_low_pass_filter(struct iio_dev *indio_dev,
	int freq)
{
	unsigned int val;

	switch (freq) {
	case 40:
		val = ADAS1000_FILTCTL_LPF_40;
		break;
	case 150:
		val = ADAS1000_FILTCTL_LPF_150;
		break;
	case 250:
		val = ADAS1000_FILTCTL_LPF_250;
		break;
	case 450:
		val = ADAS1000_FILTCTL_LPF_450;
		break;
	default:
		return -EINVAL;
	}

	return adas1000_update_reg(indio_dev, ADAS1000_FILTCTL,
		ADAS1000_FILTCTL_LPF_MASK, val);
}

static int adas1000_get_scale(struct iio_dev *indio_dev, int *scale)
{
	unsigned int val;
	int ret;

	ret = adas1000_read_reg(indio_dev, ADAS1000_ECGCTL, &val);
	if (ret)
		return ret;

	switch (val & ADAS1000_ECGCTL_GAIN_MASK) {
	case ADAS1000_ECGCTL_GAIN_1_4:
		*scale = 19618;
		break;
	case ADAS1000_ECGCTL_GAIN_2_1:
		*scale = 13079;
		break;
	case ADAS1000_ECGCTL_GAIN_2_8:
		*scale = 9809;
		break;
	case ADAS1000_ECGCTL_GAIN_4_2:
		*scale = 6539;
		break;
	default:
		return -EIO;
	}

	return 0;
}

static int adas1000_set_scale(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int scale)
{
	unsigned int ctrl;

	switch (scale) {
	case 19618:
		ctrl = ADAS1000_ECGCTL_GAIN_1_4;
		break;
	case 13079:
		ctrl = ADAS1000_ECGCTL_GAIN_2_1;
		break;
	case 9809:
		ctrl = ADAS1000_ECGCTL_GAIN_2_8;
		break;
	case 6539:
		ctrl = ADAS1000_ECGCTL_GAIN_4_2;
		break;
	default:
		return -EINVAL;
	}

	return adas1000_update_reg(indio_dev, ADAS1000_ECGCTL,
		ADAS1000_ECGCTL_GAIN_MASK, ctrl);
}

static int adas1000_get_calib_scale(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *gain)
{
	unsigned int val;
	int ret;

	ret = adas1000_read_reg(indio_dev, ADAS1000_CAL(chan->channel), &val);
	if (ret)
		return ret;

	*gain = sign_extend32(val, 11);

	return 0;
}

static int adas1000_set_calib_scale(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int gain)
{
	if (gain < -2048 || gain > 2047)
		return -EINVAL;

	return adas1000_write_reg(indio_dev, ADAS1000_CAL(chan->channel),
		ADAS1000_CAL_USRCAL | (((unsigned int)gain) & 0xfff));
}

static int adas1000_read_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val, int *val2, long info)
{
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = adas1000_read_reg(indio_dev, ADAS1000_CALDAC, val);
		*val &= 0x3ff;

		return ret ? ret : IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = adas1000_get_sample_rate(indio_dev, val);
		return ret ? ret : IIO_VAL_INT;
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		ret = adas1000_get_low_pass_filter(indio_dev, val);
		return ret ? ret : IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
		ret = adas1000_get_calib_scale(indio_dev, chan, val);
		return ret ? ret : IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->channel) {
		case ADAS1000_CALIB_DAC:
			*val = 0;
			*val2 = 2346;
			return IIO_VAL_INT_PLUS_MICRO;
		default:
			*val = 0;
			ret = adas1000_get_scale(indio_dev, val2);
			if (ret)
				return ret;
			return IIO_VAL_INT_PLUS_NANO;
		}
		break;
	case IIO_CHAN_INFO_OFFSET:
		/* This is only set by the calibration DAC */
		*val = 128;
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int adas1000_write_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int val, int val2, long info)
{
	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (chan->channel != ADAS1000_CALIB_DAC)
			return -EINVAL;

		if (val < 0 || val > 0x3ff)
			return -EINVAL;

		return adas1000_update_reg(indio_dev, ADAS1000_CALDAC,
			ADAS1000_CALDAC_CALDATA_MASK, val & 0x3ff);
	case IIO_CHAN_INFO_SAMP_FREQ:
		return adas1000_set_sample_rate(indio_dev, val);
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		return adas1000_set_low_pass_filter(indio_dev, val);
	case IIO_CHAN_INFO_SCALE:
		if (chan->channel == ADAS1000_CALIB_DAC)
			return -EINVAL;
		return adas1000_set_scale(indio_dev, chan, val2);
	case IIO_CHAN_INFO_CALIBSCALE:
		return adas1000_set_calib_scale(indio_dev, chan, val);
	}

	return -EINVAL;
}

static int adas1000_write_raw_get_fmt(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, long info)
{
	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_INT_PLUS_NANO;
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}
}

static ssize_t adas_read_ext_chan_spec_available(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, char *buf)
{
	return sprintf(buf, "%s\n", (const char *const)priv);
}

#define ADAS_CHAN_SPEC_EXT_AVAILABLE(_name, _values) \
	{ \
		.name = (_name), \
		.shared = true, \
		.read = adas_read_ext_chan_spec_available, \
		.private = (uintptr_t)(_values), \
	}

static struct iio_chan_spec_ext_info adas1000_ext_chan_info[] = {
	ADAS_CHAN_SPEC_EXT_AVAILABLE("sampling_frequency_available",
		"128000 64000 32000 16000 8000 4000 2000 1000 500"),
	ADAS_CHAN_SPEC_EXT_AVAILABLE("scale_available",
		"0.000019618 0.000013079 0.000009809 0.000006539"),
	ADAS_CHAN_SPEC_EXT_AVAILABLE("filter_low_pass_3db_frequency_available",
		"40 150 250 400"),
	{}
};

static ssize_t adas1000_read_dac_powerdown(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	unsigned int val;
	int ret;

	ret = adas1000_read_reg(indio_dev, ADAS1000_CALDAC, &val);
	if (ret)
		return ret;

	return sprintf(buf, "%d\n", !(val & ADAS1000_CALDAC_CALDACEN));
}

static ssize_t adas1000_write_dac_powerdown(struct iio_dev *indio_dev,
	 uintptr_t private, const struct iio_chan_spec *chan, const char *buf,
	 size_t len)
{
	bool pwr_down;
	int ret;

	ret = strtobool(buf, &pwr_down);
	if (ret)
		return ret;

	ret = adas1000_update_reg(indio_dev, ADAS1000_CALDAC,
		ADAS1000_CALDAC_CALDACEN,
		pwr_down ? 0 : ADAS1000_CALDAC_CALDACEN);
	if (ret)
		return ret;

	return len;
}

static struct iio_chan_spec_ext_info adas1000_ext_chan_info_dac[] = {
	{
		.name = "powerdown",
		.read = adas1000_read_dac_powerdown,
		.write = adas1000_write_dac_powerdown,
	},
	{}
};

#define ADAS1000_ADC_CHAN(_chan1, _chan2, _address, _scan_index) {	\
	.type = IIO_VOLTAGE,				\
	.indexed = 1,					\
	.differential = 1,				\
	.channel = (_chan1),				\
	.channel2 = (_chan2),				\
	.address = (_address),				\
	.info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT	\
		| IIO_CHAN_INFO_OFFSET_SHARED_BIT	\
		| IIO_CHAN_INFO_CALIBSCALE_SEPARATE_BIT	\
		| IIO_CHAN_INFO_SAMP_FREQ_SHARED_BIT	\
		| IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY_SHARED_BIT, \
	.scan_index = (_scan_index),			\
	.scan_type = {					\
		.sign = 's',				\
		.realbits = 24,				\
		.storagebits = 32,			\
		.shift = 0,				\
		.endianness = IIO_BE,			\
	},						\
	.ext_info = adas1000_ext_chan_info,		\
}

static const struct iio_chan_spec adas1000_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = ADAS1000_CALIB_DAC,
		.address = 0,
		.info_mask = IIO_CHAN_INFO_RAW_SEPARATE_BIT |
			IIO_CHAN_INFO_SCALE_SEPARATE_BIT |
			IIO_CHAN_INFO_OFFSET_SEPARATE_BIT,
		.scan_index = -1,
		.scan_type = IIO_ST(0, 10, 16, 0),
		.ext_info = adas1000_ext_chan_info_dac,
		.extend_name = "calibration",
	},

	/* Channels which messure against VCM get a odd index, others get a even
	 * index, this allows us to easily set the required control bits. */
	ADAS1000_ADC_CHAN(ADAS1000_LA, ADAS1000_RA, ADAS1000_LADATA, 0),
	ADAS1000_ADC_CHAN(ADAS1000_LL, ADAS1000_LA, ADAS1000_LLDATA, 2),
	ADAS1000_ADC_CHAN(ADAS1000_LL, ADAS1000_RA, ADAS1000_RADATA, 4),
	ADAS1000_ADC_CHAN(ADAS1000_LA, ADAS1000_VCM, ADAS1000_LADATA, 1),
	ADAS1000_ADC_CHAN(ADAS1000_LL, ADAS1000_VCM, ADAS1000_LLDATA, 3),
	ADAS1000_ADC_CHAN(ADAS1000_RA, ADAS1000_VCM, ADAS1000_RADATA, 5),
	ADAS1000_ADC_CHAN(ADAS1000_V1, ADAS1000_VCM, ADAS1000_V1DATA, 7),
	ADAS1000_ADC_CHAN(ADAS1000_V2, ADAS1000_VCM, ADAS1000_V2DATA, 9),

};

static const struct iio_trigger_ops adas1000_trigger_ops = {
	.owner = THIS_MODULE,
};

static irqreturn_t adas1000_trigger_irq(int irq, void *data)
{
	struct iio_trigger *trig = data;
	struct adas1000_state *st = trig->private_data;

	if (!st->sampling) {
		disable_irq_nosync(irq);
		st->irq_enabled = false;
		return IRQ_HANDLED;
	}

	iio_trigger_poll(trig, iio_get_time_ns());
	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

static struct iio_trigger *adas1000_allocate_trigger(struct iio_dev *indio_dev,
	int irq)
{
	struct iio_trigger *trig;
	int ret;

	trig = iio_trigger_alloc("%s-dev%d", indio_dev->name, indio_dev->id);
	if (!trig)
		return NULL;

	trig->dev.parent = indio_dev->dev.parent;
	trig->ops = &adas1000_trigger_ops;
	trig->private_data = iio_priv(indio_dev);

	ret = request_irq(irq, adas1000_trigger_irq, IRQF_TRIGGER_LOW,
			trig->name, trig);
	if (ret)
		return NULL;

	ret = iio_trigger_register(trig);
	if (ret) {
		free_irq(irq, trig);
		return NULL;
	}

	return trig;
}

struct adas1000_attribute {
	struct device_attribute dev_attr;

	unsigned int reg;
	unsigned int mask;
	bool invert;
};

static inline struct adas1000_attribute *dev_attr_to_adas1000_attr(
	struct device_attribute *dev_attr)
{
	return container_of(dev_attr, struct adas1000_attribute, dev_attr);
}

static ssize_t adas1000_dev_attr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct adas1000_attribute *adas1000_attr = dev_attr_to_adas1000_attr(attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	unsigned int val;
	int ret;

	ret = adas1000_read_reg(indio_dev, adas1000_attr->reg, &val);
	if (ret)
		return ret;
	val &= adas1000_attr->mask;

	if (adas1000_attr->invert)
		return sprintf(buf, "%d\n", !(bool)val);
	else
		return sprintf(buf, "%d\n", (bool)val);
}

static ssize_t adas1000_dev_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct adas1000_attribute *adas1000_attr = dev_attr_to_adas1000_attr(attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	unsigned int val;
	bool enable;
	int ret;

	ret = strtobool(buf, &enable);
	if (ret)
		return ret;

	if (enable ^ adas1000_attr->invert)
		val = adas1000_attr->mask;
	else
		val = 0;

	ret = adas1000_update_reg(indio_dev, adas1000_attr->reg,
		adas1000_attr->mask, val);
	if (ret)
		return ret;

	return len;
}

#define ADAS1000_DEVICE_ATTR(_name, _mode, _reg, _mask, _invert) \
	struct adas1000_attribute adas1000_dev_attr_##_name = { \
		.reg = _reg, \
		.mask = _mask, \
		.invert = _invert, \
		.dev_attr = __ATTR(_name, _mode, adas1000_dev_attr_show, \
			adas1000_dev_attr_store), \
	}

static const char * const adas1000_out_select[] = {
	"rl",
	"la",
	"ll",
	"ra",
	"v1",
	"v2",
};

static ssize_t adas1000_out_select_attr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	unsigned int val;
	int ret;

	ret = adas1000_read_reg(indio_dev, ADAS1000_CMREFCTL, &val);
	if (ret)
		return ret;

	val &= ADAS1000_CMREFCTL_RLDSEL_MASK;
	val >>= ADAS1000_CMREFCTL_RLDSEL_OFFSET;

	if (val >= ARRAY_SIZE(adas1000_out_select))
		return -EIO;

	return sprintf(buf, "%s\n", adas1000_out_select[val]);
}

static ssize_t adas1000_out_select_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	unsigned int val;
	int ret;

	for (val = 0; val < ARRAY_SIZE(adas1000_out_select); val++) {
		if (sysfs_streq(buf, adas1000_out_select[val]))
			break;
	}

	if (val == ARRAY_SIZE(adas1000_out_select))
		return -EINVAL;

	val <<= ADAS1000_CMREFCTL_RLDSEL_OFFSET;

	ret = adas1000_update_reg(indio_dev, ADAS1000_CMREFCTL,
		ADAS1000_CMREFCTL_RLDSEL_MASK, val);
	if (ret)
		return ret;

	return len;
}

static DEVICE_ATTR(rld_out_select, 0644, adas1000_out_select_attr_show,
	adas1000_out_select_attr_store);
static IIO_CONST_ATTR(rld_out_select_available, "rl la ll ra v1 v2");

static ADAS1000_DEVICE_ATTR(rld_powerdown, 0644,
	ADAS1000_CMREFCTL, ADAS1000_CMREFCTL_RLDEN, true);
static ADAS1000_DEVICE_ATTR(shield_driver_powerdown, 0644,
	ADAS1000_CMREFCTL, ADAS1000_CMREFCTL_SHLDEN, true);
static ADAS1000_DEVICE_ATTR(rld_summing_junction_la_en, 0644,
	ADAS1000_CMREFCTL, ADAS1000_CMREFCTL_LARLD, false);
static ADAS1000_DEVICE_ATTR(rld_summing_junction_ll_en, 0644,
	ADAS1000_CMREFCTL, ADAS1000_CMREFCTL_LLRLD, false);
static ADAS1000_DEVICE_ATTR(rld_summing_junction_ra_en, 0644,
	ADAS1000_CMREFCTL, ADAS1000_CMREFCTL_RARLD, false);
static ADAS1000_DEVICE_ATTR(rld_summing_junction_v1_en, 0644,
	ADAS1000_CMREFCTL, ADAS1000_CMREFCTL_V1RLD, false);
static ADAS1000_DEVICE_ATTR(rld_summing_junction_v2_en, 0644,
	ADAS1000_CMREFCTL, ADAS1000_CMREFCTL_V2RLD, false);
static ADAS1000_DEVICE_ATTR(rld_summing_junction_ce_en, 0644,
	ADAS1000_CMREFCTL, ADAS1000_CMREFCTL_CERLD, false);
static ADAS1000_DEVICE_ATTR(vcm_summing_junction_la_en, 0644,
	ADAS1000_CMREFCTL, ADAS1000_CMREFCTL_LACM, false);
static ADAS1000_DEVICE_ATTR(vcm_summing_junction_ll_en, 0644,
	ADAS1000_CMREFCTL, ADAS1000_CMREFCTL_LLCM, false);
static ADAS1000_DEVICE_ATTR(vcm_summing_junction_ra_en, 0644,
	ADAS1000_CMREFCTL, ADAS1000_CMREFCTL_RACM, false);
static ADAS1000_DEVICE_ATTR(vcm_summing_junction_v1_en, 0644,
	ADAS1000_CMREFCTL, ADAS1000_CMREFCTL_V1CM, false);
static ADAS1000_DEVICE_ATTR(vcm_summing_junction_v2_en, 0644,
	ADAS1000_CMREFCTL, ADAS1000_CMREFCTL_V2CM, false);
static ADAS1000_DEVICE_ATTR(vcm_summing_junction_ce_en, 0644,
	ADAS1000_CMREFCTL, ADAS1000_CMREFCTL_CEREFEN, false);

static struct attribute *adas1000_attributes[] = {
	&dev_attr_rld_out_select.attr,
	&iio_const_attr_rld_out_select_available.dev_attr.attr,
	&adas1000_dev_attr_shield_driver_powerdown.dev_attr.attr,
	&adas1000_dev_attr_rld_powerdown.dev_attr.attr,
	&adas1000_dev_attr_rld_summing_junction_la_en.dev_attr.attr,
	&adas1000_dev_attr_rld_summing_junction_ll_en.dev_attr.attr,
	&adas1000_dev_attr_rld_summing_junction_ra_en.dev_attr.attr,
	&adas1000_dev_attr_rld_summing_junction_v1_en.dev_attr.attr,
	&adas1000_dev_attr_rld_summing_junction_v2_en.dev_attr.attr,
	&adas1000_dev_attr_rld_summing_junction_ce_en.dev_attr.attr,
	&adas1000_dev_attr_vcm_summing_junction_la_en.dev_attr.attr,
	&adas1000_dev_attr_vcm_summing_junction_ll_en.dev_attr.attr,
	&adas1000_dev_attr_vcm_summing_junction_ra_en.dev_attr.attr,
	&adas1000_dev_attr_vcm_summing_junction_v1_en.dev_attr.attr,
	&adas1000_dev_attr_vcm_summing_junction_v2_en.dev_attr.attr,
	&adas1000_dev_attr_vcm_summing_junction_ce_en.dev_attr.attr,
	NULL,
};

static struct attribute_group adas1000_attribute_group = {
	.attrs = adas1000_attributes,
};

static const struct iio_info adas1000_info = {
	.write_raw = adas1000_write_raw,
	.read_raw = adas1000_read_raw,
	.write_raw_get_fmt = adas1000_write_raw_get_fmt,
	.update_scan_mode = adas1000_update_scan_mode,
	.debugfs_reg_access = adas1000_debugfs_reg_access,
	.attrs = &adas1000_attribute_group,
	.driver_module = THIS_MODULE,
};

static int __devinit adas1000_inital_setup(struct iio_dev *indio_dev,
	struct adas1000_platform_data *pdata)
{
	unsigned int ctrl_reg, cmref_reg;
	int ret;

	ret = adas1000_write_reg(indio_dev, ADAS1000_ECGCTL,
		ADAS1000_ECGCTL_RESET);
	if (ret)
		return ret;

	/* Always electrode mode */
	ret = adas1000_update_reg(indio_dev, ADAS1000_FRMCTL,
		ADAS1000_FRMCTL_DATAFMT, ADAS1000_FRMCTL_DATAFMT);
	if (ret)
		return ret;

	if (!pdata)
		return 0;

	ctrl_reg = 0;
	cmref_reg = 0;

	if (pdata->high_performance)
		ctrl_reg |= ADAS1000_ECGCTL_HP;

	if (pdata->enable_vref_buffer)
		ctrl_reg |= ADAS1000_ECGCTL_VREFBUF;

	if (pdata->use_external_clock)
		ctrl_reg |= ADAS1000_ECGCTL_CLKEXT;

	if (pdata->drive_external_common_mode)
		cmref_reg |= ADAS1000_CMREFCTL_DRVCM;

	if (pdata->use_external_common_mode)
		cmref_reg |= ADAS1000_CMREFCTL_EXTCM;

	ret = adas1000_update_reg(indio_dev, ADAS1000_ECGCTL,
		ADAS1000_ECGCTL_HP | ADAS1000_ECGCTL_VREFBUF |
		ADAS1000_ECGCTL_CLKEXT, ctrl_reg);
	if (ret)
		return ret;

	ret = adas1000_update_reg(indio_dev, ADAS1000_CMREFCTL,
		ADAS1000_CMREFCTL_DRVCM | ADAS1000_CMREFCTL_EXTCM,
		cmref_reg);

	return ret;
}

static int __devinit adas1000_probe(struct spi_device *spi)
{
	struct adas1000_platform_data *pdata = spi->dev.platform_data;
	struct iio_dev *indio_dev;
	struct adas1000_state *st;
	int ret;

	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	mutex_init(&st->lock);
	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &adas1000_info;
	indio_dev->channels = adas1000_channels;
	indio_dev->num_channels = ARRAY_SIZE(adas1000_channels);

	ret = adas1000_inital_setup(indio_dev, pdata);
	if (ret)
		goto error_free;

	ret = iio_triggered_buffer_setup(indio_dev, &iio_pollfunc_store_time,
		&adas1000_trigger_handler, &iio_triggered_buffer_setup_ops);
	if (ret)
		goto error_free;

	st->irq_enabled = true;
	if (spi->irq)
		st->trig = adas1000_allocate_trigger(indio_dev, spi->irq);

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_buffer_cleanup;

	adaus1000_debugfs_init(indio_dev);

	return 0;

error_buffer_cleanup:
	iio_triggered_buffer_cleanup(indio_dev);
error_free:
	iio_device_free(indio_dev);

	return ret;
}

static int __devexit adas1000_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct adas1000_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	kfree(st->frame_data);
	mutex_destroy(&st->lock);
	iio_device_free(indio_dev);

	return 0;
}

static const struct spi_device_id adas1000_ids[] = {
	{"adas1000", 0},
	{"adas1000-1", 0},
	{ }
};
MODULE_DEVICE_TABLE(spi, adas1000_ids);

static struct spi_driver adas1000_driver = {
	.driver = {
		.name	= "adas1000",
		.owner	= THIS_MODULE,
	},
	.probe		= adas1000_probe,
	.remove		= __devexit_p(adas1000_remove),
	.id_table	= adas1000_ids,
};
module_spi_driver(adas1000_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Analog Devices ADAS1000");
MODULE_LICENSE("GPL v2");
