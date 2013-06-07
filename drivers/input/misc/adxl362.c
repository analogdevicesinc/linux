/*
 * ADXL362 Three-Axis Digital Accelerometers
 *
 * Copyright (C) 2012 Michael Hennerich, Analog Devices Inc.
 * Licensed under the GPL-2.
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/spi/spi.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/log2.h>
#include <asm/unaligned.h>
#include <asm/byteorder.h>

#include <linux/input/adxl362.h>

#define REG_2B			(1 << 9) /* 16-bit register */
#define ADXL_REG_FIFO		(1 << 8) /* dummy register */

/* ADXL362 Register Map */
#define ADXL_REG_DEVID_AD	0x00
#define ADXL_REG_DEVID_MST	0x01
#define ADXL_REG_PARTID		0x02
#define ADXL_REG_REVID		0x03
#define ADXL_REG_XDATA		0x08
#define ADXL_REG_YDATA		0x09
#define ADXL_REG_ZDATA		0x0A
#define ADXL_REG_STATUS		0x0B
#define ADXL_REG_FIFO_ENTRIES_L	(0x0C | REG_2B)
#define ADXL_REG_FIFO_ENTRIES_H	0x0D
#define ADXL_REG_XDATA_L	(0x0E | REG_2B)
#define ADXL_REG_XDATA_H	0x0F
#define ADXL_REG_YDATA_L	(0x10 | REG_2B)
#define ADXL_REG_YDATA_H	0x11
#define ADXL_REG_ZDATA_L	(0x12 | REG_2B)
#define ADXL_REG_ZDATA_H	0x13
#define ADXL_REG_TEMP_L		(0x14 | REG_2B)
#define ADXL_REG_TEMP_H		0x15
#define ADXL_REG_SOFT_RESET	0x1F
#define ADXL_REG_THRESH_ACT_L	(0x20 | REG_2B)
#define ADXL_REG_THRESH_ACT_H	0x21
#define ADXL_REG_TIME_ACT	0x22
#define ADXL_REG_THRESH_INACT_L	(0x23 | REG_2B)
#define ADXL_REG_THRESH_INACT_H	0x24
#define ADXL_REG_TIME_INACT_L	(0x25 | REG_2B)
#define ADXL_REG_TIME_INACT_H	0x26
#define ADXL_REG_ACT_INACT_CTL	0x27
#define ADXL_REG_FIFO_CTL	0x28
#define ADXL_REG_FIFO_SAMPLES	0x29
#define ADXL_REG_INTMAP1	0x2A
#define ADXL_REG_INTMAP2	0x2B
#define ADXL_REG_FILTER_CTL	0x2C
#define ADXL_REG_POWER_CTL	0x2D
#define ADXL_REG_SELF_TEST	0x2E

/* STATUS */
#define ADXL_ERR_USER_REGS	(1 << 7)
#define ADXL_AWAKE		(1 << 6)
#define ADXL_INACT		(1 << 5)
#define ADXL_ACT		(1 << 4)
#define ADXL_FIFO_OVERRUN	(1 << 3)
#define ADXL_FIFO_WATERMARK	(1 << 2)
#define ADXL_FIFO_READY		(1 << 1)
#define ADXL_DATA_READY		(1 << 0)

/* ACT_INACT_CTL */
#define ADXL_LINK_LOOP(x)	(((x) & 0x3) << 4)
#define ADXL_DEFA_MODE		0
#define ADXL_LINK_MODE		2
#define ADXL_LOOP_MODE		3
#define ADXL_INACT_REF		(1 << 3)
#define ADXL_INACT_EN		(1 << 2)
#define ADXL_ACT_REF		(1 << 1)
#define ADXL_ACT_EN		(1 << 0)

/* FILTER_CTL */
#define ADXL_RANGE(x)		((x) << 6)
#define ADXL_HALF_BW		(1 << 4)
#define ADXL_EXT_SAMPLE		(1 << 3)
#define ADXL_ODR(x)		((x) & 0x7)

/* FIFO_CTL */
#define ADXL_FIFO_AH		(1 << 3)
#define ADXL_FIFO_TEMP_EN	(1 << 2)
#define ADXL_FIFO_MODE(x)	(((x) & 0x3) << 0)
#define ADXL_FIFO_MODE_DIS	0
#define ADXL_FIFO_MODE_OLDEST	1
#define ADXL_FIFO_MODE_STREAM	2
#define ADXL_FIFO_MODE_TRIG	3

/* INTMAP1/INTMAP2 */
#define ADXL_INT_LOW_ACTIVE	(1 << 7)
#define ADXL_INT_AWAKE_EN	(1 << 6)
#define ADXL_INT_INACT_EN	(1 << 5)
#define ADXL_INT_ACT_EN		(1 << 4)
#define ADXL_INT_FIFO_OVERRUN_EN (1 << 3)
#define ADXL_INT_FIFO_WATERMARK_EN (1 << 2)
#define ADXL_INT_FIFO_READY_EN	(1 << 1)
#define ADXL_INT_DATA_READY_EN	(1 << 0)

/* POWER_CTL */
#define ADXL_EXT_CLK		(1 << 6)
#define ADXL_LOW_NOISE(x)	((x) << 4)
#define ADXL_WAKE_UP		(1 << 3)
#define ADXL_AUTOSLEEP		(1 << 2)
#define ADXL_MEASUREMENT_MODE	(1 << 1)

/* FIFO Buffer */
#define X_AXIS			0
#define Y_AXIS			1
#define Z_AXIS			2
#define TEMP			3
#define FIFO_ITEM(x)		((x) >> 14)
#define FIFO_ITEM_MASK		(3 << 14)

/* device specifics */
#define MAN_ID_AD		0xAD
#define PART_ID_ADXL362		0xF2 /* octal 362 */
#define MIN_FIFO_SETS		1
#define MAX_T_FIFO_SETS		170

/* SPI specifics */
#define MAX_SPI_FREQ_HZ		5000000
#define ADXL34X_CMD_FIFO	0xD
#define ADXL34X_CMD_READ	0xB
#define ADXL34X_CMD_WRITE	0xA

/* internal math helpers */
#define SIGN(x)			((x) < 0 ? -1 : 1)
#define ODR_TO_HZ(x)		((125 << (x)) / 10)
#define HZ_TO_ODR(x)		(ilog2(((x) * 10) / 125))
#define CLAMP_WM(x)		(clamp_t(u8, x, MIN_FIFO_SETS, MAX_T_FIFO_SETS))
#define CLAMP_ACT(x)		(clamp_t(u8, x, 1, 0xFF))
#define CLAMP_INACT(x)		(clamp_t(u16, x, 1, 0xFFFF))

struct adxl362_axis_event {
	unsigned code;
	int scale;
};

struct adxl362_state {
	struct input_dev *input;
	struct spi_device *spi;
	struct mutex mutex;	/* reentrant protection for struct */
	struct adxl362_platform_data *pdata;
	struct spi_message msg;
	struct spi_transfer xfers[2];
	struct adxl362_axis_event axis_event[3];
	struct delayed_work work; /* !irq */
	unsigned long delay; /* !irq */
	unsigned irq;
	bool opened;	/* P: mutex */
	bool suspended;	/* P: mutex */
	char phys[32];
	unsigned char int_mask;
	unsigned char power_ctl;
	unsigned char filter_ctl;
	unsigned char fifo_ctl;
	unsigned char act_inact_ctl;
	unsigned char watermarks_odr[6];

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	__le16 data[512] ____cacheline_aligned;
};

static struct adxl362_platform_data adxl362_default_init = {
	.data_rate = ADXL_ODR_100HZ,	/* 100Hz */
	.data_range = ADXL_RANGE_PM_2g,	/* +/- 2000mg */
	.activity_threshold = 70,	/* 70mg (referenced) */
	.inactivity_threshold = 30,	/* 30mg (referenced) */
	.inactivity_time = 10000,	/* 10s */
	.activity_time = 1,		/* 1ms */
	.referenced_activity_en = true, /* cancel static accel. of gravity */
	.referenced_inactivity_en = true, /* cancel static accel. of gravity */
	.watermark_odr_12Hz = 1,
	.watermark_odr_25Hz = 1,
	.watermark_odr_50Hz = 1,
	.watermark_odr_100Hz = 1,
	.watermark_odr_200Hz = 2, /* limit irq/poll interval to 10ms */
	.watermark_odr_400Hz = 4, /* limit irq/poll interval to 10ms */
	.ev_code_x = ABS_X,		/* default mapping */
	.ev_code_y = ABS_Y,
	.ev_code_z = ABS_Z,
};

static int adxl362_read(struct spi_device *spi, unsigned reg)
{
	unsigned char buf[4];
	unsigned rxcnt;
	ssize_t status;

	buf[0] = ADXL34X_CMD_READ;
	buf[1] = reg;

	if (reg & REG_2B) {
		rxcnt = 2;
	} else {
		rxcnt = 1;
		buf[3] = 0;
	}

	status = spi_write_then_read(spi, &buf[0], 2, &buf[2], rxcnt);

	return (status < 0) ? status : get_unaligned_le16(&buf[2]);
}

static int adxl362_write(struct spi_device *spi,
			     unsigned reg, unsigned val)
{
	unsigned char buf[4];
	unsigned txcnt = 3;

	buf[0] = ADXL34X_CMD_WRITE;
	buf[1] = reg;
	buf[2] = val;

	if (reg & REG_2B) {
		buf[3] = val >> 8; /* high byte last */
		txcnt++;
	}

	return spi_write_then_read(spi, &buf[0], txcnt, NULL, 0);
}

/*
 * Speed path spi access function:
 * Users call sequentially from the irq threaded/poll handler,
 * no additional buffer locks required.
 */

static int adxl362_read_block(struct spi_device *spi,
				  unsigned reg, unsigned count,
				  void *buf)
{
	struct adxl362_state *ac = dev_get_drvdata(&spi->dev);
	unsigned char *txbuf = buf;
	int ret;

	if (reg == ADXL_REG_FIFO) {
		txbuf[0] = ADXL34X_CMD_FIFO;
		ac->xfers[0].len = 1;
	} else {
		txbuf[0] = ADXL34X_CMD_READ;
		txbuf[1] = reg;
		ac->xfers[0].len = 2;
	}

	ac->xfers[0].tx_buf = buf;
	ac->xfers[1].len = count;
	ac->xfers[1].rx_buf = ac->data;

	ret = spi_sync(spi, &ac->msg);
	if (ret)
		dev_err(&spi->dev, "block read failure");

	return ret;
}

static int adxl362_fifo_watermark(struct adxl362_state *ac, unsigned odr_val)
{
	unsigned char val, tmp;
	unsigned short samples;
	int ret;

	val = ac->watermarks_odr[odr_val];

	if (!ac->irq) /* poll mode */
		return val; /* return value to calculate delayed work */

	samples = val * 3; /* 512 entry fifo counts in samples not sets */
	tmp = ac->fifo_ctl;

	if (samples > 255) /* MSB is stored in ADXL_REG_FIFO_CTL */
		ac->fifo_ctl |= ADXL_FIFO_AH;
	else
		ac->fifo_ctl &= ~ADXL_FIFO_AH;

	if (tmp != ac->fifo_ctl) { /* avoid surplus writes */
		ret = adxl362_write(ac->spi, ADXL_REG_FIFO_CTL, ac->fifo_ctl);
		if (ret < 0)
			return ret;
	}

	return adxl362_write(ac->spi, ADXL_REG_FIFO_SAMPLES, samples & 0xFF);
}

static int adxl362_get_status(struct adxl362_state *ac,
			      unsigned *stat, unsigned *samples)
{
	unsigned char *buf = (u8 *)ac->data;
	int ret;

	/* read status and fifo count in one shot to reduce SPI overhead */

	ret = adxl362_read_block(ac->spi, ADXL_REG_STATUS, 3, buf);
	if (ret < 0)
		dev_err(&ac->spi->dev, "failed to query int stat\n");

	*stat = buf[0];
	*samples = get_unaligned_le16(&buf[1]); /* ADXL_REG_FIFO_ENTRIES_L */

	return ret;
}

static void adxl362_service_ev_fifo(struct adxl362_state *ac, unsigned cnt)
{
	unsigned short fifo_val;
	int i, ret;

	ret = adxl362_read_block(ac->spi, ADXL_REG_FIFO, cnt * 2, ac->data);
	if (ret < 0)
		dev_err(&ac->spi->dev, "failed to read fifo\n");

	for (i = 0; i < cnt; i++) {
		fifo_val = le16_to_cpu(ac->data[i]);
		input_report_abs(ac->input,
				 ac->axis_event[FIFO_ITEM(fifo_val)].code,
				 sign_extend32(fifo_val, 12) *
				 ac->axis_event[FIFO_ITEM(fifo_val)].scale);
		if (FIFO_ITEM(fifo_val) == Z_AXIS)
			input_sync(ac->input);
	}
}

static int adxl362_service(struct adxl362_state *ac)
{
	unsigned stat, samples;
	int ret;

	ret = adxl362_get_status(ac, &stat, &samples);
	if (stat & ADXL_FIFO_OVERRUN)
		dev_err(&ac->spi->dev, "FIFO_OVERRUN\n");

	if (stat & ADXL_ERR_USER_REGS)
		dev_err(&ac->spi->dev, "ERR_USER_REGS\n");

	if (stat & (ADXL_FIFO_READY | ADXL_FIFO_WATERMARK))
		adxl362_service_ev_fifo(ac, samples);

	return ret;
}

static unsigned long adxl362_calc_poll_rate(unsigned short rate,
					    unsigned short wmark)
{
	unsigned long delay = msecs_to_jiffies((1000 * wmark) / rate);

	if (delay >= HZ)
		delay = round_jiffies_relative(delay);

	return delay;
}

static inline void adxl362_queue_work(struct adxl362_state *ac)
{
	queue_delayed_work(system_freezable_wq, &ac->work, ac->delay);
}

static void adxl362_work(struct work_struct *work)
{
	struct adxl362_state *ac =
		container_of(work, struct adxl362_state, work.work);

	adxl362_service(ac);
	adxl362_queue_work(ac);
}

static irqreturn_t adxl362_irq(int irq, void *handle)
{
	struct adxl362_state *ac = handle;

	adxl362_service(ac);

	return IRQ_HANDLED;
}

static inline void __adxl362_disable(struct adxl362_state *ac)
{
	adxl362_write(ac->spi, ADXL_REG_POWER_CTL, 0);
}

static inline void __adxl362_enable(struct adxl362_state *ac)
{
	adxl362_write(ac->spi, ADXL_REG_POWER_CTL,
		      ac->power_ctl | ADXL_MEASUREMENT_MODE);
}

static ssize_t adxl362_rate_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct adxl362_state *ac = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ODR_TO_HZ(ADXL_ODR(ac->filter_ctl)));
}

static ssize_t adxl362_rate_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct adxl362_state *ac = dev_get_drvdata(dev);
	unsigned short rate, wmark, odr_val;
	int ret;

	ret = kstrtou16(buf, 10, &rate);
	if (ret)
		return ret;

	rate = clamp_t(u16, rate, 13, 400);

	mutex_lock(&ac->mutex);

	odr_val = ADXL_ODR(HZ_TO_ODR(rate));
	ac->filter_ctl &= ~ADXL_ODR(~0);
	ac->filter_ctl |= odr_val;

	adxl362_write(ac->spi, ADXL_REG_FILTER_CTL, ac->filter_ctl);
	wmark = adxl362_fifo_watermark(ac, odr_val);

	/* update [in]activity timers with new rate */
	adxl362_write(ac->spi, ADXL_REG_TIME_INACT_L,
		CLAMP_INACT((ac->pdata->inactivity_time * rate) / 1000));
	adxl362_write(ac->spi, ADXL_REG_TIME_ACT,
		CLAMP_ACT((ac->pdata->activity_time * rate) / 1000));

	if (!ac->irq) { /* poll mode */
		ac->delay = adxl362_calc_poll_rate(rate, wmark);
		if (ac->opened && !ac->suspended) {
			cancel_delayed_work_sync(&ac->work);
			adxl362_queue_work(ac);
		}
	}

	mutex_unlock(&ac->mutex);

	return count;
}

static DEVICE_ATTR(rate, 0664, adxl362_rate_show, adxl362_rate_store);

static ssize_t adxl362_autosleep_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct adxl362_state *ac = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", !!(ac->power_ctl & ADXL_AUTOSLEEP));
}

static ssize_t adxl362_autosleep_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct adxl362_state *ac = dev_get_drvdata(dev);
	bool val;
	int error;

	error = strtobool(buf, &val);
	if (error)
		return error;

	mutex_lock(&ac->mutex);

	if (val)
		ac->power_ctl |= ADXL_AUTOSLEEP;
	else
		ac->power_ctl &= ~ADXL_AUTOSLEEP;

	adxl362_write(ac->spi, ADXL_REG_POWER_CTL, ac->power_ctl);

	mutex_unlock(&ac->mutex);

	return count;
}

static DEVICE_ATTR(autosleep, 0664,
		   adxl362_autosleep_show, adxl362_autosleep_store);

static struct attribute *adxl362_attributes[] = {
	&dev_attr_rate.attr,
	&dev_attr_autosleep.attr,
	NULL
};

static const struct attribute_group adxl362_attr_group = {
	.attrs = adxl362_attributes,
};

static int adxl362_setup(struct adxl362_state *ac)
{
	struct adxl362_platform_data *pdata = ac->pdata;
	unsigned char scale = pdata->data_range / 2; /* scale div 1, 2, 4 */
	unsigned odr_val;
	int ret;

	ac->axis_event[X_AXIS].code = abs(pdata->ev_code_x);
	ac->axis_event[Y_AXIS].code = abs(pdata->ev_code_y);
	ac->axis_event[Z_AXIS].code = abs(pdata->ev_code_z);
	ac->axis_event[X_AXIS].scale = SIGN(pdata->ev_code_x) * scale;
	ac->axis_event[Y_AXIS].scale = SIGN(pdata->ev_code_y) * scale;
	ac->axis_event[Z_AXIS].scale = SIGN(pdata->ev_code_z) * scale;

	odr_val = ADXL_ODR(HZ_TO_ODR(pdata->data_rate));
	ac->power_ctl = ADXL_LOW_NOISE(pdata->low_power_mode);
	ac->filter_ctl = ADXL_RANGE(pdata->data_range) |
			 (pdata->half_bw ? ADXL_HALF_BW : 0) |
			  odr_val;
	ac->act_inact_ctl = ADXL_LINK_LOOP(ADXL_LOOP_MODE) |
			(pdata->referenced_activity_en ? ADXL_ACT_REF : 0) |
			(pdata->referenced_inactivity_en ? ADXL_INACT_REF : 0) |
			ADXL_INACT_EN | ADXL_ACT_EN;

	ac->fifo_ctl = ADXL_FIFO_MODE(ADXL_FIFO_MODE_STREAM);

	ac->watermarks_odr[0] = CLAMP_WM(pdata->watermark_odr_12Hz);
	ac->watermarks_odr[1] = CLAMP_WM(pdata->watermark_odr_25Hz);
	ac->watermarks_odr[2] = CLAMP_WM(pdata->watermark_odr_50Hz);
	ac->watermarks_odr[3] = CLAMP_WM(pdata->watermark_odr_100Hz);
	ac->watermarks_odr[4] = CLAMP_WM(pdata->watermark_odr_200Hz);
	ac->watermarks_odr[5] = CLAMP_WM(pdata->watermark_odr_400Hz);

	ret = adxl362_write(ac->spi, ADXL_REG_THRESH_ACT_L,
			    pdata->activity_threshold / scale);
	if (ret < 0)
		return ret;

	ret = adxl362_write(ac->spi, ADXL_REG_TIME_ACT,
			    CLAMP_ACT((pdata->activity_time *
			    ODR_TO_HZ(odr_val)) / 1000));
	if (ret < 0)
		return ret;

	ret = adxl362_write(ac->spi, ADXL_REG_THRESH_INACT_L,
			    pdata->inactivity_threshold / scale);
	if (ret < 0)
		return ret;

	ret = adxl362_write(ac->spi, ADXL_REG_TIME_INACT_L,
			    CLAMP_INACT((pdata->inactivity_time *
			    ODR_TO_HZ(odr_val)) / 1000));
	if (ret < 0)
		return ret;

	ret = adxl362_write(ac->spi, ADXL_REG_ACT_INACT_CTL,
			    ac->act_inact_ctl);
	if (ret < 0)
		return ret;

	ret = adxl362_write(ac->spi, ADXL_REG_FIFO_CTL, ac->fifo_ctl);
	if (ret < 0)
		return ret;

	ret = adxl362_fifo_watermark(ac, odr_val);
	if (ret < 0)
		return ret;

	if (!ac->irq) { /* poll mode */
		ac->delay = adxl362_calc_poll_rate(
			ODR_TO_HZ(ADXL_ODR(ac->filter_ctl)), ret);
	} else {
		ret = adxl362_write(ac->spi, pdata->use_int2 ?
			ADXL_REG_INTMAP2 : ADXL_REG_INTMAP1, ac->int_mask);
		if (ret < 0)
			return ret;
	}

	return adxl362_write(ac->spi, ADXL_REG_FILTER_CTL, ac->filter_ctl);
}

static int adxl362_input_open(struct input_dev *input)
{
	struct adxl362_state *ac = input_get_drvdata(input);

	mutex_lock(&ac->mutex);

	if (!ac->suspended)
		__adxl362_enable(ac);

	ac->opened = true;

	if (!ac->irq) { /* poll mode */
		adxl362_service(ac);
		adxl362_queue_work(ac);
	}

	mutex_unlock(&ac->mutex);

	return 0;
}

static void adxl362_input_close(struct input_dev *input)
{
	struct adxl362_state *ac = input_get_drvdata(input);

	mutex_lock(&ac->mutex);

	if (!ac->suspended)
		__adxl362_disable(ac);

	ac->opened = false;

	if (!ac->irq) /* poll mode */
		cancel_delayed_work_sync(&ac->work);

	mutex_unlock(&ac->mutex);
}

static int adxl362_probe(struct spi_device *spi)
{
	struct adxl362_platform_data *pdata = spi->dev.platform_data;
	struct input_dev *input_dev;
	struct adxl362_state *ac;
	struct device *dev = &spi->dev;
	int ret, man_id, part_id, range;
	unsigned long irqflags;

	/* don't exceed max specified SPI CLK frequency */
	if (spi->max_speed_hz > MAX_SPI_FREQ_HZ) {
		dev_err(dev, "SPI CLK %d Hz too fast\n", spi->max_speed_hz);
		return -EINVAL;
	}

	ac = devm_kzalloc(dev, sizeof(*ac), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ac || !input_dev) {
		ret = -ENOMEM;
		goto err_free_mem;
	}

	if (!pdata) {
		dev_dbg(dev,
			"No platform data: Using default initialization\n");
		pdata = &adxl362_default_init;
	}

	spi_set_drvdata(spi, ac);
	mutex_init(&ac->mutex);

	ac->input = input_dev;
	ac->pdata = pdata;
	ac->spi = spi;
	ac->irq = spi->irq;

	/* query device presence */
	man_id = adxl362_read(spi, ADXL_REG_DEVID_AD);
	part_id = adxl362_read(spi, ADXL_REG_PARTID);

	if (man_id != MAN_ID_AD || part_id != PART_ID_ADXL362) {
		dev_err(dev, "Failed to probe (0x%X:0x%X)\n", man_id, part_id);
		ret = -ENODEV;
		goto err_free_mem;
	}

	/* now set to a known state */
	adxl362_write(ac->spi, ADXL_REG_SOFT_RESET, 'R'); /* reset */

	/* setup speed path default message */
	spi_message_init(&ac->msg);
	spi_message_add_tail(&ac->xfers[0], &ac->msg);
	spi_message_add_tail(&ac->xfers[1], &ac->msg);
	ac->xfers[0].bits_per_word = 8;
	ac->xfers[1].bits_per_word = 8;

	/* setup input device */
	snprintf(ac->phys, sizeof(ac->phys), "%s/input0", dev_name(dev));
	input_dev->name = "ADXL362 accelerometer";
	input_dev->phys = ac->phys;
	input_dev->dev.parent = dev;
	input_dev->id.bustype = BUS_SPI;
	input_dev->id.vendor = man_id;
	input_dev->id.product = part_id;
	input_dev->id.version = adxl362_read(spi, ADXL_REG_REVID);
	input_dev->open = adxl362_input_open;
	input_dev->close = adxl362_input_close;

	input_set_drvdata(input_dev, ac);

	__set_bit(EV_ABS, input_dev->evbit);

	range = pdata->data_range * 1000; /* +/- 2000, 4000, 8000 mg */

	input_set_abs_params(input_dev, ABS_X, -range, range,
			     pdata->abs_fuzz, 0);
	input_set_abs_params(input_dev, ABS_Y, -range, range,
			     pdata->abs_fuzz, 0);
	input_set_abs_params(input_dev, ABS_Z, -range, range,
			     pdata->abs_fuzz, 0);

	/* request irq or init polled mode if desired */
	if (spi->irq) {
		ac->int_mask = ADXL_INT_FIFO_WATERMARK_EN |
			       ADXL_INT_FIFO_OVERRUN_EN;

		if (pdata->irqflags)
			irqflags = pdata->irqflags & IRQF_TRIGGER_MASK;
		else
			irqflags = IRQF_TRIGGER_HIGH;

		if (irqflags & (IRQF_TRIGGER_LOW | IRQF_TRIGGER_FALLING))
			ac->int_mask |= ADXL_INT_LOW_ACTIVE;

		ret = request_threaded_irq(spi->irq, NULL, adxl362_irq,
					irqflags | IRQF_ONESHOT,
					dev_name(dev), ac);
		if (ret) {
			dev_err(dev, "irq %d busy?\n", spi->irq);
			goto err_free_mem;
		}
	} else {
		INIT_DELAYED_WORK(&ac->work, adxl362_work);
	}

	ret = sysfs_create_group(&dev->kobj, &adxl362_attr_group);
	if (ret)
		goto err_free_irq;

	/* init hardware */
	ret = adxl362_setup(ac);
	if (ret)
		goto err_remove_attr;

	ret = input_register_device(input_dev);
	if (ret)
		goto err_remove_attr;

	return 0;

err_remove_attr:
	sysfs_remove_group(&dev->kobj, &adxl362_attr_group);
err_free_irq:
	if (spi->irq)
		free_irq(spi->irq, ac);
err_free_mem:
	input_free_device(input_dev);

	return ret;
}

static int adxl362_remove(struct spi_device *spi)
{
	struct adxl362_state *ac = dev_get_drvdata(&spi->dev);

	if (spi->irq)
		free_irq(spi->irq, ac);
	else
		cancel_delayed_work_sync(&ac->work);

	__adxl362_disable(ac);
	sysfs_remove_group(&spi->dev.kobj, &adxl362_attr_group);
	input_unregister_device(ac->input);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int adxl362_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct adxl362_state *ac = dev_get_drvdata(&spi->dev);

	mutex_lock(&ac->mutex);

	if (!ac->suspended && ac->opened)
		__adxl362_disable(ac);

	ac->suspended = true;

	mutex_unlock(&ac->mutex);

	return 0;
}

static int adxl362_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct adxl362_state *ac = dev_get_drvdata(&spi->dev);

	mutex_lock(&ac->mutex);

	if (ac->suspended && ac->opened)
		__adxl362_enable(ac);

	ac->suspended = false;

	mutex_unlock(&ac->mutex);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(adxl362_pm, adxl362_suspend,
			 adxl362_resume);

static struct spi_driver adxl362_driver = {
	.driver = {
		.name = "adxl362",
		.owner = THIS_MODULE,
		.pm = &adxl362_pm,
	},
	.probe   = adxl362_probe,
	.remove  = adxl362_remove,
};

module_spi_driver(adxl362_driver);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("ADXL362 Three-Axis Digital Accelerometer");
MODULE_LICENSE("GPL");
