/*
 * ADXL372 3-Axis Digital Accelerometer SPI driver
 *
 * Copyright 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/bitops.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

/*
 * ADXL372 registers definition
 */
#define ADXL372_DEVID			0x00u
#define ADXL372_DEVID_MST		0x01u
#define ADXL372_PARTID			0x02u
#define ADXL372_REVID			0x03u
#define ADXL372_STATUS_1		0x04u
#define ADXL372_STATUS_2		0x05u
#define ADXL372_FIFO_ENTRIES_2		0x06u
#define ADXL372_FIFO_ENTRIES_1		0x07u
#define ADXL372_X_DATA_H		0x08u
#define ADXL372_X_DATA_L		0x09u
#define ADXL372_Y_DATA_H		0x0Au
#define ADXL372_Y_DATA_L		0x0Bu
#define ADXL372_Z_DATA_H		0x0Cu
#define ADXL372_Z_DATA_L		0x0Du
#define ADXL372_X_MAXPEAK_H		0x15u
#define ADXL372_X_MAXPEAK_L		0x16u
#define ADXL372_Y_MAXPEAK_H		0x17u
#define ADXL372_Y_MAXPEAK_L		0x18u
#define ADXL372_Z_MAXPEAK_H		0x19u
#define ADXL372_Z_MAXPEAK_L		0x1Au
#define ADXL372_OFFSET_X		0x20u
#define ADXL372_OFFSET_Y		0x21u
#define ADXL372_OFFSET_Z		0x22u
#define ADXL372_X_THRESH_ACT_H		0x23u
#define ADXL372_X_THRESH_ACT_L		0x24u
#define ADXL372_Y_THRESH_ACT_H		0x25u
#define ADXL372_Y_THRESH_ACT_L		0x26u
#define ADXL372_Z_THRESH_ACT_H		0x27u
#define ADXL372_Z_THRESH_ACT_L		0x28u
#define ADXL372_TIME_ACT		0x29u
#define ADXL372_X_THRESH_INACT_H	0x2Au
#define ADXL372_X_THRESH_INACT_L	0x2Bu
#define ADXL372_Y_THRESH_INACT_H	0x2Cu
#define ADXL372_Y_THRESH_INACT_L	0x2Du
#define ADXL372_Z_THRESH_INACT_H	0x2Eu
#define ADXL372_Z_THRESH_INACT_L	0x2Fu
#define ADXL372_TIME_INACT_H		0x30u
#define ADXL372_TIME_INACT_L		0x31u
#define ADXL372_X_THRESH_ACT2_H		0x32u
#define ADXL372_X_THRESH_ACT2_L		0x33u
#define ADXL372_Y_THRESH_ACT2_H		0x34u
#define ADXL372_Y_THRESH_ACT2_L		0x35u
#define ADXL372_Z_THRESH_ACT2_H		0x36u
#define ADXL372_Z_THRESH_ACT2_L		0x37u
#define ADXL372_HPF			0x38u
#define ADXL372_FIFO_SAMPLES		0x39u
#define ADXL372_FIFO_CTL		0x3Au
#define ADXL372_INT1_MAP		0x3Bu
#define ADXL372_INT2_MAP		0x3Cu
#define ADXL372_TIMING			0x3Du
#define ADXL372_MEASURE			0x3Eu
#define ADXL372_POWER_CTL		0x3Fu
#define ADXL372_SELF_TEST		0x40u
#define ADXL372_RESET			0x41u
#define ADXL372_FIFO_DATA		0x42u

#define ADXL372_DEVID_VAL		0xADu
#define ADXL372_PARTID_VAL		0xFAu
#define ADXL372_RESET_CODE		0x52u

#define ADXL372_REG_READ(x)		((((x) & 0xFF) << 1) | 0x01)
#define ADXL372_REG_WRITE(x)		(((x) & 0xFF) << 1)

/* ADXL372_POWER_CTL */
#define ADXL372_POWER_CTL_INSTANT_ON_TH_MSK	BIT(5)
#define ADXL372_POWER_CTL_INSTANT_ON_TH_MODE(x)	(((x) & 0x1) << 5)
#define ADXL372_POWER_CTL_FIL_SETTLE_MSK	BIT(4)
#define ADXL372_POWER_CTL_FIL_SETTLE_MODE(x)	(((x) & 0x1) << 4)
#define ADXL372_POWER_CTL_LPF_DIS_MSK		BIT(3)
#define ADXL372_POWER_CTL_LPF_DIS_MODE(x)	(((x) & 0x1) << 3)
#define ADXL372_POWER_CTL_HPF_DIS_MSK		BIT(2)
#define ADXL372_POWER_CTL_HPF_DIS_MODE(x)	(((x) & 0x1) << 2)
#define ADXL372_POWER_CTL_MODE_MSK		GENMASK(1, 0)
#define ADXL372_POWER_CTL_MODE(x)		(((x) & 0x3) << 0)

/* ADXL372_MEASURE */
#define ADXL372_MEASURE_AUTOSLEEP_MSK		BIT(6)
#define ADXL372_MEASURE_AUTOSLEEP_MODE(x)	(((x) & 0x1) << 6)
#define ADXL372_MEASURE_LINKLOOP_MSK		GENMASK(5, 4)
#define ADXL372_MEASURE_LINKLOOP_MODE(x)	(((x) & 0x3) << 4)
#define ADXL372_MEASURE_LOW_NOISE_MSK		BIT(3)
#define ADXL372_MEASURE_LOW_NOISE_MODE(x)	(((x) & 0x1) << 3)
#define ADXL372_MEASURE_BANDWIDTH_MSK		GENMASK(2, 0)
#define ADXL372_MEASURE_BANDWIDTH_MODE(x)	(((x) & 0x7) << 0)

/* ADXL372_TIMING */
#define ADXL372_TIMING_ODR_MSK			GENMASK(7, 5)
#define ADXL372_TIMING_ODR_MODE(x)		(((x) & 0x7) << 5)
#define ADXL372_TIMING_WAKE_UP_RATE_MSK		GENMASK(4, 2)
#define ADXL372_TIMING_WAKE_UP_RATE_MODE(x)	(((x) & 0x7) << 2)
#define ADXL372_TIMING_EXT_CLK_MSK		BIT(1)
#define ADXL372_TIMING_EXT_CLK_MODE(x)		(((x) & 0x1) << 1)
#define ADXL372_TIMING_EXT_SYNC_MSK		BIT(0)
#define ADXL372_TIMING_EXT_SYNC_MODE(x)		(((x) & 0x1) << 0)

/* ADXL372_FIFO_CTL */
#define ADXL372_FIFO_CTL_FORMAT_MSK		GENMASK(5, 3)
#define ADXL372_FIFO_CTL_FORMAT_MODE(x)		(((x) & 0x7) << 3)
#define ADXL372_FIFO_CTL_MODE_MSK		GENMASK(2, 1)
#define ADXL372_FIFO_CTL_MODE_MODE(x)		(((x) & 0x3) << 1)
#define ADXL372_FIFO_CTL_SAMPLES_MSK		BIT(1)
#define ADXL372_FIFO_CTL_SAMPLES_MODE(x)	(((x) > 0xFF) ? 1 : 0)

/* ADXL372_STATUS_1 */
#define ADXL372_STATUS_1_DATA_RDY(x)		(((x) >> 0) & 0x1)
#define ADXL372_STATUS_1_FIFO_RDY(x)		(((x) >> 1) & 0x1)
#define ADXL372_STATUS_1_FIFO_FULL(x)		(((x) >> 2) & 0x1)
#define ADXL372_STATUS_1_FIFO_OVR(x)		(((x) >> 3) & 0x1)
#define ADXL372_STATUS_1_USR_NVM_BUSY(x)	(((x) >> 5) & 0x1)
#define ADXL372_STATUS_1_AWAKE(x)		(((x) >> 6) & 0x1)
#define ADXL372_STATUS_1_ERR_USR_REGS(x)	(((x) >> 7) & 0x1)

/* ADXL372_INT1_MAP */
#define ADXL372_INT1_MAP_DATA_RDY_MSK		BIT(0)
#define ADXL372_INT1_MAP_DATA_RDY_MODE(x)	(((x) & 0x1) << 0)
#define ADXL372_INT1_MAP_FIFO_RDY_MSK		BIT(1)
#define ADXL372_INT1_MAP_FIFO_RDY_MODE(x)	(((x) & 0x1) << 1)
#define ADXL372_INT1_MAP_FIFO_FULL_MSK		BIT(2)
#define ADXL372_INT1_MAP_FIFO_FULL_MODE(x)	(((x) & 0x1) << 2)
#define ADXL372_INT1_MAP_FIFO_OVR_MSK		BIT(3)
#define ADXL372_INT1_MAP_FIFO_OVR_MODE(x)	(((x) & 0x1) << 3)
#define ADXL372_INT1_MAP_INACT_MSK		BIT(4)
#define ADXL372_INT1_MAP_INACT_MODE(x)		(((x) & 0x1) << 4)
#define ADXL372_INT1_MAP_ACT_MSK		BIT(5)
#define ADXL372_INT1_MAP_ACT_MODE(x)		(((x) & 0x1) << 5)
#define ADXL372_INT1_MAP_AWAKE_MSK		BIT(6)
#define ADXL372_INT1_MAP_AWAKE_MODE(x)		(((x) & 0x1) << 6)
#define ADXL372_INT1_MAP_LOW_MSK		BIT(7)
#define ADXL372_INT1_MAP_LOW_MODE(x)		(((x) & 0x1) << 7)

/* ADXL372_INT2_MAP */
#define ADXL372_INT2_MAP_DATA_RDY_MSK		BIT(0)
#define ADXL372_INT2_MAP_DATA_RDY_MODE(x)	(((x) & 0x1) << 0)
#define ADXL372_INT2_MAP_FIFO_RDY_MSK		BIT(1)
#define ADXL372_INT2_MAP_FIFO_RDY_MODE(x)	(((x) & 0x1) << 1)
#define ADXL372_INT2_MAP_FIFO_FULL_MSK		BIT(2)
#define ADXL372_INT2_MAP_FIFO_FULL_MODE(x)	(((x) & 0x1) << 2)
#define ADXL372_INT2_MAP_FIFO_OVR_MSK		BIT(3)
#define ADXL372_INT2_MAP_FIFO_OVR_MODE(x)	(((x) & 0x1) << 3)
#define ADXL372_INT2_MAP_INACT_MSK		BIT(4)
#define ADXL372_INT2_MAP_INACT_MODE(x)		(((x) & 0x1) << 4)
#define ADXL372_INT2_MAP_ACT_MSK		BIT(5)
#define ADXL372_INT2_MAP_ACT_MODE(x)		(((x) & 0x1) << 5)
#define ADXL372_INT2_MAP_AWAKE_MSK		BIT(6)
#define ADXL372_INT2_MAP_AWAKE_MODE(x)		(((x) & 0x1) << 6)
#define ADXL372_INT2_MAP_LOW_MSK		BIT(7)
#define ADXL372_INT2_MAP_LOW_MODE(x)		(((x) & 0x1) << 7)

/*
 * At +/- 200g with 12-bit resolution, scale is computed as:
 * (200 + 200) * 9.81 / (2^12 - 1) = 0.958241
 */
#define ADXL372_USCALE	958241;

enum adxl372_axis {
	AXIS_X,
	AXIS_Y,
	AXIS_Z,
};

enum adxl372_op_mode {
	ADXL372_STANDBY,
	ADXL372_WAKE_UP,
	ADXL372_INSTANT_ON,
	ADXL372_FULL_BW_MEASUREMENT,
};

enum adxl372_act_proc_mode {
	ADXL372_DEFAULT,
	ADXL372_LINKED,
	ADXL372_LOOPED,
};

enum adxl372_th_activity {
	ADXL372_ACTIVITY,
	ADXL372_ACTIVITY2,
	ADXL372_INACTIVITY,
};

enum adxl372_threshold_en {
	ADXL372_THRESHOLD_DIS,
	ADXL372_THRESHOLD_EN,
};

enum adxl372_threshold_ref_en {
	ADXL372_THRESHOLD_REF_DIS,
	ADXL372_THRESHOLD_REF_EN,
};

enum adxl372_odr {
	ADXL372_ODR_400HZ,
	ADXL372_ODR_800HZ,
	ADXL372_ODR_1600HZ,
	ADXL372_ODR_3200HZ,
	ADXL372_ODR_6400HZ,
};

enum adxl372_bandwidth {
	ADXL372_BW_200HZ,
	ADXL372_BW_400HZ,
	ADXL372_BW_800HZ,
	ADXL372_BW_1600HZ,
	ADXL372_BW_3200HZ,
};

#define ADXL372_ACCEL_CHANNEL(index, reg, axis) {			\
	.type = IIO_ACCEL,						\
	.address = reg,							\
	.modified = 1,							\
	.channel2 = IIO_MOD_##axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),		\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = index,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 12,						\
		.storagebits = 16,					\
		.shift = 4,						\
		.endianness = IIO_CPU,					\
	},								\
}

static const struct iio_chan_spec adxl372_channels[] = {
	ADXL372_ACCEL_CHANNEL(0, ADXL372_X_DATA_H, X),
	ADXL372_ACCEL_CHANNEL(1, ADXL372_Y_DATA_H, Y),
	ADXL372_ACCEL_CHANNEL(2, ADXL372_Z_DATA_H, Z),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

struct adxl372_state {
	struct spi_device		*spi;
	struct regmap			*regmap;
	enum adxl372_op_mode		op_mode;
	enum adxl372_act_proc_mode	act_proc_mode;
	enum adxl372_odr		odr;
	enum adxl372_bandwidth		bw;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	union {
		__le16 regval;
		u8 d8[512];
	} data ____cacheline_aligned;
};

static int adxl372_spi_write_mask(struct adxl372_state *st,
				  u8 reg_addr,
				  unsigned int mask,
				  u8 data)
{
	int ret;

	ret = regmap_read(st->regmap, ADXL372_REG_READ(reg_addr),
			  (unsigned int *)&st->data.regval);
	if (ret < 0)
		return ret;

	st->data.regval &= ~mask;
	st->data.regval |= data;

	ret = regmap_write(st->regmap, ADXL372_REG_WRITE(reg_addr),
			   st->data.regval);

	return ret;
}

static int adxl372_read_axis(struct adxl372_state *st, u8 addr)
{
	int ret;

	ret = regmap_bulk_read(st->regmap, ADXL372_REG_READ(addr),
			       &st->data.regval, 2);
	if (ret < 0)
		return ret;

	return be16_to_cpu(st->data.regval);
}

static int adxl372_set_op_mode(struct adxl372_state *st,
			       enum adxl372_op_mode op_mode)
{
	int ret;

	ret = adxl372_spi_write_mask(st,
				     ADXL372_POWER_CTL,
				     ADXL372_POWER_CTL_MODE_MSK,
				     ADXL372_POWER_CTL_MODE(op_mode));

	if (ret < 0) {
		dev_err(&st->spi->dev, "Error writing mode of operation\n");
		return ret;
	}

	st->op_mode = op_mode;

	return ret;
}

static int adxl372_set_odr(struct adxl372_state *st,
			   enum adxl372_odr odr)
{
	int ret;

	ret = adxl372_spi_write_mask(st,
				     ADXL372_TIMING,
				     ADXL372_TIMING_ODR_MSK,
				     ADXL372_TIMING_ODR_MODE(odr));

	if (ret < 0) {
		dev_err(&st->spi->dev, "Error setting output data rate\n");
		return ret;
	}

	st->odr = odr;

	return ret;
}

static int adxl372_set_bandwidth(struct adxl372_state *st,
				 enum adxl372_bandwidth bw)
{
	int ret;

	ret = adxl372_spi_write_mask(st,
				     ADXL372_MEASURE,
				     ADXL372_MEASURE_BANDWIDTH_MSK,
				     ADXL372_MEASURE_BANDWIDTH_MODE(bw));

	if (ret < 0) {
		dev_err(&st->spi->dev, "Error setting bandwidth\n");
		return ret;
	}

	st->bw = bw;

	return ret;
}

static int adxl372_set_act_proc_mode(struct adxl372_state *st,
				     enum adxl372_act_proc_mode mode)
{
	int ret;

	ret = adxl372_spi_write_mask(st,
				     ADXL372_MEASURE,
				     ADXL372_MEASURE_LINKLOOP_MSK,
				     ADXL372_MEASURE_LINKLOOP_MODE(mode));

	if (ret < 0) {
		dev_err(&st->spi->dev,
			"Error writing activity processing mode\n");
		return ret;
	}

	st->act_proc_mode = mode;

	return ret;
}

static int adxl372_set_activity_threshold(struct adxl372_state *st,
		enum adxl372_th_activity act,
		enum adxl372_threshold_ref_en ref,
		enum adxl372_threshold_en enable,
		u32 threshold)

{
	u8 th_reg_high_val, th_reg_low_val, th_reg_high_addr;
	int ret;

	/* scale factor is 100 mg/code */
	th_reg_high_val = (threshold / 100) >> 3;
	th_reg_low_val = ((threshold / 100) << 5) | (ref << 1) | enable;

	switch (act) {
	case ADXL372_ACTIVITY:
		th_reg_high_addr = ADXL372_X_THRESH_ACT_H;
		break;
	case ADXL372_ACTIVITY2:
		th_reg_high_addr = ADXL372_X_THRESH_ACT2_H;
		break;
	case ADXL372_INACTIVITY:
		th_reg_high_addr = ADXL372_X_THRESH_INACT_H;
		break;
	}

	st->data.d8[0] = th_reg_high_val;
	st->data.d8[1] = th_reg_low_val;
	st->data.d8[2] = th_reg_high_val;
	st->data.d8[3] = th_reg_low_val;
	st->data.d8[4] = th_reg_high_val;
	st->data.d8[5] = th_reg_low_val;

	ret = regmap_bulk_write(st->regmap,
				ADXL372_REG_WRITE(th_reg_high_addr),
				&st->data.d8[0], 6);
	if (ret < 0)
		dev_err(&st->spi->dev, "Error writing activity threshold\n");

	return ret;
}

static int adxl372_setup(struct adxl372_state *st)
{
	int ret;

	ret = regmap_read(st->regmap, ADXL372_REG_READ(ADXL372_DEVID),
			  (unsigned int *)&st->data.regval);
	if (ret < 0)
		return ret;

	if (st->data.regval != ADXL372_DEVID_VAL) {
		dev_err(&st->spi->dev, "Invalid chip id %x\n",
			st->data.regval);
		return -ENODEV;
	}

	ret = adxl372_set_op_mode(st, ADXL372_STANDBY);
	if (ret < 0)
		return ret;

	/* Set threshold for activity detection to 500mg */
	ret = adxl372_set_activity_threshold(st, ADXL372_ACTIVITY,
					     ADXL372_THRESHOLD_REF_EN,
					     ADXL372_THRESHOLD_EN,
					     500);
	if (ret < 0)
		return ret;

	/* Set threshold for inactivity detection to 500mg */
	ret = adxl372_set_activity_threshold(st, ADXL372_INACTIVITY,
					     ADXL372_THRESHOLD_REF_EN,
					     ADXL372_THRESHOLD_EN,
					     500);
	if (ret < 0)
		return ret;

	/* Set activity processing in Looped mode */
	ret = adxl372_set_act_proc_mode(st, ADXL372_LOOPED);
	if (ret < 0)
		return ret;

	ret = adxl372_set_odr(st, ADXL372_ODR_6400HZ);
	if (ret < 0)
		return ret;

	ret = adxl372_set_bandwidth(st, ADXL372_BW_3200HZ);
	if (ret < 0)
		return ret;

	/* Set activity timer */
	ret = regmap_write(st->regmap,
			   ADXL372_REG_WRITE(ADXL372_TIME_ACT), 1);
	if (ret < 0)
		return ret;

	/* Set inactivity timer to 1s*/
	ret = regmap_write(st->regmap,
			   ADXL372_REG_WRITE(ADXL372_TIME_INACT_L), 0x28);
	if (ret < 0)
		return ret;

	/* Set the mode of operation to full bandwidth measurement mode */
	ret = adxl372_set_op_mode(st, ADXL372_FULL_BW_MEASUREMENT);
	if (ret < 0)
		return ret;

	return ret;
}

static int adxl372_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int writeval,
			      unsigned int *readval)
{
	struct adxl372_state *st = iio_priv(indio_dev);
	int ret;

	if (readval == NULL) {
		ret = regmap_write(st->regmap, ADXL372_REG_WRITE(reg),
				   writeval);
	} else {
		ret = regmap_read(st->regmap, ADXL372_REG_READ(reg),
				  readval);
	}

	return ret;
}

static int adxl372_raw(struct iio_dev *indio_dev,
		       struct iio_chan_spec const *chan,
		       int *val, int *val2, long info)
{
	struct adxl372_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = adxl372_read_axis(st, chan->address);

		if (ret < 0)
			return ret;

		*val = sign_extend32(ret >> chan->scan_type.shift,
				     chan->scan_type.realbits - 1);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = ADXL372_USCALE;

		return IIO_VAL_INT_PLUS_MICRO;
	}

	return -EINVAL;
}

static const struct iio_info adxl372_info = {
	.driver_module	= THIS_MODULE,
	.read_raw	= adxl372_raw,
	.debugfs_reg_access = &adxl372_reg_access,
};

static const struct regmap_config adxl372_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.read_flag_mask = BIT(0),
};

static int adxl372_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adxl372_state *st;
	struct regmap *regmap;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;

	regmap = devm_regmap_init_spi(spi, &adxl372_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Error initializing spi regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}
	st->regmap = regmap;

	indio_dev->channels = adxl372_channels;
	indio_dev->num_channels = ARRAY_SIZE(adxl372_channels);
	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &adxl372_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = adxl372_setup(st);
	if (ret < 0) {
		dev_err(&st->spi->dev, "ADXL372 setup failed\n");
		return ret;
	}

	ret = iio_device_register(indio_dev);
	if (ret)
		return ret;

	return 0;
}

static int adxl372_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	iio_device_unregister(indio_dev);

	return 0;
}

static const struct spi_device_id adxl372_id[] = {
	{ "adxl372", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, adxl372_id);

static struct spi_driver adxl372_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.probe = adxl372_probe,
	.remove = adxl372_remove,
	.id_table = adxl372_id,
};

module_spi_driver(adxl372_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADXL372 3-axis accelerometer driver");
MODULE_LICENSE("GPL v2");
