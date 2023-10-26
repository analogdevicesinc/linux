#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>

#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define MAX30210_PIN_CONF_REG	0x12
#define MAX30210_TEMP_CONV_REG	0x2A
#define MAX30210_FIFO_DATA_REG	0x08
#define MAX30210_TEMP_CONF_2_REG	0x29
#define MAX30210_STATUS_REG	0x00
#define MAX30210_TEMP_DATA_REG	0x2B
#define MAX30210_FIFO_CONF_1_REG	0x09
#define MAX30210_FIFO_CONF_2_REG	0x0A
#define MAX30210_INT_EN_REG	0x02
#define MAX30210_ALM_HI_REG	0x22
#define MAX30210_ALM_LO_REG	0x24

#define MAX30210_CONV_T_MASK	BIT(0)
#define MAX30210_TEMP_FREQ_MASK	GENMASK(3, 0)
#define MAX30210_ALERT_MODE	BIT(7)
#define MAX30210_AUTO_CONV_MASK	BIT(1)
#define MAX30210_EXT_CNV_EN_MASK	BIT(7)
#define MAX30210_EXT_CVT_ICFG_MASK	BIT(6)
#define MAX30210_FIFO_A_FULL_MASK	GENMASK(5, 0)
#define MAX30210_FLUSH_FIFO_MASK	BIT(4)
#define MAX30210_PWR_RDY_MASK	BIT(0)
#define MAX30210_TEMP_HI_MASK	BIT(2)
#define MAX30210_TEMP_LO_MASK	BIT(3)
#define MAX30210_TEMP_INC_MASK	BIT(4)
#define MAX30210_TEMP_DEC_MASK	BIT(5)
#define MAX30210_A_FULL_MASK	BIT(7)


#define MAX30210_FIFO_SIZE	64
#define MAX30210_FIFO_INVAL_DATA	GENMASK(23, 0)
#define MAX30210_WATERMARK_DEFAULT	(0x40 - 0x1F)

#define MAX30210_CNV_RATE_DIV(x)	BIT(9 - (((x) > 9) ? 9 : (x)))
#define MAX30210_CNV_RATE_TO_REG(wp, fp)	fls(((wp) * 1000000 + (fp)) / 15625 - 1)
#define MAX30210_INT_EN(state, mask)	((state) ? (mask) : 0x0)

struct max30210_state {
	/*
	 * Prevent simultaneous access to the i2c client.
	 */
	struct mutex lock;
	struct regmap *regmap;
	struct iio_trigger *cvt_pdb_trig;
	unsigned int watermark;
	u8 ext_cvt_en : 1;
};

static const struct regmap_config max30210_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

static int max30210_read_temp(struct regmap *regmap,
			      unsigned int reg, int *temp)
{
	__be16 uval;
	int ret;

	ret = regmap_bulk_read(regmap, reg, (u8 *)&uval, 2);
	if (ret)
		return ret;

	*temp = sign_extend32(be16_to_cpu(uval), 15);

	return IIO_VAL_INT;
}

static int max30210_write_temp(struct regmap *regmap,
			       unsigned int reg, int temp)
{
	const __be16 uval = cpu_to_be16(temp);

	return regmap_bulk_write(regmap, reg, (u8 *)&uval, 2);
}

static void max30210_fifo_read(struct iio_dev *indio_dev)
{
	struct max30210_state *st = iio_priv(indio_dev);
	u8 data[3 * MAX30210_FIFO_SIZE];
	__be32 samp;
	int ret, i, j;

	ret = regmap_bulk_read(st->regmap, MAX30210_FIFO_DATA_REG, 
			       data, 3 * st->watermark);
	if (ret < 0) {
		dev_err(&indio_dev->dev, "Failed to read from fifo.\n");
		return;
	}

	for (i = 0; i < st->watermark; i++) {
		samp = 0;
		for (j = 0; j < 3; j++) {
			samp <<= 8;
			samp |= data[3*i + j];
		}
		if (samp == MAX30210_FIFO_INVAL_DATA) {
			dev_err(&indio_dev->dev, "Invalid data\n");
			continue;
		}
		iio_push_to_buffers(indio_dev, &samp);
	}
}

static irqreturn_t max30210_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct max30210_state *st = iio_priv(indio_dev);
	unsigned int status;
	int ret;

	mutex_lock(&st->lock);
	usleep_range(640, 700);
	ret = regmap_read(st->regmap, MAX30210_STATUS_REG, &status);
	if (ret) {
		dev_err(&indio_dev->dev, "Status byte read error\n");
		goto exit_irq;
	}

	if (status & MAX30210_PWR_RDY_MASK) {
		dev_info(&indio_dev->dev, "power-on\n");
		st->watermark = MAX30210_WATERMARK_DEFAULT;
	}

	if (status & MAX30210_A_FULL_MASK)
		max30210_fifo_read(indio_dev);

	if (status & MAX30210_TEMP_HI_MASK)
		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_TEMP, 0,
						  IIO_MOD_TEMP_AMBIENT,
						  IIO_EV_TYPE_THRESH,
						  IIO_EV_DIR_RISING),
			       iio_get_time_ns(indio_dev));

	if (status & MAX30210_TEMP_LO_MASK)
		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_TEMP, 0,
						  IIO_MOD_TEMP_AMBIENT,
						  IIO_EV_TYPE_THRESH,
						  IIO_EV_DIR_FALLING),
			       iio_get_time_ns(indio_dev));

	if (status & MAX30210_TEMP_INC_MASK)
		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_TEMP, 0,
						  IIO_MOD_TEMP_AMBIENT,
						  IIO_EV_TYPE_CHANGE,
						  IIO_EV_DIR_RISING),
			       iio_get_time_ns(indio_dev));

	if (status & MAX30210_TEMP_DEC_MASK)
		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_TEMP, 0,
						  IIO_MOD_TEMP_AMBIENT,
						  IIO_EV_TYPE_CHANGE,
						  IIO_EV_DIR_FALLING),
			       iio_get_time_ns(indio_dev));

exit_irq:
	mutex_unlock(&st->lock);
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int max30210_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			      unsigned int writeval, unsigned int *readval)
{
	struct max30210_state *st = iio_priv(indio_dev);

	if (!readval)
		return regmap_write(st->regmap, reg, writeval);

	return regmap_read(st->regmap, reg, readval);
}

static int max30210_validate_trigger(struct iio_dev *indio_dev,
				     struct iio_trigger *trig)
{
	struct max30210_state *st = iio_priv(indio_dev);

	if (st->cvt_pdb_trig != trig)
		return -EINVAL;

	return 0;
}

static int max30210_read_event(struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan,
			       enum iio_event_type type,
			       enum iio_event_direction dir,
			       enum iio_event_info info, int *val, int *val2)
{
	struct max30210_state *st = iio_priv(indio_dev);

	if (info == IIO_EV_INFO_VALUE) {
		switch(dir) {
		case IIO_EV_DIR_RISING:
			switch (type) {
			case IIO_EV_TYPE_THRESH:
				return max30210_read_temp(st->regmap,
							  MAX30210_ALM_HI_REG,
							  val);
			// case IIO_EV_TYPE_CHANGE:
			// 	return 0;
			default:
				return -EINVAL;
			}
			break;
		case IIO_EV_DIR_FALLING:
			switch (type) {
			case IIO_EV_TYPE_THRESH:
				return max30210_read_temp(st->regmap,
							  MAX30210_ALM_LO_REG,
							  val);
			// case IIO_EV_TYPE_CHANGE:
			// 	return 0;
			default:
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		}
	}

	return -EINVAL;
}

static int max30210_write_event(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				enum iio_event_type type,
				enum iio_event_direction dir,
				enum iio_event_info info,
				int val, int val2)
{
	struct max30210_state *st = iio_priv(indio_dev);

	if (info == IIO_EV_INFO_VALUE) {
		switch(dir) {
		case IIO_EV_DIR_RISING:
			switch (type) {
			case IIO_EV_TYPE_THRESH:
				return max30210_write_temp(st->regmap,
							   MAX30210_ALM_HI_REG,
							   val);
			// case IIO_EV_TYPE_CHANGE:
			// 	return 0;
			default:
				return -EINVAL;
			}
			break;
		case IIO_EV_DIR_FALLING:
			switch (type) {
			case IIO_EV_TYPE_THRESH:
				return max30210_write_temp(st->regmap,
							   MAX30210_ALM_LO_REG,
							   val);
			// case IIO_EV_TYPE_CHANGE:
			// 	return 0;
			default:
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		}
	}

	return -EINVAL;
}

static int max30210_read_event_config(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       enum iio_event_type type,
				       enum iio_event_direction dir)
{
	struct max30210_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	ret = regmap_read(st->regmap, MAX30210_INT_EN_REG, &val);
	if (ret)
		return ret;

	switch(dir) {
	case IIO_EV_DIR_RISING:
		switch (type) {
		case IIO_EV_TYPE_THRESH:
			ret = FIELD_GET(MAX30210_TEMP_HI_MASK, val);
			break;
		// case IIO_EV_TYPE_CHANGE:
		// 	ret = FIELD_GET(MAX30210_TEMP_INC_MASK, val);
			break;
		default:
			return -EINVAL;
		}
		break;
	case IIO_EV_DIR_FALLING:
		switch (type) {
		case IIO_EV_TYPE_THRESH:
			ret = FIELD_GET(MAX30210_TEMP_LO_MASK, val);
			break;
		// case IIO_EV_TYPE_CHANGE:
		// 	ret = FIELD_GET(MAX30210_TEMP_DEC_MASK, val);
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int max30210_write_event_config(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       enum iio_event_type type,
				       enum iio_event_direction dir,
				       int state)
{
	struct max30210_state *st = iio_priv(indio_dev);
	unsigned int val;

	switch(dir) {
	case IIO_EV_DIR_RISING:
		switch (type) {
		case IIO_EV_TYPE_THRESH:
			val = MAX30210_INT_EN(state, MAX30210_TEMP_HI_MASK);

			return regmap_update_bits(st->regmap,
						  MAX30210_INT_EN_REG,
						  MAX30210_TEMP_HI_MASK,
						  val);
		// case IIO_EV_TYPE_CHANGE:
		// 	val = MAX30210_INT_EN(state, MAX30210_TEMP_INC_MASK);

		// 	return regmap_update_bits(st->regmap,
		// 				  MAX30210_INT_EN_REG,
		// 				  MAX30210_TEMP_INC_MASK,
		// 				  val);
		default:
			return -EINVAL;
		}
		break;
	case IIO_EV_DIR_FALLING:
		switch (type) {
		case IIO_EV_TYPE_THRESH:
			val = MAX30210_INT_EN(state, MAX30210_TEMP_LO_MASK);

			return regmap_update_bits(st->regmap,
						  MAX30210_INT_EN_REG,
						  MAX30210_TEMP_LO_MASK,
						  val);
		// case IIO_EV_TYPE_CHANGE:
		// 	val = MAX30210_INT_EN(state, MAX30210_TEMP_DEC_MASK);

		// 	return regmap_update_bits(st->regmap,
		// 				  MAX30210_INT_EN_REG,
		// 				  MAX30210_TEMP_DEC_MASK,
		// 				  val);
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int max30210_read_raw(struct iio_dev *indio_dev, 
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct max30210_state *st = iio_priv(indio_dev);
	unsigned int uval;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = 5000;

		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = regmap_read(st->regmap, MAX30210_TEMP_CONF_2_REG, &uval);
		if (ret)
			return ret;

		uval = FIELD_GET(MAX30210_TEMP_FREQ_MASK, uval);

		*val = 8;
		*val2 = MAX30210_CNV_RATE_DIV(uval);

		return IIO_VAL_FRACTIONAL;
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			goto err_dmode;

		mutex_lock(&st->lock);

		ret = regmap_write(st->regmap, MAX30210_TEMP_CONV_REG,
				   MAX30210_CONV_T_MASK);
		if (ret)
			goto err_lock;

		usleep_range(8000, 8300);
		ret = max30210_read_temp(st->regmap, MAX30210_TEMP_DATA_REG,
					 val);

		mutex_unlock(&st->lock);
		iio_device_release_direct_mode(indio_dev);

		return ret;
	default:
		return -EINVAL;
	}

	return 0;
err_lock:
	mutex_unlock(&st->lock);
err_dmode:
	iio_device_release_direct_mode(indio_dev);
	return ret;
}

static int max30210_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	struct max30210_state *st = iio_priv(indio_dev);
	unsigned int data;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		data = MAX30210_CNV_RATE_TO_REG(val, val2);
		data = FIELD_PREP(MAX30210_TEMP_FREQ_MASK, data);

		ret = regmap_update_bits(st->regmap, MAX30210_TEMP_CONF_2_REG,
					 MAX30210_TEMP_FREQ_MASK, data);
		if (ret)
			return ret;

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int max30210_write_raw_get_fmt(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static const struct iio_trigger_ops max30210_trigger_ops = {
	.validate_device = &iio_trigger_validate_own_device,
};

static int max30210_set_watermark(struct iio_dev *indio_dev, unsigned int val)
{
	struct max30210_state *st = iio_priv(indio_dev);
	unsigned int reg;
	int ret;

	if (val < 1 || val > MAX30210_FIFO_SIZE)
		return -EINVAL;

	reg = FIELD_PREP(MAX30210_FIFO_A_FULL_MASK,
			 MAX30210_FIFO_SIZE - val);
	
	ret = regmap_write(st->regmap, MAX30210_FIFO_CONF_1_REG, reg);
	if (ret)
		return ret;

	st->watermark = val;

	return 0;
}

static ssize_t hwfifo_watermark_show(struct device *dev,
				     struct device_attribute *devattr,
				     char *buf)
{
	struct max30210_state *st = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", st->watermark);
}

IIO_STATIC_CONST_DEVICE_ATTR(hwfifo_watermark_min, "1");
IIO_STATIC_CONST_DEVICE_ATTR(hwfifo_watermark_max,
			     __stringify(MAX30210_FIFO_SIZE));
static IIO_DEVICE_ATTR_RO(hwfifo_watermark, 0);

static const struct iio_dev_attr *max30210_fifo_attributes[] = {
	&iio_dev_attr_hwfifo_watermark_min,
	&iio_dev_attr_hwfifo_watermark_max,
	&iio_dev_attr_hwfifo_watermark,
	NULL,
};

static int max30210_buffer_preenable(struct iio_dev *indio_dev)
{
	struct max30210_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	ret = regmap_update_bits(st->regmap, MAX30210_INT_EN_REG,
				 MAX30210_A_FULL_MASK, MAX30210_A_FULL_MASK);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, MAX30210_FIFO_CONF_2_REG,
				 MAX30210_FLUSH_FIFO_MASK,
				 MAX30210_FLUSH_FIFO_MASK);
	if (ret)
		return ret;

	if (st->ext_cvt_en) {
		ret = regmap_update_bits(st->regmap, MAX30210_PIN_CONF_REG,
					 MAX30210_EXT_CNV_EN_MASK,
					 MAX30210_EXT_CNV_EN_MASK);
		if (ret)
			return ret;
	} else {
		val = MAX30210_AUTO_CONV_MASK | MAX30210_CONV_T_MASK;
		ret = regmap_write(st->regmap, MAX30210_TEMP_CONV_REG, val);
		if (ret)
			return ret;
	}

	return 0;
}

static int max30210_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct max30210_state *st = iio_priv(indio_dev);
	int ret;
	
	ret = regmap_update_bits(st->regmap, MAX30210_INT_EN_REG,
				 MAX30210_A_FULL_MASK, 0x0);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, MAX30210_FIFO_CONF_2_REG,
				 MAX30210_FLUSH_FIFO_MASK,
				 MAX30210_FLUSH_FIFO_MASK);
	if (ret)
		return ret;

	if (st->ext_cvt_en)
		return regmap_update_bits(st->regmap, MAX30210_PIN_CONF_REG,
					 MAX30210_EXT_CNV_EN_MASK, 0x0);

	return regmap_write(st->regmap, MAX30210_TEMP_CONV_REG, 0x0);
}

static const struct iio_buffer_setup_ops max30210_buffer_ops = {
	.preenable = max30210_buffer_preenable,
	.postdisable = max30210_buffer_postdisable,
};

static const struct iio_info max30210_info = {
	.read_raw = max30210_read_raw,
	.write_raw = max30210_write_raw,
	.write_raw_get_fmt = max30210_write_raw_get_fmt,
	.hwfifo_set_watermark = max30210_set_watermark,
	.debugfs_reg_access = &max30210_reg_access,
	.validate_trigger = &max30210_validate_trigger,
	.read_event_value = max30210_read_event,
	.write_event_value = max30210_write_event,
	.write_event_config = max30210_write_event_config,
	.read_event_config = max30210_read_event_config,
};

static const struct iio_event_spec max31827_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				 BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				 BIT(IIO_EV_INFO_ENABLE),
	// }, {
	// 	.type = IIO_EV_TYPE_CHANGE,
	// 	.dir = IIO_EV_DIR_RISING,
	// 	.mask_separate = BIT(IIO_EV_INFO_VALUE) |
	// 			 BIT(IIO_EV_INFO_ENABLE),
	// }, {
	// 	.type = IIO_EV_TYPE_CHANGE,
	// 	.dir = IIO_EV_DIR_FALLING,
	// 	.mask_separate = BIT(IIO_EV_INFO_VALUE) |
	// 			 BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_chan_spec max30210_channels = {
	.type = IIO_TEMP,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			      BIT(IIO_CHAN_INFO_SCALE) |
			      BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.output = 0,
	.scan_index = 0,
	.event_spec = max31827_events,
	.num_event_specs = ARRAY_SIZE(max31827_events),
	.scan_type = {
		.sign = 's',
		.realbits = 16,
		.storagebits = 32,
		.shift = 8,
		.endianness = IIO_BE,
	},
};

static const unsigned long max30210_scan_masks[] = {
	BIT(0),
	0
};

static int max30210_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *fwnode;
	struct iio_dev *indio_dev;
	struct max30210_state *st;
	unsigned int val, dir;
	bool prop;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_I2C_BLOCK))
		return -EOPNOTSUPP;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	mutex_init(&st->lock);

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to enable vdd regulator\n");

	st->regmap = devm_regmap_init_i2c(client, &max30210_regmap);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to allocate regmap.\n");

	indio_dev->name = "max30210";
	indio_dev->info = &max30210_info;
	indio_dev->channels = &max30210_channels;
	indio_dev->num_channels = 1;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->available_scan_masks = max30210_scan_masks;

	fwnode = dev_fwnode(dev);

	prop = fwnode_property_read_bool(fwnode, "adi,external-convert-enable");
	st->ext_cvt_en = prop;
	
	prop = fwnode_property_read_bool(fwnode, "adi,ext-conv-rising-edge");
	dir = (prop) ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
	val = FIELD_PREP(MAX30210_EXT_CVT_ICFG_MASK, prop);

	ret = regmap_update_bits(st->regmap, MAX30210_PIN_CONF_REG,
				 MAX30210_EXT_CVT_ICFG_MASK, val);
	if (ret)
		return ret;

	prop = fwnode_property_read_bool(fwnode, "adi,comp-int");
	val = FIELD_PREP(MAX30210_ALERT_MODE, prop);

	ret = regmap_update_bits(st->regmap, MAX30210_TEMP_CONF_2_REG,
				 MAX30210_ALERT_MODE, val);
	if (ret)
		return ret;

	/*
	 * Clear status byte
	 */
	ret = regmap_read(st->regmap, MAX30210_STATUS_REG, &val);
	if (ret)
		return ret;

	ret = devm_iio_triggered_buffer_setup_ext(dev, indio_dev, NULL,
						  max30210_trigger_handler,
						  IIO_BUFFER_DIRECTION_IN,
						  &max30210_buffer_ops,
						  max30210_fifo_attributes);
	if (ret)
		return ret;

	if (client->irq) {
		st->cvt_pdb_trig = devm_iio_trigger_alloc(dev, "%s-trig",
							  indio_dev->name);
		if (!st->cvt_pdb_trig)
			return -ENOMEM;

		st->cvt_pdb_trig->ops = &max30210_trigger_ops;
		iio_trigger_set_drvdata(st->cvt_pdb_trig, indio_dev);
		ret = devm_iio_trigger_register(dev, st->cvt_pdb_trig);
		if (ret)
			return ret;

		indio_dev->trig = iio_trigger_get(st->cvt_pdb_trig);

		ret = devm_request_irq(dev, client->irq,
				       iio_trigger_generic_data_rdy_poll,
				       dir, indio_dev->name, st->cvt_pdb_trig);
		if (ret)
			return ret;
	}

	return devm_iio_device_register(dev, indio_dev);
}

static const struct i2c_device_id max30210_id[] = {
	{ "max30210", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max30210_id);

static const struct of_device_id max30210_of_match[] = {
	{ .compatible = "adi,max30210" },
	{ }
};
MODULE_DEVICE_TABLE(of, max30210_of_match);


static struct i2c_driver max30210_driver = {
	.driver = {
		.name = "max30210",
		.of_match_table = max30210_of_match,
	},
	.probe_new = max30210_probe,
	.id_table = max30210_id,
};
module_i2c_driver(max30210_driver);

MODULE_AUTHOR("Daniel Matyas <daniel.matyas@analog.com>");
MODULE_DESCRIPTION("MAX30210 low-power digital temperature sensor");
MODULE_LICENSE("GPL");

