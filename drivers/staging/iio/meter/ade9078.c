// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADE9078 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <asm/unaligned.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <linux/spi/spi.h>
#include <linux/regmap.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/sysfs.h>

#include <linux/module.h>
#include <linux/moduleparam.h>
#include "ade9078.h"

/*
 * struct ade9078_device - ade9078 specific data
 * @spi: 		spi device associated to the ade9078
 * @tx			transmit buffer for the spi
 * @rx			receive buffer for the spi
 * @lock		mutex for the device
 */
struct ade9078_device {
	struct spi_device *spi;
	u8 *tx;
	u8 *rx;
	u8 tx_buff[2];
	u8 rx_buff[ADE9078_WFB_PAGE_ARRAY_SIZE] ____cacheline_aligned;
	struct spi_transfer	xfer[2];
	struct spi_message spi_msg;
	struct mutex lock;
	struct regmap *regmap;
	struct iio_dev *indio_dev;
};

//IIO channels of the ade9078
static const struct iio_chan_spec ade9078_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 0,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
							BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 32,
			.storagebits = 32,
			.shift = 0,
			.endianness = IIO_BE,
		},
	},
};

/*
 * ade9078_spi_write_reg() - ade9078 write register over SPI
 * the data format for communicating with the ade9078 over SPI
 * is very specific
 * @context:	void pointer to the SPI device
 * @reg:		address of the of desired register
 * @val:  		value to be written to the ade9078
 */
static int ade9078_spi_write_reg(void *context, unsigned int reg, unsigned int val)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct ade9078_device *ade9078_dev = spi_get_drvdata(spi);

	u16 addr;
	int ret = 0;
	struct spi_transfer xfer[] = {
		{
			.tx_buf = ade9078_dev->tx,
			.bits_per_word = 8,
			.len = 6,
		},
	};

	addr = ADE9078_WRITE_REG(reg);

	mutex_lock(&ade9078_dev->lock);
	ade9078_dev->tx[5] = (u8)  val & 0xFF;
	ade9078_dev->tx[4] = (u8) (val >> 8) & 0xFF;
	ade9078_dev->tx[3] = (u8) (val >> 16) & 0xFF;
	ade9078_dev->tx[2] = (u8) (val >> 24) & 0xFF;
	ade9078_dev->tx[1] = (u8) addr;
	ade9078_dev->tx[0] = (u8) (addr >> 8);

	//registers which are 16 bits
	if(reg > 0x480 && reg < 0x4FE)
	{
		ade9078_dev->tx[3] = (u8)  val & 0xFF;
		ade9078_dev->tx[2] = (u8) (val >> 8) & 0xFF;
		xfer[0].len = 4;
	}

	ret = spi_sync_transfer(ade9078_dev->spi, xfer, ARRAY_SIZE(xfer));
	if (ret)
	{
		dev_err(&ade9078_dev->spi->dev, "problem when writing register 0x%x", reg);
	}

	mutex_unlock(&ade9078_dev->lock);
	return ret;
}

/*
 * ade9078_spi_write_reg() - ade9078 read register over SPI
 * the data format for communicating with the ade9078 over SPI
 * is very specific
 * @context:	void pointer to the SPI device
 * @reg:		address of the of desired register
 * @val:  		value to be read to the ade9078
 */
static int ade9078_spi_read_reg(void *context, unsigned int reg, unsigned int *val)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct ade9078_device *ade9078_dev = spi_get_drvdata(spi);

	u16 addr;
	int ret = 0;
	struct spi_transfer xfer[] = {
		{
			.tx_buf = ade9078_dev->tx,
			.bits_per_word = 8,
			.len = 2,
		},
		{
			.rx_buf = ade9078_dev->rx,
			.bits_per_word = 8,
			.len = 6,
		},
	};

	addr = ADE9078_READ_REG(reg);

	mutex_lock(&ade9078_dev->lock);
	ade9078_dev->tx[1] = (u8) addr;
	ade9078_dev->tx[0] = (u8) (addr >> 8);

	//registers which are 16 bits
	if(reg > 0x480 && reg < 0x4FE)
		xfer[1].len = 4;

	ret = spi_sync_transfer(ade9078_dev->spi, xfer, ARRAY_SIZE(xfer));
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "problem when reading register 0x%x", reg);
		goto err_ret;
	}

	//registers which are 16 bits
	if(reg > 0x480 && reg < 0x4FE)
		*val = (ade9078_dev->rx[0] << 8) | ade9078_dev->rx[1];
	else
		*val = (ade9078_dev->rx[0] << 24) | (ade9078_dev->rx[1] << 16) | (ade9078_dev->rx[2] << 8) | ade9078_dev->rx[3];

err_ret:
	mutex_unlock(&ade9078_dev->lock);
	return ret;
}

/*
 * ade9078_align() - rearranges the bytes to match BE
 * @x:  		input of the 32 bit message to be rearranged
 */
static u32 ade9078_align(const u8 *x)
{
	return x[2] << 24 | x[3] << 16 | x[0] << 8 | x[1];
}

static void ade9078_pop_wfb(struct iio_poll_func *pf)
{
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	u32 i;
	u32 data;

	for(i=0; i <= ADE9078_WFB_PAGE_SIZE; i=i+4)
	{
		data = ade9078_align(&ade9078_dev->rx_buff[i]);
		iio_push_to_buffers(ade9078_dev->indio_dev, &data);
	}
}

/*
 * ade9078_trigger_handler() - the bottom half of the pollfunc
 * for the iio trigger buffer. It acquires data trough the SPI
 * and rearranges the data to match BE
 */
static irqreturn_t ade9078_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	int ret;

	if(bitmap_empty(indio_dev->active_scan_mask, indio_dev->masklength))
	{
		dev_err(&ade9078_dev->spi->dev, "Bitmap empty in trigger handler");
		goto err_out;
	}

	mutex_lock(&ade9078_dev->lock);
	ret = spi_sync(ade9078_dev->spi, &ade9078_dev->spi_msg);
	if(ret)
	{
		mutex_unlock(&ade9078_dev->lock);
		dev_err(&ade9078_dev->spi->dev, "SPI fail in trigger handler");
		goto err_out;
	}

//	data = ade9078_align(ade9078_dev->rx_buff);
//	iio_push_to_buffers_with_timestamp(indio_dev, &data, pf->timestamp);

	ade9078_pop_wfb(pf);

	mutex_unlock(&ade9078_dev->lock);

err_out:

	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

int ade9078_en_wfb(struct ade9078_device *ade9078_dev, bool state)
{
	if(state)
		return regmap_update_bits(ade9078_dev->regmap, ADDR_WFB_CFG, BIT_MASK(4), BIT_MASK(4));
	else
		return regmap_update_bits(ade9078_dev->regmap, ADDR_WFB_CFG, BIT_MASK(4), 0);
}

/*
 * ade9078_configure_scan() - sets up the transfer parameters
 * as well as the tx and rx buffers
 * @indio_dev:		the IIO device
 */
int ade9078_configure_scan(struct iio_dev *indio_dev)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	int ret = 0;
	u16 addr;

	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;

	addr = ADE9078_READ_REG(ADDR_WF_BUFF);

	ade9078_dev->tx_buff[1] = (u8) addr;
	ade9078_dev->tx_buff[0] = (u8) (addr >> 8);

	ade9078_dev->xfer[0].tx_buf = &ade9078_dev->tx_buff[0];
	ade9078_dev->xfer[0].bits_per_word = 8;
	ade9078_dev->xfer[0].len = 2;

	ade9078_dev->xfer[1].rx_buf = &ade9078_dev->rx_buff[0];
	ade9078_dev->xfer[1].bits_per_word = 8;
	ade9078_dev->xfer[1].len = ADE9078_WFB_PAGE_ARRAY_SIZE;

	spi_message_init(&ade9078_dev->spi_msg);
	spi_message_add_tail(&ade9078_dev->xfer[0], &ade9078_dev->spi_msg);
	spi_message_add_tail(&ade9078_dev->xfer[1], &ade9078_dev->spi_msg);

	return ret;
}

/*
 * Regmap configuration
 * The register access of the ade9078 requires a 16 bit address
 * with the read flag on bit 3. This is not supported by default
 * regmap functionality, thus reg_read and reg_write have been
 * replaced with custom functions
 */
static const struct regmap_config ade9078_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	//reg_read and write require the use of devm_regmap_init
	//instead of devm_regmap_init_spi
	.reg_read = ade9078_spi_read_reg,
	.reg_write = ade9078_spi_write_reg,
	.zero_flag_mask = true,
};

static int ade9078_raw_to_val(int val, int full_scale)
{
	u32 tmp = val;

	tmp /= (full_scale/1000);

	return (int)tmp;
}

/*
 * ade9078_read_raw() - IIO read function
 * @indio_dev:		the IIO device
 * @chan:			channel specs of the ade9078
 * @val:			first half of the read value
 * @val2:			second half of the read value
 * @mask:			info mask of the channel
 */
static int ade9078_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	int ret;
	u32 tmp;
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);
		ret = regmap_read(ade9078_dev->regmap, ADDR_AV_PCF, val);
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		mutex_lock(&indio_dev->mlock);
		ret = regmap_read(ade9078_dev->regmap, ADDR_AV_PCF, &tmp);
		mutex_unlock(&indio_dev->mlock);

		*val = ade9078_raw_to_val(tmp, ADE9078_PCF_FULL_SCALE_CODES);
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}

	return ret;
}

/*
 * ade9078_write_raw() - IIO write function
 * @indio_dev:		the IIO device
 * @chan:			channel specs of the ade9078
 * @val:			first half of the written value
 * @val2:			second half of the written value
 * @mask:			info mask of the channel
 */
static int ade9078_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return 0;
	}

	return -EINVAL;
}

/*
 * ade9078_reg_acess() - IIO debug register access
 * @indio_dev:		the IIO device
 * @reg:			register to be accessed
 * @tx_val:			value to be transmitted
 * @rx_val:			value to be received
 */
static int ade9078_reg_acess(struct iio_dev *indio_dev,
		unsigned int reg,
		unsigned int tx_val,
		unsigned int *rx_val)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);

	if (rx_val)
		return regmap_read(ade9078_dev->regmap, reg, rx_val);
	else
		return regmap_write(ade9078_dev->regmap, reg, tx_val);
}

/*
 * ade9078_setup() - initial register setup of the ade9078
 * @ade9078_dev:		ade9078 device data
 */
static int ade9078_setup(struct ade9078_device *ade9078_dev)
{
	int ret = 0;

	dev_info(&ade9078_dev->spi->dev, "Setup started");
	ret = regmap_write(ade9078_dev->regmap, ADDR_PGA_GAIN, ADE9078_PGA_GAIN);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_CONFIG0, ADE9078_CONFIG0);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_CONFIG1, ADE9078_CONFIG1);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_CONFIG2, ADE9078_CONFIG2);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_CONFIG3, ADE9078_CONFIG3);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_ACCMODE, ADE9078_ACCMODE);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_ZX_LP_SEL, ADE9078_ZX_LP_SEL);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_MASK0, ADE9078_MASK0);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_MASK1, ADE9078_MASK1);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_EVENT_MASK, ADE9078_EVENT_MASK);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_WFB_CFG, ADE9078_WFB_CFG);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_VLEVEL, ADE9078_VLEVEL);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_DICOEFF, ADE9078_DICOEFF);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_EGY_TIME, ADE9078_EGY_TIME);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_EP_CFG, ADE9078_EP_CFG);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_RUN, ADE9078_RUN_ON);
	if(ret)
		return ret;

	dev_info(&ade9078_dev->spi->dev, "Setup finished");

	return ret;
}

static const struct iio_info ade9078_info = {
	.read_raw = &ade9078_read_raw,
	.write_raw = &ade9078_write_raw,
	.debugfs_reg_access = &ade9078_reg_acess,
};

static int ade9078_probe(struct spi_device *spi)
{
	int ret = 0;
	struct ade9078_device *ade9078_dev;
	struct iio_dev *indio_dev;
	struct regmap *regmap;


	printk(KERN_INFO "Enter ade9078_probe\n");

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*ade9078_dev));
	if(indio_dev == NULL) {
		dev_err(&spi->dev,"Unable to allocate ADE9078 IIO");
		return -ENOMEM;
	}
	ade9078_dev = iio_priv(indio_dev);
	if(ade9078_dev == NULL) {
		dev_err(&spi->dev,"Unable to allocate ADE9078 device structure");
		return -ENOMEM;
	}

	ade9078_dev->rx = devm_kcalloc(&spi->dev, 6, sizeof(*ade9078_dev->rx), GFP_KERNEL);
	if(ade9078_dev->rx == NULL)
	{
		dev_err(&spi->dev,"Unable to allocate ADE9078 RX Buffer");
		return-ENOMEM;
	}
	ade9078_dev->tx = devm_kcalloc(&spi->dev, 10, sizeof(*ade9078_dev->tx), GFP_KERNEL);
	if(ade9078_dev->tx == NULL) {
		dev_err(&spi->dev,"Unable to allocate ADE9078 TX Buffer");
		return -ENOMEM;
	}
	regmap = devm_regmap_init(&spi->dev, NULL, spi, &ade9078_regmap_config);
	if (IS_ERR(regmap))
	{
		dev_err(&spi->dev,"Unable to allocate ADE9078 regmap");
		return PTR_ERR(regmap);
	}

	mutex_init(&ade9078_dev->lock);
	spi_set_drvdata(spi, ade9078_dev);

	ade9078_dev->spi = spi;
	ade9078_dev->spi->mode = SPI_MODE_0;
	spi_setup(ade9078_dev->spi);

	indio_dev->name = KBUILD_MODNAME;
	indio_dev->dev.parent = &ade9078_dev->spi->dev;
	indio_dev->info = &ade9078_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ade9078_channels;
	indio_dev->num_channels = ARRAY_SIZE(ade9078_channels);

	ade9078_dev->regmap = regmap;
	ade9078_dev->indio_dev = indio_dev;

	ade9078_configure_scan(indio_dev);


	ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev, &iio_pollfunc_store_time,
					      &ade9078_trigger_handler, NULL);
	if (ret) {
		dev_err(&spi->dev, "Failed to setup triggered buffer: %d\n", ret);
		return ret;
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Unable to register IIO device");
		return ret;
	}

	ret = ade9078_setup(ade9078_dev);
	if (ret) {
		dev_err(&spi->dev, "Unable to setup ADE9078");
		return ret;
	}

	ade9078_en_wfb(ade9078_dev, true);

	return ret;
};

static int ade9078_remove(struct spi_device *spi)
{
	struct ade9078_device *ade9078_dev = spi_get_drvdata(spi);
	struct iio_dev *indio_dev = ade9078_dev->indio_dev;

	printk(KERN_INFO "Exit ade9078_probe\n");
	ade9078_en_wfb(ade9078_dev, false);
	iio_device_unregister(indio_dev);

	return 0;
}

static const struct spi_device_id ade9078_id[] = {
		{"ade9078", 0},
		{}
};
static struct spi_driver ade9078_driver = {
		.driver = {
				.name = "ade9078",
		},
		.probe = ade9078_probe,
		.remove = ade9078_remove,
		.id_table = ade9078_id,
};

module_spi_driver(ade9078_driver);

MODULE_AUTHOR("Ciprian Hegbeli <ciprian.hegbeli@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADE9078 Polyphase Multifunction Energy Metering IC Driver");
MODULE_LICENSE("GPL v2");
