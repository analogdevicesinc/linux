#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <linux/module.h>
#include <linux/moduleparam.h>
#include "ade9078.h"

struct ade9078_device {
	struct spi_device *spi;
	struct mutex lock;
	u8 *tx;
	u8 *rx;
};

static const struct iio_chan_spec ade9078_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 0,
		.scan_index = 0,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
							BIT(IIO_CHAN_INFO_PROCESSED),
	},
};

static int ade9078_spi_write_reg(struct ade9078_device *dev, u16 addr, u32 val)
{
	struct ade9078_device *ade9078_dev = dev;
	u16 uiAddr;
	int ret = 0;
	struct spi_transfer xfer[] = {
		{
			.tx_buf = ade9078_dev->tx,
			.bits_per_word = 8,
			.len = 7,
		},
	};

	uiAddr = addr;
	uiAddr = (uiAddr << 4);
	uiAddr = uiAddr;

	mutex_lock(&ade9078_dev->lock);
	ade9078_dev->tx[5] = (u8)  val & 0xFF;
	ade9078_dev->tx[4] = (u8) (val >> 8) & 0xFF;
	ade9078_dev->tx[3] = (u8) (val >> 16) & 0xFF;
	ade9078_dev->tx[2] = (u8) (val >> 24) & 0xFF;
	ade9078_dev->tx[1] = (u8) uiAddr;
	ade9078_dev->tx[0] = (u8) (uiAddr >> 8);

	ret = spi_sync_transfer(ade9078_dev->spi, xfer, ARRAY_SIZE(xfer));
	if (ret)
	{
		dev_err(&ade9078_dev->spi->dev, "problem when writing register 0x%x", addr);
	}

	mutex_unlock(&ade9078_dev->lock);
	return ret;
}

static int ade9078_spi_read_reg(struct ade9078_device *dev, u16 addr, u32 *val)
{
	struct ade9078_device *ade9078_dev = dev;
	u16 uiAddr;
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

	uiAddr = addr;
	uiAddr = (uiAddr << 4);
	uiAddr = (uiAddr | 0x08);

	mutex_lock(&ade9078_dev->lock);
	ade9078_dev->tx[1] = (u8) uiAddr;
	ade9078_dev->tx[0] = (u8) (uiAddr >> 8);

	ret = spi_sync_transfer(ade9078_dev->spi, xfer, ARRAY_SIZE(xfer));
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "problem when reading register 0x%x", addr);
		goto err_ret;
	}

	*val = (ade9078_dev->rx[0] << 24) | (ade9078_dev->rx[1] << 16) | (ade9078_dev->rx[2] << 8) | ade9078_dev->rx[3];

err_ret:
	mutex_unlock(&ade9078_dev->lock);
	return ret;
}

static int ade9078_raw_to_val(int val, int full_scale)
{
	u32 tmp = val;

	tmp /= (full_scale/1000);

	return (int)tmp;
}

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
		ret = ade9078_spi_read_reg(ade9078_dev, ADDR_AV_PCF, val);
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		mutex_lock(&indio_dev->mlock);
		ret = ade9078_spi_read_reg(ade9078_dev, ADDR_AV_PCF, &tmp);
		mutex_unlock(&indio_dev->mlock);

		*val = ade9078_raw_to_val(tmp, ADE9078_PCF_FULL_SCALE_CODES);
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}

	return ret;
}

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

static int ade9078_reg_acess(struct iio_dev *indio_dev,
		unsigned int reg,
		unsigned int tx_val,
		unsigned int *rx_val)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);

	if (rx_val)
		return ade9078_spi_read_reg(ade9078_dev, reg, rx_val);
	else
		return ade9078_spi_write_reg(ade9078_dev, reg, tx_val);
}

static int ade9078_setup(struct ade9078_device *dev)
{
	int ret = 0;

	dev_info(&dev->spi->dev, "Setup started");
	ret = ade9078_spi_write_reg(dev, ADDR_PGA_GAIN, ADE9000_PGA_GAIN);
	if(ret)
		return ret;
	ret = ade9078_spi_write_reg(dev, ADDR_CONFIG0, ADE9000_CONFIG0);
	if(ret)
		return ret;
	ret = ade9078_spi_write_reg(dev, ADDR_CONFIG1, ADE9000_CONFIG1);
	if(ret)
		return ret;
	ret = ade9078_spi_write_reg(dev, ADDR_CONFIG2, ADE9000_CONFIG2);
	if(ret)
		return ret;
	ret = ade9078_spi_write_reg(dev, ADDR_CONFIG3, ADE9000_CONFIG3);
	if(ret)
		return ret;
	ret = ade9078_spi_write_reg(dev, ADDR_ACCMODE, ADE9000_ACCMODE);
	if(ret)
		return ret;
	ret = ade9078_spi_write_reg(dev, ADDR_ZX_LP_SEL, ADE9000_ZX_LP_SEL);
	if(ret)
		return ret;
	ret = ade9078_spi_write_reg(dev, ADDR_MASK0, ADE9000_MASK0);
	if(ret)
		return ret;
	ret = ade9078_spi_write_reg(dev, ADDR_MASK1, ADE9000_MASK1);
	if(ret)
		return ret;
	ret = ade9078_spi_write_reg(dev, ADDR_EVENT_MASK, ADE9000_EVENT_MASK);
	if(ret)
		return ret;
	ret = ade9078_spi_write_reg(dev, ADDR_WFB_CFG, ADE9000_WFB_CFG);
	if(ret)
		return ret;
	ret = ade9078_spi_write_reg(dev, ADDR_VLEVEL, ADE9000_VLEVEL);
	if(ret)
		return ret;
	ret = ade9078_spi_write_reg(dev, ADDR_DICOEFF, ADE9000_DICOEFF);
	if(ret)
		return ret;
	ret = ade9078_spi_write_reg(dev, ADDR_EGY_TIME, ADE9000_EGY_TIME);
	if(ret)
		return ret;
	ret = ade9078_spi_write_reg(dev, ADDR_EP_CFG, ADE9000_EP_CFG);
	if(ret)
		return ret;
	ret = ade9078_spi_write_reg(dev, ADDR_RUN, ADE9000_RUN_ON);
	if(ret)
		return ret;

	dev_info(&dev->spi->dev, "Setup finished");

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
//	u32 tmp;
	struct ade9078_device *ade9078_dev;
	struct iio_dev *indio_dev;


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

	mutex_init(&ade9078_dev->lock);
	spi_set_drvdata(spi, indio_dev);

	ade9078_dev->spi = spi;

	ade9078_dev->spi->mode = SPI_MODE_0;
	spi_setup(ade9078_dev->spi);

	indio_dev->name = KBUILD_MODNAME;
	indio_dev->dev.parent = &ade9078_dev->spi->dev;
	indio_dev->info = &ade9078_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ade9078_channels;
	indio_dev->num_channels = ARRAY_SIZE(ade9078_channels);

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Unable to register IIO device");
		return ret;
	}

//	ret = ade9078_spi_read_reg(ade9078_dev, 0x040f, &tmp);
//	if (ret)
//	{
//		printk(KERN_ALERT "Unable to read spi addr 0x040f\n");
//		return ret;
//	}
//	printk(KERN_INFO "Read value from 0x%x is 0x%x\n", 0x040f, tmp);
//
//	tmp = 0x12345678;
//	ret = ade9078_spi_write_reg(ade9078_dev, 0x0402, tmp);
//	if (ret)
//	{
//		printk(KERN_ALERT "Unable to write spi addr 0x040f\n");
//		return ret;
//	}
//	printk(KERN_INFO "Value 0x%x written to 0x%x\n", tmp, 0x040f);
//
//	ret = ade9078_spi_read_reg(ade9078_dev, 0x0402, &tmp);
//	if (ret)
//	{
//		printk(KERN_ALERT "Unable to read spi addr 0x040f\n");
//		return ret;
//	}
//	printk(KERN_INFO "Read value from 0x%x is 0x%x\n", 0x040f, tmp);

	ret = ade9078_setup(ade9078_dev);
	if (ret) {
		dev_err(&spi->dev, "Unable to setup ADE9078");
		return ret;
	}

	return ret;
};

static int ade9078_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	printk(KERN_INFO "Exit ade9078_probe\n");
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
