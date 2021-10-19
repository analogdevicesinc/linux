#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <linux/spi/spi.h>

#include<linux/module.h>
#include<linux/moduleparam.h>

struct ade9078_device {
	struct spi_device *spi;
	struct mutex lock;
	u8 *tx;
	u8 *rx;
};

int ade9078_spi_read_reg(struct ade9078_device *dev, u16 addr, u32 *val)
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
			.tx_buf = &ade9078_dev->tx[2],
			.rx_buf = ade9078_dev->rx,
			.bits_per_word = 8,
			.len = 4,
		},
	};

	uiAddr = addr;
	uiAddr = (uiAddr << 4);
	uiAddr = (uiAddr | 0x08);

	mutex_lock(&ade9078_dev->lock);
	ade9078_dev->tx[3] = 0x00;
	ade9078_dev->tx[2] = 0x00;
	ade9078_dev->tx[1] = (u8) uiAddr;
	ade9078_dev->tx[0] = (u8) (uiAddr >> 8);

	ret = spi_sync_transfer(ade9078_dev->spi, xfer, ARRAY_SIZE(xfer));
	if (ret)
	{
		dev_err(&ade9078_dev->spi->dev, "problem when reading register 0x%x",addr);
		goto err_ret;
	}

	*val = (ade9078_dev->rx[0] << 24) | (ade9078_dev->rx[1] << 16) | (ade9078_dev->rx[2] << 8) | ade9078_dev->rx[3];

err_ret:
	mutex_unlock(&ade9078_dev->lock);
	return ret;
};

static int ade9078_probe(struct spi_device *spi)
{
	int ret = 0;
	u32 tmp;
	struct ade9078_device *ade9078_dev;

	printk(KERN_INFO "Enter ade9078_probe\n");

	ade9078_dev = devm_kzalloc(&spi->dev, sizeof(ade9078_dev), GFP_KERNEL);
	if(ade9078_dev == NULL)
	{
		printk(KERN_ALERT "Unable to allocate ADE9078 device structure\n");
		return -ENOMEM;
	}

	ade9078_dev->rx = devm_kcalloc(&spi->dev, 4, sizeof(*ade9078_dev->rx), GFP_KERNEL);
	if(ade9078_dev->rx == NULL)
	{
		printk(KERN_ALERT "Unable to allocate ADE9078 RX Buffer\n");
		return-ENOMEM;
	}
	ade9078_dev->tx = devm_kcalloc(&spi->dev, 8, sizeof(*ade9078_dev->tx), GFP_KERNEL);
	if(ade9078_dev->tx == NULL)
	{
		printk(KERN_ALERT "Unable to allocate ADE9078 TX Buffer\n");
		return -ENOMEM;
	}

	mutex_init(&ade9078_dev->lock);
	spi_set_drvdata(spi, ade9078_dev);

	ade9078_dev->spi = spi;

	ade9078_dev->spi->mode = SPI_MODE_0;
	spi_setup(ade9078_dev->spi);

	ret = ade9078_spi_read_reg(ade9078_dev, 0x040f, &tmp);
	if (ret)
	{
		printk(KERN_ALERT "Unable to read spi addr 0x040f\n");
	}

	printk(KERN_INFO "Read value from 0x%x is 0x%x\n", 0x040f, tmp);

	return ret;
};

static int ade9078_remove(struct spi_device *spi)
{
	printk(KERN_INFO "Exit ade9078_probe\n");
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
