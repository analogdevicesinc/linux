/*
 * ADI-AIM ADI ADC Interface Module
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * http://wiki.analog.com/resources/fpga/xilinx/fmc/ad9467
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_spi.h>

#include "../iio.h"
#include "../sysfs.h"
#include "../buffer.h"

#include "cf_ad9467.h"

static int aim_spi_read(struct aim_state *st, unsigned reg)
{
	unsigned long timeout = jiffies + HZ / 4;
	int ret;

	if (st->spi) {
		unsigned char *buf = st->data;
		buf[0] = 0x80 | (reg >> 8);
		buf[1] = reg & 0xFF;

		ret = spi_write_then_read(st->spi, &buf[0], 2, &buf[2], 1);
		if (ret < 0)
			return ret;

		return buf[2];
	}

	aim_write(st, AD9467_PCORE_SPI_CTRL, 0);
	aim_write(st, AD9467_PCORE_SPI_CTRL,
		AD9647_SPI_START |
		AD9647_SPI_READ |
		AD9647_SPI_SEL(st->spi_ssel) |
		AD9647_SPI_ADDR(reg));

	while (!(aim_read(st, AD9467_PCORE_SPI_RDSTAT) & AD9647_SPI_IDLE)) {
		cpu_relax();
		if (time_after(jiffies, timeout))
			return -EIO;
	}

	return AD9647_SPI_READVAL(aim_read(st, AD9467_PCORE_SPI_RDSTAT));
}

static int aim_spi_write(struct aim_state *st, unsigned reg, unsigned val)
{
	unsigned long timeout = jiffies + HZ / 4;

	int ret;

	if (st->spi) {
		unsigned char *buf = st->data;
		buf[0] = reg >> 8;
		buf[1] = reg & 0xFF;
		buf[2] = val;
		ret = spi_write(st->spi, buf, 3);
		if (ret < 0)
			return ret;

		return 0;
	}

	aim_write(st, AD9467_PCORE_SPI_CTRL, 0);
	aim_write(st, AD9467_PCORE_SPI_CTRL,
		AD9647_SPI_START |
		AD9647_SPI_WRITE |
		AD9647_SPI_SEL(st->spi_ssel) |
		AD9647_SPI_ADDR(reg) |
		AD9647_SPI_DATA(val));


	while (!(aim_read(st, AD9467_PCORE_SPI_RDSTAT) & AD9647_SPI_IDLE)) {
		cpu_relax();
		if (time_after(jiffies, timeout))
			return -EIO;
	}

	return 0;
}

static int aim_debugfs_open(struct inode *inode, struct file *file)
{
	if (inode->i_private)
		file->private_data = inode->i_private;

	return 0;
}

static ssize_t aim_debugfs_pncheck_read(struct file *file, char __user *userbuf,
			      size_t count, loff_t *ppos)
{
	struct iio_dev *indio_dev = file->private_data;
	struct aim_state *st = iio_priv(indio_dev);
	char buf[80];
	ssize_t len;
	unsigned stat;

	stat = aim_read(st, AD9467_PCORE_ADC_STAT);
	len = sprintf(buf, "%s %s\n", (stat & AD9467_PCORE_ADC_STAT_PN_OOS) ?
		"Out of Sync :" : "In Sync :",
		(stat & AD9467_PCORE_ADC_STAT_PN_ERR) ?
		"PN Error" : "No Error");

	aim_write(st, AD9467_PCORE_ADC_STAT, 0xF);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t aim_debugfs_pncheck_write(struct file *file,
		     const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct iio_dev *indio_dev = file->private_data;
	struct aim_state *st = iio_priv(indio_dev);
	unsigned mode;
	char buf[80], *p = buf;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(p, userbuf, count))
		return -EFAULT;

	p[count] = 0;

	if (sysfs_streq(p, "PN9"))
		mode = TESTMODE_PN9_SEQ;
	else if (sysfs_streq(p, "PN23"))
		mode = TESTMODE_PN23_SEQ;
	else
		mode = TESTMODE_OFF;

	aim_spi_write(st, ADC_REG_TEST_IO, mode);
	aim_spi_write(st, ADC_REG_TRANSFER, TRANSFER_SYNC);

	aim_write(st, AD9467_PCORE_PN_ERR_CTRL, (mode == TESTMODE_PN23_SEQ) ?
		  AD9467_PN23_EN : AD9467_PN9_EN);

	mdelay(1); /* FIXME */

	aim_write(st, AD9467_PCORE_ADC_STAT,
		  AD9467_PCORE_ADC_STAT_PN_OOS |
		  AD9467_PCORE_ADC_STAT_PN_OOS |
		  AD9467_PCORE_ADC_STAT_OVR);

	return count;
}

static const struct file_operations aim_debugfs_pncheck_fops = {
	.open = aim_debugfs_open,
	.read = aim_debugfs_pncheck_read,
	.write = aim_debugfs_pncheck_write,
};

static int aim_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct aim_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		ret = aim_spi_write(st, reg, writeval);
		aim_spi_write(st, ADC_REG_TRANSFER, TRANSFER_SYNC);
	} else {
		ret = aim_spi_read(st, reg);
		if (ret < 0)
			return ret;
		*readval = ret;

		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int aim_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct aim_state *st = iio_priv(indio_dev);
	int i;
	unsigned vref_val;

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		vref_val = aim_spi_read(st, ADC_REG_VREF) &
			(st->id == CHIPID_AD9643 ? AD9643_REG_VREF_MASK :
			AD9467_REG_VREF_MASK);

		for (i = 0; i < st->chip_info->num_scales; i++)
			if (vref_val == st->chip_info->scale_table[i][1])
				break;

		*val =  0;
		*val2 = st->chip_info->scale_table[i][0];

		return IIO_VAL_INT_PLUS_MICRO;
	}
	return -EINVAL;
}


static int aim_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct aim_state *st = iio_priv(indio_dev);
	int i;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (val != 0)
			return -EINVAL;

		for (i = 0; i < st->chip_info->num_scales; i++)
			if (val2 == st->chip_info->scale_table[i][0]) {
				aim_spi_write(st, ADC_REG_VREF,
					st->chip_info->scale_table[i][1]);
				aim_spi_write(st, ADC_REG_TRANSFER,
					      TRANSFER_SYNC);
				return 0;
			}

		return -EINVAL;
	default:
		return -EINVAL;
	}
}

static ssize_t aim_show_scale_available(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct aim_state *st = iio_priv(indio_dev);
	int i, len = 0;

	for (i = 0; i < st->chip_info->num_scales; i++)
		len += sprintf(buf + len, "0.%06u ",
			       st->chip_info->scale_table[i][0]);

	len += sprintf(buf + len, "\n");

	return len;
}

static IIO_DEVICE_ATTR(in_voltage_scale_available, S_IRUGO,
		       aim_show_scale_available, NULL, 0);

static struct attribute *aim_attributes[] = {
	&iio_dev_attr_in_voltage_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group aim_attribute_group = {
	.attrs = aim_attributes,
};

static const int ad9467_scale_table[][2] = {
	{30517, 0}, {32043, 6}, {33569, 7},
	{35095, 8}, {36621, 9}, {38146, 10},
};

static const int ad9643_scale_table[][2] = {
	{31738, 0xF}, {31403, 0xE}, {31067, 0xD}, {30731, 0xC}, {30396, 0xB},
	{30060, 0xA}, {29724, 0x9}, {29388, 0x8}, {29053, 0x7}, {28717, 0x6},
	{28381, 0x5}, {28046, 0x4}, {27710, 0x3}, {27374, 0x2}, {27039, 0x1},
	{26703, 0x0}, {26367, 0x1F}, {26031, 0x1E}, {25696, 0x1D},
	{25360, 0x1C}, {25024, 0x1B}, {24689, 0x1A}, {24353, 0x19},
	{24017, 0x18}, {23682, 0x17}, {23346, 0x16}, {23010, 0x15},
	{22675, 0x14}, {22339, 0x13}, {22003, 0x12}, {21667, 0x11},
	{21332, 0x10},
};

#define AIM_CHAN(_chan, _si, _bits, _sign)			\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT,			\
	  .scan_index = _si,						\
	  .scan_type =  IIO_ST(_sign, _bits, 16, 0)}

static const struct aim_chip_info aim_chip_info_tbl[] = {
	[ID_AD9467] = {
		.name = "AD9467",
		.scale_table = ad9467_scale_table,
		.num_scales = ARRAY_SIZE(ad9467_scale_table),
		.num_channels = 1,
		.available_scan_masks[0] = BIT(0),
		.channel[0] = AIM_CHAN(0, 0, 16, 's'),
	},
	[ID_AD9643] = {
		.name = "AD9643",
		.scale_table = ad9643_scale_table,
		.num_scales = ARRAY_SIZE(ad9643_scale_table),
		.num_channels = 2,
		.available_scan_masks[0] = BIT(0) | BIT(1),
		.channel[0] = AIM_CHAN(0, 0, 14, 'u'),
		.channel[1] = AIM_CHAN(1, 1, 14, 'u'),
	},
};

static const struct iio_info aim_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &aim_read_raw,
	.write_raw = &aim_write_raw,
	.attrs = &aim_attribute_group,
	.debugfs_reg_access = &aim_reg_access,
};

struct aim_dma_params {
	struct device_node *of_node;
	int chan_id;
};

static bool aim_dma_filter(struct dma_chan *chan, void *param)
{
	struct aim_dma_params *p = param;

	return chan->device->dev->of_node == p->of_node &&
		chan->chan_id == p->chan_id;
}

/**
 * aim_of_probe - probe method for the AIM device.
 * @of_dev:	pointer to OF device structure
 * @match:	pointer to the structure used for matching a device
 *
 * This function probes the AIM device in the device tree.
 * It initializes the driver data structure and the hardware.
 * It returns 0, if the driver is bound to the AIM device, or a negative
 * value if there is an error.
 */
static int __devinit aim_of_probe(struct platform_device *op)
{
	struct iio_dev *indio_dev;
	struct device *dev = &op->dev;
	struct aim_state *st;
	struct resource r_mem; /* IO mem resources */
	struct spi_master *spi_master;
	struct device_node *nspi;
	struct aim_dma_params dma_params;
	struct of_phandle_args dma_spec;
	dma_cap_mask_t mask;
	resource_size_t remap_size, phys_addr;
	unsigned def_mode, dco_delay;
	int ret;

	dev_info(dev, "Device Tree Probing \'%s\'\n",
		 op->dev.of_node->name);

	/* Get iospace for the device */
	ret = of_address_to_resource(op->dev.of_node, 0, &r_mem);
	if (ret) {
		dev_err(dev, "invalid address\n");
		return ret;
	}

	indio_dev = iio_allocate_device(sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	dev_set_drvdata(dev, indio_dev);
	mutex_init(&st->lock);

	phys_addr = r_mem.start;
	remap_size = resource_size(&r_mem);
	if (!request_mem_region(phys_addr, remap_size, KBUILD_MODNAME)) {
		dev_err(dev, "Couldn't lock memory region at 0x%08llX\n",
			(unsigned long long)phys_addr);
		ret = -EBUSY;
		goto failed1;
	}

	st->regs = ioremap(phys_addr, remap_size);
	if (st->regs == NULL) {
		dev_err(dev, "Couldn't ioremap memory at 0x%08llX\n",
			(unsigned long long)phys_addr);
		ret = -EFAULT;
		goto failed2;
	}
	/* Get dma channel for the device */
	ret = of_parse_phandle_with_args(op->dev.of_node, "dma-request",
					 "#dma-cells", 0, &dma_spec);
	if (ret) {
		dev_err(dev, "Couldn't parse dma-request\n");
		goto failed2;
	}

	dma_params.of_node = dma_spec.np;
	dma_params.chan_id = dma_spec.args[0];

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE | DMA_PRIVATE, mask);

	st->rx_chan = dma_request_channel(mask, aim_dma_filter, &dma_params);
	if (!st->rx_chan) {
		dev_err(dev, "failed to find rx dma device\n");
		goto failed2;
	}
	/*
	 * Get SPI configuration interface for the device
	 * We are platform_driver, so we need other means to get the
	 * associated control interface.
	 */
	ret = of_property_read_u32(op->dev.of_node,
				   "spibus-slaveselect-connected",
				   &st->spi_ssel);
	if (ret) {
		dev_err(dev, "failed to get connected SPI slave select\n");
		goto failed3;
	}

	nspi = of_parse_phandle(op->dev.of_node, "spibus-connected", 0);
	spi_master = spi_of_node_to_master(nspi);

	if (spi_master != NULL) {
		struct spi_board_info info = {
			.modalias = KBUILD_MODNAME,
			.max_speed_hz = 1000000,
			.chip_select = st->spi_ssel,
			.mode = SPI_MODE_0 | SPI_3WIRE,
		};
		st->spi = spi_new_device(spi_master, &info);
		if (st->spi == NULL) {
			dev_err(dev, "failed to add spi device\n");
			goto failed3;
		}
	} else {
		dev_dbg(dev, "could not find SPI master node,"
			"using pcore spi implementation\n");
	}

	/* Probe device */
	st->id = aim_spi_read(st, ADC_REG_CHIP_ID);

	switch (st->id) {
	case CHIPID_AD9467:
		st->chip_info = &aim_chip_info_tbl[ID_AD9467];
		def_mode = AD9467_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		break;
	case CHIPID_AD9643:
		st->chip_info = &aim_chip_info_tbl[ID_AD9643];
		def_mode = AD9643_DEF_OUTPUT_MODE | OUTPUT_MODE_OFFSET_BINARY;
		break;
	default:
		dev_err(dev, "Unrecognized CHIP_ID 0x%X\n", st->id);
		ret = -ENODEV;
		goto failed3;
	}

	indio_dev->dev.parent = dev;
	indio_dev->name = op->dev.of_node->name;
	indio_dev->channels = st->chip_info->channel;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = st->chip_info->num_channels;
	indio_dev->available_scan_masks = st->chip_info->available_scan_masks;
	indio_dev->info = &aim_info;

	init_completion(&st->dma_complete);

	aim_spi_write(st, ADC_REG_OUTPUT_MODE, def_mode);
	aim_spi_write(st, ADC_REG_TEST_IO, TESTMODE_OFF);

	ret = of_property_read_u32(op->dev.of_node,
				   "dco-output-delay",
				   &dco_delay);
	if (!ret) {
		aim_spi_write(st, ADC_REG_OUTPUT_DELAY, dco_delay);
	}
	aim_spi_write(st, ADC_REG_TRANSFER, TRANSFER_SYNC);

	aim_configure_ring(indio_dev);
	ret = iio_buffer_register(indio_dev,
				  st->chip_info->channel,
				  ARRAY_SIZE(st->chip_info->channel));
	if (ret)
		goto failed4;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto failed4;

	dev_info(dev, "ADI AIM (0x%X) at 0x%08llX mapped to 0x%p,"
		 " DMA-%d probed ADC %s as %s\n",
		 aim_read(st, AD9467_PCORE_VERSION),
		 (unsigned long long)phys_addr, st->regs,
		 st->rx_chan->chan_id, st->chip_info->name,
		 (aim_read(st, AD9467_PCORE_IDENT) &
		AD9467_PCORE_IDENT_SLAVE) ? "SLAVE" : "MASTER");

	if (iio_get_debugfs_dentry(indio_dev))
		debugfs_create_file("pseudorandom_err_check", 0644,
					iio_get_debugfs_dentry(indio_dev),
					indio_dev, &aim_debugfs_pncheck_fops);

	return 0;

failed4:
	aim_unconfigure_ring(indio_dev);
failed3:
	dma_release_channel(st->rx_chan);
failed2:
	release_mem_region(phys_addr, remap_size);
failed1:
	iio_free_device(indio_dev);
	dev_set_drvdata(dev, NULL);

	return ret;
}

/**
 * aim_of_remove - unbinds the driver from the AIM device.
 * @of_dev:	pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees any resources allocated to
 * the device.
 */
static int __devexit aim_of_remove(struct platform_device *op)
{
	struct device *dev = &op->dev;
	struct resource r_mem; /* IO mem resources */
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct aim_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_buffer_unregister(indio_dev);
	aim_unconfigure_ring(indio_dev);

	dma_release_channel(st->rx_chan);

	iounmap(st->regs);

	/* Get iospace of the device */
	if (of_address_to_resource(op->dev.of_node, 0, &r_mem))
		dev_err(dev, "invalid address\n");
	else
		release_mem_region(r_mem.start, resource_size(&r_mem));

	iio_free_device(indio_dev);

	dev_set_drvdata(dev, NULL);

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id aim_of_match[] __devinitconst = {
	{ .compatible = "xlnx,cf-ad9467-core-1.00.a", },
	{ .compatible = "xlnx,cf-ad9643-core-1.00.a", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, aim_of_match);

static struct platform_driver aim_of_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = aim_of_match,
	},
	.probe		= aim_of_probe,
	.remove		= __devexit_p(aim_of_remove),
};

module_platform_driver(aim_of_driver);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Analog Devices ADI-AIM");
MODULE_LICENSE("GPL v2");
