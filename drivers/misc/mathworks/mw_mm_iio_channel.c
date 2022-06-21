/*
 * MathWorks Memory Mapped Channel
 *
 * Copyright 2016 The MathWorks, Inc
 *
 * Licensed under the GPL-2.
 */

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/idr.h>
#include <linux/device.h>
#include <linux/errno.h>

#include <linux/string.h>
#include <linux/mathworks/mathworks_ip.h>
#include "mw_mm_iio_channel.h"
#include "mathworks_ipcore.h"

static DEFINE_IDA(mw_mm_iio_channel_ida);

#define MWDEV_TO_MWIP(mwdev)			(mwdev->mw_ip_info)
#define IP2DEVP(mwdev)  (MWDEV_TO_MWIP(mwdev)->dev)

#define MW_MM_IIO_ENUM IIO_ENUM
#define MW_MM_IIO_ENUM_AVAILABLE(_name, _shared_by, _e) \
{ \
	.name = (_name "_available"), \
	.shared = (_shared_by), \
	.read = iio_enum_available_read, \
	.private = (uintptr_t)(_e), \
}

struct mw_mm_iio_channel_info {
	enum iio_device_direction 		iio_direction;
};

enum mw_mm_iio_reg_access {
	MW_MM_IO_MODE_DISABLED = 0,
	MW_MM_IO_MODE_ENABLED,
};

struct mw_mm_iio_chandev {
	struct mathworks_ipcore_dev 			*mwdev;
	struct device							dev;
	enum iio_device_direction 				iio_direction;
	enum mw_mm_iio_reg_access					reg_access;
};

static void mw_mm_iio_chan_ida_remove(void *opaque){
	struct mw_mm_iio_chandev* mwchan = opaque;
	ida_simple_remove(&mw_mm_iio_channel_ida, mwchan->dev.id);
}

/*************
 * IO Modes
 *************/
static const char * const mw_mm_iio_reg_accesss[] = { "disabled", "enabled" };

static int mw_mm_iio_channel_get_reg_access(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan)
{
	struct mw_mm_iio_chandev *mwchan = iio_priv(indio_dev);

	return mwchan->reg_access;
}

static int mw_mm_iio_channel_set_reg_access(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan, unsigned int reg_access)
{
	struct mw_mm_iio_chandev *mwchan = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	mwchan->reg_access = reg_access;
	mutex_unlock(&indio_dev->mlock);

	return 0;
}

static const struct iio_enum mw_mm_iio_reg_access_enum = {
	.items = mw_mm_iio_reg_accesss,
	.num_items = ARRAY_SIZE(mw_mm_iio_reg_accesss),
	.get = mw_mm_iio_channel_get_reg_access,
	.set = mw_mm_iio_channel_set_reg_access,
};

static const struct iio_chan_spec_ext_info mw_mm_iio_ch_reg_access[] = {
	MW_MM_IIO_ENUM("reg_access", IIO_SHARED_BY_ALL, &mw_mm_iio_reg_access_enum),
	MW_MM_IIO_ENUM_AVAILABLE("reg_access", IIO_SHARED_BY_ALL, &mw_mm_iio_reg_access_enum),
	{ },
};



static int mw_mm_iio_channel_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct mw_mm_iio_chandev *mwchan = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	if (mwchan->reg_access == MW_MM_IO_MODE_DISABLED){
		ret = -EACCES;
	} else {
		if (readval == NULL) {
			mw_ip_write32(mwchan->mwdev->mw_ip_info, reg & 0xFFFF, writeval);
		} else {
			*readval = mw_ip_read32(mwchan->mwdev->mw_ip_info, reg & 0xFFFF);
		}
	}

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static const struct iio_info mw_mm_iio_dev_info = {
	.debugfs_reg_access = &mw_mm_iio_channel_reg_access,
};

static int mw_mm_setup_info_channel(struct iio_dev *indio_dev,
		int index, const char *name,
		const struct iio_chan_spec_ext_info *info)
{
	struct mw_mm_iio_chandev *mwchan = iio_priv(indio_dev);
	struct iio_chan_spec *channel = (struct iio_chan_spec *)
			&indio_dev->channels[index];

	channel->type = IIO_GENERIC_DATA;
	channel->indexed = 1;
	channel->extend_name = devm_kstrdup(&mwchan->dev, name, GFP_KERNEL);
	if (!channel->extend_name)
		return -ENOMEM;
	channel->ext_info = info;
	channel->scan_index = -ENODEV;

	return 0;
}

static void mw_mm_iio_unregister(void *opaque) {
	struct device *dev = opaque;

	/* Unregister the IIO device */
	devres_release_group(dev, mw_mm_iio_unregister);
}

static int devm_mw_mm_iio_register(struct iio_dev *indio_dev) {
	struct mw_mm_iio_chandev *mwchan = iio_priv(indio_dev);
	int status;
	int chIdx = 0;

	if(!devres_open_group(&mwchan->dev, mw_mm_iio_unregister, GFP_KERNEL))
		return -ENOMEM;

	indio_dev->dev.parent = &mwchan->dev;
	indio_dev->name = dev_name(&mwchan->dev);
	indio_dev->info = &mw_mm_iio_dev_info;


	indio_dev->num_channels = 1; /* reg access channel */
	indio_dev->channels = devm_kzalloc(&mwchan->dev, (indio_dev->num_channels) * sizeof(struct iio_chan_spec), GFP_KERNEL);
	if(!indio_dev->channels)
		return -ENOMEM;

	status = mw_mm_setup_info_channel(indio_dev, chIdx++,
			"reg_access", mw_mm_iio_ch_reg_access);
	if(status)
		return status;

	status = devm_iio_device_register(&mwchan->dev, indio_dev);
	if(status)
		return status;

	devres_close_group(&mwchan->dev, mw_mm_iio_unregister);

	/* Setup the parent device to tear us down on removal */
	status = devm_add_action(mwchan->dev.parent, mw_mm_iio_unregister, &mwchan->dev);
	if(status){
		mw_mm_iio_unregister(&mwchan->dev);
		return status;
	}

	return 0;
}

/* Nothing to actually do upon release */
static void mw_mm_iio_channel_release(struct device *dev)
{
}

static struct iio_dev *devm_mw_mm_iio_alloc(
		struct mathworks_ipcore_dev *mwdev,
		struct device_node *node,
		struct mw_mm_iio_channel_info *info)
{
	struct iio_dev *indio_dev;
	struct mw_mm_iio_chandev *mwchan;
	const char *devname;
	int status;


	if(!devres_open_group(IP2DEVP(mwdev), devm_mw_mm_iio_alloc, GFP_KERNEL))
		return ERR_PTR(-ENOMEM);

	indio_dev = devm_iio_device_alloc(IP2DEVP(mwdev), sizeof(struct mw_mm_iio_chandev));
	if (!indio_dev){
		dev_err(IP2DEVP(mwdev), "Failed to allocate memory for channel %s\n",node->name);
		return ERR_PTR(-ENOMEM);
	}

	mwchan = iio_priv(indio_dev);
	mwchan->mwdev = mwdev;
	mwchan->iio_direction = info->iio_direction;

	device_initialize(&mwchan->dev);

	mwchan->dev.parent = IP2DEVP(mwdev);
	mwchan->dev.of_node = node;
	mwchan->dev.id = ida_simple_get(&mw_mm_iio_channel_ida, 0, 0, GFP_KERNEL);
	if (mwchan->dev.id < 0) {
		return ERR_PTR(mwchan->dev.id);
	}
	status = devm_add_action(IP2DEVP(mwdev),mw_mm_iio_chan_ida_remove, mwchan);
	if(status){
		mw_mm_iio_chan_ida_remove(mwchan);
		return ERR_PTR(status);
	}
	mwchan->dev.release = mw_mm_iio_channel_release;
	/* clone the parent's DMA config */
	memcpy(&mwchan->dev.archdata, &IP2DEVP(mwdev)->archdata, sizeof(struct dev_archdata));
	mwchan->dev.coherent_dma_mask = IP2DEVP(mwdev)->coherent_dma_mask;
	mwchan->dev.dma_mask = IP2DEVP(mwdev)->dma_mask;
	mwchan->dev.dma_range_map = IP2DEVP(mwdev)->dma_range_map;


	status = of_property_read_string(node, "mathworks,dev-name", &devname);
	if (!status) {
		/* Use the specified channel name */
		status = dev_set_name(&mwchan->dev, "%s:%s", dev_name(mwchan->mwdev->mw_ip_info->char_device), devname);
	} else {
		/* Use the node name + dev ID */
		status = dev_set_name(&mwchan->dev, "%s:%s%d", dev_name(mwchan->mwdev->mw_ip_info->char_device), node->name, mwchan->dev.id);
	}
	if (status)
		return ERR_PTR(status);

	status = device_add(&mwchan->dev);
	if (status)
		return ERR_PTR(status);

	status = devm_add_action(IP2DEVP(mwdev), (devm_action_fn)device_unregister, &mwchan->dev);
	if (status) {
		device_unregister(&mwchan->dev);
		return ERR_PTR(status);
	}

	devres_close_group(IP2DEVP(mwdev), devm_mw_mm_iio_alloc);

	return indio_dev;
}

static int mw_mm_iio_channel_probe(
		struct mathworks_ipcore_dev *mwdev,
		struct device_node *node,
		struct mw_mm_iio_channel_info *info)
{
	int status;
	struct iio_dev *indio_dev;

	indio_dev = devm_mw_mm_iio_alloc(mwdev, node, info);
	if (IS_ERR(indio_dev))
		return PTR_ERR(indio_dev);

	status = devm_mw_mm_iio_register(indio_dev);
	if (status)
		return status;

	return 0;
}

static struct mw_mm_iio_channel_info mw_mm_iio_mm2s_info = {
	.iio_direction = IIO_DEVICE_DIRECTION_OUT,
};

static struct mw_mm_iio_channel_info mw_mm_iio_s2mm_info = {
	.iio_direction = IIO_DEVICE_DIRECTION_IN,
};

static const struct of_device_id mw_mm_iio_channel_of_match[] = {
	{ .compatible = "mathworks,mm-write-channel-v1.00", .data = &mw_mm_iio_mm2s_info},
	{ .compatible = "mathworks,mm-read-channel-v1.00", .data = &mw_mm_iio_s2mm_info},
    {},
};

int mw_mm_iio_channels_probe(struct mathworks_ipcore_dev *mwdev)
{
	int status;

	struct device_node *child;
	const struct of_device_id *match;


	for_each_child_of_node(IP2DEVP(mwdev)->of_node,child) {
		match = of_match_node(mw_mm_iio_channel_of_match, child);
		if(match){
			status = mw_mm_iio_channel_probe(mwdev, child, (struct mw_mm_iio_channel_info *)match->data);
			if(status)
				return status;
		}
	}

	return 0;
}

EXPORT_SYMBOL_GPL(mw_mm_iio_channels_probe);

static int __init mw_mm_iio_channel_init(void)
{
	return 0;
}

static void __exit mw_mm_iio_channel_exit(void)
{

}

module_init(mw_mm_iio_channel_init);
module_exit(mw_mm_iio_channel_exit);

MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MathWorks Memory Mapped IIO Channel");
MODULE_ALIAS(DRIVER_NAME);
