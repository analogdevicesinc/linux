/*
 * MathWorks Streaming Channel
 *
 * Copyright 2016 The MathWorks, Inc
 *
 * Licensed under the GPL-2.
 */

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/idr.h>
#include <linux/device.h>
#include <linux/errno.h>

#include <linux/string.h>
#include <linux/mathworks/mathworks_ip.h>
#include "mw_stream_iio_channel.h"
#include "mathworks_ipcore.h"

static DEFINE_IDA(mw_stream_iio_channel_ida);

#define MWDEV_TO_MWIP(mwdev)			(mwdev->mw_ip_info)
#define IP2DEVP(mwdev)  (MWDEV_TO_MWIP(mwdev)->dev)

#define MW_STREAM_IIO_ENUM IIO_ENUM
#define MW_STREAM_IIO_ENUM_AVAILABLE(_name, _shared_by, _e) \
{ \
	.name = (_name "_available"), \
	.shared = (_shared_by), \
	.read = iio_enum_available_read, \
	.private = (uintptr_t)(_e), \
}

struct mw_stream_iio_channel_info {
	enum iio_device_direction 		iio_direction;
};

enum mw_stream_iio_tlast_mode {
	MW_STREAM_TLAST_MODE_AUTO = 0,
	MW_STREAM_TLAST_MODE_USER_LOGIC,
};

enum mw_stream_iio_reset_tlast_mode {
	MW_STREAM_TLAST_MODE_PREBUFFER = 0,
	MW_STREAM_TLAST_MODE_NEVER,
};

enum mw_stream_iio_reset_ip_mode {
	MW_STREAM_RESET_IP_MODE_NONE = 0,
	MW_STREAM_RESET_IP_MODE_ENABLE,
	MW_STREAM_RESET_IP_MODE_DISABLE,
	MW_STREAM_RESET_IP_MODE_ALL,
};

struct mw_stream_iio_chandev {
	struct mathworks_ipcore_dev 			*mwdev;
	struct device							dev;
	enum iio_device_direction 				iio_direction;
	const char								*dmaname;
	enum mw_stream_iio_tlast_mode			tlast_mode;
	enum mw_stream_iio_reset_tlast_mode		reset_tlast_mode;
	enum mw_stream_iio_reset_ip_mode		reset_ip_mode;
	int										tlast_cntr_addr;
	int										num_data_chan;
};

static void mw_stream_iio_chan_ida_remove(void *opaque){
	struct mw_stream_iio_chandev* mwchan = opaque;
	ida_simple_remove(&mw_stream_iio_channel_ida, mwchan->dev.id);
}

static int mw_stream_iio_buffer_submit_block(struct iio_dma_buffer_queue *queue, struct iio_dma_buffer_block *block)
{
	struct iio_dev *indio_dev = queue->driver_data;
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);
	int direction;

	if(mwchan->iio_direction == IIO_DEVICE_DIRECTION_IN) {
		direction = DMA_FROM_DEVICE;
	} else {
		direction = DMA_TO_DEVICE;
	}

	return iio_dmaengine_buffer_submit_block(queue, block, direction);
}


static int mw_stream_iio_buffer_preenable(struct iio_dev *indio_dev)
{
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);

	dev_dbg(&mwchan->dev, "buffer preenable\n");

	switch(mwchan->reset_ip_mode) {
		case MW_STREAM_RESET_IP_MODE_ENABLE:
		case MW_STREAM_RESET_IP_MODE_ALL:
			/* reset the ip core */
			dev_dbg(&mwchan->dev, "resetting IP Core\n");
			mw_ip_reset(mwchan->mwdev);
			break;
		default:
			/* Do Nothing */
			break;
	}
	if (mwchan->tlast_cntr_addr >= 0 && mwchan->tlast_mode == MW_STREAM_TLAST_MODE_AUTO) {
		if(mwchan->reset_tlast_mode == MW_STREAM_TLAST_MODE_PREBUFFER) {
			/* reset the IP core (TODO: only reset the TLAST register)*/
			mw_ip_reset(mwchan->mwdev);
		}
		/* Set the TLAST count */
		mw_ip_write32(mwchan->mwdev->mw_ip_info, mwchan->tlast_cntr_addr, indio_dev->buffer->length);
	}

	return 0;
}
static int mw_stream_iio_buffer_postenable(struct iio_dev *indio_dev)
{
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);

	dev_dbg(&mwchan->dev, "buffer postenable\n");
	return 0;
}

static int mw_stream_iio_buffer_predisable(struct iio_dev *indio_dev)
{
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);

	dev_dbg(&mwchan->dev, "buffer predisable\n");

	switch(mwchan->reset_ip_mode) {
		case MW_STREAM_RESET_IP_MODE_DISABLE:
		case MW_STREAM_RESET_IP_MODE_ALL:
			/* reset the ip core */
			dev_dbg(&mwchan->dev, "resetting IP Core\n");

			mw_ip_reset(mwchan->mwdev);
			break;
		default:
			/* Do Nothing */
			break;
	}

	return 0;
}

static int mw_stream_iio_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);

	dev_dbg(&mwchan->dev, "buffer postdisable\n");
	return 0;
}


static const struct iio_buffer_setup_ops mw_stream_iio_buffer_setup_ops = {

	.preenable = &mw_stream_iio_buffer_preenable,
	.postenable = &mw_stream_iio_buffer_postenable,
	.predisable = &mw_stream_iio_buffer_predisable,
	.postdisable = &mw_stream_iio_buffer_postdisable,
};

static const struct iio_dma_buffer_ops mw_stream_iio_buffer_dma_buffer_ops = {
	.submit = mw_stream_iio_buffer_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

/*************
 * Reset IP Modes
 *************/
static const char * const mw_stream_iio_channel_reset_ip_modes[] = { "none", "enable", "disable", "all" };

static int mw_stream_iio_channel_get_reset_ip_mode(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan)
{
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);

	return mwchan->reset_ip_mode;
}

static int mw_stream_iio_channel_set_reset_ip_mode(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan, unsigned int mode)
{
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	mwchan->reset_ip_mode = mode;
	mutex_unlock(&indio_dev->mlock);

	return 0;
}

static const struct iio_enum mw_stream_iio_channel_reset_ip_mode_enum = {
	.items = mw_stream_iio_channel_reset_ip_modes,
	.num_items = ARRAY_SIZE(mw_stream_iio_channel_reset_ip_modes),
	.get = mw_stream_iio_channel_get_reset_ip_mode,
	.set = mw_stream_iio_channel_set_reset_ip_mode,
};

/*************
 * Reset TLAST Modes
 *************/
static const char * const mw_stream_iio_channel_reset_tlast_modes[] = { "prebuffer", "never" };

static int mw_stream_iio_channel_get_reset_tlast_mode(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan)
{
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);

	return mwchan->reset_tlast_mode;
}

static int mw_stream_iio_channel_set_reset_tlast_mode(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan, unsigned int mode)
{
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	mwchan->reset_tlast_mode = mode;
	mutex_unlock(&indio_dev->mlock);

	return 0;
}

static const struct iio_enum mw_stream_iio_channel_reset_tlast_mode_enum = {
	.items = mw_stream_iio_channel_reset_tlast_modes,
	.num_items = ARRAY_SIZE(mw_stream_iio_channel_reset_tlast_modes),
	.get = mw_stream_iio_channel_get_reset_tlast_mode,
	.set = mw_stream_iio_channel_set_reset_tlast_mode,
};

/*************
 * TLAST Modes
 *************/
static const char * const mw_stream_iio_channel_tlast_modes[] = { "auto", "user_logic" };

static int mw_stream_iio_channel_get_tlast_mode(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan)
{
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);

	return mwchan->tlast_mode;
}

static int mw_stream_iio_channel_set_tlast_mode(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan, unsigned int mode)
{
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	mwchan->tlast_mode = mode;
	mutex_unlock(&indio_dev->mlock);

	return 0;
}

static const struct iio_enum mw_stream_iio_channel_tlast_mode_enum = {
	.items = mw_stream_iio_channel_tlast_modes,
	.num_items = ARRAY_SIZE(mw_stream_iio_channel_tlast_modes),
	.get = mw_stream_iio_channel_get_tlast_mode,
	.set = mw_stream_iio_channel_set_tlast_mode,
};

static const struct iio_chan_spec_ext_info mw_stream_iio_ch_tlast_info[] = {
	MW_STREAM_IIO_ENUM("tlast_mode", IIO_SHARED_BY_ALL, &mw_stream_iio_channel_tlast_mode_enum),
	MW_STREAM_IIO_ENUM_AVAILABLE("tlast_mode", IIO_SHARED_BY_ALL, &mw_stream_iio_channel_tlast_mode_enum),
	MW_STREAM_IIO_ENUM("reset_tlast_mode", IIO_SHARED_BY_ALL, &mw_stream_iio_channel_reset_tlast_mode_enum),
	MW_STREAM_IIO_ENUM_AVAILABLE("reset_tlast_mode", IIO_SHARED_BY_ALL, &mw_stream_iio_channel_reset_tlast_mode_enum),
	{ },
};

static const struct iio_chan_spec_ext_info mw_stream_iio_ch_ip_info[] = {
	MW_STREAM_IIO_ENUM("reset_ip_mode", IIO_SHARED_BY_ALL, &mw_stream_iio_channel_reset_ip_mode_enum),
	MW_STREAM_IIO_ENUM_AVAILABLE("reset_ip_mode", IIO_SHARED_BY_ALL, &mw_stream_iio_channel_reset_ip_mode_enum),
	{ },
};



static int mw_stream_iio_channel_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		mw_ip_write32(mwchan->mwdev->mw_ip_info, reg & 0xFFFF, writeval);
	} else {
		*readval = mw_ip_read32(mwchan->mwdev->mw_ip_info, reg & 0xFFFF);
	}
	mutex_unlock(&indio_dev->mlock);

	return 0;
}

static const struct iio_info mw_stream_iio_channel_info = {
	.debugfs_reg_access = &mw_stream_iio_channel_reg_access,
};

static int devm_mw_stream_configure_buffer(struct iio_dev *indio_dev)
{
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);
	struct iio_buffer *buffer;
	int status;

	buffer = iio_dmaengine_buffer_alloc(indio_dev->dev.parent, mwchan->dmaname,
			&mw_stream_iio_buffer_dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer)) {
		if(PTR_ERR(buffer) == -EPROBE_DEFER)
			dev_info(&indio_dev->dev, "Deferring probe for DMA engine driver load\n");
		else
			dev_err(&indio_dev->dev, "Failed to allocate IIO DMA buffer: %ld\n", PTR_ERR(buffer));
		return PTR_ERR(buffer);
	}

	status = devm_add_action(indio_dev->dev.parent,(devm_action_fn)iio_dmaengine_buffer_free, buffer);
	if(status){
		iio_dmaengine_buffer_free(buffer);
		return status;
	}

	iio_device_attach_buffer(indio_dev, buffer);

	indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->direction = mwchan->iio_direction;
	indio_dev->setup_ops = &mw_stream_iio_buffer_setup_ops;

	return 0;
}

static int mw_stream_setup_ip_channel(struct iio_dev *indio_dev, struct iio_chan_spec *channel){
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);

	channel->type = IIO_GENERIC_DATA;
	channel->indexed = 1;
	channel->extend_name = devm_kstrdup(&mwchan->dev, "ip_info", GFP_KERNEL);
	if (!channel->extend_name)
		return -ENOMEM;
	channel->ext_info = mw_stream_iio_ch_ip_info;
	channel->scan_index = -ENODEV;

	return 0;
}

static int mw_stream_setup_tlast_channel(struct iio_dev *indio_dev, struct iio_chan_spec *channel){
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);

	channel->type = IIO_GENERIC_DATA;
	channel->indexed = 1;
	channel->extend_name = devm_kstrdup(&mwchan->dev, "tlast_count", GFP_KERNEL);
	if (!channel->extend_name)
		return -ENOMEM;
	channel->ext_info = mw_stream_iio_ch_tlast_info;
	channel->scan_index = -ENODEV;

	return 0;
}

static const char mw_stream_iio_data_channel_compat[] = "mathworks,iio-data-channel-v1.00";

static int mw_stream_count_data_channels(struct iio_dev *indio_dev) {
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);
	struct device_node *data_node;
	int count = 0;
	for_each_child_of_node(mwchan->dev.of_node,data_node) {
		if(of_device_is_compatible(data_node, mw_stream_iio_data_channel_compat))
			count++;
	}
	return count;
}

static int mw_stream_setup_scan_type(struct iio_dev *indio_dev, struct device_node *node, struct iio_chan_spec *channel) {
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);
	int status;
	unsigned int storagebits, realbits, shift;
	char sign;
	const char *fmt;
	status = of_property_read_string(node, "mathworks,data-format", &fmt);
	if(status) {
		dev_err(&mwchan->dev, "Missing data-format specifier for %s\n", node->name);
		return status;
	}
	status = sscanf(fmt, "%c%u/%u>>%u", &sign, &storagebits, &realbits, &shift);

	if (status != 4) {
		dev_err(&mwchan->dev, "Invalid data-format specifier for %s\n", node->name);
		return -EINVAL;
	}
	channel->scan_type.sign = sign;
	channel->scan_type.storagebits = storagebits;
	channel->scan_type.realbits = realbits;
	channel->scan_type.shift = shift;
	return 0;
}

static int mw_stream_setup_data_channels(struct iio_dev *indio_dev){
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);
	struct iio_chan_spec *channel;
	struct device_node *data_node;
	int status;
	u32 scan_index = 0;
	unsigned long *available_scan_masks;


	for_each_child_of_node(mwchan->dev.of_node,data_node) {
		status = of_device_is_compatible(data_node, mw_stream_iio_data_channel_compat);
		if(!status)
			continue;
		status = of_property_read_u32(data_node, "reg", &scan_index);
		if(status){
			dev_err(&mwchan->dev, "Missing 'reg' property in node %s\n", data_node->name);
			return -EINVAL;
		}
		if (scan_index >= mwchan->num_data_chan){
			dev_err(&mwchan->dev, "Invalid 'reg' property in node %s: %d\n", data_node->name, scan_index);
			return -EINVAL;
		}
		channel = (struct iio_chan_spec *)&indio_dev->channels[scan_index];
		if(channel->indexed == 1) {
			dev_err(&mwchan->dev, "Duplicate 'reg' property in node %s: %d\n", data_node->name, scan_index);
			return -EINVAL;
		}
		channel->indexed = 1;
		channel->type = IIO_GENERIC_DATA;
		if (mwchan->iio_direction == IIO_DEVICE_DIRECTION_OUT)
			channel->output = 1;
		channel->channel = scan_index;
		channel->scan_index = scan_index;
		status = of_property_read_string(data_node, "mathworks,chan-name", &channel->extend_name);
		if (status)
			channel->extend_name = NULL;
		status = mw_stream_setup_scan_type(indio_dev, data_node, channel);
		if(status)
			return status;
	}

	/* Only allow all channels or no channels */
	available_scan_masks = devm_kzalloc(&mwchan->dev, sizeof(unsigned long)*2, GFP_KERNEL);
	if(!available_scan_masks)
		return -ENOMEM;
	available_scan_masks[0] = (1 << mwchan->num_data_chan) -1;
	indio_dev->available_scan_masks = available_scan_masks;

	return 0;
}

static void mw_stream_iio_unregister(void *opaque) {
	struct device *dev = opaque;

	/* Unregister the IIO device */
	devres_release_group(dev, mw_stream_iio_unregister);
}

static int devm_mw_stream_iio_register(struct iio_dev *indio_dev) {
	struct mw_stream_iio_chandev *mwchan = iio_priv(indio_dev);
	int status;
	int chIdx = 0;

	if(!devres_open_group(&mwchan->dev, mw_stream_iio_unregister, GFP_KERNEL))
		return -ENOMEM;

	indio_dev->dev.parent = &mwchan->dev;
	indio_dev->name = dev_name(&mwchan->dev);
	indio_dev->info = &mw_stream_iio_channel_info;

	mwchan->num_data_chan = mw_stream_count_data_channels(indio_dev);

	indio_dev->num_channels = mwchan->num_data_chan;
	indio_dev->num_channels++; /* info channel */
	if (mwchan->tlast_cntr_addr != -EINVAL)
		indio_dev->num_channels++;

	indio_dev->channels = devm_kzalloc(&mwchan->dev, (indio_dev->num_channels) * sizeof(struct iio_chan_spec), GFP_KERNEL);
	if(!indio_dev->channels)
		return -ENOMEM;

	status = mw_stream_setup_data_channels(indio_dev);
	if(status)
		return status;
	chIdx += mwchan->num_data_chan;

	status = mw_stream_setup_ip_channel(indio_dev, (struct iio_chan_spec *)&indio_dev->channels[chIdx++]);
	if(status)
		return status;

	if (mwchan->tlast_cntr_addr != -EINVAL) {
		status = mw_stream_setup_tlast_channel(indio_dev, (struct iio_chan_spec *)&indio_dev->channels[chIdx++]);
		if(status)
			return status;
	}

	status = devm_mw_stream_configure_buffer(indio_dev);
	if (status){
		return status;
	}

	status = devm_iio_device_register(&mwchan->dev, indio_dev);
	if(status)
		return status;

	devres_close_group(&mwchan->dev, mw_stream_iio_unregister);

	/* Setup the parent device to tear us down on removal */
	status = devm_add_action(mwchan->dev.parent, mw_stream_iio_unregister, &mwchan->dev);
	if(status){
		mw_stream_iio_unregister(&mwchan->dev);
		return status;
	}

	return 0;
}

/* Nothing to actually do upon release */
static void mw_stream_iio_channel_release(struct device *dev)
{
}

static struct iio_dev *devm_mw_stream_iio_alloc(
		struct mathworks_ipcore_dev *mwdev,
		struct device_node *node,
		struct mw_stream_iio_channel_info *info)
{
	struct iio_dev *indio_dev;
	struct mw_stream_iio_chandev *mwchan;
	const char *devname;
	int status;


	if(!devres_open_group(IP2DEVP(mwdev), devm_mw_stream_iio_alloc, GFP_KERNEL))
		return ERR_PTR(-ENOMEM);

	indio_dev = devm_iio_device_alloc(IP2DEVP(mwdev), sizeof(struct mw_stream_iio_chandev));
	if (!indio_dev){
		dev_err(IP2DEVP(mwdev), "Failed to allocate memory for channel %s\n",node->name);
		return ERR_PTR(-ENOMEM);
	}

	mwchan = iio_priv(indio_dev);
	mwchan->mwdev = mwdev;
	mwchan->iio_direction = info->iio_direction;

	/* Find the name of the DMA channel, there should only be one per node */
	status = of_property_read_string_index(node, "dma-names", 0, &mwchan->dmaname);
	if (status) {
		dev_err(IP2DEVP(mwdev), "Missing dma-names property for node: %s\n",node->name);
		return ERR_PTR(status);
	}
	if (mwchan->iio_direction == IIO_DEVICE_DIRECTION_IN) {
		status = of_property_read_u32(node, "mathworks,sample-cnt-reg", &mwchan->tlast_cntr_addr);
		if(status)
			mwchan->tlast_cntr_addr = -EINVAL;
	} else {
		mwchan->tlast_cntr_addr = -EINVAL;
	}

	device_initialize(&mwchan->dev);

	mwchan->dev.parent = IP2DEVP(mwdev);
	mwchan->dev.of_node = node;
	mwchan->dev.id = ida_simple_get(&mw_stream_iio_channel_ida, 0, 0, GFP_KERNEL);
	if (mwchan->dev.id < 0) {
		return ERR_PTR(mwchan->dev.id);
	}
	status = devm_add_action(IP2DEVP(mwdev),mw_stream_iio_chan_ida_remove, mwchan);
	if(status){
		mw_stream_iio_chan_ida_remove(mwchan);
		return ERR_PTR(status);
	}
	mwchan->dev.release = mw_stream_iio_channel_release;
	/* clone the parent's DMA config */
	memcpy(&mwchan->dev.archdata, &IP2DEVP(mwdev)->archdata, sizeof(struct dev_archdata));
	mwchan->dev.coherent_dma_mask = IP2DEVP(mwdev)->coherent_dma_mask;
	mwchan->dev.dma_mask = IP2DEVP(mwdev)->dma_mask;
	mwchan->dev.dma_pfn_offset = IP2DEVP(mwdev)->dma_pfn_offset;


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

	devres_close_group(IP2DEVP(mwdev), devm_mw_stream_iio_alloc);

	return indio_dev;
}

static int mw_stream_iio_channel_probe(
		struct mathworks_ipcore_dev *mwdev,
		struct device_node *node,
		struct mw_stream_iio_channel_info *info)
{
	int status;
	struct iio_dev *indio_dev;

	indio_dev = devm_mw_stream_iio_alloc(mwdev, node, info);
	if (IS_ERR(indio_dev))
		return PTR_ERR(indio_dev);

	status = devm_mw_stream_iio_register(indio_dev);
	if (status)
		return status;

	return 0;
}

static struct mw_stream_iio_channel_info mw_stream_iio_mm2s_info = {
	.iio_direction = IIO_DEVICE_DIRECTION_OUT,
};

static struct mw_stream_iio_channel_info mw_stream_iio_s2mm_info = {
	.iio_direction = IIO_DEVICE_DIRECTION_IN,
};

static const struct of_device_id mw_stream_iio_channel_of_match[] = {
	{ .compatible = "mathworks,axi4stream-mm2s-channel-v1.00", .data = &mw_stream_iio_mm2s_info},
	{ .compatible = "mathworks,axi4stream-s2mm-channel-v1.00", .data = &mw_stream_iio_s2mm_info},
    {},
};

int mw_stream_iio_channels_probe(struct mathworks_ipcore_dev *mwdev)
{
	int status;

	struct device_node *child;
	const struct of_device_id *match;


	for_each_child_of_node(IP2DEVP(mwdev)->of_node,child) {
		match = of_match_node(mw_stream_iio_channel_of_match, child);
		if(match){
			status = mw_stream_iio_channel_probe(mwdev, child, (struct mw_stream_iio_channel_info *)match->data);
			if(status)
				return status;
		}
	}

	return 0;
}

EXPORT_SYMBOL_GPL(mw_stream_iio_channels_probe);

static int __init mw_stream_iio_channel_init(void)
{
	return 0;
}

static void __exit mw_stream_iio_channel_exit(void)
{

}

module_init(mw_stream_iio_channel_init);
module_exit(mw_stream_iio_channel_exit);

MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MathWorks Streaming IIO Channel");
MODULE_ALIAS(DRIVER_NAME);
