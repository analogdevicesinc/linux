/*
 * MathWorks Shared Memory Channel
 *
 * Copyright 2019 The MathWorks, Inc
 *
 * Licensed under the GPL-2.
 */

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/idr.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/mathworks/mathworks_ip.h>

#include "mw_sharedmem_iio_channel.h"
#include "mathworks_ipcore.h"

static DEFINE_IDA(mw_sharedmem_iio_channel_ida);
static DEFINE_IDA(mw_sharedmem_region_ida);

#define MWDEV_TO_MWIP(mwdev)    (mwdev->mw_ip_info)
#define IP2DEVP(mwdev)  (MWDEV_TO_MWIP(mwdev)->dev)

#define MW_SHAREDMEM_IIO_ENUM IIO_ENUM
#define MW_SHAREDMEM_IIO_ENUM_AVAILABLE(_name, _shared_by, _e) \
{ \
	.name = (_name "_available"), \
	.shared = (_shared_by), \
	.read = iio_enum_available_read, \
	.private = (uintptr_t)(_e), \
}

#define MW_IRQ_ACK_SET		(1)
#define MW_IRQ_ACK_CLEAR	(2)
#define MW_IRQ_ACK_SET_CLEAR	(4)

enum mw_sharedmem_iio_chan_type {
	MW_SHAREDMEM_CHAN_TYPE_READ = 0,
	MW_SHAREDMEM_CHAN_TYPE_WRITE,
};

struct mw_sharedmem_iio_channel_info {
	enum iio_device_direction iio_direction;
};

struct mw_sharedmem_region {
	void		*virt;
	phys_addr_t	phys;
	size_t		size;
	int		read_count;	 // count of active readers
	struct mutex	read_count_lock; // protects read_count, only used by readers
	struct mutex	lock;		 // ensures mutual exclusion of writers
};

struct mw_sharedmem_region_dev {
	struct mathworks_ipcore_dev	*mwdev;
	struct device			dev;
	const char			*name;
	int				rd_base_reg; // IP core AXI Master Read Base Address 
	int				wr_base_reg; // IP core AXI Master Write Base Address 
	struct mw_sharedmem_region	region;
};

/* Setting to determine synchronization with IP core */
enum mw_sharedmem_iio_ip_sync_mode {
	MW_SHAREDMEM_IP_SYNC_MODE_NONE = 0,
	MW_SHAREDMEM_IP_SYNC_MODE_INTERRUPT,
};

/* Setting to determine whether to automatically set the 
   IP core AXI Master Read/Write Base Address register */
enum mw_sharedmem_iio_base_addr_mode {
	MW_SHAREDMEM_BASE_ADDR_MODE_AUTO = 0,
	MW_SHAREDMEM_BASE_ADDR_MODE_MANUAL,
};

struct mw_sharedmem_iio_chandev {
	struct mathworks_ipcore_dev		*mwdev;
	struct mw_sharedmem_region_dev		*mwregion;
	struct device				dev;
	struct mw_sharedmem_region		*region;
	enum mw_sharedmem_iio_chan_type		type;
	size_t					offset;
	struct mutex				lock;
	int					irq;
	int					irq_count;
	int					irq_ack_reg;
	int					irq_ack_mask;
	int					irq_ack_op;
	enum mw_sharedmem_iio_base_addr_mode	base_addr_mode;
	enum mw_sharedmem_iio_ip_sync_mode	ip_sync_mode;
};

struct mw_sharedmem_buffer {
	struct iio_buffer		buffer;
	struct mw_sharedmem_iio_chandev	*mwchan;
	bool				enabled;
	struct mutex			lock;
};

/***************************
 *         Buffer 
 ***************************/
static struct mw_sharedmem_buffer *buffer_to_mw_sharedmem_buffer(struct iio_buffer *buffer)
{
	return container_of(buffer, struct mw_sharedmem_buffer, buffer);
}

static int mw_sharedmem_buffer_write(struct iio_buffer *buffer, size_t n,
	const char __user *user_buffer)
{
	struct mw_sharedmem_buffer *sharedmem_buff = buffer_to_mw_sharedmem_buffer(buffer);
	struct mw_sharedmem_iio_chandev *mwchan = sharedmem_buff->mwchan;
	struct mw_sharedmem_region *region = mwchan->region;
	
	size_t offset;
	int ret;

	if (n < buffer->bytes_per_datum)
		return -EINVAL;

	mutex_lock(&mwchan->lock);
	offset = mwchan->offset;
	mutex_unlock(&mwchan->lock);
	
	n = ALIGN(n, buffer->bytes_per_datum);
	
	/* Only handle exact buffer size at end of region */
	if (n > region->size - offset)
		return -EFAULT;

	mutex_lock(&region->lock);
	if (copy_from_user(region->virt + offset, user_buffer, n)) {
		ret = -EFAULT;
		goto out_unlock;
	}
	
	ret = n;

	if (mwchan->irq > 0) {
		mutex_lock(&mwchan->lock);
		if (mwchan->irq_count > 0)
			mwchan->irq_count--;
		mutex_unlock(&mwchan->lock);
	}
	
out_unlock:
	mutex_unlock(&region->lock);

	return ret;
}

static int mw_sharedmem_buffer_read(struct iio_buffer *buffer, size_t n, 
	char __user *user_buffer)
{
	struct mw_sharedmem_buffer *sharedmem_buff = buffer_to_mw_sharedmem_buffer(buffer);
	struct mw_sharedmem_iio_chandev *mwchan = sharedmem_buff->mwchan;
	struct mw_sharedmem_region *region = mwchan->region;
	
	size_t offset;
	int ret;
	
	if (n < buffer->bytes_per_datum)
		return -EINVAL;
	
	mutex_lock(&mwchan->lock);
	offset = mwchan->offset;
	mutex_unlock(&mwchan->lock);

	n = rounddown(n, buffer->bytes_per_datum);
	
	/* Only handle exact buffer size at end of region */
	if (n > region->size - offset)
		return -EFAULT;
	
	/* Read-prioritized locking */
	mutex_lock(&region->read_count_lock);
	region->read_count++;
	if (region->read_count == 1)
		mutex_lock(&region->lock);
	mutex_unlock(&region->read_count_lock);
	
	if (copy_to_user(user_buffer, region->virt + offset, n)) {
		ret = -EFAULT;
		goto out_unlock;
	}
	
	ret = n;
	
	if (mwchan->irq > 0) {
		mutex_lock(&mwchan->lock);
		if (mwchan->irq_count > 0)
			mwchan->irq_count--;
		mutex_unlock(&mwchan->lock);
	}
	
out_unlock:
	mutex_lock(&region->read_count_lock);
	region->read_count--;
	if (region->read_count == 0)
		mutex_unlock(&region->lock);
	mutex_unlock(&region->read_count_lock);
	
	return ret;
}

int mw_sharedmem_buffer_set_bytes_per_datum(struct iio_buffer *buffer, size_t bpd)
{
	buffer->bytes_per_datum = bpd;
	return 0;
}

static int mw_sharedmem_buffer_set_length(struct iio_buffer *buffer, unsigned int length)
{
	struct mw_sharedmem_buffer *sharedmem_buff = buffer_to_mw_sharedmem_buffer(buffer);
	struct mw_sharedmem_region *region = sharedmem_buff->mwchan->region;
	if (length < 1)
		length = 1;
	if (length > region->size)
		length = region->size;
	buffer->length = length;
	return 0;
}

static int mw_sharedmem_buffer_enable(struct iio_buffer *buffer, struct iio_dev *indio_dev)
{
	struct mw_sharedmem_buffer *sharedmem_buff = buffer_to_mw_sharedmem_buffer(buffer);
	mutex_lock(&sharedmem_buff->lock);
	sharedmem_buff->enabled = true;
	mutex_unlock(&sharedmem_buff->lock);
	return 0;
}

static int mw_sharedmem_buffer_disable(struct iio_buffer *buffer, struct iio_dev *indio_dev)
{
	struct mw_sharedmem_buffer *sharedmem_buff = buffer_to_mw_sharedmem_buffer(buffer);
	mutex_lock(&sharedmem_buff->lock);
	sharedmem_buff->enabled = false;
	mutex_unlock(&sharedmem_buff->lock);
	return 0;
}

static size_t mw_sharedmem_buffer_data_available(struct iio_buffer *buffer)
{
	struct mw_sharedmem_buffer *sharedmem_buff = buffer_to_mw_sharedmem_buffer(buffer);
	struct mw_sharedmem_iio_chandev *mwchan = sharedmem_buff->mwchan;
	struct mw_sharedmem_region *region = mwchan->region;

	size_t size;
	
	mutex_lock(&mwchan->lock);
	
	if ((mwchan->ip_sync_mode == MW_SHAREDMEM_IP_SYNC_MODE_INTERRUPT) && (mwchan->irq > 0)) {
		size = mwchan->irq_count ? region->size : 0;
	} else {
		size = region->size;
	}
	
	mutex_unlock(&mwchan->lock);
	
	return size;
}

static bool mw_sharedmem_buffer_space_available(struct iio_buffer *buffer)
{
	struct mw_sharedmem_buffer *sharedmem_buff = buffer_to_mw_sharedmem_buffer(buffer);
	struct mw_sharedmem_iio_chandev *mwchan = sharedmem_buff->mwchan;
	bool space_available;
	
	mutex_lock(&mwchan->lock);

	if ((mwchan->ip_sync_mode == MW_SHAREDMEM_IP_SYNC_MODE_INTERRUPT) && (mwchan->irq > 0)) {
		space_available = (mwchan->irq_count > 0);
	} else {
		space_available = true;
	}
	
	mutex_unlock(&mwchan->lock);

	return space_available;
}

static void mw_sharedmem_buffer_release(struct iio_buffer *buffer)
{
	struct mw_sharedmem_buffer *sharedmem_buff = buffer_to_mw_sharedmem_buffer(buffer);
	mutex_destroy(&sharedmem_buff->lock);
}

static const struct iio_buffer_access_funcs mw_sharedmem_access_func = {
	.read 			= mw_sharedmem_buffer_read,
	.write 			= mw_sharedmem_buffer_write,
	.set_bytes_per_datum 	= mw_sharedmem_buffer_set_bytes_per_datum,
	.set_length 		= mw_sharedmem_buffer_set_length,
	.enable 		= mw_sharedmem_buffer_enable,
	.disable 		= mw_sharedmem_buffer_disable,
	.data_available 	= mw_sharedmem_buffer_data_available,
	.space_available 	= mw_sharedmem_buffer_space_available,
	.release 		= mw_sharedmem_buffer_release,

	.modes = INDIO_BUFFER_HARDWARE,
};

static int mw_sharedmem_buffer_init(struct mw_sharedmem_buffer *sharedmem_buff,
	struct mw_sharedmem_iio_chandev *mwchan)
{
	iio_buffer_init(&sharedmem_buff->buffer);
	sharedmem_buff->buffer.access = &mw_sharedmem_access_func;
	sharedmem_buff->mwchan = mwchan;
	sharedmem_buff->enabled = false;
	mutex_init(&sharedmem_buff->lock);
	return 0;
}

static struct iio_buffer *mw_sharedmem_buffer_alloc(struct device *dev, 
	struct mw_sharedmem_iio_chandev *mwchan)
{
	struct mw_sharedmem_buffer *sharedmem_buff;
	
	sharedmem_buff = devm_kzalloc(dev, sizeof(*sharedmem_buff), GFP_KERNEL);
	if (!sharedmem_buff)
		return ERR_PTR(-ENOMEM);
	
	mw_sharedmem_buffer_init(sharedmem_buff, mwchan);
	
	return &sharedmem_buff->buffer;
}

static void mw_sharedmem_buffer_free(struct iio_buffer *buffer)
{
	iio_buffer_put(buffer);
}

/*************
 * IRQ 
 *************/
 
static void mw_sharedmem_irq_ack(struct mw_sharedmem_iio_chandev *mwchan)
{
	uint32_t val;
	
	if (mwchan->irq_ack_reg < 0)
		return;
	
	val = mw_ip_read32(mwchan->mwdev->mw_ip_info, mwchan->irq_ack_reg);

	/* First write: SET or CLEAR */
	if (mwchan->irq_ack_op & (MW_IRQ_ACK_SET | MW_IRQ_ACK_SET_CLEAR)) {
		mw_ip_write32(mwchan->mwdev->mw_ip_info, mwchan->irq_ack_reg, val | mwchan->irq_ack_mask);
	} else {
		mw_ip_write32(mwchan->mwdev->mw_ip_info, mwchan->irq_ack_reg, val & ~mwchan->irq_ack_mask);
	}

	/* Second write: CLEAR after SET */
	if (mwchan->irq_ack_op & MW_IRQ_ACK_SET_CLEAR) {
		mw_ip_write32(mwchan->mwdev->mw_ip_info, mwchan->irq_ack_reg, val & ~mwchan->irq_ack_mask);
	}
	
}

static irqreturn_t mw_sharedmem_irq_handler(int irq, void *data)
{
	struct iio_dev *indio_dev = data;
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);
	
	/* Ack the interrupt */
	mw_sharedmem_irq_ack(mwchan);
	
	mutex_lock(&mwchan->lock);
	mwchan->irq_count++;
	mutex_unlock(&mwchan->lock);
	
	wake_up(&indio_dev->buffer->pollq);
	
	return IRQ_HANDLED;
}

/*************
 * IP Sync Modes
 *************/
static const char * const mw_sharedmem_iio_channel_ip_sync_modes[] = { "none", "interrupt" };

static int mw_sharedmem_iio_channel_get_ip_sync_mode(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan)
{
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);
	int mode;
	
	mutex_lock(&indio_dev->mlock);
	mode = mwchan->ip_sync_mode;
	mutex_unlock(&indio_dev->mlock);
	
	return mode;
}

static int mw_sharedmem_iio_channel_set_ip_sync_mode(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan, unsigned int mode)
{
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);

	if ((mode == MW_SHAREDMEM_IP_SYNC_MODE_INTERRUPT) && (mwchan->irq < 0)) {
		return -EINVAL;
	}
	
	mutex_lock(&indio_dev->mlock);
	mwchan->ip_sync_mode = mode;
	mutex_unlock(&indio_dev->mlock);

	return 0;
}

static const struct iio_enum mw_sharedmem_iio_channel_ip_sync_mode_enum = {
	.items = mw_sharedmem_iio_channel_ip_sync_modes,
	.num_items = ARRAY_SIZE(mw_sharedmem_iio_channel_ip_sync_modes),
	.get = mw_sharedmem_iio_channel_get_ip_sync_mode,
	.set = mw_sharedmem_iio_channel_set_ip_sync_mode,
};

/*************
 * IP Base Address Register Modes
 *************/
static const char * const mw_sharedmem_iio_channel_base_addr_modes[] = { "auto", "manual" };

static int mw_sharedmem_iio_channel_get_base_addr_mode(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan)
{
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);
	int mode;
	
	mutex_lock(&indio_dev->mlock);
	mode = mwchan->base_addr_mode;
	mutex_unlock(&indio_dev->mlock);
	
	return mode;
}

static int mw_sharedmem_iio_channel_set_base_addr_mode(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan, unsigned int mode)
{
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	mwchan->base_addr_mode = mode;
	mutex_unlock(&indio_dev->mlock);

	return 0;
}

static const struct iio_enum mw_sharedmem_iio_channel_base_addr_mode_enum = {
	.items = mw_sharedmem_iio_channel_base_addr_modes,
	.num_items = ARRAY_SIZE(mw_sharedmem_iio_channel_base_addr_modes),
	.get = mw_sharedmem_iio_channel_get_base_addr_mode,
	.set = mw_sharedmem_iio_channel_set_base_addr_mode,
};

static const struct iio_chan_spec_ext_info mw_sharedmem_iio_ch_ip_info[] = {
	MW_SHAREDMEM_IIO_ENUM("base_addr_mode", IIO_SHARED_BY_ALL, &mw_sharedmem_iio_channel_base_addr_mode_enum),
	MW_SHAREDMEM_IIO_ENUM_AVAILABLE("base_addr_mode", IIO_SHARED_BY_ALL, &mw_sharedmem_iio_channel_base_addr_mode_enum),
	MW_SHAREDMEM_IIO_ENUM("ip_sync_mode", IIO_SHARED_BY_ALL, &mw_sharedmem_iio_channel_ip_sync_mode_enum),
	MW_SHAREDMEM_IIO_ENUM_AVAILABLE("ip_sync_mode", IIO_SHARED_BY_ALL, &mw_sharedmem_iio_channel_ip_sync_mode_enum),
	{ },
};

/***************************
 *     IIO channel dev 
 ***************************/
 
 static void mw_sharedmem_config_ipcore_reg(struct mw_sharedmem_iio_chandev *mwchan)
{
	uint32_t base_addr32;

	base_addr32 = (uint32_t)((size_t)mwchan->region->phys & 0xFFFFFFFF);
	
	if (mwchan->mwregion->rd_base_reg >= 0) {
		/* Set the IP core's AXI Master Read Base Address */
		mw_ip_write32(mwchan->mwdev->mw_ip_info, mwchan->mwregion->rd_base_reg, base_addr32);
	}
	if (mwchan->mwregion->wr_base_reg >= 0) {
		/* Set the IP core's AXI Master Write Base Address */
		mw_ip_write32(mwchan->mwdev->mw_ip_info, mwchan->mwregion->wr_base_reg, base_addr32);
	}
}
 
static void mw_sharedmem_iio_chan_ida_remove(void *opaque){
	struct mw_sharedmem_iio_chandev* mwchan = opaque;
	ida_simple_remove(&mw_sharedmem_iio_channel_ida, mwchan->dev.id);
}
static void mw_sharedmem_region_ida_remove(void *opaque){
	struct mw_sharedmem_region_dev* mwregion = opaque;
	ida_simple_remove(&mw_sharedmem_region_ida, mwregion->dev.id);
}

static int mw_sharedmem_iio_buffer_preenable(struct iio_dev *indio_dev)
{
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);

	if (mwchan->base_addr_mode == MW_SHAREDMEM_BASE_ADDR_MODE_AUTO) {
		mw_sharedmem_config_ipcore_reg(mwchan);
	}
	
	return 0;
}

static const struct iio_buffer_setup_ops mw_sharedmem_iio_buffer_setup_ops = {

	.preenable = &mw_sharedmem_iio_buffer_preenable,
};

static const struct iio_info mw_sharedmem_iio_chandev_info;

static int devm_mw_sharedmem_configure_buffer(struct iio_dev *indio_dev)
{
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);
	struct iio_buffer *buffer;
	int status;
	
	buffer = mw_sharedmem_buffer_alloc(indio_dev->dev.parent, mwchan);
	if (IS_ERR(buffer)) {
		dev_err(&indio_dev->dev, "Failed to configure IIO buffer: %ld\n", PTR_ERR(buffer));
		return PTR_ERR(buffer);
	}

	status = devm_add_action(indio_dev->dev.parent,(devm_action_fn)mw_sharedmem_buffer_free, buffer);
	if(status){
		mw_sharedmem_buffer_free(buffer);
		return status;
	}

	iio_device_attach_buffer(indio_dev, buffer);

	indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &mw_sharedmem_iio_buffer_setup_ops;

	return 0;
}

static int devm_mw_sharedmem_configure_irq(struct iio_dev *indio_dev)
{
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);
	int status;

	if (mwchan->irq <= 0)
		return 0;
	
	status = devm_request_irq(&mwchan->dev, mwchan->irq, mw_sharedmem_irq_handler,
			0, dev_name(&mwchan->dev), indio_dev);
	if (status) {
		dev_err(&indio_dev->dev, "Failed to request IRQ %d\n", mwchan->irq);
		return status;
	}

	return 0;
}

static int mw_sharedmem_setup_ip_channel(struct iio_dev *indio_dev, struct iio_chan_spec *channel){
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);

	channel->type = IIO_GENERIC_DATA;
	channel->indexed = 1;
	channel->extend_name = devm_kstrdup(&mwchan->dev, "ip_info", GFP_KERNEL);
	if (!channel->extend_name)
		return -ENOMEM;
	channel->ext_info = mw_sharedmem_iio_ch_ip_info;
	channel->scan_index = -ENODEV;

	return 0;
}

static int mw_sharedmem_setup_data_channel(struct iio_dev *indio_dev, struct iio_chan_spec *channel) {
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);
	int status;
	unsigned long *available_scan_masks;

	channel->indexed = 1;
	channel->type = IIO_GENERIC_DATA;
	if (indio_dev->direction == IIO_DEVICE_DIRECTION_OUT)
		channel->output = 1;
	channel->channel = 0;
	channel->scan_index = 0;
	status = of_property_read_string(mwchan->dev.of_node, "mathworks,chan-name", &channel->extend_name);
	if (status)
		channel->extend_name = NULL;
	
	/* Set scan type to unsigned byte */
	channel->scan_type.sign = 'u';
	channel->scan_type.storagebits = 8;
	channel->scan_type.realbits = 8;
	channel->scan_type.shift = 0;
	
	available_scan_masks = devm_kzalloc(&mwchan->dev, sizeof(unsigned long)*2, GFP_KERNEL);
	if(!available_scan_masks)
		return -ENOMEM;
	available_scan_masks[0] = 1;
	indio_dev->available_scan_masks = available_scan_masks;

	return 0;
}

static ssize_t mw_sharedmem_iio_channel_get_offset(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, char *buf)
{
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);
	unsigned long long offset;
	
	mutex_lock(&mwchan->lock);
	offset = (unsigned long long)mwchan->offset;
	mutex_unlock(&mwchan->lock);
	
	return snprintf(buf, PAGE_SIZE, "%llu\n", offset);
}

static ssize_t mw_sharedmem_iio_channel_set_offset(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, const char *buf, size_t len)
{
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);
	unsigned long long result;
	ssize_t ret;
	
	ret = kstrtoull(buf, 0, &result);
	if (ret)
		return ret;
	
	if (result >= mwchan->region->size) {
		return -EINVAL;
	}

	mutex_lock(&mwchan->lock);
	mwchan->offset = (ssize_t)result;
	mutex_unlock(&mwchan->lock);
	
	return len;
}

static const struct iio_chan_spec_ext_info mw_sharedmem_iio_offset_channel_info = {
	.name = "offset",
	.shared = IIO_SHARED_BY_ALL,
	.read = mw_sharedmem_iio_channel_get_offset,
	.write = mw_sharedmem_iio_channel_set_offset,
};

static int mw_sharedmem_setup_offset_channel(struct iio_dev *indio_dev, struct iio_chan_spec *channel) {
	
	channel->type = IIO_GENERIC_DATA;
	channel->indexed = 1;
	channel->ext_info = &mw_sharedmem_iio_offset_channel_info;
	channel->scan_index = -ENODEV;

	return 0;
}
        
static void mw_sharedmem_iio_unregister(void *opaque) {
	struct device *dev = opaque;

	/* Unregister the IIO device */
	devres_release_group(dev, mw_sharedmem_iio_unregister);
}

static int devm_mw_sharedmem_iio_register(struct iio_dev *indio_dev, enum iio_device_direction direction) {
	struct mw_sharedmem_iio_chandev *mwchan = iio_priv(indio_dev);
	int status;

	if(!devres_open_group(&mwchan->dev, mw_sharedmem_iio_unregister, GFP_KERNEL))
		return -ENOMEM;

	indio_dev->dev.parent = &mwchan->dev;
	indio_dev->name = dev_name(&mwchan->dev);
	indio_dev->info = &mw_sharedmem_iio_chandev_info;
	indio_dev->direction = direction;
	indio_dev->num_channels = 3; // data, offset, ip

	indio_dev->channels = devm_kzalloc(&mwchan->dev, (indio_dev->num_channels) * sizeof(struct iio_chan_spec), GFP_KERNEL);
	if(!indio_dev->channels)
		return -ENOMEM;
	
	status = mw_sharedmem_setup_data_channel(indio_dev, (struct iio_chan_spec *)&indio_dev->channels[0]);
	if(status)
		return status;

	status = mw_sharedmem_setup_offset_channel(indio_dev, (struct iio_chan_spec *)&indio_dev->channels[1]);
	if(status)
		return status;
	
	status = mw_sharedmem_setup_ip_channel(indio_dev, (struct iio_chan_spec *)&indio_dev->channels[2]);
	if(status)
		return status;

	status = devm_mw_sharedmem_configure_buffer(indio_dev);
	if (status){
		return status;
	}

	status = devm_mw_sharedmem_configure_irq(indio_dev);
	if (status){
		return status;
	}

	status = devm_iio_device_register(&mwchan->dev, indio_dev);
	if(status)
		return status;

	devres_close_group(&mwchan->dev, mw_sharedmem_iio_unregister);

	/* Setup the parent device to tear us down on removal */
	status = devm_add_action(mwchan->dev.parent, mw_sharedmem_iio_unregister, &mwchan->dev);
	if(status){
		mw_sharedmem_iio_unregister(&mwchan->dev);
		return status;
	}

	return 0;
}

static void mw_sharedmem_iio_channel_release(struct device *dev)
{
	struct mw_sharedmem_iio_chandev *mwchan = (struct mw_sharedmem_iio_chandev *)dev->driver_data;
	mutex_destroy(&mwchan->lock);
	if (mwchan->irq > 0)
		free_irq(mwchan->irq, mwchan);
}

static struct iio_dev *devm_mw_sharedmem_iio_alloc(
		struct mw_sharedmem_region_dev *mwregion,
		enum iio_device_direction direction,
		struct device_node *node)		
{
	struct iio_dev *indio_dev;
	struct mathworks_ipcore_dev *mwdev = mwregion->mwdev;
	struct mw_sharedmem_iio_chandev *mwchan;
	const char *devname;
	int irq_ack_info[3];
	int status;

	if(!devres_open_group(IP2DEVP(mwdev), devm_mw_sharedmem_iio_alloc, GFP_KERNEL))
		return ERR_PTR(-ENOMEM);

	indio_dev = devm_iio_device_alloc(IP2DEVP(mwdev), sizeof(struct mw_sharedmem_iio_chandev));
	if (!indio_dev){
		dev_err(IP2DEVP(mwdev), "Failed to allocate memory for channel %s\n",node->name);
		return ERR_PTR(-ENOMEM);
	}

	mwchan = iio_priv(indio_dev);
	mwchan->mwdev = mwdev;
	mwchan->mwregion = mwregion;
	if (direction == IIO_DEVICE_DIRECTION_OUT) {
		mwchan->type = MW_SHAREDMEM_CHAN_TYPE_WRITE;
	} else {
		mwchan->type = MW_SHAREDMEM_CHAN_TYPE_READ;
	}
	mwchan->region = &mwregion->region;
	mwchan->offset = 0;
	mutex_init(&mwchan->lock);
	
	/* look for IRQ */
	mwchan->irq = of_irq_get(node, 0);
	
	if (mwchan->irq == 0) {
		return ERR_PTR(-ENOENT);
	}
	if (mwchan->irq > 0) {
		status = of_property_read_u32_array(node, "mathworks,irq-ack-info", &irq_ack_info[0], 3);
		if(status) {
			mwchan->irq_ack_reg = -EINVAL;
			mwchan->irq_ack_mask = -EINVAL;
			mwchan->irq_ack_op = -EINVAL;
		} else {
			mwchan->irq_ack_reg = irq_ack_info[0];
			mwchan->irq_ack_mask = irq_ack_info[1];
			mwchan->irq_ack_op = irq_ack_info[2];
		}
	}
	mwchan->irq_count = 0;
	
	mwchan->base_addr_mode = MW_SHAREDMEM_BASE_ADDR_MODE_MANUAL;
	mwchan->ip_sync_mode = MW_SHAREDMEM_IP_SYNC_MODE_NONE;
	
	device_initialize(&mwchan->dev);

	mwchan->dev.parent = IP2DEVP(mwdev);
	mwchan->dev.driver_data = (void*)mwchan;
	mwchan->dev.of_node = node;
	mwchan->dev.id = ida_simple_get(&mw_sharedmem_iio_channel_ida, 0, 0, GFP_KERNEL);
	if (mwchan->dev.id < 0) {
		return ERR_PTR(mwchan->dev.id);
	}
	status = devm_add_action(IP2DEVP(mwdev),mw_sharedmem_iio_chan_ida_remove, mwchan);
	if(status){
		mw_sharedmem_iio_chan_ida_remove(mwchan);
		return ERR_PTR(status);
	}
	mwchan->dev.release = mw_sharedmem_iio_channel_release;
	/* clone the parent's DMA config */
	memcpy(&mwchan->dev.archdata, &IP2DEVP(mwdev)->archdata, sizeof(struct dev_archdata));
	mwchan->dev.coherent_dma_mask = IP2DEVP(mwdev)->coherent_dma_mask;
	mwchan->dev.dma_mask = IP2DEVP(mwdev)->dma_mask;

	status = of_property_read_string(node, "mathworks,dev-name", &devname);
	if (!status) {
		/* Use the specified channel name */
		status = dev_set_name(&mwchan->dev, "%s:%s", mwregion->name, devname);
	} else {
		/* Use the node name + dev ID */
		status = dev_set_name(&mwchan->dev, "%s:%s%d", mwregion->name, node->name, mwchan->dev.id);
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

	devres_close_group(IP2DEVP(mwdev), devm_mw_sharedmem_iio_alloc);

	return indio_dev;
}

static int mw_sharedmem_iio_channel_probe(
		struct mw_sharedmem_region_dev *mwregion,
		struct device_node *node,
		enum iio_device_direction direction)
{
	int status;
	struct iio_dev *indio_dev;

	indio_dev = devm_mw_sharedmem_iio_alloc(mwregion, direction, node);
	if (IS_ERR(indio_dev))
		return PTR_ERR(indio_dev);
	
	status = devm_mw_sharedmem_iio_register(indio_dev, direction);
	if (status)
		return status;

	return 0;
}

static struct mw_sharedmem_iio_channel_info mw_sharedmem_iio_write_info = {
	.iio_direction = IIO_DEVICE_DIRECTION_OUT,
};

static struct mw_sharedmem_iio_channel_info mw_sharedmem_iio_read_info = {
	.iio_direction = IIO_DEVICE_DIRECTION_IN,
};

static const struct of_device_id mw_sharedmem_iio_channel_of_match[] = {
	{ .compatible = "mathworks,sharedmem-write-channel-v1.00", .data = &mw_sharedmem_iio_write_info},
	{ .compatible = "mathworks,sharedmem-read-channel-v1.00", .data = &mw_sharedmem_iio_read_info},
    {},
};

static int mw_sharedmem_count_iio_channels(struct device_node *node) {
	struct device_node *child;
	const struct of_device_id *match;
	int count = 0;
	for_each_child_of_node(node, child) {
		match = of_match_node(mw_sharedmem_iio_channel_of_match, child);
		if(match)
			count++;
	}
	return count;
}



/***************************
 *     Memory region dev 
 ***************************/

static int mw_sharedmem_region_init(struct mw_sharedmem_region *region, struct resource *r)
{
	region->phys = (phys_addr_t)r->start;
	region->size =  (size_t)resource_size(r);
	region->virt = memremap(region->phys, region->size, MEMREMAP_WC);
	if (region->virt == NULL) {
		return -ENOMEM;
	}
	region->read_count = 0;
	mutex_init(&region->read_count_lock);
	mutex_init(&region->lock);
	return 0;
}

static void mw_sharedmem_region_release(struct device *dev)
{
	struct mw_sharedmem_region_dev *mwregion = (struct mw_sharedmem_region_dev *)dev->driver_data;
	mutex_destroy(&mwregion->region.lock);
}

static struct mw_sharedmem_region_dev *devm_mw_sharedmem_region_alloc(
		struct mathworks_ipcore_dev *mwdev,
		struct device_node *node)
{
	struct mw_sharedmem_region_dev *mwregion;
	struct device_node *np;
	struct resource r;
	const char *devname;
	int status;
	
	if(!devres_open_group(IP2DEVP(mwdev), devm_mw_sharedmem_region_alloc, GFP_KERNEL))
		return ERR_PTR(-ENOMEM);

	/* Find reserved memory region node */
	np = of_parse_phandle(node, "memory-region", 0);
	if (!np) {
		dev_err(IP2DEVP(mwdev), "Missing memory-region property for node: %s\n", node->name);
		return ERR_PTR(-ENODEV);
	}
	
	/* Get the address assigned to the memory region */
	status = of_address_to_resource(np, 0, &r);
	if (status) {
		dev_err(IP2DEVP(mwdev), "No memory address assigned to region: %s\n",np->name);
		return ERR_PTR(status);
	}
	
	mwregion = devm_kzalloc(IP2DEVP(mwdev),sizeof(*mwregion), GFP_KERNEL);
	if (!mwregion) {
		dev_err(IP2DEVP(mwdev), "Failed to allocate memory for shared memory region %s\n",np->name);
		return ERR_PTR(-ENOMEM);
	}
	
	status = mw_sharedmem_region_init(&mwregion->region, &r);
	if (status) {
		dev_err(IP2DEVP(mwdev), "Failed to initialize shared memory region\n");
		return ERR_PTR(status);
	}

	mwregion->mwdev = mwdev;
	
	status = of_property_read_u32(node, "mathworks,rd-base-reg", &mwregion->rd_base_reg);
	if(status)
		mwregion->rd_base_reg = -EINVAL;
	
	status = of_property_read_u32(node, "mathworks,wr-base-reg", &mwregion->wr_base_reg);
	if(status)
		mwregion->wr_base_reg = -EINVAL;
	
	device_initialize(&mwregion->dev);
	
	mwregion->dev.parent = IP2DEVP(mwdev);
	mwregion->dev.driver_data = (void*)mwregion;
	mwregion->dev.of_node = node;
	mwregion->dev.id = ida_simple_get(&mw_sharedmem_region_ida, 0, 0, GFP_KERNEL);
	if (mwregion->dev.id < 0) {
		return ERR_PTR(mwregion->dev.id);
	}
	status = devm_add_action(IP2DEVP(mwdev),mw_sharedmem_region_ida_remove, mwregion);
	if(status){
		mw_sharedmem_region_ida_remove(mwregion);
		return ERR_PTR(status);
	}
	mwregion->dev.release = mw_sharedmem_region_release;
	/* clone the parent's DMA config */
	memcpy(&mwregion->dev.archdata, &IP2DEVP(mwdev)->archdata, sizeof(struct dev_archdata));
	mwregion->dev.coherent_dma_mask = IP2DEVP(mwdev)->coherent_dma_mask;
	mwregion->dev.dma_mask = IP2DEVP(mwdev)->dma_mask;
	
	status = of_property_read_string(node, "mathworks,dev-name", &devname);
	if (!status) {
		/* Use the specified device name */
		status = dev_set_name(&mwregion->dev, "%s:%s", dev_name(mwregion->mwdev->mw_ip_info->char_device), devname);
	} else {
		/* Use the node name + dev ID */
		status = dev_set_name(&mwregion->dev, "%s:%s%d", dev_name(mwregion->mwdev->mw_ip_info->char_device), node->name, mwregion->dev.id);
	}
	if (status)
		return ERR_PTR(status);
	
	mwregion->name = dev_name(&mwregion->dev);
	
	status = device_add(&mwregion->dev);
	if (status)
		return ERR_PTR(status);

	status = devm_add_action(IP2DEVP(mwdev), (devm_action_fn)device_unregister, &mwregion->dev);
	if (status) {
		device_unregister(&mwregion->dev);
		return ERR_PTR(status);
	}

	devres_close_group(IP2DEVP(mwdev), devm_mw_sharedmem_region_alloc);
	
	return mwregion;
}

static int mw_sharedmem_iio_probe(
		struct mathworks_ipcore_dev *mwdev,
		struct device_node *node)
{
	struct mw_sharedmem_region_dev *mwregion;
	struct device_node *child;
	const struct of_device_id *match;
	struct mw_sharedmem_iio_channel_info *info;
	char *size_fmt = "MB";
	size_t size_disp;
	int status;
	
	/* At least one read/write channel child node must exist */
	if (!mw_sharedmem_count_iio_channels(node)) {
		dev_err(IP2DEVP(mwdev), "No read/write channels found for node : %s\n",node->name);
		return -EINVAL;
	}
	
	mwregion = devm_mw_sharedmem_region_alloc(mwdev, node);
	if (IS_ERR(mwregion)) {
		dev_err(IP2DEVP(mwdev), "Failed to configure shared memory region: %ld\n", PTR_ERR(mwregion));
		return PTR_ERR(mwregion);
	} else {
		/* If region size is >= 1 MB, display size as MB */
		if (mwregion->region.size >= (2^20)){
			size_disp = mwregion->region.size >> 20;
		} 
		/* Otherwise display as kB */
		else {
			size_disp = mwregion->region.size >> 10;
			size_fmt[0] = 'k';
		}	
		dev_info(IP2DEVP(mwdev), "Allocated reserved memory, virt: 0x%0zX, phys: 0x%0zX, size: %zd %s\n", 
			(size_t)mwregion->region.virt, (size_t)mwregion->region.phys, size_disp, size_fmt);
	}

	/* Probe the read/write channels */
	for_each_child_of_node(node,child) {
		match = of_match_node(mw_sharedmem_iio_channel_of_match, child);
		if(match){
			info = (struct mw_sharedmem_iio_channel_info *)match->data;
			status = mw_sharedmem_iio_channel_probe(mwregion, child, info->iio_direction);
			if(status)
				return status;
		}
	}
	
	return 0;
}

static const struct of_device_id mw_sharedmem_iio_of_match[] = {
	{ .compatible = "mathworks,sharedmem-v1.00"},
    {},
};

int mw_sharedmem_iio_channels_probe(struct mathworks_ipcore_dev *mwdev)
{
	int status;

	struct device_node *child;
	const struct of_device_id *match;


	for_each_child_of_node(IP2DEVP(mwdev)->of_node,child) {
		match = of_match_node(mw_sharedmem_iio_of_match, child);
		if(match){
			status = mw_sharedmem_iio_probe(mwdev, child);
			if(status)
				return status;
		}
	}

	return 0;
}

EXPORT_SYMBOL_GPL(mw_sharedmem_iio_channels_probe);

static int __init mw_sharedmem_iio_channel_init(void)
{
	return 0;
}

static void __exit mw_sharedmem_iio_channel_exit(void)
{

}

module_init(mw_sharedmem_iio_channel_init);
module_exit(mw_sharedmem_iio_channel_exit);

MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MathWorks Shared Memory IIO Channel");
MODULE_ALIAS(DRIVER_NAME);
