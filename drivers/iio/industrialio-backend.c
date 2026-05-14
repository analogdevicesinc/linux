// SPDX-License-Identifier: GPL-2.0-only
/*
 * Framework to handle complex IIO aggregate devices.
 *
 * The typical architecture is to have one device as the frontend device which
 * can be "linked" against one or multiple backend devices. All the IIO and
 * userspace interface is expected to be registers/managed by the frontend
 * device which will callback into the backends when needed (to get/set some
 * configuration that it does not directly control).
 *
 *                                           -------------------------------------------------------
 * ------------------                        | ------------         ------------      -------  FPGA|
 * |     ADC        |------------------------| | ADC CORE |---------| DMA CORE |------| RAM |      |
 * | (Frontend/IIO) | Serial Data (eg: LVDS) | |(backend) |---------|          |------|     |      |
 * |                |------------------------| ------------         ------------      -------      |
 * ------------------                        -------------------------------------------------------
 *
 * The framework interface is pretty simple:
 *   - Backends should register themselves with devm_iio_backend_register()
 *   - Frontend devices should get backends with devm_iio_backend_get()
 *
 * Also to note that the primary target for this framework are converters like
 * ADC/DACs so iio_backend_ops will have some operations typical of converter
 * devices. On top of that, this is "generic" for all IIO which means any kind
 * of device can make use of the framework. That said, If the iio_backend_ops
 * struct begins to grow out of control, we can always refactor things so that
 * the industrialio-backend.c is only left with the really generic stuff. Then,
 * we can build on top of it depending on the needs.
 *
 * Copyright (C) 2023-2024 Analog Devices Inc.
 */
#define dev_fmt(fmt) "iio-backend: " fmt

#include <linux/cleanup.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/stringify.h>
#include <linux/types.h>

#include <linux/iio/backend.h>
#include <linux/iio/iio.h>
#include <linux/iio/iio-opaque.h>
#include <linux/iio/sysfs.h>
#include "iio_core.h"

struct iio_backend {
	struct list_head entry;
	const struct iio_backend_ops *ops;
	struct device *frontend_dev;
	struct device *dev;
	struct module *owner;
	void *priv;
	const char *name;
	struct list_head attr_list;
	struct attribute_group attr_group;
	unsigned int cached_reg_addr;
	/*
	 * This index is relative to the frontend. Meaning that for
	 * frontends with multiple backends, this will be the index of this
	 * backend. Used for the debugfs directory name.
	 */
	u8 idx;
};

/**
 * struct iio_backend_attr - Backend-specific IIO device attribute
 * @iio_attr: Underlying IIO device attribute (must be first)
 * @back: Backend this attribute belongs to
 * @ext_info: Extended channel info descriptor for this attribute
 */
struct iio_backend_attr {
	struct iio_dev_attr iio_attr;
	struct iio_backend *back;
	const struct iio_backend_chan_ext_info *ext_info;
};

#define to_iio_backend_attr(_iio_dev_attr)				\
	container_of(_iio_dev_attr, struct iio_backend_attr, iio_attr)

static_assert(offsetof(struct iio_backend_attr, iio_attr) == 0,
	      "iio_dev_attr must be the first member of iio_backend_attr");

/*
 * Helper struct for requesting buffers. This ensures that we have all data
 * that we need to free the buffer in a device managed action.
 */
struct iio_backend_buffer_pair {
	struct iio_backend *back;
	struct iio_buffer *buffer;
};

static LIST_HEAD(iio_back_list);
static DEFINE_MUTEX(iio_back_lock);

/*
 * Helper macros to call backend ops. Makes sure the option is supported.
 */
#define iio_backend_check_op(back, op) ({ \
	struct iio_backend *____back = back;				\
	int ____ret = 0;						\
									\
	if (!____back->ops->op)						\
		____ret = -EOPNOTSUPP;					\
									\
	____ret;							\
})

#define iio_backend_op_call(back, op, args...) ({		\
	struct iio_backend *__back = back;			\
	int __ret;						\
								\
	__ret = iio_backend_check_op(__back, op);		\
	if (!__ret)						\
		__ret = __back->ops->op(__back, ##args);	\
								\
	__ret;							\
})

#define iio_backend_ptr_op_call(back, op, args...) ({		\
	struct iio_backend *__back = back;			\
	void *ptr_err;						\
	int __ret;						\
								\
	__ret = iio_backend_check_op(__back, op);		\
	if (__ret)						\
		ptr_err = ERR_PTR(__ret);			\
	else							\
		ptr_err = __back->ops->op(__back, ##args);	\
								\
	ptr_err;						\
})

#define iio_backend_void_op_call(back, op, args...) {		\
	struct iio_backend *__back = back;			\
	int __ret;						\
								\
	__ret = iio_backend_check_op(__back, op);		\
	if (!__ret)						\
		__back->ops->op(__back, ##args);		\
	else							\
		dev_dbg(__back->dev, "Op(%s) not implemented\n",\
			__stringify(op));			\
}

static ssize_t iio_backend_debugfs_read_reg(struct file *file,
					    char __user *userbuf,
					    size_t count, loff_t *ppos)
{
	struct iio_backend *back = file->private_data;
	char read_buf[20];
	unsigned int val;
	int ret, len;

	ret = iio_backend_op_call(back, debugfs_reg_access,
				  back->cached_reg_addr, 0, &val);
	if (ret)
		return ret;

	len = scnprintf(read_buf, sizeof(read_buf), "0x%X\n", val);

	return simple_read_from_buffer(userbuf, count, ppos, read_buf, len);
}

static ssize_t iio_backend_debugfs_write_reg(struct file *file,
					     const char __user *userbuf,
					     size_t count, loff_t *ppos)
{
	struct iio_backend *back = file->private_data;
	unsigned int val;
	char buf[80];
	ssize_t rc;
	int ret;

	rc = simple_write_to_buffer(buf, sizeof(buf), ppos, userbuf, count);
	if (rc < 0)
		return rc;

	ret = sscanf(buf, "%i %i", &back->cached_reg_addr, &val);

	switch (ret) {
	case 1:
		return count;
	case 2:
		ret = iio_backend_op_call(back, debugfs_reg_access,
					  back->cached_reg_addr, val, NULL);
		if (ret)
			return ret;
		return count;
	default:
		return -EINVAL;
	}
}

static const struct file_operations iio_backend_debugfs_reg_fops = {
	.open = simple_open,
	.read = iio_backend_debugfs_read_reg,
	.write = iio_backend_debugfs_write_reg,
};

static ssize_t iio_backend_debugfs_read_name(struct file *file,
					     char __user *userbuf,
					     size_t count, loff_t *ppos)
{
	struct iio_backend *back = file->private_data;
	char name[128];
	int len;

	len = scnprintf(name, sizeof(name), "%s\n", back->name);

	return simple_read_from_buffer(userbuf, count, ppos, name, len);
}

static const struct file_operations iio_backend_debugfs_name_fops = {
	.open = simple_open,
	.read = iio_backend_debugfs_read_name,
};

/**
 * iio_backend_debugfs_add - Add debugfs interfaces for Backends
 * @back: Backend device
 * @indio_dev: IIO device
 */
void iio_backend_debugfs_add(struct iio_backend *back,
			     struct iio_dev *indio_dev)
{
	struct dentry *d = iio_get_debugfs_dentry(indio_dev);
	struct dentry *back_d;
	char name[128];

	if (!IS_ENABLED(CONFIG_DEBUG_FS) || !d)
		return;
	if (!back->ops->debugfs_reg_access && !back->name)
		return;

	snprintf(name, sizeof(name), "backend%d", back->idx);

	back_d = debugfs_create_dir(name, d);
	if (IS_ERR(back_d))
		return;

	if (back->ops->debugfs_reg_access)
		debugfs_create_file("direct_reg_access", 0600, back_d, back,
				    &iio_backend_debugfs_reg_fops);

	if (back->name)
		debugfs_create_file("name", 0400, back_d, back,
				    &iio_backend_debugfs_name_fops);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_debugfs_add, IIO_BACKEND);

/**
 * iio_backend_debugfs_print_chan_status - Print channel status
 * @back: Backend device
 * @chan: Channel number
 * @buf: Buffer where to print the status
 * @len: Available space
 *
 * One usecase where this is useful is for testing test tones in a digital
 * interface and "ask" the backend to dump more details on why a test tone might
 * have errors.
 *
 * RETURNS:
 * Number of copied bytes on success, negative error code on failure.
 */
ssize_t iio_backend_debugfs_print_chan_status(struct iio_backend *back,
					      unsigned int chan, char *buf,
					      size_t len)
{
	if (!IS_ENABLED(CONFIG_DEBUG_FS))
		return -ENODEV;

	return iio_backend_op_call(back, debugfs_print_chan_status, chan, buf,
				   len);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_debugfs_print_chan_status, IIO_BACKEND);

/**
 * iio_backend_chan_enable - Enable a backend channel
 * @back: Backend device
 * @chan: Channel number
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_chan_enable(struct iio_backend *back, unsigned int chan)
{
	return iio_backend_op_call(back, chan_enable, chan);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_chan_enable, IIO_BACKEND);

/**
 * iio_backend_chan_disable - Disable a backend channel
 * @back: Backend device
 * @chan: Channel number
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_chan_disable(struct iio_backend *back, unsigned int chan)
{
	return iio_backend_op_call(back, chan_disable, chan);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_chan_disable, IIO_BACKEND);

static void __iio_backend_disable(void *back)
{
	iio_backend_void_op_call(back, disable);
}

/**
 * iio_backend_disable - Backend disable
 * @back: Backend device
 */
void iio_backend_disable(struct iio_backend *back)
{
	__iio_backend_disable(back);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_disable, IIO_BACKEND);

/**
 * iio_backend_enable - Backend enable
 * @back: Backend device
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_enable(struct iio_backend *back)
{
	return iio_backend_op_call(back, enable);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_enable, IIO_BACKEND);

/**
 * devm_iio_backend_enable - Device managed backend enable
 * @dev: Consumer device for the backend
 * @back: Backend device
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int devm_iio_backend_enable(struct device *dev, struct iio_backend *back)
{
	int ret;

	ret = iio_backend_enable(back);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, __iio_backend_disable, back);
}
EXPORT_SYMBOL_NS_GPL(devm_iio_backend_enable, IIO_BACKEND);

/**
 * iio_backend_data_format_set - Configure the channel data format
 * @back: Backend device
 * @chan: Channel number
 * @data: Data format
 *
 * Properly configure a channel with respect to the expected data format. A
 * @struct iio_backend_data_fmt must be passed with the settings.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_data_format_set(struct iio_backend *back, unsigned int chan,
				const struct iio_backend_data_fmt *data)
{
	if (!data || data->type >= IIO_BACKEND_DATA_TYPE_MAX)
		return -EINVAL;

	return iio_backend_op_call(back, data_format_set, chan, data);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_data_format_set, IIO_BACKEND);

/**
 * iio_backend_data_source_set - Select data source
 * @back: Backend device
 * @chan: Channel number
 * @data: Data source
 *
 * A given backend may have different sources to stream/sync data. This allows
 * to choose that source.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_data_source_set(struct iio_backend *back, unsigned int chan,
				enum iio_backend_data_source data)
{
	if (data >= IIO_BACKEND_DATA_SOURCE_MAX)
		return -EINVAL;

	return iio_backend_op_call(back, data_source_set, chan, data);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_data_source_set, IIO_BACKEND);

/**
 * iio_backend_data_source_get - Get current data source
 * @back: Backend device
 * @chan: Channel number
 * @data: Pointer to receive the current source value
 *
 * A given backend may have different sources to stream/sync data. This allows
 * to know what source is in use.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_data_source_get(struct iio_backend *back, unsigned int chan,
				enum iio_backend_data_source *data)
{
	int ret;

	ret = iio_backend_op_call(back, data_source_get, chan, data);
	if (ret)
		return ret;

	if (*data >= IIO_BACKEND_DATA_SOURCE_MAX)
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL_NS_GPL(iio_backend_data_source_get, IIO_BACKEND);

/**
 * iio_backend_set_sampling_freq - Set channel sampling rate
 * @back: Backend device
 * @chan: Channel number
 * @sample_rate_hz: Sample rate
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_set_sampling_freq(struct iio_backend *back, unsigned int chan,
				  u64 sample_rate_hz)
{
	return iio_backend_op_call(back, set_sample_rate, chan, sample_rate_hz);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_set_sampling_freq, IIO_BACKEND);

/**
 * iio_backend_test_pattern_set - Configure a test pattern
 * @back: Backend device
 * @chan: Channel number
 * @pattern: Test pattern
 *
 * Configure a test pattern on the backend. This is typically used for
 * calibrating the timings on the data digital interface.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_test_pattern_set(struct iio_backend *back,
				 unsigned int chan,
				 enum iio_backend_test_pattern pattern)
{
	if (pattern >= IIO_BACKEND_TEST_PATTERN_MAX)
		return -EINVAL;

	return iio_backend_op_call(back, test_pattern_set, chan, pattern);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_test_pattern_set, IIO_BACKEND);

/**
 * iio_backend_chan_status - Get the channel status
 * @back: Backend device
 * @chan: Channel number
 * @error: Error indication
 *
 * Get the current state of the backend channel. Typically used to check if
 * there were any errors sending/receiving data.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_chan_status(struct iio_backend *back, unsigned int chan,
			    bool *error)
{
	return iio_backend_op_call(back, chan_status, chan, error);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_chan_status, IIO_BACKEND);

/**
 * iio_backend_iodelay_set - Set digital I/O delay
 * @back: Backend device
 * @lane: Lane number
 * @taps: Number of taps
 *
 * Controls delays on sending/receiving data. One usecase for this is to
 * calibrate the data digital interface so we get the best results when
 * transferring data. Note that @taps has no unit since the actual delay per tap
 * is very backend specific. Hence, frontend devices typically should go through
 * an array of @taps (the size of that array should typically match the size of
 * calibration points on the frontend device) and call this API.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_iodelay_set(struct iio_backend *back, unsigned int lane,
			    unsigned int taps)
{
	return iio_backend_op_call(back, iodelay_set, lane, taps);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_iodelay_set, IIO_BACKEND);

/**
 * iio_backend_data_sample_trigger - Control when to sample data
 * @back: Backend device
 * @trigger: Data trigger
 *
 * Mostly useful for input backends. Configures the backend for when to sample
 * data (eg: rising vs falling edge).
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_data_sample_trigger(struct iio_backend *back,
				    enum iio_backend_sample_trigger trigger)
{
	if (trigger >= IIO_BACKEND_SAMPLE_TRIGGER_MAX)
		return -EINVAL;

	return iio_backend_op_call(back, data_sample_trigger, trigger);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_data_sample_trigger, IIO_BACKEND);

static void iio_backend_free_buffer(void *arg)
{
	struct iio_backend_buffer_pair *pair = arg;

	iio_backend_void_op_call(pair->back, free_buffer, pair->buffer);
}

/**
 * devm_iio_backend_request_buffer - Device managed buffer request
 * @dev: Consumer device for the backend
 * @back: Backend device
 * @indio_dev: IIO device
 *
 * Request an IIO buffer from the backend. The type of the buffer (typically
 * INDIO_BUFFER_HARDWARE) is up to the backend to decide. This is because,
 * normally, the backend dictates what kind of buffering we can get.
 *
 * The backend .free_buffer() hooks is automatically called on @dev detach.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int devm_iio_backend_request_buffer(struct device *dev,
				    struct iio_backend *back,
				    struct iio_dev *indio_dev)
{
	struct iio_backend_buffer_pair *pair;
	struct iio_buffer *buffer;

	pair = devm_kzalloc(dev, sizeof(*pair), GFP_KERNEL);
	if (!pair)
		return -ENOMEM;

	buffer = iio_backend_ptr_op_call(back, request_buffer, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	/* weak reference should be all what we need */
	pair->back = back;
	pair->buffer = buffer;

	return devm_add_action_or_reset(dev, iio_backend_free_buffer, pair);
}
EXPORT_SYMBOL_NS_GPL(devm_iio_backend_request_buffer, IIO_BACKEND);

/**
 * iio_backend_read_raw - Read a channel attribute from a backend device.
 * @back:	Backend device
 * @chan:	IIO channel reference
 * @val:	First returned value
 * @val2:	Second returned value
 * @mask:	Specify the attribute to return
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_read_raw(struct iio_backend *back,
			 struct iio_chan_spec const *chan, int *val, int *val2,
			 long mask)
{
	return iio_backend_op_call(back, read_raw, chan, val, val2, mask);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_read_raw, IIO_BACKEND);


static ssize_t iio_backend_ext_info_read(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct iio_dev_attr *iio_attr = to_iio_dev_attr(attr);
	struct iio_backend_attr *battr = to_iio_backend_attr(iio_attr);

	return battr->ext_info->read(battr->back, battr->ext_info->private,
				     battr->iio_attr.c, buf);
}

static ssize_t iio_backend_ext_info_write(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t len)
{
	struct iio_dev_attr *iio_attr = to_iio_dev_attr(attr);
	struct iio_backend_attr *battr = to_iio_backend_attr(iio_attr);

	return battr->ext_info->write(battr->back, battr->ext_info->private,
				      battr->iio_attr.c, buf, len);
}

static void iio_backend_attr_free(struct iio_backend_attr *battr)
{
	__iio_device_attr_deinit(&battr->iio_attr.dev_attr);
	kfree(battr);
}

DEFINE_FREE(iio_back_attr, struct iio_backend_attr *, if (_T) iio_backend_attr_free(_T)) 

static bool iio_backend_ext_info_collision(const struct iio_chan_spec *chan,
					   const struct iio_backend_chan_ext_info *info)
{
	const struct iio_chan_spec_ext_info *fe;

	if (!chan->ext_info)
		return false;

	for (fe = chan->ext_info; fe->name; fe++)
		if (!strcmp(fe->name, info->name) && fe->shared == info->shared)
			return true;

	return false;
}

/**
 * iio_backend_oversampling_ratio_set - set the oversampling ratio
 * @back: Backend device
 * @ratio: The oversampling ratio - value 1 corresponds to no oversampling.
 *
 * Return:
 * 0 on success, negative error number on failure.
 */
int iio_backend_oversampling_ratio_set(struct iio_backend *back,
				       unsigned int chan,
				       unsigned int ratio)
{
	return iio_backend_op_call(back, oversampling_ratio_set, chan, ratio);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_oversampling_ratio_set, IIO_BACKEND);

/**
 * iio_backend_data_size_set - set the data width/size in the data bus.
 * @back: Backend device
 * @size: Size in bits
 *
 * Some frontend devices can dynamically control the word/data size on the
 * interface/data bus. Hence, the backend device needs to be aware of it so
 * data can be correctly transferred.
 *
 * Return:
 * 0 on success, negative error number on failure.
 */
int iio_backend_data_size_set(struct iio_backend *back, unsigned int size)
{
	if (!size)
		return -EINVAL;

	return iio_backend_op_call(back, data_size_set, size);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_data_size_set, IIO_BACKEND);

/**
 * iio_backend_extend_chan_spec - Extend an IIO channel
 * iio_backend_extend_chan_spec - Extend an IIO channel with backend attributes
 * @back: Backend device
 * @chan: IIO channel
 *
 * Calls the backend's extend_chan_spec op to get extended channel attributes,
 * then creates sysfs attribute entries on the backend's internal list. The
 * attributes are registered to sysfs later during iio_backend_add_extended_sysfs().
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_extend_chan_spec(struct iio_backend *back,
				 const struct iio_chan_spec *chan)
{
	const struct iio_backend_chan_ext_info *ext_info = NULL;
	const struct iio_backend_chan_ext_info *info;
	int ret;

	ret = iio_backend_op_call(back, extend_chan_spec, chan, &ext_info);
	if (ret)
		return ret;

	if (!ext_info)
		return 0;

	for (info = ext_info; info->name; info++) {
		struct iio_backend_attr *battr __free(iio_back_attr) = NULL;

		/* early check for frontend-backend attribute collision */
		if (iio_backend_ext_info_collision(chan, info)) {
			dev_err(back->dev,
				"backend attr shadows frontend ext_info: %s\n",
				info->name);
			continue;
		}

		battr = kzalloc(sizeof(*battr), GFP_KERNEL);
		if (!battr)
			return -ENOMEM;

		ret = __iio_device_attr_init(back->dev,
					     &battr->iio_attr.dev_attr,
					     info->name, chan,
					     info->read ? &iio_backend_ext_info_read : NULL,
					     info->write ? &iio_backend_ext_info_write : NULL,
					     info->shared);
		if (ret)
			return ret;

		battr->iio_attr.c = chan;
		battr->back = back;
		battr->ext_info = info;

		/* duplicate check on backend's own list */
		if (__iio_dev_attr_check_dup(&battr->iio_attr, &back->attr_list)) {
			if (info->shared == IIO_SEPARATE)
				dev_err(back->dev,
					"tried to double register : %s\n",
					battr->iio_attr.dev_attr.attr.name);
			continue;
		}

		list_add(&no_free_ptr(battr)->iio_attr.l, &back->attr_list);
	}

	return 0;
}
EXPORT_SYMBOL_NS_GPL(iio_backend_extend_chan_spec, IIO_BACKEND);


static bool iio_backend_attr_conflicts(struct iio_dev *indio_dev,
				       struct iio_backend_attr *battr)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);
	struct iio_backend *other;

	if (__iio_dev_attr_check_dup(&battr->iio_attr, &iio_dev_opaque->channel_attr_list))
		return true;

	/*
	 * Check all other backends belonging to the same frontend that were
	 * registered before battr->back
	 */
	other = list_prepare_entry(battr->back, &iio_back_list, entry);
	list_for_each_entry_continue(other, &iio_back_list, entry) {
		if (other->frontend_dev != indio_dev->dev.parent)
			continue;
		if (__iio_dev_attr_check_dup(&battr->iio_attr, &other->attr_list))
			return true;
	}
	return false;
}

/**
 * iio_backend_add_extended_sysfs - Register backend channel attrs to sysfs
 * @indio_dev: IIO device
 *
 * Called from __iio_device_register() after channel attrs are set up. Checks
 * for name collisions with frontend channel_attr_list and registers the
 * backend's attribute group.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_add_extended_sysfs(struct iio_dev *indio_dev)
{
	struct iio_backend_attr *battr;
	struct iio_backend *back;
	struct iio_dev_attr *p, *n;
	int attrcount, attrn, ret;

	guard(mutex)(&iio_back_lock);
	list_for_each_entry(back, &iio_back_list, entry) {
		if (back->frontend_dev != indio_dev->dev.parent)
			continue;

		attrcount = 0;
		list_for_each_entry_safe(p, n, &back->attr_list, l) {
			battr = to_iio_backend_attr(p);
			if (iio_backend_attr_conflicts(indio_dev, battr)) {
				dev_err(back->dev, "backend attr conflict: %s\n",
					p->dev_attr.attr.name);
				list_del(&p->l);
				iio_backend_attr_free(battr);
				continue;
			}
			attrcount++;
		}

		if (!attrcount)
			continue;

		back->attr_group.attrs = devm_kcalloc(&indio_dev->dev, attrcount + 1,
						      sizeof(*back->attr_group.attrs),
						      GFP_KERNEL);
		if (!back->attr_group.attrs)
			return -ENOMEM;

		attrn = 0;
		list_for_each_entry(p, &back->attr_list, l)
			back->attr_group.attrs[attrn++] = &p->dev_attr.attr;

		ret = iio_device_register_sysfs_group(indio_dev, &back->attr_group);
		if (ret)
			return ret;
	}

	return 0;
}

static void iio_backend_release(void *arg)
{
	struct iio_backend *back = arg;

	iio_free_chan_devattr_list(&back->attr_list);
	module_put(back->owner);
}

static int __devm_iio_backend_get(struct device *dev, struct iio_backend *back)
{
	struct device_link *link;
	int ret;

	/*
	 * Make sure the provider cannot be unloaded before the consumer module.
	 * Note that device_links would still guarantee that nothing is
	 * accessible (and breaks) but this makes it explicit that the consumer
	 * module must be also unloaded.
	 */
	if (!try_module_get(back->owner))
		return dev_err_probe(dev, -ENODEV,
				     "Cannot get module reference\n");

	ret = devm_add_action_or_reset(dev, iio_backend_release, back);
	if (ret)
		return ret;

	link = device_link_add(dev, back->dev, DL_FLAG_AUTOREMOVE_CONSUMER);
	if (!link)
		return dev_err_probe(dev, -EINVAL,
				     "Could not link to supplier(%s)\n",
				     dev_name(back->dev));

	back->frontend_dev = dev;

	dev_dbg(dev, "Found backend(%s) device\n", dev_name(back->dev));

	return 0;
}

/**
 * iio_backend_filter_type_set - Set filter type
 * @back: Backend device
 * @type: Filter type.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_filter_type_set(struct iio_backend *back,
				enum iio_backend_filter_type type)
{
	if (type >= IIO_BACKEND_FILTER_TYPE_MAX)
		return -EINVAL;

	return iio_backend_op_call(back, filter_type_set, type);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_filter_type_set, IIO_BACKEND);

/**
 * iio_backend_interface_data_align - Perform the data alignment process.
 * @back: Backend device
 * @timeout_us: Timeout value in us.
 *
 * When activated, it initates a proccess that aligns the sample's most
 * significant bit (MSB) based solely on the captured data, without
 * considering any other external signals.
 *
 * The timeout_us value must be greater than 0.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_interface_data_align(struct iio_backend *back, u32 timeout_us)
{
	if (!timeout_us)
		return -EINVAL;

	return iio_backend_op_call(back, interface_data_align, timeout_us);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_interface_data_align, IIO_BACKEND);

/**
 * iio_backend_num_lanes_set - Number of lanes enabled.
 * @back: Backend device
 * @num_lanes: Number of lanes.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_num_lanes_set(struct iio_backend *back, unsigned int num_lanes)
{
	if (!num_lanes)
		return -EINVAL;

	return iio_backend_op_call(back, num_lanes_set, num_lanes);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_num_lanes_set, IIO_BACKEND);

/**
 * iio_backend_ddr_enable - Enable interface DDR (Double Data Rate) mode
 * @back: Backend device
 *
 * Enable DDR, data is generated by the IP at each front (raising and falling)
 * of the bus clock signal.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_ddr_enable(struct iio_backend *back)
{
	return iio_backend_op_call(back, ddr_enable);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_ddr_enable, IIO_BACKEND);

/**
 * iio_backend_ddr_disable - Disable interface DDR (Double Data Rate) mode
 * @back: Backend device
 *
 * Disable DDR, setting into SDR mode (Single Data Rate).
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_ddr_disable(struct iio_backend *back)
{
	return iio_backend_op_call(back, ddr_disable);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_ddr_disable, IIO_BACKEND);

/**
 * iio_backend_data_stream_enable - Enable data stream
 * @back: Backend device
 *
 * Enable data stream over the bus interface.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_data_stream_enable(struct iio_backend *back)
{
	return iio_backend_op_call(back, data_stream_enable);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_data_stream_enable, IIO_BACKEND);

/**
 * iio_backend_data_stream_disable - Disable data stream
 * @back: Backend device
 *
 * Disable data stream over the bus interface.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_data_stream_disable(struct iio_backend *back)
{
	return iio_backend_op_call(back, data_stream_disable);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_data_stream_disable, IIO_BACKEND);

/**
 * iio_backend_data_transfer_addr - Set data address.
 * @back: Backend device
 * @address: Data register address
 *
 * Some devices may need to inform the backend about an address
 * where to read or write the data.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int iio_backend_data_transfer_addr(struct iio_backend *back, u32 address)
{
	return iio_backend_op_call(back, data_transfer_addr, address);
}
EXPORT_SYMBOL_NS_GPL(iio_backend_data_transfer_addr, IIO_BACKEND);

static struct iio_backend *__devm_iio_backend_fwnode_get_by_index(struct device *dev,
								  struct fwnode_handle *fwnode,
								  unsigned int index,
								  bool optional)
{
	struct fwnode_handle *fwnode_back;
	struct iio_backend *back;
	int ret;

	fwnode_back = fwnode_find_reference(fwnode, "io-backends", index);
	if (IS_ERR(fwnode_back)) {
		if (optional && PTR_ERR(fwnode_back) == -ENOENT)
			return NULL;

		return dev_err_cast_probe(dev, fwnode_back,
					  "Cannot get Firmware reference\n");
	}

	guard(mutex)(&iio_back_lock);
	list_for_each_entry(back, &iio_back_list, entry) {
		if (!device_match_fwnode(back->dev, fwnode_back))
			continue;

		fwnode_handle_put(fwnode_back);
		ret = __devm_iio_backend_get(dev, back);
		if (ret)
			return ERR_PTR(ret);

		back->idx = index;

		return back;
	}

	fwnode_handle_put(fwnode_back);
	return ERR_PTR(-EPROBE_DEFER);
}

static struct iio_backend *__devm_iio_backend_fwnode_get(struct device *dev, const char *name,
							 struct fwnode_handle *fwnode,
							 bool optional)
{
	unsigned int index;
	int ret;

	if (name) {
		ret = device_property_match_string(dev, "io-backend-names", name);
		if (ret < 0)
			return ERR_PTR(ret);
		index = ret;
	} else {
		index = 0;
	}

	return __devm_iio_backend_fwnode_get_by_index(dev, fwnode, index, optional);
}

/**
 * __devm_iio_backend_get_ext() - Device managed backend device get
 * @dev: Consumer device for the backend
 * @name: Backend name
 * @optional: Whether the backend is optional or not
 *
 * Get's the backend associated with @dev.
 *
 * RETURNS:
 * A backend pointer, negative error pointer otherwise.
 */
struct iio_backend *__devm_iio_backend_get_ext(struct device *dev,
					       const char *name, bool optional)
{
	return __devm_iio_backend_fwnode_get(dev, name, dev_fwnode(dev), optional);
}
EXPORT_SYMBOL_NS_GPL(__devm_iio_backend_get_ext, IIO_BACKEND);

/**
 * devm_iio_backend_get_by_index - Device managed backend device get by index
 * @dev: Consumer device for the backend
 * @index: Index of the backend in the io-backends property
 *
 * Gets the backend at @index associated with @dev.
 *
 * RETURNS:
 * A backend pointer, negative error pointer otherwise.
 */
struct iio_backend *devm_iio_backend_get_by_index(struct device *dev, unsigned int index)
{
	return __devm_iio_backend_fwnode_get_by_index(dev, dev_fwnode(dev), index, false);
}
EXPORT_SYMBOL_NS_GPL(devm_iio_backend_get_by_index, IIO_BACKEND);

/**
 * devm_iio_backend_fwnode_get - Device managed backend firmware node get
 * @dev: Consumer device for the backend
 * @name: Backend name
 * @fwnode: Firmware node of the backend consumer
 *
 * Get's the backend associated with a firmware node.
 *
 * RETURNS:
 * A backend pointer, negative error pointer otherwise.
 */
struct iio_backend *devm_iio_backend_fwnode_get(struct device *dev,
						const char *name,
						struct fwnode_handle *fwnode)
{
	return __devm_iio_backend_fwnode_get(dev, name, fwnode, false);
}
EXPORT_SYMBOL_NS_GPL(devm_iio_backend_fwnode_get, IIO_BACKEND);

/**
 * __devm_iio_backend_get_from_fwnode_lookup - Device managed fwnode backend device get
 * @dev: Consumer device for the backend
 * @fwnode: Firmware node of the backend device
 *
 * Search the backend list for a device matching @fwnode.
 * This API should not be used and it's only present for preventing the first
 * user of this framework to break it's DT ABI.
 *
 * RETURNS:
 * A backend pointer, negative error pointer otherwise.
 */
struct iio_backend *
__devm_iio_backend_get_from_fwnode_lookup(struct device *dev,
					  struct fwnode_handle *fwnode)
{
	struct iio_backend *back;
	int ret;

	guard(mutex)(&iio_back_lock);
	list_for_each_entry(back, &iio_back_list, entry) {
		if (!device_match_fwnode(back->dev, fwnode))
			continue;

		ret = __devm_iio_backend_get(dev, back);
		if (ret)
			return ERR_PTR(ret);

		return back;
	}

	return ERR_PTR(-EPROBE_DEFER);
}
EXPORT_SYMBOL_NS_GPL(__devm_iio_backend_get_from_fwnode_lookup, IIO_BACKEND);

/**
 * iio_backend_get_priv - Get driver private data
 * @back: Backend device
 */
void *iio_backend_get_priv(const struct iio_backend *back)
{
	return back->priv;
}
EXPORT_SYMBOL_NS_GPL(iio_backend_get_priv, IIO_BACKEND);

static void iio_backend_unregister(void *arg)
{
	struct iio_backend *back = arg;

	guard(mutex)(&iio_back_lock);
	list_del(&back->entry);
}

/**
 * devm_iio_backend_register - Device managed backend device register
 * @dev: Backend device being registered
 * @info: Backend info
 * @priv: Device private data
 *
 * @info is mandatory. Not providing it results in -EINVAL.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int devm_iio_backend_register(struct device *dev,
			      const struct iio_backend_info *info, void *priv)
{
	struct iio_backend *back;

	if (!info || !info->ops)
		return dev_err_probe(dev, -EINVAL, "No backend ops given\n");

	/*
	 * Through device_links, we guarantee that a frontend device cannot be
	 * bound/exist if the backend driver is not around. Hence, we can bind
	 * the backend object lifetime with the device being passed since
	 * removing it will tear the frontend/consumer down.
	 */
	back = devm_kzalloc(dev, sizeof(*back), GFP_KERNEL);
	if (!back)
		return -ENOMEM;

	back->ops = info->ops;
	back->name = info->name;
	back->owner = dev->driver->owner;
	back->dev = dev;
	back->priv = priv;
	INIT_LIST_HEAD(&back->attr_list);
	scoped_guard(mutex, &iio_back_lock)
		list_add(&back->entry, &iio_back_list);

	return devm_add_action_or_reset(dev, iio_backend_unregister, back);
}
EXPORT_SYMBOL_NS_GPL(devm_iio_backend_register, IIO_BACKEND);

MODULE_AUTHOR("Nuno Sa <nuno.sa@analog.com>");
MODULE_DESCRIPTION("Framework to handle complex IIO aggregate devices");
MODULE_LICENSE("GPL");
