/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_IIO_HW_TRIGGEREDED_BUFFER_IMPL_H_
#define _LINUX_IIO_HW_TRIGGEREDED_BUFFER_IMPL_H_

#include <linux/auxiliary_bus.h>

struct iio_buffer;
struct iio_trigger;

struct iio_hw_triggered_buffer_device {
	struct auxiliary_device adev;
	struct iio_buffer *buffer;
	struct iio_trigger *trig;
};

#endif /* _LINUX_IIO_HW_TRIGGEREDED_BUFFER_IMPL_H_ */
