/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_IIO_HW_TRIGGEREDED_BUFFER_H_
#define _LINUX_IIO_HW_TRIGGEREDED_BUFFER_H_

struct device;
struct iio_dev;
struct iio_buffer_setup_ops;

int devm_iio_hw_triggered_buffer_setup(struct device *dev,
				       struct iio_dev *indio_dev,
				       struct device *match,
				       const struct iio_buffer_setup_ops *ops);

#endif /* _LINUX_IIO_HW_TRIGGEREDED_BUFFER_H_ */
