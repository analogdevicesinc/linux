
/* The industrial I/O core, trigger consumer handling functions
 *
 * Copyright (c) 2008 Jonathan Cameron
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifdef CONFIG_IIO_TRIGGER
/**
 * iio_device_register_trigger_consumer() - set up an iio_dev to use triggers
 * @indio_dev: iio_dev associated with the device that will consume the trigger
 **/
void iio_device_register_trigger_consumer(struct iio_dev *indio_dev);

/**
 * iio_device_unregister_trigger_consumer() - reverse the registration process
 * @indio_dev: iio_dev associated with the device that consumed the trigger
 **/
void iio_device_unregister_trigger_consumer(struct iio_dev *indio_dev);


int iio_trigger_attach_poll_func(struct iio_dev *indio_dev);
int iio_trigger_detach_poll_func(struct iio_dev *indio_dev);

#else

/**
 * iio_device_register_trigger_consumer() - set up an iio_dev to use triggers
 * @indio_dev: iio_dev associated with the device that will consume the trigger
 **/
static inline int iio_device_register_trigger_consumer(
	struct iio_dev *indio_dev)
{
	return 0;
}

/**
 * iio_device_unregister_trigger_consumer() - reverse the registration process
 * @indio_dev: iio_dev associated with the device that consumed the trigger
 **/
static inline void iio_device_unregister_trigger_consumer(
	struct iio_dev *indio_dev)
{
}

static inline int iio_trigger_attach_poll_func(struct iio_dev *indio_dev)
{
	return 0;
}

static inline int iio_trigger_detach_poll_func(struct iio_dev *indio_dev)
{
	return 0;
}

#endif /* CONFIG_TRIGGER_CONSUMER */
