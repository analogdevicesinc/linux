/*
 * MathWorks Streaming Channel
 *
 * Copyright 2016 The MathWorks, Inc
 *
 * Licensed under the GPL-2.
 */

#ifndef _MW_STREAM_IIO_CHANNEL_H_
#define _MW_STREAM_IIO_CHANNEL_H_

#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/errno.h>
#include "mathworks_ipcore.h"

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

struct mw_stream_iio_chandev {
	struct mathworks_ipcore_dev 			*mwdev;
	struct device							dev;
	enum iio_device_direction 				iio_direction;
	const char								*dmaname;
	enum mw_stream_iio_tlast_mode			tlast_mode;
	enum mw_stream_iio_reset_tlast_mode		reset_tlast_mode;
	int										tlast_cntr_addr;
	int										num_data_chan;
};

/*********************************************************
* API functions
*********************************************************/
#if defined(CONFIG_MWIPCORE_IIO_STREAMING) || defined(CONFIG_MWIPCORE_IIO_STREAMING_MODULE)
extern int mw_stream_iio_channels_probe(struct mathworks_ipcore_dev	*mwdev);
#else
static inline int mw_stream_iio_channels_probe(struct mathworks_ipcore_dev	*mwdev) {
	return -ENODEV;
}
#endif

#endif /* _MW_STREAM_IIO_CHANNEL_H_ */
