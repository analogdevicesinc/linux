/*
 * MathWorks Streaming Channel
 *
 * Copyright 2016 The MathWorks, Inc
 *
 * Licensed under the GPL-2.
 */

#ifndef _MW_MM_IIO_CHANNEL_H_
#define _MW_MM_IIO_CHANNEL_H_

#include "mathworks_ipcore.h"

/*********************************************************
* API functions
*********************************************************/
#if defined(CONFIG_MWIPCORE_IIO_MM) || defined(CONFIG_MWIPCORE_IIO_MM_MODULE)
extern int mw_mm_iio_channels_probe(struct mathworks_ipcore_dev	*mwdev);
#else
static inline int mw_mm_iio_channels_probe(struct mathworks_ipcore_dev	*mwdev) {
	return -ENODEV;
}
#endif

#endif /* _MW_MM_IIO_CHANNEL_H_ */
