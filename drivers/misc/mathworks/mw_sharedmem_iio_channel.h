/*
 * MathWorks Shared Memory Channel
 *
 * Copyright 2019 The MathWorks, Inc
 *
 * Licensed under the GPL-2.
 */

#ifndef _MW_SHAREDMEM_IIO_CHANNEL_H_
#define _MW_SHAREDMEM_IIO_CHANNEL_H_

#include "mathworks_ipcore.h"

/*********************************************************
* API functions
*********************************************************/
#if defined(CONFIG_MWIPCORE_IIO_SHAREDMEM) || defined(CONFIG_MWIPCORE_IIO_SHAREDMEM_MODULE)
extern int mw_sharedmem_iio_channels_probe(struct mathworks_ipcore_dev	*mwdev);
#else
static inline int mw_sharedmem_iio_channels_probe(struct mathworks_ipcore_dev	*mwdev) {
	return -ENODEV;
}
#endif

#endif /* _MW_SHAREDMEM_IIO_CHANNEL_H_ */
