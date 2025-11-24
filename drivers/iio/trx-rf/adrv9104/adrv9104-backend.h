/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2025 Analog Devices Inc.
 */

#ifndef __ADRV9104_BACKEND_H__
#define __ADRV9104_BACKEND_H__

#include <linux/iio/backend.h>
#include <linux/types.h>

struct adrv9104_rf_phy;
struct adrv9104_chan;
struct iio_dev;

struct iio_chan_spec;

int adrv9104_backend_get(struct adrv9104_rf_phy *phy, struct adrv9104_chan *chan,
			 struct iio_dev *indio_dev,
			 struct iio_chan_spec *dds_base_chan);

struct iio_backend *adrv9104_backend_get_from_chan(struct iio_dev *indio_dev,
						   uintptr_t private,
						   const struct iio_chan_spec *chan);

int adrv9104_backend_init(struct adrv9104_rf_phy *phy, struct adrv9104_chan *chan);

static inline void adrv9104_backend_disable(struct iio_backend *back)
{
	iio_backend_disable(back);
}

static inline int adrv9104_backend_enable(struct iio_backend *back)
{
	return iio_backend_enable(back);
}

#endif
