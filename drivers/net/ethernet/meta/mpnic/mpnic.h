/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) Meta Platforms, Inc. and affiliates. */

#ifndef _MPNIC_H_
#define _MPNIC_H_

#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/types.h>

#include "mpnic_csr.h"

#define MPNIC_DRV_NAME		"mpnic"

#define MPNIC_MAX_TXQS		1024u
#define MPNIC_MAX_RXQS		1024u

struct mpnic_dev {
	struct device *dev;

	u32 __iomem *uc_addr0;

	u32 mps;
	u32 readrq;
	u8 relaxed_ord;
};

u64 mpnic_rd64(struct mpnic_dev *mpd, u32 reg);

int mpnic_dev_init(struct mpnic_dev *mpd);

static inline void mpnic_wr64(struct mpnic_dev *mpd, u32 reg, u64 val)
{
	u32 __iomem *csr = READ_ONCE(mpd->uc_addr0);

	if (csr)
		writeq(val, csr + reg);
}

static inline void mpnic_wrfl(struct mpnic_dev *mpd)
{
	mpnic_rd64(mpd, MPNIC_BDQ_SPARE);
}

static inline bool mpnic_present(struct mpnic_dev *mpd)
{
	return !!READ_ONCE(mpd->uc_addr0);
}

#endif /* _MPNIC_H_ */
