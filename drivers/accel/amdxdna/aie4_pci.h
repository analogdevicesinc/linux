/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2026, Advanced Micro Devices, Inc.
 */

#ifndef _AIE4_PCI_H_
#define _AIE4_PCI_H_

#include <linux/device.h>
#include <linux/iopoll.h>
#include <linux/pci.h>

#include "aie.h"
#include "amdxdna_mailbox.h"

struct amdxdna_dev_priv {
	u32			mbox_bar;
	u32			mbox_rbuf_bar;
	u64			mbox_info_off;
};

struct amdxdna_dev_hdl {
	struct aie_device		aie;
	const struct amdxdna_dev_priv	*priv;
	void			__iomem *mbox_base;
	void			__iomem *rbuf_base;

	struct mailbox			*mbox;
};

/* aie4_message.c */
int aie4_suspend_fw(struct amdxdna_dev_hdl *ndev);

/* aie4_sriov.c */
#if IS_ENABLED(CONFIG_PCI_IOV)
int aie4_sriov_configure(struct amdxdna_dev *xdna, int num_vfs);
int aie4_sriov_stop(struct amdxdna_dev_hdl *ndev);
#else
#define aie4_sriov_configure NULL
static inline int aie4_sriov_stop(struct amdxdna_dev_hdl *ndev)
{
	return 0;
}
#endif

extern const struct amdxdna_dev_ops aie4_ops;

#endif /* _AIE4_PCI_H_ */
