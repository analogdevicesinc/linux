/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2019 NXP
 */

#ifndef SECVIO_SC_H
#define SECVIO_SC_H

/* Includes */
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/semaphore.h>
#include <linux/nvmem-consumer.h>
#include <linux/miscdevice.h>

/* Access for sc_seco_secvio_config API */
#define SECVIO_CONFIG_READ  0
#define SECVIO_CONFIG_WRITE 1

/* Internal Structure */
struct imx_secvio_sc_data {
	struct device *dev;

	struct imx_sc_ipc *ipc_handle;

	struct notifier_block irq_nb;
	struct notifier_block report_nb;
	struct notifier_block audit_nb;

	struct nvmem_device *nvmem;

	struct miscdevice miscdev;

#ifdef CONFIG_DEBUG_FS
	struct dentry *dfs;
#endif

	u32 version;
};

/* Function declarations */
extern
int call_secvio_config(struct device *dev, u8 id, u8 access, u32 *data0,
		       u32 *data1, u32 *data2, u32 *data3, u32 *data4, u8 size);

extern
int int_imx_secvio_sc_get_state(struct device *dev,
				struct secvio_sc_notifier_info *info);

extern
int int_imx_secvio_sc_clear_state(struct device *dev, u32 hpsvs, u32 lps,
				  u32 lptds);

extern
int int_imx_secvio_sc_enable_irq(struct device *dev);

extern
int int_imx_secvio_sc_disable_irq(struct device *dev);

#ifdef CONFIG_DEBUG_FS
extern
int imx_secvio_sc_debugfs(struct device *dev);
#else
static inline
int imx_secvio_sc_debugfs(struct device *dev)
{
	return 0;
}
#endif /* CONFIG_DEBUG_FS */

#ifdef CONFIG_AUDIT
int report_to_audit_notify(struct notifier_block *nb, unsigned long status,
			   void *notif_info);
#else /* CONFIG_AUDIT */
static inline
int report_to_audit_notify(struct notifier_block *nb, unsigned long status,
			   void *notif_info)
{
	return 0;
}
#endif /* CONFIG_AUDIT */

#endif /* SECVIO_SC_H */
