// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/kthread.h>

#include "pfe_mod.h"
#include "pfe_ctrl.h"

#define TIMEOUT_MS	1000

int relax(unsigned long end)
{
	if (time_after(jiffies, end)) {
		if (time_after(jiffies, end + (TIMEOUT_MS * HZ) / 1000))
			return -1;

		if (need_resched())
			schedule();
	}

	return 0;
}

void pfe_ctrl_suspend(struct pfe_ctrl *ctrl)
{
	int id;

	mutex_lock(&ctrl->mutex);

	for (id = CLASS0_ID; id <= CLASS_MAX_ID; id++)
		pe_dmem_write(id, cpu_to_be32(0x1), CLASS_DM_RESUME, 4);

	for (id = TMU0_ID; id <= TMU_MAX_ID; id++) {
		if (id == TMU2_ID)
			continue;
		pe_dmem_write(id, cpu_to_be32(0x1), TMU_DM_RESUME, 4);
	}

#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)
	pe_dmem_write(UTIL_ID, cpu_to_be32(0x1), UTIL_DM_RESUME, 4);
#endif
	mutex_unlock(&ctrl->mutex);
}

void pfe_ctrl_resume(struct pfe_ctrl *ctrl)
{
	int pe_mask = CLASS_MASK | TMU_MASK;

#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)
	pe_mask |= UTIL_MASK;
#endif
	mutex_lock(&ctrl->mutex);
	pe_start(&pfe->ctrl, pe_mask);
	mutex_unlock(&ctrl->mutex);
}

/* PE sync stop.
 * Stops packet processing for a list of PE's (specified using a bitmask).
 * The caller must hold ctrl->mutex.
 *
 * @param ctrl		Control context
 * @param pe_mask	Mask of PE id's to stop
 *
 */
int pe_sync_stop(struct pfe_ctrl *ctrl, int pe_mask)
{
	struct pe_sync_mailbox *mbox;
	int pe_stopped = 0;
	unsigned long end = jiffies + 2;
	int i;

	pe_mask &= 0x2FF;  /*Exclude Util + TMU2 */

	for (i = 0; i < MAX_PE; i++)
		if (pe_mask & (1 << i)) {
			mbox = (void *)ctrl->sync_mailbox_baseaddr[i];

			pe_dmem_write(i, cpu_to_be32(0x1), (unsigned
					long)&mbox->stop, 4);
		}

	while (pe_stopped != pe_mask) {
		for (i = 0; i < MAX_PE; i++)
			if ((pe_mask & (1 << i)) && !(pe_stopped & (1 << i))) {
				mbox = (void *)ctrl->sync_mailbox_baseaddr[i];

				if (pe_dmem_read(i, (unsigned
					long)&mbox->stopped, 4) &
					cpu_to_be32(0x1))
					pe_stopped |= (1 << i);
			}

		if (relax(end) < 0)
			goto err;
	}

	return 0;

err:
	pr_err("%s: timeout, %x %x\n", __func__, pe_mask, pe_stopped);

	for (i = 0; i < MAX_PE; i++)
		if (pe_mask & (1 << i)) {
			mbox = (void *)ctrl->sync_mailbox_baseaddr[i];

			pe_dmem_write(i, cpu_to_be32(0x0), (unsigned
					long)&mbox->stop, 4);
	}

	return -EIO;
}

/* PE start.
 * Starts packet processing for a list of PE's (specified using a bitmask).
 * The caller must hold ctrl->mutex.
 *
 * @param ctrl		Control context
 * @param pe_mask	Mask of PE id's to start
 *
 */
void pe_start(struct pfe_ctrl *ctrl, int pe_mask)
{
	struct pe_sync_mailbox *mbox;
	int i;

	for (i = 0; i < MAX_PE; i++)
		if (pe_mask & (1 << i)) {
			mbox = (void *)ctrl->sync_mailbox_baseaddr[i];

			pe_dmem_write(i, cpu_to_be32(0x0), (unsigned
					long)&mbox->stop, 4);
		}
}

/* This function will ensure all PEs are put in to idle state */
int pe_reset_all(struct pfe_ctrl *ctrl)
{
	struct pe_sync_mailbox *mbox;
	int pe_stopped = 0;
	unsigned long end = jiffies + 2;
	int i;
	int pe_mask  = CLASS_MASK | TMU_MASK;

#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)
	pe_mask |= UTIL_MASK;
#endif

	for (i = 0; i < MAX_PE; i++)
		if (pe_mask & (1 << i)) {
			mbox = (void *)ctrl->sync_mailbox_baseaddr[i];

			pe_dmem_write(i, cpu_to_be32(0x2), (unsigned
					long)&mbox->stop, 4);
		}

	while (pe_stopped != pe_mask) {
		for (i = 0; i < MAX_PE; i++)
			if ((pe_mask & (1 << i)) && !(pe_stopped & (1 << i))) {
				mbox = (void *)ctrl->sync_mailbox_baseaddr[i];

				if (pe_dmem_read(i, (unsigned long)
							&mbox->stopped, 4) &
						cpu_to_be32(0x1))
					pe_stopped |= (1 << i);
			}

		if (relax(end) < 0)
			goto err;
	}

	return 0;

err:
	pr_err("%s: timeout, %x %x\n", __func__, pe_mask, pe_stopped);
	return -EIO;
}

int pfe_ctrl_init(struct pfe *pfe)
{
	struct pfe_ctrl *ctrl = &pfe->ctrl;
	int id;

	pr_info("%s\n", __func__);

	mutex_init(&ctrl->mutex);
	spin_lock_init(&ctrl->lock);

	for (id = CLASS0_ID; id <= CLASS_MAX_ID; id++) {
		ctrl->sync_mailbox_baseaddr[id] = CLASS_DM_SYNC_MBOX;
		ctrl->msg_mailbox_baseaddr[id] = CLASS_DM_MSG_MBOX;
	}

	for (id = TMU0_ID; id <= TMU_MAX_ID; id++) {
		if (id == TMU2_ID)
			continue;
		ctrl->sync_mailbox_baseaddr[id] = TMU_DM_SYNC_MBOX;
		ctrl->msg_mailbox_baseaddr[id] = TMU_DM_MSG_MBOX;
	}

#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)
	ctrl->sync_mailbox_baseaddr[UTIL_ID] = UTIL_DM_SYNC_MBOX;
	ctrl->msg_mailbox_baseaddr[UTIL_ID] = UTIL_DM_MSG_MBOX;
#endif

	ctrl->hash_array_baseaddr = pfe->ddr_baseaddr + ROUTE_TABLE_BASEADDR;
	ctrl->hash_array_phys_baseaddr = pfe->ddr_phys_baseaddr +
						ROUTE_TABLE_BASEADDR;

	ctrl->dev = pfe->dev;

	pr_info("%s finished\n", __func__);

	return 0;
}

void pfe_ctrl_exit(struct pfe *pfe)
{
	pr_info("%s\n", __func__);
}
