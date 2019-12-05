// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP
 *
 */

#include <linux/audit.h>

#include <soc/imx/imx-secvio-sc.h>

int report_to_audit_notify(struct notifier_block *nb, unsigned long status,
			   void *notif_info)
{
	int ret = 0;
	struct audit_buffer *ab;
	struct secvio_sc_notifier_info *info = notif_info;

	ab = audit_log_start(audit_context(), GFP_KERNEL, AUDIT_INTEGRITY_RULE);
	if (!ab) {
		ret = -ENOMEM;
		goto exit;
	}

	audit_log_format(ab, " hpsvs=0x%.08x lps=0x%.08x lptds=0x%.08x",
			 info->hpsvs, info->lps, info->lptds);
	audit_log_task_info(ab);
	audit_log_end(ab);

exit:
	return ret;
}
