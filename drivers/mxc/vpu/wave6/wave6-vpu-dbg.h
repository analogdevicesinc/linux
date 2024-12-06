/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave6 series multi-standard codec IP - debug interface
 *
 * Copyright (C) 2021 CHIPS&MEDIA INC
 */

#ifndef __WAVE6_VPU_DBG_H__
#define __WAVE6_VPU_DBG_H__

unsigned int wave6_vpu_debug(void);

#define dprintk(dev, fmt, arg...)					\
	do {								\
		if (wave6_vpu_debug())					\
			dev_info(dev, "%s: " fmt, __func__, ## arg);	\
	} while (0)

int wave6_vpu_create_dbgfs_file(struct vpu_instance *inst);
void wave6_vpu_remove_dbgfs_file(struct vpu_instance *inst);

#endif
