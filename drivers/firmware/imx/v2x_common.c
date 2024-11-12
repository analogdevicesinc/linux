// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 NXP
 */

#include "ele_base_msg.h"
#include "v2x_common.h"
#include <uapi/linux/se_ioctl.h>

static u32 v2x_fw_state;

int v2x_early_init(struct se_if_priv *priv)
{
	if (v2x_fw_state != V2X_FW_STATE_RUNNING) {
		dev_err(priv->dev, "Failure: V2X FW is not loaded.");
		return -EPERM;
	}

	return 0;
}

int v2x_late_init(struct se_if_priv *priv)
{
	int ret;

	if (v2x_fw_state == V2X_FW_STATE_UNKNOWN &&
			priv->if_defs->se_if_type == SE_TYPE_ID_HSM) {
		ret = ele_get_v2x_fw_state(priv, &v2x_fw_state);
		if (ret) {
			v2x_fw_state = V2X_FW_STATE_UNKNOWN;
			dev_warn(priv->dev, "Warning: V2X FW is not loaded.");
		}
	}

	return 0;
}
