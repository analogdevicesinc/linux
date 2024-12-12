// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 NXP
 */

#include <linux/firmware/imx/se_api.h>
#include <uapi/linux/se_ioctl.h>

#include "ele_base_msg.h"
#include "ele_fw_api.h"
#include "v2x_base_msg.h"
#include "v2x_common.h"

static u32 v2x_fw_state;

#define V2X_FW_AUTH_DBG_COMPLETE	0x15
#define V2X_FW_AUTH_NORM_COMPLETE	0x13

static bool is_v2x_fw_running(u32 v2x_fw_state)
{
	if (v2x_fw_state == V2X_FW_AUTH_DBG_COMPLETE ||
			v2x_fw_state == V2X_FW_AUTH_NORM_COMPLETE)
		return true;
	return false;
}

int v2x_early_init(struct se_if_priv *priv)
{
	struct se_if_priv *ele_priv;
	int ret = 0;

	if (!is_v2x_fw_running(v2x_fw_state)) {
		ele_priv = imx_get_se_data_info(get_se_soc_id(priv), 0);
		if (ele_priv == NULL) {
			ret = -EPERM;
			goto exit;
		}
		ret = ele_get_v2x_fw_state(ele_priv, &v2x_fw_state);
		if (ret)
			dev_warn(priv->dev, "Failed to fetch the v2x-fw-state via ELE.");

		if (!is_v2x_fw_running(v2x_fw_state)) {
			dev_err(priv->dev,
				"failure: v2x fw state [0x%x]is not loaded.",
				v2x_fw_state);
			ret = -EPERM;
		}
	}
exit:

	return ret;
}

int v2x_late_init(struct se_if_priv *priv)
{
	int ret = 0;

	if (!is_v2x_fw_running(v2x_fw_state) &&
			priv->if_defs->se_if_type == SE_TYPE_ID_HSM) {
		ret = ele_init_fw(priv);
		if (ret) {
			dev_err(priv->dev, "ELE INIT FW failed.");
			ret = -EPERM;
			goto exit;
		}
		ret = ele_get_v2x_fw_state(priv, &v2x_fw_state);
		if (ret) {
			dev_warn(priv->dev, "Failed to fetch the v2x-fw-state via ELE.");
			v2x_fw_state = V2X_FW_STATE_UNKNOWN;
		}
		ret = 0;
	}
exit:
	return ret;
}
