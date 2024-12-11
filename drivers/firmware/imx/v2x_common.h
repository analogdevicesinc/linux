/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2024 NXP
 */


#ifndef __V2X_COMMON_H__
#define __V2X_COMMON_H__

#include "se_ctrl.h"

#define V2X_FW_STATE_UNKNOWN		0x00
#define V2X_FW_STATE_RUNNING		0x15

int v2x_early_init(struct se_if_priv *priv);
int v2x_late_init(struct se_if_priv *priv);

int v2x_suspend(struct se_if_priv *priv);
int v2x_resume(struct se_if_priv *priv);
#endif /*__V2X_COMMON_H__ */
