/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2024 NXP
 *
 * Header file for the V2X Base API(s).
 */

#ifndef V2X_BASE_MSG_H
#define V2X_BASE_MSG_H

#include <linux/types.h>

#define V2X_DBG_MU_MSG_RSP_TAG		0xE1

#define V2X_START_RNG_REQ		0x0E
#define V2X_START_RNG_REQ_MSG_SZ	0x04
#define V2X_START_RNG_RSP_MSG_SZ	0x0C

#define V2X_PWR_STATE			0x04
#define V2X_PWR_STATE_MSG_SZ		0x08
#define V2X_PWR_STATE_RSP_MSG_SZ	0x08

#define V2X_PWR_OFF_REQ			0x4
#define V2X_PWR_ON_REQ			0x1
#define V2X_PERM_DENIED_FAIL_IND	0xF329
#define V2X_INVAL_OPS_FAIL_IND		0xC029

#define V2X_DEBUG_MU_MSG_VERS		0x02
#define V2X_DEBUG_MU_MSG_CMD_TAG	0x17
#define V2X_DEBUG_MU_MSG_RSP_TAG	0xE1

#define V2X_MAX_DBG_DMP_PKT		15
#define V2X_NON_DUMP_BUFFER_SZ		2
#define V2X_DBG_DUMP_REQ		0x02
#define V2X_DBG_DUMP_MSG_SZ		0x08
#define V2X_DBG_DUMP_RSP_MSG_SZ		0x5c

int v2x_start_rng(struct se_if_priv *priv);
int v2x_pwr_state(struct se_if_priv *priv, u16 action);
int v2x_debug_dump(struct se_if_priv *priv);
#endif
