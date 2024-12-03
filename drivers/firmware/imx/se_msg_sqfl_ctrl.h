/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2024 NXP
 */

#ifndef SE_MSG_SQFL_CTRL_H
#define SE_MSG_SQFL_CTRL_H

#include <linux/mutex.h>

struct se_msg_seq_ctrl {
	struct mutex se_msg_sq_lk;
	void *exp_tx_msg;
};

void se_start_enforce_msg_seq_flow(struct se_msg_seq_ctrl *se_msg_sq_ctl);
void se_continue_to_enforce_msg_seq_flow(struct se_msg_seq_ctrl *se_msg_sq_ctl,
					 void *tx_msg);
void se_halt_to_enforce_msg_seq_flow(struct se_msg_seq_ctrl *se_msg_sq_ctl);
void se_qualify_msg_seq_flow(struct se_msg_seq_ctrl *se_msg_sq_ctl,
			     void *tx_msg);
#endif
