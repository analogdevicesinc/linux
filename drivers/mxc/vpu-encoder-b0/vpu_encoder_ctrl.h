/*
 * Copyright(c) 2018 NXP. All rights reserved.
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * vpu_encoder_ctrl.h
 *
 * Author Ming Qian<ming.qian@nxp.com>
 */
#ifndef _VPU_ENCODER_CTRL_H
#define _VPU_ENCODER_CTRL_H

#include "mediasys_types.h"

int vpu_enc_setup_ctrls(struct vpu_ctx *ctx);
int vpu_enc_free_ctrls(struct vpu_ctx *ctx);

#endif
