/*
 * Copyright(c) 2018 NXP. All rights reserved.
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * vpu_encoder_mem.h
 *
 * Author Ming Qian<ming.qian@nxp.com>
 */
#ifndef _VPU_ENCODER_MEM_H
#define _VPU_ENCODER_MEM_H

#include "vpu_encoder_b0.h"

void vpu_enc_add_dma_size(struct vpu_attr *attr, unsigned long size);
void vpu_enc_sub_dma_size(struct vpu_attr *attr, unsigned long size);
int vpu_enc_alloc_dma_buffer(struct vpu_ctx *ctx, struct buffer_addr *buffer);
int vpu_enc_free_dma_buffer(struct vpu_ctx *ctx, struct buffer_addr *buffer);
void vpu_enc_init_dma_buffer(struct buffer_addr *buffer);
int vpu_enc_check_mem_overstep(struct vpu_ctx *ctx);
int vpu_enc_alloc_mem(struct vpu_ctx *ctx,
			MEDIAIP_ENC_MEM_REQ_DATA *req_data,
			pMEDIAIP_ENC_MEM_POOL pool);
int vpu_enc_free_mem(struct vpu_ctx *ctx, pMEDIAIP_ENC_MEM_POOL pool);
int vpu_enc_alloc_stream(struct vpu_ctx *ctx);
void vpu_enc_free_stream(struct vpu_ctx *ctx);

#endif
