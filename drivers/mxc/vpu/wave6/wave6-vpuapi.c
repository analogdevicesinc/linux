// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave6 series multi-standard codec IP - helper functions
 *
 * Copyright (C) 2021 CHIPS&MEDIA INC
 */

#include <linux/bug.h>
#include "wave6-vpuapi.h"
#include "wave6-regdefine.h"
#include "wave6.h"
#include "wave6-vpu-dbg.h"

static int wave6_check_dec_open_param(struct vpu_instance *inst, struct dec_open_param *param)
{
	struct vpu_attr *attr = &inst->dev->attr;

	if (param->bitstream_mode == BS_MODE_INTERRUPT)
		return -EINVAL;

	if (!(BIT(param->bitstream_mode) & attr->support_bitstream_mode))
		return -EINVAL;

	if (!(BIT(param->frame_endian) & attr->support_endian_mask))
		return -EINVAL;

	if (!(BIT(param->stream_endian) & attr->support_endian_mask))
		return -EINVAL;

	return 0;
}

int wave6_vpu_dec_open(struct vpu_instance *inst, struct dec_open_param *pop)
{
	struct dec_info *p_dec_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	ret = wave6_check_dec_open_param(inst, pop);
	if (ret)
		return ret;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	if (!wave6_vpu_is_init(vpu_dev)) {
		mutex_unlock(&vpu_dev->hw_lock);
		return -ENODEV;
	}

	inst->codec_info = kzalloc(sizeof(*inst->codec_info), GFP_KERNEL);
	if (!inst->codec_info) {
		mutex_unlock(&vpu_dev->hw_lock);
		return -ENOMEM;
	}

	p_dec_info = &inst->codec_info->dec_info;
	memcpy(&p_dec_info->open_param, pop, sizeof(struct dec_open_param));

	ret = wave6_vpu_build_up_dec_param(inst, pop);
	if (ret)
		goto free_codec_info;

	mutex_unlock(&vpu_dev->hw_lock);

	return 0;

free_codec_info:
	kfree(inst->codec_info);
	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave6_vpu_dec_close(struct vpu_instance *inst, u32 *fail_res)
{
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	*fail_res = 0;
	if (!inst->codec_info)
		return -EINVAL;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	ret = wave6_vpu_dec_fini_seq(inst, fail_res);
	if (ret) {
		dev_warn(inst->dev->dev, "dec seq end timed out\n");

		if (*fail_res == WAVE6_SYSERR_VPU_STILL_RUNNING) {
			mutex_unlock(&vpu_dev->hw_lock);
			return ret;
		}
	}

	dev_dbg(inst->dev->dev, "dec seq end complete\n");

	mutex_unlock(&vpu_dev->hw_lock);

	kfree(inst->codec_info);

	return 0;
}

int wave6_vpu_dec_issue_seq_init(struct vpu_instance *inst)
{
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	ret = wave6_vpu_dec_init_seq(inst);

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave6_vpu_dec_complete_seq_init(struct vpu_instance *inst, struct dec_initial_info *info)
{
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	ret = wave6_vpu_dec_get_seq_info(inst, info);
	if (!ret)
		p_dec_info->initial_info_obtained = true;

	info->rd_ptr = wave6_vpu_dec_get_rd_ptr(inst);
	info->wr_ptr = p_dec_info->stream_wr_ptr;

	p_dec_info->initial_info = *info;

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave6_vpu_dec_get_aux_buffer_size(struct vpu_instance *inst,
				      struct dec_aux_buffer_size_info info,
				      uint32_t *size)
{
	struct vpu_attr *attr = &inst->dev->attr;
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	int width = info.width;
	int height = info.height;
	int buf_size, twice;

	if (info.type == AUX_BUF_FBC_Y_TBL) {
		switch (inst->std) {
		case W_HEVC_DEC:
			buf_size = WAVE6_FBC_LUMA_TABLE_SIZE(width, height);
			break;
		case W_VP9_DEC:
			buf_size = WAVE6_FBC_LUMA_TABLE_SIZE(ALIGN(width, 64), ALIGN(height, 64));
			break;
		case W_AVC_DEC:
			buf_size = WAVE6_FBC_LUMA_TABLE_SIZE(width, height);
			break;
		case W_AV1_DEC:
			buf_size = WAVE6_FBC_LUMA_TABLE_SIZE(ALIGN(width, 16), ALIGN(height, 8));
			break;
		default:
			return -EINVAL;
		}
		buf_size = ALIGN(buf_size, 16);
	} else if (info.type == AUX_BUF_FBC_C_TBL) {
		if (p_dec_info->initial_info.chroma_format_idc == 2)
			twice = 2;
		else if (p_dec_info->initial_info.chroma_format_idc == 3)
			twice = 4;
		else
			twice = 1;

		switch (inst->std) {
		case W_HEVC_DEC:
			buf_size = WAVE6_FBC_CHROMA_TABLE_SIZE(width, height);
			break;
		case W_VP9_DEC:
			buf_size = WAVE6_FBC_CHROMA_TABLE_SIZE(ALIGN(width, 64), ALIGN(height, 64));
			break;
		case W_AVC_DEC:
			buf_size = WAVE6_FBC_CHROMA_TABLE_SIZE(width, height);
			break;
		case W_AV1_DEC:
			buf_size = WAVE6_FBC_CHROMA_TABLE_SIZE(ALIGN(width, 16), ALIGN(height, 8));
			break;
		default:
			return -EINVAL;
		}
		buf_size = buf_size * twice;
		buf_size = ALIGN(buf_size, 16);
	} else if (info.type == AUX_BUF_MV_COL) {
		switch (inst->std) {
		case W_HEVC_DEC:
			buf_size = WAVE6_DEC_HEVC_MVCOL_BUF_SIZE(width, height);
			break;
		case W_VP9_DEC:
			buf_size = WAVE6_DEC_VP9_MVCOL_BUF_SIZE_1(width, height) +
				WAVE6_DEC_VP9_MVCOL_BUF_SIZE_2(width, height);
			break;
		case W_AVC_DEC:
			buf_size = WAVE6_DEC_AVC_MVCOL_BUF_SIZE(width, height);
			break;
		case W_AV1_DEC:
			buf_size = WAVE6_DEC_AV1_MVCOL_BUF_SIZE_1(width, height) +
				WAVE6_DEC_AV1_MVCOL_BUF_SIZE_2(width, height);
			break;
		default:
			return -EINVAL;
		}
		buf_size = ALIGN(buf_size, 16);
	} else if (info.type == AUX_BUF_DEF_CDF) {
		if (inst->std == W_AV1_DEC)
			buf_size = WAVE6_AV1_DEFAULT_CDF_BUF_SIZE;
		else
			return -EINVAL;
	} else if (info.type == AUX_BUF_SEG_MAP) {
		if (inst->std != W_VP9_DEC)
			return -EINVAL;

		if (attr->support_command_queue)
			buf_size = WAVE6_VP9_SEGMAP_BUF_SIZE(width, height) *
				   (COMMAND_QUEUE_DEPTH + 3);
		else
			buf_size = WAVE6_VP9_SEGMAP_BUF_SIZE(width, height) * 2;
	} else if (info.type == AUX_BUF_PRE_ENT) {
		if (!attr->support_command_queue)
			return -EINVAL;

		switch (inst->std) {
		case W_VP9_DEC:
			buf_size = WAVE6_DEC_VP9_MVCOL_BUF_SIZE_1(width, height) +
				WAVE6_DEC_VP9_MVCOL_BUF_SIZE_2(width, height);
			break;
		case W_AV1_DEC:
			buf_size = WAVE6_DEC_AV1_MVCOL_BUF_SIZE_1(width, height) +
				WAVE6_DEC_AV1_MVCOL_BUF_SIZE_2(width, height);
			break;
		default:
			return -EINVAL;
		}
		buf_size = ALIGN(buf_size, 16);
	} else {
		return -EINVAL;
	}

	*size = buf_size;

	return 0;
}

int wave6_vpu_dec_register_aux_buffer(struct vpu_instance *inst,
				      struct aux_buffer_info info)
{
	struct dec_info *p_dec_info;
	struct aux_buffer *aux_bufs = info.buf_array;
	struct dec_aux_buffer_size_info size_info;
	unsigned int expected_size;
	unsigned int i;
	int ret;

	p_dec_info = &inst->codec_info->dec_info;

	size_info.width = p_dec_info->initial_info.pic_width;
	size_info.height = p_dec_info->initial_info.pic_height;
	size_info.type = info.type;

	ret = wave6_vpu_dec_get_aux_buffer_size(inst, size_info, &expected_size);
	if (ret)
		return ret;

	switch (info.type) {
	case AUX_BUF_FBC_Y_TBL:
		for (i = 0; i < info.num; i++) {
			if (expected_size > aux_bufs[i].size)
				return -EINVAL;

			p_dec_info->vb_fbc_y_tbl[aux_bufs[i].index].daddr = aux_bufs[i].addr;
			p_dec_info->vb_fbc_y_tbl[aux_bufs[i].index].size = aux_bufs[i].size;
		}
		break;
	case AUX_BUF_FBC_C_TBL:
		for (i = 0; i < info.num; i++) {
			if (expected_size > aux_bufs[i].size)
				return -EINVAL;

			p_dec_info->vb_fbc_c_tbl[aux_bufs[i].index].daddr = aux_bufs[i].addr;
			p_dec_info->vb_fbc_c_tbl[aux_bufs[i].index].size = aux_bufs[i].size;
		}
		break;
	case AUX_BUF_MV_COL:
		for (i = 0; i < info.num; i++) {
			if (expected_size > aux_bufs[i].size)
				return -EINVAL;

			p_dec_info->vb_mv[aux_bufs[i].index].daddr = aux_bufs[i].addr;
			p_dec_info->vb_mv[aux_bufs[i].index].size = aux_bufs[i].size;
		}
		break;
	case AUX_BUF_DEF_CDF:
		if (expected_size > aux_bufs[0].size)
			return -EINVAL;

		p_dec_info->vb_def_cdf.daddr = aux_bufs[0].addr;
		p_dec_info->vb_def_cdf.size = aux_bufs[0].size;
		break;
	case AUX_BUF_SEG_MAP:
		if (expected_size > aux_bufs[0].size)
			return -EINVAL;

		p_dec_info->vb_seg_map.daddr = aux_bufs[0].addr;
		p_dec_info->vb_seg_map.size = aux_bufs[0].size;
		break;
	case AUX_BUF_PRE_ENT:
		if (expected_size > aux_bufs[0].size)
			return -EINVAL;

		p_dec_info->vb_pre_ent.daddr = aux_bufs[0].addr;
		p_dec_info->vb_pre_ent.size = aux_bufs[0].size;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int wave6_vpu_dec_register_frame_buffer_ex(struct vpu_instance *inst,
					   int num_of_dec_fbs, int stride,
					   int height, int map_type)
{
	struct dec_info *p_dec_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;
	struct frame_buffer *fb;

	if (num_of_dec_fbs > WAVE6_MAX_FBS)
		return -EINVAL;

	p_dec_info = &inst->codec_info->dec_info;
	p_dec_info->stride = stride;

	if (!p_dec_info->initial_info_obtained)
		return -EINVAL;

	if (stride < p_dec_info->initial_info.pic_width || (stride % 8 != 0) ||
	    height < p_dec_info->initial_info.pic_height)
		return -EINVAL;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	fb = inst->frame_buf;
	ret = wave6_vpu_dec_register_frame_buffer(inst, &fb[0], COMPRESSED_FRAME_MAP,
						  num_of_dec_fbs);

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave6_vpu_dec_register_display_buffer_ex(struct vpu_instance *inst, struct frame_buffer fb)
{
	struct dec_info *p_dec_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	p_dec_info = &inst->codec_info->dec_info;

	if (!p_dec_info->initial_info_obtained)
		return -EINVAL;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	ret = wave6_vpu_dec_register_display_buffer(inst, fb);

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave6_vpu_dec_update_frame_buffer(struct vpu_instance *inst, struct frame_buffer *fb, int mv_index)
{
	struct vpu_device *vpu_dev = inst->dev;
	int ret;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	ret = wave6_vpu_dec_update_fb(inst, fb, mv_index);

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave6_vpu_dec_get_update_frame_buffer_info(struct vpu_instance *inst,
					       struct dec_update_fb_info *info)
{
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	if (!info)
		return -EINVAL;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	memset(info, 0, sizeof(*info));

	ret = wave6_vpu_dec_get_update_fb_info(inst, info);

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave6_vpu_dec_get_bitstream_buffer(struct vpu_instance *inst, dma_addr_t *prd_ptr,
				       dma_addr_t *pwr_ptr, uint32_t *size)
{
	struct dec_info *p_dec_info;
	dma_addr_t rd_ptr;
	dma_addr_t wr_ptr;
	int room = 0;
	struct vpu_device *vpu_dev = inst->dev;
	int ret;

	p_dec_info = &inst->codec_info->dec_info;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;
	rd_ptr = wave6_vpu_dec_get_rd_ptr(inst);
	mutex_unlock(&vpu_dev->hw_lock);

	wr_ptr = p_dec_info->stream_wr_ptr;

	if (prd_ptr)
		*prd_ptr = rd_ptr;
	if (pwr_ptr)
		*pwr_ptr = wr_ptr;
	if (size)
		*size = room;

	return 0;
}

int wave6_vpu_dec_update_bitstream_buffer(struct vpu_instance *inst, int size)
{
	struct dec_info *p_dec_info;
	dma_addr_t wr_ptr;
	dma_addr_t rd_ptr;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	if (!inst->codec_info)
		return -EINVAL;

	p_dec_info = &inst->codec_info->dec_info;
	wr_ptr = p_dec_info->stream_wr_ptr;
	rd_ptr = p_dec_info->stream_rd_ptr;

	if (size > 0) {
		if (wr_ptr < rd_ptr && rd_ptr <= wr_ptr + size)
			return -EINVAL;

		wr_ptr += size;

		p_dec_info->stream_wr_ptr = wr_ptr;
		p_dec_info->stream_rd_ptr = rd_ptr;
	}

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;
	ret = wave6_vpu_dec_set_bitstream_flag(inst, (size == 0));
	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave6_vpu_dec_start_one_frame(struct vpu_instance *inst, struct dec_param *param, u32 *res_fail)
{
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	if (p_dec_info->stride == 0) // this means frame buffers have not been registered.
		return -EINVAL;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	ret = wave6_vpu_decode(inst, param, res_fail);

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave6_vpu_dec_set_rd_ptr(struct vpu_instance *inst, dma_addr_t addr, bool update_wr_ptr)
{
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	p_dec_info->stream_rd_ptr = addr;
	if (update_wr_ptr)
		p_dec_info->stream_wr_ptr = addr;

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave6_vpu_dec_get_output_info(struct vpu_instance *inst, struct dec_output_info *info)
{
	struct dec_info *p_dec_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	if (!info)
		return -EINVAL;

	p_dec_info = &inst->codec_info->dec_info;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	memset(info, 0, sizeof(*info));

	ret = wave6_vpu_dec_get_result(inst, info);
	if (ret) {
		info->rd_ptr = p_dec_info->stream_rd_ptr;
		info->wr_ptr = p_dec_info->stream_wr_ptr;
		goto err_out;
	}

err_out:
	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave6_vpu_dec_give_command(struct vpu_instance *inst, enum codec_command cmd, void *param)
{
	struct dec_info *p_dec_info;

	if (!inst || !inst->codec_info)
		return -EINVAL;

	p_dec_info = &inst->codec_info->dec_info;

	switch (cmd) {
	case ENABLE_DEC_THUMBNAIL_MODE:
		p_dec_info->thumbnail_mode = 1;
		break;
	case DEC_RESET_FRAMEBUF_INFO: {
		int i;

		for (i = 0; i < WAVE6_MAX_FBS; i++) {
			wave6_vdi_free_dma_memory(inst->dev, &inst->frame_vbuf[i]);
			memset(&inst->frame_buf[i], 0, sizeof(struct frame_buffer));
			memset(&p_dec_info->disp_buf[i], 0, sizeof(struct frame_buffer));

			wave6_vdi_free_dma_memory(inst->dev, &inst->aux_vbuf[AUX_BUF_MV_COL][i]);
			memset(&p_dec_info->vb_mv[i], 0, sizeof(struct vpu_buf));

			wave6_vdi_free_dma_memory(inst->dev, &inst->aux_vbuf[AUX_BUF_FBC_Y_TBL][i]);
			memset(&p_dec_info->vb_fbc_y_tbl[i], 0, sizeof(struct vpu_buf));

			wave6_vdi_free_dma_memory(inst->dev, &inst->aux_vbuf[AUX_BUF_FBC_C_TBL][i]);
			memset(&p_dec_info->vb_fbc_c_tbl[i], 0, sizeof(struct vpu_buf));

			wave6_vdi_free_dma_memory(inst->dev, &inst->aux_vbuf[AUX_BUF_DEF_CDF][i]);
			memset(&p_dec_info->vb_def_cdf, 0, sizeof(struct vpu_buf));

			wave6_vdi_free_dma_memory(inst->dev, &inst->aux_vbuf[AUX_BUF_SEG_MAP][i]);
			memset(&p_dec_info->vb_seg_map, 0, sizeof(struct vpu_buf));

			wave6_vdi_free_dma_memory(inst->dev, &inst->aux_vbuf[AUX_BUF_PRE_ENT][i]);
			memset(&p_dec_info->vb_pre_ent, 0, sizeof(struct vpu_buf));
		}
		break;
	}
	case DEC_GET_SEQ_INFO: {
		struct dec_initial_info *seq_info = param;

		*seq_info = p_dec_info->initial_info;
		break;
	}

	default:
		return -EINVAL;
	}

	return 0;
}

int wave6_vpu_dec_flush_instance(struct vpu_instance *inst)
{
	struct vpu_device *vpu_dev = inst->dev;
	int ret;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	ret = wave6_vpu_dec_flush(inst);

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave6_vpu_enc_open(struct vpu_instance *inst, struct enc_open_param *pop)
{
	struct enc_info *p_enc_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	ret = wave6_vpu_enc_check_open_param(inst, pop);
	if (ret)
		return ret;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	if (!wave6_vpu_is_init(vpu_dev)) {
		mutex_unlock(&vpu_dev->hw_lock);
		return -ENODEV;
	}

	inst->codec_info = kzalloc(sizeof(*inst->codec_info), GFP_KERNEL);
	if (!inst->codec_info) {
		mutex_unlock(&vpu_dev->hw_lock);
		return -ENOMEM;
	}

	p_enc_info = &inst->codec_info->enc_info;
	p_enc_info->open_param = *pop;

	ret = wave6_vpu_build_up_enc_param(vpu_dev->dev, inst, pop);
	if (ret)
		goto free_codec_info;
	mutex_unlock(&vpu_dev->hw_lock);

	return 0;

free_codec_info:
	kfree(inst->codec_info);
	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave6_vpu_enc_close(struct vpu_instance *inst, u32 *fail_res)
{
	int ret;
	struct vpu_device *vpu_dev = inst->dev;
	*fail_res = 0;

	if (!inst->codec_info)
		return -EINVAL;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	ret = wave6_vpu_enc_fini_seq(inst, fail_res);
	if (ret) {
		dev_warn(inst->dev->dev, "enc seq end timed out\n");

		if (*fail_res == WAVE6_SYSERR_VPU_STILL_RUNNING) {
			mutex_unlock(&vpu_dev->hw_lock);
			return ret;
		}
	}

	dev_dbg(inst->dev->dev, "enc seq end timed out\n");

	mutex_unlock(&vpu_dev->hw_lock);

	kfree(inst->codec_info);

	return 0;
}

int wave6_vpu_enc_get_aux_buffer_size(struct vpu_instance *inst,
				      struct enc_aux_buffer_size_info info,
				      uint32_t *size)
{
	struct enc_info *p_enc_info = &inst->codec_info->enc_info;
	int width, height, buf_size, twice;

	if (inst->std == W_AVC_ENC) {
		width  = ALIGN(info.width, 16);
		height = ALIGN(info.height, 16);
		if (info.rotation_angle == 90 || info.rotation_angle == 270) {
			width  = ALIGN(info.height, 16);
			height = ALIGN(info.width, 16);
		}
	} else {
		width  = ALIGN(info.width, 8);
		height = ALIGN(info.height, 8);
		if ((info.rotation_angle != 0 || info.mirror_direction != 0) &&
		    !(info.rotation_angle == 180 && info.mirror_direction == MIRDIR_HOR_VER)) {
			width  = ALIGN(info.width, 32);
			height = ALIGN(info.height, 32);
		}
		if (info.rotation_angle == 90 || info.rotation_angle == 270) {
			width  = ALIGN(info.height, 32);
			height = ALIGN(info.width, 32);
		}
	}

	if (info.type == AUX_BUF_FBC_Y_TBL) {
		switch (inst->std) {
		case W_HEVC_ENC:
			buf_size = WAVE6_FBC_LUMA_TABLE_SIZE(width, height);
			break;
		case W_AVC_ENC:
			buf_size = WAVE6_FBC_LUMA_TABLE_SIZE(width, height);
			break;
		case W_AV1_ENC:
			buf_size = WAVE6_FBC_LUMA_TABLE_SIZE(width, height);
			break;
		default:
			return -EINVAL;
		}
	} else if (info.type == AUX_BUF_FBC_C_TBL) {
		switch (p_enc_info->open_param.output_format) {
		case FORMAT_422:
		case FORMAT_422_P10_16BIT_MSB:
		case FORMAT_422_P10_16BIT_LSB:
		case FORMAT_422_P10_32BIT_MSB:
		case FORMAT_422_P10_32BIT_LSB:
			twice = 2;
			break;
		case FORMAT_444:
		case FORMAT_444_P10_16BIT_MSB:
		case FORMAT_444_P10_16BIT_LSB:
		case FORMAT_444_P10_32BIT_MSB:
		case FORMAT_444_P10_32BIT_LSB:
			twice = 4;
			break;
		default:
			twice = 1;
			break;
		}
		switch (inst->std) {
		case W_HEVC_ENC:
			buf_size = WAVE6_FBC_CHROMA_TABLE_SIZE(width, height);
			break;
		case W_AVC_ENC:
			buf_size = WAVE6_FBC_CHROMA_TABLE_SIZE(width, height);
			break;
		case W_AV1_ENC:
			buf_size = WAVE6_FBC_CHROMA_TABLE_SIZE(width, height);
			break;
		default:
			return -EINVAL;
		}
		buf_size = buf_size * twice;
	} else if (info.type == AUX_BUF_MV_COL) {
		switch (inst->std) {
		case W_HEVC_ENC:
			buf_size = WAVE6_ENC_HEVC_MVCOL_BUF_SIZE(width, height);
			break;
		case W_AVC_ENC:
			buf_size = WAVE6_ENC_AVC_MVCOL_BUF_SIZE(width, height);
			break;
		case W_AV1_ENC:
			buf_size = WAVE6_ENC_AV1_MVCOL_BUF_SIZE;
			break;
		default:
			return -EINVAL;
		}
	} else if (info.type == AUX_BUF_DEF_CDF) {
		if (inst->std == W_AV1_ENC)
			buf_size = WAVE6_AV1_DEFAULT_CDF_BUF_SIZE;
		else
			return -EINVAL;
	} else if (info.type == AUX_BUF_SUB_SAMPLE) {
		switch (inst->std) {
		case W_HEVC_ENC:
		case W_AVC_ENC:
		case W_AV1_ENC:
			buf_size = WAVE6_ENC_SUBSAMPLED_SIZE(width, height);
			break;
		default:
			return -EINVAL;
		}
	} else {
		return -EINVAL;
	}

	*size = buf_size;

	return 0;
}

int wave6_vpu_enc_register_aux_buffer(struct vpu_instance *inst,
				      struct aux_buffer_info info)
{
	struct enc_info *p_enc_info;
	struct aux_buffer *aux_bufs = info.buf_array;
	struct enc_aux_buffer_size_info size_info;
	unsigned int expected_size;
	unsigned int i;
	int ret;

	p_enc_info = &inst->codec_info->enc_info;

	size_info.width = p_enc_info->width;
	size_info.height = p_enc_info->height;
	size_info.type = info.type;
	size_info.rotation_angle = p_enc_info->rotation_angle;
	size_info.mirror_direction = p_enc_info->mirror_direction;

	ret = wave6_vpu_enc_get_aux_buffer_size(inst, size_info, &expected_size);
	if (ret)
		return ret;

	switch (info.type) {
	case AUX_BUF_FBC_Y_TBL:
		for (i = 0; i < info.num; i++) {
			if (expected_size > aux_bufs[i].size)
				return -EINVAL;

			p_enc_info->vb_fbc_y_tbl[aux_bufs[i].index].daddr = aux_bufs[i].addr;
			p_enc_info->vb_fbc_y_tbl[aux_bufs[i].index].size = aux_bufs[i].size;
		}
		break;
	case AUX_BUF_FBC_C_TBL:
		for (i = 0; i < info.num; i++) {
			if (expected_size > aux_bufs[i].size)
				return -EINVAL;

			p_enc_info->vb_fbc_c_tbl[aux_bufs[i].index].daddr = aux_bufs[i].addr;
			p_enc_info->vb_fbc_c_tbl[aux_bufs[i].index].size = aux_bufs[i].size;
		}
		break;
	case AUX_BUF_MV_COL:
		for (i = 0; i < info.num; i++) {
			if (expected_size > aux_bufs[i].size)
				return -EINVAL;

			p_enc_info->vb_mv[aux_bufs[i].index].daddr = aux_bufs[i].addr;
			p_enc_info->vb_mv[aux_bufs[i].index].size = aux_bufs[i].size;
		}
		break;
	case AUX_BUF_DEF_CDF:
		if (expected_size > aux_bufs[0].size)
			return -EINVAL;

		p_enc_info->vb_def_cdf.daddr = aux_bufs[0].addr;
		p_enc_info->vb_def_cdf.size = aux_bufs[0].size;
		break;
	case AUX_BUF_SUB_SAMPLE:
		for (i = 0; i < info.num; i++) {
			if (expected_size > aux_bufs[i].size)
				return -EINVAL;

			p_enc_info->vb_sub_sam_buf[aux_bufs[i].index].daddr = aux_bufs[i].addr;
			p_enc_info->vb_sub_sam_buf[aux_bufs[i].index].size = aux_bufs[i].size;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int wave6_vpu_enc_register_frame_buffer_ex(struct vpu_instance *inst, int num, unsigned int stride,
					int height, enum tiled_map_type map_type)
{
	struct enc_info *p_enc_info = &inst->codec_info->enc_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	if (p_enc_info->stride)
		return -EINVAL;

	if (!p_enc_info->initial_info_obtained)
		return -EINVAL;

	if (num < p_enc_info->initial_info.min_frame_buffer_count)
		return -EINVAL;

	if (stride == 0 || stride % 8 != 0)
		return -EINVAL;

	if (height <= 0)
		return -EINVAL;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	p_enc_info->num_frame_buffers = num;
	p_enc_info->stride = stride;

	ret = wave6_vpu_enc_register_frame_buffer(inst, &inst->frame_buf[0]);

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

static int wave6_check_enc_param(struct vpu_instance *inst, struct enc_param *param)
{
	struct enc_info *p_enc_info = &inst->codec_info->enc_info;
	bool is_rgb_format = false;

	if (!param)
		return -EINVAL;

	if (!param->skip_picture && !param->source_frame)
		return -EINVAL;

	if (p_enc_info->open_param.wave_param.enc_bit_rate == 0 && inst->std == W_HEVC_ENC) {
		if (param->force_pic_qp_enable == 1) {
			if (param->force_pic_qp_i < 0 || param->force_pic_qp_i > 63)
				return -EINVAL;

			if (param->force_pic_qp_p < 0 || param->force_pic_qp_p > 63)
				return -EINVAL;

			if (param->force_pic_qp_b < 0 || param->force_pic_qp_b > 63)
				return -EINVAL;
		}
		if ((param->pic_stream_buffer_addr % 16 || param->pic_stream_buffer_size == 0))
			return -EINVAL;
	}

	if ((param->pic_stream_buffer_addr % 8 || param->pic_stream_buffer_size == 0))
		return -EINVAL;

	if ((p_enc_info->open_param.src_format == FORMAT_RGB_32BIT_PACKED) ||
	    (p_enc_info->open_param.src_format == FORMAT_RGB_P10_32BIT_PACKED) ||
	    (p_enc_info->open_param.src_format == FORMAT_RGB_24BIT_PACKED))
		is_rgb_format = true;

	if (is_rgb_format) {
		if (param->csc.coef_ry < -512 || param->csc.coef_ry > 511)
			return -EINVAL;
		if (param->csc.coef_gy < -512 || param->csc.coef_gy > 511)
			return -EINVAL;
		if (param->csc.coef_by < -512 || param->csc.coef_by > 511)
			return -EINVAL;
		if (param->csc.coef_rcb < -512 || param->csc.coef_rcb > 511)
			return -EINVAL;
		if (param->csc.coef_gcb < -512 || param->csc.coef_gcb > 511)
			return -EINVAL;
		if (param->csc.coef_bcb < -512 || param->csc.coef_bcb > 511)
			return -EINVAL;
		if (param->csc.coef_rcr < -512 || param->csc.coef_rcr > 511)
			return -EINVAL;
		if (param->csc.coef_gcr < -512 || param->csc.coef_gcr > 511)
			return -EINVAL;
		if (param->csc.coef_bcr < -512 || param->csc.coef_bcr > 511)
			return -EINVAL;
		if (param->csc.offset_y > 1023)
			return -EINVAL;
		if (param->csc.offset_cb > 1023)
			return -EINVAL;
		if (param->csc.offset_cr > 1023)
			return -EINVAL;
	}

	if (inst->std == W_AVC_ENC) {
		if (param->intra_4x4 != 0)
			return -EINVAL;
	} else {
		if (param->intra_4x4 > 3 || param->intra_4x4 == 1)
			return -EINVAL;
	}

	return 0;
}

static uint64_t wave6_get_timestamp(struct vpu_instance *inst)
{
	struct enc_info *p_enc_info;
	u64 pts;
	u32 fps;

	if (!inst->codec_info)
		return 0;

	p_enc_info = &inst->codec_info->enc_info;
	fps = p_enc_info->open_param.wave_param.frame_rate;
	if (fps == 0)
		fps = 30;

	pts = p_enc_info->cur_pts;
	p_enc_info->cur_pts += 90000 / fps; /* 90_k_hz/fps */

	return pts;
}

int wave6_vpu_enc_start_one_frame(struct vpu_instance *inst, struct enc_param *param, u32 *fail_res)
{
	struct enc_info *p_enc_info = &inst->codec_info->enc_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	*fail_res = 0;

	if (p_enc_info->stride == 0) // this means frame buffers have not been registered.
		return -EINVAL;

	ret = wave6_check_enc_param(inst, param);
	if (ret)
		return ret;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	p_enc_info->pts_map[param->src_idx] = p_enc_info->open_param.enable_pts ?
					      wave6_get_timestamp(inst) : param->pts;

	ret = wave6_vpu_encode(inst, param, fail_res);

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave6_vpu_enc_get_output_info(struct vpu_instance *inst, struct enc_output_info *info)
{
	struct enc_info *p_enc_info = &inst->codec_info->enc_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	ret = wave6_vpu_enc_get_result(inst, info);
	if (ret) {
		info->pts = 0;
		goto unlock;
	}

	if (info->recon_frame_index >= 0)
		info->pts = p_enc_info->pts_map[info->enc_src_idx];

unlock:
	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave6_vpu_enc_give_command(struct vpu_instance *inst, enum codec_command cmd, void *param)
{
	struct enc_info *p_enc_info;

	if (!inst || !inst->codec_info)
		return -EINVAL;

	p_enc_info = &inst->codec_info->enc_info;

	switch (cmd) {
	case ENABLE_ROTATION:
		p_enc_info->rotation_enable = true;
		break;
	case ENABLE_MIRRORING:
		p_enc_info->mirror_enable = true;
		break;
	case SET_MIRROR_DIRECTION: {
		enum mirror_direction mir_dir;

		mir_dir = *(enum mirror_direction *)param;
		if (mir_dir != MIRDIR_NONE && mir_dir != MIRDIR_HOR &&
		    mir_dir != MIRDIR_VER && mir_dir != MIRDIR_HOR_VER)
			return -EINVAL;
		p_enc_info->mirror_direction = mir_dir;
		break;
	}
	case SET_ROTATION_ANGLE: {
		int angle;

		angle = *(int *)param;
		if (angle && angle != 90 && angle != 180 && angle != 270)
			return -EINVAL;
		if (p_enc_info->initial_info_obtained && (angle == 90 || angle == 270))
			return -EINVAL;
		p_enc_info->rotation_angle = angle;
		break;
	}
	default:
		return -EINVAL;
	}
	return 0;
}

int wave6_vpu_enc_issue_seq_init(struct vpu_instance *inst)
{
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	ret = wave6_vpu_enc_init_seq(inst);

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave6_vpu_enc_update_seq(struct vpu_instance *inst)
{
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	ret = wave6_vpu_enc_change_seq(inst);

	mutex_unlock(&vpu_dev->hw_lock);

	return ret;
}

int wave6_vpu_enc_complete_seq_init(struct vpu_instance *inst, struct enc_initial_info *info)
{
	struct enc_info *p_enc_info = &inst->codec_info->enc_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	if (!info)
		return -EINVAL;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	ret = wave6_vpu_enc_get_seq_info(inst, info);
	if (ret) {
		p_enc_info->initial_info_obtained = false;
		mutex_unlock(&vpu_dev->hw_lock);
		return ret;
	}

	p_enc_info->initial_info_obtained = true;
	p_enc_info->initial_info = *info;

	mutex_unlock(&vpu_dev->hw_lock);

	return 0;
}

int wave6_vpu_enc_complete_seq_update(struct vpu_instance *inst, struct enc_initial_info *info)
{
	struct enc_info *p_enc_info = &inst->codec_info->enc_info;
	int ret;
	struct vpu_device *vpu_dev = inst->dev;

	if (!info)
		return -EINVAL;

	ret = mutex_lock_interruptible(&vpu_dev->hw_lock);
	if (ret)
		return ret;

	ret = wave6_vpu_enc_get_seq_info(inst, info);
	if (ret) {
		p_enc_info->initial_info_obtained = false;
		mutex_unlock(&vpu_dev->hw_lock);
		return ret;
	}

	mutex_unlock(&vpu_dev->hw_lock);

	return 0;
}


const char *wave6_vpu_instance_state_name(u32 state)
{
	switch (state) {
	case VPU_INST_STATE_NONE: return "none";
	case VPU_INST_STATE_OPEN: return "open";
	case VPU_INST_STATE_INIT_SEQ: return "init_seq";
	case VPU_INST_STATE_PIC_RUN: return "pic_run";
	case VPU_INST_STATE_SEEK: return "seek";
	case VPU_INST_STATE_STOP: return "stop";
	}
	return "unknown";
}

void wave6_vpu_set_instance_state(struct vpu_instance *inst, u32 state)
{
	dprintk(inst->dev->dev, "[%d] %s -> %s\n",
		inst->id,
		wave6_vpu_instance_state_name(inst->state),
		wave6_vpu_instance_state_name(state));

	inst->state = state;
	if (state == VPU_INST_STATE_PIC_RUN && !inst->performance.ts_first)
		inst->performance.ts_first = ktime_get_raw();
}

void wave6_vpu_wait_active(struct vpu_instance *inst)
{
	wave6_vpu_check_state(inst->dev);
}
