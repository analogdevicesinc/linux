// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave6 series multi-standard codec IP - decoder interface
 *
 * Copyright (C) 2021 CHIPS&MEDIA INC
 */

#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/swiotlb.h>
#include "wave6-vpu.h"
#include "wave6-vpu-dbg.h"

#define VPU_DEC_DEV_NAME "C&M Wave6 VPU decoder"
#define VPU_DEC_DRV_NAME "wave6-dec"
#define V4L2_CID_VPU_THUMBNAIL_MODE (V4L2_CID_USER_BASE + 0x1001)

static const struct vpu_format wave6_vpu_dec_fmt_list[2][6] = {
	[VPU_FMT_TYPE_CODEC] = {
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_HEVC,
			.max_width = W6_MAX_DEC_PIC_WIDTH,
			.min_width = W6_MIN_DEC_PIC_WIDTH,
			.max_height = W6_MAX_DEC_PIC_HEIGHT,
			.min_height = W6_MIN_DEC_PIC_HEIGHT,
			.num_planes = 1,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_H264,
			.max_width = W6_MAX_DEC_PIC_WIDTH,
			.min_width = W6_MIN_DEC_PIC_WIDTH,
			.max_height = W6_MAX_DEC_PIC_HEIGHT,
			.min_height = W6_MIN_DEC_PIC_HEIGHT,
			.num_planes = 1,
		},
	},
	[VPU_FMT_TYPE_RAW] = {
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_YUV420,
			.max_width = W6_MAX_DEC_PIC_WIDTH,
			.min_width = W6_MIN_DEC_PIC_WIDTH,
			.max_height = W6_MAX_DEC_PIC_HEIGHT,
			.min_height = W6_MIN_DEC_PIC_HEIGHT,
			.num_planes = 1,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV12,
			.max_width = W6_MAX_DEC_PIC_WIDTH,
			.min_width = W6_MIN_DEC_PIC_WIDTH,
			.max_height = W6_MAX_DEC_PIC_HEIGHT,
			.min_height = W6_MIN_DEC_PIC_HEIGHT,
			.num_planes = 1,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV21,
			.max_width = W6_MAX_DEC_PIC_WIDTH,
			.min_width = W6_MIN_DEC_PIC_WIDTH,
			.max_height = W6_MAX_DEC_PIC_HEIGHT,
			.min_height = W6_MIN_DEC_PIC_HEIGHT,
			.num_planes = 1,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_YUV420M,
			.max_width = W6_MAX_DEC_PIC_WIDTH,
			.min_width = W6_MIN_DEC_PIC_WIDTH,
			.max_height = W6_MAX_DEC_PIC_HEIGHT,
			.min_height = W6_MIN_DEC_PIC_HEIGHT,
			.num_planes = 3,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV12M,
			.max_width = W6_MAX_DEC_PIC_WIDTH,
			.min_width = W6_MIN_DEC_PIC_WIDTH,
			.max_height = W6_MAX_DEC_PIC_HEIGHT,
			.min_height = W6_MIN_DEC_PIC_HEIGHT,
			.num_planes = 2,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV21M,
			.max_width = W6_MAX_DEC_PIC_WIDTH,
			.min_width = W6_MIN_DEC_PIC_WIDTH,
			.max_height = W6_MAX_DEC_PIC_HEIGHT,
			.min_height = W6_MIN_DEC_PIC_HEIGHT,
			.num_planes = 2,
		},
	}
};

static int wave6_vpu_dec_seek_header(struct vpu_instance *inst);

static enum wave_std wave6_to_vpu_codstd(unsigned int v4l2_pix_fmt)
{
	switch (v4l2_pix_fmt) {
	case V4L2_PIX_FMT_H264:
		return W_AVC_DEC;
	case V4L2_PIX_FMT_HEVC:
		return W_HEVC_DEC;
	default:
		return STD_UNKNOWN;
	}
}

static const struct vpu_format *wave6_find_vpu_fmt(unsigned int v4l2_pix_fmt,
						   enum vpu_fmt_type type)
{
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(wave6_vpu_dec_fmt_list[type]); index++) {
		if (wave6_vpu_dec_fmt_list[type][index].v4l2_pix_fmt == v4l2_pix_fmt)
			return &wave6_vpu_dec_fmt_list[type][index];
	}

	return NULL;
}

static const struct vpu_format *wave6_find_vpu_fmt_by_idx(unsigned int idx,
							  enum vpu_fmt_type type)
{
	if (idx >= ARRAY_SIZE(wave6_vpu_dec_fmt_list[type]))
		return NULL;

	if (!wave6_vpu_dec_fmt_list[type][idx].v4l2_pix_fmt)
		return NULL;

	return &wave6_vpu_dec_fmt_list[type][idx];
}

static void wave6_vpu_dec_release_fb(struct vpu_instance *inst)
{
	int i;

	for (i = 0; i < WAVE6_MAX_FBS; i++) {
		wave6_free_dma(&inst->frame_vbuf[i]);
		memset(&inst->frame_buf[i], 0, sizeof(struct frame_buffer));
		wave6_free_dma(&inst->aux_vbuf[AUX_BUF_FBC_Y_TBL][i]);
		wave6_free_dma(&inst->aux_vbuf[AUX_BUF_FBC_C_TBL][i]);
		wave6_free_dma(&inst->aux_vbuf[AUX_BUF_MV_COL][i]);
		wave6_free_dma(&inst->aux_vbuf[AUX_BUF_DEF_CDF][i]);
		wave6_free_dma(&inst->aux_vbuf[AUX_BUF_SEG_MAP][i]);
		wave6_free_dma(&inst->aux_vbuf[AUX_BUF_PRE_ENT][i]);
	}
}

static void wave6_vpu_dec_destroy_instance(struct vpu_instance *inst)
{
	u32 fail_res;
	int ret;

	dprintk(inst->dev->dev, "[%d] destroy instance\n", inst->id);
	wave6_vpu_remove_dbgfs_file(inst);

	ret = wave6_vpu_dec_close(inst, &fail_res);
	if (ret) {
		dev_err(inst->dev->dev, "failed destroy instance: %d (%d)\n",
			ret, fail_res);
	}

	wave6_vpu_dec_release_fb(inst);

	wave6_vpu_set_instance_state(inst, VPU_INST_STATE_NONE);

	if (!pm_runtime_suspended(inst->dev->dev))
		pm_runtime_put_sync(inst->dev->dev);
}

static void wave6_handle_bitstream_buffer(struct vpu_instance *inst)
{
	struct vb2_v4l2_buffer *src_buf;
	u32 src_size = 0;
	int ret;

	src_buf = v4l2_m2m_next_src_buf(inst->v4l2_fh.m2m_ctx);
	if (src_buf) {
		struct vpu_buffer *vpu_buf = wave6_to_vpu_buf(src_buf);
		dma_addr_t rd_ptr = wave6_get_dma_addr(src_buf, 0);

		if (vpu_buf->consumed) {
			dev_dbg(inst->dev->dev, "%s: Already consumed buffer\n",
				__func__);
			return;
		}

		vpu_buf->ts_start = ktime_get_raw();
		vpu_buf->consumed = true;
		wave6_vpu_dec_set_rd_ptr(inst, rd_ptr, true);

		src_size = vb2_get_plane_payload(&src_buf->vb2_buf, 0);
		if (swiotlb_find_pool(inst->dev->dev, rd_ptr))
			dma_sync_single_for_device(inst->dev->dev, rd_ptr,
						   src_size, DMA_BIDIRECTIONAL);
	}

	if (!src_size) {
		dma_addr_t rd = 0, wr = 0;

		wave6_vpu_dec_get_bitstream_buffer(inst, &rd, &wr, NULL);
		wave6_vpu_dec_set_rd_ptr(inst, wr, true);
	}

	ret = wave6_vpu_dec_update_bitstream_buffer(inst, src_size);
	if (ret) {
		dev_err(inst->dev->dev, "%s: Update bitstream buffer fail %d\n",
			__func__, ret);
		return;
	}
}

static void wave6_update_pix_fmt_cap(struct v4l2_pix_format_mplane *pix_mp,
				     unsigned int width,
				     unsigned int height,
				     bool new_resolution)
{
	unsigned int aligned_width;

	if (new_resolution)
		pix_mp->plane_fmt[0].bytesperline = 0;

	aligned_width = round_up(width, 32);
	wave6_update_pix_fmt(pix_mp, aligned_width, height);
}

static int wave6_allocate_aux_buffer(struct vpu_instance *inst,
				     enum aux_buffer_type type,
				     int num)
{
	struct aux_buffer buf[WAVE6_MAX_FBS];
	struct aux_buffer_info buf_info;
	struct dec_aux_buffer_size_info size_info;
	unsigned int size;
	int i, ret;

	memset(buf, 0, sizeof(buf));

	size_info.width = inst->src_fmt.width;
	size_info.height = inst->src_fmt.height;
	size_info.type = type;

	ret = wave6_vpu_dec_get_aux_buffer_size(inst, size_info, &size);
	if (ret) {
		dev_err(inst->dev->dev, "%s: Get size fail\n", __func__);
		return ret;
	}

	num = min_t(u32, num, WAVE6_MAX_FBS);
	for (i = 0; i < num; i++) {
		inst->aux_vbuf[type][i].size = size;
		ret = wave6_alloc_dma(inst->dev->dev, &inst->aux_vbuf[type][i]);
		if (ret) {
			dev_err(inst->dev->dev, "%s: Alloc fail\n", __func__);
			return ret;
		}

		buf[i].index = i;
		buf[i].addr = inst->aux_vbuf[type][i].daddr;
		buf[i].size = inst->aux_vbuf[type][i].size;
	}

	buf_info.type = type;
	buf_info.num = num;
	buf_info.buf_array = buf;

	ret = wave6_vpu_dec_register_aux_buffer(inst, buf_info);
	if (ret) {
		dev_err(inst->dev->dev, "%s: Register fail\n", __func__);
		return ret;
	}

	return 0;
}

static void wave6_vpu_dec_handle_dst_buffer(struct vpu_instance *inst)
{
	struct vb2_v4l2_buffer *dst_buf;
	struct v4l2_m2m_buffer *v4l2_m2m_buf;
	struct vpu_buffer *vpu_buf;
	dma_addr_t buf_addr_y = 0, buf_addr_cb = 0, buf_addr_cr = 0;
	u32 buf_size = 0;
	u32 fb_stride = inst->dst_fmt.plane_fmt[0].bytesperline;
	u32 luma_size = fb_stride * inst->dst_fmt.height;
	u32 chroma_size = (fb_stride / 2) * (inst->dst_fmt.height / 2);
	struct frame_buffer disp_buffer = {0};
	struct dec_initial_info initial_info = {0};
	int consumed_num = wave6_vpu_get_consumed_fb_num(inst);
	int ret;

	wave6_vpu_dec_give_command(inst, DEC_GET_SEQ_INFO, &initial_info);

	v4l2_m2m_for_each_dst_buf(inst->v4l2_fh.m2m_ctx, v4l2_m2m_buf) {
		dst_buf = &v4l2_m2m_buf->vb;
		vpu_buf = wave6_to_vpu_buf(dst_buf);

		if (vpu_buf->consumed)
			continue;

		if (consumed_num >= W6_MAX_FB_NUM)
			break;

		if (inst->dst_fmt.num_planes == 1) {
			buf_size = vb2_plane_size(&dst_buf->vb2_buf, 0);
			buf_addr_y = wave6_get_dma_addr(dst_buf, 0);
			buf_addr_cb = buf_addr_y + luma_size;
			buf_addr_cr = buf_addr_cb + chroma_size;
		} else if (inst->dst_fmt.num_planes == 2) {
			buf_size = vb2_plane_size(&dst_buf->vb2_buf, 0) +
				   vb2_plane_size(&dst_buf->vb2_buf, 1);
			buf_addr_y = wave6_get_dma_addr(dst_buf, 0);
			buf_addr_cb = wave6_get_dma_addr(dst_buf, 1);
			buf_addr_cr = buf_addr_cb + chroma_size;
		} else if (inst->dst_fmt.num_planes == 3) {
			buf_size = vb2_plane_size(&dst_buf->vb2_buf, 0) +
				   vb2_plane_size(&dst_buf->vb2_buf, 1) +
				   vb2_plane_size(&dst_buf->vb2_buf, 2);
			buf_addr_y = wave6_get_dma_addr(dst_buf, 0);
			buf_addr_cb = wave6_get_dma_addr(dst_buf, 1);
			buf_addr_cr = wave6_get_dma_addr(dst_buf, 2);
		}
		disp_buffer.buf_y = buf_addr_y;
		disp_buffer.buf_cb = buf_addr_cb;
		disp_buffer.buf_cr = buf_addr_cr;
		disp_buffer.width = inst->src_fmt.width;
		disp_buffer.height = inst->src_fmt.height;
		disp_buffer.stride = fb_stride;
		disp_buffer.map_type = LINEAR_FRAME_MAP;
		disp_buffer.luma_bitdepth = initial_info.luma_bitdepth;
		disp_buffer.chroma_bitdepth = initial_info.chroma_bitdepth;
		disp_buffer.chroma_format_idc = initial_info.chroma_format_idc;

		ret = wave6_vpu_dec_register_display_buffer_ex(inst, disp_buffer);
		if (ret) {
			dev_err(inst->dev->dev, "fail register display buffer %d", ret);
			break;
		}

		vpu_buf->consumed = true;
		consumed_num++;
	}
}

static enum v4l2_quantization to_v4l2_quantization(u32 video_full_range_flag)
{
	switch (video_full_range_flag) {
	case 0:
		return V4L2_QUANTIZATION_LIM_RANGE;
	case 1:
		return V4L2_QUANTIZATION_FULL_RANGE;
	default:
		return V4L2_QUANTIZATION_DEFAULT;
	}
}

static enum v4l2_colorspace to_v4l2_colorspace(u32 colour_primaries)
{
	switch (colour_primaries) {
	case 1:
		return V4L2_COLORSPACE_REC709;
	case 4:
		return V4L2_COLORSPACE_470_SYSTEM_M;
	case 5:
		return V4L2_COLORSPACE_470_SYSTEM_BG;
	case 6:
		return V4L2_COLORSPACE_SMPTE170M;
	case 7:
		return V4L2_COLORSPACE_SMPTE240M;
	case 9:
		return V4L2_COLORSPACE_BT2020;
	case 11:
		return V4L2_COLORSPACE_DCI_P3;
	default:
		return V4L2_COLORSPACE_DEFAULT;
	}
}

static enum v4l2_xfer_func to_v4l2_xfer_func(u32 transfer_characteristics)
{
	switch (transfer_characteristics) {
	case 1:
		return V4L2_XFER_FUNC_709;
	case 6:
		return V4L2_XFER_FUNC_709;
	case 7:
		return V4L2_XFER_FUNC_SMPTE240M;
	case 8:
		return V4L2_XFER_FUNC_NONE;
	case 13:
		return V4L2_XFER_FUNC_SRGB;
	case 14:
		return V4L2_XFER_FUNC_709;
	case 16:
		return V4L2_XFER_FUNC_SMPTE2084;
	default:
		return V4L2_XFER_FUNC_DEFAULT;
	}
}

static enum v4l2_ycbcr_encoding to_v4l2_ycbcr_encoding(u32 matrix_coeffs)
{
	switch (matrix_coeffs) {
	case 1:
		return V4L2_YCBCR_ENC_709;
	case 5:
		return V4L2_YCBCR_ENC_601;
	case 6:
		return V4L2_YCBCR_ENC_601;
	case 7:
		return V4L2_YCBCR_ENC_SMPTE240M;
	case 9:
		return V4L2_YCBCR_ENC_BT2020;
	case 10:
		return V4L2_YCBCR_ENC_BT2020_CONST_LUM;
	default:
		return V4L2_YCBCR_ENC_DEFAULT;
	}
}

static void wave6_update_color_info(struct vpu_instance *inst,
				    struct dec_initial_info *initial_info)
{
	struct wave_color_param *color = &initial_info->color;

	if (!color->video_signal_type_present_flag)
		goto set_default_all;

	inst->quantization = to_v4l2_quantization(color->color_range);

	if (!color->color_description_present_flag)
		goto set_default_color;

	inst->colorspace = to_v4l2_colorspace(color->color_primaries);
	inst->xfer_func = to_v4l2_xfer_func(color->transfer_characteristics);
	inst->ycbcr_enc = to_v4l2_ycbcr_encoding(color->matrix_coefficients);

	return;

set_default_all:
	inst->quantization = V4L2_QUANTIZATION_DEFAULT;
set_default_color:
	inst->colorspace = V4L2_COLORSPACE_DEFAULT;
	inst->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	inst->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static enum v4l2_mpeg_video_hevc_profile to_v4l2_hevc_profile(s32 profile)
{
	switch (profile) {
	case HEVC_PROFILE_MAIN:
		return V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN;
	default:
		return V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN;
	}
}

static enum v4l2_mpeg_video_h264_profile to_v4l2_h264_profile(s32 profile)
{
	switch (profile) {
	case H264_PROFILE_BP:
		return V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
	case H264_PROFILE_MP:
		return V4L2_MPEG_VIDEO_H264_PROFILE_MAIN;
	case H264_PROFILE_EXTENDED:
		return V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED;
	case H264_PROFILE_HP:
		return V4L2_MPEG_VIDEO_H264_PROFILE_HIGH;
	default:
		return V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
	}
}

static void wave6_update_v4l2_ctrls(struct vpu_instance *inst,
				    struct dec_initial_info *info)
{
	struct v4l2_ctrl *ctrl;
	u32 min_disp_cnt;

	min_disp_cnt = info->frame_buf_delay + 1;
	ctrl = v4l2_ctrl_find(&inst->v4l2_ctrl_hdl,
			      V4L2_CID_MIN_BUFFERS_FOR_CAPTURE);
	if (ctrl)
		v4l2_ctrl_s_ctrl(ctrl, min_disp_cnt);

	if (inst->src_fmt.pixelformat == V4L2_PIX_FMT_HEVC) {
		ctrl = v4l2_ctrl_find(&inst->v4l2_ctrl_hdl,
				      V4L2_CID_MPEG_VIDEO_HEVC_PROFILE);
		if (ctrl)
			v4l2_ctrl_s_ctrl(ctrl, to_v4l2_hevc_profile(info->profile));
	} else if (inst->src_fmt.pixelformat == V4L2_PIX_FMT_H264) {
		ctrl = v4l2_ctrl_find(&inst->v4l2_ctrl_hdl,
				      V4L2_CID_MPEG_VIDEO_H264_PROFILE);
		if (ctrl)
			v4l2_ctrl_s_ctrl(ctrl, to_v4l2_h264_profile(info->profile));
	}
}

static int wave6_vpu_dec_start_decode(struct vpu_instance *inst)
{
	struct dec_param pic_param;
	int ret;
	u32 fail_res = 0;

	memset(&pic_param, 0, sizeof(struct dec_param));

	wave6_handle_bitstream_buffer(inst);
	if (inst->state == VPU_INST_STATE_OPEN) {
		ret = wave6_vpu_dec_seek_header(inst);
		if (ret) {
			vb2_queue_error(v4l2_m2m_get_src_vq(inst->v4l2_fh.m2m_ctx));
			vb2_queue_error(v4l2_m2m_get_dst_vq(inst->v4l2_fh.m2m_ctx));
		}
		return -EAGAIN;
	}

	wave6_vpu_dec_handle_dst_buffer(inst);

	ret = wave6_vpu_dec_start_one_frame(inst, &pic_param, &fail_res);
	if (ret) {
		struct vb2_v4l2_buffer *src_buf = NULL;

		dev_err(inst->dev->dev, "[%d] %s: fail %d\n", inst->id, __func__, ret);
		wave6_vpu_set_instance_state(inst, VPU_INST_STATE_STOP);

		src_buf = v4l2_m2m_src_buf_remove(inst->v4l2_fh.m2m_ctx);
		if (src_buf) {
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
			inst->sequence++;
			inst->processed_buf_num++;
			inst->error_buf_num++;
		}
	}

	return ret;
}

static void wave6_handle_decoded_frame(struct vpu_instance *inst,
				       struct dec_output_info *info)
{
	struct vb2_v4l2_buffer *src_buf;
	struct vb2_v4l2_buffer *dst_buf;
	struct vpu_buffer *vpu_buf;
	enum vb2_buffer_state state;

	state = info->decoding_success ? VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR;

	src_buf = v4l2_m2m_next_src_buf(inst->v4l2_fh.m2m_ctx);
	if (!src_buf) {
		dev_err(inst->dev->dev, "[%d] decoder can't find src buffer\n", inst->id);
		return;
	}

	vpu_buf = wave6_to_vpu_buf(src_buf);
	if (!vpu_buf || !vpu_buf->consumed) {
		dev_err(inst->dev->dev, "[%d] src buffer is not consumed\n", inst->id);
		return;
	}

	dst_buf = wave6_get_dst_buf_by_addr(inst, info->frame_decoded_addr);
	if (dst_buf) {
		struct vpu_buffer *dst_vpu_buf = wave6_to_vpu_buf(dst_buf);

		if (dst_vpu_buf->used) {
			dev_warn(inst->dev->dev, "[%d] duplication frame buffer\n", inst->id);
			inst->sequence++;
		}
		v4l2_m2m_buf_copy_metadata(src_buf, dst_buf, true);
		dst_vpu_buf->used = true;
		if (state == VB2_BUF_STATE_ERROR)
			dst_vpu_buf->error = true;
		dst_vpu_buf->ts_input = vpu_buf->ts_input;
		dst_vpu_buf->ts_start = vpu_buf->ts_start;
		dst_vpu_buf->ts_finish = ktime_get_raw();
		dst_vpu_buf->hw_time = wave6_cycle_to_ns(inst->dev, info->cycle.frame_cycle);
	}

	src_buf = v4l2_m2m_src_buf_remove(inst->v4l2_fh.m2m_ctx);
	if (state == VB2_BUF_STATE_ERROR) {
		dprintk(inst->dev->dev, "[%d] error frame %d\n", inst->id, inst->sequence);
		inst->error_buf_num++;
	}
	v4l2_m2m_buf_done(src_buf, state);
	inst->processed_buf_num++;
}

static void wave6_handle_skipped_frame(struct vpu_instance *inst)
{
	struct vb2_v4l2_buffer *src_buf;
	struct vpu_buffer *vpu_buf;

	src_buf = v4l2_m2m_next_src_buf(inst->v4l2_fh.m2m_ctx);
	if (!src_buf)
		return;

	vpu_buf = wave6_to_vpu_buf(src_buf);
	if (!vpu_buf || !vpu_buf->consumed)
		return;

	dprintk(inst->dev->dev, "[%d] skip frame %d\n", inst->id, inst->sequence);

	inst->sequence++;
	inst->processed_buf_num++;
	inst->error_buf_num++;
	src_buf = v4l2_m2m_src_buf_remove(inst->v4l2_fh.m2m_ctx);
	v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
}

static void wave6_handle_display_frame(struct vpu_instance *inst,
				       dma_addr_t addr, enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *dst_buf;
	struct vpu_buffer *vpu_buf;

	dst_buf = wave6_get_dst_buf_by_addr(inst, addr);
	if (!dst_buf)
		return;

	vpu_buf = wave6_to_vpu_buf(dst_buf);
	if (!vpu_buf->used) {
		dprintk(inst->dev->dev, "[%d] recycle display buffer\n", inst->id);
		vpu_buf->consumed = false;
		return;
	}
	for (int i = 0; i < inst->dst_fmt.num_planes; i++) {
		dma_addr_t daddr = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, i);

		if (swiotlb_find_pool(inst->dev->dev, daddr)) {
			dma_sync_single_for_cpu(inst->dev->dev, daddr,
						inst->dst_fmt.plane_fmt[i].sizeimage,
						DMA_BIDIRECTIONAL);
		}
	}

	if (inst->dst_fmt.num_planes == 1) {
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
				      inst->dst_fmt.plane_fmt[0].sizeimage);
	} else if (inst->dst_fmt.num_planes == 2) {
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
				      inst->dst_fmt.plane_fmt[0].sizeimage);
		vb2_set_plane_payload(&dst_buf->vb2_buf, 1,
				      inst->dst_fmt.plane_fmt[1].sizeimage);
	} else if (inst->dst_fmt.num_planes == 3) {
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
				      inst->dst_fmt.plane_fmt[0].sizeimage);
		vb2_set_plane_payload(&dst_buf->vb2_buf, 1,
				      inst->dst_fmt.plane_fmt[1].sizeimage);
		vb2_set_plane_payload(&dst_buf->vb2_buf, 2,
				      inst->dst_fmt.plane_fmt[2].sizeimage);
	}

	vpu_buf->ts_output = ktime_get_raw();
	wave6_vpu_handle_performance(inst, vpu_buf);

	if (vpu_buf->error)
		state = VB2_BUF_STATE_ERROR;
	dst_buf->sequence = inst->sequence++;
	dst_buf->field = V4L2_FIELD_NONE;
	if (state == VB2_BUF_STATE_ERROR)
		dprintk(inst->dev->dev, "[%d] discard frame %d\n", inst->id, dst_buf->sequence);
	v4l2_m2m_dst_buf_remove_by_buf(inst->v4l2_fh.m2m_ctx, dst_buf);
	v4l2_m2m_buf_done(dst_buf, state);
}

static void wave6_handle_display_frames(struct vpu_instance *inst,
					struct dec_output_info *info)
{
	int i;

	for (i = 0; i < info->disp_frame_num; i++)
		wave6_handle_display_frame(inst,
					   info->disp_frame_addr[i],
					   VB2_BUF_STATE_DONE);
}

static void wave6_handle_discard_frames(struct vpu_instance *inst,
					struct dec_output_info *info)
{
	int i;

	for (i = 0; i < info->release_disp_frame_num; i++)
		wave6_handle_display_frame(inst,
					   info->release_disp_frame_addr[i],
					   VB2_BUF_STATE_ERROR);
}

static void wave6_handle_last_frame(struct vpu_instance *inst,
				    struct vb2_v4l2_buffer *dst_buf)
{
	if (!dst_buf) {
		dst_buf = v4l2_m2m_dst_buf_remove(inst->v4l2_fh.m2m_ctx);
		if (!dst_buf) {
			inst->next_buf_last = true;
			return;
		}
	}

	if (inst->dst_fmt.num_planes == 1) {
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0, 0);
	} else if (inst->dst_fmt.num_planes == 2) {
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0, 0);
		vb2_set_plane_payload(&dst_buf->vb2_buf, 1, 0);
	} else if (inst->dst_fmt.num_planes == 3) {
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0, 0);
		vb2_set_plane_payload(&dst_buf->vb2_buf, 1, 0);
		vb2_set_plane_payload(&dst_buf->vb2_buf, 2, 0);
	}

	dst_buf->flags |= V4L2_BUF_FLAG_LAST;
	dst_buf->field = V4L2_FIELD_NONE;
	v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);

	if (inst->state != VPU_INST_STATE_INIT_SEQ) {
		dprintk(inst->dev->dev, "[%d] eos\n", inst->id);
		inst->eos = true;
		v4l2_m2m_set_src_buffered(inst->v4l2_fh.m2m_ctx, false);
	}
}

static void wave6_vpu_dec_retry_one_frame(struct vpu_instance *inst)
{
	struct vb2_v4l2_buffer *src_buf;
	struct vpu_buffer *vpu_buf;

	src_buf = v4l2_m2m_next_src_buf(inst->v4l2_fh.m2m_ctx);
	if (!src_buf)
		return;

	vpu_buf = wave6_to_vpu_buf(src_buf);
	vpu_buf->consumed = false;
}

static void wave6_vpu_dec_handle_source_change(struct vpu_instance *inst,
					       struct dec_initial_info *info)
{
	static const struct v4l2_event vpu_event_src_ch = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};

	dprintk(inst->dev->dev, "pic size %dx%d profile %d, min_fb_cnt : %d | min_disp_cnt : %d\n",
		info->pic_width, info->pic_height,
		info->profile, info->min_frame_buffer_count, info->frame_buf_delay);

	wave6_vpu_dec_retry_one_frame(inst);
	wave6_vpu_dec_give_command(inst, DEC_RESET_FRAMEBUF_INFO, NULL);

	wave6_vpu_set_instance_state(inst, VPU_INST_STATE_INIT_SEQ);

	inst->crop.left = info->pic_crop_rect.left;
	inst->crop.top = info->pic_crop_rect.top;
	inst->crop.width = info->pic_crop_rect.right - inst->crop.left;
	inst->crop.height = info->pic_crop_rect.bottom - inst->crop.top;

	wave6_update_v4l2_ctrls(inst, info);
	wave6_update_color_info(inst, info);
	wave6_update_pix_fmt(&inst->src_fmt, info->pic_width, info->pic_height);
	wave6_update_pix_fmt_cap(&inst->dst_fmt,
				 info->pic_width, info->pic_height,
				 true);
	v4l2_event_queue_fh(&inst->v4l2_fh, &vpu_event_src_ch);
}

static void wave6_vpu_dec_handle_decoding_warn_error(struct vpu_instance *inst,
						     struct dec_output_info *info)
{
	if (info->warn_info)
		dev_dbg(inst->dev->dev, "[%d] decoding %d warning 0x%x\n",
			inst->id, inst->processed_buf_num, info->warn_info);

	if (info->error_reason)
		dev_err(inst->dev->dev, "[%d] decoding %d error 0x%x\n",
			inst->id, inst->processed_buf_num, info->error_reason);
}

static void wave6_vpu_dec_finish_decode(struct vpu_instance *inst, int irq_status)
{
	struct dec_output_info info;
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;
	int ret;

	ret = wave6_vpu_dec_get_output_info(inst, &info);
	if (ret)
		goto finish_decode;

	dev_dbg(inst->dev->dev, "dec %d dis %d noti_flag %d stream_end %d\n",
		info.frame_decoded_flag, info.frame_display_flag,
		info.notification_flag, info.stream_end_flag);

	if (info.notification_flag & DEC_NOTI_FLAG_NO_FB) {
		wave6_vpu_dec_retry_one_frame(inst);
		goto finish_decode;
	}

	if (info.notification_flag & DEC_NOTI_FLAG_SEQ_CHANGE) {
		struct dec_initial_info initial_info = {0};

		v4l2_m2m_mark_stopped(m2m_ctx);

		if (info.frame_display_flag)
			wave6_handle_display_frames(inst, &info);

		if (info.release_disp_frame_num)
			wave6_handle_discard_frames(inst, &info);

		wave6_vpu_dec_give_command(inst, DEC_GET_SEQ_INFO, &initial_info);
		wave6_vpu_dec_handle_source_change(inst, &initial_info);

		wave6_handle_last_frame(inst, NULL);

		goto finish_decode;
	}

	wave6_vpu_dec_handle_decoding_warn_error(inst, &info);

	if (info.frame_decoded_flag)
		wave6_handle_decoded_frame(inst, &info);
	else
		wave6_handle_skipped_frame(inst);

	if (info.frame_display_flag)
		wave6_handle_display_frames(inst, &info);

	if (info.release_disp_frame_num)
		wave6_handle_discard_frames(inst, &info);

	if (info.stream_end_flag && !inst->eos)
		wave6_handle_last_frame(inst, NULL);

finish_decode:
	wave6_vpu_finish_job(inst);
}

static int wave6_vpu_dec_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	strscpy(cap->driver, VPU_DEC_DRV_NAME, sizeof(cap->driver));
	strscpy(cap->card, VPU_DEC_DRV_NAME, sizeof(cap->card));
	strscpy(cap->bus_info, "platform:" VPU_DEC_DRV_NAME, sizeof(cap->bus_info));

	return 0;
}

static int wave6_vpu_dec_enum_framesizes(struct file *f, void *fh, struct v4l2_frmsizeenum *fsize)
{
	const struct vpu_format *vpu_fmt;

	if (fsize->index)
		return -EINVAL;

	vpu_fmt = wave6_find_vpu_fmt(fsize->pixel_format, VPU_FMT_TYPE_CODEC);
	if (!vpu_fmt) {
		vpu_fmt = wave6_find_vpu_fmt(fsize->pixel_format, VPU_FMT_TYPE_RAW);
		if (!vpu_fmt)
			return -EINVAL;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = vpu_fmt->min_width;
	fsize->stepwise.max_width = vpu_fmt->max_width;
	fsize->stepwise.step_width = W6_DEC_PIC_SIZE_STEP;
	fsize->stepwise.min_height = vpu_fmt->min_height;
	fsize->stepwise.max_height = vpu_fmt->max_height;
	fsize->stepwise.step_height = W6_DEC_PIC_SIZE_STEP;

	return 0;
}

static int wave6_vpu_dec_enum_fmt_cap(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	const struct vpu_format *vpu_fmt;

	vpu_fmt = wave6_find_vpu_fmt_by_idx(f->index, VPU_FMT_TYPE_RAW);
	if (!vpu_fmt)
		return -EINVAL;

	f->pixelformat = vpu_fmt->v4l2_pix_fmt;
	f->flags = 0;

	return 0;
}

static int wave6_vpu_dec_try_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave6_to_vpu_inst(fh);
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	const struct vpu_format *vpu_fmt;
	int width, height;

	dev_dbg(inst->dev->dev, "%s: 4cc %d w %d h %d plane %d colorspace %d\n",
		__func__, pix_mp->pixelformat, pix_mp->width, pix_mp->height,
		pix_mp->num_planes, pix_mp->colorspace);

	if (!V4L2_TYPE_IS_CAPTURE(f->type))
		return -EINVAL;

	vpu_fmt = wave6_find_vpu_fmt(pix_mp->pixelformat, VPU_FMT_TYPE_RAW);
	if (!vpu_fmt) {
		width = inst->dst_fmt.width;
		height = inst->dst_fmt.height;
		pix_mp->pixelformat = inst->dst_fmt.pixelformat;
		pix_mp->num_planes = inst->dst_fmt.num_planes;
	} else {
		width = clamp(pix_mp->width,
			      vpu_fmt->min_width, round_up(inst->src_fmt.width, 32));
		height = clamp(pix_mp->height,
			       vpu_fmt->min_height, inst->src_fmt.height);
		pix_mp->pixelformat = vpu_fmt->v4l2_pix_fmt;
		pix_mp->num_planes = vpu_fmt->num_planes;
	}

	if (inst->state >= VPU_INST_STATE_INIT_SEQ) {
		width = inst->dst_fmt.width;
		height = inst->dst_fmt.height;
	}

	wave6_update_pix_fmt_cap(pix_mp, width, height, false);
	pix_mp->colorspace = inst->colorspace;
	pix_mp->ycbcr_enc = inst->ycbcr_enc;
	pix_mp->quantization = inst->quantization;
	pix_mp->xfer_func = inst->xfer_func;

	return 0;
}

static int wave6_vpu_dec_s_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave6_to_vpu_inst(fh);
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	int i, ret;

	dev_dbg(inst->dev->dev, "%s: 4cc %d w %d h %d plane %d colorspace %d\n",
		__func__, pix_mp->pixelformat, pix_mp->width, pix_mp->height,
		pix_mp->num_planes, pix_mp->colorspace);

	ret = wave6_vpu_dec_try_fmt_cap(file, fh, f);
	if (ret)
		return ret;

	inst->dst_fmt.width = pix_mp->width;
	inst->dst_fmt.height = pix_mp->height;
	inst->dst_fmt.pixelformat = pix_mp->pixelformat;
	inst->dst_fmt.field = pix_mp->field;
	inst->dst_fmt.flags = pix_mp->flags;
	inst->dst_fmt.num_planes = pix_mp->num_planes;
	for (i = 0; i < inst->dst_fmt.num_planes; i++) {
		inst->dst_fmt.plane_fmt[i].bytesperline = pix_mp->plane_fmt[i].bytesperline;
		inst->dst_fmt.plane_fmt[i].sizeimage = pix_mp->plane_fmt[i].sizeimage;
	}

	if (inst->dst_fmt.pixelformat == V4L2_PIX_FMT_NV12 ||
	    inst->dst_fmt.pixelformat == V4L2_PIX_FMT_NV12M) {
		inst->cbcr_interleave = true;
		inst->nv21 = false;
	} else if (inst->dst_fmt.pixelformat == V4L2_PIX_FMT_NV21 ||
		   inst->dst_fmt.pixelformat == V4L2_PIX_FMT_NV21M) {
		inst->cbcr_interleave = true;
		inst->nv21 = true;
	} else {
		inst->cbcr_interleave = false;
		inst->nv21 = false;
	}

	return 0;
}

static int wave6_vpu_dec_g_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave6_to_vpu_inst(fh);
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	int i;

	pix_mp->width = inst->dst_fmt.width;
	pix_mp->height = inst->dst_fmt.height;
	pix_mp->pixelformat = inst->dst_fmt.pixelformat;
	pix_mp->field = inst->dst_fmt.field;
	pix_mp->flags = inst->dst_fmt.flags;
	pix_mp->num_planes = inst->dst_fmt.num_planes;
	for (i = 0; i < pix_mp->num_planes; i++) {
		pix_mp->plane_fmt[i].bytesperline = inst->dst_fmt.plane_fmt[i].bytesperline;
		pix_mp->plane_fmt[i].sizeimage = inst->dst_fmt.plane_fmt[i].sizeimage;
	}

	pix_mp->colorspace = inst->colorspace;
	pix_mp->ycbcr_enc = inst->ycbcr_enc;
	pix_mp->quantization = inst->quantization;
	pix_mp->xfer_func = inst->xfer_func;

	return 0;
}

static int wave6_vpu_dec_enum_fmt_out(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct vpu_instance *inst = wave6_to_vpu_inst(fh);
	const struct vpu_format *vpu_fmt;

	dev_dbg(inst->dev->dev, "%s: index %d\n", __func__, f->index);

	vpu_fmt = wave6_find_vpu_fmt_by_idx(f->index, VPU_FMT_TYPE_CODEC);
	if (!vpu_fmt)
		return -EINVAL;

	f->pixelformat = vpu_fmt->v4l2_pix_fmt;
	f->flags = 0;
	f->flags = V4L2_FMT_FLAG_DYN_RESOLUTION | V4L2_FMT_FLAG_COMPRESSED;

	return 0;
}

static int wave6_vpu_dec_try_fmt_out(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave6_to_vpu_inst(fh);
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	const struct vpu_format *vpu_fmt;
	int width, height;

	dev_dbg(inst->dev->dev, "%s: 4cc %d w %d h %d plane %d colorspace %d\n",
		__func__, pix_mp->pixelformat, pix_mp->width, pix_mp->height,
		pix_mp->num_planes, pix_mp->colorspace);

	if (!V4L2_TYPE_IS_OUTPUT(f->type))
		return -EINVAL;

	vpu_fmt = wave6_find_vpu_fmt(pix_mp->pixelformat, VPU_FMT_TYPE_CODEC);
	if (!vpu_fmt) {
		width = inst->src_fmt.width;
		height = inst->src_fmt.height;
		pix_mp->pixelformat = inst->src_fmt.pixelformat;
		pix_mp->num_planes = inst->src_fmt.num_planes;
	} else {
		width = pix_mp->width;
		height = pix_mp->height;
		pix_mp->pixelformat = vpu_fmt->v4l2_pix_fmt;
		pix_mp->num_planes = vpu_fmt->num_planes;
	}

	wave6_update_pix_fmt(pix_mp, width, height);
	pix_mp->colorspace = inst->colorspace;
	pix_mp->ycbcr_enc = inst->ycbcr_enc;
	pix_mp->quantization = inst->quantization;
	pix_mp->xfer_func = inst->xfer_func;

	return 0;
}

static int wave6_vpu_dec_s_fmt_out(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave6_to_vpu_inst(fh);
	struct v4l2_pix_format_mplane in_pix_mp = f->fmt.pix_mp;
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	int i, ret;

	dev_dbg(inst->dev->dev, "%s: 4cc %d w %d h %d plane %d colorspace %d\n",
		__func__, pix_mp->pixelformat, pix_mp->width, pix_mp->height,
		pix_mp->num_planes, pix_mp->colorspace);

	ret = wave6_vpu_dec_try_fmt_out(file, fh, f);
	if (ret)
		return ret;

	pix_mp->colorspace = in_pix_mp.colorspace;
	pix_mp->ycbcr_enc = in_pix_mp.ycbcr_enc;
	pix_mp->quantization = in_pix_mp.quantization;
	pix_mp->xfer_func = in_pix_mp.xfer_func;

	inst->src_fmt.width = pix_mp->width;
	inst->src_fmt.height = pix_mp->height;
	inst->src_fmt.pixelformat = pix_mp->pixelformat;
	inst->src_fmt.field = pix_mp->field;
	inst->src_fmt.flags = pix_mp->flags;
	inst->src_fmt.num_planes = pix_mp->num_planes;
	for (i = 0; i < inst->src_fmt.num_planes; i++) {
		inst->src_fmt.plane_fmt[i].bytesperline = pix_mp->plane_fmt[i].bytesperline;
		inst->src_fmt.plane_fmt[i].sizeimage = pix_mp->plane_fmt[i].sizeimage;
	}

	inst->colorspace = pix_mp->colorspace;
	inst->ycbcr_enc = pix_mp->ycbcr_enc;
	inst->quantization = pix_mp->quantization;
	inst->xfer_func = pix_mp->xfer_func;

	wave6_update_pix_fmt_cap(&inst->dst_fmt,
				 pix_mp->width, pix_mp->height,
				 true);

	return 0;
}

static int wave6_vpu_dec_g_fmt_out(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave6_to_vpu_inst(fh);
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	int i;

	pix_mp->width = inst->src_fmt.width;
	pix_mp->height = inst->src_fmt.height;
	pix_mp->pixelformat = inst->src_fmt.pixelformat;
	pix_mp->field = inst->src_fmt.field;
	pix_mp->flags = inst->src_fmt.flags;
	pix_mp->num_planes = inst->src_fmt.num_planes;
	for (i = 0; i < pix_mp->num_planes; i++) {
		pix_mp->plane_fmt[i].bytesperline = inst->src_fmt.plane_fmt[i].bytesperline;
		pix_mp->plane_fmt[i].sizeimage = inst->src_fmt.plane_fmt[i].sizeimage;
	}

	pix_mp->colorspace = inst->colorspace;
	pix_mp->ycbcr_enc = inst->ycbcr_enc;
	pix_mp->quantization = inst->quantization;
	pix_mp->xfer_func = inst->xfer_func;

	return 0;
}

static int wave6_vpu_dec_g_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct vpu_instance *inst = wave6_to_vpu_inst(fh);

	dev_dbg(inst->dev->dev, "%s: type %d target %d\n",
		__func__, s->type, s->target);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = inst->dst_fmt.width;
		s->r.height = inst->dst_fmt.height;
		break;
	case V4L2_SEL_TGT_COMPOSE_PADDED:
	case V4L2_SEL_TGT_COMPOSE:
		s->r.left = 0;
		s->r.top = 0;
		if (inst->scaler_info.enable) {
			s->r.width = inst->scaler_info.width;
			s->r.height = inst->scaler_info.height;
		} else if (inst->crop.width && inst->crop.height) {
			s->r = inst->crop;
		} else {
			s->r.width = inst->src_fmt.width;
			s->r.height = inst->src_fmt.height;
		}
		break;
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = inst->src_fmt.width;
		s->r.height = inst->src_fmt.height;
		if (inst->crop.width && inst->crop.height)
			s->r = inst->crop;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int wave6_vpu_dec_s_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct vpu_instance *inst = wave6_to_vpu_inst(fh);
	int step = 4;
	int scale_width = 0, scale_height = 0;
	int min_scale_width = 0, min_scale_height = 0;

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	if (s->target != V4L2_SEL_TGT_COMPOSE)
		return -EINVAL;

	if (!(s->flags & (V4L2_SEL_FLAG_GE | V4L2_SEL_FLAG_LE)))
		s->flags |= V4L2_SEL_FLAG_LE;

	scale_width = clamp(s->r.width, W6_MIN_DEC_PIC_WIDTH,
			    round_up(inst->src_fmt.width, 32));
	scale_height = clamp(s->r.height, W6_MIN_DEC_PIC_HEIGHT,
			     inst->src_fmt.height);
	if (s->flags & V4L2_SEL_FLAG_GE) {
		scale_width = round_up(scale_width, step);
		scale_height = round_up(scale_height, step);
	}
	if (s->flags & V4L2_SEL_FLAG_LE) {
		scale_width = round_down(scale_width, step);
		scale_height = round_down(scale_height, step);
	}

	if (scale_width < inst->src_fmt.width ||
	    scale_height < inst->src_fmt.height)
		inst->scaler_info.enable = true;

	if (inst->scaler_info.enable) {
		min_scale_width = ALIGN((inst->src_fmt.width / 8), step);
		min_scale_height = ALIGN((inst->src_fmt.height / 8), step);

		if (scale_width < W6_MIN_DEC_PIC_WIDTH)
			scale_width = W6_MIN_DEC_PIC_WIDTH;
		if (scale_width < min_scale_width)
			scale_width = min_scale_width;
		if (scale_height < W6_MIN_DEC_PIC_HEIGHT)
			scale_height = W6_MIN_DEC_PIC_HEIGHT;
		if (scale_height < min_scale_height)
			scale_height = min_scale_height;

		inst->scaler_info.width = scale_width;
		inst->scaler_info.height = scale_height;
	}

	s->r.left = 0;
	s->r.top = 0;
	s->r.width = scale_width;
	s->r.height = scale_height;

	return 0;
}

static int wave6_vpu_dec_decoder_cmd(struct file *file, void *fh, struct v4l2_decoder_cmd *dc)
{
	struct vpu_instance *inst = wave6_to_vpu_inst(fh);
	int ret;

	dev_dbg(inst->dev->dev, "%s: cmd %d\n", __func__, dc->cmd);

	ret = v4l2_m2m_ioctl_try_decoder_cmd(file, fh, dc);
	if (ret)
		return ret;

	switch (dc->cmd) {
	case V4L2_DEC_CMD_STOP:
		dprintk(inst->dev->dev, "[%d] drain\n", inst->id);
		v4l2_m2m_set_src_buffered(inst->v4l2_fh.m2m_ctx, true);
		v4l2_m2m_try_schedule(inst->v4l2_fh.m2m_ctx);
		break;
	case V4L2_DEC_CMD_START:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ioctl_ops wave6_vpu_dec_ioctl_ops = {
	.vidioc_querycap = wave6_vpu_dec_querycap,
	.vidioc_enum_framesizes = wave6_vpu_dec_enum_framesizes,

	.vidioc_enum_fmt_vid_cap = wave6_vpu_dec_enum_fmt_cap,
	.vidioc_s_fmt_vid_cap_mplane = wave6_vpu_dec_s_fmt_cap,
	.vidioc_g_fmt_vid_cap_mplane = wave6_vpu_dec_g_fmt_cap,
	.vidioc_try_fmt_vid_cap_mplane = wave6_vpu_dec_try_fmt_cap,

	.vidioc_enum_fmt_vid_out = wave6_vpu_dec_enum_fmt_out,
	.vidioc_s_fmt_vid_out_mplane = wave6_vpu_dec_s_fmt_out,
	.vidioc_g_fmt_vid_out_mplane = wave6_vpu_dec_g_fmt_out,
	.vidioc_try_fmt_vid_out_mplane = wave6_vpu_dec_try_fmt_out,

	.vidioc_g_selection = wave6_vpu_dec_g_selection,
	.vidioc_s_selection = wave6_vpu_dec_s_selection,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_try_decoder_cmd,
	.vidioc_decoder_cmd = wave6_vpu_dec_decoder_cmd,

	.vidioc_subscribe_event = wave6_vpu_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int wave6_vpu_dec_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vpu_instance *inst = wave6_ctrl_to_vpu_inst(ctrl);

	dev_dbg(inst->dev->dev, "%s: name %s value %d\n",
		__func__, ctrl->name, ctrl->val);

	switch (ctrl->id) {
	case V4L2_CID_VPU_THUMBNAIL_MODE:
		inst->thumbnail_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY_ENABLE:
		inst->disp_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY:
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
	case V4L2_CID_MPEG_VIDEO_HEVC_PROFILE:
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops wave6_vpu_dec_ctrl_ops = {
	.s_ctrl = wave6_vpu_dec_s_ctrl,
};

static const struct v4l2_ctrl_config wave6_vpu_thumbnail_mode = {
	.ops = &wave6_vpu_dec_ctrl_ops,
	.id = V4L2_CID_VPU_THUMBNAIL_MODE,
	.name = "thumbnail mode",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.def = 0,
	.min = 0,
	.max = 1,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_WRITE_ONLY,
};

static void wave6_set_dec_openparam(struct dec_open_param *open_param,
				    struct vpu_instance *inst)
{
	open_param->inst_buffer.temp_base = inst->dev->temp_vbuf.daddr;
	open_param->inst_buffer.temp_size = inst->dev->temp_vbuf.size;
	open_param->bitstream_mode = BS_MODE_PIC_END;
	open_param->stream_endian = VPU_STREAM_ENDIAN;
	open_param->frame_endian = VPU_FRAME_ENDIAN;
	open_param->disp_mode = inst->disp_mode;
}

static int wave6_vpu_dec_create_instance(struct vpu_instance *inst)
{
	int ret;
	struct dec_open_param open_param;

	memset(&open_param, 0, sizeof(struct dec_open_param));

	wave6_vpu_activate(inst->dev);
	ret = pm_runtime_resume_and_get(inst->dev->dev);
	if (ret) {
		dev_err(inst->dev->dev, "runtime_resume failed %d\n", ret);
		return ret;
	}

	wave6_vpu_wait_activated(inst->dev);

	inst->std = wave6_to_vpu_codstd(inst->src_fmt.pixelformat);
	if (inst->std == STD_UNKNOWN) {
		dev_err(inst->dev->dev, "unsupported pixelformat: %.4s\n",
			(char *)&inst->src_fmt.pixelformat);
		ret = -EINVAL;
		goto error_pm;
	}

	wave6_set_dec_openparam(&open_param, inst);

	ret = wave6_vpu_dec_open(inst, &open_param);
	if (ret) {
		dev_err(inst->dev->dev, "failed create instance : %d\n", ret);
		goto error_pm;
	}

	dprintk(inst->dev->dev, "[%d] decoder\n", inst->id);

	if (inst->thumbnail_mode)
		wave6_vpu_dec_give_command(inst, ENABLE_DEC_THUMBNAIL_MODE, NULL);

	wave6_vpu_create_dbgfs_file(inst);
	wave6_vpu_set_instance_state(inst, VPU_INST_STATE_OPEN);
	inst->v4l2_fh.m2m_ctx->ignore_cap_streaming = true;
	v4l2_m2m_set_dst_buffered(inst->v4l2_fh.m2m_ctx, true);

	return 0;

error_pm:
	pm_runtime_put_sync(inst->dev->dev);

	return ret;
}

static int wave6_vpu_dec_prepare_fb(struct vpu_instance *inst)
{
	int ret;
	unsigned int i;
	unsigned int fb_num = 0;
	unsigned int mv_num = 0;
	unsigned int fb_stride = 0;
	unsigned int fb_height = 0;
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	struct vpu_attr *attr = &inst->dev->attr;

	fb_num = p_dec_info->initial_info.min_frame_buffer_count;
	mv_num = p_dec_info->initial_info.req_mv_buffer_count;

	fb_stride = ALIGN(inst->src_fmt.width, 32);
	fb_height = ALIGN(inst->src_fmt.height, 32);

	for (i = 0; i < fb_num; i++) {
		struct frame_buffer *frame = &inst->frame_buf[i];
		struct vpu_buf *vframe = &inst->frame_vbuf[i];
		unsigned int l_size = fb_stride * fb_height;
		unsigned int ch_size = ALIGN(fb_stride / 2, 32) * fb_height;

		vframe->size = l_size + ch_size;
		ret = wave6_alloc_dma(inst->dev->dev, vframe);
		if (ret) {
			dev_err(inst->dev->dev, "alloc FBC buffer fail : %zu\n",
				vframe->size);
			goto error;
		}

		frame->buf_y = vframe->daddr;
		frame->buf_cb = vframe->daddr + l_size;
		frame->buf_cr = (dma_addr_t)-1;
		frame->width = inst->src_fmt.width;
		frame->stride = fb_stride;
		frame->height = fb_height;
		frame->map_type = COMPRESSED_FRAME_MAP;
	}

	ret = wave6_allocate_aux_buffer(inst, AUX_BUF_FBC_Y_TBL, fb_num);
	if (ret) {
		dev_err(inst->dev->dev, "alloc FBC_Y_TBL buffer fail\n");
		goto error;
	}
	ret = wave6_allocate_aux_buffer(inst, AUX_BUF_FBC_C_TBL, fb_num);
	if (ret) {
		dev_err(inst->dev->dev, "alloc FBC_C_TBL buffer fail\n");
		goto error;
	}
	ret = wave6_allocate_aux_buffer(inst, AUX_BUF_MV_COL, mv_num);
	if (ret) {
		dev_err(inst->dev->dev, "alloc MV_COL buffer fail\n");
		goto error;
	}
	if (inst->std == W_AV1_DEC) {
		ret = wave6_allocate_aux_buffer(inst, AUX_BUF_DEF_CDF, 1);
		if (ret) {
			dev_err(inst->dev->dev, "alloc DEF_CDF buffer fail\n");
			goto error;
		}
	}
	if (inst->std == W_VP9_DEC) {
		ret = wave6_allocate_aux_buffer(inst, AUX_BUF_SEG_MAP, 1);
		if (ret) {
			dev_err(inst->dev->dev, "alloc SEG_MAP buffer fail\n");
			goto error;
		}
	}
	if ((inst->std == W_AV1_DEC || inst->std == W_VP9_DEC) &&
	    attr->support_command_queue) {
		ret = wave6_allocate_aux_buffer(inst, AUX_BUF_PRE_ENT, 1);
		if (ret) {
			dev_err(inst->dev->dev, "alloc PRE_ENT buffer fail\n");
			goto error;
		}
	}

	ret = wave6_vpu_dec_register_frame_buffer_ex(inst, fb_num, fb_stride,
						     fb_height,
						     COMPRESSED_FRAME_MAP);
	if (ret) {
		dev_err(inst->dev->dev, "register frame buffer fail %d\n", ret);
		goto error;
	}

	wave6_vpu_set_instance_state(inst, VPU_INST_STATE_PIC_RUN);

	return 0;

error:
	wave6_vpu_dec_release_fb(inst);
	return ret;
}

static int wave6_vpu_dec_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
				     unsigned int *num_planes, unsigned int sizes[],
				     struct device *alloc_devs[])
{
	struct vpu_instance *inst = vb2_get_drv_priv(q);
	struct v4l2_pix_format_mplane inst_format =
		(V4L2_TYPE_IS_OUTPUT(q->type)) ? inst->src_fmt : inst->dst_fmt;
	unsigned int i;

	dev_dbg(inst->dev->dev, "%s: num_buffers %d num_planes %d type %d\n",
		__func__, *num_buffers, *num_planes, q->type);

	if (*num_planes) {
		if (inst_format.num_planes != *num_planes)
			return -EINVAL;

		for (i = 0; i < *num_planes; i++) {
			if (sizes[i] < inst_format.plane_fmt[i].sizeimage)
				return -EINVAL;
		}
	} else {
		*num_planes = inst_format.num_planes;
		for (i = 0; i < *num_planes; i++) {
			sizes[i] = inst_format.plane_fmt[i].sizeimage;
			dev_dbg(inst->dev->dev, "size[%d] : %d\n", i, sizes[i]);
		}

		if (V4L2_TYPE_IS_CAPTURE(q->type)) {
			struct v4l2_ctrl *ctrl;
			unsigned int min_disp_cnt = 0;

			ctrl = v4l2_ctrl_find(&inst->v4l2_ctrl_hdl,
					      V4L2_CID_MIN_BUFFERS_FOR_CAPTURE);
			if (ctrl)
				min_disp_cnt = v4l2_ctrl_g_ctrl(ctrl);

			*num_buffers = max(*num_buffers, min_disp_cnt);

			if (*num_buffers > WAVE6_MAX_FBS)
				*num_buffers = min_disp_cnt;
		}
	}

	if (V4L2_TYPE_IS_OUTPUT(q->type) &&
	    inst->state == VPU_INST_STATE_SEEK) {
		wave6_vpu_pause(inst->dev->dev, 0);
		wave6_vpu_dec_destroy_instance(inst);
		wave6_vpu_pause(inst->dev->dev, 1);
	}

	return 0;
}

static int wave6_vpu_dec_seek_header(struct vpu_instance *inst)
{
	struct dec_initial_info initial_info;
	int ret;

	memset(&initial_info, 0, sizeof(struct dec_initial_info));

	ret = wave6_vpu_dec_issue_seq_init(inst);
	if (ret) {
		dev_err(inst->dev->dev, "failed wave6_vpu_dec_issue_seq_init %d\n", ret);
		return ret;
	}

	if (wave6_vpu_wait_interrupt(inst, VPU_DEC_TIMEOUT) < 0)
		dev_err(inst->dev->dev, "failed to call vpu_wait_interrupt()\n");

	ret = wave6_vpu_dec_complete_seq_init(inst, &initial_info);
	if (ret) {
		dev_err(inst->dev->dev, "vpu_dec_complete_seq_init: %d, reason : 0x%x\n",
			ret, initial_info.err_reason);
		if (initial_info.err_reason & WAVE6_SYSERR_NOT_SUPPORT) {
			ret = -EINVAL;
		} else if ((initial_info.err_reason & HEVC_ETCERR_INIT_SEQ_SPS_NOT_FOUND) ||
			   (initial_info.err_reason & AVC_ETCERR_INIT_SEQ_SPS_NOT_FOUND)) {
			wave6_handle_skipped_frame(inst);
			ret = 0;
		}
	} else {
		wave6_vpu_dec_handle_source_change(inst, &initial_info);
		inst->v4l2_fh.m2m_ctx->ignore_cap_streaming = false;
		v4l2_m2m_set_dst_buffered(inst->v4l2_fh.m2m_ctx, false);
		if (vb2_is_streaming(v4l2_m2m_get_dst_vq(inst->v4l2_fh.m2m_ctx)))
			wave6_handle_last_frame(inst, NULL);
	}

	return ret;
}

static void wave6_vpu_dec_buf_queue_src(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vpu_buffer *vpu_buf = wave6_to_vpu_buf(vbuf);
	struct vpu_instance *inst = vb2_get_drv_priv(vb->vb2_queue);

	dev_dbg(inst->dev->dev, "type %4d index %4d size[0] %4ld size[1] : %4ld | size[2] : %4ld\n",
		vb->type, vb->index, vb2_plane_size(&vbuf->vb2_buf, 0),
		vb2_plane_size(&vbuf->vb2_buf, 1), vb2_plane_size(&vbuf->vb2_buf, 2));

	vbuf->sequence = inst->queued_src_buf_num++;
	vpu_buf->ts_input = ktime_get_raw();

	v4l2_m2m_buf_queue(inst->v4l2_fh.m2m_ctx, vbuf);
}

static void wave6_vpu_dec_buf_queue_dst(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vpu_instance *inst = vb2_get_drv_priv(vb->vb2_queue);

	dev_dbg(inst->dev->dev, "type %4d index %4d size[0] %4ld size[1] : %4ld | size[2] : %4ld\n",
		vb->type, vb->index, vb2_plane_size(&vbuf->vb2_buf, 0),
		vb2_plane_size(&vbuf->vb2_buf, 1), vb2_plane_size(&vbuf->vb2_buf, 2));

	inst->queued_dst_buf_num++;
	if (inst->next_buf_last) {
		wave6_handle_last_frame(inst, vbuf);
		inst->next_buf_last = false;
	} else {
		v4l2_m2m_buf_queue(inst->v4l2_fh.m2m_ctx, vbuf);
	}
}

static void wave6_vpu_dec_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vpu_buffer *vpu_buf = wave6_to_vpu_buf(vbuf);

	vpu_buf->consumed = false;
	vpu_buf->used = false;
	vpu_buf->error = false;
	if (V4L2_TYPE_IS_OUTPUT(vb->type))
		wave6_vpu_dec_buf_queue_src(vb);
	else
		wave6_vpu_dec_buf_queue_dst(vb);
}

static int wave6_vpu_dec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct vpu_instance *inst = vb2_get_drv_priv(q);
	struct v4l2_pix_format_mplane *fmt;
	int ret = 0;

	wave6_vpu_pause(inst->dev->dev, 0);

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		fmt = &inst->src_fmt;
		if (inst->state == VPU_INST_STATE_NONE) {
			ret = wave6_vpu_dec_create_instance(inst);
			if (ret)
				goto exit;
		}

		if (inst->state == VPU_INST_STATE_SEEK)
			wave6_vpu_set_instance_state(inst, inst->state_in_seek);
	} else {
		fmt = &inst->dst_fmt;
		if (inst->state == VPU_INST_STATE_INIT_SEQ) {
			ret = wave6_vpu_dec_prepare_fb(inst);
			if (ret)
				goto exit;
		}
	}

exit:
	wave6_vpu_pause(inst->dev->dev, 1);
	if (ret)
		wave6_vpu_return_buffers(inst, q->type, VB2_BUF_STATE_QUEUED);

	dprintk(inst->dev->dev, "[%d] %s %c%c%c%c %dx%d, %d buffers, ret = %d\n",
		inst->id, V4L2_TYPE_IS_OUTPUT(q->type) ? "output" : "capture",
		fmt->pixelformat,
		fmt->pixelformat >> 8,
		fmt->pixelformat >> 16,
		fmt->pixelformat >> 24,
		fmt->width, fmt->height, vb2_get_num_buffers(q), ret);

	return ret;
}

static void wave6_vpu_dec_stop_streaming(struct vb2_queue *q)
{
	struct vpu_instance *inst = vb2_get_drv_priv(q);
	struct v4l2_m2m_ctx *m2m_ctx = inst->v4l2_fh.m2m_ctx;

	dprintk(inst->dev->dev, "[%d] %s, input %d, decode %d, error %d\n",
		inst->id, V4L2_TYPE_IS_OUTPUT(q->type) ? "output" : "capture",
		inst->queued_src_buf_num, inst->sequence, inst->error_buf_num);

	if (inst->state == VPU_INST_STATE_NONE)
		goto exit;

	wave6_vpu_pause(inst->dev->dev, 0);

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		wave6_vpu_reset_performance(inst);
		inst->queued_src_buf_num = 0;
		inst->processed_buf_num = 0;
		inst->error_buf_num = 0;
		inst->state_in_seek = inst->state;
		v4l2_m2m_set_src_buffered(inst->v4l2_fh.m2m_ctx, false);
		wave6_vpu_set_instance_state(inst, VPU_INST_STATE_SEEK);
		inst->sequence = 0;
	} else {
		if (v4l2_m2m_has_stopped(m2m_ctx))
			v4l2_m2m_clear_state(m2m_ctx);

		inst->eos = false;
		inst->queued_dst_buf_num = 0;
		inst->sequence = 0;
		wave6_vpu_dec_flush_instance(inst);
	}

	wave6_vpu_pause(inst->dev->dev, 1);

exit:
	wave6_vpu_return_buffers(inst, q->type, VB2_BUF_STATE_ERROR);
}

static int wave6_vpu_dec_buf_init(struct vb2_buffer *vb)
{
	struct vpu_instance *inst = vb2_get_drv_priv(vb->vb2_queue);
	struct dec_initial_info initial_info = {0};
	int i;

	if (V4L2_TYPE_IS_OUTPUT(vb->type))
		return 0;

	wave6_vpu_dec_give_command(inst, DEC_GET_SEQ_INFO, &initial_info);
	if (initial_info.chroma_format_idc != YUV400)
		return 0;

	for (i = 0; i < inst->dst_fmt.num_planes; i++)
		memset(vb2_plane_vaddr(vb, i), 0x80, vb2_plane_size(vb, i));

	return 0;
}

static const struct vb2_ops wave6_vpu_dec_vb2_ops = {
	.queue_setup = wave6_vpu_dec_queue_setup,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.buf_queue = wave6_vpu_dec_buf_queue,
	.start_streaming = wave6_vpu_dec_start_streaming,
	.stop_streaming = wave6_vpu_dec_stop_streaming,
	.buf_init = wave6_vpu_dec_buf_init,
};

static void wave6_set_default_format(struct v4l2_pix_format_mplane *src_fmt,
				     struct v4l2_pix_format_mplane *dst_fmt)
{
	const struct vpu_format *vpu_fmt;

	vpu_fmt = wave6_find_vpu_fmt_by_idx(0, VPU_FMT_TYPE_CODEC);
	if (vpu_fmt) {
		src_fmt->pixelformat = vpu_fmt->v4l2_pix_fmt;
		src_fmt->num_planes = vpu_fmt->num_planes;
		wave6_update_pix_fmt(src_fmt,
				     W6_DEF_DEC_PIC_WIDTH, W6_DEF_DEC_PIC_HEIGHT);
	}

	vpu_fmt = wave6_find_vpu_fmt_by_idx(0, VPU_FMT_TYPE_RAW);
	if (vpu_fmt) {
		dst_fmt->pixelformat = vpu_fmt->v4l2_pix_fmt;
		dst_fmt->num_planes = vpu_fmt->num_planes;
		wave6_update_pix_fmt_cap(dst_fmt,
					 W6_DEF_DEC_PIC_WIDTH, W6_DEF_DEC_PIC_HEIGHT,
					 true);
	}
}

static int wave6_vpu_dec_queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
	struct vpu_instance *inst = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->ops = &wave6_vpu_dec_vb2_ops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->buf_struct_size = sizeof(struct vpu_buffer);
	src_vq->min_queued_buffers = 1;
	src_vq->drv_priv = inst;
	src_vq->lock = &inst->dev->dev_lock;
	src_vq->dev = inst->dev->v4l2_dev.dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->ops = &wave6_vpu_dec_vb2_ops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->buf_struct_size = sizeof(struct vpu_buffer);
	dst_vq->min_queued_buffers = 1;
	dst_vq->drv_priv = inst;
	dst_vq->lock = &inst->dev->dev_lock;
	dst_vq->dev = inst->dev->v4l2_dev.dev;
	ret = vb2_queue_init(dst_vq);
	if (ret)
		return ret;

	return 0;
}

static const struct vpu_instance_ops wave6_vpu_dec_inst_ops = {
	.start_process = wave6_vpu_dec_start_decode,
	.finish_process = wave6_vpu_dec_finish_decode,
};

static int wave6_vpu_open_dec(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct vpu_device *dev = video_drvdata(filp);
	struct vpu_instance *inst = NULL;
	int ret;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	inst->dev = dev;
	inst->type = VPU_INST_TYPE_DEC;
	inst->ops = &wave6_vpu_dec_inst_ops;

	v4l2_fh_init(&inst->v4l2_fh, vdev);
	filp->private_data = &inst->v4l2_fh;
	v4l2_fh_add(&inst->v4l2_fh);

	inst->v4l2_fh.m2m_ctx =
		v4l2_m2m_ctx_init(dev->m2m_dev, inst, wave6_vpu_dec_queue_init);
	if (IS_ERR(inst->v4l2_fh.m2m_ctx)) {
		ret = PTR_ERR(inst->v4l2_fh.m2m_ctx);
		goto free_inst;
	}

	v4l2_ctrl_handler_init(&inst->v4l2_ctrl_hdl, 10);
	v4l2_ctrl_new_custom(&inst->v4l2_ctrl_hdl, &wave6_vpu_thumbnail_mode, NULL);
	v4l2_ctrl_new_std(&inst->v4l2_ctrl_hdl, &wave6_vpu_dec_ctrl_ops,
			  V4L2_CID_MIN_BUFFERS_FOR_CAPTURE, 1, 32, 1, 1);
	v4l2_ctrl_new_std(&inst->v4l2_ctrl_hdl, &wave6_vpu_dec_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY,
			  0, 0, 1, 0);
	v4l2_ctrl_new_std(&inst->v4l2_ctrl_hdl, &wave6_vpu_dec_ctrl_ops,
			  V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY_ENABLE,
			  0, 1, 1, 0);
	v4l2_ctrl_new_std_menu(&inst->v4l2_ctrl_hdl, &wave6_vpu_dec_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
			       V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN, 0,
			       V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN);
	v4l2_ctrl_new_std_menu(&inst->v4l2_ctrl_hdl, &wave6_vpu_dec_ctrl_ops,
			       V4L2_CID_MPEG_VIDEO_H264_PROFILE,
			       V4L2_MPEG_VIDEO_H264_PROFILE_HIGH, 0,
			       V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE);

	if (inst->v4l2_ctrl_hdl.error) {
		ret = -ENODEV;
		goto err_m2m_release;
	}

	inst->v4l2_fh.ctrl_handler = &inst->v4l2_ctrl_hdl;
	v4l2_ctrl_handler_setup(&inst->v4l2_ctrl_hdl);

	wave6_set_default_format(&inst->src_fmt, &inst->dst_fmt);
	inst->colorspace = V4L2_COLORSPACE_DEFAULT;
	inst->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	inst->quantization = V4L2_QUANTIZATION_DEFAULT;
	inst->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	return 0;

err_m2m_release:
	v4l2_m2m_ctx_release(inst->v4l2_fh.m2m_ctx);
free_inst:
	kfree(inst);
	return ret;
}

static int wave6_vpu_dec_release(struct file *filp)
{
	struct vpu_instance *inst = wave6_to_vpu_inst(filp->private_data);

	dprintk(inst->dev->dev, "[%d] release\n", inst->id);
	v4l2_m2m_ctx_release(inst->v4l2_fh.m2m_ctx);

	mutex_lock(&inst->dev->dev_lock);
	if (inst->state != VPU_INST_STATE_NONE) {
		wave6_vpu_pause(inst->dev->dev, 0);
		wave6_vpu_dec_destroy_instance(inst);
		wave6_vpu_pause(inst->dev->dev, 1);
	}
	mutex_unlock(&inst->dev->dev_lock);

	v4l2_ctrl_handler_free(&inst->v4l2_ctrl_hdl);
	v4l2_fh_del(&inst->v4l2_fh);
	v4l2_fh_exit(&inst->v4l2_fh);
	kfree(inst);

	return 0;
}

static const struct v4l2_file_operations wave6_vpu_dec_fops = {
	.owner = THIS_MODULE,
	.open = wave6_vpu_open_dec,
	.release = wave6_vpu_dec_release,
	.unlocked_ioctl = video_ioctl2,
	.poll = v4l2_m2m_fop_poll,
	.mmap = v4l2_m2m_fop_mmap,
};

int wave6_vpu_dec_register_device(struct vpu_device *dev)
{
	struct video_device *vdev_dec;
	int ret;

	vdev_dec = devm_kzalloc(dev->v4l2_dev.dev, sizeof(*vdev_dec), GFP_KERNEL);
	if (!vdev_dec)
		return -ENOMEM;

	dev->video_dev_dec = vdev_dec;

	strscpy(vdev_dec->name, VPU_DEC_DEV_NAME, sizeof(vdev_dec->name));
	vdev_dec->fops = &wave6_vpu_dec_fops;
	vdev_dec->ioctl_ops = &wave6_vpu_dec_ioctl_ops;
	vdev_dec->release = video_device_release_empty;
	vdev_dec->v4l2_dev = &dev->v4l2_dev;
	vdev_dec->vfl_dir = VFL_DIR_M2M;
	vdev_dec->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	vdev_dec->lock = &dev->dev_lock;
	video_set_drvdata(vdev_dec, dev);

	ret = video_register_device(vdev_dec, VFL_TYPE_VIDEO, -1);
	if (ret)
		return ret;


	return 0;
}

void wave6_vpu_dec_unregister_device(struct vpu_device *dev)
{
	video_unregister_device(dev->video_dev_dec);
}
