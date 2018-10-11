/*
 * Copyright(c) 2018 NXP. All rights reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * @file vpu_encoder_ctrl.c
 *
 * Author Ming Qian<ming.qian@nxp.com>
 */

#include <media/v4l2-ctrls.h>

#include "vpu_encoder_b0.h"
#include "vpu_encoder_ctrl.h"

// H264 level is maped like level 5.1 to uLevel 51, except level 1b to uLevel 14
const u_int32 h264_level[] = {
	[V4L2_MPEG_VIDEO_H264_LEVEL_1_0] = 10,
	[V4L2_MPEG_VIDEO_H264_LEVEL_1B]  = 14,
	[V4L2_MPEG_VIDEO_H264_LEVEL_1_1] = 11,
	[V4L2_MPEG_VIDEO_H264_LEVEL_1_2] = 12,
	[V4L2_MPEG_VIDEO_H264_LEVEL_1_3] = 13,
	[V4L2_MPEG_VIDEO_H264_LEVEL_2_0] = 20,
	[V4L2_MPEG_VIDEO_H264_LEVEL_2_1] = 21,
	[V4L2_MPEG_VIDEO_H264_LEVEL_2_2] = 22,
	[V4L2_MPEG_VIDEO_H264_LEVEL_3_0] = 30,
	[V4L2_MPEG_VIDEO_H264_LEVEL_3_1] = 31,
	[V4L2_MPEG_VIDEO_H264_LEVEL_3_2] = 32,
	[V4L2_MPEG_VIDEO_H264_LEVEL_4_0] = 40,
	[V4L2_MPEG_VIDEO_H264_LEVEL_4_1] = 41,
	[V4L2_MPEG_VIDEO_H264_LEVEL_4_2] = 42,
	[V4L2_MPEG_VIDEO_H264_LEVEL_5_0] = 50,
	[V4L2_MPEG_VIDEO_H264_LEVEL_5_1] = 51
};

static int set_h264_profile(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	pMEDIAIP_ENC_PARAM  param = ctx->enc_param;

	mutex_lock(&ctx->instance_mutex);
	switch (ctrl->val) {
	case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE:
		param->eProfile = MEDIAIP_ENC_PROF_H264_BP;
		break;
	case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN:
		param->eProfile = MEDIAIP_ENC_PROF_H264_MP;
		break;
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH:
		param->eProfile = MEDIAIP_ENC_PROF_H264_HP;
		break;
	default:
		vpu_dbg(LVL_ERR, "not support H264 profile %d, set to main\n",
				ctrl->val);
		param->eProfile = MEDIAIP_ENC_PROF_H264_MP;
		break;
	}
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_h264_level(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	pMEDIAIP_ENC_PARAM  param = ctx->enc_param;

	mutex_lock(&ctx->instance_mutex);
	param->uLevel = h264_level[ctrl->val];
	mutex_unlock(&ctx->instance_mutex);

	vpu_dbg(LVL_DEBUG, "set h264 level to %d\n", ctrl->val);

	return 0;
}

static int set_bitrate_mode(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	pMEDIAIP_ENC_PARAM  param = ctx->enc_param;

	mutex_lock(&ctx->instance_mutex);
	switch (ctrl->val) {
	case V4L2_MPEG_VIDEO_BITRATE_MODE_VBR:
		param->eBitRateMode =
				MEDIAIP_ENC_BITRATECONTROLMODE_CONSTANT_QP;
		break;
	case V4L2_MPEG_VIDEO_BITRATE_MODE_CBR:
		param->eBitRateMode = MEDIAIP_ENC_BITRATECONTROLMODE_CBR;
		break;
	default:
		vpu_dbg(LVL_ERR, "not support bitrate mode %d, set to cbr\n",
				ctrl->val);
		param->eBitRateMode = MEDIAIP_ENC_BITRATECONTROLMODE_CBR;
		break;
	}
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_bitrate(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	pMEDIAIP_ENC_PARAM  param = ctx->enc_param;

	mutex_lock(&ctx->instance_mutex);
	param->uTargetBitrate = ctrl->val;
	if (param->uMaxBitRate < param->uTargetBitrate)
		param->uMaxBitRate = param->uTargetBitrate;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_bitrate_peak(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	pMEDIAIP_ENC_PARAM  param = ctx->enc_param;

	mutex_lock(&ctx->instance_mutex);
	param->uMaxBitRate = ctrl->val;
	if (param->uTargetBitrate > param->uMaxBitRate)
		param->uTargetBitrate = param->uMaxBitRate;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_gop_size(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	pMEDIAIP_ENC_PARAM  param = ctx->enc_param;

	mutex_lock(&ctx->instance_mutex);
	param->uGopBLength = ctrl->val;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_i_period(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	pMEDIAIP_ENC_PARAM  param = ctx->enc_param;

	mutex_lock(&ctx->instance_mutex);
	param->uIFrameInterval = ctrl->val;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_qp(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	pMEDIAIP_ENC_PARAM  param = ctx->enc_param;

	mutex_lock(&ctx->instance_mutex);
	param->uInitSliceQP = ctrl->val;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int get_min_buffers_for_output(struct v4l2_ctrl *ctrl)
{
	vpu_dbg(LVL_DEBUG, "get min buffers for output\n");

	ctrl->val = MIN_BUFFER_COUNT;

	return 0;
}

static int set_display_re_ordering(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);
	pMEDIAIP_ENC_PARAM  param = ctx->enc_param;

	mutex_lock(&ctx->instance_mutex);
	if (ctrl->val)
		param->uLowLatencyMode = 1;
	else
		param->uLowLatencyMode = 0;
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int set_force_key_frame(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);

	set_bit(VPU_ENC_STATUS_KEY_FRAME, &ctx->status);

	return 0;
}

static int add_ctrl_h264_profile(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_h264_profile_ops = {
		.s_ctrl = set_h264_profile,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std_menu(&ctx->ctrl_handler,
				      &ctrl_h264_profile_ops,
				      V4L2_CID_MPEG_VIDEO_H264_PROFILE,
				      V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
				      0xa,
				      V4L2_MPEG_VIDEO_H264_PROFILE_MAIN);
	if (!ctrl) {
		vpu_dbg(LVL_ERR, "add ctrl h264 profile fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_h264_level(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_h264_level_ops = {
		.s_ctrl = set_h264_level,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std_menu(&ctx->ctrl_handler,
				      &ctrl_h264_level_ops,
				      V4L2_CID_MPEG_VIDEO_H264_LEVEL,
				      V4L2_MPEG_VIDEO_H264_LEVEL_5_1,
				      0x0,
				      V4L2_MPEG_VIDEO_H264_LEVEL_4_0);
	if (!ctrl) {
		vpu_dbg(LVL_ERR, "add ctrl h264 level fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_bitrate_mode(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_bitrate_mode_ops = {
		.s_ctrl = set_bitrate_mode,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std_menu(&ctx->ctrl_handler,
				      &ctrl_bitrate_mode_ops,
				      V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
				      V4L2_MPEG_VIDEO_BITRATE_MODE_CBR,
				      0x0,
				      V4L2_MPEG_VIDEO_BITRATE_MODE_VBR);
	if (!ctrl) {
		vpu_dbg(LVL_ERR, "add ctrl bitrate mode fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_bitrate(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_bitrate_ops = {
		.s_ctrl = set_bitrate,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				 &ctrl_bitrate_ops,
				 V4L2_CID_MPEG_VIDEO_BITRATE,
				 BITRATE_LOW_THRESHOLD,
				 BITRATE_HIGH_THRESHOLD,
				 1,
				 BITRATE_DEFAULT_TARGET);
	if (!ctrl) {
		vpu_dbg(LVL_ERR, "add ctrl bitrate fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_bitrate_peak(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_bitrate_ops = {
		.s_ctrl = set_bitrate_peak,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				 &ctrl_bitrate_ops,
				 V4L2_CID_MPEG_VIDEO_BITRATE_PEAK,
				 BITRATE_LOW_THRESHOLD,
				 BITRATE_HIGH_THRESHOLD,
				 1,
				 BITRATE_DEFAULT_PEAK);
	if (!ctrl) {
		vpu_dbg(LVL_ERR, "add ctrl bitrate peak fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_gop_size(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_gop_ops = {
		.s_ctrl = set_gop_size,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				 &ctrl_gop_ops,
				 V4L2_CID_MPEG_VIDEO_GOP_SIZE,
				 GOP_L_THRESHOLD,
				 GOP_H_THRESHOLD,
				 1,
				 GOP_DEFAULT);
	if (!ctrl) {
		vpu_dbg(LVL_ERR, "add ctrl gop size fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_i_period(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_i_period_ops = {
		.s_ctrl = set_i_period,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
			&ctrl_i_period_ops,
			V4L2_CID_MPEG_VIDEO_H264_I_PERIOD,
			GOP_L_THRESHOLD,
			GOP_H_THRESHOLD,
			1,
			GOP_DEFAULT);

	if (!ctrl) {
		vpu_dbg(LVL_ERR, "add ctrl i period fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_i_frame_qp(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_iframe_qp_ops = {
		.s_ctrl = set_qp,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				 &ctrl_iframe_qp_ops,
				 V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,
				 QP_MIN,
				 QP_MAX,
				 1,
				 QP_DEFAULT);
	if (!ctrl) {
		vpu_dbg(LVL_ERR, "add ctrl h264 I frame qp fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_p_frame_qp(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_pframe_qp_ops = {
		.s_ctrl = set_qp,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				 &ctrl_pframe_qp_ops,
				 V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP,
				 QP_MIN,
				 QP_MAX,
				 1,
				 QP_DEFAULT);
	if (!ctrl) {
		vpu_dbg(LVL_ERR, "add ctrl h264 P frame qp fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_b_frame_qp(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_bframe_qp_ops = {
		.s_ctrl = set_qp,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				 &ctrl_bframe_qp_ops,
				 V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP,
				 QP_MIN,
				 QP_MAX,
				 1,
				 QP_DEFAULT);
	if (!ctrl) {
		vpu_dbg(LVL_ERR, "add ctrl h264 B frame qp fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_min_buffers_for_output(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops ctrl_min_buffers_ops = {
		.g_volatile_ctrl = get_min_buffers_for_output,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
				 &ctrl_min_buffers_ops,
				 V4L2_CID_MIN_BUFFERS_FOR_OUTPUT,
				 1,
				 32,
				 1,
				 MIN_BUFFER_COUNT);
	if (!ctrl) {
		vpu_dbg(LVL_ERR, "add ctrl min buffers for output fail\n");
		return -EINVAL;
	}

	ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	return 0;
}

static int add_ctrl_display_re_ordering(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops re_ordering_ops = {
		.s_ctrl = set_display_re_ordering,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
			&re_ordering_ops,
			V4L2_CID_MPEG_VIDEO_H264_ASO,
			0, 1, 1, 0);
	if (!ctrl) {
		vpu_dbg(LVL_ERR, "add ctrl display re ordering fail\n");
		return -EINVAL;
	}

	return 0;
}

static int add_ctrl_force_key_frame(struct vpu_ctx *ctx)
{
	static const struct v4l2_ctrl_ops force_key_frame_ops = {
		.s_ctrl = set_force_key_frame,
	};
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_new_std(&ctx->ctrl_handler,
			&force_key_frame_ops,
			V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME,
			0, 0, 0, 0);
	if (!ctrl) {
		vpu_dbg(LVL_ERR, "add ctrl force key frame fail\n");
		return -EINVAL;
	}

	return 0;
}

static int vpu_enc_register_ctrls(struct vpu_ctx *ctx)
{
	add_ctrl_h264_profile(ctx);
	add_ctrl_h264_level(ctx);
	add_ctrl_bitrate_mode(ctx);
	add_ctrl_bitrate(ctx);
	add_ctrl_bitrate_peak(ctx);
	add_ctrl_gop_size(ctx);
	add_ctrl_i_period(ctx);
	add_ctrl_i_frame_qp(ctx);
	add_ctrl_p_frame_qp(ctx);
	add_ctrl_b_frame_qp(ctx);
	add_ctrl_min_buffers_for_output(ctx);
	add_ctrl_display_re_ordering(ctx);
	add_ctrl_force_key_frame(ctx);

	return 0;
}

int vpu_enc_setup_ctrls(struct vpu_ctx *ctx)
{
	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 11);
	vpu_enc_register_ctrls(ctx);
	if (ctx->ctrl_handler.error) {
		vpu_dbg(LVL_ERR, "control initialization error (%d)\n",
			ctx->ctrl_handler.error);
		return -EINVAL;
	}
	ctx->ctrl_inited = true;
	return v4l2_ctrl_handler_setup(&ctx->ctrl_handler);
}

int vpu_enc_free_ctrls(struct vpu_ctx *ctx)
{
	if (ctx->ctrl_inited) {
		v4l2_ctrl_handler_free(&ctx->ctrl_handler);
		ctx->ctrl_inited = false;
	}

	return 0;
}
