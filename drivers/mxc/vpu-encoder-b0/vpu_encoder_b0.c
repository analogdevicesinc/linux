/*
 * Copyright 2018 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file vpu_encoder_b0.c
 *
 * copyright here may be changed later
 *
 *
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/videodev2.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/file.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/platform_data/dma-imx.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/pm_runtime.h>
#include <linux/mx8_mu.h>
#include <linux/uaccess.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>

#include "vpu_encoder_b0.h"
#include "vpu_encoder_ctrl.h"
#include "vpu_encoder_config.h"

struct vpu_frame_info {
	struct list_head list;
	MEDIAIP_ENC_PIC_INFO info;
	u32 wptr;
	u32 rptr;
	u32 start;
	u32 end;
	bool eos;
};

unsigned int vpu_dbg_level_encoder = LVL_WARN;

static char *mu_cmp[] = {
	"fsl,imx8-mu1-vpu-m0",
	"fsl,imx8-mu2-vpu-m0"
};

#define ITEM_NAME(name)		\
				[name] = #name

static char *cmd2str[] = {
	ITEM_NAME(GTB_ENC_CMD_NOOP),
	ITEM_NAME(GTB_ENC_CMD_STREAM_START),
	ITEM_NAME(GTB_ENC_CMD_FRAME_ENCODE),
	ITEM_NAME(GTB_ENC_CMD_FRAME_SKIP),
	ITEM_NAME(GTB_ENC_CMD_STREAM_STOP),
	ITEM_NAME(GTB_ENC_CMD_PARAMETER_UPD),
	ITEM_NAME(GTB_ENC_CMD_TERMINATE),
	ITEM_NAME(GTB_ENC_CMD_SNAPSHOT),
	ITEM_NAME(GTB_ENC_CMD_ROLL_SNAPSHOT),
	ITEM_NAME(GTB_ENC_CMD_LOCK_SCHEDULER),
	ITEM_NAME(GTB_ENC_CMD_UNLOCK_SCHEDULER),
	ITEM_NAME(GTB_ENC_CMD_CONFIGURE_CODEC),
	ITEM_NAME(GTB_ENC_CMD_DEAD_MARK),
	ITEM_NAME(GTB_ENC_CMD_FIRM_RESET),
	ITEM_NAME(GTB_ENC_CMD_RESERVED)
};

static char *event2str[] = {
	ITEM_NAME(VID_API_EVENT_UNDEFINED),
	ITEM_NAME(VID_API_ENC_EVENT_RESET_DONE),
	ITEM_NAME(VID_API_ENC_EVENT_START_DONE),
	ITEM_NAME(VID_API_ENC_EVENT_STOP_DONE),
	ITEM_NAME(VID_API_ENC_EVENT_TERMINATE_DONE),
	ITEM_NAME(VID_API_ENC_EVENT_FRAME_INPUT_DONE),
	ITEM_NAME(VID_API_ENC_EVENT_FRAME_DONE),
	ITEM_NAME(VID_API_ENC_EVENT_FRAME_RELEASE),
	ITEM_NAME(VID_API_ENC_EVENT_PARA_UPD_DONE),
	ITEM_NAME(VID_API_ENC_EVENT_MEM_REQUEST),
	ITEM_NAME(VID_API_ENC_EVENT_RESERVED)
};

static char *get_event_str(u32 event)
{
	if (event >= VID_API_ENC_EVENT_RESERVED)
		return "UNKNOWN EVENT";
	return event2str[event];
}

static char *get_cmd_str(u32 cmdid)
{
	if (cmdid >= GTB_ENC_CMD_RESERVED)
		return "UNKNOWN CMD";
	return cmd2str[cmdid];
}

static void vpu_log_event(u_int32 uEvent, u_int32 ctxid)
{
	if (uEvent >= VID_API_ENC_EVENT_RESERVED)
		vpu_dbg(LVL_ERR, "reveive event: 0x%X, ctx id:%d\n",
				uEvent, ctxid);
	else
		vpu_dbg(LVL_DEBUG, "recevie event: %s, ctx id:%d\n",
				event2str[uEvent], ctxid);
}

static void vpu_log_cmd(u_int32 cmdid, u_int32 ctxid)
{
	if (cmdid >= GTB_ENC_CMD_RESERVED)
		vpu_dbg(LVL_ERR, "send cmd: 0x%X, ctx id:%d\n",
				cmdid, ctxid);
	else
		vpu_dbg(LVL_DEBUG, "send cmd: %s ctx id:%d\n",
				cmd2str[cmdid], ctxid);
}

static void count_event(struct vpu_ctx *ctx, u32 event)
{
	struct vpu_attr *attr;

	WARN_ON(!ctx);

	attr = get_vpu_ctx_attr(ctx);
	if (!attr)
		return;

	if (event < VID_API_ENC_EVENT_RESERVED)
		attr->statistic.event[event]++;
	else
		attr->statistic.event[VID_API_ENC_EVENT_RESERVED]++;

	attr->statistic.current_event = event;
	getrawmonotonic(&attr->statistic.ts_event);
}

static void count_cmd(struct vpu_attr *attr, u32 cmdid)
{
	WARN_ON(!attr);

	if (cmdid < GTB_ENC_CMD_RESERVED)
		attr->statistic.cmd[cmdid]++;
	else
		attr->statistic.cmd[GTB_ENC_CMD_RESERVED]++;
	attr->statistic.current_cmd = cmdid;
	getrawmonotonic(&attr->statistic.ts_cmd);
}

static void count_yuv_input(struct vpu_ctx *ctx)
{
	struct vpu_attr *attr = NULL;

	WARN_ON(!ctx);

	attr = get_vpu_ctx_attr(ctx);
	if (!attr)
		return;

	attr->statistic.yuv_count++;
}

static void count_h264_output(struct vpu_ctx *ctx)
{
	struct vpu_attr *attr = NULL;

	WARN_ON(!ctx);

	attr = get_vpu_ctx_attr(ctx);
	if (!attr)
		return;

	attr->statistic.h264_count++;
}

static void count_encoded_frame(struct vpu_ctx *ctx)
{
	struct vpu_attr *attr = NULL;

	WARN_ON(!ctx);

	attr = get_vpu_ctx_attr(ctx);
	if (!attr)
		return;

	attr->statistic.encoded_count++;
}

static void write_enc_reg(struct vpu_dev *dev, u32 val, off_t reg)
{
	writel(val, dev->regs_enc + reg);
}

/*
 * v4l2 ioctl() operation
 *
 */
static struct vpu_v4l2_fmt  formats_compressed_enc[] = {
	{
		.name       = "H264 Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_H264,
		.num_planes = 1,
		.venc_std   = VPU_VIDEO_AVC,
		.is_yuv     = 0,
	},
};

static struct vpu_v4l2_fmt  formats_yuv_enc[] = {
	{
		.name       = "4:2:0 2 Planes Y/CbCr",
		.fourcc     = V4L2_PIX_FMT_NV12,
		.num_planes	= 2,
		.venc_std   = VPU_PF_YUV420_SEMIPLANAR,
		.is_yuv     = 1,
	},
};

static struct v4l2_fract vpu_fps_list[] = {
	{1, 30},
	{1, 29},
	{1, 28},
	{1, 27},
	{1, 26},
	{1, 25},
	{1, 24},
	{1, 23},
	{1, 22},
	{1, 21},
	{1, 20},
	{1, 19},
	{1, 18},
	{1, 17},
	{1, 16},
	{1, 15},
	{1, 14},
	{1, 13},
	{1, 12},
	{1, 11},
	{1, 10},
	{1, 9},
	{1, 8},
	{1, 7},
	{1, 6},
	{1, 5},
	{1, 4},
	{1, 3},
	{1, 2},
	{1, 1},
};

static void vpu_ctx_send_cmd(struct vpu_ctx *ctx, uint32_t cmdid,
				uint32_t cmdnum, uint32_t *local_cmddata);

static void MU_sendMesgToFW(void __iomem *base, MSG_Type type, uint32_t value)
{
	MU_SendMessage(base, 1, value);
	MU_SendMessage(base, 0, type);
}

#define GET_CTX_RPC(ctx, func)	\
		func(&ctx->core_dev->shared_mem, ctx->str_index)

pMEDIAIP_ENC_YUV_BUFFER_DESC get_rpc_yuv_buffer_desc(struct vpu_ctx *ctx)
{
	return GET_CTX_RPC(ctx, rpc_get_yuv_buffer_desc);
}

pBUFFER_DESCRIPTOR_TYPE get_rpc_stream_buffer_desc(struct vpu_ctx *ctx)
{
	return GET_CTX_RPC(ctx, rpc_get_stream_buffer_desc);
}

pMEDIAIP_ENC_EXPERT_MODE_PARAM get_rpc_expert_mode_param(struct vpu_ctx *ctx)
{
	return GET_CTX_RPC(ctx, rpc_get_expert_mode_param);
}

pMEDIAIP_ENC_PARAM get_rpc_enc_param(struct vpu_ctx *ctx)
{
	return GET_CTX_RPC(ctx, rpc_get_enc_param);
}

pMEDIAIP_ENC_MEM_POOL get_rpc_mem_pool(struct vpu_ctx *ctx)
{
	return GET_CTX_RPC(ctx, rpc_get_mem_pool);
}

pENC_ENCODING_STATUS get_rpc_encoding_status(struct vpu_ctx *ctx)
{
	if (!ctx || !ctx->core_dev)
		return NULL;
	return GET_CTX_RPC(ctx, rpc_get_encoding_status);
}

pENC_DSA_STATUS_t get_rpc_dsa_status(struct vpu_ctx *ctx)
{
	if (!ctx || !ctx->core_dev)
		return NULL;
	return GET_CTX_RPC(ctx, rpc_get_dsa_status);
}

static int v4l2_ioctl_querycap(struct file *file,
		void *fh,
		struct v4l2_capability *cap
		)
{
	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);
	strncpy(cap->driver, "vpu encoder", sizeof(cap->driver) - 1);
	strlcpy(cap->card, "vpu encoder", sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:", sizeof(cap->bus_info));
	cap->version = KERNEL_VERSION(0, 0, 1);
	cap->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE |
				V4L2_CAP_STREAMING |
				V4L2_CAP_VIDEO_CAPTURE_MPLANE |
				V4L2_CAP_VIDEO_OUTPUT_MPLANE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int v4l2_ioctl_enum_fmt_vid_cap_mplane(struct file *file,
		void *fh,
		struct v4l2_fmtdesc *f
		)
{
	struct vpu_v4l2_fmt *fmt;

	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);
	if (f->index >= ARRAY_SIZE(formats_compressed_enc))
		return -EINVAL;

	fmt = &formats_compressed_enc[f->index];
	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	f->flags |= V4L2_FMT_FLAG_COMPRESSED;
	return 0;
}
static int v4l2_ioctl_enum_fmt_vid_out_mplane(struct file *file,
		void *fh,
		struct v4l2_fmtdesc *f
		)
{
	struct vpu_v4l2_fmt *fmt;

	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	if (f->index >= ARRAY_SIZE(formats_yuv_enc))
		return -EINVAL;

	fmt = &formats_yuv_enc[f->index];
	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	return 0;
}

static int v4l2_ioctl_enum_framesizes(struct file *file, void *fh,
					struct v4l2_frmsizeenum *fsize)
{
	if (!fsize)
		return -EINVAL;

	if (fsize->index)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.max_width = VPU_ENC_WIDTH_MAX;
	fsize->stepwise.max_height = VPU_ENC_HEIGHT_MAX;
	fsize->stepwise.min_width = VPU_ENC_WIDTH_MIN;
	fsize->stepwise.min_height = VPU_ENC_HEIGHT_MIN;
	fsize->stepwise.step_width = VPU_ENC_WIDTH_STEP;
	fsize->stepwise.step_height = VPU_ENC_HEIGHT_STEP;

	return 0;
}

static int v4l2_ioctl_enum_frameintervals(struct file *file, void *fh,
						struct v4l2_frmivalenum *fival)
{
	if (!fival)
		return -EINVAL;

	if (fival->index >= ARRAY_SIZE(vpu_fps_list))
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = vpu_fps_list[fival->index].numerator;
	fival->discrete.denominator = vpu_fps_list[fival->index].denominator;

	return 0;
}

static int v4l2_ioctl_g_fmt(struct file *file,
		void *fh,
		struct v4l2_format *f
		)
{
	struct vpu_ctx *ctx =           v4l2_fh_to_ctx(fh);
	struct v4l2_pix_format_mplane   *pix_mp = &f->fmt.pix_mp;
	unsigned int i;

	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pix_mp->pixelformat = V4L2_PIX_FMT_NV12;
		pix_mp->width = ctx->q_data[V4L2_SRC].width;
		pix_mp->height = ctx->q_data[V4L2_SRC].height;
		pix_mp->field = V4L2_FIELD_ANY;
		pix_mp->num_planes = 2;
		pix_mp->colorspace = V4L2_COLORSPACE_REC709;

		for (i = 0; i < pix_mp->num_planes; i++)
			pix_mp->plane_fmt[i].sizeimage = ctx->q_data[V4L2_SRC].sizeimage[i];
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pix_mp->width = ctx->q_data[V4L2_DST].width;
		pix_mp->height = ctx->q_data[V4L2_DST].height;
		pix_mp->field = V4L2_FIELD_ANY;
		pix_mp->plane_fmt[0].bytesperline = ctx->q_data[V4L2_DST].width;
		pix_mp->plane_fmt[0].sizeimage = ctx->q_data[V4L2_DST].sizeimage[0];
		pix_mp->pixelformat = V4L2_PIX_FMT_H264;
		pix_mp->num_planes = 1;
	} else
		return -EINVAL;
	return 0;
}

static void get_param_from_v4l2(pMEDIAIP_ENC_PARAM pEncParam,
		struct v4l2_pix_format_mplane *pix_mp,
		struct vpu_ctx *ctx
		)
{
	//get the param and update gpParameters
	pEncParam->uSrcStride           = pix_mp->width;
	pEncParam->uSrcWidth            = pix_mp->width;
	pEncParam->uSrcHeight           = pix_mp->height;
	pEncParam->uSrcOffset_x         = 0;
	pEncParam->uSrcOffset_y         = 0;
	pEncParam->uSrcCropWidth        = pix_mp->width;
	pEncParam->uSrcCropHeight       = pix_mp->height;
	pEncParam->uOutWidth            = pix_mp->width;
	pEncParam->uOutHeight           = pix_mp->height;
}

static u32 cpu_phy_to_mu(struct core_device *dev, u32 addr)
{
	return addr - dev->m0_p_fw_space_phy;
}

static int initialize_enc_param(struct vpu_ctx *ctx)
{
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);
	pMEDIAIP_ENC_PARAM param = &attr->param;

	mutex_lock(&ctx->instance_mutex);

	param->eCodecMode = MEDIAIP_ENC_FMT_H264;
	param->tEncMemDesc.uMemPhysAddr = ctx->encoder_mem.phy_addr;
	param->tEncMemDesc.uMemVirtAddr = ctx->encoder_mem.phy_addr;
	param->tEncMemDesc.uMemSize     = ctx->encoder_mem.size;
	param->uSrcStride = VPU_ENC_WIDTH_DEFAULT;
	param->uSrcWidth = VPU_ENC_WIDTH_DEFAULT;
	param->uSrcHeight = VPU_ENC_HEIGHT_DEFAULT;
	param->uSrcOffset_x = 0;
	param->uSrcOffset_y = 0;
	param->uSrcCropWidth = VPU_ENC_WIDTH_DEFAULT;
	param->uSrcCropHeight = VPU_ENC_HEIGHT_DEFAULT;
	param->uOutWidth = VPU_ENC_WIDTH_DEFAULT;
	param->uOutHeight = VPU_ENC_HEIGHT_DEFAULT;
	param->uFrameRate = VPU_ENC_FRAMERATE_DEFAULT;
	param->uMinBitRate = BITRATE_LOW_THRESHOLD;

	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static int check_size(u32 width, u32 height)
{
	if (width < VPU_ENC_WIDTH_MIN ||
			width > VPU_ENC_WIDTH_MAX ||
			((width - VPU_ENC_WIDTH_MIN) % VPU_ENC_WIDTH_STEP) ||
			height < VPU_ENC_HEIGHT_MIN ||
			height > VPU_ENC_HEIGHT_MAX ||
			((height - VPU_ENC_HEIGHT_MIN) % VPU_ENC_HEIGHT_STEP)) {
		vpu_dbg(LVL_ERR, "Unsupported frame size : %dx%d\n",
				width, height);
		return -EINVAL;
	}

	return 0;
}

static int check_v4l2_fmt(struct v4l2_format *f)
{
	int ret = -EINVAL;

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		ret = check_size(f->fmt.pix.width, f->fmt.pix.height);
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		ret = check_size(f->fmt.pix_mp.width, f->fmt.pix_mp.height);
		break;
	default:
		break;
	}

	return ret;
}

static struct vpu_v4l2_fmt *find_fmt_by_fourcc(struct vpu_v4l2_fmt *fmts,
						unsigned int size,
						u32 fourcc)
{
	unsigned int i;

	if (!fmts || !size)
		return NULL;

	for (i = 0; i < size; i++) {
		if (fmts[i].fourcc == fourcc)
			return &fmts[i];
	}

	return NULL;
}

static char *cvrt_fourcc_to_str(u32 pixelformat)
{
	static char str[5];

	str[0] = pixelformat & 0xff;
	str[1] = (pixelformat >> 8) & 0xff;
	str[2] = (pixelformat >> 16) & 0xff;
	str[3] = (pixelformat >> 24) & 0xff;
	str[4] = '\0';

	return str;
}

static int set_yuv_queue_fmt(struct queue_data *q_data, struct v4l2_format *f)
{
	struct vpu_v4l2_fmt *fmt = NULL;
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	int i;

	if (!q_data || !f)
		return -EINVAL;

	fmt = find_fmt_by_fourcc(q_data->supported_fmts, q_data->fmt_count,
				pix_mp->pixelformat);
	if (!fmt) {
		vpu_dbg(LVL_ERR, "unsupport yuv fmt : %s\n",
				cvrt_fourcc_to_str(pix_mp->pixelformat));
		return -EINVAL;
	}

	q_data->fourcc = pix_mp->pixelformat;
	q_data->width = pix_mp->width;
	q_data->height = pix_mp->height;
	q_data->rect.left = 0;
	q_data->rect.top = 0;
	q_data->rect.width = pix_mp->width;
	q_data->rect.height = pix_mp->height;
	q_data->sizeimage[0] = pix_mp->width * pix_mp->height;
	q_data->sizeimage[1] = pix_mp->width * pix_mp->height / 2;
	pix_mp->num_planes = fmt->num_planes;
	for (i = 0; i < pix_mp->num_planes; i++)
		pix_mp->plane_fmt[i].sizeimage = q_data->sizeimage[i];

	q_data->current_fmt = fmt;

	return 0;
}

static u32 get_enc_minimum_sizeimage(u32 width, u32 height)
{
	const u32 THRESHOLD = 256 * 1024;
	u32 sizeimage;

	sizeimage = width * height / 2;
	if (sizeimage < THRESHOLD)
		sizeimage = THRESHOLD;

	return sizeimage;
}

static int set_enc_queue_fmt(struct queue_data *q_data, struct v4l2_format *f)
{
	struct vpu_v4l2_fmt *fmt = NULL;
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	u32 sizeimage;

	if (!q_data || !f)
		return -EINVAL;

	fmt = find_fmt_by_fourcc(q_data->supported_fmts, q_data->fmt_count,
				pix_mp->pixelformat);
	if (!fmt) {
		vpu_dbg(LVL_ERR, "unsupport encode fmt : %s\n",
				cvrt_fourcc_to_str(pix_mp->pixelformat));
		return -EINVAL;
	}

	q_data->fourcc = pix_mp->pixelformat;
	q_data->width = pix_mp->width;
	q_data->height = pix_mp->height;
	sizeimage = get_enc_minimum_sizeimage(pix_mp->width, pix_mp->height);
	q_data->sizeimage[0] = max(sizeimage, pix_mp->plane_fmt[0].sizeimage);
	pix_mp->plane_fmt[0].sizeimage = q_data->sizeimage[0];

	q_data->current_fmt = fmt;

	return 0;
}

static int v4l2_ioctl_s_fmt(struct file *file,
		void *fh,
		struct v4l2_format *f
		)
{
	struct vpu_ctx                  *ctx = v4l2_fh_to_ctx(fh);
	int                             ret = 0;
	struct v4l2_pix_format_mplane   *pix_mp = &f->fmt.pix_mp;
	struct queue_data               *q_data;
	pMEDIAIP_ENC_PARAM  pEncParam;
	struct vpu_attr *attr;

	attr = get_vpu_ctx_attr(ctx);
	pEncParam = &attr->param;
	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	ret = check_v4l2_fmt(f);
	if (ret)
		return ret;

	mutex_lock(&ctx->instance_mutex);
	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		q_data = &ctx->q_data[V4L2_SRC];
		get_param_from_v4l2(pEncParam, pix_mp, ctx);
		ret = set_yuv_queue_fmt(q_data, f);
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		q_data = &ctx->q_data[V4L2_DST];
		ret = set_enc_queue_fmt(q_data, f);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&ctx->instance_mutex);

	return ret;
}

static int v4l2_ioctl_g_parm(struct file *file, void *fh,
				struct v4l2_streamparm *parm)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct vpu_attr *attr = NULL;
	pMEDIAIP_ENC_PARAM param = NULL;

	if (!parm || !ctx)
		return -EINVAL;

	attr = get_vpu_ctx_attr(ctx);
	param = &attr->param;

	parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	parm->parm.capture.capturemode = V4L2_CAP_TIMEPERFRAME;
	parm->parm.capture.timeperframe.numerator = 1;
	parm->parm.capture.timeperframe.denominator = param->uFrameRate;
	parm->parm.capture.readbuffers = 0;

	return 0;
}

static int find_proper_framerate(struct v4l2_fract *fival)
{
	int i;
	u32 min_delta = INT_MAX;
	struct v4l2_fract target_fival = {0, 0};

	if (!fival)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(vpu_fps_list); i++) {
		struct v4l2_fract *f = &vpu_fps_list[i];
		u32 delta;

		delta = abs(fival->numerator * f->denominator -
				fival->denominator * f->numerator);
		if (!delta)
			return 0;
		if (delta < min_delta) {
			target_fival.numerator = f->numerator;
			target_fival.denominator = f->denominator;
			min_delta = delta;
		}
	}
	if (!target_fival.numerator || !target_fival.denominator)
		return -EINVAL;

	fival->numerator = target_fival.numerator;
	fival->denominator = target_fival.denominator;

	return 0;
}

static int v4l2_ioctl_s_parm(struct file *file, void *fh,
				struct v4l2_streamparm *parm)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct vpu_attr *attr = NULL;
	struct v4l2_fract fival;
	int ret;

	if (!parm || !ctx)
		return -EINVAL;

	attr = get_vpu_ctx_attr(ctx);

	fival.numerator = parm->parm.capture.timeperframe.numerator;
	fival.denominator = parm->parm.capture.timeperframe.denominator;
	if (!fival.numerator || !fival.denominator)
		return -EINVAL;

	ret = find_proper_framerate(&fival);
	if (ret) {
		vpu_dbg(LVL_ERR, "Unsupported FPS : %d / %d\n",
				fival.numerator, fival.denominator);
		return ret;
	}

	mutex_lock(&ctx->instance_mutex);
	attr->param.uFrameRate = fival.denominator / fival.numerator;
	mutex_unlock(&ctx->instance_mutex);

	parm->parm.capture.timeperframe.numerator = fival.numerator;
	parm->parm.capture.timeperframe.denominator = fival.denominator;

	return 0;
}

static int v4l2_ioctl_expbuf(struct file *file,
		void *fh,
		struct v4l2_exportbuffer *buf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;

	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	return (vb2_expbuf(&q_data->vb2_q,
				buf
				));
}

static int v4l2_ioctl_subscribe_event(struct v4l2_fh *fh,
		const struct v4l2_event_subscription *sub
		)
{
	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		return -EINVAL;
	}
}

static int v4l2_ioctl_reqbufs(struct file *file,
		void *fh,
		struct v4l2_requestbuffers *reqbuf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	if (reqbuf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (reqbuf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	ret = vb2_reqbufs(&q_data->vb2_q, reqbuf);

	vpu_dbg(LVL_DEBUG, "%s() c_port_req_buf(%d)\n",
			__func__, ret);

	return ret;
}

static int v4l2_ioctl_querybuf(struct file *file,
		void *fh,
		struct v4l2_buffer *buf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	unsigned int i;
	int ret;

	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	ret = vb2_querybuf(&q_data->vb2_q, buf);
	if (!ret) {
		if (buf->memory == V4L2_MEMORY_MMAP) {
			if (V4L2_TYPE_IS_MULTIPLANAR(buf->type)) {
				for (i = 0; i < buf->length; i++)
					buf->m.planes[i].m.mem_offset |= (q_data->type << MMAP_BUF_TYPE_SHIFT);
			} else
				buf->m.offset |= (q_data->type << MMAP_BUF_TYPE_SHIFT);
		}
	}

	return ret;
}

static struct vb2_buffer *cvrt_v4l2_to_vb2_buffer(struct vb2_queue *vq,
						struct v4l2_buffer *buf)
{
	if (!vq || !buf)
		return NULL;

	if (buf->index < 0 || buf->index >= vq->num_buffers)
		return NULL;

	return vq->bufs[buf->index];
}

static u32 get_v4l2_plane_payload(struct v4l2_plane *plane)
{
	return plane->bytesused - plane->data_offset;
}

static int is_valid_output_mplane_buf(struct queue_data *q_data,
					struct vpu_v4l2_fmt *fmt,
					struct v4l2_buffer *buf)
{
	int i;

	for (i = 0; i < fmt->num_planes; i++) {
		u32 bytesused = get_v4l2_plane_payload(&buf->m.planes[i]);

		if (!bytesused)
			return 0;
		if (fmt->is_yuv && bytesused != q_data->sizeimage[i])
			return 0;
	}

	return 1;
}

static int is_valid_output_buf(struct queue_data *q_data,
				struct vpu_v4l2_fmt *fmt,
				struct v4l2_buffer *buf)
{
	if (!buf->bytesused)
		return 0;
	if (fmt->is_yuv && buf->bytesused != q_data->sizeimage[0])
		return 0;

	return 1;
}

static int precheck_qbuf(struct queue_data *q_data, struct v4l2_buffer *buf)
{
	struct vb2_buffer *vb = NULL;
	struct vpu_v4l2_fmt *fmt;
	int ret;

	if (!q_data || !buf)
		return -EINVAL;

	if (!q_data->current_fmt)
		return -EINVAL;

	vb = cvrt_v4l2_to_vb2_buffer(&q_data->vb2_q, buf);
	if (!vb) {
		vpu_dbg(LVL_ERR, "invalid v4l2 buffer index:%d\n", buf->index);
		return -EINVAL;
	}
	if (vb->state != VB2_BUF_STATE_DEQUEUED) {
		vpu_dbg(LVL_ERR, "invalid buffer state:%d\n", vb->state);
		return -EINVAL;
	}

	if (!V4L2_TYPE_IS_OUTPUT(buf->type))
		return 0;

	fmt = q_data->current_fmt;
	if (V4L2_TYPE_IS_MULTIPLANAR(buf->type))
		ret = is_valid_output_mplane_buf(q_data, fmt, buf);
	else
		ret = is_valid_output_buf(q_data, fmt, buf);
	if (!ret)
		return -EINVAL;

	return 0;
}

static int v4l2_ioctl_qbuf(struct file *file,
		void *fh,
		struct v4l2_buffer *buf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	if (V4L2_TYPE_IS_OUTPUT(buf->type) &&
		test_bit(VPU_ENC_STATUS_STOP_SEND, &ctx->status))
		return -EPIPE;

	ret = precheck_qbuf(q_data, buf);
	if (ret < 0)
		return ret;

	ret = vb2_qbuf(&q_data->vb2_q, buf);

	if (!ret && buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		count_yuv_input(ctx);

	return ret;
}

static void notify_eos(struct vpu_ctx *ctx)
{
	const struct v4l2_event ev = {
		.type = V4L2_EVENT_EOS
	};

	mutex_lock(&ctx->instance_mutex);
	if (!test_bit(VPU_ENC_STATUS_CLOSED, &ctx->status) &&
		!test_and_set_bit(VPU_ENC_STATUS_EOS_SEND, &ctx->status))
		v4l2_event_queue_fh(&ctx->fh, &ev);
	mutex_unlock(&ctx->instance_mutex);
}

static int send_eos(struct vpu_ctx *ctx)
{
	if (!ctx)
		return -EINVAL;

	if (!test_bit(VPU_ENC_STATUS_START_SEND, &ctx->status)) {
		notify_eos(ctx);
		return 0;
	} else if (!test_and_set_bit(VPU_ENC_STATUS_STOP_SEND, &ctx->status)) {
		vpu_dbg(LVL_INFO, "stop stream\n");
		vpu_ctx_send_cmd(ctx, GTB_ENC_CMD_STREAM_STOP, 0, NULL);
	}

	return 0;
}

static int v4l2_ioctl_dqbuf(struct file *file,
		void *fh,
		struct v4l2_buffer *buf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	ret = vb2_dqbuf(&q_data->vb2_q, buf, file->f_flags & O_NONBLOCK);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (!ret)
			count_h264_output(ctx);
		buf->flags = q_data->vb2_reqs[buf->index].buffer_flags;
	}

	return ret;
}

static bool format_is_support(struct vpu_v4l2_fmt *format_table,
		unsigned int table_size,
		struct v4l2_format *f)
{
	unsigned int i;

	for (i = 0; i < table_size; i++) {
		if (format_table[i].fourcc == f->fmt.pix_mp.pixelformat)
			return true;
	}
	return false;
}

static int v4l2_ioctl_try_fmt(struct file *file,
		void *fh,
		struct v4l2_format *f
		)
{
	struct v4l2_pix_format_mplane   *pix_mp = &f->fmt.pix_mp;
	unsigned int table_size;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		table_size = ARRAY_SIZE(formats_compressed_enc);
		if (!format_is_support(formats_compressed_enc, table_size, f))
			return -EINVAL;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pix_mp->field = V4L2_FIELD_ANY;
		pix_mp->colorspace = V4L2_COLORSPACE_REC709;
		table_size = ARRAY_SIZE(formats_yuv_enc);
		if (!format_is_support(formats_yuv_enc, table_size, f))
			return -EINVAL;
	} else
		return -EINVAL;

	return 0;
}

static int v4l2_ioctl_g_crop(struct file *file,
		void *fh,
		struct v4l2_crop *cr
		)
{
	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);
	cr->c.left = 0;
	cr->c.top = 0;
	cr->c.width = 0;
	cr->c.height = 0;

	return 0;
}

static int response_stop_stream(struct vpu_ctx *ctx)
{
	struct queue_data *queue;

	if (!ctx)
		return -EINVAL;

	queue = &ctx->q_data[V4L2_SRC];

	down(&queue->drv_q_lock);
	if (!list_empty(&queue->drv_q))
		goto exit;

	if (!test_bit(VPU_ENC_FLAG_WRITEABLE, &queue->rw_flag))
		goto exit;
	if (test_and_clear_bit(VPU_ENC_STATUS_STOP_REQ, &ctx->status))
		send_eos(ctx);
exit:
	up(&queue->drv_q_lock);

	return 0;
}

static int request_eos(struct vpu_ctx *ctx)
{
	WARN_ON(!ctx);

	set_bit(VPU_ENC_STATUS_STOP_REQ, &ctx->status);
	response_stop_stream(ctx);

	return 0;
}

static void clear_ctx_hang_status(struct vpu_ctx *ctx)
{
	if (test_bit(VPU_ENC_STATUS_STOP_DONE, &ctx->status))
		clear_bit(VPU_ENC_STATUS_HANG, &ctx->status);
}

static bool is_ctx_hang(struct vpu_ctx *ctx)
{
	clear_ctx_hang_status(ctx);
	return test_bit(VPU_ENC_STATUS_HANG, &ctx->status);
}

static int set_core_force_release(struct core_device *core)
{
	int i;

	if (!core)
		return -EINVAL;

	for (i = 0; i < VPU_MAX_NUM_STREAMS; i++) {
		if (!core->ctx[i])
			continue;
		set_bit(VPU_ENC_STATUS_FORCE_RELEASE, &core->ctx[i]->status);
	}

	return 0;
}

static int set_core_hang(struct core_device *core)
{
	set_core_force_release(core);
	core->hang = true;

	return 0;
}

static void clear_core_hang(struct core_device *core)
{
	if (!core)
		return;

	core->hang = false;
}

static void wait_for_stop_done(struct vpu_ctx *ctx)
{
	WARN_ON(!ctx);

	if (is_ctx_hang(ctx))
		return;
	if (!test_bit(VPU_ENC_STATUS_START_SEND, &ctx->status))
		return;
	if (test_bit(VPU_ENC_STATUS_STOP_DONE, &ctx->status))
		return;

	wait_for_completion_timeout(&ctx->stop_cmp, msecs_to_jiffies(500));
	if (!test_bit(VPU_ENC_STATUS_STOP_DONE, &ctx->status))
		set_bit(VPU_ENC_STATUS_HANG, &ctx->status);
}

static int v4l2_ioctl_encoder_cmd(struct file *file,
		void *fh,
		struct v4l2_encoder_cmd *cmd
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	switch (cmd->cmd) {
	case V4L2_ENC_CMD_START:
		break;
	case V4L2_ENC_CMD_STOP:
		request_eos(ctx);
		break;
	case V4L2_ENC_CMD_PAUSE:
		break;
	case V4L2_ENC_CMD_RESUME:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int v4l2_ioctl_streamon(struct file *file,
		void *fh,
		enum v4l2_buf_type i
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	vpu_dbg(LVL_DEBUG, "%s(), type=%d\n", __func__, i);

	if (i == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (i == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;
	ret = vb2_streamon(&q_data->vb2_q, i);
	if (!ret && i == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		clear_bit(VPU_ENC_STATUS_START_SEND, &ctx->status);
		clear_bit(VPU_ENC_STATUS_START_DONE, &ctx->status);
		clear_bit(VPU_ENC_STATUS_STOP_SEND, &ctx->status);
		clear_bit(VPU_ENC_STATUS_STOP_DONE, &ctx->status);
		clear_bit(VPU_ENC_STATUS_EOS_SEND, &ctx->status);
	}
	return ret;
}

static int v4l2_ioctl_streamoff(struct file *file,
		void *fh,
		enum v4l2_buf_type i)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	vpu_dbg(LVL_DEBUG, "%s(), type=%d\n", __func__, i);

	if (i == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (i == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	request_eos(ctx);
	wait_for_stop_done(ctx);

	ret = vb2_streamoff(&q_data->vb2_q, i);
	return ret;
}

static const struct v4l2_ioctl_ops v4l2_encoder_ioctl_ops = {
	.vidioc_querycap                = v4l2_ioctl_querycap,
	.vidioc_enum_fmt_vid_cap_mplane = v4l2_ioctl_enum_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_out_mplane = v4l2_ioctl_enum_fmt_vid_out_mplane,
	.vidioc_enum_framesizes		= v4l2_ioctl_enum_framesizes,
	.vidioc_enum_frameintervals	= v4l2_ioctl_enum_frameintervals,
	.vidioc_g_fmt_vid_cap_mplane    = v4l2_ioctl_g_fmt,
	.vidioc_g_fmt_vid_out_mplane    = v4l2_ioctl_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane  = v4l2_ioctl_try_fmt,
	.vidioc_try_fmt_vid_out_mplane  = v4l2_ioctl_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane    = v4l2_ioctl_s_fmt,
	.vidioc_s_fmt_vid_out_mplane    = v4l2_ioctl_s_fmt,
	.vidioc_g_parm			= v4l2_ioctl_g_parm,
	.vidioc_s_parm			= v4l2_ioctl_s_parm,
	.vidioc_expbuf                  = v4l2_ioctl_expbuf,
	.vidioc_g_crop                  = v4l2_ioctl_g_crop,
	.vidioc_encoder_cmd             = v4l2_ioctl_encoder_cmd,
	.vidioc_subscribe_event         = v4l2_ioctl_subscribe_event,
	.vidioc_unsubscribe_event       = v4l2_event_unsubscribe,
	.vidioc_reqbufs                 = v4l2_ioctl_reqbufs,
	.vidioc_querybuf                = v4l2_ioctl_querybuf,
	.vidioc_qbuf                    = v4l2_ioctl_qbuf,
	.vidioc_dqbuf                   = v4l2_ioctl_dqbuf,
	.vidioc_streamon                = v4l2_ioctl_streamon,
	.vidioc_streamoff               = v4l2_ioctl_streamoff,
};

static void vpu_core_send_cmd(struct core_device *core, u32 idx,
				u32 cmdid, u32 cmdnum, u32 *local_cmddata)
{
	WARN_ON(!core || idx >= VPU_MAX_NUM_STREAMS);

	vpu_log_cmd(cmdid, idx);
	count_cmd(&core->attr[idx], cmdid);

	mutex_lock(&core->cmd_mutex);
	rpc_send_cmd_buf_encoder(&core->shared_mem, idx,
				cmdid, cmdnum, local_cmddata);
	mb();
	MU_SendMessage(core->mu_base_virtaddr, 0, COMMAND);
	mutex_unlock(&core->cmd_mutex);
}

static void vpu_ctx_send_cmd(struct vpu_ctx *ctx, uint32_t cmdid,
				uint32_t cmdnum, uint32_t *local_cmddata)
{
	vpu_core_send_cmd(ctx->core_dev, ctx->str_index,
				cmdid, cmdnum, local_cmddata);
}

static int reset_vpu_core_dev(struct core_device *core_dev)
{
	if (!core_dev)
		return -EINVAL;

	set_core_force_release(core_dev);
	core_dev->fw_is_ready = false;
	core_dev->firmware_started = false;

	return 0;
}

static int sw_reset_firmware(struct core_device *core)
{
	int ret = 0;

	WARN_ON(!core);

	vpu_dbg(LVL_INFO, "sw reset firmware\n");

	init_completion(&core->start_cmp);
	vpu_core_send_cmd(core, 0, GTB_ENC_CMD_FIRM_RESET, 0, NULL);
	ret = wait_for_completion_timeout(&core->start_cmp,
						msecs_to_jiffies(1000));
	if (!ret) {
		vpu_err("error: wait for reset done timeout\n");
		set_core_hang(core);
		return -EINVAL;
	}

	return 0;
}

static int process_core_hang(struct core_device *core)
{
	int ret;

	if (!core->hang)
		return 0;

	ret = sw_reset_firmware(core);
	if (ret)
		return ret;

	clear_core_hang(core);
	return 0;
}

static void show_codec_configure(pMEDIAIP_ENC_PARAM param)
{
	if (!param)
		return;

	vpu_dbg(LVL_INFO, "Encoder Parameter:\n");
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Codec Mode", param->eCodecMode);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Profile", param->eProfile);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Level", param->uLevel);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Mem Phys Addr", param->tEncMemDesc.uMemPhysAddr);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Mem Virt Addr", param->tEncMemDesc.uMemVirtAddr);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Mem Size", param->tEncMemDesc.uMemSize);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Frame Rate", param->uFrameRate);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Source Stride", param->uSrcStride);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Source Width", param->uSrcWidth);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Source Height", param->uSrcHeight);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Source Offset x", param->uSrcOffset_x);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Source Offset y", param->uSrcOffset_y);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Source Crop Width", param->uSrcCropWidth);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Source Crop Height", param->uSrcCropHeight);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Out Width", param->uOutWidth);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Out Height", param->uOutHeight);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"I Frame Interval", param->uIFrameInterval);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"GOP Length", param->uGopBLength);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Low Latency Mode", param->uLowLatencyMode);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Bitrate Mode", param->eBitRateMode);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Target Bitrate", param->uTargetBitrate);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Min Bitrate", param->uMinBitRate);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"Max Bitrate", param->uMaxBitRate);
	vpu_dbg(LVL_INFO, "\t%20s:%16d\n",
			"QP", param->uInitSliceQP);
}

static void show_firmware_version(struct core_device *core_dev)
{
	pENC_RPC_HOST_IFACE pSharedInterface;

	if (!core_dev)
		return;

	pSharedInterface = core_dev->shared_mem.pSharedInterface;

	vpu_dbg(LVL_ALL, "vpu encoder firmware version is %d.%d.%d\n",
			(pSharedInterface->FWVersion & 0x00ff0000) >> 16,
			(pSharedInterface->FWVersion & 0x0000ff00) >> 8,
			pSharedInterface->FWVersion & 0x000000ff);
}

static int alloc_dma_buffer(struct vpu_dev *dev, struct buffer_addr *buffer)
{
	if (!dev || !buffer || !buffer->size)
		return -EINVAL;

	buffer->virt_addr = dma_alloc_coherent(dev->generic_dev,
						buffer->size,
						(dma_addr_t *)&buffer->phy_addr,
						GFP_KERNEL | GFP_DMA32);
	if (!buffer->virt_addr) {
		vpu_dbg(LVL_ERR, "encoder alloc coherent dma(%d) fail\n",
				buffer->size);
		return PTR_ERR(buffer->virt_addr);
	}
	memset_io(buffer->virt_addr, 0, buffer->size);

	return 0;
}

static void init_dma_buffer(struct buffer_addr *buffer)
{
	if (!buffer)
		return;

	buffer->virt_addr = NULL;
	buffer->phy_addr = 0;
	buffer->size = 0;
}

static int free_dma_buffer(struct vpu_dev *dev, struct buffer_addr *buffer)
{
	if (!dev || !buffer)
		return -EINVAL;

	if (!buffer->virt_addr)
		return 0;

	dma_free_coherent(dev->generic_dev, buffer->size,
				buffer->virt_addr, buffer->phy_addr);

	init_dma_buffer(buffer);

	return 0;
}

static int alloc_encoder_stream(struct vpu_ctx *ctx)
{
	int ret;

	if (ctx->encoder_stream.virt_addr)
		return 0;

	ctx->encoder_stream.size = STREAM_SIZE;
	ret = alloc_dma_buffer(ctx->dev, &ctx->encoder_stream);
	if (ret) {
		vpu_dbg(LVL_ERR, "alloc encoder stream buffer fail\n");
		return -ENOMEM;
	}

	return 0;
}

static void free_encoder_stream(struct vpu_ctx *ctx)
{
	if (ctx->encoder_stream.virt_addr) {
		dma_free_coherent(ctx->dev->generic_dev,
				ctx->encoder_stream.size,
				ctx->encoder_stream.virt_addr,
				ctx->encoder_stream.phy_addr);
		ctx->encoder_stream.size = 0;
		ctx->encoder_stream.virt_addr = NULL;
		ctx->encoder_stream.phy_addr = 0;
	}
}

static int configure_codec(struct vpu_ctx *ctx)
{
	pBUFFER_DESCRIPTOR_TYPE pEncStrBuffDesc = NULL;
	pMEDIAIP_ENC_EXPERT_MODE_PARAM pEncExpertModeParam = NULL;
	pMEDIAIP_ENC_PARAM enc_param;
	struct vpu_attr *attr;

	if (!ctx || !ctx->core_dev)
		return -EINVAL;

	attr = get_vpu_ctx_attr(ctx);
	if (!attr)
		return -EINVAL;

	if (alloc_encoder_stream(ctx))
		return -ENOMEM;

	enc_param = get_rpc_enc_param(ctx);
	pEncStrBuffDesc = get_rpc_stream_buffer_desc(ctx);

	pEncStrBuffDesc->start = ctx->encoder_stream.phy_addr;
	pEncStrBuffDesc->wptr = pEncStrBuffDesc->start;
	pEncStrBuffDesc->rptr = pEncStrBuffDesc->start;
	pEncStrBuffDesc->end = ctx->encoder_stream.phy_addr +
				ctx->encoder_stream.size;

	vpu_dbg(LVL_INFO,
		"pEncStrBuffDesc:start=%x, wptr=0x%x, rptr=%x, end=%x\n",
		pEncStrBuffDesc->start,
		pEncStrBuffDesc->wptr,
		pEncStrBuffDesc->rptr,
		pEncStrBuffDesc->end);

	pEncExpertModeParam = get_rpc_expert_mode_param(ctx);
	pEncExpertModeParam->Calib.mem_chunk_phys_addr =
					ctx->encoder_mem.phy_addr;
	pEncExpertModeParam->Calib.mem_chunk_virt_addr =
					ctx->encoder_mem.phy_addr;
	pEncExpertModeParam->Calib.mem_chunk_size = ctx->encoder_mem.size;
	pEncExpertModeParam->Calib.cb_base = ctx->encoder_stream.phy_addr;
	pEncExpertModeParam->Calib.cb_size = ctx->encoder_stream.size;

	show_firmware_version(ctx->core_dev);
	memcpy(enc_param, &attr->param, sizeof(attr->param));
	vpu_ctx_send_cmd(ctx, GTB_ENC_CMD_CONFIGURE_CODEC, 0, NULL);
	vpu_dbg(LVL_INFO, "send command GTB_ENC_CMD_CONFIGURE_CODEC\n");

	show_codec_configure(enc_param);

	return 0;
}

static void dump_vb2_data(struct vb2_buffer *vb)
{
#ifdef DUMP_DATA
	const int DATA_NUM = 10;
	char *read_data;
	u_int32 read_idx;
	char data_str[1024];
	int num = 0;

	if (!vb)
		return;

	read_data = vb2_plane_vaddr(vb, 0);
	num = snprintf(data_str, sizeof(data_str),
			"transfer data from virt 0x%p: ", read_data);
	for (read_idx = 0; read_idx < DATA_NUM; read_idx++)
		num += snprintf(data_str + num, sizeof(data_str) - num,
				" 0x%x", read_data[read_idx]);

	vpu_dbg(LVL_DEBUG, "%s\n", data_str);
#endif
}

static u32 get_vb2_plane_phy_addr(struct vb2_buffer *vb, unsigned int plane_no)
{
	dma_addr_t *dma_addr;

	dma_addr = vb2_plane_cookie(vb, plane_no);
	return *dma_addr + vb->planes[plane_no].data_offset;
}

static bool update_yuv_addr(struct vpu_ctx *ctx)
{
	bool bGotAFrame = FALSE;

	struct vb2_data_req *p_data_req;
	struct queue_data *This = &ctx->q_data[V4L2_SRC];

	pMEDIAIP_ENC_YUV_BUFFER_DESC desc;

	desc = get_rpc_yuv_buffer_desc(ctx);

	if (list_empty(&This->drv_q))
		return bGotAFrame;

	p_data_req = list_first_entry(&This->drv_q, typeof(*p_data_req), list);

	dump_vb2_data(p_data_req->vb2_buf);

	desc->uLumaBase = get_vb2_plane_phy_addr(p_data_req->vb2_buf, 0);
	desc->uChromaBase = get_vb2_plane_phy_addr(p_data_req->vb2_buf, 1);

	if (desc->uLumaBase != 0)
		bGotAFrame = TRUE;

	/*
	 * keeps increasing,
	 * so just a frame input count rather than a Frame buffer ID
	 */
	desc->uFrameID = p_data_req->id;
	if (test_and_clear_bit(VPU_ENC_STATUS_KEY_FRAME, &ctx->status))
		desc->uKeyFrame = 1;
	else
		desc->uKeyFrame = 0;
	list_del(&p_data_req->list);

	return bGotAFrame;

}

static void get_kmp_next(const u8 *p, int *next, int size)
{
	int k = -1;
	int j = 0;

	next[0] = -1;
	while (j < size - 1) {
		if (k == -1 || p[j] == p[k]) {
			++k;
			++j;
			next[j] = k;
		} else {
			k = next[k];
		}
	}
}

static int kmp_serach(u8 *s, int s_len, const u8 *p, int p_len, int *next)
{
	int i = 0;
	int j = 0;

	while (i < s_len && j < p_len) {
		if (j == -1 || s[i] == p[j]) {
			i++;
			j++;
		} else {
			j = next[j];
		}
	}
	if (j == p_len)
		return i - j;
	else
		return -1;
}

static int get_stuff_data_size(u8 *data, int size)
{
	const u8 pattern[] = VPU_STRM_END_PATTERN;
	int next[] = VPU_STRM_END_PATTERN;
	int index;

	if (size < ARRAY_SIZE(pattern))
		return 0;

	get_kmp_next(pattern, next, ARRAY_SIZE(pattern));
	index =  kmp_serach(data, size, pattern, ARRAY_SIZE(pattern), next);
	if (index < 0)
		return 0;
	return size - index - ARRAY_SIZE(pattern);
}

static void strip_stuff_data_on_tail(struct vb2_buffer *vb)
{
	u8 *ptr = vb2_plane_vaddr(vb, 0);
	unsigned long bytesused = vb2_get_plane_payload(vb, 0);
	int count = VPU_TAIL_SERACH_SIZE;
	int stuff_size;

	if (count > bytesused)
		count = bytesused;

	if (!count)
		return;

	stuff_size = get_stuff_data_size(ptr + bytesused - count, count);
	if (stuff_size) {
		vpu_dbg(LVL_WARN, "strip %d bytes stuff data\n", stuff_size);
		vb2_set_plane_payload(vb, 0, bytesused - stuff_size);
	}
}

static void fill_mem_resource(struct core_device *core_dev,
				MEDIAIP_ENC_MEM_RESOURCE *resource,
				struct buffer_addr *buffer)
{
	if (!resource || !buffer) {
		vpu_dbg(LVL_ERR, "invalid arg in %s\n", __func__);
		return;
	}
	resource->uMemPhysAddr =  buffer->phy_addr;
	resource->uMemVirtAddr = cpu_phy_to_mu(core_dev, buffer->phy_addr);
	resource->uMemSize = buffer->size;
}

static void set_mem_pattern(u32 *ptr)
{
	if (!ptr)
		return;
	*ptr = VPU_MEM_PATTERN;
}

static int check_mem_pattern(u32 *ptr)
{
	if (!ptr)
		return -EINVAL;

	if (*ptr != VPU_MEM_PATTERN)
		return -EINVAL;

	return 0;
}

static void set_enc_mem_pattern(struct vpu_ctx *ctx)
{
	u32 i;

	if (!ctx)
		return;

	for (i = 0; i < MEDIAIP_MAX_NUM_WINDSOR_SRC_FRAMES; i++) {
		set_mem_pattern(ctx->encFrame[i].virt_addr - sizeof(u32));
		set_mem_pattern(ctx->encFrame[i].virt_addr +
				ctx->encFrame[i].size);
	}
	for (i = 0; i < MEDIAIP_MAX_NUM_WINDSOR_REF_FRAMES; i++) {
		set_mem_pattern(ctx->refFrame[i].virt_addr - sizeof(u32));
		set_mem_pattern(ctx->refFrame[i].virt_addr +
				ctx->refFrame[i].size);
	}

	set_mem_pattern(ctx->actFrame.virt_addr - sizeof(u32));
	set_mem_pattern(ctx->actFrame.virt_addr + ctx->actFrame.size);
}

static void check_enc_mem_overstep(struct vpu_ctx *ctx)
{
	u32 i;
	int flag = 0;
	int ret;

	if (!ctx || !ctx->enc_buffer.virt_addr)
		return;

	for (i = 0; i < MEDIAIP_MAX_NUM_WINDSOR_SRC_FRAMES; i++) {
		ret = check_mem_pattern(ctx->encFrame[i].virt_addr -
					sizeof(u32));
		if (ret) {
			vpu_err("***error:[%d][%d]encFrame[%d] is dirty\n",
					ctx->core_dev->id, ctx->str_index, i);
			flag = 1;
		}
		ret = check_mem_pattern(ctx->encFrame[i].virt_addr +
					ctx->encFrame[i].size);
		if (ret) {
			vpu_err("***error:[%d][%d]encFrame[%d] out of bounds\n",
					ctx->core_dev->id, ctx->str_index, i);
			flag = 1;
		}
	}

	for (i = 0; i < MEDIAIP_MAX_NUM_WINDSOR_REF_FRAMES; i++) {
		ret = check_mem_pattern(ctx->refFrame[i].virt_addr -
					sizeof(u32));
		if (ret) {
			vpu_err("***error:[%d][%d]refFrame[%d] is dirty\n",
					ctx->core_dev->id, ctx->str_index, i);
			flag = 1;
		}
		ret = check_mem_pattern(ctx->refFrame[i].virt_addr +
					ctx->refFrame[i].size);
		if (ret) {
			vpu_err("***error:[%d][%d]refFrame[%d] out of bounds\n",
					ctx->core_dev->id, ctx->str_index, i);
			flag = 1;
		}
	}

	ret = check_mem_pattern(ctx->actFrame.virt_addr - sizeof(u32));
	if (ret) {
		vpu_err("***error:[%d][%d]actFrame is dirty\n",
				ctx->core_dev->id, ctx->str_index);
		flag = 1;
	}
	ret = check_mem_pattern(ctx->actFrame.virt_addr + ctx->actFrame.size);
	if (ret) {
		vpu_err("***error:[%d][%d]actFrame out of bounds\n",
				ctx->core_dev->id, ctx->str_index);
		flag = 1;
	}

	if (flag) {
		vpu_err("Error:Memory out of bounds in [%d][%d]\n",
			ctx->core_dev->id, ctx->str_index);
		set_enc_mem_pattern(ctx);
	}
}

static u32 calc_enc_mem_size(MEDIAIP_ENC_MEM_REQ_DATA *req_data)
{
	u32 size = PAGE_SIZE;
	u32 i;

	for (i = 0; i < req_data->uEncFrmNum; i++) {
		size += ALIGN(req_data->uEncFrmSize, PAGE_SIZE);
		size += PAGE_SIZE;
	}

	for (i = 0; i < req_data->uRefFrmNum; i++) {
		size += ALIGN(req_data->uRefFrmSize, PAGE_SIZE);
		size += PAGE_SIZE;
	}

	size += ALIGN(req_data->uActBufSize, PAGE_SIZE);
	size += PAGE_SIZE;

	return size;
}

static int enc_mem_free(struct vpu_ctx *ctx)
{
	u32 i;

	if (!ctx)
		return -EINVAL;

	free_dma_buffer(ctx->dev, &ctx->enc_buffer);

	for (i = 0; i < MEDIAIP_MAX_NUM_WINDSOR_SRC_FRAMES; i++)
		init_dma_buffer(&ctx->encFrame[i]);
	for (i = 0; i < MEDIAIP_MAX_NUM_WINDSOR_REF_FRAMES; i++)
		init_dma_buffer(&ctx->refFrame[i]);
	init_dma_buffer(&ctx->actFrame);

	return 0;
}

static int enc_mem_alloc(struct vpu_ctx *ctx,
			MEDIAIP_ENC_MEM_REQ_DATA *req_data)
{
	struct core_device *core_dev;
	pMEDIAIP_ENC_MEM_POOL pEncMemPool;
	int ret;
	u_int32 i;
	u32 offset = 0;

	if (!ctx || !ctx->core_dev || !req_data)
		return -EINVAL;

	vpu_dbg(LVL_INFO, "encFrame:%d,%d; refFrame:%d,%d; actFrame:%d\n",
			req_data->uEncFrmSize,
			req_data->uEncFrmNum,
			req_data->uRefFrmSize,
			req_data->uRefFrmNum,
			req_data->uActBufSize);

	enc_mem_free(ctx);

	ctx->enc_buffer.size = calc_enc_mem_size(req_data);
	vpu_dbg(LVL_INFO, "alloc %d dma for encFrame/refFrame/actFrame\n",
			ctx->enc_buffer.size);
	ret = alloc_dma_buffer(ctx->dev, &ctx->enc_buffer);
	if (ret) {
		vpu_dbg(LVL_ERR, "alloc encoder buffer fail\n");
		return ret;
	}

	core_dev = ctx->core_dev;
	pEncMemPool = get_rpc_mem_pool(ctx);
	offset = PAGE_SIZE;
	for (i = 0; i < req_data->uEncFrmNum; i++) {
		ctx->encFrame[i].size = req_data->uEncFrmSize;
		ctx->encFrame[i].phy_addr = ctx->enc_buffer.phy_addr + offset;
		ctx->encFrame[i].virt_addr = ctx->enc_buffer.virt_addr + offset;
		offset += ALIGN(ctx->encFrame[i].size, PAGE_SIZE);
		offset += PAGE_SIZE;

		vpu_dbg(LVL_INFO, "encFrame[%d]: 0x%llx, 0x%x\n", i,
			ctx->encFrame[i].phy_addr, ctx->encFrame[i].size);

		fill_mem_resource(core_dev,
				&pEncMemPool->tEncFrameBuffers[i],
				&ctx->encFrame[i]);
	}
	for (i = 0; i < req_data->uRefFrmNum; i++) {
		ctx->refFrame[i].size = req_data->uRefFrmSize;
		ctx->refFrame[i].phy_addr = ctx->enc_buffer.phy_addr + offset;
		ctx->refFrame[i].virt_addr = ctx->enc_buffer.virt_addr + offset;
		offset += ALIGN(ctx->refFrame[i].size, PAGE_SIZE);
		offset += PAGE_SIZE;

		vpu_dbg(LVL_INFO, "refFrame[%d]: 0x%llx, 0x%x\n", i,
			ctx->refFrame[i].phy_addr, ctx->refFrame[i].size);

		fill_mem_resource(core_dev,
				&pEncMemPool->tRefFrameBuffers[i],
				&ctx->refFrame[i]);
	}

	ctx->actFrame.size = req_data->uActBufSize;
	ctx->actFrame.phy_addr = ctx->enc_buffer.phy_addr + offset;
	ctx->actFrame.virt_addr = ctx->enc_buffer.virt_addr + offset;
	offset += ALIGN(ctx->actFrame.size, PAGE_SIZE);
	offset += PAGE_SIZE;

	vpu_dbg(LVL_INFO, "actFrame: 0x%llx, 0x%x\n",
			ctx->actFrame.phy_addr, ctx->actFrame.size);

	fill_mem_resource(core_dev,
			&pEncMemPool->tActFrameBufferArea, &ctx->actFrame);

	set_enc_mem_pattern(ctx);

	return 0;
}

static int check_enc_rw_flag(int flag)
{
	int ret = -EINVAL;

	switch (flag) {
	case VPU_ENC_FLAG_WRITEABLE:
	case VPU_ENC_FLAG_READABLE:
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

static void set_queue_rw_flag(struct queue_data *queue, int flag)
{
	if (!queue)
		return;

	if (check_enc_rw_flag(flag))
		return;

	set_bit(flag, &queue->rw_flag);
}

static void clear_queue_rw_flag(struct queue_data *queue, int flag)
{
	if (!queue)
		return;

	if (check_enc_rw_flag(flag))
		return;

	clear_bit(flag, &queue->rw_flag);
}

static int submit_input_and_encode(struct vpu_ctx *ctx)
{
	struct queue_data *queue;

	if (!ctx)
		return -EINVAL;

	queue = &ctx->q_data[V4L2_SRC];

	down(&queue->drv_q_lock);

	if (!test_bit(VPU_ENC_FLAG_WRITEABLE, &queue->rw_flag))
		goto exit;

	if (list_empty(&queue->drv_q))
		goto exit;

	if (test_bit(VPU_ENC_STATUS_STOP_SEND, &ctx->status))
		goto exit;

	if (update_yuv_addr(ctx)) {
		vpu_ctx_send_cmd(ctx, GTB_ENC_CMD_FRAME_ENCODE, 0, NULL);
		clear_queue_rw_flag(queue, VPU_ENC_FLAG_WRITEABLE);
	}
exit:
	up(&queue->drv_q_lock);

	return 0;
}

static void add_rptr(struct vpu_frame_info *frame, u32 length)
{
	WARN_ON(!frame);

	frame->rptr += length;
	if (frame->rptr >= frame->end)
		frame->rptr -= (frame->end - frame->start);
}

static void report_frame_type(struct vb2_data_req *p_data_req,
				struct vpu_frame_info *frame)
{
	WARN_ON(!p_data_req || !frame);

	switch (frame->info.ePicType) {
	case MEDIAIP_ENC_PIC_TYPE_IDR_FRAME:
	case MEDIAIP_ENC_PIC_TYPE_I_FRAME:
		p_data_req->buffer_flags = V4L2_BUF_FLAG_KEYFRAME;
		break;
	case MEDIAIP_ENC_PIC_TYPE_P_FRAME:
		p_data_req->buffer_flags = V4L2_BUF_FLAG_PFRAME;
		break;
	case MEDIAIP_ENC_PIC_TYPE_B_FRAME:
		p_data_req->buffer_flags = V4L2_BUF_FLAG_BFRAME;
		break;
	default:
		break;
	}
}

static u32 calc_frame_length(struct vpu_frame_info *frame)
{
	u32 length;
	u32 buffer_size;

	WARN_ON(!frame);

	if (frame->eos)
		return 0;

	buffer_size = frame->end - frame->start;
	if (!buffer_size)
		return 0;

	length = (frame->wptr - frame->rptr + buffer_size) % buffer_size;

	return length;
}

static u32 get_ptr(u32 ptr)
{
	return (ptr | 0x80000000);
}

static void *get_rptr_virt(struct vpu_ctx *ctx, struct vpu_frame_info *frame)
{
	WARN_ON(!ctx || !frame);

	return ctx->encoder_stream.virt_addr + frame->rptr - frame->start;
}

static int transfer_stream_output(struct vpu_ctx *ctx,
					struct vpu_frame_info *frame,
					struct vb2_data_req *p_data_req)
{
	struct vb2_buffer *vb = NULL;
	u32 length;
	void *pdst;

	WARN_ON(!ctx || !frame || !p_data_req);

	length = calc_frame_length(frame);
	if (!length)
		return 0;

	vb = p_data_req->vb2_buf;
	vb2_set_plane_payload(vb, 0, length);
	pdst = vb2_plane_vaddr(vb, 0);
	if (frame->rptr + length <= frame->end) {
		memcpy(pdst, get_rptr_virt(ctx, frame), length);
		add_rptr(frame, length);
	} else {
		u32 offset = frame->end - frame->rptr;

		memcpy(pdst, get_rptr_virt(ctx, frame), offset);
		add_rptr(frame, offset);
		length -= offset;
		memcpy(pdst + offset, get_rptr_virt(ctx, frame), length);
		add_rptr(frame, length);
	}
	report_frame_type(p_data_req, frame);

	return 0;
}

static int append_empty_end_frame(struct vb2_data_req *p_data_req)
{
	WARN_ON(!p_data_req);

	vb2_set_plane_payload(p_data_req->vb2_buf, 0, 0);
	p_data_req->buffer_flags = V4L2_BUF_FLAG_LAST;

	vpu_dbg(LVL_INFO, "append en empty frame as the last frame\n");

	return 0;
}

static void process_frame_done(struct queue_data *queue)
{
	struct vpu_ctx *ctx;
	struct vb2_data_req *p_data_req = NULL;
	struct vpu_frame_info *frame = NULL;
	pBUFFER_DESCRIPTOR_TYPE stream_buffer_desc;

	WARN_ON(!queue || !queue->ctx);

	ctx = queue->ctx;

	if (list_empty(&queue->drv_q))
		return;
	if (list_empty(&queue->frame_q))
		return;

	stream_buffer_desc = get_rpc_stream_buffer_desc(ctx);
	p_data_req = list_first_entry(&queue->drv_q, typeof(*p_data_req), list);
	frame = list_first_entry(&queue->frame_q, typeof(*frame), list);
	frame->rptr = get_ptr(stream_buffer_desc->rptr);

	if (frame->eos)
		append_empty_end_frame(p_data_req);
	else if (calc_frame_length(frame))
		transfer_stream_output(ctx, frame, p_data_req);
	else
		return;

	stream_buffer_desc->rptr = frame->rptr;
	list_del(&p_data_req->list);
	if (frame && !calc_frame_length(frame)) {
		list_del(&frame->list);
		vfree(frame);
	}

	strip_stuff_data_on_tail(p_data_req->vb2_buf);
	if (p_data_req->vb2_buf->state == VB2_BUF_STATE_ACTIVE)
		vb2_buffer_done(p_data_req->vb2_buf, VB2_BUF_STATE_DONE);
}

static int process_stream_output(struct vpu_ctx *ctx)
{
	struct queue_data *queue = NULL;

	if (!ctx)
		return -EINVAL;

	queue = &ctx->q_data[V4L2_DST];

	down(&queue->drv_q_lock);
	process_frame_done(queue);
	up(&queue->drv_q_lock);

	return 0;
}

static void show_enc_pic_info(MEDIAIP_ENC_PIC_INFO *pEncPicInfo)
{
#ifdef TB_REC_DBG
	vpu_dbg(LVL_DEBUG, "       - Frame ID      : 0x%x\n",
			pEncPicInfo->uFrameID);

	switch (pEncPicInfo->ePicType) {
	case MEDIAIP_ENC_PIC_TYPE_IDR_FRAME:
		vpu_dbg(LVL_DEBUG, "       - Picture Type  : IDR picture\n");
		break;
	case MEDIAIP_ENC_PIC_TYPE_I_FRAME:
		vpu_dbg(LVL_DEBUG, "       - Picture Type  : I picture\n");
		break;
	case MEDIAIP_ENC_PIC_TYPE_P_FRAME:
		vpu_dbg(LVL_DEBUG, "       - Picture Type  : P picture\n");
		break;
	case MEDIAIP_ENC_PIC_TYPE_B_FRAME:
		vpu_dbg(LVL_DEBUG, "       - Picture Type  : B picture\n");
		break;
	default:
		vpu_dbg(LVL_DEBUG, "       - Picture Type  : BI picture\n");
		break;
	}
	vpu_dbg(LVL_DEBUG, "       - Skipped frame : 0x%x\n",
			pEncPicInfo->uSkippedFrame);
	vpu_dbg(LVL_DEBUG, "       - Frame size    : 0x%x\n",
			pEncPicInfo->uFrameSize);
	vpu_dbg(LVL_DEBUG, "       - Frame CRC     : 0x%x\n",
			pEncPicInfo->uFrameCrc);
#endif
}

static int handle_event_frame_done(struct vpu_ctx *ctx,
				MEDIAIP_ENC_PIC_INFO *pEncPicInfo)
{
	struct vpu_frame_info *frame;

	if (!ctx || !pEncPicInfo)
		return -EINVAL;

	vpu_dbg(LVL_DEBUG, "pEncPicInfo->uPicEncodDone=%d\n",
			pEncPicInfo->uPicEncodDone);

	if (!pEncPicInfo->uPicEncodDone) {
		vpu_err("Pic Encoder Not Done\n");
		return -EINVAL;
	}

	show_enc_pic_info(pEncPicInfo);

	count_encoded_frame(ctx);
	frame = vmalloc(sizeof(*frame));
	if (frame) {
		struct queue_data *queue = &ctx->q_data[V4L2_DST];
		pBUFFER_DESCRIPTOR_TYPE stream_buffer_desc;

		stream_buffer_desc = get_rpc_stream_buffer_desc(ctx);
		memcpy(&frame->info, pEncPicInfo, sizeof(frame->info));
		frame->wptr = get_ptr(stream_buffer_desc->wptr);
		frame->rptr = get_ptr(stream_buffer_desc->rptr);
		frame->start = get_ptr(stream_buffer_desc->start);
		frame->end = get_ptr(stream_buffer_desc->end);
		frame->eos = false;

		down(&queue->drv_q_lock);
		list_add_tail(&frame->list, &queue->frame_q);
		up(&queue->drv_q_lock);
	} else {
		vpu_err("fail to alloc memory for frame info\n");
	}

	/* Sync the write pointer to the local view of it */
	process_stream_output(ctx);

	return 0;
}

static int handle_event_frame_release(struct vpu_ctx *ctx, u_int32 *uFrameID)
{
	struct queue_data *This = &ctx->q_data[V4L2_SRC];
	struct vb2_data_req *p_data_req;

	if (!ctx || !uFrameID)
		return -EINVAL;

	This = &ctx->q_data[V4L2_SRC];
	vpu_dbg(LVL_DEBUG, "Frame release - uFrameID = 0x%x\n", *uFrameID);
	p_data_req = &This->vb2_reqs[*uFrameID];
	if (p_data_req->vb2_buf->state == VB2_BUF_STATE_ACTIVE)
		vb2_buffer_done(p_data_req->vb2_buf, VB2_BUF_STATE_DONE);

	return 0;
}

static int handle_event_stop_done(struct vpu_ctx *ctx)
{
	struct vpu_frame_info *frame;

	WARN_ON(!ctx);

	set_bit(VPU_ENC_STATUS_STOP_DONE, &ctx->status);
	notify_eos(ctx);

	frame = vmalloc(sizeof(*frame));
	if (frame) {
		struct queue_data *queue = &ctx->q_data[V4L2_DST];

		memset(frame, 0, sizeof(*frame));
		frame->eos = true;

		down(&queue->drv_q_lock);
		list_add_tail(&frame->list, &queue->frame_q);
		up(&queue->drv_q_lock);
	} else {
		vpu_err("fail to alloc memory for last frame\n");
	}

	process_stream_output(ctx);

	complete(&ctx->stop_cmp);

	return 0;
}

static void vpu_api_event_handler(struct vpu_ctx *ctx,
				u_int32 uEvent, u_int32 *event_data)
{
	vpu_log_event(uEvent, ctx->str_index);
	count_event(ctx, uEvent);
	check_enc_mem_overstep(ctx);

	switch (uEvent) {
	case VID_API_ENC_EVENT_START_DONE:
		set_bit(VPU_ENC_STATUS_START_DONE, &ctx->status);
		set_queue_rw_flag(&ctx->q_data[V4L2_SRC],
				VPU_ENC_FLAG_WRITEABLE);
		submit_input_and_encode(ctx);
		break;
	case VID_API_ENC_EVENT_MEM_REQUEST:
		enc_mem_alloc(ctx, (MEDIAIP_ENC_MEM_REQ_DATA *)event_data);
		vpu_ctx_send_cmd(ctx, GTB_ENC_CMD_STREAM_START, 0, NULL);
		set_bit(VPU_ENC_STATUS_START_SEND, &ctx->status);
		break;
	case VID_API_ENC_EVENT_PARA_UPD_DONE:
		break;
	case VID_API_ENC_EVENT_FRAME_DONE:
		handle_event_frame_done(ctx,
					(MEDIAIP_ENC_PIC_INFO *)event_data);
		break;
	case VID_API_ENC_EVENT_FRAME_RELEASE:
		handle_event_frame_release(ctx, (u_int32 *)event_data);
		break;
	case VID_API_ENC_EVENT_STOP_DONE:
		handle_event_stop_done(ctx);
		break;
	case VID_API_ENC_EVENT_FRAME_INPUT_DONE:
		set_queue_rw_flag(&ctx->q_data[V4L2_SRC],
				VPU_ENC_FLAG_WRITEABLE);
		response_stop_stream(ctx);
		submit_input_and_encode(ctx);
		break;
	case VID_API_ENC_EVENT_TERMINATE_DONE:
		break;
	case VID_API_ENC_EVENT_RESET_DONE:
		break;
	default:
		vpu_dbg(LVL_ERR, "........unknown event : 0x%x\n", uEvent);
		break;
	}
}

static void enable_mu(struct core_device *dev)
{
	u32 mu_addr;

	vpu_dbg(LVL_ALL, "enable mu for core[%d]\n", dev->id);

	rpc_init_shared_memory_encoder(&dev->shared_mem,
				cpu_phy_to_mu(dev, dev->m0_rpc_phy),
				dev->m0_rpc_virt, dev->rpc_buf_size,
				&dev->rpc_actual_size);
	rpc_set_system_cfg_value_encoder(dev->shared_mem.pSharedInterface,
				VPU_REG_BASE, dev->id);

	if (dev->rpc_actual_size > dev->rpc_buf_size)
		vpu_err("rpc actual size(0x%x) > (0x%x), may occur overlay\n",
			dev->rpc_actual_size, dev->rpc_buf_size);

	mu_addr = cpu_phy_to_mu(dev, dev->m0_rpc_phy + dev->rpc_buf_size);
	MU_sendMesgToFW(dev->mu_base_virtaddr, PRINT_BUF_OFFSET, mu_addr);

	mu_addr = cpu_phy_to_mu(dev, dev->m0_rpc_phy);
	MU_sendMesgToFW(dev->mu_base_virtaddr, RPC_BUF_OFFSET, mu_addr);

	MU_sendMesgToFW(dev->mu_base_virtaddr, BOOT_ADDRESS,
			dev->m0_p_fw_space_phy);
	MU_sendMesgToFW(dev->mu_base_virtaddr, INIT_DONE, 2);
}

//This code is added for MU
static irqreturn_t fsl_vpu_mu_isr(int irq, void *This)
{
	struct core_device *dev = This;
	u32 msg;

	MU_ReceiveMsg(dev->mu_base_virtaddr, 0, &msg);
	if (msg == 0xaa) {
		enable_mu(dev);
	} else if (msg == 0x55) {
		dev->firmware_started = true;
		complete(&dev->start_cmp);
	}  else if (msg == 0xA5) {
		/*receive snapshot done msg and wakeup complete to suspend*/
		complete(&dev->snap_done_cmp);
	} else
		queue_work(dev->workqueue, &dev->msg_work);
	return IRQ_HANDLED;
}

/* Initialization of the MU code. */
static int vpu_mu_init(struct core_device *core_dev)
{
	struct device_node *np;
	unsigned int	vpu_mu_id;
	u32 irq;
	int ret = 0;

	/*
	 * Get the address of MU to be used for communication with the M0 core
	 */
	np = of_find_compatible_node(NULL, NULL, mu_cmp[core_dev->id]);
	if (!np) {
		vpu_dbg(LVL_ERR, "error: Cannot find MU entry in device tree\n");
		return -EINVAL;
	}
	core_dev->mu_base_virtaddr = of_iomap(np, 0);
	WARN_ON(!core_dev->mu_base_virtaddr);

	ret = of_property_read_u32_index(np,
				"fsl,vpu_ap_mu_id", 0, &vpu_mu_id);
	if (ret) {
		vpu_dbg(LVL_ERR, "Cannot get mu_id %d\n", ret);
		return -EINVAL;
	}

	core_dev->vpu_mu_id = vpu_mu_id;

	irq = of_irq_get(np, 0);

	ret = devm_request_irq(core_dev->generic_dev, irq, fsl_vpu_mu_isr,
				IRQF_EARLY_RESUME, "vpu_mu_isr", (void *)core_dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "request_irq failed %d, error = %d\n", irq, ret);
		return -EINVAL;
	}

	if (!core_dev->vpu_mu_init) {
		MU_Init(core_dev->mu_base_virtaddr);
		MU_EnableRxFullInt(core_dev->mu_base_virtaddr, 0);
		core_dev->vpu_mu_init = 1;
	}

	return ret;
}

static void send_msg_queue(struct vpu_ctx *ctx, struct event_msg *msg)
{
	u_int32 ret;

	ret = kfifo_in(&ctx->msg_fifo, msg, sizeof(struct event_msg));
	if (ret != sizeof(struct event_msg))
		vpu_dbg(LVL_ERR, "There is no memory for msg fifo, ret=%d\n", ret);
}

static bool receive_msg_queue(struct vpu_ctx *ctx, struct event_msg *msg)
{
	u_int32 ret;

	if (kfifo_len(&ctx->msg_fifo) >= sizeof(*msg)) {
		ret = kfifo_out(&ctx->msg_fifo, msg, sizeof(*msg));
		if (ret != sizeof(*msg)) {
			vpu_dbg(LVL_ERR, "kfifo_out has error, ret=%d\n", ret);
			return false;
		} else
			return true;
	} else
		return false;
}

extern u_int32 rpc_MediaIPFW_Video_message_check_encoder(struct shared_addr *This);
static void vpu_msg_run_work(struct work_struct *work)
{
	struct core_device *dev = container_of(work, struct core_device, msg_work);
	struct vpu_ctx *ctx;
	struct event_msg msg;
	struct shared_addr *This = &dev->shared_mem;

	while (rpc_MediaIPFW_Video_message_check_encoder(This) == API_MSG_AVAILABLE) {
		rpc_receive_msg_buf_encoder(This, &msg);
		if (msg.idx >= VPU_MAX_NUM_STREAMS) {
			vpu_err("msg idx(%d) is out of range\n", msg.idx);
			continue;
		}
		mutex_lock(&dev->core_mutex);
		ctx = dev->ctx[msg.idx];
		if (ctx != NULL) {
			mutex_lock(&ctx->instance_mutex);
			if (!ctx->ctx_released) {
				send_msg_queue(ctx, &msg);
				queue_work(ctx->instance_wq, &ctx->instance_work);
			}
			mutex_unlock(&ctx->instance_mutex);
		}
		mutex_unlock(&dev->core_mutex);
	}
	if (rpc_MediaIPFW_Video_message_check_encoder(This) == API_MSG_BUFFER_ERROR)
		vpu_dbg(LVL_ERR, "MSG num is too big to handle");

}
static void vpu_msg_instance_work(struct work_struct *work)
{
	struct vpu_ctx *ctx = container_of(work, struct vpu_ctx, instance_work);
	struct event_msg msg;

	while (receive_msg_queue(ctx, &msg))
		vpu_api_event_handler(ctx, msg.msgid, msg.msgdata);
}

static int vpu_queue_setup(struct vb2_queue *vq,
		unsigned int *buf_count,
		unsigned int *plane_count,
		unsigned int psize[],
		struct device *allocators[])
{
	struct queue_data  *This = (struct queue_data *)vq->drv_priv;

	vpu_dbg(LVL_DEBUG, "%s() is called\n", __func__);

	if ((vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) ||
		(vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		) {
		*plane_count = 1;
		psize[0] = This->sizeimage[0];//check alignment
	} else {
		if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
			*plane_count = 2;
			psize[0] = This->sizeimage[0];//check alignment
			psize[1] = This->sizeimage[1];//check colocated_size
		} else {
			psize[0] = This->sizeimage[0] + This->sizeimage[1];
			*plane_count = 1;
		}
	}
	return 0;
}

static int vpu_buf_prepare(struct vb2_buffer *vb)
{
	vpu_dbg(LVL_DEBUG, "%s() is called\n", __func__);
	return 0;
}


static int vpu_start_streaming(struct vb2_queue *q,
		unsigned int count
		)
{
	vpu_dbg(LVL_DEBUG, "%s() is called\n", __func__);

	return 0;
}

static void clear_queue(struct queue_data *queue)
{
	struct vpu_frame_info *frame;
	struct vpu_frame_info *tmp;

	if (!queue)
		return;

	down(&queue->drv_q_lock);

	list_for_each_entry_safe(frame, tmp, &queue->frame_q, list) {
		list_del(&frame->list);
		vfree(frame);
	}

	INIT_LIST_HEAD(&queue->frame_q);

	up(&queue->drv_q_lock);
}

static void vpu_stop_streaming(struct vb2_queue *q)
{
	struct queue_data *This = (struct queue_data *)q->drv_priv;
	struct vb2_data_req *p_data_req;
	struct vb2_data_req *p_temp;
	struct vb2_buffer *vb;

	vpu_dbg(LVL_DEBUG, "%s() is called\n", __func__);

	down(&This->drv_q_lock);
	if (!list_empty(&This->drv_q)) {
		list_for_each_entry_safe(p_data_req, p_temp, &This->drv_q, list) {
			vpu_dbg(LVL_DEBUG, "%s(%d) - list_del(%p)\n",
					__func__,
					p_data_req->id,
					p_data_req
					);
			list_del(&p_data_req->list);
		}
	}

	if (!list_empty(&q->queued_list))
		list_for_each_entry(vb, &q->queued_list, queued_entry)
			if (vb->state == VB2_BUF_STATE_ACTIVE)
				vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
	INIT_LIST_HEAD(&This->drv_q);
	up(&This->drv_q_lock);
}

static void vpu_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue    *vq = vb->vb2_queue;
	struct queue_data   *This = (struct queue_data *)vq->drv_priv;
	struct vb2_data_req *data_req;

	vpu_dbg(LVL_DEBUG, "%s() is called\n", __func__);

	down(&This->drv_q_lock);
	vpu_dbg(LVL_DEBUG, "c_port_buf_queue down\n");
	data_req = &This->vb2_reqs[vb->index];
	data_req->vb2_buf = vb;
	data_req->id = vb->index;
	list_add_tail(&data_req->list, &This->drv_q);

	up(&This->drv_q_lock);
	vpu_dbg(LVL_DEBUG, "c_port_buf_queue up vq->type=%d\n", vq->type);

	if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		struct vpu_ctx *ctx = This->ctx;

		mutex_lock(&ctx->instance_mutex);
		if (!test_and_set_bit(VPU_ENC_STATUS_CONFIGURED, &ctx->status))
			configure_codec(ctx);
		mutex_unlock(&ctx->instance_mutex);

		submit_input_and_encode(ctx);
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		process_stream_output(This->ctx);
	}
}

static void vpu_prepare(struct vb2_queue *q)
{
	vpu_dbg(LVL_DEBUG, "%s() is called\n", __func__);
}

static void vpu_finish(struct vb2_queue *q)
{
	vpu_dbg(LVL_DEBUG, "%s() is called\n", __func__);
}

static struct vb2_ops v4l2_qops = {
	.queue_setup        = vpu_queue_setup,
	.wait_prepare       = vpu_prepare,
	.wait_finish        = vpu_finish,
	.buf_prepare        = vpu_buf_prepare,
	.start_streaming    = vpu_start_streaming,
	.stop_streaming     = vpu_stop_streaming,
	.buf_queue          = vpu_buf_queue,
};

static void init_vb2_queue(struct queue_data *This, unsigned int type,
				struct vpu_ctx *ctx,
				const struct vb2_mem_ops *mem_ops,
				gfp_t gfp_flags)
{
	struct vb2_queue  *vb2_q = &This->vb2_q;
	int ret;

	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	// initialze driver queue
	INIT_LIST_HEAD(&This->drv_q);
	INIT_LIST_HEAD(&This->frame_q);
	// initialize vb2 queue
	vb2_q->type = type;
	vb2_q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	vb2_q->gfp_flags = gfp_flags;
	vb2_q->ops = &v4l2_qops;
	vb2_q->drv_priv = This;
	if (mem_ops)
		vb2_q->mem_ops = mem_ops;
	else
		vb2_q->mem_ops = &vb2_dma_contig_memops;
	vb2_q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	vb2_q->dev = &ctx->dev->plat_dev->dev;
	ret = vb2_queue_init(vb2_q);
	if (ret)
		vpu_dbg(LVL_ERR, "%s vb2_queue_init() failed (%d)!\n",
				__func__,
				ret
				);
	else
		This->vb2_q_inited = true;
}

static void init_queue_data(struct vpu_ctx *ctx)
{
	init_vb2_queue(&ctx->q_data[V4L2_SRC],
			V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			ctx,
			&vb2_dma_contig_memops,
			GFP_DMA32);
	ctx->q_data[V4L2_SRC].type = V4L2_SRC;
	sema_init(&ctx->q_data[V4L2_SRC].drv_q_lock, 1);
	init_vb2_queue(&ctx->q_data[V4L2_DST],
			V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			ctx,
			&vb2_vmalloc_memops, 0);
	ctx->q_data[V4L2_DST].type = V4L2_DST;
	sema_init(&ctx->q_data[V4L2_DST].drv_q_lock, 1);

	ctx->q_data[V4L2_SRC].ctx = ctx;
	ctx->q_data[V4L2_DST].ctx = ctx;

	ctx->q_data[V4L2_SRC].supported_fmts = formats_yuv_enc;
	ctx->q_data[V4L2_SRC].fmt_count = ARRAY_SIZE(formats_yuv_enc);
	ctx->q_data[V4L2_DST].supported_fmts = formats_compressed_enc;
	ctx->q_data[V4L2_DST].fmt_count = ARRAY_SIZE(formats_compressed_enc);
}

static void release_queue_data(struct vpu_ctx *ctx)
{
	struct queue_data *This = &ctx->q_data[V4L2_SRC];

	if (This->vb2_q_inited)
		vb2_queue_release(&This->vb2_q);
	This = &ctx->q_data[V4L2_DST];
	if (This->vb2_q_inited)
		vb2_queue_release(&This->vb2_q);
}

static void vpu_ctx_power_on(struct vpu_ctx *ctx)
{
	if (!ctx || !ctx->core_dev)
		return;

	if (ctx->power_status)
		return;
	pm_runtime_get_sync(ctx->core_dev->generic_dev);
	ctx->power_status = true;
}

static void vpu_ctx_power_off(struct vpu_ctx *ctx)
{
	if (!ctx || !ctx->core_dev)
		return;

	if (!ctx->power_status)
		return;
	pm_runtime_put_sync(ctx->core_dev->generic_dev);
	ctx->power_status = false;
}

static int set_vpu_fw_addr(struct vpu_dev *dev, struct core_device *core_dev)
{
	if (!dev || !core_dev)
		return -EINVAL;

	MU_Init(core_dev->mu_base_virtaddr);
	MU_EnableRxFullInt(core_dev->mu_base_virtaddr, 0);

	write_enc_reg(dev, core_dev->m0_p_fw_space_phy, core_dev->reg_fw_base);
	write_enc_reg(dev, 0x0, core_dev->reg_fw_base + 4);

	return 0;
}

static int vpu_firmware_download(struct vpu_dev *This, u_int32 core_id)
{
	const struct firmware *m0_pfw = NULL;
	const u8 *image;
	unsigned int FW_Size = 0;
	int ret = 0;
	char *p = This->core_dev[core_id].m0_p_fw_space_vir;

	ret = request_firmware(&m0_pfw, M0FW_FILENAME, This->generic_dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "%s() request fw %s failed(%d)\n",
			__func__, M0FW_FILENAME, ret);

		return ret;
	}
	vpu_dbg(LVL_DEBUG, "%s() request fw %s got size(%ld)\n",
			__func__, M0FW_FILENAME, m0_pfw->size);

	image = m0_pfw->data;
	FW_Size = min_t(u32, m0_pfw->size, This->core_dev[core_id].fw_buf_size);
	This->core_dev[core_id].fw_actual_size = FW_Size;

	memcpy(This->core_dev[core_id].m0_p_fw_space_vir, image, FW_Size);
	p[16] = This->plat_type;
	p[17] = core_id + 1;
	set_vpu_fw_addr(This, &This->core_dev[core_id]);

	release_firmware(m0_pfw);
	m0_pfw = NULL;

	return ret;
}

static int download_vpu_firmware(struct vpu_dev *dev,
				struct core_device *core_dev)
{
	int ret = 0;

	if (!dev || !core_dev)
		return -EINVAL;

	if (core_dev->fw_is_ready)
		return 0;

	init_completion(&core_dev->start_cmp);
	ret = vpu_firmware_download(dev, core_dev->id);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: vpu_firmware_download fail\n");
		goto exit;
	}
	wait_for_completion_timeout(&core_dev->start_cmp,
					msecs_to_jiffies(100));
	if (!core_dev->firmware_started) {
		vpu_err("core[%d] start firmware failed\n", core_dev->id);
		ret = -EINVAL;
		goto exit;
	}

	core_dev->fw_is_ready = true;
	clear_core_hang(core_dev);
exit:
	return ret;
}

static void free_instance(struct vpu_ctx *ctx)
{
	if (!ctx)
		return;

	if (ctx->dev && ctx->core_dev && ctx->str_index < VPU_MAX_NUM_STREAMS)
		ctx->core_dev->ctx[ctx->str_index] = NULL;
	kfree(ctx);
}

static u32 count_core_instance_num(struct core_device *core)
{
	int i;
	u32 count = 0;

	for (i = 0; i < VPU_MAX_NUM_STREAMS; i++) {
		if (core->ctx[i])
			count++;
	}

	return count;
}

static struct core_device *find_proper_core(struct vpu_dev *dev)
{
	struct core_device *core = NULL;
	u32 minimum = VPU_MAX_NUM_STREAMS;
	u32 count;
	int i;
	int ret;

	for (i = 0; i < dev->core_num; i++) {
		process_core_hang(dev->core_dev + i);

		ret = download_vpu_firmware(dev, dev->core_dev + i);
		if (ret)
			continue;

		count = count_core_instance_num(dev->core_dev + i);
		if (count < minimum) {
			minimum = count;
			core = dev->core_dev + i;
		}
		if (minimum == 0)
			break;
	}

	return core;
}

static int request_instance(struct core_device *core, struct vpu_ctx *ctx)
{
	int found = 0;
	int idx;

	if (!core || !ctx)
		return -EINVAL;

	for (idx = 0; idx < VPU_MAX_NUM_STREAMS; idx++) {
		if (!core->ctx[idx]) {
			found = 1;
			ctx->core_dev = core;
			ctx->str_index = idx;
			ctx->dev = core->vdev;
			core->ctx[idx] = ctx;
			break;
		}
	}

	if (!found) {
		vpu_dbg(LVL_ERR, "cann't request any instance\n");
		return -EBUSY;
	}

	return 0;
}

static int construct_vpu_ctx(struct vpu_ctx *ctx)
{
	if (!ctx)
		return -EINVAL;

	ctx->ctrl_inited = false;
	mutex_init(&ctx->instance_mutex);
	ctx->ctx_released = false;

	return 0;
}

static struct vpu_ctx *create_and_request_instance(struct vpu_dev *dev)
{
	struct core_device *core = NULL;
	struct vpu_ctx *ctx = NULL;
	int ret;

	if (!dev)
		return NULL;

	core = find_proper_core(dev);
	if (!core)
		return NULL;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return NULL;

	ret = request_instance(core, ctx);
	if (ret < 0) {
		kfree(ctx);
		return NULL;
	}

	construct_vpu_ctx(ctx);
	vpu_ctx_power_on(ctx);
	vpu_dbg(LVL_INFO, "request encoder instance : %d.%d\n",
			ctx->core_dev->id, ctx->str_index);

	return ctx;
}

static int init_vpu_ctx_fh(struct vpu_ctx *ctx, struct vpu_dev *dev)
{
	if (!ctx || !dev)
		return -EINVAL;

	mutex_lock(&ctx->instance_mutex);

	v4l2_fh_init(&ctx->fh, dev->pvpu_encoder_dev);
	v4l2_fh_add(&ctx->fh);
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	clear_bit(VPU_ENC_STATUS_CLOSED, &ctx->status);

	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

static void uninit_vpu_ctx_fh(struct vpu_ctx *ctx)
{
	if (!ctx)
		return;

	mutex_lock(&ctx->instance_mutex);

	set_bit(VPU_ENC_STATUS_CLOSED, &ctx->status);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);

	mutex_unlock(&ctx->instance_mutex);
}

static void uninit_vpu_ctx(struct vpu_ctx *ctx)
{
	if (!ctx)
		return;

	mutex_lock(&ctx->instance_mutex);
	if (ctx->instance_wq) {
		cancel_work_sync(&ctx->instance_work);
		destroy_workqueue(ctx->instance_wq);
		ctx->instance_wq = NULL;
	}
	free_encoder_stream(ctx);

	kfifo_free(&ctx->msg_fifo);
	ctx->ctx_released = true;
	mutex_unlock(&ctx->instance_mutex);
}

static int init_vpu_ctx(struct vpu_ctx *ctx)
{
	int ret;

	INIT_WORK(&ctx->instance_work, vpu_msg_instance_work);
	ctx->instance_wq = alloc_workqueue("vpu_instance",
				WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!ctx->instance_wq) {
		vpu_dbg(LVL_ERR, "error: unable to alloc workqueue for ctx\n");
		return -ENOMEM;
	}

	ret = kfifo_alloc(&ctx->msg_fifo,
			sizeof(struct event_msg) * VID_API_MESSAGE_LIMIT,
			GFP_KERNEL);
	if (ret) {
		vpu_dbg(LVL_ERR, "fail to alloc fifo when open\n");
		goto error;
	}

	init_queue_data(ctx);
	init_completion(&ctx->stop_cmp);

	set_bit(VPU_ENC_STATUS_INITIALIZED, &ctx->status);

	return 0;
error:
	uninit_vpu_ctx(ctx);
	return ret;
}

static ssize_t show_instance_info(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vpu_attr *vpu_attr;
	struct vpu_dev *vpudev;
	struct vpu_statistic *statistic;
	pMEDIAIP_ENC_PARAM param;
	int i;
	int num = 0;
	int size;

	vpu_attr = container_of(attr, struct vpu_attr,  dev_attr);
	vpudev = vpu_attr->core->vdev;

	statistic = &vpu_attr->statistic;
	param = rpc_get_enc_param(&vpu_attr->core->shared_mem, vpu_attr->index);

	num += snprintf(buf + num, PAGE_SIZE,
			"pid: %d; tgid: %d\n", vpu_attr->pid, vpu_attr->tgid);
	num += snprintf(buf + num, PAGE_SIZE, "cmd:\n");

	for (i = GTB_ENC_CMD_NOOP; i < GTB_ENC_CMD_RESERVED; i++) {
		size = snprintf(buf + num, PAGE_SIZE - num,
				"\t%40s(%2d):%16ld\n",
				cmd2str[i], i, statistic->cmd[i]);
		num += size;
	}
	num += snprintf(buf + num, PAGE_SIZE - num, "\t%40s    :%16ld\n",
			"UNKNOWN CMD", statistic->cmd[GTB_ENC_CMD_RESERVED]);

	num += snprintf(buf + num, PAGE_SIZE - num, "event:\n");
	for (i = VID_API_EVENT_UNDEFINED; i < VID_API_ENC_EVENT_RESERVED; i++) {
		size = snprintf(buf + num, PAGE_SIZE - num,
				"\t%40s(%2d):%16ld\n",
				event2str[i], i, statistic->event[i]);
		num += size;
	}
	num += snprintf(buf + num, PAGE_SIZE - num, "\t%40s    :%16ld\n",
			"UNKNOWN EVENT",
			statistic->event[VID_API_ENC_EVENT_RESERVED]);

	num += snprintf(buf + num, PAGE_SIZE - num,
			"encoder param:[setting/take effect]\n");
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Codec Mode",
			vpu_attr->param.eCodecMode, param->eCodecMode);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Profile",
			vpu_attr->param.eProfile, param->eProfile);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Level",
			vpu_attr->param.uLevel, param->uLevel);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Frame Rate",
			vpu_attr->param.uFrameRate, param->uFrameRate);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Source Stride",
			vpu_attr->param.uSrcStride, param->uSrcStride);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Source Width",
			vpu_attr->param.uSrcWidth, param->uSrcWidth);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Source Height",
			vpu_attr->param.uSrcHeight, param->uSrcHeight);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Source Offset x",
			vpu_attr->param.uSrcOffset_x, param->uSrcOffset_x);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Source Offset y",
			vpu_attr->param.uSrcOffset_y, param->uSrcOffset_y);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Source Crop Width",
			vpu_attr->param.uSrcCropWidth, param->uSrcCropWidth);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Source Crop Height",
			vpu_attr->param.uSrcCropHeight,
			param->uSrcCropHeight);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Out Width",
			vpu_attr->param.uOutWidth, param->uOutWidth);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Out Height",
			vpu_attr->param.uOutHeight, param->uOutHeight);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "I Frame Interval",
			vpu_attr->param.uIFrameInterval,
			param->uIFrameInterval);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "GOP Length",
			vpu_attr->param.uGopBLength, param->uGopBLength);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Low Latency Mode",
			vpu_attr->param.uLowLatencyMode,
			param->uLowLatencyMode);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Bitrate Mode",
			vpu_attr->param.eBitRateMode, param->eBitRateMode);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Target Bitrate",
			vpu_attr->param.uTargetBitrate,
			param->uTargetBitrate);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Min Bitrate",
			vpu_attr->param.uMinBitRate, param->uMinBitRate);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Max Bitrate",
			vpu_attr->param.uMaxBitRate, param->uMaxBitRate);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "QP",
			vpu_attr->param.uInitSliceQP,
			param->uInitSliceQP);

	num += snprintf(buf + num, PAGE_SIZE - num, "current status:\n");
	num += snprintf(buf + num, PAGE_SIZE - num,
			"%10s:%40s;%10ld.%06ld\n", "commond",
			get_cmd_str(statistic->current_cmd),
			statistic->ts_cmd.tv_sec,
			statistic->ts_cmd.tv_nsec / 1000);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"%10s:%40s;%10ld.%06ld\n", "event",
			get_event_str(statistic->current_event),
			statistic->ts_event.tv_sec,
			statistic->ts_event.tv_nsec / 1000);

	num += snprintf(buf + num, PAGE_SIZE - num,
			"dbuf input yuv count:    %ld\n", statistic->yuv_count);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"encode frame count:      %ld\n",
			statistic->encoded_count);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"dqbuf output h264 count: %ld\n",
			statistic->h264_count);
	if (!vpu_attr->core->ctx[vpu_attr->index])
		num += snprintf(buf + num, PAGE_SIZE - num,
			"<instance has been released>\n");

	return num;
}

static ssize_t show_core_info(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct core_device *core = NULL;
	char *fw = NULL;
	int num = 0;

	core = container_of(attr, struct core_device, core_attr);
	fw = core->m0_p_fw_space_vir;

	num += snprintf(buf + num, PAGE_SIZE - num,
			"core[%d] info:\n", core->id);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"vpu mu id       :%d\n", core->vpu_mu_id);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"reg fw base     :0x%08lx\n", core->reg_fw_base);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"fw space phy    :0x%08x\n", core->m0_p_fw_space_phy);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"fw space size   :0x%08x\n", core->fw_buf_size);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"fw actual size  :0x%08x\n", core->fw_actual_size);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"rpc phy         :0x%08x\n", core->m0_rpc_phy);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"rpc buf size    :0x%08x\n", core->rpc_buf_size);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"rpc actual size :0x%08x\n", core->rpc_actual_size);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"print buf phy   :0x%08x\n",
			core->m0_rpc_phy + core->rpc_buf_size);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"print buf size  :0x%08x\n", core->print_buf_size);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"fw info         :0x%02x 0x%02x\n", fw[16], fw[17]);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"fw_is_ready     :%d\n", core->fw_is_ready);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"firmware_started:%d\n", core->firmware_started);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"hang            :%d\n", core->hang);
	return num;
}

static int init_vpu_attr(struct vpu_attr *attr)
{
	if (!attr || !attr->core)
		return -EINVAL;

	memset(&attr->statistic, 0, sizeof(attr->statistic));
	memset(&attr->param, 0, sizeof(attr->param));
	attr->pid = current->pid;
	attr->tgid = current->tgid;
	if (!attr->created) {
		device_create_file(attr->core->generic_dev, &attr->dev_attr);
		attr->created = true;
	}

	return 0;
}

static int release_instance(struct vpu_ctx *ctx)
{
	struct vpu_dev *dev;

	if (!ctx || !ctx->dev)
		return -EINVAL;

	if (!test_bit(VPU_ENC_STATUS_FORCE_RELEASE, &ctx->status)) {
		if (!test_bit(VPU_ENC_STATUS_CLOSED, &ctx->status))
			return 0;
		if (test_bit(VPU_ENC_STATUS_START_SEND, &ctx->status) &&
			!test_bit(VPU_ENC_STATUS_STOP_DONE, &ctx->status))
			return -EINVAL;
		if (is_ctx_hang(ctx))
			return -EINVAL;
	}

	dev = ctx->dev;

	clear_queue(&ctx->q_data[V4L2_SRC]);
	clear_queue(&ctx->q_data[V4L2_DST]);

	uninit_vpu_ctx(ctx);
	vpu_enc_free_ctrls(ctx);
	release_queue_data(ctx);
	enc_mem_free(ctx);

	vpu_ctx_power_off(ctx);
	free_instance(ctx);

	return 0;
}

static int try_to_release_idle_instance(struct vpu_dev *dev)
{
	int i;
	int j;

	if (!dev)
		return -EINVAL;

	for (i = 0; i < dev->core_num; i++) {
		for (j = 0; j < VPU_MAX_NUM_STREAMS; j++)
			release_instance(dev->core_dev[i].ctx[j]);
	}

	return 0;
}

struct vpu_attr *get_vpu_ctx_attr(struct vpu_ctx *ctx)
{
	WARN_ON(!ctx || !ctx->core_dev);

	if (ctx->str_index >= VPU_MAX_NUM_STREAMS)
		return NULL;

	return &ctx->core_dev->attr[ctx->str_index];
}

static int vpu_enc_v4l2_open(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct vpu_dev *dev = video_get_drvdata(vdev);
	struct vpu_ctx *ctx = NULL;
	int ret;

	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	mutex_lock(&dev->dev_mutex);
	try_to_release_idle_instance(dev);

	pm_runtime_get_sync(dev->generic_dev);
	ctx = create_and_request_instance(dev);
	pm_runtime_put_sync(dev->generic_dev);
	mutex_unlock(&dev->dev_mutex);
	if (!ctx) {
		vpu_dbg(LVL_ERR, "failed to create encoder ctx\n");
		return -ENOMEM;
	}

	init_vpu_attr(get_vpu_ctx_attr(ctx));
	ret = init_vpu_ctx(ctx);
	if (ret) {
		vpu_dbg(LVL_ERR, "init vpu ctx fail\n");
		goto error;
	}

	initialize_enc_param(ctx);
	init_queue_data(ctx);
	vpu_enc_setup_ctrls(ctx);

	init_vpu_ctx_fh(ctx, dev);
	filp->private_data = &ctx->fh;

	return 0;
error:
	mutex_lock(&dev->dev_mutex);
	set_bit(VPU_ENC_STATUS_FORCE_RELEASE, &ctx->status);
	release_instance(ctx);
	mutex_unlock(&dev->dev_mutex);
	return ret;
}

static int vpu_enc_v4l2_release(struct file *filp)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(filp->private_data);
	struct vpu_dev *dev = ctx->dev;

	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	request_eos(ctx);
	wait_for_stop_done(ctx);

	uninit_vpu_ctx_fh(ctx);
	filp->private_data = NULL;

	mutex_lock(&dev->dev_mutex);
	release_instance(ctx);
	mutex_unlock(&dev->dev_mutex);

	return 0;
}

static unsigned int vpu_enc_v4l2_poll(struct file *filp, poll_table *wait)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(filp->private_data);
	struct vb2_queue *src_q, *dst_q;
	unsigned int rc = 0;

	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	if (ctx) {
		poll_wait(filp, &ctx->fh.wait, wait);

		if (v4l2_event_pending(&ctx->fh)) {
			vpu_dbg(LVL_DEBUG, "%s() v4l2_event_pending\n", __func__);
			rc |= POLLPRI;
		}

		src_q = &ctx->q_data[V4L2_SRC].vb2_q;
		dst_q = &ctx->q_data[V4L2_DST].vb2_q;

		if ((!src_q->streaming || list_empty(&src_q->queued_list))
				&& (!dst_q->streaming || list_empty(&dst_q->queued_list))) {
			return rc;
		}

		poll_wait(filp, &src_q->done_wq, wait);
		if (!list_empty(&src_q->done_list))
			rc |= POLLOUT | POLLWRNORM;
		poll_wait(filp, &dst_q->done_wq, wait);
		if (!list_empty(&dst_q->done_list))
			rc |= POLLIN | POLLRDNORM;
	} else
		rc = POLLERR;

	return rc;
}

static int vpu_enc_v4l2_mmap(struct file *filp, struct vm_area_struct *vma)
{
	long ret = -EPERM;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	struct queue_data *q_data;
	enum QUEUE_TYPE type;

	struct vpu_ctx *ctx = v4l2_fh_to_ctx(filp->private_data);

	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	if (ctx) {
		type = offset >> MMAP_BUF_TYPE_SHIFT;
		q_data = &ctx->q_data[type];

		offset &= ~MMAP_BUF_TYPE_MASK;
		offset = offset >> PAGE_SHIFT;
		vma->vm_pgoff = offset;
		ret = vb2_mmap(&q_data->vb2_q,
						vma
						);
	}

	return ret;
}

static const struct v4l2_file_operations v4l2_encoder_fops = {
	.owner = THIS_MODULE,
	.open  = vpu_enc_v4l2_open,
	.unlocked_ioctl = video_ioctl2,
	.release = vpu_enc_v4l2_release,
	.poll = vpu_enc_v4l2_poll,
	.mmap = vpu_enc_v4l2_mmap,
};

static struct video_device v4l2_videodevice_encoder = {
	.name   = "vpu encoder",
	.fops   = &v4l2_encoder_fops,
	.ioctl_ops = &v4l2_encoder_ioctl_ops,
	.vfl_dir = VFL_DIR_M2M,
};

static void vpu_setup(struct vpu_dev *This)
{
	uint32_t read_data = 0;

	vpu_dbg(LVL_INFO, "enter %s\n", __func__);
	writel(0x1, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_SCB_CLK_ENABLE_SET);
	writel(0xffffffff, This->regs_base + 0x70190);
	writel(0xffffffff, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_XMEM_RESET_SET);

	writel(0xE, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_SCB_CLK_ENABLE_SET);
	writel(0x7, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_CACHE_RESET_SET);

	writel(0x102, This->regs_base + XMEM_CONTROL);

	read_data = readl(This->regs_base+0x70108);
	vpu_dbg(LVL_IRQ, "%s read_data=%x\n", __func__, read_data);
}

static void vpu_reset(struct vpu_dev *This)
{
	vpu_dbg(LVL_INFO, "enter %s\n", __func__);
	writel(0x7, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_CACHE_RESET_CLR);
}

static int vpu_enable_hw(struct vpu_dev *This)
{
	vpu_dbg(LVL_INFO, "%s()\n", __func__);
	vpu_setup(This);

	This->hw_enable = true;

	return 0;
}

static void vpu_disable_hw(struct vpu_dev *This)
{
	This->hw_enable = false;
	vpu_reset(This);
	if (This->regs_base) {
		iounmap(This->regs_base);
		This->regs_base = NULL;
	}
}

static int get_platform_info_by_core_type(struct vpu_dev *dev, u32 core_type)
{
	int ret = 0;

	if (!dev)
		return -EINVAL;

	switch (core_type) {
	case 1:
		dev->plat_type = IMX8QXP;
		dev->core_num = 1;
		break;
	case 2:
		dev->plat_type = IMX8QM;
		dev->core_num = 2;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int parse_core_info(struct core_device *core, struct device_node *np)
{
	int ret;
	u32 val;

	WARN_ON(!core || !np);

	ret = of_property_read_u32_index(np, "reg-fw-base", core->id, &val);
	if (ret) {
		vpu_err("find reg-fw-base for core[%d] fail\n", core->id);
		return ret;
	}
	core->reg_fw_base = val;

	ret = of_property_read_u32_index(np, "fw-buf_size", core->id, &val);
	if (ret) {
		vpu_err("find fw-buf-size for core[%d] fail\n", core->id);
		core->fw_buf_size = M0_BOOT_SIZE_DEFAULT;
	} else {
		core->fw_buf_size = val;
	}
	core->fw_buf_size = max_t(u32, core->fw_buf_size, M0_BOOT_SIZE_MIN);

	ret = of_property_read_u32_index(np, "rpc-buf-size", core->id, &val);
	if (ret) {
		vpu_err("find rpc-buf-size for core[%d] fail\n", core->id);
		core->rpc_buf_size = RPC_SIZE_DEFAULT;
	} else {
		core->rpc_buf_size = val;
	}
	core->rpc_buf_size = max_t(u32, core->rpc_buf_size, RPC_SIZE_MIN);

	ret = of_property_read_u32_index(np, "print-buf-size", core->id, &val);
	if (ret) {
		vpu_err("find print-buf-size for core[%d] fail\n", core->id);
		core->print_buf_size = PRINT_SIZE_DEFAULT;
	} else {
		core->print_buf_size = val;
	}
	core->print_buf_size = max_t(u32, core->print_buf_size, PRINT_SIZE_MIN);

	return 0;
}

static int parse_dt_info(struct vpu_dev *dev, struct device_node *np)
{
	int ret;
	struct device_node *reserved_node = NULL;
	struct resource reserved_fw;
	struct resource reserved_rpc;
	u_int32 core_type;
	u32 fw_total_size = 0;
	u32 rpc_total_size = 0;
	u32 i;

	if (!dev || !np)
		return -EINVAL;

	ret = of_property_read_u32(np, "core_type", &core_type);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: Cannot get core num %d\n", ret);
		return -EINVAL;
	}
	ret = get_platform_info_by_core_type(dev, core_type);
	if (ret) {
		vpu_dbg(LVL_ERR, "invalid core_type : %d\n", core_type);
		return ret;
	}
	reserved_node = of_parse_phandle(np, "boot-region", 0);
	if (!reserved_node) {
		vpu_dbg(LVL_ERR, "error: boot-region of_parse_phandle error\n");
		return -ENODEV;
	}
	if (of_address_to_resource(reserved_node, 0, &reserved_fw)) {
		vpu_dbg(LVL_ERR,
			"error: boot-region of_address_to_resource error\n");
		return -EINVAL;
	}

	reserved_node = of_parse_phandle(np, "rpc-region", 0);
	if (!reserved_node) {
		vpu_dbg(LVL_ERR,
			"error: rpc-region of_parse_phandle error\n");
		return -ENODEV;
	}
	if (of_address_to_resource(reserved_node, 0, &reserved_rpc)) {
		vpu_dbg(LVL_ERR,
			"error: rpc-region of_address_to_resource error\n");
		return -EINVAL;
	}

	fw_total_size = 0;
	rpc_total_size = 0;
	for (i = 0; i < dev->core_num; i++) {
		struct core_device *core = &dev->core_dev[i];

		core->id = i;
		ret = parse_core_info(core, np);
		if (ret)
			return ret;

		core->m0_p_fw_space_phy = reserved_fw.start + fw_total_size;
		core->m0_rpc_phy = reserved_rpc.start + rpc_total_size;
		fw_total_size += core->fw_buf_size;
		rpc_total_size += core->rpc_buf_size;
		rpc_total_size += core->print_buf_size;
	}

	if (fw_total_size > resource_size(&reserved_fw)) {
		vpu_err("boot-region's size(0x%llx) is less than wanted:0x%x\n",
				resource_size(&reserved_fw), fw_total_size);
		return -EINVAL;
	}
	if (rpc_total_size > resource_size(&reserved_rpc)) {
		vpu_err("rpc-region's size(0x%llx) is less than wanted:0x%x\n",
				resource_size(&reserved_rpc), rpc_total_size);
		return -EINVAL;
	}

	return 0;
}

static int create_vpu_video_device(struct vpu_dev *dev)
{
	int ret;

	dev->pvpu_encoder_dev = video_device_alloc();
	if (!dev->pvpu_encoder_dev) {
		vpu_dbg(LVL_ERR, "alloc vpu encoder video device fail\n");
		return -ENOMEM;
	}

	strncpy(dev->pvpu_encoder_dev->name,
		v4l2_videodevice_encoder.name,
		sizeof(v4l2_videodevice_encoder.name));
	dev->pvpu_encoder_dev->fops = v4l2_videodevice_encoder.fops;
	dev->pvpu_encoder_dev->ioctl_ops = v4l2_videodevice_encoder.ioctl_ops;
	dev->pvpu_encoder_dev->release = video_device_release;
	dev->pvpu_encoder_dev->vfl_dir = v4l2_videodevice_encoder.vfl_dir;
	dev->pvpu_encoder_dev->v4l2_dev = &dev->v4l2_dev;
	video_set_drvdata(dev->pvpu_encoder_dev, dev);

	ret = video_register_device(dev->pvpu_encoder_dev,
					VFL_TYPE_GRABBER,
					ENCODER_NODE_NUMBER);
	if (ret) {
		vpu_dbg(LVL_ERR, "unable to register video encoder device\n");
		video_device_release(dev->pvpu_encoder_dev);
		dev->pvpu_encoder_dev = NULL;
		return ret;
	}

	return 0;
}

static int init_vpu_attrs(struct core_device *core)
{
	int i;

	WARN_ON(!core);

	for (i = 0; i < VPU_MAX_NUM_STREAMS; i++) {
		struct vpu_attr *attr = &core->attr[i];

		attr->core = core;
		attr->index = i;
		snprintf(attr->name, sizeof(attr->name) - 1, "instance.%d.%d",
				core->id, attr->index);
		attr->dev_attr.attr.name = attr->name;
		attr->dev_attr.attr.mode = VERIFY_OCTAL_PERMISSIONS(0444);
		attr->dev_attr.show = show_instance_info;

		attr->created = false;
	}

	return 0;
}

static int release_vpu_attrs(struct core_device *core)
{
	int i;

	WARN_ON(!core);

	for (i = 0; i < VPU_MAX_NUM_STREAMS; i++) {
		struct vpu_attr *attr = &core->attr[i];

		if (!attr->created)
			continue;
		device_remove_file(attr->core->generic_dev, &attr->dev_attr);
	}

	return 0;
}

static int init_vpu_core_dev(struct core_device *core_dev)
{
	int ret;

	if (!core_dev)
		return -EINVAL;

	mutex_init(&core_dev->core_mutex);
	mutex_init(&core_dev->cmd_mutex);
	init_completion(&core_dev->start_cmp);
	init_completion(&core_dev->snap_done_cmp);

	core_dev->workqueue = alloc_workqueue("vpu",
					WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!core_dev->workqueue) {
		vpu_dbg(LVL_ERR, "%s unable to alloc workqueue\n", __func__);
		ret = -ENOMEM;
		return ret;
	}

	INIT_WORK(&core_dev->msg_work, vpu_msg_run_work);

	ret = vpu_mu_init(core_dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "%s vpu mu init failed\n", __func__);
		goto error;
	}
	//firmware space for M0
	core_dev->m0_p_fw_space_vir =
		ioremap_wc(core_dev->m0_p_fw_space_phy, core_dev->fw_buf_size);
	if (!core_dev->m0_p_fw_space_vir)
		vpu_dbg(LVL_ERR, "failed to remap space for M0 firmware\n");

	memset_io(core_dev->m0_p_fw_space_vir, 0, core_dev->fw_buf_size);

	core_dev->m0_rpc_virt =
		ioremap_wc(core_dev->m0_rpc_phy, core_dev->rpc_buf_size);
	if (!core_dev->m0_rpc_virt)
		vpu_dbg(LVL_ERR, "failed to remap space for shared memory\n");

	memset_io(core_dev->m0_rpc_virt, 0, core_dev->rpc_buf_size);

	reset_vpu_core_dev(core_dev);

	init_vpu_attrs(core_dev);

	snprintf(core_dev->name, sizeof(core_dev->name) - 1,
			"core.%d", core_dev->id);
	core_dev->core_attr.attr.name = core_dev->name;
	core_dev->core_attr.attr.mode = VERIFY_OCTAL_PERMISSIONS(0444);
	core_dev->core_attr.show = show_core_info;
	device_create_file(core_dev->generic_dev, &core_dev->core_attr);

	return 0;
error:
	if (core_dev->workqueue) {
		destroy_workqueue(core_dev->workqueue);
		core_dev->workqueue = NULL;
	}
	return ret;
}

static int uninit_vpu_core_dev(struct core_device *core_dev)
{
	if (!core_dev)
		return -EINVAL;

	device_remove_file(core_dev->generic_dev, &core_dev->core_attr);
	release_vpu_attrs(core_dev);
	if (core_dev->workqueue) {
		cancel_work_sync(&core_dev->msg_work);
		destroy_workqueue(core_dev->workqueue);
		core_dev->workqueue = NULL;
	}

	if (core_dev->m0_p_fw_space_vir) {
		iounmap(core_dev->m0_p_fw_space_vir);
		core_dev->m0_p_fw_space_vir = NULL;
	}
	core_dev->m0_p_fw_space_phy = 0;

	if (core_dev->m0_rpc_virt) {
		iounmap(core_dev->m0_rpc_virt);
		core_dev->m0_rpc_virt = NULL;
	}
	core_dev->m0_rpc_phy = 0;

	if (core_dev->mu_base_virtaddr) {
		iounmap(core_dev->mu_base_virtaddr);
		core_dev->mu_base_virtaddr = NULL;
	}

	if (core_dev->generic_dev) {
		put_device(core_dev->generic_dev);
		core_dev->generic_dev = NULL;
	}

	return 0;
}

static int vpu_probe(struct platform_device *pdev)
{
	struct vpu_dev *dev;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;
	u_int32 i;
	int ret;

	if (!np) {
		vpu_dbg(LVL_ERR, "error: %s of_node is NULL\n", __func__);
		return -EINVAL;
	}

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->plat_dev = pdev;
	dev->generic_dev = get_device(&pdev->dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		vpu_dbg(LVL_ERR, "Missing platform resource data\n");
		ret = -EINVAL;
		goto error_put_dev;
	}
	dev->regs_enc = devm_ioremap_resource(dev->generic_dev, res);
	if (!dev->regs_enc) {
		vpu_dbg(LVL_ERR, "couldn't map encoder reg\n");
		ret = PTR_ERR(dev->regs_enc);
		goto error_put_dev;
	}
	dev->regs_base = ioremap(ENC_REG_BASE, 0x1000000);
	if (!dev->regs_base) {
		vpu_dbg(LVL_ERR, "%s could not map regs_base\n", __func__);
		ret = PTR_ERR(dev->regs_base);
		goto error_put_dev;
	}

	ret = parse_dt_info(dev, np);
	if (ret) {
		vpu_dbg(LVL_ERR, "parse device tree fail\n");
		goto error_iounmap;
	}

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "%s unable to register v4l2 dev\n", __func__);
		goto error_iounmap;
	}

	platform_set_drvdata(pdev, dev);

	ret = create_vpu_video_device(dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "create vpu video device fail\n");
		goto error_unreg_v4l2;
	}


	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	vpu_enable_hw(dev);

	mutex_init(&dev->dev_mutex);
	mutex_lock(&dev->dev_mutex);
	for (i = 0; i < dev->core_num; i++) {
		dev->core_dev[i].id = i;
		dev->core_dev[i].generic_dev = get_device(dev->generic_dev);
		dev->core_dev[i].vdev = dev;
		ret = init_vpu_core_dev(&dev->core_dev[i]);
		if (ret)
			goto error_init_core;
	}
	mutex_unlock(&dev->dev_mutex);
	pm_runtime_put_sync(&pdev->dev);

	vpu_dbg(LVL_ALL, "VPU Encoder registered\n");

	return 0;

error_init_core:
	mutex_lock(&dev->dev_mutex);
	for (i = 0; i < dev->core_num; i++)
		uninit_vpu_core_dev(&dev->core_dev[i]);
	mutex_unlock(&dev->dev_mutex);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	vpu_disable_hw(dev);

	if (dev->pvpu_encoder_dev) {
		video_unregister_device(dev->pvpu_encoder_dev);
		dev->pvpu_encoder_dev = NULL;
	}
error_unreg_v4l2:
	v4l2_device_unregister(&dev->v4l2_dev);
error_iounmap:
	if (dev->regs_base)
		iounmap(dev->regs_base);
error_put_dev:
	if (dev->generic_dev) {
		put_device(dev->generic_dev);
		dev->generic_dev = NULL;
	}
	return ret;
}

static int vpu_remove(struct platform_device *pdev)
{
	struct vpu_dev *dev = platform_get_drvdata(pdev);
	u_int32 i;

	mutex_lock(&dev->dev_mutex);
	for (i = 0; i < dev->core_num; i++)
		uninit_vpu_core_dev(&dev->core_dev[i]);
	mutex_unlock(&dev->dev_mutex);

	vpu_disable_hw(dev);
	pm_runtime_disable(&pdev->dev);

	if (video_get_drvdata(dev->pvpu_encoder_dev))
		video_unregister_device(dev->pvpu_encoder_dev);

	v4l2_device_unregister(&dev->v4l2_dev);
	if (dev->generic_dev) {
		put_device(dev->generic_dev);
		dev->generic_dev = NULL;
	}

	return 0;
}

static int vpu_runtime_suspend(struct device *dev)
{
	return 0;
}

static int vpu_runtime_resume(struct device *dev)
{
	return 0;
}

static int is_core_activated(struct core_device *core)
{
	WARN_ON(!core);

	if (readl_relaxed(core->mu_base_virtaddr + MU_B0_REG_CONTROL) == 0)
		return false;
	else
		return true;
}

static int is_need_shapshot(struct vpu_ctx *ctx)
{
	if (is_ctx_hang(ctx))
		return 0;
	if (!test_bit(VPU_ENC_STATUS_INITIALIZED, &ctx->status))
		return 0;
	if (!test_bit(VPU_ENC_STATUS_CONFIGURED, &ctx->status))
		return 0;
	if (test_bit(VPU_ENC_STATUS_CLOSED, &ctx->status))
		return 0;
	if (test_bit(VPU_ENC_STATUS_STOP_SEND, &ctx->status))
		return 0;
	if (test_bit(VPU_ENC_STATUS_STOP_DONE, &ctx->status))
		return 0;

	return 1;
}

static int vpu_snapshot(struct vpu_ctx *ctx)
{
	int ret;

	if (!ctx)
		return -EINVAL;

	vpu_ctx_send_cmd(ctx, GTB_ENC_CMD_SNAPSHOT, 0, NULL);
	ret = wait_for_completion_timeout(&ctx->core_dev->snap_done_cmp,
						msecs_to_jiffies(1000));
	if (!ret) {
		vpu_err("error:wait for snapdone event timeout!\n");
		return -EINVAL;
	}
	ctx->core_dev->snapshot = true;

	return 0;
}

static int resume_from_snapshot(struct core_device *core)
{
	int ret = 0;

	if (!core)
		return -EINVAL;
	if (!core->snapshot)
		return 0;

	vpu_dbg(LVL_INFO, "resume from snapshot\n");

	init_completion(&core->start_cmp);
	set_vpu_fw_addr(core->vdev, core);
	ret = wait_for_completion_timeout(&core->start_cmp,
						msecs_to_jiffies(1000));
	if (!ret) {
		vpu_err("error: wait for resume done timeout!\n");
		reset_vpu_core_dev(core);
		return -EINVAL;
	}

	return 0;
}

static int suspend_instance(struct vpu_ctx *ctx)
{
	int ret = 0;

	if (!ctx)
		return 0;

	if (test_bit(VPU_ENC_STATUS_STOP_REQ, &ctx->status) ||
		test_bit(VPU_ENC_STATUS_STOP_SEND, &ctx->status))
		wait_for_stop_done(ctx);

	if (!ctx->core_dev->snapshot && is_need_shapshot(ctx))
		ret = vpu_snapshot(ctx);

	return ret;
}

static int suspend_core(struct core_device *core)
{
	int i;
	int ret = 0;

	WARN_ON(!core);

	core->snapshot = false;

	if (!core->fw_is_ready)
		return 0;

	for (i = 0; i < VPU_MAX_NUM_STREAMS; i++) {
		ret = suspend_instance(core->ctx[i]);
		if (ret)
			return ret;
	}

	for (i = 0; i < VPU_MAX_NUM_STREAMS; i++)
		vpu_ctx_power_off(core->ctx[i]);

	core->suspend = true;

	return 0;
}

static int resume_core(struct core_device *core)
{
	int ret = 0;
	int i;

	WARN_ON(!core);

	if (!core->suspend)
		return 0;

	for (i = 0; i < VPU_MAX_NUM_STREAMS; i++)
		vpu_ctx_power_on(core->ctx[i]);

	/* if the core isn't activated, it means it has been power off and on */
	if (!is_core_activated(core)) {
		if (!core->vdev->hw_enable)
			vpu_enable_hw(core->vdev);
		if (core->snapshot)
			ret = resume_from_snapshot(core);
		else
			reset_vpu_core_dev(core);
	} else {
		if (core->snapshot)
			ret = sw_reset_firmware(core);
	}

	core->snapshot = false;
	core->suspend = false;

	return ret;
}

static int vpu_enc_suspend(struct device *dev)
{
	struct vpu_dev *vpudev = (struct vpu_dev *)dev_get_drvdata(dev);
	int i;
	int ret = 0;

	vpu_dbg(LVL_INFO, "suspend\n");

	mutex_lock(&vpudev->dev_mutex);
	pm_runtime_get_sync(dev);
	for (i = 0; i < vpudev->core_num; i++) {
		ret = suspend_core(&vpudev->core_dev[i]);
		if (ret)
			break;
	}
	pm_runtime_put_sync(dev);
	mutex_unlock(&vpudev->dev_mutex);

	vpu_dbg(LVL_INFO, "suspend done\n");

	return ret;
}

static int vpu_enc_resume(struct device *dev)
{
	struct vpu_dev *vpudev = (struct vpu_dev *)dev_get_drvdata(dev);
	int i;
	int ret = 0;

	vpu_dbg(LVL_INFO, "resume\n");

	mutex_lock(&vpudev->dev_mutex);
	pm_runtime_get_sync(dev);
	vpudev->hw_enable = false;
	for (i = 0; i < vpudev->core_num; i++) {
		ret = resume_core(&vpudev->core_dev[i]);
		if (ret)
			break;
	}
	vpudev->hw_enable = true;
	pm_runtime_put_sync(dev);
	mutex_unlock(&vpudev->dev_mutex);

	vpu_dbg(LVL_INFO, "resume done\n");

	return ret;
}

static const struct dev_pm_ops vpu_pm_ops = {
	SET_RUNTIME_PM_OPS(vpu_runtime_suspend, vpu_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(vpu_enc_suspend, vpu_enc_resume)
};

static const struct of_device_id vpu_of_match[] = {
	{ .compatible = "nxp,imx8qm-b0-vpuenc", },
	{ .compatible = "nxp,imx8qxp-b0-vpuenc", },
	{}
}
MODULE_DEVICE_TABLE(of, vpu_of_match);

static struct platform_driver vpu_driver = {
	.probe = vpu_probe,
	.remove = vpu_remove,
	.driver = {
		.name = "vpu-b0-encoder",
		.of_match_table = vpu_of_match,
		.pm = &vpu_pm_ops,
	},
};
module_platform_driver(vpu_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Linux VPU driver for Freescale i.MX/MXC");
MODULE_LICENSE("GPL");

module_param(vpu_dbg_level_encoder, int, 0644);
MODULE_PARM_DESC(vpu_dbg_level_encoder, "Debug level (0-2)");

