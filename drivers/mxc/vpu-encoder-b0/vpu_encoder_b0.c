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

static void count_event(struct vpu_statistic *statistic, u32 event)
{
	if (!statistic)
		return;

	if (event < VID_API_ENC_EVENT_RESERVED)
		statistic->event[event]++;
	else
		statistic->event[VID_API_ENC_EVENT_RESERVED]++;

	statistic->current_event = event;
	getrawmonotonic(&statistic->ts_event);
}

static void count_cmd(struct vpu_statistic *statistic, u32 cmdid)
{
	if (!statistic)
		return;

	if (cmdid < GTB_ENC_CMD_RESERVED)
		statistic->cmd[cmdid]++;
	else
		statistic->cmd[GTB_ENC_CMD_RESERVED]++;
	statistic->current_cmd = cmdid;
	getrawmonotonic(&statistic->ts_cmd);
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

static void v4l2_vpu_send_cmd(struct vpu_ctx *ctx, uint32_t idx, uint32_t cmdid, uint32_t cmdnum, uint32_t *local_cmddata);

static void MU_sendMesgToFW(void __iomem *base, MSG_Type type, uint32_t value)
{
	MU_SendMessage(base, 1, value);
	MU_SendMessage(base, 0, type);
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
	mutex_lock(&ctx->instance_mutex);

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

	mutex_unlock(&ctx->instance_mutex);
}

static u32 cpu_phy_to_mu(struct core_device *dev, u32 addr)
{
	return addr - dev->m0_p_fw_space_phy;
}

static int initialize_enc_param(struct vpu_ctx *ctx)
{
	pMEDIAIP_ENC_PARAM param = ctx->enc_param;

	mutex_lock(&ctx->instance_mutex);

	param->eCodecMode = MEDIAIP_ENC_FMT_H264;
	param->tEncMemDesc.uMemPhysAddr = ctx->encoder_mem.phy_addr;
	param->tEncMemDesc.uMemVirtAddr = ctx->encoder_mem.phy_addr;
	param->tEncMemDesc.uMemSize     = ctx->encoder_mem.size;
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

	pEncParam = ctx->enc_param;
	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	ret = check_v4l2_fmt(f);
	if (ret)
		return ret;

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

	return ret;
}

static int v4l2_ioctl_g_parm(struct file *file, void *fh,
				struct v4l2_streamparm *parm)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	pMEDIAIP_ENC_PARAM param = NULL;

	if (!parm || !ctx || !ctx->enc_param)
		return -EINVAL;

	param = ctx->enc_param;

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
	struct v4l2_fract fival;
	int ret;

	if (!parm || !ctx || !ctx->enc_param)
		return -EINVAL;

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
	ctx->enc_param->uFrameRate = fival.denominator / fival.numerator;
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

	ret = precheck_qbuf(q_data, buf);
	if (ret < 0)
		return ret;

	ret = vb2_qbuf(&q_data->vb2_q, buf);
	if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		wake_up_interruptible(&ctx->buffer_wq_output);
	else
		wake_up_interruptible(&ctx->buffer_wq_input);

	return ret;
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

	if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		buf->flags = q_data->vb2_reqs[buf->index].buffer_flags;

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

static int v4l2_ioctl_encoder_cmd(struct file *file,
		void *fh,
		struct v4l2_encoder_cmd *cmd
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	u_int32 uStrIdx = ctx->str_index;
	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	switch (cmd->cmd) {
	case V4L2_ENC_CMD_START:
		break;
	case V4L2_ENC_CMD_STOP:
		ctx->forceStop = true;
		v4l2_vpu_send_cmd(ctx, uStrIdx, GTB_ENC_CMD_STREAM_STOP, 0, NULL);
		wake_up_interruptible(&ctx->buffer_wq_input);
		wake_up_interruptible(&ctx->buffer_wq_output);
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
	ret = vb2_streamon(&q_data->vb2_q,
			i);
	ctx->forceStop = false;
	ctx->firmware_stopped = false;
	return ret;
}

static int v4l2_ioctl_streamoff(struct file *file,
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

	if (test_and_clear_bit(VPU_ENC_STATUS_CONFIGURED, &ctx->status)) {
		if (!ctx->forceStop) {
			ctx->forceStop = true;
			v4l2_vpu_send_cmd(ctx, ctx->str_index, GTB_ENC_CMD_STREAM_STOP, 0, NULL);
			wake_up_interruptible(&ctx->buffer_wq_input);
			wake_up_interruptible(&ctx->buffer_wq_output);
		}

		if (!ctx->firmware_stopped)
			wait_for_completion(&ctx->stop_cmp);
	}
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

static void v4l2_vpu_send_cmd(struct vpu_ctx *ctx, uint32_t idx, uint32_t cmdid, uint32_t cmdnum, uint32_t *local_cmddata)
{

	struct core_device  *dev = ctx->core_dev;

	vpu_log_cmd(cmdid, idx);
	count_cmd(&ctx->statistic, cmdid);

	mutex_lock(&dev->cmd_mutex);
	rpc_send_cmd_buf_encoder(&dev->shared_mem, idx, cmdid, cmdnum, local_cmddata);
	mutex_unlock(&dev->cmd_mutex);
	mb();
	MU_SendMessage(dev->mu_base_virtaddr, 0, COMMAND);
}

static void show_codec_configure(struct vpu_ctx *ctx)
{
	pMEDIAIP_ENC_PARAM param;

	if (!ctx)
		return;

	param = ctx->enc_param;
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

static int configure_codec(struct vpu_ctx *ctx)
{
	pBUFFER_DESCRIPTOR_TYPE pEncStrBuffDesc = NULL;
	pMEDIAIP_ENC_EXPERT_MODE_PARAM pEncExpertModeParam = NULL;

	if (!ctx || !ctx->core_dev)
		return -EINVAL;

	pEncStrBuffDesc = ctx->stream_buffer_desc;
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

	pEncExpertModeParam = ctx->expert_mode_param;
	pEncExpertModeParam->Calib.mem_chunk_phys_addr =
					ctx->encoder_mem.phy_addr;
	pEncExpertModeParam->Calib.mem_chunk_virt_addr =
					ctx->encoder_mem.phy_addr;
	pEncExpertModeParam->Calib.mem_chunk_size = ctx->encoder_mem.size;
	pEncExpertModeParam->Calib.cb_base = ctx->encoder_stream.phy_addr;
	pEncExpertModeParam->Calib.cb_size = ctx->encoder_stream.size;

	show_firmware_version(ctx->core_dev);
	memcpy(&ctx->actual_param, ctx->enc_param, sizeof(ctx->actual_param));
	v4l2_vpu_send_cmd(ctx, ctx->str_index,
			GTB_ENC_CMD_CONFIGURE_CODEC, 0, NULL);
	vpu_dbg(LVL_INFO, "send command GTB_ENC_CMD_CONFIGURE_CODEC\n");

	show_codec_configure(ctx);

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

static void v4l2_transfer_buffer_to_firmware(struct queue_data *This,
					struct vb2_buffer *vb)
{
	struct vpu_ctx *ctx =
		container_of(This, struct vpu_ctx, q_data[V4L2_SRC]);

	mutex_lock(&ctx->instance_mutex);
	if (!test_and_set_bit(VPU_ENC_STATUS_CONFIGURED, &ctx->status)) {
		configure_codec(ctx);
		dump_vb2_data(vb);
	}
	mutex_unlock(&ctx->instance_mutex);
}

static bool update_yuv_addr(struct vpu_ctx *ctx, u_int32 uStrIdx)
{
	bool bGotAFrame = FALSE;

	struct vb2_data_req *p_data_req;
	struct queue_data *This = &ctx->q_data[V4L2_SRC];

	pMEDIAIP_ENC_YUV_BUFFER_DESC pParamYuvBuffDesc;
	u_int32 *pphy_address;

	pParamYuvBuffDesc = ctx->yuv_buffer_desc;

	while (1) {
		if (!wait_event_interruptible_timeout(ctx->buffer_wq_input,
				(!list_empty(&This->drv_q)) || ctx->forceStop,
				msecs_to_jiffies(10))) {
			if (!ctx->forceStop)
				vpu_dbg(LVL_DEBUG, " warn: yuv wait_event_interruptible_timeout wait 10ms timeout\n");
			else
				break;
		}
		else
			break;
	}
	if (ctx->forceStop)
		return false;

	down(&This->drv_q_lock);
	if (!list_empty(&This->drv_q)) {
		p_data_req = list_first_entry(&This->drv_q,
				typeof(*p_data_req), list);

		dump_vb2_data(p_data_req->vb2_buf);

		pphy_address = (u_int32 *)vb2_plane_cookie(p_data_req->vb2_buf, 0);
		pParamYuvBuffDesc->uLumaBase = *pphy_address + p_data_req->vb2_buf->planes[0].data_offset;
		pphy_address = (u_int32 *)vb2_plane_cookie(p_data_req->vb2_buf, 1);
		pParamYuvBuffDesc->uChromaBase = *pphy_address + p_data_req->vb2_buf->planes[1].data_offset;
    /* Not sure what the test should be here for a valid frame return from vb2_plane_cookie */
		if (pParamYuvBuffDesc->uLumaBase != 0)
			bGotAFrame = TRUE;

    /* keeps increasing, so just a frame input count rather than a Frame buffer ID */
		pParamYuvBuffDesc->uFrameID = p_data_req->id;
		if (test_and_clear_bit(VPU_ENC_STATUS_KEY_FRAME, &ctx->status))
			pParamYuvBuffDesc->uKeyFrame = 1;
		else
			pParamYuvBuffDesc->uKeyFrame = 0;
		list_del(&p_data_req->list);
	}
	up(&This->drv_q_lock);
	return bGotAFrame;

}

static void report_stream_done(struct vpu_ctx *ctx,  MEDIAIP_ENC_PIC_INFO *pEncPicInfo)
{
	struct vb2_data_req *p_data_req;
	struct queue_data *This = &ctx->q_data[V4L2_DST];
	u_int32 wptr;
	u_int32 rptr;
	u_int32 start;
	u_int32 end;

	void *data_mapped;
	u_int32 length;
	u_int32 data_length = 0;
	void *rptr_virt;

	pBUFFER_DESCRIPTOR_TYPE pEncStrBuffDesc;

	/* Windsor stream buffer descriptor
	 * pEncStrBuffDesc = &RecCmdData.tEncStreamBufferDesc;
	 *
	 * VPU driver stream buffer descriptor with full address
	 * pVpuEncStrBuffDesc
	 * *
	 * Note the wprt is updated prior to calling this function
	 */
	pEncStrBuffDesc = ctx->stream_buffer_desc;


	wptr = pEncStrBuffDesc->wptr | 0x80000000;
	rptr = pEncStrBuffDesc->rptr | 0x80000000;
	start = pEncStrBuffDesc->start | 0x80000000;
	end = pEncStrBuffDesc->end | 0x80000000;
	rptr_virt = ctx->encoder_stream.virt_addr + rptr - start;

	vpu_dbg(LVL_DEBUG, "report_stream_done eptr=%x, rptr=%x, start=%x, end=%x\n", wptr, rptr, start, end);
	while (1) {
		if (!wait_event_interruptible_timeout(ctx->buffer_wq_output,
				(!list_empty(&This->drv_q)),
				msecs_to_jiffies(10))) {
			if (!ctx->forceStop)
				vpu_dbg(LVL_DEBUG, " warn: stream wait_event_interruptible_timeout wait 10ms timeout\n");
			else
				break;
		}
		else
			break;
	}

	if (!list_empty(&This->drv_q)) {
		down(&This->drv_q_lock);

		vpu_dbg(LVL_DEBUG, "report_stream_done down\n");

		p_data_req = list_first_entry(&This->drv_q, typeof(*p_data_req), list);

		vpu_dbg(LVL_DEBUG, "%s :p_data_req(%p)\n", __func__, p_data_req);
		vpu_dbg(LVL_DEBUG, "%s buf_id %d\n", __func__, p_data_req->vb2_buf->index);

		// Calculate length - the amount of space remaining in output stream buffer
		length = p_data_req->vb2_buf->planes[0].length;
		data_mapped = (void *)vb2_plane_vaddr(p_data_req->vb2_buf, 0);
		if (rptr <= wptr)
			data_length = wptr - rptr;
		else if (rptr > wptr)
			data_length = (end - rptr) + (wptr - start);

	//update the bytesused for the output buffer
	if (data_length >= length)
		p_data_req->vb2_buf->planes[0].bytesused = length;
	else
		p_data_req->vb2_buf->planes[0].bytesused = data_length;
	length = p_data_req->vb2_buf->planes[0].bytesused;

	vpu_dbg(LVL_DEBUG, "%s data_length %d, length %d\n", __func__, data_length, length);
	/* Following calculations determine how much data we can transfer into p_vb2_buf
	 * and then only copy that ammount, so rptr is the actual consumed ammount at the end*/
	if ((wptr == rptr) || (rptr > wptr)) {
		if (end - rptr >= length) {
			memcpy(data_mapped, rptr_virt, length);
			rptr += length;
			if (rptr == end)
				rptr = start;
		} else {
			memcpy(data_mapped, rptr_virt, end-rptr);
			if ((length-(end-rptr)) >= (wptr-start)) {
				memcpy(data_mapped + (end-rptr), ctx->encoder_stream.virt_addr, wptr-start);
				rptr = wptr;
			} else {
				memcpy(data_mapped + (end-rptr), ctx->encoder_stream.virt_addr, length-(end-rptr));
				rptr = start+length-(end-rptr);
			}
		}
	} else {
		if (wptr - rptr >= length) {
			memcpy(data_mapped, rptr_virt, length);
			rptr += length;
		} else {
			memcpy(data_mapped, rptr_virt, wptr - rptr);
			rptr = wptr;
		}
	}

	/* Update VPU stream buffer descriptor and Windsor FW stream buffer descriptors respectively*/
	pEncStrBuffDesc->rptr = rptr;

	list_del(&p_data_req->list);
	up(&This->drv_q_lock);

	if (pEncPicInfo->ePicType == MEDIAIP_ENC_PIC_TYPE_IDR_FRAME || pEncPicInfo->ePicType == MEDIAIP_ENC_PIC_TYPE_I_FRAME)
		p_data_req->buffer_flags = V4L2_BUF_FLAG_KEYFRAME;
	else if (pEncPicInfo->ePicType == MEDIAIP_ENC_PIC_TYPE_P_FRAME)
		p_data_req->buffer_flags = V4L2_BUF_FLAG_PFRAME;
	else if (pEncPicInfo->ePicType == MEDIAIP_ENC_PIC_TYPE_B_FRAME)
		p_data_req->buffer_flags = V4L2_BUF_FLAG_BFRAME;
	//memcpy to vb2 buffer from encpicinfo
	if (p_data_req->vb2_buf->state == VB2_BUF_STATE_ACTIVE)
		vb2_buffer_done(p_data_req->vb2_buf, VB2_BUF_STATE_DONE);
	}
	vpu_dbg(LVL_DEBUG, "report_buffer_done return\n");
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
			vpu_err("****error:encFrame[%d] is dirty\n", i);
			flag = 1;
		}
		ret = check_mem_pattern(ctx->encFrame[i].virt_addr +
					ctx->encFrame[i].size);
		if (ret) {
			vpu_err("****error:encFrame[%d] is out of bounds\n", i);
			flag = 1;
		}
	}

	for (i = 0; i < MEDIAIP_MAX_NUM_WINDSOR_REF_FRAMES; i++) {
		ret = check_mem_pattern(ctx->refFrame[i].virt_addr -
					sizeof(u32));
		if (ret) {
			vpu_err("****error:refFrame[%d] is dirty\n", i);
			flag = 1;
		}
		ret = check_mem_pattern(ctx->refFrame[i].virt_addr +
					ctx->refFrame[i].size);
		if (ret) {
			vpu_err("****error:refFrame[%d] is out of bounds\n", i);
			flag = 1;
		}
	}

	ret = check_mem_pattern(ctx->actFrame.virt_addr - sizeof(u32));
	if (ret) {
		vpu_err("****error: actFrame is dirty\n");
		flag = 1;
	}
	ret = check_mem_pattern(ctx->actFrame.virt_addr + ctx->actFrame.size);
	if (ret) {
		vpu_err("****error:actFrame is out of bounds\n");
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

	ctx->enc_buffer.size = calc_enc_mem_size(req_data);
	ret = alloc_dma_buffer(ctx->dev, &ctx->enc_buffer);
	if (ret) {
		vpu_dbg(LVL_ERR, "alloc encoder buffer fail\n");
		return ret;
	}

	core_dev = ctx->core_dev;
	pEncMemPool = ctx->mem_pool;
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

static void vpu_api_event_handler(struct vpu_ctx *ctx, u_int32 uStrIdx, u_int32 uEvent, u_int32 *event_data)
{
	vpu_log_event(uEvent, uStrIdx);
	count_event(&ctx->statistic, uEvent);
	check_enc_mem_overstep(ctx);
	if (uStrIdx < VID_API_NUM_STREAMS) {
		switch (uEvent) {
		case VID_API_ENC_EVENT_START_DONE: {
		if (update_yuv_addr(ctx, uStrIdx))
			v4l2_vpu_send_cmd(ctx, uStrIdx, GTB_ENC_CMD_FRAME_ENCODE, 0, NULL);
		} break;
		case VID_API_ENC_EVENT_MEM_REQUEST: {
			MEDIAIP_ENC_MEM_REQ_DATA *req_data = (MEDIAIP_ENC_MEM_REQ_DATA *)event_data;
			vpu_dbg(LVL_INFO, "uEncFrmSize = %d, uEncFrmNum=%d, uRefFrmSize=%d, uRefFrmNum=%d, uActBufSize=%d\n", req_data->uEncFrmSize, req_data->uEncFrmNum, req_data->uRefFrmSize, req_data->uRefFrmNum, req_data->uActBufSize);
			enc_mem_alloc(ctx, req_data);
			//update_yuv_addr(ctx,0);
			v4l2_vpu_send_cmd(ctx, uStrIdx, GTB_ENC_CMD_STREAM_START, 0, NULL);
		} break;
		case VID_API_ENC_EVENT_PARA_UPD_DONE:
		break;
		case VID_API_ENC_EVENT_FRAME_DONE: {
		MEDIAIP_ENC_PIC_INFO *pEncPicInfo = (MEDIAIP_ENC_PIC_INFO *)event_data;

		vpu_dbg(LVL_DEBUG, "VID_API_ENC_EVENT_FRAME_DONE pEncPicInfo->uPicEncodDone=%d: Encode picture done\n", pEncPicInfo->uPicEncodDone);
		if (pEncPicInfo->uPicEncodDone) {
#ifdef TB_REC_DBG
		vpu_dbg(LVL_DEBUG, "       - Frame ID      : 0x%x\n", pEncPicInfo->uFrameID);

		vpu_dbg(LVL_DEBUG, "       - Picture Type  : %s\n", pEncPicInfo->ePicType == MEDIAIP_ENC_PIC_TYPE_B_FRAME ? "B picture" :
		pEncPicInfo->ePicType == MEDIAIP_ENC_PIC_TYPE_P_FRAME ? "P picture" :
		pEncPicInfo->ePicType == MEDIAIP_ENC_PIC_TYPE_I_FRAME ? "I picture" :
		pEncPicInfo->ePicType == MEDIAIP_ENC_PIC_TYPE_IDR_FRAME ? "IDR picture" : "BI picture");
		vpu_dbg(LVL_DEBUG, "       - Skipped frame : 0x%x\n", pEncPicInfo->uSkippedFrame);
		vpu_dbg(LVL_DEBUG, "       - Frame size    : 0x%x\n", pEncPicInfo->uFrameSize);
		vpu_dbg(LVL_DEBUG, "       - Frame CRC     : 0x%x\n", pEncPicInfo->uFrameCrc);
#endif

		/* Sync the write pointer to the local view of it */

		report_stream_done(ctx, pEncPicInfo);
		}
		} break;
		case VID_API_ENC_EVENT_FRAME_RELEASE: {
		struct queue_data *This = &ctx->q_data[V4L2_SRC];
		u_int32 *uFrameID = (u_int32 *)event_data;
		struct vb2_data_req *p_data_req;

		vpu_dbg(LVL_DEBUG, "VID_API_ENC_EVENT_FRAME_RELEASE : Frame release - uFrameID = 0x%x\n", *uFrameID);
		p_data_req = &This->vb2_reqs[*uFrameID];
		if (p_data_req->vb2_buf->state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(p_data_req->vb2_buf,
					VB2_BUF_STATE_DONE
				);

		} break;
		case VID_API_ENC_EVENT_STOP_DONE: {
		const struct v4l2_event ev = {
			.type = V4L2_EVENT_EOS
		};
		ctx->firmware_stopped = true;
		complete_all(&ctx->stop_cmp);
		v4l2_event_queue_fh(&ctx->fh, &ev);
		}
		break;
		case VID_API_ENC_EVENT_FRAME_INPUT_DONE: {
		if (update_yuv_addr(ctx, uStrIdx))
			v4l2_vpu_send_cmd(ctx, uStrIdx, GTB_ENC_CMD_FRAME_ENCODE, 0, NULL);
		} break;
		case VID_API_ENC_EVENT_TERMINATE_DONE:
		break;
		default:
		vpu_dbg(LVL_ERR, "........unknown event : 0x%x\n", uEvent);
		break;
		}
	}
}

static void enable_mu(struct core_device *dev)
{
	MU_sendMesgToFW(dev->mu_base_virtaddr,
			PRINT_BUF_OFFSET,
			cpu_phy_to_mu(dev,  dev->m0_rpc_phy + M0_PRINT_OFFSET));
	MU_sendMesgToFW(dev->mu_base_virtaddr,
			RPC_BUF_OFFSET,
			cpu_phy_to_mu(dev, dev->m0_rpc_phy));
	MU_sendMesgToFW(dev->mu_base_virtaddr,
			BOOT_ADDRESS,
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
		vpu_api_event_handler(ctx, msg.idx, msg.msgid, msg.msgdata);
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

	if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		v4l2_transfer_buffer_to_firmware(This, vb);
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

static int set_vpu_fw_addr(struct vpu_dev *dev, struct core_device *core_dev)
{
	if (!dev || !core_dev)
		return -EINVAL;

	write_enc_reg(dev, core_dev->m0_p_fw_space_phy, core_dev->reg_fw_base);
	write_enc_reg(dev, 0x0, core_dev->reg_fw_base + 4);

	return 0;
}

static int vpu_firmware_download(struct vpu_dev *This, u_int32 core_id)
{
	unsigned char *image;
	unsigned int FW_Size = 0;
	int ret = 0;
	char *p = This->core_dev[core_id].m0_p_fw_space_vir;

	ret = request_firmware((const struct firmware **)&This->m0_pfw,
			M0FW_FILENAME,
			This->generic_dev
			);
	if (ret) {
		vpu_dbg(LVL_ERR, "%s() request fw %s failed(%d)\n",
			__func__, M0FW_FILENAME, ret);

		if (This->m0_pfw) {
			release_firmware(This->m0_pfw);
			This->m0_pfw = NULL;
		}
		return ret;
	} else {
		vpu_dbg(LVL_DEBUG, "%s() request fw %s got size(%d)\n",
			__func__, M0FW_FILENAME, (int)This->m0_pfw->size);
		image = (uint8_t *)This->m0_pfw->data;
		FW_Size = This->m0_pfw->size;
	}
	memcpy(This->core_dev[core_id].m0_p_fw_space_vir,
			image,
			FW_Size
			);
	p[16] = This->plat_type;
	p[17] = core_id + 1;
	set_vpu_fw_addr(This, &This->core_dev[core_id]);

	return ret;
}

static int do_download_vpu_firmware(struct vpu_dev *dev,
				struct core_device *core_dev)
{
	int ret;

	if (!dev || !core_dev)
		return -EINVAL;

	if (core_dev->fw_is_ready)
		return 0;

	ret = vpu_firmware_download(dev, core_dev->id);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: vpu_firmware_download fail\n");
		return ret;
	}
	init_completion(&core_dev->start_cmp);
	wait_for_completion_timeout(&core_dev->start_cmp,
					msecs_to_jiffies(100));
	if (!core_dev->firmware_started) {
		vpu_dbg(LVL_ERR, "start firmware failed\n");
		return -EINVAL;
	}

	core_dev->fw_is_ready = true;

	return 0;
}

static int download_vpu_firmware(struct vpu_dev *dev,
				struct core_device *core_dev)
{
	int ret;

	if (!dev || !core_dev)
		return -EINVAL;
	mutex_lock(&dev->dev_mutex);
	ret = do_download_vpu_firmware(dev, core_dev);
	mutex_unlock(&dev->dev_mutex);

	return ret;
}

static void free_instance(struct vpu_ctx *ctx)
{
	if (!ctx)
		return;

	if (ctx->dev && ctx->core_dev && ctx->str_index < VPU_MAX_NUM_STREAMS) {
		mutex_lock(&ctx->dev->dev_mutex);
		ctx->core_dev->ctx[ctx->str_index] = NULL;
		mutex_unlock(&ctx->dev->dev_mutex);
	}
	kfree(ctx);
}

static int request_instanct(struct vpu_dev *dev, struct vpu_ctx *ctx)
{
	int idx;
	int i;
	int found = 0;

	if (!dev || !ctx)
		return -EINVAL;

	mutex_lock(&dev->dev_mutex);
	for (idx = 0; idx < VPU_MAX_NUM_STREAMS; idx++) {
		for (i = 0; i < dev->core_num; i++) {
			if (!dev->core_dev[i].ctx[idx]) {
				found = 1;
				ctx->core_dev = &dev->core_dev[i];
				ctx->str_index = idx;
				ctx->dev = dev;
				dev->core_dev[i].ctx[idx] = ctx;
				break;
			}
		}
		if (found)
			break;
	}
	mutex_unlock(&dev->dev_mutex);

	if (!found) {
		vpu_dbg(LVL_ERR, "cann't request any instance\n");
		return -EBUSY;
	}

	return 0;
}

static int construct_vpu_ctx(struct vpu_ctx *ctx)
{
	struct shared_addr *shared_mem = NULL;
	int idx;

	if (!ctx)
		return -EINVAL;

	ctx->ctrl_inited = false;
	init_completion(&ctx->completion);
	init_completion(&ctx->stop_cmp);
	mutex_init(&ctx->instance_mutex);
	ctx->forceStop = false;
	ctx->firmware_stopped = false;
	ctx->ctx_released = false;
	init_waitqueue_head(&ctx->buffer_wq_output);
	init_waitqueue_head(&ctx->buffer_wq_input);

	shared_mem = &ctx->core_dev->shared_mem;
	idx = ctx->str_index;
	ctx->yuv_buffer_desc = rpc_get_yuv_buffer_desc(shared_mem, idx);
	ctx->stream_buffer_desc = rpc_get_stream_buffer_desc(shared_mem, idx);
	ctx->expert_mode_param = rpc_get_expert_mode_param(shared_mem, idx);
	ctx->enc_param = rpc_get_enc_param(shared_mem, idx);
	ctx->mem_pool = rpc_get_mem_pool(shared_mem, idx);
	ctx->encoding_status = rpc_get_encoding_status(shared_mem, idx);
	ctx->dsa_status = rpc_get_dsa_status(shared_mem, idx);

	return 0;
}

static struct vpu_ctx *create_and_request_instance(struct vpu_dev *dev)
{
	struct vpu_ctx *ctx = NULL;
	int ret;

	if (!dev)
		return NULL;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return NULL;

	ret = request_instanct(dev, ctx);
	if (ret < 0) {
		kfree(ctx);
		return NULL;
	}

	construct_vpu_ctx(ctx);

	return ctx;
}

static int init_vpu_ctx_fh(struct vpu_ctx *ctx, struct vpu_dev *dev)
{
	if (!ctx || !dev)
		return -EINVAL;

	v4l2_fh_init(&ctx->fh, dev->pvpu_encoder_dev);
	v4l2_fh_add(&ctx->fh);

	ctx->fh.ctrl_handler = &ctx->ctrl_handler;

	return 0;
}

static void uninit_vpu_ctx_fh(struct vpu_ctx *ctx)
{
	if (!ctx)
		return;

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
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
	if (ctx->encoder_stream.virt_addr) {
		dma_free_coherent(ctx->dev->generic_dev,
				ctx->encoder_stream.size,
				ctx->encoder_stream.virt_addr,
				ctx->encoder_stream.phy_addr);
		ctx->encoder_stream.size = 0;
		ctx->encoder_stream.virt_addr = NULL;
		ctx->encoder_stream.phy_addr = 0;
	}

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

	ctx->encoder_stream.size = STREAM_SIZE;
	ret = alloc_dma_buffer(ctx->dev, &ctx->encoder_stream);
	if (ret) {
		vpu_dbg(LVL_ERR, "alloc encoder stream buffer fail\n");
		goto error;
	}

	return 0;
error:
	uninit_vpu_ctx(ctx);
	return ret;
}

static ssize_t show_instance_info(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vpu_ctx *ctx;
	struct vpu_statistic *statistic;
	pMEDIAIP_ENC_PARAM param;
	int i;
	int num = 0;
	int size;
	char *fw;

	ctx = container_of(attr, struct vpu_ctx, dev_attr_instance);
	statistic = &ctx->statistic;
	param = ctx->enc_param;
	fw = ctx->core_dev->m0_p_fw_space_vir;

	num += snprintf(buf + num, PAGE_SIZE, "pid:%d\n", current->pid);
	num += snprintf(buf + num, PAGE_SIZE, "fw:%d, %d\n", fw[16], fw[17]);

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

	num += snprintf(buf + num, PAGE_SIZE - num, "encoder param:\n");
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Codec Mode",
			ctx->actual_param.eCodecMode, param->eCodecMode);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Profile",
			ctx->actual_param.eProfile, param->eProfile);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Level",
			ctx->actual_param.uLevel, param->uLevel);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Frame Rate",
			ctx->actual_param.uFrameRate, param->uFrameRate);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Source Stride",
			ctx->actual_param.uSrcStride, param->uSrcStride);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Source Width",
			ctx->actual_param.uSrcWidth, param->uSrcWidth);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Source Height",
			ctx->actual_param.uSrcHeight, param->uSrcHeight);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Source Offset x",
			ctx->actual_param.uSrcOffset_x, param->uSrcOffset_x);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Source Offset y",
			ctx->actual_param.uSrcOffset_y, param->uSrcOffset_y);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Source Crop Width",
			ctx->actual_param.uSrcCropWidth, param->uSrcCropWidth);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Source Crop Height",
			ctx->actual_param.uSrcCropHeight,
			param->uSrcCropHeight);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Out Width",
			ctx->actual_param.uOutWidth, param->uOutWidth);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Out Height",
			ctx->actual_param.uOutHeight, param->uOutHeight);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "I Frame Interval",
			ctx->actual_param.uIFrameInterval,
			param->uIFrameInterval);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "GOP Length",
			ctx->actual_param.uGopBLength, param->uGopBLength);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Low Latency Mode",
			ctx->actual_param.uLowLatencyMode,
			param->uLowLatencyMode);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Bitrate Mode",
			ctx->actual_param.eBitRateMode, param->eBitRateMode);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Target Bitrate",
			ctx->actual_param.uTargetBitrate,
			param->uTargetBitrate);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Min Bitrate",
			ctx->actual_param.uMinBitRate, param->uMinBitRate);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "Max Bitrate",
			ctx->actual_param.uMaxBitRate, param->uMaxBitRate);
	num += snprintf(buf + num, PAGE_SIZE - num,
			"\t%40s:%10d;%10d\n", "QP",
			ctx->actual_param.uInitSliceQP, param->uInitSliceQP);

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

	return num;
}

static int create_instance_file(struct vpu_ctx *ctx)
{
	if (!ctx || !ctx->dev || !ctx->dev->generic_dev || !ctx->core_dev)
		return -EINVAL;

	snprintf(ctx->name, sizeof(ctx->name) - 1, "instance.%d.%d",
			ctx->core_dev->id, ctx->str_index);
	ctx->dev_attr_instance.attr.name = ctx->name;
	ctx->dev_attr_instance.attr.mode = VERIFY_OCTAL_PERMISSIONS(0444);
	ctx->dev_attr_instance.show = show_instance_info;

	device_create_file(ctx->dev->generic_dev, &ctx->dev_attr_instance);

	return 0;
}

static int remove_instance_file(struct vpu_ctx *ctx)
{
	if (!ctx || !ctx->dev || !ctx->dev->generic_dev)
		return -EINVAL;

	device_remove_file(ctx->dev->generic_dev, &ctx->dev_attr_instance);

	return 0;
}

static int v4l2_open(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct vpu_dev *dev = video_get_drvdata(vdev);
	struct vpu_ctx *ctx = NULL;
	int ret;

	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	ctx = create_and_request_instance(dev);
	if (!ctx) {
		vpu_dbg(LVL_ERR, "failed to create encoder ctx\n");
		return -ENOMEM;
	}

	pm_runtime_get_sync(dev->generic_dev);

	ret = init_vpu_ctx(ctx);
	if (ret) {
		vpu_dbg(LVL_ERR, "init vpu ctx fail\n");
		goto err_alloc;
	}

	init_queue_data(ctx);
	init_vpu_ctx_fh(ctx, dev);
	vpu_enc_setup_ctrls(ctx);

	ret = download_vpu_firmware(dev, ctx->core_dev);
	if (ret)
		goto err_firmware_load;

	create_instance_file(ctx);
	initialize_enc_param(ctx);

	filp->private_data = &ctx->fh;

	return 0;

err_firmware_load:
	vpu_enc_free_ctrls(ctx);
	uninit_vpu_ctx_fh(ctx);
	release_queue_data(ctx);
	uninit_vpu_ctx(ctx);

err_alloc:
	pm_runtime_put_sync(dev->generic_dev);
	free_instance(ctx);
	return ret;
}

static int v4l2_release(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct vpu_dev *dev = video_get_drvdata(vdev);
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(filp->private_data);

	vpu_dbg(LVL_DEBUG, "%s()\n", __func__);

	if (!ctx->forceStop &&
		test_bit(VPU_ENC_STATUS_CONFIGURED, &ctx->status)) {
		//need send stop if app call release without calling of V4L2_ENC_CMD_STOP
		ctx->forceStop = true;
		v4l2_vpu_send_cmd(ctx, ctx->str_index, GTB_ENC_CMD_STREAM_STOP, 0, NULL);
		wake_up_interruptible(&ctx->buffer_wq_input);
		wake_up_interruptible(&ctx->buffer_wq_output);
	}

	if (!ctx->firmware_stopped &&
		test_bit(VPU_ENC_STATUS_CONFIGURED, &ctx->status))
		wait_for_completion(&ctx->stop_cmp);

	uninit_vpu_ctx(ctx);
	remove_instance_file(ctx);
	vpu_enc_free_ctrls(ctx);
	uninit_vpu_ctx_fh(ctx);
	release_queue_data(ctx);
	enc_mem_free(ctx);

	pm_runtime_put_sync(dev->generic_dev);
	free_instance(ctx);

	return 0;
}

static unsigned int v4l2_poll(struct file *filp, poll_table *wait)
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

static int v4l2_mmap(struct file *filp, struct vm_area_struct *vma)
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
	.open  = v4l2_open,
	.unlocked_ioctl = video_ioctl2,
	.release = v4l2_release,
	.poll = v4l2_poll,
	.mmap = v4l2_mmap,
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

	return 0;
}

static void vpu_disable_hw(struct vpu_dev *This)
{
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

static int parse_dt_info(struct vpu_dev *dev, struct device_node *np)
{
	int ret;
	struct device_node *reserved_node = NULL;
	struct resource reserved_res;
	u_int32 core_type;
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

	if (of_address_to_resource(reserved_node, 0, &reserved_res)) {
		vpu_dbg(LVL_ERR,
			"error: boot-region of_address_to_resource error\n");
		return -EINVAL;
	}
	dev->core_dev[0].m0_p_fw_space_phy = reserved_res.start;
	dev->core_dev[1].m0_p_fw_space_phy = reserved_res.start + M0_BOOT_SIZE;
	reserved_node = of_parse_phandle(np, "rpc-region", 0);
	if (!reserved_node) {
		vpu_dbg(LVL_ERR,
			"error: rpc-region of_parse_phandle error\n");
		return -ENODEV;
	}

	if (of_address_to_resource(reserved_node, 0, &reserved_res)) {
		vpu_dbg(LVL_ERR,
			"error: rpc-region of_address_to_resource error\n");
		return -EINVAL;
	}
	dev->core_dev[0].m0_rpc_phy = reserved_res.start;
	dev->core_dev[1].m0_rpc_phy = reserved_res.start + SHARED_SIZE;

	for (i = 0; i < dev->core_num; i++) {
		u32 val;

		ret = of_property_read_u32_index(np, "reg-fw-base", i, &val);
		if (ret) {
			vpu_dbg(LVL_ERR,
				"find reg-fw-base for core[%d] fail\n", i);
			return ret;
		}
		dev->core_dev[i].reg_fw_base = val;
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

static int reset_vpu_core_dev(struct core_device *core_dev)
{
	if (!core_dev)
		return -EINVAL;

	core_dev->fw_is_ready = false;
	core_dev->firmware_started = false;
	rpc_init_shared_memory_encoder(&core_dev->shared_mem,
				cpu_phy_to_mu(core_dev, core_dev->m0_rpc_phy),
				core_dev->m0_rpc_virt, SHARED_SIZE);
	rpc_set_system_cfg_value_encoder(core_dev->shared_mem.pSharedInterface,
				VPU_REG_BASE, core_dev->id);
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
		ioremap_wc(core_dev->m0_p_fw_space_phy, M0_BOOT_SIZE);
	if (!core_dev->m0_p_fw_space_vir)
		vpu_dbg(LVL_ERR, "failed to remap space for M0 firmware\n");

	memset_io(core_dev->m0_p_fw_space_vir, 0, M0_BOOT_SIZE);

	core_dev->m0_rpc_virt = ioremap_wc(core_dev->m0_rpc_phy, SHARED_SIZE);
	if (!core_dev->m0_rpc_virt)
		vpu_dbg(LVL_ERR, "failed to remap space for shared memory\n");

	memset_io(core_dev->m0_rpc_virt, 0, SHARED_SIZE);

	reset_vpu_core_dev(core_dev);

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

	vpu_enable_hw(dev);

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	mutex_init(&dev->dev_mutex);
	for (i = 0; i < dev->core_num; i++) {
		dev->core_dev[i].id = i;
		dev->core_dev[i].generic_dev = get_device(dev->generic_dev);
		dev->core_dev[i].vdev = dev;
		ret = init_vpu_core_dev(&dev->core_dev[i]);
		if (ret)
			goto error_init_core;
	}
	pm_runtime_put_sync(&pdev->dev);

	return 0;

error_init_core:
	for (i = 0; i < dev->core_num; i++)
		uninit_vpu_core_dev(&dev->core_dev[i]);

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

	for (i = 0; i < dev->core_num; i++)
		uninit_vpu_core_dev(&dev->core_dev[i]);

	if (dev->m0_pfw) {
		release_firmware(dev->m0_pfw);
		dev->m0_pfw = NULL;
	}
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

static struct vpu_ctx *first_available_instance(struct core_device *core_dev)
{
	int idx;

	for (idx = 0; idx < VPU_MAX_NUM_STREAMS; idx++) {
		if (!core_dev->ctx[idx])
			continue;
		if (!test_bit(VPU_ENC_STATUS_HANG, &core_dev->ctx[idx]->status))
			return core_dev->ctx[idx];
	}

	return NULL;
}

static void v4l2_vpu_send_snapshot(struct core_device *dev)
{
	struct vpu_ctx *ctx;

	/*figure out the first available instance*/
	ctx = first_available_instance(dev);

	if (!ctx)
		return;

	v4l2_vpu_send_cmd(ctx, ctx->str_index, GTB_ENC_CMD_SNAPSHOT, 0, NULL);
}

static int vpu_snapshot(struct core_device *core_dev)
{
	int ret;

	if (!first_available_instance(core_dev))
		return 0;
	/*if there is an available device, send snapshot command to firmware*/
	v4l2_vpu_send_snapshot(core_dev);

	ret = wait_for_completion_timeout(&core_dev->snap_done_cmp,
					msecs_to_jiffies(1000));
	if (!ret) {
		vpu_dbg(LVL_ERR,
			"error:wait for vpu encoder snapdone event timeout!\n");
		return -EINVAL;
	}

	return 0;
}

static int vpu_suspend(struct device *dev)
{
	struct vpu_dev *vpudev = (struct vpu_dev *)dev_get_drvdata(dev);
	u_int32 i;
	int ret;

	for (i = 0; i < vpudev->core_num; i++) {
		ret = vpu_snapshot(&vpudev->core_dev[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static int vpu_resume(struct device *dev)
{
	struct vpu_dev *vpudev = (struct vpu_dev *)dev_get_drvdata(dev);
	struct core_device *core_dev;
	u_int32 i;

	vpu_enable_hw(vpudev);

	for (i = 0; i < vpudev->core_num; i++) {
		core_dev = &vpudev->core_dev[i];
		MU_Init(core_dev->mu_base_virtaddr);
		MU_EnableRxFullInt(core_dev->mu_base_virtaddr, 0);

		if (!first_available_instance(core_dev)) {
			/*no instance is active before suspend, do reset*/
			reset_vpu_core_dev(core_dev);
		} else {
			/*resume*/
			set_vpu_fw_addr(vpudev, core_dev);
			/*wait for firmware resotre done*/
			if (!wait_for_completion_timeout(&core_dev->start_cmp, msecs_to_jiffies(1000))) {
				vpu_dbg(LVL_ERR, "error: wait for vpu encoder resume done timeout!\n");
				return -1;
			}
		}
	}
	return 0;
}

static const struct dev_pm_ops vpu_pm_ops = {
	SET_RUNTIME_PM_OPS(vpu_runtime_suspend, vpu_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(vpu_suspend, vpu_resume)
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

