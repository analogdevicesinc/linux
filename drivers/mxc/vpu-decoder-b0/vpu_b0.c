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
 * @file vpu-b0.c
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
#include <media/videobuf2-dma-sg.h>

#include "vpu_b0.h"

unsigned int vpu_dbg_level_decoder = 1;

static void vpu_api_event_handler(struct vpu_ctx *ctx, u_int32 uStrIdx, u_int32 uEvent, u_int32 *event_data);
static void v4l2_vpu_send_cmd(struct vpu_ctx *ctx, uint32_t idx, uint32_t cmdid, uint32_t cmdnum, uint32_t *local_cmddata);
static void add_eos(struct vpu_ctx *ctx, u_int32 uStrBufIdx);
static void v4l2_update_stream_addr(struct vpu_ctx *ctx, uint32_t uStrBufIdx);
static int reset_vpu_firmware(struct vpu_dev *dev);

static char *cmd2str[] = {
	"VID_API_CMD_NULL",   /*0x0*/
	"VID_API_CMD_PARSE_NEXT_SEQ", /*0x1*/
	"VID_API_CMD_PARSE_NEXT_I",
	"VID_API_CMD_PARSE_NEXT_IP",
	"VID_API_CMD_PARSE_NEXT_ANY",
	"VID_API_CMD_DEC_PIC",
	"VID_API_CMD_UPDATE_ES_WR_PTR",
	"VID_API_CMD_UPDATE_ES_RD_PTR",
	"VID_API_CMD_UPDATE_UDATA",
	"VID_API_CMD_GET_FSINFO",
	"VID_API_CMD_SKIP_PIC",
	"VID_API_CMD_DEC_CHUNK",  /*0x0b*/
	"VID_API_CMD_UNDEFINED",
	"VID_API_CMD_UNDEFINED",
	"VID_API_CMD_UNDEFINED",
	"VID_API_CMD_UNDEFINED",
	"VID_API_CMD_START",         /*0x10*/
	"VID_API_CMD_STOP",
	"VID_API_CMD_ABORT",
	"VID_API_CMD_RST_BUF",
	"VID_API_CMD_UNDEFINED",
	"VID_API_CMD_FS_RELEASE",
	"VID_API_CMD_MEM_REGION_ATTACH",
	"VID_API_CMD_MEM_REGION_DETACH",
	"VID_API_CMD_MVC_VIEW_SELECT",
	"VID_API_CMD_FS_ALLOC",   /*0x19*/
	"VID_API_CMD_UNDEFINED",
	"VID_API_CMD_UNDEFINED",
	"VID_API_CMD_DBG_GET_STATUS", /*0x1C*/
	"VID_API_CMD_DBG_START_LOG",
	"VID_API_CMD_DBG_STOP_LOG",
	"VID_API_CMD_DBG_DUMP_LOG",
	"VID_API_CMD_YUV_READY",   /*0x20*/
};

static char *event2str[] = {
	"VID_API_EVENT_NULL",  /*0x0*/
	"VID_API_EVENT_RESET_DONE",  /*0x1*/
	"VID_API_EVENT_SEQ_HDR_FOUND",
	"VID_API_EVENT_PIC_HDR_FOUND",
	"VID_API_EVENT_PIC_DECODED",
	"VID_API_EVENT_FIFO_LOW",
	"VID_API_EVENT_FIFO_HIGH",
	"VID_API_EVENT_FIFO_EMPTY",
	"VID_API_EVENT_FIFO_FULL",
	"VID_API_EVENT_BS_ERROR",
	"VID_API_EVENT_UDATA_FIFO_UPTD",
	"VID_API_EVENT_RES_CHANGE",
	"VID_API_EVENT_FIFO_OVF",
	"VID_API_EVENT_CHUNK_DECODED",  /*0x0D*/
	"VID_API_EVENT_UNDEFINED",
	"VID_API_EVENT_UNDEFINED",
	"VID_API_EVENT_REQ_FRAME_BUFF",  /*0x10*/
	"VID_API_EVENT_FRAME_BUFF_RDY",
	"VID_API_EVENT_REL_FRAME_BUFF",
	"VID_API_EVENT_STR_BUF_RST",
	"VID_API_EVENT_RET_PING",
	"VID_API_EVENT_QMETER",
	"VID_API_EVENT_STR_FMT_CHANGED",
	"VID_API_EVENT_MIPS_XCPT",
	"VID_API_EVENT_START_DONE",
	"VID_API_EVENT_STOPPED",
	"VID_API_EVENT_ABORT_DONE",
	"VID_API_EVENT_FINISHED",
	"VID_API_EVENT_DBG_STAT_UPDATE",
	"VID_API_EVENT_DBG_LOG_STARTED",
	"VID_API_EVENT_DBG_LOG_STOPPED",
	"VID_API_EVENT_DBG_LOG_UPFATED",
	"VID_API_EVENT_DBG_MSG_DEC",  /*0x20*/
	"VID_API_EVENT_DEC_SC_ERR",
	"VID_API_EVENT_CQ_FIFO_DUMP",
	"VID_API_EVENT_DBG_FIFO_DUMP",
	"VID_API_EVENT_DEC_CHECK_RES",
	"VID_API_EVENT_DEC_CFG_INFO",  /*0x25*/
};

static char *bufstat[] = {
	"FRAME_ALLOC",
	"FRAME_FREE",
	"FRAME_DECODED",
	"FRAME_READY",
	"FRAME_RELEASE",
};

static void vpu_log_event(u_int32 uEvent, u_int32 ctxid)
{
	if (uEvent > ARRAY_SIZE(event2str)-1)
		vpu_dbg(LVL_INFO, "reveive event: 0x%X, ctx id:%d\n", uEvent, ctxid);
	else
		vpu_dbg(LVL_INFO, "recevie event: %s, ctx id:%d\n", event2str[uEvent], ctxid);
}

static void vpu_log_cmd(u_int32 cmdid, u_int32 ctxid)
{
	if (cmdid > ARRAY_SIZE(cmd2str)-1)
		vpu_dbg(LVL_INFO, "send cmd: 0x%X, ctx id:%d\n", cmdid, ctxid);
	else
		vpu_dbg(LVL_INFO, "send cmd: %s ctx id:%d\n", cmd2str[cmdid], ctxid);
}
#ifdef DEBUG
static void vpu_log_stat(u_int32 status, u_int32 bufferid, u_int32 ctxid)
{
	if (status > sizeof(bufstat)-1)
		vpu_dbg(LVL_INFO, "buffer status: 0x%X, buffer id:%d ctx id:%d\n", status, bufferid, ctxid);
	else
		vpu_dbg(LVL_INFO, "buffer status: %s, buffer id:%d ctx id:%d\n", bufstat[status], bufferid, ctxid);
}
#endif
static int find_buffer_id(struct vpu_ctx *ctx, u_int32 addr)
{
	struct vb2_data_req *p_data_req;
	u_int32 LumaAddr;
	u_int32 *pphy_address;
	u_int32 i;

	for (i = 0; i < VPU_MAX_BUFFER; i++) {
		p_data_req = &ctx->q_data[V4L2_DST].vb2_reqs[i];
		if (p_data_req->vb2_buf != NULL) {
			pphy_address = (u_int32 *)vb2_plane_cookie(p_data_req->vb2_buf, 0);
			if (pphy_address != NULL) {
				LumaAddr = *pphy_address;
				if (LumaAddr == addr - ctx->dev->cm_offset)
					return i;
			} else
				vpu_dbg(LVL_ERR, "error: %s() buffer (%d) is NULL\n", __func__, i);
		}
	}

	vpu_dbg(LVL_ERR, "error: %s() can't find suitable id based on address(0x%x)\n", __func__, addr);
	return -1;
}

static void MU_sendMesgToFW(void __iomem *base, MSG_Type type, uint32_t value)
{
	MU_SendMessage(base, 1, value);
	MU_SendMessage(base, 0, type);
}
#ifdef DEBUG
static void vpu_log_shared_mem(struct vpu_ctx *ctx)
{
	struct vpu_dev *dev = ctx->dev;
	struct shared_addr *This = &dev->shared_mem;
	pDEC_RPC_HOST_IFACE pSharedInterface = (pDEC_RPC_HOST_IFACE)This->shared_mem_vir;
	MediaIPFW_Video_BufDesc *pMsgDesc = &pSharedInterface->StreamMsgBufferDesc;
	MediaIPFW_Video_BufDesc *pCmdDesc = &pSharedInterface->StreamCmdBufferDesc;
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
	u_int32 index = ctx->str_index;

	vpu_dbg(LVL_INFO, "msg: wr: 0x%x, rd: 0x%x, cmd: wr : 0x%x, rd: 0x%x\n",
			pMsgDesc->uWrPtr, pMsgDesc->uRdPtr, pCmdDesc->uWrPtr, pCmdDesc->uRdPtr);

	pStrBufDesc = dev->regs_base + DEC_MFD_XREG_SLV_BASE + MFD_MCX + MFD_MCX_OFF * index;
	vpu_dbg(LVL_INFO, "data: wptr(0x%x) rptr(0x%x) start(0x%x) end(0x%x) uStrIdx(%d)\n",
			pStrBufDesc->wptr, pStrBufDesc->rptr, pStrBufDesc->start, pStrBufDesc->end, index);
}
#endif
/*
 * v4l2 ioctl() operation
 *
 */
static struct vpu_v4l2_fmt  formats_compressed_dec[] = {
	{
		.name       = "H264 Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_H264,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_AVC,
	},
	{
		.name       = "VC1 Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_VC1_ANNEX_G,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_VC1,
	},
	{
		.name       = "VC1 RCV Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_VC1_ANNEX_L,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_VC1,
	},
	{
		.name       = "MPEG2 Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_MPEG2,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_MPEG2,
	},

	{
		.name       = "AVS Encoded Stream",
		.fourcc     = VPU_PIX_FMT_AVS,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_AVS,
	},
	{
		.name       = "MPEG4 ASP Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_MPEG4,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_ASP,
	},
	{
		.name       = "JPEG stills",
		.fourcc     = V4L2_PIX_FMT_JPEG,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_JPEG,
	},
	{
		.name       = "RV8 Encoded Stream",
		.fourcc     = VPU_PIX_FMT_RV8,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_RV8,
	},
	{
		.name       = "RV9 Encoded Stream",
		.fourcc     = VPU_PIX_FMT_RV9,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_RV9,
	},
	{
		.name       = "VP6 Encoded Stream",
		.fourcc     = VPU_PIX_FMT_VP6,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_VP6,
	},
	{
		.name       = "VP6 SPK Encoded Stream",
		.fourcc     = VPU_PIX_FMT_SPK,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_SPK,
	},
	{
		.name       = "VP8 Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_VP8,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_VP8,
	},
	{
		.name       = "H264/MVC Encoded Stream",
		.fourcc     = V4L2_PIX_FMT_H264_MVC,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_AVC_MVC,
	},
	{
		.name       = "H265 HEVC Encoded Stream",
		.fourcc     = VPU_PIX_FMT_HEVC,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_HEVC,
	},
	{
		.name       = "VP9 Encoded Stream",
		.fourcc     = VPU_PIX_FMT_VP9,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_VP9,
	},
	{
		.name       = "Logo",
		.fourcc     = VPU_PIX_FMT_LOGO,
		.num_planes = 1,
		.vdec_std   = VPU_VIDEO_UNDEFINED,
	},
};

static struct vpu_v4l2_fmt  formats_yuv_dec[] = {
	{
		.name       = "4:2:0 2 Planes Y/CbCr",
		.fourcc     = V4L2_PIX_FMT_NV12,
		.num_planes	= 2,
		.vdec_std   = VPU_PF_YUV420_SEMIPLANAR,
	},
};

static int v4l2_ioctl_querycap(struct file *file,
		void *fh,
		struct v4l2_capability *cap
		)
{
	vpu_dbg(LVL_INFO, "%s()\n", __func__);
	strncpy(cap->driver, "vpu B0", sizeof(cap->driver) - 1);
	strlcpy(cap->card, "vpu B0", sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:", sizeof(cap->bus_info));
	cap->version = KERNEL_VERSION(0, 0, 1);
	cap->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int v4l2_ioctl_enum_fmt_vid_cap_mplane(struct file *file,
		void *fh,
		struct v4l2_fmtdesc *f
		)
{
	struct vpu_v4l2_fmt *fmt;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);
	if (f->index >= ARRAY_SIZE(formats_yuv_dec))
		return -EINVAL;

	fmt = &formats_yuv_dec[f->index];
	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	return 0;
}
static int v4l2_ioctl_enum_fmt_vid_out_mplane(struct file *file,
		void *fh,
		struct v4l2_fmtdesc *f
		)
{
	struct vpu_v4l2_fmt *fmt;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);
	if (f->index >= ARRAY_SIZE(formats_compressed_dec))
		return -EINVAL;

	fmt = &formats_compressed_dec[f->index];
	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	f->flags |= V4L2_FMT_FLAG_COMPRESSED;
	return 0;
}

static void caculate_frame_size(struct vpu_ctx *ctx)
{
	u_int32 width = ctx->pSeqinfo->uHorDecodeRes;
	u_int32 height = ctx->pSeqinfo->uVerDecodeRes;
	u_int32 luma_size;
	u_int32 chroma_size;
	u_int32 chroma_height;
	bool bfield = false; //WARN need get it
	bool bOffsetPadding = false; //WARN need get it
	u_int32 uVertAlign = 256-1;
	bool b10BitFormat = (ctx->pSeqinfo->uBitDepthLuma > 8) || (ctx->pSeqinfo->uBitDepthChroma > 8);

	struct queue_data *q_data;

	q_data = &ctx->q_data[V4L2_DST];

	width = ((width + uVertAlign) & ~uVertAlign);
	q_data->stride = width;
	if (bfield)
		height = ctx->pSeqinfo->uVerRes >> 0x1;
	if (bOffsetPadding) {
		height = ((height + 0xF) & 0xFFFFFFF0);
		height += 0x10;
	}

	height = ((height + uVertAlign) & ~uVertAlign);
	chroma_height = height >> 1;
	luma_size = width * height;
	chroma_size = width * chroma_height;
	if (!b10BitFormat) {
		ctx->q_data[V4L2_DST].sizeimage[0] = luma_size;
		ctx->q_data[V4L2_DST].sizeimage[1] = chroma_size;
	} else {
		ctx->q_data[V4L2_DST].sizeimage[0] = luma_size * 2;
		ctx->q_data[V4L2_DST].sizeimage[1] = chroma_size * 2;
	}
}

static int v4l2_ioctl_g_fmt(struct file *file,
		void *fh,
		struct v4l2_format *f
		)
{
	struct vpu_ctx *ctx =           v4l2_fh_to_ctx(fh);
	struct v4l2_pix_format_mplane   *pix_mp = &f->fmt.pix_mp;
	unsigned int i;
	struct queue_data               *q_data;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		q_data = &ctx->q_data[V4L2_DST];
		pix_mp->pixelformat = V4L2_PIX_FMT_NV12;
		pix_mp->width = ctx->pSeqinfo->uHorRes > 0?ctx->pSeqinfo->uHorRes:q_data->width;
		pix_mp->height = ctx->pSeqinfo->uVerRes > 0?ctx->pSeqinfo->uVerRes:q_data->height;
		pix_mp->field = V4L2_FIELD_ANY;
		pix_mp->num_planes = 2;
		pix_mp->colorspace = V4L2_COLORSPACE_REC709;

		for (i = 0; i < pix_mp->num_planes; i++) {
			pix_mp->plane_fmt[i].bytesperline = q_data->stride;
			pix_mp->plane_fmt[i].sizeimage = q_data->sizeimage[i];
		}
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		q_data = &ctx->q_data[V4L2_SRC];
		pix_mp->width = q_data->width;
		pix_mp->height = q_data->height;
		pix_mp->field = V4L2_FIELD_ANY;
		pix_mp->num_planes = q_data->num_planes;
		for (i = 0; i < pix_mp->num_planes; i++) {
			pix_mp->plane_fmt[i].bytesperline = q_data->stride;
			pix_mp->plane_fmt[i].sizeimage = q_data->sizeimage[i];

		}
		pix_mp->pixelformat = q_data->fourcc;
	} else
		return -EINVAL;
	return 0;
}

static void set_video_standard(struct queue_data *q_data,
		struct v4l2_format *f,
		struct vpu_v4l2_fmt *pformat_table,
		uint32_t table_size)
{
	unsigned int i;

	for (i = 0; i < table_size; i++) {
		if (pformat_table[i].fourcc == f->fmt.pix_mp.pixelformat)
			q_data->vdec_std = pformat_table[i].vdec_std;
	}
}

static int v4l2_ioctl_s_fmt(struct file *file,
		void *fh,
		struct v4l2_format *f
		)
{
	struct vpu_ctx                  *ctx = v4l2_fh_to_ctx(fh);
	struct v4l2_pix_format_mplane   *pix_mp = &f->fmt.pix_mp;
	struct queue_data               *q_data;
	u_int32                         i;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		q_data = &ctx->q_data[V4L2_DST];
		set_video_standard(q_data, f, formats_yuv_dec, ARRAY_SIZE(formats_yuv_dec));
		pix_mp->num_planes = 2;
		pix_mp->colorspace = V4L2_COLORSPACE_REC709;
		for (i = 0; i < pix_mp->num_planes; i++) {
			if (ctx->q_data[V4L2_DST].stride > 0)
				pix_mp->plane_fmt[i].bytesperline = ctx->q_data[V4L2_DST].stride;
			if (ctx->q_data[V4L2_DST].sizeimage[i] > 0)
				pix_mp->plane_fmt[i].sizeimage = ctx->q_data[V4L2_DST].sizeimage[i];
		}
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		q_data = &ctx->q_data[V4L2_SRC];
		set_video_standard(q_data, f, formats_compressed_dec, ARRAY_SIZE(formats_compressed_dec));
	} else
		return -EINVAL;

	q_data->num_planes = pix_mp->num_planes;
	for (i = 0; i < q_data->num_planes; i++) {
		q_data->stride = pix_mp->plane_fmt[i].bytesperline;
		q_data->sizeimage[i] = pix_mp->plane_fmt[i].sizeimage;
	}
	q_data->fourcc = pix_mp->pixelformat;
	q_data->width = pix_mp->width;
	q_data->height = pix_mp->height;
	q_data->rect.left = 0;
	q_data->rect.top = 0;
	q_data->rect.width = pix_mp->width;
	q_data->rect.height = pix_mp->height;

	return 0;
}

static int v4l2_ioctl_expbuf(struct file *file,
		void *fh,
		struct v4l2_exportbuffer *buf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

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
	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		return -EINVAL;
	}
}
static void alloc_mbi_buffer(struct vpu_ctx *ctx,
		struct queue_data *This,
		u_int32 count)
{
	u_int32 uAlign = 0x800-1;
	u_int32 mbi_num;
	u_int32 mbi_size;
	u_int32 i;


	if (count >= MAX_MBI_NUM)
		mbi_num = MAX_MBI_NUM;
	else
		mbi_num = count;
	ctx->mbi_num = mbi_num;

	mbi_size = (This->sizeimage[0]+This->sizeimage[1])/4;
	mbi_size = ((mbi_size + uAlign) & ~uAlign);
	ctx->mbi_size = mbi_size;
	for (i = 0; i < mbi_num; i++) {
		ctx->mbi_dma_virt[i] = dma_alloc_coherent(&ctx->dev->plat_dev->dev,
			ctx->mbi_size,
			(dma_addr_t *)&ctx->mbi_dma_phy[i],
			GFP_KERNEL | GFP_DMA32
			);
	if (!ctx->mbi_dma_virt[i])
		vpu_dbg(LVL_ERR, "error: %s() mbi buffer alloc size(%x) fail!\n", __func__,  mbi_size);
	}
}
static int v4l2_ioctl_reqbufs(struct file *file,
		void *fh,
		struct v4l2_requestbuffers *reqbuf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	u_int32 i;
	int ret;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (reqbuf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (reqbuf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	if (reqbuf->count == 0)
		ctx->buffer_null = true;
	else
		ctx->buffer_null = false;

	ret = vb2_reqbufs(&q_data->vb2_q, reqbuf);
	if (!ret) {
		if (reqbuf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
			for (i = 0; i < reqbuf->count; i++)
				q_data->vb2_reqs[i].status = FRAME_ALLOC;
			alloc_mbi_buffer(ctx, q_data, reqbuf->count);
		}
	} else if (reqbuf->count != 0)
		vpu_dbg(LVL_ERR, "error: %s() can't request (%d) buffer\n", __func__, reqbuf->count);

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

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

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

static int v4l2_ioctl_qbuf(struct file *file,
		void *fh,
		struct v4l2_buffer *buf
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);
	struct queue_data *q_data;
	int ret;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		q_data = &ctx->q_data[V4L2_SRC];
		v4l2_update_stream_addr(ctx, 0);
	} else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	ret = vb2_qbuf(&q_data->vb2_q, buf);
	if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		wake_up_interruptible(&ctx->buffer_wq);
	v4l2_update_stream_addr(ctx, 0);

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

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	ret = vb2_dqbuf(&q_data->vb2_q, buf, file->f_flags & O_NONBLOCK);

	v4l2_update_stream_addr(ctx, 0);
	if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		if ((ctx->pSeqinfo->uBitDepthLuma > 8) || (ctx->pSeqinfo->uBitDepthChroma > 8))
			buf->reserved = 1;

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
	unsigned int table_size;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		table_size = ARRAY_SIZE(formats_compressed_dec);
		if (!format_is_support(formats_compressed_dec, table_size, f))
			return -EINVAL;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		table_size = ARRAY_SIZE(formats_yuv_dec);
		if (!format_is_support(formats_yuv_dec, table_size, f))
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
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);

	vpu_dbg(LVL_INFO, "%s()\n", __func__);
	cr->c.left = ctx->pSeqinfo->uFrameCropLeftOffset;
	cr->c.top = ctx->pSeqinfo->uFrameCropTopOffset;
	cr->c.width = ctx->pSeqinfo->uHorRes;
	cr->c.height = ctx->pSeqinfo->uVerRes;

	return 0;
}

static int v4l2_ioctl_decoder_cmd(struct file *file,
		void *fh,
		struct v4l2_decoder_cmd *cmd
		)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(fh);

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	switch (cmd->cmd) {
	case V4L2_DEC_CMD_START:
		break;
	case V4L2_DEC_CMD_STOP: {
		vpu_dbg(LVL_INFO, "receive V4L2_DEC_CMD_STOP\n");
		if (!ctx->firmware_stopped)	{
			// All stream has been fed to the decoder, now wait for a VID_API_EVENT_FIFO_LOW
			// to signify that the decoder has consumed all stream data.
			// ctx->stream_feed_complete is set to indicate that on the next VID_API_EVENT_FIFO_LOW
			// the driver should respond by inserting an EOS
			ctx->stream_feed_complete = true;
			vpu_dbg(LVL_INFO, "END OF STREAM FED - waiting for VID_API_EVENT_FIFO_LOW\n");

			ctx->stream_feed_complete = true;
			vpu_dbg(LVL_ALL, "END OF STREAM FED - waiting for VID_API_EVENT_FIFO_LOW\n");
			if (!wait_for_completion_timeout(&ctx->eos_cmp, msecs_to_jiffies(2000))) {
				vpu_dbg(LVL_ERR, "wait FIFO LOW timeout, insert eos directly\n");
				ctx->eos_stop_added = true;
				ctx->stream_feed_complete = false;
				v4l2_update_stream_addr(ctx, 0);
				add_eos(ctx, 0);
			}

		} else	{
			vpu_dbg(LVL_ERR, "Firmware already stopped !\n");
		}
	} break;
	case V4L2_DEC_CMD_PAUSE:
		break;
	case V4L2_DEC_CMD_RESUME:
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

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (i == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (i == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;
	ret = vb2_streamon(&q_data->vb2_q,	i);
	if (i == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		wake_up_interruptible(&ctx->buffer_wq);

	v4l2_update_stream_addr(ctx, 0);

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

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (i == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		q_data = &ctx->q_data[V4L2_SRC];
	else if (i == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		q_data = &ctx->q_data[V4L2_DST];
	else
		return -EINVAL;

	if (i == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (ctx->firmware_stopped || ctx->firmware_finished || ctx->eos_stop_added) {
			vpu_dbg(LVL_ERR, "v4l2_ioctl_streamoff() - IGNORE - stopped(%d), finished(%d), eos_added(%d), feed_complete(%d)\n",
					ctx->firmware_stopped, ctx->firmware_finished, ctx->eos_stop_added, ctx->stream_feed_complete);
		} else {
			ctx->wait_rst_done = true;
			if (ctx->stream_feed_complete == true)
				vpu_dbg(LVL_ERR, "v4l2_ioctl_streamoff() - EOS won't be inserted by driver\n");
			ctx->stream_feed_complete = false;
			vpu_dbg(LVL_INFO, "v4l2_ioctl_streamoff(): send VID_API_CMD_ABORT\n");

			v4l2_vpu_send_cmd(ctx, ctx->str_index, VID_API_CMD_ABORT, 0, NULL);

			wake_up_interruptible(&ctx->buffer_wq);
			if (!wait_for_completion_timeout(&ctx->completion, msecs_to_jiffies(1000))) {
				mutex_lock(&ctx->dev->dev_mutex);
				set_bit(ctx->str_index, &ctx->dev->hang_mask);
				mutex_unlock(&ctx->dev->dev_mutex);
				vpu_dbg(LVL_ERR, "the path id:%d firmware hang after send VID_API_CMD_ABORT\n", ctx->str_index);
			}
			vpu_dbg(LVL_INFO, "receive abort done\n");
		}
	}

	ret = vb2_streamoff(&q_data->vb2_q,
			i);

	if (ctx->dev->hang_mask & (1 << ctx->str_index))
		return -EINVAL;
	else
		return ret;
}

static const struct v4l2_ioctl_ops v4l2_decoder_ioctl_ops = {
	.vidioc_querycap                = v4l2_ioctl_querycap,
	.vidioc_enum_fmt_vid_cap_mplane = v4l2_ioctl_enum_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_out_mplane = v4l2_ioctl_enum_fmt_vid_out_mplane,
	.vidioc_g_fmt_vid_cap_mplane    = v4l2_ioctl_g_fmt,
	.vidioc_g_fmt_vid_out_mplane    = v4l2_ioctl_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane  = v4l2_ioctl_try_fmt,
	.vidioc_try_fmt_vid_out_mplane  = v4l2_ioctl_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane    = v4l2_ioctl_s_fmt,
	.vidioc_s_fmt_vid_out_mplane    = v4l2_ioctl_s_fmt,
	.vidioc_expbuf                  = v4l2_ioctl_expbuf,
	.vidioc_g_crop                  = v4l2_ioctl_g_crop,
	.vidioc_decoder_cmd             = v4l2_ioctl_decoder_cmd,
	.vidioc_subscribe_event         = v4l2_ioctl_subscribe_event,
	.vidioc_unsubscribe_event       = v4l2_event_unsubscribe,
	.vidioc_reqbufs                 = v4l2_ioctl_reqbufs,
	.vidioc_querybuf                = v4l2_ioctl_querybuf,
	.vidioc_qbuf                    = v4l2_ioctl_qbuf,
	.vidioc_dqbuf                   = v4l2_ioctl_dqbuf,
	.vidioc_streamon                = v4l2_ioctl_streamon,
	.vidioc_streamoff               = v4l2_ioctl_streamoff,
};

// Set/Get controls - v4l2 control framework

static struct vpu_v4l2_control vpu_controls_dec[] = {
	{
		.id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 1,
		.maximum = 32,
		.step = 1,
		.default_value = 4,
		.is_volatile = true,
	},
};

#define NUM_CTRLS_DEC   ARRAY_SIZE(vpu_controls_dec)

static int v4l2_dec_s_ctrl(struct v4l2_ctrl *ctrl)
{
	switch (ctrl->id) {
	default:
		vpu_dbg(LVL_INFO, "%s() Invalid control(%d)\n",
				__func__, ctrl->id);
		return -EINVAL;
	}
	return 0;
}

static int v4l2_dec_g_v_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vpu_ctx *ctx = v4l2_ctrl_to_ctx(ctrl);

	vpu_dbg(LVL_INFO, "%s() control(%d)\n",
			__func__, ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		ctrl->val = ctx->pSeqinfo->uNumDPBFrms;
		break;
	default:
		vpu_dbg(LVL_INFO, "%s() Invalid control(%d)\n",
				__func__, ctrl->id);
		return -EINVAL;
	}
	return 0;
}

static const struct v4l2_ctrl_ops   vpu_dec_ctrl_ops = {
	.s_ctrl             = v4l2_dec_s_ctrl,
	.g_volatile_ctrl    = v4l2_dec_g_v_ctrl,
};

static int ctrls_setup_decoder(struct vpu_ctx *This)
{
	int i;

	v4l2_ctrl_handler_init(&This->ctrl_handler,
			NUM_CTRLS_DEC + 1
			);
	if (This->ctrl_handler.error) {
		vpu_dbg(LVL_ERR, "%s() v4l2_ctrl_handler_init failed(%d)\n",
				__func__, This->ctrl_handler.error);

		return This->ctrl_handler.error;
	} else {
		vpu_dbg(LVL_INFO, "%s() v4l2_ctrl_handler_init ctrls(%ld)\n",
				__func__, NUM_CTRLS_DEC);
		This->ctrl_inited = true;
	}

	for (i = 0; i < NUM_CTRLS_DEC; i++) {
		This->ctrls[i] = v4l2_ctrl_new_std(&This->ctrl_handler,
				&vpu_dec_ctrl_ops,
				vpu_controls_dec[i].id,
				vpu_controls_dec[i].minimum,
				vpu_controls_dec[i].maximum,
				vpu_controls_dec[i].step,
				vpu_controls_dec[i].default_value
				);
		if (This->ctrl_handler.error ||
				!This->ctrls[i]
				) {
			vpu_dbg(LVL_ERR, "%s() v4l2_ctrl_new_std failed(%d) This->ctrls[%d](%p)\n",
					__func__, This->ctrl_handler.error, i, This->ctrls[i]);
			return This->ctrl_handler.error;
		}

		if (vpu_controls_dec[i].is_volatile &&
				This->ctrls[i]
				)
			This->ctrls[i]->flags |= V4L2_CTRL_FLAG_VOLATILE;
	}

	v4l2_ctrl_handler_setup(&This->ctrl_handler);

	return 0;
}

static void ctrls_delete_decoder(struct vpu_ctx *This)
{
	int i;

	if (This->ctrl_inited) {
		v4l2_ctrl_handler_free(&This->ctrl_handler);
		This->ctrl_inited = false;
	}
	for (i = 0; i < NUM_CTRLS_DEC; i++)
		This->ctrls[i] = NULL;
}

static void add_eos(struct vpu_ctx *ctx, u_int32 uStrBufIdx)
{
	struct vpu_dev *dev = ctx->dev;
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	uint32_t start;
	uint32_t end;
	uint32_t wptr;
	uint32_t rptr;
	uint8_t *pbbuffer;
	uint32_t *plbuffer;
	uint32_t last;
	uint32_t last2 = 0x0;
	uint32_t pad_bytes;
	static uint8_t *buffer;

	vpu_dbg(LVL_INFO, "enter %s\n", __func__);
	pStrBufDesc = ctx->dev->regs_base + DEC_MFD_XREG_SLV_BASE + MFD_MCX + MFD_MCX_OFF * ctx->str_index;
	start = pStrBufDesc->start;
	end = pStrBufDesc->end;
	wptr = pStrBufDesc->wptr;
	rptr = pStrBufDesc->rptr;

	buffer = kzalloc(MIN_SPACE, GFP_KERNEL); //for eos data
	if (!buffer)
		vpu_dbg(LVL_ERR, "error:  eos buffer alloc fail\n");
	plbuffer = (uint32_t *)buffer;
	pbbuffer = (uint8_t *)(ctx->stream_buffer_virt + wptr - start);

	// Word align
	if (((u_int64)pbbuffer)%4 != 0) {
		int i;

		pad_bytes = 4 - (((u_int64)pbbuffer)%4);
		for (i = 0; i < pad_bytes; i++)
			pbbuffer[i] = 0;
		pbbuffer += pad_bytes;
		if (end%4 != 0)
			vpu_dbg(LVL_ERR, "end address of stream not aligned by 4 bytes !\n");
		wptr += pad_bytes;
		if (wptr == end)
			wptr = start;
	}

	switch (q_data->vdec_std) {
	case VPU_VIDEO_AVC:
		last = 0x0B010000;
		break;
	case VPU_VIDEO_VC1:
		last = 0x0a010000;
		break;
	case VPU_VIDEO_MPEG2:
		last = 0xb7010000;
		break;
	case VPU_VIDEO_ASP:
		last = 0xb1010000;
		break;
	case VPU_VIDEO_SPK:
	case VPU_VIDEO_VP6:
	case VPU_VIDEO_VP8:
	case VPU_VIDEO_RV8:
	case VPU_VIDEO_RV9:
		last = 0x34010000;
		break;
	case VPU_VIDEO_HEVC:
		last = 0x4A010000;
		last2 = 0x20;
		break;
	default:
		last = 0x0;
		break;
	}

	plbuffer[0] = last;
	plbuffer[1] = last2;

#if 0
	for (i = 2; i < MIN_SPACE >> 2;  i++)
		plbuffer[i] = 0;
#endif

	if ((wptr == rptr) || (wptr > rptr)) {
		if (end - wptr >= MIN_SPACE) {
			memcpy(pbbuffer, buffer, MIN_SPACE);
			wptr += MIN_SPACE;
			if (wptr == end)
				wptr = start;
		} else {
			memcpy(pbbuffer, buffer, end-wptr);
			memcpy(ctx->stream_buffer_virt, buffer + (end-wptr), MIN_SPACE - (end-wptr));
			wptr = start + MIN_SPACE-(end-wptr);
		}
	} else {
		if (rptr - wptr >= MIN_SPACE) {
			memcpy(pbbuffer, buffer, MIN_SPACE);
			wptr += MIN_SPACE;
		} else	{
			//shouldn't enter here: suppose space is enough since add_eos() only be called in FIFO LOW
			memcpy(pbbuffer, buffer, rptr - wptr);
			wptr += (rptr - wptr);
			vpu_dbg(LVL_ERR, "No enough space to insert EOS !\n");
		}
	}
	mb();

	pStrBufDesc->wptr = wptr;
	dev->shared_mem.pSharedInterface->pStreamBuffDesc[ctx->str_index][uStrBufIdx] =
		(VPU_REG_BASE + DEC_MFD_XREG_SLV_BASE + MFD_MCX + MFD_MCX_OFF * ctx->str_index);
	kfree(buffer);
	vpu_dbg(LVL_INFO, "add eos MCX address virt=%p, phy=0x%x, index=%d\n", pStrBufDesc, dev->shared_mem.pSharedInterface->pStreamBuffDesc[ctx->str_index][uStrBufIdx], ctx->str_index);
}

TB_API_DEC_FMT vpu_format_remap(uint32_t vdec_std)
{
	TB_API_DEC_FMT malone_format = VSys_FrmtNull;

	switch (vdec_std) {
	case VPU_VIDEO_AVC:
		malone_format = VSys_AvcFrmt;
		vpu_dbg(LVL_INFO, "format translated to AVC");
		break;
	case VPU_VIDEO_VC1:
		malone_format = VSys_Vc1Frmt;
		vpu_dbg(LVL_INFO, "format translated to VC1");
		break;
	case VPU_VIDEO_MPEG2:
		malone_format = VSys_Mp2Frmt;
		vpu_dbg(LVL_INFO, "format translated to MP2");
		break;
	case VPU_VIDEO_AVS:
		malone_format = VSys_AvsFrmt;
		vpu_dbg(LVL_INFO, "format translated to AVS");
		break;
	case VPU_VIDEO_ASP:
		malone_format = VSys_AspFrmt;
		vpu_dbg(LVL_INFO, "format translated to ASP");
		break;
	case VPU_VIDEO_JPEG:
		malone_format = VSys_JpgFrmt;
		vpu_dbg(LVL_INFO, "format translated to JPG");
		break;
	case VPU_VIDEO_VP6:
		malone_format = VSys_Vp6Frmt;
		vpu_dbg(LVL_INFO, "format translated to VP6");
		break;
	case VPU_VIDEO_SPK:
		malone_format = VSys_SpkFrmt;
		vpu_dbg(LVL_INFO, "format translated to SPK");
		break;
	case VPU_VIDEO_VP8:
		malone_format = VSys_Vp8Frmt;
		vpu_dbg(LVL_INFO, "format translated to VP8");
		break;
	case VPU_VIDEO_HEVC:
		malone_format = VSys_HevcFrmt;
		vpu_dbg(LVL_INFO, "format translated to HEVC");
		break;
	case VPU_VIDEO_RV8:
		malone_format = VSys_RvFrmt;
		vpu_dbg(LVL_INFO, "format translated to RV");
		break;
	case VPU_VIDEO_RV9:
		malone_format = VSys_RvFrmt;
		vpu_dbg(LVL_INFO, "format translated to RV");
		break;
	case VPU_VIDEO_AVC_MVC:
		malone_format = VSys_AvcFrmt;
		vpu_dbg(LVL_INFO, "format translated to AVC");
		break;
	default:
		malone_format = VSys_FrmtNull;
		vpu_dbg(LVL_INFO, "unspport format");
		break;
	}
	vpu_dbg(LVL_INFO, "\n");

	return malone_format;
}

static void v4l2_vpu_send_cmd(struct vpu_ctx *ctx, uint32_t idx, uint32_t cmdid, uint32_t cmdnum, uint32_t *local_cmddata)
{
	vpu_log_cmd(cmdid, idx);
	mutex_lock(&ctx->dev->cmd_mutex);
	rpc_send_cmd_buf(&ctx->dev->shared_mem, idx, cmdid, cmdnum, local_cmddata);
	mutex_unlock(&ctx->dev->cmd_mutex);
	mb();
	MU_SendMessage(ctx->dev->mu_base_virtaddr, 0, COMMAND);
}
static void transfer_buffer_to_firmware(struct vpu_ctx *ctx, void *input_buffer, uint32_t buffer_size, uint32_t vdec_std)
{
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
	u_int32 uStrBufIdx = 0; //set to be default 0, FIX_ME later
	MediaIPFW_Video_UData *pUdataBuf =
		&ctx->dev->shared_mem.pSharedInterface->UDataBuffer[ctx->str_index];
	pDEC_RPC_HOST_IFACE pSharedInterface = ctx->dev->shared_mem.pSharedInterface;
	unsigned int *CurrStrfg = &pSharedInterface->StreamConfig[ctx->str_index];

	vpu_dbg(LVL_INFO, "enter %s, start_flag %d, index=%d, firmware_started=%d\n", __func__, ctx->start_flag, ctx->str_index, ctx->dev->firmware_started);

	vpu_dbg(LVL_ALL, "firmware version is %d.%d.%d\n", (pSharedInterface->FWVersion & 0x00ff0000) >> 16, (pSharedInterface->FWVersion & 0x0000ff00) >> 8, pSharedInterface->FWVersion & 0x000000ff);

	if (ctx->stream_buffer_size < buffer_size + MIN_SPACE)
		vpu_dbg(LVL_INFO, "circular buffer size is too small\n");
	memcpy(ctx->stream_buffer_virt, input_buffer, buffer_size);
	vpu_dbg(LVL_INFO, "transfer data from virt 0x%p: size:%d\n", ctx->stream_buffer_virt, buffer_size);
	mb();
	pStrBufDesc = ctx->dev->regs_base + DEC_MFD_XREG_SLV_BASE + MFD_MCX + MFD_MCX_OFF * ctx->str_index;
	// CAUTION: wptr must not be end
	pStrBufDesc->wptr = ctx->stream_buffer_phy + buffer_size - ctx->dev->cm_offset;
	pStrBufDesc->rptr = ctx->stream_buffer_phy - ctx->dev->cm_offset;
	pStrBufDesc->start = ctx->stream_buffer_phy - ctx->dev->cm_offset;
	pStrBufDesc->end = ctx->stream_buffer_phy + ctx->stream_buffer_size - ctx->dev->cm_offset;
	pStrBufDesc->LWM = 0x01;

	ctx->dev->shared_mem.pSharedInterface->pStreamBuffDesc[ctx->str_index][uStrBufIdx] =
		(VPU_REG_BASE + DEC_MFD_XREG_SLV_BASE + MFD_MCX + MFD_MCX_OFF * ctx->str_index);

	vpu_dbg(LVL_INFO, "transfer MCX address virt=%p, phy=0x%x, index=%d\n", pStrBufDesc, ctx->dev->shared_mem.pSharedInterface->pStreamBuffDesc[ctx->str_index][uStrBufIdx], ctx->str_index);
	pUdataBuf->uUDataBase = ctx->udata_buffer_phy - ctx->dev->cm_offset;
	pUdataBuf->uUDataSlotSize = ctx->udata_buffer_size;
	VID_STREAM_CONFIG_FORMAT_SET(vpu_format_remap(vdec_std), CurrStrfg);
}

static void v4l2_transfer_buffer_to_firmware(struct queue_data *This, struct vb2_buffer *vb)
{
	struct vpu_ctx *ctx = container_of(This, struct vpu_ctx, q_data[V4L2_SRC]);
	struct vb2_data_req *p_data_req;
	void *data_mapped;
	uint32_t buffer_size = vb->planes[0].bytesused;

	data_mapped = (void *)vb2_plane_vaddr(vb, 0);

	if (ctx->start_flag == true) {
		transfer_buffer_to_firmware(ctx, data_mapped, buffer_size, This->vdec_std);
#ifdef HANDLE_EOS
		if (vb->planes[0].bytesused < vb->planes[0].length) {
			vpu_dbg(LVL_INFO, "v4l2_transfer_buffer_to_firmware - set stream_feed_complete - DEBUG 1\n")
			ctx->stream_feed_complete = true;
		}
#endif
		v4l2_vpu_send_cmd(ctx, ctx->str_index, VID_API_CMD_START, 0, NULL);
		down(&This->drv_q_lock);
		p_data_req = list_first_entry(&This->drv_q,
				typeof(*p_data_req), list);
		list_del(&p_data_req->list);
		vb2_buffer_done(p_data_req->vb2_buf,
				VB2_BUF_STATE_DONE
				);
		up(&This->drv_q_lock);
		ctx->start_flag = false;
	}
}

static int update_stream_addr(struct vpu_ctx *ctx, void *input_buffer, uint32_t buffer_size, uint32_t uStrBufIdx)
{
	struct vpu_dev *dev = ctx->dev;
	uint32_t index = ctx->str_index;
	pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
	uint32_t nfreespace = 0;
	uint32_t wptr;
	uint32_t rptr;
	uint32_t start;
	uint32_t end;
	void *wptr_virt;
	uint32_t ret = 1;

	vpu_dbg(LVL_INFO, "enter %s\n", __func__);

	// changed to virtual address and back
	pStrBufDesc = dev->regs_base + DEC_MFD_XREG_SLV_BASE + MFD_MCX + MFD_MCX_OFF * index;
	vpu_dbg(LVL_INFO, "%s wptr(%x) rptr(%x) start(%x) end(%x) uStrBufIdx(%d)\n",
			__func__,
			pStrBufDesc->wptr,
			pStrBufDesc->rptr,
			pStrBufDesc->start,
			pStrBufDesc->end,
			uStrBufIdx
	      );
	wptr = pStrBufDesc->wptr;
	rptr = pStrBufDesc->rptr;

	start = pStrBufDesc->start;
	end = pStrBufDesc->end;
	wptr_virt = (void *)ctx->stream_buffer_virt + wptr - start;

	vpu_dbg(LVL_INFO, "update_stream_addr down\n");

	if (wptr == rptr)
		nfreespace = end - start;
	if (wptr < rptr)
		nfreespace = rptr - wptr;
	if (wptr > rptr)
		nfreespace = (end - wptr) + (rptr - start);

	if (nfreespace-buffer_size < MIN_SPACE)
			return 0;

	if (nfreespace >= buffer_size) {
		if ((wptr == rptr) || (wptr > rptr)) {
			if (end - wptr >= buffer_size) {
				memcpy(wptr_virt, input_buffer, buffer_size);
				wptr += buffer_size;
				if (wptr == end)
					wptr = start;
			} else {
				memcpy(wptr_virt, input_buffer, end-wptr);
				memcpy(ctx->stream_buffer_virt, input_buffer + (end-wptr), buffer_size - (end-wptr));
				wptr = start + buffer_size - (end-wptr);
			}
		} else {
			memcpy(wptr_virt, input_buffer, buffer_size);
			wptr += buffer_size;
		}
	} else {
		vpu_dbg(LVL_INFO, "buffer_full: the circular buffer freespace < buffer_size, treat as full");
		return 0; //do not consider this situation now
	}

	mb();
	pStrBufDesc->wptr = wptr;
			vpu_dbg(LVL_INFO, "update_stream_addr up, wptr 0x%x\n", wptr);

	dev->shared_mem.pSharedInterface->pStreamBuffDesc[index][uStrBufIdx] =
		(VPU_REG_BASE + DEC_MFD_XREG_SLV_BASE + MFD_MCX + MFD_MCX_OFF * index);

	vpu_dbg(LVL_INFO, "update address virt=%p, phy=0x%x, index=%d\n", pStrBufDesc, dev->shared_mem.pSharedInterface->pStreamBuffDesc[ctx->str_index][uStrBufIdx], ctx->str_index);
	return ret;
}
//warn uStrIdx need to refine how to handle it
static void v4l2_update_stream_addr(struct vpu_ctx *ctx, uint32_t uStrBufIdx)
{
	struct vb2_data_req *p_data_req;
	struct queue_data *This = &ctx->q_data[V4L2_SRC];
	void *input_buffer;
	uint32_t buffer_size;

	down(&This->drv_q_lock);
	while (!list_empty(&This->drv_q)) {
		p_data_req = list_first_entry(&This->drv_q,
				typeof(*p_data_req), list);

		buffer_size = p_data_req->vb2_buf->planes[0].bytesused;
		input_buffer = (void *)vb2_plane_vaddr(p_data_req->vb2_buf, 0);
		if (!update_stream_addr(ctx, input_buffer, buffer_size, uStrBufIdx)) {
			up(&This->drv_q_lock);
			vpu_dbg(LVL_INFO, " %s no space to write\n", __func__);
			return;
		}
#ifdef HANDLE_EOS
		if (buffer_size < p_data_req->vb2_buf->planes[0].length) {
			vpu_dbg(LVL_INFO, "v4l2_transfer_buffer_to_firmware - set stream_feed_complete - DEBUG 2\n")
			ctx->stream_feed_complete = true;
		}
#endif
		list_del(&p_data_req->list);
		vb2_buffer_done(p_data_req->vb2_buf,
				VB2_BUF_STATE_DONE
			       );
	}
	up(&This->drv_q_lock);

}

static void report_buffer_done(struct vpu_ctx *ctx, void *frame_info)
{
	struct vb2_data_req *p_data_req;
	struct queue_data *This = &ctx->q_data[V4L2_DST];
	u_int32 *FrameInfo = (u_int32 *)frame_info;
	u_int32 fs_id = FrameInfo[0x0];
	uint32_t stride = FrameInfo[3];
	bool b10BitFormat = (ctx->pSeqinfo->uBitDepthLuma > 8) || (ctx->pSeqinfo->uBitDepthChroma > 8);
	int buffer_id;

	vpu_dbg(LVL_INFO, "report_buffer_done fs_id=%d, ulFsLumaBase[0]=%x, stride=%d, b10BitFormat=%d, ctx->pSeqinfo->uBitDepthLuma=%d\n", fs_id, FrameInfo[1], stride, b10BitFormat, ctx->pSeqinfo->uBitDepthLuma);
	v4l2_update_stream_addr(ctx, 0);

	buffer_id = find_buffer_id(ctx, FrameInfo[1]);

	if (buffer_id == -1)
		return;

	if (buffer_id != fs_id)
		vpu_dbg(LVL_ERR, "error: find buffer_id(%d) and firmware return id(%d) doesn't match\n",
				buffer_id, fs_id);
	if (ctx->q_data[V4L2_DST].vb2_reqs[buffer_id].status != FRAME_DECODED)
		vpu_dbg(LVL_ERR, "error: buffer(%d) need to set FRAME_READY, but previous state %s is not FRAME_DECODED\n",
				buffer_id, bufstat[ctx->q_data[V4L2_DST].vb2_reqs[buffer_id].status]);

	ctx->q_data[V4L2_DST].vb2_reqs[buffer_id].status = FRAME_READY;

	down(&This->drv_q_lock);
	p_data_req = &This->vb2_reqs[buffer_id];
	p_data_req->vb2_buf->planes[0].bytesused = This->sizeimage[0];
	p_data_req->vb2_buf->planes[1].bytesused = This->sizeimage[1];
	if (p_data_req->vb2_buf->state == VB2_BUF_STATE_ACTIVE)
		vb2_buffer_done(p_data_req->vb2_buf,
				VB2_BUF_STATE_DONE
				);
	else
		vpu_dbg(LVL_ERR, "error: check buffer(%d) state(%d)\n", buffer_id, p_data_req->vb2_buf->state);
	up(&This->drv_q_lock);
	vpu_dbg(LVL_INFO, "leave %s\n", __func__);
}

/*
 * this is used for waiting the right status buffer in the queue list
 * true means find right buffer, false means not
 */
static bool wait_right_buffer(struct queue_data *This)
{
	struct vb2_data_req *p_data_req, *p_temp;

	down(&This->drv_q_lock);
	if (!list_empty(&This->drv_q)) {
		list_for_each_entry_safe(p_data_req, p_temp, &This->drv_q, list)
			if (p_data_req->status == FRAME_ALLOC
					|| p_data_req->status == FRAME_RELEASE) {
				up(&This->drv_q_lock);
				return true;
			}
	}
	up(&This->drv_q_lock);

	return false;
}

static void vpu_api_event_handler(struct vpu_ctx *ctx, u_int32 uStrIdx, u_int32 uEvent, u_int32 *event_data)
{
	struct vpu_dev *dev;
	pDEC_RPC_HOST_IFACE pSharedInterface;

	vpu_log_event(uEvent, uStrIdx);

	if (ctx == NULL) {
		vpu_dbg(LVL_ERR, "receive event: 0x%X after instance released, ignore it\n", uEvent);
		return;
	}

	if (ctx->firmware_stopped) {
		vpu_dbg(LVL_ERR, "receive event: 0x%X after stopped, ignore it\n", uEvent);
		return;
	}
	dev = ctx->dev;
	pSharedInterface = (pDEC_RPC_HOST_IFACE)dev->shared_mem.shared_mem_vir;

	switch (uEvent) {
	case VID_API_EVENT_START_DONE:
		break;
	case VID_API_EVENT_STOPPED: {
		vpu_dbg(LVL_INFO, "receive VID_API_EVENT_STOPPED\n");
		ctx->firmware_stopped = true;
		complete(&ctx->stop_cmp);
		}
		break;
	case VID_API_EVENT_RESET_DONE:
		break;
	case VID_API_EVENT_PIC_DECODED: {
		MediaIPFW_Video_QMeterInfo *pQMeterInfo = (MediaIPFW_Video_QMeterInfo *)dev->shared_mem.qmeter_mem_vir;
		MediaIPFW_Video_PicInfo *pPicInfo = (MediaIPFW_Video_PicInfo *)dev->shared_mem.pic_mem_vir;
		MediaIPFW_Video_PicDispInfo *pDispInfo = &pPicInfo[uStrIdx].DispInfo;
		MediaIPFW_Video_PicPerfInfo *pPerfInfo = &pPicInfo[uStrIdx].PerfInfo;
		MediaIPFW_Video_PicPerfDcpInfo *pPerfDcpInfo = &pPicInfo[uStrIdx].PerfDcpInfo;
		int buffer_id;
		u_int32 uDecFrmId = event_data[7];
		u_int32 uPicStartAddr = event_data[10];
		struct queue_data *This = &ctx->q_data[V4L2_DST];
		u_int32 uDpbmcCrc;

		if (ctx->buffer_null == true) {
			vpu_dbg(LVL_INFO, "frame already released\n");
			break;
		}

		if (This->vdec_std == VPU_VIDEO_HEVC)
			uDpbmcCrc = pPerfDcpInfo->uDBEDpbCRC[0];
		else
			uDpbmcCrc = pPerfInfo->uMemCRC;
		vpu_dbg(LVL_INFO, "PICINFO GET: uPicType:%d uPicStruct:%d uPicStAddr:0x%x uFrameStoreID:%d uPercentInErr:%d, uRbspBytesCount=%d, ulLumBaseAddr[0]=%x, pQMeterInfo:%p, pPicInfo:%p, pDispInfo:%p, pPerfInfo:%p, pPerfDcpInfo:%p, uPicStartAddr=0x%x, uDpbmcCrc:%x\n",
				pPicInfo[uStrIdx].uPicType, pPicInfo[uStrIdx].uPicStruct,
				pPicInfo[uStrIdx].uPicStAddr, pPicInfo[uStrIdx].uFrameStoreID,
				pPicInfo[uStrIdx].uPercentInErr, pPerfInfo->uRbspBytesCount, event_data[0],
				pQMeterInfo, pPicInfo, pDispInfo, pPerfInfo, pPerfDcpInfo, uPicStartAddr, uDpbmcCrc);

		buffer_id = find_buffer_id(ctx, event_data[0]);

		if (buffer_id == -1)
			break;

		if (buffer_id != uDecFrmId)
			vpu_dbg(LVL_ERR, "error: VID_API_EVENT_PIC_DECODED address and id doesn't match\n");
		if (ctx->q_data[V4L2_DST].vb2_reqs[buffer_id].status != FRAME_FREE)
			vpu_dbg(LVL_ERR, "error: buffer(%d) need to set FRAME_DECODED, but previous state %s is not FRAME_FREE\n",
					buffer_id, bufstat[ctx->q_data[V4L2_DST].vb2_reqs[buffer_id].status]);
		ctx->q_data[V4L2_DST].vb2_reqs[buffer_id].status = FRAME_DECODED;
		}
		break;
	case VID_API_EVENT_SEQ_HDR_FOUND: {
		MediaIPFW_Video_SeqInfo *pSeqInfo = (MediaIPFW_Video_SeqInfo *)dev->shared_mem.seq_mem_vir;
//		MediaIPFW_Video_FrameBuffer *pStreamFrameBuffer = &pSharedInterface->StreamFrameBuffer[uStrIdx];
//		MediaIPFW_Video_FrameBuffer *pStreamDCPBuffer = &pSharedInterface->StreamDCPBuffer[uStrIdx];
		MediaIPFW_Video_PitchInfo   *pStreamPitchInfo = &pSharedInterface->StreamPitchInfo[uStrIdx];
		unsigned int num = pSharedInterface->SeqInfoTabDesc.uNumSizeDescriptors;

		if (ctx->pSeqinfo == NULL)
			ctx->pSeqinfo = kzalloc(sizeof(MediaIPFW_Video_SeqInfo), GFP_KERNEL);
		else
			vpu_dbg(LVL_INFO, "pSeqinfo is not NULL, need not to realloc\n");
		memcpy(ctx->pSeqinfo, &pSeqInfo[ctx->str_index], sizeof(MediaIPFW_Video_SeqInfo));

		caculate_frame_size(ctx);
		vpu_dbg(LVL_INFO, "SEQINFO GET: uHorRes:%d uVerRes:%d uHorDecodeRes:%d uVerDecodeRes:%d uNumDPBFrms:%d, num=%d\n",
				ctx->pSeqinfo->uHorRes, ctx->pSeqinfo->uVerRes,
				ctx->pSeqinfo->uHorDecodeRes, ctx->pSeqinfo->uVerDecodeRes,
				ctx->pSeqinfo->uNumDPBFrms, num);
		if (ctx->b_firstseq) {
			const struct v4l2_event ev = {
				.type = V4L2_EVENT_SOURCE_CHANGE,
				.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION
			};
			v4l2_event_queue_fh(&ctx->fh, &ev);

			pStreamPitchInfo->uFramePitch = 0x4000;
			ctx->b_firstseq = false;
		}
		}
		break;
	case VID_API_EVENT_PIC_HDR_FOUND:
		break;
	case VID_API_EVENT_REQ_FRAME_BUFF: {
		MEDIA_PLAYER_FSREQ *pFSREQ = (MEDIA_PLAYER_FSREQ *)event_data;
		u_int32 local_cmddata[10];
		struct vb2_data_req *p_data_req, *p_temp;
		struct queue_data *This = &ctx->q_data[V4L2_DST];
		u_int32 LumaAddr, ChromaAddr;
		u_int32 *pphy_address;
		struct vb2_data_req;
		void *dcp_dma_virt;
		dma_addr_t dcp_dma_phy;
		bool buffer_flag = false;

		vpu_dbg(LVL_INFO, "VID_API_EVENT_REQ_FRAME_BUFF, type=%d, size=%ld\n", pFSREQ->eType, sizeof(MEDIA_PLAYER_FSREQ));
		if (pFSREQ->eType == MEDIAIP_DCP_REQ) {
			if (ctx->dcp_count >= MAX_DCP_NUM) {
				vpu_dbg(LVL_ERR, "error: request dcp buffers number is over MAX_DCP_NUM\n");
				break;
			}
			ctx->uDCPSize = DCP_SIZE;
			dcp_dma_virt = dma_alloc_coherent(&ctx->dev->plat_dev->dev,
					ctx->uDCPSize,
					(dma_addr_t *)&dcp_dma_phy,
					GFP_KERNEL | GFP_DMA32
					);
			if (!dcp_dma_virt)
				vpu_dbg(LVL_ERR, "error: %s() dcp buffer alloc size(%x) fail!\n", __func__, DCP_SIZE);
			ctx->dcp_dma_virt[ctx->dcp_count] = dcp_dma_virt;
			ctx->dcp_dma_phy[ctx->dcp_count] = dcp_dma_phy;

			local_cmddata[0] = ctx->dcp_count;
			local_cmddata[1] = dcp_dma_phy - ctx->dev->cm_offset;
			local_cmddata[2] = DCP_SIZE;
			local_cmddata[3] = 0;
			local_cmddata[4] = 0;
			local_cmddata[5] = 0;
			local_cmddata[6] = pFSREQ->eType;
			v4l2_vpu_send_cmd(ctx, uStrIdx, VID_API_CMD_FS_ALLOC, 7, local_cmddata);
			vpu_dbg(LVL_INFO, "VID_API_CMD_FS_ALLOC, eType=%d, index=%d\n", pFSREQ->eType, ctx->dcp_count);
			ctx->dcp_count++;
		} else if (pFSREQ->eType == MEDIAIP_MBI_REQ) {
			if (ctx->mbi_count >= ctx->mbi_num) {
				vpu_dbg(LVL_ERR, "error: request mbi buffers number(%d) is over allocted buffer number(%d)\n",
						ctx->mbi_count, ctx->mbi_num);
				break;
			}
			local_cmddata[0] = ctx->mbi_count;
			local_cmddata[1] = ctx->mbi_dma_phy[ctx->mbi_count] - ctx->dev->cm_offset;
			local_cmddata[2] = ctx->mbi_size;
			local_cmddata[3] = 0;
			local_cmddata[4] = 0;
			local_cmddata[5] = 0;
			local_cmddata[6] = pFSREQ->eType;
			v4l2_vpu_send_cmd(ctx, uStrIdx, VID_API_CMD_FS_ALLOC, 7, local_cmddata);
			vpu_dbg(LVL_INFO, "VID_API_CMD_FS_ALLOC, eType=%d, index=%d\n", pFSREQ->eType, ctx->mbi_count);
			ctx->mbi_count++;
		} else {
#if 1
			while (1) {
				if (!wait_event_interruptible_timeout(ctx->buffer_wq,
							((ctx->wait_rst_done == true) || (wait_right_buffer(This) == true)),
							msecs_to_jiffies(5000)))
					vpu_dbg(LVL_ERR, " warn: wait_event_interruptible_timeout wait 5s timeout\n");
				else
					break;
			}
#endif

			if (ctx->buffer_null == true) {
				vpu_dbg(LVL_INFO, "frame already released\n");
				break;
			}

			if (!list_empty(&This->drv_q)) {
				down(&This->drv_q_lock);
				list_for_each_entry_safe(p_data_req, p_temp, &This->drv_q, list) {
					if (p_data_req->status == FRAME_ALLOC
							|| p_data_req->status == FRAME_RELEASE){
						pphy_address = (u_int32 *)vb2_plane_cookie(p_data_req->vb2_buf, 0);
						LumaAddr = *pphy_address;
						pphy_address = (u_int32 *)vb2_plane_cookie(p_data_req->vb2_buf, 1);
						ChromaAddr = *pphy_address;
						vpu_dbg(LVL_INFO, "%s :LumaAddr(%x) ChromaAddr(%x) buf_id (%d)\n",
								__func__,
								LumaAddr,
								ChromaAddr,
								p_data_req->id
								);

						local_cmddata[0] = p_data_req->id;
						local_cmddata[1] = LumaAddr - ctx->dev->cm_offset;
						local_cmddata[2] = local_cmddata[1] + This->sizeimage[0]/2;
						local_cmddata[3] = ChromaAddr - ctx->dev->cm_offset;
						local_cmddata[4] = local_cmddata[3] + This->sizeimage[1]/2;
						local_cmddata[5] = ctx->q_data[V4L2_DST].stride;
						local_cmddata[6] = pFSREQ->eType;
						//WARN :need to check the call back VID_API_EVENT_REL_FRAME_BUFF later, when it is received, the corepond id can be released, now just do a temporary workaround
						if (p_data_req->status == FRAME_RELEASE)
							v4l2_vpu_send_cmd(ctx, uStrIdx, VID_API_CMD_FS_RELEASE, 1, &p_data_req->id);
						v4l2_vpu_send_cmd(ctx, uStrIdx, VID_API_CMD_FS_ALLOC, 7, local_cmddata);
						p_data_req->status = FRAME_FREE;
						vpu_dbg(LVL_INFO, "VID_API_CMD_FS_ALLOC, data_req->vb2_buf=%p, data_req->id=%d\n", p_data_req->vb2_buf, p_data_req->id);
						list_del(&p_data_req->list);
						buffer_flag = true;
						break;
					} else {
						vpu_dbg(LVL_INFO, "buffer %d status=0x%x is not right, find next\n", p_data_req->id, p_data_req->status);
						continue;
					}
				}
				up(&This->drv_q_lock);
				if (buffer_flag == false)
					vpu_dbg(LVL_ERR, "error: don't find the right buffer for VID_API_CMD_FS_ALLOC\n");
			} else if (ctx->wait_rst_done) {
				u_int32 i;

				for (i = 0; i < VPU_MAX_BUFFER; i++) {
					p_data_req = &This->vb2_reqs[i];
					if (p_data_req->status == FRAME_RELEASE)
						break;
				}
				if (i == VPU_MAX_BUFFER) {
					vpu_dbg(LVL_ERR, "error: don't find buffer when wait_rst_done is true\n"); //wait_rst_done is true when streamoff or v4l2_release is called
					break;
				}

				pphy_address = (u_int32 *)vb2_plane_cookie(p_data_req->vb2_buf, 0);
				LumaAddr = *pphy_address;
				pphy_address = (u_int32 *)vb2_plane_cookie(p_data_req->vb2_buf, 1);
				ChromaAddr = *pphy_address;
				vpu_dbg(LVL_INFO, "%s :LumaAddr(%x) ChromaAddr(%x) buf_id (%d)\n",
						__func__,
						LumaAddr,
						ChromaAddr,
						p_data_req->id
						);
				local_cmddata[0] = p_data_req->id;
				local_cmddata[1] = LumaAddr - ctx->dev->cm_offset;
				local_cmddata[2] = local_cmddata[1] + This->sizeimage[0]/2;
				local_cmddata[3] = ChromaAddr - ctx->dev->cm_offset;
				local_cmddata[4] = local_cmddata[3] + This->sizeimage[1]/2;
				local_cmddata[5] = ctx->q_data[V4L2_DST].stride;
				local_cmddata[6] = pFSREQ->eType;
				//WARN :need to check the call back VID_API_EVENT_REL_FRAME_BUFF later, when it is received, the corepond id can be released, now just do a temporary workaround
				if (p_data_req->status == FRAME_RELEASE)
					v4l2_vpu_send_cmd(ctx, uStrIdx, VID_API_CMD_FS_RELEASE, 1, &p_data_req->id);
				v4l2_vpu_send_cmd(ctx, uStrIdx, VID_API_CMD_FS_ALLOC, 7, local_cmddata);
				p_data_req->status = FRAME_FREE;
				vpu_dbg(LVL_INFO, "VID_API_CMD_FS_ALLOC, data_req->vb2_buf=%p, data_req->id=%d\n", p_data_req->vb2_buf, p_data_req->id);
			} else
				vpu_dbg(LVL_ERR, "error: the list is still empty");
		}
		}
		break;
	case VID_API_EVENT_REL_FRAME_BUFF: {
		MEDIA_PLAYER_FSREL *fsrel = (MEDIA_PLAYER_FSREL *)event_data;
		struct queue_data *This = &ctx->q_data[V4L2_DST];
		struct vb2_data_req *p_data_req;

		if (ctx->buffer_null == true) {
			vpu_dbg(LVL_INFO, "frame already released !!!!!!!!!!!!!!!!!\n");
			break;
		}

		if (fsrel->eType == MEDIAIP_FRAME_REQ) {
			p_data_req = &This->vb2_reqs[fsrel->uFSIdx];

			if (ctx->wait_rst_done != true && p_data_req->status != FRAME_READY)
				vpu_dbg(LVL_ERR, "error: normal release need to set status to FRAME_RELEASE but previous status %s is not FRAME_READY\n", bufstat[p_data_req->status]);
			p_data_req->status = FRAME_RELEASE;
		}
		vpu_dbg(LVL_INFO, "VID_API_EVENT_REL_FRAME_BUFF uFSIdx=%d, eType=%d, size=%ld\n", fsrel->uFSIdx, fsrel->eType, sizeof(MEDIA_PLAYER_FSREL));
	} break;
	case VID_API_EVENT_FRAME_BUFF_RDY: {
		u_int32 *FrameInfo = (u_int32 *)event_data;

		//when the buffer is not NULL, do report_buffer_done
		if (ctx->buffer_null == false)
			report_buffer_done(ctx, FrameInfo);
	}
		break;
	case VID_API_EVENT_CHUNK_DECODED:
		break;
	case VID_API_EVENT_FIFO_LOW: {
		struct vpu_dev *dev = ctx->dev;
		u_int32 uStrBufIdx = 0; //use buffer 0 for the stream
		pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;

		if (ctx->buffer_null == true) {
			vpu_dbg(LVL_INFO, "frame already released !!!!!!!!!!!!!!!!!\n");
			break;
		}
		pStrBufDesc = dev->regs_base + DEC_MFD_XREG_SLV_BASE + MFD_MCX + MFD_MCX_OFF * ctx->str_index;
		if (ctx->stream_feed_complete) {
			vpu_dbg(LVL_INFO, "%s - VID_API_EVENT_FIFO_LOW - Before wptr(%x) rptr(%x) start(%x) end(%x) uStrIdx(%d)\n",
				__func__, pStrBufDesc->wptr, pStrBufDesc->rptr, pStrBufDesc->start, pStrBufDesc->end, uStrIdx);
			vpu_dbg(LVL_INFO, "VID_API_EVENT_FIFO_LOW - ctx->stream_feed_complete = true - add_eos\n");
			// Indicate stop added so that we respond on a FINISHED event
			ctx->eos_stop_added = true;
			// Set ctx->stream_feed_complete = false so that we don't try
			// to insert another EOS on the next VID_API_EVENT_FIFO_LOW event
			ctx->stream_feed_complete = false;
			v4l2_update_stream_addr(ctx, uStrBufIdx);
			add_eos(ctx, 0);
			complete(&ctx->eos_cmp);
			vpu_dbg(LVL_INFO, "%s - VID_API_EVENT_FIFO_LOW - After wptr(%x) rptr(%x) start(%x) end(%x) uStrIdx(%d)\n",
				__func__, pStrBufDesc->wptr, pStrBufDesc->rptr, pStrBufDesc->start, pStrBufDesc->end, uStrIdx);
		} else {
			v4l2_update_stream_addr(ctx, uStrBufIdx);
		}
	} break;
	case VID_API_EVENT_FIFO_HIGH:
		break;
	case  VID_API_EVENT_FIFO_EMPTY:
		break;
	case  VID_API_EVENT_FIFO_FULL:
		break;
	case  VID_API_EVENT_FIFO_OVF:
		break;
	case  VID_API_EVENT_BS_ERROR:
		break;
	case  VID_API_EVENT_UDATA_FIFO_UPTD:
		break;
	case VID_API_EVENT_DBG_STAT_UPDATE:
		break;
	case VID_API_EVENT_DBG_LOG_STARTED:
		break;
	case VID_API_EVENT_DBG_LOG_STOPPED:
		break;
	case VID_API_EVENT_ABORT_DONE: {
		pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;

		pStrBufDesc = dev->regs_base + DEC_MFD_XREG_SLV_BASE + MFD_MCX + MFD_MCX_OFF * ctx->str_index;
		vpu_dbg(LVL_INFO, "%s AbrtDone StrBuf Curr, wptr(%x) rptr(%x) start(%x) end(%x)\n",
				__func__,
				pStrBufDesc->wptr,
				pStrBufDesc->rptr,
				pStrBufDesc->start,
				pStrBufDesc->end
				);
		pStrBufDesc->wptr = pStrBufDesc->rptr;
		v4l2_vpu_send_cmd(ctx, uStrIdx, VID_API_CMD_RST_BUF, 0, NULL);
		}
		break;
	case VID_API_EVENT_RES_CHANGE:
		{
			const struct v4l2_event ev = {
			.type = V4L2_EVENT_SOURCE_CHANGE,
			.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION
		};
		v4l2_event_queue_fh(&ctx->fh, &ev);
		}
		break;
	case VID_API_EVENT_STR_BUF_RST: {
		pSTREAM_BUFFER_DESCRIPTOR_TYPE pStrBufDesc;
		struct vb2_data_req *p_data_req;
		u_int32 i;

		pStrBufDesc = dev->regs_base + DEC_MFD_XREG_SLV_BASE + MFD_MCX + MFD_MCX_OFF * ctx->str_index;
		vpu_dbg(LVL_INFO, "%s wptr(%x) rptr(%x) start(%x) end(%x)\n",
				__func__,
				pStrBufDesc->wptr,
				pStrBufDesc->rptr,
				pStrBufDesc->start,
				pStrBufDesc->end
			  );
		ctx->wait_rst_done = false;
		for (i = 0; i < VPU_MAX_BUFFER; i++) {
			p_data_req = &ctx->q_data[V4L2_DST].vb2_reqs[i];
			if (p_data_req->vb2_buf != NULL)
				if (p_data_req->status != FRAME_RELEASE)
					vpu_dbg(LVL_INFO, "buffer(%d) status is %s when receive VID_API_EVENT_STR_BUF_RST\n", i, bufstat[p_data_req->status]);
		}
		complete(&ctx->completion);
		}
		break;
	case VID_API_EVENT_RET_PING:
		break;
	case VID_API_EVENT_STR_FMT_CHANGE:
		break;
	case VID_API_EVENT_FINISHED: {
		const struct v4l2_event ev = {
			.type = V4L2_EVENT_EOS
		};

		if (ctx->eos_stop_added) {
			if (ctx->firmware_finished == false) {
				vpu_dbg(LVL_INFO, "receive VID_API_EVENT_FINISHED\n");
				ctx->firmware_finished = true;
				v4l2_event_queue_fh(&ctx->fh, &ev); //notfiy app stream eos reached
			} else	{
				vpu_dbg(LVL_ERR, "receive VID_API_EVENT_FINISHED when ctx->firmware_finished == true - IGNORE\n");
			}
		} else {
			vpu_dbg(LVL_ERR, "receive VID_API_EVENT_FINISHED before eos_stop_added set - IGNORE\n");
		}
	}	break;
	default:
		break;
	}
	vpu_dbg(LVL_INFO, "leave %s, uEvent %d\n", __func__, uEvent);
}



//This code is added for MU

static irqreturn_t fsl_vpu_mu_isr(int irq, void *This)
{
	struct vpu_dev *dev = This;
	u32 msg;

	MU_ReceiveMsg(dev->mu_base_virtaddr, 0, &msg);
	if (msg == 0xaa) {
#ifdef CM4
		MU_sendMesgToFW(dev->mu_base_virtaddr, RPC_BUF_OFFSET, dev->m0_rpc_phy); //CM4 use absolute address
#else
		MU_sendMesgToFW(dev->mu_base_virtaddr, PRINT_BUF_OFFSET, dev->m0_rpc_phy - dev->m0_p_fw_space_phy + M0_PRINT_OFFSET);
		MU_sendMesgToFW(dev->mu_base_virtaddr, RPC_BUF_OFFSET, dev->m0_rpc_phy - dev->m0_p_fw_space_phy); //CM0 use relative address
		MU_sendMesgToFW(dev->mu_base_virtaddr, BOOT_ADDRESS, dev->m0_p_fw_space_phy);
#endif
		MU_sendMesgToFW(dev->mu_base_virtaddr, INIT_DONE, 2);

	} else if (msg == 0x55) {
		dev->firmware_started = true;
		complete(&dev->start_cmp);
	}  else if (msg == 0xA5) {
		/*receive snapshot done msg and wakeup complete to suspend*/
		complete(&dev->snap_done_cmp);
	} else
		schedule_work(&dev->msg_work);

	return IRQ_HANDLED;
}

/* Initialization of the MU code. */
static int vpu_mu_init(struct vpu_dev *dev)
{
	struct device_node *np;
	unsigned int	vpu_mu_id;
	u32 irq;
	int ret = 0;

	/*
	 * Get the address of MU to be used for communication with the M0 core
	 */
#ifdef CM4
	np = of_find_compatible_node(NULL, NULL, "fsl,imx8-mu0-vpu-m4");
	if (!np) {
		vpu_dbg(LVL_ERR, "error: Cannot find MU entry in device tree\n");
		return -EINVAL;
	}
#else
	np = of_find_compatible_node(NULL, NULL, "fsl,imx8-mu0-vpu-m0");
	if (!np) {
		vpu_dbg(LVL_ERR, "error: Cannot find MU entry in device tree\n");
		return -EINVAL;
	}
#endif
	dev->mu_base_virtaddr = of_iomap(np, 0);
	WARN_ON(!dev->mu_base_virtaddr);

	ret = of_property_read_u32_index(np,
				"fsl,vpu_ap_mu_id", 0, &vpu_mu_id);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: Cannot get mu_id %d\n", ret);
		return -EINVAL;
	}

	dev->vpu_mu_id = vpu_mu_id;

	irq = of_irq_get(np, 0);

	ret = devm_request_irq(&dev->plat_dev->dev, irq, fsl_vpu_mu_isr,
				IRQF_EARLY_RESUME, "vpu_mu_isr", (void *)dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: request_irq failed %d, error = %d\n", irq, ret);
		return -EINVAL;
	}

	if (!dev->vpu_mu_init) {
		MU_Init(dev->mu_base_virtaddr);
		MU_EnableRxFullInt(dev->mu_base_virtaddr, 0);
		dev->vpu_mu_init = 1;
	}

	return ret;
}

/*
 * Add judge to find if it has available path to decode, if all
 * path hang, reset vpu and then get one index
 */
static int vpu_next_free_instance(struct vpu_dev *dev)
{
	int count = 0;
	unsigned long hang_mask = dev->hang_mask;
	int idx;

	while (hang_mask) {
		if (hang_mask & 1)
			count++;
		hang_mask >>= 1;
	}
	if (count == VPU_MAX_NUM_STREAMS) {
		dev->hang_mask = 0;
		dev->instance_mask = 0;
		reset_vpu_firmware(dev);
	}

	idx = ffz(dev->instance_mask);
	if (idx < 0 || idx >= VPU_MAX_NUM_STREAMS)
		return -EBUSY;

	return idx;
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

extern u_int32 rpc_MediaIPFW_Video_message_check(struct shared_addr *This);
static void vpu_msg_run_work(struct work_struct *work)
{
	struct vpu_dev *dev = container_of(work, struct vpu_dev, msg_work);
	struct vpu_ctx *ctx;
	struct event_msg msg;
	struct shared_addr *This = &dev->shared_mem;

	while (rpc_MediaIPFW_Video_message_check(This) == API_MSG_AVAILABLE) {
		rpc_receive_msg_buf(This, &msg);
		mutex_lock(&dev->dev_mutex);
		ctx = dev->ctx[msg.idx];
		if (ctx != NULL) {
			mutex_lock(&ctx->instance_mutex);
			if (!ctx->ctx_released) {
				send_msg_queue(ctx, &msg);
				queue_work(ctx->instance_wq, &ctx->instance_work);
			}
			mutex_unlock(&ctx->instance_mutex);
		}
		mutex_unlock(&dev->dev_mutex);
	}
	if (rpc_MediaIPFW_Video_message_check(This) == API_MSG_BUFFER_ERROR)
		vpu_dbg(LVL_ERR, "error: message size is too big to handle\n");
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

	vpu_dbg(LVL_INFO, "%s() is called\n", __func__);

	if ((vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) ||
		(vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		) {
		if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
			*plane_count = 2;
			psize[0] = This->sizeimage[0];//check alignment
			psize[1] = This->sizeimage[1];//check colocated_size
		} else {
			psize[0] = This->sizeimage[0] + This->sizeimage[1];
			*plane_count = 1;
		}
	} else {
		*plane_count = 1;
		psize[0] = This->sizeimage[0];
	}
	return 0;
}

static int vpu_buf_prepare(struct vb2_buffer *vb)
{
	vpu_dbg(LVL_INFO, "%s() is called\n", __func__);

	return 0;
}


static int vpu_start_streaming(struct vb2_queue *q,
		unsigned int count
		)
{
	int ret = 0;

	vpu_dbg(LVL_INFO, "%s() is called\n", __func__);
	return ret;
}


static void vpu_stop_streaming(struct vb2_queue *q)
{
	struct queue_data *This = (struct queue_data *)q->drv_priv;
	struct vb2_data_req *p_data_req = NULL;
	struct vb2_data_req *p_temp;
	struct vb2_buffer *vb;

	vpu_dbg(LVL_INFO, "%s() is called\n", __func__);
	down(&This->drv_q_lock);
	if (!list_empty(&This->drv_q)) {
		list_for_each_entry_safe(p_data_req, p_temp, &This->drv_q, list) {
			vpu_dbg(LVL_INFO, "%s(%d) - list_del(%p)\n",
					__func__,
					p_data_req->id,
					p_data_req);
			list_del(&p_data_req->list);
		}
	}
	if (!list_empty(&q->queued_list))
		list_for_each_entry(vb, &q->queued_list, queued_entry) {
			if (vb->state == VB2_BUF_STATE_ACTIVE)
				vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		}
	INIT_LIST_HEAD(&This->drv_q);
	up(&This->drv_q_lock);
}

static void vpu_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue    *vq = vb->vb2_queue;
	struct queue_data   *This = (struct queue_data *)vq->drv_priv;
	struct vb2_data_req *data_req;

	vpu_dbg(LVL_INFO, "%s() is called\n", __func__);

	down(&This->drv_q_lock);
	vpu_dbg(LVL_INFO, "c_port_buf_queue down\n");
	data_req = &This->vb2_reqs[vb->index];
	data_req->vb2_buf = vb;
	data_req->id = vb->index;

	if (data_req->status != FRAME_FREE && data_req->status != FRAME_DECODED)
		list_add_tail(&data_req->list, &This->drv_q);

	vpu_dbg(LVL_INFO, "before c_port_buf_queue up, vq->type=%d, vb->index=%d\n", vq->type, vb->index);
	up(&This->drv_q_lock);
	vpu_dbg(LVL_INFO, "c_port_buf_queue up\n");

	if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		v4l2_transfer_buffer_to_firmware(This, vb);
}

static void vpu_prepare(struct vb2_queue *q)
{
	vpu_dbg(LVL_INFO, "%s() is called\n", __func__);
}

static void vpu_finish(struct vb2_queue *q)
{
	vpu_dbg(LVL_INFO, "%s() is called\n", __func__);
}

struct vb2_ops v4l2_qops = {
	.queue_setup        = vpu_queue_setup,
	.wait_prepare       = vpu_prepare,
	.wait_finish        = vpu_finish,
	.buf_prepare        = vpu_buf_prepare,
	.start_streaming    = vpu_start_streaming,
	.stop_streaming     = vpu_stop_streaming,
	.buf_queue          = vpu_buf_queue,
};

static void init_vb2_queue(struct queue_data *This, unsigned int type, struct vpu_ctx *ctx)
{
	struct vb2_queue  *vb2_q = &This->vb2_q;
	int ret;
	u_int32 i;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	for (i = 0; i < VPU_MAX_BUFFER; i++)
		This->vb2_reqs[i].status = 0;
	// initialze driver queue
	INIT_LIST_HEAD(&This->drv_q);
	// initialize vb2 queue
	vb2_q->type = type;
	vb2_q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	vb2_q->gfp_flags = GFP_DMA32;
	vb2_q->ops = &v4l2_qops;
	vb2_q->drv_priv = This;
	vb2_q->mem_ops = (struct vb2_mem_ops *)&vb2_dma_contig_memops;
	vb2_q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	vb2_q->dev = &ctx->dev->plat_dev->dev;
	ret = vb2_queue_init(vb2_q);
	if (ret)
		vpu_dbg(LVL_ERR, "error: %s vb2_queue_init() failed (%d)!\n",
				__func__,
				ret
				);
	else
		This->vb2_q_inited = true;
}

static void init_queue_data(struct vpu_ctx *ctx)
{
	init_vb2_queue(&ctx->q_data[V4L2_SRC], V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, ctx);
	ctx->q_data[V4L2_SRC].type = V4L2_SRC;
	sema_init(&ctx->q_data[V4L2_SRC].drv_q_lock, 1);
	init_vb2_queue(&ctx->q_data[V4L2_DST], V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, ctx);
	ctx->q_data[V4L2_DST].type = V4L2_DST;
	sema_init(&ctx->q_data[V4L2_DST].drv_q_lock, 1);
}

static void flush_drv_q(struct queue_data *This)
{
	struct vb2_data_req *p_data_req = NULL;
	struct vb2_data_req *p_temp;

	vpu_dbg(LVL_INFO, "%s() is called\n", __func__);
	down(&This->drv_q_lock);
	if (!list_empty(&This->drv_q)) {
		list_for_each_entry_safe(p_data_req, p_temp, &This->drv_q, list) {
			vpu_dbg(LVL_INFO, "%s(%d) - list_del(%p)\n",
					__func__,
					p_data_req->id,
					p_data_req);
			list_del(&p_data_req->list);
			vb2_buffer_done(p_data_req->vb2_buf, VB2_BUF_STATE_ERROR);
		}
	}
	INIT_LIST_HEAD(&This->drv_q);

	up(&This->drv_q_lock);

}

static void release_queue_data(struct vpu_ctx *ctx)
{
	struct queue_data *This = &ctx->q_data[V4L2_SRC];

	if (This->vb2_q_inited) {
		flush_drv_q(This);
		vb2_queue_release(&This->vb2_q);
	}
	This = &ctx->q_data[V4L2_DST];
	if (This->vb2_q_inited) {
		flush_drv_q(This);
		vb2_queue_release(&This->vb2_q);
	}
}

#ifdef CM4
static int power_CM4_up(struct vpu_dev *dev)
{
	sc_ipc_t ipcHndl;
	sc_rsrc_t core_rsrc, mu_rsrc = -1;

	ipcHndl = dev->mu_ipcHandle;
	core_rsrc = SC_R_M4_0_PID0;
	mu_rsrc = SC_R_M4_0_MU_1A;

	if (sc_pm_set_resource_power_mode(ipcHndl, core_rsrc, SC_PM_PW_MODE_ON) != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "error: failed to power up core_rsrc\n");
		return -EIO;
	}

	if (mu_rsrc != -1) {
		if (sc_pm_set_resource_power_mode(ipcHndl, mu_rsrc, SC_PM_PW_MODE_ON) != SC_ERR_NONE) {
			vpu_dbg(LVL_ERR, "error: failed to power up mu_rsrc\n");
			return -EIO;
		}
	}

	return 0;
}

static int boot_CM4_up(struct vpu_dev *dev, void *boot_addr)
{
	sc_ipc_t ipcHndl;
	sc_rsrc_t core_rsrc;
	sc_faddr_t aux_core_ram;
	void *core_ram_vir;
	u32 size;

	ipcHndl = dev->mu_ipcHandle;
	core_rsrc = SC_R_M4_0_PID0;
	aux_core_ram = 0x34FE0000;
	size = SZ_128K;

	core_ram_vir = ioremap_wc(aux_core_ram,
			size
			);
	if (!core_ram_vir)
		vpu_dbg(LVL_ERR, "error: failed to remap space for core ram\n");

	memcpy((void *)core_ram_vir, (void *)boot_addr, size);

	if (sc_pm_cpu_start(ipcHndl, core_rsrc, true, aux_core_ram) != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "error: failed to start core_rsrc\n");
		return -EIO;
	}

	return 0;
}
#endif

static int vpu_firmware_download(struct vpu_dev *This)
{
	unsigned char *image;
	unsigned int FW_Size = 0;
	void *csr_offset, *csr_cpuwait;
	int ret = 0;

	ret = request_firmware((const struct firmware **)&This->m0_pfw,
			M0FW_FILENAME,
			This->generic_dev
			);

	if (ret) {
		vpu_dbg(LVL_ERR, "error: %s() request fw %s failed(%d)\n",
			__func__, M0FW_FILENAME, ret);

		if (This->m0_pfw) {
			release_firmware(This->m0_pfw);
			This->m0_pfw = NULL;
		}
		return ret;
	} else {
		vpu_dbg(LVL_INFO, "%s() request fw %s got size(%d)\n",
			__func__, M0FW_FILENAME, (int)This->m0_pfw->size);
		image = (uint8_t *)This->m0_pfw->data;
		FW_Size = This->m0_pfw->size;
	}
	memcpy(This->m0_p_fw_space_vir,
			image,
			FW_Size
			);
#ifdef CM4
	boot_CM4_up(This, This->m0_p_fw_space_vir);
#else
	csr_offset = ioremap(0x2d040000, 4);
	writel(This->m0_p_fw_space_phy, csr_offset);
	csr_cpuwait = ioremap(0x2d040004, 4);
	writel(0x0, csr_cpuwait);
#endif
	return ret;
}

static int v4l2_open(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct vpu_dev *dev = video_get_drvdata(vdev);
	struct vpu_ctx *ctx = NULL;
	int idx;
	int ret;
	u_int32 i;

	pm_runtime_get_sync(dev->generic_dev);
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	mutex_lock(&dev->dev_mutex);
	idx = vpu_next_free_instance(dev);
	if (idx < 0) {
		ret = idx;
		mutex_unlock(&dev->dev_mutex);
		goto err_find_index;
	}
	set_bit(idx, &dev->instance_mask);
	mutex_unlock(&dev->dev_mutex);
	init_completion(&ctx->completion);
	init_completion(&ctx->stop_cmp);
	init_completion(&ctx->eos_cmp);

	v4l2_fh_init(&ctx->fh, video_devdata(filp));
	filp->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

	ctx->ctrl_inited = false;
	ctrls_setup_decoder(ctx);
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;


	ctx->instance_wq = alloc_workqueue("vpu_instance", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!ctx->instance_wq) {
		vpu_dbg(LVL_ERR, "error: %s unable to alloc workqueue for ctx\n", __func__);
		ret = -ENOMEM;
		goto err_alloc;
	}
	INIT_WORK(&ctx->instance_work, vpu_msg_instance_work);

	mutex_init(&ctx->instance_mutex);
	if (kfifo_alloc(&ctx->msg_fifo,
				sizeof(struct event_msg) * VID_API_MESSAGE_LIMIT,
				GFP_KERNEL)) {
		vpu_dbg(LVL_ERR, "fail to alloc fifo when open\n");
		ret = -ENOMEM;
		goto err_alloc;
	}
	ctx->dev = dev;
	ctx->str_index = idx;
	dev->ctx[idx] = ctx;
	ctx->b_firstseq = true;
	ctx->start_flag = true;
	ctx->wait_rst_done = false;
	ctx->firmware_stopped = false;
	ctx->firmware_finished = false;
	ctx->eos_stop_added    = false;
	ctx->stream_feed_complete = false;
	ctx->buffer_null = true; //this flag is to judge whether the buffer is null is not, it is used for the workaround that when send stop command still can receive buffer ready event, and true means buffer is null, false not
	ctx->ctx_released = false;
	ctx->pSeqinfo = kzalloc(sizeof(MediaIPFW_Video_SeqInfo), GFP_KERNEL);
	if (!ctx->pSeqinfo)
		vpu_dbg(LVL_ERR, "error: pSeqinfo alloc fail\n");
	init_queue_data(ctx);
	init_waitqueue_head(&ctx->buffer_wq);
	mutex_lock(&dev->dev_mutex);
	if (!dev->fw_is_ready) {
		ret = vpu_firmware_download(dev);
		if (ret) {
			vpu_dbg(LVL_ERR, "error: vpu_firmware_download fail\n");
			mutex_unlock(&dev->dev_mutex);
			goto err_firmware_load;
		} else
			vpu_dbg(LVL_INFO, "done: vpu_firmware_download\n");
		if (!ctx->dev->firmware_started)
			wait_for_completion(&ctx->dev->start_cmp);
		dev->fw_is_ready = true;
	}
	mutex_unlock(&dev->dev_mutex);

	rpc_set_stream_cfg_value(dev->shared_mem.pSharedInterface, ctx->str_index);
	for (i = 0; i < MAX_DCP_NUM; i++) {
		ctx->dcp_dma_virt[i] = NULL;
		ctx->dcp_dma_phy[i] = 0;
	}
	ctx->dcp_count = 0;
	for (i = 0; i < MAX_MBI_NUM; i++) {
		ctx->mbi_dma_virt[i] = NULL;
		ctx->mbi_dma_phy[i] = 0;
	}
	ctx->mbi_count = 0;
	ctx->mbi_num = 0;
	ctx->mbi_size = 0;
	ctx->stream_buffer_size = MAX_BUFFER_SIZE;
	ctx->stream_buffer_virt = dma_alloc_coherent(&ctx->dev->plat_dev->dev,
			ctx->stream_buffer_size,
			(dma_addr_t *)&ctx->stream_buffer_phy,
			GFP_KERNEL | GFP_DMA32
			);
	if (!ctx->stream_buffer_virt)
		vpu_dbg(LVL_ERR, "error: %s() stream buffer alloc size(%x) fail!\n", __func__, ctx->stream_buffer_size);
	else
		vpu_dbg(LVL_INFO, "%s() stream_buffer_size(%d) stream_buffer_virt(%p) stream_buffer_phy(%p), index(%d)\n",
				__func__, ctx->stream_buffer_size, ctx->stream_buffer_virt, (void *)ctx->stream_buffer_phy, ctx->str_index);
	ctx->udata_buffer_size = UDATA_BUFFER_SIZE;
	ctx->udata_buffer_virt = dma_alloc_coherent(&ctx->dev->plat_dev->dev,
			ctx->udata_buffer_size,
			(dma_addr_t *)&ctx->udata_buffer_phy,
			GFP_KERNEL | GFP_DMA32
			);

	if (!ctx->udata_buffer_virt)
		vpu_dbg(LVL_ERR, "error: %s() udata buffer alloc size(%x) fail!\n", __func__, ctx->udata_buffer_size);
	else
		vpu_dbg(LVL_INFO, "%s() udata_buffer_size(%d) udata_buffer_virt(%p) udata_buffer_phy(%p)\n",
				__func__, ctx->udata_buffer_size, ctx->udata_buffer_virt, (void *)ctx->udata_buffer_phy);

	return 0;

err_firmware_load:
	release_queue_data(ctx);
	ctrls_delete_decoder(ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	clear_bit(ctx->str_index, &dev->instance_mask);
	kfree(ctx);
	return ret;
err_find_index:
	pm_runtime_put_sync(dev->generic_dev);
	kfree(ctx);
	return ret;
err_alloc:
	pm_runtime_put_sync(dev->generic_dev);
	kfree(ctx);
	return ret;
}

static int v4l2_release(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct vpu_dev *dev = video_get_drvdata(vdev);
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(filp->private_data);
	u_int32 i;

	vpu_dbg(LVL_INFO, "v4l2_release() - stopped(%d), finished(%d), eos_added(%d)\n",
		ctx->firmware_stopped, ctx->firmware_finished, ctx->eos_stop_added);

	if (!ctx->firmware_stopped && ctx->start_flag == false) {
		ctx->wait_rst_done = true;
		wake_up_interruptible(&ctx->buffer_wq);  //workaround: to wakeup event handler who still may receive request frame after reset done
		vpu_dbg(LVL_INFO, "v4l2_release() - send VID_API_CMD_STOP\n");
		v4l2_vpu_send_cmd(ctx, ctx->str_index, VID_API_CMD_STOP, 0, NULL);
		if (!wait_for_completion_timeout(&ctx->stop_cmp, msecs_to_jiffies(1000))) {
			mutex_lock(&dev->dev_mutex);
			set_bit(ctx->str_index, &dev->hang_mask);
			mutex_unlock(&dev->dev_mutex);
			vpu_dbg(LVL_ERR, "the path id:%d firmware hang after send VID_API_CMD_STOP\n", ctx->str_index);
		}
	}

	release_queue_data(ctx);
	ctrls_delete_decoder(ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);

	for (i = 0; i < MAX_DCP_NUM; i++)
		if (ctx->dcp_dma_virt[i] != NULL)
		dma_free_coherent(&ctx->dev->plat_dev->dev,
				ctx->uDCPSize,
				ctx->dcp_dma_virt[i],
				ctx->dcp_dma_phy[i]
				);
	for (i = 0; i < MAX_MBI_NUM; i++)
		if (ctx->mbi_dma_virt[i] != NULL)
		dma_free_coherent(&ctx->dev->plat_dev->dev,
				ctx->mbi_size,
				ctx->mbi_dma_virt[i],
				ctx->mbi_dma_phy[i]
				);
	if (ctx->stream_buffer_virt)
		dma_free_coherent(&ctx->dev->plat_dev->dev,
				ctx->stream_buffer_size,
				ctx->stream_buffer_virt,
				ctx->stream_buffer_phy
				);
	if (ctx->udata_buffer_virt)
		dma_free_coherent(&ctx->dev->plat_dev->dev,
				ctx->udata_buffer_size,
				ctx->udata_buffer_virt,
				ctx->udata_buffer_phy
				);

	if (ctx->pSeqinfo) {
		kfree(ctx->pSeqinfo);
		ctx->pSeqinfo = NULL;
	}
	mutex_lock(&ctx->instance_mutex);
	ctx->ctx_released = true;
	kfifo_free(&ctx->msg_fifo);
	destroy_workqueue(ctx->instance_wq);
	mutex_unlock(&ctx->instance_mutex);
	ctx->stream_buffer_virt = NULL;
	mutex_lock(&dev->dev_mutex);
	if (!(dev->hang_mask & (1 << ctx->str_index))) // judge the path is hang or not, if hang, don't clear
		clear_bit(ctx->str_index, &dev->instance_mask);
	dev->ctx[ctx->str_index] = NULL;
	mutex_unlock(&dev->dev_mutex);

	pm_runtime_put_sync(ctx->dev->generic_dev);
	kfree(ctx);
	return 0;
}

static unsigned int v4l2_poll(struct file *filp, poll_table *wait)
{
	struct vpu_ctx *ctx = v4l2_fh_to_ctx(filp->private_data);
	struct vb2_queue *src_q, *dst_q;
	unsigned int rc = 0;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

	if (ctx) {
		poll_wait(filp, &ctx->fh.wait, wait);

		if (v4l2_event_pending(&ctx->fh)) {
			vpu_dbg(LVL_INFO, "%s() v4l2_event_pending\n", __func__);
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

	vpu_dbg(LVL_INFO, "%s()\n", __func__);

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

static const struct v4l2_file_operations v4l2_decoder_fops = {
	.owner = THIS_MODULE,
	.open  = v4l2_open,
	.unlocked_ioctl = video_ioctl2,
	.release = v4l2_release,
	.poll = v4l2_poll,
	.mmap = v4l2_mmap,
};

static struct video_device v4l2_videodevice_decoder = {
	.name   = "vpu decoder",
	.fops   = &v4l2_decoder_fops,
	.ioctl_ops = &v4l2_decoder_ioctl_ops,
	.vfl_dir = VFL_DIR_M2M,
};
#if 1
static int set_vpu_pwr(sc_ipc_t ipcHndl,
		sc_pm_power_mode_t pm
		)
{
	int rv = -1;
	sc_err_t sciErr;

	vpu_dbg(LVL_INFO, "%s()\n", __func__);
	if (!ipcHndl) {
		vpu_dbg(LVL_ERR, "error: --- set_vpu_pwr no IPC handle\n");
		goto set_vpu_pwrexit;
	}

	// Power on or off DEC, ENC MU
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU, pm);
	if (sciErr != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "error: --- sc_pm_set_resource_power_mode(SC_R_VPU,%d) SCI error! (%d)\n", sciErr, pm);
		goto set_vpu_pwrexit;
	}
#ifdef TEST_BUILD
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_DEC, pm);
	if (sciErr != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "error: --- sc_pm_set_resource_power_mode(SC_R_VPU_DEC,%d) SCI error! (%d)\n", sciErr, pm);
		goto set_vpu_pwrexit;
	}
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_ENC, pm);
	if (sciErr != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "error: --- sc_pm_set_resource_power_mode(SC_R_VPU_ENC,%d) SCI error! (%d)\n", sciErr, pm);
		goto set_vpu_pwrexit;
	}
#else
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_DEC_0, pm);
	if (sciErr != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "error: --- sc_pm_set_resource_power_mode(SC_R_VPU_DEC_0,%d) SCI error! (%d)\n", sciErr, pm);
		goto set_vpu_pwrexit;
	}
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_ENC_0, pm);
	if (sciErr != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "error: --- sc_pm_set_resource_power_mode(SC_R_VPU_ENC_0,%d) SCI error! (%d)\n", sciErr, pm);
		goto set_vpu_pwrexit;
	}
#endif
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_VPU_MU_0, pm);
	if (sciErr != SC_ERR_NONE) {
		vpu_dbg(LVL_ERR, "error: --- sc_pm_set_resource_power_mode(SC_R_VPU_MU_0,%d) SCI error! (%d)\n", sciErr, pm);
		goto set_vpu_pwrexit;
	}

	rv = 0;

set_vpu_pwrexit:
	return rv;
}

static void vpu_set_power(struct vpu_dev *dev, bool on)
{
	int ret;

	if (on) {
		ret = set_vpu_pwr(dev->mu_ipcHandle, SC_PM_PW_MODE_ON);
		if (ret)
			vpu_dbg(LVL_ERR, "error: failed to power on\n");
		pm_runtime_get_sync(dev->generic_dev);
	} else {
		pm_runtime_put_sync_suspend(dev->generic_dev);
		ret = set_vpu_pwr(dev->mu_ipcHandle, SC_PM_PW_MODE_OFF);
		if (ret)
			vpu_dbg(LVL_ERR, "error: failed to power off\n");
	}
}
#endif

static void vpu_setup(struct vpu_dev *This)
{
	uint32_t read_data = 0;

	vpu_dbg(LVL_INFO, "enter %s\n", __func__);
	writel(0x1, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_SCB_CLK_ENABLE_SET);
	writel(0xffffffff, This->regs_base + 0x70190);
	writel(0xffffffff, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_XMEM_RESET_SET);

	writel(0xE, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_SCB_CLK_ENABLE_SET);
	writel(0x7, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_CACHE_RESET_SET);

	writel(0x1f, This->regs_base + DEC_MFD_XREG_SLV_BASE + MFD_BLK_CTRL + MFD_BLK_CTRL_MFD_SYS_CLOCK_ENABLE_SET);
	writel(0xffffffff, This->regs_base + DEC_MFD_XREG_SLV_BASE + MFD_BLK_CTRL + MFD_BLK_CTRL_MFD_SYS_RESET_SET);

	writel(0x102, This->regs_base + XMEM_CONTROL);

	read_data = readl(This->regs_base+0x70108);
	vpu_dbg(LVL_IRQ, "%s read_data=%x\n", __func__, read_data);
}

static void vpu_reset(struct vpu_dev *This)
{
	vpu_dbg(LVL_INFO, "enter %s\n", __func__);
	writel(0x7, This->regs_base + SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_CACHE_RESET_CLR);
	writel(0xffffffff, This->regs_base + DEC_MFD_XREG_SLV_BASE + MFD_BLK_CTRL + MFD_BLK_CTRL_MFD_SYS_RESET_CLR);
}

static int vpu_enable_hw(struct vpu_dev *This)
{
	vpu_dbg(LVL_INFO, "%s()\n", __func__);
	This->vpu_clk = clk_get(&This->plat_dev->dev, "vpu_clk");
	if (IS_ERR(This->vpu_clk)) {
		vpu_dbg(LVL_ERR, "vpu_clk get error\n");
		return -ENOENT;
	}
	clk_set_rate(This->vpu_clk, 600000000);
	clk_prepare_enable(This->vpu_clk);
	vpu_setup(This);
	return 0;
}
static void vpu_disable_hw(struct vpu_dev *This)
{
	vpu_reset(This);
	if (This->vpu_clk) {
		clk_put(This->vpu_clk);
	}
}

static int reset_vpu_firmware(struct vpu_dev *dev)
{
	int ret = 0;

	vpu_dbg(LVL_ALL, "RESET: reset_vpu_firmware\n");
	vpu_set_power(dev, false);
	usleep_range(1000, 1100);
	vpu_set_power(dev, true);
	dev->fw_is_ready = false;
	dev->firmware_started = false;
	vpu_enable_hw(dev);

	rpc_init_shared_memory(&dev->shared_mem, dev->m0_rpc_phy - dev->m0_p_fw_space_phy, dev->m0_rpc_virt, SHARED_SIZE);
	rpc_set_system_cfg_value(dev->shared_mem.pSharedInterface, VPU_REG_BASE);

	MU_Init(dev->mu_base_virtaddr);
	MU_EnableRxFullInt(dev->mu_base_virtaddr, 0);

	return ret;
}

static int vpu_probe(struct platform_device *pdev)
{
	struct vpu_dev *dev;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *reserved_node;
	struct resource reserved_res;
	unsigned int mu_id;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->plat_dev = pdev;
	dev->generic_dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->regs_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->regs_base)) {
		vpu_dbg(LVL_ERR, "error: %s could not map regs_base\n", __func__);
		return PTR_ERR(dev->regs_base);
	}

	if (np) {
		reserved_node = of_parse_phandle(np, "boot-region", 0);
		if (!reserved_node) {
			vpu_dbg(LVL_ERR, "error: boot-region of_parse_phandle error\n");
			return -ENODEV;
		}

		if (of_address_to_resource(reserved_node, 0, &reserved_res)) {
			vpu_dbg(LVL_ERR, "error: boot-region of_address_to_resource error\n");
			return -EINVAL;
		}
		dev->m0_p_fw_space_phy = reserved_res.start;
		dev->cm_offset = 0;
		reserved_node = of_parse_phandle(np, "rpc-region", 0);
		if (!reserved_node) {
			vpu_dbg(LVL_ERR, "error: rpc-region of_parse_phandle error\n");
			return -ENODEV;
		}

		if (of_address_to_resource(reserved_node, 0, &reserved_res)) {
			vpu_dbg(LVL_ERR, "error: rpc-region of_address_to_resource error\n");
			return -EINVAL;
		}
		dev->m0_rpc_phy = reserved_res.start;
	} else
		vpu_dbg(LVL_ERR, "error: %s of_node is NULL\n", __func__);

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: %s unable to register v4l2 dev\n", __func__);
		return ret;
	}

	platform_set_drvdata(pdev, dev);

	dev->pvpu_decoder_dev = video_device_alloc();
	if (dev->pvpu_decoder_dev) {
		strncpy(dev->pvpu_decoder_dev->name, v4l2_videodevice_decoder.name, sizeof(v4l2_videodevice_decoder.name));
		dev->pvpu_decoder_dev->fops = v4l2_videodevice_decoder.fops;
		dev->pvpu_decoder_dev->ioctl_ops = v4l2_videodevice_decoder.ioctl_ops;
		dev->pvpu_decoder_dev->release = video_device_release;
		dev->pvpu_decoder_dev->vfl_dir = v4l2_videodevice_decoder.vfl_dir;
		dev->pvpu_decoder_dev->v4l2_dev = &dev->v4l2_dev;

		video_set_drvdata(dev->pvpu_decoder_dev, dev);

		if (video_register_device(dev->pvpu_decoder_dev,
					VFL_TYPE_GRABBER,
					-1)) {
			vpu_dbg(LVL_ERR, "error: %s unable to register video decoder device\n",
					__func__
					);
			video_device_release(dev->pvpu_decoder_dev);
			dev->pvpu_decoder_dev = NULL;
		} else {
			vpu_dbg(LVL_INFO, "%s  register video decoder device\n",
					__func__
				   );
		}
	}

	if (!dev->mu_ipcHandle) {
		ret = sc_ipc_getMuID(&mu_id);
		if (ret) {
			vpu_dbg(LVL_ERR, "error: --- sc_ipc_getMuID() cannot obtain mu id SCI error! (%d)\n", ret);
			return ret;
		}

		ret = sc_ipc_open(&dev->mu_ipcHandle, mu_id);
		if (ret) {
			vpu_dbg(LVL_ERR, "error: --- sc_ipc_getMuID() cannot open MU channel to SCU error! (%d)\n", ret);
			return ret;
		}
	}

	vpu_enable_hw(dev);

	mutex_init(&dev->dev_mutex);
	mutex_init(&dev->cmd_mutex);
	init_completion(&dev->start_cmp);
	init_completion(&dev->snap_done_cmp);
	dev->firmware_started = false;
	dev->hang_mask = 0;
	dev->instance_mask = 0;

	dev->fw_is_ready = false;
	dev->workqueue = alloc_workqueue("vpu", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!dev->workqueue) {
		vpu_dbg(LVL_ERR, "error: %s unable to alloc workqueue\n", __func__);
		ret = -ENOMEM;
		return ret;
	}

	INIT_WORK(&dev->msg_work, vpu_msg_run_work);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
#ifdef CM4
	ret = power_CM4_up(dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: failed to power on CM4\n");
		return ret;
	}
#endif
	ret = vpu_mu_init(dev);
	if (ret) {
		vpu_dbg(LVL_ERR, "error: %s vpu mu init failed\n", __func__);
		return ret;
	}

	//firmware space for M0
	dev->m0_p_fw_space_vir = ioremap_wc(dev->m0_p_fw_space_phy,
			M0_BOOT_SIZE
			);
	if (!dev->m0_p_fw_space_vir)
		vpu_dbg(LVL_ERR, "error: failed to remap space for M0 firmware\n");

	memset_io(dev->m0_p_fw_space_vir, 0, M0_BOOT_SIZE);

	dev->m0_rpc_virt = ioremap_wc(dev->m0_rpc_phy,
			SHARED_SIZE
			);
	if (!dev->m0_rpc_virt) {
		vpu_dbg(LVL_ERR, "error: failed to remap space for rpc shared memory\n");
		return -ENOMEM;
	}

	memset_io(dev->m0_rpc_virt, 0, SHARED_SIZE);
#ifdef CM4
	rpc_init_shared_memory(&dev->shared_mem, dev->m0_rpc_phy, dev->m0_rpc_virt, SHARED_SIZE);
#else
	rpc_init_shared_memory(&dev->shared_mem, dev->m0_rpc_phy - dev->m0_p_fw_space_phy, dev->m0_rpc_virt, SHARED_SIZE);
#endif
	rpc_set_system_cfg_value(dev->shared_mem.pSharedInterface, VPU_REG_BASE);

	pm_runtime_put_sync(&pdev->dev);

	return 0;
}

static int vpu_remove(struct platform_device *pdev)
{
	struct vpu_dev *dev = platform_get_drvdata(pdev);

	destroy_workqueue(dev->workqueue);
	if (dev->m0_p_fw_space_vir)
		iounmap(dev->m0_p_fw_space_vir);
	if (dev->m0_pfw) {
		release_firmware(dev->m0_pfw);
		dev->m0_pfw = NULL;
	}
	dev->m0_p_fw_space_vir = NULL;
	dev->m0_p_fw_space_phy = 0;
	dev->m0_rpc_virt = NULL;
	dev->m0_rpc_phy = 0;
	if (dev->shared_mem.shared_mem_vir)
		iounmap(dev->shared_mem.shared_mem_vir);
	dev->shared_mem.shared_mem_vir = NULL;
	dev->shared_mem.shared_mem_phy = 0;

	vpu_disable_hw(dev);
	pm_runtime_disable(&pdev->dev);

	if (video_get_drvdata(dev->pvpu_decoder_dev))
		video_unregister_device(dev->pvpu_decoder_dev);

	v4l2_device_unregister(&dev->v4l2_dev);
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

#define CHECK_BIT(var, pos) (((var) >> (pos)) & 1)

static void v4l2_vpu_send_snapshot(struct vpu_dev *dev)
{
	int i = 0;
	int strIdx = (~dev->hang_mask) & (dev->instance_mask);
	/*figure out the first available instance*/
	for (i = 0; i < VPU_MAX_NUM_STREAMS; i++) {
		if (CHECK_BIT(strIdx, i)) {
			strIdx = i;
			break;
		}
	}

	v4l2_vpu_send_cmd(dev->ctx[strIdx], strIdx, VID_API_CMD_SNAPSHOT, 0, NULL);
}

static int vpu_suspend(struct device *dev)
{
	struct vpu_dev *vpudev = (struct vpu_dev *)dev_get_drvdata(dev);

	if (vpudev->hang_mask != vpudev->instance_mask) {

		/*if there is an available device, send snapshot command to firmware*/
		v4l2_vpu_send_snapshot(vpudev);

		if (!wait_for_completion_timeout(&vpudev->snap_done_cmp, msecs_to_jiffies(1000))) {
			vpu_dbg(LVL_ERR, "error: wait for vpu decoder snapdone event timeout!\n");
			return -1;
		}
	}

	vpu_set_power(vpudev, false);

	return 0;
}

static int vpu_resume(struct device *dev)
{
	struct vpu_dev *vpudev = (struct vpu_dev *)dev_get_drvdata(dev);
	void *csr_offset, *csr_cpuwait;

	vpu_set_power(vpudev, true);
	vpu_enable_hw(vpudev);

	MU_Init(vpudev->mu_base_virtaddr);
	MU_EnableRxFullInt(vpudev->mu_base_virtaddr, 0);

	if (vpudev->hang_mask == vpudev->instance_mask) {
		/*no instance is active before suspend, do reset*/
		vpudev->fw_is_ready = false;
		vpudev->firmware_started = false;

		rpc_init_shared_memory(&vpudev->shared_mem, vpudev->m0_rpc_phy - vpudev->m0_p_fw_space_phy, vpudev->m0_rpc_virt, SHARED_SIZE);
		rpc_set_system_cfg_value(vpudev->shared_mem.pSharedInterface, VPU_REG_BASE);
	} else {
		/*resume*/
		csr_offset = ioremap(0x2d040000, 4);
		writel(vpudev->m0_p_fw_space_phy, csr_offset);
		csr_cpuwait = ioremap(0x2d040004, 4);
		writel(0x0, csr_cpuwait);
		/*wait for firmware resotre done*/
		if (!wait_for_completion_timeout(&vpudev->start_cmp, msecs_to_jiffies(1000))) {
			vpu_dbg(LVL_ERR, "error: wait for vpu decoder resume done timeout!\n");
			return -1;
		}
	}
	return 0;
}

static const struct dev_pm_ops vpu_pm_ops = {
	SET_RUNTIME_PM_OPS(vpu_runtime_suspend, vpu_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(vpu_suspend, vpu_resume)
};

static const struct of_device_id vpu_of_match[] = {
	{ .compatible = "nxp,imx8qm-b0-vpudec", },
	{ .compatible = "nxp,imx8qxp-b0-vpudec", },
	{}
}
MODULE_DEVICE_TABLE(of, vpu_of_match);

static struct platform_driver vpu_driver = {
	.probe = vpu_probe,
	.remove = vpu_remove,
	.driver = {
		.name = "vpu-b0",
		.of_match_table = vpu_of_match,
		.pm = &vpu_pm_ops,
	},
};
module_platform_driver(vpu_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Linux VPU driver for Freescale i.MX/MXC");
MODULE_LICENSE("GPL");

module_param(vpu_dbg_level_decoder, int, 0644);
MODULE_PARM_DESC(vpu_dbg_level_decoder, "Debug level (0-2)");

