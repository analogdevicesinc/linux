/*
 * Copyright 2018 NXP
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*!
 * @file vpu_b0.h
 *
 * @brief VPU B0 definition
 *
 */
#ifndef __VPU_B0_H
#define __VPU_B0_H

#include <linux/irqreturn.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/videobuf2-v4l2.h>
#include <soc/imx8/sc/svc/irq/api.h>
#include <soc/imx8/sc/ipc.h>
#include <soc/imx8/sc/sci.h>
#include <linux/mx8_mu.h>
#include <media/v4l2-event.h>
#include <linux/kfifo.h>
#include "vpu_rpc.h"

extern unsigned int vpu_dbg_level_decoder;

#define v4l2_fh_to_ctx(__fh) \
	container_of(__fh, struct vpu_ctx, fh)
#define v4l2_ctrl_to_ctx(__ctrl) \
	container_of((__ctrl)->handler, struct vpu_ctx, ctrl_handler)

#define MIN_SPACE (4096+64)

#define VPU_MAX_FORMATS 4
#define VPU_MAX_BUFFER 32
#define M0FW_FILENAME "vpu/vpu_fw_imx8_dec.bin"
#define MMAP_BUF_TYPE_SHIFT 28
#define MMAP_BUF_TYPE_MASK 0xF0000000
#define M0_BOOT_SIZE 0x1000000
#define M0_PRINT_OFFSET 0x500000
#define DCP_SIZE 0x3000000
#define MAX_BUFFER_SIZE 5242880
#define UDATA_BUFFER_SIZE 0x1000
#define SHARED_SIZE 0x00400000
#define MAX_DCP_NUM 2
#define MAX_MBI_NUM 18 // same with MEDIA_PLAYER_MAX_MBI_UNIT defined in firmware
#define MAX_TIMEOUT_COUNT 10
#ifdef CM4
#define VPU_REG_BASE 0x2c000000
#else
#define VPU_REG_BASE 0x40000000
#endif

#define V4L2_MAX_CTRLS 12
#define V4L2_PIX_FMT_NV12_10BIT    v4l2_fourcc('N', 'T', '1', '2') /*  Y/CbCr 4:2:0 for 10bit  */
#define INVALID_FRAME_DEPTH -1
#define DECODER_NODE_NUMBER 12 // use /dev/video12 as vpu decoder

struct vpu_v4l2_control {
	uint32_t id;
	enum v4l2_ctrl_type type;
	uint32_t minimum;
	uint32_t maximum;
	uint32_t step;
	uint32_t default_value;
	uint32_t menu_skip_mask;
	bool is_volatile;
};

typedef enum{
	INIT_DONE = 1,
	RPC_BUF_OFFSET,
	PRINT_BUF_OFFSET,
	BOOT_ADDRESS,
	COMMAND,
	EVENT
} MSG_Type;

enum PLAT_TYPE {
	IMX8QXP = 0,
	IMX8QM  = 1,
};

enum QUEUE_TYPE {
	V4L2_SRC = 0,
	V4L2_DST = 1,
};

enum vpu_video_standard {
	VPU_VIDEO_UNDEFINED = 0,
	VPU_VIDEO_AVC = 1,
	VPU_VIDEO_VC1 = 2,
	VPU_VIDEO_MPEG2 = 3,
	VPU_VIDEO_AVS = 4,
	VPU_VIDEO_ASP = 5,
	VPU_VIDEO_JPEG = 6,
	VPU_VIDEO_RV = 7,
	VPU_VIDEO_VP6 = 8,
	VPU_VIDEO_SPK = 9,
	VPU_VIDEO_VP8 = 10,
	VPU_VIDEO_AVC_MVC = 11,
	VPU_VIDEO_HEVC = 12,
};

typedef enum{
	EOS_PADDING_TYPE = 1,
	BUFFLUSH_PADDING_TYPE = 2
} VPU_PADDING_SCODE_TYPE;

#define VPU_PIX_FMT_AVS         v4l2_fourcc('A', 'V', 'S', '0')
#define VPU_PIX_FMT_ASP         v4l2_fourcc('A', 'S', 'P', '0')
#define VPU_PIX_FMT_RV          v4l2_fourcc('R', 'V', '0', '0')
#define VPU_PIX_FMT_VP6         v4l2_fourcc('V', 'P', '6', '0')
#define VPU_PIX_FMT_SPK         v4l2_fourcc('S', 'P', 'K', '0')
#define VPU_PIX_FMT_DIVX        v4l2_fourcc('D', 'I', 'V', 'X')
#define VPU_PIX_FMT_HEVC        v4l2_fourcc('H', 'E', 'V', 'C')
#define VPU_PIX_FMT_LOGO        v4l2_fourcc('L', 'O', 'G', 'O')

#define VPU_PIX_FMT_TILED_8     v4l2_fourcc('Z', 'T', '0', '8')
#define VPU_PIX_FMT_TILED_10    v4l2_fourcc('Z', 'T', '1', '0')

enum vpu_pixel_format {
	VPU_HAS_COLOCATED = 0x00000001,
	VPU_HAS_SPLIT_FLD = 0x00000002,
	VPU_PF_MASK       = ~(VPU_HAS_COLOCATED | VPU_HAS_SPLIT_FLD),

	VPU_IS_TILED      = 0x000000100,
	VPU_HAS_10BPP     = 0x00000200,

	VPU_IS_PLANAR     = 0x00001000,
	VPU_IS_SEMIPLANAR = 0x00002000,
	VPU_IS_PACKED     = 0x00004000,

	// Merged definitions using above flags:
	VPU_PF_UNDEFINED  = 0,
	VPU_PF_YUV420_SEMIPLANAR = 0x00010000 | VPU_IS_SEMIPLANAR,
	VPU_PF_YUV420_PLANAR = 0x00020000 | VPU_IS_PLANAR,
	VPU_PF_UYVY = 0x00040000 | VPU_IS_PACKED,
	VPU_PF_TILED_8BPP = 0x00080000 | VPU_IS_TILED | VPU_IS_SEMIPLANAR,
	VPU_PF_TILED_10BPP = 0x00100000 | VPU_IS_TILED | VPU_IS_SEMIPLANAR | VPU_HAS_10BPP,
};

struct vpu_v4l2_fmt {
	char *name;
	unsigned int fourcc;
	unsigned int num_planes;
	unsigned int vdec_std;
};

struct vb2_data_req {
	struct list_head  list;
	struct vb2_buffer *vb2_buf;
	int id;
	u_int32 status;
	bool bfield;
	u_int32 phy_addr[2]; //0 for luma, 1 for chroma
};

struct queue_data {
	unsigned int width;
	unsigned int height;
	unsigned int stride;
	unsigned int bytesperline;
	unsigned int num_planes;
	unsigned int sizeimage[2];
	unsigned int fourcc;
	unsigned int vdec_std;
	struct v4l2_rect rect;
	int buf_type; // v4l2_buf_type
	bool vb2_q_inited;
	struct vb2_queue vb2_q;    // vb2 queue
	struct list_head drv_q;    // driver queue
	struct semaphore drv_q_lock;
	struct vb2_data_req vb2_reqs[VPU_MAX_BUFFER];
	enum QUEUE_TYPE type;
};
struct vpu_ctx;
struct vpu_dev {
	struct device *generic_dev;
	struct v4l2_device v4l2_dev;
	struct video_device *pvpu_decoder_dev;
	struct platform_device *plat_dev;
	struct firmware *m0_pfw;
	void *m0_p_fw_space_vir;
	u_int32 m0_p_fw_space_phy;
	void *m0_rpc_virt;
	u_int32 m0_rpc_phy;
#ifndef DYNAMIC_MEM
	void *str_base_vir;
	u_int32 str_base_phy;
	u_int32 str_size;
#endif
	struct mutex dev_mutex;
	struct mutex cmd_mutex;
	bool fw_is_ready;
	bool firmware_started;
	struct completion start_cmp;
	struct completion snap_done_cmp;
	struct workqueue_struct *workqueue;
	struct work_struct msg_work;
	unsigned long instance_mask;
	unsigned long hang_mask; //this is used to deal with hang issue to reset firmware
	sc_ipc_t mu_ipcHandle;
	struct clk *vpu_clk;
	void __iomem *mu_base_virtaddr;
	unsigned int vpu_mu_id;
	int vpu_mu_init;
	u_int32 plat_type;

	struct clk *clk_m0;
	void __iomem *regs_base;
	void __iomem *csr_base;
	u_int32 cm_offset;

	struct shared_addr shared_mem;
	struct vpu_ctx *ctx[VPU_MAX_NUM_STREAMS];
};

struct vpu_statistic {
	unsigned long cmd[VID_API_CMD_YUV_READY + 2];
	unsigned long event[VID_API_EVENT_DEC_CFG_INFO + 2];
	unsigned long current_cmd;
	unsigned long current_event;
	struct timespec ts_cmd;
	struct timespec ts_event;
};

struct vpu_ctx {
	struct vpu_dev *dev;
	struct v4l2_fh fh;

	struct vpu_statistic statistic;
	struct device_attribute dev_attr_instance_command;
	char command_name[64];
	struct device_attribute dev_attr_instance_event;
	char event_name[64];
	struct device_attribute dev_attr_instance_buffer;
	char buffer_name[64];
	struct v4l2_ctrl *ctrls[V4L2_MAX_CTRLS];
	struct v4l2_ctrl_handler ctrl_handler;
	bool ctrl_inited;

	int str_index;
	struct queue_data q_data[2];
	struct kfifo msg_fifo;
	struct mutex instance_mutex;
	struct work_struct instance_work;
	struct workqueue_struct *instance_wq;
	struct completion completion;
	struct completion stop_cmp;
	struct completion eos_cmp;
	MediaIPFW_Video_SeqInfo *pSeqinfo;
	bool b_dis_reorder;
	bool b_firstseq;
	bool start_flag;
	bool wait_abort_done;
	bool wait_rst_done;
	bool buffer_null;
	bool firmware_stopped;
	bool firmware_finished;
	bool eos_stop_received;
	bool eos_stop_added;
	bool ctx_released;
	wait_queue_head_t buffer_wq;
	void *dpb_dma_virt;
	u_int32 uSize;
	dma_addr_t dpb_dma_phy;
	void *dcp_dma_virt[MAX_DCP_NUM];
	u_int32 uDCPSize;
	dma_addr_t dcp_dma_phy[MAX_DCP_NUM];
	u_int32 dcp_count;
	void *mbi_dma_virt[MAX_MBI_NUM];
	u_int32 mbi_size;
	dma_addr_t mbi_dma_phy[MAX_MBI_NUM];
	u_int32 mbi_count;
	u_int32 mbi_num;
	void *stream_buffer_virt;
	u_int32 stream_buffer_size;
	dma_addr_t stream_buffer_phy;
	void *udata_buffer_virt;
	u_int32 udata_buffer_size;
	dma_addr_t udata_buffer_phy;
	int frm_dis_delay;
	int frm_dec_delay;
	int frm_total_num;
};

#define LVL_INFO 3
#define LVL_EVENT  2
#define LVL_WARN  1
#define LVL_ERR  0

#define vpu_dbg(level, fmt, arg...) \
	do { \
		if (vpu_dbg_level_decoder >= (level)) \
			printk("[DEBUG]\t " fmt, ## arg); \
	} while (0)

#endif
