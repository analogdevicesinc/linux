/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * NEOISP main header file
 *
 * Copyright 2023-2024 NXP
 */

#ifndef NEOISP_H
#define NEOISP_H

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>
#include <uapi/linux/nxp_neoisp.h>
#include "neoisp_regs.h"

/*
 * defines
 */
#define DEBUG
#define NEOISP_NAME            "neoisp"

#define NEOISP_NODE_GROUPS_COUNT (8)
#define NEOISP_MIN_W             (64u)
#define NEOISP_MIN_H             (64u)
#define NEOISP_MAX_W             (4096u)
#define NEOISP_MAX_H             (4096u)
#define NEOISP_ALIGN_W           (3)
#define NEOISP_ALIGN_H           (3)
#define NEOISP_FMT_CAP           (0)
#define NEOISP_FMT_OUT           (1)
#define NEOISP_DEF_W             (640)
#define NEOISP_DEF_H             (480)

#define NEOISP_MAX_CTRLS         (1)
#define NEOISP_CTRL_PARAMS       (0)

#define NEOISP_FMT_VCAP_COUNT    (15)
#define NEOISP_FMT_VOUT_COUNT    (24)
#define NEOISP_FMT_MCAP_COUNT    (1)
#define NEOISP_FMT_MOUT_COUNT    (1)

#define NODE_DESC_IS_OUTPUT(desc) ( \
		((desc)->buf_type == V4L2_BUF_TYPE_META_OUTPUT) || \
		((desc)->buf_type == V4L2_BUF_TYPE_VIDEO_OUTPUT) || \
		((desc)->buf_type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE))

#define NODE_IS_META(node) ( \
		((node)->buf_type == V4L2_BUF_TYPE_META_OUTPUT) || \
		((node)->buf_type == V4L2_BUF_TYPE_META_CAPTURE))
#define NODE_IS_OUTPUT(node) ( \
		((node)->buf_type == V4L2_BUF_TYPE_META_OUTPUT) || \
		((node)->buf_type == V4L2_BUF_TYPE_VIDEO_OUTPUT) || \
		((node)->buf_type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE))
#define NODE_IS_CAPTURE(node) ( \
		((node)->buf_type == V4L2_BUF_TYPE_META_CAPTURE) || \
		((node)->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE) || \
		((node)->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE))
#define NODE_IS_MPLANE(node) ( \
		((node)->buf_type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) || \
		((node)->buf_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE))

#define TYPE_IS_META(typ) ( \
		((typ) == V4L2_BUF_TYPE_META_OUTPUT) || \
		((typ) == V4L2_BUF_TYPE_META_CAPTURE))

#define FMT_IS_YUV(x) ( \
		((x) == V4L2_PIX_FMT_GREY) || \
		((x) == V4L2_PIX_FMT_NV12) || \
		((x) == V4L2_PIX_FMT_NV21) || \
		((x) == V4L2_PIX_FMT_NV16) || \
		((x) == V4L2_PIX_FMT_NV61) || \
		((x) == V4L2_PIX_FMT_UYVY) || \
		((x) == V4L2_PIX_FMT_YUYV) || \
		((x) == V4L2_PIX_FMT_YUV24) || \
		((x) == V4L2_PIX_FMT_YUVX32) || \
		((x) == V4L2_PIX_FMT_VUYX32) || \
		((x) == V4L2_PIX_FMT_VYUY))

#define NEOISP_SUSPEND_TIMEOUT_MS (500)

/* For logging only */
#define NODE_NAME(node) \
	(node_desc[(node)->id].ent_name + sizeof(NEOISP_NAME))

/*
 * enums
 */
enum neoisp_q_type_e {
	NEOISP_QUEUE_SRC = 0,
	NEOISP_QUEUE_DST = 1,
};

enum neoisp_fmt_type_e {
	NEOISP_FMT_VIDEO_CAPTURE = BIT(0),
	NEOISP_FMT_VIDEO_OUTPUT = BIT(1),
	NEOISP_FMT_META_CAPTURE = BIT(2),
	NEOISP_FMT_META_OUTPUT = BIT(3),
};

enum neoisp_node_e {
	NEOISP_INPUT0_NODE,
	NEOISP_INPUT1_NODE,
	NEOISP_PARAMS_NODE,
	NEOISP_FRAME_NODE,
	NEOISP_IR_NODE,
	NEOISP_STATS_NODE,
	NEOISP_NODES_COUNT
};

/*
 * structs
 */

/*
 * struct neoisp_info_s - ISP Hardware various information
 *
 * @isp_ver: ISP version
 *
 * This structure contains information about the ISP specific to a particular
 * ISP model, version, or integration in a particular SoC.
 */
struct neoisp_info_s {
	enum neoisp_version_e neoisp_hw_ver;
};

struct neoisp_node_desc_s {
	const char *ent_name;
	enum v4l2_buf_type buf_type;
	__u32 caps;
	__u32 link_flags;
};

struct neoisp_fmt_s {
	__u32 fourcc;
	__u32 align;
	__u32 bit_depth;
	__u32 num_planes;
	__u8 pl_divisors[VB2_MAX_PLANES];
	__u8 ibpp;
	__u8 is_rgb;
	__u8 is_rgb_ir;
	enum neoisp_fmt_type_e type;
};

/*
 * Structure to describe a single node /dev/video<N> which represents a single
 * input or output queue to the neoisp device.
 */
struct neoisp_node_s {
	__u32 id;
	__s32 vfl_dir;
	enum v4l2_buf_type buf_type;
	struct video_device vfd;
	struct media_pad pad;
	struct media_intf_devnode *intf_devnode;
	struct media_link *intf_link;
	struct neoisp_node_group_s *node_group;
	struct mutex node_lock;
	struct mutex queue_lock;
	spinlock_t ready_lock;
	struct list_head ready_queue;
	struct vb2_queue queue;
	struct v4l2_format format;
	const struct neoisp_fmt_s *neoisp_format;
};

struct neoisp_node_group_s {
	__u32 id;
	__u32 frame_sequence;
	struct v4l2_device v4l2_dev;
	struct v4l2_subdev sd;
	struct neoisp_dev_s *neoisp_dev;
	struct media_device mdev;
	struct neoisp_node_s node[NEOISP_NODES_COUNT];
	__u32 streaming_map; /* bitmap of which nodes are streaming */
	struct media_pad pad[NEOISP_NODES_COUNT]; /* output pads first */
	dma_addr_t params_dma_addr;
	struct neoisp_meta_params_s *params;
};

struct neoisp_buffer_s {
	struct vb2_v4l2_buffer vb;
	struct list_head ready_list;
	__u32 params_index;
};

/* Catch currently running or queued jobs on the neoisp hw */
struct neoisp_job_s {
	struct neoisp_node_group_s *node_group;
	struct neoisp_buffer_s *buf[NEOISP_NODES_COUNT];
};

struct neoisp_dev_s {
	struct platform_device *pdev;
	void __iomem *mmio;
	void __iomem *mmio_tcm;
	struct regmap *regmap;
	struct neoisp_regs_s regs;
	struct clk_bulk_data *clks;
	__s32 num_clks;
	struct neoisp_node_group_s node_group[NEOISP_NODE_GROUPS_COUNT];
	struct neoisp_job_s queued_job, running_job;
	__s32 hw_busy; /* non-zero if a job is queued or is being started */
	spinlock_t hw_lock; /* protects "hw_busy" flag and streaming_map */
};

/*
 * Module parameters
 */

struct neoisp_mparam_conf_s {
	__u32 img_conf_cam0_ibpp0;
	__u32 img_conf_cam0_inalign0;
	__u32 img_conf_cam0_lpalign0;
	__u32 img_conf_cam0_ibpp1;
	__u32 img_conf_cam0_inalign1;
	__u32 img_conf_cam0_lpalign1;
	__u32 img0_in_ls_cam0_ls;
	__u32 img1_in_ls_cam0_ls;
	__u32 skip_ctrl0_preskip;
	__u32 skip_ctrl0_postskip;
};

struct neoisp_mparam_packetizer_s {
	__u32 ch0_ctrl_cam0_obpp;
	__u32 ch0_ctrl_cam0_rsa;
	__u32 ch0_ctrl_cam0_lsa;
	__u32 ch12_ctrl_cam0_obpp;
	__u32 ch12_ctrl_cam0_rsa;
	__u32 ch12_ctrl_cam0_lsa;
	__u32 ch12_ctrl_cam0_subsample;
	__u32 ctrl_cam0_type;
	__u32 ctrl_cam0_order0;
	__u32 ctrl_cam0_order1;
	__u32 ctrl_cam0_order2;
	__u32 ctrl_cam0_a0s;
};

struct neoisp_mod_params_s {
	struct {
		__u32 disable_params;
		__u32 disable_stats;
	} test;
	struct neoisp_mparam_conf_s conf;
	struct neoisp_mparam_packetizer_s pack;
};

/*
 * globals
 */
extern const int neoisp_fields_a[NEOISP_FIELD_COUNT]; /* array of all fields offsets */
extern const struct regmap_config neoisp_regmap_config;
extern const struct v4l2_frmsize_stepwise neoisp_frmsize_stepwise;
extern const struct neoisp_fmt_s formats_vcap[NEOISP_FMT_VCAP_COUNT];
extern const struct neoisp_fmt_s formats_vout[NEOISP_FMT_VOUT_COUNT];
extern const struct neoisp_fmt_s formats_mcap[NEOISP_FMT_MCAP_COUNT];
extern const struct neoisp_fmt_s formats_mout[NEOISP_FMT_MOUT_COUNT];
extern const struct neoisp_node_desc_s node_desc[NEOISP_NODES_COUNT];
extern struct neoisp_mod_params_s mod_params;
extern struct neoisp_meta_params_s neoisp_default_params;

#endif /* NEOISP_H */
