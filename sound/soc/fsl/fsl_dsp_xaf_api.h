/* SPDX-License-Identifier: (GPL-2.0+ OR MIT)*/
/*
 * Xtensa Audio Framework API for communication with DSP
 *
 * Copyright (C) 2017 Cadence Design Systems, Inc.
 * Copyright 2018 NXP
 */
#ifndef FSL_DSP_XAF_API_H
#define FSL_DSP_XAF_API_H

#include "fsl_dsp_library_load.h"

/* ...size of auxiliary pool for communication with DSP */
#define XA_AUX_POOL_SIZE	32

/* ...length of auxiliary pool messages */
#define XA_AUX_POOL_MSG_LENGTH	128

/* ...number of max input buffers */
#define INBUF_SIZE	4096
#define OUTBUF_SIZE	16384

struct xaf_pipeline;

struct xaf_info_s {
	u32 opcode;
	void *buf;
	u32 length;
	u32 ret;
};

struct xaf_comp {
	struct xaf_comp *next;

	struct xaf_pipeline *pipeline;
	struct xf_handle handle;

	const char *dec_id;
	int comp_type;

	struct xf_pool *inpool;
	struct xf_pool *outpool;
	void *inptr;
	void *outptr;

	struct lib_info codec_lib;
	struct lib_info codec_wrap_lib;

	int active; /* component fully initialized */
};

struct xaf_pipeline {
	struct xaf_comp *comp_chain;

	u32 input_eos;
	u32 output_eos;
};

int xaf_comp_create(struct xf_client *client, struct xf_proxy *p_proxy,
		    struct xaf_comp *p_comp, int comp_type);
int xaf_comp_delete(struct xf_client *client, struct xaf_comp *p_comp);
int xaf_comp_flush(struct xf_client *client, struct xaf_comp *p_comp);

int xaf_comp_set_config(struct xf_client *client,struct xaf_comp *p_comp,
			u32 num_param, void *p_param);
int xaf_comp_get_config(struct xf_client *client,struct xaf_comp *p_comp,
			u32 num_param, void *p_param);

int xaf_comp_add(struct xaf_pipeline *p_pipe, struct xaf_comp *p_comp);
int xaf_comp_process(struct xf_client *client, struct xaf_comp *p_comp,
		     void *p_buf, u32 length, u32 flag);
int xaf_comp_get_status(struct xaf_comp *p_comp, struct xaf_info_s *p_info);
int xaf_comp_get_msg_count(struct xaf_comp *p_comp);

int xaf_connect(struct xf_client *client,struct xaf_comp *p_src,
		struct xaf_comp *p_dest, u32 num_buf, u32 buf_length);
int xaf_disconnect(struct xf_client *client,struct xaf_comp *p_comp);

int xaf_pipeline_create(struct xaf_pipeline *p_pipe);
int xaf_pipeline_delete(struct xaf_pipeline *p_pipe);

int xaf_pipeline_send_eos(struct xaf_pipeline *p_pipe);

/* ...port routing command */
struct __attribute__((__packed__)) xf_route_port_msg {
	/* ...source port specification */
	u32 src;
	/* ...destination port specification */
	u32 dst;
	/* ...number of buffers to allocate */
	u32 alloc_number;
	/* ...length of buffer to allocate */
	u32 alloc_size;
	/* ...alignment restriction for a buffer */
	u32 alloc_align;
};

/* ...port unrouting command */
struct __attribute__((__packed__)) xf_unroute_port_msg {
	/* ...source port specification */
	u32 src;
	/* ...destination port specification */
	u32 dst;
};

/* ...check if non-zero value is a power-of-two */
#define xf_is_power_of_two(v)       (((v) & ((v) - 1)) == 0)

/* ...component initialization parameter */
struct __attribute__((__packed__)) xf_set_param_msg {
	/* ...index of parameter passed to SET_CONFIG_PARAM call */
	u32 id;
	/* ...value of parameter */
	u32 value;
};

/* ...message body (command/response) */
struct __attribute__((__packed__)) xf_get_param_msg {
	/* ...array of parameters requested */
	u32 id;
	/* ...array of parameters values */
	u32 value;
};

/* ...renderer-specific configuration parameters */
enum xa_config_param_renderer {
	XA_RENDERER_CONFIG_PARAM_CB             = 0,
	XA_RENDERER_CONFIG_PARAM_STATE          = 1,
	XA_RENDERER_CONFIG_PARAM_PCM_WIDTH      = 2,
	XA_RENDERER_CONFIG_PARAM_CHANNELS       = 3,
	XA_RENDERER_CONFIG_PARAM_SAMPLE_RATE    = 4,
	XA_RENDERER_CONFIG_PARAM_FRAME_SIZE     = 5,
	XA_RENDERER_CONFIG_PARAM_NUM            = 6,
};

#endif /* FSL_DSP_XAF_API_H */
