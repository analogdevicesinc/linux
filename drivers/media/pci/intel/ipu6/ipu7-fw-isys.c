// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Intel Corporation
 */

#include "ipu6-bus.h"
#include "ipu6-dma.h"
#include "ipu6-isys.h"
#include "ipu7-boot.h"
#include "ipu7-fw-com.h"
#include "ipu7-fw-isys.h"

static void ipu7_fw_isys_cleanup(struct ipu6_isys *isys)
{
	struct ipu6_bus_device *adev = isys->adev;
	struct ipu7_fw_com_context *fwctx = isys->fwctx;

	if (!fwctx)
		return;

	ipu6_ipu7_release_boot_config(adev);

	if (fwctx->fw_config) {
		ipu6_dma_free(adev, sizeof(*fwctx->fw_config), fwctx->fw_config,
			      fwctx->fw_config_dma_addr, 0);
		fwctx->fw_config = NULL;
		fwctx->fw_config_dma_addr = 0;
	}

	isys->fwctx = NULL;
}

static int ipu7_fw_isys_open(struct ipu6_isys *isys)
{
	return ipu6_ipu7_boot_start_fw(isys->adev);
}

static int ipu7_fw_isys_close(struct ipu6_isys *isys)
{
	int ret;

	ret = ipu6_ipu7_boot_stop_fw(isys->adev);
	if (ret)
		return ret;

	ipu7_fw_isys_cleanup(isys);

	return ret;
}

static int ipu7_fw_isys_init(struct ipu6_isys *isys, unsigned int num_streams)
{
	struct ipu7_fw_com_queue_config *queue_configs;
	struct ipu6_bus_device *adev = isys->adev;
	struct device *dev = &adev->auxdev.dev;
	struct ipu7_insys_config *fw_config;
	struct ipu7_fw_com_context *fwctx;
	dma_addr_t fw_config_dma_addr;
	unsigned int i, num_queues;
	u32 freq;
	int ret;

	/* Allocate and init firmware context. */
	fwctx = devm_kzalloc(dev, sizeof(struct ipu7_fw_com_context),
			     GFP_KERNEL);
	if (!fwctx)
		return -ENOMEM;

	fwctx->num_input_queues = IPU7_INSYS_MAX_INPUT_QUEUES;
	fwctx->num_output_queues = IPU7_INSYS_MAX_OUTPUT_QUEUES;
	num_queues = fwctx->num_input_queues + fwctx->num_output_queues;

	queue_configs = devm_kcalloc(dev, num_queues, sizeof(*queue_configs),
				     GFP_KERNEL);
	if (!queue_configs) {
		ipu7_fw_isys_cleanup(isys);
		return -ENOMEM;
	}
	fwctx->fw_entry = adev->fw_entry;
	fwctx->queue_configs = queue_configs;
	queue_configs[IPU7_INSYS_OUTPUT_MSG_QUEUE].max_capacity =
		IPU7_ISYS_SIZE_RECV_QUEUE;
	queue_configs[IPU7_INSYS_OUTPUT_MSG_QUEUE].token_size_in_bytes =
		sizeof(struct ipu7_insys_resp);
	queue_configs[IPU7_INSYS_OUTPUT_LOG_QUEUE].max_capacity =
		IPU7_ISYS_SIZE_LOG_QUEUE;
	queue_configs[IPU7_INSYS_OUTPUT_LOG_QUEUE].token_size_in_bytes =
		sizeof(struct ipu7_insys_resp);
	queue_configs[IPU7_INSYS_OUTPUT_RESERVED_QUEUE].max_capacity = 0;
	queue_configs[IPU7_INSYS_OUTPUT_RESERVED_QUEUE].token_size_in_bytes = 0;

	queue_configs[IPU7_INSYS_INPUT_DEV_QUEUE].max_capacity =
		IPU7_ISYS_MAX_STREAMS;
	queue_configs[IPU7_INSYS_INPUT_DEV_QUEUE].token_size_in_bytes =
		sizeof(struct ipu7_insys_send_queue_token);

	for (i = IPU7_INSYS_INPUT_MSG_QUEUE; i < num_queues; i++) {
		queue_configs[i].max_capacity = IPU7_ISYS_SIZE_SEND_QUEUE;
		queue_configs[i].token_size_in_bytes =
			sizeof(struct ipu7_insys_send_queue_token);
	}

	/* Allocate ISYS subsys config. */
	fw_config = ipu6_dma_alloc(adev, sizeof(*fw_config),
				   &fw_config_dma_addr, GFP_KERNEL, 0);
	if (!fw_config) {
		dev_err(dev, "Failed to allocate isys subsys config.\n");
		ipu7_fw_isys_cleanup(isys);
		return -ENOMEM;
	}
	fwctx->fw_config = fw_config;
	fwctx->fw_config_dma_addr = fw_config_dma_addr;
	memset(fw_config, 0, sizeof(*fw_config));
	fw_config->logger_config.use_source_severity = 0;
	fw_config->logger_config.use_channels_enable_bitmask = 1;
	fw_config->logger_config.channels_enable_bitmask =
				IPU7_LOGGER_CFG_CHANNEL_ENABLE_SYSCOM;
	fw_config->logger_config.hw_printf_buffer_base_addr = 0U;
	fw_config->logger_config.hw_printf_buffer_size_bytes = 0U;
	fw_config->wdt_config.wdt_timer1_us = 0;
	fw_config->wdt_config.wdt_timer2_us = 0;
	freq = ipu7_buttress_get_isys_freq(adev->isp);

	ipu6_dma_sync_single(adev, fw_config_dma_addr,
			     sizeof(struct ipu7_insys_config));

	isys->fwctx = fwctx;

	ret = ipu6_ipu7_init_boot_config(adev, queue_configs, num_queues,
					 freq, fw_config_dma_addr, 1U);
	if (ret) {
		ipu7_fw_isys_cleanup(isys);
		return ret;
	}

	ret = ipu7_fw_isys_open(isys);
	if (ret)
		ipu7_fw_isys_cleanup(isys);

	return ret;
}

static struct ipu7_insys_resp *ipu7_fw_isys_get_resp(struct ipu6_isys *isys)
{
	return ipu7_fw_com_get_token(isys->fwctx, IPU7_INSYS_OUTPUT_MSG_QUEUE);
}

static void ipu7_fw_isys_put_resp(struct ipu6_isys *isys)
{
	ipu7_fw_com_put_token(isys->fwctx, IPU7_INSYS_OUTPUT_MSG_QUEUE);
}

static int ipu7_isys_fw_pin_cfg(struct ipu6_isys_video *av,
				struct ipu7_fw_isys_stream_cfg *cfg)
{
	struct media_pad *src_pad = media_pad_remote_pad_first(&av->pad);
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(src_pad->entity);
	struct v4l2_subdev_state *state = v4l2_subdev_get_locked_active_state(sd);
	struct ipu7_fw_isys_input_pin *input_pin;
	struct ipu7_fw_isys_output_pin *output_pin;
	struct ipu6_isys_stream *stream = av->stream;
	struct ipu6_isys_queue *aq = &av->aq;
	struct v4l2_mbus_framefmt fmt;
	const struct ipu6_isys_pixelformat *pfmt =
		ipu6_isys_get_isys_format(ipu6_isys_get_format(av), 0);
	int input_pins = cfg->nof_input_pins++;
	int output_pins;
	u32 src_stream;

	src_stream = ipu6_isys_get_src_stream_by_src_pad(sd, src_pad->index);
	fmt = *v4l2_subdev_state_get_format(state, src_pad->index, src_stream);

	input_pin = &cfg->input_pins[input_pins];
	input_pin->input_res.width = fmt.width;
	input_pin->input_res.height = fmt.height;
	input_pin->dt = av->dt;
	input_pin->disable_mipi_unpacking = 0;
	if (pfmt->bpp == pfmt->bpp_packed && pfmt->bpp % BITS_PER_BYTE)
		input_pin->disable_mipi_unpacking = 1;
	input_pin->mapped_dt = IPU7_N_INSYS_MIPI_DATA_TYPE;
	input_pin->dt_rename_mode = IPU7_INSYS_MIPI_DT_NO_RENAME;
	input_pin->sync_msg_map =
		IPU7_INSYS_STREAM_SYNC_MSG_SEND_RESP_SOF |
		IPU7_INSYS_STREAM_SYNC_MSG_SEND_RESP_SOF_DISCARDED |
		IPU7_INSYS_STREAM_SYNC_MSG_SEND_IRQ_SOF |
		IPU7_INSYS_STREAM_SYNC_MSG_SEND_IRQ_SOF_DISCARDED;

	output_pins = cfg->nof_output_pins++;
	aq->fw_output = output_pins;
	stream->output_pins_queue[output_pins] = aq;

	output_pin = &cfg->output_pins[output_pins];
	memset(output_pin, 0, sizeof(*output_pin));
	output_pin->link.buffer_lines = 0;
	output_pin->link.foreign_key = IPU7_MSG_LINK_FOREIGN_KEY_NONE;
	output_pin->link.pbk_id = IPU7_MSG_LINK_PBK_ID_DONT_CARE;
	output_pin->link.pbk_slot_id = IPU7_MSG_LINK_PBK_SLOT_ID_DONT_CARE;
	output_pin->link.dest = 0; /* IPU_INSYS_OUTPUT_LINK_DEST_MEM */
	output_pin->link.use_sw_managed = 1;
	output_pin->crop.line_top = 0;
	output_pin->crop.line_bottom = 0;
	output_pin->dpcm.enable = 0;
	output_pin->ft = pfmt->css_pixelformat;
	output_pin->stride = ipu6_isys_get_bytes_per_line(av);
	output_pin->send_irq = 1;
	output_pin->input_pin_id = input_pins;

	return 0;
}

static int
ipu7_fw_isys_send_cmd(struct ipu6_isys *isys, const unsigned int stream_handle,
		      void *cpu_mapped_buf, dma_addr_t dma_mapped_buf,
		      size_t size, u16 send_type)
{
	struct ipu7_fw_com_context *ctx = isys->fwctx;
	/*struct device *dev = &isys->adev->auxdev.dev;*/
	struct ipu7_insys_send_queue_token *token;

	if (send_type >= N_IPU7_INSYS_SEND_TYPE)
		return -EINVAL;

	if (cpu_mapped_buf)
		clflush_cache_range(cpu_mapped_buf, size);

	token = ipu7_fw_com_get_token(ctx, stream_handle +
				      IPU7_INSYS_INPUT_MSG_QUEUE);
	if (!token)
		return -EBUSY;

	token->addr = dma_mapped_buf;
	token->buf_handle = (unsigned long)cpu_mapped_buf;
	token->send_type = send_type;
	token->stream_id = stream_handle;
	token->flag = IPU7_INSYS_SEND_QUEUE_TOKEN_FLAG_NONE;

	ipu7_fw_com_put_token(ctx, stream_handle + IPU7_INSYS_INPUT_MSG_QUEUE);
	/* now wakeup FW */
	ipu7_buttress_wakeup_isys(isys->adev->isp);

	return 0;
}

static void ipu7_fw_isys_dump_stream_cfg(struct device *dev,
					 struct isys_fw_msgs *msg)
{
	struct ipu7_fw_isys_stream_cfg *cfg = &msg->ipu7.stream;
	unsigned int i;

	dev_dbg(dev, "---------------------------\n");
	dev_dbg(dev, "IPU_FW_ISYS_STREAM_CFG_DATA\n");

	dev_dbg(dev, ".port id %d\n", cfg->port_id);
	dev_dbg(dev, ".vc %d\n", cfg->vc);
	dev_dbg(dev, ".nof_input_pins = %d\n", cfg->nof_input_pins);
	dev_dbg(dev, ".nof_output_pins = %d\n", cfg->nof_output_pins);
	dev_dbg(dev, ".stream_msg_map = 0x%x\n", cfg->stream_msg_map);

	for (i = 0; i < cfg->nof_input_pins; i++) {
		dev_dbg(dev, ".input_pin[%d]:\n", i);
		dev_dbg(dev, "\t.dt = 0x%0x\n",
			cfg->input_pins[i].dt);
		dev_dbg(dev, "\t.disable_mipi_unpacking = %d\n",
			cfg->input_pins[i].disable_mipi_unpacking);
		dev_dbg(dev, "\t.dt_rename_mode = %d\n",
			cfg->input_pins[i].dt_rename_mode);
		dev_dbg(dev, "\t.mapped_dt = 0x%0x\n",
			cfg->input_pins[i].mapped_dt);
		dev_dbg(dev, "\t.input_res = %d x %d\n",
			cfg->input_pins[i].input_res.width,
			cfg->input_pins[i].input_res.height);
		dev_dbg(dev, "\t.sync_msg_map = 0x%x\n",
			cfg->input_pins[i].sync_msg_map);
	}

	for (i = 0; i < cfg->nof_output_pins; i++) {
		dev_dbg(dev, ".output_pin[%d]:\n", i);
		dev_dbg(dev, "\t.input_pin_id = %d\n",
			cfg->output_pins[i].input_pin_id);
		dev_dbg(dev, "\t.stride = %d\n", cfg->output_pins[i].stride);
		dev_dbg(dev, "\t.send_irq = %d\n",
			cfg->output_pins[i].send_irq);
		dev_dbg(dev, "\t.ft = %d\n", cfg->output_pins[i].ft);

		dev_dbg(dev, "\t.link.buffer_lines = %d\n",
			cfg->output_pins[i].link.buffer_lines);
		dev_dbg(dev, "\t.link.foreign_key = %d\n",
			cfg->output_pins[i].link.foreign_key);
		dev_dbg(dev, "\t.link.granularity_pointer_update = %d\n",
			cfg->output_pins[i].link.granularity_pointer_update);
		dev_dbg(dev, "\t.link.msg_link_streaming_mode = %d\n",
			cfg->output_pins[i].link.msg_link_streaming_mode);
		dev_dbg(dev, "\t.link.pbk_id = %d\n",
			cfg->output_pins[i].link.pbk_id);
		dev_dbg(dev, "\t.link.pbk_slot_id = %d\n",
			cfg->output_pins[i].link.pbk_slot_id);
		dev_dbg(dev, "\t.link.dest = %d\n",
			cfg->output_pins[i].link.dest);
		dev_dbg(dev, "\t.link.use_sw_managed = %d\n",
			cfg->output_pins[i].link.use_sw_managed);
		dev_dbg(dev, "\t.link.is_snoop = %d\n",
			cfg->output_pins[i].link.is_snoop);

		dev_dbg(dev, "\t.crop.line_top = %d\n",
			cfg->output_pins[i].crop.line_top);
		dev_dbg(dev, "\t.crop.line_bottom = %d\n",
			cfg->output_pins[i].crop.line_bottom);

		dev_dbg(dev, "\t.dpcm_enable = %d\n",
			cfg->output_pins[i].dpcm.enable);
		dev_dbg(dev, "\t.dpcm.type = %d\n",
			cfg->output_pins[i].dpcm.type);
		dev_dbg(dev, "\t.dpcm.predictor = %d\n",
			cfg->output_pins[i].dpcm.predictor);
	}
	dev_dbg(dev, "---------------------------\n");
}

static void ipu7_fw_isys_dump_frame_buf_set(struct device *dev,
					    struct isys_fw_msgs *msg,
					    unsigned int outputs)
{
	struct ipu7_fw_isys_frame_buff_set *buf = &msg->ipu7.frame;
	unsigned int i;

	dev_dbg(dev, "--------------------------\n");
	dev_dbg(dev, "IPU_ISYS_BUFF_SET\n");
	dev_dbg(dev, ".capture_msg_map = %d\n", buf->capture_msg_map);
	dev_dbg(dev, ".frame_id = %d\n", buf->frame_id);
	dev_dbg(dev, ".skip_frame = %d\n", buf->skip_frame);

	for (i = 0; i < outputs; i++) {
		dev_dbg(dev, ".output_pin[%d]:\n", i);
		dev_dbg(dev, "\t.user_token = %llx\n",
			buf->output_pins[i].user_token);
		dev_dbg(dev, "\t.addr = 0x%x\n", buf->output_pins[i].addr);
	}
	dev_dbg(dev, "---------------------------\n");
}

static int ipu7_fw_isys_prepare_stream_cfg(struct ipu6_isys_video *av,
					   struct isys_fw_msgs *msg)
{
	struct ipu7_fw_isys_stream_cfg *cfg = &msg->ipu7.stream;
	struct device *dev = &av->isys->adev->auxdev.dev;
	struct ipu6_isys_stream *stream = av->stream;
	struct ipu6_isys_queue *aq;

	memset(cfg, 0, sizeof(*cfg));
	cfg->port_id = stream->stream_source;
	cfg->vc = stream->vc;
	cfg->stream_msg_map = IPU7_INSYS_STREAM_ENABLE_MSG_SEND_RESP |
			      IPU7_INSYS_STREAM_ENABLE_MSG_SEND_IRQ;

	list_for_each_entry(aq, &stream->queues, node) {
		struct ipu6_isys_video *__av = ipu6_isys_queue_to_video(aq);
		int ret = ipu7_isys_fw_pin_cfg(__av, cfg);

		if (ret < 0)
			return ret;
	}

	stream->nr_output_pins = cfg->nof_output_pins;

	ipu7_fw_isys_dump_stream_cfg(dev, msg);

	return 0;
}

static void
ipu7_isys_buf_to_fw_frame_buf_pin(struct vb2_buffer *vb,
				  struct ipu7_fw_isys_frame_buff_set *set)
{
	struct ipu6_isys_queue *aq = vb2_queue_to_isys_queue(vb->vb2_queue);
	struct vb2_v4l2_buffer *vvb = to_vb2_v4l2_buffer(vb);
	struct ipu6_isys_video_buffer *ivb =
		vb2_buffer_to_ipu6_isys_video_buffer(vvb);

	set->output_pins[aq->fw_output].addr = ivb->dma_addr;
	set->output_pins[aq->fw_output].user_token = (u64)(uintptr_t)set;
}

static void
ipu7_fw_isys_prepare_buf_set(struct isys_fw_msgs *msg,
			     struct ipu6_isys_stream *stream,
			     struct ipu6_isys_buffer_list *bl)
{
	struct ipu7_fw_isys_frame_buff_set *set = &msg->ipu7.frame;
	struct ipu6_isys_buffer *ib;

	WARN_ON(!bl->nbufs);

	memset(set, 0, sizeof(*set));
	set->capture_msg_map = IPU7_INSYS_FRAME_ENABLE_MSG_SEND_RESP |
			       IPU7_INSYS_FRAME_ENABLE_MSG_SEND_IRQ;
	set->frame_id = atomic_fetch_inc(&stream->buf_id) % 256;

	list_for_each_entry(ib, &bl->head, head) {
		struct vb2_buffer *vb = ipu6_isys_buffer_to_vb2_buffer(ib);

		ipu7_isys_buf_to_fw_frame_buf_pin(vb, set);
	}

	dev_dbg(&stream->isys->adev->auxdev.dev,
		"ipu7 frame_buf: cap_map=0x%x fid=%u opin[0].addr=0x%x token=0x%llx\n",
		set->capture_msg_map, set->frame_id,
		set->output_pins[0].addr, set->output_pins[0].user_token);
}

static int ipu7_fw_isys_stream_open(struct ipu6_isys *isys,
				    const unsigned int stream_handle,
				    struct isys_fw_msgs *msg)
{
	return ipu7_fw_isys_send_cmd(isys, stream_handle, &msg->ipu7.stream,
				     msg->dma_addr, sizeof(msg->ipu7.stream),
				     IPU7_INSYS_SEND_TYPE_STREAM_OPEN);
}

static int ipu7_fw_isys_stream_close(struct ipu6_isys *isys,
				     const unsigned int stream_handle)
{
	return ipu7_fw_isys_send_cmd(isys, stream_handle, NULL, 0, 0,
				     IPU7_INSYS_SEND_TYPE_STREAM_CLOSE);
}

static int ipu7_fw_isys_stream_flush(struct ipu6_isys *isys,
				     const unsigned int stream_handle)
{
	return ipu7_fw_isys_send_cmd(isys, stream_handle, NULL, 0, 0,
				     IPU7_INSYS_SEND_TYPE_STREAM_FLUSH);
}

static int ipu7_fw_isys_stream_start(struct ipu6_isys *isys,
				     const unsigned int stream_handle,
				     struct isys_fw_msgs *msg, bool capture)
{
	return ipu7_fw_isys_send_cmd(isys, stream_handle, &msg->ipu7.frame,
				     msg->dma_addr, sizeof(msg->ipu7.frame),
				     IPU7_INSYS_SEND_TYPE_STREAM_START_AND_CAPTURE);
}

static int ipu7_fw_isys_stream_capture(struct ipu6_isys *isys,
				       const unsigned int stream_handle,
				       struct isys_fw_msgs *msg)
{
	return ipu7_fw_isys_send_cmd(isys, stream_handle, &msg->ipu7.frame,
				     msg->dma_addr, sizeof(msg->ipu7.frame),
				     IPU7_INSYS_SEND_TYPE_STREAM_CAPTURE);
}

const struct ipu6_fw_isys_ops ipu7_fw_isys_ops = {
	.init = ipu7_fw_isys_init,
	.close = ipu7_fw_isys_close,
	.send_cmd = ipu7_fw_isys_send_cmd,
	.cleanup = ipu7_fw_isys_cleanup,
	.prepare_stream_cfg = ipu7_fw_isys_prepare_stream_cfg,
	.prepare_buf_set = ipu7_fw_isys_prepare_buf_set,
	.stream_open = ipu7_fw_isys_stream_open,
	.stream_start = ipu7_fw_isys_stream_start,
	.stream_capture = ipu7_fw_isys_stream_capture,
	.stream_flush = ipu7_fw_isys_stream_flush,
	.stream_close = ipu7_fw_isys_stream_close,
	.dump_stream_cfg = ipu7_fw_isys_dump_stream_cfg,
	.dump_frame_buf_set = ipu7_fw_isys_dump_frame_buf_set,
};
