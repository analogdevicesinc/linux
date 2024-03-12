// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave6 series multi-standard codec IP - low level access functions
 *
 * Copyright (C) 2021 CHIPS&MEDIA INC
 */

#include <linux/types.h>
#include <linux/debugfs.h>
#include "wave6-vpu.h"
#include "wave6-vpu-dbg.h"

static int wave6_vpu_dbg_instance(struct seq_file *s, void *data)
{
	struct vpu_instance *inst = s->private;
	struct vpu_performance_info *perf = &inst->performance;
	struct vb2_queue *vq;
	char str[128];
	int num;
	s64 tmp;
	s64 fps;

	if (!inst->v4l2_fh.m2m_ctx)
		return 0;

	num = scnprintf(str, sizeof(str), "[%s]\n",
			inst->type == VPU_INST_TYPE_DEC ? "Decoder" : "Encoder");
	if (seq_write(s, str, num))
		return 0;

	num = scnprintf(str, sizeof(str),
			"%s : product 0x%x, fw_version %d.%d.%d(r%d), hw_version 0x%x\n",
			dev_name(inst->dev->dev), inst->dev->product_code,
			(inst->dev->fw_version >> 24) & 0xFF,
			(inst->dev->fw_version >> 16) & 0xFF,
			(inst->dev->fw_version >> 0) & 0xFFFF,
			inst->dev->fw_revision, inst->dev->hw_version);
	if (seq_write(s, str, num))
		return 0;

	num = scnprintf(str, sizeof(str), "state = %s\n",
			wave6_vpu_instance_state_name(inst->state));
	if (seq_write(s, str, num))
		return 0;

	vq = v4l2_m2m_get_src_vq(inst->v4l2_fh.m2m_ctx);
	num = scnprintf(str, sizeof(str),
			"output (%2d, %2d): fmt = %c%c%c%c %d x %d, %d;\n",
			vb2_is_streaming(vq),
			vq->num_buffers,
			inst->src_fmt.pixelformat,
			inst->src_fmt.pixelformat >> 8,
			inst->src_fmt.pixelformat >> 16,
			inst->src_fmt.pixelformat >> 24,
			inst->src_fmt.width,
			inst->src_fmt.height,
			vq->last_buffer_dequeued);
	if (seq_write(s, str, num))
		return 0;

	vq = v4l2_m2m_get_dst_vq(inst->v4l2_fh.m2m_ctx);
	num = scnprintf(str, sizeof(str),
			"capture(%2d, %2d): fmt = %c%c%c%c %d x %d, %d;\n",
			vb2_is_streaming(vq),
			vq->num_buffers,
			inst->dst_fmt.pixelformat,
			inst->dst_fmt.pixelformat >> 8,
			inst->dst_fmt.pixelformat >> 16,
			inst->dst_fmt.pixelformat >> 24,
			inst->dst_fmt.width,
			inst->dst_fmt.height,
			vq->last_buffer_dequeued);
	if (seq_write(s, str, num))
		return 0;

	num = scnprintf(str, sizeof(str), "crop: (%d, %d) %d x %d\n",
			inst->crop.left,
			inst->crop.top,
			inst->crop.width,
			inst->crop.height);
	if (seq_write(s, str, num))
		return 0;

	if (inst->scaler_info.enable) {
		num = scnprintf(str, sizeof(str), "scale: %d x %d\n",
				inst->scaler_info.width, inst->scaler_info.height);
		if (seq_write(s, str, num))
			return 0;
	}

	num = scnprintf(str, sizeof(str),
			"queued src %d, dst %d, process %d, sequence %d, error %d, drain %d:%d\n",
			inst->queued_src_buf_num,
			inst->queued_dst_buf_num,
			inst->processed_buf_num,
			inst->sequence,
			inst->error_buf_num,
			inst->v4l2_fh.m2m_ctx->out_q_ctx.buffered,
			inst->eos);
	if (seq_write(s, str, num))
		return 0;

	num = scnprintf(str, sizeof(str), "fps");
	if (seq_write(s, str, num))
		return 0;
	tmp = MSEC_PER_SEC * inst->processed_buf_num;
	if (perf->ts_last > perf->ts_first + NSEC_PER_MSEC) {
		fps = DIV_ROUND_CLOSEST(tmp, (perf->ts_last - perf->ts_first) / NSEC_PER_MSEC);
		num = scnprintf(str, sizeof(str), " actual: %lld;", fps);
		if (seq_write(s, str, num))
			return 0;
	}
	if (perf->total_sw_time) {
		fps = DIV_ROUND_CLOSEST(tmp, perf->total_sw_time / NSEC_PER_MSEC);
		num = scnprintf(str, sizeof(str), " sw: %lld;", fps);
		if (seq_write(s, str, num))
			return 0;
	}
	if (perf->total_hw_time) {
		fps = DIV_ROUND_CLOSEST(tmp, perf->total_hw_time / NSEC_PER_MSEC);
		num = scnprintf(str, sizeof(str), " hw: %lld", fps);
		if (seq_write(s, str, num))
			return 0;
	}
	num = scnprintf(str, sizeof(str), "\n");
	if (seq_write(s, str, num))
		return 0;

	num = scnprintf(str, sizeof(str),
			"latency(ms) first: %llu.%06llu, max %llu.%06llu\n",
			perf->latency_first / NSEC_PER_MSEC,
			perf->latency_first % NSEC_PER_MSEC,
			perf->latency_max / NSEC_PER_MSEC,
			perf->latency_max % NSEC_PER_MSEC);
	if (seq_write(s, str, num))
		return 0;

	num = scnprintf(str, sizeof(str),
			"process frame time(ms) min: %llu.%06llu, max %llu.%06llu\n",
			perf->min_process_time / NSEC_PER_MSEC,
			perf->min_process_time % NSEC_PER_MSEC,
			perf->max_process_time / NSEC_PER_MSEC,
			perf->max_process_time % NSEC_PER_MSEC);
	if (seq_write(s, str, num))
		return 0;

	if (inst->type == VPU_INST_TYPE_DEC) {
		num = scnprintf(str, sizeof(str), "%s order\n",
				inst->disp_mode == DISP_MODE_DISP_ORDER ? "display" : "decode");
		if (seq_write(s, str, num))
			return 0;
	} else {
		struct enc_wave_param *param = &inst->enc_param;

		num = scnprintf(str, sizeof(str), "profile %d, level %d, tier %d\n",
				param->profile, param->level, param->tier);
		if (seq_write(s, str, num))
			return 0;

		num = scnprintf(str, sizeof(str), "frame_rate %d, idr_period %d, intra_period %d\n",
				param->frame_rate, param->idr_period, param->intra_period);
		if (seq_write(s, str, num))
			return 0;

		num = scnprintf(str, sizeof(str), "rc %d, mode %d, bitrate %d\n",
				param->en_rate_control,
				inst->rc_mode,
				param->enc_bit_rate);
		if (seq_write(s, str, num))
			return 0;

		num = scnprintf(str, sizeof(str),
				"qp %d, i_qp [%d, %d], p_qp [%d, %d], b_qp [%d, %d]\n",
				param->qp,
				param->min_qp_i, param->max_qp_i,
				param->min_qp_p, param->max_qp_p,
				param->min_qp_b, param->max_qp_b);
		if (seq_write(s, str, num))
			return 0;
	}

	return 0;
}

static int wave6_vpu_dbg_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, wave6_vpu_dbg_instance, inode->i_private);
}

static const struct file_operations wave6_vpu_dbg_fops = {
	.owner = THIS_MODULE,
	.open = wave6_vpu_dbg_open,
	.release = single_release,
	.read = seq_read,
};

int wave6_vpu_create_dbgfs_file(struct vpu_instance *inst)
{
	char name[64];

	if (!inst || !inst->dev || !inst->dev->debugfs)
		return -EINVAL;

	scnprintf(name, sizeof(name), "instance.%d", inst->id);
	inst->debugfs = debugfs_create_file((const char *)name,
					    VERIFY_OCTAL_PERMISSIONS(0444),
					    inst->dev->debugfs,
					    inst,
					    &wave6_vpu_dbg_fops);

	return 0;
}

void wave6_vpu_remove_dbgfs_file(struct vpu_instance *inst)
{
	if (!inst || !inst->debugfs)
		return;

	debugfs_remove(inst->debugfs);
	inst->debugfs = NULL;
}
