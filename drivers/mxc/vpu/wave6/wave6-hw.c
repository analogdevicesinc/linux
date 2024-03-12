// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave6 series multi-standard codec IP - wave6 backend logic
 *
 * Copyright (C) 2021 CHIPS&MEDIA INC
 */

#include <linux/iopoll.h>
#include "wave6-vpu.h"
#include "wave6.h"
#include "wave6-regdefine.h"
#include "wave6-av1-cdf-table.h"

#define VPU_BUSY_CHECK_TIMEOUT 3000000
#define MAX_CSC_COEFF_NUM      4

static void wave6_print_reg_err(struct vpu_device *vpu_dev, u32 reg_fail_reason)
{
	char *caller = __builtin_return_address(0);
	struct device *dev = vpu_dev->dev;

	switch (reg_fail_reason) {
	case WAVE6_SYSERR_QUEUEING_FAIL:
		dev_dbg(dev, "%s: queueing failure 0x%x\n", caller, reg_fail_reason);
		break;
	case WAVE6_SYSERR_RESULT_NOT_READY:
		dev_err(dev, "%s: result not ready 0x%x\n", caller, reg_fail_reason);
		break;
	case WAVE6_SYSERR_ACCESS_VIOLATION_HW:
		dev_err(dev, "%s: access violation 0x%x\n", caller, reg_fail_reason);
		break;
	case WAVE6_SYSERR_WATCHDOG_TIMEOUT:
		dev_err(dev, "%s: watchdog timeout 0x%x\n", caller, reg_fail_reason);
		break;
	case WAVE6_SYSERR_BUS_ERROR:
		dev_err(dev, "%s: bus error 0x%x\n", caller, reg_fail_reason);
		break;
	case WAVE6_SYSERR_DOUBLE_FAULT:
		dev_err(dev, "%s: double fault 0x%x\n", caller, reg_fail_reason);
		break;
	case WAVE6_SYSERR_VPU_STILL_RUNNING:
		dev_err(dev, "%s: still running 0x%x\n", caller, reg_fail_reason);
		break;
	default:
		dev_err(dev, "%s: failure: 0x%x\n", caller, reg_fail_reason);
		break;
	}
}

static void wave6_dec_set_display_buffer(struct vpu_instance *vpu_inst,
					 struct frame_buffer fb)
{
	struct dec_info *p_dec_info = &vpu_inst->codec_info->dec_info;
	int index = 0;

	for (index = 0; index < WAVE6_MAX_FBS; index++) {
		if (!p_dec_info->disp_buf[index].buf_y) {
			p_dec_info->disp_buf[index] = fb;
			p_dec_info->disp_buf[index].index = index;
			break;
		}
	}
}

static struct frame_buffer wave6_dec_get_display_buffer(struct vpu_instance *vpu_inst,
							dma_addr_t addr,
							bool remove)
{
	struct dec_info *p_dec_info = &vpu_inst->codec_info->dec_info;
	int index = 0;
	struct frame_buffer fb;

	memset(&fb, 0, sizeof(struct frame_buffer));

	for (index = 0; index < WAVE6_MAX_FBS; index++) {
		if (p_dec_info->disp_buf[index].buf_y == addr) {
			fb = p_dec_info->disp_buf[index];
			if (remove)
				memset(&p_dec_info->disp_buf[index], 0, sizeof(struct frame_buffer));
			break;
		}
	}

	return fb;
}

static int wave6_wait_vpu_busy(struct vpu_device *vpu_dev, unsigned int addr)
{
	u32 data;

	return read_poll_timeout(wave6_vdi_readl, data, data == 0,
				 0, VPU_BUSY_CHECK_TIMEOUT, false, vpu_dev, addr);
}

void wave6_enable_interrupt(struct vpu_device *vpu_dev)
{
	u32 data;

	data  = (1 << INT_WAVE6_ENC_SET_PARAM);
	data |= (1 << INT_WAVE6_ENC_PIC);
	data |= (1 << INT_WAVE6_BSBUF_FULL);
	data |= (1 << INT_WAVE6_INIT_SEQ);
	data |= (1 << INT_WAVE6_DEC_PIC);
	data |= (1 << INT_WAVE6_BSBUF_EMPTY);
	data |= (1 << INT_WAVE6_UPDATE_FB);

	vpu_write_reg(vpu_dev, W6_VPU_VINT_ENABLE, data);
}

static void wave6_load_av1_cdf_table(struct vpu_device *vpu_dev, struct vpu_buf *vb)
{
	const u16 *tbl_data;
	u32 tbl_size = AV1_MAX_CDF_WORDS * AV1_CDF_WORDS_SIZE * 2;

	tbl_data = def_cdf_tbl;
	wave6_vdi_write_memory(vpu_dev, vb, 0, (u8 *)def_cdf_tbl,
			       tbl_size, VDI_128BIT_LITTLE_ENDIAN);
}

void wave6_vpu_check_state(struct vpu_device *vpu_dev)
{
	int state = wave6_vpu_ctrl_get_state(vpu_dev->ctrl);

	if (state == WAVE6_VPU_STATE_PREPARE)
		wave6_vpu_ctrl_wait_done(vpu_dev->ctrl, &vpu_dev->entity);
}

bool wave6_vpu_is_init(struct vpu_device *vpu_dev)
{
	return vpu_read_reg(vpu_dev, W6_VCPU_CUR_PC) != 0;
}

int32_t wave_vpu_get_product_id(struct vpu_device *vpu_dev)
{
	u32 product_id = PRODUCT_ID_NONE;
	u32 val;

	val = vpu_read_reg(vpu_dev, W6_VPU_RET_PRODUCT_VERSION);

	switch (val) {
	case WAVE617_CODE:
		product_id = PRODUCT_ID_617; break;
	case WAVE627_CODE:
		product_id = PRODUCT_ID_627; break;
	case WAVE633_CODE:
	case WAVE637_CODE:
	case WAVE663_CODE:
	case WAVE677_CODE:
		product_id = PRODUCT_ID_637; break;
	default:
		dev_err(vpu_dev->dev, "Invalid product id (%x)\n", val);
		break;
	}
	return product_id;
}

static void wave6_send_command(struct vpu_device *vpu_dev, u32 id, u32 std, u32 cmd)
{
	if (cmd == W6_CREATE_INSTANCE) {
		vpu_write_reg(vpu_dev, W6_CMD_INSTANCE_INFO, (std << 16));

		vpu_write_reg(vpu_dev, W6_VPU_BUSY_STATUS, 1);
		vpu_write_reg(vpu_dev, W6_COMMAND, cmd);

		vpu_write_reg(vpu_dev, W6_VPU_HOST_INT_REQ, 1);
	} else {
		vpu_write_reg(vpu_dev, W6_CMD_INSTANCE_INFO, (std << 16) | (id & 0xffff));

		vpu_write_reg(vpu_dev, W6_VPU_BUSY_STATUS, 1);
		vpu_write_reg(vpu_dev, W6_COMMAND, cmd);

		vpu_write_reg(vpu_dev, W6_VPU_HOST_INT_REQ, 1);
	}
}

static int wave6_send_query(struct vpu_device *vpu_dev, u32 id, u32 std, enum W6_QUERY_OPT query_opt)
{
	int ret;
	u32 reg_val;

	vpu_write_reg(vpu_dev, W6_QUERY_OPTION, query_opt);
	wave6_send_command(vpu_dev, id, std, W6_QUERY);

	ret = wave6_wait_vpu_busy(vpu_dev, W6_VPU_BUSY_STATUS);
	if (ret) {
		dev_err(vpu_dev->dev, "query timed out opt=0x%x\n", query_opt);
		return ret;
	}

	if (!vpu_read_reg(vpu_dev, W6_RET_SUCCESS)) {
		reg_val = vpu_read_reg(vpu_dev, W6_RET_FAIL_REASON);
		wave6_print_reg_err(vpu_dev, reg_val);
		return -EIO;
	}

	return 0;
}

int wave6_vpu_get_version(struct vpu_device *vpu_dev, uint32_t *version_info,
			  uint32_t *revision)
{
	struct vpu_attr *attr = &vpu_dev->attr;
	u32 reg_val;
	u8 *str;
	int ret;
	u32 hw_config_def1, hw_config_feature;

	attr->support_command_queue = vpu_read_reg(vpu_dev, W6_RET_CQ_FLAG);

	if (attr->support_command_queue) {
		ret = wave6_send_query(vpu_dev, 0, 0, W6_QUERY_GET_VPU_INFO);
		if (ret)
			return ret;
	} else {
		wave6_send_command(vpu_dev, 0, 0, W6_GET_VPU_INFO);
		ret = wave6_wait_vpu_busy(vpu_dev, W6_VPU_BUSY_STATUS);
		if (ret) {
			dev_err(vpu_dev->dev, "%s: timeout\n", __func__);
			return ret;
		}

		if (!vpu_read_reg(vpu_dev, W6_RET_SUCCESS)) {
			dev_err(vpu_dev->dev, "%s: failed\n", __func__);
			return -EIO;
		}
	}

	reg_val = vpu_read_reg(vpu_dev, W6_RET_PRODUCT_NAME);
	str = (u8 *)&reg_val;
	attr->product_name[0] = str[3];
	attr->product_name[1] = str[2];
	attr->product_name[2] = str[1];
	attr->product_name[3] = str[0];
	attr->product_name[4] = 0;

	attr->product_id = wave_vpu_get_product_id(vpu_dev);
	attr->product_version = vpu_read_reg(vpu_dev, W6_RET_PRODUCT_VERSION);
	attr->fw_version = vpu_read_reg(vpu_dev, W6_RET_FW_API_VERSION);
	attr->fw_revision = vpu_read_reg(vpu_dev, W6_RET_FW_VERSION);
	hw_config_def1 = vpu_read_reg(vpu_dev, W6_RET_STD_DEF1);
	hw_config_feature = vpu_read_reg(vpu_dev, W6_RET_CONF_FEATURE);

	attr->support_hevc10bit_enc = (hw_config_feature >> 3) & 1;
	attr->support_avc10bit_enc = (hw_config_feature >> 11) & 1;

	attr->support_decoders = 0;
	attr->support_encoders = 0;
	if (attr->product_id == PRODUCT_ID_617) {
		attr->support_decoders  = (((hw_config_def1 >> 2) & 0x01) << STD_HEVC);
		attr->support_decoders |= (((hw_config_def1 >> 3) & 0x01) << STD_AVC);
		attr->support_decoders |= (((hw_config_def1 >> 5) & 0x01) << STD_AV1);
		attr->support_decoders |= (((hw_config_def1 >> 6) & 0x01) << STD_VP9);
	} else if (attr->product_id == PRODUCT_ID_627) {
		attr->support_encoders  = (((hw_config_def1 >> 0) & 0x01) << STD_HEVC);
		attr->support_encoders |= (((hw_config_def1 >> 1) & 0x01) << STD_AVC);
		attr->support_encoders |= (((hw_config_def1 >> 4) & 0x01) << STD_AV1);
	} else if (attr->product_id == PRODUCT_ID_637) {
		attr->support_decoders  = (((hw_config_def1 >> 2) & 0x01) << STD_HEVC);
		attr->support_decoders |= (((hw_config_def1 >> 3) & 0x01) << STD_AVC);
		attr->support_decoders |= (((hw_config_def1 >> 5) & 0x01) << STD_AV1);
		attr->support_decoders |= (((hw_config_def1 >> 6) & 0x01) << STD_VP9);
		attr->support_encoders  = (((hw_config_def1 >> 0) & 0x01) << STD_HEVC);
		attr->support_encoders |= (((hw_config_def1 >> 1) & 0x01) << STD_AVC);
		attr->support_encoders |= (((hw_config_def1 >> 4) & 0x01) << STD_AV1);
	}

	attr->support_dual_core = (hw_config_def1 >> 26) & 0x01;
	attr->support_endian_mask = BIT(VDI_LITTLE_ENDIAN) |
				    BIT(VDI_BIG_ENDIAN) |
				    BIT(VDI_32BIT_LITTLE_ENDIAN) |
				    BIT(VDI_32BIT_BIG_ENDIAN) |
				    (0xffffUL << 16);
	attr->support_bitstream_mode = BIT(BS_MODE_PIC_END);

	if (version_info)
		*version_info = attr->fw_version;
	if (revision)
		*revision = attr->fw_revision;

	return 0;
}

int wave6_vpu_build_up_dec_param(struct vpu_instance *vpu_inst,
				 struct dec_open_param *param)
{
	struct dec_info *p_dec_info = &vpu_inst->codec_info->dec_info;
	struct vpu_attr *attr = &vpu_inst->dev->attr;
	u32 reg_val;
	int ret;

	p_dec_info->cycle_per_tick = 256;
	if (param->inst_buffer.sec_size_core0) {
		p_dec_info->sec_axi_info.use_ip_enable = 1;
		p_dec_info->sec_axi_info.use_lf_row_enable = 1;
	}
	p_dec_info->user_data_enable = BIT(DEC_USERDATA_FLAG_VUI) |
				       BIT(DEC_USERDATA_FLAG_RECOVERY_POINT);
	p_dec_info->user_data_buf_addr = vpu_inst->vui_vbuf.daddr;
	p_dec_info->user_data_buf_size = vpu_inst->vui_vbuf.size;
	switch (vpu_inst->std) {
	case W_HEVC_DEC:
		p_dec_info->seq_change_mask = SEQ_CHANGE_ENABLE_ALL_HEVC;
		break;
	case W_VP9_DEC:
		p_dec_info->seq_change_mask = SEQ_CHANGE_ENABLE_ALL_VP9;
		break;
	case W_AVC_DEC:
		p_dec_info->seq_change_mask = SEQ_CHANGE_ENABLE_ALL_AVC;
		break;
	case W_AV1_DEC:
		p_dec_info->seq_change_mask = SEQ_CHANGE_ENABLE_ALL_AV1;
		break;
	default:
		return -EINVAL;
	}

	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_CREATE_INST_TEMP_BASE, param->inst_buffer.temp_base);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_CREATE_INST_TEMP_SIZE, param->inst_buffer.temp_size);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_CREATE_INST_WORK_BASE, param->inst_buffer.work_base);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_CREATE_INST_WORK_SIZE, param->inst_buffer.work_size);

	reg_val = wave6_convert_endian(param->stream_endian);
	reg_val = (~reg_val & VDI_128BIT_ENDIAN_MASK);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_CREATE_INST_BS_PARAM, reg_val);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_CREATE_INST_ADDR_EXT, param->ext_addr_vcpu);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_CREATE_INST_DISP_MODE, param->disp_mode);

	reg_val = (attr->support_command_queue) ? (COMMAND_QUEUE_DEPTH << 8) : 0;
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_CREATE_INST_CORE_INFO, reg_val | (1 << 4) | (1 << 0));
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_CREATE_INST_SEC_AXI_BASE_CORE0, param->inst_buffer.sec_base_core0);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_CREATE_INST_SEC_AXI_SIZE_CORE0, param->inst_buffer.sec_size_core0);

	reg_val = (param->is_secure_inst << 8) | (param->inst_priority);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_CREATE_INST_PRIORITY, reg_val);

	wave6_send_command(vpu_inst->dev, 0, vpu_inst->std, W6_CREATE_INSTANCE);
	ret = wave6_wait_vpu_busy(vpu_inst->dev, W6_VPU_BUSY_STATUS);
	if (ret) {
		dev_err(vpu_inst->dev->dev, "%s: timeout\n", __func__);
		return ret;
	}

	if (!vpu_read_reg(vpu_inst->dev, W6_RET_SUCCESS)) {
		u32 reason_code = vpu_read_reg(vpu_inst->dev, W6_RET_FAIL_REASON);

		wave6_print_reg_err(vpu_inst->dev, reason_code);
		return -EIO;
	}

	vpu_inst->id = vpu_read_reg(vpu_inst->dev, W6_RET_INSTANCE_ID);

	return 0;
}

int wave6_vpu_dec_init_seq(struct vpu_instance *vpu_inst)
{
	struct dec_info *p_dec_info;
	struct vpu_attr *attr = &vpu_inst->dev->attr;
	u32 cmd_option = INIT_SEQ_NORMAL, bs_option;
	int ret;

	if (!vpu_inst->codec_info)
		return -EINVAL;

	p_dec_info = &vpu_inst->codec_info->dec_info;
	if (p_dec_info->thumbnail_mode)
		cmd_option = INIT_SEQ_W_THUMBNAIL;

	/* set attributes of bitstream buffer controller */
	bs_option = 0;
	switch (p_dec_info->open_param.bitstream_mode) {
	case BS_MODE_INTERRUPT:
		bs_option = 0;
		break;
	case BS_MODE_PIC_END:
		bs_option = BSOPTION_ENABLE_EXPLICIT_END;
		break;
	default:
		return -EINVAL;
	}

	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_INIT_SEQ_BS_RD_PTR, p_dec_info->stream_rd_ptr);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_INIT_SEQ_BS_WR_PTR, p_dec_info->stream_wr_ptr);

	if (p_dec_info->stream_end_flag)
		bs_option = 3;
	if (vpu_inst->std == W_AV1_DEC)
		bs_option |= ((p_dec_info->open_param.av1_format) << 2);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_INIT_SEQ_BS_OPTION, bs_option);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_INIT_SEQ_OPTION, cmd_option);

	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_INIT_USERDATA_MASK, p_dec_info->user_data_enable);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_INIT_USERDATA_BASE, p_dec_info->user_data_buf_addr);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_INIT_USERDATA_SIZE, p_dec_info->user_data_buf_size);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_INIT_USERDATA_PARAM, (VPU_USER_DATA_ENDIAN & VDI_128BIT_ENDIAN_MASK));

	wave6_send_command(vpu_inst->dev, vpu_inst->id, vpu_inst->std, W6_INIT_SEQ);
	if (attr->support_command_queue) {
		ret = wave6_wait_vpu_busy(vpu_inst->dev, W6_VPU_BUSY_STATUS);
		if (ret) {
			dev_err(vpu_inst->dev->dev, "%s: timeout\n", __func__);
			return ret;
		}

		if (!vpu_read_reg(vpu_inst->dev, W6_RET_SUCCESS)) {
			u32 reason_code = vpu_read_reg(vpu_inst->dev, W6_RET_FAIL_REASON);

			wave6_print_reg_err(vpu_inst->dev, reason_code);
			return -EIO;
		}
	}

	return 0;
}

static void wave6_get_dec_seq_result(struct vpu_instance *vpu_inst, struct dec_initial_info *info)
{
	u32 reg_val;
	u32 profile_compatibility_flag;
	u32 left, right, top, bottom;
	int i;

	info->rd_ptr = wave6_vpu_dec_get_rd_ptr(vpu_inst);

	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_PIC_SIZE);
	info->pic_width = ((reg_val >> 16) & 0xffff);
	info->pic_height = (reg_val & 0xffff);
	info->min_frame_buffer_count = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_NUM_REQUIRED_FBC_FB);
	info->frame_buf_delay = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_NUM_REORDER_DELAY);
	info->req_mv_buffer_count = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_NUM_REQUIRED_COL_BUF);

	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_CROP_LEFT_RIGHT);
	left = (reg_val >> 16) & 0xffff;
	right = reg_val & 0xffff;
	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_CROP_TOP_BOTTOM);
	top = (reg_val >> 16) & 0xffff;
	bottom = reg_val & 0xffff;

	info->pic_crop_rect.left = left;
	info->pic_crop_rect.right = info->pic_width - right;
	info->pic_crop_rect.top = top;
	info->pic_crop_rect.bottom = info->pic_height - bottom;

	info->f_rate_numerator = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_FRAME_RATE_NR);
	info->f_rate_denominator = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_FRAME_RATE_DR);

	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_COLOR_SAMPLE_INFO);
	info->luma_bitdepth = (reg_val >> 0) & 0x0f;
	info->chroma_bitdepth = (reg_val >> 4) & 0x0f;
	info->chroma_format_idc = (reg_val >> 8) & 0x0f;
	info->aspect_rate_info = (reg_val >> 16) & 0xff;
	info->is_ext_sar = (info->aspect_rate_info == H264_VUI_SAR_IDC_EXTENDED ? true : false);
	if (info->is_ext_sar)
		info->aspect_rate_info = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_ASPECT_RATIO);
	info->bit_rate = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_BIT_RATE);

	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_SEQ_PARAM);
	info->level = reg_val & 0xff;
	profile_compatibility_flag = (reg_val >> 12) & 0xff;
	info->profile = (reg_val >> 24) & 0x1f;
	info->tier = (reg_val >> 29) & 0x01;

	if (vpu_inst->std == W_HEVC_DEC) {
		if (!info->profile) {
			if ((profile_compatibility_flag & 0x06) == 0x06)
				info->profile = HEVC_PROFILE_MAIN;
			else if ((profile_compatibility_flag & 0x04) == 0x04)
				info->profile = HEVC_PROFILE_MAIN10;
			else if ((profile_compatibility_flag & 0x08) == 0x08)
				info->profile = HEVC_PROFILE_STILLPICTURE;
			else
				info->profile = HEVC_PROFILE_MAIN;
		}
	} else if (vpu_inst->std == W_AVC_DEC) {
		info->profile = (reg_val >> 24) & 0x7f;
	}

	info->user_data.buf_addr = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_USERDATA_BASE);
	info->user_data.size = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_USERDATA_SIZE);
	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_USERDATA_IDC);
	if (reg_val) {
		info->user_data.header = reg_val;
		info->user_data.buf_full = (reg_val & 0x2);
		info->user_data.num = 0;
		for (i = 2; i < 32; i++) {
			if ((reg_val >> i) & 0x1)
				info->user_data.num++;
		}
	} else {
		info->user_data.header = 0;
		info->user_data.buf_full = 0;
		info->user_data.num = 0;
	}
}

static void wave6_update_dec_seq_result(struct vpu_instance *vpu_inst, struct dec_initial_info *info)
{
	u32 reg_val;
	u32 profile_compatibility_flag;
	u32 left, right, top, bottom;

	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_UPDATE_FB_PIC_SIZE);
	info->pic_width = ((reg_val >> 16) & 0xffff);
	info->pic_height = (reg_val & 0xffff);
	info->min_frame_buffer_count = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_UPDATE_FB_NUM_REQUIRED_FBC_FB);
	info->frame_buf_delay = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_UPDATE_FB_NUM_REORDER_DELAY);
	info->req_mv_buffer_count = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_UPDATE_FB_NUM_REQUIRED_COL_BUF);

	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_UPDATE_FB_CROP_LEFT_RIGHT);
	left = (reg_val >> 16) & 0xffff;
	right = reg_val & 0xffff;
	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_UPDATE_FB_CROP_TOP_BOTTOM);
	top = (reg_val >> 16) & 0xffff;
	bottom = reg_val & 0xffff;

	info->pic_crop_rect.left = left;
	info->pic_crop_rect.right = info->pic_width - right;
	info->pic_crop_rect.top = top;
	info->pic_crop_rect.bottom = info->pic_height - bottom;

	info->f_rate_numerator = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_UPDATE_FB_FRAME_RATE_NR);
	info->f_rate_denominator = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_UPDATE_FB_FRAME_RATE_DR);

	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_UPDATE_FB_COLOR_SAMPLE_INFO);
	info->luma_bitdepth = (reg_val >> 0) & 0x0f;
	info->chroma_bitdepth = (reg_val >> 4) & 0x0f;
	info->chroma_format_idc = (reg_val >> 8) & 0x0f;
	info->aspect_rate_info = (reg_val >> 16) & 0xff;
	info->is_ext_sar = (info->aspect_rate_info == H264_VUI_SAR_IDC_EXTENDED ? true : false);
	if (info->is_ext_sar)
		info->aspect_rate_info = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_UPDATE_FB_ASPECT_RATIO);
	info->bit_rate = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_UPDATE_FB_BIT_RATE);

	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_UPDATE_FB_SEQ_PARAM);
	info->level = reg_val & 0xff;
	profile_compatibility_flag = (reg_val >> 12) & 0xff;
	info->profile = (reg_val >> 24) & 0x1f;
	info->tier = (reg_val >> 29) & 0x01;

	if (vpu_inst->std == W_HEVC_DEC) {
		if (!info->profile) {
			if ((profile_compatibility_flag & 0x06) == 0x06)
				info->profile = HEVC_PROFILE_MAIN;
			else if ((profile_compatibility_flag & 0x04) == 0x04)
				info->profile = HEVC_PROFILE_MAIN10;
			else if ((profile_compatibility_flag & 0x08) == 0x08)
				info->profile = HEVC_PROFILE_STILLPICTURE;
			else
				info->profile = HEVC_PROFILE_MAIN;
		}
	} else if (vpu_inst->std == W_AVC_DEC) {
		info->profile = (reg_val >> 24) & 0x7f;
	}
}

int wave6_vpu_dec_get_seq_info(struct vpu_instance *vpu_inst, struct dec_initial_info *info)
{
	struct vpu_attr *attr = &vpu_inst->dev->attr;
	int ret = 0;

	if (attr->support_command_queue) {
		ret = wave6_send_query(vpu_inst->dev, vpu_inst->id, vpu_inst->std, W6_QUERY_GET_RESULT);
		if (ret)
			return ret;

		if (vpu_read_reg(vpu_inst->dev, W6_RET_DEC_DECODING_SUCCESS) != 1) {
			info->err_reason = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_ERR_INFO);
			ret = -EIO;
		} else {
			info->warn_info = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_WARN_INFO);
		}
	} else {
		if (!vpu_read_reg(vpu_inst->dev, W6_RET_SUCCESS)) {
			info->err_reason = vpu_read_reg(vpu_inst->dev, W6_RET_FAIL_REASON);

			wave6_print_reg_err(vpu_inst->dev, info->err_reason);
			return -EIO;
		}
	}

	wave6_get_dec_seq_result(vpu_inst, info);

	return ret;
}

int wave6_vpu_dec_register_frame_buffer(struct vpu_instance *vpu_inst,
					struct frame_buffer *fb_arr,
					enum tiled_map_type map_type, u32 count)
{
	struct vpu_attr *attr = &vpu_inst->dev->attr;
	struct dec_info *p_dec_info = &vpu_inst->codec_info->dec_info;
	size_t fbc_remain, mv_remain, fbc_idx = 0, mv_idx = 0;
	size_t i, k, group_num, mv_count;
	dma_addr_t fbc_cr_tbl_addr;
	u32 reg_val;
	u32 endian;
	int ret;

	mv_count = p_dec_info->initial_info.req_mv_buffer_count;

	if (vpu_inst->std == W_AV1_DEC) {
		if (!p_dec_info->vb_def_cdf.daddr)
			return -EINVAL;

		wave6_load_av1_cdf_table(vpu_inst->dev, &p_dec_info->vb_def_cdf);
		if (!p_dec_info->vb_pre_ent.daddr && attr->support_command_queue)
			return -EINVAL;
	} else if (vpu_inst->std == W_VP9_DEC) {
		if (!p_dec_info->vb_seg_map.daddr)
			return -EINVAL;
		if (!p_dec_info->vb_pre_ent.daddr && attr->support_command_queue)
			return -EINVAL;
	}

	for (i = 0; i < count; i++) {
		if (!p_dec_info->vb_fbc_y_tbl[i].daddr)
			return -EINVAL;
		if (!p_dec_info->vb_fbc_c_tbl[i].daddr)
			return -EINVAL;
	}
	for (i = 0; i < mv_count; i++) {
		if (!p_dec_info->vb_mv[i].daddr)
			return -EINVAL;
	}

	endian = wave6_convert_endian(p_dec_info->open_param.frame_endian);

	reg_val = (p_dec_info->initial_info.pic_width << 16) |
		  (p_dec_info->initial_info.pic_height);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_FB_PIC_SIZE, reg_val);
	reg_val = (p_dec_info->initial_info.chroma_format_idc << 25) |
		  (p_dec_info->initial_info.luma_bitdepth << 21) |
		  (p_dec_info->initial_info.chroma_bitdepth << 17);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_FB_COMMON_PIC_INFO, reg_val);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_FB_DEFAULT_CDF, p_dec_info->vb_def_cdf.daddr);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_FB_SEGMAP, p_dec_info->vb_seg_map.daddr);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_FB_MV_COL_PRE_ENT, p_dec_info->vb_pre_ent.daddr);

	fbc_remain = count;
	mv_remain = mv_count;
	group_num = (count > mv_count) ? ((ALIGN(count, 16) / 16) - 1) : ((ALIGN(mv_count, 16) / 16) - 1);
	for (i = 0; i <= group_num; i++) {
		bool first_group = (i == 0) ? true : false;
		bool last_group = (i == group_num) ? true : false;
		u32 set_fbc_num = (fbc_remain >= 16) ? 16 : fbc_remain;
		u32 set_mv_num = (mv_remain >= 16) ? 16 : mv_remain;
		u32 fbc_start_no = i * 16;
		u32 fbc_end_no = fbc_start_no + set_fbc_num - 1;
		u32 mv_start_no = i * 16;
		u32 mv_end_no = mv_start_no + set_mv_num - 1;

		reg_val = (p_dec_info->open_param.enable_non_ref_fbc_write << 26) |
			  (endian << 16) |
			  (last_group << 4) |
			  (first_group << 3);
		vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_FB_OPTION, reg_val);

		reg_val = (fbc_start_no << 24) |
			  (fbc_end_no << 16) |
			  (mv_start_no << 5) |
			  (mv_end_no << 0);
		vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_FB_NUM, reg_val);

		for (k = 0; k < set_fbc_num; k++) {
			vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_FB_FBC_Y0 + (k * 24),
				      fb_arr[fbc_idx].buf_y);
			vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_FB_FBC_C0 + (k * 24),
				      fb_arr[fbc_idx].buf_cb);
			vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_FB_FBC_CR0 + (k * 8),
				      fb_arr[fbc_idx].buf_cr);
			vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_FB_FBC_Y_OFFSET0 + (k * 24),
				      p_dec_info->vb_fbc_y_tbl[fbc_idx].daddr);
			vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_FB_FBC_C_OFFSET0 + (k * 24),
				      p_dec_info->vb_fbc_c_tbl[fbc_idx].daddr);
			fbc_cr_tbl_addr = p_dec_info->vb_fbc_c_tbl[fbc_idx].daddr +
						(p_dec_info->vb_fbc_c_tbl[fbc_idx].size >> 1);
			vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_FB_FBC_CR_OFFSET0 + (k * 8),
				      fbc_cr_tbl_addr);
			fbc_idx++;
		}
		fbc_remain -= k;

		for (k = 0; k < set_mv_num; k++) {
			vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_FB_MV_COL0 + (k * 24),
				      p_dec_info->vb_mv[mv_idx].daddr);
			mv_idx++;
		}
		mv_remain -= k;

		wave6_send_command(vpu_inst->dev, vpu_inst->id, vpu_inst->std, W6_SET_FB);
		ret = wave6_wait_vpu_busy(vpu_inst->dev, W6_VPU_BUSY_STATUS);
		if (ret) {
			dev_err(vpu_inst->dev->dev, "%s: timeout\n", __func__);
			return ret;
		}
	}

	if (!vpu_read_reg(vpu_inst->dev, W6_RET_SUCCESS))
		return -EIO;

	return 0;
}

int wave6_vpu_dec_register_display_buffer(struct vpu_instance *vpu_inst, struct frame_buffer fb)
{
	int ret;
	struct dec_info *p_dec_info = &vpu_inst->codec_info->dec_info;
	u32 reg_val, cbcr_interleave, nv21;
	u32 endian;
	u32 addr_y, addr_cb, addr_cr;
	u32 pixel_order = 1;
	u32 color_format = 0;
	u32 bwb_flag = 1;
	u32 justified = WTL_RIGHT_JUSTIFIED;
	u32 format_no = WTL_PIXEL_8BIT;

	cbcr_interleave = vpu_inst->cbcr_interleave;
	nv21 = vpu_inst->nv21;

	endian = wave6_convert_endian(p_dec_info->open_param.frame_endian);

	switch (p_dec_info->wtl_format) {
	case FORMAT_420:
	case FORMAT_420_P10_16BIT_MSB:
	case FORMAT_420_P10_16BIT_LSB:
	case FORMAT_420_P10_32BIT_MSB:
	case FORMAT_420_P10_32BIT_LSB:
		color_format = 1;
		break;
	case FORMAT_422:
	case FORMAT_422_P10_16BIT_MSB:
	case FORMAT_422_P10_16BIT_LSB:
	case FORMAT_422_P10_32BIT_MSB:
	case FORMAT_422_P10_32BIT_LSB:
		color_format = 2;
		break;
	case FORMAT_444:
	case FORMAT_444_P10_16BIT_MSB:
	case FORMAT_444_P10_16BIT_LSB:
	case FORMAT_444_P10_32BIT_MSB:
	case FORMAT_444_P10_32BIT_LSB:
		color_format = 3;
		break;
	case FORMAT_400:
	case FORMAT_400_P10_16BIT_MSB:
	case FORMAT_400_P10_16BIT_LSB:
	case FORMAT_400_P10_32BIT_MSB:
	case FORMAT_400_P10_32BIT_LSB:
		color_format = 0;
		break;
	default:
		return -EINVAL;
	}

	reg_val = (color_format << 3) |
		  (vpu_inst->scaler_info.scale_mode << 1) |
		  (vpu_inst->scaler_info.enable);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_DISP_SCL_PARAM, reg_val);
	reg_val = (vpu_inst->scaler_info.width << 16) |
		  (vpu_inst->scaler_info.height);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_DISP_SCL_PIC_SIZE, reg_val);
	reg_val = (p_dec_info->initial_info.pic_width << 16) |
		  (p_dec_info->initial_info.pic_height);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_DISP_PIC_SIZE, reg_val);

	switch (p_dec_info->wtl_format) {
	case FORMAT_420_P10_16BIT_MSB:
	case FORMAT_422_P10_16BIT_MSB:
	case FORMAT_444_P10_16BIT_MSB:
	case FORMAT_400_P10_16BIT_MSB:
		justified = WTL_RIGHT_JUSTIFIED;
		format_no = WTL_PIXEL_16BIT;
		break;
	case FORMAT_420_P10_16BIT_LSB:
	case FORMAT_422_P10_16BIT_LSB:
	case FORMAT_444_P10_16BIT_LSB:
	case FORMAT_400_P10_16BIT_LSB:
		justified = WTL_LEFT_JUSTIFIED;
		format_no = WTL_PIXEL_16BIT;
		break;
	case FORMAT_420_P10_32BIT_MSB:
	case FORMAT_422_P10_32BIT_MSB:
	case FORMAT_444_P10_32BIT_MSB:
	case FORMAT_400_P10_32BIT_MSB:
		justified = WTL_RIGHT_JUSTIFIED;
		format_no = WTL_PIXEL_32BIT;
		break;
	case FORMAT_420_P10_32BIT_LSB:
	case FORMAT_422_P10_32BIT_LSB:
	case FORMAT_444_P10_32BIT_LSB:
	case FORMAT_400_P10_32BIT_LSB:
		justified = WTL_LEFT_JUSTIFIED;
		format_no = WTL_PIXEL_32BIT;
		break;
	default:
		break;
	}

	reg_val = (bwb_flag << 28) | (color_format << 24) | (pixel_order << 23) |
		  (justified << 22) | (format_no << 20) |
		  (nv21 << 17) | (cbcr_interleave << 16) | (fb.stride);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_DISP_COMMON_PIC_INFO, reg_val);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_DISP_OPTION, (endian << 16));
	reg_val = (fb.luma_bitdepth << 22) |
		  (fb.chroma_bitdepth << 18) |
		  (fb.chroma_format_idc << 16);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_DISP_PIC_INFO, reg_val);

	if (p_dec_info->open_param.cbcr_order == CBCR_ORDER_REVERSED) {
		addr_y = fb.buf_y;
		addr_cb = fb.buf_cr;
		addr_cr = fb.buf_cb;
	} else {
		addr_y = fb.buf_y;
		addr_cb = fb.buf_cb;
		addr_cr = fb.buf_cr;
	}

	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_DISP_Y_BASE, addr_y);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_DISP_CB_BASE, addr_cb);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_SET_DISP_CR_BASE, addr_cr);

	wave6_send_command(vpu_inst->dev, vpu_inst->id, vpu_inst->std, W6_DEC_SET_DISP_BUF);
	ret = wave6_wait_vpu_busy(vpu_inst->dev, W6_VPU_BUSY_STATUS);
	if (ret) {
		dev_err(vpu_inst->dev->dev, "%s: timeout\n", __func__);
		return ret;
	}

	if (!vpu_read_reg(vpu_inst->dev, W6_RET_SUCCESS))
		return -EIO;

	wave6_dec_set_display_buffer(vpu_inst, fb);

	return 0;
}

int wave6_vpu_dec_update_fb(struct vpu_instance *inst, struct frame_buffer *fb, int mv_index)
{
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	int ret;
	int fbc_index = -1;
	u32 addr_fbc_y = 0, addr_fbc_c = 0, addr_fbc_cr = 0;
	u32 addr_fbc_y_tbl = 0, addr_fbc_c_tbl = 0, addr_fbc_cr_tbl = 0;
	u32 addr_mv_col = 0;
	u32 reg_val;

	if (fb && fb->index >= 0 && fb->index < WAVE6_MAX_FBS) {
		fbc_index = fb->index;
		addr_fbc_y = fb->buf_y;
		addr_fbc_c = fb->buf_cb;
		addr_fbc_cr = fb->buf_cr;
		addr_fbc_y_tbl = p_dec_info->vb_fbc_y_tbl[fbc_index].daddr;
		addr_fbc_c_tbl = p_dec_info->vb_fbc_c_tbl[fbc_index].daddr;
		addr_fbc_cr_tbl = p_dec_info->vb_fbc_c_tbl[fbc_index].daddr +
					(p_dec_info->vb_fbc_c_tbl[fbc_index].size >> 1);
	}
	if (mv_index >= 0 && mv_index < WAVE6_MAX_FBS)
		addr_mv_col = p_dec_info->vb_mv[mv_index].daddr;

	reg_val = (p_dec_info->initial_info.chroma_format_idc << 25) |
		  (p_dec_info->initial_info.luma_bitdepth << 21) |
		  (p_dec_info->initial_info.chroma_bitdepth << 17);
	vpu_write_reg(inst->dev, W6_CMD_DEC_UPDATE_FB_COMMON_PIC_INFO, reg_val);
	reg_val = (p_dec_info->initial_info.pic_width << 16) |
		  (p_dec_info->initial_info.pic_height);
	vpu_write_reg(inst->dev, W6_CMD_DEC_UPDATE_FB_PIC_SIZE, reg_val);
	vpu_write_reg(inst->dev, W6_CMD_DEC_UPDATE_FB_FBC_Y, addr_fbc_y);
	vpu_write_reg(inst->dev, W6_CMD_DEC_UPDATE_FB_FBC_C, addr_fbc_c);
	vpu_write_reg(inst->dev, W6_CMD_DEC_UPDATE_FB_FBC_CR, addr_fbc_cr);
	vpu_write_reg(inst->dev, W6_CMD_DEC_UPDATE_FB_FBC_Y_OFFSET, addr_fbc_y_tbl);
	vpu_write_reg(inst->dev, W6_CMD_DEC_UPDATE_FB_FBC_C_OFFSET, addr_fbc_c_tbl);
	vpu_write_reg(inst->dev, W6_CMD_DEC_UPDATE_FB_FBC_CR_OFFSET, addr_fbc_cr_tbl);
	vpu_write_reg(inst->dev, W6_CMD_DEC_UPDATE_FB_MV_COL, addr_mv_col);
	reg_val = ((mv_index & 0xff) << 8) | ((fbc_index & 0xff) << 0);
	vpu_write_reg(inst->dev, W6_CMD_DEC_UPDATE_FB_INDICES, reg_val);
	vpu_write_reg(inst->dev, W6_CMD_DEC_SET_FB_OPTION, 1);

	wave6_send_command(inst->dev, inst->id, inst->std, W6_SET_FB);
	ret = wave6_wait_vpu_busy(inst->dev, W6_VPU_BUSY_STATUS);
	if (ret) {
		dev_err(inst->dev->dev, "%s: timeout\n", __func__);
		return ret;
	}

	if (!vpu_read_reg(inst->dev, W6_RET_SUCCESS))
		return -EIO;

	return 0;
}

int wave6_vpu_dec_get_update_fb_info(struct vpu_instance *inst,
				     struct dec_update_fb_info *info)
{
	struct dec_info *p_dec_info = &inst->codec_info->dec_info;
	struct vpu_attr *attr = &inst->dev->attr;
	bool new_sequence, alloc_disp;
	u32 val, index;
	int ret;

	if (attr->support_command_queue) {
		ret = wave6_send_query(inst->dev, inst->id, inst->std, W6_QUERY_GET_UPDATE_FB_STATUS);
		if (ret)
			return ret;

		val = vpu_read_reg(inst->dev, W6_RET_DEC_UPDATE_FB_OPT);

		new_sequence = ((val >> 13) & 0x01);
		if (new_sequence) {
			info->state |= UPDATE_FB_STATE_SEQ_CHANGE;
			info->sequence_changed = vpu_read_reg(inst->dev, W6_RET_DEC_UPDATE_FB_NOTIFICATION);

			wave6_update_dec_seq_result(inst, &p_dec_info->initial_info);

			p_dec_info->initial_info.sequence_no++;
		}

		alloc_disp = ((val >> 12) & 0x01);
		if (alloc_disp)
			info->state |= UPDATE_FB_STATE_ALLOC_DISP;

		index = ((val >> 6) & 0x3f);
		if (index < WAVE6_MAX_FBS) {
			info->state |= UPDATE_FB_STATE_ALLOC_MV;
			info->mv_index = index;
		} else {
			info->mv_index = -1;
		}

		index = (val & 0x3f);
		if (index < WAVE6_MAX_FBS) {
			info->state |= UPDATE_FB_STATE_ALLOC_FBC;
			info->fbc_index = index;
		} else {
			info->fbc_index = -1;
		}

		info->release_disp_frame_num = vpu_read_reg(inst->dev, W6_RET_DEC_UPDATE_FB_RELEASE_LINEAR_NUM);
		for (index = 0; index < WAVE6_MAX_FBS; index++) {
			struct frame_buffer fb;
			dma_addr_t addr = vpu_read_reg(inst->dev, W6_RET_DEC_UPDATE_FB_RELEASE_LINEAR_ADDR_0 + index * 4);

			fb = wave6_dec_get_display_buffer(inst, addr, true);
			info->release_disp_frame_addr[index] = fb.buf_y;
		}
	} else {
		if (!vpu_read_reg(inst->dev, W6_RET_SUCCESS))
			return -EIO;

		val = vpu_read_reg(inst->dev, W6_RET_DEC_FB_UPDATE_REQ_INFO);

		new_sequence = vpu_read_reg(inst->dev, W6_RET_DEC_NOTIFICATION);
		if (new_sequence) {
			info->state |= UPDATE_FB_STATE_SEQ_CHANGE;
			info->sequence_changed = new_sequence;

			wave6_get_dec_seq_result(inst, &p_dec_info->initial_info);

			p_dec_info->initial_info.sequence_no++;
		}

		alloc_disp = ((val >> 12) & 0x01);
		if (alloc_disp)
			info->state |= UPDATE_FB_STATE_ALLOC_DISP;

		index = ((val >> 6) & 0x3f);
		if (index < WAVE6_MAX_FBS) {
			info->state |= UPDATE_FB_STATE_ALLOC_MV;
			info->mv_index = index;
		} else {
			info->mv_index = -1;
		}

		index = (val & 0x3f);
		if (index < WAVE6_MAX_FBS) {
			info->state |= UPDATE_FB_STATE_ALLOC_FBC;
			info->fbc_index = index;
		} else {
			info->fbc_index = -1;
		}

		val = vpu_read_reg(inst->dev, W6_RET_DEC_RELEASE_IDC);
		for (index = 0; index < WAVE6_MAX_FBS; index++) {
			struct frame_buffer fb;
			dma_addr_t addr;

			if (!(val & (1 << index)))
				continue;

			addr = vpu_read_reg(inst->dev, W6_RET_DEC_DISP_LINEAR_ADDR_0 + index * 4);
			fb = wave6_dec_get_display_buffer(inst, addr, true);
			info->release_disp_frame_addr[info->release_disp_frame_num] = fb.buf_y;
			info->release_disp_frame_num++;
		}
	}

	memcpy((void *)&info->sequence, (void *)&p_dec_info->initial_info,
	       sizeof(struct dec_initial_info));

	return 0;
}

int wave6_vpu_decode(struct vpu_instance *vpu_inst, struct dec_param *option, u32 *fail_res)
{
	struct dec_info *p_dec_info = &vpu_inst->codec_info->dec_info;
	struct dec_open_param *p_open_param = &p_dec_info->open_param;
	struct vpu_attr *attr = &vpu_inst->dev->attr;
	u32 mode_option = DEC_PIC_NORMAL, bs_option, reg_val;
	int ret;

	if (p_dec_info->thumbnail_mode) {
		mode_option = DEC_PIC_W_THUMBNAIL;
	} else if (option->skipframe_mode) {
		switch (option->skipframe_mode) {
		case WAVE_SKIPMODE_NON_IRAP:
			mode_option = SKIP_NON_IRAP;
			break;
		case WAVE_SKIPMODE_NON_REF:
			mode_option = SKIP_NON_REF_PIC;
			break;
		default:
			break;
		}
	}

	bs_option = 0;
	switch (p_open_param->bitstream_mode) {
	case BS_MODE_INTERRUPT:
		bs_option = 0;
		break;
	case BS_MODE_PIC_END:
		bs_option = BSOPTION_ENABLE_EXPLICIT_END;
		break;
	default:
		return -EINVAL;
	}

	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_PIC_BS_RD_PTR, p_dec_info->stream_rd_ptr);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_PIC_BS_WR_PTR, p_dec_info->stream_wr_ptr);
	if (p_dec_info->stream_end_flag)
		bs_option = 3;
	if (vpu_inst->std == W_AV1_DEC)
		bs_option |= ((p_open_param->av1_format) << 2);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_PIC_BS_OPTION, bs_option);

	reg_val = (p_dec_info->sec_axi_info.use_ip_enable << 1) |
		  (p_dec_info->sec_axi_info.use_lf_row_enable << 0);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_PIC_USE_SEC_AXI, reg_val);

	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_PIC_USERDATA_MASK, p_dec_info->user_data_enable);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_PIC_USERDATA_BASE, p_dec_info->user_data_buf_addr);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_PIC_USERDATA_SIZE, p_dec_info->user_data_buf_size);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_PIC_USERDATA_PARAM, (VPU_USER_DATA_ENDIAN & VDI_128BIT_ENDIAN_MASK));

	reg_val = (option->disable_film_grain << 6) |
		  (option->cra_as_bla_flag << 5) |
		  (mode_option);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_PIC_OPTION, reg_val);
	reg_val = (DECODE_ALL_SPATIAL_LAYERS << 9) |
		  (TEMPORAL_ID_MODE_ABSOLUTE << 8) |
		  (DECODE_ALL_TEMPORAL_LAYERS);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_PIC_TEMPORAL_ID_PLUS1, reg_val);
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_PIC_SEQ_CHANGE_ENABLE_FLAG, p_dec_info->seq_change_mask);
	reg_val = ((option->timestamp.hour & 0x1F) << 26) |
		  ((option->timestamp.min & 0x3F) << 20) |
		  ((option->timestamp.sec & 0x3F) << 14) |
		  ((option->timestamp.ms & 0x3FFF));
	vpu_write_reg(vpu_inst->dev, W6_CMD_DEC_PIC_TIMESTAMP, reg_val);

	wave6_send_command(vpu_inst->dev, vpu_inst->id, vpu_inst->std, W6_DEC_PIC);
	if (attr->support_command_queue) {
		ret = wave6_wait_vpu_busy(vpu_inst->dev, W6_VPU_BUSY_STATUS);
		if (ret) {
			dev_err(vpu_inst->dev->dev, "%s: timeout\n", __func__);
			return ret;
		}

		if (!vpu_read_reg(vpu_inst->dev, W6_RET_SUCCESS)) {
			*fail_res = vpu_read_reg(vpu_inst->dev, W6_RET_FAIL_REASON);
			wave6_print_reg_err(vpu_inst->dev, *fail_res);
			return -EIO;
		}
	}

	return 0;
}

int wave6_vpu_dec_get_result(struct vpu_instance *vpu_inst, struct dec_output_info *result)
{
	struct dec_info *p_dec_info = &vpu_inst->codec_info->dec_info;
	struct vpu_attr *attr = &vpu_inst->dev->attr;
	u32 reg_val, nal_unit_type, i;
	int decoded_index = -1, display_index = -1;
	int ret;

	if (attr->support_command_queue) {
		ret = wave6_send_query(vpu_inst->dev, vpu_inst->id, vpu_inst->std, W6_QUERY_GET_RESULT);
		if (ret)
			return ret;

		result->decoding_success = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_DECODING_SUCCESS);
		if (!result->decoding_success)
			result->error_reason = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_ERR_INFO);
		else
			result->warn_info = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_WARN_INFO);
	} else {
		result->decoding_success = vpu_read_reg(vpu_inst->dev, W6_RET_SUCCESS);
		if (!result->decoding_success) {
			result->error_reason = vpu_read_reg(vpu_inst->dev, W6_RET_FAIL_REASON);

			wave6_print_reg_err(vpu_inst->dev, result->error_reason);
			return -EIO;
		}
	}

	result->user_data.buf_addr = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_USERDATA_BASE);
	result->user_data.size = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_USERDATA_SIZE);
	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_USERDATA_IDC);
	if (reg_val) {
		result->user_data.header = reg_val;
		result->user_data.buf_full = (reg_val & 0x2);
		result->user_data.num = 0;
		for (i = 2; i < 32; i++) {
			if ((reg_val >> i) & 0x1)
				result->user_data.num++;
		}
	} else {
		result->user_data.header = 0;
		result->user_data.buf_full = 0;
		result->user_data.num = 0;
	}

	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_PIC_TYPE);

	nal_unit_type = (reg_val & 0x3f0) >> 4;
	result->nal_type = nal_unit_type;

	if (vpu_inst->std == W_VP9_DEC) {
		if (reg_val & 0x01)
			result->pic_type = PIC_TYPE_I;
		else if (reg_val & 0x02)
			result->pic_type = PIC_TYPE_P;
		else if (reg_val & 0x04)
			result->pic_type = PIC_TYPE_REPEAT;
		else
			result->pic_type = PIC_TYPE_MAX;
	} else if (vpu_inst->std == W_HEVC_DEC) {
		if (reg_val & 0x04)
			result->pic_type = PIC_TYPE_B;
		else if (reg_val & 0x02)
			result->pic_type = PIC_TYPE_P;
		else if (reg_val & 0x01)
			result->pic_type = PIC_TYPE_I;
		else
			result->pic_type = PIC_TYPE_MAX;
		if ((nal_unit_type == 19 || nal_unit_type == 20) && result->pic_type == PIC_TYPE_I)
			result->pic_type = PIC_TYPE_IDR;
	} else if (vpu_inst->std == W_AVC_DEC) {
		if (reg_val & 0x04)
			result->pic_type = PIC_TYPE_B;
		else if (reg_val & 0x02)
			result->pic_type = PIC_TYPE_P;
		else if (reg_val & 0x01)
			result->pic_type = PIC_TYPE_I;
		else
			result->pic_type = PIC_TYPE_MAX;
		if (nal_unit_type == 5 && result->pic_type == PIC_TYPE_I)
			result->pic_type = PIC_TYPE_IDR;
	} else {
		switch (reg_val & 0x07) {
		case 0:
			result->pic_type = PIC_TYPE_KEY; break;
		case 1:
			result->pic_type = PIC_TYPE_INTER; break;
		case 2:
			result->pic_type = PIC_TYPE_AV1_INTRA; break;
		case 3:
			result->pic_type = PIC_TYPE_AV1_SWITCH; break;
		default:
			result->pic_type = PIC_TYPE_MAX; break;
		}
	}

	result->output_flag = (reg_val >> 31) & 0x1;
	result->ctu_size = 16 << ((reg_val >> 10) & 0x3);

	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_DECODED_FLAG);
	if (reg_val) {
		struct frame_buffer fb;
		dma_addr_t addr = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_DECODED_ADDR);

		fb = wave6_dec_get_display_buffer(vpu_inst, addr, false);
		result->frame_decoded_addr = addr;
		result->frame_decoded_flag = true;
		decoded_index = fb.index;
	}

	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_DISPLAY_FLAG);
	if (reg_val) {
		struct frame_buffer fb;
		dma_addr_t addr = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_DISPLAY_ADDR);

		fb = wave6_dec_get_display_buffer(vpu_inst, addr, false);
		result->frame_display_addr = addr;
		result->frame_display_flag = true;
		display_index = fb.index;
	}

	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_DISP_IDC);
	for (i = 0; i < WAVE6_MAX_FBS; i++) {
		if (reg_val & (1 << i)) {
			dma_addr_t addr = vpu_read_reg(vpu_inst->dev,
						       W6_RET_DEC_DISP_LINEAR_ADDR_0 + i * 4);

			result->disp_frame_addr[result->disp_frame_num] = addr;
			result->disp_frame_num++;
		}
	}

	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_RELEASE_IDC);
	for (i = 0; i < WAVE6_MAX_FBS; i++) {
		if (reg_val & (1 << i)) {
			struct frame_buffer fb;
			dma_addr_t addr = vpu_read_reg(vpu_inst->dev,
						       W6_RET_DEC_DISP_LINEAR_ADDR_0 + i * 4);

			fb = wave6_dec_get_display_buffer(vpu_inst, addr, true);
			result->release_disp_frame_addr[result->release_disp_frame_num] = fb.buf_y;
			result->release_disp_frame_num++;
		}
	}

	result->stream_end_flag = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_STREAM_END);
	result->notification_flag  = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_NOTIFICATION);

	if (vpu_inst->std == W_HEVC_DEC) {
		result->decoded_poc = -1;
		result->display_poc = -1;
		if (decoded_index >= 0)
			result->decoded_poc = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_PIC_POC);
	} else if (vpu_inst->std == W_AVC_DEC) {
		result->decoded_poc = -1;
		result->display_poc = -1;
		if (decoded_index >= 0)
			result->decoded_poc = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_PIC_POC);
	} else if (vpu_inst->std == W_AV1_DEC) {
		result->decoded_poc = -1;
		result->display_poc = -1;
		if (decoded_index >= 0)
			result->decoded_poc = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_PIC_POC);

		reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_PIC_PARAM);
		result->av1_info.allow_intra_bc = (reg_val >> 0) & 0x1;
		result->av1_info.allow_screen_content_tools = (reg_val >> 1) & 0x1;
	}

	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_PIC_SIZE);
	result->dec_pic_width = reg_val >> 16;
	result->dec_pic_height = reg_val & 0xffff;

	result->num_of_err_m_bs = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_ERR_CTB_NUM) >> 16;
	result->num_of_tot_m_bs = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_ERR_CTB_NUM) & 0xffff;
	result->byte_pos_frame_start = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_AU_START_POS);
	result->byte_pos_frame_end = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_AU_END_POS);

	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_RECOVERY_POINT);
	result->h265_rp_sei.recovery_poc_cnt = reg_val & 0xFFFF;
	result->h265_rp_sei.exact_match_flag = (reg_val >> 16) & 0x01;
	result->h265_rp_sei.broken_link_flag = (reg_val >> 17) & 0x01;
	result->h265_rp_sei.exist = (reg_val >> 18) & 0x01;
	if (!result->h265_rp_sei.exist) {
		result->h265_rp_sei.recovery_poc_cnt = 0;
		result->h265_rp_sei.exact_match_flag = 0;
		result->h265_rp_sei.broken_link_flag = 0;
	}

	result->last_frame_in_au = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_LAST_FRAME_FLAG);

	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_TIMESTAMP);
	result->timestamp.hour = (reg_val >> 26) & 0x1F;
	result->timestamp.min = (reg_val >> 20) & 0x3F;
	result->timestamp.sec = (reg_val >> 14) & 0x3F;
	result->timestamp.ms = reg_val & 0x3FFF;

	result->cycle.host_cmd_s = vpu_read_reg(vpu_inst->dev, W6_RET_CMD_CQ_IN_TICK);
	result->cycle.host_cmd_e = vpu_read_reg(vpu_inst->dev, W6_RET_CMD_RQ_OUT_TICK);
	result->cycle.proc_s = vpu_read_reg(vpu_inst->dev, W6_RET_CMD_HW_RUN_TICK);
	result->cycle.proc_e = vpu_read_reg(vpu_inst->dev, W6_RET_CMD_HW_DONE_TICK);
	result->cycle.vpu_s = vpu_read_reg(vpu_inst->dev, W6_RET_CMD_FW_RUN_TICK);
	result->cycle.vpu_e = vpu_read_reg(vpu_inst->dev, W6_RET_CMD_FW_DONE_TICK);
	result->cycle.frame_cycle = (result->cycle.vpu_e - result->cycle.host_cmd_s) * p_dec_info->cycle_per_tick;
	result->cycle.proc_cycle = (result->cycle.proc_e - result->cycle.proc_s) * p_dec_info->cycle_per_tick;
	result->cycle.vpu_cycle = ((result->cycle.vpu_e - result->cycle.vpu_s) - (result->cycle.proc_e - result->cycle.proc_s)) * p_dec_info->cycle_per_tick;

	if (decoded_index >= 0 && decoded_index < WAVE6_MAX_FBS) {
		struct vpu_rect rect_info;

		rect_info.left = 0;
		rect_info.right = result->dec_pic_width;
		rect_info.top = 0;
		rect_info.bottom = result->dec_pic_height;

		if (vpu_inst->std == W_HEVC_DEC || vpu_inst->std == W_AVC_DEC) {
			rect_info = p_dec_info->initial_info.pic_crop_rect;
			p_dec_info->dec_out_info[decoded_index].decoded_poc = result->decoded_poc;
		}

		result->rc_decoded.left = rect_info.left;
		p_dec_info->dec_out_info[decoded_index].rc_decoded.left = rect_info.left;
		result->rc_decoded.right = rect_info.right;
		p_dec_info->dec_out_info[decoded_index].rc_decoded.right = rect_info.right;
		result->rc_decoded.top = rect_info.top;
		p_dec_info->dec_out_info[decoded_index].rc_decoded.top = rect_info.top;
		result->rc_decoded.bottom = rect_info.bottom;
		p_dec_info->dec_out_info[decoded_index].rc_decoded.bottom = rect_info.bottom;
	} else {
		result->rc_decoded.left = 0;
		result->rc_decoded.right = result->dec_pic_width;
		result->rc_decoded.top = 0;
		result->rc_decoded.bottom = result->dec_pic_height;
	}

	if (display_index >= 0 && display_index < WAVE6_MAX_FBS) {
		result->rc_display.left = p_dec_info->dec_out_info[display_index].rc_decoded.left;
		result->rc_display.right = p_dec_info->dec_out_info[display_index].rc_decoded.right;
		result->rc_display.top = p_dec_info->dec_out_info[display_index].rc_decoded.top;
		result->rc_display.bottom = p_dec_info->dec_out_info[display_index].rc_decoded.bottom;

		if (vpu_inst->std == W_HEVC_DEC || vpu_inst->std == W_AVC_DEC)
			result->display_poc = p_dec_info->dec_out_info[display_index].decoded_poc;

		result->disp_pic_width = p_dec_info->dec_out_info[display_index].dec_pic_width;
		result->disp_pic_height = p_dec_info->dec_out_info[display_index].dec_pic_height;
	} else {
		result->rc_display.left = 0;
		result->rc_display.right = 0;
		result->rc_display.top = 0;
		result->rc_display.bottom = 0;
		result->disp_pic_width = 0;
		result->disp_pic_height = 0;
	}

	result->rd_ptr = wave6_vpu_dec_get_rd_ptr(vpu_inst);
	result->wr_ptr = p_dec_info->stream_wr_ptr;

	result->sequence_no = p_dec_info->initial_info.sequence_no;
	if (decoded_index >= 0 && decoded_index < WAVE6_MAX_FBS)
		p_dec_info->dec_out_info[decoded_index] = *result;

	if (display_index >= 0 && display_index < WAVE6_MAX_FBS) {
		result->num_of_tot_m_bs_in_disp = p_dec_info->dec_out_info[display_index].num_of_tot_m_bs;
		result->num_of_err_m_bs_in_disp = p_dec_info->dec_out_info[display_index].num_of_err_m_bs;
	} else {
		result->num_of_tot_m_bs_in_disp = 0;
		result->num_of_err_m_bs_in_disp = 0;
	}

	if (result->notification_flag & DEC_NOTI_FLAG_SEQ_CHANGE) {
		wave6_get_dec_seq_result(vpu_inst, &p_dec_info->initial_info);
		p_dec_info->initial_info.sequence_no++;
	}

	return 0;
}

int wave6_vpu_dec_fini_seq(struct vpu_instance *vpu_inst, u32 *fail_res)
{
	int ret;

	wave6_send_command(vpu_inst->dev, vpu_inst->id, vpu_inst->std, W6_DESTROY_INSTANCE);
	ret = wave6_wait_vpu_busy(vpu_inst->dev, W6_VPU_BUSY_STATUS);
	if (ret)
		return -ETIMEDOUT;

	if (!vpu_read_reg(vpu_inst->dev, W6_RET_SUCCESS)) {
		*fail_res = vpu_read_reg(vpu_inst->dev, W6_RET_FAIL_REASON);
		wave6_print_reg_err(vpu_inst->dev, *fail_res);
		return -EIO;
	}

	return 0;
}

int wave6_vpu_dec_set_bitstream_flag(struct vpu_instance *vpu_inst, bool eos)
{
	struct dec_info *p_dec_info = &vpu_inst->codec_info->dec_info;

	p_dec_info->stream_end_flag = eos ? true : false;

	return 0;
}

dma_addr_t wave6_vpu_dec_get_rd_ptr(struct vpu_instance *vpu_inst)
{
	return vpu_read_reg(vpu_inst->dev, W6_RET_DEC_BS_RD_PTR);
}

int wave6_vpu_dec_flush(struct vpu_instance *inst)
{
	int ret, index;
	u32 unused_idc;
	u32 used_idc;
	u32 using_idc;

	wave6_send_command(inst->dev, inst->id, inst->std, W6_FLUSH_INSTANCE);
	ret = wave6_wait_vpu_busy(inst->dev, W6_VPU_BUSY_STATUS);
	if (ret)
		return -ETIMEDOUT;

	if (!vpu_read_reg(inst->dev, W6_RET_SUCCESS)) {
		u32 reg_val;

		reg_val = vpu_read_reg(inst->dev, W6_RET_FAIL_REASON);
		wave6_print_reg_err(inst->dev, reg_val);
		return -EIO;
	}

	ret = wave6_send_query(inst->dev, inst->id, inst->std, W6_QUERY_GET_FLUSH_CMD_INFO);
	if (ret)
		return ret;

	unused_idc = vpu_read_reg(inst->dev, W6_RET_DEC_FLUSH_CMD_BUF_STATE_UNUSED_IDC);
	if (unused_idc)
		dev_dbg(inst->dev->dev, "%s: unused_idc %d\n", __func__, unused_idc);

	used_idc = vpu_read_reg(inst->dev, W6_RET_DEC_FLUSH_CMD_BUF_STATE_USED_IDC);
	if (used_idc)
		dev_dbg(inst->dev->dev, "%s: used_idc %d\n", __func__, used_idc);

	using_idc = vpu_read_reg(inst->dev, W6_RET_DEC_FLUSH_CMD_BUF_STATE_USING_IDC);
	if (using_idc)
		dev_err(inst->dev->dev, "%s: using_idc %d\n", __func__, using_idc);

	for (index = 0; index < WAVE6_MAX_FBS; index++) {
		struct frame_buffer fb;
		bool remove = false;
		dma_addr_t addr = vpu_read_reg(inst->dev,
					       W6_RET_DEC_FLUSH_CMD_DISP_ADDR_0 + index * 4);

		if ((unused_idc >> index) & 0x1)
			remove = true;
		if ((used_idc >> index) & 0x1)
			remove = true;

		fb = wave6_dec_get_display_buffer(inst, addr, remove);
	}

	return 0;
}

/************************************************************************/
/* ENCODER functions */
/************************************************************************/
struct enc_cmd_set_param_reg {
	u32 enable;
	u32 src_size;
	u32 custom_map_endian;
	u32 sps_param;
	u32 pps_param;
	u32 gop_param;
	u32 intra_param;
	u32 conf_win_top_bot;
	u32 conf_win_left_right;
	u32 rdo_param;
	u32 slice_param;
	u32 intra_refresh;
	u32 intra_min_max_qp;
	u32 rc_frame_rate;
	u32 rc_target_rate;
	u32 rc_param;
	u32 hvs_param;
	u32 rc_max_bit_rate;
	u32 rc_vbv_buffer_size;
	u32 inter_min_max_qp;
	u32 rot_param;
	u32 num_units_in_tick;
	u32 time_scale;
	u32 num_ticks_poc_diff_one;
	u32 max_intra_pic_bit;
	u32 max_inter_pic_bit;
	u32 bg_param;
	u32 non_vcl_param;
	u32 vui_rbsp_addr;
	u32 hrd_rbsp_addr;
	u32 qround_offset;
	u32 quant_param_1;
	u32 quant_param_2;
	u32 custom_gop_param;
	u32 custom_gop_pic_param[MAX_GOP_NUM];
	u32 tile_param;
	u32 custom_lambda[MAX_CUSTOM_LAMBDA_NUM];
	u32 temporal_layer_qp[MAX_NUM_CHANGEABLE_TEMPORAL_LAYER];
	u32 scl_src_size;
	u32 scl_param;
	u32 color_param;
};

struct enc_cmd_change_param_reg {
	u32 enable;
	u32 rc_target_rate;
	/*add more parameters*/
};

int wave6_vpu_build_up_enc_param(struct device *dev, struct vpu_instance *vpu_inst,
				 struct enc_open_param *param)
{
	struct enc_info *p_enc_info = &vpu_inst->codec_info->enc_info;
	struct vpu_attr *attr = &vpu_inst->dev->attr;
	u32 reg_val;
	int ret;

	p_enc_info->cycle_per_tick = 256;
	p_enc_info->line_buf_int_en = param->line_buf_int_en;
	p_enc_info->stride = 0;
	p_enc_info->initial_info_obtained = false;
	if (param->inst_buffer.sec_size_core0) {
		p_enc_info->sec_axi_info.use_enc_rdo_enable = 1;
		p_enc_info->sec_axi_info.use_enc_lf_enable = 1;
	}

	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_CREATE_INST_TEMP_BASE, param->inst_buffer.temp_base);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_CREATE_INST_TEMP_SIZE, param->inst_buffer.temp_size);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_CREATE_INST_AR_TABLE_BASE, param->inst_buffer.ar_base);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_CREATE_INST_WORK_BASE, param->inst_buffer.work_base);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_CREATE_INST_WORK_SIZE, param->inst_buffer.work_size);

	reg_val = wave6_convert_endian(param->stream_endian);
	reg_val = (~reg_val & VDI_128BIT_ENDIAN_MASK);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_CREATE_INST_BS_PARAM, reg_val);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_CREATE_INST_SRC_OPT, 0);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_CREATE_INST_ADDR_EXT, param->ext_addr_vcpu);

	reg_val = (attr->support_command_queue) ? (COMMAND_QUEUE_DEPTH << 8) : 0;
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_CREATE_INST_CORE_INFO, reg_val | (1 << 4) | (1 << 0));
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_CREATE_INST_SEC_AXI_BASE_CORE0, param->inst_buffer.sec_base_core0);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_CREATE_INST_SEC_AXI_SIZE_CORE0, param->inst_buffer.sec_size_core0);

	reg_val = (param->is_secure_inst << 8) | (param->inst_priority);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_CREATE_INST_PRIORITY, reg_val);

	wave6_send_command(vpu_inst->dev, 0, vpu_inst->std, W6_CREATE_INSTANCE);
	ret = wave6_wait_vpu_busy(vpu_inst->dev, W6_VPU_BUSY_STATUS);
	if (ret) {
		dev_err(vpu_inst->dev->dev, "%s: timeout\n", __func__);
		return ret;
	}

	if (!vpu_read_reg(vpu_inst->dev, W6_RET_SUCCESS)) {
		u32 reason_code = vpu_read_reg(vpu_inst->dev, W6_RET_FAIL_REASON);

		wave6_print_reg_err(vpu_inst->dev, reason_code);
		return -EIO;
	}

	vpu_inst->id = vpu_read_reg(vpu_inst->dev, W6_RET_INSTANCE_ID);

	return 0;
}

static int wave6_set_enc_crop_info(u32 codec, struct enc_wave_param *param, int rot_mode,
				   int width, int height)
{
	int aligned_width = (codec == W_HEVC_ENC) ? ALIGN(width, 32) : ALIGN(width, 16);
	int aligned_height = (codec == W_HEVC_ENC) ? ALIGN(height, 32) : ALIGN(height, 16);
	int pad_right, pad_bot;
	int crop_right, crop_left, crop_top, crop_bot;
	int prp_mode = rot_mode >> 1; // remove prp_enable bit

	if (codec == W_HEVC_ENC &&
	    (!rot_mode || prp_mode == 14)) // prp_mode 14 : hor_mir && ver_mir && rot_180
		return 0;

	pad_right = aligned_width - width;
	pad_bot = aligned_height - height;

	if (param->conf_win.right > 0)
		crop_right = param->conf_win.right + pad_right;
	else
		crop_right = pad_right;

	if (param->conf_win.bottom > 0)
		crop_bot = param->conf_win.bottom + pad_bot;
	else
		crop_bot = pad_bot;

	crop_top = param->conf_win.top;
	crop_left = param->conf_win.left;

	param->conf_win.top = crop_top;
	param->conf_win.left = crop_left;
	param->conf_win.bottom = crop_bot;
	param->conf_win.right = crop_right;

	if (prp_mode == 1 || prp_mode == 15) {
		param->conf_win.top = crop_right;
		param->conf_win.left = crop_top;
		param->conf_win.bottom = crop_left;
		param->conf_win.right = crop_bot;
	} else if (prp_mode == 2 || prp_mode == 12) {
		param->conf_win.top = crop_bot;
		param->conf_win.left = crop_right;
		param->conf_win.bottom = crop_top;
		param->conf_win.right = crop_left;
	} else if (prp_mode == 3 || prp_mode == 13) {
		param->conf_win.top = crop_left;
		param->conf_win.left = crop_bot;
		param->conf_win.bottom = crop_right;
		param->conf_win.right = crop_top;
	} else if (prp_mode == 4 || prp_mode == 10) {
		param->conf_win.top = crop_bot;
		param->conf_win.bottom = crop_top;
	} else if (prp_mode == 8 || prp_mode == 6) {
		param->conf_win.left = crop_right;
		param->conf_win.right = crop_left;
	} else if (prp_mode == 5 || prp_mode == 11) {
		param->conf_win.top = crop_left;
		param->conf_win.left = crop_top;
		param->conf_win.bottom = crop_right;
		param->conf_win.right = crop_bot;
	} else if (prp_mode == 7 || prp_mode == 9) {
		param->conf_win.top = crop_right;
		param->conf_win.left = crop_bot;
		param->conf_win.bottom = crop_left;
		param->conf_win.right = crop_top;
	}

	return 0;
}

static bool wave6_update_enc_info(struct enc_info *p_enc_info)
{
	struct enc_open_param open_param = p_enc_info->open_param;

	p_enc_info->width = open_param.pic_width;
	p_enc_info->height = open_param.pic_height;

	switch (open_param.output_format) {
	case FORMAT_420:
	case FORMAT_420_P10_16BIT_MSB:
	case FORMAT_420_P10_16BIT_LSB:
	case FORMAT_420_P10_32BIT_MSB:
	case FORMAT_420_P10_32BIT_LSB:
		p_enc_info->color_format = 1;
		break;
	case FORMAT_422:
	case FORMAT_422_P10_16BIT_MSB:
	case FORMAT_422_P10_16BIT_LSB:
	case FORMAT_422_P10_32BIT_MSB:
	case FORMAT_422_P10_32BIT_LSB:
		p_enc_info->color_format = 2;
		break;
	case FORMAT_444:
	case FORMAT_444_P10_16BIT_MSB:
	case FORMAT_444_P10_16BIT_LSB:
	case FORMAT_444_P10_32BIT_MSB:
	case FORMAT_444_P10_32BIT_LSB:
		p_enc_info->color_format = 3;
		break;
	case FORMAT_400:
	case FORMAT_400_P10_16BIT_MSB:
	case FORMAT_400_P10_16BIT_LSB:
	case FORMAT_400_P10_32BIT_MSB:
	case FORMAT_400_P10_32BIT_LSB:
		p_enc_info->color_format = 0;
		break;
	default:
		return false;
	}

	return true;
}

static void wave6_gen_set_param_reg_common(struct enc_info *p_enc_info, enum wave_std std,
					   struct enc_cmd_set_param_reg *reg)
{
	struct enc_open_param *p_open_param = &p_enc_info->open_param;
	struct enc_wave_param *p_param = &p_open_param->wave_param;
	unsigned int i, endian;
	u32 rot_mir_mode = 0;

	endian = wave6_convert_endian(p_param->custom_map_endian);
	endian = (~endian & VDI_128BIT_ENDIAN_MASK);

	if (p_enc_info->rotation_enable) {
		switch (p_enc_info->rotation_angle) {
		case 0:
			rot_mir_mode |= 0x0; break;
		case 90:
			rot_mir_mode |= 0x3; break;
		case 180:
			rot_mir_mode |= 0x5; break;
		case 270:
			rot_mir_mode |= 0x7; break;
		}
	}

	if (p_enc_info->mirror_enable) {
		switch (p_enc_info->mirror_direction) {
		case MIRDIR_NONE:
			rot_mir_mode |= 0x0; break;
		case MIRDIR_VER:
			rot_mir_mode |= 0x9; break;
		case MIRDIR_HOR:
			rot_mir_mode |= 0x11; break;
		case MIRDIR_HOR_VER:
			rot_mir_mode |= 0x19; break;
		}
	}

	wave6_set_enc_crop_info(std, p_param, rot_mir_mode, p_enc_info->width, p_enc_info->height);

	reg->src_size = (p_enc_info->height << 16) |
			(p_enc_info->width << 0);
	reg->custom_map_endian = endian;
	reg->gop_param = (p_param->temp_layer_cnt << 16) |
			 (p_param->temp_layer[3].en_change_qp << 11) |
			 (p_param->temp_layer[2].en_change_qp << 10) |
			 (p_param->temp_layer[1].en_change_qp << 9) |
			 (p_param->temp_layer[0].en_change_qp << 8) |
			 (p_param->gop_preset_idx << 0);
	reg->intra_refresh = (p_param->intra_refresh_arg << 16) |
			     (p_param->intra_refresh_mode << 0);
	reg->intra_min_max_qp = (p_param->max_qp_i << 6) |
				(p_param->min_qp_i << 0);
	reg->rc_frame_rate = p_param->frame_rate;
	reg->rc_target_rate = p_param->enc_bit_rate;
	reg->rc_param = (p_param->rc_update_speed << 24) |
			(p_param->rc_initial_level << 20) |
			((p_param->rc_initial_qp & 0x3f) << 14) |
			(p_param->rc_mode << 13) |
			(p_param->pic_rc_max_dqp << 7) |
			(p_param->en_cu_level_rate_control << 1) |
			(p_param->en_rate_control << 0);
	reg->hvs_param = (p_param->max_delta_qp << 12) |
			 (p_param->hvs_qp_scale_div2 << 0);
	reg->rc_max_bit_rate = p_param->max_bit_rate;
	reg->rc_vbv_buffer_size = p_param->vbv_buffer_size;
	reg->inter_min_max_qp = (p_param->max_qp_b << 18) |
				(p_param->min_qp_b << 12) |
				(p_param->max_qp_p << 6) |
				(p_param->min_qp_p << 0);
	reg->rot_param = rot_mir_mode;
	reg->conf_win_top_bot = (p_param->conf_win.bottom << 16) |
				(p_param->conf_win.top << 0);
	reg->conf_win_left_right = (p_param->conf_win.right << 16) |
				   (p_param->conf_win.left << 0);
	reg->num_units_in_tick = p_param->num_units_in_tick;
	reg->time_scale = p_param->time_scale;
	reg->num_ticks_poc_diff_one = p_param->num_ticks_poc_diff_one;
	reg->max_intra_pic_bit = p_param->max_intra_pic_bit;
	reg->max_inter_pic_bit = p_param->max_inter_pic_bit;
	reg->bg_param = ((p_param->bg_delta_qp & 0x3F) << 24) |
			(p_param->bg_th_mean_diff << 10) |
			(p_param->bg_th_diff << 1) |
			(p_param->en_bg_detect << 0);
	reg->qround_offset = (p_param->q_round_inter << 13) |
			     (p_param->q_round_intra << 2);
	reg->custom_gop_param = p_param->gop_param.custom_gop_size;
	for (i = 0; i < p_param->gop_param.custom_gop_size; i++) {
		struct custom_gop_pic_param pic_param = p_param->gop_param.pic_param[i];

		reg->custom_gop_pic_param[i] = (pic_param.temporal_id << 26) |
					       ((pic_param.ref_poc_l1 & 0x3F) << 20) |
					       ((pic_param.ref_poc_l0 & 0x3F) << 14) |
					       (pic_param.use_multi_ref_p << 13) |
					       (pic_param.pic_qp << 7) |
					       (pic_param.poc_offset << 2) |
					       (pic_param.pic_type << 0);
	}
	for (i = 0; i < MAX_CUSTOM_LAMBDA_NUM; i++) {
		reg->custom_lambda[i] = (p_param->custom_lambda_ssd[i] << 7) |
					(p_param->custom_lambda_sad[i] << 0);
	}
	for (i = 0; i < MAX_NUM_CHANGEABLE_TEMPORAL_LAYER; i++) {
		reg->temporal_layer_qp[i] = (p_param->temp_layer[i].qp_b << 12) |
					    (p_param->temp_layer[i].qp_p << 6) |
					    (p_param->temp_layer[i].qp_i << 0);
	}
	reg->scl_src_size = (p_open_param->pic_height << 16) |
			    (p_open_param->pic_width << 0);
	reg->scl_param = (p_enc_info->scaler_info.coef_mode << 1) |
			 (p_enc_info->scaler_info.enable << 0);
}

static void wave6_gen_set_param_reg_hevc(struct enc_info *p_enc_info, struct enc_cmd_set_param_reg *reg)
{
	struct enc_open_param *p_open_param = &p_enc_info->open_param;
	struct enc_wave_param *p_param = &p_open_param->wave_param;

	reg->sps_param = (p_param->en_scaling_list << 31) |
			 (p_param->en_still_picture << 30) |
			 (p_param->strong_intra_smoothing << 27) |
			 (p_param->intra_trans_skip << 25) |
			 (p_param->en_sao << 24) |
			 (p_param->en_temporal_mvp << 23) |
			 (p_param->en_long_term << 21) |
			 (p_enc_info->color_format << 19) |
			 (p_param->internal_bit_depth << 14) |
			 (p_param->tier << 12) |
			 (p_param->level << 3) |
			 (p_param->profile << 0);
	reg->pps_param = ((p_param->cr_qp_offset & 0x1F) << 19) |
			 ((p_param->cb_qp_offset & 0x1F) << 14) |
			 ((p_param->tc_offset_div2 & 0xF) << 10) |
			 ((p_param->beta_offset_div2 & 0xF) << 6)  |
			 ((!p_param->en_dbk) << 5)  |
			 (p_param->lf_cross_slice_boundary_flag << 2) |
			 (p_param->constrained_intra_pred << 1);
	reg->intra_param = (p_param->intra_period << 16) |
			   (p_param->forced_idr_header << 9) |
			   (p_param->qp << 3) |
			   (p_param->decoding_refresh_type << 0);
	reg->rdo_param = (p_param->en_custom_lambda << 22) |
			 (p_param->me_center << 21) |
			 (p_param->en_qp_map << 20) |
			 (p_param->en_mode_map << 19) |
			 (p_param->en_q_round_offset << 17) |
			 (p_param->dis_coef_clear << 4) |
			 (p_param->en_adaptive_round << 3) |
			 (p_param->en_hvs_qp << 2);
	reg->slice_param = (p_param->slice_arg << 3) |
			   (p_param->slice_mode << 0);
	reg->quant_param_2 = ((p_param->lambda_dqp_inter & 0x3F) << 14) |
			     ((p_param->lambda_dqp_intra & 0x3F) << 8);
	reg->non_vcl_param = (p_open_param->hrd_rbsp_data_size << 18) |
			     (p_open_param->vui_rbsp_data_size << 4) |
			     (p_open_param->enc_hrd_rbsp_in_vps << 2) |
			     (p_open_param->enc_vui_rbsp << 1) |
			     (p_open_param->enc_aud << 0);
	reg->vui_rbsp_addr = p_open_param->vui_rbsp_data_addr;
	reg->hrd_rbsp_addr = p_open_param->hrd_rbsp_data_addr;
}

static void wave6_gen_set_param_reg_avc(struct enc_info *p_enc_info, struct enc_cmd_set_param_reg *reg)
{
	struct enc_open_param *p_open_param = &p_enc_info->open_param;
	struct enc_wave_param *p_param = &p_open_param->wave_param;

	reg->sps_param = (p_param->en_scaling_list << 31) |
			 (p_param->en_long_term << 21) |
			 (p_enc_info->color_format << 19) |
			 (p_param->internal_bit_depth << 14) |
			 (p_param->level << 3) |
			 (p_param->profile << 0);
	reg->pps_param = (p_param->en_cabac << 30) |
			 (p_param->en_transform8x8 << 29) |
			 ((p_param->cr_qp_offset & 0x1F) << 19) |
			 ((p_param->cb_qp_offset & 0x1F) << 14) |
			 ((p_param->tc_offset_div2 & 0xF) << 10) |
			 ((p_param->beta_offset_div2 & 0xF) << 6) |
			 ((!p_param->en_dbk) << 5) |
			 (p_param->lf_cross_slice_boundary_flag << 2) |
			 (p_param->constrained_intra_pred << 1);
	reg->intra_param = (p_param->forced_idr_header << 28) |
			   (p_param->idr_period << 17) |
			   (p_param->intra_period << 6) |
			   (p_param->qp << 0);
	reg->rdo_param = (p_param->en_custom_lambda << 22) |
			 (p_param->me_center << 21) |
			 (p_param->en_qp_map << 20) |
			 (p_param->en_mode_map << 19) |
			 (p_param->en_q_round_offset << 17) |
			 (p_param->dis_coef_clear << 4) |
			 (p_param->en_adaptive_round << 3) |
			 (p_param->en_hvs_qp << 2);
	reg->slice_param = (p_param->slice_arg << 3) |
			   (p_param->slice_mode << 0);
	reg->quant_param_2 = ((p_param->lambda_dqp_inter & 0x3F) << 14) |
			     ((p_param->lambda_dqp_intra & 0x3F) << 8);
	reg->non_vcl_param = (p_open_param->hrd_rbsp_data_size << 18) |
			     (p_open_param->vui_rbsp_data_size << 4) |
			     (p_open_param->enc_hrd_rbsp_in_vps << 2) |
			     (p_open_param->enc_vui_rbsp << 1) |
			     (p_open_param->enc_aud << 0);
	reg->vui_rbsp_addr = p_open_param->vui_rbsp_data_addr;
	reg->hrd_rbsp_addr = p_open_param->hrd_rbsp_data_addr;
}

static void wave6_gen_set_param_reg_av1(struct enc_info *p_enc_info, struct enc_cmd_set_param_reg *reg)
{
	struct enc_open_param *p_open_param = &p_enc_info->open_param;
	struct enc_wave_param *p_param = &p_open_param->wave_param;

	reg->sps_param = (p_param->en_scaling_list << 31) |
			 (p_param->intra_trans_skip << 25) |
			 (p_param->en_wiener << 24) |
			 (p_param->en_cdef << 23) |
			 (p_param->en_long_term << 21) |
			 (p_enc_info->color_format << 19) |
			 (p_param->internal_bit_depth << 14) |
			 (p_param->level << 3);
	reg->pps_param = (p_param->lf_sharpness << 6) |
			 ((!p_param->en_dbk) << 5);
	reg->intra_param = (p_param->intra_period << 16) |
			   (p_param->forced_idr_header << 9) |
			   (p_param->qp << 3);
	reg->rdo_param = (p_param->en_custom_lambda << 22) |
			 (p_param->me_center << 21) |
			 (p_param->en_qp_map << 20) |
			 (p_param->en_mode_map << 19) |
			 (p_param->en_q_round_offset << 17) |
			 (p_param->dis_coef_clear << 4) |
			 (p_param->en_adaptive_round << 3) |
			 (p_param->en_hvs_qp << 2);
	reg->quant_param_1 = ((p_param->u_dc_qp_delta & 0xFF) << 24) |
			     ((p_param->v_dc_qp_delta & 0xFF) << 16) |
			     ((p_param->u_ac_qp_delta & 0xFF) << 8) |
			     ((p_param->v_ac_qp_delta & 0xFF) << 0);
	reg->quant_param_2 = ((p_param->lambda_dqp_inter & 0x3F) << 14) |
			     ((p_param->lambda_dqp_intra & 0x3F) << 8) |
			     ((p_param->y_dc_qp_delta & 0xFF) << 0);
	reg->tile_param = ((p_param->row_tile_num - 1) << 4) |
			  ((p_param->col_tile_num - 1) << 0);
	reg->color_param = ((p_param->av1_color.chroma_sample_position & 0x3) << 26) |
			   (p_param->av1_color.color_range << 25) |
			   ((p_param->av1_color.matrix_coefficients & 0xFF) << 17) |
			   ((p_param->av1_color.transfer_characteristics & 0xFF) << 9) |
			   ((p_param->av1_color.color_primaries & 0xFF) << 1) |
			   (p_param->av1_color.color_description_present_flag << 0);
}

static void wave6_gen_change_param_reg_common(struct vpu_instance *vpu_inst,
					      struct enc_info *p_enc_info,
					      struct enc_cmd_change_param_reg *reg)
{
	struct enc_change_param *p_change_param = &p_enc_info->change_param;

	reg->enable = p_change_param->enable;
	reg->rc_target_rate = p_change_param->enc_bit_rate;
}

int wave6_vpu_enc_init_seq(struct vpu_instance *vpu_inst)
{
	struct enc_cmd_set_param_reg reg;
	struct vpu_attr *attr = &vpu_inst->dev->attr;
	struct enc_info *p_enc_info = &vpu_inst->codec_info->enc_info;
	u32 i;
	int ret;

	memset(&reg, 0, sizeof(struct enc_cmd_set_param_reg));

	if (!wave6_update_enc_info(p_enc_info))
		return -EINVAL;

	wave6_gen_set_param_reg_common(p_enc_info, vpu_inst->std, &reg);
	if (vpu_inst->std == W_HEVC_ENC)
		wave6_gen_set_param_reg_hevc(p_enc_info, &reg);
	else if (vpu_inst->std == W_AVC_ENC)
		wave6_gen_set_param_reg_avc(p_enc_info, &reg);
	else
		wave6_gen_set_param_reg_av1(p_enc_info, &reg);

	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_OPTION, W6_SET_PARAM_OPT_COMMON);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_ENABLE, reg.enable);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_SRC_SIZE, reg.src_size);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_CUSTOM_MAP_ENDIAN, reg.custom_map_endian);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_SPS_PARAM, reg.sps_param);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_PPS_PARAM, reg.pps_param);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_GOP_PARAM, reg.gop_param);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_INTRA_PARAM, reg.intra_param);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_CONF_WIN_TOP_BOT, reg.conf_win_top_bot);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_CONF_WIN_LEFT_RIGHT, reg.conf_win_left_right);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_RDO_PARAM, reg.rdo_param);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_SLICE_PARAM, reg.slice_param);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_INTRA_REFRESH, reg.intra_refresh);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_INTRA_MIN_MAX_QP, reg.intra_min_max_qp);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_RC_FRAME_RATE, reg.rc_frame_rate);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_RC_TARGET_RATE, reg.rc_target_rate);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_RC_PARAM, reg.rc_param);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_HVS_PARAM, reg.hvs_param);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_RC_MAX_BITRATE, reg.rc_max_bit_rate);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_RC_VBV_BUFFER_SIZE, reg.rc_vbv_buffer_size);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_INTER_MIN_MAX_QP, reg.inter_min_max_qp);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_ROT_PARAM, reg.rot_param);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_NUM_UNITS_IN_TICK, reg.num_units_in_tick);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_TIME_SCALE, reg.time_scale);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_NUM_TICKS_POC_DIFF_ONE, reg.num_ticks_poc_diff_one);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_MAX_INTRA_PIC_BIT, reg.max_intra_pic_bit);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_MAX_INTER_PIC_BIT, reg.max_inter_pic_bit);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_BG_PARAM, reg.bg_param);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_NON_VCL_PARAM, reg.non_vcl_param);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_VUI_RBSP_ADDR, reg.vui_rbsp_addr);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_HRD_RBSP_ADDR, reg.hrd_rbsp_addr);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_QROUND_OFFSET, reg.qround_offset);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_QUANT_PARAM_1, reg.quant_param_1);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_QUANT_PARAM_2, reg.quant_param_2);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_CUSTOM_GOP_PARAM, reg.custom_gop_param);
	for (i = 0; i < MAX_GOP_NUM; i++)
		vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_CUSTOM_GOP_PIC_PARAM_0 + (i*4), reg.custom_gop_pic_param[i]);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_TILE_PARAM, reg.tile_param);
	for (i = 0; i < MAX_CUSTOM_LAMBDA_NUM; i++)
		vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_CUSTOM_LAMBDA_0 + (i*4), reg.custom_lambda[i]);
	for (i = 0; i < MAX_NUM_CHANGEABLE_TEMPORAL_LAYER; i++)
		vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_TEMPORAL_LAYER_0_QP + (i*4), reg.temporal_layer_qp[i]);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_SCL_SRC_SIZE, reg.scl_src_size);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_SCL_PARAM, reg.scl_param);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_COLOR_PARAM, reg.color_param);

	wave6_send_command(vpu_inst->dev, vpu_inst->id, vpu_inst->std, W6_ENC_SET_PARAM);
	if (attr->support_command_queue) {
		ret = wave6_wait_vpu_busy(vpu_inst->dev, W6_VPU_BUSY_STATUS);
		if (ret) {
			dev_err(vpu_inst->dev->dev, "%s: timeout\n", __func__);
			return ret;
		}

		if (!vpu_read_reg(vpu_inst->dev, W6_RET_SUCCESS)) {
			u32 reason_code = vpu_read_reg(vpu_inst->dev, W6_RET_FAIL_REASON);

			wave6_print_reg_err(vpu_inst->dev, reason_code);
			return -EIO;
		}
	}

	return 0;
}

int wave6_vpu_enc_get_seq_info(struct vpu_instance *vpu_inst, struct enc_initial_info *info)
{
	struct vpu_attr *attr = &vpu_inst->dev->attr;
	int ret = 0;

	if (attr->support_command_queue) {
		ret = wave6_send_query(vpu_inst->dev, vpu_inst->id, vpu_inst->std, W6_QUERY_GET_RESULT);
		if (ret)
			return ret;

		if (vpu_read_reg(vpu_inst->dev, W6_RET_ENC_ENCODING_SUCCESS) != 1) {
			info->err_reason = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_ERR_INFO);
			ret = -EIO;
		} else {
			info->warn_info = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_WARN_INFO);
		}
	} else {
		if (!vpu_read_reg(vpu_inst->dev, W6_RET_SUCCESS)) {
			info->err_reason = vpu_read_reg(vpu_inst->dev, W6_RET_FAIL_REASON);

			wave6_print_reg_err(vpu_inst->dev, info->err_reason);
			return -EIO;
		}
	}

	info->min_frame_buffer_count = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_NUM_REQUIRED_FBC_FB);
	info->min_src_frame_count = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_MIN_SRC_BUF_NUM);
	info->max_latency_pictures = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_PIC_MAX_LATENCY_PICTURES);
	info->req_mv_buffer_count = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_NUM_REQUIRED_COL_BUF);

	return ret;
}

int wave6_vpu_enc_change_seq(struct vpu_instance *vpu_inst)
{
	struct enc_cmd_change_param_reg reg;
	struct enc_info *p_enc_info = &vpu_inst->codec_info->enc_info;
	struct vpu_attr attr = vpu_inst->dev->attr;
	int ret = 0;

	memset(&reg, 0, sizeof(struct enc_cmd_change_param_reg));

	wave6_gen_change_param_reg_common(vpu_inst, p_enc_info, &reg);

	if (!reg.enable)
		return 0;

	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_OPTION, W6_SET_PARAM_OPT_CHANGE_PARAM);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_ENABLE, reg.enable);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_PARAM_RC_TARGET_RATE, reg.rc_target_rate);

	wave6_send_command(vpu_inst->dev, vpu_inst->id, vpu_inst->std, W6_ENC_SET_PARAM);
	if (attr.support_command_queue) {
		ret = wave6_wait_vpu_busy(vpu_inst->dev, W6_VPU_BUSY_STATUS);
		if (ret) {
			dev_warn(vpu_inst->dev->dev, "enc set param timed out\n");
			return ret;
		}

		if (!vpu_read_reg(vpu_inst->dev, W6_RET_SUCCESS)) {
			u32 reason_code = vpu_read_reg(vpu_inst->dev, W6_RET_FAIL_REASON);

			wave6_print_reg_err(vpu_inst->dev, reason_code);
			return -EIO;
		}
	}

	return 0;
}

struct enc_cmd_set_fb_reg {
	u32 option;
	u32 pic_info;
	u32 pic_size;
	u32 num_fb;
	u32 fbc_stride;
	u32 fbc_y[WAVE6_MAX_FBS];
	u32 fbc_c[WAVE6_MAX_FBS];
	u32 fbc_cr[WAVE6_MAX_FBS];
	u32 fbc_y_offset[WAVE6_MAX_FBS];
	u32 fbc_c_offset[WAVE6_MAX_FBS];
	u32 fbc_cr_offset[WAVE6_MAX_FBS];
	u32 mv_col[WAVE6_MAX_FBS];
	u32 sub_sampled[WAVE6_MAX_FBS];
	u32 default_cdf;
};

static void wave6_gen_set_fb_reg(struct enc_info *p_enc_info, enum wave_std std,
				 struct frame_buffer *fb_arr, struct enc_cmd_set_fb_reg *reg)
{
	struct enc_open_param *p_open_param = &p_enc_info->open_param;
	u32 mv_count = p_enc_info->initial_info.req_mv_buffer_count;
	u32 buf_width, buf_height;
	u32 stride_l, stride_c, i;

	if (std == W_AVC_ENC) {
		buf_width  = ALIGN(p_enc_info->width, 16);
		buf_height = ALIGN(p_enc_info->height, 16);
		if (p_enc_info->rotation_angle == 90 || p_enc_info->rotation_angle == 270) {
			buf_width  = ALIGN(p_enc_info->height, 16);
			buf_height = ALIGN(p_enc_info->width, 16);
		}
	} else {
		buf_width  = ALIGN(p_enc_info->width, 8);
		buf_height = ALIGN(p_enc_info->height, 8);
		if ((p_enc_info->rotation_angle != 0 || p_enc_info->mirror_direction != 0) &&
		    !(p_enc_info->rotation_angle == 180 && p_enc_info->mirror_direction == MIRDIR_HOR_VER)) {
			buf_width  = ALIGN(p_enc_info->width, 32);
			buf_height = ALIGN(p_enc_info->height, 32);
		}
		if (p_enc_info->rotation_angle == 90 || p_enc_info->rotation_angle == 270) {
			buf_width  = ALIGN(p_enc_info->height, 32);
			buf_height = ALIGN(p_enc_info->width, 32);
		}
	}

	if ((p_enc_info->rotation_angle != 0 || p_enc_info->mirror_direction != 0) &&
	    !(p_enc_info->rotation_angle == 180 && p_enc_info->mirror_direction == MIRDIR_HOR_VER)) {
		stride_l = ALIGN((buf_width + 63), 64);
		stride_c = ALIGN((buf_width + 31), 32) / 2;
	} else {
		stride_l = ALIGN((p_enc_info->width) + 63, 64);
		stride_c = ALIGN((p_enc_info->width) + 31, 32) / 2;
	}

	reg->option = (p_open_param->enable_non_ref_fbc_write << 26) |
		      (1 << 4) |
		      (1 << 3);
	reg->pic_info = p_enc_info->stride;
	reg->pic_size = (buf_width << 16) |
			(buf_height << 0);
	reg->num_fb = ((p_enc_info->num_frame_buffers - 1) << 16) |
		      ((mv_count - 1) << 0);
	reg->fbc_stride = (stride_l << 16) |
			  (stride_c << 0);
	reg->default_cdf = p_enc_info->vb_def_cdf.daddr;

	for (i = 0; i < p_enc_info->num_frame_buffers; i++) {
		reg->fbc_y[i] = fb_arr[i].buf_y;
		reg->fbc_c[i] = fb_arr[i].buf_cb;
		reg->fbc_cr[i] = fb_arr[i].buf_cr;
		reg->fbc_y_offset[i] = p_enc_info->vb_fbc_y_tbl[i].daddr;
		reg->fbc_c_offset[i] = p_enc_info->vb_fbc_c_tbl[i].daddr;
		reg->fbc_cr_offset[i] = p_enc_info->vb_fbc_c_tbl[i].daddr +
						(p_enc_info->vb_fbc_c_tbl[i].size >> 1);
		reg->sub_sampled[i] = p_enc_info->vb_sub_sam_buf[i].daddr;
	}
	for (i = 0; i < mv_count; i++)
		reg->mv_col[i] = p_enc_info->vb_mv[i].daddr;
}

int wave6_vpu_enc_register_frame_buffer(struct vpu_instance *vpu_inst,
				       struct frame_buffer *fb_arr)
{
	struct enc_cmd_set_fb_reg *reg;
	struct enc_info *p_enc_info = &vpu_inst->codec_info->enc_info;
	u32 mv_count = p_enc_info->initial_info.req_mv_buffer_count;
	int ret;
	u32 idx;

	if (vpu_inst->std == W_AV1_ENC) {
		if (!p_enc_info->vb_def_cdf.daddr)
			return -EINVAL;

		wave6_load_av1_cdf_table(vpu_inst->dev, &p_enc_info->vb_def_cdf);
	}

	for (idx = 0; idx < p_enc_info->num_frame_buffers; idx++) {
		if (!p_enc_info->vb_fbc_y_tbl[idx].daddr)
			return -EINVAL;
		if (!p_enc_info->vb_fbc_c_tbl[idx].daddr)
			return -EINVAL;
		if (!p_enc_info->vb_sub_sam_buf[idx].daddr)
			return -EINVAL;
	}
	for (idx = 0; idx < mv_count; idx++) {
		if (!p_enc_info->vb_mv[idx].daddr)
			return -EINVAL;
	}

	reg = kzalloc(sizeof(struct enc_cmd_set_fb_reg), GFP_KERNEL);
	if (!reg)
		return -ENOMEM;

	wave6_gen_set_fb_reg(p_enc_info, vpu_inst->std, fb_arr, reg);

	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_FB_OPTION, reg->option);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_FB_PIC_INFO, reg->pic_info);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_FB_PIC_SIZE, reg->pic_size);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_FB_NUM, reg->num_fb);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_FB_FBC_STRIDE, reg->fbc_stride);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_FB_DEFAULT_CDF, reg->default_cdf);
	for (idx = 0; idx < p_enc_info->num_frame_buffers; idx++) {
		vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_FB_FBC_Y0 + (idx * 24), reg->fbc_y[idx]);
		vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_FB_FBC_C0 + (idx * 24), reg->fbc_c[idx]);
		vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_FB_FBC_CR0 + (idx * 8), reg->fbc_cr[idx]);
		vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_FB_FBC_Y_OFFSET0 + (idx * 24), reg->fbc_y_offset[idx]);
		vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_FB_FBC_C_OFFSET0 + (idx * 24), reg->fbc_c_offset[idx]);
		vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_FB_FBC_CR_OFFSET0 + (idx * 8), reg->fbc_cr_offset[idx]);
		vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_FB_SUB_SAMPLED0 + (idx * 24), reg->sub_sampled[idx]);
	}
	for (idx = 0; idx < mv_count; idx++)
		vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_SET_FB_MV_COL0 + (idx*24), reg->mv_col[idx]);

	wave6_send_command(vpu_inst->dev, vpu_inst->id, vpu_inst->std, W6_SET_FB);
	ret = wave6_wait_vpu_busy(vpu_inst->dev, W6_VPU_BUSY_STATUS);
	if (ret) {
		dev_err(vpu_inst->dev->dev, "%s: timeout\n", __func__);
		kfree(reg);
		return ret;
	}

	if (!vpu_read_reg(vpu_inst->dev, W6_RET_SUCCESS)) {
		kfree(reg);
		return -EIO;
	}

	kfree(reg);
	return 0;
}

struct enc_cmd_enc_pic_reg {
	u32 bs_start;
	u32 bs_size;
	u32 bs_option;
	u32 use_sec_axi;
	u32 report_param;
	u32 mv_histo_class0;
	u32 mv_histo_class1;
	u32 custom_map_param;
	u32 custom_map_addr;
	u32 src_pic_idx;
	u32 src_addr_y;
	u32 src_addr_u;
	u32 src_addr_v;
	u32 src_stride;
	u32 src_format;
	u32 src_axi_sel;
	u32 code_option;
	u32 pic_param;
	u32 longterm_pic;
	u32 prefix_sei_nal_addr;
	u32 prefix_sei_info;
	u32 suffix_sei_nal_addr;
	u32 suffix_sei_info;
	u32 timestamp;
	u32 csc_coeff[MAX_CSC_COEFF_NUM];
};

static bool is_format_conv(enum frame_buffer_format in_fmt,
			   enum frame_buffer_format out_fmt)
{
	if (in_fmt == FORMAT_420 ||
	    in_fmt == FORMAT_420_P10_16BIT_MSB ||
	    in_fmt == FORMAT_420_P10_16BIT_LSB ||
	    in_fmt == FORMAT_420_P10_32BIT_MSB ||
	    in_fmt == FORMAT_420_P10_32BIT_LSB) {
		if (out_fmt != FORMAT_420 &&
		    out_fmt != FORMAT_420_P10_16BIT_MSB &&
		    out_fmt != FORMAT_420_P10_16BIT_LSB &&
		    out_fmt != FORMAT_420_P10_32BIT_MSB &&
		    out_fmt != FORMAT_420_P10_32BIT_LSB)
			return TRUE;
	} else if (in_fmt == FORMAT_422 ||
		   in_fmt == FORMAT_422_P10_16BIT_MSB ||
		   in_fmt == FORMAT_422_P10_16BIT_LSB ||
		   in_fmt == FORMAT_422_P10_32BIT_MSB ||
		   in_fmt == FORMAT_422_P10_32BIT_LSB) {
		if (out_fmt != FORMAT_422 &&
		    out_fmt != FORMAT_422_P10_16BIT_MSB &&
		    out_fmt != FORMAT_422_P10_16BIT_LSB &&
		    out_fmt != FORMAT_422_P10_32BIT_MSB &&
		    out_fmt != FORMAT_422_P10_32BIT_LSB)
			return TRUE;
	} else if (in_fmt == FORMAT_444 ||
		   in_fmt == FORMAT_444_P10_16BIT_MSB ||
		   in_fmt == FORMAT_444_P10_16BIT_LSB ||
		   in_fmt == FORMAT_444_P10_32BIT_MSB ||
		   in_fmt == FORMAT_444_P10_32BIT_LSB) {
		if (out_fmt != FORMAT_444 &&
		    out_fmt != FORMAT_444_P10_16BIT_MSB &&
		    out_fmt != FORMAT_444_P10_16BIT_LSB &&
		    out_fmt != FORMAT_444_P10_32BIT_MSB &&
		    out_fmt != FORMAT_444_P10_32BIT_LSB)
			return TRUE;
	}

	return FALSE;
}

static void wave6_gen_enc_pic_reg(struct enc_info *p_enc_info, bool cbcr_interleave, bool nv21,
				  struct enc_param *opt, struct enc_cmd_enc_pic_reg *reg)
{
	struct enc_open_param open = p_enc_info->open_param;
	struct enc_wave_param param = open.wave_param;
	bool is_lsb = false;
	bool is_10bit = false;
	bool is_3p4b = false;
	u32 stride_c = 0;
	u32 src_frame_format = 0;
	u32 endian;
	u32 color_format = 0;
	bool is_ayuv = false;
	bool is_csc_format = false;
	bool is_24bit = false;
	bool format_conv;

	endian = wave6_convert_endian(open.source_endian);
	endian = (~endian & VDI_128BIT_ENDIAN_MASK);
	format_conv = is_format_conv(open.src_format, open.output_format);

	switch (open.src_format) {
	case FORMAT_420:
	case FORMAT_420_P10_16BIT_MSB:
	case FORMAT_420_P10_16BIT_LSB:
		color_format = 1;
		stride_c = (cbcr_interleave) ? opt->source_frame->stride :
					       (opt->source_frame->stride / 2);
		break;
	case FORMAT_420_P10_32BIT_MSB:
	case FORMAT_420_P10_32BIT_LSB:
		color_format = 1;
		stride_c = (cbcr_interleave) ? opt->source_frame->stride :
					       ALIGN((opt->source_frame->stride / 2), 16);
		break;
	case FORMAT_422:
	case FORMAT_422_P10_16BIT_MSB:
	case FORMAT_422_P10_16BIT_LSB:
		color_format = 2;
		stride_c = (cbcr_interleave) ? opt->source_frame->stride :
					       (opt->source_frame->stride / 2);
		stride_c = (format_conv) ? (stride_c * 2) : stride_c;
		break;
	case FORMAT_422_P10_32BIT_MSB:
	case FORMAT_422_P10_32BIT_LSB:
		color_format = 2;
		stride_c = (cbcr_interleave) ? opt->source_frame->stride :
					       ALIGN((opt->source_frame->stride / 2), 16);
		stride_c = (format_conv) ? (stride_c * 2) : stride_c;
		break;
	case FORMAT_444:
	case FORMAT_444_P10_16BIT_MSB:
	case FORMAT_444_P10_16BIT_LSB:
		color_format = 3;
		stride_c = (cbcr_interleave) ? (opt->source_frame->stride * 2) :
					       opt->source_frame->stride;
		stride_c = (format_conv) ? (stride_c * 2) : stride_c;
		break;
	case FORMAT_444_P10_32BIT_MSB:
	case FORMAT_444_P10_32BIT_LSB:
		color_format = 3;
		stride_c = (cbcr_interleave) ? ALIGN((opt->source_frame->stride * 2), 16) :
					       opt->source_frame->stride;
		stride_c = (format_conv) ? (stride_c * 2) : stride_c;
		break;
	case FORMAT_YUV444_24BIT:
		color_format = 0;
		stride_c = ALIGN((opt->source_frame->stride * 2), 16);
		break;
	case FORMAT_RGB_24BIT_PACKED:
	case FORMAT_YUV444_24BIT_PACKED:
	case FORMAT_RGB_32BIT_PACKED:
	case FORMAT_RGB_P10_32BIT_PACKED:
	case FORMAT_YUV444_32BIT_PACKED:
	case FORMAT_YUV444_P10_32BIT_PACKED:
		color_format = 4;
		stride_c = 0;
		break;
	case FORMAT_YUYV:
	case FORMAT_YVYU:
	case FORMAT_UYVY:
	case FORMAT_VYUY:
	case FORMAT_YUYV_P10_16BIT_MSB:
	case FORMAT_YVYU_P10_16BIT_MSB:
	case FORMAT_UYVY_P10_16BIT_MSB:
	case FORMAT_VYUY_P10_16BIT_MSB:
	case FORMAT_YUYV_P10_16BIT_LSB:
	case FORMAT_YVYU_P10_16BIT_LSB:
	case FORMAT_UYVY_P10_16BIT_LSB:
	case FORMAT_VYUY_P10_16BIT_LSB:
	case FORMAT_YUYV_P10_32BIT_MSB:
	case FORMAT_YVYU_P10_32BIT_MSB:
	case FORMAT_UYVY_P10_32BIT_MSB:
	case FORMAT_VYUY_P10_32BIT_MSB:
	case FORMAT_YUYV_P10_32BIT_LSB:
	case FORMAT_YVYU_P10_32BIT_LSB:
	case FORMAT_UYVY_P10_32BIT_LSB:
	case FORMAT_VYUY_P10_32BIT_LSB:
		color_format = 2;
		stride_c = 0;
		break;
	default:
		color_format = 0;
		stride_c = 0;
		break;
	}

	switch (open.src_format) {
	case FORMAT_420:
	case FORMAT_422:
	case FORMAT_444:
	case FORMAT_400:
	case FORMAT_YUYV:
	case FORMAT_YVYU:
	case FORMAT_UYVY:
	case FORMAT_VYUY:
		is_lsb = false;
		is_3p4b = false;
		break;
	case FORMAT_420_P10_16BIT_MSB:
	case FORMAT_422_P10_16BIT_MSB:
	case FORMAT_444_P10_16BIT_MSB:
	case FORMAT_400_P10_16BIT_MSB:
	case FORMAT_YUYV_P10_16BIT_MSB:
	case FORMAT_YVYU_P10_16BIT_MSB:
	case FORMAT_UYVY_P10_16BIT_MSB:
	case FORMAT_VYUY_P10_16BIT_MSB:
		is_lsb = false;
		is_10bit = true;
		is_3p4b = false;
		break;
	case FORMAT_420_P10_16BIT_LSB:
	case FORMAT_422_P10_16BIT_LSB:
	case FORMAT_444_P10_16BIT_LSB:
	case FORMAT_400_P10_16BIT_LSB:
	case FORMAT_YUYV_P10_16BIT_LSB:
	case FORMAT_YVYU_P10_16BIT_LSB:
	case FORMAT_UYVY_P10_16BIT_LSB:
	case FORMAT_VYUY_P10_16BIT_LSB:
		is_lsb = true;
		is_10bit = true;
		is_3p4b = false;
		break;
	case FORMAT_420_P10_32BIT_MSB:
	case FORMAT_422_P10_32BIT_MSB:
	case FORMAT_444_P10_32BIT_MSB:
	case FORMAT_400_P10_32BIT_MSB:
	case FORMAT_YUYV_P10_32BIT_MSB:
	case FORMAT_YVYU_P10_32BIT_MSB:
	case FORMAT_UYVY_P10_32BIT_MSB:
	case FORMAT_VYUY_P10_32BIT_MSB:
		is_lsb = false;
		is_10bit = true;
		is_3p4b = true;
		break;
	case FORMAT_420_P10_32BIT_LSB:
	case FORMAT_422_P10_32BIT_LSB:
	case FORMAT_444_P10_32BIT_LSB:
	case FORMAT_400_P10_32BIT_LSB:
	case FORMAT_YUYV_P10_32BIT_LSB:
	case FORMAT_YVYU_P10_32BIT_LSB:
	case FORMAT_UYVY_P10_32BIT_LSB:
	case FORMAT_VYUY_P10_32BIT_LSB:
		is_lsb = true;
		is_10bit = true;
		is_3p4b = true;
		break;
	case FORMAT_RGB_32BIT_PACKED:
		is_ayuv = false;
		is_csc_format = true;
		break;
	case FORMAT_RGB_P10_32BIT_PACKED:
		is_ayuv = false;
		is_csc_format = true;
		is_10bit = true;
		break;
	case FORMAT_YUV444_32BIT_PACKED:
		is_ayuv = true;
		is_csc_format = true;
		break;
	case FORMAT_YUV444_P10_32BIT_PACKED:
		is_ayuv = true;
		is_csc_format = true;
		is_10bit = true;
		break;
	case FORMAT_RGB_24BIT_PACKED:
		is_ayuv = false;
		is_csc_format = true;
		is_24bit = true;
		break;
	case FORMAT_YUV444_24BIT_PACKED:
		is_ayuv = true;
		is_csc_format = true;
		is_24bit = true;
		break;
	case FORMAT_YUV444_24BIT:
		is_ayuv = true;
		break;
	default:
		break;
	}

	src_frame_format = (nv21 << 2) | (cbcr_interleave << 1);
	switch (open.packed_format) {
	case PACKED_YUYV:
		src_frame_format = 1; break;
	case PACKED_YVYU:
		src_frame_format = 5; break;
	case PACKED_UYVY:
		src_frame_format = 9; break;
	case PACKED_VYUY:
		src_frame_format = 13; break;
	default:
		break;
	}

	reg->bs_start = opt->pic_stream_buffer_addr;
	reg->bs_size = opt->pic_stream_buffer_size;
	reg->bs_option = (p_enc_info->line_buf_int_en << 6);
	reg->use_sec_axi = (p_enc_info->sec_axi_info.use_enc_rdo_enable << 1) |
			   (p_enc_info->sec_axi_info.use_enc_lf_enable << 0);
	reg->report_param = (param.en_report_mv_histo << 1);
	reg->mv_histo_class0 = (param.report_mv_histo_threshold0 << 16) |
			       (param.report_mv_histo_threshold1 << 0);
	reg->mv_histo_class1 = (param.report_mv_histo_threshold2 << 16) |
			       (param.report_mv_histo_threshold3 << 0);
	reg->custom_map_param = (opt->custom_map_opt.custom_mode_map_enable << 1) |
				(opt->custom_map_opt.custom_roi_map_enable << 0);
	reg->custom_map_addr = opt->custom_map_opt.custom_map_addr;

	if (opt->src_end_flag)
		reg->src_pic_idx = 0xFFFFFFFF;
	else
		reg->src_pic_idx = opt->src_idx;

	reg->src_addr_y = opt->source_frame->buf_y;
	if (open.cbcr_order == CBCR_ORDER_NORMAL) {
		reg->src_addr_u = opt->source_frame->buf_cb;
		reg->src_addr_v = opt->source_frame->buf_cr;
	} else {
		reg->src_addr_u = opt->source_frame->buf_cr;
		reg->src_addr_v = opt->source_frame->buf_cb;
	}
	reg->src_stride = (opt->source_frame->stride << 16) |
			  (stride_c << 0);
	reg->src_format = (color_format << 28) |
			  (is_24bit << 25) |
			  (is_ayuv << 24) |
			  (opt->update_last_2bit << 23) |
			  ((opt->last_2bit_data & 0x3) << 21) |
			  (is_csc_format << 20) |
			  (opt->csc.format_order << 16) |
			  (endian << 12) |
			  (is_lsb << 6) |
			  (is_3p4b << 5) |
			  (is_10bit << 4) |
			  (src_frame_format << 0);
	reg->src_axi_sel = DEFAULT_SRC_AXI;
	if (opt->code_option.implicit_header_encode) {
		reg->code_option = (opt->src_end_flag << 10) |
				   (opt->code_option.encode_eob << 7) |
				   (opt->code_option.encode_eos << 6) |
				   (CODEOPT_ENC_VCL) |
				   (CODEOPT_ENC_HEADER_IMPLICIT);
	} else {
		reg->code_option = (opt->code_option.encode_eob << 7) |
				   (opt->code_option.encode_eos << 6) |
				   (opt->code_option.encode_pps << 4) |
				   (opt->code_option.encode_sps << 3) |
				   (opt->code_option.encode_vps << 2) |
				   (opt->code_option.encode_vcl << 1) |
				   (opt->code_option.implicit_header_encode << 0);
	}
	reg->pic_param = (opt->intra_4x4 << 27) |
			 (opt->force_pic_type << 21) |
			 (opt->force_pic_type_enable << 20) |
			 (opt->force_pic_qp_b << 14) |
			 (opt->force_pic_qp_p << 8) |
			 (opt->force_pic_qp_i << 2) |
			 (opt->force_pic_qp_enable << 1) |
			 (opt->skip_picture << 0);
	reg->longterm_pic = (opt->use_longterm_ref << 1) |
			    (opt->use_cur_src_as_longterm_pic << 0);
	reg->prefix_sei_nal_addr = opt->sei_nal.prefix_sei_nal_addr;
	reg->prefix_sei_info = (opt->sei_nal.prefix_sei_nal_data_size << 16) |
			       (opt->sei_nal.prefix_sei_nal_enable << 0);
	reg->suffix_sei_nal_addr = opt->sei_nal.suffix_sei_nal_addr;
	reg->suffix_sei_info = (opt->sei_nal.suffix_sei_nal_data_size << 16) |
			       (opt->sei_nal.suffix_sei_nal_enable << 0);
	reg->timestamp = ((opt->timestamp.hour & 0x1F) << 26) |
			 ((opt->timestamp.min & 0x3F) << 20) |
			 ((opt->timestamp.sec & 0x3F) << 14) |
			 ((opt->timestamp.ms & 0x3FFF));
	reg->csc_coeff[0] = ((opt->csc.coef_ry & 0x3FF) << 20) |
			    ((opt->csc.coef_gy & 0x3FF) << 10) |
			    ((opt->csc.coef_by & 0x3FF) << 0);
	reg->csc_coeff[1] = ((opt->csc.coef_rcb & 0x3FF) << 20) |
			    ((opt->csc.coef_gcb & 0x3FF) << 10) |
			    ((opt->csc.coef_bcb & 0x3FF) << 0);
	reg->csc_coeff[2] = ((opt->csc.coef_rcr & 0x3FF) << 20) |
			    ((opt->csc.coef_gcr & 0x3FF) << 10) |
			    ((opt->csc.coef_bcr & 0x3FF) << 0);
	reg->csc_coeff[3] = ((opt->csc.offset_y & 0x3FF) << 20) |
			    ((opt->csc.offset_cb & 0x3FF) << 10) |
			    ((opt->csc.offset_cr & 0x3FF) << 0);
}

int wave6_vpu_encode(struct vpu_instance *vpu_inst, struct enc_param *option, u32 *fail_res)
{
	struct enc_cmd_enc_pic_reg reg;
	struct enc_info *p_enc_info = &vpu_inst->codec_info->enc_info;
	struct vpu_attr *attr = &vpu_inst->dev->attr;
	int ret;

	memset(&reg, 0, sizeof(struct enc_cmd_enc_pic_reg));

	wave6_gen_enc_pic_reg(p_enc_info, vpu_inst->cbcr_interleave,
			      vpu_inst->nv21, option, &reg);

	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_BS_START, reg.bs_start);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_BS_SIZE, reg.bs_size);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_BS_OPTION, reg.bs_option);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_USE_SEC_AXI, reg.use_sec_axi);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_REPORT_PARAM, reg.report_param);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_MV_HISTO_CLASS0, reg.mv_histo_class0);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_MV_HISTO_CLASS1, reg.mv_histo_class1);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_CUSTOM_MAP_OPTION_PARAM, reg.custom_map_param);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_CUSTOM_MAP_OPTION_ADDR, reg.custom_map_addr);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_SRC_PIC_IDX, reg.src_pic_idx);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_SRC_ADDR_Y, reg.src_addr_y);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_SRC_ADDR_U, reg.src_addr_u);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_SRC_ADDR_V, reg.src_addr_v);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_SRC_STRIDE, reg.src_stride);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_SRC_FORMAT, reg.src_format);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_SRC_AXI_SEL, reg.src_axi_sel);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_CODE_OPTION, reg.code_option);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_PIC_PARAM, reg.pic_param);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_LONGTERM_PIC, reg.longterm_pic);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_PREFIX_SEI_NAL_ADDR, reg.prefix_sei_nal_addr);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_PREFIX_SEI_INFO, reg.prefix_sei_info);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_SUFFIX_SEI_NAL_ADDR, reg.suffix_sei_nal_addr);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_SUFFIX_SEI_INFO, reg.suffix_sei_info);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_TIMESTAMP, reg.timestamp);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_CSC_COEFF_0, reg.csc_coeff[0]);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_CSC_COEFF_1, reg.csc_coeff[1]);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_CSC_COEFF_2, reg.csc_coeff[2]);
	vpu_write_reg(vpu_inst->dev, W6_CMD_ENC_PIC_CSC_COEFF_3, reg.csc_coeff[3]);

	wave6_send_command(vpu_inst->dev, vpu_inst->id, vpu_inst->std, W6_ENC_PIC);
	if (attr->support_command_queue) {
		ret = wave6_wait_vpu_busy(vpu_inst->dev, W6_VPU_BUSY_STATUS);
		if (ret) {
			dev_err(vpu_inst->dev->dev, "%s: timeout\n", __func__);
			return -ETIMEDOUT;
		}

		if (!vpu_read_reg(vpu_inst->dev, W6_RET_SUCCESS)) {
			*fail_res = vpu_read_reg(vpu_inst->dev, W6_RET_FAIL_REASON);
			wave6_print_reg_err(vpu_inst->dev, *fail_res);
			return -EIO;
		}
	}

	return 0;
}

int wave6_vpu_enc_get_result(struct vpu_instance *vpu_inst, struct enc_output_info *result)
{
	struct enc_info *p_enc_info = &vpu_inst->codec_info->enc_info;
	struct vpu_attr *attr = &vpu_inst->dev->attr;
	u32 reg_val;
	int ret;

	if (attr->support_command_queue) {
		ret = wave6_send_query(vpu_inst->dev, vpu_inst->id, vpu_inst->std, W6_QUERY_GET_RESULT);
		if (ret)
			return ret;

		result->encoding_success = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_ENCODING_SUCCESS);
		if (!result->encoding_success)
			result->error_reason = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_ERR_INFO);
		else
			result->warn_info = vpu_read_reg(vpu_inst->dev, W6_RET_DEC_WARN_INFO);
	} else {
		result->encoding_success = vpu_read_reg(vpu_inst->dev, W6_RET_SUCCESS);
		if (!result->encoding_success) {
			result->error_reason = vpu_read_reg(vpu_inst->dev, W6_RET_FAIL_REASON);

			wave6_print_reg_err(vpu_inst->dev, result->error_reason);
			return -EIO;
		}
	}

	result->enc_pic_cnt = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_PIC_NUM);
	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_PIC_TYPE);
	result->pic_type = reg_val & 0xFFFF;

	result->enc_vcl_nut = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_VCL_NUT);
	result->recon_frame_index = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_PIC_IDX);
	if (result->recon_frame_index >= 0)
		result->recon_frame = vpu_inst->frame_buf[result->recon_frame_index];

	result->non_ref_pic_flag = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_PIC_NON_REF_PIC_FLAG);
	result->num_of_slices = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_PIC_SLICE_NUM);
	result->pic_skipped = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_PIC_SKIP);
	result->num_of_intra = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_PIC_NUM_INTRA);
	result->num_of_merge = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_PIC_NUM_MERGE);
	result->num_of_skip_block = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_PIC_NUM_SKIP);
	result->bitstream_wrap_around = 0; // only support line-buffer mode.

	result->avg_ctu_qp = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_PIC_AVG_CTU_QP);
	result->enc_pic_byte = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_PIC_BYTE);

	result->enc_gop_pic_idx = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_GOP_PIC_IDX);
	result->enc_pic_poc = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_PIC_POC);
	result->enc_src_idx = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_USED_SRC_IDX);
	result->wr_ptr = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_WR_PTR);
	result->rd_ptr = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_RD_PTR);

	result->pic_distortion_low = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_PIC_DIST_LOW);
	result->pic_distortion_high = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_PIC_DIST_HIGH);

	result->mv_histo.cnt0 = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_HISTO_CNT_0);
	result->mv_histo.cnt1 = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_HISTO_CNT_1);
	result->mv_histo.cnt2 = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_HISTO_CNT_2);
	result->mv_histo.cnt3 = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_HISTO_CNT_3);
	result->mv_histo.cnt4 = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_HISTO_CNT_4);

	result->fme_sum.lower_x0  = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_SUM_ME0_X_DIR_LOWER);
	result->fme_sum.higher_x0 = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_SUM_ME0_X_DIR_HIGHER);
	result->fme_sum.lower_y0  = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_SUM_ME0_Y_DIR_LOWER);
	result->fme_sum.higher_y0 = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_SUM_ME0_Y_DIR_HIGHER);
	result->fme_sum.lower_x1  = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_SUM_ME1_X_DIR_LOWER);
	result->fme_sum.higher_x1 = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_SUM_ME1_X_DIR_HIGHER);
	result->fme_sum.lower_y1  = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_SUM_ME1_Y_DIR_LOWER);
	result->fme_sum.higher_y1 = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_SUM_ME1_Y_DIR_HIGHER);

	result->src_y_addr = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_SRC_Y_ADDR);
	result->custom_map_addr = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_CUSTOM_MAP_OPTION_ADDR);
	result->prefix_sei_nal_addr = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_PREFIX_SEI_NAL_ADDR);
	result->suffix_sei_nal_addr = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_SUFFIX_SEI_NAL_ADDR);

	reg_val = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_TIMESTAMP);
	result->timestamp.hour = (reg_val >> 26) & 0x1F;
	result->timestamp.min = (reg_val >> 20) & 0x3F;
	result->timestamp.sec = (reg_val >> 14) & 0x3F;
	result->timestamp.ms = reg_val & 0x3FFF;

	result->bitstream_buffer = vpu_read_reg(vpu_inst->dev, W6_RET_ENC_RD_PTR);

	if (result->recon_frame_index == RECON_IDX_FLAG_HEADER_ONLY)
		result->bitstream_size = result->enc_pic_byte;
	else if (result->recon_frame_index < 0)
		result->bitstream_size = 0;
	else
		result->bitstream_size = result->enc_pic_byte;

	result->cycle.host_cmd_s = vpu_read_reg(vpu_inst->dev, W6_RET_CMD_CQ_IN_TICK);
	result->cycle.host_cmd_e = vpu_read_reg(vpu_inst->dev, W6_RET_CMD_RQ_OUT_TICK);
	result->cycle.proc_s = vpu_read_reg(vpu_inst->dev, W6_RET_CMD_HW_RUN_TICK);
	result->cycle.proc_e = vpu_read_reg(vpu_inst->dev, W6_RET_CMD_HW_DONE_TICK);
	result->cycle.vpu_s = vpu_read_reg(vpu_inst->dev, W6_RET_CMD_FW_RUN_TICK);
	result->cycle.vpu_e = vpu_read_reg(vpu_inst->dev, W6_RET_CMD_FW_DONE_TICK);
	result->cycle.frame_cycle = (result->cycle.vpu_e - result->cycle.host_cmd_s) * p_enc_info->cycle_per_tick;
	result->cycle.proc_cycle = (result->cycle.proc_e - result->cycle.proc_s) * p_enc_info->cycle_per_tick;
	result->cycle.vpu_cycle = ((result->cycle.vpu_e - result->cycle.vpu_s) - (result->cycle.proc_e - result->cycle.proc_s)) * p_enc_info->cycle_per_tick;

	return 0;
}

int wave6_vpu_enc_fini_seq(struct vpu_instance *vpu_inst, u32 *fail_res)
{
	int ret;

	wave6_send_command(vpu_inst->dev, vpu_inst->id, vpu_inst->std, W6_DESTROY_INSTANCE);
	ret = wave6_wait_vpu_busy(vpu_inst->dev, W6_VPU_BUSY_STATUS);
	if (ret)
		return -ETIMEDOUT;

	if (!vpu_read_reg(vpu_inst->dev, W6_RET_SUCCESS)) {
		*fail_res = vpu_read_reg(vpu_inst->dev, W6_RET_FAIL_REASON);
		wave6_print_reg_err(vpu_inst->dev, *fail_res);
		return -EIO;
	}

	return 0;
}

static int wave6_vpu_enc_check_gop_param(struct vpu_instance *vpu_inst,
					 struct enc_wave_param *p_param)
{
	struct vpu_device *vpu_dev = vpu_inst->dev;
	struct device *dev = vpu_dev->dev;
	int i = 0;
	bool low_delay = true;

	if (p_param->gop_preset_idx == PRESET_IDX_CUSTOM_GOP) {
		if (p_param->gop_param.custom_gop_size > 1) {
			s32 min_val = p_param->gop_param.pic_param[0].poc_offset;

			for (i = 1; i < p_param->gop_param.custom_gop_size; i++) {
				if (min_val > p_param->gop_param.pic_param[i].poc_offset) {
					low_delay = false;
					break;
				}
				min_val = p_param->gop_param.pic_param[i].poc_offset;
			}
		}
	} else if (p_param->gop_preset_idx == PRESET_IDX_ALL_I ||
		   p_param->gop_preset_idx == PRESET_IDX_IPP ||
		   p_param->gop_preset_idx == PRESET_IDX_IBBB ||
		   p_param->gop_preset_idx == PRESET_IDX_IPPPP ||
		   p_param->gop_preset_idx == PRESET_IDX_IBBBB ||
		   p_param->gop_preset_idx == PRESET_IDX_IPP_SINGLE) {
	}

	if (p_param->gop_preset_idx >= PRESET_IDX_MAX) {
		dev_err(dev, "gop_preset_idx: %d\n", p_param->gop_preset_idx);
		return -EINVAL;
	}

	if (p_param->gop_preset_idx == PRESET_IDX_CUSTOM_GOP) {
		if (p_param->gop_param.custom_gop_size < 1 || p_param->gop_param.custom_gop_size > MAX_GOP_NUM) {
			dev_err(dev, "custom_gop_size: %d\n", p_param->gop_param.custom_gop_size);
			return -EINVAL;
		}
		for (i = 0; i < p_param->gop_param.custom_gop_size; i++) {
			struct custom_gop_pic_param pic_param = p_param->gop_param.pic_param[i];

			if ((pic_param.pic_type != PIC_TYPE_I) && (pic_param.pic_type != PIC_TYPE_P) && (pic_param.pic_type != PIC_TYPE_B)) {
				dev_err(dev, "pic_param[%d].pic_type: %d\n", i, pic_param.pic_type);
				return -EINVAL;
			}
			if ((pic_param.poc_offset < 1) || (pic_param.poc_offset > p_param->gop_param.custom_gop_size)) {
				dev_err(dev, "pic_param[%d].poc_offset: %d\n", i, pic_param.poc_offset);
				return -EINVAL;
			}
			if ((pic_param.use_multi_ref_p < 0) || (pic_param.use_multi_ref_p > 1)) {
				dev_err(dev, "pic_param[%d].use_multi_ref_p: %d\n", i, pic_param.use_multi_ref_p);
				return -EINVAL;
			}
			if ((pic_param.temporal_id < 0) || (pic_param.temporal_id > 3)) {
				dev_err(dev, "pic_param[%d].temporal_id: %d\n", i, pic_param.temporal_id);
				return -EINVAL;
			}
		}
		if ((vpu_inst->std == W_AVC_ENC) && (!low_delay)) {
			for (i = 0; i < p_param->gop_param.custom_gop_size; i++) {
				if (p_param->gop_param.pic_param[i].temporal_id > 0) {
					dev_err(dev, "std: %d, pic_param[%d].temporal_id: %d\n", vpu_inst->std, i, p_param->gop_param.pic_param[i].temporal_id);
					return -EINVAL;
				}
			}
		}
	}

	if (vpu_inst->std == W_HEVC_ENC) {
		if (p_param->decoding_refresh_type > DEC_REFRESH_TYPE_IDR) {
			dev_err(dev, "decoding_refresh_type: %d\n", p_param->decoding_refresh_type);
			return -EINVAL;
		}
	} else {
		if (p_param->decoding_refresh_type != DEC_REFRESH_TYPE_NON_IRAP) {
			dev_err(dev, "decoding_refresh_type: %d\n", p_param->decoding_refresh_type);
			return -EINVAL;
		}
	}

	return 0;
}

static int wave6_vpu_enc_check_tile_slice_param(struct vpu_instance *vpu_inst,
						int width, int height,
						struct enc_wave_param *p_param)
{
	struct vpu_device *vpu_dev = vpu_inst->dev;
	struct device *dev = vpu_dev->dev;

	if (vpu_inst->std == W_AV1_ENC) {
		if (p_param->slice_mode != 0) {
			dev_err(dev, "slice_mode: %d\n", p_param->slice_mode);
			return -EINVAL;
		}
		if (p_param->slice_arg != 0) {
			dev_err(dev, "slice_arg: %d\n", p_param->slice_arg);
			return -EINVAL;
		}
		if (p_param->col_tile_num < 1 || p_param->col_tile_num > 2) {
			dev_err(dev, "col_tile_num: %d\n", p_param->col_tile_num);
			return -EINVAL;
		}
		if (p_param->row_tile_num < 1 || p_param->row_tile_num > 16) {
			dev_err(dev, "row_tile_num: %d\n", p_param->row_tile_num);
			return -EINVAL;
		}
	} else {
		if (p_param->slice_mode > 2) {
			dev_err(dev, "slice_mode: %d\n", p_param->slice_mode);
			return -EINVAL;
		}
		if (p_param->slice_mode == 1) {
			unsigned int ctu_size = (vpu_inst->std == W_AVC_ENC) ? 16 : 64;
			unsigned int mb_num = ((width + ctu_size - 1) / ctu_size) * ((height + ctu_size - 1) / ctu_size);

			if (p_param->slice_arg < 1 || p_param->slice_arg > 0x3FFFF) {
				dev_err(dev, "slice_arg: %d\n", p_param->slice_arg);
				return -EINVAL;
			}
			if (p_param->slice_arg > mb_num) {
				dev_info(dev, "slice_arg: %d, mb_num: %d\n", p_param->slice_arg, mb_num);
				p_param->slice_arg = mb_num;
			}
			if ((vpu_inst->std == W_AVC_ENC) && (p_param->slice_arg < 4)) {
				dev_info(dev, "std: %d, slice_arg: %d\n", vpu_inst->std, p_param->slice_arg);
				p_param->slice_arg = 4;
			}
		}
		if (p_param->col_tile_num != 0) {
			dev_err(dev, "col_tile_num: %d\n", p_param->col_tile_num);
			return -EINVAL;
		}
		if (p_param->row_tile_num != 0) {
			dev_err(dev, "row_tile_num: %d\n", p_param->row_tile_num);
			return -EINVAL;
		}
	}

	return 0;
}

static int wave6_vpu_enc_check_rc_param(struct vpu_instance *vpu_inst,
					struct enc_wave_param *p_param)
{
	struct vpu_device *vpu_dev = vpu_inst->dev;
	struct device *dev = vpu_dev->dev;

	if (p_param->frame_rate < 1 || p_param->frame_rate > 960) {
		dev_err(dev, "frame_rate: %d\n", p_param->frame_rate);
		return -EINVAL;
	}
	if (p_param->enc_bit_rate > 1500000000) {
		dev_err(dev, "enc_bit_rate: %d\n", p_param->enc_bit_rate);
		return -EINVAL;
	}
	if (p_param->qp > 51) {
		dev_err(dev, "qp: %d\n", p_param->qp);
		return -EINVAL;
	}
	if ((p_param->min_qp_i > 51) || (p_param->min_qp_p > 51) || (p_param->min_qp_b > 51)) {
		dev_err(dev, "min_qp_i: %d, min_qp_p: %d, min_qp_b: %d\n", p_param->min_qp_i, p_param->min_qp_p, p_param->min_qp_b);
		return -EINVAL;
	}
	if ((p_param->max_qp_i > 51) || (p_param->max_qp_p > 51) || (p_param->max_qp_b > 51)) {
		dev_err(dev, "max_qp_i: %d, max_qp_p: %d, max_qp_b: %d\n", p_param->max_qp_i, p_param->max_qp_p, p_param->max_qp_b);
		return -EINVAL;
	}
	if (p_param->min_qp_i > p_param->max_qp_i) {
		dev_err(dev, "min_qp_i: %d, max_qp_i: %d\n", p_param->min_qp_i, p_param->max_qp_i);
		return -EINVAL;
	}
	if (p_param->min_qp_p > p_param->max_qp_p) {
		dev_err(dev, "min_qp_p: %d, max_qp_p: %d\n", p_param->min_qp_p, p_param->max_qp_p);
		return -EINVAL;
	}
	if (p_param->min_qp_b > p_param->max_qp_b) {
		dev_err(dev, "min_qp_b: %d, max_qp_b: %d\n", p_param->min_qp_b, p_param->max_qp_b);
		return -EINVAL;
	}
	if (p_param->rc_initial_qp < -1 || p_param->rc_initial_qp > 51) {
		dev_err(dev, "rc_initial_qp: %d\n", p_param->rc_initial_qp);
		return -EINVAL;
	}
	if (p_param->en_rate_control != 1 && p_param->en_rate_control != 0) {
		dev_err(dev, "en_rate_control: %d\n", p_param->en_rate_control);
		return -EINVAL;
	}
	if (p_param->rc_mode > 1) {
		dev_err(dev, "rc_mode: %d\n", p_param->rc_mode);
		return -EINVAL;
	}
	if (p_param->en_rate_control) {
		if (p_param->enc_bit_rate <= p_param->frame_rate) {
			dev_err(dev, "enc_bit_rate: %d, frame_rate: %d\n", p_param->enc_bit_rate, p_param->frame_rate);
			return -EINVAL;
		}
		if (p_param->rc_initial_qp != -1) {
			if (p_param->rc_initial_qp < p_param->min_qp_i) {
				dev_err(dev, "rc_initial_qp: %d, min_qp_i: %d\n", p_param->rc_initial_qp, p_param->min_qp_i);
				return -EINVAL;
			}
			if (p_param->rc_initial_qp > p_param->max_qp_i) {
				dev_err(dev, "rc_initial_qp: %d, max_qp_i: %d\n", p_param->rc_initial_qp, p_param->max_qp_i);
				return -EINVAL;
			}
		}
	} else {
		if (p_param->qp < p_param->min_qp_i) {
			dev_err(dev, "qp: %d, min_qp_i: %d\n", p_param->qp, p_param->min_qp_i);
			return -EINVAL;
		}
		if (p_param->qp < p_param->min_qp_p) {
			dev_err(dev, "qp: %d, min_qp_p: %d\n", p_param->qp, p_param->min_qp_p);
			return -EINVAL;
		}
		if (p_param->qp < p_param->min_qp_b) {
			dev_err(dev, "qp: %d, min_qp_b: %d\n", p_param->qp, p_param->min_qp_b);
			return -EINVAL;
		}
		if (p_param->qp > p_param->max_qp_i) {
			dev_err(dev, "qp: %d, max_qp_i: %d\n", p_param->qp, p_param->max_qp_i);
			return -EINVAL;
		}
		if (p_param->qp > p_param->max_qp_p) {
			dev_err(dev, "qp: %d, max_qp_p: %d\n", p_param->qp, p_param->max_qp_p);
			return -EINVAL;
		}
		if (p_param->qp > p_param->max_qp_b) {
			dev_err(dev, "qp: %d, max_qp_b: %d\n", p_param->qp, p_param->max_qp_b);
			return -EINVAL;
		}
	}

	return 0;
}

static int wave6_vpu_enc_check_intra_param(struct vpu_instance *vpu_inst,
					   int width, int height,
					   struct enc_wave_param *p_param)
{
	struct vpu_device *vpu_dev = vpu_inst->dev;
	struct device *dev = vpu_dev->dev;
	unsigned int ctu_size = (vpu_inst->std == W_AVC_ENC) ? 16 : 64;
	unsigned int num_ctu_col = (width + ctu_size - 1) / ctu_size;
	unsigned int num_ctu_row = (height + ctu_size - 1) / ctu_size;

	if (p_param->intra_refresh_mode > INTRA_REFRESH_COLUMN) {
		dev_err(dev, "intra_refresh_mode: %d\n", p_param->intra_refresh_mode);
		return -EINVAL;
	}
	if (p_param->intra_refresh_mode != INTRA_REFRESH_NONE) {
		if (p_param->intra_refresh_arg < 1 || p_param->intra_refresh_arg > 511) {
			dev_err(dev, "intra_refresh_arg: %d\n", p_param->intra_refresh_arg);
			return -EINVAL;
		}
	}
	if ((p_param->intra_refresh_mode == INTRA_REFRESH_ROW) &&
	    (p_param->intra_refresh_arg > num_ctu_row)) {
		dev_err(dev, "intra_refresh_mode: %d, intra_refresh_arg: %d\n",
			p_param->intra_refresh_mode, p_param->intra_refresh_arg);
		return -EINVAL;
	}
	if ((p_param->intra_refresh_mode == INTRA_REFRESH_COLUMN) &&
	    (p_param->intra_refresh_arg > num_ctu_col)) {
		dev_err(dev, "intra_refresh_mode: %d, intra_refresh_arg: %d\n",
			p_param->intra_refresh_mode, p_param->intra_refresh_arg);
		return -EINVAL;
	}

	return 0;
}

static int wave6_vpu_enc_check_custom_param(struct vpu_instance *vpu_inst,
					    struct enc_wave_param *p_param)
{
	struct vpu_device *vpu_dev = vpu_inst->dev;
	struct device *dev = vpu_dev->dev;
	int i;

	if (p_param->en_qp_map != 1 && p_param->en_qp_map != 0) {
		dev_err(dev, "en_qp_map: %d\n", p_param->en_qp_map);
		return -EINVAL;
	}
	if (p_param->en_mode_map != 1 && p_param->en_mode_map != 0) {
		dev_err(dev, "en_mode_map: %d\n", p_param->en_mode_map);
		return -EINVAL;
	}
	if (p_param->en_custom_lambda != 1 && p_param->en_custom_lambda != 0) {
		dev_err(dev, "en_custom_lambda: %d\n", p_param->en_custom_lambda);
		return -EINVAL;
	}
	for (i = 0; i < MAX_CUSTOM_LAMBDA_NUM; i++) {
		if (p_param->custom_lambda_ssd[i] > 16383) {
			dev_err(dev, "custom_lambda_ssd[%d]: %d\n", i, p_param->custom_lambda_ssd[i]);
			return -EINVAL;
		}
		if (p_param->custom_lambda_sad[i] > 127) {
			dev_err(dev, "custom_lambda_sad[%d]: %d\n", i, p_param->custom_lambda_sad[i]);
			return -EINVAL;
		}
	}

	return 0;
}

static int wave6_vpu_enc_check_conf_win_size_param(struct vpu_instance *vpu_inst,
						   int width, int height,
						   struct vpu_rect conf_win)
{
	struct vpu_device *vpu_dev = vpu_inst->dev;
	struct device *dev = vpu_dev->dev;

	if (conf_win.left % 2 || conf_win.top % 2 || conf_win.right % 2 || conf_win.bottom % 2) {
		dev_err(dev, "conf_win.left: %d, conf_win.top: %d, conf_win.right: %d, conf_win.bottom: %d\n",
			conf_win.left, conf_win.top, conf_win.right, conf_win.bottom);
		return -EINVAL;
	}
	if (conf_win.left > 8192 || conf_win.top > 8192 || conf_win.right > 8192 || conf_win.bottom > 8192) {
		dev_err(dev, "conf_win.left: %d, conf_win.top: %d, conf_win.right: %d, conf_win.bottom: %d\n",
			conf_win.left, conf_win.top, conf_win.right, conf_win.bottom);
		return -EINVAL;
	}
	if ((conf_win.right + conf_win.left) > width) {
		dev_err(dev, "conf_win.left: %d, conf_win.right: %d, width: %d\n", conf_win.left, conf_win.right, width);
		return -EINVAL;
	}
	if ((conf_win.bottom + conf_win.top) > height) {
		dev_err(dev, "conf_win.top: %d, conf_win.bottom: %d, height: %d\n", conf_win.top, conf_win.bottom, height);
		return -EINVAL;
	}

	return 0;
}

static int wave6_vpu_enc_check_temporal_layer_param(struct vpu_instance *vpu_inst,
						    struct enc_wave_param *p_param)
{
	struct vpu_device *vpu_dev = vpu_inst->dev;
	struct device *dev = vpu_dev->dev;
	int i;

	if ((p_param->temp_layer_cnt < 1) || (p_param->temp_layer_cnt > 4)) {
		dev_err(dev, "temp_layer_cnt: %d\n", p_param->temp_layer_cnt);
		return -EINVAL;
	}
	if (p_param->temp_layer_cnt > 1) {
		if (p_param->gop_preset_idx == PRESET_IDX_CUSTOM_GOP || p_param->gop_preset_idx == PRESET_IDX_ALL_I) {
			dev_err(dev, "temp_layer_cnt: %d, gop_preset_idx: %d\n", p_param->temp_layer_cnt, p_param->gop_preset_idx);
			return -EINVAL;
		}
	}
	for (i = 0; i < MAX_NUM_CHANGEABLE_TEMPORAL_LAYER; i++) {
		if (p_param->temp_layer[i].en_change_qp != 1 && p_param->temp_layer[i].en_change_qp != 0) {
			dev_err(dev, "temp_layer[%d].en_change_qp: %d\n", i, p_param->temp_layer[i].en_change_qp);
			return -EINVAL;
		}
		if (p_param->temp_layer[i].qp_b > 51) {
			dev_err(dev, "temp_layer[%d].qp_b: %d\n", i, p_param->temp_layer[i].qp_b);
			return -EINVAL;
		}
		if (p_param->temp_layer[i].qp_p > 51) {
			dev_err(dev, "temp_layer[%d].qp_p: %d\n", i, p_param->temp_layer[i].qp_p);
			return -EINVAL;
		}
		if (p_param->temp_layer[i].qp_i > 51) {
			dev_err(dev, "temp_layer[%d].qp_i: %d\n", i, p_param->temp_layer[i].qp_i);
			return -EINVAL;
		}
	}

	return 0;
}

int wave6_vpu_enc_check_open_param(struct vpu_instance *vpu_inst, struct enc_open_param *pop)
{
	struct vpu_device *vpu_dev = vpu_inst->dev;
	struct device *dev = vpu_dev->dev;
	struct vpu_attr *attr = &vpu_inst->dev->attr;
	struct enc_wave_param *p_param = &pop->wave_param;

	if (vpu_inst->std != W_HEVC_ENC && vpu_inst->std != W_AVC_ENC && vpu_inst->std != W_AV1_ENC) {
		dev_err(dev, "std %d\n", vpu_inst->std);
		return -EOPNOTSUPP;
	}

	if (pop->pic_width % W6_ENC_PIC_SIZE_STEP || pop->pic_height % W6_ENC_PIC_SIZE_STEP) {
		dev_err(dev, "pic_width: %d | pic_height: %d\n", pop->pic_width, pop->pic_height);
		return -EINVAL;
	}
	if (pop->pic_width < W6_MIN_ENC_PIC_WIDTH || pop->pic_width > W6_MAX_ENC_PIC_WIDTH) {
		dev_err(dev, "pic_width: %d\n", pop->pic_width);
		return -EINVAL;
	}
	if (pop->pic_height < W6_MIN_ENC_PIC_HEIGHT || pop->pic_height > W6_MAX_ENC_PIC_HEIGHT) {
		dev_err(dev, "pic_height: %d\n", pop->pic_height);
		return -EINVAL;
	}

	if (pop->packed_format && vpu_inst->cbcr_interleave == 1) {
		dev_err(dev, "packed_format: %d, cbcr_interleave: %d\n", pop->packed_format, vpu_inst->cbcr_interleave);
		return -EINVAL;
	}
	if (pop->packed_format && vpu_inst->nv21 == 1) {
		dev_err(dev, "packed_format: %d, nv21: %d\n", pop->packed_format, vpu_inst->nv21);
		return -EINVAL;
	}
	if ((pop->src_format == FORMAT_RGB_32BIT_PACKED) || (pop->src_format == FORMAT_YUV444_32BIT_PACKED) ||
	    (pop->src_format == FORMAT_RGB_P10_32BIT_PACKED) || (pop->src_format == FORMAT_YUV444_P10_32BIT_PACKED)) {
		if (vpu_inst->cbcr_interleave == 0) {
			dev_err(dev, "src_format: %d, cbcr_interleave: %d\n", pop->src_format, vpu_inst->cbcr_interleave);
			return -EINVAL;
		}
		if (vpu_inst->nv21 == 1) {
			dev_err(dev, "src_format: %d, nv21: %d\n", pop->src_format, vpu_inst->nv21);
			return -EINVAL;
		}
	}
	if ((pop->src_format == FORMAT_RGB_24BIT_PACKED) || (pop->src_format == FORMAT_YUV444_24BIT_PACKED)) {
		if ((vpu_inst->cbcr_interleave == 0) || (vpu_inst->nv21 == 1)) {
			dev_err(dev, "src_format: %d, cbcr_interleave: %d, nv21: %d\n", pop->src_format, vpu_inst->cbcr_interleave, vpu_inst->nv21);
			return -EINVAL;
		}
	}
	if (pop->src_format == FORMAT_YUV444_24BIT) {
		if (vpu_inst->cbcr_interleave == 0) {
			dev_err(dev, "src_format: %d, cbcr_interleave: %d\n", pop->src_format, vpu_inst->cbcr_interleave);
			return -EINVAL;
		}
	}

	if (wave6_vpu_enc_check_gop_param(vpu_inst, p_param)) {
		dev_err(dev, "failed wave6_vpu_enc_check_gop_param()\n");
		return -EINVAL;
	}
	if (wave6_vpu_enc_check_tile_slice_param(vpu_inst, pop->pic_width, pop->pic_height, p_param)) {
		dev_err(dev, "failed wave6_vpu_enc_check_tile_slice_param()\n");
		return -EINVAL;
	}
	if (wave6_vpu_enc_check_rc_param(vpu_inst, p_param)) {
		dev_err(dev, "failed wave6_vpu_enc_check_rc_param()\n");
		return -EINVAL;
	}
	if (wave6_vpu_enc_check_intra_param(vpu_inst, pop->pic_width, pop->pic_height, p_param)) {
		dev_err(dev, "failed wave6_vpu_enc_check_intra_param()\n");
		return -EINVAL;
	}
	if (wave6_vpu_enc_check_custom_param(vpu_inst, p_param)) {
		dev_err(dev, "failed wave6_vpu_enc_check_custom_param()\n");
		return -EINVAL;
	}
	if (wave6_vpu_enc_check_conf_win_size_param(vpu_inst, pop->pic_width, pop->pic_height, p_param->conf_win)) {
		dev_err(dev, "failed wave6_vpu_enc_check_conf_win_size_param()\n");
		return -EINVAL;
	}
	if (wave6_vpu_enc_check_temporal_layer_param(vpu_inst, p_param)) {
		dev_err(dev, "failed wave6_vpu_enc_check_temporal_layer_param()\n");
		return -EINVAL;
	}

	if (p_param->internal_bit_depth != 8 && p_param->internal_bit_depth != 10) {
		dev_err(dev, "internal_bit_depth: %d\n", p_param->internal_bit_depth);
		return -EINVAL;
	}
	if (p_param->intra_period > 2047) {
		dev_err(dev, "intra_period: %d\n", p_param->intra_period);
		return -EINVAL;
	}
	if (p_param->intra_period == 1 && p_param->gop_preset_idx == PRESET_IDX_ALL_I) {
		dev_err(dev, "intra_period: %d, gop_preset_idx: %d\n", p_param->intra_period, p_param->gop_preset_idx);
		return -EINVAL;
	}
	if (p_param->en_long_term != 1 && p_param->en_long_term != 0) {
		dev_err(dev, "en_long_term: %d\n", p_param->en_long_term);
		return -EINVAL;
	}
	if (p_param->vbv_buffer_size < 10 || p_param->vbv_buffer_size > 100000) {
		dev_err(dev, "vbv_buffer_size: %d\n", p_param->vbv_buffer_size);
		return -EINVAL;
	}
	if (p_param->en_cu_level_rate_control != 1 && p_param->en_cu_level_rate_control != 0) {
		dev_err(dev, "en_cu_level_rate_control: %d\n", p_param->en_cu_level_rate_control);
		return -EINVAL;
	}
	if (p_param->en_hvs_qp != 1 && p_param->en_hvs_qp != 0) {
		dev_err(dev, "en_hvs_qp: %d\n", p_param->en_hvs_qp);
		return -EINVAL;
	}
	if (p_param->en_hvs_qp) {
		if (p_param->hvs_qp_scale_div2 < 1 || p_param->hvs_qp_scale_div2 > 4) {
			dev_err(dev, "hvs_qp_scale_div2: %d\n", p_param->hvs_qp_scale_div2);
			return -EINVAL;
		}
	}
	if (p_param->max_delta_qp > 12) {
		dev_err(dev, "max_delta_qp: %d\n", p_param->max_delta_qp);
		return -EINVAL;
	}
	if (p_param->rc_update_speed > 255) {
		dev_err(dev, "rc_update_speed: %d\n", p_param->rc_update_speed);
		return -EINVAL;
	}
	if (p_param->max_bit_rate > 1500000000) {
		dev_err(dev, "max_bit_rate: %d\n", p_param->max_bit_rate);
		return -EINVAL;
	}
	if (p_param->rc_initial_level > 15) {
		dev_err(dev, "rc_initial_level: %d\n", p_param->rc_initial_level);
		return -EINVAL;
	}
	if (p_param->pic_rc_max_dqp > 51) {
		dev_err(dev, "pic_rc_max_dqp: %d\n", p_param->pic_rc_max_dqp);
		return -EINVAL;
	}
	if (p_param->en_bg_detect != 1 && p_param->en_bg_detect != 0) {
		dev_err(dev, "en_bg_detect: %d\n", p_param->en_bg_detect);
		return -EINVAL;
	}
	if (p_param->bg_th_diff > 255) {
		dev_err(dev, "bg_th_diff: %d\n", p_param->bg_th_diff);
		return -EINVAL;
	}
	if (p_param->bg_th_mean_diff > 255) {
		dev_err(dev, "bg_th_mean_diff: %d\n", p_param->bg_th_mean_diff);
		return -EINVAL;
	}
	if (p_param->bg_delta_qp < -16 || p_param->bg_delta_qp > 15) {
		dev_err(dev, "bg_delta_qp: %d\n", p_param->bg_delta_qp);
		return -EINVAL;
	}
	if (p_param->me_center > 1) {
		dev_err(dev, "me_center: %d\n", p_param->me_center);
		return -EINVAL;
	}
	if (p_param->en_dbk != 1 && p_param->en_dbk != 0) {
		dev_err(dev, "en_dbk: %d\n", p_param->en_dbk);
		return -EINVAL;
	}
	if (p_param->en_scaling_list != 1 && p_param->en_scaling_list != 0) {
		dev_err(dev, "en_scaling_list: %d\n", p_param->en_scaling_list);
		return -EINVAL;
	}
	if (p_param->en_adaptive_round != 1 && p_param->en_adaptive_round != 0) {
		dev_err(dev, "en_adaptive_round: %d\n", p_param->en_adaptive_round);
		return -EINVAL;
	}
	if (p_param->q_round_intra > 255) {
		dev_err(dev, "q_round_intra: %d\n", p_param->q_round_intra);
		return -EINVAL;
	}
	if (p_param->q_round_inter > 255) {
		dev_err(dev, "q_round_inter: %d\n", p_param->q_round_inter);
		return -EINVAL;
	}
	if (p_param->dis_coef_clear != 1 && p_param->dis_coef_clear != 0) {
		dev_err(dev, "dis_coef_clear: %d\n", p_param->dis_coef_clear);
		return -EINVAL;
	}
	if (p_param->lambda_dqp_intra < -32 || p_param->lambda_dqp_intra > 31) {
		dev_err(dev, "lambda_dqp_intra: %d\n", p_param->lambda_dqp_intra);
		return -EINVAL;
	}
	if (p_param->lambda_dqp_inter < -32 || p_param->lambda_dqp_inter > 31) {
		dev_err(dev, "lambda_dqp_inter: %d\n", p_param->lambda_dqp_inter);
		return -EINVAL;
	}
	if (p_param->en_q_round_offset != 1 && p_param->en_q_round_offset != 0) {
		dev_err(dev, "en_q_round_offset: %d\n", p_param->en_q_round_offset);
		return -EINVAL;
	}
	if (p_param->forced_idr_header > 2) {
		dev_err(dev, "forced_idr_header: %d\n", p_param->forced_idr_header);
		return -EINVAL;
	}
	if (p_param->num_units_in_tick > INT_MAX) {
		dev_err(dev, "num_units_in_tick: %d\n", p_param->num_units_in_tick);
		return -EINVAL;
	}
	if (p_param->time_scale > INT_MAX) {
		dev_err(dev, "time_scale: %d\n", p_param->time_scale);
		return -EINVAL;
	}
	if (p_param->max_intra_pic_bit > 1500000000) {
		dev_err(dev, "max_intra_pic_bit: %d\n", p_param->max_intra_pic_bit);
		return -EINVAL;
	}
	if (p_param->max_inter_pic_bit > 1500000000) {
		dev_err(dev, "max_inter_pic_bit: %d\n", p_param->max_inter_pic_bit);
		return -EINVAL;
	}

	if (vpu_inst->std == W_HEVC_ENC) {
		if (p_param->internal_bit_depth == 10 && attr->support_hevc10bit_enc == false) {
			dev_err(dev, "internal_bit_depth: %d, support_hevc10bit_enc: %d\n", p_param->internal_bit_depth, attr->support_hevc10bit_enc);
			return -EOPNOTSUPP;
		}
		if (p_param->idr_period != 0) {
			dev_err(dev, "idr_period: %d\n", p_param->idr_period);
			return -EINVAL;
		}
		if (p_param->strong_intra_smoothing != 1 && p_param->strong_intra_smoothing != 0) {
			dev_err(dev, "strong_intra_smoothing: %d\n", p_param->strong_intra_smoothing);
			return -EINVAL;
		}
		if (p_param->constrained_intra_pred != 1 && p_param->constrained_intra_pred != 0) {
			dev_err(dev, "constrained_intra_pred: %d\n", p_param->constrained_intra_pred);
			return -EINVAL;
		}
		if (p_param->intra_trans_skip != 1 && p_param->intra_trans_skip != 0) {
			dev_err(dev, "intra_trans_skip: %d\n", p_param->intra_trans_skip);
			return -EINVAL;
		}
		if (p_param->en_temporal_mvp != 1 && p_param->en_temporal_mvp != 0) {
			dev_err(dev, "en_temporal_mvp: %d\n", p_param->en_temporal_mvp);
			return -EINVAL;
		}
		if (p_param->en_cabac != 0) {
			dev_err(dev, "en_cabac: %d\n", p_param->en_cabac);
			return -EINVAL;
		}
		if (p_param->en_transform8x8 != 0) {
			dev_err(dev, "en_transform8x8: %d\n", p_param->en_transform8x8);
			return -EINVAL;
		}
		if (p_param->lf_cross_slice_boundary_flag != 1 && p_param->lf_cross_slice_boundary_flag != 0) {
			dev_err(dev, "lf_cross_slice_boundary_flag: %d\n", p_param->lf_cross_slice_boundary_flag);
			return -EINVAL;
		}
		if (p_param->beta_offset_div2 < -6 || p_param->beta_offset_div2 > 6) {
			dev_err(dev, "beta_offset_div2: %d\n", p_param->beta_offset_div2);
			return -EINVAL;
		}
		if (p_param->tc_offset_div2 < -6 || p_param->tc_offset_div2 > 6) {
			dev_err(dev, "tc_offset_div2: %d\n", p_param->tc_offset_div2);
			return -EINVAL;
		}
		if (p_param->lf_sharpness != 0) {
			dev_err(dev, "lf_sharpness: %d\n", p_param->lf_sharpness);
			return -EINVAL;
		}
		if (p_param->en_sao != 1 && p_param->en_sao != 0) {
			dev_err(dev, "en_sao: %d\n", p_param->en_sao);
			return -EINVAL;
		}
		if (p_param->en_cdef != 0) {
			dev_err(dev, "en_cdef: %d\n", p_param->en_cdef);
			return -EINVAL;
		}
		if (p_param->en_wiener != 0) {
			dev_err(dev, "en_wiener: %d\n", p_param->en_wiener);
			return -EINVAL;
		}
		if (p_param->y_dc_qp_delta != 0) {
			dev_err(dev, "y_dc_qp_delta: %d\n", p_param->y_dc_qp_delta);
			return -EINVAL;
		}
		if (p_param->u_dc_qp_delta != 0) {
			dev_err(dev, "u_dc_qp_delta: %d\n", p_param->u_dc_qp_delta);
			return -EINVAL;
		}
		if (p_param->v_dc_qp_delta != 0) {
			dev_err(dev, "v_dc_qp_delta: %d\n", p_param->v_dc_qp_delta);
			return -EINVAL;
		}
		if (p_param->u_ac_qp_delta != 0) {
			dev_err(dev, "u_ac_qp_delta: %d\n", p_param->u_ac_qp_delta);
			return -EINVAL;
		}
		if (p_param->v_ac_qp_delta != 0) {
			dev_err(dev, "v_ac_qp_delta: %d\n", p_param->v_ac_qp_delta);
			return -EINVAL;
		}
		if (p_param->cb_qp_offset < -12 || p_param->cb_qp_offset > 12) {
			dev_err(dev, "cb_qp_offset: %d\n", p_param->cb_qp_offset);
			return -EINVAL;
		}
		if (p_param->cr_qp_offset < -12 || p_param->cr_qp_offset > 12) {
			dev_err(dev, "cr_qp_offset: %d\n", p_param->cr_qp_offset);
			return -EINVAL;
		}
		if (p_param->en_still_picture != 1 && p_param->en_still_picture != 0) {
			dev_err(dev, "en_still_picture: %d\n", p_param->en_still_picture);
			return -EINVAL;
		}
		if (p_param->tier > 1) {
			dev_err(dev, "tier: %d\n", p_param->tier);
			return -EINVAL;
		}
		if (p_param->profile > HEVC_PROFILE_STILLPICTURE) {
			dev_err(dev, "profile: %d\n", p_param->profile);
			return -EINVAL;
		}
		if (p_param->internal_bit_depth == 10 && p_param->profile == HEVC_PROFILE_MAIN) {
			dev_err(dev, "internal_bit_depth: %d, profile: %d\n", p_param->internal_bit_depth, p_param->profile);
			return -EINVAL;
		}
		if (p_param->num_ticks_poc_diff_one < 1 || p_param->num_ticks_poc_diff_one > 65535) {
			dev_err(dev, "num_ticks_poc_diff_one: %d\n", p_param->num_ticks_poc_diff_one);
			return -EINVAL;
		}
		if (p_param->av1_color.chroma_sample_position != 0) {
			dev_err(dev, "chroma_sample_position: %d\n", p_param->av1_color.chroma_sample_position);
			return -EINVAL;
		}
		if (p_param->av1_color.color_range != 0) {
			dev_err(dev, "color_range: %d\n", p_param->av1_color.color_range);
			return -EINVAL;
		}
		if (p_param->av1_color.matrix_coefficients != 0) {
			dev_err(dev, "matrix_coefficients: %d\n", p_param->av1_color.matrix_coefficients);
			return -EINVAL;
		}
		if (p_param->av1_color.transfer_characteristics != 0) {
			dev_err(dev, "transfer_characteristics: %d\n", p_param->av1_color.transfer_characteristics);
			return -EINVAL;
		}
		if (p_param->av1_color.color_primaries != 0) {
			dev_err(dev, "color_primaries: %d\n", p_param->av1_color.color_primaries);
			return -EINVAL;
		}
		if (p_param->av1_color.color_description_present_flag != 0) {
			dev_err(dev, "color_description_present_flag: %d\n", p_param->av1_color.chroma_sample_position);
			return -EINVAL;
		}
	} else if (vpu_inst->std == W_AVC_ENC) {
		if (p_param->internal_bit_depth == 10 && attr->support_avc10bit_enc == false) {
			dev_err(dev, "internal_bit_depth: %d, support_avc10bit_enc: %d\n", p_param->internal_bit_depth, attr->support_avc10bit_enc);
			return -EOPNOTSUPP;
		}
		if (p_param->idr_period > 2047) {
			dev_err(dev, "idr_period: %d\n", p_param->idr_period);
			return -EINVAL;
		}
		if (p_param->idr_period == 1 && p_param->gop_preset_idx == PRESET_IDX_ALL_I) {
			dev_err(dev, "idr_period: %d, gop_preset_idx: %d\n", p_param->idr_period, p_param->gop_preset_idx);
			return -EINVAL;
		}
		if (p_param->strong_intra_smoothing != 0) {
			dev_err(dev, "strong_intra_smoothing: %d\n", p_param->strong_intra_smoothing);
			return -EINVAL;
		}
		if (p_param->constrained_intra_pred != 1 && p_param->constrained_intra_pred != 0) {
			dev_err(dev, "constrained_intra_pred: %d\n", p_param->constrained_intra_pred);
			return -EINVAL;
		}
		if (p_param->intra_trans_skip != 0) {
			dev_err(dev, "intra_trans_skip: %d\n", p_param->intra_trans_skip);
			return -EINVAL;
		}
		if (p_param->en_temporal_mvp != 0) {
			dev_err(dev, "en_temporal_mvp: %d\n", p_param->en_temporal_mvp);
			return -EINVAL;
		}
		if (p_param->en_cabac != 1 && p_param->en_cabac != 0) {
			dev_err(dev, "en_cabac: %d\n", p_param->en_cabac);
			return -EINVAL;
		}
		if (p_param->en_transform8x8 != 1 && p_param->en_transform8x8 != 0) {
			dev_err(dev, "en_transform8x8: %d\n", p_param->en_transform8x8);
			return -EINVAL;
		}
		if (p_param->lf_cross_slice_boundary_flag != 1 && p_param->lf_cross_slice_boundary_flag != 0) {
			dev_err(dev, "lf_cross_slice_boundary_flag: %d\n", p_param->lf_cross_slice_boundary_flag);
			return -EINVAL;
		}
		if (p_param->beta_offset_div2 < -6 || p_param->beta_offset_div2 > 6) {
			dev_err(dev, "beta_offset_div2: %d\n", p_param->beta_offset_div2);
			return -EINVAL;
		}
		if (p_param->tc_offset_div2 < -6 || p_param->tc_offset_div2 > 6) {
			dev_err(dev, "tc_offset_div2: %d\n", p_param->tc_offset_div2);
			return -EINVAL;
		}
		if (p_param->lf_sharpness != 0) {
			dev_err(dev, "lf_sharpness: %d\n", p_param->lf_sharpness);
			return -EINVAL;
		}
		if (p_param->en_sao != 0) {
			dev_err(dev, "en_sao: %d\n", p_param->en_sao);
			return -EINVAL;
		}
		if (p_param->en_cdef != 0) {
			dev_err(dev, "en_cdef: %d\n", p_param->en_cdef);
			return -EINVAL;
		}
		if (p_param->en_wiener != 0) {
			dev_err(dev, "en_wiener: %d\n", p_param->en_wiener);
			return -EINVAL;
		}
		if (p_param->y_dc_qp_delta != 0) {
			dev_err(dev, "y_dc_qp_delta: %d\n", p_param->y_dc_qp_delta);
			return -EINVAL;
		}
		if (p_param->u_dc_qp_delta != 0) {
			dev_err(dev, "u_dc_qp_delta: %d\n", p_param->u_dc_qp_delta);
			return -EINVAL;
		}
		if (p_param->v_dc_qp_delta != 0) {
			dev_err(dev, "v_dc_qp_delta: %d\n", p_param->v_dc_qp_delta);
			return -EINVAL;
		}
		if (p_param->u_ac_qp_delta != 0) {
			dev_err(dev, "u_ac_qp_delta: %d\n", p_param->u_ac_qp_delta);
			return -EINVAL;
		}
		if (p_param->v_ac_qp_delta != 0) {
			dev_err(dev, "v_ac_qp_delta: %d\n", p_param->v_ac_qp_delta);
			return -EINVAL;
		}
		if (p_param->cb_qp_offset < -12 || p_param->cb_qp_offset > 12) {
			dev_err(dev, "cb_qp_offset: %d\n", p_param->cb_qp_offset);
			return -EINVAL;
		}
		if (p_param->cr_qp_offset < -12 || p_param->cr_qp_offset > 12) {
			dev_err(dev, "cr_qp_offset: %d\n", p_param->cr_qp_offset);
			return -EINVAL;
		}
		if (p_param->en_still_picture != 0) {
			dev_err(dev, "en_still_picture: %d\n", p_param->en_still_picture);
			return -EINVAL;
		}
		if (p_param->tier != 0) {
			dev_err(dev, "tier: %d\n", p_param->tier);
			return -EINVAL;
		}
		if (p_param->profile > H264_PROFILE_HIGH10) {
			dev_err(dev, "profile: %d\n", p_param->profile);
			return -EINVAL;
		}
		if (p_param->profile) {
			if (p_param->internal_bit_depth == 10 && p_param->profile != H264_PROFILE_HIGH10) {
				dev_err(dev, "internal_bit_depth: %d, profile: %d\n", p_param->internal_bit_depth, p_param->profile);
				return -EINVAL;
			}
		}
		if (p_param->num_ticks_poc_diff_one != 0) {
			dev_err(dev, "num_ticks_poc_diff_one: %d\n", p_param->num_ticks_poc_diff_one);
			return -EINVAL;
		}
		if (p_param->av1_color.chroma_sample_position != 0) {
			dev_err(dev, "chroma_sample_position: %d\n", p_param->av1_color.chroma_sample_position);
			return -EINVAL;
		}
		if (p_param->av1_color.color_range != 0) {
			dev_err(dev, "color_range: %d\n", p_param->av1_color.color_range);
			return -EINVAL;
		}
		if (p_param->av1_color.matrix_coefficients != 0) {
			dev_err(dev, "matrix_coefficients: %d\n", p_param->av1_color.matrix_coefficients);
			return -EINVAL;
		}
		if (p_param->av1_color.transfer_characteristics != 0) {
			dev_err(dev, "transfer_characteristics: %d\n", p_param->av1_color.transfer_characteristics);
			return -EINVAL;
		}
		if (p_param->av1_color.color_primaries != 0) {
			dev_err(dev, "color_primaries: %d\n", p_param->av1_color.color_primaries);
			return -EINVAL;
		}
		if (p_param->av1_color.color_description_present_flag != 0) {
			dev_err(dev, "color_description_present_flag: %d\n", p_param->av1_color.chroma_sample_position);
			return -EINVAL;
		}
	} else if (vpu_inst->std == W_AV1_ENC) {
		if (p_param->idr_period != 0) {
			dev_err(dev, "idr_period: %d\n", p_param->idr_period);
			return -EINVAL;
		}
		if (p_param->strong_intra_smoothing != 0) {
			dev_err(dev, "strong_intra_smoothing: %d\n", p_param->strong_intra_smoothing);
			return -EINVAL;
		}
		if (p_param->constrained_intra_pred != 0) {
			dev_err(dev, "constrained_intra_pred: %d\n", p_param->constrained_intra_pred);
			return -EINVAL;
		}
		if (p_param->intra_trans_skip != 1 && p_param->intra_trans_skip != 0) {
			dev_err(dev, "intra_trans_skip: %d\n", p_param->intra_trans_skip);
			return -EINVAL;
		}
		if (p_param->en_temporal_mvp != 0) {
			dev_err(dev, "en_temporal_mvp: %d\n", p_param->en_temporal_mvp);
			return -EINVAL;
		}
		if (p_param->en_cabac != 0) {
			dev_err(dev, "en_cabac: %d\n", p_param->en_cabac);
			return -EINVAL;
		}
		if (p_param->en_transform8x8 != 0) {
			dev_err(dev, "en_transform8x8: %d\n", p_param->en_transform8x8);
			return -EINVAL;
		}
		if (p_param->lf_cross_slice_boundary_flag != 0) {
			dev_err(dev, "lf_cross_slice_boundary_flag: %d\n", p_param->lf_cross_slice_boundary_flag);
			return -EINVAL;
		}
		if (p_param->beta_offset_div2 != 0) {
			dev_err(dev, "beta_offset_div2: %d\n", p_param->beta_offset_div2);
			return -EINVAL;
		}
		if (p_param->tc_offset_div2 != 0) {
			dev_err(dev, "tc_offset_div2: %d\n", p_param->tc_offset_div2);
			return -EINVAL;
		}
		if (p_param->lf_sharpness > 7) {
			dev_err(dev, "lf_sharpness: %d\n", p_param->lf_sharpness);
			return -EINVAL;
		}
		if (p_param->en_sao != 0) {
			dev_err(dev, "en_sao: %d\n", p_param->en_sao);
			return -EINVAL;
		}
		if (p_param->en_cdef != 1 && p_param->en_cdef != 0) {
			dev_err(dev, "en_cdef: %d\n", p_param->en_cdef);
			return -EINVAL;
		}
		if (p_param->en_wiener != 1 && p_param->en_wiener != 0) {
			dev_err(dev, "en_wiener: %d\n", p_param->en_wiener);
			return -EINVAL;
		}
		if (p_param->y_dc_qp_delta < -64 || p_param->y_dc_qp_delta > 63) {
			dev_err(dev, "y_dc_qp_delta: %d\n", p_param->y_dc_qp_delta);
			return -EINVAL;
		}
		if (p_param->u_dc_qp_delta < -64 || p_param->u_dc_qp_delta > 63) {
			dev_err(dev, "u_dc_qp_delta: %d\n", p_param->u_dc_qp_delta);
			return -EINVAL;
		}
		if (p_param->v_dc_qp_delta < -64 || p_param->v_dc_qp_delta > 63) {
			dev_err(dev, "v_dc_qp_delta: %d\n", p_param->v_dc_qp_delta);
			return -EINVAL;
		}
		if (p_param->u_ac_qp_delta < -64 || p_param->u_ac_qp_delta > 63) {
			dev_err(dev, "u_ac_qp_delta: %d\n", p_param->u_ac_qp_delta);
			return -EINVAL;
		}
		if (p_param->v_ac_qp_delta < -64 || p_param->v_ac_qp_delta > 63) {
			dev_err(dev, "v_ac_qp_delta: %d\n", p_param->v_ac_qp_delta);
			return -EINVAL;
		}
		if (p_param->cb_qp_offset != 0) {
			dev_err(dev, "cb_qp_offset: %d\n", p_param->cb_qp_offset);
			return -EINVAL;
		}
		if (p_param->cr_qp_offset != 0) {
			dev_err(dev, "cr_qp_offset: %d\n", p_param->cr_qp_offset);
			return -EINVAL;
		}
		if (p_param->en_still_picture != 0) {
			dev_err(dev, "en_still_picture: %d\n", p_param->en_still_picture);
			return -EINVAL;
		}
		if (p_param->tier != 0) {
			dev_err(dev, "tier: %d\n", p_param->tier);
			return -EINVAL;
		}
		if (p_param->profile != 0) {
			dev_err(dev, "profile: %d\n", p_param->profile);
			return -EINVAL;
		}
		if (p_param->num_ticks_poc_diff_one != 0) {
			dev_err(dev, "num_ticks_poc_diff_one: %d\n", p_param->num_ticks_poc_diff_one);
			return -EINVAL;
		}
		if (p_param->av1_color.chroma_sample_position > 3) {
			dev_err(dev, "chroma_sample_position: %d\n", p_param->av1_color.chroma_sample_position);
			return -EINVAL;
		}
		if (p_param->av1_color.color_range > 1) {
			dev_err(dev, "color_range: %d\n", p_param->av1_color.color_range);
			return -EINVAL;
		}
		if (p_param->av1_color.matrix_coefficients > 255) {
			dev_err(dev, "matrix_coefficients: %d\n", p_param->av1_color.matrix_coefficients);
			return -EINVAL;
		}
		if (p_param->av1_color.transfer_characteristics > 255) {
			dev_err(dev, "transfer_characteristics: %d\n", p_param->av1_color.transfer_characteristics);
			return -EINVAL;
		}
		if (p_param->av1_color.color_primaries > 255) {
			dev_err(dev, "color_primaries: %d\n", p_param->av1_color.color_primaries);
			return -EINVAL;
		}
		if (p_param->av1_color.color_description_present_flag > 1) {
			dev_err(dev, "color_description_present_flag: %d\n", p_param->av1_color.chroma_sample_position);
			return -EINVAL;
		}
	}

	return 0;
}

void *wave6_vpu_get_sram(struct vpu_instance *vpu_inst, dma_addr_t *dma_addr, u32 *size)
{
	return wave6_vpu_ctrl_get_sram(vpu_inst->dev->ctrl, dma_addr, size);
}

u64 wave6_cycle_to_ns(struct vpu_device *vpu_dev, u64 cycle)
{
	if (!vpu_dev || !vpu_dev->vpu_clk_rate)
		return 0;

	return (cycle * NSEC_PER_SEC) / vpu_dev->vpu_clk_rate;
}
