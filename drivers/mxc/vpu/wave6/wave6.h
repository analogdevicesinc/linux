/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave6 series multi-standard codec IP - wave6 backend definitions
 *
 * Copyright (C) 2021 CHIPS&MEDIA INC
 */

#ifndef __WAVE6_FUNCTION_H__
#define __WAVE6_FUNCTION_H__

#define BSOPTION_ENABLE_EXPLICIT_END        BIT(0)

#define WTL_RIGHT_JUSTIFIED          0
#define WTL_LEFT_JUSTIFIED           1
#define WTL_PIXEL_8BIT               0
#define WTL_PIXEL_16BIT              1
#define WTL_PIXEL_32BIT              2

bool wave6_vpu_is_init(struct vpu_device *vpu_dev);

void wave6_vpu_check_state(struct vpu_device *vpu_dev);

int32_t wave_vpu_get_product_id(struct vpu_device *vpu_dev);

int wave6_vpu_get_version(struct vpu_device *vpu_dev, u32 *version_info, uint32_t *revision);

void wave6_enable_interrupt(struct vpu_device *vpu_dev);

int wave6_vpu_build_up_dec_param(struct vpu_instance *inst, struct dec_open_param *param);

int wave6_vpu_dec_set_bitstream_flag(struct vpu_instance *inst, bool eos);

int wave6_vpu_dec_register_frame_buffer(struct vpu_instance *inst,
					struct frame_buffer *fb_arr, enum tiled_map_type map_type,
					uint32_t count);

int wave6_vpu_dec_register_display_buffer(struct vpu_instance *inst, struct frame_buffer fb);

int wave6_vpu_dec_update_fb(struct vpu_instance *inst, struct frame_buffer *fb, int mv_index);

int wave6_vpu_dec_get_update_fb_info(struct vpu_instance *inst,
				     struct dec_update_fb_info *info);

int wave6_vpu_dec_init_seq(struct vpu_instance *vpu_inst);

int wave6_vpu_dec_get_seq_info(struct vpu_instance *vpu_inst, struct dec_initial_info *info);

int wave6_vpu_decode(struct vpu_instance *vpu_inst, struct dec_param *option, u32 *fail_res);

int wave6_vpu_dec_get_result(struct vpu_instance *inst, struct dec_output_info *result);

int wave6_vpu_dec_fini_seq(struct vpu_instance *vpu_inst, u32 *fail_res);

dma_addr_t wave6_vpu_dec_get_rd_ptr(struct vpu_instance *vpu_inst);

int wave6_vpu_dec_flush(struct vpu_instance *inst);

/***< WAVE6 encoder >******/

int wave6_vpu_build_up_enc_param(struct device *dev, struct vpu_instance *inst,
				 struct enc_open_param *param);

int wave6_vpu_enc_init_seq(struct vpu_instance *vpu_inst);

int wave6_vpu_enc_change_seq(struct vpu_instance *vpu_inst);

int wave6_vpu_enc_get_seq_info(struct vpu_instance *vpu_inst, struct enc_initial_info *info);

int wave6_vpu_enc_register_frame_buffer(struct vpu_instance *vpu_inst,
					struct frame_buffer *fb_arr);

int wave6_vpu_encode(struct vpu_instance *vpu_inst, struct enc_param *option, u32 *fail_res);

int wave6_vpu_enc_get_result(struct vpu_instance *vpu_inst, struct enc_output_info *result);

int wave6_vpu_enc_fini_seq(struct vpu_instance *vpu_inst, u32 *fail_res);

int wave6_vpu_enc_check_open_param(struct vpu_instance *vpu_inst, struct enc_open_param *pop);

#endif /* __WAVE6_FUNCTION_H__ */
