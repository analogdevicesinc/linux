// SPDX-License-Identifier: MIT
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 *
 */

#include "dm_services.h"
#include "clk_mgr.h"
#include "dccg.h"
#include "core_types.h"
#include "dc_dmub_srv.h"
#include "dmub_cmd.h"
#include "abm.h"
#include "audio.h"
#include "dchubbub.h"
#include "dio.h"
#include "link_encoder.h"
#include "panel_cntl.h"
#include "link_service.h"
#include "dc_bios_types.h"
#include "gpio_service_interface.h"
#include "hwss/hw_sequencer_private.h"
#include "bls_executor/bls_executor.h"
#include "dmub_messenger/dmub_messenger.h"

static noinline_for_stack bool dc2_bls_step_supported(const struct dc2 *dc2,
		const struct block_sequence *step)
{
	const struct dc *dc = dc2 ? &dc2->dc : NULL;
	struct dmub_srv *dmub;
	struct clk_mgr *clk_mgr;
	struct dccg *dccg;
	struct dc_context *ctx;

	switch (step->func) {
	case LEGACY_CLK_MGR_INIT_CLOCKS:
		clk_mgr =
				step->params.legacy_clk_mgr_init_clocks_params.clk_mgr;
		return clk_mgr && clk_mgr->funcs &&
				clk_mgr->funcs->init_clocks;
	case LEGACY_CLK_MGR_IS_DC_MODE_PRESENT:
		clk_mgr = step->params
				.legacy_clk_mgr_is_dc_mode_present_params.clk_mgr;
		return clk_mgr && clk_mgr->funcs &&
				clk_mgr->funcs->is_dc_mode_present &&
				step->params
				.legacy_clk_mgr_is_dc_mode_present_params
				.out_dcmode_power_limits_present;
	case DCCG_INITIALIZE_CLOCKS:
		dccg = step->params.dccg_initialize_clocks_params.dccg;
		return dccg && dccg->funcs && dccg->funcs->dccg_init;
	case DMUB_SEND_DMCUB_CMD:
		ctx = step->params.send_dmcub_cmd_params.ctx;
		if (!ctx || !ctx->dmub_srv || !ctx->dmub_srv->dmub ||
				!step->params.send_dmcub_cmd_params.cmd ||
				(ctx->dmub_srv->idle_allowed && !ctx->dc))
			return false;
		return true;
	case DMUB_GPINT_SEND_AND_WAIT:
	case DMUB_HALT_FW:
		if (!dc || !dc->ctx || !dc->ctx->dmub_srv ||
				!dc->ctx->dmub_srv->dmub ||
				(dc->ctx->dmub_srv->idle_allowed &&
				!dc->ctx->dc))
			return false;
		dmub = dc->ctx->dmub_srv->dmub;
		return dmub->sw_init && dmub->hw_funcs.set_gpint &&
				dmub->hw_funcs.is_gpint_acked &&
				dmub->hw_funcs.get_gpint_response &&
				step->params.dmub_gpint_params.response;
	case PSP_RELOAD_DMUB_FW: {
		const struct dmub_srv_hw_params *hw_params =
				step->params.psp_reload_dmub_fw_params.hw_params;

		return step->params.psp_reload_dmub_fw_params.dmub &&
				hw_params && hw_params->fb_info &&
				hw_params->fb_info->num_fb >= DMUB_WINDOW_TOTAL;
	}
	case DMUB_INITIALIZE_DCN:
		ctx = step->params.send_dmcub_cmd_params.ctx;
		if (!ctx || !ctx->dmub_srv || !ctx->dmub_srv->dmub ||
				!step->params.send_dmcub_cmd_params.cmd ||
				!step->params.send_dmcub_cmd_params.cmd->cmd_common
				.header.type ||
				(ctx->dmub_srv->idle_allowed && !ctx->dc))
			return false;
		return true;
	case OPTC_SET_ODM_MEM_POWER: {
		struct dce_hwseq *hws = step->params.hwseq_reg_control_params.hws;

		return hws && hws->funcs.init_odm_mem_power;
	}
	case MMHUBBUB_SET_MEM_POWER: {
		struct dce_hwseq *hws = step->params.hwseq_reg_control_params.hws;

		return hws && hws->funcs.power_down_vga_mem;
	}
	case DCCG_SET_DCFCLK_GATING: {
		struct dce_hwseq *hws = step->params.hwseq_reg_control_params.hws;

		return hws && hws->funcs.dcfclk_gating_control;
	}
	case HPO_SET_HW_CONTROL: {
		struct dce_hwseq *hws = step->params.hwseq_reg_control_params.hws;

		return hws && hws->funcs.init_hpo_hw_control;
	}
	case PG_SET_PLANE_POWER_GATING: {
		struct dce_hwseq *hws = step->params.hwseq_reg_control_params.hws;

		return hws && hws->funcs.enable_power_gating_plane;
	}
	case DCCG_GET_REF_FREQ: {
		const struct dccg_get_ref_freq_params *p =
				&step->params.dccg_get_ref_freq_params;

		return p->dccg && p->dccg->funcs &&
				p->dccg->funcs->get_dccg_ref_freq &&
				p->out_xtalin_khz && p->out_dccg_ref_khz;
	}
	case HUBBUB_GET_DCHUB_REF_FREQ: {
		const struct hubbub_get_dchub_ref_freq_params *p =
				&step->params.hubbub_get_dchub_ref_freq_params;

		return p->hubbub && p->hubbub->funcs &&
				p->hubbub->funcs->get_dchub_ref_freq &&
				p->out_dchub_ref_khz;
	}
	case DIO_LINK_ENC_HW_INIT: {
		struct link_encoder *enc =
				step->params.link_enc_hw_init_params.link_enc;

		return enc && enc->funcs && enc->funcs->hw_init;
	}
	case DIO_LINK_ENC_IS_DIG_ENABLED: {
		const struct link_enc_is_dig_enabled_params *p =
				&step->params.link_enc_is_dig_enabled_params;

		return p->link_enc && p->link_enc->funcs &&
				p->link_enc->funcs->is_dig_enabled &&
				p->out_dig_enabled;
	}
	case DIO_LINK_ENC_FEC_IS_ACTIVE: {
		const struct link_enc_fec_is_active_params *p =
				&step->params.link_enc_fec_is_active_params;

		return p->link_enc && p->link_enc->funcs &&
				p->link_enc->funcs->fec_is_active &&
				p->out_fec_active;
	}
	case BIOS_GET_I2C_INFO: {
		const struct bios_get_i2c_info_params *p =
				&step->params.bios_get_i2c_info_params;

		return p->dcb && p->dcb->funcs && p->dcb->funcs->get_i2c_info &&
				p->out_info && p->out_result;
	}
	case CLK_MGR_GET_DISPCLK_FROM_DENTIST: {
		const struct clk_mgr_get_dispclk_from_dentist_params *p =
				&step->params.clk_mgr_get_dispclk_from_dentist_params;

		return p->clk_mgr && p->clk_mgr->funcs &&
				p->clk_mgr->funcs->get_dispclk_from_dentist &&
				p->out_dispclk_khz;
	}
	case EDP_BACKLIGHT_CONTROL: {
		struct dc_link *link = step->params.edp_control_params.link;

		return link && link->dc && link->dc->hwss.edp_backlight_control;
	}
	case EDP_POWER_CONTROL: {
		struct dc_link *link = step->params.edp_control_params.link;

		return link && link->dc && link->dc->hwss.edp_power_control;
	}
	case LINK_BLANK_ALL_DP_DISPLAYS: {
		struct dc *init_dc = step->params.dc_init_call_params.dc;

		return init_dc && init_dc->link_srv &&
				init_dc->link_srv->blank_all_dp_displays;
	}
	case HWSS_ENABLE_ACCELERATED_MODE: {
		struct dc *init_dc = step->params.dc_init_call_params.dc;

		return init_dc && init_dc->hwss.enable_accelerated_mode;
	}
	case HWSEQ_INIT_PIPES: {
		struct dc *init_dc = step->params.dc_init_call_params.dc;

		return init_dc && init_dc->hwseq &&
				init_dc->hwseq->funcs.init_pipes;
	}
	case DMUB_SET_IDLE_OPTIMIZATION:
		return step->params.dc_init_call_params.dc != NULL;
	case HWSEQ_POWER_DOWN: {
		struct dc *init_dc = step->params.dc_init_call_params.dc;

		return init_dc && init_dc->hwseq &&
				init_dc->hwseq->funcs.power_down;
	}
	case HUBBUB_ALLOW_SELF_REFRESH_CONTROL: {
		struct hubbub *hubbub = step->params
				.hubbub_allow_self_refresh_control_params.hubbub;

		return hubbub && hubbub->funcs &&
				hubbub->funcs->allow_self_refresh_control;
	}
	case LEGACY_CLK_MGR_INITIALIZE_MIN_CLOCKS: {
		const struct clk_mgr_initialize_min_clocks_params *p =
				&step->params.clk_mgr_initialize_min_clocks_params;

		return p->clk_mgr && p->clk_mgr->funcs &&
				p->clk_mgr->funcs->update_clocks && p->context;
	}
	case AUDIO_HW_INIT: {
		struct audio *audio = step->params.audio_hw_init_params.audio;

		return audio && audio->funcs && audio->funcs->hw_init;
	}
	case PANEL_CNTL_HW_INIT: {
		const struct panel_cntl_hw_init_params *p =
				&step->params.panel_cntl_hw_init_params;

		return p->panel_cntl && p->panel_cntl->funcs &&
				p->panel_cntl->funcs->hw_init &&
				p->out_backlight && p->out_user_level;
	}
	case GPIO_DDC_OPEN:
		return step->params.gpio_ddc_open_params.gpio_service != NULL;
	case ABM_INIT: {
		const struct abm_init_params *p = &step->params.abm_init_params;

		return p->abm && p->abm->funcs && p->abm->funcs->abm_init;
	}
	case DIO_AFMT_SET_MEM_POWER: {
		struct dio *dio = step->params.dio_set_mem_power_params.dio;

		return dio && dio->funcs && dio->funcs->mem_pwr_ctrl;
	}
	case DCCG_SET_ROOT_CLOCK_GATING: {
		struct dccg *gating_dccg =
				step->params.dccg_set_root_clock_gating_params.dccg;

		return gating_dccg && gating_dccg->funcs &&
				gating_dccg->funcs->allow_clock_gating;
	}
	case HUBBUB_INIT_WATERMARKS: {
		const struct hubbub_init_watermarks_params *p =
				&step->params.hubbub_init_watermarks_params;

		return p->hubbub && p->hubbub->funcs &&
				p->hubbub->funcs->init_watermarks;
	}
	case BIOS_IS_ACCELERATED_MODE: {
		const struct bios_is_accelerated_mode_params *p =
				&step->params.bios_is_accelerated_mode_params;

		return p->dcb && p->dcb->funcs &&
				p->dcb->funcs->is_accelerated_mode &&
				p->out_accelerated_mode;
	}
	case LEGACY_CLK_MGR_NOTIFY_WM_RANGES:
		clk_mgr = step->params.clk_mgr_notify_wm_ranges_params.clk_mgr;
		return clk_mgr && clk_mgr->funcs &&
				clk_mgr->funcs->notify_wm_ranges;
	case HUBBUB_INIT_CRB: {
		struct hubbub *hubbub =
				step->params.hubbub_init_crb_params.hubbub;

		return hubbub && hubbub->funcs && hubbub->funcs->init_crb;
	}
	case HUBBUB_SET_REQUEST_LIMIT: {
		struct hubbub *hubbub =
				step->params.hubbub_set_request_limit_params.hubbub;

		return hubbub && hubbub->funcs &&
				hubbub->funcs->set_request_limit;
	}
	case HUBBUB_FORCE_PSTATE_CHANGE_CONTROL: {
		struct hubbub *hubbub = step->params
				.hubbub_force_pstate_change_control_params.hubbub;

		return hubbub && hubbub->funcs &&
				hubbub->funcs->force_pstate_change_control;
	}
	default:
		return false;
	}
}

static noinline_for_stack void dc2_bls_exec_initialize_min_clocks(
		const struct clk_mgr_initialize_min_clocks_params *p)
{
	struct dc_clocks *clocks = &p->context->bw_ctx.bw.dcn.clk;

	clocks->dcfclk_deep_sleep_khz = p->dcfclk_deep_sleep_khz;
	clocks->dcfclk_khz = p->dcfclk_khz;
	clocks->socclk_khz = p->socclk_khz;
	clocks->dramclk_khz = p->dramclk_khz;
	clocks->dppclk_khz = p->dppclk_khz;
	clocks->dispclk_khz = p->dispclk_khz;
	clocks->ref_dtbclk_khz = p->ref_dtbclk_khz;
	clocks->fclk_p_state_change_support = p->fclk_p_state_change_support;
	clocks->p_state_change_support = p->p_state_change_support;

	p->clk_mgr->funcs->update_clocks(p->clk_mgr, p->context, true);
}

static noinline_for_stack enum dc2_bls_exec_result dc2_bls_exec_dmub_send(
		const struct block_sequence *entry)
{
	struct dmub_messenger messenger;
	struct dmub_messenger_init_data init_data;
	struct dmub_send_cmd_params params;
	struct dc_context *ctx;

	ctx = entry->params.send_dmcub_cmd_params.ctx;
	init_data.ctx = ctx;
	init_data.dc_dmub_srv = ctx->dmub_srv;
	params.cmd = entry->params.send_dmcub_cmd_params.cmd;
	params.wait_type = entry->params.send_dmcub_cmd_params.wait_type;

	dmub_messenger_construct(&messenger, &init_data);
	if (messenger.funcs->inbox_cmd_send_and_wait(&messenger, &params) !=
			DMUB_MSG_OK)
		return entry->params.send_dmcub_cmd_params.continue_on_error ?
				DC2_BLS_EXEC_CONTINUED_ON_ERROR :
				DC2_BLS_EXEC_ERROR;

	return DC2_BLS_EXEC_OK;
}

static noinline_for_stack enum dc2_bls_exec_result dc2_bls_exec_dmub_gpint(
		struct dc2 *dc2, const struct block_sequence *entry)
{
	struct dmub_messenger messenger;
	struct dmub_messenger_init_data init_data;

	init_data.ctx = dc2->dc.ctx;
	init_data.dc_dmub_srv = dc2->dc.ctx->dmub_srv;
	dmub_messenger_construct(&messenger, &init_data);
	if (messenger.funcs->gpint_send_and_wait(&messenger,
			&entry->params.dmub_gpint_params) != DMUB_MSG_OK)
		return DC2_BLS_EXEC_ERROR;

	return DC2_BLS_EXEC_OK;
}

enum dc2_bls_exec_result dc2_bls_execute(
		struct dc2 *dc2,
		const struct block_sequence *steps,
		unsigned int num_steps)
{
	enum dc2_bls_exec_result result = DC2_BLS_EXEC_OK;
	const struct block_sequence *entry;
	enum dc2_bls_exec_result sent;
	unsigned int i;

	if (!steps || num_steps > MAX_HWSS_BLOCK_SEQUENCE_SIZE)
		return DC2_BLS_EXEC_ERROR;

	for (i = 0; i < num_steps; i++)
		if (!dc2_bls_step_supported(dc2, &steps[i]))
			return DC2_BLS_EXEC_ERROR;

	for (i = 0; i < num_steps; i++) {
		entry = &steps[i];
		switch (entry->func) {
		/*
		 * LEGACY/pmfw-msg: remove both cases when PMFW Messenger can
		 * reproduce the same calls, order, and output.
		 */
		case LEGACY_CLK_MGR_INIT_CLOCKS:
			entry->params.legacy_clk_mgr_init_clocks_params
					.clk_mgr->funcs->init_clocks(
					entry->params
					.legacy_clk_mgr_init_clocks_params.clk_mgr);
			break;
		case LEGACY_CLK_MGR_IS_DC_MODE_PRESENT: {
			const struct legacy_clk_mgr_is_dc_mode_present_params *p =
					&entry->params
					.legacy_clk_mgr_is_dc_mode_present_params;

			*p->out_dcmode_power_limits_present =
					p->clk_mgr->funcs->is_dc_mode_present(
					p->clk_mgr);
			break;
		}
		case DCCG_INITIALIZE_CLOCKS:
			entry->params.dccg_initialize_clocks_params.dccg->funcs->dccg_init(
					entry->params.dccg_initialize_clocks_params.dccg);
			break;
		case DMUB_SEND_DMCUB_CMD:
			sent = dc2_bls_exec_dmub_send(entry);
			if (sent == DC2_BLS_EXEC_ERROR)
				return DC2_BLS_EXEC_ERROR;
			if (sent == DC2_BLS_EXEC_CONTINUED_ON_ERROR)
				result = DC2_BLS_EXEC_CONTINUED_ON_ERROR;
			break;
		case DMUB_GPINT_SEND_AND_WAIT:
		case DMUB_HALT_FW:
			if (dc2_bls_exec_dmub_gpint(dc2, entry) != DC2_BLS_EXEC_OK)
				return DC2_BLS_EXEC_ERROR;
			break;
		case PSP_RELOAD_DMUB_FW:
			if (dmub_srv_hw_init(
					entry->params.psp_reload_dmub_fw_params.dmub,
					entry->params.psp_reload_dmub_fw_params.hw_params) !=
					DMUB_STATUS_OK)
				return DC2_BLS_EXEC_ERROR;
			break;
		case DMUB_INITIALIZE_DCN:
			if (dc2_bls_exec_dmub_send(entry) != DC2_BLS_EXEC_OK)
				return DC2_BLS_EXEC_ERROR;
			break;
		/*
		 * S-INT/bld-seq: dce_hwseq init functions. Remove when the
		 * OPTC, MMHUBBUB, DCCG and HPO BLCs expose them.
		 */
		case OPTC_SET_ODM_MEM_POWER: {
			struct dce_hwseq *hws =
					entry->params.hwseq_reg_control_params.hws;

			hws->funcs.init_odm_mem_power(hws);
			break;
		}
		case MMHUBBUB_SET_MEM_POWER: {
			struct dce_hwseq *hws =
					entry->params.hwseq_reg_control_params.hws;

			hws->funcs.power_down_vga_mem(hws);
			break;
		}
		case DCCG_SET_DCFCLK_GATING: {
			struct dce_hwseq *hws =
					entry->params.hwseq_reg_control_params.hws;

			hws->funcs.dcfclk_gating_control(hws,
					entry->params.hwseq_reg_control_params
					.enable);
			break;
		}
		case HPO_SET_HW_CONTROL: {
			struct dce_hwseq *hws =
					entry->params.hwseq_reg_control_params.hws;

			hws->funcs.init_hpo_hw_control(hws,
					entry->params.hwseq_reg_control_params
					.enable);
			break;
		}
		case PG_SET_PLANE_POWER_GATING: {
			struct dce_hwseq *hws =
					entry->params.hwseq_reg_control_params.hws;

			hws->funcs.enable_power_gating_plane(hws,
					entry->params.hwseq_reg_control_params
					.enable);
			break;
		}
		case DCCG_GET_REF_FREQ: {
			const struct dccg_get_ref_freq_params *p =
					&entry->params.dccg_get_ref_freq_params;

			*p->out_xtalin_khz = p->xtalin_khz;
			p->dccg->funcs->get_dccg_ref_freq(p->dccg, p->xtalin_khz,
					p->out_dccg_ref_khz);
			break;
		}
		case HUBBUB_GET_DCHUB_REF_FREQ: {
			const struct hubbub_get_dchub_ref_freq_params *p =
					&entry->params.hubbub_get_dchub_ref_freq_params;

			p->hubbub->funcs->get_dchub_ref_freq(p->hubbub,
					p->dccg_ref_khz, p->out_dchub_ref_khz);
			break;
		}
		case DIO_LINK_ENC_HW_INIT: {
			struct link_encoder *enc =
					entry->params.link_enc_hw_init_params.link_enc;

			enc->funcs->hw_init(enc);
			break;
		}
		case DIO_LINK_ENC_IS_DIG_ENABLED: {
			const struct link_enc_is_dig_enabled_params *p =
					&entry->params.link_enc_is_dig_enabled_params;

			*p->out_dig_enabled =
					p->link_enc->funcs->is_dig_enabled(p->link_enc);
			break;
		}
		case DIO_LINK_ENC_FEC_IS_ACTIVE: {
			const struct link_enc_fec_is_active_params *p =
					&entry->params.link_enc_fec_is_active_params;

			*p->out_fec_active =
					p->link_enc->funcs->fec_is_active(p->link_enc);
			break;
		}
		/*
		 * S-INT/bld-seq: VBIOS table read through dc_bios. Remove
		 * when a BLC exposes the DDC GPIO info.
		 */
		case BIOS_GET_I2C_INFO: {
			const struct bios_get_i2c_info_params *p =
					&entry->params.bios_get_i2c_info_params;

			*p->out_result = p->dcb->funcs->get_i2c_info(p->dcb,
					p->link_id, p->out_info);
			break;
		}
		/*
		 * S-INT/link-seq: composite link call. Remove when Link
		 * builds the blanking through the shared Builder/Executor.
		 */
		case LINK_BLANK_ALL_DP_DISPLAYS: {
			struct dc *init_dc = entry->params.dc_init_call_params.dc;

			init_dc->link_srv->blank_all_dp_displays(init_dc);
			break;
		}
		/*
		 * S-INT/bld-seq: composite hwseq calls. Remove each when the
		 * DCN401 BLCs it drives are contract-conformant.
		 */
		case HWSS_ENABLE_ACCELERATED_MODE: {
			struct dc *init_dc = entry->params.dc_init_call_params.dc;

			init_dc->hwss.enable_accelerated_mode(init_dc,
					init_dc->current_state);
			break;
		}
		case HWSEQ_INIT_PIPES: {
			struct dc *init_dc = entry->params.dc_init_call_params.dc;

			init_dc->hwseq->funcs.init_pipes(init_dc,
					init_dc->current_state);
			break;
		}
		case HWSEQ_POWER_DOWN: {
			struct dc *init_dc = entry->params.dc_init_call_params.dc;

			init_dc->hwseq->funcs.power_down(init_dc);
			break;
		}
		case EDP_BACKLIGHT_CONTROL: {
			struct dc_link *link = entry->params.edp_control_params.link;

			link->dc->hwss.edp_backlight_control(link,
					entry->params.edp_control_params.enable);
			break;
		}
		case EDP_POWER_CONTROL: {
			struct dc_link *link = entry->params.edp_control_params.link;

			link->dc->hwss.edp_power_control(link,
					entry->params.edp_control_params.enable);
			break;
		}
		case HUBBUB_ALLOW_SELF_REFRESH_CONTROL: {
			const struct hubbub_allow_self_refresh_control_params *p =
					&entry->params
					.hubbub_allow_self_refresh_control_params;

			p->hubbub->funcs->allow_self_refresh_control(p->hubbub,
					p->allow);
			break;
		}
		/*
		 * S-INT/pmfw-msg: remove these cases when PMFW Messenger can
		 * reproduce the same calls, order, and output.
		 */
		case CLK_MGR_GET_DISPCLK_FROM_DENTIST: {
			const struct clk_mgr_get_dispclk_from_dentist_params *p =
					&entry->params
					.clk_mgr_get_dispclk_from_dentist_params;

			*p->out_dispclk_khz =
					p->clk_mgr->funcs->get_dispclk_from_dentist(
					p->clk_mgr);
			break;
		}
		case LEGACY_CLK_MGR_INITIALIZE_MIN_CLOCKS:
			dc2_bls_exec_initialize_min_clocks(
					&entry->params
					.clk_mgr_initialize_min_clocks_params);
			break;
		case LEGACY_CLK_MGR_NOTIFY_WM_RANGES: {
			struct clk_mgr *wm_clk_mgr = entry->params
					.clk_mgr_notify_wm_ranges_params.clk_mgr;

			wm_clk_mgr->funcs->notify_wm_ranges(wm_clk_mgr);
			break;
		}
		/*
		 * S-INT/dmub-msg: composite idle call. Remove when DMUB
		 * Messenger carries the idle commands.
		 */
		case DMUB_SET_IDLE_OPTIMIZATION:
			dc_allow_idle_optimizations(
					entry->params.dc_init_call_params.dc,
					entry->params.dc_init_call_params.enable);
			break;
		case AUDIO_HW_INIT: {
			struct audio *audio =
					entry->params.audio_hw_init_params.audio;

			audio->funcs->hw_init(audio);
			break;
		}
		case PANEL_CNTL_HW_INIT: {
			const struct panel_cntl_hw_init_params *p =
					&entry->params.panel_cntl_hw_init_params;

			*p->out_backlight =
					p->panel_cntl->funcs->hw_init(p->panel_cntl);
			*p->out_user_level = p->panel_cntl
					->stored_backlight_registers.USER_LEVEL;
			break;
		}
		case GPIO_DDC_OPEN: {
			const struct gpio_ddc_open_params *p =
					&entry->params.gpio_ddc_open_params;

			dal_gpio_ddc_open_aux_once(p->gpio_service,
					p->clk_a_register_index, p->clk_a_mask,
					&p->hw_info);
			break;
		}
		case ABM_INIT: {
			const struct abm_init_params *p =
					&entry->params.abm_init_params;

			p->abm->funcs->abm_init(p->abm, p->backlight,
					p->user_level);
			break;
		}
		case DIO_AFMT_SET_MEM_POWER: {
			const struct dio_set_mem_power_params *p =
					&entry->params.dio_set_mem_power_params;

			p->dio->funcs->mem_pwr_ctrl(p->dio, p->enable);
			break;
		}
		case DCCG_SET_ROOT_CLOCK_GATING: {
			const struct dccg_set_root_clock_gating_params *p =
					&entry->params.dccg_set_root_clock_gating_params;

			p->dccg->funcs->allow_clock_gating(p->dccg, p->allow);
			break;
		}
		case HUBBUB_INIT_WATERMARKS: {
			struct hubbub *hubbub = entry->params
					.hubbub_init_watermarks_params.hubbub;

			hubbub->funcs->init_watermarks(hubbub);
			break;
		}
		/*
		 * S-INT/bld-seq: VBIOS scratch read through dc_bios. Remove
		 * when a BLC exposes the accelerated-mode bit.
		 */
		case BIOS_IS_ACCELERATED_MODE: {
			const struct bios_is_accelerated_mode_params *p =
					&entry->params.bios_is_accelerated_mode_params;

			*p->out_accelerated_mode =
					p->dcb->funcs->is_accelerated_mode(p->dcb);
			break;
		}
		case HUBBUB_FORCE_PSTATE_CHANGE_CONTROL: {
			const struct hubbub_force_pstate_change_control_params *p =
					&entry->params
					.hubbub_force_pstate_change_control_params;

			p->hubbub->funcs->force_pstate_change_control(p->hubbub,
					p->enable, p->allow);
			break;
		}
		case HUBBUB_INIT_CRB: {
			struct hubbub *hubbub =
					entry->params.hubbub_init_crb_params.hubbub;

			hubbub->funcs->init_crb(hubbub);
			break;
		}
		case HUBBUB_SET_REQUEST_LIMIT: {
			const struct hubbub_set_request_limit_params *p =
					&entry->params.hubbub_set_request_limit_params;

			p->hubbub->funcs->set_request_limit(p->hubbub,
					p->num_chans, p->words_per_umc);
			break;
		}
		default:
			ASSERT(false);
			return DC2_BLS_EXEC_ERROR;
		}
	}

	return result;
}
