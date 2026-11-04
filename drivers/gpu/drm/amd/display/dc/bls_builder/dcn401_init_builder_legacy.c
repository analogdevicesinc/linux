// SPDX-License-Identifier: MIT
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#include "dm_services.h"
#include "core_types.h"
#include "clk_mgr.h"
#include "dccg.h"
#include "resource.h"
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
#include "dce/dce_hwseq.h"
#include "hwss/hw_sequencer_private.h"
#include "dcn32/dcn32_resource.h"
#include "bls_builder/dcn401_init_builder.h"
#include "bls_builder/dcn401_init_builder_legacy.h"

/*
 * DC2_DC1_Wrapped: dcn401_init_hw as BLS entries, built one stage per
 * call. A stage ends on a read whose value a later stage needs: Core
 * executes the stage and retains its output in dc_state.hw_init_info,
 * where the next build reads it. Same calls in the same order, and an
 * entry is left out only where dcn401_init_hw skips the call. The build
 * reads dc and hw_init_info, and writes only the sequence and the
 * output target.
 */
static_assert(LEGACY_INIT_STAGE_POWER_DOWN_EDP1 -
		LEGACY_INIT_STAGE_POWER_DOWN_EDP0 + 1 == MAX_NUM_EDP,
		"one power-down-on-boot stage per eDP slot");

static bool dcn401_dc1_is_phy_link(const struct dc_link *link)
{
	return link->ep_type == DISPLAY_ENDPOINT_PHY;
}

static bool dcn401_dc1_powers_down_pipes(struct dc *dc,
		const struct bls_builder_legacy_hw_init_info *in)
{
	return in->s5.accelerated_mode ||
			!dc->config.seamless_boot_edp_requested;
}

static bool dcn401_dc1_uses_accelerated_mode(struct dc *dc)
{
	return dc->hwss.enable_accelerated_mode &&
			dc->debug.disable_boot_optimizations;
}

static bool dcn401_dc1_has_dmub(struct dc *dc)
{
	return dc->ctx->dmub_srv && dc->ctx->dmub_srv->dmub;
}

static enum bls_builder_result dcn401_validate_dc1_links(struct dc *dc)
{
	struct dc_bios *dcb = dc->ctx->dc_bios;
	unsigned int i;

	if (dc->link_count > MAX_LINKS)
		return BLS_BUILD_ERROR_RESOURCE_MAPPING;
	for (i = 0; i < dc->link_count; i++) {
		const struct dc_link *link = dc->links[i];

		if (!link)
			return BLS_BUILD_ERROR_RESOURCE_MAPPING;
		if (dcn401_dc1_is_phy_link(link) && (!link->link_enc ||
				!link->link_enc->funcs ||
				!link->link_enc->funcs->hw_init))
			return BLS_BUILD_ERROR_RESOURCE_MAPPING;
		if (link->panel_cntl && (!link->panel_cntl->funcs ||
				!link->panel_cntl->funcs->hw_init))
			return BLS_BUILD_ERROR_RESOURCE_MAPPING;
		if (link->force_to_use_aux && (!link->ctx ||
				!dcb->funcs->get_i2c_info))
			return BLS_BUILD_ERROR_RESOURCE_MAPPING;
	}

	return BLS_BUILD_SUCCESS;
}

static enum bls_builder_result dcn401_validate_dc1_power_down(struct dc *dc)
{
	if (!dcn401_dc1_uses_accelerated_mode(dc) &&
			!dc->hwseq->funcs.init_pipes)
		return BLS_BUILD_ERROR_RESOURCE_MAPPING;
	if (!dc->current_state || !dc->clk_mgr || !dc->clk_mgr->funcs ||
			!dc->clk_mgr->funcs->update_clocks ||
			!dc->clk_mgr->bw_params)
		return BLS_BUILD_ERROR_RESOURCE_MAPPING;

	return BLS_BUILD_SUCCESS;
}

static enum bls_builder_result dcn401_validate_dc1_hwseq(struct dc *dc)
{
	const struct hwseq_private_funcs *funcs = &dc->hwseq->funcs;

	if (dc->debug.enable_mem_low_power.bits.optc &&
			!funcs->init_odm_mem_power)
		return BLS_BUILD_ERROR_RESOURCE_MAPPING;
	if (dc->debug.enable_mem_low_power.bits.vga &&
			!funcs->power_down_vga_mem)
		return BLS_BUILD_ERROR_RESOURCE_MAPPING;
	if (!dc->debug.disable_clock_gate && !funcs->dcfclk_gating_control)
		return BLS_BUILD_ERROR_RESOURCE_MAPPING;
	if (!funcs->init_hpo_hw_control)
		return BLS_BUILD_ERROR_RESOURCE_MAPPING;

	return BLS_BUILD_SUCCESS;
}

static enum bls_builder_result dcn401_validate_dc1_initialization(
		const struct bls_builder_declarative_params *params)
{
	const struct bls_builder_legacy_hw_init_info *in;
	struct dc *dc = params->dc;
	struct resource_pool *pool;
	struct dc_bios *dcb;
	unsigned int i;

	if (!dc || !dc->ctx || !dc->res_pool || !params->current_state ||
			!params->out_targets ||
			!params->out_targets->DCN_DECL_INITIALIZATION_LEGACY)
		return BLS_BUILD_ERROR_RESOURCE_MAPPING;
	in = &params->current_state->hw_init_info;
	if (in->next_stage < LEGACY_INIT_STAGE_CLOCKS ||
			in->next_stage > LEGACY_INIT_STAGE_COUNT)
		return BLS_BUILD_ERROR_RESOURCE_MAPPING;
	if (dc->ctx->dce_environment == DCE_ENV_VIRTUAL_HW)
		return BLS_BUILD_SUCCESS;

	pool = dc->res_pool;
	dcb = dc->ctx->dc_bios;
	if (!dcb || !dcb->funcs || !dcb->funcs->is_accelerated_mode ||
			!pool->dccg || !pool->dccg->funcs || !pool->hubbub ||
			!pool->hubbub->funcs || !dc->hwseq || !dc->link_srv ||
			!dc->link_srv->blank_all_dp_displays)
		return BLS_BUILD_ERROR_RESOURCE_MAPPING;
	/* Later stages re-check only what a stage read-back decides. */
	if ((in->next_stage == LEGACY_INIT_STAGE_POWER_DOWN ||
			in->next_stage == LEGACY_INIT_STAGE_MIN_CLOCKS) &&
			dcn401_dc1_powers_down_pipes(dc, in))
		return dcn401_validate_dc1_power_down(dc);
	if (in->next_stage != LEGACY_INIT_STAGE_CLOCKS)
		return BLS_BUILD_SUCCESS;

	if (dcb->fw_info_valid && (!pool->dccg->funcs->get_dccg_ref_freq ||
			!pool->hubbub->funcs->get_dchub_ref_freq))
		return BLS_BUILD_ERROR_RESOURCE_MAPPING;
	if (dcn401_validate_dc1_links(dc) != BLS_BUILD_SUCCESS)
		return BLS_BUILD_ERROR_RESOURCE_MAPPING;
	if (dcn401_validate_dc1_hwseq(dc) != BLS_BUILD_SUCCESS)
		return BLS_BUILD_ERROR_RESOURCE_MAPPING;
	for (i = 0; i < pool->audio_count; i++)
		if (!pool->audios[i] || !pool->audios[i]->funcs ||
				!pool->audios[i]->funcs->hw_init)
			return BLS_BUILD_ERROR_RESOURCE_MAPPING;
	for (i = 0; i < pool->pipe_count; i++)
		if (pool->multiple_abms[i] && pool->multiple_abms[i]->funcs &&
				!pool->multiple_abms[i]->funcs->abm_init)
			return BLS_BUILD_ERROR_RESOURCE_MAPPING;
	if (pool->dio && !pool->dio->funcs)
		return BLS_BUILD_ERROR_RESOURCE_MAPPING;
	/* Without seamless boot the pipes power down whatever the BIOS reads. */
	if (!dc->config.seamless_boot_edp_requested)
		return dcn401_validate_dc1_power_down(dc);

	return BLS_BUILD_SUCCESS;
}

static bool dcn401_add_hwseq_reg(struct block_sequence_state *sequence,
		enum block_sequence_func func, struct dce_hwseq *hws,
		bool enable)
{
	union block_sequence_params bls_params = {0};

	bls_params.hwseq_reg_control_params.hws = hws;
	bls_params.hwseq_reg_control_params.enable = enable;
	return bls_append(sequence, func, &bls_params);
}

static bool dcn401_add_dc_call(struct block_sequence_state *sequence,
		enum block_sequence_func func, struct dc *dc, bool enable)
{
	union block_sequence_params bls_params = {0};

	bls_params.dc_init_call_params.dc = dc;
	bls_params.dc_init_call_params.enable = enable;
	return bls_append(sequence, func, &bls_params);
}

static bool dcn401_add_dccg_ref_freq(struct block_sequence_state *sequence,
		struct dc *dc, struct bls_builder_output_legacy_init_stage1 *out)
{
	union block_sequence_params bls_params = {0};

	bls_params.dccg_get_ref_freq_params.dccg = dc->res_pool->dccg;
	bls_params.dccg_get_ref_freq_params.xtalin_khz =
			dc->ctx->dc_bios->fw_info.pll_info.crystal_frequency;
	bls_params.dccg_get_ref_freq_params.out_xtalin_khz =
			&out->xtalin_clock_inKhz;
	bls_params.dccg_get_ref_freq_params.out_dccg_ref_khz =
			&out->dccg_ref_clock_inKhz;
	return bls_append(sequence, DCCG_GET_REF_FREQ, &bls_params);
}

/* Core published the DCCG reference clock after the previous stage. */
static bool dcn401_add_dchub_ref_freq(struct block_sequence_state *sequence,
		struct dc *dc, struct bls_builder_output_legacy_init_stage2 *out)
{
	union block_sequence_params bls_params = {0};
	struct resource_pool *pool = dc->res_pool;

	out->dchub_ref_clock_mhz_before =
			pool->ref_clocks.dchub_ref_clock_inKhz / 1000;
	bls_params.hubbub_get_dchub_ref_freq_params.hubbub = pool->hubbub;
	bls_params.hubbub_get_dchub_ref_freq_params.dccg_ref_khz =
			pool->ref_clocks.dccg_ref_clock_inKhz;
	bls_params.hubbub_get_dchub_ref_freq_params.out_dchub_ref_khz =
			&out->dchub_ref_clock_inKhz;
	return bls_append(sequence, HUBBUB_GET_DCHUB_REF_FREQ, &bls_params);
}

static bool dcn401_add_accelerated_mode_read(
		struct block_sequence_state *sequence, struct dc *dc,
		bool *out_accelerated_mode)
{
	union block_sequence_params bls_params = {0};

	bls_params.bios_is_accelerated_mode_params.dcb = dc->ctx->dc_bios;
	bls_params.bios_is_accelerated_mode_params.out_accelerated_mode =
			out_accelerated_mode;
	return bls_append(sequence, BIOS_IS_ACCELERATED_MODE, &bls_params);
}

static bool dcn401_add_link_enc_hw_init(
		struct block_sequence_state *sequence,
		struct link_encoder *link_enc)
{
	union block_sequence_params bls_params = {0};

	bls_params.link_enc_hw_init_params.link_enc = link_enc;
	return bls_append(sequence, DIO_LINK_ENC_HW_INIT, &bls_params);
}

/* Without is_dig_enabled, dcn401_init_hw takes the DIG as disabled. */
static bool dcn401_add_dig_read(struct block_sequence_state *sequence,
		struct link_encoder *link_enc, bool *out_dig_enabled)
{
	union block_sequence_params bls_params = {0};

	if (!link_enc || !link_enc->funcs || !link_enc->funcs->is_dig_enabled)
		return true;
	bls_params.link_enc_is_dig_enabled_params.link_enc = link_enc;
	bls_params.link_enc_is_dig_enabled_params.out_dig_enabled =
			out_dig_enabled;
	return bls_append(sequence, DIO_LINK_ENC_IS_DIG_ENABLED, &bls_params);
}

static bool dcn401_add_fec_read(struct block_sequence_state *sequence,
		struct link_encoder *link_enc, bool *out_fec_active)
{
	union block_sequence_params bls_params = {0};

	bls_params.link_enc_fec_is_active_params.link_enc = link_enc;
	bls_params.link_enc_fec_is_active_params.out_fec_active =
			out_fec_active;
	return bls_append(sequence, DIO_LINK_ENC_FEC_IS_ACTIVE, &bls_params);
}

static bool dcn401_add_i2c_info_read(struct block_sequence_state *sequence,
		struct dc *dc, const struct dc_link *link,
		struct graphics_object_i2c_info *out_info,
		enum bp_result *out_result)
{
	union block_sequence_params bls_params = {0};

	bls_params.bios_get_i2c_info_params.dcb = dc->ctx->dc_bios;
	bls_params.bios_get_i2c_info_params.link_id = link->link_id;
	bls_params.bios_get_i2c_info_params.out_info = out_info;
	bls_params.bios_get_i2c_info_params.out_result = out_result;
	return bls_append(sequence, BIOS_GET_I2C_INFO, &bls_params);
}

static bool dcn401_add_dentist_read(struct block_sequence_state *sequence,
		struct clk_mgr *clk_mgr, int *out_dispclk_khz)
{
	union block_sequence_params bls_params = {0};

	bls_params.clk_mgr_get_dispclk_from_dentist_params.clk_mgr = clk_mgr;
	bls_params.clk_mgr_get_dispclk_from_dentist_params.out_dispclk_khz =
			out_dispclk_khz;
	return bls_append(sequence, CLK_MGR_GET_DISPCLK_FROM_DENTIST,
			&bls_params);
}

static bool dcn401_add_edp_control(struct block_sequence_state *sequence,
		enum block_sequence_func func, struct dc_link *link,
		bool enable)
{
	union block_sequence_params bls_params = {0};

	bls_params.edp_control_params.link = link;
	bls_params.edp_control_params.enable = enable;
	return bls_append(sequence, func, &bls_params);
}

static bool dcn401_add_self_refresh_control(
		struct block_sequence_state *sequence, struct dc *dc)
{
	union block_sequence_params bls_params = {0};

	bls_params.hubbub_allow_self_refresh_control_params.hubbub =
			dc->res_pool->hubbub;
	bls_params.hubbub_allow_self_refresh_control_params.allow =
			!dc->debug.disable_stutter;
	return bls_append(sequence, HUBBUB_ALLOW_SELF_REFRESH_CONTROL,
			&bls_params);
}

/* The clocks dcn401_initialize_min_clocks requests, from software state. */
static bool dcn401_add_min_clocks(struct block_sequence_state *sequence,
		struct dc *dc, const struct bls_builder_legacy_hw_init_info *in)
{
	union block_sequence_params bls_params = {0};
	struct clk_mgr_initialize_min_clocks_params *p =
			&bls_params.clk_mgr_initialize_min_clocks_params;
	struct clk_mgr *clk_mgr = dc->clk_mgr;
	const struct clk_limit_table_entry *entry =
			&clk_mgr->bw_params->clk_table.entries[0];

	p->clk_mgr = clk_mgr;
	p->context = dc->current_state;
	p->dcfclk_deep_sleep_khz = DCN3_2_DCFCLK_DS_INIT_KHZ;
	p->dcfclk_khz = entry->dcfclk_mhz * 1000;
	p->socclk_khz = entry->socclk_mhz * 1000;
	p->dramclk_khz = entry->memclk_mhz * 1000;
	p->dppclk_khz = entry->dppclk_mhz * 1000;
	if (dc->debug.disable_boot_optimizations)
		p->dispclk_khz = entry->dispclk_mhz * 1000;
	else if (clk_mgr->funcs->get_dispclk_from_dentist)
		/* Keep the current DISPCLK so a lit display keeps its timing. */
		p->dispclk_khz = in->s6.dispclk_from_dentist_khz;
	else
		p->dispclk_khz = clk_mgr->boot_snapshot.dispclk;
	p->ref_dtbclk_khz = entry->dtbclk_mhz * 1000;
	p->fclk_p_state_change_support = true;
	p->p_state_change_support = true;
	return bls_append(sequence, LEGACY_CLK_MGR_INITIALIZE_MIN_CLOCKS,
			&bls_params);
}

static bool dcn401_add_audio_hw_init(struct block_sequence_state *sequence,
		struct audio *audio)
{
	union block_sequence_params bls_params = {0};

	bls_params.audio_hw_init_params.audio = audio;
	return bls_append(sequence, AUDIO_HW_INIT, &bls_params);
}

static bool dcn401_add_panel_cntl_hw_init(
		struct block_sequence_state *sequence,
		struct panel_cntl *panel_cntl, uint32_t *out_backlight,
		uint32_t *out_user_level)
{
	union block_sequence_params bls_params = {0};

	bls_params.panel_cntl_hw_init_params.panel_cntl = panel_cntl;
	bls_params.panel_cntl_hw_init_params.out_backlight = out_backlight;
	bls_params.panel_cntl_hw_init_params.out_user_level = out_user_level;
	return bls_append(sequence, PANEL_CNTL_HW_INIT, &bls_params);
}

static bool dcn401_add_ddc_open(struct block_sequence_state *sequence,
		const struct dc_link *link,
		const struct graphics_object_i2c_info *i2c_info)
{
	union block_sequence_params bls_params = {0};
	struct gpio_ddc_open_params *p = &bls_params.gpio_ddc_open_params;

	p->gpio_service = link->ctx->gpio_service;
	p->clk_a_register_index = i2c_info->gpio_info.clk_a_register_index;
	p->clk_a_mask = 1 << i2c_info->gpio_info.clk_a_shift;
	p->hw_info.ddc_channel = i2c_info->i2c_line;
	p->hw_info.hw_supported = i2c_info->i2c_hw_assist;
	return bls_append(sequence, GPIO_DDC_OPEN, &bls_params);
}

/* dc_dmub_srv_ihc_set_dig_hdcp_interrupt_dest(dmub_srv, eng_id, true) */
static bool dcn401_add_hdcp_interrupt_dest(
		struct block_sequence_state *sequence, struct dc *dc,
		const struct dc_link *link, union dmub_rb_cmd *cmd)
{
	memset(cmd, 0, sizeof(*cmd));
	cmd->ihc.header.type = DMUB_CMD__IHC;
	cmd->ihc.header.sub_type = DMUB_CMD__IHC_SET_DIG_HDCP_INTERRUPT_DEST;
	cmd->ihc.header.payload_bytes = sizeof(cmd->ihc.data);
	cmd->ihc.data.dig_id = (uint8_t)link->eng_id;
	cmd->ihc.data.to_dmu = 1;
	return dcn401_add_dmub_cmd(sequence, dc->ctx, cmd,
			DM_DMUB_WAIT_TYPE_WAIT, true);
}

static bool dcn401_add_abm_init(struct block_sequence_state *sequence,
		struct abm *abm, const struct bls_builder_legacy_hw_init_info *in)
{
	union block_sequence_params bls_params = {0};

	bls_params.abm_init_params.abm = abm;
	bls_params.abm_init_params.backlight = in->s9.backlight;
	bls_params.abm_init_params.user_level = in->s9.user_level;
	return bls_append(sequence, ABM_INIT, &bls_params);
}

static bool dcn401_add_dio_mem_power(struct block_sequence_state *sequence,
		struct dio *dio)
{
	union block_sequence_params bls_params = {0};

	bls_params.dio_set_mem_power_params.dio = dio;
	bls_params.dio_set_mem_power_params.enable = false;
	return bls_append(sequence, DIO_AFMT_SET_MEM_POWER, &bls_params);
}

static bool dcn401_add_root_clock_gating(
		struct block_sequence_state *sequence, struct dccg *dccg)
{
	union block_sequence_params bls_params = {0};

	bls_params.dccg_set_root_clock_gating_params.dccg = dccg;
	bls_params.dccg_set_root_clock_gating_params.allow = true;
	return bls_append(sequence, DCCG_SET_ROOT_CLOCK_GATING, &bls_params);
}

static bool dcn401_add_init_watermarks(struct block_sequence_state *sequence,
		struct hubbub *hubbub)
{
	union block_sequence_params bls_params = {0};

	bls_params.hubbub_init_watermarks_params.hubbub = hubbub;
	return bls_append(sequence, HUBBUB_INIT_WATERMARKS, &bls_params);
}

static bool dcn401_add_notify_wm_ranges(
		struct block_sequence_state *sequence, struct clk_mgr *clk_mgr)
{
	union block_sequence_params bls_params = {0};

	bls_params.clk_mgr_notify_wm_ranges_params.clk_mgr = clk_mgr;
	return bls_append(sequence, LEGACY_CLK_MGR_NOTIFY_WM_RANGES,
			&bls_params);
}

static bool dcn401_add_force_pstate(struct block_sequence_state *sequence,
		struct hubbub *hubbub)
{
	union block_sequence_params bls_params = {0};

	bls_params.hubbub_force_pstate_change_control_params.hubbub = hubbub;
	bls_params.hubbub_force_pstate_change_control_params.enable = false;
	bls_params.hubbub_force_pstate_change_control_params.allow = false;
	return bls_append(sequence, HUBBUB_FORCE_PSTATE_CHANGE_CONTROL,
			&bls_params);
}

static bool dcn401_add_init_crb(struct block_sequence_state *sequence,
		struct hubbub *hubbub)
{
	union block_sequence_params bls_params = {0};

	bls_params.hubbub_init_crb_params.hubbub = hubbub;
	return bls_append(sequence, HUBBUB_INIT_CRB, &bls_params);
}

static bool dcn401_add_request_limit(struct block_sequence_state *sequence,
		struct dc *dc)
{
	union block_sequence_params bls_params = {0};

	bls_params.hubbub_set_request_limit_params.hubbub =
			dc->res_pool->hubbub;
	bls_params.hubbub_set_request_limit_params.num_chans =
			dc->ctx->dc_bios->vram_info.num_chans;
	bls_params.hubbub_set_request_limit_params.words_per_umc =
			dc->config.sdpif_request_limit_words_per_umc;
	return bls_append(sequence, HUBBUB_SET_REQUEST_LIMIT, &bls_params);
}

/*
 * dc_dmub_srv_query_caps_cmd. The reply stays in the command; Core
 * interprets it. The caps apply whether or not the query is sent.
 */
static bool dcn401_add_query_caps(struct block_sequence_state *sequence,
		struct dc *dc, struct bls_builder_output_legacy_init_stage11 *out)
{
	struct dc_dmub_srv *dc_dmub_srv = dc->ctx->dmub_srv;

	if (!dc_dmub_srv)
		return true;
	if (!dc_dmub_srv->dmub)
		return false;
	out->dmub_caps_read = true;
	dcn401_fill_query_feature_caps_cmd(&out->dmub_query_feature_caps_cmd);
	out->dmub_caps_query_sent = true;
	return dcn401_add_dmub_cmd(sequence, dc->ctx,
			&out->dmub_query_feature_caps_cmd,
			DM_DMUB_WAIT_TYPE_WAIT_WITH_REPLY, true);
}

/* dc_dmub_srv_notify_fw_dc_power_state(dmub_srv, D0) */
static bool dcn401_add_notify_power_state(
		struct block_sequence_state *sequence, struct dc *dc,
		union dmub_rb_cmd *cmd)
{
	if (!dcn401_dc1_has_dmub(dc))
		return true;
	memset(cmd, 0, sizeof(*cmd));
	cmd->idle_opt_set_dc_power_state.header.type = DMUB_CMD__IDLE_OPT;
	cmd->idle_opt_set_dc_power_state.header.sub_type =
			DMUB_CMD__IDLE_OPT_SET_DC_POWER_STATE;
	cmd->idle_opt_set_dc_power_state.header.payload_bytes =
			sizeof(cmd->idle_opt_set_dc_power_state) -
			sizeof(cmd->idle_opt_set_dc_power_state.header);
	cmd->idle_opt_set_dc_power_state.data.power_state =
			DMUB_IDLE_OPT_DC_POWER_STATE_D0;
	return dcn401_add_dmub_cmd(sequence, dc->ctx, cmd,
			DM_DMUB_WAIT_TYPE_WAIT, true);
}

/*
 * Power down on boot, eDP slot by slot. A slot reads its eDP's DIG only
 * after the previous slot acted, since power_down can change it. With
 * no eDP, slot 0 reads every PHY link and powers down once.
 */
static bool dcn401_add_boot_dig_read(struct dc *dc,
		bool *out_edp_dig_enabled, bool *out_boot_dig_enabled,
		struct block_sequence_state *sequence, unsigned int slot)
{
	struct dc_link *edp_links[MAX_NUM_EDP];
	unsigned int edp_num;
	unsigned int i;

	if (dc->config.seamless_boot_edp_requested)
		return true;
	dc_get_edp_links(dc, edp_links, &edp_num);
	if (edp_num)
		return slot >= edp_num || dcn401_add_dig_read(sequence,
				edp_links[slot]->link_enc, out_edp_dig_enabled);
	if (slot)
		return true;
	for (i = 0; i < dc->link_count; i++)
		if (dcn401_dc1_is_phy_link(dc->links[i]) &&
				!dcn401_add_dig_read(sequence,
				dc->links[i]->link_enc,
				&out_boot_dig_enabled[i]))
			return false;
	return true;
}

static bool dcn401_add_boot_power_down(struct dc *dc,
		bool edp_dig_enabled, const bool *boot_dig_enabled,
		struct block_sequence_state *sequence, unsigned int slot)
{
	struct dc_link *edp_links[MAX_NUM_EDP];
	struct dc_link *link;
	unsigned int edp_num;
	unsigned int i;

	if (dc->config.seamless_boot_edp_requested ||
			!dc->hwseq->funcs.power_down)
		return true;
	dc_get_edp_links(dc, edp_links, &edp_num);
	if (edp_num) {
		if (slot >= edp_num || !edp_dig_enabled ||
				!dc->hwss.edp_backlight_control ||
				!dc->hwss.edp_power_control)
			return true;
		link = edp_links[slot];
		return dcn401_add_edp_control(sequence, EDP_BACKLIGHT_CONTROL,
				link, false) &&
				dcn401_add_dc_call(sequence, HWSEQ_POWER_DOWN, dc,
				false) &&
				dcn401_add_edp_control(sequence, EDP_POWER_CONTROL,
				link, false);
	}
	if (slot)
		return true;
	for (i = 0; i < dc->link_count; i++)
		if (boot_dig_enabled[i])
			return dcn401_add_dc_call(sequence, HWSEQ_POWER_DOWN, dc,
					false);
	return true;
}

/*
 * dcn401_init_hw call order, cut at each read a later stage depends on.
 * funcs append through dcn401_dc1_add_step: per pipe for the looped
 * calls, and nothing where dcn401_init_hw skips the call. link_funcs
 * follow step-major: each link_func expands for every link before the
 * next link_func. The BLC trace logs only enc401_hw_init per link, and
 * the panel, DDC and HDCP calls program separate blocks. In funcs,
 * DIO_LINK_ENC_IS_DIG_ENABLED and HWSEQ_POWER_DOWN are the power down
 * on boot read and act of the stage's eDP slot, and DMUB_SEND_DMCUB_CMD
 * is the stage's DMUB command.
 */
struct dcn401_dc1_stage_list {
	const enum block_sequence_func *funcs;
	unsigned int num_funcs;
	const enum block_sequence_func *link_funcs;
	unsigned int num_link_funcs;
};

static const enum block_sequence_func dcn401_dc1_clocks_stage[] = {
	LEGACY_CLK_MGR_INIT_CLOCKS,
	LEGACY_CLK_MGR_IS_DC_MODE_PRESENT,
	DCCG_INITIALIZE_CLOCKS,
	OPTC_SET_ODM_MEM_POWER,
	MMHUBBUB_SET_MEM_POWER,
	DCCG_GET_REF_FREQ,
};

static const enum block_sequence_func dcn401_dc1_dchub_ref_stage[] = {
	HUBBUB_GET_DCHUB_REF_FREQ,
};

static const enum block_sequence_func dcn401_dc1_link_enc_loop[] = {
	DIO_LINK_ENC_HW_INIT,
	DIO_LINK_ENC_IS_DIG_ENABLED,
	BIOS_GET_I2C_INFO,
};

static const enum block_sequence_func dcn401_dc1_fec_loop[] = {
	DIO_LINK_ENC_FEC_IS_ACTIVE,
};

static const enum block_sequence_func dcn401_dc1_blank_stage[] = {
	PG_SET_PLANE_POWER_GATING,
	LINK_BLANK_ALL_DP_DISPLAYS,
	BIOS_IS_ACCELERATED_MODE,
};

static const enum block_sequence_func dcn401_dc1_power_down_stage[] = {
	HWSS_ENABLE_ACCELERATED_MODE,
	HWSEQ_INIT_PIPES,
	HUBBUB_ALLOW_SELF_REFRESH_CONTROL,
	CLK_MGR_GET_DISPCLK_FROM_DENTIST,
};

static const enum block_sequence_func dcn401_dc1_min_clocks_stage[] = {
	LEGACY_CLK_MGR_INITIALIZE_MIN_CLOCKS,
	DMUB_SET_IDLE_OPTIMIZATION,
	DIO_LINK_ENC_IS_DIG_ENABLED,
};

static const enum block_sequence_func dcn401_dc1_power_down_edp0_stage[] = {
	HWSEQ_POWER_DOWN,
	DIO_LINK_ENC_IS_DIG_ENABLED,
};

static const enum block_sequence_func dcn401_dc1_power_down_edp1_stage[] = {
	HWSEQ_POWER_DOWN,
	AUDIO_HW_INIT,
};

static const enum block_sequence_func dcn401_dc1_panel_loop[] = {
	PANEL_CNTL_HW_INIT,
	GPIO_DDC_OPEN,
	DMUB_SEND_DMCUB_CMD,
};

static const enum block_sequence_func dcn401_dc1_gating_stage[] = {
	ABM_INIT,
	DIO_AFMT_SET_MEM_POWER,
	DCCG_SET_ROOT_CLOCK_GATING,
	DCCG_SET_DCFCLK_GATING,
	HPO_SET_HW_CONTROL,
	BIOS_IS_ACCELERATED_MODE,
};

static const enum block_sequence_func dcn401_dc1_hubbub_stage[] = {
	HUBBUB_INIT_WATERMARKS,
	LEGACY_CLK_MGR_NOTIFY_WM_RANGES,
	HUBBUB_FORCE_PSTATE_CHANGE_CONTROL,
	HUBBUB_INIT_CRB,
	HUBBUB_SET_REQUEST_LIMIT,
	DMUB_SEND_DMCUB_CMD,
};

static const enum block_sequence_func dcn401_dc1_power_state_stage[] = {
	DMUB_SEND_DMCUB_CMD,
};

#define DCN401_DC1_STAGE(list) { list, ARRAY_SIZE(list), NULL, 0 }
#define DCN401_DC1_LINK_STAGE(loop) { NULL, 0, loop, ARRAY_SIZE(loop) }
#define DCN401_DC1_LOOP_STAGE(list, loop) \
	{ list, ARRAY_SIZE(list), loop, ARRAY_SIZE(loop) }

static const struct dcn401_dc1_stage_list
		dcn401_dc1_stages[LEGACY_INIT_STAGE_COUNT + 1] = {
	[LEGACY_INIT_STAGE_CLOCKS] = DCN401_DC1_STAGE(dcn401_dc1_clocks_stage),
	[LEGACY_INIT_STAGE_DCHUB_REF] =
			DCN401_DC1_STAGE(dcn401_dc1_dchub_ref_stage),
	[LEGACY_INIT_STAGE_LINK_ENC] =
			DCN401_DC1_LINK_STAGE(dcn401_dc1_link_enc_loop),
	[LEGACY_INIT_STAGE_FEC] = DCN401_DC1_LINK_STAGE(dcn401_dc1_fec_loop),
	[LEGACY_INIT_STAGE_BLANK] = DCN401_DC1_STAGE(dcn401_dc1_blank_stage),
	[LEGACY_INIT_STAGE_POWER_DOWN] =
			DCN401_DC1_STAGE(dcn401_dc1_power_down_stage),
	[LEGACY_INIT_STAGE_MIN_CLOCKS] =
			DCN401_DC1_STAGE(dcn401_dc1_min_clocks_stage),
	[LEGACY_INIT_STAGE_POWER_DOWN_EDP0] =
			DCN401_DC1_STAGE(dcn401_dc1_power_down_edp0_stage),
	[LEGACY_INIT_STAGE_POWER_DOWN_EDP1] =
			DCN401_DC1_LOOP_STAGE(dcn401_dc1_power_down_edp1_stage,
			dcn401_dc1_panel_loop),
	[LEGACY_INIT_STAGE_GATING] = DCN401_DC1_STAGE(dcn401_dc1_gating_stage),
	[LEGACY_INIT_STAGE_HUBBUB] = DCN401_DC1_STAGE(dcn401_dc1_hubbub_stage),
	[LEGACY_INIT_STAGE_POWER_STATE] =
			DCN401_DC1_STAGE(dcn401_dc1_power_state_stage),
};

static bool dcn401_dc1_add_link_step(struct dc *dc,
		const struct bls_builder_legacy_hw_init_info *in,
		struct bls_builder_legacy_hw_init_info *out,
		struct block_sequence_state *sequence,
		enum block_sequence_func func, unsigned int i)
{
	const struct dc_link *link = dc->links[i];

	switch (func) {
	case DIO_LINK_ENC_HW_INIT:
		return !dcn401_dc1_is_phy_link(link) ||
				dcn401_add_link_enc_hw_init(sequence,
				link->link_enc);
	case DIO_LINK_ENC_IS_DIG_ENABLED:
		return !dcn401_dc1_is_phy_link(link) ||
				dcn401_add_dig_read(sequence, link->link_enc,
				&out->s3.link_dig_enabled[i]);
	case BIOS_GET_I2C_INFO:
		return !link->force_to_use_aux ||
				dcn401_add_i2c_info_read(sequence, dc, link,
				&out->s3.link_i2c_info[i],
				&out->s3.link_i2c_result[i]);
	case DIO_LINK_ENC_FEC_IS_ACTIVE:
		return !dcn401_dc1_is_phy_link(link) ||
				!in->s3.link_dig_enabled[i] ||
				!link->link_enc->funcs->fec_is_active ||
				dcn401_add_fec_read(sequence, link->link_enc,
				&out->s4.link_fec_active[i]);
	case PANEL_CNTL_HW_INIT:
		return !link->panel_cntl ||
				dcn401_add_panel_cntl_hw_init(sequence,
				link->panel_cntl, &out->s9.backlight,
				&out->s9.user_level);
	case GPIO_DDC_OPEN:
		return !link->force_to_use_aux ||
				in->s3.link_i2c_result[i] != BP_RESULT_OK ||
				dcn401_add_ddc_open(sequence, link,
				&in->s3.link_i2c_info[i]);
	case DMUB_SEND_DMCUB_CMD:
		/* Sent whatever the I2C info read returned. */
		return !link->force_to_use_aux || !dcn401_dc1_has_dmub(dc) ||
				dcn401_add_hdcp_interrupt_dest(sequence, dc, link,
				&out->s9.dmub_ihc_cmd[i]);
	default:
		return false;
	}
}

static bool dcn401_dc1_add_step(struct dc *dc,
		const struct bls_builder_legacy_hw_init_info *in,
		struct bls_builder_legacy_hw_init_info *out,
		struct block_sequence_state *sequence,
		enum block_sequence_func func)
{
	struct resource_pool *pool = dc->res_pool;
	struct clk_mgr *clk_mgr = dc->clk_mgr;
	struct hubbub *hubbub = pool->hubbub;
	bool has_init_clocks = clk_mgr && clk_mgr->funcs &&
			clk_mgr->funcs->init_clocks;
	unsigned int i;

	switch (func) {
	case LEGACY_CLK_MGR_INIT_CLOCKS:
		return !has_init_clocks ||
				dcn401_build_clk_mgr_init(clk_mgr, sequence) ==
				BLS_BUILD_SUCCESS;
	case LEGACY_CLK_MGR_IS_DC_MODE_PRESENT:
		/* Without the callback the limits stay not present. */
		return !has_init_clocks ||
				!clk_mgr->funcs->is_dc_mode_present ||
				dcn401_build_dc_mode_read(clk_mgr,
				&out->s1.dcmode_power_limits_present,
				sequence) == BLS_BUILD_SUCCESS;
	case DCCG_INITIALIZE_CLOCKS:
		return !pool->dccg->funcs->dccg_init ||
				dcn401_build_dccg_init(pool->dccg, sequence) ==
				BLS_BUILD_SUCCESS;
	case OPTC_SET_ODM_MEM_POWER:
		return !dc->debug.enable_mem_low_power.bits.optc ||
				dcn401_add_hwseq_reg(sequence, func, dc->hwseq,
				true);
	case MMHUBBUB_SET_MEM_POWER:
		return !dc->debug.enable_mem_low_power.bits.vga ||
				dcn401_add_hwseq_reg(sequence, func, dc->hwseq,
				true);
	case DCCG_GET_REF_FREQ:
		if (!dc->ctx->dc_bios->fw_info_valid) {
			ASSERT_CRITICAL(false);
			return true;
		}
		return dcn401_add_dccg_ref_freq(sequence, dc, &out->s1);
	case HUBBUB_GET_DCHUB_REF_FREQ:
		return !dc->ctx->dc_bios->fw_info_valid ||
				dcn401_add_dchub_ref_freq(sequence, dc, &out->s2);
	case PG_SET_PLANE_POWER_GATING:
		return !dc->hwseq->funcs.enable_power_gating_plane ||
				dcn401_add_hwseq_reg(sequence, func, dc->hwseq,
				true);
	case LINK_BLANK_ALL_DP_DISPLAYS:
		return dcn401_add_dc_call(sequence, func, dc, false);
	case BIOS_IS_ACCELERATED_MODE:
		return dcn401_add_accelerated_mode_read(sequence, dc,
				in->next_stage == LEGACY_INIT_STAGE_BLANK ?
				&out->s5.accelerated_mode :
				&out->s10.accelerated_mode);
	case HWSS_ENABLE_ACCELERATED_MODE:
		return !dcn401_dc1_powers_down_pipes(dc, in) ||
				!dcn401_dc1_uses_accelerated_mode(dc) ||
				dcn401_add_dc_call(sequence, func, dc, false);
	case HWSEQ_INIT_PIPES:
		return !dcn401_dc1_powers_down_pipes(dc, in) ||
				dcn401_dc1_uses_accelerated_mode(dc) ||
				dcn401_add_dc_call(sequence, func, dc, false);
	case HUBBUB_ALLOW_SELF_REFRESH_CONTROL:
		return !dcn401_dc1_powers_down_pipes(dc, in) ||
				!hubbub->funcs->allow_self_refresh_control ||
				dcn401_add_self_refresh_control(sequence, dc);
	case CLK_MGR_GET_DISPCLK_FROM_DENTIST:
		return !dcn401_dc1_powers_down_pipes(dc, in) ||
				dc->debug.disable_boot_optimizations ||
				!clk_mgr || !clk_mgr->funcs ||
				!clk_mgr->funcs->get_dispclk_from_dentist ||
				dcn401_add_dentist_read(sequence, clk_mgr,
				&out->s6.dispclk_from_dentist_khz);
	case LEGACY_CLK_MGR_INITIALIZE_MIN_CLOCKS:
		return !dcn401_dc1_powers_down_pipes(dc, in) ||
				dcn401_add_min_clocks(sequence, dc, in);
	case DMUB_SET_IDLE_OPTIMIZATION:
		/*
		 * Off, then on: in BOCO / BOMACO the hardware can be reset
		 * while software still has idle optimizations allowed.
		 */
		return !dcn401_dc1_powers_down_pipes(dc, in) ||
				(dcn401_add_dc_call(sequence, func, dc, false) &&
				dcn401_add_dc_call(sequence, func, dc, true));
	case DIO_LINK_ENC_IS_DIG_ENABLED:
		if (in->next_stage == LEGACY_INIT_STAGE_MIN_CLOCKS)
			return dcn401_add_boot_dig_read(dc,
					&out->s7.edp_dig_enabled,
					out->s7.boot_dig_enabled, sequence, 0);
		return dcn401_add_boot_dig_read(dc, &out->s8.edp_dig_enabled,
				NULL, sequence, 1);
	case HWSEQ_POWER_DOWN:
		if (in->next_stage == LEGACY_INIT_STAGE_POWER_DOWN_EDP0)
			return dcn401_add_boot_power_down(dc,
					in->s7.edp_dig_enabled,
					in->s7.boot_dig_enabled, sequence, 0);
		return dcn401_add_boot_power_down(dc, in->s8.edp_dig_enabled,
				NULL, sequence, 1);
	case AUDIO_HW_INIT:
		for (i = 0; i < pool->audio_count; i++)
			if (!dcn401_add_audio_hw_init(sequence, pool->audios[i]))
				return false;
		return true;
	case ABM_INIT:
		for (i = 0; i < pool->pipe_count; i++) {
			struct abm *abm = pool->multiple_abms[i];

			if (abm && abm->funcs &&
					!dcn401_add_abm_init(sequence, abm, in))
				return false;
		}
		return true;
	case DIO_AFMT_SET_MEM_POWER:
		return !pool->dio || !pool->dio->funcs->mem_pwr_ctrl ||
				dcn401_add_dio_mem_power(sequence, pool->dio);
	case DCCG_SET_ROOT_CLOCK_GATING:
		return dc->debug.disable_clock_gate ||
				!pool->dccg->funcs->allow_clock_gating ||
				dcn401_add_root_clock_gating(sequence, pool->dccg);
	case DCCG_SET_DCFCLK_GATING:
		return dc->debug.disable_clock_gate ||
				dcn401_add_hwseq_reg(sequence, func, dc->hwseq,
				true);
	case HPO_SET_HW_CONTROL:
		return dcn401_add_hwseq_reg(sequence, func, dc->hwseq, true);
	case HUBBUB_INIT_WATERMARKS:
		return in->s10.accelerated_mode ||
				!hubbub->funcs->init_watermarks ||
				dcn401_add_init_watermarks(sequence, hubbub);
	case LEGACY_CLK_MGR_NOTIFY_WM_RANGES:
		return !clk_mgr || !clk_mgr->funcs ||
				!clk_mgr->funcs->notify_wm_ranges ||
				dcn401_add_notify_wm_ranges(sequence, clk_mgr);
	case HUBBUB_FORCE_PSTATE_CHANGE_CONTROL:
		return !hubbub->funcs->force_pstate_change_control ||
				dcn401_add_force_pstate(sequence, hubbub);
	case HUBBUB_INIT_CRB:
		return !hubbub->funcs->init_crb ||
				dcn401_add_init_crb(sequence, hubbub);
	case HUBBUB_SET_REQUEST_LIMIT:
		return !hubbub->funcs->set_request_limit ||
				dc->config.sdpif_request_limit_words_per_umc == 0 ||
				dcn401_add_request_limit(sequence, dc);
	case DMUB_SEND_DMCUB_CMD:
		if (in->next_stage == LEGACY_INIT_STAGE_POWER_STATE)
			return dcn401_add_notify_power_state(sequence, dc,
					&out->s12.dmub_power_state_cmd);
		return dcn401_add_query_caps(sequence, dc, &out->s11);
	default:
		return false;
	}
}

static bool dcn401_dc1_add_stage(struct dc *dc,
		const struct bls_builder_legacy_hw_init_info *in,
		struct bls_builder_legacy_hw_init_info *out,
		struct block_sequence_state *sequence)
{
	const struct dcn401_dc1_stage_list *stage =
			&dcn401_dc1_stages[in->next_stage];
	unsigned int i, j;

	for (i = 0; i < stage->num_funcs; i++)
		if (!dcn401_dc1_add_step(dc, in, out, sequence,
				stage->funcs[i]))
			return false;
	if (!stage->num_link_funcs)
		return true;
	for (j = 0; j < stage->num_link_funcs; j++)
		for (i = 0; i < dc->link_count; i++)
			if (!dcn401_dc1_add_link_step(dc, in, out, sequence,
					stage->link_funcs[j], i))
				return false;

	return true;
}

/*
 * Seed the stage output with the values its reads leave in place when
 * they are skipped: the current reference clocks, the default
 * backlight and no I2C info.
 */
static void dcn401_dc1_seed_stage_output(struct dc *dc,
		struct bls_builder_legacy_hw_init_info *out, unsigned int stage)
{
	const struct resource_pool *pool = dc->res_pool;
	unsigned int i;

	switch (stage) {
	case LEGACY_INIT_STAGE_CLOCKS:
		memset(&out->s1, 0, sizeof(out->s1));
		out->s1.xtalin_clock_inKhz = pool->ref_clocks.xtalin_clock_inKhz;
		out->s1.dccg_ref_clock_inKhz =
				pool->ref_clocks.dccg_ref_clock_inKhz;
		break;
	case LEGACY_INIT_STAGE_DCHUB_REF:
		memset(&out->s2, 0, sizeof(out->s2));
		out->s2.dchub_ref_clock_inKhz =
				pool->ref_clocks.dchub_ref_clock_inKhz;
		break;
	case LEGACY_INIT_STAGE_LINK_ENC:
		memset(&out->s3, 0, sizeof(out->s3));
		for (i = 0; i < MAX_LINKS; i++)
			out->s3.link_i2c_result[i] = BP_RESULT_NORECORD;
		break;
	case LEGACY_INIT_STAGE_FEC:
		memset(&out->s4, 0, sizeof(out->s4));
		break;
	case LEGACY_INIT_STAGE_BLANK:
		memset(&out->s5, 0, sizeof(out->s5));
		break;
	case LEGACY_INIT_STAGE_POWER_DOWN:
		memset(&out->s6, 0, sizeof(out->s6));
		break;
	case LEGACY_INIT_STAGE_MIN_CLOCKS:
		memset(&out->s7, 0, sizeof(out->s7));
		break;
	case LEGACY_INIT_STAGE_POWER_DOWN_EDP0:
		memset(&out->s8, 0, sizeof(out->s8));
		break;
	case LEGACY_INIT_STAGE_POWER_DOWN_EDP1:
		memset(&out->s9, 0, sizeof(out->s9));
		out->s9.backlight = MAX_BACKLIGHT_LEVEL;
		out->s9.user_level = MAX_BACKLIGHT_LEVEL;
		break;
	case LEGACY_INIT_STAGE_GATING:
		memset(&out->s10, 0, sizeof(out->s10));
		break;
	case LEGACY_INIT_STAGE_HUBBUB:
		memset(&out->s11, 0, sizeof(out->s11));
		break;
	case LEGACY_INIT_STAGE_POWER_STATE:
		memset(&out->s12, 0, sizeof(out->s12));
		break;
	}
}

enum bls_builder_result dcn401_build_dc1_initialization(
		struct bls_builder_declarative_params *params)
{
	const struct bls_builder_legacy_hw_init_info *in;
	struct bls_builder_legacy_hw_init_info *out;
	struct block_sequence_state sequence;
	enum bls_builder_result result;
	struct dc *dc;

	if (!params)
		return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;
	params->head_num_steps = 0;
	params->tail_num_steps = 0;
	result = dcn401_validate_dc1_initialization(params);
	if (result != BLS_BUILD_SUCCESS)
		return result;

	dc = params->dc;
	in = &params->current_state->hw_init_info;
	out = params->out_targets->DCN_DECL_INITIALIZATION_LEGACY;
	sequence.steps = params->head_sequence;
	sequence.num_steps = &params->head_num_steps;

	/* Virtual hardware only notifies DMUB of the power state. */
	if (dc->ctx->dce_environment == DCE_ENV_VIRTUAL_HW) {
		dcn401_dc1_seed_stage_output(dc, out,
				LEGACY_INIT_STAGE_POWER_STATE);
		if (!dcn401_add_notify_power_state(&sequence, dc,
				&out->s12.dmub_power_state_cmd))
			return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;
		out->next_stage = LEGACY_INIT_STAGE_COUNT + 1;
		out->init_done = true;
		return BLS_BUILD_SUCCESS;
	}

	dcn401_dc1_seed_stage_output(dc, out, in->next_stage);
	if (!dcn401_dc1_add_stage(dc, in, out, &sequence)) {
		params->head_num_steps = 0;
		return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;
	}
	out->next_stage = in->next_stage + 1;
	out->init_done = in->next_stage == LEGACY_INIT_STAGE_COUNT;

	return BLS_BUILD_SUCCESS;
}

bool dcn401_dc1_initialization_check_capability(
		const struct bls_builder_check_capability_params *params)
{
	return params && params->dc && params->dc->ctx &&
			params->dc->res_pool;
}
