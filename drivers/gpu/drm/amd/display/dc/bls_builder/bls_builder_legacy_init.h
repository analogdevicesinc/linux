/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#pragma once

#include "dmub_cmd.h"
#include "grph_object_ctrl_defs.h"
#include "bios_parser_types.h"
#include "inc/hw/hw_shared.h"

/*
 * DC2_DC1_Wrapped hardware init: dcn401_init_hw as twelve BLS stages.
 * A stage ends on a read that a later stage depends on. Stage N writes
 * bls_builder_output_legacy_init_stageN; a later stage reads it from
 * dc_state.hw_init_info.
 */
enum bls_builder_legacy_init_stage {
	LEGACY_INIT_STAGE_CLOCKS = 1,
	LEGACY_INIT_STAGE_DCHUB_REF,
	LEGACY_INIT_STAGE_LINK_ENC,
	LEGACY_INIT_STAGE_FEC,
	LEGACY_INIT_STAGE_BLANK,
	LEGACY_INIT_STAGE_POWER_DOWN,
	LEGACY_INIT_STAGE_MIN_CLOCKS,
	LEGACY_INIT_STAGE_POWER_DOWN_EDP0,
	LEGACY_INIT_STAGE_POWER_DOWN_EDP1,
	LEGACY_INIT_STAGE_GATING,
	LEGACY_INIT_STAGE_HUBBUB,
	LEGACY_INIT_STAGE_POWER_STATE,
	LEGACY_INIT_STAGE_COUNT = LEGACY_INIT_STAGE_POWER_STATE,
};

/* CLOCKS: DC-mode limits and the DCCG reference clocks. */
struct bls_builder_output_legacy_init_stage1 {
	bool dcmode_power_limits_present;
	unsigned int xtalin_clock_inKhz;
	unsigned int dccg_ref_clock_inKhz;
};

/* DCHUB_REF: the DCHUB reference clock, before and after the read. */
struct bls_builder_output_legacy_init_stage2 {
	unsigned int dchub_ref_clock_mhz_before;
	unsigned int dchub_ref_clock_inKhz;
};

/* LINK_ENC: per dc->links entry, the DIG state and the I2C info. */
struct bls_builder_output_legacy_init_stage3 {
	bool link_dig_enabled[MAX_LINKS];
	struct graphics_object_i2c_info link_i2c_info[MAX_LINKS];
	enum bp_result link_i2c_result[MAX_LINKS];
};

/* FEC: per dc->links entry, whether FEC is active. */
struct bls_builder_output_legacy_init_stage4 {
	bool link_fec_active[MAX_LINKS];
};

/* BLANK: the VBIOS accelerated-mode bit after blanking. */
struct bls_builder_output_legacy_init_stage5 {
	bool accelerated_mode;
};

/* POWER_DOWN: the DENTIST dispclk. */
struct bls_builder_output_legacy_init_stage6 {
	int dispclk_from_dentist_khz;
};

/* MIN_CLOCKS: power-down-on-boot DIG reads for eDP slot 0, or every link. */
struct bls_builder_output_legacy_init_stage7 {
	bool edp_dig_enabled;
	bool boot_dig_enabled[MAX_LINKS];
};

/* POWER_DOWN_EDP0: the power-down-on-boot DIG read for eDP slot 1. */
struct bls_builder_output_legacy_init_stage8 {
	bool edp_dig_enabled;
};

/* POWER_DOWN_EDP1: the backlight panel_cntl reports, and the IHC commands. */
struct bls_builder_output_legacy_init_stage9 {
	uint32_t backlight;
	uint32_t user_level;
	union dmub_rb_cmd dmub_ihc_cmd[MAX_LINKS];
};

/* GATING: the VBIOS accelerated-mode bit after gating. */
struct bls_builder_output_legacy_init_stage10 {
	bool accelerated_mode;
};

/*
 * HUBBUB: whether the DMUB feature caps apply, whether the query was
 * sent, and the command that holds the reply.
 */
struct bls_builder_output_legacy_init_stage11 {
	bool dmub_caps_read;
	bool dmub_caps_query_sent;
	union dmub_rb_cmd dmub_query_feature_caps_cmd;
};

/* POWER_STATE: the D0 power-state command. */
struct bls_builder_output_legacy_init_stage12 {
	union dmub_rb_cmd dmub_power_state_cmd;
};

/*
 * next_stage is the stage the builder builds next, and s1 up to the
 * stage before it are valid. Core stops at init_done.
 */
struct bls_builder_legacy_hw_init_info {
	bool init_done;
	bool init_success;
	unsigned int next_stage;
	struct bls_builder_output_legacy_init_stage1 s1;
	struct bls_builder_output_legacy_init_stage2 s2;
	struct bls_builder_output_legacy_init_stage3 s3;
	struct bls_builder_output_legacy_init_stage4 s4;
	struct bls_builder_output_legacy_init_stage5 s5;
	struct bls_builder_output_legacy_init_stage6 s6;
	struct bls_builder_output_legacy_init_stage7 s7;
	struct bls_builder_output_legacy_init_stage8 s8;
	struct bls_builder_output_legacy_init_stage9 s9;
	struct bls_builder_output_legacy_init_stage10 s10;
	struct bls_builder_output_legacy_init_stage11 s11;
	struct bls_builder_output_legacy_init_stage12 s12;
};
