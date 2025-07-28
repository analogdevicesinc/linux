/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2022, Intel Corporation. */

#ifndef _ICE_DPLL_H_
#define _ICE_DPLL_H_

#include "ice.h"

#define ICE_DPLL_RCLK_NUM_MAX	4

/** ice_dpll_pin - store info about pins
 * @pin: dpll pin structure
 * @pf: pointer to pf, which has registered the dpll_pin
 * @idx: ice pin private idx
 * @num_parents: hols number of parent pins
 * @parent_idx: hold indexes of parent pins
 * @flags: pin flags returned from HW
 * @state: state of a pin
 * @prop: pin properties
 * @freq: current frequency of a pin
 * @phase_adjust: current phase adjust value
 */
struct ice_dpll_pin {
	struct dpll_pin *pin;
	struct ice_pf *pf;
	u8 idx;
	u8 num_parents;
	u8 parent_idx[ICE_DPLL_RCLK_NUM_MAX];
	u8 flags[ICE_DPLL_RCLK_NUM_MAX];
	u8 state[ICE_DPLL_RCLK_NUM_MAX];
	struct dpll_pin_properties prop;
	u32 freq;
	s32 phase_adjust;
	u8 status;
};

/** ice_dpll - store info required for DPLL control
 * @dpll: pointer to dpll dev
 * @pf: pointer to pf, which has registered the dpll_device
 * @dpll_idx: index of dpll on the NIC
 * @input_idx: currently selected input index
 * @prev_input_idx: previously selected input index
 * @ref_state: state of dpll reference signals
 * @eec_mode: eec_mode dpll is configured for
 * @phase_offset: phase offset of active pin vs dpll signal
 * @prev_phase_offset: previous phase offset of active pin vs dpll signal
 * @input_prio: priorities of each input
 * @dpll_state: current dpll sync state
 * @prev_dpll_state: last dpll sync state
 * @active_input: pointer to active input pin
 * @prev_input: pointer to previous active input pin
 */
struct ice_dpll {
	struct dpll_device *dpll;
	struct ice_pf *pf;
	u8 dpll_idx;
	u8 input_idx;
	u8 prev_input_idx;
	u8 ref_state;
	u8 eec_mode;
	s64 phase_offset;
	s64 prev_phase_offset;
	u8 *input_prio;
	enum dpll_lock_status dpll_state;
	enum dpll_lock_status prev_dpll_state;
	enum dpll_mode mode;
	struct dpll_pin *active_input;
	struct dpll_pin *prev_input;
};

/** ice_dplls - store info required for CCU (clock controlling unit)
 * @kworker: periodic worker
 * @work: periodic work
 * @lock: locks access to configuration of a dpll
 * @eec: pointer to EEC dpll dev
 * @pps: pointer to PPS dpll dev
 * @inputs: input pins pointer
 * @outputs: output pins pointer
 * @rclk: recovered pins pointer
 * @num_inputs: number of input pins available on dpll
 * @num_outputs: number of output pins available on dpll
 * @cgu_state_acq_err_num: number of errors returned during periodic work
 * @base_rclk_idx: idx of first pin used for clock revocery pins
 * @clock_id: clock_id of dplls
 * @input_phase_adj_max: max phase adjust value for an input pins
 * @output_phase_adj_max: max phase adjust value for an output pins
 */
struct ice_dplls {
	struct kthread_worker *kworker;
	struct kthread_delayed_work work;
	struct mutex lock;
	struct ice_dpll eec;
	struct ice_dpll pps;
	struct ice_dpll_pin *inputs;
	struct ice_dpll_pin *outputs;
	struct ice_dpll_pin rclk;
	u8 num_inputs;
	u8 num_outputs;
	int cgu_state_acq_err_num;
	u8 base_rclk_idx;
	u64 clock_id;
	s32 input_phase_adj_max;
	s32 output_phase_adj_max;
};

#if IS_ENABLED(CONFIG_PTP_1588_CLOCK)
void ice_dpll_init(struct ice_pf *pf);
void ice_dpll_deinit(struct ice_pf *pf);
#else
static inline void ice_dpll_init(struct ice_pf *pf) { }
static inline void ice_dpll_deinit(struct ice_pf *pf) { }
#endif

#endif
