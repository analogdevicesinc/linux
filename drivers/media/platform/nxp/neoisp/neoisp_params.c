// SPDX-License-Identifier: GPL-2.0+
/*
 * NEOISP module parameters definition and default values
 *
 * Copyright 2023-2024 NXP
 */

#include "neoisp.h"

/* default values */
struct neoisp_mod_params_s mod_params = {
	.test = {
		.disable_params = 0,
		.disable_stats = 0,
	},
	.conf = {
		.img_conf_cam0_ibpp0 = 2,
		.img_conf_cam0_inalign0 = 1,
		.img_conf_cam0_lpalign0 = 0,
		.img_conf_cam0_ibpp1 = 2,
		.img_conf_cam0_inalign1 = 1,
		.img_conf_cam0_lpalign1 = 0,
		.img0_in_ls_cam0_ls = 0,
		.img1_in_ls_cam0_ls = 0,
		.skip_ctrl0_preskip = 0,
		.skip_ctrl0_postskip = 0,
	},
	.pack = {
		.ch0_ctrl_cam0_obpp = 6,
		.ch0_ctrl_cam0_rsa = 4,
		.ch0_ctrl_cam0_lsa = 0,
		.ch12_ctrl_cam0_obpp = 6,
		.ch12_ctrl_cam0_rsa = 4,
		.ch12_ctrl_cam0_lsa = 0,
		.ch12_ctrl_cam0_subsample = 0,
		.ctrl_cam0_type = 1,
		.ctrl_cam0_order0 = 0,
		.ctrl_cam0_order1 = 1,
		.ctrl_cam0_order2 = 2,
		.ctrl_cam0_a0s = 0,
	}
};

module_param_named(test_disable_params, mod_params.test.disable_params, uint, 0600);
module_param_named(test_disable_stats, mod_params.test.disable_stats, uint, 0600);

module_param_named(img_conf_cam0_ibpp0, mod_params.conf.img_conf_cam0_ibpp0, uint, 0600);
module_param_named(img_conf_cam0_inalign0, mod_params.conf.img_conf_cam0_inalign0, uint, 0600);
module_param_named(img_conf_cam0_lpalign0, mod_params.conf.img_conf_cam0_lpalign0, uint, 0600);
module_param_named(img_conf_cam0_ibpp1, mod_params.conf.img_conf_cam0_ibpp1, uint, 0600);
module_param_named(img_conf_cam0_inalign1, mod_params.conf.img_conf_cam0_inalign1, uint, 0600);
module_param_named(img_conf_cam0_lpalign1, mod_params.conf.img_conf_cam0_lpalign1, uint, 0600);
module_param_named(img0_in_ls_cam0_ls, mod_params.conf.img0_in_ls_cam0_ls, uint, 0600);
module_param_named(img1_in_ls_cam0_ls, mod_params.conf.img1_in_ls_cam0_ls, uint, 0600);
module_param_named(skip_ctrl0_preskip, mod_params.conf.skip_ctrl0_preskip, uint, 0600);
module_param_named(skip_ctrl0_postskip, mod_params.conf.skip_ctrl0_postskip, uint, 0600);

module_param_named(ch0_ctrl_cam0_obpp, mod_params.pack.ch0_ctrl_cam0_obpp, uint, 0600);
module_param_named(ch0_ctrl_cam0_rsa, mod_params.pack.ch0_ctrl_cam0_rsa, uint, 0600);
module_param_named(ch0_ctrl_cam0_lsa, mod_params.pack.ch0_ctrl_cam0_lsa, uint, 0600);
module_param_named(ch12_ctrl_cam0_obpp, mod_params.pack.ch12_ctrl_cam0_obpp, uint, 0600);
module_param_named(ch12_ctrl_cam0_rsa, mod_params.pack.ch12_ctrl_cam0_rsa, uint, 0600);
module_param_named(ch12_ctrl_cam0_lsa, mod_params.pack.ch12_ctrl_cam0_lsa, uint, 0600);
module_param_named(ch12_ctrl_cam0_subsample, mod_params.pack.ch12_ctrl_cam0_subsample, uint, 0600);
module_param_named(ctrl_cam0_type, mod_params.pack.ctrl_cam0_type, uint, 0600);
module_param_named(ctrl_cam0_order0, mod_params.pack.ctrl_cam0_order0, uint, 0600);
module_param_named(ctrl_cam0_order1, mod_params.pack.ctrl_cam0_order1, uint, 0600);
module_param_named(ctrl_cam0_order2, mod_params.pack.ctrl_cam0_order2, uint, 0600);
module_param_named(ctrl_cam0_a0s, mod_params.pack.ctrl_cam0_a0s, uint, 0600);
