// SPDX-License-Identifier: GPL-2.0-only
/*
 * amd-acp7x-acpi-match.c - tables and support for ACP7.x(7.B/7.F) platforms
 * ACPI enumeration.
 *
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#include <sound/soc-acpi.h>
#include "../mach-config.h"

static const struct snd_soc_acpi_endpoint single_endpoint = {
	.num = 0,
	.aggregated = 0,
	.group_position = 0,
	.group_id = 0
};

static const struct snd_soc_acpi_endpoint jack_dmic_endpoints[] = {
	/* Jack Endpoint */
	{
		.num = 0,
		.aggregated = 0,
		.group_position = 0,
		.group_id = 0,
	},
	/* DMIC Endpoint */
	{
		.num = 2,
		.aggregated = 0,
		.group_position = 0,
		.group_id = 0,
	},
};

static const struct snd_soc_acpi_endpoint jack_amp_dmic_endpoints[] = {
	/* Jack Endpoint */
	{
		.num = 0,
		.aggregated = 0,
		.group_position = 0,
		.group_id = 0,
	},
	/* Amp Endpoint */
	{
		.num = 1,
		.aggregated = 0,
		.group_position = 0,
		.group_id = 0,
	},
	/* DMIC Endpoint */
	{
		.num = 2,
		.aggregated = 0,
		.group_position = 0,
		.group_id = 0,
	},
};

static const struct snd_soc_acpi_adr_device rt721_l0_jack_dmic_adr[] = {
	{
		.adr = 0x000030025d072101ull,
		.num_endpoints = ARRAY_SIZE(jack_dmic_endpoints),
		.endpoints = jack_dmic_endpoints,
		.name_prefix = "rt721"
	}
};

static const struct snd_soc_acpi_adr_device rt721_l0_jack_amp_dmic_adr[] = {
	{
		.adr = 0x000030025d072101ull,
		.num_endpoints = ARRAY_SIZE(jack_amp_dmic_endpoints),
		.endpoints = jack_amp_dmic_endpoints,
		.name_prefix = "rt721"
	}
};

static const struct snd_soc_acpi_link_adr acp7x_rt721_only[] = {
	{
		.mask = BIT(0),
		.num_adr = ARRAY_SIZE(rt721_l0_jack_amp_dmic_adr),
		.adr_d = rt721_l0_jack_amp_dmic_adr,
	},
	{}
};

static const struct snd_soc_acpi_adr_device rt722_l0_jack_amp_dmic_adr[] = {
	{
		.adr = 0x000030025d072201ull,
		.num_endpoints = ARRAY_SIZE(jack_amp_dmic_endpoints),
		.endpoints = jack_amp_dmic_endpoints,
		.name_prefix = "rt722"
	}
};

static const struct snd_soc_acpi_link_adr acp7x_rt722_only[] = {
	{
		.mask = BIT(0),
		.num_adr = ARRAY_SIZE(rt722_l0_jack_amp_dmic_adr),
		.adr_d = rt722_l0_jack_amp_dmic_adr,
	},
	{}
};

static const struct snd_soc_acpi_adr_device rt1320_l1_single_adr[] = {
	{
		.adr = 0x000130025d132001ull,
		.num_endpoints = 1,
		.endpoints = &single_endpoint,
		.name_prefix = "rt1320-1"
	},
};

static const struct snd_soc_acpi_adr_device rt1320_l3_single_adr[] = {
	{
		.adr = 0x000330025d132001ull,
		.num_endpoints = 1,
		.endpoints = &single_endpoint,
		.name_prefix = "rt1320-1"
	},
};

static const struct snd_soc_acpi_link_adr acp7x_rt721_l0_rt1320_l1[] = {
	{
		.mask = BIT(0),
		.num_adr = ARRAY_SIZE(rt721_l0_jack_dmic_adr),
		.adr_d = rt721_l0_jack_dmic_adr,
	},
	{
		.mask = BIT(1),
		.num_adr = ARRAY_SIZE(rt1320_l1_single_adr),
		.adr_d = rt1320_l1_single_adr,
	},
	{}
};

static const struct snd_soc_acpi_link_adr acp7x_rt721_l0_rt1320_l3[] = {
	{
		.mask = BIT(0),
		.num_adr = ARRAY_SIZE(rt721_l0_jack_dmic_adr),
		.adr_d = rt721_l0_jack_dmic_adr,
	},
	{
		.mask = BIT(3),
		.num_adr = ARRAY_SIZE(rt1320_l3_single_adr),
		.adr_d = rt1320_l3_single_adr,
	},
	{}
};

struct snd_soc_acpi_mach snd_soc_acpi_amd_acp7b_sof_sdw_machines[] = {
	{
		.link_mask = BIT(0) | BIT(3),
		.links = acp7x_rt721_l0_rt1320_l3,
		.drv_name = "amd_sof_sdw",
		.sof_tplg_filename = "sof-acp7b-rt721-l0-rt1320-l3.tplg",
		.fw_filename = "sof-acp7b.ri",
	},
	{},
};
EXPORT_SYMBOL(snd_soc_acpi_amd_acp7b_sof_sdw_machines);

struct snd_soc_acpi_mach snd_soc_acpi_amd_acp7f_sof_sdw_machines[] = {
	{
		.link_mask = BIT(0) | BIT(3),
		.links = acp7x_rt721_l0_rt1320_l3,
		.drv_name = "amd_sof_sdw",
		.sof_tplg_filename = "sof-acp7f-rt721-l0-rt1320-l3.tplg",
		.fw_filename = "sof-acp7f.ri",
	},
	{
		.link_mask = BIT(0) | BIT(1),
		.links = acp7x_rt721_l0_rt1320_l1,
		.drv_name = "amd_sof_sdw",
		.sof_tplg_filename = "sof-acp7f-rt721-l0-rt1320-l1.tplg",
		.fw_filename = "sof-acp7f.ri",
	},
	{
		.link_mask = BIT(0),
		.links = acp7x_rt721_only,
		.drv_name = "amd_sof_sdw",
		.sof_tplg_filename = "sof-acp7f-rt721-l0.tplg",
		.fw_filename = "sof-acp7f.ri",
	},
	{
		.link_mask = BIT(0),
		.links = acp7x_rt722_only,
		.drv_name = "amd_sof_sdw",
		.sof_tplg_filename = "sof-acp7f-rt722-l0.tplg",
		.fw_filename = "sof-acp7f.ri",
	},
	{},
};
EXPORT_SYMBOL(snd_soc_acpi_amd_acp7f_sof_sdw_machines);

MODULE_DESCRIPTION("AMD ACP7.x SoundWire machine tables for ACPI enumeration");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vijendar Mukunda <Vijendar.Mukunda@amd.com>");
