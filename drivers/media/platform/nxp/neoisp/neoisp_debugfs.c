// SPDX-License-Identifier: GPL-2.0+
/*
 * NEOISP debugfs definition
 *
 * Copyright 2024 NXP
 * Author: Alexi Birlinger (alexi.birlinger@nxp.com)
 *
 */

#include <linux/debugfs.h>

#include "neoisp.h"
#include "neoisp_ctx.h"

/* Structure to store word when reading memory */
union udata_t {
	__u8 byte[4];
	__u16 half[2];
	__u32 word;
};

static inline int neoisp_dump_memory(struct seq_file *m, enum isp_block_map_e map, int wsize)
{
	struct neoisp_dev_s *neoispd = m->private;
	union udata_t data;
	__u32 addr;
	__u32 *src = (__u32 *)(uintptr_t)neoispd->mmio_tcm;
	__u32 offset = ISP_GET_OFF(map);
	__u32 size = ISP_GET_WSZ(map);
	int i, j;

	for (i = 0; i < size; i++) {
		addr = (offset + i) * sizeof(__u32);
		data.word = src[offset + i];

		if (wsize == sizeof(__u8)) {
			for (j = 0; j < ARRAY_SIZE(data.byte); j++)
				seq_printf(m, "%#x: %#x\n",
					   addr + (j * wsize), data.byte[j]);
		}

		if (wsize == sizeof(__u16)) {
			for (j = 0; j < ARRAY_SIZE(data.half); j++)
				seq_printf(m, "%#x: %#x\n",
					   addr + (j * wsize), data.half[j]);
		}
	}

	return 0;
}

static int neoisp_dump_vignetting_show(struct seq_file *m, void *private)
{
	return neoisp_dump_memory(m, NEO_VIGNETTING_TABLE_MAP, sizeof(__u16));
}
DEFINE_SHOW_ATTRIBUTE(neoisp_dump_vignetting);

static int neoisp_dump_drc_global_show(struct seq_file *m, void *private)
{
	return neoisp_dump_memory(m, NEO_DRC_GLOBAL_TONEMAP_MAP, sizeof(__u16));
}
DEFINE_SHOW_ATTRIBUTE(neoisp_dump_drc_global);

static int neoisp_dump_drc_local_show(struct seq_file *m, void *private)
{
	return neoisp_dump_memory(m, NEO_DRC_LOCAL_TONEMAP_MAP, sizeof(__u8));
}
DEFINE_SHOW_ATTRIBUTE(neoisp_dump_drc_local);

void neoisp_debugfs_init(struct neoisp_dev_s *neoispd)
{
	neoispd->debugfs_entry =
		debugfs_create_dir(dev_name(&neoispd->pdev->dev), NULL);

	debugfs_create_file("vignetting", 0400, neoispd->debugfs_entry, neoispd,
			    &neoisp_dump_vignetting_fops);
	debugfs_create_file("drc_global", 0400, neoispd->debugfs_entry, neoispd,
			    &neoisp_dump_drc_global_fops);
	debugfs_create_file("drc_local", 0400, neoispd->debugfs_entry, neoispd,
			    &neoisp_dump_drc_local_fops);
}

void neoisp_debugfs_exit(struct neoisp_dev_s *neoispd)
{
	debugfs_remove_recursive(neoispd->debugfs_entry);
}
