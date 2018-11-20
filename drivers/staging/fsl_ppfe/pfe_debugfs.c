// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>

#include "pfe_mod.h"

static int dmem_show(struct seq_file *s, void *unused)
{
	u32 dmem_addr, val;
	int id = (long int)s->private;
	int i;

	for (dmem_addr = 0; dmem_addr < CLASS_DMEM_SIZE; dmem_addr += 8 * 4) {
		seq_printf(s, "%04x:", dmem_addr);

		for (i = 0; i < 8; i++) {
			val = pe_dmem_read(id, dmem_addr + i * 4, 4);
			seq_printf(s, " %02x %02x %02x %02x", val & 0xff,
				   (val >> 8) & 0xff, (val >> 16) & 0xff,
				   (val >> 24) & 0xff);
		}

		seq_puts(s, "\n");
	}

	return 0;
}

static int dmem_open(struct inode *inode, struct file *file)
{
	return single_open(file, dmem_show, inode->i_private);
}

static const struct file_operations dmem_fops = {
	.open		= dmem_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int pfe_debugfs_init(struct pfe *pfe)
{
	struct dentry *d;

	pr_info("%s\n", __func__);

	pfe->dentry = debugfs_create_dir("pfe", NULL);
	if (IS_ERR_OR_NULL(pfe->dentry))
		goto err_dir;

	d = debugfs_create_file("pe0_dmem", 0444, pfe->dentry, (void *)0,
				&dmem_fops);
	if (IS_ERR_OR_NULL(d))
		goto err_pe;

	d = debugfs_create_file("pe1_dmem", 0444, pfe->dentry, (void *)1,
				&dmem_fops);
	if (IS_ERR_OR_NULL(d))
		goto err_pe;

	d = debugfs_create_file("pe2_dmem", 0444, pfe->dentry, (void *)2,
				&dmem_fops);
	if (IS_ERR_OR_NULL(d))
		goto err_pe;

	d = debugfs_create_file("pe3_dmem", 0444, pfe->dentry, (void *)3,
				&dmem_fops);
	if (IS_ERR_OR_NULL(d))
		goto err_pe;

	d = debugfs_create_file("pe4_dmem", 0444, pfe->dentry, (void *)4,
				&dmem_fops);
	if (IS_ERR_OR_NULL(d))
		goto err_pe;

	d = debugfs_create_file("pe5_dmem", 0444, pfe->dentry, (void *)5,
				&dmem_fops);
	if (IS_ERR_OR_NULL(d))
		goto err_pe;

	return 0;

err_pe:
	debugfs_remove_recursive(pfe->dentry);

err_dir:
	return -1;
}

void pfe_debugfs_exit(struct pfe *pfe)
{
	debugfs_remove_recursive(pfe->dentry);
}
