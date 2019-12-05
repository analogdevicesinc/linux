// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP
 *
 */

/*
 * The module exposes 3 files in debugfs:
 *  - secvio/info:
 *      * Read: It returns the value of the fuses and SNVS registers which are
 *              readable and related to secvio and tampers
 *      * Write: A write of the format "<hex id> [<hex value 0> <hex value 1>
 *               <hex value 2> <hex value 3> <hex value 4>](<nb values>)"
 *               will write the SNVS register having the provided id with the
 *               values provided (cf SECO ducumentation)
 *  - secvio/enable: State of the IRQ
 *  - secvio/check: Check the state of the security violation and tampers
 *                  and calls notifier
 *  - secvio/clear: Clear the state of all secvio and tampers
 */

/* Includes */
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/nvmem-consumer.h>

#include <linux/firmware/imx/svc/misc.h>
#include <linux/firmware/imx/svc/seco.h>

#include <soc/imx/imx-secvio-sc.h>
#include "imx-secvio-sc-int.h"

int fuse_reader(struct device *dev, u32 id, u32 *value, u8 mul)
{
	struct imx_secvio_sc_data *data = dev_get_drvdata(dev);
	u32 size_to_read = mul * sizeof(u32);
	int ret;

	ret = nvmem_device_read(data->nvmem, id, size_to_read, value);
	if (ret < 0) {
		dev_err(data->dev, "Failed to read fuse %d: %d\n", id, ret);
		return ret;
	}

	if (ret != size_to_read) {
		dev_err(data->dev, "Read only %d instead of %d\n", ret,
			size_to_read);
		return -ENOMEM;
	}

	return 0;
}

int snvs_reader(struct device *dev, u32 id, u32 *value, u8 mul)
{
	int ret;
	u32 *v1, *v2, *v3, *v4, *v5;

	v1 = NULL;
	v2 = NULL;
	v3 = NULL;
	v4 = NULL;
	v5 = NULL;

	switch (mul) {
	case 5:
		v5 = &value[4];
		fallthrough;
	case 4:
		v4 = &value[3];
		fallthrough;
	case 3:
		v3 = &value[2];
		fallthrough;
	case 2:
		v2 = &value[1];
		fallthrough;
	case 1:
		v1 = &value[0];
		break;
	default:
		return -EINVAL;
	}

	ret = call_secvio_config(dev, id, SECVIO_CONFIG_READ, v1, v2, v3, v4,
				 v5, mul);
	if (ret < 0)
		dev_err(dev, "Failed to read snvs reg %d: %d\n", id, ret);

	return ret;
}

int snvs_dgo_reader(struct device *dev, u32 id, u32 *value, u8 mul)
{
	struct imx_secvio_sc_data *data = dev_get_drvdata(dev);
	int ret;

	if (mul != 1)
		return -EINVAL;

	ret = imx_sc_seco_secvio_dgo_config(data->ipc_handle, id,
					    SECVIO_CONFIG_READ, value);
	if (ret)
		dev_err(dev, "Failed to read snvs dgo reg %d: %d\n", id, ret);

	return ret;
}

static const struct imx_secvio_info_entry {
	int (*reader)(struct device *dev, u32 id, u32 *value, u8 mul);
	const char *type;
	const char *name;
	u32 id;
	u8 mul;
} gs_imx_secvio_info_list[] = {
	{fuse_reader, "fuse", "trim", 30, 1},
	{fuse_reader, "fuse", "trim2", 31, 1},
	{fuse_reader, "fuse", "ctrim1", 260, 1},
	{fuse_reader, "fuse", "ctrim2", 261, 1},
	{fuse_reader, "fuse", "ctrim3", 262, 1},
	{fuse_reader, "fuse", "ctrim4", 263, 1},
	{fuse_reader, "fuse", "OSC_CAP", 768, 1},

	{snvs_reader, "snvs", "HPLR",    0x0, 1},
	{snvs_reader, "snvs", "LPLR",    0x34, 1},
	{snvs_reader, "snvs", "HPSICR",  0xc, 1},
	{snvs_reader, "snvs", "HPSVCR",  0x10, 1},
	{snvs_reader, "snvs", "HPSVS",   0x18, 1},
	{snvs_reader, "snvs", "LPSVC",   0x40, 1},
	{snvs_reader, "snvs", "LPTDC",   0x48, 2},
	{snvs_reader, "snvs", "LPSR",    0x4c, 1},
	{snvs_reader, "snvs", "LPTDS",   0xa4, 1},
	{snvs_reader, "snvs", "LPTGFC",  0x44, 3},
	{snvs_reader, "snvs", "LPATCTL", 0xe0, 1},
	{snvs_reader, "snvs", "LPATCLK", 0xe4, 1},
	{snvs_reader, "snvs", "LPATRC1", 0xe8, 2},
	{snvs_reader, "snvs", "LPMKC",   0x3c, 1},
	{snvs_reader, "snvs", "LPSMC",   0x5c, 2},
	{snvs_reader, "snvs", "LPPGD",   0x64, 1},
	{snvs_reader, "snvs", "HPVID",   0xf8, 2},

	{snvs_dgo_reader, "dgo", "Offset",  0x0, 1},
	{snvs_dgo_reader, "dgo", "PUP/PD",  0x10, 1},
	{snvs_dgo_reader, "dgo", "Anatest", 0x20, 1},
	{snvs_dgo_reader, "dgo", "T trim",  0x30, 1},
	{snvs_dgo_reader, "dgo", "Misc",    0x40, 1},
	{snvs_dgo_reader, "dgo", "Vmon",    0x50, 1},
};

struct imx_secvio_sc_info_seq_data {
	struct device *dev;
	const struct imx_secvio_info_entry *list;
	int size;
};

static void *imx_secvio_sc_info_seq_start(struct seq_file *m, loff_t *pos)
{
	struct imx_secvio_sc_info_seq_data *data = m->private;

	/* Check we are not out of bound */
	if (*pos >= data->size)
		return NULL;

	return (void *)pos;
}

static void *imx_secvio_sc_info_seq_next(struct seq_file *m, void *v, loff_t *pos)
{
	/* Increment the counter */
	++*pos;

	/* call the start function which will check the index */
	return imx_secvio_sc_info_seq_start(m, pos);
}

static void imx_secvio_sc_info_seq_stop(struct seq_file *m, void *v)
{
}

static int imx_secvio_sc_info_seq_show(struct seq_file *m, void *v)
{
	struct imx_secvio_sc_info_seq_data *data = m->private;
	const struct imx_secvio_info_entry *e;
	int ret;
	u32 vals[5];
	int idx;

	idx = *(loff_t *)v;
	e = &data->list[idx];

	/* Read the values */
	ret = e->reader(data->dev, e->id, (u32 *)&vals, e->mul);
	if (ret) {
		dev_err(data->dev, "Fail to read %s %s (idx %d)\n", e->type,
			e->name, e->id);
		return 0;
	}

	seq_printf(m, "%5s/%-10s(%.3d):", e->type, e->name, e->id);

	/* Loop over the values */
	for (idx = 0; idx < e->mul; idx++)
		seq_printf(m, " %.8x", vals[idx]);

	seq_puts(m, "\n");

	return 0;
}

static const struct seq_operations imx_secvio_sc_info_seq_ops = {
	.start = imx_secvio_sc_info_seq_start,
	.next  = imx_secvio_sc_info_seq_next,
	.stop  = imx_secvio_sc_info_seq_stop,
	.show  = imx_secvio_sc_info_seq_show,
};

static int imx_secvio_sc_info_open(struct inode *inode, struct file *file)
{
	struct imx_secvio_sc_info_seq_data *data;

	data = __seq_open_private(file, &imx_secvio_sc_info_seq_ops, sizeof(*data));
	if (!data)
		return -ENOMEM;

	data->dev = inode->i_private;
	data->list = gs_imx_secvio_info_list;
	data->size = ARRAY_SIZE(gs_imx_secvio_info_list);

	return 0;
}

static const struct file_operations imx_secvio_sc_info_ops = {
	.owner = THIS_MODULE,
	.open = imx_secvio_sc_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release_private,
};

static void if_debugfs_remove_recursive(void *dentry)
{
	debugfs_remove_recursive(dentry);
}

int imx_secvio_sc_debugfs(struct device *dev)
{
	struct imx_secvio_sc_data *data = dev_get_drvdata(dev);
	struct dentry *dir;
	int ret = 0;

	/* Create a folder */
	dir = debugfs_create_dir(dev_name(dev), NULL);
	if (IS_ERR(dir)) {
		dev_err(dev, "Failed to create dfs dir\n");
		ret = PTR_ERR(dir);
		goto exit;
	}
	data->dfs = dir;

	ret = devm_add_action(dev, if_debugfs_remove_recursive, data->dfs);
	if (ret) {
		dev_err(dev, "Failed to add managed action to disable IRQ\n");
		goto remove_fs;
	}

	/* Create the file to read info and write to reg */
	dir = debugfs_create_file("info", 0x666, data->dfs, dev,
				  &imx_secvio_sc_info_ops);
	if (IS_ERR(dir)) {
		dev_err(dev, "Failed to add info to debugfs\n");
		ret = PTR_ERR(dir);
		goto exit;
	}

exit:
	return ret;

remove_fs:
	debugfs_remove_recursive(data->dfs);
	goto exit;
}
