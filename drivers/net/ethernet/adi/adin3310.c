// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/* SES chip
 *
 * Copyright 2023 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cache.h>
#include <linux/clocksource.h>
#include <linux/crc8.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if_bridge.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/phy.h>
#include <linux/property.h>
#include <linux/platform_device.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/spi/spi.h>

#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/fs.h>

#include <net/dsa.h>
#include <net/switchdev.h>
#include <asm/unaligned.h>

#define ADIN3310_BUF_SIZE 	1500
#define ADIN3310_ID		0
#define ADIN6310_ID		1

struct adin3310_priv {
	struct platform_device *pdev;
	int nr_ports;
	u8 name[10];
	u8 buf[ADIN3310_BUF_SIZE];
	struct dentry *debug_dir;		// For debugging
	struct dentry *debug_file;		// For debugging
	struct dsa_switch_ops dsa_ops;
	struct dsa_switch dsa_sw;
	const struct adin3310_data *driver_data;
};

struct adin3310_data {
	int nr_ports;
	char name[10];
};

static int adin3310_open(struct inode *inode, struct file *file)
{
	struct adin3310_priv *priv = inode->i_private;

	file->private_data = priv;

	return 0;
}

static ssize_t adin3310_debug_read(struct file *file, char __user *buf, size_t count, loff_t *f_pos)
{
	struct adin3310_priv *priv = file->private_data;

	return simple_read_from_buffer(buf, count, f_pos, priv->buf, ADIN3310_BUF_SIZE);
}

static ssize_t adin3310_debug_write(struct file *file, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct adin3310_priv *priv = file->private_data;
	int ret;

	if (*f_pos > ADIN3310_BUF_SIZE)
		return -EINVAL;

	ret = simple_write_to_buffer(priv->buf, ADIN3310_BUF_SIZE, f_pos, buf, count);
	if (ret < 0)
		return ret;

	priv->buf[ret] = '\0';

	return ret;
}

static const struct file_operations adin3310_debug_fops = {
	.owner		= THIS_MODULE,
	.open		= adin3310_open,
	.read		= adin3310_debug_read,
	.write		= adin3310_debug_write,
};

static int adin3310_init_debug(struct adin3310_priv *priv)
{
	priv->debug_dir = debugfs_create_dir("adin3310", NULL);
	if (IS_ERR(priv->debug_dir))
		return PTR_ERR(priv->debug_dir);

	priv->debug_file = debugfs_create_file("adin3310_message", 0744, priv->debug_dir, priv, &adin3310_debug_fops);
	if (IS_ERR(priv->debug_file)) {
		debugfs_remove(priv->debug_dir);
		return PTR_ERR(priv->debug_file);
	}

	return 0;
}

static void adin3310_remove_debug(struct adin3310_priv *priv)
{
	debugfs_remove(priv->debug_dir);
}

static const struct adin3310_data adin3310_driver_data[] = {
	{ .nr_ports = 3, .name = "adin3310", },
	{ .nr_ports = 6, .name = "adin6310", },
};

static enum dsa_tag_protocol adin3310_get_tag_protocol(struct dsa_switch *ds, int port,
						       enum dsa_tag_protocol mprot)
{
	/*
	 * TODO: Under net/dsa/ will need to add our own tagging protocol.
	 * See ex: net/dsa/tag_edsa.c
	 * Also will need to add to the dsa_tag_protocol enum
	 */
	return DSA_TAG_PROTO_TRAILER;
}

static int adin3310_setup(struct dsa_switch *ds)
{
	return 0;
}

/* TODO: If going for only CPU port approach, implementation of these should be translated
 * in ethernet switch read/write command frames with the appropiate ethernet tag.
 * In order to create devlinks DSA will try to read some PHY ID's of non-CPU ports.
 *
 * Code below will output:
 * [   81.960392] DEBUG: reading: port: 1, reg: 2
 * [   81.960407] DEBUG: reading: port: 1, reg: 3
 * [   81.970738] DEBUG: reading: port: 2, reg: 2
 * [   81.970745] DEBUG: reading: port: 2, reg: 3
 *
 */
static int adin3310_phy_read(struct dsa_switch *ds, int port, int regnum)
{
	pr_info("DEBUG: reading: port: %d, reg: %d\n", port, regnum);
	return 0;
}

static int adin3310_phy_write(struct dsa_switch *ds, int port, int regnum, u16 val)
{
	pr_info("DEBUG: writing: port: %d, reg: %d, val: %u\n", port, regnum, val);
	return 0;
}

static int adin3310_register_dsa(struct adin3310_priv *priv)
{
	struct dsa_switch *dsa_sw = &priv->dsa_sw;

	/* TODO: will need here to add more ops.
	 * NOTE: get_tag_protocol() and setup() are required in order to not crash the kernel.
	 */
	priv->dsa_ops.get_tag_protocol = adin3310_get_tag_protocol;
	priv->dsa_ops.setup = adin3310_setup;
	priv->dsa_ops.phy_read = adin3310_phy_read;
	priv->dsa_ops.phy_write = adin3310_phy_write;

	dsa_sw->num_ports = priv->driver_data->nr_ports;
	dsa_sw->priv = priv;

	dsa_sw->dev = &priv->pdev->dev;
	dsa_sw->ops = &priv->dsa_ops;

	return dsa_register_switch(dsa_sw);
}

static int adin3310_platform_probe(struct platform_device *pdev)
{
	struct adin3310_priv *priv;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);
	priv->pdev = pdev;
	priv->driver_data = device_get_match_data(&pdev->dev);

	/* TODO: Just for debugging during development
	 * Will need to remove this in the end.
	 * The plan is to go to generate 802.1q ADI protocol tagged switch commands.
	 */
	ret = adin3310_init_debug(priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not init debug.\n");
		kfree(priv);

		return ret;
	}

	ret = adin3310_register_dsa(priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register dsa: %d.\n", ret);
		adin3310_remove_debug(priv);
		kfree(priv);

		return ret;
	}

	return 0;
}

static int adin3310_platform_remove(struct platform_device *pdev)
{
	struct adin3310_priv *priv = platform_get_drvdata(pdev);

	adin3310_remove_debug(priv);
	dsa_unregister_switch(&priv->dsa_sw);
	kfree(priv);

	return 0;
}

static const struct of_device_id adin3310_match_table[] = {
	{ .compatible = "adi,adin3310", .data = &adin3310_driver_data[ADIN3310_ID], },
	{ .compatible = "adi,adin6310", .data = &adin3310_driver_data[ADIN6310_ID], },
	{ }
};
MODULE_DEVICE_TABLE(of, adin3310_match_table);

static struct platform_driver adin3310_platform_driver = {
	.probe		= adin3310_platform_probe,
	.remove		= adin3310_platform_remove,
	.driver		= {
		.name		= "adin3310",
		.of_match_table = adin3310_match_table,
	},
};

static int __init adin3310_driver_init(void)
{
	return platform_driver_register(&adin3310_platform_driver);
}

static void __exit adin3310_driver_exit(void)
{
	platform_driver_unregister(&adin3310_platform_driver);
}

module_init(adin3310_driver_init);
module_exit(adin3310_driver_exit);

MODULE_DESCRIPTION("ADIN3310 Network driver");
MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_LICENSE("Dual BSD/GPL");
