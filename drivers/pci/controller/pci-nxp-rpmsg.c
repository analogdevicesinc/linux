// SPDX-License-Identifier: GPL-2.0+
/*
 * NXP PCI ECAM driver based on RPMSG
 *
 * Copyright 2024 NXP
 */

/* The nxp-pci-ecam-rpmsg transfer protocol:
 *
 *   +---------------+-------------------------------+
 *   |  Byte Offset  |            Content            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       0       |           Category            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |     1 ~ 2     |           Version             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       3       |             Type              |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       4       |           Command             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       5       |           Reserved0           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       6       |           Reserved1           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       7       |           Reserved2           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       8       |           Reserved3           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       9       |           Reserved4           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       10      |        PCIe BUS Number        |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       11      |          PCIe devfn           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |    12 ~ 13    |        Register Offset        |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       14      |             Size              |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       15      |           ERROR Code          |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |    16 ~ 19    |        Register Value         |
 *   +---------------+---+---+---+---+---+---+---+---+
 *
 * Command:
 * 0x01 = Write the register of PCIe config space
 *
 * The Size of reading/writing the register:
 * 0x01 = 1 byte
 * 0x02 = 2 bytes (word)
 * 0x04 = 4 bytes (dword)
 *
 * The definition of ERROR Code:
 * 0x00 = Success
 * 0x01 = Failed
 */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/imx_rpmsg.h>
#include <linux/module.h>
#include <linux/pci-ecam.h>
#include <linux/rpmsg.h>

#include "../pci.h"

#define NXP_RPMSG_TIMEOUT			500 /* unit: ms */

#define NXP_NETC_RPMSG_CATEGORY			0x0c
#define NXP_NETC_RPMSG_VERSION			0x0100
#define NXP_RPMSG_TYPE_REQUEST			0x00
#define NXP_RPMSG_TYPE_RESPONSE			0x01
#define NXP_RPMSG_COMMAND_WRITE			0x01

/* Serialize access to the virtual config space */
static DEFINE_MUTEX(nxp_rpmsg_lock);

struct nxp_rpmsg_msg {
	struct imx_rpmsg_head header;

	/* Payload Start*/
	u8 bus;
	u8 devfn;
	u16 reg;
	u8 size;
	u8 err_code;
	u32 reg_val;
} __packed;

struct nxp_rpmsg_info {
	struct rpmsg_device *rpdev;
	int probe_status;
	struct nxp_rpmsg_msg *msg;
	struct completion cmd_complete;

	u8 cmd;
	u8 bus;
	u8 devfn;
	u16 reg;
};

static struct nxp_rpmsg_info nxp_rpmsg;

static int nxp_rpmsg_xmit(struct nxp_rpmsg_msg *msg)
{
	int err;

	reinit_completion(&nxp_rpmsg.cmd_complete);

	nxp_rpmsg.cmd = msg->header.cmd;
	nxp_rpmsg.bus = msg->bus;
	nxp_rpmsg.devfn = msg->devfn;
	nxp_rpmsg.reg = msg->reg;

	err = rpmsg_send(nxp_rpmsg.rpdev->ept, (void *)msg,
			 sizeof(struct nxp_rpmsg_msg));
	if (err) {
		dev_err(&nxp_rpmsg.rpdev->dev, "rpmsg_send failed: %d\n", err);
		return err;
	}

	err = wait_for_completion_timeout(&nxp_rpmsg.cmd_complete,
					  msecs_to_jiffies(NXP_RPMSG_TIMEOUT));
	if (!err) {
		dev_err(&nxp_rpmsg.rpdev->dev,
			"rpmsg timeout: cmd:%u bus:%u devfn:%u\n",
			msg->header.cmd, msg->bus, msg->devfn);
		return -ETIME;
	}

	if (nxp_rpmsg.msg->err_code) {
		dev_err(&nxp_rpmsg.rpdev->dev,
			"cmd:%u rpmsg error: %u\n",
			nxp_rpmsg.cmd, nxp_rpmsg.msg->err_code);
		return -EREMOTEIO;
	}

	return 0;
}

static int nxp_rpmsg_probe(struct rpmsg_device *rpdev)
{
	if (!rpdev) {
		dev_info(&rpdev->dev, "%s failed, rpdev=NULL\n", __func__);
		nxp_rpmsg.probe_status = -EINVAL;
		return -EINVAL;
	}

	init_completion(&nxp_rpmsg.cmd_complete);

	nxp_rpmsg.rpdev = rpdev;

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
		 rpdev->src, rpdev->dst);

	return 0;
}

static void nxp_rpmsg_remove(struct rpmsg_device *rpdev)
{
	nxp_rpmsg.rpdev = NULL;
	nxp_rpmsg.probe_status = 0;
	dev_info(&rpdev->dev, "NXP PCI ECAM RPMSG driver is removed\n");
}

static int nxp_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	struct nxp_rpmsg_msg *msg = (struct nxp_rpmsg_msg *)data;

	if (msg->header.type != NXP_RPMSG_TYPE_RESPONSE)
		return -EINVAL;

	if (msg->bus != nxp_rpmsg.bus || msg->devfn != nxp_rpmsg.devfn ||
	    msg->reg != nxp_rpmsg.reg || msg->header.cmd != nxp_rpmsg.cmd) {
		dev_err(&rpdev->dev,
			"expected bus:%u devfn:%u reg:%u cmd:%u, received bus:%u devfn:%u reg:%u cmd:%u\n",
			nxp_rpmsg.bus, nxp_rpmsg.devfn, nxp_rpmsg.reg, nxp_rpmsg.cmd,
			msg->bus, msg->devfn, msg->reg, msg->header.cmd);

		return -EINVAL;
	}

	/* Receive Success */
	nxp_rpmsg.msg = msg;

	complete(&nxp_rpmsg.cmd_complete);

	return 0;
}

static struct rpmsg_device_id nxp_rpmsg_id_table[] = {
	{ .name	= "rpmsg-pci-channel" },
	{ },
};

static struct rpmsg_driver nxp_rpmsg_driver = {
	.drv.name	= "nxp-pci-rpmsg",
	.drv.owner	= THIS_MODULE,
	.id_table	= nxp_rpmsg_id_table,
	.probe		= nxp_rpmsg_probe,
	.remove		= nxp_rpmsg_remove,
	.callback	= nxp_rpmsg_cb,
};

static const bool per_bus_mapping = !IS_ENABLED(CONFIG_64BIT);

static int nxp_rpmsg_pci_ecam_add_bus(struct pci_bus *bus)
{
	struct pci_config_window *cfg = bus->sysdata;
	unsigned int bsz = 1 << cfg->bus_shift;
	unsigned int busn = bus->number;
	phys_addr_t start;

	if (!per_bus_mapping)
		return 0;

	if (busn < cfg->busr.start || busn > cfg->busr.end)
		return -EINVAL;

	busn -= cfg->busr.start;
	start = cfg->res.start + busn * bsz;

	cfg->winp[busn] = pci_remap_cfgspace(start, bsz);
	if (!cfg->winp[busn])
		return -ENOMEM;

	return 0;
}

static void nxp_rpmsg_pci_ecam_remove_bus(struct pci_bus *bus)
{
	struct pci_config_window *cfg = bus->sysdata;
	unsigned int busn = bus->number;

	if (!per_bus_mapping || busn < cfg->busr.start || busn > cfg->busr.end)
		return;

	busn -= cfg->busr.start;
	if (cfg->winp[busn]) {
		iounmap(cfg->winp[busn]);
		cfg->winp[busn] = NULL;
	}
}

static void __iomem *nxp_rpmsg_pci_ecam_map_bus(struct pci_bus *bus,
						unsigned int devfn,
						int where)
{
	return pci_ecam_map_bus(bus, devfn, where);
}

static int nxp_rpmsg_pci_config_read(struct pci_bus *bus, unsigned int devfn,
				     int where, int size, u32 *val)
	__must_hold(&pci_lock)
{
	int ret;

	raw_spin_unlock_irq(&pci_lock);
	scoped_guard(mutex, &nxp_rpmsg_lock) {
		ret = pci_generic_config_read(bus, devfn, where, size, val);
	}
	raw_spin_lock_irq(&pci_lock);

	return ret;
}

static int nxp_rpmsg_pci_config_write(struct pci_bus *bus, unsigned int devfn,
				      int where, int size, u32 val, int msg_cate,
				      int msg_ver)
	__must_hold(&pci_lock)
{
	struct nxp_rpmsg_msg *msg __free(kfree);
	int err;

	raw_spin_unlock_irq(&pci_lock);

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (!msg) {
		err = -ENOMEM;
		goto end;
	}

	msg->header.cate = msg_cate;
	msg->header.major = msg_ver >> 8;
	msg->header.minor = msg_ver & 0xff;
	msg->header.type = NXP_RPMSG_TYPE_REQUEST;
	msg->header.cmd = NXP_RPMSG_COMMAND_WRITE;
	msg->bus = bus->number;
	msg->devfn = devfn;
	msg->reg = where;
	msg->size = size;
	msg->reg_val = val;

	scoped_guard(mutex, &nxp_rpmsg_lock) {
		err = nxp_rpmsg_xmit(msg);
	}

end:
	raw_spin_lock_irq(&pci_lock);

	return err;
}

static int netc_rpmsg_pci_config_write(struct pci_bus *bus, unsigned int devfn,
				       int where, int size, u32 val)
{
	return nxp_rpmsg_pci_config_write(bus, devfn, where, size, val,
					  NXP_NETC_RPMSG_CATEGORY,
					  NXP_NETC_RPMSG_VERSION);
}

static const struct pci_ecam_ops netc_rpmsg_pci_ecam_ops = {
	.pci_ops = {
		.add_bus	= nxp_rpmsg_pci_ecam_add_bus,
		.remove_bus	= nxp_rpmsg_pci_ecam_remove_bus,
		.map_bus	= nxp_rpmsg_pci_ecam_map_bus,
		.read		= nxp_rpmsg_pci_config_read,
		.write		= netc_rpmsg_pci_config_write,
	}
};

static const struct of_device_id nxp_rpmsg_pci_of_match[] = {
	{ .compatible = "nxp,netc-rpmsg-pci-host-ecam",
	  .data = &netc_rpmsg_pci_ecam_ops },

	{ },
};
MODULE_DEVICE_TABLE(of, nxp_rpmsg_pci_of_match);

static int nxp_pci_host_probe(struct platform_device *pdev)
{
	if (nxp_rpmsg.probe_status)
		return nxp_rpmsg.probe_status;

	if (!nxp_rpmsg.rpdev)
		return -EPROBE_DEFER;

	return pci_host_common_probe(pdev);
}

static void nxp_pci_host_remove(struct platform_device *pdev)
{
	pci_host_common_remove(pdev);
}

static struct platform_driver nxp_rpmsg_pci_driver = {
	.driver = {
		.name = "nxp-rpmsg-pci-ecam",
		.of_match_table = nxp_rpmsg_pci_of_match,
	},
	.probe = nxp_pci_host_probe,
	.remove_new = nxp_pci_host_remove,
};

static int __init nxp_rpmsg_driver_init(void)
{
	int ret = 0;

	ret = register_rpmsg_driver(&nxp_rpmsg_driver);
	if (ret < 0)
		return ret;

	return platform_driver_register(&nxp_rpmsg_pci_driver);
}

static void __exit nxp_rpmsg_driver_exit(void)
{
	platform_driver_unregister(&nxp_rpmsg_pci_driver);
	unregister_rpmsg_driver(&nxp_rpmsg_driver);
}

module_init(nxp_rpmsg_driver_init);
module_exit(nxp_rpmsg_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NXP PCI ECAM driver over RPMSG");
