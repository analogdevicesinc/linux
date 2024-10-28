// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/ethtool.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include "adrv906x-switch.h"

#define EAPOL_MAC_ADDR  0x0180c2000003
#define ESMC_MAC_ADDR   0x0180c2000002

static u16 default_vids[SWITCH_MAX_PCP_PLANE_NUM] = { 2, 3, 4, 5 };
static unsigned int pcp_regen_val = 0x77000000;
static unsigned int pcp_ipv_mapping = 0x10000000;
static u16 pvid = 1;

static int adrv906x_switch_vlan_match_action_sync(struct adrv906x_eth_switch *es, u32 mask, u32 vid)
{
	void __iomem *io = es->reg_match_action;
	unsigned long stop = 0;
	u32 reg = 0;

	stop = jiffies + SECONDS_TO_WAIT * HZ;
	do {
		msleep(ADRV906X_NET_DEV_WAIT);
		reg = ioread32(io + SWITCH_MAS_OP_CTRL) & ~SWITCH_MAS_OP_CTRL_OPCODE_MASK;
	} while (reg && time_before(jiffies, stop));

	if (time_after(jiffies, stop)) {
		dev_err(&es->pdev->dev, "timeout when adding vlan to switch");
		return -EIO;
	}

	iowrite32(mask, io + SWITCH_MAS_PORT_MASK1);
	iowrite32(0, io + SWITCH_MAS_PORT_MASK2);
	iowrite32(vid, io + SWITCH_MAS_VLAN_ID);
	iowrite32(0x110, io + SWITCH_MAS_OP_CTRL);

	return 0;
}

static int adrv906x_switch_port_enable(struct adrv906x_eth_switch *es, bool enabled)
{
	void __iomem *io;
	u32 reg;
	int i;

	for (i = 0; i < SWITCH_MAX_PORT_NUM; i++) {
		io = es->switch_port[i].reg_switch_port;
		reg = ioread32(io + SWITCH_PORT_CFG_PORT);
		if (enabled)
			reg |= BIT(SWITCH_PORT_ENABLE_BIT);
		else
			reg &= ~BIT(SWITCH_PORT_ENABLE_BIT);
		iowrite32(reg, io + SWITCH_PORT_CFG_PORT);
	}
	return 0;
}

static int adrv906x_switch_port_dsa_tx_enable(struct adrv906x_eth_switch *es, bool enabled)
{
	void __iomem *io;
	int i, val;

	for (i = 0; i < SWITCH_MAX_PORT_NUM; i++) {
		io = es->switch_port[i].reg_switch_port;
		val = ioread32(io + SWITCH_PORT_CFG_QINQ);
		if (enabled)
			val |= BIT(SWITCH_DSA_TX_ENABLE_BIT);
		else
			val &= ~BIT(SWITCH_DSA_TX_ENABLE_BIT);
		iowrite32(val, io + SWITCH_PORT_CFG_QINQ);
	}
	return 0;
}

static int adrv906x_switch_port_dsa_rx_enable(struct adrv906x_eth_switch *es, bool enabled)
{
	void __iomem *io;
	int i, val;

	for (i = 0; i < SWITCH_MAX_PORT_NUM; i++) {
		io = es->switch_port[i].reg_switch_port;
		val = ioread32(io + SWITCH_PORT_CFG_QINQ);
		if (enabled)
			val |= BIT(SWITCH_DSA_RX_ENABLE_BIT);
		else
			val &= ~BIT(SWITCH_DSA_RX_ENABLE_BIT);
		iowrite32(val, io + SWITCH_PORT_CFG_QINQ);
	}
	return 0;
}

static struct vlan_cfg_list *
adrv906x_switch_port_vlan_find(struct adrv906x_eth_switch *es, u16 vid)
{
	struct vlan_cfg_list *vcl;

	mutex_lock(&es->vlan_cfg_list_lock);
	list_for_each_entry(vcl, &es->vlan_cfg_list, list) {
		if (vcl->vlan_id == vid) {
			mutex_unlock(&es->vlan_cfg_list_lock);
			return vcl;
		}
	}
	mutex_unlock(&es->vlan_cfg_list_lock);

	return NULL;
}

static int adrv906x_switch_port_vlan_add(struct adrv906x_eth_switch *es, u16 port, u16 vid)
{
	struct vlan_cfg_list *vcl;
	u32 mask;
	int ret;

	if (port >= SWITCH_MAX_PORT_NUM)
		return -EINVAL;

	if (vid >= VLAN_N_VID)
		return -EINVAL;

	vcl = adrv906x_switch_port_vlan_find(es, vid);
	if (!vcl) {
		vcl = devm_kzalloc(&es->pdev->dev, sizeof(*vcl), GFP_ATOMIC);
		if (!vcl)
			return -ENOMEM;

		vcl->vlan_id = vid;
		mutex_lock(&es->vlan_cfg_list_lock);
		list_add(&vcl->list, &es->vlan_cfg_list);
		mutex_unlock(&es->vlan_cfg_list_lock);
	}

	mask = BIT(port);
	if (!(vcl->port_mask & mask)) {
		mask |= vcl->port_mask;
		ret = adrv906x_switch_vlan_match_action_sync(es, mask, vid);
		if (ret)
			return ret;

		vcl->port_mask = mask;
	}

	return 0;
}

static int adrv906x_switch_port_pvid_set(struct adrv906x_eth_switch *es, u16 pvid)
{
	void __iomem *io;
	u32 reg = 0;
	int i;

	if (pvid >= VLAN_N_VID)
		return -EINVAL;

	for (i = 0; i < SWITCH_MAX_PORT_NUM; i++) {
		adrv906x_switch_port_vlan_add(es, i, pvid);
		io = es->switch_port[i].reg_switch_port;
		reg = ioread32(io + SWITCH_PORT_CFG_VLAN);
		reg &= ~SWITCH_PVID_MASK;
		reg |= pvid;
		iowrite32(reg, io + SWITCH_PORT_CFG_VLAN);
	}

	es->pvid = pvid;

	return 0;
}

static int adrv906x_switch_port_pcp_regen_set(struct adrv906x_eth_switch *es, u32 pcpmap)
{
	void __iomem *io;
	int i;

	for (i = 0; i < SWITCH_MAX_PORT_NUM; i++) {
		io = es->switch_port[i].reg_switch_port;
		iowrite32(pcpmap, io + SWITCH_PORT_PCP_REGEN);
	}
	return 0;
}

static int adrv906x_switch_port_ipv_mapping_set(struct adrv906x_eth_switch *es, u32 pcpmap)
{
	void __iomem *io;
	int i;

	for (i = 0; i < SWITCH_MAX_PORT_NUM; i++) {
		io = es->switch_port[i].reg_switch_port;
		iowrite32(pcpmap, io + SWITCH_PORT_PCP2IPV);
	}
	return 0;
}

int adrv906x_switch_reset_complete_wait(struct adrv906x_eth_switch *es)
{
	int wait_count = ADRV906X_SWITCH_RESET_TIMEOUT;
	u32 val;

	while (wait_count > 0) {
		val = ioread32(es->reg_match_action + SWITCH_MAS_OP_CTRL);

		if (val & SWITCH_MAS_OP_CTRL_FLUSH_MAC_TABLE)
			return 0;

		wait_count--;
		usleep_range(100, 200);
	}
	return -ETIMEDOUT;
}

void adrv906x_switch_reset_soft(struct adrv906x_eth_switch *es)
{
	u32 val;
	int ret;

	val = ioread32(es->reg_match_action + SWITCH_MAS_SOFT_RESET);

	val |= ALL_EX_MAE;
	iowrite32(val, es->reg_match_action + SWITCH_MAS_SOFT_RESET);

	val &= ~ALL_EX_MAE;
	iowrite32(val, es->reg_match_action + SWITCH_MAS_SOFT_RESET);

	ret = adrv906x_switch_reset_complete_wait(es);
	if (ret)
		dev_err(&es->pdev->dev, "reset of internal switch failed");
}

irqreturn_t adrv906x_switch_error_isr(int irq, void *dev_id)
{
	struct adrv906x_eth_switch *es = (struct adrv906x_eth_switch *)dev_id;
	int ret;

	ret = es->isr_pre_args.func(es->isr_pre_args.arg);
	if (ret)
		return IRQ_NONE;

	usleep_range(1000, 1100);
	/* TODO: Look at re-applying non-register switch configuration */
	adrv906x_switch_reset_soft(es);

	ret = es->isr_post_args.func(es->isr_post_args.arg);
	if (ret)
		return IRQ_NONE;

	return IRQ_HANDLED;
}

int adrv906x_switch_register_irqs(struct adrv906x_eth_switch *es, struct device_node *eth_switch_np)
{
	char err_irq_name[16] = { 0, };
	int ret;
	int i;

	/* Ignore the highest port number which is the CPU port */
	for (i = 0; i < (SWITCH_MAX_PORT_NUM - 1); i++) {
		snprintf(err_irq_name, ARRAY_SIZE(err_irq_name), "%s%d", "switch_error_", i);
		ret = of_irq_get_byname(eth_switch_np, err_irq_name);
		if (ret < 0)
			dev_err(&es->pdev->dev, "failed to get switch[%d] error irq", i);

		es->err_irqs[i] = ret;

		ret = devm_request_threaded_irq(&es->pdev->dev, es->err_irqs[i],
						NULL, adrv906x_switch_error_isr,
						IRQF_SHARED | IRQF_ONESHOT,
						dev_name(&es->pdev->dev), es);
		if (ret) {
			dev_err(&es->pdev->dev, "failed to request switch[%d] error irq: %d",
				i, es->err_irqs[i]);
			return ret;
		}
	}

	return ret;
}

int adrv906x_switch_probe(struct adrv906x_eth_switch *es, struct platform_device *pdev,
			  int (*isr_pre_func)(void *), int (*isr_post_func)(void *))
{
	struct device *dev = &pdev->dev;
	struct device_node *eth_switch_np, *switch_port_np;
	u32 reg, len, portid;
	int i = 0;
	int ret;

	es->pdev = pdev;
	eth_switch_np = of_get_child_by_name(es->pdev->dev.of_node, "eth_switch");
	if (!eth_switch_np) {
		dev_info(dev, "dt: switch node missing");
		return -ENODEV;
	}

	INIT_LIST_HEAD(&es->vlan_cfg_list);
	mutex_init(&es->vlan_cfg_list_lock);

	/* get switch device register address */
	of_property_read_u32_index(eth_switch_np, "reg", 0, &reg);
	of_property_read_u32_index(eth_switch_np, "reg", 1, &len);
	es->reg_switch = devm_ioremap(dev, reg, len);
	if (!es->reg_switch) {
		dev_err(dev, "ioremap switch device failed");
		return -ENOMEM;
	}

	/* get switch match action sync register address */
	of_property_read_u32_index(eth_switch_np, "reg", 2, &reg);
	of_property_read_u32_index(eth_switch_np, "reg", 3, &len);
	es->reg_match_action = devm_ioremap(dev, reg, len);
	if (!es->reg_match_action) {
		dev_err(dev, "ioremap switch mas failed");
		return -ENOMEM;
	}

	/* probe the switch ports */
	for_each_child_of_node(eth_switch_np, switch_port_np) {
		if (strcmp(switch_port_np->name, "switch-port"))
			continue;
		of_property_read_u32(switch_port_np, "id", &portid);
		if (portid != i) {
			dev_err(dev, "dt: port id mismatch");
			return -EINVAL;
		}
		/* get switch port register address */
		of_property_read_u32_index(switch_port_np, "reg", 0, &reg);
		of_property_read_u32_index(switch_port_np, "reg", 1, &len);
		es->switch_port[i].reg_switch_port = devm_ioremap(&es->pdev->dev, reg, len);
		if (!es->switch_port[i].reg_switch_port) {
			dev_err(dev, "ioremap switch port %d failed!", portid);
			return -ENOMEM;
		}
		i++;
	}

	es->isr_pre_args.func = isr_pre_func;
	es->isr_pre_args.arg = (void *)es->pdev;
	es->isr_post_args.func = isr_post_func;
	es->isr_post_args.arg = (void *)es->pdev;
	/* TODO: Add de-allocation in case of error below */
	ret = adrv906x_switch_register_irqs(es, eth_switch_np);
	if (ret)
		return ret;

	es->enabled = true;

	return 0;
}

static int adrv906x_switch_add_fdb_entry(struct device *dev, void __iomem *io,
					 u64 mac_addr, int port)
{
	u32 val, count = 0;

	do {
		val = ioread32(io + SWITCH_MAS_OP_CTRL) & ~SWITCH_MAS_OP_CTRL_OPCODE_MASK;
		if (val) {
			if (count == 2) {
				dev_err(dev, "switch is busy");
				return -EBUSY;
			}
		} else {
			break;
		}
		count++;
	} while (1);
	iowrite32(BIT(2), io + SWITCH_MAS_PORT_MASK1);
	iowrite32((mac_addr & 0xffffffff0000) >> 16, io + SWITCH_MAS_FDB_MAC_INSERT_1);
	iowrite32(((mac_addr & 0xffff) << 16) | SWITCH_INSERT_VALID |
		  SWITCH_INSERT_OVERWRITE | SWITCH_INSERT_STATIC, io + SWITCH_MAS_FDB_MAC_INSERT_2);
	iowrite32(0x100, io + SWITCH_MAS_OP_CTRL);
	return 0;
}

int adrv906x_switch_init(struct adrv906x_eth_switch *es)
{
	struct platform_device *pdev = es->pdev;
	struct device *dev = &pdev->dev;
	int i, portid, ret;
	void __iomem *io;
	u32 val;

	adrv906x_switch_port_dsa_tx_enable(es, false);
	adrv906x_switch_port_dsa_rx_enable(es, true);
	adrv906x_switch_port_pcp_regen_set(es, pcp_regen_val);
	adrv906x_switch_port_ipv_mapping_set(es, pcp_ipv_mapping);

	for (i = 0; i < SWITCH_MAX_PCP_PLANE_NUM; i++) {
		for (portid = 0; portid < SWITCH_MAX_PORT_NUM; portid++) {
			ret = adrv906x_switch_port_vlan_add(es, portid, default_vids[i]);
			if (ret)
				return ret;
		}
	}
	ret = adrv906x_switch_port_pvid_set(es, pvid);
	if (ret)
		return ret;

	/* Trap PTP messages from port 0 & 1 to port 2 */
	val = SWITCH_TRAP_ENABLE | SWITCH_TRAP_DSTPORT_CPU;
	io = es->switch_port[0].reg_switch_port;
	iowrite32(val, io + SWITCH_PORT_TRAP_PTP);
	io = es->switch_port[1].reg_switch_port;
	iowrite32(val, io + SWITCH_PORT_TRAP_PTP);

	/* set up a FDB entry to trap EAPOL messages to port 2 */
	ret = adrv906x_switch_add_fdb_entry(dev, es->reg_match_action, EAPOL_MAC_ADDR,
					    SWITCH_CPU_PORT);
	if (ret)
		return ret;
	/* set up a FDB entry to trap ESMC messages to port 2 */
	ret = adrv906x_switch_add_fdb_entry(dev, es->reg_match_action, ESMC_MAC_ADDR,
					    SWITCH_CPU_PORT);
	if (ret)
		return ret;
	adrv906x_switch_port_enable(es, true);
	return ret;
}

MODULE_LICENSE("GPL");
