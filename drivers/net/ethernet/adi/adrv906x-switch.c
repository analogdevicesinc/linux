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

static struct attribute *adrv906x_switch_attrs[4] = { NULL, NULL, NULL, NULL };

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
		dev_err(&es->pdev->dev, "Timeout when adding VLAN to switch");
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

static int adrv906x_switch_port_vlan_del(struct adrv906x_eth_switch *es, u16 port, u16 vid)
{
	struct vlan_cfg_list *vcl;
	u32 mask;
	int ret;

	if (port + 1 > SWITCH_MAX_PORT_NUM)
		return -EINVAL;

	if (vid >= VLAN_N_VID)
		return -EINVAL;

	vcl = adrv906x_switch_port_vlan_find(es, vid);
	if (!vcl)
		return -EINVAL;

	mask = BIT(port);
	if (vcl->port_mask & mask) {
		mask = vcl->port_mask & ~mask;
		ret = adrv906x_switch_vlan_match_action_sync(es, mask, vid);
		if (ret)
			return ret;

		if (mask) {
			vcl->port_mask = mask;
		} else {
			mutex_lock(&es->vlan_cfg_list_lock);
			list_del(&vcl->list);
			mutex_unlock(&es->vlan_cfg_list_lock);
			devm_kfree(&es->pdev->dev, vcl);
		}
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

	es->pcp_regen_val = pcpmap;
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

	es->pcp_ipv_mapping = pcpmap;
	for (i = 0; i < SWITCH_MAX_PORT_NUM; i++) {
		io = es->switch_port[i].reg_switch_port;
		iowrite32(pcpmap, io + SWITCH_PORT_PCP2IPV);
	}
	return 0;
}

static int adrv906x_get_attr_cmd_tokens(char *inpstr, char *tokens[])
{
	char *token, *cmdstr;
	int i, needed;

	cmdstr = inpstr;
	needed = 0;

	if (strstr(cmdstr, "pvid"))
		needed = 2;

	if (strstr(cmdstr, "vlan"))
		needed = 4;

	if (!needed)
		return -EINVAL;

	for (i = 0 ; i < needed ; i++) {
		do
			token = strsep(&cmdstr, " ");
		while (!strlen(token) && cmdstr[0] != '\0');

		if (!strlen(token))
			return -EINVAL;

		tokens[i] = token;
	}

	return 0;
}

static ssize_t port_vlan_ctrl_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t cnt)
{
	struct adrv906x_eth_switch *es = container_of(attr, struct adrv906x_eth_switch, port_vlan_ctrl_attr);
	char *cmdstr, *orig;
	char *tokens[4];
	u16 port, vid;
	int ret;

	if (cnt < 6 || cnt > 48)
		return -EINVAL;

	cmdstr = kmalloc(cnt, GFP_KERNEL);
	if (!cmdstr)
		return -ENOMEM;

	orig = cmdstr;

	strcpy(cmdstr, buf);
	ret = adrv906x_get_attr_cmd_tokens(cmdstr, tokens);
	if (ret)
		goto free_m;

	if (!strncmp(tokens[0], "vlan", 4)) {
		ret = kstrtou16(tokens[2], 10, &vid);
		if (ret)
			goto free_m;

		ret = kstrtou16(tokens[3], 10, &port);
		if (ret)
			goto free_m;

		if (port >= SWITCH_MAX_PORT_NUM) {
			ret = -EINVAL;
			goto free_m;
		}

		if (!strncmp(tokens[1], "add", 3)) {
			ret = adrv906x_switch_port_vlan_add(es, port, vid);
			if (ret)
				goto free_m;

			ret = cnt;
		}

		if (!strncmp(tokens[1], "del", 3)) {
			ret = adrv906x_switch_port_vlan_del(es, port, vid);
			if (ret)
				goto free_m;

			ret = cnt;
		}
	}

	if (!strncmp(tokens[0], "pvid", 4)) {
		ret = kstrtou16(tokens[1], 10, &vid);
		if (ret)
			return ret;

		ret = adrv906x_switch_port_pvid_set(es, vid);
		if (ret)
			goto free_m;

		ret = cnt;
	}
free_m:
	kfree(orig);

	return ret;
}

static ssize_t port_vlan_ctrl_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct adrv906x_eth_switch *es = container_of(attr, struct adrv906x_eth_switch, port_vlan_ctrl_attr);
	struct vlan_cfg_list *vcl;
	void __iomem *io;
	int char_cnt;
	u32 reg;
	int i;

	io = es->switch_port[0].reg_switch_port;
	reg = ioread32(io + SWITCH_PORT_CFG_VLAN);
	reg &= SWITCH_PVID_MASK;
	char_cnt = sprintf(buf, "%-8s%-4d\n", "pvid:", reg);
	char_cnt += sprintf(buf + char_cnt, "\n");
	char_cnt += sprintf(buf + char_cnt, "%-8s%-4s\n", "vid", "port");

	mutex_lock(&es->vlan_cfg_list_lock);
	list_for_each_entry(vcl, &es->vlan_cfg_list, list) {
		char_cnt += sprintf(buf + char_cnt, "%-5d%-3s", vcl->vlan_id, ":");
		for (i = 0; i < 3; i++) {
			if (vcl->port_mask & BIT(i))
				char_cnt += sprintf(buf + char_cnt, "%-3d", i);
			else
				char_cnt += sprintf(buf + char_cnt, "   ");
		}
		char_cnt += sprintf(buf + char_cnt, "\n");
		if (char_cnt + 16 >= PAGE_SIZE) {
			sprintf(buf + char_cnt, "...\n");
			break;
		}
	}
	mutex_unlock(&es->vlan_cfg_list_lock);

	return char_cnt;
}

static ssize_t pcp_regen_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t cnt)
{
	struct adrv906x_eth_switch *es = container_of(attr, struct adrv906x_eth_switch, pcp_regen_attr);
	u32 val;
	int ret;

	ret = kstrtou32(buf, 16, &val);
	if (ret)
		return -EINVAL;

	adrv906x_switch_port_pcp_regen_set(es, val);

	return cnt;
}

static ssize_t pcp_regen_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct adrv906x_eth_switch *es = container_of(attr, struct adrv906x_eth_switch, pcp_regen_attr);

	return sprintf(buf, "0x%08x\n", es->pcp_regen_val);
}

static ssize_t pcp2ipv_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t cnt)
{
	struct adrv906x_eth_switch *es = container_of(attr, struct adrv906x_eth_switch, pcp2ipv_attr);
	u32 val;
	int ret;

	ret = kstrtou32(buf, 16, &val);
	if (ret)
		return -EINVAL;

	adrv906x_switch_port_ipv_mapping_set(es, val);

	return cnt;
}

static ssize_t pcp2ipv_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct adrv906x_eth_switch *es = container_of(attr, struct adrv906x_eth_switch, pcp2ipv_attr);

	return sprintf(buf, "0x%08x\n", es->pcp_ipv_mapping);
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

void adrv906x_switch_unregister_attr(struct adrv906x_eth_switch *es)
{
	if (!es->attr_group.attrs)
		sysfs_remove_group(&es->pdev->dev.kobj, &es->attr_group);
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
			dev_err(&es->pdev->dev, "failed to get switch[%d] error IRQ", i);

		es->err_irqs[i] = ret;

		ret = devm_request_threaded_irq(&es->pdev->dev, es->err_irqs[i],
						NULL, adrv906x_switch_error_isr,
						IRQF_SHARED | IRQF_ONESHOT,
						dev_name(&es->pdev->dev), es);
		if (ret) {
			dev_err(&es->pdev->dev, "failed to request switch[%d] error IRQ: %d",
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
	es->attr_group.attrs = NULL;

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

	if (of_property_read_u32(eth_switch_np, "pcpregen", &es->pcp_regen_val)) {
		dev_info(dev, "dt: pcpregen property missing");
		return -ENOMEM;
	}

	if (of_property_read_u32(eth_switch_np, "pcp2ipv", &es->pcp_ipv_mapping)) {
		dev_info(dev, "dt: pcp2ipv property missing");
		return -ENOMEM;
	}
	ret = of_property_read_variable_u16_array(eth_switch_np, "vids",
						  es->default_vids, 1, SWITCH_MAX_PCP_PLANE_NUM);
	if (ret < 0) {
		dev_info(dev, "dt: vids property missing");
		return ret;
	}

	if (of_property_read_u16(eth_switch_np, "pvid", &es->pvid)) {
		dev_info(dev, "dt: pvid property missing");
		return -ENOMEM;
	}

	es->enabled = true;

	return 0;
}

int adrv906x_switch_init(struct adrv906x_eth_switch *es)
{
	int i, portid, ret;
	void __iomem *io;
	u32 val;

	adrv906x_switch_port_dsa_tx_enable(es, false);
	adrv906x_switch_port_dsa_rx_enable(es, true);
	adrv906x_switch_port_pcp_regen_set(es, es->pcp_regen_val);
	adrv906x_switch_port_ipv_mapping_set(es, es->pcp_ipv_mapping);

	for (i = 0; i < SWITCH_MAX_PCP_PLANE_NUM; i++) {
		for (portid = 0; portid < SWITCH_MAX_PORT_NUM; portid++) {
			if (!es->default_vids[i])
				continue;
			ret = adrv906x_switch_port_vlan_add(
				es, portid, es->default_vids[i]);
			if (ret)
				return ret;
		}
	}

	ret = adrv906x_switch_port_pvid_set(es, es->pvid);
	if (ret)
		return ret;

	/* Trap PTP from port 1 & 2 to port 3 */
	val = BIT(SWITCH_PTP_ENABLE_BIT) | BIT(SWITCH_PTP_DSTPORT_START_BIT + 2);
	io = es->switch_port[0].reg_switch_port;
	iowrite32(val, io + SWITCH_PORT_TRAP_PTP);
	io = es->switch_port[1].reg_switch_port;
	iowrite32(val, io + SWITCH_PORT_TRAP_PTP);
	adrv906x_switch_port_enable(es, true);

#define __SWITCH_ATTR_RW(_name) {                                                                 \
		es->_name ## _attr.attr.name = __stringify(_name);                                \
		es->_name ## _attr.attr.mode = VERIFY_OCTAL_PERMISSIONS(0644);                    \
		es->_name ## _attr.show = _name ## _show;                                         \
		es->_name ## _attr.store = _name ## _store;                                       \
}

	__SWITCH_ATTR_RW(port_vlan_ctrl);
	__SWITCH_ATTR_RW(pcp_regen);
	__SWITCH_ATTR_RW(pcp2ipv);
	adrv906x_switch_attrs[0] = &es->port_vlan_ctrl_attr.attr;
	adrv906x_switch_attrs[1] = &es->pcp_regen_attr.attr;
	adrv906x_switch_attrs[2] = &es->pcp2ipv_attr.attr;
	es->attr_group.attrs = adrv906x_switch_attrs;
	ret = sysfs_create_group(&es->pdev->dev.kobj, &es->attr_group);

	return ret;
}

MODULE_LICENSE("GPL");
