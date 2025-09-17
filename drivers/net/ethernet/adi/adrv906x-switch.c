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
#include <linux/bitfield.h>
#include <net/rtnetlink.h>
#include "adrv906x-switch.h"

static int adrv906x_switch_wait_for_mae_ready(struct adrv906x_eth_switch *es)
{
	int wait_count = SWITCH_MAE_TIMEOUT;
	u32 val;

	while (wait_count > 0) {
		val = ioread32(es->reg_match_action + SWITCH_MAS_OP_CTRL);
		val &= ~SWITCH_MAS_OP_CTRL_OPCODE_MASK;

		if (!val)
			return 0;

		wait_count--;
		usleep_range(1, 2);
	}
	return -ETIMEDOUT;
}

static int adrv906x_switch_vlan_match_action_sync(struct adrv906x_eth_switch *es, u32 mask, u32 vid)
{
	u32 val;
	int ret;

	ret = adrv906x_switch_wait_for_mae_ready(es);
	if (ret) {
		dev_err(&es->pdev->dev, "vlan add timeout");
		return ret;
	}

	iowrite32(mask, es->reg_match_action + SWITCH_MAS_PORT_MASK1);
	iowrite32(0, es->reg_match_action + SWITCH_MAS_PORT_MASK2);
	iowrite32(vid, es->reg_match_action + SWITCH_MAS_VLAN_ID);
	val = FIELD_PREP(SWITCH_MAS_OP_CTRL_OPCODE_MASK, SWITCH_MAS_OP_CTRL_VLAN_ADD) |
	      SWITCH_MAS_OP_CTRL_TRIGGER;
	iowrite32(val, es->reg_match_action + SWITCH_MAS_OP_CTRL);

	return 0;
}

void adrv906x_switch_set_mae_age_time(struct adrv906x_eth_switch *es, u8 data)
{
	u32 val;

	val = ioread32(es->reg_match_action + SWITCH_MAS_CFG_MAE);
	val &= ~CFG_MAE_AGE_TIME_MASK;
	val |= FIELD_PREP(CFG_MAE_AGE_TIME_MASK, data);
	iowrite32(val, es->reg_match_action + SWITCH_MAS_CFG_MAE);
}

static void adrv906x_switch_dsa_tx_enable(struct adrv906x_eth_switch *es, bool enabled)
{
	int i, val;

	for (i = 0; i < SWITCH_MAX_PORT_NUM; i++) {
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_CFG_QINQ);
		if (enabled)
			val |= SWITCH_PORT_CFG_DSA_TX_EN;
		else
			val &= ~SWITCH_PORT_CFG_DSA_TX_EN;
		iowrite32(val, es->switch_port[i].reg + SWITCH_PORT_CFG_QINQ);
	}
}

static void adrv906x_switch_dsa_rx_enable(struct adrv906x_eth_switch *es, bool enabled)
{
	int i, val;

	for (i = 0; i < SWITCH_MAX_PORT_NUM; i++) {
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_CFG_QINQ);
		if (enabled)
			val |= SWITCH_PORT_CFG_DSA_RX_EN;
		else
			val &= ~SWITCH_PORT_CFG_DSA_RX_EN;
		iowrite32(val, es->switch_port[i].reg + SWITCH_PORT_CFG_QINQ);
	}
}

static struct vlan_cfg_list *adrv906x_switch_vlan_find(struct adrv906x_eth_switch *es, u16 vid)
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

static int adrv906x_switch_vlan_add(struct adrv906x_eth_switch *es, u16 port, u16 vid)
{
	struct vlan_cfg_list *vcl;
	u32 mask;
	int ret;

	if (port >= SWITCH_MAX_PORT_NUM)
		return -EINVAL;

	if (vid >= VLAN_N_VID)
		return -EINVAL;

	vcl = adrv906x_switch_vlan_find(es, vid);
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

static int adrv906x_switch_pvid_set(struct adrv906x_eth_switch *es, u16 pvid)
{
	int portid, ret;
	u32 val;

	if (pvid >= VLAN_N_VID)
		return -EINVAL;

	for (portid = 0; portid < SWITCH_MAX_PORT_NUM; portid++) {
		ret = adrv906x_switch_vlan_add(es, portid, pvid);
		if (ret)
			return ret;
		val = ioread32(es->switch_port[portid].reg + SWITCH_PORT_CFG_VLAN);
		val &= ~SWITCH_PORT_PVID_MASK;
		val |= pvid;
		iowrite32(val, es->switch_port[portid].reg + SWITCH_PORT_CFG_VLAN);
	}

	es->pvid = pvid;

	return 0;
}

static void adrv906x_switch_pcp_regen_set(struct adrv906x_eth_switch *es, u32 pcpmap)
{
	int portid;

	for (portid = 0; portid < SWITCH_MAX_PORT_NUM; portid++)
		iowrite32(pcpmap, es->switch_port[portid].reg + SWITCH_PORT_PCP_REGEN);
}

static void adrv906x_switch_ipv_mapping_set(struct adrv906x_eth_switch *es, u32 pcpmap)
{
	int portid;

	for (portid = 0; portid < SWITCH_MAX_PORT_NUM; portid++)
		iowrite32(pcpmap, es->switch_port[portid].reg + SWITCH_PORT_PCP2IPV);
}

static int adrv906x_switch_default_vlan_set(struct adrv906x_eth_switch *es)
{
	u16 default_vids[] = { 2, 3, 4, 5 };
	int i, portid, ret;

	for (portid = 0; portid < SWITCH_MAX_PORT_NUM; portid++) {
		for (i = 0; i < ARRAY_SIZE(default_vids); i++) {
			ret = adrv906x_switch_vlan_add(es, portid, default_vids[i]);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int adrv906x_switch_add_fdb_entry(struct adrv906x_eth_switch *es, u64 mac_addr, int portid)
{
	u32 val, mac_addr_hi, mac_addr_lo;
	int ret;

	if (portid >= SWITCH_MAX_PORT_NUM)
		return -EINVAL;

	ret = adrv906x_switch_wait_for_mae_ready(es);
	if (ret) {
		dev_err(&es->pdev->dev, "fdb entry add timeout");
		return ret;
	}

	mac_addr_hi = FIELD_GET(0xFFFFFFFF0000, mac_addr);
	mac_addr_lo = FIELD_GET(0xFFFF, mac_addr);

	iowrite32(BIT(portid), es->reg_match_action + SWITCH_MAS_PORT_MASK1);
	iowrite32(mac_addr_hi, es->reg_match_action + SWITCH_MAS_FDB_MAC_INSERT_1);
	val = FIELD_PREP(SWITCH_INSERT_MAC_ADDR_LOW_MASK, mac_addr_lo) |
	      SWITCH_INSERT_VALID | SWITCH_INSERT_STATIC;
	iowrite32(val, es->reg_match_action + SWITCH_MAS_FDB_MAC_INSERT_2);
	val = FIELD_PREP(SWITCH_MAS_OP_CTRL_OPCODE_MASK, SWITCH_MAS_OP_CTRL_MAC_INSERT) |
	      SWITCH_MAS_OP_CTRL_TRIGGER;
	iowrite32(val, es->reg_match_action + SWITCH_MAS_OP_CTRL);

	return 0;
}

static int adrv906x_switch_packet_trapping_set(struct adrv906x_eth_switch *es)
{
	u32 val;
	int ret;

	/* Trap PTP messages from port 0 & 1 to port 2 */
	val = SWITCH_PORT_TRAP_PTP_EN | FIELD_PREP(SWITCH_PORT_TRAP_DSTPORT_MASK, SWITCH_CPU_PORT);
	iowrite32(val, es->switch_port[0].reg + SWITCH_PORT_TRAP_PTP);
	iowrite32(val, es->switch_port[1].reg + SWITCH_PORT_TRAP_PTP);

	ret = adrv906x_switch_add_fdb_entry(es, EAPOL_MAC_ADDR, SWITCH_CPU_PORT);
	if (ret)
		return ret;
	ret = adrv906x_switch_add_fdb_entry(es, ESMC_MAC_ADDR, SWITCH_CPU_PORT);
	if (ret)
		return ret;
	if (es->trap_ptp_fwd_en) {
		ret = adrv906x_switch_add_fdb_entry(es, PTP_FWD_ADDR, SWITCH_CPU_PORT);
		if (ret)
			return ret;
	}

	return 0;
}

void adrv906x_switch_reset_soft(struct adrv906x_eth_switch *es)
{
	int ret;

	iowrite32(SWITCH_ALL_EX_MAE, es->reg_switch + SWITCH_SOFT_RESET);
	iowrite32(0, es->reg_switch + SWITCH_SOFT_RESET);

	ret = adrv906x_switch_wait_for_mae_ready(es);
	if (ret)
		dev_err(&es->pdev->dev, "reset of internal switch failed");
}

static irqreturn_t adrv906x_switch_error_isr(int irq, void *dev_id)
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

static void adrv906x_switch_update_hw_stats(struct work_struct *work)
{
	struct adrv906x_eth_switch *es = container_of(work, struct adrv906x_eth_switch,
						      update_stats.work);
	unsigned int val;
	int i;

	rtnl_lock();

	for (i = 0; i < SWITCH_MAX_PORT_NUM; i++) {
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STATS_CTRL);
		val |= SWITCH_PORT_SNAPSHOT_EN;
		iowrite32(val, es->switch_port[i].reg + SWITCH_PORT_STATS_CTRL);

		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_PKT_FLTR_RX);
		es->port_stats[i].pkt_fltr_rx += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_BYTES_FLTR_RX);
		es->port_stats[i].bytes_fltr_rx += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_PKT_BUF_OVFL);
		es->port_stats[i].pkt_buf_ovfl += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_BYTES_BUF_OVFL);
		es->port_stats[i].bytes_buf_ovfl += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_PKT_ERR);
		es->port_stats[i].pkt_err += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_BYTES_ERR);
		es->port_stats[i].bytes_err += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_DROP_PKT_TX);
		es->port_stats[i].drop_pkt_tx += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_PKT_VOQ_NQN_IPV0);
		es->port_stats[i].ipv0_pkt_voq_nqn += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_PKT_VOQ_NQN_IPV1);
		es->port_stats[i].ipv1_pkt_voq_nqn += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_BYTES_VOQ_NQN_IPV0);
		es->port_stats[i].ipv0_bytes_voq_nqn += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_BYTES_VOQ_NQN_IPV1);
		es->port_stats[i].ipv1_bytes_voq_nqn += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_PKT_VOQ_DQN_IPV0);
		es->port_stats[i].ipv0_pkt_voq_dqn += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_PKT_VOQ_DQN_IPV1);
		es->port_stats[i].ipv1_pkt_voq_dqn += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_BYTES_VOQ_DQN_IPV0);
		es->port_stats[i].ipv0_bytes_voq_dqn += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_BYTES_VOQ_DQN_IPV1);
		es->port_stats[i].ipv1_bytes_voq_dqn += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_PKT_VOQ_DROPN_IPV0);
		es->port_stats[i].ipv0_pkt_voq_dropn += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_PKT_VOQ_DROPN_IPV1);
		es->port_stats[i].ipv1_pkt_voq_dropn += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_BYTES_VOQ_DROPN_IPV0);
		es->port_stats[i].ipv0_bytes_voq_dropn += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_BYTES_VOQ_DROPN_IPV1);
		es->port_stats[i].ipv1_bytes_voq_dropn += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_UCAST_PKT_RX);
		es->port_stats[i].ucast_pkt_rx += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_UCAST_BYTES_RX);
		es->port_stats[i].ucast_bytes_rx += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_UCAST_PKT_TX);
		es->port_stats[i].ucast_pkt_tx += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_UCAST_BYTES_TX);
		es->port_stats[i].ucast_bytes_tx += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_MCAST_PKT_RX);
		es->port_stats[i].mcast_pkt_rx += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_MCAST_BYTES_RX);
		es->port_stats[i].mcast_bytes_rx += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_MCAST_PKT_TX);
		es->port_stats[i].mcast_pkt_tx += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_MCAST_BYTES_TX);
		es->port_stats[i].mcast_bytes_tx += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_BCAST_PKT_RX);
		es->port_stats[i].bcast_pkt_rx += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_BCAST_BYTES_RX);
		es->port_stats[i].bcast_bytes_rx += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_BCAST_PKT_TX);
		es->port_stats[i].bcast_pkt_tx += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_BCAST_BYTES_TX);
		es->port_stats[i].bcast_bytes_tx += val;
		val = ioread32(es->switch_port[i].reg + SWITCH_PORT_STAT_CRD_BUFFER_DROP);
		es->port_stats[i].crd_buffer_drop += val;
	}

	rtnl_unlock();

	mod_delayed_work(system_long_wq, &es->update_stats, msecs_to_jiffies(1000));
}

int adrv906x_switch_port_enable(struct adrv906x_eth_switch *es, int portid, bool enabled)
{
	u32 val;

	if (portid >= SWITCH_MAX_PORT_NUM)
		return -EINVAL;

	val = ioread32(es->switch_port[portid].reg + SWITCH_PORT_CFG_PORT);
	if (enabled)
		val |= SWITCH_PORT_CFG_PORT_EN;
	else
		val &= ~SWITCH_PORT_CFG_PORT_EN;
	iowrite32(val, es->switch_port[portid].reg + SWITCH_PORT_CFG_PORT);

	return 0;
}

int adrv906x_switch_register_irqs(struct adrv906x_eth_switch *es, struct device_node *np)
{
	char err_irq_name[16] = { 0, };
	int ret;
	int i;

	/* Ignore the highest port number which is the CPU port */
	for (i = 0; i < (SWITCH_MAX_PORT_NUM - 1); i++) {
		snprintf(err_irq_name, ARRAY_SIZE(err_irq_name), "%s%d", "switch_error_", i);
		ret = of_irq_get_byname(np, err_irq_name);
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
			  int (*isr_pre_func)(void *), int (*isr_post_func)(void *), void *isr_arg)
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

	es->trap_ptp_fwd_en = of_property_read_bool(eth_switch_np, "trap-ptp-forwardable");

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
		es->switch_port[i].reg = devm_ioremap(&es->pdev->dev, reg, len);
		if (!es->switch_port[i].reg) {
			dev_err(dev, "ioremap switch port %d failed!", portid);
			return -ENOMEM;
		}
		i++;
	}

	es->isr_pre_args.func = isr_pre_func;
	es->isr_pre_args.arg = isr_arg;
	es->isr_post_args.func = isr_post_func;
	es->isr_post_args.arg = isr_arg;
	/* TODO: Add de-allocation in case of error below */
	ret = adrv906x_switch_register_irqs(es, eth_switch_np);
	if (ret)
		return ret;

	es->enabled = true;

	return 0;
}

void adrv906x_switch_cleanup(struct adrv906x_eth_switch *es)
{
	cancel_delayed_work(&es->update_stats);
}

int adrv906x_switch_init(struct adrv906x_eth_switch *es)
{
	int portid, ret;

	adrv906x_switch_dsa_tx_enable(es, false);
	adrv906x_switch_dsa_rx_enable(es, true);
	adrv906x_switch_pcp_regen_set(es, SWITCH_PCP_REGEN_VAL);
	adrv906x_switch_ipv_mapping_set(es, SWITCH_PCP_IPV_MAPPING);
	ret = adrv906x_switch_default_vlan_set(es);
	if (ret)
		return ret;
	adrv906x_switch_set_mae_age_time(es, AGE_TIME_5MIN_25G);
	ret = adrv906x_switch_pvid_set(es, SWITCH_PVID);
	if (ret)
		return ret;
	ret = adrv906x_switch_packet_trapping_set(es);
	if (ret)
		return ret;

	for (portid = 0; portid < SWITCH_MAX_PORT_NUM; portid++) {
		ret = adrv906x_switch_port_enable(es, portid, portid == SWITCH_CPU_PORT);
		if (ret)
			return ret;
	}

	INIT_DELAYED_WORK(&es->update_stats, adrv906x_switch_update_hw_stats);
	mod_delayed_work(system_long_wq, &es->update_stats, msecs_to_jiffies(1000));
	return 0;
}

MODULE_LICENSE("GPL");
