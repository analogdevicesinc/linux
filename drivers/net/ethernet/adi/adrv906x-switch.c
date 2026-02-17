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
#include "adrv906x-cmn.h"

#define SWITCH_RECOVERY_NONE         0
#define SWITCH_RECOVERY_VID          BIT(0)
#define SWITCH_RECOVERY_FDB          BIT(1)

static struct attribute *adrv906x_switch_attrs[4] = { NULL, NULL, NULL, NULL };

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
		udelay(1);
	}
	return -ETIMEDOUT;
}

static int adrv906x_switch_vlan_match_action_sync(struct adrv906x_eth_switch *es, u32 mask, u32 vid)
{
	u32 val;
	int ret;

	/* Skip MAE update if all interfaces are down. The PLL may be
	 * unlocked or undergoing reconfiguration, making register access
	 * unreliable. The software state is preserved and will be synced
	 * to MAE during recovery when interfaces come back up.
	 *
	 * Note: Caller must hold es->lock mutex when calling this function.
	 */
	if (!(es->port_enabled_mask & ~BIT(SWITCH_CPU_PORT)))
		return 0;

	ret = adrv906x_switch_wait_for_mae_ready(es);
	if (ret) {
		dev_err(&es->pdev->dev, "vlan add timeout");
		return ret;
	}

	iowrite32(mask, es->reg_match_action + SWITCH_MAS_PORT_MASK1);
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

int adrv906x_switch_vlan_add(struct adrv906x_eth_switch *es, u16 port, u16 vid)
{
	u32 mask;
	int ret;

	if (port >= SWITCH_MAX_PORT_NUM)
		return -EINVAL;

	if (vid == 0 || vid >= VLAN_N_VID - 1)
		return -EINVAL;

	if (vid == es->pvid) {
		dev_info(&es->pdev->dev, "cannot add pvid %u to a port", vid);
		return -EINVAL;
	}

	mutex_lock(&es->lock);
	mask = BIT(port);
	if (!(es->vlan_port_mask[vid] & mask)) {
		mask |= es->vlan_port_mask[vid];
		ret = adrv906x_switch_vlan_match_action_sync(es, mask, vid);
		if (ret) {
			mutex_unlock(&es->lock);
			return ret;
		}
		es->vlan_port_mask[vid] = mask;
	}
	mutex_unlock(&es->lock);

	return 0;
}

int adrv906x_switch_vlan_del(struct adrv906x_eth_switch *es, u16 port, u16 vid)
{
	u32 mask;
	int ret;

	if (port + 1 > SWITCH_MAX_PORT_NUM)
		return -EINVAL;

	if (vid == 0 || vid >= VLAN_N_VID - 1)
		return -EINVAL;

	if (vid == es->pvid) {
		dev_info(&es->pdev->dev, "cannot remove pvid %u from port %u", vid, port);
		return -EINVAL;
	}

	mutex_lock(&es->lock);
	mask = BIT(port);
	if (es->vlan_port_mask[vid] & mask) {
		mask = es->vlan_port_mask[vid] & ~mask;
		ret = adrv906x_switch_vlan_match_action_sync(es, mask, vid);
		if (ret) {
			mutex_unlock(&es->lock);
			return ret;
		}
		es->vlan_port_mask[vid] = mask;
	}
	mutex_unlock(&es->lock);

	return 0;
}

int adrv906x_switch_vlan_add_cpuport(struct adrv906x_eth_switch *es, u16 vid)
{
	/* if host port is already configured for this vid, skip it */
	if (es->vlan_port_mask[vid] & BIT(SWITCH_CPU_PORT))
		return 0;

	return adrv906x_switch_vlan_add(es, SWITCH_CPU_PORT, vid);
}

int adrv906x_switch_vlan_del_cpuport(struct adrv906x_eth_switch *es, u16 vid)
{
	/* if another port is configured for this vid, don't remove vid from host port */
	if (es->vlan_port_mask[vid] & ~BIT(SWITCH_CPU_PORT))
		return 0;
	return adrv906x_switch_vlan_del(es, SWITCH_CPU_PORT, vid);
}

static int adrv906x_switch_pvid_set(struct adrv906x_eth_switch *es, u16 pvid)
{
	int portid, ret;
	u32 val;

	if (pvid == 0 || pvid >= VLAN_N_VID - 1)
		return -EINVAL;

	mutex_lock(&es->lock);
	if (es->vlan_port_mask[pvid]) {
		dev_err(&es->pdev->dev, "cannot set pvid %u, already used", pvid);
		mutex_unlock(&es->lock);
		return -EINVAL;
	}

	ret = adrv906x_switch_vlan_match_action_sync(es, SWITCH_PVID_PORT_MASK, pvid);
	if (ret) {
		dev_err(&es->pdev->dev, "pvid set timed out");
		mutex_unlock(&es->lock);
		return ret;
	}

	es->vlan_port_mask[pvid] = SWITCH_PVID_PORT_MASK;

	for (portid = 0; portid < SWITCH_MAX_PORT_NUM; portid++) {
		val = ioread32(es->switch_port[portid].reg + SWITCH_PORT_CFG_VLAN);
		val &= ~SWITCH_PORT_PVID_MASK;
		val |= pvid;
		iowrite32(val, es->switch_port[portid].reg + SWITCH_PORT_CFG_VLAN);
	}
	mutex_unlock(&es->lock);

	es->pvid = pvid;

	return 0;
}

static void adrv906x_switch_vlan_enable(struct adrv906x_eth_switch *es)
{
	int portid;
	u32 val;

	for (portid = 0; portid < SWITCH_MAX_PORT_NUM; portid++) {
		val = ioread32(es->switch_port[portid].reg + SWITCH_PORT_CFG_VLAN);
		if (es->vlan_enabled)
			val |= SWITCH_PORT_VLAN_EN_MASK;
		else
			val &= ~SWITCH_PORT_VLAN_EN_MASK;
		iowrite32(val, es->switch_port[portid].reg + SWITCH_PORT_CFG_VLAN);
	}
}

static void adrv906x_switch_pcp_regen_set(struct adrv906x_eth_switch *es, u32 pcpmap)
{
	int portid;

	es->pcp_regen_val = pcpmap;
	for (portid = 0; portid < SWITCH_MAX_PORT_NUM; portid++)
		iowrite32(pcpmap, es->switch_port[portid].reg + SWITCH_PORT_PCP_REGEN);
}

static void adrv906x_switch_ipv_mapping_set(struct adrv906x_eth_switch *es, u32 pcpmap)
{
	int portid;

	es->pcp_ipv_mapping = pcpmap;
	for (portid = 0; portid < SWITCH_MAX_PORT_NUM; portid++)
		iowrite32(pcpmap, es->switch_port[portid].reg + SWITCH_PORT_PCP2IPV);
}

static int adrv906x_switch_flush_fdb(struct adrv906x_eth_switch *es)
{
	u32 val;
	int ret;

	mutex_lock(&es->lock);
	ret = adrv906x_switch_wait_for_mae_ready(es);
	if (ret) {
		dev_err(&es->pdev->dev, "fdb flush timeout");
		mutex_unlock(&es->lock);
		return ret;
	}

	val = FIELD_PREP(SWITCH_MAS_OP_CTRL_OPCODE_MASK, SWITCH_MAS_OP_CTRL_MAC_TABLE_HARD_FLUSH) |
	      SWITCH_MAS_OP_CTRL_TRIGGER;
	iowrite32(val, es->reg_match_action + SWITCH_MAS_OP_CTRL);
	mutex_unlock(&es->lock);

	return 0;
}

int adrv906x_switch_add_fdb_entry(struct adrv906x_eth_switch *es, u64 mac_addr, int portid,
				  bool is_static)
{
	u32 val, mac_addr_hi, mac_addr_lo;
	int ret;

	if (portid >= SWITCH_MAX_PORT_NUM)
		return -EINVAL;

	/* Skip MAE update if all interfaces are down. The PLL may be
	 * unlocked or undergoing reconfiguration, making register access
	 * unreliable. The software state is preserved and will be synced
	 * to MAE during recovery when interfaces come back up.
	 */
	mutex_lock(&es->lock);

	if (!(es->port_enabled_mask & ~BIT(SWITCH_CPU_PORT))) {
		mutex_unlock(&es->lock);
		return 0;
	}

	ret = adrv906x_switch_wait_for_mae_ready(es);
	if (ret) {
		dev_err(&es->pdev->dev, "fdb entry add timeout");
		mutex_unlock(&es->lock);
		return ret;
	}

	mac_addr_hi = FIELD_GET(0xFFFFFFFF0000, mac_addr);
	mac_addr_lo = FIELD_GET(0xFFFF, mac_addr);

	iowrite32(BIT(portid), es->reg_match_action + SWITCH_MAS_PORT_MASK1);
	iowrite32(mac_addr_hi, es->reg_match_action + SWITCH_MAS_FDB_MAC_INSERT_1);
	val = FIELD_PREP(SWITCH_INSERT_MAC_ADDR_LOW_MASK, mac_addr_lo) |
	      (is_static ? SWITCH_INSERT_STATIC : 0) | SWITCH_INSERT_VALID;
	iowrite32(val, es->reg_match_action + SWITCH_MAS_FDB_MAC_INSERT_2);
	val = FIELD_PREP(SWITCH_MAS_OP_CTRL_OPCODE_MASK, SWITCH_MAS_OP_CTRL_MAC_INSERT) |
	      SWITCH_MAS_OP_CTRL_TRIGGER;
	iowrite32(val, es->reg_match_action + SWITCH_MAS_OP_CTRL);
	mutex_unlock(&es->lock);

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

	ret = adrv906x_switch_add_fdb_entry(es, EAPOL_MAC_ADDR, SWITCH_CPU_PORT, true);
	if (ret)
		return ret;
	ret = adrv906x_switch_add_fdb_entry(es, ESMC_MAC_ADDR, SWITCH_CPU_PORT, true);
	if (ret)
		return ret;
	if (es->trap_ptp_fwd_en) {
		ret = adrv906x_switch_add_fdb_entry(es, PTP_FWD_ADDR, SWITCH_CPU_PORT, true);
		if (ret)
			return ret;
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
		do {
			token = strsep(&cmdstr, " ");
			if (!token)
				return -EINVAL;
		} while (!strlen(token) && cmdstr[0] != '\0');

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
	struct adrv906x_eth_switch *es =
		container_of(attr, struct adrv906x_eth_switch, port_vlan_ctrl_attr);
	char cmdstr[64];  /* Max input is 48 + null terminator */
	char *tokens[4];
	u16 port, vid;
	int ret;

	if (cnt < 6 || cnt > 48)
		return -EINVAL;

	memset(tokens, 0, sizeof(tokens));
	memcpy(cmdstr, buf, cnt);
	cmdstr[cnt] = '\0';

	ret = adrv906x_get_attr_cmd_tokens(cmdstr, tokens);
	if (ret)
		return ret;

	if (!strncmp(tokens[0], "vlan", 4)) {
		ret = kstrtou16(tokens[2], 10, &vid);
		if (ret)
			return ret;

		ret = kstrtou16(tokens[3], 10, &port);
		if (ret)
			return ret;

		if (port >= SWITCH_MAX_PORT_NUM)
			return -EINVAL;

		if (!strncmp(tokens[1], "add", 3))
			ret = adrv906x_switch_vlan_add(es, port, vid);
		else if (!strncmp(tokens[1], "del", 3))
			ret = adrv906x_switch_vlan_del(es, port, vid);
		else
			ret = -EINVAL;

		return ret ? ret : cnt;
	} else if (!strncmp(tokens[0], "pvid", 4)) {
		ret = kstrtou16(tokens[1], 10, &vid);
		if (ret)
			return ret;

		ret = adrv906x_switch_pvid_set(es, vid);

		return ret ? ret : cnt;
	}

	return -EINVAL;
}

static ssize_t port_vlan_ctrl_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct adrv906x_eth_switch *es =
		container_of(attr, struct adrv906x_eth_switch, port_vlan_ctrl_attr);
	void __iomem *io;
	int char_cnt;
	int i, vid;
	u32 reg;

	io = es->switch_port[0].reg;
	reg = ioread32(io + SWITCH_PORT_CFG_VLAN);
	reg &= SWITCH_PORT_PVID_MASK;
	char_cnt = sprintf(buf, "%-8s%-4d\n", "pvid:", reg);
	char_cnt += sprintf(buf + char_cnt, "\n");
	char_cnt += sprintf(buf + char_cnt, "%-8s%-4s\n", "vid", "port");

	mutex_lock(&es->lock);
	for (vid = 0; vid < VLAN_N_VID; vid++) {
		if (es->vlan_port_mask[vid] == 0)
			continue;

		/* Check space before writing: ~20 bytes per VLAN entry + 4 for "...\n" */
		if (char_cnt + 24 >= PAGE_SIZE) {
			if (char_cnt + 4 < PAGE_SIZE)
				char_cnt += sprintf(buf + char_cnt, "...\n");
			break;
		}

		char_cnt += sprintf(buf + char_cnt, "%-5d%-3s", vid, ":");
		for (i = 0; i < 3; i++) {
			if (es->vlan_port_mask[vid] & BIT(i))
				char_cnt += sprintf(buf + char_cnt, "%-3d", i);
			else
				char_cnt += sprintf(buf + char_cnt, "   ");
		}
		char_cnt += sprintf(buf + char_cnt, "\n");
	}
	mutex_unlock(&es->lock);

	return char_cnt;
}

void adrv906x_switch_unregister_attr(struct adrv906x_eth_switch *es)
{
	if (es->attr_group.attrs)
		if (es->vlan_enabled)
			sysfs_remove_group(&es->pdev->dev.kobj, &es->attr_group);
}

static int __adrv906x_switch_port_enable(struct adrv906x_eth_switch *es, int portid, bool enabled)
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

int adrv906x_switch_port_reset(struct adrv906x_eth_switch *es)
{
	int portid, ret;
	unsigned long flags;

	/* Track switch port reset count */
	es->port_reset_count++;

	ret = es->isr_pre_args.func(es->isr_pre_args.arg);
	if (ret)
		return ret;

	usleep_range(1000, 1100);

	mutex_lock(&es->lock);
	/* Temporarily enable all disabled ports for reset */
	for (portid = 0; portid < SWITCH_MAX_PORT_NUM; portid++) {
		if (!(es->port_enabled_mask & BIT(portid))) {
			__adrv906x_switch_port_enable(es, portid, true);
		}
	}

	adrv906x_cmn_switch_ports_reset(es);

	/* Restore original port enabled/disabled state */
	for (portid = 0; portid < SWITCH_MAX_PORT_NUM; portid++)
		if (!(es->port_enabled_mask & BIT(portid)))
			__adrv906x_switch_port_enable(es, portid, false);

	/* Soft reset */
	iowrite32(SWITCH_ALL_EX_MAE, es->reg_switch + SWITCH_SOFT_RESET);
	iowrite32(0, es->reg_switch + SWITCH_SOFT_RESET);

	mutex_unlock(&es->lock);

	/* Wake up recovery thread to restore VLAN membership */
	spin_lock_irqsave(&es->cmd_flag_lock, flags);
	es->wait_cmd_flag |= SWITCH_RECOVERY_VID | SWITCH_RECOVERY_FDB;
	spin_unlock_irqrestore(&es->cmd_flag_lock, flags);
	wake_up_interruptible(&es->recovery_wq);

	ret = es->isr_post_args.func(es->isr_post_args.arg);
	if (ret)
		return ret;

	return 0;
}

static irqreturn_t adrv906x_switch_error_isr(int irq, void *dev_id)
{
	struct adrv906x_eth_switch *es = (struct adrv906x_eth_switch *)dev_id;
	int ret;

	/* Identify which port triggered the error IRQ */
	if (es->err_irqs[0] == irq) {
		atomic64_inc(&es->err_irq_count[0]);
	} else if (es->err_irqs[1] == irq) {
		atomic64_inc(&es->err_irq_count[1]);
	} else {
		dev_err(&es->pdev->dev, "unknown switch error irq %d (expected %d or %d)",
			irq, es->err_irqs[0], es->err_irqs[1]);
		return IRQ_NONE;
	}

	ret = adrv906x_switch_port_reset(es);
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
	if (portid >= SWITCH_MAX_PORT_NUM)
		return -EINVAL;

	mutex_lock(&es->lock);
	__adrv906x_switch_port_enable(es, portid, enabled);
	if (enabled)
		es->port_enabled_mask |= BIT(portid);
	else
		es->port_enabled_mask &= ~BIT(portid);
	mutex_unlock(&es->lock);

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
		if (ret < 0) {
			dev_err(&es->pdev->dev, "failed to get switch[%d] error irq", i);
			return ret;
		}

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

	return 0;
}

int adrv906x_switch_probe(struct adrv906x_eth_switch *es, struct platform_device *pdev,
			  int (*isr_pre_func)(void *), int (*isr_post_func)(void *), void *isr_arg)
{
	struct device *dev = &pdev->dev;
	struct device_node *eth_switch_np, *switch_port_np;
	u16 default_vids[] = { 2, 3, 4, 5 };
	u32 reg, len, portid;
	int i = 0;
	int ret;

	es->pdev = pdev;
	eth_switch_np = of_get_child_by_name(es->pdev->dev.of_node, "eth_switch");
	if (!eth_switch_np) {
		dev_info(dev, "dt: switch node missing");
		return -ENODEV;
	}

	mutex_init(&es->lock);
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

	es->trap_ptp_fwd_en = of_property_read_bool(eth_switch_np, "trap-ptp-forwardable");

	/* probe the switch ports */
	for_each_child_of_node(eth_switch_np, switch_port_np) {
		if (strcmp(switch_port_np->name, "switch-port"))
			continue;
		of_property_read_u32(switch_port_np, "id", &portid);
		if (portid != i) {
			dev_err(dev, "dt: port id mismatch");
			of_node_put(switch_port_np);
			return -EINVAL;
		}
		/* get switch port register address */
		of_property_read_u32_index(switch_port_np, "reg", 0, &reg);
		of_property_read_u32_index(switch_port_np, "reg", 1, &len);
		es->switch_port[i].reg = devm_ioremap(&es->pdev->dev, reg, len);
		if (!es->switch_port[i].reg) {
			dev_err(dev, "ioremap switch port %d failed!", portid);
			of_node_put(switch_port_np);
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

	ret = of_property_read_variable_u16_array(eth_switch_np, "vids",
						  es->default_vids, 1, NUM_DEFAULT_VIDS);
	if (ret < 0)
		for (i = 0; i < NUM_DEFAULT_VIDS; i++)
			es->default_vids[i] = default_vids[i];

	if (of_property_read_u16(eth_switch_np, "pvid", &es->pvid))
		es->pvid = SWITCH_PVID;

	if (of_property_read_bool(eth_switch_np, "vlan_enabled")) {
		es->vlan_enabled = true;
		dev_info(dev, "vlan is enabled");
	} else {
		es->vlan_enabled = false;
		dev_info(dev, "vlan is disabled");
	}

	es->enabled = true;

	return 0;
}

void adrv906x_switch_cleanup(struct adrv906x_eth_switch *es)
{
	if (es->recovery_task) {
		kthread_stop(es->recovery_task);
		es->recovery_task = NULL;
	}
	cancel_delayed_work(&es->update_stats);
}

static void adrv906x_switch_vlan_membership_recovery(struct adrv906x_eth_switch *es)
{
	struct device *dev = &es->pdev->dev;
	int ret, vid;

	mutex_lock(&es->lock);
	for (vid = 0; vid < VLAN_N_VID; vid++) {
		if (es->vlan_port_mask[vid] == 0)
			continue;
		ret = adrv906x_switch_vlan_match_action_sync(es, es->vlan_port_mask[vid], vid);
		if (ret)
			dev_warn(dev, "failed recovering vlan %d", vid);
	}
	mutex_unlock(&es->lock);
}

static int adrv906x_switch_recovery(void *data)
{
	struct adrv906x_eth_switch *es = (struct adrv906x_eth_switch *)data;
	struct device *dev = &es->pdev->dev;
	unsigned long flags;

	while (!kthread_should_stop()) {
		wait_event_interruptible(es->recovery_wq,
					 es->wait_cmd_flag != SWITCH_RECOVERY_NONE ||
					 kthread_should_stop());

		if (kthread_should_stop())
			break;
		dev_dbg(dev, "switch recovery cmd 0x%x", es->wait_cmd_flag);
		if (es->wait_cmd_flag & SWITCH_RECOVERY_VID) {
			spin_lock_irqsave(&es->cmd_flag_lock, flags);
			es->wait_cmd_flag &= ~SWITCH_RECOVERY_VID;
			spin_unlock_irqrestore(&es->cmd_flag_lock, flags);
			dev_info(dev, "restore vlan membership after switch reset");
			adrv906x_switch_vlan_membership_recovery(es);
		}
		if (es->wait_cmd_flag & SWITCH_RECOVERY_FDB) {
			spin_lock_irqsave(&es->cmd_flag_lock, flags);
			es->wait_cmd_flag &= ~SWITCH_RECOVERY_FDB;
			spin_unlock_irqrestore(&es->cmd_flag_lock, flags);
			dev_info(dev, "restore fdb after switch reset");
			adrv906x_switch_flush_fdb(es);
			adrv906x_switch_packet_trapping_set(es);
		}
	}

	return 0;
}

int adrv906x_switch_init(struct adrv906x_eth_switch *es)
{
	int portid, i, ret;
	struct device *dev = &es->pdev->dev;

	adrv906x_switch_flush_fdb(es);
	adrv906x_switch_dsa_tx_enable(es, false);
	adrv906x_switch_dsa_rx_enable(es, true);
	adrv906x_switch_pcp_regen_set(es, SWITCH_PCP_REGEN_VAL);
	adrv906x_switch_ipv_mapping_set(es, SWITCH_PCP_IPV_MAPPING);

	for (portid = 0; portid < SWITCH_MAX_PORT_NUM; portid++) {
		for (i = 0; i < NUM_DEFAULT_VIDS; i++) {
			if (!es->default_vids[i])
				continue;
			ret = adrv906x_switch_vlan_add(es, portid, es->default_vids[i]);
			if (ret)
				return ret;
		}
	}

	adrv906x_switch_set_mae_age_time(es, AGE_TIME_5MIN_25G);
	if (es->pvid != SWITCH_PVID) {
		ret = adrv906x_switch_pvid_set(es, es->pvid);
		if (ret)
			return ret;
	}

	ret = adrv906x_switch_packet_trapping_set(es);
	if (ret)
		return ret;

	adrv906x_switch_vlan_enable(es);
	for (portid = 0; portid < SWITCH_MAX_PORT_NUM; portid++) {
		ret = adrv906x_switch_port_enable(es, portid, portid == SWITCH_CPU_PORT);
		if (ret)
			return ret;
	}

#define __SWITCH_ATTR_RW(_name) {                                                                 \
		es->_name ## _attr.attr.name = __stringify(_name);                                \
		es->_name ## _attr.attr.mode = VERIFY_OCTAL_PERMISSIONS(0644);                    \
		es->_name ## _attr.show = _name ## _show;                                         \
		es->_name ## _attr.store = _name ## _store;                                       \
}

	__SWITCH_ATTR_RW(port_vlan_ctrl);
	adrv906x_switch_attrs[0] = &es->port_vlan_ctrl_attr.attr;
	es->attr_group.attrs = adrv906x_switch_attrs;
	if (es->vlan_enabled) {
		ret = sysfs_create_group(&es->pdev->dev.kobj, &es->attr_group);
		if (ret) {
			dev_err(dev, "failed to create sysfs group");
			return ret;
		}
	}

	INIT_DELAYED_WORK(&es->update_stats, adrv906x_switch_update_hw_stats);
	mod_delayed_work(system_long_wq, &es->update_stats, msecs_to_jiffies(1000));

	spin_lock_init(&es->cmd_flag_lock);
	init_waitqueue_head(&es->recovery_wq);
	es->wait_cmd_flag = SWITCH_RECOVERY_NONE;
	es->recovery_task = kthread_create(adrv906x_switch_recovery, (void *)es,
					   "adrv906x_switch_recovery_thread");
	if (es->recovery_task) {
		dev_info(dev, "recovery thread created");
		wake_up_process(es->recovery_task);
	} else {
		dev_info(dev, "recovery thread creation failed");
	}

	return 0;
}

MODULE_LICENSE("GPL");
