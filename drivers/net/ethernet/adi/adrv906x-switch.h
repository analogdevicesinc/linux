// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __ADRV906X_SWITCH_H__
#define __ADRV906X_SWITCH_H__

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/io.h>

#define SWITCH_MAX_PORT_NUM                             3
#define SWITCH_MAX_PCP_PLANE_NUM                        4

#define SECONDS_TO_WAIT                                 2
#define ADRV906X_NET_DEV_WAIT                           100     /* msecs */
#define ADRV906X_SWITCH_RESET_TIMEOUT                   50      /* read count */

#define SWITCH_PORT_CFG_PORT                            0x0004
#define SWITCH_PORT_CFG_VLAN                            0x0008
#define SWITCH_PORT_CFG_QINQ                            0x000c
#define SWITCH_PORT_PCP_REGEN                           0x0010
#define SWITCH_PORT_TRAP_PTP                            0x0084
#define SWITCH_PORT_PCP2IPV                             0x008c
#define SWITCH_PTP_ENABLE_BIT                           24
#define SWITCH_PTP_DSTPORT_START_BIT                    16
#define SWITCH_DSA_TX_ENABLE_BIT                        17
#define SWITCH_DSA_RX_ENABLE_BIT                        16
#define SWITCH_MAC_LEARN_EN                             2
#define SWITCH_MAC_FWD_EN                               1
#define SWITCH_PORT_ENABLE_BIT                          0
#define SWITCH_PVID_MASK                                GENMASK(11, 0)

#define SWITCH_MAS_OP_CTRL                              0x0000
#define   SWITCH_MAS_OP_CTRL_OPCODE_MASK                GENMASK(7, 4)
#define   SWITCH_MAS_OP_CTRL_FLUSH_MAC_TABLE            BIT(2)
#define SWITCH_MAS_PORT_MASK1                           0x0004
#define SWITCH_MAS_PORT_MASK2                           0x0008
#define SWITCH_MAS_VLAN_ID                              0x0014
#define SWITCH_MAS_SOFT_RESET                           0x0020
#define   ALL_EX_MAE                                    BIT(0)

struct switch_port {
	unsigned int config_mask;
	void __iomem *reg_switch_port;
};

struct switch_pcp {
	u32 pcpregen;
	u32 pcp2ipv;
	u8 pcp_express;
};

struct vlan_cfg_list {
	struct list_head list;
	u32 port_mask;
	u16 vlan_id;
	u8 pcp_val;
};

struct switch_isr_args {
	int (*func)(void *arg);
	void *arg;
};

struct adrv906x_eth_switch {
	struct platform_device *pdev;
	bool enabled;
	unsigned int pcp_ipv_mapping;
	unsigned int pcp_regen_val;
	struct switch_port switch_port[SWITCH_MAX_PORT_NUM];
	struct list_head vlan_cfg_list;
	struct mutex vlan_cfg_list_lock; /* VLan cfg list lock */
	struct device_attribute port_vlan_ctrl_attr;
	struct device_attribute pcp_regen_attr;
	struct device_attribute pcp2ipv_attr;
	struct attribute_group attr_group;
	void __iomem *reg_match_action;
	void __iomem *reg_switch;
	u16 default_vids[SWITCH_MAX_PCP_PLANE_NUM];
	u16 pvid;
	int err_irqs[2];
	struct switch_isr_args isr_pre_args;
	struct switch_isr_args isr_post_args;
};

int adrv906x_switch_register_irqs(struct adrv906x_eth_switch *es, struct device_node *eth_switch_np);
int adrv906x_switch_register_attr(struct adrv906x_eth_switch *es);
void adrv906x_switch_unregister_attr(struct adrv906x_eth_switch *es);
int adrv906x_switch_probe(struct adrv906x_eth_switch *es, struct platform_device *pdev,
			  int (*isr_pre_func)(void *), int (*isr_post_func)(void *));
int adrv906x_switch_init(struct adrv906x_eth_switch *es);

#endif /* __ADRV906X_SWITCH_H__ */
