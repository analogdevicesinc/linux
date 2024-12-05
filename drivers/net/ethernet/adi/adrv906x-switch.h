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
#define SWITCH_CPU_PORT                                 2

#define SWITCH_MAE_TIMEOUT                              50      /* read count */

#define SWITCH_PORT_CFG_PORT                            0x0004
#define   SWITCH_PORT_CFG_PORT_EN                       BIT(0)
#define SWITCH_PORT_CFG_VLAN                            0x0008
#define   SWITCH_PORT_PVID_MASK                         GENMASK(11, 0)
#define SWITCH_PORT_CFG_QINQ                            0x000c
#define   SWITCH_PORT_CFG_DSA_TX_EN                     BIT(17)
#define   SWITCH_PORT_CFG_DSA_RX_EN                     BIT(16)
#define SWITCH_PORT_PCP_REGEN                           0x0010
#define SWITCH_PORT_TRAP_PTP                            0x0084
#define   SWITCH_PORT_TRAP_PTP_EN                       BIT(24)
#define   SWITCH_PORT_TRAP_DSTPORT_MASK                 GENMASK(23, 16)
#define SWITCH_PORT_PCP2IPV                             0x008c
#define SWITCH_MAC_LEARN_EN                             2
#define SWITCH_MAC_FWD_EN                               1

#define SWITCH_MAS_OP_CTRL                              0x0000
#define   SWITCH_MAS_OP_CTRL_TRIGGER                    BIT(8)
#define   SWITCH_MAS_OP_CTRL_OPCODE_MASK                GENMASK(7, 4)
#define   SWITCH_MAS_OP_CTRL_MAC_INSERT                 0
#define   SWITCH_MAS_OP_CTRL_VLAN_ADD                   1
#define   SWITCH_MAS_OP_CTRL_MAC_TABLE_SOFT_FLUSH       2
#define   SWITCH_MAS_OP_CTRL_MAC_TABLE_HARD_FLUSH       3
#define   SWITCH_MAS_OP_CTRL_FLUSH_MAC_TABLE_PENDING    BIT(2)
#define SWITCH_MAS_PORT_MASK1                           0x0004
#define SWITCH_MAS_PORT_MASK2                           0x0008
#define SWITCH_MAS_FDB_MAC_INSERT_1                     0x000c
#define SWITCH_MAS_FDB_MAC_INSERT_2                     0x0010
#define   SWITCH_INSERT_MAC_ADDR_LOW_MASK               GENMASK(31, 16)
#define   SWITCH_INSERT_VALID                           BIT(0)
#define   SWITCH_INSERT_STATIC                          BIT(1)
#define   SWITCH_INSERT_OVERWRITE                       BIT(3)
#define SWITCH_MAS_VLAN_ID                              0x0014

#define SWITCH_SOFT_RESET                               0x0020
#define   SWITCH_ALL_EX_MAE                             BIT(0)

#define SWITCH_PCP_REGEN_VAL                            0x77000000
#define SWITCH_PCP_IPV_MAPPING                          0x10000000
#define SWITCH_PVID                                     1

#define EAPOL_MAC_ADDR                                  0x0180c2000003
#define ESMC_MAC_ADDR                                   0x0180c2000002

struct switch_port {
	unsigned int config_mask;
	void __iomem *reg;
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
	struct switch_port switch_port[SWITCH_MAX_PORT_NUM];
	struct list_head vlan_cfg_list;
	struct mutex vlan_cfg_list_lock; /* VLan cfg list lock */
	void __iomem *reg_match_action;
	void __iomem *reg_switch;
	u16 pvid;
	int err_irqs[2];
	struct switch_isr_args isr_pre_args;
	struct switch_isr_args isr_post_args;
};

int adrv906x_switch_port_enable(struct adrv906x_eth_switch *es, int portid, bool enabled);
int adrv906x_switch_register_irqs(struct adrv906x_eth_switch *es,
				  struct device_node *eth_switch_np);
int adrv906x_switch_probe(struct adrv906x_eth_switch *es, struct platform_device *pdev,
			  int (*isr_pre_func)(void *), int (*isr_post_func)(void *));
int adrv906x_switch_init(struct adrv906x_eth_switch *es);

#endif /* __ADRV906X_SWITCH_H__ */
