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
#define SWITCH_PORT_STATS_NUM                           32
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
#define SWITCH_PORT_STATS_CTRL                          0x0200
#define   SWITCH_PORT_SNAPSHOT_EN                       BIT(0)
#define SWITCH_PORT_STAT_PKT_FLTR_RX                    0x0204
#define SWITCH_PORT_STAT_BYTES_FLTR_RX                  0x0208
#define SWITCH_PORT_STAT_PKT_BUF_OVFL                   0x020C
#define SWITCH_PORT_STAT_BYTES_BUF_OVFL                 0x0210
#define SWITCH_PORT_STAT_PKT_ERR                        0x0214
#define SWITCH_PORT_STAT_BYTES_ERR                      0x0218
#define SWITCH_PORT_STAT_DROP_PKT_TX                    0x021C
#define SWITCH_PORT_STAT_PKT_VOQ_NQN_IPV0               0x0220
#define SWITCH_PORT_STAT_PKT_VOQ_NQN_IPV1               0x0224
#define SWITCH_PORT_STAT_BYTES_VOQ_NQN_IPV0             0x0240
#define SWITCH_PORT_STAT_BYTES_VOQ_NQN_IPV1             0x0244
#define SWITCH_PORT_STAT_PKT_VOQ_DQN_IPV0               0x0260
#define SWITCH_PORT_STAT_PKT_VOQ_DQN_IPV1               0x0264
#define SWITCH_PORT_STAT_BYTES_VOQ_DQN_IPV0             0x0280
#define SWITCH_PORT_STAT_BYTES_VOQ_DQN_IPV1             0x0284
#define SWITCH_PORT_STAT_PKT_VOQ_DROPN_IPV0             0x02A0
#define SWITCH_PORT_STAT_PKT_VOQ_DROPN_IPV1             0x02A4
#define SWITCH_PORT_STAT_BYTES_VOQ_DROPN_IPV0           0x02C0
#define SWITCH_PORT_STAT_BYTES_VOQ_DROPN_IPV1           0x02C4
#define SWITCH_PORT_STAT_UCAST_PKT_RX                   0x02E0
#define SWITCH_PORT_STAT_UCAST_BYTES_RX                 0x02E4
#define SWITCH_PORT_STAT_UCAST_PKT_TX                   0x02E8
#define SWITCH_PORT_STAT_UCAST_BYTES_TX                 0x02EC
#define SWITCH_PORT_STAT_MCAST_PKT_RX                   0x02F0
#define SWITCH_PORT_STAT_MCAST_BYTES_RX                 0x02F4
#define SWITCH_PORT_STAT_MCAST_PKT_TX                   0x02F8
#define SWITCH_PORT_STAT_MCAST_BYTES_TX                 0x02FC
#define SWITCH_PORT_STAT_BCAST_PKT_RX                   0x0300
#define SWITCH_PORT_STAT_BCAST_BYTES_RX                 0x0304
#define SWITCH_PORT_STAT_BCAST_PKT_TX                   0x0308
#define SWITCH_PORT_STAT_BCAST_BYTES_TX                 0x030C
#define SWITCH_PORT_STAT_CRD_BUFFER_DROP                0x0310
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
#define SWITCH_MAS_CFG_MAE                              0x0018
#define   CFG_MAE_AGE_TIME_MASK                         GENMASK(11, 4)
#define   AGE_TIME_5MIN_25G                             0x19
#define   AGE_TIME_5MIN_10G                             0x0A

#define SWITCH_SOFT_RESET                               0x0020
#define   SWITCH_ALL_EX_MAE                             BIT(0)

#define SWITCH_PCP_REGEN_VAL                            0x77000000
#define SWITCH_PCP_IPV_MAPPING                          0x10000000
#define SWITCH_PVID                                     1

#define EAPOL_MAC_ADDR                                  0x0180c2000003
#define ESMC_MAC_ADDR                                   0x0180c2000002
#define PTP_FWD_ADDR                                    0x011b19000000

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

struct switch_port_stats {
	u64 pkt_fltr_rx;
	u64 bytes_fltr_rx;
	u64 pkt_buf_ovfl;
	u64 bytes_buf_ovfl;
	u64 pkt_err;
	u64 bytes_err;
	u64 drop_pkt_tx;
	u64 ipv0_pkt_voq_nqn;
	u64 ipv1_pkt_voq_nqn;
	u64 ipv0_bytes_voq_nqn;
	u64 ipv1_bytes_voq_nqn;
	u64 ipv0_pkt_voq_dqn;
	u64 ipv1_pkt_voq_dqn;
	u64 ipv0_bytes_voq_dqn;
	u64 ipv1_bytes_voq_dqn;
	u64 ipv0_pkt_voq_dropn;
	u64 ipv1_pkt_voq_dropn;
	u64 ipv0_bytes_voq_dropn;
	u64 ipv1_bytes_voq_dropn;
	u64 ucast_pkt_rx;
	u64 ucast_bytes_rx;
	u64 ucast_pkt_tx;
	u64 ucast_bytes_tx;
	u64 mcast_pkt_rx;
	u64 mcast_bytes_rx;
	u64 mcast_pkt_tx;
	u64 mcast_bytes_tx;
	u64 bcast_pkt_rx;
	u64 bcast_bytes_rx;
	u64 bcast_pkt_tx;
	u64 bcast_bytes_tx;
	u64 crd_buffer_drop;
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
	struct switch_port_stats port_stats[SWITCH_MAX_PORT_NUM];
	struct delayed_work update_stats;
	bool trap_ptp_fwd_en;
};

int adrv906x_switch_port_enable(struct adrv906x_eth_switch *es, int portid, bool enabled);
int adrv906x_switch_register_irqs(struct adrv906x_eth_switch *es,
				  struct device_node *eth_switch_np);
int adrv906x_switch_probe(struct adrv906x_eth_switch *es, struct platform_device *pdev,
			  int (*isr_pre_func)(void *), int (*isr_post_func)(void *), void *isr_arg);
int adrv906x_switch_init(struct adrv906x_eth_switch *es);
void adrv906x_switch_set_mae_age_time(struct adrv906x_eth_switch *es, u8 data);
void adrv906x_switch_reset_soft(struct adrv906x_eth_switch *es);
void adrv906x_switch_cleanup(struct adrv906x_eth_switch *es);

#endif /* __ADRV906X_SWITCH_H__ */
