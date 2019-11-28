/* SPDX-License-Identifier: GPL-2.0 */
/* Intel FPGA Quad-Speed Ethernet MAC driver
 * Copyright (C) 2019 Intel Corporation. All rights reserved.
 *
 * Contributors:
 *   Roman Bulgakov
 *   Yu Ying Choo
 *   Joyce Ooi
 *
 * Original driver contributed by GlobalLogic.
 */

#ifndef __INTEL_FPGA_QSE_LL_H__
#define __INTEL_FPGA_QSE_LL_H__

#define INTEL_FPGA_QSE_LL_RESOURCE_NAME	"intel_fpga_qse_ll"

#include <linux/bitops.h>
#include <linux/if_vlan.h>
#include <linux/list.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/ptp_clock_kernel.h>
#include "intel_fpga_tod.h"

#define INTEL_FPGA_QSE_SW_RESET_WATCHDOG_CNTR	10000

/* 10G MAC Reset Control registers bit definitions
 */
#define MAC_TX_RESET	BIT(0)
#define MAC_RX_RESET	BIT(8)

/* 10G MAC TX Config and Control registers bit definitions
 */
#define MAC_TX_TRANSFER_CTL_TX_DISABLE		BIT(0)

/* 10G MAC TX Packet Transfer registers bit definitions
 */
#define MAC_TX_TRANSFER_STATUS_BUSY		BIT(8)
#define MAC_TX_TRANSFER_STATUS_RST		BIT(12)

/* TX Pad Inserter register bit definitions
 */
#define MAC_TX_PADINS_CTL_ENA		BIT(0)

/* TX CRC Inserter register bit definitions
 */
#define MAC_TX_CRCINS_CTL_ALWAYS		BIT(0)
#define MAC_TX_CRCINS_CTL_ENA			BIT(1)

#define MAX_TX_PREAMBLE_CTL_ENA			BIT(0)

/* TX Address Inserter register bit definitions
 */
#define MAC_TX_ADDRINS_CTL_ENA			BIT(0)

/* TX MAX Length register bit definitions
 */
#define MAC_TX_FRM_MAX_LENGTH			0xffff

/* TX VLAN Detection register bit definitions
 */
#define MAC_TX_VLAN_DETECT_ENA			BIT(0)

/* TX IPG 10G Average len register bit definitions
 * 0 -> 8 bytes
 * 1 -> 12 bytes
 */
#define MAC_TX_IPG_10G_LEN				BIT(0)

/* TX IPG 1000/100/10 Average len register bit
 * definitions
 * Should be between 8 and 15
 */
#define MAC_TX_IPG_LEN					0xffff

/* TX Pausefreame control register bit definitions
 */
#define MAC_TX_PAUSEFRAME_CTL_DISABLE	0x0
#define MAC_TX_PAUSEFRAME_CTL_XON_ENA	0x1
#define MAC_TX_PAUSEFRAME_CTL_XOFF_ENA	0x2

#define MAC_TX_PAUSEFRAME_QUANTA		0xffff

#define MAC_TX_PAUSEFRAME_HOLDOFF_QUANTA		0xffff

/* TX Pauseframe enable register bit definitions
 */
#define MAC_TX_PAUSEFRAME_ENA		BIT(0)
#define MAC_TX_PAUSEFRAME_REQ_CFG		BIT(1)

/* TX Priority base flow control register bit definitions
 */
#define MAC_TX_PFC_PRIO_ENA_0		BIT(0)
#define MAC_TX_PFC_PRIO_ENA_1		BIT(1)
#define MAC_TX_PFC_PRIO_ENA_2		BIT(2)
#define MAC_TX_PFC_PRIO_ENA_3		BIT(3)
#define MAC_TX_PFC_PRIO_ENA_4		BIT(4)
#define MAC_TX_PFC_PRIO_ENA_5		BIT(5)
#define MAC_TX_PFC_PRIO_ENA_6		BIT(6)
#define MAC_TX_PFC_PRIO_ENA_7		BIT(7)

/* TX unidirectional  control register bit definitions
 */
#define MAX_TX_UNDIR_ENA	BIT(0)

/* 10G MAC RX Config and Control registers bit definitions
 */
#define MAC_RX_TRANSFER_CTL_RX_DISABLE		BIT(0)

/* 10G MAC RX Packet Transfer registers bit definitions
 */
#define MAC_RX_TRANSFER_STATUS_BUSY		BIT(8)
#define MAC_RX_TRANSFER_STATUS_RST		BIT(12)

/* RX Pad/CRC Remover register bit definitions
 * 10 is reserved, only 00, 01, 11 allowed
 */
#define MAC_RX_PADCRC_CTL_CRC_REMOVE		BIT(0)
#define MAC_RX_PADCRC_CTL_ALL_REMOVE		BIT(1)

/* RX CRC Checker register bit definitions
 */
#define MAC_RX_CRCCHECK_CTL_ENA			BIT(1)

/* RX Custom Preamble register bit definitions
 */
#define MAC_RX_CUST_PREAMBLE_FWD_EN		BIT(0)

/* RX Preamble register bit definitions
 */
#define MAC_RX_PREAMBLE_FWD_EN		BIT(0)

/* RX frame control register bit definitions
 */
#define MAC_RX_FRM_CTL_EN_ALLUCAST		BIT(0)
#define MAC_RX_FRM_CTL_EN_ALLMCAST		BIT(1)
#define MAC_RX_FRM_CTL_FWD_CONTROL		BIT(3)
#define MAC_RX_FRM_CTL_FWD_PAUSE		BIT(4)
#define MAC_RX_FRM_CTL_IGNORE_PAUSE		BIT(5)
#define MAC_RX_FRM_CTL_EN_SUPP0			BIT(16)
#define MAC_RX_FRM_CTL_EN_SUPP1			BIT(17)
#define MAC_RX_FRM_CTL_EN_SUPP2			BIT(18)
#define MAC_RX_FRM_CTL_EN_SUPP3			BIT(19)

/* RX MAX Length register bit definitions
 */
#define MAC_RX_FRM_MAX_LENGTH			0xffff

/* RX VLAN Detection register bit definitions
 */
#define MAC_RX_VLAN_DETECT_ENA			BIT(0)

/* RX Priority base flow control register bit definitions
 */
#define MAC_RX_PFC_CTL_PRIO_ENA_0		BIT(0)
#define MAC_RX_PFC_CTL_PRIO_ENA_1		BIT(1)
#define MAC_RX_PFC_CTL_PRIO_ENA_2		BIT(2)
#define MAC_RX_PFC_CTL_PRIO_ENA_3		BIT(3)
#define MAC_RX_PFC_CTL_PRIO_ENA_4		BIT(4)
#define MAC_RX_PFC_CTL_PRIO_ENA_5		BIT(5)
#define MAC_RX_PFC_CTL_PRIO_ENA_6		BIT(6)
#define MAC_RX_PFC_CTL_PRIO_ENA_7		BIT(7)
#define MAC_RX_PFC_CTL_CTL_FRM_FWD_ENA	BIT(16)

/* Timestamp TX Assymetry register bit definitions
 */
#define MAC_TIMESTAMP_TX_ASSYMETRY_VAL	0x1ffff
#define MAC_TIMESTAMP_TX_ASSYMETRY_DIR	BIT(17)

/* MAC ECC Enable and Status register bit definitions
 * Enables external signalling of det / corr of ECC
 * errors
 */
#define MAC_ECC_ERR_DET_CORR_ENA		BIT(0)
#define MAC_ECC_ERR_DET_UNCORR_ENA		BIT(1)
#define MAC_ECC_STATUS_DET_CORR			BIT(0)
#define MAC_ECC_STATUS_DET_UNCORR		BIT(1)

/* Statistics counters control register bit definitions
 */
#define MAC_STATS_CTL_CLEAR			BIT(0)

/* Flow Control defines */
#define FLOW_OFF	0
#define FLOW_RX		1
#define FLOW_TX		2
#define FLOW_ON		(FLOW_TX | FLOW_RX)

/* PHY Speed
 */
#define PHY_ETH_SPEED_1000			0x0
#define PHY_ETH_SPEED_2500			0x1
#define PHY_ETH_SPEED_10000			0x3

/* PHY Reconfiguration
 */
#define PHY_RECONFIG_BUSY			BIT(0)
#define PHY_RECONFIG_START			BIT(16)

/* Timestamp Registers
 */
struct intel_fpga_qse_ll_timestamp_ctrl {
	u32 period_10g;
	u32 reserved1[1];
	u32 fns_adjustment_10g;
	u32 reserved3[1];
	u32 ns_adjustment_10g;
	u32 reserved_5[3];
	u32 period_mult_speed;
	u32 reserved_9[1];
	u32 fns_adjustment_mult_speed;
	u32 reserved_b[1];
	u32 ns_adjustment_mult_speed;
	u32 reserved_d[3];
};

/* Altera 10G Ethernet MAC RX/TX statistics counters within MAC register space.
 * To read the statistic counters, read the LSB before reading the MSB.
 */
struct intel_fpga_qse_ll_mac_stats {
	u32 clear;
	u32 reserved_1[0x1];
	u32 frames_ok_lsb;
	u32 frames_ok_msb;
	u32 frames_err_lsb;
	u32 frames_err_msb;
	u32 frames_crc_err_lsb;
	u32 frames_crc_err_msb;
	u32 octets_ok_lsb;
	u32 octets_ok_msb;
	u32 pause_mac_ctrl_frames_lsb;
	u32 pause_mac_ctrl_frames_msb;
	u32 if_errors_lsb;
	u32 if_errors_msb;
	u32 unicast_frames_ok_lsb;
	u32 unicast_frames_ok_msb;
	u32 unicast_frames_err_lsb;
	u32 unicast_frames_err_msb;
	u32 multicast_frames_ok_lsb;
	u32 multicast_frames_ok_msb;
	u32 multicast_frames_err_lsb;
	u32 multicast_frames_err_msb;
	u32 broadcast_frames_ok_lsb;
	u32 broadcast_frames_ok_msb;
	u32 broadcast_frames_err_lsb;
	u32 broadcast_frames_err_msb;
	u32 eth_stats_oct_lsb;
	u32 eth_stats_oct_msb;
	u32 eth_stats_pkts_lsb;
	u32 eth_stats_pkts_msb;
	u32 eth_stats_undersize_pkts_lsb;
	u32 eth_stats_undersize_pkts_msb;
	u32 eth_stats_oversize_pkts_lsb;
	u32 eth_stats_oversize_pkts_msb;
	u32 eth_stats_pkts_64_oct_lsb;
	u32 eth_stats_pkts_64_oct_msb;
	u32 eth_stats_pkts_65to127_oct_lsb;
	u32 eth_stats_pkts_65to127_oct_msb;
	u32 eth_stats_pkts_128to255_oct_lsb;
	u32 eth_stats_pkts_128to255_oct_msb;
	u32 eth_stats_pkts_256to511_oct_lsb;
	u32 eth_stats_pkts_256to511_oct_msb;
	u32 eth_stats_pkts_512to1023_oct_lsb;
	u32 eth_stats_pkts_512to1023_oct_msb;
	u32 eth_stats_pkts_1024to1518_oct_lsb;
	u32 eth_stats_pkts_1024to1518_oct_msb;
	u32 eth_stats_pkts_1519tox_oct_lsb;
	u32 eth_stats_pkts_1519tox_oct_msb;
	u32 eth_stats_fragments_lsb;
	u32 eth_stats_fragments_msb;
	u32 eth_stats_jabbers_lsb;
	u32 eth_stats_jabbers_msb;
	u32 eth_stats_crc_err_lsb;
	u32 eth_stats_crc_err_msb;
	u32 unicast_mac_ctrl_frames_lsb;
	u32 unicast_mac_ctrl_frames_msb;
	u32 multicast_mac_ctrl_frames_lsb;
	u32 multicast_mac_ctrl_frames_msb;
	u32 broadcast_mac_ctrl_frames_lsb;
	u32 broadcast_mac_ctrl_frames_msb;
	u32 pfc_mac_ctrl_frames_lsb;
	u32 pfc_mac_ctrl_frames_msb;
};

/* Altera 10G Low Latency Ethernet MAC register space. Note that some of these
 * registers may or may not be present depending upon options chosen by the user
 * when the core was configured and built. Please consult the Altera 10G MAC
 * User Guide for details.
 */
struct intel_fpga_qse_ll_mac {
	/* Reserved 0x0 to 0xf words */
	u32 reserved_0[16];
	/* 32-bit primary MAC address word 0 bits 0 to 31 of the primary
	 * MAC address
	 */
	u32 primary_mac_addr0;						//0x10
	/* 32-bit primary MAC address word 1 bits 32 to 47 of the primary
	 * MAC address
	 */
	u32 primary_mac_addr1;						//0x11
	/* Reserved 0x12 to 0x1E words */
	u32 reserved_12[13];
	/* TX and RX Datapath reset control. TX: bit 0 RX bit 8 */
	u32 mac_reset_control;						//0x1f
	/* Enable / Disable TX data path */
	u32 tx_packet_control;						//0x20
	u32 reserved_21[1];						//0x21
	/* TX Datapath status, Idle: bit 8 Reset: bit 12 */
	u32 tx_transfer_status;						//0x22
	u32 reserved_23[1];						//0x23
	/* Control pad insertion to ensure minimum packet size is met */
	u32 tx_pad_control;						//0x24
	u32 reserved_25[1];						//0x25
	/* Enable CRC insertion into dataframe Note bit 1 should always be 1 */
	u32 tx_crc_control;						//0x26
	u32 reserved_27[1];						//0x27
	/* Bypass control for preamble insertion into the data frame */
	u32 tx_preamble_control;					//0x28
	u32 reserved_29[1];						//0x29
	/* Override source MAC address with address in primary_mac_address */
	u32 tx_src_addr_override;					//0x2a
	u32 reserved_2b[1];						//0x2b
	/* MAX MTU */
	u32 tx_frame_maxlength;						//0x2c
	/*Disable VLAN Tag detection */
	u32 tx_vlan_detection;						//0x2d
	/* Set the average Inter Packet Gap for 10Gb 0: 8 bytes 1: 12 bytes */
	u32 tx_ipg_10g;							//0x2e
	/* IPG settings for 10/100/1000, set between 8-15 on bits[3:0] */
	u32 tx_ipg_10m_100m_1g;						//0x2f
	/* Reserved 0x30 to 0x3D words */
	u32 reserved_30[14];
	/* 36 bit counter for tx buffer underflow */
	u32 tx_underflow_counter0;					//0x3e
	u32 tx_underflow_counter1;					//0x3f
	/* Pauseframe trigger condition confifuration */
	u32 tx_pauseframe_control;					//0x40
	u32 reserved_41[1];						//0x41
	/* Quana value used for XOFF generation */
	u32 tx_pauseframe_quanta;					//0x42
	/* Gap between consecutive XOFF frames */
	u32 tx_pauseframe_holdoff_quanta;				//0x43
	u32 tx_pauseframe_enable;					//0x44
	u32 reserved_45[1];						//0x45
	u32 tx_pfc_priority_enable;					//0x46
	u32 reserved_47[1];						//0x47
	/* Specifies pause quanta per queue */
	u32 pfc_pause_quanta_0;						//0x48
	u32 pfc_pause_quanta_1;						//0x49
	u32 pfc_pause_quanta_2;						//0x4a
	u32 pfc_pause_quanta_3;						//0x4b
	u32 pfc_pause_quanta_4;						//0x4c
	u32 pfc_pause_quanta_5;						//0x4d
	u32 pfc_pause_quanta_6;						//0x4e
	u32 pfc_pause_quanta_7;						//0x4f
	/* Reserved 0x50 to 0x57 words */
	u32 reserved_50[8];
	/* Specifies gap between XOFF pause frames per queue */
	u32 pfc_holdoff_quanta_0;					//0x58
	u32 pfc_holdoff_quanta_1;					//0x59
	u32 pfc_holdoff_quanta_2;					//0x5a
	u32 pfc_holdoff_quanta_3;					//0x5b
	u32 pfc_holdoff_quanta_4;					//0x5c
	u32 pfc_holdoff_quanta_5;					//0x5d
	u32 pfc_holdoff_quanta_6;					//0x5e
	u32 pfc_holdoff_quanta_7;					//0x5f
	/* Reserved 0x60 to 0x6f words */
	u32 reserved_60[16];
	/* unidirectional feature and fault control */
	u32 tx_unidir_control;						//0x70
	/* Reserved 0x71 to 0x9f words */
	u32 reserved_71[47];
	/* Enable / Disable RX data path */
	u32 rx_transfer_control;					//0xa0
	u32 reserved_a1[1];						//0xa1
	/* RX Datapath status, Idle: bit 8 Reset: bit 12 */
	u32 rx_transfer_status;						//0xa2
	u32 reserved_a3[1];						//0xa3
	/* Padding and CRC removal on receive */
	u32 rx_padcrc_control;						//0xa4
	u32 reserved_a5[1];						//0xa5
	/* Check CRC on receive */
	u32 rx_crccheck_control;					//0xa6
	u32 reserved_a7[1];						//0xa7
	/* Enable forwarding of custom preable*/
	u32 rx_custom_preamble_forward;					//0xa8
	u32 reserved_a9[1];						//0xa9
	/* Preamble passthru enable on receive */
	u32 rx_preamble_control;					//0xaa
	u32 reserved_ab[1];						//0xab
	/* Enable/Disable reception of UCAST MCAST and pause frames */
	u32 rx_frame_control;						//0xac
	u32 reserved_ad[1];						//0xad
	/* Maximum allowable frame size */
	u32 rx_frame_maxlength;						//0xae
	/* Enable RX VLAN detection */
	u32 rx_vlan_detection;						//0xaf
	/* 4 supplementary RX MAC addresses */
	u32 rx_frame_spaddr0_0;						//0xb0
	u32 rx_frame_spaddr0_1;						//0xb1
	u32 rx_frame_spaddr1_0;						//0xb2
	u32 rx_frame_spaddr1_1;						//0xb3
	u32 rx_frame_spaddr2_0;						//0xb4
	u32 rx_frame_spaddr2_1;						//0xb5
	u32 rx_frame_spaddr3_0;						//0xb6
	u32 rx_frame_spaddr3_1;						//0xb7
	/* Reserved 0xB8 to 0xBF words */
	u32 reserved_b8[8];
	/* Enable priority based flow control */
	u32 rx_pfc_control;						//0xc0
	/* Reserved 0xC1 to 0xFB words */
	u32 reserved_c1[59];
	/* RX packet overflow counter counts truncated pkts */
	u32 rx_pktovrflow_error0;					//0xfc
	u32 rx_pktovrflow_error1;					//0xfd
	/* RX packet overflow counter counts dropped pkts */
	u32 rx_pktovrflow_eth_stats_dropevents0;			//0xfe
	u32 rx_pktovrflow_eth_stats_dropevents1;			//0xff
	/* Timestamp Registers */
	/* TX 0x100 to 0x10f */
	struct intel_fpga_qse_ll_timestamp_ctrl tx_timestamp_ctrl;
	/* Specifies value and direction of assymetric arithmetic operation */
	u32 tx_asymmetry;						//0x110
	/* Reserved 0x111 to 0x11f words */
	u32 reserved_111[15];
	/* RX 0x120 to 0x12f */
	struct intel_fpga_qse_ll_timestamp_ctrl rx_timestamp_ctrl;
	/* Reserved 0x130 to 0x13f words */
	u32 reserved_130[16];
	struct intel_fpga_qse_ll_mac_stats tx_stats;
	/* Reserved 0x17e to 0x1bf words */
	u32 reserved_17e[66];
	struct intel_fpga_qse_ll_mac_stats rx_stats;
	/* Reserved 0x1fe to 0x23f words */
	u32 reserved_1fe[66];
	/* ECC status indicates error and correction status */
	u32 ecc_status;							//0x240
	/* ECC Enable */
	u32 ecc_enable;							//0x241
};

#define qse_csroffs(a)	(offsetof(struct intel_fpga_qse_ll_mac, a))
#define qse_rxstat_csroffs(a) \
	(offsetof(struct intel_fpga_qse_ll_mac, rx_stats.a))
#define qse_txstat_csroffs(a) \
	(offsetof(struct intel_fpga_qse_ll_mac, tx_stats.a))
#define qse_rx_ts_csroffs(a) \
	(offsetof(struct intel_fpga_qse_ll_mac, rx_timestamp_ctrl.a))
#define qse_tx_ts_csroffs(a) \
	(offsetof(struct intel_fpga_qse_ll_mac, tx_timestamp_ctrl.a))

/* Altera 10GBASE-KR register space.
 */
struct arria10_10gbase_kr {				// word addresses
	u32 hssi_regs[4096];				// 0x0 to 0x3ff
	u32 reserved_400[68];				// 0x400 to 0x443

	/* PMA Regs */
	u32 pma_reset;					// 0x444
	u32 reserved_445[28];				// 0x445 to 0x460
	u32 phy_serial_loopback;			// 0x461
	u32 reserved[2];				// 0x462 to 0x463
	u32 pma_rx_set_locktodata;			// 0x464
	u32 pma_rx_set_locktoref;			// 0x465
	u32 pma_rx_is_lockedtodata;			// 0x466
	u32 pma_rx_is_lockedtoref;			// 0x467
	u32 reserved_468[24];				// 0x468 to 0x479

	/* Enhanced PCS Regs */
	u32 pcs_indirect_addr;				// 0x480
	u32 pcs_rclr_error_count;			// 0x481
	u32 pcs_status;					// 0x482
	u32 reserved_483[13];				// 0x483 to 0x48f
	u32 pcs_control_1g;				// 0x490
	u32 pcs_status_1g;				// 0x491
	u32 pcs_dev_ability_1g;				// 0x492
	u32 reserved_493[21];				// 0x493 to 0x4a7

	/* 1G Data mode */
	u32 pma_electrical_setting;			// 0x4a8
	u32 pma_status;					// 0x4a9
	u32 reserved_4aa[6];				// 0x4aa to 0x4af

	/* KR Registers */
	u32 seq_control;				// 0x4b0
	u32 seq_status;					// 0x4b1
	u32 kr_fec_tx_error_insert;			// 0x4b2
	u32 reserved_4b3[2];				// 0x4b3 to 0x4b4
	u32 reserved_40g[10];				// 0x4b5 to 0x4bf
	u32 an_control;					// 0x4c0
	u32 an_control_expand;				// 0x4c1
	u32 an_status;					// 0x4c2
	u32 an_base_page_regs0;				// 0x4c3
	u32 an_base_page_regs1;				// 0x4c4
	u32 an_base_page_regs2;				// 0x4c5
	u32 an_base_page_regs3;				// 0x4c6
	u32 an_base_page_regs4;				// 0x4c7
	u32 an_base_page_regs5;				// 0x4c8
	u32 an_base_page_regs6;				// 0x4c9
	u32 an_base_page_regs7;				// 0x4ca
	u32 an_received_ability;			// 0x4cb
	u32 reserve_4cc[4];				// 0x4cc to 0x4cf
	u32 link_train_control;				// 0x4d0
	u32 link_train_control_ext;			// 0x4d1
	u32 link_train_status;				// 0x4d2
	u32 ber_time;					// 0x4d3
	u32 ld_status;					// 0x4d4
	u32 lt_setting;					// 0x4d5
	u32 pma_settings;				// 0x4d6
	u32 reserved_40g_ext[10];			// 0x4d7 to 0x4ff
};

#define arria10_10gbase_kr_csroffs(a) (offsetof(struct arria10_10gbase_kr, a))

/* PHY Reconfiguration Controller Address Space
 */
struct intel_fpga_phy_reconfig {
	u32 logical_chan_num;				//0x0
	u32 speed_reconfig;				//0x4
	u32 reconfig_busy;				//0x8
};

#define phy_csroffs(a)	(offsetof(struct intel_fpga_phy_reconfig, a))

/* RX FIFO Address Space
 */
struct intel_fpga_rx_fifo {
	u32 fill_level;					//0x00
	u32 reserved;					//0x04
	u32 almost_full_threshold;			//0x08
	u32 almost_empty_threshold;			//0x0C
	u32 cut_through_threshold;			//0x10
	u32 drop_on_error;				//0x14
};

#define rx_fifo_csroffs(a)	(offsetof(struct intel_fpga_rx_fifo, a))

/* Altera Stratix Multi-rate Ethernet PHY register space.
 */
struct stratix10_usxgmii_addr {
	u32 usxgmii_ctrl;		// USXGMII Reg Control 0x400
	u32 usxgmii_status;		// USXGMII Reg Status 0x401
	u32 reserved_402[3];		// Reserved 0x402:0x404
	u32 usxgmii_partner_ability;	// USXGMII Reg Partner Ability 0x405
	u32 reserved_406[5];		// Reserved 0x406:0x411
	u32 usxgmii_link_timer;		// USXGMII link timer 0x412
	u32 reserved_413[78];		// Reserved 0x413:0x460
	u32 pma_rx_is_lockedtodata;	// USXGMII Serial Loopback 0x461
};

struct intel_fpga_qse_private {
	struct net_device *dev;
	struct device *device;
	struct napi_struct napi;

	/* Phylink */
	struct phylink *phylink;
	struct phylink_config phylink_config;

	/* MAC address space */
	struct intel_fpga_qse_ll_mac __iomem *mac_dev;

	/* Shared DMA structure */
	struct altera_dma_private dma_priv;

	/* Shared PTP structure */
	struct intel_fpga_tod_private ptp_priv;
	u32 ptp_enable;

	/* FIFO address space */
	struct intel_fpga_rx_fifo __iomem *rx_fifo;

	/* PHY transceiver (XCVR) address space */
	void __iomem *xcvr_ctrl;

	/* PHY reconfiguration controller address space */
	struct intel_fpga_phy_reconfig __iomem *phy_reconfig_csr;

	/* Interrupts */
	u32 tx_irq;
	u32 rx_irq;

	/* RX/TX MAC FIFO configs */
	u32 tx_fifo_depth;
	u32 rx_fifo_depth;
	u32 rx_fifo_almost_full;
	u32 rx_fifo_almost_empty;
	u32 max_mtu;

	/* Hash filter settings */
	u32 hash_filter;
	u32 added_unicast;

	/* MAC command_config register protection */
	spinlock_t mac_cfg_lock;

	/* Tx path protection */
	spinlock_t tx_lock;

	/* Rx DMA & interrupt control protection */
	spinlock_t rxdma_irq_lock;

	/* MAC flow control */
	unsigned int flow_ctrl;
	unsigned int pause;

	/* PMA digital delay */
	u32 tx_pma_delay_ns;
	u32 rx_pma_delay_ns;
	u32 tx_pma_delay_fns;
	u32 rx_pma_delay_fns;

	/* PHY */
	void __iomem *mac_extra_control;
	int phy_addr;		/* PHY's MDIO address, -1 for autodetection */
	phy_interface_t phy_iface;
	struct mii_bus *mdio;
	int oldspeed;
	int oldduplex;
	int oldlink;

	/* ethtool msglvl option */
	u32 msg_enable;
	struct altera_dmaops *dmaops;
};

/* XCVR 10GBASE-R registers bit definitions
 */
#define XCVR_10GBASE_R_CHANNEL			0
#define XCVR_10GBASE_R_RX_DATA_READY		BIT(7)

/* Function prototypes
 */
void intel_fpga_qse_ll_set_ethtool_ops(struct net_device *dev);

#ifdef CONFIG_INTEL_FPGA_QSE_DEBUG_FS
int intel_fpga_qse_ll_init_fs(struct net_device *dev);
void intel_fpga_qse_ll_exit_fs(struct net_device *dev);
#else
static inline int intel_fpga_qse_ll_init_fs(struct net_device *dev)
{
	return 0;
}

static inline void intel_fpga_qse_ll_exit_fs(struct net_device *dev) {}
#endif /* CONFIG_INTEL_FPGA_QSE_DEBUG_FS */

#endif /* __INTEL_FPGA_QSE_LL_H__ */
