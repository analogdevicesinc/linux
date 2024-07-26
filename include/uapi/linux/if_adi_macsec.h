// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
// -----------------------------------------------------------------------------
// Comcores ApS (R) all rights reserved.
//
// *****************************************************************************
#ifndef _UAPI_CCO_MACSEC_H
#define _UAPI_CCO_MACSEC_H

#include <linux/types.h>

#define CCO_MACSEC_GENL_NAME "cco_macsec"
#define CCO_MACSEC_GENL_VERSION 1

// Comcores MACsec capabilities
#define CCO_CS_AES_GCM_128       1
#define CCO_CS_AES_GCM_256       2
#define CCO_CS_AES_GCM_XPN_128   4
#define CCO_CS_AES_GCM_XPN_256   8
struct cco_macsec_capabilities {
	u64 aes_gcm_128_cs_id;      // AES-GCM-128 Cipher Suite globally unique 64-bit (EUI-64) identifier.
	u64 aes_gcm_256_cs_id;      // AES-GCM-256 Cipher Suite globally unique 64-bit (EUI-64) identifier.
	u16 no_of_peers;            // Maximum number of peers per CA.
	u16 no_of_key_entries_rx;   // Maximum number of supported keys in ingress.
	u16 no_of_key_entries_tx;   // Maximum number of supported keys in egress.
	u16 no_of_secys;            // Maximum number of virtual ports/SecYs instantiated.
	u16 no_tt_entries_rx;       // Maximum number of rules for traffic mapping table in ingress.
	u16 no_tt_entries_tx;       // Maximum number of rules for traffic mapping table in egress.
	u8  confidentiality_offs;   // Confidentiality offset supported (bit per SecY).
	u8  available_ciphersuites; // Bitmask, see CCO_CS_* defines.
	u8  vlan_in_clear;          // VLAN in clear supported (bit per SecY).
	u8  ICVLength;              // Number of octets in the ICV.
	u8  changesDataLength;      // 1 if Cipher Suite changes data length.
	u8  offsConfidentiality;    // 1 if a selectable offset for confidentiality can be provided.
	u8  integrityProtection;    // 1 if integrity protection without confidentiality can be provided.
	u8  maxTxKeys;              // Max. number of keys in simultaneous use for Tx (per SecY).
	u8  maxTxChannels;          // Max. number of Tx channels (per SecY).
	u8  maxRxKeys;              // Max. number of keys in simultaneous use for Rx (per SecY).
	u8  maxRxChannels;          // Max. number of Rx channels (per SecY).
	u8  spare;
};

// Comcores MACsec SecY extended configuration
struct cco_macsec_secy_ext {
	u8  vlan_in_clear;          // 1 to select VLAN-in-clear
	u8  confidentiality_offset; // Confidentiality offset in bytes (default 0)
	u8  spare[2];
	u32 secy_txsc_pn_thr;       // SecY / Tx-SC packet number (PN) threshold
};

// Comcores MACsec SecY port statistics
struct cco_macsec_port_stats {
	u32 ifInOctets;
	u32 ifInUcPkts;
	u32 ifInMcPkts;
	u32 ifInBcPkts;
	u32 ifInDiscards;
	u32 ifInErrors;
	u32 ifOutOctets;
	u32 ifOutUcPkts;
	u32 ifOutMcPkts;
	u32 ifOutBcPkts;
	u32 ifOutErrors;
};

// Comcores MACsec SecY extra port statistics
struct cco_macsec_ext_port_stats {
	u32 compTxDisable;         // if common port is disabled
	u32 compRxDisable;         // if common port is disabled
	u32 txSecYDisable;         // if common port is enabled, but control port is disabled
	u32 rxSecYDisable;         // if common port is enabled, but control port is disabled
	u32 txReceivingDisable;    // if common port is enabled but reception is disabled
	u32 rxTransmittingDisable; // if common port is enabled but transmission is disabled
};

// Comcores MACsec traffic rule
struct cco_macsec_traffic_rule {
	u8  macAddr[6];
	u16 vlanId;
	u16 ethType;
	u8  reserved;
#define CCO_TRAFFIC_RULE_SEL_MACADDR (1 << 0) // bit 0
#define CCO_TRAFFIC_RULE_SEL_VLAN    (1 << 1) // bit 1
#define CCO_TRAFFIC_RULE_SEL_ETHTYPE (1 << 2) // bit 2
#define CCO_TRAFFIC_RULE_SEL_OTHER   (1 << 3) // bit 3
	u8  fieldSelect;
	u32 other;
	u32 secy_index;  // index of SecY associated with the rule
};

// Comcores MACsec time stats (jiffies)
struct cco_macsec_time_stats {
	u32 createdTime;
	u32 startedTime;
	u32 stoppedTime;
	u32 spare;
};

enum cco_macsec_attrs {
	CCO_MACSEC_ATTR_UNSPEC,
	CCO_MACSEC_ATTR_CAPABILITIES,   /* struct cco_macsec_capabilities */
	CCO_MACSEC_ATTR_SECY_EXT,       /* struct cco_macsec_secy_ext */
	CCO_MACSEC_ATTR_IFINDEX,        /* u32, ifindex of the MACsec netdevice */
	CCO_MACSEC_ATTR_INDEX,          /* u32, index */
	CCO_MACSEC_ATTR_SCI,            /* u64 (sci_t), identifies Rx-SC */
	CCO_MACSEC_ATTR_TRAFFIC_RULE,   /* struct cco_macsec_traffic_rule */
	CCO_MACSEC_ATTR_PORT_STATS,     /* struct cco_macsec_port_stats */
	CCO_MACSEC_ATTR_EXT_PORT_STATS, /* struct cco_macsec_ext_port_stats */
	CCO_MACSEC_ATTR_TIME_STATS,     /* struct cco_macsec_time_stats */
	__CCO_MACSEC_ATTR_END,
	NUM_CCO_MACSEC_ATTR = __CCO_MACSEC_ATTR_END,
	CCO_MACSEC_ATTR_MAX = __CCO_MACSEC_ATTR_END - 1,
};

enum cco_macsec_nl_commands {
	CCO_MACSEC_CMD_GET_CAPABILITIES,    // returns CCO_MACSEC_ATTR_CAPABILITIES
	CCO_MACSEC_CMD_GET_SECY_EXT,        // CCO_MACSEC_ATTR_IFINDEX, returns CCO_MACSEC_ATTR_SECY_EXT
	CCO_MACSEC_CMD_SET_SECY_EXT,        // CCO_MACSEC_ATTR_IFINDEX + CCO_MACSEC_ATTR_SECY_EXT
	CCO_MACSEC_CMD_GET_RX_TRAFFIC_RULE, // CCO_MACSEC_ATTR_INDEX, returns CCO_MACSEC_ATTR_TRAFFIC_RULE
	CCO_MACSEC_CMD_SET_RX_TRAFFIC_RULE, // CCO_MACSEC_ATTR_INDEX + CCO_MACSEC_ATTR_TRAFFIC_RULE
	CCO_MACSEC_CMD_GET_TX_TRAFFIC_RULE, // CCO_MACSEC_ATTR_INDEX, returns CCO_MACSEC_ATTR_TRAFFIC_RULE
	CCO_MACSEC_CMD_SET_TX_TRAFFIC_RULE, // CCO_MACSEC_ATTR_INDEX + CCO_MACSEC_ATTR_TRAFFIC_RULE
	CCO_MACSEC_CMD_GET_PORT_STATS,      // CCO_MACSEC_ATTR_IFINDEX, returns CCO_MACSEC_ATTR_PORT_STATS (controlled port)
	CCO_MACSEC_CMD_GET_UPORT_STATS,     // CCO_MACSEC_ATTR_IFINDEX, returns CCO_MACSEC_ATTR_PORT_STATS (uncontrolled port)
	CCO_MACSEC_CMD_GET_EXT_PORT_STATS,  // CCO_MACSEC_ATTR_IFINDEX, returns CCO_MACSEC_ATTR_EXT_PORT_STATS
	CCO_MACSEC_CMD_GET_TXSC_EXT,        // CCO_MACSEC_ATTR_IFINDEX, returns CCO_MACSEC_ATTR_TIME_STATS
	CCO_MACSEC_CMD_GET_RXSC_EXT,        // CCO_MACSEC_ATTR_IFINDEX + CCO_MACSEC_ATTR_SCI, returns CCO_MACSEC_ATTR_TIME_STATS
	CCO_MACSEC_CMD_GET_TXSA_EXT,        // CCO_MACSEC_ATTR_IFINDEX + CCO_MACSEC_ATTR_INDEX (0-3), returns CCO_MACSEC_ATTR_TIME_STATS
	CCO_MACSEC_CMD_GET_RXSA_EXT,        // CCO_MACSEC_ATTR_IFINDEX + CCO_MACSEC_ATTR_SCI + CCO_MACSEC_ATTR_INDEX (0-3), returns CCO_MACSEC_ATTR_TIME_STATS
	CCO_MACSEC_CMD_CLEAR_STATS,         // CCO_MACSEC_ATTR_IFINDEX, clear all MACsec stats for ifIndex
};

#endif /* _UAPI_CCO_MACSEC_H */
