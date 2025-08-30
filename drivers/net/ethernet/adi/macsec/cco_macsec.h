// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2023-2025, Analog Devices Incorporated, All Rights Reserved
 */
 
#ifndef _CCO_MACSEC_H_
#define _CCO_MACSEC_H_

#include "cco_regdefs.h"

#include <linux/types.h>
#include <linux/netdevice.h>
#include <net/macsec.h>
#include <net/genetlink.h>
#include <uapi/linux/if_adi_macsec.h>

#if IS_ENABLED(CONFIG_KUNIT_TEST)
#define netdev_info(dev, fmt, ...) printk(KERN_NOTICE,  fmt, ##__VA_ARGS__)
#define netdev_warn(dev, fmt, ...) printk(KERN_WARNING, fmt, ##__VA_ARGS__)
#define netdev_err(dev, fmt, ...)  printk(KERN_ERROR,   fmt, ##__VA_ARGS__)
#endif

// Maximum number of MACsec SecY's supported
#define CCO_MACSEC_SECY_MAX 16

// Maximum number of MACsec Rx channels supported (per SecY)
#define CCO_MACSEC_RXSC_MAX 32

// Maximum number of MACsec keys supported
#define CCO_MACSEC_KEYS 64

// MACsec key use (unused, Rx, Tx or both)
#define CCO_MACSEC_KEY_RX        1
#define CCO_MACSEC_KEY_TX        2

struct cco_macsec_priv {
	// MACsec capabilities
	struct cco_macsec_capabilities capabilities;

	// MACsec key ID table
	u8 key_id_table[CCO_MACSEC_KEYS][MACSEC_KEYID_LEN];

	// MACsec key use (unavailable, Rx, Tx or both)
	u8 key_use[CCO_MACSEC_KEYS];

	// MACsec key reference count
	u8 key_refcnt[CCO_MACSEC_KEYS];

	// Array of configured SecYs
	struct macsec_secy *secy_array[CCO_MACSEC_SECY_MAX];

	// 1 if SecY has been stopped (via mdo_dev_stop)
	u8 secy_stopped[CCO_MACSEC_SECY_MAX];

	// vlan-in-clear setting for SecY
	u8 secy_vlan_in_clear[CCO_MACSEC_SECY_MAX];

	// Number of initial octets of each MSDU without confidentiality protection per SecY
	u8 secy_confidentiality_offs[CCO_MACSEC_SECY_MAX];

	// SecY ifIndex
	u32 secy_ifIndex[CCO_MACSEC_SECY_MAX];

	// Array of configured Rx-SCs per SecY
	struct macsec_rx_sc *rxsc_array[CCO_MACSEC_SECY_MAX][CCO_MACSEC_RXSC_MAX];

	// Tx/Rx-SA enabled array
	u8 sa_enabled[CCO_MACSEC_SECY_MAX][CCO_MACSEC_RXSC_MAX];

	// createdTime, startedTime, stoppedTime:
	struct cco_macsec_time_stats txsc_ext[CCO_MACSEC_SECY_MAX];
	struct cco_macsec_time_stats rxsc_ext[CCO_MACSEC_SECY_MAX][CCO_MACSEC_RXSC_MAX];
	struct cco_macsec_time_stats txsa_ext[CCO_MACSEC_SECY_MAX][MACSEC_NUM_AN];
	struct cco_macsec_time_stats rxsa_ext[CCO_MACSEC_SECY_MAX][CCO_MACSEC_RXSC_MAX][MACSEC_NUM_AN];

	// stats:
	struct macsec_dev_stats dev_stats[CCO_MACSEC_SECY_MAX];
	struct macsec_tx_sc_stats txsc_stats[CCO_MACSEC_SECY_MAX];
	struct macsec_rx_sc_stats rxsc_stats[CCO_MACSEC_SECY_MAX][CCO_MACSEC_RXSC_MAX];
	struct cco_macsec_port_stats port_stats[CCO_MACSEC_SECY_MAX];
	struct cco_macsec_port_stats uport_stats[CCO_MACSEC_SECY_MAX];
	struct cco_macsec_ext_port_stats ext_port_stats[CCO_MACSEC_SECY_MAX];
};

// provided by the Ethernet device driver module:
struct cco_macsec_priv* cco_macsec_get_priv(struct net_device *netdev);
void cco_macsec_reg_wr(struct net_device *netdev, unsigned long addr, u32 value);
u32  cco_macsec_reg_rd(struct net_device *netdev, unsigned long addr);
void cco_macsec_max_framesize_get(struct net_device *netdev, u32* max_framesize);

// called from the Ethernet device driver module:
int cco_macsec_init(struct net_device *dev);
void cco_macsec_exit(struct net_device *dev);
void cco_macsec_commonport_status_update(struct net_device *netdev, u8 operational, u8 enabled);
irqreturn_t cco_macsec_isr(int irq, void *dev_id);

#endif /* _CCO_MACSEC_H_ */
