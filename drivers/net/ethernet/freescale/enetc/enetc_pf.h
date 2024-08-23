/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2017-2019 NXP */

#include "enetc.h"
#include "enetc_devlink.h"
#include <linux/phylink.h>

#define ENETC_PF_NUM_RINGS	8

/* This means the ENETC PF is owned by M core, but its VFs are
 * owned by A core.
 */
#define ENETC_PF_VIRTUAL_DEVID	0x080b

enum enetc_vf_flags {
	ENETC_VF_FLAG_PF_SET_MAC	= BIT(0),
	ENETC_VF_FLAG_TRUSTED		= BIT(1)
};

struct enetc_vf_state {
	enum enetc_vf_flags flags;
};

struct enetc_port_caps {
	bool half_duplex;
	bool wol;
	int num_vsi;
	int num_msix;
	int num_rx_bdr;
	int num_tx_bdr;
	int mac_filter_num;
	int vlan_filter_num;
	int ipf_words_num;
};

struct enetc_pf_hw_ops {
	void (*set_si_primary_mac)(struct enetc_hw *hw, int si, const u8 *addr);
	void (*get_si_primary_mac)(struct enetc_hw *hw, int si, u8 *addr);
	void (*set_si_based_vlan)(struct enetc_hw *hw, int si, u16 vlan, u8 qos);
	void (*get_si_based_vlan)(struct enetc_hw *hw, int si, u32 *vlan, u32 *qos);
	void (*set_si_anti_spoofing)(struct enetc_hw *hw, int si, bool en);
	void (*set_si_vlan_promisc)(struct enetc_hw *hw, char si_map);
	void (*set_si_mac_promisc)(struct enetc_hw *hw, int si, int type, bool en);
	void (*set_si_mac_hash_filter)(struct enetc_hw *hw, int si, int type, u64 hash);
	void (*set_si_vlan_hash_filter)(struct enetc_hw *hw, int si, u64 hash);
	void (*set_loopback)(struct net_device *ndev, bool en);
	void (*set_tc_tsd)(struct enetc_hw *hw, int tc, bool en);
	void (*set_tc_msdu)(struct enetc_hw *hw, u32 *max_sdu);
	void (*reset_tc_msdu)(struct enetc_hw *hw);
	bool (*get_time_gating)(struct enetc_hw *hw);
	void (*set_time_gating)(struct enetc_hw *hw, bool en);
};

struct enetc_mac_list_entry {
	struct ntmp_mfe mfe;
	struct hlist_node node;
};

struct enetc_vlan_list_entry {
	struct ntmp_vfe vfe;
	struct hlist_node node;
};

struct enetc_pf {
	struct enetc_si *si;
	int num_vfs; /* number of active VFs, after sriov_init */
	int total_vfs; /* max number of VFs, set for PF at probe */
	struct enetc_port_caps caps;
	struct enetc_vf_state *vf_state;

	struct enetc_msg_swbd rxmsg[ENETC_MAX_NUM_VFS];
	bool vf_link_status_notify[ENETC_MAX_NUM_VFS];

	char vlan_promisc_simap; /* bitmap of SIs in VLAN promisc mode */

	struct mii_bus *mdio; /* saved for cleanup */
	struct mii_bus *imdio;
	struct phylink_pcs *pcs;

	phy_interface_t if_mode;
	struct phylink_config phylink_config;
	const struct enetc_pf_hw_ops *hw_ops;
	struct enetc_devlink_priv *devl_priv;

	u8 mac_addr_base[ETH_ALEN];

	struct hlist_head mac_list; /* MAC address filter table */
	struct mutex mac_list_lock; /* mac_list lock */
	int num_mac_fe;	/* number of mac address filter table entries */

	struct hlist_head vlan_list; /* VLAN address filter table */
	struct mutex vlan_list_lock; /* mac_list lock */
	int num_vlan_fe; /* number of VLAN address filter table entries */
};

#define phylink_to_enetc_pf(config) \
	container_of((config), struct enetc_pf, phylink_config)

int enetc_msg_psi_init(struct enetc_pf *pf);
void enetc_msg_psi_free(struct enetc_pf *pf);
void enetc_msg_handle_rxmsg(struct enetc_pf *pf, int mbox_id, u16 *status);

int enetc_mdiobus_create(struct enetc_pf *pf, struct device_node *node);
void enetc_mdiobus_destroy(struct enetc_pf *pf);
void enetc_phylink_destroy(struct enetc_ndev_priv *priv);
int enetc_phylink_create(struct enetc_ndev_priv *priv,
			 struct device_node *node,
			 const struct phylink_mac_ops *pl_mac_ops);

void enetc_pf_netdev_setup(struct enetc_si *si, struct net_device *ndev,
			   const struct net_device_ops *ndev_ops);
int enetc_setup_mac_addresses(struct device_node *np, struct enetc_pf *pf);
int enetc_pf_set_mac_addr(struct net_device *ndev, void *addr);
int enetc_vlan_rx_add_vid(struct net_device *ndev, __be16 prot, u16 vid);
int enetc_vlan_rx_del_vid(struct net_device *ndev, __be16 prot, u16 vid);
int enetc_pf_set_vf_mac(struct net_device *ndev, int vf, u8 *mac);
int enetc_pf_set_vf_vlan(struct net_device *ndev, int vf, u16 vlan,
			 u8 qos, __be16 proto);
int enetc_pf_set_vf_spoofchk(struct net_device *ndev, int vf, bool en);
int enetc_pf_set_vf_trust(struct net_device *ndev, int vf, bool setting);
int enetc_pf_get_vf_config(struct net_device *ndev, int vf,
			   struct ifla_vf_info *ivi);
int enetc_pf_set_features(struct net_device *ndev, netdev_features_t features);
int enetc_pf_setup_tc(struct net_device *ndev, enum tc_setup_type type,
		      void *type_data);
int enetc_sriov_configure(struct pci_dev *pdev, int num_vfs);
void enetc_pf_flush_mac_exact_filter(struct enetc_pf *pf, int si_id,
				     int mac_type);
int enetc_pf_set_mac_exact_filter(struct enetc_pf *pf, int si_id,
				  struct enetc_mac_entry *mac,
				  int mac_cnt);
int enetc_pf_send_msg(struct enetc_pf *pf, u32 msg_code, u16 ms_mask);

static inline void enetc_pf_register_hw_ops(struct enetc_pf *pf,
					    const struct enetc_pf_hw_ops *hw_ops)
{
	pf->hw_ops = hw_ops;
}

static inline bool enetc_pf_is_owned_by_mcore(struct pci_dev *pdev)
{
	if (pdev->vendor == PCI_VENDOR_ID_NXP2 &&
	    pdev->device == ENETC_PF_VIRTUAL_DEVID)
		return true;

	return false;
}

static inline bool enetc_pf_is_vf_trusted(struct enetc_pf *pf, int vf_id)
{
	if (vf_id >= pf->total_vfs)
		return false;

	return !!(pf->vf_state[vf_id].flags & ENETC_VF_FLAG_TRUSTED);
}
