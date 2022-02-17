/* Copyright 2008-2011 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __MAC_H
#define __MAC_H

#include <linux/device.h>	/* struct device, BUS_ID_SIZE */
#include <linux/if_ether.h>	/* ETH_ALEN */
#include <linux/phy.h>		/* phy_interface_t, struct phy_device */
#include <linux/list.h>

#include "lnxwrp_fsl_fman.h"	/* struct port_device */

enum {DTSEC, XGMAC, MEMAC};

struct mac_device {
	struct device		*dev;
	void			*priv;
	uint8_t			 cell_index;
	struct resource		*res;
	void __iomem		*vaddr;
	uint8_t			 addr[ETH_ALEN];
	bool			 promisc;

	struct fm		*fm_dev;
	struct fm_port		*port_dev[2];

	phy_interface_t		 phy_if;
	u32			 if_support;
	bool			 link;
	bool			 half_duplex;
	uint16_t		 speed;
	uint16_t		 max_speed;
	struct device_node	*phy_node;
	char			 fixed_bus_id[MII_BUS_ID_SIZE + 3];
	struct device_node	*tbi_node;
	struct phy_device	*phy_dev;
	void			*fm;
	/* List of multicast addresses */
	struct list_head	 mc_addr_list;
	struct fixed_phy_status	 *fixed_link;

	bool autoneg_pause;
	bool rx_pause_req;
	bool tx_pause_req;
	bool rx_pause_active;
	bool tx_pause_active;

	struct fm_mac_dev *(*get_mac_handle)(struct mac_device *mac_dev);
	int (*init_phy)(struct net_device *net_dev, struct mac_device *mac_dev);
	int (*init)(struct mac_device *mac_dev);
	int (*start)(struct mac_device *mac_dev);
	int (*stop)(struct mac_device *mac_dev);
	int (*set_promisc)(struct fm_mac_dev *fm_mac_dev, bool enable);
	int (*change_addr)(struct fm_mac_dev *fm_mac_dev, const uint8_t *addr);
	int (*set_multi)(struct net_device *net_dev,
			 struct mac_device *mac_dev);
	int (*uninit)(struct fm_mac_dev *fm_mac_dev);
	int (*ptp_enable)(struct fm_mac_dev *fm_mac_dev);
	int (*ptp_disable)(struct fm_mac_dev *fm_mac_dev);
	int (*set_rx_pause)(struct fm_mac_dev *fm_mac_dev, bool en);
	int (*set_tx_pause)(struct fm_mac_dev *fm_mac_dev, bool en);
	int (*fm_rtc_enable)(struct fm *fm_dev);
	int (*fm_rtc_disable)(struct fm *fm_dev);
	int (*fm_rtc_get_cnt)(struct fm *fm_dev, uint64_t *ts);
	int (*fm_rtc_set_cnt)(struct fm *fm_dev, uint64_t ts);
	int (*fm_rtc_get_drift)(struct fm *fm_dev, uint32_t *drift);
	int (*fm_rtc_set_drift)(struct fm *fm_dev, uint32_t drift);
	int (*fm_rtc_set_alarm)(struct fm *fm_dev, uint32_t id, uint64_t time);
	int (*fm_rtc_set_fiper)(struct fm *fm_dev, uint32_t id,
				uint64_t fiper);
	int (*fm_rtc_enable_interrupt)(struct fm *fm_dev, uint32_t events);
	int (*fm_rtc_disable_interrupt)(struct fm *fm_dev, uint32_t events);

	int (*set_wol)(struct fm_port *port, struct fm_mac_dev *fm_mac_dev,
			bool en);
	int (*dump_mac_regs)(struct mac_device *h_mac, char *buf, int nn);
	int (*dump_mac_rx_stats)(struct mac_device *h_mac, char *buf, int nn);
	int (*dump_mac_tx_stats)(struct mac_device *h_mac, char *buf, int nn);
};

struct mac_address {
	uint8_t addr[ETH_ALEN];
	struct list_head list;
};

#define get_fm_handle(net_dev) \
	(((struct dpa_priv_s *)netdev_priv(net_dev))->mac_dev->fm_dev)

#define for_each_port_device(i, port_dev)	\
	for (i = 0; i < ARRAY_SIZE(port_dev); i++)

static inline __attribute((nonnull)) void *macdev_priv(
		const struct mac_device *mac_dev)
{
	return (void *)mac_dev + sizeof(*mac_dev);
}

extern const char	*mac_driver_description;
extern const size_t	 mac_sizeof_priv[];
extern void (*const mac_setup[])(struct mac_device *mac_dev);

int set_mac_active_pause(struct mac_device *mac_dev, bool rx, bool tx);
void get_pause_cfg(struct mac_device *mac_dev, bool *rx_pause, bool *tx_pause);

#endif	/* __MAC_H */
