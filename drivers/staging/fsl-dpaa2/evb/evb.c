/* Copyright 2015 Freescale Semiconductor Inc.
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
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>
#include <linux/if_vlan.h>
#include <linux/ethtool.h>

#include <uapi/linux/if_bridge.h>
#include <net/netlink.h>

#include <linux/fsl/mc.h>

#include "dpdmux.h"
#include "dpdmux-cmd.h"

static const char evb_drv_version[] = "0.1";

/* Minimal supported DPDMUX version */
#define DPDMUX_MIN_VER_MAJOR			6
#define DPDMUX_MIN_VER_MINOR			0

/* IRQ index */
#define DPDMUX_MAX_IRQ_NUM			2

/* MAX FRAME LENGTH (currently 10k) */
#define EVB_MAX_FRAME_LENGTH		(10 * 1024)
#define EVB_MAX_MTU			(EVB_MAX_FRAME_LENGTH - VLAN_ETH_HLEN)
#define EVB_MIN_MTU			68

struct evb_port_priv {
	struct net_device	*netdev;
	struct list_head	list;
	u16			port_index;
	struct evb_priv		*evb_priv;
	u8			vlans[VLAN_VID_MASK + 1];
};

struct evb_priv {
	/* keep first */
	struct evb_port_priv	uplink;

	struct fsl_mc_io	*mc_io;
	struct list_head	port_list;
	struct dpdmux_attr	attr;
	u16			mux_handle;
	int			dev_id;
};

static int _evb_port_carrier_state_sync(struct net_device *netdev)
{
	struct evb_port_priv		*port_priv = netdev_priv(netdev);
	struct dpdmux_link_state	state;
	int err;

	err = dpdmux_if_get_link_state(port_priv->evb_priv->mc_io, 0,
				       port_priv->evb_priv->mux_handle,
				       port_priv->port_index, &state);
	if (unlikely(err)) {
		netdev_err(netdev, "dpdmux_if_get_link_state() err %d\n", err);
		return err;
	}

	WARN_ONCE(state.up > 1, "Garbage read into link_state");

	if (state.up)
		netif_carrier_on(port_priv->netdev);
	else
		netif_carrier_off(port_priv->netdev);

	return 0;
}

static int evb_port_open(struct net_device *netdev)
{
	int			err;

	/* FIXME: enable port when support added */

	err = _evb_port_carrier_state_sync(netdev);
	if (err) {
		netdev_err(netdev, "ethsw_port_carrier_state_sync err %d\n",
			   err);
		return err;
	}

	return 0;
}

static netdev_tx_t evb_dropframe(struct sk_buff *skb, struct net_device *dev)
{
	/* we don't support I/O for now, drop the frame */
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static int evb_links_state_update(struct evb_priv *priv)
{
	struct evb_port_priv	*port_priv;
	struct list_head	*pos;
	int err;

	list_for_each(pos, &priv->port_list) {
		port_priv = list_entry(pos, struct evb_port_priv, list);

		err = _evb_port_carrier_state_sync(port_priv->netdev);
		if (err)
			netdev_err(port_priv->netdev,
				   "_evb_port_carrier_state_sync err %d\n",
				   err);
	}

	return 0;
}

static irqreturn_t evb_irq0_handler(int irq_num, void *arg)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t _evb_irq0_handler_thread(int irq_num, void *arg)
{
	struct device		*dev = (struct device *)arg;
	struct fsl_mc_device	*evb_dev = to_fsl_mc_device(dev);
	struct net_device	*netdev = dev_get_drvdata(dev);
	struct evb_priv		*priv = netdev_priv(netdev);
	struct fsl_mc_io	*io = priv->mc_io;
	u16 token = priv->mux_handle;
	int irq_index = DPDMUX_IRQ_INDEX_IF;

	/* Mask the events and the if_id reserved bits to be cleared on read */
	u32 status = DPDMUX_IRQ_EVENT_LINK_CHANGED | 0xFFFF0000;
	int err;

	/* Sanity check */
	if (WARN_ON(!evb_dev || !evb_dev->irqs || !evb_dev->irqs[irq_index]))
		goto out;
	if (WARN_ON(evb_dev->irqs[irq_index]->virq != (u32)irq_num))
		goto out;

	err = dpdmux_get_irq_status(io, 0, token, irq_index, &status);
	if (unlikely(err)) {
		netdev_err(netdev, "Can't get irq status (err %d)", err);
		err = dpdmux_clear_irq_status(io, 0, token, irq_index,
					      0xFFFFFFFF);
		if (unlikely(err))
			netdev_err(netdev, "Can't clear irq status (err %d)",
				   err);
		goto out;
	}

	if (status & DPDMUX_IRQ_EVENT_LINK_CHANGED) {
		err = evb_links_state_update(priv);
		if (unlikely(err))
			goto out;
	}

out:
	return IRQ_HANDLED;
}

static int evb_setup_irqs(struct fsl_mc_device *evb_dev)
{
	struct device		*dev = &evb_dev->dev;
	struct net_device	*netdev = dev_get_drvdata(dev);
	struct evb_priv		*priv = netdev_priv(netdev);
	int err = 0;
	struct fsl_mc_device_irq *irq;
	const int irq_index = DPDMUX_IRQ_INDEX_IF;
	u32 mask = DPDMUX_IRQ_EVENT_LINK_CHANGED;

	err = fsl_mc_allocate_irqs(evb_dev);
	if (unlikely(err)) {
		dev_err(dev, "MC irqs allocation failed\n");
		return err;
	}

	if (WARN_ON(evb_dev->obj_desc.irq_count != DPDMUX_MAX_IRQ_NUM)) {
		err = -EINVAL;
		goto free_irq;
	}

	err = dpdmux_set_irq_enable(priv->mc_io, 0, priv->mux_handle,
				    irq_index, 0);
	if (unlikely(err)) {
		dev_err(dev, "dpdmux_set_irq_enable err %d\n", err);
		goto free_irq;
	}

	irq = evb_dev->irqs[irq_index];

	err = devm_request_threaded_irq(dev, irq->virq,
					evb_irq0_handler,
					_evb_irq0_handler_thread,
					IRQF_NO_SUSPEND | IRQF_ONESHOT,
					dev_name(dev), dev);
	if (unlikely(err)) {
		dev_err(dev, "devm_request_threaded_irq(): %d", err);
		goto free_irq;
	}

	err = dpdmux_set_irq_mask(priv->mc_io, 0, priv->mux_handle,
				  irq_index, mask);
	if (unlikely(err)) {
		dev_err(dev, "dpdmux_set_irq_mask(): %d", err);
		goto free_devm_irq;
	}

	err = dpdmux_set_irq_enable(priv->mc_io, 0, priv->mux_handle,
				    irq_index, 1);
	if (unlikely(err)) {
		dev_err(dev, "dpdmux_set_irq_enable(): %d", err);
		goto free_devm_irq;
	}

	return 0;

free_devm_irq:
	devm_free_irq(dev, irq->virq, dev);
free_irq:
	fsl_mc_free_irqs(evb_dev);
	return err;
}

static void evb_teardown_irqs(struct fsl_mc_device *evb_dev)
{
	struct device		*dev = &evb_dev->dev;
	struct net_device	*netdev = dev_get_drvdata(dev);
	struct evb_priv		*priv = netdev_priv(netdev);

	dpdmux_set_irq_enable(priv->mc_io, 0, priv->mux_handle,
			      DPDMUX_IRQ_INDEX_IF, 0);

	devm_free_irq(dev,
		      evb_dev->irqs[DPDMUX_IRQ_INDEX_IF]->virq,
		      dev);
	fsl_mc_free_irqs(evb_dev);
}

static int evb_port_add_rule(struct net_device *netdev,
			     const unsigned char *addr, u16 vid)
{
	struct evb_port_priv	*port_priv = netdev_priv(netdev);
	struct dpdmux_l2_rule	rule = { .vlan_id = vid };
	int			err;

	if (addr)
		ether_addr_copy(rule.mac_addr, addr);

	err = dpdmux_if_add_l2_rule(port_priv->evb_priv->mc_io,
				    0,
				    port_priv->evb_priv->mux_handle,
				    port_priv->port_index, &rule);
	if (unlikely(err))
		netdev_err(netdev, "dpdmux_if_add_l2_rule err %d\n", err);
	return err;
}

static int evb_port_del_rule(struct net_device *netdev,
			     const unsigned char *addr, u16 vid)
{
	struct evb_port_priv	*port_priv = netdev_priv(netdev);
	struct dpdmux_l2_rule	rule = { .vlan_id = vid };
	int err;

	if (addr)
		ether_addr_copy(rule.mac_addr, addr);

	err = dpdmux_if_remove_l2_rule(port_priv->evb_priv->mc_io,
				       0,
				       port_priv->evb_priv->mux_handle,
				       port_priv->port_index, &rule);
	if (unlikely(err))
		netdev_err(netdev, "dpdmux_if_remove_l2_rule err %d\n", err);
	return err;
}

static bool _lookup_address(struct net_device *netdev,
			    const unsigned char *addr)
{
	struct netdev_hw_addr	   *ha;
	struct netdev_hw_addr_list *list = (is_unicast_ether_addr(addr)) ?
					   &netdev->uc : &netdev->mc;

	netif_addr_lock_bh(netdev);
	list_for_each_entry(ha, &list->list, list) {
		if (ether_addr_equal(ha->addr, addr)) {
			netif_addr_unlock_bh(netdev);
			return true;
		}
	}
	netif_addr_unlock_bh(netdev);
	return false;
}

static inline int evb_port_fdb_prep(struct nlattr *tb[],
				    struct net_device *netdev,
				    const unsigned char *addr, u16 *vid,
				    bool del)
{
	struct evb_port_priv	*port_priv = netdev_priv(netdev);
	struct evb_priv		*evb_priv = port_priv->evb_priv;

	*vid = 0;

	if (evb_priv->attr.method != DPDMUX_METHOD_MAC &&
	    evb_priv->attr.method != DPDMUX_METHOD_C_VLAN_MAC) {
		netdev_err(netdev,
			   "EVB mode does not support MAC classification\n");
		return -EOPNOTSUPP;
	}

	/* check if the address is configured on this port */
	if (_lookup_address(netdev, addr)) {
		if (!del)
			return -EEXIST;
	} else {
		if (del)
			return -ENOENT;
	}

	if (tb[NDA_VLAN] && evb_priv->attr.method == DPDMUX_METHOD_C_VLAN_MAC) {
		if (nla_len(tb[NDA_VLAN]) != sizeof(unsigned short)) {
			netdev_err(netdev, "invalid vlan size %d\n",
				   nla_len(tb[NDA_VLAN]));
			return -EINVAL;
		}

		*vid = nla_get_u16(tb[NDA_VLAN]);

		if (!*vid || *vid >= VLAN_VID_MASK) {
			netdev_err(netdev, "invalid vid value 0x%04x\n", *vid);
			return -EINVAL;
		}
	} else if (evb_priv->attr.method == DPDMUX_METHOD_C_VLAN_MAC) {
		netdev_err(netdev,
			   "EVB mode requires explicit VLAN configuration\n");
		return -EINVAL;
	} else if (tb[NDA_VLAN]) {
		netdev_warn(netdev, "VLAN not supported, argument ignored\n");
	}

	return 0;
}

static int evb_port_fdb_add(struct ndmsg *ndm, struct nlattr *tb[],
			    struct net_device *netdev,
			    const unsigned char *addr, u16 vid, u16 flags,
			    struct netlink_ext_ack *extack)
{
	u16 _vid;
	int err;

	/* TODO: add replace support when added to iproute bridge */
	if (!(flags & NLM_F_REQUEST)) {
		netdev_err(netdev,
			   "evb_port_fdb_add unexpected flags value %08x\n",
			   flags);
		return -EINVAL;
	}

	err = evb_port_fdb_prep(tb, netdev, addr, &_vid, 0);
	if (unlikely(err))
		return err;

	err = evb_port_add_rule(netdev, addr, _vid);
	if (unlikely(err))
		return err;

	if (is_unicast_ether_addr(addr)) {
		err = dev_uc_add(netdev, addr);
		if (unlikely(err)) {
			netdev_err(netdev, "dev_uc_add err %d\n", err);
			return err;
		}
	} else {
		err = dev_mc_add(netdev, addr);
		if (unlikely(err)) {
			netdev_err(netdev, "dev_mc_add err %d\n", err);
			return err;
		}
	}

	return 0;
}

static int evb_port_fdb_del(struct ndmsg *ndm, struct nlattr *tb[],
			    struct net_device *netdev,
			    const unsigned char *addr, u16 vid,
			    struct netlink_ext_ack *extack)
{
	u16 _vid;
	int err;

	err = evb_port_fdb_prep(tb, netdev, addr, &_vid, 1);
	if (unlikely(err))
		return err;

	err = evb_port_del_rule(netdev, addr, _vid);
	if (unlikely(err))
		return err;

	if (is_unicast_ether_addr(addr)) {
		err = dev_uc_del(netdev, addr);
		if (unlikely(err)) {
			netdev_err(netdev, "dev_uc_del err %d\n", err);
			return err;
		}
	} else {
		err = dev_mc_del(netdev, addr);
		if (unlikely(err)) {
			netdev_err(netdev, "dev_mc_del err %d\n", err);
			return err;
		}
	}

	return 0;
}

static int evb_change_mtu(struct net_device *netdev,
			  int mtu)
{
	struct evb_port_priv	*port_priv = netdev_priv(netdev);
	struct evb_priv		*evb_priv = port_priv->evb_priv;
	struct list_head	*pos;
	int			err = 0;

	/* This operation is not permitted on downlinks */
	if (port_priv->port_index > 0)
		return -EPERM;

	err = dpdmux_set_max_frame_length(evb_priv->mc_io,
					  0,
					  evb_priv->mux_handle,
					  (uint16_t)(mtu + VLAN_ETH_HLEN));

	if (unlikely(err)) {
		netdev_err(netdev, "dpdmux_ul_set_max_frame_length err %d\n",
			   err);
		return err;
	}

	/* Update the max frame length for downlinks */
	list_for_each(pos, &evb_priv->port_list) {
		port_priv = list_entry(pos, struct evb_port_priv, list);
		port_priv->netdev->mtu = mtu;
	}

	netdev->mtu = mtu;
	return 0;
}

static const struct nla_policy ifla_br_policy[IFLA_MAX + 1] = {
	[IFLA_BRIDGE_FLAGS]	= { .type = NLA_U16 },
	[IFLA_BRIDGE_MODE]	= { .type = NLA_U16 },
	[IFLA_BRIDGE_VLAN_INFO]	= { .type = NLA_BINARY,
				.len = sizeof(struct bridge_vlan_info), },
};

static int evb_setlink_af_spec(struct net_device *netdev,
			       struct nlattr **tb)
{
	struct bridge_vlan_info	*vinfo;
	struct evb_port_priv	*port_priv = netdev_priv(netdev);
	int			err = 0;

	if (!tb[IFLA_BRIDGE_VLAN_INFO]) {
		netdev_err(netdev, "no VLAN INFO in nlmsg\n");
		return -EOPNOTSUPP;
	}

	vinfo = nla_data(tb[IFLA_BRIDGE_VLAN_INFO]);

	if (!vinfo->vid || vinfo->vid > VLAN_VID_MASK)
		return -EINVAL;

	err = evb_port_add_rule(netdev, NULL, vinfo->vid);
	if (unlikely(err))
		return err;

	port_priv->vlans[vinfo->vid] = 1;

	return 0;
}

static int evb_setlink(struct net_device *netdev,
		       struct nlmsghdr *nlh,
		       u16 flags,
		       struct netlink_ext_ack *extack)
{
	struct evb_port_priv	*port_priv = netdev_priv(netdev);
	struct evb_priv		*evb_priv = port_priv->evb_priv;
	struct nlattr		*attr;
	struct nlattr		*tb[(IFLA_BRIDGE_MAX > IFLA_BRPORT_MAX) ?
					IFLA_BRIDGE_MAX : IFLA_BRPORT_MAX + 1];
	int			err = 0;

	if (evb_priv->attr.method != DPDMUX_METHOD_C_VLAN &&
	    evb_priv->attr.method != DPDMUX_METHOD_S_VLAN) {
		netdev_err(netdev,
			   "EVB mode does not support VLAN only classification\n");
		return -EOPNOTSUPP;
	}

	attr = nlmsg_find_attr(nlh, sizeof(struct ifinfomsg), IFLA_AF_SPEC);
	if (attr) {
		err = nla_parse_nested_deprecated(tb, IFLA_BRIDGE_MAX, attr,
						  ifla_br_policy, NULL);
		if (unlikely(err)) {
			netdev_err(netdev,
				   "nla_parse_nested for br_policy err %d\n",
				   err);
			return err;
		}

		err = evb_setlink_af_spec(netdev, tb);
		return err;
	}

	netdev_err(netdev, "nlmsg_find_attr found no AF_SPEC\n");
	return -EOPNOTSUPP;
}

static int __nla_put_netdev(struct sk_buff *skb, struct net_device *netdev)
{
	struct evb_port_priv	*port_priv = netdev_priv(netdev);
	struct evb_priv		*evb_priv = port_priv->evb_priv;
	u8			operstate = netif_running(netdev) ?
				netdev->operstate : IF_OPER_DOWN;
	int			iflink;
	int			err;

	err = nla_put_string(skb, IFLA_IFNAME, netdev->name);
	if (unlikely(err))
		goto nla_put_err;
	err = nla_put_u32(skb, IFLA_MASTER, evb_priv->uplink.netdev->ifindex);
	if (unlikely(err))
		goto nla_put_err;
	err = nla_put_u32(skb, IFLA_MTU, netdev->mtu);
	if (unlikely(err))
		goto nla_put_err;
	err = nla_put_u8(skb, IFLA_OPERSTATE, operstate);
	if (unlikely(err))
		goto nla_put_err;
	if (netdev->addr_len) {
		err = nla_put(skb, IFLA_ADDRESS, netdev->addr_len,
			      netdev->dev_addr);
		if (unlikely(err))
			goto nla_put_err;
	}

	iflink = dev_get_iflink(netdev);
	if (netdev->ifindex != iflink) {
		err = nla_put_u32(skb, IFLA_LINK, iflink);
		if (unlikely(err))
			goto nla_put_err;
	}

	return 0;

nla_put_err:
	netdev_err(netdev, "nla_put_ err %d\n", err);
	return err;
}

static int __nla_put_port(struct sk_buff *skb, struct net_device *netdev)
{
	struct nlattr	*nest;
	int		err;

	nest = nla_nest_start_noflag(skb, IFLA_PROTINFO | NLA_F_NESTED);
	if (!nest) {
		netdev_err(netdev, "nla_nest_start failed\n");
		return -ENOMEM;
	}

	err = nla_put_u8(skb, IFLA_BRPORT_STATE, BR_STATE_FORWARDING);
	if (unlikely(err))
		goto nla_put_err;
	err = nla_put_u16(skb, IFLA_BRPORT_PRIORITY, 0);
	if (unlikely(err))
		goto nla_put_err;
	err = nla_put_u32(skb, IFLA_BRPORT_COST, 0);
	if (unlikely(err))
		goto nla_put_err;
	err = nla_put_u8(skb, IFLA_BRPORT_MODE, 0);
	if (unlikely(err))
		goto nla_put_err;
	err = nla_put_u8(skb, IFLA_BRPORT_GUARD, 0);
	if (unlikely(err))
		goto nla_put_err;
	err = nla_put_u8(skb, IFLA_BRPORT_PROTECT, 0);
	if (unlikely(err))
		goto nla_put_err;
	err = nla_put_u8(skb, IFLA_BRPORT_FAST_LEAVE, 0);
	if (unlikely(err))
		goto nla_put_err;
	err = nla_put_u8(skb, IFLA_BRPORT_LEARNING, 0);
	if (unlikely(err))
		goto nla_put_err;
	err = nla_put_u8(skb, IFLA_BRPORT_UNICAST_FLOOD, 1);
	if (unlikely(err))
		goto nla_put_err;
	nla_nest_end(skb, nest);

	return 0;

nla_put_err:
	netdev_err(netdev, "nla_put_ err %d\n", err);
	nla_nest_cancel(skb, nest);
	return err;
}

static int __nla_put_vlan(struct sk_buff *skb,  struct net_device *netdev)
{
	struct evb_port_priv	*port_priv = netdev_priv(netdev);
	struct nlattr		*nest;
	struct bridge_vlan_info	vinfo;
	const u8		*vlans = port_priv->vlans;
	u16			i;
	int			err;

	nest = nla_nest_start_noflag(skb, IFLA_AF_SPEC);
	if (!nest) {
		netdev_err(netdev, "nla_nest_start failed");
		return -ENOMEM;
	}

	for (i = 0; i < VLAN_VID_MASK + 1; i++) {
		if (!vlans[i])
			continue;

		vinfo.flags = 0;
		vinfo.vid = i;

		err = nla_put(skb, IFLA_BRIDGE_VLAN_INFO,
			      sizeof(vinfo), &vinfo);
		if (unlikely(err))
			goto nla_put_err;
	}

	nla_nest_end(skb, nest);

	return 0;

nla_put_err:
	netdev_err(netdev, "nla_put_ err %d\n", err);
	nla_nest_cancel(skb, nest);
	return err;
}

static int evb_getlink(struct sk_buff *skb, u32 pid, u32 seq,
		       struct net_device *netdev, u32 filter_mask, int nlflags)
{
	struct evb_port_priv	*port_priv = netdev_priv(netdev);
	struct evb_priv		*evb_priv = port_priv->evb_priv;
	struct ifinfomsg	*hdr;
	struct nlmsghdr		*nlh;
	int			err;

	if (evb_priv->attr.method != DPDMUX_METHOD_C_VLAN &&
	    evb_priv->attr.method != DPDMUX_METHOD_S_VLAN) {
		return 0;
	}

	nlh = nlmsg_put(skb, pid, seq, RTM_NEWLINK, sizeof(*hdr), NLM_F_MULTI);
	if (!nlh)
		return -EMSGSIZE;

	hdr = nlmsg_data(nlh);
	memset(hdr, 0, sizeof(*hdr));
	hdr->ifi_family = AF_BRIDGE;
	hdr->ifi_type = netdev->type;
	hdr->ifi_index = netdev->ifindex;
	hdr->ifi_flags = dev_get_flags(netdev);

	err = __nla_put_netdev(skb, netdev);
	if (unlikely(err))
		goto nla_put_err;

	err = __nla_put_port(skb, netdev);
	if (unlikely(err))
		goto nla_put_err;

	/* Check if the VID information is requested */
	if (filter_mask & RTEXT_FILTER_BRVLAN) {
		err = __nla_put_vlan(skb, netdev);
		if (unlikely(err))
			goto nla_put_err;
	}

	nlmsg_end(skb, nlh);
	return skb->len;

nla_put_err:
	nlmsg_cancel(skb, nlh);
	return -EMSGSIZE;
}

static int evb_dellink(struct net_device *netdev,
		       struct nlmsghdr *nlh,
		       u16 flags)
{
	struct nlattr		*tb[IFLA_BRIDGE_MAX + 1];
	struct nlattr		*spec;
	struct bridge_vlan_info	*vinfo;
	struct evb_port_priv	*port_priv = netdev_priv(netdev);
	int			err = 0;

	spec = nlmsg_find_attr(nlh, sizeof(struct ifinfomsg), IFLA_AF_SPEC);
	if (!spec)
		return 0;

	err = nla_parse_nested_deprecated(tb, IFLA_BRIDGE_MAX, spec,
					  ifla_br_policy, NULL);
	if (unlikely(err))
		return err;

	if (!tb[IFLA_BRIDGE_VLAN_INFO])
		return -EOPNOTSUPP;

	vinfo = nla_data(tb[IFLA_BRIDGE_VLAN_INFO]);

	if (!vinfo->vid || vinfo->vid > VLAN_VID_MASK)
		return -EINVAL;

	err = evb_port_del_rule(netdev, NULL, vinfo->vid);
	if (unlikely(err)) {
		netdev_err(netdev, "evb_port_del_rule err %d\n", err);
		return err;
	}
	port_priv->vlans[vinfo->vid] = 0;

	return 0;
}

static void evb_port_get_stats(struct net_device *netdev,
			       struct rtnl_link_stats64 *storage)
{
	struct evb_port_priv	*port_priv = netdev_priv(netdev);
	u64			tmp;
	int			err;

	err = dpdmux_if_get_counter(port_priv->evb_priv->mc_io,
				    0,
				    port_priv->evb_priv->mux_handle,
				    port_priv->port_index,
				    DPDMUX_CNT_ING_FRAME, &storage->rx_packets);
	if (unlikely(err))
		goto error;

	err = dpdmux_if_get_counter(port_priv->evb_priv->mc_io,
				    0,
				    port_priv->evb_priv->mux_handle,
				    port_priv->port_index,
				    DPDMUX_CNT_ING_BYTE, &storage->rx_bytes);
	if (unlikely(err))
		goto error;

	err = dpdmux_if_get_counter(port_priv->evb_priv->mc_io,
				    0,
				    port_priv->evb_priv->mux_handle,
				    port_priv->port_index,
				    DPDMUX_CNT_ING_FLTR_FRAME, &tmp);
	if (unlikely(err))
		goto error;

	err = dpdmux_if_get_counter(port_priv->evb_priv->mc_io,
				    0,
				    port_priv->evb_priv->mux_handle,
				    port_priv->port_index,
				    DPDMUX_CNT_ING_FRAME_DISCARD,
				    &storage->rx_dropped);
	if (unlikely(err)) {
		storage->rx_dropped = tmp;
		goto error;
	}
	storage->rx_dropped += tmp;

	err = dpdmux_if_get_counter(port_priv->evb_priv->mc_io,
				    0,
				    port_priv->evb_priv->mux_handle,
				    port_priv->port_index,
				    DPDMUX_CNT_ING_MCAST_FRAME,
				    &storage->multicast);
	if (unlikely(err))
		goto error;

	err = dpdmux_if_get_counter(port_priv->evb_priv->mc_io,
				    0,
				    port_priv->evb_priv->mux_handle,
				    port_priv->port_index,
				    DPDMUX_CNT_EGR_FRAME, &storage->tx_packets);
	if (unlikely(err))
		goto error;

	err = dpdmux_if_get_counter(port_priv->evb_priv->mc_io,
				    0,
				    port_priv->evb_priv->mux_handle,
				    port_priv->port_index,
				    DPDMUX_CNT_EGR_BYTE, &storage->tx_bytes);
	if (unlikely(err))
		goto error;

	err = dpdmux_if_get_counter(port_priv->evb_priv->mc_io,
				    0,
				    port_priv->evb_priv->mux_handle,
				    port_priv->port_index,
				    DPDMUX_CNT_EGR_FRAME_DISCARD,
				    &storage->tx_dropped);
	if (unlikely(err))
		goto error;

	return;

error:
	netdev_err(netdev, "dpdmux_if_get_counter err %d\n", err);
}

static const struct net_device_ops evb_port_ops = {
	.ndo_open		= &evb_port_open,

	.ndo_start_xmit		= &evb_dropframe,

	.ndo_fdb_add		= &evb_port_fdb_add,
	.ndo_fdb_del		= &evb_port_fdb_del,

	.ndo_get_stats64	= &evb_port_get_stats,
	.ndo_change_mtu		= &evb_change_mtu,
};

static void evb_get_drvinfo(struct net_device *netdev,
			    struct ethtool_drvinfo *drvinfo)
{
	struct evb_port_priv *port_priv = netdev_priv(netdev);
	u16 version_major, version_minor;
	int err;

	strscpy(drvinfo->driver, KBUILD_MODNAME, sizeof(drvinfo->driver));
	strscpy(drvinfo->version, evb_drv_version, sizeof(drvinfo->version));

	err = dpdmux_get_api_version(port_priv->evb_priv->mc_io, 0,
				     &version_major,
				     &version_minor);
	if (err)
		strscpy(drvinfo->fw_version, "N/A",
			sizeof(drvinfo->fw_version));
	else
		snprintf(drvinfo->fw_version, sizeof(drvinfo->fw_version),
			 "%u.%u", version_major, version_minor);

	strscpy(drvinfo->bus_info, dev_name(netdev->dev.parent->parent),
		sizeof(drvinfo->bus_info));
}

static int evb_get_link_ksettings(struct net_device *netdev,
				  struct ethtool_link_ksettings *link_settings)
{
	struct evb_port_priv *port_priv = netdev_priv(netdev);
	struct dpdmux_link_state state = {0};
	int err = 0;

	err = dpdmux_if_get_link_state(port_priv->evb_priv->mc_io, 0,
				       port_priv->evb_priv->mux_handle,
				       port_priv->port_index,
				       &state);
	if (err) {
		netdev_err(netdev, "ERROR %d getting link state", err);
		goto out;
	}

	/* At the moment, we have no way of interrogating the DPMAC
	 * from the DPDMUX side or there may not exist a DPMAC at all.
	 * Report only autoneg state, duplexity and speed.
	 */
	if (state.options & DPDMUX_LINK_OPT_AUTONEG)
		link_settings->base.autoneg = AUTONEG_ENABLE;
	if (!(state.options & DPDMUX_LINK_OPT_HALF_DUPLEX))
		link_settings->base.duplex = DUPLEX_FULL;
	link_settings->base.speed = state.rate;

out:
	return err;
}

static int evb_set_link_ksettings(struct net_device *netdev,
				  const struct ethtool_link_ksettings *link_settings)
{
	struct evb_port_priv *port_priv = netdev_priv(netdev);
	struct dpdmux_link_state state = {0};
	struct dpdmux_link_cfg cfg = {0};
	int err = 0;

	netdev_dbg(netdev, "Setting link parameters...");

	err = dpdmux_if_get_link_state(port_priv->evb_priv->mc_io, 0,
				       port_priv->evb_priv->mux_handle,
				       port_priv->port_index,
				       &state);
	if (err) {
		netdev_err(netdev, "ERROR %d getting link state", err);
		goto out;
	}

	/* Due to a temporary MC limitation, the DPDMUX port must be down
	 * in order to be able to change link settings. Taking steps to let
	 * the user know that.
	 */
	if (netif_running(netdev)) {
		netdev_info(netdev,
			    "Sorry, interface must be brought down first.\n");
		return -EACCES;
	}

	cfg.options = state.options;
	cfg.rate = link_settings->base.speed;
	if (link_settings->base.autoneg == AUTONEG_ENABLE)
		cfg.options |= DPDMUX_LINK_OPT_AUTONEG;
	else
		cfg.options &= ~DPDMUX_LINK_OPT_AUTONEG;
	if (link_settings->base.duplex == DUPLEX_HALF)
		cfg.options |= DPDMUX_LINK_OPT_HALF_DUPLEX;
	else
		cfg.options &= ~DPDMUX_LINK_OPT_HALF_DUPLEX;

	err = dpdmux_if_set_link_cfg(port_priv->evb_priv->mc_io, 0,
				     port_priv->evb_priv->mux_handle,
				     port_priv->port_index,
				     &cfg);
	if (err)
		/* ethtool will be loud enough if we return an error; no point
		 * in putting our own error message on the console by default
		 */
		netdev_dbg(netdev, "ERROR %d setting link cfg", err);

out:
	return err;
}

static struct {
	enum dpdmux_counter_type id;
	char name[ETH_GSTRING_LEN];
} evb_ethtool_counters[] =  {
	{DPDMUX_CNT_ING_FRAME,			"rx frames"},
	{DPDMUX_CNT_ING_BYTE,			"rx bytes"},
	{DPDMUX_CNT_ING_FLTR_FRAME,		"rx filtered frames"},
	{DPDMUX_CNT_ING_FRAME_DISCARD,		"rx discarded frames"},
	{DPDMUX_CNT_ING_BCAST_FRAME,		"rx b-cast frames"},
	{DPDMUX_CNT_ING_BCAST_BYTES,		"rx b-cast bytes"},
	{DPDMUX_CNT_ING_MCAST_FRAME,		"rx m-cast frames"},
	{DPDMUX_CNT_ING_MCAST_BYTE,		"rx m-cast bytes"},
	{DPDMUX_CNT_EGR_FRAME,			"tx frames"},
	{DPDMUX_CNT_EGR_BYTE,			"tx bytes"},
	{DPDMUX_CNT_EGR_FRAME_DISCARD,		"tx discarded frames"},
	{DPDMUX_CNT_ING_NO_BUFFER_DISCARD,	"rx discarded no buffer frames"},
};

static int evb_ethtool_get_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(evb_ethtool_counters);
	default:
		return -EOPNOTSUPP;
	}
}

static void evb_ethtool_get_strings(struct net_device *netdev,
				    u32 stringset, u8 *data)
{
	u32 i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < ARRAY_SIZE(evb_ethtool_counters); i++)
			memcpy(data + i * ETH_GSTRING_LEN,
			       evb_ethtool_counters[i].name, ETH_GSTRING_LEN);
		break;
	}
}

static void evb_ethtool_get_stats(struct net_device *netdev,
				  struct ethtool_stats *stats,
				  u64 *data)
{
	struct evb_port_priv	*port_priv = netdev_priv(netdev);
	u32			i;
	int			err;

	for (i = 0; i < ARRAY_SIZE(evb_ethtool_counters); i++) {
		err = dpdmux_if_get_counter(port_priv->evb_priv->mc_io,
					    0,
					    port_priv->evb_priv->mux_handle,
					    port_priv->port_index,
					    evb_ethtool_counters[i].id,
					    &data[i]);
		if (err)
			netdev_err(netdev, "dpdmux_if_get_counter[%s] err %d\n",
				   evb_ethtool_counters[i].name, err);
	}
}

static const struct ethtool_ops evb_port_ethtool_ops = {
	.get_drvinfo		= &evb_get_drvinfo,
	.get_link		= &ethtool_op_get_link,
	.get_link_ksettings	= &evb_get_link_ksettings,
	.set_link_ksettings	= &evb_set_link_ksettings,
	.get_strings		= &evb_ethtool_get_strings,
	.get_ethtool_stats	= &evb_ethtool_get_stats,
	.get_sset_count		= &evb_ethtool_get_sset_count,
};

static int evb_open(struct net_device *netdev)
{
	struct evb_priv	*priv = netdev_priv(netdev);
	int		err = 0;

	err = dpdmux_enable(priv->mc_io, 0, priv->mux_handle);
	if (unlikely(err))
		netdev_err(netdev, "dpdmux_enable err %d\n", err);

	return err;
}

static int evb_close(struct net_device *netdev)
{
	struct evb_priv	*priv = netdev_priv(netdev);
	int		err = 0;

	err = dpdmux_disable(priv->mc_io, 0, priv->mux_handle);
	if (unlikely(err))
		netdev_err(netdev, "dpdmux_disable err %d\n", err);

	return err;
}

static const struct net_device_ops evb_ops = {
	.ndo_start_xmit		= &evb_dropframe,
	.ndo_open		= &evb_open,
	.ndo_stop		= &evb_close,

	.ndo_bridge_setlink	= &evb_setlink,
	.ndo_bridge_getlink	= &evb_getlink,
	.ndo_bridge_dellink	= &evb_dellink,

	.ndo_get_stats64	= &evb_port_get_stats,
	.ndo_change_mtu		= &evb_change_mtu,
};

static int evb_takedown(struct fsl_mc_device *evb_dev)
{
	struct device		*dev = &evb_dev->dev;
	struct net_device	*netdev = dev_get_drvdata(dev);
	struct evb_priv		*priv = netdev_priv(netdev);
	int			err;

	err = dpdmux_close(priv->mc_io, 0, priv->mux_handle);
	if (unlikely(err))
		dev_warn(dev, "dpdmux_close err %d\n", err);

	return 0;
}

static int evb_init(struct fsl_mc_device *evb_dev)
{
	struct device		*dev = &evb_dev->dev;
	struct net_device	*netdev = dev_get_drvdata(dev);
	struct evb_priv		*priv = netdev_priv(netdev);
	u16			version_major;
	u16			version_minor;
	int			err = 0;

	priv->dev_id = evb_dev->obj_desc.id;

	err = dpdmux_open(priv->mc_io, 0, priv->dev_id, &priv->mux_handle);
	if (unlikely(err)) {
		dev_err(dev, "dpdmux_open err %d\n", err);
		goto err_exit;
	}
	if (!priv->mux_handle) {
		dev_err(dev, "dpdmux_open returned null handle but no error\n");
		err = -EFAULT;
		goto err_exit;
	}

	err = dpdmux_get_attributes(priv->mc_io, 0, priv->mux_handle,
				    &priv->attr);
	if (unlikely(err)) {
		dev_err(dev, "dpdmux_get_attributes err %d\n", err);
		goto err_close;
	}

	err = dpdmux_get_api_version(priv->mc_io, 0,
				     &version_major,
				     &version_minor);
	if (unlikely(err)) {
		dev_err(dev, "dpdmux_get_api_version err %d\n", err);
		goto err_close;
	}

	/* Minimum supported DPDMUX version check */
	if (version_major < DPDMUX_MIN_VER_MAJOR ||
	    (version_major == DPDMUX_MIN_VER_MAJOR &&
	     version_minor < DPDMUX_MIN_VER_MINOR)) {
		dev_err(dev, "DPDMUX version %d.%d not supported. Use %d.%d or greater.\n",
			version_major, version_minor,
			DPDMUX_MIN_VER_MAJOR, DPDMUX_MIN_VER_MAJOR);
		err = -ENOTSUPP;
		goto err_close;
	}

	err = dpdmux_reset(priv->mc_io, 0, priv->mux_handle);
	if (unlikely(err)) {
		dev_err(dev, "dpdmux_reset err %d\n", err);
		goto err_close;
	}

	return 0;

err_close:
	dpdmux_close(priv->mc_io, 0, priv->mux_handle);
err_exit:
	return err;
}

static void evb_remove(struct fsl_mc_device *evb_dev)
{
	struct device		*dev = &evb_dev->dev;
	struct net_device	*netdev = dev_get_drvdata(dev);
	struct evb_priv		*priv = netdev_priv(netdev);
	struct evb_port_priv	*port_priv;
	struct list_head	*pos;

	list_for_each(pos, &priv->port_list) {
		port_priv = list_entry(pos, struct evb_port_priv, list);

		rtnl_lock();
		netdev_upper_dev_unlink(port_priv->netdev, netdev);
		rtnl_unlock();

		unregister_netdev(port_priv->netdev);
		free_netdev(port_priv->netdev);
	}

	evb_teardown_irqs(evb_dev);

	unregister_netdev(netdev);

	evb_takedown(evb_dev);
	fsl_mc_portal_free(priv->mc_io);

	dev_set_drvdata(dev, NULL);
	free_netdev(netdev);
}

static int evb_probe(struct fsl_mc_device *evb_dev)
{
	struct device		*dev;
	struct evb_priv		*priv = NULL;
	struct net_device	*netdev = NULL;
	char			port_name[IFNAMSIZ];
	int			i;
	int			err = 0;

	dev = &evb_dev->dev;

	/* register switch device, it's for management only - no I/O */
	netdev = alloc_etherdev(sizeof(*priv));
	if (!netdev) {
		dev_err(dev, "alloc_etherdev error\n");
		return -ENOMEM;
	}
	netdev->netdev_ops = &evb_ops;

	dev_set_drvdata(dev, netdev);

	priv = netdev_priv(netdev);

	err = fsl_mc_portal_allocate(evb_dev, FSL_MC_IO_ATOMIC_CONTEXT_PORTAL,
				     &priv->mc_io);
	if (err) {
		if (err == -ENXIO)
			err = -EPROBE_DEFER;
		else
			dev_err(dev, "fsl_mc_portal_allocate err %d\n", err);
		goto err_free_netdev;
	}

	if (!priv->mc_io) {
		dev_err(dev, "fsl_mc_portal_allocate returned null handle but no error\n");
		err = -EFAULT;
		goto err_free_netdev;
	}

	err = evb_init(evb_dev);
	if (unlikely(err)) {
		dev_err(dev, "evb init err %d\n", err);
		goto err_free_cmdport;
	}

	INIT_LIST_HEAD(&priv->port_list);
	netdev->flags |= IFF_PROMISC | IFF_MASTER;

	dev_alloc_name(netdev, "evb%d");

	/* register switch ports */
	snprintf(port_name, IFNAMSIZ, "%sp%%d", netdev->name);

	/* only register downlinks? */
	for (i = 0; i < priv->attr.num_ifs + 1; i++) {
		struct net_device *port_netdev;
		struct evb_port_priv *port_priv;

		if (i) {
			port_netdev =
				alloc_etherdev(sizeof(struct evb_port_priv));
			if (!port_netdev) {
				dev_err(dev, "alloc_etherdev error\n");
				goto err_takedown;
			}

			port_priv = netdev_priv(port_netdev);

			port_netdev->flags |= IFF_PROMISC | IFF_SLAVE;

			dev_alloc_name(port_netdev, port_name);
		} else {
			port_netdev = netdev;
			port_priv = &priv->uplink;
		}

		port_priv->netdev = port_netdev;
		port_priv->evb_priv = priv;
		port_priv->port_index = i;

		SET_NETDEV_DEV(port_netdev, dev);

		if (i) {
			port_netdev->netdev_ops = &evb_port_ops;

			err = register_netdev(port_netdev);
			if (err < 0) {
				dev_err(dev, "register_netdev err %d\n", err);
				free_netdev(port_netdev);
				goto err_takedown;
			}

			rtnl_lock();
			err = netdev_master_upper_dev_link(port_netdev, netdev,
							   NULL, NULL, NULL);
			if (unlikely(err)) {
				dev_err(dev, "netdev_master_upper_dev_link err %d\n",
					err);
				unregister_netdev(port_netdev);
				free_netdev(port_netdev);
				rtnl_unlock();
				goto err_takedown;
			}
			rtmsg_ifinfo(RTM_NEWLINK, port_netdev,
				     IFF_SLAVE, GFP_KERNEL, 0, NULL);
			rtnl_unlock();

			list_add(&port_priv->list, &priv->port_list);
		} else {
			/* Set MTU limits only on uplink */
			port_netdev->min_mtu = EVB_MIN_MTU;
			port_netdev->max_mtu = EVB_MAX_MTU;

			err = register_netdev(netdev);

			if (err < 0) {
				dev_err(dev, "register_netdev error %d\n", err);
				goto err_takedown;
			}
		}

		port_netdev->ethtool_ops = &evb_port_ethtool_ops;

		/* ports are up from init */
		rtnl_lock();
		err = dev_open(port_netdev, NULL);
		rtnl_unlock();
		if (unlikely(err))
			dev_warn(dev, "dev_open err %d\n", err);
	}

	/* setup irqs */
	err = evb_setup_irqs(evb_dev);
	if (unlikely(err)) {
		dev_warn(dev, "evb_setup_irqs err %d\n", err);
		goto err_takedown;
	}

	dev_info(dev, "probed evb device with %d ports\n",
		 priv->attr.num_ifs);
	return 0;

err_takedown:
	evb_remove(evb_dev);
err_free_cmdport:
	fsl_mc_portal_free(priv->mc_io);
err_free_netdev:
	return err;
}

static const struct fsl_mc_device_id evb_match_id_table[] = {
	{
		.vendor = FSL_MC_VENDOR_FREESCALE,
		.obj_type = "dpdmux",
	},
	{}
};

static struct fsl_mc_driver evb_drv = {
	.driver = {
		.name		= KBUILD_MODNAME,
		.owner		= THIS_MODULE,
	},
	.probe		= evb_probe,
	.remove		= evb_remove,
	.match_id_table = evb_match_id_table,
};

module_fsl_mc_driver(evb_drv);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Layerscape DPAA Edge Virtual Bridge driver (prototype)");
