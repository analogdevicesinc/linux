// SPDX-License-Identifier: GPL-2.0
/* Copyright 2019 NXP
 */
#include <linux/dsa/ocelot.h>

#include "tag.h"

#define OCELOT_NAME	"ocelot"
#define SEVILLE_NAME	"seville"

static void ocelot_xmit_common(struct sk_buff *skb, struct net_device *netdev,
			       __be32 ifh_prefix, void **ifh)
{
	struct dsa_port *dp = dsa_user_to_port(netdev);
	struct dsa_switch *ds = dp->ds;
	u64 vlan_tci, tag_type;
	void *injection;
	__be32 *prefix;
	u32 rew_op = 0;
	u64 qos_class;

	ocelot_xmit_get_vlan_info(skb, dsa_port_bridge_dev_get(dp), &vlan_tci,
				  &tag_type);

	qos_class = netdev_get_num_tc(netdev) ?
		    netdev_get_prio_tc_map(netdev, skb->priority) : skb->priority;

	injection = skb_push(skb, OCELOT_TAG_LEN);
	prefix = skb_push(skb, OCELOT_SHORT_PREFIX_LEN);

	*prefix = ifh_prefix;
	memset(injection, 0, OCELOT_TAG_LEN);
	ocelot_ifh_set_bypass(injection, 1);
	ocelot_ifh_set_src(injection, ds->num_ports);
	ocelot_ifh_set_qos_class(injection, qos_class);
	ocelot_ifh_set_vlan_tci(injection, vlan_tci);
	ocelot_ifh_set_tag_type(injection, tag_type);

	rew_op = ocelot_ptp_rew_op(skb);
	if (rew_op)
		ocelot_ifh_set_rew_op(injection, rew_op);

	*ifh = injection;
}

static struct sk_buff *ocelot_xmit(struct sk_buff *skb,
				   struct net_device *netdev)
{
	struct dsa_port *dp = dsa_user_to_port(netdev);
	void *injection;

	ocelot_xmit_common(skb, netdev, cpu_to_be32(0x8880000a), &injection);
	ocelot_ifh_set_dest(injection, BIT_ULL(dp->index));

	return skb;
}

static struct sk_buff *seville_xmit(struct sk_buff *skb,
				    struct net_device *netdev)
{
	struct dsa_port *dp = dsa_user_to_port(netdev);
	void *injection;

	ocelot_xmit_common(skb, netdev, cpu_to_be32(0x88800005), &injection);
	seville_ifh_set_dest(injection, BIT_ULL(dp->index));

	return skb;
}

static struct sk_buff *ocelot_rcv(struct sk_buff *skb,
				  struct net_device *netdev)
{
	u64 src_port, qos_class;
	u64 vlan_tci, tag_type;
	u8 *start = skb->data;
	struct dsa_port *dp;
	u8 *extraction;
	u16 vlan_tpid;
	u64 rew_val;

	/* Revert skb->data by the amount consumed by the DSA conduit,
	 * so it points to the beginning of the frame.
	 */
	skb_push(skb, ETH_HLEN);
	/* We don't care about the short prefix, it is just for easy entrance
	 * into the DSA conduit's RX filter. Discard it now by moving it into
	 * the headroom.
	 */
	skb_pull(skb, OCELOT_SHORT_PREFIX_LEN);
	/* And skb->data now points to the extraction frame header.
	 * Keep a pointer to it.
	 */
	extraction = skb->data;
	/* Now the EFH is part of the headroom as well */
	skb_pull(skb, OCELOT_TAG_LEN);
	/* Reset the pointer to the real MAC header */
	skb_reset_mac_header(skb);
	skb_reset_mac_len(skb);
	/* And move skb->data to the correct location again */
	skb_pull(skb, ETH_HLEN);

	/* Remove from inet csum the extraction header */
	skb_postpull_rcsum(skb, start, OCELOT_TOTAL_TAG_LEN);

	ocelot_xfh_get_src_port(extraction, &src_port);
	ocelot_xfh_get_qos_class(extraction, &qos_class);
	ocelot_xfh_get_tag_type(extraction, &tag_type);
	ocelot_xfh_get_vlan_tci(extraction, &vlan_tci);
	ocelot_xfh_get_rew_val(extraction, &rew_val);

	skb->dev = dsa_conduit_find_user(netdev, 0, src_port);
	if (!skb->dev)
		/* The switch will reflect back some frames sent through
		 * sockets opened on the bare DSA conduit. These will come back
		 * with src_port equal to the index of the CPU port, for which
		 * there is no user registered. So don't print any error
		 * message here (ignore and drop those frames).
		 */
		return NULL;

	dsa_default_offload_fwd_mark(skb);
	skb->priority = qos_class;
	OCELOT_SKB_CB(skb)->tstamp_lo = rew_val;

	/* Ocelot switches copy frames unmodified to the CPU. However, it is
	 * possible for the user to request a VLAN modification through
	 * VCAP_IS1_ACT_VID_REPLACE_ENA. In this case, what will happen is that
	 * the VLAN ID field from the Extraction Header gets updated, but the
	 * 802.1Q header does not (the classified VLAN only becomes visible on
	 * egress through the "port tag" of front-panel ports).
	 * So, for traffic extracted by the CPU, we want to pick up the
	 * classified VLAN and manually replace the existing 802.1Q header from
	 * the packet with it, so that the operating system is always up to
	 * date with the result of tc-vlan actions.
	 * NOTE: In VLAN-unaware mode, we don't want to do that, we want the
	 * frame to remain unmodified, because the classified VLAN is always
	 * equal to the pvid of the ingress port and should not be used for
	 * processing.
	 */
	dp = dsa_user_to_port(skb->dev);
	vlan_tpid = tag_type ? ETH_P_8021AD : ETH_P_8021Q;

	if (dsa_port_is_vlan_filtering(dp) &&
	    eth_hdr(skb)->h_proto == htons(vlan_tpid)) {
		u16 dummy_vlan_tci;

		skb_push_rcsum(skb, ETH_HLEN);
		__skb_vlan_pop(skb, &dummy_vlan_tci);
		skb_pull_rcsum(skb, ETH_HLEN);
		__vlan_hwaccel_put_tag(skb, htons(vlan_tpid), vlan_tci);
	}

	return skb;
}

static const struct dsa_device_ops ocelot_netdev_ops = {
	.name			= OCELOT_NAME,
	.proto			= DSA_TAG_PROTO_OCELOT,
	.xmit			= ocelot_xmit,
	.rcv			= ocelot_rcv,
	.needed_headroom	= OCELOT_TOTAL_TAG_LEN,
	.promisc_on_conduit	= true,
};

DSA_TAG_DRIVER(ocelot_netdev_ops);
MODULE_ALIAS_DSA_TAG_DRIVER(DSA_TAG_PROTO_OCELOT, OCELOT_NAME);

static const struct dsa_device_ops seville_netdev_ops = {
	.name			= SEVILLE_NAME,
	.proto			= DSA_TAG_PROTO_SEVILLE,
	.xmit			= seville_xmit,
	.rcv			= ocelot_rcv,
	.needed_headroom	= OCELOT_TOTAL_TAG_LEN,
	.promisc_on_conduit	= true,
};

DSA_TAG_DRIVER(seville_netdev_ops);
MODULE_ALIAS_DSA_TAG_DRIVER(DSA_TAG_PROTO_SEVILLE, SEVILLE_NAME);

static struct dsa_tag_driver *ocelot_tag_driver_array[] = {
	&DSA_TAG_DRIVER_NAME(ocelot_netdev_ops),
	&DSA_TAG_DRIVER_NAME(seville_netdev_ops),
};

module_dsa_tag_drivers(ocelot_tag_driver_array);

MODULE_DESCRIPTION("DSA tag driver for Ocelot family of switches, using NPI port");
MODULE_LICENSE("GPL v2");
