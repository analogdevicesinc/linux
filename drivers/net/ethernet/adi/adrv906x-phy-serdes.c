// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/delay.h>
#include <net/netlink.h>
#include <net/genetlink.h>
#include "adrv906x-phy-serdes.h"
#include "adrv906x-cmn.h"

#define SERDES_GENL_NAME         "adrv906x"
#define SERDES_GENL_VERSION      1
#define SERDES_GENL_MC_GRP_NAME  "adrv906x_mcgrp"
#define SERDES_MAX_LANES         4
#define SERDES_LANE_MSK          GENMASK(15, 0)
#define SERDES_SPEED_MSK         GENMASK(31, 16)
#define SERDES_TIMEOUT_SECOND    1000

typedef int (*adrv906x_serdes_action)(struct adrv906x_serdes *serdes);

enum adrv906x_serdes_states {
	STATE_IDLE,
	STATE_CAL_REQUEST,
	STATE_CAL_STARTED,
	STATE_LOS,
	STATE_RUNNING,
	STATE_PWR_DOWN,
};

enum adrv906x_serdes_events {
	EVENT_LINK_UP,
	EVENT_LINK_DOWN,
	EVENT_STOP_SUCCESS,
	EVENT_NETLINK_ACK,
	EVENT_NETLINK_NACK,
	EVENT_SIGNAL_OK,
	EVENT_LOS_DETECTED,
};

enum adrv906x_serdes_attrs {
	ATTR_UNSPEC,
	ATTR_COMMAND_PAYLOAD, /* u32, link speed (bits 31:16), lane id (bits 15:0) */
	__ATTR_MAX,
	NUM_ATTR	= __ATTR_MAX,
	ATTR_MAX	= __ATTR_MAX - 1,
};

enum adrv906x_serdes_nl_commands {
	COMMAND_STOP_SUCCESS,
	COMMAND_SIGNAL_OK,
	COMMAND_LOS_DETECTED,
	COMMAND_CAL_REQ,
	COMMAND_PWR_DOWN_REQ,
	COMMAND_REQ_DONE,
	COMMAND_RESET_4PACK_REQ,
};

struct adrv906x_serdes_transition {
	u32 src_state;
	u32 event;
	adrv906x_serdes_action action;
	u32 dst_state;
};

static int adrv906x_serdes_stop_success_recv(struct sk_buff *skb, struct genl_info *info);
static int adrv906x_serdes_signal_ok_recv(struct sk_buff *skb, struct genl_info *info);
static int adrv906x_serdes_los_detected_recv(struct sk_buff *skb, struct genl_info *info);
static int adrv906x_serdes_reset_4pack_recv(struct sk_buff *skb, struct genl_info *info);
static int adrv906x_serdes_start_cal_send(struct adrv906x_serdes *serdes);
static int adrv906x_serdes_pwr_down_send(struct adrv906x_serdes *serdes);
static int adrv906x_serdes_start_timer(struct adrv906x_serdes *serdes);
static int adrv906x_serdes_do_nothing(struct adrv906x_serdes *serdes);
static int adrv906x_serdes_stop_timer(struct adrv906x_serdes *serdes);

static struct adrv906x_serdes_transition adrv906x_serdes_transitions[] = {
	/* Source State      Event               Action                          Destination State */
	{ STATE_IDLE,	     EVENT_LINK_UP,	 adrv906x_serdes_start_cal_send, STATE_CAL_REQUEST },
	{ STATE_CAL_REQUEST, EVENT_NETLINK_ACK,	 adrv906x_serdes_do_nothing,	 STATE_CAL_STARTED },
	{ STATE_CAL_REQUEST, EVENT_NETLINK_NACK, adrv906x_serdes_start_timer,	 STATE_CAL_REQUEST },
	{ STATE_CAL_REQUEST, EVENT_LINK_DOWN,	 adrv906x_serdes_pwr_down_send,	 STATE_PWR_DOWN	   },
	{ STATE_CAL_STARTED, EVENT_LINK_UP,	 adrv906x_serdes_start_cal_send, STATE_CAL_REQUEST },
	{ STATE_CAL_STARTED, EVENT_LINK_DOWN,	 adrv906x_serdes_pwr_down_send,	 STATE_PWR_DOWN	   },
	{ STATE_CAL_STARTED, EVENT_SIGNAL_OK,	 adrv906x_serdes_stop_timer,	 STATE_RUNNING	   },
	{ STATE_RUNNING,     EVENT_LINK_UP,	 adrv906x_serdes_start_cal_send, STATE_CAL_REQUEST },
	{ STATE_RUNNING,     EVENT_LOS_DETECTED, adrv906x_serdes_do_nothing,	 STATE_LOS	   },
	{ STATE_RUNNING,     EVENT_LINK_DOWN,	 adrv906x_serdes_pwr_down_send,	 STATE_PWR_DOWN	   },
	{ STATE_LOS,	     EVENT_LINK_UP,	 adrv906x_serdes_start_cal_send, STATE_CAL_REQUEST },
	{ STATE_LOS,	     EVENT_SIGNAL_OK,	 adrv906x_serdes_do_nothing,	 STATE_RUNNING	   },
	{ STATE_LOS,	     EVENT_LINK_DOWN,	 adrv906x_serdes_pwr_down_send,	 STATE_PWR_DOWN	   },
	{ STATE_PWR_DOWN,    EVENT_STOP_SUCCESS, adrv906x_serdes_do_nothing,	 STATE_IDLE	   },
	{ STATE_PWR_DOWN,    EVENT_NETLINK_NACK, adrv906x_serdes_do_nothing,	 STATE_IDLE	   },
	{ STATE_PWR_DOWN,    EVENT_LINK_UP,	 adrv906x_serdes_start_cal_send, STATE_CAL_REQUEST },
};

static struct nla_policy adrv906x_serdes_genl_policy[ATTR_MAX + 1] = {
	[ATTR_COMMAND_PAYLOAD] = { .type = NLA_U32 },
};

static const struct genl_small_ops adrv906x_serdes_genl_ops[] = {
	{
		.cmd = COMMAND_SIGNAL_OK,
		.doit = adrv906x_serdes_signal_ok_recv,
	},
	{
		.cmd = COMMAND_STOP_SUCCESS,
		.doit = adrv906x_serdes_stop_success_recv,
	},
	{
		.cmd = COMMAND_LOS_DETECTED,
		.doit = adrv906x_serdes_los_detected_recv,
	},
	{
		.cmd = COMMAND_RESET_4PACK_REQ,
		.doit = adrv906x_serdes_reset_4pack_recv,
	},
};

static const struct genl_multicast_group adrv906x_serdes_genl_mcgrps[] = {
	{ .name = SERDES_GENL_MC_GRP_NAME },
};

static struct genl_family adrv906x_serdes_fam __ro_after_init = {
	.name		= SERDES_GENL_NAME,
	.hdrsize	= 0,
	.version	= SERDES_GENL_VERSION,
	.maxattr	= ATTR_MAX,
	.policy		= adrv906x_serdes_genl_policy,
	.module		= THIS_MODULE,
	.small_ops	= adrv906x_serdes_genl_ops,
	.n_small_ops	= ARRAY_SIZE(adrv906x_serdes_genl_ops),
	.mcgrps		= adrv906x_serdes_genl_mcgrps,
	.n_mcgrps	= ARRAY_SIZE(adrv906x_serdes_genl_mcgrps),
};

static struct adrv906x_serdes *adrv906x_serdes_devs[SERDES_MAX_LANES];

static char *adrv906x_serdes_state_to_str(u32 state)
{
	switch (state) {
	case STATE_IDLE:        return "IDLE";
	case STATE_CAL_REQUEST: return "CAL_REQUEST";
	case STATE_CAL_STARTED: return "CAL_STARTED";
	case STATE_LOS:         return "LOS";
	case STATE_RUNNING:     return "RUNNING";
	case STATE_PWR_DOWN:    return "PWR_DOWN";
	default:                return "UNKNOWN";
	}
}

static char *adrv906x_serdes_event_to_str(u32 event)
{
	switch (event) {
	case EVENT_LINK_UP:      return "LINK_UP";
	case EVENT_LINK_DOWN:    return "LINK_DOWN";
	case EVENT_STOP_SUCCESS: return "STOP_SUCCESS";
	case EVENT_NETLINK_ACK:  return "NETLINK_ACK";
	case EVENT_NETLINK_NACK: return "NETLINK_NACK";
	case EVENT_SIGNAL_OK:    return "SIGNAL_OK";
	case EVENT_LOS_DETECTED: return "LOS_DETECTED";
	default:                 return "UNKNOWN";
	}
}

int adrv906x_serdes_genl_register_family(void)
{
	return genl_register_family(&adrv906x_serdes_fam);
}

int adrv906x_serdes_genl_unregister_family(void)
{
	return genl_unregister_family(&adrv906x_serdes_fam);
}

static int adrv906x_serdes_send_message(u32 cmd, u32 lane, u32 speed)
{
	struct sk_buff *skb;
	void *hdr;
	u32 data;
	int ret;

	data = FIELD_PREP(SERDES_LANE_MSK, lane) | FIELD_PREP(SERDES_SPEED_MSK, speed);

	skb = genlmsg_new(NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (unlikely(!skb))
		return -ENOMEM;

	hdr = genlmsg_put(skb, 0, 0, &adrv906x_serdes_fam, 0, cmd);
	if (unlikely(!hdr)) {
		nlmsg_free(skb);
		return -ENOMEM;
	}

	ret = nla_put_u32(skb, ATTR_COMMAND_PAYLOAD, data);
	if (ret) {
		genlmsg_cancel(skb, hdr);
		nlmsg_free(skb);
		return ret;
	}

	genlmsg_end(skb, hdr);

	ret = genlmsg_multicast(&adrv906x_serdes_fam, skb, 0, 0, GFP_KERNEL);

	return ret;
}

static void adrv906x_serdes_lookup_transitions(struct adrv906x_serdes *serdes, u32 event)
{
	struct adrv906x_serdes_transition *transition;
	int i;

	for (i = 0; i < ARRAY_SIZE(adrv906x_serdes_transitions); i++) {
		transition = &adrv906x_serdes_transitions[i];

		if (transition->src_state == serdes->state && transition->event == event) {
			phydev_dbg(serdes->phydev, "serdes[%d], event: %s, transition: %s -> %s",
				   serdes->lane,
				   adrv906x_serdes_event_to_str(event),
				   adrv906x_serdes_state_to_str(serdes->state),
				   adrv906x_serdes_state_to_str(transition->dst_state));
			serdes->state = transition->dst_state;
			transition->action(serdes);
			break;
		}
	}
}

static int adrv906x_serdes_parse_message(struct genl_info *info, u32 *lane, u32 *speed)
{
	u32 data;

	if (!info->attrs[ATTR_COMMAND_PAYLOAD])
		return -EINVAL;

	data = nla_get_u32(info->attrs[ATTR_COMMAND_PAYLOAD]);
	*lane = FIELD_GET(SERDES_LANE_MSK, data);
	*speed = FIELD_GET(SERDES_SPEED_MSK, data);

	if (*lane >= SERDES_MAX_LANES)
		return -EINVAL;

	if (*speed != SPEED_10000 && *speed != SPEED_25000)
		return -EINVAL;

	return 0;
}

static int adrv906x_serdes_signal_ok_recv(struct sk_buff *skb, struct genl_info *info)
{
	struct adrv906x_serdes *serdes;
	struct phy_device *phydev;
	struct net_device *netdev;
	u32 lane, speed;
	int ret;

	ret = adrv906x_serdes_parse_message(info, &lane, &speed);
	if (ret)
		return ret;

	serdes = adrv906x_serdes_devs[lane];
	phydev = serdes->phydev;
	netdev = phydev->attached_dev;

	serdes->rx_path_en(phydev, true);
	adrv906x_eth_cmn_serdes_tx_sync_trigger(netdev);
	adrv906x_serdes_lookup_transitions(serdes, EVENT_SIGNAL_OK);

	return 0;
}

static int adrv906x_serdes_stop_success_recv(struct sk_buff *skb, struct genl_info *info)
{
	struct adrv906x_serdes *serdes;
	u32 lane, speed;
	int ret;

	ret = adrv906x_serdes_parse_message(info, &lane, &speed);
	if (ret)
		return ret;

	serdes = adrv906x_serdes_devs[lane];
	adrv906x_serdes_lookup_transitions(serdes, EVENT_STOP_SUCCESS);

	return 0;
}

static int adrv906x_serdes_los_detected_recv(struct sk_buff *skb, struct genl_info *info)
{
	struct adrv906x_serdes *serdes;
	struct phy_device *phydev;
	struct net_device *netdev;
	u32 lane, speed;
	int ret;

	ret = adrv906x_serdes_parse_message(info, &lane, &speed);
	if (ret)
		return ret;

	serdes = adrv906x_serdes_devs[lane];
	phydev = serdes->phydev;
	netdev = phydev->attached_dev;

	serdes->rx_path_en(phydev, false);
	adrv906x_serdes_lookup_transitions(serdes, EVENT_LOS_DETECTED);

	return 0;
}

static int adrv906x_serdes_reset_4pack_recv(struct sk_buff *skb, struct genl_info *info)
{
	struct adrv906x_serdes *serdes;
	struct phy_device *phydev;
	struct net_device *netdev;
	u32 lane, speed;
	int ret;

	ret = adrv906x_serdes_parse_message(info, &lane, &speed);
	if (ret)
		return ret;

	serdes = adrv906x_serdes_devs[lane];
	phydev = serdes->phydev;
	netdev = phydev->attached_dev;

	adrv906x_eth_cmn_serdes_reset_4pack(netdev);
	adrv906x_serdes_send_message(COMMAND_REQ_DONE, lane, speed);

	return 0;
}

static int adrv906x_serdes_start_cal_send(struct adrv906x_serdes *serdes)
{
	u32 event;
	int ret;

	ret = adrv906x_serdes_send_message(COMMAND_CAL_REQ, serdes->lane, serdes->speed);
	event = ret ? EVENT_NETLINK_NACK : EVENT_NETLINK_ACK;

	adrv906x_serdes_lookup_transitions(serdes, event);

	return 0;
}

static int adrv906x_serdes_stop_timer(struct adrv906x_serdes *serdes)
{
	cancel_delayed_work(&serdes->retry_send);

	return 0;
}

static int adrv906x_serdes_start_timer(struct adrv906x_serdes *serdes)
{
	mod_delayed_work(system_long_wq, &serdes->retry_send,
			 msecs_to_jiffies(SERDES_TIMEOUT_SECOND));

	return 0;
}

static int adrv906x_serdes_pwr_down_send(struct adrv906x_serdes *serdes)
{
	int ret;

	adrv906x_serdes_stop_timer(serdes);
	ret = adrv906x_serdes_send_message(COMMAND_PWR_DOWN_REQ, serdes->lane, serdes->speed);
	if (ret)
		adrv906x_serdes_lookup_transitions(serdes, EVENT_NETLINK_NACK);

	return 0;
}

static void adrv906x_serdes_retry_start_cal_send(struct work_struct *work)
{
	struct adrv906x_serdes *serdes = container_of(work, struct adrv906x_serdes, retry_send.work);

	adrv906x_serdes_start_cal_send(serdes);
}

static int adrv906x_serdes_do_nothing(struct adrv906x_serdes *serdes)
{
	return 0;
}

static struct adrv906x_serdes *adrv906x_serdes_instance_get(struct phy_device *phydev)
{
	struct adrv906x_serdes *serdes;
	int i;

	for (i = 0; i < SERDES_MAX_LANES; i++) {
		serdes = adrv906x_serdes_devs[i];

		if (serdes->phydev == phydev)
			return serdes;
	}

	return NULL;
}

int adrv906x_serdes_cal_start(struct phy_device *phydev)
{
	struct adrv906x_serdes *serdes;
	struct net_device *netdev;

	netdev = phydev->attached_dev;
	serdes = adrv906x_serdes_instance_get(phydev);
	if (!serdes)
		return -EINVAL;

	serdes->rx_path_en(phydev, false);
	serdes->tx_path_en(phydev, true);
	serdes->speed = phydev->speed;
	adrv906x_serdes_lookup_transitions(serdes, EVENT_LINK_UP);

	return 0;
}

int adrv906x_serdes_cal_stop(struct phy_device *phydev)
{
	struct adrv906x_serdes *serdes = adrv906x_serdes_instance_get(phydev);

	if (!serdes)
		return -EINVAL;

	adrv906x_serdes_lookup_transitions(serdes, EVENT_LINK_DOWN);

	return 0;
}

int adrv906x_serdes_open(struct phy_device *phydev, struct adrv906x_serdes *serdes,
			 adrv906x_serdes_cb tx_cb, adrv906x_serdes_cb rx_cb)
{
	serdes->phydev = phydev;
	serdes->lane = phydev->mdio.addr;
	serdes->tx_path_en = tx_cb;
	serdes->rx_path_en = rx_cb;

	if (serdes->lane >= SERDES_MAX_LANES)
		return -EINVAL;

	adrv906x_serdes_devs[serdes->lane] = serdes;
	INIT_DELAYED_WORK(&serdes->retry_send, adrv906x_serdes_retry_start_cal_send);

	return 0;
}

int adrv906x_serdes_close(struct phy_device *phydev)
{
	struct adrv906x_serdes *serdes;

	serdes = adrv906x_serdes_instance_get(phydev);
	if (!serdes)
		return -EINVAL;

	cancel_delayed_work(&serdes->retry_send);

	return 0;
}
