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

#define SERDES_GENL_NAME         "adrv906x"
#define SERDES_GENL_VERSION      1
#define SERDES_GENL_MC_GRP_NAME  "adrv906x_mcgrp"
#define SERDES_MAX_LANES         2
#define SERDES_LANE_MSK          GENMASK(15, 0)
#define SERDES_SPEED_MSK         GENMASK(31, 16)
#define SERDES_TIMEOUT_SECOND    1000

typedef int (*adrv906x_serdes_action)(struct adrv906x_serdes *serdes);

enum adrv906x_serdes_states {
	SERDES_STATE_IDLE,
	SERDES_STATE_CAL_REQUEST,
	SERDES_STATE_CAL_STARTED,
	SERDES_STATE_LOS,
	SERDES_STATE_RUNNING,
	SERDES_STATE_PWR_DOWN,
};

enum adrv906x_serdes_events {
	SERDES_EVENT_LINK_UP,
	SERDES_EVENT_LINK_DOWN,
	SERDES_EVENT_STOP_SUCCESS,
	SERDES_EVENT_NETLINK_ACK,
	SERDES_EVENT_NETLINK_NACK,
	SERDES_EVENT_SIGNAL_OK,
	SERDES_EVENT_LOS_DETECTED,
};

enum {
	SERDES_ATTR_UNSPEC,
	SERDES_ATTR_CMD_PAYLOAD, /* u32, link speed (bits 31:16), lane id (bits 15:0) */
	__SERDES_ATTR_MAX,
	NUM_SERDES_ATTR = __SERDES_ATTR_MAX,
	SERDES_ATTR_MAX = __SERDES_ATTR_MAX - 1,
};

enum adrv906x_serdes_nl_commands {
	SERDES_CMD_STOP_SUCCESS,
	SERDES_CMD_SIGNAL_OK,
	SERDES_CMD_LOS_DETECTED,
	SERDES_CMD_CAL_REQ,
	SERDES_CMD_PWR_DOWN_REQ,
};

struct adrv906x_serdes_transition {
	u32 src_state;
	u32 event;
	adrv906x_serdes_action action;
	u32 dst_state;
};

static int adrv906x_serdes_stop_success(struct sk_buff *skb, struct genl_info *info);
static int adrv906x_serdes_signal_ok(struct sk_buff *skb, struct genl_info *info);
static int adrv906x_serdes_los_detected(struct sk_buff *skb, struct genl_info *info);
static int adrv906x_serdes_cal_req(struct adrv906x_serdes *serdes);
static int adrv906x_serdes_pwr_down_req(struct adrv906x_serdes *serdes);
static int adrv906x_serdes_start_timer(struct adrv906x_serdes *serdes);
static int adrv906x_serdes_start_pcs(struct adrv906x_serdes *serdes);
static int adrv906x_serdes_do_nothing(struct adrv906x_serdes *serdes);
static int adrv906x_serdes_stop_timer(struct adrv906x_serdes *serdes);

static struct adrv906x_serdes_transition adrv906x_serdes_transitions[] = {
	/* Source State             Event                      Action                        Destination State       */
	{ SERDES_STATE_IDLE,	    SERDES_EVENT_LINK_UP,      adrv906x_serdes_cal_req,	     SERDES_STATE_CAL_REQUEST },
	{ SERDES_STATE_CAL_REQUEST, SERDES_EVENT_NETLINK_ACK,  adrv906x_serdes_do_nothing,   SERDES_STATE_CAL_STARTED },
	{ SERDES_STATE_CAL_REQUEST, SERDES_EVENT_NETLINK_NACK, adrv906x_serdes_start_timer,  SERDES_STATE_CAL_REQUEST },
	{ SERDES_STATE_CAL_REQUEST, SERDES_EVENT_LINK_DOWN,    adrv906x_serdes_pwr_down_req, SERDES_STATE_PWR_DOWN    },
	{ SERDES_STATE_CAL_STARTED, SERDES_EVENT_LINK_UP,      adrv906x_serdes_cal_req,	     SERDES_STATE_CAL_REQUEST },
	{ SERDES_STATE_CAL_STARTED, SERDES_EVENT_LINK_DOWN,    adrv906x_serdes_pwr_down_req, SERDES_STATE_PWR_DOWN    },
	{ SERDES_STATE_CAL_STARTED, SERDES_EVENT_SIGNAL_OK,    adrv906x_serdes_start_pcs,    SERDES_STATE_RUNNING     },
	{ SERDES_STATE_RUNNING,	    SERDES_EVENT_LINK_UP,      adrv906x_serdes_cal_req,	     SERDES_STATE_CAL_REQUEST },
	{ SERDES_STATE_RUNNING,	    SERDES_EVENT_LOS_DETECTED, adrv906x_serdes_do_nothing,   SERDES_STATE_LOS	      },
	{ SERDES_STATE_RUNNING,	    SERDES_EVENT_LINK_DOWN,    adrv906x_serdes_pwr_down_req, SERDES_STATE_PWR_DOWN    },
	{ SERDES_STATE_LOS,	    SERDES_EVENT_LINK_UP,      adrv906x_serdes_cal_req,	     SERDES_STATE_CAL_REQUEST },
	{ SERDES_STATE_LOS,	    SERDES_EVENT_SIGNAL_OK,    adrv906x_serdes_do_nothing,   SERDES_STATE_RUNNING     },
	{ SERDES_STATE_LOS,	    SERDES_EVENT_LINK_DOWN,    adrv906x_serdes_pwr_down_req, SERDES_STATE_PWR_DOWN    },
	{ SERDES_STATE_PWR_DOWN,    SERDES_EVENT_STOP_SUCCESS, adrv906x_serdes_do_nothing,   SERDES_STATE_IDLE	      },
	{ SERDES_STATE_PWR_DOWN,    SERDES_EVENT_NETLINK_NACK, adrv906x_serdes_do_nothing,   SERDES_STATE_IDLE	      },
	{ SERDES_STATE_PWR_DOWN,    SERDES_EVENT_LINK_UP,      adrv906x_serdes_cal_req,	     SERDES_STATE_CAL_REQUEST },
};

static char *adrv906x_serdes_state_to_str(u32 state)
{
	switch (state) {
	case SERDES_STATE_IDLE:        return "IDLE";
	case SERDES_STATE_CAL_REQUEST: return "CAL_REQUEST";
	case SERDES_STATE_CAL_STARTED: return "CAL_STARTED";
	case SERDES_STATE_LOS:         return "LOS";
	case SERDES_STATE_RUNNING:     return "RUNNING";
	case SERDES_STATE_PWR_DOWN:    return "PWR_DOWN";
	default:                       return "UNKNOWN";
	}
}

static char *adrv906x_serdes_event_to_str(u32 event)
{
	switch (event) {
	case SERDES_EVENT_LINK_UP:      return "LINK_UP";
	case SERDES_EVENT_LINK_DOWN:    return "LINK_DOWN";
	case SERDES_EVENT_STOP_SUCCESS: return "STOP_SUCCESS";
	case SERDES_EVENT_NETLINK_ACK:  return "NETLINK_ACK";
	case SERDES_EVENT_NETLINK_NACK: return "NETLINK_NACK";
	case SERDES_EVENT_SIGNAL_OK:    return "SIGNAL_OK";
	case SERDES_EVENT_LOS_DETECTED: return "LOS_DETECTED";
	default:                        return "UNKNOWN";
	}
}

static struct nla_policy adrv906x_serdes_genl_policy[SERDES_ATTR_MAX + 1] = {
	[SERDES_ATTR_CMD_PAYLOAD] = { .type = NLA_U32 },
};

static const struct genl_small_ops adrv906x_serdes_genl_ops[] = {
	{
		.cmd = SERDES_CMD_SIGNAL_OK,
		.doit = adrv906x_serdes_signal_ok,
	},
	{
		.cmd = SERDES_CMD_STOP_SUCCESS,
		.doit = adrv906x_serdes_stop_success,
	},
	{
		.cmd = SERDES_CMD_LOS_DETECTED,
		.doit = adrv906x_serdes_los_detected,
	},
};

static const struct genl_multicast_group adrv906x_serdes_genl_mcgrps[] = {
	{ .name = SERDES_GENL_MC_GRP_NAME },
};

static struct genl_family adrv906x_serdes_fam __ro_after_init = {
	.name		= SERDES_GENL_NAME,
	.hdrsize	= 0,
	.version	= SERDES_GENL_VERSION,
	.maxattr	= SERDES_ATTR_MAX,
	.policy		= adrv906x_serdes_genl_policy,
	.module		= THIS_MODULE,
	.small_ops	= adrv906x_serdes_genl_ops,
	.n_small_ops	= ARRAY_SIZE(adrv906x_serdes_genl_ops),
	.mcgrps		= adrv906x_serdes_genl_mcgrps,
	.n_mcgrps	= ARRAY_SIZE(adrv906x_serdes_genl_mcgrps),
};

static struct adrv906x_serdes *adrv906x_serdes_devs[SERDES_MAX_LANES];

int adrv906x_serdes_genl_register_family(void)
{
	return genl_register_family(&adrv906x_serdes_fam);
}

int adrv906x_serdes_genl_unregister_family(void)
{
	return genl_unregister_family(&adrv906x_serdes_fam);
}

int adrv906x_serdes_send_multicast(u32 cmd, u32 data)
{
	struct sk_buff *skb;
	void *hdr;
	int ret;

	skb = genlmsg_new(NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (unlikely(!skb))
		return -ENOMEM;

	hdr = genlmsg_put(skb, 0, 0, &adrv906x_serdes_fam, 0, cmd);
	if (unlikely(!hdr)) {
		nlmsg_free(skb);
		return -ENOMEM;
	}

	ret = nla_put_u32(skb, SERDES_ATTR_CMD_PAYLOAD, data);
	if (ret) {
		genlmsg_cancel(skb, hdr);
		nlmsg_free(skb);
		return ret;
	}

	genlmsg_end(skb, hdr);

	ret = genlmsg_multicast(&adrv906x_serdes_fam, skb, 0, 0, GFP_KERNEL);

	return ret;
}

void adrv906x_serdes_lookup_transitions(struct adrv906x_serdes *serdes, u32 event)
{
	struct adrv906x_serdes_transition *transition;
	int i;

	for (i = 0; i < sizeof(adrv906x_serdes_transitions) / sizeof(struct adrv906x_serdes_transition); i++) {
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

static int adrv906x_serdes_signal_ok(struct sk_buff *skb, struct genl_info *info)
{
	struct adrv906x_serdes *serdes;
	u32 data, lane, speed;

	if (!info->attrs[SERDES_ATTR_CMD_PAYLOAD])
		return -EINVAL;

	data = nla_get_u32(info->attrs[SERDES_ATTR_CMD_PAYLOAD]);
	lane = FIELD_GET(SERDES_LANE_MSK, data);
	speed = FIELD_GET(SERDES_SPEED_MSK, data);

	if (lane >= SERDES_MAX_LANES)
		return -EINVAL;

	serdes = adrv906x_serdes_devs[lane];
	adrv906x_serdes_lookup_transitions(serdes, SERDES_EVENT_SIGNAL_OK);

	return 0;
}

static int adrv906x_serdes_stop_success(struct sk_buff *skb, struct genl_info *info)
{
	struct adrv906x_serdes *serdes;
	u32 data, lane, speed;

	if (!info->attrs[SERDES_ATTR_CMD_PAYLOAD])
		return -EINVAL;

	data = nla_get_u32(info->attrs[SERDES_ATTR_CMD_PAYLOAD]);
	lane = FIELD_GET(SERDES_LANE_MSK, data);
	speed = FIELD_GET(SERDES_SPEED_MSK, data);

	if (lane >= SERDES_MAX_LANES)
		return -EINVAL;

	serdes = adrv906x_serdes_devs[lane];
	adrv906x_serdes_lookup_transitions(serdes, SERDES_EVENT_STOP_SUCCESS);

	return 0;
}

static int adrv906x_serdes_los_detected(struct sk_buff *skb, struct genl_info *info)
{
	struct adrv906x_serdes *serdes;
	u32 data, lane, speed;

	if (!info->attrs[SERDES_ATTR_CMD_PAYLOAD])
		return -EINVAL;

	data = nla_get_u32(info->attrs[SERDES_ATTR_CMD_PAYLOAD]);
	lane = FIELD_GET(SERDES_LANE_MSK, data);
	speed = FIELD_GET(SERDES_SPEED_MSK, data);

	if (lane >= SERDES_MAX_LANES)
		return -EINVAL;

	serdes = adrv906x_serdes_devs[lane];
	adrv906x_serdes_lookup_transitions(serdes, SERDES_EVENT_LOS_DETECTED);

	return 0;
}

int adrv906x_serdes_cal_req(struct adrv906x_serdes *serdes)
{
	u32 event;
	u32 data;
	int ret;

	data = FIELD_PREP(SERDES_LANE_MSK, serdes->lane) |
	       FIELD_PREP(SERDES_SPEED_MSK, serdes->speed);

	ret = adrv906x_serdes_send_multicast(SERDES_CMD_CAL_REQ, data);
	event = ret ? SERDES_EVENT_NETLINK_NACK : SERDES_EVENT_NETLINK_ACK;

	adrv906x_serdes_lookup_transitions(serdes, event);

	return 0;
}

int adrv906x_serdes_stop_timer(struct adrv906x_serdes *serdes)
{
	cancel_delayed_work(&serdes->send_req);

	return 0;
}

int adrv906x_serdes_start_timer(struct adrv906x_serdes *serdes)
{
	mod_delayed_work(system_long_wq, &serdes->send_req,
			 msecs_to_jiffies(SERDES_TIMEOUT_SECOND));

	return 0;
}

int adrv906x_serdes_pwr_down_req(struct adrv906x_serdes *serdes)
{
	u32 data;
	int ret;

	data = FIELD_PREP(SERDES_LANE_MSK, serdes->lane) |
	       FIELD_PREP(SERDES_SPEED_MSK, serdes->speed);

	adrv906x_serdes_stop_timer(serdes);
	ret = adrv906x_serdes_send_multicast(SERDES_CMD_PWR_DOWN_REQ, data);
	if (ret)
		adrv906x_serdes_lookup_transitions(serdes, SERDES_EVENT_NETLINK_NACK);

	return 0;
}

static void adrv906x_serdes_send_req(struct work_struct *work)
{
	struct adrv906x_serdes *serdes = container_of(work, struct adrv906x_serdes, send_req.work);

	adrv906x_serdes_cal_req(serdes);
}

int adrv906x_serdes_start_pcs(struct adrv906x_serdes *serdes)
{
	struct phy_device *phydev = serdes->phydev;
	int ret;

	adrv906x_serdes_stop_timer(serdes);
	phydev->speed = serdes->speed;
	ret = serdes->cb(phydev);

	return ret;
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

	serdes = adrv906x_serdes_instance_get(phydev);
	if (!serdes)
		return -EINVAL;

	serdes->speed = phydev->speed;
	adrv906x_serdes_lookup_transitions(serdes, SERDES_EVENT_LINK_UP);

	return 0;
}

int adrv906x_serdes_cal_stop(struct phy_device *phydev)
{
	struct adrv906x_serdes *serdes = adrv906x_serdes_instance_get(phydev);

	if (!serdes)
		return -EINVAL;

	adrv906x_serdes_lookup_transitions(serdes, SERDES_EVENT_LINK_DOWN);

	return 0;
}

int adrv906x_serdes_open(struct phy_device *phydev, struct adrv906x_serdes *serdes,
			 adrv906x_serdes_cal_done_cb cb)
{
	serdes->phydev = phydev;
	serdes->lane = phydev->mdio.addr;
	serdes->cb = cb;

	if (serdes->lane >= SERDES_MAX_LANES)
		return -EINVAL;

	adrv906x_serdes_devs[serdes->lane] = serdes;
	INIT_DELAYED_WORK(&serdes->send_req, adrv906x_serdes_send_req);

	return 0;
}

int adrv906x_serdes_close(struct phy_device *phydev)
{
	struct adrv906x_serdes *serdes;

	serdes = adrv906x_serdes_instance_get(phydev);
	if (!serdes)
		return -EINVAL;

	cancel_delayed_work(&serdes->send_req);

	return 0;
}
