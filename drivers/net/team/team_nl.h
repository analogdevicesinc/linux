/* SPDX-License-Identifier: ((GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause) */
/* Do not edit directly, auto-generated from: */
/*	Documentation/netlink/specs/team.yaml */
/* YNL-GEN kernel header */

#ifndef _LINUX_TEAM_GEN_H
#define _LINUX_TEAM_GEN_H

#include <net/netlink.h>
#include <net/genetlink.h>

#include <uapi/linux/if_team.h>

/* Common nested types */
extern const struct nla_policy team_attr_option_nl_policy[TEAM_ATTR_OPTION_ARRAY_INDEX + 1];
extern const struct nla_policy team_item_option_nl_policy[TEAM_ATTR_ITEM_OPTION + 1];

/* Global operation policy for team */
extern const struct nla_policy team_nl_policy[TEAM_ATTR_LIST_OPTION + 1];

/* Ops table for team */
extern const struct genl_small_ops team_nl_ops[4];

int team_nl_noop_doit(struct sk_buff *skb, struct genl_info *info);
int team_nl_options_set_doit(struct sk_buff *skb, struct genl_info *info);
int team_nl_options_get_doit(struct sk_buff *skb, struct genl_info *info);
int team_nl_port_list_get_doit(struct sk_buff *skb, struct genl_info *info);

#endif /* _LINUX_TEAM_GEN_H */
