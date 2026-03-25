// SPDX-License-Identifier: ((GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause)
/* Do not edit directly, auto-generated from: */
/*	Documentation/netlink/specs/sunrpc_cache.yaml */
/* YNL-GEN kernel source */
/* To regenerate run: tools/net/ynl/ynl-regen.sh */

#include <net/netlink.h>
#include <net/genetlink.h>
#include <linux/sunrpc/cache.h>

#include "netlink.h"

#include <uapi/linux/sunrpc_netlink.h>

/* Common nested types */
const struct nla_policy sunrpc_ip_map_nl_policy[SUNRPC_A_IP_MAP_EXPIRY + 1] = {
	[SUNRPC_A_IP_MAP_SEQNO] = { .type = NLA_U64, },
	[SUNRPC_A_IP_MAP_CLASS] = { .type = NLA_NUL_STRING, },
	[SUNRPC_A_IP_MAP_ADDR] = { .type = NLA_NUL_STRING, },
	[SUNRPC_A_IP_MAP_DOMAIN] = { .type = NLA_NUL_STRING, },
	[SUNRPC_A_IP_MAP_NEGATIVE] = { .type = NLA_FLAG, },
	[SUNRPC_A_IP_MAP_EXPIRY] = { .type = NLA_U64, },
};

/* SUNRPC_CMD_IP_MAP_GET_REQS - dump */
static const struct nla_policy sunrpc_ip_map_get_reqs_nl_policy[SUNRPC_A_IP_MAP_REQS_REQUESTS + 1] = {
	[SUNRPC_A_IP_MAP_REQS_REQUESTS] = NLA_POLICY_NESTED(sunrpc_ip_map_nl_policy),
};

/* SUNRPC_CMD_IP_MAP_SET_REQS - do */
static const struct nla_policy sunrpc_ip_map_set_reqs_nl_policy[SUNRPC_A_IP_MAP_REQS_REQUESTS + 1] = {
	[SUNRPC_A_IP_MAP_REQS_REQUESTS] = NLA_POLICY_NESTED(sunrpc_ip_map_nl_policy),
};

/* Ops table for sunrpc */
static const struct genl_split_ops sunrpc_nl_ops[] = {
	{
		.cmd		= SUNRPC_CMD_IP_MAP_GET_REQS,
		.dumpit		= sunrpc_nl_ip_map_get_reqs_dumpit,
		.policy		= sunrpc_ip_map_get_reqs_nl_policy,
		.maxattr	= SUNRPC_A_IP_MAP_REQS_REQUESTS,
		.flags		= GENL_ADMIN_PERM | GENL_CMD_CAP_DUMP,
	},
	{
		.cmd		= SUNRPC_CMD_IP_MAP_SET_REQS,
		.doit		= sunrpc_nl_ip_map_set_reqs_doit,
		.policy		= sunrpc_ip_map_set_reqs_nl_policy,
		.maxattr	= SUNRPC_A_IP_MAP_REQS_REQUESTS,
		.flags		= GENL_ADMIN_PERM | GENL_CMD_CAP_DO,
	},
};

static const struct genl_multicast_group sunrpc_nl_mcgrps[] = {
	[SUNRPC_NLGRP_NONE] = { "none", },
	[SUNRPC_NLGRP_EXPORTD] = { "exportd", },
};

struct genl_family sunrpc_nl_family __ro_after_init = {
	.name		= SUNRPC_FAMILY_NAME,
	.version	= SUNRPC_FAMILY_VERSION,
	.netnsok	= true,
	.parallel_ops	= true,
	.module		= THIS_MODULE,
	.split_ops	= sunrpc_nl_ops,
	.n_split_ops	= ARRAY_SIZE(sunrpc_nl_ops),
	.mcgrps		= sunrpc_nl_mcgrps,
	.n_mcgrps	= ARRAY_SIZE(sunrpc_nl_mcgrps),
};
