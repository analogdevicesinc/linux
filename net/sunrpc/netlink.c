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

/* Ops table for sunrpc */
static const struct genl_split_ops sunrpc_nl_ops[] = {
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
