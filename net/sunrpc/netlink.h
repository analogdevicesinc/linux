/* SPDX-License-Identifier: ((GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause) */
/* Do not edit directly, auto-generated from: */
/*	Documentation/netlink/specs/sunrpc_cache.yaml */
/* YNL-GEN kernel header */
/* To regenerate run: tools/net/ynl/ynl-regen.sh */

#ifndef _LINUX_SUNRPC_GEN_H
#define _LINUX_SUNRPC_GEN_H

#include <net/netlink.h>
#include <net/genetlink.h>

#include <uapi/linux/sunrpc_netlink.h>

enum {
	SUNRPC_NLGRP_NONE,
	SUNRPC_NLGRP_EXPORTD,
};

extern struct genl_family sunrpc_nl_family;

#endif /* _LINUX_SUNRPC_GEN_H */
