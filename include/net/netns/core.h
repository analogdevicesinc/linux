/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __NETNS_CORE_H__
#define __NETNS_CORE_H__

#include <linux/types.h>

struct ctl_table_header;
struct prot_inuse;
struct cpumask;

struct netns_core {
	/* core sysctls */
	struct ctl_table_header	*sysctl_hdr;

	int	sysctl_somaxconn;
	int	sysctl_txq_reselection;
	int	sysctl_optmem_max;
	u8	sysctl_txrehash;
	u8	sysctl_tstamp_allow_data;
	u8	sysctl_bypass_prot_mem;

#ifdef CONFIG_PROC_FS
	struct prot_inuse __percpu *prot_inuse;
#endif

#if IS_ENABLED(CONFIG_RPS) && IS_ENABLED(CONFIG_SYSCTL)
	struct cpumask *rps_default_mask;
#endif
};

#endif
