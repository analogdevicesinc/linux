// SPDX-License-Identifier: GPL-2.0
/*
 * procfs-based user access to knfsd statistics
 *
 * /proc/net/rpc/nfsd
 *
 * Format:
 *	rc <hits> <misses> <nocache>
 *			Statistsics for the reply cache
 *	fh <stale> <deprecated filehandle cache stats>
 *			statistics for filehandle lookup
 *	io <bytes-read> <bytes-written>
 *			statistics for IO throughput
 *	th <threads> <deprecated thread usage histogram stats>
 *			number of threads
 *	ra <deprecated ra-cache stats>
 *
 *	plus generic RPC stats (see net/sunrpc/stats.c)
 *
 * Copyright (C) 1995, 1996, 1997 Olaf Kirch <okir@monad.swb.de>
 */

#include <linux/seq_file.h>
#include <linux/module.h>
#include <linux/sunrpc/stats.h>
#include <net/net_namespace.h>

#include "nfsd.h"

static int nfsd_show(struct seq_file *seq, void *v)
{
	struct net *net = pde_data(file_inode(seq->file));
	struct nfsd_net *nn = net_generic(net, nfsd_net_id);
	int i;

	seq_printf(seq, "rc %lld %lld %lld\nfh %lld 0 0 0 0\nio %lld %lld\n",
		   percpu_counter_sum_positive(&nn->counter[NFSD_STATS_RC_HITS]),
		   percpu_counter_sum_positive(&nn->counter[NFSD_STATS_RC_MISSES]),
		   percpu_counter_sum_positive(&nn->counter[NFSD_STATS_RC_NOCACHE]),
		   percpu_counter_sum_positive(&nn->counter[NFSD_STATS_FH_STALE]),
		   percpu_counter_sum_positive(&nn->counter[NFSD_STATS_IO_READ]),
		   percpu_counter_sum_positive(&nn->counter[NFSD_STATS_IO_WRITE]));

	/* thread usage: */
	seq_printf(seq, "th %u 0", atomic_read(&nfsd_th_cnt));

	/* deprecated thread usage histogram stats */
	for (i = 0; i < 10; i++)
		seq_puts(seq, " 0.000");

	/* deprecated ra-cache stats */
	seq_puts(seq, "\nra 0 0 0 0 0 0 0 0 0 0 0 0\n");

	/* show my rpc info */
	svc_seq_show(seq, &nn->nfsd_svcstats);

#ifdef CONFIG_NFSD_V4
	/* Show count for individual nfsv4 operations */
	/* Writing operation numbers 0 1 2 also for maintaining uniformity */
	seq_printf(seq, "proc4ops %u", LAST_NFS4_OP + 1);
	for (i = 0; i <= LAST_NFS4_OP; i++) {
		seq_printf(seq, " %lld",
			   percpu_counter_sum_positive(&nn->counter[NFSD_STATS_NFS4_OP(i)]));
	}
	seq_printf(seq, "\nwdeleg_getattr %lld",
		percpu_counter_sum_positive(&nn->counter[NFSD_STATS_WDELEG_GETATTR]));

	seq_putc(seq, '\n');
#endif

	return 0;
}

DEFINE_PROC_SHOW_ATTRIBUTE(nfsd);

void nfsd_proc_stat_init(struct net *net)
{
	struct nfsd_net *nn = net_generic(net, nfsd_net_id);

	svc_proc_register(net, &nn->nfsd_svcstats, &nfsd_proc_ops);
}

void nfsd_proc_stat_shutdown(struct net *net)
{
	svc_proc_unregister(net, "nfsd");
}
