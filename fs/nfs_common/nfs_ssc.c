// SPDX-License-Identifier: GPL-2.0-only
/*
 * Helper for knfsd's SSC to access ops in NFS client modules
 *
 * Author: Dai Ngo <dai.ngo@oracle.com>
 *
 * Copyright (c) 2020, Oracle and/or its affiliates.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/nfs_ssc.h>
#include "../nfs/nfs4_fs.h"

struct nfs_ssc_client_ops_tbl {
	const struct nfs4_ssc_client_ops *ssc_nfs4_ops;
};

static struct nfs_ssc_client_ops_tbl nfs_ssc_client_tbl __read_mostly;

/**
 * nfsd42_ssc_open - Open a file to be used for server-to-server copy
 * @ss_mnt: active mount point on which the source file resides
 * @src_fh: file handle of the source file to be copied
 * @stateid: stateid to use for COPY operation
 *
 * Caller must close the returned file using nfsd42_ssc_close().
 *
 * Return: an open file, or an ERR_PTR on error
 */
struct file *nfsd42_ssc_open(struct vfsmount *ss_mnt, struct nfs_fh *src_fh,
			     nfs4_stateid *stateid)
{
	/*
	 * Built under CONFIG_NFS_V4_2_SSC_HELPER, which the NFS client
	 * enables on its own. The dispatch below is live only when the
	 * server also sets CONFIG_NFSD_V4_2_INTER_SSC; without it the
	 * source file cannot be opened, so callers get -EIO.
	 */
#if IS_ENABLED(CONFIG_NFSD_V4_2_INTER_SSC)
	const struct nfs4_ssc_client_ops *ops = nfs_ssc_client_tbl.ssc_nfs4_ops;

	if (ops)
		return ops->sco_open(ss_mnt, src_fh, stateid);
#endif

	return ERR_PTR(-EIO);
}
EXPORT_SYMBOL_GPL(nfsd42_ssc_open);

/**
 * nfsd42_ssc_close - Close a file opened with nfsd42_ssc_open()
 * @filp: struct file to be closed
 *
 * The real cleanup happens unconditionally in nfsd4_cleanup_inter_ssc().
 * The vfsmount is pinned until this function is called, preventing
 * the client from unregistering its SSC ops.
 */
void nfsd42_ssc_close(struct file *filp)
{
	/* Live only under CONFIG_NFSD_V4_2_INTER_SSC; see nfsd42_ssc_open(). */
#if IS_ENABLED(CONFIG_NFSD_V4_2_INTER_SSC)
	const struct nfs4_ssc_client_ops *ops = nfs_ssc_client_tbl.ssc_nfs4_ops;

	if (ops)
		ops->sco_close(filp);
#endif
}
EXPORT_SYMBOL_GPL(nfsd42_ssc_close);

#ifdef CONFIG_NFS_V4_2
/**
 * nfs42_ssc_register - install the NFS_V4 client ops in the nfs_ssc_client_tbl
 * @ops: NFS_V4 ops to be installed
 *
 * Return values:
 *   None
 */
void nfs42_ssc_register(const struct nfs4_ssc_client_ops *ops)
{
	nfs_ssc_client_tbl.ssc_nfs4_ops = ops;
}
EXPORT_SYMBOL_GPL(nfs42_ssc_register);

/**
 * nfs42_ssc_unregister - uninstall the NFS_V4 client ops from
 *				the nfs_ssc_client_tbl
 * @ops: ops to be uninstalled
 *
 * Return values:
 *   None
 */
void nfs42_ssc_unregister(const struct nfs4_ssc_client_ops *ops)
{
	if (nfs_ssc_client_tbl.ssc_nfs4_ops != ops)
		return;

	nfs_ssc_client_tbl.ssc_nfs4_ops = NULL;
}
EXPORT_SYMBOL_GPL(nfs42_ssc_unregister);
#endif /* CONFIG_NFS_V4_2 */
