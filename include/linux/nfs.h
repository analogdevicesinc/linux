/* SPDX-License-Identifier: GPL-2.0 */
/*
 * NFS protocol definitions
 *
 * This file contains constants mostly for Version 2 of the protocol,
 * but also has a couple of NFSv3 bits in (notably the error codes).
 */
#ifndef _LINUX_NFS_H
#define _LINUX_NFS_H

#include <linux/cred.h>
#include <linux/sunrpc/auth.h>
#include <linux/sunrpc/msg_prot.h>
#include <linux/nfs_fh.h>

#include <uapi/linux/nfs.h>

/* The LOCALIO program is entirely private to Linux and is
 * NOT part of the uapi.
 */
#define NFS_LOCALIO_PROGRAM		400122
#define LOCALIOPROC_NULL		0
#define LOCALIOPROC_UUID_IS_LOCAL	1

enum nfs3_stable_how {
	NFS_UNSTABLE = 0,
	NFS_DATA_SYNC = 1,
	NFS_FILE_SYNC = 2,

	/* used by direct.c to mark verf as invalid */
	NFS_INVALID_STABLE_HOW = -1
};

#endif /* _LINUX_NFS_H */
