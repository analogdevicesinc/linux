/* SPDX-License-Identifier: GPL-2.0 */
/*
 * NFSv3 protocol definitions
 */
#ifndef _LINUX_NFS3_H
#define _LINUX_NFS3_H

#include <uapi/linux/nfs3.h>

enum nfs3_stable_how {
	NFS_UNSTABLE = 0,
	NFS_DATA_SYNC = 1,
	NFS_FILE_SYNC = 2,

	/* used to mark verf as invalid */
	NFS_INVALID_STABLE_HOW = -1
};

/* Number of 32bit words in post_op_attr */
#define NFS3_POST_OP_ATTR_WORDS		22

#endif /* _LINUX_NFS3_H */
