/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _FS_FUSE_DEV_H
#define _FS_FUSE_DEV_H

struct fuse_conn;

void fuse_init_server_timeout(struct fuse_conn *fc, unsigned int timeout);

#endif /* _FS_FUSE_DEV_H */
