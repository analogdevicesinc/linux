/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _FS_FUSE_DEV_H
#define _FS_FUSE_DEV_H

#include <linux/cleanup.h>

struct fuse_conn;
struct fuse_chan;

struct fuse_chan *fuse_chan_new(void);
struct fuse_chan *fuse_dev_chan_new(void);
void fuse_chan_release(struct fuse_chan *fch);
void fuse_chan_free(struct fuse_chan *fch);
DEFINE_FREE(fuse_chan_free, struct fuse_chan *, if (_T) fuse_chan_free(_T))

void fuse_init_server_timeout(struct fuse_conn *fc, unsigned int timeout);

#endif /* _FS_FUSE_DEV_H */
