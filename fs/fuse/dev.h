/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _FS_FUSE_DEV_H
#define _FS_FUSE_DEV_H

#include <linux/cleanup.h>

/** Maximum number of outstanding background requests */
#define FUSE_DEFAULT_MAX_BACKGROUND 12

struct fuse_conn;
struct fuse_chan;
struct fuse_dev;

struct fuse_chan *fuse_chan_new(void);
struct fuse_chan *fuse_dev_chan_new(void);
void fuse_chan_release(struct fuse_chan *fch);
void fuse_chan_free(struct fuse_chan *fch);
DEFINE_FREE(fuse_chan_free, struct fuse_chan *, if (_T) fuse_chan_free(_T))

void fuse_dev_install(struct fuse_dev *fud, struct fuse_conn *fc);
void fuse_dev_put(struct fuse_dev *fud);

void fuse_init_server_timeout(struct fuse_chan *fch, unsigned int timeout);

#endif /* _FS_FUSE_DEV_H */
