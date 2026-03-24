/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _FS_FUSE_DEV_H
#define _FS_FUSE_DEV_H

#include <linux/cleanup.h>

/** Maximum number of outstanding background requests */
#define FUSE_DEFAULT_MAX_BACKGROUND 12

struct fuse_conn;
struct fuse_chan;
struct fuse_dev;
struct file;

struct fuse_chan *fuse_chan_new(void);
struct fuse_chan *fuse_dev_chan_new(void);
void fuse_chan_release(struct fuse_chan *fch);
void fuse_chan_free(struct fuse_chan *fch);
unsigned int fuse_chan_num_background(struct fuse_chan *fch);
unsigned int fuse_chan_max_background(struct fuse_chan *fch);
void fuse_chan_max_background_set(struct fuse_chan *fch, unsigned int val);
unsigned int fuse_chan_num_waiting(struct fuse_chan *fch);
void fuse_chan_set_fc(struct fuse_chan *fch, struct fuse_conn *fc);
void fuse_chan_set_initialized(struct fuse_chan *fch);
void fuse_chan_io_uring_enable(struct fuse_chan *fch);

struct fuse_forget_link *fuse_alloc_forget(void);
void fuse_chan_queue_forget(struct fuse_chan *fch, struct fuse_forget_link *forget,
			    u64 nodeid, u64 nlookup);


DEFINE_FREE(fuse_chan_free, struct fuse_chan *, if (_T) fuse_chan_free(_T))

void fuse_dev_install(struct fuse_dev *fud, struct fuse_conn *fc);
bool fuse_dev_verify(struct fuse_dev *fud, struct fuse_conn *fc);
void fuse_dev_put(struct fuse_dev *fud);
bool fuse_dev_is_installed(struct fuse_dev *fud);
bool fuse_dev_is_sync_init(struct fuse_dev *fud);
struct fuse_dev *fuse_dev_grab(struct file *file);

void fuse_init_server_timeout(struct fuse_chan *fch, unsigned int timeout);

/* Abort all requests */
void fuse_chan_abort(struct fuse_chan *fch, bool abort_with_err);
void fuse_chan_wait_aborted(struct fuse_chan *fch);

#ifdef CONFIG_FUSE_IO_URING
bool fuse_uring_enabled(void);
void fuse_uring_destruct(struct fuse_chan *fch);
#else /* CONFIG_FUSE_IO_URING */
static inline bool fuse_uring_enabled(void)
{
	return false;
}

static inline void fuse_uring_destruct(struct fuse_chan *fch)
{
}
#endif /* CONFIG_FUSE_IO_URING */

#endif /* _FS_FUSE_DEV_H */
