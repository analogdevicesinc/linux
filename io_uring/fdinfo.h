// SPDX-License-Identifier: GPL-2.0

struct io_ring_ctx;

void io_uring_show_fdinfo(struct seq_file *m, struct file *f);

#ifdef CONFIG_PROC_FS
__cold void io_uring_dump_reqs(struct io_ring_ctx *ctx, const char *prefix);
#else
static inline void io_uring_dump_reqs(struct io_ring_ctx *ctx, const char *prefix)
{
}
#endif
