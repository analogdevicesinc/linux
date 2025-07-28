/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2019 HUAWEI, Inc.
 *             https://www.huawei.com/
 */
#ifndef __EROFS_FS_COMPRESS_H
#define __EROFS_FS_COMPRESS_H

#include "internal.h"

struct z_erofs_decompress_req {
	struct super_block *sb;
	struct page **in, **out;
	unsigned short pageofs_in, pageofs_out;
	unsigned int inputsize, outputsize;

	unsigned int alg;       /* the algorithm for decompression */
	bool inplace_io, partial_decoding, fillgaps;
	gfp_t gfp;      /* allocation flags for extra temporary buffers */
};

struct z_erofs_decompressor {
	int (*config)(struct super_block *sb, struct erofs_super_block *dsb,
		      void *data, int size);
	int (*decompress)(struct z_erofs_decompress_req *rq,
			  struct page **pagepool);
	int (*init)(void);
	void (*exit)(void);
	char *name;
};

/* some special page->private (unsigned long, see below) */
#define Z_EROFS_SHORTLIVED_PAGE		(-1UL << 2)
#define Z_EROFS_PREALLOCATED_PAGE	(-2UL << 2)

/*
 * For all pages in a pcluster, page->private should be one of
 * Type                         Last 2bits      page->private
 * short-lived page             00              Z_EROFS_SHORTLIVED_PAGE
 * preallocated page (tryalloc) 00              Z_EROFS_PREALLOCATED_PAGE
 * cached/managed page          00              pointer to z_erofs_pcluster
 * online page (file-backed,    01/10/11        sub-index << 2 | count
 *              some pages can be used for inplace I/O)
 *
 * page->mapping should be one of
 * Type                 page->mapping
 * short-lived page     NULL
 * preallocated page    NULL
 * cached/managed page  non-NULL or NULL (invalidated/truncated page)
 * online page          non-NULL
 *
 * For all managed pages, PG_private should be set with 1 extra refcount,
 * which is used for page reclaim / migration.
 */

/*
 * Currently, short-lived pages are pages directly from buddy system
 * with specific page->private (Z_EROFS_SHORTLIVED_PAGE).
 * In the future world of Memdescs, it should be type 0 (Misc) memory
 * which type can be checked with a new helper.
 */
static inline bool z_erofs_is_shortlived_page(struct page *page)
{
	return page->private == Z_EROFS_SHORTLIVED_PAGE;
}

static inline bool z_erofs_put_shortlivedpage(struct page **pagepool,
					      struct page *page)
{
	if (!z_erofs_is_shortlived_page(page))
		return false;
	erofs_pagepool_add(pagepool, page);
	return true;
}

extern const struct z_erofs_decompressor z_erofs_lzma_decomp;
extern const struct z_erofs_decompressor z_erofs_deflate_decomp;
extern const struct z_erofs_decompressor z_erofs_zstd_decomp;
extern const struct z_erofs_decompressor *z_erofs_decomp[];

struct z_erofs_stream_dctx {
	struct z_erofs_decompress_req *rq;
	unsigned int inpages, outpages;	/* # of {en,de}coded pages */
	int no, ni;			/* the current {en,de}coded page # */

	unsigned int avail_out;		/* remaining bytes in the decoded buffer */
	unsigned int inbuf_pos, inbuf_sz;
					/* current status of the encoded buffer */
	u8 *kin, *kout;			/* buffer mapped pointers */
	void *bounce;			/* bounce buffer for inplace I/Os */
	bool bounced;			/* is the bounce buffer used now? */
};

int z_erofs_stream_switch_bufs(struct z_erofs_stream_dctx *dctx, void **dst,
			       void **src, struct page **pgpl);
int z_erofs_fixup_insize(struct z_erofs_decompress_req *rq, const char *padbuf,
			 unsigned int padbufsize);
int __init z_erofs_init_decompressor(void);
void z_erofs_exit_decompressor(void);
#endif
