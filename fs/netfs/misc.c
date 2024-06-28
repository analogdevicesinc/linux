// SPDX-License-Identifier: GPL-2.0-only
/* Miscellaneous routines.
 *
 * Copyright (C) 2023 Red Hat, Inc. All Rights Reserved.
 * Written by David Howells (dhowells@redhat.com)
 */

#include <linux/swap.h>
#include "internal.h"

/*
 * Attach a folio to the buffer and maybe set marks on it to say that we need
 * to put the folio later and twiddle the pagecache flags.
 */
int netfs_xa_store_and_mark(struct xarray *xa, unsigned long index,
			    struct folio *folio, unsigned int flags,
			    gfp_t gfp_mask)
{
	XA_STATE_ORDER(xas, xa, index, folio_order(folio));

retry:
	xas_lock(&xas);
	for (;;) {
		xas_store(&xas, folio);
		if (!xas_error(&xas))
			break;
		xas_unlock(&xas);
		if (!xas_nomem(&xas, gfp_mask))
			return xas_error(&xas);
		goto retry;
	}

	if (flags & NETFS_FLAG_PUT_MARK)
		xas_set_mark(&xas, NETFS_BUF_PUT_MARK);
	if (flags & NETFS_FLAG_PAGECACHE_MARK)
		xas_set_mark(&xas, NETFS_BUF_PAGECACHE_MARK);
	xas_unlock(&xas);
	return xas_error(&xas);
}

/*
 * Create the specified range of folios in the buffer attached to the read
 * request.  The folios are marked with NETFS_BUF_PUT_MARK so that we know that
 * these need freeing later.
 */
int netfs_add_folios_to_buffer(struct xarray *buffer,
			       struct address_space *mapping,
			       pgoff_t index, pgoff_t to, gfp_t gfp_mask)
{
	struct folio *folio;
	int ret;

	if (to + 1 == index) /* Page range is inclusive */
		return 0;

	do {
		/* TODO: Figure out what order folio can be allocated here */
		folio = filemap_alloc_folio(readahead_gfp_mask(mapping), 0);
		if (!folio)
			return -ENOMEM;
		folio->index = index;
		ret = netfs_xa_store_and_mark(buffer, index, folio,
					      NETFS_FLAG_PUT_MARK, gfp_mask);
		if (ret < 0) {
			folio_put(folio);
			return ret;
		}

		index += folio_nr_pages(folio);
	} while (index <= to && index != 0);

	return 0;
}

/*
 * Clear an xarray buffer, putting a ref on the folios that have
 * NETFS_BUF_PUT_MARK set.
 */
void netfs_clear_buffer(struct xarray *buffer)
{
	struct folio *folio;
	XA_STATE(xas, buffer, 0);

	rcu_read_lock();
	xas_for_each_marked(&xas, folio, ULONG_MAX, NETFS_BUF_PUT_MARK) {
		folio_put(folio);
	}
	rcu_read_unlock();
	xa_destroy(buffer);
}

/**
 * netfs_dirty_folio - Mark folio dirty and pin a cache object for writeback
 * @mapping: The mapping the folio belongs to.
 * @folio: The folio being dirtied.
 *
 * Set the dirty flag on a folio and pin an in-use cache object in memory so
 * that writeback can later write to it.  This is intended to be called from
 * the filesystem's ->dirty_folio() method.
 *
 * Return: true if the dirty flag was set on the folio, false otherwise.
 */
bool netfs_dirty_folio(struct address_space *mapping, struct folio *folio)
{
	struct inode *inode = mapping->host;
	struct netfs_inode *ictx = netfs_inode(inode);
	struct fscache_cookie *cookie = netfs_i_cookie(ictx);
	bool need_use = false;

	_enter("");

	if (!filemap_dirty_folio(mapping, folio))
		return false;
	if (!fscache_cookie_valid(cookie))
		return true;

	if (!(inode->i_state & I_PINNING_NETFS_WB)) {
		spin_lock(&inode->i_lock);
		if (!(inode->i_state & I_PINNING_NETFS_WB)) {
			inode->i_state |= I_PINNING_NETFS_WB;
			need_use = true;
		}
		spin_unlock(&inode->i_lock);

		if (need_use)
			fscache_use_cookie(cookie, true);
	}
	return true;
}
EXPORT_SYMBOL(netfs_dirty_folio);

/**
 * netfs_unpin_writeback - Unpin writeback resources
 * @inode: The inode on which the cookie resides
 * @wbc: The writeback control
 *
 * Unpin the writeback resources pinned by netfs_dirty_folio().  This is
 * intended to be called as/by the netfs's ->write_inode() method.
 */
int netfs_unpin_writeback(struct inode *inode, struct writeback_control *wbc)
{
	struct fscache_cookie *cookie = netfs_i_cookie(netfs_inode(inode));

	if (wbc->unpinned_netfs_wb)
		fscache_unuse_cookie(cookie, NULL, NULL);
	return 0;
}
EXPORT_SYMBOL(netfs_unpin_writeback);

/**
 * netfs_clear_inode_writeback - Clear writeback resources pinned by an inode
 * @inode: The inode to clean up
 * @aux: Auxiliary data to apply to the inode
 *
 * Clear any writeback resources held by an inode when the inode is evicted.
 * This must be called before clear_inode() is called.
 */
void netfs_clear_inode_writeback(struct inode *inode, const void *aux)
{
	struct fscache_cookie *cookie = netfs_i_cookie(netfs_inode(inode));

	if (inode->i_state & I_PINNING_NETFS_WB) {
		loff_t i_size = i_size_read(inode);
		fscache_unuse_cookie(cookie, aux, &i_size);
	}
}
EXPORT_SYMBOL(netfs_clear_inode_writeback);

/**
 * netfs_invalidate_folio - Invalidate or partially invalidate a folio
 * @folio: Folio proposed for release
 * @offset: Offset of the invalidated region
 * @length: Length of the invalidated region
 *
 * Invalidate part or all of a folio for a network filesystem.  The folio will
 * be removed afterwards if the invalidated region covers the entire folio.
 */
void netfs_invalidate_folio(struct folio *folio, size_t offset, size_t length)
{
	struct netfs_folio *finfo;
	size_t flen = folio_size(folio);

	_enter("{%lx},%zx,%zx", folio->index, offset, length);

	if (!folio_test_private(folio))
		return;

	finfo = netfs_folio_info(folio);

	if (offset == 0 && length >= flen)
		goto erase_completely;

	if (finfo) {
		/* We have a partially uptodate page from a streaming write. */
		unsigned int fstart = finfo->dirty_offset;
		unsigned int fend = fstart + finfo->dirty_len;
		unsigned int end = offset + length;

		if (offset >= fend)
			return;
		if (end <= fstart)
			return;
		if (offset <= fstart && end >= fend)
			goto erase_completely;
		if (offset <= fstart && end > fstart)
			goto reduce_len;
		if (offset > fstart && end >= fend)
			goto move_start;
		/* A partial write was split.  The caller has already zeroed
		 * it, so just absorb the hole.
		 */
	}
	return;

erase_completely:
	netfs_put_group(netfs_folio_group(folio));
	folio_detach_private(folio);
	folio_clear_uptodate(folio);
	kfree(finfo);
	return;
reduce_len:
	finfo->dirty_len = offset + length - finfo->dirty_offset;
	return;
move_start:
	finfo->dirty_len -= offset - finfo->dirty_offset;
	finfo->dirty_offset = offset;
}
EXPORT_SYMBOL(netfs_invalidate_folio);

/**
 * netfs_release_folio - Try to release a folio
 * @folio: Folio proposed for release
 * @gfp: Flags qualifying the release
 *
 * Request release of a folio and clean up its private state if it's not busy.
 * Returns true if the folio can now be released, false if not
 */
bool netfs_release_folio(struct folio *folio, gfp_t gfp)
{
	struct netfs_inode *ctx = netfs_inode(folio_inode(folio));
	unsigned long long end;

	end = folio_pos(folio) + folio_size(folio);
	if (end > ctx->zero_point)
		ctx->zero_point = end;

	if (folio_test_private(folio))
		return false;
	fscache_note_page_release(netfs_i_cookie(ctx));
	return true;
}
EXPORT_SYMBOL(netfs_release_folio);
