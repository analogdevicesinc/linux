// SPDX-License-Identifier: GPL-2.0+
/*
 * NILFS B-tree node cache
 *
 * Copyright (C) 2005-2008 Nippon Telegraph and Telephone Corporation.
 *
 * Originally written by Seiji Kihara.
 * Fully revised by Ryusuke Konishi for stabilization and simplification.
 *
 */

#include <linux/types.h>
#include <linux/buffer_head.h>
#include <linux/mm.h>
#include <linux/backing-dev.h>
#include <linux/gfp.h>
#include "nilfs.h"
#include "mdt.h"
#include "dat.h"
#include "page.h"
#include "btnode.h"


/**
 * nilfs_init_btnc_inode - initialize B-tree node cache inode
 * @btnc_inode: inode to be initialized
 *
 * nilfs_init_btnc_inode() sets up an inode for B-tree node cache.
 */
void nilfs_init_btnc_inode(struct inode *btnc_inode)
{
	struct nilfs_inode_info *ii = NILFS_I(btnc_inode);

	btnc_inode->i_mode = S_IFREG;
	ii->i_flags = 0;
	memset(&ii->i_bmap_data, 0, sizeof(struct nilfs_bmap));
	mapping_set_gfp_mask(btnc_inode->i_mapping, GFP_NOFS);
}

void nilfs_btnode_cache_clear(struct address_space *btnc)
{
	invalidate_mapping_pages(btnc, 0, -1);
	truncate_inode_pages(btnc, 0);
}

struct buffer_head *
nilfs_btnode_create_block(struct address_space *btnc, __u64 blocknr)
{
	struct inode *inode = btnc->host;
	struct buffer_head *bh;

	bh = nilfs_grab_buffer(inode, btnc, blocknr, BIT(BH_NILFS_Node));
	if (unlikely(!bh))
		return ERR_PTR(-ENOMEM);

	if (unlikely(buffer_mapped(bh) || buffer_uptodate(bh) ||
		     buffer_dirty(bh))) {
		/*
		 * The block buffer at the specified new address was already
		 * in use.  This can happen if it is a virtual block number
		 * and has been reallocated due to corruption of the bitmap
		 * used to manage its allocation state (if not, the buffer
		 * clearing of an abandoned b-tree node is missing somewhere).
		 */
		nilfs_error(inode->i_sb,
			    "state inconsistency probably due to duplicate use of b-tree node block address %llu (ino=%lu)",
			    (unsigned long long)blocknr, inode->i_ino);
		goto failed;
	}
	memset(bh->b_data, 0, i_blocksize(inode));
	bh->b_blocknr = blocknr;
	set_buffer_mapped(bh);
	set_buffer_uptodate(bh);

	folio_unlock(bh->b_folio);
	folio_put(bh->b_folio);
	return bh;

failed:
	folio_unlock(bh->b_folio);
	folio_put(bh->b_folio);
	brelse(bh);
	return ERR_PTR(-EIO);
}

int nilfs_btnode_submit_block(struct address_space *btnc, __u64 blocknr,
			      sector_t pblocknr, blk_opf_t opf,
			      struct buffer_head **pbh, sector_t *submit_ptr)
{
	struct buffer_head *bh;
	struct inode *inode = btnc->host;
	struct folio *folio;
	int err;

	bh = nilfs_grab_buffer(inode, btnc, blocknr, BIT(BH_NILFS_Node));
	if (unlikely(!bh))
		return -ENOMEM;

	err = -EEXIST; /* internal code */
	folio = bh->b_folio;

	if (buffer_uptodate(bh) || buffer_dirty(bh))
		goto found;

	if (pblocknr == 0) {
		pblocknr = blocknr;
		if (inode->i_ino != NILFS_DAT_INO) {
			struct the_nilfs *nilfs = inode->i_sb->s_fs_info;

			/* blocknr is a virtual block number */
			err = nilfs_dat_translate(nilfs->ns_dat, blocknr,
						  &pblocknr);
			if (unlikely(err)) {
				brelse(bh);
				goto out_locked;
			}
		}
	}

	if (opf & REQ_RAHEAD) {
		if (pblocknr != *submit_ptr + 1 || !trylock_buffer(bh)) {
			err = -EBUSY; /* internal code */
			brelse(bh);
			goto out_locked;
		}
	} else { /* opf == REQ_OP_READ */
		lock_buffer(bh);
	}
	if (buffer_uptodate(bh)) {
		unlock_buffer(bh);
		err = -EEXIST; /* internal code */
		goto found;
	}
	set_buffer_mapped(bh);
	bh->b_blocknr = pblocknr; /* set block address for read */
	bh->b_end_io = end_buffer_read_sync;
	get_bh(bh);
	submit_bh(opf, bh);
	bh->b_blocknr = blocknr; /* set back to the given block address */
	*submit_ptr = pblocknr;
	err = 0;
found:
	*pbh = bh;

out_locked:
	folio_unlock(folio);
	folio_put(folio);
	return err;
}

/**
 * nilfs_btnode_delete - delete B-tree node buffer
 * @bh: buffer to be deleted
 *
 * nilfs_btnode_delete() invalidates the specified buffer and delete the page
 * including the buffer if the page gets unbusy.
 */
void nilfs_btnode_delete(struct buffer_head *bh)
{
	struct address_space *mapping;
	struct folio *folio = bh->b_folio;
	pgoff_t index = folio->index;
	int still_dirty;

	folio_get(folio);
	folio_lock(folio);
	folio_wait_writeback(folio);

	nilfs_forget_buffer(bh);
	still_dirty = folio_test_dirty(folio);
	mapping = folio->mapping;
	folio_unlock(folio);
	folio_put(folio);

	if (!still_dirty && mapping)
		invalidate_inode_pages2_range(mapping, index, index);
}

/**
 * nilfs_btnode_prepare_change_key - prepare to change the search key of a
 *                                   b-tree node block
 * @btnc: page cache in which the b-tree node block is buffered
 * @ctxt: structure for exchanging context information for key change
 *
 * nilfs_btnode_prepare_change_key() prepares to move the contents of the
 * b-tree node block of the old key given in the "oldkey" member of @ctxt to
 * the position of the new key given in the "newkey" member of @ctxt in the
 * page cache @btnc.  Here, the key of the block is an index in units of
 * blocks, and if the page and block sizes match, it matches the page index
 * in the page cache.
 *
 * If the page size and block size match, this function attempts to move the
 * entire folio, and in preparation for this, inserts the original folio into
 * the new index of the cache.  If this insertion fails or if the page size
 * and block size are different, it falls back to a copy preparation using
 * nilfs_btnode_create_block(), inserts a new block at the position
 * corresponding to "newkey", and stores the buffer head pointer in the
 * "newbh" member of @ctxt.
 *
 * Note that the current implementation does not support folio sizes larger
 * than the page size.
 *
 * Return: 0 on success, or the following negative error code on failure.
 * * %-EIO	- I/O error (metadata corruption).
 * * %-ENOMEM	- Insufficient memory available.
 */
int nilfs_btnode_prepare_change_key(struct address_space *btnc,
				    struct nilfs_btnode_chkey_ctxt *ctxt)
{
	struct buffer_head *obh, *nbh;
	struct inode *inode = btnc->host;
	__u64 oldkey = ctxt->oldkey, newkey = ctxt->newkey;
	int err;

	if (oldkey == newkey)
		return 0;

	obh = ctxt->bh;
	ctxt->newbh = NULL;

	if (inode->i_blkbits == PAGE_SHIFT) {
		struct folio *ofolio = obh->b_folio;
		folio_lock(ofolio);
retry:
		/* BUG_ON(oldkey != obh->b_folio->index); */
		if (unlikely(oldkey != ofolio->index))
			NILFS_FOLIO_BUG(ofolio,
				       "invalid oldkey %lld (newkey=%lld)",
				       (unsigned long long)oldkey,
				       (unsigned long long)newkey);

		xa_lock_irq(&btnc->i_pages);
		err = __xa_insert(&btnc->i_pages, newkey, ofolio, GFP_NOFS);
		xa_unlock_irq(&btnc->i_pages);
		/*
		 * Note: folio->index will not change to newkey until
		 * nilfs_btnode_commit_change_key() will be called.
		 * To protect the folio in intermediate state, the folio lock
		 * is held.
		 */
		if (!err)
			return 0;
		else if (err != -EBUSY)
			goto failed_unlock;

		err = invalidate_inode_pages2_range(btnc, newkey, newkey);
		if (!err)
			goto retry;
		/* fallback to copy mode */
		folio_unlock(ofolio);
	}

	nbh = nilfs_btnode_create_block(btnc, newkey);
	if (IS_ERR(nbh))
		return PTR_ERR(nbh);

	BUG_ON(nbh == obh);
	ctxt->newbh = nbh;
	return 0;

 failed_unlock:
	folio_unlock(obh->b_folio);
	return err;
}

/**
 * nilfs_btnode_commit_change_key - commit the change of the search key of
 *                                  a b-tree node block
 * @btnc: page cache in which the b-tree node block is buffered
 * @ctxt: structure for exchanging context information for key change
 *
 * nilfs_btnode_commit_change_key() executes the key change based on the
 * context @ctxt prepared by nilfs_btnode_prepare_change_key().  If no valid
 * block buffer is prepared in "newbh" of @ctxt (i.e., a full folio move),
 * this function removes the folio from the old index and completes the move.
 * Otherwise, it copies the block data and inherited flag states of "oldbh"
 * to "newbh" and clears the "oldbh" from the cache.  In either case, the
 * relocated buffer is marked as dirty.
 *
 * As with nilfs_btnode_prepare_change_key(), the current implementation does
 * not support folio sizes larger than the page size.
 */
void nilfs_btnode_commit_change_key(struct address_space *btnc,
				    struct nilfs_btnode_chkey_ctxt *ctxt)
{
	struct buffer_head *obh = ctxt->bh, *nbh = ctxt->newbh;
	__u64 oldkey = ctxt->oldkey, newkey = ctxt->newkey;
	struct folio *ofolio;

	if (oldkey == newkey)
		return;

	if (nbh == NULL) {	/* blocksize == pagesize */
		ofolio = obh->b_folio;
		if (unlikely(oldkey != ofolio->index))
			NILFS_FOLIO_BUG(ofolio,
				       "invalid oldkey %lld (newkey=%lld)",
				       (unsigned long long)oldkey,
				       (unsigned long long)newkey);
		mark_buffer_dirty(obh);

		xa_lock_irq(&btnc->i_pages);
		__xa_erase(&btnc->i_pages, oldkey);
		__xa_set_mark(&btnc->i_pages, newkey, PAGECACHE_TAG_DIRTY);
		xa_unlock_irq(&btnc->i_pages);

		ofolio->index = obh->b_blocknr = newkey;
		folio_unlock(ofolio);
	} else {
		nilfs_copy_buffer(nbh, obh);
		mark_buffer_dirty(nbh);

		nbh->b_blocknr = newkey;
		ctxt->bh = nbh;
		nilfs_btnode_delete(obh); /* will decrement bh->b_count */
	}
}

/**
 * nilfs_btnode_abort_change_key - abort the change of the search key of a
 *                                 b-tree node block
 * @btnc: page cache in which the b-tree node block is buffered
 * @ctxt: structure for exchanging context information for key change
 *
 * nilfs_btnode_abort_change_key() cancels the key change associated with the
 * context @ctxt prepared via nilfs_btnode_prepare_change_key() and performs
 * any necessary cleanup.  If no valid block buffer is prepared in "newbh" of
 * @ctxt, this function removes the folio from the destination index and aborts
 * the move.  Otherwise, it clears "newbh" from the cache.
 *
 * As with nilfs_btnode_prepare_change_key(), the current implementation does
 * not support folio sizes larger than the page size.
 */
void nilfs_btnode_abort_change_key(struct address_space *btnc,
				   struct nilfs_btnode_chkey_ctxt *ctxt)
{
	struct buffer_head *nbh = ctxt->newbh;
	__u64 oldkey = ctxt->oldkey, newkey = ctxt->newkey;

	if (oldkey == newkey)
		return;

	if (nbh == NULL) {	/* blocksize == pagesize */
		xa_erase_irq(&btnc->i_pages, newkey);
		folio_unlock(ctxt->bh->b_folio);
	} else {
		/*
		 * When canceling a buffer that a prepare operation has
		 * allocated to copy a node block to another location, use
		 * nilfs_btnode_delete() to initialize and release the buffer
		 * so that the buffer flags will not be in an inconsistent
		 * state when it is reallocated.
		 */
		nilfs_btnode_delete(nbh);
	}
}
