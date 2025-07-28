// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2000-2005 Silicon Graphics, Inc.
 * All Rights Reserved.
 */
#include "xfs.h"
#include "xfs_fs.h"
#include "xfs_shared.h"
#include "xfs_format.h"
#include "xfs_log_format.h"
#include "xfs_trans_resv.h"
#include "xfs_bit.h"
#include "xfs_mount.h"
#include "xfs_inode.h"
#include "xfs_alloc.h"
#include "xfs_bmap.h"
#include "xfs_bmap_btree.h"
#include "xfs_bmap_util.h"
#include "xfs_trans.h"
#include "xfs_trans_space.h"
#include "xfs_icache.h"
#include "xfs_rtalloc.h"
#include "xfs_sb.h"
#include "xfs_rtbitmap.h"
#include "xfs_quota.h"
#include "xfs_log_priv.h"
#include "xfs_health.h"

/*
 * Return whether there are any free extents in the size range given
 * by low and high, for the bitmap block bbno.
 */
STATIC int
xfs_rtany_summary(
	struct xfs_rtalloc_args	*args,
	int			low,	/* low log2 extent size */
	int			high,	/* high log2 extent size */
	xfs_fileoff_t		bbno,	/* bitmap block number */
	int			*maxlog) /* out: max log2 extent size free */
{
	struct xfs_mount	*mp = args->mp;
	int			error;
	int			log;	/* loop counter, log2 of ext. size */
	xfs_suminfo_t		sum;	/* summary data */

	/* There are no extents at levels >= m_rsum_cache[bbno]. */
	if (mp->m_rsum_cache) {
		high = min(high, mp->m_rsum_cache[bbno] - 1);
		if (low > high) {
			*maxlog = -1;
			return 0;
		}
	}

	/*
	 * Loop over logs of extent sizes.
	 */
	for (log = high; log >= low; log--) {
		/*
		 * Get one summary datum.
		 */
		error = xfs_rtget_summary(args, log, bbno, &sum);
		if (error) {
			return error;
		}
		/*
		 * If there are any, return success.
		 */
		if (sum) {
			*maxlog = log;
			goto out;
		}
	}
	/*
	 * Found nothing, return failure.
	 */
	*maxlog = -1;
out:
	/* There were no extents at levels > log. */
	if (mp->m_rsum_cache && log + 1 < mp->m_rsum_cache[bbno])
		mp->m_rsum_cache[bbno] = log + 1;
	return 0;
}


/*
 * Copy and transform the summary file, given the old and new
 * parameters in the mount structures.
 */
STATIC int
xfs_rtcopy_summary(
	struct xfs_rtalloc_args	*oargs,
	struct xfs_rtalloc_args	*nargs)
{
	xfs_fileoff_t		bbno;	/* bitmap block number */
	int			error;
	int			log;	/* summary level number (log length) */
	xfs_suminfo_t		sum;	/* summary data */

	for (log = oargs->mp->m_rsumlevels - 1; log >= 0; log--) {
		for (bbno = oargs->mp->m_sb.sb_rbmblocks - 1;
		     (xfs_srtblock_t)bbno >= 0;
		     bbno--) {
			error = xfs_rtget_summary(oargs, log, bbno, &sum);
			if (error)
				goto out;
			if (sum == 0)
				continue;
			error = xfs_rtmodify_summary(oargs, log, bbno, -sum);
			if (error)
				goto out;
			error = xfs_rtmodify_summary(nargs, log, bbno, sum);
			if (error)
				goto out;
			ASSERT(sum > 0);
		}
	}
	error = 0;
out:
	xfs_rtbuf_cache_relse(oargs);
	return 0;
}
/*
 * Mark an extent specified by start and len allocated.
 * Updates all the summary information as well as the bitmap.
 */
STATIC int
xfs_rtallocate_range(
	struct xfs_rtalloc_args	*args,
	xfs_rtxnum_t		start,	/* start rtext to allocate */
	xfs_rtxlen_t		len)	/* in/out: summary block number */
{
	struct xfs_mount	*mp = args->mp;
	xfs_rtxnum_t		end;	/* end of the allocated rtext */
	int			error;
	xfs_rtxnum_t		postblock = 0; /* first rtext allocated > end */
	xfs_rtxnum_t		preblock = 0; /* first rtext allocated < start */

	end = start + len - 1;
	/*
	 * Assume we're allocating out of the middle of a free extent.
	 * We need to find the beginning and end of the extent so we can
	 * properly update the summary.
	 */
	error = xfs_rtfind_back(args, start, &preblock);
	if (error)
		return error;

	/*
	 * Find the next allocated block (end of free extent).
	 */
	error = xfs_rtfind_forw(args, end, mp->m_sb.sb_rextents - 1,
			&postblock);
	if (error)
		return error;

	/*
	 * Decrement the summary information corresponding to the entire
	 * (old) free extent.
	 */
	error = xfs_rtmodify_summary(args,
			xfs_highbit64(postblock + 1 - preblock),
			xfs_rtx_to_rbmblock(mp, preblock), -1);
	if (error)
		return error;

	/*
	 * If there are blocks not being allocated at the front of the
	 * old extent, add summary data for them to be free.
	 */
	if (preblock < start) {
		error = xfs_rtmodify_summary(args,
				xfs_highbit64(start - preblock),
				xfs_rtx_to_rbmblock(mp, preblock), 1);
		if (error)
			return error;
	}

	/*
	 * If there are blocks not being allocated at the end of the
	 * old extent, add summary data for them to be free.
	 */
	if (postblock > end) {
		error = xfs_rtmodify_summary(args,
				xfs_highbit64(postblock - end),
				xfs_rtx_to_rbmblock(mp, end + 1), 1);
		if (error)
			return error;
	}

	/*
	 * Modify the bitmap to mark this extent allocated.
	 */
	return xfs_rtmodify_range(args, start, len, 0);
}

/* Reduce @rtxlen until it is a multiple of @prod. */
static inline xfs_rtxlen_t
xfs_rtalloc_align_len(
	xfs_rtxlen_t	rtxlen,
	xfs_rtxlen_t	prod)
{
	if (unlikely(prod > 1))
		return rounddown(rtxlen, prod);
	return rtxlen;
}

/*
 * Make sure we don't run off the end of the rt volume.  Be careful that
 * adjusting maxlen downwards doesn't cause us to fail the alignment checks.
 */
static inline xfs_rtxlen_t
xfs_rtallocate_clamp_len(
	struct xfs_mount	*mp,
	xfs_rtxnum_t		startrtx,
	xfs_rtxlen_t		rtxlen,
	xfs_rtxlen_t		prod)
{
	xfs_rtxlen_t		ret;

	ret = min(mp->m_sb.sb_rextents, startrtx + rtxlen) - startrtx;
	return xfs_rtalloc_align_len(ret, prod);
}

/*
 * Attempt to allocate an extent minlen<=len<=maxlen starting from
 * bitmap block bbno.  If we don't get maxlen then use prod to trim
 * the length, if given.  Returns error; returns starting block in *rtx.
 * The lengths are all in rtextents.
 */
STATIC int
xfs_rtallocate_extent_block(
	struct xfs_rtalloc_args	*args,
	xfs_fileoff_t		bbno,	/* bitmap block number */
	xfs_rtxlen_t		minlen,	/* minimum length to allocate */
	xfs_rtxlen_t		maxlen,	/* maximum length to allocate */
	xfs_rtxlen_t		*len,	/* out: actual length allocated */
	xfs_rtxnum_t		*nextp,	/* out: next rtext to try */
	xfs_rtxlen_t		prod,	/* extent product factor */
	xfs_rtxnum_t		*rtx)	/* out: start rtext allocated */
{
	struct xfs_mount	*mp = args->mp;
	xfs_rtxnum_t		besti = -1; /* best rtext found so far */
	xfs_rtxnum_t		end;	/* last rtext in chunk */
	xfs_rtxnum_t		i;	/* current rtext trying */
	xfs_rtxnum_t		next;	/* next rtext to try */
	xfs_rtxlen_t		scanlen; /* number of free rtx to look for */
	xfs_rtxlen_t		bestlen = 0; /* best length found so far */
	int			stat;	/* status from internal calls */
	int			error;

	/*
	 * Loop over all the extents starting in this bitmap block up to the
	 * end of the rt volume, looking for one that's long enough.
	 */
	end = min(mp->m_sb.sb_rextents, xfs_rbmblock_to_rtx(mp, bbno + 1)) - 1;
	for (i = xfs_rbmblock_to_rtx(mp, bbno); i <= end; i++) {
		/* Make sure we don't scan off the end of the rt volume. */
		scanlen = xfs_rtallocate_clamp_len(mp, i, maxlen, prod);
		if (scanlen < minlen)
			break;

		/*
		 * See if there's a free extent of scanlen starting at i.
		 * If it's not so then next will contain the first non-free.
		 */
		error = xfs_rtcheck_range(args, i, scanlen, 1, &next, &stat);
		if (error)
			return error;
		if (stat) {
			/*
			 * i to scanlen is all free, allocate and return that.
			 */
			*len = scanlen;
			*rtx = i;
			return 0;
		}

		/*
		 * In the case where we have a variable-sized allocation
		 * request, figure out how big this free piece is,
		 * and if it's big enough for the minimum, and the best
		 * so far, remember it.
		 */
		if (minlen < maxlen) {
			xfs_rtxnum_t	thislen;	/* this extent size */

			thislen = next - i;
			if (thislen >= minlen && thislen > bestlen) {
				besti = i;
				bestlen = thislen;
			}
		}
		/*
		 * If not done yet, find the start of the next free space.
		 */
		if (next >= end)
			break;
		error = xfs_rtfind_forw(args, next, end, &i);
		if (error)
			return error;
	}

	/* Searched the whole thing & didn't find a maxlen free extent. */
	if (besti == -1)
		goto nospace;

	/*
	 * Ensure bestlen is a multiple of prod, but don't return a too-short
	 * extent.
	 */
	bestlen = xfs_rtalloc_align_len(bestlen, prod);
	if (bestlen < minlen)
		goto nospace;

	/*
	 * Pick besti for bestlen & return that.
	 */
	*len = bestlen;
	*rtx = besti;
	return 0;
nospace:
	/* Allocation failed.  Set *nextp to the next block to try. */
	*nextp = next;
	return -ENOSPC;
}

/*
 * Allocate an extent of length minlen<=len<=maxlen, starting at block
 * bno.  If we don't get maxlen then use prod to trim the length, if given.
 * Returns error; returns starting block in *rtx.
 * The lengths are all in rtextents.
 */
STATIC int
xfs_rtallocate_extent_exact(
	struct xfs_rtalloc_args	*args,
	xfs_rtxnum_t		start,	/* starting rtext number to allocate */
	xfs_rtxlen_t		minlen,	/* minimum length to allocate */
	xfs_rtxlen_t		maxlen,	/* maximum length to allocate */
	xfs_rtxlen_t		*len,	/* out: actual length allocated */
	xfs_rtxlen_t		prod,	/* extent product factor */
	xfs_rtxnum_t		*rtx)	/* out: start rtext allocated */
{
	struct xfs_mount	*mp = args->mp;
	xfs_rtxnum_t		next;	/* next rtext to try (dummy) */
	xfs_rtxlen_t		alloclen; /* candidate length */
	xfs_rtxlen_t		scanlen; /* number of free rtx to look for */
	int			isfree;	/* extent is free */
	int			error;

	ASSERT(minlen % prod == 0);
	ASSERT(maxlen % prod == 0);

	/* Make sure we don't run off the end of the rt volume. */
	scanlen = xfs_rtallocate_clamp_len(mp, start, maxlen, prod);
	if (scanlen < minlen)
		return -ENOSPC;

	/* Check if the range in question (for scanlen) is free. */
	error = xfs_rtcheck_range(args, start, scanlen, 1, &next, &isfree);
	if (error)
		return error;

	if (isfree) {
		/* start to scanlen is all free; allocate it. */
		*len = scanlen;
		*rtx = start;
		return 0;
	}

	/*
	 * If not, allocate what there is, if it's at least minlen.
	 */
	alloclen = next - start;
	if (alloclen < minlen)
		return -ENOSPC;

	/* Ensure alloclen is a multiple of prod. */
	alloclen = xfs_rtalloc_align_len(alloclen, prod);
	if (alloclen < minlen)
		return -ENOSPC;

	*len = alloclen;
	*rtx = start;
	return 0;
}

/*
 * Allocate an extent of length minlen<=len<=maxlen, starting as near
 * to start as possible.  If we don't get maxlen then use prod to trim
 * the length, if given.  The lengths are all in rtextents.
 */
STATIC int
xfs_rtallocate_extent_near(
	struct xfs_rtalloc_args	*args,
	xfs_rtxnum_t		start,	/* starting rtext number to allocate */
	xfs_rtxlen_t		minlen,	/* minimum length to allocate */
	xfs_rtxlen_t		maxlen,	/* maximum length to allocate */
	xfs_rtxlen_t		*len,	/* out: actual length allocated */
	xfs_rtxlen_t		prod,	/* extent product factor */
	xfs_rtxnum_t		*rtx)	/* out: start rtext allocated */
{
	struct xfs_mount	*mp = args->mp;
	int			maxlog;	/* max useful extent from summary */
	xfs_fileoff_t		bbno;	/* bitmap block number */
	int			error;
	int			i;	/* bitmap block offset (loop control) */
	int			j;	/* secondary loop control */
	int			log2len; /* log2 of minlen */
	xfs_rtxnum_t		n;	/* next rtext to try */

	ASSERT(minlen % prod == 0);
	ASSERT(maxlen % prod == 0);

	/*
	 * If the block number given is off the end, silently set it to
	 * the last block.
	 */
	if (start >= mp->m_sb.sb_rextents)
		start = mp->m_sb.sb_rextents - 1;

	/*
	 * Try the exact allocation first.
	 */
	error = xfs_rtallocate_extent_exact(args, start, minlen, maxlen, len,
			prod, rtx);
	if (error != -ENOSPC)
		return error;

	bbno = xfs_rtx_to_rbmblock(mp, start);
	i = 0;
	j = -1;
	ASSERT(minlen != 0);
	log2len = xfs_highbit32(minlen);
	/*
	 * Loop over all bitmap blocks (bbno + i is current block).
	 */
	for (;;) {
		/*
		 * Get summary information of extents of all useful levels
		 * starting in this bitmap block.
		 */
		error = xfs_rtany_summary(args, log2len, mp->m_rsumlevels - 1,
				bbno + i, &maxlog);
		if (error)
			return error;

		/*
		 * If there are any useful extents starting here, try
		 * allocating one.
		 */
		if (maxlog >= 0) {
			xfs_extlen_t maxavail =
				min_t(xfs_rtblock_t, maxlen,
				      (1ULL << (maxlog + 1)) - 1);
			/*
			 * On the positive side of the starting location.
			 */
			if (i >= 0) {
				/*
				 * Try to allocate an extent starting in
				 * this block.
				 */
				error = xfs_rtallocate_extent_block(args,
						bbno + i, minlen, maxavail, len,
						&n, prod, rtx);
				if (error != -ENOSPC)
					return error;
			}
			/*
			 * On the negative side of the starting location.
			 */
			else {		/* i < 0 */
				int	maxblocks;

				/*
				 * Loop backwards to find the end of the extent
				 * we found in the realtime summary.
				 *
				 * maxblocks is the maximum possible number of
				 * bitmap blocks from the start of the extent
				 * to the end of the extent.
				 */
				if (maxlog == 0)
					maxblocks = 0;
				else if (maxlog < mp->m_blkbit_log)
					maxblocks = 1;
				else
					maxblocks = 2 << (maxlog - mp->m_blkbit_log);

				/*
				 * We need to check bbno + i + maxblocks down to
				 * bbno + i. We already checked bbno down to
				 * bbno + j + 1, so we don't need to check those
				 * again.
				 */
				j = min(i + maxblocks, j);
				for (; j >= i; j--) {
					error = xfs_rtallocate_extent_block(args,
							bbno + j, minlen,
							maxavail, len, &n, prod,
							rtx);
					if (error != -ENOSPC)
						return error;
				}
			}
		}
		/*
		 * Loop control.  If we were on the positive side, and there's
		 * still more blocks on the negative side, go there.
		 */
		if (i > 0 && (int)bbno - i >= 0)
			i = -i;
		/*
		 * If positive, and no more negative, but there are more
		 * positive, go there.
		 */
		else if (i > 0 && (int)bbno + i < mp->m_sb.sb_rbmblocks - 1)
			i++;
		/*
		 * If negative or 0 (just started), and there are positive
		 * blocks to go, go there.  The 0 case moves to block 1.
		 */
		else if (i <= 0 && (int)bbno - i < mp->m_sb.sb_rbmblocks - 1)
			i = 1 - i;
		/*
		 * If negative or 0 and there are more negative blocks,
		 * go there.
		 */
		else if (i <= 0 && (int)bbno + i > 0)
			i--;
		/*
		 * Must be done.  Return failure.
		 */
		else
			break;
	}
	return -ENOSPC;
}

static int
xfs_rtalloc_sumlevel(
	struct xfs_rtalloc_args	*args,
	int			l,	/* level number */
	xfs_rtxlen_t		minlen,	/* minimum length to allocate */
	xfs_rtxlen_t		maxlen,	/* maximum length to allocate */
	xfs_rtxlen_t		prod,	/* extent product factor */
	xfs_rtxlen_t		*len,	/* out: actual length allocated */
	xfs_rtxnum_t		*rtx)	/* out: start rtext allocated */
{
	xfs_fileoff_t		i;	/* bitmap block number */
	int			error;

	for (i = 0; i < args->mp->m_sb.sb_rbmblocks; i++) {
		xfs_suminfo_t	sum;	/* summary information for extents */
		xfs_rtxnum_t	n;	/* next rtext to be tried */

		error = xfs_rtget_summary(args, l, i, &sum);
		if (error)
			return error;

		/*
		 * Nothing there, on to the next block.
		 */
		if (!sum)
			continue;

		/*
		 * Try allocating the extent.
		 */
		error = xfs_rtallocate_extent_block(args, i, minlen, maxlen,
				len, &n, prod, rtx);
		if (error != -ENOSPC)
			return error;

		/*
		 * If the "next block to try" returned from the allocator is
		 * beyond the next bitmap block, skip to that bitmap block.
		 */
		if (xfs_rtx_to_rbmblock(args->mp, n) > i + 1)
			i = xfs_rtx_to_rbmblock(args->mp, n) - 1;
	}

	return -ENOSPC;
}

/*
 * Allocate an extent of length minlen<=len<=maxlen, with no position
 * specified.  If we don't get maxlen then use prod to trim
 * the length, if given.  The lengths are all in rtextents.
 */
STATIC int
xfs_rtallocate_extent_size(
	struct xfs_rtalloc_args	*args,
	xfs_rtxlen_t		minlen,	/* minimum length to allocate */
	xfs_rtxlen_t		maxlen,	/* maximum length to allocate */
	xfs_rtxlen_t		*len,	/* out: actual length allocated */
	xfs_rtxlen_t		prod,	/* extent product factor */
	xfs_rtxnum_t		*rtx)	/* out: start rtext allocated */
{
	int			error;
	int			l;	/* level number (loop control) */

	ASSERT(minlen % prod == 0);
	ASSERT(maxlen % prod == 0);
	ASSERT(maxlen != 0);

	/*
	 * Loop over all the levels starting with maxlen.
	 *
	 * At each level, look at all the bitmap blocks, to see if there are
	 * extents starting there that are long enough (>= maxlen).
	 *
	 * Note, only on the initial level can the allocation fail if the
	 * summary says there's an extent.
	 */
	for (l = xfs_highbit32(maxlen); l < args->mp->m_rsumlevels; l++) {
		error = xfs_rtalloc_sumlevel(args, l, minlen, maxlen, prod, len,
				rtx);
		if (error != -ENOSPC)
			return error;
	}

	/*
	 * Didn't find any maxlen blocks.  Try smaller ones, unless we are
	 * looking for a fixed size extent.
	 */
	if (minlen > --maxlen)
		return -ENOSPC;
	ASSERT(minlen != 0);
	ASSERT(maxlen != 0);

	/*
	 * Loop over sizes, from maxlen down to minlen.
	 *
	 * This time, when we do the allocations, allow smaller ones to succeed,
	 * but make sure the specified minlen/maxlen are in the possible range
	 * for this summary level.
	 */
	for (l = xfs_highbit32(maxlen); l >= xfs_highbit32(minlen); l--) {
		error = xfs_rtalloc_sumlevel(args, l,
				max_t(xfs_rtxlen_t, minlen, 1 << l),
				min_t(xfs_rtxlen_t, maxlen, (1 << (l + 1)) - 1),
				prod, len, rtx);
		if (error != -ENOSPC)
			return error;
	}

	return -ENOSPC;
}

static int
xfs_alloc_rsum_cache(
	struct xfs_mount	*mp,
	xfs_extlen_t		rbmblocks)
{
	/*
	 * The rsum cache is initialized to the maximum value, which is
	 * trivially an upper bound on the maximum level with any free extents.
	 */
	mp->m_rsum_cache = kvmalloc(rbmblocks, GFP_KERNEL);
	if (!mp->m_rsum_cache)
		return -ENOMEM;
	memset(mp->m_rsum_cache, -1, rbmblocks);
	return 0;
}

/*
 * If we changed the rt extent size (meaning there was no rt volume previously)
 * and the root directory had EXTSZINHERIT and RTINHERIT set, it's possible
 * that the extent size hint on the root directory is no longer congruent with
 * the new rt extent size.  Log the rootdir inode to fix this.
 */
static int
xfs_growfs_rt_fixup_extsize(
	struct xfs_mount	*mp)
{
	struct xfs_inode	*ip = mp->m_rootip;
	struct xfs_trans	*tp;
	int			error = 0;

	xfs_ilock(ip, XFS_IOLOCK_EXCL);
	if (!(ip->i_diflags & XFS_DIFLAG_RTINHERIT) ||
	    !(ip->i_diflags & XFS_DIFLAG_EXTSZINHERIT))
		goto out_iolock;

	error = xfs_trans_alloc_inode(ip, &M_RES(mp)->tr_ichange, 0, 0, false,
			&tp);
	if (error)
		goto out_iolock;

	xfs_trans_log_inode(tp, ip, XFS_ILOG_CORE);
	error = xfs_trans_commit(tp);
	xfs_iunlock(ip, XFS_ILOCK_EXCL);

out_iolock:
	xfs_iunlock(ip, XFS_IOLOCK_EXCL);
	return error;
}

static int
xfs_growfs_rt_bmblock(
	struct xfs_mount	*mp,
	xfs_rfsblock_t		nrblocks,
	xfs_agblock_t		rextsize,
	xfs_fileoff_t		bmbno)
{
	struct xfs_inode	*rbmip = mp->m_rbmip;
	struct xfs_inode	*rsumip = mp->m_rsumip;
	struct xfs_rtalloc_args	args = {
		.mp		= mp,
	};
	struct xfs_rtalloc_args	nargs = {
	};
	struct xfs_mount	*nmp;
	xfs_rfsblock_t		nrblocks_step;
	xfs_rtbxlen_t		freed_rtx;
	int			error;


	nrblocks_step = (bmbno + 1) * NBBY * mp->m_sb.sb_blocksize * rextsize;

	nmp = nargs.mp = kmemdup(mp, sizeof(*mp), GFP_KERNEL);
	if (!nmp)
		return -ENOMEM;

	/*
	 * Calculate new sb and mount fields for this round.
	 */
	nmp->m_sb.sb_rextsize = rextsize;
	xfs_mount_sb_set_rextsize(nmp, &nmp->m_sb);
	nmp->m_sb.sb_rbmblocks = bmbno + 1;
	nmp->m_sb.sb_rblocks = min(nrblocks, nrblocks_step);
	nmp->m_sb.sb_rextents = xfs_rtb_to_rtx(nmp, nmp->m_sb.sb_rblocks);
	nmp->m_sb.sb_rextslog = xfs_compute_rextslog(nmp->m_sb.sb_rextents);
	nmp->m_rsumlevels = nmp->m_sb.sb_rextslog + 1;
	nmp->m_rsumblocks = xfs_rtsummary_blockcount(mp, nmp->m_rsumlevels,
			nmp->m_sb.sb_rbmblocks);

	/*
	 * Recompute the growfsrt reservation from the new rsumsize, so that the
	 * transaction below use the new, potentially larger value.
	 * */
	xfs_trans_resv_calc(nmp, &nmp->m_resv);
	error = xfs_trans_alloc(mp, &M_RES(nmp)->tr_growrtfree, 0, 0, 0,
			&args.tp);
	if (error)
		goto out_free;
	nargs.tp = args.tp;

	xfs_rtbitmap_lock(mp);
	xfs_rtbitmap_trans_join(args.tp);

	/*
	 * Update the bitmap inode's size ondisk and incore.  We need to update
	 * the incore size so that inode inactivation won't punch what it thinks
	 * are "posteof" blocks.
	 */
	rbmip->i_disk_size = nmp->m_sb.sb_rbmblocks * nmp->m_sb.sb_blocksize;
	i_size_write(VFS_I(rbmip), rbmip->i_disk_size);
	xfs_trans_log_inode(args.tp, rbmip, XFS_ILOG_CORE);

	/*
	 * Update the summary inode's size.  We need to update the incore size
	 * so that inode inactivation won't punch what it thinks are "posteof"
	 * blocks.
	 */
	rsumip->i_disk_size = nmp->m_rsumblocks * nmp->m_sb.sb_blocksize;
	i_size_write(VFS_I(rsumip), rsumip->i_disk_size);
	xfs_trans_log_inode(args.tp, rsumip, XFS_ILOG_CORE);

	/*
	 * Copy summary data from old to new sizes when the real size (not
	 * block-aligned) changes.
	 */
	if (mp->m_sb.sb_rbmblocks != nmp->m_sb.sb_rbmblocks ||
	    mp->m_rsumlevels != nmp->m_rsumlevels) {
		error = xfs_rtcopy_summary(&args, &nargs);
		if (error)
			goto out_cancel;
	}

	/*
	 * Update superblock fields.
	 */
	if (nmp->m_sb.sb_rextsize != mp->m_sb.sb_rextsize)
		xfs_trans_mod_sb(args.tp, XFS_TRANS_SB_REXTSIZE,
			nmp->m_sb.sb_rextsize - mp->m_sb.sb_rextsize);
	if (nmp->m_sb.sb_rbmblocks != mp->m_sb.sb_rbmblocks)
		xfs_trans_mod_sb(args.tp, XFS_TRANS_SB_RBMBLOCKS,
			nmp->m_sb.sb_rbmblocks - mp->m_sb.sb_rbmblocks);
	if (nmp->m_sb.sb_rblocks != mp->m_sb.sb_rblocks)
		xfs_trans_mod_sb(args.tp, XFS_TRANS_SB_RBLOCKS,
			nmp->m_sb.sb_rblocks - mp->m_sb.sb_rblocks);
	if (nmp->m_sb.sb_rextents != mp->m_sb.sb_rextents)
		xfs_trans_mod_sb(args.tp, XFS_TRANS_SB_REXTENTS,
			nmp->m_sb.sb_rextents - mp->m_sb.sb_rextents);
	if (nmp->m_sb.sb_rextslog != mp->m_sb.sb_rextslog)
		xfs_trans_mod_sb(args.tp, XFS_TRANS_SB_REXTSLOG,
			nmp->m_sb.sb_rextslog - mp->m_sb.sb_rextslog);

	/*
	 * Free the new extent.
	 */
	freed_rtx = nmp->m_sb.sb_rextents - mp->m_sb.sb_rextents;
	error = xfs_rtfree_range(&nargs, mp->m_sb.sb_rextents, freed_rtx);
	xfs_rtbuf_cache_relse(&nargs);
	if (error)
		goto out_cancel;

	/*
	 * Mark more blocks free in the superblock.
	 */
	xfs_trans_mod_sb(args.tp, XFS_TRANS_SB_FREXTENTS, freed_rtx);

	/*
	 * Update the calculated values in the real mount structure.
	 */
	mp->m_rsumlevels = nmp->m_rsumlevels;
	mp->m_rsumblocks = nmp->m_rsumblocks;
	xfs_mount_sb_set_rextsize(mp, &mp->m_sb);

	/*
	 * Recompute the growfsrt reservation from the new rsumsize.
	 */
	xfs_trans_resv_calc(mp, &mp->m_resv);

	error = xfs_trans_commit(args.tp);
	if (error)
		goto out_free;

	/*
	 * Ensure the mount RT feature flag is now set.
	 */
	mp->m_features |= XFS_FEAT_REALTIME;

	kfree(nmp);
	return 0;

out_cancel:
	xfs_trans_cancel(args.tp);
out_free:
	kfree(nmp);
	return error;
}

/*
 * Calculate the last rbmblock currently used.
 *
 * This also deals with the case where there were no rtextents before.
 */
static xfs_fileoff_t
xfs_last_rt_bmblock(
	struct xfs_mount	*mp)
{
	xfs_fileoff_t		bmbno = mp->m_sb.sb_rbmblocks;

	/* Skip the current block if it is exactly full. */
	if (xfs_rtx_to_rbmword(mp, mp->m_sb.sb_rextents) != 0)
		bmbno--;
	return bmbno;
}

/*
 * Grow the realtime area of the filesystem.
 */
int
xfs_growfs_rt(
	xfs_mount_t	*mp,		/* mount point for filesystem */
	xfs_growfs_rt_t	*in)		/* growfs rt input struct */
{
	xfs_fileoff_t	bmbno;		/* bitmap block number */
	struct xfs_buf	*bp;		/* temporary buffer */
	int		error;		/* error return value */
	xfs_extlen_t	nrbmblocks;	/* new number of rt bitmap blocks */
	xfs_rtxnum_t	nrextents;	/* new number of realtime extents */
	xfs_extlen_t	nrsumblocks;	/* new number of summary blocks */
	xfs_extlen_t	rbmblocks;	/* current number of rt bitmap blocks */
	xfs_extlen_t	rsumblocks;	/* current number of rt summary blks */
	uint8_t		*rsum_cache;	/* old summary cache */
	xfs_agblock_t	old_rextsize = mp->m_sb.sb_rextsize;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	/* Needs to have been mounted with an rt device. */
	if (!XFS_IS_REALTIME_MOUNT(mp))
		return -EINVAL;

	if (!mutex_trylock(&mp->m_growlock))
		return -EWOULDBLOCK;
	/*
	 * Mount should fail if the rt bitmap/summary files don't load, but
	 * we'll check anyway.
	 */
	error = -EINVAL;
	if (!mp->m_rbmip || !mp->m_rsumip)
		goto out_unlock;

	/* Shrink not supported. */
	if (in->newblocks <= mp->m_sb.sb_rblocks)
		goto out_unlock;
	/* Can only change rt extent size when adding rt volume. */
	if (mp->m_sb.sb_rblocks > 0 && in->extsize != mp->m_sb.sb_rextsize)
		goto out_unlock;

	/* Range check the extent size. */
	if (XFS_FSB_TO_B(mp, in->extsize) > XFS_MAX_RTEXTSIZE ||
	    XFS_FSB_TO_B(mp, in->extsize) < XFS_MIN_RTEXTSIZE)
		goto out_unlock;

	/* Unsupported realtime features. */
	error = -EOPNOTSUPP;
	if (xfs_has_rmapbt(mp) || xfs_has_reflink(mp) || xfs_has_quota(mp))
		goto out_unlock;

	error = xfs_sb_validate_fsb_count(&mp->m_sb, in->newblocks);
	if (error)
		goto out_unlock;
	/*
	 * Read in the last block of the device, make sure it exists.
	 */
	error = xfs_buf_read_uncached(mp->m_rtdev_targp,
				XFS_FSB_TO_BB(mp, in->newblocks - 1),
				XFS_FSB_TO_BB(mp, 1), 0, &bp, NULL);
	if (error)
		goto out_unlock;
	xfs_buf_relse(bp);

	/*
	 * Calculate new parameters.  These are the final values to be reached.
	 */
	nrextents = div_u64(in->newblocks, in->extsize);
	if (nrextents == 0) {
		error = -EINVAL;
		goto out_unlock;
	}
	nrbmblocks = xfs_rtbitmap_blockcount(mp, nrextents);
	nrsumblocks = xfs_rtsummary_blockcount(mp,
			xfs_compute_rextslog(nrextents) + 1, nrbmblocks);

	/*
	 * New summary size can't be more than half the size of
	 * the log.  This prevents us from getting a log overflow,
	 * since we'll log basically the whole summary file at once.
	 */
	if (nrsumblocks > (mp->m_sb.sb_logblocks >> 1)) {
		error = -EINVAL;
		goto out_unlock;
	}

	/*
	 * Get the old block counts for bitmap and summary inodes.
	 * These can't change since other growfs callers are locked out.
	 */
	rbmblocks = XFS_B_TO_FSB(mp, mp->m_rbmip->i_disk_size);
	rsumblocks = XFS_B_TO_FSB(mp, mp->m_rsumip->i_disk_size);
	/*
	 * Allocate space to the bitmap and summary files, as necessary.
	 */
	error = xfs_rtfile_initialize_blocks(mp->m_rbmip, rbmblocks,
			nrbmblocks, NULL);
	if (error)
		goto out_unlock;
	error = xfs_rtfile_initialize_blocks(mp->m_rsumip, rsumblocks,
			nrsumblocks, NULL);
	if (error)
		goto out_unlock;

	rsum_cache = mp->m_rsum_cache;
	if (nrbmblocks != mp->m_sb.sb_rbmblocks) {
		error = xfs_alloc_rsum_cache(mp, nrbmblocks);
		if (error)
			goto out_unlock;
	}

	/* Initialize the free space bitmap one bitmap block at a time. */
	for (bmbno = xfs_last_rt_bmblock(mp); bmbno < nrbmblocks; bmbno++) {
		error = xfs_growfs_rt_bmblock(mp, in->newblocks, in->extsize,
				bmbno);
		if (error)
			goto out_free;
	}

	if (old_rextsize != in->extsize) {
		error = xfs_growfs_rt_fixup_extsize(mp);
		if (error)
			goto out_free;
	}

	/* Update secondary superblocks now the physical grow has completed */
	error = xfs_update_secondary_sbs(mp);

out_free:
	/*
	 * If we had to allocate a new rsum_cache, we either need to free the
	 * old one (if we succeeded) or free the new one and restore the old one
	 * (if there was an error).
	 */
	if (rsum_cache != mp->m_rsum_cache) {
		if (error) {
			kvfree(mp->m_rsum_cache);
			mp->m_rsum_cache = rsum_cache;
		} else {
			kvfree(rsum_cache);
		}
	}

out_unlock:
	mutex_unlock(&mp->m_growlock);
	return error;
}

/*
 * Initialize realtime fields in the mount structure.
 */
int				/* error */
xfs_rtmount_init(
	struct xfs_mount	*mp)	/* file system mount structure */
{
	struct xfs_buf		*bp;	/* buffer for last block of subvolume */
	struct xfs_sb		*sbp;	/* filesystem superblock copy in mount */
	xfs_daddr_t		d;	/* address of last block of subvolume */
	int			error;

	sbp = &mp->m_sb;
	if (sbp->sb_rblocks == 0)
		return 0;
	if (mp->m_rtdev_targp == NULL) {
		xfs_warn(mp,
	"Filesystem has a realtime volume, use rtdev=device option");
		return -ENODEV;
	}
	mp->m_rsumlevels = sbp->sb_rextslog + 1;
	mp->m_rsumblocks = xfs_rtsummary_blockcount(mp, mp->m_rsumlevels,
			mp->m_sb.sb_rbmblocks);
	mp->m_rbmip = mp->m_rsumip = NULL;
	/*
	 * Check that the realtime section is an ok size.
	 */
	d = (xfs_daddr_t)XFS_FSB_TO_BB(mp, mp->m_sb.sb_rblocks);
	if (XFS_BB_TO_FSB(mp, d) != mp->m_sb.sb_rblocks) {
		xfs_warn(mp, "realtime mount -- %llu != %llu",
			(unsigned long long) XFS_BB_TO_FSB(mp, d),
			(unsigned long long) mp->m_sb.sb_rblocks);
		return -EFBIG;
	}
	error = xfs_buf_read_uncached(mp->m_rtdev_targp,
					d - XFS_FSB_TO_BB(mp, 1),
					XFS_FSB_TO_BB(mp, 1), 0, &bp, NULL);
	if (error) {
		xfs_warn(mp, "realtime device size check failed");
		return error;
	}
	xfs_buf_relse(bp);
	return 0;
}

static int
xfs_rtalloc_count_frextent(
	struct xfs_mount		*mp,
	struct xfs_trans		*tp,
	const struct xfs_rtalloc_rec	*rec,
	void				*priv)
{
	uint64_t			*valp = priv;

	*valp += rec->ar_extcount;
	return 0;
}

/*
 * Reinitialize the number of free realtime extents from the realtime bitmap.
 * Callers must ensure that there is no other activity in the filesystem.
 */
int
xfs_rtalloc_reinit_frextents(
	struct xfs_mount	*mp)
{
	uint64_t		val = 0;
	int			error;

	xfs_rtbitmap_lock_shared(mp, XFS_RBMLOCK_BITMAP);
	error = xfs_rtalloc_query_all(mp, NULL, xfs_rtalloc_count_frextent,
			&val);
	xfs_rtbitmap_unlock_shared(mp, XFS_RBMLOCK_BITMAP);
	if (error)
		return error;

	spin_lock(&mp->m_sb_lock);
	mp->m_sb.sb_frextents = val;
	spin_unlock(&mp->m_sb_lock);
	percpu_counter_set(&mp->m_frextents, mp->m_sb.sb_frextents);
	return 0;
}

/*
 * Read in the bmbt of an rt metadata inode so that we never have to load them
 * at runtime.  This enables the use of shared ILOCKs for rtbitmap scans.  Use
 * an empty transaction to avoid deadlocking on loops in the bmbt.
 */
static inline int
xfs_rtmount_iread_extents(
	struct xfs_inode	*ip,
	unsigned int		lock_class)
{
	struct xfs_trans	*tp;
	int			error;

	error = xfs_trans_alloc_empty(ip->i_mount, &tp);
	if (error)
		return error;

	xfs_ilock(ip, XFS_ILOCK_EXCL | lock_class);

	error = xfs_iread_extents(tp, ip, XFS_DATA_FORK);
	if (error)
		goto out_unlock;

	if (xfs_inode_has_attr_fork(ip)) {
		error = xfs_iread_extents(tp, ip, XFS_ATTR_FORK);
		if (error)
			goto out_unlock;
	}

out_unlock:
	xfs_iunlock(ip, XFS_ILOCK_EXCL | lock_class);
	xfs_trans_cancel(tp);
	return error;
}

/*
 * Get the bitmap and summary inodes and the summary cache into the mount
 * structure at mount time.
 */
int					/* error */
xfs_rtmount_inodes(
	xfs_mount_t	*mp)		/* file system mount structure */
{
	int		error;		/* error return value */
	xfs_sb_t	*sbp;

	sbp = &mp->m_sb;
	error = xfs_iget(mp, NULL, sbp->sb_rbmino, 0, 0, &mp->m_rbmip);
	if (xfs_metadata_is_sick(error))
		xfs_rt_mark_sick(mp, XFS_SICK_RT_BITMAP);
	if (error)
		return error;
	ASSERT(mp->m_rbmip != NULL);

	error = xfs_rtmount_iread_extents(mp->m_rbmip, XFS_ILOCK_RTBITMAP);
	if (error)
		goto out_rele_bitmap;

	error = xfs_iget(mp, NULL, sbp->sb_rsumino, 0, 0, &mp->m_rsumip);
	if (xfs_metadata_is_sick(error))
		xfs_rt_mark_sick(mp, XFS_SICK_RT_SUMMARY);
	if (error)
		goto out_rele_bitmap;
	ASSERT(mp->m_rsumip != NULL);

	error = xfs_rtmount_iread_extents(mp->m_rsumip, XFS_ILOCK_RTSUM);
	if (error)
		goto out_rele_summary;

	error = xfs_alloc_rsum_cache(mp, sbp->sb_rbmblocks);
	if (error)
		goto out_rele_summary;
	return 0;

out_rele_summary:
	xfs_irele(mp->m_rsumip);
out_rele_bitmap:
	xfs_irele(mp->m_rbmip);
	return error;
}

void
xfs_rtunmount_inodes(
	struct xfs_mount	*mp)
{
	kvfree(mp->m_rsum_cache);
	if (mp->m_rbmip)
		xfs_irele(mp->m_rbmip);
	if (mp->m_rsumip)
		xfs_irele(mp->m_rsumip);
}

/*
 * Pick an extent for allocation at the start of a new realtime file.
 * Use the sequence number stored in the atime field of the bitmap inode.
 * Translate this to a fraction of the rtextents, and return the product
 * of rtextents and the fraction.
 * The fraction sequence is 0, 1/2, 1/4, 3/4, 1/8, ..., 7/8, 1/16, ...
 */
static xfs_rtxnum_t
xfs_rtpick_extent(
	xfs_mount_t		*mp,		/* file system mount point */
	xfs_trans_t		*tp,		/* transaction pointer */
	xfs_rtxlen_t		len)		/* allocation length (rtextents) */
{
	xfs_rtxnum_t		b;		/* result rtext */
	int			log2;		/* log of sequence number */
	uint64_t		resid;		/* residual after log removed */
	uint64_t		seq;		/* sequence number of file creation */
	struct timespec64	ts;		/* timespec in inode */

	xfs_assert_ilocked(mp->m_rbmip, XFS_ILOCK_EXCL);

	ts = inode_get_atime(VFS_I(mp->m_rbmip));
	if (!(mp->m_rbmip->i_diflags & XFS_DIFLAG_NEWRTBM)) {
		mp->m_rbmip->i_diflags |= XFS_DIFLAG_NEWRTBM;
		seq = 0;
	} else {
		seq = ts.tv_sec;
	}
	if ((log2 = xfs_highbit64(seq)) == -1)
		b = 0;
	else {
		resid = seq - (1ULL << log2);
		b = (mp->m_sb.sb_rextents * ((resid << 1) + 1ULL)) >>
		    (log2 + 1);
		if (b >= mp->m_sb.sb_rextents)
			div64_u64_rem(b, mp->m_sb.sb_rextents, &b);
		if (b + len > mp->m_sb.sb_rextents)
			b = mp->m_sb.sb_rextents - len;
	}
	ts.tv_sec = seq + 1;
	inode_set_atime_to_ts(VFS_I(mp->m_rbmip), ts);
	xfs_trans_log_inode(tp, mp->m_rbmip, XFS_ILOG_CORE);
	return b;
}

static void
xfs_rtalloc_align_minmax(
	xfs_rtxlen_t		*raminlen,
	xfs_rtxlen_t		*ramaxlen,
	xfs_rtxlen_t		*prod)
{
	xfs_rtxlen_t		newmaxlen = *ramaxlen;
	xfs_rtxlen_t		newminlen = *raminlen;
	xfs_rtxlen_t		slack;

	slack = newmaxlen % *prod;
	if (slack)
		newmaxlen -= slack;
	slack = newminlen % *prod;
	if (slack)
		newminlen += *prod - slack;

	/*
	 * If adjusting for extent size hint alignment produces an invalid
	 * min/max len combination, go ahead without it.
	 */
	if (newmaxlen < newminlen) {
		*prod = 1;
		return;
	}
	*ramaxlen = newmaxlen;
	*raminlen = newminlen;
}

static int
xfs_rtallocate(
	struct xfs_trans	*tp,
	xfs_rtblock_t		bno_hint,
	xfs_rtxlen_t		minlen,
	xfs_rtxlen_t		maxlen,
	xfs_rtxlen_t		prod,
	bool			wasdel,
	bool			initial_user_data,
	bool			*rtlocked,
	xfs_rtblock_t		*bno,
	xfs_extlen_t		*blen)
{
	struct xfs_rtalloc_args	args = {
		.mp		= tp->t_mountp,
		.tp		= tp,
	};
	xfs_rtxnum_t		start = 0;
	xfs_rtxnum_t		rtx;
	xfs_rtxlen_t		len = 0;
	int			error = 0;

	/*
	 * Lock out modifications to both the RT bitmap and summary inodes.
	 */
	if (!*rtlocked) {
		xfs_rtbitmap_lock(args.mp);
		xfs_rtbitmap_trans_join(tp);
		*rtlocked = true;
	}

	/*
	 * For an allocation to an empty file at offset 0, pick an extent that
	 * will space things out in the rt area.
	 */
	if (bno_hint)
		start = xfs_rtb_to_rtx(args.mp, bno_hint);
	else if (initial_user_data)
		start = xfs_rtpick_extent(args.mp, tp, maxlen);

	if (start) {
		error = xfs_rtallocate_extent_near(&args, start, minlen, maxlen,
				&len, prod, &rtx);
		/*
		 * If we can't allocate near a specific rt extent, try again
		 * without locality criteria.
		 */
		if (error == -ENOSPC) {
			xfs_rtbuf_cache_relse(&args);
			error = 0;
		}
	}

	if (!error) {
		error = xfs_rtallocate_extent_size(&args, minlen, maxlen, &len,
				prod, &rtx);
	}

	if (error)
		goto out_release;

	error = xfs_rtallocate_range(&args, rtx, len);
	if (error)
		goto out_release;

	xfs_trans_mod_sb(tp, wasdel ?
			XFS_TRANS_SB_RES_FREXTENTS : XFS_TRANS_SB_FREXTENTS,
			-(long)len);
	*bno = xfs_rtx_to_rtb(args.mp, rtx);
	*blen = xfs_rtxlen_to_extlen(args.mp, len);

out_release:
	xfs_rtbuf_cache_relse(&args);
	return error;
}

static int
xfs_rtallocate_align(
	struct xfs_bmalloca	*ap,
	xfs_rtxlen_t		*ralen,
	xfs_rtxlen_t		*raminlen,
	xfs_rtxlen_t		*prod,
	bool			*noalign)
{
	struct xfs_mount	*mp = ap->ip->i_mount;
	xfs_fileoff_t		orig_offset = ap->offset;
	xfs_extlen_t		minlen = mp->m_sb.sb_rextsize;
	xfs_extlen_t            align;	/* minimum allocation alignment */
	xfs_extlen_t		mod;	/* product factor for allocators */
	int			error;

	if (*noalign) {
		align = mp->m_sb.sb_rextsize;
	} else {
		align = xfs_get_extsz_hint(ap->ip);
		if (!align)
			align = 1;
		if (align == mp->m_sb.sb_rextsize)
			*noalign = true;
	}

	error = xfs_bmap_extsize_align(mp, &ap->got, &ap->prev, align, 1,
			ap->eof, 0, ap->conv, &ap->offset, &ap->length);
	if (error)
		return error;
	ASSERT(ap->length);
	ASSERT(xfs_extlen_to_rtxmod(mp, ap->length) == 0);

	/*
	 * If we shifted the file offset downward to satisfy an extent size
	 * hint, increase minlen by that amount so that the allocator won't
	 * give us an allocation that's too short to cover at least one of the
	 * blocks that the caller asked for.
	 */
	if (ap->offset != orig_offset)
		minlen += orig_offset - ap->offset;

	/*
	 * Set ralen to be the actual requested length in rtextents.
	 *
	 * If the old value was close enough to XFS_BMBT_MAX_EXTLEN that
	 * we rounded up to it, cut it back so it's valid again.
	 * Note that if it's a really large request (bigger than
	 * XFS_BMBT_MAX_EXTLEN), we don't hear about that number, and can't
	 * adjust the starting point to match it.
	 */
	*ralen = xfs_extlen_to_rtxlen(mp, min(ap->length, XFS_MAX_BMBT_EXTLEN));
	*raminlen = max_t(xfs_rtxlen_t, 1, xfs_extlen_to_rtxlen(mp, minlen));
	ASSERT(*raminlen > 0);
	ASSERT(*raminlen <= *ralen);

	/*
	 * Only bother calculating a real prod factor if offset & length are
	 * perfectly aligned, otherwise it will just get us in trouble.
	 */
	div_u64_rem(ap->offset, align, &mod);
	if (mod || ap->length % align)
		*prod = 1;
	else
		*prod = xfs_extlen_to_rtxlen(mp, align);

	if (*prod > 1)
		xfs_rtalloc_align_minmax(raminlen, ralen, prod);
	return 0;
}

int
xfs_bmap_rtalloc(
	struct xfs_bmalloca	*ap)
{
	xfs_fileoff_t		orig_offset = ap->offset;
	xfs_rtxlen_t		prod = 0;  /* product factor for allocators */
	xfs_rtxlen_t		ralen = 0; /* realtime allocation length */
	xfs_rtblock_t		bno_hint = NULLRTBLOCK;
	xfs_extlen_t		orig_length = ap->length;
	xfs_rtxlen_t		raminlen;
	bool			rtlocked = false;
	bool			noalign = false;
	bool			initial_user_data =
		ap->datatype & XFS_ALLOC_INITIAL_USER_DATA;
	int			error;

retry:
	error = xfs_rtallocate_align(ap, &ralen, &raminlen, &prod, &noalign);
	if (error)
		return error;

	if (xfs_bmap_adjacent(ap))
		bno_hint = ap->blkno;

	error = xfs_rtallocate(ap->tp, bno_hint, raminlen, ralen, prod,
			ap->wasdel, initial_user_data, &rtlocked,
			&ap->blkno, &ap->length);
	if (error == -ENOSPC) {
		if (!noalign) {
			/*
			 * We previously enlarged the request length to try to
			 * satisfy an extent size hint.  The allocator didn't
			 * return anything, so reset the parameters to the
			 * original values and try again without alignment
			 * criteria.
			 */
			ap->offset = orig_offset;
			ap->length = orig_length;
			noalign = true;
			goto retry;
		}

		ap->blkno = NULLFSBLOCK;
		ap->length = 0;
		return 0;
	}
	if (error)
		return error;

	xfs_bmap_alloc_account(ap);
	return 0;
}
