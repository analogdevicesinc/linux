// SPDX-License-Identifier: GPL-2.0
/*
 * QNX4 file system, Linux implementation.
 *
 * Version : 0.2.1
 *
 * Using parts of the xiafs filesystem.
 *
 * History :
 *
 * 28-05-1998 by Richard Frowijn : first release.
 * 20-06-1998 by Frank Denis : Linux 2.1.99+ & dcache support.
 */

#include <linux/buffer_head.h>
#include "qnx4.h"

static int qnx4_readdir(struct file *file, struct dir_context *ctx)
{
	struct inode *inode = file_inode(file);
	unsigned int offset;
	struct buffer_head *bh;
	unsigned long blknum;
	int ix, ino;
	int size;

	QNX4DEBUG((KERN_INFO "qnx4_readdir:i_size = %ld\n", (long) inode->i_size));
	QNX4DEBUG((KERN_INFO "pos                 = %ld\n", (long) ctx->pos));

	while (ctx->pos < inode->i_size) {
		blknum = qnx4_block_map(inode, ctx->pos >> QNX4_BLOCK_SIZE_BITS);
		bh = sb_bread(inode->i_sb, blknum);
		if (bh == NULL) {
			printk(KERN_ERR "qnx4_readdir: bread failed (%ld)\n", blknum);
			return 0;
		}
		ix = (ctx->pos >> QNX4_DIR_ENTRY_SIZE_BITS) % QNX4_INODES_PER_BLOCK;
		for (; ix < QNX4_INODES_PER_BLOCK; ix++, ctx->pos += QNX4_DIR_ENTRY_SIZE) {
			union qnx4_directory_entry *de;
			const char *fname;

			offset = ix * QNX4_DIR_ENTRY_SIZE;
			de = (union qnx4_directory_entry *) (bh->b_data + offset);

			fname = get_entry_fname(de, &size);
			if (!fname)
				continue;

			if (!(de->de_status & QNX4_FILE_LINK)) {
				ino = blknum * QNX4_INODES_PER_BLOCK + ix - 1;
			} else {
				ino = ( le32_to_cpu(de->link.dl_inode_blk) - 1 ) *
					QNX4_INODES_PER_BLOCK +
					de->link.dl_inode_ndx;
			}

			QNX4DEBUG((KERN_INFO "qnx4_readdir:%.*s\n", size, fname));
			if (!dir_emit(ctx, fname, size, ino, DT_UNKNOWN)) {
				brelse(bh);
				return 0;
			}
		}
		brelse(bh);
	}
	return 0;
}

const struct file_operations qnx4_dir_operations =
{
	.llseek		= generic_file_llseek,
	.read		= generic_read_dir,
	.iterate_shared	= qnx4_readdir,
	.fsync		= generic_file_fsync,
};

const struct inode_operations qnx4_dir_inode_operations =
{
	.lookup		= qnx4_lookup,
};
