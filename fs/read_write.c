// SPDX-License-Identifier: GPL-2.0
/*
 *  linux/fs/read_write.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 */

#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/sched/xacct.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/uio.h>
#include <linux/fsnotify.h>
#include <linux/security.h>
#include <linux/export.h>
#include <linux/syscalls.h>
#include <linux/pagemap.h>
#include <linux/splice.h>
#include <linux/compat.h>
#include <linux/mount.h>
#include <linux/fs.h>
#include <linux/filelock.h>
#include "internal.h"

#include <linux/uaccess.h>
#include <asm/unistd.h>

const struct file_operations generic_ro_fops = {
	.llseek		= generic_file_llseek,
	.read_iter	= generic_file_read_iter,
	.mmap_prepare	= generic_file_readonly_mmap_prepare,
	.splice_read	= filemap_splice_read,
	.setlease	= generic_setlease,
};

EXPORT_SYMBOL(generic_ro_fops);

static inline bool unsigned_offsets(struct file *file)
{
	return file->f_op->fop_flags & FOP_UNSIGNED_OFFSET;
}

/**
 * vfs_setpos_cookie - update the file offset for lseek and reset cookie
 * @file:	file structure in question
 * @offset:	file offset to seek to
 * @maxsize:	maximum file size
 * @cookie:	cookie to reset
 *
 * Update the file offset to the value specified by @offset if the given
 * offset is valid and it is not equal to the current file offset and
 * reset the specified cookie to indicate that a seek happened.
 *
 * Return the specified offset on success and -EINVAL on invalid offset.
 */
static loff_t vfs_setpos_cookie(struct file *file, loff_t offset,
				loff_t maxsize, u64 *cookie)
{
	if (offset < 0 && !unsigned_offsets(file))
		return -EINVAL;
	if (offset > maxsize)
		return -EINVAL;

	if (offset != file->f_pos) {
		file->f_pos = offset;
		if (cookie)
			*cookie = 0;
	}
	return offset;
}

/**
 * vfs_setpos - update the file offset for lseek
 * @file:	file structure in question
 * @offset:	file offset to seek to
 * @maxsize:	maximum file size
 *
 * This is a low-level filesystem helper for updating the file offset to
 * the value specified by @offset if the given offset is valid and it is
 * not equal to the current file offset.
 *
 * Return the specified offset on success and -EINVAL on invalid offset.
 */
loff_t vfs_setpos(struct file *file, loff_t offset, loff_t maxsize)
{
	return vfs_setpos_cookie(file, offset, maxsize, NULL);
}
EXPORT_SYMBOL(vfs_setpos);

/**
 * must_set_pos - check whether f_pos has to be updated
 * @file: file to seek on
 * @offset: offset to use
 * @whence: type of seek operation
 * @eof: end of file
 *
 * Check whether f_pos needs to be updated and update @offset according
 * to @whence.
 *
 * Return: 0 if f_pos doesn't need to be updated, 1 if f_pos has to be
 * updated, and negative error code on failure.
 */
static int must_set_pos(struct file *file, loff_t *offset, int whence, loff_t eof)
{
	switch (whence) {
	case SEEK_END:
		*offset += eof;
		break;
	case SEEK_CUR:
		/*
		 * Here we special-case the lseek(fd, 0, SEEK_CUR)
		 * position-querying operation.  Avoid rewriting the "same"
		 * f_pos value back to the file because a concurrent read(),
		 * write() or lseek() might have altered it
		 */
		if (*offset == 0) {
			*offset = file->f_pos;
			return 0;
		}
		break;
	case SEEK_DATA:
		/*
		 * In the generic case the entire file is data, so as long as
		 * offset isn't at the end of the file then the offset is data.
		 */
		if ((unsigned long long)*offset >= eof)
			return -ENXIO;
		break;
	case SEEK_HOLE:
		/*
		 * There is a virtual hole at the end of the file, so as long as
		 * offset isn't i_size or larger, return i_size.
		 */
		if ((unsigned long long)*offset >= eof)
			return -ENXIO;
		*offset = eof;
		break;
	}

	return 1;
}

/**
 * generic_file_llseek_size - generic llseek implementation for regular files
 * @file:	file structure to seek on
 * @offset:	file offset to seek to
 * @whence:	type of seek
 * @maxsize:	max size of this file in file system
 * @eof:	offset used for SEEK_END position
 *
 * This is a variant of generic_file_llseek that allows passing in a custom
 * maximum file size and a custom EOF position, for e.g. hashed directories
 *
 * Synchronization:
 * SEEK_SET and SEEK_END are unsynchronized (but atomic on 64bit platforms)
 * SEEK_CUR is synchronized against other SEEK_CURs, but not read/writes.
 * read/writes behave like SEEK_SET against seeks.
 */
loff_t
generic_file_llseek_size(struct file *file, loff_t offset, int whence,
		loff_t maxsize, loff_t eof)
{
	int ret;

	ret = must_set_pos(file, &offset, whence, eof);
	if (ret < 0)
		return ret;
	if (ret == 0)
		return offset;

	if (whence == SEEK_CUR) {
		/*
		 * If the file requires locking via f_pos_lock we know
		 * that mutual exclusion for SEEK_CUR on the same file
		 * is guaranteed. If the file isn't locked, we take
		 * f_lock to protect against f_pos races with other
		 * SEEK_CURs.
		 */
		if (file_seek_cur_needs_f_lock(file)) {
			guard(spinlock)(&file->f_lock);
			return vfs_setpos(file, file->f_pos + offset, maxsize);
		}
		return vfs_setpos(file, file->f_pos + offset, maxsize);
	}

	return vfs_setpos(file, offset, maxsize);
}
EXPORT_SYMBOL(generic_file_llseek_size);

/**
 * generic_llseek_cookie - versioned llseek implementation
 * @file:	file structure to seek on
 * @offset:	file offset to seek to
 * @whence:	type of seek
 * @cookie:	cookie to update
 *
 * See generic_file_llseek for a general description and locking assumptions.
 *
 * In contrast to generic_file_llseek, this function also resets a
 * specified cookie to indicate a seek took place.
 */
loff_t generic_llseek_cookie(struct file *file, loff_t offset, int whence,
			     u64 *cookie)
{
	struct inode *inode = file->f_mapping->host;
	loff_t maxsize = inode->i_sb->s_maxbytes;
	loff_t eof = i_size_read(inode);
	int ret;

	if (WARN_ON_ONCE(!cookie))
		return -EINVAL;

	/*
	 * Require that this is only used for directories that guarantee
	 * synchronization between readdir and seek so that an update to
	 * @cookie is correctly synchronized with concurrent readdir.
	 */
	if (WARN_ON_ONCE(!(file->f_mode & FMODE_ATOMIC_POS)))
		return -EINVAL;

	ret = must_set_pos(file, &offset, whence, eof);
	if (ret < 0)
		return ret;
	if (ret == 0)
		return offset;

	/* No need to hold f_lock because we know that f_pos_lock is held. */
	if (whence == SEEK_CUR)
		return vfs_setpos_cookie(file, file->f_pos + offset, maxsize, cookie);

	return vfs_setpos_cookie(file, offset, maxsize, cookie);
}
EXPORT_SYMBOL(generic_llseek_cookie);

/**
 * generic_file_llseek - generic llseek implementation for regular files
 * @file:	file structure to seek on
 * @offset:	file offset to seek to
 * @whence:	type of seek
 *
 * This is a generic implementation of ->llseek useable for all normal local
 * filesystems.  It just updates the file offset to the value specified by
 * @offset and @whence.
 */
loff_t generic_file_llseek(struct file *file, loff_t offset, int whence)
{
	struct inode *inode = file->f_mapping->host;

	return generic_file_llseek_size(file, offset, whence,
					inode->i_sb->s_maxbytes,
					i_size_read(inode));
}
EXPORT_SYMBOL(generic_file_llseek);

/**
 * fixed_size_llseek - llseek implementation for fixed-sized devices
 * @file:	file structure to seek on
 * @offset:	file offset to seek to
 * @whence:	type of seek
 * @size:	size of the file
 *
 */
loff_t fixed_size_llseek(struct file *file, loff_t offset, int whence, loff_t size)
{
	switch (whence) {
	case SEEK_SET: case SEEK_CUR: case SEEK_END:
		return generic_file_llseek_size(file, offset, whence,
						size, size);
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL(fixed_size_llseek);

/**
 * no_seek_end_llseek - llseek implementation for files without SEEK_END
 * @file:	file structure to seek on
 * @offset:	file offset to seek to
 * @whence:	type of seek
 *
 */
loff_t no_seek_end_llseek(struct file *file, loff_t offset, int whence)
{
	switch (whence) {
	case SEEK_SET: case SEEK_CUR:
		return generic_file_llseek_size(file, offset, whence,
						OFFSET_MAX, 0);
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL(no_seek_end_llseek);

/**
 * no_seek_end_llseek_size - llseek implementation for files without SEEK_END
 * @file:	file structure to seek on
 * @offset:	file offset to seek to
 * @whence:	type of seek
 * @size:	maximal offset allowed
 *
 */
loff_t no_seek_end_llseek_size(struct file *file, loff_t offset, int whence, loff_t size)
{
	switch (whence) {
	case SEEK_SET: case SEEK_CUR:
		return generic_file_llseek_size(file, offset, whence,
						size, 0);
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL(no_seek_end_llseek_size);

/**
 * noop_llseek - No Operation Performed llseek implementation
 * @file:	file structure to seek on
 * @offset:	file offset to seek to
 * @whence:	type of seek
 *
 * This is an implementation of ->llseek useable for the rare special case when
 * userspace expects the seek to succeed but the (device) file is actually not
 * able to perform the seek. In this case you use noop_llseek() instead of
 * falling back to the default implementation of ->llseek.
 */
loff_t noop_llseek(struct file *file, loff_t offset, int whence)
{
	return file->f_pos;
}
EXPORT_SYMBOL(noop_llseek);

loff_t default_llseek(struct file *file, loff_t offset, int whence)
{
	struct inode *inode = file_inode(file);
	loff_t retval;

	retval = inode_lock_killable(inode);
	if (retval)
		return retval;
	switch (whence) {
		case SEEK_END:
			offset += i_size_read(inode);
			break;
		case SEEK_CUR:
			if (offset == 0) {
				retval = file->f_pos;
				goto out;
			}
			offset += file->f_pos;
			break;
		case SEEK_DATA:
			/*
			 * In the generic case the entire file is data, so as
			 * long as offset isn't at the end of the file then the
			 * offset is data.
			 */
			if (offset >= inode->i_size) {
				retval = -ENXIO;
				goto out;
			}
			break;
		case SEEK_HOLE:
			/*
			 * There is a virtual hole at the end of the file, so
			 * as long as offset isn't i_size or larger, return
			 * i_size.
			 */
			if (offset >= inode->i_size) {
				retval = -ENXIO;
				goto out;
			}
			offset = inode->i_size;
			break;
	}
	retval = -EINVAL;
	if (offset >= 0 || unsigned_offsets(file)) {
		if (offset != file->f_pos)
			file->f_pos = offset;
		retval = offset;
	}
out:
	inode_unlock(inode);
	return retval;
}
EXPORT_SYMBOL(default_llseek);

loff_t vfs_llseek(struct file *file, loff_t offset, int whence)
{
	if (!(file->f_mode & FMODE_LSEEK))
		return -ESPIPE;
	return file->f_op->llseek(file, offset, whence);
}
EXPORT_SYMBOL(vfs_llseek);

static off_t ksys_lseek(unsigned int fd, off_t offset, unsigned int whence)
{
	off_t retval;
	CLASS(fd_pos, f)(fd);
	if (fd_empty(f))
		return -EBADF;

	retval = -EINVAL;
	if (whence <= SEEK_MAX) {
		loff_t res = vfs_llseek(fd_file(f), offset, whence);
		retval = res;
		if (res != (loff_t)retval)
			retval = -EOVERFLOW;	/* LFS: should only happen on 32 bit platforms */
	}
	return retval;
}

SYSCALL_DEFINE3(lseek, unsigned int, fd, off_t, offset, unsigned int, whence)
{
	return ksys_lseek(fd, offset, whence);
}

#ifdef CONFIG_COMPAT
COMPAT_SYSCALL_DEFINE3(lseek, unsigned int, fd, compat_off_t, offset, unsigned int, whence)
{
	return ksys_lseek(fd, offset, whence);
}
#endif

#if !defined(CONFIG_64BIT) || defined(CONFIG_COMPAT) || \
	defined(__ARCH_WANT_SYS_LLSEEK)
SYSCALL_DEFINE5(llseek, unsigned int, fd, unsigned long, offset_high,
		unsigned long, offset_low, loff_t __user *, result,
		unsigned int, whence)
{
	int retval;
	CLASS(fd_pos, f)(fd);
	loff_t offset;

	if (fd_empty(f))
		return -EBADF;

	if (whence > SEEK_MAX)
		return -EINVAL;

	offset = vfs_llseek(fd_file(f), ((loff_t) offset_high << 32) | offset_low,
			whence);

	retval = (int)offset;
	if (offset >= 0) {
		retval = -EFAULT;
		if (!copy_to_user(result, &offset, sizeof(offset)))
			retval = 0;
	}
	return retval;
}
#endif

int rw_verify_area(int read_write, struct file *file, const loff_t *ppos, size_t count)
{
	int mask = read_write == READ ? MAY_READ : MAY_WRITE;
	int ret;

	if (unlikely((ssize_t) count < 0))
		return -EINVAL;

	if (ppos) {
		loff_t pos = *ppos;

		if (unlikely(pos < 0)) {
			if (!unsigned_offsets(file))
				return -EINVAL;
			if (count >= -pos) /* both values are in 0..LLONG_MAX */
				return -EOVERFLOW;
		} else if (unlikely((loff_t) (pos + count) < 0)) {
			if (!unsigned_offsets(file))
				return -EINVAL;
		}
	}

	ret = security_file_permission(file, mask);
	if (ret)
		return ret;

	return fsnotify_file_area_perm(file, mask, ppos, count);
}
EXPORT_SYMBOL(rw_verify_area);

static ssize_t new_sync_read(struct file *filp, char __user *buf, size_t len, loff_t *ppos)
{
	struct kiocb kiocb;
	struct iov_iter iter;
	ssize_t ret;

	init_sync_kiocb(&kiocb, filp);
	kiocb.ki_pos = (ppos ? *ppos : 0);
	iov_iter_ubuf(&iter, ITER_DEST, buf, len);

	ret = filp->f_op->read_iter(&kiocb, &iter);
	BUG_ON(ret == -EIOCBQUEUED);
	if (ppos)
		*ppos = kiocb.ki_pos;
	return ret;
}

static int warn_unsupported(struct file *file, const char *op)
{
	pr_warn_ratelimited(
		"kernel %s not supported for file %pD4 (pid: %d comm: %.20s)\n",
		op, file, current->pid, current->comm);
	return -EINVAL;
}

ssize_t __kernel_read(struct file *file, void *buf, size_t count, loff_t *pos)
{
	struct kvec iov = {
		.iov_base	= buf,
		.iov_len	= min_t(size_t, count, MAX_RW_COUNT),
	};
	struct kiocb kiocb;
	struct iov_iter iter;
	ssize_t ret;

	if (WARN_ON_ONCE(!(file->f_mode & FMODE_READ)))
		return -EINVAL;
	if (!(file->f_mode & FMODE_CAN_READ))
		return -EINVAL;
	/*
	 * Also fail if ->read_iter and ->read are both wired up as that
	 * implies very convoluted semantics.
	 */
	if (unlikely(!file->f_op->read_iter || file->f_op->read))
		return warn_unsupported(file, "read");

	init_sync_kiocb(&kiocb, file);
	kiocb.ki_pos = pos ? *pos : 0;
	iov_iter_kvec(&iter, ITER_DEST, &iov, 1, iov.iov_len);
	ret = file->f_op->read_iter(&kiocb, &iter);
	if (ret > 0) {
		if (pos)
			*pos = kiocb.ki_pos;
		fsnotify_access(file);
		add_rchar(current, ret);
	}
	inc_syscr(current);
	return ret;
}

ssize_t kernel_read(struct file *file, void *buf, size_t count, loff_t *pos)
{
	ssize_t ret;

	ret = rw_verify_area(READ, file, pos, count);
	if (ret)
		return ret;
	return __kernel_read(file, buf, count, pos);
}
EXPORT_SYMBOL(kernel_read);

ssize_t vfs_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
	ssize_t ret;

	if (!(file->f_mode & FMODE_READ))
		return -EBADF;
	if (!(file->f_mode & FMODE_CAN_READ))
		return -EINVAL;
	if (unlikely(!access_ok(buf, count)))
		return -EFAULT;

	ret = rw_verify_area(READ, file, pos, count);
	if (ret)
		return ret;
	if (count > MAX_RW_COUNT)
		count =  MAX_RW_COUNT;

	if (file->f_op->read)
		ret = file->f_op->read(file, buf, count, pos);
	else if (file->f_op->read_iter)
		ret = new_sync_read(file, buf, count, pos);
	else
		ret = -EINVAL;
	if (ret > 0) {
		fsnotify_access(file);
		add_rchar(current, ret);
	}
	inc_syscr(current);
	return ret;
}

static ssize_t new_sync_write(struct file *filp, const char __user *buf, size_t len, loff_t *ppos)
{
	struct kiocb kiocb;
	struct iov_iter iter;
	ssize_t ret;

	init_sync_kiocb(&kiocb, filp);
	kiocb.ki_pos = (ppos ? *ppos : 0);
	iov_iter_ubuf(&iter, ITER_SOURCE, (void __user *)buf, len);

	ret = filp->f_op->write_iter(&kiocb, &iter);
	BUG_ON(ret == -EIOCBQUEUED);
	if (ret > 0 && ppos)
		*ppos = kiocb.ki_pos;
	return ret;
}

/* caller is responsible for file_start_write/file_end_write */
ssize_t __kernel_write_iter(struct file *file, struct iov_iter *from, loff_t *pos)
{
	struct kiocb kiocb;
	ssize_t ret;

	if (WARN_ON_ONCE(!(file->f_mode & FMODE_WRITE)))
		return -EBADF;
	if (!(file->f_mode & FMODE_CAN_WRITE))
		return -EINVAL;
	/*
	 * Also fail if ->write_iter and ->write are both wired up as that
	 * implies very convoluted semantics.
	 */
	if (unlikely(!file->f_op->write_iter || file->f_op->write))
		return warn_unsupported(file, "write");

	init_sync_kiocb(&kiocb, file);
	kiocb.ki_pos = pos ? *pos : 0;
	ret = file->f_op->write_iter(&kiocb, from);
	if (ret > 0) {
		if (pos)
			*pos = kiocb.ki_pos;
		fsnotify_modify(file);
		add_wchar(current, ret);
	}
	inc_syscw(current);
	return ret;
}

/* caller is responsible for file_start_write/file_end_write */
ssize_t __kernel_write(struct file *file, const void *buf, size_t count, loff_t *pos)
{
	struct kvec iov = {
		.iov_base	= (void *)buf,
		.iov_len	= min_t(size_t, count, MAX_RW_COUNT),
	};
	struct iov_iter iter;
	iov_iter_kvec(&iter, ITER_SOURCE, &iov, 1, iov.iov_len);
	return __kernel_write_iter(file, &iter, pos);
}
/*
 * autofs is one of the few internal kernel users that actually
 * wants this _and_ can be built as a module. So we need to export
 * this symbol for autofs, even though it really isn't appropriate
 * for any other kernel modules.
 */
EXPORT_SYMBOL_FOR_MODULES(__kernel_write, "autofs4");

ssize_t kernel_write(struct file *file, const void *buf, size_t count,
			    loff_t *pos)
{
	ssize_t ret;

	ret = rw_verify_area(WRITE, file, pos, count);
	if (ret)
		return ret;

	file_start_write(file);
	ret =  __kernel_write(file, buf, count, pos);
	file_end_write(file);
	return ret;
}
EXPORT_SYMBOL(kernel_write);

ssize_t vfs_write(struct file *file, const char __user *buf, size_t count, loff_t *pos)
{
	ssize_t ret;

	if (!(file->f_mode & FMODE_WRITE))
		return -EBADF;
	if (!(file->f_mode & FMODE_CAN_WRITE))
		return -EINVAL;
	if (unlikely(!access_ok(buf, count)))
		return -EFAULT;

	ret = rw_verify_area(WRITE, file, pos, count);
	if (ret)
		return ret;
	if (count > MAX_RW_COUNT)
		count =  MAX_RW_COUNT;
	file_start_write(file);
	if (file->f_op->write)
		ret = file->f_op->write(file, buf, count, pos);
	else if (file->f_op->write_iter)
		ret = new_sync_write(file, buf, count, pos);
	else
		ret = -EINVAL;
	if (ret > 0) {
		fsnotify_modify(file);
		add_wchar(current, ret);
	}
	inc_syscw(current);
	file_end_write(file);
	return ret;
}

/* file_ppos returns &file->f_pos or NULL if file is stream */
static inline loff_t *file_ppos(struct file *file)
{
	return file->f_mode & FMODE_STREAM ? NULL : &file->f_pos;
}

ssize_t ksys_read(unsigned int fd, char __user *buf, size_t count)
{
	CLASS(fd_pos, f)(fd);
	ssize_t ret = -EBADF;

	if (!fd_empty(f)) {
		loff_t pos, *ppos = file_ppos(fd_file(f));
		if (ppos) {
			pos = *ppos;
			ppos = &pos;
		}
		ret = vfs_read(fd_file(f), buf, count, ppos);
		if (ret >= 0 && ppos)
			fd_file(f)->f_pos = pos;
	}
	return ret;
}

/**
 * sys_read - Read data from a file descriptor
 * @fd: File descriptor to read from
 * @buf: User-space buffer to read data into
 * @count: Maximum number of bytes to read
 *
 * long-desc: Reads at most count bytes from fd into the user buffer buf. For
 *   seekable files (regular files, block devices), the read begins at the
 *   current file offset, and the file offset is advanced by the number of
 *   bytes read. For stream files (FMODE_STREAM, such as pipes, FIFOs, and
 *   sockets), the file offset is not used. Other files, including many
 *   character devices, keep a file offset although a driver may ignore it.
 *
 *   If count is zero, the usual checks still run and the file's read method is
 *   still called. Common implementations (regular files, pipes, sockets)
 *   return zero without transferring data.
 *
 *   On success the number of bytes read is returned; zero means end of file
 *   for regular files. A read can return fewer bytes than requested, for
 *   example near end of file, on a pipe, socket or terminal with less data
 *   pending, or when a signal arrives after some data was copied.
 *
 *   On Linux, read() transfers at most MAX_RW_COUNT (INT_MAX & PAGE_MASK,
 *   0x7ffff000 with 4KB pages, just under 2GB) bytes per call, regardless of
 *   whether the filesystem would allow more. This is to avoid issues with
 *   signed arithmetic overflow on 32-bit systems.
 *
 *   POSIX allows reads that are interrupted after reading some data to either
 *   return -1 (with errno set to EINTR) or return the number of bytes already
 *   read. Linux follows the latter behavior: if data has been read before a
 *   signal arrives, the call returns the bytes read rather than failing.
 *
 * contexts: process, sleepable
 *
 * param: fd
 *   type: fd, input
 *   constraint-type: range(0, INT_MAX)
 *   cdesc: Must be a valid, open file descriptor with read permission.
 *     The file must have been opened with O_RDONLY or O_RDWR. Special values
 *     like AT_FDCWD are not valid. File descriptors for directories return
 *     EISDIR. Standard file descriptors 0 (stdin), 1 (stdout), 2 (stderr) are
 *     valid if open and readable.
 *
 * param: buf
 *   type: user_ptr, output
 *   constraint-type: buffer(2)
 *   cdesc: Must point to a writable user-space region of at least count bytes.
 *     The range is checked via access_ok() and a range outside user space
 *     fails with EFAULT. NULL is not rejected by that check. With a count of 0
 *     the call returns 0 in common cases. With a count above 0 the copy fails
 *     with EFAULT, or the call returns a short count if some data was copied
 *     first. The buffer may be partially written if an error occurs mid-read.
 *     O_DIRECT reads may require block-size alignment (see STATX_DIOALIGN).
 *
 * param: count
 *   type: uint, input
 *   cdesc: Maximum number of bytes to read. The range buf to buf + count must
 *     lie within user space or access_ok() fails with EFAULT, which includes
 *     counts that do not fit in ssize_t on 64-bit kernels. A count of 0 passes
 *     the same checks and the file's read method is still called, typically
 *     returning 0 without transferring data. After access_ok() and
 *     rw_verify_area() succeed, counts above MAX_RW_COUNT (INT_MAX &
 *     PAGE_MASK, 0x7ffff000 with 4KB pages) are clamped to MAX_RW_COUNT.
 *
 * return:
 *   type: int
 *   check-type: range
 *   success: >= 0
 *   desc: On success, returns the number of bytes read (non-negative). Zero
 *     indicates end-of-file (EOF) for regular files, or no data available
 *     from a device that does not block. The return value may be less than
 *     count if fewer bytes were available (short read). Partial reads are
 *     not errors. On error, returns a negative error code.
 *
 * error: EBADF, Bad file descriptor
 *   desc: fd is not a valid file descriptor, or fd was not opened for reading.
 *     This includes file descriptors opened with O_WRONLY, O_PATH, or file
 *     descriptors that have been closed. Also returned if the file structure
 *     does not have FMODE_READ set.
 *
 * error: EFAULT, Bad address
 *   desc: buf points outside the accessible address space. The buffer address
 *     failed access_ok() validation. Can also occur if a fault happens during
 *     copy_to_user() when transferring data to user space after the read
 *     completes in kernel space.
 *
 * error: EINVAL, Invalid argument
 *   desc: Returned in several cases: (1) The file has no read or read_iter
 *     method (FMODE_CAN_READ is not set). (2) The file was opened with
 *     O_DIRECT and the buffer alignment, offset, or count does not meet the
 *     filesystem's alignment requirements. (3) For timerfd file descriptors,
 *     the buffer is smaller than 8 bytes. (4) rw_verify_area() rejects a
 *     negative file position, or a position plus count that overflows, on a
 *     file without FOP_UNSIGNED_OFFSET. (5) The count, cast to ssize_t, is
 *     negative (32-bit kernels only).
 *
 * error: EISDIR, Is a directory
 *   desc: fd refers to a directory. Directories cannot be read using read();
 *     use getdents64() instead. This error is returned by the generic_read_dir()
 *     handler installed for directory file operations.
 *
 * error: EAGAIN, Resource temporarily unavailable
 *   desc: fd refers to a file (pipe, socket, device) that is marked non-blocking
 *     (O_NONBLOCK) and the read would block. Equivalent to EWOULDBLOCK. The
 *     application should retry the read later or use select/poll/epoll.
 *
 * error: EINTR, Interrupted system call
 *   desc: The call was interrupted by a signal before any data was read. This
 *     only occurs if no data has been transferred; if some data was read before
 *     the signal, the call returns the number of bytes read. The caller should
 *     typically restart the read.
 *
 * error: EIO, Input/output error
 *   desc: A low-level I/O error occurred. For regular files, this typically
 *     indicates a hardware error on the storage device, a filesystem error,
 *     or a network filesystem timeout. For terminals, it is returned when a
 *     background process group reads its controlling terminal and SIGTTIN is
 *     ignored or blocked, or the process group is orphaned.
 *
 * error: EOVERFLOW, Value too large for defined data type
 *   desc: Returned by rw_verify_area() only for files with FOP_UNSIGNED_OFFSET
 *     (for example /proc/pid/mem) when the file position is negative, meaning
 *     above LLONG_MAX, and count is at least -pos so that the read would wrap
 *     past the end of the 64-bit offset space. Files without
 *     FOP_UNSIGNED_OFFSET get EINVAL for a negative position or a position
 *     plus count that overflows.
 *
 * error: ENOBUFS, No buffer space available
 *   desc: Returned when reading from pipe-based watch queues (CONFIG_WATCH_QUEUE)
 *     when the buffer is too small to hold a complete notification, or when
 *     reading packets from pipes with PIPE_BUF_FLAG_WHOLE set.
 *
 * error: ERESTARTSYS, Restart system call (internal)
 *   desc: Internal error code indicating the syscall should be restarted. This
 *     is typically translated to EINTR if SA_RESTART is not set on the signal
 *     handler, or the syscall is transparently restarted if SA_RESTART is set.
 *     User space should not see this error code directly.
 *
 * error: EACCES, Permission denied
 *   desc: The security subsystem (LSM such as SELinux or AppArmor) denied
 *     the read operation via security_file_permission(). This can occur even
 *     if the file was successfully opened, as LSM policies may enforce checks
 *     on each operation.
 *
 * error: EPERM, Operation not permitted
 *   desc: Returned by fanotify permission events (CONFIG_FANOTIFY_ACCESS_PERMISSIONS)
 *     when a user-space fanotify listener denies the read operation via
 *     fsnotify_file_area_perm().
 *
 * error: ENODATA, No data available
 *   desc: Returned when a filesystem built on netfs (for example AFS or Ceph)
 *     ends a read subrequest short without making progress.
 *
 * error: EOPNOTSUPP, Operation not supported
 *   desc: Returned by the read method of the file, not by the generic VFS
 *     path. For example, a socket whose protocol has no receive method
 *     (sock_no_recvmsg) fails with EOPNOTSUPP. A file with no read method at
 *     all fails with EINVAL instead.
 *
 * lock: file->f_pos_lock
 *   type: mutex
 *   acquired: true
 *   released: true
 *   desc: For regular files that require atomic position updates (FMODE_ATOMIC_POS),
 *     the f_pos_lock mutex is acquired by fdget_pos() at syscall entry and released
 *     by fdput_pos() at syscall exit. This serializes concurrent reads that share
 *     the same file description. Not acquired for files opened with FMODE_STREAM
 *     (pipes, sockets) or when the file is not shared.
 *
 * lock: Filesystem-specific locks
 *   type: custom
 *   acquired: true
 *   released: true
 *   desc: The filesystem's read_iter or read method may acquire additional locks.
 *     For regular files, this typically includes the inode's i_rwsem for certain
 *     operations. For pipes, the pipe->mutex is acquired. For sockets, socket
 *     lock is acquired. These are internal to the file operation and released
 *     before return.
 *
 * lock: RCU read-side
 *   type: rcu
 *   acquired: true
 *   released: true
 *   desc: Taken only when the files_struct is shared with other threads, in
 *     which case the fd lookup in fdget() uses RCU and takes a file reference.
 *     A private table needs no RCU. The RCU read lock is acquired and released
 *     internally by the fd lookup path, not held across the entire syscall.
 *     fdput() releases the file reference count, not the RCU lock.
 *
 * signal: Any signal
 *   direction: receive
 *   action: return
 *   condition: When blocked waiting for data on interruptible operations
 *   desc: The syscall may be interrupted by signals while waiting for data to
 *     become available (pipes, sockets, terminals). If interrupted before any
 *     data is read, returns -EINTR or -ERESTARTSYS. If data has already been
 *     read, returns the number of bytes read.
 *   errno: -EINTR
 *   timing: during
 *   restartable: yes
 *
 * side-effect: file_position
 *   target: file->f_pos
 *   condition: For seekable files when read succeeds (returns > 0)
 *   desc: The file offset (f_pos) is advanced by the number of bytes read.
 *     For stream files (FMODE_STREAM such as pipes and sockets), the offset
 *     is not used or modified. The offset update is protected by f_pos_lock
 *     when the file is shared between threads/processes.
 *   reversible: no
 *
 * side-effect: modify_state
 *   target: inode access time (atime)
 *   condition: When the read_iter implementation calls file_accessed() and
 *     O_NOATIME is not set
 *   desc: The filesystem's read_iter implementation updates atime via
 *     file_accessed() and touch_atime(). Buffered reads (filemap_read) do this
 *     after the data loop, including at EOF and after errors, but not for a
 *     count of 0. O_DIRECT reads (generic_file_read_iter) do it before the
 *     I/O. The update may be suppressed by mount options (noatime, relatime),
 *     the O_NOATIME flag, or if the filesystem does not support atime.
 *     Relatime updates atime only if it is older than mtime or ctime, or more
 *     than a day old.
 *   reversible: no
 *
 * side-effect: modify_state
 *   target: task I/O accounting
 *   condition: When CONFIG_TASK_XACCT is enabled and the read method is invoked
 *   desc: Updates the current task's I/O accounting statistics. The rchar field
 *     (read characters) is incremented by bytes read via add_rchar() only on
 *     successful reads (ret > 0). The syscr field (syscall read count) is
 *     incremented via inc_syscr() once the read method has been invoked, even
 *     if it fails, but not when the FMODE_READ, FMODE_CAN_READ, access_ok() or
 *     rw_verify_area() checks fail. These statistics are visible in
 *     /proc/[pid]/io.
 *   reversible: no
 *
 * side-effect: modify_state
 *   target: fsnotify events
 *   condition: When read returns > 0
 *   desc: Generates an FS_ACCESS fsnotify event via fsnotify_access() allowing
 *     inotify, fanotify, and dnotify watchers to be notified of the read. This
 *     occurs after data transfer completes successfully.
 *   reversible: no
 *
 * constraint: MAX_RW_COUNT
 *   desc: After the access_ok() and rw_verify_area() checks pass, the count
 *     parameter is silently clamped to MAX_RW_COUNT (INT_MAX & PAGE_MASK, just
 *     under 2GB) to prevent integer overflow in internal calculations. This is
 *     transparent to the caller. The syscall succeeds but reads at most
 *     MAX_RW_COUNT bytes.
 *   expr: actual_count = min(count, MAX_RW_COUNT)
 *
 * constraint: File must be open for reading
 *   desc: The file descriptor must have been opened with O_RDONLY or O_RDWR.
 *     Files opened with O_WRONLY or O_PATH lack FMODE_READ and return EBADF.
 *     Files that lack FMODE_CAN_READ because they have no read or read_iter
 *     method return EINVAL.
 *   expr: (file->f_mode & FMODE_READ) && (file->f_mode & FMODE_CAN_READ)
 *
 * examples: n = read(fd, buf, sizeof(buf));  // Basic read
 *   n = read(STDIN_FILENO, buf, 1024);  // Read from stdin
 *   while ((n = read(fd, buf, 4096)) > 0) { process(buf, n); }  // Read loop
 *   if (read(fd, buf, count) == 0) { handle_eof(); }  // Check for EOF
 *
 * notes: The behavior of read() varies significantly depending on the type of
 *   file descriptor:
 *
 *   - Regular files: Reads from current position, advances position, returns 0
 *     at EOF. Short reads are rare but possible near EOF or on signal.
 *
 *   - Pipes and FIFOs: Blocking by default. Returns available data (up to count)
 *     or blocks until data is available. Returns 0 when all writers have closed.
 *     O_NONBLOCK returns EAGAIN when empty instead of blocking.
 *
 *   - Sockets: Similar to pipes. Specific behavior depends on socket type and
 *     protocol. MSG_* flags can be specified via recv() for more control.
 *
 *   - Terminals: Line-buffered in canonical mode; read returns when newline is
 *     entered or buffer is full. Raw mode returns immediately when data available.
 *     Special handling for signals (SIGINT on Ctrl+C, etc.).
 *
 *   - Device special files: Behavior is device-specific. Some devices support
 *     seeking, others do not. Read size may be constrained by device.
 *
 *   Race condition: Concurrent reads from the same file description (not just
 *   file descriptor) can race on the file position. Linux 3.14+ provides atomic
 *   position updates for regular files via f_pos_lock, but applications should
 *   use pread() for concurrent positioned reads.
 *
 *   O_DIRECT reads bypass the page cache and typically require aligned buffers
 *   and positions. Alignment requirements are filesystem-specific; use statx()
 *   with STATX_DIOALIGN (Linux 6.1+) to query. Unaligned O_DIRECT reads fail
 *   with EINVAL on most filesystems.
 *
 *   For splice(2)-like zero-copy reads, consider using splice(), sendfile(),
 *   or copy_file_range() instead of read() + write().
 */
SYSCALL_DEFINE3(read, unsigned int, fd, char __user *, buf, size_t, count)
{
	return ksys_read(fd, buf, count);
}

ssize_t ksys_write(unsigned int fd, const char __user *buf, size_t count)
{
	CLASS(fd_pos, f)(fd);
	ssize_t ret = -EBADF;

	if (!fd_empty(f)) {
		loff_t pos, *ppos = file_ppos(fd_file(f));
		if (ppos) {
			pos = *ppos;
			ppos = &pos;
		}
		ret = vfs_write(fd_file(f), buf, count, ppos);
		if (ret >= 0 && ppos)
			fd_file(f)->f_pos = pos;
	}

	return ret;
}

/**
 * sys_write - Write data to a file descriptor
 * @fd: File descriptor to write to
 * @buf: User-space buffer containing data to write
 * @count: Maximum number of bytes to write
 *
 * long-desc: Writes at most count bytes from the user buffer buf to fd. For
 *   seekable files (regular files, block devices), the write begins at the
 *   current file offset, and the file offset is advanced by the number of
 *   bytes written. If the file was opened with O_APPEND, the file offset is
 *   first set to the end of the file before writing. For stream files
 *   (FMODE_STREAM, such as pipes, FIFOs, and sockets), the file offset is not
 *   used and writing occurs at the position defined by the device. Other
 *   files, including many character devices, keep a file offset although a
 *   driver may ignore it.
 *
 *   Fewer than count bytes may be written, for example when the filesystem
 *   runs out of space, the write reaches RLIMIT_FSIZE, or a signal arrives
 *   after some data was written. The caller writes the rest with another
 *   write() call.
 *
 *   On Linux, write() transfers at most MAX_RW_COUNT (INT_MAX & PAGE_MASK,
 *   0x7ffff000 with 4KB pages, just under 2GB) bytes per call, regardless of
 *   whether the file or filesystem would allow more. This prevents signed
 *   arithmetic overflow.
 *
 *   For regular files, a successful write() does not guarantee that data has been
 *   committed to disk. Use fsync(2) or fdatasync(2) if durability is required.
 *   For O_SYNC or O_DSYNC files, the kernel automatically syncs data on write.
 *
 *   POSIX permits writes that are interrupted after partial writes to either
 *   return -1 with errno=EINTR, or to return the count of bytes already written.
 *   Linux implements the latter behavior: if some data has been written before
 *   a signal arrives, write() returns the number of bytes written rather than
 *   failing with EINTR.
 *
 * contexts: process, sleepable
 *
 * param: fd
 *   type: fd, input
 *   constraint-type: range(0, INT_MAX)
 *   cdesc: Must be a valid, open file descriptor with write permission.
 *     The file must have been opened with O_WRONLY or O_RDWR. File descriptors
 *     opened with O_RDONLY, O_PATH, or that have been closed return EBADF.
 *     Standard file descriptors 0 (stdin), 1 (stdout), 2 (stderr) are valid if
 *     open and writable. AT_FDCWD and other special values are not valid.
 *
 * param: buf
 *   type: user_ptr, input
 *   constraint-type: buffer(2)
 *   cdesc: Must point to a readable user-space region of at least count bytes.
 *     The range is checked via access_ok() and a range outside user space
 *     fails with EFAULT. NULL is not rejected by that check. With a count of 0
 *     the call returns 0 in common cases. With a count above 0 the copy from
 *     user space fails with EFAULT, or the call returns a short count if part
 *     of the buffer was copied first. O_DIRECT writes may require block-size
 *     alignment (see STATX_DIOALIGN).
 *
 * param: count
 *   type: uint, input
 *   cdesc: Maximum number of bytes to write. The range buf to buf + count must
 *     lie within user space or access_ok() fails with EFAULT, which includes
 *     counts that do not fit in ssize_t on 64-bit kernels. On 32-bit kernels
 *     such a count reaches rw_verify_area() and fails with EINVAL. A count of
 *     0 is passed to the file's write method, which typically returns 0 but
 *     may trigger driver-specific side effects. After the checks pass, counts
 *     above MAX_RW_COUNT (INT_MAX & PAGE_MASK, 0x7ffff000 with 4KB pages) are
 *     clamped.
 *
 * return:
 *   type: int
 *   check-type: range
 *   success: >= 0
 *   desc: On success, returns the number of bytes written (non-negative). Zero
 *     indicates that nothing was written (count was 0, or a device-specific
 *     write method accepted no data). Non-blocking writes that cannot proceed
 *     return -EAGAIN instead. The return value may be less than count due to
 *     resource limits, signal interruption, or device constraints (short
 *     write). On error, returns a negative error code.
 *
 * error: EBADF, Bad file descriptor
 *   desc: fd is not a valid file descriptor, or fd was not opened for writing.
 *     This includes file descriptors opened with O_RDONLY, O_PATH, or file
 *     descriptors that have been closed. Also returned if the file structure
 *     does not have FMODE_WRITE set.
 *
 * error: EFAULT, Bad address
 *   desc: buf points outside the accessible address space. The buffer address
 *     failed access_ok() validation. Can also occur if a fault happens during
 *     copy_from_user() when reading data from user space.
 *
 * error: EINVAL, Invalid argument
 *   desc: Returned in several cases: (1) The file has no write or write_iter
 *     method (FMODE_CAN_WRITE is not set). (2) The file was opened with
 *     O_DIRECT and the buffer alignment, offset, or count does not meet the
 *     filesystem's alignment requirements. (3) rw_verify_area() rejects a
 *     negative file position, or a position plus count that overflows, on a
 *     file without FOP_UNSIGNED_OFFSET. (4) The count, cast to ssize_t, is
 *     negative (32-bit kernels only, as 64-bit kernels fail access_ok() first
 *     with EFAULT).
 *
 * error: EAGAIN, Resource temporarily unavailable
 *   desc: fd refers to a file (pipe, socket, device) that is marked non-blocking
 *     (O_NONBLOCK) and the write would block because the buffer is full.
 *     Equivalent to EWOULDBLOCK. The application should retry later or use
 *     select/poll/epoll to wait for writability.
 *
 * error: EINTR, Interrupted system call
 *   desc: The call was interrupted by a signal before any data was written. This
 *     only occurs if no data has been transferred; if some data was written
 *     before the signal, the call returns the number of bytes written. The
 *     caller should typically restart the write.
 *
 * error: EPIPE, Broken pipe
 *   desc: fd refers to a pipe or socket whose reading end has been closed.
 *     When this condition occurs, the calling process also receives a SIGPIPE
 *     signal. If the signal is caught or ignored, EPIPE is still returned.
 *     For sockets, MSG_NOSIGNAL (via send()) suppresses the signal. For
 *     pwritev2(), the RWF_NOSIGNAL flag suppresses it.
 *
 * error: EFBIG, File too large
 *   desc: The write starts at or beyond a file size limit. The generic write
 *     path returns EFBIG when the starting position is at or beyond
 *     RLIMIT_FSIZE, in which case the process also receives SIGXFSZ, or at or
 *     beyond the maximum file size of the filesystem. Without O_LARGEFILE
 *     (32-bit systems only) that maximum is 2GB minus one byte. A write that
 *     starts below a limit but would cross it is shortened to end at the
 *     limit.
 *
 * error: ENOSPC, No space left on device
 *   desc: The device containing the file has no room for the data. This can
 *     occur mid-write resulting in a short write followed by ENOSPC on retry.
 *
 * error: EDQUOT, Disk quota exceeded
 *   desc: The user's quota of disk blocks on the filesystem has been exhausted.
 *     Like ENOSPC, this can result in a short write.
 *
 * error: EIO, Input/output error
 *   desc: A low-level I/O error occurred while modifying the inode or writing
 *     data. This typically indicates hardware failure, filesystem corruption,
 *     or network filesystem timeout. Some data may have been written.
 *
 * error: EPERM, Operation not permitted
 *   desc: The operation was prevented (1) by a file seal (F_SEAL_WRITE or
 *     F_SEAL_FUTURE_WRITE on memfd/shmem, or F_SEAL_GROW for a write that
 *     extends the file), (2) by a filesystem-specific check, for example ext4
 *     refuses writes to an immutable inode, (3) because the file is a
 *     read-only block device, (4) by an LSM hook denying the operation, or (5)
 *     by a fanotify pre-content (FAN_PRE_ACCESS) listener denying the write.
 *
 * error: EOVERFLOW, Value too large for defined data type
 *   desc: Returned by rw_verify_area() only for files with FOP_UNSIGNED_OFFSET
 *     (for example /proc/pid/mem) when the file position is negative, meaning
 *     above LLONG_MAX, and count is at least -pos so that the write would wrap
 *     past the end of the 64-bit offset space. Files without
 *     FOP_UNSIGNED_OFFSET get EINVAL for a negative position or a position
 *     plus count that overflows. Exceeding filesystem file size limits is
 *     EFBIG, not EOVERFLOW.
 *
 * error: EDESTADDRREQ, Destination address required
 *   desc: fd is a datagram socket for which no peer address has been set using
 *     connect(2). Use sendto(2) to specify the destination address.
 *
 * error: ETXTBSY, Text file busy
 *   desc: The file is being used as a swap file (IS_SWAPFILE).
 *
 * error: EXDEV, Cross-device link
 *   desc: When writing to a pipe that has been configured as a watch queue
 *     (CONFIG_WATCH_QUEUE), direct write() calls are not supported.
 *
 * error: ENOMEM, Out of memory
 *   desc: Insufficient kernel memory was available for the write operation.
 *     For pipes, this occurs when allocating pages for the pipe buffer.
 *
 * error: ERESTARTSYS, Restart system call (internal)
 *   desc: Internal error code indicating the syscall should be restarted. This
 *     is converted to EINTR if SA_RESTART is not set on the signal handler, or
 *     the syscall is transparently restarted if SA_RESTART is set. User space
 *     should not see this error code directly.
 *
 * error: EACCES, Permission denied
 *   desc: The security subsystem (LSM such as SELinux or AppArmor) denied the
 *     write operation via security_file_permission(). This can occur even if
 *     the file was successfully opened.
 *
 * lock: file->f_pos_lock
 *   type: mutex
 *   acquired: true
 *   released: true
 *   desc: For regular files that require atomic position updates (FMODE_ATOMIC_POS),
 *     the f_pos_lock mutex is acquired by fdget_pos() at syscall entry and released
 *     by fdput_pos() at syscall exit. This serializes concurrent writes sharing
 *     the same file description. Not acquired for stream files (FMODE_STREAM like
 *     pipes and sockets) or when the file is not shared.
 *
 * lock: sb->s_writers (freeze protection)
 *   type: custom
 *   acquired: true
 *   released: true
 *   desc: For regular files, file_start_write() acquires freeze protection on
 *     the superblock via sb_start_write() before the write, and file_end_write()
 *     releases it after. This prevents writes during filesystem freeze. Not
 *     acquired for non-regular files (pipes, sockets, devices).
 *
 * lock: inode->i_rwsem
 *   type: semaphore
 *   acquired: true
 *   released: true
 *   desc: For regular files using generic_file_write_iter(), the inode's i_rwsem
 *     is acquired in write mode before modifying file data. This is internal to
 *     the filesystem and released before return. Not all filesystems use this
 *     pattern.
 *
 * lock: pipe->mutex
 *   type: mutex
 *   acquired: true
 *   released: true
 *   desc: For pipes and FIFOs, the pipe's mutex is held while modifying pipe
 *     buffers. Released temporarily while waiting for space, then reacquired.
 *
 * lock: RCU read-side
 *   type: rcu
 *   acquired: true
 *   released: true
 *   desc: Taken only when the files_struct is shared with other threads, in
 *     which case the fd lookup in fdget() uses RCU and takes a file reference.
 *     A private table needs no RCU. The RCU read lock is acquired and released
 *     internally by the fd lookup path, not held across the entire syscall.
 *     fdput() releases the file reference count, not the RCU lock.
 *
 * signal: SIGPIPE
 *   number: SIGPIPE
 *   direction: send
 *   action: terminate
 *   condition: Writing to a pipe or socket with no readers
 *   desc: When writing to a pipe whose read end is closed, or a socket whose
 *     peer has closed, SIGPIPE is sent to the calling process. The default
 *     action terminates the process. Use signal(SIGPIPE, SIG_IGN) to suppress
 *     for write(). EPIPE is returned regardless of signal disposition.
 *   timing: during
 *
 * signal: SIGXFSZ
 *   number: SIGXFSZ
 *   direction: send
 *   action: coredump
 *   condition: Write starts at or beyond RLIMIT_FSIZE
 *   desc: When a write starts at or beyond the soft file size limit
 *     (RLIMIT_FSIZE), generic_write_check_limits() sends SIGXFSZ and the write
 *     returns EFBIG. The default action terminates with a core dump. A write
 *     that starts below the limit but would cross it is shortened to end at
 *     the limit, with no signal. If RLIMIT_FSIZE is RLIM_INFINITY, no signal
 *     is sent.
 *   timing: during
 *
 * signal: Any signal
 *   direction: receive
 *   action: return
 *   condition: While blocked waiting for space (pipes, sockets)
 *   desc: The syscall may be interrupted by signals while waiting for buffer
 *     space to become available. If interrupted before any data is written,
 *     returns -EINTR or -ERESTARTSYS. If data was already written, returns the
 *     byte count. Restartable if SA_RESTART is set and no data was written.
 *   errno: -EINTR
 *   timing: during
 *   restartable: yes
 *
 * side-effect: file_position
 *   target: file->f_pos
 *   condition: For seekable files when write succeeds (returns > 0)
 *   desc: The file offset (f_pos) is advanced by the number of bytes written.
 *     For files opened with O_APPEND, f_pos is first set to file size. For
 *     stream files (FMODE_STREAM such as pipes and sockets), the offset is not
 *     used or modified. Position updates are protected by f_pos_lock when
 *     shared.
 *   reversible: no
 *
 * side-effect: modify_state
 *   target: inode timestamps (mtime, ctime)
 *   condition: Before the data is copied, for a non-zero count on the generic
 *     write path
 *   desc: Updates the file's modification time (mtime) and change time (ctime)
 *     via file_update_time(), which runs before the data is copied, so the
 *     timestamps can change even if the write then fails or is short. The
 *     update is skipped for inodes flagged NOCMTIME and when the timestamps
 *     are unchanged. The timestamp precision depends on whether the filesystem
 *     type sets FS_MGTIME (multigrain timestamps).
 *   reversible: no
 *
 * side-effect: modify_state
 *   target: SUID/SGID bits (mode) and file capabilities
 *   condition: Write to a regular file that has S_ISUID, S_ISGID, or file
 *     capabilities set, with a non-zero count
 *   desc: file_remove_privs() clears S_ISUID, and clears S_ISGID when S_IXGRP
 *     is also set or the caller is not in the file's group, unless the caller
 *     has CAP_FSETID. File capabilities (the security.capability xattr) are
 *     removed as well, regardless of CAP_FSETID. This is a security feature to
 *     prevent privilege escalation via modified setuid binaries. It runs before
 *     the data is copied.
 *   reversible: no
 *
 * side-effect: modify_state
 *   target: file data
 *   condition: When write succeeds (returns > 0)
 *   desc: Modifies the file's data content. For regular files, data is written
 *     to the page cache (buffered I/O) or directly to storage (O_DIRECT).
 *     Data is not guaranteed to be persistent until fsync() or fdatasync()
 *     completes, or writeback has written it to storage.
 *   reversible: no
 *
 * side-effect: modify_state
 *   target: task I/O accounting
 *   condition: When CONFIG_TASK_XACCT is enabled and the write method is
 *     invoked
 *   desc: Updates the current task's I/O accounting statistics. The wchar field
 *     (write characters) is incremented by bytes written via add_wchar() only on
 *     successful writes (ret > 0). The syscw field (syscall write count) is
 *     incremented via inc_syscw() once the write method has been invoked, even
 *     if it fails, but not when the FMODE_WRITE, FMODE_CAN_WRITE, access_ok()
 *     or rw_verify_area() checks fail. These statistics are visible in
 *     /proc/[pid]/io.
 *   reversible: no
 *
 * side-effect: modify_state
 *   target: fsnotify events
 *   condition: When write returns > 0
 *   desc: Generates an FS_MODIFY fsnotify event via fsnotify_modify(), allowing
 *     inotify, fanotify, and dnotify watchers to be notified of the write.
 *
 * capability: CAP_FSETID
 *   type: bypass_check
 *   allows: Keep the SUID and SGID bits set when the file is written
 *   without: SUID is cleared on write, and SGID is cleared if S_IXGRP is set
 *     or the caller is not in the file's group
 *   condition: Checked in setattr_should_drop_suidgid() during
 *     file_remove_privs()
 *
 * constraint: MAX_RW_COUNT
 *   desc: After the access_ok() and rw_verify_area() checks pass, the count
 *     parameter is silently clamped to MAX_RW_COUNT (INT_MAX & PAGE_MASK, just
 *     under 2GB) to prevent integer overflow in internal calculations. This is
 *     transparent to the caller.
 *   expr: actual_count = min(count, MAX_RW_COUNT)
 *
 * constraint: File must be open for writing
 *   desc: The file descriptor must have been opened with O_WRONLY or O_RDWR.
 *     Files opened with O_RDONLY or O_PATH cannot be written and return EBADF.
 *     The file must have both FMODE_WRITE and FMODE_CAN_WRITE flags set.
 *   expr: (file->f_mode & FMODE_WRITE) && (file->f_mode & FMODE_CAN_WRITE)
 *
 * constraint: RLIMIT_FSIZE
 *   desc: The size of data written is constrained by the RLIMIT_FSIZE resource
 *     limit. In generic_write_check_limits(), a write that starts at or beyond
 *     the limit sends SIGXFSZ and returns EFBIG. A write that starts below the
 *     limit but would cross it is shortened to limit - pos bytes, with no
 *     signal.
 *   expr: pos < rlimit(RLIMIT_FSIZE) || rlimit(RLIMIT_FSIZE) == RLIM_INFINITY
 *
 * constraint: File seals
 *   desc: For memfd or shmem files with F_SEAL_WRITE or F_SEAL_FUTURE_WRITE
 *     seals applied, all write operations fail with EPERM. With F_SEAL_GROW,
 *     writes that would extend file size fail with EPERM.
 *
 * examples: n = write(fd, buf, sizeof(buf));  // Basic write
 *   n = write(STDOUT_FILENO, msg, strlen(msg));  // Write to stdout
 *   // Handle short writes:
 *   while (total < len) {
 *     n = write(fd, buf + total, len - total);
 *     if (n < 0) break;
 *     total += n;
 *   }
 *   // Pipe error handling:
 *   if (write(pipefd[1], &byte, 1) < 0 && errno == EPIPE)
 *     handle_broken_pipe();
 *
 * notes: The behavior of write() varies significantly depending on the type of
 *   file descriptor:
 *
 *   - Regular files: Writes to the page cache (buffered) or directly to storage
 *     (O_DIRECT). Short writes are rare except near RLIMIT_FSIZE or disk full.
 *     O_APPEND is atomic for determining write position.
 *
 *   - Pipes and FIFOs: Blocking by default. Writes up to PIPE_BUF (4096 bytes
 *     on Linux) are guaranteed atomic. Larger writes may be interleaved with
 *     writes from other processes. Blocks if pipe is full; returns EAGAIN with
 *     O_NONBLOCK. SIGPIPE/EPIPE if no readers.
 *
 *   - Sockets: Behavior depends on socket type and protocol. Stream sockets
 *     (TCP) may return partial writes. Datagram sockets (UDP) typically write
 *     complete messages or fail. SIGPIPE/EPIPE for broken connections (unless
 *     MSG_NOSIGNAL). EDESTADDRREQ for unconnected datagram sockets.
 *
 *   - Terminals: May block on flow control. Canonical vs raw mode affects
 *     behavior. Special characters may be interpreted.
 *
 *   - Device special files: Behavior is device-specific. Block devices behave
 *     similarly to regular files. Character device behavior varies.
 *
 *   Race condition considerations: Concurrent writes from threads sharing a
 *   file description race on the file position. Linux 3.14+ provides atomic
 *   position updates via f_pos_lock for regular files (FMODE_ATOMIC_POS), but
 *   for maximum safety, use pwrite() for concurrent positioned writes.
 *
 *   O_DIRECT writes bypass the page cache and typically require buffer and
 *   offset alignment to filesystem block size. Query requirements via statx()
 *   with STATX_DIOALIGN (Linux 6.1+). Unaligned O_DIRECT writes return EINVAL
 *   on most filesystems.
 *
 *   For zero-copy writes, consider using splice(2), sendfile(2), or vmsplice(2)
 *   instead of copying data through user-space buffers with write().
 *
 *   Partial writes (short writes) must be handled by application code.
 *   Applications should loop until all data is written or an error occurs.
 */
SYSCALL_DEFINE3(write, unsigned int, fd, const char __user *, buf,
		size_t, count)
{
	return ksys_write(fd, buf, count);
}

ssize_t ksys_pread64(unsigned int fd, char __user *buf, size_t count,
		     loff_t pos)
{
	if (pos < 0)
		return -EINVAL;

	CLASS(fd, f)(fd);
	if (fd_empty(f))
		return -EBADF;

	if (fd_file(f)->f_mode & FMODE_PREAD)
		return vfs_read(fd_file(f), buf, count, &pos);

	return -ESPIPE;
}

SYSCALL_DEFINE4(pread64, unsigned int, fd, char __user *, buf,
			size_t, count, loff_t, pos)
{
	return ksys_pread64(fd, buf, count, pos);
}

#if defined(CONFIG_COMPAT) && defined(__ARCH_WANT_COMPAT_PREAD64)
COMPAT_SYSCALL_DEFINE5(pread64, unsigned int, fd, char __user *, buf,
		       size_t, count, compat_arg_u64_dual(pos))
{
	return ksys_pread64(fd, buf, count, compat_arg_u64_glue(pos));
}
#endif

ssize_t ksys_pwrite64(unsigned int fd, const char __user *buf,
		      size_t count, loff_t pos)
{
	if (pos < 0)
		return -EINVAL;

	CLASS(fd, f)(fd);
	if (fd_empty(f))
		return -EBADF;

	if (fd_file(f)->f_mode & FMODE_PWRITE)
		return vfs_write(fd_file(f), buf, count, &pos);

	return -ESPIPE;
}

SYSCALL_DEFINE4(pwrite64, unsigned int, fd, const char __user *, buf,
			 size_t, count, loff_t, pos)
{
	return ksys_pwrite64(fd, buf, count, pos);
}

#if defined(CONFIG_COMPAT) && defined(__ARCH_WANT_COMPAT_PWRITE64)
COMPAT_SYSCALL_DEFINE5(pwrite64, unsigned int, fd, const char __user *, buf,
		       size_t, count, compat_arg_u64_dual(pos))
{
	return ksys_pwrite64(fd, buf, count, compat_arg_u64_glue(pos));
}
#endif

static ssize_t do_iter_readv_writev(struct file *filp, struct iov_iter *iter,
		loff_t *ppos, int type, rwf_t flags)
{
	struct kiocb kiocb;
	ssize_t ret;

	init_sync_kiocb(&kiocb, filp);
	ret = kiocb_set_rw_flags(&kiocb, flags, type);
	if (ret)
		return ret;
	kiocb.ki_pos = (ppos ? *ppos : 0);

	if (type == READ)
		ret = filp->f_op->read_iter(&kiocb, iter);
	else
		ret = filp->f_op->write_iter(&kiocb, iter);
	BUG_ON(ret == -EIOCBQUEUED);
	if (ppos)
		*ppos = kiocb.ki_pos;
	return ret;
}

/* Do it by hand, with file-ops */
static ssize_t do_loop_readv_writev(struct file *filp, struct iov_iter *iter,
		loff_t *ppos, int type, rwf_t flags)
{
	ssize_t ret = 0;

	if (flags & ~RWF_HIPRI)
		return -EOPNOTSUPP;

	while (iov_iter_count(iter)) {
		ssize_t nr;

		if (type == READ) {
			nr = filp->f_op->read(filp, iter_iov_addr(iter),
						iter_iov_len(iter), ppos);
		} else {
			nr = filp->f_op->write(filp, iter_iov_addr(iter),
						iter_iov_len(iter), ppos);
		}

		if (nr < 0) {
			if (!ret)
				ret = nr;
			break;
		}
		ret += nr;
		if (nr != iter_iov_len(iter))
			break;
		iov_iter_advance(iter, nr);
	}

	return ret;
}

ssize_t vfs_iocb_iter_read(struct file *file, struct kiocb *iocb,
			   struct iov_iter *iter)
{
	size_t tot_len;
	ssize_t ret = 0;

	if (!file->f_op->read_iter)
		return -EINVAL;
	if (!(file->f_mode & FMODE_READ))
		return -EBADF;
	if (!(file->f_mode & FMODE_CAN_READ))
		return -EINVAL;

	tot_len = iov_iter_count(iter);
	if (!tot_len)
		goto out;
	ret = rw_verify_area(READ, file, &iocb->ki_pos, tot_len);
	if (ret < 0)
		return ret;

	ret = file->f_op->read_iter(iocb, iter);
out:
	if (ret >= 0)
		fsnotify_access(file);
	return ret;
}
EXPORT_SYMBOL(vfs_iocb_iter_read);

ssize_t vfs_iter_read(struct file *file, struct iov_iter *iter, loff_t *ppos,
		      rwf_t flags)
{
	size_t tot_len;
	ssize_t ret = 0;

	if (!file->f_op->read_iter)
		return -EINVAL;
	if (!(file->f_mode & FMODE_READ))
		return -EBADF;
	if (!(file->f_mode & FMODE_CAN_READ))
		return -EINVAL;

	tot_len = iov_iter_count(iter);
	if (!tot_len)
		goto out;
	ret = rw_verify_area(READ, file, ppos, tot_len);
	if (ret < 0)
		return ret;

	ret = do_iter_readv_writev(file, iter, ppos, READ, flags);
out:
	if (ret >= 0)
		fsnotify_access(file);
	return ret;
}
EXPORT_SYMBOL(vfs_iter_read);

/*
 * Caller is responsible for calling kiocb_end_write() on completion
 * if async iocb was queued.
 */
ssize_t vfs_iocb_iter_write(struct file *file, struct kiocb *iocb,
			    struct iov_iter *iter)
{
	size_t tot_len;
	ssize_t ret = 0;

	if (!file->f_op->write_iter)
		return -EINVAL;
	if (!(file->f_mode & FMODE_WRITE))
		return -EBADF;
	if (!(file->f_mode & FMODE_CAN_WRITE))
		return -EINVAL;

	tot_len = iov_iter_count(iter);
	if (!tot_len)
		return 0;
	ret = rw_verify_area(WRITE, file, &iocb->ki_pos, tot_len);
	if (ret < 0)
		return ret;

	kiocb_start_write(iocb);
	ret = file->f_op->write_iter(iocb, iter);
	if (ret != -EIOCBQUEUED)
		kiocb_end_write(iocb);
	if (ret > 0)
		fsnotify_modify(file);

	return ret;
}
EXPORT_SYMBOL(vfs_iocb_iter_write);

ssize_t vfs_iter_write(struct file *file, struct iov_iter *iter, loff_t *ppos,
		       rwf_t flags)
{
	size_t tot_len;
	ssize_t ret;

	if (!(file->f_mode & FMODE_WRITE))
		return -EBADF;
	if (!(file->f_mode & FMODE_CAN_WRITE))
		return -EINVAL;
	if (!file->f_op->write_iter)
		return -EINVAL;

	tot_len = iov_iter_count(iter);
	if (!tot_len)
		return 0;

	ret = rw_verify_area(WRITE, file, ppos, tot_len);
	if (ret < 0)
		return ret;

	file_start_write(file);
	ret = do_iter_readv_writev(file, iter, ppos, WRITE, flags);
	if (ret > 0)
		fsnotify_modify(file);
	file_end_write(file);

	return ret;
}
EXPORT_SYMBOL(vfs_iter_write);

static ssize_t vfs_readv(struct file *file, const struct iovec __user *vec,
			 unsigned long vlen, loff_t *pos, rwf_t flags)
{
	struct iovec iovstack[UIO_FASTIOV];
	struct iovec *iov = iovstack;
	struct iov_iter iter;
	size_t tot_len;
	ssize_t ret = 0;

	if (!(file->f_mode & FMODE_READ))
		return -EBADF;
	if (!(file->f_mode & FMODE_CAN_READ))
		return -EINVAL;

	ret = import_iovec(ITER_DEST, vec, vlen, ARRAY_SIZE(iovstack), &iov,
			   &iter);
	if (ret < 0)
		return ret;

	tot_len = iov_iter_count(&iter);
	if (!tot_len)
		goto out;

	ret = rw_verify_area(READ, file, pos, tot_len);
	if (ret < 0)
		goto out;

	if (file->f_op->read_iter)
		ret = do_iter_readv_writev(file, &iter, pos, READ, flags);
	else
		ret = do_loop_readv_writev(file, &iter, pos, READ, flags);
out:
	if (ret >= 0)
		fsnotify_access(file);
	kfree(iov);
	return ret;
}

static ssize_t vfs_writev(struct file *file, const struct iovec __user *vec,
			  unsigned long vlen, loff_t *pos, rwf_t flags)
{
	struct iovec iovstack[UIO_FASTIOV];
	struct iovec *iov = iovstack;
	struct iov_iter iter;
	size_t tot_len;
	ssize_t ret = 0;

	if (!(file->f_mode & FMODE_WRITE))
		return -EBADF;
	if (!(file->f_mode & FMODE_CAN_WRITE))
		return -EINVAL;

	ret = import_iovec(ITER_SOURCE, vec, vlen, ARRAY_SIZE(iovstack), &iov,
			   &iter);
	if (ret < 0)
		return ret;

	tot_len = iov_iter_count(&iter);
	if (!tot_len)
		goto out;

	ret = rw_verify_area(WRITE, file, pos, tot_len);
	if (ret < 0)
		goto out;

	file_start_write(file);
	if (file->f_op->write_iter)
		ret = do_iter_readv_writev(file, &iter, pos, WRITE, flags);
	else
		ret = do_loop_readv_writev(file, &iter, pos, WRITE, flags);
	if (ret > 0)
		fsnotify_modify(file);
	file_end_write(file);
out:
	kfree(iov);
	return ret;
}

static ssize_t do_readv(unsigned long fd, const struct iovec __user *vec,
			unsigned long vlen, rwf_t flags)
{
	CLASS(fd_pos, f)(fd);
	ssize_t ret = -EBADF;

	if (!fd_empty(f)) {
		loff_t pos, *ppos = file_ppos(fd_file(f));
		if (ppos) {
			pos = *ppos;
			ppos = &pos;
		}
		ret = vfs_readv(fd_file(f), vec, vlen, ppos, flags);
		if (ret >= 0 && ppos)
			fd_file(f)->f_pos = pos;
	}

	if (ret > 0)
		add_rchar(current, ret);
	inc_syscr(current);
	return ret;
}

static ssize_t do_writev(unsigned long fd, const struct iovec __user *vec,
			 unsigned long vlen, rwf_t flags)
{
	CLASS(fd_pos, f)(fd);
	ssize_t ret = -EBADF;

	if (!fd_empty(f)) {
		loff_t pos, *ppos = file_ppos(fd_file(f));
		if (ppos) {
			pos = *ppos;
			ppos = &pos;
		}
		ret = vfs_writev(fd_file(f), vec, vlen, ppos, flags);
		if (ret >= 0 && ppos)
			fd_file(f)->f_pos = pos;
	}

	if (ret > 0)
		add_wchar(current, ret);
	inc_syscw(current);
	return ret;
}

static inline loff_t pos_from_hilo(unsigned long high, unsigned long low)
{
#define HALF_LONG_BITS (BITS_PER_LONG / 2)
	return (((loff_t)high << HALF_LONG_BITS) << HALF_LONG_BITS) | low;
}

static ssize_t do_preadv(unsigned long fd, const struct iovec __user *vec,
			 unsigned long vlen, loff_t pos, rwf_t flags)
{
	ssize_t ret = -EBADF;

	if (pos < 0)
		return -EINVAL;

	CLASS(fd, f)(fd);
	if (!fd_empty(f)) {
		ret = -ESPIPE;
		if (fd_file(f)->f_mode & FMODE_PREAD)
			ret = vfs_readv(fd_file(f), vec, vlen, &pos, flags);
	}

	if (ret > 0)
		add_rchar(current, ret);
	inc_syscr(current);
	return ret;
}

static ssize_t do_pwritev(unsigned long fd, const struct iovec __user *vec,
			  unsigned long vlen, loff_t pos, rwf_t flags)
{
	ssize_t ret = -EBADF;

	if (pos < 0)
		return -EINVAL;

	CLASS(fd, f)(fd);
	if (!fd_empty(f)) {
		ret = -ESPIPE;
		if (fd_file(f)->f_mode & FMODE_PWRITE)
			ret = vfs_writev(fd_file(f), vec, vlen, &pos, flags);
	}

	if (ret > 0)
		add_wchar(current, ret);
	inc_syscw(current);
	return ret;
}

SYSCALL_DEFINE3(readv, unsigned long, fd, const struct iovec __user *, vec,
		unsigned long, vlen)
{
	return do_readv(fd, vec, vlen, 0);
}

SYSCALL_DEFINE3(writev, unsigned long, fd, const struct iovec __user *, vec,
		unsigned long, vlen)
{
	return do_writev(fd, vec, vlen, 0);
}

SYSCALL_DEFINE5(preadv, unsigned long, fd, const struct iovec __user *, vec,
		unsigned long, vlen, unsigned long, pos_l, unsigned long, pos_h)
{
	loff_t pos = pos_from_hilo(pos_h, pos_l);

	return do_preadv(fd, vec, vlen, pos, 0);
}

SYSCALL_DEFINE6(preadv2, unsigned long, fd, const struct iovec __user *, vec,
		unsigned long, vlen, unsigned long, pos_l, unsigned long, pos_h,
		rwf_t, flags)
{
	loff_t pos = pos_from_hilo(pos_h, pos_l);

	if (pos == -1)
		return do_readv(fd, vec, vlen, flags);

	return do_preadv(fd, vec, vlen, pos, flags);
}

SYSCALL_DEFINE5(pwritev, unsigned long, fd, const struct iovec __user *, vec,
		unsigned long, vlen, unsigned long, pos_l, unsigned long, pos_h)
{
	loff_t pos = pos_from_hilo(pos_h, pos_l);

	return do_pwritev(fd, vec, vlen, pos, 0);
}

SYSCALL_DEFINE6(pwritev2, unsigned long, fd, const struct iovec __user *, vec,
		unsigned long, vlen, unsigned long, pos_l, unsigned long, pos_h,
		rwf_t, flags)
{
	loff_t pos = pos_from_hilo(pos_h, pos_l);

	if (pos == -1)
		return do_writev(fd, vec, vlen, flags);

	return do_pwritev(fd, vec, vlen, pos, flags);
}

/*
 * Various compat syscalls.  Note that they all pretend to take a native
 * iovec - import_iovec will properly treat those as compat_iovecs based on
 * in_compat_syscall().
 */
#ifdef CONFIG_COMPAT
#ifdef __ARCH_WANT_COMPAT_SYS_PREADV64
COMPAT_SYSCALL_DEFINE4(preadv64, unsigned long, fd,
		const struct iovec __user *, vec,
		unsigned long, vlen, loff_t, pos)
{
	return do_preadv(fd, vec, vlen, pos, 0);
}
#endif

COMPAT_SYSCALL_DEFINE5(preadv, compat_ulong_t, fd,
		const struct iovec __user *, vec,
		compat_ulong_t, vlen, u32, pos_low, u32, pos_high)
{
	loff_t pos = ((loff_t)pos_high << 32) | pos_low;

	return do_preadv(fd, vec, vlen, pos, 0);
}

#ifdef __ARCH_WANT_COMPAT_SYS_PREADV64V2
COMPAT_SYSCALL_DEFINE5(preadv64v2, unsigned long, fd,
		const struct iovec __user *, vec,
		unsigned long, vlen, loff_t, pos, rwf_t, flags)
{
	if (pos == -1)
		return do_readv(fd, vec, vlen, flags);
	return do_preadv(fd, vec, vlen, pos, flags);
}
#endif

COMPAT_SYSCALL_DEFINE6(preadv2, compat_ulong_t, fd,
		const struct iovec __user *, vec,
		compat_ulong_t, vlen, u32, pos_low, u32, pos_high,
		rwf_t, flags)
{
	loff_t pos = ((loff_t)pos_high << 32) | pos_low;

	if (pos == -1)
		return do_readv(fd, vec, vlen, flags);
	return do_preadv(fd, vec, vlen, pos, flags);
}

#ifdef __ARCH_WANT_COMPAT_SYS_PWRITEV64
COMPAT_SYSCALL_DEFINE4(pwritev64, unsigned long, fd,
		const struct iovec __user *, vec,
		unsigned long, vlen, loff_t, pos)
{
	return do_pwritev(fd, vec, vlen, pos, 0);
}
#endif

COMPAT_SYSCALL_DEFINE5(pwritev, compat_ulong_t, fd,
		const struct iovec __user *,vec,
		compat_ulong_t, vlen, u32, pos_low, u32, pos_high)
{
	loff_t pos = ((loff_t)pos_high << 32) | pos_low;

	return do_pwritev(fd, vec, vlen, pos, 0);
}

#ifdef __ARCH_WANT_COMPAT_SYS_PWRITEV64V2
COMPAT_SYSCALL_DEFINE5(pwritev64v2, unsigned long, fd,
		const struct iovec __user *, vec,
		unsigned long, vlen, loff_t, pos, rwf_t, flags)
{
	if (pos == -1)
		return do_writev(fd, vec, vlen, flags);
	return do_pwritev(fd, vec, vlen, pos, flags);
}
#endif

COMPAT_SYSCALL_DEFINE6(pwritev2, compat_ulong_t, fd,
		const struct iovec __user *,vec,
		compat_ulong_t, vlen, u32, pos_low, u32, pos_high, rwf_t, flags)
{
	loff_t pos = ((loff_t)pos_high << 32) | pos_low;

	if (pos == -1)
		return do_writev(fd, vec, vlen, flags);
	return do_pwritev(fd, vec, vlen, pos, flags);
}
#endif /* CONFIG_COMPAT */

static ssize_t do_sendfile(int out_fd, int in_fd, loff_t *ppos,
			   size_t count, loff_t max)
{
	struct inode *in_inode, *out_inode;
	struct pipe_inode_info *opipe;
	loff_t pos;
	loff_t out_pos;
	ssize_t retval;
	int fl;

	/*
	 * Get input file, and verify that it is ok..
	 */
	CLASS(fd, in)(in_fd);
	if (fd_empty(in))
		return -EBADF;
	if (!(fd_file(in)->f_mode & FMODE_READ))
		return -EBADF;
	if (!ppos) {
		pos = fd_file(in)->f_pos;
	} else {
		pos = *ppos;
		if (!(fd_file(in)->f_mode & FMODE_PREAD))
			return -ESPIPE;
	}
	retval = rw_verify_area(READ, fd_file(in), &pos, count);
	if (retval < 0)
		return retval;
	if (count > MAX_RW_COUNT)
		count =  MAX_RW_COUNT;

	/*
	 * Get output file, and verify that it is ok..
	 */
	CLASS(fd, out)(out_fd);
	if (fd_empty(out))
		return -EBADF;
	if (!(fd_file(out)->f_mode & FMODE_WRITE))
		return -EBADF;
	in_inode = file_inode(fd_file(in));
	out_inode = file_inode(fd_file(out));
	out_pos = fd_file(out)->f_pos;

	if (!max)
		max = min(in_inode->i_sb->s_maxbytes, out_inode->i_sb->s_maxbytes);

	if (unlikely(pos + count > max)) {
		if (pos >= max)
			return -EOVERFLOW;
		count = max - pos;
	}

	fl = 0;
#if 0
	/*
	 * We need to debate whether we can enable this or not. The
	 * man page documents EAGAIN return for the output at least,
	 * and the application is arguably buggy if it doesn't expect
	 * EAGAIN on a non-blocking file descriptor.
	 */
	if (fd_file(in)->f_flags & O_NONBLOCK)
		fl = SPLICE_F_NONBLOCK;
#endif
	opipe = get_pipe_info(fd_file(out), true);
	if (!opipe) {
		retval = rw_verify_area(WRITE, fd_file(out), &out_pos, count);
		if (retval < 0)
			return retval;
		retval = do_splice_direct(fd_file(in), &pos, fd_file(out), &out_pos,
					  count, fl);
	} else {
		if (fd_file(out)->f_flags & O_NONBLOCK)
			fl |= SPLICE_F_NONBLOCK;

		retval = splice_file_to_pipe(fd_file(in), opipe, &pos, count, fl);
	}

	if (retval > 0) {
		add_rchar(current, retval);
		add_wchar(current, retval);
		fsnotify_access(fd_file(in));
		fsnotify_modify(fd_file(out));
		fd_file(out)->f_pos = out_pos;
		if (ppos)
			*ppos = pos;
		else
			fd_file(in)->f_pos = pos;
	}

	inc_syscr(current);
	inc_syscw(current);
	if (pos > max)
		retval = -EOVERFLOW;
	return retval;
}

SYSCALL_DEFINE4(sendfile, int, out_fd, int, in_fd, off_t __user *, offset, size_t, count)
{
	loff_t pos;
	off_t off;
	ssize_t ret;

	if (offset) {
		if (unlikely(get_user(off, offset)))
			return -EFAULT;
		pos = off;
		ret = do_sendfile(out_fd, in_fd, &pos, count, MAX_NON_LFS);
		if (unlikely(put_user(pos, offset)))
			return -EFAULT;
		return ret;
	}

	return do_sendfile(out_fd, in_fd, NULL, count, 0);
}

SYSCALL_DEFINE4(sendfile64, int, out_fd, int, in_fd, loff_t __user *, offset, size_t, count)
{
	loff_t pos;
	ssize_t ret;

	if (offset) {
		if (unlikely(copy_from_user(&pos, offset, sizeof(loff_t))))
			return -EFAULT;
		ret = do_sendfile(out_fd, in_fd, &pos, count, 0);
		if (unlikely(put_user(pos, offset)))
			return -EFAULT;
		return ret;
	}

	return do_sendfile(out_fd, in_fd, NULL, count, 0);
}

#ifdef CONFIG_COMPAT
COMPAT_SYSCALL_DEFINE4(sendfile, int, out_fd, int, in_fd,
		compat_off_t __user *, offset, compat_size_t, count)
{
	loff_t pos;
	off_t off;
	ssize_t ret;

	if (offset) {
		if (unlikely(get_user(off, offset)))
			return -EFAULT;
		pos = off;
		ret = do_sendfile(out_fd, in_fd, &pos, count, MAX_NON_LFS);
		if (unlikely(put_user(pos, offset)))
			return -EFAULT;
		return ret;
	}

	return do_sendfile(out_fd, in_fd, NULL, count, 0);
}

COMPAT_SYSCALL_DEFINE4(sendfile64, int, out_fd, int, in_fd,
		compat_loff_t __user *, offset, compat_size_t, count)
{
	loff_t pos;
	ssize_t ret;

	if (offset) {
		if (unlikely(copy_from_user(&pos, offset, sizeof(loff_t))))
			return -EFAULT;
		ret = do_sendfile(out_fd, in_fd, &pos, count, 0);
		if (unlikely(put_user(pos, offset)))
			return -EFAULT;
		return ret;
	}

	return do_sendfile(out_fd, in_fd, NULL, count, 0);
}
#endif

/*
 * Performs necessary checks before doing a file copy
 *
 * Can adjust amount of bytes to copy via @req_count argument.
 * Returns appropriate error code that caller should return or
 * zero in case the copy should be allowed.
 */
static int generic_copy_file_checks(struct file *file_in, loff_t pos_in,
				    struct file *file_out, loff_t pos_out,
				    size_t *req_count, unsigned int flags)
{
	struct inode *inode_in = file_inode(file_in);
	struct inode *inode_out = file_inode(file_out);
	uint64_t count = *req_count;
	loff_t size_in;
	int ret;

	ret = generic_file_rw_checks(file_in, file_out);
	if (ret)
		return ret;

	/*
	 * We allow some filesystems to handle cross sb copy, but passing
	 * a file of the wrong filesystem type to filesystem driver can result
	 * in an attempt to dereference the wrong type of ->private_data, so
	 * avoid doing that until we really have a good reason.
	 *
	 * nfs and cifs define several different file_system_type structures
	 * and several different sets of file_operations, but they all end up
	 * using the same ->copy_file_range() function pointer.
	 */
	if (flags & COPY_FILE_SPLICE) {
		/* cross sb splice is allowed */
	} else if (file_out->f_op->copy_file_range) {
		if (file_in->f_op->copy_file_range !=
		    file_out->f_op->copy_file_range)
			return -EXDEV;
	} else if (file_inode(file_in)->i_sb != file_inode(file_out)->i_sb) {
		return -EXDEV;
	}

	/* Don't touch certain kinds of inodes */
	if (IS_IMMUTABLE(inode_out))
		return -EPERM;

	if (IS_SWAPFILE(inode_in) || IS_SWAPFILE(inode_out))
		return -ETXTBSY;

	/* Ensure offsets don't wrap. */
	if (pos_in + count < pos_in || pos_out + count < pos_out)
		return -EOVERFLOW;

	/* Shorten the copy to EOF */
	size_in = i_size_read(inode_in);
	if (pos_in >= size_in)
		count = 0;
	else
		count = min(count, size_in - (uint64_t)pos_in);

	ret = generic_write_check_limits(file_out, pos_out, &count);
	if (ret)
		return ret;

	/* Don't allow overlapped copying within the same file. */
	if (inode_in == inode_out &&
	    pos_out + count > pos_in &&
	    pos_out < pos_in + count)
		return -EINVAL;

	*req_count = count;
	return 0;
}

/*
 * copy_file_range() differs from regular file read and write in that it
 * specifically allows return partial success.  When it does so is up to
 * the copy_file_range method.
 */
ssize_t vfs_copy_file_range(struct file *file_in, loff_t pos_in,
			    struct file *file_out, loff_t pos_out,
			    size_t len, unsigned int flags)
{
	ssize_t ret;
	bool splice = flags & COPY_FILE_SPLICE;
	bool samesb = file_inode(file_in)->i_sb == file_inode(file_out)->i_sb;

	if (flags & ~COPY_FILE_SPLICE)
		return -EINVAL;

	ret = generic_copy_file_checks(file_in, pos_in, file_out, pos_out, &len,
				       flags);
	if (unlikely(ret))
		return ret;

	ret = rw_verify_area(READ, file_in, &pos_in, len);
	if (unlikely(ret))
		return ret;

	ret = rw_verify_area(WRITE, file_out, &pos_out, len);
	if (unlikely(ret))
		return ret;

	if (len == 0)
		return 0;

	/*
	 * Make sure return value doesn't overflow in 32bit compat mode.  Also
	 * limit the size for all cases except when calling ->copy_file_range().
	 */
	if (splice || !file_out->f_op->copy_file_range || in_compat_syscall())
		len = min_t(size_t, MAX_RW_COUNT, len);

	file_start_write(file_out);

	/*
	 * Cloning is supported by more file systems, so we implement copy on
	 * same sb using clone, but for filesystems where both clone and copy
	 * are supported (e.g. nfs,cifs), we only call the copy method.
	 */
	if (!splice && file_out->f_op->copy_file_range) {
		ret = file_out->f_op->copy_file_range(file_in, pos_in,
						      file_out, pos_out,
						      len, flags);
	} else if (!splice && file_in->f_op->remap_file_range && samesb) {
		ret = file_in->f_op->remap_file_range(file_in, pos_in,
				file_out, pos_out, len, REMAP_FILE_CAN_SHORTEN);
		/* fallback to splice */
		if (ret <= 0)
			splice = true;
	} else if (samesb) {
		/* Fallback to splice for same sb copy for backward compat */
		splice = true;
	}

	file_end_write(file_out);

	if (!splice)
		goto done;

	/*
	 * We can get here for same sb copy of filesystems that do not implement
	 * ->copy_file_range() in case filesystem does not support clone or in
	 * case filesystem supports clone but rejected the clone request (e.g.
	 * because it was not block aligned).
	 *
	 * In both cases, fall back to kernel copy so we are able to maintain a
	 * consistent story about which filesystems support copy_file_range()
	 * and which filesystems do not, that will allow userspace tools to
	 * make consistent desicions w.r.t using copy_file_range().
	 *
	 * We also get here if caller (e.g. nfsd) requested COPY_FILE_SPLICE
	 * for server-side-copy between any two sb.
	 *
	 * In any case, we call do_splice_direct() and not splice_file_range(),
	 * without file_start_write() held, to avoid possible deadlocks related
	 * to splicing from input file, while file_start_write() is held on
	 * the output file on a different sb.
	 */
	ret = do_splice_direct(file_in, &pos_in, file_out, &pos_out, len, 0);
done:
	if (ret > 0) {
		fsnotify_access(file_in);
		add_rchar(current, ret);
		fsnotify_modify(file_out);
		add_wchar(current, ret);
	}

	inc_syscr(current);
	inc_syscw(current);

	return ret;
}
EXPORT_SYMBOL(vfs_copy_file_range);

SYSCALL_DEFINE6(copy_file_range, int, fd_in, loff_t __user *, off_in,
		int, fd_out, loff_t __user *, off_out,
		size_t, len, unsigned int, flags)
{
	loff_t pos_in;
	loff_t pos_out;
	ssize_t ret = -EBADF;

	CLASS(fd, f_in)(fd_in);
	if (fd_empty(f_in))
		return -EBADF;

	CLASS(fd, f_out)(fd_out);
	if (fd_empty(f_out))
		return -EBADF;

	if (off_in) {
		if (copy_from_user(&pos_in, off_in, sizeof(loff_t)))
			return -EFAULT;
	} else {
		pos_in = fd_file(f_in)->f_pos;
	}

	if (off_out) {
		if (copy_from_user(&pos_out, off_out, sizeof(loff_t)))
			return -EFAULT;
	} else {
		pos_out = fd_file(f_out)->f_pos;
	}

	if (flags != 0)
		return -EINVAL;

	ret = vfs_copy_file_range(fd_file(f_in), pos_in, fd_file(f_out), pos_out, len,
				  flags);
	if (ret > 0) {
		pos_in += ret;
		pos_out += ret;

		if (off_in) {
			if (copy_to_user(off_in, &pos_in, sizeof(loff_t)))
				ret = -EFAULT;
		} else {
			fd_file(f_in)->f_pos = pos_in;
		}

		if (off_out) {
			if (copy_to_user(off_out, &pos_out, sizeof(loff_t)))
				ret = -EFAULT;
		} else {
			fd_file(f_out)->f_pos = pos_out;
		}
	}
	return ret;
}

/*
 * Don't operate on ranges the page cache doesn't support, and don't exceed the
 * LFS limits.  If pos is under the limit it becomes a short access.  If it
 * exceeds the limit we return -EFBIG.
 */
int generic_write_check_limits(struct file *file, loff_t pos, loff_t *count)
{
	struct inode *inode = file->f_mapping->host;
	loff_t max_size = inode->i_sb->s_maxbytes;
	loff_t limit = rlimit(RLIMIT_FSIZE);

	if (limit != RLIM_INFINITY) {
		if (pos >= limit) {
			send_sig(SIGXFSZ, current, 0);
			return -EFBIG;
		}
		*count = min(*count, limit - pos);
	}

	if (!(file->f_flags & O_LARGEFILE))
		max_size = MAX_NON_LFS;

	if (unlikely(pos >= max_size))
		return -EFBIG;

	*count = min(*count, max_size - pos);

	return 0;
}
EXPORT_SYMBOL_GPL(generic_write_check_limits);

/* Like generic_write_checks(), but takes size of write instead of iter. */
int generic_write_checks_count(struct kiocb *iocb, loff_t *count)
{
	struct file *file = iocb->ki_filp;
	struct inode *inode = file->f_mapping->host;

	if (IS_SWAPFILE(inode))
		return -ETXTBSY;

	if (!*count)
		return 0;

	if (iocb->ki_flags & IOCB_APPEND)
		iocb->ki_pos = i_size_read(inode);

	if ((iocb->ki_flags & IOCB_NOWAIT) &&
	    !((iocb->ki_flags & IOCB_DIRECT) ||
	      (file->f_op->fop_flags & FOP_BUFFER_WASYNC)))
		return -EINVAL;

	return generic_write_check_limits(iocb->ki_filp, iocb->ki_pos, count);
}
EXPORT_SYMBOL(generic_write_checks_count);

/*
 * Performs necessary checks before doing a write
 *
 * Can adjust writing position or amount of bytes to write.
 * Returns the number of bytes that may be written on success (which
 * may be less than requested if truncated), or a negative error code.
 */
ssize_t generic_write_checks(struct kiocb *iocb, struct iov_iter *from)
{
	loff_t count = iov_iter_count(from);
	int ret;

	ret = generic_write_checks_count(iocb, &count);
	if (ret)
		return ret;

	iov_iter_truncate(from, count);
	return iov_iter_count(from);
}
EXPORT_SYMBOL(generic_write_checks);

/*
 * Performs common checks before doing a file copy/clone
 * from @file_in to @file_out.
 */
int generic_file_rw_checks(struct file *file_in, struct file *file_out)
{
	struct inode *inode_in = file_inode(file_in);
	struct inode *inode_out = file_inode(file_out);

	/* Don't copy dirs, pipes, sockets... */
	if (S_ISDIR(inode_in->i_mode) || S_ISDIR(inode_out->i_mode))
		return -EISDIR;
	if (!S_ISREG(inode_in->i_mode) || !S_ISREG(inode_out->i_mode))
		return -EINVAL;

	if (!(file_in->f_mode & FMODE_READ) ||
	    !(file_out->f_mode & FMODE_WRITE) ||
	    (file_out->f_flags & O_APPEND))
		return -EBADF;

	return 0;
}

int generic_atomic_write_valid(struct kiocb *iocb, struct iov_iter *iter)
{
	size_t len = iov_iter_count(iter);

	if (!iter_is_ubuf(iter))
		return -EINVAL;

	if (!is_power_of_2(len))
		return -EINVAL;

	if (!IS_ALIGNED(iocb->ki_pos, len))
		return -EINVAL;

	if (!(iocb->ki_flags & IOCB_DIRECT))
		return -EOPNOTSUPP;

	return 0;
}
EXPORT_SYMBOL_GPL(generic_atomic_write_valid);
