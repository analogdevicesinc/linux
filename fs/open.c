// SPDX-License-Identifier: GPL-2.0-only
/*
 *  linux/fs/open.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 */

#include <linux/string.h>
#include <linux/mm.h>
#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/fsnotify.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/namei.h>
#include <linux/backing-dev.h>
#include <linux/capability.h>
#include <linux/securebits.h>
#include <linux/security.h>
#include <linux/mount.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/personality.h>
#include <linux/pagemap.h>
#include <linux/syscalls.h>
#include <linux/rcupdate.h>
#include <linux/audit.h>
#include <linux/falloc.h>
#include <linux/fs_struct.h>
#include <linux/dnotify.h>
#include <linux/compat.h>
#include <linux/mnt_idmapping.h>
#include <linux/filelock.h>

#include "internal.h"

int do_truncate(const struct mnt_idmap *idmap, struct dentry *dentry,
		loff_t length, unsigned int time_attrs, struct file *filp)
{
	int ret;
	struct iattr newattrs;

	/* Not pretty: "inode->i_size" shouldn't really be signed. But it is. */
	if (length < 0)
		return -EINVAL;

	newattrs.ia_size = length;
	newattrs.ia_valid = ATTR_SIZE | time_attrs;
	if (filp) {
		newattrs.ia_file = filp;
		newattrs.ia_valid |= ATTR_FILE;
	}

	/* Remove suid, sgid, and file capabilities on truncate too */
	ret = dentry_needs_remove_privs(idmap, dentry);
	if (ret < 0)
		return ret;
	if (ret)
		newattrs.ia_valid |= ret | ATTR_FORCE;

	ret = inode_lock_killable(dentry->d_inode);
	if (ret)
		return ret;

	/* Note any delegations or leases have already been broken: */
	ret = notify_change(idmap, dentry, &newattrs, NULL);
	inode_unlock(dentry->d_inode);
	return ret;
}

int vfs_truncate(const struct path *path, loff_t length)
{
	const struct mnt_idmap *idmap;
	struct inode *inode;
	int error;

	inode = path->dentry->d_inode;

	/* For directories it's -EISDIR, for other non-regulars - -EINVAL */
	if (S_ISDIR(inode->i_mode))
		return -EISDIR;
	if (!S_ISREG(inode->i_mode))
		return -EINVAL;

	idmap = mnt_idmap(path->mnt);
	error = inode_permission(idmap, inode, MAY_WRITE);
	if (error)
		return error;

	error = fsnotify_truncate_perm(path, length);
	if (error)
		return error;

	error = mnt_want_write(path->mnt);
	if (error)
		return error;

	error = -EPERM;
	if (IS_APPEND(inode))
		goto mnt_drop_write_and_out;

	error = get_write_access(inode);
	if (error)
		goto mnt_drop_write_and_out;

	/*
	 * Make sure that there are no leases.  get_write_access() protects
	 * against the truncate racing with a lease-granting setlease().
	 */
	error = break_lease(inode, O_WRONLY);
	if (error)
		goto put_write_and_out;

	error = security_path_truncate(path);
	if (!error)
		error = do_truncate(idmap, path->dentry, length, 0, NULL);

put_write_and_out:
	put_write_access(inode);
mnt_drop_write_and_out:
	mnt_drop_write(path->mnt);

	return error;
}
EXPORT_SYMBOL_GPL(vfs_truncate);

int ksys_truncate(const char __user *pathname, loff_t length)
{
	unsigned int lookup_flags = LOOKUP_FOLLOW;
	struct path path;
	int error;

	if (length < 0)	/* sorry, but loff_t says... */
		return -EINVAL;

	CLASS(filename, name)(pathname);
retry:
	error = filename_lookup(AT_FDCWD, name, lookup_flags, &path, NULL);
	if (!error) {
		error = vfs_truncate(&path, length);
		path_put(&path);
		if (retry_estale(error, lookup_flags)) {
			lookup_flags |= LOOKUP_REVAL;
			goto retry;
		}
	}
	return error;
}

SYSCALL_DEFINE2(truncate, const char __user *, path, long, length)
{
	return ksys_truncate(path, length);
}

#ifdef CONFIG_COMPAT
COMPAT_SYSCALL_DEFINE2(truncate, const char __user *, path, compat_off_t, length)
{
	return ksys_truncate(path, length);
}
#endif

int do_ftruncate(struct file *file, loff_t length, unsigned int flags)
{
	struct dentry *dentry = file->f_path.dentry;
	struct inode *inode = dentry->d_inode;
	int error;

	if (!S_ISREG(inode->i_mode) || !(file->f_mode & FMODE_WRITE))
		return -EINVAL;

	/*
	 * Cannot ftruncate over 2^31 bytes without large file support, either
	 * through opening with O_LARGEFILE or by using ftruncate64().
	 */
	if (length > MAX_NON_LFS &&
	    !(file->f_flags & O_LARGEFILE) && !(flags & FTRUNCATE_LFS))
		return -EINVAL;

	/* Check IS_APPEND on real upper inode */
	if (IS_APPEND(file_inode(file)))
		return -EPERM;

	error = security_file_truncate(file);
	if (error)
		return error;

	error = fsnotify_truncate_perm(&file->f_path, length);
	if (error)
		return error;

	scoped_guard(super_write, inode->i_sb)
		return do_truncate(file_mnt_idmap(file), dentry, length,
				   ATTR_MTIME | ATTR_CTIME, file);
}

int ksys_ftruncate(unsigned int fd, loff_t length, unsigned int flags)
{
	if (length < 0)
		return -EINVAL;
	CLASS(fd, f)(fd);
	if (fd_empty(f))
		return -EBADF;

	return do_ftruncate(fd_file(f), length, flags);
}

SYSCALL_DEFINE2(ftruncate, unsigned int, fd, off_t, length)
{
	return ksys_ftruncate(fd, length, 0);
}

#ifdef CONFIG_COMPAT
COMPAT_SYSCALL_DEFINE2(ftruncate, unsigned int, fd, compat_off_t, length)
{
	return ksys_ftruncate(fd, length, 0);
}
#endif

/* LFS versions of truncate are only needed on 32 bit machines */
#if BITS_PER_LONG == 32
SYSCALL_DEFINE2(truncate64, const char __user *, path, loff_t, length)
{
	return ksys_truncate(path, length);
}

SYSCALL_DEFINE2(ftruncate64, unsigned int, fd, loff_t, length)
{
	return ksys_ftruncate(fd, length, FTRUNCATE_LFS);
}
#endif /* BITS_PER_LONG == 32 */

#if defined(CONFIG_COMPAT) && defined(__ARCH_WANT_COMPAT_TRUNCATE64)
COMPAT_SYSCALL_DEFINE3(truncate64, const char __user *, pathname,
		       compat_arg_u64_dual(length))
{
	return ksys_truncate(pathname, compat_arg_u64_glue(length));
}
#endif

#if defined(CONFIG_COMPAT) && defined(__ARCH_WANT_COMPAT_FTRUNCATE64)
COMPAT_SYSCALL_DEFINE3(ftruncate64, unsigned int, fd,
		       compat_arg_u64_dual(length))
{
	return ksys_ftruncate(fd, compat_arg_u64_glue(length), FTRUNCATE_LFS);
}
#endif

int vfs_fallocate(struct file *file, int mode, loff_t offset, loff_t len)
{
	struct inode *inode = file_inode(file);
	int ret;
	loff_t sum;

	if (offset < 0 || len <= 0)
		return -EINVAL;

	if (mode & ~(FALLOC_FL_MODE_MASK | FALLOC_FL_KEEP_SIZE))
		return -EOPNOTSUPP;

	/*
	 * Modes are exclusive, even if that is not obvious from the encoding
	 * as bit masks and the mix with the flag in the same namespace.
	 *
	 * To make things even more complicated, FALLOC_FL_ALLOCATE_RANGE is
	 * encoded as no bit set.
	 */
	switch (mode & FALLOC_FL_MODE_MASK) {
	case FALLOC_FL_ALLOCATE_RANGE:
	case FALLOC_FL_UNSHARE_RANGE:
	case FALLOC_FL_ZERO_RANGE:
		break;
	case FALLOC_FL_PUNCH_HOLE:
		if (!(mode & FALLOC_FL_KEEP_SIZE))
			return -EOPNOTSUPP;
		break;
	case FALLOC_FL_COLLAPSE_RANGE:
	case FALLOC_FL_INSERT_RANGE:
	case FALLOC_FL_WRITE_ZEROES:
		if (mode & FALLOC_FL_KEEP_SIZE)
			return -EOPNOTSUPP;
		break;
	default:
		return -EOPNOTSUPP;
	}

	if (!(file->f_mode & FMODE_WRITE))
		return -EBADF;

	/*
	 * On append-only files only space preallocation is supported.
	 */
	if ((mode & ~FALLOC_FL_KEEP_SIZE) && IS_APPEND(inode))
		return -EPERM;

	if (IS_IMMUTABLE(inode))
		return -EPERM;

	/*
	 * We cannot allow any fallocate operation on an active swapfile
	 */
	if (IS_SWAPFILE(inode))
		return -ETXTBSY;

	/*
	 * Revalidate the write permissions, in case security policy has
	 * changed since the files were opened.
	 */
	ret = security_file_permission(file, MAY_WRITE);
	if (ret)
		return ret;

	ret = fsnotify_file_area_perm(file, MAY_WRITE, &offset, len);
	if (ret)
		return ret;

	if (S_ISFIFO(inode->i_mode))
		return -ESPIPE;

	if (S_ISDIR(inode->i_mode))
		return -EISDIR;

	if (!S_ISREG(inode->i_mode) && !S_ISBLK(inode->i_mode))
		return -ENODEV;

	/* Check for wraparound */
	if (check_add_overflow(offset, len, &sum))
		return -EFBIG;

	if (sum > inode->i_sb->s_maxbytes)
		return -EFBIG;

	if (!file->f_op->fallocate)
		return -EOPNOTSUPP;

	file_start_write(file);
	ret = file->f_op->fallocate(file, mode, offset, len);

	/*
	 * Create inotify and fanotify events.
	 *
	 * To keep the logic simple always create events if fallocate succeeds.
	 * This implies that events are even created if the file size remains
	 * unchanged, e.g. when using flag FALLOC_FL_KEEP_SIZE.
	 */
	if (ret == 0)
		fsnotify_modify(file);

	file_end_write(file);
	return ret;
}
EXPORT_SYMBOL_GPL(vfs_fallocate);

int ksys_fallocate(int fd, int mode, loff_t offset, loff_t len)
{
	CLASS(fd, f)(fd);

	if (fd_empty(f))
		return -EBADF;

	return vfs_fallocate(fd_file(f), mode, offset, len);
}

SYSCALL_DEFINE4(fallocate, int, fd, int, mode, loff_t, offset, loff_t, len)
{
	return ksys_fallocate(fd, mode, offset, len);
}

#if defined(CONFIG_COMPAT) && defined(__ARCH_WANT_COMPAT_FALLOCATE)
COMPAT_SYSCALL_DEFINE6(fallocate, int, fd, int, mode, compat_arg_u64_dual(offset),
		       compat_arg_u64_dual(len))
{
	return ksys_fallocate(fd, mode, compat_arg_u64_glue(offset),
			      compat_arg_u64_glue(len));
}
#endif

/*
 * access() needs to use the real uid/gid, not the effective uid/gid.
 * We do this by temporarily clearing all FS-related capabilities and
 * switching the fsuid/fsgid around to the real ones.
 *
 * Creating new credentials is expensive, so we try to skip doing it,
 * which we can if the result would match what we already got.
 */
static bool access_need_override_creds(int flags)
{
	const struct cred *cred;

	if (flags & AT_EACCESS)
		return false;

	cred = current_cred();
	if (!uid_eq(cred->fsuid, cred->uid) ||
	    !gid_eq(cred->fsgid, cred->gid))
		return true;

	if (!issecure(SECURE_NO_SETUID_FIXUP)) {
		kuid_t root_uid = make_kuid(cred->user_ns, 0);
		if (!uid_eq(cred->uid, root_uid)) {
			if (!cap_isclear(cred->cap_effective))
				return true;
		} else {
			if (!cap_isidentical(cred->cap_effective,
			    cred->cap_permitted))
				return true;
		}
	}

	return false;
}

static const struct cred *access_override_creds(void)
{
	struct cred *override_cred;

	override_cred = prepare_creds();
	if (!override_cred)
		return NULL;

	/*
	 * XXX access_need_override_creds performs checks in hopes of skipping
	 * this work. Make sure it stays in sync if making any changes in this
	 * routine.
	 */

	override_cred->fsuid = override_cred->uid;
	override_cred->fsgid = override_cred->gid;

	if (!issecure(SECURE_NO_SETUID_FIXUP)) {
		/* Clear the capabilities if we switch to a non-root user */
		kuid_t root_uid = make_kuid(override_cred->user_ns, 0);
		if (!uid_eq(override_cred->uid, root_uid))
			cap_clear(override_cred->cap_effective);
		else
			override_cred->cap_effective =
				override_cred->cap_permitted;
	}

	/*
	 * The new set of credentials can *only* be used in
	 * task-synchronous circumstances, and does not need
	 * RCU freeing, unless somebody then takes a separate
	 * reference to it.
	 *
	 * NOTE! This is _only_ true because this credential
	 * is used purely for override_creds() that installs
	 * it as the subjective cred. Other threads will be
	 * accessing ->real_cred, not the subjective cred.
	 *
	 * If somebody _does_ make a copy of this (using the
	 * 'get_current_cred()' function), that will clear the
	 * non_rcu field, because now that other user may be
	 * expecting RCU freeing. But normal thread-synchronous
	 * cred accesses will keep things non-racy to avoid RCU
	 * freeing.
	 */
	override_cred->non_rcu = 1;
	return override_creds(override_cred);
}

static int do_faccessat(int dfd, const char __user *filename, int mode, int flags)
{
	struct path path;
	struct inode *inode;
	int res;
	unsigned int lookup_flags = LOOKUP_FOLLOW;
	const struct cred *old_cred = NULL;

	if (mode & ~S_IRWXO)	/* where's F_OK, X_OK, W_OK, R_OK? */
		return -EINVAL;

	if (flags & ~(AT_EACCESS | AT_SYMLINK_NOFOLLOW | AT_EMPTY_PATH))
		return -EINVAL;

	if (flags & AT_SYMLINK_NOFOLLOW)
		lookup_flags &= ~LOOKUP_FOLLOW;

	if (access_need_override_creds(flags)) {
		old_cred = access_override_creds();
		if (!old_cred)
			return -ENOMEM;
	}

	CLASS(filename_uflags, name)(filename, flags);
retry:
	res = filename_lookup(dfd, name, lookup_flags, &path, NULL);
	if (res)
		goto out;

	inode = d_backing_inode(path.dentry);

	if ((mode & MAY_EXEC) && S_ISREG(inode->i_mode)) {
		/*
		 * MAY_EXEC on regular files is denied if the fs is mounted
		 * with the "noexec" flag.
		 */
		res = -EACCES;
		if (path_noexec(&path))
			goto out_path_release;
	}

	res = inode_permission(mnt_idmap(path.mnt), inode, mode | MAY_ACCESS);
	/* SuS v2 requires we report a read only fs too */
	if (res || !(mode & S_IWOTH) || special_file(inode->i_mode))
		goto out_path_release;
	/*
	 * This is a rare case where using __mnt_is_readonly()
	 * is OK without a mnt_want/drop_write() pair.  Since
	 * no actual write to the fs is performed here, we do
	 * not need to telegraph to that to anyone.
	 *
	 * By doing this, we accept that this access is
	 * inherently racy and know that the fs may change
	 * state before we even see this result.
	 */
	if (__mnt_is_readonly(path.mnt))
		res = -EROFS;

out_path_release:
	path_put(&path);
	if (retry_estale(res, lookup_flags)) {
		lookup_flags |= LOOKUP_REVAL;
		goto retry;
	}
out:
	if (old_cred)
		put_cred(revert_creds(old_cred));

	return res;
}

SYSCALL_DEFINE3(faccessat, int, dfd, const char __user *, filename, int, mode)
{
	return do_faccessat(dfd, filename, mode, 0);
}

SYSCALL_DEFINE4(faccessat2, int, dfd, const char __user *, filename, int, mode,
		int, flags)
{
	return do_faccessat(dfd, filename, mode, flags);
}

SYSCALL_DEFINE2(access, const char __user *, filename, int, mode)
{
	return do_faccessat(AT_FDCWD, filename, mode, 0);
}

SYSCALL_DEFINE1(chdir, const char __user *, filename)
{
	struct path path;
	int error;
	unsigned int lookup_flags = LOOKUP_FOLLOW | LOOKUP_DIRECTORY;
	CLASS(filename, name)(filename);
retry:
	error = filename_lookup(AT_FDCWD, name, lookup_flags, &path, NULL);
	if (!error) {
		error = path_permission(&path, MAY_EXEC | MAY_CHDIR);
		if (!error)
			set_fs_pwd(current->fs, &path);
		path_put(&path);
		if (retry_estale(error, lookup_flags)) {
			lookup_flags |= LOOKUP_REVAL;
			goto retry;
		}
	}
	return error;
}

SYSCALL_DEFINE1(fchdir, unsigned int, fd)
{
	int error;

	if ((int)fd == FD_FAILFS_ROOT)
		return failfs_current_chdir();

	CLASS(fd_raw, f)(fd);
	if (fd_empty(f))
		return -EBADF;

	if (!d_can_lookup(fd_file(f)->f_path.dentry))
		return -ENOTDIR;

	error = file_permission(fd_file(f), MAY_EXEC | MAY_CHDIR);
	if (!error)
		set_fs_pwd(current->fs, &fd_file(f)->f_path);
	return error;
}

SYSCALL_DEFINE1(chroot, const char __user *, filename)
{
	struct path path;
	int error;
	unsigned int lookup_flags = LOOKUP_FOLLOW | LOOKUP_DIRECTORY;
	CLASS(filename, name)(filename);
retry:
	error = filename_lookup(AT_FDCWD, name, lookup_flags, &path, NULL);
	if (error)
		return error;

	error = path_permission(&path, MAY_EXEC | MAY_CHDIR);
	if (error)
		goto dput_and_out;

	error = -EPERM;
	if (!ns_capable(current_user_ns(), CAP_SYS_CHROOT))
		goto dput_and_out;
	error = security_path_chroot(&path);
	if (!error)
		set_fs_root(current->fs, &path);
dput_and_out:
	path_put(&path);
	if (retry_estale(error, lookup_flags)) {
		lookup_flags |= LOOKUP_REVAL;
		goto retry;
	}
	return error;
}

SYSCALL_DEFINE2(fchroot, int, fd, unsigned int, flags)
{
	struct path path;
	int error;

	if (flags)
		return -EINVAL;

	if (fd == FD_FAILFS_ROOT) {
		if (!ns_capable(current_user_ns(), CAP_SYS_CHROOT)) {
			if (!task_no_new_privs(current))
				return -EPERM;
			/* A shared fs_struct lets a sibling exec setuid past the check above. */
			if (current->fs->users != 1)
				return -EINVAL;
			/* Moving the root to failfs lifts the old root's ".." barrier. */
			if (current_chrooted())
				return -EPERM;
		}
		failfs_get_root(&path);
	} else {
		CLASS(fd_raw, f)(fd);
		if (fd_empty(f))
			return -EBADF;

		if (!d_can_lookup(fd_file(f)->f_path.dentry))
			return -ENOTDIR;

		error = file_permission(fd_file(f), MAY_EXEC | MAY_CHDIR);
		if (error)
			return error;

		if (!ns_capable(current_user_ns(), CAP_SYS_CHROOT))
			return -EPERM;

		path = fd_file(f)->f_path;
		path_get(&path);
	}

	error = security_path_chroot(&path);
	if (!error)
		set_fs_root(current->fs, &path);
	path_put(&path);
	return error;
}

int chmod_common(const struct path *path, umode_t mode)
{
	struct inode *inode = path->dentry->d_inode;
	struct delegated_inode delegated_inode = { };
	struct iattr newattrs;
	int error;

	error = mnt_want_write(path->mnt);
	if (error)
		return error;
retry_deleg:
	error = inode_lock_killable(inode);
	if (error)
		goto out_mnt_unlock;
	error = security_path_chmod(path, mode);
	if (error)
		goto out_unlock;
	newattrs.ia_mode = (mode & S_IALLUGO) | (inode->i_mode & ~S_IALLUGO);
	newattrs.ia_valid = ATTR_MODE | ATTR_CTIME;
	error = notify_change(mnt_idmap(path->mnt), path->dentry,
			      &newattrs, &delegated_inode);
out_unlock:
	inode_unlock(inode);
	if (is_delegated(&delegated_inode)) {
		error = break_deleg_wait(&delegated_inode);
		if (!error)
			goto retry_deleg;
	}
out_mnt_unlock:
	mnt_drop_write(path->mnt);
	return error;
}

int vfs_fchmod(struct file *file, umode_t mode)
{
	audit_file(file);
	return chmod_common(&file->f_path, mode);
}

SYSCALL_DEFINE2(fchmod, unsigned int, fd, umode_t, mode)
{
	CLASS(fd, f)(fd);

	if (fd_empty(f))
		return -EBADF;

	return vfs_fchmod(fd_file(f), mode);
}

static int do_fchmodat(int dfd, const char __user *filename, umode_t mode,
		       unsigned int flags)
{
	struct path path;
	int error;
	unsigned int lookup_flags;

	if (unlikely(flags & ~(AT_SYMLINK_NOFOLLOW | AT_EMPTY_PATH)))
		return -EINVAL;

	lookup_flags = (flags & AT_SYMLINK_NOFOLLOW) ? 0 : LOOKUP_FOLLOW;
	CLASS(filename_uflags, name)(filename, flags);
retry:
	error = filename_lookup(dfd, name, lookup_flags, &path, NULL);
	if (!error) {
		error = chmod_common(&path, mode);
		path_put(&path);
		if (retry_estale(error, lookup_flags)) {
			lookup_flags |= LOOKUP_REVAL;
			goto retry;
		}
	}
	return error;
}

SYSCALL_DEFINE4(fchmodat2, int, dfd, const char __user *, filename,
		umode_t, mode, unsigned int, flags)
{
	return do_fchmodat(dfd, filename, mode, flags);
}

SYSCALL_DEFINE3(fchmodat, int, dfd, const char __user *, filename,
		umode_t, mode)
{
	return do_fchmodat(dfd, filename, mode, 0);
}

SYSCALL_DEFINE2(chmod, const char __user *, filename, umode_t, mode)
{
	return do_fchmodat(AT_FDCWD, filename, mode, 0);
}

/*
 * Check whether @kuid is valid and if so generate and set vfsuid_t in
 * ia_vfsuid.
 *
 * Return: true if @kuid is valid, false if not.
 */
static inline bool setattr_vfsuid(struct iattr *attr, kuid_t kuid)
{
	if (!uid_valid(kuid))
		return false;
	attr->ia_valid |= ATTR_UID;
	attr->ia_vfsuid = VFSUIDT_INIT(kuid);
	return true;
}

/*
 * Check whether @kgid is valid and if so generate and set vfsgid_t in
 * ia_vfsgid.
 *
 * Return: true if @kgid is valid, false if not.
 */
static inline bool setattr_vfsgid(struct iattr *attr, kgid_t kgid)
{
	if (!gid_valid(kgid))
		return false;
	attr->ia_valid |= ATTR_GID;
	attr->ia_vfsgid = VFSGIDT_INIT(kgid);
	return true;
}

int chown_common(const struct path *path, uid_t user, gid_t group)
{
	const struct mnt_idmap *idmap;
	struct user_namespace *fs_userns;
	struct inode *inode = path->dentry->d_inode;
	struct delegated_inode delegated_inode = { };
	int error;
	struct iattr newattrs;
	kuid_t uid;
	kgid_t gid;

	uid = make_kuid(current_user_ns(), user);
	gid = make_kgid(current_user_ns(), group);

	idmap = mnt_idmap(path->mnt);
	fs_userns = i_user_ns(inode);

retry_deleg:
	newattrs.ia_vfsuid = INVALID_VFSUID;
	newattrs.ia_vfsgid = INVALID_VFSGID;
	newattrs.ia_valid =  ATTR_CTIME;
	if ((user != (uid_t)-1) && !setattr_vfsuid(&newattrs, uid))
		return -EINVAL;
	if ((group != (gid_t)-1) && !setattr_vfsgid(&newattrs, gid))
		return -EINVAL;
	error = inode_lock_killable(inode);
	if (error)
		return error;
	if (!S_ISDIR(inode->i_mode))
		newattrs.ia_valid |= ATTR_KILL_SUID | ATTR_KILL_PRIV |
				     setattr_should_drop_sgid(idmap, inode);
	/* Continue to send actual fs values, not the mount values. */
	error = security_path_chown(
		path,
		from_vfsuid(idmap, fs_userns, newattrs.ia_vfsuid),
		from_vfsgid(idmap, fs_userns, newattrs.ia_vfsgid));
	if (!error)
		error = notify_change(idmap, path->dentry, &newattrs,
				      &delegated_inode);
	inode_unlock(inode);
	if (is_delegated(&delegated_inode)) {
		error = break_deleg_wait(&delegated_inode);
		if (!error)
			goto retry_deleg;
	}
	return error;
}

int do_fchownat(int dfd, const char __user *filename, uid_t user, gid_t group,
		int flag)
{
	struct path path;
	int error;
	int lookup_flags;

	if ((flag & ~(AT_SYMLINK_NOFOLLOW | AT_EMPTY_PATH)) != 0)
		return -EINVAL;

	lookup_flags = (flag & AT_SYMLINK_NOFOLLOW) ? 0 : LOOKUP_FOLLOW;
	CLASS(filename_uflags, name)(filename, flag);
retry:
	error = filename_lookup(dfd, name, lookup_flags, &path, NULL);
	if (!error) {
		error = mnt_want_write(path.mnt);
		if (!error) {
			error = chown_common(&path, user, group);
			mnt_drop_write(path.mnt);
		}
		path_put(&path);
		if (retry_estale(error, lookup_flags)) {
			lookup_flags |= LOOKUP_REVAL;
			goto retry;
		}
	}
	return error;
}

SYSCALL_DEFINE5(fchownat, int, dfd, const char __user *, filename, uid_t, user,
		gid_t, group, int, flag)
{
	return do_fchownat(dfd, filename, user, group, flag);
}

SYSCALL_DEFINE3(chown, const char __user *, filename, uid_t, user, gid_t, group)
{
	return do_fchownat(AT_FDCWD, filename, user, group, 0);
}

SYSCALL_DEFINE3(lchown, const char __user *, filename, uid_t, user, gid_t, group)
{
	return do_fchownat(AT_FDCWD, filename, user, group,
			   AT_SYMLINK_NOFOLLOW);
}

int vfs_fchown(struct file *file, uid_t user, gid_t group)
{
	int error;

	error = mnt_want_write_file(file);
	if (error)
		return error;
	audit_file(file);
	error = chown_common(&file->f_path, user, group);
	mnt_drop_write_file(file);
	return error;
}

int ksys_fchown(unsigned int fd, uid_t user, gid_t group)
{
	CLASS(fd, f)(fd);

	if (fd_empty(f))
		return -EBADF;

	return vfs_fchown(fd_file(f), user, group);
}

SYSCALL_DEFINE3(fchown, unsigned int, fd, uid_t, user, gid_t, group)
{
	return ksys_fchown(fd, user, group);
}

static inline int file_get_write_access(struct file *f)
{
	int error;

	error = get_write_access(f->f_inode);
	if (unlikely(error))
		return error;
	error = mnt_get_write_access(f->f_path.mnt);
	if (unlikely(error))
		goto cleanup_inode;
	if (unlikely(f->f_mode & FMODE_BACKING)) {
		error = mnt_get_write_access(backing_file_user_path(f)->mnt);
		if (unlikely(error))
			goto cleanup_mnt;
	}
	return 0;

cleanup_mnt:
	mnt_put_write_access(f->f_path.mnt);
cleanup_inode:
	put_write_access(f->f_inode);
	return error;
}

/*
 * Populate struct file
 *
 * NOTE: it assumes f_path is populated and consumes the caller's reference.
 */
static int do_dentry_open(struct file *f,
			  int (*open)(struct inode *, struct file *))
{
	static const struct file_operations empty_fops = {};
	struct inode *inode = f->f_path.dentry->d_inode;
	int error;

	f->f_inode = inode;
	f->f_mapping = inode->i_mapping;
	f->f_wb_err = filemap_sample_wb_err(f->f_mapping);
	f->f_sb_err = file_sample_sb_err(f);

	if (unlikely(f->f_flags & O_PATH)) {
		f->f_mode = FMODE_PATH | FMODE_OPENED;
		file_set_fsnotify_mode(f, FMODE_NONOTIFY);
		f->f_op = &empty_fops;
		return 0;
	}

	if ((f->f_mode & (FMODE_READ | FMODE_WRITE)) == FMODE_READ) {
		i_readcount_inc(inode);
	} else if (f->f_mode & FMODE_WRITE && !special_file(inode->i_mode)) {
		error = file_get_write_access(f);
		if (unlikely(error))
			goto cleanup_file;
		f->f_mode |= FMODE_WRITER;
	}

	/* POSIX.1-2008/SUSv4 Section XSI 2.9.7 */
	if (S_ISREG(inode->i_mode) || S_ISDIR(inode->i_mode))
		f->f_mode |= FMODE_ATOMIC_POS;

	f->f_op = fops_get(inode->i_fop);
	if (WARN_ON(!f->f_op)) {
		error = -ENODEV;
		goto cleanup_all;
	}

	error = security_file_open(f);
	if (unlikely(error))
		goto cleanup_all;

	/*
	 * Call fsnotify open permission hook and set FMODE_NONOTIFY_* bits
	 * according to existing permission watches.
	 * If FMODE_NONOTIFY mode was already set for an fanotify fd or for a
	 * pseudo file, this call will not change the mode.
	 */
	error = fsnotify_open_perm_and_set_mode(f);
	if (unlikely(error))
		goto cleanup_all;

	error = break_lease(file_inode(f), f->f_flags);
	if (unlikely(error))
		goto cleanup_all;

	/* normally all 3 are set; ->open() can clear them if needed */
	f->f_mode |= FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE;
	if (!open)
		open = f->f_op->open;
	if (open) {
		error = open(inode, f);
		if (error)
			goto cleanup_all;
	}
	f->f_mode |= FMODE_OPENED;
	if ((f->f_mode & FMODE_READ) &&
	     likely(f->f_op->read || f->f_op->read_iter))
		f->f_mode |= FMODE_CAN_READ;
	if ((f->f_mode & FMODE_WRITE) &&
	     likely(f->f_op->write || f->f_op->write_iter))
		f->f_mode |= FMODE_CAN_WRITE;
	if ((f->f_mode & FMODE_LSEEK) && !f->f_op->llseek)
		f->f_mode &= ~FMODE_LSEEK;
	if (f->f_mapping->a_ops && f->f_mapping->a_ops->direct_IO)
		f->f_mode |= FMODE_CAN_ODIRECT;

	f->f_flags &= ~(O_CREAT | O_EXCL | O_NOCTTY | O_TRUNC | __O_REGULAR);
	f->f_iocb_flags = iocb_flags(f);

	file_ra_state_init(&f->f_ra, f->f_mapping->host->i_mapping);

	if ((f->f_flags & O_DIRECT) && !(f->f_mode & FMODE_CAN_ODIRECT))
		return -EINVAL;

	return 0;

cleanup_all:
	if (WARN_ON_ONCE(error > 0))
		error = -EINVAL;
	fops_put(f->f_op);
	put_file_access(f);
cleanup_file:
	path_put(&f->f_path);
	f->__f_path.mnt = NULL;
	f->__f_path.dentry = NULL;
	f->f_inode = NULL;
	return error;
}

/**
 * finish_open - finish opening a file
 * @file: file pointer
 * @dentry: pointer to dentry
 * @open: open callback
 *
 * This can be used to finish opening a file passed to i_op->atomic_open().
 *
 * If the open callback is set to NULL, then the standard f_op->open()
 * filesystem callback is substituted.
 *
 * NB: the dentry reference is _not_ consumed.  If, for example, the dentry is
 * the return value of d_splice_alias(), then the caller needs to perform dput()
 * on it after finish_open().
 *
 * Returns zero on success or -errno if the open failed.
 */
int finish_open(struct file *file, struct dentry *dentry,
		int (*open)(struct inode *, struct file *))
{
	BUG_ON(file->f_mode & FMODE_OPENED); /* once it's opened, it's opened */

	file->__f_path.dentry = dentry;
	path_get(&file->f_path);
	return do_dentry_open(file, open);
}
EXPORT_SYMBOL(finish_open);

/**
 * finish_no_open - finish ->atomic_open() without opening the file
 *
 * @file: file pointer
 * @dentry: dentry, ERR_PTR(-E...) or NULL (as returned from ->lookup())
 *
 * This can be used to set the result of a lookup in ->atomic_open().
 *
 * NB: unlike finish_open() this function does consume the dentry reference and
 * the caller need not dput() it.
 *
 * Returns 0 or -E..., which must be the return value of ->atomic_open() after
 * having called this function.
 */
int finish_no_open(struct file *file, struct dentry *dentry)
{
	if (IS_ERR(dentry))
		return PTR_ERR(dentry);
	file->__f_path.dentry = dentry;
	return 0;
}
EXPORT_SYMBOL(finish_no_open);

char *file_path(struct file *filp, char *buf, int buflen)
{
	return d_path(&filp->f_path, buf, buflen);
}
EXPORT_SYMBOL(file_path);

/**
 * vfs_open - open the file at the given path
 * @path: path to open
 * @file: newly allocated file with f_flag initialized
 */
int vfs_open(const struct path *path, struct file *file)
{
	int ret;

	file->__f_path = *path;
	path_get(&file->f_path);
	ret = do_dentry_open(file, NULL);
	if (!ret) {
		/*
		 * Once we return a file with FMODE_OPENED, __fput() will call
		 * fsnotify_close(), so we need fsnotify_open() here for
		 * symmetry.
		 */
		fsnotify_open(file);
	}
	return ret;
}

/**
 * vfs_open_consume - open the file at the given path and consume the reference
 * @path: path to open
 * @file: newly allocated file with f_flag initialized
 */
int vfs_open_consume(struct path *path, struct file *file)
{
	int ret;

	file->__f_path = *path;
	path->mnt = NULL;
	path->dentry = NULL;
	ret = do_dentry_open(file, NULL);
	if (!ret) {
		fsnotify_open(file);
	}
	return ret;
}

struct file *dentry_open(const struct path *path, int flags,
			 const struct cred *cred)
{
	int error;
	struct file *f;

	/* We must always pass in a valid mount pointer. */
	BUG_ON(!path->mnt);

	f = alloc_empty_file(flags, cred);
	if (!IS_ERR(f)) {
		error = vfs_open(path, f);
		if (error) {
			fput(f);
			f = ERR_PTR(error);
		}
	}
	return f;
}
EXPORT_SYMBOL(dentry_open);

struct file *dentry_open_nonotify(const struct path *path, int flags,
				  const struct cred *cred)
{
	struct file *f = alloc_empty_file(flags, cred);
	if (!IS_ERR(f)) {
		int error;

		file_set_fsnotify_mode(f, FMODE_NONOTIFY);
		error = vfs_open(path, f);
		if (error) {
			fput(f);
			f = ERR_PTR(error);
		}
	}
	return f;
}

/**
 * kernel_file_open - open a file for kernel internal use
 * @path:	path of the file to open
 * @flags:	open flags
 * @cred:	credentials for open
 *
 * Open a file for use by in-kernel consumers. The file is not accounted
 * against nr_files and must not be installed into the file descriptor
 * table.
 *
 * Return: Opened file on success, an error pointer on failure.
 */
struct file *kernel_file_open(const struct path *path, int flags,
				const struct cred *cred)
{
	struct file *f;
	int error;

	f = alloc_empty_file_noaccount(flags, cred);
	if (IS_ERR(f))
		return f;

	error = vfs_open(path, f);
	if (error) {
		fput(f);
		return ERR_PTR(error);
	}
	return f;
}
EXPORT_SYMBOL_GPL(kernel_file_open);

#define WILL_CREATE(flags)	(flags & (O_CREAT | __O_TMPFILE))
#define O_PATH_FLAGS		(O_DIRECTORY | O_NOFOLLOW | O_PATH | O_CLOEXEC | O_EMPTYPATH)

inline struct open_how build_open_how(int flags, umode_t mode)
{
	struct open_how how = {
		.flags = flags & VALID_OPEN_FLAGS,
		.mode = mode & S_IALLUGO,
	};

	/* O_PATH beats everything else. */
	if (how.flags & O_PATH)
		how.flags &= O_PATH_FLAGS;
	/* Modes should only be set for create-like flags. */
	if (!WILL_CREATE(how.flags))
		how.mode = 0;
	return how;
}

inline int build_open_flags(const struct open_how *how, struct open_flags *op)
{
	u64 flags = how->flags;
	u64 strip = O_CLOEXEC;
	int lookup_flags = 0;
	int acc_mode = ACC_MODE(flags);

	BUILD_BUG_ON_MSG(upper_32_bits(VALID_OPEN_FLAGS),
			 "VALID_OPEN_FLAGS must fit in 32 bits");
	/* The whole point: OPENAT2_REGULAR must be unrepresentable in int. */
	BUILD_BUG_ON_MSG(!upper_32_bits(OPENAT2_REGULAR),
			 "OPENAT2_REGULAR must live in the upper 32 bits of open_how::flags");
	/* Prevent a future bit collision between UAPI and internal carrier. */
	BUILD_BUG_ON_MSG(OPENAT2_REGULAR & VALID_OPEN_FLAGS,
			 "OPENAT2_REGULAR must not alias any open()/openat() flag");
	BUILD_BUG_ON_MSG(__O_REGULAR & VALID_OPENAT2_FLAGS,
			 "__O_REGULAR must not alias any user-visible flag");

	/*
	 * Strip flags that aren't relevant in determining struct open_flags.
	 */
	flags &= ~strip;

	/*
	 * Older syscalls implicitly clear all of the invalid flags or argument
	 * values before calling build_open_flags(), but openat2(2) checks all
	 * of its arguments.
	 */
	if (flags & ~VALID_OPENAT2_FLAGS)
		return -EINVAL;
	if (how->resolve & ~VALID_RESOLVE_FLAGS)
		return -EINVAL;

	/* Scoping flags are mutually exclusive. */
	if ((how->resolve & RESOLVE_BENEATH) && (how->resolve & RESOLVE_IN_ROOT))
		return -EINVAL;

	/* Deal with the mode. */
	if (WILL_CREATE(flags)) {
		if (how->mode & ~S_IALLUGO)
			return -EINVAL;
		op->mode = how->mode | S_IFREG;
	} else {
		if (how->mode != 0)
			return -EINVAL;
		op->mode = 0;
	}

	/*
	 * Block bugs where O_DIRECTORY | O_CREAT created regular files.
	 * Note, that blocking O_DIRECTORY | O_CREAT here also protects
	 * O_TMPFILE below which requires O_DIRECTORY being raised.
	 */
	if ((flags & (O_DIRECTORY | O_CREAT)) == (O_DIRECTORY | O_CREAT))
		return -EINVAL;

	/* Now handle the creative implementation of O_TMPFILE. */
	if (flags & __O_TMPFILE) {
		/*
		 * In order to ensure programs get explicit errors when trying
		 * to use O_TMPFILE on old kernels we enforce that O_DIRECTORY
		 * is raised alongside __O_TMPFILE.
		 */
		if (!(flags & O_DIRECTORY))
			return -EINVAL;
		if (!(acc_mode & MAY_WRITE))
			return -EINVAL;
	}
	/*
	 * Asking to open a directory and a regular file at the same time is
	 * contradictory.
	 */
	if ((flags & (O_DIRECTORY | OPENAT2_REGULAR)) ==
	    (O_DIRECTORY | OPENAT2_REGULAR))
		return -EINVAL;

	if (flags & O_PATH) {
		/* O_PATH only permits certain other flags to be set. */
		if (flags & ~O_PATH_FLAGS)
			return -EINVAL;
		acc_mode = 0;
	}

	/*
	 * O_SYNC is implemented as __O_SYNC|O_DSYNC.  As many places only
	 * check for O_DSYNC if the need any syncing at all we enforce it's
	 * always set instead of having to deal with possibly weird behaviour
	 * for malicious applications setting only __O_SYNC.
	 */
	if (flags & __O_SYNC)
		flags |= O_DSYNC;

	/*
	 * Translate the upper-32-bit UAPI bit OPENAT2_REGULAR into the
	 * kernel-internal lower-32-bit __O_REGULAR carrier so the bit
	 * survives the assignment to op->open_flag (an int) below and the
	 * subsequent flow through f->f_flags (unsigned int) and the
	 * i_op->atomic_open() callback (unsigned). do_dentry_open() strips
	 * __O_REGULAR before the file becomes visible to userspace.
	 */
	if (flags & OPENAT2_REGULAR) {
		flags &= ~OPENAT2_REGULAR;
		flags |= __O_REGULAR;
	}

	op->open_flag = flags;

	/* O_TRUNC implies we need access checks for write permissions */
	if (flags & O_TRUNC)
		acc_mode |= MAY_WRITE;

	/* Allow the LSM permission hook to distinguish append
	   access from general write access. */
	if (flags & O_APPEND)
		acc_mode |= MAY_APPEND;

	op->acc_mode = acc_mode;

	op->intent = flags & O_PATH ? 0 : LOOKUP_OPEN;

	if (flags & O_CREAT) {
		op->intent |= LOOKUP_CREATE;
		if (flags & O_EXCL) {
			op->intent |= LOOKUP_EXCL;
			flags |= O_NOFOLLOW;
		}
	}

	if (flags & O_DIRECTORY)
		lookup_flags |= LOOKUP_DIRECTORY;
	if (!(flags & O_NOFOLLOW))
		lookup_flags |= LOOKUP_FOLLOW;
	if (flags & O_EMPTYPATH)
		lookup_flags |= LOOKUP_EMPTY;

	if (how->resolve & RESOLVE_NO_XDEV)
		lookup_flags |= LOOKUP_NO_XDEV;
	if (how->resolve & RESOLVE_NO_MAGICLINKS)
		lookup_flags |= LOOKUP_NO_MAGICLINKS;
	if (how->resolve & RESOLVE_NO_SYMLINKS)
		lookup_flags |= LOOKUP_NO_SYMLINKS;
	if (how->resolve & RESOLVE_BENEATH)
		lookup_flags |= LOOKUP_BENEATH;
	if (how->resolve & RESOLVE_IN_ROOT)
		lookup_flags |= LOOKUP_IN_ROOT;
	if (how->resolve & RESOLVE_CACHED) {
		/* Don't bother even trying for create/truncate/tmpfile open */
		if (flags & (O_TRUNC | O_CREAT | __O_TMPFILE))
			return -EAGAIN;
		lookup_flags |= LOOKUP_CACHED;
	}

	op->lookup_flags = lookup_flags;
	return 0;
}

/**
 * file_open_name - open file and return file pointer
 *
 * @name:	struct filename containing path to open
 * @flags:	open flags as per the open(2) second argument
 * @mode:	mode for the new file if O_CREAT is set, else ignored
 *
 * This is the helper to open a file from kernelspace if you really
 * have to.  But in generally you should not do this, so please move
 * along, nothing to see here..
 */
struct file *file_open_name(struct filename *name, int flags, umode_t mode)
{
	struct open_flags op;
	struct open_how how = build_open_how(flags, mode);
	int err = build_open_flags(&how, &op);
	if (err)
		return ERR_PTR(err);
	return do_file_open(AT_FDCWD, name, &op);
}

/**
 * filp_open - open file and return file pointer
 *
 * @filename:	path to open
 * @flags:	open flags as per the open(2) second argument
 * @mode:	mode for the new file if O_CREAT is set, else ignored
 *
 * This is the helper to open a file from kernelspace if you really
 * have to.  But in generally you should not do this, so please move
 * along, nothing to see here..
 */
struct file *filp_open(const char *filename, int flags, umode_t mode)
{
	CLASS(filename_kernel, name)(filename);
	return file_open_name(name, flags, mode);
}
EXPORT_SYMBOL(filp_open);

struct file *file_open_root(const struct path *root,
			    const char *filename, int flags, umode_t mode)
{
	struct open_flags op;
	struct open_how how = build_open_how(flags, mode);
	int err = build_open_flags(&how, &op);
	if (err)
		return ERR_PTR(err);
	return do_file_open_root(root, filename, &op);
}
EXPORT_SYMBOL(file_open_root);

static int do_sys_openat2(int dfd, const char __user *filename,
			  struct open_how *how)
{
	struct open_flags op;
	int err = build_open_flags(how, &op);
	if (unlikely(err))
		return err;

	CLASS(filename_flags, name)(filename, op.lookup_flags);
	return FD_ADD(how->flags, do_file_open(dfd, name, &op));
}

int do_sys_open(int dfd, const char __user *filename, int flags, umode_t mode)
{
	struct open_how how = build_open_how(flags, mode);
	return do_sys_openat2(dfd, filename, &how);
}


/**
 * sys_open - Open or create a file
 * @filename: Pathname of the file to open or create
 * @flags: File access mode and behavior flags (O_RDONLY, O_WRONLY, O_RDWR, etc.)
 * @mode: File permission bits for newly created files (only with O_CREAT/O_TMPFILE)
 *
 * long-desc: Opens the file named by filename, relative to the current
 *   working directory if the path is relative. With O_CREAT, the file is
 *   created if it does not exist. With O_TMPFILE, filename must name an
 *   existing directory, in which an unnamed file is created. A new file gets
 *   mode & ~umask as its permission bits.
 *
 *   The low two bits of flags (O_ACCMODE) select the access mode: O_RDONLY,
 *   O_WRONLY or O_RDWR. File creation and file status flags are ORed in.
 *
 *   File creation flags: O_CREAT, O_EXCL, O_NOCTTY, O_TRUNC, O_DIRECTORY,
 *   O_NOFOLLOW, O_CLOEXEC, O_TMPFILE, O_EMPTYPATH. O_EMPTYPATH permits an
 *   empty filename, which for open() refers to the current working directory.
 *
 *   File status flags: O_APPEND, FASYNC, O_DIRECT, O_DSYNC, O_LARGEFILE,
 *   O_NOATIME, O_NONBLOCK (O_NDELAY), O_PATH, O_SYNC. These become part of the
 *   file's open file description and can be retrieved with fcntl(F_GETFL). Only
 *   O_APPEND, O_NONBLOCK, O_DIRECT, O_NOATIME and FASYNC can be changed later
 *   with fcntl(F_SETFL).
 *
 *   On success the lowest-numbered file descriptor not currently open in the
 *   process is returned.
 *
 *   On 64-bit systems, O_LARGEFILE is automatically added to the flags. On 32-bit
 *   systems, files larger than 2GB require O_LARGEFILE to be explicitly set.
 *
 *   open() is equivalent to openat(AT_FDCWD, filename, flags, mode).
 *
 * contexts: process, sleepable
 *
 * param: filename
 *   type: path, input
 *   constraint-type: user_path
 *   cdesc: Must be a valid null-terminated path string in user memory.
 *     Maximum path length is PATH_MAX (4096 bytes) including null terminator.
 *     For relative paths, resolution starts from current working directory.
 *     The path is followed (symlinks resolved) unless O_NOFOLLOW is specified.
 *
 * param: flags
 *   type: int, input
 *   constraint-type: mask(O_RDONLY | O_WRONLY | O_RDWR | O_CREAT | O_EXCL | O_NOCTTY |
 *                         O_TRUNC | O_APPEND | O_NONBLOCK | O_NDELAY | O_DSYNC | O_SYNC |
 *                         FASYNC | O_DIRECT | O_LARGEFILE | O_DIRECTORY | O_NOFOLLOW |
 *                         O_NOATIME | O_CLOEXEC | O_PATH | O_TMPFILE | O_EMPTYPATH)
 *   cdesc: Should be one of O_RDONLY (0), O_WRONLY (1), or O_RDWR (2) as the
 *     access mode. Additional flags may be ORed. O_CREAT combined with
 *     O_DIRECTORY or O_TMPFILE, O_TMPFILE without O_DIRECTORY, and O_TMPFILE
 *     with read-only mode return EINVAL. With O_PATH, open() silently drops
 *     every other flag except O_DIRECTORY, O_NOFOLLOW, O_CLOEXEC and
 *     O_EMPTYPATH (only openat2() rejects them with EINVAL). Unknown flags are
 *     silently ignored for backward compatibility (unlike openat2 which
 *     rejects them).
 *
 * param: mode
 *   type: uint, input
 *   cdesc: Only meaningful when O_CREAT or O_TMPFILE is specified in
 *     flags. Specifies the file mode bits (permissions and setuid/setgid/sticky
 *     bits) for a newly created file. The effective mode is (mode & ~umask).
 *     When O_CREAT/O_TMPFILE is not set, mode is ignored. Mode values exceeding
 *     S_IALLUGO (07777) are masked off.
 *
 * return:
 *   type: int
 *   check-type: fd
 *   success: >= 0
 *   desc: On success, returns a new file descriptor (non-negative integer).
 *     The returned file descriptor is the lowest-numbered descriptor not
 *     currently open for the process. On error, returns a negative error code.
 *
 * error: EACCES, Permission denied
 *   desc: The requested access to the file is not allowed, or search permission
 *     is denied for one of the directories in the path prefix of pathname, or
 *     the file did not exist yet and write access to the parent directory is
 *     not allowed, or O_TRUNC is specified but write permission is denied, or
 *     pathname is a device special file on a nodev mount, or O_CREAT on an
 *     existing FIFO or regular file in a sticky directory is refused by
 *     protected_fifos or protected_regular, or a security module denies the
 *     open.
 *
 * error: EAGAIN, Resource temporarily unavailable
 *   desc: O_NONBLOCK was specified and a conflicting lease is held on the file,
 *     so the open would have to wait for the lease to break. break_lease()
 *     returns -EWOULDBLOCK, which has the same value as EAGAIN.
 *
 * error: EBUSY, Device or resource busy
 *   desc: O_EXCL was specified in flags and pathname refers to a block device
 *     that is in use by the system (e.g., it is mounted).
 *
 * error: EDQUOT, Disk quota exceeded
 *   desc: O_CREAT is specified and the file does not exist, and the user's quota
 *     of disk blocks or inodes on the filesystem has been exhausted.
 *
 * error: EEXIST, File exists
 *   desc: O_CREAT and O_EXCL were specified in flags, but pathname already exists.
 *     This error is atomic with respect to file creation - it prevents race
 *     conditions (TOCTOU) when creating files.
 *
 * error: EFAULT, Bad address
 *   desc: pathname points outside the process's accessible address space.
 *
 * error: EINTR, Interrupted system call
 *   desc: A signal arrived while the open was blocked waiting for the partner
 *     of a FIFO open (fifo_open), waiting for a conflicting lease to break
 *     (__break_lease), or inside a driver's open method. The kernel-internal
 *     -ERESTARTSYS is reported as EINTR unless the handler uses SA_RESTART.
 *
 * error: EINVAL, Invalid argument
 *   desc: Returned for several conditions: (1) Invalid O_* flag combinations
 *     (O_CREAT with O_DIRECTORY, O_CREAT with O_TMPFILE, O_TMPFILE without
 *     O_DIRECTORY, O_TMPFILE with read-only access). (2) O_DIRECT requested
 *     but the filesystem does not support it.
 *
 * error: EISDIR, Is a directory
 *   desc: pathname refers to a directory and the access requested involved
 *     writing (O_WRONLY, O_RDWR, or O_TRUNC). Also returned when O_CREAT is
 *     specified and pathname names an existing directory or ends in a slash.
 *
 * error: ELOOP, Too many symbolic links
 *   desc: Too many symbolic links were encountered in resolving pathname, or
 *     O_NOFOLLOW was specified but pathname refers to a symbolic link. With
 *     O_PATH and O_NOFOLLOW the symbolic link itself is opened instead.
 *
 * error: EMFILE, Too many open files
 *   desc: The per-process limit on the number of open file descriptors has been
 *     reached. This limit is RLIMIT_NOFILE (default typically 1024, max set by
 *     /proc/sys/fs/nr_open).
 *
 * error: ENAMETOOLONG, File name too long
 *   desc: pathname was too long, exceeding PATH_MAX (4096) bytes, or a single
 *     path component exceeded NAME_MAX (usually 255) bytes.
 *
 * error: ENFILE, Too many open files in system
 *   desc: The system-wide limit on the total number of open files has been
 *     reached (/proc/sys/fs/file-max). Processes with CAP_SYS_ADMIN can exceed
 *     this limit.
 *
 * error: ENODEV, No such device
 *   desc: The filesystem or driver open method failed with ENODEV, or the
 *     file's inode has no file operations assigned. A device special file with
 *     no registered device fails with ENXIO instead.
 *
 * error: ENOENT, No such file or directory
 *   desc: A directory component in pathname does not exist or is a dangling
 *     symbolic link, or O_CREAT is not set and the named file does not exist,
 *     or pathname is an empty string and O_EMPTYPATH is not specified.
 *
 * error: ENOMEM, Out of memory
 *   desc: The kernel could not allocate sufficient memory for the file structure,
 *     path lookup structures, or the filename buffer.
 *
 * error: ENOSPC, No space left on device
 *   desc: O_CREAT was specified and the file does not exist, and the directory
 *     or filesystem containing the file has no room for a new file entry.
 *
 * error: ENOTDIR, Not a directory
 *   desc: A component used as a directory in pathname is not actually a directory,
 *     or O_DIRECTORY was specified and pathname was not a directory.
 *
 * error: ENXIO, No such device or address
 *   desc: O_NONBLOCK | O_WRONLY is set and the named file is a FIFO and no
 *     process has the FIFO open for reading. Also returned when opening a device
 *     special file whose device does not exist (chrdev_open, blkdev_open), or
 *     when opening a socket inode.
 *
 * error: EOPNOTSUPP, Operation not supported
 *   desc: The filesystem containing pathname does not support O_TMPFILE.
 *
 * error: EOVERFLOW, Value too large for defined data type
 *   desc: pathname refers to a regular file that is too large to be opened.
 *     This occurs on 32-bit systems without O_LARGEFILE when the file size
 *     exceeds 2GB (2^31 - 1 bytes).
 *
 * error: EPERM, Operation not permitted
 *   desc: O_NOATIME flag was specified but the effective UID of the caller did
 *     not match the owner of the file and the caller is not privileged, or the
 *     file is append-only and O_TRUNC was specified or write mode without
 *     O_APPEND, or the file is immutable, or a seal prevents the operation.
 *
 * error: EROFS, Read-only file system
 *   desc: pathname refers to a file on a read-only filesystem and write access
 *     was requested.
 *
 * error: ETXTBSY, Text file busy
 *   desc: Write access or O_TRUNC was requested for an executable image that
 *     is currently being executed, or O_TRUNC was requested on an active swap
 *     file. A swap file can otherwise be opened for writing.
 *
 * lock: files->file_lock
 *   type: spinlock
 *   acquired: true
 *   released: true
 *   desc: Acquired when allocating a file descriptor slot. Held briefly during
 *     fd allocation via alloc_fd() and released before the syscall returns.
 *
 * lock: inode->i_rwsem (parent directory)
 *   type: semaphore
 *   acquired: true
 *   released: true
 *   desc: Conditional, taken only when the final component is not resolved by
 *     the lockless dcache lookup. lookup_open() takes it exclusively with
 *     inode_lock() when O_CREAT is set and shared with inode_lock_shared()
 *     otherwise. Slow-path lookup of path components takes it shared. Released
 *     when the lookup returns. The open path has no killable variant.
 *
 * lock: RCU read-side
 *   type: rcu
 *   acquired: true
 *   released: true
 *   desc: Path lookup uses RCU mode initially for performance. If RCU lookup
 *     fails (returns -ECHILD), falls back to reference-based lookup.
 *
 * signal: Any signal
 *   direction: receive
 *   action: return
 *   condition: When blocked in an interruptible wait
 *   desc: The syscall may be interrupted while waiting for the partner of a
 *     FIFO open (fifo_open), for a conflicting lease to break (__break_lease),
 *     or inside a driver's open method. The wait returns -ERESTARTSYS, which
 *     is restarted after the handler with SA_RESTART and reported as EINTR
 *     otherwise.
 *   errno: -EINTR
 *   timing: during
 *   restartable: yes
 *
 * side-effect: resource_create | alloc_memory
 *   target: file descriptor, file structure, dentry cache
 *   desc: Allocates a new file descriptor in the process's fd table. Allocates
 *     a struct file from the filp slab cache. May allocate dentries and inodes
 *     during path lookup. System-wide file count (nr_files) is incremented.
 *   reversible: yes
 *
 * side-effect: filesystem
 *   target: filesystem, inode
 *   condition: When O_CREAT is specified and file doesn't exist
 *   desc: Creates a new file on the filesystem. Creates new inode, allocates
 *     data blocks as needed, and creates directory entry. Updates parent
 *     directory mtime and ctime.
 *   reversible: no
 *
 * side-effect: filesystem
 *   target: file content
 *   condition: When O_TRUNC is specified for existing file
 *   desc: Truncates the file to zero length, releasing data blocks. Updates
 *     file mtime and ctime. May trigger notifications to lease holders.
 *   reversible: no
 *
 * side-effect: modify_state
 *   target: inode timestamps
 *   condition: When a symlink is followed, or O_TRUNC or O_CREAT takes effect
 *   desc: Opening does not update the atime of the opened file. Reads update it
 *     later, and O_NOATIME only affects those reads. Following a symlink in
 *     the path may update the symlink's atime, subject to the mount atime
 *     options. O_TRUNC and file creation update mtime and ctime.
 *
 * capability: CAP_DAC_OVERRIDE
 *   type: bypass_check
 *   allows: Bypass file read, write, and execute permission checks
 *   without: Standard DAC (discretionary access control) checks are applied
 *   condition: Checked when file permission would otherwise deny access
 *
 * capability: CAP_DAC_READ_SEARCH
 *   type: bypass_check
 *   allows: Bypass read permission on files and search permission on directories
 *   without: Must have read permission on file or search permission on directory
 *   condition: Checked during path traversal and file open
 *
 * capability: CAP_FOWNER
 *   type: bypass_check
 *   allows: Use O_NOATIME on files not owned by caller
 *   without: O_NOATIME returns EPERM if caller is not file owner
 *   condition: Checked when O_NOATIME is specified and caller is not owner
 *
 * capability: CAP_SYS_ADMIN
 *   type: increase_limit
 *   allows: Exceed the system-wide file limit (file-max)
 *   without: Returns ENFILE when system limit is reached
 *   condition: Checked in alloc_empty_file() when nr_files >= max_files
 *
 * constraint: RLIMIT_NOFILE (per-process fd limit)
 *   desc: The returned file descriptor must be less than the process's
 *     RLIMIT_NOFILE limit. Default is typically 1024, maximum is controlled
 *     by /proc/sys/fs/nr_open (default 1048576). Exceeding returns EMFILE.
 *   expr: fd < rlimit(RLIMIT_NOFILE)
 *
 * constraint: file-max (system-wide limit)
 *   desc: System-wide limit on open files in /proc/sys/fs/file-max. Processes
 *     without CAP_SYS_ADMIN receive ENFILE when this limit is reached. The
 *     limit is computed based on system memory at boot time.
 *   expr: nr_files < files_stat.max_files || capable(CAP_SYS_ADMIN)
 *
 * constraint: PATH_MAX
 *   desc: Maximum length of pathname including null terminator is PATH_MAX
 *     (4096 bytes). Individual path components must not exceed NAME_MAX (255).
 *
 * examples: fd = open("/etc/passwd", O_RDONLY);  // Read existing file
 *   fd = open("/tmp/newfile", O_WRONLY | O_CREAT | O_TRUNC, 0644);  // Create/truncate
 *   fd = open("/tmp/lockfile", O_WRONLY | O_CREAT | O_EXCL, 0600);  // Exclusive create
 *   fd = open("/dev/null", O_RDWR);  // Open device
 *   fd = open("/tmp", O_RDONLY | O_DIRECTORY);  // Open directory
 *   fd = open("/tmp", O_TMPFILE | O_RDWR, 0600);  // Anonymous temp file
 *
 * notes: O_RDONLY is defined as 0, so (flags & O_RDONLY) always evaluates to zero.
 *   Test access mode using (flags & O_ACCMODE) == O_RDONLY.
 *
 *   When O_CREAT is specified without O_EXCL, there is a race condition between
 *   testing for file existence and creating it. Use O_CREAT | O_EXCL for atomic
 *   exclusive file creation.
 *
 *   O_CLOEXEC should be used in multithreaded programs to prevent file descriptor
 *   leaks to child processes between fork() and execve().
 *
 *   O_DIRECT has alignment requirements that vary by filesystem. Use statx()
 *   with STATX_DIOALIGN (Linux 6.1+) to query requirements. Unaligned I/O may
 *   fail with EINVAL or fall back to buffered I/O.
 *
 *   O_PATH opens a file descriptor that can be used only for certain operations
 *   (fstat, dup, fcntl, close, fchdir on directories, as dirfd for *at() calls).
 *   I/O operations will fail with EBADF.
 */
SYSCALL_DEFINE3(open, const char __user *, filename, int, flags, umode_t, mode)
{
	if (force_o_largefile())
		flags |= O_LARGEFILE;
	return do_sys_open(AT_FDCWD, filename, flags, mode);
}

SYSCALL_DEFINE4(openat, int, dfd, const char __user *, filename, int, flags,
		umode_t, mode)
{
	if (force_o_largefile())
		flags |= O_LARGEFILE;
	return do_sys_open(dfd, filename, flags, mode);
}

SYSCALL_DEFINE4(openat2, int, dfd, const char __user *, filename,
		struct open_how __user *, how, size_t, usize)
{
	int err;
	struct open_how tmp;

	BUILD_BUG_ON(sizeof(struct open_how) < OPEN_HOW_SIZE_VER0);
	BUILD_BUG_ON(sizeof(struct open_how) != OPEN_HOW_SIZE_LATEST);

	if (unlikely(usize < OPEN_HOW_SIZE_VER0))
		return -EINVAL;
	if (unlikely(usize > PAGE_SIZE))
		return -E2BIG;

	err = copy_struct_from_user(&tmp, sizeof(tmp), how, usize);
	if (err)
		return err;

	audit_openat2_how(&tmp);

	/* O_LARGEFILE is only allowed for non-O_PATH. */
	if (!(tmp.flags & O_PATH) && force_o_largefile())
		tmp.flags |= O_LARGEFILE;

	return do_sys_openat2(dfd, filename, &tmp);
}

#ifdef CONFIG_COMPAT
/*
 * Exactly like sys_open(), except that it doesn't set the
 * O_LARGEFILE flag.
 */
COMPAT_SYSCALL_DEFINE3(open, const char __user *, filename, int, flags, umode_t, mode)
{
	return do_sys_open(AT_FDCWD, filename, flags, mode);
}

/*
 * Exactly like sys_openat(), except that it doesn't set the
 * O_LARGEFILE flag.
 */
COMPAT_SYSCALL_DEFINE4(openat, int, dfd, const char __user *, filename, int, flags, umode_t, mode)
{
	return do_sys_open(dfd, filename, flags, mode);
}
#endif

#ifndef __alpha__

/*
 * For backward compatibility?  Maybe this should be moved
 * into arch/i386 instead?
 */
SYSCALL_DEFINE2(creat, const char __user *, pathname, umode_t, mode)
{
	int flags = O_CREAT | O_WRONLY | O_TRUNC;

	if (force_o_largefile())
		flags |= O_LARGEFILE;
	return do_sys_open(AT_FDCWD, pathname, flags, mode);
}
#endif

/*
 * "id" is the POSIX thread ID. We use the
 * files pointer for this..
 */
static int filp_flush(struct file *filp, fl_owner_t id)
{
	int retval = 0;

	if (CHECK_DATA_CORRUPTION(file_count(filp) == 0, filp,
			"VFS: Close: file count is 0 (f_op=%ps)",
			filp->f_op)) {
		return 0;
	}

	if (filp->f_op->flush)
		retval = filp->f_op->flush(filp, id);

	if (likely(!(filp->f_mode & FMODE_PATH))) {
		dnotify_flush(filp, id);
		locks_remove_posix(filp, id);
	}
	return retval;
}

int filp_close(struct file *filp, fl_owner_t id)
{
	int retval;

	retval = filp_flush(filp, id);
	fput_close(filp);

	return retval;
}
EXPORT_SYMBOL(filp_close);

/* Like filp_close() but the last reference is put right here. */
int filp_close_sync(struct file *filp, fl_owner_t id)
{
	int retval;

	/* Kernel threads must never put their final reference here. */
	VFS_WARN_ON_ONCE(current->flags & PF_KTHREAD);
	retval = filp_flush(filp, id);
	fput_close_sync(filp);

	return retval;
}

/**
 * sys_close - Close a file descriptor
 * @fd: The file descriptor to close
 *
 * long-desc: Terminates access to an open file descriptor, releasing the file
 *   descriptor for reuse by subsequent open(), dup(), or similar syscalls.
 *
 *   Traditional POSIX advisory record locks held by the process on the
 *   associated file are released when any of its fds for that inode is
 *   closed, not only the last one. OFD locks and flock locks are associated
 *   with the open file description and are only released when the last
 *   reference to that open file description is dropped.
 *
 *   Closing a file descriptor drops one reference to its open file
 *   description. Other descriptors (dup(), fork(), SCM_RIGHTS messages in
 *   flight) and operations still running on the file, such as a concurrent
 *   read(), also hold references. Only when the last reference is dropped are
 *   the associated resources freed. If the file was previously unlinked, the
 *   file itself is deleted when the last reference is dropped.
 *
 *   Except when close() fails with EBADF, the file descriptor is released even
 *   when close() returns an error, because it is released before the flush
 *   that may fail. POSIX leaves the state of the descriptor unspecified after
 *   EINTR. Retrying close() after an error may close an unrelated file
 *   descriptor that another thread has since been given.
 *
 *   Errors returned from close() come only from the file's ->flush() method,
 *   called by filp_flush(). Errors from ->release() are not reported.
 *   Filesystems without a ->flush() method, such as ext4, xfs and btrfs, never
 *   report write errors from close(). Network filesystems such as NFS, CIFS and
 *   FUSE implement ->flush() and report deferred write errors (EIO, ENOSPC,
 *   EDQUOT) at close time. A successful return does not mean the data reached
 *   storage; call fsync() before close() for that.
 *
 *   On close, the following cleanup operations are performed: the ->flush()
 *   method is called if the file has one, POSIX advisory locks are removed,
 *   dnotify registrations are cleaned up, and the file reference is released.
 *   If this was the last reference, additional cleanup includes: fsnotify close
 *   notification, epoll cleanup, OFD, flock and lease removal, FASYNC cleanup,
 *   the ->release() method, and the file structure deallocation.
 *
 * contexts: process, sleepable
 *
 * param: fd
 *   type: fd, input
 *   constraint-type: range(0, INT_MAX)
 *   cdesc: Must be a valid, open file descriptor for the current process.
 *     The value 0, 1, or 2 (stdin, stdout, stderr) may be closed like any other
 *     fd, though this is unusual and may cause issues with libraries that assume
 *     these descriptors are valid. The parameter is unsigned int to match kernel
 *     file descriptor table indexing. A value that is not open, including any
 *     value at or above the current table size, fails with EBADF.
 *
 * return:
 *   type: int
 *   check-type: exact
 *   success: 0
 *   desc: Returns 0 on success. On error, returns a negative error code. Except
 *     for EBADF, the file descriptor is still closed when an error is returned
 *     and must not be used again. The error comes from the file's ->flush()
 *     method, not from the fd remaining open. With EBADF, nothing was released.
 *
 * error: EBADF, Bad file descriptor
 *   desc: fd is at or above the file descriptor table size, has no file
 *     assigned (including a slot reserved by a concurrent open() that has not
 *     installed its file yet), or was already closed. This is the only error
 *     for which no file descriptor was released.
 *
 * error: EINTR, Interrupted system call
 *   desc: The flush operation was interrupted by a signal before completion.
 *     This occurs when a driver's ->flush() method (for example wdm_flush() in
 *     drivers/usb/class/cdc-wdm.c) performs an interruptible wait that receives
 *     a signal. The file descriptor is still released and must not be used
 *     again. Kernel-internal restart codes (ERESTARTSYS,
 *     ERESTARTNOINTR, ERESTARTNOHAND, ERESTART_RESTARTBLOCK) are converted to
 *     EINTR because restarting the syscall would be incorrect once the fd is
 *     freed.
 *
 * error: EIO, I/O error
 *   desc: The file's ->flush() method reported an I/O error, typically a
 *     deferred write error on a network filesystem such as NFS, CIFS or FUSE,
 *     or an error from a driver's ->flush(). Previously buffered write data
 *     may have been lost.
 *
 * error: ENOSPC, No space left on device
 *   desc: The file's ->flush() method reported that there was insufficient
 *     space to flush buffered writes, for example on NFS when the server runs
 *     out of space between write() and close().
 *
 * error: EDQUOT, Disk quota exceeded
 *   desc: The file's ->flush() method reported that the user's disk quota was
 *     exceeded while flushing buffered writes, for example on NFS when the
 *     quota is exceeded between write() and close().
 *
 * lock: files->file_lock
 *   type: spinlock
 *   acquired: true
 *   released: true
 *   desc: Taken by file_close_fd() to look up and clear the fd slot atomically,
 *     so two concurrent close() calls on one fd cannot both obtain the struct
 *     file. Dropped before ->flush() and the final fput. From then on the fd
 *     number may be handed out again. An fd reserved by a concurrent open() but
 *     not yet installed has a NULL slot, so close() returns EBADF.
 *
 * lock: file->f_lock
 *   type: spinlock
 *   acquired: true
 *   released: true
 *   desc: Taken from __fput() on the last reference, by eventpoll_release_file()
 *     when the file is registered with epoll, and by fasync_remove_entry()
 *     (through the ->fasync() method) when FASYNC is set. Protects the epoll
 *     and fasync links of the file.
 *
 * lock: ep->mtx
 *   type: mutex
 *   acquired: true
 *   released: true
 *   desc: Acquired during epoll cleanup if the file was monitored by epoll.
 *     Used to safely remove the file from epoll interest lists.
 *
 * lock: flc_lock
 *   type: spinlock
 *   acquired: true
 *   released: true
 *   desc: File lock context spinlock. Taken by locks_remove_posix() from
 *     filp_flush() on every close when the file has POSIX locks, and by
 *     locks_remove_file() from __fput() on the last reference to remove OFD,
 *     flock, and lease locks.
 *
 * signal: pending_signals
 *   direction: receive
 *   action: return
 *   condition: When close-time flush performs interruptible wait
 *   desc: If the close-time ->flush() method (for example wdm_flush() in
 *     cdc-wdm) performs an interruptible wait and a signal is pending, the wait
 *     is interrupted. Any kernel restart codes are converted to EINTR since
 *     close cannot be restarted after the fd is freed.
 *   errno: -EINTR
 *   timing: during
 *   restartable: no
 *
 * side-effect: resource_destroy | irreversible
 *   target: File descriptor table entry
 *   desc: The file descriptor is removed from the process's file descriptor
 *     table, making the fd number available for reuse by subsequent open(),
 *     dup(), or similar calls. This happens before the flush that may fail, so
 *     an error return does not undo it.
 *   condition: Always (when fd is valid)
 *   reversible: no
 *
 * side-effect: lock_release
 *   target: POSIX advisory locks, OFD locks, flock locks
 *   desc: POSIX locks held by this process on the inode are removed on every
 *     close via locks_remove_posix() in filp_flush(), except for O_PATH files.
 *     OFD and flock locks are removed via locks_remove_file() in __fput() only
 *     when this is the last reference to the open file description.
 *   condition: Any close of a non-O_PATH file for POSIX locks, last reference
 *     for OFD and flock locks
 *   reversible: no
 *
 * side-effect: resource_destroy
 *   target: File leases
 *   desc: Any file leases held on the file are removed during locks_remove_file()
 *     when this is the last reference to the open file description.
 *   condition: File had leases and this is the last reference
 *   reversible: no
 *
 * side-effect: modify_state
 *   target: dnotify registrations
 *   desc: Directory notification (dnotify) registrations associated with this
 *     file are cleaned up via dnotify_flush(). This only applies to directories.
 *   condition: File is a directory with dnotify registrations
 *   reversible: no
 *
 * side-effect: modify_state
 *   target: epoll interest lists
 *   desc: If the file was being monitored by epoll instances, it is removed
 *     from those interest lists via eventpoll_release(), which runs from
 *     __fput() only on the last reference. While other references remain, the
 *     epoll registrations stay active.
 *   condition: File was added to epoll instances and this is the last reference
 *   reversible: no
 *
 * side-effect: filesystem
 *   target: Buffered data
 *   desc: The file's ->flush() method runs before the file reference is
 *     dropped (for example on NFS, CIFS and FUSE) and may return errors such as
 *     EIO, ENOSPC or EDQUOT.
 *   condition: The filesystem or driver provides a ->flush() method
 *   reversible: no
 *
 * side-effect: free_memory
 *   target: struct file and related structures
 *   desc: When this is the last reference to the file, the file structure is
 *     freed and the dentry and mount references are released.
 *   condition: This is the last reference to the file
 *   reversible: no
 *
 * side-effect: filesystem
 *   target: Unlinked file deletion
 *   desc: If the file was previously unlinked (deleted) but kept open, closing
 *     the last reference causes the actual file data to be removed from the
 *     filesystem and the inode to be freed.
 *   condition: File was unlinked and this is the last reference
 *   reversible: no
 *
 * state-trans: file_descriptor
 *   from: open
 *   to: closed/free
 *   condition: Valid fd passed to close
 *   desc: The file descriptor transitions from open (usable) to closed (invalid).
 *     The fd number becomes available for reuse.
 *
 * state-trans: file_reference_count
 *   from: n
 *   to: n-1 (or freed if n was 1)
 *   condition: Always on successful fd lookup
 *   desc: The file's reference count is decremented. If this was the last
 *     reference, the file is fully cleaned up and freed.
 *
 * examples: close(fd);  // Ignoring the result loses ->flush() errors
 *   if (close(fd) == -1) perror("close");  // Log errors for debugging
 *   fsync(fd); close(fd);  // Ensure data persistence before closing
 *
 * notes: close() drops its reference with fput_close_sync(), so when it is the last
 *   reference __fput() runs synchronously in the calling task before close()
 *   returns, instead of being deferred to task work.
 *
 *   Calling close() on a file descriptor while another thread is using it
 *   (e.g., in a blocking read() or write()) does not interrupt the blocked
 *   operation. The blocked operation continues on the underlying file and
 *   may complete even after close() returns.
 */
/*
 * Careful here! We test whether the file pointer is NULL before
 * releasing the fd. This ensures that one clone task can't release
 * an fd while another clone is opening it.
 */
SYSCALL_DEFINE1(close, unsigned int, fd)
{
	int retval;
	struct file *file;

	file = file_close_fd(fd);
	if (!file)
		return -EBADF;

	/*
	 * We're returning to user space. Don't bother
	 * with any delayed fput() cases.
	 */
	retval = filp_close_sync(file, current->files);

	if (likely(retval == 0))
		return 0;

	/* can't restart close syscall because file table entry was cleared */
	if (retval == -ERESTARTSYS ||
	    retval == -ERESTARTNOINTR ||
	    retval == -ERESTARTNOHAND ||
	    retval == -ERESTART_RESTARTBLOCK)
		retval = -EINTR;

	return retval;
}

/*
 * This routine simulates a hangup on the tty, to arrange that users
 * are given clean terminals at login time.
 */
SYSCALL_DEFINE0(vhangup)
{
	if (capable(CAP_SYS_TTY_CONFIG)) {
		tty_vhangup_self();
		return 0;
	}
	return -EPERM;
}

/*
 * Called when an inode is about to be open.
 * We use this to disallow opening large files on 32bit systems if
 * the caller didn't specify O_LARGEFILE.  On 64bit systems we force
 * on this flag in sys_open.
 */
int generic_file_open(struct inode * inode, struct file * filp)
{
	if (!(filp->f_flags & O_LARGEFILE) && i_size_read(inode) > MAX_NON_LFS)
		return -EOVERFLOW;
	return 0;
}

EXPORT_SYMBOL(generic_file_open);

/*
 * This is used by subsystems that don't want seekable
 * file descriptors. The function is not supposed to ever fail, the only
 * reason it returns an 'int' and not 'void' is so that it can be plugged
 * directly into file_operations structure.
 */
int nonseekable_open(struct inode *inode, struct file *filp)
{
	filp->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

EXPORT_SYMBOL(nonseekable_open);

/*
 * stream_open is used by subsystems that want stream-like file descriptors.
 * Such file descriptors are not seekable and don't have notion of position
 * (file.f_pos is always 0 and ppos passed to .read()/.write() is always NULL).
 * Contrary to file descriptors of other regular files, .read() and .write()
 * can run simultaneously.
 *
 * stream_open never fails and is marked to return int so that it could be
 * directly used as file_operations.open .
 */
int stream_open(struct inode *inode, struct file *filp)
{
	filp->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE | FMODE_ATOMIC_POS);
	filp->f_mode |= FMODE_STREAM;
	return 0;
}

EXPORT_SYMBOL(stream_open);
