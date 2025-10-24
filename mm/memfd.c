/*
 * memfd_create system call and file sealing support
 *
 * Code was originally included in shmem.c, and broken out to facilitate
 * use by hugetlbfs as well as tmpfs.
 *
 * This file is released under the GPL.
 */

#include <linux/fs.h>
#include <linux/vfs.h>
#include <linux/pagemap.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/sched/signal.h>
#include <linux/khugepaged.h>
#include <linux/syscalls.h>
#include <linux/hugetlb.h>
#include <linux/shmem_fs.h>
#include <linux/memfd.h>
#include <linux/pid_namespace.h>
#include <uapi/linux/memfd.h>
#include "swap.h"

/*
 * We need a tag: a new tag would expand every xa_node by 8 bytes,
 * so reuse a tag which we firmly believe is never set or cleared on tmpfs
 * or hugetlbfs because they are memory only filesystems.
 */
#define MEMFD_TAG_PINNED        PAGECACHE_TAG_TOWRITE
#define LAST_SCAN               4       /* about 150ms max */

static bool memfd_folio_has_extra_refs(struct folio *folio)
{
	return folio_ref_count(folio) != folio_expected_ref_count(folio);
}

static void memfd_tag_pins(struct xa_state *xas)
{
	struct folio *folio;
	int latency = 0;

	lru_add_drain();

	xas_lock_irq(xas);
	xas_for_each(xas, folio, ULONG_MAX) {
		if (!xa_is_value(folio) && memfd_folio_has_extra_refs(folio))
			xas_set_mark(xas, MEMFD_TAG_PINNED);

		if (++latency < XA_CHECK_SCHED)
			continue;
		latency = 0;

		xas_pause(xas);
		xas_unlock_irq(xas);
		cond_resched();
		xas_lock_irq(xas);
	}
	xas_unlock_irq(xas);
}

/*
 * This is a helper function used by memfd_pin_user_pages() in GUP (gup.c).
 * It is mainly called to allocate a folio in a memfd when the caller
 * (memfd_pin_folios()) cannot find a folio in the page cache at a given
 * index in the mapping.
 */
struct folio *memfd_alloc_folio(struct file *memfd, pgoff_t idx)
{
#ifdef CONFIG_HUGETLB_PAGE
	struct folio *folio;
	gfp_t gfp_mask;

	if (is_file_hugepages(memfd)) {
		/*
		 * The folio would most likely be accessed by a DMA driver,
		 * therefore, we have zone memory constraints where we can
		 * alloc from. Also, the folio will be pinned for an indefinite
		 * amount of time, so it is not expected to be migrated away.
		 */
		struct inode *inode = file_inode(memfd);
		struct hstate *h = hstate_file(memfd);
		int err = -ENOMEM;
		long nr_resv;

		gfp_mask = htlb_alloc_mask(h);
		gfp_mask &= ~(__GFP_HIGHMEM | __GFP_MOVABLE);
		idx >>= huge_page_order(h);

		nr_resv = hugetlb_reserve_pages(inode, idx, idx + 1, NULL, 0);
		if (nr_resv < 0)
			return ERR_PTR(nr_resv);

		folio = alloc_hugetlb_folio_reserve(h,
						    numa_node_id(),
						    NULL,
						    gfp_mask);
		if (folio) {
			err = hugetlb_add_to_page_cache(folio,
							memfd->f_mapping,
							idx);
			if (err) {
				folio_put(folio);
				goto err_unresv;
			}

			hugetlb_set_folio_subpool(folio, subpool_inode(inode));
			folio_unlock(folio);
			return folio;
		}
err_unresv:
		if (nr_resv > 0)
			hugetlb_unreserve_pages(inode, idx, idx + 1, 0);
		return ERR_PTR(err);
	}
#endif
	return shmem_read_folio(memfd->f_mapping, idx);
}

/*
 * Setting SEAL_WRITE requires us to verify there's no pending writer. However,
 * via get_user_pages(), drivers might have some pending I/O without any active
 * user-space mappings (eg., direct-IO, AIO). Therefore, we look at all folios
 * and see whether it has an elevated ref-count. If so, we tag them and wait for
 * them to be dropped.
 * The caller must guarantee that no new user will acquire writable references
 * to those folios to avoid races.
 */
static int memfd_wait_for_pins(struct address_space *mapping)
{
	XA_STATE(xas, &mapping->i_pages, 0);
	struct folio *folio;
	int error, scan;

	memfd_tag_pins(&xas);

	error = 0;
	for (scan = 0; scan <= LAST_SCAN; scan++) {
		int latency = 0;

		if (!xas_marked(&xas, MEMFD_TAG_PINNED))
			break;

		if (!scan)
			lru_add_drain_all();
		else if (schedule_timeout_killable((HZ << scan) / 200))
			scan = LAST_SCAN;

		xas_set(&xas, 0);
		xas_lock_irq(&xas);
		xas_for_each_marked(&xas, folio, ULONG_MAX, MEMFD_TAG_PINNED) {
			bool clear = true;

			if (!xa_is_value(folio) &&
			    memfd_folio_has_extra_refs(folio)) {
				/*
				 * On the last scan, we clean up all those tags
				 * we inserted; but make a note that we still
				 * found folios pinned.
				 */
				if (scan == LAST_SCAN)
					error = -EBUSY;
				else
					clear = false;
			}
			if (clear)
				xas_clear_mark(&xas, MEMFD_TAG_PINNED);

			if (++latency < XA_CHECK_SCHED)
				continue;
			latency = 0;

			xas_pause(&xas);
			xas_unlock_irq(&xas);
			cond_resched();
			xas_lock_irq(&xas);
		}
		xas_unlock_irq(&xas);
	}

	return error;
}

static unsigned int *memfd_file_seals_ptr(struct file *file)
{
	if (shmem_file(file))
		return &SHMEM_I(file_inode(file))->seals;

#ifdef CONFIG_HUGETLBFS
	if (is_file_hugepages(file))
		return &HUGETLBFS_I(file_inode(file))->seals;
#endif

	return NULL;
}

#define F_ALL_SEALS (F_SEAL_SEAL | \
		     F_SEAL_EXEC | \
		     F_SEAL_SHRINK | \
		     F_SEAL_GROW | \
		     F_SEAL_WRITE | \
		     F_SEAL_FUTURE_WRITE)

static int memfd_add_seals(struct file *file, unsigned int seals)
{
	struct inode *inode = file_inode(file);
	unsigned int *file_seals;
	int error;

	/*
	 * SEALING
	 * Sealing allows multiple parties to share a tmpfs or hugetlbfs file
	 * but restrict access to a specific subset of file operations. Seals
	 * can only be added, but never removed. This way, mutually untrusted
	 * parties can share common memory regions with a well-defined policy.
	 * A malicious peer can thus never perform unwanted operations on a
	 * shared object.
	 *
	 * Seals are only supported on special tmpfs or hugetlbfs files and
	 * always affect the whole underlying inode. Once a seal is set, it
	 * may prevent some kinds of access to the file. Currently, the
	 * following seals are defined:
	 *   SEAL_SEAL: Prevent further seals from being set on this file
	 *   SEAL_SHRINK: Prevent the file from shrinking
	 *   SEAL_GROW: Prevent the file from growing
	 *   SEAL_WRITE: Prevent write access to the file
	 *   SEAL_EXEC: Prevent modification of the exec bits in the file mode
	 *
	 * As we don't require any trust relationship between two parties, we
	 * must prevent seals from being removed. Therefore, sealing a file
	 * only adds a given set of seals to the file, it never touches
	 * existing seals. Furthermore, the "setting seals"-operation can be
	 * sealed itself, which basically prevents any further seal from being
	 * added.
	 *
	 * Semantics of sealing are only defined on volatile files. Only
	 * anonymous tmpfs and hugetlbfs files support sealing. More
	 * importantly, seals are never written to disk. Therefore, there's
	 * no plan to support it on other file types.
	 */

	if (!(file->f_mode & FMODE_WRITE))
		return -EPERM;
	if (seals & ~(unsigned int)F_ALL_SEALS)
		return -EINVAL;

	inode_lock(inode);

	file_seals = memfd_file_seals_ptr(file);
	if (!file_seals) {
		error = -EINVAL;
		goto unlock;
	}

	if (*file_seals & F_SEAL_SEAL) {
		error = -EPERM;
		goto unlock;
	}

	if ((seals & F_SEAL_WRITE) && !(*file_seals & F_SEAL_WRITE)) {
		error = mapping_deny_writable(file->f_mapping);
		if (error)
			goto unlock;

		error = memfd_wait_for_pins(file->f_mapping);
		if (error) {
			mapping_allow_writable(file->f_mapping);
			goto unlock;
		}
	}

	/*
	 * SEAL_EXEC implies SEAL_WRITE, making W^X from the start.
	 */
	if (seals & F_SEAL_EXEC && inode->i_mode & 0111)
		seals |= F_SEAL_SHRINK|F_SEAL_GROW|F_SEAL_WRITE|F_SEAL_FUTURE_WRITE;

	*file_seals |= seals;
	error = 0;

unlock:
	inode_unlock(inode);
	return error;
}

static int memfd_get_seals(struct file *file)
{
	unsigned int *seals = memfd_file_seals_ptr(file);

	return seals ? *seals : -EINVAL;
}

long memfd_fcntl(struct file *file, unsigned int cmd, unsigned int arg)
{
	long error;

	switch (cmd) {
	case F_ADD_SEALS:
		error = memfd_add_seals(file, arg);
		break;
	case F_GET_SEALS:
		error = memfd_get_seals(file);
		break;
	default:
		error = -EINVAL;
		break;
	}

	return error;
}

#define MFD_NAME_PREFIX "memfd:"
#define MFD_NAME_PREFIX_LEN (sizeof(MFD_NAME_PREFIX) - 1)
#define MFD_NAME_MAX_LEN (NAME_MAX - MFD_NAME_PREFIX_LEN)

#define MFD_ALL_FLAGS (MFD_CLOEXEC | MFD_ALLOW_SEALING | MFD_HUGETLB | MFD_NOEXEC_SEAL | MFD_EXEC)

static int check_sysctl_memfd_noexec(unsigned int *flags)
{
#ifdef CONFIG_SYSCTL
	struct pid_namespace *ns = task_active_pid_ns(current);
	int sysctl = pidns_memfd_noexec_scope(ns);

	if (!(*flags & (MFD_EXEC | MFD_NOEXEC_SEAL))) {
		if (sysctl >= MEMFD_NOEXEC_SCOPE_NOEXEC_SEAL)
			*flags |= MFD_NOEXEC_SEAL;
		else
			*flags |= MFD_EXEC;
	}

	if (!(*flags & MFD_NOEXEC_SEAL) && sysctl >= MEMFD_NOEXEC_SCOPE_NOEXEC_ENFORCED) {
		pr_err_ratelimited(
			"%s[%d]: memfd_create() requires MFD_NOEXEC_SEAL with vm.memfd_noexec=%d\n",
			current->comm, task_pid_nr(current), sysctl);
		return -EACCES;
	}
#endif
	return 0;
}

static inline bool is_write_sealed(unsigned int seals)
{
	return seals & (F_SEAL_WRITE | F_SEAL_FUTURE_WRITE);
}

static int check_write_seal(vm_flags_t *vm_flags_ptr)
{
	vm_flags_t vm_flags = *vm_flags_ptr;
	vm_flags_t mask = vm_flags & (VM_SHARED | VM_WRITE);

	/* If a private mapping then writability is irrelevant. */
	if (!(mask & VM_SHARED))
		return 0;

	/*
	 * New PROT_WRITE and MAP_SHARED mmaps are not allowed when
	 * write seals are active.
	 */
	if (mask & VM_WRITE)
		return -EPERM;

	/*
	 * This is a read-only mapping, disallow mprotect() from making a
	 * write-sealed mapping writable in future.
	 */
	*vm_flags_ptr &= ~VM_MAYWRITE;

	return 0;
}

int memfd_check_seals_mmap(struct file *file, vm_flags_t *vm_flags_ptr)
{
	int err = 0;
	unsigned int *seals_ptr = memfd_file_seals_ptr(file);
	unsigned int seals = seals_ptr ? *seals_ptr : 0;

	if (is_write_sealed(seals))
		err = check_write_seal(vm_flags_ptr);

	return err;
}

static int sanitize_flags(unsigned int *flags_ptr)
{
	unsigned int flags = *flags_ptr;

	if (!(flags & MFD_HUGETLB)) {
		if (flags & ~MFD_ALL_FLAGS)
			return -EINVAL;
	} else {
		/* Allow huge page size encoding in flags. */
		if (flags & ~(MFD_ALL_FLAGS |
				(MFD_HUGE_MASK << MFD_HUGE_SHIFT)))
			return -EINVAL;
	}

	/* Invalid if both EXEC and NOEXEC_SEAL are set.*/
	if ((flags & MFD_EXEC) && (flags & MFD_NOEXEC_SEAL))
		return -EINVAL;

	return check_sysctl_memfd_noexec(flags_ptr);
}

static char *alloc_name(const char __user *uname)
{
	int error;
	char *name;
	long len;

	name = kmalloc(NAME_MAX + 1, GFP_KERNEL);
	if (!name)
		return ERR_PTR(-ENOMEM);

	memcpy(name, MFD_NAME_PREFIX, MFD_NAME_PREFIX_LEN);
	/* returned length does not include terminating zero */
	len = strncpy_from_user(&name[MFD_NAME_PREFIX_LEN], uname, MFD_NAME_MAX_LEN + 1);
	if (len < 0) {
		error = -EFAULT;
		goto err_name;
	} else if (len > MFD_NAME_MAX_LEN) {
		error = -EINVAL;
		goto err_name;
	}

	return name;

err_name:
	kfree(name);
	return ERR_PTR(error);
}

static struct file *alloc_file(const char *name, unsigned int flags)
{
	unsigned int *file_seals;
	struct file *file;
	struct inode *inode;
	int err = 0;

	if (flags & MFD_HUGETLB) {
		file = hugetlb_file_setup(name, 0, VM_NORESERVE,
					HUGETLB_ANONHUGE_INODE,
					(flags >> MFD_HUGE_SHIFT) &
					MFD_HUGE_MASK);
	} else {
		file = shmem_file_setup(name, 0, VM_NORESERVE);
	}
	if (IS_ERR(file))
		return file;

	inode = file_inode(file);
	err = security_inode_init_security_anon(inode,
			&QSTR(MEMFD_ANON_NAME), NULL);
	if (err) {
		fput(file);
		file = ERR_PTR(err);
		return file;
	}

	file->f_mode |= FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE;
	file->f_flags |= O_LARGEFILE;

	if (flags & MFD_NOEXEC_SEAL) {
		inode->i_mode &= ~0111;
		file_seals = memfd_file_seals_ptr(file);
		if (file_seals) {
			*file_seals &= ~F_SEAL_SEAL;
			*file_seals |= F_SEAL_EXEC;
		}
	} else if (flags & MFD_ALLOW_SEALING) {
		/* MFD_EXEC and MFD_ALLOW_SEALING are set */
		file_seals = memfd_file_seals_ptr(file);
		if (file_seals)
			*file_seals &= ~F_SEAL_SEAL;
	}

	return file;
}

SYSCALL_DEFINE2(memfd_create,
		const char __user *, uname,
		unsigned int, flags)
{
	struct file *file;
	int fd, error;
	char *name;

	error = sanitize_flags(&flags);
	if (error < 0)
		return error;

	name = alloc_name(uname);
	if (IS_ERR(name))
		return PTR_ERR(name);

	fd = get_unused_fd_flags((flags & MFD_CLOEXEC) ? O_CLOEXEC : 0);
	if (fd < 0) {
		error = fd;
		goto err_free_name;
	}

	file = alloc_file(name, flags);
	if (IS_ERR(file)) {
		error = PTR_ERR(file);
		goto err_free_fd;
	}

	fd_install(fd, file);
	kfree(name);
	return fd;

err_free_fd:
	put_unused_fd(fd);
err_free_name:
	kfree(name);
	return error;
}
