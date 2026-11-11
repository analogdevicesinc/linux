.. SPDX-License-Identifier: GPL-2.0

===================================
File management in the Linux kernel
===================================

This document describes how locking for files (struct file)
and file descriptor table (struct files) works.

Up until 2.6.12, the file descriptor table has been protected
with a lock (files->file_lock) and reference count (files->count).
->file_lock protected accesses to all the file related fields
of the table. ->count was used for sharing the file descriptor
table between tasks cloned with CLONE_FILES flag. Typically
this would be the case for posix threads. As with the common
refcounting model in the kernel, the last task doing
a put_files_struct() frees the file descriptor (fd) table.
The files (struct file) themselves are protected using
reference count (->f_count).

In the new lock-free model of file descriptor management,
the reference counting is similar, but the locking is
based on RCU. The file descriptor table contains multiple
elements - the fd sets (open_fds and close_on_exec, the
array of file pointers, the sizes of the sets and the array
etc.). In order for the updates to appear atomic to
a lock-free reader, all the elements of the file descriptor
table are in a separate structure - struct fdtable.
files_struct contains a pointer to struct fdtable through
which the actual fd table is accessed. Initially the
fdtable is embedded in files_struct itself. On a subsequent
expansion of fdtable, a new fdtable structure is allocated
and files->fdtab points to the new structure. The fdtable
structure is freed with RCU and lock-free readers either
see the old fdtable or the new fdtable making the update
appear atomic. Here are the locking rules for
the fdtable structure -

1. All references to the fdtable must be done through
   the files_fdtable() macro::

	struct fdtable *fdt;

	rcu_read_lock();

	fdt = files_fdtable(files);
	....
	if (n <= fdt->max_fds)
		....
	...
	rcu_read_unlock();

   files_fdtable() uses rcu_dereference() macro which takes care of
   the memory barrier requirements for lock-free dereference.
   The fdtable pointer must be read within the read-side
   critical section.

2. Reading of the fdtable as described above must be protected
   by rcu_read_lock()/rcu_read_unlock().

3. For any update to the fd table, files->file_lock must
   be held.

4. To look up the file structure given an fd, a reader must use one of
   the fget() helpers: fget(), fget_raw(), fget_task(), fget_task_next()
   or fdget() and friends. These take the reference and take care of
   the barrier requirements of the lock-free lookup.

   An example::

	struct file *file;

	file = fget(fd);
	if (file) {
		...
		fput(file);
	}
	....

   fdget() skips the reference when the file descriptor table isn't
   shared, see the comment above __fget_light() for the rules that come
   with that. Under files->file_lock a file can be looked up without a
   reference with files_lookup_fd_locked() as long as the lock is held.

5. Since both fdtable and file structures can be looked up
   lock-free, they must be installed using rcu_assign_pointer()
   API. If they are looked up lock-free, rcu_dereference()
   must be used. However it is advisable to use files_fdtable()
   and the fget() helpers which take care of these issues.

6. While updating, the fdtable pointer must be looked up while
   holding files->file_lock. If ->file_lock is dropped, then
   another thread expand the files thereby creating a new
   fdtable and making the earlier fdtable pointer stale.

   For example::

	spin_lock(&files->file_lock);
	fd = locate_fd(files, file, start);
	if (fd >= 0) {
		/* locate_fd() may have expanded fdtable, load the ptr */
		fdt = files_fdtable(files);
		__set_open_fd(fd, fdt);
		__clear_close_on_exec(fd, fdt);
		spin_unlock(&files->file_lock);
	.....

   Since locate_fd() can drop ->file_lock (and reacquire ->file_lock),
   the fdtable pointer (fdt) must be loaded after locate_fd().

Files are allocated from a SLAB_TYPESAFE_BY_RCU cache since commit
0ede61d8589c ("file: convert to SLAB_TYPESAFE_BY_RCU"). Freeing a file
doesn't wait for a grace period anymore. The memory can be handed out
again right away and only the type of the object is stable under
rcu_read_lock(). So it isn't sufficient to take a reference on the file
found in a descriptor slot. The file might have been closed, freed and
recycled in the meantime and the reference lands on whoever owns the
object now. The lock-free lookup in __fget_files_rcu() and
__get_file_rcu() therefore:

(1) loads the pointer from the slot and takes a reference with
    file_ref_get(), which fails if the file is dead,

(2) reloads the pointer and compares it against the earlier load, if they
    differ the file was recycled and the reference is dropped again, and

(3) returns the reloaded pointer so that everything the caller reads
    from the file depends on the reload.

In addition, it isn't possible to access or check fields in struct file
without first acquiring a reference on it under rcu lookup. Not doing
that was always very dodgy and it was only usable for non-pointer data
in struct file. With SLAB_TYPESAFE_BY_RCU it is necessary that callers
either first acquire a reference or they must hold the files_lock of the
fdtable.
