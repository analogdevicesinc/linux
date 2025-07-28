/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_CLEANUP_H
#define _LINUX_CLEANUP_H

#include <linux/compiler.h>

/**
 * DOC: scope-based cleanup helpers
 *
 * The "goto error" pattern is notorious for introducing subtle resource
 * leaks. It is tedious and error prone to add new resource acquisition
 * constraints into code paths that already have several unwind
 * conditions. The "cleanup" helpers enable the compiler to help with
 * this tedium and can aid in maintaining LIFO (last in first out)
 * unwind ordering to avoid unintentional leaks.
 *
 * As drivers make up the majority of the kernel code base, here is an
 * example of using these helpers to clean up PCI drivers. The target of
 * the cleanups are occasions where a goto is used to unwind a device
 * reference (pci_dev_put()), or unlock the device (pci_dev_unlock())
 * before returning.
 *
 * The DEFINE_FREE() macro can arrange for PCI device references to be
 * dropped when the associated variable goes out of scope::
 *
 *	DEFINE_FREE(pci_dev_put, struct pci_dev *, if (_T) pci_dev_put(_T))
 *	...
 *	struct pci_dev *dev __free(pci_dev_put) =
 *		pci_get_slot(parent, PCI_DEVFN(0, 0));
 *
 * The above will automatically call pci_dev_put() if @dev is non-NULL
 * when @dev goes out of scope (automatic variable scope). If a function
 * wants to invoke pci_dev_put() on error, but return @dev (i.e. without
 * freeing it) on success, it can do::
 *
 *	return no_free_ptr(dev);
 *
 * ...or::
 *
 *	return_ptr(dev);
 *
 * The DEFINE_GUARD() macro can arrange for the PCI device lock to be
 * dropped when the scope where guard() is invoked ends::
 *
 *	DEFINE_GUARD(pci_dev, struct pci_dev *, pci_dev_lock(_T), pci_dev_unlock(_T))
 *	...
 *	guard(pci_dev)(dev);
 *
 * The lifetime of the lock obtained by the guard() helper follows the
 * scope of automatic variable declaration. Take the following example::
 *
 *	func(...)
 *	{
 *		if (...) {
 *			...
 *			guard(pci_dev)(dev); // pci_dev_lock() invoked here
 *			...
 *		} // <- implied pci_dev_unlock() triggered here
 *	}
 *
 * Observe the lock is held for the remainder of the "if ()" block not
 * the remainder of "func()".
 *
 * Now, when a function uses both __free() and guard(), or multiple
 * instances of __free(), the LIFO order of variable definition order
 * matters. GCC documentation says:
 *
 * "When multiple variables in the same scope have cleanup attributes,
 * at exit from the scope their associated cleanup functions are run in
 * reverse order of definition (last defined, first cleanup)."
 *
 * When the unwind order matters it requires that variables be defined
 * mid-function scope rather than at the top of the file.  Take the
 * following example and notice the bug highlighted by "!!"::
 *
 *	LIST_HEAD(list);
 *	DEFINE_MUTEX(lock);
 *
 *	struct object {
 *	        struct list_head node;
 *	};
 *
 *	static struct object *alloc_add(void)
 *	{
 *	        struct object *obj;
 *
 *	        lockdep_assert_held(&lock);
 *	        obj = kzalloc(sizeof(*obj), GFP_KERNEL);
 *	        if (obj) {
 *	                LIST_HEAD_INIT(&obj->node);
 *	                list_add(obj->node, &list):
 *	        }
 *	        return obj;
 *	}
 *
 *	static void remove_free(struct object *obj)
 *	{
 *	        lockdep_assert_held(&lock);
 *	        list_del(&obj->node);
 *	        kfree(obj);
 *	}
 *
 *	DEFINE_FREE(remove_free, struct object *, if (_T) remove_free(_T))
 *	static int init(void)
 *	{
 *	        struct object *obj __free(remove_free) = NULL;
 *	        int err;
 *
 *	        guard(mutex)(&lock);
 *	        obj = alloc_add();
 *
 *	        if (!obj)
 *	                return -ENOMEM;
 *
 *	        err = other_init(obj);
 *	        if (err)
 *	                return err; // remove_free() called without the lock!!
 *
 *	        no_free_ptr(obj);
 *	        return 0;
 *	}
 *
 * That bug is fixed by changing init() to call guard() and define +
 * initialize @obj in this order::
 *
 *	guard(mutex)(&lock);
 *	struct object *obj __free(remove_free) = alloc_add();
 *
 * Given that the "__free(...) = NULL" pattern for variables defined at
 * the top of the function poses this potential interdependency problem
 * the recommendation is to always define and assign variables in one
 * statement and not group variable definitions at the top of the
 * function when __free() is used.
 *
 * Lastly, given that the benefit of cleanup helpers is removal of
 * "goto", and that the "goto" statement can jump between scopes, the
 * expectation is that usage of "goto" and cleanup helpers is never
 * mixed in the same function. I.e. for a given routine, convert all
 * resources that need a "goto" cleanup to scope-based cleanup, or
 * convert none of them.
 */

/*
 * DEFINE_FREE(name, type, free):
 *	simple helper macro that defines the required wrapper for a __free()
 *	based cleanup function. @free is an expression using '_T' to access the
 *	variable. @free should typically include a NULL test before calling a
 *	function, see the example below.
 *
 * __free(name):
 *	variable attribute to add a scoped based cleanup to the variable.
 *
 * no_free_ptr(var):
 *	like a non-atomic xchg(var, NULL), such that the cleanup function will
 *	be inhibited -- provided it sanely deals with a NULL value.
 *
 *	NOTE: this has __must_check semantics so that it is harder to accidentally
 *	leak the resource.
 *
 * return_ptr(p):
 *	returns p while inhibiting the __free().
 *
 * Ex.
 *
 * DEFINE_FREE(kfree, void *, if (_T) kfree(_T))
 *
 * void *alloc_obj(...)
 * {
 *	struct obj *p __free(kfree) = kmalloc(...);
 *	if (!p)
 *		return NULL;
 *
 *	if (!init_obj(p))
 *		return NULL;
 *
 *	return_ptr(p);
 * }
 *
 * NOTE: the DEFINE_FREE()'s @free expression includes a NULL test even though
 * kfree() is fine to be called with a NULL value. This is on purpose. This way
 * the compiler sees the end of our alloc_obj() function as:
 *
 *	tmp = p;
 *	p = NULL;
 *	if (p)
 *		kfree(p);
 *	return tmp;
 *
 * And through the magic of value-propagation and dead-code-elimination, it
 * eliminates the actual cleanup call and compiles into:
 *
 *	return p;
 *
 * Without the NULL test it turns into a mess and the compiler can't help us.
 */

#define DEFINE_FREE(_name, _type, _free) \
	static inline void __free_##_name(void *p) { _type _T = *(_type *)p; _free; }

#define __free(_name)	__cleanup(__free_##_name)

#define __get_and_null(p, nullvalue)   \
	({                                  \
		__auto_type __ptr = &(p);   \
		__auto_type __val = *__ptr; \
		*__ptr = nullvalue;         \
		__val;                      \
	})

static inline __must_check
const volatile void * __must_check_fn(const volatile void *val)
{ return val; }

#define no_free_ptr(p) \
	((typeof(p)) __must_check_fn(__get_and_null(p, NULL)))

#define return_ptr(p)	return no_free_ptr(p)


/*
 * DEFINE_CLASS(name, type, exit, init, init_args...):
 *	helper to define the destructor and constructor for a type.
 *	@exit is an expression using '_T' -- similar to FREE above.
 *	@init is an expression in @init_args resulting in @type
 *
 * EXTEND_CLASS(name, ext, init, init_args...):
 *	extends class @name to @name@ext with the new constructor
 *
 * CLASS(name, var)(args...):
 *	declare the variable @var as an instance of the named class
 *
 * Ex.
 *
 * DEFINE_CLASS(fdget, struct fd, fdput(_T), fdget(fd), int fd)
 *
 *	CLASS(fdget, f)(fd);
 *	if (!fd_file(f))
 *		return -EBADF;
 *
 *	// use 'f' without concern
 */

#define DEFINE_CLASS(_name, _type, _exit, _init, _init_args...)		\
typedef _type class_##_name##_t;					\
static inline void class_##_name##_destructor(_type *p)			\
{ _type _T = *p; _exit; }						\
static inline _type class_##_name##_constructor(_init_args)		\
{ _type t = _init; return t; }

#define EXTEND_CLASS(_name, ext, _init, _init_args...)			\
typedef class_##_name##_t class_##_name##ext##_t;			\
static inline void class_##_name##ext##_destructor(class_##_name##_t *p)\
{ class_##_name##_destructor(p); }					\
static inline class_##_name##_t class_##_name##ext##_constructor(_init_args) \
{ class_##_name##_t t = _init; return t; }

#define CLASS(_name, var)						\
	class_##_name##_t var __cleanup(class_##_name##_destructor) =	\
		class_##_name##_constructor


/*
 * DEFINE_GUARD(name, type, lock, unlock):
 *	trivial wrapper around DEFINE_CLASS() above specifically
 *	for locks.
 *
 * DEFINE_GUARD_COND(name, ext, condlock)
 *	wrapper around EXTEND_CLASS above to add conditional lock
 *	variants to a base class, eg. mutex_trylock() or
 *	mutex_lock_interruptible().
 *
 * guard(name):
 *	an anonymous instance of the (guard) class, not recommended for
 *	conditional locks.
 *
 * scoped_guard (name, args...) { }:
 *	similar to CLASS(name, scope)(args), except the variable (with the
 *	explicit name 'scope') is declard in a for-loop such that its scope is
 *	bound to the next (compound) statement.
 *
 *	for conditional locks the loop body is skipped when the lock is not
 *	acquired.
 *
 * scoped_cond_guard (name, fail, args...) { }:
 *      similar to scoped_guard(), except it does fail when the lock
 *      acquire fails.
 *
 */

#define DEFINE_GUARD(_name, _type, _lock, _unlock) \
	DEFINE_CLASS(_name, _type, if (_T) { _unlock; }, ({ _lock; _T; }), _type _T); \
	static inline void * class_##_name##_lock_ptr(class_##_name##_t *_T) \
	{ return *_T; }

#define DEFINE_GUARD_COND(_name, _ext, _condlock) \
	EXTEND_CLASS(_name, _ext, \
		     ({ void *_t = _T; if (_T && !(_condlock)) _t = NULL; _t; }), \
		     class_##_name##_t _T) \
	static inline void * class_##_name##_ext##_lock_ptr(class_##_name##_t *_T) \
	{ return class_##_name##_lock_ptr(_T); }

#define guard(_name) \
	CLASS(_name, __UNIQUE_ID(guard))

#define __guard_ptr(_name) class_##_name##_lock_ptr

#define scoped_guard(_name, args...)					\
	for (CLASS(_name, scope)(args),					\
	     *done = NULL; __guard_ptr(_name)(&scope) && !done; done = (void *)1)

#define scoped_cond_guard(_name, _fail, args...) \
	for (CLASS(_name, scope)(args), \
	     *done = NULL; !done; done = (void *)1) \
		if (!__guard_ptr(_name)(&scope)) _fail; \
		else

/*
 * Additional helper macros for generating lock guards with types, either for
 * locks that don't have a native type (eg. RCU, preempt) or those that need a
 * 'fat' pointer (eg. spin_lock_irqsave).
 *
 * DEFINE_LOCK_GUARD_0(name, lock, unlock, ...)
 * DEFINE_LOCK_GUARD_1(name, type, lock, unlock, ...)
 * DEFINE_LOCK_GUARD_1_COND(name, ext, condlock)
 *
 * will result in the following type:
 *
 *   typedef struct {
 *	type *lock;		// 'type := void' for the _0 variant
 *	__VA_ARGS__;
 *   } class_##name##_t;
 *
 * As above, both _lock and _unlock are statements, except this time '_T' will
 * be a pointer to the above struct.
 */

#define __DEFINE_UNLOCK_GUARD(_name, _type, _unlock, ...)		\
typedef struct {							\
	_type *lock;							\
	__VA_ARGS__;							\
} class_##_name##_t;							\
									\
static inline void class_##_name##_destructor(class_##_name##_t *_T)	\
{									\
	if (_T->lock) { _unlock; }					\
}									\
									\
static inline void *class_##_name##_lock_ptr(class_##_name##_t *_T)	\
{									\
	return _T->lock;						\
}


#define __DEFINE_LOCK_GUARD_1(_name, _type, _lock)			\
static inline class_##_name##_t class_##_name##_constructor(_type *l)	\
{									\
	class_##_name##_t _t = { .lock = l }, *_T = &_t;		\
	_lock;								\
	return _t;							\
}

#define __DEFINE_LOCK_GUARD_0(_name, _lock)				\
static inline class_##_name##_t class_##_name##_constructor(void)	\
{									\
	class_##_name##_t _t = { .lock = (void*)1 },			\
			 *_T __maybe_unused = &_t;			\
	_lock;								\
	return _t;							\
}

#define DEFINE_LOCK_GUARD_1(_name, _type, _lock, _unlock, ...)		\
__DEFINE_UNLOCK_GUARD(_name, _type, _unlock, __VA_ARGS__)		\
__DEFINE_LOCK_GUARD_1(_name, _type, _lock)

#define DEFINE_LOCK_GUARD_0(_name, _lock, _unlock, ...)			\
__DEFINE_UNLOCK_GUARD(_name, void, _unlock, __VA_ARGS__)		\
__DEFINE_LOCK_GUARD_0(_name, _lock)

#define DEFINE_LOCK_GUARD_1_COND(_name, _ext, _condlock)		\
	EXTEND_CLASS(_name, _ext,					\
		     ({ class_##_name##_t _t = { .lock = l }, *_T = &_t;\
		        if (_T->lock && !(_condlock)) _T->lock = NULL;	\
			_t; }),						\
		     typeof_member(class_##_name##_t, lock) l)		\
	static inline void * class_##_name##_ext##_lock_ptr(class_##_name##_t *_T) \
	{ return class_##_name##_lock_ptr(_T); }


#endif /* _LINUX_CLEANUP_H */
