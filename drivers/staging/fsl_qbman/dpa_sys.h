/* Copyright 2008-2012 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DPA_SYS_H
#define DPA_SYS_H

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/bootmem.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/kthread.h>
#include <linux/memblock.h>
#include <linux/completion.h>
#include <linux/log2.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/device.h>
#include <linux/uio_driver.h>
#include <linux/smp.h>
#include <linux/fsl_hypervisor.h>
#include <linux/vmalloc.h>
#include <linux/ctype.h>
#include <linux/math64.h>
#include <linux/bitops.h>

#include <linux/fsl_usdpaa.h>

/* When copying aligned words or shorts, try to avoid memcpy() */
#define CONFIG_TRY_BETTER_MEMCPY

/* For 2-element tables related to cache-inhibited and cache-enabled mappings */
#define DPA_PORTAL_CE 0
#define DPA_PORTAL_CI 1

/***********************/
/* Misc inline assists */
/***********************/

#if defined CONFIG_PPC32
#include "dpa_sys_ppc32.h"
#elif defined CONFIG_PPC64
#include "dpa_sys_ppc64.h"
#elif defined CONFIG_ARM
#include "dpa_sys_arm.h"
#elif defined CONFIG_ARM64
#include "dpa_sys_arm64.h"
#endif


#ifdef CONFIG_FSL_DPA_CHECKING
#define DPA_ASSERT(x) \
	do { \
		if (!(x)) { \
			pr_crit("ASSERT: (%s:%d) %s\n", __FILE__, __LINE__, \
				__stringify_1(x)); \
			dump_stack(); \
			panic("assertion failure"); \
		} \
	} while (0)
#else
#define DPA_ASSERT(x)
#endif

/* memcpy() stuff - when you know alignments in advance */
#ifdef CONFIG_TRY_BETTER_MEMCPY
static inline void copy_words(void *dest, const void *src, size_t sz)
{
	u32 *__dest = dest;
	const u32 *__src = src;
	size_t __sz = sz >> 2;
	BUG_ON((unsigned long)dest & 0x3);
	BUG_ON((unsigned long)src & 0x3);
	BUG_ON(sz & 0x3);
	while (__sz--)
		*(__dest++) = *(__src++);
}
static inline void copy_shorts(void *dest, const void *src, size_t sz)
{
	u16 *__dest = dest;
	const u16 *__src = src;
	size_t __sz = sz >> 1;
	BUG_ON((unsigned long)dest & 0x1);
	BUG_ON((unsigned long)src & 0x1);
	BUG_ON(sz & 0x1);
	while (__sz--)
		*(__dest++) = *(__src++);
}
static inline void copy_bytes(void *dest, const void *src, size_t sz)
{
	u8 *__dest = dest;
	const u8 *__src = src;
	while (sz--)
		*(__dest++) = *(__src++);
}
#else
#define copy_words memcpy
#define copy_shorts memcpy
#define copy_bytes memcpy
#endif

/************/
/* RB-trees */
/************/

/* We encapsulate RB-trees so that its easier to use non-linux forms in
 * non-linux systems. This also encapsulates the extra plumbing that linux code
 * usually provides when using RB-trees. This encapsulation assumes that the
 * data type held by the tree is u32. */

struct dpa_rbtree {
	struct rb_root root;
};
#define DPA_RBTREE { .root = RB_ROOT }

static inline void dpa_rbtree_init(struct dpa_rbtree *tree)
{
	tree->root = RB_ROOT;
}

#define IMPLEMENT_DPA_RBTREE(name, type, node_field, val_field) \
static inline int name##_push(struct dpa_rbtree *tree, type *obj) \
{ \
	struct rb_node *parent = NULL, **p = &tree->root.rb_node; \
	while (*p) { \
		u32 item; \
		parent = *p; \
		item = rb_entry(parent, type, node_field)->val_field; \
		if (obj->val_field < item) \
			p = &parent->rb_left; \
		else if (obj->val_field > item) \
			p = &parent->rb_right; \
		else \
			return -EBUSY; \
	} \
	rb_link_node(&obj->node_field, parent, p); \
	rb_insert_color(&obj->node_field, &tree->root); \
	return 0; \
} \
static inline void name##_del(struct dpa_rbtree *tree, type *obj) \
{ \
	rb_erase(&obj->node_field, &tree->root); \
} \
static inline type *name##_find(struct dpa_rbtree *tree, u32 val) \
{ \
	type *ret; \
	struct rb_node *p = tree->root.rb_node; \
	while (p) { \
		ret = rb_entry(p, type, node_field); \
		if (val < ret->val_field) \
			p = p->rb_left; \
		else if (val > ret->val_field) \
			p = p->rb_right; \
		else \
			return ret; \
	} \
	return NULL; \
}

/************/
/* Bootargs */
/************/

/* Qman has "qportals=" and Bman has "bportals=", they use the same syntax
 * though; a comma-separated list of items, each item being a cpu index and/or a
 * range of cpu indices, and each item optionally be prefixed by "s" to indicate
 * that the portal associated with that cpu should be shared. See bman_driver.c
 * for more specifics. */
static int __parse_portals_cpu(const char **s, unsigned int *cpu)
{
	*cpu = 0;
	if (!isdigit(**s))
		return -EINVAL;
	while (isdigit(**s))
		*cpu = *cpu * 10 + (*((*s)++) - '0');
	return 0;
}
static inline int parse_portals_bootarg(char *str, struct cpumask *want_shared,
					struct cpumask *want_unshared,
					const char *argname)
{
	const char *s = str;
	unsigned int shared, cpu1, cpu2, loop;

keep_going:
	if (*s == 's') {
		shared = 1;
		s++;
	} else
		shared = 0;
	if (__parse_portals_cpu(&s, &cpu1))
		goto err;
	if (*s == '-') {
		s++;
		if (__parse_portals_cpu(&s, &cpu2))
			goto err;
		if (cpu2 < cpu1)
			goto err;
	} else
		cpu2 = cpu1;
	for (loop = cpu1; loop <= cpu2; loop++)
		cpumask_set_cpu(loop, shared ? want_shared : want_unshared);
	if (*s == ',') {
		s++;
		goto keep_going;
	} else if ((*s == '\0') || isspace(*s))
		return 0;
err:
	pr_crit("Malformed %s argument: %s, offset: %lu\n", argname, str,
		(unsigned long)s - (unsigned long)str);
	return -EINVAL;
}
#ifdef CONFIG_FSL_USDPAA
/* Hooks from fsl_usdpaa_irq.c to fsl_usdpaa.c */
int usdpaa_get_portal_config(struct file *filp, void *cinh,
			     enum usdpaa_portal_type ptype, unsigned int *irq,
			     void **iir_reg);
#endif
#endif /* DPA_SYS_H */
