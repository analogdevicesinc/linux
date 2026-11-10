/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef _PLATFORM_PROFILE_INTERNAL_H
#define _PLATFORM_PROFILE_INTERNAL_H

#include <linux/kconfig.h>
#include <linux/kobject.h>

#ifdef CONFIG_PLATFORM_PROFILE_LEGACY
struct kobject *platform_profile_compat_kobj(void);
#else
static inline struct kobject *platform_profile_compat_kobj(void)
{
	return NULL;
}
#endif

#endif
