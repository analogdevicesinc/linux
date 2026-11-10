// SPDX-License-Identifier: GPL-2.0-or-later
/* Compatibility with the original ACPI platform profile sysfs ABI. */

#include <linux/acpi.h>
#include <linux/kobject.h>

#include "profile.h"

struct kobject *platform_profile_compat_kobj(void)
{
	if (acpi_disabled)
		return NULL;

	return kobject_get(acpi_kobj);
}
