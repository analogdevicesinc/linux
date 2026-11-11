.. SPDX-License-Identifier: GPL-2.0

==========================
Platform Profile Selection
==========================

On modern systems the platform performance, temperature, fan and other
hardware related characteristics are often dynamically configurable. The
platform configuration is often automatically adjusted to the current
conditions by some automatic mechanism (which may very well live outside
the kernel).

These auto platform adjustment mechanisms often can be configured with
one of several platform profiles, with either a bias towards low power
operation or towards performance.

The purpose of the platform_profile attribute is to offer a generic sysfs
API for selecting the platform profile of these automatic mechanisms.

The framework is independent of the firmware interface used by a provider.
Drivers can implement profile selection using ACPI, an embedded controller,
or another platform-specific mechanism. The core does not discover providers
or interpret firmware descriptions.

The firmware-independent userspace interface is available under
``/sys/class/platform-profile/platform-profile-X/``, with ``name``,
``choices`` and ``profile`` attributes for each registered provider.

On ACPI systems with ``CONFIG_PLATFORM_PROFILE_LEGACY`` enabled,
``/sys/firmware/acpi/platform_profile_choices`` and
``/sys/firmware/acpi/platform_profile`` remain available as compatibility
interfaces for aggregate profile selection across registered providers.
New userspace should select profiles through the individual provider class
devices. These remain available when the legacy interface is disabled and
on non-ACPI systems. Kernel profile cycling also remains available without
the legacy interface.

The legacy aggregate interface is deprecated. Removal is intended after the
2028 LTS kernel, once userspace has migrated to released software using the
class devices. Compatibility remains enabled by default during the migration;
the date alone does not trigger removal. Accessing either legacy attribute
emits a one-time kernel notice directing userspace to the class interface.

Note that this API is only for selecting the platform profile, it is
NOT a goal of this API to allow monitoring the resulting performance
characteristics. Monitoring performance is best done with device/vendor
specific tools, e.g. turbostat.

Specifically, when selecting a high performance profile the actual achieved
performance may be limited by various factors such as: the heat generated
by other components, room temperature, free air flow at the bottom of a
laptop, etc. It is explicitly NOT a goal of this API to let userspace know
about any sub-optimal conditions which are impeding reaching the requested
performance level.

Since numbers on their own cannot represent the multiple variables that a
profile will adjust (power consumption, heat generation, etc) this API
uses strings to describe the various profiles. To make sure that userspace
gets a consistent experience the sysfs-platform_profile ABI document defines
a fixed set of profile names. Drivers *must* map their internal profile
representation onto this fixed set.

If there is no good match when mapping then a new profile name may be
added. Drivers which wish to introduce new profile names must:

 1. Explain why the existing profile names cannot be used.
 2. Add the new profile name, along with a clear description of the
    expected behaviour, to the sysfs-platform_profile ABI documentation.

"Custom" profile support
========================
The platform_profile class also supports profiles advertising a "custom"
profile. This is intended to be set by drivers when the settings in the
driver have been modified in a way that a standard profile doesn't represent
the current state.

Multiple driver support
=======================
When multiple drivers on a system advertise a platform profile handler, the
platform profile handler core will only advertise the profiles that are
common between all drivers to the ``/sys/firmware/acpi`` interfaces, when
ACPI is active and ``CONFIG_PLATFORM_PROFILE_LEGACY`` is enabled.

This is to ensure there is no ambiguity on what the profile names mean when
all handlers don't support a profile.

Individual drivers register a ``platform-profile`` class device with
``name``, ``choices`` and ``profile`` attributes.

To discover which driver is associated with a platform profile handler the
user can read the ``name`` attribute of the class device.

To discover available profiles from the class interface the user can read the
``choices`` attribute.

If a user wants to select a profile for a specific driver, they can do so
by writing to the ``profile`` attribute of the driver's class device.

This will allow users to set different profiles for different drivers on the
same system. If the selected profile by individual drivers differs the
ACPI aggregate interface will display the profile 'custom' to indicate that
the profiles are not the same.

While the ACPI ``platform_profile`` attribute has the value ``custom``,
writing a common profile from ``platform_profile_choices`` to it will set
the profile for all drivers. ``custom`` cannot be written to this aggregate
interface.

Aggregate profile changes are applied to providers sequentially. If a provider
returns an error, providers updated earlier are not rolled back. Userspace can
read the per-provider interfaces to determine their resulting state.
