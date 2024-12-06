# SPDX-License-Identifier: GPL-2.0-only
# drm/tegra depends on host1x, so if both drivers are built-in care must be
# taken to initialize them in the correct order. Link order is the only way
# to ensure this currently.
obj-y			+= arm/
obj-y			+= host1x/ drm/ vga/
obj-y			+= imx/
obj-$(CONFIG_TRACE_GPU_MEM)		+= trace/
