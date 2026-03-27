# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_DAX) += dax.o
obj-$(CONFIG_DEV_DAX) += device_dax.o
obj-$(CONFIG_DEV_DAX_KMEM) += kmem.o
obj-$(CONFIG_DEV_DAX_PMEM) += dax_pmem.o
obj-$(CONFIG_DEV_DAX_CXL) += dax_cxl.o
obj-$(CONFIG_DEV_DAX_FSDEV) += fsdev_dax.o

dax-y := super.o
dax-y += bus.o
device_dax-y := device.o
dax_pmem-y := pmem.o
dax_cxl-y := cxl.o
fsdev_dax-y := fsdev.o

obj-y += hmem/
