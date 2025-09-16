# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)

ifeq ($(CONFIG_MACH_SC573_EZKIT),y)
zreladdr-y	+= 0x82008000
params_phys-y	:= 0x82000100
endif

ifeq ($(CONFIG_MACH_SC584_EZKIT),y)
zreladdr-y	+= 0x89008000
params_phys-y	:= 0x89000100
endif

ifeq ($(CONFIG_MACH_SC589_EZKIT),y)
zreladdr-y	+= 0xC2008000
params_phys-y	:= 0xC2000100
endif

ifeq ($(CONFIG_MACH_SC589_MINI),y)
zreladdr-y	+= 0xC2008000
params_phys-y	:= 0xC2000100
endif

ifeq ($(CONFIG_MACH_SC594_SOM),y)
zreladdr-y      += 0xA0008000
params_phys-y   := 0xA0000100
endif
