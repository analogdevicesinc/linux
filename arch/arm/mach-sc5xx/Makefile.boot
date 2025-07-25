# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)

ifeq ($(CONFIG_ARCH_SC57X),y)
zreladdr-y	+= 0x82008000
params_phys-y	:= 0x82000100
endif

ifeq ($(CONFIG_ARCH_SC58X),y)
zreladdr-y	+= 0xC2008000
params_phys-y	:= 0xC2000100
endif

ifeq ($(CONFIG_ARCH_SC59X),y)
zreladdr-y      += 0xA0008000
params_phys-y   := 0xA0000100
endif
