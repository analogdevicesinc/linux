# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)-or-later

ifeq ($(CONFIG_MACH_SC594_SOM),y)
zreladdr-y      += 0x82008000
params_phys-y   := 0x82000100
else
zreladdr-y	+= 0x80008000
params_phys-y	:= 0x80000100
endif
