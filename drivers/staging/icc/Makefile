# SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)-or-later

obj-$(CONFIG_ICC) := icc.o
PWD := core/

icc-$(CONFIG_CORE_CONTROL) := $(PWD)core_control.o
icc-$(CONFIG_ICC_PROTOCOL) += $(PWD)protocol.o

ccflags-y += -Idrivers/staging/icc/include
