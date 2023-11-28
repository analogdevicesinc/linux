# SPDX-License-Identifier: GPL-2.0
# Makefile for WAVE6 driver

wave6-objs += wave6-vpu.o \
	wave6-vdi.o \
	wave6-vpuapi.o \
	wave6-vpu-dec.o \
	wave6-vpu-enc.o \
	wave6-hw.o \
	wave6-vpu-v4l2.o \
	wave6-vpu-dbg.o

obj-$(CONFIG_MXC_VIDEO_WAVE6_CTRL) += wave6-vpu-ctrl.o
obj-$(CONFIG_MXC_VIDEO_WAVE6) += wave6.o
