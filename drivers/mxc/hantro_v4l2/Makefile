#
# Makefile for the VPU drivers.
#

EXTRA_CFLAGS += -I$(src)/include/ -DV4L2_DRIVER

obj-$(CONFIG_MXC_HANTRO_V4L2) = vsiv4l2.o
vsiv4l2-objs = vsi-v4l2.o vsi-v4l2daemon.o vsi-v4l2-config.o vsi-v4l2-dec.o vsi-v4l2-enc.o
