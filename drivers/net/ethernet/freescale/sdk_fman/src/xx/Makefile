#
# Makefile for the Freescale Ethernet controllers
#
ccflags-y           += -DVERSION=\"\"
#
#Include netcomm SW specific definitions
include $(srctree)/drivers/net/ethernet/freescale/sdk_fman/ncsw_config.mk

obj-y		+= fsl-ncsw-xx.o

ifneq ($(CONFIG_FMAN_ARM),y)
fsl-ncsw-xx-objs	:=  xx_linux.o \
				module_strings.o
else
fsl-ncsw-xx-objs	:= xx_arm_linux.o \
				module_strings.o
endif

