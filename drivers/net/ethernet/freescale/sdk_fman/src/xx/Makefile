#
# Makefile for the Freescale Ethernet controllers
#
ccflags-y           += -DVERSION=\"\"
#
#Include netcomm SW specific definitions
include $(srctree)/drivers/net/ethernet/freescale/sdk_fman/ncsw_config.mk

obj-y		+= fsl-ncsw-xx.o

fsl-ncsw-xx-objs	:= xx_arm_linux.o \
				module_strings.o
