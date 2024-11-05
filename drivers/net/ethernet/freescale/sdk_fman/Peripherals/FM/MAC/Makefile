#
# Makefile for the Freescale Ethernet controllers
#
ccflags-y           += -DVERSION=\"\"
#
#Include netcomm SW specific definitions
include $(srctree)/drivers/net/ethernet/freescale/sdk_fman/ncsw_config.mk

NCSW_FM_INC = $(srctree)/drivers/net/ethernet/freescale/sdk_fman/Peripherals/FM/inc

ccflags-y += -I$(NCSW_FM_INC)

obj-y		+= fsl-ncsw-MAC.o

fsl-ncsw-MAC-objs := \
	fm_mac.o \
	fman_crc32.o \
	fman_memac.o \
	fman_memac_mii_acc.o \
	memac.o \
	memac_mii_acc.o
