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

fsl-ncsw-MAC-objs	:=  dtsec.o dtsec_mii_acc.o fm_mac.o tgec.o tgec_mii_acc.o \
			    fman_dtsec.o fman_dtsec_mii_acc.o fman_memac.o \
			    fman_tgec.o fman_crc32.o

ifeq ($(CONFIG_FMAN_V3H),y)
fsl-ncsw-MAC-objs	+=  memac.o memac_mii_acc.o fman_memac_mii_acc.o
endif
ifeq ($(CONFIG_FMAN_V3L),y)
fsl-ncsw-MAC-objs       +=  memac.o memac_mii_acc.o fman_memac_mii_acc.o
endif
ifeq ($(CONFIG_FMAN_ARM),y)
fsl-ncsw-MAC-objs       +=  memac.o memac_mii_acc.o fman_memac_mii_acc.o
endif

