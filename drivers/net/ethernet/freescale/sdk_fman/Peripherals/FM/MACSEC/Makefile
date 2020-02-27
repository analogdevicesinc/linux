#
# Makefile for the Freescale Ethernet controllers
#
ccflags-y           += -DVERSION=\"\"
#
#Include netcomm SW specific definitions
include $(srctree)/drivers/net/ethernet/freescale/sdk_fman/ncsw_config.mk

NCSW_FM_INC = $(srctree)/drivers/net/ethernet/freescale/sdk_fman/Peripherals/FM/inc

ccflags-y += -I$(NCSW_FM_INC)

obj-y		+= fsl-ncsw-macsec.o

fsl-ncsw-macsec-objs	:= fm_macsec.o fm_macsec_guest.o fm_macsec_master.o fm_macsec_secy.o
