#
# Makefile for the Freescale Ethernet controllers
#
ccflags-y           += -DVERSION=\"\"
#
#Include netcomm SW specific definitions
include $(srctree)/drivers/net/ethernet/freescale/sdk_fman/ncsw_config.mk
NCSW_FM_INC = $(srctree)/drivers/net/ethernet/freescale/sdk_fman/Peripherals/FM/inc

ccflags-y += -I$(NCSW_FM_INC)


obj-y		+= fsl-ncsw-PFM1.o

fsl-ncsw-PFM1-objs	:=   fm.o fm_muram.o fman.o

obj-y		+= MAC/
obj-y		+= Pcd/
obj-y		+= SP/
obj-y		+= Port/
obj-y		+= HC/
obj-y		+= MACSEC/

obj-$(CONFIG_FSL_SDK_FMAN_RTC_API)	+= Rtc/
