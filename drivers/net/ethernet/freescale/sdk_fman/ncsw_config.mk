#
# Makefile config for the Freescale NetcommSW
#
NET_DPA     = $(srctree)/drivers/net
DRV_DPA     = $(srctree)/drivers/net/ethernet/freescale/sdk_dpaa
FMAN        = $(srctree)/drivers/net/ethernet/freescale/sdk_fman

ifdef CONFIG_FMAN_V3H
ccflags-y +=-include $(FMAN)/fmanv3h_dflags.h
endif
ifdef CONFIG_FMAN_V3L
ccflags-y +=-include $(FMAN)/fmanv3l_dflags.h
endif
ifdef CONFIG_FMAN_ARM
ccflags-y +=-include $(FMAN)/ls1043_dflags.h
endif

ccflags-y += -I$(DRV_DPA)/
ccflags-y += -I$(FMAN)/inc
ccflags-y += -I$(FMAN)/inc/cores
ccflags-y += -I$(FMAN)/inc/etc
ccflags-y += -I$(FMAN)/inc/Peripherals
ccflags-y += -I$(FMAN)/inc/flib

ifdef CONFIG_FMAN_V3H
ccflags-y += -I$(FMAN)/inc/integrations/FMANV3H
endif
ifdef CONFIG_FMAN_V3L
ccflags-y += -I$(FMAN)/inc/integrations/FMANV3L
endif
ifdef CONFIG_FMAN_ARM
ccflags-y += -I$(FMAN)/inc/integrations/LS1043
endif

# FM_COMPAT is used in kernel headers in case of kernel option leaking
ifeq ("$(CONFIG_COMPAT)", "y")
ccflags-y += -DFM_COMPAT
endif

ccflags-y += -I$(FMAN)/src/inc
ccflags-y += -I$(FMAN)/src/inc/system
ccflags-y += -I$(FMAN)/src/inc/wrapper
ccflags-y += -I$(FMAN)/src/inc/xx
ccflags-y += -I$(srctree)/include/uapi/linux/fmd
ccflags-y += -I$(srctree)/include/uapi/linux/fmd/Peripherals
ccflags-y += -I$(srctree)/include/uapi/linux/fmd/integrations
