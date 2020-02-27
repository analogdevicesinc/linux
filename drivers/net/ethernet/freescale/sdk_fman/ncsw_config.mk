#
# Makefile config for the Freescale NetcommSW
#
NET_DPA     = $(srctree)/drivers/net
DRV_DPA     = $(srctree)/drivers/net/ethernet/freescale/sdk_dpaa
FMAN        = $(srctree)/drivers/net/ethernet/freescale/sdk_fman

ifeq ("$(CONFIG_FMAN_P3040_P4080_P5020)", "y")
ccflags-y +=-include $(FMAN)/p3040_4080_5020_dflags.h
endif
ifeq ("$(CONFIG_FMAN_P1023)", "y")
ccflags-y +=-include $(FMAN)/p1023_dflags.h
endif
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

ifeq ("$(CONFIG_FMAN_P3040_P4080_P5020)", "y")
ccflags-y += -I$(FMAN)/inc/integrations/P3040_P4080_P5020
endif
ifeq ("$(CONFIG_FMAN_P1023)", "y")
ccflags-y += -I$(FMAN)/inc/integrations/P1023
endif
ifdef CONFIG_FMAN_V3H
ccflags-y += -I$(FMAN)/inc/integrations/FMANV3H
endif
ifdef CONFIG_FMAN_V3L
ccflags-y += -I$(FMAN)/inc/integrations/FMANV3L
endif
ifdef CONFIG_FMAN_ARM
ccflags-y += -I$(FMAN)/inc/integrations/LS1043
endif

ccflags-y += -I$(FMAN)/src/inc
ccflags-y += -I$(FMAN)/src/inc/system
ccflags-y += -I$(FMAN)/src/inc/wrapper
ccflags-y += -I$(FMAN)/src/inc/xx
ccflags-y += -I$(srctree)/include/uapi/linux/fmd
ccflags-y += -I$(srctree)/include/uapi/linux/fmd/Peripherals
ccflags-y += -I$(srctree)/include/uapi/linux/fmd/integrations
