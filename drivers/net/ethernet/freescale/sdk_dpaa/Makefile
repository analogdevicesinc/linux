#
# Makefile for the Freescale Ethernet controllers
#
ccflags-y += -DVERSION=\"\"
#
# Include netcomm SW specific definitions
include $(srctree)/drivers/net/ethernet/freescale/sdk_fman/ncsw_config.mk

ccflags-y += -I$(NET_DPA)

obj-$(CONFIG_FSL_SDK_DPAA_ETH) += fsl_mac.o fsl_dpa.o

fsl_dpa-objs += dpaa_ethtool.o dpaa_eth_sysfs.o dpaa_eth.o dpaa_eth_sg.o dpaa_eth_common.o
ifeq ($(CONFIG_FSL_DPAA_DBG_LOOP),y)
fsl_dpa-objs += dpaa_debugfs.o
endif
ifeq ($(CONFIG_FSL_DPAA_1588),y)
fsl_dpa-objs += dpaa_1588.o
endif

ifeq ($(CONFIG_FSL_DPAA_CEETM),y)
ccflags-y += -I$(srctree)/drivers/net/ethernet/freescale/sdk_fman/src/wrapper
fsl_dpa-objs += dpaa_eth_ceetm.o
endif

fsl_mac-objs += mac.o mac-api.o

# Advanced drivers
ifeq ($(CONFIG_FSL_DPAA_ADVANCED_DRIVERS),y)
obj-$(CONFIG_FSL_SDK_DPAA_ETH) += fsl_advanced.o
obj-$(CONFIG_FSL_SDK_DPAA_ETH) += fsl_proxy.o

fsl_advanced-objs += dpaa_eth_base.o
# suport for multiple drivers per kernel module comes in kernel 3.14
# so we are forced to generate several modules for the advanced drivers
fsl_proxy-objs += dpaa_eth_proxy.o

ifeq ($(CONFIG_FSL_DPAA_OFFLINE_PORTS),y)
obj-$(CONFIG_FSL_SDK_DPAA_ETH) += fsl_oh.o

fsl_oh-objs += offline_port.o
endif
endif

# Needed by the tracing framework
CFLAGS_dpaa_eth.o := -I$(src)
