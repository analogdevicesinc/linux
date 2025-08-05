SRCS = devices/adrv903x/private/src/adrv903x_tx.c \
	devices/adrv903x/private/src/adrv903x_reg_addr_macros.c \
	devices/adrv903x/private/src/adrv903x_binloader.c \
	devices/adrv903x/private/src/adrv903x_cpu.c \
	devices/adrv903x/private/src/adrv903x_shared_resource_manager.c \
	devices/adrv903x/private/src/adrv903x_agc.c \
	devices/adrv903x/private/src/adrv903x_rx.c \
	devices/adrv903x/private/src/adrv903x_radioctrl.c \
	devices/adrv903x/private/src/adrv903x_datainterface.c \
	devices/adrv903x/private/src/adrv903x_struct_endian.c \
	devices/adrv903x/private/src/adrv903x_gpio.c \
	devices/adrv903x/private/src/adrv903x_init.c \
	devices/adrv903x/private/bf/adrv903x_bf_serdes_rxdig_phy_regmap_core1p2.c \
	devices/adrv903x/private/bf/adrv903x_bf_actrl_orx_west_regmap.c \
	devices/adrv903x/private/bf/adrv903x_bf_streamproc.c \
	devices/adrv903x/private/bf/adrv903x_bf_rx_ddc.c \
	devices/adrv903x/private/bf/adrv903x_bf_jrx_link.c \
	devices/adrv903x/private/bf/adrv903x_bf_core.c \
	devices/adrv903x/private/bf/adrv903x_bf_orx_dig.c \
	devices/adrv903x/private/bf/adrv903x_bf_rx_dig.c \
	devices/adrv903x/private/bf/adrv903x_bf_tx_datapath.c \
	devices/adrv903x/private/bf/adrv903x_bf_tx_funcs.c \
	devices/adrv903x/private/bf/adrv903x_bf_tdr_dpath_top.c \
	devices/adrv903x/private/bf/adrv903x_bf_pll_mem_map.c \
	devices/adrv903x/private/bf/adrv903x_bf_serdes_rxdig_8pack_regmap_core1p2.c \
	devices/adrv903x/private/bf/adrv903x_bf_serdes_txdig_phy_regmap_core1p2.c \
	devices/adrv903x/private/bf/adrv903x_bf_jtx_link.c \
	devices/adrv903x/private/bf/adrv903x_bf_rx_funcs.c \
	devices/adrv903x/private/bf/adrv903x_bf_jesd_common.c \
	devices/adrv903x/private/bf/adrv903x_bf_tx_dig.c \
	devices/adrv903x/public/src/adi_adrv903x_error.c \
	devices/adrv903x/public/src/adi_adrv903x_tx.c \
	devices/adrv903x/public/src/adi_adrv903x_hal.c \
	devices/adrv903x/public/src/adi_adrv903x_cals.c \
	devices/adrv903x/public/src/adi_adrv903x_gpio.c \
	devices/adrv903x/public/src/adi_adrv903x_radioctrl.c \
	devices/adrv903x/public/src/adi_adrv903x_cpu.c \
	devices/adrv903x/public/src/adi_adrv903x_datainterface.c \
	devices/adrv903x/public/src/adi_adrv903x_agc.c \
	devices/adrv903x/public/src/adi_adrv903x_rx.c \
	devices/adrv903x/public/src/adi_adrv903x_core.c \
	devices/adrv903x/public/src/adi_adrv903x_utilities.c \
	common/adi_logging/adi_common_log.c \
	common/adi_error/adi_common_error.c \
	common/adi_hal/adi_common_hal.c \
	platforms/linux_platform.c \
	platforms/adi_library.c \
	platforms/adi_platform.c \
	initdata.c\
	adrv903x.c \
	adrv903x_conv.c

ccflags-y += -I$(srctree)/$(src)/devices/adrv903x/private/include/ \
	-I$(srctree)/$(src)/devices/adrv903x/public/include/  \
	-I$(srctree)/$(src)/devices/adrv903x/multiver/  \
	-I$(srctree)/$(src)/common/ \
	-I$(srctree)/$(src)/common/adi_libc/  \
	-I$(srctree)/$(src)/common/adi_hal/  \
	-I$(srctree)/$(src)/common/adi_error/  \
	-I$(srctree)/$(src)/common/adi_logging/  \
	-I$(srctree)/$(src)/platforms/ \
	-DADI_COMMON_VERBOSE=1 \
	-DADRV903X_CHANNELID_CHECK=0 \
	-DADI_ADRV903X_RADIOCTRL_RANGE_CHECK=1 \
	-DADRV903X_BITFIELD_ADDR_CHECK=0 \
	-DADRV903X_BITFIELD_NULL_CHECK=0 \
	-DADRV903X_BITFIELD_VALUE_CHECK=0 \
	-Wno-error=missing-prototypes

adrv903x_drv-y := $(SRCS:.c=.o)
obj-$(CONFIG_ADRV903X) += adrv903x_drv.o

