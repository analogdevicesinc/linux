adrv903x_drv-y = \
	devices/adrv903x/private/src/adrv903x_tx.o \
	devices/adrv903x/private/src/adrv903x_reg_addr_macros.o \
	devices/adrv903x/private/src/adrv903x_binloader.o \
	devices/adrv903x/private/src/adrv903x_cpu.o \
	devices/adrv903x/private/src/adrv903x_shared_resource_manager.o \
	devices/adrv903x/private/src/adrv903x_agc.o \
	devices/adrv903x/private/src/adrv903x_rx.o \
	devices/adrv903x/private/src/adrv903x_radioctrl.o \
	devices/adrv903x/private/src/adrv903x_datainterface.o \
	devices/adrv903x/private/src/adrv903x_struct_endian.o \
	devices/adrv903x/private/src/adrv903x_gpio.o \
	devices/adrv903x/private/src/adrv903x_init.o \
	devices/adrv903x/private/bf/adrv903x_bf_serdes_rxdig_phy_regmap_core1p2.o \
	devices/adrv903x/private/bf/adrv903x_bf_actrl_orx_west_regmap.o \
	devices/adrv903x/private/bf/adrv903x_bf_streamproc.o \
	devices/adrv903x/private/bf/adrv903x_bf_rx_ddc.o \
	devices/adrv903x/private/bf/adrv903x_bf_jrx_link.o \
	devices/adrv903x/private/bf/adrv903x_bf_core.o \
	devices/adrv903x/private/bf/adrv903x_bf_orx_dig.o \
	devices/adrv903x/private/bf/adrv903x_bf_rx_dig.o \
	devices/adrv903x/private/bf/adrv903x_bf_tx_datapath.o \
	devices/adrv903x/private/bf/adrv903x_bf_tx_funcs.o \
	devices/adrv903x/private/bf/adrv903x_bf_tdr_dpath_top.o \
	devices/adrv903x/private/bf/adrv903x_bf_pll_mem_map.o \
	devices/adrv903x/private/bf/adrv903x_bf_serdes_rxdig_8pack_regmap_core1p2.o \
	devices/adrv903x/private/bf/adrv903x_bf_serdes_txdig_phy_regmap_core1p2.o \
	devices/adrv903x/private/bf/adrv903x_bf_jtx_link.o \
	devices/adrv903x/private/bf/adrv903x_bf_rx_funcs.o \
	devices/adrv903x/private/bf/adrv903x_bf_jesd_common.o \
	devices/adrv903x/private/bf/adrv903x_bf_tx_dig.o \
	devices/adrv903x/public/src/adi_adrv903x_error.o \
	devices/adrv903x/public/src/adi_adrv903x_tx.o \
	devices/adrv903x/public/src/adi_adrv903x_hal.o \
	devices/adrv903x/public/src/adi_adrv903x_cals.o \
	devices/adrv903x/public/src/adi_adrv903x_gpio.o \
	devices/adrv903x/public/src/adi_adrv903x_radioctrl.o \
	devices/adrv903x/public/src/adi_adrv903x_cpu.o \
	devices/adrv903x/public/src/adi_adrv903x_datainterface.o \
	devices/adrv903x/public/src/adi_adrv903x_agc.o \
	devices/adrv903x/public/src/adi_adrv903x_rx.o \
	devices/adrv903x/public/src/adi_adrv903x_core.o \
	devices/adrv903x/public/src/adi_adrv903x_utilities.o \
	common/adi_logging/adi_common_log.o \
	common/adi_error/adi_common_error.o \
	common/adi_hal/adi_common_hal.o \
	platforms/linux_platform.o \
	platforms/adi_library.o \
	platforms/adi_platform.o \
	initdata.o \
	adrv903x.o \
	adrv903x_conv.o

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

# Suppress frame size warnings for vendor API files with large stack buffers
CFLAGS_devices/adrv903x/public/src/adi_adrv903x_cpu.o := -Wframe-larger-than=2048
CFLAGS_devices/adrv903x/public/src/adi_adrv903x_cals.o := -Wframe-larger-than=2048
CFLAGS_devices/adrv903x/public/src/adi_adrv903x_utilities.o := -Wframe-larger-than=2048

# Suppress format warnings for vendor error handling (uses C standard PRIx32 macros incompatible with printk)
CFLAGS_common/adi_error/adi_common_error.o := -Wno-format

obj-$(CONFIG_ADRV903X) += adrv903x_drv.o

