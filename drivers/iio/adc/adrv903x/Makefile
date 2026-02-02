adrv903x_drv-y = adrv903x.o
adrv903x_drv-y += adrv903x_conv.o adrv903x_ramc.o
adrv903x_drv-y += devices/adrv903x/private/src/adrv903x_tx.o
adrv903x_drv-y += devices/adrv903x/private/src/adrv903x_reg_addr_macros.o
adrv903x_drv-y += devices/adrv903x/private/src/adrv903x_binloader.o
adrv903x_drv-y += devices/adrv903x/private/src/adrv903x_cpu.o
adrv903x_drv-y += devices/adrv903x/private/src/adrv903x_shared_resource_manager.o
adrv903x_drv-y += devices/adrv903x/private/src/adrv903x_agc.o
adrv903x_drv-y += devices/adrv903x/private/src/adrv903x_rx.o
adrv903x_drv-y += devices/adrv903x/private/src/adrv903x_radioctrl.o
adrv903x_drv-y += devices/adrv903x/private/src/adrv903x_datainterface.o
adrv903x_drv-y += devices/adrv903x/private/src/adrv903x_struct_endian.o
adrv903x_drv-y += devices/adrv903x/private/src/adrv903x_gpio.o
adrv903x_drv-y += devices/adrv903x/private/src/adrv903x_init.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_serdes_rxdig_phy_regmap_core1p2.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_actrl_orx_west_regmap.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_streamproc.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_rx_ddc.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_jrx_link.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_core.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_orx_dig.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_rx_dig.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_tx_datapath.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_tx_funcs.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_tdr_dpath_top.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_pll_mem_map.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_serdes_rxdig_8pack_regmap_core1p2.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_serdes_txdig_phy_regmap_core1p2.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_jtx_link.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_rx_funcs.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_jesd_common.o
adrv903x_drv-y += devices/adrv903x/private/bf/adrv903x_bf_tx_dig.o
adrv903x_drv-y += devices/adrv903x/public/src/adi_adrv903x_error.o
adrv903x_drv-y += devices/adrv903x/public/src/adi_adrv903x_tx.o
adrv903x_drv-y += devices/adrv903x/public/src/adi_adrv903x_hal.o
adrv903x_drv-y += devices/adrv903x/public/src/adi_adrv903x_cals.o
adrv903x_drv-y += devices/adrv903x/public/src/adi_adrv903x_gpio.o
adrv903x_drv-y += devices/adrv903x/public/src/adi_adrv903x_radioctrl.o
adrv903x_drv-y += devices/adrv903x/public/src/adi_adrv903x_cpu.o
adrv903x_drv-y += devices/adrv903x/public/src/adi_adrv903x_datainterface.o
adrv903x_drv-y += devices/adrv903x/public/src/adi_adrv903x_agc.o
adrv903x_drv-y += devices/adrv903x/public/src/adi_adrv903x_rx.o
adrv903x_drv-y += devices/adrv903x/public/src/adi_adrv903x_core.o
adrv903x_drv-y += devices/adrv903x/public/src/adi_adrv903x_utilities.o
adrv903x_drv-y += common/adi_logging/adi_common_log.o
adrv903x_drv-y += common/adi_error/adi_common_error.o
adrv903x_drv-y += common/adi_hal/adi_common_hal.o
adrv903x_drv-y += platforms/linux_platform.o
adrv903x_drv-y += platforms/adi_library.o
adrv903x_drv-y += platforms/adi_platform.o
adrv903x_drv-y += initdata.o

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

obj-$(CONFIG_ADRV903X) += adrv903x_drv.o