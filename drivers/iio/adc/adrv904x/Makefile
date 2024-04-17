SRCS = 	devices/adrv904x/public/src/adi_adrv904x_cals.c \
	devices/adrv904x/public/src/adi_adrv904x_cpu.c \
	devices/adrv904x/public/src/adi_adrv904x_core.c \
	devices/adrv904x/public/src/adi_adrv904x_datainterface.c \
	devices/adrv904x/public/src/adi_adrv904x_rx.c \
	devices/adrv904x/public/src/adi_adrv904x_hal.c \
	devices/adrv904x/public/src/adi_adrv904x_dfe_dpd.c \
	devices/adrv904x/public/src/adi_adrv904x_dfe_clgc.c \
	devices/adrv904x/public/src/adi_adrv904x_error.c \
	devices/adrv904x/public/src/adi_adrv904x_utilities.c \
	devices/adrv904x/public/src/adi_adrv904x_tx.c \
	devices/adrv904x/public/src/adi_adrv904x_dfe_cpu.c \
	devices/adrv904x/public/src/adi_adrv904x_gpio.c \
	devices/adrv904x/public/src/adi_adrv904x_dfe_cfr.c \
	devices/adrv904x/public/src/adi_adrv904x_radioctrl.c \
	devices/adrv904x/public/src/adi_adrv904x_dfe_utilities.c \
	devices/adrv904x/public/src/adi_adrv904x_dfe_sbet.c \
	devices/adrv904x/public/src/adi_adrv904x_dfe_vswr.c \
	devices/adrv904x/public/src/adi_adrv904x_agc.c \
	devices/adrv904x/private/src/adrv904x_binloader.c \
	devices/adrv904x/private/src/adrv904x_cpu.c \
	devices/adrv904x/private/src/adrv904x_dfe_dpd.c \
	devices/adrv904x/private/src/adrv904x_rx.c \
	devices/adrv904x/private/src/adrv904x_agc.c \
	devices/adrv904x/private/src/adrv904x_init.c \
	devices/adrv904x/private/src/adrv904x_dfe_cfr.c \
	devices/adrv904x/private/src/adrv904x_dfe_cpu.c \
	devices/adrv904x/private/src/adrv904x_reg_addr_macros.c \
	devices/adrv904x/private/src/adrv904x_carrier_reconfigure.c \
	devices/adrv904x/private/src/adrv904x_gpio.c \
	devices/adrv904x/private/src/adrv904x_tx.c \
	devices/adrv904x/private/src/adrv904x_struct_endian.c \
	devices/adrv904x/private/src/adrv904x_dfe_vswr.c \
	devices/adrv904x/private/src/adrv904x_radioctrl.c \
	devices/adrv904x/private/src/adrv904x_datainterface.c \
	devices/adrv904x/private/src/adrv904x_shared_resource_manager.c \
	devices/adrv904x/private/bf/adrv904x_bf_rx_ddc.c \
	devices/adrv904x/private/bf/adrv904x_bf_tx_funcs.c \
	devices/adrv904x/private/bf/adrv904x_bf_rx_dig.c \
	devices/adrv904x/private/bf/adrv904x_bf_sequencer.c \
	devices/adrv904x/private/bf/adrv904x_bf_tx_band_duc.c \
	devices/adrv904x/private/bf/adrv904x_bf_tx_dig.c \
	devices/adrv904x/private/bf/adrv904x_bf_orx_dig.c \
	devices/adrv904x/private/bf/adrv904x_bf_cduc_funcs.c \
	devices/adrv904x/private/bf/adrv904x_bf_streamproc.c \
	devices/adrv904x/private/bf/adrv904x_bf_tx_dpd_act_mmr.c \
	devices/adrv904x/private/bf/adrv904x_bf_serdes_rxdig_8pack_regmap_core1p2.c \
	devices/adrv904x/private/bf/adrv904x_bf_tx_datapath.c \
	devices/adrv904x/private/bf/adrv904x_bf_streamproc_channel.c \
	devices/adrv904x/private/bf/adrv904x_bf_tx_dfe_dig_regs.c \
	devices/adrv904x/private/bf/adrv904x_bf_cddc_funcs.c \
	devices/adrv904x/private/bf/adrv904x_bf_serdes_txdig_phy_regmap_core1p2.c \
	devices/adrv904x/private/bf/adrv904x_bf_jesd_common.c \
	devices/adrv904x/private/bf/adrv904x_bf_tx_cfr_reg.c \
	devices/adrv904x/private/bf/adrv904x_bf_jrx_link.c \
	devices/adrv904x/private/bf/adrv904x_bf_pll_mem_map.c \
	devices/adrv904x/private/bf/adrv904x_bf_tdr_dpath_top.c \
	devices/adrv904x/private/bf/adrv904x_bf_core.c \
	devices/adrv904x/private/bf/adrv904x_bf_cduc_hb_dpath.c \
	devices/adrv904x/private/bf/adrv904x_bf_rx_funcs.c \
	devices/adrv904x/private/bf/adrv904x_bf_cddc_hb_dpath.c \
	devices/adrv904x/private/bf/adrv904x_bf_actrl_orx_west_regmap.c \
	devices/adrv904x/private/bf/adrv904x_bf_cap_buf_mmr.c \
	devices/adrv904x/private/bf/adrv904x_bf_jtx_link.c \
	common/adi_logging/adi_common_log.c \
	common/adi_error/adi_common_error.c \
	common/adi_hal/adi_common_hal.c \
	platforms/linux_platform.c \
	platforms/adi_library.c \
	initdata.c\
	adrv904x.c \
	adrv904x_conv.c

ccflags-y += -I$(srctree)/$(src)/devices/adrv904x/private/include/ \
	-I$(srctree)/$(src)/devices/adrv904x/public/include/  \
	-I$(srctree)/$(src)/devices/adrv904x/multiver/  \
	-I$(srctree)/$(src)/common/ \
	-I$(srctree)/$(src)/common/adi_libc/  \
	-I$(srctree)/$(src)/common/adi_hal/  \
	-I$(srctree)/$(src)/common/adi_error/  \
	-I$(srctree)/$(src)/common/adi_logging/  \
	-I$(srctree)/$(src)/platforms/ \
	-DADI_COMMON_VERBOSE=1 \
	-DADRV904X_CHANNELID_CHECK=0 \
	-DADI_ADRV904X_RADIOCTRL_RANGE_CHECK=1 \
	-DADI_ADRV904X_ARM_VERBOSE=0 \
	-DADRV904X_BITFIELD_ADDR_CHECK=0 \
	-DADRV904X_BITFIELD_NULL_CHECK=0 \
	-DADRV904X_BITFIELD_VALUE_CHECK=0 \
	-DADI_ENABLE_DELAY_MATCHING_LOG_PRINTS=0

adrv904x_drv-y := $(SRCS:.c=.o)
obj-$(CONFIG_ADRV904X) += adrv904x_drv.o
