adrv9025_drv-y := \
	devices/adrv9025/private/src/adrv9025_bf_analog_orx_mem_map.o \
	devices/adrv9025/private/src/adrv9025_bf_analog_rx_mem_map.o \
	devices/adrv9025/private/src/adrv9025_bf_analog_tx_mem_map.o \
	devices/adrv9025/private/src/adrv9025_bf_core.o \
	devices/adrv9025/private/src/adrv9025_bf_deser.o \
	devices/adrv9025/private/src/adrv9025_bf_hal.o \
	devices/adrv9025/private/src/adrv9025_bf_jesd_common.o \
	devices/adrv9025/private/src/adrv9025_bf_jrx_link.o \
	devices/adrv9025/private/src/adrv9025_bf_jtx_link.o \
	devices/adrv9025/private/src/adrv9025_bf_orx.o \
	devices/adrv9025/private/src/adrv9025_bf_pll_mem_map.o \
	devices/adrv9025/private/src/adrv9025_bf_rx.o \
	devices/adrv9025/private/src/adrv9025_bf_tx.o \
	devices/adrv9025/private/src/adrv9025_bf_txdac_mem_map.o \
	devices/adrv9025/private/src/adrv9025_cals.o \
	devices/adrv9025/private/src/adrv9025_cpu.o \
	devices/adrv9025/private/src/adrv9025_crc32.o \
	devices/adrv9025/private/src/adrv9025_data_interface.o \
	devices/adrv9025/private/src/adrv9025_dfe.o \
	devices/adrv9025/private/src/adrv9025_dynamic_slicer.o \
	devices/adrv9025/private/src/adrv9025_gpio.o \
	devices/adrv9025/private/src/adrv9025_init.o \
	devices/adrv9025/private/src/adrv9025_radioctrl.o \
	devices/adrv9025/private/src/adrv9025_rx.o \
	devices/adrv9025/private/src/adrv9025_shared_resource_manager.o \
	devices/adrv9025/private/src/adrv9025_tx.o \
	devices/adrv9025/private/src/adrv9025_utilities.o \
	devices/adrv9025/public/src/adi_adrv9025_agc.o \
	devices/adrv9025/public/src/adi_adrv9025_arm.o \
	devices/adrv9025/public/src/adi_adrv9025.o \
	devices/adrv9025/public/src/adi_adrv9025_cals.o \
	devices/adrv9025/public/src/adi_adrv9025_cpu.o \
	devices/adrv9025/public/src/adi_adrv9025_data_interface.o \
	devices/adrv9025/public/src/adi_adrv9025_dfe.o \
	devices/adrv9025/public/src/adi_adrv9025_dynamic_slicer.o \
	devices/adrv9025/public/src/adi_adrv9025_gpio.o \
	devices/adrv9025/public/src/adi_adrv9025_hal.o \
	devices/adrv9025/public/src/adi_adrv9025_radioctrl.o \
	devices/adrv9025/public/src/adi_adrv9025_rx.o \
	devices/adrv9025/public/src/adi_adrv9025_tx.o \
	devices/adrv9025/public/src/adi_adrv9025_utilities.o \
	common/adi_logging/adi_common_log.o \
	common/adi_error/adi_common_error.o  \
	common/adi_hal/adi_common_hal.o \
	platforms/linux_platform.o \
	adrv9025.o \
	adrv9025_conv.o

# For now just disable it in this one file
ifdef CONFIG_CC_IS_GCC
CFLAGS_devices/adrv9025/public/src/adi_adrv9025_data_interface.o = -Wno-error=old-style-declaration
endif

ccflags-y += -I$(src)/devices/adrv9025/private/include/ \
	-I$(src)/devices/adrv9025/public/include/  \
	-I$(src)/common/ \
	-I$(src)/common/adi_hal/  \
	-I$(src)/common/adi_error/  \
	-I$(src)/common/adi_libc/  \
	-I$(src)/common/adi_logging/  \
	-I$(src)/platforms/ \
	-DADRV9025_CHANNELID_CHECK=0 \
	-DADI_COMMON_VERBOSE=1 \
	-DADI_ADRV9025_RADIOCTRL_RANGE_CHECK=1 \
	-DADI_ADRV9025_ARM_VERBOSE=0 \
	-Wno-error=missing-prototypes

obj-$(CONFIG_ADRV9025) += adrv9025_drv.o
