SRCS = 	devices/adrv9025/private/src/adrv9025_bf_analog_orx_mem_map.c \
	devices/adrv9025/private/src/adrv9025_bf_analog_rx_mem_map.c \
	devices/adrv9025/private/src/adrv9025_bf_analog_tx_mem_map.c \
	devices/adrv9025/private/src/adrv9025_bf_core.c \
	devices/adrv9025/private/src/adrv9025_bf_deser.c \
	devices/adrv9025/private/src/adrv9025_bf_hal.c \
	devices/adrv9025/private/src/adrv9025_bf_jesd_common.c \
	devices/adrv9025/private/src/adrv9025_bf_jrx_link.c \
	devices/adrv9025/private/src/adrv9025_bf_jtx_link.c \
	devices/adrv9025/private/src/adrv9025_bf_orx.c \
	devices/adrv9025/private/src/adrv9025_bf_pll_mem_map.c \
	devices/adrv9025/private/src/adrv9025_bf_rx.c \
	devices/adrv9025/private/src/adrv9025_bf_tx.c \
	devices/adrv9025/private/src/adrv9025_bf_txdac_mem_map.c \
	devices/adrv9025/private/src/adrv9025_cals.c \
	devices/adrv9025/private/src/adrv9025_cpu.c \
	devices/adrv9025/private/src/adrv9025_crc32.c \
	devices/adrv9025/private/src/adrv9025_data_interface.c \
	devices/adrv9025/private/src/adrv9025_gpio.c \
	devices/adrv9025/private/src/adrv9025_init.c \
	devices/adrv9025/private/src/adrv9025_radioctrl.c \
	devices/adrv9025/private/src/adrv9025_rx.c \
	devices/adrv9025/private/src/adrv9025_shared_resource_manager.c \
	devices/adrv9025/private/src/adrv9025_tx.c \
	devices/adrv9025/private/src/adrv9025_utilities.c \
	devices/adrv9025/public/src/adi_adrv9025_agc.c \
	devices/adrv9025/public/src/adi_adrv9025_arm.c \
	devices/adrv9025/public/src/adi_adrv9025.c \
	devices/adrv9025/public/src/adi_adrv9025_cals.c \
	devices/adrv9025/public/src/adi_adrv9025_cpu.c \
	devices/adrv9025/public/src/adi_adrv9025_data_interface.c \
	devices/adrv9025/public/src/adi_adrv9025_gpio.c \
	devices/adrv9025/public/src/adi_adrv9025_hal.c \
	devices/adrv9025/public/src/adi_adrv9025_radioctrl.c \
	devices/adrv9025/public/src/adi_adrv9025_rx.c \
	devices/adrv9025/public/src/adi_adrv9025_tx.c \
	devices/adrv9025/public/src/adi_adrv9025_utilities.c \
	common/adi_logging/adi_common_log.c \
	common/adi_error/adi_common_error.c  \
	common/adi_hal/adi_common_hal.c \
	platforms/linux_platform.c \
	adrv9025.c \
	adrv9025_conv.c

# Avoid FP operations and data sytpes - Remove DPD *_dfe.c files
#SRCS := $(filter-out /devices/adrv9025/private/src/adrv9025_dfe.c /devices/adrv9025/public/src/adi_adrv9025_dfe.c, $(SRCS))

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
	-DADI_ADRV9025_ARM_VERBOSE=0

adrv9025_drv-y := $(SRCS:.c=.o)
obj-$(CONFIG_ADRV9025) += adrv9025_drv.o
