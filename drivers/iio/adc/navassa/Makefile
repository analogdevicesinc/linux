# SPDX-License-Identifier: GPL-2.0

adrv9002_drv-y = adrv9002.o
adrv9002_drv-y += adrv9002_conv.o
adrv9002_drv-$(CONFIG_DEBUG_FS) += adrv9002_debugfs.o
adrv9002_drv-y += platforms/linux_platform.o
adrv9002_drv-y += third_party/jsmn/jsmn.o
adrv9002_drv-y += common/adi_common_log.o
adrv9002_drv-y += common/adi_common_error.o
adrv9002_drv-y += common/adi_common_hal_wrapper.o
adrv9002_drv-y += devices/adrv9001/private/src/adrv9001_validators.o
adrv9002_drv-y += devices/adrv9001/private/src/adrv9001_rx.o
adrv9002_drv-y += devices/adrv9001/private/src/adrv9001_crc32.o
adrv9002_drv-y += devices/adrv9001/private/src/adrv9001_arm.o
adrv9002_drv-y += devices/adrv9001/private/src/adrv9001_bf_hal.o
adrv9002_drv-y += devices/adrv9001/private/src/adrv9001_init.o
adrv9002_drv-y += devices/adrv9001/private/src/adrv9001_gpio.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_common.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_tx.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_arm.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_bbdc.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_gpio.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_auxdac.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_utilities.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_cals.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_auxadc.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_radio.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_mcs.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_ssi.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_rx_gaincontrol.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_stream.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_spi.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_rx.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_orx.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_dpd.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_fh.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_powermanagement.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_powersavingandmonitormode.o
adrv9002_drv-y += devices/adrv9001/public/src/adi_adrv9001_profileutil.o

obj-$(CONFIG_ADRV9001) += adrv9002_drv.o

ccflags-y += -I$(srctree)/$(src)/devices/adrv9001/private/include \
	-I$(srctree)/$(src)/devices/adrv9001/private/include/bitfields/c0 \
	-I$(srctree)/$(src)/devices/adrv9001/public/include  \
	-I$(srctree)/$(src)/common/ \
	-I$(srctree)/$(src)/platforms/ \
	-I$(srctree)/$(src)/third_party/jsmn/ \
	-I$(srctree)/$(src)/third_party/adi_pmag_macros/ \
	-DADI_DYNAMIC_PROFILE_LOAD

ccflags-$(CONFIG_ADRV9001_COMMON_VERBOSE) += -DADI_COMMON_VERBOSE
ccflags-$(CONFIG_ADRV9001_ARM_VERBOSE) += -DADI_ADRV9001_ARM_VERBOSE
ccflags-$(CONFIG_ADRV9001_VALIDATE_PARAMS) += -DADI_VALIDATE_PARAMS
