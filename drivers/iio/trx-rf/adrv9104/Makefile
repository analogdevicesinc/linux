# SPDX-License-Identifier: GPL-2.0

adrv9104_drv-y = adrv9104-linux.o
adrv9104_drv-y += common/adi_common_log.o
adrv9104_drv-y += common/adi_common_error.o
adrv9104_drv-y += common/adi_common_hal_wrapper.o
adrv9104_drv-y += devices/adrv910x/private/src/adrv910x_arm.o
adrv9104_drv-y += devices/adrv910x/private/src/adrv910x_bf_hal.o
adrv9104_drv-y += devices/adrv910x/private/src/adrv910x_init.o
adrv9104_drv-y += devices/adrv910x/private/src/adrv910x_rx.o
adrv9104_drv-y += devices/adrv910x/private/src/adrv910x_validators.o
adrv9104_drv-y += devices/adrv910x/public/src/adi_adrv910x_arm.o
adrv9104_drv-y += devices/adrv910x/public/src/adi_adrv910x_auxadc.o
adrv9104_drv-y += devices/adrv910x/public/src/adi_adrv910x_auxdac.o
adrv9104_drv-y += devices/adrv910x/public/src/adi_adrv910x.o
adrv9104_drv-y += devices/adrv910x/public/src/adi_adrv910x_cals.o
adrv9104_drv-y += devices/adrv910x/public/src/adi_adrv910x_common.o
adrv9104_drv-y += devices/adrv910x/public/src/adi_adrv910x_dpd.o
adrv9104_drv-y += devices/adrv910x/public/src/adi_adrv910x_gpio.o
adrv9104_drv-y += devices/adrv910x/public/src/adi_adrv910x_powersavingandmonitormode.o
adrv9104_drv-y += devices/adrv910x/public/src/adi_adrv910x_radio.o
adrv9104_drv-y += devices/adrv910x/public/src/adi_adrv910x_rx.o
adrv9104_drv-y += devices/adrv910x/public/src/adi_adrv910x_spi.o
adrv9104_drv-y += devices/adrv910x/public/src/adi_adrv910x_ssi.o
adrv9104_drv-y += devices/adrv910x/public/src/adi_adrv910x_stream.o
adrv9104_drv-y += devices/adrv910x/public/src/adi_adrv910x_tx.o

obj-$(CONFIG_ADRV9104) += adrv9104_drv.o

ccflags-y += -I$(src)/devices/adrv910x/private/include \
	-I$(src)/devices/adrv910x/private/include/bitfields/c0 \
	-I$(src)/devices/adrv910x/public/include  \
	-I$(src)/common/ \

ccflags-$(CONFIG_ADRV9104_COMMON_VERBOSE) += -DADI_COMMON_VERBOSE
ccflags-$(CONFIG_ADRV9104_ARM_VERBOSE) += -DADI_ADRV910X_ARM_VERBOSE
ccflags-$(CONFIG_ADRV9104_VALIDATE_PARAMS) += -DADI_VALIDATE_PARAMS
