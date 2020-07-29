SRCS = \
	adrv9002_init_data.c \
	platforms/linux_platform.c \
	adrv9002_conv.c \
	third_party/jsmn/jsmn.c \
	common/adi_logging/adi_common_log.c \
	common/adi_error/adi_common_error.c \
	common/adi_common.c \
	common/adi_hal/adi_common_hal.c \
	adrv9002.c \
	devices/adrv9001/private/src/adrv9001_validators.c \
	devices/adrv9001/private/src/adrv9001_rx.c \
	devices/adrv9001/private/src/adrv9001_crc32.c \
	devices/adrv9001/private/src/adrv9001_utilities.c \
	devices/adrv9001/private/src/adrv9001_arm.c \
	devices/adrv9001/private/src/adrv9001_bf_hal.c \
	devices/adrv9001/private/src/adrv9001_init.c \
	devices/adrv9001/private/src/adrv9001_gpio.c \
	devices/adrv9001/private/src/adrv9001_powermanagement.c \
	devices/adrv9001/public/src/adi_adrv9001_tx.c \
	devices/adrv9001/public/src/adi_adrv9001_arm.c \
	devices/adrv9001/public/src/adi_adrv9001_bbdc.c \
	devices/adrv9001/public/src/adi_adrv9001_gpio.c \
	devices/adrv9001/public/src/adi_adrv9001_auxdac.c \
	devices/adrv9001/public/src/adi_adrv9001_utilities.c \
	devices/adrv9001/public/src/adi_adrv9001_cals.c \
	devices/adrv9001/public/src/adi_adrv9001_auxadc.c \
	devices/adrv9001/public/src/adi_adrv9001_radio.c \
	devices/adrv9001/public/src/adi_adrv9001_mcs.c \
	devices/adrv9001/public/src/adi_adrv9001_ssi.c \
	devices/adrv9001/public/src/adi_adrv9001_rx_gaincontrol.c \
	devices/adrv9001/public/src/adi_adrv9001_stream.c \
	devices/adrv9001/public/src/adi_adrv9001_spi.c \
	devices/adrv9001/public/src/adi_adrv9001_rx.c \
	devices/adrv9001/public/src/adi_adrv9001.c \
	devices/adrv9001/public/src/adi_adrv9001_orx.c \
	devices/adrv9001/public/src/adi_adrv9001_dpd.c

ccflags-y += -I$(src)/devices/adrv9001/private/include \
	-I$(src)/devices/adrv9001/private/include/bitfields \
	-I$(src)/devices/adrv9001/public/include  \
	-I$(src)/boards/adrv9001/private/include  \
	-I$(src)/boards/adrv9001/public/include/  \
	-I$(src)/common/ \
	-I$(src)/common/adi_hal/  \
	-I$(src)/common/adi_error/  \
	-I$(src)/common/adi_libc/  \
	-I$(src)/common/adi_logging/  \
	-I$(src)/platforms/ \
	-I$(src)/third_party/jsmn/ \
	-I$(src)/third_party/adi_pmag_macros/ \
	-DSI_REV_B0 \
	-DADI_DYNAMIC_PROFILE_LOAD \
	-DADI_COMMON_VERBOSE=1 \
	-DADI_ADRV9001_ARM_VERBOSE \
	-DADI_VALIDATE_PARAMS


obj-y = $(SRCS:.c=.o)
