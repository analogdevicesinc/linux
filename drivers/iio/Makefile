# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the industrial I/O core.
#

obj-$(CONFIG_IIO) += industrialio.o
industrialio-y := industrialio-core.o industrialio-event.o inkern.o
industrialio-$(CONFIG_IIO_BUFFER) += industrialio-buffer.o
industrialio-$(CONFIG_IIO_TRIGGER) += industrialio-trigger.o
industrialio-$(CONFIG_ACPI) += industrialio-acpi.o

obj-$(CONFIG_IIO_CONFIGFS) += industrialio-configfs.o
obj-$(CONFIG_IIO_GTS_HELPER) += industrialio-gts-helper.o
obj-$(CONFIG_IIO_SW_DEVICE) += industrialio-sw-device.o
obj-$(CONFIG_IIO_SW_TRIGGER) += industrialio-sw-trigger.o
obj-$(CONFIG_IIO_TRIGGERED_EVENT) += industrialio-triggered-event.o
obj-$(CONFIG_IIO_BACKEND) += industrialio-backend.o

obj-y += accel/
obj-y += adc/
obj-y += addac/
obj-y += afe/
obj-y += amplifiers/
obj-y += beamformer/
obj-y += buffer/
obj-y += cdc/
obj-y += chemical/
obj-y += common/
obj-y += dac/
obj-y += dummy/
obj-y += gyro/
obj-y += filter/
obj-y += frequency/
obj-y += health/
obj-y += humidity/
obj-y += imu/
obj-y += jesd204/
obj-y += light/
obj-y += logic/
obj-y += magnetometer/
obj-y += multiplexer/
obj-y += regmap/
obj-y += orientation/
obj-y += position/
obj-y += potentiometer/
obj-y += potentiostat/
obj-y += pressure/
obj-y += proximity/
obj-y += resolver/
obj-y += temperature/
obj-y += test/
obj-y += trigger/

