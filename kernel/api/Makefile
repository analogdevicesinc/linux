# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Kernel API Specification Framework
#

# Core API specification framework
obj-y				+= kernel_api_spec.o

# Debugfs interface for kernel API specs
obj-$(CONFIG_KAPI_SPEC_DEBUGFS) += kapi_debugfs.o

# KUnit tests
obj-$(CONFIG_KAPI_KUNIT_TEST) += kapi_kunit.o
