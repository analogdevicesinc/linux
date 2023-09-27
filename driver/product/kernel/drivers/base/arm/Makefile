# SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
#
# (C) COPYRIGHT 2021-2023 ARM Limited. All rights reserved.
#
# This program is free software and is provided to you under the terms of the
# GNU General Public License version 2 as published by the Free Software
# Foundation, and any use by you of this program is subject to the terms
# of such GNU license.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, you can access it online at
# http://www.gnu.org/licenses/gpl-2.0.html.
#
#

#
# Paths
#
KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
KDIR ?= $(KERNEL_SRC)
M ?= $(shell pwd)

ifeq ($(KDIR),)
    $(error Must specify KDIR to point to the kernel to target))
endif

CONFIGS :=

ifeq ($(MALI_KCONFIG_EXT_PREFIX),)
    #
    # Default configuration values
    #
    CONFIG_MALI_BASE_MODULES ?= n

    ifeq ($(CONFIG_MALI_BASE_MODULES),y)
        CONFIG_MALI_CSF_SUPPORT ?= n

        ifneq ($(CONFIG_DMA_SHARED_BUFFER),n)
            CONFIG_DMA_SHARED_BUFFER_TEST_EXPORTER ?= y
        else
            # Prevent misuse when CONFIG_DMA_SHARED_BUFFER=n
            CONFIG_DMA_SHARED_BUFFER_TEST_EXPORTER = n
        endif

        CONFIG_MALI_MEMORY_GROUP_MANAGER ?= y

        ifneq ($(CONFIG_MALI_CSF_SUPPORT), n)
            CONFIG_MALI_PROTECTED_MEMORY_ALLOCATOR ?= y
        endif

    else
        # Prevent misuse when CONFIG_MALI_BASE_MODULES=n
        CONFIG_DMA_SHARED_BUFFER_TEST_EXPORTER = n
        CONFIG_MALI_MEMORY_GROUP_MANAGER = n
        CONFIG_MALI_PROTECTED_MEMORY_ALLOCATOR = n

    endif

    CONFIGS += \
        CONFIG_MALI_BASE_MODULES \
        CONFIG_MALI_CSF_SUPPORT \
        CONFIG_DMA_SHARED_BUFFER_TEST_EXPORTER \
        CONFIG_MALI_MEMORY_GROUP_MANAGER \
        CONFIG_MALI_PROTECTED_MEMORY_ALLOCATOR \

endif

#
# MAKE_ARGS to pass the custom CONFIGs on out-of-tree build
#
# Generate the list of CONFIGs and values.
# $(value config) is the name of the CONFIG option.
# $(value $(value config)) is its value (y, m).
# When the CONFIG is not set to y or m, it defaults to n.
MAKE_ARGS := $(foreach config,$(CONFIGS), \
                    $(if $(filter y m,$(value $(value config))), \
                        $(value config)=$(value $(value config)), \
                        $(value config)=n))

#
# EXTRA_CFLAGS to define the custom CONFIGs on out-of-tree build
#
# Generate the list of CONFIGs defines with values from CONFIGS.
# $(value config) is the name of the CONFIG option.
# When set to y or m, the CONFIG gets defined to 1.
EXTRA_CFLAGS := $(foreach config,$(CONFIGS), \
                    $(if $(filter y m,$(value $(value config))), \
                        -D$(value config)=1))

CFLAGS_MODULE += -Wall -Werror

ifeq ($(CONFIG_GCOV_KERNEL), y)
    CFLAGS_MODULE += $(call cc-option, -ftest-coverage)
    CFLAGS_MODULE += $(call cc-option, -fprofile-arcs)
    EXTRA_CFLAGS += -DGCOV_PROFILE=1
endif

ifeq ($(CONFIG_MALI_KCOV),y)
    CFLAGS_MODULE += $(call cc-option, -fsanitize-coverage=trace-cmp)
    EXTRA_CFLAGS += -DKCOV=1
    EXTRA_CFLAGS += -DKCOV_ENABLE_COMPARISONS=1
endif

# The following were added to align with W=1 in scripts/Makefile.extrawarn
# from the Linux source tree (v5.18.14)
CFLAGS_MODULE += -Wextra -Wunused -Wno-unused-parameter
CFLAGS_MODULE += -Wmissing-declarations
CFLAGS_MODULE += -Wmissing-format-attribute
CFLAGS_MODULE += -Wmissing-prototypes
CFLAGS_MODULE += -Wold-style-definition
# The -Wmissing-include-dirs cannot be enabled as the path to some of the
# included directories change depending on whether it is an in-tree or
# out-of-tree build.
CFLAGS_MODULE += $(call cc-option, -Wunused-but-set-variable)
CFLAGS_MODULE += $(call cc-option, -Wunused-const-variable)
CFLAGS_MODULE += $(call cc-option, -Wpacked-not-aligned)
CFLAGS_MODULE += $(call cc-option, -Wstringop-truncation)
# The following turn off the warnings enabled by -Wextra
CFLAGS_MODULE += -Wno-sign-compare
CFLAGS_MODULE += -Wno-shift-negative-value
# This flag is needed to avoid build errors on older kernels
CFLAGS_MODULE += $(call cc-option, -Wno-cast-function-type)

KBUILD_CPPFLAGS += -DKBUILD_EXTRA_WARN1

# The following were added to align with W=2 in scripts/Makefile.extrawarn
# from the Linux source tree (v5.18.14)
CFLAGS_MODULE += -Wdisabled-optimization
# The -Wshadow flag cannot be enabled unless upstream kernels are
# patched to fix redefinitions of certain built-in functions and
# global variables.
CFLAGS_MODULE += $(call cc-option, -Wlogical-op)
CFLAGS_MODULE += -Wmissing-field-initializers
# -Wtype-limits must be disabled due to build failures on kernel 5.x
CFLAGS_MODULE += -Wno-type-limits
CFLAGS_MODULE += $(call cc-option, -Wmaybe-uninitialized)
CFLAGS_MODULE += $(call cc-option, -Wunused-macros)

KBUILD_CPPFLAGS += -DKBUILD_EXTRA_WARN2

# This warning is disabled to avoid build failures in some kernel versions
CFLAGS_MODULE += -Wno-ignored-qualifiers

all:
	$(MAKE) -C $(KDIR) M=$(M) $(MAKE_ARGS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" modules

modules_install:
	$(MAKE) -C $(KDIR) M=$(M) $(MAKE_ARGS) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(M) $(MAKE_ARGS) clean
