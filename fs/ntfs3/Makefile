# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the ntfs3 filesystem support.
#

# Subset of W=1 warnings
subdir-ccflags-y += -Wextra -Wunused -Wno-unused-parameter
subdir-ccflags-y += -Wmissing-declarations
subdir-ccflags-y += -Wmissing-format-attribute
subdir-ccflags-y += -Wmissing-prototypes
subdir-ccflags-y += -Wold-style-definition
subdir-ccflags-y += -Wmissing-include-dirs
condflags := \
	$(call cc-option, -Wunused-but-set-variable)		\
	$(call cc-option, -Wunused-const-variable)		\
	$(call cc-option, -Wpacked-not-aligned)			\
	$(call cc-option, -Wstringop-truncation)		\
	$(call cc-option, -Wmaybe-uninitialized)
subdir-ccflags-y += $(condflags)
# The following turn off the warnings enabled by -Wextra
subdir-ccflags-y += -Wno-missing-field-initializers
subdir-ccflags-y += -Wno-sign-compare
subdir-ccflags-y += -Wno-type-limits
subdir-ccflags-y += -Wno-shift-negative-value

# to check robot warnings
ccflags-y += -Wint-to-pointer-cast \
	$(call cc-option,-Wunused-but-set-variable,-Wunused-const-variable) \
	$(call cc-option,-Wold-style-declaration,-Wout-of-line-declaration)

obj-$(CONFIG_NTFS3_FS) += ntfs3.o

ntfs3-y :=	attrib.o \
		attrlist.o \
		bitfunc.o \
		bitmap.o \
		dir.o \
		fsntfs.o \
		frecord.o \
		file.o \
		fslog.o \
		inode.o \
		index.o \
		lznt.o \
		namei.o \
		record.o \
		run.o \
		super.o \
		upcase.o \
		xattr.o

ntfs3-$(CONFIG_NTFS3_LZX_XPRESS) += $(addprefix lib/,\
		decompress_common.o \
		lzx_decompress.o \
		xpress_decompress.o \
		)