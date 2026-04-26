# SPDX-License-Identifier: LGPL-2.1 OR BSD-2-Clause
# Copyright (c) 2026 Meta Platforms, Inc. and affiliates.

.PHONY: clean

# Defaults for standalone builds

CLANG ?= clang
BPFTOOL ?= bpftool
LDLIBS ?= -lbpf -lelf -lz -lrt -lpthread -lzstd

ifeq ($(V),1)
Q =
msg =
else
Q ?= @
msg = @printf '  %-8s%s %s%s\n' "$(1)" "$(if $(2), [$(2)])" "$(notdir $(3))" "$(if $(4), $(4))";
endif

IS_LITTLE_ENDIAN = $(shell $(CC) -dM -E - </dev/null | \
			grep 'define __BYTE_ORDER__ __ORDER_LITTLE_ENDIAN__')
BPF_TARGET_ENDIAN ?= $(if $(IS_LITTLE_ENDIAN),--target=bpfel,--target=bpfeb)

LIBARENA=$(abspath .)
BPFDIR=$(abspath $(LIBARENA)/..)

INCLUDE_DIR ?= $(BPFDIR)/tools/include
LIBBPF_INCLUDE ?= $(INCLUDE_DIR)

# Scan src/ and selftests/ to generate the final binaries
LIBARENA_SOURCES = $(wildcard $(LIBARENA)/src/*.bpf.c) $(wildcard $(LIBARENA)/selftests/*.bpf.c)
LIBARENA_OBJECTS = $(notdir $(LIBARENA_SOURCES:.bpf.c=.bpf.o))

INCLUDES = -I$(LIBARENA)/include -I$(BPFDIR)
ifneq ($(INCLUDE_DIR),)
INCLUDES += -I$(INCLUDE_DIR)
endif
ifneq ($(LIBBPF_INCLUDE),)
INCLUDES += -I$(LIBBPF_INCLUDE)
endif

# ENABLE_ATOMICS_TESTS required because we use arena spinlocks
override BPF_CFLAGS += -DENABLE_ATOMICS_TESTS
override BPF_CFLAGS += -O2 -g
override BPF_CFLAGS += -Wno-incompatible-pointer-types-discards-qualifiers
# Required for suppressing harmless vmlinux.h-related warnings.
override BPF_CFLAGS += -Wno-missing-declarations
override BPF_CFLAGS += $(INCLUDES)

CFLAGS = -O2 -no-pie
CFLAGS += $(INCLUDES)

vpath %.bpf.c $(LIBARENA)/src $(LIBARENA)/selftests
vpath %.c $(LIBARENA)/src $(LIBARENA)/selftests

libarena.skel.h: libarena.bpf.o
	$(call msg,GEN-SKEL,libarena,$@)
	$(Q)$(BPFTOOL) gen skeleton $< name "libarena" > $@

libarena.bpf.o: $(LIBARENA_OBJECTS)
	$(call msg,GEN-OBJ,libarena,$@)
	$(Q)$(BPFTOOL) gen object $@ $^

%.bpf.o: %.bpf.c
	$(call msg,CLNG-BPF,libarena,$@)
	$(Q)$(CLANG) $(BPF_CFLAGS) $(BPF_TARGET_ENDIAN) -c $< -o $@

clean:
	$(Q)rm -f *.skel.h *.bpf.o
