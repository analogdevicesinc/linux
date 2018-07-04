# SPDX-License-Identifier: GPL-2.0
#
# Makefile for key management
#

#
# Core
#
obj-y := \
	gc.o \
	key.o \
	keyring.o \
	keyctl.o \
	permission.o \
	process_keys.o \
	request_key.o \
	request_key_auth.o \
	user_defined.o
compat-obj-$(CONFIG_KEY_DH_OPERATIONS) += compat_dh.o
obj-$(CONFIG_COMPAT) += compat.o $(compat-obj-y)
obj-$(CONFIG_PROC_FS) += proc.o
obj-$(CONFIG_SYSCTL) += sysctl.o
obj-$(CONFIG_PERSISTENT_KEYRINGS) += persistent.o
obj-$(CONFIG_KEY_DH_OPERATIONS) += dh.o
obj-$(CONFIG_ASYMMETRIC_KEY_TYPE) += keyctl_pkey.o

#
# Key types
#
obj-$(CONFIG_BIG_KEYS) += big_key.o
obj-$(CONFIG_TRUSTED_KEYS) += trusted-keys/
CFLAGS_secure_key.o += -I$(obj)/../../drivers/crypto/caam/
CFLAGS_securekey_desc.o += -I$(obj)/../../drivers/crypto/caam/
obj-$(CONFIG_SECURE_KEYS) += securekey.o
securekey-y := securekey_desc.o \
	       secure_key.o
obj-$(CONFIG_ENCRYPTED_KEYS) += encrypted-keys/
