# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_CRYPTO_DEV_ADI_CRC) += adi_crc.o

obj-$(CONFIG_CRYPTO_DEV_ADI_PKTE) += adi-pkte-crypto.o
adi-pkte-crypto-y := adi-pkte.o
adi-pkte-crypto-$(CONFIG_CRYPTO_DEV_ADI_HASH) += adi-pkte-hash.o
adi-pkte-crypto-$(CONFIG_CRYPTO_DEV_ADI_SKCIPHER) += adi-pkte-skcipher.o
