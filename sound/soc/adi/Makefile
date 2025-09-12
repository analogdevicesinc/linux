# SPDX-License-Identifier: GPL-2.0-only
snd-soc-adi-axi-i2s-objs := axi-i2s.o
snd-soc-adi-axi-spdif-objs := axi-spdif.o
snd-soc-sc5xx-pcm-objs := sc5xx-pcm.o
snd-soc-sc5xx-sport-$(CONFIG_SND_SC5XX_SPORT) := sc5xx-sport.o
snd-soc-adi-sc5xx-i2s-objs := sc5xx-i2s.o
snd-soc-sc5xx-asoc-card-objs := sc5xx-asoc-card.o
snd-soc-sc5xx-sport-$(CONFIG_SND_SC5XX_SPORT_SHARC) := sc5xx-sport-sharc.o icap/src/icap.o icap/src/platform/icap_linux_kernel_rpmsg.o
snd-soc-sharc-alsa-asoc-card-objs := sharc-alsa-asoc-card.o icap/src/icap.o icap/src/platform/icap_linux_kernel_rpmsg.o

obj-$(CONFIG_SND_SOC_ADI_AXI_I2S) += snd-soc-adi-axi-i2s.o
obj-$(CONFIG_SND_SOC_ADI_AXI_SPDIF) += snd-soc-adi-axi-spdif.o
obj-$(CONFIG_SND_SC5XX_PCM) += snd-soc-sc5xx-pcm.o snd-soc-sc5xx-sport.o
obj-$(CONFIG_SND_SOC_ADI_SC5XX_I2S) += snd-soc-adi-sc5xx-i2s.o
obj-$(CONFIG_SND_SC5XX_MACHINE) += snd-soc-sc5xx-asoc-card.o
obj-$(CONFIG_SND_SC5XX_SHARC_ALSA_CARD) += snd-soc-sharc-alsa-asoc-card.o