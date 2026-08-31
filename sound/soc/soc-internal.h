/* SPDX-License-Identifier: GPL-2.0-only
 *
 * soc-internal.h
 *
 * Copyright (c) 2026 Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 */
#ifndef __SOC_INTERNAL_H
#define __SOC_INTERNAL_H

/*
 * This header is for ALSA SoC Framework internal, not for Vender drivers.
 * The ALSA SoC functions for Vender drivers are defined in linux/include/sound/xxx.h
 * as snd_soc_xxx();
 */

/*
 * In soc-core
 */
char *snd_soc_fmt_single_name(struct device *dev, int *id);
char *snd_soc_fmt_multiple_name(struct device *dev, struct snd_soc_dai_driver *dai_drv);
int snd_soc_add_controls(struct snd_card *card, struct device *dev,
			 const struct snd_kcontrol_new *controls, int num_controls,
			 const char *prefix, void *data);

#endif /* __SOC_INTERNAL_H */
