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
 * REMOVE ME
 * Temporary definition
 */
void snd_soc_card_debugfs_init(struct snd_soc_card *card);
void snd_soc_card_debugfs_cleanup(struct snd_soc_card *card);
void snd_soc_card_resume_init(struct snd_soc_card *card);
void snd_soc_card_fill_dummy_dai(struct snd_soc_card *card);
int snd_soc_card_init_pcm_runtime(struct snd_soc_card *card,
				  struct snd_soc_pcm_runtime *rtd);
void snd_soc_card_link_dais_remove(struct snd_soc_card *card);
int snd_soc_card_link_dais_probe(struct snd_soc_card *card);
void snd_soc_card_link_components_remove(struct snd_soc_card *card);
int snd_soc_card_link_components_probe(struct snd_soc_card *card);
void snd_soc_card_aux_unbind(struct snd_soc_card *card);
int snd_soc_card_aux_bind(struct snd_soc_card *card);

/*
 * In soc-core
 */
extern struct mutex client_mutex;

char *snd_soc_fmt_single_name(struct device *dev, int *id);
char *snd_soc_fmt_multiple_name(struct device *dev, struct snd_soc_dai_driver *dai_drv);
int snd_soc_add_controls(struct snd_card *card, struct device *dev,
			 const struct snd_kcontrol_new *controls, int num_controls,
			 const char *prefix, void *data);
struct snd_soc_component *snd_soc_find_component(const struct snd_soc_dai_link_component *dlc);
#ifdef CONFIG_PM_SLEEP
void snd_soc_playback_digital_mute(struct snd_soc_card *card, int mute);
void snd_soc_dapm_suspend_resume(struct snd_soc_card *card, int event);
#endif

/*
 * In soc-dai
 */
void snd_soc_dai_symmetric_set_params(struct snd_soc_dai *dai,
				      struct snd_pcm_hw_params *params);
int snd_soc_dai_symmetric_apply(struct snd_pcm_substream *substream, struct snd_soc_dai *dai);
int snd_soc_dai_symmetric_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params);
void snd_soc_dai_symmetric_update(struct snd_pcm_substream *substream);
struct clk *snd_soc_dai_get_bclk(struct snd_soc_dai *dai);
unsigned int snd_soc_dai_get_bclk_ratio(struct snd_soc_dai *dai);
void snd_soc_dai_active_update(struct snd_soc_dai *dai, int stream, int action);

/*
 * In soc-component
 */
struct device_node *snd_soc_component_to_node(struct snd_soc_component *component);
bool snd_soc_component_matches_dlc(struct snd_soc_component *component,
				   const struct snd_soc_dai_link_component *dlc);
void snd_soc_component_remove(struct snd_soc_component *component, int probed);
int snd_soc_component_probe(struct snd_soc_component *component, struct snd_soc_card *card);
struct list_head *snd_soc_component_get_list_head(void);

#define for_each_component(component)					\
	list_for_each_entry(component, snd_soc_component_get_list_head(), list)

#endif /* __SOC_INTERNAL_H */
