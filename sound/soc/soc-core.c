// SPDX-License-Identifier: GPL-2.0+
//
// soc-core.c  --  ALSA SoC Audio Layer
//
// Copyright 2005 Wolfson Microelectronics PLC.
// Copyright 2005 Openedhand Ltd.
// Copyright (C) 2010 Slimlogic Ltd.
// Copyright (C) 2010 Texas Instruments Inc.
//
// Author: Liam Girdwood <lrg@slimlogic.co.uk>
//         with code, comments and ideas from :-
//         Richard Purdie <richard@openedhand.com>
//
//  TODO:
//   o Add hw rules to enforce rates, etc.
//   o More testing with other codecs/machines.
//   o Add more codecs and platforms to ensure good API coverage.
//   o Support TDM on PCM and I2S

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/acpi.h>
#include <linux/string_choices.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dpcm.h>
#include <sound/soc-topology.h>
#include "soc-internal.h"

#define CREATE_TRACE_POINTS
#include <trace/events/asoc.h>

DEFINE_MUTEX(client_mutex);

/*
 * This is used if driver don't need to have CPU/Codec/Platform
 * dai_link. see soc.h
 */
struct snd_soc_dai_link_component null_dailink_component[0];
EXPORT_SYMBOL_GPL(null_dailink_component);

/*
 * This is a timeout to do a DAPM powerdown after a stream is closed().
 * It can be used to eliminate pops between different playback streams, e.g.
 * between two audio tracks.
 */
static int pmdown_time = 5000;
module_param(pmdown_time, int, 0);
MODULE_PARM_DESC(pmdown_time, "DAPM stream powerdown time (msecs)");

static ssize_t pmdown_time_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct snd_soc_pcm_runtime *rtd = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%ld\n", rtd->pmdown_time);
}

static ssize_t pmdown_time_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct snd_soc_pcm_runtime *rtd = dev_get_drvdata(dev);
	int ret;

	ret = kstrtol(buf, 10, &rtd->pmdown_time);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR_RW(pmdown_time);

static struct attribute *soc_dev_attrs[] = {
	&dev_attr_pmdown_time.attr,
	NULL
};

static umode_t soc_dev_attr_is_visible(struct kobject *kobj,
				       struct attribute *attr, int idx)
{
	struct device *dev = kobj_to_dev(kobj);
	struct snd_soc_pcm_runtime *rtd = dev_get_drvdata(dev);

	if (!rtd)
		return 0;

	if (attr == &dev_attr_pmdown_time.attr)
		return attr->mode; /* always visible */
	return rtd->dai_link->num_codecs ? attr->mode : 0; /* enabled only with codec */
}

static const struct attribute_group soc_dapm_dev_group = {
	.attrs = snd_soc_dapm_dev_attrs,
	.is_visible = soc_dev_attr_is_visible,
};

static const struct attribute_group soc_dev_group = {
	.attrs = soc_dev_attrs,
	.is_visible = soc_dev_attr_is_visible,
};

static const struct attribute_group *soc_dev_attr_groups[] = {
	&soc_dapm_dev_group,
	&soc_dev_group,
	NULL
};

#ifdef CONFIG_DEBUG_FS
struct dentry *snd_soc_debugfs_root;
EXPORT_SYMBOL_GPL(snd_soc_debugfs_root);

static int dai_list_show(struct seq_file *m, void *v)
{
	struct snd_soc_component *component;
	struct snd_soc_dai *dai;
	guard(mutex)(&client_mutex);

	for_each_component(component)
		for_each_component_dais(component, dai)
			seq_printf(m, "%s\n", dai->name);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(dai_list);

static int component_list_show(struct seq_file *m, void *v)
{
	struct snd_soc_component *component;
	guard(mutex)(&client_mutex);

	for_each_component(component)
		seq_printf(m, "%s\n", component->name);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(component_list);

static void snd_soc_debugfs_init(void)
{
	snd_soc_debugfs_root = debugfs_create_dir("asoc", NULL);

	debugfs_create_file("dais", 0444, snd_soc_debugfs_root, NULL,
			    &dai_list_fops);

	debugfs_create_file("components", 0444, snd_soc_debugfs_root, NULL,
			    &component_list_fops);

	snd_soc_dapm_debugfs_pop_time(snd_soc_debugfs_root);
}

static void snd_soc_debugfs_exit(void)
{
	debugfs_remove_recursive(snd_soc_debugfs_root);
}

#else

static inline void snd_soc_debugfs_init(void) { }
static inline void snd_soc_debugfs_exit(void) { }

#endif

static inline int snd_soc_dlc_component_is_empty(struct snd_soc_dai_link_component *dlc)
{
	return !(dlc->dai_args || dlc->name || dlc->of_node);
}

static inline int snd_soc_dlc_component_is_invalid(struct snd_soc_dai_link_component *dlc)
{
	return (dlc->name && dlc->of_node);
}

static inline int snd_soc_dlc_dai_is_empty(struct snd_soc_dai_link_component *dlc)
{
	return !(dlc->dai_args || dlc->dai_name);
}

static int snd_soc_rtd_add_component(struct snd_soc_pcm_runtime *rtd,
				     struct snd_soc_component *component)
{
	struct snd_soc_component *comp;
	int i;

	for_each_rtd_components(rtd, i, comp) {
		/* already connected */
		if (comp == component)
			return 0;
	}

	/* see for_each_rtd_components */
	rtd->num_components++; // increment flex array count at first
	rtd->components[rtd->num_components - 1] = component;

	return 0;
}

struct snd_soc_component *snd_soc_rtdcom_lookup(struct snd_soc_pcm_runtime *rtd,
						const char *driver_name)
{
	struct snd_soc_component *component;
	int i;

	if (!driver_name)
		return NULL;

	/*
	 * NOTE
	 *
	 * snd_soc_rtdcom_lookup() will find component from rtd by using
	 * specified driver name.
	 * But, if many components which have same driver name are connected
	 * to 1 rtd, this function will return 1st found component.
	 */
	for_each_rtd_components(rtd, i, component) {
		const char *component_name = component->driver->name;

		if (!component_name)
			continue;

		if ((component_name == driver_name) ||
		    strcmp(component_name, driver_name) == 0)
			return component;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(snd_soc_rtdcom_lookup);

struct snd_soc_pcm_runtime
*snd_soc_get_pcm_runtime(struct snd_soc_card *card,
			 struct snd_soc_dai_link *dai_link)
{
	struct snd_soc_pcm_runtime *rtd;

	for_each_card_rtds(card, rtd) {
		if (rtd->dai_link == dai_link)
			return rtd;
	}
	dev_dbg(card->dev, "ASoC: failed to find rtd %s\n", dai_link->name);
	return NULL;
}
EXPORT_SYMBOL_GPL(snd_soc_get_pcm_runtime);

/*
 * Power down the audio subsystem pmdown_time msecs after close is called.
 * This is to ensure there are no pops or clicks in between any music tracks
 * due to DAPM power cycling.
 */
void snd_soc_close_delayed_work(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	int playback = SNDRV_PCM_STREAM_PLAYBACK;

	snd_soc_dpcm_mutex_lock(rtd);

	dev_dbg(rtd->dev,
		"ASoC: pop wq checking: %s status: %s waiting: %s\n",
		codec_dai->driver->playback.stream_name,
		snd_soc_dai_stream_active(codec_dai, playback) ?
		"active" : "inactive",
		str_yes_no(rtd->pop_wait));

	/* are we waiting on this codec DAI stream */
	if (rtd->pop_wait == 1) {
		rtd->pop_wait = 0;
		snd_soc_dapm_stream_event(rtd, playback,
					  SND_SOC_DAPM_STREAM_STOP);
	}

	snd_soc_dpcm_mutex_unlock(rtd);
}
EXPORT_SYMBOL_GPL(snd_soc_close_delayed_work);

static void soc_release_rtd_dev(struct device *dev)
{
	/* "dev" means "rtd->dev" */
	kfree(dev);
}

static void soc_free_pcm_runtime(struct snd_soc_pcm_runtime *rtd)
{
	if (!rtd)
		return;

	list_del(&rtd->list);

	flush_delayed_work(&rtd->delayed_work);
	snd_soc_pcm_component_free(rtd);

	/*
	 * we don't need to call kfree() for rtd->dev
	 * see
	 *	soc_release_rtd_dev()
	 *
	 * We don't need rtd->dev NULL check, because
	 * it is alloced *before* rtd.
	 * see
	 *	soc_new_pcm_runtime()
	 *
	 * We don't need to mind freeing for rtd,
	 * because it was created from dev (= rtd->dev)
	 * see
	 *	soc_new_pcm_runtime()
	 *
	 *		rtd = devm_kzalloc(dev, ...);
	 *		rtd->dev = dev
	 */
	device_unregister(rtd->dev);
}

static void close_delayed_work(struct work_struct *work) {
	struct snd_soc_pcm_runtime *rtd =
			container_of(work, struct snd_soc_pcm_runtime,
				     delayed_work.work);

	if (rtd->close_delayed_work_func)
		rtd->close_delayed_work_func(rtd);
}

static struct snd_soc_pcm_runtime *soc_new_pcm_runtime(
	struct snd_soc_card *card, struct snd_soc_dai_link *dai_link)
{
	struct snd_soc_pcm_runtime *rtd;
	struct device *dev;
	int ret;
	int stream;

	/*
	 * for rtd->dev
	 */
	dev = kzalloc_obj(struct device);
	if (!dev)
		return NULL;

	dev->parent	= card->dev;
	dev->release	= soc_release_rtd_dev;

	dev_set_name(dev, "%s", dai_link->name);

	ret = device_register(dev);
	if (ret < 0) {
		put_device(dev); /* soc_release_rtd_dev */
		return NULL;
	}

	/*
	 * for rtd
	 */
	rtd = devm_kzalloc(dev,
			   struct_size(rtd, components,
				       dai_link->num_cpus +
				       dai_link->num_codecs +
				       dai_link->num_platforms),
			   GFP_KERNEL);
	if (!rtd) {
		device_unregister(dev);
		return NULL;
	}

	rtd->dev = dev;
	INIT_LIST_HEAD(&rtd->list);
	for_each_pcm_streams(stream) {
		INIT_LIST_HEAD(&rtd->dpcm[stream].be_clients);
		INIT_LIST_HEAD(&rtd->dpcm[stream].fe_clients);
	}
	dev_set_drvdata(dev, rtd);
	INIT_DELAYED_WORK(&rtd->delayed_work, close_delayed_work);

	if ((dai_link->num_cpus + dai_link->num_codecs) == 0) {
		dev_err(dev, "ASoC: it has no CPU or codec DAIs\n");
		goto free_rtd;
	}

	/*
	 * for rtd->dais
	 */
	rtd->dais = devm_kcalloc(dev, dai_link->num_cpus + dai_link->num_codecs,
					sizeof(struct snd_soc_dai *),
					GFP_KERNEL);
	if (!rtd->dais)
		goto free_rtd;

	/*
	 * dais = [][][][][][][][][][][][][][][][][][]
	 *	  ^cpu_dais         ^codec_dais
	 *	  |--- num_cpus ---|--- num_codecs --|
	 * see
	 *	snd_soc_rtd_to_cpu()
	 *	snd_soc_rtd_to_codec()
	 */
	rtd->card	= card;
	rtd->dai_link	= dai_link;
	rtd->id		= card->num_rtd++;
	rtd->pmdown_time = pmdown_time;			/* default power off timeout */

	/* see for_each_card_rtds */
	list_add_tail(&rtd->list, &card->rtd_list);

	ret = device_add_groups(dev, soc_dev_attr_groups);
	if (ret < 0)
		goto free_rtd;

	return rtd;

free_rtd:
	soc_free_pcm_runtime(rtd);
	return NULL;
}

void snd_soc_flush_all_delayed_work(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd;

	for_each_card_rtds(card, rtd)
		flush_delayed_work(&rtd->delayed_work);
}

#ifdef CONFIG_PM_SLEEP
void snd_soc_playback_digital_mute(struct snd_soc_card *card, int mute)
{
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *dai;
	int playback = SNDRV_PCM_STREAM_PLAYBACK;
	int i;

	for_each_card_rtds(card, rtd) {

		if (rtd->dai_link->ignore_suspend)
			continue;

		for_each_rtd_dais(rtd, i, dai) {
			if (snd_soc_dai_stream_active(dai, playback))
				snd_soc_dai_digital_mute(dai, mute, playback);
		}
	}
}

void snd_soc_dapm_suspend_resume(struct snd_soc_card *card, int event)
{
	struct snd_soc_pcm_runtime *rtd;
	int stream;

	for_each_card_rtds(card, rtd) {

		if (rtd->dai_link->ignore_suspend)
			continue;

		for_each_pcm_streams(stream)
			snd_soc_dapm_stream_event(rtd, stream, event);
	}
}

/* powers down audio subsystem for suspend */
int snd_soc_suspend(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct snd_soc_component *component;
	struct snd_soc_pcm_runtime *rtd;
	int i;

	/* If the card is not initialized yet there is nothing to do */
	if (!snd_soc_card_is_instantiated(card))
		return 0;

	/*
	 * Due to the resume being scheduled into a workqueue we could
	 * suspend before that's finished - wait for it to complete.
	 */
	snd_power_wait(card->snd_card);

	/* we're going to block userspace touching us until resume completes */
	snd_power_change_state(card->snd_card, SNDRV_CTL_POWER_D3hot);

	/* mute any active DACs */
	snd_soc_playback_digital_mute(card, 1);

	/* suspend all pcms */
	for_each_card_rtds(card, rtd) {
		if (rtd->dai_link->ignore_suspend)
			continue;

		snd_pcm_suspend_all(rtd->pcm);
	}

	snd_soc_card_suspend_pre(card);

	/* close any waiting streams */
	snd_soc_flush_all_delayed_work(card);

	snd_soc_dapm_suspend_resume(card, SND_SOC_DAPM_STREAM_SUSPEND);

	/* Recheck all endpoints too, their state is affected by suspend */
	snd_soc_dapm_mark_endpoints_dirty(card);
	snd_soc_dapm_sync(snd_soc_card_to_dapm(card));

	/* suspend all COMPONENTs */
	for_each_card_rtds(card, rtd) {

		if (rtd->dai_link->ignore_suspend)
			continue;

		for_each_rtd_components(rtd, i, component) {
			struct snd_soc_dapm_context *dapm = snd_soc_component_to_dapm(component);

			/*
			 * ignore if component was already suspended
			 */
			if (snd_soc_component_is_suspended(component))
				continue;

			/*
			 * If there are paths active then the COMPONENT will be
			 * held with bias _ON and should not be suspended.
			 */
			switch (snd_soc_dapm_get_bias_level(dapm)) {
			case SND_SOC_BIAS_STANDBY:
				/*
				 * If the COMPONENT is capable of idle
				 * bias off then being in STANDBY
				 * means it's doing something,
				 * otherwise fall through.
				 */
				if (!snd_soc_dapm_get_idle_bias(dapm)) {
					dev_dbg(component->dev,
						"ASoC: idle_bias_off CODEC on over suspend\n");
					break;
				}
				fallthrough;

			case SND_SOC_BIAS_OFF:
				snd_soc_component_suspend(component);
				if (component->regmap)
					regcache_mark_dirty(component->regmap);
				/* deactivate pins to sleep state */
				pinctrl_pm_select_sleep_state(component->dev);
				break;
			default:
				dev_dbg(component->dev,
					"ASoC: COMPONENT is on over suspend\n");
				break;
			}
		}
	}

	snd_soc_card_suspend_post(card);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_suspend);

/* powers up audio subsystem after a suspend */
int snd_soc_resume(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct snd_soc_component *component;

	/* If the card is not initialized yet there is nothing to do */
	if (!snd_soc_card_is_instantiated(card))
		return 0;

	/* activate pins from sleep state */
	for_each_card_components(card, component)
		if (snd_soc_component_active(component))
			pinctrl_pm_select_default_state(component->dev);

	dev_dbg(dev, "ASoC: Scheduling resume work\n");
	if (!schedule_work(&card->deferred_resume_work))
		dev_err(dev, "ASoC: resume work item may be lost\n");

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_resume);

#else
#define snd_soc_suspend NULL
#define snd_soc_resume NULL
#endif

struct of_phandle_args *snd_soc_copy_dai_args(struct device *dev,
					      const struct of_phandle_args *args)
{
	struct of_phandle_args *ret = devm_kzalloc(dev, sizeof(*ret), GFP_KERNEL);

	if (!ret)
		return NULL;

	*ret = *args;

	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_copy_dai_args);

struct snd_soc_component *snd_soc_find_component(const struct snd_soc_dai_link_component *dlc)
{
	struct snd_soc_component *component;

	lockdep_assert_held(&client_mutex);

	/*
	 * NOTE
	 *
	 * It returns *1st* found component, but some driver
	 * has few components by same of_node/name
	 * ex)
	 *	CPU component and generic DMAEngine component
	 */
	for_each_component(component)
		if (snd_soc_component_matches_dlc(component, dlc))
			return component;

	return NULL;
}

/**
 * snd_soc_find_dai - Find a registered DAI
 *
 * @dlc: name of the DAI or the DAI driver and optional component info to match
 *
 * This function will search all registered components and their DAIs to
 * find the DAI of the same name. The component's of_node and name
 * should also match if being specified.
 *
 * Return: pointer of DAI, or NULL if not found.
 */
struct snd_soc_dai *snd_soc_find_dai(
	const struct snd_soc_dai_link_component *dlc)
{
	struct snd_soc_component *component;
	struct snd_soc_dai *dai;

	lockdep_assert_held(&client_mutex);

	/* Find CPU DAI from registered DAIs */
	for_each_component(component)
		if (snd_soc_component_matches_dlc(component, dlc))
			for_each_component_dais(component, dai)
				if (snd_soc_dai_matches_dlc(dai, dlc))
					return dai;

	return NULL;
}
EXPORT_SYMBOL_GPL(snd_soc_find_dai);

struct snd_soc_dai *snd_soc_find_dai_with_mutex(
	const struct snd_soc_dai_link_component *dlc)
{
	guard(mutex)(&client_mutex);

	return snd_soc_find_dai(dlc);
}
EXPORT_SYMBOL_GPL(snd_soc_find_dai_with_mutex);

static int soc_dai_link_sanity_check(struct snd_soc_card *card,
				     struct snd_soc_dai_link *link)
{
	int i;
	struct snd_soc_dai_link_component *dlc;

	/* Codec check */
	for_each_link_codecs(link, i, dlc) {
		/*
		 * Codec must be specified by 1 of name or OF node,
		 * not both or neither.
		 */
		if (snd_soc_dlc_component_is_invalid(dlc))
			goto component_invalid;

		if (snd_soc_dlc_component_is_empty(dlc))
			goto component_empty;

		/* Codec DAI name must be specified */
		if (snd_soc_dlc_dai_is_empty(dlc))
			goto dai_empty;

		/*
		 * Defer card registration if codec component is not added to
		 * component list.
		 */
		if (!snd_soc_find_component(dlc))
			goto component_not_found;
	}

	/* Platform check */
	for_each_link_platforms(link, i, dlc) {
		/*
		 * Platform may be specified by either name or OF node, but it
		 * can be left unspecified, then no components will be inserted
		 * in the rtdcom list
		 */
		if (snd_soc_dlc_component_is_invalid(dlc))
			goto component_invalid;

		if (snd_soc_dlc_component_is_empty(dlc))
			goto component_empty;

		/*
		 * Defer card registration if platform component is not added to
		 * component list.
		 */
		if (!snd_soc_find_component(dlc))
			goto component_not_found;
	}

	/* CPU check */
	for_each_link_cpus(link, i, dlc) {
		/*
		 * CPU device may be specified by either name or OF node, but
		 * can be left unspecified, and will be matched based on DAI
		 * name alone..
		 */
		if (snd_soc_dlc_component_is_invalid(dlc))
			goto component_invalid;


		if (snd_soc_dlc_component_is_empty(dlc)) {
			/*
			 * At least one of CPU DAI name or CPU device name/node must be specified
			 */
			if (snd_soc_dlc_dai_is_empty(dlc))
				goto component_dai_empty;
		} else {
			/*
			 * Defer card registration if Component is not added
			 */
			if (!snd_soc_find_component(dlc))
				goto component_not_found;
		}
	}

	return 0;

component_invalid:
	dev_err(card->dev, "ASoC: Both Component name/of_node are set for %s\n", link->name);
	return -EINVAL;

component_empty:
	dev_err(card->dev, "ASoC: Neither Component name/of_node are set for %s\n", link->name);
	return -EINVAL;

component_not_found:
	dev_dbg(card->dev, "ASoC: Component %s not found for link %s\n", dlc->name, link->name);
	return -EPROBE_DEFER;

dai_empty:
	dev_err(card->dev, "ASoC: DAI name is not set for %s\n", link->name);
	return -EINVAL;

component_dai_empty:
	dev_err(card->dev, "ASoC: Neither DAI/Component name/of_node are set for %s\n", link->name);
	return -EINVAL;
}

#define MAX_DEFAULT_CH_MAP_SIZE 8
static struct snd_soc_dai_link_ch_map default_ch_map_sync[MAX_DEFAULT_CH_MAP_SIZE] = {
	{ .cpu = 0, .codec = 0 },
	{ .cpu = 1, .codec = 1 },
	{ .cpu = 2, .codec = 2 },
	{ .cpu = 3, .codec = 3 },
	{ .cpu = 4, .codec = 4 },
	{ .cpu = 5, .codec = 5 },
	{ .cpu = 6, .codec = 6 },
	{ .cpu = 7, .codec = 7 },
};
static struct snd_soc_dai_link_ch_map default_ch_map_1cpu[MAX_DEFAULT_CH_MAP_SIZE] = {
	{ .cpu = 0, .codec = 0 },
	{ .cpu = 0, .codec = 1 },
	{ .cpu = 0, .codec = 2 },
	{ .cpu = 0, .codec = 3 },
	{ .cpu = 0, .codec = 4 },
	{ .cpu = 0, .codec = 5 },
	{ .cpu = 0, .codec = 6 },
	{ .cpu = 0, .codec = 7 },
};
static struct snd_soc_dai_link_ch_map default_ch_map_1codec[MAX_DEFAULT_CH_MAP_SIZE] = {
	{ .cpu = 0, .codec = 0 },
	{ .cpu = 1, .codec = 0 },
	{ .cpu = 2, .codec = 0 },
	{ .cpu = 3, .codec = 0 },
	{ .cpu = 4, .codec = 0 },
	{ .cpu = 5, .codec = 0 },
	{ .cpu = 6, .codec = 0 },
	{ .cpu = 7, .codec = 0 },
};
static int snd_soc_compensate_channel_connection_map(struct snd_soc_card *card,
						     struct snd_soc_dai_link *dai_link)
{
	struct snd_soc_dai_link_ch_map *ch_maps;
	int i;

	/*
	 * dai_link->ch_maps indicates how CPU/Codec are connected.
	 * It will be a map seen from a larger number of DAI.
	 * see
	 *	soc.h :: [dai_link->ch_maps Image sample]
	 */

	/* it should have ch_maps if connection was N:M */
	if (dai_link->num_cpus > 1 && dai_link->num_codecs > 1 &&
	    dai_link->num_cpus != dai_link->num_codecs && !dai_link->ch_maps) {
		dev_err(card->dev, "need to have ch_maps when N:M connection (%s)",
			dai_link->name);
		return -EINVAL;
	}

	/* do nothing if it has own maps */
	if (dai_link->ch_maps)
		goto sanity_check;

	/* check default map size */
	if (dai_link->num_cpus   > MAX_DEFAULT_CH_MAP_SIZE ||
	    dai_link->num_codecs > MAX_DEFAULT_CH_MAP_SIZE) {
		dev_err(card->dev, "soc-core.c needs update default_connection_maps");
		return -EINVAL;
	}

	/* Compensate missing map for ... */
	if (dai_link->num_cpus == dai_link->num_codecs)
		dai_link->ch_maps = default_ch_map_sync;	/* for 1:1 or N:N */
	else if (dai_link->num_cpus <  dai_link->num_codecs)
		dai_link->ch_maps = default_ch_map_1cpu;	/* for 1:N */
	else
		dai_link->ch_maps = default_ch_map_1codec;	/* for N:1 */

sanity_check:
	dev_dbg(card->dev, "dai_link %s\n", dai_link->stream_name);
	for_each_link_ch_maps(dai_link, i, ch_maps) {
		if ((ch_maps->cpu   >= dai_link->num_cpus) ||
		    (ch_maps->codec >= dai_link->num_codecs)) {
			dev_err(card->dev,
				"unexpected dai_link->ch_maps[%d] index (cpu(%d/%d) codec(%d/%d))",
				i,
				ch_maps->cpu,	dai_link->num_cpus,
				ch_maps->codec,	dai_link->num_codecs);
			return -EINVAL;
		}

		dev_dbg(card->dev, "  [%d] cpu%d <-> codec%d\n",
			i, ch_maps->cpu, ch_maps->codec);
	}

	return 0;
}

/**
 * snd_soc_remove_pcm_runtime - Remove a pcm_runtime from card
 * @card: The ASoC card to which the pcm_runtime has
 * @rtd: The pcm_runtime to remove
 *
 * This function removes a pcm_runtime from the ASoC card.
 */
void snd_soc_remove_pcm_runtime(struct snd_soc_card *card,
				struct snd_soc_pcm_runtime *rtd)
{
	if (!rtd)
		return;

	lockdep_assert_held(&client_mutex);

	/*
	 * Notify the machine driver for extra destruction
	 */
	snd_soc_card_remove_dai_link(card, rtd->dai_link);

	soc_free_pcm_runtime(rtd);
}
EXPORT_SYMBOL_GPL(snd_soc_remove_pcm_runtime);

/**
 * snd_soc_add_pcm_runtime - Add a pcm_runtime dynamically via dai_link
 * @card: The ASoC card to which the pcm_runtime is added
 * @dai_link: The DAI link to find pcm_runtime
 *
 * This function adds a pcm_runtime ASoC card by using dai_link.
 *
 * Note: Topology can use this API to add pcm_runtime when probing the
 * topology component. And machine drivers can still define static
 * DAI links in dai_link array.
 */
static int snd_soc_add_pcm_runtime(struct snd_soc_card *card,
				   struct snd_soc_dai_link *dai_link)
{
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai_link_component *codec, *platform, *cpu;
	struct snd_soc_component *component;
	int i, id, ret;

	lockdep_assert_held(&client_mutex);

	/*
	 * Notify the machine driver for extra initialization
	 */
	ret = snd_soc_card_add_dai_link(card, dai_link);
	if (ret < 0)
		return ret;

	if (dai_link->ignore)
		return 0;

	dev_dbg(card->dev, "ASoC: binding %s\n", dai_link->name);

	ret = soc_dai_link_sanity_check(card, dai_link);
	if (ret < 0)
		return ret;

	rtd = soc_new_pcm_runtime(card, dai_link);
	if (!rtd)
		return -ENOMEM;

	for_each_link_cpus(dai_link, i, cpu) {
		snd_soc_rtd_to_cpu(rtd, i) = snd_soc_find_dai(cpu);
		if (!snd_soc_rtd_to_cpu(rtd, i)) {
			dev_info(card->dev, "ASoC: CPU DAI %s not registered\n",
				 cpu->dai_name);
			goto _err_defer;
		}
		snd_soc_rtd_add_component(rtd, snd_soc_rtd_to_cpu(rtd, i)->component);
	}

	/* Find CODEC from registered CODECs */
	for_each_link_codecs(dai_link, i, codec) {
		snd_soc_rtd_to_codec(rtd, i) = snd_soc_find_dai(codec);
		if (!snd_soc_rtd_to_codec(rtd, i)) {
			dev_info(card->dev, "ASoC: CODEC DAI %s not registered\n",
				 codec->dai_name);
			goto _err_defer;
		}

		snd_soc_rtd_add_component(rtd, snd_soc_rtd_to_codec(rtd, i)->component);
	}

	/* Find PLATFORM from registered PLATFORMs */
	for_each_link_platforms(dai_link, i, platform) {
		for_each_component(component) {
			if (!snd_soc_component_matches_dlc(component, platform))
				continue;

			if (snd_soc_component_is_dummy(component) && component->num_dai)
				continue;

			snd_soc_rtd_add_component(rtd, component);
		}
	}

	/*
	 * Most drivers will register their PCMs using DAI link ordering but
	 * topology based drivers can use the DAI link id field to set PCM
	 * device number and then use rtd + a base offset of the BEs.
	 *
	 * FIXME
	 *
	 * This should be implemented by using "dai_link" feature instead of
	 * "component" feature.
	 */
	id = rtd->id;
	for_each_rtd_components(rtd, i, component) {
		if (!component->driver->use_dai_pcm_id)
			continue;

		if (rtd->dai_link->no_pcm)
			id += component->driver->be_pcm_base;
		else
			id = rtd->dai_link->id;
	}
	rtd->id = id;

	return 0;

_err_defer:
	snd_soc_remove_pcm_runtime(card, rtd);
	return -EPROBE_DEFER;
}

int snd_soc_add_pcm_runtimes(struct snd_soc_card *card,
			     struct snd_soc_dai_link *dai_link,
			     int num_dai_link)
{
	for (int i = 0; i < num_dai_link; i++) {
		int ret;

		ret = snd_soc_compensate_channel_connection_map(card, dai_link + i);
		if (ret < 0)
			return ret;

		ret = snd_soc_add_pcm_runtime(card, dai_link + i);
		if (ret < 0)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_add_pcm_runtimes);

/**
 * snd_soc_runtime_set_dai_fmt() - Change DAI link format for a ASoC runtime
 * @rtd: The runtime for which the DAI link format should be changed
 * @dai_fmt: The new DAI link format
 *
 * This function updates the DAI link format for all DAIs connected to the DAI
 * link for the specified runtime.
 *
 * Note: For setups with a static format set the dai_fmt field in the
 * corresponding snd_dai_link struct instead of using this function.
 *
 * Returns 0 on success, otherwise a negative error code.
 */
int snd_soc_runtime_set_dai_fmt(struct snd_soc_pcm_runtime *rtd,
				unsigned int dai_fmt)
{
	struct snd_soc_dai *cpu_dai;
	struct snd_soc_dai *codec_dai;
	unsigned int ext_fmt;
	unsigned int i;
	int ret;

	if (!dai_fmt)
		return 0;

	/*
	 * dai_fmt has 4 types
	 *	1. SND_SOC_DAIFMT_FORMAT_MASK
	 *	2. SND_SOC_DAIFMT_CLOCK
	 *	3. SND_SOC_DAIFMT_INV
	 *	4. SND_SOC_DAIFMT_CLOCK_PROVIDER
	 *
	 * 4. CLOCK_PROVIDER is set from Codec perspective in dai_fmt. So it will be flipped
	 * when this function calls set_fmt() for CPU (CBx_CFx -> Bx_Cx). see below.
	 * This mean, we can't set CPU/Codec both are clock consumer for example.
	 * New idea handles 4. in each dai->ext_fmt. It can keep compatibility.
	 *
	 * Legacy
	 *	dai_fmt  includes 1, 2, 3, 4
	 *
	 * New idea
	 *	dai_fmt  includes 1, 2, 3
	 *	ext_fmt  includes 4
	 */
	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		ext_fmt = rtd->dai_link->codecs[i].ext_fmt;
		ret = snd_soc_dai_set_fmt(codec_dai, dai_fmt | ext_fmt);
		if (ret != 0 && ret != -ENOTSUPP)
			return ret;
	}

	/* Flip the polarity for the "CPU" end of link */
	/* Will effect only for 4. SND_SOC_DAIFMT_CLOCK_PROVIDER */
	dai_fmt = snd_soc_daifmt_clock_provider_flipped(dai_fmt);

	for_each_rtd_cpu_dais(rtd, i, cpu_dai) {
		ext_fmt = rtd->dai_link->cpus[i].ext_fmt;
		ret = snd_soc_dai_set_fmt(cpu_dai, dai_fmt | ext_fmt);
		if (ret != 0 && ret != -ENOTSUPP)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_runtime_set_dai_fmt);

/* probes a new socdev */
static int soc_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	/*
	 * no card, so machine driver should be registering card
	 * we should not be here in that case so ret error
	 */
	if (!card)
		return -EINVAL;

	dev_warn(&pdev->dev,
		 "ASoC: machine %s should use snd_soc_register_card()\n",
		 card->name);

	/* Bodge while we unpick instantiation */
	card->dev = &pdev->dev;

	return devm_snd_soc_register_card(&pdev->dev, card);
}

int snd_soc_poweroff(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct snd_soc_component *component;

	if (!snd_soc_card_is_instantiated(card))
		return 0;

	/*
	 * Flush out pmdown_time work - we actually do want to run it
	 * now, we're shutting down so no imminent restart.
	 */
	snd_soc_flush_all_delayed_work(card);

	snd_soc_dapm_shutdown(card);

	/* deactivate pins to sleep state */
	for_each_card_components(card, component)
		pinctrl_pm_select_sleep_state(component->dev);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_poweroff);

const struct dev_pm_ops snd_soc_pm_ops = {
	.suspend = snd_soc_suspend,
	.resume = snd_soc_resume,
	.freeze = snd_soc_suspend,
	.thaw = snd_soc_resume,
	.poweroff = snd_soc_poweroff,
	.restore = snd_soc_resume,
};
EXPORT_SYMBOL_GPL(snd_soc_pm_ops);

/* ASoC platform driver */
static struct platform_driver soc_driver = {
	.driver		= {
		.name		= "soc-audio",
		.pm		= &snd_soc_pm_ops,
	},
	.probe		= soc_probe,
};

/**
 * snd_soc_cnew - create new control
 * @_template: control template
 * @data: control private data
 * @long_name: control long name
 * @prefix: control name prefix
 *
 * Create a new mixer control from a template control.
 *
 * Returns 0 for success, else error.
 */
struct snd_kcontrol *snd_soc_cnew(const struct snd_kcontrol_new *_template,
				  void *data, const char *long_name,
				  const char *prefix)
{
	struct snd_kcontrol_new template;
	struct snd_kcontrol *kcontrol;
	char *name = NULL;

	memcpy(&template, _template, sizeof(template));
	template.index = 0;

	if (!long_name)
		long_name = template.name;

	if (prefix) {
		name = kasprintf(GFP_KERNEL, "%s %s", prefix, long_name);
		if (!name)
			return NULL;

		template.name = name;
	} else {
		template.name = long_name;
	}

	kcontrol = snd_ctl_new1(&template, data);

	kfree(name);

	return kcontrol;
}
EXPORT_SYMBOL_GPL(snd_soc_cnew);

int snd_soc_add_controls(struct snd_card *card, struct device *dev,
			 const struct snd_kcontrol_new *controls, int num_controls,
			 const char *prefix, void *data)
{
	int i;

	for (i = 0; i < num_controls; i++) {
		const struct snd_kcontrol_new *control = &controls[i];
		int err = snd_ctl_add(card, snd_soc_cnew(control, data,
							 control->name, prefix));
		if (err < 0) {
			dev_err(dev, "ASoC: Failed to add %s: %d\n",
				control->name, err);
			return err;
		}
	}

	return 0;
}

/**
 * snd_soc_register_card - Register a card with the ASoC core
 *
 * @card: Card to register
 *
 */
int snd_soc_register_card(struct snd_soc_card *card)
{
	if (!card->name || !card->dev)
		return -EINVAL;

	card->dapm = snd_soc_dapm_alloc(card->dev);
	if (!card->dapm)
		return -ENOMEM;

	dev_set_drvdata(card->dev, card);

	INIT_LIST_HEAD(&card->widgets);
	INIT_LIST_HEAD(&card->paths);
	INIT_LIST_HEAD(&card->dapm_list);
	INIT_LIST_HEAD(&card->aux_comp_list);
	INIT_LIST_HEAD(&card->component_dev_list);
	INIT_LIST_HEAD(&card->list);
	INIT_LIST_HEAD(&card->rtd_list);
	INIT_LIST_HEAD(&card->dapm_dirty);

	card->instantiated = 0;
	mutex_init(&card->mutex);
	mutex_init(&card->dapm_mutex);
	mutex_init(&card->pcm_mutex);

	guard(mutex)(&client_mutex);

	return snd_soc_card_bind_call(card);
}
EXPORT_SYMBOL_GPL(snd_soc_register_card);

/**
 * snd_soc_unregister_card - Unregister a card with the ASoC core
 *
 * @card: Card to unregister
 *
 */
void snd_soc_unregister_card(struct snd_soc_card *card)
{
	guard(mutex)(&client_mutex);

	snd_soc_card_unbind(card);
	list_del(&card->list);

	dev_dbg(card->dev, "ASoC: Unregistered card '%s'\n", card->name);
}
EXPORT_SYMBOL_GPL(snd_soc_unregister_card);

/*
 * Simplify DAI link configuration by removing ".-1" from device names
 * and sanitizing names.
 */
char *snd_soc_fmt_single_name(struct device *dev, int *id)
{
	const char *devname = dev_name(dev);
	char *found, *name;
	unsigned int id1, id2;
	int __id;

	if (devname == NULL)
		return NULL;

	name = devm_kstrdup(dev, devname, GFP_KERNEL);
	if (!name)
		return NULL;

	/* are we a "%s.%d" name (platform and SPI components) */
	found = strstr(name, dev->driver->name);
	if (found) {
		/* get ID */
		if (sscanf(&found[strlen(dev->driver->name)], ".%d", &__id) == 1) {

			/* discard ID from name if ID == -1 */
			if (__id == -1)
				found[strlen(dev->driver->name)] = '\0';
		}

	/* I2C component devices are named "bus-addr" */
	} else if (sscanf(name, "%x-%x", &id1, &id2) == 2) {

		/* create unique ID number from I2C addr and bus */
		__id = ((id1 & 0xffff) << 16) + id2;

		devm_kfree(dev, name);

		/* sanitize component name for DAI link creation */
		name = devm_kasprintf(dev, GFP_KERNEL, "%s.%s", dev->driver->name, devname);
	} else {
		__id = 0;
	}

	if (id)
		*id = __id;

	return name;
}

/*
 * Simplify DAI link naming for single devices with multiple DAIs by removing
 * any ".-1" and using the DAI name (instead of device name).
 */
char *snd_soc_fmt_multiple_name(struct device *dev, struct snd_soc_dai_driver *dai_drv)
{
	if (dai_drv->name == NULL) {
		dev_err(dev,
			"ASoC: error - multiple DAI %s registered with no name\n",
			dev_name(dev));
		return NULL;
	}

	return devm_kstrdup(dev, dai_drv->name, GFP_KERNEL);
}

/**
 * snd_soc_unregister_dais - Unregister DAIs from the ASoC core
 *
 * @component: The component for which the DAIs should be unregistered
 */
static void snd_soc_unregister_dais(struct snd_soc_component *component)
{
	struct snd_soc_dai *dai, *_dai;

	for_each_component_dais_safe(component, dai, _dai)
		snd_soc_unregister_dai(dai);
}

/**
 * snd_soc_register_dais - Register a DAI with the ASoC core
 *
 * @component: The component the DAIs are registered for
 * @dai_drv: DAI driver to use for the DAIs
 * @count: Number of DAIs
 */
static int snd_soc_register_dais(struct snd_soc_component *component,
				 struct snd_soc_dai_driver *dai_drv,
				 size_t count)
{
	struct snd_soc_dai *dai;
	unsigned int i;
	int ret;

	for (i = 0; i < count; i++) {
		dai = snd_soc_register_dai(component, dai_drv + i, count == 1 &&
					   component->driver->legacy_dai_naming);
		if (dai == NULL) {
			ret = -ENOMEM;
			goto err;
		}
	}

	return 0;

err:
	snd_soc_unregister_dais(component);

	return ret;
}

#define ENDIANNESS_MAP(name) \
	(SNDRV_PCM_FMTBIT_##name##LE | SNDRV_PCM_FMTBIT_##name##BE)
static u64 endianness_format_map[] = {
	ENDIANNESS_MAP(S16_),
	ENDIANNESS_MAP(U16_),
	ENDIANNESS_MAP(S24_),
	ENDIANNESS_MAP(U24_),
	ENDIANNESS_MAP(S32_),
	ENDIANNESS_MAP(U32_),
	ENDIANNESS_MAP(S24_3),
	ENDIANNESS_MAP(U24_3),
	ENDIANNESS_MAP(S20_3),
	ENDIANNESS_MAP(U20_3),
	ENDIANNESS_MAP(S18_3),
	ENDIANNESS_MAP(U18_3),
	ENDIANNESS_MAP(FLOAT_),
	ENDIANNESS_MAP(FLOAT64_),
	ENDIANNESS_MAP(IEC958_SUBFRAME_),
};

/*
 * Fix up the DAI formats for endianness: codecs don't actually see
 * the endianness of the data but we're using the CPU format
 * definitions which do need to include endianness so we ensure that
 * codec DAIs always have both big and little endian variants set.
 */
static void convert_endianness_formats(struct snd_soc_pcm_stream *stream)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(endianness_format_map); i++)
		if (stream->formats & endianness_format_map[i])
			stream->formats |= endianness_format_map[i];
}

static void snd_soc_del_component_unlocked(struct snd_soc_component *component)
{
	struct snd_soc_card *card = component->card;
	bool instantiated;

	snd_soc_unregister_dais(component);

	if (card) {
		instantiated = card->instantiated;
		snd_soc_card_unbind(card);
		if (instantiated)
			list_add(&card->list, &unbind_card_list);
	}

	list_del(&component->list);
}

struct snd_soc_component *snd_soc_component_new(struct device *dev, const char *name,
						const struct snd_soc_component_driver *driver)
{
	struct snd_soc_component *component;

	component = devm_kzalloc(dev, sizeof(*component), GFP_KERNEL);
	if (!component)
		return ERR_PTR(-ENOMEM);

	component->dapm = snd_soc_dapm_alloc(dev);
	if (!component->dapm)
		return ERR_PTR(-ENOMEM);

	if (!name) {
		name = snd_soc_fmt_single_name(dev, NULL);
		if (!name) {
			dev_err(dev, "ASoC: Failed to allocate name\n");
			return ERR_PTR(-ENOMEM);
		}
	}

	component->dev = dev;
	component->name = name;
	component->driver = driver;

	INIT_LIST_HEAD(&component->dai_list);
	INIT_LIST_HEAD(&component->dobj_list);
	INIT_LIST_HEAD(&component->card_list);
	INIT_LIST_HEAD(&component->list);
	INIT_LIST_HEAD(&component->card_aux_list);
	mutex_init(&component->io_mutex);

	return component;
}
EXPORT_SYMBOL_GPL(snd_soc_component_new);

int snd_soc_component_register(struct snd_soc_component *component,
			       struct snd_soc_dai_driver *dai_drv,
			       int num_dai)
{
	struct snd_soc_card *card, *c;
	int ret;
	int i;
	guard(mutex)(&client_mutex);

	if (component->driver->endianness) {
		for (i = 0; i < num_dai; i++) {
			convert_endianness_formats(&dai_drv[i].playback);
			convert_endianness_formats(&dai_drv[i].capture);
		}
	}

	ret = snd_soc_register_dais(component, dai_drv, num_dai);
	if (ret < 0) {
		dev_err(component->dev, "ASoC: Failed to register DAIs: %d\n",
			ret);
		goto err_cleanup;
	}

	if (!component->driver->write && !component->driver->read) {
		if (!component->regmap)
			component->regmap = dev_get_regmap(component->dev,
							   NULL);
	}

	/* see for_each_component */
	list_add(&component->list, snd_soc_component_get_list_head());

	list_for_each_entry_safe(card, c, &unbind_card_list, list)
		snd_soc_card_bind_call(card);

err_cleanup:
	if (ret < 0)
		snd_soc_del_component_unlocked(component);

	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_component_register);

int snd_soc_register_component(struct device *dev,
			       const struct snd_soc_component_driver *component_driver,
			       struct snd_soc_dai_driver *dai_drv,
			       int num_dai)
{
	struct snd_soc_component *component;

	component = snd_soc_component_new(dev, NULL, component_driver);
	if (IS_ERR(component))
		return PTR_ERR(component);

	return snd_soc_component_register(component, dai_drv, num_dai);
}
EXPORT_SYMBOL_GPL(snd_soc_register_component);

/**
 * snd_soc_unregister_component_by_driver - Unregister component using a given driver
 * from the ASoC core
 *
 * @dev: The device to unregister
 * @component_driver: The component driver to unregister
 */
void snd_soc_unregister_component_by_driver(struct device *dev,
					    const struct snd_soc_component_driver *component_driver)
{
	const char *driver_name = NULL;

	if (component_driver)
		driver_name = component_driver->name;

	guard(mutex)(&client_mutex);

	while (1) {
		struct snd_soc_component *component = snd_soc_lookup_component_nolocked(dev, driver_name);

		if (!component)
			break;

		snd_soc_del_component_unlocked(component);
	}
}
EXPORT_SYMBOL_GPL(snd_soc_unregister_component_by_driver);

/* Retrieve a card's name from device tree */
int snd_soc_of_parse_card_name(struct snd_soc_card *card,
			       const char *propname)
{
	struct device_node *np;
	int ret;

	if (!card->dev) {
		pr_err("card->dev is not set before calling %s\n", __func__);
		return -EINVAL;
	}

	np = card->dev->of_node;

	ret = of_property_read_string_index(np, propname, 0, &card->name);
	/*
	 * EINVAL means the property does not exist. This is fine providing
	 * card->name was previously set, which is checked later in
	 * snd_soc_register_card.
	 */
	if (ret < 0 && ret != -EINVAL) {
		dev_err(card->dev,
			"ASoC: Property '%s' could not be read: %d\n",
			propname, ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_of_parse_card_name);

static const struct snd_soc_dapm_widget simple_widgets[] = {
	SND_SOC_DAPM_MIC("Microphone", NULL),
	SND_SOC_DAPM_LINE("Line", NULL),
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

int snd_soc_of_parse_audio_simple_widgets(struct snd_soc_card *card,
					  const char *propname)
{
	struct device_node *np = card->dev->of_node;
	struct snd_soc_dapm_widget *widgets;
	const char *template, *wname;
	int i, j, num_widgets;

	num_widgets = of_property_count_strings(np, propname);
	if (num_widgets < 0) {
		dev_err(card->dev,
			"ASoC: Property '%s' does not exist\n",	propname);
		return -EINVAL;
	}
	if (!num_widgets) {
		dev_err(card->dev, "ASoC: Property '%s's length is zero\n",
			propname);
		return -EINVAL;
	}
	if (num_widgets & 1) {
		dev_err(card->dev,
			"ASoC: Property '%s' length is not even\n", propname);
		return -EINVAL;
	}

	num_widgets /= 2;

	widgets = devm_kcalloc(card->dev, num_widgets, sizeof(*widgets),
			       GFP_KERNEL);
	if (!widgets) {
		dev_err(card->dev,
			"ASoC: Could not allocate memory for widgets\n");
		return -ENOMEM;
	}

	for (i = 0; i < num_widgets; i++) {
		int ret = of_property_read_string_index(np, propname,
							2 * i, &template);
		if (ret) {
			dev_err(card->dev,
				"ASoC: Property '%s' index %d read error:%d\n",
				propname, 2 * i, ret);
			return -EINVAL;
		}

		for (j = 0; j < ARRAY_SIZE(simple_widgets); j++) {
			if (!strncmp(template, simple_widgets[j].name,
				     strlen(simple_widgets[j].name))) {
				widgets[i] = simple_widgets[j];
				break;
			}
		}

		if (j >= ARRAY_SIZE(simple_widgets)) {
			dev_err(card->dev,
				"ASoC: DAPM widget '%s' is not supported\n",
				template);
			return -EINVAL;
		}

		ret = of_property_read_string_index(np, propname,
						    (2 * i) + 1,
						    &wname);
		if (ret) {
			dev_err(card->dev,
				"ASoC: Property '%s' index %d read error:%d\n",
				propname, (2 * i) + 1, ret);
			return -EINVAL;
		}

		widgets[i].name = wname;
	}

	card->of_dapm_widgets = widgets;
	card->num_of_dapm_widgets = num_widgets;

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_of_parse_audio_simple_widgets);

int snd_soc_of_parse_pin_switches(struct snd_soc_card *card, const char *prop)
{
	const unsigned int nb_controls_max = 16;
	const char **strings, *control_name;
	struct snd_kcontrol_new *controls;
	struct device *dev = card->dev;
	unsigned int i, nb_controls;
	int ret;

	if (!of_property_present(dev->of_node, prop))
		return 0;

	strings = devm_kcalloc(dev, nb_controls_max,
			       sizeof(*strings), GFP_KERNEL);
	if (!strings)
		return -ENOMEM;

	ret = of_property_read_string_array(dev->of_node, prop,
					    strings, nb_controls_max);
	if (ret < 0)
		return ret;

	nb_controls = (unsigned int)ret;

	controls = devm_kcalloc(dev, nb_controls,
				sizeof(*controls), GFP_KERNEL);
	if (!controls)
		return -ENOMEM;

	for (i = 0; i < nb_controls; i++) {
		control_name = devm_kasprintf(dev, GFP_KERNEL,
					      "%s Switch", strings[i]);
		if (!control_name)
			return -ENOMEM;

		controls[i].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
		controls[i].name = control_name;
		controls[i].info = snd_soc_dapm_info_pin_switch;
		controls[i].get = snd_soc_dapm_get_pin_switch;
		controls[i].put = snd_soc_dapm_put_pin_switch;
		controls[i].private_value = (unsigned long)strings[i];
	}

	card->controls = controls;
	card->num_controls = nb_controls;

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_of_parse_pin_switches);

int snd_soc_of_get_slot_mask(struct device_node *np,
			     const char *prop_name,
			     unsigned int *mask)
{
	u32 val;
	const __be32 *of_slot_mask = of_get_property(np, prop_name, &val);
	int i;

	if (!of_slot_mask)
		return 0;
	val /= sizeof(u32);
	for (i = 0; i < val; i++)
		if (be32_to_cpup(&of_slot_mask[i]))
			*mask |= (1 << i);

	return val;
}
EXPORT_SYMBOL_GPL(snd_soc_of_get_slot_mask);

int snd_soc_of_parse_tdm_slot(struct device_node *np,
			      unsigned int *tx_mask,
			      unsigned int *rx_mask,
			      unsigned int *slots,
			      unsigned int *slot_width)
{
	u32 val;
	int ret;

	if (tx_mask)
		snd_soc_of_get_slot_mask(np, "dai-tdm-slot-tx-mask", tx_mask);
	if (rx_mask)
		snd_soc_of_get_slot_mask(np, "dai-tdm-slot-rx-mask", rx_mask);

	ret = of_property_read_u32(np, "dai-tdm-slot-num", &val);
	if (ret && ret != -EINVAL)
		return ret;
	if (!ret && slots)
		*slots = val;

	ret = of_property_read_u32(np, "dai-tdm-slot-width", &val);
	if (ret && ret != -EINVAL)
		return ret;
	if (!ret && slot_width)
		*slot_width = val;

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_of_parse_tdm_slot);

/**
 * snd_soc_of_xlate_dai_name - Resolve a sound-dai phandle argument to a
 *                             DAI name by searching the component DAI list.
 * @component: ASoC component to search.
 * @args:      Phandle arguments from the sound-dai property; args[0] is the
 *             DAI ID.
 * @dai_name:  Output pointer set to the matched DAI name on success.
 *
 * Return: 0 on success, -EINVAL if args_count != 1 or no match is found.
 */
int snd_soc_of_xlate_dai_name(struct snd_soc_component *component,
			      const struct of_phandle_args *args,
			      const char **dai_name)
{
	struct snd_soc_dai *dai;
	int id;

	if (args->args_count != 1)
		return -EINVAL;

	id = args->args[0];

	for_each_component_dais(component, dai) {
		if (dai->driver->id == id) {
			*dai_name = snd_soc_dai_name(dai);
			return 0;
		}
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_of_xlate_dai_name);

void snd_soc_dlc_use_cpu_as_platform(struct snd_soc_dai_link_component *platforms,
				     struct snd_soc_dai_link_component *cpus)
{
	platforms->of_node	= cpus->of_node;
	platforms->dai_args	= cpus->dai_args;
}
EXPORT_SYMBOL_GPL(snd_soc_dlc_use_cpu_as_platform);

void snd_soc_of_parse_node_prefix(struct device_node *np,
				  struct snd_soc_codec_conf *codec_conf,
				  struct device_node *of_node,
				  const char *propname)
{
	const char *str;
	int ret;

	ret = of_property_read_string(np, propname, &str);
	if (ret < 0) {
		/* no prefix is not error */
		return;
	}

	codec_conf->dlc.of_node	= of_node;
	codec_conf->name_prefix	= str;
}
EXPORT_SYMBOL_GPL(snd_soc_of_parse_node_prefix);

int snd_soc_of_parse_audio_routing(struct snd_soc_card *card,
				   const char *propname)
{
	struct device_node *np = card->dev->of_node;
	int num_routes;
	struct snd_soc_dapm_route *routes;
	int i;

	num_routes = of_property_count_strings(np, propname);
	if (num_routes < 0 || num_routes & 1) {
		dev_err(card->dev,
			"ASoC: Property '%s' does not exist or its length is not even\n",
			propname);
		return -EINVAL;
	}
	num_routes /= 2;

	routes = devm_kcalloc(card->dev, num_routes, sizeof(*routes),
			      GFP_KERNEL);
	if (!routes) {
		dev_err(card->dev,
			"ASoC: Could not allocate DAPM route table\n");
		return -ENOMEM;
	}

	for (i = 0; i < num_routes; i++) {
		int ret = of_property_read_string_index(np, propname,
							2 * i, &routes[i].sink);
		if (ret) {
			dev_err(card->dev,
				"ASoC: Property '%s' index %d could not be read: %d\n",
				propname, 2 * i, ret);
			return -EINVAL;
		}
		ret = of_property_read_string_index(np, propname,
			(2 * i) + 1, &routes[i].source);
		if (ret) {
			dev_err(card->dev,
				"ASoC: Property '%s' index %d could not be read: %d\n",
				propname, (2 * i) + 1, ret);
			return -EINVAL;
		}
	}

	card->num_of_dapm_routes = num_routes;
	card->of_dapm_routes = routes;

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_of_parse_audio_routing);

int snd_soc_of_parse_aux_devs(struct snd_soc_card *card, const char *propname)
{
	struct device_node *node = card->dev->of_node;
	struct snd_soc_aux_dev *aux;
	int num, i;

	num = of_count_phandle_with_args(node, propname, NULL);
	if (num == -ENOENT) {
		return 0;
	} else if (num < 0) {
		dev_err(card->dev, "ASOC: Property '%s' could not be read: %d\n",
			propname, num);
		return num;
	}

	aux = devm_kcalloc(card->dev, num, sizeof(*aux), GFP_KERNEL);
	if (!aux)
		return -ENOMEM;
	card->aux_dev = aux;
	card->num_aux_devs = num;

	for_each_card_pre_auxs(card, i, aux) {
		aux->dlc.of_node = of_parse_phandle(node, propname, i);
		if (!aux->dlc.of_node)
			return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_of_parse_aux_devs);

int snd_soc_of_parse_ignore_suspend_widgets(struct snd_soc_card *card,
					    const char *propname)
{
	struct device_node *np = card->dev->of_node;
	int num_widgets;
	const char **widgets;
	int i;

	num_widgets = of_property_count_strings(np, propname);
	if (num_widgets < 0) {
		dev_err(card->dev,
			"ASoC: Property '%s' does not exist\n", propname);
		return -EINVAL;
	}

	widgets = devm_kcalloc(card->dev, num_widgets, sizeof(char *), GFP_KERNEL);
	if (!widgets)
		return -ENOMEM;

	for (i = 0; i < num_widgets; i++) {
		const char *name;
		int ret = of_property_read_string_index(np, propname, i, &name);

		if (ret) {
			dev_err(card->dev,
				"ASoC: Property '%s' could not be read: %d\n",
				propname, ret);
			return -EINVAL;
		}
		widgets[i] = name;
	}

	card->num_of_ignore_suspend_widgets = num_widgets;
	card->of_ignore_suspend_widgets = widgets;

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_of_parse_ignore_suspend_widgets);

unsigned int snd_soc_daifmt_clock_provider_flipped(unsigned int dai_fmt)
{
	unsigned int inv_dai_fmt = dai_fmt & ~SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK;

	switch (dai_fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_CBP_CFP:
		inv_dai_fmt |= SND_SOC_DAIFMT_CBC_CFC;
		break;
	case SND_SOC_DAIFMT_CBP_CFC:
		inv_dai_fmt |= SND_SOC_DAIFMT_CBC_CFP;
		break;
	case SND_SOC_DAIFMT_CBC_CFP:
		inv_dai_fmt |= SND_SOC_DAIFMT_CBP_CFC;
		break;
	case SND_SOC_DAIFMT_CBC_CFC:
		inv_dai_fmt |= SND_SOC_DAIFMT_CBP_CFP;
		break;
	}

	return inv_dai_fmt;
}
EXPORT_SYMBOL_GPL(snd_soc_daifmt_clock_provider_flipped);

unsigned int snd_soc_daifmt_clock_provider_from_bitmap(unsigned int bit_frame)
{
	/*
	 * bit_frame is return value from
	 *	snd_soc_daifmt_parse_clock_provider_raw()
	 */

	/* Codec base */
	switch (bit_frame) {
	case 0x11:
		return SND_SOC_DAIFMT_CBP_CFP;
	case 0x10:
		return SND_SOC_DAIFMT_CBP_CFC;
	case 0x01:
		return SND_SOC_DAIFMT_CBC_CFP;
	default:
		return SND_SOC_DAIFMT_CBC_CFC;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_daifmt_clock_provider_from_bitmap);

unsigned int snd_soc_daifmt_parse_format(struct device_node *np,
					 const char *prefix)
{
	int ret;
	char prop[128];
	unsigned int format = 0;
	int bit, frame;
	const char *str;
	struct {
		char *name;
		unsigned int val;
	} of_fmt_table[] = {
		{ "i2s",	SND_SOC_DAIFMT_I2S },
		{ "right_j",	SND_SOC_DAIFMT_RIGHT_J },
		{ "left_j",	SND_SOC_DAIFMT_LEFT_J },
		{ "dsp_a",	SND_SOC_DAIFMT_DSP_A },
		{ "dsp_b",	SND_SOC_DAIFMT_DSP_B },
		{ "ac97",	SND_SOC_DAIFMT_AC97 },
		{ "pdm",	SND_SOC_DAIFMT_PDM},
		{ "msb",	SND_SOC_DAIFMT_MSB },
		{ "lsb",	SND_SOC_DAIFMT_LSB },
	};

	if (!prefix)
		prefix = "";

	/*
	 * check "dai-format = xxx"
	 * or    "[prefix]format = xxx"
	 * SND_SOC_DAIFMT_FORMAT_MASK area
	 */
	ret = of_property_read_string(np, "dai-format", &str);
	if (ret < 0) {
		snprintf(prop, sizeof(prop), "%sformat", prefix);
		ret = of_property_read_string(np, prop, &str);
	}
	if (ret == 0) {
		int i;

		for (i = 0; i < ARRAY_SIZE(of_fmt_table); i++) {
			if (strcmp(str, of_fmt_table[i].name) == 0) {
				format |= of_fmt_table[i].val;
				break;
			}
		}
	}

	/*
	 * check "[prefix]continuous-clock"
	 * SND_SOC_DAIFMT_CLOCK_MASK area
	 */
	snprintf(prop, sizeof(prop), "%scontinuous-clock", prefix);
	if (of_property_read_bool(np, prop))
		format |= SND_SOC_DAIFMT_CONT;
	else
		format |= SND_SOC_DAIFMT_GATED;

	/*
	 * check "[prefix]bitclock-inversion"
	 * check "[prefix]frame-inversion"
	 * SND_SOC_DAIFMT_INV_MASK area
	 */
	snprintf(prop, sizeof(prop), "%sbitclock-inversion", prefix);
	bit = of_property_read_bool(np, prop);

	snprintf(prop, sizeof(prop), "%sframe-inversion", prefix);
	frame = of_property_read_bool(np, prop);

	switch ((bit << 4) + frame) {
	case 0x11:
		format |= SND_SOC_DAIFMT_IB_IF;
		break;
	case 0x10:
		format |= SND_SOC_DAIFMT_IB_NF;
		break;
	case 0x01:
		format |= SND_SOC_DAIFMT_NB_IF;
		break;
	default:
		/* SND_SOC_DAIFMT_NB_NF is default */
		break;
	}

	return format;
}
EXPORT_SYMBOL_GPL(snd_soc_daifmt_parse_format);

unsigned int snd_soc_daifmt_parse_clock_provider_raw(struct device_node *np,
						     const char *prefix,
						     struct device_node **bitclkmaster,
						     struct device_node **framemaster)
{
	char prop[128];
	unsigned int bit, frame;

	if (!np)
		return 0;

	if (!prefix)
		prefix = "";

	/*
	 * check "[prefix]bitclock-master"
	 * check "[prefix]frame-master"
	 */
	snprintf(prop, sizeof(prop), "%sbitclock-master", prefix);
	bit = of_property_present(np, prop);
	if (bit && bitclkmaster)
		*bitclkmaster = of_parse_phandle(np, prop, 0);

	snprintf(prop, sizeof(prop), "%sframe-master", prefix);
	frame = of_property_present(np, prop);
	if (frame && framemaster)
		*framemaster = of_parse_phandle(np, prop, 0);

	/*
	 * return bitmap.
	 * It will be parameter of
	 *	snd_soc_daifmt_clock_provider_from_bitmap()
	 */
	return (bit << 4) + frame;
}
EXPORT_SYMBOL_GPL(snd_soc_daifmt_parse_clock_provider_raw);

int snd_soc_get_stream_cpu(const struct snd_soc_dai_link *dai_link, int stream)
{
	/*
	 * [Normal]
	 *
	 * Playback
	 *	CPU  : SNDRV_PCM_STREAM_PLAYBACK
	 *	Codec: SNDRV_PCM_STREAM_PLAYBACK
	 *
	 * Capture
	 *	CPU  : SNDRV_PCM_STREAM_CAPTURE
	 *	Codec: SNDRV_PCM_STREAM_CAPTURE
	 */
	if (!dai_link->c2c_params)
		return stream;

	/*
	 * [Codec2Codec]
	 *
	 * Playback
	 *	CPU  : SNDRV_PCM_STREAM_CAPTURE
	 *	Codec: SNDRV_PCM_STREAM_PLAYBACK
	 *
	 * Capture
	 *	CPU  : SNDRV_PCM_STREAM_PLAYBACK
	 *	Codec: SNDRV_PCM_STREAM_CAPTURE
	 */
	if (stream == SNDRV_PCM_STREAM_CAPTURE)
		return SNDRV_PCM_STREAM_PLAYBACK;

	return SNDRV_PCM_STREAM_CAPTURE;
}
EXPORT_SYMBOL_GPL(snd_soc_get_stream_cpu);

int snd_soc_get_dai_id(struct device_node *ep)
{
	struct snd_soc_dai_link_component dlc = {
		.of_node = of_graph_get_port_parent(ep),
	};
	int ret;


	/*
	 * For example HDMI case, HDMI has video/sound port,
	 * but ALSA SoC needs sound port number only.
	 * Thus counting HDMI DT port/endpoint doesn't work.
	 * Then, it should have .of_xlate_dai_id
	 */
	ret = -ENOTSUPP;

	scoped_guard(mutex, &client_mutex) {
		struct snd_soc_component *component = snd_soc_find_component(&dlc);

		if (component)
			ret = snd_soc_component_of_xlate_dai_id(component, ep);
	}

	of_node_put(dlc.of_node);

	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_get_dai_id);

int snd_soc_get_dlc(const struct of_phandle_args *args, struct snd_soc_dai_link_component *dlc)
{
	struct snd_soc_component *pos;
	int ret = -EPROBE_DEFER;
	guard(mutex)(&client_mutex);

	for_each_component(pos) {
		struct device_node *component_of_node = snd_soc_component_to_node(pos);

		if (component_of_node != args->np || !pos->num_dai)
			continue;

		ret = snd_soc_component_of_xlate_dai_name(pos, args, &dlc->dai_name);
		if (ret == -ENOTSUPP) {
			struct snd_soc_dai *dai;
			int id = -1;

			switch (args->args_count) {
			case 0:
				id = 0; /* same as dai_drv[0] */
				break;
			case 1:
				id = args->args[0];
				break;
			default:
				/* not supported */
				break;
			}

			if (id < 0 || id >= pos->num_dai) {
				ret = -EINVAL;
				continue;
			}

			ret = 0;

			/* find target DAI */
			for_each_component_dais(pos, dai) {
				if (id == 0)
					break;
				id--;
			}

			dlc->dai_name	= snd_soc_dai_name(dai);
		} else if (ret) {
			/*
			 * if another error than ENOTSUPP is returned go on and
			 * check if another component is provided with the same
			 * node. This may happen if a device provides several
			 * components
			 */
			continue;
		}

		break;
	}

	if (ret == 0)
		dlc->of_node = args->np;

	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_get_dlc);

int snd_soc_of_get_dlc(struct device_node *of_node,
		       struct of_phandle_args *args,
		       struct snd_soc_dai_link_component *dlc,
		       int index)
{
	struct of_phandle_args __args;
	int ret;

	if (!args)
		args = &__args;

	ret = of_parse_phandle_with_args(of_node, "sound-dai",
					 "#sound-dai-cells", index, args);
	if (ret)
		return ret;

	return snd_soc_get_dlc(args, dlc);
}
EXPORT_SYMBOL_GPL(snd_soc_of_get_dlc);

int snd_soc_get_dai_name(const struct of_phandle_args *args,
			 const char **dai_name)
{
	struct snd_soc_dai_link_component dlc;
	int ret = snd_soc_get_dlc(args, &dlc);

	if (ret == 0)
		*dai_name = dlc.dai_name;

	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_get_dai_name);

int snd_soc_of_get_dai_name(struct device_node *of_node,
			    const char **dai_name, int index)
{
	struct snd_soc_dai_link_component dlc;
	int ret = snd_soc_of_get_dlc(of_node, NULL, &dlc, index);

	if (ret == 0)
		*dai_name = dlc.dai_name;

	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_of_get_dai_name);

struct snd_soc_dai *snd_soc_get_dai_via_args(const struct of_phandle_args *dai_args)
{
	struct snd_soc_dai *dai;
	struct snd_soc_component *component;
	guard(mutex)(&client_mutex);

	for_each_component(component) {
		for_each_component_dais(component, dai)
			if (snd_soc_dai_matches_args(dai, dai_args))
				return dai;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(snd_soc_get_dai_via_args);

static void __snd_soc_of_put_component(struct snd_soc_dai_link_component *component)
{
	if (component->of_node) {
		of_node_put(component->of_node);
		component->of_node = NULL;
	}
}

static int __snd_soc_of_get_dai_link_component_alloc(
	struct device *dev, struct device_node *of_node,
	struct snd_soc_dai_link_component **ret_component,
	int *ret_num)
{
	struct snd_soc_dai_link_component *component;
	int num;

	/* Count the number of CPUs/CODECs */
	num = of_count_phandle_with_args(of_node, "sound-dai", "#sound-dai-cells");
	if (num <= 0) {
		if (num == -ENOENT)
			dev_err(dev, "No 'sound-dai' property\n");
		else
			dev_err(dev, "Bad phandle in 'sound-dai'\n");
		return num;
	}
	component = devm_kcalloc(dev, num, sizeof(*component), GFP_KERNEL);
	if (!component)
		return -ENOMEM;

	*ret_component	= component;
	*ret_num	= num;

	return 0;
}

/*
 * snd_soc_of_put_dai_link_codecs - Dereference device nodes in the codecs array
 * @dai_link: DAI link
 *
 * Dereference device nodes acquired by snd_soc_of_get_dai_link_codecs().
 */
void snd_soc_of_put_dai_link_codecs(struct snd_soc_dai_link *dai_link)
{
	struct snd_soc_dai_link_component *component;
	int index;

	for_each_link_codecs(dai_link, index, component)
		__snd_soc_of_put_component(component);
}
EXPORT_SYMBOL_GPL(snd_soc_of_put_dai_link_codecs);

/*
 * snd_soc_of_get_dai_link_codecs - Parse a list of CODECs in the devicetree
 * @dev: Card device
 * @of_node: Device node
 * @dai_link: DAI link
 *
 * Builds an array of CODEC DAI components from the DAI link property
 * 'sound-dai'.
 * The array is set in the DAI link and the number of DAIs is set accordingly.
 * The device nodes in the array (of_node) must be dereferenced by calling
 * snd_soc_of_put_dai_link_codecs() on @dai_link.
 *
 * Returns 0 for success
 */
int snd_soc_of_get_dai_link_codecs(struct device *dev,
				   struct device_node *of_node,
				   struct snd_soc_dai_link *dai_link)
{
	struct snd_soc_dai_link_component *component;
	int index, ret;

	ret = __snd_soc_of_get_dai_link_component_alloc(dev, of_node,
					 &dai_link->codecs, &dai_link->num_codecs);
	if (ret < 0)
		return ret;

	/* Parse the list */
	for_each_link_codecs(dai_link, index, component) {
		ret = snd_soc_of_get_dlc(of_node, NULL, component, index);
		if (ret)
			goto err;
	}
	return 0;
err:
	snd_soc_of_put_dai_link_codecs(dai_link);
	dai_link->codecs = NULL;
	dai_link->num_codecs = 0;
	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_of_get_dai_link_codecs);

/*
 * snd_soc_of_put_dai_link_cpus - Dereference device nodes in the codecs array
 * @dai_link: DAI link
 *
 * Dereference device nodes acquired by snd_soc_of_get_dai_link_cpus().
 */
void snd_soc_of_put_dai_link_cpus(struct snd_soc_dai_link *dai_link)
{
	struct snd_soc_dai_link_component *component;
	int index;

	for_each_link_cpus(dai_link, index, component)
		__snd_soc_of_put_component(component);
}
EXPORT_SYMBOL_GPL(snd_soc_of_put_dai_link_cpus);

/*
 * snd_soc_of_get_dai_link_cpus - Parse a list of CPU DAIs in the devicetree
 * @dev: Card device
 * @of_node: Device node
 * @dai_link: DAI link
 *
 * Is analogous to snd_soc_of_get_dai_link_codecs but parses a list of CPU DAIs
 * instead.
 *
 * Returns 0 for success
 */
int snd_soc_of_get_dai_link_cpus(struct device *dev,
				 struct device_node *of_node,
				 struct snd_soc_dai_link *dai_link)
{
	struct snd_soc_dai_link_component *component;
	int index, ret;

	/* Count the number of CPUs */
	ret = __snd_soc_of_get_dai_link_component_alloc(dev, of_node,
					 &dai_link->cpus, &dai_link->num_cpus);
	if (ret < 0)
		return ret;

	/* Parse the list */
	for_each_link_cpus(dai_link, index, component) {
		ret = snd_soc_of_get_dlc(of_node, NULL, component, index);
		if (ret)
			goto err;
	}
	return 0;
err:
	snd_soc_of_put_dai_link_cpus(dai_link);
	dai_link->cpus = NULL;
	dai_link->num_cpus = 0;
	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_of_get_dai_link_cpus);

static int __init snd_soc_init(void)
{
	int ret;

	snd_soc_debugfs_init();
	ret = snd_soc_util_init();
	if (ret)
		goto err_util_init;

	ret = platform_driver_register(&soc_driver);
	if (ret)
		goto err_register;
	return 0;

err_register:
	snd_soc_util_exit();
err_util_init:
	snd_soc_debugfs_exit();
	return ret;
}
module_init(snd_soc_init);

static void __exit snd_soc_exit(void)
{
	snd_soc_util_exit();
	snd_soc_debugfs_exit();

	platform_driver_unregister(&soc_driver);
}
module_exit(snd_soc_exit);

/* Module information */
MODULE_AUTHOR("Liam Girdwood, lrg@slimlogic.co.uk");
MODULE_DESCRIPTION("ALSA SoC Core");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:soc-audio");
