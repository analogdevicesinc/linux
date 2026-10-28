// SPDX-License-Identifier: GPL-2.0
//
// soc-card.c
//
// Copyright (C) 2019 Renesas Electronics Corp.
// Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
//

#include <linux/debugfs.h>
#include <linux/lockdep.h>
#include <linux/rwsem.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "soc-internal.h"

#define soc_card_ret(dai, ret) _soc_card_ret(dai, __func__, ret)
static inline int _soc_card_ret(struct snd_soc_card *card,
				const char *func, int ret)
{
	return snd_soc_ret(card->dev, ret,
			   "at %s() on %s\n", func, card->name);
}

#ifdef CONFIG_DEBUG_FS
void snd_soc_card_debugfs_init(struct snd_soc_card *card)
{
	card->debugfs_card_root = debugfs_create_dir(card->name,
						     snd_soc_debugfs_root);

	snd_soc_dapm_debugfs_init(snd_soc_card_to_dapm(card), card->debugfs_card_root);
}

void snd_soc_card_debugfs_cleanup(struct snd_soc_card *card)
{
	debugfs_remove_recursive(card->debugfs_card_root);
	card->debugfs_card_root = NULL;
}
#else
void snd_soc_card_debugfs_init(struct snd_soc_card *card) { }
void snd_soc_card_debugfs_cleanup(struct snd_soc_card *card) { }
#endif /* CONFIG_DEBUG_FS */

#ifdef CONFIG_PM_SLEEP
/*
 * deferred resume work, so resume can complete before we finished
 * setting our codec back up, which can be very slow on I2C
 */
static void snd_soc_card_resume_deferred(struct work_struct *work)
{
	struct snd_soc_card *card =
		container_of(work, struct snd_soc_card,
			     deferred_resume_work);
	struct snd_soc_component *component;

	/*
	 * our power state is still SNDRV_CTL_POWER_D3hot from suspend time,
	 * so userspace apps are blocked from touching us
	 */

	dev_dbg(card->dev, "ASoC: starting resume work\n");

	/* Bring us up into D2 so that DAPM starts enabling things */
	snd_power_change_state(card->snd_card, SNDRV_CTL_POWER_D2);

	snd_soc_card_resume_pre(card);

	for_each_card_components(card, component) {
		if (snd_soc_component_is_suspended(component))
			snd_soc_component_resume(component);
	}

	snd_soc_dapm_suspend_resume(card, SND_SOC_DAPM_STREAM_RESUME);

	/* unmute any active DACs */
	snd_soc_playback_digital_mute(card, 0);

	snd_soc_card_resume_post(card);

	dev_dbg(card->dev, "ASoC: resume work completed\n");

	/* Recheck all endpoints too, their state is affected by suspend */
	snd_soc_dapm_mark_endpoints_dirty(card);
	snd_soc_dapm_sync(snd_soc_card_to_dapm(card));

	/* userspace can access us now we are back as we were before */
	snd_power_change_state(card->snd_card, SNDRV_CTL_POWER_D0);
}

void snd_soc_card_resume_init(struct snd_soc_card *card)
{
	/* deferred resume work */
	INIT_WORK(&card->deferred_resume_work, snd_soc_card_resume_deferred);
}
#else
void snd_soc_card_resume_init(struct snd_soc_card *card) { }
#endif /* CONFIG_PM_SLEEP */

struct snd_kcontrol *snd_soc_card_get_kcontrol(struct snd_soc_card *soc_card,
					       const char *name)
{
	if (unlikely(!name))
		return NULL;

	return snd_ctl_find_id_mixer(soc_card->snd_card, name);
}
EXPORT_SYMBOL_GPL(snd_soc_card_get_kcontrol);

static int jack_new(struct snd_soc_card *card, const char *id, int type,
		    struct snd_soc_jack *jack, bool initial_kctl)
{
	mutex_init(&jack->mutex);
	jack->card = card;
	INIT_LIST_HEAD(&jack->pins);
	INIT_LIST_HEAD(&jack->jack_zones);
	BLOCKING_INIT_NOTIFIER_HEAD(&jack->notifier);

	return snd_jack_new(card->snd_card, id, type, &jack->jack, initial_kctl, false);
}

/**
 * snd_soc_card_jack_new - Create a new jack without pins
 * @card:  ASoC card
 * @id:    an identifying string for this jack
 * @type:  a bitmask of enum snd_jack_type values that can be detected by
 *         this jack
 * @jack:  structure to use for the jack
 *
 * Creates a new jack object without pins. If adding pins later,
 * snd_soc_card_jack_new_pins() should be used instead with 0 as num_pins
 * argument.
 *
 * Returns zero if successful, or a negative error code on failure.
 * On success jack will be initialised.
 */
int snd_soc_card_jack_new(struct snd_soc_card *card, const char *id, int type,
			  struct snd_soc_jack *jack)
{
	return soc_card_ret(card, jack_new(card, id, type, jack, true));
}
EXPORT_SYMBOL_GPL(snd_soc_card_jack_new);

/**
 * snd_soc_card_jack_new_pins - Create a new jack with pins
 * @card:  ASoC card
 * @id:    an identifying string for this jack
 * @type:  a bitmask of enum snd_jack_type values that can be detected by
 *         this jack
 * @jack:  structure to use for the jack
 * @pins:  Array of jack pins to be added to the jack or NULL
 * @num_pins: Number of elements in the @pins array
 *
 * Creates a new jack object with pins. If not adding pins,
 * snd_soc_card_jack_new() should be used instead.
 *
 * Returns zero if successful, or a negative error code on failure.
 * On success jack will be initialised.
 */
int snd_soc_card_jack_new_pins(struct snd_soc_card *card, const char *id,
			       int type, struct snd_soc_jack *jack,
			       struct snd_soc_jack_pin *pins,
			       unsigned int num_pins)
{
	int ret;

	ret = jack_new(card, id, type, jack, false);
	if (ret)
		goto end;

	if (num_pins)
		ret = snd_soc_jack_add_pins(jack, num_pins, pins);
end:
	return soc_card_ret(card, ret);
}
EXPORT_SYMBOL_GPL(snd_soc_card_jack_new_pins);

int snd_soc_card_suspend_pre(struct snd_soc_card *card)
{
	int ret = 0;

	if (card->suspend_pre)
		ret = card->suspend_pre(card);

	return soc_card_ret(card, ret);
}

int snd_soc_card_suspend_post(struct snd_soc_card *card)
{
	int ret = 0;

	if (card->suspend_post)
		ret = card->suspend_post(card);

	return soc_card_ret(card, ret);
}

int snd_soc_card_resume_pre(struct snd_soc_card *card)
{
	int ret = 0;

	if (card->resume_pre)
		ret = card->resume_pre(card);

	return soc_card_ret(card, ret);
}

int snd_soc_card_resume_post(struct snd_soc_card *card)
{
	int ret = 0;

	if (card->resume_post)
		ret = card->resume_post(card);

	return soc_card_ret(card, ret);
}

int snd_soc_card_probe(struct snd_soc_card *card)
{
	if (card->probe) {
		int ret = card->probe(card);

		if (ret < 0)
			return soc_card_ret(card, ret);

		/*
		 * It has "card->probe" and "card->late_probe" callbacks.
		 * So, set "probed" flag here, because it needs to care
		 * about "late_probe".
		 *
		 * see
		 *	snd_soc_bind_card()
		 *	snd_soc_card_late_probe()
		 */
		card->probed = 1;
	}

	return 0;
}

int snd_soc_card_late_probe(struct snd_soc_card *card)
{
	if (card->late_probe) {
		int ret = card->late_probe(card);

		if (ret < 0)
			return soc_card_ret(card, ret);
	}

	/*
	 * It has "card->probe" and "card->late_probe" callbacks,
	 * and "late_probe" callback is called after "probe".
	 * This means, we can set "card->probed" flag afer "late_probe"
	 * for all cases.
	 *
	 * see
	 *	snd_soc_bind_card()
	 *	snd_soc_card_probe()
	 */
	card->probed = 1;

	return 0;
}

void snd_soc_card_fixup_controls(struct snd_soc_card *card)
{
	if (card->fixup_controls)
		card->fixup_controls(card);
}

int snd_soc_card_remove(struct snd_soc_card *card)
{
	int ret = 0;

	if (card->probed &&
	    card->remove)
		ret = card->remove(card);

	card->probed = 0;

	return soc_card_ret(card, ret);
}

int snd_soc_card_set_bias_level(struct snd_soc_card *card,
				struct snd_soc_dapm_context *dapm,
				enum snd_soc_bias_level level)
{
	int ret = 0;

	if (card->set_bias_level)
		ret = card->set_bias_level(card, dapm, level);

	return soc_card_ret(card, ret);
}

int snd_soc_card_set_bias_level_post(struct snd_soc_card *card,
				     struct snd_soc_dapm_context *dapm,
				     enum snd_soc_bias_level level)
{
	int ret = 0;

	if (card->set_bias_level_post)
		ret = card->set_bias_level_post(card, dapm, level);

	return soc_card_ret(card, ret);
}

int snd_soc_card_add_dai_link(struct snd_soc_card *card,
			      struct snd_soc_dai_link *dai_link)
{
	int ret = 0;

	if (card->add_dai_link)
		ret = card->add_dai_link(card, dai_link);

	return soc_card_ret(card, ret);
}
EXPORT_SYMBOL_GPL(snd_soc_card_add_dai_link);

void snd_soc_card_remove_dai_link(struct snd_soc_card *card,
				  struct snd_soc_dai_link *dai_link)
{
	if (card->remove_dai_link)
		card->remove_dai_link(card, dai_link);
}
EXPORT_SYMBOL_GPL(snd_soc_card_remove_dai_link);

void snd_soc_card_set_topology_name(struct snd_soc_card *card, const char *prefix)
{
	if (!prefix || !card->name)
		return;

	if (!card->topology_shortname)
		card->topology_shortname = devm_kasprintf(card->dev, GFP_KERNEL,
							  "%s-%s", prefix, card->name);

	card->name = card->topology_shortname;
}
EXPORT_SYMBOL_GPL(snd_soc_card_set_topology_name);

/**
 * snd_soc_card_add_controls - add an array of controls to a SoC card.
 * Convenience function to add a list of controls.
 *
 * @soc_card: SoC card to add controls to
 * @controls: array of controls to add
 * @num_controls: number of elements in the array
 *
 * Return 0 for success, else error.
 */
int snd_soc_card_add_controls(struct snd_soc_card *soc_card,
			      const struct snd_kcontrol_new *controls, int num_controls)
{
	struct snd_card *card = soc_card->snd_card;

	return snd_soc_add_controls(card, soc_card->dev, controls, num_controls, NULL, soc_card);
}
EXPORT_SYMBOL_GPL(snd_soc_card_add_controls);
