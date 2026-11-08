// SPDX-License-Identifier: GPL-2.0
//
// soc-card.c
//
// Copyright (C) 2019 Renesas Electronics Corp.
// Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
//

#include <linux/debugfs.h>
#include <linux/dmi.h>
#include <linux/lockdep.h>
#include <linux/pinctrl/consumer.h>
#include <linux/rwsem.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-link.h>
#include <sound/jack.h>
#include "soc-internal.h"

static LIST_HEAD(unbind_card_list);

#define soc_card_ret(dai, ret) _soc_card_ret(dai, __func__, ret)
static inline int _soc_card_ret(struct snd_soc_card *card,
				const char *func, int ret)
{
	return snd_soc_ret(card->dev, ret,
			   "at %s() on %s\n", func, card->name);
}

#ifdef CONFIG_DEBUG_FS
static void snd_soc_card_debugfs_init(struct snd_soc_card *card)
{
	card->debugfs_card_root = debugfs_create_dir(card->name,
						     snd_soc_debugfs_root);

	snd_soc_dapm_debugfs_init(snd_soc_card_to_dapm(card), card->debugfs_card_root);
}

static void snd_soc_card_debugfs_cleanup(struct snd_soc_card *card)
{
	debugfs_remove_recursive(card->debugfs_card_root);
	card->debugfs_card_root = NULL;
}
#else
static inline void snd_soc_card_debugfs_init(struct snd_soc_card *card) { }
static inline void snd_soc_card_debugfs_cleanup(struct snd_soc_card *card) { }
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

static void snd_soc_card_resume_init(struct snd_soc_card *card)
{
	/* deferred resume work */
	INIT_WORK(&card->deferred_resume_work, snd_soc_card_resume_deferred);
}
#else
static inline void snd_soc_card_resume_init(struct snd_soc_card *card) { }
#endif /* CONFIG_PM_SLEEP */

static void snd_soc_card_fill_dummy_dai(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *dai_link;
	int i;

	/*
	 * COMP_DUMMY() creates size 0 array on dai_link.
	 * Fill it as dummy DAI in case of CPU/Codec here.
	 * Do nothing for Platform.
	 */
	for_each_card_prelinks(card, i, dai_link) {
		if (dai_link->num_cpus == 0 && dai_link->cpus) {
			dai_link->num_cpus	= 1;
			dai_link->cpus		= &snd_soc_dummy_dlc;
		}
		if (dai_link->num_codecs == 0 && dai_link->codecs) {
			dai_link->num_codecs	= 1;
			dai_link->codecs	= &snd_soc_dummy_dlc;
		}
	}
}

static int snd_soc_card_init_pcm_runtime(struct snd_soc_card *card,
					 struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai_link *dai_link = rtd->dai_link;
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int ret;

	/* do machine specific initialization */
	ret = snd_soc_link_init(rtd);
	if (ret < 0)
		return ret;

	ret = snd_soc_runtime_set_dai_fmt(rtd, snd_soc_dai_auto_select_format(rtd));
	if (ret)
		goto err;

	/* add DPCM sysfs entries */
	soc_dpcm_debugfs_add(rtd);

	/* create compress_device if driver is supporting */
	ret = snd_soc_dai_compress_new(cpu_dai, rtd);
	if (ret != -ENOTSUPP) { /* driver is supporting */
		if (ret < 0)
			goto err;
		goto end;
	}

	/* create the pcm */
	ret = soc_new_pcm(rtd);
	if (ret < 0) {
		dev_err(card->dev, "ASoC: can't create pcm %s :%d\n",
			dai_link->stream_name, ret);
		goto err;
	}

	ret = snd_soc_pcm_dai_new(rtd);
	if (ret < 0)
		goto err;
end:
	rtd->initialized = true;

	return 0;
err:
	snd_soc_link_exit(rtd);

	return ret;
}

static void snd_soc_card_link_dais_remove(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd;
	int order;

	for_each_comp_order(order) {
		for_each_card_rtds(card, rtd) {
			/* remove all rtd connected DAIs in good order */
			snd_soc_pcm_dai_remove(rtd, order);
		}
	}
}

static int snd_soc_card_link_dais_probe(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd;
	int order, ret;

	for_each_comp_order(order) {
		for_each_card_rtds(card, rtd) {
			/* probe all rtd connected DAIs in good order */
			ret = snd_soc_pcm_dai_probe(rtd, order);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static void snd_soc_card_link_components_remove(struct snd_soc_card *card)
{
	struct snd_soc_component *component;
	struct snd_soc_pcm_runtime *rtd;
	int i, order;

	for_each_comp_order(order) {
		for_each_card_rtds(card, rtd) {
			for_each_rtd_components(rtd, i, component) {
				if (component->driver->remove_order != order)
					continue;

				snd_soc_component_remove(component, 1);
			}
		}
	}
}

static int snd_soc_card_link_components_probe(struct snd_soc_card *card)
{
	struct snd_soc_component *component;
	struct snd_soc_pcm_runtime *rtd;
	int i, ret, order;

	for_each_comp_order(order) {
		for_each_card_rtds(card, rtd) {
			for_each_rtd_components(rtd, i, component) {
				if (component->driver->probe_order != order)
					continue;

				ret = snd_soc_component_probe(component, card);
				if (ret < 0)
					return ret;
			}
		}
	}

	return 0;
}

static void snd_soc_card_aux_unbind(struct snd_soc_card *card)
{
	struct snd_soc_component *component, *_component;

	for_each_card_auxs_safe(card, component, _component) {
		/* for snd_soc_component_init() */
		snd_soc_component_set_aux(component, NULL);
		list_del(&component->card_aux_list);
	}
}

static int snd_soc_card_aux_bind(struct snd_soc_card *card)
{
	struct snd_soc_component *component;
	struct snd_soc_aux_dev *aux;
	int i;

	for_each_card_pre_auxs(card, i, aux) {
		/* codecs, usually analog devices */
		component = snd_soc_find_component(&aux->dlc);
		if (!component)
			return -EPROBE_DEFER;

		/* for snd_soc_component_init() */
		snd_soc_component_set_aux(component, aux);
		/* see for_each_card_auxs */
		list_add(&component->card_aux_list, &card->aux_comp_list);
	}
	return 0;
}

static int snd_soc_card_aux_probe(struct snd_soc_card *card)
{
	struct snd_soc_component *component;
	int order;
	int ret;

	for_each_comp_order(order) {
		for_each_card_auxs(card, component) {
			if (component->driver->probe_order != order)
				continue;

			ret = snd_soc_component_probe(component, card);
			if (ret < 0)
				return ret;
		}
	}

	return 0;
}

static void snd_soc_card_aux_remove(struct snd_soc_card *card)
{
	struct snd_soc_component *comp, *_comp;
	int order;

	for_each_comp_order(order) {
		for_each_card_auxs_safe(card, comp, _comp) {
			if (comp->driver->remove_order == order)
				snd_soc_component_remove(comp, 1);
		}
	}
}

#ifdef CONFIG_DMI
/*
 * If a DMI filed contain strings in this blacklist (e.g.
 * "Type2 - Board Manufacturer" or "Type1 - TBD by OEM"), it will be taken
 * as invalid and dropped when setting the card long name from DMI info.
 */
static const char * const dmi_blacklist[] = {
	"To be filled by OEM",
	"TBD by OEM",
	"Default String",
	"Board Manufacturer",
	"Board Vendor Name",
	"Board Product Name",
	NULL,	/* terminator */
};

/*
 * Trim special characters, and replace '-' with '_' since '-' is used to
 * separate different DMI fields in the card long name. Only number and
 * alphabet characters and a few separator characters are kept.
 */
static void cleanup_dmi_name(char *name)
{
	int i, j = 0;

	for (i = 0; name[i]; i++) {
		if (isalnum(name[i]) || (name[i] == '.')
		    || (name[i] == '_'))
			name[j++] = name[i];
		else if (name[i] == '-')
			name[j++] = '_';
	}

	name[j] = '\0';
}

/*
 * Check if a DMI field is valid, i.e. not containing any string
 * in the black list and not the empty string.
 */
static int is_dmi_valid(const char *field)
{
	int i = 0;

	if (!field[0])
		return 0;

	while (dmi_blacklist[i]) {
		if (strstr(field, dmi_blacklist[i]))
			return 0;
		i++;
	}

	return 1;
}

/*
 * Append a string to dmi_longname with character cleanups.
 */
#define DMI_LONGNAME_LEN	80
static void append_dmi_string(char *dst, const char *str)
{
	size_t dst_len = DMI_LONGNAME_LEN;
	size_t len;

	len = strlen(dst);
	snprintf(dst + len, dst_len - len, "-%s", str);

	len++;	/* skip the separator "-" */
	if (len < dst_len)
		cleanup_dmi_name(dst + len);
}

/**
 * snd_soc_card_set_dmi_name() - Register DMI names to card
 * @card: The card to register DMI names
 *
 * An Intel machine driver may be used by many different devices but are
 * difficult for userspace to differentiate, since machine drivers usually
 * use their own name as the card short name and leave the card long name
 * blank. To differentiate such devices and fix bugs due to lack of
 * device-specific configurations, this function allows DMI info to be used
 * as the sound card long name, in the format of
 * "vendor-product-version-board"
 * (Character '-' is used to separate different DMI fields here).
 * This will help the user space to load the device-specific Use Case Manager
 * (UCM) configurations for the card.
 *
 * Possible card long names may be:
 * DellInc.-XPS139343-01-0310JH
 * ASUSTeKCOMPUTERINC.-T100TA-1.0-T100TA
 * Circuitco-MinnowboardMaxD0PLATFORM-D0-MinnowBoardMAX
 *
 * This function also supports flavoring the card longname to provide
 * the extra differentiation, like "vendor-product-version-board-flavor".
 *
 * We only keep number and alphabet characters and a few separator characters
 * in the card long name since UCM in the user space uses the card long names
 * as card configuration directory names and AudoConf cannot support special
 * characters like SPACE.
 *
 * Returns 0 on success, otherwise a negative error code.
 */
static int snd_soc_card_set_dmi_name(struct snd_soc_card *card)
{
	const char *vendor, *product, *board;
	char *dmi_longname;

	if (card->long_name)
		return 0; /* long name already set by driver or from DMI */

	if (!dmi_available)
		return 0;

	/* make up dmi long name as: vendor-product-version-board */
	vendor = dmi_get_system_info(DMI_BOARD_VENDOR);
	if (!vendor || !is_dmi_valid(vendor)) {
		dev_warn(card->dev, "ASoC: no DMI vendor name!\n");
		return 0;
	}

	dmi_longname = devm_kzalloc(card->dev, DMI_LONGNAME_LEN, GFP_KERNEL);
	if (!dmi_longname)
		return -ENOMEM;

	snprintf(dmi_longname, DMI_LONGNAME_LEN, "%s", vendor);
	cleanup_dmi_name(dmi_longname);

	product = dmi_get_system_info(DMI_PRODUCT_NAME);
	if (product && is_dmi_valid(product)) {
		const char *product_version = dmi_get_system_info(DMI_PRODUCT_VERSION);

		append_dmi_string(dmi_longname, product);

		/*
		 * some vendors like Lenovo may only put a self-explanatory
		 * name in the product version field
		 */
		if (product_version && is_dmi_valid(product_version))
			append_dmi_string(dmi_longname, product_version);
	}

	board = dmi_get_system_info(DMI_BOARD_NAME);
	if (board && is_dmi_valid(board)) {
		if (!product || strcasecmp(board, product))
			append_dmi_string(dmi_longname, board);
	} else if (!product) {
		/* fall back to using legacy name */
		dev_warn(card->dev, "ASoC: no DMI board/product name!\n");
		return 0;
	}

	/* set the card long name */
	card->long_name = dmi_longname;

	return 0;
}
#else
static inline int snd_soc_card_set_dmi_name(struct snd_soc_card *card)
{
	return 0;
}
#endif /* CONFIG_DMI */

static void soc_card_check_tplg_fes(struct snd_soc_card *card)
{
	struct snd_soc_component *component;
	struct snd_soc_dai_link *dai_link;
	int i;

	for_each_component(component) {

		/* does this component override BEs ? */
		if (!component->driver->ignore_machine)
			continue;

		/* for this machine ? */
		if (!strcmp(component->driver->ignore_machine,
			    card->dev->driver->name))
			goto match;
		if (strcmp(component->driver->ignore_machine,
			   dev_name(card->dev)))
			continue;
match:
		/* machine matches, so override the rtd data */
		for_each_card_prelinks(card, i, dai_link) {

			/* ignore this FE */
			if (dai_link->dynamic) {
				dai_link->ignore = true;
				continue;
			}

			dev_dbg(card->dev, "info: override BE DAI link %s\n",
				dai_link->name);

			/* override platform component */
			if (!dai_link->platforms) {
				dev_err(card->dev, "init platform error");
				continue;
			}

			if (component->dev->of_node)
				dai_link->platforms->of_node = component->dev->of_node;
			else
				dai_link->platforms->name = component->name;

			/* convert non BE into BE */
			dai_link->no_pcm = 1;

			/*
			 * override any BE fixups
			 * see
			 *	snd_soc_link_be_hw_params_fixup()
			 */
			dai_link->be_hw_params_fixup =
				component->driver->be_hw_params_fixup;

			/*
			 * most BE links don't set stream name, so set it to
			 * dai link name if it's NULL to help bind widgets.
			 */
			if (!dai_link->stream_name)
				dai_link->stream_name = dai_link->name;
		}

		/* Inform userspace we are using alternate topology */
		snd_soc_card_set_topology_name(card, component->driver->topology_name_prefix);
	}
}

#define soc_card_setup_name(card, name, name1, name2)			\
	__soc_card_setup_name(card, name, sizeof(name), name1, name2)
static void __soc_card_setup_name(struct snd_soc_card *card,
				  char *name, int len,
				  const char *name1, const char *name2)
{
	const char *src = name1 ? name1 : name2;
	int i;

	snprintf(name, len, "%s", src);

	if (name != card->snd_card->driver)
		return;

	/*
	 * Name normalization (driver field)
	 *
	 * The driver name is somewhat special, as it's used as a key for
	 * searches in the user-space.
	 *
	 * ex)
	 *	"abcd??efg" -> "abcd__efg"
	 */
	for (i = 0; i < len; i++) {
		switch (name[i]) {
		case '_':
		case '-':
		case '\0':
			break;
		default:
			if (!isalnum(name[i]))
				name[i] = '_';
			break;
		}
	}

	/*
	 * The driver field should contain a valid string from the user view.
	 * The wrapping usually does not work so well here. Set a smaller string
	 * in the specific ASoC driver.
	 */
	if (strlen(src) > len - 1)
		dev_err(card->dev, "ASoC: driver name too long '%s' -> '%s'\n", src, name);
}

static void snd_soc_card_cleanup_resources(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd, *n;

	if (card->snd_card)
		snd_card_disconnect_sync(card->snd_card);

	snd_soc_dapm_shutdown(card);

	/* release machine specific resources */
	for_each_card_rtds(card, rtd)
		if (rtd->initialized)
			snd_soc_link_exit(rtd);
	/* flush delayed work before removing DAIs and DAPM widgets */
	snd_soc_flush_all_delayed_work(card);

	/* remove and free each DAI */
	snd_soc_card_link_dais_remove(card);
	snd_soc_card_link_components_remove(card);

	for_each_card_rtds_safe(card, rtd, n)
		snd_soc_remove_pcm_runtime(card, rtd);

	/* remove auxiliary devices */
	snd_soc_card_aux_remove(card);
	snd_soc_card_aux_unbind(card);

	snd_soc_dapm_free(snd_soc_card_to_dapm(card));
	snd_soc_card_debugfs_cleanup(card);

	/* remove the card */
	snd_soc_card_remove(card);

	if (card->snd_card) {
		snd_card_free(card->snd_card);
		card->snd_card = NULL;
	}
}

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
		 *	snd_soc_card_bind()
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
	 *	snd_soc_card_bind()
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

static void snd_soc_remove_device_links(struct snd_soc_card *card)
{
	struct snd_soc_component *component;

	for_each_card_components(card, component) {
		if (component->card_device_link) {
			device_link_del(component->card_device_link);
			component->card_device_link = NULL;
		}
	}
}

void snd_soc_card_unbind(struct snd_soc_card *card, bool reuse)
{
	if (snd_soc_card_is_instantiated(card)) {
		card->instantiated = false;

		snd_soc_remove_device_links(card);

		snd_soc_card_cleanup_resources(card);

		if (reuse)
			list_add(&card->list, &unbind_card_list);
	}

	if (!reuse)
		list_del(&card->list);
}

static int snd_soc_card_bind(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_component *component;
	struct snd_soc_dapm_context *dapm = snd_soc_card_to_dapm(card);
	int ret;

	snd_soc_card_mutex_lock_root(card);
	snd_soc_card_fill_dummy_dai(card);

	snd_soc_dapm_init(dapm, card, NULL);
	list_del_init(&card->list);

	/* check whether any platform is ignore machine FE and using topology */
	soc_card_check_tplg_fes(card);

	/* bind aux_devs too */
	ret = snd_soc_card_aux_bind(card);
	if (ret < 0)
		goto probe_end;

	/* add predefined DAI links to the list */
	card->num_rtd = 0;
	ret = snd_soc_add_pcm_runtimes(card, card->dai_link, card->num_links);
	if (ret < 0)
		goto probe_end;

	/* card bind complete so register a sound card */
	ret = snd_card_new(card->dev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
			   card->owner, 0, &card->snd_card);
	if (ret < 0) {
		dev_err(card->dev,
			"ASoC: can't create sound card for card %s: %d\n",
			card->name, ret);
		goto probe_end;
	}

	snd_soc_card_debugfs_init(card);

	snd_soc_card_resume_init(card);

	ret = snd_soc_dapm_new_controls(dapm, card->dapm_widgets,
					card->num_dapm_widgets);
	if (ret < 0)
		goto probe_end;

	ret = snd_soc_dapm_new_controls(dapm, card->of_dapm_widgets,
					card->num_of_dapm_widgets);
	if (ret < 0)
		goto probe_end;

	/* initialise the sound card only once */
	ret = snd_soc_card_probe(card);
	if (ret < 0)
		goto probe_end;

	/* probe all components used by DAI links on this card */
	ret = snd_soc_card_link_components_probe(card);
	if (ret < 0) {
		if (ret != -EPROBE_DEFER) {
			dev_err(card->dev,
				"ASoC: failed to instantiate card %d\n", ret);
		}
		goto probe_end;
	}

	/* probe auxiliary components */
	ret = snd_soc_card_aux_probe(card);
	if (ret < 0) {
		dev_err(card->dev,
			"ASoC: failed to probe aux component %d\n", ret);
		goto probe_end;
	}

	/* probe all DAI links on this card */
	ret = snd_soc_card_link_dais_probe(card);
	if (ret < 0) {
		dev_err(card->dev,
			"ASoC: failed to instantiate card %d\n", ret);
		goto probe_end;
	}

	for_each_card_rtds(card, rtd) {
		ret = snd_soc_card_init_pcm_runtime(card, rtd);
		if (ret < 0)
			goto probe_end;
	}

	snd_soc_dapm_link_dai_widgets(card);
	snd_soc_dapm_connect_dai_link_widgets(card);

	ret = snd_soc_add_card_controls(card, card->controls,
					card->num_controls);
	if (ret < 0)
		goto probe_end;

	ret = snd_soc_dapm_add_routes(dapm, card->dapm_routes,
				      card->num_dapm_routes);
	if (ret < 0)
		goto probe_end;

	ret = snd_soc_dapm_add_routes(dapm, card->of_dapm_routes,
				      card->num_of_dapm_routes);
	if (ret < 0)
		goto probe_end;

	/* try to set some sane longname if DMI is available */
	snd_soc_card_set_dmi_name(card);

	soc_card_setup_name(card, card->snd_card->shortname,	card->name,		NULL);
	soc_card_setup_name(card, card->snd_card->longname,	card->long_name,	card->name);
	soc_card_setup_name(card, card->snd_card->driver,	card->driver_name,	card->name);

	if (card->components) {
		/* the current implementation of snd_component_add() accepts */
		/* multiple components in the string separated by space, */
		/* but the string collision (identical string) check might */
		/* not work correctly */
		ret = snd_component_add(card->snd_card, card->components);
		if (ret < 0) {
			dev_err(card->dev, "ASoC: %s snd_component_add() failed: %d\n",
				card->name, ret);
			goto probe_end;
		}
	}

	/*
	 * Add device_link from card to component so that system_suspend
	 * will be done in the correct order. The card must suspend first
	 * to stop audio activity before the components suspend.
	 *
	 * If a driver pair already have a link in the opposite direction
	 * they must manage their own suspend order.
	 */
	for_each_card_components(card, component) {
		if (card->dev == component->dev)
			continue;

		component->card_device_link = device_link_add(card->dev,
							      component->dev,
							      DL_FLAG_STATELESS);
		if (!component->card_device_link) {
			dev_warn(card->dev, "Could not create device link to %s\n",
				 dev_name(component->dev));
		}
	}

	ret = snd_soc_card_late_probe(card);
	if (ret < 0)
		goto probe_end;

	ret = snd_soc_dapm_ignore_suspend_widgets(card);
	if (ret < 0)
		goto probe_end;

	snd_soc_dapm_new_widgets(card);
	for_each_card_components(card, component) {
		ret = snd_soc_component_fixup_controls(component);
		if (ret < 0)
			goto probe_end;
	}
	snd_soc_card_fixup_controls(card);

	ret = snd_card_register(card->snd_card);
	if (ret < 0) {
		dev_err(card->dev, "ASoC: failed to register soundcard %d\n",
			ret);
		goto probe_end;
	}

	card->instantiated = 1;
	snd_soc_dapm_mark_endpoints_dirty(card);
	snd_soc_dapm_sync(dapm);

	/* deactivate pins to sleep state */
	for_each_card_components(card, component)
		if (!snd_soc_component_active(component))
			pinctrl_pm_select_sleep_state(component->dev);

probe_end:
	if (ret < 0) {
		snd_soc_remove_device_links(card);
		snd_soc_card_cleanup_resources(card);
	}

	if (ret == -EPROBE_DEFER) {
		list_add(&card->list, &unbind_card_list);
		ret = 0;
	}
	snd_soc_card_mutex_unlock(card);

	return ret;
}

void snd_soc_card_rebind(void)
{
	struct snd_soc_card *card, *c;

	list_for_each_entry_safe(card, c, &unbind_card_list, list)
		snd_soc_card_bind_call(card);
}

static void devm_snd_soc_card_bind_release(struct device *dev, void *res)
{
	snd_soc_unregister_card(*(struct snd_soc_card **)res);
}

static int devm_snd_soc_card_bind(struct device *dev, struct snd_soc_card *card)
{
	struct snd_soc_card **ptr;
	int ret;

	/* The procedure may be called many times during the lifetime of the card. */
	devres_destroy(dev, devm_snd_soc_card_bind_release, NULL, NULL);

	ptr = devres_alloc(devm_snd_soc_card_bind_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	ret = snd_soc_card_bind(card);
	if (ret == 0) {
		*ptr = card;
		devres_add(dev, ptr);
	} else {
		devres_free(ptr);
	}

	return ret;
}

int snd_soc_card_bind_call(struct snd_soc_card *card)
{
	if (card->devres_dev)
		return devm_snd_soc_card_bind(card->devres_dev, card);
	return snd_soc_card_bind(card);
}
