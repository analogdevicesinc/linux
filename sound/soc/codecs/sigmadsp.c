/*
 * Load Analog Devices SigmaStudio firmware files
 *
 * Copyright 2009-2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <sound/control.h>
#include <sound/soc.h>

#include <asm/unaligned.h>

#include "sigmadsp.h"

#define SIGMA_MAGIC "ADISIGM"

#define SIGMA_FW_CHUNK_TYPE_DATA 0
#define SIGMA_FW_CHUNK_TYPE_CONTROL 1
#define SIGMA_FW_CHUNK_TYPE_SAMPLERATES 2

struct sigmadsp_control {
	struct list_head head;
	unsigned int samplerates;
	unsigned int addr;
	unsigned int num_bytes;
	const char *name;
	struct snd_kcontrol *kcontrol;
	bool cached;
	uint8_t cache[];
};

struct sigmadsp_data {
	struct list_head head;
	unsigned int samplerates;
	unsigned int addr;
	unsigned int length;
	uint8_t data[];
};

struct sigma_fw_chunk {
	__le32 length;
	__le32 tag;
	__le32 samplerates;
} __packed;

struct sigma_fw_chunk_data {
	struct sigma_fw_chunk chunk;
	__be16 addr;
	uint8_t data[];
} __packed;

struct sigma_fw_chunk_control {
	struct sigma_fw_chunk chunk;
	__le16 type;
	__le16 addr;
	__le16 num_bytes;
	const char name[];
} __packed;

struct sigma_fw_chunk_samplerate {
	struct sigma_fw_chunk chunk;
	__le32 samplerates[];
} __packed;

struct sigma_firmware_header {
	unsigned char magic[7];
	u8 version;
	__le32 crc;
} __packed;

enum {
	SIGMA_ACTION_WRITEXBYTES = 0,
	SIGMA_ACTION_WRITESINGLE,
	SIGMA_ACTION_WRITESAFELOAD,
	SIGMA_ACTION_DELAY,
	SIGMA_ACTION_PLLWAIT,
	SIGMA_ACTION_NOOP,
	SIGMA_ACTION_END,
};

struct sigma_action {
	u8 instr;
	u8 len_hi;
	__le16 len;
	__be16 addr;
	unsigned char payload[];
} __packed;

static int sigmadsp_rate_to_index(struct sigmadsp *sigmadsp, unsigned int rate)
{
	unsigned int i;

	for (i = 0; i < sigmadsp->rate_constraints.count; i++) {
		if (sigmadsp->rate_constraints.list[i] == rate)
			return i;
	}

	return -EINVAL;
}

static int sigmadsp_write(struct sigmadsp *sigmadsp, unsigned int addr,
	const uint8_t data[], size_t len)
{
	return sigmadsp->write(sigmadsp->control_data, addr, data, len);
}

static int sigmadsp_read(struct sigmadsp *sigmadsp, unsigned int addr,
	uint8_t data[], size_t len)
{
	return sigmadsp->read(sigmadsp->control_data, addr, data, len);
}

static int sigma_fw_ctrl_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *info)
{
	struct sigmadsp_control *ctrl = (void *)kcontrol->private_value;

	info->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	info->count = ctrl->num_bytes;

	return 0;
}

static int sigma_fw_ctrl_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct sigmadsp_control *ctrl = (void *)kcontrol->private_value;
	struct sigmadsp *sigmadsp = snd_kcontrol_chip(kcontrol);
	uint8_t *data;
	int ret;

	data = ucontrol->value.bytes.data;

	if (ctrl->num_bytes <= 4 || ctrl->num_bytes > 20 || !sigmadsp->ops)
		ret = sigmadsp_write(sigmadsp, ctrl->addr, data,
			ctrl->num_bytes);
	else
		ret = sigmadsp->ops->safeload(sigmadsp, ctrl->addr, data,
			ctrl->num_bytes);

	if (ret == 0)
		memcpy(ctrl->cache, ucontrol->value.bytes.data, ctrl->num_bytes);

	return ret;
}

static int sigma_fw_ctrl_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct sigmadsp_control *ctrl = (void *)kcontrol->private_value;
	struct sigmadsp *sigmadsp = snd_kcontrol_chip(kcontrol);

	if (!ctrl->cached) {
		sigmadsp_read(sigmadsp, ctrl->addr, ctrl->cache,
			ctrl->num_bytes);
		ctrl->cached = true;
	}

	memcpy(ucontrol->value.bytes.data, ctrl->cache, ctrl->num_bytes);

	return 0;
}

static int sigma_fw_load_data(struct sigmadsp *sigmadsp,
	const struct sigma_fw_chunk *chunk, unsigned int length)
{
	const struct sigma_fw_chunk_data *data_chunk;
	struct sigmadsp_data *data;

	if (length <= sizeof(*data_chunk))
	    return -EINVAL;

	data_chunk = (struct sigma_fw_chunk_data *)chunk;
	
	length -= sizeof(*data_chunk);
	
	data = kzalloc(sizeof(*data) + length, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->addr = le16_to_cpu(data_chunk->addr);
	data->length = length;
	memcpy(data->data, data_chunk->data, length);
	list_add_tail(&data->head, &sigmadsp->data_list);

	return 0;
}

static void sigma_fw_control_free(struct snd_kcontrol *kcontrol)
{
	struct sigmadsp_control *ctrl = (void *)kcontrol->private_value;

	ctrl->kcontrol = NULL;
}

static bool sigma_fw_validate_control_name(const char *name, unsigned int len)
{
	unsigned int i;

	for (i = 0; i < len; i++) {
		/* Normal ASCII characters are valid */
		if (name[i] < ' ' || name[i] > '~')
			return false;
	}

	return true;
}

static int sigma_fw_load_control(struct sigmadsp *sigmadsp,
	const struct sigma_fw_chunk *chunk, unsigned int length)
{
	const struct sigma_fw_chunk_control *ctrl_chunk;
	struct sigmadsp_control *ctrl;
	unsigned int num_bytes;
	size_t name_len;
	char *name;
	int ret;

	if (length <= sizeof(*ctrl_chunk))
	    return -EINVAL;

	ctrl_chunk = (const struct sigma_fw_chunk_control *)chunk;

	name_len = length - sizeof(*ctrl_chunk);
	if (name_len > 43) /* Max ALSA control name length */
	    return -EINVAL;

	/* Make sure there are no %NUL bytes in the string */
	if (!sigma_fw_validate_control_name(ctrl_chunk->name, name_len))
		return -EINVAL;

	num_bytes = le16_to_cpu(ctrl_chunk->num_bytes);
	ctrl = kzalloc(sizeof(*ctrl) + num_bytes, GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	name = kzalloc(name_len + 1, GFP_KERNEL);
	if (!name) {
		ret = -ENOMEM;
		goto err_free_ctrl;
	}
	memcpy(name, ctrl_chunk->name, name_len);
	name[name_len] = '\0';
	ctrl->name = name;
	
	ctrl->addr = le16_to_cpu(ctrl_chunk->addr);
	ctrl->num_bytes = num_bytes;

	list_add_tail(&ctrl->head, &sigmadsp->ctrl_list);

	return 0;

err_free_ctrl:
	kfree(ctrl);

	return ret;
}

static int sigma_fw_load_samplerates(struct sigmadsp *sigmadsp,
	const struct sigma_fw_chunk *chunk, unsigned int length)
{
	const struct sigma_fw_chunk_samplerate *rate_chunk;
	unsigned int num_rates;
	unsigned int *rates;
	unsigned int i;

	rate_chunk = (const struct sigma_fw_chunk_samplerate *)chunk;

	num_rates = (length - sizeof(*rate_chunk)) / sizeof(__le32);

	if (num_rates > 32 || num_rates == 0)
		return -EINVAL;

	/* We only allow one samplerates block per file */
	if (sigmadsp->rate_constraints.count)
		return -EINVAL;

	rates = kcalloc(num_rates, sizeof(*rates), GFP_KERNEL);
	if (!rates)
		return -ENOMEM;

	for (i = 0; i < num_rates; i++)
		rates[i] = le32_to_cpu(rate_chunk->samplerates[i]);

	sigmadsp->rate_constraints.count = num_rates;
	sigmadsp->rate_constraints.list = rates;

	return 0;
}

static int sigmadsp_fw_load_v2(struct snd_soc_codec *codec,
	struct sigmadsp *sigmadsp, const struct firmware *fw)
{
	struct sigma_fw_chunk *chunk;
	unsigned int length, pos;
	int ret;

	/*
	 * Make sure that there is at least one chunk to avoid integer
	 * underflows later on.
	 */
	if (fw->size < sizeof(*chunk) + sizeof(struct sigma_firmware_header))
		return -EINVAL;

	pos = sizeof(struct sigma_firmware_header);

	while (pos < fw->size - sizeof(*chunk)) {
		chunk = (struct sigma_fw_chunk *)(fw->data + pos);

		length = le32_to_cpu(chunk->length);

		if (length > fw->size - pos || length < sizeof(*chunk))
			return -EINVAL;

		switch (chunk->tag) {
		case SIGMA_FW_CHUNK_TYPE_DATA:
			ret = sigma_fw_load_data(sigmadsp, chunk, length);
			break;
		case SIGMA_FW_CHUNK_TYPE_CONTROL:
			ret = sigma_fw_load_control(sigmadsp, chunk, length);
			break;
		case SIGMA_FW_CHUNK_TYPE_SAMPLERATES:
			ret = sigma_fw_load_samplerates(sigmadsp, chunk, length);
			break;
		default:
			dev_warn(codec->dev, "Unkown chunk type: %d\n",
				chunk->tag);
			ret = 0;
			break;
		}

		if (ret)
			return ret;

		/*
		 * This can not overflow since if length is larger than the
		 * maximum firmware size (0x4000000) we'll error out earilier.
		 */
		pos += ALIGN(length, sizeof(__le32));
	}

	return 0;
}

static inline u32 sigma_action_len(struct sigma_action *sa)
{
	return (sa->len_hi << 16) | le16_to_cpu(sa->len);
}

static size_t sigma_action_size(struct sigma_action *sa)
{
	size_t payload = 0;

	switch (sa->instr) {
	case SIGMA_ACTION_WRITEXBYTES:
	case SIGMA_ACTION_WRITESINGLE:
	case SIGMA_ACTION_WRITESAFELOAD:
		payload = sigma_action_len(sa);
		break;
	default:
		break;
	}

	payload = ALIGN(payload, 2);

	return payload + sizeof(struct sigma_action);
}

/*
 * Returns a negative error value in case of an error, 0 if processing of
 * the firmware should be stopped after this action, 1 otherwise.
 */
static int process_sigma_action(struct sigmadsp *sigmadsp,
	struct sigma_action *sa)
{
	size_t len = sigma_action_len(sa);
	int ret;

	pr_debug("%s: instr:%i addr:%#x len:%zu\n", __func__,
		sa->instr, sa->addr, len);

	switch (sa->instr) {
	case SIGMA_ACTION_WRITEXBYTES:
	case SIGMA_ACTION_WRITESINGLE:
	case SIGMA_ACTION_WRITESAFELOAD:
		ret = sigmadsp_write(sigmadsp, be16_to_cpu(sa->addr),
				sa->payload, len - 2);
		if (ret < 0)
			return -EINVAL;
		break;
	case SIGMA_ACTION_DELAY:
		udelay(len);
		len = 0;
		break;
	case SIGMA_ACTION_END:
		return 0;
	default:
		return -EINVAL;
	}

	return 1;
}

static int sigmadsp_fw_load_v1(struct sigmadsp *sigmadsp,
	const struct firmware *fw)
{
	struct sigma_action *sa;
	size_t size, pos;
	int ret;

	pos = sizeof(struct sigma_firmware_header);

	while (pos + sizeof(*sa) <= fw->size) {
		sa = (struct sigma_action *)(fw->data + pos);

		size = sigma_action_size(sa);
		pos += size;
		if (pos > fw->size || size == 0)
			break;

		ret = process_sigma_action(sigmadsp, sa);

		pr_debug("%s: action returned %i\n", __func__, ret);

		if (ret <= 0)
			return ret;
	}

	if (pos != fw->size)
		return -EINVAL;

	return 0;
}

void sigmadsp_firmware_release(struct sigmadsp *sigmadsp)
{
	struct sigmadsp_control *ctrl, *_ctrl;
	struct sigmadsp_data *data, *_data;

	sigmadsp_reset(sigmadsp);

	list_for_each_entry_safe(ctrl, _ctrl, &sigmadsp->ctrl_list, head) {
		kfree(ctrl->name);
		kfree(ctrl);
	}

	list_for_each_entry_safe(data, _data, &sigmadsp->data_list, head)
		kfree(data);

	INIT_LIST_HEAD(&sigmadsp->ctrl_list);
	INIT_LIST_HEAD(&sigmadsp->data_list);
}
EXPORT_SYMBOL_GPL(sigmadsp_firmware_release);

int sigmadsp_firmware_load(struct sigmadsp *sigmadsp,
	struct snd_soc_codec *codec, const char *name)
{
	const struct sigma_firmware_header *ssfw_head;
	const struct firmware *fw;
	int ret;
	u32 crc;

	/* first load the blob */
	ret = request_firmware(&fw, name, codec->dev);
	if (ret) {
		pr_debug("%s: request_firmware() failed with %i\n", __func__, ret);
		goto done;
	}

	/* then verify the header */
	ret = -EINVAL;

	/*
	 * Reject too small or unreasonable large files. The upper limit has been
	 * chosen a bit arbitrarily, but it should be enough for all practical
	 * purposes and having the limit makes it easier to avoid integer
	 * overflows later in the loading process.
	 */
	if (fw->size < sizeof(*ssfw_head) || fw->size >= 0x4000000) {
		dev_err(codec->dev, "Failed to load firmware: Invalid size\n");
		goto done;
	}

	ssfw_head = (void *)fw->data;
	if (memcmp(ssfw_head->magic, SIGMA_MAGIC, ARRAY_SIZE(ssfw_head->magic))) {
		dev_err(codec->dev, "Failed to load firmware: Invalid magic\n");
		goto done;
	}

	crc = crc32(0, fw->data + sizeof(*ssfw_head),
			fw->size - sizeof(*ssfw_head));
	pr_debug("%s: crc=%x\n", __func__, crc);
	if (crc != le32_to_cpu(ssfw_head->crc)) {
		dev_err(codec->dev, "Failed to load firmware: Wrong crc checksum: expected %x got %x\n",
			le32_to_cpu(ssfw_head->crc), crc);
		goto done;
	}

	switch (ssfw_head->version) {
	case 1:
		ret = sigmadsp_fw_load_v1(sigmadsp, fw);
		break;
	case 2:
		ret = sigmadsp_fw_load_v2(codec, sigmadsp, fw);
		break;
	default:
		dev_err(codec->dev, "Unknown firmware version: %d\n",
			ssfw_head->version);
		ret = -EINVAL;
		break;
	}

	sigmadsp->codec = codec;

done:
	release_firmware(fw);

	return ret;
}
EXPORT_SYMBOL_GPL(sigmadsp_firmware_load);

void sigmadsp_init(struct sigmadsp *sigmadsp, const struct sigmadsp_ops *ops)
{
	sigmadsp->ops = ops;

	INIT_LIST_HEAD(&sigmadsp->ctrl_list);
	INIT_LIST_HEAD(&sigmadsp->data_list);
}
EXPORT_SYMBOL_GPL(sigmadsp_init);

void sigmadsp_reset(struct sigmadsp *sigmadsp)
{
	struct sigmadsp_control *ctrl;

	list_for_each_entry(ctrl, &sigmadsp->ctrl_list, head) {
		if (ctrl->kcontrol)
			snd_ctl_remove(sigmadsp->codec->card->snd_card, ctrl->kcontrol);
	}

	sigmadsp->current_samplerate = 0;
}
EXPORT_SYMBOL_GPL(sigmadsp_reset);

static int sigmadsp_alloc_control(struct sigmadsp *sigmadsp,
	struct sigmadsp_control *ctrl)
{
	struct snd_kcontrol_new template;
	struct snd_kcontrol *kcontrol;
	int ret;

	memset(&template, 0, sizeof(template));
	template.iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	template.name = ctrl->name;
	template.info = sigma_fw_ctrl_info;
	template.get = sigma_fw_ctrl_get;
	template.put = sigma_fw_ctrl_put;
	template.private_value = (unsigned long)ctrl;

	kcontrol = snd_ctl_new1(&template, sigmadsp);
	if (!kcontrol)
		return -ENOMEM;

	kcontrol->private_free = sigma_fw_control_free;

	ret = snd_ctl_add(sigmadsp->codec->card->snd_card, kcontrol);
	if (ret)
	    return ret;

	ctrl->kcontrol = kcontrol;

	return 0;
}

int sigmadsp_setup(struct sigmadsp *sigmadsp, unsigned int samplerate)
{
	struct sigmadsp_control *ctrl;
	struct sigmadsp_data *data;
	unsigned int samplerate_bit;
	int samplerate_index;
	int ret;

	if (sigmadsp->current_samplerate == samplerate)
		return 0;

	sigmadsp_reset(sigmadsp);

	samplerate_index = sigmadsp_rate_to_index(sigmadsp, samplerate);
	if (samplerate_index < 0)
		return samplerate_index;
	
	samplerate_bit = BIT(samplerate_index);

	list_for_each_entry(data, &sigmadsp->data_list, head) {
		if (data->samplerates && !(data->samplerates & samplerate_bit))
			continue;
		ret = sigmadsp_write(sigmadsp, data->addr, data->data,
			data->length);
		if (ret)
			goto err;
	}

	list_for_each_entry(ctrl, &sigmadsp->ctrl_list, head) {
		if (ctrl->samplerates && !(ctrl->samplerates & samplerate_bit))
			continue;
		ret = sigmadsp_alloc_control(sigmadsp, ctrl);
		if (ret)
			goto err;
	}

	sigmadsp->current_samplerate = samplerate;

	return 0;

err:
	sigmadsp_reset(sigmadsp);

	return ret;
}
EXPORT_SYMBOL_GPL(sigmadsp_setup);

int sigmadsp_restrict_params(struct sigmadsp *sigmadsp,
	struct snd_pcm_substream *substream)
{
	if (sigmadsp->rate_constraints.count == 0)
		return 0;

	return snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE, &sigmadsp->rate_constraints);
}
EXPORT_SYMBOL_GPL(sigmadsp_restrict_params);

MODULE_LICENSE("GPL");
