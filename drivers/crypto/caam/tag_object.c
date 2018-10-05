// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Copyright 2018 NXP
 */

#include <linux/export.h>
#include <linux/string.h>
#include <linux/errno.h>

#include "tag_object.h"
#include "desc.h"

/*
 * Magic number to clearly identify the structure is for us
 * 0x54 = 'T'
 * 0x61 = 'a'
 * 0x67 = 'g'
 * 0x4f = 'O'
 */
#define TAG_OBJECT_MAGIC 0x5461674f

/**
 * struct tagged_object - Structure representing a tagged object
 * @tag : The configuration of the data
 * @object : The object
 */
struct tagged_object {
	struct tag_object_conf tag;
	char object;
};

/**
 * is_bk_type() - Determines if black key type.
 * @type: The type
 *
 * Return: True if black key type, False otherwise.
 */
static bool is_bk_type(enum tag_type type)
{
	return (type == TAG_TYPE_BLACK_KEY_ECB) ||
		(type == TAG_TYPE_BLACK_KEY_ECB_TRUSTED) ||
		(type == TAG_TYPE_BLACK_KEY_CCM) ||
		(type == TAG_TYPE_BLACK_KEY_CCM_TRUSTED);
}

/**
 * is_bk_conf() - Determines if black key conf.
 * @tag_obj_conf : The tag object conf
 *
 * Return: True if black key conf, False otherwise.
 */
bool is_bk_conf(const struct tag_object_conf *tag_obj_conf)
{
	return is_bk_type(tag_obj_conf->header.type);
}
EXPORT_SYMBOL(is_bk_conf);

/**
 * get_bk_conf() - Gets the block conf.
 * @tag_obj_conf : The tag object conf
 *
 * Return: The block conf.
 */
const struct blackey_conf *get_bk_conf(const struct tag_object_conf *tag_obj_conf)
{
	return &tag_obj_conf->conf.bk_conf;
}

/**
 * get_tag_object_overhead() - Gets the tag object overhead.
 *
 * Return: The tag object overhead.
 */
size_t get_tag_object_overhead(void)
{
	return TAG_OVERHEAD;
}
EXPORT_SYMBOL(get_tag_object_overhead);

/**
 * is_valid_type() - Determines if valid type.
 * @type : The type
 *
 * Return: True if valid type, False otherwise.
 */
bool is_valid_type(enum tag_type type)
{
	return (type > TAG_TYPE_NOT_SUPPORTED) && (type < NB_TAG_TYPE);
}
EXPORT_SYMBOL(is_valid_type);

/**
 * is_valid_header() - Determines if valid header.
 * @header : The header
 *
 * Return: True if valid tag object conf, False otherwise.
 */
static bool is_valid_header(const struct conf_header *header)
{
	bool valid = header->_magic_number == TAG_OBJECT_MAGIC;

	valid = valid && is_valid_type(header->type);

	return valid;
}

/**
 * is_valid_tag_object_conf() - Determines if valid tag object conf.
 * @tag_obj_conf : The tag object conf
 *
 * Return: True if valid header, False otherwise.
 */
bool is_valid_tag_object_conf(const struct tag_object_conf *tag_obj_conf)
{
	bool valid = true;

	valid = is_valid_header(&tag_obj_conf->header);

	return valid;
}
EXPORT_SYMBOL(is_valid_tag_object_conf);

/**
 * get_tag_object_conf() - Gets a pointer on the tag object conf.
 * @tag_obj_conf : The tag object conf
 * @buffer : The buffer
 * @size : The size
 *
 * Return: 0 if success, else error code
 */
int get_tag_object_conf(void *buffer, size_t size,
			struct tag_object_conf **tag_obj_conf)
{
	bool is_valid;
	struct tagged_object *tago = (struct tagged_object *)buffer;
	size_t conf_size = get_tag_object_overhead();

	/* Check we can retrieve the conf */
	if (size < conf_size)
		return -EINVAL;

	is_valid = is_valid_tag_object_conf(&tago->tag);

	*tag_obj_conf = &tago->tag;

	return (is_valid) ? 0 : -EINVAL;
}
EXPORT_SYMBOL(get_tag_object_conf);

/**
 * init_tag_object_header() - Initialize the tag object header
 * @conf_header : The configuration header
 * @type : The type
 *
 * It initialize the header structure
 */
void init_tag_object_header(struct conf_header *conf_header,
			    enum tag_type type)
{
	conf_header->_magic_number = TAG_OBJECT_MAGIC;
	conf_header->type = type;
}
EXPORT_SYMBOL(init_tag_object_header);

/**
 * set_tag_object_conf() - Sets the tag object conf.
 * @tag_obj_conf : The tag object conf
 * @buffer : The buffer
 * @obj_size : The object size
 * @to_size : The tagged object size
 *
 * Return: 0 if success, else error code
 */
int set_tag_object_conf(const struct tag_object_conf *tag_obj_conf,
			void *buffer, size_t obj_size, u32 *to_size)
{
	struct tagged_object *tago = buffer;
	size_t conf_size = get_tag_object_overhead();
	size_t req_size = obj_size + conf_size;

	/* Check we can set the conf */
	if (*to_size < req_size) {
		*to_size = req_size;
		return -EINVAL;
	}

	/* Move the object */
	memmove(&tago->object, buffer, obj_size);

	/* Copy the tag */
	memcpy(&tago->tag, tag_obj_conf, conf_size);

	*to_size = req_size;

	return 0;
}
EXPORT_SYMBOL(set_tag_object_conf);

/**
 * init_blackey_conf() - Initialize the black key configuration
 * @blackey_conf : The blackey conf
 * @len : The length
 * @ccm : The ccm
 * @tk : The trusted key
 *
 * It initialize the black key configuration structure
 */
void init_blackey_conf(struct blackey_conf *blackey_conf,
		       size_t len, bool ccm, bool tk)
{
	blackey_conf->real_len = len;
	blackey_conf->load = KEY_ENC
				| ((ccm) ? KEY_EKT : 0)
				| ((tk) ? KEY_TK : 0);
}
EXPORT_SYMBOL(init_blackey_conf);

/**
 * get_blackey_conf() - Get the black key configuration
 * @blackey_conf : The blackey conf
 * @real_len : The real length
 * @load_param : The load parameter
 *
 * It retrieve the black key configuration
 */
void get_blackey_conf(const struct blackey_conf *blackey_conf,
		      u32 *real_len, u32 *load_param)
{
	*real_len = blackey_conf->real_len;
	*load_param = blackey_conf->load;
}
EXPORT_SYMBOL(get_blackey_conf);

/**
 * get_tagged_data() - Get a pointer on the data and the size
 * @tagged_object : Pointer on the tagged object
 * @tagged_object_size : tagged object size in bytes
 * @data : Pointer on the data
 * @data_size : data size in bytes
 *
 * Return: 0 if success, else error code
 */
int get_tagged_data(void *tagged_object, size_t tagged_object_size,
		    void **data, u32 *data_size)
{
	struct tagged_object *tago =
		(struct tagged_object *)tagged_object;
	size_t conf_size = get_tag_object_overhead();

	/* Check we can retrieve the object */
	if (tagged_object_size < conf_size)
		return -EINVAL;

	/* Retrieve the object */
	*data = &tago->object;
	*data_size = tagged_object_size - conf_size;

	return 0;
}
EXPORT_SYMBOL(get_tagged_data);
