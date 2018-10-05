/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/*
 * Copyright 2018-2019 NXP
 */

#ifndef _TAG_OBJECT_H_
#define _TAG_OBJECT_H_

#include <linux/types.h>

#define TAG_MIN_SIZE (2 * sizeof(struct conf_header))
#define TAG_OVERHEAD sizeof(struct tag_object_conf)

/**
 * enum tag_type - Type of data represented by the tag
 */
enum tag_type {
	/** @TAG_TYPE_NOT_SUPPORTED: The type is not supported */
	TAG_TYPE_NOT_SUPPORTED = 0,

	/* Type that passes is_tag_type_valid() */
	/** @TAG_TYPE_BLACK_KEY_ECB: Black key encrypted with ECB */
	TAG_TYPE_BLACK_KEY_ECB,
	/**
	 * @TAG_TYPE_BLACK_KEY_ECB_TRUSTED: ECB Black key created by trusted
	 * descriptor
	 */
	TAG_TYPE_BLACK_KEY_ECB_TRUSTED,
	/** @TAG_TYPE_BLACK_KEY_CCM: Black key encrypted with CCM */
	TAG_TYPE_BLACK_KEY_CCM,
	/**
	 * @TAG_TYPE_BLACK_KEY_CCM_TRUSTED: CCM Black key created by trusted
	 * descriptor
	 */
	TAG_TYPE_BLACK_KEY_CCM_TRUSTED,

	/** @NB_TAG_TYPE: Number of type of tag */
	NB_TAG_TYPE,
};

/**
 * struct conf_header - Common struture holding the type of data and the magic
 * number
 * @_magic_number : A magic number to identify the structure
 * @type : The type of data contained
 */
struct conf_header {
	u32 _magic_number;
	u32 type;
};

/**
 * struct blackey_conf - Configuration for a black key
 * @load : Load parameter for CAAM
 * @real_len : Length of the key before encryption
 */
struct blackey_conf {
	u32 load;
	u32 real_len;
};

/**
 * struct tag_object_conf - Common structure which is the tag applied to data
 * @header : Part of the data initialized with common function
 * :c:func:`init_tag_object_header`
 * @conf : Configuration data about the object tagged, initialized with
 * specific function
 */
struct tag_object_conf {
	struct conf_header header;
	union {
		struct blackey_conf bk_conf;
	} conf;
};

bool is_bk_conf(const struct tag_object_conf *tag_obj_conf);

bool is_valid_tag_object_conf(const struct tag_object_conf *tag_obj_conf);

void init_tag_object_header(struct conf_header *conf_header,
			    enum tag_type type);

int get_tag_object_conf(const void *buffer, size_t buffer_size,
			struct tag_object_conf **tag_obj_conf);

int set_tag_object_conf(const struct tag_object_conf *tag_obj_conf,
			void *buffer, size_t obj_size, u32 *to_size);

size_t get_tag_object_overhead(void);

void get_blackey_conf(const struct blackey_conf *blackey_conf,
		      u32 *real_len, u32 *load_param);

void init_blackey_conf(struct blackey_conf *blackey_conf,
		       size_t len, bool ccm, bool tk);

int get_tagged_data(const void *buffer, size_t buffer_size,
		    const void **data, u32 *data_size);

#endif /* _TAG_OBJECT_H_ */
