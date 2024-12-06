// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * Copyright 2020, 2022 NXP
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>

#include "caamkeyblob.h"
#include "intern.h"
#include <linux/caam_keygen.h>

#define DEVICE_NAME "caam-keygen"

static long caam_keygen_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg);

/**
 * tag_black_obj      - Tag a black object (key/blob) with a tag object header.
 *
 * @info              : keyblob_info structure, which contains
 *                      the black key/blob, obtained from CAAM,
 *                      that needs to be tagged
 * @black_max_len     : The maximum size of the black object (blob/key)
 * @blob              : Used to determine if it's a blob or key object
 *
 * Return             : '0' on success, error code otherwise
 */
static int tag_black_obj(struct keyblob_info *info, size_t black_max_len,
			 bool blob)
{
	struct header_conf tag;
	u32 type;
	int ret;
	u32 size_tagged = black_max_len;

	if (!info)
		return -EINVAL;

	type = info->type;

	/* Prepare and set the tag */
	if (blob) {
		init_tag_object_header(&tag, 0, type, info->key_len,
				       info->blob_len);
		ret = set_tag_object_header_conf(&tag, info->blob,
						 info->blob_len,
						 &size_tagged);
	} else {
		init_tag_object_header(&tag, 0, type, info->key_len,
				       info->black_key_len);
		ret = set_tag_object_header_conf(&tag, info->black_key,
						 info->black_key_len,
						 &size_tagged);
	}
	if (ret)
		return ret;

	/* Update the size of the black key tagged */
	if (blob)
		info->blob_len = size_tagged;
	else
		info->black_key_len = size_tagged;

	return ret;
}

/**
 * send_err_msg      - Send the error message from kernel to user-space
 *
 * @msg              : The message to be sent
 * @output           : The output buffer where we want to copy the error msg
 * @size             : The size of output buffer
 */
static void send_err_msg(char *msg, void __user *output, size_t size)
{
	size_t min_s;
	char null_ch = 0;

	/* Not enough space to copy any message */
	if (size <= 1)
		return;

	min_s = min(size - 1, strlen(msg));
	/*
	 * Avoid compile and checkpatch warnings, since we don't
	 * care about return value from copy_to_user
	 */
	(void)(copy_to_user(output, msg, min_s) + 1);
	/* Copy null terminator */
	(void)(copy_to_user((output + min_s), &null_ch, 1) + 1);
}

/**
 * validate_key_size - Validate the key size from user.
 *                     This can be the exact size given by user when
 *                     generating a black key from random (with -s),
 *                     or the size of the plaintext (with -t).
 *
 * @key_len          : The size of key we want to validate
 * @output           : The output buffer where we want to copy the error msg
 * @size             : The size of output buffer
 *
 *Return             : '0' on success, error code otherwise
 */
static int validate_key_size(size_t key_len, void __user *output, size_t size)
{
	char *msg = NULL;

	if (key_len < MIN_KEY_SIZE || key_len > MAX_KEY_SIZE) {
		msg = "Invalid key size, expected values are between 16 and 64 bytes.\n";
		send_err_msg(msg, output, size);
		return -EINVAL;
	}

	return 0;
}

/**
 * validate_input    - Validate the input from user and set the
 *                     keyblob_info structure.
 *                     This contains the input key in case of black key
 *                     generated from plaintext or size for random
 *                     black key.
 *
 * @key_crt          : Structure with data from user
 * @arg              : User-space argument from ioctl call
 * @info             : keyblob_info structure, will be updated with all the
 *                     data from user-space
 * @create_key_op    : Used to determine if it's a create or import operation
 *
 * Return            : '0' on success, error code otherwise
 */
static int validate_input(struct caam_keygen_cmd *key_crt, unsigned long arg,
			  struct keyblob_info *info, bool create_key_op)
{
	char *tmp, *msg;
	size_t tmp_size;
	bool random = false;
	int ret = 0;
	u32 tmp_len = 0;
	char null_ch = 0;

	/*
	 * So far, we only support Black keys, encrypted with JDKEK,
	 * kept in general memory, non-secure state.
	 * Therefore, default value for type is 1.
	 */
	u32 type = 1;

	if (copy_from_user(key_crt, (void __user *)arg,
			   sizeof(struct caam_keygen_cmd)))
		return -EFAULT;

	/* Get blob_len from user. */
	info->blob_len = key_crt->blob_len;
	/* Get black_key_len from user. */
	info->black_key_len = key_crt->black_key_len;

	/*
	 * Based on operation type validate a different set of input data.
	 *
	 * For key creation, validate the Encrypted Key Type,
	 * the Key Mode and Key Value
	 */
	if (create_key_op) {
		/*
		 * Validate arguments received from user.
		 * These must be at least 1 since
		 * they have null terminator.
		 */
		if (key_crt->key_enc_len < 1 || key_crt->key_mode_len < 1 ||
		    key_crt->key_value_len < 1) {
			msg = "Invalid arguments.\n";
			send_err_msg(msg, u64_to_user_ptr(key_crt->blob),
				     key_crt->blob_len);
			return -EFAULT;
		}
		/*
		 * Allocate memory for temporary buffer used to
		 * get the user arguments from user-space
		 */
		tmp_size = max_t(size_t, key_crt->key_enc_len,
				 max_t(size_t, key_crt->key_mode_len,
				       key_crt->key_value_len)) + 1;
		tmp = kmalloc(tmp_size, GFP_KERNEL);
		if (!tmp) {
			msg = "Unable to allocate memory for temporary buffer.\n";
			send_err_msg(msg, u64_to_user_ptr(key_crt->blob),
				     key_crt->blob_len);
			return -ENOMEM;
		}
		/* Add null terminator */
		tmp[tmp_size - 1] = null_ch;
		/*
		 * Validate and set, in type, the Encrypted Key Type
		 * given from user-space.
		 * This must be ecb or ccm.
		 */
		if (copy_from_user(tmp, u64_to_user_ptr(key_crt->key_enc),
				   key_crt->key_enc_len)) {
			msg = "Unable to copy from user the Encrypted Key Type.\n";
			send_err_msg(msg, u64_to_user_ptr(key_crt->blob),
				     key_crt->blob_len);
			ret = -EFAULT;
			goto free_resource;
		}
		if (!strcmp(tmp, "ccm")) {
			type |= BIT(TAG_OBJ_EKT_OFFSET);
		} else if (strcmp(tmp, "ecb")) {
			msg = "Invalid argument for Encrypted Key Type, expected ecb or ccm.\n";
			send_err_msg(msg, u64_to_user_ptr(key_crt->blob),
				     key_crt->blob_len);
			ret = -EINVAL;
			goto free_resource;
		}
		/*
		 * Validate the Key Mode given from user-space.
		 * This must be -t (text), for a black key generated
		 * from a plaintext, or -s (size) for a black key
		 * generated from random.
		 */
		if (copy_from_user(tmp, u64_to_user_ptr(key_crt->key_mode),
				   key_crt->key_mode_len)) {
			msg = "Unable to copy from user the Key Mode: random (-s) or plaintext (-t).\n";
			send_err_msg(msg, u64_to_user_ptr(key_crt->blob),
				     key_crt->blob_len);
			ret = -EFAULT;
			goto free_resource;
		}
		if (!strcmp(tmp, "-s")) {
			random = true; /* black key generated from random */
		} else if (strcmp(tmp, "-t")) {
			msg = "Invalid argument for Key Mode, expected -s or -t.\n";
			send_err_msg(msg, u64_to_user_ptr(key_crt->blob),
				     key_crt->blob_len);
			ret = -EINVAL;
			goto free_resource;
		}
		/*
		 * Validate and set, into keyblob_info structure,
		 * the plaintext or key size, based on Key Mode.
		 */
		if (copy_from_user(tmp, u64_to_user_ptr(key_crt->key_value),
				   key_crt->key_value_len)) {
			msg = "Unable to copy from user the Key Value: size or plaintext.\n";
			send_err_msg(msg, u64_to_user_ptr(key_crt->blob),
				     key_crt->blob_len);
			ret = -EFAULT;
			goto free_resource;
		}
		/* Black key generated from random, get its size */
		if (random) {
			info->key = NULL;
			ret = kstrtou32(tmp, 10, &tmp_len);
			if (ret != 0) {
				msg = "Invalid key size.\n";
				send_err_msg(msg, u64_to_user_ptr(key_crt->blob),
					     key_crt->blob_len);
				goto free_resource;
			}
			ret = validate_key_size(tmp_len,
						u64_to_user_ptr(key_crt->blob),
						key_crt->blob_len);
			if (ret)
				goto free_resource;

			info->key_len = tmp_len;
		} else {
			/*
			 * Black key generated from plaintext,
			 * get the plaintext (input key) and its size
			 */
			ret = validate_key_size(key_crt->key_value_len,
						u64_to_user_ptr(key_crt->blob),
						key_crt->blob_len);
			if (ret)
				goto free_resource;

			info->key = tmp;
			info->key_len = key_crt->key_value_len;
		}
		info->type = type;
	} else {
		/* For key import, get the blob from user-space */
		if (copy_from_user(info->blob, u64_to_user_ptr(key_crt->blob),
				   info->blob_len)) {
			msg = "Unable to copy from user the blob.\n";
			send_err_msg(msg, u64_to_user_ptr(key_crt->black_key),
				     key_crt->black_key_len);
			return -EFAULT;
		}
	}

	goto exit;

free_resource:
	kfree(tmp);

exit:
	return ret;
}

/**
 * keygen_create_keyblob - Generate key and blob
 *
 * @info             : keyblob_info structure, will be updated with
 *                     the black key and blob data from CAAM
 *
 * Return            : '0' on success, error code otherwise
 */
static int keygen_create_keyblob(struct keyblob_info *info)
{
	int ret = 0;
	struct device *jrdev;

	/* Allocate caam job ring for operation to be performed from CAAM */
	jrdev = caam_jr_alloc();
	if (IS_ERR(jrdev)) {
		pr_err("Job Ring Device allocation failed\n");
		return PTR_ERR(jrdev);
	}

	/* Create a black key */
	ret = generate_black_key(jrdev, info);
	if (ret) {
		dev_err(jrdev, "Black key generation failed: (%d)\n", ret);
		goto free_jr;
	}

	/* Clear the input key, if exists */
	if (info->key)
		memset(info->key, 0, info->key_len);

	/* Set key modifier, used as revision number, for blob */
	info->key_mod = caam_key_modifier;
	info->key_mod_len = ARRAY_SIZE(caam_key_modifier);

	/*
	 * Encapsulate the key, into a black blob, in general memory
	 * (the only memory type supported, right now)
	 */
	ret = caam_blob_encap(jrdev, info);
	if (ret) {
		dev_err(jrdev, "Blob encapsulation of black key failed: %d\n",
			ret);
		goto free_jr;
	}

	/* Tag the black key so it can be passed to CAAM Crypto API */
	ret = tag_black_obj(info, sizeof(info->black_key), false);
	if (ret) {
		dev_err(jrdev, "Black key tagging failed: %d\n", ret);
		goto free_jr;
	}

	/* Tag the black blob so it can be passed to CAAM Crypto API */
	ret = tag_black_obj(info, sizeof(info->blob), true);
	if (ret) {
		dev_err(jrdev, "Black blob tagging failed: %d\n", ret);
		goto free_jr;
	}

free_jr:
	caam_jr_free(jrdev);

	return ret;
}

/**
 * keygen_import_key - Import a black key from a blob
 *
 * @info             : keyblob_info structure, will be updated with
 *                     the black key obtained after blob decapsulation by CAAM
 *
 * Return            : '0' on success, error code otherwise
 */
static int keygen_import_key(struct keyblob_info *info)
{
	int ret = 0;
	struct device *jrdev;
	struct header_conf *header;
	struct tagged_object *tag_obj;

	/* Allocate CAAM Job Ring for operation to be performed from CAAM */
	jrdev = caam_jr_alloc();
	if (IS_ERR(jrdev)) {
		pr_err("Job Ring Device allocation failed\n");
		return PTR_ERR(jrdev);
	}

	/* Set key modifier, used as revision number, for blob */
	info->key_mod = caam_key_modifier;
	info->key_mod_len = ARRAY_SIZE(caam_key_modifier);

	print_hex_dump_debug("input blob @ " __stringify(__LINE__) " : ",
			     DUMP_PREFIX_ADDRESS, 16, 4, info->blob,
			     info->blob_len, 1);

	/* Check if one can retrieve the tag object header configuration */
	if (info->blob_len <= TAG_OVERHEAD_SIZE) {
		dev_err(jrdev, "Invalid blob length\n");
		ret = -EINVAL;
		goto free_jr;
	}

	/* Retrieve the tag object */
	tag_obj = (struct tagged_object *)info->blob;

	/*
	 * Check tag object header configuration
	 * and retrieve the tag object header configuration
	 */
	if (is_valid_header_conf(&tag_obj->header)) {
		header = &tag_obj->header;
	} else {
		dev_err(jrdev,
			"Unable to get tag object header configuration for blob\n");
		ret = -EINVAL;
		goto free_jr;
	}

	info->key_len = header->red_key_len;

	/* Validate the red key size extracted from blob */
	if (info->key_len < MIN_KEY_SIZE || info->key_len > MAX_KEY_SIZE) {
		dev_err(jrdev,
			"Invalid red key length extracted from blob, expected values are between 16 and 64 bytes\n");
		ret = -EINVAL;
		goto free_jr;
	}

	info->type = header->type;

	/* Update blob length by removing the header size */
	info->blob_len -= TAG_OVERHEAD_SIZE;

	/*
	 * Check the received, from user, blob length
	 * with the one from tag header
	 */
	if (info->blob_len != header->obj_len) {
		dev_err(jrdev, "Mismatch between received blob length and the one from tag header\n");
		ret = -EINVAL;
		goto free_jr;
	}

	/*
	 * Decapsulate the blob into a black key,
	 * in general memory (the only memory type supported, right now)
	 */
	ret = caam_blob_decap(jrdev, info);
	if (ret) {
		dev_err(jrdev, "Blob decapsulation failed: %d\n", ret);
		goto free_jr;
	}

	/* Tag the black key so it can be passed to CAAM Crypto API */
	ret = tag_black_obj(info, sizeof(info->black_key), false);
	if (ret)
		dev_err(jrdev, "Black key tagging failed: %d\n", ret);

free_jr:
	caam_jr_free(jrdev);

	return ret;
}

/**
 * send_output       - Send the output data (tagged key and blob)
 *                     from kernel to user-space.
 *
 * @key_crt          : Structure used to transfer data
 *                     from user-space to kernel
 * @info             : keyblob_info structure, which contains all
 *                     the data obtained from CAAM that needs to
 *                     be transferred to user-space
 * @create_key_op    : Used to determine if it's a create or import operation
 * @err              : Error code received from previous operations
 *
 * Return            : '0' on success, error code otherwise
 */
static int send_output(struct caam_keygen_cmd *key_crt, unsigned long arg,
		       struct keyblob_info *info, bool create_key_op, int err)
{
	int ret = 0;
	char *msg;

	/* Free resource used on validate_input */
	kfree(info->key);

	if (err)
		return err;

	/* Check if there's enough space to copy black key to user */
	if (key_crt->black_key_len < info->black_key_len) {
		msg = "Not enough space for black key.\n";
		send_err_msg(msg, u64_to_user_ptr(key_crt->blob),
			     key_crt->blob_len);
		/* Send, to user, the necessary size for key */
		key_crt->black_key_len = info->black_key_len;

		ret = -EINVAL;
		goto exit;
	}
	key_crt->black_key_len = info->black_key_len;

	/* For key import, copy to user only the black key */
	if (copy_to_user(u64_to_user_ptr(key_crt->black_key),
			 info->black_key, info->black_key_len))
		return -EFAULT;

	/* For key creation, copy to user, also, the blob */
	if (create_key_op) {
		/* Check if there's enough space to copy blob user */
		if (key_crt->blob_len < info->blob_len) {
			msg = "Not enough space for blob key.\n";
			send_err_msg(msg, u64_to_user_ptr(key_crt->blob),
				     key_crt->blob_len);
			/* Send, to user, the necessary size for blob */
			key_crt->blob_len = info->blob_len;

			ret = -EINVAL;
			goto exit;
		}

		key_crt->blob_len = info->blob_len;

		if (copy_to_user(u64_to_user_ptr(key_crt->blob), info->blob,
				 info->blob_len))
			return -EFAULT;
	}

exit:
	if (copy_to_user((void __user *)arg, key_crt,
			 sizeof(struct caam_keygen_cmd)))
		return -EFAULT;

	return ret;
}

static long caam_keygen_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	int ret = 0;
	struct keyblob_info info = {.key = NULL};
	struct caam_keygen_cmd key_crt;
	/* Used to determine if it's a create or import operation */
	bool create_key_op = false;

	switch (cmd) {
	case CAAM_KEYGEN_IOCTL_CREATE:
	{
		create_key_op = true;

		/* Validate user-space input */
		ret = validate_input(&key_crt, arg, &info, create_key_op);
		if (ret)
			break;

		/* Create tagged key and blob */
		ret = keygen_create_keyblob(&info);

		/* Send data from kernel to user-space */
		ret = send_output(&key_crt, arg, &info, create_key_op, ret);

		break;
	}
	case CAAM_KEYGEN_IOCTL_IMPORT:
	{
		/* Validate user-space input */
		ret = validate_input(&key_crt, arg, &info, create_key_op);
		if (ret)
			break;

		/* Import tagged key from blob */
		ret = keygen_import_key(&info);

		/* Send data from kernel to user-space */
		ret = send_output(&key_crt, arg, &info, create_key_op, ret);

		break;
	}
	default:
		ret = -ENOTTY;
	}

	return ret;
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = caam_keygen_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static struct miscdevice caam_keygen_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &fops
};

int caam_keygen_init(void)
{
	int ret;

	ret = misc_register(&caam_keygen_dev);
	if (ret) {
		pr_err("Failed to register device %s\n",
		       caam_keygen_dev.name);
		return ret;
	}

	pr_info("Device %s registered\n", caam_keygen_dev.name);

	return 0;
}

void caam_keygen_exit(void)
{
	misc_deregister(&caam_keygen_dev);

	pr_info("caam_keygen unregistered\n");
}
