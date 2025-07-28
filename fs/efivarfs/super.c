// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2012 Red Hat, Inc.
 * Copyright (C) 2012 Jeremy Kerr <jeremy.kerr@canonical.com>
 */

#include <linux/ctype.h>
#include <linux/efi.h>
#include <linux/fs.h>
#include <linux/fs_context.h>
#include <linux/fs_parser.h>
#include <linux/module.h>
#include <linux/pagemap.h>
#include <linux/ucs2_string.h>
#include <linux/slab.h>
#include <linux/magic.h>
#include <linux/statfs.h>
#include <linux/notifier.h>
#include <linux/printk.h>

#include "internal.h"

static int efivarfs_ops_notifier(struct notifier_block *nb, unsigned long event,
				 void *data)
{
	struct efivarfs_fs_info *sfi = container_of(nb, struct efivarfs_fs_info, nb);

	switch (event) {
	case EFIVAR_OPS_RDONLY:
		sfi->sb->s_flags |= SB_RDONLY;
		break;
	case EFIVAR_OPS_RDWR:
		sfi->sb->s_flags &= ~SB_RDONLY;
		break;
	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static void efivarfs_evict_inode(struct inode *inode)
{
	clear_inode(inode);
}

static int efivarfs_show_options(struct seq_file *m, struct dentry *root)
{
	struct super_block *sb = root->d_sb;
	struct efivarfs_fs_info *sbi = sb->s_fs_info;
	struct efivarfs_mount_opts *opts = &sbi->mount_opts;

	if (!uid_eq(opts->uid, GLOBAL_ROOT_UID))
		seq_printf(m, ",uid=%u",
				from_kuid_munged(&init_user_ns, opts->uid));
	if (!gid_eq(opts->gid, GLOBAL_ROOT_GID))
		seq_printf(m, ",gid=%u",
				from_kgid_munged(&init_user_ns, opts->gid));
	return 0;
}

static int efivarfs_statfs(struct dentry *dentry, struct kstatfs *buf)
{
	const u32 attr = EFI_VARIABLE_NON_VOLATILE |
			 EFI_VARIABLE_BOOTSERVICE_ACCESS |
			 EFI_VARIABLE_RUNTIME_ACCESS;
	u64 storage_space, remaining_space, max_variable_size;
	u64 id = huge_encode_dev(dentry->d_sb->s_dev);
	efi_status_t status;

	/* Some UEFI firmware does not implement QueryVariableInfo() */
	storage_space = remaining_space = 0;
	if (efi_rt_services_supported(EFI_RT_SUPPORTED_QUERY_VARIABLE_INFO)) {
		status = efivar_query_variable_info(attr, &storage_space,
						    &remaining_space,
						    &max_variable_size);
		if (status != EFI_SUCCESS && status != EFI_UNSUPPORTED)
			pr_warn_ratelimited("query_variable_info() failed: 0x%lx\n",
					    status);
	}

	/*
	 * This is not a normal filesystem, so no point in pretending it has a block
	 * size; we declare f_bsize to 1, so that we can then report the exact value
	 * sent by EFI QueryVariableInfo in f_blocks and f_bfree
	 */
	buf->f_bsize	= 1;
	buf->f_namelen	= NAME_MAX;
	buf->f_blocks	= storage_space;
	buf->f_bfree	= remaining_space;
	buf->f_type	= dentry->d_sb->s_magic;
	buf->f_fsid	= u64_to_fsid(id);

	/*
	 * In f_bavail we declare the free space that the kernel will allow writing
	 * when the storage_paranoia x86 quirk is active. To use more, users
	 * should boot the kernel with efi_no_storage_paranoia.
	 */
	if (remaining_space > efivar_reserved_space())
		buf->f_bavail = remaining_space - efivar_reserved_space();
	else
		buf->f_bavail = 0;

	return 0;
}
static const struct super_operations efivarfs_ops = {
	.statfs = efivarfs_statfs,
	.drop_inode = generic_delete_inode,
	.evict_inode = efivarfs_evict_inode,
	.show_options = efivarfs_show_options,
};

/*
 * Compare two efivarfs file names.
 *
 * An efivarfs filename is composed of two parts,
 *
 *	1. A case-sensitive variable name
 *	2. A case-insensitive GUID
 *
 * So we need to perform a case-sensitive match on part 1 and a
 * case-insensitive match on part 2.
 */
static int efivarfs_d_compare(const struct dentry *dentry,
			      unsigned int len, const char *str,
			      const struct qstr *name)
{
	int guid = len - EFI_VARIABLE_GUID_LEN;

	if (name->len != len)
		return 1;

	/* Case-sensitive compare for the variable name */
	if (memcmp(str, name->name, guid))
		return 1;

	/* Case-insensitive compare for the GUID */
	return strncasecmp(name->name + guid, str + guid, EFI_VARIABLE_GUID_LEN);
}

static int efivarfs_d_hash(const struct dentry *dentry, struct qstr *qstr)
{
	unsigned long hash = init_name_hash(dentry);
	const unsigned char *s = qstr->name;
	unsigned int len = qstr->len;

	if (!efivarfs_valid_name(s, len))
		return -EINVAL;

	while (len-- > EFI_VARIABLE_GUID_LEN)
		hash = partial_name_hash(*s++, hash);

	/* GUID is case-insensitive. */
	while (len--)
		hash = partial_name_hash(tolower(*s++), hash);

	qstr->hash = end_name_hash(hash);
	return 0;
}

static const struct dentry_operations efivarfs_d_ops = {
	.d_compare = efivarfs_d_compare,
	.d_hash = efivarfs_d_hash,
	.d_delete = always_delete_dentry,
};

static struct dentry *efivarfs_alloc_dentry(struct dentry *parent, char *name)
{
	struct dentry *d;
	struct qstr q;
	int err;

	q.name = name;
	q.len = strlen(name);

	err = efivarfs_d_hash(parent, &q);
	if (err)
		return ERR_PTR(err);

	d = d_alloc(parent, &q);
	if (d)
		return d;

	return ERR_PTR(-ENOMEM);
}

static int efivarfs_callback(efi_char16_t *name16, efi_guid_t vendor,
			     unsigned long name_size, void *data,
			     struct list_head *list)
{
	struct super_block *sb = (struct super_block *)data;
	struct efivar_entry *entry;
	struct inode *inode = NULL;
	struct dentry *dentry, *root = sb->s_root;
	unsigned long size = 0;
	char *name;
	int len;
	int err = -ENOMEM;
	bool is_removable = false;

	if (guid_equal(&vendor, &LINUX_EFI_RANDOM_SEED_TABLE_GUID))
		return 0;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return err;

	memcpy(entry->var.VariableName, name16, name_size);
	memcpy(&(entry->var.VendorGuid), &vendor, sizeof(efi_guid_t));

	len = ucs2_utf8size(entry->var.VariableName);

	/* name, plus '-', plus GUID, plus NUL*/
	name = kmalloc(len + 1 + EFI_VARIABLE_GUID_LEN + 1, GFP_KERNEL);
	if (!name)
		goto fail;

	ucs2_as_utf8(name, entry->var.VariableName, len);

	if (efivar_variable_is_removable(entry->var.VendorGuid, name, len))
		is_removable = true;

	name[len] = '-';

	efi_guid_to_str(&entry->var.VendorGuid, name + len + 1);

	name[len + EFI_VARIABLE_GUID_LEN+1] = '\0';

	/* replace invalid slashes like kobject_set_name_vargs does for /sys/firmware/efi/vars. */
	strreplace(name, '/', '!');

	inode = efivarfs_get_inode(sb, d_inode(root), S_IFREG | 0644, 0,
				   is_removable);
	if (!inode)
		goto fail_name;

	dentry = efivarfs_alloc_dentry(root, name);
	if (IS_ERR(dentry)) {
		err = PTR_ERR(dentry);
		goto fail_inode;
	}

	__efivar_entry_get(entry, NULL, &size, NULL);
	__efivar_entry_add(entry, list);

	/* copied by the above to local storage in the dentry. */
	kfree(name);

	inode_lock(inode);
	inode->i_private = entry;
	i_size_write(inode, size + sizeof(entry->var.Attributes));
	inode_unlock(inode);
	d_add(dentry, inode);

	return 0;

fail_inode:
	iput(inode);
fail_name:
	kfree(name);
fail:
	kfree(entry);
	return err;
}

static int efivarfs_destroy(struct efivar_entry *entry, void *data)
{
	efivar_entry_remove(entry);
	kfree(entry);
	return 0;
}

enum {
	Opt_uid, Opt_gid,
};

static const struct fs_parameter_spec efivarfs_parameters[] = {
	fsparam_uid("uid", Opt_uid),
	fsparam_gid("gid", Opt_gid),
	{},
};

static int efivarfs_parse_param(struct fs_context *fc, struct fs_parameter *param)
{
	struct efivarfs_fs_info *sbi = fc->s_fs_info;
	struct efivarfs_mount_opts *opts = &sbi->mount_opts;
	struct fs_parse_result result;
	int opt;

	opt = fs_parse(fc, efivarfs_parameters, param, &result);
	if (opt < 0)
		return opt;

	switch (opt) {
	case Opt_uid:
		opts->uid = result.uid;
		break;
	case Opt_gid:
		opts->gid = result.gid;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int efivarfs_fill_super(struct super_block *sb, struct fs_context *fc)
{
	struct efivarfs_fs_info *sfi = sb->s_fs_info;
	struct inode *inode = NULL;
	struct dentry *root;
	int err;

	sb->s_maxbytes          = MAX_LFS_FILESIZE;
	sb->s_blocksize         = PAGE_SIZE;
	sb->s_blocksize_bits    = PAGE_SHIFT;
	sb->s_magic             = EFIVARFS_MAGIC;
	sb->s_op                = &efivarfs_ops;
	sb->s_d_op		= &efivarfs_d_ops;
	sb->s_time_gran         = 1;

	if (!efivar_supports_writes())
		sb->s_flags |= SB_RDONLY;

	inode = efivarfs_get_inode(sb, NULL, S_IFDIR | 0755, 0, true);
	if (!inode)
		return -ENOMEM;
	inode->i_op = &efivarfs_dir_inode_operations;

	root = d_make_root(inode);
	sb->s_root = root;
	if (!root)
		return -ENOMEM;

	sfi->sb = sb;
	sfi->nb.notifier_call = efivarfs_ops_notifier;
	err = blocking_notifier_chain_register(&efivar_ops_nh, &sfi->nb);
	if (err)
		return err;

	return efivar_init(efivarfs_callback, sb, &sfi->efivarfs_list);
}

static int efivarfs_get_tree(struct fs_context *fc)
{
	return get_tree_single(fc, efivarfs_fill_super);
}

static int efivarfs_reconfigure(struct fs_context *fc)
{
	if (!efivar_supports_writes() && !(fc->sb_flags & SB_RDONLY)) {
		pr_err("Firmware does not support SetVariableRT. Can not remount with rw\n");
		return -EINVAL;
	}

	return 0;
}

static const struct fs_context_operations efivarfs_context_ops = {
	.get_tree	= efivarfs_get_tree,
	.parse_param	= efivarfs_parse_param,
	.reconfigure	= efivarfs_reconfigure,
};

static int efivarfs_init_fs_context(struct fs_context *fc)
{
	struct efivarfs_fs_info *sfi;

	if (!efivar_is_available())
		return -EOPNOTSUPP;

	sfi = kzalloc(sizeof(*sfi), GFP_KERNEL);
	if (!sfi)
		return -ENOMEM;

	INIT_LIST_HEAD(&sfi->efivarfs_list);

	sfi->mount_opts.uid = GLOBAL_ROOT_UID;
	sfi->mount_opts.gid = GLOBAL_ROOT_GID;

	fc->s_fs_info = sfi;
	fc->ops = &efivarfs_context_ops;
	return 0;
}

static void efivarfs_kill_sb(struct super_block *sb)
{
	struct efivarfs_fs_info *sfi = sb->s_fs_info;

	blocking_notifier_chain_unregister(&efivar_ops_nh, &sfi->nb);
	kill_litter_super(sb);

	/* Remove all entries and destroy */
	efivar_entry_iter(efivarfs_destroy, &sfi->efivarfs_list, NULL);
	kfree(sfi);
}

static struct file_system_type efivarfs_type = {
	.owner   = THIS_MODULE,
	.name    = "efivarfs",
	.init_fs_context = efivarfs_init_fs_context,
	.kill_sb = efivarfs_kill_sb,
	.parameters = efivarfs_parameters,
};

static __init int efivarfs_init(void)
{
	return register_filesystem(&efivarfs_type);
}

static __exit void efivarfs_exit(void)
{
	unregister_filesystem(&efivarfs_type);
}

MODULE_AUTHOR("Matthew Garrett, Jeremy Kerr");
MODULE_DESCRIPTION("EFI Variable Filesystem");
MODULE_LICENSE("GPL");
MODULE_ALIAS_FS("efivarfs");

module_init(efivarfs_init);
module_exit(efivarfs_exit);
