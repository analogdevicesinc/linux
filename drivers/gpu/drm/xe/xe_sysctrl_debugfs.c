// SPDX-License-Identifier: MIT
/*
 * Copyright © 2026 Intel Corporation
 */

#include <linux/cleanup.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/kstrtox.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/string_choices.h>
#include <linux/uaccess.h>

#include "xe_pm.h"
#include "xe_printk.h"
#include "xe_sysctrl.h"
#include "xe_sysctrl_debugfs.h"
#include "xe_sysctrl_mailbox.h"
#include "xe_sysctrl_mailbox_types.h"
#include "xe_sysctrl_types.h"

static ssize_t xe_sysctrl_loopback_write(struct file *file, const char __user *ubuf,
					 size_t len, loff_t *offp)
{
	char *kbuf __free(kfree) = NULL;
	u8 *input __free(kfree) = NULL;
	struct seq_file *m = file->private_data;
	struct xe_sysctrl_debugfs_entry *entry = m->private;
	struct xe_device *xe = sc_to_xe(entry->sc);
	struct xe_sysctrl_mailbox_command cmd = {};
	char *token, *tmp;
	unsigned long val;
	size_t input_len = 0;
	size_t max_input;
	size_t out_len = 0;
	int status;

	if (*offp)
		return -ESPIPE;

	if (len == 0 || len >= PAGE_SIZE)
		return -EINVAL;

	kbuf = kmalloc(len + 1, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	max_input = min_t(size_t, len, XE_SYSCTRL_MB_MAX_DATA_SIZE);
	input = kmalloc(max_input, GFP_KERNEL);
	if (!input)
		return -ENOMEM;

	if (copy_from_user(kbuf, ubuf, len))
		return -EFAULT;
	kbuf[len] = '\0';

	tmp = kbuf;
	while ((token = strsep(&tmp, " \t\n")) != NULL) {
		if (*token == '\0')
			continue;

		if (input_len >= max_input) {
			xe_err(xe, "sysctrl: loopback payload too large (max %zu bytes)\n",
			       max_input);
			return -EINVAL;
		}

		if (kstrtoul(token, 0, &val) || val > 0xFF) {
			xe_err(xe, "sysctrl: invalid loopback token '%s'\n", token);
			return -EINVAL;
		}

		input[input_len++] = (u8)val;
	}

	if (input_len == 0) {
		xe_err(xe, "sysctrl: no loopback payload given\n");
		return -EINVAL;
	}

	xe_sysctrl_create_command(&cmd, entry->group, entry->command,
				  input, input_len, entry->response_buf, input_len);

	scoped_guard(mutex, &entry->lock) {
		guard(xe_pm_runtime)(xe);
		entry->status = xe_sysctrl_send_command(entry->sc, &cmd, &out_len);
		entry->response_len = entry->status ? 0 : out_len;
		status = entry->status;
	}

	return status ? status : len;
}

static int xe_sysctrl_loopback_show(struct seq_file *m, void *data)
{
	struct xe_sysctrl_debugfs_entry *entry = m->private;
	size_t i;

	guard(mutex)(&entry->lock);

	seq_printf(m, "Command: group=0x%02x cmd=0x%02x\n", entry->group, entry->command);
	seq_printf(m, "Status: %d (%s)\n", entry->status, entry->status ? "FAILED" : "SUCCESS");
	seq_printf(m, "Response: %zu bytes\n", entry->response_len);

	if (entry->response_len) {
		seq_puts(m, "Response data:\n");
		for (i = 0; i < entry->response_len; i++) {
			if (i && (i % 16) == 0)
				seq_putc(m, '\n');
			seq_printf(m, "%02x ", entry->response_buf[i]);
		}
		seq_putc(m, '\n');
	}

	seq_puts(m, "\nUsage:\n");
	seq_puts(m, "  echo \"<byte0> [byte1 ...]\" > loopback\n");
	seq_puts(m, "  cat loopback\n");

	return 0;
}

static int xe_sysctrl_loopback_open(struct inode *inode, struct file *file)
{
	return single_open(file, xe_sysctrl_loopback_show, inode->i_private);
}

static const struct file_operations xe_sysctrl_loopback_fops = {
	.owner = THIS_MODULE,
	.open = xe_sysctrl_loopback_open,
	.read = seq_read,
	.write = xe_sysctrl_loopback_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int sysctrl_parse_ras_error_inject(struct xe_device *xe, char *tmp,
					  struct xe_sysctrl_diag_ras_err_inj_req *req)
{
	unsigned int nfields = 0;
	char *token;
	unsigned long val;

	while ((token = strsep(&tmp, " \t\n")) != NULL) {
		if (*token == '\0')
			continue;

		if (kstrtoul(token, 0, &val))
			goto inval;

		switch (nfields) {
		case 0:
			if (val > U16_MAX)
				goto inval;
			req->ras_block_id = val;
			break;
		case 1:
			if (val > U16_MAX)
				goto inval;
			req->ras_sub_block_id = val;
			break;
		case 2:
			if (val > U16_MAX)
				goto inval;
			req->err_type = val;
			break;
		case 3:
			if (val > U32_MAX)
				goto inval;
			req->params = val;
			break;
		default:
			xe_err(xe, "sysctrl: too many ras_error_inject arguments\n");
			return -EINVAL;
		}
		nfields++;
	}

	if (nfields < 3) {
		xe_err(xe,
		       "sysctrl: usage: <ras_block_id> <ras_sub_block_id> <err_type> [params]\n");
		return -EINVAL;
	}

	return 0;

inval:
	xe_err(xe, "sysctrl: invalid ras_error_inject token '%s'\n", token);
	return -EINVAL;
}

static ssize_t xe_sysctrl_ras_error_inject_write(struct file *file, const char __user *ubuf,
						 size_t len, loff_t *offp)
{
	char *kbuf __free(kfree) = NULL;
	struct seq_file *m = file->private_data;
	struct xe_sysctrl_debugfs_entry *entry = m->private;
	struct xe_device *xe = sc_to_xe(entry->sc);
	struct xe_sysctrl_diag_ras_err_inj_req req = {};
	struct xe_sysctrl_mailbox_command cmd = {};
	u8 resp_hdr_only[sizeof(u32)];
	size_t out_len = 0;
	int status;

	if (*offp)
		return -ESPIPE;

	if (len == 0 || len >= PAGE_SIZE)
		return -EINVAL;

	kbuf = kmalloc(len + 1, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	if (copy_from_user(kbuf, ubuf, len))
		return -EFAULT;
	kbuf[len] = '\0';

	status = sysctrl_parse_ras_error_inject(xe, kbuf, &req);
	if (status)
		return status;

	xe_sysctrl_create_command(&cmd, entry->group, entry->command,
				  &req, sizeof(req), resp_hdr_only,
				  sizeof(resp_hdr_only));

	scoped_guard(mutex, &entry->lock) {
		guard(xe_pm_runtime)(xe);
		status = xe_sysctrl_send_command(entry->sc, &cmd, &out_len);
		entry->status = status;
	}

	return status ? status : len;
}

static int xe_sysctrl_ras_error_inject_show(struct seq_file *m, void *data)
{
	struct xe_sysctrl_debugfs_entry *entry = m->private;
	struct xe_device *xe = sc_to_xe(entry->sc);
	bool fw_ready;

	scoped_guard(xe_pm_runtime, xe)
		fw_ready = xe_sysctrl_is_diag_fw_ready(xe);

	guard(mutex)(&entry->lock);

	seq_printf(m, "Command: group=0x%02x cmd=0x%02x\n", entry->group, entry->command);
	seq_printf(m, "Diag firmware ready: %s\n", str_yes_no(fw_ready));
	seq_printf(m, "Status: %d (%s)\n", entry->status, entry->status ? "FAILED" : "SUCCESS");

	seq_puts(m, "\nUsage:\n");
	seq_puts(m, "  echo \"<ras_block_id> <ras_sub_block_id> <err_type> [params]\" > ras_error_inject\n");
	seq_puts(m, "  cat ras_error_inject\n");

	return 0;
}

static int xe_sysctrl_ras_error_inject_open(struct inode *inode, struct file *file)
{
	struct xe_sysctrl_debugfs_entry *entry = inode->i_private;
	struct xe_device *xe = sc_to_xe(entry->sc);
	bool fw_ready;

	scoped_guard(xe_pm_runtime, xe)
		fw_ready = xe_sysctrl_is_diag_fw_ready(xe);

	if (!fw_ready) {
		xe_err(xe, "sysctrl: diag firmware not ready, ras_error_inject unavailable\n");
		return -ENODEV;
	}

	return single_open(file, xe_sysctrl_ras_error_inject_show, inode->i_private);
}

static const struct file_operations xe_sysctrl_ras_error_inject_fops = {
	.owner = THIS_MODULE,
	.open = xe_sysctrl_ras_error_inject_open,
	.read = seq_read,
	.write = xe_sysctrl_ras_error_inject_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void xe_sysctrl_register_entry(struct dentry *root, struct xe_sysctrl_debugfs_entry *entry,
				      struct xe_sysctrl *sc, const char *name,
				      u8 group, u8 command,
				      const struct file_operations *fops)
{
	struct xe_device *xe = sc_to_xe(sc);

	if (devm_mutex_init(xe->drm.dev, &entry->lock)) {
		xe_err(xe, "sysctrl: failed to init %s debugfs entry lock\n", name);
		return;
	}

	entry->sc = sc;
	entry->group = group;
	entry->command = command;
	entry->response_len = 0;
	entry->status = 0;
	entry->timeout_ms = 0;

	debugfs_create_file(name, 0600, root, entry, fops);
}

static char *sysctrl_mailbox_next_token(char **tmp)
{
	char *token;

	do {
		token = strsep(tmp, " \t\n");
	} while (token && *token == '\0');

	return token;
}

static ssize_t xe_sysctrl_mailbox_write(struct file *file, const char __user *ubuf,
					size_t len, loff_t *offp)
{
	char *kbuf __free(kfree) = NULL;
	u8 *input __free(kfree) = NULL;
	struct seq_file *m = file->private_data;
	struct xe_sysctrl_debugfs_entry *entry = m->private;
	struct xe_device *xe = sc_to_xe(entry->sc);
	struct xe_sysctrl_mailbox_command cmd = {};
	char *token, *tmp;
	unsigned long val;
	unsigned int timeout_ms = 0;
	size_t input_len = 0;
	size_t max_input;
	size_t out_len = 0;
	size_t resp_len = XE_SYSCTRL_MB_MAX_DATA_SIZE;
	u8 group, command;
	int status;

	if (*offp)
		return -ESPIPE;

	if (len == 0 || len >= PAGE_SIZE)
		return -EINVAL;

	kbuf = kmalloc(len + 1, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	max_input = min_t(size_t, len, XE_SYSCTRL_MB_MAX_DATA_SIZE);
	input = kmalloc(max_input, GFP_KERNEL);
	if (!input)
		return -ENOMEM;

	if (copy_from_user(kbuf, ubuf, len))
		return -EFAULT;
	kbuf[len] = '\0';

	tmp = kbuf;

	token = sysctrl_mailbox_next_token(&tmp);
	if (!token || kstrtoul(token, 0, &val) || val > 0xFF) {
		xe_err(xe, "sysctrl: invalid mailbox group id\n");
		return -EINVAL;
	}
	group = (u8)val;

	token = sysctrl_mailbox_next_token(&tmp);
	if (!token || kstrtoul(token, 0, &val) || val > SYSCTRL_HDR_COMMAND_MAX) {
		xe_err(xe, "sysctrl: invalid mailbox command id (max 0x%x)\n",
		       SYSCTRL_HDR_COMMAND_MAX);
		return -EINVAL;
	}
	command = (u8)val;

	token = sysctrl_mailbox_next_token(&tmp);
	if (token && strstarts(token, "timeout=")) {
		if (kstrtouint(token + strlen("timeout="), 0, &timeout_ms) ||
		    timeout_ms > XE_SYSCTRL_MB_MAX_TIMEOUT_MS) {
			xe_err(xe, "sysctrl: invalid mailbox timeout (max %u ms)\n",
			       XE_SYSCTRL_MB_MAX_TIMEOUT_MS);
			return -EINVAL;
		}
		token = sysctrl_mailbox_next_token(&tmp);
	}

	if (token && strstarts(token, "resplen=")) {
		unsigned int resplen;

		if (kstrtouint(token + strlen("resplen="), 0, &resplen) ||
		    !resplen || resplen > XE_SYSCTRL_MB_MAX_DATA_SIZE) {
			xe_err(xe, "sysctrl: invalid mailbox resplen (1-%zu bytes)\n",
			       (size_t)XE_SYSCTRL_MB_MAX_DATA_SIZE);
			return -EINVAL;
		}
		resp_len = resplen;
		token = sysctrl_mailbox_next_token(&tmp);
	}

	while (token) {
		if (input_len >= max_input) {
			xe_err(xe, "sysctrl: mailbox payload too large (max %zu bytes)\n",
			       max_input);
			return -EINVAL;
		}

		if (kstrtoul(token, 0, &val) || val > 0xFF) {
			xe_err(xe, "sysctrl: invalid mailbox payload byte '%s'\n", token);
			return -EINVAL;
		}

		input[input_len++] = (u8)val;
		token = sysctrl_mailbox_next_token(&tmp);
	}

	scoped_guard(mutex, &entry->lock) {
		entry->group = group;
		entry->command = command;
		entry->timeout_ms = timeout_ms;

		xe_sysctrl_create_command(&cmd, group, command, input_len ? input : NULL, input_len,
					  entry->response_buf, resp_len);
		cmd.timeout_ms = timeout_ms;

		guard(xe_pm_runtime)(xe);
		status = xe_sysctrl_send_command(entry->sc, &cmd, &out_len);
		entry->status = status;
		entry->response_len = status ? 0 : out_len;
	}

	return status ? status : len;
}

static int xe_sysctrl_mailbox_show(struct seq_file *m, void *data)
{
	struct xe_sysctrl_debugfs_entry *entry = m->private;
	size_t i;

	guard(mutex)(&entry->lock);

	seq_printf(m, "Command: group=0x%02x cmd=0x%02x\n", entry->group, entry->command);
	seq_printf(m, "Timeout: %u ms%s\n", entry->timeout_ms ?: XE_SYSCTRL_MB_DEFAULT_TIMEOUT_MS,
		   entry->timeout_ms ? "" : " (default)");
	seq_printf(m, "Status: %d (%s)\n", entry->status, entry->status ? "FAILED" : "SUCCESS");
	seq_printf(m, "Response: %zu bytes\n", entry->response_len);

	if (entry->response_len) {
		seq_puts(m, "Response data:\n");
		for (i = 0; i < entry->response_len; i++) {
			if (i && (i % 16) == 0)
				seq_putc(m, '\n');
			seq_printf(m, "%02x ", entry->response_buf[i]);
		}
		seq_putc(m, '\n');
	}

	seq_puts(m, "\nUsage:\n");
	seq_puts(m, "  echo \"<group> <command> [timeout=<ms>] [resplen=<N>] [byte0 byte1 ...]\" > mailbox\n");
	seq_puts(m, "  cat mailbox\n");

	return 0;
}

static int xe_sysctrl_mailbox_open(struct inode *inode, struct file *file)
{
	return single_open(file, xe_sysctrl_mailbox_show, inode->i_private);
}

static const struct file_operations xe_sysctrl_mailbox_fops = {
	.owner = THIS_MODULE,
	.open = xe_sysctrl_mailbox_open,
	.read = seq_read,
	.write = xe_sysctrl_mailbox_write,
	.llseek = seq_lseek,
	.release = single_release,
};

/**
 * xe_sysctrl_debugfs_register - Register debugfs entries for System Controller
 * @sc: xe_sysctrl instance
 * @parent: parent debugfs directory
 */
void xe_sysctrl_debugfs_register(struct xe_sysctrl *sc, struct dentry *parent)
{
	struct dentry *root;

	root = debugfs_create_dir("sc", parent);
	if (IS_ERR(root))
		return;

	sc->debugfs.root = root;

	xe_sysctrl_register_entry(root, &sc->debugfs.loopback, sc, "loopback",
				  XE_SYSCTRL_GROUP_CORE, XE_SYSCTRL_CMD_LOOPBACK,
				  &xe_sysctrl_loopback_fops);

	xe_sysctrl_register_entry(root, &sc->debugfs.ras_error_inject, sc, "ras_error_inject",
				  XE_SYSCTRL_GROUP_DIAG, XE_SYSCTRL_CMD_DIAG_RAS_ERR_INJECT,
				  &xe_sysctrl_ras_error_inject_fops);

	xe_sysctrl_register_entry(root, &sc->debugfs.mailbox, sc, "mailbox", 0, 0,
				  &xe_sysctrl_mailbox_fops);
}
