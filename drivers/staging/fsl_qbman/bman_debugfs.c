/* Copyright 2010-2011 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <linux/module.h>
#include <linux/fsl_bman.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/of.h>

static struct dentry *dfs_root; /* debugfs root directory */

/*******************************************************************************
 *  Query Buffer Pool State
 ******************************************************************************/
static int query_bp_state_show(struct seq_file *file, void *offset)
{
	int ret;
	struct bm_pool_state state;
	int i, j;
	u32 mask;

	memset(&state, 0, sizeof(struct bm_pool_state));
	ret = bman_query_pools(&state);
	if (ret) {
		seq_printf(file, "Error %d\n", ret);
		return 0;
	}
	seq_puts(file, "bp_id  free_buffers_avail  bp_depleted\n");
	for (i = 0; i < 2; i++) {
		mask = 0x80000000;
		for (j = 0; j < 32; j++) {
			seq_printf(file,
			 "  %-2u           %-3s             %-3s\n",
			 (i*32)+j,
			 (state.as.state.__state[i] & mask) ? "no" : "yes",
			 (state.ds.state.__state[i] & mask) ? "yes" : "no");
			 mask >>= 1;
		}
	}
	return 0;
}

static int query_bp_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, query_bp_state_show, NULL);
}

static const struct file_operations query_bp_state_fops = {
	.owner          = THIS_MODULE,
	.open		= query_bp_state_open,
	.read           = seq_read,
	.release	= single_release,
};

static int __init bman_debugfs_module_init(void)
{
	int ret = 0;
	struct dentry *d;
	struct device_node *dn;

	dn = of_find_compatible_node(NULL, NULL, "fsl,bman");
	if (!dn) {
		pr_debug("No fsl,bman node\n");
		return 0;
	}
	dfs_root = debugfs_create_dir("bman", NULL);
	if (dfs_root == NULL) {
		ret = -ENOMEM;
		pr_err("Cannot create bman debugfs dir\n");
		goto _return;
	}
	d = debugfs_create_file("query_bp_state",
		S_IRUGO,
		dfs_root,
		NULL,
		&query_bp_state_fops);
	if (d == NULL) {
		ret = -ENOMEM;
		pr_err("Cannot create query_bp_state\n");
		goto _return;
	}
	return 0;

_return:
	debugfs_remove_recursive(dfs_root);
	return ret;
}

static void __exit bman_debugfs_module_exit(void)
{
	debugfs_remove_recursive(dfs_root);
}


module_init(bman_debugfs_module_init);
module_exit(bman_debugfs_module_exit);
MODULE_LICENSE("Dual BSD/GPL");
