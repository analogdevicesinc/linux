/* Copyright 2008-2013 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
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
#include <linux/fsl_qman.h>	/* struct qm_mcr_querycgr */
#include <linux/debugfs.h>
#include "dpaa_debugfs.h"
#include "dpaa_eth.h" /* struct dpa_priv_s, dpa_percpu_priv_s, dpa_bp */

#define DPA_DEBUGFS_DESCRIPTION "FSL DPAA Ethernet debugfs entries"
#define DPA_ETH_DEBUGFS_ROOT "fsl_dpa"

static struct dentry *dpa_debugfs_root;

static int __cold dpa_debugfs_loop_open(struct inode *inode, struct file *file);
static ssize_t dpa_loop_write(struct file *f,
	const char __user *buf, size_t count, loff_t *off);

static const struct file_operations dpa_debugfs_lp_fops = {
	.open		= dpa_debugfs_loop_open,
	.write		= dpa_loop_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dpa_debugfs_loop_show(struct seq_file *file, void *offset)
{
	struct dpa_priv_s *priv;

	BUG_ON(offset == NULL);

	priv = netdev_priv((struct net_device *)file->private);
	seq_printf(file, "%d->%d\n", priv->loop_id, priv->loop_to);

	return 0;
}

static int user_input_convert(const char __user *user_buf, size_t count,
			      long *val)
{
	char buf[12];

	if (count > sizeof(buf) - 1)
		return -EINVAL;
	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;
	buf[count] = '\0';
	if (kstrtol(buf, 0, val))
		return -EINVAL;
	return 0;
}

static ssize_t dpa_loop_write(struct file *f,
	const char __user *buf, size_t count, loff_t *off)
{
	struct dpa_priv_s *priv;
	struct net_device *netdev;
	struct seq_file *sf;
	int ret;
	long val;

	ret = user_input_convert(buf, count, &val);
	if (ret)
		return ret;

	sf = (struct seq_file *)f->private_data;
	netdev = (struct net_device *)sf->private;
	priv = netdev_priv(netdev);

	priv->loop_to = ((val < 0) || (val > 20)) ? -1 : val;

	return count;
}

static int __cold dpa_debugfs_loop_open(struct inode *inode, struct file *file)
{
	int			 _errno;
	const struct net_device	*net_dev;

	_errno = single_open(file, dpa_debugfs_loop_show, inode->i_private);
	if (unlikely(_errno < 0)) {
		net_dev = (struct net_device *)inode->i_private;

		if (netif_msg_drv((struct dpa_priv_s *)netdev_priv(net_dev)))
			netdev_err(net_dev, "single_open() = %d\n",
					_errno);
	}

	return _errno;
}


int dpa_netdev_debugfs_create(struct net_device *net_dev)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	static int cnt;
	char loop_file_name[100];

	if (unlikely(dpa_debugfs_root == NULL)) {
		pr_err(KBUILD_MODNAME ": %s:%hu:%s(): \t%s\n",
				   KBUILD_BASENAME".c", __LINE__, __func__,
				   "root debugfs missing, possible module ordering issue");
		return -ENOMEM;
	}

	sprintf(loop_file_name, "eth%d_loop", ++cnt);
	priv->debugfs_loop_file = debugfs_create_file(loop_file_name,
							 S_IRUGO,
							 dpa_debugfs_root,
							 net_dev,
							 &dpa_debugfs_lp_fops);
	if (unlikely(priv->debugfs_loop_file == NULL)) {
		netdev_err(net_dev, "debugfs_create_file(%s/%s)",
				dpa_debugfs_root->d_iname,
				loop_file_name);

		return -ENOMEM;
	}
	return 0;
}

void dpa_netdev_debugfs_remove(struct net_device *net_dev)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);

	debugfs_remove(priv->debugfs_loop_file);
}

int __init dpa_debugfs_module_init(void)
{
	int	 _errno = 0;

	pr_info(KBUILD_MODNAME ": " DPA_DEBUGFS_DESCRIPTION "\n");

	dpa_debugfs_root = debugfs_create_dir(DPA_ETH_DEBUGFS_ROOT, NULL);

	if (unlikely(dpa_debugfs_root == NULL)) {
		_errno = -ENOMEM;
		pr_err(KBUILD_MODNAME ": %s:%hu:%s():\n",
				   KBUILD_BASENAME".c", __LINE__, __func__);
		pr_err("\tdebugfs_create_dir(%s/"KBUILD_MODNAME") = %d\n",
			   DPA_ETH_DEBUGFS_ROOT, _errno);
	}

	return _errno;
}

void __exit dpa_debugfs_module_exit(void)
{
	debugfs_remove(dpa_debugfs_root);
}
