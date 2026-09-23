/*
*  Copyright (c) 2004 The Regents of the University of Michigan.
*  Copyright (c) 2012 Jeff Layton <jlayton@redhat.com>
*  All rights reserved.
*
*  Andy Adamson <andros@citi.umich.edu>
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*  3. Neither the name of the University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
*  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
*  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
*  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
*  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
*  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
*  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
*  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include <crypto/sha2.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <net/net_namespace.h>
#include <linux/sunrpc/rpc_pipe_fs.h>
#include <linux/sunrpc/clnt.h>
#include <linux/nfsd/cld.h>

#include "nfsd.h"
#include "nfs4ctl.h"
#include "state.h"
#include "netns.h"

#define NFSDDBG_FACILITY                NFSDDBG_PROC

/* Declarations */
struct nfsd4_client_tracking_ops {
	int (*init)(struct net *);
	void (*exit)(struct net *);
	void (*create)(struct nfs4_client *);
	void (*remove)(struct nfs4_client *);
	int (*check)(struct nfs4_client *);
	void (*grace_done)(struct nfsd_net *);
	uint8_t version;
	size_t msglen;
};

static const struct nfsd4_client_tracking_ops nfsd4_cld_tracking_ops;
static const struct nfsd4_client_tracking_ops nfsd4_cld_tracking_ops_v2;

/* Globals */
#define NFSD_PIPE_DIR		"nfsd"
#define NFSD_CLD_PIPE		"cld"

/* per-net-ns structure for holding cld upcall info */
struct cld_net {
	struct rpc_pipe		*cn_pipe;
	spinlock_t		 cn_lock;
	struct list_head	 cn_list;
	unsigned int		 cn_xid;
};

struct cld_upcall {
	struct list_head	 cu_list;
	struct cld_net		*cu_net;
	struct completion	 cu_done;
	/* daemon has read the whole upcall; protected by cn_lock */
	bool			 cu_inflight;
	struct rpc_pipe_msg	 cu_pipe_msg;
	union {
		struct cld_msg_hdr	 cu_hdr;
		struct cld_msg		 cu_msg;
		struct cld_msg_v2	 cu_msg_v2;
	} cu_u;
};

static int
__cld_pipe_upcall(struct rpc_pipe *pipe, void *cmsg, struct nfsd_net *nn)
{
	int ret;
	struct cld_upcall *cup = container_of(cmsg, struct cld_upcall, cu_u);
	struct rpc_pipe_msg *msg = &cup->cu_pipe_msg;

	memset(msg, 0, sizeof(*msg));
	msg->data = cmsg;
	msg->len = nn->client_tracking_ops->msglen;

	ret = rpc_queue_upcall(pipe, msg);
	if (ret < 0) {
		goto out;
	}

	wait_for_completion(&cup->cu_done);

	if (msg->errno < 0)
		ret = msg->errno;
out:
	return ret;
}

static int
cld_pipe_upcall(struct rpc_pipe *pipe, void *cmsg, struct nfsd_net *nn)
{
	int ret;

	/*
	 * -EAGAIN occurs when pipe is closed and reopened while there are
	 *  upcalls queued.
	 */
	do {
		ret = __cld_pipe_upcall(pipe, cmsg, nn);
	} while (ret == -EAGAIN);

	return ret;
}

static ssize_t
__cld_pipe_inprogress_downcall(const struct cld_msg_v2 __user *cmsg,
		struct nfsd_net *nn)
{
	uint8_t cmd, princhashlen;
	struct xdr_netobj name, princhash = { .len = 0, .data = NULL };
	char *namecopy __free(kfree) = NULL;
	char *princhashcopy __free(kfree) = NULL;
	uint16_t namelen;

	if (get_user(cmd, &cmsg->cm_cmd)) {
		dprintk("%s: error when copying cmd from userspace", __func__);
		return -EFAULT;
	}
	if (cmd == Cld_GraceStart) {
		if (nn->client_tracking_ops->version >= 2) {
			const struct cld_clntinfo __user *ci;

			ci = &cmsg->cm_u.cm_clntinfo;
			if (get_user(namelen, &ci->cc_name.cn_len))
				return -EFAULT;
			if (namelen == 0 || namelen > NFS4_OPAQUE_LIMIT) {
				dprintk("%s: invalid namelen (%u)", __func__, namelen);
				return -EINVAL;
			}
			namecopy = memdup_user(&ci->cc_name.cn_id, namelen);
			if (IS_ERR(namecopy))
				return PTR_ERR(namecopy);
			name.data = namecopy;
			name.len = namelen;
			if (get_user(princhashlen, &ci->cc_princhash.cp_len))
				return -EFAULT;
			if (princhashlen > SHA256_DIGEST_SIZE) {
				dprintk("%s: invalid princhashlen (%u)",
					__func__, princhashlen);
				return -EINVAL;
			}
			if (princhashlen > 0) {
				princhashcopy = memdup_user(
					&ci->cc_princhash.cp_data,
					princhashlen);
				if (IS_ERR(princhashcopy))
					return PTR_ERR(princhashcopy);
				princhash.data = princhashcopy;
				princhash.len = princhashlen;
			} else
				princhash.len = 0;
		} else {
			const struct cld_name __user *cnm;

			cnm = &cmsg->cm_u.cm_name;
			if (get_user(namelen, &cnm->cn_len))
				return -EFAULT;
			if (namelen == 0 || namelen > NFS4_OPAQUE_LIMIT) {
				dprintk("%s: invalid namelen (%u)", __func__, namelen);
				return -EINVAL;
			}
			namecopy = memdup_user(&cnm->cn_id, namelen);
			if (IS_ERR(namecopy))
				return PTR_ERR(namecopy);
			name.data = namecopy;
			name.len = namelen;
		}
		if (!nfs4_client_to_reclaim(name, princhash, nn))
			return -EFAULT;
		return nn->client_tracking_ops->msglen;
	}
	return -EFAULT;
}

static ssize_t
cld_pipe_downcall(struct file *filp, const char __user *src, size_t mlen)
{
	struct cld_upcall *tmp, *cup;
	struct cld_msg_hdr __user *hdr = (struct cld_msg_hdr __user *)src;
	struct cld_msg_v2 __user *cmsg = (struct cld_msg_v2 __user *)src;
	uint32_t xid;
	struct nfsd_net *nn = net_generic(file_inode(filp)->i_sb->s_fs_info,
						nfsd_net_id);
	struct cld_net *cn = nn->cld_net;
	int16_t status;

	if (mlen != nn->client_tracking_ops->msglen) {
		dprintk("%s: got %zu bytes, expected %zu\n", __func__, mlen,
			nn->client_tracking_ops->msglen);
		return -EINVAL;
	}

	/* copy just the xid so we can try to find that */
	if (copy_from_user(&xid, &hdr->cm_xid, sizeof(xid)) != 0) {
		dprintk("%s: error when copying xid from userspace", __func__);
		return -EFAULT;
	}

	/*
	 * copy the status so we know whether to remove the upcall from the
	 * list (for -EINPROGRESS, we just want to make sure the xid is
	 * valid, not remove the upcall from the list)
	 */
	if (get_user(status, &hdr->cm_status)) {
		dprintk("%s: error when copying status from userspace", __func__);
		return -EFAULT;
	}

	/* walk the list and find corresponding xid */
	cup = NULL;
	spin_lock(&cn->cn_lock);
	list_for_each_entry(tmp, &cn->cn_list, cu_list) {
		if (get_unaligned(&tmp->cu_u.cu_hdr.cm_xid) == xid) {
			/*
			 * Completing now would return the waiter
			 * while its message is still queued on the
			 * pipe.
			 */
			if (!tmp->cu_inflight)
				break;
			cup = tmp;
			if (status != -EINPROGRESS)
				list_del_init(&cup->cu_list);
			break;
		}
	}
	spin_unlock(&cn->cn_lock);

	/* no upcall to complete? */
	if (!cup) {
		dprintk("%s: no completable upcall -- xid=%u\n", __func__, xid);
		return -EINVAL;
	}

	if (status == -EINPROGRESS)
		return __cld_pipe_inprogress_downcall(cmsg, nn);

	if (copy_from_user(&cup->cu_u.cu_msg_v2, src, mlen) != 0) {
		cup->cu_pipe_msg.errno = -EFAULT;
		complete(&cup->cu_done);
		return -EFAULT;
	}

	complete(&cup->cu_done);
	return mlen;
}

static void
cld_pipe_destroy_msg(struct rpc_pipe_msg *msg)
{
	struct cld_upcall *cup = container_of(msg, struct cld_upcall,
						 cu_pipe_msg);
	struct cld_net *cn = cup->cu_net;

	/*
	 * errno >= 0 means the daemon read the whole message and a
	 * downcall will complete the upcall.
	 */
	spin_lock(&cn->cn_lock);
	cup->cu_inflight = msg->errno >= 0;
	spin_unlock(&cn->cn_lock);

	if (msg->errno >= 0)
		return;

	complete(&cup->cu_done);
}

/*
 * An upcall the daemon has consumed is off every pipe list, so the
 * purge in rpc_pipe_release() and rpc_close_pipes() cannot reach it.
 * Release its waiter here instead; no downcall can arrive now.
 */
static void
cld_release_pipe(struct inode *inode)
{
	struct nfsd_net *nn = net_generic(inode->i_sb->s_fs_info, nfsd_net_id);
	struct cld_net *cn = nn->cld_net;
	struct cld_upcall *cup;

	spin_lock(&cn->cn_lock);
	list_for_each_entry(cup, &cn->cn_list, cu_list) {
		if (!cup->cu_inflight)
			continue;
		cup->cu_inflight = false;
		cup->cu_pipe_msg.errno = -EPIPE;
		complete(&cup->cu_done);
	}
	spin_unlock(&cn->cn_lock);
}

static const struct rpc_pipe_ops cld_upcall_ops = {
	.upcall		= rpc_pipe_generic_upcall,
	.downcall	= cld_pipe_downcall,
	.destroy_msg	= cld_pipe_destroy_msg,
	.release_pipe	= cld_release_pipe,
};

static int
nfsd4_cld_register_sb(struct super_block *sb, struct rpc_pipe *pipe)
{
	struct dentry *dir;
	int err;

	dir = rpc_d_lookup_sb(sb, NFSD_PIPE_DIR);
	if (dir == NULL)
		return -ENOENT;
	err = rpc_mkpipe_dentry(dir, NFSD_CLD_PIPE, NULL, pipe);
	dput(dir);
	return err;
}

static int
nfsd4_cld_register_net(struct net *net, struct rpc_pipe *pipe)
{
	struct super_block *sb;
	int err;

	sb = rpc_get_sb_net(net);
	if (!sb)
		return 0;
	err = nfsd4_cld_register_sb(sb, pipe);
	rpc_put_sb_net(net);
	return err;
}

static void
nfsd4_cld_unregister_net(struct net *net, struct rpc_pipe *pipe)
{
	struct super_block *sb;

	sb = rpc_get_sb_net(net);
	if (sb) {
		rpc_unlink(pipe);
		rpc_put_sb_net(net);
	}
}

/* Initialize rpc_pipefs pipe for communication with client tracking daemon */
static int
__nfsd4_init_cld_pipe(struct net *net)
{
	int ret;
	struct nfsd_net *nn = net_generic(net, nfsd_net_id);
	struct cld_net *cn;

	if (nn->cld_net)
		return 0;

	cn = kzalloc_obj(*cn);
	if (!cn) {
		ret = -ENOMEM;
		goto err;
	}

	cn->cn_pipe = rpc_mkpipe_data(&cld_upcall_ops, RPC_PIPE_WAIT_FOR_OPEN);
	if (IS_ERR(cn->cn_pipe)) {
		ret = PTR_ERR(cn->cn_pipe);
		goto err;
	}
	spin_lock_init(&cn->cn_lock);
	INIT_LIST_HEAD(&cn->cn_list);

	/*
	 * The pipe's methods reach @cn through nn->cld_net, so set
	 * it before the pipe can be opened.
	 */
	nn->cld_net = cn;

	ret = nfsd4_cld_register_net(net, cn->cn_pipe);
	if (unlikely(ret))
		goto err_destroy_data;

	return 0;

err_destroy_data:
	nn->cld_net = NULL;
	rpc_destroy_pipe_data(cn->cn_pipe);
err:
	kfree(cn);
	printk(KERN_ERR "NFSD: unable to create nfsdcld upcall pipe (%d)\n",
			ret);
	return ret;
}

static int
nfsd4_init_cld_pipe(struct net *net)
{
	int status;

	status = __nfsd4_init_cld_pipe(net);
	if (!status)
		pr_info("NFSD: Using old nfsdcld client tracking operations.\n");
	return status;
}

static void
nfsd4_remove_cld_pipe(struct net *net)
{
	struct nfsd_net *nn = net_generic(net, nfsd_net_id);
	struct cld_net *cn = nn->cld_net;

	nfsd4_cld_unregister_net(net, cn->cn_pipe);
	rpc_destroy_pipe_data(cn->cn_pipe);
	kfree(nn->cld_net);
	nn->cld_net = NULL;
}

static struct cld_upcall *
alloc_cld_upcall(struct nfsd_net *nn)
{
	struct cld_upcall *new, *tmp;
	struct cld_net *cn = nn->cld_net;

	new = kzalloc_obj(*new);
	if (!new)
		return new;

	/* FIXME: hard cap on number in flight? */
restart_search:
	spin_lock(&cn->cn_lock);
	list_for_each_entry(tmp, &cn->cn_list, cu_list) {
		if (tmp->cu_u.cu_msg.cm_xid == cn->cn_xid) {
			cn->cn_xid++;
			spin_unlock(&cn->cn_lock);
			goto restart_search;
		}
	}
	init_completion(&new->cu_done);
	new->cu_u.cu_msg.cm_vers = nn->client_tracking_ops->version;
	put_unaligned(cn->cn_xid++, &new->cu_u.cu_msg.cm_xid);
	new->cu_net = cn;
	list_add(&new->cu_list, &cn->cn_list);
	spin_unlock(&cn->cn_lock);

	dprintk("%s: allocated xid %u\n", __func__, new->cu_u.cu_msg.cm_xid);

	return new;
}

static void
free_cld_upcall(struct cld_upcall *victim)
{
	struct cld_net *cn = victim->cu_net;

	spin_lock(&cn->cn_lock);
	list_del(&victim->cu_list);
	spin_unlock(&cn->cn_lock);
	kfree(victim);
}

/* Ask daemon to create a new record */
static void
nfsd4_cld_create(struct nfs4_client *clp)
{
	int ret;
	struct cld_upcall *cup;
	struct nfsd_net *nn = net_generic(clp->net, nfsd_net_id);
	struct cld_net *cn = nn->cld_net;

	/* Don't upcall if it's already stored */
	if (test_bit(NFSD4_CLIENT_STABLE, &clp->cl_flags))
		return;

	cup = alloc_cld_upcall(nn);
	if (!cup) {
		ret = -ENOMEM;
		goto out_err;
	}

	cup->cu_u.cu_msg.cm_cmd = Cld_Create;
	cup->cu_u.cu_msg.cm_u.cm_name.cn_len = clp->cl_name.len;
	memcpy(cup->cu_u.cu_msg.cm_u.cm_name.cn_id, clp->cl_name.data,
			clp->cl_name.len);

	ret = cld_pipe_upcall(cn->cn_pipe, &cup->cu_u.cu_msg, nn);
	if (!ret) {
		ret = cup->cu_u.cu_msg.cm_status;
		set_bit(NFSD4_CLIENT_STABLE, &clp->cl_flags);
	}

	free_cld_upcall(cup);
out_err:
	if (ret)
		printk(KERN_ERR "NFSD: Unable to create client "
				"record on stable storage: %d\n", ret);
}

/* Ask daemon to create a new record */
static void
nfsd4_cld_create_v2(struct nfs4_client *clp)
{
	int ret;
	struct cld_upcall *cup;
	struct nfsd_net *nn = net_generic(clp->net, nfsd_net_id);
	struct cld_net *cn = nn->cld_net;
	struct cld_msg_v2 *cmsg;
	char *principal = NULL;

	/* Don't upcall if it's already stored */
	if (test_bit(NFSD4_CLIENT_STABLE, &clp->cl_flags))
		return;

	cup = alloc_cld_upcall(nn);
	if (!cup) {
		ret = -ENOMEM;
		goto out_err;
	}

	cmsg = &cup->cu_u.cu_msg_v2;
	cmsg->cm_cmd = Cld_Create;
	cmsg->cm_u.cm_clntinfo.cc_name.cn_len = clp->cl_name.len;
	memcpy(cmsg->cm_u.cm_clntinfo.cc_name.cn_id, clp->cl_name.data,
			clp->cl_name.len);
	if (clp->cl_cred.cr_raw_principal)
		principal = clp->cl_cred.cr_raw_principal;
	else if (clp->cl_cred.cr_principal)
		principal = clp->cl_cred.cr_principal;
	if (principal) {
		sha256(principal, strlen(principal),
		       cmsg->cm_u.cm_clntinfo.cc_princhash.cp_data);
		cmsg->cm_u.cm_clntinfo.cc_princhash.cp_len = SHA256_DIGEST_SIZE;
	} else
		cmsg->cm_u.cm_clntinfo.cc_princhash.cp_len = 0;

	ret = cld_pipe_upcall(cn->cn_pipe, cmsg, nn);
	if (!ret) {
		ret = cmsg->cm_status;
		set_bit(NFSD4_CLIENT_STABLE, &clp->cl_flags);
	}

	free_cld_upcall(cup);
out_err:
	if (ret)
		pr_err("NFSD: Unable to create client record on stable storage: %d\n",
				ret);
}

/* Ask daemon to create a new record */
static void
nfsd4_cld_remove(struct nfs4_client *clp)
{
	int ret;
	struct cld_upcall *cup;
	struct nfsd_net *nn = net_generic(clp->net, nfsd_net_id);
	struct cld_net *cn = nn->cld_net;

	/* Don't upcall if it's already removed */
	if (!test_bit(NFSD4_CLIENT_STABLE, &clp->cl_flags))
		return;

	cup = alloc_cld_upcall(nn);
	if (!cup) {
		ret = -ENOMEM;
		goto out_err;
	}

	cup->cu_u.cu_msg.cm_cmd = Cld_Remove;
	cup->cu_u.cu_msg.cm_u.cm_name.cn_len = clp->cl_name.len;
	memcpy(cup->cu_u.cu_msg.cm_u.cm_name.cn_id, clp->cl_name.data,
			clp->cl_name.len);

	ret = cld_pipe_upcall(cn->cn_pipe, &cup->cu_u.cu_msg, nn);
	if (!ret) {
		ret = cup->cu_u.cu_msg.cm_status;
		clear_bit(NFSD4_CLIENT_STABLE, &clp->cl_flags);
	}

	free_cld_upcall(cup);
out_err:
	if (ret)
		printk(KERN_ERR "NFSD: Unable to remove client "
				"record from stable storage: %d\n", ret);
}

/*
 * For older nfsdcld's that do not allow us to "slurp" the clients
 * from the tracking database during startup.
 *
 * Check for presence of a record, and update its timestamp
 */
static int
nfsd4_cld_check_v0(struct nfs4_client *clp)
{
	int ret;
	struct cld_upcall *cup;
	struct nfsd_net *nn = net_generic(clp->net, nfsd_net_id);
	struct cld_net *cn = nn->cld_net;

	/* Don't upcall if one was already stored during this grace pd */
	if (test_bit(NFSD4_CLIENT_STABLE, &clp->cl_flags))
		return 0;

	cup = alloc_cld_upcall(nn);
	if (!cup) {
		printk(KERN_ERR "NFSD: Unable to check client record on "
				"stable storage: %d\n", -ENOMEM);
		return -ENOMEM;
	}

	cup->cu_u.cu_msg.cm_cmd = Cld_Check;
	cup->cu_u.cu_msg.cm_u.cm_name.cn_len = clp->cl_name.len;
	memcpy(cup->cu_u.cu_msg.cm_u.cm_name.cn_id, clp->cl_name.data,
			clp->cl_name.len);

	ret = cld_pipe_upcall(cn->cn_pipe, &cup->cu_u.cu_msg, nn);
	if (!ret) {
		ret = cup->cu_u.cu_msg.cm_status;
		set_bit(NFSD4_CLIENT_STABLE, &clp->cl_flags);
	}

	free_cld_upcall(cup);
	return ret;
}

/*
 * For newer nfsdcld's that allow us to "slurp" the clients
 * from the tracking database during startup.
 *
 * Check for presence of a record in the reclaim_str_hashtbl
 */
static int
nfsd4_cld_check(struct nfs4_client *clp)
{
	struct nfs4_client_reclaim *crp;
	struct nfsd_net *nn = net_generic(clp->net, nfsd_net_id);

	/* did we already find that this client is stable? */
	if (test_bit(NFSD4_CLIENT_STABLE, &clp->cl_flags))
		return 0;

	/* look for it in the reclaim hashtable otherwise */
	down_read(&nn->reclaim_str_hashtbl_lock);
	crp = nfsd4_find_reclaim_client(clp->cl_name, nn);
	if (crp)
		goto found;

	up_read(&nn->reclaim_str_hashtbl_lock);
	return -ENOENT;
found:
	crp->cr_clp = clp;
	up_read(&nn->reclaim_str_hashtbl_lock);
	return 0;
}

static int
nfsd4_cld_check_v2(struct nfs4_client *clp)
{
	struct nfsd_net *nn = net_generic(clp->net, nfsd_net_id);
	struct nfs4_client_reclaim *crp;
	unsigned int princhashlen;
	char *principal = NULL;

	/* did we already find that this client is stable? */
	if (test_bit(NFSD4_CLIENT_STABLE, &clp->cl_flags))
		return 0;

	/* look for it in the reclaim hashtable otherwise */
	down_read(&nn->reclaim_str_hashtbl_lock);
	crp = nfsd4_find_reclaim_client(clp->cl_name, nn);
	if (crp)
		goto found;

	up_read(&nn->reclaim_str_hashtbl_lock);
	return -ENOENT;
found:
	princhashlen = crp->cr_princhash.len;
	if (princhashlen) {
		u8 digest[SHA256_DIGEST_SIZE];
		u8 *pdata;

		if (clp->cl_cred.cr_raw_principal)
			principal = clp->cl_cred.cr_raw_principal;
		else if (clp->cl_cred.cr_principal)
			principal = clp->cl_cred.cr_principal;
		if (principal == NULL) {
			up_read(&nn->reclaim_str_hashtbl_lock);
			return -ENOENT;
		}
		sha256(principal, strlen(principal), digest);
		pdata = crp->cr_princhash.data;
		if (memcmp(pdata, digest, princhashlen)) {
			up_read(&nn->reclaim_str_hashtbl_lock);
			return -ENOENT;
		}
	}
	crp->cr_clp = clp;
	up_read(&nn->reclaim_str_hashtbl_lock);
	return 0;
}

static int
nfsd4_cld_grace_start(struct nfsd_net *nn)
{
	int ret;
	struct cld_upcall *cup;
	struct cld_net *cn = nn->cld_net;

	cup = alloc_cld_upcall(nn);
	if (!cup) {
		ret = -ENOMEM;
		goto out_err;
	}

	cup->cu_u.cu_msg.cm_cmd = Cld_GraceStart;
	ret = cld_pipe_upcall(cn->cn_pipe, &cup->cu_u.cu_msg, nn);
	if (!ret)
		ret = cup->cu_u.cu_msg.cm_status;

	free_cld_upcall(cup);
out_err:
	if (ret)
		dprintk("%s: Unable to get clients from userspace: %d\n",
			__func__, ret);
	return ret;
}

/* For older nfsdcld's that need cm_gracetime */
static void
nfsd4_cld_grace_done_v0(struct nfsd_net *nn)
{
	int ret;
	struct cld_upcall *cup;
	struct cld_net *cn = nn->cld_net;

	cup = alloc_cld_upcall(nn);
	if (!cup) {
		ret = -ENOMEM;
		goto out_err;
	}

	cup->cu_u.cu_msg.cm_cmd = Cld_GraceDone;
	cup->cu_u.cu_msg.cm_u.cm_gracetime = nn->boot_time;
	ret = cld_pipe_upcall(cn->cn_pipe, &cup->cu_u.cu_msg, nn);
	if (!ret)
		ret = cup->cu_u.cu_msg.cm_status;

	free_cld_upcall(cup);
out_err:
	if (ret)
		printk(KERN_ERR "NFSD: Unable to end grace period: %d\n", ret);
}

/*
 * For newer nfsdcld's that do not need cm_gracetime.  We also need to call
 * nfs4_release_reclaim() to clear out the reclaim_str_hashtbl.
 */
static void
nfsd4_cld_grace_done(struct nfsd_net *nn)
{
	int ret;
	struct cld_upcall *cup;
	struct cld_net *cn = nn->cld_net;

	cup = alloc_cld_upcall(nn);
	if (!cup) {
		ret = -ENOMEM;
		goto out_err;
	}

	cup->cu_u.cu_msg.cm_cmd = Cld_GraceDone;
	ret = cld_pipe_upcall(cn->cn_pipe, &cup->cu_u.cu_msg, nn);
	if (!ret)
		ret = cup->cu_u.cu_msg.cm_status;

	free_cld_upcall(cup);
out_err:
	nfs4_release_reclaim(nn);
	if (ret)
		printk(KERN_ERR "NFSD: Unable to end grace period: %d\n", ret);
}

static int
nfs4_cld_state_init(struct net *net)
{
	struct nfsd_net *nn = net_generic(net, nfsd_net_id);
	int i;

	nn->reclaim_str_hashtbl = kmalloc_objs(struct list_head,
					       CLIENT_HASH_SIZE);
	if (!nn->reclaim_str_hashtbl)
		return -ENOMEM;

	for (i = 0; i < CLIENT_HASH_SIZE; i++)
		INIT_LIST_HEAD(&nn->reclaim_str_hashtbl[i]);
	nn->reclaim_str_hashtbl_size = 0;
	init_rwsem(&nn->reclaim_str_hashtbl_lock);
	set_bit(NFSD_NET_TRACK_RECLAIM_COMPLETES, &nn->flags);
	atomic_set(&nn->nr_reclaim_complete, 0);

	return 0;
}

static void
nfs4_cld_state_shutdown(struct net *net)
{
	struct nfsd_net *nn = net_generic(net, nfsd_net_id);

	clear_bit(NFSD_NET_TRACK_RECLAIM_COMPLETES, &nn->flags);
	kfree(nn->reclaim_str_hashtbl);
}

static bool
cld_running(struct nfsd_net *nn)
{
	struct cld_net *cn = nn->cld_net;
	struct rpc_pipe *pipe = cn->cn_pipe;

	return pipe->nreaders || pipe->nwriters;
}

static int
nfsd4_cld_get_version(struct nfsd_net *nn)
{
	int ret = 0;
	struct cld_upcall *cup;
	struct cld_net *cn = nn->cld_net;
	uint8_t version;

	cup = alloc_cld_upcall(nn);
	if (!cup) {
		ret = -ENOMEM;
		goto out_err;
	}
	cup->cu_u.cu_msg.cm_cmd = Cld_GetVersion;
	ret = cld_pipe_upcall(cn->cn_pipe, &cup->cu_u.cu_msg, nn);
	if (!ret) {
		ret = cup->cu_u.cu_msg.cm_status;
		if (ret)
			goto out_free;
		version = cup->cu_u.cu_msg.cm_u.cm_version;
		dprintk("%s: userspace returned version %u\n",
				__func__, version);
		if (version < 1)
			version = 1;
		else if (version > CLD_UPCALL_VERSION)
			version = CLD_UPCALL_VERSION;

		switch (version) {
		case 1:
			nn->client_tracking_ops = &nfsd4_cld_tracking_ops;
			break;
		case 2:
			nn->client_tracking_ops = &nfsd4_cld_tracking_ops_v2;
			break;
		default:
			break;
		}
	}
out_free:
	free_cld_upcall(cup);
out_err:
	if (ret)
		dprintk("%s: Unable to get version from userspace: %d\n",
			__func__, ret);
	return ret;
}

static int
nfsd4_cld_tracking_init(struct net *net)
{
	int status;
	struct nfsd_net *nn = net_generic(net, nfsd_net_id);
	bool running;
	int retries = 10;

	status = nfs4_cld_state_init(net);
	if (status)
		return status;

	status = __nfsd4_init_cld_pipe(net);
	if (status)
		goto err_shutdown;

	/*
	 * rpc pipe upcalls take 30 seconds to time out, so do not queue an
	 * upcall until nfsdcld has opened the pipefs file just created.
	 * nfsdcld should already be running before nfsd is started, so
	 * this wait is short.
	 */
	while (!(running = cld_running(nn)) && retries--)
		msleep(100);

	if (!running) {
		status = -ETIMEDOUT;
		goto err_remove;
	}

	status = nfsd4_cld_get_version(nn);
	if (status == -EOPNOTSUPP)
		pr_warn("NFSD: nfsdcld GetVersion upcall failed. Please upgrade nfsdcld.\n");

	status = nfsd4_cld_grace_start(nn);
	if (status) {
		if (status == -EOPNOTSUPP)
			pr_warn("NFSD: nfsdcld GraceStart upcall failed. Please upgrade nfsdcld.\n");
		nfs4_release_reclaim(nn);
		goto err_remove;
	} else
		pr_info("NFSD: Using nfsdcld client tracking operations.\n");
	return 0;

err_remove:
	nfsd4_remove_cld_pipe(net);
err_shutdown:
	nfs4_cld_state_shutdown(net);
	return status;
}

static void
nfsd4_cld_tracking_exit(struct net *net)
{
	struct nfsd_net *nn = net_generic(net, nfsd_net_id);

	nfs4_release_reclaim(nn);
	nfsd4_remove_cld_pipe(net);
	nfs4_cld_state_shutdown(net);
}

/* For older nfsdcld's */
static const struct nfsd4_client_tracking_ops nfsd4_cld_tracking_ops_v0 = {
	.init		= nfsd4_init_cld_pipe,
	.exit		= nfsd4_remove_cld_pipe,
	.create		= nfsd4_cld_create,
	.remove		= nfsd4_cld_remove,
	.check		= nfsd4_cld_check_v0,
	.grace_done	= nfsd4_cld_grace_done_v0,
	.version	= 1,
	.msglen		= sizeof(struct cld_msg),
};

/* For newer nfsdcld's */
static const struct nfsd4_client_tracking_ops nfsd4_cld_tracking_ops = {
	.init		= nfsd4_cld_tracking_init,
	.exit		= nfsd4_cld_tracking_exit,
	.create		= nfsd4_cld_create,
	.remove		= nfsd4_cld_remove,
	.check		= nfsd4_cld_check,
	.grace_done	= nfsd4_cld_grace_done,
	.version	= 1,
	.msglen		= sizeof(struct cld_msg),
};

/* v2 create/check ops include the principal, if available */
static const struct nfsd4_client_tracking_ops nfsd4_cld_tracking_ops_v2 = {
	.init		= nfsd4_cld_tracking_init,
	.exit		= nfsd4_cld_tracking_exit,
	.create		= nfsd4_cld_create_v2,
	.remove		= nfsd4_cld_remove,
	.check		= nfsd4_cld_check_v2,
	.grace_done	= nfsd4_cld_grace_done,
	.version	= 2,
	.msglen		= sizeof(struct cld_msg_v2),
};

int
nfsd4_client_tracking_init(struct net *net)
{
	struct nfsd_net *nn = net_generic(net, nfsd_net_id);
	int status;

	/* just run the init if it the method is already decided */
	if (nn->client_tracking_ops)
		goto do_init;

	nn->client_tracking_ops = &nfsd4_cld_tracking_ops;
	status = nn->client_tracking_ops->init(net);
	if (!status)
		return status;
	if (status != -ETIMEDOUT) {
		nn->client_tracking_ops = &nfsd4_cld_tracking_ops_v0;
		status = nn->client_tracking_ops->init(net);
		if (!status)
			return status;
	}
	goto out;
do_init:
	status = nn->client_tracking_ops->init(net);
out:
	if (status) {
		pr_warn("NFSD: Unable to initialize client recovery tracking! (%d)\n", status);
		pr_warn("NFSD: Is nfsdcld running?\n");
		nn->client_tracking_ops = NULL;
	}
	return status;
}

void
nfsd4_client_tracking_exit(struct net *net)
{
	struct nfsd_net *nn = net_generic(net, nfsd_net_id);

	if (nn->client_tracking_ops) {
		nn->client_tracking_ops->exit(net);
		nn->client_tracking_ops = NULL;
	}
}

void
nfsd4_client_record_create(struct nfs4_client *clp)
{
	struct nfsd_net *nn = net_generic(clp->net, nfsd_net_id);

	if (nn->client_tracking_ops)
		nn->client_tracking_ops->create(clp);
}

void
nfsd4_client_record_remove(struct nfs4_client *clp)
{
	struct nfsd_net *nn = net_generic(clp->net, nfsd_net_id);

	if (nn->client_tracking_ops)
		nn->client_tracking_ops->remove(clp);
}

int
nfsd4_client_record_check(struct nfs4_client *clp)
{
	struct nfsd_net *nn = net_generic(clp->net, nfsd_net_id);

	if (nn->client_tracking_ops)
		return nn->client_tracking_ops->check(clp);

	return -EOPNOTSUPP;
}

void
nfsd4_record_grace_done(struct nfsd_net *nn)
{
	if (nn->client_tracking_ops)
		nn->client_tracking_ops->grace_done(nn);
}

static int
rpc_pipefs_event(struct notifier_block *nb, unsigned long event, void *ptr)
{
	struct super_block *sb = ptr;
	struct net *net = sb->s_fs_info;
	struct nfsd_net *nn = net_generic(net, nfsd_net_id);
	struct cld_net *cn = nn->cld_net;
	int ret = 0;

	if (!try_module_get(THIS_MODULE))
		return 0;

	if (!cn) {
		module_put(THIS_MODULE);
		return 0;
	}

	switch (event) {
	case RPC_PIPEFS_MOUNT:
		ret = nfsd4_cld_register_sb(sb, cn->cn_pipe);
		break;
	case RPC_PIPEFS_UMOUNT:
		rpc_unlink(cn->cn_pipe);
		break;
	default:
		ret = -ENOTSUPP;
		break;
	}
	module_put(THIS_MODULE);
	return ret;
}

static struct notifier_block nfsd4_cld_block = {
	.notifier_call = rpc_pipefs_event,
};

int
register_cld_notifier(void)
{
	WARN_ON(!nfsd_net_id);
	return rpc_pipefs_notifier_register(&nfsd4_cld_block);
}

void
unregister_cld_notifier(void)
{
	rpc_pipefs_notifier_unregister(&nfsd4_cld_block);
}
