/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Hodge-podge collection of knfsd-related stuff.
 * I will sort this out later.
 *
 * Copyright (C) 1995-1997 Olaf Kirch <okir@monad.swb.de>
 */

#ifndef LINUX_NFSD_NFSD_H
#define LINUX_NFSD_NFSD_H

#include <linux/types.h>
#include <linux/mount.h>

#include <linux/nfs.h>
#include <linux/nfs2.h>
#include <linux/nfs3.h>
#include <linux/sunrpc/svc.h>
#include <linux/sunrpc/svc_xprt.h>

#include <uapi/linux/nfsd/debug.h>

#undef ifdebug
#ifdef CONFIG_SUNRPC_DEBUG
# define ifdebug(flag)		if (nfsd_debug & NFSDDBG_##flag)
#else
# define ifdebug(flag)		if (0)
#endif

/*
 * nfsd version
 */
#define NFSD_MINVERS			2
#define	NFSD_MAXVERS			4
#define NFSD_SUPPORTED_MINOR_VERSION	2
bool nfsd_support_version(int vers);

/*
 * Default and maximum payload size (NFS READ or WRITE), in bytes.
 * The maximum is an implementation limit.
 */
enum {
	NFSSVC_DEFBLKSIZE       = 4 * 1024 * 1024,
	NFSSVC_MAXBLKSIZE       = RPCSVC_MAXPAYLOAD,
};

/* Maximum number of operations per session compound */
#define NFSD_MAX_OPS_PER_COMPOUND	200

extern struct svc_program	nfsd_programs[];
extern const struct svc_version	nfsd_version2, nfsd_version3, nfsd_version4;
extern struct mutex		nfsd_mutex;
extern atomic_t			nfsd_th_cnt;		/* number of available threads */

extern const struct seq_operations nfs_exports_op;

struct nfsd_thread_local_info {
	struct nfs4_client	**ntli_lease_breaker;
	int			ntli_cachetype;
};

/*
 * Common void argument and result helpers
 */
struct nfsd_voidargs { };
struct nfsd_voidres { };
bool		nfssvc_decode_voidarg(struct svc_rqst *rqstp,
				      struct xdr_stream *xdr);
bool		nfssvc_encode_voidres(struct svc_rqst *rqstp,
				      struct xdr_stream *xdr);

/*
 * Function prototypes.
 */
int		nfsd_svc(int n, int *nservers, struct net *net,
			 const struct cred *cred, const char *scope);
int		nfsd_dispatch(struct svc_rqst *rqstp);

int		nfsd_nrthreads(struct net *);
int		nfsd_nrpools(struct net *);
int		nfsd_get_nrthreads(int n, int *, struct net *);
int		nfsd_set_nrthreads(int n, int *, struct net *);
void		nfsd_shutdown_threads(struct net *net);

struct svc_rqst *nfsd_current_rqst(void);

struct nfsdfs_client {
	struct kref cl_ref;
	void (*cl_release)(struct kref *kref);
};

struct nfsd_net;

struct nfsdfs_client *get_nfsdfs_client(struct inode *);
struct dentry *nfsd_client_mkdir(struct nfsd_net *nn,
				 struct nfsdfs_client *ncl, u32 id,
				 const struct tree_descr *,
				 struct dentry **fdentries);
void nfsd_client_rmdir(struct dentry *dentry);
int nfsd_cache_notify(struct cache_detail *cd, struct cache_head *h, u32 cache_type);

#if defined(CONFIG_NFSD_V2_ACL) || defined(CONFIG_NFSD_V3_ACL)
#ifdef CONFIG_NFSD_V2_ACL
extern const struct svc_version nfsd_acl_version2;
#else
#define nfsd_acl_version2 NULL
#endif
#ifdef CONFIG_NFSD_V3_ACL
extern const struct svc_version nfsd_acl_version3;
#else
#define nfsd_acl_version3 NULL
#endif
#endif

#if IS_ENABLED(CONFIG_NFS_LOCALIO)
extern const struct svc_version localio_version1;
#endif

enum vers_op {NFSD_SET, NFSD_CLEAR, NFSD_TEST, NFSD_AVAIL };
int nfsd_vers(struct nfsd_net *nn, int vers, enum vers_op change);
int nfsd_minorversion(struct nfsd_net *nn, u32 minorversion, enum vers_op change);
void nfsd_reset_versions(struct nfsd_net *nn);
int nfsd_create_serv(struct net *net);
void nfsd_destroy_serv(struct net *net);

#ifdef CONFIG_DEBUG_FS
void nfsd_debugfs_init(void);
void nfsd_debugfs_exit(void);
#else
static inline void nfsd_debugfs_init(void) {}
static inline void nfsd_debugfs_exit(void) {}
#endif

extern bool nfsd_disable_splice_read __read_mostly;
extern bool nfsd_delegts_enabled __read_mostly;

enum {
	/* Any new NFSD_IO enum value must be added at the end */
	NFSD_IO_BUFFERED,
	NFSD_IO_DONTCACHE,
	NFSD_IO_DIRECT,
};

extern u64 nfsd_io_cache_read __read_mostly;
extern u64 nfsd_io_cache_write __read_mostly;

extern int nfsd_max_blksize;

static inline int nfsd_v4client(struct svc_rqst *rq)
{
	return rq && rq->rq_prog == NFS_PROGRAM && rq->rq_vers == 4;
}

/* 
 * NFSv4 State
 */
#ifdef CONFIG_NFSD_V4
extern unsigned long max_delegations;
int nfsd4_init_slabs(void);
void nfsd4_free_slabs(void);
int nfs4_state_start(void);
int nfs4_state_start_net(struct net *net);
void nfs4_state_shutdown(void);
void nfs4_state_shutdown_net(struct net *net);
int nfs4_reset_recoverydir(char *recdir);
char * nfs4_recoverydir(void);
bool nfsd4_spo_must_allow(struct svc_rqst *rqstp);
int nfsd4_create_laundry_wq(void);
void nfsd4_destroy_laundry_wq(void);
bool nfsd_wait_for_delegreturn(struct svc_rqst *rqstp, struct inode *inode);
#else
static inline int nfsd4_init_slabs(void) { return 0; }
static inline void nfsd4_free_slabs(void) { }
static inline int nfs4_state_start(void) { return 0; }
static inline int nfs4_state_start_net(struct net *net) { return 0; }
static inline void nfs4_state_shutdown(void) { }
static inline void nfs4_state_shutdown_net(struct net *net) { }
static inline int nfs4_reset_recoverydir(char *recdir) { return 0; }
static inline char * nfs4_recoverydir(void) {return NULL; }
static inline bool nfsd4_spo_must_allow(struct svc_rqst *rqstp)
{
	return false;
}
static inline int nfsd4_create_laundry_wq(void) { return 0; };
static inline void nfsd4_destroy_laundry_wq(void) {};
static inline bool nfsd_wait_for_delegreturn(struct svc_rqst *rqstp,
					      struct inode *inode)
{
	return false;
}
#endif

/*
 * lockd binding
 */
void		nfsd_lockd_init(void);
void		nfsd_lockd_shutdown(void);


#ifdef CONFIG_NFSD_V4

/* before processing a COMPOUND operation, we have to check that there
 * is enough space in the buffer for XDR encode to succeed.  otherwise,
 * we might process an operation with side effects, and be unable to
 * tell the client that the operation succeeded.
 *
 * COMPOUND_SLACK_SPACE - this is the minimum bytes of buffer space
 * needed to encode an "ordinary" _successful_ operation.  (GETATTR,
 * READ, READDIR, and READLINK have their own buffer checks.)  if we
 * fall below this level, we fail the next operation with NFS4ERR_RESOURCE.
 *
 * COMPOUND_ERR_SLACK_SPACE - this is the minimum bytes of buffer space
 * needed to encode an operation which has failed with NFS4ERR_RESOURCE.
 * care is taken to ensure that we never fall below this level for any
 * reason.
 */
#define	COMPOUND_SLACK_SPACE		140    /* OP_GETFH */
#define COMPOUND_ERR_SLACK_SPACE	16     /* OP_SETATTR */

#define NFSD_LAUNDROMAT_MINTIMEOUT      1   /* seconds */
#define	NFSD_COURTESY_CLIENT_TIMEOUT	(24 * 60 * 60)	/* seconds */
#define	NFSD_CLIENT_MAX_TRIM_PER_RUN	128
#define	NFS4_CLIENTS_PER_GB		1024
#define NFSD_DELEGRETURN_TIMEOUT	(HZ / 34)	/* 30ms */
#define	NFSD_CB_GETATTR_TIMEOUT		NFSD_DELEGRETURN_TIMEOUT

extern int nfsd4_is_junction(struct dentry *dentry);
extern int register_cld_notifier(void);
extern void unregister_cld_notifier(void);
#ifdef CONFIG_NFSD_V4_2_INTER_SSC
extern void nfsd4_ssc_init_umount_work(struct nfsd_net *nn);
#endif

extern void nfsd4_init_leases_net(struct nfsd_net *nn);

#else /* CONFIG_NFSD_V4 */
static inline int nfsd4_is_junction(struct dentry *dentry)
{
	return 0;
}

static inline void nfsd4_init_leases_net(struct nfsd_net *nn) { };

#define register_cld_notifier() 0
#define unregister_cld_notifier() do { } while(0)

#endif /* CONFIG_NFSD_V4 */

#endif /* LINUX_NFSD_NFSD_H */
