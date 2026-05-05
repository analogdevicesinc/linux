/*
 * This file was extracted by hand from
 * https://www.rfc-editor.org/rfc/rfc1813.html .
 *
 * Note that RFC 1813 is Informational. Its official date of
 * publication (June 1995) is before the IETF required its RFCs to
 * carry an explicit copyright or other IP ownership notices.
 *
 * Note also that RFC 1813 does not specify the whole NLM4 protocol.
 * In particular, the argument and result types are not present in
 * that document, and had to be reverse-engineered.
 */

/*
 * The NLMv4 protocol
 */

pragma header nlm4;

/*
 * The following definitions are missing in RFC 1813,
 * but can be found in the OpenNetworking Network Lock
 * Manager protocol:
 *
 * https://pubs.opengroup.org/onlinepubs/9629799/chap10.htm
 */

const LM_MAXSTRLEN = 1024;

const LM_MAXNAMELEN = 1025;

const MAXNETOBJ_SZ = 1024;

typedef opaque netobj<MAXNETOBJ_SZ>;

enum fsh4_mode {
	fsm_DN = 0,		/* deny none */
	fsm_DR = 1,		/* deny read */
	fsm_DW = 2,		/* deny write */
	fsm_DRW = 3		/* deny read/write */
};

enum fsh4_access {
	fsa_NONE = 0,		/* for completeness */
	fsa_R = 1,		/* read-only */
	fsa_W = 2,		/* write-only */
	fsa_RW = 3		/* read/write */
};

/*
 * The following definitions come from the OpenNetworking
 * Network Status Monitor protocol:
 *
 * https://pubs.opengroup.org/onlinepubs/9629799/chap11.htm
 */

const SM_MAXSTRLEN = 1024;

/*
 * The NLM protocol as extracted from:
 * https://tools.ietf.org/html/rfc1813 Appendix II
 */

typedef unsigned hyper uint64;

typedef hyper int64;

typedef unsigned long uint32;

typedef long int32;

enum nlm4_stats {
	NLM4_GRANTED = 0,
	NLM4_DENIED = 1,
	NLM4_DENIED_NOLOCKS = 2,
	NLM4_BLOCKED = 3,
	NLM4_DENIED_GRACE_PERIOD = 4,
	NLM4_DEADLCK = 5,
	NLM4_ROFS = 6,
	NLM4_STALE_FH = 7,
	NLM4_FBIG = 8,
	NLM4_FAILED = 9
};

pragma big_endian nlm4_stats;

struct nlm4_holder {
	bool		exclusive;
	int32		svid;
	netobj		oh;
	uint64		l_offset;
	uint64		l_len;
};

union nlm4_testrply switch (nlm4_stats stat) {
	case NLM4_DENIED:
		nlm4_holder	holder;
	default:
		void;
};

struct nlm4_stat {
	nlm4_stats	stat;
};

struct nlm4_res {
	netobj		cookie;
	nlm4_stat	stat;
};

struct nlm4_testres {
	netobj		cookie;
	nlm4_testrply	stat;
};

struct nlm4_lock {
	string		caller_name<LM_MAXSTRLEN>;
	netobj		fh;
	netobj		oh;
	int32		svid;
	uint64		l_offset;
	uint64		l_len;
};

struct nlm4_lockargs {
	netobj		cookie;
	bool		block;
	bool		exclusive;
	nlm4_lock	alock;
	bool		reclaim;
	int32		state;
};

struct nlm4_cancargs {
	netobj		cookie;
	bool		block;
	bool		exclusive;
	nlm4_lock	alock;
};

struct nlm4_testargs {
	netobj		cookie;
	bool		exclusive;
	nlm4_lock	alock;
};

struct nlm4_unlockargs {
	netobj		cookie;
	nlm4_lock	alock;
};

struct nlm4_share {
	string		caller_name<LM_MAXSTRLEN>;
	netobj		fh;
	netobj		oh;
	fsh4_mode	mode;
	fsh4_access	access;
};

struct nlm4_shareargs {
	netobj		cookie;
	nlm4_share	share;
	bool		reclaim;
};

struct nlm4_shareres {
	netobj		cookie;
	nlm4_stats	stat;
	int32		sequence;
};

struct nlm4_notify {
	string		name<LM_MAXNAMELEN>;
	int32		state;
};

/*
 * Argument for the Linux-private SM_NOTIFY procedure
 */
const SM_PRIV_SIZE = 16;

struct nlm4_notifyargs {
	nlm4_notify	notify;
	opaque		private[SM_PRIV_SIZE];
};

program NLM4_PROG {
	version NLM4_VERS {
		void		NLMPROC4_NULL(void)			= 0;
		nlm4_testres	NLMPROC4_TEST(nlm4_testargs)		= 1;
		nlm4_res	NLMPROC4_LOCK(nlm4_lockargs)		= 2;
		nlm4_res	NLMPROC4_CANCEL(nlm4_cancargs)		= 3;
		nlm4_res	NLMPROC4_UNLOCK(nlm4_unlockargs)	= 4;
		nlm4_res	NLMPROC4_GRANTED(nlm4_testargs)		= 5;
		void		NLMPROC4_TEST_MSG(nlm4_testargs)	= 6;
		void		NLMPROC4_LOCK_MSG(nlm4_lockargs)	= 7;
		void		NLMPROC4_CANCEL_MSG(nlm4_cancargs)	= 8;
		void		NLMPROC4_UNLOCK_MSG(nlm4_unlockargs)	= 9;
		void		NLMPROC4_GRANTED_MSG(nlm4_testargs)	= 10;
		void		NLMPROC4_TEST_RES(nlm4_testres)		= 11;
		void		NLMPROC4_LOCK_RES(nlm4_res)		= 12;
		void		NLMPROC4_CANCEL_RES(nlm4_res)		= 13;
		void		NLMPROC4_UNLOCK_RES(nlm4_res)		= 14;
		void		NLMPROC4_GRANTED_RES(nlm4_res)		= 15;
		void		NLMPROC4_SM_NOTIFY(nlm4_notifyargs)	= 16;
		nlm4_shareres	NLMPROC4_SHARE(nlm4_shareargs)		= 20;
		nlm4_shareres	NLMPROC4_UNSHARE(nlm4_shareargs)	= 21;
		nlm4_res	NLMPROC4_NM_LOCK(nlm4_lockargs)		= 22;
		void		NLMPROC4_FREE_ALL(nlm4_notify)		= 23;
	} = 4;
} = 100021;
