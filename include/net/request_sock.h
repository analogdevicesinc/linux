/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * NET		Generic infrastructure for Network protocols.
 *
 *		Definitions for request_sock
 *
 * Authors:	Arnaldo Carvalho de Melo <acme@conectiva.com.br>
 *
 * 		From code originally in include/net/tcp.h
 */
#ifndef _REQUEST_SOCK_H
#define _REQUEST_SOCK_H

#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/bug.h>
#include <linux/refcount.h>

#include <net/sock.h>
#include <net/rstreason.h>

struct request_sock;
struct sk_buff;
struct dst_entry;
struct proto;

struct request_sock_ops {
	int		family;
	unsigned int	obj_size;
	struct kmem_cache	*slab;
	char		*slab_name;
	int		(*rtx_syn_ack)(const struct sock *sk,
				       struct request_sock *req);
	void		(*send_ack)(const struct sock *sk, struct sk_buff *skb,
				    struct request_sock *req);
	void		(*send_reset)(const struct sock *sk,
				      struct sk_buff *skb,
				      enum sk_rst_reason reason);
	void		(*destructor)(struct request_sock *req);
	void		(*syn_ack_timeout)(const struct request_sock *req);
};

int inet_rtx_syn_ack(const struct sock *parent, struct request_sock *req);

struct saved_syn {
	u32 mac_hdrlen;
	u32 network_hdrlen;
	u32 tcp_hdrlen;
	u8 data[];
};

/* struct request_sock - mini sock to represent a connection request
 */
struct request_sock {
	struct sock_common		__req_common;
#define rsk_refcnt			__req_common.skc_refcnt
#define rsk_hash			__req_common.skc_hash
#define rsk_listener			__req_common.skc_listener
#define rsk_window_clamp		__req_common.skc_window_clamp
#define rsk_rcv_wnd			__req_common.skc_rcv_wnd

	struct request_sock		*dl_next;
	u16				mss;
	u8				num_retrans; /* number of retransmits */
	u8				syncookie:1; /* True if
						      * 1) tcpopts needs to be encoded in
						      *    TS of SYN+ACK
						      * 2) ACK is validated by BPF kfunc.
						      */
	u8				num_timeout:7; /* number of timeouts */
	u32				ts_recent;
	struct timer_list		rsk_timer;
	const struct request_sock_ops	*rsk_ops;
	struct sock			*sk;
	struct saved_syn		*saved_syn;
	u32				secid;
	u32				peer_secid;
	u32				timeout;
};

static inline struct request_sock *inet_reqsk(const struct sock *sk)
{
	return (struct request_sock *)sk;
}

static inline struct sock *req_to_sk(struct request_sock *req)
{
	return (struct sock *)req;
}

/**
 * skb_steal_sock - steal a socket from an sk_buff
 * @skb: sk_buff to steal the socket from
 * @refcounted: is set to true if the socket is reference-counted
 * @prefetched: is set to true if the socket was assigned from bpf
 */
static inline struct sock *skb_steal_sock(struct sk_buff *skb,
					  bool *refcounted, bool *prefetched)
{
	struct sock *sk = skb->sk;

	if (!sk) {
		*prefetched = false;
		*refcounted = false;
		return NULL;
	}

	*prefetched = skb_sk_is_prefetched(skb);
	if (*prefetched) {
#if IS_ENABLED(CONFIG_SYN_COOKIES)
		if (sk->sk_state == TCP_NEW_SYN_RECV && inet_reqsk(sk)->syncookie) {
			struct request_sock *req = inet_reqsk(sk);

			*refcounted = false;
			sk = req->rsk_listener;
			req->rsk_listener = NULL;
			return sk;
		}
#endif
		*refcounted = sk_is_refcounted(sk);
	} else {
		*refcounted = true;
	}

	skb->destructor = NULL;
	skb->sk = NULL;
	return sk;
}

static inline struct request_sock *
reqsk_alloc_noprof(const struct request_sock_ops *ops, struct sock *sk_listener,
	    bool attach_listener)
{
	struct request_sock *req;

	req = kmem_cache_alloc_noprof(ops->slab, GFP_ATOMIC | __GFP_NOWARN);
	if (!req)
		return NULL;
	req->rsk_listener = NULL;
	if (attach_listener) {
		if (unlikely(!refcount_inc_not_zero(&sk_listener->sk_refcnt))) {
			kmem_cache_free(ops->slab, req);
			return NULL;
		}
		req->rsk_listener = sk_listener;
	}
	req->rsk_ops = ops;
	req_to_sk(req)->sk_prot = sk_listener->sk_prot;
	sk_node_init(&req_to_sk(req)->sk_node);
	sk_tx_queue_clear(req_to_sk(req));
	req->saved_syn = NULL;
	req->syncookie = 0;
	req->timeout = 0;
	req->num_timeout = 0;
	req->num_retrans = 0;
	req->sk = NULL;
	refcount_set(&req->rsk_refcnt, 0);

	return req;
}
#define reqsk_alloc(...)	alloc_hooks(reqsk_alloc_noprof(__VA_ARGS__))

static inline void __reqsk_free(struct request_sock *req)
{
	req->rsk_ops->destructor(req);
	if (req->rsk_listener)
		sock_put(req->rsk_listener);
	kfree(req->saved_syn);
	kmem_cache_free(req->rsk_ops->slab, req);
}

static inline void reqsk_free(struct request_sock *req)
{
	WARN_ON_ONCE(refcount_read(&req->rsk_refcnt) != 0);
	__reqsk_free(req);
}

static inline void reqsk_put(struct request_sock *req)
{
	if (refcount_dec_and_test(&req->rsk_refcnt))
		reqsk_free(req);
}

/*
 * For a TCP Fast Open listener -
 *	lock - protects the access to all the reqsk, which is co-owned by
 *		the listener and the child socket.
 *	qlen - pending TFO requests (still in TCP_SYN_RECV).
 *	max_qlen - max TFO reqs allowed before TFO is disabled.
 *
 *	XXX (TFO) - ideally these fields can be made as part of "listen_sock"
 *	structure above. But there is some implementation difficulty due to
 *	listen_sock being part of request_sock_queue hence will be freed when
 *	a listener is stopped. But TFO related fields may continue to be
 *	accessed even after a listener is closed, until its sk_refcnt drops
 *	to 0 implying no more outstanding TFO reqs. One solution is to keep
 *	listen_opt around until	sk_refcnt drops to 0. But there is some other
 *	complexity that needs to be resolved. E.g., a listener can be disabled
 *	temporarily through shutdown()->tcp_disconnect(), and re-enabled later.
 */
struct fastopen_queue {
	struct request_sock	*rskq_rst_head; /* Keep track of past TFO */
	struct request_sock	*rskq_rst_tail; /* requests that caused RST.
						 * This is part of the defense
						 * against spoofing attack.
						 */
	spinlock_t	lock;
	int		qlen;		/* # of pending (TCP_SYN_RECV) reqs */
	int		max_qlen;	/* != 0 iff TFO is currently enabled */

	struct tcp_fastopen_context __rcu *ctx; /* cipher context for cookie */
};

/** struct request_sock_queue - queue of request_socks
 *
 * @rskq_accept_head - FIFO head of established children
 * @rskq_accept_tail - FIFO tail of established children
 * @rskq_defer_accept - User waits for some data after accept()
 *
 */
struct request_sock_queue {
	spinlock_t		rskq_lock;
	u8			rskq_defer_accept;

	u32			synflood_warned;
	atomic_t		qlen;
	atomic_t		young;

	struct request_sock	*rskq_accept_head;
	struct request_sock	*rskq_accept_tail;
	struct fastopen_queue	fastopenq;  /* Check max_qlen != 0 to determine
					     * if TFO is enabled.
					     */
};

void reqsk_queue_alloc(struct request_sock_queue *queue);

void reqsk_fastopen_remove(struct sock *sk, struct request_sock *req,
			   bool reset);

static inline bool reqsk_queue_empty(const struct request_sock_queue *queue)
{
	return READ_ONCE(queue->rskq_accept_head) == NULL;
}

static inline struct request_sock *reqsk_queue_remove(struct request_sock_queue *queue,
						      struct sock *parent)
{
	struct request_sock *req;

	spin_lock_bh(&queue->rskq_lock);
	req = queue->rskq_accept_head;
	if (req) {
		sk_acceptq_removed(parent);
		WRITE_ONCE(queue->rskq_accept_head, req->dl_next);
		if (queue->rskq_accept_head == NULL)
			queue->rskq_accept_tail = NULL;
	}
	spin_unlock_bh(&queue->rskq_lock);
	return req;
}

static inline void reqsk_queue_removed(struct request_sock_queue *queue,
				       const struct request_sock *req)
{
	if (req->num_timeout == 0)
		atomic_dec(&queue->young);
	atomic_dec(&queue->qlen);
}

static inline void reqsk_queue_added(struct request_sock_queue *queue)
{
	atomic_inc(&queue->young);
	atomic_inc(&queue->qlen);
}

static inline int reqsk_queue_len(const struct request_sock_queue *queue)
{
	return atomic_read(&queue->qlen);
}

static inline int reqsk_queue_len_young(const struct request_sock_queue *queue)
{
	return atomic_read(&queue->young);
}

/* RFC 7323 2.3 Using the Window Scale Option
 *  The window field (SEG.WND) of every outgoing segment, with the
 *  exception of <SYN> segments, MUST be right-shifted by
 *  Rcv.Wind.Shift bits.
 *
 * This means the SEG.WND carried in SYNACK can not exceed 65535.
 * We use this property to harden TCP stack while in NEW_SYN_RECV state.
 */
static inline u32 tcp_synack_window(const struct request_sock *req)
{
	return min(req->rsk_rcv_wnd, 65535U);
}
#endif /* _REQUEST_SOCK_H */
