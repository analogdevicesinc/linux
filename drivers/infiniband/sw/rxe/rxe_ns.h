/* SPDX-License-Identifier: GPL-2.0 OR Linux-OpenIB */

#ifndef RXE_NS_H
#define RXE_NS_H

struct sock *rxe_ns_pernet_sk4(struct net *net);
void rxe_ns_pernet_set_sk4(struct net *net, struct sock *sk);

#if IS_ENABLED(CONFIG_IPV6)
void rxe_ns_pernet_set_sk6(struct net *net, struct sock *sk);
struct sock *rxe_ns_pernet_sk6(struct net *net);
#else /* IPv6 */
static inline struct sock *rxe_ns_pernet_sk6(struct net *net)
{
	return NULL;
}

static inline void rxe_ns_pernet_set_sk6(struct net *net, struct sock *sk)
{
}
#endif /* IPv6 */

int rxe_namespace_init(void);
void rxe_namespace_exit(void);

#endif /* RXE_NS_H */
