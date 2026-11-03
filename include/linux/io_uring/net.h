/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef _LINUX_IO_URING_NET_H
#define _LINUX_IO_URING_NET_H

#include <linux/errno.h>
#include <linux/types.h>
#include <net/net_debug.h>

struct io_uring_cmd;
struct sk_buff;
struct ubuf_info;

#if defined(CONFIG_IO_URING)
int io_uring_cmd_sock(struct io_uring_cmd *cmd, unsigned int issue_flags);
void io_tx_ubuf_complete(struct sk_buff *skb, struct ubuf_info *uarg,
			 bool success);
int io_link_skb(struct sk_buff *skb, struct ubuf_info *uarg);

#else
static inline int io_uring_cmd_sock(struct io_uring_cmd *cmd,
				    unsigned int issue_flags)
{
	return -EOPNOTSUPP;
}

static inline void io_tx_ubuf_complete(struct sk_buff *skb,
				       struct ubuf_info *uarg, bool success)
{
	DEBUG_NET_WARN_ON_ONCE(1);
}

static inline int io_link_skb(struct sk_buff *skb, struct ubuf_info *uarg)
{
	return -EOPNOTSUPP;
}
#endif

#endif
