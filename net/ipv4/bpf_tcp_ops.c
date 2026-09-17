// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2026 Meta Platforms, Inc. and affiliates. */

#include <linux/bpf.h>
#include <linux/btf_ids.h>
#include <linux/bpf_verifier.h>
#include <net/bpf_sk_storage.h>
#include <net/tcp.h>

static int timeout_init_stub(struct sock *sk, struct request_sock *req__nullable)
{
	struct bpf_tramp_run_ctx *ctx =
		container_of(current->bpf_ctx, struct bpf_tramp_run_ctx, run_ctx);

	return ctx->retval;
}

static int rwnd_init_stub(struct sock *sk, struct request_sock *req__nullable)
{
	struct bpf_tramp_run_ctx *ctx =
		container_of(current->bpf_ctx, struct bpf_tramp_run_ctx, run_ctx);

	return ctx->retval;
}

static void active_established_stub(struct sock *sk, struct sk_buff *skb__nullable)
{
}

static void passive_established_stub(struct sock *sk, struct sk_buff *skb)
{
}

static void rto_stub(struct sock *sk)
{
}

static void rtt_stub(struct sock *sk, long mrtt, u32 srtt)
{
}

static void set_state_stub(struct sock *sk, int state)
{
}

static void retrans_stub(struct sock *sk, struct sk_buff *skb, int err)
{
}

static void connect_stub(struct sock *sk)
{
}

static void listen_stub(struct sock *sk)
{
}

static struct bpf_tcp_ops __bpf_tcp_ops = {
	.timeout_init = timeout_init_stub,
	.rwnd_init = rwnd_init_stub,
	.active_established = active_established_stub,
	.passive_established = passive_established_stub,
	.rto = rto_stub,
	.rtt = rtt_stub,
	.set_state = set_state_stub,
	.retrans = retrans_stub,
	.connect = connect_stub,
	.listen = listen_stub,
};

BPF_CALL_0(bpf_tcp_ops_get_retval)
{
	struct bpf_tramp_run_ctx *ctx =
		container_of(current->bpf_ctx, struct bpf_tramp_run_ctx, run_ctx);

	/* bpf_get_retval() is only exposed to timeout_init/rwnd_init, which
	 * always run via bpf_tcp_ops_call_int(). Its run_ctx carries the int
	 * return value chained across the bpf_tcp_ops attached to the cgroup
	 * and is this program's saved_run_ctx.
	 */
	if (WARN_ON_ONCE(!ctx->saved_run_ctx))
		return 0;

	return container_of(ctx->saved_run_ctx, struct bpf_tramp_run_ctx,
			    run_ctx)->retval;
}

const struct bpf_func_proto bpf_tcp_ops_get_retval_proto = {
	.func		= bpf_tcp_ops_get_retval,
	.gpl_only	= false,
	.ret_type	= RET_INTEGER,
};

static const struct bpf_func_proto *
get_func_proto(enum bpf_func_id func_id, const struct bpf_prog *prog)
{
	u32 moff = prog->aux->attach_st_ops_member_off;

	switch (func_id) {
	case BPF_FUNC_sk_storage_get:
		return &bpf_sk_storage_get_proto;
	case BPF_FUNC_sk_storage_delete:
		return &bpf_sk_storage_delete_proto;
	case BPF_FUNC_setsockopt:
		/* The listener is not locked. */
		if (moff == offsetof(struct bpf_tcp_ops, rwnd_init) ||
		    moff == offsetof(struct bpf_tcp_ops, timeout_init))
			return NULL;
		return &bpf_sk_setsockopt_proto;
	case BPF_FUNC_getsockopt:
		if (moff == offsetof(struct bpf_tcp_ops, rwnd_init) ||
		    moff == offsetof(struct bpf_tcp_ops, timeout_init))
			return NULL;
		return &bpf_sk_getsockopt_proto;
	case BPF_FUNC_get_retval:
		if (moff == offsetof(struct bpf_tcp_ops, timeout_init) ||
		    moff == offsetof(struct bpf_tcp_ops, rwnd_init))
			return &bpf_tcp_ops_get_retval_proto;
		return NULL;
	default:
		return bpf_base_func_proto(func_id, prog);
	}
}

static bool is_valid_access(int off, int size, enum bpf_access_type type,
			    const struct bpf_prog *prog, struct bpf_insn_access_aux *info)
{
	if (!bpf_tracing_btf_ctx_access(off, size, type, prog, info))
		return false;

	if (base_type(info->reg_type) == PTR_TO_BTF_ID &&
	    !bpf_type_has_unsafe_modifiers(info->reg_type) &&
	    info->btf_id == btf_sock_ids[BTF_SOCK_TYPE_SOCK])
		/* promote it to tcp_sock */
		info->btf_id = btf_sock_ids[BTF_SOCK_TYPE_TCP];

	return true;
}

static int bpf_tcp_ops_init_member(const struct btf_type *t,
				   const struct btf_member *member,
				   void *kdata, const void *udata)
{
	return 0;
}

static int bpf_tcp_ops_check_member(const struct btf_type *t,
				    const struct btf_member *member,
				    const struct bpf_prog *prog)
{
	if (prog->sleepable)
		return -EINVAL;

	return 0;
}

static int bpf_tcp_ops_init(struct btf *btf)
{
	return 0;
}

static int bpf_tcp_ops_validate(void *kdata)
{
	return 0;
}

static const struct bpf_verifier_ops bpf_tcp_ops_verifier = {
	.get_func_proto		= get_func_proto,
	.is_valid_access	= is_valid_access,
};

static struct bpf_struct_ops bpf_tcp_ops = {
	.verifier_ops = &bpf_tcp_ops_verifier,
	.init_member = bpf_tcp_ops_init_member,
	.check_member = bpf_tcp_ops_check_member,
	.init = bpf_tcp_ops_init,
	.validate = bpf_tcp_ops_validate,
	.name = "bpf_tcp_ops",
	.cgroup_atype = CGROUP_TCP_SOCK_OPS,
	.cfi_stubs = &__bpf_tcp_ops,
	.owner = THIS_MODULE,
};

static int __init __bpf_tcp_ops_init(void)
{
	return register_bpf_struct_ops(&bpf_tcp_ops, bpf_tcp_ops);
}
late_initcall(__bpf_tcp_ops_init);
