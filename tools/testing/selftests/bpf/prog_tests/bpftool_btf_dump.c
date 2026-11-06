// SPDX-License-Identifier: GPL-2.0
#include <test_progs.h>
#include <bpftool_helpers.h>
#include <bpf/btf.h>
#include <limits.h>
#include <unistd.h>
#include "btf_helpers.h"
#include "testing_helpers.h"

#define DUMP_BUF_SZ		(16 * 1024)

#define EXPECTED_SORTED		"bpftool_btf_dump_sorted.expected"
#define EXPECTED_UNSORTED	"bpftool_btf_dump_unsorted.expected"

/*
 *	struct holey {
 *		int c;
 *		<64-bit hole>
 *		int tail;
 *	};
 *	enum { E0 = 1 };
 *	struct s { int f; };
 *	struct __sk_buff {
 *		int len;
 *		union { int sk; };
 *	};
 *	struct bpf_sk_lookup {
 *		union {
 *			union { int cookie; };
 *			int family;
 *		};
 *	};
 *	struct bpf_tcp_sock { int snd_cwnd; };
 *	struct bpf_nf_ctx { int f; };
 *	typedef int __arena *arena_ptr;
 */
static struct btf *mk_btf(void)
{
	struct btf *btf;

	btf = btf__new_empty();
	if (!ASSERT_OK_PTR(btf, "new_empty"))
		return NULL;

	btf__add_int(btf, "int", 4, BTF_INT_SIGNED);
	btf__add_int(btf, "long int", 4, BTF_INT_SIGNED);

	btf__add_struct(btf, "holey", 16);
	btf__add_field(btf, "c", 1, 0, 0);
	btf__add_field(btf, "tail", 1, 96, 0);

	btf__add_enum(btf, NULL, 4);
	btf__add_enum_value(btf, "E0", 1);

	btf__add_struct(btf, "s", 4);
	btf__add_field(btf, "f", 1, 0, 0);

	btf__add_union(btf, NULL, 4);
	btf__add_field(btf, "sk", 1, 0, 0);
	btf__add_struct(btf, "__sk_buff", 8);
	btf__add_field(btf, "len", 1, 0, 0);
	btf__add_field(btf, NULL, 6, 32, 0);

	btf__add_union(btf, NULL, 4);
	btf__add_field(btf, "cookie", 1, 0, 0);
	btf__add_union(btf, NULL, 4);
	btf__add_field(btf, NULL, 8, 0, 0);
	btf__add_field(btf, "family", 1, 0, 0);
	btf__add_struct(btf, "bpf_sk_lookup", 4);
	btf__add_field(btf, NULL, 9, 0, 0);

	btf__add_struct(btf, "bpf_tcp_sock", 4);
	btf__add_field(btf, "snd_cwnd", 1, 0, 0);
	btf__add_struct(btf, "bpf_nf_ctx", 4);
	btf__add_field(btf, "f", 1, 0, 0);

	btf__add_type_tag(btf, "arena", 1);
	btf__add_ptr(btf, 13);
	btf__add_typedef(btf, "arena_ptr", 14);

	VALIDATE_RAW_BTF(
		btf,
		"[1] INT 'int' size=4 bits_offset=0 nr_bits=32 encoding=SIGNED",
		"[2] INT 'long int' size=4 bits_offset=0 nr_bits=32 encoding=SIGNED",
		"[3] STRUCT 'holey' size=16 vlen=2\n"
		"\t'c' type_id=1 bits_offset=0\n"
		"\t'tail' type_id=1 bits_offset=96",
		"[4] ENUM '(anon)' encoding=UNSIGNED size=4 vlen=1\n"
		"\t'E0' val=1",
		"[5] STRUCT 's' size=4 vlen=1\n"
		"\t'f' type_id=1 bits_offset=0",
		"[6] UNION '(anon)' size=4 vlen=1\n"
		"\t'sk' type_id=1 bits_offset=0",
		"[7] STRUCT '__sk_buff' size=8 vlen=2\n"
		"\t'len' type_id=1 bits_offset=0\n"
		"\t'(anon)' type_id=6 bits_offset=32",
		"[8] UNION '(anon)' size=4 vlen=1\n"
		"\t'cookie' type_id=1 bits_offset=0",
		"[9] UNION '(anon)' size=4 vlen=2\n"
		"\t'(anon)' type_id=8 bits_offset=0\n"
		"\t'family' type_id=1 bits_offset=0",
		"[10] STRUCT 'bpf_sk_lookup' size=4 vlen=1\n"
		"\t'(anon)' type_id=9 bits_offset=0",
		"[11] STRUCT 'bpf_tcp_sock' size=4 vlen=1\n"
		"\t'snd_cwnd' type_id=1 bits_offset=0",
		"[12] STRUCT 'bpf_nf_ctx' size=4 vlen=1\n"
		"\t'f' type_id=1 bits_offset=0",
		"[13] TYPE_TAG 'arena' type_id=1",
		"[14] PTR '(anon)' type_id=13",
		"[15] TYPEDEF 'arena_ptr' type_id=14");

	return btf;
}

/* mk_btf() with a nonzero byte in an unknown header extension */
static struct btf *mk_hdr_extra_btf(void)
{
	struct btf *btf, *ext_btf = NULL;
	const __u32 extra_sz = 4;
	__u32 hdr_len, sz;
	const void *raw;
	char *ext;

	btf = mk_btf();
	if (!btf)
		return NULL;

	raw = btf__raw_data(btf, &sz);
	if (!ASSERT_OK_PTR(raw, "raw_data"))
		goto out;

	hdr_len = ((const struct btf_header *)raw)->hdr_len;

	ext = calloc(1, sz + extra_sz);
	if (!ASSERT_OK_PTR(ext, "alloc_ext"))
		goto out;

	memcpy(ext, raw, hdr_len);
	ext[hdr_len] = 0xA5; /* any nonzero value */
	memcpy(ext + hdr_len + extra_sz, raw + hdr_len, sz - hdr_len);
	((struct btf_header *)ext)->hdr_len = hdr_len + extra_sz;

	ext_btf = btf__new(ext, sz + extra_sz);
	ASSERT_OK_PTR(ext_btf, "btf_new_ext");
	free(ext);
out:
	btf__free(btf);
	return ext_btf;
}

/*
 * Older kernel BTF encodes the arena attribute as "address_space(1)":
 *
 *	int __attribute__((address_space(1))) *arena(void);
 */
static struct btf *mk_legacy_arena_btf(void)
{
	struct btf *btf;

	btf = btf__new_empty();
	if (!ASSERT_OK_PTR(btf, "new_empty"))
		return NULL;

	btf__add_int(btf, "int", 4, BTF_INT_SIGNED);
	btf__add_type_attr(btf, "address_space(1)", 1);
	btf__add_ptr(btf, 2);
	btf__add_func_proto(btf, 3);
	btf__add_func(btf, "arena", BTF_FUNC_GLOBAL, 4);
	btf__add_decl_tag(btf, "bpf_kfunc", 5, -1);

	VALIDATE_RAW_BTF(
		btf,
		"[1] INT 'int' size=4 bits_offset=0 nr_bits=32 encoding=SIGNED",
		"[2] TYPE_TAG 'address_space(1)' type_id=1 kflag=1",
		"[3] PTR '(anon)' type_id=2",
		"[4] FUNC_PROTO '(anon)' ret_type_id=3 vlen=0",
		"[5] FUNC 'arena' type_id=4 linkage=global",
		"[6] DECL_TAG 'bpf_kfunc' type_id=5 component_idx=-1");

	return btf;
}

static struct btf *legacy_arena_base;

/* Module BTF over legacy_arena_base, with an arena kfunc of its own */
static struct btf *mk_legacy_arena_split_btf(void)
{
	struct btf *btf;

	btf = btf__new_empty_split(legacy_arena_base);
	if (!ASSERT_OK_PTR(btf, "new_empty_split"))
		return NULL;

	btf__add_type_attr(btf, "address_space(1)", 1);
	btf__add_ptr(btf, 7);
	btf__add_func_proto(btf, 8);
	btf__add_func(btf, "arena_mod", BTF_FUNC_GLOBAL, 9);
	btf__add_decl_tag(btf, "bpf_kfunc", 10, -1);

	return btf;
}

static struct btf *mk_loc_btf(void)
{
	struct btf *btf;

	btf = btf__new_empty();
	if (!ASSERT_OK_PTR(btf, "new_empty"))
		return NULL;

	btf__add_int(btf, "int", 4, BTF_INT_SIGNED);
	btf__add_func_proto(btf, 1);
	btf__add_func_param(btf, "arg1", 1);
	btf__add_func_param(btf, "arg2", 1);
	btf__add_func_param(btf, "arg3", 1);
	btf__add_func_param(btf, "arg4", 1);
	btf__add_func_param(btf, "arg5", 1);
	btf__add_func_param(btf, "arg6", 1);
	btf__add_func(btf, "foo", BTF_FUNC_STATIC, 2);

	btf__add_loc_param(btf, 4, BTF_LOC_PARAM_REG);
	btf__add_loc_param_value(btf, 1);
	btf__add_loc_param(btf, 8, BTF_LOC_PARAM_REG |
			    BTF_LOC_PARAM_DEREF | BTF_LOC_PARAM_OFFSET);
	btf__add_loc_param_value(btf, 2);
	btf__add_loc_param_value(btf, 0x10);
	btf__add_loc_param(btf, 8, BTF_LOC_PARAM_REG |
			   BTF_LOC_PARAM_OFFSET | BTF_LOC_PARAM_SIGNED);
	btf__add_loc_param_value(btf, BTF_LOC_PARAM_FBREG);
	btf__add_loc_param_value(btf, -0x10);
	btf__add_loc_param(btf, 8, BTF_LOC_PARAM_ADDR | BTF_LOC_PARAM_CONST);
	btf__add_loc_param_value(btf, 0x9abcdef0);
	btf__add_loc_param_value(btf, 0x12345678);
	btf__add_loc_param(btf, 8, BTF_LOC_PARAM_CONST);
	btf__add_loc_param_value(btf, 0xfeedface);
	btf__add_loc_param_value(btf, 0xdeadbeef);
	btf__add_loc_param(btf, 8, BTF_LOC_PARAM_REG |
			    BTF_LOC_PARAM_DEREF | BTF_LOC_PARAM_OFFSET |
			    BTF_LOC_PARAM_SIGNED);
	btf__add_loc_param_value(btf, BTF_LOC_PARAM_FBREG);
	btf__add_loc_param_value(btf, -0x20);
	btf__add_loc_proto(btf);
	btf__add_loc_proto_param(btf, 4);
	btf__add_loc_proto_param(btf, 5);
	btf__add_loc_proto_param(btf, 6);
	btf__add_loc_proto_param(btf, 7);
	btf__add_loc_proto_param(btf, 8);
	btf__add_loc_proto_param(btf, 9);
	btf__add_locsec(btf, "inline.text");
	btf__add_locsec_loc(btf, 3, 10, 64);

	return btf;
}

static int btf_to_tmpfile(struct btf *(*mk)(void), char *path)
{
	ssize_t written;
	struct btf *btf;
	const void *raw;
	int err = -1;
	__u32 sz;
	int fd;

	btf = mk();
	if (!btf)
		return -1;

	raw = btf__raw_data(btf, &sz);
	if (!ASSERT_OK_PTR(raw, "raw_data"))
		goto out;

	snprintf(path, PATH_MAX, "/tmp/bpftool_btf_dump.XXXXXX");
	fd = mkstemp(path);
	if (!ASSERT_OK_FD(fd, "mkstemp_btf"))
		goto out;

	written = write(fd, raw, sz);
	close(fd);
	if (!ASSERT_EQ(written, sz, "write_btf")) {
		unlink(path);
		goto out;
	}

	err = 0;
out:
	btf__free(btf);
	return err;
}

static char *dump_c(const char *btf_path, const char *base_path, bool sorted)
{
	char args[MAX_BPFTOOL_CMD_LEN];
	char *buf;
	int err;

	buf = malloc(DUMP_BUF_SZ);
	if (!ASSERT_OK_PTR(buf, "alloc_dump"))
		return NULL;

	snprintf(args, sizeof(args), "%s%s btf dump file %s format c%s",
		 base_path ? "-B " : "", base_path ?: "", btf_path, sorted ? "" : " unsorted");

	err = get_bpftool_command_output(args, buf, DUMP_BUF_SZ);
	if (!ASSERT_OK(err, "btf_dump_format_c")) {
		free(buf);
		return NULL;
	}

	return buf;
}

static char *dump_raw(const char *btf_path)
{
	char args[MAX_BPFTOOL_CMD_LEN];
	char *buf;
	int err;

	buf = malloc(DUMP_BUF_SZ);
	if (!ASSERT_OK_PTR(buf, "alloc_dump"))
		return NULL;

	snprintf(args, sizeof(args), "btf dump file %s", btf_path);
	err = get_bpftool_command_output(args, buf, DUMP_BUF_SZ);
	if (!ASSERT_OK(err, "btf_dump_raw")) {
		free(buf);
		return NULL;
	}

	return buf;
}

static char *read_expected(const char *path)
{
	char *buf = NULL;
	size_t cap = 0;
	FILE *f;
	int err;

	f = fopen(path, "r");
	if (!f) {
		err = errno;
		PRINT_FAIL("can't open expected output '%s': errno %d\n", path, err);
		return NULL;
	}

	/* no NUL in a generated header, so this reads to the end */
	if (getdelim(&buf, &cap, '\0', f) < 0) {
		err = errno;
		PRINT_FAIL("can't read expected output '%s': errno %d\n", path, err);
		free(buf);
		buf = NULL;
	}

	fclose(f);
	return buf;
}

static void test_dump(const char *btf_path, bool sorted)
{
	const char *exp_path;
	char *dump, *exp;

	exp_path = sorted ? EXPECTED_SORTED : EXPECTED_UNSORTED;

	dump = dump_c(btf_path, NULL, sorted);
	if (!dump)
		return;

	exp = read_expected(exp_path);
	if (!exp)
		goto out_dump;

	ASSERT_TEXT_EQ(dump, exp, sorted ? "cmp_sorted" : "cmp_unsorted");

	free(exp);
out_dump:
	free(dump);
}

static void test_legacy_arena(const char *btf_path)
{
	char *dump;

	dump = dump_c(btf_path, NULL, true);
	if (!dump)
		return;

	ASSERT_HAS_SUBSTR(dump, "int __attribute__((address_space(1))) *arena(void)",
			  "arena_kfunc");
	free(dump);
}

static void test_legacy_arena_split(const char *base_path)
{
	char path[PATH_MAX];
	char *dump;

	legacy_arena_base = btf__parse(base_path, NULL);
	if (!ASSERT_OK_PTR(legacy_arena_base, "parse_base"))
		return;
	if (btf_to_tmpfile(mk_legacy_arena_split_btf, path))
		goto out;

	dump = dump_c(path, base_path, true);
	unlink(path);
	if (!dump)
		goto out;

	ASSERT_HAS_SUBSTR(dump, "int __attribute__((address_space(1))) *arena(void)",
			  "base_arena_kfunc");
	ASSERT_HAS_SUBSTR(dump, "int __attribute__((address_space(1))) *arena_mod(void)",
			  "split_arena_kfunc");
	free(dump);
out:
	btf__free(legacy_arena_base);
}

static void test_loc_dump(const char *btf_path)
{
	const char expected[] =
		"[1] INT 'int' size=4 bits_offset=0 nr_bits=32 encoding=SIGNED\n"
		"[2] FUNC_PROTO '(anon)' ret_type_id=1 vlen=6\n"
		"\t'arg1' type_id=1\n"
		"\t'arg2' type_id=1\n"
		"\t'arg3' type_id=1\n"
		"\t'arg4' type_id=1\n"
		"\t'arg5' type_id=1\n"
		"\t'arg6' type_id=1\n"
		"[3] FUNC 'foo' type_id=2 linkage=static\n"
		"[4] LOC_PARAM '(anon)' size=4 flags=0x8 vlen=1 values='reg1'\n"
		"[5] LOC_PARAM '(anon)' size=8 flags=0x38 vlen=2 values='*(reg2 + 0x10)'\n"
		"[6] LOC_PARAM '(anon)' size=8 flags=0x29 vlen=2 values='fbreg - 0x10'\n"
		"[7] LOC_PARAM '(anon)' size=8 flags=0x6 vlen=2 values='address 0x123456789abcdef0'\n"
		"[8] LOC_PARAM '(anon)' size=8 flags=0x2 vlen=2 values='const 0xdeadbeeffeedface'\n"
		"[9] LOC_PARAM '(anon)' size=8 flags=0x39 vlen=2 values='*(fbreg - 0x20)'\n"
		"[10] LOC_PROTO '(anon)' vlen=6\n"
		"\ttype_id=4 value='reg1'\n"
		"\ttype_id=5 value='*(reg2 + 0x10)'\n"
		"\ttype_id=6 value='fbreg - 0x10'\n"
		"\ttype_id=7 value='address 0x123456789abcdef0'\n"
		"\ttype_id=8 value='const 0xdeadbeeffeedface'\n"
		"\ttype_id=9 value='*(fbreg - 0x20)'\n"
		"[11] LOCSEC 'inline.text' vlen=1\n"
		"\tfunc='foo(arg1 [reg1], arg2 [*(reg2 + 0x10)], arg3 [fbreg - 0x10], arg4 [address 0x123456789abcdef0], arg5 [const 0xdeadbeeffeedface], arg6 [*(fbreg - 0x20)])' func_type_id=3 loc_proto_type_id=10 offset=64\n";
	char *dump;

	dump = dump_raw(btf_path);
	if (!dump)
		return;

	ASSERT_TEXT_EQ(dump, expected, "cmp_loc_dump");
	free(dump);
}

void test_bpftool_btf_dump(void)
{
	char path[PATH_MAX];

	if (!btf_to_tmpfile(mk_btf, path)) {
		if (test__start_subtest("c_sorted"))
			test_dump(path, true);
		if (test__start_subtest("c_unsorted"))
			test_dump(path, false);
		unlink(path);
	}

	if (!btf_to_tmpfile(mk_hdr_extra_btf, path)) {
		if (test__start_subtest("hdr_extra"))
			test_dump(path, true);
		unlink(path);
	}

	if (!btf_to_tmpfile(mk_legacy_arena_btf, path)) {
		if (test__start_subtest("legacy_arena"))
			test_legacy_arena(path);
		if (test__start_subtest("legacy_arena_split"))
			test_legacy_arena_split(path);
		unlink(path);
	}

	if (!btf_to_tmpfile(mk_loc_btf, path)) {
		if (test__start_subtest("loc_dump"))
			test_loc_dump(path);
		unlink(path);
	}
}
