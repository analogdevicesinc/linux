// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests for the rules by which the ids of one credential allow it to act
 * on another, see "One task acting on another" in kernel/cred.c.
 *
 * Each rule is held against the expression it replaced, word for word, as it
 * stood where it was used, for every combination of a few ids.
 */
#include <kunit/test.h>
#include <linux/cred.h>
#include <linux/uidgid.h>

/* Root, since two of the rules know it, and two others. */
static const uid_t ids[] = { 0, 1000, 1001 };
#define NR_IDS	ARRAY_SIZE(ids)

static void set_uids(struct cred *cred, unsigned int *n)
{
	cred->uid = KUIDT_INIT(ids[*n % NR_IDS]);
	*n /= NR_IDS;
	cred->euid = KUIDT_INIT(ids[*n % NR_IDS]);
	*n /= NR_IDS;
	cred->suid = KUIDT_INIT(ids[*n % NR_IDS]);
	*n /= NR_IDS;
}

/* NR_IDS to the power of six: three uids each for the subject and the object. */
#define NR_UID_COMBINATIONS	(NR_IDS * NR_IDS * NR_IDS * NR_IDS * NR_IDS * NR_IDS)

static void uid_combination(unsigned int n, struct cred *cred, struct cred *tcred)
{
	memset(cred, 0, sizeof(*cred));
	memset(tcred, 0, sizeof(*tcred));
	set_uids(cred, &n);
	set_uids(tcred, &n);
}

/* kill_ok_by_cred() and kill_as_cred_perm() */
static bool old_signal(const struct cred *cred, const struct cred *tcred)
{
	return uid_eq(cred->euid, tcred->suid) ||
	       uid_eq(cred->euid, tcred->uid) ||
	       uid_eq(cred->uid, tcred->suid) ||
	       uid_eq(cred->uid, tcred->uid);
}

/* sigio_perm(), which had the uid and the euid of the owner in @fown */
static bool old_sigio(const struct cred *fown, const struct cred *cred)
{
	return (uid_eq(fown->euid, GLOBAL_ROOT_UID) ||
		uid_eq(fown->euid, cred->suid) || uid_eq(fown->euid, cred->uid) ||
		uid_eq(fown->uid,  cred->suid) || uid_eq(fown->uid,  cred->uid));
}

/* check_same_owner(), twice */
static bool old_sched(const struct cred *cred, const struct cred *pcred)
{
	return (uid_eq(cred->euid, pcred->euid) ||
		uid_eq(cred->euid, pcred->uid));
}

/* set_one_prio_perm() */
static bool old_nice(const struct cred *cred, const struct cred *pcred)
{
	if (uid_eq(pcred->uid,  cred->euid) ||
	    uid_eq(pcred->euid, cred->euid))
		return true;
	return false;
}

/* set_task_ioprio() */
static bool old_ioprio(const struct cred *cred, const struct cred *tcred)
{
	if (!uid_eq(tcred->uid, cred->euid) &&
	    !uid_eq(tcred->uid, cred->uid))
		return false;
	return true;
}

/* __cgroup1_procs_write() and rdtgroup_task_write_permission() */
static bool old_move(const struct cred *cred, const struct cred *tcred)
{
	if (!uid_eq(cred->euid, GLOBAL_ROOT_UID) &&
	    !uid_eq(cred->euid, tcred->uid) &&
	    !uid_eq(cred->euid, tcred->suid))
		return false;
	return true;
}

static void cred_ids_uid_rules(struct kunit *test)
{
	struct cred *cred, *tcred;
	unsigned int n;

	cred = kunit_kzalloc(test, sizeof(*cred), GFP_KERNEL);
	tcred = kunit_kzalloc(test, sizeof(*tcred), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, cred);
	KUNIT_ASSERT_NOT_NULL(test, tcred);

	for (n = 0; n < NR_UID_COMBINATIONS; n++) {
		uid_combination(n, cred, tcred);

		KUNIT_EXPECT_EQ_MSG(test, cred_ids_allow_signal(cred, tcred),
				    old_signal(cred, tcred), "combination %u", n);
		KUNIT_EXPECT_EQ_MSG(test, cred_ids_allow_sigio(cred, tcred),
				    old_sigio(cred, tcred), "combination %u", n);
		KUNIT_EXPECT_EQ_MSG(test, cred_ids_allow_sched(cred, tcred),
				    old_sched(cred, tcred), "combination %u", n);
		KUNIT_EXPECT_EQ_MSG(test, cred_ids_allow_sched(cred, tcred),
				    old_nice(cred, tcred), "combination %u", n);
		KUNIT_EXPECT_EQ_MSG(test, cred_ids_allow_ioprio(cred, tcred),
				    old_ioprio(cred, tcred), "combination %u", n);
		KUNIT_EXPECT_EQ_MSG(test, cred_ids_allow_move(cred, tcred),
				    old_move(cred, tcred), "combination %u", n);
	}
}

/* __ptrace_may_access(), with the ids it picked by its mode */
static bool old_ptrace(kuid_t caller_uid, kgid_t caller_gid,
		       const struct cred *tcred)
{
	if (uid_eq(caller_uid, tcred->euid) &&
	    uid_eq(caller_uid, tcred->suid) &&
	    uid_eq(caller_uid, tcred->uid)  &&
	    gid_eq(caller_gid, tcred->egid) &&
	    gid_eq(caller_gid, tcred->sgid) &&
	    gid_eq(caller_gid, tcred->gid))
		return true;
	return false;
}

/* check_prlimit_permission() */
static bool old_prlimit(const struct cred *cred, const struct cred *tcred)
{
	return (uid_eq(cred->uid, tcred->euid) &&
		uid_eq(cred->uid, tcred->suid) &&
		uid_eq(cred->uid, tcred->uid)  &&
		gid_eq(cred->gid, tcred->egid) &&
		gid_eq(cred->gid, tcred->sgid) &&
		gid_eq(cred->gid, tcred->gid));
}

/* keyctl_session_to_parent() */
static bool old_session_to_parent(const struct cred *mycred,
				  const struct cred *pcred)
{
	if (!uid_eq(pcred->uid,	 mycred->euid) ||
	    !uid_eq(pcred->euid, mycred->euid) ||
	    !uid_eq(pcred->suid, mycred->euid) ||
	    !gid_eq(pcred->gid,	 mycred->egid) ||
	    !gid_eq(pcred->egid, mycred->egid) ||
	    !gid_eq(pcred->sgid, mycred->egid))
		return false;
	return true;
}

/*
 * Two values for each of twelve ids: uid, euid, fsuid and their gids for the
 * subject, uid, euid, suid and their gids for the object.
 */
#define NR_CONTROL_COMBINATIONS	(1U << 12)

static void cred_ids_control_rule(struct kunit *test)
{
	struct cred *cred, *tcred;
	unsigned int n;

	cred = kunit_kzalloc(test, sizeof(*cred), GFP_KERNEL);
	tcred = kunit_kzalloc(test, sizeof(*tcred), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, cred);
	KUNIT_ASSERT_NOT_NULL(test, tcred);

	for (n = 0; n < NR_CONTROL_COMBINATIONS; n++) {
		cred->uid    = KUIDT_INIT(1000 + !!(n & BIT(0)));
		cred->euid   = KUIDT_INIT(1000 + !!(n & BIT(1)));
		cred->fsuid  = KUIDT_INIT(1000 + !!(n & BIT(2)));
		cred->gid    = KGIDT_INIT(1000 + !!(n & BIT(3)));
		cred->egid   = KGIDT_INIT(1000 + !!(n & BIT(4)));
		cred->fsgid  = KGIDT_INIT(1000 + !!(n & BIT(5)));
		tcred->uid   = KUIDT_INIT(1000 + !!(n & BIT(6)));
		tcred->euid  = KUIDT_INIT(1000 + !!(n & BIT(7)));
		tcred->suid  = KUIDT_INIT(1000 + !!(n & BIT(8)));
		tcred->gid   = KGIDT_INIT(1000 + !!(n & BIT(9)));
		tcred->egid  = KGIDT_INIT(1000 + !!(n & BIT(10)));
		tcred->sgid  = KGIDT_INIT(1000 + !!(n & BIT(11)));

		KUNIT_EXPECT_EQ_MSG(test,
				    cred_ids_allow_control(cred, CRED_IDS_FS, tcred),
				    old_ptrace(cred->fsuid, cred->fsgid, tcred),
				    "combination %u", n);
		KUNIT_EXPECT_EQ_MSG(test,
				    cred_ids_allow_control(cred, CRED_IDS_REAL, tcred),
				    old_ptrace(cred->uid, cred->gid, tcred),
				    "combination %u", n);
		KUNIT_EXPECT_EQ_MSG(test,
				    cred_ids_allow_control(cred, CRED_IDS_REAL, tcred),
				    old_prlimit(cred, tcred), "combination %u", n);
		KUNIT_EXPECT_EQ_MSG(test,
				    cred_ids_allow_control(cred, CRED_IDS_EFFECTIVE, tcred),
				    old_session_to_parent(cred, tcred),
				    "combination %u", n);
	}
}

static struct kunit_case cred_ids_cases[] = {
	KUNIT_CASE(cred_ids_uid_rules),
	KUNIT_CASE(cred_ids_control_rule),
	{}
};

static struct kunit_suite cred_ids_suite = {
	.name = "cred_ids",
	.test_cases = cred_ids_cases,
};
kunit_test_suite(cred_ids_suite);

MODULE_DESCRIPTION("KUnit tests for the id rules of credentials");
MODULE_LICENSE("GPL");
