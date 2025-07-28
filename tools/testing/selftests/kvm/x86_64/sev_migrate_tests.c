// SPDX-License-Identifier: GPL-2.0-only
#include <linux/kvm.h>
#include <linux/psp-sev.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <errno.h>
#include <pthread.h>

#include "test_util.h"
#include "kvm_util.h"
#include "processor.h"
#include "sev.h"
#include "kselftest.h"

#define NR_MIGRATE_TEST_VCPUS 4
#define NR_MIGRATE_TEST_VMS 3
#define NR_LOCK_TESTING_THREADS 3
#define NR_LOCK_TESTING_ITERATIONS 10000

bool have_sev_es;

static struct kvm_vm *sev_vm_create(bool es)
{
	struct kvm_vm *vm;
	int i;

	vm = vm_create_barebones();
	if (!es)
		sev_vm_init(vm);
	else
		sev_es_vm_init(vm);

	for (i = 0; i < NR_MIGRATE_TEST_VCPUS; ++i)
		__vm_vcpu_add(vm, i);

	sev_vm_launch(vm, es ? SEV_POLICY_ES : 0);

	if (es)
		vm_sev_ioctl(vm, KVM_SEV_LAUNCH_UPDATE_VMSA, NULL);
	return vm;
}

static struct kvm_vm *aux_vm_create(bool with_vcpus)
{
	struct kvm_vm *vm;
	int i;

	vm = vm_create_barebones();
	if (!with_vcpus)
		return vm;

	for (i = 0; i < NR_MIGRATE_TEST_VCPUS; ++i)
		__vm_vcpu_add(vm, i);

	return vm;
}

static int __sev_migrate_from(struct kvm_vm *dst, struct kvm_vm *src)
{
	return __vm_enable_cap(dst, KVM_CAP_VM_MOVE_ENC_CONTEXT_FROM, src->fd);
}


static void sev_migrate_from(struct kvm_vm *dst, struct kvm_vm *src)
{
	int ret;

	ret = __sev_migrate_from(dst, src);
	TEST_ASSERT(!ret, "Migration failed, ret: %d, errno: %d", ret, errno);
}

static void test_sev_migrate_from(bool es)
{
	struct kvm_vm *src_vm;
	struct kvm_vm *dst_vms[NR_MIGRATE_TEST_VMS];
	int i, ret;

	src_vm = sev_vm_create(es);
	for (i = 0; i < NR_MIGRATE_TEST_VMS; ++i)
		dst_vms[i] = aux_vm_create(true);

	/* Initial migration from the src to the first dst. */
	sev_migrate_from(dst_vms[0], src_vm);

	for (i = 1; i < NR_MIGRATE_TEST_VMS; i++)
		sev_migrate_from(dst_vms[i], dst_vms[i - 1]);

	/* Migrate the guest back to the original VM. */
	ret = __sev_migrate_from(src_vm, dst_vms[NR_MIGRATE_TEST_VMS - 1]);
	TEST_ASSERT(ret == -1 && errno == EIO,
		    "VM that was migrated from should be dead. ret %d, errno: %d", ret,
		    errno);

	kvm_vm_free(src_vm);
	for (i = 0; i < NR_MIGRATE_TEST_VMS; ++i)
		kvm_vm_free(dst_vms[i]);
}

struct locking_thread_input {
	struct kvm_vm *vm;
	struct kvm_vm *source_vms[NR_LOCK_TESTING_THREADS];
};

static void *locking_test_thread(void *arg)
{
	int i, j;
	struct locking_thread_input *input = (struct locking_thread_input *)arg;

	for (i = 0; i < NR_LOCK_TESTING_ITERATIONS; ++i) {
		j = i % NR_LOCK_TESTING_THREADS;
		__sev_migrate_from(input->vm, input->source_vms[j]);
	}

	return NULL;
}

static void test_sev_migrate_locking(void)
{
	struct locking_thread_input input[NR_LOCK_TESTING_THREADS];
	pthread_t pt[NR_LOCK_TESTING_THREADS];
	int i;

	for (i = 0; i < NR_LOCK_TESTING_THREADS; ++i) {
		input[i].vm = sev_vm_create(/* es= */ false);
		input[0].source_vms[i] = input[i].vm;
	}
	for (i = 1; i < NR_LOCK_TESTING_THREADS; ++i)
		memcpy(input[i].source_vms, input[0].source_vms,
		       sizeof(input[i].source_vms));

	for (i = 0; i < NR_LOCK_TESTING_THREADS; ++i)
		pthread_create(&pt[i], NULL, locking_test_thread, &input[i]);

	for (i = 0; i < NR_LOCK_TESTING_THREADS; ++i)
		pthread_join(pt[i], NULL);
	for (i = 0; i < NR_LOCK_TESTING_THREADS; ++i)
		kvm_vm_free(input[i].vm);
}

static void test_sev_migrate_parameters(void)
{
	struct kvm_vm *sev_vm, *sev_es_vm, *vm_no_vcpu, *vm_no_sev,
		*sev_es_vm_no_vmsa;
	int ret;

	vm_no_vcpu = vm_create_barebones();
	vm_no_sev = aux_vm_create(true);
	ret = __sev_migrate_from(vm_no_vcpu, vm_no_sev);
	TEST_ASSERT(ret == -1 && errno == EINVAL,
		    "Migrations require SEV enabled. ret %d, errno: %d", ret,
		    errno);

	if (!have_sev_es)
		goto out;

	sev_vm = sev_vm_create(/* es= */ false);
	sev_es_vm = sev_vm_create(/* es= */ true);
	sev_es_vm_no_vmsa = vm_create_barebones();
	sev_es_vm_init(sev_es_vm_no_vmsa);
	__vm_vcpu_add(sev_es_vm_no_vmsa, 1);

	ret = __sev_migrate_from(sev_vm, sev_es_vm);
	TEST_ASSERT(
		ret == -1 && errno == EINVAL,
		"Should not be able migrate to SEV enabled VM. ret: %d, errno: %d",
		ret, errno);

	ret = __sev_migrate_from(sev_es_vm, sev_vm);
	TEST_ASSERT(
		ret == -1 && errno == EINVAL,
		"Should not be able migrate to SEV-ES enabled VM. ret: %d, errno: %d",
		ret, errno);

	ret = __sev_migrate_from(vm_no_vcpu, sev_es_vm);
	TEST_ASSERT(
		ret == -1 && errno == EINVAL,
		"SEV-ES migrations require same number of vCPUS. ret: %d, errno: %d",
		ret, errno);

	ret = __sev_migrate_from(vm_no_vcpu, sev_es_vm_no_vmsa);
	TEST_ASSERT(
		ret == -1 && errno == EINVAL,
		"SEV-ES migrations require UPDATE_VMSA. ret %d, errno: %d",
		ret, errno);

	kvm_vm_free(sev_vm);
	kvm_vm_free(sev_es_vm);
	kvm_vm_free(sev_es_vm_no_vmsa);
out:
	kvm_vm_free(vm_no_vcpu);
	kvm_vm_free(vm_no_sev);
}

static int __sev_mirror_create(struct kvm_vm *dst, struct kvm_vm *src)
{
	return __vm_enable_cap(dst, KVM_CAP_VM_COPY_ENC_CONTEXT_FROM, src->fd);
}


static void sev_mirror_create(struct kvm_vm *dst, struct kvm_vm *src)
{
	int ret;

	ret = __sev_mirror_create(dst, src);
	TEST_ASSERT(!ret, "Copying context failed, ret: %d, errno: %d", ret, errno);
}

static void verify_mirror_allowed_cmds(struct kvm_vm *vm)
{
	struct kvm_sev_guest_status status;
	int cmd_id;

	for (cmd_id = KVM_SEV_INIT; cmd_id < KVM_SEV_NR_MAX; ++cmd_id) {
		int ret;

		/*
		 * These commands are allowed for mirror VMs, all others are
		 * not.
		 */
		switch (cmd_id) {
		case KVM_SEV_LAUNCH_UPDATE_VMSA:
		case KVM_SEV_GUEST_STATUS:
		case KVM_SEV_DBG_DECRYPT:
		case KVM_SEV_DBG_ENCRYPT:
			continue;
		default:
			break;
		}

		/*
		 * These commands should be disallowed before the data
		 * parameter is examined so NULL is OK here.
		 */
		ret = __vm_sev_ioctl(vm, cmd_id, NULL);
		TEST_ASSERT(
			ret == -1 && errno == EINVAL,
			"Should not be able call command: %d. ret: %d, errno: %d",
			cmd_id, ret, errno);
	}

	vm_sev_ioctl(vm, KVM_SEV_GUEST_STATUS, &status);
}

static void test_sev_mirror(bool es)
{
	struct kvm_vm *src_vm, *dst_vm;
	int i;

	src_vm = sev_vm_create(es);
	dst_vm = aux_vm_create(false);

	sev_mirror_create(dst_vm, src_vm);

	/* Check that we can complete creation of the mirror VM.  */
	for (i = 0; i < NR_MIGRATE_TEST_VCPUS; ++i)
		__vm_vcpu_add(dst_vm, i);

	if (es)
		vm_sev_ioctl(dst_vm, KVM_SEV_LAUNCH_UPDATE_VMSA, NULL);

	verify_mirror_allowed_cmds(dst_vm);

	kvm_vm_free(src_vm);
	kvm_vm_free(dst_vm);
}

static void test_sev_mirror_parameters(void)
{
	struct kvm_vm *sev_vm, *sev_es_vm, *vm_no_vcpu, *vm_with_vcpu;
	int ret;

	sev_vm = sev_vm_create(/* es= */ false);
	vm_with_vcpu = aux_vm_create(true);
	vm_no_vcpu = aux_vm_create(false);

	ret = __sev_mirror_create(sev_vm, sev_vm);
	TEST_ASSERT(
		ret == -1 && errno == EINVAL,
		"Should not be able copy context to self. ret: %d, errno: %d",
		ret, errno);

	ret = __sev_mirror_create(vm_no_vcpu, vm_with_vcpu);
	TEST_ASSERT(ret == -1 && errno == EINVAL,
		    "Copy context requires SEV enabled. ret %d, errno: %d", ret,
		    errno);

	ret = __sev_mirror_create(vm_with_vcpu, sev_vm);
	TEST_ASSERT(
		ret == -1 && errno == EINVAL,
		"SEV copy context requires no vCPUS on the destination. ret: %d, errno: %d",
		ret, errno);

	if (!have_sev_es)
		goto out;

	sev_es_vm = sev_vm_create(/* es= */ true);
	ret = __sev_mirror_create(sev_vm, sev_es_vm);
	TEST_ASSERT(
		ret == -1 && errno == EINVAL,
		"Should not be able copy context to SEV enabled VM. ret: %d, errno: %d",
		ret, errno);

	ret = __sev_mirror_create(sev_es_vm, sev_vm);
	TEST_ASSERT(
		ret == -1 && errno == EINVAL,
		"Should not be able copy context to SEV-ES enabled VM. ret: %d, errno: %d",
		ret, errno);

	kvm_vm_free(sev_es_vm);

out:
	kvm_vm_free(sev_vm);
	kvm_vm_free(vm_with_vcpu);
	kvm_vm_free(vm_no_vcpu);
}

static void test_sev_move_copy(void)
{
	struct kvm_vm *dst_vm, *dst2_vm, *dst3_vm, *sev_vm, *mirror_vm,
		      *dst_mirror_vm, *dst2_mirror_vm, *dst3_mirror_vm;

	sev_vm = sev_vm_create(/* es= */ false);
	dst_vm = aux_vm_create(true);
	dst2_vm = aux_vm_create(true);
	dst3_vm = aux_vm_create(true);
	mirror_vm = aux_vm_create(false);
	dst_mirror_vm = aux_vm_create(false);
	dst2_mirror_vm = aux_vm_create(false);
	dst3_mirror_vm = aux_vm_create(false);

	sev_mirror_create(mirror_vm, sev_vm);

	sev_migrate_from(dst_mirror_vm, mirror_vm);
	sev_migrate_from(dst_vm, sev_vm);

	sev_migrate_from(dst2_vm, dst_vm);
	sev_migrate_from(dst2_mirror_vm, dst_mirror_vm);

	sev_migrate_from(dst3_mirror_vm, dst2_mirror_vm);
	sev_migrate_from(dst3_vm, dst2_vm);

	kvm_vm_free(dst_vm);
	kvm_vm_free(sev_vm);
	kvm_vm_free(dst2_vm);
	kvm_vm_free(dst3_vm);
	kvm_vm_free(mirror_vm);
	kvm_vm_free(dst_mirror_vm);
	kvm_vm_free(dst2_mirror_vm);
	kvm_vm_free(dst3_mirror_vm);

	/*
	 * Run similar test be destroy mirrors before mirrored VMs to ensure
	 * destruction is done safely.
	 */
	sev_vm = sev_vm_create(/* es= */ false);
	dst_vm = aux_vm_create(true);
	mirror_vm = aux_vm_create(false);
	dst_mirror_vm = aux_vm_create(false);

	sev_mirror_create(mirror_vm, sev_vm);

	sev_migrate_from(dst_mirror_vm, mirror_vm);
	sev_migrate_from(dst_vm, sev_vm);

	kvm_vm_free(mirror_vm);
	kvm_vm_free(dst_mirror_vm);
	kvm_vm_free(dst_vm);
	kvm_vm_free(sev_vm);
}

int main(int argc, char *argv[])
{
	TEST_REQUIRE(kvm_has_cap(KVM_CAP_VM_MOVE_ENC_CONTEXT_FROM));
	TEST_REQUIRE(kvm_has_cap(KVM_CAP_VM_COPY_ENC_CONTEXT_FROM));

	TEST_REQUIRE(kvm_cpu_has(X86_FEATURE_SEV));

	have_sev_es = kvm_cpu_has(X86_FEATURE_SEV_ES);

	if (kvm_has_cap(KVM_CAP_VM_MOVE_ENC_CONTEXT_FROM)) {
		test_sev_migrate_from(/* es= */ false);
		if (have_sev_es)
			test_sev_migrate_from(/* es= */ true);
		test_sev_migrate_locking();
		test_sev_migrate_parameters();
		if (kvm_has_cap(KVM_CAP_VM_COPY_ENC_CONTEXT_FROM))
			test_sev_move_copy();
	}
	if (kvm_has_cap(KVM_CAP_VM_COPY_ENC_CONTEXT_FROM)) {
		test_sev_mirror(/* es= */ false);
		if (have_sev_es)
			test_sev_mirror(/* es= */ true);
		test_sev_mirror_parameters();
	}
	return 0;
}
