// SPDX-License-Identifier: GPL-2.0
/*
 * steal/stolen time test
 *
 * Copyright (C) 2020, Red Hat, Inc.
 */
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <pthread.h>
#include <linux/kernel.h>
#include <asm/kvm.h>
#ifdef __riscv
#include "sbi.h"
#else
#include <asm/kvm_para.h>
#endif

#include "test_util.h"
#include "kvm_util.h"
#include "processor.h"
#include "ucall_common.h"

#define NR_VCPUS		4
#define ST_GPA_BASE		(1 << 30)

static void *st_gva[NR_VCPUS];
static uint64_t guest_stolen_time[NR_VCPUS];

#if defined(__x86_64__)

/* steal_time must have 64-byte alignment */
#define STEAL_TIME_SIZE		((sizeof(struct kvm_steal_time) + 63) & ~63)

static void check_status(struct kvm_steal_time *st)
{
	GUEST_ASSERT(!(READ_ONCE(st->version) & 1));
	GUEST_ASSERT_EQ(READ_ONCE(st->flags), 0);
	GUEST_ASSERT_EQ(READ_ONCE(st->preempted), 0);
}

static void guest_code(int cpu)
{
	struct kvm_steal_time *st = st_gva[cpu];
	uint32_t version;

	GUEST_ASSERT_EQ(rdmsr(MSR_KVM_STEAL_TIME), ((uint64_t)st_gva[cpu] | KVM_MSR_ENABLED));

	memset(st, 0, sizeof(*st));
	GUEST_SYNC(0);

	check_status(st);
	WRITE_ONCE(guest_stolen_time[cpu], st->steal);
	version = READ_ONCE(st->version);
	check_status(st);
	GUEST_SYNC(1);

	check_status(st);
	GUEST_ASSERT(version < READ_ONCE(st->version));
	WRITE_ONCE(guest_stolen_time[cpu], st->steal);
	check_status(st);
	GUEST_DONE();
}

static bool is_steal_time_supported(struct kvm_vcpu *vcpu)
{
	return kvm_cpu_has(X86_FEATURE_KVM_STEAL_TIME);
}

static void steal_time_init(struct kvm_vcpu *vcpu, uint32_t i)
{
	int ret;

	/* ST_GPA_BASE is identity mapped */
	st_gva[i] = (void *)(ST_GPA_BASE + i * STEAL_TIME_SIZE);
	sync_global_to_guest(vcpu->vm, st_gva[i]);

	ret = _vcpu_set_msr(vcpu, MSR_KVM_STEAL_TIME,
			    (ulong)st_gva[i] | KVM_STEAL_RESERVED_MASK);
	TEST_ASSERT(ret == 0, "Bad GPA didn't fail");

	vcpu_set_msr(vcpu, MSR_KVM_STEAL_TIME, (ulong)st_gva[i] | KVM_MSR_ENABLED);
}

static void steal_time_dump(struct kvm_vm *vm, uint32_t vcpu_idx)
{
	struct kvm_steal_time *st = addr_gva2hva(vm, (ulong)st_gva[vcpu_idx]);

	ksft_print_msg("VCPU%d:\n", vcpu_idx);
	ksft_print_msg("    steal:     %lld\n", st->steal);
	ksft_print_msg("    version:   %d\n", st->version);
	ksft_print_msg("    flags:     %d\n", st->flags);
	ksft_print_msg("    preempted: %d\n", st->preempted);
	ksft_print_msg("    u8_pad:    %d %d %d\n",
			st->u8_pad[0], st->u8_pad[1], st->u8_pad[2]);
	ksft_print_msg("    pad:       %d %d %d %d %d %d %d %d %d %d %d\n",
			st->pad[0], st->pad[1], st->pad[2], st->pad[3],
			st->pad[4], st->pad[5], st->pad[6], st->pad[7],
			st->pad[8], st->pad[9], st->pad[10]);
}

#elif defined(__aarch64__)

/* PV_TIME_ST must have 64-byte alignment */
#define STEAL_TIME_SIZE		((sizeof(struct st_time) + 63) & ~63)

#define SMCCC_ARCH_FEATURES	0x80000001
#define PV_TIME_FEATURES	0xc5000020
#define PV_TIME_ST		0xc5000021

struct st_time {
	uint32_t rev;
	uint32_t attr;
	uint64_t st_time;
};

static int64_t smccc(uint32_t func, uint64_t arg)
{
	struct arm_smccc_res res;

	smccc_hvc(func, arg, 0, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

static void check_status(struct st_time *st)
{
	GUEST_ASSERT_EQ(READ_ONCE(st->rev), 0);
	GUEST_ASSERT_EQ(READ_ONCE(st->attr), 0);
}

static void guest_code(int cpu)
{
	struct st_time *st;
	int64_t status;

	status = smccc(SMCCC_ARCH_FEATURES, PV_TIME_FEATURES);
	GUEST_ASSERT_EQ(status, 0);
	status = smccc(PV_TIME_FEATURES, PV_TIME_FEATURES);
	GUEST_ASSERT_EQ(status, 0);
	status = smccc(PV_TIME_FEATURES, PV_TIME_ST);
	GUEST_ASSERT_EQ(status, 0);

	status = smccc(PV_TIME_ST, 0);
	GUEST_ASSERT_NE(status, -1);
	GUEST_ASSERT_EQ(status, (ulong)st_gva[cpu]);

	st = (struct st_time *)status;
	GUEST_SYNC(0);

	check_status(st);
	WRITE_ONCE(guest_stolen_time[cpu], st->st_time);
	GUEST_SYNC(1);

	check_status(st);
	WRITE_ONCE(guest_stolen_time[cpu], st->st_time);
	GUEST_DONE();
}

static bool is_steal_time_supported(struct kvm_vcpu *vcpu)
{
	struct kvm_device_attr dev = {
		.group = KVM_ARM_VCPU_PVTIME_CTRL,
		.attr = KVM_ARM_VCPU_PVTIME_IPA,
	};

	return !__vcpu_ioctl(vcpu, KVM_HAS_DEVICE_ATTR, &dev);
}

static void steal_time_init(struct kvm_vcpu *vcpu, uint32_t i)
{
	struct kvm_vm *vm = vcpu->vm;
	uint64_t st_ipa;
	int ret;

	struct kvm_device_attr dev = {
		.group = KVM_ARM_VCPU_PVTIME_CTRL,
		.attr = KVM_ARM_VCPU_PVTIME_IPA,
		.addr = (uint64_t)&st_ipa,
	};

	vcpu_ioctl(vcpu, KVM_HAS_DEVICE_ATTR, &dev);

	/* ST_GPA_BASE is identity mapped */
	st_gva[i] = (void *)(ST_GPA_BASE + i * STEAL_TIME_SIZE);
	sync_global_to_guest(vm, st_gva[i]);

	st_ipa = (ulong)st_gva[i] | 1;
	ret = __vcpu_ioctl(vcpu, KVM_SET_DEVICE_ATTR, &dev);
	TEST_ASSERT(ret == -1 && errno == EINVAL, "Bad IPA didn't report EINVAL");

	st_ipa = (ulong)st_gva[i];
	vcpu_ioctl(vcpu, KVM_SET_DEVICE_ATTR, &dev);

	ret = __vcpu_ioctl(vcpu, KVM_SET_DEVICE_ATTR, &dev);
	TEST_ASSERT(ret == -1 && errno == EEXIST, "Set IPA twice without EEXIST");
}

static void steal_time_dump(struct kvm_vm *vm, uint32_t vcpu_idx)
{
	struct st_time *st = addr_gva2hva(vm, (ulong)st_gva[vcpu_idx]);

	ksft_print_msg("VCPU%d:\n", vcpu_idx);
	ksft_print_msg("    rev:     %d\n", st->rev);
	ksft_print_msg("    attr:    %d\n", st->attr);
	ksft_print_msg("    st_time: %ld\n", st->st_time);
}

#elif defined(__riscv)

/* SBI STA shmem must have 64-byte alignment */
#define STEAL_TIME_SIZE		((sizeof(struct sta_struct) + 63) & ~63)

static vm_paddr_t st_gpa[NR_VCPUS];

struct sta_struct {
	uint32_t sequence;
	uint32_t flags;
	uint64_t steal;
	uint8_t preempted;
	uint8_t pad[47];
} __packed;

static void sta_set_shmem(vm_paddr_t gpa, unsigned long flags)
{
	unsigned long lo = (unsigned long)gpa;
#if __riscv_xlen == 32
	unsigned long hi = (unsigned long)(gpa >> 32);
#else
	unsigned long hi = gpa == -1 ? -1 : 0;
#endif
	struct sbiret ret = sbi_ecall(SBI_EXT_STA, 0, lo, hi, flags, 0, 0, 0);

	GUEST_ASSERT(ret.value == 0 && ret.error == 0);
}

static void check_status(struct sta_struct *st)
{
	GUEST_ASSERT(!(READ_ONCE(st->sequence) & 1));
	GUEST_ASSERT(READ_ONCE(st->flags) == 0);
	GUEST_ASSERT(READ_ONCE(st->preempted) == 0);
}

static void guest_code(int cpu)
{
	struct sta_struct *st = st_gva[cpu];
	uint32_t sequence;
	long out_val = 0;
	bool probe;

	probe = guest_sbi_probe_extension(SBI_EXT_STA, &out_val);
	GUEST_ASSERT(probe && out_val == 1);

	sta_set_shmem(st_gpa[cpu], 0);
	GUEST_SYNC(0);

	check_status(st);
	WRITE_ONCE(guest_stolen_time[cpu], st->steal);
	sequence = READ_ONCE(st->sequence);
	check_status(st);
	GUEST_SYNC(1);

	check_status(st);
	GUEST_ASSERT(sequence < READ_ONCE(st->sequence));
	WRITE_ONCE(guest_stolen_time[cpu], st->steal);
	check_status(st);
	GUEST_DONE();
}

static bool is_steal_time_supported(struct kvm_vcpu *vcpu)
{
	uint64_t id = RISCV_SBI_EXT_REG(KVM_RISCV_SBI_EXT_STA);
	unsigned long enabled;

	vcpu_get_reg(vcpu, id, &enabled);
	TEST_ASSERT(enabled == 0 || enabled == 1, "Expected boolean result");

	return enabled;
}

static void steal_time_init(struct kvm_vcpu *vcpu, uint32_t i)
{
	/* ST_GPA_BASE is identity mapped */
	st_gva[i] = (void *)(ST_GPA_BASE + i * STEAL_TIME_SIZE);
	st_gpa[i] = addr_gva2gpa(vcpu->vm, (vm_vaddr_t)st_gva[i]);
	sync_global_to_guest(vcpu->vm, st_gva[i]);
	sync_global_to_guest(vcpu->vm, st_gpa[i]);
}

static void steal_time_dump(struct kvm_vm *vm, uint32_t vcpu_idx)
{
	struct sta_struct *st = addr_gva2hva(vm, (ulong)st_gva[vcpu_idx]);
	int i;

	pr_info("VCPU%d:\n", vcpu_idx);
	pr_info("    sequence:  %d\n", st->sequence);
	pr_info("    flags:     %d\n", st->flags);
	pr_info("    steal:     %"PRIu64"\n", st->steal);
	pr_info("    preempted: %d\n", st->preempted);
	pr_info("    pad:      ");
	for (i = 0; i < 47; ++i)
		pr_info("%d", st->pad[i]);
	pr_info("\n");
}

#endif

static void *do_steal_time(void *arg)
{
	struct timespec ts, stop;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	stop = timespec_add_ns(ts, MIN_RUN_DELAY_NS);

	while (1) {
		clock_gettime(CLOCK_MONOTONIC, &ts);
		if (timespec_to_ns(timespec_sub(ts, stop)) >= 0)
			break;
	}

	return NULL;
}

static void run_vcpu(struct kvm_vcpu *vcpu)
{
	struct ucall uc;

	vcpu_run(vcpu);

	switch (get_ucall(vcpu, &uc)) {
	case UCALL_SYNC:
	case UCALL_DONE:
		break;
	case UCALL_ABORT:
		REPORT_GUEST_ASSERT(uc);
	default:
		TEST_ASSERT(false, "Unexpected exit: %s",
			    exit_reason_str(vcpu->run->exit_reason));
	}
}

int main(int ac, char **av)
{
	struct kvm_vcpu *vcpus[NR_VCPUS];
	struct kvm_vm *vm;
	pthread_attr_t attr;
	pthread_t thread;
	cpu_set_t cpuset;
	unsigned int gpages;
	long stolen_time;
	long run_delay;
	bool verbose;
	int i;

	verbose = ac > 1 && (!strncmp(av[1], "-v", 3) || !strncmp(av[1], "--verbose", 10));

	/* Set CPU affinity so we can force preemption of the VCPU */
	CPU_ZERO(&cpuset);
	CPU_SET(0, &cpuset);
	pthread_attr_init(&attr);
	pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuset);
	pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

	/* Create a VM and an identity mapped memslot for the steal time structure */
	vm = vm_create_with_vcpus(NR_VCPUS, guest_code, vcpus);
	gpages = vm_calc_num_guest_pages(VM_MODE_DEFAULT, STEAL_TIME_SIZE * NR_VCPUS);
	vm_userspace_mem_region_add(vm, VM_MEM_SRC_ANONYMOUS, ST_GPA_BASE, 1, gpages, 0);
	virt_map(vm, ST_GPA_BASE, ST_GPA_BASE, gpages);

	ksft_print_header();
	TEST_REQUIRE(is_steal_time_supported(vcpus[0]));
	ksft_set_plan(NR_VCPUS);

	/* Run test on each VCPU */
	for (i = 0; i < NR_VCPUS; ++i) {
		steal_time_init(vcpus[i], i);

		vcpu_args_set(vcpus[i], 1, i);

		/* First VCPU run initializes steal-time */
		run_vcpu(vcpus[i]);

		/* Second VCPU run, expect guest stolen time to be <= run_delay */
		run_vcpu(vcpus[i]);
		sync_global_from_guest(vm, guest_stolen_time[i]);
		stolen_time = guest_stolen_time[i];
		run_delay = get_run_delay();
		TEST_ASSERT(stolen_time <= run_delay,
			    "Expected stolen time <= %ld, got %ld",
			    run_delay, stolen_time);

		/* Steal time from the VCPU. The steal time thread has the same CPU affinity as the VCPUs. */
		run_delay = get_run_delay();
		pthread_create(&thread, &attr, do_steal_time, NULL);
		do
			sched_yield();
		while (get_run_delay() - run_delay < MIN_RUN_DELAY_NS);
		pthread_join(thread, NULL);
		run_delay = get_run_delay() - run_delay;
		TEST_ASSERT(run_delay >= MIN_RUN_DELAY_NS,
			    "Expected run_delay >= %ld, got %ld",
			    MIN_RUN_DELAY_NS, run_delay);

		/* Run VCPU again to confirm stolen time is consistent with run_delay */
		run_vcpu(vcpus[i]);
		sync_global_from_guest(vm, guest_stolen_time[i]);
		stolen_time = guest_stolen_time[i] - stolen_time;
		TEST_ASSERT(stolen_time >= run_delay,
			    "Expected stolen time >= %ld, got %ld",
			    run_delay, stolen_time);

		if (verbose) {
			ksft_print_msg("VCPU%d: total-stolen-time=%ld test-stolen-time=%ld%s\n",
				       i, guest_stolen_time[i], stolen_time,
				       stolen_time == run_delay ?
				       " (BONUS: guest test-stolen-time even exactly matches test-run_delay)" : "");
			steal_time_dump(vm, i);
		}
		ksft_test_result_pass("vcpu%d\n", i);
	}

	/* Print results and exit() accordingly */
	ksft_finished();
}
