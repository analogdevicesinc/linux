// SPDX-License-Identifier: GPL-2.0-only
/*
 * A guest access to memory whose host mapping cannot be faulted in (here: a
 * shmem page beyond EOF) must make KVM_RUN fail with -EFAULT.
 *
 * On architectures with asynchronous page faults, KVM first tries to fault
 * the page in without waiting (FOLL_NOWAIT) and hands the fault to a worker
 * if that would need I/O.  The error must not be mistaken for "needs I/O",
 * otherwise the vCPU retries the access forever and KVM_RUN never returns.
 * The architecture hooks below make sure that the async path is actually
 * possible, i.e. that the error would be misclassified if KVM got it wrong.
 */
#include <signal.h>
#include <unistd.h>
#include <linux/sizes.h>
#include <linux/stringify.h>

#include "kvm_util.h"
#include "processor.h"
#include "test_util.h"
#include "ucall_common.h"

#define TEST_SLOT	10
#define TEST_SIZE	SZ_1M
#define TIMEOUT_SECS	10

#ifdef __s390x__
#define __LC_EXT_NEW_PSW		0x1b0
#define __LC_SAVED_PSW			0x200	/* free for the OS */
#define __LC_WAIT_PSW			0x210	/* free for the OS */
#define PSW_MASK_EXT			0x0100000000000000ULL
#define PSW_MASK_DAT			0x0400000000000000ULL
#define PSW_MASK_WAIT			0x0002000000000000ULL
#define CR0_SERVICE_SIGNAL_SUBMASK	BIT_ULL(63 - 54)

/*
 * Handle pfault like Linux does: on the init interrupt (ext cpu address
 * 0x0600) remember the interrupted PSW and wait until the done interrupt
 * (0x0680), then retry the access.  A guest that retried at once would
 * fill up KVM's queue of async page faults and get the synchronous fault
 * path.  Runs with DAT off, so lowcore is addressed by real addresses:
 * 0x84 is the ext cpu address, 0x130 the ext old PSW.  Done differs from
 * init only in bit 0x80 of 0x85, test it like Linux tests PF_COMPLETE.
 */
extern char ext_int_handler[];
asm(".pushsection .text\n"
	"ext_int_handler:\n"
	"	tm	0x85,0x80\n"
	"	jo	1f\n"
	"	mvc	" __stringify(__LC_SAVED_PSW) "(16),0x130\n"
	"	lpswe	" __stringify(__LC_WAIT_PSW) "\n"
	"1:	lpswe	" __stringify(__LC_SAVED_PSW) "\n"
	".popsection\n");

static void arch_enable_host_async_pf(struct kvm_vm *vm, struct kvm_vcpu *vcpu)
{
	u64 *lowcore = addr_gpa2hva(vm, 0);
	struct kvm_sregs sregs;
	gva_t token_gva;
	gpa_t handler;
	int flic;

	flic = kvm_create_device(vm, KVM_DEV_TYPE_FLIC);
	kvm_device_attr_set(flic, KVM_DEV_FLIC_APF_ENABLE, 0, NULL);

	/* The DIAG 0x258 handshake, with masks that match any guest PSW. */
	token_gva = vm_alloc_page(vm);
	vcpu_set_reg(vcpu, KVM_REG_S390_PFTOKEN, addr_gva2gpa(vm, token_gva));
	vcpu_set_reg(vcpu, KVM_REG_S390_PFSELECT, 0);
	vcpu_set_reg(vcpu, KVM_REG_S390_PFCOMPARE, 0);

	vcpu_sregs_get(vcpu, &sregs);
	sregs.crs[0] |= CR0_SERVICE_SIGNAL_SUBMASK;
	vcpu_sregs_set(vcpu, &sregs);
	vcpu->run->psw_mask |= PSW_MASK_EXT;

	handler = addr_gva2gpa(vm, (gva_t)ext_int_handler);
	lowcore[__LC_EXT_NEW_PSW / 8] = vcpu->run->psw_mask &
					~(PSW_MASK_EXT | PSW_MASK_DAT);
	lowcore[__LC_EXT_NEW_PSW / 8 + 1] = handler;
	lowcore[__LC_WAIT_PSW / 8] = (vcpu->run->psw_mask & ~PSW_MASK_DAT) |
				     PSW_MASK_WAIT;
	lowcore[__LC_WAIT_PSW / 8 + 1] = handler;
}
#else
/* x86 needs guest interrupts on, see guest_code(); nothing to do elsewhere. */
static void arch_enable_host_async_pf(struct kvm_vm *vm, struct kvm_vcpu *vcpu)
{
}
#endif

static void guest_code(u64 gva, u64 size)
{
#ifdef __x86_64__
	/* KVM does async #PF, or its halt fallback, only with IRQs enabled. */
	sti_nop();
#endif
	/* Within EOF: must work, proves that the mapping is set up right. */
	WRITE_ONCE(*(u64 *)gva, 1);
	GUEST_SYNC(0);

	/* Beyond EOF: KVM_RUN must fail with -EFAULT. */
	WRITE_ONCE(*(u64 *)(gva + size - sizeof(u64)), 1);
	GUEST_FAIL("Store beyond EOF succeeded");
}

static void sig_alarm(int sig)
{
	TEST_FAIL("KVM_RUN did not return within %d seconds, the fault is retried forever",
		  TIMEOUT_SECS);
}

int main(void)
{
	struct sigaction sa = { .sa_handler = sig_alarm };
	struct userspace_mem_region *region;
	struct kvm_vcpu *vcpu;
	struct kvm_vm *vm;
	struct ucall uc;
	gpa_t gpa;
	gva_t gva;
	u64 npages;
	int r;

	vm = vm_create_with_one_vcpu(&vcpu, guest_code);

	/* After slot 0 and the page that some architectures use for ucalls. */
	region = memslot2region(vm, 0);
	gpa = align_up(region->region.guest_phys_addr +
		       region->region.memory_size + vm->page_size, TEST_SIZE);
	npages = TEST_SIZE / vm->page_size;

	vm_userspace_mem_region_add(vm, VM_MEM_SRC_SHMEM, gpa, TEST_SLOT,
				    npages, 0);
	region = memslot2region(vm, TEST_SLOT);
	/* Only the first host page is backed, everything behind it is beyond EOF. */
	TEST_ASSERT(!ftruncate(region->fd, getpagesize()), "ftruncate() failed");

	gva = vm_unused_gva_gap(vm, TEST_SIZE, KVM_UTIL_MIN_VADDR);
	virt_map(vm, gva, gpa, npages);
	vcpu_args_set(vcpu, 2, gva, TEST_SIZE);

	arch_enable_host_async_pf(vm, vcpu);

	vcpu_run(vcpu);
	switch (get_ucall(vcpu, &uc)) {
	case UCALL_SYNC:
		break;
	case UCALL_ABORT:
		REPORT_GUEST_ASSERT(uc);
		break;
	default:
		TEST_FAIL("Unexpected ucall %lu", uc.cmd);
	}
	TEST_ASSERT_EQ(*(u64 *)region->host_mem, 1);

	TEST_ASSERT(!sigaction(SIGALRM, &sa, NULL), "sigaction() failed");
	alarm(TIMEOUT_SECS);
	r = _vcpu_run(vcpu);
	alarm(0);

	TEST_ASSERT(r == -1 && errno == EFAULT,
		    "Expected KVM_RUN to fail with EFAULT, got r = %d, errno = %d (%s), exit reason %s",
		    r, errno, strerror(errno), exit_reason_str(vcpu->run->exit_reason));

	kvm_vm_free(vm);
	return 0;
}
