// SPDX-License-Identifier: GPL-2.0
/*
 * MADV_COLD and MADV_PAGEOUT tests.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <unistd.h>

#include "kselftest_harness.h"
#include "vm_util.h"

/* Reclaim can fail transiently, so retry for up to a second. */
#define PAGEOUT_RETRIES 100

static size_t page_size;
/* Whether MADV_PAGEOUT can swap out anonymous memory, see main(). */
static bool can_swap;

/*
 * Keep this task on one CPU, so that draining that CPU's LRU batches drains
 * the folios this task faulted in.
 */
static void pin_to_current_cpu(void)
{
	int cpu = sched_getcpu();
	cpu_set_t *mask;
	size_t size;

	if (cpu < 0)
		ksft_exit_fail_perror("sched_getcpu");
	/* CPU numbers can exceed CPU_SETSIZE, so size the mask to fit. */
	mask = CPU_ALLOC(cpu + 1);
	if (!mask)
		ksft_exit_fail_perror("CPU_ALLOC");
	size = CPU_ALLOC_SIZE(cpu + 1);
	CPU_ZERO_S(size, mask);
	CPU_SET_S(cpu, size, mask);
	if (sched_setaffinity(0, size, mask))
		ksft_exit_fail_perror("sched_setaffinity");
	CPU_FREE(mask);
}

/* Whether pagemap shows PFNs, which it only does with CAP_SYS_ADMIN. */
static bool pagemap_shows_pfns(struct __test_metadata *_metadata,
			       int pagemap_fd)
{
	unsigned long pfn;
	char *page;

	page = mmap(NULL, page_size, PROT_READ | PROT_WRITE,
		    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	ASSERT_NE(page, MAP_FAILED);
	page[0] = 1;
	pfn = pagemap_get_pfn(pagemap_fd, page);
	ASSERT_EQ(munmap(page, page_size), 0);

	return pfn && pfn != -1UL;
}

FIXTURE(madv_cold_pageout)
{
	int pagemap_fd;
	/* Needs root, and is -1 unless pagemap also shows PFNs. */
	int kpageflags_fd;
};

FIXTURE_SETUP(madv_cold_pageout)
{
	self->pagemap_fd = open("/proc/self/pagemap", O_RDONLY);
	ASSERT_GE(self->pagemap_fd, 0);
	self->kpageflags_fd = open("/proc/kpageflags", O_RDONLY);
	/* Page flags are no use without the PFNs to look them up by. */
	if (self->kpageflags_fd >= 0 &&
	    !pagemap_shows_pfns(_metadata, self->pagemap_fd)) {
		close(self->kpageflags_fd);
		self->kpageflags_fd = -1;
	}
}

FIXTURE_TEARDOWN(madv_cold_pageout)
{
	if (self->kpageflags_fd >= 0)
		close(self->kpageflags_fd);
	close(self->pagemap_fd);
}

/* Page out the range until its first page is swapped, or madvise() fails. */
static bool pageout_until_swapped(char *mapping, size_t size, int pagemap_fd)
{
	int retry;

	for (retry = 0; retry < PAGEOUT_RETRIES; retry++) {
		if (madvise(mapping, size, MADV_PAGEOUT))
			return false;
		if (pagemap_is_swapped(pagemap_fd, mapping))
			return true;
		usleep(10000);
	}

	return false;
}

/* Page out the range until its first page is unmapped, or madvise() fails. */
static bool pageout_until_unmapped(char *mapping, size_t size, int pagemap_fd)
{
	int retry;

	for (retry = 0; retry < PAGEOUT_RETRIES; retry++) {
		if (madvise(mapping, size, MADV_PAGEOUT))
			return false;
		if (!pagemap_is_populated(pagemap_fd, mapping))
			return true;
		usleep(10000);
	}

	return false;
}

/* Create an unlinked, zero-filled temporary file, or return -1. */
static int create_temp_file(size_t size)
{
	char template[] = "/tmp/madv_cold_pageout-XXXXXX";
	int fd;

	fd = mkstemp(template);
	if (fd < 0)
		return -1;
	unlink(template);
	if (ftruncate(fd, size)) {
		close(fd);
		return -1;
	}

	return fd;
}

/* Map an unlinked, zero-filled temporary file with the given permissions. */
static char *map_temp_file(struct __test_metadata *_metadata, size_t size,
			   int flags, mode_t mode)
{
	char *mapping;
	int fd;

	fd = create_temp_file(size);
	ASSERT_GE(fd, 0);
	ASSERT_EQ(fchmod(fd, mode), 0);

	mapping = mmap(NULL, size, PROT_READ | PROT_WRITE, flags, fd, 0);
	ASSERT_NE(mapping, MAP_FAILED);
	close(fd);

	return mapping;
}

/*
 * Both hints are best effort, so these tests check what userspace can rely
 * on: which mappings are accepted, which pages must never be reclaimed, and
 * that neither hint changes memory contents or populates the range. main()
 * checks once whether reclaim works at all. A test that needs a page to be
 * reclaimed skips if it does not, and fails if the page is then not reclaimed.
 */

/* Neither hint may populate a hole between two populated pages. */
static void check_skips_pte_hole(struct __test_metadata *_metadata,
				 int pagemap_fd, int advice)
{
	char *mapping;

	mapping = mmap(NULL, 3 * page_size, PROT_READ | PROT_WRITE,
		       MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	ASSERT_NE(mapping, MAP_FAILED);
	/*
	 * Keep a large folio from covering the hole. Without THP there are no
	 * large anonymous folios, and this fails harmlessly.
	 */
	madvise(mapping, 3 * page_size, MADV_NOHUGEPAGE);
	mapping[0] = 1;
	mapping[2 * page_size] = 1;
	ASSERT_FALSE(pagemap_is_populated(pagemap_fd, mapping + page_size));

	ASSERT_EQ(madvise(mapping, 3 * page_size, advice), 0);
	ASSERT_FALSE(pagemap_is_populated(pagemap_fd, mapping + page_size));
	ASSERT_EQ(mapping[0], 1);
	ASSERT_EQ(mapping[2 * page_size], 1);

	ASSERT_EQ(munmap(mapping, 3 * page_size), 0);
}

TEST_F(madv_cold_pageout, cold_skips_pte_hole)
{
	check_skips_pte_hole(_metadata, self->pagemap_fd, MADV_COLD);
}

TEST_F(madv_cold_pageout, pageout_skips_pte_hole)
{
	check_skips_pte_hole(_metadata, self->pagemap_fd, MADV_PAGEOUT);
}

/* Neither hint may be applied to locked memory. */
TEST_F(madv_cold_pageout, locked_vma_is_rejected)
{
	char *mapping;

	mapping = mmap(NULL, page_size, PROT_READ | PROT_WRITE,
		       MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	ASSERT_NE(mapping, MAP_FAILED);
	mapping[0] = 1;
	if (mlock(mapping, page_size))
		SKIP(return, "could not lock a page");

	ASSERT_EQ(madvise(mapping, page_size, MADV_COLD), -1);
	ASSERT_EQ(errno, EINVAL);
	ASSERT_EQ(madvise(mapping, page_size, MADV_PAGEOUT), -1);
	ASSERT_EQ(errno, EINVAL);
	ASSERT_EQ(mapping[0], 1);

	ASSERT_EQ(munmap(mapping, page_size), 0);
}

/* Neither hint may be applied to hugetlb memory. */
TEST_F(madv_cold_pageout, hugetlb_vma_is_rejected)
{
	size_t size = default_huge_page_size();
	char *mapping;

	if (!size)
		SKIP(return, "hugetlb is not supported");
	mapping = mmap(NULL, size, PROT_READ | PROT_WRITE,
		       MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB, -1, 0);
	if (mapping == MAP_FAILED)
		SKIP(return, "no hugetlb pages are available");
	mapping[0] = 1;

	ASSERT_EQ(madvise(mapping, size, MADV_COLD), -1);
	ASSERT_EQ(errno, EINVAL);
	ASSERT_EQ(madvise(mapping, size, MADV_PAGEOUT), -1);
	ASSERT_EQ(errno, EINVAL);
	ASSERT_EQ(mapping[0], 1);

	ASSERT_EQ(munmap(mapping, size), 0);
}

/* A zero-length range is accepted. */
TEST_F(madv_cold_pageout, zero_length_is_accepted)
{
	char *mapping;

	mapping = mmap(NULL, page_size, PROT_READ | PROT_WRITE,
		       MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	ASSERT_NE(mapping, MAP_FAILED);

	ASSERT_EQ(madvise(mapping, 0, MADV_COLD), 0);
	ASSERT_EQ(madvise(mapping, 0, MADV_PAGEOUT), 0);

	ASSERT_EQ(munmap(mapping, page_size), 0);
}

/* The start of the range must be page-aligned. */
TEST_F(madv_cold_pageout, unaligned_start_is_rejected)
{
	char *mapping;

	mapping = mmap(NULL, 2 * page_size, PROT_READ | PROT_WRITE,
		       MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	ASSERT_NE(mapping, MAP_FAILED);
	mapping[0] = 1;

	ASSERT_EQ(madvise(mapping + 1, page_size, MADV_COLD), -1);
	ASSERT_EQ(errno, EINVAL);
	ASSERT_EQ(madvise(mapping + 1, page_size, MADV_PAGEOUT), -1);
	ASSERT_EQ(errno, EINVAL);
	ASSERT_EQ(mapping[0], 1);

	ASSERT_EQ(munmap(mapping, 2 * page_size), 0);
}

/*
 * An unmapped gap in the range makes either hint fail with ENOMEM, after
 * advising the mapped pages on both sides of the gap. Only MADV_PAGEOUT has
 * an effect on those pages that the test can observe.
 */
TEST_F(madv_cold_pageout, unmapped_gap_fails_after_advising)
{
	bool swapped = false;
	char *mapping;
	int retry;

	mapping = mmap(NULL, 3 * page_size, PROT_READ | PROT_WRITE,
		       MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	ASSERT_NE(mapping, MAP_FAILED);
	mapping[0] = 1;
	mapping[2 * page_size] = 1;
	ASSERT_EQ(munmap(mapping + page_size, page_size), 0);

	ASSERT_EQ(madvise(mapping, 3 * page_size, MADV_COLD), -1);
	ASSERT_EQ(errno, ENOMEM);
	ASSERT_EQ(madvise(mapping, 3 * page_size, MADV_PAGEOUT), -1);
	ASSERT_EQ(errno, ENOMEM);
	ASSERT_EQ(mapping[0], 1);
	ASSERT_EQ(mapping[2 * page_size], 1);

	if (!can_swap)
		SKIP(return, "MADV_PAGEOUT cannot swap");

	for (retry = 0; retry < PAGEOUT_RETRIES && !swapped; retry++) {
		ASSERT_EQ(madvise(mapping, 3 * page_size, MADV_PAGEOUT), -1);
		ASSERT_EQ(errno, ENOMEM);
		swapped = pagemap_is_swapped(self->pagemap_fd, mapping) &&
			  pagemap_is_swapped(self->pagemap_fd,
					     mapping + 2 * page_size);
		if (!swapped)
			usleep(10000);
	}
	ASSERT_TRUE(swapped)
		TH_LOG("MADV_PAGEOUT did not page out both sides of the gap");
	ASSERT_EQ(mapping[0], 1);
	ASSERT_EQ(mapping[2 * page_size], 1);

	ASSERT_EQ(munmap(mapping, page_size), 0);
	ASSERT_EQ(munmap(mapping + 2 * page_size, page_size), 0);
}

/* Neither hint may fault a swapped page back in. */
static void check_skips_swapped_pte(struct __test_metadata *_metadata,
				    int pagemap_fd, int advice)
{
	char *mapping;

	if (!can_swap)
		SKIP(return, "MADV_PAGEOUT cannot swap");
	mapping = mmap(NULL, page_size, PROT_READ | PROT_WRITE,
		       MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	ASSERT_NE(mapping, MAP_FAILED);
	mapping[0] = 1;
	ASSERT_TRUE(pageout_until_swapped(mapping, page_size, pagemap_fd));

	ASSERT_EQ(madvise(mapping, page_size, advice), 0);
	ASSERT_TRUE(pagemap_is_swapped(pagemap_fd, mapping));
	ASSERT_EQ(mapping[0], 1);

	ASSERT_EQ(munmap(mapping, page_size), 0);
}

TEST_F(madv_cold_pageout, cold_skips_swapped_pte)
{
	check_skips_swapped_pte(_metadata, self->pagemap_fd, MADV_COLD);
}

TEST_F(madv_cold_pageout, pageout_skips_swapped_pte)
{
	check_skips_swapped_pte(_metadata, self->pagemap_fd, MADV_PAGEOUT);
}

/* MADV_COLD must not discard a dirty page of a shared file mapping. */
TEST_F(madv_cold_pageout, cold_preserves_dirty_shared_file_page)
{
	char *mapping;

	mapping = map_temp_file(_metadata, page_size, MAP_SHARED, 0600);
	mapping[0] = 1;

	ASSERT_EQ(madvise(mapping, page_size, MADV_COLD), 0);
	ASSERT_EQ(mapping[0], 1);

	ASSERT_EQ(munmap(mapping, page_size), 0);
}

/*
 * MADV_PAGEOUT may write a dirty shared file page back or leave it resident,
 * but must not discard it.
 */
TEST_F(madv_cold_pageout, pageout_preserves_dirty_shared_file_page)
{
	char *mapping;

	mapping = map_temp_file(_metadata, page_size, MAP_SHARED, 0600);
	mapping[0] = 1;

	if (!pageout_until_unmapped(mapping, page_size, self->pagemap_fd))
		SKIP(return, "MADV_PAGEOUT did not unmap the page");
	ASSERT_EQ(mapping[0], 1);

	ASSERT_EQ(munmap(mapping, page_size), 0);
}

/* MADV_PAGEOUT must not lose the contents of shared anonymous memory. */
TEST_F(madv_cold_pageout, pageout_preserves_shared_anon_page)
{
	char *mapping;

	mapping = mmap(NULL, page_size, PROT_READ | PROT_WRITE,
		       MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	ASSERT_NE(mapping, MAP_FAILED);
	mapping[0] = 1;

	/* MADV_PAGEOUT unmaps a shmem page even when it cannot swap it out. */
	ASSERT_TRUE(pageout_until_unmapped(mapping, page_size,
					   self->pagemap_fd));
	ASSERT_EQ(mapping[0], 1);

	ASSERT_EQ(munmap(mapping, page_size), 0);
}

/*
 * SHM_LOCK makes shmem pages unevictable without setting VM_LOCKED on the
 * VMA, so MADV_PAGEOUT accepts the range but must not reclaim the page.
 */
TEST_F(madv_cold_pageout, pageout_skips_unevictable_page)
{
	unsigned long pfn;
	uint64_t flags;
	char *mapping;
	int shmid;

	if (self->kpageflags_fd < 0)
		SKIP(return, "cannot read page flags");
	shmid = shmget(IPC_PRIVATE, page_size, IPC_CREAT | 0600);
	ASSERT_GE(shmid, 0);
	if (shmctl(shmid, SHM_LOCK, NULL)) {
		shmctl(shmid, IPC_RMID, NULL);
		SKIP(return, "could not lock a shmem segment");
	}
	mapping = shmat(shmid, NULL, 0);
	/* The segment is destroyed once it is detached. */
	shmctl(shmid, IPC_RMID, NULL);
	ASSERT_NE(mapping, (void *)-1);
	mapping[0] = 1;

	/*
	 * MADV_COLD drains this CPU's LRU add batch, which makes the page
	 * unevictable.
	 */
	ASSERT_EQ(madvise(mapping, page_size, MADV_COLD), 0);
	pfn = pagemap_get_pfn(self->pagemap_fd, mapping);
	ASSERT_NE(pfn, -1UL);
	ASSERT_EQ(pageflags_get(pfn, self->kpageflags_fd, &flags), 0);
	if (!(flags & KPF_UNEVICTABLE))
		SKIP(return, "could not make the page unevictable");

	ASSERT_EQ(madvise(mapping, page_size, MADV_PAGEOUT), 0);
	ASSERT_TRUE(pagemap_is_populated(self->pagemap_fd, mapping));
	ASSERT_EQ(mapping[0], 1);

	ASSERT_EQ(shmdt(mapping), 0);
}

/* Swap out an anonymous page, to see whether swap works at all. */
static bool probe_swap(int pagemap_fd)
{
	bool swapped;
	char *page;

	page = mmap(NULL, page_size, PROT_READ | PROT_WRITE,
		    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (page == MAP_FAILED)
		ksft_exit_fail_perror("mmap");
	page[0] = 1;
	swapped = pageout_until_swapped(page, page_size, pagemap_fd);
	munmap(page, page_size);

	return swapped;
}

int main(int argc, char **argv)
{
	int pagemap_fd;

	page_size = getpagesize();
	/* The harness forks each test, which inherits this. */
	pin_to_current_cpu();
	/* Only root can reserve the page that hugetlb_vma_is_rejected maps. */
	if (!geteuid())
		hugetlb_setup_default(1);

	pagemap_fd = open("/proc/self/pagemap", O_RDONLY);
	if (pagemap_fd < 0)
		ksft_exit_fail_perror("open /proc/self/pagemap");
	can_swap = probe_swap(pagemap_fd);
	if (!can_swap)
		ksft_print_msg("MADV_PAGEOUT cannot swap\n");
	close(pagemap_fd);

	return test_harness_run(argc, argv);
}
