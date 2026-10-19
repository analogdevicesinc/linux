// SPDX-License-Identifier: GPL-2.0
/*
 * MADV_COLD and MADV_PAGEOUT tests.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <stdbool.h>
#include <sys/mman.h>
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

FIXTURE(madv_cold_pageout)
{
	int pagemap_fd;
};

FIXTURE_SETUP(madv_cold_pageout)
{
	self->pagemap_fd = open("/proc/self/pagemap", O_RDONLY);
	ASSERT_GE(self->pagemap_fd, 0);
}

FIXTURE_TEARDOWN(madv_cold_pageout)
{
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
