// SPDX-License-Identifier: GPL-2.0-only
/*
 * KSM functional tests
 *
 * Copyright 2022, Red Hat, Inc.
 *
 * Author(s): David Hildenbrand <david@redhat.com>
 */
#define _GNU_SOURCE
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <linux/userfaultfd.h>

#include "../kselftest.h"
#include "vm_util.h"

#define KiB 1024u
#define MiB (1024 * KiB)
#define FORK_EXEC_CHILD_PRG_NAME "ksm_fork_exec_child"

#define MAP_MERGE_FAIL ((void *)-1)
#define MAP_MERGE_SKIP ((void *)-2)

enum ksm_merge_mode {
	KSM_MERGE_PRCTL,
	KSM_MERGE_MADVISE,
	KSM_MERGE_NONE, /* PRCTL already set */
};

static int mem_fd;
static int pages_to_scan_fd;
static int sleep_millisecs_fd;
static int pagemap_fd;
static size_t pagesize;

static void init_global_file_handles(void);

static bool range_maps_duplicates(char *addr, unsigned long size)
{
	unsigned long offs_a, offs_b, pfn_a, pfn_b;

	/*
	 * There is no easy way to check if there are KSM pages mapped into
	 * this range. We only check that the range does not map the same PFN
	 * twice by comparing each pair of mapped pages.
	 */
	for (offs_a = 0; offs_a < size; offs_a += pagesize) {
		pfn_a = pagemap_get_pfn(pagemap_fd, addr + offs_a);
		/* Page not present or PFN not exposed by the kernel. */
		if (pfn_a == -1ul || !pfn_a)
			continue;

		for (offs_b = offs_a + pagesize; offs_b < size;
		     offs_b += pagesize) {
			pfn_b = pagemap_get_pfn(pagemap_fd, addr + offs_b);
			if (pfn_b == -1ul || !pfn_b)
				continue;
			if (pfn_a == pfn_b)
				return true;
		}
	}
	return false;
}

static char *__mmap_and_merge_range(char val, unsigned long size, int prot,
				  enum ksm_merge_mode mode)
{
	char *map;
	char *err_map = MAP_MERGE_FAIL;
	int ret;

	/* Stabilize accounting by disabling KSM completely. */
	if (ksm_stop() < 0) {
		ksft_print_msg("Disabling (unmerging) KSM failed\n");
		return err_map;
	}

	if (ksm_get_self_merging_pages() > 0) {
		ksft_print_msg("Still pages merged\n");
		return err_map;
	}

	map = mmap(NULL, size, PROT_READ|PROT_WRITE,
		   MAP_PRIVATE|MAP_ANON, -1, 0);
	if (map == MAP_FAILED) {
		ksft_print_msg("mmap() failed\n");
		return err_map;
	}

	/* Don't use THP. Ignore if THP are not around on a kernel. */
	if (madvise(map, size, MADV_NOHUGEPAGE) && errno != EINVAL) {
		ksft_print_msg("MADV_NOHUGEPAGE failed\n");
		goto unmap;
	}

	/* Make sure each page contains the same values to merge them. */
	memset(map, val, size);

	if (mprotect(map, size, prot)) {
		ksft_print_msg("mprotect() failed\n");
		err_map = MAP_MERGE_SKIP;
		goto unmap;
	}

	switch (mode) {
	case KSM_MERGE_PRCTL:
		ret = prctl(PR_SET_MEMORY_MERGE, 1, 0, 0, 0);
		if (ret < 0 && errno == EINVAL) {
			ksft_print_msg("PR_SET_MEMORY_MERGE not supported\n");
			err_map = MAP_MERGE_SKIP;
			goto unmap;
		} else if (ret) {
			ksft_print_msg("PR_SET_MEMORY_MERGE=1 failed\n");
			goto unmap;
		}
		break;
	case KSM_MERGE_MADVISE:
		if (madvise(map, size, MADV_MERGEABLE)) {
			ksft_print_msg("MADV_MERGEABLE failed\n");
			goto unmap;
		}
		break;
	case KSM_MERGE_NONE:
		break;
	}

	/* Run KSM to trigger merging and wait. */
	if (ksm_start() < 0) {
		ksft_print_msg("Running KSM failed\n");
		goto unmap;
	}

	/*
	 * Check if anything was merged at all. Ignore the zero page that is
	 * accounted differently (depending on kernel support).
	 */
	if (val && !ksm_get_self_merging_pages()) {
		ksft_print_msg("No pages got merged\n");
		goto unmap;
	}

	return map;
unmap:
	munmap(map, size);
	return err_map;
}

static char *mmap_and_merge_range(char val, unsigned long size, int prot,
				  enum ksm_merge_mode mode)
{
	char *map;
	char *ret = MAP_FAILED;

	map = __mmap_and_merge_range(val, size, prot, mode);
	if (map == MAP_MERGE_FAIL)
		ksft_test_result_fail("Merging memory failed");
	else if (map == MAP_MERGE_SKIP)
		ksft_test_result_skip("Merging memory skipped");
	else
		ret = map;

	return ret;
}

static void test_unmerge(void)
{
	const unsigned int size = 2 * MiB;
	char *map;

	ksft_print_msg("[RUN] %s\n", __func__);

	map = mmap_and_merge_range(0xcf, size, PROT_READ | PROT_WRITE, KSM_MERGE_MADVISE);
	if (map == MAP_FAILED)
		return;

	if (madvise(map, size, MADV_UNMERGEABLE)) {
		ksft_test_result_fail("MADV_UNMERGEABLE failed\n");
		goto unmap;
	}

	ksft_test_result(!range_maps_duplicates(map, size),
			 "Pages were unmerged\n");
unmap:
	ksm_stop();
	munmap(map, size);
}

static void test_unmerge_zero_pages(void)
{
	const unsigned int size = 2 * MiB;
	char *map;
	unsigned int offs;
	unsigned long pages_expected;

	ksft_print_msg("[RUN] %s\n", __func__);

	if (ksm_get_self_zero_pages() < 0) {
		ksft_test_result_skip("accessing \"/proc/self/ksm_stat\" failed\n");
		return;
	}

	if (ksm_use_zero_pages() < 0) {
		ksft_test_result_skip("write \"/sys/kernel/mm/ksm/use_zero_pages\" failed\n");
		return;
	}

	/* Let KSM deduplicate zero pages. */
	map = mmap_and_merge_range(0x00, size, PROT_READ | PROT_WRITE, KSM_MERGE_MADVISE);
	if (map == MAP_FAILED)
		return;

	/* Check if ksm_zero_pages is updated correctly after KSM merging */
	pages_expected = size / pagesize;
	if (pages_expected != ksm_get_self_zero_pages()) {
		ksft_test_result_fail("'ksm_zero_pages' updated after merging\n");
		goto unmap;
	}

	/* Try to unmerge half of the region */
	if (madvise(map, size / 2, MADV_UNMERGEABLE)) {
		ksft_test_result_fail("MADV_UNMERGEABLE failed\n");
		goto unmap;
	}

	/* Check if ksm_zero_pages is updated correctly after unmerging */
	pages_expected /= 2;
	if (pages_expected != ksm_get_self_zero_pages()) {
		ksft_test_result_fail("'ksm_zero_pages' updated after unmerging\n");
		goto unmap;
	}

	/* Trigger unmerging of the other half by writing to the pages. */
	for (offs = size / 2; offs < size; offs += pagesize)
		*((unsigned int *)&map[offs]) = offs;

	/* Now we should have no zeropages remaining. */
	if (ksm_get_self_zero_pages()) {
		ksft_test_result_fail("'ksm_zero_pages' updated after write fault\n");
		goto unmap;
	}

	/* Check if ksm zero pages are really unmerged */
	ksft_test_result(!range_maps_duplicates(map, size),
			"KSM zero pages were unmerged\n");
unmap:
	ksm_stop();
	munmap(map, size);
}

static void test_unmerge_discarded(void)
{
	const unsigned int size = 2 * MiB;
	char *map;

	ksft_print_msg("[RUN] %s\n", __func__);

	map = mmap_and_merge_range(0xcf, size, PROT_READ | PROT_WRITE, KSM_MERGE_MADVISE);
	if (map == MAP_FAILED)
		return;

	/* Discard half of all mapped pages so we have pte_none() entries. */
	if (madvise(map, size / 2, MADV_DONTNEED)) {
		ksft_test_result_fail("MADV_DONTNEED failed\n");
		goto unmap;
	}

	if (madvise(map, size, MADV_UNMERGEABLE)) {
		ksft_test_result_fail("MADV_UNMERGEABLE failed\n");
		goto unmap;
	}

	ksft_test_result(!range_maps_duplicates(map, size),
			 "Pages were unmerged\n");
unmap:
	ksm_stop();
	munmap(map, size);
}

#ifdef __NR_userfaultfd
static void test_unmerge_uffd_wp(void)
{
	struct uffdio_writeprotect uffd_writeprotect;
	const unsigned int size = 2 * MiB;
	struct uffdio_api uffdio_api;
	char *map;
	int uffd;

	ksft_print_msg("[RUN] %s\n", __func__);

	map = mmap_and_merge_range(0xcf, size, PROT_READ | PROT_WRITE, KSM_MERGE_MADVISE);
	if (map == MAP_FAILED)
		return;

	/* See if UFFD is around. */
	uffd = syscall(__NR_userfaultfd, O_CLOEXEC | O_NONBLOCK);
	if (uffd < 0) {
		ksft_test_result_skip("__NR_userfaultfd failed\n");
		goto unmap;
	}

	/* See if UFFD-WP is around. */
	uffdio_api.api = UFFD_API;
	uffdio_api.features = 0;
	if (ioctl(uffd, UFFDIO_API, &uffdio_api) < 0) {
		if (errno == EINVAL)
			ksft_test_result_skip("The API version requested is not supported\n");
		else
			ksft_test_result_fail("UFFDIO_API failed: %s\n", strerror(errno));

		goto close_uffd;
	}
	if (!(uffdio_api.features & UFFD_FEATURE_PAGEFAULT_FLAG_WP)) {
		ksft_test_result_skip("UFFD_FEATURE_PAGEFAULT_FLAG_WP not available\n");
		goto close_uffd;
	}

	/*
	 * UFFDIO_API must only be called once to enable features.
	 * So we close the old userfaultfd and create a new one to
	 * actually enable UFFD_FEATURE_PAGEFAULT_FLAG_WP.
	 */
	close(uffd);
	uffd = syscall(__NR_userfaultfd, O_CLOEXEC | O_NONBLOCK);
	if (uffd < 0) {
		ksft_test_result_fail("__NR_userfaultfd failed\n");
		goto unmap;
	}

	/* Now, enable it ("two-step handshake") */
	uffdio_api.api = UFFD_API;
	uffdio_api.features = UFFD_FEATURE_PAGEFAULT_FLAG_WP;
	if (ioctl(uffd, UFFDIO_API, &uffdio_api) < 0) {
		ksft_test_result_fail("UFFDIO_API failed: %s\n", strerror(errno));
		goto close_uffd;
	}

	/* Register UFFD-WP, no need for an actual handler. */
	if (uffd_register(uffd, map, size, false, true, false)) {
		ksft_test_result_fail("UFFDIO_REGISTER_MODE_WP failed\n");
		goto close_uffd;
	}

	/* Write-protect the range using UFFD-WP. */
	uffd_writeprotect.range.start = (unsigned long) map;
	uffd_writeprotect.range.len = size;
	uffd_writeprotect.mode = UFFDIO_WRITEPROTECT_MODE_WP;
	if (ioctl(uffd, UFFDIO_WRITEPROTECT, &uffd_writeprotect)) {
		ksft_test_result_fail("UFFDIO_WRITEPROTECT failed\n");
		goto close_uffd;
	}

	if (madvise(map, size, MADV_UNMERGEABLE)) {
		ksft_test_result_fail("MADV_UNMERGEABLE failed\n");
		goto close_uffd;
	}

	ksft_test_result(!range_maps_duplicates(map, size),
			 "Pages were unmerged\n");
close_uffd:
	close(uffd);
unmap:
	ksm_stop();
	munmap(map, size);
}
#endif

/* Verify that KSM can be enabled / queried with prctl. */
static void test_prctl(void)
{
	int ret;

	ksft_print_msg("[RUN] %s\n", __func__);

	ret = prctl(PR_SET_MEMORY_MERGE, 1, 0, 0, 0);
	if (ret < 0 && errno == EINVAL) {
		ksft_test_result_skip("PR_SET_MEMORY_MERGE not supported\n");
		return;
	} else if (ret) {
		ksft_test_result_fail("PR_SET_MEMORY_MERGE=1 failed\n");
		return;
	}

	ret = prctl(PR_GET_MEMORY_MERGE, 0, 0, 0, 0);
	if (ret < 0) {
		ksft_test_result_fail("PR_GET_MEMORY_MERGE failed\n");
		return;
	} else if (ret != 1) {
		ksft_test_result_fail("PR_SET_MEMORY_MERGE=1 not effective\n");
		return;
	}

	ret = prctl(PR_SET_MEMORY_MERGE, 0, 0, 0, 0);
	if (ret) {
		ksft_test_result_fail("PR_SET_MEMORY_MERGE=0 failed\n");
		return;
	}

	ret = prctl(PR_GET_MEMORY_MERGE, 0, 0, 0, 0);
	if (ret < 0) {
		ksft_test_result_fail("PR_GET_MEMORY_MERGE failed\n");
		return;
	} else if (ret != 0) {
		ksft_test_result_fail("PR_SET_MEMORY_MERGE=0 not effective\n");
		return;
	}

	ksft_test_result_pass("Setting/clearing PR_SET_MEMORY_MERGE works\n");
}

static int test_child_ksm(void)
{
	const unsigned int size = 2 * MiB;
	char *map;

	/* Test if KSM is enabled for the process. */
	if (prctl(PR_GET_MEMORY_MERGE, 0, 0, 0, 0) != 1)
		return 1;

	/* Test if merge could really happen. */
	map = __mmap_and_merge_range(0xcf, size, PROT_READ | PROT_WRITE, KSM_MERGE_NONE);
	if (map == MAP_MERGE_FAIL)
		return 2;
	else if (map == MAP_MERGE_SKIP)
		return 3;

	ksm_stop();
	munmap(map, size);
	return 0;
}

static void test_child_ksm_err(int status)
{
	if (status == 1)
		ksft_test_result_fail("unexpected PR_GET_MEMORY_MERGE result in child\n");
	else if (status == 2)
		ksft_test_result_fail("Merge in child failed\n");
	else if (status == 3)
		ksft_test_result_skip("Merge in child skipped\n");
	else if (status == 4)
		ksft_test_result_fail("Binary not found\n");
}

/* Verify that prctl ksm flag is inherited. */
static void test_prctl_fork(void)
{
	int ret, status;
	pid_t child_pid;

	ksft_print_msg("[RUN] %s\n", __func__);

	ret = prctl(PR_SET_MEMORY_MERGE, 1, 0, 0, 0);
	if (ret < 0 && errno == EINVAL) {
		ksft_test_result_skip("PR_SET_MEMORY_MERGE not supported\n");
		return;
	} else if (ret) {
		ksft_test_result_fail("PR_SET_MEMORY_MERGE=1 failed\n");
		return;
	}

	child_pid = fork();
	if (!child_pid) {
		init_global_file_handles();
		exit(test_child_ksm());
	} else if (child_pid < 0) {
		ksft_test_result_fail("fork() failed\n");
		return;
	}

	if (waitpid(child_pid, &status, 0) < 0) {
		ksft_test_result_fail("waitpid() failed\n");
		return;
	}

	status = WEXITSTATUS(status);
	if (status) {
		test_child_ksm_err(status);
		return;
	}

	if (prctl(PR_SET_MEMORY_MERGE, 0, 0, 0, 0)) {
		ksft_test_result_fail("PR_SET_MEMORY_MERGE=0 failed\n");
		return;
	}

	ksft_test_result_pass("PR_SET_MEMORY_MERGE value is inherited\n");
}

static int start_ksmd_and_set_frequency(char *pages_to_scan, char *sleep_ms)
{
	int ksm_fd;

	ksm_fd = open("/sys/kernel/mm/ksm/run", O_RDWR);
	if (ksm_fd < 0)
		return -errno;

	if (write(ksm_fd, "1", 1) != 1)
		return -errno;

	if (write(pages_to_scan_fd, pages_to_scan, strlen(pages_to_scan)) <= 0)
		return -errno;

	if (write(sleep_millisecs_fd, sleep_ms, strlen(sleep_ms)) <= 0)
		return -errno;

	return 0;
}

static int stop_ksmd_and_restore_frequency(void)
{
	int ksm_fd;

	ksm_fd = open("/sys/kernel/mm/ksm/run", O_RDWR);
	if (ksm_fd < 0)
		return -errno;

	if (write(ksm_fd, "2", 1) != 1)
		return -errno;

	if (write(pages_to_scan_fd, "100", 3) <= 0)
		return -errno;

	if (write(sleep_millisecs_fd, "20", 2) <= 0)
		return -errno;

	return 0;
}

static void test_prctl_fork_exec(void)
{
	int ret, status;
	pid_t child_pid;

	ksft_print_msg("[RUN] %s\n", __func__);

	if (start_ksmd_and_set_frequency("2000", "0"))
		ksft_test_result_fail("set ksmd's scanning frequency failed\n");

	ret = prctl(PR_SET_MEMORY_MERGE, 1, 0, 0, 0);
	if (ret < 0 && errno == EINVAL) {
		ksft_test_result_skip("PR_SET_MEMORY_MERGE not supported\n");
		return;
	} else if (ret) {
		ksft_test_result_fail("PR_SET_MEMORY_MERGE=1 failed\n");
		return;
	}

	child_pid = fork();
	if (child_pid == -1) {
		ksft_test_result_skip("fork() failed\n");
		return;
	} else if (child_pid == 0) {
		char *prg_name = "./ksm_functional_tests";
		char *argv_for_program[] = { prg_name, FORK_EXEC_CHILD_PRG_NAME, NULL };

		execv(prg_name, argv_for_program);
		exit(4);
	}

	if (waitpid(child_pid, &status, 0) > 0) {
		if (WIFEXITED(status)) {
			status = WEXITSTATUS(status);
			if (status) {
				test_child_ksm_err(status);
				return;
			}
		} else {
			ksft_test_result_fail("program didn't terminate normally\n");
			return;
		}
	} else {
		ksft_test_result_fail("waitpid() failed\n");
		return;
	}

	if (prctl(PR_SET_MEMORY_MERGE, 0, 0, 0, 0)) {
		ksft_test_result_fail("PR_SET_MEMORY_MERGE=0 failed\n");
		return;
	}

	if (stop_ksmd_and_restore_frequency()) {
		ksft_test_result_fail("restore ksmd frequency failed\n");
		return;
	}

	ksft_test_result_pass("PR_SET_MEMORY_MERGE value is inherited\n");
}

static void test_prctl_unmerge(void)
{
	const unsigned int size = 2 * MiB;
	char *map;

	ksft_print_msg("[RUN] %s\n", __func__);

	map = mmap_and_merge_range(0xcf, size, PROT_READ | PROT_WRITE, KSM_MERGE_PRCTL);
	if (map == MAP_FAILED)
		return;

	if (prctl(PR_SET_MEMORY_MERGE, 0, 0, 0, 0)) {
		ksft_test_result_fail("PR_SET_MEMORY_MERGE=0 failed\n");
		goto unmap;
	}

	ksft_test_result(!range_maps_duplicates(map, size),
			 "Pages were unmerged\n");
unmap:
	ksm_stop();
	munmap(map, size);
}

static void test_prot_none(void)
{
	const unsigned int size = 2 * MiB;
	char *map;
	int i;

	ksft_print_msg("[RUN] %s\n", __func__);

	map = mmap_and_merge_range(0x11, size, PROT_NONE, KSM_MERGE_MADVISE);
	if (map == MAP_FAILED)
		goto unmap;

	/* Store a unique value in each page on one half using ptrace */
	for (i = 0; i < size / 2; i += pagesize) {
		lseek(mem_fd, (uintptr_t) map + i, SEEK_SET);
		if (write(mem_fd, &i, sizeof(i)) != sizeof(i)) {
			ksft_test_result_fail("ptrace write failed\n");
			goto unmap;
		}
	}

	/* Trigger unsharing on the other half. */
	if (madvise(map + size / 2, size / 2, MADV_UNMERGEABLE)) {
		ksft_test_result_fail("MADV_UNMERGEABLE failed\n");
		goto unmap;
	}

	ksft_test_result(!range_maps_duplicates(map, size),
			 "Pages were unmerged\n");
unmap:
	ksm_stop();
	munmap(map, size);
}

static void test_fork_ksm_merging_page_count(void)
{
	const unsigned int size = 2 * MiB;
	char *map;
	pid_t child_pid;
	int status;

	ksft_print_msg("[RUN] %s\n", __func__);

	map = mmap_and_merge_range(0xcf, size, PROT_READ | PROT_WRITE, KSM_MERGE_MADVISE);
	if (map == MAP_FAILED)
		return;

	child_pid = fork();
	if (!child_pid) {
		init_global_file_handles();
		exit(ksm_get_self_merging_pages());
	} else if (child_pid < 0) {
		ksft_test_result_fail("fork() failed\n");
		goto unmap;
	}

	if (waitpid(child_pid, &status, 0) < 0) {
		ksft_test_result_fail("waitpid() failed\n");
		goto unmap;
	}

	status = WEXITSTATUS(status);
	if (status) {
		ksft_test_result_fail("ksm_merging_page in child: %d\n", status);
		goto unmap;
	}

	ksft_test_result_pass("ksm_merging_pages is not inherited after fork\n");

unmap:
	ksm_stop();
	munmap(map, size);
}

static void init_global_file_handles(void)
{
	mem_fd = open("/proc/self/mem", O_RDWR);
	if (mem_fd < 0)
		ksft_exit_fail_msg("opening /proc/self/mem failed\n");
	if (ksm_stop() < 0)
		ksft_exit_skip("accessing \"/sys/kernel/mm/ksm/run\") failed\n");
	if (ksm_get_full_scans() < 0)
		ksft_exit_skip("accessing \"/sys/kernel/mm/ksm/full_scans\") failed\n");
	pagemap_fd = open("/proc/self/pagemap", O_RDONLY);
	if (pagemap_fd < 0)
		ksft_exit_skip("open(\"/proc/self/pagemap\") failed\n");
	if (ksm_get_self_merging_pages() < 0)
		ksft_exit_skip("accessing \"/proc/self/ksm_merging_pages\") failed\n");

	pages_to_scan_fd = open("/sys/kernel/mm/ksm/pages_to_scan", O_RDWR);
	if (pages_to_scan_fd < 0)
		ksft_exit_fail_msg("opening /sys/kernel/mm/ksm/pages_to_scan failed\n");
	sleep_millisecs_fd = open("/sys/kernel/mm/ksm/sleep_millisecs", O_RDWR);
	if (sleep_millisecs_fd < 0)
		ksft_exit_fail_msg("opening /sys/kernel/mm/ksm/sleep_millisecs failed\n");
}

int main(int argc, char **argv)
{
	unsigned int tests = 9;
	int err;

	if (argc > 1 && !strcmp(argv[1], FORK_EXEC_CHILD_PRG_NAME)) {
		init_global_file_handles();
		exit(test_child_ksm());
	}

#ifdef __NR_userfaultfd
	tests++;
#endif

	ksft_print_header();
	ksft_set_plan(tests);

	pagesize = getpagesize();

	init_global_file_handles();

	test_unmerge();
	test_unmerge_zero_pages();
	test_unmerge_discarded();
#ifdef __NR_userfaultfd
	test_unmerge_uffd_wp();
#endif

	test_prot_none();

	test_prctl();
	test_prctl_fork();
	test_prctl_fork_exec();
	test_prctl_unmerge();
	test_fork_ksm_merging_page_count();

	err = ksft_get_fail_cnt();
	if (err)
		ksft_exit_fail_msg("%d out of %d tests failed\n",
				   err, ksft_test_num());
	ksft_exit_pass();
}
