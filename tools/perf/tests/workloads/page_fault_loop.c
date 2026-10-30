// SPDX-License-Identifier: GPL-2.0
#include <linux/compiler.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>
#include "../tests.h"

static int page_fault_loop(int argc __maybe_unused, const char **argv __maybe_unused)
{
	size_t size = getpagesize();
	char *page;
	int ret = 1;

	page = mmap(NULL, size, PROT_READ, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (page == MAP_FAILED) {
		perror("mmap");
		return 1;
	}

	for (int i = 0; i < 256; i++) {
		/* Drop the PTE so the kernel must resolve a fault on each read. */
		if (madvise(page, size, MADV_DONTNEED)) {
			perror("madvise");
			goto out;
		}
		if (READ_ONCE(*page)) {
			fprintf(stderr, "Anonymous page is not zero-filled\n");
			goto out;
		}
	}
	ret = 0;
out:
	munmap(page, size);
	return ret;
}

DEFINE_WORKLOAD(page_fault_loop);
