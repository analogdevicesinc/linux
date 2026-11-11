// SPDX-License-Identifier: GPL-2.0
/*
 * hwpoison-panic: trigger an intentional kernel panic by injecting a
 * hwpoison error on a kernel-owned page, to check that
 * vm.panic_on_unrecoverable_memory_failure fires.
 *
 * Three kinds of kernel-owned page can be targeted, selected with -k
 * (default: rodata):
 *
 *   rodata  - a PG_reserved page in the kernel rodata range, from
 *             /proc/iomem "Kernel rodata".  x86 and riscv only.
 *   slab    - a slab page found via /proc/kpageflags (KPF_SLAB).
 *   pgtable - a page-table page found via /proc/kpageflags (KPF_PGTABLE).
 *
 * slab and pgtable work everywhere; rodata needs an architecture that
 * publishes the rodata resource in /proc/iomem.
 *
 * The slab and pgtable variants exercise memory_failure() -> get_any_page()
 * on a non PG_reserved kernel-owned page, catching regressions where
 * get_any_page() collapses such pages into a transient -EIO instead of
 * -ENOTRECOVERABLE.
 *
 * A successful run crashes the kernel, so --yes-panic-my-kernel is required
 * and the tool is meant for a disposable VM (e.g. virtme-ng) with the serial
 * console captured.  The result is observed externally: the kernel panics with
 *   "Memory failure: <pfn>: unrecoverable page"
 * Returning at all means no panic fired, so every exit status is non-zero:
 * either a failure, or an inconclusive run where the target PFN raced to
 * another page type before injection.
 *
 * Author: Breno Leitao <leitao@debian.org>
 */
#define _GNU_SOURCE
#define _FILE_OFFSET_BITS 64
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../../../include/uapi/linux/kernel-page-flags.h"
#include "../../include/linux/sizes.h"
#include <api/fs/fs.h>

#define SYSCTL_PATH	"/proc/sys/vm/panic_on_unrecoverable_memory_failure"
#define INJECT_PATH	"/sys/devices/system/memory/hard_offline_page"
#define PROC_KPAGEFLAGS	"/proc/kpageflags"

/* Not exported by the uapi header, see tools/mm/page-types.c. */
#define KPF_RESERVED	32

#define BIT(name)	(1ULL << KPF_##name)
#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))

/* The kpageflags bit identifying each kernel-owned page kind we target. */
static const struct page_kind {
	const char *name;
	uint64_t bit;
} page_kinds[] = {
	{ "rodata",  BIT(RESERVED) },
	{ "slab",    BIT(SLAB)     },
	{ "pgtable", BIT(PGTABLE)  },
};

static unsigned long page_size;

static void fatal(const char *x, ...)
{
	va_list ap;

	va_start(ap, x);
	vfprintf(stderr, x, ap);
	va_end(ap);
	exit(EXIT_FAILURE);
}

static int kpageflags_read(int fd, uint64_t pfn, uint64_t *flags)
{
	ssize_t bytes;

	bytes = pread(fd, flags, sizeof(*flags), (off_t)pfn * sizeof(*flags));

	return bytes == sizeof(*flags) ? 0 : -1;
}

static int pick_rodata_phys_addr(uint64_t *phys_addr)
{
	unsigned long long start, end;
	char line[256], label[64];
	int ret = -1;
	FILE *f;

	f = fopen("/proc/iomem", "r");
	if (!f)
		return -1;

	/* Sub-resources are indented: "  02500000-02ffffff : Kernel rodata" */
	while (fgets(line, sizeof(line), f)) {
		if (sscanf(line, " %llx-%llx : %63[^\n]", &start, &end,
			   label) != 3)
			continue;
		if (strcmp(label, "Kernel rodata") || end <= start)
			continue;
		/* Page-align up and return the first byte of that page. */
		*phys_addr = ((start + page_size - 1) / page_size) * page_size;
		ret = 0;
		break;
	}

	fclose(f);
	return ret;
}

static int pick_kpageflags_phys_addr(uint64_t want, uint64_t *phys_addr)
{
	uint64_t pfn = SZ_16M / page_size;
	uint64_t flags;
	int ret = -1;
	int fd;

	fd = open(PROC_KPAGEFLAGS, O_RDONLY);
	if (fd < 0)
		return -1;

	for (; kpageflags_read(fd, pfn, &flags) == 0; pfn++) {
		if ((flags & want) && !(flags & BIT(HWPOISON)) &&
		    !(flags & BIT(NOPAGE)) && !(flags & BIT(COMPOUND_TAIL))) {
			*phys_addr = pfn * page_size;
			ret = 0;
			break;
		}
	}

	close(fd);
	return ret;
}

static int read_sysctl(unsigned long *val)
{
	FILE *f = fopen(SYSCTL_PATH, "r");
	int ret;

	if (!f)
		return -1;
	ret = fscanf(f, "%lu", val) == 1 ? 0 : -1;
	fclose(f);

	return ret;
}

static int write_sysctl(unsigned long val)
{
	FILE *f = fopen(SYSCTL_PATH, "w");
	int ret;

	if (!f)
		return -1;
	ret = fprintf(f, "%lu", val) < 0 ? -1 : 0;
	fclose(f);

	return ret;
}

/* hard_offline_page() injects with MF_SW_SIMULATED, so unpoison is allowed. */
static void unpoison_pfn(uint64_t pfn)
{
	char path[PATH_MAX], buf[32];
	const char *debugfs;
	int fd, len;

	debugfs = debugfs__mount();
	if (!debugfs)
		return;

	snprintf(path, sizeof(path), "%s/hwpoison/unpoison-pfn", debugfs);
	fd = open(path, O_WRONLY);
	if (fd < 0)
		return;

	len = snprintf(buf, sizeof(buf), "0x%llx\n", (unsigned long long)pfn);
	if (write(fd, buf, len) < 0)
		perror("unpoison-pfn");
	close(fd);
}

static const char *inject_hwpoison(const struct page_kind *kind,
				   uint64_t phys_addr, uint64_t pfn)
{
	char buf[32];
	int fd, len;
	ssize_t ret;

	printf("injecting hwpoison at phys 0x%llx (pfn 0x%llx, kind=%s)\n",
	       (unsigned long long)phys_addr, (unsigned long long)pfn,
	       kind->name);
	printf("expecting kernel panic: 'Memory failure: <pfn>: unrecoverable page'\n");
	fflush(stdout);

	fd = open(INJECT_PATH, O_WRONLY);
	if (fd < 0)
		return "cannot open " INJECT_PATH;

	len = snprintf(buf, sizeof(buf), "0x%llx", (unsigned long long)phys_addr);
	ret = write(fd, buf, len);
	close(fd);

	if (ret != len)
		return "inject failed before reaching the panic path";

	return "inject returned without panic; sysctl ineffective";
}

static const struct page_kind *lookup_kind(const char *name)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(page_kinds); i++)
		if (!strcmp(page_kinds[i].name, name))
			return &page_kinds[i];

	return NULL;
}

static void usage(void)
{
	printf("hwpoison-panic [options]\n"
	       "            -k|--kind <kind>       rodata (default, x86 and riscv only),\n"
	       "                                   slab or pgtable\n"
	       "            --yes-panic-my-kernel  really run it: the machine panics\n"
	       "                                   and every unsaved write is lost\n"
	       "            -h|--help              show this\n");
}

static const struct option opts[] = {
	{ "kind",                 1, NULL, 'k' },
	{ "yes-panic-my-kernel",  0, NULL, 'y' },
	{ "help",                 0, NULL, 'h' },
	{ NULL,                   0, NULL, 0 },
};

static const struct page_kind *parse_args(int argc, char **argv, int *armed)
{
	const struct page_kind *kind;
	const char *name = "rodata";
	int c;

	while ((c = getopt_long(argc, argv, "k:h", opts, NULL)) != -1) {
		switch (c) {
		case 'k':
			name = optarg;
			break;
		case 'y':
			*armed = 1;
			break;
		case 'h':
			usage();
			exit(EXIT_SUCCESS);
		default:
			usage();
			exit(EXIT_FAILURE);
		}
	}

	kind = lookup_kind(name);
	if (!kind)
		fatal("unknown kind '%s' (expected: rodata|slab|pgtable)\n",
		      name);

	return kind;
}

static void check_prereqs(int armed)
{
	if (geteuid())
		fatal("must run as root\n");
	if (access(SYSCTL_PATH, W_OK))
		fatal("%s not present (kernel without the sysctl?)\n",
		      SYSCTL_PATH);
	if (access(INJECT_PATH, W_OK))
		fatal("%s not present (no MEMORY_FAILURE?)\n", INJECT_PATH);
	if (!armed)
		fatal("this panics the kernel; pass --yes-panic-my-kernel\n");
}

static uint64_t pick_phys_addr(const struct page_kind *kind)
{
	uint64_t addr;

	if (kind->bit == BIT(RESERVED)) {
		if (pick_rodata_phys_addr(&addr))
			fatal("no \"Kernel rodata\" in /proc/iomem; try -k slab or -k pgtable\n");
	} else {
		if (pick_kpageflags_phys_addr(kind->bit, &addr))
			fatal("no usable %s PFN in %s\n", kind->name,
			      PROC_KPAGEFLAGS);
	}

	return addr;
}

static void arm_sysctl(unsigned long *prior)
{
	if (read_sysctl(prior))
		fatal("failed to read %s\n", SYSCTL_PATH);
	if (write_sysctl(1))
		fatal("failed to enable %s\n", SYSCTL_PATH);
}

static void recheck_kind(const struct page_kind *kind, uint64_t pfn,
			 const char *verdict)
{
	uint64_t flags;
	int fd, ret;

	fd = open(PROC_KPAGEFLAGS, O_RDONLY);
	if (fd < 0)
		fatal("%s (could not open %s)\n", verdict, PROC_KPAGEFLAGS);
	ret = kpageflags_read(fd, pfn, &flags);
	close(fd);

	if (ret)
		fatal("%s (could not reconfirm page type via %s)\n", verdict,
		      PROC_KPAGEFLAGS);
	if (flags & kind->bit)
		fatal("%s (page still %s)\n", verdict, kind->name);

	fprintf(stderr, "target PFN no longer %s; raced before inject, inconclusive\n",
		kind->name);
}

int main(int argc, char **argv)
{
	uint64_t phys_addr, pfn;
	const struct page_kind *kind;
	const char *verdict;
	unsigned long prior;
	int armed = 0;

	kind = parse_args(argc, argv, &armed);
	check_prereqs(armed);
	page_size = getpagesize();

	phys_addr = pick_phys_addr(kind);
	pfn = phys_addr / page_size;
	arm_sysctl(&prior);

	verdict = inject_hwpoison(kind, phys_addr, pfn);

	/* No panic fired.  Put the machine back, then report. */
	write_sysctl(prior);
	unpoison_pfn(pfn);
	recheck_kind(kind, pfn, verdict);

	return EXIT_FAILURE;
}
