// SPDX-License-Identifier: GPL-2.0
/*
 * smpu_exmon - does an A55 LDAXR arm an ADSP-SC84x SMPU exclusive monitor?
 *
 * For every target, on every online CPU, with IRQs off:
 *   1. one plain load (reachability, before any exclusive)
 *   2. snapshot SMPU{2,4,9} EXACADD/EXACSTAT[0..2]
 *   3. cacheable targets: DC CIVAC the line so the LDAXR has to miss
 *   4. one bare LDAXR (no STXR)
 *   5. snapshot again and print every entry that changed, and whether any
 *      valid entry now holds the target's 64-byte line
 * Optionally, bounded LDAXR/STXR pairs that store back the value just read
 * (memory content never changes), reporting how often STXR succeeds.
 *
 * wprobe=1 (CL2_0 only): on a scratch word l2_addr + 0xc0, NC mapping, first
 * online CPU, writes different values and restores the original:
 *   1. plain store 0x11111111 (baseline)
 *   2. LDAXR                          -> entry armed?
 *   3. plain store 0x22222222         -> does it clear the entry?
 *   4. LDAXR + STXR 0x33333333, plain read-back
 *      -> did a failed STXR still update memory?
 *
 *   target  mapping                           memory type     enabled
 *   ddr-wb  kmalloc()                         Normal WB       always (+pairs)
 *   l2-wb   ioremap_cache(l2_addr + 0x00)     Normal WB       always
 *   l2-nc   ioremap_wc(l2_addr + 0x40)        Normal NC       always (pairs: pair=1)
 *   l2-dev  ioremap_np(l2_addr + 0x80)        Device-nGnRnE   dev=1  (pairs: pair=1)
 *   ddr-nc  vmap(page, pgprot_writecombine)   Normal NC       ddr_nc=1 (+pairs)
 *
 * WARNING: an exclusive to a target that has no monitor makes the A55 take
 * an "implementation fault (unsupported exclusive)" abort (DFSC 0x35). In
 * kernel mode that is an oops (insmod gets killed). Only L2 RAM through the
 * core ports (0x2040_0000-0x207F_FFFF, HRM Table 45-2) is documented as
 * exclusive-capable, hence l2-dev and ddr-nc are opt-in.
 *
 * Results so far, open questions and next steps: smpu_exmon-HANDOVER.md
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/irqflags.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

#include <asm/barrier.h>
#include <asm/cputype.h>
#include <asm/sysreg.h>

/* SMPU registers (HRM ch.12, SVD). Offsets < 0x800: NS-accessible (DSPSI-23) */
#define SMPU_CTL		0x000
#define SMPU_STAT		0x004
#define SMPU_EXACADD(n)		(0x1a0 + 8 * (n))
#define SMPU_EXACSTAT(n)	(0x1a4 + 8 * (n))
#define SMPU_REVID		0x220
#define SMPU_NR_EXA		3

#define EXACSTAT_VALID		BIT(0)
#define EXACSTAT_ARLEN		GENMASK(4, 1)
#define EXACSTAT_ARSIZE		GENMASK(7, 5)
#define EXACSTAT_ARID		GENMASK(20, 8)

/*
 * DSU cluster registers, AArch64 names (DSU TRM r4p1 Table B1-1: Op0 3,
 * CRn c15, Op1 0, CRm c3), i.e. MRS S3_0_C15_C3_<op2>. All three are
 * readable at EL1 without traps (B1.7-B1.9 "Accessibility").
 */
#define SYS_CLUSTERCFR_EL1	sys_reg(3, 0, 15, 3, 0)
#define SYS_CLUSTERIDR_EL1	sys_reg(3, 0, 15, 3, 1)
#define SYS_CLUSTERECTLR_EL1	sys_reg(3, 0, 15, 3, 4)

#define BITV(v, n)		((unsigned int)(((v) >> (n)) & 1))

/* L2 RAM reachable through CL2_0/CL2_1, "EX-Acc: Yes" in HRM Table 45-2 */
#define L2_EXACC_START		0x20400000UL
#define L2_EXACC_END		0x20800000UL

#define LINE			64
#define OFF_WB			0x00
#define OFF_NC			0x40
#define OFF_DEV			0x80
#define OFF_PROBE		0xc0
#define OFF_SPAN		0x100

/* CL2_0 serves the first 2 MB of L2 RAM (HRM Table 45-2) */
#define L2_CL2_0_END		0x20600000UL

#define WP_V0			0x11111111U
#define WP_V1			0x22222222U
#define WP_V2			0x33333333U

#define NR_SMPU			3

static struct {
	const char *name;
	const char *desc;
	phys_addr_t base;
	void __iomem *regs;
} smpus[NR_SMPU] = {
	{ "SMPU2", "CL2_0, L2 0x2040_0000-0x205F_FFFF", 0x31083000 },
	{ "SMPU4", "CL2_1, L2 0x2060_0000-0x207F_FFFF", 0x31085000 },
	{ "SMPU9", "DMC0, DDR",                         0x310a0000 },
};

static unsigned long l2_addr[4] = { 0x205ff000, 0x207ff000 };
static unsigned int n_l2_addr = 2;
module_param_array(l2_addr, ulong, &n_l2_addr, 0444);
MODULE_PARM_DESC(l2_addr, "L2 physical addresses, 256-byte aligned (default: one per core port)");

static bool opt_pair;
module_param_named(pair, opt_pair, bool, 0444);
MODULE_PARM_DESC(pair, "Also run LDAXR/STXR pairs on l2-nc/l2-dev (stores back the value read; use addresses nobody else writes)");

static bool opt_dev;
module_param_named(dev, opt_dev, bool, 0444);
MODULE_PARM_DESC(dev, "Also test a Device-nGnRnE mapping of L2 (what adi sram_mmap uses)");

static bool opt_ddr_nc;
module_param_named(ddr_nc, opt_ddr_nc, bool, 0444);
MODULE_PARM_DESC(ddr_nc, "Also test Normal-NC DDR (SMPU9 exclusive support is undocumented: may oops)");

static bool opt_wprobe;
module_param_named(wprobe, opt_wprobe, bool, 0444);
MODULE_PARM_DESC(wprobe, "Write probe on l2_addr + 0xc0 (NC, CL2_0): writes different values, use an unused word");

static bool opt_force;
module_param_named(force, opt_force, bool, 0444);
MODULE_PARM_DESC(force, "Allow l2_addr outside 0x2040_0000-0x207F_FFFF (may oops)");

static unsigned int tries = 1000;
module_param(tries, uint, 0444);
MODULE_PARM_DESC(tries, "LDAXR/STXR pairs per target and CPU (max 100000)");

static bool all_cpus = true;
module_param(all_cpus, bool, 0444);
MODULE_PARM_DESC(all_cpus, "Run on every online CPU (default) or only the first");

static bool opt_verbose;
module_param_named(verbose, opt_verbose, bool, 0444);
MODULE_PARM_DESC(verbose, "Print all monitor entries, not only changed/matching ones");

static bool show_dsu = true;
module_param(show_dsu, bool, 0444);
MODULE_PARM_DESC(show_dsu, "Dump core/DSU configuration (MIDR, CLUSTERIDR/CFR/ECTLR)");

struct snap {
	u32 add[NR_SMPU][SMPU_NR_EXA];
	u32 stat[NR_SMPU][SMPU_NR_EXA];
};

struct job {
	/* in */
	void *va;		/* address the exclusives are issued to */
	phys_addr_t pa;
	void *flush_va;		/* cacheable VA to DC CIVAC first, or NULL */
	bool do_pair;
	/* out */
	int cpu;
	u64 mpidr;
	u32 plain;
	u32 val;
	unsigned int pair_ok;
	struct snap before, after_ld, after_pair;
};

static __always_inline u32 ldr32(const void *p)
{
	u32 v;

	asm volatile("ldr	%w0, [%1]" : "=r" (v) : "r" (p) : "memory");
	return v;
}

static __always_inline void str32(void *p, u32 v)
{
	asm volatile("str	%w0, [%1]" : : "r" (v), "r" (p) : "memory");
}

static __always_inline u32 ldaxr32(const void *p)
{
	u32 v;

	asm volatile("ldaxr	%w0, [%1]" : "=r" (v) : "r" (p) : "memory");
	return v;
}

/* Returns the STXR status: 0 = success. Stores back the value just read. */
static __always_inline u32 ldaxr_stxr_same32(void *p)
{
	u32 v, fail;

	asm volatile(
	"	ldaxr	%w[v], [%[p]]\n"
	"	stxr	%w[f], %w[v], [%[p]]\n"
	: [v] "=&r" (v), [f] "=&r" (fail)
	: [p] "r" (p)
	: "memory");
	return fail;
}

/* LDAXR then STXR of a new value. Returns the STXR status: 0 = success. */
static __always_inline u32 ldaxr_stxr32(void *p, u32 newv, u32 *old)
{
	u32 v, fail;

	asm volatile(
	"	ldaxr	%w[v], [%[p]]\n"
	"	stxr	%w[f], %w[n], [%[p]]\n"
	: [v] "=&r" (v), [f] "=&r" (fail)
	: [p] "r" (p), [n] "r" (newv)
	: "memory");
	*old = v;
	return fail;
}

static __always_inline void dc_civac(const void *p)
{
	asm volatile("dc	civac, %0" : : "r" (p) : "memory");
}

static __always_inline void clrex(void)
{
	asm volatile("clrex" : : : "memory");
}

static void snapshot(struct snap *s)
{
	int i, n;

	for (i = 0; i < NR_SMPU; i++) {
		for (n = 0; n < SMPU_NR_EXA; n++) {
			s->add[i][n] = readl_relaxed(smpus[i].regs + SMPU_EXACADD(n));
			s->stat[i][n] = readl_relaxed(smpus[i].regs + SMPU_EXACSTAT(n));
		}
	}
}

static long job_fn(void *arg)
{
	struct job *j = arg;
	unsigned long flags;
	unsigned int i;

	local_irq_save(flags);
	j->cpu = smp_processor_id();
	j->mpidr = read_cpuid_mpidr();

	/* An abort here (not at the LDAXR) means: not reachable at all */
	j->plain = ldr32(j->va);
	dsb(sy);

	snapshot(&j->before);
	if (j->flush_va)
		dc_civac(j->flush_va);
	dsb(sy);
	j->val = ldaxr32(j->va);
	dsb(sy);
	snapshot(&j->after_ld);
	clrex();

	if (j->do_pair) {
		for (i = 0; i < tries; i++)
			if (!ldaxr_stxr_same32(j->va))
				j->pair_ok++;
		clrex();
		dsb(sy);
		snapshot(&j->after_pair);
	}

	/* Leave no cached copy behind */
	if (j->flush_va) {
		dc_civac(j->flush_va);
		dsb(sy);
	}
	local_irq_restore(flags);

	return 0;
}

/* Requester port from the low 7 ID bits, HRM Table 45-7 */
static const char *id_port(u32 id)
{
	switch (id & 0x7f) {
	case 0x29: return "A55 M1 AXI";
	case 0x59: return "A55 M0 AXI";
	case 0x69: return "A55 MMR";
	case 0x09: return "SH0 DPORT";
	case 0x39: return "SH0 DPORT L2";
	case 0x19: return "SH0 iDMA";
	case 0x49: return "HSM";
	default:   return "other";
	}
}

static void print_entry(bool chg, bool hit, int i, int n, u32 add, u32 stat)
{
	u32 id = FIELD_GET(EXACSTAT_ARID, stat);

	pr_info("    %s %s EXA%d: %s addr=0x%08x id=0x%04x [%s, upper=0x%02x] %uB x%u%s\n",
		chg ? "changed" : "       ", smpus[i].name, n,
		(stat & EXACSTAT_VALID) ? "VALID" : "-----", add, id,
		id_port(id), id >> 7,
		1U << (u32)FIELD_GET(EXACSTAT_ARSIZE, stat),
		(u32)FIELD_GET(EXACSTAT_ARLEN, stat) + 1,
		hit ? "  <== target line" : "");
}

static bool holds(const struct snap *s, int i, int n, phys_addr_t pa)
{
	return (s->stat[i][n] & EXACSTAT_VALID) &&
	       !(((phys_addr_t)s->add[i][n] ^ pa) & ~(phys_addr_t)(LINE - 1));
}

static void report(const char *when, const struct snap *b,
		   const struct snap *a, phys_addr_t pa)
{
	bool now = false, before = false, armed = false;
	int i, n;

	for (i = 0; i < NR_SMPU; i++) {
		for (n = 0; n < SMPU_NR_EXA; n++) {
			bool chg = a->add[i][n] != b->add[i][n] ||
				   a->stat[i][n] != b->stat[i][n];
			bool hit = holds(a, i, n, pa);

			before |= holds(b, i, n, pa);
			now |= hit;
			armed |= hit && chg;
			if (opt_verbose || chg || hit)
				print_entry(chg, hit, i, n, a->add[i][n], a->stat[i][n]);
		}
	}

	if (armed)
		pr_info("    => after %s: an SMPU monitor entry now holds the target line\n", when);
	else if (now)
		pr_info("    => after %s: target line held, entry unchanged by this step\n", when);
	else if (before)
		pr_info("    => after %s: target line no longer held (entry cleared)\n", when);
	else
		pr_info("    => after %s: no SMPU monitor entry holds the target line\n", when);
}

static void print_job(const struct job *j)
{
	pr_info("  cpu%d (MPIDR 0x%llx): plain read 0x%08x, LDAXR read 0x%08x\n",
		j->cpu, j->mpidr, j->plain, j->val);
	report("LDAXR", &j->before, &j->after_ld, j->pa);
	if (!j->do_pair)
		return;
	pr_info("  cpu%d: LDAXR/STXR pairs: STXR succeeded %u/%u\n", j->cpu, j->pair_ok, tries);
	report("pairs", &j->after_ld, &j->after_pair, j->pa);
}

static void run_target(const char *what, void *va, phys_addr_t pa,
		       void *flush_va, bool do_pair)
{
	struct job *j;
	int cpu;

	j = kmalloc(sizeof(*j), GFP_KERNEL);
	if (!j)
		return;

	pr_info("=== %s, pa %pa%s\n", what, &pa, do_pair ? ", with pairs" : "");

	cpus_read_lock();
	for_each_online_cpu(cpu) {
		if (!all_cpus && (unsigned int)cpu != cpumask_first(cpu_online_mask))
			continue;
		memset(j, 0, sizeof(*j));
		j->va = va;
		j->pa = pa;
		j->flush_va = flush_va;
		j->do_pair = do_pair;
		/* Last line in the log if the access below aborts */
		pr_info("  cpu%d: issuing exclusives...\n", cpu);
		work_on_cpu(cpu, job_fn, j);
		print_job(j);
	}
	cpus_read_unlock();

	kfree(j);
}

struct wprobe {
	void *va;
	phys_addr_t pa;
	int cpu;
	u32 orig, ld1, ld2, final, stxr;
	struct snap s0, s1, s2, s3;
};

static long wprobe_fn(void *arg)
{
	struct wprobe *w = arg;
	unsigned long flags;

	local_irq_save(flags);
	w->cpu = smp_processor_id();

	/* 1. Baseline: known value */
	w->orig = ldr32(w->va);
	str32(w->va, WP_V0);
	dsb(sy);
	snapshot(&w->s0);

	/* 2. Arm a reservation */
	w->ld1 = ldaxr32(w->va);
	dsb(sy);
	snapshot(&w->s1);
	clrex();

	/* 3. Plain store to the reserved word, same requester */
	str32(w->va, WP_V1);
	dsb(sy);
	snapshot(&w->s2);

	/* 4. LDAXR + STXR of a new value, then read back */
	w->stxr = ldaxr_stxr32(w->va, WP_V2, &w->ld2);
	dsb(sy);
	w->final = ldr32(w->va);
	snapshot(&w->s3);
	clrex();

	/* Restore */
	str32(w->va, w->orig);
	dsb(sy);
	local_irq_restore(flags);

	return 0;
}

static void run_wprobe(void *va, phys_addr_t pa)
{
	const char *verdict;
	struct wprobe *w;
	unsigned int cpu;

	w = kzalloc(sizeof(*w), GFP_KERNEL);
	if (!w)
		return;
	w->va = va;
	w->pa = pa;

	pr_info("=== wprobe: L2, Normal Non-cacheable (ioremap_wc), pa %pa\n", &pa);

	cpus_read_lock();
	cpu = cpumask_first(cpu_online_mask);
	/* Last line in the log if the probe below aborts */
	pr_info("  cpu%u: issuing probe...\n", cpu);
	work_on_cpu(cpu, wprobe_fn, w);
	cpus_read_unlock();

	pr_info("  cpu%d: original 0x%08x restored\n", w->cpu, w->orig);
	pr_info("  step 1: plain store 0x%08x\n", WP_V0);
	pr_info("  step 2: LDAXR read 0x%08x (expect 0x%08x)\n", w->ld1, WP_V0);
	report("step 2 (LDAXR)", &w->s0, &w->s1, pa);
	pr_info("  step 3: plain store 0x%08x (same requester as the reservation)\n", WP_V1);
	report("step 3 (plain store)", &w->s1, &w->s2, pa);
	pr_info("  step 4: LDAXR read 0x%08x (expect 0x%08x), STXR 0x%08x -> status %u, read back 0x%08x\n",
		w->ld2, WP_V1, WP_V2, w->stxr, w->final);
	report("step 4 (LDAXR+STXR)", &w->s2, &w->s3, pa);

	if (!w->stxr && w->final == WP_V2)
		verdict = "STXR succeeded, memory updated";
	else if (w->stxr && w->final == WP_V1)
		verdict = "clean failure: STXR failed and memory NOT updated (write did not match the reservation)";
	else if (w->stxr && w->final == WP_V2)
		verdict = "STXR reported failure but memory WAS updated (handled as a normal write)";
	else if (!w->stxr)
		verdict = "STXR reported success but memory NOT updated";
	else
		verdict = "unexpected read-back value";
	pr_info("  => wprobe verdict: %s\n", verdict);

	kfree(w);
}

static void flush_page(void *va)
{
	unsigned int i;

	for (i = 0; i < PAGE_SIZE; i += LINE)
		dc_civac(va + i);
	dsb(sy);
}

static void test_ddr_wb(void)
{
	u32 *buf = kmalloc(PAGE_SIZE, GFP_KERNEL);

	if (!buf)
		return;
	buf[0] = 0xc0ffee00;
	run_target("ddr-wb: DDR, Normal Write-Back (kmalloc)",
		   buf, virt_to_phys(buf), buf, true);
	kfree(buf);
}

static void test_l2(unsigned long addr)
{
	unsigned long page = addr & PAGE_MASK, off = addr & ~PAGE_MASK;
	void __iomem *m;

	if ((addr & (OFF_SPAN - 1)) || off > PAGE_SIZE - OFF_SPAN) {
		pr_err("l2_addr 0x%lx: must be 256-byte aligned\n", addr);
		return;
	}
	if (!opt_force && (addr < L2_EXACC_START || addr + OFF_SPAN > L2_EXACC_END)) {
		pr_err("l2_addr 0x%lx: outside 0x%lx-0x%lx, skipping (force=1 to override)\n",
		       addr, L2_EXACC_START, L2_EXACC_END - 1);
		return;
	}

	/* One mapping at a time: no simultaneous aliases with different types */
	m = ioremap_cache(page, PAGE_SIZE);
	if (m) {
		run_target("l2-wb: L2, Normal Write-Back (ioremap_cache)",
			   (__force void *)m + off + OFF_WB, addr + OFF_WB,
			   (__force void *)m + off + OFF_WB, false);
		/* Drop anything the WB mapping pulled in, speculative fills too */
		flush_page((__force void *)m);
		iounmap(m);
	}

	m = ioremap_wc(page, PAGE_SIZE);
	if (m) {
		run_target("l2-nc: L2, Normal Non-cacheable (ioremap_wc)",
			   (__force void *)m + off + OFF_NC, addr + OFF_NC,
			   NULL, opt_pair);
		if (opt_wprobe && (opt_force || addr + OFF_SPAN <= L2_CL2_0_END))
			run_wprobe((__force void *)m + off + OFF_PROBE,
				   addr + OFF_PROBE);
		else if (opt_wprobe)
			pr_info("wprobe: skipped for l2_addr 0x%lx (not CL2_0, where NC LDAXR aborts; force=1 to override)\n",
				addr);
		iounmap(m);
	}

	if (!opt_dev)
		return;
	m = ioremap_np(page, PAGE_SIZE);
	if (m) {
		run_target("l2-dev: L2, Device-nGnRnE (ioremap_np)",
			   (__force void *)m + off + OFF_DEV, addr + OFF_DEV,
			   NULL, opt_pair);
		iounmap(m);
	}
}

static void test_ddr_nc(void)
{
	struct page *pg = alloc_page(GFP_KERNEL);
	u32 *lin;
	void *nc;

	if (!pg)
		return;
	lin = page_address(pg);
	lin[OFF_NC / 4] = 0xc0ffee01;
	/* Push the linear (WB) alias out before using the NC alias */
	flush_page(lin);

	nc = vmap(&pg, 1, VM_MAP, pgprot_writecombine(PAGE_KERNEL));
	if (nc) {
		run_target("ddr-nc: DDR, Normal Non-cacheable (vmap WC)",
			   nc + OFF_NC, page_to_phys(pg) + OFF_NC, NULL, true);
		vunmap(nc);
	}
	flush_page(lin);
	__free_page(pg);
}

/* Core and DSU configuration: DSU TRM r4p1 B1.7-B1.9, routing per A6.1.1 */
static void dump_dsu(void)
{
	static const char * const bus_name[] = {
		"single 128-bit ACE", "dual 128-bit ACE",
		"single 128-bit CHI", "single 256-bit CHI",
	};
	u32 midr = read_cpuid_id();
	u64 idr = read_sysreg_s(SYS_CLUSTERIDR_EL1);
	u64 cfr = read_sysreg_s(SYS_CLUSTERCFR_EL1);
	u64 ectlr = read_sysreg_s(SYS_CLUSTERECTLR_EL1);
	unsigned int bus = FIELD_GET(GENMASK(10, 9), cfr);

	pr_info("MIDR_EL1 0x%08x: implementer 0x%02x part 0x%03x r%up%u\n",
		midr, (unsigned int)MIDR_IMPLEMENTOR(midr),
		(unsigned int)MIDR_PARTNUM(midr),
		(unsigned int)MIDR_VARIANT(midr),
		(unsigned int)MIDR_REVISION(midr));

	pr_info("CLUSTERIDR_EL1 0x%08llx: DSU r%up%u\n", idr,
		(unsigned int)FIELD_GET(GENMASK(7, 4), idr),
		(unsigned int)FIELD_GET(GENMASK(3, 0), idr));

	pr_info("CLUSTERCFR_EL1 0x%08llx: %u core(s), %u PE(s), %s, ACP %s, peripheral port %s, L3 %s\n",
		cfr,
		(unsigned int)FIELD_GET(GENMASK(2, 0), cfr) + 1,
		(unsigned int)FIELD_GET(GENMASK(27, 24), cfr) + 1,
		(bus == 3 && BITV(cfr, 13)) ? "dual 256-bit CHI" : bus_name[bus],
		BITV(cfr, 11) ? "present" : "absent",
		BITV(cfr, 12) ? "present" : "absent",
		BITV(cfr, 4) ? "present" : "absent");

	pr_info("CLUSTERECTLR_EL1 0x%08llx: NC-behaviour[0]=%u flush-UC-evict[2]=%u evict-disable[3]=%u prefetch-delay[10:8]=%u UC-evict[14]=%u\n",
		ectlr, BITV(ectlr, 0), BITV(ectlr, 2), BITV(ectlr, 3),
		(unsigned int)FIELD_GET(GENMASK(10, 8), ectlr), BITV(ectlr, 14));

	if (bus == 1)
		pr_info("routing (DSU TRM A6.1.1): Device -> ACE IF0, Normal-NC -> %s, Cacheable -> interleaved on INTERLEAVE_ADDR_BIT\n",
			BITV(ectlr, 0) ? "interleaved like Cacheable" : "ACE IF0");
	else
		pr_info("routing: single master interface, all traffic uses it\n");

	pr_info("not readable: INTERLEAVE_ADDR_BIT (build parameter), BROADCASTOUTER/BROADCASTCACHEMAINT (input pins)\n");
}

static int __init smpu_exmon_init(void)
{
	unsigned int i;
	int ret = 0;

	tries = min(tries, 100000U);

	for (i = 0; i < NR_SMPU; i++) {
		smpus[i].regs = ioremap(smpus[i].base, SZ_4K);
		if (!smpus[i].regs) {
			ret = -ENOMEM;
			goto out;
		}
		pr_info("%s (%s) @ %pa: REVID 0x%02x CTL 0x%08x STAT 0x%08x\n",
			smpus[i].name, smpus[i].desc, &smpus[i].base,
			readl(smpus[i].regs + SMPU_REVID),
			readl(smpus[i].regs + SMPU_CTL),
			readl(smpus[i].regs + SMPU_STAT));
	}

	if (show_dsu)
		dump_dsu();

	test_ddr_wb();
	for (i = 0; i < n_l2_addr; i++)
		test_l2(l2_addr[i]);
	if (opt_ddr_nc)
		test_ddr_nc();

	pr_info("done (rmmod %s before running again)\n", KBUILD_MODNAME);
out:
	for (i = 0; i < NR_SMPU; i++) {
		if (smpus[i].regs)
			iounmap(smpus[i].regs);
		smpus[i].regs = NULL;
	}
	return ret;
}

static void __exit smpu_exmon_exit(void)
{
}

module_init(smpu_exmon_init);
module_exit(smpu_exmon_exit);

MODULE_DESCRIPTION("ADSP-SC84x: check whether A55 exclusives arm the SMPU exclusive monitors");
MODULE_LICENSE("GPL");
