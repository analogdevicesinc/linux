# smpu_exmon: A55 exclusives vs SC84x SMPU monitors (handover)

Status: work in progress, 2026-09-30. Started as a review of
`sc846-shared-memory-cacheability-matrix.md` (e-mail thread "Possible
Exclusive access issue in A55 due to HASH").

Legend: **[F]** documented fact (reference in §6) · **[M]** measured with this
module (run in §3) · **[I]** inference · **[H]** hypothesis / guess, not verified.

## 1. The tool

`smpu_exmon.c`, `CONFIG_ADI_SMPU_EXMON` (module only). All work happens at
load time, results go to dmesg, `rmmod` before the next run.

Per target, per online CPU, IRQs off: plain load, snapshot of all monitor
entries of SMPU2/4/9, `DC CIVAC` (cacheable targets only, forces a miss), one
bare `LDAXR`, snapshot, print what changed.

| target | mapping | enabled |
|---|---|---|
| ddr-wb | `kmalloc`, Normal WB | always, with pairs |
| l2-wb | `ioremap_cache(l2_addr + 0x00)` | always |
| l2-nc | `ioremap_wc(l2_addr + 0x40)` | always, pairs with `pair=1` |
| l2-dev | `ioremap_np(l2_addr + 0x80)`, Device-nGnRnE | `dev=1` |
| ddr-nc | `vmap(page, pgprot_writecombine)` | `ddr_nc=1`, with pairs |
| wprobe | `ioremap_wc(l2_addr + 0xc0)` | `wprobe=1` (CL2_0 only unless `force=1`) |

- **pairs:** `tries` (1000) x `LDAXR` + `STXR` of the value just read. STXR data
  depends on the load; memory never changes.
- **wprobe:** store 0x11111111, `LDAXR`, store 0x22222222, `LDAXR` +
  `STXR 0x33333333` (independent value), read back, restore original.
- **startup dump:** SMPU REVID/CTL/STAT, `MIDR_EL1`, `CLUSTERIDR/CFR/ECTLR_EL1`.
- Other parameters: `l2_addr=` (up to 4, 256-byte aligned), `all_cpus`,
  `verbose`, `show_dsu`, `tries`.

Tool pitfalls:

- Default `l2_addr` contains 0x207ff000, whose NC `LDAXR` **oopses** (R3).
  Always pass `l2_addr`; `l2_addr=0` skips the L2 tests.
- The wprobe address check and its message are stale: the F-M0 window
  0x2060_0000-0x2070_FFFF works (currently needs `force=1`).
- An unsupported exclusive is an oops [F: `arch/arm64/mm/fault.c`, DFSC 0x35
  -> `do_bad`]. `insmod` then hangs holding `cpus_read_lock`: reboot.
- SMPU monitor state survives `rmmod` [M: R2], and see "poisoning" (R8, R10).
- Tested so far: NS EL1 only, cpu0 only (cpu1 not online), SHARC state not
  recorded.

## 2. Documented background

| # | Fact | Ref |
|---|---|---|
| B1 | SMPU2/4 "support exclusive access over CL2_0 and CL2_1 ... between SHARC FX DPORT and A55". "EX-Acc: Yes" only for CL2_0 (0x2040_0000-0x205F_FFFF + ROM) and CL2_1 (0x2060_0000-0x207F_FFFF); DL2_0, CL2_2, DL2_1: "No" | HRM ch.45, text above Table 45-2, Table 45-2 |
| B2 | `EXACADD[n]` = address, `EXACSTAT[n]` = ARID[20:8] ARSIZE[7:5] ARLEN[4:1] VALID[0], read-only. HRM gives no offsets or count | HRM Tables 12-9, 12-10 |
| B3 | 3 slots at +0x1A0 + 8n (ADD) / +0x1A4 + 8n (STAT), only in SMPU2, SMPU4 (`derivedFrom` SMPU2), SMPU9 | SVD; UBOOT header l.21005-21013, 21106-21114, 25263-25271 |
| B4 | 13-bit requester IDs, low 7 bits fixed: A55 M1 AXI `0101001` (0x0029), A55 M0 AXI `1011001`, A55_MMR `1101001`, SH0 DPORT `0001001` | HRM Table 45-7 |
| B5 | DSU has one 128-bit data port + 64-bit MP. NIC-400 async bridge splits data into F-M0 = 0x2060_0000-0x2070_FFFF and F-M1 = 0x0-0x205F_FFFF + 0x2071_0000-0xFFFF_FFFF (includes LPDDR4 0x8000_0000-0xBFFF_FFFF); MP = 0x3000_0000-0x3FFF_FFFF | APPS slides 3, 4 |
| B6 | AXI: monitor records address + ARID of an exclusive read; exclusive write gets EXOKAY only if nothing wrote the location since; a failed exclusive write gets OKAY and "must not update the address location"; IDs/size/length must match; monitored exclusives must not be cacheable; a slave without a monitor answers OKAY | AXI §6.2.1-6.2.5, Table 7-1 |
| B7 | A55: NC/Device LDXR to a region without exclusive support -> data abort DFSC 0b110101. Atomics (LSE) to NC/Device need interconnect atomics, else abort. ACE has no atomic transactions | A55 §A6.4, §A6.4.2; DSU Table A6-3 |
| B8 | DSU: Write-Back transfers are only 64-byte linefills/evictions; exclusive read/write transfers exist only for Normal-NC/Device. Only Inner+Outer WB is cached | DSU §A6.6, §A6.7 |
| B9 | DSU peripheral port: exclusives unsupported, "Store exclusive instructions will fail ... but the memory location might be updated" | DSU §A9.1 |
| B10 | U-Boot writes 0x500 (NS R/W enable) to SECURECTL of SMPU2/3/4/5/6/9/11/12, no regions. SMPU_CTL reset value is 0 | UBOOT `sc5xx_soc_init()`; SVD |
| B11 | Apps bare-metal tests run on A55[0] at EL3 (Secure). DSU -> NIC-400 path described as "ACE5 (AXI-4, non-coherent)" | APPS slide 2 |

## 3. Measured

Linux 6.18.31-00320-g67fae9a8f5fc. Commands are `insmod smpu_exmon.ko ...`.

| Boot | R | Parameters | Result |
|---|---|---|---|
| A | R1 | `l2_addr=0x205ff000` | ddr-wb, l2-wb: no SMPU entry; ddr-wb pairs 1000/1000. l2-nc: new `SMPU2 EXA0 VALID addr=0x205ff040 id=0x0029 [A55 M1] 4B x1` |
| A | R2 | `l2_addr=0x205ff000 pair=1` | l2-nc entry unchanged (left from R1); **STXR 0/1000**; entry still VALID |
| A | R3 | `l2_addr=0x207ff000 pair=1` | l2-wb read works. l2-nc `LDAXR` **oops**: ESR 0x96000035 (FSC 0x35 unsupported exclusive), insn `885ffd08` = `ldaxr w8, [x8]`, PTE AttrIndx 2 = `MT_NORMAL_NC` |
| B | R4 | `l2_addr=0x205fe000 pair=1` | Dump: MIDR 0x412fd050 (A55 r2p0), CLUSTERIDR 0x41 (DSU r4p1), CLUSTERCFR 0x01001111 (2 cores, **single 128-bit ACE**, ACP absent, peripheral port present, L3), CLUSTERECTLR 0x500 (reset value), SMPU2/4/9 REVID 0x10, CTL 0x1 (RSDIS=1), STAT 0. l2-nc: new SMPU2 EXA0 0x205fe040 id 0x0029; **STXR 0/1000**, entry stays. First SMPU exclusive activity of boot B |
| B | R5 | `l2_addr=0x206ff000 pair=1` | new `SMPU4 EXA0 VALID addr=0x206ff040 id=0x0059 [A55 M0] 4B x1`; **STXR 1000/1000**; entry cleared afterwards |
| B | R6 | `l2_addr=0x205fe000 wprobe=1` | probe 0x205fe0c0 (SMPU2): step 2 armed; step 3 plain store **cleared** the entry; step 4 **STXR status 1 but memory = 0x33333333**, entry VALID |
| B | R7 | `l2_addr=0x206ff000 wprobe=1 force=1` | probe 0x206ff0c0 (SMPU4): same as R6 |
| B | R8 | `l2_addr=0x206ff000 pair=1` | STXR fails (reported, log not kept) |
| C | R9 | `l2_addr=0x206ff000 pair=1` x3 | pass (reported) |
| C | R10 | `l2_addr=0x206ff000 wprobe=1 force=1`, then `pair=1` | pairs **fail** until the next clean reboot (reported, reproducible) |

Boot B followed the R3 oops (reboot type not recorded). Boot C: "clean
reboot" (warm vs cold not established). Register decodes per B2/B3 and DSU
§B1.7-B1.9; bridge port per B5; ID names per B4.

Measured facts in short:

- **M1** Write-Back `LDAXR` (after `DC CIVAC`) never created or changed an SMPU
  entry, on DDR (SMPU9) or L2 (SMPU2, SMPU4). NC `LDAXR` on the same SMPUs did
  (R1, R4, R5).
- **M2** NC `LDAXR` arms one slot with the exact byte address, the port ID and
  4 B x 1 beat (R1, R4, R5).
- **M3** Store-back pairs: F-M1 -> SMPU2 0/1000 (R2, R4); F-M0 -> SMPU4
  1000/1000 on a fresh boot, entry cleared after success (R5, R9).
- **M4** F-M1 at 0x207ff040: NC `LDAXR` aborts, plain WB read works (R3).
- **M5** A plain store to the reserved word (same requester) clears the entry
  (R6, R7).
- **M6** `STXR` with an independent value after that: status "failed", memory
  written, entry not cleared, on both paths (R6, R7).
- **M7** After the probe, F-M0 pairs fail until a clean reboot (R8, R10).
- **M8** The DSU has a single ACE data port (R4).

## 4. Inferences and hypotheses

- **[I]** Write-Back exclusives are decided inside the A55 cluster (coherence
  based monitor) and never reach an SMPU monitor (B8, M1, with M2 as positive
  control on the same SMPUs). The registers cannot tell "linefill not flagged
  exclusive" from "flagged but ignored".
- **[I]** HRM "A55 M0/M1 AXI" = bridge F-M0/F-M1, "A55_MMR" = MP (B4, B5, M2).
  A55_MMR was never observed.
- **[I]** 0x2071_0000-0x207F_FFFF: F-M1 has no path to CL2_1, so it reaches L2
  through an "EX-Acc: No" port, which answers OKAY, so the A55 aborts (B1, B5,
  B6, B7, M4). The port actually used is not verified. Either the bridge map
  or HRM's CL2_1 range is wrong; M4 supports the bridge map.
- **[I]** M6 violates AXI §6.2.3 (a failed exclusive write updated memory).
  Unlike M5 the entry was not cleared, so the write was handled as a failed
  exclusive and still forwarded. Externally this matches B9, but our accesses
  do not use MP (IDs 0x029/0x059).
- **[I]** Impact: on affected paths LL/SC loops never succeed and failed
  attempts can still write memory: livelock plus silent corruption. Linux
  kernel atomics are LSE (`ARM64_USE_LSE_ATOMICS` default y) and abort on NC
  memory instead (B7). Kernel LL/SC on NC memory only happens via futex on user
  mappings (`arch/arm64/include/asm/futex.h`), e.g. `sram_mmap.c` mappings
  (`pgprot_noncached`).
- **[H]** Why R7 step 4 fails on F-M0 while R5 pairs pass: (a) the independent
  STXR value lets the write reach the monitor before the reservation is
  recorded; (b) the preceding plain store; (c) earlier poisoning. Not isolated.
- **[H]** "Poisoning" (M7): hidden state in SMPU4 or in the bridge, invisible
  in `EXACSTAT`, cleared by a clean reboot.
- **[H]** F-M1 path defect vs poisoning: R4 failed as the first SMPU exclusive
  activity after a reboot, which points at F-M1/SMPU2 itself, but whether that
  reboot reset the SMPUs is unknown.
- **[H]** "HASH" in the thread title may be the bridge address split.

## 5. Next steps

1. Clean reboot, then first thing `l2_addr=0x205fe000 pair=1`: does
   F-M1 -> SMPU2 fail on a surely fresh boot?
2. Clean reboot, `l2_addr=0 ddr_nc=1`: F-M1 -> SMPU9 (DDR), first exclusive test
   on SMPU9, may oops. With step 1: F-M1 common factor vs SMPU2 only.
3. Poison F-M0 (`l2_addr=0x206ff000 wprobe=1 force=1`), then run steps 1/2: is
   the bad state shared (bridge/DSU) or per SMPU?
4. Bisect the poisoning (module change): wprobe variants a) plain store only,
   b) independent STXR only, c) data-dependent STXR (loaded + 0x11111111). Each
   from a clean reboot, followed by F-M0 pairs.
5. `pair_mode=inc` (module change): data-dependent increment; memory delta vs
   success count exposes failed-but-written STXRs inside loops.
6. Secure vs Non-secure: same sequences at EL3 (U-Boot command or apps bare
   metal, B11).
7. SWU (HRM ch.46): address watch on each L2 port for 0x207ff040 (which port
   does F-M1 use?), ID compare on CL2_0's write channel (the STXR's AWID).
8. Bring up cpu1: per-core ID bits.
9. Tool fixes: default `l2_addr=0x205ff000,0x206ff000`; wprobe check allows
   0x2040_0000-0x2070_FFFF, blocks 0x2071_0000-0x207F_FFFF; dump all entries at
   load; a `skip_l2` parameter.
10. Ask ADI/apps: intended bridge map vs HRM CL2_1 range; exclusives through
    F-M1; failed exclusive writes updating memory; poisoning; who sets
    SMPU_CTL.RSDIS; requester/EL/memory type used by their exclusive test.

## 6. References

- **HRM:** ADSP-2184x/SC84x SHARC-FX Hardware Reference, preliminary rev 0.2:
  ch.12 SMPU (Tables 12-1, 12-8, 12-9, 12-10), ch.45 SCB (Tables 45-1, 45-2 and
  the sentence above it, 45-7), ch.1 A55 (Table 1-3), ch.46 SWU. Data sheet
  Tables 4 and 6 (memory map).
- **SVD:** `ADSP-SC84x.svd`: `SMPU2_EXAn[0..2]_EXACADD/EXACSTAT` (0x1a0-0x1b4),
  `SMPU4 derivedFrom="SMPU2"`, `SMPU9_EXAn[0..2]`, SMPU2 `CTL` resetValue 0.
- **UBOOT:** u-boot `arch/arm/mach-sc5xx/init/ADSP-SC84xW.h` (lines above),
  `arch/arm/mach-sc5xx/sc846-som.c` `sc5xx_soc_init()`.
- **A55:** Arm Cortex-A55 TRM r2p0, 100442_0200_00_en: §A6.4 "Atomic
  instructions", §A6.4.2-A6.4.3.
- **DSU:** Arm DynamIQ Shared Unit TRM r4p1, 100453_0401_03_en: Tables A6-3,
  A6-5, A6-6, B1-1; §A6.6, §A6.7, §A9.1, §B1.7-B1.9.
  https://documentation-service.arm.com/static/5e7e1bd8b2608e4d7f0a35b4
- **AXI:** AMBA AXI Protocol Specification, ARM IHI 0022C: §6.1 (Table 6-1),
  §6.2.1-6.2.5, Table 7-1, §13.15.1.
- **APPS:** `EHP2_A55_L3_Cache_Issue_Summary_For_ARM_v00.pdf` (ADI
  applications, 22/Sep/2026), slides 2-4.
- **KERN:** this tree: `arch/arm64/mm/fault.c`,
  `arch/arm64/include/asm/futex.h`, `arch/arm64/Kconfig`
  (`ARM64_USE_LSE_ATOMICS`), `drivers/misc/adi/sram_mmap.c`.
