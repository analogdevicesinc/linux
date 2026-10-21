// SPDX-License-Identifier: GPL-2.0
/* memory.c: Prom routine for acquiring various bits of information
 *           about RAM on the machine, both virtual and physical.
 *
 * Copyright (C) 1995, 2008 David S. Miller (davem@davemloft.net)
 * Copyright (C) 1997 Michael A. Griffith (grif@acm.org)
 */

#include <linux/kernel.h>
#include <linux/memblock.h>
#include <linux/init.h>

#include <asm/openprom.h>
#include <asm/oplib.h>
#include <asm/page.h>

static void __init prom_meminit_v0(void)
{
	struct linux_mlist_v0 *p;

	for (p = *(romvec->pv_v0mem.v0_available); p; p = p->theres_more)
		memblock_add(p->start_adr, p->num_bytes & PAGE_MASK);
}

static void __init prom_meminit_v2(void)
{
	struct linux_prom_registers reg[64];
	phandle node;
	int size, num_ents, i;

	node = prom_searchsiblings(prom_getchild(prom_root_node), "memory");
	size = prom_getproperty(node, "available", (char *) reg, sizeof(reg));
	num_ents = size / sizeof(struct linux_prom_registers);

	for (i = 0; i < num_ents; i++)
		memblock_add(reg[i].phys_addr, reg[i].reg_size & PAGE_MASK);
}

/* Initialize the memory lists based upon the prom version. */
void __init prom_meminit(void)
{
	switch (prom_vers) {
	case PROM_V0:
		prom_meminit_v0();
		break;

	case PROM_V2:
	case PROM_V3:
		prom_meminit_v2();
		break;

	default:
		break;
	}
}
