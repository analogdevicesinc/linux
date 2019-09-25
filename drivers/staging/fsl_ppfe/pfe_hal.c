// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

#include "pfe_mod.h"
#include "pfe/pfe.h"

/* A-010897: Jumbo frame is not supported */
extern bool pfe_errata_a010897;

#define PFE_RCR_MAX_FL_MASK	0xC000FFFF

void *cbus_base_addr;
void *ddr_base_addr;
unsigned long ddr_phys_base_addr;
unsigned int ddr_size;

static struct pe_info pe[MAX_PE];

/* Initializes the PFE library.
 * Must be called before using any of the library functions.
 *
 * @param[in] cbus_base		CBUS virtual base address (as mapped in
 * the host CPU address space)
 * @param[in] ddr_base		PFE DDR range virtual base address (as
 * mapped in the host CPU address space)
 * @param[in] ddr_phys_base	PFE DDR range physical base address (as
 * mapped in platform)
 * @param[in] size		PFE DDR range size (as defined by the host
 * software)
 */
void pfe_lib_init(void *cbus_base, void *ddr_base, unsigned long ddr_phys_base,
		  unsigned int size)
{
	cbus_base_addr = cbus_base;
	ddr_base_addr = ddr_base;
	ddr_phys_base_addr = ddr_phys_base;
	ddr_size = size;

	pe[CLASS0_ID].dmem_base_addr = CLASS_DMEM_BASE_ADDR(0);
	pe[CLASS0_ID].pmem_base_addr = CLASS_IMEM_BASE_ADDR(0);
	pe[CLASS0_ID].pmem_size = CLASS_IMEM_SIZE;
	pe[CLASS0_ID].mem_access_wdata = CLASS_MEM_ACCESS_WDATA;
	pe[CLASS0_ID].mem_access_addr = CLASS_MEM_ACCESS_ADDR;
	pe[CLASS0_ID].mem_access_rdata = CLASS_MEM_ACCESS_RDATA;

	pe[CLASS1_ID].dmem_base_addr = CLASS_DMEM_BASE_ADDR(1);
	pe[CLASS1_ID].pmem_base_addr = CLASS_IMEM_BASE_ADDR(1);
	pe[CLASS1_ID].pmem_size = CLASS_IMEM_SIZE;
	pe[CLASS1_ID].mem_access_wdata = CLASS_MEM_ACCESS_WDATA;
	pe[CLASS1_ID].mem_access_addr = CLASS_MEM_ACCESS_ADDR;
	pe[CLASS1_ID].mem_access_rdata = CLASS_MEM_ACCESS_RDATA;

	pe[CLASS2_ID].dmem_base_addr = CLASS_DMEM_BASE_ADDR(2);
	pe[CLASS2_ID].pmem_base_addr = CLASS_IMEM_BASE_ADDR(2);
	pe[CLASS2_ID].pmem_size = CLASS_IMEM_SIZE;
	pe[CLASS2_ID].mem_access_wdata = CLASS_MEM_ACCESS_WDATA;
	pe[CLASS2_ID].mem_access_addr = CLASS_MEM_ACCESS_ADDR;
	pe[CLASS2_ID].mem_access_rdata = CLASS_MEM_ACCESS_RDATA;

	pe[CLASS3_ID].dmem_base_addr = CLASS_DMEM_BASE_ADDR(3);
	pe[CLASS3_ID].pmem_base_addr = CLASS_IMEM_BASE_ADDR(3);
	pe[CLASS3_ID].pmem_size = CLASS_IMEM_SIZE;
	pe[CLASS3_ID].mem_access_wdata = CLASS_MEM_ACCESS_WDATA;
	pe[CLASS3_ID].mem_access_addr = CLASS_MEM_ACCESS_ADDR;
	pe[CLASS3_ID].mem_access_rdata = CLASS_MEM_ACCESS_RDATA;

	pe[CLASS4_ID].dmem_base_addr = CLASS_DMEM_BASE_ADDR(4);
	pe[CLASS4_ID].pmem_base_addr = CLASS_IMEM_BASE_ADDR(4);
	pe[CLASS4_ID].pmem_size = CLASS_IMEM_SIZE;
	pe[CLASS4_ID].mem_access_wdata = CLASS_MEM_ACCESS_WDATA;
	pe[CLASS4_ID].mem_access_addr = CLASS_MEM_ACCESS_ADDR;
	pe[CLASS4_ID].mem_access_rdata = CLASS_MEM_ACCESS_RDATA;

	pe[CLASS5_ID].dmem_base_addr = CLASS_DMEM_BASE_ADDR(5);
	pe[CLASS5_ID].pmem_base_addr = CLASS_IMEM_BASE_ADDR(5);
	pe[CLASS5_ID].pmem_size = CLASS_IMEM_SIZE;
	pe[CLASS5_ID].mem_access_wdata = CLASS_MEM_ACCESS_WDATA;
	pe[CLASS5_ID].mem_access_addr = CLASS_MEM_ACCESS_ADDR;
	pe[CLASS5_ID].mem_access_rdata = CLASS_MEM_ACCESS_RDATA;

	pe[TMU0_ID].dmem_base_addr = TMU_DMEM_BASE_ADDR(0);
	pe[TMU0_ID].pmem_base_addr = TMU_IMEM_BASE_ADDR(0);
	pe[TMU0_ID].pmem_size = TMU_IMEM_SIZE;
	pe[TMU0_ID].mem_access_wdata = TMU_MEM_ACCESS_WDATA;
	pe[TMU0_ID].mem_access_addr = TMU_MEM_ACCESS_ADDR;
	pe[TMU0_ID].mem_access_rdata = TMU_MEM_ACCESS_RDATA;

	pe[TMU1_ID].dmem_base_addr = TMU_DMEM_BASE_ADDR(1);
	pe[TMU1_ID].pmem_base_addr = TMU_IMEM_BASE_ADDR(1);
	pe[TMU1_ID].pmem_size = TMU_IMEM_SIZE;
	pe[TMU1_ID].mem_access_wdata = TMU_MEM_ACCESS_WDATA;
	pe[TMU1_ID].mem_access_addr = TMU_MEM_ACCESS_ADDR;
	pe[TMU1_ID].mem_access_rdata = TMU_MEM_ACCESS_RDATA;

	pe[TMU3_ID].dmem_base_addr = TMU_DMEM_BASE_ADDR(3);
	pe[TMU3_ID].pmem_base_addr = TMU_IMEM_BASE_ADDR(3);
	pe[TMU3_ID].pmem_size = TMU_IMEM_SIZE;
	pe[TMU3_ID].mem_access_wdata = TMU_MEM_ACCESS_WDATA;
	pe[TMU3_ID].mem_access_addr = TMU_MEM_ACCESS_ADDR;
	pe[TMU3_ID].mem_access_rdata = TMU_MEM_ACCESS_RDATA;

#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)
	pe[UTIL_ID].dmem_base_addr = UTIL_DMEM_BASE_ADDR;
	pe[UTIL_ID].mem_access_wdata = UTIL_MEM_ACCESS_WDATA;
	pe[UTIL_ID].mem_access_addr = UTIL_MEM_ACCESS_ADDR;
	pe[UTIL_ID].mem_access_rdata = UTIL_MEM_ACCESS_RDATA;
#endif
}

/* Writes a buffer to PE internal memory from the host
 * through indirect access registers.
 *
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID,
 * ..., UTIL_ID)
 * @param[in] src		Buffer source address
 * @param[in] mem_access_addr	DMEM destination address (must be 32bit
 * aligned)
 * @param[in] len		Number of bytes to copy
 */
void pe_mem_memcpy_to32(int id, u32 mem_access_addr, const void *src, unsigned
int len)
{
	u32 offset = 0, val, addr;
	unsigned int len32 = len >> 2;
	int i;

	addr = mem_access_addr | PE_MEM_ACCESS_WRITE |
		PE_MEM_ACCESS_BYTE_ENABLE(0, 4);

	for (i = 0; i < len32; i++, offset += 4, src += 4) {
		val = *(u32 *)src;
		writel(cpu_to_be32(val), pe[id].mem_access_wdata);
		writel(addr + offset, pe[id].mem_access_addr);
	}

	len = (len & 0x3);
	if (len) {
		val = 0;

		addr = (mem_access_addr | PE_MEM_ACCESS_WRITE |
			PE_MEM_ACCESS_BYTE_ENABLE(0, len)) + offset;

		for (i = 0; i < len; i++, src++)
			val |= (*(u8 *)src) << (8 * i);

		writel(cpu_to_be32(val), pe[id].mem_access_wdata);
		writel(addr, pe[id].mem_access_addr);
	}
}

/* Writes a buffer to PE internal data memory (DMEM) from the host
 * through indirect access registers.
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID,
 * ..., UTIL_ID)
 * @param[in] src		Buffer source address
 * @param[in] dst		DMEM destination address (must be 32bit
 * aligned)
 * @param[in] len		Number of bytes to copy
 */
void pe_dmem_memcpy_to32(int id, u32 dst, const void *src, unsigned int len)
{
	pe_mem_memcpy_to32(id, pe[id].dmem_base_addr | dst |
				PE_MEM_ACCESS_DMEM, src, len);
}

/* Writes a buffer to PE internal program memory (PMEM) from the host
 * through indirect access registers.
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID,
 * ..., TMU3_ID)
 * @param[in] src		Buffer source address
 * @param[in] dst		PMEM destination address (must be 32bit
 * aligned)
 * @param[in] len		Number of bytes to copy
 */
void pe_pmem_memcpy_to32(int id, u32 dst, const void *src, unsigned int len)
{
	pe_mem_memcpy_to32(id, pe[id].pmem_base_addr | (dst & (pe[id].pmem_size
				- 1)) | PE_MEM_ACCESS_IMEM, src, len);
}

/* Reads PE internal program memory (IMEM) from the host
 * through indirect access registers.
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID,
 * ..., TMU3_ID)
 * @param[in] addr		PMEM read address (must be aligned on size)
 * @param[in] size		Number of bytes to read (maximum 4, must not
 * cross 32bit boundaries)
 * @return			the data read (in PE endianness, i.e BE).
 */
u32 pe_pmem_read(int id, u32 addr, u8 size)
{
	u32 offset = addr & 0x3;
	u32 mask = 0xffffffff >> ((4 - size) << 3);
	u32 val;

	addr = pe[id].pmem_base_addr | ((addr & ~0x3) & (pe[id].pmem_size - 1))
		| PE_MEM_ACCESS_IMEM | PE_MEM_ACCESS_BYTE_ENABLE(offset, size);

	writel(addr, pe[id].mem_access_addr);
	val = be32_to_cpu(readl(pe[id].mem_access_rdata));

	return (val >> (offset << 3)) & mask;
}

/* Writes PE internal data memory (DMEM) from the host
 * through indirect access registers.
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID,
 * ..., UTIL_ID)
 * @param[in] addr		DMEM write address (must be aligned on size)
 * @param[in] val		Value to write (in PE endianness, i.e BE)
 * @param[in] size		Number of bytes to write (maximum 4, must not
 * cross 32bit boundaries)
 */
void pe_dmem_write(int id, u32 val, u32 addr, u8 size)
{
	u32 offset = addr & 0x3;

	addr = pe[id].dmem_base_addr | (addr & ~0x3) | PE_MEM_ACCESS_WRITE |
		PE_MEM_ACCESS_DMEM | PE_MEM_ACCESS_BYTE_ENABLE(offset, size);

	/* Indirect access interface is byte swapping data being written */
	writel(cpu_to_be32(val << (offset << 3)), pe[id].mem_access_wdata);
	writel(addr, pe[id].mem_access_addr);
}

/* Reads PE internal data memory (DMEM) from the host
 * through indirect access registers.
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID,
 * ..., UTIL_ID)
 * @param[in] addr		DMEM read address (must be aligned on size)
 * @param[in] size		Number of bytes to read (maximum 4, must not
 * cross 32bit boundaries)
 * @return			the data read (in PE endianness, i.e BE).
 */
u32 pe_dmem_read(int id, u32 addr, u8 size)
{
	u32 offset = addr & 0x3;
	u32 mask = 0xffffffff >> ((4 - size) << 3);
	u32 val;

	addr = pe[id].dmem_base_addr | (addr & ~0x3) | PE_MEM_ACCESS_DMEM |
			PE_MEM_ACCESS_BYTE_ENABLE(offset, size);

	writel(addr, pe[id].mem_access_addr);

	/* Indirect access interface is byte swapping data being read */
	val = be32_to_cpu(readl(pe[id].mem_access_rdata));

	return (val >> (offset << 3)) & mask;
}

/* This function is used to write to CLASS internal bus peripherals (ccu,
 * pe-lem) from the host
 * through indirect access registers.
 * @param[in]	val	value to write
 * @param[in]	addr	Address to write to (must be aligned on size)
 * @param[in]	size	Number of bytes to write (1, 2 or 4)
 *
 */
void class_bus_write(u32 val, u32 addr, u8 size)
{
	u32 offset = addr & 0x3;

	writel((addr & CLASS_BUS_ACCESS_BASE_MASK), CLASS_BUS_ACCESS_BASE);

	addr = (addr & ~CLASS_BUS_ACCESS_BASE_MASK) | PE_MEM_ACCESS_WRITE |
			(size << 24);

	writel(cpu_to_be32(val << (offset << 3)), CLASS_BUS_ACCESS_WDATA);
	writel(addr, CLASS_BUS_ACCESS_ADDR);
}

/* Reads from CLASS internal bus peripherals (ccu, pe-lem) from the host
 * through indirect access registers.
 * @param[in] addr	Address to read from (must be aligned on size)
 * @param[in] size	Number of bytes to read (1, 2 or 4)
 * @return		the read data
 *
 */
u32 class_bus_read(u32 addr, u8 size)
{
	u32 offset = addr & 0x3;
	u32 mask = 0xffffffff >> ((4 - size) << 3);
	u32 val;

	writel((addr & CLASS_BUS_ACCESS_BASE_MASK), CLASS_BUS_ACCESS_BASE);

	addr = (addr & ~CLASS_BUS_ACCESS_BASE_MASK) | (size << 24);

	writel(addr, CLASS_BUS_ACCESS_ADDR);
	val = be32_to_cpu(readl(CLASS_BUS_ACCESS_RDATA));

	return (val >> (offset << 3)) & mask;
}

/* Writes data to the cluster memory (PE_LMEM)
 * @param[in] dst	PE LMEM destination address (must be 32bit aligned)
 * @param[in] src	Buffer source address
 * @param[in] len	Number of bytes to copy
 */
void class_pe_lmem_memcpy_to32(u32 dst, const void *src, unsigned int len)
{
	u32 len32 = len >> 2;
	int i;

	for (i = 0; i < len32; i++, src += 4, dst += 4)
		class_bus_write(*(u32 *)src, dst, 4);

	if (len & 0x2) {
		class_bus_write(*(u16 *)src, dst, 2);
		src += 2;
		dst += 2;
	}

	if (len & 0x1) {
		class_bus_write(*(u8 *)src, dst, 1);
		src++;
		dst++;
	}
}

/* Writes value to the cluster memory (PE_LMEM)
 * @param[in] dst	PE LMEM destination address (must be 32bit aligned)
 * @param[in] val	Value to write
 * @param[in] len	Number of bytes to write
 */
void class_pe_lmem_memset(u32 dst, int val, unsigned int len)
{
	u32 len32 = len >> 2;
	int i;

	val = val | (val << 8) | (val << 16) | (val << 24);

	for (i = 0; i < len32; i++, dst += 4)
		class_bus_write(val, dst, 4);

	if (len & 0x2) {
		class_bus_write(val, dst, 2);
		dst += 2;
	}

	if (len & 0x1) {
		class_bus_write(val, dst, 1);
		dst++;
	}
}

#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)

/* Writes UTIL program memory (DDR) from the host.
 *
 * @param[in] addr	Address to write (virtual, must be aligned on size)
 * @param[in] val		Value to write (in PE endianness, i.e BE)
 * @param[in] size		Number of bytes to write (2 or 4)
 */
static void util_pmem_write(u32 val, void *addr, u8 size)
{
	void *addr64 = (void *)((unsigned long)addr & ~0x7);
	unsigned long off = 8 - ((unsigned long)addr & 0x7) - size;

	/*
	 * IMEM should  be loaded as a 64bit swapped value in a 64bit aligned
	 * location
	 */
	if (size == 4)
		writel(be32_to_cpu(val), addr64 + off);
	else
		writew(be16_to_cpu((u16)val), addr64 + off);
}

/* Writes a buffer to UTIL program memory (DDR) from the host.
 *
 * @param[in] dst	Address to write (virtual, must be at least 16bit
 * aligned)
 * @param[in] src	Buffer to write (in PE endianness, i.e BE, must have
 * same alignment as dst)
 * @param[in] len	Number of bytes to write (must be at least 16bit
 * aligned)
 */
static void util_pmem_memcpy(void *dst, const void *src, unsigned int len)
{
	unsigned int len32;
	int i;

	if ((unsigned long)src & 0x2) {
		util_pmem_write(*(u16 *)src, dst, 2);
		src += 2;
		dst += 2;
		len -= 2;
	}

	len32 = len >> 2;

	for (i = 0; i < len32; i++, dst += 4, src += 4)
		util_pmem_write(*(u32 *)src, dst, 4);

	if (len & 0x2)
		util_pmem_write(*(u16 *)src, dst, len & 0x2);
}
#endif

/* Loads an elf section into pmem
 * Code needs to be at least 16bit aligned and only PROGBITS sections are
 * supported
 *
 * @param[in] id	PE identification (CLASS0_ID, ..., TMU0_ID, ...,
 * TMU3_ID)
 * @param[in] data	pointer to the elf firmware
 * @param[in] shdr	pointer to the elf section header
 *
 */
static int pe_load_pmem_section(int id, const void *data,
				struct elf32_shdr *shdr)
{
	u32 offset = be32_to_cpu(shdr->sh_offset);
	u32 addr = be32_to_cpu(shdr->sh_addr);
	u32 size = be32_to_cpu(shdr->sh_size);
	u32 type = be32_to_cpu(shdr->sh_type);

#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)
	if (id == UTIL_ID) {
		pr_err("%s: unsupported pmem section for UTIL\n",
		       __func__);
		return -EINVAL;
	}
#endif

	if (((unsigned long)(data + offset) & 0x3) != (addr & 0x3)) {
		pr_err(
			"%s: load address(%x) and elf file address(%lx) don't have the same alignment\n"
			, __func__, addr, (unsigned long)data + offset);

		return -EINVAL;
	}

	if (addr & 0x1) {
		pr_err("%s: load address(%x) is not 16bit aligned\n",
		       __func__, addr);
		return -EINVAL;
	}

	if (size & 0x1) {
		pr_err("%s: load size(%x) is not 16bit aligned\n",
		       __func__, size);
		return -EINVAL;
	}

	switch (type) {
	case SHT_PROGBITS:
		pe_pmem_memcpy_to32(id, addr, data + offset, size);

		break;

	default:
		pr_err("%s: unsupported section type(%x)\n", __func__,
		       type);
		return -EINVAL;
	}

	return 0;
}

/* Loads an elf section into dmem
 * Data needs to be at least 32bit aligned, NOBITS sections are correctly
 * initialized to 0
 *
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID,
 * ..., UTIL_ID)
 * @param[in] data		pointer to the elf firmware
 * @param[in] shdr		pointer to the elf section header
 *
 */
static int pe_load_dmem_section(int id, const void *data,
				struct elf32_shdr *shdr)
{
	u32 offset = be32_to_cpu(shdr->sh_offset);
	u32 addr = be32_to_cpu(shdr->sh_addr);
	u32 size = be32_to_cpu(shdr->sh_size);
	u32 type = be32_to_cpu(shdr->sh_type);
	u32 size32 = size >> 2;
	int i;

	if (((unsigned long)(data + offset) & 0x3) != (addr & 0x3)) {
		pr_err(
			"%s: load address(%x) and elf file address(%lx) don't have the same alignment\n",
			__func__, addr, (unsigned long)data + offset);

		return -EINVAL;
	}

	if (addr & 0x3) {
		pr_err("%s: load address(%x) is not 32bit aligned\n",
		       __func__, addr);
		return -EINVAL;
	}

	switch (type) {
	case SHT_PROGBITS:
		pe_dmem_memcpy_to32(id, addr, data + offset, size);
		break;

	case SHT_NOBITS:
		for (i = 0; i < size32; i++, addr += 4)
			pe_dmem_write(id, 0, addr, 4);

		if (size & 0x3)
			pe_dmem_write(id, 0, addr, size & 0x3);

		break;

	default:
		pr_err("%s: unsupported section type(%x)\n", __func__,
		       type);
		return -EINVAL;
	}

	return 0;
}

/* Loads an elf section into DDR
 * Data needs to be at least 32bit aligned, NOBITS sections are correctly
 * initialized to 0
 *
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID,
 * ..., UTIL_ID)
 * @param[in] data		pointer to the elf firmware
 * @param[in] shdr		pointer to the elf section header
 *
 */
static int pe_load_ddr_section(int id, const void *data,
			       struct elf32_shdr *shdr,
			       struct device *dev) {
	u32 offset = be32_to_cpu(shdr->sh_offset);
	u32 addr = be32_to_cpu(shdr->sh_addr);
	u32 size = be32_to_cpu(shdr->sh_size);
	u32 type = be32_to_cpu(shdr->sh_type);
	u32 flags = be32_to_cpu(shdr->sh_flags);

	switch (type) {
	case SHT_PROGBITS:
		if (flags & SHF_EXECINSTR) {
			if (id <= CLASS_MAX_ID) {
				/* DO the loading only once in DDR */
				if (id == CLASS0_ID) {
					pr_err(
						"%s: load address(%x) and elf file address(%lx) rcvd\n",
						__func__, addr,
						(unsigned long)data + offset);
					if (((unsigned long)(data + offset)
						& 0x3) != (addr & 0x3)) {
						pr_err(
							"%s: load address(%x) and elf file address(%lx) don't have the same alignment\n"
							, __func__, addr,
						(unsigned long)data + offset);

						return -EINVAL;
					}

					if (addr & 0x1) {
						pr_err(
							"%s: load address(%x) is not 16bit aligned\n"
							, __func__, addr);
						return -EINVAL;
					}

					if (size & 0x1) {
						pr_err(
							"%s: load length(%x) is not 16bit aligned\n"
							, __func__, size);
						return -EINVAL;
					}
					memcpy(DDR_PHYS_TO_VIRT(
						DDR_PFE_TO_PHYS(addr)),
						data + offset, size);
				}
#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)
			} else if (id == UTIL_ID) {
				if (((unsigned long)(data + offset) & 0x3)
					!= (addr & 0x3)) {
					pr_err(
						"%s: load address(%x) and elf file address(%lx) don't have the same alignment\n"
						, __func__, addr,
						(unsigned long)data + offset);

					return -EINVAL;
				}

				if (addr & 0x1) {
					pr_err(
						"%s: load address(%x) is not 16bit aligned\n"
						, __func__, addr);
					return -EINVAL;
				}

				if (size & 0x1) {
					pr_err(
						"%s: load length(%x) is not 16bit aligned\n"
						, __func__, size);
					return -EINVAL;
				}

				util_pmem_memcpy(DDR_PHYS_TO_VIRT(
							DDR_PFE_TO_PHYS(addr)),
							data + offset, size);
			}
#endif
			} else {
				pr_err(
					"%s: unsupported ddr section type(%x) for PE(%d)\n"
						, __func__, type, id);
				return -EINVAL;
			}

		} else {
			memcpy(DDR_PHYS_TO_VIRT(DDR_PFE_TO_PHYS(addr)), data
				+ offset, size);
		}

		break;

	case SHT_NOBITS:
		memset(DDR_PHYS_TO_VIRT(DDR_PFE_TO_PHYS(addr)), 0, size);

		break;

	default:
		pr_err("%s: unsupported section type(%x)\n", __func__,
		       type);
		return -EINVAL;
	}

	return 0;
}

/* Loads an elf section into pe lmem
 * Data needs to be at least 32bit aligned, NOBITS sections are correctly
 * initialized to 0
 *
 * @param[in] id		PE identification (CLASS0_ID,..., CLASS5_ID)
 * @param[in] data		pointer to the elf firmware
 * @param[in] shdr		pointer to the elf section header
 *
 */
static int pe_load_pe_lmem_section(int id, const void *data,
				   struct elf32_shdr *shdr)
{
	u32 offset = be32_to_cpu(shdr->sh_offset);
	u32 addr = be32_to_cpu(shdr->sh_addr);
	u32 size = be32_to_cpu(shdr->sh_size);
	u32 type = be32_to_cpu(shdr->sh_type);

	if (id > CLASS_MAX_ID) {
		pr_err(
			"%s: unsupported pe-lmem section type(%x) for PE(%d)\n",
			 __func__, type, id);
		return -EINVAL;
	}

	if (((unsigned long)(data + offset) & 0x3) != (addr & 0x3)) {
		pr_err(
			"%s: load address(%x) and elf file address(%lx) don't have the same alignment\n",
			__func__, addr, (unsigned long)data + offset);

		return -EINVAL;
	}

	if (addr & 0x3) {
		pr_err("%s: load address(%x) is not 32bit aligned\n",
		       __func__, addr);
		return -EINVAL;
	}

	switch (type) {
	case SHT_PROGBITS:
		class_pe_lmem_memcpy_to32(addr, data + offset, size);
		break;

	case SHT_NOBITS:
		class_pe_lmem_memset(addr, 0, size);
		break;

	default:
		pr_err("%s: unsupported section type(%x)\n", __func__,
		       type);
		return -EINVAL;
	}

	return 0;
}

/* Loads an elf section into a PE
 * For now only supports loading a section to dmem (all PE's), pmem (class and
 * tmu PE's),
 * DDDR (util PE code)
 *
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID,
 * ..., UTIL_ID)
 * @param[in] data		pointer to the elf firmware
 * @param[in] shdr		pointer to the elf section header
 *
 */
int pe_load_elf_section(int id, const void *data, struct elf32_shdr *shdr,
			struct device *dev) {
	u32 addr = be32_to_cpu(shdr->sh_addr);
	u32 size = be32_to_cpu(shdr->sh_size);

	if (IS_DMEM(addr, size))
		return pe_load_dmem_section(id, data, shdr);
	else if (IS_PMEM(addr, size))
		return pe_load_pmem_section(id, data, shdr);
	else if (IS_PFE_LMEM(addr, size))
		return 0;
	else if (IS_PHYS_DDR(addr, size))
		return pe_load_ddr_section(id, data, shdr, dev);
	else if (IS_PE_LMEM(addr, size))
		return pe_load_pe_lmem_section(id, data, shdr);

	pr_err("%s: unsupported memory range(%x)\n", __func__,
	       addr);
	return 0;
}

/**************************** BMU ***************************/

/* Initializes a BMU block.
 * @param[in] base	BMU block base address
 * @param[in] cfg	BMU configuration
 */
void bmu_init(void *base, struct BMU_CFG *cfg)
{
	bmu_disable(base);

	bmu_set_config(base, cfg);

	bmu_reset(base);
}

/* Resets a BMU block.
 * @param[in] base	BMU block base address
 */
void bmu_reset(void *base)
{
	writel(CORE_SW_RESET, base + BMU_CTRL);

	/* Wait for self clear */
	while (readl(base + BMU_CTRL) & CORE_SW_RESET)
		;
}

/* Enabled a BMU block.
 * @param[in] base	BMU block base address
 */
void bmu_enable(void *base)
{
	writel(CORE_ENABLE, base + BMU_CTRL);
}

/* Disables a BMU block.
 * @param[in] base	BMU block base address
 */
void bmu_disable(void *base)
{
	writel(CORE_DISABLE, base + BMU_CTRL);
}

/* Sets the configuration of a BMU block.
 * @param[in] base	BMU block base address
 * @param[in] cfg	BMU configuration
 */
void bmu_set_config(void *base, struct BMU_CFG *cfg)
{
	writel(cfg->baseaddr, base + BMU_UCAST_BASE_ADDR);
	writel(cfg->count & 0xffff, base + BMU_UCAST_CONFIG);
	writel(cfg->size & 0xffff, base + BMU_BUF_SIZE);

	/* Interrupts are never used */
	writel(cfg->low_watermark, base + BMU_LOW_WATERMARK);
	writel(cfg->high_watermark, base + BMU_HIGH_WATERMARK);
	writel(0x0, base + BMU_INT_ENABLE);
}

/**************************** MTIP GEMAC ***************************/

/* Enable Rx Checksum Engine. With this enabled, Frame with bad IP,
 *   TCP or UDP checksums are discarded
 *
 * @param[in] base	GEMAC base address.
 */
void gemac_enable_rx_checksum_offload(void *base)
{
	/*Do not find configuration to do this */
}

/* Disable Rx Checksum Engine.
 *
 * @param[in] base	GEMAC base address.
 */
void gemac_disable_rx_checksum_offload(void *base)
{
	/*Do not find configuration to do this */
}

/* GEMAC set speed.
 * @param[in] base	GEMAC base address
 * @param[in] speed	GEMAC speed (10, 100 or 1000 Mbps)
 */
void gemac_set_speed(void *base, enum mac_speed gem_speed)
{
	u32 ecr = readl(base + EMAC_ECNTRL_REG) & ~EMAC_ECNTRL_SPEED;
	u32 rcr = readl(base + EMAC_RCNTRL_REG) & ~EMAC_RCNTRL_RMII_10T;

	switch (gem_speed) {
	case SPEED_10M:
			rcr |= EMAC_RCNTRL_RMII_10T;
			break;

	case SPEED_1000M:
			ecr |= EMAC_ECNTRL_SPEED;
			break;

	case SPEED_100M:
	default:
			/*It is in 100M mode */
			break;
	}
	writel(ecr, (base + EMAC_ECNTRL_REG));
	writel(rcr, (base + EMAC_RCNTRL_REG));
}

/* GEMAC set duplex.
 * @param[in] base	GEMAC base address
 * @param[in] duplex	GEMAC duplex mode (Full, Half)
 */
void gemac_set_duplex(void *base, int duplex)
{
	if (duplex == DUPLEX_HALF) {
		writel(readl(base + EMAC_TCNTRL_REG) & ~EMAC_TCNTRL_FDEN, base
			+ EMAC_TCNTRL_REG);
		writel(readl(base + EMAC_RCNTRL_REG) | EMAC_RCNTRL_DRT, (base
			+ EMAC_RCNTRL_REG));
	} else{
		writel(readl(base + EMAC_TCNTRL_REG) | EMAC_TCNTRL_FDEN, base
			+ EMAC_TCNTRL_REG);
		writel(readl(base + EMAC_RCNTRL_REG) & ~EMAC_RCNTRL_DRT, (base
			+ EMAC_RCNTRL_REG));
	}
}

/* GEMAC set mode.
 * @param[in] base	GEMAC base address
 * @param[in] mode	GEMAC operation mode (MII, RMII, RGMII, SGMII)
 */
void gemac_set_mode(void *base, int mode)
{
	u32 val = readl(base + EMAC_RCNTRL_REG);

	/*Remove loopbank*/
	val &= ~EMAC_RCNTRL_LOOP;

	/* Enable flow control and MII mode.PFE firmware always expects
       CRC should be forwarded by MAC to validate CRC in software.*/
	val |= (EMAC_RCNTRL_FCE | EMAC_RCNTRL_MII_MODE);

	writel(val, base + EMAC_RCNTRL_REG);
}

/* GEMAC enable function.
 * @param[in] base	GEMAC base address
 */
void gemac_enable(void *base)
{
	writel(readl(base + EMAC_ECNTRL_REG) | EMAC_ECNTRL_ETHER_EN, base +
		EMAC_ECNTRL_REG);
}

/* GEMAC disable function.
 * @param[in] base	GEMAC base address
 */
void gemac_disable(void *base)
{
	writel(readl(base + EMAC_ECNTRL_REG) & ~EMAC_ECNTRL_ETHER_EN, base +
		EMAC_ECNTRL_REG);
}

/* GEMAC TX disable function.
 * @param[in] base	GEMAC base address
 */
void gemac_tx_disable(void *base)
{
	writel(readl(base + EMAC_TCNTRL_REG) | EMAC_TCNTRL_GTS, base +
		EMAC_TCNTRL_REG);
}

void gemac_tx_enable(void *base)
{
	writel(readl(base + EMAC_TCNTRL_REG) & ~EMAC_TCNTRL_GTS, base +
			EMAC_TCNTRL_REG);
}

/* Sets the hash register of the MAC.
 * This register is used for matching unicast and multicast frames.
 *
 * @param[in] base	GEMAC base address.
 * @param[in] hash	64-bit hash to be configured.
 */
void gemac_set_hash(void *base, struct pfe_mac_addr *hash)
{
	writel(hash->bottom,  base + EMAC_GALR);
	writel(hash->top, base + EMAC_GAUR);
}

void gemac_set_laddrN(void *base, struct pfe_mac_addr *address,
		      unsigned int entry_index)
{
	if ((entry_index < 1) || (entry_index > EMAC_SPEC_ADDR_MAX))
		return;

	entry_index = entry_index - 1;
	if (entry_index < 1) {
		writel(htonl(address->bottom),  base + EMAC_PHY_ADDR_LOW);
		writel((htonl(address->top) | 0x8808), base +
			EMAC_PHY_ADDR_HIGH);
	} else {
		writel(htonl(address->bottom),  base + ((entry_index - 1) * 8)
			+ EMAC_SMAC_0_0);
		writel((htonl(address->top) | 0x8808), base + ((entry_index -
			1) * 8) + EMAC_SMAC_0_1);
	}
}

void gemac_clear_laddrN(void *base, unsigned int entry_index)
{
	if ((entry_index < 1) || (entry_index > EMAC_SPEC_ADDR_MAX))
		return;

	entry_index = entry_index - 1;
	if (entry_index < 1) {
		writel(0, base + EMAC_PHY_ADDR_LOW);
		writel(0, base + EMAC_PHY_ADDR_HIGH);
	} else {
		writel(0,  base + ((entry_index - 1) * 8) + EMAC_SMAC_0_0);
		writel(0, base + ((entry_index - 1) * 8) + EMAC_SMAC_0_1);
	}
}

/* Set the loopback mode of the MAC.  This can be either no loopback for
 * normal operation, local loopback through MAC internal loopback module or PHY
 *   loopback for external loopback through a PHY.  This asserts the external
 * loop pin.
 *
 * @param[in] base	GEMAC base address.
 * @param[in] gem_loop	Loopback mode to be enabled. LB_LOCAL - MAC
 * Loopback,
 *			LB_EXT - PHY Loopback.
 */
void gemac_set_loop(void *base, enum mac_loop gem_loop)
{
	pr_info("%s()\n", __func__);
	writel(readl(base + EMAC_RCNTRL_REG) | EMAC_RCNTRL_LOOP, (base +
		EMAC_RCNTRL_REG));
}

/* GEMAC allow frames
 * @param[in] base	GEMAC base address
 */
void gemac_enable_copy_all(void *base)
{
	writel(readl(base + EMAC_RCNTRL_REG) | EMAC_RCNTRL_PROM, (base +
		EMAC_RCNTRL_REG));
}

/* GEMAC do not allow frames
 * @param[in] base	GEMAC base address
 */
void gemac_disable_copy_all(void *base)
{
	writel(readl(base + EMAC_RCNTRL_REG) & ~EMAC_RCNTRL_PROM, (base +
		EMAC_RCNTRL_REG));
}

/* GEMAC allow broadcast function.
 * @param[in] base	GEMAC base address
 */
void gemac_allow_broadcast(void *base)
{
	writel(readl(base + EMAC_RCNTRL_REG) & ~EMAC_RCNTRL_BC_REJ, base +
		EMAC_RCNTRL_REG);
}

/* GEMAC no broadcast function.
 * @param[in] base	GEMAC base address
 */
void gemac_no_broadcast(void *base)
{
	writel(readl(base + EMAC_RCNTRL_REG) | EMAC_RCNTRL_BC_REJ, base +
		EMAC_RCNTRL_REG);
}

/* GEMAC enable 1536 rx function.
 * @param[in]	base	GEMAC base address
 */
void gemac_enable_1536_rx(void *base)
{
	/* Set 1536 as Maximum frame length */
	writel((readl(base + EMAC_RCNTRL_REG) & PFE_RCR_MAX_FL_MASK)
		| (1536 << 16), base +	EMAC_RCNTRL_REG);
}

/* GEMAC set rx Max frame length.
 * @param[in]	base	GEMAC base address
 * @param[in]	mtu	new mtu
 */
void gemac_set_rx_max_fl(void *base, int mtu)
{
	/* Set mtu as Maximum frame length */
	writel((readl(base + EMAC_RCNTRL_REG) & PFE_RCR_MAX_FL_MASK)
		| (mtu << 16), base + EMAC_RCNTRL_REG);
}

/* GEMAC enable stacked vlan function.
 * @param[in]	base	GEMAC base address
 */
void gemac_enable_stacked_vlan(void *base)
{
	/* MTIP doesn't support stacked vlan */
}

/* GEMAC enable pause rx function.
 * @param[in] base	GEMAC base address
 */
void gemac_enable_pause_rx(void *base)
{
	writel(readl(base + EMAC_RCNTRL_REG) | EMAC_RCNTRL_FCE,
	       base + EMAC_RCNTRL_REG);
}

/* GEMAC disable pause rx function.
 * @param[in] base	GEMAC base address
 */
void gemac_disable_pause_rx(void *base)
{
	writel(readl(base + EMAC_RCNTRL_REG) & ~EMAC_RCNTRL_FCE,
	       base + EMAC_RCNTRL_REG);
}

/* GEMAC enable pause tx function.
 * @param[in] base GEMAC base address
 */
void gemac_enable_pause_tx(void *base)
{
	writel(EMAC_RX_SECTION_EMPTY_V, base + EMAC_RX_SECTION_EMPTY);
}

/* GEMAC disable pause tx function.
 * @param[in] base GEMAC base address
 */
void gemac_disable_pause_tx(void *base)
{
	writel(0x0, base + EMAC_RX_SECTION_EMPTY);
}

/* GEMAC wol configuration
 * @param[in] base	GEMAC base address
 * @param[in] wol_conf	WoL register configuration
 */
void gemac_set_wol(void *base, u32 wol_conf)
{
	u32  val = readl(base + EMAC_ECNTRL_REG);

	if (wol_conf)
		val |= (EMAC_ECNTRL_MAGIC_ENA | EMAC_ECNTRL_SLEEP);
	else
		val &= ~(EMAC_ECNTRL_MAGIC_ENA | EMAC_ECNTRL_SLEEP);
	writel(val, base + EMAC_ECNTRL_REG);
}

/* Sets Gemac bus width to 64bit
 * @param[in] base       GEMAC base address
 * @param[in] width     gemac bus width to be set possible values are 32/64/128
 */
void gemac_set_bus_width(void *base, int width)
{
}

/* Sets Gemac configuration.
 * @param[in] base	GEMAC base address
 * @param[in] cfg	GEMAC configuration
 */
void gemac_set_config(void *base, struct gemac_cfg *cfg)
{
	/*GEMAC config taken from VLSI */
	writel(0x00000004, base + EMAC_TFWR_STR_FWD);
	writel(0x00000005, base + EMAC_RX_SECTION_FULL);

	if (pfe_errata_a010897)
		writel(0x0000076c, base + EMAC_TRUNC_FL);
	else
		writel(0x00003fff, base + EMAC_TRUNC_FL);

	writel(0x00000030, base + EMAC_TX_SECTION_EMPTY);
	writel(0x00000000, base + EMAC_MIB_CTRL_STS_REG);

	gemac_set_mode(base, cfg->mode);

	gemac_set_speed(base, cfg->speed);

	gemac_set_duplex(base, cfg->duplex);
}

/**************************** GPI ***************************/

/* Initializes a GPI block.
 * @param[in] base	GPI base address
 * @param[in] cfg	GPI configuration
 */
void gpi_init(void *base, struct gpi_cfg *cfg)
{
	gpi_reset(base);

	gpi_disable(base);

	gpi_set_config(base, cfg);
}

/* Resets a GPI block.
 * @param[in] base	GPI base address
 */
void gpi_reset(void *base)
{
	writel(CORE_SW_RESET, base + GPI_CTRL);
}

/* Enables a GPI block.
 * @param[in] base	GPI base address
 */
void gpi_enable(void *base)
{
	writel(CORE_ENABLE, base + GPI_CTRL);
}

/* Disables a GPI block.
 * @param[in] base	GPI base address
 */
void gpi_disable(void *base)
{
	writel(CORE_DISABLE, base + GPI_CTRL);
}

/* Sets the configuration of a GPI block.
 * @param[in] base	GPI base address
 * @param[in] cfg	GPI configuration
 */
void gpi_set_config(void *base, struct gpi_cfg *cfg)
{
	writel(CBUS_VIRT_TO_PFE(BMU1_BASE_ADDR + BMU_ALLOC_CTRL),	base
		+ GPI_LMEM_ALLOC_ADDR);
	writel(CBUS_VIRT_TO_PFE(BMU1_BASE_ADDR + BMU_FREE_CTRL),	base
		+ GPI_LMEM_FREE_ADDR);
	writel(CBUS_VIRT_TO_PFE(BMU2_BASE_ADDR + BMU_ALLOC_CTRL),	base
		+ GPI_DDR_ALLOC_ADDR);
	writel(CBUS_VIRT_TO_PFE(BMU2_BASE_ADDR + BMU_FREE_CTRL),	base
		+ GPI_DDR_FREE_ADDR);
	writel(CBUS_VIRT_TO_PFE(CLASS_INQ_PKTPTR), base + GPI_CLASS_ADDR);
	writel(DDR_HDR_SIZE, base + GPI_DDR_DATA_OFFSET);
	writel(LMEM_HDR_SIZE, base + GPI_LMEM_DATA_OFFSET);
	writel(0, base + GPI_LMEM_SEC_BUF_DATA_OFFSET);
	writel(0, base + GPI_DDR_SEC_BUF_DATA_OFFSET);
	writel((DDR_HDR_SIZE << 16) |	LMEM_HDR_SIZE,	base + GPI_HDR_SIZE);
	writel((DDR_BUF_SIZE << 16) |	LMEM_BUF_SIZE,	base + GPI_BUF_SIZE);

	writel(((cfg->lmem_rtry_cnt << 16) | (GPI_DDR_BUF_EN << 1) |
		GPI_LMEM_BUF_EN), base + GPI_RX_CONFIG);
	writel(cfg->tmlf_txthres, base + GPI_TMLF_TX);
	writel(cfg->aseq_len,	base + GPI_DTX_ASEQ);
	writel(1, base + GPI_TOE_CHKSUM_EN);

	if (cfg->mtip_pause_reg) {
		writel(cfg->mtip_pause_reg, base + GPI_CSR_MTIP_PAUSE_REG);
		writel(EGPI_PAUSE_TIME, base + GPI_TX_PAUSE_TIME);
	}
}

/**************************** CLASSIFIER ***************************/

/* Initializes CLASSIFIER block.
 * @param[in] cfg	CLASSIFIER configuration
 */
void class_init(struct class_cfg *cfg)
{
	class_reset();

	class_disable();

	class_set_config(cfg);
}

/* Resets CLASSIFIER block.
 *
 */
void class_reset(void)
{
	writel(CORE_SW_RESET, CLASS_TX_CTRL);
}

/* Enables all CLASS-PE's cores.
 *
 */
void class_enable(void)
{
	writel(CORE_ENABLE, CLASS_TX_CTRL);
}

/* Disables all CLASS-PE's cores.
 *
 */
void class_disable(void)
{
	writel(CORE_DISABLE, CLASS_TX_CTRL);
}

/*
 * Sets the configuration of the CLASSIFIER block.
 * @param[in] cfg	CLASSIFIER configuration
 */
void class_set_config(struct class_cfg *cfg)
{
	u32 val;

	/* Initialize route table */
	if (!cfg->resume)
		memset(DDR_PHYS_TO_VIRT(cfg->route_table_baseaddr), 0, (1 <<
		cfg->route_table_hash_bits) * CLASS_ROUTE_SIZE);

#if !defined(LS1012A_PFE_RESET_WA)
	writel(cfg->pe_sys_clk_ratio,	CLASS_PE_SYS_CLK_RATIO);
#endif

	writel((DDR_HDR_SIZE << 16) | LMEM_HDR_SIZE,	CLASS_HDR_SIZE);
	writel(LMEM_BUF_SIZE,				CLASS_LMEM_BUF_SIZE);
	writel(CLASS_ROUTE_ENTRY_SIZE(CLASS_ROUTE_SIZE) |
		CLASS_ROUTE_HASH_SIZE(cfg->route_table_hash_bits),
		CLASS_ROUTE_HASH_ENTRY_SIZE);
	writel(HIF_PKT_CLASS_EN | HIF_PKT_OFFSET(sizeof(struct hif_hdr)),
	       CLASS_HIF_PARSE);

	val = HASH_CRC_PORT_IP | QB2BUS_LE;

#if defined(CONFIG_IP_ALIGNED)
	val |= IP_ALIGNED;
#endif

	/*
	 *  Class PE packet steering will only work if TOE mode, bridge fetch or
	 * route fetch are enabled (see class/qb_fet.v). Route fetch would
	 * trigger additional memory copies (likely from DDR because of hash
	 * table size, which cannot be reduced because PE software still
	 * relies on hash value computed in HW), so when not in TOE mode we
	 * simply enable HW bridge fetch even though we don't use it.
	 */
	if (cfg->toe_mode)
		val |= CLASS_TOE;
	else
		val |= HW_BRIDGE_FETCH;

	writel(val, CLASS_ROUTE_MULTI);

	writel(DDR_PHYS_TO_PFE(cfg->route_table_baseaddr),
	       CLASS_ROUTE_TABLE_BASE);
	writel(CLASS_PE0_RO_DM_ADDR0_VAL,		CLASS_PE0_RO_DM_ADDR0);
	writel(CLASS_PE0_RO_DM_ADDR1_VAL,		CLASS_PE0_RO_DM_ADDR1);
	writel(CLASS_PE0_QB_DM_ADDR0_VAL,		CLASS_PE0_QB_DM_ADDR0);
	writel(CLASS_PE0_QB_DM_ADDR1_VAL,		CLASS_PE0_QB_DM_ADDR1);
	writel(CBUS_VIRT_TO_PFE(TMU_PHY_INQ_PKTPTR),	CLASS_TM_INQ_ADDR);

	writel(23, CLASS_AFULL_THRES);
	writel(23, CLASS_TSQ_FIFO_THRES);

	writel(24, CLASS_MAX_BUF_CNT);
	writel(24, CLASS_TSQ_MAX_CNT);
}

/**************************** TMU ***************************/

void tmu_reset(void)
{
	writel(SW_RESET, TMU_CTRL);
}

/* Initializes TMU block.
 * @param[in] cfg	TMU configuration
 */
void tmu_init(struct tmu_cfg *cfg)
{
	int q, phyno;

	tmu_disable(0xF);
	mdelay(10);

#if !defined(LS1012A_PFE_RESET_WA)
	/* keep in soft reset */
	writel(SW_RESET, TMU_CTRL);
#endif
	writel(0x3, TMU_SYS_GENERIC_CONTROL);
	writel(750, TMU_INQ_WATERMARK);
	writel(CBUS_VIRT_TO_PFE(EGPI1_BASE_ADDR +
		GPI_INQ_PKTPTR),	TMU_PHY0_INQ_ADDR);
	writel(CBUS_VIRT_TO_PFE(EGPI2_BASE_ADDR +
		GPI_INQ_PKTPTR),	TMU_PHY1_INQ_ADDR);
	writel(CBUS_VIRT_TO_PFE(HGPI_BASE_ADDR +
		GPI_INQ_PKTPTR),	TMU_PHY3_INQ_ADDR);
	writel(CBUS_VIRT_TO_PFE(HIF_NOCPY_RX_INQ0_PKTPTR), TMU_PHY4_INQ_ADDR);
	writel(CBUS_VIRT_TO_PFE(UTIL_INQ_PKTPTR), TMU_PHY5_INQ_ADDR);
	writel(CBUS_VIRT_TO_PFE(BMU2_BASE_ADDR + BMU_FREE_CTRL),
	       TMU_BMU_INQ_ADDR);

	writel(0x3FF,	TMU_TDQ0_SCH_CTRL);	/*
						 * enabling all 10
						 * schedulers [9:0] of each TDQ
						 */
	writel(0x3FF,	TMU_TDQ1_SCH_CTRL);
	writel(0x3FF,	TMU_TDQ3_SCH_CTRL);

#if !defined(LS1012A_PFE_RESET_WA)
	writel(cfg->pe_sys_clk_ratio,	TMU_PE_SYS_CLK_RATIO);
#endif

#if !defined(LS1012A_PFE_RESET_WA)
	writel(DDR_PHYS_TO_PFE(cfg->llm_base_addr),	TMU_LLM_BASE_ADDR);
	/* Extra packet pointers will be stored from this address onwards */

	writel(cfg->llm_queue_len,	TMU_LLM_QUE_LEN);
	writel(5,			TMU_TDQ_IIFG_CFG);
	writel(DDR_BUF_SIZE,		TMU_BMU_BUF_SIZE);

	writel(0x0,			TMU_CTRL);

	/* MEM init */
	pr_info("%s: mem init\n", __func__);
	writel(MEM_INIT,	TMU_CTRL);

	while (!(readl(TMU_CTRL) & MEM_INIT_DONE))
		;

	/* LLM init */
	pr_info("%s: lmem init\n", __func__);
	writel(LLM_INIT,	TMU_CTRL);

	while (!(readl(TMU_CTRL) & LLM_INIT_DONE))
		;
#endif
	/* set up each queue for tail drop */
	for (phyno = 0; phyno < 4; phyno++) {
		if (phyno == 2)
			continue;
		for (q = 0; q < 16; q++) {
			u32 qdepth;

			writel((phyno << 8) | q, TMU_TEQ_CTRL);
			writel(1 << 22, TMU_TEQ_QCFG); /*Enable tail drop */

			if (phyno == 3)
				qdepth = DEFAULT_TMU3_QDEPTH;
			else
				qdepth = (q == 0) ? DEFAULT_Q0_QDEPTH :
						DEFAULT_MAX_QDEPTH;

			/* LOG: 68855 */
			/*
			 * The following is a workaround for the reordered
			 * packet and BMU2 buffer leakage issue.
			 */
			if (CHIP_REVISION() == 0)
				qdepth = 31;

			writel(qdepth << 18, TMU_TEQ_HW_PROB_CFG2);
			writel(qdepth >> 14, TMU_TEQ_HW_PROB_CFG3);
		}
	}

#ifdef CFG_LRO
	/* Set TMU-3 queue 5 (LRO) in no-drop mode */
	writel((3 << 8) | TMU_QUEUE_LRO, TMU_TEQ_CTRL);
	writel(0, TMU_TEQ_QCFG);
#endif

	writel(0x05, TMU_TEQ_DISABLE_DROPCHK);

	writel(0x0, TMU_CTRL);
}

/* Enables TMU-PE cores.
 * @param[in] pe_mask	TMU PE mask
 */
void tmu_enable(u32 pe_mask)
{
	writel(readl(TMU_TX_CTRL) | (pe_mask & 0xF), TMU_TX_CTRL);
}

/* Disables TMU cores.
 * @param[in] pe_mask	TMU PE mask
 */
void tmu_disable(u32 pe_mask)
{
	writel(readl(TMU_TX_CTRL) & ~(pe_mask & 0xF), TMU_TX_CTRL);
}

/* This will return the tmu queue status
 * @param[in] if_id	gem interface id or TMU index
 * @return		returns the bit mask of busy queues, zero means all
 * queues are empty
 */
u32 tmu_qstatus(u32 if_id)
{
	return cpu_to_be32(pe_dmem_read(TMU0_ID + if_id, TMU_DM_PESTATUS +
		offsetof(struct pe_status, tmu_qstatus), 4));
}

u32 tmu_pkts_processed(u32 if_id)
{
	return cpu_to_be32(pe_dmem_read(TMU0_ID + if_id, TMU_DM_PESTATUS +
		offsetof(struct pe_status, rx), 4));
}

/**************************** UTIL ***************************/

/* Resets UTIL block.
 */
void util_reset(void)
{
	writel(CORE_SW_RESET, UTIL_TX_CTRL);
}

/* Initializes UTIL block.
 * @param[in] cfg	UTIL configuration
 */
void util_init(struct util_cfg *cfg)
{
	writel(cfg->pe_sys_clk_ratio,   UTIL_PE_SYS_CLK_RATIO);
}

/* Enables UTIL-PE core.
 *
 */
void util_enable(void)
{
	writel(CORE_ENABLE, UTIL_TX_CTRL);
}

/* Disables UTIL-PE core.
 *
 */
void util_disable(void)
{
	writel(CORE_DISABLE, UTIL_TX_CTRL);
}

/**************************** HIF ***************************/
/* Initializes HIF copy block.
 *
 */
void hif_init(void)
{
	/*Initialize HIF registers*/
	writel((HIF_RX_POLL_CTRL_CYCLE << 16) | HIF_TX_POLL_CTRL_CYCLE,
	       HIF_POLL_CTRL);
}

/* Enable hif tx DMA and interrupt
 *
 */
void hif_tx_enable(void)
{
	writel(HIF_CTRL_DMA_EN, HIF_TX_CTRL);
	writel((readl(HIF_INT_ENABLE) | HIF_INT_EN | HIF_TXPKT_INT_EN),
	       HIF_INT_ENABLE);
}

/* Disable hif tx DMA and interrupt
 *
 */
void hif_tx_disable(void)
{
	u32	hif_int;

	writel(0, HIF_TX_CTRL);

	hif_int = readl(HIF_INT_ENABLE);
	hif_int &= HIF_TXPKT_INT_EN;
	writel(hif_int, HIF_INT_ENABLE);
}

/* Enable hif rx DMA and interrupt
 *
 */
void hif_rx_enable(void)
{
	hif_rx_dma_start();
	writel((readl(HIF_INT_ENABLE) | HIF_INT_EN | HIF_RXPKT_INT_EN),
	       HIF_INT_ENABLE);
}

/* Disable hif rx DMA and interrupt
 *
 */
void hif_rx_disable(void)
{
	u32	hif_int;

	writel(0, HIF_RX_CTRL);

	hif_int = readl(HIF_INT_ENABLE);
	hif_int &= HIF_RXPKT_INT_EN;
	writel(hif_int, HIF_INT_ENABLE);
}
