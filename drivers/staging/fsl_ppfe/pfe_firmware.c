// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

/*
 * @file
 * Contains all the functions to handle parsing and loading of PE firmware
 * files.
 */
#include <linux/firmware.h>

#include "pfe_mod.h"
#include "pfe_firmware.h"
#include "pfe/pfe.h"
#include <linux/of_platform.h>
#include <linux/of_address.h>

static struct elf32_shdr *get_elf_section_header(const u8 *fw,
						 const char *section)
{
	struct elf32_hdr *elf_hdr = (struct elf32_hdr *)fw;
	struct elf32_shdr *shdr;
	struct elf32_shdr *shdr_shstr;
	Elf32_Off e_shoff = be32_to_cpu(elf_hdr->e_shoff);
	Elf32_Half e_shentsize = be16_to_cpu(elf_hdr->e_shentsize);
	Elf32_Half e_shnum = be16_to_cpu(elf_hdr->e_shnum);
	Elf32_Half e_shstrndx = be16_to_cpu(elf_hdr->e_shstrndx);
	Elf32_Off shstr_offset;
	Elf32_Word sh_name;
	const char *name;
	int i;

	/* Section header strings */
	shdr_shstr = (struct elf32_shdr *)((u8 *)elf_hdr + e_shoff + e_shstrndx
					* e_shentsize);
	shstr_offset = be32_to_cpu(shdr_shstr->sh_offset);

	for (i = 0; i < e_shnum; i++) {
		shdr = (struct elf32_shdr *)((u8 *)elf_hdr + e_shoff
					     + i * e_shentsize);

		sh_name = be32_to_cpu(shdr->sh_name);

		name = (const char *)((u8 *)elf_hdr + shstr_offset + sh_name);

		if (!strcmp(name, section))
			return shdr;
	}

	pr_err("%s: didn't find section %s\n", __func__, section);

	return NULL;
}

#if defined(CFG_DIAGS)
static int pfe_get_diags_info(const u8 *fw, struct pfe_diags_info
				*diags_info)
{
	struct elf32_shdr *shdr;
	unsigned long offset, size;

	shdr = get_elf_section_header(fw, ".pfe_diags_str");
	if (shdr) {
		offset = be32_to_cpu(shdr->sh_offset);
		size = be32_to_cpu(shdr->sh_size);
		diags_info->diags_str_base = be32_to_cpu(shdr->sh_addr);
		diags_info->diags_str_size = size;
		diags_info->diags_str_array = kmalloc(size, GFP_KERNEL);
		memcpy(diags_info->diags_str_array, fw + offset, size);

		return 0;
	} else {
		return -1;
	}
}
#endif

static void pfe_check_version_info(const u8 *fw)
{
	/*static char *version = NULL;*/
	const u8 *elf_data = fw;
	static char *version;

	struct elf32_shdr *shdr = get_elf_section_header(fw, ".version");

	if (shdr) {
		if (!version) {
			/*
			 * this is the first fw we load, use its version
			 * string as reference (whatever it is)
			 */
			version = (char *)(elf_data +
					be32_to_cpu(shdr->sh_offset));

			pr_info("PFE binary version: %s\n", version);
		} else {
			/*
			 * already have loaded at least one firmware, check
			 * sequence can start now
			 */
			if (strcmp(version, (char *)(elf_data +
				be32_to_cpu(shdr->sh_offset)))) {
				pr_info(
				"WARNING: PFE firmware binaries from incompatible version\n");
			}
		}
	} else {
		/*
		 * version cannot be verified, a potential issue that should
		 * be reported
		 */
		pr_info(
			 "WARNING: PFE firmware binaries from incompatible version\n");
	}
}

/* PFE elf firmware loader.
 * Loads an elf firmware image into a list of PE's (specified using a bitmask)
 *
 * @param pe_mask	Mask of PE id's to load firmware to
 * @param fw		Pointer to the firmware image
 *
 * @return		0 on success, a negative value on error
 *
 */
int pfe_load_elf(int pe_mask, const u8 *fw, struct pfe *pfe)
{
	struct elf32_hdr *elf_hdr = (struct elf32_hdr *)fw;
	Elf32_Half sections = be16_to_cpu(elf_hdr->e_shnum);
	struct elf32_shdr *shdr = (struct elf32_shdr *)(fw +
					be32_to_cpu(elf_hdr->e_shoff));
	int id, section;
	int rc;

	pr_info("%s\n", __func__);

	/* Some sanity checks */
	if (strncmp(&elf_hdr->e_ident[EI_MAG0], ELFMAG, SELFMAG)) {
		pr_err("%s: incorrect elf magic number\n", __func__);
		return -EINVAL;
	}

	if (elf_hdr->e_ident[EI_CLASS] != ELFCLASS32) {
		pr_err("%s: incorrect elf class(%x)\n", __func__,
		       elf_hdr->e_ident[EI_CLASS]);
		return -EINVAL;
	}

	if (elf_hdr->e_ident[EI_DATA] != ELFDATA2MSB) {
		pr_err("%s: incorrect elf data(%x)\n", __func__,
		       elf_hdr->e_ident[EI_DATA]);
		return -EINVAL;
	}

	if (be16_to_cpu(elf_hdr->e_type) != ET_EXEC) {
		pr_err("%s: incorrect elf file type(%x)\n", __func__,
		       be16_to_cpu(elf_hdr->e_type));
		return -EINVAL;
	}

	for (section = 0; section < sections; section++, shdr++) {
		if (!(be32_to_cpu(shdr->sh_flags) & (SHF_WRITE | SHF_ALLOC |
			SHF_EXECINSTR)))
			continue;

		for (id = 0; id < MAX_PE; id++)
			if (pe_mask & (1 << id)) {
				rc = pe_load_elf_section(id, elf_hdr, shdr,
							 pfe->dev);
				if (rc < 0)
					goto err;
			}
	}

	pfe_check_version_info(fw);

	return 0;

err:
	return rc;
}

int get_firmware_in_fdt(const u8 **pe_fw, const char *name)
{
	struct device_node *np;
	const unsigned int *len;
	const void *data;

	if (!strcmp(name, CLASS_FIRMWARE_FILENAME)) {
		/* The firmware should be inside the device tree. */
		np = of_find_compatible_node(NULL, NULL,
					     "fsl,pfe-class-firmware");
		if (!np) {
			pr_info("Failed to find the node\n");
			return -ENOENT;
		}

		data = of_get_property(np, "fsl,class-firmware", NULL);
		if (data) {
			len = of_get_property(np, "length", NULL);
			pr_info("CLASS fw of length %d bytes loaded from FDT.\n",
				be32_to_cpu(*len));
		} else {
			pr_info("fsl,class-firmware not found!!!!\n");
			return -ENOENT;
		}
		of_node_put(np);
		*pe_fw = data;
	} else if (!strcmp(name, TMU_FIRMWARE_FILENAME)) {
		np = of_find_compatible_node(NULL, NULL,
					     "fsl,pfe-tmu-firmware");
		if (!np) {
			pr_info("Failed to find the node\n");
			return -ENOENT;
		}

		data = of_get_property(np, "fsl,tmu-firmware", NULL);
		if (data) {
			len = of_get_property(np, "length", NULL);
			pr_info("TMU fw of length %d bytes loaded from FDT.\n",
				be32_to_cpu(*len));
		} else {
			pr_info("fsl,tmu-firmware not found!!!!\n");
			return -ENOENT;
		}
		of_node_put(np);
		*pe_fw = data;
	} else if (!strcmp(name, UTIL_FIRMWARE_FILENAME)) {
		np = of_find_compatible_node(NULL, NULL,
					     "fsl,pfe-util-firmware");
		if (!np) {
			pr_info("Failed to find the node\n");
			return -ENOENT;
		}

		data = of_get_property(np, "fsl,util-firmware", NULL);
		if (data) {
			len = of_get_property(np, "length", NULL);
			pr_info("UTIL fw of length %d bytes loaded from FDT.\n",
				be32_to_cpu(*len));
		} else {
			pr_info("fsl,util-firmware not found!!!!\n");
			return -ENOENT;
		}
		of_node_put(np);
		*pe_fw = data;
	} else {
		pr_err("firmware:%s not known\n", name);
		return -EINVAL;
	}

	return 0;
}

/* PFE firmware initialization.
 * Loads different firmware files from filesystem.
 * Initializes PE IMEM/DMEM and UTIL-PE DDR
 * Initializes control path symbol addresses (by looking them up in the elf
 * firmware files
 * Takes PE's out of reset
 *
 * @return	0 on success, a negative value on error
 *
 */
int pfe_firmware_init(struct pfe *pfe)
{
	const struct firmware *class_fw, *tmu_fw;
	const u8 *class_elf_fw, *tmu_elf_fw;
	int rc = 0, fs_load = 0;
#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)
	const struct firmware *util_fw;
	const u8 *util_elf_fw;

#endif

	pr_info("%s\n", __func__);

#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)
	if (get_firmware_in_fdt(&class_elf_fw, CLASS_FIRMWARE_FILENAME) ||
	    get_firmware_in_fdt(&tmu_elf_fw, TMU_FIRMWARE_FILENAME) ||
	    get_firmware_in_fdt(&util_elf_fw, UTIL_FIRMWARE_FILENAME))
#else
	if (get_firmware_in_fdt(&class_elf_fw, CLASS_FIRMWARE_FILENAME) ||
	    get_firmware_in_fdt(&tmu_elf_fw, TMU_FIRMWARE_FILENAME))
#endif
	{
		pr_info("%s:PFE firmware not found in FDT.\n", __func__);
		pr_info("%s:Trying to load firmware from filesystem...!\n", __func__);

		/* look for firmware in filesystem...!*/
		fs_load = 1;
		if (request_firmware(&class_fw, CLASS_FIRMWARE_FILENAME, pfe->dev)) {
			pr_err("%s: request firmware %s failed\n", __func__,
			       CLASS_FIRMWARE_FILENAME);
			rc = -ETIMEDOUT;
			goto err0;
		}
		class_elf_fw = class_fw->data;

		if (request_firmware(&tmu_fw, TMU_FIRMWARE_FILENAME, pfe->dev)) {
			pr_err("%s: request firmware %s failed\n", __func__,
			       TMU_FIRMWARE_FILENAME);
			rc = -ETIMEDOUT;
			goto err1;
		}
		tmu_elf_fw = tmu_fw->data;

#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)
		if (request_firmware(&util_fw, UTIL_FIRMWARE_FILENAME, pfe->dev)) {
			pr_err("%s: request firmware %s failed\n", __func__,
			       UTIL_FIRMWARE_FILENAME);
			rc = -ETIMEDOUT;
			goto err2;
		}
		util_elf_fw = util_fw->data;
#endif
	}

	rc = pfe_load_elf(CLASS_MASK, class_elf_fw, pfe);
	if (rc < 0) {
		pr_err("%s: class firmware load failed\n", __func__);
		goto err3;
	}

#if defined(CFG_DIAGS)
	rc = pfe_get_diags_info(class_elf_fw, &pfe->diags.class_diags_info);
	if (rc < 0) {
		pr_warn(
			"PFE diags won't be available for class PEs\n");
		rc = 0;
	}
#endif

	rc = pfe_load_elf(TMU_MASK, tmu_elf_fw, pfe);
	if (rc < 0) {
		pr_err("%s: tmu firmware load failed\n", __func__);
		goto err3;
	}

#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)
	rc = pfe_load_elf(UTIL_MASK, util_elf_fw, pfe);
	if (rc < 0) {
		pr_err("%s: util firmware load failed\n", __func__);
		goto err3;
	}

#if defined(CFG_DIAGS)
	rc = pfe_get_diags_info(util_elf_fw, &pfe->diags.util_diags_info);
	if (rc < 0) {
		pr_warn(
			"PFE diags won't be available for util PE\n");
		rc = 0;
	}
#endif

	util_enable();
#endif

	tmu_enable(0xf);
	class_enable();

err3:
#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)
	if (fs_load)
		release_firmware(util_fw);
err2:
#endif
	if (fs_load)
		release_firmware(tmu_fw);

err1:
	if (fs_load)
		release_firmware(class_fw);

err0:
	return rc;
}

/* PFE firmware cleanup
 * Puts PE's in reset
 *
 *
 */
void pfe_firmware_exit(struct pfe *pfe)
{
	pr_info("%s\n", __func__);

	if (pe_reset_all(&pfe->ctrl) != 0)
		pr_err("Error: Failed to stop PEs, PFE reload may not work correctly\n");

	class_disable();
	tmu_disable(0xf);
#if !defined(CONFIG_FSL_PPFE_UTIL_DISABLED)
	util_disable();
#endif
}
