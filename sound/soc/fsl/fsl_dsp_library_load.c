// SPDX-License-Identifier: GPL-2.0+
// Copyright 2018 NXP
// Copyright (c) 2012-2013 by Tensilica Inc.

#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/elf.h>

#include "fsl_dsp.h"
#include "fsl_dsp_library_load.h"

static Elf32_Half xtlib_host_half(Elf32_Half v, int byteswap)
{
	return (byteswap) ? (v >> 8) | (v << 8) : v;
}

static Elf32_Word xtlib_host_word(Elf32_Word v, int byteswap)
{
	if (byteswap) {
		v = ((v & 0x00FF00FF) << 8) | ((v & 0xFF00FF00) >> 8);
		v = (v >> 16) | (v << 16);
	}
	return v;
}

static int xtlib_verify_magic(Elf32_Ehdr *header,
			      struct lib_info *lib_info)
{
	struct xtlib_loader_globals *xtlib_globals =
					&lib_info->xtlib_globals;
	Elf32_Byte magic_no;

	magic_no =  header->e_ident[EI_MAG0];
	if (magic_no != 0x7f)
		return -1;

	magic_no = header->e_ident[EI_MAG1];
	if (magic_no != 'E')
		return -1;

	magic_no = header->e_ident[EI_MAG2];
	if (magic_no != 'L')
		return -1;

	magic_no = header->e_ident[EI_MAG3];
	if (magic_no != 'F')
		return -1;

	if (header->e_ident[EI_CLASS] != ELFCLASS32)
		return -1;

	{
		/* determine byte order  */
		union {
			short s;
			char c[sizeof(short)];
		} u;

		u.s = 1;

		if (header->e_ident[EI_DATA] == ELFDATA2LSB)
			xtlib_globals->byteswap = u.c[sizeof(short) - 1] == 1;
		else if (header->e_ident[EI_DATA] == ELFDATA2MSB)
			xtlib_globals->byteswap = u.c[0] == 1;
		else
			return -1;
	}

	return 0;
}

static void xtlib_load_seg(Elf32_Phdr *pheader, void *src_addr, xt_ptr dst_addr,
			   struct lib_info *lib_info)
{
	struct xtlib_loader_globals *xtlib_globals =
					&lib_info->xtlib_globals;
	Elf32_Word bytes_to_copy = xtlib_host_word(pheader->p_filesz,
						xtlib_globals->byteswap);
	Elf32_Word bytes_to_zero = xtlib_host_word(pheader->p_memsz,
						xtlib_globals->byteswap)
						- bytes_to_copy;
	unsigned int i;
	char *src_back, *dst_back;

	void *zero_addr = (void *)dst_addr + bytes_to_copy;

	if (bytes_to_copy > 0) {
	//	memcpy((void *)(dst_addr), src_addr, bytes_to_copy);
		src_back = (char *)src_addr;
		dst_back = (char *)dst_addr;
		for (i = 0; i < bytes_to_copy; i++)
			*dst_back++ = *src_back++;
	}

	if (bytes_to_zero > 0) {
	//	memset(zero_addr, 0, bytes_to_zero);
		dst_back = (char *)zero_addr;
		for (i = 0; i < bytes_to_zero; i++)
			*dst_back++ = 0;
	}
}

#define xtlib_xt_half  xtlib_host_half
#define xtlib_xt_word  xtlib_host_word

static xt_ptr align_ptr(xt_ptr ptr, xt_uint align)
{
	return (xt_ptr)(((xt_uint)ptr + align - 1) & ~(align - 1));
}

static xt_ptr xt_ptr_offs(xt_ptr base, Elf32_Word offs,
			  struct lib_info *lib_info)
{
	struct xtlib_loader_globals *xtlib_globals =
					&lib_info->xtlib_globals;

	return (xt_ptr)xtlib_xt_word((xt_uint)base +
		xtlib_host_word(offs, xtlib_globals->byteswap),
			xtlib_globals->byteswap);
}

static Elf32_Dyn *find_dynamic_info(Elf32_Ehdr *eheader,
				    struct lib_info *lib_info)
{
	char *base_addr = (char *)eheader;
	struct xtlib_loader_globals *xtlib_globals =
					&lib_info->xtlib_globals;
	Elf32_Phdr *pheader = (Elf32_Phdr *)(base_addr +
			xtlib_host_word(eheader->e_phoff,
					xtlib_globals->byteswap));

	int seg = 0;
	int num = xtlib_host_half(eheader->e_phnum, xtlib_globals->byteswap);

	while (seg < num) {
		if (xtlib_host_word(pheader[seg].p_type,
				    xtlib_globals->byteswap) == PT_DYNAMIC) {
			return (Elf32_Dyn *)(base_addr +
				xtlib_host_word(pheader[seg].p_offset,
						xtlib_globals->byteswap));
		}
		seg++;
	}
	return 0;
}

static int find_align(Elf32_Ehdr *header,
		      struct lib_info *lib_info)
{
	struct xtlib_loader_globals *xtlib_globals =
					&lib_info->xtlib_globals;
	Elf32_Shdr *sheader = (Elf32_Shdr *) (((char *)header) +
		xtlib_host_word(header->e_shoff, xtlib_globals->byteswap));

	int sec = 0;
	int num = xtlib_host_half(header->e_shnum, xtlib_globals->byteswap);

	int align = 0;

	while (sec < num) {
		if (sheader[sec].sh_type != SHT_NULL &&
		    xtlib_host_word(sheader[sec].sh_size,
				    xtlib_globals->byteswap) > 0) {
			int sec_align =
				xtlib_host_word(sheader[sec].sh_addralign,
						xtlib_globals->byteswap);
			if (sec_align > align)
				align = sec_align;
		}
		sec++;
	}

	return align;
}

static int validate_dynamic(Elf32_Ehdr *header,
			    struct lib_info *lib_info)
{
	struct xtlib_loader_globals *xtlib_globals =
					&lib_info->xtlib_globals;

	if (xtlib_verify_magic(header, lib_info) != 0)
		return XTLIB_NOT_ELF;

	if (xtlib_host_half(header->e_type,
			    xtlib_globals->byteswap) != ET_DYN)
		return XTLIB_NOT_DYNAMIC;

	return XTLIB_NO_ERR;
}

static int validate_dynamic_splitload(Elf32_Ehdr *header,
				      struct lib_info *lib_info)
{
	struct xtlib_loader_globals *xtlib_globals =
					&lib_info->xtlib_globals;
	Elf32_Phdr *pheader;
	int err = validate_dynamic(header, lib_info);

	if (err != XTLIB_NO_ERR)
		return err;

	/* make sure it's split load pi library, expecting three headers,
	 * code, data and dynamic, for example:
	 *
	 *LOAD	off    0x00000094 vaddr 0x00000000 paddr 0x00000000 align 2**0
	 *	filesz 0x00000081 memsz 0x00000081 flags r-x
	 *LOAD  off    0x00000124 vaddr 0x00000084 paddr 0x00000084 align 2**0
	 *	filesz 0x000001ab memsz 0x000011bc flags rwx
	 *DYNAMIC off  0x00000124 vaddr 0x00000084 paddr 0x00000084 align 2**2
	 *	  filesz 0x000000a0 memsz 0x000000a0 flags rw-
	 */

	if (xtlib_host_half(header->e_phnum, xtlib_globals->byteswap) != 3)
		return XTLIB_NOT_SPLITLOAD;

	pheader = (Elf32_Phdr *)((char *)header +
		xtlib_host_word(header->e_phoff, xtlib_globals->byteswap));

	/* LOAD R-X */
	if (xtlib_host_word(pheader[0].p_type,
			    xtlib_globals->byteswap) != PT_LOAD ||
				(xtlib_host_word(pheader[0].p_flags,
						xtlib_globals->byteswap)
				& (PF_R | PF_W | PF_X)) != (PF_R | PF_X))
		return XTLIB_NOT_SPLITLOAD;

	/* LOAD RWX */
	if (xtlib_host_word(pheader[1].p_type,
			    xtlib_globals->byteswap) != PT_LOAD ||
			(xtlib_host_word(pheader[1].p_flags,
			xtlib_globals->byteswap)
			& (PF_R | PF_W | PF_X)) != (PF_R | PF_W | PF_X))
		return XTLIB_NOT_SPLITLOAD;

	/* DYNAMIC RW- */
	if (xtlib_host_word(pheader[2].p_type,
			    xtlib_globals->byteswap) != PT_DYNAMIC ||
			(xtlib_host_word(pheader[2].p_flags,
			xtlib_globals->byteswap)
			& (PF_R | PF_W | PF_X)) != (PF_R | PF_W))
		return XTLIB_NOT_SPLITLOAD;

	return XTLIB_NO_ERR;
}

static unsigned int
xtlib_split_pi_library_size(struct xtlib_packaged_library *library,
			    unsigned int *code_size,
			    unsigned int *data_size,
			    struct lib_info *lib_info)
{
	struct xtlib_loader_globals *xtlib_globals =
					&lib_info->xtlib_globals;
	Elf32_Phdr *pheader;
	Elf32_Ehdr *header = (Elf32_Ehdr *)library;
	int align;
	int err = validate_dynamic_splitload(header, lib_info);

	if (err != XTLIB_NO_ERR) {
		xtlib_globals->err = err;
		return err;
	}

	align = find_align(header, lib_info);

	pheader = (Elf32_Phdr *)((char *)library +
		xtlib_host_word(header->e_phoff, xtlib_globals->byteswap));

	*code_size = xtlib_host_word(pheader[0].p_memsz,
					xtlib_globals->byteswap) + align;
	*data_size = xtlib_host_word(pheader[1].p_memsz,
					xtlib_globals->byteswap) + align;

	return XTLIB_NO_ERR;
}

static int get_dyn_info(Elf32_Ehdr *eheader,
			xt_ptr dst_addr, xt_uint src_offs,
			xt_ptr dst_data_addr, xt_uint src_data_offs,
			struct xtlib_pil_info *info,
			struct lib_info *lib_info)
{
	unsigned int jmprel = 0;
	unsigned int pltrelsz = 0;
	struct xtlib_loader_globals *xtlib_globals =
					&lib_info->xtlib_globals;
	Elf32_Dyn *dyn_entry = find_dynamic_info(eheader, lib_info);

	if (dyn_entry == 0)
		return XTLIB_NO_DYNAMIC_SEGMENT;

	info->dst_addr = (xt_uint)xtlib_xt_word((Elf32_Word)dst_addr,
						xtlib_globals->byteswap);
	info->src_offs = xtlib_xt_word(src_offs, xtlib_globals->byteswap);
	info->dst_data_addr = (xt_uint)xtlib_xt_word(
			(Elf32_Word)dst_data_addr + src_data_offs,
			xtlib_globals->byteswap);
	info->src_data_offs = xtlib_xt_word(src_data_offs,
				xtlib_globals->byteswap);

	dst_addr -= src_offs;
	dst_data_addr = dst_data_addr + src_data_offs - src_data_offs;

	info->start_sym = xt_ptr_offs(dst_addr, eheader->e_entry, lib_info);

	info->align = xtlib_xt_word(find_align(eheader, lib_info),
					xtlib_globals->byteswap);

	info->text_addr = 0;

	while (dyn_entry->d_tag != DT_NULL) {
		switch ((Elf32_Sword) xtlib_host_word(
			(Elf32_Word)dyn_entry->d_tag,
			xtlib_globals->byteswap)) {
		case DT_RELA:
			info->rel = xt_ptr_offs(dst_data_addr,
					dyn_entry->d_un.d_ptr, lib_info);
			break;
		case DT_RELASZ:
			info->rela_count = xtlib_xt_word(
				xtlib_host_word(dyn_entry->d_un.d_val,
						xtlib_globals->byteswap) /
				sizeof(Elf32_Rela),
				xtlib_globals->byteswap);
			break;
		case DT_INIT:
			info->init = xt_ptr_offs(dst_addr,
					dyn_entry->d_un.d_ptr, lib_info);
			break;
		case DT_FINI:
			info->fini = xt_ptr_offs(dst_addr,
					dyn_entry->d_un.d_ptr, lib_info);
			break;
		case DT_HASH:
			info->hash = xt_ptr_offs(dst_data_addr,
					dyn_entry->d_un.d_ptr, lib_info);
			break;
		case DT_SYMTAB:
			info->symtab = xt_ptr_offs(dst_data_addr,
					dyn_entry->d_un.d_ptr, lib_info);
			break;
		case DT_STRTAB:
			info->strtab = xt_ptr_offs(dst_data_addr,
					dyn_entry->d_un.d_ptr, lib_info);
			break;
		case DT_JMPREL:
			jmprel = dyn_entry->d_un.d_val;
			break;
		case DT_PLTRELSZ:
			pltrelsz = dyn_entry->d_un.d_val;
			break;
		case DT_LOPROC + 2:
			info->text_addr = xt_ptr_offs(dst_addr,
					dyn_entry->d_un.d_ptr, lib_info);
			break;

		default:
			/* do nothing */
			break;
		}
		dyn_entry++;
	}

	return XTLIB_NO_ERR;
}

static xt_ptr
xtlib_load_split_pi_library_common(struct xtlib_packaged_library *library,
				   xt_ptr destination_code_address,
				   xt_ptr destination_data_address,
				   struct xtlib_pil_info *info,
				   struct lib_info *lib_info)
{
	struct xtlib_loader_globals *xtlib_globals =
					&lib_info->xtlib_globals;
	Elf32_Ehdr *header = (Elf32_Ehdr *)library;
	Elf32_Phdr *pheader;
	unsigned int align;
	int err = validate_dynamic_splitload(header, lib_info);
	xt_ptr destination_code_address_back;
	xt_ptr destination_data_address_back;

	if (err != XTLIB_NO_ERR) {
		xtlib_globals->err = err;
		return 0;
	}

	align = find_align(header, lib_info);

	destination_code_address_back = destination_code_address;
	destination_data_address_back = destination_data_address;

	destination_code_address = align_ptr(destination_code_address, align);
	destination_data_address = align_ptr(destination_data_address, align);
	lib_info->code_buf_virt += (destination_code_address -
				destination_code_address_back);
	lib_info->data_buf_virt += (destination_data_address -
				destination_data_address_back);

	pheader = (Elf32_Phdr *)((char *)library +
			xtlib_host_word(header->e_phoff,
					xtlib_globals->byteswap));

	err = get_dyn_info(header,
			   destination_code_address,
			   xtlib_host_word(pheader[0].p_paddr,
					   xtlib_globals->byteswap),
			   destination_data_address,
			   xtlib_host_word(pheader[1].p_paddr,
					   xtlib_globals->byteswap),
			   info,
			   lib_info);

	if (err != XTLIB_NO_ERR) {
		xtlib_globals->err = err;
		return 0;
	}

	/* loading code */
	xtlib_load_seg(&pheader[0],
		       (char *)library + xtlib_host_word(pheader[0].p_offset,
			xtlib_globals->byteswap),
			(xt_ptr)lib_info->code_buf_virt,
			lib_info);

	if (info->text_addr == 0)
		info->text_addr =
		(xt_ptr)xtlib_xt_word((Elf32_Word)destination_code_address,
						xtlib_globals->byteswap);

	/* loading data */
	xtlib_load_seg(&pheader[1],
		       (char *)library + xtlib_host_word(pheader[1].p_offset,
						xtlib_globals->byteswap),
		  (xt_ptr)lib_info->data_buf_virt +
		  xtlib_host_word(pheader[1].p_paddr,
				  xtlib_globals->byteswap),
		  lib_info);

	return (xt_ptr)xtlib_host_word((Elf32_Word)info->start_sym,
				       xtlib_globals->byteswap);
}

static xt_ptr
xtlib_host_load_split_pi_library(struct xtlib_packaged_library *library,
				 xt_ptr destination_code_address,
				 xt_ptr destination_data_address,
				 struct xtlib_pil_info *info,
				 struct lib_info *lib_info)
{
	return  xtlib_load_split_pi_library_common(library,
						   destination_code_address,
						   destination_data_address,
						   info,
						   lib_info);
}

static long
load_dpu_with_library(struct xf_client *client, struct xf_proxy *proxy,
		      struct lib_info *lib_info)
{
	struct fsl_dsp *dsp_priv = container_of(proxy, struct fsl_dsp, proxy);
	unsigned char *srambuf;
	struct lib_dnld_info_t dpulib;
	struct file *file;
	struct xf_buffer *buf;
	Elf32_Phdr *pheader;
	Elf32_Ehdr *header;
	loff_t pos = 0;
	unsigned int align;
	int filesize = 0;
	long ret_val = 0;

	file = filp_open(lib_info->filename, O_RDONLY, 0);
	if (IS_ERR(file))
		return PTR_ERR(file);

	vfs_llseek(file, 0, SEEK_END);
	filesize = (int)file->f_pos;

	srambuf = kmalloc(filesize, GFP_KERNEL);
	if (!srambuf)
		return -ENOMEM;

	vfs_llseek(file, 0, SEEK_SET);
	kernel_read(file, srambuf, filesize, &pos);
	filp_close(file, NULL);

	ret_val = xtlib_split_pi_library_size(
			(struct xtlib_packaged_library *)(srambuf),
			(unsigned int *)&dpulib.size_code,
			(unsigned int *)&dpulib.size_data,
			lib_info);
	if (ret_val != XTLIB_NO_ERR)
		return -EINVAL;

	lib_info->code_buf_size = dpulib.size_code;
	lib_info->data_buf_size = dpulib.size_data;

	header = (Elf32_Ehdr *)srambuf;
	pheader = (Elf32_Phdr *)((char *)srambuf +
				xtlib_host_word(header->e_phoff,
						lib_info->xtlib_globals.byteswap));

	align = find_align(header, lib_info);
	ret_val = xf_pool_alloc(client, proxy, 1, dpulib.size_code + align,
				XF_POOL_AUX, &lib_info->code_section_pool);
        if (ret_val) {
                kfree(srambuf);
                pr_err("Allocation failure for loading code section\n");
                return -ENOMEM;
        }

        ret_val = xf_pool_alloc(client, proxy, 1,
                                dpulib.size_data + pheader[1].p_paddr + align,
                                XF_POOL_AUX, &lib_info->data_section_pool);
        if (ret_val) {
                kfree(srambuf);
                pr_err("Allocation failure for loading data section\n");
                return -ENOMEM;
        }

	buf = xf_buffer_get(lib_info->code_section_pool);
	lib_info->code_buf_virt = xf_buffer_data(buf);
	lib_info->code_buf_phys = ((u64)xf_buffer_data(buf) -
					(u64)dsp_priv->scratch_buf_virt) +
					dsp_priv->scratch_buf_phys;
	lib_info->code_buf_size = dpulib.size_code + align;
	xf_buffer_put(buf);

	buf = xf_buffer_get(lib_info->data_section_pool);
	lib_info->data_buf_virt = xf_buffer_data(buf);
	lib_info->data_buf_phys = ((u64)xf_buffer_data(buf) -
					(u64)dsp_priv->scratch_buf_virt) +
					dsp_priv->scratch_buf_phys;
	lib_info->data_buf_size = dpulib.size_data + align + pheader[1].p_paddr;
	xf_buffer_put(buf);

	dpulib.pbuf_code = (unsigned long)lib_info->code_buf_phys;
	dpulib.pbuf_data = (unsigned long)lib_info->data_buf_phys;

	dpulib.ppil_inf = &lib_info->pil_info;
	xtlib_host_load_split_pi_library((struct xtlib_packaged_library *)srambuf,
					 (xt_ptr)(dpulib.pbuf_code),
					 (xt_ptr)(dpulib.pbuf_data),
					 (struct xtlib_pil_info *)dpulib.ppil_inf,
					 (void *)lib_info);
	kfree(srambuf);

	return ret_val;
}

static long
unload_dpu_with_library(struct xf_client *client, struct xf_proxy *proxy,
			struct lib_info *lib_info)
{
        xf_pool_free(client, lib_info->code_section_pool);
        xf_pool_free(client, lib_info->data_section_pool);

	return 0;
}

long xf_load_lib(struct xf_client *client,
		struct xf_handle *handle, struct lib_info *lib_info)
{
	void *b = xf_handle_aux(handle);
	struct icm_xtlib_pil_info icm_info;
	struct xf_proxy *proxy = handle->proxy;
	struct xf_message  msg;
	struct xf_message *rmsg;
	long ret_val;

	ret_val = load_dpu_with_library(client, proxy, lib_info);
	if (ret_val)
		return ret_val;

	memcpy((void *)(&icm_info.pil_info), (void *)(&lib_info->pil_info),
	       sizeof(struct xtlib_pil_info));

	icm_info.lib_type = lib_info->lib_type;

	/* ...set message parameters */
	msg.id = __XF_MSG_ID(__XF_AP_CLIENT(0, 0), __XF_PORT_SPEC2(handle->id, 0));
	msg.id = XF_MSG_AP_FROM_USER(msg.id, client->id);
        msg.opcode = XF_LOAD_LIB;
        msg.buffer = b;
        msg.length = sizeof(struct icm_xtlib_pil_info);
        msg.ret = 0;

        /* ...copy lib info */
        memcpy(b, (void *)&icm_info, xf_buffer_length(handle->aux));

        /* ...execute command synchronously */
	rmsg = xf_cmd_send_recv_complete(client, proxy, msg.id, msg.opcode,
					 msg.buffer, msg.length, &client->work,
					 &client->compr_complete);
	if (IS_ERR(rmsg))
		return PTR_ERR(rmsg);

//	xf_msg_free(proxy, rmsg);
//	xf_unlock(&proxy->lock);

	return 0;
}

long xf_unload_lib(struct xf_client *client, struct xf_handle *handle, struct lib_info *lib_info)
{
	void *b = xf_handle_aux(handle);
	struct xf_proxy *proxy = handle->proxy;
	struct xf_message msg;
	struct xf_message *rmsg;
	struct icm_xtlib_pil_info icm_info;

	memset((void *)&icm_info, 0, sizeof(struct icm_xtlib_pil_info));
	icm_info.lib_type = lib_info->lib_type;

	/* ...set message parameters */
	msg.id = __XF_MSG_ID(__XF_AP_CLIENT(0, 0),__XF_PORT_SPEC2(handle->id, 0));
	msg.id = XF_MSG_AP_FROM_USER(msg.id, client->id);
	msg.opcode = XF_UNLOAD_LIB;
	msg.buffer = b;
	msg.length = sizeof(struct icm_xtlib_pil_info);
	msg.ret = 0;

	/* ...copy lib info */
	memcpy(b, (void *)&icm_info, xf_buffer_length(handle->aux));

	/* ...execute command synchronously */
	rmsg = xf_cmd_send_recv_complete(client, proxy, msg.id, msg.opcode,
					 msg.buffer, msg.length, &client->work,
					 &client->compr_complete);
	if (IS_ERR(rmsg))
		return PTR_ERR(rmsg);

//	xf_msg_free(proxy, rmsg);
//	xf_unlock(&proxy->lock);

	return unload_dpu_with_library(client, proxy, lib_info);
}
