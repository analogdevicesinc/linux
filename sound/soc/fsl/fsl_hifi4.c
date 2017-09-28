/*
 * Freescale HIFI 4 driver
 *
 * Copyright (c) 2012-2013 by Tensilica Inc. ALL RIGHTS RESERVED.
 * Copyright 2017 NXP.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Copyright (c) 2001 William L. Pitts
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms are freely
 * permitted provided that the above copyright notice and this
 * paragraph and the following disclaimer are duplicated in all
 * such forms.
 *
 * This software is provided "AS IS" and without any express or
 * implied warranties, including, without limitation, the implied
 * warranties of merchantability and fitness for a particular
 * purpose.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/file.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/platform_data/dma-imx.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/pm_runtime.h>
#include <linux/mx8_mu.h>
#include <linux/uaccess.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include <uapi/linux/mxc_hifi4.h>
#include <soc/imx8/sc/svc/irq/api.h>
#include <soc/imx8/sc/ipc.h>
#include <soc/imx8/sc/sci.h>
#include "fsl_hifi4.h"


#ifdef CONFIG_COMPAT
struct decode_info_compat32 {
	compat_long_t in_buf_addr;
	__s32 in_buf_size;
	__s32 in_buf_off;
	compat_long_t out_buf_addr;
	__s32 out_buf_size;
	__s32 out_buf_off;
	__u32 cycles;
	__u32 input_over;
	__u32 process_id;
};

struct binary_info_compat32 {
	__s32 type;
	compat_long_t file;
	__u32 process_id;
};

static int get_binary_info_compat32(struct binary_info *kp,
				struct binary_info_compat32 __user *up) {
	void __user *up_ptr;
	compat_long_t p;

	if (!access_ok(VERIFY_READ, up, sizeof(struct binary_info_compat32)) ||
		get_user(kp->type, &up->type) ||
		get_user(p, &up->file) ||
		get_user(kp->process_id, &up->process_id)
	   ) {
		return -EFAULT;
	}

	up_ptr = compat_ptr(p);
	kp->file = (char *)up_ptr;

	return 0;
}

static int put_binary_info_compat32(struct binary_info *kp,
				struct binary_info_compat32 *up) {

	if (!access_ok(VERIFY_WRITE, up, sizeof(struct binary_info_compat32)) ||
		put_user(kp->process_id, &up->process_id)
	   ) {
		return -EFAULT;
	}

	return 0;
}

static int get_decode_info_compat32(struct decode_info *kp,
				struct decode_info_compat32 *up) {
	void __user *up_ptr1;
	void __user *up_ptr2;
	compat_long_t p1;
	compat_long_t p2;

	if (!access_ok(VERIFY_READ, up, sizeof(struct decode_info_compat32)) ||
		get_user(p1, &up->in_buf_addr) ||
		get_user(kp->in_buf_size, &up->in_buf_size) ||
		get_user(kp->in_buf_off, &up->in_buf_off) ||
		get_user(p2, &up->out_buf_addr) ||
		get_user(kp->out_buf_size, &up->out_buf_size) ||
		get_user(kp->out_buf_off, &up->out_buf_off) ||
		get_user(kp->cycles, &up->cycles) ||
		get_user(kp->input_over, &up->input_over) ||
		get_user(kp->process_id, &up->process_id)
	   ) {
		return -EFAULT;
	}

	up_ptr1 = compat_ptr(p1);
	up_ptr2 = compat_ptr(p2);

	kp->in_buf_addr = (void *)up_ptr1;
	kp->out_buf_addr = (void *)up_ptr2;

	return 0;
}

static int put_decode_info_compat32(struct decode_info *kp,
				struct decode_info_compat32 *up) {

	if (!access_ok(VERIFY_WRITE, up, sizeof(struct decode_info_compat32)) ||
		put_user(kp->in_buf_off, &up->in_buf_off) ||
		put_user(kp->out_buf_off, &up->out_buf_off) ||
		put_user(kp->cycles, &up->cycles) ||
		put_user(kp->input_over, &up->input_over)
	   ) {
		return -EFAULT;
	}

	return 0;
}
#endif

long switch_codec(struct fsl_hifi4 *hifi4_priv, int id)
{
	union icm_header_t apu_icm;
	struct hifi4_ext_msg ext_msg;
	struct icm_switch_info_t switch_info;
	int i;
	long ret = 0;

	switch_info.proc_id = id;
	switch_info.status  = 0;

	init_completion(&hifi4_priv->cmd_complete);
	hifi4_priv->is_done = 0;

	apu_icm.allbits = 0;	/* clear all bits; */

	apu_icm.ack = 0;
	apu_icm.intr = 1;
	apu_icm.msg = ICM_SWITCH_CODEC;
	apu_icm.size = 8;

	ext_msg.phys = hifi4_priv->msg_buf_phys;
	ext_msg.size = sizeof(struct icm_switch_info_t);
	memcpy(hifi4_priv->msg_buf_virt, &switch_info,
				sizeof(struct icm_switch_info_t));

	icm_intr_extended_send(hifi4_priv, apu_icm.allbits, &ext_msg);

	/* wait for response here */
	ret = icm_ack_wait(hifi4_priv, apu_icm.allbits);
	if (ret)
		return ret;

	/* check whether the dsp framework switches successfully or not */
	ret = hifi4_priv->ret_status;
	if (ret)
		return ret;

	/* Because this variables are shared for every codec, so when
	 * switching, need to recover it value for current codec.
	 */
	for (i = 0; i < MULTI_CODEC_NUM; i++) {
		if (hifi4_priv->process_info[i].process_id == id) {
			if (hifi4_priv->process_info[i].status) {
				hifi4_priv->cur_res_id = i;
				hifi4_priv->pil_info =
				  hifi4_priv->process_info[i].pil_info_info;
				hifi4_priv->objtype =
				  hifi4_priv->process_info[i].codec_id;
				hifi4_priv->codec_iobuf_info.proc_id =
				  hifi4_priv->process_info[i].proc_id;
				break;
			}
		}
	}
	/* update the current process id to the new process id */
	hifi4_priv->process_id = id;

	return ret;
}

long load_dpu_with_library(struct fsl_hifi4 *hifi4_priv)
{
	struct file *fpInfile;
	unsigned char *srambuf = NULL;
	struct lib_dnld_info_t dpulib;
	int filesize = 0;
	unsigned int id;
	long ret_val = 0;

	id = hifi4_priv->cur_res_id;

	/* Load DPU's main program to System memory */
	fpInfile = file_open_name(hifi4_priv->objfile, O_RDONLY, 0);
	if (IS_ERR(fpInfile))
		return PTR_ERR(fpInfile);

	vfs_llseek(fpInfile, 0, SEEK_END);
	filesize = (int)fpInfile->f_pos;

	srambuf = kmalloc(filesize, GFP_KERNEL);
	vfs_llseek(fpInfile, 0, SEEK_SET);

	kernel_read(fpInfile, srambuf, filesize, NULL);
	filp_close(fpInfile, NULL);

	ret_val = xtlib_split_pi_library_size(
			(struct xtlib_packaged_library *)(srambuf),
			(unsigned int *)&(dpulib.size_code),
			(unsigned int *)&(dpulib.size_data),
			hifi4_priv);
	if (ret_val != XTLIB_NO_ERR)
		return ret_val;

	hifi4_priv->size_code = dpulib.size_code;
	hifi4_priv->size_data = dpulib.size_data;

	dpulib.pbuf_code =  (unsigned long)hifi4_priv->code_buf_phys;
	dpulib.pbuf_data =
		(unsigned long)hifi4_priv->process_info[id].data_buf_phys;

	dpulib.ppil_inf = &hifi4_priv->pil_info;
	xtlib_host_load_split_pi_library(
			(struct xtlib_packaged_library *) (srambuf),
			(xt_ptr) (dpulib.pbuf_code),
			(xt_ptr) (dpulib.pbuf_data),
			(struct xtlib_pil_info *)dpulib.ppil_inf,
			(memcpy_func)&memcpy_hifi,
			(memset_func)&memset_hifi,
			(void *)hifi4_priv);

	kfree(srambuf);

	return ret_val;
}

Elf32_Half xtlib_host_half(Elf32_Half v, int byteswap)
{
	return (byteswap) ? (v >> 8) | (v << 8) : v;
}

Elf32_Word xtlib_host_word(Elf32_Word v, int byteswap)
{
	if (byteswap) {
		v = ((v & 0x00FF00FF) << 8) | ((v & 0xFF00FF00) >> 8);
		v = (v >> 16) | (v << 16);
	}
	return v;
}

int xtlib_verify_magic(Elf32_Ehdr *header,
				    struct fsl_hifi4 *hifi4_priv)
{
	struct xtlib_loader_globals *xtlib_globals = &hifi4_priv->xtlib_globals;
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

void xtlib_load_seg(Elf32_Phdr *pheader, void *src_addr, xt_ptr dst_addr,
			memcpy_func mcpy, memset_func mset,
			struct fsl_hifi4 *hifi4_priv)
{
	struct xtlib_loader_globals *xtlib_globals = &hifi4_priv->xtlib_globals;
	Elf32_Word bytes_to_copy = xtlib_host_word(pheader->p_filesz,
						xtlib_globals->byteswap);
	Elf32_Word bytes_to_zero = xtlib_host_word(pheader->p_memsz,
						xtlib_globals->byteswap)
						- bytes_to_copy;

	void *zero_addr = (void *)dst_addr + bytes_to_copy;

	if (bytes_to_copy > 0)
		mcpy((void *)(dst_addr), src_addr, bytes_to_copy);

	if (bytes_to_zero > 0)
		mset(zero_addr, 0, bytes_to_zero);
}

#define xtlib_xt_half  xtlib_host_half
#define xtlib_xt_word  xtlib_host_word

static xt_ptr align_ptr(xt_ptr ptr, xt_uint align)
{
	return (xt_ptr)(((xt_uint)ptr + align - 1) & ~(align - 1));
}

static xt_ptr xt_ptr_offs(xt_ptr base, Elf32_Word offs,
				    struct fsl_hifi4 *hifi4_priv)
{
	struct xtlib_loader_globals *xtlib_globals = &hifi4_priv->xtlib_globals;

	return (xt_ptr) xtlib_xt_word((xt_uint)base +
		xtlib_host_word(offs, xtlib_globals->byteswap),
			xtlib_globals->byteswap);
}

static Elf32_Dyn *find_dynamic_info(Elf32_Ehdr *eheader,
				    struct fsl_hifi4 *hifi4_priv)
{
	char *base_addr = (char *)eheader;
	struct xtlib_loader_globals *xtlib_globals = &hifi4_priv->xtlib_globals;
	Elf32_Phdr *pheader = (Elf32_Phdr *)(base_addr +
			xtlib_host_word(eheader->e_phoff,
					xtlib_globals->byteswap));

	int seg = 0;
	int num = xtlib_host_half(eheader->e_phnum, xtlib_globals->byteswap);

	while (seg < num) {
		if (xtlib_host_word(pheader[seg].p_type,
				xtlib_globals->byteswap) == PT_DYNAMIC) {
			return (Elf32_Dyn *) (base_addr +
				xtlib_host_word(pheader[seg].p_offset,
						xtlib_globals->byteswap));
		}
		seg++;
	}
	return 0;
}

static int find_align(Elf32_Ehdr *header,
				    struct fsl_hifi4 *hifi4_priv)
{
	struct xtlib_loader_globals *xtlib_globals = &hifi4_priv->xtlib_globals;
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
				    struct fsl_hifi4 *hifi4_priv)
{
	struct xtlib_loader_globals *xtlib_globals = &hifi4_priv->xtlib_globals;

	if (xtlib_verify_magic(header, hifi4_priv) != 0)
		return XTLIB_NOT_ELF;

	if (xtlib_host_half(header->e_type,
				xtlib_globals->byteswap) != ET_DYN)
		return XTLIB_NOT_DYNAMIC;

	return XTLIB_NO_ERR;
}

static int validate_dynamic_splitload(Elf32_Ehdr *header,
				    struct fsl_hifi4 *hifi4_priv)
{
	struct xtlib_loader_globals *xtlib_globals = &hifi4_priv->xtlib_globals;
	Elf32_Phdr *pheader;
	int err = validate_dynamic(header, hifi4_priv);

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

	pheader = (Elf32_Phdr *) ((char *)header +
		xtlib_host_word(header->e_phoff, xtlib_globals->byteswap));

	/* LOAD R-X */
	if (xtlib_host_word(pheader[0].p_type,
				xtlib_globals->byteswap) != PT_LOAD
	    || (xtlib_host_word(pheader[0].p_flags, xtlib_globals->byteswap)
				& (PF_R | PF_W | PF_X)) != (PF_R | PF_X))
		return XTLIB_NOT_SPLITLOAD;

	/* LOAD RWX */
	if (xtlib_host_word(pheader[1].p_type,
				xtlib_globals->byteswap) != PT_LOAD
		|| (xtlib_host_word(pheader[1].p_flags,
				xtlib_globals->byteswap)
			& (PF_R | PF_W | PF_X)) != (PF_R | PF_W | PF_X))
		return XTLIB_NOT_SPLITLOAD;

	/* DYNAMIC RW- */
	if (xtlib_host_word(pheader[2].p_type,
				xtlib_globals->byteswap) != PT_DYNAMIC
		|| (xtlib_host_word(pheader[2].p_flags,
					xtlib_globals->byteswap)
				& (PF_R | PF_W | PF_X)) != (PF_R | PF_W))
		return XTLIB_NOT_SPLITLOAD;

	return XTLIB_NO_ERR;
}



unsigned int xtlib_split_pi_library_size(
				struct xtlib_packaged_library *library,
				unsigned int *code_size,
				unsigned int *data_size,
				struct fsl_hifi4 *hifi4_priv)
{
	struct xtlib_loader_globals *xtlib_globals = &hifi4_priv->xtlib_globals;
	Elf32_Phdr *pheader;
	Elf32_Ehdr *header = (Elf32_Ehdr *) library;
	int align;
	int err = validate_dynamic_splitload(header, hifi4_priv);

	if (err != XTLIB_NO_ERR) {
		xtlib_globals->err = err;
		return err;
	}

	align = find_align(header, hifi4_priv);

	pheader = (Elf32_Phdr *) ((char *)library +
		xtlib_host_word(header->e_phoff, xtlib_globals->byteswap));

	*code_size = xtlib_host_word(pheader[0].p_memsz,
					xtlib_globals->byteswap) + align;
	*data_size = xtlib_host_word(pheader[1].p_memsz,
					xtlib_globals->byteswap) + align;

	return XTLIB_NO_ERR;
}



static int get_dyn_info(Elf32_Ehdr *eheader,
				xt_ptr dst_addr,
				xt_uint src_offs,
				xt_ptr dst_data_addr,
				xt_uint src_data_offs,
				struct xtlib_pil_info *info,
				struct fsl_hifi4 *hifi4_priv)
{
	unsigned int jmprel = 0;
	unsigned int pltrelsz = 0;
	struct xtlib_loader_globals *xtlib_globals = &hifi4_priv->xtlib_globals;
	Elf32_Dyn *dyn_entry = find_dynamic_info(eheader, hifi4_priv);

	if (dyn_entry == 0)
		return XTLIB_NO_DYNAMIC_SEGMENT;

	info->dst_addr = (xt_uint) xtlib_xt_word((Elf32_Word) dst_addr,
						xtlib_globals->byteswap);
	info->src_offs = xtlib_xt_word(src_offs, xtlib_globals->byteswap);
	info->dst_data_addr = (xt_uint) xtlib_xt_word(
			(Elf32_Word) dst_data_addr + src_data_offs,
			xtlib_globals->byteswap);
	info->src_data_offs = xtlib_xt_word(src_data_offs,
				xtlib_globals->byteswap);

	dst_addr -= src_offs;
	dst_data_addr = dst_data_addr + src_data_offs - src_data_offs;

	info->start_sym = xt_ptr_offs(dst_addr, eheader->e_entry, hifi4_priv);

	info->align = xtlib_xt_word(find_align(eheader, hifi4_priv),
					xtlib_globals->byteswap);

	info->text_addr = 0;

	while (dyn_entry->d_tag != DT_NULL) {
		switch ((Elf32_Sword) xtlib_host_word(
			(Elf32_Word)dyn_entry->d_tag,
			xtlib_globals->byteswap)) {
		case DT_RELA:
			info->rel = xt_ptr_offs(dst_data_addr,
					dyn_entry->d_un.d_ptr, hifi4_priv);
			break;
		case DT_RELASZ:
			info->rela_count = xtlib_xt_word(
				xtlib_host_word(dyn_entry->d_un.d_val,
				xtlib_globals->byteswap) / sizeof(Elf32_Rela),
				xtlib_globals->byteswap);
			break;
		case DT_INIT:
			info->init = xt_ptr_offs(dst_addr,
					dyn_entry->d_un.d_ptr, hifi4_priv);
			break;
		case DT_FINI:
			info->fini = xt_ptr_offs(dst_addr,
					dyn_entry->d_un.d_ptr, hifi4_priv);
			break;
		case DT_HASH:
			info->hash = xt_ptr_offs(dst_data_addr,
					dyn_entry->d_un.d_ptr, hifi4_priv);
			break;
		case DT_SYMTAB:
			info->symtab = xt_ptr_offs(dst_data_addr,
					dyn_entry->d_un.d_ptr, hifi4_priv);
			break;
		case DT_STRTAB:
			info->strtab = xt_ptr_offs(dst_data_addr,
					dyn_entry->d_un.d_ptr, hifi4_priv);
			break;
		case DT_JMPREL:
			jmprel = dyn_entry->d_un.d_val;
			break;
		case DT_PLTRELSZ:
			pltrelsz = dyn_entry->d_un.d_val;
			break;
		case DT_LOPROC + 2:
			info->text_addr = xt_ptr_offs(dst_addr,
					dyn_entry->d_un.d_ptr, hifi4_priv);
			break;

		default:
			/* do nothing */
			break;
		}
		dyn_entry++;
	}

	return XTLIB_NO_ERR;
}


static xt_ptr xtlib_load_split_pi_library_common(
				struct xtlib_packaged_library *library,
				xt_ptr destination_code_address,
				xt_ptr destination_data_address,
				struct xtlib_pil_info *lib_info,
				memcpy_func mcpy_fn,
				memset_func mset_fn,
				struct fsl_hifi4 *hifi4_priv)
{
	struct xtlib_loader_globals *xtlib_globals = &hifi4_priv->xtlib_globals;
	Elf32_Ehdr *header = (Elf32_Ehdr *) library;
	Elf32_Phdr *pheader;
	unsigned int align, id;
	int err = validate_dynamic_splitload(header, hifi4_priv);

	if (err != XTLIB_NO_ERR) {
		xtlib_globals->err = err;
		return 0;
	}
	id = hifi4_priv->cur_res_id;

	align = find_align(header, hifi4_priv);

	destination_code_address = align_ptr(destination_code_address, align);
	destination_data_address = align_ptr(destination_data_address, align);

	pheader = (Elf32_Phdr *) ((char *)library +
			xtlib_host_word(header->e_phoff,
				xtlib_globals->byteswap));

	err = get_dyn_info(header,
			destination_code_address,
			xtlib_host_word(pheader[0].p_paddr,
					xtlib_globals->byteswap),
			destination_data_address,
			xtlib_host_word(pheader[1].p_paddr,
					xtlib_globals->byteswap),
			lib_info,
			hifi4_priv);

	if (err != XTLIB_NO_ERR) {
		xtlib_globals->err = err;
		return 0;
	}

	/* loading code */
	xtlib_load_seg(&pheader[0],
		(char *)library + xtlib_host_word(pheader[0].p_offset,
			xtlib_globals->byteswap),
			(xt_ptr)hifi4_priv->code_buf_virt,
			mcpy_fn, mset_fn, hifi4_priv);

	if (lib_info->text_addr == 0)
		lib_info->text_addr =
		(xt_ptr) xtlib_xt_word((Elf32_Word) destination_code_address,
						xtlib_globals->byteswap);

	/* loading data */
	xtlib_load_seg(&pheader[1],
		  (char *)library + xtlib_host_word(pheader[1].p_offset,
						xtlib_globals->byteswap),
		  (xt_ptr)hifi4_priv->process_info[id].data_buf_virt +
		  xtlib_host_word(pheader[1].p_paddr,
						xtlib_globals->byteswap),
		  mcpy_fn, mset_fn, hifi4_priv);


	if (err != XTLIB_NO_ERR) {
		xtlib_globals->err = err;
		return 0;
	}

	return (xt_ptr) xtlib_host_word((Elf32_Word) lib_info->start_sym,
						xtlib_globals->byteswap);
}

xt_ptr xtlib_host_load_split_pi_library(struct xtlib_packaged_library *library,
				  xt_ptr destination_code_address,
				  xt_ptr destination_data_address,
				  struct xtlib_pil_info *lib_info,
				  memcpy_func mcpy_fn,
				  memset_func mset_fn,
				  struct fsl_hifi4 *hifi4_priv)
{
	return  xtlib_load_split_pi_library_common(library,
					      destination_code_address,
					      destination_data_address,
					      lib_info,
					      mcpy_fn,
					      mset_fn,
					      hifi4_priv);
}


static long fsl_hifi4_init_codec(struct fsl_hifi4 *hifi4_priv,
							void __user *user)
{
	struct device *dev = hifi4_priv->dev;
	union icm_header_t apu_icm;
	struct hifi4_ext_msg ext_msg;
	int id;
	long ret = 0;

	ret = copy_from_user(&id, user, sizeof(int));
	if (ret) {
		dev_err(dev, "failed to get para from user space\n");
		return -EFAULT;
	}

	if (hifi4_priv->process_id != id) {
		ret = switch_codec(hifi4_priv, id);
		if (ret) {
			dev_err(dev, "failed to switch codec in codec init\n");
			return ret;
		}
	}

	init_completion(&hifi4_priv->cmd_complete);
	hifi4_priv->is_done = 0;

	apu_icm.allbits = 0;	/* clear all bits;*/
	apu_icm.ack = 0;
	apu_icm.intr = 1;
	apu_icm.msg = ICM_PI_LIB_INIT;
	apu_icm.size = 8;

	ext_msg.phys = hifi4_priv->msg_buf_phys;
	ext_msg.size = sizeof(struct xtlib_pil_info);

	memcpy(hifi4_priv->msg_buf_virt, &hifi4_priv->pil_info,
						sizeof(struct xtlib_pil_info));
	icm_intr_extended_send(hifi4_priv, apu_icm.allbits, &ext_msg);

	/* wait for response here */
	ret = icm_ack_wait(hifi4_priv, apu_icm.allbits);
	if (ret)
		return ret;

	return 0;
}

static long fsl_hifi4_decode_frame(struct fsl_hifi4 *hifi4_priv,
						void __user *user)
{
	struct device *dev = hifi4_priv->dev;
	union icm_header_t apu_icm;
	struct hifi4_ext_msg ext_msg;
	struct decode_info decode_info;
	struct icm_cdc_iobuf_t *codec_iobuf_info =
				&hifi4_priv->codec_iobuf_info;
	long ret;

	ret = copy_from_user(&decode_info, user, sizeof(decode_info));
	if (ret) {
		dev_err(dev, "failed to get para from user space\n");
		return -EFAULT;
	}

	if (hifi4_priv->process_id != decode_info.process_id) {
		ret = switch_codec(hifi4_priv, decode_info.process_id);
		if (ret) {
			dev_err(dev, "failed to switch codec in codec decode frame\n");
			return ret;
		}
	}

	if (decode_info.in_buf_size > INPUT_BUF_SIZE ||
		decode_info.out_buf_size != OUTPUT_BUF_SIZE) {
		dev_err(dev, "param error\n");
		return -EINVAL;
	}

	if (decode_info.in_buf_off == 0) {
		ret = copy_from_user(hifi4_priv->in_buf_virt,
					(void __user *)decode_info.in_buf_addr,
					decode_info.in_buf_size);
		if (ret) {
			dev_err(dev, "failed to copy from user\n");
			return -EFAULT;
		}
		codec_iobuf_info->inp_cur_offset   = 0;
	}

	codec_iobuf_info->inp_addr_sysram  = hifi4_priv->in_buf_phys;
	codec_iobuf_info->inp_buf_size_max = decode_info.in_buf_size;
	codec_iobuf_info->inp_cur_offset = decode_info.in_buf_off;

	codec_iobuf_info->out_addr_sysram  = hifi4_priv->out_buf_phys;
	codec_iobuf_info->out_buf_size_max = hifi4_priv->out_buf_size;
	codec_iobuf_info->out_cur_offset   = 0;

	codec_iobuf_info->input_over   = decode_info.input_over;

	init_completion(&hifi4_priv->cmd_complete);
	hifi4_priv->is_done = 0;

	apu_icm.allbits = 0;	/* clear all bits; */
	apu_icm.ack  = 0;
	apu_icm.intr = 1;
	apu_icm.msg  = ICM_EMPTY_THIS_BUFFER;
	apu_icm.size = 8;
	ext_msg.phys = hifi4_priv->msg_buf_phys;
	ext_msg.size = sizeof(struct icm_cdc_iobuf_t);

	memcpy(hifi4_priv->msg_buf_virt, codec_iobuf_info,
						sizeof(struct icm_cdc_iobuf_t));
	icm_intr_extended_send(hifi4_priv, apu_icm.allbits, &ext_msg);

	/* wait for response here */
	ret = icm_ack_wait(hifi4_priv, apu_icm.allbits);
	if (ret)
		return ret;

	ret = copy_to_user((void __user *)decode_info.out_buf_addr,
				hifi4_priv->out_buf_virt,
				codec_iobuf_info->out_cur_offset);
	if (ret) {
		dev_err(dev, "failed to copy to user\n");
		return -EFAULT;
	}

	decode_info.in_buf_off = codec_iobuf_info->inp_cur_offset;
	decode_info.out_buf_off = codec_iobuf_info->out_cur_offset;
	decode_info.cycles = codec_iobuf_info->cycles;

	ret = copy_to_user(user, &decode_info, sizeof(decode_info));
	if (ret) {
		dev_err(dev, "failed to send para to user space\n");
		return -EFAULT;
	}

	ret = hifi4_priv->ret_status;
	return ret;
}

#ifdef CONFIG_COMPAT
static long fsl_hifi4_decode_frame_compat32(struct fsl_hifi4 *hifi4_priv,
						void __user *user)
{
	struct device *dev = hifi4_priv->dev;
	union icm_header_t apu_icm;
	struct hifi4_ext_msg ext_msg;
	struct decode_info decode_info;
	struct icm_cdc_iobuf_t *codec_iobuf_info =
				&hifi4_priv->codec_iobuf_info;
	long ret;

	ret = get_decode_info_compat32(&decode_info, user);
	if (ret) {
		dev_err(dev, "failed to get para from user space in compat32 mode\n");
		return ret;
	}

	if (hifi4_priv->process_id != decode_info.process_id) {
		ret = switch_codec(hifi4_priv, decode_info.process_id);
		if (ret) {
			dev_err(dev, "failed to switch codec in codec decode frame in compat32 mode\n");
			return ret;
		}
	}

	if (decode_info.in_buf_size > INPUT_BUF_SIZE ||
		decode_info.out_buf_size != OUTPUT_BUF_SIZE) {
		dev_err(dev, "param error\n");
		return -EINVAL;
	}

	if (decode_info.in_buf_off == 0) {
		ret = copy_from_user(hifi4_priv->in_buf_virt,
					(void __user *)decode_info.in_buf_addr,
					decode_info.in_buf_size);
		if (ret) {
			dev_err(dev, "failed to copy from user\n");
			return -EFAULT;
		}
		codec_iobuf_info->inp_cur_offset   = 0;
	}

	codec_iobuf_info->inp_addr_sysram  = hifi4_priv->in_buf_phys;
	codec_iobuf_info->inp_buf_size_max = decode_info.in_buf_size;
	codec_iobuf_info->inp_cur_offset = decode_info.in_buf_off;

	codec_iobuf_info->out_addr_sysram  = hifi4_priv->out_buf_phys;
	codec_iobuf_info->out_buf_size_max = hifi4_priv->out_buf_size;
	codec_iobuf_info->out_cur_offset   = 0;

	codec_iobuf_info->input_over   = decode_info.input_over;

	init_completion(&hifi4_priv->cmd_complete);
	hifi4_priv->is_done = 0;

	apu_icm.allbits = 0;	/* clear all bits; */
	apu_icm.ack  = 0;
	apu_icm.intr = 1;
	apu_icm.msg  = ICM_EMPTY_THIS_BUFFER;
	apu_icm.size = 8;
	ext_msg.phys = hifi4_priv->msg_buf_phys;
	ext_msg.size = sizeof(struct icm_cdc_iobuf_t);

	memcpy(hifi4_priv->msg_buf_virt, codec_iobuf_info,
						sizeof(struct icm_cdc_iobuf_t));
	icm_intr_extended_send(hifi4_priv, apu_icm.allbits, &ext_msg);

	/* wait for response here */
	ret = icm_ack_wait(hifi4_priv, apu_icm.allbits);
	if (ret)
		return ret;

	ret = copy_to_user((void __user *)decode_info.out_buf_addr,
				hifi4_priv->out_buf_virt,
				codec_iobuf_info->out_cur_offset);
	if (ret) {
		dev_err(dev, "failed to copy to user\n");
		return -EFAULT;
	}

	decode_info.in_buf_off = codec_iobuf_info->inp_cur_offset;
	decode_info.out_buf_off = codec_iobuf_info->out_cur_offset;
	decode_info.cycles = codec_iobuf_info->cycles;

	ret = put_decode_info_compat32(&decode_info, user);
	if (ret) {
		dev_err(dev, "failed to send para to user space in compat32 mode\n");
		return ret;
	}

	ret = hifi4_priv->ret_status;
	return ret;
}
#endif

static long fsl_hifi4_get_pcm_prop(struct fsl_hifi4 *hifi4_priv,
						void __user *user)
{
	struct device *dev = hifi4_priv->dev;
	union icm_header_t apu_icm;
	struct icm_pcm_prop_t *pcm_prop_info = &hifi4_priv->pcm_prop_info;
	struct prop_info prop_info;
	long ret = 0;

	ret = copy_from_user(&prop_info, user, sizeof(prop_info));
	if (ret) {
		dev_err(dev, "failed to get para from user space\n");
		return -EFAULT;
	}

	if (hifi4_priv->process_id != prop_info.process_id) {
		ret = switch_codec(hifi4_priv, prop_info.process_id);
		if (ret) {
			dev_err(dev, "failed to switch codec in codec get param\n");
			return ret;
		}
	}

	init_completion(&hifi4_priv->cmd_complete);
	hifi4_priv->is_done = 0;

	apu_icm.allbits = 0;	/* clear all bits;*/
	apu_icm.ack = 0;
	apu_icm.intr = 1;
	apu_icm.msg = ICM_GET_PCM_PROP;
	apu_icm.size = 0;

	icm_intr_send(hifi4_priv, apu_icm.allbits);

	/* wait for response here */
	ret = icm_ack_wait(hifi4_priv, apu_icm.allbits);
	if (ret)
		return ret;

	prop_info.samplerate = pcm_prop_info->sfreq;
	prop_info.channels = pcm_prop_info->channels;
	prop_info.bits = pcm_prop_info->bits;
	prop_info.consumed_bytes = pcm_prop_info->consumed_bytes;

	ret = copy_to_user(user, &prop_info, sizeof(prop_info));
	if (ret) {
		dev_err(dev, "failed to send para to user space\n");
		return -EFAULT;
	}

	ret = hifi4_priv->ret_status;
	return ret;
}

static int fsl_hifi4_set_config(struct fsl_hifi4 *hifi4_priv, void __user *user)
{
	struct device *dev = hifi4_priv->dev;
	union icm_header_t apu_icm;
	struct hifi4_ext_msg ext_msg;
	struct prop_config prop_config;
	int ret;

	ret = copy_from_user(&prop_config, user, sizeof(prop_config));
	if (ret) {
		dev_err(dev, "failed to get para from user space: %d\n", ret);
		return -EFAULT;
	}

	if (hifi4_priv->process_id != prop_config.process_id) {
		ret = switch_codec(hifi4_priv, prop_config.process_id);
		if (ret) {
			dev_err(dev, "failed to switch codec in codec set param\n");
			return ret;
		}
	}

	init_completion(&hifi4_priv->cmd_complete);
	hifi4_priv->is_done = 0;

	apu_icm.allbits = 0;
	apu_icm.ack = 0;
	apu_icm.intr = 1;
	apu_icm.msg = ICM_SET_PARA_CONFIG;
	apu_icm.size = 8;

	ext_msg.phys = hifi4_priv->msg_buf_phys;
	ext_msg.size = sizeof(prop_config);

	memcpy(hifi4_priv->msg_buf_virt, &prop_config, sizeof(prop_config));
	icm_intr_extended_send(hifi4_priv, apu_icm.allbits, &ext_msg);

	icm_ack_wait(hifi4_priv, apu_icm.allbits);
	if (ret)
		return ret;

	ret = hifi4_priv->ret_status;

	return ret;
}

static long fsl_hifi4_load_codec(struct fsl_hifi4 *hifi4_priv,
						void __user *user)
{
	struct device *dev = hifi4_priv->dev;
	struct filename *fpInfile;
	union icm_header_t apu_icm;
	struct binary_info binary_info;
	struct icm_pilib_size_t lib_alloc_mem;
	struct hifi4_ext_msg ext_msg;
	long ret = 0;
	long i;

	ret = copy_from_user(&binary_info, user, sizeof(binary_info));
	if (ret) {
		dev_err(dev, "failed to get para from user space\n");
		return -EFAULT;
	}

	fpInfile = getname(binary_info.file);
	if (IS_ERR(fpInfile)) {
		dev_err(dev, "failed to getname(), err = %ld\n",
					PTR_ERR(fpInfile));
		return PTR_ERR(fpInfile);
	}

	/* check whether the dsp driver has available resource or not */
	for (i = 0; i < MULTI_CODEC_NUM; i++) {
		if (!(hifi4_priv->process_info[i].status)) {
			hifi4_priv->process_info[i].status = 1;
			hifi4_priv->available_resource--;
			break;
		}
	}
	if (i >= MULTI_CODEC_NUM) {
		dev_err(dev, "out of range of multi codec max number\n");
		return -EINVAL;
	}

	/* If dsp driver has available resource, produce a new process
	 * for the new codec.
	 */
	hifi4_priv->process_id_count++;

	ret = switch_codec(hifi4_priv, hifi4_priv->process_id_count);
	if (ret) {
		dev_err(dev, "failed to switch codec in codec load\n");
		return ret;
	}

	hifi4_priv->objfile = fpInfile;
	hifi4_priv->objtype = binary_info.type;

	hifi4_priv->cur_res_id = i;
	ret = load_dpu_with_library(hifi4_priv);
	if (ret) {
		dev_err(dev, "failed to load code binary, err = %ld\n", ret);
	}

	init_completion(&hifi4_priv->cmd_complete);
	hifi4_priv->is_done = 0;

	apu_icm.allbits = 0;	/* clear all bits; */
	apu_icm.ack = 0;
	apu_icm.intr = 1;
	apu_icm.msg = ICM_PI_LIB_MEM_ALLOC;
	apu_icm.size = 8;

	ext_msg.phys = hifi4_priv->msg_buf_phys;
	ext_msg.size = sizeof(struct icm_pilib_size_t);

	lib_alloc_mem.codec_type = hifi4_priv->objtype;
	lib_alloc_mem.text_size = hifi4_priv->size_code;
	lib_alloc_mem.data_size = hifi4_priv->size_data;

	memcpy(hifi4_priv->msg_buf_virt, &lib_alloc_mem,
					sizeof(struct icm_pilib_size_t));
	icm_intr_extended_send(hifi4_priv, apu_icm.allbits, &ext_msg);

	/* wait for response here */
	ret = icm_ack_wait(hifi4_priv, apu_icm.allbits);
	if (ret)
		return ret;

	/* save current codec information */
	hifi4_priv->process_info[i].process_id = hifi4_priv->process_id;
	hifi4_priv->process_info[i].codec_id = hifi4_priv->objtype;
	hifi4_priv->process_info[i].pil_info_info = hifi4_priv->pil_info;

	/* return process id of this codec to user space */
	binary_info.process_id = hifi4_priv->process_id;
	ret = copy_to_user(user, &binary_info, sizeof(struct binary_info));
	if (ret) {
		dev_err(dev, "failed to send para to user space\n");
		return -EFAULT;
	}

	hifi4_priv->ret_status = 0;

	dev_dbg(dev, "code binary is loaded\n");

	return ret;
}

#ifdef CONFIG_COMPAT
static long fsl_hifi4_load_codec_compat32(struct fsl_hifi4 *hifi4_priv,
						void __user *user)
{
	struct device *dev = hifi4_priv->dev;
	struct filename *fpInfile;
	union icm_header_t apu_icm;
	struct binary_info binary_info;
	struct icm_pilib_size_t lib_alloc_mem;
	struct hifi4_ext_msg ext_msg;
	long ret = 0;
	long i;

	ret = get_binary_info_compat32(&binary_info, user);
	if (ret) {
		dev_err(dev, "failed to get para from user space in compat32 mode\n");
		return ret;
	}

	fpInfile = getname(binary_info.file);
	if (IS_ERR(fpInfile)) {
		dev_err(dev, "failed to getname(), err = %ld\n",
					PTR_ERR(fpInfile));
		return PTR_ERR(fpInfile);
	}

	/* check whether the dsp driver has available resource or not */
	for (i = 0; i < MULTI_CODEC_NUM; i++) {
		if (!(hifi4_priv->process_info[i].status)) {
			hifi4_priv->process_info[i].status = 1;
			hifi4_priv->available_resource--;
			break;
		}
	}
	if (i >= MULTI_CODEC_NUM) {
		dev_err(dev, "out of range of multi codec max number\n");
		return -EINVAL;
	}

	/* If dsp driver has available resource, produce a new process
	 * for the new codec.
	 */
	hifi4_priv->process_id_count++;

	ret = switch_codec(hifi4_priv, hifi4_priv->process_id_count);
	if (ret) {
		dev_err(dev, "failed to switch codec in codec load\n");
		return ret;
	}

	hifi4_priv->objfile = fpInfile;
	hifi4_priv->objtype = binary_info.type;

	hifi4_priv->cur_res_id = i;
	ret = load_dpu_with_library(hifi4_priv);
	if (ret) {
		dev_err(dev, "failed to load code binary, err = %ld\n", ret);
		return ret;
	}

	init_completion(&hifi4_priv->cmd_complete);
	hifi4_priv->is_done = 0;

	apu_icm.allbits = 0;	/* clear all bits; */
	apu_icm.ack = 0;
	apu_icm.intr = 1;
	apu_icm.msg = ICM_PI_LIB_MEM_ALLOC;
	apu_icm.size = 8;

	ext_msg.phys = hifi4_priv->msg_buf_phys;
	ext_msg.size = sizeof(struct icm_pilib_size_t);

	lib_alloc_mem.codec_type = hifi4_priv->objtype;
	lib_alloc_mem.text_size = hifi4_priv->size_code;
	lib_alloc_mem.data_size = hifi4_priv->size_data;

	memcpy(hifi4_priv->msg_buf_virt, &lib_alloc_mem,
					sizeof(struct icm_pilib_size_t));
	icm_intr_extended_send(hifi4_priv, apu_icm.allbits, &ext_msg);

	/* wait for response here */
	ret = icm_ack_wait(hifi4_priv, apu_icm.allbits);
	if (ret)
		return ret;

	/* save current codec information */
	hifi4_priv->process_info[i].process_id = hifi4_priv->process_id;
	hifi4_priv->process_info[i].codec_id = hifi4_priv->objtype;
	hifi4_priv->process_info[i].pil_info_info = hifi4_priv->pil_info;

	/* return process id of this codec to user space */
	binary_info.process_id = hifi4_priv->process_id;
	ret = put_binary_info_compat32(&binary_info, user);
	if (ret) {
		dev_err(dev, "failed to send para to user space\n");
		return ret;
	}

	hifi4_priv->ret_status = 0;

	dev_dbg(dev, "code binary is loaded\n");

	return ret;
}
#endif

static long fsl_hifi4_codec_open(struct fsl_hifi4 *hifi4_priv,
							void __user *user)
{
	struct device *dev = hifi4_priv->dev;
	union icm_header_t apu_icm;
	struct icm_cdc_uinp_t cdc_user_inp;
	struct hifi4_ext_msg ext_msg;
	int id;
	long ret = 0;

	ret = copy_from_user(&id, user, sizeof(int));
	if (ret) {
		dev_err(dev, "failed to get para from user space\n");
		return -EFAULT;
	}

	if (hifi4_priv->process_id != id) {
		ret = switch_codec(hifi4_priv, id);
		if (ret) {
			dev_err(dev, "failed to switch codec in codec open\n");
			return ret;
		}
	}

	cdc_user_inp.proc_id   = 0;
	cdc_user_inp.codec_id  = hifi4_priv->objtype;

	init_completion(&hifi4_priv->cmd_complete);
	hifi4_priv->is_done = 0;

	apu_icm.allbits = 0;	/* clear all bits; */

	apu_icm.ack = 0;
	apu_icm.intr = 1;
	apu_icm.msg = ICM_OPEN;
	apu_icm.size = 8;

	ext_msg.phys = hifi4_priv->msg_buf_phys;
	ext_msg.size = sizeof(struct icm_cdc_uinp_t);
	memcpy(hifi4_priv->msg_buf_virt, &cdc_user_inp,
					sizeof(struct icm_cdc_uinp_t));

	icm_intr_extended_send(hifi4_priv, apu_icm.allbits, &ext_msg);

	/* wait for response here */
	ret = icm_ack_wait(hifi4_priv, apu_icm.allbits);
	if (ret)
		return ret;

	/* save current codec information */
	hifi4_priv->process_info[hifi4_priv->cur_res_id].proc_id =
					hifi4_priv->codec_iobuf_info.proc_id;

	ret = hifi4_priv->ret_status;
	return ret;
}

static int fsl_hifi4_codec_close(struct fsl_hifi4 *hifi4_priv,
							void __user *user)
{
	struct device *dev = hifi4_priv->dev;
	union icm_header_t apu_icm;
	int id;
	long ret = 0;

	ret = copy_from_user(&id, user, sizeof(int));
	if (ret) {
		dev_err(dev, "failed to get para from user space\n");
		return -EFAULT;
	}

	if (hifi4_priv->process_id != id) {
		ret = switch_codec(hifi4_priv, id);
		if (ret) {
			dev_err(dev, "failed to switch codec in codec close\n");
			return ret;
		}
	}

	init_completion(&hifi4_priv->cmd_complete);
	hifi4_priv->is_done = 0;

	apu_icm.allbits = 0;	/* clear all bits;*/
	apu_icm.ack = 0;
	apu_icm.intr = 1;
	apu_icm.msg = ICM_CLOSE;
	apu_icm.size = 0;
	icm_intr_send(hifi4_priv, apu_icm.allbits);

	/* wait for response here */
	ret = icm_ack_wait(hifi4_priv, apu_icm.allbits);
	if (ret)
		return ret;

	/* Making status to 0 means releasing the resource that
	 * current codec occupies.
	 */
	hifi4_priv->process_info[hifi4_priv->cur_res_id].status = 0;
	hifi4_priv->available_resource++;

	/* If no codec occupies the resource, zero the process id count */
	if (hifi4_priv->available_resource >= MULTI_CODEC_NUM)
		hifi4_priv->process_id_count = 0;

	return ret;
}

static int fsl_hifi4_codec_reset(struct fsl_hifi4 *hifi4_priv,
							void __user *user)
{
	struct device *dev = hifi4_priv->dev;
	union icm_header_t apu_icm;
	struct icm_cdc_iobuf_t *codec_iobuf_info =
				&hifi4_priv->codec_iobuf_info;
	int id;
	long err = 0;
	unsigned long ret = 0;

	ret = copy_from_user(&id, user, sizeof(int));
	if (ret) {
		dev_err(dev, "failed to get para from user space\n");
		return -EFAULT;
	}

	if (hifi4_priv->process_id != id) {
		err = switch_codec(hifi4_priv, id);
		if (err) {
			dev_err(dev, "failed to switch codec in codec reset\n");
			return err;
		}
	}

	init_completion(&hifi4_priv->cmd_complete);
	hifi4_priv->is_done = 0;

	apu_icm.allbits = 0;	/* clear all bits;*/
	apu_icm.ack = 0;
	apu_icm.intr = 1;
	apu_icm.msg = ICM_RESET;
	apu_icm.size = 0;
	icm_intr_send(hifi4_priv, apu_icm.allbits);

	/* wait for response here */
	err = icm_ack_wait(hifi4_priv, apu_icm.allbits);
	if (err)
		return err;

	/* reset codec_iobuf_info */
	codec_iobuf_info->inp_buf_size_max = 0;
	codec_iobuf_info->inp_cur_offset = 0;

	codec_iobuf_info->out_buf_size_max = 0;
	codec_iobuf_info->out_cur_offset   = 0;

	err = hifi4_priv->ret_status;

	return err;
}

static struct miscdevice hifi4_miscdev = {
	.name	= "mxc_hifi4",
	.minor	= MISC_DYNAMIC_MINOR,
};

static long fsl_hifi4_ioctl(struct file *file, unsigned int cmd,
						unsigned long arg)
{
	struct fsl_hifi4_engine *hifi4_engine;
	struct fsl_hifi4 *hifi4_priv;
	void __user *user;
	long ret = 0;

	hifi4_engine = file->private_data;
	hifi4_priv = hifi4_engine->hifi4_priv;
	user = (void __user *)arg;

	mutex_lock(&hifi4_priv->hifi4_mutex);

	switch (cmd) {
	case HIFI4_LOAD_CODEC:
		ret = fsl_hifi4_load_codec(hifi4_priv, user);
		break;
	case HIFI4_INIT_CODEC:
		ret = fsl_hifi4_init_codec(hifi4_priv, user);
		break;
	case HIFI4_CODEC_OPEN:
		ret = fsl_hifi4_codec_open(hifi4_priv, user);
		break;
	case HIFI4_DECODE_ONE_FRAME:
		ret = fsl_hifi4_decode_frame(hifi4_priv, user);
		break;
	case HIFI4_CODEC_CLOSE:
		ret = fsl_hifi4_codec_close(hifi4_priv, user);
		break;
	case HIFI4_UNLOAD_CODEC:
		break;
	case HIFI4_GET_PCM_PROP:
		ret = fsl_hifi4_get_pcm_prop(hifi4_priv, user);
		break;
	case HIFI4_SET_CONFIG:
		ret = fsl_hifi4_set_config(hifi4_priv, user);
		break;
	case HIFI4_RESET_CODEC:
		ret = fsl_hifi4_codec_reset(hifi4_priv, user);
		break;
	default:
		break;
	}

	mutex_unlock(&hifi4_priv->hifi4_mutex);

	return ret;
}

#ifdef CONFIG_COMPAT
static long fsl_hifi4_compat_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	struct fsl_hifi4_engine *hifi4_engine;
	struct fsl_hifi4 *hifi4_priv;
	void __user *user;
	long ret = 0;

	hifi4_engine = file->private_data;
	hifi4_priv = hifi4_engine->hifi4_priv;
	user = compat_ptr(arg);

	mutex_lock(&hifi4_priv->hifi4_mutex);

	switch (cmd) {
	case HIFI4_LOAD_CODEC:
		ret = fsl_hifi4_load_codec_compat32(hifi4_priv, user);
		break;
	case HIFI4_INIT_CODEC:
		ret = fsl_hifi4_init_codec(hifi4_priv, user);
		break;
	case HIFI4_CODEC_OPEN:
		ret = fsl_hifi4_codec_open(hifi4_priv, user);
		break;
	case HIFI4_DECODE_ONE_FRAME:
		ret = fsl_hifi4_decode_frame_compat32(hifi4_priv, user);
		break;
	case HIFI4_CODEC_CLOSE:
		ret = fsl_hifi4_codec_close(hifi4_priv, user);
		break;
	case HIFI4_UNLOAD_CODEC:
		break;
	case HIFI4_GET_PCM_PROP:
		ret = fsl_hifi4_get_pcm_prop(hifi4_priv, user);
		break;
	case HIFI4_SET_CONFIG:
		ret = fsl_hifi4_set_config(hifi4_priv, user);
		break;
	case HIFI4_RESET_CODEC:
		ret = fsl_hifi4_codec_reset(hifi4_priv, user);
		break;
	default:
		break;
	}

	mutex_unlock(&hifi4_priv->hifi4_mutex);

	return ret;
}
#endif

void resource_release(struct fsl_hifi4 *hifi4_priv)
{
	int i;

	hifi4_priv->available_resource = MULTI_CODEC_NUM;
	for (i = 0; i < MULTI_CODEC_NUM; i++) {
		hifi4_priv->process_info[i].data_buf_virt =
				hifi4_priv->data_buf_virt +
				i * hifi4_priv->data_buf_size / MULTI_CODEC_NUM;
		hifi4_priv->process_info[i].data_buf_phys =
				hifi4_priv->data_buf_phys +
				i * hifi4_priv->data_buf_size / MULTI_CODEC_NUM;

		hifi4_priv->process_info[i].status = 0;
	}
	send_dpu_ext_msg_addr(hifi4_priv);
}

static int fsl_hifi4_open(struct inode *inode, struct file *file)
{
	struct fsl_hifi4 *hifi4_priv;
	struct device *dev;
	struct fsl_hifi4_engine *hifi4_engine;
	int ret = 0;

	hifi4_priv = dev_get_drvdata(hifi4_miscdev.parent);
	dev = hifi4_priv->dev;

	mutex_lock(&hifi4_priv->hifi4_mutex);

	hifi4_engine = devm_kzalloc(dev,
				sizeof(struct fsl_hifi4_engine), GFP_KERNEL);
	if (!hifi4_engine)
		return -ENOMEM;

	hifi4_engine->hifi4_priv = hifi4_priv;

	file->private_data = hifi4_engine;

	if (!hifi4_priv->is_ready) {
		init_completion(&hifi4_priv->cmd_complete);

		ret = request_firmware_nowait(THIS_MODULE,
				FW_ACTION_HOTPLUG, hifi4_priv->fw_name,
				dev,
				GFP_KERNEL, hifi4_priv, hifi4_load_firmware);

		if (ret) {
			dev_err(dev, "failed to load firmware\n");
			return ret;
		}

		ret = icm_ack_wait(hifi4_priv, 0);
		if (ret)
			return ret;
		dev_info(dev, "hifi driver registered\n");
	}

	/* increase reference counter when opening device */
	atomic_long_inc(&hifi4_priv->refcnt);

	mutex_unlock(&hifi4_priv->hifi4_mutex);

	return ret;
}

static int fsl_hifi4_close(struct inode *inode, struct file *file)
{
	struct fsl_hifi4 *hifi4_priv;
	struct device *dev;
	struct fsl_hifi4_engine *hifi4_engine;

	hifi4_priv = dev_get_drvdata(hifi4_miscdev.parent);
	mutex_lock(&hifi4_priv->hifi4_mutex);

	dev = hifi4_priv->dev;
	hifi4_engine = file->private_data;
	devm_kfree(dev, hifi4_engine);

	/* decrease reference counter when closing device */
	atomic_long_dec(&hifi4_priv->refcnt);
	/* If device is free, reinitialize the resource of
	 * hifi4 driver and framework
	 */
	if (atomic_long_read(&hifi4_priv->refcnt) <= 0)
		resource_release(hifi4_priv);

	mutex_unlock(&hifi4_priv->hifi4_mutex);

	return 0;
}

void *memset_hifi(void *dest, int c, size_t count)
{
	unsigned int *dl = (unsigned int *)dest;
	size_t n = round_up(count, 4) / 4;

	/* while all data is aligned (common case), copy a word at a time */
	if ((((ulong)dest) & (sizeof(*dl) - 1)) != 0)
		pr_info("dest %p not 4 bytes aligned\n", dest);

	while (n--) {
		writel_relaxed(0,  dl);
		dl++;
	}

	return dest;
}

void *memcpy_hifi(void *dest, const void *src, size_t count)
{
	unsigned int *dl = (unsigned int *)dest, *sl = (unsigned int *)src;
	size_t n = round_up(count, 4) / 4;

	if (src == dest)
		return dest;

	/* while all data is aligned (common case), copy a word at a time */
	if ((((ulong)dest | (ulong)src) & (sizeof(*dl) - 1)) != 0)
		pr_info("dest %p src %p not 4 bytes aligned\n", dest, src);

	while (n--) {
		writel_relaxed(*sl,  dl);
		dl++;
		sl++;
	}

	return dest;
}

u32 icm_intr_send(struct fsl_hifi4 *hifi4_priv, u32 msg)
{
	MU_SendMessage(hifi4_priv->mu_base_virtaddr, 0, msg);
	return 0;
}

u32 icm_intr_extended_send(struct fsl_hifi4 *hifi4_priv, u32 msg,
						struct hifi4_ext_msg *ext_msg)
{
	struct device *dev = hifi4_priv->dev;
	union icm_header_t lmsg;

	lmsg.allbits = msg;
	if (lmsg.size != 8)
		dev_err(dev, "too much ext msg\n");

	MU_SendMessage(hifi4_priv->mu_base_virtaddr, 1, ext_msg->phys);
	MU_SendMessage(hifi4_priv->mu_base_virtaddr, 2, ext_msg->size);
	MU_SendMessage(hifi4_priv->mu_base_virtaddr, 0, msg);

	return 0;
}

long icm_ack_wait(struct fsl_hifi4 *hifi4_priv, u32 msg)
{
	struct device *dev = hifi4_priv->dev;
	union icm_header_t ref_msg;
	int err;

	ref_msg.allbits = msg;
	/* wait response from mu */
	err = wait_for_completion_timeout(&hifi4_priv->cmd_complete,
				msecs_to_jiffies(1000));
	if (!err) {
		dev_err(dev, "icm ack timeout!\n");
		return -ETIMEDOUT;
	}

	dev_dbg(dev, "Ack recd for message 0x%08x\n", ref_msg.allbits);

	return 0;
}

int process_act_complete(struct fsl_hifi4 *hifi4_priv, u32 msg)
{
	union icm_header_t recd_msg;
	u32 ext_msg_addr;
	u32 ext_msg_size = 0;
	u32 *pmsg_apu = (u32 *)	hifi4_priv->msg_buf_virt +  2048/4;
	struct icm_cdc_iobuf_t *codec_iobuf_info =
					&hifi4_priv->codec_iobuf_info;
	struct icm_pcm_prop_t *pcm_prop_info = &hifi4_priv->pcm_prop_info;
	int ret_val = 0;

	recd_msg.allbits = msg;

	if (recd_msg.size == 8) {
		MU_ReceiveMsg(hifi4_priv->mu_base_virtaddr, 1, &ext_msg_addr);
		MU_ReceiveMsg(hifi4_priv->mu_base_virtaddr, 2, &ext_msg_size);
	}

	switch (recd_msg.sub_msg) {
	case ICM_PI_LIB_MEM_ALLOC:
		{
			hifi4_priv->is_done = 1;
			complete(&hifi4_priv->cmd_complete);
		}
		break;

	case ICM_OPEN:
		{
			struct icm_open_resp_info_t *pext_msg =
			    (struct icm_open_resp_info_t *)pmsg_apu;
			codec_iobuf_info->proc_id = pext_msg->proc_id;
			hifi4_priv->dpu_tstamp =
			  (struct timestamp_info_t *)((long)pext_msg->dtstamp);
			hifi4_priv->ret_status = pext_msg->ret;
			hifi4_priv->is_done = 1;
			complete(&hifi4_priv->cmd_complete);
		}
		break;
	case ICM_EMPTY_THIS_BUFFER:
		{
			struct icm_cdc_iobuf_t *ext_msg =
					(struct icm_cdc_iobuf_t *)pmsg_apu;
			if (codec_iobuf_info->proc_id == ext_msg->proc_id) {
				codec_iobuf_info->inp_cur_offset =
						ext_msg->inp_cur_offset;
				codec_iobuf_info->out_cur_offset =
						ext_msg->out_cur_offset;
				codec_iobuf_info->cycles =
						ext_msg->cycles;
			}
			hifi4_priv->ret_status = ext_msg->ret;
			hifi4_priv->is_done = 1;
			complete(&hifi4_priv->cmd_complete);
		}
		break;

	case ICM_GET_PCM_PROP:
		{
			struct icm_pcm_prop_t *ext_msg =
					(struct icm_pcm_prop_t *)pmsg_apu;
			pcm_prop_info->proc_id  = ext_msg->proc_id;
			pcm_prop_info->pcmbytes = ext_msg->pcmbytes;
			pcm_prop_info->sfreq    = ext_msg->sfreq;
			pcm_prop_info->channels = ext_msg->channels;
			pcm_prop_info->bits     = ext_msg->bits;
			pcm_prop_info->consumed_bytes =
					ext_msg->consumed_bytes;
			hifi4_priv->ret_status = ext_msg->ret;

			hifi4_priv->is_done = 1;
			complete(&hifi4_priv->cmd_complete);
		}
		break;

	case ICM_SET_PARA_CONFIG:
		{
			struct prop_config *ext_msg =
				(struct prop_config *)pmsg_apu;
			hifi4_priv->ret_status = ext_msg->ret;
			hifi4_priv->is_done = 1;
			complete(&hifi4_priv->cmd_complete);
		}
		break;

	case ICM_PI_LIB_INIT:
		hifi4_priv->is_done = 1;
		complete(&hifi4_priv->cmd_complete);
		break;

	case ICM_CLOSE:
		hifi4_priv->is_done = 1;
		complete(&hifi4_priv->cmd_complete);
		break;

	case ICM_SWITCH_CODEC:
		{
			struct icm_switch_info_t *ext_msg =
				(struct icm_switch_info_t *)pmsg_apu;
			hifi4_priv->ret_status = ext_msg->status;
			hifi4_priv->is_done = 1;
			complete(&hifi4_priv->cmd_complete);
		}
		break;

	case ICM_RESET:
		{
			struct icm_reset_info_t *ext_msg =
				(struct icm_reset_info_t *)pmsg_apu;
			hifi4_priv->ret_status = ext_msg->ret;
			hifi4_priv->is_done = 1;
			complete(&hifi4_priv->cmd_complete);
		}
		break;

	default:
		ret_val = -1;
		break;
	}
	return ret_val;
}

int send_dpu_ext_msg_addr(struct fsl_hifi4 *hifi4_priv)
{
	union icm_header_t apu_icm;
	struct hifi4_ext_msg ext_msg;
	struct hifi4_mem_msg *dpu_ext_msg =
			(struct hifi4_mem_msg *)hifi4_priv->msg_buf_virt;
	int ret_val = 0;

	apu_icm.allbits = 0;	/* clear all bits; */
	apu_icm.ack  = 0;
	apu_icm.intr = 1;
	apu_icm.msg  = ICM_EXT_MSG_ADDR;
	apu_icm.size = 8;
	ext_msg.phys = hifi4_priv->msg_buf_phys;
	/* 10 means variable numbers that need to be transferred */
	ext_msg.size = 10*4;	/* 10 * sizeof(int) */
	dpu_ext_msg->ext_msg_phys = hifi4_priv->msg_buf_phys + 2048;
	dpu_ext_msg->ext_msg_size = 2048;
	dpu_ext_msg->code_phys =  hifi4_priv->code_buf_phys;
	dpu_ext_msg->code_size =  hifi4_priv->code_buf_size;
	dpu_ext_msg->data_phys =  hifi4_priv->data_buf_phys;
	dpu_ext_msg->data_size =  hifi4_priv->data_buf_size;
	dpu_ext_msg->scratch_phys =  hifi4_priv->scratch_buf_phys;
	dpu_ext_msg->scratch_size =  hifi4_priv->scratch_buf_size;
	dpu_ext_msg->system_input_buf_phys =  hifi4_priv->in_buf_phys;
	dpu_ext_msg->system_input_buf_size =  hifi4_priv->in_buf_size;

	icm_intr_extended_send(hifi4_priv, apu_icm.allbits, &ext_msg);

	return ret_val;
}

static irqreturn_t fsl_hifi4_mu_isr(int irq, void *dev_id)
{
	struct fsl_hifi4 *hifi4_priv = dev_id;
	struct device *dev = hifi4_priv->dev;
	union icm_header_t recd_msg;
	int ret_val;
	u32 reg;

	MU_ReceiveMsg(hifi4_priv->mu_base_virtaddr, 0, &reg);
	recd_msg = (union icm_header_t)reg;
	if (recd_msg.intr == 1) {
		dev_dbg(dev, "INTR: Received ICM intr, msg 0x%08x\n",
						recd_msg.allbits);
		switch (recd_msg.msg) {
		case ICM_CORE_EXIT:
			break;
		case ICM_CORE_READY:
			send_dpu_ext_msg_addr(hifi4_priv);
			hifi4_priv->is_ready = 1;
			complete(&hifi4_priv->cmd_complete);
			break;
		case ICM_EXT_MSG_ADDR:
			break;

		case ICM_OPEN:
		case ICM_EMPTY_THIS_BUFFER:
		case ICM_CLOSE:
			break;

		case ICM_DPU_ACTION_COMPLETE:
			ret_val = process_act_complete(hifi4_priv,
							recd_msg.allbits);
			break;

		default:
			break;
		}
	} else if (recd_msg.ack == 1) {
		dev_dbg(dev, "INTR: Received ICM ack 0x%08x\n", recd_msg.size);
		recd_msg.ack = 0;
	} else {
		dev_dbg(dev, "Received false ICM intr 0x%08x\n",
							recd_msg.allbits);
	}

	return IRQ_HANDLED;
}

static void hifi4_load_firmware(const struct firmware *fw, void *context)
{
	struct fsl_hifi4 *hifi4_priv = context;
	struct device *dev = hifi4_priv->dev;
	Elf32_Ehdr *ehdr; /* Elf header structure pointer */
	Elf32_Shdr *shdr; /* Section header structure pointer */
	unsigned char *strtab = 0; /* String table pointer */
	unsigned char *image; /* Binary image pointer */
	int i; /* Loop counter */
	unsigned long addr;

	if (!fw) {
		dev_info(dev, "external firmware not found\n");
		return;
	}

	addr = (unsigned long)fw->data;
	ehdr = (Elf32_Ehdr *)addr;

	/* Find the section header string table for output info */
	shdr = (Elf32_Shdr *)(addr + ehdr->e_shoff +
			(ehdr->e_shstrndx * sizeof(Elf32_Shdr)));

	if (shdr->sh_type == SHT_STRTAB)
		strtab = (unsigned char *)(addr + shdr->sh_offset);

	/* Load each appropriate section */
	for (i = 0; i < ehdr->e_shnum; ++i) {
		shdr = (Elf32_Shdr *)(addr + ehdr->e_shoff +
				(i * sizeof(Elf32_Shdr)));

		if (!(shdr->sh_flags & SHF_ALLOC) ||
			shdr->sh_addr == 0 || shdr->sh_size == 0)
			continue;

		if (strtab) {
			dev_dbg(dev, "%sing %s @ 0x%08lx (%ld bytes)\n",
			  (shdr->sh_type == SHT_NOBITS) ? "Clear" : "Load",
				&strtab[shdr->sh_name],
				(unsigned long)shdr->sh_addr,
				(long)shdr->sh_size);
		}

		if ((!strcmp(&strtab[shdr->sh_name], ".rodata")) ||
		    (!strcmp(&strtab[shdr->sh_name], ".text"))   ||
		    (!strcmp(&strtab[shdr->sh_name], ".data"))   ||
		    (!strcmp(&strtab[shdr->sh_name], ".bss"))
		   ) {
			shdr->sh_addr = shdr->sh_addr + MEMORY_REMAP_OFFSET;
		}

		if (shdr->sh_type == SHT_NOBITS) {
			memset_hifi((void *)(hifi4_priv->regs +
					(shdr->sh_addr - hifi4_priv->paddr)),
					0,
					shdr->sh_size);
		} else {
			image = (unsigned char *)addr + shdr->sh_offset;
			memcpy_hifi((void *)(hifi4_priv->regs +
					(shdr->sh_addr - hifi4_priv->paddr)),
					(const void *)image,
					shdr->sh_size);
		}
	}

	/* start the core */
	sc_pm_cpu_start(hifi4_priv->hifi_ipcHandle,
					SC_R_DSP, true, hifi4_priv->iram);
}

/* Initialization of the MU code. */
int hifi4_mu_init(struct fsl_hifi4 *hifi4_priv)
{
	struct device *dev = hifi4_priv->dev;
	struct device_node *np;
	unsigned int	hifi_mu_id;
	u32 irq;
	int ret = 0;

	/*
	 * Get the address of MU to be used for communication with the hifi
	 */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx8-mu-hifi");
	if (!np) {
		dev_err(dev, "Cannot find MU entry in device tree\n");
		return -EINVAL;
	}
	hifi4_priv->mu_base_virtaddr = of_iomap(np, 0);
	WARN_ON(!hifi4_priv->mu_base_virtaddr);

	ret = of_property_read_u32_index(np,
				"fsl,hifi_ap_mu_id", 0, &hifi_mu_id);
	if (ret) {
		dev_err(dev, "Cannot get mu_id %d\n", ret);
		return -EINVAL;
	}

	hifi4_priv->hifi_mu_id = hifi_mu_id;

	irq = of_irq_get(np, 0);

	ret = devm_request_irq(hifi4_priv->dev, irq, fsl_hifi4_mu_isr,
				IRQF_EARLY_RESUME, "hifi4_mu_isr", hifi4_priv);
	if (ret) {
		dev_err(dev, "request_irq failed %d, err = %d\n", irq, ret);
		return -EINVAL;
	}

	if (!hifi4_priv->hifi_mu_init) {
		MU_Init(hifi4_priv->mu_base_virtaddr);
		MU_EnableRxFullInt(hifi4_priv->mu_base_virtaddr, 0);
		hifi4_priv->hifi_mu_init = 1;
	}

	return ret;
}

static const struct file_operations hifi4_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= fsl_hifi4_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = fsl_hifi4_compat_ioctl,
#endif
	.open		= fsl_hifi4_open,
	.release	= fsl_hifi4_close,
};

static int fsl_hifi4_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_hifi4 *hifi4_priv;
	const char *fw_name;
	struct resource *res;
	void __iomem *regs;
	uint32_t mu_id;
	sc_err_t sciErr;
	void *buf_virt;
	dma_addr_t buf_phys;
	int size, offset, i;
	int ret;

	hifi4_priv = devm_kzalloc(&pdev->dev, sizeof(*hifi4_priv), GFP_KERNEL);
	if (!hifi4_priv)
		return -ENOMEM;

	hifi4_priv->dev = &pdev->dev;

	/* Get the addresses and IRQ */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	hifi4_priv->paddr = res->start;
	hifi4_priv->regs  = regs;

	hifi4_priv->dram0 = hifi4_priv->paddr + DRAM0_OFFSET;
	hifi4_priv->dram1 = hifi4_priv->paddr + DRAM1_OFFSET;
	hifi4_priv->iram  = hifi4_priv->paddr + IRAM_OFFSET;
	hifi4_priv->sram  = hifi4_priv->paddr + SYSRAM_OFFSET;

	sciErr = sc_ipc_getMuID(&mu_id);
	if (sciErr != SC_ERR_NONE) {
		dev_err(&pdev->dev, "Cannot obtain MU ID\n");
		return sciErr;
	}

	sciErr = sc_ipc_open(&hifi4_priv->hifi_ipcHandle, mu_id);
	if (sciErr != SC_ERR_NONE) {
		dev_err(&pdev->dev, "Cannot open MU channel to SCU %d, %d\n",
								mu_id, sciErr);
		return sciErr;
	};

	if (sc_pm_set_resource_power_mode(hifi4_priv->hifi_ipcHandle,
			SC_R_DSP, SC_PM_PW_MODE_ON) != SC_ERR_NONE) {
		dev_err(&pdev->dev, "Error power on HIFI\n");
		return -EIO;
	}

	if (sc_pm_set_resource_power_mode(hifi4_priv->hifi_ipcHandle,
			SC_R_DSP_RAM, SC_PM_PW_MODE_ON) != SC_ERR_NONE) {
		dev_err(&pdev->dev, "Error power on HIFI RAM\n");
		return -EIO;
	}

	sciErr = sc_misc_set_control(hifi4_priv->hifi_ipcHandle, SC_R_DSP,
				SC_C_OFS_SEL, 1);
	if (sciErr != SC_ERR_NONE) {
		dev_err(&pdev->dev, "Error system address offset source select\n");
		return -EIO;
	}

	sciErr = sc_misc_set_control(hifi4_priv->hifi_ipcHandle, SC_R_DSP,
				SC_C_OFS_AUDIO, 0x20);
	if (sciErr != SC_ERR_NONE) {
		dev_err(&pdev->dev, "Error system address offset of AUDIO\n");
		return -EIO;
	}

	sciErr = sc_misc_set_control(hifi4_priv->hifi_ipcHandle, SC_R_DSP,
				SC_C_OFS_PERIPH, 0x5A);
	if (sciErr != SC_ERR_NONE) {
		dev_err(&pdev->dev, "Error system address offset of PERIPH\n");
		return -EIO;
	}

	sciErr = sc_misc_set_control(hifi4_priv->hifi_ipcHandle, SC_R_DSP,
				SC_C_OFS_IRQ, 0x51);
	if (sciErr != SC_ERR_NONE) {
		dev_err(&pdev->dev, "Error system address offset of IRQ\n");
		return -EIO;
	}

	ret = hifi4_mu_init(hifi4_priv);
	if (ret)
		return ret;

	ret = of_property_read_string(np, "fsl,hifi4-firmware", &fw_name);
	hifi4_priv->fw_name = fw_name;

	platform_set_drvdata(pdev, hifi4_priv);
	pm_runtime_enable(&pdev->dev);

	hifi4_miscdev.fops = &hifi4_fops,
	hifi4_miscdev.parent = &pdev->dev,
	ret = misc_register(&hifi4_miscdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register misc device %d\n", ret);
		return ret;
	}

	/* code buffer */
	hifi4_priv->code_buf_virt = hifi4_priv->regs  + LIBRARY_CODE_OFFSET;
	hifi4_priv->code_buf_phys = hifi4_priv->paddr + LIBRARY_CODE_OFFSET -
							MEMORY_REMAP_OFFSET;
	hifi4_priv->code_buf_size = LIBRARY_CODE_SIZE;

	size = MSG_BUF_SIZE + INPUT_BUF_SIZE +
			OUTPUT_BUF_SIZE + FIRMWARE_DATA_BUF_SIZE +
			SCRATCH_DATA_BUF_SIZE;

	buf_virt = dma_alloc_coherent(&pdev->dev, size, &buf_phys, GFP_KERNEL);
	if (!buf_virt) {
		dev_err(&pdev->dev, "failed alloc memory.\n");
		return -ENOMEM;
	}

	/* msg buffer */
	hifi4_priv->msg_buf_virt = buf_virt;
	hifi4_priv->msg_buf_phys = buf_phys;
	hifi4_priv->msg_buf_size = MSG_BUF_SIZE;
	offset = MSG_BUF_SIZE;

	/* input buffer */
	hifi4_priv->in_buf_virt = buf_virt + offset;
	hifi4_priv->in_buf_phys = buf_phys + offset;
	hifi4_priv->in_buf_size = INPUT_BUF_SIZE;
	offset += INPUT_BUF_SIZE;

	/* output buffer */
	hifi4_priv->out_buf_virt = buf_virt + offset;
	hifi4_priv->out_buf_phys = buf_phys + offset;
	hifi4_priv->out_buf_size = OUTPUT_BUF_SIZE;
	offset += OUTPUT_BUF_SIZE;

	/* codec library data section */
	hifi4_priv->data_buf_virt = buf_virt + offset;
	hifi4_priv->data_buf_phys = buf_phys + offset;
	hifi4_priv->data_buf_size = FIRMWARE_DATA_BUF_SIZE;
	offset += FIRMWARE_DATA_BUF_SIZE;

	hifi4_priv->scratch_buf_virt = buf_virt + offset;
	hifi4_priv->scratch_buf_phys = buf_phys + offset;
	hifi4_priv->scratch_buf_size = SCRATCH_DATA_BUF_SIZE;

	/* process_id_count is a counter to produce new id
	 * process_id is current codec's id
	 */
	hifi4_priv->process_id_count = 0;
	hifi4_priv->process_id = 0;

	/* initialize the resources of multi codec
	 * MULTI_CODEC_NUM is the max codec number that dsp
	 * driver and framework can support.
	 */
	hifi4_priv->available_resource = MULTI_CODEC_NUM;
	for (i = 0; i < MULTI_CODEC_NUM; i++) {
		hifi4_priv->process_info[i].data_buf_virt =
				hifi4_priv->data_buf_virt +
				i * hifi4_priv->data_buf_size / MULTI_CODEC_NUM;
		hifi4_priv->process_info[i].data_buf_phys =
				hifi4_priv->data_buf_phys +
				i * hifi4_priv->data_buf_size / MULTI_CODEC_NUM;

		hifi4_priv->process_info[i].status = 0;
	}

	/* initialize the reference counter for hifi4_priv
	 * structure
	 */
	atomic_long_set(&hifi4_priv->refcnt, 0);
	mutex_init(&hifi4_priv->hifi4_mutex);

	return 0;
}

static int fsl_hifi4_remove(struct platform_device *pdev)
{
	struct fsl_hifi4 *hifi4_priv = platform_get_drvdata(pdev);
	int size;

	misc_deregister(&hifi4_miscdev);

	size = MSG_BUF_SIZE + INPUT_BUF_SIZE +
			OUTPUT_BUF_SIZE + FIRMWARE_DATA_BUF_SIZE +
			SCRATCH_DATA_BUF_SIZE;
	dma_free_coherent(&pdev->dev, size, hifi4_priv->msg_buf_virt,
				hifi4_priv->msg_buf_phys);

	return 0;
}

#ifdef CONFIG_PM
static int fsl_hifi4_runtime_resume(struct device *dev)
{
	return 0;
}

static int fsl_hifi4_runtime_suspend(struct device *dev)
{
	return 0;
}
#endif /* CONFIG_PM */

#ifdef CONFIG_PM_SLEEP
static int fsl_hifi4_suspend(struct device *dev)
{
	return 0;
}

static int fsl_hifi4_resume(struct device *dev)
{
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops fsl_hifi4_pm = {
	SET_RUNTIME_PM_OPS(fsl_hifi4_runtime_suspend,
					fsl_hifi4_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(fsl_hifi4_suspend, fsl_hifi4_resume)
};

static const struct of_device_id fsl_hifi4_ids[] = {
	{ .compatible = "fsl,imx8qxp-hifi4", },
	{}
};
MODULE_DEVICE_TABLE(of, fsl_hifi4_ids);

static struct platform_driver fsl_hifi4_driver = {
	.probe = fsl_hifi4_probe,
	.remove = fsl_hifi4_remove,
	.driver = {
		.name = "fsl-hifi4",
		.of_match_table = fsl_hifi4_ids,
		.pm = &fsl_hifi4_pm,
	},
};
module_platform_driver(fsl_hifi4_driver);

MODULE_DESCRIPTION("Freescale HIFI 4 driver");
MODULE_ALIAS("platform:fsl-hifi4");
MODULE_LICENSE("Dual BSD/GPL");
