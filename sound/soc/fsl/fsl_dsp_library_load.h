// SPDX-License-Identifier: GPL-2.0+
// Copyright 2018 NXP
// Copyright (c) 2012-2013 by Tensilica Inc.

#ifndef FSL_DSP_LIBRARY_LOAD_H
#define FSL_DSP_LIBRARY_LOAD_H

#include "fsl_dsp_pool.h"

#define Elf32_Byte  unsigned char
#define xt_ptr      unsigned long
#define xt_int      int
#define xt_uint     unsigned int
#define xt_ulong    unsigned long

struct xtlib_packaged_library;

enum {
	XTLIB_NO_ERR = 0,
	XTLIB_NOT_ELF = 1,
	XTLIB_NOT_DYNAMIC = 2,
	XTLIB_NOT_STATIC = 3,
	XTLIB_NO_DYNAMIC_SEGMENT = 4,
	XTLIB_UNKNOWN_SYMBOL = 5,
	XTLIB_NOT_ALIGNED = 6,
	XTLIB_NOT_SPLITLOAD = 7,
	XTLIB_RELOCATION_ERR = 8
};

enum lib_type {
	DSP_CODEC_LIB = 1,
	DSP_CODEC_WRAP_LIB
};

struct xtlib_loader_globals {
	int err;
	int byteswap;
};

struct xtlib_pil_info {
	xt_uint  dst_addr;
	xt_uint  src_offs;
	xt_uint  dst_data_addr;
	xt_uint  src_data_offs;
	xt_uint  start_sym;
	xt_uint  text_addr;
	xt_uint  init;
	xt_uint  fini;
	xt_uint  rel;
	xt_int  rela_count;
	xt_uint  hash;
	xt_uint  symtab;
	xt_uint  strtab;
	xt_int  align;
};

struct icm_xtlib_pil_info {
	struct xtlib_pil_info pil_info;
	unsigned int lib_type;
};

struct lib_dnld_info_t {
	unsigned long pbuf_code;
	unsigned long pbuf_data;
	unsigned int size_code;
	unsigned int size_data;
	struct xtlib_pil_info *ppil_inf;
	unsigned int lib_on_dpu;    /* 0: not loaded, 1: loaded. */
};

struct lib_info {
	struct xtlib_pil_info  pil_info;
	struct xtlib_loader_globals	xtlib_globals;

	struct xf_pool	 *code_section_pool;
	struct xf_pool	 *data_section_pool;

	void        *code_buf_virt;
	unsigned int code_buf_phys;
	unsigned int code_buf_size;
	void        *data_buf_virt;
	unsigned int data_buf_phys;
	unsigned int data_buf_size;

	const char   *filename;
	unsigned int lib_type;
};

long xf_load_lib(struct xf_client *client, struct xf_handle *handle, struct lib_info *lib_info);
long xf_unload_lib(struct xf_client *client, struct xf_handle *handle, struct lib_info *lib_info);

#endif
