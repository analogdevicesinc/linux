/*
 * Freescale HIFI 4 driver
 *
 * Copyright 2017 NXP.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <uapi/linux/mxc_hifi4.h>


#define Elf32_Byte	unsigned char
#define xt_ptr		unsigned long
#define xt_int		int
#define xt_uint		unsigned int
#define xt_ulong	unsigned long

typedef void (*memcpy_func) (void *dest, const void *src, size_t n);
typedef void (*memset_func) (void *s, int c, size_t n);
struct xtlib_packaged_library;

#define MULTI_CODEC_NUM		5

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

struct xtlib_loader_globals {
	int err;
	int byteswap;
};

struct timestamp_info_t {
	u32 offset_last_inp_byt_read_frm_sysram;
	u32 offset_last_inp_byt_decoded;
	u32 samples_produced;
	u32 sample_rate;
};

struct icm_cdc_iobuf_t {
	u32 proc_id;			/* non-zero indicates success; */
	u32 inp_addr_sysram;		/* init by APU */
	u32 inp_buf_size_max;		/* init by APU */
	u32 inp_cur_offset;		/* init by APU, updated by DPU */
	u32 out_addr_sysram;		/* init by APU */
	u32 out_buf_size_max;		/* init by APU */
	u32 out_cur_offset;		/* init by APU, updated by DPU */
	s32 ret;
	u32 cycles;		        /* consumed cycles during executing */
	u32 input_over;			/* indicate external stream is over*/
};

struct icm_cdc_uinp_t {
	u32 proc_id;			/* audio id */
	u32 codec_id;			/* codec identifier */
};

struct icm_pcm_prop_t {
	u32 proc_id;			/* audio id */
	u32 pcmbytes;			/* total bytes in the wav file */
	u32 sfreq;			/* sample rate */
	u32 channels;			/* output channels */
	u32 bits;			/* bits per sample */
	u32 consumed_bytes;
	s32 ret;
};

struct xtlib_overlay_info {
	u32 start_addr;
	u32 codec_type;
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

union icm_header_t {
	struct {
		u32 msg:6;
		u32 sub_msg:6;		/* sub_msg will have ICM_MSG when
					 *  msg=ICM_XXX_ACTION_COMPLETE
					 */
		u32 rsvd:3;		/* reserved */
		u32 intr:1;		/* intr = 1 when sending msg. */
		u32 size:15;		/* =size in bytes (excluding header)
					 * to follow when intr=1,
					 * =response message when ack=1
					 */
		u32 ack:1;
	};
	u32 allbits;
} icm_header_t;

enum icm_action_t {
	ICM_CORE_READY = 1,
	ICM_PI_LIB_MEM_ALLOC,
	ICM_PI_LIB_INIT,
	ICM_PI_LIB_UNLOAD,

	ICM_DPU_ACTION_COMPLETE,
	ICM_APU_ACTION_COMPLETE,

	ICM_OPEN,
	ICM_EMPTY_THIS_BUFFER,
	ICM_FILL_THIS_BUFFER,
	ICM_PAUSE,
	ICM_CLOSE,

	ICM_GET_PCM_PROP,
	ICM_SET_PARA_CONFIG,

	ICM_CORE_EXIT,
	ICM_EXT_MSG_ADDR,

	ICM_SWITCH_CODEC,
};

enum aud_status_t {
	AUD_IDLE = 0,
	AUD_STOPPED,
	AUD_DECODING,
	AUD_PAUSED
};

struct icm_open_resp_info_t {
	u32 proc_id;
	u32 dtstamp;                /* address value of timestamp_info_t */

	s32 ret;
};

struct icm_switch_info_t {
	u32 proc_id;    /* audio id */
	u32 status;     /* codec status */
};

struct lib_dnld_info_t {
	unsigned long pbuf_code;
	unsigned long pbuf_data;
	unsigned int size_code;
	unsigned int size_data;
	struct xtlib_pil_info *ppil_inf;
	unsigned int lib_on_dpu;	/* 0: not loaded, 1: loaded. */
};


struct icm_pilib_size_t {
	u32 codec_type;
	u32 text_size;
	u32 data_size;
};

struct icm_process_info {
	unsigned int process_id;
	unsigned int codec_id;
	unsigned int proc_id;

	void *data_buf_virt;
	dma_addr_t data_buf_phys;

	struct xtlib_pil_info pil_info_info;
	unsigned int status;
};

struct fsl_hifi4 {
	struct device			*dev;
	const char			*fw_name;
	void __iomem			*regs;
	void __iomem			*mu_base_virtaddr;
	sc_ipc_t			hifi_ipcHandle;
	sc_ipc_t			mu_ipcHandle;
	unsigned int			hifi_mu_id;
	int				hifi_mu_init;
	unsigned long			paddr;
	unsigned long			dram0;
	unsigned long			dram1;
	unsigned long			iram;
	unsigned long			sram;
	void				*msg_buf_virt;
	dma_addr_t			 msg_buf_phys;
	int				 msg_buf_size;
	void				*in_buf_virt;
	dma_addr_t			 in_buf_phys;
	int				 in_buf_size;
	void				*out_buf_virt;
	dma_addr_t			 out_buf_phys;
	int				 out_buf_size;

	void				*code_buf_virt;
	dma_addr_t			 code_buf_phys;
	int				 code_buf_size;
	void				*data_buf_virt;
	dma_addr_t			 data_buf_phys;
	int				 data_buf_size;
	void				*scratch_buf_virt;
	dma_addr_t			 scratch_buf_phys;
	int				 scratch_buf_size;

	int				is_ready;
	int				is_done;

	struct completion		cmd_complete;
	char				*objmem;
	struct filename			*objfile;
	char				objtype;
	unsigned int			start_addr;
	int				ret_status;
	struct icm_cdc_iobuf_t		codec_iobuf_info;
	struct icm_pcm_prop_t		pcm_prop_info;
	struct xtlib_pil_info		pil_info;
	struct xtlib_loader_globals	xtlib_globals;
	struct timestamp_info_t		*dpu_tstamp;

	struct mutex hifi4_mutex;

	unsigned int process_id;
	unsigned int process_id_count;

	unsigned int size_code;
	unsigned int size_data;

	struct icm_process_info process_info[MULTI_CODEC_NUM];
	unsigned int available_resource;
	unsigned int cur_res_id;
};

struct fsl_hifi4_engine {
	struct fsl_hifi4		*hifi4_priv;
};

struct hifi4_ext_msg {
	u32	phys;
	u32	size;
};

struct hifi4_mem_msg {
	u32	ext_msg_phys;
	u32	ext_msg_size;
	u32	code_phys;
	u32	code_size;
	u32	data_phys;
	u32	data_size;
	u32	scratch_phys;
	u32	scratch_size;
	u32	system_input_buf_phys;
	u32	system_input_buf_size;
};

#define IRAM_OFFSET		0x10000
#define IRAM_SIZE		2048

#define DRAM0_OFFSET		0x0
#define DRAM0_SIZE		0x8000

#define DRAM1_OFFSET		0x8000
#define DRAM1_SIZE		0x8000

#define SYSRAM_OFFSET		0x18000
#define SYSRAM_SIZE		0x40000

#define SYSROM_OFFSET		0x58000
#define SYSROM_SIZE		0x30000

#define LIBRARY_CODE_OFFSET	0x38000
#define LIBRARY_CODE_SIZE	0x50000

#define MSG_BUF_SIZE		4096
#define INPUT_BUF_SIZE		4096
#define OUTPUT_BUF_SIZE		16384
#define FIRMWARE_DATA_BUF_SIZE	(MULTI_CODEC_NUM * 0x80000)
#define SCRATCH_DATA_BUF_SIZE	(MULTI_CODEC_NUM * 0x80000)

#define MEMORY_REMAP_OFFSET	0x39000000

#define SC_C_OFS_SEL    39
#define SC_C_OFS_AUDIO  40
#define SC_C_OFS_PERIPH 41
#define SC_C_OFS_IRQ    42

static void hifi4_load_firmware(const struct firmware *fw, void *context);
u32 icm_intr_send(struct fsl_hifi4 *hifi4_priv, u32 msg);
u32 icm_intr_extended_send(struct fsl_hifi4 *hifi4_priv, u32 msg,
					struct hifi4_ext_msg *ext_msg);
long icm_ack_wait(struct fsl_hifi4 *hifi4_priv, u32 msg);

unsigned int xtlib_split_pi_library_size(
				struct xtlib_packaged_library *library,
				unsigned int *code_size,
				unsigned int *data_size,
				struct fsl_hifi4 *hifi4_priv);

xt_ptr xtlib_host_load_split_pi_library(
				struct xtlib_packaged_library *library,
				  xt_ptr destination_code_address,
				  xt_ptr destination_data_address,
				  struct xtlib_pil_info *lib_info,
				  memcpy_func mcpy_fn,
				  memset_func mset_fn,
				  struct fsl_hifi4 *hifi4_priv);

void *memcpy_hifi(void *dest, const void *src, size_t count);
void *memset_hifi(void *dest, int c, size_t count);
