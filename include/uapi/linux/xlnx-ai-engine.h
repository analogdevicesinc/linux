/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Copyright (c) 2020, Xilinx Inc.
 */

#ifndef _UAPI_AI_ENGINE_H_
#define _UAPI_AI_ENGINE_H_


#include <linux/ioctl.h>
#include <linux/types.h>

enum aie_reg_op {
	AIE_REG_WRITE,
	AIE_REG_BLOCKWRITE,
	AIE_REG_BLOCKSET,
	AIE_REG_MASKWRITE,
	AIE_REG_MASKPOLL,
	AIE_CONFIG_SHIMDMA_BD,
	AIE_CONFIG_SHIMDMA_DMABUF_BD,
};

/**
 * enum aie_module_type - identifies different hardware modules within a
 *			  tile type. AIE tile may have memory and core
 *			  module. While a PL or shim tile may have PL module.
 * @AIE_MEM_MOD: comprises of the following sub-modules,
 *			* data memory.
 *			* tile DMA.
 *			* lock module.
 *			* events, event broadcast and event actions.
 *			* tracing and profiling.
 * @AIE_CORE_MOD: comprises of the following sub-modules,
 *			* AIE core.
 *			* program Memory.
 *			* events, event broadcast and event actions.
 *			* tracing and profiling.
 *			* AXI-MM and AXI-S tile interconnects.
 * @AIE_PL_MOD: comprises of the following sub-modules,
 *			* PL interface.
 *			* AXI-MM and AXI-S tile interconnects.
 *			* Level 1 interrupt controllers.
 *			* events, event broadcast and event actions.
 *			* tracing and profiling.
 * @AIE_NOC_MOD: comprises of the following sub-modules,
 *			* interface from NoC Slave Unit (NSU)
 *			  (bridge to AXI-MM switch)
 *			* interfaces to NoC NoC Master Unit (NMU)
 *				* shim DMA & locks
 *				* NoC stream interface
 */
enum aie_module_type {
	AIE_MEM_MOD,
	AIE_CORE_MOD,
	AIE_PL_MOD,
	AIE_NOC_MOD,
};

/**
 * enum aie_rsc_type - defines AI engine hardware resource types
 * @AIE_RSCTYPE_PERF: perfcounter resource
 * @AIE_RSCTYPE_USEREVENT: user events resource
 * @AIE_RSCTYPE_TRACECONTROL: trace controller resource
 * @AIE_RSCTYPE_PCEVENT: PC events resource
 * @AIE_RSCTYPE_SSSELECT: stream switch port select resource
 * @AIE_RSCTYPE_BROADCAST: broadcast events resource
 * @AIE_RSCTYPE_COMBOEVENT: combo events resource
 * @AIE_RSCTYPE_GROUPEVENTS: group events resource
 * @AIE_RSCTYPE_MAX: total number of resources
 */
enum aie_rsc_type {
	AIE_RSCTYPE_PERF,
	AIE_RSCTYPE_USEREVENT,
	AIE_RSCTYPE_TRACECONTROL,
	AIE_RSCTYPE_PCEVENT,
	AIE_RSCTYPE_SSSELECT,
	AIE_RSCTYPE_BROADCAST,
	AIE_RSCTYPE_COMBOEVENT,
	AIE_RSCTYPE_GROUPEVENTS,
	AIE_RSCTYPE_MAX
};

/**
 * enum aie_part_status - defines AI engine partition status
 * @XAIE_PART_STATUS_IDLE: partition is idle
 * @XAIE_PART_STATUS_INUSE: partition is in use
 * @XAIE_PART_STATUS_INVALID: partition is invalid to the system
 *			      that is system cannot see the partition
 */
enum aie_part_status {
	XAIE_PART_STATUS_IDLE,
	XAIE_PART_STATUS_INUSE,
	XAIE_PART_STATUS_INVALID,
};

/*
 * AI engine partition control flags
 */
/* Not reset when release AI engine partition */
#define XAIE_PART_NOT_RST_ON_RELEASE	0x00000001U

/*
 * AI engine resource property flags
 */
/*
 * For resources which needs to be allocated contiguous
 * such as combo events, it needs to be 0, 1; 2, 3;
 * or 0, 1, 2, 3
 */
#define XAIE_RSC_PATTERN_BLOCK		(1U << 0)

/* Any broadcast channel id */
#define XAIE_BROADCAST_ID_ANY		0xFFFFFFFFU

/* request a channel to broadcast to the whole partition */
#define XAIE_BROADCAST_ALL		(1U << 0)

/**
 * struct aie_location - AIE location information
 * @col: column id
 * @row: row id
 */
struct aie_location {
	__u32 col;
	__u32 row;
};

/**
 * struct aie_location_byte - AIE location information with single byte for
 *			      column and row
 * @row: row id
 * @col: column id
 *
 * This structure follows the SSW AIE row and col sequence.
 */
struct aie_location_byte {
	__u8 row;
	__u8 col;
};

/**
 * struct aie_range - AIE range information
 * @start: start tile location
 * @size: size of the range, number of columns and rows
 */
struct aie_range {
	struct aie_location start;
	struct aie_location size;
};

/**
 * struct aie_mem - AIE memory information
 * @range: range of tiles of the memory
 * @offset: register offset within a tile of the memory
 * @size: of a the memory in one tile
 * @fd: file descriptor of the memory
 */
struct aie_mem {
	struct aie_range range;
	__kernel_size_t offset;
	__kernel_size_t size;
	int fd;
};

/**
 * struct aie_mem_args - AIE memory enquiry arguments
 * @num_mems: number of "struct aie_mem" elements
 *	      e.g. two memory information elements, one for tile core memory,
 *	      and the other for tile data memory.
 * @mems: array of AI engine memory information elements
 */
struct aie_mem_args {
	unsigned int num_mems;
	struct aie_mem *mems;
};

/**
 * struct aie_reg_args - AIE access register arguments
 * @op: if this request is to read, write or poll register
 * @mask: mask for mask write, 0 for not mask write
 * @offset: offset of register to the start of an AI engine partition
 * @val: value to write or get
 * @dataptr: pointer to data buffer for block write
 * @len: length of the buffer pointed by data
 */
struct aie_reg_args {
	enum aie_reg_op op;
	__u32 mask;
	__u64 offset;
	__u32 val;
	__u64 dataptr;
	__u32 len;
};

/**
 * struct aie_range_args - AIE range request arguments
 * @partition_id: partition id. It is used to identify the
 *		  AI engine partition in the system.
 * @uid: image identifier loaded on the AI engine partition
 * @range: range of AIE tiles
 * @status: indicate if the AI engine is in use.
 *	    0 means not in used, otherwise, in use.
 */
struct aie_range_args {
	__u32 partition_id;
	__u32 uid;
	struct aie_range range;
	__u32 status;
};

/**
 * struct aie_partition_query - AIE partition query arguments
 * @partition_cnt: number of defined partitions in the system
 * @partitions: buffer to store defined partitions information.
 */
struct aie_partition_query {
	struct aie_range_args *partitions;
	__u32 partition_cnt;
};

#define AIE_PART_ID_START_COL_SHIFT	0U
#define AIE_PART_ID_NUM_COLS_SHIFT	8U
#define AIE_PART_ID_START_COL_MASK	(0xFFU << AIE_PART_ID_START_COL_SHIFT)
#define AIE_PART_ID_NUM_COLS_MASK	(0xFFU << AIE_PART_ID_NUM_COLS_SHIFT)

#define aie_part_id_get_val(part_id, F) \
	(((part_id) & AIE_PART_ID_##F ##_MASK) >> AIE_PART_ID_##F ##_SHIFT)
#define aie_part_id_get_start_col(part_id) \
	aie_part_id_get_val((part_id), START_COL)
#define aie_part_id_get_num_cols(part_id) \
	aie_part_id_get_val((part_id), NUM_COLS)

/**
 * struct aie_partition_req - AIE request partition arguments
 * @partition_id: partition node id. It is used to identify the AI engine
 *		  partition. Its format is:
 *		  Reserved_16bits | start_col_8bits | num_cols_8bits
 * @uid: image identifier loaded on the AI engine partition
 * @meta_data: meta data to indicate which resources used by application.
 * @flag: used for application to indicate particular driver requirements
 *	  application wants to have for the partition. e.g. do not clean
 *	  resource when closing the partition.
 * @user_event1_complete: inference complete call back function.
 * @user_event1_priv: user_event1_complete's priv data.
 */
struct aie_partition_req {
	__u32 partition_id;
	__u32 uid;
	__u64 meta_data;
	__u32 flag;
	void (*user_event1_complete)(__u32 partition_id, void *user_event1_prov);
	void *user_event1_priv;
};

/**
 * struct aie_partition_init_args - AIE partition initialization arguments
 * @locs: Allocated array of tile locations that will be used
 * @num_tiles: Number of tiles to use
 * @init_opts: Partition initialization options
 * @ecc_scrub: ecc_scrub period.
 * @handshake: HSA handshake data.
 * @handshake_size: handshake data size.
 */
struct aie_partition_init_args {
	struct aie_location *locs;
	__u32 num_tiles;
	__u32 init_opts;
	__u32 ecc_scrub;
	__u32 *handshake;
	__u32 handshake_size;
};

/*
 * AI engine partition initialize options
 */
#define AIE_PART_INIT_OPT_START_NUM_COL		0U
#define AIE_PART_INIT_OPT_COLUMN_RST		BIT(0)
#define AIE_PART_INIT_OPT_SHIM_RST		BIT(1)
#define AIE_PART_INIT_OPT_BLOCK_NOCAXIMMERR	BIT(2)
#define AIE_PART_INIT_OPT_ISOLATE		BIT(3)
#define AIE_PART_INIT_OPT_ZEROIZEMEM		BIT(4)
#define AIE_PART_INIT_OPT_UC_ENB_MEM_PRIV	BIT(5)
#define AIE_PART_INIT_ERROR_HANDLING		BIT(6)
#define AIE_PART_INIT_OPT_DIS_COLCLK_BUFF	BIT(7)
#define AIE_PART_INIT_OPT_SET_L2_IRQ		BIT(8)
#define AIE_PART_INIT_OPT_HW_ERR_INT		BIT(9)
#define AIE_PART_INIT_OPT_HW_ERR_MASK		BIT(10)
#define AIE_PART_INIT_OPT_SET_ECC_SCRUB_PERIOD	BIT(11)
#define AIE_PART_INIT_OPT_NMU_CONFIG		BIT(12)
#define AIE_PART_INIT_OPT_ENB_COLCLK_BUFF	BIT(13)
#define AIE_PART_INIT_OPT_UC_ZEROIZATION	BIT(14)
#define AIE_PART_INIT_OPT_UC_DIS_MEM_PRIV	BIT(15)
#define AIE_PART_INIT_OPT_ENB_NOC_DMA_PAUSE	BIT(16)
#define AIE_PART_INIT_OPT_ENB_UC_DMA_PAUSE	BIT(17)
#define AIE_PART_INIT_OPT_HW_ERR_STS		BIT(18)
#define AIE_PART_INIT_OPT_DIS_MEM_INTERLEAVE	BIT(19)
#define AIE_PART_INIT_OPT_HANDSHAKE		BIT(20)

#define AIE_PART_INIT_OPT_DEFAULT	(AIE_PART_INIT_OPT_COLUMN_RST		|	\
					 AIE_PART_INIT_OPT_SHIM_RST		|	\
					 AIE_PART_INIT_OPT_BLOCK_NOCAXIMMERR	|	\
					 AIE_PART_INIT_OPT_ISOLATE		|	\
					 AIE_PART_INIT_ERROR_HANDLING		|	\
					 AIE_PART_INIT_OPT_UC_ENB_MEM_PRIV	|	\
					 AIE_PART_INIT_OPT_SET_L2_IRQ		|	\
					 AIE_PART_INIT_OPT_HW_ERR_INT		|	\
					 AIE_PART_INIT_OPT_HW_ERR_MASK		|	\
					 AIE_PART_INIT_OPT_NMU_CONFIG)

#define AIE_PART_TEARDOWN_OPT_DEFAULT	(AIE_PART_INIT_OPT_COLUMN_RST		|	\
					 AIE_PART_INIT_OPT_SHIM_RST		|	\
					 AIE_PART_INIT_OPT_ZEROIZEMEM		|	\
					 AIE_PART_INIT_OPT_ENB_NOC_DMA_PAUSE	|	\
					 AIE_PART_INIT_OPT_ENB_UC_DMA_PAUSE	|	\
					 AIE_PART_INIT_OPT_DIS_COLCLK_BUFF)

/*
 * AI engine partition uc zeroization options
 */
#define AIE_PART_ZEROIZE_UC_PM			BIT(0)
#define AIE_PART_ZEROIZE_UC_PRIVATE_DM		BIT(1)
#define AIE_PART_ZEROIZE_UC_SHARED_DM		BIT(2)
#define AIE_PART_ZEROIZE_UC_MEM_ALL		(AIE_PART_ZEROIZE_UC_PM |		\
						 AIE_PART_ZEROIZE_UC_PRIVATE_DM |	\
						 AIE_PART_ZEROIZE_UC_SHARED_DM)		\

/**
 * struct aie_dma_bd_args - AIE DMA buffer descriptor information
 * @bd: DMA buffer descriptor
 * @data_va: virtual address of the data
 * @loc: Tile location relative to the start of a partition
 * @bd_id: buffer descriptor id
 */
struct aie_dma_bd_args {
	__u32 *bd;
	__u64 data_va;
	struct aie_location loc;
	__u32 bd_id;
};

/**
 * struct aie_dmabuf_bd_args - AIE dmabuf buffer descriptor information
 * @bd: DMA buffer descriptor, within the buffer descriptor, the address field
 *	will be the offset to the start of the dmabuf
 * @buf_fd: DMA buffer handler which is dmabuf file descriptor
 * @loc: Tile location relative to the start of a partition
 * @bd_id: buffer descriptor id
 */
struct aie_dmabuf_bd_args {
	__u32 *bd;
	struct aie_location loc;
	int buf_fd;
	__u32 bd_id;
};

/**
 * struct aie_tiles_array - AIE tiles array
 * @locs: tiles locations array
 * @num_tiles: number of tiles in the tiles locations array
 */
struct aie_tiles_array {
	struct aie_location *locs;
	__u32 num_tiles;
};

/**
 * struct aie_column_args - AIE columns args
 * @start_col : start column
 * @num_cols : number of columns in aie
 * @enable : enable/disable the columns
 */
struct aie_column_args {
	__u32 start_col;
	__u32 num_cols;
	__u8 enable;
};

/**
 * struct aie_part_fd - AIE partition fd
 * @args: aie column args
 * @partition_id: It is used to identify the
 *		  AI engine partition in the system
 * @uid: It is uid of aie partition
 * @fd: It is file descriptor of AI engine partition
 */
struct aie_part_fd {
	struct aie_column_args args;
	__u32 partition_id;
	__u32 uid;
	int fd;
};

/**
 * struct aie_part_fd_list - AIE partition fd list
 * @list: aie parititon list
 * @num_entries: num of entries in the list
 */
struct aie_part_fd_list {
	struct aie_part_fd *list;
	int num_entries;
};

/**
 * struct aie_txn_inst - AIE transaction instance
 * @num_cmds: number commands containing register ops
 * @cmdsptr: pointer to the buffer containing register ops
 */
struct aie_txn_inst {
	__u32 num_cmds;
	__u64 cmdsptr;
};

/**
 * struct aie_rsc_req - AIE resource request
 * @loc: tile location
 * @mod: module type
 * @type: resource type
 * @num_rscs: number of resource per request
 * @flag: resource property, such as it needs to be in pattern block such as
 *	  if @num_rscs is 2, it needs to be 0,1; 2,3, or 4,5
 */
struct aie_rsc_req {
	struct aie_location loc;
	__u32 mod;
	__u32 type;
	__u32 num_rscs;
	__u8 flag;
};

/**
 * struct aie_rsc - AIE resource properties
 * @loc: tile location, single byte for column and row each
 * @mod: module type
 * @type: resource type
 * @id: resource id
 */
struct aie_rsc {
	struct aie_location_byte loc;
	__u32 mod;
	__u32 type;
	__u32 id;
};

/**
 * struct aie_rsc_req_rsp - AIE resource request and response structure
 * @req: resource request per tile module
 * @rscs: allocated resources array of `struct aie_rsc`
 */
struct aie_rsc_req_rsp {
	struct aie_rsc_req req;
	__u64 rscs;
};

/**
 * struct aie_rsc_bc_req - AIE broadcast channel request
 * @rscs: broadcast channel resource array for every module and every tile
 *	  of the channel
 * @num_rscs: number of expected broadcast channel resources on the path,
 *	      it also indicates the number of expected modules on the path.
 * @flag: user flag to indicate if it is to get a broadcast channel for the
 *	  whole partition.
 * @id: broadcast channel ID. XAIE_BROADCAST_ID_ANY, it means not particular
 *	id is specified, driver will allocate a free one.
 */
struct aie_rsc_bc_req {
	__u64 rscs;
	__u32 num_rscs;
	__u32 flag;
	__u32 id;
};

/* AI engine resource statistics types */
#define AIE_RSC_STAT_TYPE_STATIC	0U
#define AIE_RSC_STAT_TYPE_AVAIL		1U
#define AIE_RSC_STAT_TYPE_MAX		2U

/**
 * struct aie_rsc_user_stat - AIE user requested resource statistics
 * @loc: tile location, single byte for column and row each
 * @mod: module type
 * @type: resource type
 * @num_rscs: number of resources
 */
struct aie_rsc_user_stat {
	struct aie_location_byte loc;
	__u8 mod;
	__u8 type;
	__u8 num_rscs;
} __attribute__((packed, aligned(4)));

/**
 * struct aie_rsc_user_stat_array - AIE user requested resource statistics array
 * @stats: resource statistics array
 * @num_stats: number of resource statistics elements
 * @stats_type: resource statistics type
 */
struct aie_rsc_user_stat_array {
	__u64 stats;
	__u32 num_stats;
	__u32 stats_type;
};

#define AIE_IOCTL_BASE 'A'

/* AI engine device IOCTL operations */
#define AIE_ENQUIRE_PART_IOCTL		_IOWR(AIE_IOCTL_BASE, 0x1, \
					      struct aie_partition_query)
#define AIE_REQUEST_PART_IOCTL		_IOR(AIE_IOCTL_BASE, 0x2, \
					     struct aie_partition_req)
#define AIE_GET_PARTITION_FD_LIST_IOCTL	_IOWR(AIE_IOCTL_BASE, 0x3, \
					     struct aie_part_fd_list)

/* AI engine partition IOCTL operations */
/**
 * DOC: AIE_PARTITION_INIT_IOCTL - initializes AI engine partition
 *
 * This ioctl is used initialize a partition. Options parameter can
 * be passed for initialization options. This operation does the
 * following steps to initialize AI engine partition:
 * 1. Clock gate all columns
 * 2. Enable column reset
 * 3. Ungate all columns
 * 4. Disable column reset
 * 5. Reset shim tiles
 * 6. Setup AXI MM not to return errors for AXI decode or slave
 *    errors, raise events instead.
 * 7. Setup partition isolation
 * 8. Zeroize memory
 */
#define AIE_PARTITION_INIT_IOCTL	_IOW(AIE_IOCTL_BASE, 0x3, \
					     struct aie_partition_init_args)

/**
 * DOC: AIE_PARTITION_TEAR_IOCTL - teardown AI engine partition
 *
 * This ioctl is used teardown a partition. This operation does the
 * following steps to teardown AI engine partition:
 * 1. Clock gate all columns
 * 2. Enable column reset
 * 3. Ungate all columns
 * 4. Disable column reset
 * 5. Reset shim tiles
 * 6. Zeroize memory
 * 7. Clock gate all columns
 */
#define AIE_PARTITION_TEAR_IOCTL	_IO(AIE_IOCTL_BASE, 0x4)

/**
 * DOC: AIE_PARTITION_CLR_CONTEXT_IOCTL - clear context of AI engine partition
 *
 * This ioctl is used to clear context of a partition.  This operation does the
 * following steps to clear context of AI engine partition:
 * - Gate all columns
 * - Reset AI engine partition columns
 * - Ungate all columns
 * - Reset shim tiles
 * - Setup axi mm to raise events
 * - Setup partition isolation
 * - Zeroize data memory
 * - Setup L2 intrupt
 */
#define AIE_PARTITION_CLR_CONTEXT_IOCTL _IO(AIE_IOCTL_BASE, 0x5)

#define AIE_REG_IOCTL			_IOWR(AIE_IOCTL_BASE, 0x8, \
					      struct aie_reg_args)
/**
 * DOC: AIE_GET_MEM_IOCTL - enquire information of memories in the AI engine
 *			    partition
 * This ioctl is used to get the information of all the different types of
 * memories in the AI engine partition. Application can get the memories
 * information in two steps:
 * 1. passing 0 as @num_mems in struct aie_mem_args to enquire the number of
 *    different memories in the partition, the value will be returned in
 *    @num_mems.
 * 2. passing the number of memories in @num_mems and valid pointer as @mems of
 *    struct aie_mem_args to store the details information of different
 *    memories. The driver will create DMA buf for each type of memories, and
 *    will return the memory addressing information along with the DMA buf file
 *    descriptors in @mems.
 * After getting the memories information, user can use mmap() with the DMA buf
 * file descriptor to enable access the memories from userspace.
 */
#define AIE_GET_MEM_IOCTL		_IOWR(AIE_IOCTL_BASE, 0x9, \
					      struct aie_mem_args)
/**
 * DOC: AIE_ATTACH_DMABUF_IOCTL - attach a dmabuf to AI engine partition
 *
 * This ioctl is used to attach a dmabuf to the AI engine partition. AI engine
 * partition will return the number of scatter gather list elements of the
 * dmabuf.
 */
#define AIE_ATTACH_DMABUF_IOCTL		_IOR(AIE_IOCTL_BASE, 0xa, int)

/**
 * DOC: AIE_DETACH_DMABUF_IOCTL - dettach a dmabuf from AI engine partition
 *
 * This ioctl is used to detach a dmabuf from the AI engine partition
 */
#define AIE_DETACH_DMABUF_IOCTL		_IOR(AIE_IOCTL_BASE, 0xb, int)

/**
 * DOC: AIE_SET_DMABUF_BD_IOCTL - set buffer descriptor to SHIM DMA
 *
 * This ioctl is used to set the buffer descriptor to SHIM DMA
 */
#define AIE_SET_SHIMDMA_BD_IOCTL	_IOW(AIE_IOCTL_BASE, 0xd, \
					     struct aie_dma_bd_args)

/**
 * DOC: AIE_REQUEST_TILES_IOCTL - request AI engine tiles
 *
 * This ioctl is used to request tiles.
 * When requested the AI engine partition, the kernel driver will scan the
 * partition to track which tiles are enabled or not. After that, if user
 * want to request for more tiles, it will use this ioctl to request more
 * tiles.
 * If the aie_tiles_array is empty, it means it will request for all tiles
 * in the partition.
 */
#define AIE_REQUEST_TILES_IOCTL		_IOW(AIE_IOCTL_BASE, 0xe, \
					     struct aie_tiles_array)

/**
 * DOC: AIE_RELEASE_TILES_IOCTL - release AI engine tiles
 *
 * This ioctl is used to release tiles
 */
#define AIE_RELEASE_TILES_IOCTL		_IOW(AIE_IOCTL_BASE, 0xf, \
					     struct aie_tiles_array)

/**
 * DOC: AIE_SET_SHIMDMA_DMABUF_BD_IOCTL - set buffer descriptor which contains
 *					  dmabuf to SHIM DMA
 *
 * This ioctl is used to set the buffer descriptor to SHIM DMA. The
 * aie_dmabuf_bd_args contains the dmabuf fd and the buffer descriptor contents.
 * The address field in the buffer descriptor contents should be the offset to
 * the start of the dmabuf.
 */
#define AIE_SET_SHIMDMA_DMABUF_BD_IOCTL	_IOW(AIE_IOCTL_BASE, 0x10, \
					     struct aie_dmabuf_bd_args)

/**
 * DOC: AIE_TRANSACTION_IOCTL - execute the register operations to
 *					configure AIE partition
 *
 * This ioctl is used to perform multiple register operations like write,
 * mask write, block set and block write on AIE partition. The aie_txn_inst
 * contains the buffer with all the register operations required by the
 * application.
 */
#define AIE_TRANSACTION_IOCTL		_IOWR(AIE_IOCTL_BASE, 0x11, \
					     struct aie_txn_inst)

/**
 * DOC: AIE_RSC_REQ_IOCTL - request a type of resources of a tile
 *
 * This ioctl is used to request a type of resources of a tile of an AI engine
 * partition.
 * AI engine partitition driver will check if there are the requested number
 * of resources available. If yes, fill in the allcoated resource IDs in the
 * resources array provided by user.
 */
#define AIE_RSC_REQ_IOCTL		_IOW(AIE_IOCTL_BASE, 0x14, \
					     struct aie_rsc_req_rsp)

/**
 * DOC: AIE_RSC_REQ_SPECIFIC_IOCTL - request statically allocated resource
 *
 * This ioctl is used to request to use a specified allcoated resource
 * AI engine partitition driver will check if the resource has been allocated
 * at compilation time. If yes, and no one else has requested it, it returns
 * success.
 */
#define AIE_RSC_REQ_SPECIFIC_IOCTL	_IOW(AIE_IOCTL_BASE, 0x15, \
					     struct aie_rsc)

/**
 * DOC: AIE_RSC_RELEASE_IOCTL - release allocated resource
 *
 * This ioctl is used to release a resource and returns it to the resource
 * pool, so that next time if user want to request for a resource, it is
 * available
 */
#define AIE_RSC_RELEASE_IOCTL		_IOW(AIE_IOCTL_BASE, 0x16, \
					     struct aie_rsc)

/**
 * DOC: AIE_RSC_FREE_IOCTL - free allocated resource
 *
 * This ioctl is used to free an allocated resource. It will unmark the
 * resource from runtime used. If the resource is allocated at compilation
 * time, it will not be returned back to the resource pool.
 */
#define AIE_RSC_FREE_IOCTL		_IOW(AIE_IOCTL_BASE, 0x17, \
					     struct aie_rsc)

/**
 * DOC: AIE_RSC_CHECK_AVAIL_IOCTL - check if resource is available
 *
 * This ioctl is used to check how many resources are available for a specified
 * type of resource.
 */
#define AIE_RSC_CHECK_AVAIL_IOCTL	_IOW(AIE_IOCTL_BASE, 0x18, \
					     struct aie_rsc_req)

/**
 * DOC: AIE_RSC_GET_COMMON_BROADCAST_IOCTL - get a common broadcast channel for
 *					     the specified set of AI engine
 *					     modules.
 *
 * This ioctl is used to get a common broadcast channel for the specified set
 * of AI engine modules in the resources array. If the any of the input set of
 * tiles is gated, it will return failure. This ioctl will not check the
 * connection of the input modules set.
 * The driver will fill in the resource ID with the assigned broadcast channel
 * ID of the resources array.
 * If the XAIE_BROADCAST_ALL is set in the request flag, it will get the
 * broadcast channel for all the ungated tiles of the partition.
 * If a particular broadcast channel id is specified in the request, if will
 * check if the channel is available for the specified modules, or the whole
 * partition depends on if XAIE_BROADCAST_ALL is set.
 */
#define AIE_RSC_GET_COMMON_BROADCAST_IOCTL	_IOW(AIE_IOCTL_BASE, 0x19, \
						struct aie_rsc_bc_req)

/**
 * DOC: AIE_RSC_GET_STAT_IOCTL - get resource usage statistics
 *
 * This ioctl is used to get resource usage statistics. User passes an array of
 * resource statistics requests and the resources statistics type that is if it
 * is statically allocated or available resources. Each request specifies the
 * tile, module type and the resource type. This ioctl returns the number of
 * resources of the specified statistics type per request.
 */
#define AIE_RSC_GET_STAT_IOCTL		_IOW(AIE_IOCTL_BASE, 0x1a, \
					struct aie_rsc_user_stat_array)

/**
 * DOC: AIE_SET_COLUMN_CLOCK_IOCTL - enable/disable the column clock
 *
 * This ioctl is used to enable and disable the column clock.
 * User passes an aie_tile_array, If enable is set, It will request the
 * AI engine partition, the kernel driver will scan the partition to track
 * which tiles are enabled or not. After that, if user want to request for
 * more tiles, it will use this ioctl to request more tiles.
 * If enable is set to zero, It will release the AI engine titles
 * If the aie_tiles_array is empty, it means it will request/release for all
 * tiles in the partition.
 */
#define AIE_SET_COLUMN_CLOCK_IOCTL	_IOW(AIE_IOCTL_BASE, 0x1b, \
					struct aie_tiles_array)

/**
 * DOC: AIE_DMA_MEM_ALLOCATE_IOCTL - allocates the DMA memory
 *
 * This ioctl is used to allocate physically contiguous memory for DMA
 * transactions and export it as a dma-buf which is passed to the userspace as
 * a file descriptor. The file descriptor is memory mapped into the process's
 * address space.
 */
#define AIE_DMA_MEM_ALLOCATE_IOCTL	_IOW(AIE_IOCTL_BASE, 0x1c, \
					     __kernel_size_t)

/**
 * DOC: AIE_DMA_MEM_FREE_IOCTL - de-allocates the DMA memory
 *
 * This ioctl is used to de-allocate physically contiguous memory for DMA
 * transactions and unexport it as a dma-buf which is passed to the userspace as
 * a file descriptor.
 */
#define AIE_DMA_MEM_FREE_IOCTL          _IOW(AIE_IOCTL_BASE, 0x1d, int)

/**
 * DOC: AIE_UPDATE_SHIMDMA_DMABUF_BD_ADDR_IOCTL - updates the SHIM DMA address
 *						  in the BD registers.
 *
 * This ioctl is used to update the SHIM DMA address. The
 * aie_dmabuf_bd_args contains the dmabuf fd and the offset. The buffer
 * descriptor only contains the offset to be updated.
 */
#define AIE_UPDATE_SHIMDMA_DMABUF_BD_ADDR_IOCTL	_IOW(AIE_IOCTL_BASE, 0x1e, \
						struct aie_dmabuf_bd_args)

#endif
