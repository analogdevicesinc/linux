/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Type definitions for the Microsoft Hypervisor.
 */
#ifndef _HV_HVHDK_MINI_H
#define _HV_HVHDK_MINI_H

#include "hvgdk_mini.h"

/*
 * Doorbell connection_info flags.
 */
#define HV_DOORBELL_FLAG_TRIGGER_SIZE_MASK  0x00000007
#define HV_DOORBELL_FLAG_TRIGGER_SIZE_ANY   0x00000000
#define HV_DOORBELL_FLAG_TRIGGER_SIZE_BYTE  0x00000001
#define HV_DOORBELL_FLAG_TRIGGER_SIZE_WORD  0x00000002
#define HV_DOORBELL_FLAG_TRIGGER_SIZE_DWORD 0x00000003
#define HV_DOORBELL_FLAG_TRIGGER_SIZE_QWORD 0x00000004
#define HV_DOORBELL_FLAG_TRIGGER_ANY_VALUE  0x80000000

/* Each generic set contains 64 elements */
#define HV_GENERIC_SET_SHIFT		(6)
#define HV_GENERIC_SET_MASK		(63)

enum hv_generic_set_format {
	HV_GENERIC_SET_SPARSE_4K,
	HV_GENERIC_SET_ALL,
};
#define HV_GENERIC_SET_FORMAT hv_generic_set_format

enum hv_scheduler_type {
	HV_SCHEDULER_TYPE_LP		= 1, /* Classic scheduler w/o SMT */
	HV_SCHEDULER_TYPE_LP_SMT	= 2, /* Classic scheduler w/ SMT */
	HV_SCHEDULER_TYPE_CORE_SMT	= 3, /* Core scheduler */
	HV_SCHEDULER_TYPE_ROOT		= 4, /* Root / integrated scheduler */
	HV_SCHEDULER_TYPE_MAX
};

/* HV_STATS_AREA_TYPE */
enum hv_stats_area_type {
	HV_STATS_AREA_SELF = 0,
	HV_STATS_AREA_PARENT = 1,
	HV_STATS_AREA_INTERNAL = 2,
	HV_STATS_AREA_COUNT
};

enum hv_stats_object_type {
	HV_STATS_OBJECT_HYPERVISOR		= 0x00000001,
	HV_STATS_OBJECT_LOGICAL_PROCESSOR	= 0x00000002,
	HV_STATS_OBJECT_PARTITION		= 0x00010001,
	HV_STATS_OBJECT_VP			= 0x00010002
};

union hv_stats_object_identity {
	/* hv_stats_hypervisor */
	struct {
		u8 reserved[15];
		u8 stats_area_type;
	} __packed hv;

	/* hv_stats_logical_processor */
	struct {
		u32 lp_index;
		u8 reserved[11];
		u8 stats_area_type;
	} __packed lp;

	/* hv_stats_partition */
	struct {
		u64 partition_id;
		u8  reserved[7];
		u8  stats_area_type;
	} __packed partition;

	/* hv_stats_vp */
	struct {
		u64 partition_id;
		u32 vp_index;
		u16 flags;
		u8  reserved;
		u8  stats_area_type;
	} __packed vp;
};

enum hv_partition_property_code {
	/* Privilege properties */
	HV_PARTITION_PROPERTY_PRIVILEGE_FLAGS			= 0x00010000,
	HV_PARTITION_PROPERTY_SYNTHETIC_PROC_FEATURES		= 0x00010001,

	/* Resource properties */
	HV_PARTITION_PROPERTY_GPA_PAGE_ACCESS_TRACKING		= 0x00050005,
	HV_PARTITION_PROPERTY_UNIMPLEMENTED_MSR_ACTION		= 0x00050017,

	/* Compatibility properties */
	HV_PARTITION_PROPERTY_PROCESSOR_XSAVE_FEATURES		= 0x00060002,
	HV_PARTITION_PROPERTY_XSAVE_STATES                      = 0x00060007,
	HV_PARTITION_PROPERTY_MAX_XSAVE_DATA_SIZE		= 0x00060008,
	HV_PARTITION_PROPERTY_PROCESSOR_CLOCK_FREQUENCY		= 0x00060009,

	/* Extended properties with larger property values */
	HV_PARTITION_PROPERTY_VMM_CAPABILITIES			= 0x00090007,
};

#define HV_PARTITION_VMM_CAPABILITIES_BANK_COUNT		1
#define HV_PARTITION_VMM_CAPABILITIES_RESERVED_BITFIELD_COUNT	59

struct hv_partition_property_vmm_capabilities {
	u16 bank_count;
	u16 reserved[3];
	union {
		u64 as_uint64[HV_PARTITION_VMM_CAPABILITIES_BANK_COUNT];
		struct {
			u64 map_gpa_preserve_adjustable: 1;
			u64 vmm_can_provide_overlay_gpfn: 1;
			u64 vp_affinity_property: 1;
#if IS_ENABLED(CONFIG_ARM64)
			u64 vmm_can_provide_gic_overlay_locations: 1;
#else
			u64 reservedbit3: 1;
#endif
			u64 assignable_synthetic_proc_features: 1;
			u64 reserved0: HV_PARTITION_VMM_CAPABILITIES_RESERVED_BITFIELD_COUNT;
		} __packed;
	};
} __packed;

enum hv_snp_status {
	HV_SNP_STATUS_NONE = 0,
	HV_SNP_STATUS_AVAILABLE = 1,
	HV_SNP_STATUS_INCOMPATIBLE = 2,
	HV_SNP_STATUS_PSP_UNAVAILABLE = 3,
	HV_SNP_STATUS_PSP_INIT_FAILED = 4,
	HV_SNP_STATUS_PSP_BAD_FW_VERSION = 5,
	HV_SNP_STATUS_BAD_CONFIGURATION = 6,
	HV_SNP_STATUS_PSP_FW_UPDATE_IN_PROGRESS = 7,
	HV_SNP_STATUS_PSP_RB_INIT_FAILED = 8,
	HV_SNP_STATUS_PSP_PLATFORM_STATUS_FAILED = 9,
	HV_SNP_STATUS_PSP_INIT_LATE_FAILED = 10,
};

enum hv_system_property {
	/* Add more values when needed */
	HV_SYSTEM_PROPERTY_SCHEDULER_TYPE = 15,
	HV_DYNAMIC_PROCESSOR_FEATURE_PROPERTY = 21,
	HV_SYSTEM_PROPERTY_CRASHDUMPAREA = 47,
};

#define HV_PFN_RANGE_PGBITS 24  /* HV_SPA_PAGE_RANGE_ADDITIONAL_PAGES_BITS */
union hv_pfn_range {            /* HV_SPA_PAGE_RANGE */
	u64 as_uint64;
	struct {
		/* 39:0: base pfn.  63:40: additional pages */
		u64 base_pfn : 64 - HV_PFN_RANGE_PGBITS;
		u64 add_pfns : HV_PFN_RANGE_PGBITS;
	} __packed;
};

enum hv_dynamic_processor_feature_property {
	/* Add more values when needed */
	HV_X64_DYNAMIC_PROCESSOR_FEATURE_MAX_ENCRYPTED_PARTITIONS = 13,
	HV_X64_DYNAMIC_PROCESSOR_FEATURE_SNP_STATUS = 16,
};

struct hv_input_get_system_property {
	u32 property_id; /* enum hv_system_property */
	union {
		u32 as_uint32;
#if IS_ENABLED(CONFIG_X86)
		/* enum hv_dynamic_processor_feature_property */
		u32 hv_processor_feature;
#endif
		/* More fields to be filled in when needed */
	};
} __packed;

struct hv_output_get_system_property {
	union {
		u32 scheduler_type; /* enum hv_scheduler_type */
#if IS_ENABLED(CONFIG_X86)
		u64 hv_processor_feature_value;
#endif
		union hv_pfn_range hv_cda_info; /* CrashdumpAreaAddress */
		u64 hv_tramp_pa;                /* CrashdumpTrampolineAddress */
	};
} __packed;

struct hv_input_map_stats_page {
	u32 type; /* enum hv_stats_object_type */
	u32 padding;
	union hv_stats_object_identity identity;
} __packed;

struct hv_input_map_stats_page2 {
	u32 type; /* enum hv_stats_object_type */
	u32 padding;
	union hv_stats_object_identity identity;
	u64 map_location;
} __packed;

struct hv_output_map_stats_page {
	u64 map_location;
} __packed;

struct hv_input_unmap_stats_page {
	u32 type; /* enum hv_stats_object_type */
	u32 padding;
	union hv_stats_object_identity identity;
} __packed;

struct hv_proximity_domain_flags {
	u32 proximity_preferred : 1;
	u32 reserved : 30;
	u32 proximity_info_valid : 1;
} __packed;

struct hv_proximity_domain_info {
	u32 domain_id;
	struct hv_proximity_domain_flags flags;
} __packed;

/* HvDepositMemory hypercall */
struct hv_deposit_memory {	/* HV_INPUT_DEPOSIT_MEMORY */
	u64 partition_id;
	u64 gpa_page_list[];
} __packed;

struct hv_input_withdraw_memory {
	u64 partition_id;
	struct hv_proximity_domain_info proximity_domain_info;
} __packed;

struct hv_output_withdraw_memory {
	DECLARE_FLEX_ARRAY(u64, gpa_page_list);
} __packed;

/* HV Map GPA (Guest Physical Address) Flags */
#define HV_MAP_GPA_PERMISSIONS_NONE	       0x0
#define HV_MAP_GPA_READABLE		       0x1
#define HV_MAP_GPA_WRITABLE		       0x2
#define HV_MAP_GPA_KERNEL_EXECUTABLE	       0x4
#define HV_MAP_GPA_USER_EXECUTABLE	       0x8
#define HV_MAP_GPA_EXECUTABLE		       0xC
#define HV_MAP_GPA_PERMISSIONS_MASK	       0xF
#define HV_MAP_GPA_ADJUSTABLE		    0x8000
#define HV_MAP_GPA_NO_ACCESS		   0x10000
#define HV_MAP_GPA_NOT_CACHED		  0x200000
#define HV_MAP_GPA_LARGE_PAGE		0x80000000

struct hv_input_map_gpa_pages {
	u64 target_partition_id;
	u64 target_gpa_base;
	u32 map_flags;
	u32 padding;
	u64 source_gpa_page_list[];
} __packed;

union hv_gpa_page_access_state_flags {
	struct {
		u64 clear_accessed : 1;
		u64 set_accessed : 1;
		u64 clear_dirty : 1;
		u64 set_dirty : 1;
		u64 reserved : 60;
	} __packed;
	u64 as_uint64;
};

struct hv_input_get_gpa_pages_access_state {
	u64  partition_id;
	union hv_gpa_page_access_state_flags flags;
	u64 hv_gpa_page_number;
} __packed;

union hv_gpa_page_access_state {
	struct {
		u8 accessed : 1;
		u8 dirty : 1;
		u8 reserved: 6;
	};
	u8 as_uint8;
} __packed;

enum hv_crashdump_action {
	HV_CRASHDUMP_NONE = 0,
	HV_CRASHDUMP_SUSPEND_ALL_VPS,
	HV_CRASHDUMP_PREPARE_FOR_STATE_SAVE,
	HV_CRASHDUMP_STATE_SAVED,
	HV_CRASHDUMP_ENTRY,
};

struct hv_partition_event_root_crashdump_input {
	u32 crashdump_action; /* enum hv_crashdump_action */
} __packed;

struct hv_input_disable_hyp_ex {   /* HV_X64_INPUT_DISABLE_HYPERVISOR_EX */
	u64 rip;
	u64 arg;
} __packed;

struct hv_crashdump_area {	   /* HV_CRASHDUMP_AREA */
	u32 version;
	union {
		u32 flags_as_uint32;
		struct {
			u32 cda_valid : 1;
			u32 cda_unused : 31;
		} __packed;
	};
	/* more unused fields */
} __packed;

union hv_partition_event_input {
	struct hv_partition_event_root_crashdump_input crashdump_input;
};

enum hv_partition_event {
	HV_PARTITION_EVENT_ROOT_CRASHDUMP = 2,
};

struct hv_input_notify_partition_event {
	u32 event;      /* enum hv_partition_event */
	union hv_partition_event_input input;
} __packed;

struct hv_lp_startup_status {
	u64 hv_status;
	u64 substatus1;
	u64 substatus2;
	u64 substatus3;
	u64 substatus4;
	u64 substatus5;
	u64 substatus6;
} __packed;

struct hv_input_add_logical_processor {
	u32 lp_index;
	u32 apic_id;
	struct hv_proximity_domain_info proximity_domain_info;
} __packed;

struct hv_output_add_logical_processor {
	struct hv_lp_startup_status startup_status;
} __packed;

enum {	/* HV_SUBNODE_TYPE */
	HV_SUBNODE_ANY		= 0,
	HV_SUBNODE_SOCKET,
	HV_SUBNODE_CLUSTER,
	HV_SUBNODE_L3,
	HV_SUBNODE_COUNT,
	HV_SUBNODE_INVALID	= -1
};

struct hv_create_vp {	/* HV_INPUT_CREATE_VP */
	u64 partition_id;
	u32 vp_index;
	u8 padding[3];
	u8 subnode_type;
	u64 subnode_id;
	struct hv_proximity_domain_info proximity_domain_info;
	u64 flags;
} __packed;

/* HV_INTERRUPT_TRIGGER_MODE */
enum hv_interrupt_trigger_mode {
	HV_INTERRUPT_TRIGGER_MODE_EDGE	= 0,
	HV_INTERRUPT_TRIGGER_MODE_LEVEL	= 1,
};

/* HV_DEVICE_INTERRUPT_DESCRIPTOR */
struct hv_device_interrupt_descriptor {
	u32 interrupt_type;
	u32 trigger_mode;
	u32 vector_count;
	u32 reserved;
	struct hv_device_interrupt_target target;
} __packed;

/* HV_INPUT_MAP_DEVICE_INTERRUPT */
struct hv_input_map_device_interrupt {
	u64 partition_id;
	u64 device_id;
	u32 flags;
	u32 base_irt_idx;
	struct hv_interrupt_entry logical_interrupt_entry;
	struct hv_device_interrupt_descriptor interrupt_descriptor;
} __packed;

/* HV_OUTPUT_MAP_DEVICE_INTERRUPT */
struct hv_output_map_device_interrupt {
	struct hv_interrupt_entry interrupt_entry;
	u64 ext_status_deprecated[5];
} __packed;

/* HV_INPUT_UNMAP_DEVICE_INTERRUPT */
struct hv_input_unmap_device_interrupt {
	u64 partition_id;
	u64 device_id;
	struct hv_interrupt_entry interrupt_entry;
	u32 flags;
} __packed;

#define HV_SOURCE_SHADOW_NONE		    0x0
#define HV_SOURCE_SHADOW_BRIDGE_BUS_RANGE   0x1

struct hv_send_ipi_ex { /* HV_INPUT_SEND_SYNTHETIC_CLUSTER_IPI_EX */
	u32 vector;
	u32 reserved;
	struct hv_vpset vp_set;
} __packed;

typedef u16 hv_pci_rid;		/* HV_PCI_RID */
typedef u16 hv_pci_segment;	/* HV_PCI_SEGMENT */
typedef u64 hv_logical_device_id;
union hv_pci_bdf {	/* HV_PCI_BDF */
	u16 as_uint16;

	struct {
		u8 function : 3;
		u8 device : 5;
		u8 bus;
	};
} __packed;

union hv_pci_bus_range {
	u16 as_uint16;

	struct {
		u8 subordinate_bus;
		u8 secondary_bus;
	};
} __packed;

enum hv_device_type {		/* HV_DEVICE_TYPE */
	HV_DEVICE_TYPE_LOGICAL	= 0,
	HV_DEVICE_TYPE_PCI	= 1,
	HV_DEVICE_TYPE_IOAPIC	= 2,
	HV_DEVICE_TYPE_ACPI	= 3,
};

union hv_device_id {		/* HV_DEVICE_ID */
	u64 as_uint64;

	struct {
		u64 reserved0 : 62;
		u64 device_type : 2;
	};

	/* HV_DEVICE_TYPE_LOGICAL */
	struct {
		u64 id : 62;
		u64 device_type : 2;
	} logical;

	/* HV_DEVICE_TYPE_PCI */
	struct {
		union {
			hv_pci_rid rid;
			union hv_pci_bdf bdf;
		};

		hv_pci_segment segment;
		union hv_pci_bus_range shadow_bus_range;

		u16 phantom_function_bits : 2;
		u16 source_shadow : 1;

		u16 rsvdz0 : 11;
		u16 device_type : 2;
	} pci;

	/* HV_DEVICE_TYPE_IOAPIC */
	struct {
		u8 ioapic_id;
		u8 rsvdz0;
		u16 rsvdz1;
		u16 rsvdz2;

		u16 rsvdz3 : 14;
		u16 device_type : 2;
	} ioapic;

	/* HV_DEVICE_TYPE_ACPI */
	struct {
		u32 input_mapping_base;
		u32 input_mapping_count : 30;
		u32 device_type : 2;
	} acpi;
} __packed;

#endif /* _HV_HVHDK_MINI_H */
