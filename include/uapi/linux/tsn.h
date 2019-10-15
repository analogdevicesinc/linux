/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) WITH Linux-syscall-note */
/* Copyright 2017-2023 NXP */

#ifndef __UAPI_GENL_TSN_H
#define __UAPI_GENL_TSN_H

#define	TSN_GENL_NAME		"TSN_GEN_CTRL"
#define	TSN_GENL_VERSION	0x1

#define MAX_USER_SIZE 0
#define MAX_ATTR_SIZE 3072
#define MAX_TOTAL_MSG_SIZE  (MAX_USER_SIZE + MAX_ATTR_SIZE)
#define MAX_ENTRY_SIZE 2048
#define MAX_ENTRY_NUMBER 128
#define MAX_IFNAME_COUNT 64

#define TSN_MULTICAST_GROUP_QBV	"qbv"
#define TSN_MULTICAST_GROUP_QCI	"qci"

/* multicast groups */
enum tsn_multicast_groups {
	TSN_MCGRP_QBV,
	TSN_MCGRP_QCI,
	TSN_MCGRP_MAX
};

enum tsn_capability {
	TSN_CAP_QBV = 0x1,
	TSN_CAP_QCI = 0x2,
	TSN_CAP_QBU = 0x4,
	TSN_CAP_CBS = 0x8, /* Credit-based Shapter Qav */
	TSN_CAP_CB  = 0x10, /* 8021CB redundancy and replication */
	TSN_CAP_TBS = 0x20, /* Time Based schedule */
	TSN_CAP_CTH = 0x40, /* cut through */
};

/*
 * Commands sent from userspace
 * Not versioned. New commands should only be inserted at the enum's end
 * prior to __TSN_CMD_MAX
 */

enum {
	TSN_CMD_UNSPEC = 0,	/* Reserved */
	TSN_CMD_QBV_SET,
	TSN_CMD_QBV_GET,
	TSN_CMD_QBV_GET_STATUS,
	TSN_CMD_CB_STREAMID_SET,
	TSN_CMD_CB_STREAMID_GET,
	TSN_CMD_CB_STREAMID_GET_COUNTS,
	TSN_CMD_QCI_CAP_GET, /* Qci capability get length capability get */
	TSN_CMD_QCI_SFI_SET,
	TSN_CMD_QCI_SFI_GET,
	TSN_CMD_QCI_SFI_GET_COUNTS,
	TSN_CMD_QCI_SGI_SET,
	TSN_CMD_QCI_SGI_GET,
	TSN_CMD_QCI_SGI_GET_STATUS,
	TSN_CMD_QCI_FMI_SET,
	TSN_CMD_QCI_FMI_GET,
	TSN_CMD_CBS_SET,
	TSN_CMD_CBS_GET,
	TSN_CMD_QBU_SET,
	TSN_CMD_QBU_GET_STATUS,
	TSN_CMD_QAV_SET_CBS,
	TSN_CMD_QAV_GET_CBS,
	TSN_CMD_TSD_SET,
	TSN_CMD_TSD_GET,
	TSN_CMD_CT_SET,
	TSN_CMD_CBGEN_SET,
	TSN_CMD_CBREC_SET,
	TSN_CMD_CBSTAT_GET,
	TSN_CMD_PCPMAP_SET,
	TSN_CMD_DSCP_SET,
	TSN_CMD_ECHO,			/* user->kernel request/get-response */
	TSN_CMD_REPLY,			/* kernel->user event */
	TSN_CMD_CAP_GET,
	__TSN_CMD_MAX,
};
#define TSN_CMD_MAX (__TSN_CMD_MAX - 1)


enum {
	TSN_CMD_ATTR_UNSPEC = 0,
	TSN_CMD_ATTR_MESG,		/* demo message  */
	TSN_CMD_ATTR_DATA,		/* demo data */
	TSN_ATTR_IFNAME,
	TSN_ATTR_PORT_NUMBER,
	TSN_ATTR_QBV,
	TSN_ATTR_STREAM_IDENTIFY,	/* stream identify */
	TSN_ATTR_QCI_SP,		/* psfp port capbility parameters */
	TSN_ATTR_QCI_SFI,		/* psfp stream filter instance */
	TSN_ATTR_QCI_SGI,		/* psfp stream gate instance */
	TSN_ATTR_QCI_FMI,		/* psfp flow meter instance */
	TSN_ATTR_CBS,			/* credit-based shaper */
	TSN_ATTR_TSD,			/* Time Specific Departure */
	TSN_ATTR_QBU,			/* preemption */
	TSN_ATTR_CT,			/* cut through */
	TSN_ATTR_CBGEN,			/* 802.1CB sequence generate */
	TSN_ATTR_CBREC,			/* 802.1CB sequence recover */
	TSN_ATTR_CBSTAT,		/* 802.1CB status */
	TSN_ATTR_PCPMAP,
	TSN_ATTR_DSCP,
	TSN_ATTR_CAP,			/* TSN capbility */
	__TSN_CMD_ATTR_MAX,
};
#define TSN_CMD_ATTR_MAX (__TSN_CMD_ATTR_MAX - 1)

enum {
	TSN_CAP_ATTR_UNSPEC,
	TSN_CAP_ATTR_QBV,
	TSN_CAP_ATTR_QCI,
	TSN_CAP_ATTR_QBU,
	TSN_CAP_ATTR_CBS,
	TSN_CAP_ATTR_CB,
	TSN_CAP_ATTR_TBS,
	TSN_CAP_ATTR_CTH,
	__TSN_CAP_ATTR_MAX,
	TSN_CAP_ATTR_MAX = __TSN_CAP_ATTR_MAX - 1,
};

enum {
	TSN_QBU_ATTR_UNSPEC,
	TSN_QBU_ATTR_ADMIN_STATE,
	TSN_QBU_ATTR_HOLD_ADVANCE,
	TSN_QBU_ATTR_RELEASE_ADVANCE,
	TSN_QBU_ATTR_ACTIVE,
	TSN_QBU_ATTR_HOLD_REQUEST,
	__TSN_QBU_ATTR_MAX,
	TSN_QBU_ATTR_MAX = __TSN_QBU_ATTR_MAX - 1,
};

enum {
	TSN_CBS_ATTR_UNSPEC,
	TSN_CBS_ATTR_TC_INDEX,
	TSN_CBS_ATTR_BW,
	__TSN_CBS_ATTR_MAX,
	TSN_CBS_ATTR_MAX = __TSN_CBS_ATTR_MAX - 1,
};

enum {
	TSN_TSD_ATTR_UNSPEC,
	TSN_TSD_ATTR_DISABLE,
	TSN_TSD_ATTR_ENABLE,
	TSN_TSD_ATTR_PERIOD,
	TSN_TSD_ATTR_MAX_FRM_NUM,
	TSN_TSD_ATTR_CYCLE_NUM,
	TSN_TSD_ATTR_LOSS_STEPS,
	TSN_TSD_ATTR_SYN_IMME,
	__TSN_TSD_ATTR_MAX,
	TSN_TSD_ATTR_MAX = __TSN_TSD_ATTR_MAX - 1,
};

enum {
	TSN_STREAMID_ATTR_UNSPEC,
	TSN_STREAMID_ATTR_INDEX,
	TSN_STREAMID_ATTR_ENABLE,
	TSN_STREAMID_ATTR_DISABLE,
	TSN_STREAMID_ATTR_STREAM_HANDLE,
	TSN_STREAMID_ATTR_IFOP,
	TSN_STREAMID_ATTR_OFOP,
	TSN_STREAMID_ATTR_IFIP,
	TSN_STREAMID_ATTR_OFIP,
	TSN_STREAMID_ATTR_TYPE,
	TSN_STREAMID_ATTR_NDMAC,
	TSN_STREAMID_ATTR_NTAGGED,
	TSN_STREAMID_ATTR_NVID,
	TSN_STREAMID_ATTR_SMAC,
	TSN_STREAMID_ATTR_STAGGED,
	TSN_STREAMID_ATTR_SVID,
	TSN_STREAMID_ATTR_COUNTERS_PSI,
	TSN_STREAMID_ATTR_COUNTERS_PSO,
	TSN_STREAMID_ATTR_COUNTERS_PSPPI,
	TSN_STREAMID_ATTR_COUNTERS_PSPPO,
	__TSN_STREAMID_ATTR_MAX,
	TSN_STREAMID_ATTR_MAX = __TSN_STREAMID_ATTR_MAX - 1,
};

enum {
	TSN_QCI_STREAM_ATTR_UNSPEC = 0,
	TSN_QCI_STREAM_ATTR_MAX_SFI,
	TSN_QCI_STREAM_ATTR_MAX_SGI,
	TSN_QCI_STREAM_ATTR_MAX_FMI,
	TSN_QCI_STREAM_ATTR_SLM,
	__TSN_QCI_STREAM_ATTR_MAX,
	TSN_QCI_STREAM_ATTR_MAX = __TSN_QCI_STREAM_ATTR_MAX - 1,
};

enum {
	TSN_QCI_SFI_ATTR_UNSPEC = 0,
	TSN_QCI_SFI_ATTR_INDEX,
	TSN_QCI_SFI_ATTR_ENABLE,
	TSN_QCI_SFI_ATTR_DISABLE,
	TSN_QCI_SFI_ATTR_STREAM_HANDLE,
	TSN_QCI_SFI_ATTR_PRIO_SPEC,
	TSN_QCI_SFI_ATTR_GATE_ID,
	TSN_QCI_SFI_ATTR_FILTER_TYPE,
	TSN_QCI_SFI_ATTR_FLOW_ID,
	TSN_QCI_SFI_ATTR_MAXSDU,
	TSN_QCI_SFI_ATTR_COUNTERS,
	TSN_QCI_SFI_ATTR_OVERSIZE_ENABLE,
	TSN_QCI_SFI_ATTR_OVERSIZE,
	__TSN_QCI_SFI_ATTR_MAX,
	TSN_QCI_SFI_ATTR_MAX = __TSN_QCI_SFI_ATTR_MAX - 1,
};

enum {
	TSN_QCI_SFI_ATTR_COUNTERS_UNSPEC = 0,
	TSN_QCI_SFI_ATTR_MATCH,
	TSN_QCI_SFI_ATTR_PASS,
	TSN_QCI_SFI_ATTR_DROP,
	TSN_QCI_SFI_ATTR_SDU_DROP,
	TSN_QCI_SFI_ATTR_SDU_PASS,
	TSN_QCI_SFI_ATTR_RED,
	__TSN_QCI_SFI_ATTR_COUNT_MAX,
	TSN_QCI_SFI_ATTR_COUNT_MAX = __TSN_QCI_SFI_ATTR_COUNT_MAX - 1,
};

enum {
	TSN_QCI_SGI_ATTR_UNSPEC = 0,
	TSN_QCI_SGI_ATTR_INDEX,
	TSN_QCI_SGI_ATTR_ENABLE,
	TSN_QCI_SGI_ATTR_DISABLE,
	TSN_QCI_SGI_ATTR_CONFCHANGE,
	TSN_QCI_SGI_ATTR_IRXEN,		/* Invalid rx enable*/
	TSN_QCI_SGI_ATTR_IRX,
	TSN_QCI_SGI_ATTR_OEXEN,		/* Octet exceed enable */
	TSN_QCI_SGI_ATTR_OEX,
	TSN_QCI_SGI_ATTR_ADMINENTRY,
	TSN_QCI_SGI_ATTR_OPERENTRY,
	TSN_QCI_SGI_ATTR_CCTIME,	/* config change time */
	TSN_QCI_SGI_ATTR_TICKG,
	TSN_QCI_SGI_ATTR_CUTIME,
	TSN_QCI_SGI_ATTR_CPENDING,
	TSN_QCI_SGI_ATTR_CCERROR,
	__TSN_QCI_SGI_ATTR_MAX,
	TSN_QCI_SGI_ATTR_MAX = __TSN_QCI_SGI_ATTR_MAX - 1,
};

enum {
	TSN_SGI_ATTR_CTRL_UNSPEC = 0,
	TSN_SGI_ATTR_CTRL_INITSTATE,
	TSN_SGI_ATTR_CTRL_LEN,
	TSN_SGI_ATTR_CTRL_CYTIME,
	TSN_SGI_ATTR_CTRL_CYTIMEEX,
	TSN_SGI_ATTR_CTRL_BTIME,
	TSN_SGI_ATTR_CTRL_INITIPV,
	TSN_SGI_ATTR_CTRL_GCLENTRY,
	__TSN_SGI_ATTR_CTRL_MAX,
	TSN_SGI_ATTR_CTRL_MAX = __TSN_SGI_ATTR_CTRL_MAX - 1,
};

enum {
	TSN_SGI_ATTR_GCL_UNSPEC = 0,
	TSN_SGI_ATTR_GCL_GATESTATE,
	TSN_SGI_ATTR_GCL_IPV,
	TSN_SGI_ATTR_GCL_INTERVAL,
	TSN_SGI_ATTR_GCL_OCTMAX,
	__TSN_SGI_ATTR_GCL_MAX,
	TSN_SGI_ATTR_GCL_MAX = __TSN_SGI_ATTR_GCL_MAX - 1,
};

enum {
	TSN_QCI_FMI_ATTR_UNSPEC = 0,
	TSN_QCI_FMI_ATTR_INDEX,
	TSN_QCI_FMI_ATTR_ENABLE,
	TSN_QCI_FMI_ATTR_DISABLE,
	TSN_QCI_FMI_ATTR_CIR,
	TSN_QCI_FMI_ATTR_CBS,
	TSN_QCI_FMI_ATTR_EIR,
	TSN_QCI_FMI_ATTR_EBS,
	TSN_QCI_FMI_ATTR_CF,
	TSN_QCI_FMI_ATTR_CM,
	TSN_QCI_FMI_ATTR_DROPYL,
	TSN_QCI_FMI_ATTR_MAREDEN,
	TSN_QCI_FMI_ATTR_MARED,
	TSN_QCI_FMI_ATTR_COUNTERS,
	__TSN_QCI_FMI_ATTR_MAX,
	TSN_QCI_FMI_ATTR_MAX = __TSN_QCI_FMI_ATTR_MAX - 1,
};

enum {
	TSN_QBV_ATTR_UNSPEC,
	TSN_QBV_ATTR_ENABLE,
	TSN_QBV_ATTR_DISABLE,
	TSN_QBV_ATTR_CONFIGCHANGE,
	TSN_QBV_ATTR_CONFIGCHANGETIME,
	TSN_QBV_ATTR_MAXSDU,
	TSN_QBV_ATTR_GRANULARITY,
	TSN_QBV_ATTR_CURRENTTIME,
	TSN_QBV_ATTR_CONFIGPENDING,
	TSN_QBV_ATTR_CONFIGCHANGEERROR,
	TSN_QBV_ATTR_ADMINENTRY,
	TSN_QBV_ATTR_OPERENTRY,
	TSN_QBV_ATTR_LISTMAX,
	__TSN_QBV_ATTR_MAX,
	TSN_QBV_ATTR_MAX = __TSN_QBV_ATTR_MAX - 1,
};

enum {
	TSN_QBV_ATTR_CTRL_UNSPEC,
	TSN_QBV_ATTR_CTRL_LISTCOUNT,
	TSN_QBV_ATTR_CTRL_GATESTATE,
	TSN_QBV_ATTR_CTRL_CYCLETIME,
	TSN_QBV_ATTR_CTRL_CYCLETIMEEXT,
	TSN_QBV_ATTR_CTRL_BASETIME,
	TSN_QBV_ATTR_CTRL_LISTENTRY,
	__TSN_QBV_ATTR_CTRL_MAX,
	TSN_QBV_ATTR_CTRL_MAX = __TSN_QBV_ATTR_CTRL_MAX - 1,
};

enum {
	TSN_QBV_ATTR_ENTRY_UNSPEC,
	TSN_QBV_ATTR_ENTRY_ID,
	TSN_QBV_ATTR_ENTRY_GC,
	TSN_QBV_ATTR_ENTRY_TM,
	__TSN_QBV_ATTR_ENTRY_MAX,
	TSN_QBV_ATTR_ENTRY_MAX = __TSN_QBV_ATTR_ENTRY_MAX - 1,
};

enum {
	TSN_CT_ATTR_UNSPEC,
	TSN_CT_ATTR_QUEUE_STATE,
	__TSN_CT_ATTR_MAX,
	TSN_CT_ATTR_MAX = __TSN_CT_ATTR_MAX - 1,
};

enum {
	TSN_CBGEN_ATTR_UNSPEC,
	TSN_CBGEN_ATTR_INDEX,
	TSN_CBGEN_ATTR_PORT_MASK,
	TSN_CBGEN_ATTR_SPLIT_MASK,
	TSN_CBGEN_ATTR_SEQ_LEN,
	TSN_CBGEN_ATTR_SEQ_NUM,
	__TSN_CBGEN_ATTR_MAX,
	TSN_CBGEN_ATTR_MAX = __TSN_CBGEN_ATTR_MAX - 1,
};

enum {
	TSN_CBREC_ATTR_UNSPEC,
	TSN_CBREC_ATTR_INDEX,
	TSN_CBREC_ATTR_SEQ_LEN,
	TSN_CBREC_ATTR_HIS_LEN,
	TSN_CBREC_ATTR_TAG_POP_EN,
	__TSN_CBREC_ATTR_MAX,
	TSN_CBREC_ATTR_MAX = __TSN_CBREC_ATTR_MAX - 1,
};

enum {
	TSN_CBSTAT_ATTR_UNSPEC,
	TSN_CBSTAT_ATTR_INDEX,
	TSN_CBSTAT_ATTR_GEN_REC,
	TSN_CBSTAT_ATTR_ERR,
	TSN_CBSTAT_ATTR_SEQ_NUM,
	TSN_CBSTAT_ATTR_SEQ_LEN,
	TSN_CBSTAT_ATTR_SPLIT_MASK,
	TSN_CBSTAT_ATTR_PORT_MASK,
	TSN_CBSTAT_ATTR_HIS_LEN,
	TSN_CBSTAT_ATTR_SEQ_HIS,
	__TSN_CBSTAT_ATTR_MAX,
	TSN_CBSTAT_ATTR_MAX = __TSN_CBSTAT_ATTR_MAX - 1,
};

enum {
	TSN_DSCP_ATTR_UNSPEC,
	TSN_DSCP_ATTR_DISABLE,
	TSN_DSCP_ATTR_INDEX,
	TSN_DSCP_ATTR_COS,
	TSN_DSCP_ATTR_DPL,
	__TSN_DSCP_ATTR_MAX,
	TSN_DSCP_ATTR_MAX = __TSN_DSCP_ATTR_MAX - 1,
};

enum {
	TSN_PCP_ATTR_UNSPEC,
	TSN_PCP_ATTR_PCP,
	TSN_PCP_ATTR_DEI,
	TSN_PCP_ATTR_COS,
	TSN_PCP_ATTR_DPL,
	__TSN_PCP_ATTR_MAX,
};
#define TSN_PCP_ATTR_MAX (__TSN_PCP_ATTR_MAX - 1)

#define ptptime_t __u64

#define MAX_QUEUE_CNT 8

struct tsn_preempt_status {
	/* The value of admin_state shows a 8-bits vector value for showing
	 * the framePreemptionAdminStatus parameter and PreemptionPriority
	 * for the traffic class. Bit-7 is the highest priority traffic class
	 * and the bit-0 is the lowest priority traffic class.
	 * The bit is express (0) and is preemptible (1).
	 */
	__u8 admin_state;
	/* The value of the holdAdvance parameter for the port in nanoseconds.
	 * There is no default value; the holdAdvance is a property of the
	 * underlying MAC." This parameter corresponds to the holdAdvance
	 * parameter in 802.1Qbu.
	 */
	__u32 hold_advance;

	/* The value of the releaseAdvance parameter for the port in
	 * nanoseconds.  There is no default value; the releaseAdvance is a
	 * property of the underlying MAC." This parameter corresponds to the
	 * releaseAdvance parameter in 802.1Qbu.
	 */
	__u32 release_advance;

	/* The value is active (TRUE) when preemption is operationally active
	 * for the port, and idle (FALSE) otherwise.  This parameter corresponds
	 * to the preemptionActive parameter in 802.1Qbu.
	 */
	__u8 preemption_active;

	/* The value is hold (1) when the sequence of gate operations for
	 * the port has executed a Set-And-Hold-MAC operation, and release
	 * (2) when the sequence of gate operations has executed a
	 * Set-And-Release-MAC operation. The value of this object is release
	 * (FALSE) on system initialization.  This parameter corresponds to the
	 * holdRequest parameter in 802.1Qbu.
	 */
	__u8 hold_request;
};

enum tsn_tx_mode  {
	TX_MODE_STRICT,
	TX_MODE_CBS,
	TX_MODE_ETS,
	TX_MODE_VENDOR_DEFINE = 255,
};

#define QUEUE_TX_MASK ((1 << TX_MODE_STRICT) | (1 << TX_MODE_CBS) \
			| (1 << TX_MODE_ETS) | (1 << TX_MODE_VENDOR_DEFINE))

struct cbs_status {
	__u8 delta_bw; /* percentage, 0~100 */
	__u32 idleslope;
	__s32 sendslope;
	__u32 maxframesize;
	__u32 hicredit;
	__s32 locredit;
	__u32 maxninference;
};

struct tx_queue {
	/* tx_queue_capbility shows the queue's capability mask.
	 * refer the enum tsn_tx_mode
	 */
	__u8 capability;

	/* tx_queue_mode is current queue working mode */
	__u8 mode;

	/* prio is showing the queue priority */
	__u8 prio;

	/* mstat shows the status data of cbs or priority */
	union {
		struct cbs_status cbs;
	};
};

struct port_status {
	/* txqueue_cnt shows how many queues in this port */
	__u8 queue_cnt;

	/* max_rate(Mbit/s) is the port transmit rate current port is setting */
	__u32 max_rate;

	/* tsn_capability mask the tsn capability */
	__u32 tsn_capability;
};

enum tsn_cb_streamid_type {
	STREAMID_RESERVED = 0,
	/* Null Stream identification */
	STREAMID_NULL,
	/* Source MAC and VLAN Stream identification */
	STREAMID_SMAC_VLAN,
	/* Active Destination MAC and VLAN stream identification */
	STREAMID_DMAC_VLAN,
	/* IP stream identification */
	STREAMID_IP,
};

/* When instantiating an instance of the Null Stream identification function
 * 8021CB(6.4) for a particular input Stream, the managed objects in the
 * following subsections serve as the tsnStreamIdParameters managed object
 * 8021CB claus(9.1.1.7).
 */
struct tsn_cb_null_streamid {
	/* tsnCpeNullDownDestMac. Specifies the destination_address that
	 * identifies a packet in an Enhanced Internal Sublayer Service (EISS)
	 * indication primitive, to the Null Stream identification function.
	 */
	__u64 dmac;

	/* tsnCpeNullDownTagged. It can take the following values:
	 * 1 tagged: A frame must have a VLAN tag to be recognized as belonging
	 * to the Stream.
	 * 2 priority: A frame must be untagged, or have a VLAN tag with a VLAN
	 * ID = 0 to be recognized as belonging to the Stream.
	 * 3 all: A frame is recognized as belonging to the Stream whether
	 * tagged or not.
	 */
	__u8 tagged;

	/* tsnCpeNullDownVlan. Specifies the vlan_identifier parameter that
	 * identifies a packet in an EISS indication primitive to the Null
	 * Stream identification function. A value of 0 indicates that the vlan
	 * _identifier parameter is ignored on EISS indication primitives.
	 */
	__u16 vid;
};

struct tsn_cb_source_streamid {
	__u64 smac;
	__u8 tagged;
	__u16 vid;
};

struct tsn_cb_dest_streamid {
	__u64 down_dmac;
	__u8 down_tagged;
	__u16 down_vid;
	__u8 down_prio;
	__u64 up_dmac;
	__u8 up_tagged;
	__u16 up_vid;
	__u8 up_prio;
};

struct tsn_cb_ip_streamid {
	__u64 dmac;
	__u8 tagged;
	__u16 vid;
	__u64 siph;
	__u64 sipl;
	__u64 diph;
	__u64 dipl;
	__u8 dscp;
	__u8 npt;
	__u16 sport;
	__u16 dport;
};

/* 802.1CB stream identify table clause 9.1 */
struct tsn_cb_streamid {
	/* The objects in a given entry of the Stream identity table are used
	 * to control packets whose stream_handle subparameter is equal to the
	 * entry tsnStreamIdHandle object.
	 */
	__s32 handle;

	/* The list of ports on which an in-facing Stream identification
	 * function in the output (towards the system forwarding function)
	 * direction Only Active Destination MAC and VLAN Stream identification
	 * (or nothing) can be configured.
	 */
	__u32 ifac_oport;

	/* The list of ports on which an out-facing Stream identification
	 * function in the output (towards the physical interface) direction.
	 * Only Active Destination MAC and VLAN Stream identification
	 * (or nothing) can be configured.
	 */
	__u32 ofac_oport;

	/* The list of ports on which an in-facing Stream identification
	 * function in the input (coming from the system forwarding function)
	 * direction
	 */
	__u32 ifac_iport;

	/* The list of ports on which an out-facing Stream identification
	 * function in the input (coming from the physical interface) direction
	 * .
	 */
	__u32 ofac_iport;

	/* An enumerated value indicating the method used to identify packets
	 * belonging to the Stream.
	 * The Organizationally Unique Identifier (OUI) or Company Identifier
	 * (CID) to identify the organization defining the enumerated type
	 * should be: 00-80-C2
	 * 1: null stream identification
	 * 2: source mac and vlan stream identification
	 * 3: activ destination mac and vlan stream identification
	 * 4: ip stream identifaciton
	 */
	__u8 type;

	/* tsnStreamIdParameters The number of controlling parameters for a
	 * Stream identification method, their types and values, are specific
	 * to the tsnStreamIdIdentificationType
	 */
	union {
		struct tsn_cb_null_streamid nid;
		struct tsn_cb_source_streamid sid;
		struct tsn_cb_dest_streamid did;
		struct tsn_cb_ip_streamid iid;
	} para;
};

/* Following counters are instantiated for each port on which the Stream
 * identification function (6.2) is configured. The counters are indexed by
 * port number, facing (in-facing or out-facing), and stream_handle value
 * (tsnStreamIdHandle, 9.1.1.1).
 */
struct tsn_cb_streamid_counters {
	struct {
		__u64 input;
		__u64 output;
	} per_stream;

	struct {
		__u64 input;
		__u64 output;
	} per_streamport[32];
};

/* 802.1Qci Stream Parameter Table, read from port */
struct tsn_qci_psfp_stream_param {
	/* MaxStreamFilterInstances.
	 * The maximum number of Stream Filter instances supported by this
	 * Bridge component.
	 */
	__s32 max_sf_instance;

	/* MaxStreamGateInstances
	 * The maximum number of Stream Gate instances supported by this Bridge
	 * component.
	 */
	__s32 max_sg_instance;

	/* MaxFlowMeterInstances
	 * The maximum number of Flow Meter instances supported by this Bridge
	 * component.
	 */
	__s32 max_fm_instance;

	/* SupportedListMax
	 * The maximum value supported by this Bridge component of the
	 * AdminControlListLength and OperControlListLength parameters.
	 */
	__s32 supported_list_max;
};

/* 802.1Qci Stream Filter Instance Table, counters part only. */
struct tsn_qci_psfp_sfi_counters {
	/* The MatchingFramesCount counter counts received frames that match
	 * this stream filter.
	 */
	__u64 matching_frames_count;

	/* The PassingFramesCount counter counts received frames that pass the
	 * gate associated with this stream filter.
	 */
	__u64 passing_frames_count;

	/* The NotPassingFramesCount counter counts received frames that do not
	 * pass the gate associated with this stream filter.
	 */
	__u64 not_passing_frames_count;

	/* The PassingSDUCount counter counts received frames that pass the SDU
	 * size filter specification associated with this stream filter.
	 */
	__u64 passing_sdu_count;

	/* The NotPassingSDUCount counter counts received frames that do not
	 * pass the SDU size filter specification associated with this stream
	 * filter.
	 */
	__u64 not_passing_sdu_count;

	/* The  REDFramesCount counter counts received random early detection
	 * (RED) frames associated with this stream filter.
	 */
	__u64 red_frames_count;
};

/* 802.1Qci Stream Filter Instance Table, configuration part only. */
struct tsn_qci_psfp_sfi_conf {

	/* The StreamHandleSpec parameter contains a stream identifier
	 * specification value. A value of -1 denotes the wild card value; zero
	 * or positive values denote stream identifier values.
	 */
	__s32 stream_handle_spec;

	/* The PrioritySpec parameter contains a priority specification value.
	 * A value of -1 denotes the wild card value; zero or positive values
	 * denote priority values.
	 */
	__s8 priority_spec;

	/* The StreamGateInstanceID parameter contains the index of an entry in
	 * the Stream Gate Table.
	 */
	__u32 stream_gate_instance_id;

	/* The filter specifications. The actions specified in a filter
	 * specification can result in a frame passing or failing the specified
	 * filter. Frames that fail a filter are discarded.
	 */
	struct {
		/* The MaximumSDUSize parameter specifies the maximum allowed
		 * frame size for the stream. Any frame exceeding this value
		 * will be dropped.  A value of 0 denote that the MaximumSDUSize
		 * filter is disabled for this stream.
		 */
		__u16 maximum_sdu_size;

		/* The FlowMeterInstanceID parameter contains the index of an
		 * entry in the Flow Meter Table.  A value of -1 denotes that
		 * no flow meter is assigned; zero or positive values denote
		 * flow meter IDs.
		 */
		__s32 flow_meter_instance_id;
	} stream_filter;

	/* The StreamBlockedDueToOversizeFrameEnable object contains a Boolean
	 * value that indicates whether the StreamBlockedDueToOversizeFrame
	 * function is enabled (TRUE) or disabled (FALSE).
	 */
	__u8 block_oversize_enable;

	/* The StreamBlockedDueToOversizeFrame object contains a Boolean value
	 * that indicates whether, if the StreamBlockedDueToOversizeFrame
	 * function is enabled, all frames are to be discarded (TRUE) or not
	 * (FALSE).
	 */
	__u8 block_oversize;
};

/* 802.1Qci Stream Gate Control List Entry. */
struct tsn_qci_psfp_gcl {
	/* The GateState parameter specifies a desired state, open (true) or
	 * closed (false), for the stream gate.
	 */
	__u8 gate_state;

	/* An IPV is encoded as a signed integer.  A negative denotes the null
	 * value; zero or positive values denote internal priority values.
	 */
	__s8 ipv;

	/* A TimeInterval is encoded in 4 octets as a 32-bit unsigned integer,
	 * representing a number of nanoseconds.
	 */
	__u32 time_interval;

	/* The maximum number of octets that are permitted to pass the gate
	 * during the specified TimeInterval.  If zero, there is no maximum.
	 */
	__u32 octet_max;

};

/* 802.1Qci Stream Gate Admin/Operation common list control parameters */
struct tsn_qci_sg_control {
	/* The administrative/operation value of the GateStates parameter
	 * for the stream gate.  A value of false indicates closed;
	 * a value of true indicates open.
	 */
	__u8 gate_states;

	/* The administrative/operation value of the ListMax parameter for the
	 * gate. The integer value indicates the number of entries (TLVs) in
	 * the AdminControlList/OperControlList.
	 */
	__u8 control_list_length;

	/* The administrative/operation value of the CycleTime parameter for
	 * the gate.  The value is an unsigned integer number of nanoseconds.
	 */
	__u32 cycle_time;

	/* The administrative/operation value of the CycleTimeExtension
	 * parameter for the gate.  The value is an unsigned integer number
	 * of nanoseconds.
	 */
	__u32 cycle_time_extension;

	/* The administrative/operation value of the BaseTime parameter for the
	 * gate.  The value is a representation of a PTPtime value, consisting
	 * of a 48-bit integer number of seconds and a 32-bit integer number of
	 * nanoseconds.
	 */
	ptptime_t base_time;

	/* The administrative/operation value of the IPV parameter for the gate.
	 * A value of -1 denotes the null value; zero or positive values denote
	 * internal priority values.
	 */
	__s8 init_ipv;

	/* control_list contend the gate control list of
	 * administrative/operation
	 */
	struct tsn_qci_psfp_gcl *gcl;
};

/* 802.1Qci Stream Gate Instance Table, configuration part only. */
struct tsn_qci_psfp_sgi_conf {
	/* The GateEnabled parameter determines whether the stream gate is
	 * active (true) or inactive (false).
	 */
	__u8 gate_enabled;

	/* The ConfigChange parameter signals the start of a configuration
	 * change when it is set to TRUE. This should only be done when the
	 * various administrative parameters are all set to appropriate values.
	 */
	__u8 config_change;

	/* admin control parameters with admin control list */
	struct tsn_qci_sg_control admin;

	/* The GateClosedDueToInvalidRxEnable object contains a Boolean value
	 * that indicates whether the GateClosedDueToInvalidRx function is
	 * enabled (TRUE) or disabled (FALSE).
	 */
	__u8 block_invalid_rx_enable;

	/* The GateClosedDueToInvalidRx object contains a Boolean value that
	 * indicates whether, if the GateClosedDueToInvalidRx function is
	 * enabled, all frames are to be discarded (TRUE) or not (FALSE).
	 */
	__u8 block_invalid_rx;

	/* The GateClosedDueToOctetsExceededEnable object contains a Boolean
	 * value that indicates whether the GateClosedDueToOctetsExceeded
	 * function is enabled (TRUE) or disabled (FALSE).
	 */
	__u8 block_octets_exceeded_enable;

	/* The GateClosedDueToOctetsExceeded object contains a Boolean value
	 * that indicates whether, if the GateClosedDueToOctetsExceeded
	 * function is enabled, all frames are to be discarded (TRUE) or not
	 * (FALSE).
	 */
	__u8 block_octets_exceeded;
};

/* 802.1Qci Stream Gate Instance Table, status part only. */
struct tsn_psfp_sgi_status {

	/* admin control parameters with admin control list */
	struct tsn_qci_sg_control oper;

	/* The PTPtime at which the next config change is scheduled to occur.
	 * The value is a representation of a PTPtime value, consisting of a
	 * 48-bit integer number of seconds and a 32-bit integer number of
	 * nanoseconds.
	 */
	ptptime_t config_change_time;

	/* The granularity of the cycle time clock, represented as an unsigned
	 * number of tenths of nanoseconds.
	 */
	__u32 tick_granularity;

	/* The current time, in PTPtime, as maintained by the local system.
	 * The value is a representation of a PTPtime value, consisting of a
	 * 48-bit integer number of seconds and a 32-bit integer number of
	 * nanoseconds.
	 */
	ptptime_t current_time;

	/* The value of the ConfigPending state machine variable.  The value is
	 * TRUE if a configuration change is in progress but has not yet
	 * completed.
	 */
	__u8 config_pending;

	/* A counter of the number of times that a re-configuration of the
	 * traffic schedule has been requested with the old schedule still
	 * running and the requested base time was in the past.
	 */
	__u64 config_change_error;

};

/* 802.1Qci Flow Meter Instance Table. */
struct tsn_qci_psfp_fmi {
	/* The FlowMeterCIR parameter contains an integer value that represents
	 * the CIR value for the flow meter, in kbit/s.
	 */
	__u32 cir;

	/* The FlowMeterCBS parameter contains an integer value that represents
	 * the CBS value for the flow meter, in octets.
	 */
	__u32 cbs;

	/* The FlowMeterEIR parameter contains an integer value that represents
	 * the EIR value for the flow meter, in kbit/s.
	 */
	__u32 eir;

	/* The FlowMeterEBS parameter contains an integer value that represents
	 * the EBS value for the flow meter, in octets.
	 */
	__u32 ebs;

	/* The FlowMeterCF parameter contains a Boolean value that represents
	 * the CF value for the flow meter, as a Boolean value indicating no
	 * coupling (FALSE) or coupling (TRUE).
	 */
	__u8 cf;

	/* The FlowMeterCM parameter contains a Boolean value that represents
	 * the CM value for the flow meter, as a Boolean value indicating
	 * colorBlind (FALSE) or colorAware (TRUE).
	 */
	__u8 cm;

	/* The FlowMeterDropOnYellow parameter contains a Boolean value that
	 * indicates whether yellow frames are dropped (TRUE) or have
	 * drop_eligible set to TRUE (FALSE).
	 */
	__u8 drop_on_yellow;

	/* The FlowMeterMarkAllFramesRedEnable parameter contains a Boolean
	 * value that indicates whether the MarkAllFramesRed function
	 * is enabled (TRUE) or disabled (FALSE).
	 */
	__u8 mark_red_enable;

	/* The FlowMeterMarkAllFramesRed parameter contains a Boolean value
	 * that indicates whether, if the MarkAllFramesRed function is enabled,
	 * all frames are to be discarded (TRUE) or not (FALSE).
	 */
	__u8 mark_red;
};

struct tsn_qci_psfp_fmi_counters {
	__u64 bytecount;
	__u64 drop;
	__u64 dr0_green;
	__u64 dr1_green;
	__u64 dr2_yellow;
	__u64 remark_yellow;
	__u64 dr3_red;
	__u64 remark_red;
};

/* 802.1cb */
struct tsn_seq_gen_conf {

	/* The InputPortMask parameter contains a port mask.
	 * If the packet is from input port belonging to this
	 * port mask then it's on known stream and sequence
	 * generation parameters can be applied.
	 */
	__u8 iport_mask;

	/* The SplitMask parameter contains a output port mask
	 * used to add redundant paths.
	 */
	__u8 split_mask;

	/* The SequenceSpaceLenLog parameter is a value to specifies
	 * number of bits to be used for sequence number.
	 */
	__u8 seq_len;

	/* The SequenceNumber parameter is a value to used for
	 * outgoing packet's sequence number generation.
	 */
	__u32 seq_num;
};

struct tsn_seq_rec_conf {

	/* The SequenceSpaceLenLog parameter is a value to specifies
	 * number of bits to be used for sequence number.
	 */
	__u8 seq_len;

	/* The HistorySpaceLenLog parameter is a value to specifies
	 * number of bits to be used for history register.
	 */
	__u8 his_len;

	/* The RTagPopEnable parameter contains a __u8 to enable removal
	 * of redundancy tag from the packet.
	 */
	__u8 rtag_pop_en;
};

struct tsn_cb_status {

	/* The GenRecover parameter contains a value specifies type
	 * of stream sequence parameters:
	 *	0: Stream sequence parameters are for generation.
	 *	1: Stream sequence parameters are for recovery.
	 */
	__u8 gen_rec;

	/* The ErrStatus parameter indicates stream's error status
	 * 1: This switch is expected to sequence the stream,
	 *    but the incoming packet has sequence number.
	 * 2: This switch is expected to recover the stream,
	 *    but the incoming packet is NONSEQ.
	 */
	__u8 err;

	/* The SequenceNumber parameter is a value to used for
	 * outgoing packet's sequence number generation.
	 */
	__u32 seq_num;

	/* The SequenceSpaceLenLog parameter is a value to specifies
	 * number of bits to be used for sequence number.
	 */
	__u8 seq_len;

	/* The SplitMask parameter contains a output port mask
	 * used to add redundant paths.
	 */
	__u8 split_mask;

	/* The InputPortMask parameter contains a port mask.
	 * If the packet is from input port belonging to this
	 * port mask then it's on known stream and sequence
	 * generation parameters can be applied.
	 */
	__u8 iport_mask;

	/* The HistorySpaceLenLog parameter is a value to specifies
	 * number of bits to be used for history register.
	 */
	__u8 his_len;

	/* The SequenceHistory parameter Maintains history of sequence
	 * numbers of received packets.
	 */
	__u32 seq_his;
};

/* An entry for gate control list */
struct tsn_qbv_entry {
	/* Octet represent the gate states for the corresponding traffic
	 * classes.
	 * The MS bit corresponds to traffic class 7.
	 * The LS bit to traffic class 0.
	 * A bit value of 0 indicates closed;
	 * A bit value of 1 indicates open.
	 */
	__u8 gate_state;

	/* A TimeInterval is encoded in 4 octets as a 32-bit unsigned integer,
	 * representing a number of nanoseconds.
	 */
	__u32 time_interval;
};

/* The administrative/operation time and gate list */
struct tsn_qbv_basic {
	/* The administrative/operation value of the GateStates parameter for
	 * the Port.
	 * The bits of the octet represent the gate states for the
	 * corresponding traffic classes; the MS bit corresponds to traffic
	 * class 7, the LS bit to traffic class 0. A bit value of 0 indicates
	 * closed; a bit value of 1 indicates open.
	 * The value of this object MUST be retained
	 * across reinitializations of the management system.
	 */
	__u8 gate_states;

	/* The administrative/operation value of the ListMax parameter for the
	 * port. The integer value indicates the number of entries (TLVs) in
	 * the AdminControlList. The value of this object MUST be retained
	 * across reinitializations of the management system.
	 */
	__u32 control_list_length;

	/* The administrative/operation value of the AdminCycleTime
	 * parameter for the Port. The numerator and denominator together
	 * represent the cycle time as a rational number of seconds.  The value
	 * of this object MUST be retained across reinitializations of the
	 * management system.
	 */
	__u32 cycle_time;

	/* The administrative/operation value of the CycleTimeExtension
	 * parameter for the Port. The value is an unsigned integer number of
	 * nanoseconds.
	 * The value of this object MUST be retained across reinitializations
	 * of the management system.
	 */

	__u32 cycle_time_extension;

	/* The administrative/operation value of the BaseTime parameter for the
	 * Port.  The value is a representation of a PTPtime value, consisting
	 * of a 48-bit integer number of seconds and a 32-bit integer number of
	 * nanoseconds.
	 * The value of this object MUST be retained across reinitializations of
	 * the management system.
	 */
	ptptime_t base_time;

	/* admin_control_list represent the AdminControlList/OperControlList.
	 * The administrative version of the gate control list for the Port.
	 */
	struct tsn_qbv_entry *control_list;
};

struct tsn_qbv_conf {
	/* The GateEnabled parameter determines whether traffic scheduling is
	 * active (true) or inactive (false).  The value of this object MUST be
	 * retained across reinitializations of the management system.
	 */
	__u8 gate_enabled;

	/* The maxsdu parameter denoting the maximum SDU size supported by the
	 * queue.
	 */
	__u32 maxsdu;

	/* The ConfigChange parameter signals the start of a configuration
	 * change when it is set to TRUE. This should only be done when the
	 * various administrative parameters are all set to appropriate values.
	 */
	__u8 config_change;

	/* The admin parameter signals the admin relate cycletime, basictime,
	 * gatelist paraters.
	 */
	struct tsn_qbv_basic admin;
};

/* 802.1Qbv (Time Aware Shaper) port status */
struct tsn_qbv_status {
	/* The PTPtime at which the next config change is scheduled to occur.
	 * The value is a representation of a PTPtime value, consisting of a
	 * 48-bit integer number of seconds and a 32-bit integer number of
	 * nanoseconds.  The value of this object MUST be retained across
	 * reinitializations of the management system.
	 */
	ptptime_t config_change_time;

	/* The granularity of the cycle time clock, represented as an unsigned
	 * number of tenths of nanoseconds.  The value of this object MUST be
	 * retained across reinitializations of the management system.
	 */
	__u32 tick_granularity;

	/* The current time, in PTPtime, as maintained by the local system.
	 * The value is a representation of a PTPtime value, consisting of a
	 * 48-bit integer number of seconds and a 32-bit integer number of
	 * nanoseconds.
	 */
	ptptime_t  current_time;

	/* The value of the ConfigPending state machine variable.  The value is
	 * TRUE if a configuration change is in progress but has not yet
	 * completed.
	 */
	__u8 config_pending;

	/* A counter of the number of times that a re-configuration of the
	 * traffic schedule has been requested with the old schedule still
	 * running and the requested base time was in the past.
	 */
	__u64 config_change_error;

	/* The maximum value supported by this Port of the
	 * AdminControlListLength and OperControlListLength parameters.
	 */
	__u32 supported_list_max;

	/* Operation settings parameters and Oper gate list */
	struct tsn_qbv_basic oper;
};

/* Time Specific Departure parameters */
struct tsn_tsd {
	__u8 enable;

	/* The cycle time, in units of microsecond(us)*/
	__u32 period;

	/* The maximum number of frames which could be transmitted on one cycle
	 *  The exceeding frames will be transmitted on next cycle.
	 */
	__u32 maxFrameNum;

	/* Specify the time of the first cycle begins.
	 *      1:  begin when the queue get the first frame to transmit.
	 *      2:  begin immediately at the end of setting function.
	 */
	__u32 syn_flag;
};

struct tsn_tsd_status {
	__u8 enable;
	__u32 period;
	__u32 maxFrameNum;
	__u32 flag;
	__u32 cycleNum;
	__u32 loss_steps;
};

struct tsn_qos_switch_dscp_conf {
	__u8 trust;
	__u8 cos;
	__u8 dpl;
	__u8 remark;
	__u8 dscp; /* New ingress translated DSCP value */
};

struct tsn_qos_switch_pcp_conf {
	__u8 pcp;
	__u8 dei;
	__u8 cos;
	__u8 dpl;
};

#endif /* _UAPI_GENL_TSN_H */
