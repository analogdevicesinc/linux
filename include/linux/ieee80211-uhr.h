/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * IEEE 802.11 UHR definitions
 *
 * Copyright (c) 2025-2026 Intel Corporation
 */
#ifndef LINUX_IEEE80211_UHR_H
#define LINUX_IEEE80211_UHR_H

#include <linux/types.h>
#include <linux/bitfield.h>
#include <linux/if_ether.h>
#include <linux/unaligned.h>
#include "ieee80211-eht.h"

#define IEEE80211_UHR_OPER_PARAMS_DPS_ENA		0x0001
#define IEEE80211_UHR_OPER_PARAMS_NPCA_ENA		0x0002
#define IEEE80211_UHR_OPER_PARAMS_PEDCA_ENA		0x0004
#define IEEE80211_UHR_OPER_PARAMS_DBE_ENA		0x0008
#define IEEE80211_UHR_OPER_PARAMS_DBE_BW		0x0070
#define IEEE80211_UHR_OPER_PARAMS_ELR_RX		0x0080
#define IEEE80211_UHR_OPER_PARAMS_DUO_PRES		0x0100
#define IEEE80211_UHR_OPER_PARAMS_DPS_PRES		0x0200
#define IEEE80211_UHR_OPER_PARAMS_NPCA_PRES		0x0400
#define IEEE80211_UHR_OPER_PARAMS_PEDCA_PRES		0x0800
#define IEEE80211_UHR_OPER_PARAMS_DBE_PRES		0x1000

struct ieee80211_uhr_operation {
	__le16 params;
	u8 variable[];
} __packed;

#define IEEE80211_UHR_NPCA_PARAMS_PRIMARY_CHAN		0x000000FF
#define IEEE80211_UHR_NPCA_PARAMS_MIN_DUR_THRESH	0x00000F00
#define IEEE80211_UHR_NPCA_PARAMS_SWITCH_DELAY		0x0003F000
#define IEEE80211_UHR_NPCA_PARAMS_SWITCH_BACK_DELAY	0x00FC0000
#define IEEE80211_UHR_NPCA_PARAMS_INIT_QSRC		0x03000000
#define IEEE80211_UHR_NPCA_PARAMS_MOPLEN		0x04000000
#define IEEE80211_UHR_NPCA_PARAMS_DIS_SUBCH_BMAP_PRES	0x08000000

/**
 * struct ieee80211_uhr_npca_info - npca operation information
 *
 * This structure is the "NPCA Operation Parameters field format" of "UHR
 * Operation Element" fields as described in P802.11bn_D1.5
 * subclause 9.4.2.355.2, see Figure 9-aa6.
 *
 * Refer to IEEE80211_UHR_NPCA*
 * @params:
 *	NPCA Primary Channel - NPCA primary channel
 *	NPCA_Min Duration Threshold - Minimum duration of inter-BSS activity
 *	NPCA Switching Delay -
 *		Time needed by an NPCA AP to switch from the
 *		BSS primary channel to the NPCA primary channel
 *		in the unit of 4 µs.
 *	NPCA Switching Back Delay -
 *		Time to switch from the NPCA primary channel
 *		to the BSS primary channel in the unit of 4 µs.
 *	NPCA Initial QSRC -
 *		Initialize the EDCAF QSRC[AC] variables
 *		when an NPCA STA in the BSS
 *		switches to NPCA operation.
 *	NPCA MOPLEN -
 *		Indicates which conditions can be used to
 *		initiate an NPCA operation,
 *		1 -> both PHYLEN NPCA operation and MOPLEN
 *		NPCA operation are
 *		permitted in the BSS
 *		0 -> only PHYLEN NPCA operation is allowed in the BSS.
 *	NPCA Disabled Subchannel Bitmap Present -
 *		Indicates whether the NPCA Disabled Subchannel
 *		Bitmap field is present. A 1 in this field indicates that
 *		the NPCA Disabled Subchannel Bitmap field is present
 * @dis_subch_bmap:
 *		A bit in the bitmap that lies within the BSS bandwidth is set
 *		to 1 to indicate that the corresponding 20 MHz subchannel is
 *		punctured and is set to 0 to indicate that the corresponding
 *		20 MHz subchannel is not punctured. A bit in the bitmap that
 *		falls outside of the BSS bandwidth is reserved. This field is
 *		present when the value of the NPCA Disabled Subchannel Bitmap
 *		Field Present field is equal to 1, and not present, otherwise
 */
struct ieee80211_uhr_npca_info {
	__le32 params;
	__le16 dis_subch_bmap[];
} __packed;

static inline int
ieee80211_uhr_npca_info_size(const struct ieee80211_uhr_npca_info *npca)
{
	if (npca->params & cpu_to_le32(IEEE80211_UHR_NPCA_PARAMS_DIS_SUBCH_BMAP_PRES))
		return sizeof(*npca) + sizeof(npca->dis_subch_bmap[0]);

	return sizeof(*npca);
}

#define IEEE80211_UHR_DPS_PADDING_DELAY			0x0000003F
#define IEEE80211_UHR_DPS_TRANSITION_DELAY		0x00003F00
#define IEEE80211_UHR_DPS_ICF_REQUIRED			0x00010000
#define IEEE80211_UHR_DPS_PARAMETERIZED_FLAG		0x00020000
#define IEEE80211_UHR_DPS_LC_MODE_BW			0x001C0000
#define IEEE80211_UHR_DPS_LC_MODE_NSS			0x01E00000
#define IEEE80211_UHR_DPS_LC_MODE_MCS			0x1E000000
#define IEEE80211_UHR_DPS_MOBILE_AP_DPS_STATIC_HCM	0x20000000

/**
 * struct ieee80211_uhr_dps_info - DPS operation information
 *
 * This structure is the "DPS Operation Parameter field" of "UHR
 * Operation Element" fields as described in P802.11bn_D1.5
 * subclause 9.4.1.87. See Figure 9-207u.
 *
 * Refer to IEEE80211_UHR_DPS*
 * @params:
 *	DPS Padding Delay -
 *		Indicates the minimum MAC padding
 *		duration that is required by a DPS STA
 *		in an ICF to cause the STA to transition
 *		from the lower capability mode to the
 *		higher capability mode. The DPS Padding
 *		Delay field is in units of 4 µs.
 *	DPS Transition Delay -
 *		Indicates the amount of time required by a
 *		DPS STA to transition from the higher
 *		capability mode to the lower capability
 *		mode. The DPS Transition Delay field is in
 *		units of 4 µs.
 *	ICF Required -
 *		Indicates when the DPS assisting STA needs
 *		to transmit an ICF frame to the peer DPS STA
 *		before performing the frame exchanges with
 *		the peer DPS STA in a TXOP.
 *			1 -> indicates that the transmission of the
 *			ICF frame to the peer DPS STA prior to
 *			any frame exchange is needed.
 *			0 -> ICF transmission before the frame
 *			exchanges with the peer DPS STA is only
 *			needed if the frame exchange is performed
 *			in the HC mode.
 *	Parameterized Flag -
 *		0 -> indicates that only 20 MHz, 1 SS,
 *		non-HT PPDU format with the data
 *		rate of 6, 12, and 24 Mb/s as the
 *		default mode are supported by the
 *		DPS STA in the LC mode
 *		1 -> indicates that a bandwidth up to the
 *		bandwidth indicated in the LC Mode
 *		Bandwidth field, a number of spatial
 *		streams up to the NSS indicated in
 *		the LC Mode Nss field, and an MCS up
 *		to the MCS indicated in the LC Mode
 *		MCS fields are supported by the DPS
 *		STA in the LC mode as the
 *		parameterized mode.
 *	LC Mode Bandwidth -
 *		Indicates the maximum bandwidth supported
 *		by the STA in the LC mode.
 *	LC Mode NSS -
 *		Indicates the maximum number of the spatial
 *		streams supported by the STA in the LC mode.
 *	LC Mode MCS -
 *		Indicates the highest MCS supported by the STA
 *		in the LC mode.
 *	Mobile AP DPS Static HCM -
 *		1 -> indicates that it will remain in the DPS high
 *		capability mode until the next TBTT on that
 *		link.
 *		0 -> otherwise.
 */
struct ieee80211_uhr_dps_info {
	__le32 params;
} __packed;

enum ieee80211_uhr_duo_params {
	IEEE80211_UHR_DUO_PARAMS_MAX_STANDALONE_DUO_BSPR	= 0x1f,
};

/**
 * struct ieee80211_uhr_duo_info - DUO information
 * @params: the parameters, see &enum ieee80211_uhr_duo_params
 */
struct ieee80211_uhr_duo_info {
	u8 params;
} __packed;

#define IEEE80211_UHR_DBE_OPER_BANDWIDTH			0x07
#define IEEE80211_UHR_DBE_OPER_DIS_SUBCHANNEL_BITMAP_PRES	0x08

/**
 * enum ieee80211_uhr_dbe_oper_bw - DBE Operational Bandwidth
 *
 * Encoding for the DBE Operational Bandwidth field in the UHR Operation
 * element (DBE Operation Parameters).
 *
 * @IEEE80211_UHR_DBE_OPER_BW_40: 40 MHz operational DBE bandwidth
 * @IEEE80211_UHR_DBE_OPER_BW_80: 80 MHz operational DBE bandwidth
 * @IEEE80211_UHR_DBE_OPER_BW_160: 160 MHz operational DBE bandwidth
 * @IEEE80211_UHR_DBE_OPER_BW_320_1: 320-1 MHz operational DBE bandwidth
 * @IEEE80211_UHR_DBE_OPER_BW_320_2: 320-2 MHz operational DBE bandwidth
 */
enum ieee80211_uhr_dbe_oper_bw {
	IEEE80211_UHR_DBE_OPER_BW_40 = 1,
	IEEE80211_UHR_DBE_OPER_BW_80 = 2,
	IEEE80211_UHR_DBE_OPER_BW_160 = 3,
	IEEE80211_UHR_DBE_OPER_BW_320_1 = 4,
	IEEE80211_UHR_DBE_OPER_BW_320_2 = 5,
};

/**
 * ieee80211_uhr_dbe_bw_mhz - get bandwidth in MHz from UHR DBE bandwidth
 * @bw: UHR DBE bandwidth
 *
 * Return: the bandwidth in MHz, or -1 for invalid values
 */
static inline int ieee80211_uhr_dbe_bw_mhz(enum ieee80211_uhr_dbe_oper_bw bw)
{
	switch (bw) {
	case IEEE80211_UHR_DBE_OPER_BW_40:
		return 40;
	case IEEE80211_UHR_DBE_OPER_BW_80:
		return 80;
	case IEEE80211_UHR_DBE_OPER_BW_160:
		return 160;
	case IEEE80211_UHR_DBE_OPER_BW_320_1:
	case IEEE80211_UHR_DBE_OPER_BW_320_2:
		return 320;
	default:
		return -1;
	}
}

/**
 * struct ieee80211_uhr_dbe_info - DBE operation information
 *
 * This structure is the "DBE Operation Parameters field" of
 * "UHR Operation Element" fields as described in P802.11bn_D1.5
 * subclause 9.4.2.355.4, see Figure 9-aa9.
 *
 * Refer to IEEE80211_UHR_DBE_OPER*
 * @params:
 *	B0-B2 - DBE Operational Bandwidth field, see
 *	"enum ieee80211_uhr_dbe_oper_bw" for values.
 *	Value 0 is reserved.
 *	Value 1 indicates 40 MHz operational DBE bandwidth.
 *	Value 2 indicates 80 MHz operational DBE bandwidth.
 *	Value 3 indicates 160 MHz operational DBE bandwidth.
 *	Value 4 indicates 320-1 MHz operational DBE bandwidth.
 *	Value 5 indicates 320-2 MHz operational DBE bandwidth.
 *	Values 6 to 7 are reserved.
 *	B3 - DBE Disabled Subchannel Bitmap Present.
 * @dis_subch_bmap: DBE Disabled Subchannel Bitmap field is set to indicate
 *	disabled 20 MHz subchannels within the DBE Bandwidth.
 */
struct ieee80211_uhr_dbe_info {
	u8 params;
	__le16 dis_subch_bmap[];
} __packed;

static inline int
ieee80211_uhr_dbe_info_size(const struct ieee80211_uhr_dbe_info *dbe)
{
	if (dbe->params & IEEE80211_UHR_DBE_OPER_DIS_SUBCHANNEL_BITMAP_PRES)
		return sizeof(*dbe) + sizeof(dbe->dis_subch_bmap[0]);

	return sizeof(*dbe);
}

#define IEEE80211_UHR_P_EDCA_ECWMIN		0x0007
#define IEEE80211_UHR_P_EDCA_ECWMAX		0x0038
#define IEEE80211_UHR_P_EDCA_AIFSN		0x01C0
#define IEEE80211_UHR_P_EDCA_CW_DS		0x0600
#define IEEE80211_UHR_P_EDCA_PSRC_THRESHOLD	0x3800
#define IEEE80211_UHR_P_EDCA_QSRC_THRESHOLD	0xC000

/**
 * struct ieee80211_uhr_p_edca_info - P-EDCA operation information
 *
 * This structure is the "P-EDCA Operation Parameters field" of
 * "UHR Operation Element" fields as described in P802.11bn_D1.5
 * subclause 9.4.2.355.3, see Figure 9-aa8.
 *
 * Refer to IEEE80211_UHR_P_EDCA*
 * @params: ECWmin, ECWmax, AIFSN, CW DS, PSRC threshold, and QSRC threshold.
 *	- CWmin/CWmax values used by a P-EDCA STA during P-EDCA contention.
 *	- The AIFSN field indicates the AIFSN value used by a P-EDCA STA
 *	  during P-EDCA contention.
 *	- The CW DS field indicates the value used for randomization of the
 *	  transmission slot of the DS-CTS frame. The value 3 is reserved.
 *	  The value 0 indicates that randomization is not enabled.
 *	- The P-EDCA PSRC threshold field indicates the maximum number of
 *	  allowed consecutive DS-CTS transmissions. The value 0 and values
 *	  greater than 4 are reserved.
 *	- The P-EDCA QSRC threshold field indicates the value of the
 *	  QSRC[AC_VO] counter required to start P-EDCA contention. The
 *	  value 0 is reserved.
 */
struct ieee80211_uhr_p_edca_info {
	__le16 params;
} __packed;

static inline bool ieee80211_uhr_oper_size_ok(const u8 *data, u8 len)
{
	const struct ieee80211_uhr_operation *oper = (const void *)data;
	u8 needed = sizeof(*oper);

	if (len < needed)
		return false;

	/* DPS Operation Parameters (fixed 4 bytes) */
	if (oper->params & cpu_to_le16(IEEE80211_UHR_OPER_PARAMS_DPS_PRES)) {
		needed += sizeof(struct ieee80211_uhr_dps_info);
		if (len < needed)
			return false;
	}

	if (oper->params & cpu_to_le16(IEEE80211_UHR_OPER_PARAMS_DUO_PRES)) {
		needed += 1;
		if (len < needed)
			return false;
	}

	/* NPCA Operation Parameters (fixed 4 bytes + optional 2 bytes) */
	if (oper->params & cpu_to_le16(IEEE80211_UHR_OPER_PARAMS_NPCA_PRES)) {
		const struct ieee80211_uhr_npca_info *npca =
			(const void *)(data + needed);

		if (len < needed + sizeof(*npca))
			return false;

		needed += ieee80211_uhr_npca_info_size(npca);
		if (len < needed)
			return false;
	}

	/* P-EDCA Operation Parameters */
	if (oper->params & cpu_to_le16(IEEE80211_UHR_OPER_PARAMS_PEDCA_PRES)) {
		needed += sizeof(struct ieee80211_uhr_p_edca_info);
		if (len < needed)
			return false;
	}

	/* DBE Operation Parameters (fixed 1 byte + optional 2 bytes) */
	if (oper->params & cpu_to_le16(IEEE80211_UHR_OPER_PARAMS_DBE_PRES)) {
		const struct ieee80211_uhr_dbe_info *dbe =
			(const void *)(data + needed);

		if (len < needed + sizeof(*dbe))
			return false;

		needed += ieee80211_uhr_dbe_info_size(dbe);
		if (len < needed)
			return false;
	}

	return len >= needed;
}

/* Note: must ensure ieee80211_uhr_oper_size_ok(...) first */
static inline const struct ieee80211_uhr_npca_info *
ieee80211_uhr_npca_info(const struct ieee80211_uhr_operation *oper)
{
	const u8 *pos = oper->variable;

	if (!(oper->params & cpu_to_le16(IEEE80211_UHR_OPER_PARAMS_NPCA_PRES)))
		return NULL;

	if (!(oper->params & cpu_to_le16(IEEE80211_UHR_OPER_PARAMS_NPCA_ENA)))
		return NULL;

	if (oper->params & cpu_to_le16(IEEE80211_UHR_OPER_PARAMS_DUO_PRES))
		pos += 1;

	if (oper->params & cpu_to_le16(IEEE80211_UHR_OPER_PARAMS_DPS_PRES))
		pos += sizeof(struct ieee80211_uhr_dps_info);

	return (const void *)pos;
}

static inline const __le16 *
ieee80211_uhr_npca_dis_subch_bitmap(const struct ieee80211_uhr_operation *oper)
{
	const struct ieee80211_uhr_npca_info *npca;

	npca = ieee80211_uhr_npca_info(oper);
	if (!npca)
		return NULL;
	if (!(npca->params & cpu_to_le32(IEEE80211_UHR_NPCA_PARAMS_DIS_SUBCH_BMAP_PRES)))
		return NULL;
	return npca->dis_subch_bmap;
}

/* Note: must ensure ieee80211_uhr_oper_size_ok(...) first */
static inline const struct ieee80211_uhr_dbe_info *
ieee80211_uhr_oper_dbe_info(const struct ieee80211_uhr_operation *oper)
{
	const u8 *pos = oper->variable;

	if (!(oper->params & cpu_to_le16(IEEE80211_UHR_OPER_PARAMS_DBE_PRES)))
		return NULL;

	if (!(oper->params & cpu_to_le16(IEEE80211_UHR_OPER_PARAMS_DBE_ENA)))
		return NULL;

	if (oper->params & cpu_to_le16(IEEE80211_UHR_OPER_PARAMS_DUO_PRES))
		pos += 1;

	if (oper->params & cpu_to_le16(IEEE80211_UHR_OPER_PARAMS_DPS_PRES))
		pos += sizeof(struct ieee80211_uhr_dps_info);

	if (oper->params & cpu_to_le16(IEEE80211_UHR_OPER_PARAMS_NPCA_PRES)) {
		const struct ieee80211_uhr_npca_info *npca = (const void *)pos;

		pos += sizeof(*npca);
		if (npca->params & cpu_to_le32(IEEE80211_UHR_NPCA_PARAMS_DIS_SUBCH_BMAP_PRES))
			pos += sizeof(npca->dis_subch_bmap[0]);
	}

	if (oper->params & cpu_to_le16(IEEE80211_UHR_OPER_PARAMS_PEDCA_PRES))
		pos += sizeof(struct ieee80211_uhr_p_edca_info);

	return (const void *)pos;
}

#define IEEE80211_UHR_MAC_CAP0_DPS_SUPP			0x01
#define IEEE80211_UHR_MAC_CAP0_DPS_ASSIST_SUPP		0x02
#define IEEE80211_UHR_MAC_CAP0_DPS_AP_STATIC_HCM_SUPP	0x04
#define IEEE80211_UHR_MAC_CAP0_NPCA_SUPP		0x10
#define IEEE80211_UHR_MAC_CAP0_ENH_BSR_SUPP		0x20
#define IEEE80211_UHR_MAC_CAP0_ADD_MAP_TID_SUPP		0x40
#define IEEE80211_UHR_MAC_CAP0_EOTSP_SUPP		0x80

#define IEEE80211_UHR_MAC_CAP1_DSO_SUPP			0x01
#define IEEE80211_UHR_MAC_CAP1_PEDCA_SUPP		0x02
#define IEEE80211_UHR_MAC_CAP1_DBE_SUPP			0x04
#define IEEE80211_UHR_MAC_CAP1_UL_LLI_SUPP		0x08
#define IEEE80211_UHR_MAC_CAP1_P2P_LLI_SUPP		0x10
#define IEEE80211_UHR_MAC_CAP1_PUO_SUPP			0x20
#define IEEE80211_UHR_MAC_CAP1_AP_PUO_SUPP		0x40
#define IEEE80211_UHR_MAC_CAP1_DUO_SUPP			0x80

#define IEEE80211_UHR_MAC_CAP2_OMC_UL_MU_DIS_RX_SUPP	0x01
#define IEEE80211_UHR_MAC_CAP2_AOM_SUPP			0x02
#define IEEE80211_UHR_MAC_CAP2_IFCS_LOC_SUPP		0x04
#define IEEE80211_UHR_MAC_CAP2_UHR_TRS_SUPP		0x08
#define IEEE80211_UHR_MAC_CAP2_TXSPG_SUPP		0x10
#define IEEE80211_UHR_MAC_CAP2_TXOP_RET_IN_TXSPG	0x20
#define IEEE80211_UHR_MAC_CAP2_UHR_OM_PU_TO_LOW		0xC0

#define IEEE80211_UHR_MAC_CAP3_UHR_OM_PU_TO_HIGH	0x03
#define IEEE80211_UHR_MAC_CAP3_PARAM_UPD_ADV_NOTIF_INTV	0x7C
#define IEEE80211_UHR_MAC_CAP3_UPD_IND_TIM_INTV_LOW	0x80

#define IEEE80211_UHR_MAC_CAP4_UPD_IND_TIM_INTV_HIGH	0x0F
#define IEEE80211_UHR_MAC_CAP4_BOUNDED_ESS		0x10
#define IEEE80211_UHR_MAC_CAP4_BTM_ASSURANCE		0x20
#define IEEE80211_UHR_MAC_CAP4_CO_BF_SUPP		0x40
#define IEEE80211_UHR_MAC_CAP4_CO_SR_SUPP		0x80

#define IEEE80211_UHR_MAC_CAP5_MAPC_ENH_MSRMT_SUPP	0x01

#define IEEE80211_UHR_MAC_CAP_DBE_MAX_BW		0x07
#define IEEE80211_UHR_MAC_CAP_DBE_EHT_MCS_MAP_160_PRES	0x08
#define IEEE80211_UHR_MAC_CAP_DBE_EHT_MCS_MAP_320_PRES	0x10

/* struct ieee80211_uhr_cap_dbe::bwcap[].cap */
#define IEEE80211_UHR_MAC_CAP_DBE_CAP_NUM_SND_DIMS		0x07
#define IEEE80211_UHR_MAC_CAP_DBE_CAP_NON_OFDMA_UL_MUMIMO	0x08
#define IEEE80211_UHR_MAC_CAP_DBE_CAP_MU_BEAMFORMER		0x10
#define IEEE80211_UHR_MAC_CAP_DBE_CAP_BEAMFORMEE_SS		0xe0

struct ieee80211_uhr_cap_dbe {
	u8 cap;
	u8 max_switch_time_period;
	u8 mode_change_intvl;
	/* present 0, 1 or 2 times depending on _PRES bits */
	struct ieee80211_uhr_cap_dbe_bwcap {
		struct ieee80211_eht_mcs_nss_supp_bw eht_mcs_map;
		u8 cap;
	} __packed bwcap[];
} __packed;

/**
 * enum ieee80211_uhr_dbe_max_supported_bw - DBE Maximum Supported Bandwidth
 *
 * As per spec P802.11bn_D1.5 Table 9-bb8 "Encoding of the DBE Maximum
 * Supported Bandwidth field".
 *
 * @IEEE80211_UHR_DBE_MAX_BW_40: Indicates 40 MHz DBE max supported bw
 * @IEEE80211_UHR_DBE_MAX_BW_80: Indicates 80 MHz DBE max supported bw
 * @IEEE80211_UHR_DBE_MAX_BW_160: Indicates 160 MHz DBE max supported bw
 * @IEEE80211_UHR_DBE_MAX_BW_320: Indicates 320 MHz DBE max supported bw
 */
enum ieee80211_uhr_dbe_max_supported_bw {
	IEEE80211_UHR_DBE_MAX_BW_40 = 1,
	IEEE80211_UHR_DBE_MAX_BW_80 = 2,
	IEEE80211_UHR_DBE_MAX_BW_160 = 3,
	IEEE80211_UHR_DBE_MAX_BW_320 = 4,
};

struct ieee80211_uhr_cap_mac {
	u8 mac_cap[6];
} __packed;

#define IEEE80211_UHR_PHY_CAP_MAX_NSS_RX_SND_NDP_LE80	0x00000001
#define IEEE80211_UHR_PHY_CAP_MAX_NSS_RX_DL_MU_LE80	0x00000002
#define IEEE80211_UHR_PHY_CAP_MAX_NSS_RX_SND_NDP_160	0x00000004
#define IEEE80211_UHR_PHY_CAP_MAX_NSS_RX_DL_MU_160	0x00000008
#define IEEE80211_UHR_PHY_CAP_MAX_NSS_RX_SND_NDP_320	0x00000010
#define IEEE80211_UHR_PHY_CAP_MAX_NSS_RX_DL_MU_320	0x00000020
#define IEEE80211_UHR_PHY_CAP_ELR_TX			0x00000040
#define IEEE80211_UHR_PHY_CAP_ELR_RX			0x00000080
#define IEEE80211_UHR_PHY_CAP_PART_BW_DL_MUMIMO		0x00000100
#define IEEE80211_UHR_PHY_CAP_PART_BW_UL_MUMIMO		0x00000200
#define IEEE80211_UHR_PHY_CAP_MCS15			0x00000400
#define IEEE80211_UHR_PHY_CAP_2XLDPC_TX			0x00000800
#define IEEE80211_UHR_PHY_CAP_2XLDPC_RX			0x00001000
#define IEEE80211_UHR_PHY_CAP_UEQM_TX_MAX_NSS		0x00006000
#define IEEE80211_UHR_PHY_CAP_UEQM_RX_MAX_NSS		0x00018000
#define IEEE80211_UHR_PHY_CAP_CO_BF_JOINT_SOUNDING	0x00040000
#define IEEE80211_UHR_PHY_CAP_IM_TX			0x00080000
#define IEEE80211_UHR_PHY_CAP_IM_RX			0x00100000
#define IEEE80211_UHR_PHY_CAP_CO_SR_MODE_1		0x00200000
#define IEEE80211_UHR_PHY_CAP_CO_SR_MODE_2		0x00400000
#define IEEE80211_UHR_PHY_CAP_DRU_DBW_20_IN_PBW_20	0x00800000
#define IEEE80211_UHR_PHY_CAP_DRU_DBW_40_IN_PBW_40	0x01000000
#define IEEE80211_UHR_PHY_CAP_DRU_DBW_80_IN_PBW_80	0x02000000
#define IEEE80211_UHR_PHY_CAP_DRU_DBW_80_IN_PBW_160	0x04000000
#define IEEE80211_UHR_PHY_CAP_DRU_DBW_80_IN_PBW_320	0x08000000
#define IEEE80211_UHR_PHY_CAP_DRU_DBW_20_IN_PBW_GE80	0x10000000
#define IEEE80211_UHR_PHY_CAP_DRU_DBW_40_IN_PBW_GE80	0x20000000
#define IEEE80211_UHR_PHY_CAP_DRU_DBW_60_IN_PBW_GE80	0x40000000
#define IEEE80211_UHR_PHY_CAP_DRU_RRU_HYBRID_MODE	0x80000000

struct ieee80211_uhr_cap_phy {
	__le32 cap;
	u8 reserved;
} __packed;

struct ieee80211_uhr_cap {
	struct ieee80211_uhr_cap_mac mac;
	struct ieee80211_uhr_cap_phy phy;
	/* optional DBE capabilities */
	u8 variable[];
} __packed;

static inline bool ieee80211_uhr_capa_size_ok(const u8 *data, u8 len,
					      bool from_ap)
{
	const struct ieee80211_uhr_cap *cap = (const void *)data;
	size_t needed = sizeof(*cap);

	if (len < needed)
		return false;

	/*
	 * A non-AP STA does not include the DBE Capability Parameters field
	 * in the UHR MAC Capabilities Information field.
	 */
	if (from_ap && cap->mac.mac_cap[1] & IEEE80211_UHR_MAC_CAP1_DBE_SUPP) {
		const struct ieee80211_uhr_cap_dbe *dbe;

		needed += sizeof(struct ieee80211_uhr_cap_dbe);
		if (len < needed)
			return false;

		dbe = (const void *)cap->variable;

		if (dbe->cap & IEEE80211_UHR_MAC_CAP_DBE_EHT_MCS_MAP_160_PRES)
			needed += sizeof(dbe->bwcap[0]);

		if (dbe->cap & IEEE80211_UHR_MAC_CAP_DBE_EHT_MCS_MAP_320_PRES)
			needed += sizeof(dbe->bwcap[0]);
	}

	return len >= needed;
}

#define IEEE80211_UHR_OM_PU_TO_128TU	11

/**
 * ieee80211_uhr_capa_get_om_pu_to_us - get OM parameter update timeout in usec
 * @cap: the UHR capability element, size must be validated
 *
 * Return: the OM parameter update timeout in usec, or -1 if it's not valid
 */
static inline int
ieee80211_uhr_capa_get_om_pu_to_us(const struct ieee80211_uhr_cap *cap)
{
	u8 timeout;

	timeout = u8_get_bits(cap->mac.mac_cap[3],
			      IEEE80211_UHR_MAC_CAP3_UHR_OM_PU_TO_HIGH);
	timeout <<= 2;
	timeout |= u8_get_bits(cap->mac.mac_cap[2],
			       IEEE80211_UHR_MAC_CAP2_UHR_OM_PU_TO_LOW);

	if (timeout > IEEE80211_UHR_OM_PU_TO_128TU)
		return -1;

	if (!timeout)
		return 0;

	return 128 << (timeout - 1);
}

/* only valid from AP, must check ieee80211_uhr_capa_size_ok(..., true) */
static inline const struct ieee80211_uhr_cap_dbe *
ieee80211_uhr_dbe_cap(const struct ieee80211_uhr_cap *cap)
{
	if (!(cap->mac.mac_cap[1] & IEEE80211_UHR_MAC_CAP1_DBE_SUPP))
		return NULL;

	return (const void *)cap->variable;
}

enum ieee80211_protected_uhr_action {
	IEEE80211_PROTECTED_UHR_ACTION_LINK_RECONFIG_REQUEST	= 0,
	IEEE80211_PROTECTED_UHR_ACTION_LINK_RECONFIG_RESPONSE	= 1,
	IEEE80211_PROTECTED_UHR_ACTION_LINK_RECONFIG_NOTIFY	= 2,
};

enum ieee80211_uhr_link_reconfig_request_type {
	IEEE80211_UHR_LINK_RECONFIG_REQUEST_ST_PREP		= 0,
	IEEE80211_UHR_LINK_RECONFIG_REQUEST_ST_EXEC		= 1,
	IEEE80211_UHR_LINK_RECONFIG_REQUEST_OMP_REQUEST		= 3,
};

enum ieee80211_uhr_link_reconfig_response_type {
	IEEE80211_UHR_LINK_RECONFIG_RESPONSE_ST_PREP		= 0,
	IEEE80211_UHR_LINK_RECONFIG_RESPONSE_ST_EXEC		= 1,
};

enum ieee80211_uhr_link_reconfig_notify_type {
	IEEE80211_UHR_LINK_RECONFIG_NOTIFY_DL_DRAINED		= 2,
	IEEE80211_UHR_LINK_RECONFIG_NOTIFY_OMP_RESPONSE		= 3,
	IEEE80211_UHR_LINK_RECONFIG_NOTIFY_ST_UPDATE		= 5,
};

enum ieee80211_uhr_mode_change_control {
	IEEE80211_UHR_MODE_CHANGE_CONTROL_MODE_ID		= 0x3f,
	IEEE80211_UHR_MODE_CHANGE_CONTROL_MODE_ENABLE		= 0x40,
	IEEE80211_UHR_MODE_CHANGE_CONTROL_MODE_UPDATE		= 0x80,
};

enum ieee80211_uhr_mode_change_mode_length_specific_ctrl {
	IEEE80211_UHR_MODE_CHANGE_MODE_LENGTH		= 0x0f,
	IEEE80211_UHR_MODE_CHANGE_MODE_SPECIFIC_CTRL	= 0xf0,
};

enum ieee80211_uhr_mode_change_mode_id {
	IEEE80211_UHR_MODE_CHANGE_MODE_ID_DPS			= 0,
	IEEE80211_UHR_MODE_CHANGE_MODE_ID_NPCA			= 1,
	IEEE80211_UHR_MODE_CHANGE_MODE_ID_DUO			= 2,
	IEEE80211_UHR_MODE_CHANGE_MODE_ID_DSO			= 3,
	IEEE80211_UHR_MODE_CHANGE_MODE_ID_P_EDCA		= 4,
	IEEE80211_UHR_MODE_CHANGE_MODE_ID_ELR_RX		= 5,
	IEEE80211_UHR_MODE_CHANGE_MODE_ID_AOM			= 6,
	IEEE80211_UHR_MODE_CHANGE_MODE_ID_LLI			= 7,
	IEEE80211_UHR_MODE_CHANGE_MODE_ID_CO_BF			= 8,
	IEEE80211_UHR_MODE_CHANGE_MODE_ID_CO_SR			= 9,
	IEEE80211_UHR_MODE_CHANGE_MODE_ID_EMLSR			= 10,
	IEEE80211_UHR_MODE_CHANGE_MODE_ID_DBE			= 11,
};

struct ieee80211_uhr_mode_change_tuple {
	u8 control;
	u8 variable[];
} __packed;

static inline const struct ieee80211_uhr_mode_change_tuple *
ieee80211_uhr_mode_change_tuple_next(const struct ieee80211_uhr_mode_change_tuple *tuple,
				     size_t remaining)
{
	size_t needed = sizeof(*tuple);

	if (remaining < needed)
		return NULL;

	if (tuple->control & IEEE80211_UHR_MODE_CHANGE_CONTROL_MODE_ENABLE) {
		needed += 1;
		if (remaining < needed)
			return NULL;

		needed += u8_get_bits(tuple->variable[0],
				      IEEE80211_UHR_MODE_CHANGE_MODE_LENGTH);
		if (remaining < needed)
			return NULL;
	}

	return (const void *)((const u8 *)tuple + needed);
}

#define for_each_uhr_mode_change_tuple(data, len, tuple)		\
	for (tuple = (const void *)(data);				\
	     ieee80211_uhr_mode_change_tuple_next(			\
			tuple, (len) - ((const u8 *)tuple - (data)));	\
	     tuple = ieee80211_uhr_mode_change_tuple_next(		\
			tuple, (len) - ((const u8 *)tuple - (data))))

struct ieee80211_uhr_parameters_update {
	u8 countdown;
	u8 tuples[];
} __packed;

enum ieee80211_uhr_param_update_tuple_control {
	IEEE80211_UHR_PARAM_UPDATE_MODE_CTRL_MODE_ID		= 0x3f,
	IEEE80211_UHR_PARAM_UPDATE_MODE_CTRL_MODE_ENABLE	= 0x40,
	IEEE80211_UHR_PARAM_UPDATE_MODE_CTRL_MODE_UPDATE	= 0x80,
};

enum ieee80211_uhr_param_update_mode_id {
	IEEE80211_UHR_PARAM_UPDATE_MODE_ID_DPS			= 0,
	IEEE80211_UHR_PARAM_UPDATE_MODE_ID_NPCA			= 1,
	IEEE80211_UHR_PARAM_UPDATE_MODE_ID_DUO			= 2,
	IEEE80211_UHR_PARAM_UPDATE_MODE_ID_P_EDCA		= 3,
	IEEE80211_UHR_PARAM_UPDATE_MODE_ID_DBE			= 4,
	IEEE80211_UHR_PARAM_UPDATE_MODE_ID_AP_PUO		= 5,
	IEEE80211_UHR_PARAM_UPDATE_MODE_ID_ELR_RX		= 6,

	NUM_IEEE80211_UHR_PARAM_UPDATE_MODE_IDS
};

struct ieee80211_uhr_param_update_tuple {
	u8 control;
	/* mode length followed by mode specific */
	u8 variable[];
} __packed;

static inline int
ieee80211_uhr_param_update_tuple_size(const struct ieee80211_uhr_param_update_tuple *tuple)
{
	if (!(tuple->control & IEEE80211_UHR_PARAM_UPDATE_MODE_CTRL_MODE_ENABLE))
		return sizeof(*tuple);

	/* fixed size (control) + length field + value of length field */
	return sizeof(*tuple) + 1 + tuple->variable[0];
}

/* this also serves as a type-check for the macro */
static inline size_t
ieee80211_uhr_param_upd_tuples_rem(const struct ieee80211_uhr_parameters_update *upd,
				   size_t len,
				   const struct ieee80211_uhr_param_update_tuple *tuple)
{
	return len - ((const u8 *)tuple - (const u8 *)upd);
}

#define for_each_uhr_param_update_tuple(elem, len, tuple)		\
	for (tuple = (const void *)((elem) ? (elem)->tuples : NULL);	\
	     (elem) &&							\
	     ieee80211_uhr_param_upd_tuples_rem(elem, len, tuple) >=	\
		sizeof(*tuple) &&					\
	     ieee80211_uhr_param_upd_tuples_rem(elem, len, tuple) >=	\
		sizeof(*tuple) +					\
		!!(tuple->control &					\
		   IEEE80211_UHR_PARAM_UPDATE_MODE_CTRL_MODE_ENABLE) &&	\
	     ieee80211_uhr_param_upd_tuples_rem(elem, len, tuple) >=	\
		ieee80211_uhr_param_update_tuple_size(tuple);		\
	     tuple = (const void *)((const u8 *)tuple +			\
				    ieee80211_uhr_param_update_tuple_size(tuple)))

static inline bool
ieee80211_uhr_param_update_ok(const void *elem, size_t len)
{
	const struct ieee80211_uhr_parameters_update *upd = elem;
	const struct ieee80211_uhr_param_update_tuple *tuple;

	/* the loop checks, but also check here for our after-the-loop check */
	if (len < sizeof(*upd) + sizeof(*tuple))
		return false;

	for_each_uhr_param_update_tuple(upd, len, tuple) {
		u8 params_len;

		/* no further checks for disabled modes */
		if (!(tuple->control & IEEE80211_UHR_PARAM_UPDATE_MODE_CTRL_MODE_ENABLE))
			continue;

		params_len = tuple->variable[0];

		switch (u8_get_bits(tuple->control,
				    IEEE80211_UHR_PARAM_UPDATE_MODE_CTRL_MODE_ID)) {
		case IEEE80211_UHR_PARAM_UPDATE_MODE_ID_DPS:
			if (params_len < sizeof(struct ieee80211_uhr_dps_info))
				return false;
			break;
		case IEEE80211_UHR_PARAM_UPDATE_MODE_ID_NPCA:
			if (params_len < sizeof(struct ieee80211_uhr_npca_info))
				return false;
			if (params_len <
			    ieee80211_uhr_npca_info_size((const void *)(tuple->variable + 1)))
				return false;
			break;
		case IEEE80211_UHR_PARAM_UPDATE_MODE_ID_DUO:
			/*
			 * This one is defined very strangely in the spec,
			 * having a reserved (!) enable bit and always
			 * including the parameters. Let's hope it'll get
			 * unified with the others and implement that.
			 */
			if (params_len < sizeof(struct ieee80211_uhr_duo_info))
				return false;
			break;
		case IEEE80211_UHR_PARAM_UPDATE_MODE_ID_P_EDCA:
			if (params_len < sizeof(struct ieee80211_uhr_p_edca_info))
				return false;
			break;
		case IEEE80211_UHR_PARAM_UPDATE_MODE_ID_DBE:
			if (params_len < sizeof(struct ieee80211_uhr_dbe_info))
				return false;
			if (params_len <
			    ieee80211_uhr_dbe_info_size((const void *)(tuple->variable + 1)))
				return false;
			break;
		case IEEE80211_UHR_PARAM_UPDATE_MODE_ID_AP_PUO:
		case IEEE80211_UHR_PARAM_UPDATE_MODE_ID_ELR_RX:
			/* no further length checks */
			break;
		}
	}

	/* valid only if the whole data was consumed by tuples */
	return (const u8 *)tuple == (const u8 *)elem + len;
}

struct ieee80211_smd_bss_trans_scs_list {
	u8 n_ids;
	u8 ids[];
} __packed;

static inline bool
ieee80211_smd_bss_trans_scs_list_ok(const u8 **pos, size_t *remaining)
{
	const struct ieee80211_smd_bss_trans_scs_list *scs = (const void *)*pos;
	size_t size;

	if (*remaining < sizeof(*scs))
		return false;

	size = sizeof(*scs) + scs->n_ids;
	if (*remaining < size)
		return false;

	*pos += size;
	*remaining -= size;
	return true;
}

enum ieee80211_st_control_prep_req {
	IEEE80211_ST_CONTROL_PREP_REQ_NO_DL_SN_TRANSFER	= 0x01,
	IEEE80211_ST_CONTROL_PREP_REQ_NO_UL_SN_TRANSFER	= 0x02,
	IEEE80211_ST_CONTROL_PREP_REQ_SCS_LIST_PRESENT	= 0x04,
};

/**
 * struct ieee80211_smd_bss_trans_params_prep_req - SMD BSS Transition Parameters
 *	element in preparation request
 * @st_control: ST control field, see &enum ieee80211_st_control_prep_req
 * @listen_interval: listen interval
 * @variable: variable (currently SCS list)
 */
struct ieee80211_smd_bss_trans_params_prep_req {
	u8 st_control;
	__le16 listen_interval;
	u8 variable[];
} __packed;

enum ieee80211_st_control_prep_resp {
	IEEE80211_ST_CONTROL_PREP_RESP_DL_BA_INFO_PRESENT	= 0x01,
	IEEE80211_ST_CONTROL_PREP_RESP_UL_BA_INFO_PRESENT	= 0x02,
	IEEE80211_ST_CONTROL_PREP_RESP_SCS_LIST_PRESENT		= 0x04,
};

/* Note: D2.0 - we expect this to change */
struct ieee80211_smd_bss_trans_ext_ba_params {
	__le16 ba_param_set;
	u8 addba_ext_param_set;
} __packed;

struct ieee80211_smd_bss_trans_ba_info {
	u8 tid_bitmap;
	struct ieee80211_smd_bss_trans_ext_ba_params params[];
} __packed;

static inline bool
ieee80211_smd_bss_trans_ba_info_ok(const u8 **pos, size_t *remaining)
{
	const struct ieee80211_smd_bss_trans_ba_info *ba = (const void *)*pos;
	size_t size;

	if (*remaining < sizeof(*ba))
		return false;

	/* one extended BA parameters entry per TID set in the bitmap */
	size = sizeof(*ba) + hweight8(ba->tid_bitmap) * sizeof(ba->params[0]);
	if (*remaining < size)
		return false;

	*pos += size;
	*remaining -= size;
	return true;
}

/**
 * struct ieee80211_smd_bss_trans_params_prep_resp - SMD BSS Transition Parameters
 *	element in preparation response
 * @st_control: ST control field, see &enum ieee80211_st_control_prep_resp
 * @variable: variable (DL BA info, UL BA info, SCS list)
 */
struct ieee80211_smd_bss_trans_params_prep_resp {
	u8 st_control;
	u8 variable[];
} __packed;

static inline bool
ieee80211_smd_prep_resp_size_ok(const u8 *data, u8 len)
{
	const struct ieee80211_smd_bss_trans_params_prep_resp *params =
		(const void *)data;
	const u8 *pos = params->variable;
	size_t remaining;

	if (len < sizeof(*params))
		return false;
	remaining = len - sizeof(*params);

	/* DL BA info, UL BA info and the SCS list follow in order, if present */
	if ((params->st_control &
	     IEEE80211_ST_CONTROL_PREP_RESP_DL_BA_INFO_PRESENT) &&
	    !ieee80211_smd_bss_trans_ba_info_ok(&pos, &remaining))
		return false;

	if ((params->st_control &
	     IEEE80211_ST_CONTROL_PREP_RESP_UL_BA_INFO_PRESENT) &&
	    !ieee80211_smd_bss_trans_ba_info_ok(&pos, &remaining))
		return false;

	if ((params->st_control &
	     IEEE80211_ST_CONTROL_PREP_RESP_SCS_LIST_PRESENT) &&
	    !ieee80211_smd_bss_trans_scs_list_ok(&pos, &remaining))
		return false;

	return true;
}

/**
 * ieee80211_smd_prep_resp_ba_info - get per-TID Block Ack info from a
 *	preparation response element
 * @data: the SMD BSS Transition Parameters (preparation response) element data,
 *	which must already have passed ieee80211_smd_prep_resp_size_ok()
 * @uplink: %true for the uplink BA info, %false for the downlink BA info
 * @tid: the TID to look up
 *
 * Return: a pointer to the extended BA parameters for @tid in the requested
 *	direction, or %NULL if that direction is not present or @tid is not set
 *	in its TID bitmap.
 */
static inline const struct ieee80211_smd_bss_trans_ext_ba_params *
ieee80211_smd_prep_resp_ba_info(const u8 *data, bool uplink, u8 tid)
{
	const struct ieee80211_smd_bss_trans_params_prep_resp *params =
		(const void *)data;
	const struct ieee80211_smd_bss_trans_ba_info *ba;
	const u8 *pos = params->variable;

	if (uplink) {
		if (!(params->st_control &
		      IEEE80211_ST_CONTROL_PREP_RESP_UL_BA_INFO_PRESENT))
			return NULL;

		/* the UL BA info follows the DL BA info, if present */
		if (params->st_control &
		    IEEE80211_ST_CONTROL_PREP_RESP_DL_BA_INFO_PRESENT) {
			ba = (const void *)pos;
			pos += sizeof(*ba) +
			       hweight8(ba->tid_bitmap) * sizeof(ba->params[0]);
		}
	} else if (!(params->st_control &
		     IEEE80211_ST_CONTROL_PREP_RESP_DL_BA_INFO_PRESENT)) {
		return NULL;
	}

	ba = (const void *)pos;

	if (!(ba->tid_bitmap & BIT(tid)))
		return NULL;

	/* entries are ordered by TID, one per bit set below @tid */
	return &ba->params[hweight8(ba->tid_bitmap & (BIT(tid) - 1))];
}

enum ieee80211_st_control_exec_req {
	IEEE80211_ST_CONTROL_EXEC_REQ_DL_TID_BITMAP_PRESENT	= 0x01,
};

/**
 * struct ieee80211_smd_bss_trans_params_exec_req - SMD BSS Transition Parameters
 *	element in execution request
 * @st_control: ST control field, see &enum ieee80211_st_control_exec_req
 * @variable: variable (currently DL TID bitmap)
 */
struct ieee80211_smd_bss_trans_params_exec_req {
	u8 st_control;
	u8 variable[];
} __packed;

enum ieee80211_st_control_exec_resp {
	IEEE80211_ST_CONTROL_EXEC_RESP_NOM_MAX_DL_DRAIN_PRESENT	= 0x01,
	IEEE80211_ST_CONTROL_EXEC_RESP_UL_SN_LIST_PRESENT	= 0x02,
};

struct ieee80211_smd_bss_trans_ul_sn_list {
	u8 tid_bitmap;
	/*
	 * a list of 12-bit SNs for each TID in the bitmap,
	 * then rounded up to be a full number of octets
	 */
	u8 variable[];
} __packed;

static inline bool
ieee80211_smd_bss_trans_ul_sn_list_ok(const u8 **pos, size_t *remaining)
{
	const struct ieee80211_smd_bss_trans_ul_sn_list *sn =
		(const void *)*pos;
	size_t size;

	if (*remaining < sizeof(*sn))
		return false;

	/* one 12-bit SN per TID set in the bitmap, padded to whole octets */
	size = sizeof(*sn) + DIV_ROUND_UP(hweight8(sn->tid_bitmap) * 12, 8);
	if (*remaining < size)
		return false;

	*pos += size;
	*remaining -= size;
	return true;
}

/**
 * struct ieee80211_smd_bss_trans_params_exec_resp - SMD BSS Transition Parameters
 *	element in execution response
 * @st_control: ST control field, see &enum ieee80211_st_control_exec_resp
 * @variable: variable (nominal maximum DL draining period duration,
 *	Latest UL SN List)
 */
struct ieee80211_smd_bss_trans_params_exec_resp {
	u8 st_control;
	u8 variable[];
} __packed;

static inline bool
ieee80211_smd_exec_resp_size_ok(const u8 *data, u8 len)
{
	const struct ieee80211_smd_bss_trans_params_exec_resp *params =
		(const void *)data;
	const u8 *pos = params->variable;
	size_t remaining;

	if (len < sizeof(*params))
		return false;
	remaining = len - sizeof(*params);

	/* nominal maximum DL draining period duration */
	if (params->st_control &
	    IEEE80211_ST_CONTROL_EXEC_RESP_NOM_MAX_DL_DRAIN_PRESENT) {
		if (remaining < sizeof(__le16))
			return false;
		pos += sizeof(__le16);
		remaining -= sizeof(__le16);
	}

	/* Latest UL SN List */
	if ((params->st_control &
	     IEEE80211_ST_CONTROL_EXEC_RESP_UL_SN_LIST_PRESENT) &&
	    !ieee80211_smd_bss_trans_ul_sn_list_ok(&pos, &remaining))
		return false;

	return true;
}

/**
 * ieee80211_smd_exec_resp_ul_sn - get a per-TID uplink SN from an execution
 *	response element
 * @data: the SMD BSS Transition Parameters (execution response) element data,
 *	which must already have passed ieee80211_smd_exec_resp_size_ok()
 * @tid: the TID to look up
 *
 * Return: the 12-bit uplink sequence number for @tid, or -1 if the UL SN list
 *	is not present or @tid is not set in its TID bitmap.
 */
static inline int
ieee80211_smd_exec_resp_ul_sn(const u8 *data, u8 tid)
{
	const struct ieee80211_smd_bss_trans_params_exec_resp *params =
		(const void *)data;
	const struct ieee80211_smd_bss_trans_ul_sn_list *sn;
	const u8 *pos = params->variable;
	unsigned int offset, bit_offset;

	if (!(params->st_control &
	      IEEE80211_ST_CONTROL_EXEC_RESP_UL_SN_LIST_PRESENT))
		return -1;

	/* the UL SN list follows the nominal maximum DL draining duration */
	if (params->st_control &
	    IEEE80211_ST_CONTROL_EXEC_RESP_NOM_MAX_DL_DRAIN_PRESENT)
		pos += sizeof(__le16);

	sn = (const void *)pos;

	if (!(sn->tid_bitmap & BIT(tid)))
		return -1;

	/* 12-bit SNs, ordered by TID, packed least-significant-bit first */
	offset = hweight8(sn->tid_bitmap & (BIT(tid) - 1));
	bit_offset = offset * 12;

	return (get_unaligned_le16(&sn->variable[bit_offset / 8]) >>
		(bit_offset % 8)) & 0xfff;
}

enum ieee80211_dl_data_drain_info_flags {
	IEEE80211_DL_DATA_DRAIN_INFO_DRAIN_COMPLETED		= 0x01,
	IEEE80211_DL_DATA_DRAIN_INFO_PER_TID_DL_BITMAP_PRESENT	= 0x02,
};

/**
 * struct ieee80211_dl_data_drain_info - DL Data Drain Info field
 * @flags: the info field with flags from &enum ieee80211_dl_data_drain_info_flags
 * @variable: variable (Per-TID DL bitmap)
 */
struct ieee80211_dl_data_drain_info {
	u8 flags;
	u8 variable[];
} __packed;

#endif /* LINUX_IEEE80211_UHR_H */
