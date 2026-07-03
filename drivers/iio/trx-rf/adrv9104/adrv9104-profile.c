// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADRV9104 device profile handling.
 *
 * The device profile (deviceProfileBundle_t: the profile plus its PFIR
 * coefficient banks) is provided as a Configurator JSON blob. This file parses
 * that JSON into the packed bundle and serialises it back, using a table-driven
 * schema where every JSON key is bound to its struct member via
 * offsetof()/sizeof(). A default profile is loaded from firmware at probe and
 * new profiles are pushed at runtime through the firmware-upload interface.
 */
#include <linux/array_size.h>
#include <linux/base64.h>
#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/compiler.h>
#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/kstrtox.h>
#include <linux/math.h>
#include <linux/slab.h>
#include <linux/sprintf.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include <linux/iio/iio.h>

#include "adrv9104.h"
#include "adrv9104-profile.h"

#include "device_profile_bundle_t.h"

enum adrv9104_profile_ftype {
	ADRV9104_PROFILE_INT,
	ADRV9104_PROFILE_BOOL,
	ADRV9104_PROFILE_BYTES,
	ADRV9104_PROFILE_INT_ARRAY,
	ADRV9104_PROFILE_STRUCT,
	ADRV9104_PROFILE_STRUCT_ARRAY,
	ADRV9104_PROFILE_SKIP,
};

struct adrv9104_profile_schema {
	const struct adrv9104_profile_field *fields;
	unsigned int nfields;
};

struct adrv9104_profile_field {
	const char *name;
	unsigned short offset;
	unsigned char type;
	unsigned char is_signed;
	unsigned char elem_size;
	unsigned short count;
	unsigned short stride;
	const struct adrv9104_profile_schema *sub;
};

#define ADRV9104_FIELD_INT(TYPE, MEMBER) {				\
	.name = #MEMBER, .offset = offsetof(TYPE, MEMBER),		\
	.type = ADRV9104_PROFILE_INT,					\
	.is_signed = is_signed_type(__typeof__(((TYPE *)0)->MEMBER)),	\
	.elem_size = sizeof(((TYPE *)0)->MEMBER), .count = 1 }

#define ADRV9104_FIELD_BOOL(TYPE, MEMBER) {				\
	.name = #MEMBER, .offset = offsetof(TYPE, MEMBER),		\
	.type = ADRV9104_PROFILE_BOOL,					\
	.elem_size = sizeof(((TYPE *)0)->MEMBER), .count = 1 }

/*
 * uint8_t[] serialises as a base64 string; every other scalar array serialises
 * as a JSON number array. The choice is made from the element size.
 */
#define ADRV9104_FIELD_ARRAY(TYPE, MEMBER) {				\
	.name = #MEMBER, .offset = offsetof(TYPE, MEMBER),		\
	.type = (sizeof(((TYPE *)0)->MEMBER[0]) == 1) ?			\
		ADRV9104_PROFILE_BYTES : ADRV9104_PROFILE_INT_ARRAY,	\
	.is_signed = is_signed_type(__typeof__(((TYPE *)0)->MEMBER[0])),	\
	.elem_size = sizeof(((TYPE *)0)->MEMBER[0]),			\
	.count = ARRAY_SIZE(((TYPE *)0)->MEMBER) }

#define ADRV9104_FIELD_STRUCT(TYPE, MEMBER, SUB) {			\
	.name = #MEMBER, .offset = offsetof(TYPE, MEMBER),		\
	.type = ADRV9104_PROFILE_STRUCT, .sub = &(SUB) }

#define ADRV9104_FIELD_STRUCT_ARRAY(TYPE, MEMBER, SUB) {		\
	.name = #MEMBER, .offset = offsetof(TYPE, MEMBER),		\
	.type = ADRV9104_PROFILE_STRUCT_ARRAY, .sub = &(SUB),		\
	.stride = sizeof(((TYPE *)0)->MEMBER[0]),			\
	.count = ARRAY_SIZE(((TYPE *)0)->MEMBER) }

#define ADRV9104_FIELD_SKIP(NAME) { .name = NAME, .type = ADRV9104_PROFILE_SKIP }

#define ADRV9104_SCHEMA(FIELDS) { .fields = (FIELDS), .nfields = ARRAY_SIZE(FIELDS) }

static const struct adrv9104_profile_field adrv9104_ver_fields[] = {
	ADRV9104_FIELD_INT(profileVersion_t, major),
	ADRV9104_FIELD_INT(profileVersion_t, minor),
};

static const struct adrv9104_profile_schema adrv9104_sch_ver =
	ADRV9104_SCHEMA(adrv9104_ver_fields);

static const struct adrv9104_profile_field adrv9104_lo_config_fields[] = {
	ADRV9104_FIELD_INT(loConfig_t, extLoMode),
	ADRV9104_FIELD_INT(loConfig_t, extLoPllConnect),
	ADRV9104_FIELD_INT(loConfig_t, extLoDiv),
};

static const struct adrv9104_profile_schema adrv9104_sch_lo_config =
	ADRV9104_SCHEMA(adrv9104_lo_config_fields);

static const struct adrv9104_profile_field adrv9104_ssi_fields[] = {
	ADRV9104_FIELD_INT(ssiConfig_t, ssiType),
	ADRV9104_FIELD_INT(ssiConfig_t, ssiDataFormatSel),
	ADRV9104_FIELD_INT(ssiConfig_t, numLaneSel),
	ADRV9104_FIELD_INT(ssiConfig_t, strobeType),
	ADRV9104_FIELD_INT(ssiConfig_t, lsbFirst),
	ADRV9104_FIELD_INT(ssiConfig_t, qFirst),
	ADRV9104_FIELD_INT(ssiConfig_t, txRefClockPin),
	ADRV9104_FIELD_INT(ssiConfig_t, lvdsBitInversion),
	ADRV9104_FIELD_INT(ssiConfig_t, lvdsUseLsbIn12bitMode),
	ADRV9104_FIELD_INT(ssiConfig_t, rxSsiDataSrc),
	ADRV9104_FIELD_INT(ssiConfig_t, txDataSrc),
	ADRV9104_FIELD_INT(ssiConfig_t, lvdsRxClkInversionEn),
	ADRV9104_FIELD_INT(ssiConfig_t, cmosTxDdrNegStrobeEn),
	ADRV9104_FIELD_INT(ssiConfig_t, cmosDdrPosClkEn),
	ADRV9104_FIELD_INT(ssiConfig_t, cmosClkInversionEn),
	ADRV9104_FIELD_INT(ssiConfig_t, ddrEn),
	ADRV9104_FIELD_INT(ssiConfig_t, rxMaskStrobeEn),
	ADRV9104_FIELD_ARRAY(ssiConfig_t, padding),
};

static const struct adrv9104_profile_schema adrv9104_sch_ssi =
	ADRV9104_SCHEMA(adrv9104_ssi_fields);

static const struct adrv9104_profile_field adrv9104_tx_bbf_fields[] = {
	ADRV9104_FIELD_INT(txBbfConfig_t, butterFilterBw),
	ADRV9104_FIELD_INT(txBbfConfig_t, txBbfPowerMode),
	ADRV9104_FIELD_ARRAY(txBbfConfig_t, padding),
};

static const struct adrv9104_profile_schema adrv9104_sch_tx_bbf =
	ADRV9104_SCHEMA(adrv9104_tx_bbf_fields);

static const struct adrv9104_profile_field adrv9104_tx_predist_fields[] = {
	ADRV9104_FIELD_INT(txPredistConfig_t, pdBiasCurrent),
	ADRV9104_FIELD_INT(txPredistConfig_t, pdGainEn),
	ADRV9104_FIELD_INT(txPredistConfig_t, prePDRealPole),
	ADRV9104_FIELD_INT(txPredistConfig_t, postPDRealPole),
	ADRV9104_FIELD_ARRAY(txPredistConfig_t, padding),
};

static const struct adrv9104_profile_schema adrv9104_sch_tx_predist =
	ADRV9104_SCHEMA(adrv9104_tx_predist_fields);

static const struct adrv9104_profile_field adrv9104_tx_elb_fields[] = {
	ADRV9104_FIELD_INT(txElbConfig_t, extLoopBackType),
	ADRV9104_FIELD_INT(txElbConfig_t, extLoopBackForInitCal),
	ADRV9104_FIELD_INT(txElbConfig_t, peakLoopBackPower),
};

static const struct adrv9104_profile_schema adrv9104_sch_tx_elb =
	ADRV9104_SCHEMA(adrv9104_tx_elb_fields);

static const struct adrv9104_profile_field adrv9104_tx_config_fields[] = {
	ADRV9104_FIELD_INT(txConfig_t, primaryBw_Hz),
	ADRV9104_FIELD_INT(txConfig_t, txInputRate),
	ADRV9104_FIELD_INT(txConfig_t, txInterfaceSampleRate),
	ADRV9104_FIELD_INT(txConfig_t, offsetLo_Hz),
	ADRV9104_FIELD_INT(txConfig_t, outputSignalingSel),
	ADRV9104_FIELD_STRUCT(txConfig_t, txBbfConfig, adrv9104_sch_tx_bbf),
	ADRV9104_FIELD_STRUCT(txConfig_t, txPredistConfig, adrv9104_sch_tx_predist),
	ADRV9104_FIELD_STRUCT(txConfig_t, txElbConfig, adrv9104_sch_tx_elb),
	ADRV9104_FIELD_INT(txConfig_t, freqDev),
	ADRV9104_FIELD_STRUCT(txConfig_t, txSsiConfig, adrv9104_sch_ssi),
	ADRV9104_FIELD_INT(txConfig_t, LoSelect),
	ADRV9104_FIELD_ARRAY(txConfig_t, padding),
};

static const struct adrv9104_profile_schema adrv9104_sch_tx_config =
	ADRV9104_SCHEMA(adrv9104_tx_config_fields);

static const struct adrv9104_profile_field adrv9104_rx_config_fields[] = {
	ADRV9104_FIELD_INT(rxConfig_t, primaryBw_Hz),
	ADRV9104_FIELD_INT(rxConfig_t, rxOutputRate),
	ADRV9104_FIELD_INT(rxConfig_t, rxInterfaceSampleRate),
	ADRV9104_FIELD_INT(rxConfig_t, offsetLo_Hz),
	ADRV9104_FIELD_INT(rxConfig_t, ncoEn),
	ADRV9104_FIELD_INT(rxConfig_t, outputSignalingSel),
	ADRV9104_FIELD_INT(rxConfig_t, adcCorner),
	ADRV9104_FIELD_INT(rxConfig_t, adcClk_Hz),
	ADRV9104_FIELD_INT(rxConfig_t, corner3dB),
	ADRV9104_FIELD_INT(rxConfig_t, tiaPower),
	ADRV9104_FIELD_STRUCT(rxConfig_t, rxSsiConfig, adrv9104_sch_ssi),
	ADRV9104_FIELD_INT(rxConfig_t, gainTableType),
	ADRV9104_FIELD_INT(rxConfig_t, LoSelect),
};

static const struct adrv9104_profile_schema adrv9104_sch_rx_config =
	ADRV9104_SCHEMA(adrv9104_rx_config_fields);

static const struct adrv9104_profile_field adrv9104_nco_fields[] = {
	ADRV9104_FIELD_INT(ncoDpConfig_t, freq),
	ADRV9104_FIELD_INT(ncoDpConfig_t, sampleFreq),
	ADRV9104_FIELD_INT(ncoDpConfig_t, phase),
	ADRV9104_FIELD_INT(ncoDpConfig_t, realOut),
	ADRV9104_FIELD_INT(ncoDpConfig_t, freeRunClkEnable),
};

static const struct adrv9104_profile_schema adrv9104_sch_nco =
	ADRV9104_SCHEMA(adrv9104_nco_fields);

static const struct adrv9104_profile_field adrv9104_tx_pre_proc_fields[] = {
	ADRV9104_FIELD_INT(txPreProc_t, symMapSymbol0),
	ADRV9104_FIELD_INT(txPreProc_t, symMapSymbol1),
	ADRV9104_FIELD_INT(txPreProc_t, symMapSymbol2),
	ADRV9104_FIELD_INT(txPreProc_t, symMapSymbol3),
	ADRV9104_FIELD_INT(txPreProc_t, symMapDivFactor),
	ADRV9104_FIELD_INT(txPreProc_t, symMapQVal),
	ADRV9104_FIELD_INT(txPreProc_t, preProcMode),
	ADRV9104_FIELD_INT(txPreProc_t, symMapperMode),
	ADRV9104_FIELD_INT(txPreProc_t, txPreProcWbNbPfirIBank),
	ADRV9104_FIELD_INT(txPreProc_t, txPreProcWbNbPfirQBank),
	ADRV9104_FIELD_ARRAY(txPreProc_t, padding),
};

static const struct adrv9104_profile_schema adrv9104_sch_tx_pre_proc =
	ADRV9104_SCHEMA(adrv9104_tx_pre_proc_fields);

static const struct adrv9104_profile_field adrv9104_tx_wb_int_top_fields[] = {
	ADRV9104_FIELD_INT(txWbIntTop_t, interpBy2Blk30En),
	ADRV9104_FIELD_INT(txWbIntTop_t, interpBy2Blk28En),
	ADRV9104_FIELD_INT(txWbIntTop_t, interpBy2Blk26En),
	ADRV9104_FIELD_INT(txWbIntTop_t, interpBy2Blk24En),
	ADRV9104_FIELD_INT(txWbIntTop_t, interpBy2Blk22En),
	ADRV9104_FIELD_INT(txWbIntTop_t, wbLpfBlk22p1En),
	ADRV9104_FIELD_ARRAY(txWbIntTop_t, padding),
};

static const struct adrv9104_profile_schema adrv9104_sch_tx_wb_int_top =
	ADRV9104_SCHEMA(adrv9104_tx_wb_int_top_fields);

static const struct adrv9104_profile_field adrv9104_tx_nb_int_top_fields[] = {
	ADRV9104_FIELD_INT(txNbIntTop_t, interpBy2Blk20En),
	ADRV9104_FIELD_INT(txNbIntTop_t, interpBy2Blk18En),
	ADRV9104_FIELD_INT(txNbIntTop_t, interpBy2Blk16En),
	ADRV9104_FIELD_INT(txNbIntTop_t, interpBy2Blk14En),
	ADRV9104_FIELD_INT(txNbIntTop_t, interpBy2Blk12En),
	ADRV9104_FIELD_INT(txNbIntTop_t, interpBy3Blk10En),
	ADRV9104_FIELD_INT(txNbIntTop_t, interpBy2Blk8En),
	ADRV9104_FIELD_INT(txNbIntTop_t, scicBlk32En),
	ADRV9104_FIELD_INT(txNbIntTop_t, scicBlk32DivFactor),
	ADRV9104_FIELD_ARRAY(txNbIntTop_t, padding),
};

static const struct adrv9104_profile_schema adrv9104_sch_tx_nb_int_top =
	ADRV9104_SCHEMA(adrv9104_tx_nb_int_top_fields);

static const struct adrv9104_profile_field adrv9104_tx_nb_int_top_new_fields[] = {
	ADRV9104_FIELD_INT(txNbIntTopNew_t, interpBy5_2En),
	ADRV9104_FIELD_INT(txNbIntTopNew_t, interpBy5_1En),
	ADRV9104_FIELD_INT(txNbIntTopNew_t, interpBy3_1En),
	ADRV9104_FIELD_INT(txNbIntTopNew_t, interpBy2_6En),
	ADRV9104_FIELD_INT(txNbIntTopNew_t, interpBy2_5En),
	ADRV9104_FIELD_INT(txNbIntTopNew_t, interpBy2_4En),
	ADRV9104_FIELD_INT(txNbIntTopNew_t, interpBy2_3En),
	ADRV9104_FIELD_INT(txNbIntTopNew_t, interpBy2_2En),
	ADRV9104_FIELD_INT(txNbIntTopNew_t, interpBy2_1En),
	ADRV9104_FIELD_INT(txNbIntTopNew_t, scicNbBlk32En),
	ADRV9104_FIELD_INT(txNbIntTopNew_t, scicNbBlk32DivFactor),
	ADRV9104_FIELD_INT(txNbIntTopNew_t, padding),
};

static const struct adrv9104_profile_schema adrv9104_sch_tx_nb_int_top_new =
	ADRV9104_SCHEMA(adrv9104_tx_nb_int_top_new_fields);

static const struct adrv9104_profile_field adrv9104_tx_int_top_fields[] = {
	ADRV9104_FIELD_INT(txIntTop_t, interpBy3Blk44p1En),
	ADRV9104_FIELD_INT(txIntTop_t, sinc3Blk44En),
	ADRV9104_FIELD_INT(txIntTop_t, sinc2Blk42En),
	ADRV9104_FIELD_INT(txIntTop_t, interpBy3Blk40En),
	ADRV9104_FIELD_INT(txIntTop_t, interpBy3Blk40p1En),
	ADRV9104_FIELD_INT(txIntTop_t, interpBy2Blk38En),
	ADRV9104_FIELD_INT(txIntTop_t, interpBy2Blk36En),
	ADRV9104_FIELD_INT(txIntTop_t, interpBy2BlkEn),
};

static const struct adrv9104_profile_schema adrv9104_sch_tx_int_top =
	ADRV9104_SCHEMA(adrv9104_tx_int_top_fields);

static const struct adrv9104_profile_field adrv9104_tx_int_top_freq_dev_map_fields[] = {
	ADRV9104_FIELD_INT(txIntTopFreqDevMap_t, rrc2Frac),
	ADRV9104_FIELD_INT(txIntTopFreqDevMap_t, mpll),
	ADRV9104_FIELD_INT(txIntTopFreqDevMap_t, nchLsw),
	ADRV9104_FIELD_INT(txIntTopFreqDevMap_t, nchMsb),
	ADRV9104_FIELD_INT(txIntTopFreqDevMap_t, freqDevMapEn),
	ADRV9104_FIELD_INT(txIntTopFreqDevMap_t, txRoundEn),
	ADRV9104_FIELD_ARRAY(txIntTopFreqDevMap_t, padding),
};

static const struct adrv9104_profile_schema adrv9104_sch_tx_int_top_freq_dev_map =
	ADRV9104_SCHEMA(adrv9104_tx_int_top_freq_dev_map_fields);

static const struct adrv9104_profile_field adrv9104_tx_iqdm_duc_fields[] = {
	ADRV9104_FIELD_INT(txIqdmDuc_t, iqdmDucMode),
	ADRV9104_FIELD_ARRAY(txIqdmDuc_t, padding),
	ADRV9104_FIELD_INT(txIqdmDuc_t, iqdmDev),
	ADRV9104_FIELD_INT(txIqdmDuc_t, iqdmDevOffset),
	ADRV9104_FIELD_INT(txIqdmDuc_t, iqdmScalar),
	ADRV9104_FIELD_INT(txIqdmDuc_t, iqdmThreshold),
	ADRV9104_FIELD_STRUCT(txIqdmDuc_t, iqdmNco, adrv9104_sch_nco),
};

static const struct adrv9104_profile_schema adrv9104_sch_tx_iqdm_duc =
	ADRV9104_SCHEMA(adrv9104_tx_iqdm_duc_fields);

static const struct adrv9104_profile_field adrv9104_tx_dp_profile_fields[] = {
	ADRV9104_FIELD_STRUCT(txDpProfile_t, preProc, adrv9104_sch_tx_pre_proc),
	ADRV9104_FIELD_STRUCT(txDpProfile_t, wbIntTop, adrv9104_sch_tx_wb_int_top),
	ADRV9104_FIELD_STRUCT(txDpProfile_t, nbIntTop, adrv9104_sch_tx_nb_int_top),
	ADRV9104_FIELD_STRUCT(txDpProfile_t, intTop, adrv9104_sch_tx_int_top),
	ADRV9104_FIELD_STRUCT(txDpProfile_t, intTopFreqDevMap,
			      adrv9104_sch_tx_int_top_freq_dev_map),
	ADRV9104_FIELD_STRUCT(txDpProfile_t, iqdmDuc, adrv9104_sch_tx_iqdm_duc),
};

static const struct adrv9104_profile_schema adrv9104_sch_tx_dp_profile =
	ADRV9104_SCHEMA(adrv9104_tx_dp_profile_fields);

static const struct adrv9104_profile_field adrv9104_tx1_config_fields[] = {
	ADRV9104_FIELD_STRUCT(tx1Config_t, txDpProfile, adrv9104_sch_tx_dp_profile),
	ADRV9104_FIELD_ARRAY(tx1Config_t, reserved),
};

static const struct adrv9104_profile_schema adrv9104_sch_tx1_config =
	ADRV9104_SCHEMA(adrv9104_tx1_config_fields);

static const struct adrv9104_profile_field adrv9104_tx_nb_dp_profile_fields[] = {
	ADRV9104_FIELD_STRUCT(txNbDpProfile_t, preProc, adrv9104_sch_tx_pre_proc),
	ADRV9104_FIELD_STRUCT(txNbDpProfile_t, nbIntTopNew, adrv9104_sch_tx_nb_int_top_new),
	ADRV9104_FIELD_STRUCT(txNbDpProfile_t, intTop, adrv9104_sch_tx_int_top),
	ADRV9104_FIELD_STRUCT(txNbDpProfile_t, intTopFreqDevMap,
			      adrv9104_sch_tx_int_top_freq_dev_map),
	ADRV9104_FIELD_STRUCT(txNbDpProfile_t, iqdmDuc, adrv9104_sch_tx_iqdm_duc),
};

static const struct adrv9104_profile_schema adrv9104_sch_tx_nb_dp_profile =
	ADRV9104_SCHEMA(adrv9104_tx_nb_dp_profile_fields);

static const struct adrv9104_profile_field adrv9104_tx_nb_config_fields[] = {
	ADRV9104_FIELD_STRUCT(txNbConfig_t, txNbDpProfile, adrv9104_sch_tx_nb_dp_profile),
	ADRV9104_FIELD_ARRAY(txNbConfig_t, reserved),
};

static const struct adrv9104_profile_schema adrv9104_sch_tx_nb_config =
	ADRV9104_SCHEMA(adrv9104_tx_nb_config_fields);

static const struct adrv9104_profile_field adrv9104_rx_nb_dec_top_fields[] = {
	ADRV9104_FIELD_INT(rxNbDecTop_t, scicBlk23En),
	ADRV9104_FIELD_INT(rxNbDecTop_t, scicBlk23DivFactor),
	ADRV9104_FIELD_INT(rxNbDecTop_t, scicBlk23LowRippleEn),
	ADRV9104_FIELD_INT(rxNbDecTop_t, decBy2Blk35En),
	ADRV9104_FIELD_INT(rxNbDecTop_t, decBy2Blk37En),
	ADRV9104_FIELD_INT(rxNbDecTop_t, decBy2Blk39En),
	ADRV9104_FIELD_INT(rxNbDecTop_t, decBy2Blk41En),
	ADRV9104_FIELD_INT(rxNbDecTop_t, decBy2Blk43En),
	ADRV9104_FIELD_INT(rxNbDecTop_t, decBy3Blk45En),
	ADRV9104_FIELD_INT(rxNbDecTop_t, decBy2Blk47En),
	ADRV9104_FIELD_ARRAY(rxNbDecTop_t, padding),
};

static const struct adrv9104_profile_schema adrv9104_sch_rx_nb_dec_top =
	ADRV9104_SCHEMA(adrv9104_rx_nb_dec_top_fields);

static const struct adrv9104_profile_field adrv9104_rx_wb_dec_top_fields[] = {
	ADRV9104_FIELD_INT(rxWbDecTop_t, decBy2Blk25En),
	ADRV9104_FIELD_INT(rxWbDecTop_t, decBy2Blk27En),
	ADRV9104_FIELD_INT(rxWbDecTop_t, decBy2Blk29En),
	ADRV9104_FIELD_INT(rxWbDecTop_t, decBy2Blk31En),
	ADRV9104_FIELD_INT(rxWbDecTop_t, decBy2Blk33En),
	ADRV9104_FIELD_INT(rxWbDecTop_t, wbLpfBlk33p1En),
	ADRV9104_FIELD_ARRAY(rxWbDecTop_t, padding),
};

static const struct adrv9104_profile_schema adrv9104_sch_rx_wb_dec_top =
	ADRV9104_SCHEMA(adrv9104_rx_wb_dec_top_fields);

static const struct adrv9104_profile_field adrv9104_rx_dec_top_fields[] = {
	ADRV9104_FIELD_INT(rxDecTop_t, decBy3Blk15En),
	ADRV9104_FIELD_INT(rxDecTop_t, decBy3Blk15_1En),
	ADRV9104_FIELD_INT(rxDecTop_t, decBy2Hb3Blk17p1En),
	ADRV9104_FIELD_INT(rxDecTop_t, decBy2Hb4Blk17p2En),
	ADRV9104_FIELD_INT(rxDecTop_t, decBy2Hb5Blk19p1En),
	ADRV9104_FIELD_INT(rxDecTop_t, decBy2Hb6Blk19p2En),
	ADRV9104_FIELD_ARRAY(rxDecTop_t, padding),
};

static const struct adrv9104_profile_schema adrv9104_sch_rx_dec_top =
	ADRV9104_SCHEMA(adrv9104_rx_dec_top_fields);

static const struct adrv9104_profile_field adrv9104_rx_sinc_hb_top_fields[] = {
	ADRV9104_FIELD_INT(rxSincHbTop_t, sincGainMux),
	ADRV9104_FIELD_INT(rxSincHbTop_t, sincMux),
	ADRV9104_FIELD_INT(rxSincHbTop_t, hbMux),
	ADRV9104_FIELD_INT(rxSincHbTop_t, isGainCompEnabled),
	ADRV9104_FIELD_ARRAY(rxSincHbTop_t, gainComp9GainI),
	ADRV9104_FIELD_ARRAY(rxSincHbTop_t, gainComp9GainQ),
};

static const struct adrv9104_profile_schema adrv9104_sch_rx_sinc_hb_top =
	ADRV9104_SCHEMA(adrv9104_rx_sinc_hb_top_fields);

static const struct adrv9104_profile_field adrv9104_rx_wb_nb_comp_pfir_fields[] = {
	ADRV9104_FIELD_INT(rxWbNbCompPFir_t, rxWbNbCompPFirInMuxSel),
	ADRV9104_FIELD_INT(rxWbNbCompPFir_t, rxWbNbCompPFirEn),
	ADRV9104_FIELD_INT(rxWbNbCompPFir_t, rxWbNbCompPFirBankSel),
	ADRV9104_FIELD_ARRAY(rxWbNbCompPFir_t, padding),
};

static const struct adrv9104_profile_schema adrv9104_sch_rx_wb_nb_comp_pfir =
	ADRV9104_SCHEMA(adrv9104_rx_wb_nb_comp_pfir_fields);

static const struct adrv9104_profile_field adrv9104_rx_resamp_fields[] = {
	ADRV9104_FIELD_INT(rxReSampConfig_t, rxReSampEn),
	ADRV9104_FIELD_ARRAY(rxReSampConfig_t, padding),
	ADRV9104_FIELD_INT(rxReSampConfig_t, reSampPhaseI),
	ADRV9104_FIELD_INT(rxReSampConfig_t, reSampPhaseQ),
};

static const struct adrv9104_profile_schema adrv9104_sch_rx_resamp =
	ADRV9104_SCHEMA(adrv9104_rx_resamp_fields);

static const struct adrv9104_profile_field adrv9104_rx_nb_nco_fields[] = {
	ADRV9104_FIELD_INT(rxNbNcoConfig_t, rxNbNcoEn),
	ADRV9104_FIELD_ARRAY(rxNbNcoConfig_t, padding),
	ADRV9104_FIELD_STRUCT(rxNbNcoConfig_t, rxNbNcoConfig, adrv9104_sch_nco),
};

static const struct adrv9104_profile_schema adrv9104_sch_rx_nb_nco =
	ADRV9104_SCHEMA(adrv9104_rx_nb_nco_fields);

static const struct adrv9104_profile_field adrv9104_rx_nb_dem_fields[] = {
	ADRV9104_FIELD_STRUCT(rxnbDemConfig_t, rxNbNco, adrv9104_sch_rx_nb_nco),
	ADRV9104_FIELD_STRUCT(rxnbDemConfig_t, rxWbNbCompPFir, adrv9104_sch_rx_wb_nb_comp_pfir),
	ADRV9104_FIELD_STRUCT(rxnbDemConfig_t, reSamp, adrv9104_sch_rx_resamp),
	ADRV9104_FIELD_INT(rxnbDemConfig_t, gsOutMuxSel),
	ADRV9104_FIELD_INT(rxnbDemConfig_t, dnSampleSel),
	ADRV9104_FIELD_INT(rxnbDemConfig_t, rxOutSel),
	ADRV9104_FIELD_INT(rxnbDemConfig_t, rxRoundMode),
};

static const struct adrv9104_profile_schema adrv9104_sch_rx_nb_dem =
	ADRV9104_SCHEMA(adrv9104_rx_nb_dem_fields);

static const struct adrv9104_profile_field adrv9104_rx_dp_profile_fields[] = {
	ADRV9104_FIELD_STRUCT(rxDpProfile_t, nbDecTop, adrv9104_sch_rx_nb_dec_top),
	ADRV9104_FIELD_STRUCT(rxDpProfile_t, wbDecTop, adrv9104_sch_rx_wb_dec_top),
	ADRV9104_FIELD_STRUCT(rxDpProfile_t, decTop, adrv9104_sch_rx_dec_top),
	ADRV9104_FIELD_STRUCT(rxDpProfile_t, sincHbTop, adrv9104_sch_rx_sinc_hb_top),
	ADRV9104_FIELD_STRUCT(rxDpProfile_t, rxNbDem, adrv9104_sch_rx_nb_dem),
};

static const struct adrv9104_profile_schema adrv9104_sch_rx_dp_profile =
	ADRV9104_SCHEMA(adrv9104_rx_dp_profile_fields);

static const struct adrv9104_profile_field adrv9104_rx1_config_fields[] = {
	ADRV9104_FIELD_STRUCT(rx1Config_t, rxDpProfile, adrv9104_sch_rx_dp_profile),
	ADRV9104_FIELD_INT(rx1Config_t, wbSelNbDemod),
	ADRV9104_FIELD_ARRAY(rx1Config_t, reserved),
};

static const struct adrv9104_profile_schema adrv9104_sch_rx1_config =
	ADRV9104_SCHEMA(adrv9104_rx1_config_fields);

static const struct adrv9104_profile_field adrv9104_rx_nb_dec_top_nb_fields[] = {
	ADRV9104_FIELD_INT(rxNbDecTopNb_t, scicEn),
	ADRV9104_FIELD_INT(rxNbDecTopNb_t, scicDivFactor),
	ADRV9104_FIELD_INT(rxNbDecTopNb_t, rxDpScicControl),
	ADRV9104_FIELD_INT(rxNbDecTopNb_t, decBy5Blk14En),
	ADRV9104_FIELD_INT(rxNbDecTopNb_t, decBy5Blk16En),
	ADRV9104_FIELD_INT(rxNbDecTopNb_t, decBy3Blk18En),
	ADRV9104_FIELD_INT(rxNbDecTopNb_t, decBy2Blk20En),
	ADRV9104_FIELD_INT(rxNbDecTopNb_t, decBy2Blk22En),
	ADRV9104_FIELD_INT(rxNbDecTopNb_t, decBy2Blk24En),
	ADRV9104_FIELD_INT(rxNbDecTopNb_t, decBy2Blk26En),
	ADRV9104_FIELD_INT(rxNbDecTopNb_t, decBy2Blk27En),
	ADRV9104_FIELD_INT(rxNbDecTopNb_t, decBy2Blk29En),
};

static const struct adrv9104_profile_schema adrv9104_sch_rx_nb_dec_top_nb =
	ADRV9104_SCHEMA(adrv9104_rx_nb_dec_top_nb_fields);

static const struct adrv9104_profile_field adrv9104_rx_nb_sinc_hb_top_fields[] = {
	ADRV9104_FIELD_INT(rxNbSincHbTop_t, sincGainMux),
	ADRV9104_FIELD_INT(rxNbSincHbTop_t, sincMux),
	ADRV9104_FIELD_INT(rxNbSincHbTop_t, hbMux),
	ADRV9104_FIELD_INT(rxNbSincHbTop_t, isGainCompEnabled),
	ADRV9104_FIELD_ARRAY(rxNbSincHbTop_t, gainComp9GainI),
	ADRV9104_FIELD_ARRAY(rxNbSincHbTop_t, gainComp9GainQ),
};

static const struct adrv9104_profile_schema adrv9104_sch_rx_nb_sinc_hb_top =
	ADRV9104_SCHEMA(adrv9104_rx_nb_sinc_hb_top_fields);

static const struct adrv9104_profile_field adrv9104_dp_in_fifo_fields[] = {
	ADRV9104_FIELD_INT(dpInFifoConfig_t, dpInFifoTestDataSel),
	ADRV9104_FIELD_INT(dpInFifoConfig_t, dpInFifoEn),
	ADRV9104_FIELD_INT(dpInFifoConfig_t, dpInFifoMode),
	ADRV9104_FIELD_ARRAY(dpInFifoConfig_t, padding),
};

static const struct adrv9104_profile_schema adrv9104_sch_dp_in_fifo =
	ADRV9104_SCHEMA(adrv9104_dp_in_fifo_fields);

static const struct adrv9104_profile_field adrv9104_rx_nb_dem_nb_fields[] = {
	ADRV9104_FIELD_INT(rxnbDemConfigNb_t, wbPthSel),
	ADRV9104_FIELD_INT(rxnbDemConfigNb_t, dp3Sel),
	ADRV9104_FIELD_INT(rxnbDemConfigNb_t, corrFifoDelayEn),
	ADRV9104_FIELD_INT(rxnbDemConfigNb_t, corrSelDp2En),
	ADRV9104_FIELD_STRUCT(rxnbDemConfigNb_t, dpInFifo, adrv9104_sch_dp_in_fifo),
	ADRV9104_FIELD_STRUCT(rxnbDemConfigNb_t, rxNbNco, adrv9104_sch_rx_nb_nco),
	ADRV9104_FIELD_STRUCT_ARRAY(rxnbDemConfigNb_t, rxWbNbCompPFir,
				    adrv9104_sch_rx_wb_nb_comp_pfir),
	ADRV9104_FIELD_STRUCT_ARRAY(rxnbDemConfigNb_t, reSamp, adrv9104_sch_rx_resamp),
	ADRV9104_FIELD_ARRAY(rxnbDemConfigNb_t, gsOutMuxSel),
	ADRV9104_FIELD_ARRAY(rxnbDemConfigNb_t, dnSampleSel),
	ADRV9104_FIELD_ARRAY(rxnbDemConfigNb_t, FDSampleDelay),
	ADRV9104_FIELD_ARRAY(rxnbDemConfigNb_t, pulPfirDelay),
	ADRV9104_FIELD_ARRAY(rxnbDemConfigNb_t, rxOutSel),
	ADRV9104_FIELD_ARRAY(rxnbDemConfigNb_t, rxRoundMode),
	ADRV9104_FIELD_ARRAY(rxnbDemConfigNb_t, freqDeviation),
	ADRV9104_FIELD_ARRAY(rxnbDemConfigNb_t, symbolRate),
	ADRV9104_FIELD_INT(rxnbDemConfigNb_t, dpArmSel),
	ADRV9104_FIELD_ARRAY(rxnbDemConfigNb_t, dpEnabled),
	ADRV9104_FIELD_ARRAY(rxnbDemConfigNb_t, padding),
};

static const struct adrv9104_profile_schema adrv9104_sch_rx_nb_dem_nb =
	ADRV9104_SCHEMA(adrv9104_rx_nb_dem_nb_fields);

static const struct adrv9104_profile_field adrv9104_rx_nb_dp_profile_fields[] = {
	ADRV9104_FIELD_STRUCT(rxNbDpProfile_t, nbDecTopNb, adrv9104_sch_rx_nb_dec_top_nb),
	ADRV9104_FIELD_STRUCT(rxNbDpProfile_t, sincHbTopNb, adrv9104_sch_rx_nb_sinc_hb_top),
	ADRV9104_FIELD_STRUCT(rxNbDpProfile_t, rxNbDemNb, adrv9104_sch_rx_nb_dem_nb),
};

static const struct adrv9104_profile_schema adrv9104_sch_rx_nb_dp_profile =
	ADRV9104_SCHEMA(adrv9104_rx_nb_dp_profile_fields);

static const struct adrv9104_profile_field adrv9104_agc_high_fields[] = {
	ADRV9104_FIELD_INT(rxNbAgcHighThresholdSettings_t, ApdPeakOverloadThresh),
	ADRV9104_FIELD_INT(rxNbAgcHighThresholdSettings_t, ApdPeakUnderloadThresh),
	ADRV9104_FIELD_INT(rxNbAgcHighThresholdSettings_t, Hb2PeakOverloadThresh),
	ADRV9104_FIELD_INT(rxNbAgcHighThresholdSettings_t, Hb2PeakUnderloadThresh),
	ADRV9104_FIELD_INT(rxNbAgcHighThresholdSettings_t, Hb2SecondaryUpperThresh),
	ADRV9104_FIELD_ARRAY(rxNbAgcHighThresholdSettings_t, padding),
};

static const struct adrv9104_profile_schema adrv9104_sch_agc_high =
	ADRV9104_SCHEMA(adrv9104_agc_high_fields);

static const struct adrv9104_profile_field adrv9104_gain_control_fields[] = {
	ADRV9104_FIELD_BOOL(rx_gainControl_t, rxNbAgcHighThresholdModeEnable),
	ADRV9104_FIELD_BOOL(rx_gainControl_t, rxNbApdLegacyMode),
	ADRV9104_FIELD_ARRAY(rx_gainControl_t, padding),
	ADRV9104_FIELD_STRUCT(rx_gainControl_t, agcHighThresholdSettings, adrv9104_sch_agc_high),
};

static const struct adrv9104_profile_schema adrv9104_sch_gain_control =
	ADRV9104_SCHEMA(adrv9104_gain_control_fields);

static const struct adrv9104_profile_field adrv9104_rx_nb_config_fields[] = {
	ADRV9104_FIELD_STRUCT(rxNbConfig_t, rxNbDpProfile, adrv9104_sch_rx_nb_dp_profile),
	ADRV9104_FIELD_INT(rxNbConfig_t, adcOption),
	ADRV9104_FIELD_STRUCT(rxNbConfig_t, gainControl, adrv9104_sch_gain_control),
};

static const struct adrv9104_profile_schema adrv9104_sch_rx_nb_config =
	ADRV9104_SCHEMA(adrv9104_rx_nb_config_fields);

static const struct adrv9104_profile_field adrv9104_pll_modulus_fields[] = {
	ADRV9104_FIELD_ARRAY(pllModulus_t, modulus),
	ADRV9104_FIELD_ARRAY(pllModulus_t, dmModulus),
};

static const struct adrv9104_profile_schema adrv9104_sch_pll_modulus =
	ADRV9104_SCHEMA(adrv9104_pll_modulus_fields);

static const struct adrv9104_profile_field adrv9104_sys_config_fields[] = {
	ADRV9104_FIELD_INT(deviceSysConfig_t, duplexMode),
	ADRV9104_FIELD_INT(deviceSysConfig_t, fhModeOn),
	ADRV9104_FIELD_INT(deviceSysConfig_t, numDynamicProfile),
	ADRV9104_FIELD_INT(deviceSysConfig_t, mcsMode),
	ADRV9104_FIELD_INT(deviceSysConfig_t, intLOPllLockTime_us),
	ADRV9104_FIELD_INT(deviceSysConfig_t, extVCO1PllLockTime_us),
	ADRV9104_FIELD_INT(deviceSysConfig_t, extVCO2PllLockTime_us),
	ADRV9104_FIELD_STRUCT(deviceSysConfig_t, pllModuli, adrv9104_sch_pll_modulus),
	ADRV9104_FIELD_INT(deviceSysConfig_t, pllPhaseSyncWait_us),
	ADRV9104_FIELD_INT(deviceSysConfig_t, mcsInterfaceType),
	ADRV9104_FIELD_INT(deviceSysConfig_t, warmBootOn),
	ADRV9104_FIELD_ARRAY(deviceSysConfig_t, padding),
};

static const struct adrv9104_profile_schema adrv9104_sch_sys_config =
	ADRV9104_SCHEMA(adrv9104_sys_config_fields);

static const struct adrv9104_profile_field adrv9104_profile_fields[] = {
	ADRV9104_FIELD_STRUCT(deviceProfile_t, ver, adrv9104_sch_ver),
	ADRV9104_FIELD_INT(deviceProfile_t, vcoFreq_daHz),
	ADRV9104_FIELD_INT(deviceProfile_t, hsDigFreq_Hz),
	ADRV9104_FIELD_INT(deviceProfile_t, hsDigFreq_PLL_Hz),
	ADRV9104_FIELD_INT(deviceProfile_t, devClock_Hz),
	ADRV9104_FIELD_INT(deviceProfile_t, ps1ArmClkDiv),
	ADRV9104_FIELD_INT(deviceProfile_t, ps1ArmClkDiv_PLL),
	ADRV9104_FIELD_INT(deviceProfile_t, armPowerSavingClkDiv),
	ADRV9104_FIELD_INT(deviceProfile_t, refClockOutEn),
	ADRV9104_FIELD_INT(deviceProfile_t, clkPllPowerLevel),
	ADRV9104_FIELD_INT(deviceProfile_t, clkGenDis),
	ADRV9104_FIELD_INT(deviceProfile_t, clkDoublerEn),
	ADRV9104_FIELD_INT(deviceProfile_t, ps2En),
	ADRV9104_FIELD_INT(deviceProfile_t, ps2ArmClkDiv),
	ADRV9104_FIELD_INT(deviceProfile_t, loPhaseSync),
	ADRV9104_FIELD_STRUCT_ARRAY(deviceProfile_t, loConfig, adrv9104_sch_lo_config),
	ADRV9104_FIELD_INT(deviceProfile_t, chanConfig),
	ADRV9104_FIELD_INT(deviceProfile_t, observationMixer),
	ADRV9104_FIELD_ARRAY(deviceProfile_t, padding),
	ADRV9104_FIELD_STRUCT_ARRAY(deviceProfile_t, txConfig, adrv9104_sch_tx_config),
	ADRV9104_FIELD_STRUCT_ARRAY(deviceProfile_t, rxConfig, adrv9104_sch_rx_config),
	ADRV9104_FIELD_ARRAY(deviceProfile_t, txAllowedInitCalMask),
	ADRV9104_FIELD_ARRAY(deviceProfile_t, txAllowedTrackingCalMask),
	ADRV9104_FIELD_ARRAY(deviceProfile_t, rxAllowedInitCalMask),
	ADRV9104_FIELD_ARRAY(deviceProfile_t, rxAllowedTrackingCalMask),
	ADRV9104_FIELD_STRUCT_ARRAY(deviceProfile_t, orxConfig, adrv9104_sch_rx_config),
	ADRV9104_FIELD_STRUCT_ARRAY(deviceProfile_t, ilbConfig, adrv9104_sch_rx_config),
	ADRV9104_FIELD_STRUCT_ARRAY(deviceProfile_t, elbConfig, adrv9104_sch_rx_config),
	ADRV9104_FIELD_STRUCT(deviceProfile_t, tx1Config, adrv9104_sch_tx1_config),
	ADRV9104_FIELD_STRUCT(deviceProfile_t, rx1Config, adrv9104_sch_rx1_config),
	ADRV9104_FIELD_STRUCT(deviceProfile_t, orx1Config, adrv9104_sch_rx1_config),
	ADRV9104_FIELD_STRUCT(deviceProfile_t, ilb1Config, adrv9104_sch_rx1_config),
	ADRV9104_FIELD_STRUCT(deviceProfile_t, elb1Config, adrv9104_sch_rx1_config),
	ADRV9104_FIELD_STRUCT(deviceProfile_t, txNbConfig, adrv9104_sch_tx_nb_config),
	ADRV9104_FIELD_STRUCT(deviceProfile_t, rxNbConfig, adrv9104_sch_rx_nb_config),
	ADRV9104_FIELD_STRUCT(deviceProfile_t, orxNbConfig, adrv9104_sch_rx_nb_config),
	ADRV9104_FIELD_STRUCT(deviceProfile_t, ilbNbConfig, adrv9104_sch_rx_nb_config),
	ADRV9104_FIELD_STRUCT(deviceProfile_t, elbNbConfig, adrv9104_sch_rx_nb_config),
	ADRV9104_FIELD_STRUCT(deviceProfile_t, sysConfig, adrv9104_sch_sys_config),
	ADRV9104_FIELD_INT(deviceProfile_t, checksum),
};

static const struct adrv9104_profile_schema adrv9104_sch_profile =
	ADRV9104_SCHEMA(adrv9104_profile_fields);

static const struct adrv9104_profile_field adrv9104_pfir_wb_nb_fields[] = {
	ADRV9104_FIELD_INT(pfirWbNbBuffer_t, numCoeff),
	ADRV9104_FIELD_INT(pfirWbNbBuffer_t, symmetricSel),
	ADRV9104_FIELD_INT(pfirWbNbBuffer_t, tapsSel),
	ADRV9104_FIELD_INT(pfirWbNbBuffer_t, gainSel),
	ADRV9104_FIELD_ARRAY(pfirWbNbBuffer_t, coefficients),
};

static const struct adrv9104_profile_schema adrv9104_sch_pfir_wb_nb =
	ADRV9104_SCHEMA(adrv9104_pfir_wb_nb_fields);

static const struct adrv9104_profile_field adrv9104_pfir_pulse_fields[] = {
	ADRV9104_FIELD_INT(pfirPulseBuffer_t, numCoeff),
	ADRV9104_FIELD_INT(pfirPulseBuffer_t, symmetricSel),
	ADRV9104_FIELD_INT(pfirPulseBuffer_t, taps),
	ADRV9104_FIELD_INT(pfirPulseBuffer_t, gainSel),
	ADRV9104_FIELD_ARRAY(pfirPulseBuffer_t, coefficients),
};

static const struct adrv9104_profile_schema adrv9104_sch_pfir_pulse =
	ADRV9104_SCHEMA(adrv9104_pfir_pulse_fields);

static const struct adrv9104_profile_field adrv9104_pfir_mag21_fields[] = {
	ADRV9104_FIELD_INT(pfirMag21Buffer_t, numCoeff),
	ADRV9104_FIELD_ARRAY(pfirMag21Buffer_t, padding),
	ADRV9104_FIELD_ARRAY(pfirMag21Buffer_t, coefficients),
};

static const struct adrv9104_profile_schema adrv9104_sch_pfir_mag21 =
	ADRV9104_SCHEMA(adrv9104_pfir_mag21_fields);

static const struct adrv9104_profile_field adrv9104_pfir_mag13_fields[] = {
	ADRV9104_FIELD_INT(pfirMag13Buffer_t, numCoeff),
	ADRV9104_FIELD_ARRAY(pfirMag13Buffer_t, padding),
	ADRV9104_FIELD_ARRAY(pfirMag13Buffer_t, coefficients),
};

static const struct adrv9104_profile_schema adrv9104_sch_pfir_mag13 =
	ADRV9104_SCHEMA(adrv9104_pfir_mag13_fields);

static const struct adrv9104_profile_field adrv9104_pfir_buffer_fields[] = {
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirRxWbNbChFilterCoeff_A, adrv9104_sch_pfir_wb_nb),
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirRxWbNbChFilterCoeff_B, adrv9104_sch_pfir_wb_nb),
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirRxWbNbChFilterCoeff_C, adrv9104_sch_pfir_wb_nb),
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirRxWbNbChFilterCoeff_D, adrv9104_sch_pfir_wb_nb),
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirRxWbNbChFilterCoeff_E, adrv9104_sch_pfir_wb_nb),
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirRxWbNbChFilterCoeff_F, adrv9104_sch_pfir_wb_nb),
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirTxWbNbPulShpCoeff_A, adrv9104_sch_pfir_wb_nb),
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirTxWbNbPulShpCoeff_B, adrv9104_sch_pfir_wb_nb),
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirTxWbNbPulShpCoeff_C, adrv9104_sch_pfir_wb_nb),
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirTxWbNbPulShpCoeff_D, adrv9104_sch_pfir_wb_nb),
	ADRV9104_FIELD_STRUCT_ARRAY(pfirBuffer_t, pfirRxNbPulShpDp1, adrv9104_sch_pfir_pulse),
	ADRV9104_FIELD_STRUCT_ARRAY(pfirBuffer_t, pfirRxNbPulShpDp2, adrv9104_sch_pfir_pulse),
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirRxMagLowTiaLowSR_RX1, adrv9104_sch_pfir_mag21),
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirRxMagLowTiaHighSR_RX1, adrv9104_sch_pfir_mag21),
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirRxMagHighTiaHighSR_RX1, adrv9104_sch_pfir_mag21),
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirRxMagLowTia_RXNB, adrv9104_sch_pfir_mag21),
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirRxMagHighTia_RXNB, adrv9104_sch_pfir_mag21),
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirTxMagComp1, adrv9104_sch_pfir_mag21),
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirTxMagComp2, adrv9104_sch_pfir_mag21),
	ADRV9104_FIELD_STRUCT_ARRAY(pfirBuffer_t, pfirTxMagCompNb, adrv9104_sch_pfir_mag13),
	ADRV9104_FIELD_STRUCT_ARRAY(pfirBuffer_t, pfirRxMagCompNb, adrv9104_sch_pfir_mag13),
	ADRV9104_FIELD_STRUCT(pfirBuffer_t, pfirRxNbHb2Mitigation, adrv9104_sch_pfir_mag21),
	ADRV9104_FIELD_INT(pfirBuffer_t, checksum),
};

static const struct adrv9104_profile_schema adrv9104_sch_pfir_buffer =
	ADRV9104_SCHEMA(adrv9104_pfir_buffer_fields);

static const struct adrv9104_profile_field adrv9104_bundle_fields[] = {
	ADRV9104_FIELD_STRUCT(deviceProfileBundle_t, profile, adrv9104_sch_profile),
	ADRV9104_FIELD_STRUCT(deviceProfileBundle_t, pfirBuffer, adrv9104_sch_pfir_buffer),
	ADRV9104_FIELD_SKIP("Version"),
};

static const struct adrv9104_profile_schema adrv9104_sch_bundle =
	ADRV9104_SCHEMA(adrv9104_bundle_fields);

static u32 adrv9104_profile_read_int(const void *p, unsigned int size, bool is_signed)
{
	u32 value;

	switch (size) {
	case 1:
		value = *(const u8 *)p;
		break;
	case 2:
		value = *(const u16 *)p;
		break;
	default:
		value = *(const u32 *)p;
		break;
	}

	return is_signed ? sign_extend32(value, size * 8 - 1) : value;
}

static void adrv9104_profile_write_int(void *p, unsigned int size, u32 value)
{
	switch (size) {
	case 1:
		 *(u8 *)p = value;
		return;
	case 2:
		 *(u16 *)p = value;
		return;
	default:
		 *(u32 *)p = value;
		return;
	}
}

/*
 * RFC4648 base64 encodes every 3 input bytes as 4 output chars, padding the
 * final group with '='. So the encoded length is ceil(count / 3) * 4.
 */
#define ADRV9104_B64_LEN(nbytes)	(DIV_ROUND_UP(nbytes, 3) * 4)

struct adrv9104_profile_parser {
	struct device *dev;
	const char *start;
	const char *cur;
	const char *end;
};

static void adrv9104_profile_skip_space(struct adrv9104_profile_parser *p)
{
	while (p->cur < p->end) {
		char c = *p->cur;

		if (c != ' ' && c != '\t' && c != '\n' && c != '\r')
			return;

		p->cur++;
	}
}

static char adrv9104_profile_peek(struct adrv9104_profile_parser *p)
{
	adrv9104_profile_skip_space(p);
	return p->cur < p->end ? *p->cur : '\0';
}

static int adrv9104_profile_expect(struct adrv9104_profile_parser *p, char c)
{
	if (adrv9104_profile_peek(p) == c) {
		p->cur++;
		return 0;
	}

	dev_err(p->dev, "expected '%c' at offset %td\n", c, p->cur - p->start);
	return -EINVAL;
}

static int adrv9104_profile_parse_string(struct adrv9104_profile_parser *p,
					 const char **start, size_t *len)
{
	const char *close;
	int ret;

	ret = adrv9104_profile_expect(p, '"');
	if (ret)
		return ret;

	/* Strings in this format (base64, version) never contain escapes. */
	close = memchr(p->cur, '"', p->end - p->cur);
	if (!close) {
		dev_err(p->dev, "unterminated string at offset %td\n", p->cur - p->start);
		return -EINVAL;
	}

	*start = p->cur;
	*len = close - p->cur;
	p->cur = close + 1;
	return 0;
}

static int adrv9104_profile_parse_number(struct adrv9104_profile_parser *p, s64 *out)
{
	const char *begin;
	char buf[24];
	size_t n;

	/* Isolate the numeric token and let kstrtos64() do the conversion. */
	adrv9104_profile_skip_space(p);
	begin = p->cur;
	while (p->cur < p->end && (*p->cur == '-' || isdigit(*p->cur)))
		p->cur++;

	n = p->cur - begin;
	if (!n || n >= sizeof(buf)) {
		dev_err(p->dev, "invalid number at offset %td\n", begin - p->start);
		return -EINVAL;
	}

	memcpy(buf, begin, n);
	buf[n] = '\0';

	/* kstrtos64() because it handles both signed and unsigned integers */
	return kstrtos64(buf, 10, out);
}

static const struct adrv9104_profile_field *
adrv9104_profile_find_field(const struct adrv9104_profile_schema *s, const char *name, size_t len)
{
	unsigned int i;

	for (i = 0; i < s->nfields; i++) {
		const struct adrv9104_profile_field *field = &s->fields[i];

		if (!strncmp(field->name, name, len) && !field->name[len])
			return field;
	}

	return ERR_PTR(-EINVAL);
}

static int adrv9104_profile_parse_base64(struct adrv9104_profile_parser *p, u8 *dst,
					 unsigned int count)
{
	const char *s;
	size_t len;
	int ret;

	ret = adrv9104_profile_parse_string(p, &s, &len);
	if (ret)
		return ret;

	/* bound the input so base64_decode() cannot overflow @dst */
	if (len != ADRV9104_B64_LEN(count) || base64_decode(s, len, dst) != (int)count) {
		dev_err(p->dev, "invalid base64 at offset %td\n", s - p->start);
		return -EINVAL;
	}

	return 0;
}

static int adrv9104_profile_parse_schema(struct adrv9104_profile_parser *p,
					 const struct adrv9104_profile_schema *s, void *base)
{
	int ret;

	ret = adrv9104_profile_expect(p, '{');
	if (ret)
		return ret;

	/*
	 * Parse one "key": value pair per iteration; the loop ends at the first
	 * missing ',' (then a closing '}' is required). p->cur strictly advances
	 * each iteration and is bounded by p->end, so malformed input cannot loop
	 * forever - it fails in one of the called helpers.
	 */
	for (;;) {
		const struct adrv9104_profile_field *field;
		const char *name;
		unsigned int i;
		size_t nlen;
		u8 *dst;
		s64 n;

		ret = adrv9104_profile_parse_string(p, &name, &nlen);
		if (ret)
			return ret;

		ret = adrv9104_profile_expect(p, ':');
		if (ret)
			return ret;

		field = adrv9104_profile_find_field(s, name, nlen);
		if (IS_ERR(field)) {
			dev_err(p->dev, "unknown key '%.*s' at offset %td\n",
				(int)nlen, name, name - p->start);
			return PTR_ERR(field);
		}

		dst = (u8 *)base + field->offset;

		switch (field->type) {
		case ADRV9104_PROFILE_INT:
			ret = adrv9104_profile_parse_number(p, &n);
			if (ret)
				return ret;

			adrv9104_profile_write_int(dst, field->elem_size, n);
			break;
		case ADRV9104_PROFILE_BOOL:
			if (adrv9104_profile_peek(p) == 't') {
				adrv9104_profile_write_int(dst, field->elem_size, 1);
				p->cur += strlen("true");
			} else if (adrv9104_profile_peek(p) == 'f') {
				adrv9104_profile_write_int(dst, field->elem_size, 0);
				p->cur += strlen("false");
			} else {
				dev_err(p->dev, "invalid bool at offset %td\n",
					p->cur - p->start);
				return -EINVAL;
			}
			break;
		case ADRV9104_PROFILE_BYTES:
			ret = adrv9104_profile_parse_base64(p, dst, field->count);
			if (ret)
				return ret;

			break;
		case ADRV9104_PROFILE_INT_ARRAY:
			ret = adrv9104_profile_expect(p, '[');
			if (ret)
				return ret;

			for (i = 0; i < field->count; i++) {
				ret = adrv9104_profile_parse_number(p, &n);
				if (ret)
					return ret;

				adrv9104_profile_write_int(dst + i * field->elem_size,
							   field->elem_size, n);

				if (i + 1 == field->count)
					break;

				ret = adrv9104_profile_expect(p, ',');
				if (ret)
					return ret;
			}

			ret = adrv9104_profile_expect(p, ']');
			if (ret)
				return ret;

			break;
		case ADRV9104_PROFILE_STRUCT:
			ret = adrv9104_profile_parse_schema(p, field->sub, dst);
			if (ret)
				return ret;

			break;
		case ADRV9104_PROFILE_STRUCT_ARRAY:
			ret = adrv9104_profile_expect(p, '[');
			if (ret)
				return ret;

			for (i = 0; i < field->count; i++) {
				ret = adrv9104_profile_parse_schema(p, field->sub,
								    dst + i * field->stride);
				if (ret)
					return ret;

				if (i + 1 == field->count)
					break;

				ret = adrv9104_profile_expect(p, ',');
				if (ret)
					return ret;
			}

			ret = adrv9104_profile_expect(p, ']');
			if (ret)
				return ret;

			break;
		case ADRV9104_PROFILE_SKIP:
			/*
			 * Only "Version" is skipped for now, and it is a string, so
			 * just consume it. Add a proper value-skipper if other
			 * ignored keys ever show up.
			 */
			ret = adrv9104_profile_parse_string(p, &name, &nlen);
			if (ret)
				return ret;

			break;
		default:
			return -EINVAL;
		}

		if (adrv9104_profile_peek(p) != ',')
			break;

		p->cur++;
	}

	return adrv9104_profile_expect(p, '}');
}

static int adrv9104_profile_parse(struct device *dev, const char *buf, size_t len,
				  deviceProfileBundle_t *out)
{
	struct adrv9104_profile_parser p = {
		.dev = dev,
		.start = buf,
		.cur = buf,
		.end = buf + len,
	};

	return adrv9104_profile_parse_schema(&p, &adrv9104_sch_bundle, out);
}

struct adrv9104_profile_writer {
	char *buf;
	size_t cap;
	size_t len;
};

static int adrv9104_profile_put_mem(struct adrv9104_profile_writer *w, const char *src, size_t n)
{
	if (w->len + n >= w->cap)
		return -ENOSPC;

	memcpy(w->buf + w->len, src, n);
	w->len += n;

	return 0;
}

static int adrv9104_profile_put_char(struct adrv9104_profile_writer *w, char c)
{
	return adrv9104_profile_put_mem(w, &c, 1);
}

static int adrv9104_profile_put_str(struct adrv9104_profile_writer *w, const char *s)
{
	return adrv9104_profile_put_mem(w, s, strlen(s));
}

static int adrv9104_profile_put_int(struct adrv9104_profile_writer *w, const void *p,
				    unsigned int size, bool is_signed)
{
	u32 value = adrv9104_profile_read_int(p, size, is_signed);
	char tmp[24];
	int n;

	if (is_signed)
		n = snprintf(tmp, sizeof(tmp), "%d", (int)value);
	else
		n = snprintf(tmp, sizeof(tmp), "%u", value);

	return adrv9104_profile_put_mem(w, tmp, n);
}

static int adrv9104_profile_put_base64(struct adrv9104_profile_writer *w, const u8 *src,
				       unsigned int count)
{
	int ret;

	ret = adrv9104_profile_put_char(w, '"');
	if (ret)
		return ret;

	if (w->len + ADRV9104_B64_LEN(count) >= w->cap)
		return -ENOSPC;

	w->len += base64_encode(src, count, w->buf + w->len);

	return adrv9104_profile_put_char(w, '"');
}

static int adrv9104_profile_put_schema(struct adrv9104_profile_writer *w,
				       const struct adrv9104_profile_schema *s, const void *base)
{
	bool need_comma = false;
	unsigned int i;
	int ret;

	ret = adrv9104_profile_put_char(w, '{');
	if (ret)
		return ret;

	for (i = 0; i < s->nfields; i++) {
		const struct adrv9104_profile_field *field = &s->fields[i];
		const u8 *src = (const u8 *)base + field->offset;
		unsigned int j;

		if (field->type == ADRV9104_PROFILE_SKIP)
			continue;

		if (need_comma) {
			ret = adrv9104_profile_put_char(w, ',');
			if (ret)
				return ret;
		} else {
			need_comma = true;
		}

		ret = adrv9104_profile_put_char(w, '"');
		if (ret)
			return ret;

		ret = adrv9104_profile_put_str(w, field->name);
		if (ret)
			return ret;

		ret = adrv9104_profile_put_str(w, "\":");
		if (ret)
			return ret;

		switch (field->type) {
		case ADRV9104_PROFILE_INT:
			ret = adrv9104_profile_put_int(w, src, field->elem_size, field->is_signed);
			break;
		case ADRV9104_PROFILE_BOOL: {
			bool value = !!adrv9104_profile_read_int(src, field->elem_size, false);

			ret = adrv9104_profile_put_str(w, value ? "true" : "false");
			break;
		}
		case ADRV9104_PROFILE_BYTES:
			ret = adrv9104_profile_put_base64(w, src, field->count);
			break;
		case ADRV9104_PROFILE_INT_ARRAY:
			ret = adrv9104_profile_put_char(w, '[');
			if (ret)
				return ret;

			for (j = 0; j < field->count; j++) {
				ret = adrv9104_profile_put_int(w, src + j * field->elem_size,
							       field->elem_size, field->is_signed);
				if (ret)
					return ret;

				if (j + 1 == field->count)
					break;

				ret = adrv9104_profile_put_char(w, ',');
				if (ret)
					return ret;
			}

			ret = adrv9104_profile_put_char(w, ']');
			break;
		case ADRV9104_PROFILE_STRUCT:
			ret = adrv9104_profile_put_schema(w, field->sub, src);
			break;
		case ADRV9104_PROFILE_STRUCT_ARRAY:
			ret = adrv9104_profile_put_char(w, '[');
			if (ret)
				return ret;

			for (j = 0; j < field->count; j++) {
				ret = adrv9104_profile_put_schema(w, field->sub,
								  src + j * field->stride);
				if (ret)
					return ret;

				if (j + 1 == field->count)
					break;

				ret = adrv9104_profile_put_char(w, ',');
				if (ret)
					return ret;
			}

			ret = adrv9104_profile_put_char(w, ']');
			break;
		default:
			return -EINVAL;
		}

		if (ret)
			return ret;
	}

	return adrv9104_profile_put_char(w, '}');
}

int adrv9104_profile_serialize(struct adrv9104_rf_phy *phy, char *out, size_t outlen)
{
	struct adrv9104_profile_writer w = { .buf = out, .cap = outlen };
	int ret;

	ret = adrv9104_profile_put_schema(&w, &adrv9104_sch_bundle, &phy->profile_bundle);
	if (ret)
		return ret;

	w.buf[w.len] = '\0';

	return w.len;
}

int adrv9104_profile_load(struct adrv9104_rf_phy *phy)
{
	int ret;

	const struct firmware *fw __free(firmware) = NULL;
	ret = request_firmware(&fw, ADRV9104_PROFILE_DEFAULT_NAME, phy->dev);
	if (ret)
		return dev_err_probe(phy->dev, ret, "failed to request profile '%s'\n",
				     ADRV9104_PROFILE_DEFAULT_NAME);

	return adrv9104_profile_parse(phy->dev, fw->data, fw->size, &phy->profile_bundle);
}

static enum fw_upload_err adrv9104_profile_fwu_prepare(struct fw_upload *fwu,
						       const u8 *data, u32 size)
{
	struct adrv9104_rf_phy *phy = fwu->dd_handle;

	if (!size || size > ADRV9104_PROFILE_MAX_SZ)
		return FW_UPLOAD_ERR_INVALID_SIZE;

	phy->profile_new = kzalloc(sizeof(*phy->profile_new), GFP_KERNEL);
	if (!phy->profile_new)
		return FW_UPLOAD_ERR_RW_ERROR;

	return FW_UPLOAD_ERR_NONE;
}

static enum fw_upload_err adrv9104_profile_fwu_write(struct fw_upload *fwu, const u8 *data,
						     u32 offset, u32 size, u32 *written)
{
	struct adrv9104_rf_phy *phy = fwu->dd_handle;
	int ret;

	dev_dbg(phy->dev, "writing %u bytes at offset %u (sz=%u)\n", size, offset, size);

	ret = adrv9104_profile_parse(phy->dev, data + offset, size, phy->profile_new);
	if (ret)
		return FW_UPLOAD_ERR_FW_INVALID;

	dev_dbg(phy->dev, "profile upload complete\n");

	*written = size;
	return FW_UPLOAD_ERR_NONE;
}

static enum fw_upload_err adrv9104_profile_fwu_poll_complete(struct fw_upload *fwu)
{
	struct adrv9104_rf_phy *phy = fwu->dd_handle;
	int ret;

	ret = adrv9104_init_with_profile(phy, phy->profile_new);
	if (ret)
		return FW_UPLOAD_ERR_HW_ERROR;

	return FW_UPLOAD_ERR_NONE;
}

static void adrv9104_profile_fwu_cancel(struct fw_upload *fwu)
{
	/*
	 * Mandatory op, but we do not have anything to cancel. As soon as .prepare() starts,
	 * there is nothing to cancel, and .cleanup() will free the allocated memory.
	 */
}

static void adrv9104_profile_fwu_cleanup(struct fw_upload *fwu)
{
	struct adrv9104_rf_phy *phy = fwu->dd_handle;

	kfree(phy->profile_new);
}

static const struct fw_upload_ops adrv9104_profile_fwu_ops = {
	.prepare = adrv9104_profile_fwu_prepare,
	.write = adrv9104_profile_fwu_write,
	.poll_complete = adrv9104_profile_fwu_poll_complete,
	.cancel = adrv9104_profile_fwu_cancel,
	.cleanup = adrv9104_profile_fwu_cleanup,
};

static void adrv9104_profile_fwu_unregister(void *fwu)
{
	firmware_upload_unregister(fwu);
}

int adrv9104_profile_register(struct adrv9104_rf_phy *phy, struct iio_dev *indio_dev)
{
	struct fw_upload *fwu;
	char name[32];

	/* Name the upload node after the IIO device ("iio:deviceN:profile"). */
	snprintf(name, sizeof(name), "%s:profile", dev_name(&indio_dev->dev));

	fwu = firmware_upload_register(THIS_MODULE, phy->dev, name,
				       &adrv9104_profile_fwu_ops, phy);
	if (IS_ERR(fwu))
		return dev_err_probe(phy->dev, PTR_ERR(fwu),
				     "failed to register profile upload\n");

	return devm_add_action_or_reset(phy->dev, adrv9104_profile_fwu_unregister, fwu);
}
