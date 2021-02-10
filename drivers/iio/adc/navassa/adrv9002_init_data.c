// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV9002 RF Transceiver Init file
 *
 * Copyright 2020 Analog Devices Inc.
 */
#include "adrv9002.h"
#include "adi_adrv9001_clockSettings_types.h"
#include "adi_adrv9001_deviceSysConfig_types.h"
#include "adi_adrv9001_pfirBuffer_types.h"
#include "adi_adrv9001_profile_types.h"
#include "adi_adrv9001_rxSettings_types.h"
#include "adi_adrv9001_types.h"
#include "adi_adrv9001_txSettings_types.h"

static struct adi_adrv9001_SpiSettings spiSettings = {
	.msbFirst = 1,
	.enSpiStreaming = 0,
	.autoIncAddrUp = 1,
	.fourWireMode = 1,
	.cmosPadDrvStrength = ADI_ADRV9001_CMOSPAD_DRV_STRONG,
};

struct adi_adrv9001_Init adrv9002_init_lvds = {
	.clocks = {
		.deviceClock_kHz = 38400,
		.clkPllVcoFreq_daHz = 884736000,
		.clkPllHsDiv = ADI_ADRV9001_HSDIV_4,
		.clkPllMode = ADI_ADRV9001_CLK_PLL_HP_MODE,
		.clk1105Div = ADI_ADRV9001_INTERNAL_CLOCK_DIV_2,
		.armClkDiv = ADI_ADRV9001_INTERNAL_CLOCK_DIV_6,
		.armPowerSavingClkDiv = 1,
		.refClockOutEnable = true,
		.auxPllPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
		.clkPllPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
		.padRefClkDrv = 0,
		.extLo1OutFreq_kHz = 0,
		.extLo2OutFreq_kHz = 0,
		.rfPll1LoMode = ADI_ADRV9001_INT_LO1,
		.rfPll2LoMode = ADI_ADRV9001_INT_LO1,
		.ext1LoType = ADI_ADRV9001_EXT_LO_DIFFERENTIAL,
		.ext2LoType = ADI_ADRV9001_EXT_LO_DIFFERENTIAL,
		.rx1RfInputSel = ADI_ADRV9001_RX_A,
		.rx2RfInputSel = ADI_ADRV9001_RX_A,
		.extLo1Divider = 2,
		.extLo2Divider = 2,
		.rfPllPhaseSyncMode = ADI_ADRV9001_RFPLLMCS_NOSYNC,
		.rx1LoSelect = ADI_ADRV9001_LOSEL_LO2,
		.rx2LoSelect = ADI_ADRV9001_LOSEL_LO2,
		.tx1LoSelect = ADI_ADRV9001_LOSEL_LO1,
		.tx2LoSelect = ADI_ADRV9001_LOSEL_LO1,
		.rx1LoDivMode = ADI_ADRV9001_LO_DIV_MODE_LOW_POWER,
		.rx2LoDivMode = ADI_ADRV9001_LO_DIV_MODE_LOW_POWER,
		.tx1LoDivMode = ADI_ADRV9001_LO_DIV_MODE_LOW_POWER,
		.tx2LoDivMode = ADI_ADRV9001_LO_DIV_MODE_LOW_POWER,
		.loGen1Select = ADI_ADRV9001_LOGENPOWER_RFPLL_LDO,
		.loGen2Select = ADI_ADRV9001_LOGENPOWER_RFPLL_LDO
	},
	.rx = {
		.rxInitChannelMask = 243,
		.rxChannelCfg = {
			{
				.profile =
				{
					.primarySigBandwidth_Hz = 9000000,
					.rxOutputRate_Hz = 15360000,
					.rxInterfaceSampleRate_Hz = 15360000,
					.rxOffsetLo_kHz = 0,
					.rxSignalOnLo = 0,
					.outputSignaling = ADI_ADRV9001_RX_IQ,
					.filterOrder = 1,
					.filterOrderLp = 1,
					.hpAdcCorner = 20000000,
					.lpAdcCorner = 0,
					.adcClk_kHz = 2211840,
					.rxCorner3dB_kHz = 40000,
					.rxCorner3dBLp_kHz = 40000,
					.tiaPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.tiaPowerLp = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.channelType = 1,
					.adcType = ADI_ADRV9001_ADC_HP,
					.lpAdcCalMode = ADI_ADRV9001_ADC_LOWPOWER_PERIODIC,
					.rxDpProfile =
					{
						.rxNbDecTop =
						{
							.scicBlk23En = 0,
							.scicBlk23DivFactor = 1,
							.scicBlk23LowRippleEn = 0,
							.decBy2Blk35En = 0,
							.decBy2Blk37En = 0,
							.decBy2Blk39En = 0,
							.decBy2Blk41En = 0,
							.decBy2Blk43En = 0,
							.decBy3Blk45En = 0,
							.decBy2Blk47En = 0
						},
						.rxWbDecTop =
						{
							.decBy2Blk25En = 0,
							.decBy2Blk27En = 0,
							.decBy2Blk29En = 0,
							.decBy2Blk31En = 1,
							.decBy2Blk33En = 1,
							.wbLpfBlk33p1En = 0
						},
						.rxDecTop =
						{
							.decBy3Blk15En = 1,
							.decBy2Hb3Blk17p1En = 0,
							.decBy2Hb4Blk17p2En = 0,
							.decBy2Hb5Blk19p1En = 0,
							.decBy2Hb6Blk19p2En = 0
						},
						.rxSincHBTop =
						{
							.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
							.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
							.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB1,
							.isGainCompEnabled = 0,
							.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
							.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
						},
						.rxNbDem =
						{
							.dpInFifo =
							{
								.dpInFifoEn = 0,
								.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
								.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
							},
							.rxNbNco =
							{
								.rxNbNcoEn = 0,
								.rxNbNcoConfig =
								{
									.freq = 0,
									.sampleFreq = 0,
									.phase = 0,
									.realOut = 0
								}
							},
							.rxWbNbCompPFir =
							{
								.bankSel = ADI_ADRV9001_PFIR_BANK_A,
								.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
								.rxWbNbCompPFirEn = 1
							},
							.resamp =
							{
								.rxResampEn = 0,
								.resampPhaseI = 0,
								.resampPhaseQ = 0
							},
							.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
							.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
							.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
							.dpArmSel = ADI_ADRV9001_DP_SEL
						}
					},
					.rxSsiConfig =
					{
						.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
						.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
						.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
						.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
						.lsbFirst = 0,
						.qFirst = 0,
						.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED,
						.lvdsBitInversion = 0,
						.lvdsUseLsbIn12bitMode = 0,
						.lvdsTxFullRefClkEn = false,
						.lvdsRxClkInversionEn = false,
						.cmosTxDdrNegStrobeEn = false,
						.cmosDdrPosClkEn = false,
						.cmosDdrClkInversionEn = false,
						.cmosDdrEn = false,
						.rxMaskStrobeEn = false
					}
				}
			},
			{
				.profile =
				{
					.primarySigBandwidth_Hz = 9000000,
					.rxOutputRate_Hz = 15360000,
					.rxInterfaceSampleRate_Hz = 15360000,
					.rxOffsetLo_kHz = 0,
					.rxSignalOnLo = 0,
					.outputSignaling = ADI_ADRV9001_RX_IQ,
					.filterOrder = 1,
					.filterOrderLp = 1,
					.hpAdcCorner = 20000000,
					.lpAdcCorner = 0,
					.adcClk_kHz = 2211840,
					.rxCorner3dB_kHz = 40000,
					.rxCorner3dBLp_kHz = 40000,
					.tiaPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.tiaPowerLp = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.channelType = 2,
					.adcType = ADI_ADRV9001_ADC_HP,
					.lpAdcCalMode = ADI_ADRV9001_ADC_LOWPOWER_PERIODIC,
					.rxDpProfile =
					{
						.rxNbDecTop =
						{
							.scicBlk23En = 0,
							.scicBlk23DivFactor = 1,
							.scicBlk23LowRippleEn = 0,
							.decBy2Blk35En = 0,
							.decBy2Blk37En = 0,
							.decBy2Blk39En = 0,
							.decBy2Blk41En = 0,
							.decBy2Blk43En = 0,
							.decBy3Blk45En = 0,
							.decBy2Blk47En = 0
						},
						.rxWbDecTop =
						{
							.decBy2Blk25En = 0,
							.decBy2Blk27En = 0,
							.decBy2Blk29En = 0,
							.decBy2Blk31En = 1,
							.decBy2Blk33En = 1,
							.wbLpfBlk33p1En = 0
						},
						.rxDecTop =
						{
							.decBy3Blk15En = 1,
							.decBy2Hb3Blk17p1En = 0,
							.decBy2Hb4Blk17p2En = 0,
							.decBy2Hb5Blk19p1En = 0,
							.decBy2Hb6Blk19p2En = 0
						},
						.rxSincHBTop =
						{
							.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
							.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
							.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB1,
							.isGainCompEnabled = 0,
							.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
							.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
						},
						.rxNbDem =
						{
							.dpInFifo =
							{
								.dpInFifoEn = 0,
								.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
								.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
							},
							.rxNbNco =
							{
								.rxNbNcoEn = 0,
								.rxNbNcoConfig =
								{
									.freq = 0,
									.sampleFreq = 0,
									.phase = 0,
									.realOut = 0
								}
							},
							.rxWbNbCompPFir =
							{
								.bankSel = ADI_ADRV9001_PFIR_BANK_C,
								.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
								.rxWbNbCompPFirEn = 1
							},
							.resamp =
							{
								.rxResampEn = 0,
								.resampPhaseI = 0,
								.resampPhaseQ = 0
							},
							.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
							.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
							.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
							.dpArmSel = ADI_ADRV9001_DP_SEL
						}
					},
					.rxSsiConfig =
					{
						.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
						.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
						.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
						.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
						.lsbFirst = 0,
						.qFirst = 0,
						.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED,
						.lvdsBitInversion = 0,
						.lvdsUseLsbIn12bitMode = 0,
						.lvdsTxFullRefClkEn = false,
						.lvdsRxClkInversionEn = false,
						.cmosTxDdrNegStrobeEn = false,
						.cmosDdrPosClkEn = false,
						.cmosDdrClkInversionEn = false,
						.cmosDdrEn = false,
						.rxMaskStrobeEn = false
					}
				}
			},
			{
				.profile =
				{
					.primarySigBandwidth_Hz = 9000000,
					.rxOutputRate_Hz = 15360000,
					.rxInterfaceSampleRate_Hz = 15360000,
					.rxOffsetLo_kHz = 0,
					.rxSignalOnLo = 0,
					.outputSignaling = ADI_ADRV9001_RX_IQ,
					.filterOrder = 1,
					.filterOrderLp = 1,
					.hpAdcCorner = 50000000,
					.lpAdcCorner = 0,
					.adcClk_kHz = 2211840,
					.rxCorner3dB_kHz = 100000,
					.rxCorner3dBLp_kHz = 100000,
					.tiaPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.tiaPowerLp = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.channelType = 16,
					.adcType = ADI_ADRV9001_ADC_HP,
					.lpAdcCalMode = ADI_ADRV9001_ADC_LOWPOWER_PERIODIC,
					.rxDpProfile =
					{
						.rxNbDecTop =
						{
							.scicBlk23En = 0,
							.scicBlk23DivFactor = 1,
							.scicBlk23LowRippleEn = 0,
							.decBy2Blk35En = 0,
							.decBy2Blk37En = 0,
							.decBy2Blk39En = 0,
							.decBy2Blk41En = 0,
							.decBy2Blk43En = 0,
							.decBy3Blk45En = 0,
							.decBy2Blk47En = 0
						},
						.rxWbDecTop =
						{
							.decBy2Blk25En = 0,
							.decBy2Blk27En = 0,
							.decBy2Blk29En = 0,
							.decBy2Blk31En = 0,
							.decBy2Blk33En = 1,
							.wbLpfBlk33p1En = 1
						},
						.rxDecTop =
						{
							.decBy3Blk15En = 1,
							.decBy2Hb3Blk17p1En = 0,
							.decBy2Hb4Blk17p2En = 0,
							.decBy2Hb5Blk19p1En = 0,
							.decBy2Hb6Blk19p2En = 0
						},
						.rxSincHBTop =
						{
							.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
							.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
							.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB2,
							.isGainCompEnabled = 0,
							.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384  },
							.gainComp9GainQ = { 0, 0, 0, 0, 0, 0  }
						},
						.rxNbDem =
						{
							.dpInFifo =
							{
								.dpInFifoEn = 0,
								.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
								.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
							},
							.rxNbNco =
							{
								.rxNbNcoEn = 0,
								.rxNbNcoConfig =
								{
									.freq = 0,
									.sampleFreq = 0,
									.phase = 0,
									.realOut = 0
								}
							},
							.rxWbNbCompPFir =
							{
								.bankSel = ADI_ADRV9001_PFIR_BANK_B,
								.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
								.rxWbNbCompPFirEn = 1
							},
							.resamp =
							{
								.rxResampEn = 0,
								.resampPhaseI = 0,
								.resampPhaseQ = 0
							},
							.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
							.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
							.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
							.dpArmSel = ADI_ADRV9001_DP_SEL
						}
					},
					.rxSsiConfig =
					{
						.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
						.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
						.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
						.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
						.lsbFirst = 0,
						.qFirst = 0,
						.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED,
						.lvdsBitInversion = 0,
						.lvdsUseLsbIn12bitMode = 0,
						.lvdsTxFullRefClkEn = false,
						.lvdsRxClkInversionEn = false,
						.cmosTxDdrNegStrobeEn = false,
						.cmosDdrPosClkEn = false,
						.cmosDdrClkInversionEn = false,
						.cmosDdrEn = false,
						.rxMaskStrobeEn = false
					}
				}
			},
			{
				.profile =
				{
					.primarySigBandwidth_Hz = 9000000,
					.rxOutputRate_Hz = 15360000,
					.rxInterfaceSampleRate_Hz = 15360000,
					.rxOffsetLo_kHz = 0,
					.rxSignalOnLo = 0,
					.outputSignaling = ADI_ADRV9001_RX_IQ,
					.filterOrder = 1,
					.filterOrderLp = 1,
					.hpAdcCorner = 50000000,
					.lpAdcCorner = 0,
					.adcClk_kHz = 2211840,
					.rxCorner3dB_kHz = 100000,
					.rxCorner3dBLp_kHz = 100000,
					.tiaPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.tiaPowerLp = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.channelType = 32,
					.adcType = ADI_ADRV9001_ADC_HP,
					.lpAdcCalMode = ADI_ADRV9001_ADC_LOWPOWER_PERIODIC,
					.rxDpProfile =
					{
						.rxNbDecTop =
						{
							.scicBlk23En = 0,
							.scicBlk23DivFactor = 1,
							.scicBlk23LowRippleEn = 0,
							.decBy2Blk35En = 0,
							.decBy2Blk37En = 0,
							.decBy2Blk39En = 0,
							.decBy2Blk41En = 0,
							.decBy2Blk43En = 0,
							.decBy3Blk45En = 0,
							.decBy2Blk47En = 0
						},
						.rxWbDecTop =
						{
							.decBy2Blk25En = 0,
							.decBy2Blk27En = 0,
							.decBy2Blk29En = 0,
							.decBy2Blk31En = 1,
							.decBy2Blk33En = 1,
							.wbLpfBlk33p1En = 0
						},
						.rxDecTop =
						{
							.decBy3Blk15En = 1,
							.decBy2Hb3Blk17p1En = 0,
							.decBy2Hb4Blk17p2En = 0,
							.decBy2Hb5Blk19p1En = 0,
							.decBy2Hb6Blk19p2En = 0
						},
						.rxSincHBTop =
						{
							.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
							.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_ZERO,
							.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB2,
							.isGainCompEnabled = 0,
							.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
							.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
						},
						.rxNbDem =
						{
							.dpInFifo =
							{
								.dpInFifoEn = 0,
								.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
								.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
							},
							.rxNbNco =
							{
								.rxNbNcoEn = 0,
								.rxNbNcoConfig =
								{
									.freq = 0,
									.sampleFreq = 0,
									.phase = 0,
									.realOut = 0
								}
							},
							.rxWbNbCompPFir =
							{
								.bankSel = ADI_ADRV9001_PFIR_BANK_D,
								.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
								.rxWbNbCompPFirEn = 1
							},
							.resamp =
							{
								.rxResampEn = 0,
								.resampPhaseI = 0,
								.resampPhaseQ = 0
							},
							.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
							.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
							.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
							.dpArmSel = ADI_ADRV9001_DP_SEL
						}
					},
					.rxSsiConfig =
					{
						.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
						.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
						.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
						.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
						.lsbFirst = 0,
						.qFirst = 0,
						.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED,
						.lvdsBitInversion = 0,
						.lvdsUseLsbIn12bitMode = 0,
						.lvdsTxFullRefClkEn = false,
						.lvdsRxClkInversionEn = false,
						.cmosTxDdrNegStrobeEn = false,
						.cmosDdrPosClkEn = false,
						.cmosDdrClkInversionEn = false,
						.cmosDdrEn = false,
						.rxMaskStrobeEn = false
					}
				}
			},
			{
				.profile =
				{
					.primarySigBandwidth_Hz = 9000000,
					.rxOutputRate_Hz = 15360000,
					.rxInterfaceSampleRate_Hz = 15360000,
					.rxOffsetLo_kHz = 0,
					.rxSignalOnLo = 0,
					.outputSignaling = ADI_ADRV9001_RX_IQ,
					.filterOrder = 1,
					.filterOrderLp = 1,
					.hpAdcCorner = 50000000,
					.lpAdcCorner = 0,
					.adcClk_kHz = 2211840,
					.rxCorner3dB_kHz = 100000,
					.rxCorner3dBLp_kHz = 100000,
					.tiaPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.tiaPowerLp = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.channelType = 64,
					.adcType = ADI_ADRV9001_ADC_HP,
					.lpAdcCalMode = ADI_ADRV9001_ADC_LOWPOWER_PERIODIC,
					.rxDpProfile =
					{
						.rxNbDecTop =
						{
							.scicBlk23En = 0,
							.scicBlk23DivFactor = 1,
							.scicBlk23LowRippleEn = 0,
							.decBy2Blk35En = 0,
							.decBy2Blk37En = 0,
							.decBy2Blk39En = 0,
							.decBy2Blk41En = 0,
							.decBy2Blk43En = 0,
							.decBy3Blk45En = 0,
							.decBy2Blk47En = 0
						},
						.rxWbDecTop =
						{
							.decBy2Blk25En = 0,
							.decBy2Blk27En = 0,
							.decBy2Blk29En = 0,
							.decBy2Blk31En = 1,
							.decBy2Blk33En = 1,
							.wbLpfBlk33p1En = 0
						},
						.rxDecTop =
						{
							.decBy3Blk15En = 1,
							.decBy2Hb3Blk17p1En = 0,
							.decBy2Hb4Blk17p2En = 0,
							.decBy2Hb5Blk19p1En = 0,
							.decBy2Hb6Blk19p2En = 0
						},
						.rxSincHBTop =
						{
							.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
							.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
							.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB2,
							.isGainCompEnabled = 0,
							.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
							.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
						},
						.rxNbDem =
						{
							.dpInFifo =
							{
								.dpInFifoEn = 0,
								.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
								.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
							},
							.rxNbNco =
							{
								.rxNbNcoEn = 0,
								.rxNbNcoConfig =
								{
									.freq = 0,
									.sampleFreq = 0,
									.phase = 0,
									.realOut = 0
								}
							},
							.rxWbNbCompPFir =
							{
								.bankSel = ADI_ADRV9001_PFIR_BANK_B,
								.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
								.rxWbNbCompPFirEn = 1
							},
							.resamp =
							{
								.rxResampEn = 0,
								.resampPhaseI = 0,
								.resampPhaseQ = 0
							},
							.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
							.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
							.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
							.dpArmSel = ADI_ADRV9001_DP_SEL
						}
					},
					.rxSsiConfig =
					{
						.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
						.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
						.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
						.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
						.lsbFirst = 0,
						.qFirst = 0,
						.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED,
						.lvdsBitInversion = 0,
						.lvdsUseLsbIn12bitMode = 0,
						.lvdsTxFullRefClkEn = false,
						.lvdsRxClkInversionEn = false,
						.cmosTxDdrNegStrobeEn = false,
						.cmosDdrPosClkEn = false,
						.cmosDdrClkInversionEn = false,
						.cmosDdrEn = false,
						.rxMaskStrobeEn = false
					}
				}
			},
			{
				.profile =
				{
					.primarySigBandwidth_Hz = 9000000,
					.rxOutputRate_Hz = 15360000,
					.rxInterfaceSampleRate_Hz = 15360000,
					.rxOffsetLo_kHz = 0,
					.rxSignalOnLo = 0,
					.outputSignaling = ADI_ADRV9001_RX_IQ,
					.filterOrder = 1,
					.filterOrderLp = 1,
					.hpAdcCorner = 50000000,
					.lpAdcCorner = 0,
					.adcClk_kHz = 2211840,
					.rxCorner3dB_kHz = 100000,
					.rxCorner3dBLp_kHz = 100000,
					.tiaPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.tiaPowerLp = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.channelType = 128,
					.adcType = ADI_ADRV9001_ADC_HP,
					.lpAdcCalMode = ADI_ADRV9001_ADC_LOWPOWER_PERIODIC,
					.rxDpProfile =
					{
						.rxNbDecTop =
						{
							.scicBlk23En = 0,
							.scicBlk23DivFactor = 1,
							.scicBlk23LowRippleEn = 0,
							.decBy2Blk35En = 0,
							.decBy2Blk37En = 0,
							.decBy2Blk39En = 0,
							.decBy2Blk41En = 0,
							.decBy2Blk43En = 0,
							.decBy3Blk45En = 0,
							.decBy2Blk47En = 0
						},
						.rxWbDecTop =
						{
							.decBy2Blk25En = 0,
							.decBy2Blk27En = 0,
							.decBy2Blk29En = 0,
							.decBy2Blk31En = 1,
							.decBy2Blk33En = 1,
							.wbLpfBlk33p1En = 0
						},
						.rxDecTop =
						{
							.decBy3Blk15En = 1,
							.decBy2Hb3Blk17p1En = 0,
							.decBy2Hb4Blk17p2En = 0,
							.decBy2Hb5Blk19p1En = 0,
							.decBy2Hb6Blk19p2En = 0
						},
						.rxSincHBTop =
						{
							.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
							.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
							.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB2,
							.isGainCompEnabled = 0,
							.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
							.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
						},
						.rxNbDem =
						{
							.dpInFifo =
							{
								.dpInFifoEn = 0,
								.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
								.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
							},
							.rxNbNco =
							{
								.rxNbNcoEn = 0,
								.rxNbNcoConfig =
								{
									.freq = 0,
									.sampleFreq = 0,
									.phase = 0,
									.realOut = 0
								}
							},
							.rxWbNbCompPFir =
							{
								.bankSel = ADI_ADRV9001_PFIR_BANK_D,
								.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
								.rxWbNbCompPFirEn = 1
							},
							.resamp =
							{
								.rxResampEn = 0,
								.resampPhaseI = 0,
								.resampPhaseQ = 0
							},
							.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
							.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
							.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
							.dpArmSel = ADI_ADRV9001_DP_SEL
						}
					},
					.rxSsiConfig =
					{
						.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
						.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
						.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
						.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
						.lsbFirst = 0,
						.qFirst = 0,
						.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED,
						.lvdsBitInversion = 0,
						.lvdsUseLsbIn12bitMode = 0,
						.lvdsTxFullRefClkEn = false,
						.lvdsRxClkInversionEn = false,
						.cmosTxDdrNegStrobeEn = false,
						.cmosDdrPosClkEn = false,
						.cmosDdrClkInversionEn = false,
						.cmosDdrEn = false,
						.rxMaskStrobeEn = false
					}
				}
			},
			{
				.profile =
				{
					.primarySigBandwidth_Hz = 12500,
					.rxOutputRate_Hz = 0,
					.rxInterfaceSampleRate_Hz = 0,
					.rxOffsetLo_kHz = 0,
					.rxSignalOnLo = 0,
					.outputSignaling = ADI_ADRV9001_RX_IQ,
					.filterOrder = 1,
					.filterOrderLp = 1,
					.hpAdcCorner = 0,
					.lpAdcCorner = 0,
					.adcClk_kHz = 0,
					.rxCorner3dB_kHz = 0,
					.rxCorner3dBLp_kHz = 0,
					.tiaPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.tiaPowerLp = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.channelType = 0,
					.adcType = ADI_ADRV9001_ADC_HP,
					.lpAdcCalMode = ADI_ADRV9001_ADC_LOWPOWER_PERIODIC,
					.rxDpProfile =
					{
						.rxNbDecTop =
						{
							.scicBlk23En = 0,
							.scicBlk23DivFactor = 0,
							.scicBlk23LowRippleEn = 0,
							.decBy2Blk35En = 0,
							.decBy2Blk37En = 0,
							.decBy2Blk39En = 0,
							.decBy2Blk41En = 0,
							.decBy2Blk43En = 0,
							.decBy3Blk45En = 0,
							.decBy2Blk47En = 0
						},
						.rxWbDecTop =
						{
							.decBy2Blk25En = 0,
							.decBy2Blk27En = 0,
							.decBy2Blk29En = 0,
							.decBy2Blk31En = 0,
							.decBy2Blk33En = 0,
							.wbLpfBlk33p1En = 0
						},
						.rxDecTop =
						{
							.decBy3Blk15En = 0,
							.decBy2Hb3Blk17p1En = 0,
							.decBy2Hb4Blk17p2En = 0,
							.decBy2Hb5Blk19p1En = 0,
							.decBy2Hb6Blk19p2En = 0
						},
						.rxSincHBTop =
						{
							.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
							.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_ZERO,
							.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB1,
							.isGainCompEnabled = 0,
							.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
							.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
						},
						.rxNbDem =
						{
							.dpInFifo =
							{
								.dpInFifoEn = 0,
								.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
								.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
							},
							.rxNbNco =
							{
								.rxNbNcoEn = 0,
								.rxNbNcoConfig =
								{
									.freq = 0,
									.sampleFreq = 0,
									.phase = 0,
									.realOut = 0
								}
							},
							.rxWbNbCompPFir =
							{
								.bankSel = ADI_ADRV9001_PFIR_BANK_A,
								.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
								.rxWbNbCompPFirEn = 1
							},
							.resamp =
							{
								.rxResampEn = 0,
								.resampPhaseI = 0,
								.resampPhaseQ = 0
							},
							.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
							.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
							.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
							.dpArmSel = ADI_ADRV9001_DP_SEL
						}
					},
					.rxSsiConfig =
					{
						.ssiType = ADI_ADRV9001_SSI_TYPE_DISABLE,
						.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_2_BIT_SYMBOL_DATA,
						.numLaneSel = ADI_ADRV9001_SSI_1_LANE,
						.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
						.lsbFirst = 0,
						.qFirst = 0,
						.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED,
						.lvdsBitInversion = 0,
						.lvdsUseLsbIn12bitMode = 0,
						.lvdsTxFullRefClkEn = false,
						.lvdsRxClkInversionEn = false,
						.cmosTxDdrNegStrobeEn = false,
						.cmosDdrPosClkEn = false,
						.cmosDdrClkInversionEn = false,
						.cmosDdrEn = false,
						.rxMaskStrobeEn = false
					}
				}
			},
			{
				.profile =
				{
					.primarySigBandwidth_Hz = 12500,
					.rxOutputRate_Hz = 0,
					.rxInterfaceSampleRate_Hz = 0,
					.rxOffsetLo_kHz = 0,
					.rxSignalOnLo = 0,
					.outputSignaling = ADI_ADRV9001_RX_IQ,
					.filterOrder = 1,
					.filterOrderLp = 1,
					.hpAdcCorner = 0,
					.lpAdcCorner = 0,
					.adcClk_kHz = 0,
					.rxCorner3dB_kHz = 0,
					.rxCorner3dBLp_kHz = 0,
					.tiaPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.tiaPowerLp = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.channelType = 0,
					.adcType = ADI_ADRV9001_ADC_HP,
					.lpAdcCalMode = ADI_ADRV9001_ADC_LOWPOWER_PERIODIC,
					.rxDpProfile =
					{
						.rxNbDecTop =
						{
							.scicBlk23En = 0,
							.scicBlk23DivFactor = 0,
							.scicBlk23LowRippleEn = 0,
							.decBy2Blk35En = 0,
							.decBy2Blk37En = 0,
							.decBy2Blk39En = 0,
							.decBy2Blk41En = 0,
							.decBy2Blk43En = 0,
							.decBy3Blk45En = 0,
							.decBy2Blk47En = 0
						},
						.rxWbDecTop =
						{
							.decBy2Blk25En = 0,
							.decBy2Blk27En = 0,
							.decBy2Blk29En = 0,
							.decBy2Blk31En = 0,
							.decBy2Blk33En = 0,
							.wbLpfBlk33p1En = 0
						},
						.rxDecTop =
						{
							.decBy3Blk15En = 0,
							.decBy2Hb3Blk17p1En = 0,
							.decBy2Hb4Blk17p2En = 0,
							.decBy2Hb5Blk19p1En = 0,
							.decBy2Hb6Blk19p2En = 0
						},
						.rxSincHBTop =
						{
							.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
							.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_ZERO,
							.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB1,
							.isGainCompEnabled = 0,
							.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
							.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
						},
						.rxNbDem =
						{
							.dpInFifo =
							{
								.dpInFifoEn = 0,
								.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
								.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
							},
							.rxNbNco =
							{
								.rxNbNcoEn = 0,
								.rxNbNcoConfig =
								{
									.freq = 0,
									.sampleFreq = 0,
									.phase = 0,
									.realOut = 0
								}
							},
							.rxWbNbCompPFir =
							{
								.bankSel = ADI_ADRV9001_PFIR_BANK_A,
								.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
								.rxWbNbCompPFirEn = 1
							},
							.resamp =
							{
								.rxResampEn = 0,
								.resampPhaseI = 0,
								.resampPhaseQ = 0
							},
							.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
							.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
							.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
							.dpArmSel = ADI_ADRV9001_DP_SEL
						}
					},
					.rxSsiConfig =
					{
						.ssiType = ADI_ADRV9001_SSI_TYPE_DISABLE,
						.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_2_BIT_SYMBOL_DATA,
						.numLaneSel = ADI_ADRV9001_SSI_1_LANE,
						.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
						.lsbFirst = 0,
						.qFirst = 0,
						.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED,
						.lvdsBitInversion = 0,
						.lvdsUseLsbIn12bitMode = 0,
						.lvdsTxFullRefClkEn = false,
						.lvdsRxClkInversionEn = false,
						.cmosTxDdrNegStrobeEn = false,
						.cmosDdrPosClkEn = false,
						.cmosDdrClkInversionEn = false,
						.cmosDdrEn = false,
						.rxMaskStrobeEn = false
					}
				}
			}
		}
	},
	.tx = {
		.txInitChannelMask = 12,
		.txProfile = {
			{
				.primarySigBandwidth_Hz = 9000000,
				.txInputRate_Hz = 15360000,
				.txInterfaceSampleRate_Hz = 15360000,
				.txOffsetLo_kHz = 0,
				.validDataDelay = 0,
				.txBbf3dBCorner_kHz = 50000,
				.outputSignaling = ADI_ADRV9001_TX_IQ,
				.txPdBiasCurrent = 1,
				.txPdGainEnable = 0,
				.txPrePdRealPole_kHz = 1000000,
				.txPostPdRealPole_kHz = 530000,
				.txBbfPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
				.txExtLoopBackType = 0,
				.txExtLoopBackForInitCal = 0,
				.txPeakLoopBackPower = 0,
				.frequencyDeviation_Hz = 0,
				.txDpProfile =
				{
					.txPreProc =
					{
						.txPreProcSymbol0 = 0,
						.txPreProcSymbol1 = 0,
						.txPreProcSymbol2 = 0,
						.txPreProcSymbol3 = 0,
						.txPreProcSymMapDivFactor = 1,
						.txPreProcMode = ADI_ADRV9001_TX_DP_PREPROC_IQ_DATA_WITH_PFIRS,
						.txPreProcWbNbPfirIBankSel = ADI_ADRV9001_PFIR_BANK_A,
						.txPreProcWbNbPfirQBankSel = ADI_ADRV9001_PFIR_BANK_B
					},
					.txWbIntTop =
					{
						.txInterpBy2Blk30En = 0,
						.txInterpBy2Blk28En = 0,
						.txInterpBy2Blk26En = 0,
						.txInterpBy2Blk24En = 1,
						.txInterpBy2Blk22En = 1,
						.txWbLpfBlk22p1En = 0
					},
					.txNbIntTop =
					{
						.txInterpBy2Blk20En = 0,
						.txInterpBy2Blk18En = 0,
						.txInterpBy2Blk16En = 0,
						.txInterpBy2Blk14En = 0,
						.txInterpBy2Blk12En = 0,
						.txInterpBy3Blk10En = 0,
						.txInterpBy2Blk8En = 0,
						.txScicBlk32En = 0,
						.txScicBlk32DivFactor = 1
					},
					.txIntTop =
					{
						.interpBy3Blk44p1En = 1,
						.sinc3Blk44En = 0,
						.sinc2Blk42En = 0,
						.interpBy3Blk40En = 1,
						.interpBy2Blk38En = 0,
						.interpBy2Blk36En = 0
					},
					.txIntTopFreqDevMap =
					{
						.rrc2Frac = 0,
						.mpll = 0,
						.nchLsw = 0,
						.nchMsb = 0,
						.freqDevMapEn = 0,
						.txRoundEn = 1
					},
					.txIqdmDuc =
					{
						.iqdmDucMode = ADI_ADRV9001_TX_DP_IQDMDUC_MODE0,
						.iqdmDev = 0,
						.iqdmDevOffset = 0,
						.iqdmScalar = 0,
						.iqdmThreshold = 0,
						.iqdmNco =
						{
							.freq = 0,
							.sampleFreq = 61440000,
							.phase = 0,
							.realOut = 0
						}
					}
				},
				.txSsiConfig =
				{
					.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
					.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
					.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
					.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
					.lsbFirst = 0,
					.qFirst = 0,
					.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DCLK_OUT_ENABLED,
					.lvdsBitInversion = 0,
					.lvdsUseLsbIn12bitMode = 0,
					.lvdsTxFullRefClkEn = false,
					.lvdsRxClkInversionEn = false,
					.cmosTxDdrNegStrobeEn = false,
					.cmosDdrPosClkEn = false,
					.cmosDdrClkInversionEn = false,
					.cmosDdrEn = false,
					.rxMaskStrobeEn = false
				}
			},
			{
				.primarySigBandwidth_Hz = 9000000,
				.txInputRate_Hz = 15360000,
				.txInterfaceSampleRate_Hz = 15360000,
				.txOffsetLo_kHz = 0,
				.validDataDelay = 0,
				.txBbf3dBCorner_kHz = 50000,
				.outputSignaling = ADI_ADRV9001_TX_IQ,
				.txPdBiasCurrent = 1,
				.txPdGainEnable = 0,
				.txPrePdRealPole_kHz = 1000000,
				.txPostPdRealPole_kHz = 530000,
				.txBbfPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
				.txExtLoopBackType = 0,
				.txExtLoopBackForInitCal = 0,
				.txPeakLoopBackPower = 0,
				.frequencyDeviation_Hz = 0,
				.txDpProfile =
				{
					.txPreProc =
					{
						.txPreProcSymbol0 = 0,
						.txPreProcSymbol1 = 0,
						.txPreProcSymbol2 = 0,
						.txPreProcSymbol3 = 0,
						.txPreProcSymMapDivFactor = 1,
						.txPreProcMode = ADI_ADRV9001_TX_DP_PREPROC_IQ_DATA_WITH_PFIRS,
						.txPreProcWbNbPfirIBankSel = ADI_ADRV9001_PFIR_BANK_C,
						.txPreProcWbNbPfirQBankSel = ADI_ADRV9001_PFIR_BANK_D
					},
					.txWbIntTop =
					{
						.txInterpBy2Blk30En = 0,
						.txInterpBy2Blk28En = 0,
						.txInterpBy2Blk26En = 0,
						.txInterpBy2Blk24En = 1,
						.txInterpBy2Blk22En = 1,
						.txWbLpfBlk22p1En = 0
					},
					.txNbIntTop =
					{
						.txInterpBy2Blk20En = 0,
						.txInterpBy2Blk18En = 0,
						.txInterpBy2Blk16En = 0,
						.txInterpBy2Blk14En = 0,
						.txInterpBy2Blk12En = 0,
						.txInterpBy3Blk10En = 0,
						.txInterpBy2Blk8En = 0,
						.txScicBlk32En = 0,
						.txScicBlk32DivFactor = 1
					},
					.txIntTop =
					{
						.interpBy3Blk44p1En = 1,
						.sinc3Blk44En = 0,
						.sinc2Blk42En = 0,
						.interpBy3Blk40En = 1,
						.interpBy2Blk38En = 0,
						.interpBy2Blk36En = 0
					},
					.txIntTopFreqDevMap =
					{
						.rrc2Frac = 0,
						.mpll = 0,
						.nchLsw = 0,
						.nchMsb = 0,
						.freqDevMapEn = 0,
						.txRoundEn = 1
					},
					.txIqdmDuc =
					{
						.iqdmDucMode = ADI_ADRV9001_TX_DP_IQDMDUC_MODE0,
						.iqdmDev = 0,
						.iqdmDevOffset = 0,
						.iqdmScalar = 0,
						.iqdmThreshold = 0,
						.iqdmNco =
						{
							.freq = 0,
							.sampleFreq = 61440000,
							.phase = 0,
							.realOut = 0
						}
					}
				},
				.txSsiConfig =
				{
					.ssiType = ADI_ADRV9001_SSI_TYPE_LVDS,
					.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
					.numLaneSel = ADI_ADRV9001_SSI_2_LANE,
					.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
					.lsbFirst = 0,
					.qFirst = 0,
					.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DCLK_OUT_ENABLED,
					.lvdsBitInversion = 0,
					.lvdsUseLsbIn12bitMode = 0,
					.lvdsTxFullRefClkEn = false,
					.lvdsRxClkInversionEn = false,
					.cmosTxDdrNegStrobeEn = false,
					.cmosDdrPosClkEn = false,
					.cmosDdrClkInversionEn = false,
					.cmosDdrEn = false,
					.rxMaskStrobeEn = false
				}
			}
		}
	},
	.sysConfig = {
		.duplexMode = ADI_ADRV9001_FDD_MODE,
		.fhModeOn = 0,
		.numDynamicProfile = 1,
		.mcsMode = ADI_ADRV9001_MCSMODE_DISABLED,
		.adcTypeMonitor = ADI_ADRV9001_ADC_HP,
		.pllLockTime_us = 750,
		.pllModulus = {
				.modulus = { 8388593, 8388593, 8388593, 8388593, 8388593 },
				.dmModulus = { 8388593, 8388593 }
			}
	},
	.pfirBuffer = {
		.pfirRxWbNbChFilterCoeff_A = {
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 475, 312, -782, -39, 1201, -777, -1182, 1981, 177, -2874, 1941, 2393, -4416, 225, 5594, -4581, -3668, 8650, -1992, -9342, 9646, 4213, -15137, 6404, 13615, -18199, -2610, 23969, -15142, -17198, 31204, -3269, -34604, 30213, 17955, -49337, 16361, 45636, -53954, -12567, 72920, -40769, -54562, 89506, -4148, -102269, 83183, 57280, -142874, 41767, 139213, -158628, -45955, 231679, -125964, -193870, 320642, -4532, -442087, 390927, 347244, -1055854, 429729, 4391599, 4391599, 429729, -1055854, 347244, 390927, -442087, -4532, 320642, -193870, -125964, 231679, -45955, -158628, 139213, 41767, -142874, 57280, 83183, -102269, -4148, 89506, -54562, -40769, 72920, -12567, -53954, 45636, 16361, -49337, 17955, 30213, -34604, -3269, 31204, -17198, -15142, 23969, -2610, -18199, 13615, 6404, -15137, 4213, 9646, -9342, -1992, 8650, -3668, -4581, 5594, 225, -4416, 2393, 1941, -2874, 177, 1981, -1182, -777, 1201, -39, -782, 312, 0 }
		},
		.pfirRxWbNbChFilterCoeff_B = {
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8388608, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirRxWbNbChFilterCoeff_C = {
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 475, 312, -782, -39, 1201, -777, -1182, 1981, 177, -2874, 1941, 2393, -4416, 225, 5594, -4581, -3668, 8650, -1992, -9342, 9646, 4213, -15137, 6404, 13615, -18199, -2610, 23969, -15142, -17198, 31204, -3269, -34604, 30213, 17955, -49337, 16361, 45636, -53954, -12567, 72920, -40769, -54562, 89506, -4148, -102269, 83183, 57280, -142874, 41767, 139213, -158628, -45955, 231679, -125964, -193870, 320642, -4532, -442087, 390927, 347244, -1055854, 429729, 4391599, 4391599, 429729, -1055854, 347244, 390927, -442087, -4532, 320642, -193870, -125964, 231679, -45955, -158628, 139213, 41767, -142874, 57280, 83183, -102269, -4148, 89506, -54562, -40769, 72920, -12567, -53954, 45636, 16361, -49337, 17955, 30213, -34604, -3269, 31204, -17198, -15142, 23969, -2610, -18199, 13615, 6404, -15137, 4213, 9646, -9342, -1992, 8650, -3668, -4581, 5594, 225, -4416, 2393, 1941, -2874, 177, 1981, -1182, -777, 1201, -39, -782, 312, 0 }
		},
		.pfirRxWbNbChFilterCoeff_D = {
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8388608, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirTxWbNbPulShpCoeff_A = {
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirTxWbNbPulShpCoeff_B = {
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirTxWbNbPulShpCoeff_C = {
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirTxWbNbPulShpCoeff_D = {
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirRxNbPulShp = {
			{
				.numCoeff = 128,
				.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
				.taps = 128,
				.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
				.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8388608, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
			},
			{
				.numCoeff = 128,
				.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
				.taps = 128,
				.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
				.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8388608, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
			}
		},
		.pfirRxMagLowTiaLowSRHp = {
			{
				.numCoeff = 21,
				.coefficients = { -21, 111, -363, 870, -1718, 2932, -4416, 5958, -7152, 6556, 27252, 6556, -7152, 5958, -4416, 2932, -1718, 870, -363, 111, -21 }
			},
			{
				.numCoeff = 21,
				.coefficients = { -21, 111, -363, 870, -1718, 2932, -4416, 5958, -7152, 6556, 27252, 6556, -7152, 5958, -4416, 2932, -1718, 870, -363, 111, -21 }
			}
		},
		.pfirRxMagLowTiaHighSRHp = {
			{
				.numCoeff = 21,
				.coefficients = { -27, 14, 422, -1035, 109, 1789, 267, -4738, -1617, 11376, 19649, 11376, -1617, -4738, 267, 1789, 109, -1035, 422, 14, -27 }
			},
			{
				.numCoeff = 21,
				.coefficients = { -27, 14, 422, -1035, 109, 1789, 267, -4738, -1617, 11376, 19649, 11376, -1617, -4738, 267, 1789, 109, -1035, 422, 14, -27 }
			}
		},
		.pfirRxMagHighTiaHighSRHp = {
			{
				.numCoeff = 21,
				.coefficients = { 28, -160, 472, -885, 984, -36, -2594, 6779, -11030, 10718, 24218, 10718, -11030, 6779, -2594, -36, 984, -885, 472, -160, 28 }
			},
			{
				.numCoeff = 21,
				.coefficients = { 28, -160, 472, -885, 984, -36, -2594, 6779, -11030, 10718, 24218, 10718, -11030, 6779, -2594, -36, 984, -885, 472, -160, 28 }
			}
		},
		.pfirRxMagLowTiaLowSRLp = {
			{
				.numCoeff = 21,
				.coefficients = { -21, 111, -363, 870, -1717, 2931, -4415, 5957, -7153, 6560, 27245, 6560, -7153, 5957, -4415, 2931, -1717, 870, -363, 111, -21 }
			},
			{
				.numCoeff = 21,
				.coefficients = { -21, 111, -363, 870, -1717, 2931, -4415, 5957, -7153, 6560, 27245, 6560, -7153, 5957, -4415, 2931, -1717, 870, -363, 111, -21 }
			}
		},
		.pfirRxMagLowTiaHighSRLp = {
			{
				.numCoeff = 21,
				.coefficients = { -28, 14, 420, -1033, 109, 1787, 264, -4732, -1611, 11374, 19637, 11374, -1611, -4732, 264, 1787, 109, -1033, 420, 14, -28 }
			},
			{
				.numCoeff = 21,
				.coefficients = { -28, 14, 420, -1033, 109, 1787, 264, -4732, -1611, 11374, 19637, 11374, -1611, -4732, 264, 1787, 109, -1033, 420, 14, -28 }
			}
		},
		.pfirRxMagHighTiaHighSRLp = {
			{
				.numCoeff = 21,
				.coefficients = { 28, -160, 472, -887, 988, -48, -2570, 6741, -10987, 10702, 24208, 10702, -10987, 6741, -2570, -48, 988, -887, 472, -160, 28 }
			},
			{
				.numCoeff = 21,
				.coefficients = { 28, -160, 472, -887, 988, -48, -2570, 6741, -10987, 10702, 24208, 10702, -10987, 6741, -2570, -48, 988, -887, 472, -160, 28 }
			}
		},
		.pfirTxMagComp1 = {
			.numCoeff = 21,
			.coefficients = { 8, -20, -56, 533, -1988, 5017, -9518, 13961, -13377, 6783, 30083, 6783, -13377, 13961, -9518, 5017, -1988, 533, -56, -20, 8 }
		},
		.pfirTxMagComp2 = {
			.numCoeff = 21,
			.coefficients = { 8, -20, -56, 533, -1988, 5017, -9518, 13961, -13377, 6783, 30083, 6783, -13377, 13961, -9518, 5017, -1988, 533, -56, -20, 8 }
		},
		.pfirTxMagCompNb = {
			{
				.numCoeff = 13,
				.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
			},
			{
				.numCoeff = 13,
				.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
			}
		},
		.pfirRxMagCompNb = {
			{
				.numCoeff = 13,
				.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
			},
			{
				.numCoeff = 13,
				.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
			}
		}
	}
};
/* different profile for CMOS - 1.4MHz BW */
static struct adi_adrv9001_Init adrv9002_init_cmos = {
	.clocks = {
		.deviceClock_kHz = 38400,
		.clkPllVcoFreq_daHz = 884736000,
		.clkPllHsDiv = ADI_ADRV9001_HSDIV_4,
		.clkPllMode = ADI_ADRV9001_CLK_PLL_HP_MODE,
		.clk1105Div = ADI_ADRV9001_INTERNAL_CLOCK_DIV_2,
		.armClkDiv = ADI_ADRV9001_INTERNAL_CLOCK_DIV_6,
		.armPowerSavingClkDiv = 1,
		.refClockOutEnable = true,
		.auxPllPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
		.clkPllPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
		.padRefClkDrv = 0,
		.extLo1OutFreq_kHz = 0,
		.extLo2OutFreq_kHz = 0,
		.rfPll1LoMode = ADI_ADRV9001_INT_LO1,
		.rfPll2LoMode = ADI_ADRV9001_INT_LO1,
		.ext1LoType = ADI_ADRV9001_EXT_LO_DIFFERENTIAL,
		.ext2LoType = ADI_ADRV9001_EXT_LO_DIFFERENTIAL,
		.rx1RfInputSel = ADI_ADRV9001_RX_A,
		.rx2RfInputSel = ADI_ADRV9001_RX_A,
		.extLo1Divider = 2,
		.extLo2Divider = 2,
		.rfPllPhaseSyncMode = ADI_ADRV9001_RFPLLMCS_NOSYNC,
		.rx1LoSelect = ADI_ADRV9001_LOSEL_LO2,
		.rx2LoSelect = ADI_ADRV9001_LOSEL_LO2,
		.tx1LoSelect = ADI_ADRV9001_LOSEL_LO1,
		.tx2LoSelect = ADI_ADRV9001_LOSEL_LO1,
		.rx1LoDivMode = ADI_ADRV9001_LO_DIV_MODE_LOW_POWER,
		.rx2LoDivMode = ADI_ADRV9001_LO_DIV_MODE_LOW_POWER,
		.tx1LoDivMode = ADI_ADRV9001_LO_DIV_MODE_LOW_POWER,
		.tx2LoDivMode = ADI_ADRV9001_LO_DIV_MODE_LOW_POWER,
		.loGen1Select = ADI_ADRV9001_LOGENPOWER_RFPLL_LDO,
		.loGen2Select = ADI_ADRV9001_LOGENPOWER_RFPLL_LDO
	},
	.rx = {
		.rxInitChannelMask = 243,
		.rxChannelCfg = {
			{
				.profile =
				{
					.primarySigBandwidth_Hz = 1008000,
					.rxOutputRate_Hz = 1920000,
					.rxInterfaceSampleRate_Hz = 1920000,
					.rxOffsetLo_kHz = 0,
					.rxSignalOnLo = 0,
					.outputSignaling = ADI_ADRV9001_RX_IQ,
					.filterOrder = 1,
					.filterOrderLp = 1,
					.hpAdcCorner = 20000000,
					.lpAdcCorner = 0,
					.adcClk_kHz = 2211840,
					.rxCorner3dB_kHz = 40000,
					.rxCorner3dBLp_kHz = 40000,
					.tiaPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.tiaPowerLp = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.channelType = 1,
					.adcType = ADI_ADRV9001_ADC_HP,
					.lpAdcCalMode = ADI_ADRV9001_ADC_LOWPOWER_PERIODIC,
					.rxDpProfile =
					{
						.rxNbDecTop =
						{
							.scicBlk23En = 0,
							.scicBlk23DivFactor = 1,
							.scicBlk23LowRippleEn = 0,
							.decBy2Blk35En = 0,
							.decBy2Blk37En = 0,
							.decBy2Blk39En = 0,
							.decBy2Blk41En = 0,
							.decBy2Blk43En = 0,
							.decBy3Blk45En = 0,
							.decBy2Blk47En = 0
						},
						.rxWbDecTop =
						{
							.decBy2Blk25En = 1,
							.decBy2Blk27En = 1,
							.decBy2Blk29En = 1,
							.decBy2Blk31En = 1,
							.decBy2Blk33En = 1,
							.wbLpfBlk33p1En = 0
						},
						.rxDecTop =
						{
							.decBy3Blk15En = 1,
							.decBy2Hb3Blk17p1En = 0,
							.decBy2Hb4Blk17p2En = 0,
							.decBy2Hb5Blk19p1En = 0,
							.decBy2Hb6Blk19p2En = 0
						},
						.rxSincHBTop =
						{
							.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
							.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
							.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB1,
							.isGainCompEnabled = 0,
							.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
							.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
						},
						.rxNbDem =
						{
							.dpInFifo =
							{
								.dpInFifoEn = 0,
								.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
								.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
							},
							.rxNbNco =
							{
								.rxNbNcoEn = 0,
								.rxNbNcoConfig =
								{
									.freq = 0,
									.sampleFreq = 0,
									.phase = 0,
									.realOut = 0
								}
							},
							.rxWbNbCompPFir =
							{
								.bankSel = ADI_ADRV9001_PFIR_BANK_A,
								.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
								.rxWbNbCompPFirEn = 1
							},
							.resamp =
							{
								.rxResampEn = 0,
								.resampPhaseI = 0,
								.resampPhaseQ = 0
							},
							.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
							.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
							.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
							.dpArmSel = ADI_ADRV9001_DP_SEL
						}
					},
					.rxSsiConfig =
					{
						.ssiType = ADI_ADRV9001_SSI_TYPE_CMOS,
						.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
						.numLaneSel = ADI_ADRV9001_SSI_1_LANE,
						.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
						.lsbFirst = 0,
						.qFirst = 0,
						.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED,
						.lvdsBitInversion = 0,
						.lvdsUseLsbIn12bitMode = 0,
						.lvdsTxFullRefClkEn = false,
						.lvdsRxClkInversionEn = false,
						.cmosTxDdrNegStrobeEn = false,
						.cmosDdrPosClkEn = false,
						.cmosDdrClkInversionEn = false,
						.cmosDdrEn = false,
						.rxMaskStrobeEn = false
					}
				}
			},
			{
				.profile =
				{
					.primarySigBandwidth_Hz = 1008000,
					.rxOutputRate_Hz = 1920000,
					.rxInterfaceSampleRate_Hz = 1920000,
					.rxOffsetLo_kHz = 0,
					.rxSignalOnLo = 0,
					.outputSignaling = ADI_ADRV9001_RX_IQ,
					.filterOrder = 1,
					.filterOrderLp = 1,
					.hpAdcCorner = 20000000,
					.lpAdcCorner = 0,
					.adcClk_kHz = 2211840,
					.rxCorner3dB_kHz = 40000,
					.rxCorner3dBLp_kHz = 40000,
					.tiaPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.tiaPowerLp = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.channelType = 2,
					.adcType = ADI_ADRV9001_ADC_HP,
					.lpAdcCalMode = ADI_ADRV9001_ADC_LOWPOWER_PERIODIC,
					.rxDpProfile =
					{
						.rxNbDecTop =
						{
							.scicBlk23En = 0,
							.scicBlk23DivFactor = 1,
							.scicBlk23LowRippleEn = 0,
							.decBy2Blk35En = 0,
							.decBy2Blk37En = 0,
							.decBy2Blk39En = 0,
							.decBy2Blk41En = 0,
							.decBy2Blk43En = 0,
							.decBy3Blk45En = 0,
							.decBy2Blk47En = 0
						},
						.rxWbDecTop =
						{
							.decBy2Blk25En = 1,
							.decBy2Blk27En = 1,
							.decBy2Blk29En = 1,
							.decBy2Blk31En = 1,
							.decBy2Blk33En = 1,
							.wbLpfBlk33p1En = 0
						},
						.rxDecTop =
						{
							.decBy3Blk15En = 1,
							.decBy2Hb3Blk17p1En = 0,
							.decBy2Hb4Blk17p2En = 0,
							.decBy2Hb5Blk19p1En = 0,
							.decBy2Hb6Blk19p2En = 0
						},
						.rxSincHBTop =
						{
							.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
							.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
							.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB1,
							.isGainCompEnabled = 0,
							.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
							.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
						},
						.rxNbDem =
						{
							.dpInFifo =
							{
								.dpInFifoEn = 0,
								.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
								.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
							},
							.rxNbNco =
							{
								.rxNbNcoEn = 0,
								.rxNbNcoConfig =
								{
									.freq = 0,
									.sampleFreq = 0,
									.phase = 0,
									.realOut = 0
								}
							},
							.rxWbNbCompPFir =
							{
								.bankSel = ADI_ADRV9001_PFIR_BANK_C,
								.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
								.rxWbNbCompPFirEn = 1
							},
							.resamp =
							{
								.rxResampEn = 0,
								.resampPhaseI = 0,
								.resampPhaseQ = 0
							},
							.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
							.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
							.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
							.dpArmSel = ADI_ADRV9001_DP_SEL
						}
					},
					.rxSsiConfig =
					{
						.ssiType = ADI_ADRV9001_SSI_TYPE_CMOS,
						.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
						.numLaneSel = ADI_ADRV9001_SSI_1_LANE,
						.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
						.lsbFirst = 0,
						.qFirst = 0,
						.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED,
						.lvdsBitInversion = 0,
						.lvdsUseLsbIn12bitMode = 0,
						.lvdsTxFullRefClkEn = false,
						.lvdsRxClkInversionEn = false,
						.cmosTxDdrNegStrobeEn = false,
						.cmosDdrPosClkEn = false,
						.cmosDdrClkInversionEn = false,
						.cmosDdrEn = false,
						.rxMaskStrobeEn = false
					}
				}
			},
			{
				.profile =
				{
					.primarySigBandwidth_Hz = 1008000,
					.rxOutputRate_Hz = 1920000,
					.rxInterfaceSampleRate_Hz = 1920000,
					.rxOffsetLo_kHz = 0,
					.rxSignalOnLo = 0,
					.outputSignaling = ADI_ADRV9001_RX_IQ,
					.filterOrder = 1,
					.filterOrderLp = 1,
					.hpAdcCorner = 50000000,
					.lpAdcCorner = 0,
					.adcClk_kHz = 2211840,
					.rxCorner3dB_kHz = 100000,
					.rxCorner3dBLp_kHz = 100000,
					.tiaPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.tiaPowerLp = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.channelType = 16,
					.adcType = ADI_ADRV9001_ADC_HP,
					.lpAdcCalMode = ADI_ADRV9001_ADC_LOWPOWER_PERIODIC,
					.rxDpProfile =
					{
						.rxNbDecTop =
						{
							.scicBlk23En = 0,
							.scicBlk23DivFactor = 1,
							.scicBlk23LowRippleEn = 0,
							.decBy2Blk35En = 0,
							.decBy2Blk37En = 0,
							.decBy2Blk39En = 0,
							.decBy2Blk41En = 0,
							.decBy2Blk43En = 0,
							.decBy3Blk45En = 0,
							.decBy2Blk47En = 0
						},
						.rxWbDecTop =
						{
							.decBy2Blk25En = 1,
							.decBy2Blk27En = 1,
							.decBy2Blk29En = 1,
							.decBy2Blk31En = 1,
							.decBy2Blk33En = 1,
							.wbLpfBlk33p1En = 0
						},
						.rxDecTop =
						{
							.decBy3Blk15En = 1,
							.decBy2Hb3Blk17p1En = 0,
							.decBy2Hb4Blk17p2En = 0,
							.decBy2Hb5Blk19p1En = 0,
							.decBy2Hb6Blk19p2En = 0
						},
						.rxSincHBTop =
						{
							.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
							.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
							.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB2,
							.isGainCompEnabled = 0,
							.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
							.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
						},
						.rxNbDem =
						{
							.dpInFifo =
							{
								.dpInFifoEn = 0,
								.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
								.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
							},
							.rxNbNco =
							{
								.rxNbNcoEn = 0,
								.rxNbNcoConfig =
								{
									.freq = 0,
									.sampleFreq = 0,
									.phase = 0,
									.realOut = 0
								}
							},
							.rxWbNbCompPFir =
							{
								.bankSel = ADI_ADRV9001_PFIR_BANK_B,
								.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
								.rxWbNbCompPFirEn = 1
							},
							.resamp =
							{
								.rxResampEn = 0,
								.resampPhaseI = 0,
								.resampPhaseQ = 0
							},
							.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
							.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
							.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
							.dpArmSel = ADI_ADRV9001_DP_SEL
						}
					},
					.rxSsiConfig =
					{
						.ssiType = ADI_ADRV9001_SSI_TYPE_CMOS,
						.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
						.numLaneSel = ADI_ADRV9001_SSI_1_LANE,
						.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
						.lsbFirst = 0,
						.qFirst = 0,
						.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED,
						.lvdsBitInversion = 0,
						.lvdsUseLsbIn12bitMode = 0,
						.lvdsTxFullRefClkEn = false,
						.lvdsRxClkInversionEn = false,
						.cmosTxDdrNegStrobeEn = false,
						.cmosDdrPosClkEn = false,
						.cmosDdrClkInversionEn = false,
						.cmosDdrEn = false,
						.rxMaskStrobeEn = false
					}
				}
			},
			{
				.profile =
				{
					.primarySigBandwidth_Hz = 1008000,
					.rxOutputRate_Hz = 1920000,
					.rxInterfaceSampleRate_Hz = 1920000,
					.rxOffsetLo_kHz = 0,
					.rxSignalOnLo = 0,
					.outputSignaling = ADI_ADRV9001_RX_IQ,
					.filterOrder = 1,
					.filterOrderLp = 1,
					.hpAdcCorner = 50000000,
					.lpAdcCorner = 0,
					.adcClk_kHz = 2211840,
					.rxCorner3dB_kHz = 100000,
					.rxCorner3dBLp_kHz = 100000,
					.tiaPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.tiaPowerLp = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.channelType = 32,
					.adcType = ADI_ADRV9001_ADC_HP,
					.lpAdcCalMode = ADI_ADRV9001_ADC_LOWPOWER_PERIODIC,
					.rxDpProfile =
					{
						.rxNbDecTop =
						{
							.scicBlk23En = 0,
							.scicBlk23DivFactor = 1,
							.scicBlk23LowRippleEn = 0,
							.decBy2Blk35En = 0,
							.decBy2Blk37En = 0,
							.decBy2Blk39En = 0,
							.decBy2Blk41En = 0,
							.decBy2Blk43En = 0,
							.decBy3Blk45En = 0,
							.decBy2Blk47En = 0
						},
						.rxWbDecTop =
						{
							.decBy2Blk25En = 1,
							.decBy2Blk27En = 1,
							.decBy2Blk29En = 1,
							.decBy2Blk31En = 1,
							.decBy2Blk33En = 1,
							.wbLpfBlk33p1En = 0
						},
						.rxDecTop =
						{
							.decBy3Blk15En = 1,
							.decBy2Hb3Blk17p1En = 0,
							.decBy2Hb4Blk17p2En = 0,
							.decBy2Hb5Blk19p1En = 0,
							.decBy2Hb6Blk19p2En = 0
						},
						.rxSincHBTop =
						{
							.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
							.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
							.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB2,
							.isGainCompEnabled = 0,
							.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
							.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
						},
						.rxNbDem =
						{
							.dpInFifo =
							{
								.dpInFifoEn = 0,
								.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
								.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
							},
							.rxNbNco =
							{
								.rxNbNcoEn = 0,
								.rxNbNcoConfig =
								{
									.freq = 0,
									.sampleFreq = 0,
									.phase = 0,
									.realOut = 0
								}
							},
							.rxWbNbCompPFir =
							{
								.bankSel = ADI_ADRV9001_PFIR_BANK_D,
								.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
								.rxWbNbCompPFirEn = 1
							},
							.resamp =
							{
								.rxResampEn = 0,
								.resampPhaseI = 0,
								.resampPhaseQ = 0
							},
							.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
							.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
							.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
							.dpArmSel = ADI_ADRV9001_DP_SEL
						}
					},
					.rxSsiConfig =
					{
						.ssiType = ADI_ADRV9001_SSI_TYPE_CMOS,
						.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
						.numLaneSel = ADI_ADRV9001_SSI_1_LANE,
						.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
						.lsbFirst = 0,
						.qFirst = 0,
						.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED,
						.lvdsBitInversion = 0,
						.lvdsUseLsbIn12bitMode = 0,
						.lvdsTxFullRefClkEn = false,
						.lvdsRxClkInversionEn = false,
						.cmosTxDdrNegStrobeEn = false,
						.cmosDdrPosClkEn = false,
						.cmosDdrClkInversionEn = false,
						.cmosDdrEn = false,
						.rxMaskStrobeEn = false
					}
				}
			},
			{
				.profile =
				{
					.primarySigBandwidth_Hz = 1008000,
					.rxOutputRate_Hz = 1920000,
					.rxInterfaceSampleRate_Hz = 1920000,
					.rxOffsetLo_kHz = 0,
					.rxSignalOnLo = 0,
					.outputSignaling = ADI_ADRV9001_RX_IQ,
					.filterOrder = 1,
					.filterOrderLp = 1,
					.hpAdcCorner = 50000000,
					.lpAdcCorner = 0,
					.adcClk_kHz = 2211840,
					.rxCorner3dB_kHz = 100000,
					.rxCorner3dBLp_kHz = 100000,
					.tiaPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.tiaPowerLp = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.channelType = 64,
					.adcType = ADI_ADRV9001_ADC_HP,
					.lpAdcCalMode = ADI_ADRV9001_ADC_LOWPOWER_PERIODIC,
					.rxDpProfile =
					{
						.rxNbDecTop =
						{
							.scicBlk23En = 0,
							.scicBlk23DivFactor = 1,
							.scicBlk23LowRippleEn = 0,
							.decBy2Blk35En = 0,
							.decBy2Blk37En = 0,
							.decBy2Blk39En = 0,
							.decBy2Blk41En = 0,
							.decBy2Blk43En = 0,
							.decBy3Blk45En = 0,
							.decBy2Blk47En = 0
						},
						.rxWbDecTop =
						{
							.decBy2Blk25En = 1,
							.decBy2Blk27En = 1,
							.decBy2Blk29En = 1,
							.decBy2Blk31En = 1,
							.decBy2Blk33En = 1,
							.wbLpfBlk33p1En = 0
						},
						.rxDecTop =
						{
							.decBy3Blk15En = 1,
							.decBy2Hb3Blk17p1En = 0,
							.decBy2Hb4Blk17p2En = 0,
							.decBy2Hb5Blk19p1En = 0,
							.decBy2Hb6Blk19p2En = 0
						},
						.rxSincHBTop =
						{
							.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
							.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
							.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB2,
							.isGainCompEnabled = 0,
							.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
							.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
						},
						.rxNbDem =
						{
							.dpInFifo =
							{
								.dpInFifoEn = 0,
								.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
								.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
							},
							.rxNbNco =
							{
								.rxNbNcoEn = 0,
								.rxNbNcoConfig =
								{
									.freq = 0,
									.sampleFreq = 0,
									.phase = 0,
									.realOut = 0
								}
							},
							.rxWbNbCompPFir =
							{
								.bankSel = ADI_ADRV9001_PFIR_BANK_B,
								.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
								.rxWbNbCompPFirEn = 1
							},
							.resamp =
							{
								.rxResampEn = 0,
								.resampPhaseI = 0,
								.resampPhaseQ = 0
							},
							.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
							.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
							.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
							.dpArmSel = ADI_ADRV9001_DP_SEL
						}
					},
					.rxSsiConfig =
					{
						.ssiType = ADI_ADRV9001_SSI_TYPE_CMOS,
						.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
						.numLaneSel = ADI_ADRV9001_SSI_1_LANE,
						.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
						.lsbFirst = 0,
						.qFirst = 0,
						.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED,
						.lvdsBitInversion = 0,
						.lvdsUseLsbIn12bitMode = 0,
						.lvdsTxFullRefClkEn = false,
						.lvdsRxClkInversionEn = false,
						.cmosTxDdrNegStrobeEn = false,
						.cmosDdrPosClkEn = false,
						.cmosDdrClkInversionEn = false,
						.cmosDdrEn = false,
						.rxMaskStrobeEn = false
					}
				}
			},
			{
				.profile =
				{
					.primarySigBandwidth_Hz = 1008000,
					.rxOutputRate_Hz = 1920000,
					.rxInterfaceSampleRate_Hz = 1920000,
					.rxOffsetLo_kHz = 0,
					.rxSignalOnLo = 0,
					.outputSignaling = ADI_ADRV9001_RX_IQ,
					.filterOrder = 1,
					.filterOrderLp = 1,
					.hpAdcCorner = 50000000,
					.lpAdcCorner = 0,
					.adcClk_kHz = 2211840,
					.rxCorner3dB_kHz = 100000,
					.rxCorner3dBLp_kHz = 100000,
					.tiaPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.tiaPowerLp = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.channelType = 64,
					.adcType = ADI_ADRV9001_ADC_HP,
					.lpAdcCalMode = ADI_ADRV9001_ADC_LOWPOWER_PERIODIC,
					.rxDpProfile =
					{
						.rxNbDecTop =
						{
							.scicBlk23En = 0,
							.scicBlk23DivFactor = 1,
							.scicBlk23LowRippleEn = 0,
							.decBy2Blk35En = 0,
							.decBy2Blk37En = 0,
							.decBy2Blk39En = 0,
							.decBy2Blk41En = 0,
							.decBy2Blk43En = 0,
							.decBy3Blk45En = 0,
							.decBy2Blk47En = 0
						},
						.rxWbDecTop =
						{
							.decBy2Blk25En = 1,
							.decBy2Blk27En = 1,
							.decBy2Blk29En = 1,
							.decBy2Blk31En = 1,
							.decBy2Blk33En = 1,
							.wbLpfBlk33p1En = 0
						},
						.rxDecTop =
						{
							.decBy3Blk15En = 1,
							.decBy2Hb3Blk17p1En = 0,
							.decBy2Hb4Blk17p2En = 0,
							.decBy2Hb5Blk19p1En = 0,
							.decBy2Hb6Blk19p2En = 0
						},
						.rxSincHBTop =
						{
							.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
							.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6,
							.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB2,
							.isGainCompEnabled = 0,
							.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
							.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
						},
						.rxNbDem =
						{
							.dpInFifo =
							{
								.dpInFifoEn = 0,
								.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
								.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
							},
							.rxNbNco =
							{
								.rxNbNcoEn = 0,
								.rxNbNcoConfig =
								{
									.freq = 0,
									.sampleFreq = 0,
									.phase = 0,
									.realOut = 0
								}
							},
							.rxWbNbCompPFir =
							{
								.bankSel = ADI_ADRV9001_PFIR_BANK_D,
								.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
								.rxWbNbCompPFirEn = 1
							},
							.resamp =
							{
								.rxResampEn = 0,
								.resampPhaseI = 0,
								.resampPhaseQ = 0
							},
							.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
							.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
							.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
							.dpArmSel = ADI_ADRV9001_DP_SEL
						}
					},
					.rxSsiConfig =
					{
						.ssiType = ADI_ADRV9001_SSI_TYPE_CMOS,
						.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
						.numLaneSel = ADI_ADRV9001_SSI_1_LANE,
						.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
						.lsbFirst = 0,
						.qFirst = 0,
						.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED,
						.lvdsBitInversion = 0,
						.lvdsUseLsbIn12bitMode = 0,
						.lvdsTxFullRefClkEn = false,
						.lvdsRxClkInversionEn = false,
						.cmosTxDdrNegStrobeEn = false,
						.cmosDdrPosClkEn = false,
						.cmosDdrClkInversionEn = false,
						.cmosDdrEn = false,
						.rxMaskStrobeEn = false
					}
				}
			},
			{
				.profile =
				{
					.primarySigBandwidth_Hz = 12500,
					.rxOutputRate_Hz = 0,
					.rxInterfaceSampleRate_Hz = 0,
					.rxOffsetLo_kHz = 0,
					.rxSignalOnLo = 0,
					.outputSignaling = ADI_ADRV9001_RX_IQ,
					.filterOrder = 1,
					.filterOrderLp = 1,
					.hpAdcCorner = 0,
					.lpAdcCorner = 0,
					.adcClk_kHz = 0,
					.rxCorner3dB_kHz = 0,
					.rxCorner3dBLp_kHz = 0,
					.tiaPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.tiaPowerLp = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.channelType = 0,
					.adcType = ADI_ADRV9001_ADC_HP,
					.lpAdcCalMode = ADI_ADRV9001_ADC_LOWPOWER_PERIODIC,
					.rxDpProfile =
					{
						.rxNbDecTop =
						{
							.scicBlk23En = 0,
							.scicBlk23DivFactor = 0,
							.scicBlk23LowRippleEn = 0,
							.decBy2Blk35En = 0,
							.decBy2Blk37En = 0,
							.decBy2Blk39En = 0,
							.decBy2Blk41En = 0,
							.decBy2Blk43En = 0,
							.decBy3Blk45En = 0,
							.decBy2Blk47En = 0
						},
						.rxWbDecTop =
						{
							.decBy2Blk25En = 0,
							.decBy2Blk27En = 0,
							.decBy2Blk29En = 0,
							.decBy2Blk31En = 0,
							.decBy2Blk33En = 0,
							.wbLpfBlk33p1En = 0
						},
						.rxDecTop =
						{
							.decBy3Blk15En = 0,
							.decBy2Hb3Blk17p1En = 0,
							.decBy2Hb4Blk17p2En = 0,
							.decBy2Hb5Blk19p1En = 0,
							.decBy2Hb6Blk19p2En = 0
						},
						.rxSincHBTop =
						{
							.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
							.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_ZERO,
							.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB1,
							.isGainCompEnabled = 0,
							.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
							.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
						},
						.rxNbDem =
						{
							.dpInFifo =
							{
								.dpInFifoEn = 0,
								.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
								.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
							},
							.rxNbNco =
							{
								.rxNbNcoEn = 0,
								.rxNbNcoConfig =
								{
									.freq = 0,
									.sampleFreq = 0,
									.phase = 0,
									.realOut = 0
								}
							},
							.rxWbNbCompPFir =
							{
								.bankSel = ADI_ADRV9001_PFIR_BANK_A,
								.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
								.rxWbNbCompPFirEn = 1
							},
							.resamp =
							{
								.rxResampEn = 0,
								.resampPhaseI = 0,
								.resampPhaseQ = 0
							},
							.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
							.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
							.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
							.dpArmSel = ADI_ADRV9001_DP_SEL
						}
					},
					.rxSsiConfig =
					{
						.ssiType = ADI_ADRV9001_SSI_TYPE_DISABLE,
						.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_2_BIT_SYMBOL_DATA,
						.numLaneSel = ADI_ADRV9001_SSI_1_LANE,
						.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
						.lsbFirst = 0,
						.qFirst = 0,
						.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED,
						.lvdsBitInversion = 0,
						.lvdsUseLsbIn12bitMode = 0,
						.lvdsTxFullRefClkEn = false,
						.lvdsRxClkInversionEn = false,
						.cmosTxDdrNegStrobeEn = false,
						.cmosDdrPosClkEn = false,
						.cmosDdrClkInversionEn = false,
						.cmosDdrEn = false,
						.rxMaskStrobeEn = false
					}
				}
			},
			{
				.profile =
				{
					.primarySigBandwidth_Hz = 12500,
					.rxOutputRate_Hz = 0,
					.rxInterfaceSampleRate_Hz = 0,
					.rxOffsetLo_kHz = 0,
					.rxSignalOnLo = 0,
					.outputSignaling = ADI_ADRV9001_RX_IQ,
					.filterOrder = 1,
					.filterOrderLp = 1,
					.hpAdcCorner = 0,
					.lpAdcCorner = 0,
					.adcClk_kHz = 0,
					.rxCorner3dB_kHz = 0,
					.rxCorner3dBLp_kHz = 0,
					.tiaPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.tiaPowerLp = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
					.channelType = 0,
					.adcType = ADI_ADRV9001_ADC_HP,
					.lpAdcCalMode = ADI_ADRV9001_ADC_LOWPOWER_PERIODIC,
					.rxDpProfile =
					{
						.rxNbDecTop =
						{
							.scicBlk23En = 0,
							.scicBlk23DivFactor = 0,
							.scicBlk23LowRippleEn = 0,
							.decBy2Blk35En = 0,
							.decBy2Blk37En = 0,
							.decBy2Blk39En = 0,
							.decBy2Blk41En = 0,
							.decBy2Blk43En = 0,
							.decBy3Blk45En = 0,
							.decBy2Blk47En = 0
						},
						.rxWbDecTop =
						{
							.decBy2Blk25En = 0,
							.decBy2Blk27En = 0,
							.decBy2Blk29En = 0,
							.decBy2Blk31En = 0,
							.decBy2Blk33En = 0,
							.wbLpfBlk33p1En = 0
						},
						.rxDecTop =
						{
							.decBy3Blk15En = 0,
							.decBy2Hb3Blk17p1En = 0,
							.decBy2Hb4Blk17p2En = 0,
							.decBy2Hb5Blk19p1En = 0,
							.decBy2Hb6Blk19p2En = 0
						},
						.rxSincHBTop =
						{
							.sincGainMux = ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB,
							.sincMux = ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_ZERO,
							.hbMux = ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB1,
							.isGainCompEnabled = 0,
							.gainComp9GainI = { 16384, 16384, 16384, 16384, 16384, 16384 },
							.gainComp9GainQ = { 0, 0, 0, 0, 0, 0 }
						},
						.rxNbDem =
						{
							.dpInFifo =
							{
								.dpInFifoEn = 0,
								.dpInFifoMode = ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING,
								.dpInFifoTestDataSel = ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL
							},
							.rxNbNco =
							{
								.rxNbNcoEn = 0,
								.rxNbNcoConfig =
								{
									.freq = 0,
									.sampleFreq = 0,
									.phase = 0,
									.realOut = 0
								}
							},
							.rxWbNbCompPFir =
							{
								.bankSel = ADI_ADRV9001_PFIR_BANK_A,
								.rxWbNbCompPFirInMuxSel = ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN,
								.rxWbNbCompPFirEn = 1
							},
							.resamp =
							{
								.rxResampEn = 0,
								.resampPhaseI = 0,
								.resampPhaseQ = 0
							},
							.gsOutMuxSel = ADI_ADRV9001_GS_OUT_MUX_BYPASS,
							.rxOutSel = ADI_ADRV9001_RX_OUT_IQ_SEL,
							.rxRoundMode = ADI_ADRV9001_RX_ROUNDMODE_IQ,
							.dpArmSel = ADI_ADRV9001_DP_SEL
						}
					},
					.rxSsiConfig =
					{
						.ssiType = ADI_ADRV9001_SSI_TYPE_DISABLE,
						.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_2_BIT_SYMBOL_DATA,
						.numLaneSel = ADI_ADRV9001_SSI_1_LANE,
						.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
						.lsbFirst = 0,
						.qFirst = 0,
						.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED,
						.lvdsBitInversion = 0,
						.lvdsUseLsbIn12bitMode = 0,
						.lvdsTxFullRefClkEn = false,
						.lvdsRxClkInversionEn = false,
						.cmosTxDdrNegStrobeEn = false,
						.cmosDdrPosClkEn = false,
						.cmosDdrClkInversionEn = false,
						.cmosDdrEn = false,
						.rxMaskStrobeEn = false
					}
				}
			}
		}
	},
	.tx = {
		.txInitChannelMask = 12,
		.txProfile = {
			{
				.primarySigBandwidth_Hz = 1008000,
				.txInputRate_Hz = 1920000,
				.txInterfaceSampleRate_Hz = 1920000,
				.txOffsetLo_kHz = 0,
				.validDataDelay = 0,
				.txBbf3dBCorner_kHz = 50000,
				.outputSignaling = ADI_ADRV9001_TX_IQ,
				.txPdBiasCurrent = 1,
				.txPdGainEnable = 0,
				.txPrePdRealPole_kHz = 1000000,
				.txPostPdRealPole_kHz = 530000,
				.txBbfPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
				.txExtLoopBackType = 0,
				.txExtLoopBackForInitCal = 0,
				.txPeakLoopBackPower = 0,
				.frequencyDeviation_Hz = 0,
				.txDpProfile =
				{
					.txPreProc =
					{
						.txPreProcSymbol0 = 0,
						.txPreProcSymbol1 = 0,
						.txPreProcSymbol2 = 0,
						.txPreProcSymbol3 = 0,
						.txPreProcSymMapDivFactor = 1,
						.txPreProcMode = ADI_ADRV9001_TX_DP_PREPROC_IQ_DATA_WITH_PFIRS,
						.txPreProcWbNbPfirIBankSel = ADI_ADRV9001_PFIR_BANK_A,
						.txPreProcWbNbPfirQBankSel = ADI_ADRV9001_PFIR_BANK_B
					},
					.txWbIntTop =
					{
						.txInterpBy2Blk30En = 1,
						.txInterpBy2Blk28En = 1,
						.txInterpBy2Blk26En = 1,
						.txInterpBy2Blk24En = 1,
						.txInterpBy2Blk22En = 1,
						.txWbLpfBlk22p1En = 0
					},
					.txNbIntTop =
					{
						.txInterpBy2Blk20En = 0,
						.txInterpBy2Blk18En = 0,
						.txInterpBy2Blk16En = 0,
						.txInterpBy2Blk14En = 0,
						.txInterpBy2Blk12En = 0,
						.txInterpBy3Blk10En = 0,
						.txInterpBy2Blk8En = 0,
						.txScicBlk32En = 0,
						.txScicBlk32DivFactor = 1
					},
					.txIntTop =
					{
						.interpBy3Blk44p1En = 1,
						.sinc3Blk44En = 0,
						.sinc2Blk42En = 0,
						.interpBy3Blk40En = 1,
						.interpBy2Blk38En = 0,
						.interpBy2Blk36En = 0
					},
					.txIntTopFreqDevMap =
					{
						.rrc2Frac = 0,
						.mpll = 0,
						.nchLsw = 0,
						.nchMsb = 0,
						.freqDevMapEn = 0,
						.txRoundEn = 1
					},
					.txIqdmDuc =
					{
						.iqdmDucMode = ADI_ADRV9001_TX_DP_IQDMDUC_MODE0,
						.iqdmDev = 0,
						.iqdmDevOffset = 0,
						.iqdmScalar = 0,
						.iqdmThreshold = 0,
						.iqdmNco =
						{
							.freq = 0,
							.sampleFreq = 61440000,
							.phase = 0,
							.realOut = 0
						}
					}
				},
				.txSsiConfig =
				{
					.ssiType = ADI_ADRV9001_SSI_TYPE_CMOS,
					.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
					.numLaneSel = ADI_ADRV9001_SSI_1_LANE,
					.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
					.lsbFirst = 0,
					.qFirst = 0,
					.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DCLK_OUT_ENABLED,
					.lvdsBitInversion = 0,
					.lvdsUseLsbIn12bitMode = 0,
					.lvdsTxFullRefClkEn = false,
					.lvdsRxClkInversionEn = false,
					.cmosTxDdrNegStrobeEn = false,
					.cmosDdrPosClkEn = false,
					.cmosDdrClkInversionEn = false,
					.cmosDdrEn = false,
					.rxMaskStrobeEn = false
				}
			},
			{
				.primarySigBandwidth_Hz = 1008000,
				.txInputRate_Hz = 1920000,
				.txInterfaceSampleRate_Hz = 1920000,
				.txOffsetLo_kHz = 0,
				.validDataDelay = 0,
				.txBbf3dBCorner_kHz = 50000,
				.outputSignaling = ADI_ADRV9001_TX_IQ,
				.txPdBiasCurrent = 1,
				.txPdGainEnable = 0,
				.txPrePdRealPole_kHz = 1000000,
				.txPostPdRealPole_kHz = 530000,
				.txBbfPower = ADI_ADRV9001_COMPONENT_POWER_LEVEL_HIGH,
				.txExtLoopBackType = 0,
				.txExtLoopBackForInitCal = 0,
				.txPeakLoopBackPower = 0,
				.frequencyDeviation_Hz = 0,
				.txDpProfile =
				{
					.txPreProc =
					{
						.txPreProcSymbol0 = 0,
						.txPreProcSymbol1 = 0,
						.txPreProcSymbol2 = 0,
						.txPreProcSymbol3 = 0,
						.txPreProcSymMapDivFactor = 1,
						.txPreProcMode = ADI_ADRV9001_TX_DP_PREPROC_IQ_DATA_WITH_PFIRS,
						.txPreProcWbNbPfirIBankSel = ADI_ADRV9001_PFIR_BANK_C,
						.txPreProcWbNbPfirQBankSel = ADI_ADRV9001_PFIR_BANK_D
					},
					.txWbIntTop =
					{
						.txInterpBy2Blk30En = 1,
						.txInterpBy2Blk28En = 1,
						.txInterpBy2Blk26En = 1,
						.txInterpBy2Blk24En = 1,
						.txInterpBy2Blk22En = 1,
						.txWbLpfBlk22p1En = 0
					},
					.txNbIntTop =
					{
						.txInterpBy2Blk20En = 0,
						.txInterpBy2Blk18En = 0,
						.txInterpBy2Blk16En = 0,
						.txInterpBy2Blk14En = 0,
						.txInterpBy2Blk12En = 0,
						.txInterpBy3Blk10En = 0,
						.txInterpBy2Blk8En = 0,
						.txScicBlk32En = 0,
						.txScicBlk32DivFactor = 1
					},
					.txIntTop =
					{
						.interpBy3Blk44p1En = 1,
						.sinc3Blk44En = 0,
						.sinc2Blk42En = 0,
						.interpBy3Blk40En = 1,
						.interpBy2Blk38En = 0,
						.interpBy2Blk36En = 0
					},
					.txIntTopFreqDevMap =
					{
						.rrc2Frac = 0,
						.mpll = 0,
						.nchLsw = 0,
						.nchMsb = 0,
						.freqDevMapEn = 0,
						.txRoundEn = 1
					},
					.txIqdmDuc =
					{
						.iqdmDucMode = ADI_ADRV9001_TX_DP_IQDMDUC_MODE0,
						.iqdmDev = 0,
						.iqdmDevOffset = 0,
						.iqdmScalar = 0,
						.iqdmThreshold = 0,
						.iqdmNco =
						{
							.freq = 0,
							.sampleFreq = 61440000,
							.phase = 0,
							.realOut = 0
						}
					}
				},
				.txSsiConfig =
				{
					.ssiType = ADI_ADRV9001_SSI_TYPE_CMOS,
					.ssiDataFormatSel = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
					.numLaneSel = ADI_ADRV9001_SSI_1_LANE,
					.strobeType = ADI_ADRV9001_SSI_SHORT_STROBE,
					.lsbFirst = 0,
					.qFirst = 0,
					.txRefClockPin = ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DCLK_OUT_ENABLED,
					.lvdsBitInversion = 0,
					.lvdsUseLsbIn12bitMode = 0,
					.lvdsTxFullRefClkEn = false,
					.lvdsRxClkInversionEn = false,
					.cmosTxDdrNegStrobeEn = false,
					.cmosDdrPosClkEn = false,
					.cmosDdrClkInversionEn = false,
					.cmosDdrEn = false,
					.rxMaskStrobeEn = false
				}
			}
		}
	},
	.sysConfig = {
		.duplexMode = ADI_ADRV9001_FDD_MODE,
		.fhModeOn = 0,
		.numDynamicProfile = 1,
		.mcsMode = ADI_ADRV9001_MCSMODE_DISABLED,
		.adcTypeMonitor = ADI_ADRV9001_ADC_HP,
		.pllLockTime_us = 750,
		.pllModulus = {
			.modulus = { 8388593, 8388593, 8388593, 8388593, 8388593 },
			.dmModulus = { 8388593, 8388593 }
		}
	},
	.pfirBuffer = {
		.pfirRxWbNbChFilterCoeff_A = {
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -392, -228, 1164, -426, -2275, 3118, 1517, -7492, 4673, 8616, -16569, 2081, 24816, -27732, -11318, 53353, -35059, -44826, 94798, -27039, -110096, 145573, 15827, -224287, 197850, 132551, -432387, 241249, 465740, -1004873, 265939, 4480229, 4480229, 265939, -1004873, 465740, 241249, -432387, 132551, 197850, -224287, 15827, 145573, -110096, -27039, 94798, -44826, -35059, 53353, -11318, -27732, 24816, 2081, -16569, 8616, 4673, -7492, 1517, 3118, -2275, -426, 1164, -228, -392, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirRxWbNbChFilterCoeff_B = {
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8388608, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirRxWbNbChFilterCoeff_C = {
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -392, -228, 1164, -426, -2275, 3118, 1517, -7492, 4673, 8616, -16569, 2081, 24816, -27732, -11318, 53353, -35059, -44826, 94798, -27039, -110096, 145573, 15827, -224287, 197850, 132551, -432387, 241249, 465740, -1004873, 265939, 4480229, 4480229, 265939, -1004873, 465740, 241249, -432387, 132551, 197850, -224287, 15827, 145573, -110096, -27039, 94798, -44826, -35059, 53353, -11318, -27732, 24816, 2081, -16569, 8616, 4673, -7492, 1517, 3118, -2275, -426, 1164, -228, -392, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirRxWbNbChFilterCoeff_D = {
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8388608, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirTxWbNbPulShpCoeff_A = {
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirTxWbNbPulShpCoeff_B = {
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirTxWbNbPulShpCoeff_C = {
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirTxWbNbPulShpCoeff_D = {
			.numCoeff = 128,
			.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
			.tapsSel = ADI_ADRV9001_PFIR_128_TAPS,
			.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
			.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
		},
		.pfirRxNbPulShp = {
			{
				.numCoeff = 128,
				.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
				.taps = 128,
				.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
				.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8388608, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
			},
			{
				.numCoeff = 128,
				.symmetricSel = ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC,
				.taps = 128,
				.gainSel = ADI_ADRV9001_PFIR_GAIN_ZERO_DB,
				.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8388608, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
			}
		},
		.pfirRxMagLowTiaLowSRHp = {
			{
				.numCoeff = 21,
				.coefficients = { -21, 111, -363, 870, -1718, 2932, -4416, 5958, -7152, 6556, 27252, 6556, -7152, 5958, -4416, 2932, -1718, 870, -363, 111, -21 }
			},
			{
				.numCoeff = 21,
				.coefficients = { -21, 111, -363, 870, -1718, 2932, -4416, 5958, -7152, 6556, 27252, 6556, -7152, 5958, -4416, 2932, -1718, 870, -363, 111, -21 }
			}
		},
		.pfirRxMagLowTiaHighSRHp = {
			{
				.numCoeff = 21,
				.coefficients = { -27, 14, 422, -1035, 109, 1789, 267, -4738, -1617, 11376, 19649, 11376, -1617, -4738, 267, 1789, 109, -1035, 422, 14, -27 }
			},
			{
				.numCoeff = 21,
				.coefficients = { -27, 14, 422, -1035, 109, 1789, 267, -4738, -1617, 11376, 19649, 11376, -1617, -4738, 267, 1789, 109, -1035, 422, 14, -27 }
			}
		},
		.pfirRxMagHighTiaHighSRHp = {
			{
				.numCoeff = 21,
				.coefficients = { 28, -160, 472, -885, 984, -36, -2594, 6779, -11030, 10718, 24218, 10718, -11030, 6779, -2594, -36, 984, -885, 472, -160, 28 }
			},
			{
				.numCoeff = 21,
				.coefficients = { 28, -160, 472, -885, 984, -36, -2594, 6779, -11030, 10718, 24218, 10718, -11030, 6779, -2594, -36, 984, -885, 472, -160, 28 }
			}
		},
		.pfirRxMagLowTiaLowSRLp = {
			{
				.numCoeff = 21,
				.coefficients = { -21, 111, -363, 870, -1717, 2931, -4415, 5957, -7153, 6560, 27245, 6560, -7153, 5957, -4415, 2931, -1717, 870, -363, 111, -21 }
			},
			{
				.numCoeff = 21,
				.coefficients = { -21, 111, -363, 870, -1717, 2931, -4415, 5957, -7153, 6560, 27245, 6560, -7153, 5957, -4415, 2931, -1717, 870, -363, 111, -21 }
			}
		},
		.pfirRxMagLowTiaHighSRLp = {
			{
				.numCoeff = 21,
				.coefficients = { -28, 14, 420, -1033, 109, 1787, 264, -4732, -1611, 11374, 19637, 11374, -1611, -4732, 264, 1787, 109, -1033, 420, 14, -28 }
			},
			{
				.numCoeff = 21,
				.coefficients = { -28, 14, 420, -1033, 109, 1787, 264, -4732, -1611, 11374, 19637, 11374, -1611, -4732, 264, 1787, 109, -1033, 420, 14, -28 }
			}
		},
		.pfirRxMagHighTiaHighSRLp = {
			{
				.numCoeff = 21,
				.coefficients = { 28, -160, 472, -887, 988, -48, -2570, 6741, -10987, 10702, 24208, 10702, -10987, 6741, -2570, -48, 988, -887, 472, -160, 28 }
			},
			{
				.numCoeff = 21,
				.coefficients = { 28, -160, 472, -887, 988, -48, -2570, 6741, -10987, 10702, 24208, 10702, -10987, 6741, -2570, -48, 988, -887, 472, -160, 28 }
			}
		},
		.pfirTxMagComp1 = {
			.numCoeff = 21,
			.coefficients = { 8, -20, -56, 533, -1988, 5017, -9518, 13961, -13377, 6783, 30083, 6783, -13377, 13961, -9518, 5017, -1988, 533, -56, -20, 8 }
		},
		.pfirTxMagComp2 = {
			.numCoeff = 21,
			.coefficients = { 8, -20, -56, 533, -1988, 5017, -9518, 13961, -13377, 6783, 30083, 6783, -13377, 13961, -9518, 5017, -1988, 533, -56, -20, 8 }
		},
		.pfirTxMagCompNb = {
			{
				.numCoeff = 13,
				.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
			},
			{
				.numCoeff = 13,
				.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
			}
		},
		.pfirRxMagCompNb = {
			{
				.numCoeff = 13,
				.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
			},
			{
				.numCoeff = 13,
				.coefficients = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
			}
		}
	}
};

static struct adi_adrv9001_Init *adrv9002_init = &adrv9002_init_lvds;

struct adi_adrv9001_SpiSettings *adrv9002_spi_settings_get(void)
{
	return &spiSettings;
}

struct adi_adrv9001_Init *adrv9002_init_get(void)
{
	return adrv9002_init;
}

void adrv9002_cmos_default_set(void)
{
	/*
	 * If we are here, it means that we are on a cmos bitfile and we should
	 * use the cmos default profile
	 */
	adrv9002_init = &adrv9002_init_cmos;
}

