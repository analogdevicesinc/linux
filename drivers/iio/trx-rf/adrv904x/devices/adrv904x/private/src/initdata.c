
            /**
            * Copyright 2015 - 2025 Analog Devices Inc.
            * Released under the ADRV904X API license, for more information
            * see the "LICENSE.pdf" file in this zip file.
            */#include "initdata.h"

adi_adrv904x_Version_t initStructApiVersion = {2, 15, 0, 5};

adi_adrv904x_CpuFwVersion_t initStructArmVersion = { {2, 15, 0, 5} , ADI_ADRV904X_CPU_FW_BUILD_RELEASE};

adi_adrv904x_Version_t initStructStreamVersion = {2, 15, 0, 5};

adi_adrv904x_Init_t deviceInitStruct = 
{
    { // spiOptionsInit
        1,  // allowSpiStreaming
        1,  // allowAhbAutoIncrement
        1,  // allowAhbSpiFifoMode
    },
    { // clocks
        0,  // DevClkOnChipTermResEn
    },
    { // cpuMemDump
        { // filePath (array)
            68, 101, 118, 105, 99, 101, 67, 112, 117, 77, 101, 109, 68, 117, 109, 112,
            46, 98, 105, 110, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        }, // filePath (end of array)
    },
    { // rx
        { // rxChannelCfg (array)
            {  // rxChannelCfg[0]
                { // rxDataFormat
                    ADI_ADRV904X_GAIN_COMPENSATION_DISABLED,  // formatSelect
                    { // floatingPointConfig
                        ADI_ADRV904X_FP_FORMAT_SIGN_EXP_SIGNIFICAND,  // fpDataFormat
                        ADI_ADRV904X_ROUND_TO_EVEN,  // fpRoundMode
                        ADI_ADRV904X_2_EXPONENTBITS,  // fpNumExpBits
                        ADI_ADRV904X_FPATTEN_0DB,  // fpAttenSteps
                        ADI_ADRV904X_FP_FORMAT_HIDE_LEADING_ONE_DISABLE,  // fpHideLeadingOne
                        ADI_ADRV904X_FP_FORMAT_NAN_ENCODE_DISABLE,  // fpEncodeNan
                    },
                    { // integerConfigSettings
                        ADI_ADRV904X_NO_EMBEDDED_SLICER_BITS,  // intEmbeddedBits
                        ADI_ADRV904X_INTEGER_16BIT_2SCOMP,  // intSampleResolution
                        ADI_ADRV904X_NO_PARITY,  // intParity
                        ADI_ADRV904X_LOWER_NIBBLE_ON_I,  // intEmbeddedPos
                    },
                    { // slicerConfigSettings
                        ADI_ADRV904X_EXTSLICER_STEPSIZE_1DB,  // extSlicerStepSize
                        ADI_ADRV904X_INTSLICER_STEPSIZE_1DB,  // intSlicerStepSize
                        ADI_ADRV904X_EXTSLICER_RX_GPIO_DISABLE,  // extSlicerGpioSelect
                        { // intSlicerGpioSelect (array)
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[0]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[1]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[2]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[3]
                        }, // intSlicerGpioSelect (end of array)
                    },
                    { // embOvldMonitorSettings
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbQ
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneQ
                        ADI_ADRV904X_HB2_LOW_SRC_OVRG_LOW,  // embeddedMonitorHb2LowSrcSel
                        ADI_ADRV904X_HB2_HIGH_SRC_OVRG_HIGH,  // embeddedMonitorHb2HighSrcSel
                        ADI_ADRV904X_APD_LOW_SRC_LOWER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdLowSrcSel
                        ADI_ADRV904X_APD_HIGH_SRC_UPPER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdHighSrcSel
                        0,  // invertHb2Flag
                        0,  // invertApdFlag
                    },
                    0,  // externalLnaGain
                    0,  // tempCompensationEnable
                },
                255,  // rxGainIndexInit
            }, // rxChannelCfg[0]
            {  // rxChannelCfg[1]
                { // rxDataFormat
                    ADI_ADRV904X_GAIN_COMPENSATION_DISABLED,  // formatSelect
                    { // floatingPointConfig
                        ADI_ADRV904X_FP_FORMAT_SIGN_EXP_SIGNIFICAND,  // fpDataFormat
                        ADI_ADRV904X_ROUND_TO_EVEN,  // fpRoundMode
                        ADI_ADRV904X_2_EXPONENTBITS,  // fpNumExpBits
                        ADI_ADRV904X_FPATTEN_0DB,  // fpAttenSteps
                        ADI_ADRV904X_FP_FORMAT_HIDE_LEADING_ONE_DISABLE,  // fpHideLeadingOne
                        ADI_ADRV904X_FP_FORMAT_NAN_ENCODE_DISABLE,  // fpEncodeNan
                    },
                    { // integerConfigSettings
                        ADI_ADRV904X_NO_EMBEDDED_SLICER_BITS,  // intEmbeddedBits
                        ADI_ADRV904X_INTEGER_16BIT_2SCOMP,  // intSampleResolution
                        ADI_ADRV904X_NO_PARITY,  // intParity
                        ADI_ADRV904X_LOWER_NIBBLE_ON_I,  // intEmbeddedPos
                    },
                    { // slicerConfigSettings
                        ADI_ADRV904X_EXTSLICER_STEPSIZE_1DB,  // extSlicerStepSize
                        ADI_ADRV904X_INTSLICER_STEPSIZE_1DB,  // intSlicerStepSize
                        ADI_ADRV904X_EXTSLICER_RX_GPIO_DISABLE,  // extSlicerGpioSelect
                        { // intSlicerGpioSelect (array)
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[0]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[1]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[2]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[3]
                        }, // intSlicerGpioSelect (end of array)
                    },
                    { // embOvldMonitorSettings
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbQ
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneQ
                        ADI_ADRV904X_HB2_LOW_SRC_OVRG_LOW,  // embeddedMonitorHb2LowSrcSel
                        ADI_ADRV904X_HB2_HIGH_SRC_OVRG_HIGH,  // embeddedMonitorHb2HighSrcSel
                        ADI_ADRV904X_APD_LOW_SRC_LOWER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdLowSrcSel
                        ADI_ADRV904X_APD_HIGH_SRC_UPPER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdHighSrcSel
                        0,  // invertHb2Flag
                        0,  // invertApdFlag
                    },
                    0,  // externalLnaGain
                    0,  // tempCompensationEnable
                },
                255,  // rxGainIndexInit
            }, // rxChannelCfg[1]
            {  // rxChannelCfg[2]
                { // rxDataFormat
                    ADI_ADRV904X_GAIN_COMPENSATION_DISABLED,  // formatSelect
                    { // floatingPointConfig
                        ADI_ADRV904X_FP_FORMAT_SIGN_EXP_SIGNIFICAND,  // fpDataFormat
                        ADI_ADRV904X_ROUND_TO_EVEN,  // fpRoundMode
                        ADI_ADRV904X_2_EXPONENTBITS,  // fpNumExpBits
                        ADI_ADRV904X_FPATTEN_0DB,  // fpAttenSteps
                        ADI_ADRV904X_FP_FORMAT_HIDE_LEADING_ONE_DISABLE,  // fpHideLeadingOne
                        ADI_ADRV904X_FP_FORMAT_NAN_ENCODE_DISABLE,  // fpEncodeNan
                    },
                    { // integerConfigSettings
                        ADI_ADRV904X_NO_EMBEDDED_SLICER_BITS,  // intEmbeddedBits
                        ADI_ADRV904X_INTEGER_16BIT_2SCOMP,  // intSampleResolution
                        ADI_ADRV904X_NO_PARITY,  // intParity
                        ADI_ADRV904X_LOWER_NIBBLE_ON_I,  // intEmbeddedPos
                    },
                    { // slicerConfigSettings
                        ADI_ADRV904X_EXTSLICER_STEPSIZE_1DB,  // extSlicerStepSize
                        ADI_ADRV904X_INTSLICER_STEPSIZE_1DB,  // intSlicerStepSize
                        ADI_ADRV904X_EXTSLICER_RX_GPIO_DISABLE,  // extSlicerGpioSelect
                        { // intSlicerGpioSelect (array)
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[0]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[1]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[2]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[3]
                        }, // intSlicerGpioSelect (end of array)
                    },
                    { // embOvldMonitorSettings
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbQ
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneQ
                        ADI_ADRV904X_HB2_LOW_SRC_OVRG_LOW,  // embeddedMonitorHb2LowSrcSel
                        ADI_ADRV904X_HB2_HIGH_SRC_OVRG_HIGH,  // embeddedMonitorHb2HighSrcSel
                        ADI_ADRV904X_APD_LOW_SRC_LOWER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdLowSrcSel
                        ADI_ADRV904X_APD_HIGH_SRC_UPPER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdHighSrcSel
                        0,  // invertHb2Flag
                        0,  // invertApdFlag
                    },
                    0,  // externalLnaGain
                    0,  // tempCompensationEnable
                },
                255,  // rxGainIndexInit
            }, // rxChannelCfg[2]
            {  // rxChannelCfg[3]
                { // rxDataFormat
                    ADI_ADRV904X_GAIN_COMPENSATION_DISABLED,  // formatSelect
                    { // floatingPointConfig
                        ADI_ADRV904X_FP_FORMAT_SIGN_EXP_SIGNIFICAND,  // fpDataFormat
                        ADI_ADRV904X_ROUND_TO_EVEN,  // fpRoundMode
                        ADI_ADRV904X_2_EXPONENTBITS,  // fpNumExpBits
                        ADI_ADRV904X_FPATTEN_0DB,  // fpAttenSteps
                        ADI_ADRV904X_FP_FORMAT_HIDE_LEADING_ONE_DISABLE,  // fpHideLeadingOne
                        ADI_ADRV904X_FP_FORMAT_NAN_ENCODE_DISABLE,  // fpEncodeNan
                    },
                    { // integerConfigSettings
                        ADI_ADRV904X_NO_EMBEDDED_SLICER_BITS,  // intEmbeddedBits
                        ADI_ADRV904X_INTEGER_16BIT_2SCOMP,  // intSampleResolution
                        ADI_ADRV904X_NO_PARITY,  // intParity
                        ADI_ADRV904X_LOWER_NIBBLE_ON_I,  // intEmbeddedPos
                    },
                    { // slicerConfigSettings
                        ADI_ADRV904X_EXTSLICER_STEPSIZE_1DB,  // extSlicerStepSize
                        ADI_ADRV904X_INTSLICER_STEPSIZE_1DB,  // intSlicerStepSize
                        ADI_ADRV904X_EXTSLICER_RX_GPIO_DISABLE,  // extSlicerGpioSelect
                        { // intSlicerGpioSelect (array)
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[0]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[1]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[2]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[3]
                        }, // intSlicerGpioSelect (end of array)
                    },
                    { // embOvldMonitorSettings
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbQ
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneQ
                        ADI_ADRV904X_HB2_LOW_SRC_OVRG_LOW,  // embeddedMonitorHb2LowSrcSel
                        ADI_ADRV904X_HB2_HIGH_SRC_OVRG_HIGH,  // embeddedMonitorHb2HighSrcSel
                        ADI_ADRV904X_APD_LOW_SRC_LOWER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdLowSrcSel
                        ADI_ADRV904X_APD_HIGH_SRC_UPPER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdHighSrcSel
                        0,  // invertHb2Flag
                        0,  // invertApdFlag
                    },
                    0,  // externalLnaGain
                    0,  // tempCompensationEnable
                },
                255,  // rxGainIndexInit
            }, // rxChannelCfg[3]
            {  // rxChannelCfg[4]
                { // rxDataFormat
                    ADI_ADRV904X_GAIN_COMPENSATION_DISABLED,  // formatSelect
                    { // floatingPointConfig
                        ADI_ADRV904X_FP_FORMAT_SIGN_EXP_SIGNIFICAND,  // fpDataFormat
                        ADI_ADRV904X_ROUND_TO_EVEN,  // fpRoundMode
                        ADI_ADRV904X_2_EXPONENTBITS,  // fpNumExpBits
                        ADI_ADRV904X_FPATTEN_0DB,  // fpAttenSteps
                        ADI_ADRV904X_FP_FORMAT_HIDE_LEADING_ONE_DISABLE,  // fpHideLeadingOne
                        ADI_ADRV904X_FP_FORMAT_NAN_ENCODE_DISABLE,  // fpEncodeNan
                    },
                    { // integerConfigSettings
                        ADI_ADRV904X_NO_EMBEDDED_SLICER_BITS,  // intEmbeddedBits
                        ADI_ADRV904X_INTEGER_16BIT_2SCOMP,  // intSampleResolution
                        ADI_ADRV904X_NO_PARITY,  // intParity
                        ADI_ADRV904X_LOWER_NIBBLE_ON_I,  // intEmbeddedPos
                    },
                    { // slicerConfigSettings
                        ADI_ADRV904X_EXTSLICER_STEPSIZE_1DB,  // extSlicerStepSize
                        ADI_ADRV904X_INTSLICER_STEPSIZE_1DB,  // intSlicerStepSize
                        ADI_ADRV904X_EXTSLICER_RX_GPIO_DISABLE,  // extSlicerGpioSelect
                        { // intSlicerGpioSelect (array)
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[0]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[1]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[2]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[3]
                        }, // intSlicerGpioSelect (end of array)
                    },
                    { // embOvldMonitorSettings
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbQ
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneQ
                        ADI_ADRV904X_HB2_LOW_SRC_OVRG_LOW,  // embeddedMonitorHb2LowSrcSel
                        ADI_ADRV904X_HB2_HIGH_SRC_OVRG_HIGH,  // embeddedMonitorHb2HighSrcSel
                        ADI_ADRV904X_APD_LOW_SRC_LOWER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdLowSrcSel
                        ADI_ADRV904X_APD_HIGH_SRC_UPPER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdHighSrcSel
                        0,  // invertHb2Flag
                        0,  // invertApdFlag
                    },
                    0,  // externalLnaGain
                    0,  // tempCompensationEnable
                },
                255,  // rxGainIndexInit
            }, // rxChannelCfg[4]
            {  // rxChannelCfg[5]
                { // rxDataFormat
                    ADI_ADRV904X_GAIN_COMPENSATION_DISABLED,  // formatSelect
                    { // floatingPointConfig
                        ADI_ADRV904X_FP_FORMAT_SIGN_EXP_SIGNIFICAND,  // fpDataFormat
                        ADI_ADRV904X_ROUND_TO_EVEN,  // fpRoundMode
                        ADI_ADRV904X_2_EXPONENTBITS,  // fpNumExpBits
                        ADI_ADRV904X_FPATTEN_0DB,  // fpAttenSteps
                        ADI_ADRV904X_FP_FORMAT_HIDE_LEADING_ONE_DISABLE,  // fpHideLeadingOne
                        ADI_ADRV904X_FP_FORMAT_NAN_ENCODE_DISABLE,  // fpEncodeNan
                    },
                    { // integerConfigSettings
                        ADI_ADRV904X_NO_EMBEDDED_SLICER_BITS,  // intEmbeddedBits
                        ADI_ADRV904X_INTEGER_16BIT_2SCOMP,  // intSampleResolution
                        ADI_ADRV904X_NO_PARITY,  // intParity
                        ADI_ADRV904X_LOWER_NIBBLE_ON_I,  // intEmbeddedPos
                    },
                    { // slicerConfigSettings
                        ADI_ADRV904X_EXTSLICER_STEPSIZE_1DB,  // extSlicerStepSize
                        ADI_ADRV904X_INTSLICER_STEPSIZE_1DB,  // intSlicerStepSize
                        ADI_ADRV904X_EXTSLICER_RX_GPIO_DISABLE,  // extSlicerGpioSelect
                        { // intSlicerGpioSelect (array)
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[0]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[1]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[2]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[3]
                        }, // intSlicerGpioSelect (end of array)
                    },
                    { // embOvldMonitorSettings
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbQ
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneQ
                        ADI_ADRV904X_HB2_LOW_SRC_OVRG_LOW,  // embeddedMonitorHb2LowSrcSel
                        ADI_ADRV904X_HB2_HIGH_SRC_OVRG_HIGH,  // embeddedMonitorHb2HighSrcSel
                        ADI_ADRV904X_APD_LOW_SRC_LOWER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdLowSrcSel
                        ADI_ADRV904X_APD_HIGH_SRC_UPPER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdHighSrcSel
                        0,  // invertHb2Flag
                        0,  // invertApdFlag
                    },
                    0,  // externalLnaGain
                    0,  // tempCompensationEnable
                },
                255,  // rxGainIndexInit
            }, // rxChannelCfg[5]
            {  // rxChannelCfg[6]
                { // rxDataFormat
                    ADI_ADRV904X_GAIN_COMPENSATION_DISABLED,  // formatSelect
                    { // floatingPointConfig
                        ADI_ADRV904X_FP_FORMAT_SIGN_EXP_SIGNIFICAND,  // fpDataFormat
                        ADI_ADRV904X_ROUND_TO_EVEN,  // fpRoundMode
                        ADI_ADRV904X_2_EXPONENTBITS,  // fpNumExpBits
                        ADI_ADRV904X_FPATTEN_0DB,  // fpAttenSteps
                        ADI_ADRV904X_FP_FORMAT_HIDE_LEADING_ONE_DISABLE,  // fpHideLeadingOne
                        ADI_ADRV904X_FP_FORMAT_NAN_ENCODE_DISABLE,  // fpEncodeNan
                    },
                    { // integerConfigSettings
                        ADI_ADRV904X_NO_EMBEDDED_SLICER_BITS,  // intEmbeddedBits
                        ADI_ADRV904X_INTEGER_16BIT_2SCOMP,  // intSampleResolution
                        ADI_ADRV904X_NO_PARITY,  // intParity
                        ADI_ADRV904X_LOWER_NIBBLE_ON_I,  // intEmbeddedPos
                    },
                    { // slicerConfigSettings
                        ADI_ADRV904X_EXTSLICER_STEPSIZE_1DB,  // extSlicerStepSize
                        ADI_ADRV904X_INTSLICER_STEPSIZE_1DB,  // intSlicerStepSize
                        ADI_ADRV904X_EXTSLICER_RX_GPIO_DISABLE,  // extSlicerGpioSelect
                        { // intSlicerGpioSelect (array)
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[0]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[1]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[2]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[3]
                        }, // intSlicerGpioSelect (end of array)
                    },
                    { // embOvldMonitorSettings
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbQ
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneQ
                        ADI_ADRV904X_HB2_LOW_SRC_OVRG_LOW,  // embeddedMonitorHb2LowSrcSel
                        ADI_ADRV904X_HB2_HIGH_SRC_OVRG_HIGH,  // embeddedMonitorHb2HighSrcSel
                        ADI_ADRV904X_APD_LOW_SRC_LOWER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdLowSrcSel
                        ADI_ADRV904X_APD_HIGH_SRC_UPPER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdHighSrcSel
                        0,  // invertHb2Flag
                        0,  // invertApdFlag
                    },
                    0,  // externalLnaGain
                    0,  // tempCompensationEnable
                },
                255,  // rxGainIndexInit
            }, // rxChannelCfg[6]
            {  // rxChannelCfg[7]
                { // rxDataFormat
                    ADI_ADRV904X_GAIN_COMPENSATION_DISABLED,  // formatSelect
                    { // floatingPointConfig
                        ADI_ADRV904X_FP_FORMAT_SIGN_EXP_SIGNIFICAND,  // fpDataFormat
                        ADI_ADRV904X_ROUND_TO_EVEN,  // fpRoundMode
                        ADI_ADRV904X_2_EXPONENTBITS,  // fpNumExpBits
                        ADI_ADRV904X_FPATTEN_0DB,  // fpAttenSteps
                        ADI_ADRV904X_FP_FORMAT_HIDE_LEADING_ONE_DISABLE,  // fpHideLeadingOne
                        ADI_ADRV904X_FP_FORMAT_NAN_ENCODE_DISABLE,  // fpEncodeNan
                    },
                    { // integerConfigSettings
                        ADI_ADRV904X_NO_EMBEDDED_SLICER_BITS,  // intEmbeddedBits
                        ADI_ADRV904X_INTEGER_16BIT_2SCOMP,  // intSampleResolution
                        ADI_ADRV904X_NO_PARITY,  // intParity
                        ADI_ADRV904X_LOWER_NIBBLE_ON_I,  // intEmbeddedPos
                    },
                    { // slicerConfigSettings
                        ADI_ADRV904X_EXTSLICER_STEPSIZE_1DB,  // extSlicerStepSize
                        ADI_ADRV904X_INTSLICER_STEPSIZE_1DB,  // intSlicerStepSize
                        ADI_ADRV904X_EXTSLICER_RX_GPIO_DISABLE,  // extSlicerGpioSelect
                        { // intSlicerGpioSelect (array)
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[0]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[1]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[2]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[3]
                        }, // intSlicerGpioSelect (end of array)
                    },
                    { // embOvldMonitorSettings
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbQ
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneQ
                        ADI_ADRV904X_HB2_LOW_SRC_OVRG_LOW,  // embeddedMonitorHb2LowSrcSel
                        ADI_ADRV904X_HB2_HIGH_SRC_OVRG_HIGH,  // embeddedMonitorHb2HighSrcSel
                        ADI_ADRV904X_APD_LOW_SRC_LOWER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdLowSrcSel
                        ADI_ADRV904X_APD_HIGH_SRC_UPPER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdHighSrcSel
                        0,  // invertHb2Flag
                        0,  // invertApdFlag
                    },
                    0,  // externalLnaGain
                    0,  // tempCompensationEnable
                },
                255,  // rxGainIndexInit
            }, // rxChannelCfg[7]
            {  // rxChannelCfg[8]
                { // rxDataFormat
                    ADI_ADRV904X_GAIN_COMPENSATION_DISABLED,  // formatSelect
                    { // floatingPointConfig
                        ADI_ADRV904X_FP_FORMAT_SIGN_EXP_SIGNIFICAND,  // fpDataFormat
                        ADI_ADRV904X_ROUND_TO_EVEN,  // fpRoundMode
                        ADI_ADRV904X_2_EXPONENTBITS,  // fpNumExpBits
                        ADI_ADRV904X_FPATTEN_0DB,  // fpAttenSteps
                        ADI_ADRV904X_FP_FORMAT_HIDE_LEADING_ONE_DISABLE,  // fpHideLeadingOne
                        ADI_ADRV904X_FP_FORMAT_NAN_ENCODE_DISABLE,  // fpEncodeNan
                    },
                    { // integerConfigSettings
                        ADI_ADRV904X_NO_EMBEDDED_SLICER_BITS,  // intEmbeddedBits
                        ADI_ADRV904X_INTEGER_16BIT_2SCOMP,  // intSampleResolution
                        ADI_ADRV904X_NO_PARITY,  // intParity
                        ADI_ADRV904X_LOWER_NIBBLE_ON_I,  // intEmbeddedPos
                    },
                    { // slicerConfigSettings
                        ADI_ADRV904X_EXTSLICER_STEPSIZE_1DB,  // extSlicerStepSize
                        ADI_ADRV904X_INTSLICER_STEPSIZE_1DB,  // intSlicerStepSize
                        ADI_ADRV904X_EXTSLICER_RX_GPIO_DISABLE,  // extSlicerGpioSelect
                        { // intSlicerGpioSelect (array)
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[0]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[1]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[2]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[3]
                        }, // intSlicerGpioSelect (end of array)
                    },
                    { // embOvldMonitorSettings
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbQ
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneQ
                        ADI_ADRV904X_HB2_LOW_SRC_OVRG_LOW,  // embeddedMonitorHb2LowSrcSel
                        ADI_ADRV904X_HB2_HIGH_SRC_OVRG_HIGH,  // embeddedMonitorHb2HighSrcSel
                        ADI_ADRV904X_APD_LOW_SRC_LOWER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdLowSrcSel
                        ADI_ADRV904X_APD_HIGH_SRC_UPPER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdHighSrcSel
                        0,  // invertHb2Flag
                        0,  // invertApdFlag
                    },
                    0,  // externalLnaGain
                    0,  // tempCompensationEnable
                },
                255,  // rxGainIndexInit
            }, // rxChannelCfg[8]
            {  // rxChannelCfg[9]
                { // rxDataFormat
                    ADI_ADRV904X_GAIN_COMPENSATION_DISABLED,  // formatSelect
                    { // floatingPointConfig
                        ADI_ADRV904X_FP_FORMAT_SIGN_EXP_SIGNIFICAND,  // fpDataFormat
                        ADI_ADRV904X_ROUND_TO_EVEN,  // fpRoundMode
                        ADI_ADRV904X_2_EXPONENTBITS,  // fpNumExpBits
                        ADI_ADRV904X_FPATTEN_0DB,  // fpAttenSteps
                        ADI_ADRV904X_FP_FORMAT_HIDE_LEADING_ONE_DISABLE,  // fpHideLeadingOne
                        ADI_ADRV904X_FP_FORMAT_NAN_ENCODE_DISABLE,  // fpEncodeNan
                    },
                    { // integerConfigSettings
                        ADI_ADRV904X_NO_EMBEDDED_SLICER_BITS,  // intEmbeddedBits
                        ADI_ADRV904X_INTEGER_16BIT_2SCOMP,  // intSampleResolution
                        ADI_ADRV904X_NO_PARITY,  // intParity
                        ADI_ADRV904X_LOWER_NIBBLE_ON_I,  // intEmbeddedPos
                    },
                    { // slicerConfigSettings
                        ADI_ADRV904X_EXTSLICER_STEPSIZE_1DB,  // extSlicerStepSize
                        ADI_ADRV904X_INTSLICER_STEPSIZE_1DB,  // intSlicerStepSize
                        ADI_ADRV904X_EXTSLICER_RX_GPIO_DISABLE,  // extSlicerGpioSelect
                        { // intSlicerGpioSelect (array)
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[0]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[1]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[2]
                            ADI_ADRV904X_GPIO_INVALID,  // intSlicerGpioSelect[3]
                        }, // intSlicerGpioSelect (end of array)
                    },
                    { // embOvldMonitorSettings
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbQ
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneI
                        ADI_ADRV904X_RX_EMB_MON_SRC_NO_MON_DATA_EMBEDDED,  // embeddedMonitorSrcLsbPlusOneQ
                        ADI_ADRV904X_HB2_LOW_SRC_OVRG_LOW,  // embeddedMonitorHb2LowSrcSel
                        ADI_ADRV904X_HB2_HIGH_SRC_OVRG_HIGH,  // embeddedMonitorHb2HighSrcSel
                        ADI_ADRV904X_APD_LOW_SRC_LOWER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdLowSrcSel
                        ADI_ADRV904X_APD_HIGH_SRC_UPPER_LEVEL_BLOCKER_EXCEEDED,  // embeddedMonitorApdHighSrcSel
                        0,  // invertHb2Flag
                        0,  // invertApdFlag
                    },
                    0,  // externalLnaGain
                    0,  // tempCompensationEnable
                },
                255,  // rxGainIndexInit
            }, // rxChannelCfg[9]
        }, // rxChannelCfg (end of array)
    },
    { // tx
        { // txChannelCfg (array)
            {  // txChannelCfg[0]
                { // txAttenCfg
                    { // updateCfg
                        { // srcCfg
                            ADI_ADRV904X_TXATTEN_UPD_SRC_S0,  // updateSrc
                            ADI_ADRV904X_GPIO_INVALID,  // s0OrS1Gpio
                        },
                        { // trgCfg
                            ADI_ADRV904X_TXATTEN_UPD_TRG_NONE,  // updateTrg
                            ADI_ADRV904X_GPIO_INVALID,  // triggerGpio
                        },
                    },
                    ADI_ADRV904X_TXATTEN_0P05_DB,  // txAttenStepSize
                },
                0,  // txAttenInit_mdB
                { // txpowerMonitorCfg
                    ADI_ADRV904X_POST_DPD_HC_OUTPUT,  // inputSel
                    0,  // peakThreshold
                    0,  // measDuration
                    0,  // peakCount
                    0,  // peakErrorClearRequired
                    0,  // peakPowerEnable
                    0,  // peakPowerIrqEnable
                    0,  // avgThreshold
                    0,  // avgErrorClearRequired
                    0,  // avgPowerEnable
                    0,  // avgPowerIrqEnable
                    0,  // avgPeakRatioEnable
                },
                { // srlCfg
                    ADI_ADRV904X_POST_DPD_HC_OUTPUT,  // inputSel
                    0,  // srdOffset
                    0,  // srdEnable
                    0,  // srdIrqEnable
                    0,  // autoRecoveryWaitTime
                    0,  // autoRecoveryEnable
                    0,  // autoRecoveryDisableTimerWhenTxOff
                    0,  // srdStatisticsEnable
                    0,  // srdStatisticsMode
                },
                { // protectionRampCfg
                    0x00,  // rampDownMask
                    0,  // altEventClearReqd
                },
            }, // txChannelCfg[0]
            {  // txChannelCfg[1]
                { // txAttenCfg
                    { // updateCfg
                        { // srcCfg
                            ADI_ADRV904X_TXATTEN_UPD_SRC_S0,  // updateSrc
                            ADI_ADRV904X_GPIO_INVALID,  // s0OrS1Gpio
                        },
                        { // trgCfg
                            ADI_ADRV904X_TXATTEN_UPD_TRG_NONE,  // updateTrg
                            ADI_ADRV904X_GPIO_INVALID,  // triggerGpio
                        },
                    },
                    ADI_ADRV904X_TXATTEN_0P05_DB,  // txAttenStepSize
                },
                0,  // txAttenInit_mdB
                { // txpowerMonitorCfg
                    ADI_ADRV904X_POST_DPD_HC_OUTPUT,  // inputSel
                    0,  // peakThreshold
                    0,  // measDuration
                    0,  // peakCount
                    0,  // peakErrorClearRequired
                    0,  // peakPowerEnable
                    0,  // peakPowerIrqEnable
                    0,  // avgThreshold
                    0,  // avgErrorClearRequired
                    0,  // avgPowerEnable
                    0,  // avgPowerIrqEnable
                    0,  // avgPeakRatioEnable
                },
                { // srlCfg
                    ADI_ADRV904X_POST_DPD_HC_OUTPUT,  // inputSel
                    0,  // srdOffset
                    0,  // srdEnable
                    0,  // srdIrqEnable
                    0,  // autoRecoveryWaitTime
                    0,  // autoRecoveryEnable
                    0,  // autoRecoveryDisableTimerWhenTxOff
                    0,  // srdStatisticsEnable
                    0,  // srdStatisticsMode
                },
                { // protectionRampCfg
                    0x00,  // rampDownMask
                    0,  // altEventClearReqd
                },
            }, // txChannelCfg[1]
            {  // txChannelCfg[2]
                { // txAttenCfg
                    { // updateCfg
                        { // srcCfg
                            ADI_ADRV904X_TXATTEN_UPD_SRC_S0,  // updateSrc
                            ADI_ADRV904X_GPIO_INVALID,  // s0OrS1Gpio
                        },
                        { // trgCfg
                            ADI_ADRV904X_TXATTEN_UPD_TRG_NONE,  // updateTrg
                            ADI_ADRV904X_GPIO_INVALID,  // triggerGpio
                        },
                    },
                    ADI_ADRV904X_TXATTEN_0P05_DB,  // txAttenStepSize
                },
                0,  // txAttenInit_mdB
                { // txpowerMonitorCfg
                    ADI_ADRV904X_POST_DPD_HC_OUTPUT,  // inputSel
                    0,  // peakThreshold
                    0,  // measDuration
                    0,  // peakCount
                    0,  // peakErrorClearRequired
                    0,  // peakPowerEnable
                    0,  // peakPowerIrqEnable
                    0,  // avgThreshold
                    0,  // avgErrorClearRequired
                    0,  // avgPowerEnable
                    0,  // avgPowerIrqEnable
                    0,  // avgPeakRatioEnable
                },
                { // srlCfg
                    ADI_ADRV904X_POST_DPD_HC_OUTPUT,  // inputSel
                    0,  // srdOffset
                    0,  // srdEnable
                    0,  // srdIrqEnable
                    0,  // autoRecoveryWaitTime
                    0,  // autoRecoveryEnable
                    0,  // autoRecoveryDisableTimerWhenTxOff
                    0,  // srdStatisticsEnable
                    0,  // srdStatisticsMode
                },
                { // protectionRampCfg
                    0x00,  // rampDownMask
                    0,  // altEventClearReqd
                },
            }, // txChannelCfg[2]
            {  // txChannelCfg[3]
                { // txAttenCfg
                    { // updateCfg
                        { // srcCfg
                            ADI_ADRV904X_TXATTEN_UPD_SRC_S0,  // updateSrc
                            ADI_ADRV904X_GPIO_INVALID,  // s0OrS1Gpio
                        },
                        { // trgCfg
                            ADI_ADRV904X_TXATTEN_UPD_TRG_NONE,  // updateTrg
                            ADI_ADRV904X_GPIO_INVALID,  // triggerGpio
                        },
                    },
                    ADI_ADRV904X_TXATTEN_0P05_DB,  // txAttenStepSize
                },
                0,  // txAttenInit_mdB
                { // txpowerMonitorCfg
                    ADI_ADRV904X_POST_DPD_HC_OUTPUT,  // inputSel
                    0,  // peakThreshold
                    0,  // measDuration
                    0,  // peakCount
                    0,  // peakErrorClearRequired
                    0,  // peakPowerEnable
                    0,  // peakPowerIrqEnable
                    0,  // avgThreshold
                    0,  // avgErrorClearRequired
                    0,  // avgPowerEnable
                    0,  // avgPowerIrqEnable
                    0,  // avgPeakRatioEnable
                },
                { // srlCfg
                    ADI_ADRV904X_POST_DPD_HC_OUTPUT,  // inputSel
                    0,  // srdOffset
                    0,  // srdEnable
                    0,  // srdIrqEnable
                    0,  // autoRecoveryWaitTime
                    0,  // autoRecoveryEnable
                    0,  // autoRecoveryDisableTimerWhenTxOff
                    0,  // srdStatisticsEnable
                    0,  // srdStatisticsMode
                },
                { // protectionRampCfg
                    0x00,  // rampDownMask
                    0,  // altEventClearReqd
                },
            }, // txChannelCfg[3]
            {  // txChannelCfg[4]
                { // txAttenCfg
                    { // updateCfg
                        { // srcCfg
                            ADI_ADRV904X_TXATTEN_UPD_SRC_S0,  // updateSrc
                            ADI_ADRV904X_GPIO_INVALID,  // s0OrS1Gpio
                        },
                        { // trgCfg
                            ADI_ADRV904X_TXATTEN_UPD_TRG_NONE,  // updateTrg
                            ADI_ADRV904X_GPIO_INVALID,  // triggerGpio
                        },
                    },
                    ADI_ADRV904X_TXATTEN_0P05_DB,  // txAttenStepSize
                },
                0,  // txAttenInit_mdB
                { // txpowerMonitorCfg
                    ADI_ADRV904X_POST_DPD_HC_OUTPUT,  // inputSel
                    0,  // peakThreshold
                    0,  // measDuration
                    0,  // peakCount
                    0,  // peakErrorClearRequired
                    0,  // peakPowerEnable
                    0,  // peakPowerIrqEnable
                    0,  // avgThreshold
                    0,  // avgErrorClearRequired
                    0,  // avgPowerEnable
                    0,  // avgPowerIrqEnable
                    0,  // avgPeakRatioEnable
                },
                { // srlCfg
                    ADI_ADRV904X_POST_DPD_HC_OUTPUT,  // inputSel
                    0,  // srdOffset
                    0,  // srdEnable
                    0,  // srdIrqEnable
                    0,  // autoRecoveryWaitTime
                    0,  // autoRecoveryEnable
                    0,  // autoRecoveryDisableTimerWhenTxOff
                    0,  // srdStatisticsEnable
                    0,  // srdStatisticsMode
                },
                { // protectionRampCfg
                    0x00,  // rampDownMask
                    0,  // altEventClearReqd
                },
            }, // txChannelCfg[4]
            {  // txChannelCfg[5]
                { // txAttenCfg
                    { // updateCfg
                        { // srcCfg
                            ADI_ADRV904X_TXATTEN_UPD_SRC_S0,  // updateSrc
                            ADI_ADRV904X_GPIO_INVALID,  // s0OrS1Gpio
                        },
                        { // trgCfg
                            ADI_ADRV904X_TXATTEN_UPD_TRG_NONE,  // updateTrg
                            ADI_ADRV904X_GPIO_INVALID,  // triggerGpio
                        },
                    },
                    ADI_ADRV904X_TXATTEN_0P05_DB,  // txAttenStepSize
                },
                0,  // txAttenInit_mdB
                { // txpowerMonitorCfg
                    ADI_ADRV904X_POST_DPD_HC_OUTPUT,  // inputSel
                    0,  // peakThreshold
                    0,  // measDuration
                    0,  // peakCount
                    0,  // peakErrorClearRequired
                    0,  // peakPowerEnable
                    0,  // peakPowerIrqEnable
                    0,  // avgThreshold
                    0,  // avgErrorClearRequired
                    0,  // avgPowerEnable
                    0,  // avgPowerIrqEnable
                    0,  // avgPeakRatioEnable
                },
                { // srlCfg
                    ADI_ADRV904X_POST_DPD_HC_OUTPUT,  // inputSel
                    0,  // srdOffset
                    0,  // srdEnable
                    0,  // srdIrqEnable
                    0,  // autoRecoveryWaitTime
                    0,  // autoRecoveryEnable
                    0,  // autoRecoveryDisableTimerWhenTxOff
                    0,  // srdStatisticsEnable
                    0,  // srdStatisticsMode
                },
                { // protectionRampCfg
                    0x00,  // rampDownMask
                    0,  // altEventClearReqd
                },
            }, // txChannelCfg[5]
            {  // txChannelCfg[6]
                { // txAttenCfg
                    { // updateCfg
                        { // srcCfg
                            ADI_ADRV904X_TXATTEN_UPD_SRC_S0,  // updateSrc
                            ADI_ADRV904X_GPIO_INVALID,  // s0OrS1Gpio
                        },
                        { // trgCfg
                            ADI_ADRV904X_TXATTEN_UPD_TRG_NONE,  // updateTrg
                            ADI_ADRV904X_GPIO_INVALID,  // triggerGpio
                        },
                    },
                    ADI_ADRV904X_TXATTEN_0P05_DB,  // txAttenStepSize
                },
                0,  // txAttenInit_mdB
                { // txpowerMonitorCfg
                    ADI_ADRV904X_POST_DPD_HC_OUTPUT,  // inputSel
                    0,  // peakThreshold
                    0,  // measDuration
                    0,  // peakCount
                    0,  // peakErrorClearRequired
                    0,  // peakPowerEnable
                    0,  // peakPowerIrqEnable
                    0,  // avgThreshold
                    0,  // avgErrorClearRequired
                    0,  // avgPowerEnable
                    0,  // avgPowerIrqEnable
                    0,  // avgPeakRatioEnable
                },
                { // srlCfg
                    ADI_ADRV904X_POST_DPD_HC_OUTPUT,  // inputSel
                    0,  // srdOffset
                    0,  // srdEnable
                    0,  // srdIrqEnable
                    0,  // autoRecoveryWaitTime
                    0,  // autoRecoveryEnable
                    0,  // autoRecoveryDisableTimerWhenTxOff
                    0,  // srdStatisticsEnable
                    0,  // srdStatisticsMode
                },
                { // protectionRampCfg
                    0x00,  // rampDownMask
                    0,  // altEventClearReqd
                },
            }, // txChannelCfg[6]
            {  // txChannelCfg[7]
                { // txAttenCfg
                    { // updateCfg
                        { // srcCfg
                            ADI_ADRV904X_TXATTEN_UPD_SRC_S0,  // updateSrc
                            ADI_ADRV904X_GPIO_INVALID,  // s0OrS1Gpio
                        },
                        { // trgCfg
                            ADI_ADRV904X_TXATTEN_UPD_TRG_NONE,  // updateTrg
                            ADI_ADRV904X_GPIO_INVALID,  // triggerGpio
                        },
                    },
                    ADI_ADRV904X_TXATTEN_0P05_DB,  // txAttenStepSize
                },
                0,  // txAttenInit_mdB
                { // txpowerMonitorCfg
                    ADI_ADRV904X_POST_DPD_HC_OUTPUT,  // inputSel
                    0,  // peakThreshold
                    0,  // measDuration
                    0,  // peakCount
                    0,  // peakErrorClearRequired
                    0,  // peakPowerEnable
                    0,  // peakPowerIrqEnable
                    0,  // avgThreshold
                    0,  // avgErrorClearRequired
                    0,  // avgPowerEnable
                    0,  // avgPowerIrqEnable
                    0,  // avgPeakRatioEnable
                },
                { // srlCfg
                    ADI_ADRV904X_POST_DPD_HC_OUTPUT,  // inputSel
                    0,  // srdOffset
                    0,  // srdEnable
                    0,  // srdIrqEnable
                    0,  // autoRecoveryWaitTime
                    0,  // autoRecoveryEnable
                    0,  // autoRecoveryDisableTimerWhenTxOff
                    0,  // srdStatisticsEnable
                    0,  // srdStatisticsMode
                },
                { // protectionRampCfg
                    0x00,  // rampDownMask
                    0,  // altEventClearReqd
                },
            }, // txChannelCfg[7]
        }, // txChannelCfg (end of array)
    },
    { // uart (array)
        {  // uart[0]
            0,  // enable
            ADI_ADRV904X_GPIO_09,  // pinSelect
        }, // uart[0]
        {  // uart[1]
            0,  // enable
            ADI_ADRV904X_GPIO_10,  // pinSelect
        }, // uart[1]
        {  // uart[2]
            0,  // enable
            ADI_ADRV904X_GPIO_11,  // pinSelect
        }, // uart[2]
        {  // uart[3]
            0,  // enable
            ADI_ADRV904X_GPIO_12,  // pinSelect
        }, // uart[3]
    }, // uart (end of array)
    { // gpIntPreInit
        { // gpInt0Mask
            0xFFFFFFFFFFFF,  // lowerMask
            0xFFFFFFFFFFFF,  // upperMask
        },
        { // gpInt1Mask
            0xFFFFFFFFFFFF,  // lowerMask
            0xFFFFFFFFFFFF,  // upperMask
        },
    },
    { // radioCtrlPreInit
        ADI_ADRV904X_GPIO_INVALID,  // radioSequencerSsbSyncGpioCtrl
        { // radioSequencerGpioDigOut (array)
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[0]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[1]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[2]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[3]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[4]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[5]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[6]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[7]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[8]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[9]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[10]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[11]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[12]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[13]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[14]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[15]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[16]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[17]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[18]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[19]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[20]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[21]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[22]
            ADI_ADRV904X_GPIO_INVALID,  // radioSequencerGpioDigOut[23]
        }, // radioSequencerGpioDigOut (end of array)
        { // radioSequencerGpioAnaOut (array)
            ADI_ADRV904X_GPIO_ANA_INVALID,  // radioSequencerGpioAnaOut[0]
            ADI_ADRV904X_GPIO_ANA_INVALID,  // radioSequencerGpioAnaOut[1]
            ADI_ADRV904X_GPIO_ANA_INVALID,  // radioSequencerGpioAnaOut[2]
            ADI_ADRV904X_GPIO_ANA_INVALID,  // radioSequencerGpioAnaOut[3]
            ADI_ADRV904X_GPIO_ANA_INVALID,  // radioSequencerGpioAnaOut[4]
            ADI_ADRV904X_GPIO_ANA_INVALID,  // radioSequencerGpioAnaOut[5]
            ADI_ADRV904X_GPIO_ANA_INVALID,  // radioSequencerGpioAnaOut[6]
            ADI_ADRV904X_GPIO_ANA_INVALID,  // radioSequencerGpioAnaOut[7]
            ADI_ADRV904X_GPIO_ANA_INVALID,  // radioSequencerGpioAnaOut[8]
            ADI_ADRV904X_GPIO_ANA_INVALID,  // radioSequencerGpioAnaOut[9]
            ADI_ADRV904X_GPIO_ANA_INVALID,  // radioSequencerGpioAnaOut[10]
            ADI_ADRV904X_GPIO_ANA_INVALID,  // radioSequencerGpioAnaOut[11]
            ADI_ADRV904X_GPIO_ANA_INVALID,  // radioSequencerGpioAnaOut[12]
            ADI_ADRV904X_GPIO_ANA_INVALID,  // radioSequencerGpioAnaOut[13]
            ADI_ADRV904X_GPIO_ANA_INVALID,  // radioSequencerGpioAnaOut[14]
            ADI_ADRV904X_GPIO_ANA_INVALID,  // radioSequencerGpioAnaOut[15]
        }, // radioSequencerGpioAnaOut (end of array)
    },
    { // dfeUart0
        0,  // enableGpioTx
        0,  // enableGpioRx
        0,  // enableGpioCts
        0,  // enableGpioRts
    },
};
adi_adrv904x_PostMcsInit_t utilityInit = 
{
    { // radioCtrlCfg
        { // txRadioCtrlModeCfg
            ADI_ADRV904X_TX_EN_SPI_MODE,  // txEnableMode
            0xFF,  // txChannelMask
        },
        { // rxRadioCtrlModeCfg
            ADI_ADRV904X_RX_EN_SPI_MODE,  // rxEnableMode
            0xFF,  // rxChannelMask
        },
        { // orxRadioCtrlModeCfg
            ADI_ADRV904X_ORX_EN_SPI_MODE,  // orxEnableMode
            0x300,  // orxChannelMask
        },
    },
    { // radioCtrlGpioCfg
        { // txEnMapping (array)
            0, 0, 0, 0, 0, 0, 0, 0,
        }, // txEnMapping (end of array)
        { // txAltMapping (array)
            0, 0, 0, 0, 0, 0, 0, 0,
        }, // txAltMapping (end of array)
        { // rxEnMapping (array)
            0, 0, 0, 0, 0, 0, 0, 0,
        }, // rxEnMapping (end of array)
        { // rxAltMapping (array)
            0, 0, 0, 0, 0, 0, 0, 0,
        }, // rxAltMapping (end of array)
    },
    0,  // radioCtrlTxRxEnPinSel
    0,  // radioCtrlTxRxEnCfgSel
    { // gpIntPostInit
        { // gpInt0Mask
            0xFFFFFFFFFFFF,  // lowerMask
            0xFFFFFFFFFFFF,  // upperMask
        },
        { // gpInt1Mask
            0xFFFFFFFFFFFF,  // lowerMask
            0xFFFFFFFFFFFF,  // upperMask
        },
    },
    { // initCals
        0x4EFE,  // calMask
        0xFF,  // rxChannelMask
        0xFF,  // txChannelMask
        0x03,  // orxChannelMask
        0,  // warmBoot
    },
};
