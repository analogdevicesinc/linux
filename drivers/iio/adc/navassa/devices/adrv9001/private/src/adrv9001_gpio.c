/**
* \file
* \brief Contains private functions related to GPIO feature implementation defined in
*        adrv9001_gpio.h
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
/* "adi_adrv9001_user.h" contains the #define that other header file use */
#include "adi_adrv9001_user.h"

/* Header file corresponding to the C file */
#include "adi_adrv9001_gpio.h"

/* ADI specific header files */
#include "adi_adrv9001.h"
#include "adi_adrv9001_error.h"
#include "adrv9001_gpio.h"
#include "adrv9001_reg_addr_macros.h"
#include "adrv9001_bf.h"

int32_t adrv9001_GpIntHandler(adi_adrv9001_Device_t *device, adi_adrv9001_gpIntStatus_t *gpIntStatus)
{
    int32_t recoveryAction = 10000;
    const char * errMsg = NULL;

    static const uint32_t ADI_ADRV9001_GP_MASK_RX_DP_RECEIVE_ERROR          = 0x08000000; /* bit27 */
    static const uint32_t ADI_ADRV9001_GP_MASK_TX_DP_TRANSMIT_ERROR         = 0x04000000; /* bit26 */
    static const uint32_t ADI_ADRV9001_GP_MASK_RX_DP_READ_REQUEST_FROM_BBIC = 0x02000000; /* bit25*/
    static const uint32_t ADI_ADRV9001_GP_MASK_TX_DP_WRITE_REQUEST_TO_BBIC  = 0x01000000; /* bit24 */
    static const uint32_t ADI_ADRV9001_GP_MASK_STREAM_PROCESSOR_3_ERROR     = 0x00100000; /* bit20 */
    static const uint32_t ADI_ADRV9001_GP_MASK_STREAM_PROCESSOR_2_ERROR     = 0x00080000; /* bit19 */
    static const uint32_t ADI_ADRV9001_GP_MASK_STREAM_PROCESSOR_1_ERROR     = 0x00040000; /* bit18 */
    static const uint32_t ADI_ADRV9001_GP_MASK_STREAM_PROCESSOR_0_ERROR     = 0x00020000; /* bit17 */
    static const uint32_t ADI_ADRV9001_GP_MASK_MAIN_STREAM_PROCESSOR_ERROR  = 0x00010000; /* bit16 */
    static const uint32_t ADI_ADRV9001_GP_MASK_LSSI_RX2_CLK_MCS             = 0x00008000; /* bit15 */
    static const uint32_t ADI_ADRV9001_GP_MASK_LSSI_RX1_CLK_MCS             = 0x00004000; /* bit14 */
    static const uint32_t ADI_ADRV9001_GP_MASK_CLK_1105_MCS_SECOND          = 0x00002000; /* bit13 */
    static const uint32_t ADI_ADRV9001_GP_MASK_CLK_1105_MCS                 = 0x00001000; /* bit12 */
    static const uint32_t ADI_ADRV9001_GP_MASK_CLK_PLL_LOCK                 = 0x00000800; /* bit11 */
    static const uint32_t ADI_ADRV9001_GP_MASK_AUX_PLL_LOCK                 = 0x00000400; /* bit10 */
    static const uint32_t ADI_ADRV9001_GP_MASK_RF2_SYNTH_LOCK               = 0x00000200; /* bit09 */
    static const uint32_t ADI_ADRV9001_GP_MASK_RF_SYNTH_LOCK                = 0x00000100; /* bit08 */
    static const uint32_t ADI_ADRV9001_GP_MASK_CLK_PLL_LOW_POWER_LOCK       = 0x00000080; /* bit07 */
    static const uint32_t ADI_ADRV9001_GP_MASK_TX2_PA_PROTECTION_ERROR      = 0x00000040; /* bit06 */
    static const uint32_t ADI_ADRV9001_GP_MASK_TX1_PA_PROTECTION_ERROR      = 0x00000020; /* bit05 */
    static const uint32_t ADI_ADRV9001_GP_MASK_CORE_ARM_MONITOR_ERROR       = 0x00000010; /* bit04 */
    static const uint32_t ADI_ADRV9001_GP_MASK_CORE_ARM_CALIBRATION_ERROR   = 0x00000008; /* bit03 */
    static const uint32_t ADI_ADRV9001_GP_MASK_CORE_ARM_SYSTEM_ERROR        = 0x00000004; /* bit02 */
    static const uint32_t ADI_ADRV9001_GP_MASK_CORE_FORCE_GP_INTERRUPT      = 0x00000002; /* bit01 */
    static const uint32_t ADI_ADRV9001_GP_MASK_CORE_ARM_ERROR               = 0x00000001; /* bit00 */

    /* test for RX DP receive error IRQ bit 27 */
    if ((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_RX_DP_RECEIVE_ERROR) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: SPI interface receive error";
    }
    /* test for TX DP transmit error IRQ bit 26 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_TX_DP_TRANSMIT_ERROR) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: SPI interface transmit error";
    }
    /* test for RX DP read request IRQ bit 25 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_RX_DP_READ_REQUEST_FROM_BBIC) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: RX DP read request error";
    }
    /* test for TX DP write request IRQ bit 24 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_TX_DP_WRITE_REQUEST_TO_BBIC) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: TX DP write request error";
    }
    /* test for stream3 IRQ bit 20 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_STREAM_PROCESSOR_3_ERROR) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: stream3 error";
    }
    /* test for stream2 IRQ bit 19 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_STREAM_PROCESSOR_2_ERROR) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: stream2 error";
    }
    /* test for stream1 IRQ bit 18 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_STREAM_PROCESSOR_1_ERROR) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: stream1 error";
    }
    /* test for stream0 IRQ bit 17 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_STREAM_PROCESSOR_0_ERROR) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: stream0 error";
    }
    /* test for core stream bit 16 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_MAIN_STREAM_PROCESSOR_ERROR) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: core stream processor error";
    }
    /* test for RX2 LSSI MCS bit 15 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_LSSI_RX2_CLK_MCS) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: RX2 LSSI MCS";
    }
   /* test for RX1 LSSI MCS bit 14 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_LSSI_RX1_CLK_MCS) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: RX1 LSSI MCS";
    }
    /* test for main clock 1105 second MCS bit 13 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_CLK_1105_MCS_SECOND) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: main clock 1105 second MCS";
    }
    /* test for main clock 1105 MCS bit 12 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_CLK_1105_MCS) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: main clock 1105 MCS";
    }
    /* test for Clock PLL lock indicator bit 11 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_CLK_PLL_LOCK) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: Clock PLL lock indicator";
    }
    /* test for auxillary Clock PLL lock indicator bit 10 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_AUX_PLL_LOCK) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: auxiliary Clock PLL lock indicator";
    }
    /* test for RF PLL lock2 bit 09 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_RF2_SYNTH_LOCK) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: RF PLL2 lock indicator";
    }
    /* test for RF PLL lock1 bit 08 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_RF_SYNTH_LOCK) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: RF PLL1 lock indicatord";
    }
    /* test for low power PLL lock indicator bit 07 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_CLK_PLL_LOW_POWER_LOCK) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: low-power PLL lock indicator";
    }
    /* test for TX2 PA protection error bit 06 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_TX2_PA_PROTECTION_ERROR) > 0)
    {
        recoveryAction = ADI_ADRV9001_ACT_ERR_BBIC_LOG_ERROR;
        errMsg = "GP Interrupt from source: TX2 PA protection error";
    }
    /* test for TX1 PA protection error bit 05 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_TX1_PA_PROTECTION_ERROR) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: TX1 PA protection error";
    }
    /* test for ARM monitor interrupt bit 04 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_CORE_ARM_MONITOR_ERROR) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: ARM monitor interrupt";
    }
    /* test for ARM calibration Error bit 03 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_CORE_ARM_CALIBRATION_ERROR) > 0)
    {
        recoveryAction = ADI_ADRV9001_ACT_WARN_RERUN_TRCK_CAL;
        errMsg = "GP Interrupt from source: ARM Calibration error";
    }
    /* test for ARM System Error bit 02 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_CORE_ARM_SYSTEM_ERROR) > 0)
    {
        recoveryAction = ADI_ADRV9001_ACT_ERR_BBIC_LOG_ERROR;
        errMsg = "GP Interrupt from source: ARM System error";
    }
    /* test for GP interrupt source bit 01 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_CORE_FORCE_GP_INTERRUPT) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: Force GP interrupt(Set by firmware to send an interrupt to BBIC)";
    }
    /* test for ARM Error bit 00 */
    else if((gpIntStatus->gpIntActiveSources & ADI_ADRV9001_GP_MASK_CORE_ARM_ERROR) > 0)
    {
        recoveryAction = ADI_COMMON_ACT_ERR_RESET_FULL;
        errMsg = "GP Interrupt from source: ARM error";
    }
    else
    {
        recoveryAction = ADI_ADRV9001_ACT_ERR_BBIC_LOG_ERROR;
        errMsg = "Unknown GP Interrupt source";
    }

    if (recoveryAction == 10000)
    {
        recoveryAction = ADI_COMMON_ACT_NO_ACTION;
    }

    ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API, ADI_ADRV9001_ERR_GP_INTERRUPT, recoveryAction, gpIntStatus->gpIntStatus, errMsg);
    ADI_ERROR_RETURN(device->common.error.newAction);

    return device->common.error.newAction;
}

int32_t adrv9001_GpInterruptsMaskPinBfSet(adi_adrv9001_Device_t *device, uint32_t bfValue)
{
    int32_t status = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    /* Write to Field 8 bits */
    status = adi_bf_hal_Register_Write(device, (0x200 + 0x9E), (uint8_t)(bfValue >> 24));
    if (0 != status) return status;

    /* Write to Field 8 bits */
    status = adi_bf_hal_Register_Write(device, (0x200 + 0x9F), (uint8_t)(bfValue >> 16));
    if (0 != status) return status;

    /* Write to Field 8 bits */
    status = adi_bf_hal_Register_Write(device, (0x200 + 0xA0), (uint8_t)(bfValue >> 8));
    if (0 != status) return status;

    /* Write to Field 8 bits */
    status = adi_bf_hal_Register_Write(device, (0x200 + 0xA1), (uint8_t)(bfValue >> 0));

    return status;
}

int32_t adrv9001_GpInterruptsMaskPinBfGet(adi_adrv9001_Device_t *device, uint32_t *bfValue)
{
    int32_t status = 0;
    uint8_t register_value = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

    ADI_FUNCTION_ENTRY_LOG(&device->common);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

    /* Read Field 8 bits */
    status = adi_bf_hal_Register_Read(device, (0x200 + 0x9E), &register_value);
    *bfValue = 0;
    if (0 != status) return status;
    *bfValue = (*bfValue << 8) | register_value;

    /* Read Field 8 bits */
    status = adi_bf_hal_Register_Read(device, (0x200 + 0x9F), &register_value);
    if (0 != status) return status;
    *bfValue = (*bfValue << 8) | register_value;

    /* Read Field 8 bits */
    status = adi_bf_hal_Register_Read(device, (0x200 + 0xA0), &register_value);
    if (0 != status) return status;
    *bfValue = (*bfValue << 8) | register_value;

    /* Read Field 8 bits */
    status = adi_bf_hal_Register_Read(device, (0x200 + 0xA1), &register_value);
    *bfValue = (*bfValue << 8) | register_value;

    return status;
}

int32_t adrv9001_GpInterruptsStatusWordBfGet(adi_adrv9001_Device_t *device, uint32_t *bfValue)
{
    int32_t status = ADI_COMMON_ACT_NO_ACTION;
    uint8_t register_value = 0;

    ADI_NULL_DEVICE_PTR_RETURN(device);

#ifdef ADRV9001_BITFIELD_NULL_CHECK
    /* NULL check */
    ADI_NULL_PTR_RETURN(&device->common, bfValue);
#endif /* ADRV9001_BITFIELD_NULL_CHECK */

    /* Read Field 8 bits */
    status = adi_bf_hal_Register_Read(device, (0x200 + 0xA3), &register_value);
    *bfValue = 0;
    if (0 != status) return status;
    *bfValue = (*bfValue << 8) | register_value;

    /* Read Field 8 bits */
    status = adi_bf_hal_Register_Read(device, (0x200 + 0xA4), &register_value);
    if (0 != status) return status;
    *bfValue = (*bfValue << 8) | register_value;

    /* Read Field 8 bits */
    status = adi_bf_hal_Register_Read(device, (0x200 + 0xA5), &register_value);
    if (0 != status) return status;
    *bfValue = (*bfValue << 8) | register_value;

    /* Read Field 8 bits */
    status = adi_bf_hal_Register_Read(device, (0x200 + 0xA6), &register_value);
    *bfValue = (*bfValue << 8) | register_value;

    return status;
}

int32_t adrv9001_GpCaseConfig(adi_adrv9001_Device_t *device, uint8_t tx1portEn, uint8_t tx2portEn, adi_adrv9001_GpioOptions_e gpioOptions)
{
	uint8_t tx1portDisable = 0;
	uint8_t tx2portDisable = 0;
	uint8_t ch2Powerup = 0;

	/* Check device pointer is not null */
	ADI_NULL_DEVICE_PTR_RETURN(device);

	if (tx1portEn)
	{
		tx1portDisable = 0;
	}
	else
	{
		tx1portDisable = 1;
	}

	if (tx2portEn)
	{
		tx2portDisable = 0;
	}
	else
	{
		tx2portDisable = 1;
	}

	ADI_EXPECT(adrv9001_NvsRegmapCore1_SsiTx1PortDisable_Set, device, tx1portDisable);
	ADI_EXPECT(adrv9001_NvsRegmapCore1_SsiTx2PortDisable_Set, device, tx2portDisable);

	if (gpioOptions == ADI_ADRV9001_GPIO_12_13_INPUT)
	{
		/* set refclk pad src ctrl as gpio12-13 */
		ADI_EXPECT(adrv9001_NvsRegmapCore1_SsiCmosGpioTx1RefclkGpioSel_Set, device, 0x01);
		/* set adrv pin as input */
		ADI_EXPECT(adrv9001_NvsRegmapCore_Tx1RefclkCmosNOe_Set, device, 0x00);
		ADI_EXPECT(adrv9001_NvsRegmapCore_Tx1RefclkCmosPOe_Set, device, 0x00);
		ADI_EXPECT(adrv9001_NvsRegmapCore_Tx1RefclkCmosNIe_Set, device, 0x01);
		ADI_EXPECT(adrv9001_NvsRegmapCore_Tx1RefclkCmosPIe_Set, device, 0x01);
	}
	else if (gpioOptions == ADI_ADRV9001_GPIO_12_13_OUTPUT)
	{
		/* set refclk pad src ctrl as gpio12-13 */
		ADI_EXPECT(adrv9001_NvsRegmapCore1_SsiCmosGpioTx1RefclkGpioSel_Set, device, 0x01);
		/* set adrv pin as output */
		ADI_EXPECT(adrv9001_NvsRegmapCore_Tx1RefclkCmosNOe_Set, device, 0x01);
		ADI_EXPECT(adrv9001_NvsRegmapCore_Tx1RefclkCmosPOe_Set, device, 0x01);
		ADI_EXPECT(adrv9001_NvsRegmapCore_Tx1RefclkCmosNIe_Set, device, 0x00);
		ADI_EXPECT(adrv9001_NvsRegmapCore_Tx1RefclkCmosPIe_Set, device, 0x00);
	}
	else if (gpioOptions == ADI_ADRV9001_GPIO_14_15_INPUT)
	{
		/* set refclk pad src ctrl as gpio14-15 */
		ADI_EXPECT(adrv9001_NvsRegmapCore1_SsiCmosGpioTx2RefclkGpioSel_Set, device, 0x01);
		/* set adrv pin as input */
		ADI_EXPECT(adrv9001_NvsRegmapCore_Tx2RefclkCmosNOe_Set, device, 0x00);
		ADI_EXPECT(adrv9001_NvsRegmapCore_Tx2RefclkCmosPOe_Set, device, 0x00);
		ADI_EXPECT(adrv9001_NvsRegmapCore_Tx2RefclkCmosNIe_Set, device, 0x01);
		ADI_EXPECT(adrv9001_NvsRegmapCore_Tx2RefclkCmosPIe_Set, device, 0x01);
	}
	else if (gpioOptions == ADI_ADRV9001_GPIO_14_15_OUTPUT)
	{
		/* set refclk pad src ctrl as gpio14-15 */
		ADI_EXPECT(adrv9001_NvsRegmapCore1_SsiCmosGpioTx2RefclkGpioSel_Set, device, 0x01);
		/* set adrv pin as output */
		ADI_EXPECT(adrv9001_NvsRegmapCore_Tx2RefclkCmosNOe_Set, device, 0x01);
		ADI_EXPECT(adrv9001_NvsRegmapCore_Tx2RefclkCmosPOe_Set, device, 0x01);
		ADI_EXPECT(adrv9001_NvsRegmapCore_Tx2RefclkCmosNIe_Set, device, 0x00);
		ADI_EXPECT(adrv9001_NvsRegmapCore_Tx2RefclkCmosPIe_Set, device, 0x00);

		ADI_EXPECT(adrv9001_NvsRegmapCore1_PgChannel2PowerEn_Get, device, &ch2Powerup);
		if (!ch2Powerup)
		{
			/* Power up CH2 */
			/* Turn on power switch */
			ADI_EXPECT(adrv9001_NvsRegmapCore1_PgChannel2PowerEn_Set, device, 0x01);
			/* Turn off ISO cells */
			ADI_EXPECT(adrv9001_NvsRegmapCore1_PgChannel2PowerIsoDis_Set, device, 0x00);
			/* Turn on power latches */
			ADI_EXPECT(adrv9001_NvsRegmapCore1_PgChannel2PowerScanLatchEn_Set, device, 0x01);
			/* Release reset */
			ADI_EXPECT(adrv9001_NvsRegmapCore1_PgChannel2PowerResetb_Set, device, 0x01);
		}
	}

	ADI_API_RETURN(device);
}

static __maybe_unused int32_t __maybe_unused adrv9001_GpCaseSet_Validate(adi_adrv9001_Device_t *device, adi_adrv9001_GpioOptions_e gpioOptions)
{
	ADI_RANGE_CHECK(device, gpioOptions, ADI_ADRV9001_GPIO_12_13_INPUT, ADI_ADRV9001_GPIO_14_15_OUTPUT);
	ADI_API_RETURN(device);
}

int32_t adrv9001_GpCaseSet(adi_adrv9001_Device_t *device, adi_adrv9001_GpioOptions_e gpioOptions)
{
	uint8_t txSsiRefClockEnable[2] = { 0 };
	uint8_t tx1channelEnable = 0;
	uint8_t tx2channelEnable = 0;
	uint8_t channel1Enabled = 0;
	uint8_t channel2Enabled = 0;

	ADI_PERFORM_VALIDATION(adrv9001_GpCaseSet_Validate, device, gpioOptions);

	txSsiRefClockEnable[0] = device->devStateInfo.txSsiRefClockEnabled[0];
	txSsiRefClockEnable[1] = device->devStateInfo.txSsiRefClockEnabled[1];

	if (ADI_ADRV9001_TX1 == (device->devStateInfo.initializedChannels & ADI_ADRV9001_TX1))
	{
		tx1channelEnable = 1;
	}
	else
	{
		tx1channelEnable = 0;
	}

	if (ADI_ADRV9001_TX2 == (device->devStateInfo.initializedChannels & ADI_ADRV9001_TX2))
	{
		tx2channelEnable = 1;
	}
	else
	{
		tx2channelEnable = 0;
	}

	if ((ADI_ADRV9001_RX1 == (device->devStateInfo.initializedChannels & ADI_ADRV9001_RX1)) || (ADI_ADRV9001_TX1 == (device->devStateInfo.initializedChannels & ADI_ADRV9001_TX1)))
	{
		channel1Enabled = 1;
	}
	else
	{
		channel1Enabled = 0;
	}

	if ((ADI_ADRV9001_RX2 == (device->devStateInfo.initializedChannels & ADI_ADRV9001_RX2)) || (ADI_ADRV9001_TX2 == (device->devStateInfo.initializedChannels & ADI_ADRV9001_TX2)))
	{
		channel2Enabled = 1;
	}
	else
	{
		channel2Enabled = 0;
	}

	if (tx1channelEnable && txSsiRefClockEnable[0] && tx2channelEnable && txSsiRefClockEnable[1] && channel1Enabled && channel2Enabled)
	{
		/* Case - 1 */
		if ((gpioOptions == ADI_ADRV9001_GPIO_12_13_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_12_13_OUTPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_OUTPUT))
		{
			ADI_ERROR_REPORT(&device->common,
				             ADI_COMMON_ERRSRC_API,
				             ADI_COMMON_ERR_API_FAIL,
				             ADI_COMMON_ACT_ERR_CHECK_PARAM,
				             gpioOptions,
				             "Case-1 : GPIO 12-15 already used as Ref Clk, not usable as GPIO.");
		}
	}
	else if (tx1channelEnable && txSsiRefClockEnable[0] && tx2channelEnable && (!txSsiRefClockEnable[1]) && channel1Enabled && channel2Enabled)
	{
		/* Case - 2 */
		if (gpioOptions == ADI_ADRV9001_GPIO_14_15_INPUT)
		{
			ADI_EXPECT(adrv9001_GpCaseConfig, device, tx1channelEnable, tx2channelEnable, gpioOptions);
		}
		else if ((gpioOptions == ADI_ADRV9001_GPIO_12_13_OUTPUT) || (gpioOptions == ADI_ADRV9001_GPIO_12_13_INPUT))
		{
			ADI_ERROR_REPORT(&device->common,
				             ADI_COMMON_ERRSRC_API,
				             ADI_COMMON_ERR_API_FAIL,
				             ADI_COMMON_ACT_ERR_CHECK_PARAM,
				             gpioOptions,
				             "Case-2 : GPIO 12-13 already used as Ref Clk, not usable as GPIO.");
		}
		else if (gpioOptions == ADI_ADRV9001_GPIO_14_15_OUTPUT)
		{
			ADI_ERROR_REPORT(&device->common,
				             ADI_COMMON_ERRSRC_API,
				             ADI_COMMON_ERR_API_FAIL,
				             ADI_COMMON_ACT_ERR_CHECK_PARAM,
				             gpioOptions,
				             "Case-2 : GPIO 14-15 not usable as Output");
		}
	}
	else if (tx1channelEnable && txSsiRefClockEnable[0] && (!tx2channelEnable) && (!txSsiRefClockEnable[1]) && channel1Enabled && channel2Enabled)
	{
		/* Case - 3 */
		if ((gpioOptions == ADI_ADRV9001_GPIO_14_15_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_OUTPUT))
		{
			ADI_EXPECT(adrv9001_GpCaseConfig, device, tx1channelEnable, tx2channelEnable, gpioOptions);
		}
		else if ((gpioOptions == ADI_ADRV9001_GPIO_12_13_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_12_13_OUTPUT))
		{
			ADI_ERROR_REPORT(&device->common,
				             ADI_COMMON_ERRSRC_API,
				             ADI_COMMON_ERR_API_FAIL,
				             ADI_COMMON_ACT_ERR_CHECK_PARAM,
				             gpioOptions,
				             "Case-3 : GPIO 12-13 already used as Ref Clk, not usable as GPIO.");
		}
	}
	else if (tx1channelEnable && txSsiRefClockEnable[0] && (!tx2channelEnable) && (!txSsiRefClockEnable[1]) && channel1Enabled && (!channel2Enabled))
	{
		/* Case - 4 */
		if ((gpioOptions == ADI_ADRV9001_GPIO_14_15_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_OUTPUT))
		{
			ADI_EXPECT(adrv9001_GpCaseConfig, device, tx1channelEnable, tx2channelEnable, gpioOptions);
		}
		else if ((gpioOptions == ADI_ADRV9001_GPIO_12_13_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_12_13_OUTPUT))
		{
			ADI_ERROR_REPORT(&device->common,
				             ADI_COMMON_ERRSRC_API,
				             ADI_COMMON_ERR_API_FAIL,
				             ADI_COMMON_ACT_ERR_CHECK_PARAM,
				             gpioOptions,
				            "Case-4 : GPIO 12-13 already used as Ref Clk, not usable as GPIO.");
		}
	}
	else if (tx1channelEnable && (!txSsiRefClockEnable[0]) && tx2channelEnable && txSsiRefClockEnable[1] && channel1Enabled && channel2Enabled)
	{
		/* Case - 5 */
		if (gpioOptions == ADI_ADRV9001_GPIO_12_13_INPUT)
		{
			ADI_EXPECT(adrv9001_GpCaseConfig, device, tx1channelEnable, tx2channelEnable, gpioOptions);
		}
		else if ((gpioOptions == ADI_ADRV9001_GPIO_14_15_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_OUTPUT))
		{
			ADI_ERROR_REPORT(&device->common,
				             ADI_COMMON_ERRSRC_API,
				             ADI_COMMON_ERR_API_FAIL,
				             ADI_COMMON_ACT_ERR_CHECK_PARAM,
				             gpioOptions,
				             "Case-5 : GPIO 14-15 already used as Ref Clk, not usable as GPIO.");
		}
		else if (gpioOptions == ADI_ADRV9001_GPIO_12_13_OUTPUT)
		{
			ADI_ERROR_REPORT(&device->common,
				             ADI_COMMON_ERRSRC_API,
				             ADI_COMMON_ERR_API_FAIL,
				             ADI_COMMON_ACT_ERR_CHECK_PARAM,
				             gpioOptions,
				             "Case-5 : GPIO 12-13 not usable as Output.");
		}
	}
	else if (tx1channelEnable && (!txSsiRefClockEnable[0]) && tx2channelEnable && (!txSsiRefClockEnable[1]) && channel1Enabled && channel2Enabled)
	{
		/* Case - 6 */
		if ((gpioOptions == ADI_ADRV9001_GPIO_12_13_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_INPUT))
		{
			ADI_EXPECT(adrv9001_GpCaseConfig, device, tx1channelEnable, tx2channelEnable, gpioOptions);
		}
		else if ((gpioOptions == ADI_ADRV9001_GPIO_12_13_OUTPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_OUTPUT))
		{
			ADI_ERROR_REPORT(&device->common,
				             ADI_COMMON_ERRSRC_API,
				             ADI_COMMON_ERR_API_FAIL,
				             ADI_COMMON_ACT_ERR_CHECK_PARAM,
				             gpioOptions,
				             "Case-6 : GPIO 12-15 not usable as Output.");
		}
	}
	else if (tx1channelEnable && (!txSsiRefClockEnable[0]) && (!tx2channelEnable) && (!txSsiRefClockEnable[1]) && channel1Enabled && channel2Enabled)
	{
		/* Case - 7 */
		if ((gpioOptions == ADI_ADRV9001_GPIO_12_13_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_OUTPUT))
		{
			ADI_EXPECT(adrv9001_GpCaseConfig, device, tx1channelEnable, tx2channelEnable, gpioOptions);
		}
		else if (gpioOptions == ADI_ADRV9001_GPIO_12_13_OUTPUT)
		{
			ADI_ERROR_REPORT(&device->common,
				             ADI_COMMON_ERRSRC_API,
				             ADI_COMMON_ERR_API_FAIL,
				             ADI_COMMON_ACT_ERR_CHECK_PARAM,
				             gpioOptions,
				             "Case-7 : GPIO 12-13 not usable as Output.");
		}
	}
	else if (tx1channelEnable && (!txSsiRefClockEnable[0]) && (!tx2channelEnable) && (!txSsiRefClockEnable[1]) && channel1Enabled && (!channel2Enabled))
	{
		/* Case - 8 */
		if ((gpioOptions == ADI_ADRV9001_GPIO_12_13_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_OUTPUT))
		{
			ADI_EXPECT(adrv9001_GpCaseConfig, device, tx1channelEnable, tx2channelEnable, gpioOptions);
		}
		else if (gpioOptions == ADI_ADRV9001_GPIO_12_13_OUTPUT)
		{
			ADI_ERROR_REPORT(&device->common,
				             ADI_COMMON_ERRSRC_API,
				             ADI_COMMON_ERR_API_FAIL,
				             ADI_COMMON_ACT_ERR_CHECK_PARAM,
				             gpioOptions,
				             "Case-8 : GPIO 12-13 not usable as Output.");
		}
	}
	else if ((!tx1channelEnable) && (!txSsiRefClockEnable[0]) && tx2channelEnable && (txSsiRefClockEnable[1]) && channel1Enabled && channel2Enabled)
	{
		/* Case - 9 */
		if ((gpioOptions == ADI_ADRV9001_GPIO_12_13_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_12_13_OUTPUT))
		{
			ADI_EXPECT(adrv9001_GpCaseConfig, device, tx1channelEnable, tx2channelEnable, gpioOptions);
		}
		else if ((gpioOptions == ADI_ADRV9001_GPIO_14_15_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_OUTPUT))
		{
			ADI_ERROR_REPORT(&device->common,
				             ADI_COMMON_ERRSRC_API,
				             ADI_COMMON_ERR_API_FAIL,
				             ADI_COMMON_ACT_ERR_CHECK_PARAM,
				             gpioOptions,
				             "Case-9 : GPIO 14-15 already used as Ref Clk, not usable as GPIO.");
		}
	}
	else if ((!tx1channelEnable) && (!txSsiRefClockEnable[0]) && tx2channelEnable && txSsiRefClockEnable[1] && (!channel1Enabled) && channel2Enabled)
	{
		/* Case - 10 */
		if ((gpioOptions == ADI_ADRV9001_GPIO_12_13_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_12_13_OUTPUT))
		{
			ADI_EXPECT(adrv9001_GpCaseConfig, device, tx1channelEnable, tx2channelEnable, gpioOptions);
		}
		else if ((gpioOptions == ADI_ADRV9001_GPIO_14_15_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_OUTPUT))
		{
			ADI_ERROR_REPORT(&device->common,
				             ADI_COMMON_ERRSRC_API,
				             ADI_COMMON_ERR_API_FAIL,
				             ADI_COMMON_ACT_ERR_CHECK_PARAM,
				             gpioOptions,
				             "Case-10 : GPIO 14-15 already used as Ref Clk, not usable as GPIO.");
		}
	}
	else if ((!tx1channelEnable) && (!txSsiRefClockEnable[0]) && tx2channelEnable && (!txSsiRefClockEnable[1]) && channel1Enabled && channel2Enabled)
	{
		/* Case - 11 */
		if ((gpioOptions == ADI_ADRV9001_GPIO_12_13_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_12_13_OUTPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_INPUT))
		{
			ADI_EXPECT(adrv9001_GpCaseConfig, device, tx1channelEnable, tx2channelEnable, gpioOptions);
		}
		else if (gpioOptions == ADI_ADRV9001_GPIO_14_15_OUTPUT)
		{
			ADI_ERROR_REPORT(&device->common,
				             ADI_COMMON_ERRSRC_API,
				             ADI_COMMON_ERR_API_FAIL,
				             ADI_COMMON_ACT_ERR_CHECK_PARAM,
				             gpioOptions,
				             "Case-11 : GPIO 14-15 not usable as Output.");
		}
	}
	else if ((!tx1channelEnable) && (!txSsiRefClockEnable[0]) && tx2channelEnable && (!txSsiRefClockEnable[1]) && (!channel1Enabled) && channel2Enabled)
	{
		/* Case - 12 */
		if ((gpioOptions == ADI_ADRV9001_GPIO_12_13_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_12_13_OUTPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_INPUT))
		{
			ADI_EXPECT(adrv9001_GpCaseConfig, device, tx1channelEnable, tx2channelEnable, gpioOptions);
		}
		else if (gpioOptions == ADI_ADRV9001_GPIO_14_15_OUTPUT)
		{
			ADI_ERROR_REPORT(&device->common,
				             ADI_COMMON_ERRSRC_API,
				             ADI_COMMON_ERR_API_FAIL,
				             ADI_COMMON_ACT_ERR_CHECK_PARAM,
				             gpioOptions,
				            "Case-12 : GPIO 14-15 not usable as Output.");
		}
	}
	else if ((!tx1channelEnable) && (!txSsiRefClockEnable[0]) && (!tx2channelEnable) && (!txSsiRefClockEnable[1]) && channel1Enabled && channel2Enabled)
	{
		/* Case - 13 */
		if ((gpioOptions == ADI_ADRV9001_GPIO_12_13_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_12_13_OUTPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_OUTPUT))
		{
			ADI_EXPECT(adrv9001_GpCaseConfig, device, tx1channelEnable, tx2channelEnable, gpioOptions);
		}
	}
	else if ((!tx1channelEnable) && (!txSsiRefClockEnable[0]) && (!tx2channelEnable) && (!txSsiRefClockEnable[1]) && channel1Enabled && (!channel2Enabled))
	{
		/* Case - 14 */
		if ((gpioOptions == ADI_ADRV9001_GPIO_12_13_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_12_13_OUTPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_OUTPUT))
		{
			ADI_EXPECT(adrv9001_GpCaseConfig, device, tx1channelEnable, tx2channelEnable, gpioOptions);
		}
	}
	else if ((!tx1channelEnable) && (!txSsiRefClockEnable[0]) && (!tx2channelEnable) && (!txSsiRefClockEnable[1]) && (!channel1Enabled) && channel2Enabled)
	{
		/* Case - 15 */
		if ((gpioOptions == ADI_ADRV9001_GPIO_12_13_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_12_13_OUTPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_INPUT) || (gpioOptions == ADI_ADRV9001_GPIO_14_15_OUTPUT))
		{
			ADI_EXPECT(adrv9001_GpCaseConfig, device, tx1channelEnable, tx2channelEnable, gpioOptions);
		}
	}
	else
	{
		ADI_ERROR_REPORT(&device->common,
			             ADI_COMMON_ERRSRC_API,
			             ADI_COMMON_ERR_INV_PARAM,
			             ADI_COMMON_ACT_ERR_CHECK_PARAM,
			             NULL,
			             "Invalid GPIO configuration");
	}

	ADI_API_RETURN(device);
}
