/**
* \file
* \brief Contains Aux ADC features related function implementation defined in
* adi_adrv910x_auxadc.h
*
* ADRV910X API Version: $ADI_ADRV910X_API_VERSION$
*/

/**
* Copyright 2025 Analog Devices Inc.
* Released under the ADRV910X API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#include "adi_adrv910x_arm.h"
#include "adi_adrv910x_auxadc.h"
#include "adrv910x_bf.h"
/* System header files */
#ifdef __KERNEL__
#include <linux/kernel.h>
#else
#include <stdlib.h>
#include <math.h>
#endif
static __maybe_unused int32_t __maybe_unused adi_adrv910x_AuxAdc_Configure_Validate(adi_adrv910x_Device_t *device,
	adi_adrv910x_AuxAdc_e auxAdc)
{
	ADI_RANGE_CHECK(device, auxAdc, ADI_ADRV910X_AUXADC0, ADI_ADRV910X_AUXADC1);
	ADI_API_RETURN(device);
}

int32_t adi_adrv910x_AuxAdc_Configure(adi_adrv910x_Device_t *device,
	adi_adrv910x_AuxAdc_e auxAdc,
	bool enable)
{
	uint8_t bfValue = (true == enable) ? 0 : 1;
	static const uint8_t AUXADC_RESET_HIGH = 0x1;
	static const uint8_t AUXADC_RESET_LOW = 0x0;
	/* Value '3' is used to select port0 in the SPI register configuration. */
	static const uint8_t AUXADC_PORT_SEL = 3;

	ADI_PERFORM_VALIDATION(adi_adrv910x_AuxAdc_Configure_Validate, device, auxAdc);

	if (auxAdc == ADI_ADRV910X_AUXADC0)
	{
		ADI_EXPECT(adrv910x_NvsRegmapCore_AuxAdc0Reset_Set, device, AUXADC_RESET_HIGH);
		ADI_EXPECT(adrv910x_NvsRegmapCore1_AuxAdcClkDiv_Set, device, 0x6);
		ADI_EXPECT(adrv910x_NvsRegmapCore1_TempSenseClkArmSel_Set, device, 0x1);
		ADI_EXPECT(adrv910x_NvsRegmapCore_AuxAdc0DecimationCtl_Set, device, 0x0);
		ADI_EXPECT(adrv910x_NvsRegmapCore3_AuxAdc0Sel_Set, device, AUXADC_PORT_SEL);
		ADI_EXPECT(adrv910x_NvsRegmapCore1_AuxAdc0ClockEnable_Set, device, 0x1);
		ADI_EXPECT(adrv910x_NevisMonitorRegmapCore_AuxAdc0Pd_Set, device, bfValue);
		ADI_EXPECT(adrv910x_NvsRegmapCore_AuxAdc0Reset_Set, device, AUXADC_RESET_LOW);
	}
	else
	{
		ADI_EXPECT(adrv910x_NvsRegmapCore_AuxAdc1Reset_Set, device, AUXADC_RESET_HIGH);
		ADI_EXPECT(adrv910x_NvsRegmapCore1_AuxAdcClkDiv_Set, device, 0x6);
		ADI_EXPECT(adrv910x_NvsRegmapCore1_TempSenseClkArmSel_Set, device, 0x1);
		ADI_EXPECT(adrv910x_NvsRegmapCore_AuxAdc1DecimationCtl_Set, device, 0x0);
		ADI_EXPECT(adrv910x_NvsRegmapCore3_AuxAdc1Sel_Set, device, AUXADC_PORT_SEL);
		ADI_EXPECT(adrv910x_NvsRegmapCore1_AuxAdc1ClockEnable_Set, device, 0x1);
		ADI_EXPECT(adrv910x_NevisMonitorRegmapCore_AuxAdc1Pd_Set, device, bfValue);
		ADI_EXPECT(adrv910x_NvsRegmapCore_AuxAdc1Reset_Set, device, AUXADC_RESET_LOW);
	}

	ADI_API_RETURN(device);
}

static __maybe_unused int32_t __maybe_unused adi_adrv910x_AuxAdc_Inspect_Validate(adi_adrv910x_Device_t *device,
	adi_adrv910x_AuxAdc_e auxAdc,
	bool *enabled)
{
	ADI_NULL_PTR_RETURN(&device->common, enabled);
	ADI_RANGE_CHECK(device, auxAdc, ADI_ADRV910X_AUXADC0, ADI_ADRV910X_AUXADC1);
	ADI_API_RETURN(device);
}

int32_t adi_adrv910x_AuxAdc_Inspect(adi_adrv910x_Device_t *device,
	adi_adrv910x_AuxAdc_e auxAdc,
	bool *enabled)
{
	uint8_t bfEnable = 0;

	ADI_PERFORM_VALIDATION(adi_adrv910x_AuxAdc_Inspect_Validate, device, auxAdc, enabled);

	if (auxAdc == ADI_ADRV910X_AUXADC0)
	{
		ADI_EXPECT(adrv910x_NevisMonitorRegmapCore_AuxAdc0Pd_Get, device, &bfEnable);
	}
	else
	{
		ADI_EXPECT(adrv910x_NevisMonitorRegmapCore_AuxAdc1Pd_Get, device, &bfEnable);
	}
	*enabled = (0 == bfEnable) ? true : false;

	ADI_API_RETURN(device);
}

static __maybe_unused int32_t __maybe_unused adi_adrv910x_AuxAdc_Voltage_Get_Validate(adi_adrv910x_Device_t *device,
	adi_adrv910x_AuxAdc_e auxAdc,
	uint16_t *auxAdc_mV)
{
	ADI_NULL_PTR_RETURN(&device->common, auxAdc_mV);
	ADI_RANGE_CHECK(device, auxAdc, ADI_ADRV910X_AUXADC0, ADI_ADRV910X_AUXADC1);

	ADI_API_RETURN(device);
}

int32_t adi_adrv910x_AuxAdc_Voltage_Get(adi_adrv910x_Device_t *device,
	adi_adrv910x_AuxAdc_e auxAdc,
	uint16_t *auxAdc_mV)
{
	uint16_t auxAdcCode = 0;
	uint32_t auxAdcWait, armClk_Hz, auxAdcClk_Hz, status;	
	uint8_t refClkDiv, hsClkDiv, armClkDiv, auxAdcClkSel;
	static const uint16_t MEASURED_OFFSET = 0;
	static const uint16_t MEASURED_GAIN = 4096;
	
	//These settings are hardcoded in auxadc configure
	static const uint32_t AUXADC_CLK_DIV_ENCODED = 63; // Encoded clock divider value (x6 ==> x3F or d63)
	uint32_t auxadcClkDiv = AUXADC_CLK_DIV_ENCODED;
	uint32_t decimator = 2048; //decimator is hardcoded as x0. This corresponds to 2048.
	
	ADI_PERFORM_VALIDATION(adi_adrv910x_AuxAdc_Voltage_Get_Validate, device, auxAdc, auxAdc_mV);
	
	ADI_EXPECT(adrv910x_NvsRegmapCore1_AuxAdcClkArmSel_Get, device, &auxAdcClkSel);
	if (auxAdcClkSel == 0)
	{
		//Clock is ref clock
		ADI_EXPECT(adrv910x_NevisMonitorRegmapCore_RefClkIntDevclkDivideRatio_Get, device, &refClkDiv);
		//read ref clk divider		
		auxAdcClk_Hz = (device->devStateInfo.devClock_Hz) / (1U << refClkDiv);
	}
	
	else
	{	
		armClk_Hz = device->devStateInfo.hsDigFreq_Hz/ device->devStateInfo.ps1ArmClkDiv;
	}
	
	/* Get 12 bit ADC word from selected AuxADC */
	if (auxAdc == ADI_ADRV910X_AUXADC0)
	{
		ADI_EXPECT(adrv910x_NvsRegmapCore3_AuxAdc0Linearity_Set, device, 0x800);
		ADI_EXPECT(adrv910x_NvsRegmapCore3_AuxAdc0DecLinearDataCapture_Set, device, 0x1);
		//Add harware configurable timer based on AUXADC clock
		auxAdcWait = 2000000 / (auxAdcClk_Hz  / auxadcClkDiv / decimator);
		adi_common_hal_Wait_us(&device->common, auxAdcWait);
		ADI_EXPECT(adrv910x_NvsRegmapCore3_AuxAdc0ReadData_Get, device, &auxAdcCode);
	}
	else
	{
		ADI_EXPECT(adrv910x_NvsRegmapCore3_AuxAdc1Linearity_Set, device, 0x800);
		ADI_EXPECT(adrv910x_NvsRegmapCore3_AuxAdc1DecLinearDataCapture_Set, device, 0x1);
		//Add harware configurable timer based on AUXADC clock
		auxAdcWait = 2000000 / (auxAdcClk_Hz  / auxadcClkDiv / decimator);	
		adi_common_hal_Wait_us(&device->common, auxAdcWait);
		ADI_EXPECT(adrv910x_NvsRegmapCore3_AuxAdc1ReadData_Get, device, &auxAdcCode);
	}
	/*TODO: offset and gain may be read from ADRV910X device for this calculation */
	/* AUXADC_mV = (auxAdcCode - meausured_offset) / measured_gain */
	*auxAdc_mV = 1000 * (auxAdcCode - MEASURED_OFFSET) / MEASURED_GAIN;

	ADI_API_RETURN(device);
}

