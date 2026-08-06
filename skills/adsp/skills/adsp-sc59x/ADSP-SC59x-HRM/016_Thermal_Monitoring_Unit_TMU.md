# Thermal Monitoring Unit (TMU)

<!-- source: 016_Thermal_Monitoring_Unit_TMU.pdf | original pages 711–730 -->

## 14   Thermal Monitoring Unit (TMU)

The TMU provides on-chip temperature measurement which is important in applications that have substantial power consumption. The TMU is integrated into the processor die and digital infrastructure using an MMR-based system access to measure the die temperature variations in real-time.

## TMU Features

The TMU supports the following features:

- On-chip temperature sensing
- Programmable over-temperature and under-temperature limits
- Programmable conversion rate
- Averaging feature available
- Temperature gain and offset correction options
- Uses dedicated channel of HADC for conversion
- Programmable blanking and refresh period for TMU conversion

## TMU Functional Description

Following sections provide the functional description of TMU.

## ADSP-SC59x TMU Register List

Thermal monitoring unit

Table 14-1: ADSP-SC59x TMU Register List

| Name            | Description               |
|-----------------|---------------------------|
| TMU_ALRT_LIM_HI | Alert High Limit Register |
| TMU_ALRT_LIM_LO | Alert Low Limit Register  |
| TMU_AVG         | Averaging Register        |

Table 14-1: ADSP-SC59x TMU Register List (Continued)

| Name           | Description                           |
|----------------|---------------------------------------|
| TMU_CNV_BLANK  | Temperature Conversion Blank Register |
| TMU_CTL        | TMUControl Register                   |
| TMU_FLT_LIM_HI | Fault High Limit Register             |
| TMU_FLT_LIM_LO | Fault Low Limit Register              |
| TMU_GAIN       | Gain Value Register                   |
| TMU_IMSK       | Interrupt Mask Register               |
| TMU_OFFSET     | Offset Register                       |
| TMU_REFR_CNTR  | Temperature Refresh Counter           |
| TMU_STAT       | Status Register                       |
| TMU_TEMP       | Temperature Value Register            |

## ADSP-SC59x TMU Interrupt List

Table 14-2: ADSP-SC59x TMU Interrupt List

|   Interrupt ID | Name       | Description   | Sensitivity   | DMA Channel   |
|----------------|------------|---------------|---------------|---------------|
|              7 | TMU0_FAULT | TMU0 Fault    |               |               |
|              8 | TMU0_ALERT | TMU0 Fault    |               |               |

## ADSP-SC59x TMU Trigger List

Table 14-3: ADSP-SC59x TMU Trigger List Generators

|   Trigger ID | Name       | Description          | Sensitivity   |
|--------------|------------|----------------------|---------------|
|          153 | TMU0_FAULT | TMU0 TM0 Fault Event |               |
|          154 | TMU0_ALERT | TMU0 TM0 Alert Event |               |

Table 14-4: ADSP-SC59x TMU Trigger List Receivers

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         | None   | None          | None          |

## TMU Definitions

The following definitions are helpful when using the TMU module.

## Thermal Diode

A special type of diode whose electrical properties change with the temperature in a defined way.

## DTM

Dynamic thermal management is a set of techniques that adapt the run-time behavior of a processor to achieve the highest performance under thermal constraints.

## TMU Block Diagram

The Thermal Monitoring Unit Block Diagram shows the main block inside TMU.

Figure 14-1: Thermal Monitoring Unit Block Diagram

<!-- image -->

## TMU Architectural Concepts

The following sections provide information on the architecture and system integration of the TMU.

## System Integration

Dynamic thermal management (DTM) techniques adapt the run-time behavior of a processor to achieve the highest performance under thermal constraints. One of the most important aspects of DTM is to capture the run-time variations in the temperature caused by power consumption variations due to workload changes.

The accuracy of thermal measurements directly affects the efficiency of thermal management as well as the performance of the processor. Temperature estimations lower or higher than the actual temperature may cause late or early activation of DTM techniques.

- Late activation of DTM can result in degraded reliability because the temperature may exceed the designated thresholds.
- Early activation of DTM can have significant impact on performance.

The TMU is an analog thermal sensor that consists of a temperature-sensing diode, a calibrated reference current source, and a current comparator. The sensor placement error is one of the most important sources of inaccuracy in values obtained from thermal sensors.

The TMU module provides the temperature monitoring capability to the chip and implements thermal management in the end system. It provides many features which ensure the minimum load on the software and also minimal or no external components for a flexible temperature monitoring system.

## Digital Thermometer

The TMU functions as a digital thermometer with an MMR-based system access. The on-die temperature value is measured and digitized through an A/D converter. The temperature value is stored and updated periodically in an MMR register. The TMU can be configured to generate an interrupt as it crosses the upper temperature limit ( TMU\_FLT\_LIM\_HI register). A thermal event is also routed to the interrupt port of the sensor. The event is then routed to the SEC, to ensure that core intervention is not required in the event of overheating.

## Temperature Sensor Averaging

The temperature sensor averaging feature enhances the accuracy of the temperature measurements. To enable this feature, the TMU\_AVG bits must be enabled in the control register after power-up. In this mode, the averaging reduces the effect of noise on the temperature result. The temperature is measured each time a conversion is performed. A moving average method is used to determine the result in the temperature value register. The total time to measure a temperature channel is typically 1 ms.

## TMU and HADC

The TMU modules uses one of the ADC channels to digitize the temperature reading. Therefore, the HADC module is shared by the TMU. If the TMU is not enabled, the HADC operates at its full throughput. TMU can be enabled periodically or permanently. In periodic enable mode, it is enabled once in a specific refresh period. After enabling, the TMU measures the temperature for a specific blanking period. During the blanking period, the TMU uses the ADC for temperature measurement, so that the ADC is not available for data conversion.

The typical value of blanking period is 21× 10 4  system clock (SCLK0) cycles

The typical value of refresh period is 21×10 6  SCLK0 cycles

So, for every 21×10 6 cycles, 21× 10 4  cycles are used for temperature measurement. Program these values for blanking period and refresh period through the TMU registers.

When TMU is enabled, the HADC modules is not available for external input conversion. This state is indicated by the HADC\_STAT.TMUHADC\_BUSY status bit. All the requests for HADC conversion are ignored when HADC\_STAT.TMUHADC\_BUSY bit is high.

In the periodic enable mode, the ongoing HADC conversion is completed activating the TMU to measure temperature as shown. If HADC is in autoscan mode, the TMU is activated after completing the ongoing sequence. If HADC is in fixed-conversion mode, all the fixed conversions are completed and followed by TMU activation.

## TMU Event Control

The TMU generates different events depending on the state of the TMU temperature measurement and the different thresholds set in thresh hold registers. These events are reported in the TMU\_STAT register as shown below. It can generate an event for each of following conditions.

- The fault high limit is configured in the TMU\_FLT\_LIM\_HI register. The interrupt is generated when the temperature value is greater than or equal to this value.
- The alert high limit is configured in the TMU\_ALRT\_LIM\_HI register. The interrupt is generated when the temperature value is greater than or equal to this value.
- The fault low limit is configured in the TMU\_FLT\_LIM\_LO register. The TMU\_STAT.FLTLO status bit is set when the temperature value is less than or equal to this value.
- The alert low limit is configured in the TMU\_ALRT\_LIM\_LO register. The TMU\_STAT.FLTLO status bit is set when the temperature value is less than or equal to this value.

Interrupts and status conditions can be masked (disabled) or unmasked (enabled) by setting and clearing bits in the TMU\_IMSK register.

- NOTE: Only TMU\_FLT\_LIM\_HI and TMU\_ALRT\_LIM\_HI generate an interrupt to core. Other events only result in status change in the TMU\_STAT register.

## Status and Error Signals

When the measured temperature value exceeds the high or low limits that are configured in the TMU\_FLT\_LIM\_HI / TMU\_FLT\_LIM\_LO and the TMU\_ALRT\_LIM\_HI / TMU\_ALRT\_LIM\_LO registers, corresponding thermal events are triggered.

The fault and alert events are sent to the core through the status register ( TMU\_STAT ). Both the TMU\_STAT.FLTHI and TMU\_STAT.ALRTHI bits are sticky bits and are cleared by a W1C operation by the core.

The alert and fault registers can be programmed through the MMR bus interface as shown in the TMU block diagram. A write into the TMU\_FLT\_LIM\_HI / TMU\_FLT\_LIM\_LO and TMU\_ALRT\_LIM\_HI / TMU\_ALRT\_LIM\_LO registers or the TMU\_TEMP register is not allowed when the events are being triggered.

## TMU Programming Guidelines

The following section provides basic programming information for the TMU module.

To get the best performance and accuracy from the TMU, initialize the TMU\_GAIN , TMU\_OFFSET and TMU\_AVG registers before enabling the module.

After these registers are programmed, the rest of the TMU initialization can take place. This includes setting up the fault and alert limits and then enabling the TMU.

## TMU Calibration

Calibration settings are comprised of gain and offset settings. These settings must be programmed into the corresponding device registers before the temperature is read from the TMU temperature value register. The calibration setting ensures that the reported temperature approximates the actual die temperature (within the error specified in the data sheet). The specified accuracy of the reported temperature cannot be met if gain and offset settings are not used.

The TMU configuration software programs gain and offset values as part of the TMU initialization process. The gain value is programmed in the TMU\_GAIN.VALUE field. The offset value is programmed in the TMU\_OFFSET.VALUE bit field. The TMU Calibration Settings table provides the calibration setting for the processor.

Table 14-5: TMU Calibration Settings

| Bit Field        | Programmed Value   |
|------------------|--------------------|
| TMU_GAIN.VALUE   | 0x004              |
| TMU_OFFSET.VALUE | 0x7D40             |

NOTE: Refer to the product data sheet for the expected TMU accuracy achieved using the settings.

NOTE: The calibration settings provide expected TMU performance at temperatures above room temperature. The accuracy is not guaranteed for temperatures below room temperature.

## ADSP-SC59x TMU Register Descriptions

Thermal monitoring unit (TMU) contains the following registers.

Table 14-6: ADSP-SC59x TMU Register List

| Name            | Description                           |
|-----------------|---------------------------------------|
| TMU_ALRT_LIM_HI | Alert High Limit Register             |
| TMU_ALRT_LIM_LO | Alert Low Limit Register              |
| TMU_AVG         | Averaging Register                    |
| TMU_CNV_BLANK   | Temperature Conversion Blank Register |
| TMU_CTL         | TMUControl Register                   |
| TMU_FLT_LIM_HI  | Fault High Limit Register             |

Table 14-6: ADSP-SC59x TMU Register List (Continued)

| Name           | Description                 |
|----------------|-----------------------------|
| TMU_FLT_LIM_LO | Fault Low Limit Register    |
| TMU_GAIN       | Gain Value Register         |
| TMU_IMSK       | Interrupt Mask Register     |
| TMU_OFFSET     | Offset Register             |
| TMU_REFR_CNTR  | Temperature Refresh Counter |
| TMU_STAT       | Status Register             |
| TMU_TEMP       | Temperature Value Register  |

## Alert High Limit Register

The TMU\_ALRT\_LIM\_HI register sets the temperature alert high limit as an integer value. The value is stored in two's complement format. Asserts TMU\_STAT.ALRTHI if the TMU\_TEMP value is greater than or equal to TMU\_ALRT\_LIM\_HI . The TMU\_ALRT\_LIM\_HI value should be programmed for value greater than 8'h3C (60 degC).

Figure 14-2: TMU\_ALRT\_LIM\_HI Register Diagram

<!-- image -->

Table 14-7: TMU\_ALRT\_LIM\_HI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8:0 (R/W)          | VALUE      | Value. The TMU_ALRT_LIM_HI.VALUE bit field configures the alert temperature high limit as an integer value stored in two's complement format. If the temperature value is greater than or equal to the high-limit the TMU_STAT.ALRTHI bit is set. |

## Alert Low Limit Register

The TMU\_ALRT\_LIM\_LO register configures the alert temperature low limit.

Figure 14-3: TMU\_ALRT\_LIM\_LO Register Diagram

<!-- image -->

Table 14-8: TMU\_ALRT\_LIM\_LO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8:0 (R/W)          | VALUE      | Value. The TMU_ALRT_LIM_LO.VALUE bit field configures the alert temperature low limit as an integer value stored in two's complement format. If the temperature value is less than or equal to the low-limit the TMU_STAT.ALRTLO bit is set. |

## Averaging Register

The TMU\_AVG register enables averaging on the TMU.

Figure 14-4: TMU\_AVG Register Diagram

<!-- image -->

Table 14-9: TMU\_AVG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | EN         | Value. The TMU_AVG.EN bit enables averaging on the TMU. Averaging is done using the formula (7 x previous_avg_value + current_value)/8. Initially the current_value is tak- en as previous_avg_value. 0 No averaging 1 Enable averaging |

## Temperature Conversion Blank Register

The TMU\_CNV\_BLANK register specifies blanking period in which TMU utilizes HADC for temperature conversion.

The conversion time is calculated as: (VALUE+1)*50k SCLK cycles. So, the minimum conversion time is 50k SCLK cycles. Default conversion time is 200k SCLK cycles. For this duration, the system cannot use HADC for data conversion and HADC does take input for data-conversion. The status is indicated by TMUHADC\_BUSY bit of the HADC block.

Figure 14-5: TMU\_CNV\_BLANK Register Diagram

<!-- image -->

Table 14-10: TMU\_CNV\_BLANK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 28:24              | PERIOD     | Blanking Period.          |
| (R/W)              |            |                           |

## TMU Control Register

The TMU\_CTL register contains bits that allow programs to configure and enable the TMU.

Figure 14-6: TMU\_CTL Register Diagram

<!-- image -->

Table 14-11: TMU\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                | Description/Enumeration                                                                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | TMEN_FORCE | Indefinite Enable. Asserting the TMU_CTL.TMEN_FORCE bit enables the TMUindefinitely. HADC is always used by the TMU, so the HADC cannot be used to convert channel inputs.                                                                                                             | Indefinite Enable. Asserting the TMU_CTL.TMEN_FORCE bit enables the TMUindefinitely. HADC is always used by the TMU, so the HADC cannot be used to convert channel inputs.                                                                                                             |
| 11:4 (R/W)         | SCLKDIV    | System Clock Divide. The TMU_CTL.SCLKDIV bit selects the division ratio for the system clock (SCLK). SCLK is divided by (21+4*SCLKDIV). Thus, by default the SCLK is divided by 21.                                                                                                    | System Clock Divide. The TMU_CTL.SCLKDIV bit selects the division ratio for the system clock (SCLK). SCLK is divided by (21+4*SCLKDIV). Thus, by default the SCLK is divided by 21.                                                                                                    |
| 3 (R/W)            | TMEN       | Periodic Enable. Asserting the TMU_CTL.TMEN bit enables the TMUperiodically. TMUis enabled once in every refresh period defined by the TMU_REFR_CNTR register. And in each refresh period, TMUmeasures temperature for the blanking period (Tblank). It is de- fined by CNV_BLANK bit. | Periodic Enable. Asserting the TMU_CTL.TMEN bit enables the TMUperiodically. TMUis enabled once in every refresh period defined by the TMU_REFR_CNTR register. And in each refresh period, TMUmeasures temperature for the blanking period (Tblank). It is de- fined by CNV_BLANK bit. |
|                    |            | 0                                                                                                                                                                                                                                                                                      | DisableTMU                                                                                                                                                                                                                                                                             |
|                    |            | 1                                                                                                                                                                                                                                                                                      | Enable TMUperiodically                                                                                                                                                                                                                                                                 |
| 0 (R/W)            | TMPU       | TMUEnable. The TMU_CTL.TMPU bit enables the TMU. By default, the module is in power- down mode. The peripheral interface is active even in power-down mode (theTMU registers can be read/written).                                                                                     | TMUEnable. The TMU_CTL.TMPU bit enables the TMU. By default, the module is in power- down mode. The peripheral interface is active even in power-down mode (theTMU registers can be read/written).                                                                                     |
|                    |            | 0                                                                                                                                                                                                                                                                                      | Power Down the analog circuitry ofTMU                                                                                                                                                                                                                                                  |
|                    |            | 1                                                                                                                                                                                                                                                                                      | Power Up the analog circuitry ofTMU                                                                                                                                                                                                                                                    |

## Fault High Limit Register

The TMU\_FLT\_LIM\_HI register sets the temperature fault high limit as an integer value. The value is stored in two's complement format. Asserts TMU\_STAT.FLTHI if the TMU\_TEMP value is greater than or equal to TMU\_FLT\_LIM\_HI . The TMU\_FLT\_LIM\_HI value should be programmed for value greater than 8'h3C (60 degC).

Figure 14-7: TMU\_FLT\_LIM\_HI Register Diagram

<!-- image -->

Table 14-12: TMU\_FLT\_LIM\_HI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8:0 (R/W)          | VALUE      | Value. The TMU_FLT_LIM_HI.VALUE bit field sets the temperature high limit as an inte- ger in two's complement format. If the limit value is greater than or equal to the high- limit the TMU_STAT.FLTHI bit is set. |

## Fault Low Limit Register

The TMU\_FLT\_LIM\_LO register configures the fault temperature low limit.

Figure 14-8: TMU\_FLT\_LIM\_LO Register Diagram

<!-- image -->

Table 14-13: TMU\_FLT\_LIM\_LO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8:0 (R/W)          | VALUE      | Value. The TMU_FLT_LIM_LO.VALUE bit field configures the fault temperature low limit as an integer which is stored in two's complement format. If the temperature value is less than or equal to the low-limit, the TMU_STAT.FLTLO bit is set. |

## Gain Value Register

The TMU\_GAIN register is used to configure the gain value in two's complement format. This value is used to correct the gain error in the TMU\_TEMP register.

Figure 14-9: TMU\_GAIN Register Diagram

<!-- image -->

Table 14-14: TMU\_GAIN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9:0                | VALUE      | Value. The TMU_GAIN.VALUE bit field configures the gain value in two's complement for- mat. This value is used to correct the gain error in the TMU_TEMP register. |
| (R/W)              |            |                                                                                                                                                                    |

## Interrupt Mask Register

The TMU\_IMSK register provides bits that are used to mask and unmask the interrupts associated with the TMU module.

Figure 14-10: TMU\_IMSK Register Diagram

<!-- image -->

Table 14-15: TMU\_IMSK Register Fields

| Bit No. (Access)   | Bit Name                                                                                          | Description/Enumeration                                                                                                                    |
|--------------------|---------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | ALRTLO                                                                                            | Alert Low Mask. The TMU_IMSK.ALRTLO bit masks or unmasks the alert low status change ( TMU_STAT.ALRTLO ).                                  |
| 2 (R/W)            | FLTLO Fault Low The (                                                                             | Mask. TMU_IMSK.FLTLO bit masks or unmasks the fault low status change TMU_STAT.FLTLO ).                                                    |
| 1 (R/W)            | ALRTHI                                                                                            | 1 Mask fault low status change Alert High Mask. The TMU_IMSK.ALRTHI bit masks or unmasks the alert high status change ( TMU_STAT.ALRTHI ). |
| 0 (R/W)            | FLTHI Fault High Mask. The TMU_IMSK.FLTHI bit masks or unmasks the fault high ( TMU_STAT.FLTHI ). | interrupt                                                                                                                                  |

## Offset Register

The value programmed in the TMU\_OFFSET register is used to correct the offset error in TMU\_TEMP register.

Figure 14-11: TMU\_OFFSET Register Diagram

<!-- image -->

Table 14-16: TMU\_OFFSET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:0 (R/W)         | VALUE      | Value. The TMU_OFFSET.VALUE bit field provides the offset value which is used to correct the offset error in the TMU_TEMP register. This value is in a two's complement fixed point Q7.7 format i.e 8 MSB's represent integer part in 2's complement format. and remaining 7 LSB's represent decimal part. Offset will be applied after multiplying gain. |

## Temperature Refresh Counter

The TMU\_REFR\_CNTR register defines the refresh-period to refresh temperature value.

Figure 14-12: TMU\_REFR\_CNTR Register Diagram

<!-- image -->

Table 14-17: TMU\_REFR\_CNTR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5:0 (R/W)          | VALUE      | Refresh Period. The TMU_REFR_CNTR.VALUE bit field defines the period to refresh the tempera- ture value. Temperature is refreshed for every (VALUE+1)*(21+4*SCLKDIV)*1M sys- tem-clock cycles. SCLKDIV is defined in TMU_CTL register. So, by default, the peri- od is 21 million system-clock cycles. |

## Status Register

The TMU\_STAT register bits indicate when an error or fault is detected.

Figure 14-13: TMU\_STAT Register Diagram

<!-- image -->

Table 14-18: TMU\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/NW)           | ALRTLO     | Alert Low. The TMU_STAT.ALRTLO bit is set when the temperature value is less than or equal to the setting in the TMU_ALRT_LIM_LO register. The bit assertion is masked if TMU_IMSK.ALRTLO is HIGH.                                                                                                                                                  |
| 6 (R/NW)           | FLTLO      | Fault Low. The TMU_STAT.FLTLO bit is set when the temperature value is less than or equal to the setting in the TMU_FLT_LIM_LO register. The bit assertion is masked if TMU_IMSK.FLTLO is HIGH.                                                                                                                                                     |
| 5 (R/W1C)          | ALRTHI     | Alert High. The TMU_STAT.ALRTHI bit is set when the temperature value is greater than or equal to the setting in the TMU_ALRT_LIM_HI register. The temperature value reg- ister is set to 16'h0D80 (effective temperature value of 27degrees C) when a W1C op- eration is done on this bit. The bit assertion is masked if TMU_IMSK.ALRTHI is HIGH. |
| 4 (R/W1C)          | FLTHI      | Fault High. The TMU_STAT.FLTHI bit is set when the temperature value is greater than or equal to the setting in the TMU_FLT_LIM_HI register. The temperature value regis- ter is set to 16'h0D80 (effective temperature value of 27 degrees C) when a W1C is done on this bit. The bit assertion is masked if TMU_IMSK.FLTHI is HIGH.               |

## Temperature Value Register

The TMU\_TEMP register provides the temperature value from the A/D converter and the status.

Figure 14-14: TMU\_TEMP Register Diagram

<!-- image -->

Table 14-19: TMU\_TEMP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/NW)        | VALUE      | Value. The TMU_TEMP.VALUE bit field is the temperature value from A/D converter. This value is stored in two's complement fixed point Q8.7 format where the 9 MSB's repre- sent the integer part in 2's complement, and the remaining 7 LSB's represent the deci- mal part. |