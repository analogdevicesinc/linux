## 48   Thermal Monitoring Unit (TMU)

The TMU provides on-chip temperature measurement which is important in applications that have substantial power consumption. The TMU is integrated into the processor die and digital infrastructure using an MMR-based system access to measure the die temperature variations in real-time.

## TMU Features

The TMU supports the following features:

- On-chip temperature sensing
- Programmable over-temperature and under-temperature limits
- Programmable conversion rate
- Programmable clock source selection to run the sensor off an independent local clock
- Averaging feature available
- Temperature gain and offset correction options

## TMU Functional Description

Following sections provide the functional description of TMU.

## ADSP-SC58x TMU Register List

Thermal monitoring unit

Table 48-1: ADSP-SC58x TMU Register List

| Name            | Description               |
|-----------------|---------------------------|
| TMU_ALRT_LIM_HI | Alert High Limit Register |
| TMU_ALRT_LIM_LO | Alert Low Limit Register  |
| TMU_AVG         | Averaging Register        |
| TMU_CTL         | TMUControl Register       |

Table 48-1: ADSP-SC58x TMU Register List (Continued)

| Name           | Description                |
|----------------|----------------------------|
| TMU_FLT_LIM_HI | Fault High Limit Register  |
| TMU_FLT_LIM_LO | Fault Low Limit Register   |
| TMU_GAIN       | Gain Value Register        |
| TMU_IMSK       | Interrupt Mask Register    |
| TMU_OFFSET     | Offset Register            |
| TMU_STAT       | Status Register            |
| TMU_TEMP       | Temperature Value Register |

## ADSP-SC58x TMU Interrupt List

Table 48-2: ADSP-SC58x TMU Interrupt List

|   Interrupt ID | Name       | Description   | Sensitivity   | DMA Channel   |
|----------------|------------|---------------|---------------|---------------|
|              6 | TMU0_FAULT | TMU0 Fault    |               |               |

## ADSP-SC58x TMU Trigger List

Table 48-3: ADSP-SC58x TMU Trigger List Masters

|   Trigger ID | Name       | Description      | Sensitivity   |
|--------------|------------|------------------|---------------|
|          131 | TMU0_FAULT | TMU0 Fault Event |               |

Table 48-4: ADSP-SC58x TMU Trigger List Slaves

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
|              |        | None          |               |

## TMU Definitions

The following definitions are helpful when using the TMU module.

## Thermal Diode

A special type of diode whose electrical properties change with the temperature in a defined way.

## DTM

Dynamic thermal management is a set of techniques that adapt the run-time behavior of a processor to achieve the highest performance under thermal constraints.

## TMU Block Diagram

The Thermal Monitoring Unit Block Diagram shows the main block inside TMU.

Figure 48-1: Thermal Monitoring Unit Block Diagram

![Image](51_Thermal_Monitoring_Unit_(TMU)_artifacts/image_000000_0fc42260d24eccac43636a12ba12df6dff7896faede65afc2513946a4b1475bc.png)

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

The TMU functions as a digital thermometer with an MMR-based system access. The on-die temperature value is measured and digitized through an A/D converter. The temperature value is stored and updated periodically in an MMR register. The TMU can be configured to generate an interrupt as it crosses the upper temperature limit ( TMU\_FLT\_LIM\_HI register). A thermal event is also routed to the interrupt port of the sensor which is routed to the SEC, to ensure that core intervention is not required in the event of overheating.

## Temperature Sensor Averaging

A temperature sensor averaging feature is to enhance the accuracy of the temperature measurements. To enable the temperature sensor averaging feature, the TMU\_AVG bits must be enabled in the control register after power-up. In this mode, the averaging reduces the effect of noise on the temperature result. The temperature is measured each time a conversion is performed and a moving average method is used to determine the result in the temperature value register. The total time to measure a temperature channel is typically 1 ms.

## TMU Event Control

The TMU generates different events depending on the state of the TMU temperature measurement and the different thresholds set in thresh hold registers. These events are reported in the TMU\_STAT register as shown below. It can generate an event for each of following conditions.

- The fault high limit is configured in the TMU\_FLT\_LIM\_HI register. The interrupt is generated when the temperature value is greater than or equal to this value.
- The alert high limit is configured in the TMU\_ALRT\_LIM\_HI register. The TMU\_STAT.ALRTHI status bit is set when the temperature value is greater than or equal to this value.
- The fault low limit is configured in the TMU\_FLT\_LIM\_LO register. The TMU\_STAT.FLTLO status bit is set when the temperature value is less than or equal to this value.
- The alert low limit is configured in the TMU\_ALRT\_LIM\_LO register. The TMU\_STAT.FLTLO status bit is set when the temperature value is less than or equal to this value.

Interrupts and status conditions can be masked (disabled) or unmasked (enabled) by setting and clearing bits in the TMU\_IMSK register.

NOTE: Only TMU\_FLT\_LIM\_HI generates an interrupt to core. Other events only result in status change in the TMU\_STAT register.

## Status and Error Signals

When the measured temperature value exceeds the high or low limits that are configured in the TMU\_FLT\_LIM\_HI / TMU\_FLT\_LIM\_LO and the TMU\_ALRT\_LIM\_HI / TMU\_ALRT\_LIM\_LO registers, corresponding thermal events are triggered.

The fault and alert events are sent to the core through the status register ( TMU\_STAT ). Both the TMU\_STAT.FLTHI and TMU\_STAT.ALRTHI bits are sticky bits and are cleared by a W1C operation by the core.

The alert and fault registers can be programmed through the MMR bus interface as shown in the TMU block diagram. A write into the TMU\_FLT\_LIM\_HI / TMU\_FLT\_LIM\_LO and TMU\_ALRT\_LIM\_HI / TMU\_ALRT\_LIM\_LO registers or the TMU\_TEMP register is not allowed when the events are being triggered.

## TMU Programming Guidelines

The following section provides basic programming information for the TMU module.

To get the best performance and accuracy from the TMU, intialize the TMU\_GAIN , TMU\_OFFSET and TMU\_AVG registers before enabling the module. Contact Analog Devices, Inc for the current best values to use.

After these registers are programmed, the rest of the TMU initialization can take place. This includes setting up the fault and alert limits and then enabling the TMU.

## ADSP-SC58x TMU Register Descriptions

Thermal monitoring unit (TMU) contains the following registers.

Table 48-5: ADSP-SC58x TMU Register List

| Name            | Description                |
|-----------------|----------------------------|
| TMU_ALRT_LIM_HI | Alert High Limit Register  |
| TMU_ALRT_LIM_LO | Alert Low Limit Register   |
| TMU_AVG         | Averaging Register         |
| TMU_CTL         | TMUControl Register        |
| TMU_FLT_LIM_HI  | Fault High Limit Register  |
| TMU_FLT_LIM_LO  | Fault Low Limit Register   |
| TMU_GAIN        | Gain Value Register        |
| TMU_IMSK        | Interrupt Mask Register    |
| TMU_OFFSET      | Offset Register            |
| TMU_STAT        | Status Register            |
| TMU_TEMP        | Temperature Value Register |

## Alert High Limit Register

The TMU\_ALRT\_LIM\_HI register sets the temperature alert high limit as an integer value. The value is stored in two's complement format. Asserts TMU\_STAT.ALRTHI if the TMU\_TEMP value is greater than or equal to TMU\_ALRT\_LIM\_HI . The TMU\_ALRT\_LIM\_HI value should be programmed for value greater than 8'h3C (60 degC).

Figure 48-2: TMU\_ALRT\_LIM\_HI Register Diagram

![Image](51_Thermal_Monitoring_Unit_(TMU)_artifacts/image_000001_70b23f850f2e3cb03c5367e2c628492f79824c7d3aef3dfa3dadb0249707060d.png)

Table 48-6: TMU\_ALRT\_LIM\_HI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Value. The TMU_ALRT_LIM_HI.VALUE bit field configures the alert temperature high limit as an integer value stored in two's complement format. If the temperature value is greater than or equal to the high-limit the TMU_STAT.ALRTHI bit is set. The limit- value should be programmed for value greater than 8h3C (60 degC). |

## Alert Low Limit Register

The TMU\_ALRT\_LIM\_LO register configures the alert temperature low limit.

Figure 48-3: TMU\_ALRT\_LIM\_LO Register Diagram

![Image](51_Thermal_Monitoring_Unit_(TMU)_artifacts/image_000002_30d76983fcf3fa22db6fddc38b3d64ad72430062acdc9d04911c5c7a71f6df02.png)

Table 48-7: TMU\_ALRT\_LIM\_LO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Value. The TMU_ALRT_LIM_LO.VALUE bit field configures the alert temperature low limit as an integer value stored in two's complement format. If the temperature value is less than or equal to the low-limit the TMU_STAT.ALRTLO bit is set. |

## Averaging Register

The TMU\_AVG register enables averaging on the TMU.

Figure 48-4: TMU\_AVG Register Diagram

![Image](51_Thermal_Monitoring_Unit_(TMU)_artifacts/image_000003_5dca3e1f624a12405342f254e0ae32dd6be7064e155038b9abd1917a9f793f71.png)

Table 48-8: TMU\_AVG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | AVG_EN     | Value. The TMU_AVG.AVG_EN bit enables averaging on the TMU. Averaging is done using the formula (7 x previous_avg_value + current_value)/8. Initially the current_value is taken as previous_avg_value. 0 No averaging |
| 0 (R/W)            | AVG_EN     | 1 Enable averaging                                                                                                                                                                                                     |
| 0 (R/W)            | AVG_EN     |                                                                                                                                                                                                                        |

## TMU Control Register

The TMU\_CTL register contains bits that allow programs to configure and enable the TMU.

Figure 48-5: TMU\_CTL Register Diagram

![Image](51_Thermal_Monitoring_Unit_(TMU)_artifacts/image_000004_a7ab2496e2b8202f69b73ef011d9d71dd936154fe929705cebefef18d6e13171.png)

Table 48-9: TMU\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:4 (R/W)          | OSCDIV     | Oscillator Divide Ratio. The TMU_CTL.OSCDIV bit field configures the division ratio for the SCLK. SCLK is divided by (21 + 4 * TMU_CTL.OSCDIV ). By default the SCLK is divided by 21.                   |
| 3 (R/W)            | STARTCNV   | Start Conversion. When the TMU_CTL.STARTCNV bit is set, ADC conversion starts. 0 ADC conversion not started                                                                                              |
| 1 (R/W)            | OSCSEL     | Oscillator Select. The TMU_CTL.OSCSEL bit selects the local oscillator. By default the TMUworks off the divided system clock (SCLK).                                                                     |
| 0 (R/W)            | EN         | Enable. The TMU_CTL.EN bit enables the TMU. By default the module is in power-down mode. The peripheral interface is active even in power-down mode (the TMUregisters can be read/written). 0 DisableTMU |
| 0 (R/W)            | EN         |                                                                                                                                                                                                          |
| 0 (R/W)            | EN         |                                                                                                                                                                                                          |

## Fault High Limit Register

The TMU\_FLT\_LIM\_HI register sets the temperature fault high limit as an integer value. The value is stored in two's complement format. Asserts TMU\_STAT.FLTHI if the TMU\_TEMP value is greater than or equal to TMU\_FLT\_LIM\_HI . The TMU\_FLT\_LIM\_HI value should be programmed for value greater than 8'h3C (60 degC).

Figure 48-6: TMU\_FLT\_LIM\_HI Register Diagram

![Image](51_Thermal_Monitoring_Unit_(TMU)_artifacts/image_000005_70b23f850f2e3cb03c5367e2c628492f79824c7d3aef3dfa3dadb0249707060d.png)

Table 48-10: TMU\_FLT\_LIM\_HI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Value. The TMU_FLT_LIM_HI.VALUE bit field sets the temperature high limit as an inte- ger in two's complement format. If the limit value is greater than or equal to the high- limit the TMU_STAT.FLTHI bit is set. The limit-value should be programmed for value greater than 8h3C (60 degC). |

## Fault Low Limit Register

The TMU\_FLT\_LIM\_LO register configures the fault temperature low limit.

Figure 48-7: TMU\_FLT\_LIM\_LO Register Diagram

![Image](51_Thermal_Monitoring_Unit_(TMU)_artifacts/image_000006_30d76983fcf3fa22db6fddc38b3d64ad72430062acdc9d04911c5c7a71f6df02.png)

Table 48-11: TMU\_FLT\_LIM\_LO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Value. The TMU_FLT_LIM_LO.VALUE bit field configures the fault temperature low limit as an integer which is stored in two's complement format. If the temperature value is less than or equal to the low-limit, the TMU_STAT.FLTLO bit is set. |

## Gain Value Register

The TMU\_GAIN register is used to configure the gain value in two's complement format. This value is used to correct the gain error in the TMU\_TEMP register.

Figure 48-8: TMU\_GAIN Register Diagram

![Image](51_Thermal_Monitoring_Unit_(TMU)_artifacts/image_000007_afa6b81a3f1afa6af523b1f28f548eccac41b020e88bdd7e21ea81f01539344e.png)

Table 48-12: TMU\_GAIN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0                | VALUE      | Value. The TMU_GAIN.VALUE bit field configures the gain value in two's complement for- mat. This value is used to correct the gain error in the TMU_TEMP register. |
| (R/W)              |            |                                                                                                                                                                    |

## Interrupt Mask Register

The TMU\_IMSK register provides bit that are used to mask and unmask the interrupts associated with the TMU module.

Figure 48-9: TMU\_IMSK Register Diagram

![Image](51_Thermal_Monitoring_Unit_(TMU)_artifacts/image_000008_3df2c5b4e4ab4d8259b352b31b237cea60103c1c95d67e09be27324dd72bc2e7.png)

Table 48-13: TMU\_IMSK Register Fields

| Bit No. (Access)   | Bit Name                                                                                          | Description/Enumeration                                                                                                                    |
|--------------------|---------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | ALRTLO                                                                                            | Alert Low Mask. The TMU_IMSK.ALRTLO bit masks or unmasks the alert low status change ( TMU_STAT.ALRTLO ).                                  |
| 2 (R/W)            | FLTLO Fault Low The (                                                                             | Mask. TMU_IMSK.FLTLO bit masks or unmasks the fault low status change TMU_STAT.FLTLO ).                                                    |
| 1 (R/W)            | ALRTHI                                                                                            | 1 Mask fault low status change Alert High Mask. The TMU_IMSK.ALRTHI bit masks or unmasks the alert high status change ( TMU_STAT.ALRTHI ). |
| 0 (R/W)            | FLTHI Fault High Mask. The TMU_IMSK.FLTHI bit masks or unmasks the fault high ( TMU_STAT.FLTHI ). | interrupt                                                                                                                                  |

## Offset Register

The value programmed in the TMU\_OFFSET register is used to correct the offset error in TMU\_TEMP register.

Figure 48-10: TMU\_OFFSET Register Diagram

![Image](51_Thermal_Monitoring_Unit_(TMU)_artifacts/image_000009_b695b49106a501ca117316c1fcc486242e3916cc784dd5fa58c1d1521cdc74f0.png)

Table 48-14: TMU\_OFFSET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:0 (R/W)         | VALUE      | Value. The TMU_OFFSET.VALUE bit field provides the offset value which is used to correct the offset error in the TMU_TEMP register. This value is in a two's complement fixed point Q3.8 format where the 4 MSB's represent the integer part in two's complement format and the remaining 8 LSB's represent the decimal part. The offset value is ap- plied after multiplying the gain. |

## Status Register

The TMU\_STAT register bits indicate when an error or fault is detected.

Figure 48-11: TMU\_STAT Register Diagram

![Image](51_Thermal_Monitoring_Unit_(TMU)_artifacts/image_000010_df13788aba4350ab429fc7dab150e7a20000f06ef6f420c79f8fa74157663d0c.png)

Table 48-15: TMU\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/NW)           | ALRTLO     | Alert Low. The TMU_STAT.ALRTLO bit is set when the temperature value is less than or equal to the setting in the TMU_ALRT_LIM_LO register. The bit assertion is masked if TMU_IMSK.ALRTLO is HIGH.                                                                                                                                                                                                                                                                                                                              |
| 6 (R/NW)           | FLTLO      | Fault Low. The TMU_STAT.FLTLO bit is set when the temperature value is less than or equal to the setting in the TMU_FLT_LIM_LO register. The bit assertion is masked if TMU_IMSK.FLTLO is HIGH.                                                                                                                                                                                                                                                                                                                                 |
| 5 (R/W1C)          | ALRTHI     | Alert High. The TMU_STAT.ALRTHI bit is set when the temperature value is greater than or equal to the setting in the TMU_ALRT_LIM_HI register. When Write1toClear is done on TMU_STAT.ALRTHI , it will be cleared only if the TMU_TEMP value is less than TMU_ALRT_LIM_HI value. If the bit is cleared by 'Write1toClear', The TMU_TEMP register is set to 16'h2C00 (effective temperature value of 44). The value 16'h2C00 will be changed according to gain, offset values from TMU_GAIN , TMU_OFFSET registers respectively. |
| 4 (R/W1C)          | FLTHI      | Fault High. The TMU_STAT.FLTHI bit is set when the temperature value is greater than or equal to the setting in the TMU_FLT_LIM_HI register. When Write1toClear is done on TMU_STAT.FLTHI , it will be cleared only if the TMU_TEMP value is less than TMU_FLT_LIM_HI value. If the bit is cleared by 'Write1toClear', The TMU_TEMP register is set to 16'h2C00 (effective temperature value of 44). The value 16'h2C00 will be changed according to gain, offset values from TMU_GAIN , TMU_OFFSET reg- isters respectively.   |

## Temperature Value Register

The TMU\_TEMP register provides the temperature value from the A/D converter and the status.

Figure 48-12: TMU\_TEMP Register Diagram

![Image](51_Thermal_Monitoring_Unit_(TMU)_artifacts/image_000011_598d1e630731b0a0ced911a1128a0841be82a5a152df8eb711d9671a2f311193.png)

Table 48-16: TMU\_TEMP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/NW)        | VALUE      | Value. The TMU_TEMP.VALUE bit field is the temperature value from A/D converter. This value is stored in two's complement fixed point Q7.8 format where the 8 MSB's repre- sent the integer part in 2's complement, and the remaining 8 LSB's represent the deci- mal part. |