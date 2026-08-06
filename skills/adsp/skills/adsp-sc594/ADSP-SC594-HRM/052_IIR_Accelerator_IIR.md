# IIR Accelerator (IIR)

<!-- source: 052_IIR_Accelerator_IIR.pdf | original pages 2885–2933 -->

## 45   IIR Accelerator (IIR)

The processor includes an Infinite Impulse Response (IIR) filter accelerator implemented in hardware that reduces the processing load on the core, freeing it up for other tasks.

## Features

The accelerator supports a maximum of 32 channels in legacy mode. In Auto Configuration Mode (ACM), there is no limit on number of channels. The accelerator locally stores all the biquad coefficients of 32 channels in legacy mode. Window size can be configured from 1 (sample-based) to 1024.

The IIR has the following features:

- Supports IEEE floating point format 32/40-bit
- Rounding modes compatible with SHARC+ core MACs
- Sample-based or window-based processing
- Up to 64 cascaded biquads per channel
- Up to 32 filter channels available in TDM
- Allows biquad save state storage

NOTE: The IIR accelerator module has local memory that is not accessible by the core during regular operation mode. Unlike in the previous SHARC processors, the IIR accelerator modules each have access to the system memory (on-chip/off-chip).

Unlike in the previous SHARC processors, where only one of the IIR or FIR accelerators can be enabled at a time, the processor can simultaneously use both the IIR and the FIR accelerators.

## IIR Definitions

The following definitions are helpful when using the IIR filter.

## Infinite Impulse Response (IIR) filter

A recursive filter calculates its output by using current and previous inputs, along with previous outputs. IIR filters are digital filters that have an infinite impulse response.

## Automatic Configuration Mode (ACM)

The IIR accelerator loads biquad coefficients for only the current channel before processing the current channel.

## Legacy Mode

The IIR accelerator loads biquad coefficients for all channels before processing the first channel.

## Task Control Block (TCB)

A method to store task-specific parameters and calculation states in a user-initialized memory buffer and permitting user tasks to directly access the values. The reduction of the amount of time required for context switching can lessen the execution time for user tasks.

## Biquad (biquadratic)

A second-order recursive digital linear filter, containing two poles and two zeros. In the Z domain, its transfer function consists of the ratio of two quadratic functions. The biquad form II implements the equation in two parts with difference equations and two less storage delay components.

## Multiply and Accumulate (MAC) Unit

The IIR subblock with a pipelined multiplier and accumulator that performs floating-point calculations on data and fetches coefficients. Can perform 32-bit or 40-bit operations.

## Window-based Processing Mode

Includes using a window size of one or greater (maximum of 1024) for a channel.

## Save Biquad State Mode

The mode where the IIR filter stores the current biquad states in local memory when it must switch to another highpriority accelerator task. After resuming, these states can be reloaded, and IIR processing continued.

## Clocking

The IIR accelerator runs at the speed of core clock.

## Functional Description

The IIR Accelerator Block Diagram shows the blocks of the IIR hardware accelerator.

The accelerator has:

- a coefficient memory size of 1440 × 40 bits (288 biquads x 5 coefficients).
- a data memory size of 576 × 40 bits (288 biquads x 2 states).
- one SHARC+ core compatible MAC unit with an input data buffer to supply data to the MAC.

Figure 45-1: IIR Accelerator Block Diagram

<!-- image -->

A transposed direct form II biquadratic is used to implement the IIR accelerator, which reduces coefficient sensitivity. The Transposed Direct Form II Biquad figure shows the signal flow graph for the biquad structure.

Figure 45-2: Transposed Direct Form II Biquad

<!-- image -->

The accelerator has the following logical subblocks.

- A data path unit with the following elements:
- 32/40-bit coefficient memory (Ak, Bk) for storing biquad coefficients

- 32/40-bit input data (Xk) and state (Dk)
- One 40/32-bit floating-point multiplier and adder (MAC) unit
- An input data buffer to efficiently supply data to MAC
- One 40-bit result register to hold result of the biquad
- Configuration registers for controlling various parameters such as the number of biquads, the number of channels, interrupt control, and DMA control.
- A core access interface for writing the DMA/filter configuration registers and for reading the status registers.
- A DMA bus interface for transferring data to and from the accelerator. This interface also preloads the coefficients (Ak, Bk) and state (Dk) at startup.
- DMA configuration registers for the transfer of input data, output data, and coefficients.

The accelerator block is integrated with the SHARC core. For more information, see Figure 47-5 SHARC Fabric Connectivity.

## ADSP-2159x\_SC592\_SC594 IIR Register List

The IIR module reduces the processing load on the core. For more information on IIR functionality, see the IIR register descriptions.

Table 45-1: ADSP-2159x\_SC592\_SC594 IIR Register List

| Name             | Description                         |
|------------------|-------------------------------------|
| IIR_CHNPTR       | Chain Pointer Register              |
| IIR_COEFIDX      | Coefficient Buffer Index Register   |
| IIR_COEFLEN      | Coefficient Buffer Length Register  |
| IIR_COEFMOD      | Coefficient Index Modifier Register |
| IIR_CTL1         | Global Control Register             |
| IIR_CTL2         | Channel Control Register            |
| IIR_DBG_ADDR     | IIR Debug Address Register          |
| IIR_DBG_CTL      | IIR Debug Control Register          |
| IIR_DBG_RDDAT_HI | IIR Debug Read Data High Register   |
| IIR_DBG_RDDAT_LO | IIR Debug Read Data Low Register    |
| IIR_DBG_WRDAT_HI | IIR Debug Write Data High Register  |
| IIR_DBG_WRDAT_LO | IIR Debug Write Data Low Register   |
| IIR_DMASTAT      | DMAStatus Register                  |
| IIR_INBASE       | Input Buffer Base Register          |

Table 45-1: ADSP-2159x\_SC592\_SC594 IIR Register List (Continued)

| Name        | Description                             |
|-------------|-----------------------------------------|
| IIR_INIDX   | Input Data Index Register               |
| IIR_INLEN   | Input Data Buffer Length Register       |
| IIR_INMOD   | Input Data Index Modifier Register      |
| IIR_MACSTAT | MAC Status Register                     |
| IIR_OUTBASE | Output Buffer Base Register             |
| IIR_OUTIDX  | Output Data Buffer Index Register       |
| IIR_OUTLEN  | IIR Output Data Buffer Length Register  |
| IIR_OUTMOD  | IIR Output Data Index Modifier Register |
| IIR_SCTL1   | Software Control Register1              |
| IIR_SCTL2   | Software Control Register2              |
| IIR_SGCTL   | Secondary Global Control Register       |

## ADSP-2159x\_SC592\_SC594 IIR Interrupt List

Table 45-2: ADSP-2159x\_SC592\_SC594 IIR Interrupt List

|   Interrupt ID | Name      | Description        | Sensitivity   | DMA Channel   |
|----------------|-----------|--------------------|---------------|---------------|
|            168 | IIR0_DMA  | IIR0 Core1DMA      | Edge          |               |
|            169 | IIR0_STAT | IIR0 Core 1 Status | Edge          |               |
|            170 | IIR1_DMA  | IIR1 Core1DMA      | Edge          |               |
|            171 | IIR1_STAT | IIR1 Core 1 Status | Edge          |               |
|            172 | IIR2_DMA  | IIR2 Core1DMA      | Edge          |               |
|            173 | IIR2_STAT | IIR2 Core 1 Status | Edge          |               |
|            174 | IIR3_DMA  | IIR3 Core1DMA      | Edge          |               |
|            175 | IIR3_STAT | IIR3 Core 1 Status | Edge          |               |
|            178 | IIR4_DMA  | IIR4 Core2DMA      | Edge          |               |
|            179 | IIR4_STAT | IIR4 Core 2 Status | Edge          |               |
|            180 | IIR5_DMA  | IIR5 Core2DMA      | Edge          |               |
|            181 | IIR5_STAT | IIR5 Core 2 Status | Edge          |               |
|            182 | IIR6_DMA  | IIR6 Core2DMA      | Edge          |               |
|            183 | IIR6_STAT | IIR6 Core 2 Status | Edge          |               |
|            184 | IIR7_DMA  | IIR7 Core2DMA      | Edge          |               |

Table 45-2: ADSP-2159x\_SC592\_SC594 IIR Interrupt List (Continued)

|   Interrupt ID | Name         | Description               | Sensitivity   | DMA Channel   |
|----------------|--------------|---------------------------|---------------|---------------|
|            185 | IIR7_STAT    | IIR7 Core 2 Status        | Edge          |               |
|            252 | IIR0_BUS_ERR | IIR0 Core 1 IIR Bus Error | Edge          |               |
|            253 | IIR1_BUS_ERR | IIR1 Core 1 IIR Bus Error | Edge          |               |
|            254 | IIR2_BUS_ERR | IIR2 Core 1 IIR Bus Error | Edge          |               |
|            255 | IIR3_BUS_ERR | IIR3 Core 1 IIR Bus Error | Edge          |               |
|            257 | IIR4_BUS_ERR | IIR4 Core 2 IIR Bus Error | Edge          |               |
|            258 | IIR5_BUS_ERR | IIR5 Core 2 IIR Bus Error | Edge          |               |
|            259 | IIR6_BUS_ERR | IIR6 Core 2 IIR Bus Error | Edge          |               |
|            260 | IIR7_BUS_ERR | IIR7 Core 2 IIR Bus Error | Edge          |               |

## ADSP-2159x\_SC592\_SC594 IIR Trigger List

Table 45-3: ADSP-2159x\_SC592\_SC594 IIR Trigger List Generators

|   Trigger ID | Name        | Description   | Sensitivity   |
|--------------|-------------|---------------|---------------|
|           39 | C1_IIR0_DMA | IIR0 Core1DMA | Edge          |
|           40 | C1_IIR1_DMA | IIR1 Core1DMA | Edge          |
|           41 | C1_IIR2_DMA | IIR2 Core1DMA | Edge          |
|           42 | C1_IIR3_DMA | IIR3 Core1DMA | Edge          |
|           43 | C2_IIR0_DMA | IIR4 Core2DMA | Edge          |
|           44 | C2_IIR1_DMA | IIR5 Core2DMA | Edge          |
|           45 | C2_IIR2_DMA | IIR6 Core2DMA | Edge          |
|           46 | C2_IIR3_DMA | IIR7 Core2DMA | Edge          |

Table 45-4: ADSP-2159x\_SC592\_SC594 IIR Trigger List Receivers

|   Trigger ID | Name         | Description                           | Sensitivity   |
|--------------|--------------|---------------------------------------|---------------|
|           23 | C1_IIR0_TRGI | IIR0 Core 1 IIR Wait on Trigger Input | Pulse         |
|           24 | C1_IIR1_TRGI | IIR1 Core 1 IIR Wait on Trigger Input | Pulse         |
|           25 | C1_IIR2_TRGI | IIR2 Core 1 IIR Wait on Trigger Input | Pulse         |
|           26 | C1_IIR3_TRGI | IIR3 Core 1 IIR Wait on Trigger Input | Pulse         |
|           27 | C2_IIR0_TRGI | IIR4 Core 2 IIR Wait on Trigger Input | Pulse         |
|           28 | C2_IIR1_TRGI | IIR5 Core 2 IIR Wait on Trigger Input | Pulse         |

Table 45-4: ADSP-2159x\_SC592\_SC594 IIR Trigger List Receivers (Continued)

|   Trigger ID | Name         | Description                           | Sensitivity   |
|--------------|--------------|---------------------------------------|---------------|
|           29 | C2_IIR2_TRGI | IIR6 Core 2 IIR Wait on Trigger Input | Pulse         |
|           30 | C2_IIR3_TRGI | IIR7 Core 2 IIR Wait on Trigger Input | Pulse         |

## Multiply and Accumulate (MAC) Unit

The IIR MAC Unit figure shows a pipelined multiplier and accumulator unit that operates on the data and coefficients fetched from the data and coefficient memory. The MAC can perform 32 -bit floating-point or 40 -bit floating-point MAC operations. The 32 -bit floating-point operations generate 32 -bit results and the 40 -bit floatingpoint operations generate 40 -bit results.

Figure 45-3: IIR MAC Unit

<!-- image -->

## Input Data and Biquad State

The size of the data memory is 576 × 40 bits and locally holds the dk1 and dk2 state of all biquads. The DMA controller fetches the sample data from internal memory and calculates the output plus the dk1 and dk2 values for each biquad and stores them in local data memory.

## Coefficient Memory

The size of coefficient memory is 1440 × 40 bits and stores all the coefficients for all biquads. At startup, the DMA loads the coefficients from system memory into local coefficient memory.

## Internal Memory Storage

This section describes the required storage model for the IIR accelerator.

## Coefficient Memory Storage

Store the coefficients and Dk values for a specific biquad BQD[k] in internal memory in the following order: Ak0, Ak1, Bk1, Ak2, Bk2, Dk2, and Dk1.

NOTE: The naming convention for the filter coefficients used here is different from the one used in MATLAB. The following conversion should be used when using MATLAB generated coefficients:

(Akx = bx and Bkx = -ax).

Store the coefficients for each biquad in the order:

```
b0, b1, -a1, b2, -a2, dk2, dk1
```

For N biquad stages, store the coefficients in the order:

```
b01, b11, -a11, b21, -a21, dk21, dk11, b02, b12, -a12, b22, -a22, dk22, dk12, ,...... b0N, b1N, -a1N, b2N, -a2N, dk2N, dk1N.
```

where bxN and axN are the coefficients ([b, a]) for the Nth biquad stage.

## Operating Modes

The accelerator operates:

- In Window Processing Mode, that also includes sample-based processing mode
- In 40 -Bit Floating-Point Mode
- In Save Biquad State Mode
- In Auto Configuration Mode (ACM)

## Window Processing Mode

Select the sample-based processing mode by configuring window size to one. All biquads for a particular channel process one sample, and then calculate a final output sample in this mode.

In window-based mode, multiple output samples (maximum of 1024, equal to the window size) of that channel are calculated. After these calculations are complete, the accelerator begins processing the next channel. A register field provides a window size parameter to specify the length of the window.

## 40 -Bit Floating-Point Mode

In 40 -bit floating-point mode, the accelerator treats input data and coefficients as a 40 -bit floating-point numbers. The 40-bit floating-point MAC operations generate 40 -bit results. This mode can be selected by setting the IIR\_CTL1.FORTYBIT bit.

In ACM mode, 40 -bit floating-point mode can be selected by setting the IIR\_SGCTL.FORTYBIT bit field for each channel.

As the DMA bus width is 32 bits, in 40 -bit mode the IIR accelerator performs two packed 32 -bit accesses to the memory to:

- fetch one 40 -bit input or coefficient data

- store one 40 -bit output word

The first 32 -bit word provides the lower 32 bits and the 8 LSBs of the second 32-bit word provides rest of the upper 8 bits of the complete 40 -bit word. The 32 -Bit to 40 -Bit Packing figure shows the 32- to 40-bit packing used by the accelerator.

NOTE: Overhead time may be required to pack the input 40 -bit data into the format acceptable by the IIR accelerator and for unpacking the output of accelerator to a format acceptable to the rest of the application.

Figure 45-4: 32 -Bit to 40 -Bit Packing

<!-- image -->

## Save Biquad State Mode

The IIR\_CTL1.SS bit stores the current biquad states in local memory (writes all the Dk1 and Dk2 states back into the system memory states). This is useful for applications that require fast switching to another high-priority accelerator task-a required IIR to FIR processing transition for example. After resuming, these states can be reloaded and IIR processing can be continued. DMA status is automatically stored after each iteration.

NOTE: In legacy mode, the accelerator loads the biquad coefficients for all the channels before processing the first channel. After processing the last channel, when the IIR\_CTL1.SS bit is set, the accelerator stores the biquad states of all the channels in local memory (writes all the Dk1 and Dk2 states back into the system memory states). In ACM mode, the accelerator loads the biquad coefficients for only the current channel before starting to process the current channel. When the IIR\_SGCTL.SS bit is set, the accelerator stores the biquad states of the current channel in local memory before starting calculations for the next channel.

NOTE: During the save state operation, it is not permitted to have write access to any of the IIR accelerator registers that are loaded by chaining. Attempted writes to these registers can result in the blocking of IOP core reads until the save state operation completes.

## Auto Configuration Mode (ACM)

The accelerator can be operated in legacy mode or Auto Configuration Mode (ACM). ACM is configured by setting the IIR\_CTL1.ACM bit. The default functional mode is legacy mode. The accelerator must be disabled before changing the accelerator mode.

The ACM provides the following features:

- Halt and Queuing

The core can pause processing the current Transfer Control Block (TCB) chain by setting the IIR\_CTL1.HALT bit. The accelerator acknowledges the core by setting the IIR\_DMASTAT.HALT\_STAT

bit. The core can then submit or insert new TCBs. After the core acts, the accelerator resumes processing by clearing the IIR\_CTL1.HALT bit. Before halting the accelerator, when the initial TCB chain is processed, the accelerator goes to the idle state. T o do this, disable the accelerator and then enable it by toggling the IIR\_CTL1.EN . Clear the IIR\_CTL1.HALT bit to resume processing.

- No Channel Number Limitation

Unlike in legacy mode, there is no fixed channel number limitation, and the accelerator ignores the value programmed in the IIR\_CTL1.CH field. The application can queue an unlimited number of channels and TCBs dynamically, plus the accelerator keeps processing the TCBs until the chain pointer is null.

- Selective Interrupt

The core can enable and mask interrupt generation for each channel by setting the IIR\_CTL2.IMASK bit. When the bit is cleared, an interrupt generates after the channel completes.

- Selective Controller/Target T rigger

The core can enable/mask trigger generation by the accelerator after the end of processing of each channel using the IIR\_CTL2.TMASK bit. The accelerator can also wait for a trigger after loading the TCB and coefficients and before processing a channel for which the IIR\_CTL2.TWAIT bit is set.

This feature can also be enabled in legacy mode by setting the IIR\_CTL1.L\_TIMASK\_EN and IIR\_CTL1.L\_TWAIT\_EN bits. In legacy mode, when the IIR\_CTL1.L\_TIMASK\_EN and IIR\_CTL1.L\_TWAIT\_EN bits are set, then the IIR\_CTL2.TMASK and IIR\_CTL2.TWAIT bits can be used to enable/mask trigger generation and wait for a trigger for each channel respectively.

Additionally, use the IIR\_CTL1 register to change parameters such as rounding mode, 40-bit mode, and save state for each channel. The IIR\_SCTL1 and IIR\_SCTL2 registers can be used as general-purpose registers.

## Data Transfers

The IIR filter works exclusively through DMA.

## IIR Accelerator TCB

The location of the DMA parameters for the next sequence comes from the chain pointer register. This register points to the next set of DMA parameters stored in the system memory of the processor known as the Task Control Block (TCB). In chained DMA operations, the processor automatically initializes and then begins another DMA transfer once the current DMA transfer completes. Each new set of parameters is stored in a user-initialized memory buffer or TCB for a chosen peripheral.

## Chain Assignment

The structure of a TCB is conceptually the same as the structure of a linked list. Each TCB has several data values and a pointer to the next TCB. The chain pointer of a TCB can point to itself to re-run the same DMA continuously. The IIR accelerator reads each word of the TCB and loads it into the corresponding register. A TCB with a chain pointer register value of zero indicates the end of the chain (no further TCBs to load). The IIR accelerator supports

circular buffer chained DMA. The following tables show the required TCBs for chained DMA in Legacy Mode and ACM. TCB refers to the start address of the TCB array.

NOTE: The IIR accelerator DMA has two available TCB loading sequences:

- Loads five parameters for the coefficients ( IIR\_CTL2 , IIR\_COEFIDX , IIR\_COEFMOD , IIR\_COEFLEN , and IIR\_CHNPTR ).
- Loads 10 parameters for the data ( IIR\_CTL2 , IIR\_INIDX , IIR\_INMOD , IIR\_INLEN , IIR\_INBASE , IIR\_OUTIDX , IIR\_OUTMOD , IIR\_OUTLEN , IIR\_OUTBASE , and IIR\_CHNPTR ).

Initialize IIR\_CHNPTR to TCB+12 in legacy mode and TCB+15 in ACM.

Table 45-5: IIR TCBs for Chained DMA in Legacy Mode

| Address   | Register    |
|-----------|-------------|
| TCB       | IIR_CHNPTR  |
| TCB + 0x1 | IIR_COEFLEN |
| TCB + 0x2 | IIR_COEFMOD |
| TCB + 0x3 | IIR_COEFIDX |
| TCB + 0x4 | IIR_OUTBASE |
| TCB + 0x5 | IIR_OUTLEN  |
| TCB + 0x6 | IIR_OUTMOD  |
| TCB + 0x7 | IIR_OUTIDX  |
| TCB + 0x8 | IIR_INBASE  |
| TCB + 0x9 | IIR_INLEN   |
| TCB + 0xA | IIR_INMOD   |
| TCB + 0xB | IIR_INIDX   |
| TCB + 0xC | IIR_CTL2    |

Table 45-6: IIR TCBs for Chained DMA in ACM

| Address   | Register    |
|-----------|-------------|
| TCB       | IIR_CHNPTR  |
| TCB + 0x1 | IIR_SCTL1   |
| TCB + 0x2 | IIR_SCTL2   |
| TCB + 0x3 | IIR_SGCTL   |
| TCB + 0x4 | IIR_COEFLEN |
| TCB + 0x5 | IIR_COEFMOD |

Table 45-6: IIR TCBs for Chained DMA in ACM (Continued)

| Address   | Register    |
|-----------|-------------|
| TCB + 0x6 | IIR_COEFIDX |
| TCB + 0x7 | IIR_OUTBASE |
| TCB + 0x8 | IIR_OUTLEN  |
| TCB + 0x9 | IIR_OUTMOD  |
| TCB + 0xA | IIR_OUTIDX  |
| TCB + 0xB | IIR_INBASE  |
| TCB + 0xC | IIR_INLEN   |
| TCB + 0xD | IIR_INMOD   |
| TCB + 0xE | IIR_INIDX   |
| TCB + 0xF | IIR_CTL2    |

## DMA Access

The IIR accelerator has two DMA channels (accelerator input and output) to connect to the system memory. The DMA controller fetches the data and coefficients from memory and stores the result.

## Burst Mode Access

Burst mode enhances the throughput of the DMA channel and reduces the overall load on the system. When the IIR\_CTL1.BURSTEN bit is set, it loads all the biquads for all the channels with burst enabled. The IIR module supports only INCR8 burst.

## Chain Pointer DMA

The DMA controller supports a circular buffer chain pointer DMA. One transfer control block (TCB) must be configured for each channel. The TCB contains:

- A control register value to configure the filter parameters (such as number of biquads, window size) for each channel. For ACM, additional parameters such as an interrupt mask, trigger mask, and trigger wait exist.
- Software control register values in ACM for each channel.
- Secondary control register value in ACM to configure rounding mode, 40-bit mode, and save state for each channel.
- DMA parameter register values for the input data.
- DMA parameter register values for coefficient load.
- DMA parameter register values for output data.

In legacy mode, The chain pointer ( IIR\_CHNPTR ) field of the last channel TCB should point to the first channel TCB. This configuration exists so that when the IIR accelerator is enabled, the module:

1. Loads the coefficients (Ak, Bk) and state variables (Dk) for all the channels into the local coefficient memory of the IIR.
2. Loops back to the first channel again to start fetching the input data for processing.

In ACM, the chain pointer field of the last channel TCB need not point to the first channel TCB.

When the accelerator is enabled, the module:

1. Only loads the coefficients (Ak, Bk) and state variables (Dk) for the current channel being processed.
2. Starts fetching input data for processing.
3. After processing the current channel, when the IIR\_SGCTL.SS bit is set, it saves the biquad states of the current channel in local memory and moves to the next channel.

In ACM, the accelerator keeps processing the TCBs until the chain pointer becomes null.

The accelerator loads the TCB into its internal registers and uses these values to fetch coefficients, fetch data, and store results. After processing a window of data for any channel, the accelerator writes back the IIR\_INIDX (input index register) and IIR\_OUTIDX (output index register) values to the TCB in memory. Data processing can then begin from where it left off during the next time slot of that channel.

For 32-bit mode, the write-back values for the index registers are equal to IIRII + W and IIROI + W.

For 40-bit mode, the write-back values are: IIR\_INIDX + 2 × W and IIR\_OUTIDX + 2 × W.

Accelerator input and output channels connect to system memory.

NOTE: The IIR\_CTL2 register is part of the IIR TCB. This configuration allows software to program individual IIR channels having different control attributes.

In ACM, when the IIR\_CTL1.SMQ\_LIUPS\_EN bit is set, the accelerator updates the IIR\_INIDX and IIR\_OUTIDX bit fields of the TCB in memory after processing a window of data and according to the circular buffer scheme. When the IIR\_CTL1.SMQ\_LIUPS\_EN bit is cleared, the accelerator updates the IIR\_INIDX and IIR\_OUTIDX bit fields of the TCB in memory to 0x00000000 and 0xFFFFFFFF after processing a window of data. The IIR\_CTL1.SMQ\_LIUPS\_EN bit is only valid in ACM.

Figure 45-5: Circular Buffer Addressing

<!-- image -->

## Interrupts

The IIR Interrupt Overview table provides the source of interrupt and service instructions for the IIR interrupts.

Table 45-7: IIR Interrupt Overview

| Accelerator Mode        | Default Programmable Interrupt   | Sources                            | Masking        | Service                                |
|-------------------------|----------------------------------|------------------------------------|----------------|----------------------------------------|
| Legacy Mode             | IIR_DMA                          | Window Complete                    | N/A            | ROC from IIR_DMASTAT + RTI instruction |
| Legacy Mode             | IIR_DMA                          | All channels complete              | N/A            | ROC from IIR_DMASTAT + RTI instruction |
| Legacy Mode             | IIR_STAT                         | MAC IEEE floating point exceptions | N/A            | ROC from IIR_MACSTAT + RTI instruction |
| Legacy Mode             | IIR_STAT                         | MAC fixed point overflow           | N/A            | ROC from IIR_MACSTAT + RTI instruction |
| Auto Configuration Mode | IIR_DMA                          | Window Complete                    | IIR_CTL2.IMASK | ROC from IIR_DMASTAT + RTI instruction |
| Auto Configuration Mode | IIR_STAT                         | MAC IEEE floating point exceptions | N/A            | ROC from IIR_MACSTAT + RTI instruction |
| Auto Configuration Mode | IIR_STAT                         | MAC fixed point overflow           | N/A            | ROC from IIR_MACSTAT + RTI instruction |

## Sources

The IIR module drives two interrupt signals, IIR\_DMA for the DMA status and IIR\_STAT for the MAC status. The IIR module generates interrupts as described in the following sections.

## Window Complete

This interrupt generates at the end of each channel when all the output samples are calculated for a window and the channel writes back updated index values.

In legacy mode, when the IIR\_CTL1.CCINTR bit is set, an interrupt generates after completion of window processing of each channel. In ACM, the interrupt generation can be selectively masked using the IIR\_CTL2.IMASK bit for each channel.

Interrupt masking for each channel can be enabled in legacy mode by setting the IIR\_CTL1.L\_TIMASK\_EN bit. In legacy mode, when the IIR\_CTL1.L\_TIMASK\_EN bit is set, interrupt generation for each channel is controlled using the IIR\_CTL2.IMASK bit.

## All Channels Complete

This interrupt generates when all the channels are complete or when one iteration of time slots completes. The interrupt follows the access completion rule, where the interrupt generates when all data are written back to system memory.

The All Channels Complete interrupt source only applies in legacy mode. When the IIR\_CTL1.CCINTR and IIR\_CTL1.L\_TIMASK\_EN bits are not set in Legacy Mode, the all channel complete interrupt generates. In ACM, interrupt generation for each channel is controlled using the IIR\_CTL2.IMASK bit.

NOTE: In ACM, the Output DMA complete interrupt can be generated after the Save State completes for the channel by clearing the IIR\_SGCTL.SSESEL bit . When this bit is set, the Output DMA interrupt generates before the Save State completes for the channel.

In Legacy Mode, when the IIR\_CTL1.CCINTR and IIR\_CTL1.SSESEL bits are cleared, the All Channels Done Interrupt generates after the Save State operation completes for all the channels.

## MAC Status

A MAC status interrupt generates under the following conditions:

- Multiplier result zero-set when multiplier result is zero
- Multiplier result infinity-set when multiplier result is infinity
- Multiply invalid-set when multiply operation is invalid
- Adder result zero-set when adder result is zero
- Adder result infinity-set when adder result is infinity
- Adder invalid-set when addition is invalid

## Service

The DMA interrupt status bits are sticky and are cleared when the DMA status register is read. When a MAC status interrupt occurs, programs can find this state (and clear) by reading the MAC status register ( IIR\_MACSTAT ). The MAC interrupt status bits are sticky.

The status interrupt sources are derived from the IIR\_MACSTAT register. A status interrupt can occur due to the last set of MAC operations of a processing iteration that correspond to a particular channel. The interrupt is generated continuously and cannot be stopped, even after disabling the accelerator. The interrupt can only be stopped when another processing iteration results in a non-zero or valid multiply or add result.

## Programming Model

The following sections provide programming examples for legacy and auto configuration modes.

## Programming Legacy Mode and ACM

- Legacy Mode Programming
- Auto Configuration Mode Programming

## Legacy Mode Programming

The IIR accelerator supports up to 32 channels which are time division multiplexed (TDM). Each channel can have a maximum of 64 biquads. The window size for each channel is configurable using the control registers. The maximum configured window size is 1024 samples.

For programming multi-channel filtering in Legacy Mode:

1. Program the number of channels using the IIR\_CTL1.CH bits.
2. Configure the TCBs in system memory with one channel TCB pointing to the next channel TCB. The TCB of the last channel should point to the TCB of the first channel.
3. Write the first TCB value into the IIR\_CHNPTR register and enable the accelerator.
4. When the IIR\_CTL1.BURSTEN bit is set, ensure that the coefficient buffer end address plus seven words does not fall outside the valid address range and the buffer is initialized. When the burst feature is enabled, the IIR module issues an INCR8 burst. When the data count % 8== 7, the extra seven words are always read, but ignored by the IIR module.

NOTE: Data count = ∑ (Bn*7) or ∑ (Bn*14); where Bn is number of biquads for channel N. Each biquads contains 7 words for 32 -bit mode or 14 words in 40 -bit mode.

The Biquad Processing Program Flow figure shows the biquad processing program flow.

Figure 45-6: Biquad Processing Program Flow

<!-- image -->

The accelerator uses the following procedure to process biquads.

1. The controller loads all coefficients of all the channels into local storage.
2. Once all the coefficients are loaded, the controller goes to the first biquad of the first channel and calculates the output of the first biquad and updates the intermediate results for that biquad. When the IIR\_CTL1.L\_TWAIT\_EN bit is set and the IIR\_CTL2.TWAIT bit is set for the channel, the accelerator waits for the input trigger to start the computation.
3. The accelerator then moves to the next biquad of that channel and repeats the process until all the biquads for that channel are completed and the results are stored to memory.

4. The procedure is repeated for the next sample until processing completes for one window of the corresponding channel.
5. After one channel window is processed, the accelerator moves to the next channel and computes the results.

NOTE: All the addresses must be aligned on 32 -bit address boundaries and must not contain the lower two bits (assumed to be zeros).

## Auto Configuration Mode Programming

For ACM, there is no limit on the number of channels the accelerator can process. The accelerator keeps processing the TCBs until the chain pointer becomes null. Each channel can have a maximum of 64 biquads. The maximum configured window size is 1024 samples.

The Multi-channel Filtering in Auto Configuration Mode figure shows multi-channel filtering in ACM. Multiple channels are processed in a time division multiplexed (TDM) format.

Figure 45-7: Multi-channel Filtering in Auto Configuration Mode

<!-- image -->

To program and understand the multi-channel filtering in ACM:

1. Configure the TCBs in system memory with the one channel TCB pointing to the next channel TCB. There is no limit on the number of channels to be configured. The TCB of the last channel does not need to point to the TCB of the first channel.
2. Write the first TCB value to the IIR\_CHNPTR register and enable the accelerator.

The accelerator fetches the first channel TCB. Using it as pointer, loads the biquad coefficients and biquad states for that channel into the IIR local memory and loads the IIR\_CTL2 and IIR\_SGCTL registers to configure the filter parameters corresponding to that channel.

When the IIR\_CTL2.TWAIT bit is set, the accelerator waits for a input trigger to start the window processing for the channel.

The accelerator computes the output of the first biquad and updates the intermediate results for that biquad. Then, the accelerator moves to the next biquad of that channel and repeats the process until all the biquads for that channel are completed and the results are stored to memory. The process is repeated with the next sample until one window of the corresponding channel is processed. When the IIR\_CTL1.SS bit is set, the accelerator saves the states of the biquads of the channel into system memory. If the IIR\_CTL2.TMASK bit is cleared the accelerator sends an output trigger after completion of processing the channel.

When the IIR\_CTL2.IMASK bit is cleared, the accelerator interrupts the core after completion of processing a particular channel.

At the end of the window, the accelerator updates the IIR\_INIDX and IIR\_OUTIDX registers to 0x00000000 and 0xFFFFFFFF in the TCB of system memory and moves to the next channel.

3. At any instant, the core can halt the accelerator as needed. It sets the IIR\_CTL1.HALT bit and appropriately takes action to submit/insert new TCBs and clears the IIR\_CTL1.HALT bit to resume channel processing.

When the IIR\_CHNPTR register is zero (last channel is being processed or channel processing is complete) after halting the accelerator, the IIR\_CHNPTR register is written before resuming the accelerator channel processing.

When the accelerator is idle after halt, IIR\_CTL1.EN bit is toggled to disable and re-enable the accelerator and the IIR\_CTL1.HALT bit is cleared and the accelerator resumes channel processing.

4. The accelerator continues processing until all the channels are complete. Repeat Step 3 (as required) to submit or insert new channels.

## ADSP-2159x\_SC592\_SC594 IIR Register Descriptions

The IIR filter accelerator (IIR) contains the following registers.

Table 45-8: ADSP-2159x\_SC592\_SC594 IIR Register List

| Name         | Description                         |
|--------------|-------------------------------------|
| IIR_CHNPTR   | Chain Pointer Register              |
| IIR_COEFIDX  | Coefficient Buffer Index Register   |
| IIR_COEFLEN  | Coefficient Buffer Length Register  |
| IIR_COEFMOD  | Coefficient Index Modifier Register |
| IIR_CTL1     | Global Control Register             |
| IIR_CTL2     | Channel Control Register            |
| IIR_DBG_ADDR | IIR Debug Address Register          |
| IIR_DBG_CTL  | IIR Debug Control Register          |

Table 45-8: ADSP-2159x\_SC592\_SC594 IIR Register List (Continued)

| Name             | Description                             |
|------------------|-----------------------------------------|
| IIR_DBG_RDDAT_HI | IIR Debug Read Data High Register       |
| IIR_DBG_RDDAT_LO | IIR Debug Read Data Low Register        |
| IIR_DBG_WRDAT_HI | IIR Debug Write Data High Register      |
| IIR_DBG_WRDAT_LO | IIR Debug Write Data Low Register       |
| IIR_DMASTAT      | DMAStatus Register                      |
| IIR_INBASE       | Input Buffer Base Register              |
| IIR_INIDX        | Input Data Index Register               |
| IIR_INLEN        | Input Data Buffer Length Register       |
| IIR_INMOD        | Input Data Index Modifier Register      |
| IIR_MACSTAT      | MAC Status Register                     |
| IIR_OUTBASE      | Output Buffer Base Register             |
| IIR_OUTIDX       | Output Data Buffer Index Register       |
| IIR_OUTLEN       | IIR Output Data Buffer Length Register  |
| IIR_OUTMOD       | IIR Output Data Index Modifier Register |
| IIR_SCTL1        | Software Control Register1              |
| IIR_SCTL2        | Software Control Register2              |
| IIR_SGCTL        | Secondary Global Control Register       |

## Chain Pointer Register

The IIR\_CHNPTR register should be written with word address without the lower 2 bits.

Figure 45-8: IIR\_CHNPTR Register Diagram

<!-- image -->

Table 45-9: IIR\_CHNPTR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                            |
|--------------------|------------|--------------------------------------------------------------------|
| 29:0               | VALUE      | IIR Chain Pointer Address.                                         |
| (R/W)              |            | The IIR_CHNPTR.VALUE bit field contains the chain pointer address. |

## Coefficient Buffer Index Register

The IIR\_COEFIDX register contains the word address with the lower 2 bits removed.

Figure 45-9: IIR\_COEFIDX Register Diagram

<!-- image -->

Table 45-10: IIR\_COEFIDX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                |
|--------------------|------------|------------------------------------------------------------------------|
| 29:0               | VALUE      | Coefficient Buffer Index.                                              |
| (R/W)              |            | The IIR_COEFIDX.VALUE bit field provides the coefficient buffer index. |

## Coefficient Buffer Length Register

The IIR\_COEFLEN register provides the coefficient buffer length.

Figure 45-10: IIR\_COEFLEN Register Diagram

<!-- image -->

Table 45-11: IIR\_COEFLEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                 |
|--------------------|------------|-------------------------------------------------------------------------|
| 15:0               | VALUE      | Coefficient Length.                                                     |
| (R/W)              |            | The IIR_COEFLEN.VALUE bit field provides the coefficient buffer length. |

## Coefficient Index Modifier Register

The IIR\_COEFMOD register provides the coefficient index modifier.

Figure 45-11: IIR\_COEFMOD Register Diagram

<!-- image -->

Table 45-12: IIR\_COEFMOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                            |
|--------------------|------------|--------------------------------------------------------------------|
| 15:0               | VALUE      | Coefficient Modifier.                                              |
| (R/W)              |            | The IIR_COEFMOD.VALUE bit field provides the coefficient modifier. |

## Global Control Register

The IIR\_CTL1 register is used to configure the global parameters for the accelerator. These parameters include the number of channels, channel auto iterate, DMA enable, and accelerator enable.

Figure 45-12: IIR\_CTL1 Register Diagram

<!-- image -->

Table 45-13: IIR\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | ACM        | Auto Configuration Mode. The IIR_CTL1.ACM bit configures the mode for loading the TCB. 0 Legacy Mode                                                                                                                                                                                                                    |
| 30 (R/W)           | HALT       | Pause accelerator. The IIR_CTL1.HALT bit determines whether the accelerator pauses so that the core can check the status and modify or submit a job or the accelerator is released for fur- ther processing of data. This bit is only valid in Auto Configuration Mode (ACM). 0 Release accelerator 1 Pause accelerator |

Table 45-13: IIR\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                               |
|--------------------|--------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W)           | SSESEL       | Write Back Interrupt Select. The IIR_CTL1.SSESEL bit selects whether to generate an interrupt after the write back or after save state completion.                                                                    |
| 29 (R/W)           | SSESEL       | 0 Generate Interrupt after save state completion                                                                                                                                                                      |
| 29 (R/W)           | SSESEL       | 1 Generate interrupt after write back completion                                                                                                                                                                      |
| 28 (R/W)           | L_TWAIT_EN   | Enable TWAIT Feature in Legacy Mode. The IIR_CTL1.L_TWAIT_EN bit indicates whether the TWAIT feature is availa- ble in legacy mode. This bit is only valid in legacy mode.                                            |
| 28 (R/W)           | L_TWAIT_EN   | 0 Not Available                                                                                                                                                                                                       |
| 28 (R/W)           | L_TWAIT_EN   | 1 Available                                                                                                                                                                                                           |
| 27 (R/W)           | L_TIMASK_EN  | Enable Trigger and Interrupt Masking in Legacy Mode. The IIR_CTL1.L_TIMASK_EN bit enables trigger and interrupt masking in legacy mode. This bit is only valid in legacy mode.                                        |
| 27 (R/W)           | L_TIMASK_EN  | 0 Disable                                                                                                                                                                                                             |
| 27 (R/W)           | L_TIMASK_EN  | 1 Enable                                                                                                                                                                                                              |
| 26 (R/W)           | SMQ_LIUPS_EN | Legacy I/P and O/P Index Update Scheme in SMART_Q Mode. The IIR_CTL1.SMQ_LIUPS_EN bit configures the scheme the accelerator uses to update the input index (II) and output index (OI). This bit is only valid in ACM. |
| 26 (R/W)           | SMQ_LIUPS_EN | 0 Update the II with all '0' and the OI with all 'F', respec- tively                                                                                                                                                  |
| 26 (R/W)           | SMQ_LIUPS_EN | 1 Update II and OI after circular buffer scheme                                                                                                                                                                       |
| 17 (R/W)           | BURSTEN      | Burst Mode Enable. The IIR_CTL1.BURSTEN bit field is set, burst access is enabled.                                                                                                                                    |
| 17 (R/W)           | BURSTEN      | 0 Disable burst mode                                                                                                                                                                                                  |
| 17 (R/W)           | BURSTEN      | 1 Enable burst mode                                                                                                                                                                                                   |
| 14 (R/W)           | RND          | Rounding Mode. The IIR_CTL1.RND bit field selects the rounding mode for floating-point format.                                                                                                                        |
| 14 (R/W)           | RND          | 0 Round to nearest (even)                                                                                                                                                                                             |
| 14 (R/W)           | RND          | 1 Truncate (Round towards zero)                                                                                                                                                                                       |

Table 45-13: IIR\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                        | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | FORTYBIT   | 40-Bit Floating-Point Boundary Select. The IIR_CTL1.FORTYBIT bit selects between 32-bit IEEE floating-point format or 40-bit IEEE floating-point format.                                                                                                                                                                                                                                       | 40-Bit Floating-Point Boundary Select. The IIR_CTL1.FORTYBIT bit selects between 32-bit IEEE floating-point format or 40-bit IEEE floating-point format.                                                                                                                                                                                                                                       |
| 12 (R/W)           | FORTYBIT   | 0                                                                                                                                                                                                                                                                                                                                                                                              | 32-bit IEEE floating-point                                                                                                                                                                                                                                                                                                                                                                     |
| 12 (R/W)           | FORTYBIT   | 1                                                                                                                                                                                                                                                                                                                                                                                              | 40-bit IEEE floating-point                                                                                                                                                                                                                                                                                                                                                                     |
| 11 (R/W)           | CCINTR     | Channel Complete Interrupt. The IIR_CTL1.CCINTR bit configures the channel complete interrupt to generate when all channels are done or after each channel is done. This bit is only valid in lega- cy mode when the IIR_CTL1.L_TIMASK_EN bit is cleared (=0). In Auto Configura- tion Mode (ACM) or when the IIR_CTL1.L_TIMASK_EN is set (=1), interrupt gen-                                 | Channel Complete Interrupt. The IIR_CTL1.CCINTR bit configures the channel complete interrupt to generate when all channels are done or after each channel is done. This bit is only valid in lega- cy mode when the IIR_CTL1.L_TIMASK_EN bit is cleared (=0). In Auto Configura- tion Mode (ACM) or when the IIR_CTL1.L_TIMASK_EN is set (=1), interrupt gen-                                 |
| 11 (R/W)           | CCINTR     | 0                                                                                                                                                                                                                                                                                                                                                                                              | Interrupt is generated only when all channels are done (default)                                                                                                                                                                                                                                                                                                                               |
| 11 (R/W)           | CCINTR     | 1                                                                                                                                                                                                                                                                                                                                                                                              | Interrupt is generated after each channel is done (de- fault)                                                                                                                                                                                                                                                                                                                                  |
| 10 (R/W)           | SS         | Save Biquad State. The IIR_CTL1.SS bit configures the accelerator to store the Dk register settings in- to the internal memory. This can be used to save the biquad states before switching to another high priority accelerator task. This bit is only valid in legacy mode. In Auto Configuration Mode (ACM), the save state for each channel is controlled using the                        | Save Biquad State. The IIR_CTL1.SS bit configures the accelerator to store the Dk register settings in- to the internal memory. This can be used to save the biquad states before switching to another high priority accelerator task. This bit is only valid in legacy mode. In Auto Configuration Mode (ACM), the save state for each channel is controlled using the                        |
| 9 (R/W)            | CAI        | Channel Auto Iterate. The IIR_CTL1.CAI bit sets whether TDMprocessing stops (idle) once all channels complete processing or moves to first channel and continues TDMprocessing in a loop when all channels complete processing. Channel Auto Iterate is not available in Auto Configuration Mode (ACM). The accelerator keeps processing the TCBs until the chain pointer becomes null in ACM. | Channel Auto Iterate. The IIR_CTL1.CAI bit sets whether TDMprocessing stops (idle) once all channels complete processing or moves to first channel and continues TDMprocessing in a loop when all channels complete processing. Channel Auto Iterate is not available in Auto Configuration Mode (ACM). The accelerator keeps processing the TCBs until the chain pointer becomes null in ACM. |
| 9 (R/W)            | CAI        | 0                                                                                                                                                                                                                                                                                                                                                                                              | TDMprocessing stops (idle) once all channels complete processing                                                                                                                                                                                                                                                                                                                               |
| 9 (R/W)            | CAI        | 1                                                                                                                                                                                                                                                                                                                                                                                              | Moves to first channel and continues TDMprocessing in a loop when all channels complete processing                                                                                                                                                                                                                                                                                             |
| 8                  | DMAEN      | DMAEnable.                                                                                                                                                                                                                                                                                                                                                                                     | DMAEnable.                                                                                                                                                                                                                                                                                                                                                                                     |
| (R/W)              |            | The IIR_CTL1.DMAEN bit enables DMAon the accelerator.                                                                                                                                                                                                                                                                                                                                          | The IIR_CTL1.DMAEN bit enables DMAon the accelerator.                                                                                                                                                                                                                                                                                                                                          |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                              | Disable                                                                                                                                                                                                                                                                                                                                                                                        |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                              | Enable                                                                                                                                                                                                                                                                                                                                                                                         |

Table 45-13: IIR\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5:1 (R/W)          | CH         | Number of Channels. The IIR_CTL1.CH bit field configures the number of channels and is programma- ble between 0-31 (channels = NCH+1). This bit field is only valid in legacy mode. In Auto Configuration Mode (ACM), there is no limit on the number of channels. the accelerator keeps processing the TCBs until the chain pointer becomes null. |
| 0 (R/W)            | EN         | IIR Enable. The IIR_CTL1.EN bit enables or disables the IIR accelerator.                                                                                                                                                                                                                                                                           |
| 0 (R/W)            | EN         | 0 IIR disabled                                                                                                                                                                                                                                                                                                                                     |
| 0 (R/W)            | EN         | 1 IIR enabled                                                                                                                                                                                                                                                                                                                                      |

## Channel Control Register

The IIR\_CTL2 register is used to configure the channel specific parameters. These parameters include the number of biquads and window size. In Auto Configuration Mode (ACM), this register is also used to configure additional channel specific parameters like interrupts and triggers.

Figure 45-13: IIR\_CTL2 Register Diagram

<!-- image -->

Table 45-14: IIR\_CTL2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | TMASK      | Trigger mask bit. The IIR_CTL2.TMASK bit enables trigger generation for the channel. It is valid in Auto Configuration Mode (ACM) or if CTL1.L_TIMASK_EN is set.                |
| 28 (R/W)           | TWAIT      | Wait for trigger. The IIR_CTL2.TWAIT bit disables the wait for the trigger. It is valid in Auto Con- figuration Mode (ACM) or if CTL1.L_TIMASK_EN is set.                       |
| 24 (R/W)           | IMASK      | Interrupt Mask. The IIR_CTL2.IMASK bit enables interrupt generation for the channel. This bit is valid in Auto Configuration Mode (ACM) or if CTL1.L_TIMASK_EN is set. 0 Enable |

Table 45-14: IIR\_CTL2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:14 (R/W)        | WINDOW     | Window Size Parameter. The IIR_CTL2.WINDOW bit field configures the window size which specifies the number of samples/blocks to process (sample based processing = window size of 1). This bit field should be programmed to "actual window size required -1". For example, for sample based processing this bit field should be programmed to 0. |
| 13:12 (R/W)        | PRIO       | Priority Level. The IIR_CTL2.PRIO bit field indicates the priority.                                                                                                                                                                                                                                                                               |
| 13:12 (R/W)        | PRIO       | 0 Level 0 (lowest)                                                                                                                                                                                                                                                                                                                                |
| 13:12 (R/W)        | PRIO       | 1 Level 1                                                                                                                                                                                                                                                                                                                                         |
| 13:12 (R/W)        | PRIO       | 2 Level 2                                                                                                                                                                                                                                                                                                                                         |
| 13:12 (R/W)        | PRIO       | 3 Level 3 (highest)                                                                                                                                                                                                                                                                                                                               |
| 5:0 (R/W)          | BIQUADS    | Number of Biquads. The IIR_CTL2.BIQUADS bit field configures the number of biquads and is pro- grammable between 0-63 (number of Biquads = BIQUADS + 1).                                                                                                                                                                                          |

## IIR Debug Address Register

The IIR\_DBG\_ADDR register holds the debug address. If bit 11 is set, coefficient memory is selected.

Figure 45-14: IIR\_DBG\_ADDR Register Diagram

<!-- image -->

Table 45-15: IIR\_DBG\_ADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10:0 (R/W)         | VALUE      | Debug Address, Coefficient Memory Select. The IIR_DBG_ADDR.VALUE bit field holds the debug address (bits 0-10). Bit 11 configures whether the memory access is to coefficient memory (=0) or to delay line memory (=1). |

## IIR Debug Control Register

Figure 45-15: IIR\_DBG\_CTL Register Diagram

<!-- image -->

Table 45-16: IIR\_DBG\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                | Description/Enumeration                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W1S)          | ADRINC     | Address Auto Increment. The IIR_DBG_CTL.ADRINC bit allows the address register to auto increment on IIR_DBG_WRDAT_HI / IIR_DBG_WRDAT_LO writes and IIR_DBG_RDDAT_HI / IIR_DBG_RDDAT_LO reads.          | Address Auto Increment. The IIR_DBG_CTL.ADRINC bit allows the address register to auto increment on IIR_DBG_WRDAT_HI / IIR_DBG_WRDAT_LO writes and IIR_DBG_RDDAT_HI / IIR_DBG_RDDAT_LO reads.          |
| 4 (R/W1S)          | MEM        | Local Memory Access. The IIR_DBG_CTL.MEM bit allows the data and coefficients memory to be indirect- ly accessed.                                                                                      | Local Memory Access. The IIR_DBG_CTL.MEM bit allows the data and coefficients memory to be indirect- ly accessed.                                                                                      |
| 2 (R/W1S)          | RUN        | Release the MAC. The IIR_DBG_CTL.RUN bit releases the MAC; it is is self clearing after one IIR clock cycle.                                                                                           | Release the MAC. The IIR_DBG_CTL.RUN bit releases the MAC; it is is self clearing after one IIR clock cycle.                                                                                           |
| 1 (R/W)            | HLD        | Hold or Single Step. The IIR_DBG_CTL.HLD bit function is based on the IIR_DBG_CTL.MEM bit setting. For IIR_DBG_CTL.MEM = 0 this bit sets single step. For IIR_DBG_CTL.MEM = 1 this bit sets hold data. | Hold or Single Step. The IIR_DBG_CTL.HLD bit function is based on the IIR_DBG_CTL.MEM bit setting. For IIR_DBG_CTL.MEM = 0 this bit sets single step. For IIR_DBG_CTL.MEM = 1 this bit sets hold data. |
| 1 (R/W)            | HLD        | 0                                                                                                                                                                                                      | No effect                                                                                                                                                                                              |
| 1 (R/W)            | HLD        | 1                                                                                                                                                                                                      | Single step (IIR_DBGMEM=0) or Hold data (IIR_DBGMEM=1)                                                                                                                                                 |
| 0 (R/W)            | EN         | Debug Mode Enable. The IIR_DBG_CTL.EN bit enables debug mode. For local memory access, the IIR_CTL1 register can be cleared.                                                                           | Debug Mode Enable. The IIR_DBG_CTL.EN bit enables debug mode. For local memory access, the IIR_CTL1 register can be cleared.                                                                           |
| 0 (R/W)            | EN         | 0                                                                                                                                                                                                      | Disable                                                                                                                                                                                                |
| 0 (R/W)            | EN         | 1                                                                                                                                                                                                      | Enable                                                                                                                                                                                                 |

## IIR Debug Read Data High Register

The IIR\_DBG\_RDDAT\_HI register is part of the 40-bit wide debug mode read data register and holds the upper 8 bits.

Figure 45-16: IIR\_DBG\_RDDAT\_HI Register Diagram

<!-- image -->

Table 45-17: IIR\_DBG\_RDDAT\_HI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                               |
|--------------------|------------|-----------------------------------------------------------------------|
| 7:0                | VALUE      | Debug Read Data Highest 8 Bits.                                       |
| (R/NW)             |            | The IIR_DBG_RDDAT_HI.VALUE bit field holds the upper 8-bit read data. |

## IIR Debug Read Data Low Register

The IIR\_DBG\_RDDAT\_LO register is part of the 40-bit wide debug mode read data register and holds the lower 32 bits.

Figure 45-17: IIR\_DBG\_RDDAT\_LO Register Diagram

<!-- image -->

Table 45-18: IIR\_DBG\_RDDAT\_LO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                |
|--------------------|------------|------------------------------------------------------------------------|
| 31:0               | VALUE      | Debug Read Data Lower 32 Bits.                                         |
| (R/NW)             |            | The IIR_DBG_RDDAT_LO.VALUE bit field holds the lower 32-bit read data. |

## IIR Debug Write Data High Register

The IIR\_DBG\_WRDAT\_HI register is part of the 40-bit wide debug mode write data register and holds the upper 8 bits.

Figure 45-18: IIR\_DBG\_WRDAT\_HI Register Diagram

<!-- image -->

Table 45-19: IIR\_DBG\_WRDAT\_HI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                |
|--------------------|------------|------------------------------------------------------------------------|
| 7:0                | VALUE      | Debug Write Data Highest 8 Bits.                                       |
| (R/W)              |            | The IIR_DBG_WRDAT_HI.VALUE bit field holds the upper 8-bit write data. |

## IIR Debug Write Data Low Register

The IIR\_DBG\_WRDAT\_LO register is part of the 40-bit wide debug mode write data register and holds the lower 32 bits.

Figure 45-19: IIR\_DBG\_WRDAT\_LO Register Diagram

<!-- image -->

Table 45-20: IIR\_DBG\_WRDAT\_LO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                 |
|--------------------|------------|-------------------------------------------------------------------------|
| 31:0               | VALUE      | Debug Write Data Lower 32 Bits.                                         |
| (R/W)              |            | The IIR_DBG_WRDAT_LO.VALUE bit field holds the lower 32-bit write data. |

## DMA Status Register

The IIR\_DMASTAT registers indicate the status of DMA operations.

Figure 45-20: IIR\_DMASTAT Register Diagram

<!-- image -->

Table 45-21: IIR\_DMASTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/NW)          | HALT_STAT  | Accelerator HALT status. Acknowledge to core that accelerator is in halt state, if set.                                                                                                                                             |
| 11:7 (RC/NW)       | CURCHNL    | Current Channel. The IIR_DMASTAT.CURCHNL bit field indicates the channel that is being process- ed in the TDMslot. Zero indicates the last slot.                                                                                    |
| 6 (RC/NW)          | ACDONE     | All Channels Done. The IIR_DMASTAT.ACDONE bit indicates all channels are done processing. Note that the IIR_CTL1.CCINTR bit does not affect this status bit. This bit is sticky and is cleared on register read.                    |
| 5 (R/NW)           | WDONE      | Current Channel Done. The IIR_DMASTAT.WDONE bit indicates the processing of the current channel is complete. Note that the IIR_CTL1.CCINTR bit does not affect this status bit. This bit is sticky and is cleared on register read. |
| 4 (R/NW)           | SVDK       | Save Updated Dk State. If there is more than one channel ( IIR_CTL1.CH >0), the IIR_DMASTAT.SVDK bit toggles between 0 and 1 as it starts and completes the save state operation on one channel at a time.                          |

Table 45-21: IIR\_DMASTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/NW)           | WRBK       | Write Back. The IIR_DMASTAT.WRBK bit indicates the accelerator is writing back updated in- dex registers.                                                    |
| 2 (R/NW)           | PPGS       | MAC Processing in Progress. The IIR_DMASTAT.PPGS bit indicates MAC processing is in progress.                                                                |
| 1 (R/NW)           | CDKLD      | Coefficient and Dk Loading. The IIR_DMASTAT.CDKLD bit indicates the coefficient and Dk are loading.                                                          |
| 0 (R/NW)           | CPLD       | Chain Pointer Loading Status. The IIR_DMASTAT.CPLD bit indicates the IIR is in the chain pointer load state. 0 State machine not in chain pointer load state |
| 0 (R/NW)           | CPLD       | 1 State machine in chain pointer load state                                                                                                                  |
| 0 (R/NW)           | CPLD       |                                                                                                                                                              |

## Input Buffer Base Register

The IIR\_INBASE register contains the word address with the lower 2 bits removed.

Figure 45-21: IIR\_INBASE Register Diagram

<!-- image -->

Table 45-22: IIR\_INBASE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                     |
|--------------------|------------|-----------------------------------------------------------------------------|
| 29:0               | VALUE      | Input Data Buffer Base.                                                     |
| (R/W)              |            | The IIR_INBASE.VALUE bit field value is the input data buffer base address. |

## Input Data Index Register

The IIR\_INIDX register contains a word address with the lower 2 bits removed.

Figure 45-22: IIR\_INIDX Register Diagram

<!-- image -->

Table 45-23: IIR\_INIDX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                             |
|--------------------|------------|---------------------------------------------------------------------|
| 29:0               | VALUE      | Input Data Buffer Index.                                            |
| (R/W)              |            | The IIR_INIDX.VALUE bit field value is the input data buffer index. |

## Input Data Buffer Length Register

The IIR\_INLEN register provides the input data buffer length.

Figure 45-23: IIR\_INLEN Register Diagram

<!-- image -->

Table 45-24: IIR\_INLEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                              |
|--------------------|------------|----------------------------------------------------------------------|
| 15:0               | VALUE      | Input Data Buffer Length.                                            |
| (R/W)              |            | The IIR_INLEN.VALUE bit field value is the input data buffer length. |

## Input Data Index Modifier Register

The IIR\_INMOD register provides the 16-bit input data buffer index modifier.

Figure 45-24: IIR\_INMOD Register Diagram

<!-- image -->

Table 45-25: IIR\_INMOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                       |
|--------------------|------------|-------------------------------------------------------------------------------|
| 15:0               | VALUE      | 16-bit Input Data Buffer Index Modifier.                                      |
| (R/W)              |            | The IIR_INMOD.VALUE bit field value is the 16-bit input data buffer modifier. |

## MAC Status Register

The IIR\_MACSTAT register indicates the status of MAC operations.

Figure 45-25: IIR\_MACSTAT Register Diagram

<!-- image -->

Table 45-26: IIR\_MACSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------|
| 5 (R/NW)           | AINV       | Addition Invalid. The IIR_MACSTAT.AINV bit indicates the addition is invalid.                    |
| 4 (R/NW)           | ARI        | Adder Result Infinity. The IIR_MACSTAT.ARI bit indicates the adder result is infinity.           |
| 3 (R/NW)           | ARZ        | Adder Result Zero. The IIR_MACSTAT.ARZ bit indicates the adder result is zero.                   |
| 2 (R/NW)           | MINV       | Multiply Invalid. The IIR_MACSTAT.MINV bit indicates the multiply operation is invalid.          |
| 1 (R/NW)           | MRI        | Multiplier Result Infinity. The IIR_MACSTAT.MRI bit indicates the multiplier result is infinity. |
| 0 (R/NW)           | MRZ        | Multiplier Result Zero. The IIR_MACSTAT.MRZ bit indicates the multiplier result is zero.         |

## Output Buffer Base Register

The IIR\_OUTBASE register contains the word address with the lower 2 bits removed.

Figure 45-26: IIR\_OUTBASE Register Diagram

<!-- image -->

Table 45-27: IIR\_OUTBASE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                  |
|--------------------|------------|--------------------------------------------------------------------------|
| 29:0               | VALUE      | Output Buffer Base.                                                      |
| (R/W)              |            | The IIR_OUTBASE.VALUE bit field provides the output buffer base address. |

## Output Data Buffer Index Register

The IIR\_OUTIDX register is written with word address without the lower 2 bits.

Figure 45-27: IIR\_OUTIDX Register Diagram

<!-- image -->

Table 45-28: IIR\_OUTIDX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                               |
|--------------------|------------|-----------------------------------------------------------------------|
| 29:0               | VALUE      | Output Data Buffer Index.                                             |
| (R/W)              |            | The IIR_OUTIDX.VALUE bit field provides the output data buffer index. |

## IIR Output Data Buffer Length Register

The IIR\_OUTLEN register provides the output data buffer length.

Figure 45-28: IIR\_OUTLEN Register Diagram

<!-- image -->

Table 45-29: IIR\_OUTLEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                |
|--------------------|------------|------------------------------------------------------------------------|
| 15:0               | VALUE      | 16-bit Output Data Buffer Length.                                      |
| (R/W)              |            | The IIR_OUTLEN.VALUE bit field provides the output data buffer length. |

## IIR Output Data Index Modifier Register

The IIR\_OUTMOD register provides the output data index modifier.

Figure 45-29: IIR\_OUTMOD Register Diagram

<!-- image -->

Table 45-30: IIR\_OUTMOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                        |
|--------------------|------------|--------------------------------------------------------------------------------|
| 15:0               | VALUE      | 16-bit Input Data Buffer Index Modifier.                                       |
| (R/W)              |            | The IIR_OUTMOD.VALUE bit field provides the output data buffer index modifier. |

## Software Control Register1

The IIR\_SCTL1 register is used by Smart Queuing APIs in ACM mode to exchange relevant job information amongst APIs.

Figure 45-30: IIR\_SCTL1 Register Diagram

<!-- image -->

Table 45-31: IIR\_SCTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration      |
|--------------------|------------|------------------------------|
| 31:0               | VALUE      | Software Control Register 1. |
| (R/W)              |            |                              |

## Software Control Register2

The IIR\_SCTL2 register is used by Smart Queuing APIs in ACM mode to exchange relevant job information amongst APIs.

Figure 45-31: IIR\_SCTL2 Register Diagram

<!-- image -->

Table 45-32: IIR\_SCTL2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration     |
|--------------------|------------|-----------------------------|
| 31:0               | VALUE      | Software Control Register2. |
| (R/W)              |            |                             |

## Secondary Global Control Register

The IIR\_SGCTL register configures the global parameters for the accelerator in ACM mode for loading CTL1 register as part of TCB.

Figure 45-32: IIR\_SGCTL Register Diagram

<!-- image -->

Table 45-33: IIR\_SGCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W)           | SSESEL     | Select Between Write Back or Save State Completion Interrupt.                                                                                                                                                                         |
| 14 (R/W)           | RND        | Rounding Mode Select. The IIR_SGCTL.RND bit selects the rounding mode for a floating-point mode compatible with SHARC+.                                                                                                               |
| 12 (R/W)           | FORTYBIT   | 40-Bit Floating-Point Format Select. The IIR_SGCTL.FORTYBIT bit selects the floating point format. 0 32-bit floating point                                                                                                            |
| 10 (R/W)           | SS         | Save Biquad State. The IIR_SGCTL.SS bit configures the accelerator to store the Dk register settings into the internal memory. This can be used to save the biquad states before switching to another high priority accelerator task. |