## 44   IIR Accelerator (IIR)

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
- Supports real-time coefficient slewing using a dedicated hardware slewing engine

NOTE: The IIR accelerator module has local memory that is not accessible by the core during regular operation mode. Unlike in the previous SHARC processors, the IIR accelerator modules each have access to the system memory (on-chip/off-chip).

Unlike in the previous SHARC processors, where only one of the IIR or FIR accelerators can be enabled at a time, the processor can simultaneously use both the IIR and the FIR accelerators.

## IIR Definitions

The following definitions are helpful when using the IIR filter and calculating slewing coefficients.

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

The mode where the IIR filter stores the current biquad states in local memory when it must switch to another high-priority accelerator task. After resuming, these states can be reloaded, and IIR processing continued.

## Slewing

Computation of IIR filter coefficients in real time and in parallel with IIR filtering. The slewing engine calculates coefficients using a slew rate, slew frequency (number of samples), and slewing time (number of iterations).

## Slew Rate ( ɑ )

A 32-bit floating-point number (0 ≤ ɑ ≤ 1) that indicates the slewing process rate. Alpha ( ɑ ) values approximating zero indicates a rapidly changing slew process. Values approximating one indicate a slow changing slew.

## Slew Frequency (n)

Determines the number of samples (n+1) that pass before the IIR coefficients are slewed.

## Slew Time (N)

Determines the number of iterations of coefficient slewing performed by the slewing engine.

## Slew Mode

To achieve multiple target values for each coefficient, change the slew rate (either constant or linear) or set the slewing parameters for each channel.

## Clocking

The IIR accelerator runs at the speed of core clock.

## Functional Description

The IIR Accelerator Block Diagram shows the blocks of the IIR hardware accelerator.

The accelerator has:

- a coefficient memory size of 1440 × 40 bits (288 biquads x 5 coefficients).
- a data memory size of 576 × 40 bits (288 biquads x 2 states).
- one SHARC+ core compatible MAC unit with an input data buffer to supply data to the MAC.

Figure 44-1: IIR Accelerator Block Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000000_d274df07912200eb2d653374d2aab8f70dbe4aba1c10d0a797fdcd7d5156ccd4.png)

A transposed direct form II biquadratic is used to implement the IIR accelerator, which reduces coefficient sensitivity. The Transposed Direct Form II Biquad figure shows the signal flow graph for the biquad structure.

Figure 44-2: Transposed Direct Form II Biquad

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000001_f896bb4e057080fa2cb4f408b2567578aea25bb9e9d9b7473be737bd0b47b18d.png)

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

## ADSP-2184x IIR Register List

The IIR module reduces the processing load on the core. For more information on IIR functionality, see the IIR register descriptions.

Table 44-1: ADSP-2184x IIR Register List

| Name             | Description                             |
|------------------|-----------------------------------------|
| IIR_ALPHA[n]     | Channel n Alpha                         |
| IIR_CHNPTR       | Chain Pointer Register                  |
| IIR_COEFIDX      | Coefficient Buffer Index Register       |
| IIR_COEFLEN      | Coefficient Buffer Length Register      |
| IIR_COEFMOD      | Coefficient Index Modifier Register     |
| IIR_CTL1         | Global Control Register                 |
| IIR_CTL2         | Channel Control Register                |
| IIR_DBG_ADDR     | IIR Debug Address Register              |
| IIR_DBG_CTL      | IIR Debug Control Register              |
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

Table 44-1: ADSP-2184x IIR Register List (Continued)

| Name             | Description                              |
|------------------|------------------------------------------|
| IIR_SCTL1        | Software Control Register1               |
| IIR_SCTL2        | Software Control Register2               |
| IIR_SGCTL        | Secondary Global Control Register        |
| IIR_SLEW_CNFG[n] | Channel 0 Slewing Duration and Frequency |

## ADSP-2184x IIR Interrupt List

Table 44-2: ADSP-2184x IIR Interrupt List

|   Interrupt ID | Name         | Description               | Sensitivity   | DMA Channel   |
|----------------|--------------|---------------------------|---------------|---------------|
|            131 | IIR0_DMA     | IIR0 Core1DMA             | Edge          |               |
|            132 | IIR0_STAT    | IIR0 Core 1 Status        | Edge          |               |
|            133 | IIR1_DMA     | IIR1 Core1DMA             | Edge          |               |
|            134 | IIR1_STAT    | IIR1 Core 1 Status        | Edge          |               |
|            135 | IIR2_DMA     | IIR2 Core1DMA             | Edge          |               |
|            136 | IIR2_STAT    | IIR2 Core 1 Status        | Edge          |               |
|            137 | IIR3_DMA     | IIR3 Core1DMA             | Edge          |               |
|            138 | IIR3_STAT    | IIR3 Core 1 Status        | Edge          |               |
|            139 | IIR0_BUS_ERR | IIR0 Core 1 IIR Bus Error | Edge          |               |
|            140 | IIR1_BUS_ERR | IIR1 Core 1 IIR Bus Error | Edge          |               |
|            141 | IIR2_BUS_ERR | IIR2 Core 1 IIR Bus Error | Edge          |               |
|            142 | IIR3_BUS_ERR | IIR3 Core 1 IIR Bus Error | Edge          |               |

## ADSP-2184x IIR Trigger List

Table 44-3: ADSP-2184x IIR Trigger List Generators

|   Trigger ID | Name     | Description   | Sensitivity   |
|--------------|----------|---------------|---------------|
|           60 | IIR0_DMA | IIR0 Core1DMA | Edge          |
|           61 | IIR1_DMA | IIR1 Core1DMA | Edge          |
|           62 | IIR2_DMA | IIR2 Core1DMA | Edge          |
|           63 | IIR3_DMA | IIR3 Core1DMA | Edge          |

Table 44-4: ADSP-2184x IIR Trigger List Receivers

|   Trigger ID | Name      | Description                           | Sensitivity   |
|--------------|-----------|---------------------------------------|---------------|
|          102 | IIR0_TRGI | IIR0 Core 1 IIR Wait on Trigger Input | Pulse         |
|          103 | IIR1_TRGI | IIR1 Core 1 IIR Wait on Trigger Input | Pulse         |
|          104 | IIR2_TRGI | IIR2 Core 1 IIR Wait on Trigger Input | Pulse         |
|          105 | IIR3_TRGI | IIR3 Core 1 IIR Wait on Trigger Input | Pulse         |

## Multiply and Accumulate (MAC) Unit

The IIR MAC Unit figure shows a pipelined multiplier and accumulator unit that operates on the data and coefficients fetched from the data and coefficient memory. The MAC can perform 32 -bit floating-point or 40 -bit floating-point MAC operations. The 32 -bit floating-point operations generate 32 -bit results and the 40 -bit floatingpoint operations generate 40 -bit results.

Figure 44-3: IIR MAC Unit

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000002_f51dd357e66c9199f2a298522f2b733a666f95b038fc7e675a2b7f56694a11ea.png)

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

## Target Coefficient Memory Storage

For coefficient slewing, the structure of the target coefficient array is like the coefficient memory storage. The first five coefficient of each biquad are stored in internal memory with the order: Ak0, Ak1, Bk1, Ak2, and Bk2. Following the five coefficients, two zeroes replace the initial state values Dk1 and Dk2.

IMPORTANT: The naming convention used in this hardware reference and that used in MATLAB follows the same conversion rule described for the Coefficient Memory Storage (Akx = bx and Bkx = -ax).

## Operating Modes

The accelerator operates:

- In Window Processing Mode, that also includes sample-based processing mode
- In 40 -Bit Floating-Point Mode
- In Save Biquad State Mode
- In Auto Configuration Mode (ACM)
- Calculates Coefficient Slewing

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

Figure 44-4: 32 -Bit to 40 -Bit Packing

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000003_d88794c572268e559368b920b9214dd8ace45fe8c698326d23f217634de4d9cd.png)

## Save Biquad State Mode

The IIR\_CTL1.SS bit stores the current biquad states in local memory (writes all the Dk1 and Dk2 states back into the system memory states). This is useful for applications that require fast switching to another high-priority accelerator task-a required IIR to FIR processing transition for example. After resuming, these states can be reloaded and IIR processing can be continued. DMA status is automatically stored after each iteration.

NOTE: In legacy mode, the accelerator loads the biquad coefficients for all the channels before processing the first channel. After processing the last channel, when the IIR\_CTL1.SS bit is set, the accelerator stores the biquad states of all the channels in local memory (writes all the Dk1 and Dk2 states back into the system memory states). In ACM mode, the accelerator loads the biquad coefficients for only the current channel

before starting to process the current channel. When the IIR\_SGCTL.SS bit is set, the accelerator stores the biquad states of the current channel in local memory before starting calculations for the next channel.

NOTE: During the save state operation, it is not permitted to have write access to any of the IIR accelerator registers that are loaded by chaining. Attempted writes to these registers can result in the blocking of IOP core reads until the save state operation completes.

## Auto Configuration Mode (ACM)

The accelerator can be operated in legacy mode or Auto Configuration Mode (ACM). ACM is configured by setting the IIR\_CTL1.ACM bit. The default functional mode is legacy mode. The accelerator must be disabled before changing the accelerator mode.

The ACM provides the following features:

- Halt and Queuing

The core can pause processing the current Transfer Control Block (TCB) chain by setting the IIR\_CTL1.HALT bit. The accelerator acknowledges the core by setting the IIR\_DMASTAT.HALT\_STAT bit. The core can then submit or insert new TCBs. After the core acts, the accelerator resumes processing by clearing the IIR\_CTL1.HALT bit. Before halting the accelerator, when the initial TCB chain is processed, the accelerator goes to the idle state. T o do this, disable the accelerator and then enable it by toggling the IIR\_CTL1.EN . Clear the IIR\_CTL1.HALT bit to resume processing.

- No Channel Number Limitation

Unlike in legacy mode, there is no fixed channel number limitation, and the accelerator ignores the value programmed in the IIR\_CTL1.CH field. The application can queue an unlimited number of channels and TCBs dynamically, plus the accelerator keeps processing the TCBs until the chain pointer is null.

- Selective Interrupt

The core can enable and mask interrupt generation for each channel by setting the IIR\_CTL2.IMASK bit. When the bit is cleared, an interrupt generates after the channel completes.

- Selective Controller/Target T rigger

The core can enable/mask trigger generation by the accelerator after the end of processing of each channel using the IIR\_CTL2.TMASK bit. The accelerator can also wait for a trigger after loading the TCB and coefficients and before processing a channel for which the IIR\_CTL2.TWAIT bit is set.

This feature can also be enabled in legacy mode by setting the IIR\_CTL1.L\_TIMASK\_EN and IIR\_CTL1.L\_TWAIT\_EN bits. In legacy mode, when the IIR\_CTL1.L\_TIMASK\_EN and IIR\_CTL1.L\_TWAIT\_EN bits are set, then the IIR\_CTL2.TMASK and IIR\_CTL2.TWAIT bits can be used to enable/mask trigger generation and wait for a trigger for each channel respectively.

Additionally, use the IIR\_CTL1 register to change parameters such as rounding mode, 40-bit mode, and save state for each channel. The IIR\_SCTL1 and IIR\_SCTL2 registers can be used as general-purpose registers.

## Coefficient Slewing

The following sections describe the features, parameters, and real-time processing for coefficient slewing.

- Coefficient Slewing Features
- Coefficient Slewing Parameters
- Coefficient Real-time Slewing

## Coefficient Slewing Features

Enable the slewing feature by setting IIR\_CTL1.SLEW\_EN bit. When this bit is cleared, the IIR operates in legacy mode. Once set, this bit should not be cleared until the IIR\_CTL1.EN bit is cleared.

The slewing feature does not support the ACM operating mode.

The slewing feature can be enabled in burst/non-burstmode, 40-bit or 32-bit mode, and in the channel auto interate (CAI) mode.

The IIR instances IIR0 and IIR1 each have one slewing engine.

Slewing calculations happen in parallel to its biquad execution within the IIR block. When slewed coefficients are needed for the next sample calculation, slewing of coefficients happens during the current sample.

Coefficients can be slewed for each sample (sample -based) or after a configured interval of samples.

Slew parameters can be configured on a per channel basis using the:

- Slewing Rate ( ɑ )
- Slewing Frequency (n)
- Slewing Time (N)

These parameters are part of the TCB for each channel. The TCB length is increased by three for legacy mode when the coefficient slewing is enabled.

With appropriate values of ɑ , n and N, the user can ensure that the coefficients approximate their target values at the end of the slewing process for each channel. Without proper values, the user may observe audio glitches after slewing completion because of the differences in target and final coefficients.

A separate memory for storing the Target Coefficients (C T ) resides inside the IIR accelerator.

The slewing rate ( ɑ ) can be kept constant or can be linearly changing from one to zero. The IIR\_CTL2.SLEW\_LALPHA\_MODE control bit defines the alpha configuration (constant or varying) operating for each channel.

Whenever the slewing saturates or slewing time expires (that is, after N iterations), C[n] either remains at the last iterated value or attains C T, based on the IIR\_CTL2.SLEW\_SAT\_EN bit set for each channel.

Slewing parameters ( ɑ , N, n) and target coefficients (C T ) can be updated during the slewing process by enabling the IIR\_CTL1.SLEW\_PARAMS\_UPDATE bit and by disabling and reenabling the DMA.

During channel switching, the slewing engine remembers the state of slewing in the current channel and restores the state once the channel comes back.

## Coefficient Slewing Parameters

## Slewing Rate ( ɑ )

The slewing rate ( ɑ ) is 32 -bit floating-point number with a value zero through one. When ɑ approximates one, it indicates a slow slewing process. When ɑ approximates zero, it indicates a fast slewing process. The slewing rate provides a 32 -bit floating-point number as the sixth element in the task control block (that is, TCB+0x5 or the IIR\_ALPHA[n] register for the nth channel).

## Slewing Frequency (n)

The slewing frequency (n) determines after how many samples the slew calculation for the IIR coefficients happens. When n = 0, beginning from the second sample every sample output is calculated using the updated slewed coefficients (see the IIR Coefficient Slewing (constant alpha mode) n=0 and n=3 Examples figure). For n &gt; 0, the accelerator coefficients are slewed after every n+1 samples and output of n+1 samples are calculated using last slewed coefficients. An internal counter keeps track of the number of samples completed to take a decision on performing the slewing for the next sample.

For instance, when n = 3, starting from the 5th sample, every fourth sample (N+1) output is calculated using new coefficients (that is, fifth through eighth samples). Coefficients are slewed again at the nineth sample and every fourth sample thereafter.

Figure 44-5: IIR Coefficient Slewing (constant alpha mode) n=0 and n=3 Examples

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000004_4358a5773ca0d80a39753ebc08b477fe806f0b41be4367397d0417ee44b82e1f.png)

## Slewing Time (N)

The slewing time (N) indicates the number of iterations of coefficient slewing performed by the slewing engine. This parameter is defined for each channel and decremented inside the slewing engine after each slewing iteration.

Once it reaches zero, the slewing stops. The total number of samples processed during an entire slewing interval is (n*N) samples.

## Coefficient Real-time Slewing

This processor has a dedicated hardware to slew the IIR coefficients in real time and in parallel to the IIR filtering. Typically, slewing done through software takes up significant core MIPS and DMA overhead for loading slewed coefficients. The dedicated hardware engine placed inside the IIR filter can slew IIR biquad coefficients in real time and avoids using the core. Slewing ensures a smooth transition of IIR coefficients during the change of IIR parameters, that can otherwise result in audio click noises.

Figure 44-6: IIR Slewing Block Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000005_cf383c01de91b749b94e4a8b95749d572d4161dde4056157cd8eb318f89af195.png)

One slewing engine connects to each of the IIR filters. The coefficient slewing is done based on the following recursive equation:

```
C[n] = (1-ɑ) * CT + ɑ*C[n-1],  where C = {Ak0, Ak1, Bk1, Ak2, Bk2} and 0≤ɑ≤1 where CT is the target coefficient, ɑ is the slewing rate, and n is sample number.
```

## Data Transfers

The IIR filter works exclusively through DMA.

## IIR Accelerator TCB

The location of the DMA parameters for the next sequence comes from the chain pointer register. This register points to the next set of DMA parameters stored in the system memory of the processor known as the Task Control Block (TCB). In chained DMA operations, the processor automatically initializes and then begins another DMA transfer once the current DMA transfer completes. Each new set of parameters is stored in a user-initialized memory buffer or TCB for a chosen peripheral.

## Chain Assignment

The structure of a TCB is conceptually the same as the structure of a linked list. Each TCB has several data values and a pointer to the next TCB. The chain pointer of a TCB can point to itself to re-run the same DMA continuously. The IIR accelerator reads each word of the TCB and loads it into the corresponding register. A TCB with a chain pointer register value of zero indicates the end of the chain (no further TCBs to load). The IIR

accelerator supports circular buffer chained DMA. The following tables show the required TCBs for chained DMA in Legacy Mode and ACM. TCB refers to the start address of the TCB array.

NOTE: The IIR accelerator DMA has two available TCB loading sequences:

- Loads five parameters for the coefficients ( IIR\_CTL2 , IIR\_COEFIDX , IIR\_COEFMOD , IIR\_COEFLEN , and IIR\_CHNPTR ).
- Loads 10 parameters for the data ( IIR\_CTL2 , IIR\_INIDX , IIR\_INMOD , IIR\_INLEN , IIR\_INBASE , IIR\_OUTIDX , IIR\_OUTMOD , IIR\_OUTLEN , IIR\_OUTBASE , and IIR\_CHNPTR ).

Initialize IIR\_CHNPTR to TCB+12 in legacy mode and TCB+15 in ACM.

Table 44-5: IIR TCBs for Chained DMA in Lega-

cy Mode

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

Table 44-6: IIR TCBs for Chained DMA in ACM

| Address   | Register    |
|-----------|-------------|
| TCB       | IIR_CHNPTR  |
| TCB + 0x1 | IIR_SCTL1   |
| TCB + 0x2 | IIR_SCTL2   |
| TCB + 0x3 | IIR_SGCTL   |
| TCB + 0x4 | IIR_COEFLEN |
| TCB + 0x5 | IIR_COEFMOD |

Table 44-6: IIR TCBs for Chained DMA in ACM (Continued)

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

Table 44-7: Slewing TCB Structure

| Address   | Register                   |
|-----------|----------------------------|
| TCB       | IIR_CHNPTR                 |
| TCB+0x1   | IIR_COEFLEN                |
| TCB+0x2   | IIR_COEFMOD                |
| TCB+0x3   | IIR_COEFIDX                |
| TCB+0x4   | IIR_CTIDX                  |
| TCB+0x5   | IIR_ALPHA[n]               |
| TCB+0x6   | IIR_Nn                     |
| TCB+0x7   | IIR_OUTBASE                |
| TCB+0x8   | IIR_OUTLEN                 |
| TCB+0x9   | IIR_OUTMOD                 |
| TCB+0xA   | IIR_OUTIDX                 |
| TCB+0xB   | IIR_INBASE                 |
| TCB+0xC   | IIR_INLEN                  |
| TCB+0xD   | IIR_INMOD                  |
| TCB+0xE   | IIR_INIDX                  |
| TCB+0xF   | IIR_CTL2.SLEW_LALPHA_M ODE |

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

The accelerator loads the TCB into its internal registers and uses these values to fetch coefficients, fetch data, and store results. After processing a window of data for any channel, the accelerator writes back the IIR\_INIDX (input

index register) and IIR\_OUTIDX (output index register) values to the TCB in memory. Data processing can then begin from where it left off during the next time slot of that channel.

For 32-bit mode, the write-back values for the index registers are equal to IIRII + W and IIROI + W.

For 40-bit mode, the write-back values are: IIR\_INIDX + 2 × W and IIR\_OUTIDX + 2 × W.

Accelerator input and output channels connect to system memory.

NOTE: The IIR\_CTL2 register is part of the IIR TCB. This configuration allows software to program individual IIR channels having different control attributes.

In ACM, when the IIR\_CTL1.SMQ\_LIUPS\_EN bit is set, the accelerator updates the IIR\_INIDX and IIR\_OUTIDX bit fields of the TCB in memory after processing a window of data and according to the circular buffer scheme. When the IIR\_CTL1.SMQ\_LIUPS\_EN bit is cleared, the accelerator updates the IIR\_INIDX and IIR\_OUTIDX bit fields of the TCB in memory to 0x00000000 and 0xFFFFFFFF after processing a window of data. The IIR\_CTL1.SMQ\_LIUPS\_EN bit is only valid in ACM.

Figure 44-7: Circular Buffer Addressing

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000006_eb1460d2d6a09f998dd707dc2a3068d2c2c2ec2605af63e29e9e1c4b92ebe785.png)

## Interrupts

The IIR Interrupt Overview table provides the source of interrupt and service instructions for the IIR interrupts.

Table 44-8: IIR Interrupt Overview

| Accelerator Mode   | Default Programmable Interrupt   | Sources                            | Masking   | Service                                |
|--------------------|----------------------------------|------------------------------------|-----------|----------------------------------------|
| Legacy Mode        | IIR_DMA                          | Window Complete                    | N/A       | ROC from IIR_DMASTAT + RTI instruction |
| Legacy Mode        |                                  | All channels complete              |           |                                        |
| Legacy Mode        | IIR_STAT                         | MAC IEEE floating point exceptions | N/A       | ROC from IIR_MACSTAT + RTI instruction |
| Legacy Mode        |                                  | MAC fixed point overflow           |           |                                        |

Table 44-8: IIR Interrupt Overview (Continued)

| Accelerator Mode        | Default Programmable Interrupt   | Sources                            | Masking        | Service                                |
|-------------------------|----------------------------------|------------------------------------|----------------|----------------------------------------|
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

The slewing engine inside IIR accelerator has a floating-point subtractor, multiplier, and adder for calculating the next iteration of coefficients. The slewing engine interrupts are ORed with the previous IIR MAC Status interrupt and only sampled when slewing is enabled (set the IIR\_CTL1.SLEW\_MAC\_INT\_EN bit). A MAC status interrupt generates for any of the following conditions:

- Adder output is infinity, invalid, or zero
- Multiplier output is infinity, invalid, or zero
- Subtractor output is infinity or invalid

## Service

The DMA interrupt status bits are sticky and are cleared when the DMA status register is read. When a MAC status interrupt occurs, programs can find this state (and clear) by reading the MAC status register ( IIR\_MACSTAT ). The MAC interrupt status bits are sticky.

The status interrupt sources are derived from the IIR\_MACSTAT register. A status interrupt can occur due to the last set of MAC operations of a processing iteration that correspond to a particular channel. The interrupt is generated continuously and cannot be stopped, even after disabling the accelerator. The interrupt can only be stopped when another processing iteration results in a non-zero or valid multiply or add result.

## Programming Model

The following sections provide programming examples for legacy and auto configuration modes.

The Coefficient Slewing Programming section provides detailed programming steps for the different modes to calculate slewing coefficients.

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

Figure 44-8: Biquad Processing Program Flow

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000007_647fa3862ad0953e4cb81bea1c7fdcc4c1fa0c34208f6168f503af68cc24fd67.png)

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

Figure 44-9: Multi-channel Filtering in Auto Configuration Mode

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000008_df1617dc4932a2f637012183f8acec65c4624c90eddd2904c1c42f96e38c8582.png)

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

## Coefficient Slewing Programming

- Coefficient Slewing Program Flow
- Programming Constant Alpha Slewing Mode
- Programming Linear Changing Alpha Slewing Mode
- Programming Intermediate Target Coefficient Update Mode

NOTE: The save state bit should be disabled ( IIR\_CTL1.SS =0) when the coefficient slewing bit is enabled ( IIR\_CTL1.SLEW\_EN =1).

## Coefficient Slewing Program Flow

The Slewing Operation Program Flow figure captures the coefficient slewing program flow that is common for all slewing modes described:

Figure 44-10: Slewing Operation Program Flow

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000009_e001b16f83f1ca3fd939c4ab0fe575630711c3114fd9527c961cb02154a04133.png)

## Programming Constant Alpha Slewing Mode

To program slewing engine in constant alpha ( ɑ ) mode, follow the below mentioned steps:

1. Enable coefficient slewing by setting the IIR\_CTL1.SLEW\_EN bit.
2. Provide the pointer to the target coefficient memory ( TCB+0x4 , IIR\_CTIDX) along with current coefficient memory pointer ( TCB+0x3 , IIR\_COEFIDX ).
3. Set slewing rate ( ɑ ) for the current channel using the 32-bit float format ( TCB+0x5 , IIR\_ALPHA[n] ).
4. Program the slewing frequency (n) and slewing time (N) values at ( TCB+0x6 , IIR\_Nn) using the format in the IIR\_SLEW\_CNFG[n] register. Bits[0:7] should hold the value of slewing frequency (n) and bits[23:8] should hold the value of slewing time (N). The remaining bits must be zero.
5. Clear the IIR\_CTL2.SLEW\_LALPHA\_MODE bit ( TCB+0xF ) to configure the slewing engine for constant alpha slewing mode.
6. When exact target coefficients are required after N iterations of slewing, set the IIR\_CTL2.SLEW\_SAT\_EN bit to one.

NOTE: In constant alpha slewing mode, the window size should be a multiple of the slewing frequency (n+1).

## Programming Linear Changing Alpha Slewing Mode

In this mode, the slewing rate ( ɑ ) changes along with the coefficients after every iteration of slewing. During N slewing iterations, the value of alpha changes as (N-1)/N, (N-2)/N, … ,1/N, 0/N. Because ɑ =0 for the last iteration of slewing, exact target coefficients are reached for each biquad.

To program the slewing engine in linearly changing alpha ( ɑ ) mode, follow these steps:

1. Enable coefficient slewing by setting the IIR\_CTL1.SLEW\_EN bit.
2. Provide the pointer to the target coefficient memory ( TCB+0x4 , IIR\_CTIDX) along with current coefficient memory pointer ( TCB+0x3 , IIR\_COEFIDX ).
3. Set slewing rate ( ɑ ) for the current channel using the 32-bit float format ( TCB+0x5 , IIR\_ALPHA[n] ).
4. Program the slewing frequency (n) and slewing time (N) values at ( TCB+0x6 , IIR\_Nn) using the format in the IIR\_SLEW\_CNFG[n] register. Bits[0:7] should hold the value of slewing frequency (n) and bits[23:8] should hold the value of slewing time (N). The remaining bits must be zero.
5. Set the IIR\_CTL2.SLEW\_LALPHA\_MODE bit to enable linear varying alpha mode for this channel.
6. Program the slewing rate ( ɑ ) in ( TCB+0x5 , IIR\_ALPHA[n] ) as 1/N in 32-bit float format. The slewing engine internally calculates the actual alpha to be used for each iteration.
7. The value of alpha is only updated at the start of the window. This restricts the slewing frequency (n) to a multiple of the window sample size. That is, n = p*(window size-1) , where p=1, 2, 3…

## Programming Intermediate Target Coefficient Update Mode

Use Intermediate Target Coefficient Update Mode for achieving multiple target values for each coefficient after each slewing interval.

To program the slewing engine in Coefficient Update Mode, follow these steps:

1. Enable coefficient slewing by setting the IIR\_CTL1.SLEW\_EN bit.
2. Provide the pointer to the target coefficient memory ( TCB+0x4 , IIR\_CTIDX) along with current coefficient memory pointer ( TCB+0x3 , IIR\_COEFIDX ).
3. Set slewing rate ( ɑ ) for the current channel using the 32-bit float format ( TCB+0x5 , IIR\_ALPHA[n] ).
4. Program the slewing frequency (n) and slewing time (N) values at ( TCB+0x6 , IIR\_Nn) using the format in the IIR\_SLEW\_CNFG[n] register. Bits[0:7] should hold the value of slewing frequency (n) and bits[23:8] should hold the value of slewing time (N). The remaining bits must be zero.
5. The slewing parameters slew rate( ɑ ), C T , slewing time (N), and slewing frequency (n) can be updated during the slewing process.
6. When servicing a DMA interrupt, toggle the IIR\_CTL1.DMAEN and IIR\_CTL1.SLEW\_MAC\_INT\_EN bits from zero to one to enable this mode.
7. When this mode is enabled, the IIR loads updated values of alpha, N, n, and the target coefficients.

## ADSP-2184x IIR Register Descriptions

The IIR filter accelerator (IIR) contains the following registers.

Table 44-9: ADSP-2184x IIR Register List

| Name             | Description                              |
|------------------|------------------------------------------|
| IIR_ALPHA[n]     | Channel n Alpha                          |
| IIR_CHNPTR       | Chain Pointer Register                   |
| IIR_COEFIDX      | Coefficient Buffer Index Register        |
| IIR_COEFLEN      | Coefficient Buffer Length Register       |
| IIR_COEFMOD      | Coefficient Index Modifier Register      |
| IIR_CTL1         | Global Control Register                  |
| IIR_CTL2         | Channel Control Register                 |
| IIR_DBG_ADDR     | IIR Debug Address Register               |
| IIR_DBG_CTL      | IIR Debug Control Register               |
| IIR_DBG_RDDAT_HI | IIR Debug Read Data High Register        |
| IIR_DBG_RDDAT_LO | IIR Debug Read Data Low Register         |
| IIR_DBG_WRDAT_HI | IIR Debug Write Data High Register       |
| IIR_DBG_WRDAT_LO | IIR Debug Write Data Low Register        |
| IIR_DMASTAT      | DMAStatus Register                       |
| IIR_INBASE       | Input Buffer Base Register               |
| IIR_INIDX        | Input Data Index Register                |
| IIR_INLEN        | Input Data Buffer Length Register        |
| IIR_INMOD        | Input Data Index Modifier Register       |
| IIR_MACSTAT      | MAC Status Register                      |
| IIR_OUTBASE      | Output Buffer Base Register              |
| IIR_OUTIDX       | Output Data Buffer Index Register        |
| IIR_OUTLEN       | IIR Output Data Buffer Length Register   |
| IIR_OUTMOD       | IIR Output Data Index Modifier Register  |
| IIR_SCTL1        | Software Control Register1               |
| IIR_SCTL2        | Software Control Register2               |
| IIR_SGCTL        | Secondary Global Control Register        |
| IIR_SLEW_CNFG[n] | Channel 0 Slewing Duration and Frequency |

## Channel n Alpha

The IIR\_ALPHA[n] register gives access to Alpha, which is in 32-bit floating-point format. The register has both read and write access. However, a write to this register cannot be performed during non-IDLE state of DMA.

Figure 44-11: IIR\_ALPHA[n] Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000010_02f1f73fff1a8edb3014c69ef0c275071fa879dbb1988296446706bec7130b89.png)

Table 44-10: IIR\_ALPHA[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VAL        | Alpha Value. The IIR_ALPHA[n].VAL bit field indicates the alpha value for channel n in slewing mode. |

## Chain Pointer Register

The IIR\_CHNPTR register should be written with word address without the lower 2 bits.

Figure 44-12: IIR\_CHNPTR Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000011_66c3bc902f49d7c98bbc017451120960495db38514d75a5d6fed68168f9f7a50.png)

Table 44-11: IIR\_CHNPTR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                            |
|--------------------|------------|--------------------------------------------------------------------|
| 29:0               | VALUE      | IIR Chain Pointer Address.                                         |
| (R/W)              |            | The IIR_CHNPTR.VALUE bit field contains the chain pointer address. |

## Coefficient Buffer Index Register

The IIR\_COEFIDX register contains the word address with the lower 2 bits removed.

Figure 44-13: IIR\_COEFIDX Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000012_9d866bf6513b94c039dee86f2a99dd77481bba5f367e1cdef1124bcfe8b9a9ca.png)

Table 44-12: IIR\_COEFIDX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                |
|--------------------|------------|------------------------------------------------------------------------|
| 29:0               | VALUE      | Coefficient Buffer Index.                                              |
| (R/W)              |            | The IIR_COEFIDX.VALUE bit field provides the coefficient buffer index. |

## Coefficient Buffer Length Register

The IIR\_COEFLEN register provides the coefficient buffer length.

Figure 44-14: IIR\_COEFLEN Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000013_6a280d9186469b1ea3448d3f88153530a50a3b38e886a2eaaa8aa208dd4e8c64.png)

Table 44-13: IIR\_COEFLEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                 |
|--------------------|------------|-------------------------------------------------------------------------|
| 15:0               | VALUE      | Coefficient Length.                                                     |
| (R/W)              |            | The IIR_COEFLEN.VALUE bit field provides the coefficient buffer length. |

## Coefficient Index Modifier Register

The IIR\_COEFMOD register provides the coefficient index modifier.

Figure 44-15: IIR\_COEFMOD Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000014_080f37030d0ed4b32832f2a5f97ef27cff9d99c1a79764f546a2b93f401ac42d.png)

Table 44-14: IIR\_COEFMOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                            |
|--------------------|------------|--------------------------------------------------------------------|
| 15:0               | VALUE      | Coefficient Modifier.                                              |
| (R/W)              |            | The IIR_COEFMOD.VALUE bit field provides the coefficient modifier. |

## Global Control Register

The IIR\_CTL1 register is used to configure the global parameters for the accelerator. These parameters include the number of channels, channel auto iterate, DMA enable, and accelerator enable.

Figure 44-16: IIR\_CTL1 Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000015_065d22eb8cc6106d026ad73b96adfac2bc3fd80585968ef0e5d998ea47a20509.png)

Table 44-15: IIR\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------|
| 31 (R/W)           | ACM        | Auto Configuration Mode. The IIR_CTL1.ACM bit configures the mode for loading the TCB. |
| 31 (R/W)           | ACM        | 0 Legacy Mode                                                                          |
| 31 (R/W)           | ACM        | 1 Auto Configuration Mode                                                              |
| 31 (R/W)           | ACM        |                                                                                        |

Table 44-15: IIR\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                                     |
|--------------------|--------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30 (R/W)           | HALT         | Pause accelerator. The IIR_CTL1.HALT bit determines whether the accelerator pauses so that the core can check the status and modify or submit a job or the accelerator is released for further processing of data. This bit is only valid in Auto Configuration Mode (ACM). |
| 29 (R/W)           | SSESEL       | Write Back Interrupt Select. The IIR_CTL1.SSESEL bit selects whether to generate an interrupt after the write back or after save state completion.                                                                                                                          |
| 28 (R/W)           | L_TWAIT_EN   | Enable TWAIT Feature in Legacy Mode. The IIR_CTL1.L_TWAIT_EN bit indicates whether the TWAIT feature is availa- ble in legacy mode. This bit is only valid in legacy mode.                                                                                                  |
| 27 (R/W)           | L_TIMASK_EN  | 1 Available Enable Trigger and Interrupt Masking in Legacy Mode. The IIR_CTL1.L_TIMASK_EN bit enables trigger and interrupt masking in legacy mode. This bit is only valid in legacy mode.                                                                                  |
| 26 (R/W)           | SMQ_LIUPS_EN | 0 Disable 1 Enable Legacy I/P and O/P Index Update Scheme in SMART_Q Mode. The IIR_CTL1.SMQ_LIUPS_EN bit configures the scheme the accelerator uses to                                                                                                                      |
| 17 (R/W)           | BURSTEN      | update the input index (II) and output index (OI). This bit is only valid in ACM. 0 Update the II with all '0' and the OI with all 'F', respec- tively 1 Update II and OI after circular buffer scheme                                                                      |
|                    |              | Burst Mode Enable. The IIR_CTL1.BURSTEN bit field is set, burst access is enabled. 0 Disable burst mode                                                                                                                                                                     |

Table 44-15: IIR\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|-----------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | RND             | Rounding Mode. The IIR_CTL1.RND bit field selects the rounding mode for floating-point format.                                                                                                                                                                                                                                                                                                                            |
| 14 (R/W)           | RND             | 0 Round to nearest (even)                                                                                                                                                                                                                                                                                                                                                                                                 |
| 13 (R/W)           | SLEW_MAC_INT_EN | Slew Interrupt Enable. The IIR_CTL1.SLEW_MAC_INT_EN bit enables interrupts from slewing floating point units.                                                                                                                                                                                                                                                                                                             |
| 12 (R/W)           | FORTYBIT        | 40-Bit Floating-Point Boundary Select. The IIR_CTL1.FORTYBIT bit selects between 32-bit IEEE floating-point format or 40-bit IEEE floating-point format.                                                                                                                                                                                                                                                                  |
| 12 (R/W)           | FORTYBIT        | 0 32-bit IEEE floating-point                                                                                                                                                                                                                                                                                                                                                                                              |
| 11 (R/W)           | CCINTR          | Channel Complete Interrupt. The IIR_CTL1.CCINTR bit configures the channel complete interrupt to generate when all channels are done or after each channel is done. This bit is only valid in legacy mode when the IIR_CTL1.L_TIMASK_EN bit is cleared (=0). In Auto Configuration Mode (ACM) or when the IIR_CTL1.L_TIMASK_EN is set (=1), interrupt generation for each channel is controlled using IIR_CTL2.IMASK bit. |
| 11 (R/W)           | CCINTR          | 0 Interrupt is generated only when all channels are done (default)                                                                                                                                                                                                                                                                                                                                                        |
| 11 (R/W)           | CCINTR          | 1 Interrupt is generated after each channel is done (de- fault)                                                                                                                                                                                                                                                                                                                                                           |
| 10 (R/W)           | SS              | Save Biquad State. The IIR_CTL1.SS bit configures the accelerator to store the Dk register settings into the internal memory. This can be used to save the biquad states before switching to another high priority accelerator task. This bit is only valid in legacy mode. In Auto Configuration Mode (ACM), the save state for each channel is controlled using the IIR_SGCTL.SS bit                                    |

Table 44-15: IIR\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name             | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|----------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | CAI                  | Channel Auto Iterate. The IIR_CTL1.CAI bit sets whether TDMprocessing stops (idle) once all channels complete processing or moves to first channel and continues TDMprocessing in a loop when all channels complete processing. Channel Auto Iterate is not available in Auto Configuration Mode (ACM). The accelerator keeps processing the TCBs until the chain pointer becomes null in ACM.                              |
| 8 (R/W)            | DMAEN                | DMAEnable. The IIR_CTL1.DMAEN bit enables DMAon the accelerator. 0 Disable                                                                                                                                                                                                                                                                                                                                                  |
| 7 (R/W)            | SLEW_EN              | Slewing Enable. The IIR_CTL1.SLEW_EN bit enables control of IIR coefficients slewing engine. 0 Disable Slewing Engine (default)                                                                                                                                                                                                                                                                                             |
| 6 (R/W)            | SLEW_PARAMS_UP- DATE | Slewing Parameters. Slewing parameters (alpha, slewing duration, slewing frequency) and target co- efficients can be updated during the process of slewing by enabling this IIR_CTL1.SLEW_PARAMS_UPDATE bit and by disabling and re-enabling the DMA.                                                                                                                                                                       |
| 5:1 (R/W)          | CH                   | Number of Channels. The IIR_CTL1.CH bit field configures the number of channels and is programma- ble between 0-31 (channels = NCH+1). This bit field is only valid in legacy mode. In Auto Configuration Mode (ACM), there is no limit on the number of channels. the accelerator keeps processing the TCBs until the chain pointer becomes null. IIR Enable. The IIR_CTL1.EN bit enables or disables the IIR accelerator. |
| 0 (R/W)            | EN                   | 0 IIR disabled 1 IIR enabled                                                                                                                                                                                                                                                                                                                                                                                                |

## Channel Control Register

The IIR\_CTL2 register is used to configure the channel specific parameters. These parameters include the number of biquads and window size. In Auto Configuration Mode (ACM), this register is also used to configure additional channel specific parameters like interrupts and triggers.

Figure 44-17: IIR\_CTL2 Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000016_82099e0ca2f70f99aaf9b2008f48349c98d4463b0fcaadfa2a25e45865e62fda.png)

Table 44-16: IIR\_CTL2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | TMASK      | Trigger mask bit. The IIR_CTL2.TMASK bit enables trigger generation for the channel. It is valid in Auto Configuration Mode (ACM) or if CTL1.L_TIMASK_EN is set. 0 Enable                                                                                                 |
| 28 (R/W)           | TWAIT      | Wait for trigger. The IIR_CTL2.TWAIT bit disables the wait for the trigger. It is valid in Auto Configuration Mode (ACM) or if CTL1.L_TIMASK_EN is set. 0 Disable wait for the trigger for the channel 1 Enable wait for external trigger input assertion for the channel |

Table 44-16: IIR\_CTL2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 24 (R/W)           | IMASK            | Interrupt Mask. The IIR_CTL2.IMASK bit enables interrupt generation for the channel. This bit is valid in Auto Configuration Mode (ACM) or if CTL1.L_TIMASK_EN is set.                                                                                                                                                                            |
| 24 (R/W)           | IMASK            | 0 Enable                                                                                                                                                                                                                                                                                                                                          |
| 24 (R/W)           | IMASK            | 1 Mask                                                                                                                                                                                                                                                                                                                                            |
| 23:14 (R/W)        | WINDOW           | Window Size Parameter. The IIR_CTL2.WINDOW bit field configures the window size which specifies the number of samples/blocks to process (sample based processing = window size of 1). This bit field should be programmed to "actual window size required -1". For example, for sample based processing this bit field should be programmed to 0. |
| 13:12 (R/W)        | PRIO             | Priority Level. The IIR_CTL2.PRIO bit field indicates the priority.                                                                                                                                                                                                                                                                               |
| 13:12 (R/W)        | PRIO             | 0 Level 0 (lowest)                                                                                                                                                                                                                                                                                                                                |
| 13:12 (R/W)        | PRIO             | 1 Level 1                                                                                                                                                                                                                                                                                                                                         |
| 13:12 (R/W)        | PRIO             | 2 Level 2                                                                                                                                                                                                                                                                                                                                         |
| 13:12 (R/W)        | PRIO             | 3 Level 3 (highest)                                                                                                                                                                                                                                                                                                                               |
| 7 (R/W)            | SLEW_SAT_EN      | Control bit to choose the way slewing saturates. IIR_CTL2.SLEW_SAT_EN bit field is set to HIGH to assume target coefficients after slewing duration. If this bit is not set, coefficients will remain at the last iteration values                                                                                                                |
| 6 (R/W)            | SLEW_LALPHA_MODE | Alpha Mode. The IIR_CTL2.SLEW_LALPHA_MODE bit controls whether to slew in constant alpha mode or linearly changing alpha mode. Program IIR_CTL2.SLEW_LALPHA_MODE bit field as, 0: Default setting; Constant alpha mode 1: Linearly changing alpha mode Mode (default)                                                                             |
| 6 (R/W)            | SLEW_LALPHA_MODE | 0 Constant Alpha                                                                                                                                                                                                                                                                                                                                  |
| 6 (R/W)            | SLEW_LALPHA_MODE | 1 Linearly Changing Alpha Mode                                                                                                                                                                                                                                                                                                                    |
| 5:0 (R/W)          | BIQUADS          | Number of Biquads. The IIR_CTL2.BIQUADS bit field configures the number of biquads and is pro- grammable between 0-63 (number of Biquads = BIQUADS + 1).                                                                                                                                                                                          |

## IIR Debug Address Register

The IIR\_DBG\_ADDR register holds the debug address. If bit 11 is set, coefficient memory is selected.

Figure 44-18: IIR\_DBG\_ADDR Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000017_6df0991ebcb129c9810879e5984fe783d00206a46e867c958bc08f8017f3c0fa.png)

Table 44-17: IIR\_DBG\_ADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10:0 (R/W)         | VALUE      | Debug Address, Coefficient Memory Select. The IIR_DBG_ADDR.VALUE bit field holds the debug address (bits 0-10). Bit 11 configures whether the memory access is to coefficient memory (=0) or to delay line memory (=1). |

## IIR Debug Control Register

Figure 44-19: IIR\_DBG\_CTL Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000018_a91c51b660d405c9f6a11e7a996e43ba0884813af35ffdd80ee05c590225b8f2.png)

Table 44-18: IIR\_DBG\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                | Description/Enumeration                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W1S)          | ADRINC     | Address Auto Increment. The IIR_DBG_CTL.ADRINC bit allows the address register to au- to increment on IIR_DBG_WRDAT_HI / IIR_DBG_WRDAT_LO writes and IIR_DBG_RDDAT_HI / IIR_DBG_RDDAT_LO reads.        | Address Auto Increment. The IIR_DBG_CTL.ADRINC bit allows the address register to au- to increment on IIR_DBG_WRDAT_HI / IIR_DBG_WRDAT_LO writes and IIR_DBG_RDDAT_HI / IIR_DBG_RDDAT_LO reads.        |
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

Figure 44-20: IIR\_DBG\_RDDAT\_HI Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000019_1271921848d339e031be292b6b7ff793f380eaaefeaddef80c64be60c766bada.png)

Table 44-19: IIR\_DBG\_RDDAT\_HI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                               |
|--------------------|------------|-----------------------------------------------------------------------|
| 7:0                | VALUE      | Debug Read Data Highest 8 Bits.                                       |
| (R/NW)             |            | The IIR_DBG_RDDAT_HI.VALUE bit field holds the upper 8-bit read data. |

## IIR Debug Read Data Low Register

The IIR\_DBG\_RDDAT\_LO register is part of the 40-bit wide debug mode read data register and holds the lower 32 bits.

Figure 44-21: IIR\_DBG\_RDDAT\_LO Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000020_8cf559d7fb028aad78853ae582e53c1bb451c5c139c397377a63e2b0bbdb93a8.png)

Table 44-20: IIR\_DBG\_RDDAT\_LO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                |
|--------------------|------------|------------------------------------------------------------------------|
| 31:0               | VALUE      | Debug Read Data Lower 32 Bits.                                         |
| (R/NW)             |            | The IIR_DBG_RDDAT_LO.VALUE bit field holds the lower 32-bit read data. |

## IIR Debug Write Data High Register

The IIR\_DBG\_WRDAT\_HI register is part of the 40-bit wide debug mode write data register and holds the upper 8 bits.

Figure 44-22: IIR\_DBG\_WRDAT\_HI Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000021_8850907e6b7bcad1f5fc5479c5b5e339bedac5cfbb9a4e6740f282d7e3da6bc2.png)

Table 44-21: IIR\_DBG\_WRDAT\_HI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                |
|--------------------|------------|------------------------------------------------------------------------|
| 7:0                | VALUE      | Debug Write Data Highest 8 Bits.                                       |
| (R/W)              |            | The IIR_DBG_WRDAT_HI.VALUE bit field holds the upper 8-bit write data. |

## IIR Debug Write Data Low Register

The IIR\_DBG\_WRDAT\_LO register is part of the 40-bit wide debug mode write data register and holds the lower 32 bits.

Figure 44-23: IIR\_DBG\_WRDAT\_LO Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000022_9dc8cbc5ed240d330298478ce3385bafef7fecc84685986890c74a0246f76bf6.png)

Table 44-22: IIR\_DBG\_WRDAT\_LO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                 |
|--------------------|------------|-------------------------------------------------------------------------|
| 31:0               | VALUE      | Debug Write Data Lower 32 Bits.                                         |
| (R/W)              |            | The IIR_DBG_WRDAT_LO.VALUE bit field holds the lower 32-bit write data. |

## DMA Status Register

The IIR\_DMASTAT registers indicate the status of DMA operations.

Figure 44-24: IIR\_DMASTAT Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000023_4b549d1efd69c9fab135f2ee13429200618e651c2547403f53cc4a47be962927.png)

Table 44-23: IIR\_DMASTAT Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                             |
|--------------------|----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/NW)          | CHNL_SLEW_DONE | Channel Slew Done. The IIR_DMASTAT.CHNL_SLEW_DONE bit field indicates the completion of slewing (N iterations) in the current channel.                                                                                              |
| 12 (R/NW)          | HALT_STAT      | Accelerator HALT status. Acknowledge to core that accelerator is in halt state, if set.                                                                                                                                             |
| 11:7 (RC/NW)       | CURCHNL        | Current Channel. The IIR_DMASTAT.CURCHNL bit field indicates the channel that is being process- ed in the TDMslot. Zero indicates the last slot.                                                                                    |
| 6 (RC/NW)          | ACDONE         | All Channels Done. The IIR_DMASTAT.ACDONE bit indicates all channels are done processing. Note that the IIR_CTL1.CCINTR bit does not affect this status bit. This bit is sticky and is cleared on register read.                    |
| 5 (R/NW)           | WDONE          | Current Channel Done. The IIR_DMASTAT.WDONE bit indicates the processing of the current channel is complete. Note that the IIR_CTL1.CCINTR bit does not affect this status bit. This bit is sticky and is cleared on register read. |

Table 44-23: IIR\_DMASTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/NW)           | SVDK       | Save Updated Dk State. If there is more than one channel ( IIR_CTL1.CH >0), the IIR_DMASTAT.SVDK bit toggles between 0 and 1 as it starts and completes the save state operation on one channel at a time. |
| 3 (R/NW)           | WRBK       | Write Back. The IIR_DMASTAT.WRBK bit indicates the accelerator is writing back updated index registers.                                                                                                    |
| 2 (R/NW)           | PPGS       | MAC Processing in Progress. The IIR_DMASTAT.PPGS bit indicates MAC processing is in progress.                                                                                                              |
| 1 (R/NW)           | CDKLD      | Coefficient and Dk Loading. The IIR_DMASTAT.CDKLD bit indicates the coefficient and Dk are loading.                                                                                                        |
| 0 (R/NW)           | CPLD       | Chain Pointer Loading Status. The IIR_DMASTAT.CPLD bit indicates the IIR is in the chain pointer load state.                                                                                               |
|                    |            | 0 State machine not in chain pointer load state                                                                                                                                                            |
|                    |            | 1 State machine in chain pointer load state                                                                                                                                                                |

## Input Buffer Base Register

The IIR\_INBASE register contains the word address with the lower 2 bits removed.

Figure 44-25: IIR\_INBASE Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000024_2356fbe47f8f09a89c38c8be4932ab4963893054326688898ed9ad13e33bb790.png)

Table 44-24: IIR\_INBASE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                     |
|--------------------|------------|-----------------------------------------------------------------------------|
| 29:0               | VALUE      | Input Data Buffer Base.                                                     |
| (R/W)              |            | The IIR_INBASE.VALUE bit field value is the input data buffer base address. |

## Input Data Index Register

The IIR\_INIDX register contains a word address with the lower 2 bits removed.

Figure 44-26: IIR\_INIDX Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000025_b903b87776c78109f880390c37a755688772c7ee49544c3471d50d3f74f0af79.png)

Table 44-25: IIR\_INIDX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                             |
|--------------------|------------|---------------------------------------------------------------------|
| 29:0               | VALUE      | Input Data Buffer Index.                                            |
| (R/W)              |            | The IIR_INIDX.VALUE bit field value is the input data buffer index. |

## Input Data Buffer Length Register

The IIR\_INLEN register provides the input data buffer length.

Figure 44-27: IIR\_INLEN Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000026_5c96d59e24358d53ed33be530163bc3ea8d983583a454239c72aa79ea9dd40ca.png)

Table 44-26: IIR\_INLEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                              |
|--------------------|------------|----------------------------------------------------------------------|
| 15:0               | VALUE      | Input Data Buffer Length.                                            |
| (R/W)              |            | The IIR_INLEN.VALUE bit field value is the input data buffer length. |

## Input Data Index Modifier Register

The IIR\_INMOD register provides the 16-bit input data buffer index modifier.

Figure 44-28: IIR\_INMOD Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000027_09ebf293f00aa9466989ccc7e7e1fe5b3ec1b724776186fc72aaf736110a636d.png)

Table 44-27: IIR\_INMOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                       |
|--------------------|------------|-------------------------------------------------------------------------------|
| 15:0               | VALUE      | 16-bit Input Data Buffer Index Modifier.                                      |
| (R/W)              |            | The IIR_INMOD.VALUE bit field value is the 16-bit input data buffer modifier. |

## MAC Status Register

The IIR\_MACSTAT register indicates the status of MAC operations.

Figure 44-29: IIR\_MACSTAT Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000028_d98f1dfe4ed44c23ca435702b3075c85e1b365587482935ed5a13f06f0dc2786.png)

Table 44-28: IIR\_MACSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------|
| 14 (R/NW)          | SLEW_SINV  | Subtraction Invalid. The IIR_MACSTAT.SLEW_SINV bit indicates the subtraction is invalid.              |
| 13 (R/NW)          | SLEW_SRI   | Subtractor Result Infinity. The IIR_MACSTAT.SLEW_SRI bit indicates the subtractor result is infinity. |
| 12 (R/NW)          | SLEW_SRZ   | Subtractor Result Zero. The IIR_MACSTAT.SLEW_SRZ bit indicates the subtractor result is zero.         |
| 11 (R/NW)          | SLEW_AINV  | Addition Invalid. The IIR_MACSTAT.SLEW_AINV bit indicates the addition is invalid.                    |
| 10 (R/NW)          | SLEW_ARI   | Adder Result Infinity. The IIR_MACSTAT.SLEW_ARI bit indicates the adder result is infinity.           |

Table 44-28: IIR\_MACSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------|
| 9 (R/NW)           | SLEW_ARZ   | Adder Result Zero. The IIR_MACSTAT.SLEW_ARZ bit indicates the adder result is zero.                   |
| 8 (R/NW)           | SLEW_MINV  | Multiply Invalid. The IIR_MACSTAT.SLEW_MINV bit indicates the multiply operation is invalid.          |
| 7 (R/NW)           | SLEW_MRI   | Multiplier Result Infinity. The IIR_MACSTAT.SLEW_MRI bit indicates the multiplier result is infinity. |
| 6 (R/NW)           | SLEW_MRZ   | Multiplier Result Zero. The IIR_MACSTAT.SLEW_MRZ bit indicates the multiplier result is zero.         |
| 5 (R/NW)           | AINV       | Addition Invalid. The IIR_MACSTAT.AINV bit indicates the addition is invalid.                         |
| 4 (R/NW)           | ARI        | Adder Result Infinity. The IIR_MACSTAT.ARI bit indicates the adder result is infinity.                |
| 3 (R/NW)           | ARZ        | Adder Result Zero. The IIR_MACSTAT.ARZ bit indicates the adder result is zero.                        |
| 2 (R/NW)           | MINV       | Multiply Invalid. The IIR_MACSTAT.MINV bit indicates the multiply operation is invalid.               |
| 1 (R/NW)           | MRI        | Multiplier Result Infinity. The IIR_MACSTAT.MRI bit indicates the multiplier result is infinity.      |
| 0 (R/NW)           | MRZ        | Multiplier Result Zero. The IIR_MACSTAT.MRZ bit indicates the multiplier result is zero.              |

## Output Buffer Base Register

The IIR\_OUTBASE register contains the word address with the lower 2 bits removed.

Figure 44-30: IIR\_OUTBASE Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000029_14bc58f9d41269e4fd7d61f9caf416fa08c06faa491d632392c7ee36e1dfe39f.png)

Table 44-29: IIR\_OUTBASE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                  |
|--------------------|------------|--------------------------------------------------------------------------|
| 29:0               | VALUE      | Output Buffer Base.                                                      |
| (R/W)              |            | The IIR_OUTBASE.VALUE bit field provides the output buffer base address. |

## Output Data Buffer Index Register

The IIR\_OUTIDX register is written with word address without the lower 2 bits.

Figure 44-31: IIR\_OUTIDX Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000030_44e65b179f495972f08e0bdf5e04897d82cd9d583ca66d27ac43e801ca866367.png)

Table 44-30: IIR\_OUTIDX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                               |
|--------------------|------------|-----------------------------------------------------------------------|
| 29:0               | VALUE      | Output Data Buffer Index.                                             |
| (R/W)              |            | The IIR_OUTIDX.VALUE bit field provides the output data buffer index. |

## IIR Output Data Buffer Length Register

The IIR\_OUTLEN register provides the output data buffer length.

Figure 44-32: IIR\_OUTLEN Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000031_30c23cfa9a1120fd3f1be9d69dc541aa9e793f5fcf77391baca92db18403a187.png)

Table 44-31: IIR\_OUTLEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                |
|--------------------|------------|------------------------------------------------------------------------|
| 15:0               | VALUE      | 16-bit Output Data Buffer Length.                                      |
| (R/W)              |            | The IIR_OUTLEN.VALUE bit field provides the output data buffer length. |

## IIR Output Data Index Modifier Register

The IIR\_OUTMOD register provides the output data index modifier.

Figure 44-33: IIR\_OUTMOD Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000032_29c6a77a70ad133837ade65b974007495b6e401faf6e6ac3337564e088154052.png)

Table 44-32: IIR\_OUTMOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                        |
|--------------------|------------|--------------------------------------------------------------------------------|
| 15:0               | VALUE      | 16-bit Input Data Buffer Index Modifier.                                       |
| (R/W)              |            | The IIR_OUTMOD.VALUE bit field provides the output data buffer index modifier. |

## Software Control Register1

The IIR\_SCTL1 register is used by Smart Queuing APIs in ACM mode to exchange relevant job information amongst APIs.

Figure 44-34: IIR\_SCTL1 Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000033_25740e2e6d8f88dd5e2bbe7574b1bb6bbec7879ea74f12c35b49307d1979536d.png)

Table 44-33: IIR\_SCTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration      |
|--------------------|------------|------------------------------|
| 31:0               | VALUE      | Software Control Register 1. |

## Software Control Register2

The IIR\_SCTL2 register is used by Smart Queuing APIs in ACM mode to exchange relevant job information amongst APIs.

Figure 44-35: IIR\_SCTL2 Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000034_548640a7afd94ff2404cd250ff3ddce63e3f165f2d8e6dc4d8b61b92fd855b01.png)

Table 44-34: IIR\_SCTL2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration     |
|--------------------|------------|-----------------------------|
| 31:0               | VALUE      | Software Control Register2. |

## Secondary Global Control Register

The IIR\_SGCTL register configures the global parameters for the accelerator in ACM mode for loading CTL1 register as part of TCB.

Figure 44-36: IIR\_SGCTL Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000035_aadc404d68d0d7be8fed5817db3874862ef769be839ae12e24091492048187a5.png)

Table 44-35: IIR\_SGCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W)           | SSESEL     | Select Between Write Back or Save State Completion Interrupt.                                                                                                                                                                         |
| 14 (R/W)           | RND        | Rounding Mode Select. The IIR_SGCTL.RND bit selects the rounding mode for a floating-point mode compatible with SHARC+.                                                                                                               |
| 12 (R/W)           | FORTYBIT   | 40-Bit Floating-Point Format Select. The IIR_SGCTL.FORTYBIT bit selects the floating point format. 0 32-bit floating point                                                                                                            |
| 10 (R/W)           | SS         | Save Biquad State. The IIR_SGCTL.SS bit configures the accelerator to store the Dk register settings into the internal memory. This can be used to save the biquad states before switching to another high priority accelerator task. |

## Channel 0 Slewing Duration and Frequency

The IIR\_SLEW\_CNFG[n] registers give access to the slewing duration, slewing frequency, and counter values of the particular channel. The register has both read and write access. However, a write to this register cannot be performed during non-IDLE state of DMA.

Figure 44-37: IIR\_SLEW\_CNFG[n] Register Diagram

![Image](47_IIR_Accelerator_(IIR)_artifacts/image_000036_b5e80ef8b1453c9b58dfa7aa076c2a452fa0e8e004ea9902fd0343c29a6c8391.png)

Table 44-36: IIR\_SLEW\_CNFG[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------|
| 23:8 (R/W)         | SLEW_DRTN  | Slewing Duration. The IIR_SLEW_CNFG[n].SLEW_DRTN bit field programs number of samples processing during which slewing is done.             |
| 7:0 (R/W)          | SLEW_FREQ  | Slewing Frequency. The IIR_SLEW_CNFG[n].SLEW_FREQ bit field indicates the number of samples after which coefficients slewing is performed. |