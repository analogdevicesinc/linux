## 43   FIR Accelerator (FIR)

Finite Impulse Response (FIR) filters are frequently used in digital signal processing applications. The FIR accelerator is a dedicated hardware interface used to perform filter processing to reduce the instruction processing load on the core. FIR filters are used in a wide array of applications including multi-rate processing with an interpolator or decimator.

## Features

This hardware module can perform FIR filters without core intervention. This allows programs to use the core to implement complex algorithms, effectively adding more bandwidth to the processor.

The FIR supports the following features:

- Fixed-point and 32-bit IEEE floating-point format
- Four SHARC+ core compatible MAC units that operate in parallel
- Rounding modes compatible with SHARC+ core MACs
- Single rate or multi-rate window processing
- Programmable rates with decimation or interpolation mode
- Up to 32 filter channels available in TDM in legacy mode
- Burst transfers on data or coefficient loads

NOTE: The FIR accelerator module has a local memory that the core cannot access during regular operation. Unlike previous SHARC processors, the FIR accelerator modules each have access to the system memory (on-chip or off-chip).

Also, unlike the previous SHARC processors, where only one of the FIR or IIR accelerators can be used at a time, the SHARC+ processor can use both accelerators simultaneously.

## Clocking

The FIR accelerator runs at the speed of the core clock frequency (CCLK).

## Functional Description

The FIR Block Diagram shows the 1024-TAP FIR hardware accelerator. The accelerator consists of a 1024 word coefficient memory, a 1024 deep delay line for data, and four MAC units. The accelerator runs at /CCLK frequency.

Figure 43-1: FIR Block Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000000_2e2ba6fdcdfd41975dd6b6c47c34585f556a463cb99707ebcbe134d44b2f71a5.png)

The FIR accelerator has the following logical sub blocks:

- A datapath unit that consists of:
- A 1024 deep coefficient memory
- A 1024 deep delay line for the data
- Four 32-bit floating-point and fixed-point multiplier and adder units
- One 32-bit prefetch buffer that operates in a pipelined fashion
- One 32-bit buffer to hold the previous partial sum
- One 32-bit buffer to hold the output
- Configuration registers for the number of TAPs, number of channels, filter enable, interrupt control, DMA enable, up sample or down sample control, and ratios.
- Core access interface for writing to the DMA and filter configuration registers and reading the status register.
- DMA bus interface for transferring data and coefficients to and from the accelerator.
- DMA configuration registers including chain pointer, input, output, and coefficient registers.

## ADSP-2184x FIR Register List

The FIR accelerator is a dedicated hardware interface used to perform filter processing to reduce the instruction processing load on the core.

Table 43-1: ADSP-2184x FIR Register List

| Name          | Description                       |
|---------------|-----------------------------------|
| FIR_CHNPTR    | FIR Chain Pointer Register        |
| FIR_COEFCNT   | FIR Coefficient Count Register    |
| FIR_COEFIDX   | FIR Coefficient Index Register    |
| FIR_COEFMOD   | FIR Coefficient Modifier Register |
| FIR_CTL1      | FIR Global Control Register       |
| FIR_CTL2      | FIR Channel Control Register      |
| FIR_DBG_ADDR  | Debug Address Register            |
| FIR_DBG_CTL   | FIR Debug Control Register        |
| FIR_DBG_RDDAT | FIR Debug Data Read Register      |
| FIR_DBG_WRDAT | FIR Debug Data Write Register     |
| FIR_DMASTAT   | FIR DMAStatus Register            |
| FIR_INBASE    | FIR Input Data Base Register      |
| FIR_INCNT     | FIR Input Data Count Register     |
| FIR_INIDX     | FIR Input Data Index Register     |
| FIR_INMOD     | FIR Input Data Modifier Register  |
| FIR_MACSTAT   | FIR MAC Status Register           |
| FIR_OUTBASE   | FIR Output Data Base Register     |
| FIR_OUTCNT    | FIR Output Data Count Register    |
| FIR_OUTIDX    | FIR Output Data Index Register    |
| FIR_OUTMOD    | FIR Output Data Modifier Register |
| FIR_SCTL1     | Software Control Register 1       |
| FIR_SCTL2     | Software Control Register 2       |
| FIR_SGCTL     | Secondary Global Control Register |

## ADSP-2184x FIR Interrupt List

Table 43-2: ADSP-2184x FIR Interrupt List

|   Interrupt ID | Name      | Description        | Sensitivity   | DMA Channel   |
|----------------|-----------|--------------------|---------------|---------------|
|            108 | FIR0_DMA  | FIR0 Core1DMA      | Edge          |               |
|            109 | FIR0_STAT | FIR0 Core 1 Status | Edge          |               |
|            110 | FIR1_DMA  | FIR1 Core1DMA      | Edge          |               |

Table 43-2: ADSP-2184x FIR Interrupt List (Continued)

|   Interrupt ID | Name         | Description               | Sensitivity   | DMA Channel   |
|----------------|--------------|---------------------------|---------------|---------------|
|            111 | FIR1_STAT    | FIR1 Core 1 Status        | Edge          |               |
|            112 | FIR0_BUS_ERR | FIR0 Core 1 FIR Bus Error | Level         |               |
|            113 | FIR1_BUS_ERR | FIR1 Core 1 FIR Bus Error | Level         |               |

## ADSP-2184x FIR Trigger List

Table 43-3: ADSP-2184x FIR Trigger List Generators

|   Trigger ID | Name     | Description   | Sensitivity   |
|--------------|----------|---------------|---------------|
|           53 | FIR0_DMA | FIR0 Core1DMA | Edge          |
|           54 | FIR1_DMA | FIR1 Core1DMA | Edge          |

Table 43-4: ADSP-2184x FIR Trigger List Receivers

|   Trigger ID | Name      | Description                           | Sensitivity   |
|--------------|-----------|---------------------------------------|---------------|
|           98 | FIR0_TRGI | FIR0 Core 1 FIR Wait on Trigger Input | Pulse         |
|           99 | FIR1_TRGI | FIR1 Core 1 FIR Wait on Trigger Input | Pulse         |

## Compute Block

The MAC unit, shown in the FIR MAC Unit figure, has four multiply accumulators. The accumulators operate simultaneously on a single filter as described below.

- The MAC unit operates on the data and coefficient fetched from the data and coefficient RAMs
- Each MAC can perform 32-bit floating-point or 32-bit fixed-point MAC operations
- Floating-point format is IEEE-compliant
- Multiply and accumulation operation (addition) are pipelined
- A 32-bit floating-point MAC operation generates 32-bit multiply results
- A 32-bit fixed-point operation generates 80-bit results (64-bit result + 16 guard bits)

Figure 43-2: FIR MAC Unit

## Partial Sum Register

The partial sum register is useful for Floating-point Multi-Iteration mode. For a particular channel, the intermediate MAC result is written to the output buffer of the system memory (on-chip or off-chip). If the same channel is requested again, the partial result register is updated with the intermediate MAC result through DMA from the output buffer of the system memory. The result is added to the current MAC result after each iteration. This process repeats until all iterations complete (the entire soft filter length is processed).

## Delay Line Memory

The accelerator has a 1024 TAP delay line to hold the data locally. The DMA controller fetches the data from system memory and loads it into the delay line. Four read accesses can be made to the delay line simultaneously.

## Coefficient Memory

The accelerator has a 1024 deep coefficient memory to store the coefficients. The DMA controller loads the coefficients from system memory into coefficient memory. Four coefficients can be fetched from the coefficient memory simultaneously. If the soft filter length is more than 1024, processing happens in multi-iteration mode.

## Prefetch Data Buffer

The prefetch data buffer enables pipeline operation. One data sample is prefetched when the compute unit is operating on the delay line corresponding to the current sample. The data prefetched in this buffer is later used to update the delay line for the next sample. This operation happens in parallel again when the compute unit is not accessing the delay line. In other words, it happens when the compute unit is adding the output from the four MACs and the partial sum register.

| Cycles     | 1   | 2   | 3   | 4   | 5   | 6   |
|------------|-----|-----|-----|-----|-----|-----|
| Output DMA |     |     | N   | N1  | N2  | N3  |
| Compute    |     | N   | N1  | N2  | N3  |     |

Table 43-5: Pipeline Operation for Window Size = 1

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000001_175bfc39bac6128faa4160718d5959d79d9eae4b58a5f0386d954f54db53a0d2.png)

Table 43-5: Pipeline Operation for Window Size = 1 (Continued)

| Cycles    | 1   | 2           | 3           | 4           | 5   | 6   |
|-----------|-----|-------------|-------------|-------------|-----|-----|
| Input DMA | N   | prefetch N1 | prefetch N2 | prefetch N3 |     |     |

## Processing Output

The accelerator uses all four MACs simultaneously to calculate one output sample as shown in the Multi-Iteration Filtering Flow figure and the following procedure.

Figure 43-3: Multi-Iteration Filtering Flow

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000002_567fadbb8926f312107c9f809893f9c042f61d077352b947200197995cc68a3d.png)

1. The accelerator fetches four input data from the delay line and four corresponding coefficients from the coefficient memory and feeds them to the MAC units for multiply and accumulation operations.
2. The accelerator repeats the procedure with the next four input data and coefficients until all the TAPs complete. For example, this procedure happens N/4 times for an N TAP filter.

3. When all the TAPs are complete, the accelerator adds the four MAC outputs together to the previous partial sum (if any) to calculate the final result.
4. Finally, that output sample is stored back in system memory.

## System Memory Storage

The following sections describe the storage format for the accelerator.

CAUTION: Store all data in memory aligned to the word address boundaries. Any other programmed addresses do not flag an error.

## Coefficients and Input Buffer Storage

For any N TAP filter with coefficients:

```
C[i] i = 0,1, ... N - 1
```

store the coefficients in system memory buffer in the order:

```
C[N - 1], C[N - 2] ... C[1],C[0]
```

and CI should point to C(N - 1) .

## Single Rate Input Filtering

The total size of the input buffer must at least be equal to N - 1 + W . If the input buffer that needs to be processed is:

```
x[n],x[n+1],x[n+2] ... x[n+W-1]
```

store it in the memory as

```
x[n-(N-1)], x[n-(N-2)]
```

```
... x[n-1], x[n], x[n+1] ... x[n+W-1]
```

and the FIR\_INIDX register value should point to x[n - (N - 1)]

## Decimation

Assuming M = decimation ratio, the total size of the input buffer should at least be equal to N - 1 + W x M . If the input buffer that needs to be processed is:

```
x[n],x[n+1],x[n+2]....x[n+WxM-1],
```

store it in the memory as:

```
x[n-(N-1)], x[n-(N-2)]....x[n-1], x[n], x[n+1]....x[n+WxM-1]
```

and the FIR\_INIDX register value should point to x[n-(N-1)] .

## Interpolation

Assuming L= interpolation ratio, the total size of the input buffer should be at least equal to:

Ceil ((N-1)/L)+W/L .

If the input buffer that needs to be processed is:

```
x[n], x[n+1], x[n+2]....x[n+W/L-1], and K = Ceil((N-1)/L)
```

store it in the memory as:

```
x[n-k], x[n-(K-1)], x[n-(K-2)]....x[n-1], x[n], x[n+1].... x[n+W/L-1]
```

and the FIR\_INIDX register value should point to x[n-K] .

## Operating Modes

The FIR core performs a sum-of-products operation to compute the convolution sum. It supports single-rate, decimation, and interpolation functions.

## Single Rate Processing

In a single-rate filter, the output result rate is equal to the input sample rate. The filter output Y(n) is computed according to following equation where N is the number of filter coefficients: c(k) k = 0 to N - 1 are the filter coefficients and x(n) represents the input time-series.

<!-- formula-not-decoded -->

Figure 43-4: Filter Output Calculation

## Single Iteration

Results are computed in a single iteration when the soft filter length is less than or equal to 1024.

## Floating-point Multi-Iteration

Results are computed in multiple iterations when the soft filter length is greater than 1024 (for example, 2048 TAPs on a 1024 hard filter length). In this mode, the accelerator implements two iterations of 1024 TAPs.

NOTE: If the soft filter length is not a multiple of the hard filter length, the accelerator iterates until the soft filter length is satisfied.

Example: 550 taps on a 256 tap filter. In this example, the FIR accelerator implements two iterations of 256 taps and one iteration of 38 taps.

NOTE: Multi-iteration mode is not supported in fixed-point format.

## Window Processing

In window-based mode, multiple output samples (up to 1024) equal to the window size of that channel are calculated. After these calculations are complete, the accelerator begins processing the next channel. A configurable window size parameter is provided to specify the length of the window.

To select sample-based processing mode, configure the window size to 1. In this mode, one sample from a particular channel is processed through all the taps of that channel and the final output sample is calculated.

## Multi-Rate Processing

Multi-rate filters change the sampling rate of a signal-they convert the input samples of a signal to a different set of data that represents the same signal sampled at a different rate.

## Decimation

A decimation filter provides a single output result for every M input samples, where M is the decimation ratio. The output rate is 1/M th of the input rate. The filter implementation exploits the low output sample rate by not starting a computation until a new set of M input samples is available.

In this mode, after low-pass filtering (for anti-aliasing), FIR logic discards the ratio - 1 samples of output data. For performance optimization, FIR logic skips the computation of output samples, which are discarded.

The input buffer size for decimation filters is N - 1 + (W × M) where:

- N is the number of taps
- W is the window size
- M is the decimation ratio

The window size ( FIR\_CTL2.WINDOW bits) must be programmed with the number of output samples.

To start this mode, programs set the FIR\_CTL2.RATIO and FIR\_CTL2.UPSAMP bits (along with normal filter setting). Also, the FIR\_CTL2.TAPLEN bit field value should be greater than or equal to the FIR\_CTL2.RATIO bit field value for the decimation filter.

## Interpolation

An interpolation filter provides L output results for each new input sample, where L is the interpolation ratio. The output rate is L times the input rate.

In this mode, according to the ratio specified in configuration register, FIR logic inserts L - 1 zeros between any two input samples (upsampling). It then performs the interpolation (through the FIR filter).

Both upsampling and downsampling do not support multi-iteration mode. Therefore, the filtering operation only happens on up to 1024 TAPs and the ratio of up and downsampling can only be an integer value.

In an interpolation filter, FIR logic inserts L - 1 zeros between each sample. The program must ensure that these zeros fully shift out of the delay line before moving on to the next channel. This operation puts a restriction on window size in terms of L the sample ratio as showing in the expression:

WINDOWSIZE = n × SAMPLERATIO , where n is the number of input samples.

The input buffer size is smallest integer greater than or equal to (N - 1 + W) L for interpolation filters where:

- N is the number of taps
- W is the window size
- L is the interpolation ratio

To start the mode, programs configure the FIR\_CTL2.RATIO and FIR\_CTL2.UPSAMP bits (along with filter settings).

## Floating-Point Data Format

The FIR accelerator treats data and coefficients in 32-bit floating-point format as the default functional mode.

## Fixed-Point Data Format

In fixed-point mode, the 32-bit input data or coefficient is treated as fixed-point. A 32-bit fixed-point MAC operation generates an 80-bit result. Fixed-point data or coefficients can be unsigned integer, unsigned fractional and signed integer.

- NOTE: In fixed-point mode, the entire 80-bit result register is always written back in bursts of 3 × 32 bits. The first word is the LSW, the second word is the MSW, and the third word is a 16-bit overflow. The remaining 16 bits are padded with zeros. Therefore, for fixed-point mode: WINDOWSIZE = WINDOWSIZE × 3.

If the signed fractional format is used, the output must be scaled by 2. The MAC does not right shift to remove the redundant sign bit. A final routine must decimate the output buffer to the desired samples.

Multi-iteration mode is not supported in this format. Therefore, the maximum TAP length is 1024.

## Auto Configuration Mode (ACM)

The accelerator can be operated in legacy mode or Auto Configuration Mode (ACM), controlled by the FIR\_CTL1.ACM bit. The default functional mode is legacy mode. The accelerator mode can only be changed when the accelerator is disabled.

The ACM provides the following additional features:

- Halt and queuing

The core may pause the current Transfer Control Block (TCB) chain being processed by setting the FIR\_CTL1.HALT bit. The accelerator acknowledges the core by setting the FIR\_DMASTAT.HALT\_STAT bit. The core can appropriately take action to submit/insert new TCB's. After the core acts, the accelerator processing can be resumed by clearing the FIR\_CTL1.HALT bit. Before halting the accelerator, if the initial TCB chain is processed, the accelerator comes to the idle state. In this case, the accelerator has to be disabled and enabled by toggling the FIR\_CTL1.EN bit and the FIR\_CTL1.HALT bit must be cleared to resume processing.

- No channel number limitation

Unlike legacy mode, there is no fixed channel number limitation and, therefore, the accelerator ignores the value programmed in the FIR\_CTL1.CH field. The application can queue an unlimited number of channels/TCBs dynamically and the accelerator keeps processing the TCBs until the chain pointer becomes null.

- Selective interrupt

The core can enable/mask interrupt generation for each channel using the FIR\_CTL2.IMASK bit. If the bit is cleared, an interrupt is generated after completion of the channel.

- Selective controller / target trigger

The core can enable/mask trigger generation by the accelerator after the end of processing of each channel using the FIR\_CTL2.TMASK bit. The accelerator can also wait for a trigger after loading the TCB and coefficients and before processing a channel for which the FIR\_CTL2.TWAIT bit is set.

In addition to the above features, there are three additional fields as part of the TCB FIR\_SCTL1 , FIR\_SCTL2 , and FIR\_SGCTL registers. The FIR\_SGCTL register can be used to change the FIR\_CTL1 parameters such as rounding mode and fixed-point mode for each channel. The FIR\_SCTL1 and FIR\_SCTL2 registers can be used as general-purpose registers.

## Data Transfer

The FIR filter works exclusively through DMA.

## Chain Assignment

The structure of a TCB is conceptually the same as a traditional linked list. Each TCB has several data values and a pointer to the next TCB. Further, the chain pointer of a TCB can point to itself to continuously re-run the same DMA. The FIR accelerator reads each word of the TCB and loads it into the corresponding register. The end of the chain (no further TCBs are loaded) is indicated by a TCB with a chain pointer register value of zero.

The FIR accelerator DMA supports circular buffer chained DMA. The FIR accelerator does not support circular buffering for the coefficient buffer.

Table 43-6: TCBs for Chained DMA in Legacy Mode

| Address   | Register    |
|-----------|-------------|
| TCB       | FIR_CHNPTR  |
| TCB + 0x1 | FIR_COEFCNT |
| TCB + 0x2 | FIR_COEFMOD |
| TCB + 0x3 | FIR_COEFIDX |
| TCB + 0x4 | FIR_OUTBASE |
| TCB + 0x5 | FIR_OUTCNT  |
| TCB + 0x6 | FIR_OUTMOD  |
| TCB + 0x7 | FIR_OUTIDX  |
| TCB + 0x8 | FIR_INBASE  |
| TCB + 0x9 | FIR_INCNT   |
| TCB + 0xA | FIR_INMOD   |
| TCB + 0xB | FIR_INIDX   |
| TCB + 0xC | FIR_CTL2    |

Table 43-7: TCBs for Chained DMA in Auto Configuration Mode

| Address   | Register    |
|-----------|-------------|
| TCB       | FIR_CHNPTR  |
| TCB + 0x1 | FIR_SCTL1   |
| TCB + 0x2 | FIR_SCTL2   |
| TCB + 0x3 | FIR_SGCTL   |
| TCB + 0x4 | FIR_COEFCNT |
| TCB + 0x5 | FIR_COEFMOD |
| TCB + 0x6 | FIR_COEFIDX |
| TCB + 0x7 | FIR_OUTBASE |
| TCB + 0x8 | FIR_OUTCNT  |
| TCB + 0x9 | FIR_OUTMOD  |
| TCB + 0xA | FIR_OUTIDX  |
| TCB + 0xB | FIR_INBASE  |
| TCB + 0xC | FIR_INCNT   |
| TCB + 0xD | FIR_INMOD   |
| TCB + 0xE | FIR_INIDX   |
| TCB + 0xF | FIR_CTL2    |

The FIR\_COEFCNT register is loaded with the values in the FIR\_COEFCNT TCB field and is decremented from that value onwards. However, coefficient loading continues until the number of coefficients, equal to the tap length, are read. This condition is true even if the FIR\_COEFCNT register reaches zero as in the case of a tap length = 10, and the FIR\_COEFCNT bit field in the TCB is initialized to 0. The value in the FIR\_COEFCNT register is -10 after all coefficients are loaded.

NOTE: Initialize the FIR\_CHNPTR register to TCB + 12 in legacy mode and TCB + 15 in ACM mode.

## DMA Access

The FIR accelerator has two DMA channels (accelerator input and output) to connect to the system memory. The DMA controller fetches the data and coefficients from memory and stores the result.

## Burst Access Support

Burst access enhances the throughput of the DMA channel and reduces the overall load on the system fabric. Burst support is provided for TCB, data, and coefficient loads on the DMA channel. The FIR module supports burst transfers of size SINGLE, INCR4 INCR8 and INCR16. The DMA access are automatically split into these four types of burst by the FIRA accelerator for optimum performance. For a TCB load, a burst access of length 16 is issued which reduces the TCB load time.

An additional bit ( FIR\_CTL1.BURSTEN ) enables the burst feature. Burst transfers are always of size SINGLE when the modifier is other than 1. In cases of burst transfers around the circular buffer boundary, the design ensures that the burst does not cross the buffer boundary.

## Data and Coefficient Load in Parallel

To reduce the overall loading time of data and coefficients, the operation occurs over separate channels. After the TCB load, both data and coefficients are loaded in parallel over two DMA channels (CH0 and CH1). If there is no conflict in the system when accessing the data and coefficients, the loading time is significantly improved over when loaded sequentially using a single channel. This mode is enabled by default and can be disabled by setting the FIR\_CTL1.DCP\_DIS bit. When disabled, coefficient load and data load occur sequentially over a single channel (CH0).

## Prefetch Buffer for DMA Channels

An access latency exists in every burst transfer for the address issued for the first data to arrive. T o reduce the latency, both channels of the FIR contain a prefetch buffer. When one burst access is in-progress, the FIR prefetch buffer receives the next burst, which reduces the latency for consecutive accesses. The FIR prefetch buffer is disabled by default. Setting the FIR\_CTL1.PFB\_EN bit enables the prefetch buffer.

## Accelerator TCB

The location of the DMA parameters for the next sequence comes from the chain pointer register that points to the next set of DMA parameters stored in the internal memory of the processor. In chained DMA operations, the

accelerator automatically initializes and starts another DMA transfer when the current DMA transfer is complete. Each new set of parameters is stored in a user-initialized memory buffer or TCB for a chosen peripheral.

## Chain Pointer DMA

The DMA controller supports circular buffer chain pointer DMA. One TCB must be configured for each channel. The DMA controller contains the following:

- A control register value to configure the filter parameters (such as filter tap length, window size, sample rate conversion settings) for each channel. In ACM mode, additional parameters such as interrupt mask, trigger mask, and trigger wait are also available.
- Software control register values in ACM mode for each channel.
- Secondary control register value to configure rounding mode, fixed-point mode, and two's complement for each channel.
- DMA parameter register values for the input data (delay line).
- DMA parameter register values for coefficient load.
- DMA parameter register values for output data.

Intermediate results in multi-iteration mode are saved in the output buffer.

As shown in the Circular Buffer Addressing figure, the accelerator loads the TCB into its internal registers and uses these values to fetch coefficients and data and to store results. After processing a window of data for any channel, the accelerator writes back the appropriate values to the FIR\_INIDX and FIR\_OUTIDX bit fields of the TCB in memory. Then, data processing can begin from where it left off during the next time slot of that channel.

The write-back value for input buffer is:

- FIR\_INIDX + W for single rate filtering
- FIR\_INIDX + W × M for decimation (M = decimation ratio)
- FIR\_INIDX + W/L for interpolation (L = interpolation ratio)
- The write-back value for output buffer in floating point mode is: FIR\_OUTIDX + W
- The write-back value for output buffer in fixed-point mode is: FIR\_OUTIDX + 3 × W

NOTE: The FIR\_CTL2 register is part of the FIR TCB. This configuration allows programming individual FIR channels with different control attributes.

The above index updates are valid only for legacy mode. In ACM mode, the FIR\_INIDX and FIR\_OUTIDX bit fields of the TCB in memory are updated to 0x00000000 and 0xFFFFFFFF by the accelerator after processing a window of data.

In ACM mode, when the FIR\_CTL1.SMQ\_LIUPS\_EN bit is set, the accelerator updates the FIR\_INIDX and FIR\_OUTIDX bit fields of the TCB in memory after processing a window of data

and according to the circular buffer scheme. When the FIR\_CTL1.SMQ\_LIUPS\_EN bit is cleared, the accelerator updates the FIR\_INIDX and FIR\_OUTIDX bit fields of the TCB in memory to 0x00000000 and 0xFFFFFFFF after processing a window of data. The FIR\_CTL1.SMQ\_LIUPS\_EN bit is only valid in ACM mode.

Figure 43-5: Circular Buffer Addressing

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000003_15eee9ae9792ba844199a46117d9e221b2c9de2189472bc49c0137ce5d7bcdf1.png)

## Programming Model

The following sections provide general programming information for the FIR accelerator.

## Legacy Mode

The Wait for Core Intervention ≥ Idle (if auto channel iterate bit = 0) figure shows the diagram for multichannel filtering. Multiple channels are processed in a time division multiplexed (TDM) format. After completing all the channels, the accelerator can either repeat the slots or wait for core intervention.

For multichannel filtering, use the following steps.

1. Program the number of channels using the FIR\_CTL1.CH bits.
2. Configure the TCBs in system memory with one channel's TCB pointing to the next channel's TCB.
3. Write the first TCB value into the FIR\_CHNPTR register and enable the accelerator.
- a. The accelerator fetches the first channel's TCB and, using it as pointer, prefills the delay line and coefficient memory and loads the FIR\_CTL2 register to configure the filter parameters corresponding to that channel.
- b. The accelerator then calculates output samples corresponding to one window and stores the data back in internal memory.
- c. At the end of the window the accelerator updates the FIR\_INIDX and FIR\_OUTIDX registers in the TCB of system memory and moves to the next channel.
- d. When all the channels are finished and the auto channel iterate bit ( FIR\_CTL1.CAI ) =1, the accelerator processes the first channel again and iterates through the channels. If the FIR\_CTL1.CAI bit =0, the accelerator waits for core intervention.

Figure 43-6: Wait for Core Intervention ≥ Idle (if auto channel iterate bit = 0)

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000004_164df0e91291b96085977ff8287a393e0e010f9e8272e1942f7bb53e5eceaaee.png)

NOTE: All the addresses programmed in the TCB correspond to 32-bit address boundaries and should not contain the lower 2 bits (assumed as zeros).

## Auto Configuration Mode (ACM)

The figure shows multi-channel filtering in ACM. Multiple channels are processed in a time multiplexed format (TDM).

Figure 43-7: Multi-Channel Filtering in Auto Configuration Mode

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000005_1c4564c3e4af466a4fe0008463f4bb27374a9438e04d3fa67d89ab5d2d4d9351.png)

For multi-channel filtering:

1. Configure the Transfer Control Blocks (TCBs) in system memory with one channel's TCB pointing to the next channel's TCB. There is no limit on the number of channels to be configured.

2. Write the first TCB value to the FIR\_CHNPTR register and enable the accelerator.

The accelerator fetches the first channel's TCB. Using it as pointer, it prefills the delay line and coefficient memory and loads the FIR\_CTL2 and FIR\_SGCTL registers to configure the filter parameters corresponding to that channel.

When the FIR\_CTL2.TWAIT bit is set, the accelerator waits for a input trigger to start the window processing for the channel.

The accelerator then calculates output samples corresponding to one window and stores the data back in internal memory. When the FIR\_CTL2.TMASK bit is cleared the accelerator sends an output trigger after completion of processing of the channel.

When the FIR\_CTL2.IMASK bit is cleared, the accelerator interrupts the core after completion of processing of a particular channel.

At the end of the window, the accelerator updates the FIR\_INIDX and FIR\_OUTIDX registers to 0x00000000 and 0xFFFFFFFF in the TCB of system memory and moves to the next channel.

3. At any instant, as required, the core halts the accelerator. It sets the FIR\_CTL1.HALT bit and appropriately takes action to submit or insert new TCBs and clears the FIR\_CTL1.HALT bit to resume channel processing.

If the FIR\_CHNPTR register is zero (last channel is being processed or channel processing is complete) after halting the accelerator, the FIR\_CHNPTR register is appropriately written before resuming the accelerator channel processing.

When the accelerator is idle after halting, FIR\_CTL1.EN bit is toggled to disable and re-enable the accelerator and the FIR\_CTL1.HALT bit is cleared, and the accelerator resumes channel processing.

4. The accelerator continues processing until all the channels are complete. Repeat Step 3 (if required) to submit/insert new channels.

NOTE: Channel auto iterate ( FIR\_CTL1.CAI ) is not supported in ACM.

## Debug Mode

The next sections show the steps required for reading and writing local memory in debug mode.

## Write to Local Memory

1. Clear the FIR\_CTL1.DMAEN bit.
2. Set the FIR\_DBG\_CTL.EN , FIR\_DBG\_CTL.MEM , and FIR\_DBG\_CTL.HLD bits.
3. Set the FIR\_DBG\_CTL.ADRINC bit for address auto increment.
4. Write the start address to the FIR\_DBG\_ADDR register.

NOTE: If bit 11 in the FIR\_DBG\_ADDR register is set, coefficient memory is selected.

5. Write data to the FIR\_DBG\_WRDAT register.

## Read from Local Memory

1. Clear the FIR\_CTL1.DMAEN bit.
2. Set the FIR\_DBG\_CTL.EN , FIR\_DBG\_CTL.MEM , and FIR\_DBG\_CTL.HLD bits.
3. Set the FIR\_DBG\_CTL.ADRINC bit for address auto increment.
4. Write the start address to the FIR\_DBG\_ADDR register.

NOTE: If bit 11 in the FIR\_DBG\_ADDR register is set, coefficient memory is selected.

5. Read data from the FIR\_DBG\_RDDAT register.

## Single-Step Mode

Single-step mode can be used for debug purposes. An extra debug register is used in this mode.

1. Enable stop DMA during breakpoint hit in the emulator settings.
2. Clear the FIR\_DBG\_CTL.HLD bit and enable the FIR\_DBG\_CTL.EN and FIR\_DBG\_CTL.RUN bits.
3. Program the FIR module according to the application.
4. In single-step mode, each iteration is updated in the emulator session.

## Computing FIR Output, Tap Length Greater than 4096

With little core intervention, the FIR accelerator can also be used to calculate output for a tap length greater than 4096 taps. The section demonstrates the calculation with an example of 8192 taps.

1. Divide the transfer function of an 8192 FIR filter into two 4096 FIR filters:

<!-- formula-not-decoded -->

2. Divide the filter coefficients of an 8192 tap filter among two 4096 tap FIR filters:

Filter 1

```
Coefficients = b0, b1, b2,……, b4095 Input data = x[n], x [n - 1],……….x[n - 4095] Filter 2 Coefficients = b4096, b4097,…………..,b8191 Input data = x[n - 4096], x[n - 4097],……x[n - 8191]
```

The accelerator can be used in two-channel mode where:

- channel 1 operates on x[n]…x[n - 4095] input data with the filter coefficients of filter 1 and

- channel 2 operates on x[n - 4096]…x[n - 8191] with the filter coefficients of filter 2.

Once both the channels are processed, add the partial sum output of both the channels to get the final output. Implement this approach (tap length = TAPS = 8192, window size = WINDOW) using the following programming steps.

1. Create a circular input data buffer in system memory (IBUF). The buffer must be large enough to avoid overwriting data before the accelerator processes it. Ideally, the input buffer size for a channel is TAPS + WINDOW - 1.
2. Create a coefficient buffer of size TAPS (8192) (CBUF).
3. Create one output buffer of size WINDOW (OBUF) and another temporary output buffer (OBUF1) to store the partial sum.
4. Create two TCBs in system memory with first TCB chained to the second and second chained to the first in a circular manner.
- a. Point the FIR\_COEFIDX bit field of the first TCB to the start address of the coefficient buffer (CBUF) and that of the second TCB to 4096 offset from the start of the coefficient buffer (CBUF + 4096).
- b. Point the FIR\_OUTBASE and FIR\_OUTIDX bit fields of the first TCB to the start address of OBUF and that of the second TCB to the start address of OBUF1.
- c. Point the FIR\_INIDX bit field of the first TCB to the start address of IBUF and that of the second TCB to 4096 offset from the start address of IBUF .
- d. Configure the FIR\_CTL2 bit field of both the TCB for tap length = TAP/2 = 4096 and window size = WINDOW.
5. Initialize the FIR\_CHNPTR register to point to the first TCB.
6. Program the FIR\_CTL1 register to initiate the accelerator processing by setting the FIR\_CTL1.EN and FIR\_CTL1.DMAEN bits and the number of channels configured as 2.
7. Wait for the FIR all channels done interrupt ( FIR\_DMASTAT.ACDONE ) to occur. Inside the ISR, add the partial sum results using the core from both the output buffers (OBUF and OBUF1) to get the final output. To save memory, replace the contents of the buffer OBUF with the final output result.

## Debug Features

The following sections provide information for debugging the FIR accelerator.

## Local Memory Access

The contents of FIR delay line and coefficient memories are made observable for debug by setting the FIR\_DBG\_CTL.EN / FIR\_DBG\_CTL.MEM and FIR\_DBG\_CTL.HLD bits. The debug address register ( FIR\_DBG\_ADDR ) and two data registers are provided for debug operations. Bit 11 of the FIR\_DBG\_ADDR register selects coefficient memory when set (=1) and selects delay line memory when cleared (=0).

In the debug mode, the read data register ( FIR\_DBG\_RDDAT ) returns the contents of the memory location pointed to by the address register. Data can be written into any memory location using FIR\_DBG\_WRDAT register writes. If the address auto-increment bit ( FIR\_DBG\_CTL.ADRINC ) is set, the address register auto-increments on FIR\_DBG\_WRDAT writes and FIR\_DBG\_RDDAT reads. During auto-increment the FIR\_DBG\_ADDR register cannot cross the data memory or coefficient memory boundary.

## Single-Step Mode

Programs can single step through the MAC operations and observe the memory contents after each step. The FIR\_DBG\_CTL.EN , FIR\_DBG\_CTL.HLD , and FIR\_DBG\_CTL.MEM bits control the FIR MAC units.

## Emulation Considerations

In FIR debug mode, the DMA operations are not observable.

## Interrupts

The FIR Interrupt Overview table provides the source of interrupt and service instructions for the FIR interrupts.

Table 43-8: FIR Interrupt Overview

| Accelerator Mode        | Default Programmable Interrupt   | Sources                            | Masking        | Service                                |
|-------------------------|----------------------------------|------------------------------------|----------------|----------------------------------------|
| Legacy Mode             | FIR_DMA                          | Window Complete                    | N/A            | ROC from FIR_DMASTAT + RTI instruction |
| Legacy Mode             | FIR_DMA                          | All channels complete              | N/A            | ROC from FIR_DMASTAT + RTI instruction |
| Legacy Mode             | FIR_STAT                         | MAC IEEE floating point exceptions | N/A            | ROC from FIR_MACSTAT + RTI instruction |
| Legacy Mode             | FIR_STAT                         | MAC fixed point over- flow         | N/A            | ROC from FIR_MACSTAT + RTI instruction |
| Auto Configuration Mode | FIR_DMA                          | Window Complete                    | FIR_CTL2.IMASK | ROC from FIR_DMASTAT + RTI instruction |
| Auto Configuration Mode | FIR_STAT                         | MAC IEEE floating point exceptions | N/A            | ROC from FIR_MACSTAT + RTI instruction |
| Auto Configuration Mode | FIR_STAT                         | MAC fixed point over- flow         | N/A            | ROC from FIR_MACSTAT + RTI instruction |

## Sources

The FIR module drives two interrupt signals: FIR\_DMA for the DMA status and FIR\_STAT for the MAC status. The FIR module generates interrupts as described in the following sections.

## Window Complete

This interrupt is generated at the end of each channel when all the output samples are calculated corresponding to a window and updated index values are written back.

In legacy mode, if the FIR\_CTL1.CCINTR bit is set, an interrupt is generated after completion of window processing of each channel. In ACM, the interrupt generation can be selectively masked using the FIR\_CTL2.IMASK bit for each channel.

## All Channels Complete

This interrupt is generated when all the channels are complete or when one iteration of time slots completes. Note that the interrupt follows the access completion rule, where the interrupt is generated when all data are written back to system memory.

The all channels complete interrupt source is only applicable in legacy mode. If the FIR\_CTL1.CCINTR bit is not set in legacy mode, the all channel complete interrupt is generated. In ACM mode, interrupt generation for each channel is controlled using the FIR\_CTL2.IMASK bit.

NOTE: The FIR\_CTL1.CCINTR bit is valid only in legacy mode. In ACM mode, interrupt generation for each channel is controlled using the FIR\_CTL2.IMASK bit.

## MAC Status

A MAC status interrupt is generated under the following conditions and is reflected in the FIR\_MACSTAT register.

- Multiplier result zero - Set if multiplier result is zero
- Multiplier result infinity - Set if multiplier result is infinity
- Multiply invalid - Set if multiply operation is invalid
- Adder result zero - Set if adder result is zero
- Adder result infinity - Set if adder result is infinity
- Adder invalid - Set if addition is invalid
- Adder overflow - for fixed-point operation

## Service

The DMA interrupt status bits are sticky and are cleared when the DMA status register is read. When a MAC status interrupt occurs, programs can find this state by reading the MAC status register ( FIR\_MACSTAT ). The read of the register clears the (sticky) MAC interrupt status bits.

The status interrupt sources are derived from the FIR\_MACSTAT register. A status interrupt can occur due to the last set of MAC operations of a processing iteration that correspond to a particular channel. The interrupt is generated continuously and cannot be stopped, even after disabling the accelerator. The interrupt can only be stopped when another processing iteration results in a non-zero or valid multiply or add result.

## ADSP-2184x FIR Register Descriptions

The FIR accelerator (FIR) contains the following registers.

Table 43-9: ADSP-2184x FIR Register List

| Name          | Description                       |
|---------------|-----------------------------------|
| FIR_CHNPTR    | FIR Chain Pointer Register        |
| FIR_COEFCNT   | FIR Coefficient Count Register    |
| FIR_COEFIDX   | FIR Coefficient Index Register    |
| FIR_COEFMOD   | FIR Coefficient Modifier Register |
| FIR_CTL1      | FIR Global Control Register       |
| FIR_CTL2      | FIR Channel Control Register      |
| FIR_DBG_ADDR  | Debug Address Register            |
| FIR_DBG_CTL   | FIR Debug Control Register        |
| FIR_DBG_RDDAT | FIR Debug Data Read Register      |
| FIR_DBG_WRDAT | FIR Debug Data Write Register     |
| FIR_DMASTAT   | FIR DMAStatus Register            |
| FIR_INBASE    | FIR Input Data Base Register      |
| FIR_INCNT     | FIR Input Data Count Register     |
| FIR_INIDX     | FIR Input Data Index Register     |
| FIR_INMOD     | FIR Input Data Modifier Register  |
| FIR_MACSTAT   | FIR MAC Status Register           |
| FIR_OUTBASE   | FIR Output Data Base Register     |
| FIR_OUTCNT    | FIR Output Data Count Register    |
| FIR_OUTIDX    | FIR Output Data Index Register    |
| FIR_OUTMOD    | FIR Output Data Modifier Register |
| FIR_SCTL1     | Software Control Register 1       |
| FIR_SCTL2     | Software Control Register 2       |
| FIR_SGCTL     | Secondary Global Control Register |

## FIR Chain Pointer Register

The FIR\_CHNPTR register contains the chain pointer address.

Figure 43-8: FIR\_CHNPTR Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000006_356f58b18f36556d82ea35a0a2e0c811818a8bcffc5fe5ff720f5ae5648bfd10.png)

Table 43-10: FIR\_CHNPTR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                            |
|--------------------|------------|--------------------------------------------------------------------|
| 29:0               | VALUE      | Chain Pointer Address.                                             |
| (R/W)              |            | The FIR_CHNPTR.VALUE bit field contains the chain pointer address. |

## FIR Coefficient Count Register

The FIR\_COEFCNT register contains the 16-bit coefficient buffer count.

Figure 43-9: FIR\_COEFCNT Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000007_3c5a99db18753193a1e2c16a61e28a758cfcd21892835fa1bc1bd4905d3b03f8.png)

Table 43-11: FIR\_COEFCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                      |
|--------------------|------------|------------------------------------------------------------------------------|
| 15:0               | CCNT       | 16-bit Coefficient Buffer Count.                                             |
| (R/W)              |            | The FIR_COEFCNT.CCNT bit field contains the 16-bit coefficient buffer count. |

## FIR Coefficient Index Register

The FIR\_COEFIDX register contains the coefficient index word address with the lower two bits removed.

Figure 43-10: FIR\_COEFIDX Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000008_60ab0358aef248e57057853e41e631c984402d9234071cc6dbb0bedc9de24099.png)

Table 43-12: FIR\_COEFIDX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------|
| 29:0 (R/W)         | VALUE      | Word Addresses with Lower 2 Bits Removed. The FIR_COEFIDX.VALUE bit field contains the word addresses with the lower 2 bits removed. |

## FIR Coefficient Modifier Register

The FIR\_COEFMOD register contains the 16-bit coefficient index modifier.

Figure 43-11: FIR\_COEFMOD Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000009_514e5b426f086f397d4311157a2dfab9bc78532ffee5ec8ebbfcd0b4ae96a4f3.png)

Table 43-13: FIR\_COEFMOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                         |
|--------------------|------------|---------------------------------------------------------------------------------|
| 15:0               | VALUE      | 16-bit Coefficient Index Modifier.                                              |
| (R/W)              |            | The FIR_COEFMOD.VALUE bit field contains the 16-bit coefficient index modifier. |

## FIR Global Control Register

The FIR\_CTL1 register is used to configure the global parameters for the accelerator. These parameters include the number of channels, channel auto iterate, DMA enable, and accelerator enable.

Figure 43-12: FIR\_CTL1 Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000010_a865416f120fc288368c7b5af722a4c1a7d4e6dfaf0b59a0ba52e7715fb04634.png)

Table 43-14: FIR\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------|
| 31 (R/W)           | ACM        | Auto Configuration Mode. The FIR_CTL1.ACM bit configures the mode for loading the TCB. |

Table 43-14: FIR\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                                                                                                    |
|--------------------|--------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30 (R/W)           | HALT         | HALT (Pause Accelerator). This bit is only valid in Auto Configuration mode (ACM) 0 Release. Release accelerator for further processing of da-                                                                                                                                                                                             |
| 29 (R/W)           | BURST16_DIS  | Disable 16 Burst. The FIR_CTL1.BURST16_DIS bit configures the accelerator to disable 16 burst when in burst mode. 0 Maximum burst of INCR16 in burst mode forDMA                                                                                                                                                                           |
| 28 (R/W)           | DCP_DIS      | Disable Data Coefficient Load in Parallel. The FIR_CTL1.DCP_DIS bit enables data and coefficient loading to occur in parallel. 0 Enable                                                                                                                                                                                                    |
| 27 (R/W)           | PFB_EN       | Enable PFB. When set (=1), the FIR_CTL1.PFB_EN bit enables the prefetch buffer on both FIR channels. 0 Disable 1 Enable                                                                                                                                                                                                                    |
| 26 (R/W)           | SMQ_LIUPS_EN |                                                                                                                                                                                                                                                                                                                                            |
|                    |              | Legacy I/P and O/P Index Update Scheme in SMART_Q Mode. The FIR_CTL1.SMQ_LIUPS_EN bit configures the scheme the accelerator uses to update the input index (II) and output index (OI). This bit is only valid in ACM. 0 Update the II with all '0' and the OI with all 'F', respec- tively 1 Update II and OI after circular buffer scheme |
| 14 (R/W)           | RND          | Rounding Mode. The FIR_CTL1.RND bit configures the accelerator to use a rounding mode. 0 Round to Nearest                                                                                                                                                                                                                                  |

Table 43-14: FIR\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | TC         | Two's-Complement. The FIR_CTL1.TC bit configures the accelerator to use either unsigned integer or signed integer                                                                                                                                                                                      |
| 13 (R/W)           | TC         | 0 Unsigned Integer                                                                                                                                                                                                                                                                                     |
| 13 (R/W)           | TC         | 1 Signed Integer                                                                                                                                                                                                                                                                                       |
| 12 (R/W)           | FXD        | Fixed-Point Accelerator Select. The FIR_CTL1.FXD bit configures the accelerator to use either 32-bit IEEE float- ing-point or 32-bit fixed-point.                                                                                                                                                      |
| 12 (R/W)           | FXD        | 0 32-bit IEEE Floating-Point                                                                                                                                                                                                                                                                           |
| 12 (R/W)           | FXD        | 1 32-bit Fixed Point                                                                                                                                                                                                                                                                                   |
| 11 (R/W)           | CCINTR     | Channel Complete Interrupt. The FIR_CTL1.CCINTR bit configures the accelerator to generate an interrupt when each or all channels are done. This bit is only valid in Legacy Mode. In Auto Configuration Mode (ACM), interrupt generation for each channel is controlled using the FIR_CTL2.IMASK bit. |
| 11 (R/W)           | CCINTR     | 0 Interrupt is Generated Only When All Channels are Done                                                                                                                                                                                                                                               |
| 11 (R/W)           | CCINTR     | 1 Interrupt is Generated After Each Channel is Done                                                                                                                                                                                                                                                    |
| 9 (R/W)            | CAI        | Channel Auto Iterate. The FIR_CTL1.CAI bit, if cleared, causes the TDMprocessing to stop (idle) once all channels are done. If set, processing moves to the first channel and continuesTDM processing in a loop when all channels are done. Channel Auto Iterate is not available                      |
| 9 (R/W)            | CAI        | 0 TDMProcessing Stops (IDLE) Once All Channels are Over                                                                                                                                                                                                                                                |
| 9 (R/W)            | CAI        | 1 Moves to First Channel and Continues TDMProcess- ing in a Loop When All Channels are Over                                                                                                                                                                                                            |
| 8 (R/W)            | DMAEN      | DMAEnable. The FIR_CTL1.DMAEN bit enables and disables DMAon the FIR accelerator.                                                                                                                                                                                                                      |
| 8 (R/W)            | DMAEN      | 0 DMADisabled                                                                                                                                                                                                                                                                                          |
| 8 (R/W)            | DMAEN      | 1 DMAEnabled                                                                                                                                                                                                                                                                                           |
| 6 (R/W)            | BURSTEN    | Burst Mode Select. When the FIR_CTL1.BURSTEN bit is set, burst mode is enabled for coefficient loads and data loads. When cleared, burst mode is disabled. By default, burst mode is disabled.                                                                                                         |

Table 43-14: FIR\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                    | Description/Enumeration                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5:1 (R/W)          | CH         | Number of Channels. The FIR_CTL1.CH bit field configures the number of channels and is programma- ble from 0 to 31. This bit field is only valid in Legacy Mode. There is no channel number limitation in Auto Configuration Mode (ACM) as the accelerator keeps processing the TCBs until the chain pointer becomes null. | Number of Channels. The FIR_CTL1.CH bit field configures the number of channels and is programma- ble from 0 to 31. This bit field is only valid in Legacy Mode. There is no channel number limitation in Auto Configuration Mode (ACM) as the accelerator keeps processing the TCBs until the chain pointer becomes null. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                          | Channel 1                                                                                                                                                                                                                                                                                                                  |
|                    |            | 1-30                                                                                                                                                                                                                                                                                                                       | Channel 2-31                                                                                                                                                                                                                                                                                                               |
|                    |            | 31                                                                                                                                                                                                                                                                                                                         | Channel 32                                                                                                                                                                                                                                                                                                                 |
| 0                  | EN         | FIR Enable.                                                                                                                                                                                                                                                                                                                | FIR Enable.                                                                                                                                                                                                                                                                                                                |
| (R/W)              |            | The FIR_CTL1.EN bit enables and disables the FIR accelerator.                                                                                                                                                                                                                                                              | The FIR_CTL1.EN bit enables and disables the FIR accelerator.                                                                                                                                                                                                                                                              |
|                    |            | 0                                                                                                                                                                                                                                                                                                                          | Disable FIR                                                                                                                                                                                                                                                                                                                |
|                    |            | 1                                                                                                                                                                                                                                                                                                                          | Enable FIR                                                                                                                                                                                                                                                                                                                 |

## FIR Channel Control Register

The FIR\_CTL2 register is used to configure the channel specific parameters such as filter TAP length, window size, sample rate conversion, up/down sampling and ratio. In Auto Configuration Mode(ACM), this register is also used to configure additional channel specific parameters like interrupts and triggers.

Figure 43-13: FIR\_CTL2 Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000011_4db033071c83acc21610eef8a8a56eac4803d942f7469df6465d4aeac8b58dfd.png)

Table 43-15: FIR\_CTL2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | TMASK      | Trigger Mask. The FIR_CTL2.TMASK bit enables trigger generation for the channel. It is only valid in Auto Configuration Mode (ACM). 0 Enable |
| 30 (R/W)           | UPSAMP     | Up Sampling Enable. The FIR_CTL2.UPSAMP bit enables up sampling. 0 Down Sampling 1 Up Sampling                                               |

Table 43-15: FIR\_CTL2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W)           | SRCEN      | Sample Rate Conversion Enable. The FIR_CTL2.SRCEN bit field enables sample rate conversion.                                                                         |
| 29 (R/W)           | SRCEN      | 0 Disabled                                                                                                                                                          |
| 29 (R/W)           | SRCEN      | 1 Enabled                                                                                                                                                           |
| 28 (R/W)           | TWAIT      | Wait for Trigger. The FIR_CTL2.TWAIT bit disables the wait for the trigger. It is only valid in Auto Configuration Mode (ACM).                                      |
| 28 (R/W)           | TWAIT      | 0 Disable wait for trigger for the channel                                                                                                                          |
| 28 (R/W)           | TWAIT      | 1 Enable wait for the external trigger assertion                                                                                                                    |
| 27:25 (R/W)        | RATIO      | Up/Down Sampling Ratio. The FIR_CTL2.RATIO bit field sets the sampling ratio ( FIR_CTL2.RATIO + 1).                                                                 |
| 24 (R/W)           | IMASK      | Interrupt Mask. The FIR_CTL2.IMASK bit enables interrupt generation for the channel. This bit is only valid in Auto Configuration Mode (ACM).                       |
| 24 (R/W)           | IMASK      | 0 Enable                                                                                                                                                            |
| 23:14 (R/W)        | WINDOW     | Window Size. The FIR_CTL2.WINDOW bit field sets the window size which specifies the number of sample/block to process (sample based processing = window size of 0). |
| 13:12 (R/W)        | PRIO       | Priority Level. The FIR_CTL2.PRIO bit field indicates the priority.                                                                                                 |
| 13:12 (R/W)        | PRIO       | 0 Level 0 (lowest)                                                                                                                                                  |
| 13:12 (R/W)        | PRIO       | 1 Level 1                                                                                                                                                           |
| 13:12 (R/W)        | PRIO       | 2 Level 2                                                                                                                                                           |
| 13:12 (R/W)        | PRIO       | 3 Level 3 (highest)                                                                                                                                                 |
| 11:0 (R/W)         | TAPLEN     | Tap Length. The FIR_CTL2.TAPLEN bit field sets the tap length which is programmable be- tween 0-4095 (Tap Length = FIR_CTL2.TAPLEN + 1).                            |

## Debug Address Register

The FIR\_DBG\_ADDR register holds the debug address.

Figure 43-14: FIR\_DBG\_ADDR Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000012_6df0991ebcb129c9810879e5984fe783d00206a46e867c958bc08f8017f3c0fa.png)

Table 43-16: FIR\_DBG\_ADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10:0 (R/W)         | VALUE      | Debug Address, Coefficient Memory Select. The FIR_DBG_ADDR.VALUE bit field holds the debug address (bits 0-10). Bit 11 configures whether the memory access is to coefficient memory (=0) or to delay line memory (=1). |

## FIR Debug Control Register

The FIR\_DBG\_CTL register controls debugging operations such as enabling debug mode running, hold or single stepping.

Figure 43-15: FIR\_DBG\_CTL Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000013_bd346aea7faa9fab0abd0c72a6ba49d4432c41facb04ca2c7b3e2bcb2cb84ab3.png)

Table 43-17: FIR\_DBG\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W)            | ADRINC     | Address Auto Increment. The FIR_DBG_CTL.ADRINC bit bit allows the address register to auto-increment on FIR_DBG_WRDAT writes and FIR_DBG_RDDAT reads. |
| 4 (R/W)            | MEM        | Local Memory Access. When the FIR_DBG_CTL.MEM bit is set, the data and coefficients memory can be indirectly accessed.                                |
| 2 (R/W1S)          | RUN        | Release MAC. The FIR_DBG_CTL.RUN bit releases the MAC. This bit is self-clearing after one FIR clock cycle.                                           |
| 1 (R/W)            | HLD        | Hold. The FIR_DBG_CTL.HLD bit holds or single-steps through the FIR.                                                                                  |
| 1 (R/W)            | HLD        | 0 Hold                                                                                                                                                |
| 1 (R/W)            | HLD        | 1 Single-step                                                                                                                                         |
| 0 (R/W)            | EN         | Debug Mode Enable. The FIR_DBG_CTL.EN bit enables debug mode. For local memory access, the FIR_CTL1 register can be cleared. Mode                     |
| 0 (R/W)            | EN         | 0 Disable Debug                                                                                                                                       |
| 0 (R/W)            | EN         | 1 Enable Debug Mode                                                                                                                                   |

## FIR Debug Data Read Register

The FIR\_DBG\_RDDAT register hold the debug read data.

Figure 43-16: FIR\_DBG\_RDDAT Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000014_2a6e86c5918498346dd15549006760e562c992e6592f061d17c4dfa03aa88330.png)

Table 43-18: FIR\_DBG\_RDDAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                      |
|--------------------|------------|--------------------------------------------------------------|
| 31:0               | VALUE      | Debug Read Data.                                             |
| (R/W)              |            | The FIR_DBG_RDDAT.VALUE bit field holds the debug read data. |

## FIR Debug Data Write Register

The FIR\_DBG\_WRDAT register holds the debug write data.

Figure 43-17: FIR\_DBG\_WRDAT Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000015_3ab9a6f9e0c448bdd5bf5d39d52643ac41892931c900e78938c2bca656edf0c1.png)

Table 43-19: FIR\_DBG\_WRDAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                       |
|--------------------|------------|---------------------------------------------------------------|
| 31:0               | VALUE      | Debug Write Data.                                             |
| (R/W)              |            | The FIR_DBG_WRDAT.VALUE bit field holds the debug write data. |

## FIR DMA Status Register

The FIR\_DMASTAT register provides information about chain pointer loading, coefficient DMA, data preload DMA, processing in progress, window complete, all channels complete.

Figure 43-18: FIR\_DMASTAT Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000016_d5e04a1c4275989f456cf5928297b8fbf86268b33d85d876380af77025a7a7de.png)

Table 43-20: FIR\_DMASTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/NW)          | HALT_STAT  | Accelerator HALT Status. Accelerator HALT status. 1 - Acknowledge to core that acceleration is in halt state. 0 - Not in halt state                                                                                                             |
| 13:12 (R/NW)       | CURITER    | Current MAC Iteration. The FIR_DMASTAT.CURITER bit indicates the current MAC iteration in multi- iteration mode. Zero indicates the final iteration.                                                                                            |
| 11:7 (R/NW)        | CURCHNL    | Current Channel. The FIR_DMASTAT.CURCHNL bit indicates the channel that is being processed in the TDMslot. Zero indicates the last slot.                                                                                                        |
| 6 (RC/NW)          | ACDONE     | All Channels Done. The FIR_DMASTAT.ACDONE bit indicates the accelerator that processing all channels is complete. This is a sticky bit and is cleared on a register read. The FIR_CTL1.CCINTR bit does not affect the FIR_DMASTAT.ACDONE bit.   |
| 5 (RC/NW)          | WDONE      | Channel Done. The FIR_DMASTAT.WDONE bit indicates the accelerator that processing the cur- rent channel is complete. This is a sticky bit and is cleared on a register read. The FIR_CTL1.CCINTR bit does not affect the FIR_DMASTAT.WDONE bit. |

Table 43-20: FIR\_DMASTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------|
| 4 (R/NW)           | WRBK       | Writing Back. The FIR_DMASTAT.WRBK bit indicates the accelerator is writing back the updated index registers.      |
| 3 (R/NW)           | PPGS       | MAC Processing in Progress. The FIR_DMASTAT.PPGS bit indicates MAC processing in progress.                         |
| 2 (R/NW)           | DLD        | Data Preload. The FIR_DMASTAT.DLD bit indicates data preloading.                                                   |
| 1 (R/NW)           | CLD        | Coefficient Loading. The FIR_DMASTAT.CLD bit indicates coefficient loading.                                        |
| 0 (R/NW)           | CPLD       | Chain Pointer Loading Status. The FIR_DMASTAT.CPLD bit indicates the state machine is in chain pointer load state. |

## FIR Input Data Base Register

The FIR\_INBASE register contains the input word base address with the lower two bits removed.

Figure 43-19: FIR\_INBASE Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000017_e42339325bd48ddd2823e29ce730b57dda50d1e932052025814efa016aa85178.png)

Table 43-21: FIR\_INBASE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------|
| 29:0 (R/W)         | VALUE      | Word Address with Lower 2 Bits Removed. The FIR_INBASE.VALUE bit field contains the word address with the lower 2 bits removed. |

## FIR Input Data Count Register

The FIR\_INCNT register contains the 16-bit input data count.

Figure 43-20: FIR\_INCNT Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000018_e738dffffcfca6f7913f8a85c3d2fa925f044df6ff6742f4dd6868d70ef6564b.png)

Table 43-22: FIR\_INCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                             |
|--------------------|------------|---------------------------------------------------------------------|
| 15:0               | VALUE      | 16-bit Input Data Count.                                            |
| (R/W)              |            | The FIR_INCNT.VALUE bit field contains the 16-bit input data count. |

## FIR Input Data Index Register

The FIR\_INIDX register contains the input word address with the lower two bits removed.

Figure 43-21: FIR\_INIDX Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000019_e42339325bd48ddd2823e29ce730b57dda50d1e932052025814efa016aa85178.png)

Table 43-23: FIR\_INIDX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------|
| 29:0 (R/W)         | VALUE      | Word Address with Lower 2 Bits Removed. The FIR_INIDX.VALUE bit field contains the input word address with the lower two bits removed. |

## FIR Input Data Modifier Register

The FIR\_INMOD register contains the 16-bit input data buffer modifier.

Figure 43-22: FIR\_INMOD Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000020_c730b6f1ba9aa88de866168ee8a7c12a78401f58b6b68ff87858a83bbc98996f.png)

Table 43-24: FIR\_INMOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                       |
|--------------------|------------|-------------------------------------------------------------------------------|
| 15:0               | VALUE      | 16-bit Input Data Buffer Modifier.                                            |
| (R/W)              |            | The FIR_INMOD.VALUE bit field contains the 16-bit input data buffer modifier. |

## FIR MAC Status Register

The FIR\_MACSTAT register provides the status of MAC operations. The status of all four multipliers/adders are available separately for programs to poll. In fixed-point mode only, the ARIx bits are used (all other bits are reserved).

Figure 43-23: FIR\_MACSTAT Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000021_79cd1724a37a3cce45ed69835018147900b3ce1321431b6f93cc0d0f30f62083.png)

Table 43-25: FIR\_MACSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                              |
|--------------------|------------|----------------------------------------------------------------------|
| 23                 | AINV3      | Addition Invalid.                                                    |
| (RC/NW)            |            | The FIR_MACSTAT.AINV3 bit is set if the adder 3 addition is invalid. |

Table 43-25: FIR\_MACSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------|
| 22 (RC/NW)         | ARI3       | Adder Result Overflow. The FIR_MACSTAT.ARI3 bit is set if the adder 3 result is infinity. Indicates overflow in fixed-point mode. |
| 21 (RC/NW)         | ARZ3       | Adder Result Underflow. The FIR_MACSTAT.ARZ3 bit is set if the adder 3 result is zero.                                            |
| 20 (RC/NW)         | MINV3      | Multiply Invalid. The FIR_MACSTAT.MINV3 bit is set if the multiplier 3 multiply operation is invalid.                             |
| 19 (RC/NW)         | MRI3       | Multiplier Result Overflow. The FIR_MACSTAT.MRI3 bit is set if the multiplier 3 result is infinity.                               |
| 18 (RC/NW)         | MRZ3       | Multiplier Result Underflow. The FIR_MACSTAT.MRZ3 bit is set if the multiplier 3 result is zero.                                  |
| 17 (RC/NW)         | AINV2      | Addition Invalid. The FIR_MACSTAT.AINV2 bit is set if the adder 2 addition is invalid.                                            |
| 16 (RC/NW)         | ARI2       | Adder Result Overflow. The FIR_MACSTAT.ARI2 bit is set if the adder 2 result is infinity. Indicates overflow in fixed-point mode. |
| 15 (RC/NW)         | ARZ2       | Adder Result Underflow. The FIR_MACSTAT.ARZ2 bit is set if the adder 2 result is zero.                                            |
| 14 (RC/NW)         | MINV2      | Multiply Invalid. The FIR_MACSTAT.MINV2 bit is set if the multiplier 2 multiply operation is invalid.                             |
| 13 (RC/NW)         | MRI2       | Multiplier Result Overflow. The FIR_MACSTAT.MRI2 bit is set if the multiplier 2 result is infinity.                               |
| 12 (RC/NW)         | MRZ2       | Multiplier Result Underflow. The FIR_MACSTAT.MRZ2 bit is set if the multiplier 2 result is zero.                                  |
| 11 (RC/NW)         | AINV1      | Addition Invalid. The FIR_MACSTAT.AINV1 bit is set if the adder 1 addition is invalid.                                            |
| 10 (RC/NW)         | ARI1       | Adder Result Overflow. The FIR_MACSTAT.ARI1 bit is set if the adder 1 result is infinity. Indicates overflow in fixed-point mode. |
| 9 (RC/NW)          | ARZ1       | Adder Result Underflow. The FIR_MACSTAT.ARZ1 bit is set if the adder 1 result is zero.                                            |

Table 43-25: FIR\_MACSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------|
| 8 (RC/NW)          | MINV1      | Multiply Invalid. The FIR_MACSTAT.MINV1 bit is set if the multiplier 1 multiply operation is invalid.                             |
| 7 (RC/NW)          | MRI1       | Multiplier Result Overflow. The FIR_MACSTAT.MRI1 bit is set if the multiplier 1 result is infinity.                               |
| 6 (RC/NW)          | MRZ1       | Multiplier Result Underflow. The FIR_MACSTAT.MRZ1 bit is set if the multiplier 1 result is zero.                                  |
| 5 (RC/NW)          | AINV0      | Addition Invalid. The FIR_MACSTAT.AINV0 bit is set if the adder 0 addition is invalid.                                            |
| 4 (RC/NW)          | ARI0       | Adder Result Overflow. The FIR_MACSTAT.ARI0 bit is set if the adder 0 result is infinity. Indicates overflow in fixed-point mode. |
| 3 (RC/NW)          | ARZ0       | Adder Result Underflow. The FIR_MACSTAT.ARZ0 bit is set if the adder 0 result is zero.                                            |
| 2 (RC/NW)          | MINV0      | Multiply Invalid. The FIR_MACSTAT.MINV0 bit is set if the multiplier 0 multiply operation is invalid.                             |
| 1 (RC/NW)          | MRI0       | Multiplier Result Overflow. The FIR_MACSTAT.MRI0 bit is set if the multiplier 0 result is infinity.                               |
| 0 (RC/NW)          | MRZ0       | Multiplier Result Underflow. The FIR_MACSTAT.MRZ0 bit is set if multiplier 0 result is zero.                                      |

## FIR Output Data Base Register

The FIR\_OUTBASE register contains the output word base address with the lower two bits removed

Figure 43-24: FIR\_OUTBASE Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000022_e42339325bd48ddd2823e29ce730b57dda50d1e932052025814efa016aa85178.png)

Table 43-26: FIR\_OUTBASE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------|
| 29:0 (R/W)         | VALUE      | Word Address with Lower 2 Bits Removed. The FIR_OUTBASE.VALUE bit field contains the word address with the lower 2 bits removed. |

## FIR Output Data Count Register

The FIR\_OUTCNT register contains the 16-bit output buffer count.

Figure 43-25: FIR\_OUTCNT Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000023_e067c34828d742858d84e29c633f9ddeb02e93fdfcdbe26da0c18b19318380b8.png)

Table 43-27: FIR\_OUTCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                 |
|--------------------|------------|-------------------------------------------------------------------------|
| 15:0               | VALUE      | 16-bit Output Buffer Count.                                             |
| (R/W)              |            | The FIR_OUTCNT.VALUE bit field contains the 16-bit output buffer count. |

## FIR Output Data Index Register

The FIR\_OUTIDX register contains the output word address with the lower two bits removed.

Figure 43-26: FIR\_OUTIDX Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000024_bccf62e962cea5cbdd2378843679153e6320050155f8f8719dc8361ad7ea8c29.png)

Table 43-28: FIR\_OUTIDX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------|
| 29:0 (R/W)         | VALUE      | Word Address with Lower 2 Bits Removed. The FIR_OUTIDX.VALUE bit field contains the word address with the lower 2 bits removed. |

## FIR Output Data Modifier Register

The FIR\_OUTMOD register contains the 16-bit output data buffer modifier.

Figure 43-27: FIR\_OUTMOD Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000025_b8113def32dd1ca6efaf0535304523686794caf503080961d5157f1bacfa9dd8.png)

Table 43-29: FIR\_OUTMOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                  |
|--------------------|------------|--------------------------------------------------------------------------|
| 15:0               | VALUE      | 16-bit Output Data Modifier.                                             |
| (R/W)              |            | The FIR_OUTMOD.VALUE bit field contains the 16-bit output data modifier. |

## Software Control Register 1

The FIR\_SCTL1 register is used to control software.

Figure 43-28: FIR\_SCTL1 Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000026_70221bda6f4fc147a56a5ab8663a28a9d6cf67ec7a3ed5067a7a116c46895593.png)

Table 43-30: FIR\_SCTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Software control field 1. |
| (R/W)              |            |                           |

## Software Control Register 2

The FIR\_SCTL2 register is used to control software.

Figure 43-29: FIR\_SCTL2 Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000027_42d8130e0345d0f1c6fef022e8793208c413d052b52a3cef99bb0624ab68efb2.png)

Table 43-31: FIR\_SCTL2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Software control field 2. |
| (R/W)              |            |                           |

## Secondary Global Control Register

The FIR\_SGCTL register is used to configure the global parameters for the accelerator in ACM mode for loading CTL1 register bits as part of TCB.

Figure 43-30: FIR\_SGCTL Register Diagram

![Image](46_FIR_Accelerator_(FIR)_artifacts/image_000028_f487452431bf989b824df93807ba66ba52302d299c07c947b526714e5ed80c54.png)

Table 43-32: FIR\_SGCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | RND        | Rounding Mode Select for Floating-Point Mode. The FIR_SGCTL.RND bit indicates whether the rounding mode is to the nearest or truncated (round towards zero).                                                                     |
| 13 (R/W)           | TC         | Two's-Complement Format Input Select. The FIR_SGCTL.TC bit indicates the two's complement format input for fixed- point mode.                                                                                                    |
| 12 (R/W)           | FXD        | Fixed-Point Accelerator Select. When set (=1), the FIR_SGCTL.FXD bit indicates a 32-bit fixed point accelerator. When cleared (=0), the bit indicates a 32-bit floating-point compatible with SHARC- XI. 0 32-bit floating-point |
| 12 (R/W)           |            |                                                                                                                                                                                                                                  |