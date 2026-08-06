## 51   FIR Accelerator (FIR)

Finite Impulse Response (FIR) filters are frequently used in DSP applications. The FIR accelerator is a dedicated hardware interface used to perform filter processing to reduce the instruction processing load on the core. FIR filters are used in a wide array of applications including multi-rate processing with an interpolator or decimator.

## Features

This hardware module can perform FIR filters without core intervention. This feature gives programs freedom to use the core to implement complex algorithms, effectively adding more bandwidth to the processor. The FIR supports the following features:

- Fixed-point and 32-bit IEEE floating-point format
- Four MAC units that operate in parallel
- Various rounding modes
- Single rate or multi-rate window processing
- Programmable rates with decimation or interpolation mode
- Up to 32 filter channels available in TDM

NOTE: The FIR accelerator module has local memory that the core cannot access during regular operation. Unlike previous SHARC processors, the FIR accelerator modules each have access to the system memory (on-chip or off-chip).

Also, unlike in previous SHARC processors, where only one of the FIR or IIR accelerators can be used at a time, the SHARC+ processor can use both of these accelerators simultaneously.

## Clocking

The FIR accelerator runs at the maximum speed of the system clock frequency (SCLK0).

## Functional Description

The FIR Block Diagram shows the block diagram of the 1024-TAP FIR hardware accelerator. The accelerator consists of a 1024 word coefficient memory, a 1024 deep delay line for data, and four MAC units. The accelerator runs at the SCLK0 frequency.

Figure 51-1: FIR Block Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000000_43cec4d10cb77e51bc392de84009cbabe13e696a67a5551e0d744e18e1a493f2.png)

The FIR accelerator has the following logical sub blocks:

1. A datapath unit that consists of:
- A 1024 deep coefficient memory
- A 1024 deep delay line for the data
- Four 32-bit floating-point and fixed-point multiplier and adder units
- One 32-bit prefetch buffer to operate in a pipelined fashion
- One 32-bit buffer to hold previous partial sum
- One 32-bit buffer to hold the output
2. Configuration registers for the number of TAPs, number of channels, filter enable, interrupt control, DMA enable, up sample or down sample control, and ratios.
3. Core access interface for writing to the DMA and filter configuration registers and reading the status register.
4. DMA bus interface for transferring data and coefficients to and from the accelerator.
5. DMA configuration registers including chain pointer, input, output, and coefficient registers.

## ADSP-SC58x FIR Register List

The FIR accelerator is a dedicated hardware interface used to perform filter processing to reduce the instruction processing load on the core.

Table 51-1: ADSP-SC58x FIR Register List

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

## ADSP-SC58x FIR Interrupt List

Table 51-2: ADSP-SC58x FIR Interrupt List

|   Interrupt ID | Name      | Description   | Sensitivity   | DMA Channel   |
|----------------|-----------|---------------|---------------|---------------|
|            153 | FIR0_DMA  | FIR0DMA       | Edge          |               |
|            154 | FIR0_STAT | FIR0 Status   | Edge          |               |

## ADSP-SC58x FIR Trigger List

Table 51-3: ADSP-SC58x FIR Trigger List Masters

|   Trigger ID | Name     | Description   | Sensitivity   |
|--------------|----------|---------------|---------------|
|           60 | FIR0_DMA | FIR0DMA       | Edge          |

Table 51-4: ADSP-SC58x FIR Trigger List Slaves

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         | None   | None          | None          |

## Compute Block

The MAC unit, shown in the FIR MAC Unit figure, has four multiply accumulators. The accumulators operate simultaneously on a single filter as described as follows:

- The MAC unit operates on the data and coefficient fetched from the data and coefficient RAMs
- Each MAC can perform 32-bit floating-point or 32-bit fixed-point MAC operations
- Floating-point format is IEEE-compliant
- Multiply and accumulation operation (addition) are pipelined
- A 32-bit floating-point MAC operation generates 32-bit multiply results
- A 32-bit fixed-point operation generates 80-bit results (64-bit result + 16 guard bits)

Figure 51-2: FIR MAC Unit

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000001_301556a0a54add08ff7a5a7e6e0dd2e14c1e2641a8d6623a6e43c0b8ad669faf.png)

## Partial Sum Register

The partial sum register is useful for Floating-point Multi-Iteration mode. For a particular channel, the intermediate MAC result is written to the output buffer of the system (on-chip or off-chip) memory. If the same channel is requested again, the partial result register is updated with the intermediate MAC result through DMA from output

buffer of the system memory. The result is added to the current MAC result after each iteration. This process repeats until all iterations complete (the entire soft filter length is processed).

## Delay Line Memory

The accelerator has a 1024 TAP delay line to hold the data locally. The DMA controller fetches the data from system memory and loads it into the delay line. Four read accesses can be made to the delay line simultaneously.

## Coefficient Memory

The accelerator has a 1024 deep coefficient memory to store the coefficients. The DMA controller loads the coefficients from system memory into coefficient memory. Four coefficients can be fetched from the coefficient memory simultaneously. If the soft filter length is more than 1024, processing happens in multi-iteration mode.

## Prefetch Data Buffer

The prefetch data buffer enables pipeline operation. One data sample is prefetched when the compute unit is operating on the delay line corresponding to the current sample. The data prefetched in this buffer is later used to update the delay line for the next sample. This operation happens in parallel again when the compute unit is not accessing the delay line. In other words, it happens when the compute unit is adding the output from the four MACs and the partial sum register.

Table 51-5: Pipeline Operation for Window Size = 1

| Cycles     | 1   | 2           | 3           | 4           | 5   | 6   |
|------------|-----|-------------|-------------|-------------|-----|-----|
| Output DMA |     |             | N           | N1          | N2  | N3  |
| Compute    |     | N           | N1          | N2          | N3  |     |
| Input DMA  | N   | prefetch N1 | prefetch N2 | prefetch N3 |     |     |

## Processing Output

The accelerator uses all four MACs simultaneously to calculate one output sample as shown in the Multi-Iteration Filtering Flow figure and the following procedure.

Figure 51-3: Multi-Iteration Filtering Flow

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000002_d98a6de1e72490f40e10a8e0b040ec4bfecd190c07039e0b3c54c188dcded223.png)

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
x[n-(N-1)], x[n-(N-2)] ... x[n-1], x[n], x[n+1] ... x[n+W-1]
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

Assuming L= interpolation ratio, the total size of the input buffer should be at least equal to Ceil ((N-1)/L)

+W/L

.

If the input buffer that needs to be processed is:

```
x[n], x[n+1], x[n+2]....x[n+W/L-1], and K= Ceil((N-1)/L)
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

Figure 51-4: Filter Output Calculation

## Single Iteration

Results are computed in a single iteration when the soft filter length is less than or equal to 1024.

## Floating-point Multi-Iteration

Results are computed in multiple iterations when the soft filter length is greater than 1024 (for example, 2048 TAPs on a 1024 hard filter length). In this mode, the controller implements two iterations of 1024 TAPs.

NOTE: If the soft filter length is not a multiple of the hard filter length, the controller iterates until the soft filter length is satisfied.

Example: 550 taps on a 256 tap filter. In this example, the FIR controller implements two iterations of 256 taps and one iteration of 38 taps.

NOTE: Multi-iteration mode is not supported in fixed-point format.

## Window Processing

Configure the window size to 1 to select sample-based processing mode. In this mode, one sample from a particular channel is processed through all the biquads of that channel and the final output sample is calculated.

In window-based mode, multiple output samples (up to 1024) equal to the window size of that channel are calculated. After these calculations are complete, the accelerator begins processing the next channel. A configurable window size parameter is provided to specify the length of the window.

## Multi-Rate Processing

Multi-rate filters change the sampling rate of a signal-they convert the input samples of a signal to a different set of data that represents the same signal sampled at a different rate.

## Decimation

A decimation filter provides a single output result for every M input samples, where M is the decimation ratio. The output rate is 1/M'th of the input rate. The filter implementation exploits the low output sample rate by not starting a computation until a new set of M input samples is available.

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

In an interpolation filter, FIR logic inserts L - 1 zeros between each sample. The program has to make sure that these zeros fully shift out of the delay line before moving on to the next channel. This operation puts a restriction on window size in terms of L the sample ratio as showing in the expression:

WINDOWSIZE = n × SAMPLERATIO , where n is the number of input samples.

The input buffer size is smallest integer greater than or equal to (N - 1 + W)/L for interpolation filters where:

- N is the number of taps
- W is the window size
- L is the interpolation ratio

To start the mode, programs configure the FIR\_CTL2.RATIO and FIR\_CTL2.UPSAMP bits (along with filter settings).

## Channel Processing

The Single Channel Filtering Flow figure shows the flow diagram for processing a single channel. Channels are processed in TDM format by setting the FIR\_CTL1.CH bits to a value greater than one. In the time slot corresponding to a particular channel, the corresponding TCB is loaded from system memory.

1. The FIR\_CTL2 register value is fetched from system memory and is used to configure the filter parameters for that channel.
2. The accelerator fetches the coefficients using the FIR\_COEFIDX register as the pointer and loads them into coefficient memory.
3. The delay line is pre-filled using the FIR\_INIDX register as the pointer.
4. The accelerator calculates the first output and stores the result back into the output buffer using the FIR\_OUTIDX register as the pointer.
5. While calculating the output, the accelerator fetches the next data in parallel. After one window of data is processed, the index registers in the system memory TCB are updated. In the next time slot of the same channel, processing can continue from where it stopped.
6. Processing moves to the next channel and repeats the procedure. If the soft filter length is more than the hard filter length, multiple iterations occur to process the window.

Figure 51-5: Single Channel Filtering Flow

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000003_1f18c70a1cf3517b4fe326e811c4560294fb282f9b9ac8bf041351d07dfeb3fa.png)

## Floating-Point Data Format

The FIR accelerator treats data and coefficients in 32-bit floating-point format as the default functional mode.

## Fixed-Point Data Format

In fixed-point mode, the 32-bit input data or coefficient is treated as fixed point. A 32-bit fixed-point MAC operation generates an 80-bit result. Fixed-point data or coefficients can be unsigned integer, unsigned fractional and signed integer.

NOTE: In fixed-point mode, the entire 80-bit result register is always written back in bursts of 3 × 32 bits. The first word is the LSW, the second word is the MSW, and the third word is a 16-bit overflow. The remaining 16 bits are padded with zeros. Therefore, for fixed-point mode: WINDOWSIZE = WINDOWSIZE × 3.

If the signed fractional format is used, the output must be scaled by 2. The MAC does not right shift to remove the redundant sign bit. A final routine must decimate the output buffer to the desired samples.

Multi-iteration mode is not supported in this format. Therefore, the maximum TAP length is 1024.

## Data Transfer

The FIR filter works exclusively through DMA.

## Chain Assignment

The structure of a TCB is conceptually the same as a traditional linked-list. Each TCB has several data values and a pointer to the next TCB. Further, the chain pointer of a TCB can point to itself to continuously re-run the same DMA. The FIR accelerator reads each word of the TCB and loads it into the corresponding register. The end of the chain (no further TCBs are loaded) is indicated by a TCB with a chain pointer register value of zero.

The FIR accelerator DMA supports circular buffer chained DMA. The FIR accelerator does not support circular buffering for the coefficient buffer. The TCBs for Chained DMA table shows the required TCBs for chained DMA.

Table 51-6: TCBs for Chained DMA

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

The FIR\_COEFCNT register is loaded with the values in the FIR\_COEFCNT TCB field and is decremented from that value onwards. However, coefficient loading continues until the number of coefficients, equal to the tap length, are read. This condition is true even if the FIR\_COEFCNT register reaches zero as in the case of a tap length = 10, and the FIR\_COEFCNT field in the TCB is initialized to 0. The value in the FIR\_COEFCNT register is -10 after all coefficients are loaded.

NOTE: Initialize FIR\_CHNPTR to TCB + 12.

## DMA Access

The FIR accelerator has two DMA channels (accelerator input and output) to connect to the system memory. The DMA controller fetches the data and coefficients from memory and stores the result.

## Accelerator TCB

The location of the DMA parameters for the next sequence comes from the chain pointer register that points to the next set of DMA parameters stored in the internal memory of the processor. In chained DMA operations, the processor automatically initializes and then begins another DMA transfer when the current DMA transfer is complete. Each new set of parameters is stored in a user-initialized memory buffer or TCB for a chosen peripheral.

## Chain Pointer DMA

The DMA controller supports circular buffer chain pointer DMA. One transfer control block (TCB) must be configured for each channel. The TCB contains:

- A control register value to configure the filter parameters (such as filter tap length, window size, sample rate conversion settings) for each channel
- DMA parameter register values for the input data (delay line)
- DMA parameter register values for coefficient load
- DMA parameter register values for output data

Intermediate results in multi-iteration mode are saved in the output buffer.

As shown in the Circular Buffer Addressing figure, the accelerator loads the TCB into its internal registers and uses these values to fetch coefficients and data and to store results. After processing a window of data for any channel, the accelerator writes back the appropriate values to the FIR\_INIDX and FIR\_OUTIDX bit fields of the TCB in memory. Then, data processing can begin from where it left off during the next time slot of that channel.

The write-back value for input buffer is:

- FIR\_INIDX + W for single rate filtering
- FIR\_INIDX + W × M for decimation (M = decimation ratio)
- FIR\_INIDX + W/L for interpolation (L = interpolation ratio)
- The write-back value for output buffer in floating point mode is: FIR\_OUTIDX + W
- The write-back value for output buffer in fixed-point mode is: FIR\_OUTIDX + 3 × W

NOTE: The FIR\_CTL2 register is part of the FIR TCB. This configuration allows programming individual FIR channels with different control attributes.

Figure 51-6: Circular Buffer Addressing

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000004_17bfd773d2b40629ba86542c5125f10e54bd9b324565eb4e481d805f2abc7331.png)

## Programming Model

The following sections provide general programming information for the FIR accelerator.

## Single Channel Processing

1. Create input, coefficient, and output buffers in system memory.

For input and coefficient buffer storage format, see Coefficients and Input Buffer Storage.

2. Create the TCBs in system memory. Each TCB corresponds to a particular channel.

TCBs hold the FIR\_CTL2 register which allows programming the window size and tap size along with up or down sample enable, sample rate conversion enable, and the conversion ratio for decimation and interpolation filters.

3. Configure the index, modifier and length entries in the TCBs to point to the corresponding channels' data buffer, coefficient buffer, and output data buffer.

The output index register must point to the start of the output buffer. However, initialize the value of the input index register based on the explanation provided in Coefficients and Input Buffer Storage.

4. The core configures the FIR\_CTL1 register with the number of channels (one channel), fixed- or floatingpoint format.
5. Set the enable bit to start the accelerator operation in the modes configured (in FIR\_CTL1 and FIR\_CTL2 registers) by loading the TCB of the first channel. Once the first channel window is calculated, the input and output index registers are written back to internal memory corresponding to the first channel. Once the writeback is complete the accelerator moves into idle.

NOTE: All the addresses programmed in the TCB correspond to 32-bit address boundaries and should not contain the lower 2 bits (assumed as zeros).

## Multichannel Processing

The Wait for Core Intervention ≥ Idle (if auto channel iterate bit = 0) figure shows the diagram for multichannel filtering. Multiple channels are processed in a time division multiplexed (TDM) format. After completing all the channels, the accelerator can either repeat the slots or wait for core intervention.

For multichannel filtering, use the following steps.

1. Program the number of channels using the FIR\_CTL1.CH bits.
2. Configure the TCBs in system memory with one channel's TCB pointing to the next channel's TCB.
3. Write the first TCB value into the FIR\_CHNPTR register and enable the accelerator.

The accelerator fetches the first channel's TCB and, using it as pointer, prefills the delay line and coefficient memory and loads the FIR\_CTL2 register to configure the filter parameters corresponding to that channel.

The accelerator then calculates output samples corresponding to one window and stores the data back in internal memory.

At the end of the window the accelerator updates the FIR\_INIDX and FIR\_OUTIDX registers in the TCB of system memory and moves to the next channel.

When all the channels are finished and the auto channel iterate bit ( FIR\_CTL1.CAI ) =1, the accelerator processes the first channel again and iterates through the channels. If the FIR\_CTL1.CAI bit =0, the accelerator waits for core intervention.

Figure 51-7: Wait for Core Intervention ≥ Idle (if auto channel iterate bit = 0)

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000005_ada51adedfcfef25050a9f4c4a4b2b7faf3f82ff8298a0aeb728a71e2a3f1a33.png)

## Dynamic Coefficient Processing Notes

1. The dynamic update of the coefficients can be useful for the FIR accelerator. The FIR accelerator reloads the coefficients for each iteration (if the FIR\_CTL1.CAI bit is set) before the start of processing of each channel.
2. The dynamic coefficient update is possible for single iteration mode (tap length ≤ 1024). Ensure that the new coefficients are updated after the accelerator loads the coefficients for current processing and before the next processing starts. The expression for the maximum time available for the coefficient memory update should be equal to 49 +N × 11 + W × (N/4 + 2) SCLK cycles.

3. For multi-iteration mode, dynamic updates are not supported. Programs must finish current processing, disable the accelerator, update the coefficients, and reenable the accelerator.

## Debug Mode

The next sections show the steps required for reading and writing local memory in debug mode.

## Write to Local Memory

1. Clear the FIR\_CTL1.DMAEN bit.
2. Set the FIR\_DBG\_CTL.EN , FIR\_DBG\_CTL.MEM , and FIR\_DBG\_CTL.HLD bits.
3. Set the FIR\_DBG\_CTL.ADRINC bit for address auto increment.
4. Write the start address to the FIR\_DBG\_ADDR register.

NOTE: If the bit 11 in the FIR\_DBG\_ADDR register is set, coefficient memory is selected.

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
2. Clear the FIR\_DBG\_CTL.HLD bit and enable FIR\_DBG\_CTL.EN and FIR\_DBG\_CTL.RUN bits.
3. Program the FIR module according to the application.
4. In single-step mode, each iteration is updated in the emulator session.

## FIR Programming Example

In this example the application requires the FIR to filter six channels of data. The first four channels require a 256 TAP filter and the last two channels require a 1024 TAP filter. The window size for all the channels is 128.

1. Create a circular data buffer in system memory for each channel.

The buffer should be large enough to avoid overwriting data before the accelerator processes it. Ideally, the input buffer size for a channel is tap length + window size - 1 for that channel. The 256 coefficients of each of the first four channels and the 1024 coefficients each of the last two channels are also configured in system memory buffers. The output buffer size is equal to the window size.

2. Create six TCBs in system memory with each channel's chain pointer (CP) entry pointing to the next channel's and the sixth channel's CP entry pointing back to the first channel's in a circular manner.
3. Configure the FIR\_CTL2 register for the first four channels' TCBs to 256 TAPs and a window size of 128, and the next two channels for 1024 TAPs and a window size of 128, respectively.
4. Configure the index, modifier, length entries in the TCBs to point to the corresponding channel's data buffer, coefficient buffer, and output data buffer. The location of the first channel's TCB is written to the FIR\_CHNPTR register. The FIR\_CTL1.CH bit field is then programmed with a value that corresponds to six channels.
- a. The accelerator iterates through six channels once and then waits for core intervention (the FIR\_CTL1.CAI bit is not set, the DMA is enabled, and the FIR\_CTL1.EN bit is set).
- b. The accelerator loads the TCB of the first channel, then loads the coefficient and data, and processes one window.
- c. After saving the index values to memory, the accelerator moves to the next channel.
- d. After all six channels are complete, the accelerator halts and waits for core intervention.

## Computing FIR Output, Tap Length Greater than 4096

With little core intervention, the FIR accelerator can also be used to calculate output for a tap length greater than 4096 taps. The section demonstrates the calculation with an example of 8192 taps.

1. Divide the transfer function of an 8192 FIR filter into two 4096 FIR filters:

<!-- formula-not-decoded -->

2. Divide the filter coefficients of an 8192 tap filter among two 4096 tap FIR filters:

Filter 1

```
Coefficients = b0, b1, b2,……, b4095 Input data = x[n], x [n - 1],……….x[n - 4095] Filter 2 Coefficients = b4096, b4097,…………..,b8191 Input data = x[n - 4096], x[n - 4097],……x[n - 8191]
```

The accelerator can be used in two-channel mode where channel 1 operates on x[n]…x[n - 4095] input data with the filter coefficients of filter 1 and channel 2 operates on x[n - 4096]…x[n - 8191] with the filter coefficients of filter 2.

Once both the channels are processed, add the partial sum output of both the channels can to get the final output. Implement this approach (tap length = TAPS = 8192, window size = WINDOW) using the following programming steps.

1. Create a circular input data buffer in system memory (IBUF). The buffer must be large enough to avoid overwriting data before the accelerator processes it. Ideally, the input buffer size for a channel is TAPS + WINDOW - 1.
2. Create a coefficient buffer of size TAPS (8192) (CBUF).
3. Create one output buffer of size WINDOW (OBUF) and another temporary output buffer (OBUF1) to store the partial sum.
4. Create two TCBs in system memory with first TCB chained to the second and second one chained to the first in circular manner.
- a. The FIR\_COEFIDX bit field of the first TCB should point to the start address of the coefficient buffer (CBUF) and that of the second TCB should point to 4096 offset from the start of the coefficient buffer (CBUF + 4096).
- b. The FIR\_OUTBASE and FIR\_OUTIDX bit field of the first TCB should point to the start address of OBUF and that of the second TCB should point to the start address of OBUF1.
- c. The FIR\_INIDX bit field of the first TCB should point to the start address of IBUF and that of the second TCB should point to 4096 offset from the start address of IBUF .
- d. Configure the FIR\_CTL2 bit field of both the TCB for tap length = TAP/2 = 4096 and window size = WINDOW.
5. Initialize the FIR\_CHNPTR register pointing to the first TCB.
6. Program the FIR\_CTL1 register to initiate the accelerator processing now by setting the FIR\_CTL1.EN and FIR\_CTL1.DMAEN bits and the number of channels configured as 2.
7. Wait for the FIR all channel done interrupt ( FIR\_DMASTAT.ACDONE ) to occur. Inside the ISR, add the partial sum results using the core from both the output buffers (OBUF and OBUF1) to get the final output. To save memory, replace the contents of the buffer OBUF with the final output result.

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

Table 51-7: FIR Interrupt Overview

| Default Programmable Inter- rupt   | Sources                                                                    | Masking   | Service                                |
|------------------------------------|----------------------------------------------------------------------------|-----------|----------------------------------------|
| FIR_DMA FIR_STAT                   | Input DMAcomplete Output DMAcomplete Window complete All channels complete | N/A       | ROC from FIR_DMASTAT + RTI instruction |
| FIR_DMA FIR_STAT                   | MAC IEEE floating-point exceptions MAC fixed-point overflow                |           | ROC from FIR_MACSTAT + RTI instruction |

## Sources

The FIR module drives two interrupt signals: FIR\_DMA for the DMA status and FIR\_STAT for the MAC status. The FIR module generates interrupts as described in the following sections.

## Window Complete

This interrupt is generated at the end of each channel when all the output samples are calculated corresponding to a window and updated index values are written back.

## All Channels Complete

This interrupt is generated when all the channels are complete or when one iteration of time slots completes. Note that the interrupt follows the access completion rule, where the interrupt is generated when all data are written back to system memory.

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

When a DMA interrupt occurs, programs can find whether the input or output DMA interrupt occurred by reading the DMA status register ( FIR\_DMASTAT ). The DMA interrupt status bits are sticky and are cleared when the DMA status register is read. When a MAC status interrupt occurs, programs can find this state by reading the MAC status register ( FIR\_MACSTAT ). The read of the register clears the (sticky) MAC interrupt status bits.

The status interrupt sources are derived from the FIR\_MACSTAT register. A status interrupt can occur due to the last set of MAC operations of a processing iteration that correspond to a particular channel. The interrupt is generated continuously and cannot be stopped, even after disabling the accelerator. The interrupt can only be stopped when another processing iteration results in a non-zero or valid multiply or add result. However, in this situation, it is difficult to isolate whether the interrupt corresponds to the previous processing iteration or the current one. This functionality makes using status interrupts impractical.

Another option is to poll the status bits of the FIR\_MACSTAT register inside the DMA interrupt service routine. However, consider the behavior of the status bits. The status bits in the FIR\_MACSTAT registers are sticky. Once a status bit is set, it gets cleared only when the FIR\_MACSTAT register is read and the previous set of MAC operations resulted in a non-zero, valid output. Therefore, if the last set of MAC operations of a processing iteration results in a zero, non-valid output, the corresponding status bit are not cleared, even after reading the FIR\_MACSTAT register. T o avoid a false indication in the next processing iteration, ensure that all the status bits are cleared after the current iteration finishes.

The solution is to read the FIR\_MACSTAT register twice inside the DMA interrupt service routine. The first read identifies which status bits are set. The second read is used to discover if the status bit was set because of the last set of MAC operations. If the status bit was not set because of the last set of MAC operations, it provides a zero result.

If the bit was set because of the last set of MAC operations, clear the status bit by performing a simple dummy FIR processing iteration (tap length = 4 and window size = 1). Choose the appropriate FIR\_MACSTAT register after the processing is complete.

For more information, see the "FIR MAC Status Register" section.

## Effect Latency

The total effect latency is a combination of the write effect latency (core access) plus the peripheral effect latency (peripheral-specific).

## Write Effect Latency

For details on write effect latency, see the SHARC Processor Programming Reference .

## FIR Throughput

Accelerator input and output channels are used to connect to internal memory. Data throughput is one 32-bit data word per SCLK cycle for writes to memory, provided there are no conflicts. Read throughput from memory, throughput is one 32-bit data word per two SCLK clock cycles.

The following information describes the performance of the FIR accelerator in SCLK cycles. The total number of SCLK cycles for a single channel single rate filtering N &lt;= 1024 is the sum of the cycles taken for the following operations.

- TCB load (SCLK cycles) = 13 × 11 + 3
- Coefficient Load (SCLK cycles)= N × 11 + 3
- Data Load (SCLK cycles)= N × 11 + 3
- Calculation (SCLK cycles)= (N/4 + 2) × W + 16
- Write Back (SCLK Cycles) = 27

Total number of SCLK cycles for a single rate filtering N &lt;= 1024 is:

```
(195 + 2 × N × 11 + W × (N/4+2) ) × C SCLK cycles
```

where:

- N is the Number of taps
- W is the Window size
- C is the Number of channels

## ADSP-SC58x FIR Register Descriptions

The FIR accelerator (FIR) contains the following registers.

Table 51-8: ADSP-SC58x FIR Register List

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

## FIR Chain Pointer Register

The FIR\_CHNPTR register contains the chain pointer address.

Figure 51-8: FIR\_CHNPTR Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000006_6484082f9476f52f654ce2f1b8477d138778c972efdb841efed050b500c8488c.png)

Table 51-9: FIR\_CHNPTR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                            |
|--------------------|------------|--------------------------------------------------------------------|
| 29:0               | VALUE      | Chain Pointer Address.                                             |
| (R/W)              |            | The FIR_CHNPTR.VALUE bit field contains the chain pointer address. |

## FIR Coefficient Count Register

The FIR\_COEFCNT register contains the 16-bit coefficient buffer count.

Figure 51-9: FIR\_COEFCNT Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000007_6bea8679a6931ff1935631853de94812e45c86a2b2451aed4b91e42b9e48a884.png)

Table 51-10: FIR\_COEFCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                      |
|--------------------|------------|------------------------------------------------------------------------------|
| 15:0               | CCNT       | 16-bit Coefficient Buffer Count.                                             |
| (R/W)              |            | The FIR_COEFCNT.CCNT bit field contains the 16-bit coefficient buffer count. |

## FIR Coefficient Index Register

The FIR\_COEFIDX register contains the coefficient index word address with the lower two bits removed.

Figure 51-10: FIR\_COEFIDX Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000008_79d00be7024880ff665148b786e0f9489e28a55071f7910153199e332f94841c.png)

Table 51-11: FIR\_COEFIDX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------|
| 29:0 (R/W)         | VALUE      | Word Addresses with Lower 2 Bits Removed. The FIR_COEFIDX.VALUE bit field contains the word addresses with the lower 2 bits removed. |

## FIR Coefficient Modifier Register

The FIR\_COEFMOD register contains the 16-bit coefficient index modifier.

Figure 51-11: FIR\_COEFMOD Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000009_f6ae5218da3483ee46ee134357630ff4c14ec2e3098e0409f8351ad73ddb8770.png)

Table 51-12: FIR\_COEFMOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                         |
|--------------------|------------|---------------------------------------------------------------------------------|
| 15:0               | VALUE      | 16-bit Coefficient Index Modifier.                                              |
| (R/W)              |            | The FIR_COEFMOD.VALUE bit field contains the 16-bit coefficient index modifier. |

## FIR Global Control Register

The FIR\_CTL1 register is used to configure the global parameters for the accelerator. These parameters include the number of channels, channel auto iterate, DMA enable, and accelerator enable.

Figure 51-12: FIR\_CTL1 Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000010_ca29a6fda27f12dc4ef8cc02ddc851ae01c7aae351594289cc48a2c81698c7f4.png)

Table 51-13: FIR\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                           | Description/Enumeration                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------|
| 16:14 (R/W)        | RND        | Rounding Mode. The FIR_CTL1.RND bit configures the accelerator to use one of the following rounding modes.                                        | Rounding Mode. The FIR_CTL1.RND bit configures the accelerator to use one of the following rounding modes.                                        |
| 16:14 (R/W)        | RND        | 0                                                                                                                                                 | Round to Nearest                                                                                                                                  |
| 16:14 (R/W)        | RND        | 1                                                                                                                                                 | Truncate (Round away from zero)                                                                                                                   |
| 13 (R/W)           | TC         | Two's-Complement. The FIR_CTL1.TC bit configures the accelerator to use either unsigned integer or signed integer                                 | Two's-Complement. The FIR_CTL1.TC bit configures the accelerator to use either unsigned integer or signed integer                                 |
| 13 (R/W)           | TC         | 0                                                                                                                                                 | Unsigned integer                                                                                                                                  |
| 13 (R/W)           | TC         | 1                                                                                                                                                 | Signed integer                                                                                                                                    |
| 12 (R/W)           | FXD        | Fixed-Point Accelerator Select. The FIR_CTL1.FXD bit configures the accelerator to use either 32-bit IEEE float- ing-point or 32-bit fixed-point. | Fixed-Point Accelerator Select. The FIR_CTL1.FXD bit configures the accelerator to use either 32-bit IEEE float- ing-point or 32-bit fixed-point. |
| 12 (R/W)           | FXD        | 0                                                                                                                                                 | 32-bit IEEE floating-point                                                                                                                        |
| 12 (R/W)           | FXD        | 1                                                                                                                                                 | 32-bit fixed point                                                                                                                                |

Table 51-13: FIR\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | CCINTR     | Channel Complete Interrupt. The FIR_CTL1.CCINTR bit configures the accelerator to generate an interrupt when each or all channels are done.                                                                                                 |
| 11 (R/W)           | CCINTR     | 0 Interrupt is generated only when all channels are done                                                                                                                                                                                    |
| 11 (R/W)           | CCINTR     | 1 Interrupt is generated after each channel is done                                                                                                                                                                                         |
| 9 (R/W)            | CAI        | Channel Auto Iterate. The FIR_CTL1.CAI bit, if cleared, causes the TDMprocessing to stop (idle) once all channels are done. If set, processing moves to the first channel and continuesTDM processing in a loop when all channels are done. |
| 9 (R/W)            | CAI        | 0 TDMprocessing stops (idle) once all channels are done                                                                                                                                                                                     |
| 9 (R/W)            | CAI        | 1 Moves to first channel and continues TDMprocessing in a loop when all channels are done                                                                                                                                                   |
| 8 (R/W)            | DMAEN      | DMAEnable. The FIR_CTL1.DMAEN bit enables and disables DMAon the FIR accelerator.                                                                                                                                                           |
| 8 (R/W)            | DMAEN      | 0 DMAdisabled                                                                                                                                                                                                                               |
| 8 (R/W)            | DMAEN      | 1 DMAenabled                                                                                                                                                                                                                                |
| 5:1 (R/W)          | CH         | Number of Channels. The FIR_CTL1.CH bit field configures the number of channels and is programma- ble from 0 to 31.                                                                                                                         |
| 5:1 (R/W)          | CH         | 0 Channel 1                                                                                                                                                                                                                                 |
| 5:1 (R/W)          | CH         | 1-30 Channel 2-31                                                                                                                                                                                                                           |
| 5:1 (R/W)          | CH         | 31 Channel 32                                                                                                                                                                                                                               |
| 0 (R/W)            | EN         | FIR Enable. The FIR_CTL1.EN bit enables and disables the FIR accelerator.                                                                                                                                                                   |
| 0 (R/W)            | EN         | 0 Disable FIR                                                                                                                                                                                                                               |
| 0 (R/W)            | EN         | 1 Enable FIR                                                                                                                                                                                                                                |

## FIR Channel Control Register

The FIR\_CTL2 register is used to configure the channel specific parameters such as filter TAP length, window size, sample rate conversion, up/down sampling and ratio.

Figure 51-13: FIR\_CTL2 Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000011_95c652f7ed0367c252f67768772766c2ef86f36bb6a06ed8c54b62b92a8f62f3.png)

Table 51-14: FIR\_CTL2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30 (R/W)           | UPSAMP     | Up Sampling Enable. The FIR_CTL2.UPSAMP bit enables up sampling. 0 Down sampling                                                                                    |
| 29 (R/W)           | SRCEN      | Sample Rate Conversion Enable. The FIR_CTL2.SRCEN bit field enables sample rate conversion. 0 Disabled                                                              |
| 27:25 (R/W)        | RATIO      | Up/Down Sampling Ratio. The FIR_CTL2.RATIO bit field sets the sampling ratio ( FIR_CTL2.RATIO + 1).                                                                 |
| 23:14 (R/W)        | WINDOW     | Window Size. The FIR_CTL2.WINDOW bit field sets the window size which specifies the number of sample/block to process (sample based processing = window size of 0). |
| 11:0 (R/W)         | TAPLEN     | Tap Length. The FIR_CTL2.TAPLEN bit field sets the tap length which is programmable be- tween 0-4095 (Tap Length = FIR_CTL2.TAPLEN + 1).                            |

## Debug Address Register

The FIR\_DBG\_ADDR register holds the debug address.

Figure 51-14: FIR\_DBG\_ADDR Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000012_f39923c0c00b943fdab6e9ef5129b3e094fa2651a12220f912426fdfac836914.png)

Table 51-15: FIR\_DBG\_ADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10:0 (R/W)         | VALUE      | Debug Address, Coefficient Memory Select. The FIR_DBG_ADDR.VALUE bit field holds the debug address (bits 0-10). Bit 11 configures whether the memory access is to coefficient memory (=0) or to delay line memory (=1). |

## FIR Debug Control Register

The FIR\_DBG\_CTL register controls debugging operations such as enabling debug mode running, hold or single stepping.

Figure 51-15: FIR\_DBG\_CTL Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000013_70ba74f4f21789dedf49d55844000b2cc60be92af7cff68fef818c9631eada49.png)

Table 51-16: FIR\_DBG\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                               | Description/Enumeration                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W)            | ADRINC     | Address Auto Increment. The FIR_DBG_CTL.ADRINC bit bit allows the address register to auto-increment on FIR_DBG_WRDAT writes and FIR_DBG_RDDAT reads. | Address Auto Increment. The FIR_DBG_CTL.ADRINC bit bit allows the address register to auto-increment on FIR_DBG_WRDAT writes and FIR_DBG_RDDAT reads. |
| 4 (R/W)            | MEM        | Local Memory Access. When the FIR_DBG_CTL.MEM bit is set, the data and coefficients memory can be indirectly accessed.                                | Local Memory Access. When the FIR_DBG_CTL.MEM bit is set, the data and coefficients memory can be indirectly accessed.                                |
| 2 (R/W1S)          | RUN        | Release MAC. The FIR_DBG_CTL.RUN bit releases the MAC. This bit is self-clearing after one FIR clock cycle.                                           | Release MAC. The FIR_DBG_CTL.RUN bit releases the MAC. This bit is self-clearing after one FIR clock cycle.                                           |
| 1 (R/W)            | HLD        | Hold. The FIR_DBG_CTL.HLD bit holds or single-steps through the FIR.                                                                                  | Hold. The FIR_DBG_CTL.HLD bit holds or single-steps through the FIR.                                                                                  |
| 1 (R/W)            | HLD        | 0                                                                                                                                                     | Hold                                                                                                                                                  |
| 1 (R/W)            | HLD        | 1                                                                                                                                                     | Single-step                                                                                                                                           |
| 0 (R/W)            | EN         | Debug Mode Enable. The FIR_DBG_CTL.EN bit enables debug mode. For local memory access, the FIR_CTL1 register can be cleared.                          | Debug Mode Enable. The FIR_DBG_CTL.EN bit enables debug mode. For local memory access, the FIR_CTL1 register can be cleared.                          |
| 0 (R/W)            | EN         | 0                                                                                                                                                     | Disable debug mode                                                                                                                                    |
| 0 (R/W)            | EN         | 1                                                                                                                                                     | Enable debug mode                                                                                                                                     |

## FIR Debug Data Read Register

The FIR\_DBG\_RDDAT register hold the debug read data.

Figure 51-16: FIR\_DBG\_RDDAT Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000014_5447fa4e79dba583ec5906c71c1af615cead5c1630c50edf83e97d5e7a026b6e.png)

Table 51-17: FIR\_DBG\_RDDAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                      |
|--------------------|------------|--------------------------------------------------------------|
| 31:0               | VALUE      | Debug Read Data.                                             |
| (R/W)              |            | The FIR_DBG_RDDAT.VALUE bit field holds the debug read data. |

## FIR Debug Data Write Register

The FIR\_DBG\_WRDAT register holds the debug write data.

Figure 51-17: FIR\_DBG\_WRDAT Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000015_cd9a49333c00fc9cace7915b14840f07aadac5f2c0be4170fd4b2dc50709c953.png)

Table 51-18: FIR\_DBG\_WRDAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                       |
|--------------------|------------|---------------------------------------------------------------|
| 31:0               | VALUE      | Debug Write Data.                                             |
| (R/W)              |            | The FIR_DBG_WRDAT.VALUE bit field holds the debug write data. |

## FIR DMA Status Register

The FIR\_DMASTAT register provides information about chain pointer loading, coefficient DMA, data preload DMA, processing in progress, window complete, all channels complete.

Figure 51-18: FIR\_DMASTAT Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000016_599e7be166ac9f5d994e054fa27ea7b00d0c68ad150d1217f4b41b0f2a8194ed.png)

Table 51-19: FIR\_DMASTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:12 (R/NW)       | CURITER    | Current MAC Iteration. The FIR_DMASTAT.CURITER bit indicates the current MAC iteration in multi- iteration mode. Zero indicates the final iteration.                                                                                            |
| 11:7 (R/NW)        | CURCHNL    | Current Channel. The FIR_DMASTAT.CURCHNL bit indicates the channel that is being processed in the TDMslot. Zero indicates the last slot.                                                                                                        |
| 6 (RC/NW)          | ACDONE     | All Channels Done. The FIR_DMASTAT.ACDONE bit indicates the accelerator that processing all chan- nels is complete. This is a sticky bit and is cleared on a register read. The FIR_CTL1.CCINTR bit does not affect the FIR_DMASTAT.ACDONE bit. |
| 5 (RC/NW)          | WDONE      | Channel Done. The FIR_DMASTAT.WDONE bit indicates the accelerator that processing the current channel is complete. This is a sticky bit and is cleared on a register read. The FIR_CTL1.CCINTR bit does not affect the FIR_DMASTAT.WDONE bit.   |
| 4 (R/NW)           | WRBK       | Writing Back. The FIR_DMASTAT.WRBK bit indicates the accelerator is writing back the updated index registers.                                                                                                                                   |

Table 51-19: FIR\_DMASTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------|
| 3 (R/NW)           | PPGS       | MAC Processing in Progress. The FIR_DMASTAT.PPGS bit indicates MAC processing in progress.                         |
| 2 (R/NW)           | DLD        | Data Preload. The FIR_DMASTAT.DLD bit indicates data preloading.                                                   |
| 1 (R/NW)           | CLD        | Coefficient Loading. The FIR_DMASTAT.CLD bit indicates coefficient loading.                                        |
| 0 (R/NW)           | CPLD       | Chain Pointer Loading Status. The FIR_DMASTAT.CPLD bit indicates the state machine is in chain pointer load state. |

## FIR Input Data Base Register

The FIR\_INBASE register contains the input word base address with the lower two bits removed.

Figure 51-19: FIR\_INBASE Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000017_3b6379087dfc070681554db949c5b4e8a720e7c4ae3833c4ddec1b25840d7c92.png)

Table 51-20: FIR\_INBASE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 29:0 (R/W)         | VALUE      | Word Address with Lower 2 Bits Removed. The FIR_INBASE.VALUE bit field contains the the word address with the lower 2 bits removed. |

## FIR Input Data Count Register

The FIR\_INCNT register contains the 16-bit input data count.

Figure 51-20: FIR\_INCNT Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000018_aec4fca2068b94d4f3fa255f1d5a5f205bdb50b61f40208d92a61f6a17f34afd.png)

Table 51-21: FIR\_INCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                             |
|--------------------|------------|---------------------------------------------------------------------|
| 15:0               | VALUE      | 16-bit Input Data Count.                                            |
| (R/W)              |            | The FIR_INCNT.VALUE bit field contains the 16-bit input data count. |

## FIR Input Data Index Register

The FIR\_INIDX register contains the input word address with the lower two bits removed.

Figure 51-21: FIR\_INIDX Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000019_3b6379087dfc070681554db949c5b4e8a720e7c4ae3833c4ddec1b25840d7c92.png)

Table 51-22: FIR\_INIDX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------|
| 29:0 (R/W)         | VALUE      | Word Address with Lower 2 Bits Removed. The FIR_INIDX.VALUE bit field contains the input word address with the lower two bits removed. |

## FIR Input Data Modifier Register

The FIR\_INMOD register contains the 16-bit input data buffer modifier.

Figure 51-22: FIR\_INMOD Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000020_787e7906c4494ee488f702cacf1742090af6514270bb327183e1d7167ab7e0e5.png)

Table 51-23: FIR\_INMOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                       |
|--------------------|------------|-------------------------------------------------------------------------------|
| 15:0               | VALUE      | 16-bit Input Data Buffer Modifier.                                            |
| (R/W)              |            | The FIR_INMOD.VALUE bit field contains the 16-bit input data buffer modifier. |

## FIR MAC Status Register

The FIR\_MACSTAT register provides the status of MAC operations. The status of all four multipliers/adders are available separately for programs to poll. In fixed-point mode only, the ARIx bits are used (all other bits are reserved).

Figure 51-23: FIR\_MACSTAT Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000021_7aba2d5584f490ff79e7098e8d91692b70aa7ea90a2e4c231ea6c8dc61ce5432.png)

Table 51-24: FIR\_MACSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                              |
|--------------------|------------|----------------------------------------------------------------------|
| 23                 | AINV3      | Addition Invalid.                                                    |
| (RC/NW)            |            | The FIR_MACSTAT.AINV3 bit is set if the adder 3 addition is invalid. |

Table 51-24: FIR\_MACSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 22 (RC/NW)         | ARI3       | Adder Result Overflow. The FIR_MACSTAT.ARI3 bit is set if the adder 3 result is infinity. Indicates over- flow in fixed-point mode. |
| 21 (RC/NW)         | ARZ3       | Adder Result Underflow. The FIR_MACSTAT.ARZ3 bit is set if the adder 3 result is zero.                                              |
| 20 (RC/NW)         | MINV3      | Multiply Invalid. The FIR_MACSTAT.MINV3 bit is set if the multiplier 3 multiply operation is inva- lid.                             |
| 19 (RC/NW)         | MRI3       | Multiplier Result Overflow. The FIR_MACSTAT.MRI3 bit is set if the multiplier 3 result is infinity.                                 |
| 18 (RC/NW)         | MRZ3       | Multiplier Result Underflow. The FIR_MACSTAT.MRZ3 bit is set if the multiplier 3 result is zero.                                    |
| 17 (RC/NW)         | AINV2      | Addition Invalid. The FIR_MACSTAT.AINV2 bit is set if the adder 2 addition is invalid.                                              |
| 16 (RC/NW)         | ARI2       | Adder Result Overflow. The FIR_MACSTAT.ARI2 bit is set if the adder 2 result is infinity. Indicates over- flow in fixed-point mode. |
| 15 (RC/NW)         | ARZ2       | Adder Result Underflow. The FIR_MACSTAT.ARZ2 bit is set if the adder 2 result is zero.                                              |
| 14 (RC/NW)         | MINV2      | Multiply Invalid. The FIR_MACSTAT.MINV2 bit is set if the multiplier 2 multiply operation is inva- lid.                             |
| 13 (RC/NW)         | MRI2       | Multiplier Result Overflow. The FIR_MACSTAT.MRI2 bit is set if the multiplier 2 result is infinity.                                 |
| 12 (RC/NW)         | MRZ2       | Multiplier Result Underflow. The FIR_MACSTAT.MRZ2 bit is set if the multiplier 2 result is zero.                                    |
| 11 (RC/NW)         | AINV1      | Addition Invalid. The FIR_MACSTAT.AINV1 bit is set if the adder 1 addition is invalid.                                              |
| 10 (RC/NW)         | ARI1       | Adder Result Overflow. The FIR_MACSTAT.ARI1 bit is set if the adder 1 result is infinity. Indicates over- flow in fixed-point mode. |
| 9 (RC/NW)          | ARZ1       | Adder Result Underflow. The FIR_MACSTAT.ARZ1 bit is set if the adder 1 result is zero.                                              |

Table 51-24: FIR\_MACSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 8 (RC/NW)          | MINV1      | Multiply Invalid. The FIR_MACSTAT.MINV1 bit is set if the multiplier 1 multiply operation is inva- lid.                             |
| 7 (RC/NW)          | MRI1       | Multiplier Result Overflow. The FIR_MACSTAT.MRI1 bit is set if the multiplier 1 result is infinity.                                 |
| 6 (RC/NW)          | MRZ1       | Multiplier Result Underflow. The FIR_MACSTAT.MRZ1 bit is set if the multiplier 1 result is zero.                                    |
| 5 (RC/NW)          | AINV0      | Addition Invalid. The FIR_MACSTAT.AINV0 bit is set if the adder 0 addition is invalid.                                              |
| 4 (RC/NW)          | ARI0       | Adder Result Overflow. The FIR_MACSTAT.ARI0 bit is set if the adder 0 result is infinity. Indicates over- flow in fixed-point mode. |
| 3 (RC/NW)          | ARZ0       | Adder Result Underflow. The FIR_MACSTAT.ARZ0 bit is set if the adder 0 result is zero.                                              |
| 2 (RC/NW)          | MINV0      | Multiply Invalid. The FIR_MACSTAT.MINV0 bit is set if the multiplier 0 multiply operation is inva- lid.                             |
| 1 (RC/NW)          | MRI0       | Multiplier Result Overflow. The FIR_MACSTAT.MRI0 bit is set if the multiplier 0 result is infinity.                                 |
| 0 (RC/NW)          | MRZ0       | Multiplier Result Underflow. The FIR_MACSTAT.MRZ0 bit is set if multiplier 0 result is zero.                                        |

## FIR Output Data Base Register

The FIR\_OUTBASE register contains the output word base address with the lower two bits removed

Figure 51-24: FIR\_OUTBASE Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000022_3b6379087dfc070681554db949c5b4e8a720e7c4ae3833c4ddec1b25840d7c92.png)

Table 51-25: FIR\_OUTBASE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------|
| 29:0 (R/W)         | VALUE      | Word Address with Lower 2 Bits Removed. The FIR_OUTBASE.VALUE bit field contains the word address with the lower 2 bits removed. |

## FIR Output Data Count Register

The FIR\_OUTCNT register contains the 16-bit output buffer count.

Figure 51-25: FIR\_OUTCNT Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000023_0066ffd0a49bac934b16ce6c52a7ba837cdd19e969a0465c632f888d4d199543.png)

Table 51-26: FIR\_OUTCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                 |
|--------------------|------------|-------------------------------------------------------------------------|
| 15:0               | VALUE      | 16-bit Output Buffer Count.                                             |
| (R/W)              |            | The FIR_OUTCNT.VALUE bit field contains the 16-bit output buffer count. |

## FIR Output Data Index Register

The FIR\_OUTIDX register contains the output word address with the lower two bits removed.

Figure 51-26: FIR\_OUTIDX Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000024_3b6379087dfc070681554db949c5b4e8a720e7c4ae3833c4ddec1b25840d7c92.png)

Table 51-27: FIR\_OUTIDX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 29:0 (R/W)         | VALUE      | Word Address with Lower 2 Bits Removed. The FIR_OUTIDX.VALUE bit field contains the the word address with the lower 2 bits removed. |

## FIR Output Data Modifier Register

The FIR\_OUTMOD register contains the 16-bit output data buffer modifier.

Figure 51-27: FIR\_OUTMOD Register Diagram

![Image](54_FIR_Accelerator_(FIR)_artifacts/image_000025_986eb4169e5e5d72cdab45c1115692ad6f34fc2fb52e0d06e13d7f11c3d96704.png)

Table 51-28: FIR\_OUTMOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                  |
|--------------------|------------|--------------------------------------------------------------------------|
| 15:0               | VALUE      | 16-bit Output Data Modifier.                                             |
| (R/W)              |            | The FIR_OUTMOD.VALUE bit field contains the 16-bit output data modifier. |