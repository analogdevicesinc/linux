## 50   FFT Accelerator (FFTA)

The Fast Fourier Transform Accelerator (FFTA) performs memory to memory FFT/IFFT operations without core software intervention. Additionally, the FFTA architecture allows execution of complex, pipelined, memory to memory algorithms including ping-ponged, windowed frequency domain filtering and very large FFTs. The FFTA may also be used in conjunction with minimal computation support from a core in applications such as the overlap-add operations required for large frequency-domain-based convolutions.

## FFTA Features

The following list describes the FFTA features.

- Supports both complex and real FFT and IFFT operations.
- Supports 64, 128, 256, 512, 1024, 2048 points in small FFT mode and 4096, 8192, 16384, 32768, 65536, 131072, 262144, 524288, 1048576, 2097152, 4194304 points in large FFT mode.
- Supports the IEEE-754/854 single-precision floating-point data format, round to even.
- Radix-4 butterfly efficiency at a radix-2 (integer power of two) point granularity.
- Automatic insertion of zeros for real FFTs.
- Supports automatic conjugating of the twiddle factors for IFFT.
- Supports automatic scaling of FFT and IFFT inputs.
- Hardware support for windowing and frequency domain filtering.
- Hardware support for magnitude squared FFT output.
- Hardware support for pipelined data flow.
- Dedicated high speed DMA engines for data load and dump with a data width 64-bit clocked by SYSCLK.
- Supports data and coefficient access from both on-chip (L1/L2) and off-chip memories (L3).
- Optional support for bypassing the compute engine to perform high speed memory-to-memory MDMA transfers.
- Clock division options for power reductions. Supports 1:1, 1:2, 1:4 and 1:8 clock ratio modes.

## FFTA Functional Description

The FFTA module provides the following functionality.

## Complex and real FFT and IFFT operations

The inputs and outputs for the complex FFT routines are packed arrays of floating point numbers. In a packed array, the real and imaginary parts of each complex number are placed in alternate neighboring elements.

## Hardware support for windowing, frequency domain filtering and pipelined data flow

Windowing is a technique used to shape the time portion of measurement data, to minimize edge effects that result in spectral leakage in the FFT spectrum. By using Window functions correctly, the spectral resolution of the frequency-domain result increases. Data pipelining helps to process one set of frames while other is being moved into and out of the accelerator via DMA.

Dedicated high speed DMA engines for data load and dump with a data width 64-bit is clocked by SYSCLK. This enables faster data movement between the memory internal to the accelerator and memories external to the accerator (L1/L2/L3).

## Supports data and coefficient access from both on-chip (L1/L2) and off-chip memories (L3)

This functionality helps to process larger data buffers which cannot fit within the limited size of the L1/L2 memories.

## ADSP-SC58x FFTA Register List

High-Speed FFT Compute Unit

Table 50-1: ADSP-SC58x FFTA Register List

| Name              | Description                      |
|-------------------|----------------------------------|
| FFTA_CTL          | Control Register                 |
| FFTA_INST[nn]     | Instruction Memory Register      |
| FFTA_LC[nn]       | Loop Counter Value Register      |
| FFTA_PC           | Program Counter Register         |
| FFTA_SCALE        | FFT/IFFT Scale Factor Register   |
| FFTA_STAT         | Status Register                  |
| FFTA_THREADOFFSET | Thread Count Offset Register     |
| FFTA_WCTL         | Wrapper Control Register         |
| FFTA_XFRLEFT[nn]  | Load/Dump Transfer Left Register |

## FFTA Definitions

To make the best use of the FFTA, it is useful to understand the following terms.

## Small FFT

Small FFT corresponds to the FFT/IFFT operation where number of points are less than or equal to 2048. These operations can be accomplished directly with the local memory supported by the FFTA.

## Large FFT

Large FFT corresponds to the FFT/IFFT operation where the number of points is greater than 2048. These operations are not directly supported by the FFTA because of the limited local memory. These are carried out in multiple stages using a divide and conquer approach by performing a number of small FFT operations.

## CCES

CrossCore Embedded Studio

## RTL

CrossCore Embedded Studio C/C++ Runtime Library

## CCES RTL Manual

CrossCore Embedded Studio C/C++ Compiler and Library Manual for SHARC® Processors

## ADSP-SC58x FFTA Interrupt List

Table 50-2: ADSP-SC58x FFTA Interrupt List

|   Interrupt ID | Name            | Description             | Sensitivity   |   DMA Channel |
|----------------|-----------------|-------------------------|---------------|---------------|
|            150 | FFTA0_TXDMA     | FFTA0 TransmitDMA       |               |            41 |
|            151 | FFTA0_RXDMA     | FFTA0 ReceiveDMA        |               |            42 |
|            152 | FFTA0_STAT      | FFTA0 Status            |               |               |
|            210 | FFTA0_TXDMA_ERR | FFTA0 Transmit DMAError |               |               |
|            211 | FFTA0_RXDMA_ERR | FFTA0 Receive DMAError  |               |               |

## ADSP-SC58x FFTA Trigger List

Table 50-3: ADSP-SC58x FFTA Trigger List Masters

|   Trigger ID | Name          | Description       | Sensitivity   |
|--------------|---------------|-------------------|---------------|
|           57 | FFTA0_TXDMA   | FFTA0 TransmitDMA |               |
|           58 | FFTA0_RXDMA   | FFTA0 ReceiveDMA  |               |
|           59 | FFTA0_TRIGOUT | FFTA0 Trigger Out |               |

Table 50-4: ADSP-SC58x FFTA Trigger List Slaves

|   Trigger ID | Name        | Description       | Sensitivity   |
|--------------|-------------|-------------------|---------------|
|           44 | FFTA0_TXDMA | FFTA0 TransmitDMA | Pulse         |
|           45 | FFTA0_RXDMA | FFTA0 ReceiveDMA  | Pulse         |

## FFTA Block Diagram

The FFTA contains a compute engine that operates in conjunction with a High Speed Distributed DMA Engine (HS-DDE). The compute engine appears as a data sink and data source to the two dedicated DDEs. The DDEs stream data into and out of the engine's FIFOs. To support faster data movement, the HS-DDE engine has a bus width of 64 bits and runs at SYSCLK speed. The compute engine data bus is 256 bits wide. The data transfer between the DMA engines and compute engine happens via data pack and unpack logic.

The FFTA compute engine runs at FFTCLK. The value of FFTCLK can go up to a maximum of SYSCLK. However, to achieve lower power consumption, it is also possible to operate the compute engine at frequencies lower than SYSCLK. This can be done by programming the FFTCLK:SYSCLK frequency ratio in the FFTA\_WCTL.FFTCLKRATIO bit bield.

The two channel high speed DMA engines can also (optionally) be used for high speed memory to memory DMA (MDMA) transfer if the FFT compute engine is not used in the system.

Figure 50-1: FFTA Block Diagram

![Image](53_FFT_Accelerator_(FFTA)_artifacts/image_000000_1cfda0eb83e6ebf23d540e602ae2541ead9c40e48a41e45a38f2f0f9e4cae0be.png)

## FFTA Programming Model

A software library containing support for using the FFTA is provided with the CrossCore Embedded Studio. Using this library, C or C++ programs running on an ARM Cortex A5 or SHARC+ ADSP-215xx and ADSP-SC5xx core can access the FFTA to implement various specific use scenarios as discussed in the following sections. Analog Devices Inc. does not support programming the FFTA at register level (except for the optional programming of the FFTA\_WCTL register). The detailed description of the FFTA registers is provided in this chapter for help with debugging.

## FFTA Use Cases

The following section provides the conceptual description of the FFTA use cases supported by the CCES RTL APIs. For exact description on the programming model for using the APIs, refer to the CrossCore Embedded Studio C/C++ Library Manual for SHARC Processors manual.

1. Single-Shot FFT
2. Pipelined small FFT
3. Pipelined small interleaved FFT and IFFT operations

## Single-Shot FFT

This use case is equivalent to the case where the core calls the CCES RTL's FFT/IFFT core library function. An API is called to perform an FFT/IFFT operation on a set of data and wait for the FFT/IFFT processing to finish. In this case all three units of the FFTA (input, compute, and output) operate in a sequential manner as illustrated by the Single-Shot figure. Notice how each frame passes through the stages of the FFTA block (input &gt; compute &gt; output) sequentially. The total time required to complete the FFT operation is equal to the time taken to load the input data

plus time taken to compute the FFT/IFFT plus the time taken to dump the output data. CCES provides an FFTA version of these functions/APIs and the APIs supporting this use case always operate in synchronous mode.

Figure 50-2: Single-Shot

![Image](53_FFT_Accelerator_(FFTA)_artifacts/image_000001_1284f3a64e6fe6abb44489f0c43337f719d8a118d3f4a39210ba65e2ee2664a4.png)

## Pipelined Small FFT

The piplined small FFT is used where an FFT/IFFT operation with a fixed number of points is performed continuously on more than one set of input data. The FFTA architecture supports pipelining of the input and output data which allows the FFTA to load one set of input data and dump another set of processed output data in parallel while another set of data (fetched in the previous pipeline cycle) is being processed. This helps to suppress the input and output DMA overheads while the FFTA compute engine is busy processing the data. This way, programs can take maximum advantage of the FFT compute engine's performance.

To support such cases, the FFTA APIs provide a mechanism where the FFTA is configured one time to perform the FFT/IFFT operation with a fixed number of points in a continuous pipelined manner. Next, the APIs related to the data transfers (asynchronous) are called to send the input data to and collect the output data from the FFTA. After all the data is processed, the FFTA can be closed. For more details on these APIs, refer to the CCES RTL manual.

The Pipelined Small FFT figure illustrates this use case. There is large startup latency (Input DMA time + Compute time + Output DMA time) involved to fill the pipeline with three frames. After that, each frame can be processed in a lesser steady state time (maximum of Input DMA time, Compute time, and Output DMA time).

Figure 50-3: Pipelined Small FFT

![Image](53_FFT_Accelerator_(FFTA)_artifacts/image_000002_d1244e64059c68a8d2baf4e2dba7249f28357a2ba21521f93fdc021e49c9f5cc.png)

## Pipelined Small Interleaved FFT and IFFT Operations

Pipelined small interleaved FFT and IFFT operations are useful when data is first converted to the frequency domain, processed in the frequency domain and converted back to the time domain. The Typical Data Flow

Frequency Domain Processing figure illustrates this where a particular input data frame N is first converted to the frequency domain (N') with an FFT operation. This FFT output is then processed in the frequency domain to produce the output frame N''. The processed output frame (N'') is then converted in to time domain output frame (N''').

Figure 50-4: Typical Data Flow Frequency Domain Processing

![Image](53_FFT_Accelerator_(FFTA)_artifacts/image_000003_529d145f9ff8a0c7ad18914b32ae7e2fece2156e20a9f3305b75a255a9724d9a.png)

The FFTA can be used to perform the above operation on multiple sets of data in a pipelined manner as shown in the Pipelined Interleaved FFT/IFFT figure. Unlike in the Pipelined Small FFT section, the compute engine performs both FFT and IFFT operations together in an interleaved manner. Note that the input data for the first valid IFFT operation (1'') is available only when the FFT output of the first frame (1') is processed by the core. It is necessary to send dummy input data at the start of the pipeline for the initial few IFFT operations as shown in the Pipelined Interleaved FFT/IFFT figure. The output of these IFFT operations can be ignored. Similarly, dummy data must be sent at the end of the pipeline for the last few FFT operations. The output of these FFT operations can be ignored.

Figure 50-5: Pipelined Interleaved FFT/IFFT

| INPUT (FFTA)      | 1   |   x | 2   | x   | 3   | 1'   | 4   | 2'   |
|-------------------|-----|-----|-----|-----|-----|------|-----|------|
| COMPUTE (FFTA)    |     |   1 | x   | 2   | x   | 3    | 1'  | 4    |
| OUTPUT (FFTA)     |     |     | 1'  | x   | 2'  | x    | 3'  | 1''' |
| PROCESSING (CORE) |     |     |     |     |     | 2'   |     | 3'   |

| INPUT (FFTA)      | N     | N-2'   | N+1'   | N-1'   | N+2'   | N''   | N+3   | N+1'   |
|-------------------|-------|--------|--------|--------|--------|-------|-------|--------|
| COMPUTE (FFTA)    | N-3'' | N      | N-2'   | N+1    | N-1'   | N+2   | N     | N+3'   |
| OUTPUT (FFTA)     | N-1'  | N-3''' | N'     | N-2''' | N+1'   | N-1'' | N+2'  | N'''   |
| PROCESSING (CORE) | N-2'  | N-1'   | N-1'   | N'     | N'     | N+1'  | N+1'  | N+2'   |

| INPUT (FFTA)      | P     | P-2'   | x    | P-1'   | x    | P''   | x   | x    |
|-------------------|-------|--------|------|--------|------|-------|-----|------|
| COMPUTE (FFTA)    | P-3'' | P      | P-2' | x      | P-1' | x     | P'' | x    |
| OUTPUT (FFTA)     | P-1'  | P-3''' | P'   | P-2''  | x'   | P-1'' | x   | P''' |
| PROCESSING (CORE) | P-2'  | P-1'   |      | P'     |      |       |     |      |

![Image](53_FFT_Accelerator_(FFTA)_artifacts/image_000004_8d3fc8b642ab2a81011a6cb2313160b761c3e09a34f2f21502483c7970f75244.png)

Similar to the Pipelined Small FFT section, the FFTA APIs provide a mechanism where the FFTA can be configured once to perform interleaved FFT and IFFT operations with a fixed number of points in a continuous pipelined manner. After that, programs just need to call the APIs related only to the data transfers (asynchronous) to send the

input data to and collect the output data from the FFTA. After all the data is processed, the FFTA can be closed. For more details on these APIs, refer to the CCES RTL manual.

## Fast Convolution

The FFTA can also be used to perform FIR filtering with the help of fast convolution. In this method, FFTA converts the input data in frequency domain, multiplies it with the FIR coefficients pre-transformed in the frequency domain, and then performs IFFT to return output in the time domain. The core is responsible for maintaining the output delay line and performing overlap-add operation.

For example, the FFTA Block Diagram shows how a block of 512 input samples is processed with an FIR filter of 8192 taps with the help of fast convolution method.

Figure 50-6: FFTA Block Diagram

![Image](53_FFT_Accelerator_(FFTA)_artifacts/image_000005_4b62421d91bdf71e2055f2dabd673e1b96d1811f7bf504109b515e6544ad7a6a.png)

The steps involved are as follows:

1. The FIR coefficients are divided in to 16 (=8192/512) sections h(0), h(1), h(2),…..h(15). Each section is padded with 512 zeros to generate hz(0), hz(1), hz(2),….hz(15) and then converted into the frequency domain (1024 point FFT) to generate H(0), H(1), H(2),….H(15).
2. Each time a new block of 512 samples x(n) is received, it is first padded with 512 zeros and the converted to frequency domain (1024 point FFT) to generate X(N).
3. X(N) is then multiplied with each of the 16 sections H(0), H(1), H(2),….H(15) to generate X(N)H(0), X(N)H(1), X(N)H(2),…. X(N)H(15).
4. All the 16 blocks of the result in 3. are then converted in to time domain (1024 point IFFT) to generate the output sections y'(0), y'(1),y'(2)……y'(15) each containing 1024 output samples.

5. The output sections generated in 4. are overlapped and added with the existing output delay line.
6. The left most 512 samples from the output delay line generated in 6. are extracted to generate 512 output samples corresponding to the original 512 input samples.
7. The delay line is then shifted left by 512 samples which is used as an input for the next processing iteration.

## ADSP-SC58x FFTA Register Descriptions

High-Speed FFT Compute Unit (FFTA) contains the following registers.

Table 50-5: ADSP-SC58x FFTA Register List

| Name              | Description                      |
|-------------------|----------------------------------|
| FFTA_CTL          | Control Register                 |
| FFTA_INST[nn]     | Instruction Memory Register      |
| FFTA_LC[nn]       | Loop Counter Value Register      |
| FFTA_PC           | Program Counter Register         |
| FFTA_SCALE        | FFT/IFFT Scale Factor Register   |
| FFTA_STAT         | Status Register                  |
| FFTA_THREADOFFSET | Thread Count Offset Register     |
| FFTA_WCTL         | Wrapper Control Register         |
| FFTA_XFRLEFT[nn]  | Load/Dump Transfer Left Register |

## Control Register

The FFTA\_CTL register is used to reset, start, pause and single step the HPFP-FFT engine.

Figure 50-7: FFTA\_CTL Register Diagram

![Image](53_FFT_Accelerator_(FFTA)_artifacts/image_000006_1c92ba6fdac28495f3058f9cbf9a0b9e2f96a77ecb4a3d4b8500476f02278e05.png)

Table 50-6: FFTA\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | STEPEN     | Single-Step Enable. When the FFTA_CTL.STEPEN and FFTA_CTL.EN bits are set together the en- gine to release the instruction currently pointed to by the PC to execution, increment the PC then stop execution. The FFTA_CTL.STEPEN and FFTA_CTL.EN bits are automatically cleared when the released instruction completes execution. The FFTA_CTL.EN bit or FFTA_CTL.STEPEN bit can then be read to determine if the instruction step has completed. Note that a single step advances the PC one incre- ment and at least one thread completes execution but one or two other threads may still not have completed execution. The FFTA_CTL.STEPEN control bit is self-clear- ing. 0 Single-Step Disabled |
| 1 (R/W)            | RST        | Reset HPFP-FFT Engine. When the FFTA_CTL.RST bit is set all MMRs are cleared. The FFTA_CTL.RST bit is self-clearing.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 1 (R/W)            |            | 0 Not reset HPFP-FFT engine                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 1 (R/W)            |            | 1 Reset HPFP-FFT engine                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 0 (R/W)            | EN         | Enable HPFP-FFT Engine. The FFTA_CTL.EN bit must be set for the engine to process instructions. When the FFTA_CTL.EN bit is cleared the engine is in an idle state. HPFP-FFT engine                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 0 (R/W)            |            | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 0 (R/W)            |            | 1 Enable HPFP-FFT engine                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

## Instruction Memory Register

The FFTA\_INST[nn] MMR registers hold (up to 64) FFT instructions.

Figure 50-8: FFTA\_INST[nn] Register Diagram

![Image](53_FFT_Accelerator_(FFTA)_artifacts/image_000007_b3106b0213fb32dff7f86d920f47a7ef248ae243cd4c0f6ba40efbd940c0f973.png)

Table 50-7: FFTA\_INST[nn] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Instruction Value. The FFTA_INST[nn].VALUE bit field contains 64 instruction MMRto hold FFT instructions. |

## Loop Counter Value Register

The FFTA\_LC[nn] register holds the current value of the corresponding loop counter.

Figure 50-9: FFTA\_LC[nn] Register Diagram

![Image](53_FFT_Accelerator_(FFTA)_artifacts/image_000008_a0c579c74631be0a5e9a747ff37585a0de24b9d73fb2cd59b81bf7392025deab.png)

Table 50-8: FFTA\_LC[nn] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration       |
|--------------------|------------|-------------------------------|
| 24:0               | VALUE      | Current value of the counter. |
| (R/NW)             |            |                               |

## Program Counter Register

The FFTA\_PC register contains the MMR address offset of the current instruction in the instruction queue. The PC advances unless the instruction it points to is not acknowledged. When the instruction pointed to is acknowledged it starts the task and immediately advances. See the rules for instruction acknowledge in the Sequencer section of the Programming reference.

Note that the nop(), jumpCNZ(), load\_loop\_cntr(), incr\_thread\_offset() and load\_scale() instructions are immediately acknowledged. Also note that only 6 bits are used, which implies that this counter is incremented in the fashion of modulo 64. For example, If the current PC=63, after it is incremented by 1, PC=0.

Figure 50-10: FFTA\_PC Register Diagram

![Image](53_FFT_Accelerator_(FFTA)_artifacts/image_000009_09049e833beca352171df3ffd99ef7287c41c725e0e301d327d292058f41a5a6.png)

Table 50-9: FFTA\_PC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5:0 (R/NW)         | NXTSEQ     | Next Sequence Instruction Number. The FFTA_PC.NXTSEQ bits provide the value by which the PC advances unless the instruction it points to is not acknowledged. When the instruction pointed to is ac- knowledged it starts the task and immediately advances. See the rules for instruction acknowledgement in the Sequencer section. The nop(), jumpCNZ(), load_loop_cntr(), incr_thread_offset() and load_scale() instructions are immediately acknowledged. Note that only 6 bits are used, which implies that this counter is incremented in the fashion of modulo 64. For example, If the current PC=63, after it is incremented by 1, PC=0. |

## FFT/IFFT Scale Factor Register

The FFTA\_SCALE register is loaded using the load\_scale() instruction. The scale factor is in a single-precision IEEE format. On reset the SCALE is set to 1.0 (floating point).

Figure 50-11: FFTA\_SCALE Register Diagram

![Image](53_FFT_Accelerator_(FFTA)_artifacts/image_000010_056b5ed03a1fc54bb3cc6f558577ff8bf85efcae5bd2201d3ba04463454a5eb0.png)

Table 50-10: FFTA\_SCALE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Scale Value.              |
| (R/NW)             |            | Reset to 1.0              |

## Status Register

The FFTA\_STAT register indicates the status of FFT operations. Sticky status flags remain set until cleared by reset. In the status description the term math operation or math result refers to all multiply, add and subtract results including intermediate operations.

Figure 50-12: FFTA\_STAT Register Diagram

![Image](53_FFT_Accelerator_(FFTA)_artifacts/image_000011_379735ca65913091c19531884cb9d744af63961399a559e9833e843860c1a10c.png)

Table 50-11: FFTA\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------|
| 18 (R/NW)          | INVMAGIC   | Invalid Magic word in the initialization header. Invalid magic word in the initialization header.                       |
| 17:14 (R/NW)       | CTLSTATE2  | Thread 2 Control State. The FFTA_STAT.CTLSTATE2 bit indicates the data available state (reflects state of fft.valid_o). |

Table 50-11: FFTA\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |                                                                                                                         | Description/Enumeration                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------|
|                    |            | 5                                                                                                                       | multiply state                                                                                                          |
|                    |            | 6                                                                                                                       | maganitude squared state                                                                                                |
|                    |            | 7                                                                                                                       | generate twiddles state                                                                                                 |
|                    |            | 8                                                                                                                       | bit reversal state                                                                                                      |
| 13:10 (R/NW)       | CTLSTATE1  | Thread 1 Control State. The FFTA_STAT.CTLSTATE1 bit indicates the data available state (reflects state of fft.valid_o). | Thread 1 Control State. The FFTA_STAT.CTLSTATE1 bit indicates the data available state (reflects state of fft.valid_o). |
|                    |            | 0                                                                                                                       | idle state                                                                                                              |
|                    |            | 1                                                                                                                       | load coef state                                                                                                         |
|                    |            | 2                                                                                                                       | load data state                                                                                                         |
|                    |            | 3                                                                                                                       | dump data state                                                                                                         |
|                    |            | 4                                                                                                                       | fft state                                                                                                               |
|                    |            | 5                                                                                                                       | multiply state                                                                                                          |
|                    |            | 6                                                                                                                       | maganitude squared state                                                                                                |
|                    |            | 7                                                                                                                       | generate twiddles state                                                                                                 |
|                    |            | 8                                                                                                                       | bit reversal state                                                                                                      |
| 9:6 (R/NW)         | CTLSTATE0  | Thread 0 Control State. The FFTA_STAT.CTLSTATE0 bit indicates the data available state (reflects state of fft.valid_o). | Thread 0 Control State. The FFTA_STAT.CTLSTATE0 bit indicates the data available state (reflects state of fft.valid_o). |
|                    |            | 0                                                                                                                       | idle state                                                                                                              |
|                    |            | 1                                                                                                                       | load coef state                                                                                                         |
|                    |            | 2                                                                                                                       | load data state                                                                                                         |
|                    |            | 3                                                                                                                       | dump data state                                                                                                         |
|                    |            | 4                                                                                                                       | fft state                                                                                                               |
|                    |            | 5                                                                                                                       | multiply state                                                                                                          |
|                    |            | 6                                                                                                                       | maganitude squared state                                                                                                |
|                    |            | 7                                                                                                                       | generate twiddles state                                                                                                 |
|                    |            | 8                                                                                                                       | bit reversal state                                                                                                      |

Table 50-11: FFTA\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/NW)           | VLDOUT     | Data Available State. The FFTA_STAT.VLDOUT bit indicates that data is available in the OFIFO to be read out (reflects state of fft.valid_o).                                                                                                                                                                                 |
| 5 (R/NW)           | VLDOUT     | 0 Data Not Available                                                                                                                                                                                                                                                                                                         |
| 5 (R/NW)           | VLDOUT     | 1 Data Available                                                                                                                                                                                                                                                                                                             |
| 4 (R/NW)           | RDYOUT     | Ready for Data State. The FFTA_STAT.RDYOUT bit indicates that the IFIFO is ready to receive data (re- flects state of fft.ready_o).                                                                                                                                                                                          |
| 4 (R/NW)           | RDYOUT     | 0 Data Out Not Ready                                                                                                                                                                                                                                                                                                         |
| 4 (R/NW)           | RDYOUT     | 1 Data Out Ready                                                                                                                                                                                                                                                                                                             |
| 3 (R/NW)           | INVLDINST  | An Invalid Instruction Was Executed. The FFTA_STAT.INVLDINST bit indicates an invalid instruction was executed. Asserts a FFT_I interrupt output when the INVALIDINST flag is first set. This IN- VAILIDINST flag is sticky.                                                                                                 |
| 3 (R/NW)           | INVLDINST  | 0 No Invalid Instruction                                                                                                                                                                                                                                                                                                     |
| 3 (R/NW)           | INVLDINST  | 1 An Invalid Instruction Was Executed                                                                                                                                                                                                                                                                                        |
| 2 (R/NW)           | OVR        | Math Result Is Greater Than The Maximum Normalized Number. The FFTA_STAT.OVR bit indicates a math result after rounding has a magnitude greater than the maximum normalized number. Asserts a FFT_I interrupt output when the OFLOW flag is first set. This OFLOW flag is sticky.                                            |
| 2 (R/NW)           | OVR        | 0 No Overflow                                                                                                                                                                                                                                                                                                                |
| 2 (R/NW)           | OVR        | 1 Overflow                                                                                                                                                                                                                                                                                                                   |
| 1 (R/NW)           | UNDR       | Math Result Is Smaller Than The Minimum Normalized Number, But Not A Zero. The FFTA_STAT.UNDR bit indicates a math result after rounding has a magnitude less than the minimum normalized number, and it is not an exact zero. Asserts a FFT_I interrupt output when the UFLOW flag is first set. This UFLOW flag is sticky. |
| 1 (R/NW)           | UNDR       | 0 No Underflow                                                                                                                                                                                                                                                                                                               |
| 1 (R/NW)           | UNDR       | 1 Underflow                                                                                                                                                                                                                                                                                                                  |
| 0 (R/NW)           | INVLDIN    | NAN Input Data or Invalid Math Operation. The FFTA_STAT.INVLDIN bit indicates a NAN input data or invalid math opera- tion: (0 x infinity, infinity - infinity). Asserts FFT_I interrupt output when the INVA- LIDIN flag is first set. This INVALIDIN flag is sticky.                                                       |
| 0 (R/NW)           | INVLDIN    | 0 Input And Math Opererations Are Valid                                                                                                                                                                                                                                                                                      |
| 0 (R/NW)           | INVLDIN    | 1 Input And/Or Math Opererations Are Invalid                                                                                                                                                                                                                                                                                 |

## Thread Count Offset Register

The FFTA\_THREADOFFSET register is a modulo 3 added to an instruction's specified thread number to create the actual thread assignment. The incr\_thread\_offset() instruction will modulo 3 increment the thread offset register.

Figure 50-13: FFTA\_THREADOFFSET Register Diagram

![Image](53_FFT_Accelerator_(FFTA)_artifacts/image_000012_4ce1beadb9b7fc8df5776d65156522b1c3077a39104b6ebbcc18bc4d64f822a7.png)

Table 50-12: FFTA\_THREADOFFSET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 1:0                | VALUE      | Thread Offset Value.      |
| (R/NW)             |            |                           |

## Wrapper Control Register

The FFTA\_WCTL register contains bits that configure the FFTCLK divider value and enable the FFT engine.

Figure 50-14: FFTA\_WCTL Register Diagram

![Image](53_FFT_Accelerator_(FFTA)_artifacts/image_000013_7a2d30745c926d5908b5fcf89d24e80f78963855a754285c37b14944af8b1b22.png)

Table 50-13: FFTA\_WCTL Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                    |
|--------------------|-------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2:1 (R/W)          | FFTCLKRATIO | FFTCLK Divider Value. The FFTA_WCTL.FFTCLKRATIO bits provide the FFTCLK divider value which selects the FFTCLK:SYSCLK_0 frequency ratio. 0 FFTCLK:SYSCLK_0 = 1:1 1 FFTCLK:SYSCLK_0 = 1:2 2 |
| 2:1 (R/W)          | FFTCLKRATIO | FFTCLK:SYSCLK_0= 1:4                                                                                                                                                                       |
| 2:1 (R/W)          | FFTCLKRATIO | 3 FFTCLK:SYSCLK_0 = 1:8                                                                                                                                                                    |
| 0 (R/W)            | EN          | FFT Engine Enable. The FFTA_WCTL.EN bit, when set (=1) enables the FFT engine.                                                                                                             |
| 0 (R/W)            | EN          | 0 Disable FFT engine and enableMDMA                                                                                                                                                        |
| 0 (R/W)            | EN          | 1 Enable FFT engine and DisableMDMA                                                                                                                                                        |

## Load/Dump Transfer Left Register

The three FFTA\_XFRLEFT[nn] registers, one for each thread, contain the number of 64-bit data or coefficient loads or data dumps remaining if a load or dump instruction is executing in that thread. The value of each register ranges from 0 to 1023, which means (Load/dump transfers remaining for thread)/8. So load/dump remaining can cover the range of [0, 1023*8]=[0, 8184]. If no load or dump is executing in that thread then the value is 0. The values reflect the count at the start of the pipeline (stage N0).

Figure 50-15: FFTA\_XFRLEFT[nn] Register Diagram

![Image](53_FFT_Accelerator_(FFTA)_artifacts/image_000014_6e2ae0c456ac1bf26c75ceb5f70e6aa5ca2aa001280c593619eec4cce8c03fd6.png)

Table 50-14: FFTA\_XFRLEFT[nn] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                  |
|--------------------|------------|------------------------------------------|
| 12:0               | VALUE      | Load/Dump Transfer remaining For Thread. |
| (R/NW)             |            |                                          |