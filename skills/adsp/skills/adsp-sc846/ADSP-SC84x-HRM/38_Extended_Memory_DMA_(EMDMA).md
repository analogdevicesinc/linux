## 35   Extended Memory DMA (EMDMA)

The Extended Memory DMA engine can be used in applications that copy data in a non-sequential manner. This includes delay lines, scatter and gather, and circular access types.

NOTE: Previous SHARC processors featured external port DMA. Current SHARC products use EMDMA that can access all memory locations for source and destination DMA operations.

## EMDMA Features

EMDMA frees the processor core, allowing it to perform other operations while the data transfers between memories occurs as a background task.

EMDMA has the following features and capabilities.

- Standard mode DMA Transfer
- Chained mode with direction on-the-fly
- Tap list mode (scatter/gather)
- Delay line mode (write to read)
- All the DMA modes can operate in circular fashion
- In circular operation, some modes allow a write back of the index pointer for correct addressing of the next Transfer Control Block (TCB)

## EMDMA Functional Description

The following sections provide information on the functional operations and operating modes of EMDMA.

## Internal-to-internal DMA

This is accomplished by indexing all external parameter registers with internal addresses.

## DMA bursting

DMA supports burst transfers only when the modifier is 1, for any other modifier values, single accesses are performed. The burst size is not user configurable and is chosen by the processor for optimal performance. The maximum burst size is 8 and the minimum is 1.

## Transfer control blocks

The structure of a TCB is conceptually the same as that of a traditional linked list. Each TCB has several data values and a pointer to the next TCB. Further, the chain pointer of a TCB may point to itself to continuously re-run the same DMA.

## Chain pointer DMA

In chained DMA operations, the processor automatically initializes and then begins another DMA transfer when the current DMA transfer is complete.

## ADSP-2184x EMDMA Register List

The EMDMA controllers support a variety of direct memory access operations which can access any system memory and transfer the entire block of data. A set of registers govern EMDMA operations. For more information on EMDMA functionality, see the EMDMA register descriptions.

Table 35-1: ADSP-2184x EMDMA Register List

| Name         | Description                         |
|--------------|-------------------------------------|
| EMDMA_BASE   | External Base Address Register      |
| EMDMA_BUFLEN | Circular Buffer Length Register     |
| EMDMA_CHNPTR | Chain Pointer Register              |
| EMDMA_CNT0   | Internal Count Register             |
| EMDMA_CNT1   | External Count Register             |
| EMDMA_CTL    | External Memory DMAControl Register |
| EMDMA_INDX0  | Internal Index Register             |
| EMDMA_INDX1  | External Index Register             |
| EMDMA_MOD0   | Internal Modifier Register          |
| EMDMA_MOD1   | External Modifier Register          |
| EMDMA_TCNT   | Delay Line Tap Count Register       |
| EMDMA_TPTR   | Tap List Pointer Register           |

## ADSP-2184x EMDMA Interrupt List

Table 35-2: ADSP-2184x EMDMA Interrupt List

|   Interrupt ID | Name        | Description          | Sensitivity   | DMA Channel   |
|----------------|-------------|----------------------|---------------|---------------|
|            106 | EMDMA0_DONE | EMDMA0 Transfer Done | Edge          |               |
|            107 | EMDMA1_DONE | EMDMA1 Transfer Done | Edge          |               |

## ADSP-2184x EMDMA Trigger List

Table 35-3: ADSP-2184x EMDMA Trigger List Generators

|   Trigger ID | Name        | Description    | Sensitivity   |
|--------------|-------------|----------------|---------------|
|           51 | EMDMA0_DONE | EMDMA0 DMADone | Edge          |
|           52 | EMDMA1_DONE | EMDMA1 DMADone | Edge          |

Table 35-4: ADSP-2184x EMDMA Trigger List Receivers

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         | None   | None          | None          |

## DMA Addressing

Besides the traditional internal-to-external addressing type, the DMA module also supports internal-to-internal transfers. This is accomplished by indexing all external parameter registers with internal addresses. The DMA controller recognizes the transfer by addresses and not by an additional control bit setting.

All the DMA addresses given by EMDMA are word-aligned byte addresses. The programming for the index registers is provided in the Register Descriptions section.

## DMA Burst Transfers

DMA supports burst transfers only when the modifier is 1. For any other modifier values, single accesses are performed.

The burst size is not user-configurable and is chosen by the processor for optimal performance. The maximum burst size is 8 and the minimum is 1.

The EMDMA uses appropriate burst transfer sizes for optimal throughput. For example, if the word count is 15, then 5 bursts are performed with burst sizes of 8+4+1+1+1 transfers.

## Transfer Control Block (TCB) Memory Storage

The location of the DMA parameters for the next sequence comes from the chain pointer register that points to the next set of DMA parameters stored in the processor's internal memory. In chained DMA operations, the processor automatically initializes and then begins another DMA transfer when the current DMA transfer is complete. Each new set of parameters is stored in a user-initialized memory buffer or TCB for a chosen peripheral.

## Chain Assignment

The structure of a TCB is conceptually the same as that of a traditional linked list. Each TCB has several data values and a pointer to the next TCB. Further, the chain pointer of a TCB may point to itself to continuously rerun the same DMA. The EMDMA reads each word of the TCB and loads it into the corresponding register. The end of the chain (no further TCBs are loaded) is indicated by a TCB with a chain pointer register value of zero.

The address field of the chain pointer registers is only 30 bits wide. If a program writes a symbolic address to bit 30 of the chain pointer there may be a conflict with the EMDMA\_CHNPTR.PCI bit. Programs should clear the upper bits of the address then AND the EMDMA\_CHNPTR.PCI bit separately, if needed.

## Starting Chain Loading

A DMA sequence is defined as the sum of the DMA transfer from when the parameter registers initialize to when the count register decrements to zero. The EMDMA module has a chaining enable bit ( EMDMA\_CTL.CHEN ).

To start the chain, write the internal (channel 0) index address of the first TCB to the chain pointer register ( EMDMA\_CHNPTR ). When chaining is enabled, DMA transfers are initiated by writing a memory address to the chain pointer register. This is also an easy way to start a single DMA sequence, with no subsequent chained DMAs.

During TCB chain loading, the EMDMA loads the DMA channel parameter registers with values retrieved from system memory.

The address in the chain pointer register points to the highest address of the TCB. This contains the internal (channel 0) index parameter. This means that if a program declares an array to hold the TCB, the chain pointer register should point to the last location of the array and not to the first TCB location.

## Buffered Chain Loading Register

The chain pointer register ( EMDMA\_CHNPTR ) is buffered. Before the chain loading starts, the buffer is copied into the chain pointer register and is decremented after each register is loaded.

The chain pointer register can be loaded with a new address at any time during the DMA sequence ( EMDMA\_CTL.CHEN =1). This allows a DMA channel to have chaining status deactivated (chain pointer register = 0x0) until some event occurs that loads the chain pointer register with a non-zero value. Writing all zeros to the address field of the chain pointer register also deactivates chaining for the next TCB.

## TCB Storage

The EMDMA supports several types of DMA, resulting in different lengths of TCBs. The TCB size varies from six locations (chained DMA) to 13 locations (delay line DMA). The EMDMA TCBs table shows the required TCBs for chained DMA.

In the following tables, TCB refers to the start address of the TCB array.

Table 35-5: EMDMA TCBs for Standard DMA

| Address   | Register     |
|-----------|--------------|
| TCB       | EMDMA_CHNPTR |
| TCB + 0x1 | EMDMA_MOD1   |
| TCB + 0x2 | EMDMA_INDX1  |
| TCB + 0x3 | EMDMA_CNT0   |
| TCB + 0x4 | EMDMA_MOD0   |
| TCB + 0x5 | EMDMA_INDX0  |

The order the descriptors are fetched with circular buffering enabled is shown in the EMDMA TCBs for Standard Circular DMA table.

Table 35-6: EMDMA TCBs for Standard Circular DMA

| Address   | Register     |
|-----------|--------------|
| TCB       | EMDMA_CHNPTR |
| TCB + 0x1 | EMDMA_BUFLEN |
| TCB + 0x2 | EMDMA_BASE   |
| TCB + 0x3 | EMDMA_MOD1   |
| TCB + 0x4 | EMDMA_INDX1  |
| TCB + 0x5 | EMDMA_CNT0   |
| TCB + 0x6 | EMDMA_MOD0   |
| TCB + 0x7 | EMDMA_INDX0  |

For delay line DMA, TCB loading is split into two sequences to improve overall priority. The first TCB loads the write parameters ( EMDMA\_INDX0 -EMDMA\_BUFLEN ) and the second loads the read parameters ( EMDMA\_INDX0 -EMDMA\_CHNPTR ). This two stage loading is transparent to the application. The order the descriptors are fetched for delay line DMA, as shown in the EMDMA TCBs for Delay Line DMA table.

Table 35-7: EMDMA TCBs for Delay Line DMA

| Address         | Register     |
|-----------------|--------------|
| Delay Line Read |              |
| TCB             | EMDMA_CHNPTR |
| TCB + 0x1       | EMDMA_TPTR   |
| TCB + 0x2       | EMDMA_TCNT   |
| TCB + 0x3       | EMDMA_MOD1   |
| TCB + 0x4       | EMDMA_CNT0   |

Table 35-7: EMDMA TCBs for Delay Line DMA (Continued)

| Address          | Register     |
|------------------|--------------|
| TCB + 0x5        | EMDMA_INDX0  |
| Delay Line Write |              |
| TCB + 0x6        | EMDMA_BUFLEN |
| TCB + 0x7        | EMDMA_BASE   |
| TCB + 0x8        | EMDMA_MOD1   |
| TCB + 0x9        | EMDMA_INDX1  |
| TCB + 0xA        | EMDMA_CNT0   |
| TCB + 0xB        | EMDMA_MOD0   |
| TCB + 0xC        | EMDMA_INDX0  |

The order the descriptors are fetched for scatter/gather DMA with circular buffering disabled is shown in the EMDMA TCBs for Scatter/Gather DMA table.

Table 35-8: EMDMA TCBs for Scatter/Gather DMA

| Address   | Register     |
|-----------|--------------|
| TCB       | EMDMA_CHNPTR |
| TCB + 0x1 | EMDMA_TPTR   |
| TCB + 0x2 | EMDMA_TCNT   |
| TCB + 0x3 | EMDMA_MOD1   |
| TCB + 0x4 | EMDMA_INDX1  |
| TCB + 0x5 | EMDMA_CNT0   |
| TCB + 0x6 | EMDMA_MOD0   |
| TCB + 0x7 | EMDMA_INDX0  |

The order the descriptors are fetched for scatter/gather DMA with circular buffering enabled is shown in the EMDMA TCBs for Circular Scatter/Gather DMA table.

Table 35-9: EMDMA TCBs for Circular Scatter/Gather DMA

| Address   | Register     |
|-----------|--------------|
| TCB       | EMDMA_CHNPTR |
| TCB + 0x1 | EMDMA_BUFLEN |
| TCB + 0x2 | EMDMA_BASE   |
| TCB + 0x3 | EMDMA_TPTR   |
| TCB + 0x4 | EMDMA_TCNT   |

Table 35-9: EMDMA TCBs for Circular Scatter/Gather DMA (Continued)

| Address   | Register    |
|-----------|-------------|
| TCB + 0x5 | EMDMA_MOD1  |
| TCB + 0x6 | EMDMA_INDX1 |
| TCB + 0x7 | EMDMA_CNT0  |
| TCB + 0x8 | EMDMA_MOD0  |
| TCB + 0x9 | EMDMA_INDX0 |

## EMDMA Operating Modes

This section and the EMDMA\_CTL Register Bit to Operating Modes table show the different DMA modes which can be used. The complete register bit descriptions are in the External Memory DMA Control Registers (EMDMA\_CTL) .

Table 35-10: EMDMA\_CTL Register Bit to Operating Modes

| Bit (Name)    | Standard     | Chained      | Scatter/Gather   | Delay Line   |
|---------------|--------------|--------------|------------------|--------------|
| Control Bits  | Control Bits | Control Bits | Control Bits     | Control Bits |
| 0 (EN)        | Valid        | Valid        | Valid            | Valid        |
| 1 (TRAN)      | Valid        | Valid        | Valid            | N/A          |
| 2 (CHEN)      | Valid        | Valid        | Valid            | Valid        |
| 3 (DLEN)      | N/A          | N/A          | N/A              | Valid        |
| 4 (CBEN)      | Valid        | Valid        | Valid            | Valid        |
| 5 (DFLSH)     | Valid        | Valid        | Valid            | Valid        |
| 6             | N/A          | N/A          | N/A              | N/A          |
| 7 (WRBEN)     | N/A          | Valid        | (=0)             | (=1)         |
| 8 (OFCEN)     |              | Valid        |                  | N/A          |
| 9 (TLEN)      | N/A          | N/A          | Valid            | N/A          |
| 11-10         | N/A          | N/A          | N/A              | N/A          |
| 12 (INTDONE0) | Valid        | Valid        | Valid            | Valid        |
| 15-13         | N/A          | N/A          | N/A              | N/A          |
| Status Bits   | Status Bits  | Status Bits  | Status Bits      | Status Bits  |
| 17-16 (DFS)   | Valid        | Valid        | Valid            | Valid        |
| 19-18         | N/A          | N/A          | N/A              | N/A          |
| 20 (DMAS0)    | Valid        | Valid        | Valid            | Valid        |
| 21 (CHS)      | N/A          | Valid        | N/A              | Valid        |

Table 35-10: EMDMA\_CTL Register Bit to Operating Modes (Continued)

| Bit (Name)   | Standard   | Chained   | Scatter/Gather   | Delay Line   |
|--------------|------------|-----------|------------------|--------------|
| 22 (TLS)     | N/A        | N/A       | Valid            | N/A          |
| 23 (WBS)     | N/A        | N/A       | N/A              | Valid        |
| 24 (DMAS1)   | Valid      | Valid     | Valid            | Valid        |
| 25 (DIRS)    | Valid      | Valid     | Valid            | Valid        |
| 31-26        | N/A        | N/A       | N/A              | N/A          |

NOTE: Reading additional bit-field information from N/A (Not applicable) bits does not generate a meaningful result.

A program sets up a DMA channel by writing the transfer's parameters to the DMA parameter registers.

NOTE: The EMDMA\_CNT1 parameter register (read only) is a copy of the EMDMA\_CNT0 register. If EMDMA\_CNT0 is written, the EMDMA\_CNT1 register is updated automatically.

## Standard DMA

A standard DMA (once it is configured) transfers data from location A to location B. An interrupt can be used to indicate the end of the transfer. To start a new DMA sequence after the current one is finished, a program must first clear the DMA enable bit ( EMDMA\_CTL.EN ), write new parameters to the index, modify, and count registers (parameter registers), then set the DMA enable bit to re-enable DMA.

This DMA type resembles the traditional DMA type to initialize the different internal and external parameters (channel0 and channel1) (index, modify and count) registers and configuration of the DMA control registers.

## Circular Buffered DMA

Circular buffered DMA resembles the traditional core DAG circular buffered mode by using registers for circular buffering. In this mode, the DMA needs two additional registers (base and length) to support reads and writes to a circular buffer. The Circular Buffering Write DMA and Circular Buffering Read DMA figures illustrate circular buffered DMA, in contrast with the Standard Write DMA figure.

Figure 35-1: Circular Buffering Write DMA

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000000_aa32b50f0cddc05e182f49cc8626733e5937764fc0c1c1716b1ba666ed55accd.png)

Figure 35-2: Circular Buffering Read DMA

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000001_d126a35b09f0353100696d7b4f0ea9efd5ec45f3120db07d9c0c5375e898fb78.png)

Figure 35-3: Standard Write DMA

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000002_62187b4c39d36aafe1ddc9e5be1945a42feb985cb52aaff8b8676010a3cc3674.png)

NOTE: Circular buffering is available for all operating modes (standard, chained, tap list and delay line DMA).

## Chained DMA Mode

Chained DMA sequences are a set of multiple DMA operations, each auto-initializing the next in line. It is used to support automated access by a linked list (repetitive reads and writes to a defined location defined by the individual TCBs). To start a new DMA sequence after the current one is finished, the EMDMA automatically loads new index, modify, and count values pointed to by that channel's EMDMA\_CHNPTR register. Using chaining, programs can set up consecutive DMA operations and each operation can have different attributes.

DMA data transfers can be set up as continuous or periodic. With chained DMA, the attributes of a specific DMA are stored in internal memory and are referred to as a T ransfer Control Block or TCB. Extended Memory DMA loads these attributes in chains for execution. This allows for multiple chains that are finite or infinite.

NOTE: When chaining is enabled, polling should not be used to determine DMA status only because the DMA appears inactive if it is sampled while the next TCB is loading. In such cases where chaining is enabled, along with the polling of DMA status bit, polling of the chaining status bit should also occur to so that the correct status of the DMA is known. For example, the EMDMA\_CTL.CHS bit should be polled as well as the EMDMA\_CTL.DMAS0 and EMDMA\_CTL.DMAS1 bits when EMDMA is configured in DMA chaining mode.

## Data Direction On-the-Fly

A change of external memory data direction for each individual TCB in a chain sequence is allowed.

The EMDMA\_CHNPTR.CPDR bit changes the data flow direction. If the EMDMA\_CHNPTR.CPDR =0, writes through channel 0 are performed; if EMDMA\_CHNPTR.CPDR =1, channel 0 reads are performed. This works similarly to the EMDMA\_CHNPTR.PCI bit. The EMDMA\_CTL.OFCEN and EMDMA\_CTL.CHEN bits must be set (=1) to enable this functionality.

NOTE: If chaining is enabled with the EMDMA\_CTL.OFCEN =1, then the EMDMA\_CTL.TRAN bit has no effect, and the data direction is determined by the EMDMA\_CHNPTR.CPDR bit.

## Write Back Circular Index Pointer

Operating the DMA in circular mode requires some special considerations. The index pointer of start address within the buffer may wrap around for the case when IC × IM &gt; EL or it does not finish if IC × IM &lt; EL where:

IC is the value of the EMDMA\_INDX0 register

IM is the value of the EMDMA\_MOD0 register

EL is the value of the EMDMA\_BUFLEN register

In both cases, the TCB start address is no longer valid.

Setting the EMDMA\_CTL.WRBEN bit writes (at the end of current TCB block) the current index address + 1 into the TCB memory which is the start address for the next TCB. This bit is only selectable for chained DMA mode. For tap list and delay line modes, this bit is hardwired to 0 or 1.

## Scatter/Gather DMA

The purpose of scatter/gather DMA is the transfer of data from/to non-contiguous memory blocks.

The scatter/gather DMA type is a fixed block size scatter/gather DMA that relies on tap list entries in system memory to calculate the (Channel 1 DMA Address) to scatter/gather the DMA. When the DMA direction is Channel 1 write ( EMDMA\_CTL.TRAN =1) it is a scatter DMA. When ( EMDMA\_CTL.TRAN =0), it is a gather DMA. This mode also supports chained and circular buffer chained DMAs.

See the Scatter DMA (Writes) , Gather DMA (Reads) , Circular Buffering Scatter DMA (Writes) , and Circular Gather DMA (Reads) figures.

Figure 35-4: Scatter DMA (Writes)

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000003_24b7f0577e09ce6d86b9dcb9f72797d53a5fcfdf095fd5aa62a1ea29ab6c2409.png)

Figure 35-5: Gather DMA (Reads)

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000004_b1c3a840d6f7125d94aa1fcbda3d439251bb8978b46411fa770cc1ced6ffbbf4.png)

Figure 35-6: Circular Buffering Scatter DMA (Writes)

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000005_24a887eec7d71e20f7034c838cdf9c2263af10937daf2de4cfe31bd086c01dd4.png)

Figure 35-7: Circular Gather DMA (Reads)

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000006_faa95c45a6707268d130729159b7a3778a129429f73009bdfaa82a81c1c805f4.png)

For each 32-bit tap read, the Channel 1 read index is shown in the Read/Write Index Pre-Modify (Scatter/Gather DMA) table. Note that one tap list entry starts multiple reads.

Table 35-11: Read/Write Index Pre-Modify (Scatter/Gather DMA)

| Pre-Modify Address Equation EMDMA_INDX0 + EMDMA_TPTR   | Result     | Result   |
|--------------------------------------------------------|------------|----------|
| [ EMDMA_TCNT ] + ( EMDMA_MOD1                          | Block Size | Tap      |
| EMDMA_INDX0 + EMDMA_TPTR [0] + EMDMA_MOD1 × 0          | N          | 0        |
| EMDMA_INDX0 + EMDMA_TPTR [0] + EMDMA_MOD1 × 1          | N          | 0        |
| EMDMA_INDX0 + EMDMA_TPTR [0] + EMDMA_MOD1 × 2          | N          | 0        |
| EMDMA_INDX0 + EMDMA_TPTR [0] + EMDMA_MOD1 ×N           | N          | 0        |
| EMDMA_INDX0 + EMDMA_TPTR [1] + EMDMA_MOD1 × 0          | N          | 1        |
| EMDMA_INDX0 + EMDMA_TPTR [1] + EMDMA_MOD1 × 1          | N          | 1        |
| EMDMA_INDX0 + EMDMA_TPTR [1] + EMDMA_MOD1 × 2          | N          | 1        |
| EMDMA_INDX0 + EMDMA_TPTR [1] + EMDMA_MOD1 ×N           | N          | 1        |
| EMDMA_INDX0 + EMDMA_TPTR [M] + EMDMA_MOD1 × 0          | N          | M        |
| EMDMA_INDX0 + EMDMA_TPTR [M] + EMDMA_MOD1 × 1          | N          | M        |
| EMDMA_INDX0 + EMDMA_TPTR [M] + EMDMA_MOD1 × 2          | N          | M        |
| EMDMA_INDX0 + EMDMA_TPTR [M] + EMDMA_MOD1 ×N           | N          | M        |

## Pre Modified Read/Write Index

For scatter/gather DMA, the tap list modifiers are employed, and the number of taps is determined by the tap list count register ( EMDMA\_TCNT ). The number of sequential reads (block size) from every tap is determined by the internal count register ( EMDMA\_CNT0 ), and is the same for every tap. The read/write pointer in external index register ( EMDMA\_INDX1 ) serves as the index address for these read/writes.

TL[N] is the first tap list entry in the memory as pointed to by the tap list pointer register ( EMDMA\_TPTR ). The tap list entries are 27-bit signed integers. For each read/write block, the DMA state machine fetches the offset from the tap list. The offset is added to the EMDMA\_INDX1 register value to get the start address of the next block. The Channel 1 addresses are circular buffered if circular buffering is enabled (see the Circular Buffering Scatter DMA (Writes) and Circular Buffering Gather DMA (Reads) figures in Scatter/Gather DMA).

Once the EMDMA\_CNT0 register for the final tap decrements to zero ( EMDMA\_TCNT and EMDMA\_CNT0 are zero), then the tap list DMA access is complete, and the DMA completion interrupt is generated (when chaining is enabled the interrupt depends on the EMDMA\_CHNPTR.PCI bit setting).

The write back mode ( EMDMA\_CTL.WRBEN bit) is hardwired to zero for tap list based DMA (as the addressing is pre-modify, and therefore the EMDMA\_INDX1 value coincides with the TCB value even at the end of the DMA).

## Delay Line DMA

Delay line DMA is used to support reads and writes to delay line buffers with limited core interaction. In this sense, delay line DMA is a quantity of integrated writes followed by reads from channel 1 (a delay line DMA access). Delay line DMA is described in the following sections.

- NOTE: Delay line DMA can only operate by using chained DMA mode ( EMDMA\_CTL.CHEN bit set). In order to use delay line DMA for a single DMA sequence, initialize the EMDMA\_CHNPTR register to zero in the TCB.

NOTE: Delay line DMA can be used in any system memory.

The delay line DMA operation follows these steps:

1. Load the first half of TCB for writing (seven parameters).
2. DMA writes to the delay line buffer until IC = 0.
3. Update the EI index pointer if circular mode is enabled.
4. Load the second half of TCB for reading (six parameters).
5. DMA tap based reads from the delay line buffer until RC = 0. Jump to step 1.

Writes to delay line. The number of writes is determined by the EMDMA\_CNT0 register. The data is fetched from the EMDMA\_INDX0 register and the EMDMA\_MOD0 register is used as the internal modifier. The EMDMA\_INDX1 register serves as the external index and is incremented by the EMDMA\_MOD1 register after each write. These writes are circular buffered if circular buffering is enabled. See the Write to Delay Line Buffer figure.

Figure 35-8: Write to Delay Line Buffer

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000007_bd772938f8bce42dc462ee4453d55d24c3cb7b80b555c2b9376f88674bd2ef1e.png)

When the writes are complete, ( EMDMA\_CNT0 =0) the EMDMA\_INDX1 register, which serves as the write pointer of the delay line, is written back ( EMDMA\_CTL.WRBEN is hardwired to 1) to the TCB location from where it was fetched.

Reads from the delay line. For reads, the tap list (TL) modifiers are used, and the number of reads is determined by the EMDMA\_CNT0 register. The write pointer in the EMDMA\_INDX1 register serves as the index address for these reads (reads start from where writes end). The EMDMA\_INDX1 register, along with tap list modifiers, are used in a pre-modify addressing mode to create the external address for the reads. Therefore, for each read, the DMA controller fetches the external modifier ( EMDMA\_TCNT register) from the tap list and the reads are circular buffered (if enabled). See the Read From Delay Line Buffer figure.

Figure 35-9: Read From Delay Line Buffer

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000008_7afd200208f68ad41fbd5715ced4232b63f3103a5b8ad49da757dbe4d2bd245b.png)

## Pre-Modified Read Index

Note that TL[N] is the first tap list entry in memory pointed to by the tap list pointer register ( EMDMA\_TPTR ). Tap list entries are 27-bit signed integers. Therefore, for each read-block, the DMA state machine fetches the offset external modifier from the tap list. The reads are circular buffered if circular buffering is enabled.

NOTE: The channel 1 DMA address generation follows pre-modify addressing for reads in delay line DMA and therefore the EMDMA\_INDX1 register values are not updated. Also the EMDMA\_MOD1 register does not have any effect during these delay line reads. Once the read count completes, the EMDMA\_CNT0 register decrements to zero (both EMDMA\_CNT0 and EMDMA\_TCNT are zero) for the final tap. Finally, the delay line DMA access completes, and the DMA completion interrupt is generated. When chaining is enabled, the interrupt is dependent on the EMDMA\_CHNPTR.PCI bit setting. The delay line DMA can only be initialized using the TCB. To use the delay line DMA for a single DMA sequence, initialize the EMDMA\_CHNPTR register to zero in the TCB.

For each 32-bit tap read, the channel 1 read index is shown in the Read/Write Index Pre-Modify (Scatter/Gather DMA) table. Note that one tap list entry starts multiple reads.

Table 35-12: Read/Write Index Pre-Modify (Scatter/Gather DMA)

| Pre-Modify Address Equation                                           | Result     | Result   |
|-----------------------------------------------------------------------|------------|----------|
| EMDMA_INDX1 + EMDMA_TPTR [ EMDMA_TCNT ] + ( EMDMA_MOD1 × EMDMA_CNT1 ) | Block Size | Tap      |
| EMDMA_INDX1 + EMDMA_TPTR [0] + EMDMA_MOD1 × 0                         | N          | 0        |
| EMDMA_INDX1 + EMDMA_TPTR [0] + EMDMA_MOD1 × 1                         |            |          |
| EMDMA_INDX1 + EMDMA_TPTR [0] + EMDMA_MOD1 × 2                         |            |          |
| EMDMA_INDX1 + EMDMA_TPTR [0] + EMDMA_MOD1 ×N                          |            |          |

Table 35-12: Read/Write Index Pre-Modify (Scatter/Gather DMA) (Continued)

| Pre-Modify Address Equation                                           | Result     | Result   |
|-----------------------------------------------------------------------|------------|----------|
| EMDMA_INDX1 + EMDMA_TPTR [ EMDMA_TCNT ] + ( EMDMA_MOD1 × EMDMA_CNT1 ) | Block Size | Tap      |
| EMDMA_INDX1 + EMDMA_TPTR [1] + EMDMA_MOD1 × 0                         | N          | 1        |
| EMDMA_INDX1 + EMDMA_TPTR [1] + EMDMA_MOD1 × 1                         | N          | 1        |
| EMDMA_INDX1 + EMDMA_TPTR [1] + EMDMA_MOD1 × 2                         | N          | 1        |
| EMDMA_INDX1 + EMDMA_TPTR [1] + EMDMA_MOD1 ×N                          | N          | 1        |
| EMDMA_INDX1 + EMDMA_TPTR [M] + EMDMA_MOD1 × 0                         | N          | M        |
| EMDMA_INDX1 + EMDMA_TPTR [M] + EMDMA_MOD1 × 1                         | N          | M        |
| EMDMA_INDX1 + EMDMA_TPTR [M] + EMDMA_MOD1 × 2                         | N          | M        |
| EMDMA_INDX1 + EMDMA_TPTR [M] + EMDMA_MOD1 ×N                          | N          | M        |

## ADSP-2184x EMDMA Register Descriptions

Extended Memory DMA (EMDMA) contains the following registers.

Table 35-13: ADSP-2184x EMDMA Register List

| Name         | Description                         |
|--------------|-------------------------------------|
| EMDMA_BASE   | External Base Address Register      |
| EMDMA_BUFLEN | Circular Buffer Length Register     |
| EMDMA_CHNPTR | Chain Pointer Register              |
| EMDMA_CNT0   | Internal Count Register             |
| EMDMA_CNT1   | External Count Register             |
| EMDMA_CTL    | External Memory DMAControl Register |
| EMDMA_INDX0  | Internal Index Register             |
| EMDMA_INDX1  | External Index Register             |
| EMDMA_MOD0   | Internal Modifier Register          |
| EMDMA_MOD1   | External Modifier Register          |
| EMDMA_TCNT   | Delay Line Tap Count Register       |
| EMDMA_TPTR   | Tap List Pointer Register           |

## External Base Address Register

The EMDMA\_BASE register contains the external base address of the delay line buffer. This is used for maintaining circular buffered read/writes to the delay line.

Figure 35-10: EMDMA\_BASE Register Diagram

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000009_d3cba279fac4786f901ca56bd3ce03fd40201bc9a1ecba169424ad4d4c68fdd5.png)

Table 35-14: EMDMA\_BASE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------|
| 29:0 (R/W)         | ADDR       | External Delay Line Base Address. The EMDMA_BASE.ADDR bit field contains the external base address of the delay line buffer. |

## Circular Buffer Length Register

The EMDMA\_BUFLEN register holds the circular buffer length for the delay line DMA.

Figure 35-11: EMDMA\_BUFLEN Register Diagram

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000010_e1145f9a0694e32899072d8650c55459844bbc585824f0f381b778eed4f0ac08.png)

Table 35-15: EMDMA\_BUFLEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------|
| 25:0 (R/W)         | CLEN       | Delay Line Circular Buffer Length. The EMDMA_BUFLEN.CLEN bit field holds the circular buffer length for the delay line DMA. |

## Chain Pointer Register

The EMDMA\_CHNPTR register contains the address of the next descriptor in memory when the EMDMA\_CTL.CHEN bit =1. This register also has bits to change DMA directions for each descriptor and the PCI bit.

Note that the lower 30-bits of this register are to be written with 30 MSB's of the word-aligned byte addresses of the corresponding next descriptor address.

Figure 35-12: EMDMA\_CHNPTR Register Diagram

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000011_e4c36fb2f9e4276ff0c3672904a7e3b9f0ba3fe2b09c5bbc14538a8ce39e0058.png)

Table 35-16: EMDMA\_CHNPTR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                     | Description/Enumeration                                                                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | CPDR       | CPDR DMADirection for the next TCB. The EMDMA_CHNPTR.CPDR bit configures whether the DMAis a write to internal memory or a read from internal memory. Note: this setting is applicable only if EMDMA_CTL.OFCEN =1 and is not applicable for delay line DMA. | CPDR DMADirection for the next TCB. The EMDMA_CHNPTR.CPDR bit configures whether the DMAis a write to internal memory or a read from internal memory. Note: this setting is applicable only if EMDMA_CTL.OFCEN =1 and is not applicable for delay line DMA. |
| 31 (R/W)           | CPDR       | 0                                                                                                                                                                                                                                                           | Write to Channel 0 (Channel 1 reads)                                                                                                                                                                                                                        |
| 30 (R/W)           | PCI        | Program Controlled Interrupt. The EMDMA_CHNPTR.PCI bit PCI sets whether an interrupt is generated after the current TCB or if no interrupt is generated. (Only affects DMAif chaining is enabled).                                                          | Program Controlled Interrupt. The EMDMA_CHNPTR.PCI bit PCI sets whether an interrupt is generated after the current TCB or if no interrupt is generated. (Only affects DMAif chaining is enabled).                                                          |
| 30 (R/W)           | PCI        | 0                                                                                                                                                                                                                                                           | Enable DMAChannel interrupt to occur at the comple- tion of the entire DMAchained transfer                                                                                                                                                                  |
| 30 (R/W)           | PCI        | 1                                                                                                                                                                                                                                                           | Enable DMAChannel interrupt to occur at the comple- tion of current DMAsequence                                                                                                                                                                             |

Table 35-16: EMDMA\_CHNPTR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------|
| 29:0 (R/W)         | ADDR       | Next Descriptor (Chain) Pointer Address. The EMDMA_CHNPTR.ADDR bit field provides the address of the next descriptor in memory. |

## Internal Count Register

The EMDMA\_CNT0 register contains the number of words to be transferred for channel 0 DMA.

Note: If delay line DMA is enabled then the EMDMA\_CNT0 register serves as the count register for the delay line writes.

Figure 35-13: EMDMA\_CNT0 Register Diagram

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000012_60deb08092d2126e2f2e12c605d6854ab8d1d0307f2475c9d1daf6a006275372.png)

Table 35-17: EMDMA\_CNT0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                              |
|--------------------|------------|------------------------------------------------------|
| 31:0               | VALUE      | DMAWord Count.                                       |
| (R/W)              |            | The EMDMA_CNT0.VALUE bit field is the DMAword count. |

## External Count Register

The EMDMA\_CNT1 register contains the number of words to be transferred for channel 1 DMA.

Note: If delay line DMA is enabled then the EMDMA\_CNT1 register serves as the count register for the delay line writes.

Figure 35-14: EMDMA\_CNT1 Register Diagram

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000013_b0e96e549babcd2570ea54e06d04b83c60bc2878d0f68c0283cdce0aefad4291.png)

Table 35-18: EMDMA\_CNT1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                   |
|--------------------|------------|-----------------------------------------------------------|
| 31:0               | CNT1       | External Word Count.                                      |
| (R/NW)             |            | The EMDMA_CNT1.CNT1 bit field is the external word count. |

## External Memory DMA Control Register

The EMDMA\_CTL register contains bits that enable and configure EMDMA and indicate DMA transfer status.

Figure 35-15: EMDMA\_CTL Register Diagram

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000014_062f9112e0713912cb1f851ed2c48fb42d3a5f5280949bc632f50b781dc0e7d5.png)

Table 35-19: EMDMA\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R/NW)          | DIRS       | DMATransfer Direction Status. The EMDMA_CTL.DIRS bit provides the DMAtransfer status direction. This is useful for delay line DMAwhere the transfer direction changes with the state of the DMAstate machine. For standard DMA, the EMDMA_CTL.DIRS bit reflects the state of the EMDMA_CTL.TRAN bit. |
| 25 (R/NW)          | DIRS       | 0 DMAdirection is Channel 1 Reads                                                                                                                                                                                                                                                                    |
| 25 (R/NW)          | DIRS       | 1 DMAdirection is Channel 1 Writes                                                                                                                                                                                                                                                                   |

Table 35-19: EMDMA\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| 24 (R/NW)          | DMAS1      | DMAExternal Interface Status. The EMDMA_CTL.DMAS1 bit provides the DMAchannel 1 transfer status. 0 Channel 1 DMAdoes not have any access pending |
| 23 (R/NW)          | WBS        | Write Back Status. The EMDMA_CTL.WBS bit provides the delay line write pointer write back status. 0 Write pointer write back is not active       |
| 22 (R/NW)          | TLS        | TAP List Loading Status. The EMDMA_CTL.TLS bit provides the DMAtap list loading status. 0 TAP list loading is not active                         |
| 21 (R/NW)          | CHS        | DMAChaining Status. The EMDMA_CTL.CHS bit provides the DMAchaining status. 0 DMAchain loading is not active                                      |
| 20 (R/NW)          | DMAS0      | 1 DMAchain loading is active DMATransfer Status. The EMDMA_CTL.DMAS0 bit provides the DMAchannel 0 transfer status. 0 DMAidle                    |
| 17:16 (R/NW)       | DFS        | 1 DMAin progress DMAFIFO Status. The EMDMA_CTL.DFS bit field provides the DMAFIFO status.                                                        |

Table 35-19: EMDMA\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | INTDONE0   | Internal DMACompletion Interrupt (Control). The EMDMA_CTL.INTDONE0 bit configures when the DMAcomplete interrupt is generated. The EMDMA_CTL.INTDONE0 =1 setting is provided for backward compatibility with older SHARC processors.                                                                                                                                                                                                                                                              | Internal DMACompletion Interrupt (Control). The EMDMA_CTL.INTDONE0 bit configures when the DMAcomplete interrupt is generated. The EMDMA_CTL.INTDONE0 =1 setting is provided for backward compatibility with older SHARC processors.                                                                                                                                                                                                                                                              |
| 12 (R/W)           | INTDONE0   | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Interrupt on access completion (Channel 0 or Channel 1 DMAcompletion)                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 12 (R/W)           | INTDONE0   | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Interrupt on Channel 0 DMAcompletion.                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 9 (R/W)            | TLEN       | Tap List DMAEnable. The EMDMA_CTL.TLEN bit enables scatter/gather tap list DMA.                                                                                                                                                                                                                                                                                                                                                                                                                   | Tap List DMAEnable. The EMDMA_CTL.TLEN bit enables scatter/gather tap list DMA.                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 9 (R/W)            | TLEN       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Disables the tap list based scatter/gatherDMA                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 9 (R/W)            | TLEN       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Enables the tap list based scatter/gatherDMA                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 8 (R/W)            | OFCEN      | On the Fly Control Loading Enable. The control bits in EMDMA_CHNPTR register are used to describe the next TCB behavior if the EMDMA_CTL.OFCEN bit is set. Therefore, the DMAcontrols can be changed from TCB to TCB. 0 = disables the control bits in the EMDMA_CHNPTR register. 1 = enables the control bits in the EMDMA_CHNPTR register. If chaining is enabled with EMDMA_CTL.OFCEN bit set, the EMDMA_CTL.TRAN bit has no effect, and direction is determined by the EMDMA_CHNPTR.CPDR bit. | On the Fly Control Loading Enable. The control bits in EMDMA_CHNPTR register are used to describe the next TCB behavior if the EMDMA_CTL.OFCEN bit is set. Therefore, the DMAcontrols can be changed from TCB to TCB. 0 = disables the control bits in the EMDMA_CHNPTR register. 1 = enables the control bits in the EMDMA_CHNPTR register. If chaining is enabled with EMDMA_CTL.OFCEN bit set, the EMDMA_CTL.TRAN bit has no effect, and direction is determined by the EMDMA_CHNPTR.CPDR bit. |
| 7 (R/W)            | WRBEN      | Write Back Enable. The EMDMA_CTL.WRBEN bit enables write back of the EIEP register after reads and or writes. Write back is automatically enabled for delay line DMA. WRBEN is applicable only if chaining is enabled ( EMDMA_CTL.CHEN =1).                                                                                                                                                                                                                                                       | Write Back Enable. The EMDMA_CTL.WRBEN bit enables write back of the EIEP register after reads and or writes. Write back is automatically enabled for delay line DMA. WRBEN is applicable only if chaining is enabled ( EMDMA_CTL.CHEN =1).                                                                                                                                                                                                                                                       |
| 5 (R/W)            | DFLSH      | Flush DMAFIFO. The EMDMA_CTL.DFLSH bit flushes the DMAFIFO. The buffer is only flushed when this bit is set. It can be set with the enable bit. Setting this bit also clears the EMDMA_CTL.DFS bit.                                                                                                                                                                                                                                                                                               | Flush DMAFIFO. The EMDMA_CTL.DFLSH bit flushes the DMAFIFO. The buffer is only flushed when this bit is set. It can be set with the enable bit. Setting this bit also clears the EMDMA_CTL.DFS bit.                                                                                                                                                                                                                                                                                               |
| 4 (R/W)            | CBEN       | Circular Buffering Enable. The EMDMA_CTL.CBEN bit enables circular buffering. Circular buffering can be used with normal DMAas well. If circular buffering is enabled with chaining for normal DMA, then ELEP and EBEP should be part of the TCB.                                                                                                                                                                                                                                                 | Circular Buffering Enable. The EMDMA_CTL.CBEN bit enables circular buffering. Circular buffering can be used with normal DMAas well. If circular buffering is enabled with chaining for normal DMA, then ELEP and EBEP should be part of the TCB.                                                                                                                                                                                                                                                 |
| 4 (R/W)            | CBEN       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Disables circular buffering with delay lineDMA                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 4 (R/W)            | CBEN       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Enables circular buffering with delay lineDMA                                                                                                                                                                                                                                                                                                                                                                                                                                                     |

Table 35-19: EMDMA\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | DLEN       | Enable Delay Line DMA. The EMDMA_CTL.DLEN bit enables delay line DMA. This bit is applicable only when the EMDMA_CTL.CHEN bit =1.                                                                                                                                                                             |
| 2 (R/W)            | CHEN       | Enable Chaining. The EMDMA_CTL.CHEN bit enables DMAchaining. 0 Chaining disabled                                                                                                                                                                                                                              |
| 1 (R/W)            | TRAN       | DMADirection. The EMDMA_CTL.TRAN bit determines the DMAdata direction. Note: If the delay line DMAis enabled, this bit does not have any effect. For delay line DMA, transfer direction depends on the state of delay line transfers. For internal-inter- nal or external-external DMA, this bit must be set. |
| 0 (R/W)            | EN         | DMAEnable. The EMDMA_CTL.EN bit enables DMA. 0 DisableDMA                                                                                                                                                                                                                                                     |

## Internal Index Register

The EMDMA\_INDX0 register contains the start address of the buffer for channel 0 DMA.

Note: For delay line DMA the EMDMA\_INDX0 register serves as the delay line write index which is the start address of the channel 0 DMA buffer for the channel 1 write data.

Figure 35-16: EMDMA\_INDX0 Register Diagram

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000015_7321c00d3ce067986ee9343dda498f84e9288bd15eb09d2598e396d4c19773ee.png)

Table 35-20: EMDMA\_INDX0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------|
| 29:0 (R/W)         | VALUE      | DMAbuffer Start Address. The EMDMA_INDX0.VALUE bit field is written with the 30 MSBs of the word -aligned byte addresses. |

## External Index Register

The EMDMA\_INDX1 register contains the start address of the buffer for channel 1 DMA.

Figure 35-17: EMDMA\_INDX1 Register Diagram

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000016_99ffde9a89bb0872ac099cc61753d466021f2d15619a3157451ec30650dffffe.png)

Table 35-21: EMDMA\_INDX1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                           |
|--------------------|------------|-------------------------------------------------------------------|
| 29:0               | VALUE      | DMAExternal Address Index.                                        |
| (R/W)              |            | The EMDMA_INDX1.VALUE bit field is the DMAexternal address index. |

## Internal Modifier Register

The EMDMA\_MOD0 register contains the channel 0 DMA address modifier.

Figure 35-18: EMDMA\_MOD0 Register Diagram

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000017_dacb8b684e1571a3137fc906bf162c7a85ca7a319d88a38b96d7445967501985.png)

Table 35-22: EMDMA\_MOD0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                    |
|--------------------|------------|------------------------------------------------------------|
| 15:0               | VALUE      | DMAAddress Modifier.                                       |
| (R/W)              |            | The EMDMA_MOD0.VALUE bit field is the DMAaddress modifier. |

## External Modifier Register

The EMDMA\_MOD1 register contains the external (channel 1 DMA) address modifier.

Figure 35-19: EMDMA\_MOD1 Register Diagram

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000018_4076b963f4a9bf27a15c4676a79f040b8273de3731a5faedd86a8ae139108e82.png)

Table 35-23: EMDMA\_MOD1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                             |
|--------------------|------------|---------------------------------------------------------------------|
| 26:0               | VALUE      | DMAExternal Address Modifier.                                       |
| (R/W)              |            | The EMDMA_MOD1.VALUE bit field is the DMAexternal address modifier. |

## Delay Line Tap Count Register

The EMDMA\_TCNT register is the tap count register for delay line DMA. This register holds the length of the tap list (the number of taps). The total number of words read from the delay line is equal to the EMDMA\_TCNT (tap count) multiplied by the EMDMA\_CNT1 (read block size).

Figure 35-20: EMDMA\_TCNT Register Diagram

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000019_e1c5d2c0aa60b6634dbbe71f9b8884ba5db0341a335a23cfb4d1a39af56c5cba.png)

Table 35-24: EMDMA\_TCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                      |
|--------------------|------------|------------------------------------------------------------------------------|
| 31:0               | VALUE      | Delay Line Tap Count.                                                        |
| (R/W)              |            | The EMDMA_TCNT.VALUE bit field is the tap count register for delay line DMA. |

## Tap List Pointer Register

The EMDMA\_TPTR register holds the address of an array in memory which holds offsets to be used when accessing a delay line in system memory. The offset represents the first address of each read block.

Note: the lower 30-bits of this register are to be written with 30 MSBs of the word-aligned byte address of the array.

Figure 35-21: EMDMA\_TPTR Register Diagram

![Image](38_Extended_Memory_DMA_(EMDMA)_artifacts/image_000020_926bc6685fde612352368e02f749d09b7c420b42e3968fd0522c7d141371ca49.png)

Table 35-25: EMDMA\_TPTR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| 29:0 (R/W)         | VALUE      | Delay Line Tap List Pointer. The EMDMA_TPTR.VALUE bit field contains the offsets to be used when accessing a delay line in system memory. |