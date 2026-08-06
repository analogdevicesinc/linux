# Cyclic Redundancy Check (CRC)

<!-- source: 044_Cyclic_Redundancy_Check_CRC.pdf | original pages 2540–2580 -->

## 37   Cyclic Redundancy Check (CRC)

The CRC peripheral performs the cyclic redundancy check (CRC) of the block of data that is presented to the peripheral. The peripheral provides a means to verify periodically the integrity of the system memory, the contents of memory-mapped registers (MMRs), or communication message objects. It is based on a CRC32 engine that computes the signature of 32-bit data presented to the peripheral.

The dedicated hardware compares the calculated signature of the operation to a pre-loaded expected signature. If the two signatures fail to match, the peripheral generates an error.

The source channel of the memory-to-memory DMA channels can provide data. The CRC optionally forwards data to memory through the destination DMA channel. Alternatively, the peripheral supports data presented by any qualified controller of the CRC peripheral bus.

The CRC peripheral implements a reduced table-look-up algorithm to compute the signature of the data. The CRC uses a programmable 32-bit CRC polynomial to generate the look-up table (LUT) contents automatically.

More CRC peripheral modes allow for initializing large memory sections with a constant value, or for verifying that sections of memory are equal to a constant value.

NOTE: CRC is supported by MDMA0, MDMA1, MDMA4, and MDMA5 channels only.

## CRC Features

The CRC peripheral supports a number of key features.

- Memory scan modes for memory verification
- Memory transfer modes for on-the-fly CRC calculations while transferring data from one memory to another
- A programmable 32-bit CRC polynomial with automatic LUT generation
- Data mirroring options

The CRC module also includes the following features.

- CRC checksum computation and comparison modes
- 32-bit programmable CRC polynomial with bit reverse option
- Automatic look-up table (LUT) generation

- Data mirroring options for endian and reflected polynomial cases
- Automatic clear and preset of results
- Fault and error interrupt reporting
- DMA and MMR based operation

Because the CRC module is closely tied to memory-to-memory DMA (MDMA) channel pairs, the use cases include the following features.

- Memory scan mode with CRC compute or compare
- Memory transfer mode with CRC compute or compare
- Memory fill operation with 32-bit data patterns
- Memory verify operation
- MMR write access to FIFO of destination DMA
- MMR read access to FIFO of source DMA
- Profiting from advanced DMA features, like descriptor mode and bandwidth control or monitor

## CRC Functional Description

The CRC peripheral supports a number of modes of operation that allow for the initialization and verification of regions of memory. The peripheral supports efficient memory-fill and verification operations on regions of memory with or against a constant value. These modes of operation do not require the CRC engine to calculate a signature. Other modes of operation allow for the calculation of CRC signature and verification for a memory region. The modes allow for on-the-fly CRC calculation when performing memory-to-memory DMA transfers from one memory region to another.

To minimize the need for core accesses, the peripheral interfaces with one or more (depending on processor features) memory-to memory DMA (MDMA) channels. This connectivity permits flexible configuration, in which data can be written-to or read-from the peripheral using DMA transactions, core transactions, or a combination of both.

Figure 37-1: Memory Flow

<!-- image -->

## ADSP-2159x\_SC592\_SC594 CRC Register List

The Cyclic Redundancy Check (CRC) unit includes the data comparison, polynomial operation, and look up table generation features needed for CRC operation. The CRC provides CRC protection as specified by many functional safety requirements. This unit facilitates the system software's ability to periodically check the correctness of the

code/data available in memory. A set of registers govern CRC operations. For more information on CRC functionality, see the CRC register descriptions.

Table 37-1: ADSP-2159x\_SC592\_SC594 CRC Register List

| Name           | Description                     |
|----------------|---------------------------------|
| CRC_COMP       | Data Compare Register           |
| CRC_CTL        | Control Register                |
| CRC_DCNT       | Data Word Count Register        |
| CRC_DCNTCAP    | Data Count Capture Register     |
| CRC_DCNTRLD    | Data Word Count Reload Register |
| CRC_DFIFO      | Data FIFO Register              |
| CRC_FILLVAL    | Fill Value Register             |
| CRC_INEN       | Interrupt Enable Register       |
| CRC_INEN_CLR   | Interrupt Enable Clear Register |
| CRC_INEN_SET   | Interrupt Enable Set Register   |
| CRC_POLY       | Polynomial Register             |
| CRC_RESULT_CUR | CRC Current Result Register     |
| CRC_RESULT_FIN | CRC Final Result Register       |
| CRC_STAT       | Status Register                 |

## ADSP-2159x\_SC592\_SC594 CRC Interrupt List

Table 37-2: ADSP-2159x\_SC592\_SC594 CRC Interrupt List

|   Interrupt ID | Name         | Description                | Sensitivity   | DMA Channel   |
|----------------|--------------|----------------------------|---------------|---------------|
|            190 | CRC0_ERR     | CRC0 Error                 | Level         |               |
|            191 | CRC1_ERR     | CRC1 Error                 | Level         |               |
|            196 | CRC0_DCNTEXP | CRC0 Data Count Expiration | Level         |               |
|            197 | CRC1_DCNTEXP | CRC1 Data Count Expiration | Level         |               |
|            198 | CRC2_ERR     | CRC2 Error                 | Level         |               |
|            199 | CRC3_ERR     | CRC3 Error                 | Level         |               |
|            204 | CRC2_DCNTEXP | CRC2 Data Count Expiration | Level         |               |
|            205 | CRC3_DCNTEXP | CRC3 Data Count Expiration | Level         |               |

## CRC Definitions

To make the best use of the CRC, it is useful to understand the following terms.

## CRC

Acronym for Cyclic Redundancy Check. An error detection code that can detect changes within a block of data.

## CRC Polynomial

The 32-bit polynomial used by the CRC engine to generate the look-up table required for the CRC implementation

## LUT

Acronym for the Look-up Table. The look-up table is automatically generated from the supplied 32-bit CRC polynomial.

## DMA

Acronym for Direct Memory Access. Used to describe a data transfer that takes place through a DMA channel allowing data distribution around a system without intervention from the core.

## MDMA

Acronym for Memory-To-Memory DMA transfer that often requires the use of two DMA channels to transfer data from one memory region to another memory region. One DMA channel is configured as a source channel and the second as a destination channel.

## CRC Block Diagram

The CRC Block Diagram shows the functional block diagram of the CRC. The following sections describe the blocks.

Figure 37-2: CRC Block Diagram

<!-- image -->

## Peripheral DMA Bus

The CRC peripheral provides both an incoming and outgoing datapath to the peripheral DMA bus. The MDMA source channel is interfaced to the incoming datapath providing data to the CRC peripheral. For memory transfer and data fill modes, the CRC uses the MDMA destination channel to either output the data from the CRC FIFO or use the data for the fill operation.

## MMR Access Bus

The core uses the MMR access bus to access all the memory-mapped registers of the peripheral for configuration, status, and debug purposes. The core can also use the MMR access bus to feed data to the CRC peripheral or read data from the FIFO of the CRC peripheral. The CRC operation is an alternative to the DMA channel operation to read data from the FIFO.

Data received by MMR writes can transfer to destination DMA. Similarly, data received by source DMA can be output through the MMR interface. Optionally, intermediate results can be made available to the MMR interface.

## Mirror Block

The mirror block individually controls bit-reversing of the polynomial, the computation results, and the expected result. Bit mirroring, byte mirroring, word swapping, and any combination of these operations can control endian and the reflection of processed data.

## Data FIFO

The CRC data FIFO is a 32-bit-wide 4-entry FIFO. The FIFO is accessible to both the peripheral DMA bus and the MMR access bus. The FIFO status is accessible from the CRC\_STAT register.

## DMA Request Generator

The DMA request generator is responsible for granting incoming DMA requests from the source DMA channel and issuing outgoing DMA requests to the destination DMA channel.

## CRC Engine

The CRC engine is a 32-bit CRC engine that implements the reduced table look-up scheme. The CRC engine provides support for a user-programmable 32-bit polynomial that the CRC uses to load the look-up table parameters required for the CRC calculation. The CRC engine is a single cycle implementation operating on 32 bits of data per cycle.

## Compare Logic

The compare logic takes the final CRC signature and compares it to the expected CRC signature, generating a CRC compare error when the signatures do not match. A compare error can flag a system fault.

## CRC Architectural Concepts

A 32-bit polynomial is required before calculation of the CRC signature can occur. The CRC uses the polynomial to generate the contents of an internal look-up table that the reduced table look-up implementation requires. The look-up table is automatically generated when the polynomial is written. It must be initialized prior to any operation that requires the use of the CRC engine.

The mirror block logic can manipulate the data presented to the CRC engine before the CRC uses the data in the calculation of the CRC signature. The data mirror operation is configurable to allow for bit reversing, byte reversing, and 16-bit word swapping operations on the incoming data. For memory transfer compute-and-compare operations, programs can configure the peripheral to output the data in the same form in which it was received. Or, the operation can output the mirrored data in the same manner that it is presented to the CRC engine.

While the CRC peripheral is in operation, the status of the FIFO is continually updated and reflected in the CRC\_STAT register. The FIFO status is required for core-based accesses to the CRC peripheral. The status indicates when:

- The CRC peripheral can receive data

- Data is available for reading from the FIFO
- The result of the CRC\_RESULT\_CUR register has been updated

The status of the CRC\_RESULT\_CUR register indicates that the current CRC calculation has completed and the result is available.

## Look-up Table

The look-up table consists of a set of sixteen 32-bit registers that hardware populates automatically when a write access takes place to the CRC\_POLY register. 16 clock cycles are required to generate all 16 look-up table entries. The status of the process for generating the look-up table is reflected in CRC\_STAT.LUTDONE allowing for software to poll on the completion of the event or for generation of an interrupt.

NOTE: Hardware must populate the look-up table before any operation using the CRC peripheral can take place, even if the operation does not use the CRC engine. The peripheral does not issue any data requests until the table generation process is complete. In addition, the CRC\_STAT.IBR field, that indicates the input buffer status as required for core-based transfers, is only valid upon completion of the process for generating the look-up table.

## Data Mirroring

The data mirror block can be configured to manipulate the incoming data before the data passes to the CRC engine and, optionally, to the FIFO. This configuration allows the peripheral to handle various forms of endianness and to function with reflected polynomials.

There are three configuration bits that control the data mirroring process: CRC\_CTL.BITMIRR , CRC\_CTL.BYTMIRR , and CRC\_CTL.W16SWP . The Data Mirroring Options table details how these options affect the incoming data and the output generated by the mirror block.

Table 37-3: Data Mirroring Options

|   W16SWP |   BYTMIRR |   BITMIRR | Output Data                                            |
|----------|-----------|-----------|--------------------------------------------------------|
|        0 |         0 |         0 | Dout[31:0] = Din[31:0]                                 |
|        0 |         0 |         1 | Dout[31:0] = Din[24:31],Din[16:23],Din[8:15],Din[0:7]  |
|        0 |         1 |         0 | Dout[31:0] = Din[7:0],Din[15:8],Din[23:16],Din[31:24]  |
|        0 |         1 |         1 | Dout[31:0] = Din[0:7],Din[8:15],Din[16:23],Din[24:31]  |
|        1 |         0 |         0 | Dout[31:0] = Din[15:0], D[31:16]                       |
|        1 |         0 |         1 | Dout[31:0] = Din[8:15],Din[0:7], Din[24:31],Din[16:23] |
|        1 |         1 |         0 | Dout[31:0] = Din[23:16],Din[31:24], Din[7:0],Din[15:8] |
|        1 |         1 |         1 | Dout[31:0] = Din[16:23],Din[24:31], Din[0:7],Din[8:15] |

When the CRC is configured to operate in the memory transfer compute-and-compare mode, the bit-reversed output data can be written to the FIFO. This feature is controlled through the CRC\_CTL.FDSEL field.

In addition to providing bit swapping and mirror options to the incoming data, the CRC peripheral also supports bit mirroring on the following registers.

- CRC\_RESULT\_CUR and CRC\_RESULT\_FIN , controlled through the CRC\_CTL.RSLTMIRR field. When mirroring is enabled, the values to be written to these registers are fully bit-reversed before the write operation occurs.
- CRC\_POLY , controlled through the CRC\_CTL.POLYMIRR field. When mirroring is enabled, the 32-bit polynomial is fully bit-reversed before the write operation to the register occurs.
- CRC\_COMP , controlled through the CRC\_CTL.CMPMIRR field. When mirroring is enabled, the contents to be loaded to this register are fully bit-reversed before the write operation occurs.

## FIFO Status and Data Requests

The CRC peripheral provides indication of the input and output buffer status through CRC\_STAT.IBR and CRC\_STAT.OBR respectively. For core-based operations, software must monitor these status fields prior to writing to or reading from the CRC FIFO. No write to the CRC FIFO can occur while CRC\_STAT.IBR indicates that the buffer is not ready to accept data. Similarly, the CRC FIFO cannot be read until CRC\_STAT.OBR indicates that data is available.

The memory scan modes of operation only require the monitoring of the input buffer status. The memory transfer, compute-and-compare mode uses both input and output buffer status. If the current result of the CRC computation is required, then software must verify that the current operation has completed and that the intermediate result is ready. The CRC\_STAT.IRR indicates the status.

NOTE: The memory transfer fill mode of operation requires the use of a DMA channel. The CRC does not support core reads from the CRC FIFO for this mode of operation.

Memory transfer, compute-and-compare mode uses burst transactions to make the most efficient use of the available resources. In this mode, when the FIFO is initially empty and the peripheral is enabled, the CRC\_STAT.IBR bit indicates that the CRC is ready to accept data. The peripheral generates data requests to the source DMA channel (if the CRC uses DMA). While the number of words remaining in the CRC\_DCNT register is greater than the FIFO depth, the peripheral issues data requests or accepts incoming data in bursts. The peripheral continues until the CRC FIFO becomes full.

Once full, the CRC\_STAT.IBR and CRC\_STAT.OBR bits are updated, and then the CRC issues outgoing data requests. Only when the FIFO is empty can the peripheral accept further incoming data, and the CRC\_STAT.IBR and CRC\_STAT.OBR bits are updated once again.

Once CRC\_DCNT is decremented such that the number of words waiting for processing is less than the number required to fill the FIFO, the burst mode of operation is disabled. Incoming data is accepted when the FIFO is not full. Outgoing data is available when the FIFO is not empty. Therefore, there are no restrictions requiring the word count to be a multiple of the FIFO depth.

All other CRC modes of operation indicate that incoming data can be accepted when the FIFO is not full. Outgoing data is available when the FIFO is not empty.

The CRC\_CTL.OBRSTALL and CRC\_CTL.IRRSTALL bit configurations also influence how the CRC generates data requests and status bits. The following list describes the bits.

- The CRC\_CTL.OBRSTALL bit can be configured such that the CRC peripheral stalls as soon as there is output data available in the FIFO. Use this mode of operation only in memory transfer, compute-and-compare mode. This mode results in the processing of one 32-bit word at a time. The peripheral does not request or accept incoming data until the current value being processed is read from the peripheral.
- The CRC\_CTL.IRRSTALL bit can be configured so that the CRC peripheral stalls all further incoming data requests until the CRC\_RESULT\_CUR register is read after being updated. Use this mode of operation for CRC signature generation. It is not applicable to memory transfer data-fill mode or memory scan data-verify mode of operation.

## CRC Operating Modes

The following sections describe the various operating modes of the CRC interface.

## Data Transfer Modes

The CRC peripheral supports two main categories of operation involving data transfers:

- Memory scan mode
- Memory transfer mode

Memory scan modes are read-only operations that allow the contents of memory to be read into the peripheral and verified for correctness. There are two forms of memory scan mode:

- CRC compute-and-compare performs a CRC calculation on data presented to the peripheral and compares the CRC result with a pre-determined and pre-loaded result. An error is generated when the results differ.
- Data verify compares each 32-bit data word presented to the CRC peripheral to a pre-loaded 32-bit value and generates an error when the data differs.

Both modes of operation require at the most, a single DMA channel to read the data from memory into the peripheral. No data is forwarded to the data output or destination DMA. The CRC can also use core-driven transfers for either of these modes of operation.

The memory transfer modes involve memory write or memory read-and-write operations allowing for memory to be initialized or transferred from one region of memory to another. There are two forms of memory transfer mode:

- CRC compute-and-compare performs a full data transfer from one memory region to another memory region. The CRC generates a signature on the data presented and compares it with a pre-determined and pre-loaded result. An error is generated when the results differ.
- Data fill initializes a region of memory with a pre-loaded 32-bit constant value.

The CRC compute-and-compare mode of operation requires both incoming and outgoing data channels. The operation occurs either using DMA channels, using core-driven write or read operations to and from the FIFO or using

a combination of both. The data fill mode of operation requires only a memory write DMA destination channelthis mode does not support core driven operations.

## Memory Scan Compute-and-Compare Mode

In this mode of operation, the CRC engine of the peripheral is enabled. The mode is configured through the CRC\_CTL.OPMODE field and the CRC engine performs a 32-bit CRC operation on the incoming data stream.

The length of the data stream is configured through the CRC\_DCNT register. The accumulated result of the CRC operation is contained in the CRC\_RESULT\_CUR register. As the CRC engine processes each 32-bit word, the CRC\_DCNT register is decremented and CRC\_RESULT\_CUR is updated.

Once CRC\_DCNT decrements to zero, the contents of the CRC\_RESULT\_CUR register are copied to CRC\_RESULT\_FIN and CRC\_STAT.DCNTEXP is updated accordingly. The CRC uses the CRC\_COMP register to store the expected result of the operation. After the CRC calculation, CRC\_COMP is compared with CRC\_RESULT\_FIN and CRC\_STAT.CMPERR is updated to reflect the status of the compare operation. CRC\_STAT.CMPERR must be cleared before the next CRC operation is performed.

The CRC peripheral also contains the CRC\_DCNTRLD register. The CRC uses this register to reload CRC\_DCNT upon completion of the CRC operation in preparation for the next transfer.

The initial seed of the CRC computation can be configured through CRC\_CTL.AUTOCLRZ and CRC\_CTL.AUTOCLRF . This configuration provides a way to reset CRC\_RESULT\_CUR to 0x00000000, 0xFFFFFFFF or to leave the current register contents untouched for the next operation.

The peripheral can be configured to allow for the compare error and data expiration events to generate an interrupt.

## Memory Scan Data Verify

In this mode of operation, the CRC engine of the peripheral is not required. The mode is enabled through the CRC\_CTL.OPMODE field. Each 32-bit word of the data stream is compared with a constant value that is stored in the CRC\_COMP register. The CRC\_DCNT register contains the number of words for comparison. The CRC\_DCNT register is decremented upon receiving a new 32-bit word from the data stream. If the compare operation fails, the CRC\_STAT.CMPERR bit is updated and the contents of CRC\_DCNT are captured in the CRC\_DCNTCAP register. This information can be used to identify the location in the data stream where the error occurred. Clear the CRC\_STAT.CMPERR field to reenable capturing of further errors.

Once CRC\_DCNT decrements to zero, CRC\_STAT.DCNTEXP is updated accordingly to signal the end of the operation. The peripheral can be configured to allow for the compare error and data expiration events to generate an interrupt.

## Memory Transfer Compute-and-Compare Mode

In this mode of operation, the CRC engine of the peripheral is enabled. The mode is configured through the CRC\_CTL.OPMODE field and the CRC engine performs a 32-bit CRC operation on the incoming data stream.

The length of the data stream is configured through the CRC\_DCNT register. The accumulated result of the CRC operation is contained in the CRC\_RESULT\_CUR register. As the CRC engine processes each 32-bit word, the CRC\_DCNT register is decremented and CRC\_RESULT\_CUR is updated.

Once CRC\_DCNT decrements to zero, the contents of the CRC\_RESULT\_CUR register are copied to CRC\_RESULT\_FIN and CRC\_STAT.DCNTEXP is updated accordingly. The CRC uses the CRC\_COMP register to store the expected result of the operation. Upon completion of the CRC calculation, CRC\_COMP is compared with CRC\_RESULT\_FIN and CRC\_STAT.CMPERR is updated to reflect the status of the compare operation. Clear CRC\_STAT.CMPERR before the next CRC operation is performed.

The CRC peripheral also contains CRC\_DCNTRLD register. The CRC uses this register to reload CRC\_DCNT upon completion of the CRC operation in preparation for the next transfer.

The initial seed of the CRC computation can be configured through CRC\_CTL.AUTOCLRZ and CRC\_CTL.AUTOCLRF . This configuration provides a means to reset CRC\_RESULT\_CUR to 0x00000000, 0xFFFFFFFF or to leave the current register contents untouched for the next operation.

The peripheral can be configured to allow for the compare error and data expiration events to generate an interrupt.

## Memory Transfer Data Fill Mode

In this mode of operation, the CRC engine of the peripheral is not required. The mode is enabled through the CRC\_CTL.OPMODE field. The CRC\_FILLVAL register is written with a 32-bit value. The CRC uses this value to initialize a block memory through the memory-to-memory DMA destination channel. When the CRC peripheral and the DMA destination channel are enabled, the contents of the CRC\_FILLVAL register is written to the DMA channel to initialize the memory region. The CRC\_DCNT register contains the number of words for the write operation.

Once CRC\_DCNT decrements to zero, CRC\_STAT.DCNTEXP is updated accordingly to signal the end of the operation. The peripheral can be configured to allow for the data expiration event to generate an interrupt.

## CRC Event Control

The CRC peripheral can enable certain CRC status operations to generate an interrupt event to the system event controller. There, a CRC error can be qualified as a system fault.

## Interrupt Signals

The CRC peripheral can generate two interrupt requests that are optionally enabled as interrupts with the Arm core, or as events to SEC1 for fault operations. One is a CRC status interrupt and the other is a CRC error interrupt.

The CRC\_STAT.CMPERR status bit can be configured as an interrupt and is signaled through the CRC error interrupt signal. The CRC\_STAT.CMPERR status field is set whenever the CRC peripheral performs a compare operation that fails. This status can be the result of a failed memory scan data-verify operation that compares the contents of a memory range with a constant 32-bit value. Or, it can be the result of a CRC signature calculated for a memory region that does not match the expected pre-programmed result for a memory-compare operation.

The CRC\_STAT.DCNTEXP status bit is set when the CRC\_DCNT register has decremented to zero. The status indicates that the CRC peripheral has now processed all the data requested for the current CRC operation. The CRC can also use this signal to generate an interrupt. The interrupt is signaled on the CRC status interrupt signal.

Both these status bits can be configured to generate and interrupt through the CRC\_INEN register. The CRC\_INEN register also has bit set, CRC\_INEN\_SET , and bit clear CRC\_INEN\_CLR equivalent registers that the CRC uses for the enabling and disabling of these interrupt sources.

The CRC\_STAT register has two write one to clear (W1C) fields for clearing the two interrupt sources.

- NOTE: Disabling the CRC peripheral through the CRC\_CTL.BLKEN bit does not result in the clearing of interrupt sources. Clear the interrupt sources using a W1C operation to the CRC\_STAT register.

## CRC Programming Model

It is important to note the following restrictions when using the CRC peripheral with the DMA channels:

1. When enabling the CRC peripheral and the DMA channels, enable the CRC peripheral prior to enabling the DMA channels.
2. When disabling the CRC peripheral and the DMA channels, disable the DMA channels prior to disabling the CRC peripheral.

## CRC Mode Configuration

Describes a number of tasks showing the various operation modes of the CRC peripheral.

- Look-up Table Generation
- Core Driven Memory Scan Compute-and-Compare Mode
- DMA Driven Memory Scan Compute-and-Compare Mode
- Core Driven Memory Scan Data Verify Mode
- DMA Driven Memory Scan Data Verify Mode
- Core Driven Memory Transfer Compute-and-Compare Mode
- DMA Driven Memory Transfer Compute-and-Compare Mode
- DMA Driven Memory Transfer Data Fill Mode

## Look-up Table Generation

Describes the steps required to initialize the CRC peripheral LUT.

1. Write the 32-bit CRC polynomial of choice to the CRC\_POLY register.

ADDITIONAL INFORMATION: This operation results in the CRC peripheral starting the LUT initialization process. The CRC\_STAT.LUTDONE bit is updated to reflect the operation is in progress.

2. Poll the CRC\_STAT.LUTDONE bit until the status bit indicates that the operation is completed.

The CRC peripheral has completed initialization of all the LUT registers and is now ready for data operations. The CRC\_STAT.LUTDONE bit remains in the current state until the CRC\_POLY register is written again, or the peripheral or processor are reset.

## Core Driven Memory Scan Compute-and-Compare Mode

Performs CRC signature calculation and verification for a region of memory using core transactions. The CRC peripheral is configured such that it operates in burst mode due to the stalling options configured through disabling the CRC\_CTL register.

The task assumes the following:

- The polynomial has been loaded and the look-up table is fully initialized
- All CRC interrupts have been serviced (none pending)
- The CRC block is disabled per CRC\_CTL.BLKEN
1. Initialize the CRC\_DCNT register. ADDITIONAL INFORMATION: The value loaded must represent the number of 32-bit words in the memory

region for which the software calculates and verifies the signature.

2. Initialize the CRC\_DCNTRLD register. ADDITIONAL INFORMATION: This value is used to reload the CRC\_DCNT register upon completion of current CRC operation. If no further operation is needed, then this register can be initialized to zero.
3. Initialize the CRC\_RESULT\_CUR register. ADDITIONAL INFORMATION: This register can be initialized to provide an initial seed for the CRC operation that is about to take place.
4. Initialize the CRC\_COMP register. ADDITIONAL INFORMATION: This register contains the pre-calculated final CRC signature result for the memory region that the software uses in the final compare operation.
5. Initialize the CRC\_INEN register.
5. ADDITIONAL INFORMATION: The CRC uses this register to enable the generation of the CRC interrupts for notification of compare errors and block completion. Configure these interrupts. If enabled, ensure that the corresponding interrupt handlers are also configured.
6. Initialize CRC\_CTL register with the CRC\_CTL.OPMODE bit set to memory scan compute-and-compare mode and the CRC\_CTL.BLKEN bit configured to enable the CRC peripheral.
- Disable the CRC\_CTL.OBRSTALL and CRC\_CTL.IRRSTALL bit options for this task example.
- Configure all mirroring and bit reversal options.

- Configure CRC auto-clear options.

The CRC peripheral is now enabled and ready for the core or DMA channel to write data.

7. Write memory region data to the CRC peripheral.
- a. While CRC\_STAT.IBR bit indicates that the input buffer is ready, write the CRC\_DFIFO register with 32-bit data.

ADDITIONAL INFORMATION: Repeat this step until all data has been written.

8. Poll the CRC\_STAT.DCNTEXP bit if the interrupt was disabled.
2. ADDITIONAL INFORMATION: Perform this step only if counter expired interrupt is disabled. Polling en-

sures that all the data has been processed.

9. Poll the CRC\_STAT.CMPERR bit if the interrupt was disabled to check for a compare error. ADDITIONAL INFORMATION: Perform this step only if the compare error interrupt is not enabled.
10. Write to the CRC\_STAT register to clear both the CRC\_STAT.DCNTEXP and CRC\_STAT.CMPERR bits. ADDITIONAL INFORMATION: If interrupts were enabled, then clear of these status bits within the interrupt handlers for the respective interrupts. The CRC compute-and-compare operation is now complete. The CRC peripheral is ready to be configured for

the next CRC operation.

The integrity check of the memory through the expected CRC signature has completed. The final result is indicated through the CRC\_STAT.CMPERR bit and the corresponding interrupt when enabled.

Clear any W1C CRC status bits before performing more CRC operations.

## DMA Driven Memory Scan Compute-and-Compare Mode

Performs CRC signature calculation and verification for a region of memory using DMA transactions. The CRC peripheral is configured such that it operates in the burst mode of operation due to the stalling options configured through disabling CRC\_CTL .

The task assumes the following:

- The polynomial has been loaded and the look-up table is fully initialized
- All CRC interrupts have been serviced (none pending)
- The CRC block is disabled per the CRC\_CTL.BLKEN bit.
1. Initialize the CRC\_DCNT register.
- ADDITIONAL INFORMATION: The value loaded must represent the number of 32-bit words in the memory

region for which the software calculates and verifies the signature.

2. Initialize the CRC\_DCNTRLD register. ADDITIONAL INFORMATION: This value is used to reload the CRC\_DCNT register upon completion of current operation. If no further operation is needed, then this register can be initialized to zero.
3. Initialize the CRC\_RESULT\_CUR register.
3. ADDITIONAL INFORMATION: This register can be initialized to provide an initial seed for the CRC operation that is about to take place.
4. Initialize the CRC\_COMP register.
5. ADDITIONAL INFORMATION: This register contains the pre-calculated final CRC signature result for the memory region that the software uses in the final operation.
5. Initialize the CRC\_INEN register.
7. ADDITIONAL INFORMATION: The CRC module uses this register to enable the generation of the CRC interrupts for notification of compare errors and block completion. Configure these interrupts, as needed. If enabled, ensure that the corresponding interrupt handlers are also configured.
6. Initialize the CRC\_CTL register with the CRC\_CTL.OPMODE bit set to memory scan compute compare mode and the CRC\_CTL.BLKEN bit configured to enable the CRC peripheral.
- Disable the CRC\_CTL.OBRSTALL and CRC\_CTL.IRRSTALL bit options for this task example.
- Configure all mirroring and bit reversal options.
- Configure all CRC auto clear options.

The CRC peripheral is now enabled and ready for the core or DMA channel to write data.

7. Configure and enable the memory-to-memory source DMA channel for memory read STOP mode. ADDITIONAL INFORMATION: This step starts the data transfer from the memory region and writes the data to the CRC peripheral.
8. Poll the CRC\_STAT.DCNTEXP bit if the interrupt was disabled.
3. ADDITIONAL INFORMATION: Perform this step only if the counter expired interrupt is disabled. Polling
4. ensures all the data has been processed.
9. Poll the CRC\_STAT.CMPERR bit if the interrupt was disabled to check for a compare error. ADDITIONAL INFORMATION: Perform this step only if the compare error interrupt is not enabled.
10. Write the CRC\_STAT register to clear both the CRC\_STAT.DCNTEXP and CRC\_STAT.CMPERR bits. ADDITIONAL INFORMATION: If interrupts were enabled, then clear these status bits within the interrupt handlers for the respective interrupts.
7. The CRC compute-and-compare operation is now complete. The CRC peripheral is ready to be configured for
8. the next CRC operation.

The integrity check of the memory through the expected CRC signature has completed and the final result indicated is through CRC\_STAT.CMPERR and the corresponding interrupt, when enabled.

Clear any W1C CRC status bits before performing a further CRC operation. Clear any W1C status bits of the memory-to-memory source DMA channel before the next CRC operation.

## Core Driven Memory Scan Data Verify Mode

Reads a region of memory using core transactions and performs a compare operation on each 32-bit word against a single pre-loaded 32-bit constant. The compare error interrupt is enabled to capture and log the location of any compare errors.

The task assumes the following:

- The polynomial has been loaded and the look-up table is fully initialized
- All CRC interrupts have been serviced (none pending)
- The CRC block is disabled per CRC\_CTL.BLKEN

The interrupt service routine for the compare error interrupt reads and stores the contents of CRC\_DCNTCAP register to a buffer before clearing the compare error interrupt.

1. Initialize the CRC\_DCNT register. ADDITIONAL INFORMATION: The value loaded must represent the number of 32-bit words in the memory region for which the software calculates and verifies the signature.
2. Initialize the CRC\_DCNTRLD register.
3. ADDITIONAL INFORMATION: This value is used to reload the CRC\_DCNT register upon completion of

current CRC operation. If no further operation is needed, then this register can be initialized to zero.

3. Initialize the CRC\_COMP register.
2. ADDITIONAL INFORMATION: This register contains the 32-bit constant that the memory region is expec-

ted to be filled with. Each 32 bit of data presented to the peripheral is compared with this value.

4. Initialize the CRC\_INEN register. ADDITIONAL INFORMATION: The CRC module uses this register to enable the generation of the CRC interrupts for notification of compare errors and block completion. Configure these interrupts. If enabled, ensure that the corresponding interrupt handlers are also configured.
5. Initialize the CRC\_CTL register with the CRC\_CTL.OPMODE bit set to memory scan data verify mode and the CRC\_CTL.BLKEN bit configured to enable the CRC peripheral.

The CRC peripheral is now enabled and ready for the core or DMA channel to write data.

6. Write memory region data to the CRC peripheral.

- a. Poll the CRC\_STAT.IBR bit until input buffer is ready.
- b. Write the CRC\_DFIFO register with 32-bit data. ADDITIONAL INFORMATION: Repeat these two steps until the entire memory region has been written to the CRC peripheral.
7. Poll the CRC\_INEN\_SET.DCNTEXP bit if the interrupt was disabled.
4. ADDITIONAL INFORMATION: Perform this step only if counter expired interrupt is disabled. Polling ensures all the data has been processed.
8. Check if the buffer used to capture the CRC\_DCNTCAP register upon a compare error has any new entries. ADDITIONAL INFORMATION: The values captures in the buffer provide a means to locate where in the memory region the failures occurred.
9. Write to the CRC\_STAT to clear both the CRC\_INEN\_SET.DCNTEXP and CRC\_INEN.CMPERR bits. ADDITIONAL INFORMATION: If interrupts were enabled, the clear these status bits within the interrupt handlers for the respective interrupts.

The CRC memory scan-verify operation is now complete. The CRC peripheral is ready to be configured for the next CRC operation.

The result of the integrity check of the memory with the 32-bit constant is indicated through the CRC\_INEN.CMPERR bit and the corresponding interrupt, when enabled. Each comparison error is traceable due to the logging of CRC\_DCNTCAP from within the compare error interrupt handler.

Clear any W1C CRC status bits before performing a further CRC operation.

## DMA Driven Memory Scan Data Verify Mode

The memory scan data verify mode reads a region of memory using DMA transactions and performs a compare operation on each 32-bit word against a single pre-loaded 32-bit constant. The compare error interrupt is enabled to capture and log the location of any compare errors.

The task assumes the following:

- The polynomial has been loaded and the look-up table is fully initialized
- All CRC interrupts have been serviced (none pending)
- The CRC block is disabled per the CRC\_CTL.BLKEN bit

The interrupt service routine for the compare error interrupt reads and stores the contents of the CRC\_DCNTCAP register to a buffer before clearing the compare error interrupt.

1. Initialize the CRC\_DCNT register.

ADDITIONAL INFORMATION: The value loaded must represent the number of 32-bit words in the memory region for which the software calculates and verifies the signature.

2. Initialize the CRC\_DCNTRLD register. ADDITIONAL INFORMATION: The CRC module uses this register to reload the CRC\_DCNT register upon completion of current CRC operation. If no further operation is needed, then this register can be initialized to zero.
3. Initialize the CRC\_COMP register.
3. ADDITIONAL INFORMATION: This register contains the 32-bit constant that the memory region is expected to be filled with. Each 32 bit of data presented to the peripheral is compared with this value.
4. Initialize the CRC\_INEN register.
5. ADDITIONAL INFORMATION: The CRC module uses this register to enable the generation of the CRC interrupts for notification of compare errors and block completion. Configure these interrupts, as needed. If enabled, ensure that the corresponding interrupt handlers are also configured.
5. Initialize the CRC\_CTL register with the CRC\_CTL.OPMODE bit set to memory scan data verify mode and CRC\_CTL.BLKEN configured to enable the CRC peripheral. The CRC peripheral is now enabled and ready for the core or DMA channel to write the data.
6. Configure and enable the memory-to-memory source DMA channel for memory read STOP mode. ADDITIONAL INFORMATION: This step starts the data transfer from the memory region and writes the data to the CRC peripheral.
7. Poll the CRC\_STAT.DCNTEXP bit if the interrupt was disabled. ADDITIONAL INFORMATION: Perform this step only if counter expired interrupt is disabled. Polling ensures all the data has been processed.
8. Check if the buffer used to capture the CRC\_DCNTCAP register upon a compare error has any new entries. ADDITIONAL INFORMATION: The values captures in the buffer provide a means to locate where in the memory region the failures occurred.
9. Write the CRC\_STAT register to clear both the CRC\_STAT.DCNTEXP and CRC\_STAT.CMPERR bits. ADDITIONAL INFORMATION: If interrupts were enabled, then clear these status bits within the interrupt handlers for the respective interrupts. The CRC memory scan-verify operation is now complete. The CRC peripheral is ready to be configured for

the next CRC operation.

The result of the integrity check of the memory with the 32-bit constant is indicated through the CRC\_STAT.CMPERR bit and the corresponding interrupt when enabled. Each comparison error is traceable due to the logging of the CRC\_DCNTCAP register from within the compare error interrupt handler.

Clear any W1C CRC status bits and DMA status bits before performing a further CRC operation.

## Core Driven Memory Transfer Compute-and-Compare Mode

The memory transfer compute-and-compare mode performs CRC signature calculation and verification for a region of memory using core transactions while copying the contents to another memory region. The CRC peripheral is configured such that it operates in the burst mode of operation due to the stalling options configured through disabling the CRC\_CTL register.

The task assumes the following:

- The polynomial has been loaded and the look-up table is fully initialized
- All CRC interrupts have been serviced (none pending)
- The CRC block is disabled per the CRC\_CTL.BLKEN bit
1. Initialize the CRC\_DCNT register. ADDITIONAL INFORMATION: The value loaded must represent the number of 32-bit words in the memory region for which the software calculates and verifies the signature.
2. Initialize the CRC\_DCNTRLD register.
- ADDITIONAL INFORMATION: This value is used to reload the CRC\_DCNT register upon completion of the current CRC operation. If no further operation is needed, then this register can be initialized to zero.
3. Initialize the CRC\_RESULT\_CUR register. ADDITIONAL INFORMATION: This register can be initialized to provide an initial seed for the CRC operation that is about to take place.
4. Initialize the CRC\_COMP register. ADDITIONAL INFORMATION: This register contains the pre-calculated final CRC signature result for the memory region that the software uses in the final compare operation.
5. Initialize the CRC\_INEN register. ADDITIONAL INFORMATION: The CRC module uses this register to enable the generation of the CRC interrupts for notification of compare errors and block completion. Configure these interrupts, as needed. If enabled, ensure that the corresponding interrupt handlers are also configured.
6. Initialize the CRC\_CTL register with the CRC\_CTL.OPMODE bit set to memory scan compute-and-compare mode and the CRC\_CTL.BLKEN bit configured to enable the CRC peripheral.
- a. Disable the CRC\_CTL.OBRSTALL bit and the CRC\_CTL.IRRSTALL bit options for this task example.
- b. Configure all mirroring and bit reversal options.

- c. Configure CRC auto clear options

The CRC peripheral is now enabled and ready for the core or DMA channel to write data.

7. Write memory region data to the CRC peripheral and read it back to the new destination.
- a. While the CRC\_STAT.IBR bit indicates that the input buffer is ready, write the CRC\_DFIFO register with 32-bit data.
- b. While the CRC\_STAT.OBR bit indicates that the output buffer is ready, read the CRC\_DFIFO register and store data to new destination. ADDITIONAL INFORMATION: Repeat these two steps until all required data has been processed

through the CRC peripheral and copied to the new destination.

8. Poll the CRC\_STAT.DCNTEXP bit if the interrupt was disabled.
2. ADDITIONAL INFORMATION: Perform this step only if the counter expired interrupt is disabled. Polling ensures all the data has been processed.
9. Poll the CRC\_STAT.CMPERR bit if the interrupt was disabled to check for a compare error. ADDITIONAL INFORMATION: Perform this step only if the compare error interrupt is not enabled.
10. Write the CRC\_STAT register to clear both CRC\_STAT.DCNTEXP and CRC\_STAT.CMPERR bits.
5. ADDITIONAL INFORMATION: If interrupts were enabled, then clear these status bits within the interrupt handlers for the respective interrupts.

The CRC compute-and-compare operation is now complete. The CRC peripheral is ready to be configured for the next CRC operation. The memory region has also been copied to its new destination.

The memory region has been copied to a new location and an integrity check of the memory through the expected CRC signature has also completed. The final result is indicated through the CRC\_STAT.CMPERR bit and the corresponding interrupt when enabled.

Clear any W1C CRC status bits before performing a further CRC operation.

## DMA Driven Memory Transfer Compute-and-Compare Mode

The memory transfer compute-and-compare mode performs CRC signature calculation and verification for a region of memory using DMA transactions. The memory region is also copied to another memory region using memoryto-memory DMA transfers. The CRC peripheral is configured such that it operates in burst mode due to the stalling options configured through disabling CRC\_CTL .

The task assumes the following:

- The polynomial has been loaded and the look-up table is fully initialized
- All CRC interrupts have been serviced (none pending)

- The CRC block is disabled per the CRC\_CTL.BLKEN register.
1. Initialize the CRC\_DCNT register.

ADDITIONAL INFORMATION: The value loaded must represent the number of 32-bit words in the memory region for which the software calculates and verifies the signature.

2. Initialize the CRC\_DCNTRLD register.
2. ADDITIONAL INFORMATION: This value is used to reload the CRC\_DCNT register upon completion of current CRC operation. If no further operation is needed, then this register can be initialized to zero.
3. Initialize the CRC\_RESULT\_CUR register.
4. ADDITIONAL INFORMATION: This register can be initialized to provide an initial seed for the CRC operation that is about to take place.
4. Initialize the CRC\_COMP register. ADDITIONAL INFORMATION: This register contains the pre-calculated final CRC signature result for the memory region that the software uses in the final compare operation.
5. Initialize the CRC\_INEN register. ADDITIONAL INFORMATION: The CRC module uses this register to enable the generation of the CRC interrupts for notification of compare errors and block completion. Configure these interrupts, as needed. If enabled, ensure that the corresponding interrupt handlers are also configured.
6. Initialize the CRC\_CTL register with the CRC\_CTL.OPMODE bit set to memory scan compute compare mode and CRC\_CTL.BLKEN configured to enable the CRC peripheral.
- a. Disable the CRC\_CTL.OBRSTALL and the CRC\_CTL.IRRSTALL bit options for this task example.
- b. Configure all mirroring and bit reversal options
- c. Configure CRC auto clear options

The CRC peripheral is now enabled and ready for the core or DMA channel to write data.

7. Configure and enable the memory-to-memory source DMA channel for memory read STOP mode and destination DMA channel for memory write STOP mode.
2. ADDITIONAL INFORMATION: This step starts the data transfer from one memory region to another through the memory-to-memory DMA channels and the CRC peripheral.
8. Poll the CRC\_STAT.DCNTEXP bit if the interrupt was disabled. ADDITIONAL INFORMATION: Perform this step only if counter expired interrupt is disabled. Polling ensures all the data has been processed.
9. Poll the CRC\_STAT.CMPERR bit if the interrupt was disabled to check for a compare error.

ADDITIONAL INFORMATION: Perform this step only if the compare error interrupt is not enabled.

10. Write the CRC\_STAT register to clear both the CRC\_STAT.DCNTEXP and the CRC\_STAT.CMPERR bits. ADDITIONAL INFORMATION: If interrupts were enabled, then clear these status bits within the interrupt

handlers for the respective interrupts.

The CRC compute-and-compare operation is now complete. The CRC peripheral is ready to be configured for the next CRC operation. The memory region has also been copied to its new destination.

The integrity check of the memory through the expected CRC signature has completed and the final result is indicated through the CRC\_STAT.CMPERR bit and the corresponding interrupt when enabled. The memory region has also been copied to its final destination.

Clear any W1C CRC status bits before performing a further CRC operation. Also, clear any W1C status bits of the memory-to-memory source and destination DMA channels before the next CRC operation.

## DMA Driven Memory Transfer Data Fill Mode

This mode initializes a region of memory to a constant 32-bit value using DMA transactions.

The task assumes the following:

- The polynomial has been loaded and the look-up table is fully initialized
- All CRC interrupts have been serviced (none pending)
- The CRC block is disabled per the CRC\_CTL.BLKEN bit
1. Initialize the CRC\_DCNT register. ADDITIONAL INFORMATION: The value loaded must represent the number of 32-bit words in the memory region for which the software calculates and verifies the signature.
2. Initialize the CRC\_DCNTRLD register. ADDITIONAL INFORMATION: This value is used to reload the CRC\_DCNT register upon completion of current CRC operation. If no further operation is needed, then this register can be initialized to zero.
3. Initialize the CRC\_FILLVAL register. ADDITIONAL INFORMATION: This register contains the 32-bit constant that the CRC module uses to fill the memory region.
4. Initialize the CRC\_INEN register. ADDITIONAL INFORMATION: The CRC module uses this register to enable the generation of the CRC interrupts for notification of block completion. Configure these interrupts as required. If enabled, ensure that the corresponding interrupt handlers are also configured.

5. Initialize the CRC\_CTL register with the CRC\_CTL.OPMODE bit set to memory transfer fill mode and the CRC\_CTL.BLKEN bit configured to enable the CRC peripheral.
2. The CRC peripheral is now enabled and is ready for the DMA channel to write data.
6. Configure and enable the memory-to-memory destination DMA channel for memory write STOP mode.
4. ADDITIONAL INFORMATION: This step starts the data transfer taking the constant 32-bit value from the CRC peripheral and writing the data to the DMA channel.
7. Poll the CRC\_STAT.DCNTEXP bit if the interrupt was disabled. ADDITIONAL INFORMATION: Perform this step only if counter expired interrupt is disabled. Polling ensures that all the data has been processed.
8. Write the CRC\_STAT register to clear the CRC\_STAT.DCNTEXP bit. ADDITIONAL INFORMATION: If interrupts were enabled, then clear this status bit within the interrupt handlers for the respective interrupts. The CRC memory transfer fill operation is complete, and the CRC peripheral is ready to be configured for the

next CRC operation.

The memory region is filled with the constant data and the CRC peripheral is ready to be configured for a new operation.

Clear any W1C CRC status bits and DMA status bits before performing a further CRC operation.

## ADSP-2159x\_SC592\_SC594 CRC Register Descriptions

Cyclic Redundancy Check Unit (CRC) contains the following registers.

Table 37-4: ADSP-2159x\_SC592\_SC594 CRC Register List

| Name         | Description                     |
|--------------|---------------------------------|
| CRC_COMP     | Data Compare Register           |
| CRC_CTL      | Control Register                |
| CRC_DCNT     | Data Word Count Register        |
| CRC_DCNTCAP  | Data Count Capture Register     |
| CRC_DCNTRLD  | Data Word Count Reload Register |
| CRC_DFIFO    | Data FIFO Register              |
| CRC_FILLVAL  | Fill Value Register             |
| CRC_INEN     | Interrupt Enable Register       |
| CRC_INEN_CLR | Interrupt Enable Clear Register |
| CRC_INEN_SET | Interrupt Enable Set Register   |

Table 37-4: ADSP-2159x\_SC592\_SC594 CRC Register List (Continued)

| Name           | Description                 |
|----------------|-----------------------------|
| CRC_POLY       | Polynomial Register         |
| CRC_RESULT_CUR | CRC Current Result Register |
| CRC_RESULT_FIN | CRC Final Result Register   |
| CRC_STAT       | Status Register             |

## Data Compare Register

The CRC\_COMP register contains the value corresponding to the expected CRC result or signature for the current data stream. At the end of the operation, the content of this register is used to compare against the result produced by the CRC operation. In data verify mode, each incoming data value is compared with the content of this register.

Figure 37-3: CRC\_COMP Register Diagram

<!-- image -->

Table 37-5: CRC\_COMP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Expected CRC Result Value. The CRC_COMP.VALUE bit field contains the value corresponding to the expected CRC result or signature for the current data stream. |

## Control Register

The CRC\_CTL register configures the operation modes and settings for the CRC.

Figure 37-4: CRC\_CTL Register Diagram

<!-- image -->

Table 37-6: CRC\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                         | Description/Enumeration                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 22 (R/W)           | CMPMIRR    | COMPARE Register Mirroring. The CRC_CTL.CMPMIRR bit enables data mirroring for the CRC_COMP compare register. When enabled, the 32-bit value in this register is fully bit mirrored (reversed). The bit-reversed value is used for comparison with the CRC_RESULT_FIN register. | COMPARE Register Mirroring. The CRC_CTL.CMPMIRR bit enables data mirroring for the CRC_COMP compare register. When enabled, the 32-bit value in this register is fully bit mirrored (reversed). The bit-reversed value is used for comparison with the CRC_RESULT_FIN register. |
| 21 (R/W)           | POLYMIRR   | Polynomial Register Mirroring. The CRC_CTL.POLYMIRR bit enables data mirroring for the CRC_POLY polyno- mial register. When enabled, the 32-bit value in this register is fully bit mirrored (re- versed). The bit-reversed value is used for CRC computations.                 | Polynomial Register Mirroring. The CRC_CTL.POLYMIRR bit enables data mirroring for the CRC_POLY polyno- mial register. When enabled, the 32-bit value in this register is fully bit mirrored (re- versed). The bit-reversed value is used for CRC computations.                 |
| 21 (R/W)           | POLYMIRR   | 0                                                                                                                                                                                                                                                                               | Disable polynomial mirroring                                                                                                                                                                                                                                                    |
| 21 (R/W)           | POLYMIRR   | 1                                                                                                                                                                                                                                                                               | Enable polynomial mirroring                                                                                                                                                                                                                                                     |

Table 37-6: CRC\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W)           | RSLTMIRR   | Result Register Mirroring. The CRC_CTL.RSLTMIRR bit enables data mirroring for the CRC_RESULT_CUR and CRC_RESULT_FIN result registers. When enabled, the 32-bit values in these registers are fully bit mirrored (reversed).                                                                                                |
| 20 (R/W)           | RSLTMIRR   | 0 Disable result mirroring                                                                                                                                                                                                                                                                                                  |
| 20 (R/W)           | RSLTMIRR   | 1 Enable result mirroring                                                                                                                                                                                                                                                                                                   |
| 19 (R/W)           | FDSEL      | FIFO Data Select. The CRC_CTL.FDSEL bit selects whether the CRC writes modified or unmodified data to the FIFO in memory transfer mode. If enabled, the data written is affected by the state of the data mirroring selections ( CRC_CTL.BITMIRR , CRC_CTL.BYTMIRR , and CRC_CTL.W16SWP ) before being written to the FIFO. |
| 19 (R/W)           | FDSEL      | 0 Write unmodified data to FIFO                                                                                                                                                                                                                                                                                             |
| 19 (R/W)           | FDSEL      | 1 Write modified data to FIFO                                                                                                                                                                                                                                                                                               |
| 18 (R/W)           | W16SWP     | Word16 Swapping. The CRC_CTL.W16SWP bit enables the CRC's data mirror block to swap the upper and lower 16-bit words within the 32-bit input data, before further processing.                                                                                                                                               |
| 18 (R/W)           | W16SWP     | 0 Disable word16 swapping                                                                                                                                                                                                                                                                                                   |
| 18 (R/W)           | W16SWP     | 1 Enable word16 swapping                                                                                                                                                                                                                                                                                                    |
| 17 (R/W)           | BYTMIRR    | Byte Mirroring. The CRC_CTL.BYTMIRR bit enables the CRC's data mirror block to mirror the bytes within the 32-bit input data, before further processing.                                                                                                                                                                    |
| 17 (R/W)           | BYTMIRR    | 0 Disable byte mirroring                                                                                                                                                                                                                                                                                                    |
| 17 (R/W)           | BYTMIRR    | 1 Enable byte mirroring                                                                                                                                                                                                                                                                                                     |
| 16 (R/W)           | BITMIRR    | Bit Mirroring. The CRC_CTL.BITMIRR bit enables the CRC's data mirror block to mirror the bits within each byte of the 32-bit input data, before further processing.                                                                                                                                                         |
| 16 (R/W)           | BITMIRR    | 0 Disable bit mirroring                                                                                                                                                                                                                                                                                                     |
| 16 (R/W)           | BITMIRR    | 1 Enable bit mirroring                                                                                                                                                                                                                                                                                                      |
| 13 (R/W)           | IRRSTALL   | Intermediate Result Ready Stall. The CRC_CTL.IRRSTALL bit enables stalling the state machine for input data when there is a valid intermediate result to be read in the CRC_RESULT_CUR regis- ter. This feature should be used only in CRC computation modes (for example, CRC_CTL.OPMODE =1 or =3).                        |
| 13 (R/W)           | IRRSTALL   | 0 Do not stall                                                                                                                                                                                                                                                                                                              |
| 13 (R/W)           | IRRSTALL   | 1 Stall on IRR                                                                                                                                                                                                                                                                                                              |

Table 37-6: CRC\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | OBRSTALL   | Output Buffer Ready Stall. The CRC_CTL.OBRSTALL bit enables stalling the state machine for input data when there is valid data in the output buffer. This feature should be used only in mem- ory-to-memory transfer modes (for example, CRC_CTL.OPMODE =1).                                                                                       |
| 9 (R/W)            | AUTOCLRF   | 1 Stall on OBR Auto Clear to One. The CRC_CTL.AUTOCLRF bit enables auto clear to one when the CRC is in inter- mediate results ready stall mode ( CRC_CTL.IRRSTALL =1) and the CRC data count expires ( CRC_DCNT =0). Note that the CRC_CTL.AUTOCLRZ bit must be disabled, or the CRC_CTL.AUTOCLRF bit has no effect. 0 No auto clear 1 Auto clear |
| 8 (R/W)            | AUTOCLRZ   | Auto Clear to Zero. The CRC_CTL.AUTOCLRZ bit enables auto clear to zero when the CRC is in inter- mediate results ready stall mode ( CRC_CTL.IRRSTALL =1) and the CRC data count expires ( CRC_DCNT =0). Note that CRC_CTL.AUTOCLRF must be disabled, or the CRC_CTL.AUTOCLRZ has no effect. 0 No auto clear                                       |
| 7:4 (R/W)          | OPMODE     | 1 Auto clear Operation Mode. The CRC_CTL.OPMODE bit field selects the memory transfer or scan mode. 0 Reserved 1 CRC compute/compare memory transfer 2 Data fill memory transfer 3 CRC compute/compare memory scan 4 Data verify memory scan                                                                                                       |
| 0                  |            |                                                                                                                                                                                                                                                                                                                                                    |
|                    | BLKEN      | Block Enable. The CRC_CTL.BLKEN bit enables and disables the CRC operation.                                                                                                                                                                                                                                                                        |
| (R/W)              |            |                                                                                                                                                                                                                                                                                                                                                    |
|                    |            | 0 Disable                                                                                                                                                                                                                                                                                                                                          |
|                    |            | 1 Enable                                                                                                                                                                                                                                                                                                                                           |

## Data Word Count Register

The CRC\_DCNT register holds the word count that is used for the CRC operation. On transfer of every 32-bit word, the CRC decrements by 1 the content of this register. When the count decrements to zero, this event triggers a CRC compare action, and the CRC\_DCNT register is automatically loaded from the CRC\_DCNTRLD register for the next CRC operation.

Note that the initial value programmed into the CRC\_DCNT register may be different from what is programmed in the CRC\_DCNTRLD register.

Figure 37-5: CRC\_DCNT Register Diagram

<!-- image -->

Table 37-7: CRC\_DCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Data Word Count. The CRC_DCNT.VALUE bit field holds the word count that is used for the CRC op- eration. |

## Data Count Capture Register

The CRC\_DCNTCAP register captures the CRC\_DCNT value when a compare operation fails in data verify mode. This capture can be used to track the position of an error in the data stream. The capture operation is enabled only if the CRC\_STAT.CMPERR bit indicates no compare error. After an error occurs and the data count is captured, no further errors are logged until the CRC\_STAT.CMPERR bit is cleared. T o obtain the position of an error in the data stream, subtract the CRC\_DCNTCAP register value from the initial CRC\_DCNT .

Figure 37-6: CRC\_DCNTCAP Register Diagram

<!-- image -->

Table 37-8: CRC\_DCNTCAP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Data Count Capture Value. The CRC_DCNTCAP.VALUE bit field contains the CRC_DCNT value when a com- pare operation fails in data verify mode. |

## Data Word Count Reload Register

The CRC\_DCNTRLD register holds the value that the CRC automatically loads into CRC\_DCNT when the CRC\_DCNT decrements to 0. At startup, the value programmed in CRC\_DCNT and the CRC\_DCNTRLD register could be different. So, for the first iteration, the CRC operation happens for the count initially programmed in the CRC\_DCNT register. While for subsequent CRC operations, the count is taken from the CRC\_DCNTRLD register.

Figure 37-7: CRC\_DCNTRLD Register Diagram

<!-- image -->

Table 37-9: CRC\_DCNTRLD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Reload Value. The CRC_DCNTRLD.VALUE bit field holds the value that automatically loads into CRC_DCNT when the CRC_DCNT decrements to 0. |

## Data FIFO Register

In memory transfer mode (non-data fill mode), the data from the DMA or processor core buses is written into the CRC\_DFIFO on each input data grant (DMA grant or core write). Data is read from this FIFO on each output data grant (DMA grant or core read). FIFO status information is available in the CRC\_STAT register. Whenever, the FIFO has valid data, output data requests are generated.

Note that in non-memory transfer mode and in data fill mode, the input data does not get written into this FIFO. So, this register should not be read in these modes.

Figure 37-8: CRC\_DFIFO Register Diagram

<!-- image -->

Table 37-10: CRC\_DFIFO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Data FIFO Value. The CRC_DFIFO.VALUE bit field is the data from the DMAor processor core bus- es. |

## Fill Value Register

The CRC\_FILLVAL register holds the value that the CRC uses for the memory fill operation. In data fill mode, the value programmed in this register is used for the memory fill operation.

Figure 37-9: CRC\_FILLVAL Register Diagram

<!-- image -->

Table 37-11: CRC\_FILLVAL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Memory Fill Value. The CRC_FILLVAL.VALUE bit field holds the value that the CRC uses for the memory fill operation. |

## Interrupt Enable Register

The CRC\_INEN register unmasks (enables) or masks (disables) interrupt requests generated in the CRC from going to the processor core.

Note that CRC interrupts are not disabled when the CRC is disabled ( CRC\_CTL.BLKEN =0).

Figure 37-10: CRC\_INEN Register Diagram

<!-- image -->

Table 37-12: CRC\_INEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | DCNTEXP    | Data Count Expired (Status) Interrupt Enable. The CRC_INEN.DCNTEXP enables (unmasks) the data count expired (CRC status) interrupt. 0 Disable (mask) interrupt                                              |
| 1 (R/W)            | CMPERR     | Compare Error Interrupt Enable. The CRC_INEN.CMPERR enables (unmasks) the data compare interrupt, which is generated when CRC data comparison fails. 0 Disable (mask) interrupt 1 Enable (unmask) interrupt |

## Interrupt Enable Clear Register

The CRC\_INEN\_CLR register permits clearing individual bits in the CRC\_INEN register without affecting other bits in the register.

Figure 37-11: CRC\_INEN\_CLR Register Diagram

<!-- image -->

Table 37-13: CRC\_INEN\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 4 (R0/WC)          | DCNTEXP    | Data Count Expired. The CRC_INEN_CLR.DCNTEXP bit clears the data count expired (status) interrupt. 0 No effect |
| 1 (R0/WC)          | CMPERR     | Compare Error Clear. The CRC_INEN_CLR.CMPERR bit clears the compare error interrupt. 0 No effect 1 Clear bit   |

## Interrupt Enable Set Register

The CRC\_INEN\_SET register permits setting individual bits in the CRC\_INEN register without affecting other bits in the register.

Figure 37-12: CRC\_INEN\_SET Register Diagram

<!-- image -->

Table 37-14: CRC\_INEN\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R0/WS)          | DCNTEXP    | Data Count Expired (Status) Interrupt Enable Set. The CRC_INEN_SET.DCNTEXP bit sets the data count expired (status) interrupt. effect |
| 4 (R0/WS)          | DCNTEXP    | 0 No                                                                                                                                  |
| 1 (R0/WS)          | CMPERR     | Compare Error Interrupt. The CRC_INEN_SET.CMPERR bit sets the compare error interrupt. 0 No effect bit                                |
| 1 (R0/WS)          | CMPERR     | 1 Set                                                                                                                                 |
| 1 (R0/WS)          | CMPERR     |                                                                                                                                       |

## Polynomial Register

The CRC\_POLY register holds a 32-bit polynomial for CRC operations. Bit 31 corresponds to the coefficient of x31 of the CRC polynomial, bit 30 corresponds to the coefficient of x30, and so on through bit 0. A coefficient of x32 is assumed to be "1" for any polynomial that is selected. Based on the polynomial in the CRC\_POLY register, the CRC generates a look-up table (LUT), which is used to compute the CRC of the incoming data stream.

Figure 37-13: CRC\_POLY Register Diagram

<!-- image -->

Table 37-15: CRC\_POLY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                      |
|--------------------|------------|------------------------------------------------------------------------------|
| 31:0               | VALUE      | CRC Polynomial Value.                                                        |
| (R/W)              |            | The CRC_POLY.VALUE bit field holds the 32-bit polynomial for CRC operations. |

## CRC Current Result Register

The CRC\_RESULT\_CUR register holds the current or intermediate CRC result. It is updated when new data is written into the CRC. Each time the CRC\_DCNT expires, the CRC loads the value from this register into the CRC\_RESULT\_FIN register. The CRC\_RESULT\_CUR register may be set to auto clear to zero or auto clear to ones when CRC\_DCNT expires by configuring the CRC\_CTL.AUTOCLRZ and CRC\_CTL.AUTOCLRF bits. Before starting a CRC operation, the CRC\_RESULT\_CUR register should be programmed to the desired value.

Note that this register can be read by the processor core at any time.

Figure 37-14: CRC\_RESULT\_CUR Register Diagram

<!-- image -->

Table 37-16: CRC\_RESULT\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Intermediate CRC Result. The CRC_RESULT_CUR.VALUE bit field holds the current or intermediate CRC result. |

## CRC Final Result Register

The CRC\_RESULT\_FIN register holds the final CRC computed for a data stream. A data stream is a DMA of CRC\_DCNT number of words into the CRC. When CRC\_DCNT decrements to zero for each data stream, the CRC loads the CRC\_RESULT\_FIN register with the value from the CRC\_RESULT\_CUR register.

Figure 37-15: CRC\_RESULT\_FIN Register Diagram

<!-- image -->

Table 37-17: CRC\_RESULT\_FIN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Computed CRC. The CRC_RESULT_FIN.VALUE bit field holds the final CRC computed for a data stream. |

## Status Register

The CRC\_STAT register indicates the status for CRC operations and interrupt generation.

Figure 37-16: CRC\_STAT Register Diagram

<!-- image -->

Table 37-18: CRC\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 22:20 (R/NW)       | FSTAT      | FIFO Status. The CRC_STAT.FSTAT indicates the current FIFO status. This field is read-only.                                                                                                                                  |
| 22:20 (R/NW)       | FSTAT      | 0 FIFO empty                                                                                                                                                                                                                 |
| 22:20 (R/NW)       | FSTAT      | 1 FIFO has 1 data                                                                                                                                                                                                            |
| 22:20 (R/NW)       | FSTAT      | 2 FIFO has 2 data                                                                                                                                                                                                            |
| 22:20 (R/NW)       | FSTAT      | 3 FIFO has 3 data                                                                                                                                                                                                            |
| 22:20 (R/NW)       | FSTAT      | 4 FIFO has 4 data (full)                                                                                                                                                                                                     |
| 19 (R/NW)          | LUTDONE    | Look-Up Table Done. The CRC_STAT.LUTDONE bit indicates that the CRC has generated the look-up ta- ble for the current polynomial. This read-only bit is cleared at reset and cleared when the CRC_POLY is written. No status |
| 19 (R/NW)          | LUTDONE    | 0                                                                                                                                                                                                                            |
| 19 (R/NW)          | LUTDONE    | 1 LUT generation done                                                                                                                                                                                                        |

Table 37-18: CRC\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (R/NW)          | IRR        | Intermediate Result Ready. The CRC_STAT.IRR bit indicates that the CRC has updated the CRC_RESULT_CUR register with intermediate CRC results for the new data written to the CRC. The processor core should read from the CRC_RESULT_CUR register only after detecting CRC_STAT.IRR =1. This read-only bit is cleared by CRC hard-                                                                      |
| 18 (R/NW)          | IRR        | 0 No status                                                                                                                                                                                                                                                                                                                                                                                             |
| 18 (R/NW)          | IRR        | 1 Intermediate results ready                                                                                                                                                                                                                                                                                                                                                                            |
| 17 (R/NW)          | OBR        | Output Buffer Ready. The CRC_STAT.OBR bit indicates that the CRC has data ready for the processor core to read. The processor core should read from the CRC only after detecting CRC_STAT.OBR =1. This read-only bit is cleared by CRC hardware.                                                                                                                                                        |
| 17 (R/NW)          | OBR        | 0 No status                                                                                                                                                                                                                                                                                                                                                                                             |
| 17 (R/NW)          | OBR        | 1 Output buffer ready                                                                                                                                                                                                                                                                                                                                                                                   |
| 16 (R/NW)          | IBR        | Input Buffer Ready. The CRC_STAT.IBR bit indicates that the CRC is ready to accept a processor core write. The processor core should write to the input register only after detecting that CRC_STAT.IBR =1. This read-only bit is cleared by CRC hardware.                                                                                                                                              |
| 16 (R/NW)          | IBR        | 0 No status                                                                                                                                                                                                                                                                                                                                                                                             |
| 16 (R/NW)          | IBR        | 1 Input buffer ready                                                                                                                                                                                                                                                                                                                                                                                    |
| 4 (R/W1C)          | DCNTEXP    | Data Count Expired. The CRC_STAT.DCNTEXP bit indicates that the CRC_DCNT has expired. This W1C bit is not automatically cleared when the CRC is disabled ( CRC_CTL.BLKEN =0). When the CRC sets this bit on CRC_DCNT expiry, the CRC generates the CRC_INEN.DCNTEXP interrupt.                                                                                                                          |
| 4 (R/W1C)          | DCNTEXP    | 0 No status                                                                                                                                                                                                                                                                                                                                                                                             |
| 4 (R/W1C)          | DCNTEXP    | 1 Data counter expired                                                                                                                                                                                                                                                                                                                                                                                  |
| 1 (R/W1C)          | CMPERR     | Compare Error. The CRC_STAT.CMPERR bit indicates that a CRC mismatch or data mismatch has been detected. This W1C bit is not automatically cleared when the CRC is disabled ( CRC_CTL.BLKEN =0). When the CRC sets this bit on detecting a mismatch, the CRC generates the CRC_INEN.CMPERR interrupt. While this bit is set, the CRC_DCNTCAP register is disabled from capturing the data count values. |
| 1 (R/W1C)          | CMPERR     | 0 No status                                                                                                                                                                                                                                                                                                                                                                                             |
| 1 (R/W1C)          | CMPERR     | 1 Compare error                                                                                                                                                                                                                                                                                                                                                                                         |