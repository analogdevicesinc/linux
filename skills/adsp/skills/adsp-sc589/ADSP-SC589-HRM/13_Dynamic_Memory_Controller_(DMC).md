## 10   Dynamic Memory Controller (DMC)

The dynamic memory controller (DMC) provides a glueless interface between DDR3/DDR2/LPDDR SDRAMs and the system crossbar interface (SCB). The DMC enables execution of instructions from, as well as transfer of data to and from, DDR3, DDR2 SDRAM or LPDDR SDRAM respectively.

- NOTE: The terms DDR2, DDR3, and LPDDR SDRAM are referred to generically as DDR SDRAM in the rest of this chapter unless otherwise noted.
- NOTE: According to JEDEC STD specification (JESD79 -3 -1A.01) DDR3L marked devices (1.35V) can also be connected to ADSP-SC589 processors as these devices also meet 1.5V voltage level specifications of DDR3 marked devices.

The DMC is partitioned in a manner that allows reconfiguration and maintainability. The memory access protocol state machine along with JEDEC standard specific logic is embedded in the protocol controller . An access and operation reordering mechanism is incorporated as an efficiency controller . An SCB slave interface is provided to interface with the on-chip interconnect. This interface results in an efficient slave implementation owing to its out-of-order transaction capabilities. The control and status registers present in the DMC can be accessed using the MMR access bus.

The DMC supports access to the external memory by core and DMA accesses.

## DMC Features

The DMC includes a protocol controller that supports:

- JESD79-3E compatible double data rate DDR3 SDRAM devices
- JESD79-2E compatible DDR2 SDRAM devices
- JESD209A low-power DDR (LPDDR) SDRAM devices

The features of the dynamic memory controller are:

- Provides 16-bit data only interface to SDRAM devices
- Supports a single external rank (one chip select)
- Supports various burst lengths

- Provides page hit detection that supports multiple column accesses to the same row
- User-specified active, precharge, and refresh commands.
- Programmable SDRAM access timing parameters
- Enables automatic refresh generation with programmable refresh intervals
- Self-refresh mode to reduce system power consumption
- Efficient transaction processing to improve throughput and bandwidth using:
- Software programmable SCB IDs to allow SCB ID-based priority
- The ability to postpone up to eight auto-refresh commands
- Software selectable closed page scheme on a per bank basis
- Simple transaction scheduling mechanism to reduce read write turnaround frequency on the memory bus
- Accesses with the same SCB ID are scheduled back-to-back to take advantage of same page access in SDRAM
- Caching of SDRAM read data burst for specific masters to reduce the latency for same burst accesses.

## The DDR2 features are:

- 256M bit to 4G-bit device sizes
- Burst length BL = 4 or 8
- Support for additive latency
- Support for programmable ODT and drive impedance from memory end
- Support for programmable (and ZQ calibration) ODT and drive impedance from the processor end

## The DDR 3 features are:

- 512 Mb to 8 Gb device sizes
- Burst length BL = 8
- Support for additive latency
- Support for programmable (and ZQ calibration) ODT and drive impedance
- Support for programmable (and ZQ calibration) ODT and drive impedance from the processor end

## The LPDDR features are:

- 64M bit to 2G-bit device sizes
- Burst length BL = 4 or 8
- Support for deep power-down mode

## Feature Exclusions

The DMC exclusions are as follows:

For DDR2:

- 4-bit and 8-bit wide DDR2 DRAM memories are not supported
- OCD is not supported
- Burst interleaved accesses are not supported
- Single-ended signaling mode not supported

## For DDR3:

- 4-bit and 8-bit wide DDR2 DRAM memories are not supported
- Burst interleaved accesses are not supported
- Both burst chop-BC4 and BC4-on-the-fly are not supported
- Auto refresh pull-in is not supported
- Write leveling is not supported
- DLL off mode is not supported

## For LPDDR:

- 32-bit wide LPDDR memory devices are not supported
- Status register read (SRR) is not supported
- Sampling the optional temperature output (TQ) signal is not supported
- Clock stop mode is not supported
- Bursts of 2 and 16 words are not supported
- No support for BURST\_TERMINATE command
- Dual-die, two CS# and two CKE packages are not supported

## DMC Functional Description

The dynamic memory controller consists of master and slave interfaces, a protocol controller, and an efficiency controller. The following sections describe the function of these interfaces and controllers.

## ADSP-SC58x DMC Register List

The Dynamic Memory Controller module (DMC) provides an interface to external double-data-rate SDRAM. This interface supports various DDR standards (see chapter descriptions). A set of registers governs DMC controller operations. For more information on DMC controller functionality, see the DMC Controller Register Descriptions.

Table 10-1: ADSP-SC58x DMC Register List

| Name                    | Description                                                |
|-------------------------|------------------------------------------------------------|
| DMC_CFG                 | Configuration Register                                     |
| DMC_CPHY_CTL            | Controller to PHY Interface Register                       |
| DMC_CTL                 | Control Register                                           |
| DMC_DLLCTL              | DLL Control Register                                       |
| DMC_DT_CALIB_ADDR       | Data Calibration Address Register                          |
| DMC_DT_DATA_CALIB_DATA0 | Data Calibration Data 0 Register                           |
| DMC_DT_DATA_CALIB_DATA1 | Data Calibration Data 1 Register                           |
| DMC_EFFCTL              | Efficiency Control Register                                |
| DMC_EMR1                | Shadow EMR1 DDR2 Register                                  |
| DMC_EMR2                | Shadow EMR2 Register (DDR2)/Shadow EMR Register (LPDDR)    |
| DMC_MR                  | Shadow MRRegister (DDR2/LPDDR), Shadow MR0 Register (DDR3) |
| DMC_MR1                 | Shadow MR1 Register (DDR3)                                 |
| DMC_MR2                 | Shadow MR2 Register (DDR3)                                 |
| DMC_MSK                 | Mask (Mode Register Shadow) Register                       |
| DMC_PRIO                | Priority ID Register 1                                     |
| DMC_PRIO2               | Priority ID Register 2                                     |
| DMC_PRIOMSK             | Priority ID Mask Register 1                                |
| DMC_PRIOMSK2            | Priority ID Mask Register 2                                |
| DMC_RDDATABUFID1        | DMCRead Data Buffer ID Register 1                          |
| DMC_RDDATABUFID2        | DMCRead Data Buffer ID Register 2                          |
| DMC_RDDATABUFMSK1       | DMCRead Data Buffer Mask Register 1                        |
| DMC_RDDATABUFMSK2       | DMCRead Data Buffer Mask Register 2                        |
| DMC_STAT                | Status Register                                            |
| DMC_TR0                 | Timing 0 Register                                          |
| DMC_TR1                 | Timing 1 Register                                          |
| DMC_TR2                 | Timing 2 Register                                          |

## ADSP-SC58x DMC Register List

A set of registers governs DMCPHY operations. For more information on functionality, see the register descriptions.

Table 10-2: ADSP-SC58x DMC Register List

| Name            | Description                        |
|-----------------|------------------------------------|
| DMC_CAL_PADCTL0 | Calibration PAD Control 0 Register |
| DMC_CAL_PADCTL2 | Calibration PAD Control 2 Register |
| DMC_PHY_CTL0    | PHY Control 0 Register             |
| DMC_PHY_CTL1    | PHY Control 1 Register             |
| DMC_PHY_CTL2    | PHY Control 2 Register             |
| DMC_PHY_CTL3    | PHY Control 3 Register             |
| DMC_PHY_CTL4    | PHY Control 4 Register             |

## Protocol Controller

The DDR SDRAM protocol controller translates memory access requests from the SCB (system crossbar) interface to JEDEC protocol-specific transactions used by DDR SDRAM devices.

The protocol controller ensures that the various timing parameters are met before reading and writing the SDRAM. The controller also performs the SDRAM initialization sequence as mandated by the standard. The protocol controller can:

- Issue reads and writes
- Precharge a row in a bank
- Activate a row in a bank
- Put the SDRAM devices in self-refresh and power-down modes

The protocol controller takes mode register writes from the MMR interface and translates them into mode register writes to SDRAM. Writing into the mode register is restricted through a mask register.

## Efficiency Controller

The efficiency controller controls the ordering of transfers buffered in the read and write command buffers. It attempts to order transfers to optimize the available memory bandwidth. The DMC uses a number of schemes, described in the following sections, to increase the throughput.

## Page-Based Scheduling

The DMC parses each write and read transaction that it buffered and gets the information of the row (page) and bank address. The protocol controller maintains the information about the pages that are opened in each bank. The efficiency controller uses the information about the opened pages while scheduling the buffered transactions. The transactions to the opened pages are given higher priority than the other outstanding transactions.

## Same Master Transaction Scheduling

The DMC also stores the ID of each transaction that it buffered. In most of the cases, the transactions related to a master result in page hits from the locality of reference rule. The efficiency controller uses the ID information of the

transactions while scheduling. When the page-based scheduling of the buffered transactions is complete, same master transaction scheduling is triggered. If multiple transactions from a master are received, the efficiency controller schedules the transactions back-to-back.

## DMC Read Data Buffer

The DMC read data buffer contains a data buffer and an address buffer. The depth of the data buffer is equal to the burst length that is programmed in SDRAM. The address buffer holds the corresponding SDRAM burst address. When an SDRAM write address from any master matches an address in the DMC read data buffer, the DMC invalidates the related data in the read buffer. When the DMC\_RDDATABUFMSK1 or DMC\_RDDATABUFMSK2 register is programmed with a value other than zero, the DMC read data buffer operation is enabled. The set of masters whose data is buffered and retrieved are programmed in the DMC\_RDDATABUFID1 or DMC\_RDDATABUFID2 registers. The DMC can use the DMC\_RDDATABUFMSK1 and DMC\_RDDATABUFMSK2 ID registers to select a set of masters similar to the programming of the DMC\_PRIOMSK and DMC\_PRIOMSK2 registers.

See the SCB ID-Based Priority section for details.

## Closed Page Per Bank

The DMC\_EFFCTL register provides per-bank granularity for closing pages. The software can determine that most accesses to a given bank in memory always result in a missed page. In this case, set the PREC\_BANK bit corresponding to the required bank to close the row after every transfer. This proactive step can result in reduced thrashing and increases memory throughput.

## SCB ID-Based Priority

The primary goal of the dynamic memory controller is to improve sustainable memory system bandwidth so that the service time for the average request can be reduced. However, to service critical requests from any master in the system, the DMC provides a mechanism to elevate priority of a given access. The DMC priority ID registers ( DMC\_PRIO and DMC\_PRIO2 ) can be programmed with up to two SCB IDs with elevated priority.

After every access in a snapshot, the command buffers are searched to determine whether a commands ID matches with the ID programmed in the DMC\_PRIO and DMC\_PRIO2 registers. The priority SCB ID access is sent before the subsequent access in the snapshot if:

- A match occurs, and
- The direction of the access (for example write) is the same as the direction of the snapshot (write)

There is an alternative to providing priority to a specific SCB ID. If a number of IDs from the same master require priority, program the DMC priority mask ID registers ( DMC\_PRIOMSK and DMC\_PRIOMSK2 ) so that the corresponding bits are 0. The DMC uses a combination of the DMC\_PRIO and DMC\_PRIO2 registers and the DMC\_PRIOMSK / DMC\_PRIOMSK2 registers to elevate the priority of a select few or all IDs that belong to a master. By default, none of the IDs are prioritized. The following are a few possibilities.

- The DMC\_PRIOMSK field is set to 0x00000000. If a single ID (7234) needs priority, set the DMC\_PRIOMSK field to 0xFFFFFFFF and set the DMC\_PRIO field to 7234.
- If the DMC\_PRIOMSK field is set to 0xFFFFFFFE, the SCB IDs 7234 and 7235 are given priority.

- If the DMC\_PRIOMSK field is set to 0xFFFFFFFC, the SCB IDs 7234, 7235, 7236 and 7237 are given priority.
- If two transactions with priority, one read and the other a write, are outstanding, the priority transaction that does not change the direction of the DMC access gets priority.
- The other priority transaction is handled at the beginning of the next snapshot. For example, if a write snapshot is in progress, the write priority transaction is sent. The read priority transaction is sent at the beginning of the next read snapshot.

NOTE: Use SCB-ID-based priority judiciously because it can significantly reduce the throughput of the DMC.

## Delaying up to Eight Auto-Refresh Commands

The DMC uses this method to ensure that auto-refresh does not interfere with any critical data transfers. Up to eight auto-refresh commands can accumulate in the DMC. The exact number of auto-refresh commands can be programmed using the DMC\_EFFCTL.NUMREF bit.

After the first refresh command is accumulated, the DMC constantly looks for an opportunity to schedule a refresh command. When the SCB read and write command buffers become empty for the programmed number of clock cycles ( DMC\_EFFCTL.IDLECYC bit field), the accumulated number of refresh commands are sent back-to-back to the DRAM. (The empty state of the SCB command buffers implies that no access is outstanding.)

After every refresh, the SCB command buffers are checked to ensure that they stay empty. If the SCB command buffers are always full, once the programmed number of refresh commands accumulates, the refresh operation is elevated to urgent priority. One refresh command is sent immediately. After this process, the DMC continues to wait for an opportunity to send out refresh commands. If self-refresh mode is enabled, all pending refresh commands are given out only after that DMC enters into self-refresh mode.

## Page and Bank Interleaving

Page and bank interleaving allow consecutive row accesses to fall into the same bank (bank interleaving) or into a different bank (page interleaving). The DMC uses bank interleaving by default ( DMC\_CTL.ADDRMODE bit =0). If the DMC\_CTL.ADDRMODE bit =1, the DMC uses page interleaving. Page misses in one addressing mode result in hits in the other addressing mode.

## System Crossbar Slave Interface

The DMC uses the system crossbar slave interface to move all data. The system crossbar interface accepts interleaved write transactions and sends out-of-order responses. The read and write interfaces consist of buffers for address, data, and control information transferred to or from the system crossbar bus.

The system crossbar interface transactions are sent to the SDRAM only after the SDRAM has been initialized. However, if transactions arrive before or during initialization, they accumulate in the system crossbar interface and are sent out to the protocol controller once the initialization completes.

To increase throughput, the system crossbar write-response is sent out as soon as the final DDR burst is scheduled for transfer into the SDRAM. However, if an auto-refresh is needed, the scheduled write data is sent only after the

auto-refresh. A delay can occur. The delay is a maximum of 64 clock cycles from the moment the write response is sent on the SCB to the write operation of the data into SDRAM.

The system crossbar interface performs the following operations:

- Buffers read and write command requests from the system crossbar bus
- Processes the requests by converting them to protocol controller user-interface transfers
- Sends and receives data to or from the protocol controller
- Creates a suitable read/write response and sends read data back to the system crossbar bus

The system crossbar slave interface supports the following:

- All burst lengths (1-16)
- Incremental and wrap bursts
- Data transfer sizes of 8-bit, 16-bit, or 32-bit
- Arrival of write data before write address
- Generation of error responses which include:
- Any access to an unimplemented region of the external memory space
- Any access when the SDRAM is in self-refresh, power-down, or deep power-down mode (in case of LPDDR)
- Any access when the direct command interface is in operation

## Read/Write Command and Data Buffers

The system crossbar interface consists of a four-deep read command buffer and a four-deep write command buffer. Up to four write commands and four read commands can be waiting for access to the SDRAM. The system crossbar write buffer is 32 deep. It can support write data interleaving of two. The system crossbar read buffer is 32 deep.

## Peripheral Bus Slave Interface

The peripheral bus slave interface connects the dynamic memory controller to the peripheral bus and provides a host controller with access to the registers. The peripheral bus slave interface supports the following features:

- Read and write word accesses
- 32-bit data bus

## Architectural Concepts

The following sections provide information on the architecture of the interface.

## Controller On Die Termination (ODT)

The controller ODT is enabled with the granularity of a byte lane. The description of this feature can be obtained in the description of the corresponding PHY registers. Controller ODT involves extra overhead in terms of power consumption during reads.

The DMC implements dynamic on die termination at processor pads. When controller ODT is enabled, the termination resistors in the pads are turned on when the controller reads data from the DRAM. These resistors are turned off when the controller writes to the DRAM.

## Mode Register Set and Extended Mode Register Set Command

The load mode register command initializes the SDRAM operation parameters. The DMC supports the mode register set and extended mode register set commands. The controller automatically issues the mode register set command during power-on initialization and also when the DMC\_MR register is written with the DMC\_MSK.MR bit. The mode register set command is sent after the ongoing data transfer completes.

The DMC automatically issues the mode register set command when the shadow EMR1/EMR2/EMR3 registers are written. The corresponding DMC\_MSK.EMR3DMC\_MSK.EMR2 / DMC\_MSK.EMR1 bits must be enabled.

## DDR3 Reset Functionality

DDR3 contains an additional pin corresponding to reset functionality. Reset is part of the initialization sequence but it can be performed asynchronously when needed. The reset procedure is similar to the steps involved in the initialization except the initial part of power-up.

To perform reset on the DDR3 module:

1. Check to ensure the module is in the idle state by polling the DMC\_STAT.IDLE bit (0x0008).
2. Set the DMC\_CTL.RESET bit (0x0004).
3. Monitor the DMC\_STAT.RESETDONE bit for the completion of the reset function.

Do not perform any transactions during a module reset. Wait for the DMC\_STAT.RESETDONE signal.

## DDR3 SDRAM Organization

The DMC supports DDR3 SDRAM memory modules ranging from 512 Mb to 8 Gb. The following tables list the address translation mechanism from the user interface to DDR3 memory interface. The controller also supports two types of addressing modes: bank interleaving ( DMC\_CTL.ADDRMODE =1) and page interleaving ( DMC\_CTL.ADDRMODE =0).

## Bank Interleaving

The DDR3 Bank Interleaving table shows DDR3 bank interleaving.

Table 10-3: DDR3 Bank Interleaving

| SDRAM size   | Bank address bits   | Row address bits   | Column address bits   |
|--------------|---------------------|--------------------|-----------------------|
| 512 Mb       | 25:24               | 23:11              | 10:1                  |

Table 10-3: DDR3 Bank Interleaving (Continued)

| SDRAM size   | Bank address bits   | Row address bits   | Column address bits   |
|--------------|---------------------|--------------------|-----------------------|
| 1 Gb         | 26:24               | 23:11              | 10:1                  |
| 2 Gb         | 27:25               | 24:11              | 10:1                  |
| 4 Gb         | 28:26               | 25:11              | 10:1                  |
| 8 Gb         | 29:27               | 26:11              | 10:1                  |

## Page Interleaving

The DDR3 Page Interleaving table shows DDR3 page interleaving.

Table 10-4: DDR3 Page Interleaving

| SDRAM size   | Row address bits   | Bank address bits   | Column address bits   |
|--------------|--------------------|---------------------|-----------------------|
| 512 Mb       | 25:13              | 12:11               | 10:1                  |
| 1 Gb         | 26:14              | 13:11               | 10:1                  |
| 2 Gb         | 27:14              | 13:11               | 10:1                  |
| 4 Gb         | 28:14              | 13:11               | 10.1                  |
| 8 Gb         | 29:14              | 13:11               | 10.1                  |

## DDR2 SDRAM Organization

The DMC supports DDR2 SDRAM memory modules ranging from 256M bit to 4G bit. The following tables list the address translation mechanism from the user interface to DDR2 memory interface. The controller also supports two types of addressing modes: bank interleaving ( DMC\_CTL.ADDRMODE =1) and page interleaving ( DMC\_CTL.ADDRMODE =0).

## Bank Interleaving

The DDR2 Bank Interleaving table shows DDR2 bank interleaving.

Table 10-5: DDR2 Bank Interleaving

| SDRAM size   | Bank address bits   | Row address bits   | Column address bits   |
|--------------|---------------------|--------------------|-----------------------|
| 256 Mb       | 24:23               | 22:10              | 9:1                   |
| 512 Mb       | 25:24               | 23:11              | 10:1                  |
| 1 Gb         | 26:24               | 23:11              | 10:1                  |
| 2 Gb         | 27:25               | 24:11              | 10:1                  |
| 4 Gb         | 28:26               | 25:11              | 10:1                  |

## Page Interleaving

The DDR2 Page Interleaving table shows DDR2 page interleaving.

Table 10-6: DDR2 Page Interleaving

| SDRAM size   | Row address bits   | Bank address bits   | Column address bits   |
|--------------|--------------------|---------------------|-----------------------|
| 256 Mb       | 24:12              | 11:10               | 9.1                   |
| 512 Mb       | 25:13              | 12:11               | 10:1                  |
| 1 Gb         | 26:14              | 13:11               | 10:1                  |
| 2 Gb         | 27:14              | 13:11               | 10:1                  |
| 4 Gb         | 28:14              | 13:11               | 10.1                  |

## LPDDR SDRAM Organization

The DMC supports LPDDR SDRAM memory modules ranging from 64M bit to 2G bit. The following tables list the address translation mechanism from the user interface to LPDDR memory interface.

The controller also supports two types of addressing modes: bank interleaving ( DMC\_CTL.ADDRMODE =1) and page interleaving ( DMC\_CTL.ADDRMODE =0).

## Bank Interleaving

The LPDDR Bank Interleaving table shows LPDDR bank interleaving.

Table 10-7: LPDDR Bank Interleaving

| SDRAM size   | Bank address bits   | Row address bits   | Column address bits   |
|--------------|---------------------|--------------------|-----------------------|
| 64 Mb        | 22:21               | 20:9               | 8:1                   |
| 128 Mb       | 23:22               | 21:10              | 9:1                   |
| 256 Mb       | 24:23               | 22:10              | 9:1                   |
| 512 Mb       | 25:24               | 23:11              | 10:1                  |
| 1 Gb         | 26:24               | 23:11              | 10:1                  |
| 2 Gb         | 27:26               | 25:12              | 11:1                  |

## Page Interleaving

The LPDDR Page Interleaving table shows LPDDR page interleaving.

Table 10-8: LPDDR Page Interleaving

| SDRAM size   | Row address bits   | Bank address bits   | Column address bits   |
|--------------|--------------------|---------------------|-----------------------|
| 64 Mb        | 22:11              | 10:9                | 8.1                   |
| 128 Mb       | 23:12              | 11:10               | 9:1                   |
| 256 Mb       | 24:12              | 11:10               | 9:1                   |
| 512 Mb       | 25:13              | 12:11               | 10:1                  |
| 1 Gb         | 26:14              | 13:11               | 10:1                  |

Table 10-8: LPDDR Page Interleaving (Continued)

| SDRAM size   | Row address bits   | Bank address bits   |   Column address bits |
|--------------|--------------------|---------------------|-----------------------|
| 2 Gb         | 27:14              | 13:11               |                  10.1 |

## DMC Clocking

The DMC uses a divided-down version of the PLLCLK (PLL clock) to generate an internal clock for clocking the DMC block and interface. The specific value of the DCLK frequency is programmed in the CGU\_DIV register. The section Initializing the DMC (ADSP-SC58x) describes the procedure.

For information on the maximum clock frequency supported for specific modes, refer to the processor data sheet.

NOTE: For the processor variants that have two DMC blocks, both blocks run on the same DCLK frequency.

NOTE: In some cases, it might be required to generate a DCLK frequency asynchronous to CCLK (for example, CCLK=450 MHz and DCLK=400 MHz). For these cases, one CGU can be used to generate CCLK and another can be used to generate DCLK. For more details, refer to the CGU chapter.

## DMC DMA

The DMC supports DMA-based transfers to and from external DDR SDRAM memory and internal memory.

The DMC DMA controller, part of the distributed DMA engines (DDE) that are dispersed through the infrastructure, connects to the system crossbar fabric.

The DMC uses two DDEs for memory-to-memory DMA (MDMA). One channel is the source channel, and the second, the destination channel.

DMA transfers on the processor are descriptor-based or register-based. Register-based DMA allows the processor to program DMA control registers directly to initiate a DMA transfer. On completion, the control registers can be automatically updated with their original setup values for continuous transfer, if needed. Descriptor-based DMA transfers require a set of parameters stored within memory to initiate a DMA sequence. This transfer allows the chaining together of multiple DMA sequences. In descriptor-based DMA operations, a DMA channel can be programmed to set up and start another DMA transfer automatically after the current sequence completes.

Refer to the DMA chapter for further details.

Enhanced DMA operations (such as delay line DMA, scatter or gather DMA) are also supported to or from the DMC module. For more details, refer to the Extended Memory DMA (EMDMA) chapter.

## DMC Operating Modes

By default, the DMC is in DDR2 mode. To enable DDR3 or LPDDR mode, the corresponding bits in the DMC\_CTL and DMC\_PHY\_CTL4 register must be configured.

## DDR2 Mode

This mode is the default mode of the DMC module and supports JESD79-2E compatible DDR2 SDRAM. In this mode, the DMC\_CTL.DDR3EN bit =0, the DMC\_CTL.LPDDR bit =0, and the DMC\_PHY\_CTL4.DDRMODE bit field is 0b'01.

## DDR3 Mode

The DMC module supports JESD79-3E compatible double data rate DDR3 SDRAM. To configure this mode of operation, first set (=1) the DMC\_CTL.DDR3EN bit and set the DMC\_PHY\_CTL4.DDRMODE bit field to 0b'00.

## LPDDR Mode

The DMC module supports JESD209A low-power DDR (LPDDR) SDRAM. To configure this mode of operation, set (=1) the DMC\_CTL.LPDDR bit and set the DMC\_PHY\_CTL4.DDRMODE bit field to 0b'11.

## Deep Power-Down Mode

The DMC module supports JESD209A low-power DDR (LPDDR) SDRAM. To configure this mode of operation, set (=1) the DMC\_CTL.LPDDR bit and set the DMC\_PHY\_CTL4.DDRMODE bit field to 0b'11.

When the processor does not require the data stored in SDRAM (assume reset state of SDRAM), the DMC can put the SDRAM in deep power-down mode. When the DMC is in deep power-down, any data accesses cause the DMC to generate a bus error. To configure this mode, set (=1) the DMC\_CTL.DPDREQ bit when low-power DMC operation is enabled ( DMC\_CTL.LPDDR =1).

The DMC\_STAT.IDLE bit indicates the activity in the DMC. If this bit is set, there is no activity all through the DMC. Deep power can be entered by setting the DMC\_CTL.DPDREQ bit. The DMC\_STAT.DPDACK bit is asserted when the SDRAM enters deep power-down mode. The DMC stays in deep power-down mode as long as the DMC\_CTL.DPDREQ bit is asserted.

Clearing (=0) the DMC\_CTL.DPDREQ bit causes the DMC to exit deep power-down mode. Also, when exiting deep power-down mode, the controller clears the DMC\_STAT.DPDACK bit. The user must re-initialize the DMC after it comes out of deep power-down mode.

## Self-Refresh Mode

For low-power consumption, the SDRAM can be put in self-refresh mode. When no data activity occurs, the DMC can put the SDRAM in self refresh to save power. The DMC\_STAT.IDLE bit indicates the activity on the DMC. If this bit is set, there is no activity in the DMC.

Enable self-refresh mode by writing the DMC\_CTL.SRREQ bit. The DMC stays in a self-refresh state as long as this bit is asserted. The DMC\_STAT.SRACK bit indicates when the SDRAM enters self-refresh mode.

When the DMC is in self-refresh mode, the DMC generates an SCB error when any data accesses (read or write requests) is requested.

The DMC can be brought out of self-refresh mode by clearing the DMC\_CTL.SRREQ bit again. The controller clears the DMC\_STAT.SRACK bit after the self-refresh operation completes.

## DMC Event Control

The DMC has no related interrupt or trigger event information.

## DMC Programming Model

The dynamic memory controller contains five groups of memory-mapped registers. The DMC uses the MMR access bus to connect to these registers.

- Control and status registers. These registers control the various operation modes of the dynamic memory controller and provide status.
- Timing parameter registers. The value programmed in these registers depends on the speed grade of the SDRAM device used.
- Mode register mirror registers. These shadow registers are copies of the mode registers residing in the SDRAM device.
- PHY control and status registers. The DMC uses these registers to control the operation of the PHY.
- PAD control registers. The DMC uses these registers to control the various aspects of the I/O pads.

The DMC control registers contain sensitive timing parameters and settings for the DDR SDRAM. These registers are programmed with values that are in the operating range of the DDR used.

Writing to reserved fields or writing any reserved values in register bits can cause the dynamic memory controller to function erroneously.

## Programming Considerations for DDR2, DDR3, and LPDDR Memory

The DDR2, DDR3, and LPDDR Programming table shows important programming considerations and differences across DDR2, DDR3, and LPDDR memory technologies. The table serves as a quick reference when configuring the DMC and PHY registers.

Table 10-9: DDR2, DDR3, and LPDDR Programming

| PHY/Control- ler   | Description                     | Registers and Bit Fields Involved   | DDR3                                                                   | DDR2                                                                   | LPDDR                                                                   |
|--------------------|---------------------------------|-------------------------------------|------------------------------------------------------------------------|------------------------------------------------------------------------|-------------------------------------------------------------------------|
| PHY                | Enabling DDR3/ DDR2/LPDDR modes | DMC_PHY_CTL4                        | Select DDR3 mode by setting the DMC_PHY_CTL4. DDRMODE bit field to 00. | Select DDR2 mode by setting the DMC_PHY_CTL4 .DDRMODE bit field to 01. | Select LPDDR mode by setting the DMC_PHY_CTL4. DDRMODE bit field to 11. |

Table 10-9: DDR2, DDR3, and LPDDR Programming (Continued)

| PHY/Control- ler   | Description                                         | Registers and Bit Fields Involved   | DDR3                                                                                                                                                                                                                                                                                                                                                                                     | DDR2                                                                                                                                                                                                                                                                                                                                                                                     | LPDDR                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|-----------------------------------------------------|-------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| PHY                | ODT and drive impe- dance calibration               | DMC_CAL_ PADCTL0 , DMC_CAL_ PADCTL2 | The DMCsupports ODT and drive impe- dance calibration. Configure the DMC_CAL_ PADCTL0 and DMC_CAL_ PADCTL2 registers accordingly. For de- tails, refer PAD Cali- bration for Driver Im- pedance and On Die Termination (ODT) section. Programming driver impedance is required. Programming ODT is optional. To disable ODT, set the DMC_PHY_CTL1. BYPODTEN bit.                         | The DMCsupports OCD calibration. Configure the DMC_CAL_ PADCTL0 register ac- cordingly.                                                                                                                                                                                                                                                                                                  | The DMCdoes not support ODT and drive impedance cali- bration. No need to program the DMC_CAL_ PADCTL0 and DMC_CAL_ PADCTL2 registers. Must make sure that the DMC_PHY_CTL1. BYPODTEN bit is set to bypass the process- or ODT settings.                                                                                                                                                 |
| Controller         | Enabling DDR3/ DDR2/LPDDR modes                     | DMC_CTL.DDR3EN , DMC_CTL.LPDDR      | Select DDR3 mode by setting the DMC_CTL.DDR3EN bit. Make sure that the bit DMC_CTL.LPDDR is cleared.                                                                                                                                                                                                                                                                                     | Default is DDR2 mode. Make sure that both the bits DMC_CTL.LPDDR and DMC_CTL.DDR3EN are cleared.                                                                                                                                                                                                                                                                                         | Select LPDDR mode by setting the DMC_CTL.LPDDR bit. Make sure that the bit DMC_CTL.DDR3EN is cleared.                                                                                                                                                                                                                                                                                    |
| Controller         | Configuring DMC_CTL.RDTOWR bit field                | DMC_CTL.RDTOWR                      | Make sure that the DMC_CTL.RDTOWR bit field is always set to 010 (=2 in decimal).                                                                                                                                                                                                                                                                                                        | Make sure that the DMC_CTL.RDTOWR bit field is always set to 010 (=2 in decimal).                                                                                                                                                                                                                                                                                                        | Make sure that the DMC_CTL.RDTOWR bit field is always set to 010 (=2 in decimal).                                                                                                                                                                                                                                                                                                        |
| Controller         | Configuring DMC_CFG register fields                 | DMC_CFG                             | Make sure that the DMC_CFG.IFWID and DMC_CFG.SDRWID bit fields are always set to 0010 (=2 in decimal) as the DMConly supports 16-bit wide interface and SDRAM widths. Make sure that the bit field DMC_CFG.EXTBANK is always set to 0000 as the DMConly sup- ports one external bank. Select the DMC_CFG.SDRSIZE as per the SDRAM size. Supported sizes are 64 Mb to 2 Gb for LPDDR, 256 | Make sure that the DMC_CFG.IFWID and DMC_CFG.SDRWID bit fields are always set to 0010 (=2 in decimal) as the DMConly supports 16-bit wide interface and SDRAM widths. Make sure that the bit field DMC_CFG.EXTBANK is always set to 0000 as the DMConly sup- ports one external bank. Select the DMC_CFG.SDRSIZE as per the SDRAM size. Supported sizes are 64 Mb to 2 Gb for LPDDR, 256 | Make sure that the DMC_CFG.IFWID and DMC_CFG.SDRWID bit fields are always set to 0010 (=2 in decimal) as the DMConly supports 16-bit wide interface and SDRAM widths. Make sure that the bit field DMC_CFG.EXTBANK is always set to 0000 as the DMConly sup- ports one external bank. Select the DMC_CFG.SDRSIZE as per the SDRAM size. Supported sizes are 64 Mb to 2 Gb for LPDDR, 256 |
| Controller         | Configuring controller timing parameter reg- isters | DMC_TR0 , DMC_TR1 , DMC_TR2         | Configure the parameters t RCD , t WTR , t RP , t RAS , t MRD , t REF , t RFC , t RRD , t WR , t XP , t CKE (DDR3/DDR2/LPDDR), and t FAW , t RTP (DDR3/DDR2) in terms of DCLK cycles.                                                                                                                                                                                                    | Configure the parameters t RCD , t WTR , t RP , t RAS , t MRD , t REF , t RFC , t RRD , t WR , t XP , t CKE (DDR3/DDR2/LPDDR), and t FAW , t RTP (DDR3/DDR2) in terms of DCLK cycles.                                                                                                                                                                                                    | Configure the parameters t RCD , t WTR , t RP , t RAS , t MRD , t REF , t RFC , t RRD , t WR , t XP , t CKE (DDR3/DDR2/LPDDR), and t FAW , t RTP (DDR3/DDR2) in terms of DCLK cycles.                                                                                                                                                                                                    |

Table 10-9: DDR2, DDR3, and LPDDR Programming (Continued)

| PHY/Control- ler   | Description                                                              | Registers and Bit Fields Involved                                      | DDR3                                                                                                                                                                                                                                                                                    | DDR2                                                                                                                                                                                                                                                                    | LPDDR                                                                                                                                                                                                                                                                                           |
|--------------------|--------------------------------------------------------------------------|------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Controller         | Configuring burst length                                                 | DMC_MR.BLEN (for DDR2 and LPDDR), DMC_MR0.BLEN for DDR3                | The DMConly sup- ports burst length of 8. Configure the DMC_MR0.BLEN field to 00 only.                                                                                                                                                                                                  | The DMCsupports both burst length of 4 and 8. Configure DMC_MR.BLEN field to 10 for burst length of 4 and to 11 for burst length of 8 words.                                                                                                                            | The DMCsupports both burst length of 4 and 8. Configure DMC_MR.BLEN field to 10 for burst length of 4 and to 11 for burst length of 8 words.                                                                                                                                                    |
| Controller         | Configuring CAS la- tency                                                | DMC_MR.CL (for DDR2 and LPDDR), DMC_MR0.CL0, and DMC_MR0.CL (for DDR3) | The DMCsupports CAS latencies of 5 to14. Refer to the DMC_MR.CL register description for more details.                                                                                                                                                                                  | The DMCsupports CAS latencies of 3 to 6. Refer to the DMC_MR register de- scription for more de- tails.                                                                                                                                                                 | The DMCsupports CAS latency of 3. Re- fer to the DMC_MR register description for more details.                                                                                                                                                                                                  |
| Controller         | Configuring the DMC_MR.DLLRST bit (DDR2/LPDDR) or DMC_MR0(DDR3)          | DMC_MR.DLLRST / DMC_MR0.DLLRS T                                        | Set this bit while per- forming the initializa- tion                                                                                                                                                                                                                                    | Setting of this bit is optional for DDR2.                                                                                                                                                                                                                               | Reserved                                                                                                                                                                                                                                                                                        |
| Controller         | Write recovery timing                                                    | DMC_MR.WRRECOV and DMC_MR0.WRRE- COV                                   | The DMCsupports WRvalues of 5 to 16. Refer to the DMC_MR0 register description for more details.                                                                                                                                                                                        | The DMCsupports WRvalues of 2 to 8. Refer to the DMC_MR register description for more details.                                                                                                                                                                          | Reserved                                                                                                                                                                                                                                                                                        |
| Controller         | Configuring DMC_MR1(DDR3) or DMC_EMR1 (DDR2) register                    | DMC_MR1(DDR3) or DMC_EMR1 (DDR2)                                       | The DMCcan use this register to configure memory drive impedance, ODT, and additive latency parameters. Refer to the corresponding register descriptions for                                                                                                                            | more details.                                                                                                                                                                                                                                                           | This register is not used for LPDDR pro- gramming.                                                                                                                                                                                                                                              |
| Controller         | Configuring DMC_MR2(DDR3) or DMC_EMR2 (DDR2) or DMC_EMR (LPDDR) register | DMC_MR2(DDR3) or DMC_EMR2 (DDR2) or DMC_EMR (LPDDR)                    | Configure the write latency field (CWL) to the required value. The DMCcan also use this register to en- able Auto Self-Refresh (ASR) and to select Self-Refresh Tempera- ture (SRT) functional- ities in the memory device. For more de- tails on these func- tionalities, refer to the | The DMCcan use this register to config- ure the Partial Array Self-Refresh (PASR) and High Tempera- ture Self-Refresh Rate Enable (SRF) func- tionalities of DDR2 memory device. For more details on these functionalities, refer to the DDR2 memory device data sheet. | The DMCcan use this register to config- ure the Partial Array Self-Refresh (PASR), Temperature compen- sated self-refresh (TCSR), and drive strength (DS) func- tionalities of the mem- ory device. For more details on these func- tionalities, refer to the LPDDR memory de- vice data sheet. |

Table 10-9: DDR2, DDR3, and LPDDR Programming (Continued)

| PHY/Control- ler   | Description                           | Registers and Bit Fields Involved   | DDR3                                                                                           | DDR2                                                                                           | LPDDR                                                                                          |
|--------------------|---------------------------------------|-------------------------------------|------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------|
|                    |                                       |                                     | DDR3 memory device data sheets.                                                                |                                                                                                |                                                                                                |
| Controller         | Configuring the DMC_DLLCTL regis- ter | DMC_DLLCTL                          | Always configure the DMC_DLLCTL.DLLCALRDCNT field to 0x48 and DMC_DLLCTL.DATACYC field to 0x9. | Always configure the DMC_DLLCTL.DLLCALRDCNT field to 0x48 and DMC_DLLCTL.DATACYC field to 0x9. | Always configure the DMC_DLLCTL.DLLCALRDCNT field to 0x48 and DMC_DLLCTL.DATACYC field to 0x9. |

## PHY DLL Calibration

The PHY DLL calibration is performed as part of the SDRAM power-up initialization. It calibrates data against the DQS and CLK signal. However, running DLL calibration after self-refresh or at an arbitrary time is required in certain cases.

The DMC allows PHY DLL calibration to start by setting the DMC\_CTL.DLLCAL bit. The DMC\_STAT.DLLCALDONE bit can be used to monitor the progress of the calibration. Once calibration is over, this bit is set. Once the calibration procedure is started by writing to the DMC\_CAL\_PADCTL0.CALSTRT bit, the full calibration takes 300 DCLK cycles to complete.

NOTE: DLL calibration can be initiated only when the DMC is idle ( DMC\_STAT.IDLE = 1).

## DDR2 OCD Calibration

OCD calibration is not supported by the ADSP-SC58x processors.

## DDR3 ZQ Calibration Short CMD

The ZQ calibration short command is generally used to correct small variations in ZQ (approximately 0.5%). To perform ZQ calibration, the controller is first checked for its idle state. Once the idle bit is obtained, a ZQCS command can be issued by setting the DMC\_STAT.ZQCSDONE bit (0x0004). The DMC\_STAT.ZQCSDONE bit (0x0008) can be used to monitor the calibration sequence. When this bit is '0', it indicates that the calibration is ongoing. When it is '1', it indicates that the calibration is done. As an example, a GP timer can be used to periodically trigger a ZQCS command to address tiny variations.

NOTE: The ZQ calibration function is essential for normal operation of DDR3. With the reference of the external resistance (240 Ω ± 1%) connected to the DMC\_RZQ pin, DDR3 calibrates the Ron and Rtt values of the ZQ pin against temperature and voltage variations.

## DDR3 ZQ Calibration Long CMD

Several DDR3 impedance calibrations are implemented for optimal signal integrity. The long ZQ calibration is used after power-up and the short ZQ calibration is used periodically during normal operation to compensate for voltage and temperature drift. These calibration sequences improve connectivity between the SDRAM pads and the PCB trace. The DMC\_RZQ pin on the SDRAM is connected to an external precision resistor that adjusts the output

driver impedance Rtt and ODT values to match the trace impedance. The connection reduces impedance discontinuity and minimizes signal reflections.

The command has two variants named as ZQ calibration long (ZQCL) and ZQ calibration short (ZQCS). The ZQCL command is issued during initialization and after self-refresh exit command. It can be issued later depending on the system environment.

The DMC pads can be autocalibrated to the required driver impedance Rtt using an external resistance RZQ and the On Die Termination (ODT) value using the corresponding bits ( DMC\_CAL\_PADCTL2 ). The autocalibration logic translates these values into a corresponding drive strength control inside the PHY and then routed to the PADS. Autocalibration starts as soon as the DMC\_CAL\_PADCTL0.CALSTRT bit is programmed (set the DCLK at the required frequency before setting this bit). Autocalibration expects the program to select two different member sets of pads (address/command pads versus CLK/Data/DQS/DM pads).

## On Die Termination (DDR2/DDR3)

The DMC supports dynamic On Die Termination (ODT) at the pads. When the controller ODT is set, the termination resistors in the pads are turned on when the controller reads data from the DRAM. These resistors are turned off when the controller is writing to the DRAM. Controller ODT is enabled with the granularity of a byte lane. The description of this feature can be obtained in the description of the corresponding PHY registers.

ODT resistance Rtt is selectable in the same way as DDR2 SDRAM ([A9, A6, A2] in MR1, [A10, A9] in MR2)

DDR3 SDRAM inherits the ODT function provided for DDR2 SDRAM, and provides extended ODT mode.

Synchronous ODT : ODT timing is the same as that of DDR2 SDRAM

Asynchronous ODT : ODT timing in the slow exit power-down mode

Dynamic ODT : Function that can dynamically switch the ODT resistance during a write operation without an MRS command. It improves signal quality during a write operation.

## Output Driver Impedance

Output driver impedance (Rtt) of DQ, DQS, /DQS/DM is selectable in the same way as DDR2 SDRAM ([A5, A1] in MR1). Rtt can fluctuate with the process, voltage, and temperature (PVT). DDR2 SDRAM can calibrate Rtt fluctuation due to PVT using the optional OCD (off-chip driver calibration) function. DDR3 SDRAM uses the ZQ calibration function instead of the OCD function.

In addition to the driver impedance, the bidirectional pads (Data and DQS) also require the initialization sequence to program the termination impedance by writing to the field DMC\_CAL\_PADCTL2.IMPRTT . The DMC pads use parallel termination, one branch goes from the pad to the I/O supply. The other branch goes to the I/O ground. The value programmed to this 8-bit field is the value to be used for each branch. There is a correction factor involved while programming this register. The value of this correction factor is 0.8. For example, suppose that a termination of 50 Ω is required on the data pads to match with the board trace. The value is programmed to 100 × 0.8 = 80, as the two parallel paths lead to an effective impedance of 50 ohms.

## Initializing the DMC (ADSP-SC58x)

To initialize the DMC, use the following steps. If it is not the first time that the DMC initializes, check to first ensure that the DMC is idle and not in the midst of any activity.

If DMC initialization occurs for the first time after power-up, PHY and PAD initialization is a requirement. The initialization occurs with the following steps:

For LPDDR mode, set the DMC\_PHY\_CTL4.DDRMODE bits to 0b'11 (3 in decimal) and set the DMC\_PHY\_CTL1.BYPODTEN bit.

For DDR2/DDR3 modes, follow these steps to perform pad impedance calibration:

1. Set the device mode in the DMC\_PHY\_CTL4.DDRMODE to DDR2 or DD3.
2. Configure the required values in the DMC\_CAL\_PADCTL0 and DMC\_CAL\_PADCTL2 registers without setting the DMC\_CAL\_PADCTL0.CALSTRT bit.
3. Set the DMC\_CAL\_PADCTL0.CALSTRT bit.
4. Wait for 300 DCLK cycles for the PAD calibration to complete.

Bits 0, 1, 2, 3 of the DMC\_PHY\_CTL0 register and the bits 31 through 26 of the DMC\_PHY\_CTL2 register are timing trim bits. Always set these bits for DDR2 and DDR3 modes. For example, set these bits during firsttime DMC initialization. Then, software does not need to touch or clear these bits.

Bits 0, 1, 2, 3 of the DMC\_PHY\_CTL0 register and bits 26, 27, 28, 29, 30, and 31 of the DMC\_PHY\_CTL2 register are timing trim bits. Always set these bits for DDR2 and DDR3 modes. For example, set these bits during first-time DMC initialization. Then, software does not need to touch or clear these bits.

Use the following C code to set these bits for the first time DMC initialization:

```
*pREG_DMC0_PHY_CTL0 |= 0x0000000F; *pREG_DMC0_PHY_CTL2 |= 0xFC000000;
```

NOTE: Bits 6, 7, 25 and 27 of the DMC\_PHY\_CTL3 register should always be set for all the DDR modes (DDR3/DDR2/LPDDR). The program can set these bits during first-time DMC initialization. Then, software need not touch or clear these bits. Use the following C code to set these bits for the first time DMC initialization:

```
*pREG_DMC0_PHY_CTL3=0xA0000C0;
```

NOTE: For DDR3 mode, set bit 1 and configure bits [5:2] of the DMC\_CPHY\_CTL register with WL = CWL + AL in DCLK cycles. For example, in case of DMC0, if CWL =6 and AL =0, program the DMC\_CPHY\_CTL register with the value 0x0000001A.

Use the following steps for first-time DMC initialization and reinitialization:

1. Perform first-time DMC initialization, as needed.
2. ADDITIONAL INFORMATION: Perform this step only for the first time DMC initialization after power-up or reset. Skip this step if reinitializing the DMC.
- a. Set the DMC\_PHY\_CTL0.RESETDLL bit of the DMC\_PHY\_CTL0 register.
- b. Initialize the CGU to change the DCLK frequency.
- a. Clear the DMC\_PHY\_CTL0.RESETDLL bit of the DMC\_PHY\_CTL0 register.
2. Reinitialize the DMC with a DCLK change
- a. Place the DMC in self-refresh mode.
- b. Set the DMC\_PHY\_CTL0.RESETDLL bit of the DMC\_PHY\_CTL0 register.
- c. Initialize the CGU to change the DCLK frequency.
- d. Clear the DMC\_PHY\_CTL0.RESETDLL bit of the DMC\_PHY\_CTL0 register.
- e. Bring the DMC out of self-refresh mode.
3. If not already done, wait 9000 DCLK cycles to ensure that the DLL locked.
4. Program the DMC\_CFG , DMC\_CTL , DMC\_TR0 , DMC\_TR1 , and DMC\_TR2 registers to the appropriate values to set proper SDRAM cycle timing options.
14. ADDITIONAL INFORMATION: For example, t RAS , t RC, t RP , t RCD, t WR , t FAW are some of the parameters.
5. Program the shadow registers DMC\_MR (DDR2/DDR3/LPDDR), DMC\_EMR1 (DDR2)/ DMC\_EMR1 (DDR3), DMC\_EMR1 (DDR2)/ DMC\_EMR1 (LPDDR) DMC\_EMR1 (DDR3), with the needed burst length, CAS latency, additive latency, and other parameters.
6. Finally, after programming these registers, write the DMC\_CTL.INIT bit to the DMC control register to begin the power-up initialization sequence.
7. Wait for the SDRAM initialization sequence to complete by making sure that the DMC\_STAT.INITDONE bit is set.

Figure 10-1: DMC Initialization Flow

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000000_d0ff795174b0b008764f64a89e93f80970f0519295c2b23744a94c735348818f.png)

The DMC accumulates system crossbar transactions that occur during or before initialization and sends them to SDRAM once the SDRAM initialization or DLL calibration is complete.

NOTE: During the DMC PHY DLL calibration, a particular set of locations in the DRAM is written followed immediately by a series of reads. The DMC PHY needs information about the data to be read during the PHY DLL calibration prior to the operation. The controller performs one burst write operation to the address programmed in the DMC\_DT\_CALIB\_ADDR register. The exact address chosen does not matter during memory initialization.

If calibration of the PHY is performed when the DRAM contains valid data, ensure that this address points to an unused address. Otherwise, this operation modifies application data stored at the address selected. The DLL calibration modifies all 16 bytes corresponding to the 16-byte aligned address, even if the

address programmed is not 16-byte aligned. For example, it updates all the locations 0x80000000 to 0x8000000F regardless of whether the address is programmed as 0x80000000 or 0x80000004.

The program performs second-time initialization for cases where the DMC has already been initialized. The initialization can be through a preload during a debug session, or through code executed during the booting process.

## ADSP-SC58x DMC Register Descriptions

Dynamic Memory Controller (DMC) contains the following registers.

Table 10-10: ADSP-SC58x DMC Register List

| Name                    | Description                                                |
|-------------------------|------------------------------------------------------------|
| DMC_CFG                 | Configuration Register                                     |
| DMC_CPHY_CTL            | Controller to PHY Interface Register                       |
| DMC_CTL                 | Control Register                                           |
| DMC_DLLCTL              | DLL Control Register                                       |
| DMC_DT_CALIB_ADDR       | Data Calibration Address Register                          |
| DMC_DT_DATA_CALIB_DATA0 | Data Calibration Data 0 Register                           |
| DMC_DT_DATA_CALIB_DATA1 | Data Calibration Data 1 Register                           |
| DMC_EFFCTL              | Efficiency Control Register                                |
| DMC_EMR1                | Shadow EMR1 DDR2 Register                                  |
| DMC_EMR2                | Shadow EMR2 Register (DDR2)/Shadow EMR Register (LPDDR)    |
| DMC_MR                  | Shadow MRRegister (DDR2/LPDDR), Shadow MR0 Register (DDR3) |
| DMC_MR1                 | Shadow MR1 Register (DDR3)                                 |
| DMC_MR2                 | Shadow MR2 Register (DDR3)                                 |
| DMC_MSK                 | Mask (Mode Register Shadow) Register                       |
| DMC_PRIO                | Priority ID Register 1                                     |
| DMC_PRIO2               | Priority ID Register 2                                     |
| DMC_PRIOMSK             | Priority ID Mask Register 1                                |
| DMC_PRIOMSK2            | Priority ID Mask Register 2                                |
| DMC_RDDATABUFID1        | DMCRead Data Buffer ID Register 1                          |
| DMC_RDDATABUFID2        | DMCRead Data Buffer ID Register 2                          |
| DMC_RDDATABUFMSK1       | DMCRead Data Buffer Mask Register 1                        |
| DMC_RDDATABUFMSK2       | DMCRead Data Buffer Mask Register 2                        |
| DMC_STAT                | Status Register                                            |
| DMC_TR0                 | Timing 0 Register                                          |

Table 10-10: ADSP-SC58x DMC Register List (Continued)

| Name    | Description       |
|---------|-------------------|
| DMC_TR1 | Timing 1 Register |
| DMC_TR2 | Timing 2 Register |

## Configuration Register

The DMC\_CFG register selects SDRAM device specific parameters and selects the SDRAM interface width.

Figure 10-2: DMC\_CFG Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000001_7d467201193271a90e8a1fd8d87a26d0e8ee66b0db646285ae700927c9e4f04c.png)

Table 10-11: DMC\_CFG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                      | Description/Enumeration                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:12 (R/W)        | EXTBANK    | External Banks. The DMC_CFG.EXTBANK bits select the number of external banks connected to the DMC. Note that all values other than those shown are reserved. | External Banks. The DMC_CFG.EXTBANK bits select the number of external banks connected to the DMC. Note that all values other than those shown are reserved. |
|                    |            | 0                                                                                                                                                            | 1 External Bank                                                                                                                                              |
|                    |            | 1-15                                                                                                                                                         | Reserved                                                                                                                                                     |
| 11:8 (R/W)         | SDRSIZE    | SDRAM Size. The DMC_CFG.SDRSIZE bits select the size of individual SDRAM connected to the DMC. Note that all values other than those shown are reserved.     | SDRAM Size. The DMC_CFG.SDRSIZE bits select the size of individual SDRAM connected to the DMC. Note that all values other than those shown are reserved.     |
|                    |            | 0                                                                                                                                                            | 64M Bit SDRAM (LPDDR Only)                                                                                                                                   |
|                    |            | 1                                                                                                                                                            | 128M Bit SDRAM (LPDDR Only)                                                                                                                                  |
|                    |            | 2                                                                                                                                                            | 256M Bit SDRAM                                                                                                                                               |
|                    |            | 3                                                                                                                                                            | 512M Bit SDRAM                                                                                                                                               |
|                    |            | 4                                                                                                                                                            | 1G Bit SDRAM                                                                                                                                                 |
|                    |            | 5                                                                                                                                                            | 2G Bit SDRAM                                                                                                                                                 |
|                    |            | 6                                                                                                                                                            | 4G Bit SDRAM                                                                                                                                                 |
|                    |            | 7                                                                                                                                                            | 8G Bit SDRAM                                                                                                                                                 |

Table 10-11: DMC\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                       | Description/Enumeration                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:4 (R/W)          | SDRWID     | SDRAM Width. The DMC_CFG.SDRWID bits select the width of the individual SDRAM connected to the DMC. Note that all values other than those shown are reserved. | SDRAM Width. The DMC_CFG.SDRWID bits select the width of the individual SDRAM connected to the DMC. Note that all values other than those shown are reserved. |
| 7:4 (R/W)          | SDRWID     | 0-1                                                                                                                                                           | Reserved                                                                                                                                                      |
| 7:4 (R/W)          | SDRWID     | 2                                                                                                                                                             | 16-Bit Wide SDRAM                                                                                                                                             |
| 7:4 (R/W)          | SDRWID     | 3-15                                                                                                                                                          | Reserved                                                                                                                                                      |
| 3:0 (R/W)          | IFWID      | Interface Width. The DMC_CFG.IFWID bits select the width of the interface between the DMCand                                                                  | Interface Width. The DMC_CFG.IFWID bits select the width of the interface between the DMCand                                                                  |
| 3:0 (R/W)          | IFWID      | 0-1                                                                                                                                                           | Reserved                                                                                                                                                      |
| 3:0 (R/W)          | IFWID      | 2                                                                                                                                                             | 16-Bit Wide Interface. All other values are reserved. This field specifies the interface width between the con- troller and the SDRAM.                        |
| 3:0 (R/W)          | IFWID      | 3-15                                                                                                                                                          | Reserved                                                                                                                                                      |

## Controller to PHY Interface Register

Figure 10-3: DMC\_CPHY\_CTL Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000002_f9473138f42027beebb3636aafc8908b8beb42e24f22df62b07e87c1520934e3.png)

Table 10-12: DMC\_CPHY\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                |
|--------------------|------------|----------------------------------------|
| 31:0               | CPHY_CTL   | Register given from Controller to PHY. |

## Control Register

The DMC\_CTL register controls DMC modes, DLL calibration, and DRAM initialization.

Figure 10-4: DMC\_CTL Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000003_b02e6fabc165402e93c89ccc4024d538a5b5980d345e777b068accaab0af031b.png)

Table 10-13: DMC\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                    | Description/Enumeration                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------|
| 25 (R0/W)          | ZQCL       | ZQ Calibration Long. The DMC_CTL.ZQCL bit starts the ZQ calibration long sequence. Note that this bit always reads as 0.   | ZQ Calibration Long. The DMC_CTL.ZQCL bit starts the ZQ calibration long sequence. Note that this bit always reads as 0.   |
| 25 (R0/W)          | ZQCL       | 0                                                                                                                          | No effect                                                                                                                  |
| 25 (R0/W)          | ZQCL       | 1                                                                                                                          | Triggers ZQ calibration long sequence                                                                                      |
| 24 (R0/W)          | ZQCS       | ZQ Calibration Short. The DMC_CTL.ZQCS bit starts the ZQ calibration short sequence. Note that this bit always reads as 0. | ZQ Calibration Short. The DMC_CTL.ZQCS bit starts the ZQ calibration short sequence. Note that this bit always reads as 0. |
| 24 (R0/W)          | ZQCS       | 0                                                                                                                          | No effect                                                                                                                  |
| 24 (R0/W)          | ZQCS       | 1                                                                                                                          | Triggers ZQ calibration short sequence                                                                                     |

Table 10-13: DMC\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                            | Description/Enumeration                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R0/W)          | DLLCAL     | DLL Calibration Start. The DMC_CTL.DLLCAL bit starts the PHY DLL calibration sequence. Note that this bit always reads as 0.                                                                                       | DLL Calibration Start. The DMC_CTL.DLLCAL bit starts the PHY DLL calibration sequence. Note that this bit always reads as 0.                                                                                       |
| 13 (R0/W)          | DLLCAL     | 0                                                                                                                                                                                                                  | No effect                                                                                                                                                                                                          |
| 13 (R0/W)          | DLLCAL     | 1                                                                                                                                                                                                                  | Start PHY DLL calibration                                                                                                                                                                                          |
| 12 (R/W)           | PPREF      | Postpone Refresh. The DMC_CTL.PPREF bit enables postponing the DMCs sending of auto-refresh commands. When enabled, the DMCaccumulates refresh commands. The field selects the number of refresh commands that the | Postpone Refresh. The DMC_CTL.PPREF bit enables postponing the DMCs sending of auto-refresh commands. When enabled, the DMCaccumulates refresh commands. The field selects the number of refresh commands that the |
| 12 (R/W)           | PPREF      | 0                                                                                                                                                                                                                  | Disable Postpone Refresh                                                                                                                                                                                           |
| 11:9 (R/W)         | RDTOWR     | Read-to-Write Cycle. The DMC_CTL.RDTOWR bits select the number of cycles that the DMCadds when a write operation follows a read operation. For proper operation, it should be program- med with the value of 010.  | Read-to-Write Cycle. The DMC_CTL.RDTOWR bits select the number of cycles that the DMCadds when a write operation follows a read operation. For proper operation, it should be program- med with the value of 010.  |
| 11:9 (R/W)         | RDTOWR     | 0                                                                                                                                                                                                                  | 1 Cycle Added from JEDEC Spec Value                                                                                                                                                                                |
| 11:9 (R/W)         | RDTOWR     | 1                                                                                                                                                                                                                  | 2 Cycles Added from JEDEC Spec Value                                                                                                                                                                               |
| 11:9 (R/W)         | RDTOWR     | 2                                                                                                                                                                                                                  | 3 Cycles Added from JEDEC Spec Value                                                                                                                                                                               |
| 11:9 (R/W)         | RDTOWR     | 3                                                                                                                                                                                                                  | 4 Cycles Added from JEDEC Spec Value                                                                                                                                                                               |
| 11:9 (R/W)         | RDTOWR     | 4                                                                                                                                                                                                                  | 5 Cycles Added from JEDEC Spec Value                                                                                                                                                                               |
| 11:9 (R/W)         | RDTOWR     | 5                                                                                                                                                                                                                  | 6 Cycles Added from JEDEC Spec Value                                                                                                                                                                               |
| 11:9 (R/W)         | RDTOWR     | 6                                                                                                                                                                                                                  | 7 Cycles Added from JEDEC Spec Value                                                                                                                                                                               |
| 11:9 (R/W)         | RDTOWR     | 7                                                                                                                                                                                                                  | 8 Cycles Added from JEDEC Spec Value                                                                                                                                                                               |

Table 10-13: DMC\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | ADDRMODE   | Addressing (Page/Bank) Mode. The DMC_CTL.ADDRMODE bit selects whether the DMCuses page or bank inter- leaving for addressing. When using page interleaving, the bank address bits follow the most significant column address bits. When using bank interleaving, the bank address bits follow the most significant row address bits. |
| 8 (R/W)            | ADDRMODE   | 0 Bank Interleaving                                                                                                                                                                                                                                                                                                                  |
| 8 (R/W)            | ADDRMODE   | 1 Page Interleaving                                                                                                                                                                                                                                                                                                                  |
| 7 (R/W)            | RESET      | Reset SDRAM. The DMC_CTL.RESET bit starts the reset sequence. Note that this bit always reads as 0.                                                                                                                                                                                                                                  |
| 7 (R/W)            | RESET      | 0 No effect                                                                                                                                                                                                                                                                                                                          |
| 7 (R/W)            | RESET      | 1 Starts reset sequence                                                                                                                                                                                                                                                                                                              |
| 6 (R/W)            | PREC       | Precharge.                                                                                                                                                                                                                                                                                                                           |
| 6 (R/W)            | PREC       | 0 No Effect                                                                                                                                                                                                                                                                                                                          |
| 6 (R/W)            | PREC       | 1 Enable Precharge                                                                                                                                                                                                                                                                                                                   |
| 5 (R/W)            | DPDREQ     | Deep Power-Down Request. The DMC_CTL.DPDREQ bit enables deep power-down mode if low power DMCop-                                                                                                                                                                                                                                     |
| 5 (R/W)            | DPDREQ     | 0 Disable Deep Power-Down                                                                                                                                                                                                                                                                                                            |
| 5 (R/W)            | DPDREQ     | 1 Enable Deep Power-Down                                                                                                                                                                                                                                                                                                             |
| 4 (R/W)            | PDREQ      | Power Down Request. The DMC_CTL.PDREQ bit enables power-down mode. When the DMCis in power- down mode, any data accesses cause the DMCto generate a bus error. The DRAM remains in power-down mode as along as this bit is 1.                                                                                                        |
| 4 (R/W)            | PDREQ      | 0 Disable Power-Down                                                                                                                                                                                                                                                                                                                 |
| 4 (R/W)            | PDREQ      | 1 Enable Power-Down                                                                                                                                                                                                                                                                                                                  |

Table 10-13: DMC\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | SRREQ      | Self-Refresh Request. The DMC_CTL.SRREQ bit enables self-refresh mode. When the DMCis in self-re- fresh mode, any data accesses cause the DMCto generate a bus error. The DRAM re- mains in self-refresh mode as along as this bit is 1. |
| 2 (R0/W)           | INIT       | Initialize DRAM Start. The DMC_CTL.INIT bit starts the power up DRAM initialization sequence and DLL calibration sequence. Note that this bit always reads as 0.                                                                         |
| 1 (R/W)            | LPDDR      | Low Power DDR Mode. The DMC_CTL.LPDDR bit selects whether the DMCoperates in low power DDR mode or DDR2 mode.                                                                                                                            |
| 0 (R/W)            | DDR3       | Mode. The DMC_CTL.DDR3EN bit selects whether the DMCoperates in DDR3 mode or DDR2 mode.                                                                                                                                                  |
|                    | DDR3EN     | 0 Enable DDR2 mode                                                                                                                                                                                                                       |

## DLL Control Register

The DMC\_DLLCTL register holds the programmable parameters associated with the DLLs within the DMC PHY.

Figure 10-5: DMC\_DLLCTL Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000004_074e32c5801b9b3ec9dbd7318197d3388d41f9f8180413608c26494acdc87e98.png)

Table 10-14: DMC\_DLLCTL Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                             |
|--------------------|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:8 (R/W)         | DATACYC     | Data Cycles. The DMC_DLLCTL.DATACYC bits select the latency after which the DMCreads da- ta from the PHY. This field must be written with the value (9). All other values are reserved. Taking round trip delay into account, the DLL indicates whether a latency of 2 cycles is supported by means of status bits. |
| 7:0 (R/W)          | DLLCALRDCNT | DLL Calibration RD Count. The DMC_DLLCTL.DLLCALRDCNT field selects the number of read operations that the PHY uses for DLL calibration.                                                                                                                                                                             |

## Data Calibration Address Register

The DMC\_DT\_CALIB\_ADDR register provides the address used for the data calibration for read and write. During the DMC PHY DLL calibration, a particular set of locations in the DRAM is written and a series of reads are performed back to back to calibrate the PHY. The DMC PHY needs prior information about the data that would be read during the PHY DLL calibration. The controller performs one burst write operation to the address programmed in DMC\_DT\_CALIB\_ADDR (0x0090).

Note: While the exact address chosen does not matter much during memory initialization, if calibration of the PHY is performed when the DRAM contains valid data, care needs to be taken to ensure that this address points to an unused address. Else, this operation will modify application data stored at the address selected.

Figure 10-6: DMC\_DT\_CALIB\_ADDR Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000005_e1a384ea6da982b1b953cb747e7dd47e5334d623790a01196aeb6f78c6988e1b.png)

Table 10-15: DMC\_DT\_CALIB\_ADDR Register Fields

| Bit No. (Access)   | Bit Name          | Description/Enumeration                                                                                                                                          |
|--------------------|-------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | DMC_DT_CALIB_ADDR | Data calibration address. The DMC_DT_CALIB_ADDR.DMC_DT_CALIB_ADDR bit field contains the ad- dress to be programmed for the data calibration for read and write. |

## Data Calibration Data 0 Register

The DMC\_DT\_DATA\_CALIB\_DATA0 register contains the first 32-bit data used for the write during the data calibration.

Figure 10-7: DMC\_DT\_DATA\_CALIB\_DATA0 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000006_b3c6113f28f40b5d700c313d961e10361c9cad8ca2ef549317a304a854e14c56.png)

Table 10-16: DMC\_DT\_DATA\_CALIB\_DATA0 Register Fields

| Bit No. (Access)   | Bit Name                  | Description/Enumeration                                                                                                                                               |
|--------------------|---------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | DMC_DT_DATA_CAL- IB_DATA0 | Data Calibration Data 0. The DMC_DT_DATA_CALIB_DATA0.DMC_DT_DATA_CALIB_DATA0 bit field contains the first 32 bit data used for the write during the data calibration. |

## Data Calibration Data 1 Register

The DMC\_DT\_DATA\_CALIB\_DATA1 register contains the second 32-bit data used for the write during the data calibration.

Figure 10-8: DMC\_DT\_DATA\_CALIB\_DATA1 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000007_f1a77d17de943523991f361f9223b0636d82c49bca901fca15cb73f8c0673474.png)

Table 10-17: DMC\_DT\_DATA\_CALIB\_DATA1 Register Fields

| Bit No. (Access)   | Bit Name                  | Description/Enumeration                                                                                                                                                |
|--------------------|---------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | DMC_DT_DATA_CAL- IB_DATA1 | Data Calibration Data 1. The DMC_DT_DATA_CALIB_DATA1.DMC_DT_DATA_CALIB_DATA1 bit field contains the second 32 bit data used for the write during the data calibration. |

## Efficiency Control Register

The DMC\_EFFCTL register control DMC features that improve throughput efficiency. These include features such as auto-refresh management, precharge options, and write data options.

Figure 10-9: DMC\_EFFCTL Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000008_ecc5c1f9ffe45adc1f364bfd7e12848fad8e727f0f8ffd84f8224f7275752c2b.png)

Table 10-18: DMC\_EFFCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:20 (R/W)        | IDLECYC    | Idle Cycle. The DMC_EFFCTL.IDLECYC bits select the number of cycles after which theDMC issues any accumulated auto-refresh commands if postpone refresh is enabled ( DMC_CTL.PPREF =1). When DMC_EFFCTL.IDLECYC is set to 0, the DMCig- nores the DMC_CTL.PPREF selection and does not accumulate/postpone periodic auto-refresh commands. Note 1: By default, accumulated auto-refresh commands are issued after counting four idle cycles. Note 2: This value is ignored if DMC_CTL.PPREF is not set. Note 3: Setting this value to 0000 overrides the "postpone refresh" feature and does | Idle Cycle. The DMC_EFFCTL.IDLECYC bits select the number of cycles after which theDMC issues any accumulated auto-refresh commands if postpone refresh is enabled ( DMC_CTL.PPREF =1). When DMC_EFFCTL.IDLECYC is set to 0, the DMCig- nores the DMC_CTL.PPREF selection and does not accumulate/postpone periodic auto-refresh commands. Note 1: By default, accumulated auto-refresh commands are issued after counting four idle cycles. Note 2: This value is ignored if DMC_CTL.PPREF is not set. Note 3: Setting this value to 0000 overrides the "postpone refresh" feature and does |
|                    |            | 0-15                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | 0 to 15 Idle Cycles to Postpone Refresh Commands                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |

Table 10-18: DMC\_EFFCTL Register Fields (Continued)

| Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Number of Refresh Commands. The DMC_EFFCTL.NUMREF bits select the number of auto-refresh commands that the DMCcan accumulate if postpone refresh is enabled ( DMC_CTL.PPREF =1). The number of auto-refresh commands that can accumulate depends on whether theDMC is in DDR2 or LPDDR mode as selected by the DMC_CTL.LPDDR bit. In LPDDR mode, the DMCcan accumulate up to four auto-refresh commands. In DDR2 mode, the DMCmay accumulate up to eight auto-refresh commands. Note 1: By default, accumulated auto-refresh commands are issued after counting four idle cycles. Note 2: This value is ignored if DMC_CTL.PPREF is not set. 0 No Refresh Commands Accumulate 1 1 Refresh Command May Accumulate 2 2 Refresh Commands May Accumulate 3 3 Refresh Commands May Accumulate 4 4 Refresh Commands May Accumulate 5 5 Refresh Commands May Accumulate | Number of Refresh Commands. The DMC_EFFCTL.NUMREF bits select the number of auto-refresh commands that the DMCcan accumulate if postpone refresh is enabled ( DMC_CTL.PPREF =1). The number of auto-refresh commands that can accumulate depends on whether theDMC is in DDR2 or LPDDR mode as selected by the DMC_CTL.LPDDR bit. In LPDDR mode, the DMCcan accumulate up to four auto-refresh commands. In DDR2 mode, the DMCmay accumulate up to eight auto-refresh commands. Note 1: By default, accumulated auto-refresh commands are issued after counting four idle cycles. Note 2: This value is ignored if DMC_CTL.PPREF is not set. 0 No Refresh Commands Accumulate 1 1 Refresh Command May Accumulate 2 2 Refresh Commands May Accumulate 3 3 Refresh Commands May Accumulate 4 4 Refresh Commands May Accumulate 5 5 Refresh Commands May Accumulate | Number of Refresh Commands. The DMC_EFFCTL.NUMREF bits select the number of auto-refresh commands that the DMCcan accumulate if postpone refresh is enabled ( DMC_CTL.PPREF =1). The number of auto-refresh commands that can accumulate depends on whether theDMC is in DDR2 or LPDDR mode as selected by the DMC_CTL.LPDDR bit. In LPDDR mode, the DMCcan accumulate up to four auto-refresh commands. In DDR2 mode, the DMCmay accumulate up to eight auto-refresh commands. Note 1: By default, accumulated auto-refresh commands are issued after counting four idle cycles. Note 2: This value is ignored if DMC_CTL.PPREF is not set. 0 No Refresh Commands Accumulate 1 1 Refresh Command May Accumulate 2 2 Refresh Commands May Accumulate 3 3 Refresh Commands May Accumulate 4 4 Refresh Commands May Accumulate 5 5 Refresh Commands May Accumulate | Number of Refresh Commands. The DMC_EFFCTL.NUMREF bits select the number of auto-refresh commands that the DMCcan accumulate if postpone refresh is enabled ( DMC_CTL.PPREF =1). The number of auto-refresh commands that can accumulate depends on whether theDMC is in DDR2 or LPDDR mode as selected by the DMC_CTL.LPDDR bit. In LPDDR mode, the DMCcan accumulate up to four auto-refresh commands. In DDR2 mode, the DMCmay accumulate up to eight auto-refresh commands. Note 1: By default, accumulated auto-refresh commands are issued after counting four idle cycles. Note 2: This value is ignored if DMC_CTL.PPREF is not set. 0 No Refresh Commands Accumulate 1 1 Refresh Command May Accumulate 2 2 Refresh Commands May Accumulate 3 3 Refresh Commands May Accumulate 4 4 Refresh Commands May Accumulate 5 5 Refresh Commands May Accumulate | Number of Refresh Commands. The DMC_EFFCTL.NUMREF bits select the number of auto-refresh commands that the DMCcan accumulate if postpone refresh is enabled ( DMC_CTL.PPREF =1). The number of auto-refresh commands that can accumulate depends on whether theDMC is in DDR2 or LPDDR mode as selected by the DMC_CTL.LPDDR bit. In LPDDR mode, the DMCcan accumulate up to four auto-refresh commands. In DDR2 mode, the DMCmay accumulate up to eight auto-refresh commands. Note 1: By default, accumulated auto-refresh commands are issued after counting four idle cycles. Note 2: This value is ignored if DMC_CTL.PPREF is not set. 0 No Refresh Commands Accumulate 1 1 Refresh Command May Accumulate 2 2 Refresh Commands May Accumulate 3 3 Refresh Commands May Accumulate 4 4 Refresh Commands May Accumulate 5 5 Refresh Commands May Accumulate | Number of Refresh Commands. The DMC_EFFCTL.NUMREF bits select the number of auto-refresh commands that the DMCcan accumulate if postpone refresh is enabled ( DMC_CTL.PPREF =1). The number of auto-refresh commands that can accumulate depends on whether theDMC is in DDR2 or LPDDR mode as selected by the DMC_CTL.LPDDR bit. In LPDDR mode, the DMCcan accumulate up to four auto-refresh commands. In DDR2 mode, the DMCmay accumulate up to eight auto-refresh commands. Note 1: By default, accumulated auto-refresh commands are issued after counting four idle cycles. Note 2: This value is ignored if DMC_CTL.PPREF is not set. 0 No Refresh Commands Accumulate 1 1 Refresh Command May Accumulate 2 2 Refresh Commands May Accumulate 3 3 Refresh Commands May Accumulate 4 4 Refresh Commands May Accumulate 5 5 Refresh Commands May Accumulate | Number of Refresh Commands. The DMC_EFFCTL.NUMREF bits select the number of auto-refresh commands that the DMCcan accumulate if postpone refresh is enabled ( DMC_CTL.PPREF =1). The number of auto-refresh commands that can accumulate depends on whether theDMC is in DDR2 or LPDDR mode as selected by the DMC_CTL.LPDDR bit. In LPDDR mode, the DMCcan accumulate up to four auto-refresh commands. In DDR2 mode, the DMCmay accumulate up to eight auto-refresh commands. Note 1: By default, accumulated auto-refresh commands are issued after counting four idle cycles. Note 2: This value is ignored if DMC_CTL.PPREF is not set. 0 No Refresh Commands Accumulate 1 1 Refresh Command May Accumulate 2 2 Refresh Commands May Accumulate 3 3 Refresh Commands May Accumulate 4 4 Refresh Commands May Accumulate 5 5 Refresh Commands May Accumulate |
| 8 8 Refresh Commands May Accumulate Precharge Bank 7. The DMC_EFFCTL.PRECBANK7 bit enables precharge (closes the page) of bank 7 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. 0 Disable Precharge Bank 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | 8 8 Refresh Commands May Accumulate Precharge Bank 7. The DMC_EFFCTL.PRECBANK7 bit enables precharge (closes the page) of bank 7 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. 0 Disable Precharge Bank 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | 8 8 Refresh Commands May Accumulate Precharge Bank 7. The DMC_EFFCTL.PRECBANK7 bit enables precharge (closes the page) of bank 7 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. 0 Disable Precharge Bank 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | 8 8 Refresh Commands May Accumulate Precharge Bank 7. The DMC_EFFCTL.PRECBANK7 bit enables precharge (closes the page) of bank 7 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. 0 Disable Precharge Bank 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | 8 8 Refresh Commands May Accumulate Precharge Bank 7. The DMC_EFFCTL.PRECBANK7 bit enables precharge (closes the page) of bank 7 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. 0 Disable Precharge Bank 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | 8 8 Refresh Commands May Accumulate Precharge Bank 7. The DMC_EFFCTL.PRECBANK7 bit enables precharge (closes the page) of bank 7 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. 0 Disable Precharge Bank 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | 8 8 Refresh Commands May Accumulate Precharge Bank 7. The DMC_EFFCTL.PRECBANK7 bit enables precharge (closes the page) of bank 7 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. 0 Disable Precharge Bank 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 1 Enable Precharge Bank 7 Precharge Bank 6. The DMC_EFFCTL.PRECBANK6 bit enables precharge (closes the page) of bank 6 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. 0 Disable Precharge Bank 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | 1 Enable Precharge Bank 7 Precharge Bank 6. The DMC_EFFCTL.PRECBANK6 bit enables precharge (closes the page) of bank 6 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. 0 Disable Precharge Bank 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | 1 Enable Precharge Bank 7 Precharge Bank 6. The DMC_EFFCTL.PRECBANK6 bit enables precharge (closes the page) of bank 6 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. 0 Disable Precharge Bank 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | 1 Enable Precharge Bank 7 Precharge Bank 6. The DMC_EFFCTL.PRECBANK6 bit enables precharge (closes the page) of bank 6 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. 0 Disable Precharge Bank 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | 1 Enable Precharge Bank 7 Precharge Bank 6. The DMC_EFFCTL.PRECBANK6 bit enables precharge (closes the page) of bank 6 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. 0 Disable Precharge Bank 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | 1 Enable Precharge Bank 7 Precharge Bank 6. The DMC_EFFCTL.PRECBANK6 bit enables precharge (closes the page) of bank 6 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. 0 Disable Precharge Bank 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | 1 Enable Precharge Bank 7 Precharge Bank 6. The DMC_EFFCTL.PRECBANK6 bit enables precharge (closes the page) of bank 6 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. 0 Disable Precharge Bank 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 1 Enable Precharge Bank 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | 1 Enable Precharge Bank 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | 1 Enable Precharge Bank 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | 1 Enable Precharge Bank 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | 1 Enable Precharge Bank 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | 1 Enable Precharge Bank 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | 1 Enable Precharge Bank 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

Table 10-18: DMC\_EFFCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | PRECBANK5  | Precharge Bank 5. The DMC_EFFCTL.PRECBANK5 bit enables precharge (closes the page) of bank 5 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged.        |
| 13 (R/W)           | PRECBANK5  | 0 Disable Precharge Bank 5                                                                                                                                                                                                                                                                                       |
| 13 (R/W)           | PRECBANK5  | 1 Enable Precharge Bank 5                                                                                                                                                                                                                                                                                        |
| 12 (R/W)           | PRECBANK4  | Precharge Bank 4. The DMC_EFFCTL.PRECBANK4 bit enables precharge (closes the page) of bank 4 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged.        |
| 12 (R/W)           | PRECBANK4  | 0 Disable Precharge Bank 4                                                                                                                                                                                                                                                                                       |
| 12 (R/W)           | PRECBANK4  | 1 Enable Precharge Bank 4                                                                                                                                                                                                                                                                                        |
| 11 (R/W)           | PRECBANK3  | Precharge Bank 3. The DMC_EFFCTL.PRECBANK3 bit enables precharge (closes the page) of bank 3 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged.        |
| 11 (R/W)           | PRECBANK3  | 0 Disable Precharge Bank 3                                                                                                                                                                                                                                                                                       |
| 11 (R/W)           | PRECBANK3  | 1 Enable Precharge Bank 3                                                                                                                                                                                                                                                                                        |
| 10 (R/W)           | PRECBANK2  | Precharge Bank 2. The DMC_EFFCTL.PRECBANK2 bit enables precharge (closes the page) of bank 2 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged.        |
| 10 (R/W)           | PRECBANK2  | 0 Disable Precharge Bank 2                                                                                                                                                                                                                                                                                       |
| 10 (R/W)           | PRECBANK2  | 1 Enable Precharge Bank 2                                                                                                                                                                                                                                                                                        |
| 9 (R/W)            | PRECBANK1  | Precharge Bank 1. The DMC_EFFCTL.PRECBANK1 bit enables precharge (closes the page) of bank 1 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. Bank 1 |
| 9 (R/W)            | PRECBANK1  | 0 Disable Precharge                                                                                                                                                                                                                                                                                              |
| 9 (R/W)            | PRECBANK1  | 1 Enable Precharge Bank 1                                                                                                                                                                                                                                                                                        |

Table 10-18: DMC\_EFFCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | PRECBANK0  | Precharge Bank 0. The DMC_EFFCTL.PRECBANK0 bit enables precharge (closes the page) of bank 0 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. |
| 8 (R/W)            | PRECBANK0  | 0 Disable Precharge Bank 0                                                                                                                                                                                                                                                                                |
| 8 (R/W)            | PRECBANK0  | 1 Enable Precharge Bank 0                                                                                                                                                                                                                                                                                 |

## Shadow EMR1 DDR2 Register

The DMC\_EMR1 register in the DMC shadows the EMR1 register in the SDRAM when the DMC is in DDR2 mode ( DMC\_CTL.LPDDR =0). This register is used only when the DMC is operating in DDR2 mode.

If unmasked by the corresponding bit in the shadow mask register ( DMC\_MSK.EMR1 =1), a write to DMC\_EMR1 triggers an extended 'mode register set' command on the memory interface. If masked, a write to DMC\_EMR1 only updates the register in the DMC, not the register in the SDRAM.

Figure 10-10: DMC\_EMR1 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000009_17e7039571322476d7981435071029cff89f91da5caea488cc7b922cbe722efc.png)

Table 10-19: DMC\_EMR1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | QOFF       | Output Buffer Enable. The DMC_EMR1.QOFF bit enables the SDRAM output pins. For more information about this operation, see the data sheet for the SDRAM being used in your system.                                                         |
| 10 (R/W)           | DQS        | LDQS/UDQS Enable. The DMC_EMR1.DQS bit enables the single ended operation of the DMC_LDQS / DMC_LDQS or DMC_UDQS / DMC_UDQS pin. For more information about this opera- tion, see the data sheet for the SDRAM being used in your system. |
| 6 (R/W)            | RTT1       | RTT Termination Resistance 1. The DMC_EMR1.RTT1 bit combines with the DMC_EMR1.RTT0 bit to set the ter- mination resistance. See the DMC_EMR1.RTT0 bit description for more information.                                                  |

Table 10-19: DMC\_EMR1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                  | Description/Enumeration                                                                                                                                                                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5:3 (R/W)          | AL         | Additive Latency. The DMC_EMR1.AL bits select a number of added latency time for CAS operations in terms of clock cycles (t CK ). For more information about this operation, see the data sheet for the SDRAM being used in your system.                                 | Additive Latency. The DMC_EMR1.AL bits select a number of added latency time for CAS operations in terms of clock cycles (t CK ). For more information about this operation, see the data sheet for the SDRAM being used in your system.                                 |
| 5:3 (R/W)          | AL         | 0                                                                                                                                                                                                                                                                        | 0 Clock Cycles Added                                                                                                                                                                                                                                                     |
| 5:3 (R/W)          | AL         | 1                                                                                                                                                                                                                                                                        | 1 Clock Cycle Added                                                                                                                                                                                                                                                      |
| 5:3 (R/W)          | AL         | 2                                                                                                                                                                                                                                                                        | 2 Clock Cycles Added                                                                                                                                                                                                                                                     |
| 5:3 (R/W)          | AL         | 3                                                                                                                                                                                                                                                                        | 3 Clock Cycles Added                                                                                                                                                                                                                                                     |
| 5:3 (R/W)          | AL         | 4                                                                                                                                                                                                                                                                        | 4 Clock Cycles Added                                                                                                                                                                                                                                                     |
| 5:3 (R/W)          | AL         | 5                                                                                                                                                                                                                                                                        | 5 Clock Cycles Added                                                                                                                                                                                                                                                     |
| 2                  | RTT0       | Termination Resistance 0. The DMC_EMR1.RTT0 bit and the DMC_EMR1.RTT1 bits select the SDRAM ter- mination resistance. RTT1=0, RTT0=0: No ODT at memory device RTT1=0, RTT0=1: 75 Ohm ODT at memory device                                                                | Termination Resistance 0. The DMC_EMR1.RTT0 bit and the DMC_EMR1.RTT1 bits select the SDRAM ter- mination resistance. RTT1=0, RTT0=0: No ODT at memory device RTT1=0, RTT0=1: 75 Ohm ODT at memory device                                                                |
| 1 (R/W)            | DIC        | Output Driver Impedance Control. The DMC_EMR1.DIC bit selects the drive strength mode for the SDRAM. For more information about this operation, see the data sheet for the SDRAM being used in your system. It must be kept at 0 if the SDRAM does not support this bit. | Output Driver Impedance Control. The DMC_EMR1.DIC bit selects the drive strength mode for the SDRAM. For more information about this operation, see the data sheet for the SDRAM being used in your system. It must be kept at 0 if the SDRAM does not support this bit. |
| 1 (R/W)            | DIC        | 0                                                                                                                                                                                                                                                                        | Full Strength                                                                                                                                                                                                                                                            |
| 1 (R/W)            | DIC        | 1                                                                                                                                                                                                                                                                        | Reduced Strength                                                                                                                                                                                                                                                         |
| 0 (R/W)            | DLLEN      | DLL Enable. The DMC_EMR1.DLLEN bit enables the DLL in the SDRAM. For more information about this operation, see the data sheet for the SDRAM being used in your system.                                                                                                  | DLL Enable. The DMC_EMR1.DLLEN bit enables the DLL in the SDRAM. For more information about this operation, see the data sheet for the SDRAM being used in your system.                                                                                                  |
| 0 (R/W)            | DLLEN      | 0                                                                                                                                                                                                                                                                        | Enable DLL (Normal Operation)                                                                                                                                                                                                                                            |
| 0 (R/W)            | DLLEN      | 1                                                                                                                                                                                                                                                                        | Disable DLL (Test/Debug Operation)                                                                                                                                                                                                                                       |

## Shadow EMR2 Register (DDR2)/Shadow EMR Register (LPDDR)

The DMC\_EMR2 register in the DMC shadows the EMR2 register in the SDRAM when the DMC is in DDR2 mode ( DMC\_CTL.LPDDR =0) and shadows the EMR register in the SDRAM when the DMC is in LPDDR mode ( DMC\_CTL.LPDDR =1). If unmasked by the corresponding bit in the shadow mask register ( DMC\_MSK.EMR2 =1), a write to DMC\_EMR2 triggers an extended 'mode register set' command on the memory interface. If masked, a write to DMC\_EMR2 only updates the register in the DMC, not the register in the SDRAM.

Figure 10-11: DMC\_EMR2 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000010_1507f8514ad4b5c8255b239ed841d864a25d5d9c9027531fdd5fc8db73f579f0.png)

Table 10-20: DMC\_EMR2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | SRF        | High Temperature Self-Refresh. The DMC_EMR2.SRF bit enables the SDRAM's high temperature self-refresh rate feature when the DMCis in DDR2 mode. (This bit is reserved in LPDDR mode.) For more information about this operation, see the data sheet for the SDRAM being used                                                 | High Temperature Self-Refresh. The DMC_EMR2.SRF bit enables the SDRAM's high temperature self-refresh rate feature when the DMCis in DDR2 mode. (This bit is reserved in LPDDR mode.) For more information about this operation, see the data sheet for the SDRAM being used                                                 |
| 7 (R/W)            | SRF        | 0                                                                                                                                                                                                                                                                                                                            | Disable                                                                                                                                                                                                                                                                                                                      |
| 6:5 (R/W)          | DS         | Drive Strength. The DMC_EMR2.DS bits select the drive strength value when the DMCis in LPDDR mode. (These bits are reserved when the DMCis in DDR2 mode.) Note that all val- ues other than those shown are reserved. For more information about this operation, see the data sheet for the SDRAM being used in your system. | Drive Strength. The DMC_EMR2.DS bits select the drive strength value when the DMCis in LPDDR mode. (These bits are reserved when the DMCis in DDR2 mode.) Note that all val- ues other than those shown are reserved. For more information about this operation, see the data sheet for the SDRAM being used in your system. |
| 6:5 (R/W)          | DS         | 4                                                                                                                                                                                                                                                                                                                            | Octant Drive strength                                                                                                                                                                                                                                                                                                        |
| 6:5 (R/W)          | DS         | 0                                                                                                                                                                                                                                                                                                                            | Full Drive Strength                                                                                                                                                                                                                                                                                                          |
| 6:5 (R/W)          | DS         | 1                                                                                                                                                                                                                                                                                                                            | 1/2 Drive Strength                                                                                                                                                                                                                                                                                                           |
| 6:5 (R/W)          | DS         | 2                                                                                                                                                                                                                                                                                                                            | 3/4 Drive Strength                                                                                                                                                                                                                                                                                                           |
| 6:5 (R/W)          | DS         | 3                                                                                                                                                                                                                                                                                                                            | 1/4 Drive Strength                                                                                                                                                                                                                                                                                                           |

Table 10-20: DMC\_EMR2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                       | Description/Enumeration                                                                                                                                                                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4:3                | TCSR       | Temperature Compensated Self-Refresh.                                                                                                                                                                                                                                                         | Temperature Compensated Self-Refresh.                                                                                                                                                                                                                                                         |
| (R/W)              |            | The DMC_EMR2.TCSR bits select the temperature for applying temperature compen- sated self-refresh when the DMCis in LPDDR mode. (These bits are reserved when the DMCis in DDR2 mode.) For more information about this operation, see the data sheet for the SDRAM being used in your system. | The DMC_EMR2.TCSR bits select the temperature for applying temperature compen- sated self-refresh when the DMCis in LPDDR mode. (These bits are reserved when the DMCis in DDR2 mode.) For more information about this operation, see the data sheet for the SDRAM being used in your system. |
|                    |            | 0                                                                                                                                                                                                                                                                                             | 70 degree C (in LPDDR Mode)                                                                                                                                                                                                                                                                   |
|                    |            | 1                                                                                                                                                                                                                                                                                             | 45 degree C                                                                                                                                                                                                                                                                                   |
|                    |            | 2                                                                                                                                                                                                                                                                                             | 15 degree C                                                                                                                                                                                                                                                                                   |
|                    |            | 3                                                                                                                                                                                                                                                                                             | 85 degree C                                                                                                                                                                                                                                                                                   |
| 2:0 (R/W)          | PASR       | Partial Array Self-Refresh. The DMC_EMR2.PASR bits select the amount of memory to be refreshed during self- refresh. For more information about this operation, see the data sheet for the SDRAM being used in your system.                                                                   | Partial Array Self-Refresh. The DMC_EMR2.PASR bits select the amount of memory to be refreshed during self- refresh. For more information about this operation, see the data sheet for the SDRAM being used in your system.                                                                   |
|                    |            | 0                                                                                                                                                                                                                                                                                             | Full Array for DDR2 , DDR3, and LPDDR Modes. Applies to both 4 and 8 bank devices.                                                                                                                                                                                                            |
|                    |            | 1                                                                                                                                                                                                                                                                                             | 1/2 Array for DDR2 , DDR3, and LPDDR Modes. For 4 bank devices, BA[1:0] =00 and 01. For 8 bank devi- ces, BA[2:0] = 000, 001, 010, 011                                                                                                                                                        |
|                    |            | 2                                                                                                                                                                                                                                                                                             | 1/4 Array for DDR2 , DDR3, and LPDDR Modes. For 4 bank devices, BA[1:0] = 00. For 8 bank devices, BA[2:0] = 000 and 001.                                                                                                                                                                      |
|                    |            | 3                                                                                                                                                                                                                                                                                             | 1/8 Array for 8 DDR2 or DDR3 Banks Only. Reserved for LPDDR. For 4 bank devices, not defined. For 8 bank devices, BA[2:0] = 000.                                                                                                                                                              |
|                    |            | 4                                                                                                                                                                                                                                                                                             | 3/4 Array for DDR2 or DDR3. Reserved for LPDDR. For 4 bank devices, BA[1:0]=01, 10 and 11. For 8 bank devices, BA[2:0] = 010, 011, 100, 101, 110, and 111.                                                                                                                                    |
|                    |            | 5                                                                                                                                                                                                                                                                                             | 1/2 Array for DDR2 or DDR3. 1/8 Array for LPDDR. For 4 bank devices, BA[1:0]=10 and 11. For 8 bank de- vices, BA[2:0] = 100, 101, 110, and 111.                                                                                                                                               |
|                    |            | 6                                                                                                                                                                                                                                                                                             | 1/4 Array for DDR2 or DDR3. 1/16 Array for LPDDR. For 4 bank devices, BA[1:0]=11. For 8 bank devices, BA[2:0] =110 and 111.                                                                                                                                                                   |
|                    |            | 7                                                                                                                                                                                                                                                                                             | 1/8 array (for DDR2 or DDR3 Banks only); Reserved (LPDDR)                                                                                                                                                                                                                                     |

## Shadow MR Register (DDR2/LPDDR), Shadow MR0 Register (DDR3)

The DMC\_MR register in the DMC shadows the MR register in the SDRAM when the DMC is in DDR2 mode or LPDDR mode ( DMC\_CTL.LPDDR =0 or =1 and DMC\_CTL.DDR3EN =0) or DDR3 mode ( DMC\_CTL.DDR3EN =0 or =1 and DMC\_CTL.LPDDR =0). If unmasked by the corresponding bit in the shadow mask register ( DMC\_MSK.MR =1), a write to DMC\_MR triggers a 'mode register set' command on the memory interface. If masked, a write to DMC\_MR only updates the register in the DMC, not the register in the SDRAM.

Figure 10-12: DMC\_MR Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000011_99b0b0d23f6cd52df14fc5374ea8a91c49bb538a8cfa308fa919595f87069813.png)

Table 10-21: DMC\_MR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                             | Description/Enumeration                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | PD         | Active Power Down Mode. The DMC_MR.PD bit selects the active power-down mode. Note that this parameter applies only for DDR2/DDR3 mode and is reserved for LPDDR mode. For more in- formation about this mode, see the data sheet for the SDRAM being used in your sys- tem.                        | Active Power Down Mode. The DMC_MR.PD bit selects the active power-down mode. Note that this parameter applies only for DDR2/DDR3 mode and is reserved for LPDDR mode. For more in- formation about this mode, see the data sheet for the SDRAM being used in your sys- tem.                        |
| 12 (R/W)           | PD         | 0                                                                                                                                                                                                                                                                                                   | Fast Exit (normal)                                                                                                                                                                                                                                                                                  |
| 11:9 (R/W)         | WRRECOV    | Write Recovery. The DMC_MR.WRRECOV bit selects the write recovery time in terms of clock cycles (t CK ). Note that this parameter applies only for DDR2/DDR3 mode and is reserved for LPDDR mode. For more information about this mode, see the data sheet for the SDRAM being used in your system. | Write Recovery. The DMC_MR.WRRECOV bit selects the write recovery time in terms of clock cycles (t CK ). Note that this parameter applies only for DDR2/DDR3 mode and is reserved for LPDDR mode. For more information about this mode, see the data sheet for the SDRAM being used in your system. |
| 11:9 (R/W)         | WRRECOV    | 0                                                                                                                                                                                                                                                                                                   | 16 clock cycles for DDR3 only                                                                                                                                                                                                                                                                       |
| 11:9 (R/W)         | WRRECOV    | 1                                                                                                                                                                                                                                                                                                   | 2 Clock Cycles for DDR2 and 5 clock cycles for DDR3                                                                                                                                                                                                                                                 |
| 11:9 (R/W)         | WRRECOV    | 2                                                                                                                                                                                                                                                                                                   | 3 Clock Cycles for DDR2 and 6 clock cycles for DDR3                                                                                                                                                                                                                                                 |
| 11:9 (R/W)         | WRRECOV    | 3                                                                                                                                                                                                                                                                                                   | 4 Clock Cycles for DDR2 and 7 clock cycles for DDR3                                                                                                                                                                                                                                                 |
| 11:9 (R/W)         | WRRECOV    | 4                                                                                                                                                                                                                                                                                                   | 5 Clock Cycles for DDR2 and 8 clock cycles for DDR3                                                                                                                                                                                                                                                 |

Table 10-21: DMC\_MR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | DLLRST     | 5 6 Clock Cycles for DDR2 and 10 clock cycles for DDR3 6 7 Clock Cycles for DDR2 and 12 clock cycles for DDR3 7 8 Clock Cycles for DDR2 and 14 clock cycles for                                                                                                                           |
|                    |            | DDR3                                                                                                                                                                                                                                                                                      |
|                    |            | DLL Reset. The DMC_MR.DLLRST bit initiates a DLL reset on the SDRAM. Note that this pa- rameter applies only for DDR2/DDR3 mode and is reserved for LPDDR mode. For more information about this operation, see the data sheet for the SDRAM being used in your system. 0 Normal Operation |
|                    |            | 1 Reset DLL                                                                                                                                                                                                                                                                               |

Table 10-21: DMC\_MR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:4 (R/W)          | CL         | CAS Latency. The DMC_MR.CL bit field selects latency from the assertion of a read/write signal to the SDRAM until the first valid data on the output from the SDRAM in terms of clock cycles. For more information about this operation, see the data sheet for the SDRAM being used in your system. The valid numbers for DDR2 and LPDDR are: 010 = Reserved 011 = 3 100 = 4 (DDR2 only) 101 = 5 (DDR2 only) 110 = 6 (DDR2 only) For DDR3 only bit [2] DMC_MR.CL0 should be used along with bits [6:4]: 0010 = 5 0100 = 6 0110 = 7 1000 = 8 1010 = 9 1100 = 10 1110 = 11 0001 = 12 | CAS Latency. The DMC_MR.CL bit field selects latency from the assertion of a read/write signal to the SDRAM until the first valid data on the output from the SDRAM in terms of clock cycles. For more information about this operation, see the data sheet for the SDRAM being used in your system. The valid numbers for DDR2 and LPDDR are: 010 = Reserved 011 = 3 100 = 4 (DDR2 only) 101 = 5 (DDR2 only) 110 = 6 (DDR2 only) For DDR3 only bit [2] DMC_MR.CL0 should be used along with bits [6:4]: 0010 = 5 0100 = 6 0110 = 7 1000 = 8 1010 = 9 1100 = 10 1110 = 11 0001 = 12 |
| 2 (R/W)            | CL0        | CAS Latency 0. The DMC_MR.CL0 bit is applicable for DDR3 only and is used in conjunction with the DMC_MR.CL bits.                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | CAS Latency 0. The DMC_MR.CL0 bit is applicable for DDR3 only and is used in conjunction with the DMC_MR.CL bits.                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 1:0 (R/W)          | BLEN       | Burst Length. The DMC_MR.BLEN bits select burst length for transfers. For more information about this operation, see the data sheet for the SDRAM being used in your system. Note that values other than those shown are not supported.                                                                                                                                                                                                                                                                                                                                             | Burst Length. The DMC_MR.BLEN bits select burst length for transfers. For more information about this operation, see the data sheet for the SDRAM being used in your system. Note that values other than those shown are not supported.                                                                                                                                                                                                                                                                                                                                             |
| 1:0 (R/W)          | BLEN       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | 8-Bit Burst Length - DDR3 only                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 1:0 (R/W)          | BLEN       | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | 4-Bit Burst Length -LPDDR/DDR2 only                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |

## Shadow MR1 Register (DDR3)

The DMC\_MR1 register is a mirror of the DDR3 SDRAM Mode register 1. This register is used only when the DMC is operating in DDR3 mode. A write to this register triggers an extended "mode register 1 set" command on the memory interface provided the corresponding mask bit is set in the mask register. Else, only the mirror register is updated.

Figure 10-13: DMC\_MR1 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000012_8224244c4bab299825c47ff60c057bf36132e54fcd6506373f23583bd1059fdc.png)

Table 10-22: DMC\_MR1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | QOFF       | Output Buffer Enable. The DMC_MR1.QOFF bit enables the SDRAM output pins. For more information about this operation, see the data sheet for the SDRAM being used in your system. 0 Output buffer enabled                                                                                                                                                                             |
| 11 (R/W)           | TDQS       | Termination Data Strobe. The DMC_MR1.TDQS bit provides additional termination resistance outputs that may be useful in some system configurations. The DMC_MR1.TDQS bit is not supported in x4 or x16 configurations. When enabled via the mode register, the same termination resistance function is applied to the TDQS/TDQS# pins that is applied to the DQS/ DQS# pins. 0 Enable |
| 11 (R/W)           |            | 1 Disable                                                                                                                                                                                                                                                                                                                                                                            |
| 11 (R/W)           |            |                                                                                                                                                                                                                                                                                                                                                                                      |

Table 10-22: DMC\_MR1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | RTT2       | Rtt_nom. The DMC_MR1.RTT2 bit is used in conjunction with the DMC_MR1.RTT0 and DMC_MR1.RTT1 bits. (9 6 2) 0 0 0 Rtt_Nom disabled 0 0 1 RZQ/4 0 1 0 RZQ/2 0 1 1 RZQ/6 1 0 0 RZQ/12 (reserved if Rtt_Nom is used during writes) 1 0 1 RZQ/8 (reserved if Rtt_Nom is used during writes) 1 1 0 Reserved 1 1 1 Reserved |
| 6 (R/W)            | RTT1       | Rtt_nom. The DMC_MR1.RTT1 bit combines with the DMC_MR1.RTT0 bit to set the termi- nation resistance. See the DMC_MR1.RTT2 and DMC_MR1.RTT0 bit description for more information.                                                                                                                                   |
| 5 (R/W)            | DIC1       | Output Driver Impedance Control. The DMC_MR1.DIC1 bit is used in conjunction with the DMC_MR1.DIC0 bit. (5, 1) 0 0 RZQ/6 0 1 RZQ/7 1 0 Reserved 1 1 Reserved                                                                                                                                                        |
| 4:3 (R/W)          | AL         | Additive Latency. The DMC_MR1.AL bits select a number of added latency time for CAS operations in terms of clock cycles (t CK ). For more information about this operation, see the data sheet for the SDRAM being used in your system. 0 AL disabled 1 CL-1 2 CL-2                                                 |
| 2 (R/W)            | RTT0       | Rtt_nom. The DMC_MR1.RTT0 bit combines with the DMC_MR1.RTT1 and DMC_MR1.RTT1 bits to set the termination resistance. See the DMC_MR1.RTT1 and DMC_MR1.RTT2 bit descriptions for more information.                                                                                                                  |
| 4:3 (R/W)          | AL         |                                                                                                                                                                                                                                                                                                                     |
| 4:3 (R/W)          | AL         |                                                                                                                                                                                                                                                                                                                     |

Table 10-22: DMC\_MR1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | DIC0       | Output Driver Impedance control. The DMC_MR1.DIC0 bit is used with the DMC_MR1.DIC1 bit.                                                                                                  |
| 0 (R/W)            | DLLEN      | DLL Enable. The DMC_MR1.DLLEN bit enables the DLL in the SDRAM. For more information about this operation, see the data sheet for the SDRAM being used in your system. 0 Enable 1 Disable |

## Shadow MR2 Register (DDR3)

The DMC\_MR2 register mirrors DDR3 SDRAM device Mode register 2 when the controller is operating in DDR3 mode. A write to this register triggers an extended "mode register set" command on the memory interface provided the corresponding mask bit is set in the mask register. Else, only the mirror register is updated.

Figure 10-14: DMC\_MR2 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000013_5701a9196e437967874936c82056bbb35f2bd6b3ffbb5eebb6ba5aa8509d3212.png)

Table 10-23: DMC\_MR2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | SRT        | Self Refresh Temperature Range. The DMC_MR2.SRT bit enables high temperature self-refresh rate.                   |
| 6 (R/W)            | ASR        | Auto Self Refresh. 0 Manual SR Reference (SRT)                                                                    |
| 5:3 (R/W)          | CWL        | Latency. 0 5 clock cycles                                                                                         |
|                    |            | CAS Write 1 6 clock cycles 2 7 clock cycles 3 8 clock cycles 4 9 clock cycles 5 10 clock cycles 6 11 clock cycles |

Table 10-23: DMC\_MR2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2:0 (R/W)          | PASR       | Partial Array Self refresh. The DMC_MR2.PASR bits select the amount of memory to be refreshed during self refresh. For more information about this operation, see the data sheet for the SDRAM being used in your system. | Partial Array Self refresh. The DMC_MR2.PASR bits select the amount of memory to be refreshed during self refresh. For more information about this operation, see the data sheet for the SDRAM being used in your system. |
|                    |            | 0                                                                                                                                                                                                                         | 4 banks: full array, 8 banks: full array                                                                                                                                                                                  |
|                    |            | 1                                                                                                                                                                                                                         | 4 banks: Half Array (BA[1:0]=00&01), 8 banks: Half Array (BA[2:0] = 000, 001, 010, &011)                                                                                                                                  |
|                    |            | 2                                                                                                                                                                                                                         | 4 banks: Quarter Array (BA[1:0]=00), 8 banks: Quarter Array (BA[2:0] = 000&001)                                                                                                                                           |
|                    |            | 3                                                                                                                                                                                                                         | 4 banks: not defined, 8 banks: 1/8th array (BA[2:0] = 000)                                                                                                                                                                |
|                    |            | 4                                                                                                                                                                                                                         | 4 banks: 3/4 Array (BA[1:0]=01, 10&11), 8 banks: 3/4 Array (BA[2:0] = 010, 011, 100, 101, 110, &111)                                                                                                                      |
|                    |            | 5                                                                                                                                                                                                                         | 4 banks: Half Array (BA[1:0]=10&11), 8 banks: Half Array (BA[2:0] = 100, 101, 110, &111)                                                                                                                                  |
|                    |            | 6                                                                                                                                                                                                                         | 4 banks: Quarter Array (BA[1:0]=11), 8 banks: Quarter Array (BA[2:0] =110 &111)                                                                                                                                           |
|                    |            | 7                                                                                                                                                                                                                         | 4 banks: not defined, 8 banks: 1/8th array (BA[2:0] = 111)                                                                                                                                                                |

## Mask (Mode Register Shadow) Register

The DMC\_MSK register permits masking (disabling) writes to the MR and EMRn registers in the SDRAM. When masked, writes to these registers go instead to shadow copies of these registers ( DMC\_MR , DMC\_EMR1 , DMC\_EMR2 ), which are maintained within the DMC. When a shadow register's corresponding bit is unmasked (enabled), the DMC generates the MRS or EMRS command to transfer the contents of the shadow register (in the DMC) to the actual register (in the SDRAM).

Figure 10-15: DMC\_MSK Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000014_61c051d25cdd22dd4b9f162ea38c2f0541b7ee875efe1efad934370f072aa30e.png)

Table 10-24: DMC\_MSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                         | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | EMR3       | Shadow EMR3 Unmask. The DMC_MSK.EMR3 bit masks or unmasks writes to the EMR3 register (in DDR2) in the SDRAM. When masked, writes to this register instead go to the EMR3 register. When unmasked, the DMCwrites the EMR3 value to the EMR3 register (in DDR2) in the SDRAM. After completing the write, the DMCclears this bit. Note that this bit must not be enabled when in LPDDR mode ( DMC_CTL.LPDDR =1). | Shadow EMR3 Unmask. The DMC_MSK.EMR3 bit masks or unmasks writes to the EMR3 register (in DDR2) in the SDRAM. When masked, writes to this register instead go to the EMR3 register. When unmasked, the DMCwrites the EMR3 value to the EMR3 register (in DDR2) in the SDRAM. After completing the write, the DMCclears this bit. Note that this bit must not be enabled when in LPDDR mode ( DMC_CTL.LPDDR =1). |
| 10 (R/W)           | EMR2       | Shadow EMR2 Unmask. The DMC_MSK.EMR2 bit masks or unmasks writes to the EMR2 register (in DDR2) or the EMR register (in LPDDR) in the SDRAM. When masked, writes to this regis- ter instead go to the DMC_EMR2 register. When unmasked, the DMCwrites the DMC_EMR2 value to the EMR2 register (in DDR2) or the EMR register (in LPDDR) in the SDRAM. After completing the write, the DMCclears this bit.        | Shadow EMR2 Unmask. The DMC_MSK.EMR2 bit masks or unmasks writes to the EMR2 register (in DDR2) or the EMR register (in LPDDR) in the SDRAM. When masked, writes to this regis- ter instead go to the DMC_EMR2 register. When unmasked, the DMCwrites the DMC_EMR2 value to the EMR2 register (in DDR2) or the EMR register (in LPDDR) in the SDRAM. After completing the write, the DMCclears this bit.        |
| 10 (R/W)           | EMR2       | 0                                                                                                                                                                                                                                                                                                                                                                                                               | Mask (Disable) Write to EMR2                                                                                                                                                                                                                                                                                                                                                                                    |
| 10 (R/W)           | EMR2       | 1                                                                                                                                                                                                                                                                                                                                                                                                               | Unmask (Enable) Write to EMR2                                                                                                                                                                                                                                                                                                                                                                                   |

Table 10-24: DMC\_MSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                             | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | EMR1       | Shadow EMR1 Unmask. The DMC_MSK.EMR1 bit masks or unmasks writes to the EMR1 register in the SDRAM. When masked, writes to this register instead go to the DMC_EMR1 register. When unmasked, the DMCwrites the DMC_EMR1 value to the EMR1 register in the SDRAM. After completing the write, the DMCclears this bit. Note that this bit must not be enabled when in LPDDR mode ( DMC_CTL.LPDDR =1). | Shadow EMR1 Unmask. The DMC_MSK.EMR1 bit masks or unmasks writes to the EMR1 register in the SDRAM. When masked, writes to this register instead go to the DMC_EMR1 register. When unmasked, the DMCwrites the DMC_EMR1 value to the EMR1 register in the SDRAM. After completing the write, the DMCclears this bit. Note that this bit must not be enabled when in LPDDR mode ( DMC_CTL.LPDDR =1). |
| 8 (R/W)            | MR         | Shadow MRUnmask. The DMC_MSK.MR bit masks or unmasks writes to the MRregister in the SDRAM. When masked, writes to this register instead go to the DMC_MR register. When un- masked, the DMCwrites the DMC_MR value to the MRregister in the SDRAM. After completing the write, the DMCclears this bit.                                                                                             | Shadow MRUnmask. The DMC_MSK.MR bit masks or unmasks writes to the MRregister in the SDRAM. When masked, writes to this register instead go to the DMC_MR register. When un- masked, the DMCwrites the DMC_MR value to the MRregister in the SDRAM. After completing the write, the DMCclears this bit.                                                                                             |
| 8 (R/W)            | MR         | 0                                                                                                                                                                                                                                                                                                                                                                                                   | Mask (Disable) Write toMR                                                                                                                                                                                                                                                                                                                                                                           |
| 8 (R/W)            | MR         | 1                                                                                                                                                                                                                                                                                                                                                                                                   | Unmask (Enable) Write toMR                                                                                                                                                                                                                                                                                                                                                                          |

## Priority ID Register 1

The DMC\_PRIO register allows transactions from selected masters that generate specific SCB IDs to obtain higher priority than the transactions proceeding in the usual fashion. The contents of the register are masked with the contents of the DMC\_PRIOMSK register to obtain a single SCB ID or a range of IDs that get elevated priority.

Figure 10-16: DMC\_PRIO Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000015_d6f7a195a50eaa23c305a93c17bae4a40114e91a87100e718d1c2514cf851b11.png)

Table 10-25: DMC\_PRIO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                  |
|--------------------|------------|------------------------------------------|
| 31:0               | ID1        | SCB ID1 that Requires Elevated Priority. |

## Priority ID Register 2

The DMC\_PRIO2 register is another register which allows transactions from selected masters that generate specific SCB IDs to obtain higher priority than the transactions proceeding in the usual fashion. The contents of the register are masked with the contents of the DMC\_PRIOMSK2 register to obtain a single SCB ID or a range of IDs that get elevated priority.

Figure 10-17: DMC\_PRIO2 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000016_b35fbed0e77fae7595a36c8e41ceb1f0fbf57d3413c206d572dd54fe3293b523.png)

Table 10-26: DMC\_PRIO2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                  |
|--------------------|------------|------------------------------------------|
| 31:0               | ID2        | SCB ID2 that Requires Elevated Priority. |
| (R/W)              |            |                                          |

## Priority ID Mask Register 1

The DMC\_PRIOMSK register masks the respective ID bits in the DMC\_PRIOMSK register. This masking provides for elevating the access priority of either a single ID or a range of IDs.

Figure 10-18: DMC\_PRIOMSK Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000017_81068c570c3ad19442483e404a0d0e4ec6f9d375c9cdf4c519768703ccdd8415.png)

Table 10-27: DMC\_PRIOMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | ID1MSK     | Mask for SCB ID1.         |
| (R/W)              |            |                           |

## Priority ID Mask Register 2

The DMC\_PRIOMSK2 register bits mask the respective ID bits in the DMC\_PRIO2 register. This masking provides for elevating the access priority of either a single ID or a range of IDs.

Figure 10-19: DMC\_PRIOMSK2 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000018_4d21eeaf8e28743a335500584a14df97a86fff3817fd81a295abb7c60b8bd4cc.png)

Table 10-28: DMC\_PRIOMSK2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | ID2MSK     | Mask for SCB ID2.         |
| (R/W)              |            |                           |

## DMC Read Data Buffer ID Register 1

The DMC\_RDDATABUFID1 register allows read transactions from selected masters to make use of DMC read data buffer. The contents of the register are masked with the contents of the DMC\_RDDATABUFMSK1 register to obtain a single SCB ID or a range of IDs that get elevated priority.

Figure 10-20: DMC\_RDDATABUFID1 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000019_4007a9ea3fdcecde384fe891e129da8fe8aa5b674b89cc35dec2afdf9c678de8.png)

Table 10-29: DMC\_RDDATABUFID1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration        |
|--------------------|------------|--------------------------------|
| 31:0               | VALUE      | Mask for Read Data Buffer ID1. |

## DMC Read Data Buffer ID Register 2

The DMC\_RDDATABUFID2 register allows read transactions from selected masters to make use of DMC read data buffer. The contents of the register are masked with the contents of the DMC\_RDDATABUFMSK2 register to obtain a single SCB ID or a range of IDs that get elevated priority.

Figure 10-21: DMC\_RDDATABUFID2 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000020_f246ecb4316ddf0090ba7f1faca9b8528d48f4c4668bacec60974e1c061dde7e.png)

Table 10-30: DMC\_RDDATABUFID2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration        |
|--------------------|------------|--------------------------------|
| 31:0               | VALUE      | Mask for Read Data Buffer ID2. |

## DMC Read Data Buffer Mask Register 1

The DMC\_RDDATABUFMSK1 register bits mask the respective ID bits in the DMC Priority Mask ID register.

Figure 10-22: DMC\_RDDATABUFMSK1 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000021_5d47df21e723f5105d0e90f2982d304927b54a8b479624e5104fc8b67eb5cc52.png)

Table 10-31: DMC\_RDDATABUFMSK1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration        |
|--------------------|------------|--------------------------------|
| 31:0               | VALUE      | Mask for Read Data Buffer ID1. |
| (R/W)              |            |                                |

## DMC Read Data Buffer Mask Register 2

The DMC\_RDDATABUFMSK2 register bits mask the respective ID bits in the DMC Priority Mask ID register.

Figure 10-23: DMC\_RDDATABUFMSK2 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000022_2f95495946071ef8ea9d12004ebfb6234b56de0bfa245f17d528481edb463469.png)

Table 10-32: DMC\_RDDATABUFMSK2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration        |
|--------------------|------------|--------------------------------|
| 31:0               | VALUE      | Mask for Read Data Buffer ID2. |
| (R/W)              |            |                                |

## Status Register

The DMC\_STAT register indicates status for modes selected with the DMC\_CTL register and indicates status DMC operations.

Figure 10-24: DMC\_STAT Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000023_256910e75d431f62aae04ca3f5f7da6facf21e3404f3e10beeb504c0e5c58137.png)

Table 10-33: DMC\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R/NW)          | ZQCLDONE   | ZQ Calibration Long Done. The DMC_STAT.ZQCLDONE bit checks if the ZQ calibration long sub routine is done.                                                                                                                                                                                      |
| 24 (R/NW)          | ZQCSDONE   | ZQ Calibration Short Done. The DMC_STAT.ZQCSDONE bit checks if the ZQ calibration short sub routine is done.                                                                                                                                                                                    |
| 23:20 (R/NW)       | PHYRDPHASE | PHY Read Phase. The DMC_STAT.PHYRDPHASE bits indicate the latency after which the DMCmay read from the PHY. Taking round trip delay into account, the DLL indicates the exact number of clock cycles after which the controller needs to read data. Values other than those shown are reserved. |

Table 10-33: DMC\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |                                                                                                                                                                                                                             | Description/Enumeration                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |            | 2                                                                                                                                                                                                                           | 2 Clock Cycles Latency                                                                                                                                                                                                      |
|                    |            | 3                                                                                                                                                                                                                           | 3 Clock Cycles Latency                                                                                                                                                                                                      |
|                    |            | 4                                                                                                                                                                                                                           | 4 Clock Cycles Latency                                                                                                                                                                                                      |
|                    |            | 5                                                                                                                                                                                                                           | 5 Clock Cycles Latency                                                                                                                                                                                                      |
|                    |            | 6                                                                                                                                                                                                                           | 6 Clock Cycles Latency                                                                                                                                                                                                      |
|                    |            | 7                                                                                                                                                                                                                           | 7Clock Cycles Latency                                                                                                                                                                                                       |
| 19:16 (R/NW)       | PENDREF    | Pending Refresh. The DMC_STAT.PENDREF bits indicate the number of pending auto-refresh com- mands whose value can be from "0000" to "0111". When the DMCis in low power DDR mode ( DMC_CTL.LPDDR =1), the maximum value for | Pending Refresh. The DMC_STAT.PENDREF bits indicate the number of pending auto-refresh com- mands whose value can be from "0000" to "0111". When the DMCis in low power DDR mode ( DMC_CTL.LPDDR =1), the maximum value for |
| 13 (R/NW)          | DLLCALDONE | DLL Calibration Done. The DMC_STAT.DLLCALDONE indicates that the PHY DLL calibration sequence is complete.                                                                                                                  | DLL Calibration Done. The DMC_STAT.DLLCALDONE indicates that the PHY DLL calibration sequence is complete.                                                                                                                  |
|                    |            | 0                                                                                                                                                                                                                           | No Status                                                                                                                                                                                                                   |
|                    |            | 1                                                                                                                                                                                                                           | Completed PHY DLL Calibration                                                                                                                                                                                               |
| 7                  | RESETDONE  | Reset Done.                                                                                                                                                                                                                 | Reset Done.                                                                                                                                                                                                                 |
|                    |            | 0                                                                                                                                                                                                                           | SDRAM Reset is ongoing                                                                                                                                                                                                      |
|                    |            | 1                                                                                                                                                                                                                           | SDRAM Reset is done                                                                                                                                                                                                         |
| 5 (R/NW)           | DPDACK     | Deep Power-Down Acknowledge. The DMC_STAT.DPDACK bit indicates that deep power-down mode is active. Note that this status is available in low power DDR mode ( DMC_CTL.LPDDR =1) only.                                      | Deep Power-Down Acknowledge. The DMC_STAT.DPDACK bit indicates that deep power-down mode is active. Note that this status is available in low power DDR mode ( DMC_CTL.LPDDR =1) only.                                      |
|                    |            | 0                                                                                                                                                                                                                           | Not in Deep Power-Down Mode                                                                                                                                                                                                 |
|                    |            | 1                                                                                                                                                                                                                           | Deep Power-Down Mode Active                                                                                                                                                                                                 |
| 4 (R/NW)           | PDACK      | Power-Down Acknowledge. The DMC_STAT.PDACK bit indicates that power-down mode is active.                                                                                                                                    | Power-Down Acknowledge. The DMC_STAT.PDACK bit indicates that power-down mode is active.                                                                                                                                    |
|                    |            | 0                                                                                                                                                                                                                           | Not in Power-Down Mode                                                                                                                                                                                                      |
|                    |            | 1                                                                                                                                                                                                                           | Power-Down Mode Active                                                                                                                                                                                                      |
| 3 (R/NW)           | SRACK      | Self-Refresh Acknowledge.                                                                                                                                                                                                   | that self-refresh mode is active.                                                                                                                                                                                           |
|                    |            | DMC_STAT.SRACK bit indicates                                                                                                                                                                                                | 0 Not in Self-Refresh Mode                                                                                                                                                                                                  |
|                    |            | 1                                                                                                                                                                                                                           | Self-Refresh Mode Active                                                                                                                                                                                                    |

Table 10-33: DMC\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/NW)           | INITDONE   | Initialization Done. The DMC_STAT.INITDONE bit indicates that the initialization sequence is com- plete. 0 No Status 1 Initialize Done |
| 0 (R/NW)           | IDLE       | Idle State. The DMC_STAT.IDLE bit indicates whether the DMCis idle or busy. 0 Busy 1 Idle                                              |

## Timing 0 Register

The DMC\_TR0 register selects timing parameters for DMC operation to corresponding with parameters of the SDRAM device that is used in the system. The timing registers must be programmed to match the device for correct operation of the SDRAM and must be programmed before initializing the SDRAM. Note that all values for bit fields in DMC\_TR0 are in increments of clock cycle time (t CK).

Figure 10-25: DMC\_TR0 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000024_324113897b29c8c0efe453953391c3439cb299dc41fdfa97abbbc096b4edb332.png)

Table 10-34: DMC\_TR0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:28 (R/W)        | TMRD       | Timing Mode Register Delay. The DMC_TR0.TMRD field selects the set-to-active timing parameter (t MRD ), which is the number of clock cycles that occur after the mode registers in the SDRAM are set and before the next command is issued.       |
| 25:20 (R/W)        | TRC        | Timing Row Cycle. The DMC_TR0.TRC field selects the active-to-active time (t RC ), which is the mini- mum number of clock cycles that occur from an active command to the next active command in the same bank.                                   |
| 16:12 (R/W)        | TRAS       | Timing Row Active Time. The DMC_TR0.TRAS field selects the active-to-precharge time (t RAS ), which is the number of clock cycles that occur from an active command until a precharge com- mand is allowed.                                       |
| 11:8 (R/W)         | TRP        | Timing RAS Precharge. The DMC_TR0.TRP field selects the precharge-to-active time (t RP ), which is the number of clock cycles that occur while the SDRAM recovers from a precharge com- mand and becomes ready to accept the next active command. |

Table 10-34: DMC\_TR0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:4 (R/W)          | TWTR       | Timing Write to Read. The DMC_TR0.TWTR field selects the write-to-read delay time (t WTR ), which is the number of clock cycles that occur from the last write data to the next read command. |
| 3:0 (R/W)          | TRCD       | Timing RAS to CAS Delay. The DMC_TR0.TRCD field selects the RAS to CAS delay time (t RCD ), which is the number of clock cycles that occur from an active command to a read/write assertion.  |

## Timing 1 Register

The DMC\_TR1 register selects timing parameters for DMC operation to corresponding with parameters of the SDRAM device that is used in the system. The timing registers must be programmed to match the device for correct operation of the SDRAM and must be programmed before initializing the SDRAM. Note that all values for bit fields in DMC\_TR1 are in increments of clock cycle time (t CK).

Figure 10-26: DMC\_TR1 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000025_4f30a202834ad3312876470a58bb4c5649660b734a1e06652c64aca352d7ee55.png)

Table 10-35: DMC\_TR1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30:28 (R/W)        | TRRD       | Timing Read-Read Delay. The DMC_TR1.TRRD field selects the active-to-active time (t RRD ), which is the mini- mum number of clock cycles occurring from a bank x active command to a bank y active command.                                                                                                                                                                                                                                                                                                                                             |
| 23:16 (R/W)        | TRFC       | Timing Refresh-to-Command. The DMC_TR1.TRFC field selects the refresh-to-active command delay (t RFC ), which is the number of clock cycles required for the SDRAM to recover from a refresh signal to be ready to take the next command. It is also the number of clock cycles needed for the SDRAM to recover from executing one active command and ready to accept the next active command.                                                                                                                                                          |
| 13:0 (R/W)         | TREF       | Timing Refresh Interval. The DMC_TR1.TREF field selects the refresh interval time (t REF ), which is the num- ber of clock cycles occurring from one refresh command to the next refresh command. The actual timing of issuing a precharge command may be delayed by if the SDRAM is processing a normal access. However, the delay is not accumulative so there is no need to shorten the refresh interval to account for the memory access time. The non- accumulative refresh delay typically increases memory bandwidth by a few percentage points. |

## Timing 2 Register

The DMC\_TR2 register selects timing parameters for DMC operation to corresponding with parameters of the SDRAM device that is used in the system. The timing registers must be programmed to match the device for correct operation of the SDRAM and before initializing the SDRAM.

Note that all values for bit fields in DMC\_TR2 are in increments of clock cycle time (t CK).

Figure 10-27: DMC\_TR2 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000026_ec3b8c462d8f29c396c289a1833a36f045f2d29b4720c50afc52fe56519a0aa8.png)

Table 10-36: DMC\_TR2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:20 (R/W)        | TCKE       | Timing Clock Enable. The DMC_TR2.TCKE field selects the CKE minimum pulsewidth (t CKE ).                                                                                                                                                                                                                                        |
| 19:16 (R/W)        | TXP        | Timing Exit Power Down. The DMC_TR2.TXP field selects the exit power down to next valid command time (t XP ).                                                                                                                                                                                                                   |
| 15:12 (R/W)        | TWR        | Timing Write Recovery. The DMC_TR2.TWR field selects the write recovery time (t WR ). Note that this param- eter applies to LPDDR only.                                                                                                                                                                                         |
| 11:8 (R/W)         | TRTP       | Timing Read-to-Precharge. The DMC_TR2.TRTP bit field selects the internal read to precharge time (t RTP ) for DDR2/DDR3 modes. If the resulting value in DDR2/DDR3 mode is less than 2, program the bit field to 2. This value (2) is the minimum t RTP time for DDR2/DDR3 mode. For LPDDR mode program this bit field to zero. |
| 4:0 (R/W)          | TFAW       | Timing Four-Activated-Window. The DMC_TR2.TFAW field selects the four-banks-activated window time (t FAW ). No more than four SDRAM banks should be activated within this window.                                                                                                                                               |

## ADSP-SC58x DMC Register Descriptions

DMCPHY (DMC) contains the following registers.

Table 10-37: ADSP-SC58x DMC Register List

| Name            | Description                        |
|-----------------|------------------------------------|
| DMC_CAL_PADCTL0 | Calibration PAD Control 0 Register |
| DMC_CAL_PADCTL2 | Calibration PAD Control 2 Register |
| DMC_PHY_CTL0    | PHY Control 0 Register             |
| DMC_PHY_CTL1    | PHY Control 1 Register             |
| DMC_PHY_CTL2    | PHY Control 2 Register             |
| DMC_PHY_CTL3    | PHY Control 3 Register             |
| DMC_PHY_CTL4    | PHY Control 4 Register             |

## Calibration PAD Control 0 Register

The DMC\_CAL\_PADCTL0 register sets the pad calibration controls.

Figure 10-28: DMC\_CAL\_PADCTL0 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000027_881e54f3ef0787754839f855eabbd65ddc8022b6c05a75ce2dddde9764b4aeff.png)

Table 10-38: DMC\_CAL\_PADCTL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | RTTCALEN   | RTT Calibration Enable. The DMC_CAL_PADCTL0.RTTCALEN bit is set to 1 at reset. Programming this bit to 0 is not allowed.     |
| 30 (R/W)           | PDCALEN    | PULLDOWN Calibration Enable. The DMC_CAL_PADCTL0.PDCALEN bit is set to 1 at reset. Programming this bit to 0 is not allowed. |
| 29 (R/W)           | PUCALEN    | PULLUP Calibration Enable. The DMC_CAL_PADCTL0.PUCALEN bit is set to 1 at reset. Programming this bit to 0 is not allowed.   |
| 28 (R/W)           | CALSTRT    | Start New Calibration ( Hardware Cleared).                                                                                   |

## Calibration PAD Control 2 Register

The DMC\_CAL\_PADCTL2 register sets the pad calibration controls. The DMC pads can be auto-calibrated to the required driver impedance and the On Die Termination (ODT) value using the corresponding bits in this register. These values are translated by the auto calibration logic into a corresponding drive strength control inside the PHY and then routed to the PADS. Auto-calibration starts as soon as the DMC\_CAL\_PADCTL0.CALSTRT bit is programmed. The DCLK needs to be set at the required frequency before setting the DMC\_CAL\_PADCTL0.CALSTRT bit.

Figure 10-29: DMC\_CAL\_PADCTL2 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000028_732b71e67479bdf22557be58263c95906775b355abd1f1f7dc3e5e9ccd41df82.png)

Table 10-39: DMC\_CAL\_PADCTL2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:16 (R/W)        | IMPRTT     | Impedance RTT Value. Writing to the DMC_CAL_PADCTL2.IMPRTT bit field sets the required initializa- tion sequence to program the termination impedance for the data PADS and the DQS PADS.                                                                       |
| 15:8 (R/W)         | IMPWRDQ    | Impedance for DQ. The DMC_CAL_PADCTL2.IMPWRDQ bit field sets the drive impedance forDQ DQS CLK and DMpads. Data pads ( DDR_DQ[NN] ), DQS pads ( DDR_LDQS , / DDR_LDQS , DDR_UDQS , / DDR_UDQS ), Clock pads ( DDR_CK , / DDR_CK ),DM pads ( DDR_UDM , DDR_LDM ) |
| 7:0 (R/W)          | IMPWRAD    | Impedance for ADDR_CMD PADS. The DMC_CAL_PADCTL2.IMPWRAD bit field sets the desired drive for address pads ( DDR_A[NN] ), Command pads ( DDR_RAS , DDR_CAS , DDR_CKE , DDR_WE , DDR_CS[N] , DDR_ODT ).                                                          |

## PHY Control 0 Register

The DMC\_PHY\_CTL0 register controls programmable PHY features.

Figure 10-30: DMC\_PHY\_CTL0 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000029_21fe1417f34f93c213b3f8d5da3aa540db10935b9558303f31e85069dd7b34e9.png)

Table 10-40: DMC\_PHY\_CTL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | RESETDAT   | Reset Data Capture Logic. The DMC_PHY_CTL0.RESETDAT bit resets the data capture logic only, including P and Nbuffers. If Quickboot is used, this bit does not have any effect. The DMC_PHY_CTL0.RESETDAT bit is reset by the hardware. A read of this bit returns zero. |
| 11 (R/W)           | RESETDLL   | Reset DLL. The DMC_PHY_CTL0.RESETDLL bit resets DLL control logic only, including the 90 degree DQS shifters. If Quickboot is used, this bit does not have any effect. The DMC_PHY_CTL0.RESETDLL bit is reset by the hardware a read of this bit returns zero.          |

## PHY Control 1 Register

The DMC\_PHY\_CTL1 register controls programmable PHY features.

Figure 10-31: DMC\_PHY\_CTL1 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000030_ee5ca92013d47d282db2c8a2db78f880c9b4a06e46fe0342a6b5c07fd008d976.png)

Table 10-41: DMC\_PHY\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration     |
|--------------------|------------|-----------------------------|
| 19                 | BYPODTEN   | Bypass ODTEN for DQand DQS. |
| (R/W)              | BYPODTEN   | 0 Reserved                  |
| (R/W)              | BYPODTEN   | 1 Reserved                  |

## PHY Control 2 Register

The DMC\_PHY\_CTL2 register controls programmable PHY features. Program this register as per the programming guidelines for proper operation of the DMC.

Figure 10-32: DMC\_PHY\_CTL2 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000031_2669f75f02df923735b9a62f78bc9cef02310bb0acea282ab61080b820caf2d4.png)

Table 10-42: DMC\_PHY\_CTL2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | 32 Bit Value.             |
| (R/W)              |            |                           |

## PHY Control 3 Register

The DMC\_PHY\_CTL3 register controls programmable PHY features. Program this register as per the programming guidelines for proper operation of DMC.

Figure 10-33: DMC\_PHY\_CTL3 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000032_2669f75f02df923735b9a62f78bc9cef02310bb0acea282ab61080b820caf2d4.png)

Table 10-43: DMC\_PHY\_CTL3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | 32 Bit Value.             |
| (R/W)              |            |                           |

## PHY Control 4 Register

The DMC\_PHY\_CTL4 register controls programmable PHY features.

Figure 10-34: DMC\_PHY\_CTL4 Register Diagram

![Image](13_Dynamic_Memory_Controller_(DMC)_artifacts/image_000033_76a29896b35b08c38b058ea5089b617f7c68d91bf538f17cdce0820ebe9af62c.png)

Table 10-44: DMC\_PHY\_CTL4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | CLKDIS     | Clock Disable. The DMC_PHY_CTL4.CLKDIS bit enables and disables the DDR clock. 0 Enable Clock                                                                                |
| 1:0 (R/W)          | DDRMODE    | DDR Mode Select. The DMC_PHY_CTL4.DDRMODE bit field selects between the various DDR modes. Not all modes are available on all processors. 0 DDR3 Mode 1 DDR2 Mode 2 Reserved |