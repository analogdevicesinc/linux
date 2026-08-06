# Dynamic Memory Controller (DMC)

<!-- source: 012_Dynamic_Memory_Controller_DMC.pdf | original pages 371–464 -->

## 10   Dynamic Memory Controller (DMC)

The dynamic memory controller (DMC) provides a glueless interface between DDR3 SDRAMs and the system crossbar interface (SCB). The DMC enables execution of instructions from, as well as transfer of data to and from, DDR3 SDRAM respectively.

NOTE: The term DDR3 is referred to generically as DDR SDRAM in the rest of this chapter unless otherwise noted.

The DMC is partitioned in a manner that allows reconfiguration and maintainability. The memory access protocol state machine along with JEDEC standard specific logic is embedded in the protocol controller . An access and operation reordering mechanism is incorporated as an efficiency controller . An SCB bus responder interface is provided to interface with the on-chip interconnect. This interface results in an efficient completer implementation owing to its out-of-order transaction capabilities. The control and status registers present in the DMC can be accessed using the MMR access bus.

The DMC supports access to the external memory by core and DMA accesses.

## DMC Features

The DMC includes a protocol controller that supports:

- JESD79-3E compatible double data rate DDR3 SDRAM devices

The features of the dynamic memory controller are:

- Provides 16-bit data only interface to SDRAM devices
- Supports a single external rank (one chip select)
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
- Caching of SDRAM read data burst for specific controllers to reduce the latency for same burst accesses.

## The DDR3 features are:

- 512 Mb to 8 Gb device sizes
- Burst length BL = 8
- Support for additive latency
- Support for programmable (and ZQ calibration) ODT and drive impedance
- Support for read leveling
- Support for write leveling

## Feature Exclusions

The DMC exclusions are as follows:

For DDR3:

- 4-bit and 8-bit wide DDR3 DRAM memories are not supported
- Burst interleaved accesses are not supported
- Both burst chop-BC4 and BC4-on-the-fly are not supported
- Auto refresh pull-in is not supported
- DLL off mode is not supported

## DMC Functional Description

The dynamic memory controller consists of controller and target interfaces, a protocol controller, and an efficiency controller. The following sections describe the function of these interfaces and controllers.

## ADSP-2159x\_SC592\_SC594 DMC Register List

The Dynamic Memory Controller module (DMC) provides an interface to external double-data-rate SDRAM. This interface supports various DDR standards (see chapter descriptions). A set of registers governs DMC controller operations. For more information on DMC controller functionality, see the DMC Controller Register Descriptions.

Table 10-1: ADSP-2159x\_SC592\_SC594 DMC Register List

| Name                    | Description                                                  |
|-------------------------|--------------------------------------------------------------|
| DMC_CFG                 | Configuration Register                                       |
| DMC_CTL                 | Control Register                                             |
| DMC_DLLCTL              | DLL Control Register                                         |
| DMC_DT_CALIB_ADDR       | Data Calibration Address Register                            |
| DMC_DT_DATA_CALIB_DATA0 | Data Calibration Data 0 Register                             |
| DMC_DT_DATA_CALIB_DATA1 | Data Calibration Data 1 Register                             |
| DMC_EFFCTL              | Efficiency Control Register                                  |
| DMC_EMR3                | Shadow EMR3 Register                                         |
| DMC_MR                  | Shadow MR0 Register (DDR3)                                   |
| DMC_MR1                 | Shadow MR1 Register (DDR3)                                   |
| DMC_MR2                 | Shadow MR2 Register (DDR3)                                   |
| DMC_MSK                 | Mask (Mode Register Shadow) Register                         |
| DMC_PM_CTL              | DMCPerformance Control Register                              |
| DMC_PM_IDCYDQ_CNT       | Performance Monitor Idle Cycle DQCount Register              |
| DMC_PM_IDPG_CNT         | Performance Monitor Idle Page Count Register                 |
| DMC_PM_PGHT_CNT         | Performance Monitor Page Hit Count                           |
| DMC_PM_PGMS_CNT         | Performance Monitor Page Miss Count                          |
| DMC_PM_RD2WRTA_CNT      | Performance Monitor Read to Write Turn Around Count Register |
| DMC_PM_REF_CNT          | Performance Monitor Refresh Count Register                   |
| DMC_PM_WR2RDTA_CNT      | Performance Monitor Write to Read Turn Around Count Register |
| DMC_PRIO                | Priority ID Register 1                                       |
| DMC_PRIO2               | Priority ID Register 2                                       |
| DMC_PRIOMSK             | Priority ID Mask Register 1                                  |
| DMC_PRIOMSK2            | Priority ID Mask Register 2                                  |
| DMC_RDDATABUFID1        | DMCRead Data Buffer ID Register 1                            |
| DMC_RDDATABUFID2        | DMCRead Data Buffer ID Register 2                            |
| DMC_RDDATABUFMSK1       | DMCRead Data Buffer Mask Register 1                          |

Table 10-1: ADSP-2159x\_SC592\_SC594 DMC Register List (Continued)

| Name                   | Description                                       |
|------------------------|---------------------------------------------------|
| DMC_RDDATABUFMSK2      | DMCRead Data Buffer Mask Register 2               |
| DMC_RDD_BUFHT_CNT      | Performance Monitor Buffer Hit Count Register     |
| DMC_SCB_PM_BRLN_CNT    | SCB Performance AXI Transfer Count Register       |
| DMC_SCB_PM_BRSZ_CNT    | SCB Performance AXI Transfer Count Register       |
| DMC_SCB_PM_IDRDDCY_CNT | SCB Performance Monitor Idle Cycle Count Register |
| DMC_SCB_PM_IDWRDCY_CNT | SCB Performance Monitor Idle Cycle Count Register |
| DMC_STAT               | Status Register                                   |
| DMC_TR0                | Timing 0 Register                                 |
| DMC_TR1                | Timing 1 Register                                 |
| DMC_TR2                | Timing 2 Register                                 |

## ADSP-2159x\_SC592\_SC594 DMC Register List

DMCPHY (DMC) contains the following registers.

Table 10-2: ADSP-2159x\_SC592\_SC594 DMC Register List

| Name               | Description                        |
|--------------------|------------------------------------|
| DMC_DDR_CA_CTL     | DDR CA Lane Control Register       |
| DMC_DDR_LANE0_CTL0 | Data Lane 0 Control Register 0     |
| DMC_DDR_LANE0_CTL1 | Data Lane 0 Control Register 1     |
| DMC_DDR_LANE1_CTL0 | Data Lane 1 Control Register 0     |
| DMC_DDR_LANE1_CTL1 | Data Lane 1 Control Register 1     |
| DMC_DDR_ROOT_CTL   | DDR ROOT Module Control Register   |
| DMC_DDR_SCRATCH_2  | Scratch Register 2                 |
| DMC_DDR_SCRATCH_3  | Scratch Register 3                 |
| DMC_DDR_SCRATCH_4  | Scratch Register 4                 |
| DMC_DDR_SCRATCH_5  | Scratch Register 5                 |
| DMC_DDR_SCRATCH_6  | Scratch Register 6                 |
| DMC_DDR_SCRATCH_7  | Scratch Register 7                 |
| DMC_DDR_ZQ_CTL0    | DDR Calibration Control Register 0 |
| DMC_DDR_ZQ_CTL1    | DDR Calibration Control Register 1 |
| DMC_DDR_ZQ_CTL2    | DDR Calibration Control Register 2 |

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

## Same Controller Transaction Scheduling

The DMC also stores the ID of each transaction that it buffered. In most of the cases, the transactions related to a controller result in page hits from the locality of reference rule. The efficiency controller uses the ID information of the transactions while scheduling. When the page-based scheduling of the buffered transactions is complete, same controller transaction scheduling is triggered. If multiple transactions from a controller are received, the efficiency controller schedules the transactions back-to-back.

## DMC Read Data Buffer

The DMC read data buffer contains a data buffer and an address buffer. The depth of the data buffer is equal to the burst length that is programmed in SDRAM. The address buffer holds the corresponding SDRAM burst address. When an SDRAM write address from any controller matches an address in the DMC read data buffer, the DMC invalidates the related data in the read buffer. When the DMC\_RDDATABUFMSK1 or DMC\_RDDATABUFMSK2 register is programmed with a value other than zero, the DMC read data buffer operation is enabled. The set of controllers whose data is buffered and retrieved are programmed in the DMC\_RDDATABUFID1 or DMC\_RDDATABUFID2 registers. The DMC can use the DMC\_RDDATABUFMSK1 and DMC\_RDDATABUFMSK2

ID registers to select a set of controllers similar to the programming of the DMC\_PRIOMSK and DMC\_PRIOMSK2 registers.

See the SCB ID-Based Priority section for details.

## Closed Page Per Bank

The DMC\_EFFCTL register provides per-bank granularity for closing pages. The software can determine that most accesses to a given bank in memory always result in a missed page. In this case, set the PREC\_BANK bit corresponding to the required bank to close the row after every transfer. This proactive step can result in reduced thrashing and increases memory throughput.

## SCB ID-Based Priority

The primary goal of the dynamic memory controller is to improve sustainable memory system bandwidth so that the service time for the average request can be reduced. However, to service critical requests from any controller in the system, the DMC provides a mechanism to elevate priority of a given access. The DMC priority ID registers ( DMC\_PRIO and DMC\_PRIO2 ) can be programmed with up to two SCB IDs with elevated priority.

After every access in a snapshot, the command buffers are searched to determine whether an ID of a command matches with the ID programmed in the DMC\_PRIO and DMC\_PRIO2 registers. The priority SCB ID access is sent before the subsequent access in the snapshot if:

- A match occurs, and
- The direction of the access (for example write) is the same as the direction of the snapshot (write)

There is an alternative to providing priority to a specific SCB ID. If a number of IDs from the same controller require priority, program the DMC priority mask ID registers ( DMC\_PRIOMSK and DMC\_PRIOMSK2 ) so that the corresponding bits are 0. The DMC uses a combination of the DMC\_PRIO and DMC\_PRIO2 registers and the DMC\_PRIOMSK / DMC\_PRIOMSK2 registers to elevate the priority of a select few or all IDs that belong to a controller. By default, none of the IDs are prioritized. The following are a few possibilities:

- The DMC\_PRIOMSK field is set to 0x00000000. If a single ID (7234) needs priority, set the DMC\_PRIOMSK field to 0xFFFFFFFF and set the DMC\_PRIO field to 7234.
- If the DMC\_PRIOMSK field is set to 0xFFFFFFFE, the SCB IDs 7234 and 7235 are given priority.
- If the DMC\_PRIOMSK field is set to 0xFFFFFFFC, the SCB IDs 7234, 7235, 7236, and 7237 are given priority.
- If two transactions with priority, one read and the other a write, are outstanding, the priority transaction that does not change the direction of the DMC access gets priority. The other priority transaction is handled at the beginning of the next snapshot. For example, if a write snapshot is in-progress, the write priority transaction is sent. The read priority transaction is sent at the beginning of the next read snapshot.

NOTE: Use SCB ID-based priority judiciously because it can significantly reduce the throughput of the DMC.

## Delaying up to Eight Auto-Refresh Commands

The DMC uses this method to ensure that auto-refresh does not interfere with any critical data transfers. Up to eight auto-refresh commands can accumulate in the DMC. The exact number of auto-refresh commands can be programmed using the DMC\_EFFCTL.NUMREF bit.

After the first refresh command is accumulated, the DMC constantly looks for an opportunity to schedule a refresh command. When the SCB read and write command buffers become empty for the programmed number of clock cycles ( DMC\_EFFCTL.IDLECYC bit field), the accumulated number of refresh commands are sent back-to-back to the DRAM. (The empty state of the SCB command buffers implies that no access is outstanding.)

After every refresh, the SCB command buffers are checked to ensure that they stay empty. If the SCB command buffers are always full, once the programmed number of refresh commands accumulates, the refresh operation is elevated to urgent priority. One refresh command is sent immediately. After this process, the DMC continues to wait for an opportunity to send out refresh commands. If self-refresh mode is enabled, all pending refresh commands are given out only after that DMC enters self-refresh mode.

## Page and Bank Interleaving

Page and bank interleaving allow consecutive row accesses to fall into the same bank (bank interleaving) or into a different bank (page interleaving). The DMC uses bank interleaving by default ( DMC\_CTL.ADDRMODE bit =0). If the DMC\_CTL.ADDRMODE bit =1, the DMC uses page interleaving. Page misses in one addressing mode result in hits in the other addressing mode.

## System Crossbar Completer Interface

The DMC uses the system crossbar completer interface to move all data. The system crossbar interface accepts interleaved write transactions and sends out-of-order responses. The read and write interfaces consist of buffers for address, data, and control information transferred to or from the system crossbar bus.

The system crossbar interface transactions are sent to the SDRAM only after the SDRAM has been initialized. However, if transactions arrive before or during initialization, they accumulate in the system crossbar interface and are sent out to the protocol controller once the initialization completes.

To increase throughput, the system crossbar write-response is sent out as soon as the final DDR burst is scheduled for transfer into the SDRAM. However, if an auto-refresh is needed, the scheduled write data is sent only after the auto-refresh. A delay can occur. The delay is a maximum of 64 clock cycles from the moment the write response is sent on the SCB to the write operation of the data into SDRAM.

The system crossbar interface performs the following operations:

- Buffers read and write command requests from the system crossbar bus
- Processes the requests by converting them to protocol controller user-interface transfers
- Sends and receives data to or from the protocol controller
- Creates a suitable read/write response and sends read data back to the system crossbar bus

The system crossbar completer interface supports the following:

- All burst lengths (1-16)
- Incremental and wrap bursts
- Data transfer sizes of 8-bit, 16-bit, or 32-bit
- Arrival of write data before write address
- Generation of error responses which include:
- Any access to an unimplemented region of the external memory space
- Any access when the SDRAM is in self-refresh mode/power-down mode
- Any access when the direct command interface is in operation

## Read/Write Command and Data Buffers

The system crossbar interface consists of a four-deep read command buffer and a four-deep write command buffer. Up to four write commands and four read commands can be waiting for access to the SDRAM. The system crossbar write buffer is 32 deep. It can support write data interleaving of two. The system crossbar read buffer is 32 deep.

## Peripheral Bus Completer Interface

The peripheral bus completer interface connects the dynamic memory controller to the peripheral bus and provides a host controller with access to the registers. The peripheral bus completer interface supports the following features:

- Read and write word accesses
- 32-bit data bus

## Architectural Concepts

The following sections provide information on the architecture of the interface.

## Controller On Die Termination (ODT)

The controller ODT is enabled with the granularity of a byte lane. The description of this feature can be obtained in the description of the corresponding PHY registers. Controller ODT involves extra overhead in terms of power consumption during reads.

The DMC implements dynamic on die termination at processor pads. When controller ODT is enabled, the termination resistors in the pads are turned on when the controller reads data from the DRAM. These resistors are turned off when the controller writes to the DRAM.

## Mode Register Set and Extended Mode Register Set Command

The load mode register command initializes the SDRAM operation parameters. The DMC supports the mode register set and extended mode register set commands. The controller automatically issues the mode register set command during power-on initialization and also when the DMC\_MR register is written with the DMC\_MSK.MR bit. The mode register set command is sent after the ongoing data transfer completes.

## DDR3 Reset Functionality

DDR3 contains an additional pin corresponding to reset functionality. Reset is part of the initialization sequence, but it can be performed asynchronously when needed. The reset procedure is similar to the steps involved in the initialization except the initial part of power-up.

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
| 1 Gb         | 26:24               | 23:11              | 10:1                  |
| 2 Gb         | 27:25               | 24:11              | 10:1                  |
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

## DMC Clocking

The DMC uses a divided-down version of the PLLCLK (PLL clock) to generate an internal clock for clocking the DMC block and interface. The specific value of the DCLK frequency is programmed in the CGU\_DIV register.

For information on the maximum clock frequency supported for specific modes, refer to the processor data sheet.

NOTE: For details on DMC clocking, see the Clock Generation Unit (CGU) and Clock Generation Unit (CGU) chapters.

## DMC DMA

The DMC supports DMA-based transfers to and from external DDR SDRAM memory and internal memory.

The DMC DMA controller, part of the distributed DMA engines (DDE) that are dispersed through the infrastructure, connects to the system crossbar fabric.

The DMC uses two DDEs for memory-to-memory DMA (MDMA). One channel is the source channel, and the second, the destination channel.

DMA transfers on the processor are descriptor-based or register-based. Register-based DMA allows the processor to program DMA control registers directly to initiate a DMA transfer. On completion, the control registers can be automatically updated with their original setup values for continuous transfer, if needed. Descriptor-based DMA transfers require a set of parameters stored within memory to initiate a DMA sequence. This transfer allows the chaining together of multiple DMA sequences. In descriptor-based DMA operations, a DMA channel can be programmed to set up and start another DMA transfer automatically after the current sequence completes.

Enhanced DMA operations (such as delay line DMA, scatter or gather DMA) are also supported to or from the DMC module.

## DMC Operating Modes

## DDR3 Mode

The DMC module supports JESD79-3E compatible double data rate DDR3 SDRAM. To configure this mode of operation, first set (=1) the DMC\_CTL.DDR3EN bit

## Self-Refresh Mode

For low-power consumption, the SDRAM can be put in self-refresh mode. When no data activity occurs, the DMC can put the SDRAM in self refresh to save power. The DMC\_STAT.IDLE bit indicates the activity on the DMC. If this bit is set, there is no activity in the DMC.

Enable self-refresh mode by writing the DMC\_CTL.SRREQ bit. The DMC stays in a self-refresh state when this bit is asserted. The DMC\_STAT.SRACK bit indicates when the SDRAM enters self-refresh mode.

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

## PHY DLL Calibration

The PHY DLL calibration is performed as part of the SDRAM power-up initialization. It calibrates data against the DQS and CLK signal. However, running DLL calibration after self-refresh or at an arbitrary time is required in certain cases.

The DMC allows PHY DLL calibration to start by setting the DMC\_CTL.DLLCAL bit. The DMC\_STAT.DLLCALDONE bit can be used to monitor the progress of the calibration. Once calibration is over, this bit is set. Once the calibration procedure is started by writing to the DMC\_CAL\_PADCTL0.CALSTRT bit, the full calibration takes 300 DCLK cycles to complete.

NOTE: DLL calibration can be initiated only when the DMC is idle ( DMC\_STAT.IDLE = 1).

## DDR3 ZQ Calibration Short CMD

The ZQ calibration short command is generally used to correct small variations in ZQ (~0.5%). To perform ZQ calibration, the controller is first checked for its idle state. Once the idle bit is obtained, a ZQCS command can be

issued by setting the DMC\_STAT.ZQCSDONE bit (0x0004). The DMC\_STAT.ZQCSDONE bit (0x0008) can be used to monitor the calibration sequence. When this bit is 0, it indicates that the calibration is ongoing. When it is 1, it indicates that the calibration is done. As an example, a GP timer can be used to periodically trigger a ZQCS command to address tiny variations.

- NOTE: The ZQ calibration function is essential for normal operation of DDR3. With the reference of the external resistance (240 Ω ± 1%) connected to the DMC\_RZQ pin, DDR3 calibrates the R on  and R tt  values of the ZQ pin against temperature and voltage variations.

## DDR3 ZQ Calibration Long CMD

Several DDR3 impedance calibrations are implemented for optimal signal integrity. The long ZQ calibration is used after power-up and the short ZQ calibration is used periodically during normal operation to compensate for voltage and temperature drift. These calibration sequences improve connectivity between the SDRAM pads and the PCB trace. The DMC\_RZQ pin on the SDRAM is connected to an external precision resistor that adjusts the output driver impedance Rtt and ODT values to match the trace impedance. The connection reduces impedance discontinuity and minimizes signal reflections.

The command has two variants named as ZQ calibration long (ZQCL) and ZQ calibration short (ZQCS). The ZQCL command is issued during initialization and after self-refresh exit command. It can be issued later depending on the system environment.

The DMC pads can be auto calibrated to the required driver impedance Rtt using an external resistance RZQ and the On Die Termination (ODT) value using the corresponding bits ( DMC\_CAL\_PADCTL2 ). The autocalibration logic translates these values into a corresponding drive strength control inside the PHY and then routed to the PADS. Autocalibration starts as soon as the bit is programmed (set the DCLK at the required frequency before setting this DMC\_CAL\_PADCTL0.CALSTRT bit). Autocalibration expects the program to select two different member sets of pads (address/command pads versus CLK/Data/DQS/DM pads).

## On Die Termination ()

The DMC supports dynamic On Die Termination (ODT) at the pads. When the controller ODT is set, the termination resistors in the pads are turned on when the controller reads data from the DRAM. These resistors are turned off when the controller is writing to the DRAM. Controller ODT is enabled with the granularity of a byte lane. The description of this feature can be obtained in the description of the corresponding PHY registers.

DDR3 SDRAM provides extended ODT mode.

- Asynchronous ODT : ODT timing in the slow exit power-down mode
- Dynamic ODT : Function that can dynamically switch the ODT resistance during a write operation without an MRS command. It improves signal quality during a write operation.

## Leveling Techniques

The DMC controller supports read and write leveling.

## Write Leveling

It is difficult to meet the specifications with the on-board parasitic, trace length variability along with high frequency. The DMC controller supports leveling to compensate the skews between clock and strobe. This skew compensation is done independently for every byte lane. Write leveling is performed by the DDR3 memory controller as a part of initialization by programming the DMC\_MR1.WL bit.

To dynamically perform write leveling:

1. Ensure that the DMC is idle.
2. Program the DMC\_MR1.WL bit.
3. Enable the DMC\_MSK register for the DMC\_MR1 register.
4. Wait for 2000 DMC clock cycles to complete write leveling.

## Read Leveling

Read leveling is useful for calibrating read data timing. It is compensated for imbalanced loading on read path. Read leveling can be performed on LSB or byte lane. It is performed as a part of initialization sequence by setting the DMC\_EMR3.MPR bit.

To dynamically perform read leveling:

1. Ensure that the DMC is idle.
2. Program the DMC\_EMR3.MPR bit.
3. Enable DMC mask register DMC\_EMR3 register.
4. Wait for 2000 DMC clock cycles to complete read leveling.

## Read Leveling during DQS Strobe Gating

To ensure robust read timing, read leveling must be done individually for all data bits. Some DDR3 memory devices do not drive read leveling patterns on all data pins. The devices send pattern only information on the LSBs of each byte lane. In these cases, the DMC\_EMR3 register cannot be used for read leveling. Instead, use the DMC\_CTL.RL\_DQS bit for read leveling.

Read leveling during DQS strobe gating is dynamically supported except during initialization.

## Initializing the DMC

Complete the following procedures to initialize the DMC module.

- Resetting the DMC Lane
- Performing ZQ Calibration
- Programming the DMC Controller

## Resetting the DMC Lane

If there is a change in the DMC clock frequency, the DDR lane must be reset using the following procedure. Once the lane is reset, wait 9000 DDR clock cycles for the DDR DLL to lock before restarting the DDR.

```
1. Set the DMC_DDR_LANE0_CTL0.CB_RSTDLL and DMC_DDR_LANE1_CTL0.CB_RSTDLL bits. 2. Change the DMC clock frequency. 3. Clear the DMC_DDR_LANE0_CTL0.CB_RSTDLL and DMC_DDR_LANE1_CTL0.CB_RSTDLL bits.
```

## Performing ZQ Calibration

Perform the following ZQ calibration procedure for the proper impedance matching.

1. Program the DMC\_DDR\_ZQ\_CTL0.IMPWRADD bit field with the drive strength of the address and command signals. A 100 Ω drive strength is recommended for the drive strength of address and command signals. When programming a drive strength of 100 Ω , write 0x64 into this bit field.
2. Program the DMC\_DDR\_ZQ\_CTL0.IMPWRDQ bit field with the drive strength of the DQ, DQS, DM and clock signals. A 100 Ω drive strength is recommended for the drive strength of address and command signals. When programming a drive strength of 100 Ω , write 0x64 into this bit field.
3. Program the DMC\_DDR\_ZQ\_CTL0.IMPRTT bit field with the adjusted On Die Termination (ODT) for the Data and DQS signals for the read operation. Due to a correction factor, program this field to 80% of the equivalent ODT.

```
DMC_DDR_ZQ_CTL0.IMPRTT value = ODT*2*0.8
```

For example, if a 50 Ω terminating resistance is required on the data pads to match the trace impedance to the board impedance, there will be two 50 Ω resistance data pads in parallel. The value is programmed to 100 × 0.8 = 80.

4. Write 0 to the DMC\_DDR\_ZQ\_CTL1 register.
5. Program 0x70000000 to the DMC\_DDR\_ZQ\_CTL2 register.
6. Program 0x0000000 to the DMC\_DDR\_CA\_CTL register.
7. Program 0x0000000 to the DMC\_DDR\_ROOT\_CTL register.
8. Program 0x00010000 to the DMC\_DDR\_ROOT\_CTL register.
9. Wait for 8000 DMC clock cycles.
10. Program 0x0C000001 to the DMC\_DDR\_CA\_CTL register.
11. Wait for 8000 DMC clock cycles.
12. Program 0x0000000 to the DMC\_DDR\_CA\_CTL register.
13. Program 0x0000000 to the DMC\_DDR\_ROOT\_CTL register.

## Programming Duty Cycles

Perform the following procedure to program the DQS duty cycle and clock duty cycle trim.

1. Program the DMC\_DDR\_LANE0\_CTL0 and DMC\_DDR\_LANE1\_CTL0 registers with DMC\_DDR\_LANE0\_CTL0.BYPCODE / DMC\_DDR\_LANE0\_CTL1.BYPCODE , DMC\_DDR\_LANE1\_CTL0.BYPSELP / DMC\_DDR\_LANE0\_CTL1.BYPCODE and DMC\_DDR\_LANE0\_CTL0.BYPENB / DMC\_DDR\_LANE1\_CTL0.BYPENB to trim the DQS duty cycle adjustment.
2. Program DMC\_DDR\_CA\_CTL registers with DMC\_DDR\_CA\_CTL.BYPCODE0 , DMC\_DDR\_CA\_CTL.BYPCODE1 , DMC\_DDR\_CA\_CTL.BYPSELP and DMC\_DDR\_CA\_CTL.BYPENB to trim the clock duty cycle adjustment.

## Programming the DMC Controller

Perform the following procedure to program the DMC controller.

1. Program the DMC\_CFG register with the size of the DDR memory device. Program the DMC\_CFG.IFWID and DMC\_CFG.SDRWID bit fields to 16-bits wide.
2. Program the DMC\_TR0 , DMC\_TR1 , DMC\_TR2 registers with the timing parameters as required by the DDR memory device.
3. Program the DMC\_MR and DMC\_MR1 registers with the appropriate values. A memory ODT of 120 Ω is recommended. Write leveling is recommended for a 666 MHz operating frequency.
4. Program the DMC\_MR2 register. If read leveling is required, program the DMC\_EMR3 register. Read leveling is recommended for a 666 MHz operating frequency.
5. Program the DMC\_DLLCTL register. Program the DMC\_DLLCTL.DATACYC bit field with 15ns/ddr clock period in ns. Program the DMC\_DLLCTL.DLLCALRDCNT bit field with 240.
6. Wait for 2000 DMC clock cycles.
7. Set the DMC\_DDR\_CA\_CTL.SW\_REFRESH bit, keeping the other bits unchanged.
8. Wait for five DMC clock cycles.
9. Set the DMC\_DDR\_ROOT\_CTL.SW\_REFRESH bit. Program the DMC\_DDR\_ROOT\_CTL.PIPE\_OFSTDCYCLE bit field with 2.
10. Program the DMC\_CTL register.
- DMC\_CTL.RDTOWR = 5
- Set the DMC\_CTL.DDR3EN bit.
- Set DMC\_CTL.AL\_EN if the operating frequency is above 667 MHz.

- Set DMC\_CTL.RL\_DQS if operating frequency above 667 MHz and to enable read leveling during data strobing.
- Set the DMC\_CTL.INIT bit.
11. If write leveling is enabled, wait for 600 DMC clock cycles or until the DMC\_MR1.WL bit auto clears.
12. If read leveling is enabled wait for 2000 DMC clock cycles or until the DMC\_EMR3.MPR bit auto clears.
13. If read leveling during data strobe gating is enabled, wait for 600 DMC clock cycles or until the DMC\_CTL.RL\_DQS bit auto clears.
14. Wait for 722000 DMC clock cycles to complete DMC initialization or until the DMC\_STAT.INITDONE bit is set.
15. Set the DMC\_DDR\_LANE0\_CTL1.COMP\_DCYCLE and DMC\_DDR\_LANE1\_CTL1.COMP\_DCYCLE bits, without affecting other bits.
16. Wait for 10 DMC clock cycles.
17. Clear the DMC\_DDR\_LANE0\_CTL1.COMP\_DCYCLE and DMC\_DDR\_LANE1\_CTL1.COMP\_DCYCLE bits, without affecting other bits.
18. Set and then clear the DMC\_DDR\_LANE0\_CTL0.CB\_RSTDAT bit, without affecting other bits.
19. Set and then clear DMC\_DDR\_LANE1\_CTL0.CB\_RSTDAT bit, without affecting other bits.
20. Wait for 2500 DMC clock cycles.
21. Read the DMC\_STAT.PHYRDPHASE bit field. Program this value into the DMC\_DLLCTL.DATACYC bit field, without affecting other bits in the DMC\_DLLCTL register.
22. Reprogram the DMC\_CTL.DDR3EN and DMC\_CTL.RDTOWR bits. If the operation frequency is above 667 MHz set the DMC\_CTL.AL\_EN bit.

## Programming DQ Delay Trim

To trim or fine tune the delay on the data lane, the write leveling code should be read first from the data lanes. The write leveling code can be incremented to increase setup time or decremented to increase hold time before writing it back to the data lanes.

For lane 0, complete the following steps:

1. Write 0x000000D0 to the DMC\_DDR\_LANE0\_CTL1 register without changing other bits.
2. Wait 2500 DMC clock cycles.
3. Write 0x00400000 to the DMC\_DDR\_ROOT\_CTL register.
4. Wait 2500 DMC clock cycles.
5. Write 0x00000000 to the DMC\_DDR\_ROOT\_CTL register.

```
6. Bits [20:16] of the DMC_DDR_SCRATCH_4 holds the write leveling code for lane 0.
```

```
7. Increment or decrement the write leveling code as needed. Write it to the
```

- DMC\_DDR\_LANE0\_CTL1.BYPCODE bit. Set the
8. Wait 2500 DMC clock cycles.

For lane 1, complete the following steps:

1. Write 0x000000D0 to the DMC\_DDR\_LANE1\_CTL1 register without changing other bits.
2. Wait 2500 DMC clock cycles.
3. Write 0x00800000 to the DMC\_DDR\_ROOT\_CTL register.
4. Wait 2500 DMC clock cycles.
5. Write 0x00000000 to the DMC\_DDR\_ROOT\_CTL register.
6. Bits [20:16] of the DMC\_DDR\_SCRATCH\_5 holds the write leveling code for lane 1.
7. Increment or decrement the write leveling code as needed. Write it to the DMC\_DDR\_LANE1\_CTL1.BYPCODE bit. Set the DMC\_DDR\_LANE1\_CTL1.BYPDELCHAINEN
8. Wait 2500 DMC clock cycles.

## DMC Performance Monitor (DMCPM)

The dynamic memory controller includes performance monitors to obtain the performance index for SDRAM and SCB fabric. These counters can be used to analyze the DMC access patterns and bus utilization (both DMC and SCB bus) at a system level. The DMCPM provides seven DMC performance counters and five SCB performance counters. The DMCPM has 32-bit counters that (when enabled) increment based on events.

To enable performance monitoring, set the DMC\_PM\_CTL.PMCNT bit = 1.

## ADSP-2159x\_SC592\_SC594 DMC Register Descriptions

Dynamic Memory Controller (DMC) contains the following registers.

Table 10-5: ADSP-2159x\_SC592\_SC594 DMC Register List

| Name                    | Description                       |
|-------------------------|-----------------------------------|
| DMC_CFG                 | Configuration Register            |
| DMC_CTL                 | Control Register                  |
| DMC_DLLCTL              | DLL Control Register              |
| DMC_DT_CALIB_ADDR       | Data Calibration Address Register |
| DMC_DT_DATA_CALIB_DATA0 | Data Calibration Data 0 Register  |

- DMC\_DDR\_LANE0\_CTL1.BYPDELCHAINEN

Table 10-5: ADSP-2159x\_SC592\_SC594 DMC Register List (Continued)

| Name                    | Description                                                  |
|-------------------------|--------------------------------------------------------------|
| DMC_DT_DATA_CALIB_DATA1 | Data Calibration Data 1 Register                             |
| DMC_EFFCTL              | Efficiency Control Register                                  |
| DMC_EMR3                | Shadow EMR3 Register                                         |
| DMC_MR                  | Shadow MR0 Register (DDR3)                                   |
| DMC_MR1                 | Shadow MR1 Register (DDR3)                                   |
| DMC_MR2                 | Shadow MR2 Register (DDR3)                                   |
| DMC_MSK                 | Mask (Mode Register Shadow) Register                         |
| DMC_PM_CTL              | DMCPerformance Control Register                              |
| DMC_PM_IDCYDQ_CNT       | Performance Monitor Idle Cycle DQCount Register              |
| DMC_PM_IDPG_CNT         | Performance Monitor Idle Page Count Register                 |
| DMC_PM_PGHT_CNT         | Performance Monitor Page Hit Count                           |
| DMC_PM_PGMS_CNT         | Performance Monitor Page Miss Count                          |
| DMC_PM_RD2WRTA_CNT      | Performance Monitor Read to Write Turn Around Count Register |
| DMC_PM_REF_CNT          | Performance Monitor Refresh Count Register                   |
| DMC_PM_WR2RDTA_CNT      | Performance Monitor Write to Read Turn Around Count Register |
| DMC_PRIO                | Priority ID Register 1                                       |
| DMC_PRIO2               | Priority ID Register 2                                       |
| DMC_PRIOMSK             | Priority ID Mask Register 1                                  |
| DMC_PRIOMSK2            | Priority ID Mask Register 2                                  |
| DMC_RDDATABUFID1        | DMCRead Data Buffer ID Register 1                            |
| DMC_RDDATABUFID2        | DMCRead Data Buffer ID Register 2                            |
| DMC_RDDATABUFMSK1       | DMCRead Data Buffer Mask Register 1                          |
| DMC_RDDATABUFMSK2       | DMCRead Data Buffer Mask Register 2                          |
| DMC_RDD_BUFHT_CNT       | Performance Monitor Buffer Hit Count Register                |
| DMC_SCB_PM_BRLN_CNT     | SCB Performance AXI Transfer Count Register                  |
| DMC_SCB_PM_BRSZ_CNT     | SCB Performance AXI Transfer Count Register                  |
| DMC_SCB_PM_IDRDDCY_CNT  | SCB Performance Monitor Idle Cycle Count Register            |
| DMC_SCB_PM_IDWRDCY_CNT  | SCB Performance Monitor Idle Cycle Count Register            |
| DMC_STAT                | Status Register                                              |
| DMC_TR0                 | Timing 0 Register                                            |
| DMC_TR1                 | Timing 1 Register                                            |

Table 10-5: ADSP-2159x\_SC592\_SC594 DMC Register List (Continued)

| Name    | Description       |
|---------|-------------------|
| DMC_TR2 | Timing 2 Register |

## Configuration Register

The DMC\_CFG register selects SDRAM device specific parameters and selects the SDRAM interface width.

Figure 10-1: DMC\_CFG Register Diagram

<!-- image -->

Table 10-6: DMC\_CFG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                       | Description/Enumeration                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:12 (R/W)        | EXTBANK    | External Banks. The DMC_CFG.EXTBANK bits select the number of external banks connected to the DMC. Note that all values other than those shown are reserved.  | External Banks. The DMC_CFG.EXTBANK bits select the number of external banks connected to the DMC. Note that all values other than those shown are reserved.  |
| 15:12 (R/W)        | EXTBANK    | 0                                                                                                                                                             | 1 External Bank                                                                                                                                               |
| 15:12 (R/W)        | EXTBANK    | 1-15                                                                                                                                                          | Reserved                                                                                                                                                      |
| 11:8 (R/W)         | SDRSIZE    | SDRAM Size. The DMC_CFG.SDRSIZE bits select the size of individual SDRAM connected to the DMC. Note that all values other than those shown are reserved.      | SDRAM Size. The DMC_CFG.SDRSIZE bits select the size of individual SDRAM connected to the DMC. Note that all values other than those shown are reserved.      |
| 11:8 (R/W)         | SDRSIZE    | 3                                                                                                                                                             | 512M Bit SDRAM                                                                                                                                                |
| 11:8 (R/W)         | SDRSIZE    | 4                                                                                                                                                             | 1G Bit SDRAM                                                                                                                                                  |
| 11:8 (R/W)         | SDRSIZE    | 5                                                                                                                                                             | 2G Bit SDRAM                                                                                                                                                  |
| 11:8 (R/W)         | SDRSIZE    | 6                                                                                                                                                             | 4G Bit SDRAM                                                                                                                                                  |
| 11:8 (R/W)         | SDRSIZE    | 7                                                                                                                                                             | 8G Bit SDRAM                                                                                                                                                  |
| 7:4 (R/W)          | SDRWID     | SDRAM Width. The DMC_CFG.SDRWID bits select the width of the individual SDRAM connected to the DMC. Note that all values other than those shown are reserved. | SDRAM Width. The DMC_CFG.SDRWID bits select the width of the individual SDRAM connected to the DMC. Note that all values other than those shown are reserved. |
| 7:4 (R/W)          | SDRWID     | 0-1                                                                                                                                                           | Reserved                                                                                                                                                      |
| 7:4 (R/W)          | SDRWID     | 2                                                                                                                                                             | 16-Bit Wide SDRAM                                                                                                                                             |
| 7:4 (R/W)          | SDRWID     | 3-15                                                                                                                                                          | Reserved                                                                                                                                                      |

Table 10-6: DMC\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                       | Description/Enumeration                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0                | IFWID      | Interface Width. The DMC_CFG.IFWID bits select the width of the interface between the DMCand SDRAM. Note that all values other than those shown are reserved. | Interface Width. The DMC_CFG.IFWID bits select the width of the interface between the DMCand SDRAM. Note that all values other than those shown are reserved. |
| (R/W)              |            | 0-1                                                                                                                                                           | Reserved                                                                                                                                                      |
|                    |            | 2                                                                                                                                                             | 16-Bit Wide Interface. All other values are reserved. This field specifies the interface width between the con- troller and the SDRAM.                        |
|                    |            | 3-15                                                                                                                                                          | Reserved                                                                                                                                                      |

## Control Register

The DMC\_CTL register controls DMC modes, DLL calibration, and DRAM initialization.

Figure 10-2: DMC\_CTL Register Diagram

<!-- image -->

Table 10-7: DMC\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------|
| 27 (R/W)           | AL_EN      | AL_EN. When set, enables 800MHz operation. 0 Disables greater than 667MHz operation 1 Enables greater than 667MHz operation              |
| 26 (R/W)           | RL_DQS     | RL_DQS. When set, enables read leveling during DQS gating training. This bit auto-clears on read leveling completion. 0 Enable 1 Disable |

Table 10-7: DMC\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R0/W)          | ZQCL       | ZQ Calibration Long. The DMC_CTL.ZQCL bit starts the ZQ calibration long sequence. Note that this bit always reads as 0.                                                                                                                                                                                                                                                                                                            |
| 25 (R0/W)          | ZQCL       | 0 No effect                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 25 (R0/W)          | ZQCL       | 1 Triggers ZQ calibration long sequence                                                                                                                                                                                                                                                                                                                                                                                             |
| 24 (R0/W)          | ZQCS       | ZQ Calibration Short. The DMC_CTL.ZQCS bit starts the ZQ calibration short sequence. Note that this bit always reads as 0.                                                                                                                                                                                                                                                                                                          |
| 24 (R0/W)          | ZQCS       | 0 No effect                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 24 (R0/W)          | ZQCS       | 1 Triggers ZQ calibration short sequence                                                                                                                                                                                                                                                                                                                                                                                            |
| 13 (R0/W)          | DLLCAL     | DLL Calibration Start. The DMC_CTL.DLLCAL bit starts the PHY DLL calibration sequence. Note that this bit always reads as 0.                                                                                                                                                                                                                                                                                                        |
| 13 (R0/W)          | DLLCAL     | 0 No effect                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 13 (R0/W)          | DLLCAL     | 1 Start PHY DLL calibration                                                                                                                                                                                                                                                                                                                                                                                                         |
| 12 (R/W)           | PPREF      | Postpone Refresh. The DMC_CTL.PPREF bit enables postponing the DMCs sending of auto-refresh commands. When enabled, the DMCaccumulates refresh commands. The DMC_EFFCTL.NUMREF field selects the number of refresh commands that the DMCmay accumulate. When disabled, the DMC_TR1.TREF field selects the interval for auto-refresh command distribution. A maximum of eight auto-refresh commands can be accumulated in DDR3 mode. |
| 12 (R/W)           | PPREF      | 0 Disable Postpone Refresh                                                                                                                                                                                                                                                                                                                                                                                                          |
| 12 (R/W)           | PPREF      | 1 Enable Postpone Refresh                                                                                                                                                                                                                                                                                                                                                                                                           |
| 11:9 (R/W)         | RDTOWR     | Read-to-Write Cycle. The DMC_CTL.RDTOWR bits select the number of cycles that the DMCadds when a write operation follows a read operation. For proper operation, it should be program- med with the value of 010.                                                                                                                                                                                                                   |
| 11:9 (R/W)         | RDTOWR     | 0 1 Cycle Added from JEDEC Spec Value                                                                                                                                                                                                                                                                                                                                                                                               |
| 11:9 (R/W)         | RDTOWR     | 1 2 Cycles Added from JEDEC Spec Value                                                                                                                                                                                                                                                                                                                                                                                              |
| 11:9 (R/W)         | RDTOWR     | 2 3 Cycles Added from JEDEC Spec Value                                                                                                                                                                                                                                                                                                                                                                                              |
| 11:9 (R/W)         | RDTOWR     | 3 4 Cycles Added from JEDEC Spec Value                                                                                                                                                                                                                                                                                                                                                                                              |
| 11:9 (R/W)         | RDTOWR     | 4 5 Cycles Added from JEDEC Spec Value                                                                                                                                                                                                                                                                                                                                                                                              |
| 11:9 (R/W)         | RDTOWR     | 5 6 Cycles Added from JEDEC Spec Value                                                                                                                                                                                                                                                                                                                                                                                              |

Table 10-7: DMC\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |                                                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |            | 6                                                                                                                                                                                                                                                                                                                                    | 7 Cycles Added from JEDEC Spec Value                                                                                                                                                                                                                                                                                                 |
|                    |            | 7                                                                                                                                                                                                                                                                                                                                    | 8 Cycles Added from JEDEC Spec Value                                                                                                                                                                                                                                                                                                 |
| 8 (R/W)            | ADDRMODE   | Addressing (Page/Bank) Mode. The DMC_CTL.ADDRMODE bit selects whether the DMCuses page or bank inter- leaving for addressing. When using page interleaving, the bank address bits follow the most significant column address bits. When using bank interleaving, the bank address bits follow the most significant row address bits. | Addressing (Page/Bank) Mode. The DMC_CTL.ADDRMODE bit selects whether the DMCuses page or bank inter- leaving for addressing. When using page interleaving, the bank address bits follow the most significant column address bits. When using bank interleaving, the bank address bits follow the most significant row address bits. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                    | Bank Interleaving                                                                                                                                                                                                                                                                                                                    |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                    | Page Interleaving                                                                                                                                                                                                                                                                                                                    |
| 7 (R/W)            | RESET      | Reset SDRAM. The DMC_CTL.RESET bit starts the reset sequence. Note that this bit always reads as 0.                                                                                                                                                                                                                                  | Reset SDRAM. The DMC_CTL.RESET bit starts the reset sequence. Note that this bit always reads as 0.                                                                                                                                                                                                                                  |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                    | No effect                                                                                                                                                                                                                                                                                                                            |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                    | Starts reset sequence                                                                                                                                                                                                                                                                                                                |
| 6                  | PREC       | Precharge.                                                                                                                                                                                                                                                                                                                           | Precharge.                                                                                                                                                                                                                                                                                                                           |
| (R/W)              |            | DMC_CTL.PREC bit enables precharge, When disabled, all accesses DMCneeds to close them.                                                                                                                                                                                                                                              | which closes DRAM rows result in the respective DRAM rows 0 No Effect                                                                                                                                                                                                                                                                |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                    | Enable Precharge                                                                                                                                                                                                                                                                                                                     |
| 4 (R/W)            | PDREQ      | Power Down Request. The DMC_CTL.PDREQ bit enables power-down mode. When the DMCis in power- down mode, any data accesses cause the DMCto generate a bus error. The DRAM remains in power-down mode as along as this bit is 1.                                                                                                        | Power Down Request. The DMC_CTL.PDREQ bit enables power-down mode. When the DMCis in power- down mode, any data accesses cause the DMCto generate a bus error. The DRAM remains in power-down mode as along as this bit is 1.                                                                                                        |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                    | Disable Power-Down                                                                                                                                                                                                                                                                                                                   |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                    | Enable Power-Down                                                                                                                                                                                                                                                                                                                    |
| 3 (R/W)            | SRREQ      | Self-Refresh Request. The DMC_CTL.SRREQ bit enables self-refresh mode. When the DMCis in self-re- fresh mode, any data accesses cause the DMCto generate a bus error. The DRAM re- mains in self-refresh mode as along as this bit is 1.                                                                                             | Self-Refresh Request. The DMC_CTL.SRREQ bit enables self-refresh mode. When the DMCis in self-re- fresh mode, any data accesses cause the DMCto generate a bus error. The DRAM re- mains in self-refresh mode as along as this bit is 1.                                                                                             |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                    | Disable Self-Refresh                                                                                                                                                                                                                                                                                                                 |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                    | Enable Self-Refresh                                                                                                                                                                                                                                                                                                                  |

Table 10-7: DMC\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R0/W)           | INIT       | Initialize DRAM Start. The DMC_CTL.INIT bit starts the power up DRAM initialization sequence and DLL calibration sequence. Note that this bit always reads as 0. 0 No Effect |
| 0 (R/W)            | DDR3EN     | DDR3 Mode. The DMC_CTL.DDR3EN bit selects whether the DMCoperates in DDR3 mode. 0 Reserved 1 Enable DDR3 mode                                                                |

## DLL Control Register

The DMC\_DLLCTL register holds the programmable parameters associated with the DLLs within the DMC PHY.

Figure 10-3: DMC\_DLLCTL Register Diagram

<!-- image -->

Table 10-8: DMC\_DLLCTL Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|-------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:8 (R/W)         | DATACYC     | Data Cycles. The DMC_DLLCTL.DATACYC bits select the latency after which the DMCreads da- ta from the PHY. This field must be written with the value indicated in the DMC_STAT.PHYRDPHASE field, or data corruption occurs on all SDRAM reads. 2 2 Clock Cycles Latency 3 3 Clock Cycles Latency 4 4 Clock Cycles Latency 5 5 Clock Cycles Latency 6 6 Clock Cycles Latency 7 7 Clock Cycles Latency 8 8 Clock Cycles Latency 9 9 Clock Cycles Latency 10 10 Clock Cycles Latency 11 11 Clock Cycles Latency |
|                    |             | 12 12 Clock Cycles Latency                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|                    |             | 13 13 Clock Cycles Latency                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|                    |             | 14 14 Clock Cycles Latency                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|                    |             | 15 15 Clock Cycles Latency                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 7:0 (R/W)          | DLLCALRDCNT | DLL Calibration RD Count. The DMC_DLLCTL.DLLCALRDCNT field selects the number of read operations that the PHY uses for DLL calibration.                                                                                                                                                                                                                                                                                                                                                                     |

## Data Calibration Address Register

The DMC\_DT\_CALIB\_ADDR register provides the address used for the data calibration for read and write. During the DMC PHY DLL calibration, a particular set of locations in the DRAM is written and a series of reads are performed back to back to calibrate the PHY. The DMC PHY needs prior information about the data that would be read during the PHY DLL calibration. The controller performs one burst write operation to the address programmed in DMC\_DT\_CALIB\_ADDR (0x0090).

Note: While the exact address chosen does not matter much during memory initialization, if calibration of the PHY is performed when the DRAM contains valid data, care needs to be taken to ensure that this address points to an unused address. Else, this operation will modify application data stored at the address selected.

Figure 10-4: DMC\_DT\_CALIB\_ADDR Register Diagram

<!-- image -->

Table 10-9: DMC\_DT\_CALIB\_ADDR Register Fields

| Bit No. (Access)   | Bit Name          | Description/Enumeration                                                                                                                                          |
|--------------------|-------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | DMC_DT_CALIB_ADDR | Data calibration address. The DMC_DT_CALIB_ADDR.DMC_DT_CALIB_ADDR bit field contains the ad- dress to be programmed for the data calibration for read and write. |

## Data Calibration Data 0 Register

The DMC\_DT\_DATA\_CALIB\_DATA0 register contains the first 32-bit data used for the write during the data calibration.

Figure 10-5: DMC\_DT\_DATA\_CALIB\_DATA0 Register Diagram

<!-- image -->

Table 10-10: DMC\_DT\_DATA\_CALIB\_DATA0 Register Fields

| Bit No. (Access)   | Bit Name                  | Description/Enumeration                                                                                                                                               |
|--------------------|---------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | DMC_DT_DATA_CAL- IB_DATA0 | Data Calibration Data 0. The DMC_DT_DATA_CALIB_DATA0.DMC_DT_DATA_CALIB_DATA0 bit field contains the first 32 bit data used for the write during the data calibration. |

## Data Calibration Data 1 Register

The DMC\_DT\_DATA\_CALIB\_DATA1 register contains the second 32-bit data used for the write during the data calibration.

Figure 10-6: DMC\_DT\_DATA\_CALIB\_DATA1 Register Diagram

<!-- image -->

Table 10-11: DMC\_DT\_DATA\_CALIB\_DATA1 Register Fields

| Bit No. (Access)   | Bit Name                  | Description/Enumeration                                                                                                                                                |
|--------------------|---------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | DMC_DT_DATA_CAL- IB_DATA1 | Data Calibration Data 1. The DMC_DT_DATA_CALIB_DATA1.DMC_DT_DATA_CALIB_DATA1 bit field contains the second 32 bit data used for the write during the data calibration. |

## Efficiency Control Register

The DMC\_EFFCTL register control DMC features that improve throughput efficiency. These include features such as auto-refresh management, precharge options, and write data options.

Figure 10-7: DMC\_EFFCTL Register Diagram

<!-- image -->

Table 10-12: DMC\_EFFCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:20 (R/W)        | IDLECYC    | Idle Cycle. The DMC_EFFCTL.IDLECYC bits select the number of cycles after which theDMC issues any accumulated auto-refresh commands if postpone refresh is enabled ( DMC_CTL.PPREF =1). When DMC_EFFCTL.IDLECYC is set to 0, the DMCig- nores the DMC_CTL.PPREF selection and does not accumulate/postpone periodic auto-refresh commands. Note 1: By default, accumulated auto-refresh commands are issued after counting four idle cycles. Note 2: This value is ignored if DMC_CTL.PPREF is not set. Note 3: Setting this value to 0000 overrides the "postpone refresh" feature and does | Idle Cycle. The DMC_EFFCTL.IDLECYC bits select the number of cycles after which theDMC issues any accumulated auto-refresh commands if postpone refresh is enabled ( DMC_CTL.PPREF =1). When DMC_EFFCTL.IDLECYC is set to 0, the DMCig- nores the DMC_CTL.PPREF selection and does not accumulate/postpone periodic auto-refresh commands. Note 1: By default, accumulated auto-refresh commands are issued after counting four idle cycles. Note 2: This value is ignored if DMC_CTL.PPREF is not set. Note 3: Setting this value to 0000 overrides the "postpone refresh" feature and does |
|                    |            | 0-15                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | 0 to 15 Idle Cycles to Postpone Refresh Commands                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |

Table 10-12: DMC\_EFFCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19:16 (R/W)        | NUMREF     | Number of Refresh Commands. The DMC_EFFCTL.NUMREF bits select the number of auto-refresh commands that the DMCcan accumulate if postpone refresh is enabled ( DMC_CTL.PPREF =1). In DDR3 mode, the DMCmay accumulate up to eight auto-refresh commands. Note 1: By default, accumulated auto-refresh commands are issued after counting four | Number of Refresh Commands. The DMC_EFFCTL.NUMREF bits select the number of auto-refresh commands that the DMCcan accumulate if postpone refresh is enabled ( DMC_CTL.PPREF =1). In DDR3 mode, the DMCmay accumulate up to eight auto-refresh commands. Note 1: By default, accumulated auto-refresh commands are issued after counting four |
| 19:16 (R/W)        | NUMREF     | 0                                                                                                                                                                                                                                                                                                                                            | No Refresh Commands Accumulate                                                                                                                                                                                                                                                                                                               |
| 19:16 (R/W)        | NUMREF     | 1                                                                                                                                                                                                                                                                                                                                            | 1 Refresh Command May Accumulate                                                                                                                                                                                                                                                                                                             |
| 19:16 (R/W)        | NUMREF     | 2                                                                                                                                                                                                                                                                                                                                            | 2 Refresh Commands May Accumulate                                                                                                                                                                                                                                                                                                            |
| 19:16 (R/W)        | NUMREF     | 3                                                                                                                                                                                                                                                                                                                                            | 3 Refresh Commands May Accumulate                                                                                                                                                                                                                                                                                                            |
| 19:16 (R/W)        | NUMREF     | 4                                                                                                                                                                                                                                                                                                                                            | 4 Refresh Commands May Accumulate                                                                                                                                                                                                                                                                                                            |
| 19:16 (R/W)        | NUMREF     | 5                                                                                                                                                                                                                                                                                                                                            | 5 Refresh Commands May Accumulate                                                                                                                                                                                                                                                                                                            |
| 19:16 (R/W)        | NUMREF     | 6                                                                                                                                                                                                                                                                                                                                            | 6 Refresh Commands May Accumulate                                                                                                                                                                                                                                                                                                            |
| 19:16 (R/W)        | NUMREF     | 7                                                                                                                                                                                                                                                                                                                                            | 7 Refresh Commands May Accumulate                                                                                                                                                                                                                                                                                                            |
| 19:16 (R/W)        | NUMREF     | 8                                                                                                                                                                                                                                                                                                                                            | 8 Refresh Commands May Accumulate                                                                                                                                                                                                                                                                                                            |
| 15 (R/W)           | PRECBANK7  | Precharge Bank 7. The DMC_EFFCTL.PRECBANK7 bit enables precharge (closes the page) of bank 7 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged.                                    | Precharge Bank 7. The DMC_EFFCTL.PRECBANK7 bit enables precharge (closes the page) of bank 7 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged.                                    |
| 15 (R/W)           | PRECBANK7  | 0                                                                                                                                                                                                                                                                                                                                            | Disable Precharge Bank 7                                                                                                                                                                                                                                                                                                                     |
| 15 (R/W)           | PRECBANK7  | 1                                                                                                                                                                                                                                                                                                                                            | Enable Precharge Bank 7                                                                                                                                                                                                                                                                                                                      |
| 14 (R/W)           | PRECBANK6  | Precharge Bank 6. The DMC_EFFCTL.PRECBANK6 bit enables precharge (closes the page) of bank 6 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If DMC_CTL.PREC =1) then all banks are precharged.                                      | Precharge Bank 6. The DMC_EFFCTL.PRECBANK6 bit enables precharge (closes the page) of bank 6 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If DMC_CTL.PREC =1) then all banks are precharged.                                      |
| 14 (R/W)           | PRECBANK6  | 0                                                                                                                                                                                                                                                                                                                                            | Disable Precharge Bank 6                                                                                                                                                                                                                                                                                                                     |
| 14 (R/W)           | PRECBANK6  | 1                                                                                                                                                                                                                                                                                                                                            | Enable Precharge Bank 6                                                                                                                                                                                                                                                                                                                      |

Table 10-12: DMC\_EFFCTL Register Fields (Continued)

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
| 10 (R/W)           | PRECBANK2  | Precharge Bank 2. The DMC_EFFCTL.PRECBANK2 bit enables precharge (closes the page) of bank 2 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1).                                                                                                                                      |
| 10 (R/W)           | PRECBANK2  | 0 Disable Precharge Bank 2                                                                                                                                                                                                                                                                                       |
| 10 (R/W)           | PRECBANK2  | 1 Enable Precharge Bank 2                                                                                                                                                                                                                                                                                        |
| 9 (R/W)            | PRECBANK1  | Precharge Bank 1. The DMC_EFFCTL.PRECBANK1 bit enables precharge (closes the page) of bank 1 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. Bank 1 |
| 9 (R/W)            | PRECBANK1  | 0 Disable Precharge                                                                                                                                                                                                                                                                                              |
| 9 (R/W)            | PRECBANK1  | 1 Enable Precharge Bank 1                                                                                                                                                                                                                                                                                        |

Table 10-12: DMC\_EFFCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | PRECBANK0  | Precharge Bank 0. The DMC_EFFCTL.PRECBANK0 bit enables precharge (closes the page) of bank 0 after each transfer if the DMCprecharge feature is enabled ( DMC_CTL.PREC =1). Note: The ( DMC_CTL.PREC ) takes precedence over value in this register. If ( DMC_CTL.PREC =1) then all banks are precharged. |
| 8 (R/W)            | PRECBANK0  | 0 Disable Precharge Bank 0                                                                                                                                                                                                                                                                                |
| 8 (R/W)            | PRECBANK0  | 1 Enable Precharge Bank 0                                                                                                                                                                                                                                                                                 |

## Shadow EMR3 Register

The DMC\_EMR3 register in the DMC shadows the EMR3 register in the SDRAM when the DMC is in DDR3 mode ( DMC\_CTL.DDR3EN =1). If unmasked by the corresponding bit in the shadow mask register ( DMC\_MSK.EMR3 =1), a write to DMC\_EMR3 triggers an extended 'mode register set' command on the memory interface. If masked, a write to DMC\_EMR3 only updates the register in the DMC, not the register in the SDRAM.

Figure 10-8: DMC\_EMR3 Register Diagram

<!-- image -->

Table 10-13: DMC\_EMR3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | MPR        | Multi Purpose Read Enable (Read Leveling). The DMC_EMR3.MPR bit enables high temperature self-refresh rate. |
| 2 (R/W)            | MPR        | 0 Disable                                                                                                   |
| 2 (R/W)            | MPR        | 1 Enable                                                                                                    |
| 1:0                | MPR_LOC    | Hardcoded to "00". Reads a pre-defined pattern on MPR read..                                                |
| (R/NW)             |            |                                                                                                             |

## Shadow MR0 Register (DDR3)

The DMC\_MR register in the DMC shadows the MR register in the SDRAM when the DMC is DDR3 mode ( DMC\_CTL.DDR3EN =1). If unmasked by the corresponding bit in the shadow mask register ( DMC\_MSK.MR =1), a write to DMC\_MR triggers a 'mode register set' command on the memory interface. If masked, a write to DMC\_MR only updates the register in the DMC, not the register in the SDRAM.

Figure 10-9: DMC\_MR Register Diagram

<!-- image -->

Table 10-14: DMC\_MR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                         | Description/Enumeration                                                                                                                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | PD         | Active Power Down Mode. The DMC_MR.PD bit selects the active power-down mode. Note that this parameter applies only for DDR3 mode. For more information about this mode, see the data sheet for the SDRAM being used in your system.                            | Active Power Down Mode. The DMC_MR.PD bit selects the active power-down mode. Note that this parameter applies only for DDR3 mode. For more information about this mode, see the data sheet for the SDRAM being used in your system.                            |
| 12 (R/W)           | PD         | 0                                                                                                                                                                                                                                                               | Fast Exit (normal)                                                                                                                                                                                                                                              |
| 12 (R/W)           | PD         | 1                                                                                                                                                                                                                                                               | Slow Exit (low power)                                                                                                                                                                                                                                           |
| 11:9 (R/W)         | WRRECOV    | Write Recovery. The DMC_MR.WRRECOV bit selects the write recovery time in terms of clock cycles (t CK ). Note that this parameter applies only for DDR3 mode. For more information about this mode, see the data sheet for the SDRAM being used in your system. | Write Recovery. The DMC_MR.WRRECOV bit selects the write recovery time in terms of clock cycles (t CK ). Note that this parameter applies only for DDR3 mode. For more information about this mode, see the data sheet for the SDRAM being used in your system. |
| 11:9 (R/W)         | WRRECOV    | 0                                                                                                                                                                                                                                                               | 16 clock cycles for DDR3                                                                                                                                                                                                                                        |
| 11:9 (R/W)         | WRRECOV    | 1                                                                                                                                                                                                                                                               | 5 clock cycles for DDR3                                                                                                                                                                                                                                         |
| 11:9 (R/W)         | WRRECOV    | 2                                                                                                                                                                                                                                                               | 6 clock cycles for DDR3                                                                                                                                                                                                                                         |
| 11:9 (R/W)         | WRRECOV    | 3                                                                                                                                                                                                                                                               | 7 clock cycles for DDR3                                                                                                                                                                                                                                         |
| 11:9 (R/W)         | WRRECOV    | 4                                                                                                                                                                                                                                                               | 8 clock cycles for DDR3                                                                                                                                                                                                                                         |
| 11:9 (R/W)         | WRRECOV    | 5                                                                                                                                                                                                                                                               | 10 clock cycles for DDR3                                                                                                                                                                                                                                        |
| 11:9 (R/W)         | WRRECOV    | 6                                                                                                                                                                                                                                                               | 12 clock cycles for DDR3                                                                                                                                                                                                                                        |

Table 10-14: DMC\_MR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |            | 7 14 clock cycles for DDR3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 8 (R/W)            | DLLRST     | DLL Reset. The DMC_MR.DLLRST bit initiates a DLL reset on the SDRAM. Note that this pa- rameter applies only for DDR3 mode. For more information about this operation, see the data sheet for the SDRAM being used in your system.                                                                                                                                                                                                                                                                                                 |
| 8 (R/W)            | DLLRST     | 0 Normal Operation                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 8 (R/W)            | DLLRST     | 1 Reset DLL                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 6:4 (R/W)          | CL         | CAS Latency. The DMC_MR.CL bit field select latency from the assertion of a read/write signal to the SDRAM until the first valid data on the output from the SDRAM in terms of clock cycles. For more information about this operation, see the data sheet for the SDRAM being used in your system. For DDR3 only, Bit[2] of this register must be used along with [6:4]. Following CAS values are seen. 0010: 5 0100: 6 0110: 7 1000: 8 1010: 9 1100: 10 1110: 11 0001: 12 0011: 13 0101: 14 All other combinations are reserved. |
| 2 (R/W)            | CL0        | CAS Latency 0. The DMC_MR.CL0 bit is applicable for DDR3 only and is used in conjunction with the DMC_MR.CL bits.                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 1:0 (R/W)          | BLEN       | Burst Length. The DMC_MR.BLEN bits select burst length for transfers. For more information about this operation, see the data sheet for the SDRAM being used in your system. Note that values other than those shown are not supported.                                                                                                                                                                                                                                                                                            |
| 1:0 (R/W)          | BLEN       | 0 8-Bit Burst Length - DDR3 only                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 1:0 (R/W)          | BLEN       | 3 Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |

## Shadow MR1 Register (DDR3)

The DMC\_MR1 register is a mirror of the DDR3 SDRAM Mode register 1. This register is used only when the DMC is operating in DDR3 mode. A write to this register triggers an extended "mode register 1 set" command on the memory interface provided the corresponding mask bit is set in the mask register. Else, only the mirror register is updated.

Figure 10-10: DMC\_MR1 Register Diagram

<!-- image -->

Table 10-15: DMC\_MR1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | QOFF       | Output Buffer Enable. The DMC_MR1.QOFF bit enables the SDRAM output pins. For more information about this operation, see the data sheet for the SDRAM being used in your system. 0 Output buffer enabled                                                                                                                                                                             |
| 11 (R/W)           | TDQS       | Termination Data Strobe. The DMC_MR1.TDQS bit provides additional termination resistance outputs that may be useful in some system configurations. The DMC_MR1.TDQS bit is not supported in x4 or x16 configurations. When enabled via the mode register, the same termination resistance function is applied to the TDQS/TDQS# pins that is applied to the DQS/ DQS# pins. 0 Enable |
| 11 (R/W)           |            | 1 Disable                                                                                                                                                                                                                                                                                                                                                                            |
| 11 (R/W)           |            |                                                                                                                                                                                                                                                                                                                                                                                      |

Table 10-15: DMC\_MR1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | RTT2       | Rtt_nom. The DMC_MR1.RTT2 bit is used in conjunction with the DMC_MR1.RTT0 and DMC_MR1.RTT1 bits. (9 6 2) 0 0 0 Rtt_Nom disabled 0 0 1 RZQ/4 0 1 0 RZQ/2 0 1 1 RZQ/6 1 0 0 RZQ/12 (reserved if Rtt_Nom is used during writes) 1 0 1 RZQ/8 (reserved if Rtt_Nom is used during writes) 1 1 0 Reserved 1 1 1 Reserved |
| 7 (R/W)            | WL         | Write Leveling Enable. The DMC_MR1.WL bit enables the SDRAM output pins. For more information about this operation, see the data sheet for the SDRAM being used in your system. 0 Disable                                                                                                                           |
| 6 (R/W)            | RTT1       | Rtt_nom. The DMC_MR1.RTT1 bit combines with the DMC_MR1.RTT0 bit to set the termi- nation resistance. See the DMC_MR1.RTT2 and DMC_MR1.RTT0 bit description for more information.                                                                                                                                   |
| 5 (R/W)            | DIC1       | Output Driver Impedance Control. The DMC_MR1.DIC1 bit is used in conjunction with the DMC_MR1.DIC0 bit. (5, 1) 0 0 RZQ/6 0 1 RZQ/7 1 0 Reserved 1 1 Reserved                                                                                                                                                        |

Table 10-15: DMC\_MR1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                 | Description/Enumeration                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4:3 (R/W)          | AL         | Additive Latency. The DMC_MR1.AL bits select a number of added latency time for CAS operations in terms of clock cycles (t CK ). For more information about this operation, see the data sheet for the SDRAM being used in your system. | Additive Latency. The DMC_MR1.AL bits select a number of added latency time for CAS operations in terms of clock cycles (t CK ). For more information about this operation, see the data sheet for the SDRAM being used in your system. |
| 4:3 (R/W)          | AL         | 0                                                                                                                                                                                                                                       | AL disabled                                                                                                                                                                                                                             |
| 4:3 (R/W)          | AL         | 1                                                                                                                                                                                                                                       | CL-1                                                                                                                                                                                                                                    |
| 4:3 (R/W)          | AL         | 2                                                                                                                                                                                                                                       | CL-2                                                                                                                                                                                                                                    |
| 4:3 (R/W)          | AL         | 3                                                                                                                                                                                                                                       | Reserved                                                                                                                                                                                                                                |
| 2 (R/W)            | RTT0       | Rtt_nom. The DMC_MR1.RTT0 bit combines with the DMC_MR1.RTT1 and DMC_MR1.RTT1 bits to set the termination resistance. See the DMC_MR1.RTT1                                                                                              | Rtt_nom. The DMC_MR1.RTT0 bit combines with the DMC_MR1.RTT1 and DMC_MR1.RTT1 bits to set the termination resistance. See the DMC_MR1.RTT1                                                                                              |
| 1 (R/W)            | DIC0       | Output Driver Impedance control. The DMC_MR1.DIC0 bit is used with the DMC_MR1.DIC1 bit.                                                                                                                                                | Output Driver Impedance control. The DMC_MR1.DIC0 bit is used with the DMC_MR1.DIC1 bit.                                                                                                                                                |
| 0 (R/W)            | DLLEN      | DLL Enable. The DMC_MR1.DLLEN bit enables the DLL in the SDRAM. For more information about this operation, see the data sheet for the SDRAM being used in your system.                                                                  | DLL Enable. The DMC_MR1.DLLEN bit enables the DLL in the SDRAM. For more information about this operation, see the data sheet for the SDRAM being used in your system.                                                                  |
| 0 (R/W)            | DLLEN      | 0                                                                                                                                                                                                                                       | Enable                                                                                                                                                                                                                                  |
| 0 (R/W)            | DLLEN      | 1                                                                                                                                                                                                                                       | Disable                                                                                                                                                                                                                                 |

## Shadow MR2 Register (DDR3)

The DMC\_MR2 register mirrors DDR3 SDRAM device Mode register 2 when the controller is operating in DDR3 mode. A write to this register triggers an extended "mode register set" command on the memory interface provided the corresponding mask bit is set in the mask register. Else, only the mirror register is updated.

Figure 10-11: DMC\_MR2 Register Diagram

<!-- image -->

Table 10-16: DMC\_MR2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | SRT        | Self Refresh Temperature Range. The DMC_MR2.SRT bit enables high temperature self-refresh rate.                   |
| 6 (R/W)            | ASR        | Auto Self Refresh. 0 Manual SR Reference (SRT)                                                                    |
| 5:3 (R/W)          | CWL        | Latency. 0 5 clock cycles                                                                                         |
|                    |            | CAS Write 1 6 clock cycles 2 7 clock cycles 3 8 clock cycles 4 9 clock cycles 5 10 clock cycles 6 11 clock cycles |

Table 10-16: DMC\_MR2 Register Fields (Continued)

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

The DMC\_MSK register permits masking (disabling) writes to the MR and EMRn registers in the SDRAM in DDR3 Mode. When masked, writes to these registers go instead to shadow copies of these registers ( DMC\_MR , DMC\_MR1 , DMC\_MR2 ), which are maintained within the DMC. When a shadow register's corresponding bit is unmasked (enabled), the DMC generates the MRS or EMRS command to transfer the contents of the shadow register (in the DMC) to the actual register (in the SDRAM).

Figure 10-12: DMC\_MSK Register Diagram

<!-- image -->

Table 10-17: DMC\_MSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | EMR3       | Shadow EMR3 Unmask. The DMC_MSK.EMR3 bit masks or unmasks writes to the EMR3 register in the SDRAM. When masked, writes to this register instead go to the EMR3 register. When unmasked, the DMCwrites the EMR3 value to the EMR3 register in the SDRAM. After completing the write, the DMCclears this bit. | Shadow EMR3 Unmask. The DMC_MSK.EMR3 bit masks or unmasks writes to the EMR3 register in the SDRAM. When masked, writes to this register instead go to the EMR3 register. When unmasked, the DMCwrites the EMR3 value to the EMR3 register in the SDRAM. After completing the write, the DMCclears this bit. |
|                    |            | 0                                                                                                                                                                                                                                                                                                            | Mask (Disable) Write to EMR3                                                                                                                                                                                                                                                                                 |
| 10 (R/W)           | MR2        | Shadow MR2 Unmask. The DMC_MSK.MR2 bit masks or unmasks writes to the MR2 register (in DDR3 mode) in the SDRAM. When masked, writes to this register instead go to the DMC_MR2 register. When unmasked, the DMCwrites the DMC_MR2 value to the                                                               | Shadow MR2 Unmask. The DMC_MSK.MR2 bit masks or unmasks writes to the MR2 register (in DDR3 mode) in the SDRAM. When masked, writes to this register instead go to the DMC_MR2 register. When unmasked, the DMCwrites the DMC_MR2 value to the                                                               |
|                    |            | 0                                                                                                                                                                                                                                                                                                            | Mask (Disable) Write to EMR2                                                                                                                                                                                                                                                                                 |
|                    |            | 1                                                                                                                                                                                                                                                                                                            | Unmask (Enable) Write to EMR2                                                                                                                                                                                                                                                                                |

Table 10-17: DMC\_MSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | MR1        | Shadow MR1 Unmask. The DMC_MSK.MR1 bit masks or unmasks writes to the MR1 register in the DDR3 SDRAM. When masked, writes to this register instead go to the DMC_MR1 register. When unmasked, the DMCwrites the DMC_MR1 value to the MR1 register in the SDRAM. After completing the write, the DMCclears this bit. Note that this bit is valid only for DDR3 Mode of operation. | Shadow MR1 Unmask. The DMC_MSK.MR1 bit masks or unmasks writes to the MR1 register in the DDR3 SDRAM. When masked, writes to this register instead go to the DMC_MR1 register. When unmasked, the DMCwrites the DMC_MR1 value to the MR1 register in the SDRAM. After completing the write, the DMCclears this bit. Note that this bit is valid only for DDR3 Mode of operation. |
| 9 (R/W)            | MR1        | 0                                                                                                                                                                                                                                                                                                                                                                                | Mask (Disable) Write to EMR1                                                                                                                                                                                                                                                                                                                                                     |
| 8 (R/W)            | MR         | Shadow MRUnmask. The DMC_MSK.MR bit masks or unmasks writes to the MRregister in the SDRAM. When masked, writes to this register instead go to the DMC_MR register. When un- masked, the DMCwrites the DMC_MR value to the MRregister in the SDRAM. After completing the write, the DMCclears this bit.                                                                          | Shadow MRUnmask. The DMC_MSK.MR bit masks or unmasks writes to the MRregister in the SDRAM. When masked, writes to this register instead go to the DMC_MR register. When un- masked, the DMCwrites the DMC_MR value to the MRregister in the SDRAM. After completing the write, the DMCclears this bit.                                                                          |
| 8 (R/W)            | MR         | 0                                                                                                                                                                                                                                                                                                                                                                                | Mask (Disable) Write toMR                                                                                                                                                                                                                                                                                                                                                        |
| 8 (R/W)            | MR         | 1                                                                                                                                                                                                                                                                                                                                                                                | Unmask (Enable) Write toMR                                                                                                                                                                                                                                                                                                                                                       |

## DMC Performance Control Register

The DMC\_PM\_CTL register generates a performance index. It does this by obtaining a count of various performance indicators.

Figure 10-13: DMC\_PM\_CTL Register Diagram

<!-- image -->

Table 10-18: DMC\_PM\_CTL Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                                                       |
|--------------------|--------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W)           | INVOPERATION | Inverted Operation. The DMC_PM_CTL.INVOPERATION bit is inverts the operation of the idle cycle counter. When enabled the counter reports the number of cycles for which the corre- sponding bus is NOT idle. Used for the DQbus, SCB Read data bus and SCB Write data bus. 0 Normal operation |
| 17:16 (R/W)        | PMEFFCTLRWEN | Performance Monitor Read/Write Select. The DMC_PM_CTL.PMEFFCTLRWEN bit selects whether the performance monitor efficiency counter is used for read transactions or write transactions 1 Read 2 Write                                                                                          |

Table 10-18: DMC\_PM\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:12 (R/W)        | BRSZSEL    | Burst Size Select. The DMC_PM_CTL.BRSZSEL bit field reports the value of number of SCB transfers corresponding to the burst size shown by these bits.     |
| 15:12 (R/W)        | BRSZSEL    | 0 Burst size 8                                                                                                                                            |
| 15:12 (R/W)        | BRSZSEL    | 1 Burst size 16                                                                                                                                           |
| 15:12 (R/W)        | BRSZSEL    | 2 Burst size 32                                                                                                                                           |
| 11:8 (R/W)         | BRLNSEL    | Burst Length Select. The DMC_PM_CTL.BRLNSEL bit field reports the value of number of SCB transfers corresponding to the burst length shown by these bits. |
| 11:8 (R/W)         | BRLNSEL    | 0 Burst length 1                                                                                                                                          |
| 11:8 (R/W)         | BRLNSEL    | 1 Burst length 2                                                                                                                                          |
| 11:8 (R/W)         | BRLNSEL    | 3 Burst length 4                                                                                                                                          |
| 11:8 (R/W)         | BRLNSEL    | 7 Burst length 8                                                                                                                                          |
| 7:4 (R/W)          | BANKSEL    | Bank Select. The DMC_PM_CTL.BANKSEL bit field chooses which bank or all banks are being selected for the page hit or page miss counters.                  |
| 7:4 (R/W)          | BANKSEL    | 0 Bank 0                                                                                                                                                  |
| 7:4 (R/W)          | BANKSEL    | 1 Bank 1                                                                                                                                                  |
| 7:4 (R/W)          | BANKSEL    | 2 Bank 2                                                                                                                                                  |
| 7:4 (R/W)          | BANKSEL    | 3 Bank 3                                                                                                                                                  |
| 7:4 (R/W)          | BANKSEL    | 4 Bank 4                                                                                                                                                  |
| 7:4 (R/W)          | BANKSEL    | 5 Bank 5                                                                                                                                                  |
| 7:4 (R/W)          | BANKSEL    | 6 Bank 6                                                                                                                                                  |
| 7:4 (R/W)          | BANKSEL    | 7 Bank 7                                                                                                                                                  |
| 7:4 (R/W)          | BANKSEL    | 8 All Banks                                                                                                                                               |

Table 10-18: DMC\_PM\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------|
| 3:1 (R/W)          | PMCNTSCALE | Performance Monitor Scale. The DMC_PM_CTL.PMCNTSCALE bit bit field reports the scale of the main to sub- counter. 0 1:1 |
| 3:1 (R/W)          | PMCNTSCALE | 1 1:10                                                                                                                  |
| 3:1 (R/W)          | PMCNTSCALE | 2 1:100                                                                                                                 |
| 3:1 (R/W)          | PMCNTSCALE | 4 1:1000                                                                                                                |
| 0 (R/W)            | PMCNT      | Performance Monitor Control. The DMC_PM_CTL.PMCNT bit enables performance monitoring. counters                          |
| 0 (R/W)            | PMCNT      | 0 Disable the                                                                                                           |
| 0 (R/W)            | PMCNT      | 1 Enable the counters                                                                                                   |

## Performance Monitor Idle Cycle DQ Count Register

The DMC\_PM\_IDCYDQ\_CNT register reports the number of idle cycles on the DQ bus.

Figure 10-14: DMC\_PM\_IDCYDQ\_CNT Register Diagram

<!-- image -->

Table 10-19: DMC\_PM\_IDCYDQ\_CNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Idle cycles count.        |
| (R/NW)             |            |                           |

## Performance Monitor Idle Page Count Register

The DMC\_PM\_IDPG\_CNT register reports the number of accesses to idle pages.

Figure 10-15: DMC\_PM\_IDPG\_CNT Register Diagram

<!-- image -->

Table 10-20: DMC\_PM\_IDPG\_CNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Idle page access count.   |
| (R/NW)             |            |                           |

## Performance Monitor Page Hit Count

The DMC\_PM\_PGHT\_CNT register reports the number of page hit accesses.

Figure 10-16: DMC\_PM\_PGHT\_CNT Register Diagram

<!-- image -->

Table 10-21: DMC\_PM\_PGHT\_CNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Page hit count.           |
| (R/NW)             |            |                           |

## Performance Monitor Page Miss Count

The DMC\_PM\_PGMS\_CNT register reports the number of missed page accesses.

Figure 10-17: DMC\_PM\_PGMS\_CNT Register Diagram

<!-- image -->

Table 10-22: DMC\_PM\_PGMS\_CNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Page miss count.          |
| (R/NW)             |            |                           |

## Performance Monitor Read to Write Turn Around Count Register

The DMC\_PM\_RD2WRTA\_CNT register reports the number of times the read to write turn around occurs.

Figure 10-18: DMC\_PM\_RD2WRTA\_CNT Register Diagram

<!-- image -->

Table 10-23: DMC\_PM\_RD2WRTA\_CNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration          |
|--------------------|------------|----------------------------------|
| 31:0               | VALUE      | Read to write turn around count. |
| (R/NW)             |            |                                  |

## Performance Monitor Refresh Count Register

The DMC\_PM\_REF\_CNT register reports the number of times a refresh occurs.

Figure 10-19: DMC\_PM\_REF\_CNT Register Diagram

<!-- image -->

Table 10-24: DMC\_PM\_REF\_CNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Refresh count.            |
| (R/NW)             |            |                           |

## Performance Monitor Write to Read Turn Around Count Register

The DMC\_PM\_WR2RDTA\_CNT register reports the number of times the write to read turn around occurs.

<!-- image -->

Write to read turn around count

Figure 10-20: DMC\_PM\_WR2RDTA\_CNT Register Diagram

Table 10-25: DMC\_PM\_WR2RDTA\_CNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration          |
|--------------------|------------|----------------------------------|
| 31:0               | VALUE      | Write to read turn around count. |
| (R/NW)             |            |                                  |

## Priority ID Register 1

The DMC\_PRIO register allows transactions from selected requesters that generate specific SCB IDs to obtain higher priority than the transactions proceeding in the usual fashion. The contents of the register are masked with the contents of the DMC\_PRIOMSK register to obtain a single SCB ID or a range of IDs that get elevated priority.

Figure 10-21: DMC\_PRIO Register Diagram

<!-- image -->

Table 10-26: DMC\_PRIO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                  |
|--------------------|------------|------------------------------------------|
| 31:0               | ID1        | SCB ID1 that Requires Elevated Priority. |

## Priority ID Register 2

The DMC\_PRIO2 register is another register which allows transactions from selected requesters that generate specific SCB IDs to obtain higher priority than the transactions proceeding in the usual fashion. The contents of the register are masked with the contents of the DMC\_PRIOMSK2 register to obtain a single SCB ID or a range of IDs that get elevated priority.

Figure 10-22: DMC\_PRIO2 Register Diagram

<!-- image -->

Table 10-27: DMC\_PRIO2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                  |
|--------------------|------------|------------------------------------------|
| 31:0               | ID2        | SCB ID2 that Requires Elevated Priority. |
| (R/W)              |            |                                          |

## Priority ID Mask Register 1

The DMC\_PRIOMSK register masks the respective ID bits in the DMC\_PRIOMSK register. This masking provides for elevating the access priority of either a single ID or a range of IDs.

Figure 10-23: DMC\_PRIOMSK Register Diagram

<!-- image -->

Table 10-28: DMC\_PRIOMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | ID1MSK     | Mask for SCB ID1.         |
| (R/W)              |            |                           |

## Priority ID Mask Register 2

The DMC\_PRIOMSK2 register bits mask the respective ID bits in the DMC\_PRIO2 register. This masking provides for elevating the access priority of either a single ID or a range of IDs.

Figure 10-24: DMC\_PRIOMSK2 Register Diagram

<!-- image -->

Table 10-29: DMC\_PRIOMSK2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | ID2MSK     | Mask for SCB ID2.         |
| (R/W)              |            |                           |

## DMC Read Data Buffer ID Register 1

The DMC\_RDDATABUFID1 register allows read transactions from selected requesters to make use of DMC read data buffer. The contents of the register are masked with the contents of the DMC\_RDDATABUFMSK1 register to obtain a single SCB ID or a range of IDs that get elevated priority.

Figure 10-25: DMC\_RDDATABUFID1 Register Diagram

<!-- image -->

Table 10-30: DMC\_RDDATABUFID1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration        |
|--------------------|------------|--------------------------------|
| 31:0               | VALUE      | Mask for Read Data Buffer ID1. |

## DMC Read Data Buffer ID Register 2

The DMC\_RDDATABUFID2 register allows read transactions from selected requesters to make use of DMC read data buffer. The contents of the register are masked with the contents of the DMC\_RDDATABUFMSK2 register to obtain a single SCB ID or a range of IDs that get elevated priority.

Figure 10-26: DMC\_RDDATABUFID2 Register Diagram

<!-- image -->

Table 10-31: DMC\_RDDATABUFID2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration        |
|--------------------|------------|--------------------------------|
| 31:0               | VALUE      | Mask for Read Data Buffer ID2. |

## DMC Read Data Buffer Mask Register 1

The DMC\_RDDATABUFMSK1 register bits mask the respective ID bits in the DMC Priority Mask ID register.

Figure 10-27: DMC\_RDDATABUFMSK1 Register Diagram

<!-- image -->

Table 10-32: DMC\_RDDATABUFMSK1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration        |
|--------------------|------------|--------------------------------|
| 31:0               | VALUE      | Mask for Read Data Buffer ID1. |
| (R/W)              |            |                                |

## DMC Read Data Buffer Mask Register 2

The DMC\_RDDATABUFMSK2 register bits mask the respective ID bits in the DMC Priority Mask ID register.

Figure 10-28: DMC\_RDDATABUFMSK2 Register Diagram

<!-- image -->

Table 10-33: DMC\_RDDATABUFMSK2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration        |
|--------------------|------------|--------------------------------|
| 31:0               | VALUE      | Mask for Read Data Buffer ID2. |
| (R/W)              |            |                                |

## Performance Monitor Buffer Hit Count Register

The DMC\_RDD\_BUFHT\_CNT register reports the count of the number of read data buffer hits.

Figure 10-29: DMC\_RDD\_BUFHT\_CNT Register Diagram

<!-- image -->

Table 10-34: DMC\_RDD\_BUFHT\_CNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Buffer Hit Count.         |
| (R/NW)             |            |                           |

## SCB Performance AXI Transfer Count Register

The DMC\_SCB\_PM\_BRLN\_CNT register reports the count of number of times AXI transfer of burst length selected by performance control register is encountered.

Figure 10-30: DMC\_SCB\_PM\_BRLN\_CNT Register Diagram

<!-- image -->

Table 10-35: DMC\_SCB\_PM\_BRLN\_CNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | Burst Length Transfer Count. The DMC_SCB_PM_BRLN_CNT.VALUE field indicates the number of times the AXI transfer of the selected burst length is encountered. The performance control reg- ister selects the burst length of the transfer. |

## SCB Performance AXI Transfer Count Register

The DMC\_SCB\_PM\_BRSZ\_CNT register reports the count of number of times AXI transfer of burst size selected by performance control register is encountered.

Figure 10-31: DMC\_SCB\_PM\_BRSZ\_CNT Register Diagram

<!-- image -->

Table 10-36: DMC\_SCB\_PM\_BRSZ\_CNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | Burst Size Transfer Count. The DMC_SCB_PM_BRSZ_CNT.VALUE bit field indicates the number of times the AXI transfer of the selected burst size is encountered. The performance control register selects the burst size of the transfer. |

## SCB Performance Monitor Idle Cycle Count Register

The DMC\_SCB\_PM\_IDRDDCY\_CNT register reports the count of the number idle cycles on the read data bus.

Figure 10-32: DMC\_SCB\_PM\_IDRDDCY\_CNT Register Diagram

<!-- image -->

Table 10-37: DMC\_SCB\_PM\_IDRDDCY\_CNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | Read Idle Cycle Count. The DMC_SCB_PM_IDRDDCY_CNT.VALUE bit field indicates the number idle cy- cles on the read data bus. |
| (R/NW)             |            |                                                                                                                            |

## SCB Performance Monitor Idle Cycle Count Register

The DMC\_SCB\_PM\_IDWRDCY\_CNT register reports the count of number idle cycles on the write data bus.

Figure 10-33: DMC\_SCB\_PM\_IDWRDCY\_CNT Register Diagram

<!-- image -->

Table 10-38: DMC\_SCB\_PM\_IDWRDCY\_CNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | Write Idle Cycle Count. The DMC_SCB_PM_IDWRDCY_CNT.VALUE bit field indicates the number of idle cycles on the write data bus. |
| (R/NW)             |            |                                                                                                                               |

## Status Register

The DMC\_STAT register indicates status for modes selected with the DMC\_CTL register and indicates status DMC operations.

Figure 10-34: DMC\_STAT Register Diagram

<!-- image -->

Table 10-39: DMC\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R/NW)          | ZQCLDONE   | ZQ Calibration Long Done. The DMC_STAT.ZQCLDONE bit checks if the ZQ calibration long sub routine is done.                                                                                                                                                                                                               |
| 24 (R/NW)          | ZQCSDONE   | ZQ Calibration Short Done. The DMC_STAT.ZQCSDONE bit checks if the ZQ calibration short sub routine is done.                                                                                                                                                                                                             |
| 23:20 (R/NW)       | PHYRDPHASE | PHY Read Phase. The DMC_STAT.PHYRDPHASE bits indicate the latency after which the DMCmay read from the PHY. Taking round trip delay into account, the DLL indicates the exact number of clock cycles after which the controller needs to read data. Values other than those shown are reserved. 2 2 Clock Cycles Latency |

Table 10-39: DMC\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------|
|                    |            | 3 3 Clock Cycles Latency                                                                                                                    |
|                    |            | 4 4 Clock Cycles Latency                                                                                                                    |
|                    |            | 5 5 Clock Cycles Latency                                                                                                                    |
|                    |            | 6 6 Clock Cycles Latency                                                                                                                    |
|                    |            | 7 7 Clock Cycles Latency                                                                                                                    |
|                    |            | 8 8 Clock Cycles Latency                                                                                                                    |
|                    |            | 9 9 Clock Cycles Latency                                                                                                                    |
|                    |            | 10 10 Clock Cycles Latency                                                                                                                  |
|                    |            | 11 11 Clock Cycles Latency                                                                                                                  |
|                    |            | 12 12 Clock Cycles Latency                                                                                                                  |
|                    |            | 13 13 Clock Cycles Latency                                                                                                                  |
|                    |            | 14 14 Clock Cycles Latency                                                                                                                  |
|                    |            | 15 15 Clock Cycles Latency                                                                                                                  |
| 19:16 (R/NW)       | PENDREF    | Pending Refresh. The DMC_STAT.PENDREF bits indicate the number of pending auto-refresh com- mands whose value can be from "0000" to "0111". |
| 13 (R/NW)          | DLLCALDONE | DLL Calibration Done. The DMC_STAT.DLLCALDONE indicates that the PHY DLL calibration sequence is complete.                                  |
|                    |            | 0 No Status                                                                                                                                 |
|                    |            | 1 Completed PHY DLL Calibration                                                                                                             |
| 7 (R/NW)           | RESETDONE  | Reset Done. The DMC_STAT.RESETDONE bit indicates that the reset sequence is complete.                                                       |
|                    |            | 0 SDRAM Reset is ongoing                                                                                                                    |
|                    |            | 1 SDRAM Reset is done                                                                                                                       |
| 4 (R/NW)           | PDACK      | Power-Down Acknowledge. The DMC_STAT.PDACK bit indicates that power-down mode is active.                                                    |
|                    |            | 0 Not in Power-Down Mode                                                                                                                    |
|                    |            | 1 Power-Down Mode Active                                                                                                                    |

Table 10-39: DMC\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------|
| 3 (R/NW)           | SRACK      | Self-Refresh Acknowledge. The DMC_STAT.SRACK bit indicates that self-refresh mode is active. 0 Not in Self-Refresh Mode |
| 2 (R/NW)           | INITDONE   | Initialization Done. The DMC_STAT.INITDONE bit indicates that the initialization sequence is com- plete.                |
| 0 (R/NW)           | IDLE       | Idle State. The DMC_STAT.IDLE bit indicates whether the DMCis idle or busy. 0 Busy 1 Idle                               |

## Timing 0 Register

The DMC\_TR0 register selects timing parameters for DMC operation to corresponding with parameters of the SDRAM device that is used in the system. The timing registers must be programmed to match the device for correct operation of the SDRAM and must be programmed before initializing the SDRAM. Note that all values for bit fields in DMC\_TR0 are in increments of clock cycle time (t CK).

Figure 10-35: DMC\_TR0 Register Diagram

<!-- image -->

Table 10-40: DMC\_TR0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:28 (R/W)        | TMRD       | Timing Mode Register Delay. The DMC_TR0.TMRD field selects the set-to-active timing parameter (t MRD ), which is the number of clock cycles that occur after the mode registers in the SDRAM are set and before the next command is issued.       |
| 25:20 (R/W)        | TRC        | Timing Row Cycle. The DMC_TR0.TRC field selects the active-to-active time (t RC ), which is the mini- mum number of clock cycles that occur from an active command to the next active command in the same bank.                                   |
| 16:12 (R/W)        | TRAS       | Timing Row Active Time. The DMC_TR0.TRAS field selects the active-to-precharge time (t RAS ), which is the number of clock cycles that occur from an active command until a precharge com- mand is allowed.                                       |
| 11:8 (R/W)         | TRP        | Timing RAS Precharge. The DMC_TR0.TRP field selects the precharge-to-active time (t RP ), which is the number of clock cycles that occur while the SDRAM recovers from a precharge com- mand and becomes ready to accept the next active command. |

Table 10-40: DMC\_TR0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:4 (R/W)          | TWTR       | Timing Write to Read. The DMC_TR0.TWTR field selects the write-to-read delay time (t WTR ), which is the number of clock cycles that occur from the last write data to the next read command. |
| 3:0 (R/W)          | TRCD       | Timing RAS to CAS Delay. The DMC_TR0.TRCD field selects the RAS to CAS delay time (t RCD ), which is the number of clock cycles that occur from an active command to a read/write assertion.  |

## Timing 1 Register

The DMC\_TR1 register selects timing parameters for DMC operation to corresponding with parameters of the SDRAM device that is used in the system. The timing registers must be programmed to match the device for correct operation of the SDRAM and must be programmed before initializing the SDRAM. Note that all values for bit fields in DMC\_TR1 are in increments of clock cycle time (t CK).

Figure 10-36: DMC\_TR1 Register Diagram

<!-- image -->

Table 10-41: DMC\_TR1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30:28 (R/W)        | TRRD       | Timing Read-Read Delay. The DMC_TR1.TRRD field selects the active-to-active time (t RRD ), which is the mini- mum number of clock cycles occurring from a bank x active command to a bank y active command.                                                                                                                                                                                                                                                                                                                                             |
| 24:16 (R/W)        | TRFC       | Timing Refresh-to-Command. The DMC_TR1.TRFC field selects the refresh-to-active command delay (t RFC ), which is the number of clock cycles required for the SDRAM to recover from a refresh signal to be ready to take the next command. It is also the number of clock cycles needed for the SDRAM to recover from executing one active command and ready to accept the next active command.                                                                                                                                                          |
| 13:0 (R/W)         | TREF       | Timing Refresh Interval. The DMC_TR1.TREF field selects the refresh interval time (t REF ), which is the num- ber of clock cycles occurring from one refresh command to the next refresh command. The actual timing of issuing a precharge command may be delayed by if the SDRAM is processing a normal access. However, the delay is not accumulative so there is no need to shorten the refresh interval to account for the memory access time. The non- accumulative refresh delay typically increases memory bandwidth by a few percentage points. |

## Timing 2 Register

The DMC\_TR2 register selects timing parameters for DMC operation to corresponding with parameters of the SDRAM device that is used in the system. The timing registers must be programmed to match the device for correct operation of the SDRAM and before initializing the SDRAM.

Note that all values for bit fields in DMC\_TR2 are in increments of clock cycle time (t CK).

Figure 10-37: DMC\_TR2 Register Diagram

<!-- image -->

Table 10-42: DMC\_TR2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:20 (R/W)        | TCKE       | Timing Clock Enable. The DMC_TR2.TCKE field selects the CKE minimum pulsewidth (t CKE ).                                                                                                                                                                                                                                                                                      |
| 19:16 (R/W)        | TXP        | Timing Exit Power Down. The DMC_TR2.TXP field selects the exit power down to next valid command time (t XP ).                                                                                                                                                                                                                                                                 |
| 11:8 (R/W)         | TRTP       | Timing Read-to-Precharge. The DMC_TR2.TRTP field selects the internal read to precharge time (t RTP ). If the resulting value is less than 2, the register needs to be programmed with two. Note: Minimum t RTP that needs to be programmed is 2.                                                                                                                             |
| 5 (RX/W)           | TFAW5      | Extended Timing Four-Active Window Bit 5. The DMC_TR2.TFAW5 bit is the extended bit for FAW timing for 800 MHz opera- tion for values greater than 31. It is only applicable when DMC_CTL. AL_EN= 1. When DMC_CTL. AL_EN = 0, the DMC_TR2.TFAW5 bit is reserved. When DMC_CTL. AL_EN = 1, the total FAW[5:0] = {TFAW5,TFAW[4:0]}. NOTE: This bit is write-only. Read as zero. |

Table 10-42: DMC\_TR2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4:0 (R/W)          | TFAW       | Timing Four-Activated-Window. The DMC_TR2.TFAW field selects the four-banks activated window time (t FAW ). No more than four SDRAM banks should be activated within this window. |

## ADSP-2159x\_SC592\_SC594 DMC Register Descriptions

1600MHz DMCPHY (DDR3/DDR3L) (DMC) contains the following registers.

Table 10-43: ADSP-2159x\_SC592\_SC594 DMC Register List

| Name               | Description                        |
|--------------------|------------------------------------|
| DMC_DDR_CA_CTL     | DDR CA Lane Control Register       |
| DMC_DDR_LANE0_CTL0 | Data Lane 0 Control Register 0     |
| DMC_DDR_LANE0_CTL1 | Data Lane 0 Control Register 1     |
| DMC_DDR_LANE1_CTL0 | Data Lane 1 Control Register 0     |
| DMC_DDR_LANE1_CTL1 | Data Lane 1 Control Register 1     |
| DMC_DDR_ROOT_CTL   | DDR ROOT Module Control Register   |
| DMC_DDR_SCRATCH_2  | Scratch Register 2                 |
| DMC_DDR_SCRATCH_3  | Scratch Register 3                 |
| DMC_DDR_SCRATCH_4  | Scratch Register 4                 |
| DMC_DDR_SCRATCH_5  | Scratch Register 5                 |
| DMC_DDR_SCRATCH_6  | Scratch Register 6                 |
| DMC_DDR_SCRATCH_7  | Scratch Register 7                 |
| DMC_DDR_ZQ_CTL0    | DDR Calibration Control Register 0 |
| DMC_DDR_ZQ_CTL1    | DDR Calibration Control Register 1 |
| DMC_DDR_ZQ_CTL2    | DDR Calibration Control Register 2 |

## DDR CA Lane Control Register

Figure 10-38: DMC\_DDR\_CA\_CTL Register Diagram

<!-- image -->

Table 10-44: DMC\_DDR\_CA\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30:26 (R/W)        | SLAVE_ID   | SW Completer Read/Write Completer ID. The DMC_DDR_CA_CTL.SLAVE_ID bit field is the completer ID for SW Com- pleter read or write transactions.                                                                                                                     |
| 24:23 (R/W)        | BYPCODE1   | Duty Cycle Level Select. The DMC_DDR_CA_CTL.BYPCODE1 bit field combines with BYPCODE0 to select one of four levels of duty cycle adjustment. Valid values for this bit field are 8,4,2,1, and 0. A larger value implies a larger offset. The default setting is 0. |
| 20:19 (R/W)        | BYPCODE0   | Duty Cycle Level Select. The DMC_DDR_CA_CTL.BYPCODE0 bit field combines with BYPCODE1 to select one of four levels of duty cycle adjustment. Valid values for this bit field are 8,4,2,1, and 0. A larger value implies a larger offset. The default setting is 0. |
| 18 (R/W)           | BYPSELP    | Duty Cycle Offset Direction Select. The DMC_DDR_CA_CTL.BYPSELP bit selects the direction of the duty cycle offset. 0 Negative                                                                                                                                      |
| 18 (R/W)           | BYPSELP    | 1 Positive                                                                                                                                                                                                                                                         |
| 18 (R/W)           | BYPSELP    |                                                                                                                                                                                                                                                                    |

Table 10-44: DMC\_DDR\_CA\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                 |
|--------------------|-----------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W)           | BYPENB          | CLK Duty Cycle Offset Adjustment Enable. The DMC_DDR_CA_CTL.BYPENB bit enables the CLK duty cycle offset adjust- ment.                  |
| 16:15 (R/W)        | MF_SEL          | SPI Master Frequency Divider Select. The DMC_DDR_CA_CTL.MF_SEL bit selects the SPI master frequency divider for root to lane transfers. |
| 14 (R/W)           | SW_REFRESH      | Refresh Lane DLL Code.                                                                                                                  |
| 2 (R/W)            | SWTRIG_SLVRD_CA | Software Trigger Completer Read.                                                                                                        |
| 1                  | SWTRIG_SLVWR_CA | Software Trigger Completer                                                                                                              |
| (R/W)              |                 | Write.                                                                                                                                  |

## Data Lane 0 Control Register 0

Figure 10-39: DMC\_DDR\_LANE0\_CTL0 Register Diagram

<!-- image -->

Table 10-45: DMC\_DDR\_LANE0\_CTL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------|
| 27 (R/W)           | CB_RSTDAT  | Reset the Data Pads.                                                                                                                       |
| 15:12 (R/W)        | BYPCODE    | Duty Cycle Level. Valid values for this bit field are 8,4,2,1, and zero. A larger value implies a larger offset. The default setting is 0. |
| 11 (R/W)           | BYPSELP    | Duty Cycle Offset Direction Select. The DMC_DDR_LANE0_CTL0.BYPSELP bit selects the direction of the duty cycle offset.                     |
| 11 (R/W)           | BYPSELP    | 0 Negative                                                                                                                                 |
| 10 (R/W)           | BYPENB     | DQS Duty Cycle Offset Adjustment Enable. The DMC_DDR_LANE0_CTL0.BYPENB bit enables the DQS duty cycle offset ad- justment.                 |
| 10 (R/W)           | BYPENB     | 0 Disable                                                                                                                                  |
| 10 (R/W)           | BYPENB     | 1 Enable                                                                                                                                   |
| 8 (R/W)            | CB_RSTDLL  | Reset the Lane DLL.                                                                                                                        |

Table 10-45: DMC\_DDR\_LANE0\_CTL0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                       | Description/Enumeration                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------|
| 6:5 (R/W)          | MF_SEL     | SPI Master Frequency Divider Select. The DMC_DDR_LANE0_CTL0.MF_SEL bit selects the SPI master frequency divid- er for root to lane transfers. | SPI Master Frequency Divider Select. The DMC_DDR_LANE0_CTL0.MF_SEL bit selects the SPI master frequency divid- er for root to lane transfers. |
| 6:5 (R/W)          | MF_SEL     | 0                                                                                                                                             | Divide by 2                                                                                                                                   |
| 6:5 (R/W)          | MF_SEL     | 1                                                                                                                                             | Divide by 4                                                                                                                                   |
| 6:5 (R/W)          | MF_SEL     | 2                                                                                                                                             | Divide by 8                                                                                                                                   |
| 6:5 (R/W)          | MF_SEL     | 3                                                                                                                                             | Divide by 16                                                                                                                                  |
| 2:1                | DDR_MODE   | DDR Mode.                                                                                                                                     | DDR Mode.                                                                                                                                     |
| (R/W)              | DDR_MODE   | 0                                                                                                                                             | DDR3/3L Mode                                                                                                                                  |
| 0                  | LANE_DIS   | Clock Gate Data Lane.                                                                                                                         | Clock Gate Data Lane.                                                                                                                         |
| (R/W)              | LANE_DIS   | 0                                                                                                                                             | Lane is Enabled                                                                                                                               |
| (R/W)              | LANE_DIS   | 1                                                                                                                                             | Lane is Clock Gated                                                                                                                           |

## Data Lane 0 Control Register 1

Figure 10-40: DMC\_DDR\_LANE0\_CTL1 Register Diagram

<!-- image -->

Table 10-46: DMC\_DDR\_LANE0\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                             |
|--------------------|---------------|---------------------------------------------------------------------|
| 15 (R/W)           | BYPDELCHAINEN | Bypass enable for delay chain controlling extra delay on DATA pins. |
| 14:10 (R/W)        | BYPCODE       | Bypass Code for extra delay on DATA pins.                           |
| 1 (R/W)            | COMP_DCYCLE   | Compute Datacycle.                                                  |

## Data Lane 1 Control Register 0

Figure 10-41: DMC\_DDR\_LANE1\_CTL0 Register Diagram

<!-- image -->

Table 10-47: DMC\_DDR\_LANE1\_CTL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------|
| 27 (R/W)           | CB_RSTDAT  | Reset the Data Pads.                                                                                                                       |
| 15:12 (R/W)        | BYPCODE    | Duty Cycle Level. Valid values for this bit field are 8,4,2,1, and zero. A larger value implies a larger offset. The default setting is 0. |
| 11 (R/W)           | BYPSELP    | Duty Cycle Offset Direction Select. The DMC_DDR_LANE1_CTL0.BYPSELP bit selects the direction of the duty cycle offset.                     |
| 11 (R/W)           | BYPSELP    | 0 Negative                                                                                                                                 |
| 10 (R/W)           | BYPENB     | DQS Duty Cycle Offset Adjustment Enable. The DMC_DDR_LANE1_CTL0.BYPENB bit enables the DQS duty cycle offset ad- justment.                 |
| 10 (R/W)           | BYPENB     | 0 Disable                                                                                                                                  |
| 10 (R/W)           | BYPENB     | 1 Enable                                                                                                                                   |
| 8 (R/W)            | CB_RSTDLL  | Reset the Lane DLL.                                                                                                                        |

Table 10-47: DMC\_DDR\_LANE1\_CTL0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                       | Description/Enumeration                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------|
| 6:5 (R/W)          | MF_SEL     | SPI Master Frequency Divider Select. The DMC_DDR_LANE1_CTL0.MF_SEL bit selects the SPI master frequency divid- er for root to lane transfers. | SPI Master Frequency Divider Select. The DMC_DDR_LANE1_CTL0.MF_SEL bit selects the SPI master frequency divid- er for root to lane transfers. |
| 6:5 (R/W)          | MF_SEL     | 0                                                                                                                                             | Divide by 2                                                                                                                                   |
| 6:5 (R/W)          | MF_SEL     | 1                                                                                                                                             | Divide by 4                                                                                                                                   |
| 6:5 (R/W)          | MF_SEL     | 2                                                                                                                                             | Divide by 8                                                                                                                                   |
| 6:5 (R/W)          | MF_SEL     | 3                                                                                                                                             | Divide by 16                                                                                                                                  |
| 2:1                | DDR_MODE   | DDR Mode.                                                                                                                                     | DDR Mode.                                                                                                                                     |
| (R/W)              | DDR_MODE   | 0                                                                                                                                             | DDR3/3L Mode                                                                                                                                  |
| 0                  | LANE_DIS   | Clock Gate Data Lane.                                                                                                                         | Clock Gate Data Lane.                                                                                                                         |
| (R/W)              | LANE_DIS   | 0                                                                                                                                             | Lane is Enabled                                                                                                                               |
| (R/W)              | LANE_DIS   | 1                                                                                                                                             | Lane is Clock Gated                                                                                                                           |

## Data Lane 1 Control Register 1

Figure 10-42: DMC\_DDR\_LANE1\_CTL1 Register Diagram

<!-- image -->

Table 10-48: DMC\_DDR\_LANE1\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                             |
|--------------------|---------------|---------------------------------------------------------------------|
| 15 (R/W)           | BYPDELCHAINEN | Bypass enable for delay chain controlling extra delay on DATA pins. |
| 14:10 (R/W)        | BYPCODE       | Bypass Code for extra delay on DATA pins.                           |
| 1 (R/W)            | COMP_DCYCLE   | Compute Datacycle.                                                  |

## DDR ROOT Module Control Register

Figure 10-43: DMC\_DDR\_ROOT\_CTL Register Diagram

<!-- image -->

Table 10-49: DMC\_DDR\_ROOT\_CTL Register Fields

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                              |
|--------------------|------------------|------------------------------------------------------------------------------------------------------|
| 26 (R/W)           | CALIB_MODE_B     | Select control bits for CA lane from scratch registers instead of ZQ_CTL registers.                  |
| 21 (R/W)           | TRIG_RD_XFER_ALL | All Lane Read Status. Software trigger to read the status data from all lanes.                       |
| 18 (R/W)           | TRIG_WR_XFER_L1  | Write Transfer from Root to Lane1. Software trigger for a write transfer from the root to lane 1.    |
| 17 (R/W)           | TRIG_WR_XFER_L0  | Write Transfer from Root to Lane 0. Software trigger for a write transfer from the root to lane 0.   |
| 16 (R/W)           | TRIG_WR_XFER_CA  | Write Transfer from Root to CA Lane. Software trigger for a write transfer from the root to CA lane. |

Table 10-49: DMC\_DDR\_ROOT\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                   |
|--------------------|-----------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| 15:14 (R/W)        | MF_SEL          | SPI Master Frequency Divider Select. The DMC_DDR_ROOT_CTL.MF_SEL bit selects the SPI master frequency divider for root to lane transfers. |
| 13 (R/W)           | SW_REFRESH      | Refresh Lane DLL Code.                                                                                                                    |
| 12:10 (R/W)        | PIPE_OFSTDCYCLE | Pipeline offset for PHYC_DATACYCLE.                                                                                                       |
| 9:6 (R/W)          | CPHY_DCYCLE_VAL | Bypass Value for CPHY_DATACYCLE.                                                                                                          |
| 5 (R/W)            | CPHY_DCYCLE_BYP | Bypass Enable for CPHY_DATACYCLE.                                                                                                         |
| 4:1 (R/W)          | PHYC_DCYCLE_VAL | Bypass Value for PHYC_DATACYCLE.                                                                                                          |
| 0 (R/W)            | PHYC_DCYCLE_BYP | Bypass Enable for PHYC_DATACYCLE.                                                                                                         |

## Scratch Register 2

Figure 10-44: DMC\_DDR\_SCRATCH\_2 Register Diagram

<!-- image -->

Table 10-50: DMC\_DDR\_SCRATCH\_2 Register Fields

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                        |
|--------------------|------------------|----------------------------------------------------------------|
| 31:0 (R/W)         | CALIB_MODE_B_CTL | Control bits for CA lane Completers when CALIB_MODE_B is HIGH. |

## Scratch Register 3

Figure 10-45: DMC\_DDR\_SCRATCH\_3 Register Diagram

<!-- image -->

Table 10-51: DMC\_DDR\_SCRATCH\_3 Register Fields

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                        |
|--------------------|------------------|----------------------------------------------------------------|
| 31:0 (R/W)         | CALIB_MODE_B_CTL | Control bits for CA lane Completers when CALIB_MODE_B is HIGH. |

## Scratch Register 4

Figure 10-46: DMC\_DDR\_SCRATCH\_4 Register Diagram

<!-- image -->

Table 10-52: DMC\_DDR\_SCRATCH\_4 Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                   |
|--------------------|----------------|-------------------------------------------|
| 31:0 (R/W)         | WR_LVL_CODE_L0 | Write leveling code read back from Lane0. |

## Scratch Register 5

Figure 10-47: DMC\_DDR\_SCRATCH\_5 Register Diagram

<!-- image -->

Table 10-53: DMC\_DDR\_SCRATCH\_5 Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                   |
|--------------------|----------------|-------------------------------------------|
| 31:0 (R/W)         | WR_LVL_CODE_L1 | Write leveling code read back from Lane1. |

## Scratch Register 6

Figure 10-48: DMC\_DDR\_SCRATCH\_6 Register Diagram

<!-- image -->

Table 10-54: DMC\_DDR\_SCRATCH\_6 Register Fields

| Bit No. (Access)   | Bit Name        | Description/Enumeration                              |
|--------------------|-----------------|------------------------------------------------------|
| 31:0 (R/W)         | CALIB_CODE_STAT | Calibration code read back from CALIB PAD Completer. |

## Scratch Register 7

Figure 10-49: DMC\_DDR\_SCRATCH\_7 Register Diagram

<!-- image -->

Table 10-55: DMC\_DDR\_SCRATCH\_7 Register Fields

| Bit No. (Access)   | Bit Name        | Description/Enumeration                              |
|--------------------|-----------------|------------------------------------------------------|
| 31:0 (R/W)         | CALIB_CODE_STAT | Calibration code read back from CALIB PAD Completer. |

## DDR Calibration Control Register 0

Figure 10-50: DMC\_DDR\_ZQ\_CTL0 Register Diagram

<!-- image -->

Table 10-56: DMC\_DDR\_ZQ\_CTL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:16 (R/W)        | IMPRTT     | Data/DQS ODT. Desired ODT in ohms for Data and DQS pads.                                                                                       |
| 15:8 (R/W)         | IMPWRDQ    | Data/DQS/DM/CLK Drive Strength. Desired drive strength in ohms for Data, DQS, DM, and clock pads. A drive strength of 100 ohms is recommended. |
| 7:0 (R/W)          | IMPWRADD   | Address/Command Drive Strength. Desired drive strength in ohms for address and command pads. A drive strength of 100 ohms is recommended.      |

## DDR Calibration Control Register 1

<!-- image -->

BSCAN Mode

Figure 10-51: DMC\_DDR\_ZQ\_CTL1 Register Diagram

Table 10-57: DMC\_DDR\_ZQ\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------|
| 22 (R/W)           | USEFNBUS   | BSCAN Mode. The bit controls whether to use bypass codes or functional codes in BSCAN mode. |
| 12 (R/W)           | PDCAL      | Calibration Pad Power Down. Software bypass to power down the calibration pad.              |
| 11:6 (R/W)         | BYPRDPD    | Pull-down Termination Bypass Code.                                                          |
| 5:0 (R/W)          | BYPRDPU    | Pull-up Termination Bypass Code.                                                            |

## DDR Calibration Control Register 2

Figure 10-52: DMC\_DDR\_ZQ\_CTL2 Register Diagram

<!-- image -->

Table 10-58: DMC\_DDR\_ZQ\_CTL2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                              |
|--------------------|------------|------------------------------------------------------|
| 30 (R/W)           | PDCALEN    | Driver Pull-down Strength Calibration Enable.        |
| 29 (R/W)           | PUCALEN    | Driver Pull-up Strength Calibration Enable.          |
| 28 (R/W)           | CALSTRT    | New Impedance Calibration Enable.                    |
| 27 (R/W)           | CALTYPE    | Calibration Type. Short or Long Calibration          |
| 26:21 (R/W)        | BYPPDDQ    | Bypass Code for DQ,DQS,CLK,DM Pull-down Driver Code. |
| 20:15 (R/W)        | BYPPUDQ    | Bypass Code for DQ,DQS,CLK,DM Pull-up Code.          |

Table 10-58: DMC\_DDR\_ZQ\_CTL2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------|
| 14:9 (R/W)         | BYPPDADD   | Address/Command Bypass Pull-down Code.                                                  |
| 8:3 (R/W)          | BYPPUADD   | Address/Command Bypass Pull-up Codes. Bypass pull-up codes for address and command pads |
| 2 (R/W)            | BYPIMRD    | Bypass Termination Code Enable.                                                         |
| 1 (R/W)            | BYPIMDQ    | DQ/DQS/CLK/DM Bypass Driver Code Enable.                                                |
| 0 (R/W)            | BYPIMAD    | Address/Command Bypass Code Enable. Bypass code enable for address and command pads.    |