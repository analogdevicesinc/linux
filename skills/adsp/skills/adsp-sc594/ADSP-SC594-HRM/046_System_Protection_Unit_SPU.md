# System Protection Unit (SPU)

<!-- source: 046_System_Protection_Unit_SPU.pdf | original pages 2585–2608 -->

## 39   System Protection Unit (SPU)

In a system with multiple system MMR requesters, peripheral configurations can be changed unintentionally, leading to bad data or even system malfunctions. The peripherals are shared resources in the system. The SPU restricts access to certain MMRs, similar to the functionality of a semaphore.

The SPU also protects peripherals based on security settings. It is part of the overall security infrastructure of the processor.

## SPU Features

The SPU has the following features:

- Write-protect system MMR from certain system requesters and core requesters.
- Simultaneously lock multiple peripheral configuration registers through a global lock mechanism.
- Write-protect and block access to its own write-protection registers from other system requesters.
- Defined security privileges to peripherals and system resources.
- Security protection to guard secure peripheral MMRs against non-secure accesses.

## SPU Functional Description

The following sections provide information on the function of the SPU.

## ADSP-2159x\_SC592\_SC594 SPU Register List

The System Protection Unit (SPU) provides a set of registers that can protect system resources from errant writes. The protection categories are global lock (protects configuration registers) and write protect register lock (protects the write protect register). For more information on SPU functionality, see the SPU register descriptions.

Table 39-1: ADSP-2159x\_SC592\_SC594 SPU Register List

| Name          | Description           |
|---------------|-----------------------|
| SPU_CTL       | Control Register      |
| SPU_SECURECHK | Secure Check Register |

Table 39-1: ADSP-2159x\_SC592\_SC594 SPU Register List (Continued)

| Name           | Description                |
|----------------|----------------------------|
| SPU_SECURECTL  | Secure Control Register    |
| SPU_SECUREC[n] | Secure Core Registers      |
| SPU_SECUREP[n] | Secure Peripheral Register |
| SPU_STAT       | Status Register            |
| SPU_WP[n]      | Write Protect Register n   |

## ADSP-2159x\_SC592\_SC594 SPU Interrupt List

Table 39-2: ADSP-2159x\_SC592\_SC594 SPU Interrupt List

|   Interrupt ID | Name     | Description    | Sensitivity   | DMA Channel   |
|----------------|----------|----------------|---------------|---------------|
|            216 | SPU0_INT | SPU0 Interrupt | Level         |               |

## Peripheral Register Write Protection

The SPU has a write-protection register ( SPU\_WP[n] ) associated with each peripheral. Each of these write-protection registers has the exact same bits that correspond to a particular SMMR requester (for example, Core 0, MDMA). When the bits are set, the SPU locks the corresponding SMMR requesters from accessing the register address space of the associated peripheral. The bits in the register can be cleared to allow access to the registers of the peripheral again. When the SPU initializes the write-protection register, any writes that are in-progress complete before the SPU blocks subsequent writes.

In the SPU Write Protect Registers figure, each write-protect register in the SPU is associated with a particular peripheral.

Figure 39-1: SPU Write Protect Registers

<!-- image -->

Figure: SPU Write-protect registers Each write-protect register in the SPU is associated with a particular peripheral. In the figure, a write-protect register in the SPU module blocks write-attempts to the MMR space of the associated peripheral. The bits in the write-protect register specify from which requesters to block write-access.

NOTE: A SPU write protection register ( SPU\_WP[n] ) exists for the SPU alone. If all defined bits are set in this register for the SPU, any configurations in the SPU are locked and cannot be changed. Only a system reset can restore access to the SPU.

Figure: SPU Write-Protect Register Blocking Access from System Master 0 and Core Master 1 Figure 39-2: SPU Write-Protect Register Blocking Access from System Requester 0 and Core Requester 1

<!-- image -->

## Global Locking

A write-protect register in the SPU blocks write-attempts to the associated peripheral's MMR  space.  The bits in the write protect register specify which masters to block write-access from.

The SPU also has global locking capability. When enabled by setting SPU\_CTL.GLCK bit field to a value other than 0xAD, a system-wide global lock signal is active. Some peripherals have a lock enable bit in their control register. When this bit is set, the peripheral recognizes the global lock signal and blocks further write-accesses to its own control register. Access to the configuration register of the peripheral is enabled when the global lock is turned off in the SPU.

The Global Locking figure is a conceptual diagram. The diagram shows how the SPU module (or any peripheral) blocks any write attempts to its control register when:

- The global lock signal from the SPU is active, and
- The global lock enable bit is set in the control register of the peripheral

Figure 39-3: Global Locking

<!-- image -->

Figure: Conceptual Figure of Global Locking The SPU can write-protect its own registers. When the SPU\_CTL.WPLCK bit is set and global locking is enabled, the SPU blocks accesses to the SPU write-protection registers. To enable write access to the write-protection registers in the SPU, disable the global locking.

Peripheral blocks write attempts to Control MMR if the

Global Lock bit is set in the peripheral's Control MMR

and the Global Lock Signal is active from the SPU

-

## SPU Block Diagram

The SPU System-Level Block Diagram shows a system-level block diagram of where the SPU is located in the system. It resides between the SMMR interface and the system crossbar. Depending on the configuration of the SPU write-protect registers, it can block access to some peripherals from certain SMMR requesters.

Figure 39-4: SPU System-Level Block Diagram

<!-- image -->

## SPU Architectural Concepts

As shown in the block diagram, the SPU sits between the system crossbar (SCB) and the SMMR interface to the peripherals. The SPU gates any MMR access to any peripheral from any requester that comes through the SCB. Depending on the configuration of the write-protection registers in the SPU, the SPU does or does not allow the MMR write to go through.

The SPU also checks whether the transaction is a secure or non-secure transaction and blocks it according to the configured security setting for the target destination. A secure requester can generate secure read or secure write transactions which can access secure or non-secure completers. A non-secure requester can generate non-secure read or non-secure write transactions and can only access non-secure completers.

## SPU Event Control

The system protection unit provides write-protection against MMRs peripherals and its own write-protect registers. If a write attempt is made to any locked MMR peripheral the SPU has write-protected, it blocks the write. The SPU generates a bus error to the requester that attempted the write. That requester does or does not generate an event, based on the returned error.

The SPU can be configured to generate an interrupt for the write-protection violation by setting the SPU\_CTL.PINTEN bit. The SPU can also be configured to generate an interrupt for a security violation by setting the SPU\_SECURECTL.SINTEN bit. If either one or both bits is triggered, the SPU\_STAT.VIRQ bit is set.

The SPU can also lock its own registers from write attempts. If a write-attempt is made to a locked register in the SPU, the SPU blocks it and records it as an error in the SPU\_STAT.LWERR bit. Again, the SPU generates a bus error to the requester that attempted the write.

The requester does or does not generate an event, based on the returned error.

The SPU does not generate an event for a blocked write access to an SPU register. If the SPU\_CTL.PINTEN bit is set, the SPU triggers an interrupt for this blocked access attempt.

The global lock is enabled by setting the SPU\_CTL.GLCK bit to something other than 0xAD. If the lock bit is set in that same configuration register, a peripheral can block write access to its configuration register. When the SPU blocks a write attempt, the peripheral logs and reports the failed attempt. The SPU is unaware and therefore does not provide any indication of a failed write attempt to the configuration register of the peripheral.

## SPU Programming Model

The system protection unit (SPU) consists of write-protect and access-protect registers. Each one corresponds to a different peripheral instance. Bits in the write-protect registers correspond to system requesters that can modify the MMR contents of the peripherals. By writing to these write-protect registers, the corresponding memory-mapped registers of the peripheral are write-protected against requesters whose bits in the write-protect register are set.

The SPU globally locks the control register of the peripheral. Peripherals that support this feature have a lock enable bit in their control register. The peripheral blocks any additional write attempts to its control register from any requester when:

- The global lock signal is active from the SPU, and
- The lock enable bit of the peripheral is set

If the lock enable bit of a peripheral is not set and the global lock signal is active, access to that control register of the peripheral is still allowed. T o grant access again, disable the global lock signal from the SPU by writing the value 0xAD into the SPU\_CTL.GLCK bit field.

Another protection mechanism that the SPU offers is write-protection against the write-protection registers. If the write protect register lock bit ( SPU\_CTL.WPLCK ) is set and the global lock signal is active, writes to the writeprotect registers of the SPU are blocked. To reenable access to the write-protect registers in the SPU, deactivate the global lock signal by writing 0xAD into the SPU\_CTL.GLCK bit field.

For security, the SPU provides a set of SPU\_SECUREC[n] registers (one for each processor core from Analog Devices) to configure their security settings. The SPU also provides a set of SPU\_SECUREP[n] registers (one for each peripheral instance) to configure their security settings.

## Enabling and Disabling the SPU

The SPU is always operating. There are no bits to enable or disable the SPU. The SPU configuration can be updated at any time. Any ongoing transactions finish before a new configuration is in effect. By default, the SPU does not write-protect any of the MMRs.

## Write-Protecting the SPU

The SPU is treated like any other peripheral in the system. As such, the SPU also has an associated write-protection register. If this write-protection register is configured to block all writes from all requesters, any SPU configuration remains the same until the next system reset.

## Checking the Security State

In some cases while running a peripheral, an application system requester does not know whether they are a secure requester generating secure transactions or not. The SPU provides a means for checking the security state of the requester through the SPU\_SECURECHK register. When read by a secure requester, the register reads 0xFFFFFFFF and when read by a non-secure requester, the value is 0x00000000.

## SPU Mode Configuration

The SPU can provide address range-wide protection by write-protecting the peripherals MMR address range from system MMR requesters. It can also provide register wide protection using global locking. Peripherals that support this feature can enable it in their respective configuration register. When the SPU enables the global lock signal, all subsequent writes to the configuration register of the peripheral are blocked until the global lock signal is deasserted. Similarly, the write-protection registers of the SPU can be write-protected using the global lock signal as well. The SPU uses all these modes of operation together.

## Locking Write-Protect Registers

Use the following steps to lock (write-protect) a register.

1. Set the SPU\_CTL.WPLCK bit and configure the SPU\_CTL.GLCK field to something other than 0xAD.

The SPU write-protect registers are blocked from further write accesses.

## Protecting a Peripheral

Use the following procedure to protect a peripheral.

1. Determine which peripheral needs protection and locate the corresponding write-protect register ( SPU\_WP[n] ) in the SPU. See the Write-Protect and Secure Peripheral Registers section.
2. Determine the SMMR requesters from which the peripheral needs protection. Then, set the corresponding bit or bits in the write-protect register ( SPU\_WP[n] ) for the peripheral. See the Write-Protect and Secure Peripheral Registers section.

After setting the write-protect register for the particular peripheral, the identified SMMR requesters are blocked from writing to any MMR in the address space of the peripheral. This block remains in place until the bits in the write-protect register are cleared.

## Configuring Security Privileges of a Peripheral

Use the following procedure to configure the security privileges of a peripheral.

1. Determine the peripheral and its corresponding secure peripheral register ( SPU\_SECUREP[n] ) in the SPU. See the Write-Protect and Secure Peripheral Registers section.
2. If the peripheral is to be a secure completer that only accepts secure transactions, set bit 0 ( SPU\_SECUREP[n].SSEC ).
3. If the peripheral is to be a secure requester that generates secure transactions (keeping in mind not all peripherals can be requesters), set bit 1 ( SPU\_SECUREP[n].MSEC ).

This procedure sets the security privileges of a peripheral.

NOTE: Only a secure requester can set security privileges, keeping the chain of trust intact. If a non-secure requester configures the security privileges, it can undermine security protection.

## ADSP-2159x\_SC592\_SC594 SPU Register Descriptions

System Protection Unit (SPU) contains the following registers.

Table 39-3: ADSP-2159x\_SC592\_SC594 SPU Register List

| Name           | Description                |
|----------------|----------------------------|
| SPU_CTL        | Control Register           |
| SPU_SECURECHK  | Secure Check Register      |
| SPU_SECURECTL  | Secure Control Register    |
| SPU_SECUREC[n] | Secure Core Registers      |
| SPU_SECUREP[n] | Secure Peripheral Register |
| SPU_STAT       | Status Register            |
| SPU_WP[n]      | Write Protect Register n   |

## Control Register

The SPU control register ( SPU\_CTL ) provides a global lock for configuration registers as well as control for locking the write protect ( SPU\_WP[n] ) registers. It also controls the generation of an interrupt to report blocked accesses.

<!-- image -->

Write Protect Register Lock

Figure 39-5: SPU\_CTL Register Diagram

Table 39-4: SPU\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | WPLCK      | Write Protect Register Lock. When the SPU_CTL.WPLCK bit is set in combination with the SPU_CTL.GLCK bit, writes to the SPU's write protect registers are blocked and return an error. 0 Disable                               |
| 14 (R/W)           | PINTEN     | Protection Violation Interrupt Enable. When the SPU_CTL.PINTEN bit is set (=1), a block of any transaction according to the configured settings produces an interrupt.                                                        |
| 7:0 (R/W)          | GLCK       | Global Lock. The SPU_CTL.GLCK controls the global lock signal. The global lock signal provides register-based write protection. Writing 0xAD to this field disables the lock, and writ- ing any other value enables the lock. |

## Secure Check Register

The SPU\_SECURECHK register reads by secure requesters return 0xFFFFFFFF . Reads by non-secure requesters return 0x00000000.

Figure 39-6: SPU\_SECURECHK Register Diagram

<!-- image -->

Table 39-5: SPU\_SECURECHK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | DATA       | Read Data. The SPU_SECURECHK.DATA bit field performs reads. Reads by secure requesters return 0xFFFFFFFF. Reads by non-secure requesters return 0x00000000. |
| (R/NW)             |            |                                                                                                                                                             |

## Secure Control Register

The SPU Secure Control Register ( SPU\_SECURECTL ) allows the user to lock write access to all the SPU\_SECUREC[n] and SPU\_SECUREP[n] registers as well as configure the interrupt generation in an event of a security error. It also allows bulk clear of the SSEC bits and/or MSEC bits in the SPU\_SECUREP[n] registers.

Figure 39-7: SPU\_SECURECTL Register Diagram

<!-- image -->

Table 39-6: SPU\_SECURECTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | SCRLCK     | Secure Register Lock. When the SPU_SECURECTL.SCRLCK bit is set in combination with the SPU_CTL.GLCK bit, writes to the Security Configuration registers ( SPU_SECUREC[n] and SPU_SECUREP[n] ) are blocked and return an error which is captured in the SPU_STAT.LWERR bit. |
| 14 (R/W)           | SINTEN     | Secure Violation Interrupt Enable. The SPU_SECURECTL.SINTEN bit generates an interrupt if a security violation was captured. Interrupt status is provided in the SPU_STAT.VIRQ bit.                                                                                        |
| 5 (R0/W)           | MSECCLR    | Requester Secure Clear. When the SPU_SECURECTL.MSECCLR bit is set, the SPU_SECUREP[n].MSEC bits in all SPU_SECUREP[n] registers are cleared. The SPU_SECURECTL.MSECCLR bit always reads back as a 0. 0 No Action 1 Clear All Requester Secure Control Bits                 |

Table 39-6: SPU\_SECURECTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R0/W)           | SSECCLR    | Completer Secure Clear. When the SPU_SECURECTL.SSECCLR bit is set, the SPU_SECUREP[n].SSEC bits in all SPU_SECUREP[n] registers are cleared. The SPU_SECURECTL.SSECCLR bit always reads back as a 0. |
| 4 (R0/W)           | SSECCLR    | 0 No Action                                                                                                                                                                                          |
| 4 (R0/W)           | SSECCLR    | 1 Clear All Completer Secure Control Bits                                                                                                                                                            |

## Secure Core Registers

A SPU register exists for every DSP core in the system. The bits enable or disable security for features in the core.

Figure 39-8: SPU\_SECUREC[n] Register Diagram

<!-- image -->

Table 39-7: SPU\_SECUREC[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | CSEC[n]    | Core Secure. The SPU_SECUREC[n].CSEC[n] bit controls whether non-secure accesses are al- lowed to L1 memory of the processor core. When =1, the core (as a completer) is set as secure meaning only secure transactions are allowed to L1. Disable |
| 0 (R/W)            | CSEC[n]    | 0                                                                                                                                                                                                                                                  |
| 0 (R/W)            | CSEC[n]    | 1 Enable                                                                                                                                                                                                                                           |

## Secure Peripheral Register

In the system, each SPU\_SECUREP[n] register is assigned to a specific MMR address range associated with one peripheral. Each SPU\_SECUREP[n] has a Completer Secure (SSEC) bit and a Requester Secure (MSEC) bit. When the SSEC bit is set, the SPU will only allow secure requesters generating secure transactions to access the peripheral's MMR address space. When the MSEC bit is set, the associated peripheral will be secure and will generate secure transactions.

Figure 39-9: SPU\_SECUREP[n] Register Diagram

<!-- image -->

Table 39-8: SPU\_SECUREP[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | MSEC       | Requester Secure Enable. The SPU_SECUREP[n].MSEC bit controls whether the peripheral generates secure transactions as a requester. When clear (=0), the peripheral generates non-secure trans- actions as a requester (if applicable). When set (=1), the peripheral generates secure transactions as a requester. 0 Disable                                      |
| 0 (R/W)            | SSEC       | Completer Secure Enable. The SPU_SECUREP[n].SSEC bit controls whether the peripheral is protected from non-secure transactions. When clear (=0), the security status of the transaction is ignored. When set (=1), only secure transactions are allowed to access the address space of the peripheral and non-secure transactions are blocked. 0 Disable 1 Enable |

## Status Register

The SPU\_STAT register indicates if there have been any errors, active interrupts and global lock status.

Figure 39-10: SPU\_STAT Register Diagram

<!-- image -->

Table 39-9: SPU\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | Lock Write Error. The SPU_STAT.LWERR indicates whether there has been an attempted write to a register in the SPU with its lock bit ( SPU_CTL.WPLCK or SCRLCK) set while SPU_CTL.GLCK was asserted. This bit is W1C. |
| 30 (R/W1C)         | ADDRERR    | Address Error. The SPU_STAT.ADDRERR indicates whether there has been an attempted write to a read-only register or an access an invalid address in the SPU MMRaddress range. This bit is W1C.                        |
| 12 (R/W1C)         | VIRQ       | Violation Interrupt Request. The SPU_STAT.VIRQ bit indicates that a security and/or protection violation has been detected and interrupt asserted. This is a W1C bit. 0 Inactive                                     |
| 0 (R/NW)           | GLCK       | Global Lock Status. The SPU_STAT.GLCK indicates whether the global lock is enabled or disabled. 0 Disabled (global_lock=0)                                                                                           |

## Write Protect Register n

In the system, each SPU\_WP[n] register is assigned to a specific MMR address range associated with one peripheral. When the appropriate bits are set, writes to the peripheral from a specific requester are blocked and an error is returned to the requester. For more information, see the processor specific additional information for the SPU\_WP[n] register.

Figure 39-11: SPU\_WP[n] Register Diagram

<!-- image -->

Table 39-10: SPU\_WP[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18:16 (R/W)        | SM[n]      | System Requester x Write Protect Enable. The SPU_WP[n].SM[n] bits correspond to different system requesters in the sys- tem. When a particular bit is set in this field, the corresponding system requester can- not write to the corresponding peripheral's MMRaddress space. The write attempt is blocked by the SPU. |
| 2:0 (R/W)          | CM[n]      | Core Requester x Write Protect Enable. The SPU_WP[n].CM[n] bits correspond to different cores in the system. When a particular bit is set in this field, the corresponding core cannot write to the correspond- ing peripheral's MMRaddress space. The write attempt is blocked by the SPU.                             |

## Write-Protect, Secure Peripheral, and Secure Core Registers

The SPU consists of a collection of write-protect registers each of which are associated with a specific peripheral or target. The SPU also has a collection of secure peripheral registers which are also associated with specific peripherals. The table gives the write-protect register and secure peripheral number for each of the peripherals that are provided with write protection and security through the SPU. The SPU for the processor is configured with 136 write-protect registers and also 136 secure peripheral registers. The number corresponding to a peripheral correlates to both the Write-Protect register and the Secure Peripheral register.

For each processor, there are different numbers of controllers that are able to access the SMMR space. The SPU\_WPn.CMn and SPU\_WPn.SMn Bits table shows which bits enable the protection against which controller.

The SPU\_WP[n] register shows which bits enable the protection against which controller.

Table 39-11: SPU\_WPn.CMn and SPU\_WPn.SMn Bits

| Bit Name   | Description         |
|------------|---------------------|
| CM_WP[0]   | Arm Core M1 PORT    |
| CM_WP[1]   | SHARC Core0DPORTMMR |
| CM_WP[2]   | SHARC Core1DPORTMMR |
| SM_WP[0]   | DBG                 |
| SM_WP[1]   | EnhancedBWMDMA      |
| SM_WP[2]   | EnhancedBWMDMA1     |

For each peripheral, there is a corresponding write-protect register, SPU\_WP[n] , and secure peripheral register, SPU\_SECUREP[n] . The table shows the write-protect register and secure peripheral number for each peripheral.

Table 39-12: Write-Protect Register and Secure Peripheral Number(n)

|   No. | Peripheral/Block Name   | SPU WPn Mapping   | SECUREPn Mapping   | Controller Capable   |
|-------|-------------------------|-------------------|--------------------|----------------------|
|     0 | MMRGGPVSPACE            | WP0               | SECUREP0           | N/A                  |
|     1 | DMC0_PRG_CDC_BRI DGE    | WP1               | SECUREP1           | N/A                  |
|     2 | SPIF GPV SPACE          | WP2               | SECUREP2           | N/A                  |
|     3 | LP0                     | WP3               | SECUREP3           | N/A                  |
|     4 | LP1                     | WP4               | SECUREP4           | N/A                  |
|     5 | LP0 DDE                 | WP5               | SECUREP5           | Yes                  |
|     6 | LP1 DDE                 | WP6               | SECUREP6           | Yes                  |
|     7 | TWI3                    | WP7               | SECUREP7           | N/A                  |
|     8 | TWI4                    | WP8               | SECUREP8           | N/A                  |
|     9 | TWI5                    | WP9               | SECUREP9           | N/A                  |
|    10 | TWI0                    | WP10              | SECUREP10          | N/A                  |
|    11 | TWI1                    | WP11              | SECUREP11          | N/A                  |
|    12 | TWI2                    | WP12              | SECUREP12          | N/A                  |
|    13 | SPORT0A                 | WP13              | SECUREP13          | N/A                  |
|    14 | SPORT0B                 | WP14              | SECUREP14          | N/A                  |
|    15 | SPORT1A                 | WP15              | SECUREP15          | N/A                  |
|    16 | SPORT1B                 | WP16              | SECUREP16          | N/A                  |
|    17 | SPORT2A                 | WP17              | SECUREP17          | N/A                  |

Table 39-12: Write-Protect Register and Secure Peripheral Number(n) (Continued)

|   No. | Peripheral/Block Name   | SPU WPn Mapping   | SECUREPn Mapping   | Controller Capable   |
|-------|-------------------------|-------------------|--------------------|----------------------|
|    18 | SPORT2B                 | WP18              | SECUREP18          | N/A                  |
|    19 | SPORT3A                 | WP19              | SECUREP19          | N/A                  |
|    20 | SPORT3B                 | WP20              | SECUREP20          | N/A                  |
|    21 | SPORT4A                 | WP21              | SECUREP21          | N/A                  |
|    22 | SPORT4B                 | WP22              | SECUREP22          | N/A                  |
|    23 | SPORT5A                 | WP23              | SECUREP23          | N/A                  |
|    24 | SPORT5B                 | WP24              | SECUREP24          | N/A                  |
|    25 | SPORT6A                 | WP25              | SECUREP25          | N/A                  |
|    26 | SPORT6B                 | WP26              | SECUREP26          | N/A                  |
|    27 | SPORT7A                 | WP27              | SECUREP27          | N/A                  |
|    28 | SPORT7B                 | WP28              | SECUREP28          | N/A                  |
|    29 | UART0                   | WP29              | SECUREP29          | N/A                  |
|    30 | UART1                   | WP30              | SECUREP30          | N/A                  |
|    31 | UART2                   | WP31              | SECUREP31          | N/A                  |
|    32 | UART3                   | WP32              | SECUREP32          | N/A                  |
|    33 | PORTA                   | WP33              | SECUREP33          | N/A                  |
|    34 | PORTB                   | WP34              | SECUREP34          | N/A                  |
|    35 | PORTC                   | WP35              | SECUREP35          | N/A                  |
|    36 | PORTD                   | WP36              | SECUREP36          | N/A                  |
|    37 | PORTE                   | WP37              | SECUREP37          | N/A                  |
|    38 | PORTF                   | WP38              | SECUREP38          | N/A                  |
|    39 | PORTG                   | WP39              | SECUREP39          | N/A                  |
|    40 | PORTH                   | WP40              | SECUREP40          | N/A                  |
|    41 | PORTI                   | WP41              | SECUREP41          | N/A                  |
|    42 | PADS                    | WP42              | SECUREP42          | N/A                  |
|    43 | PINT0                   | WP43              | SECUREP43          | N/A                  |
|    44 | PINT1                   | WP44              | SECUREP44          | N/A                  |
|    45 | PINT2                   | WP45              | SECUREP45          | N/A                  |
|    46 | PINT3                   | WP46              | SECUREP46          | N/A                  |
|    47 | PINT4                   | WP47              | SECUREP47          | N/A                  |
|    48 | PINT5                   | WP48              | SECUREP48          | N/A                  |

Table 39-12: Write-Protect Register and Secure Peripheral Number(n) (Continued)

|   No. | Peripheral/Block Name   | SPU WPn Mapping   | SECUREPn Mapping   | Controller Capable   |
|-------|-------------------------|-------------------|--------------------|----------------------|
|    49 | PINT6                   | WP49              | SECUREP49          | N/A                  |
|    50 | PINT7                   | WP50              | SECUREP50          | N/A                  |
|    51 | reserved                | reserved          | reserved           | reserved             |
|    52 | reserved                | reserved          | reserved           | reserved             |
|    53 | WDT0                    | WP53              | SECUREP53          | N/A                  |
|    54 | WDT1                    | WP54              | SECUREP54          | N/A                  |
|    55 | WDT2                    | WP55              | SECUREP55          | N/A                  |
|    56 | CNT0                    | WP56              | SECUREP56          | N/A                  |
|    57 | OTPMMR                  | WP57              | SECUREP57          | N/A                  |
|    58 | SMPU-OTP                | WP58              | SECUREP58          | N/A                  |
|    59 | reserved                | reserved          | reserved           | reserved             |
|    60 | HADC0                   | WP60              | SECUREP60          | N/A                  |
|    61 | TMU0                    | WP61              | SECUREP61          | N/A                  |
|    62 | TMR                     | WP62              | SECUREP62          | N/A                  |
|    63 | SPORT0A DDE             | WP63              | SECUREP63          | Yes                  |
|    64 | SPORT0B DDE             | WP64              | SECUREP64          | Yes                  |
|    65 | SPORT1A DDE             | WP65              | SECUREP65          | Yes                  |
|    66 | SPORT1B DDE             | WP66              | SECUREP66          | Yes                  |
|    67 | SPORT2A DDE             | WP67              | SECUREP67          | Yes                  |
|    68 | SPORT2B DDE             | WP68              | SECUREP68          | Yes                  |
|    69 | SPORT3A DDE             | WP69              | SECUREP69          | Yes                  |
|    70 | SPORT3B DDE             | WP70              | SECUREP70          | Yes                  |
|    71 | SPORT4A DDE             | WP71              | SECUREP71          | Yes                  |
|    72 | SPORT4B DDE             | WP72              | SECUREP72          | Yes                  |
|    73 | SPORT5A DDE             | WP73              | SECUREP73          | Yes                  |
|    74 | SPORT5B DDE             | WP74              | SECUREP74          | Yes                  |
|    75 | SPORT6A DDE             | WP75              | SECUREP75          | Yes                  |
|    76 | SPORT6B DDE             | WP76              | SECUREP76          | Yes                  |
|    77 | SPORT7A DDE             | WP77              | SECUREP77          | Yes                  |
|    78 | SPORT7B DDE             | WP78              | SECUREP78          | Yes                  |
|    79 | UART0RX DDE             | WP79              | SECUREP79          | Yes                  |

Table 39-12: Write-Protect Register and Secure Peripheral Number(n) (Continued)

|   No. | Peripheral/Block Name   | SPU WPn Mapping   | SECUREPn Mapping   | Controller Capable   |
|-------|-------------------------|-------------------|--------------------|----------------------|
|    80 | UART0TX DDE             | WP80              | SECUREP80          | Yes                  |
|    81 | UART1RX DDE             | WP81              | SECUREP81          | Yes                  |
|    82 | UART1TX DDE             | WP82              | SECUREP82          | Yes                  |
|    83 | UART2RX DDE             | WP83              | SECUREP83          | Yes                  |
|    84 | UART2TX DDE             | WP84              | SECUREP84          | Yes                  |
|    85 | UART3RX DDE             | WP85              | SECUREP85          | Yes                  |
|    86 | UART3TX DDE             | WP86              | SECUREP86          | Yes                  |
|    87 | PPI0 DDE0               | WP87              | SECUREP87          | Yes                  |
|    88 | PPI0 DDE1               | WP88              | SECUREP88          | Yes                  |
|    89 | OSPI                    | WP89              | SECUREP89          | N/A                  |
|    90 | reserved                | reserved          | reserved           | reserved             |
|    91 | EPPI0                   | WP91              | SECUREP91          | N/A                  |
|    92 | reserved                | reserved          | reserved           | reserved             |
|    93 | SPI0TX DDE              | WP93              | SECUREP93          | Yes                  |
|    94 | SPI0RX DDE              | WP94              | SECUREP94          | Yes                  |
|    95 | SPI1TX DDE              | WP95              | SECUREP95          | Yes                  |
|    96 | SPI1RX DDE              | WP96              | SECUREP96          | Yes                  |
|    97 | SPI2TX DDE              | WP97              | SECUREP97          | Yes                  |
|    98 | SPI2RX DDE              | WP98              | SECUREP98          | Yes                  |
|    99 | SPI3TX DDE              | WP99              | SECUREP99          | Yes                  |
|   100 | SPI3RX DDE              | WP100             | SECUREP100         | Yes                  |
|   101 | SPI0                    | WP101             | SECUREP101         | N/A                  |
|   102 | SPI1                    | WP102             | SECUREP102         | N/A                  |
|   103 | SPI2                    | WP103             | SECUREP103         | N/A                  |
|   104 | SPI3                    | WP104             | SECUREP104         | N/A                  |
|   105 | EMAC0                   | WP105             | SECUREP105         | Yes                  |
|   106 | EMAC1                   | WP106             | SECUREP106         | Yes                  |
|   107 | CAN0                    | WP107             | SECUREP107         | N/A                  |
|   108 | CAN1                    | WP108             | SECUREP108         | N/A                  |
|   109 | DMC0                    | WP109             | SECUREP109         | N/A                  |
|   110 | DMC0 PHY                | WP110             | SECUREP110         | N/A                  |

Table 39-12: Write-Protect Register and Secure Peripheral Number(n) (Continued)

|   No. | Peripheral/Block Name   | SPU WPn Mapping   | SECUREPn Mapping   | Controller Capable   |
|-------|-------------------------|-------------------|--------------------|----------------------|
|   111 | DMC0 DFT                | WP111             | SECUREP111         | N/A                  |
|   112 | L2CTL0                  | WP112             | SECUREP112         | N/A                  |
|   113 | SMPU L2CTL0 CL2 0       | WP113             | SECUREP113         | N/A                  |
|   114 | SMPU L2CTL0 DL2 0       | WP114             | SECUREP114         | N/A                  |
|   115 | SMPU L2CTL0 CL2 1       | WP115             | SECUREP115         | N/A                  |
|   116 | SMPU L2CTL0 DL2 1       | WP116             | SECUREP116         | N/A                  |
|   117 | SMPU L2CTL0 CL2 2       | WP117             | SECUREP117         | N/A                  |
|   118 | SEC0                    | WP118             | SECUREP118         | N/A                  |
|   119 | TRU0                    | WP119             | SECUREP119         | N/A                  |
|   120 | SPU0                    | WP120             | SECUREP120         | N/A                  |
|   121 | RCU0                    | WP121             | SECUREP121         | N/A                  |
|   122 | CGU0                    | WP122             | SECUREP122         | N/A                  |
|   123 | CGU1                    | WP123             | SECUREP123         | N/A                  |
|   124 | CDU0                    | WP124             | SECUREP124         | N/A                  |
|   125 | DPM0                    | WP125             | SECUREP125         | N/A                  |
|   126 | PLL0                    | WP126             | SECUREP126         | N/A                  |
|   127 | PLL1                    | WP127             | SECUREP127         | N/A                  |
|   128 | SWU_L2CTL0_CL2_0        | WP128             | SECUREP128         | N/A                  |
|   129 | SWU_L2CTL0_DL2_0        | WP129             | SECUREP129         | N/A                  |
|   130 | SWU_L2CTL0_CL2_1        | WP130             | SECUREP130         | N/A                  |
|   131 | SWU_L2CTL0_DL2_1        | WP131             | SECUREP131         | N/A                  |
|   132 | SWU SMMR                | WP132             | SECUREP132         | N/A                  |
|   133 | SWU_L2CTL0_CL2_2        | WP133             | SECUREP133         | N/A                  |
|   134 | MDMA2                   | WP134             | SECUREP134         | Yes                  |
|   135 | MDMA3                   | WP135             | SECUREP135         | Yes                  |
|   136 | MDMA7                   | WP136             | SECUREP136         | Yes                  |
|   137 | MLB0                    | WP137             | SECUREP137         | Yes                  |
|   138 | SWU DMC0                | WP138             | SECUREP138         | N/A                  |
|   139 | SMPU DMC0               | WP139             | SECUREP139         | N/A                  |
|   140 | SMPU SPI2               | WP140             | SECUREP140         | N/A                  |
|   141 | MEC0                    | WP141             | SECUREP141         | N/A                  |

Table 39-12: Write-Protect Register and Secure Peripheral Number(n) (Continued)

|   No. | Peripheral/Block Name   | SPU WPn Mapping   | SECUREPn Mapping   | Controller Capable   |
|-------|-------------------------|-------------------|--------------------|----------------------|
|   142 | MEC1                    | WP142             | SECUREP142         | N/A                  |
|   143 | MEC2                    | WP143             | SECUREP143         | N/A                  |
|   144 | CRC0                    | WP144             | SECUREP144         | N/A                  |
|   145 | CRC1                    | WP145             | SECUREP145         | N/A                  |
|   146 | MDMA0 DDE0 CRC0         | WP146             | SECUREP146         | Yes                  |
|   147 | MDMA0 DDE1 CRC0         | WP147             | SECUREP147         | Yes                  |
|   148 | MDMA1 DDE0 CRC1         | WP148             | SECUREP148         | Yes                  |
|   149 | MDMA1 DDE1 CRC1         | WP149             | SECUREP149         | Yes                  |
|   150 | MDMA4 DDE0 CRC2         | WP150             | SECUREP150         | Yes                  |
|   151 | MDMA4 DDE1 CRC2         | WP151             | SECUREP151         | Yes                  |
|   152 | MDMA5 DDE0 CRC3         | WP152             | SECUREP152         | Yes                  |
|   153 | MDMA5 DDE1 CRC3         | WP153             | SECUREP153         | Yes                  |
|   154 | SWU SPI2                | WP154             | SECUREP154         | N/A                  |
|   155 | MISC REG                | WP155             | SECUREP155         | N/A                  |
|   156 | CRC2                    | WP156             | SECUREP156         | N/A                  |
|   157 | CRC3                    | WP157             | SECUREP157         | N/A                  |
|   158 | MDMA6                   | WP158             | SECUREP158         | Yes                  |
|   159 | STM0                    | WP159             | SECUREP159         | N/A                  |
|   160 | GIC_Port0               | WP160             | SECUREP160         | N/A                  |
|   161 | GIC_Port1               | WP161             | SECUREP161         | N/A                  |
|   162 | SH1_IIR1                | WP162             | SECUREP162         | N/A                  |
|   163 | SH1_IIR2                | WP163             | SECUREP163         | N/A                  |
|   164 | SH1_IIR3                | WP164             | SECUREP164         | N/A                  |
|   165 | SH1_FIR                 | WP165             | SECUREP165         | N/A                  |
|   166 | SH1_IIR0                | WP166             | SECUREP166         | N/A                  |
|   167 | SH0_IIR1                | WP167             | SECUREP167         | N/A                  |
|   168 | SH0_IIR2                | WP168             | SECUREP168         | N/A                  |
|   169 | SH0_IIR3                | WP169             | SECUREP169         | N/A                  |
|   170 | SH0_FIR                 | WP170             | SECUREP170         | N/A                  |
|   171 | SH0_IIR0                | WP171             | SECUREP171         | N/A                  |
|   172 | USB0                    | WP172             | SECUREP172         | Yes                  |

Table 39-12: Write-Protect Register and Secure Peripheral Number(n) (Continued)

|   No. | Peripheral/Block Name   | SPU WPn Mapping   | SECUREPn Mapping   | Controller Capable   |
|-------|-------------------------|-------------------|--------------------|----------------------|
|   173 | DAI0                    | WP173             | SECUREP173         | N/A                  |
|   174 | DAI1                    | WP174             | SECUREP174         | N/A                  |
|   175 | CRYPTO SPE              | WP175             | SECUREP175         | Yes                  |
|   176 | CRYPTO PKP              | WP176             | SECUREP176         | N/A                  |
|   177 | DLMDMA0                 | WP177             | SECUREP177         | Yes                  |
|   178 | DAPROM                  | WP178             | SECUREP178         | N/A                  |
|   179 | SHARC0 DBG              | WP179             | SECUREP179         | N/A                  |
|   180 | SHARC0 CTI              | WP180             | SECUREP180         | N/A                  |
|   181 | SHARC0 PTM              | WP181             | SECUREP181         | N/A                  |
|   182 | STM                     | WP182             | SECUREP182         | N/A                  |
|   183 | SHARC1 DBG              | WP183             | SECUREP183         | N/A                  |
|   184 | SHARC1 CTI              | WP184             | SECUREP184         | N/A                  |
|   185 | SHARC1 PTM              | WP185             | SECUREP185         | N/A                  |
|   186 | CSTF                    | WP186             | SECUREP186         | N/A                  |
|   187 | ETF                     | WP187             | SECUREP187         | N/A                  |
|   188 | ETR                     | WP188             | SECUREP188         | N/A                  |
|   189 | TPIU                    | WP189             | SECUREP189         | N/A                  |
|   190 | CTI Trace               | WP190             | SECUREP190         | N/A                  |
|   191 | CTI System              | WP191             | SECUREP191         | N/A                  |
|   192 | A5 IntegrationROM       | WP192             | SECUREP192         | N/A                  |
|   193 | A5 DBG                  | WP193             | SECUREP193         | N/A                  |
|   194 | A5 PMU                  | WP194             | SECUREP194         | N/A                  |
|   195 | A5 CTI                  | WP195             | SECUREP195         | N/A                  |
|   196 | A5 ETM                  | WP196             | SECUREP196         | N/A                  |
|   197 | TAPCMMR                 | WP197             | SECUREP197         | N/A                  |
|   198 | Debug Control           | WP198             | SECUREP198         | N/A                  |
|   199 | SWU C0 S1               | WP199             | SECUREP199         | N/A                  |
|   200 | SWU C0 S2               | WP200             | SECUREP200         | N/A                  |
|   201 | SWU C1 S1               | WP201             | SECUREP201         | N/A                  |
|   202 | SWU C1 S2               | WP202             | SECUREP202         | N/A                  |

## Processor-Specific Information

## Global Locking

The global lock signal from the SPU along with the peripheral lock bit can be used to provide lock functionality for the control MMR of the peripheral. The Global Lock ( SPU\_CTL.GLCK ) field determines whether global lock is enabled or not. Global Lock is disabled if the SPU\_CTL.GLCK field is 0xAD (default value), otherwise it is enabled. The following is a list of peripherals that have the global lock bit in their control MMR.

- General-Purpose IO (GPIO)
- System Event Controller (SEC0)
- Trigger Routing Unit (TRU0
- Clock Generation Unit (CGU0)
- Clock Generation Unit1 (CGU1)
- Clock Distribution Unit (CDU0)
- Dynamic Power Management (DPM)
- Reset Control Unit (RCU0)
- System Protection Unit (SPU0)
- L2 Memory Controller (L2CTL0)