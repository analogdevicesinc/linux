## 38   System Protection Unit (SPU)

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

## ADSP-2184x SPU Register List

The System Protection Unit (SPU) provides a set of registers that can protect system resources from errant writes. The protection categories are global lock (protects configuration registers) and write protect register lock (protects the write protect register). For more information on SPU functionality, see the SPU register descriptions.

Table 38-1: ADSP-2184x SPU Register List

| Name          | Description           |
|---------------|-----------------------|
| SPU_CTL       | Control Register      |
| SPU_SECURECHK | Secure Check Register |

Table 38-1: ADSP-2184x SPU Register List (Continued)

| Name           | Description                |
|----------------|----------------------------|
| SPU_SECURECTL  | Secure Control Register    |
| SPU_SECUREC[n] | Secure Core Registers      |
| SPU_SECUREP[n] | Secure Peripheral Register |
| SPU_STAT       | Status Register            |
| SPU_WP[n]      | Write Protect Register n   |

## ADSP-2184x SPU Interrupt List

Table 38-2: ADSP-2184x SPU Interrupt List

|   Interrupt ID | Name     | Description    | Sensitivity   | DMA Channel   |
|----------------|----------|----------------|---------------|---------------|
|            306 | SPU0_INT | SPU0 Interrupt | Level         |               |

## Peripheral Register Write Protection

The SPU has a write-protection register ( SPU\_WP[n] ) associated with each peripheral. Each of these write-protection registers has the exact same bits that correspond to a particular SMMR requester (for example, Core 0, MDMA). When the bits are set, the SPU locks the corresponding SMMR requesters from accessing the register address space of the associated peripheral. The bits in the register can be cleared to allow access to the registers of the peripheral again. When the SPU initializes the write-protection register, any writes that are in-progress complete before the SPU blocks subsequent writes.

In the SPU Write Protect Registers figure, each write-protect register in the SPU is associated with a particular peripheral.

Figure 38-1: SPU Write Protect Registers

![Image](41_System_Protection_Unit_(SPU)_artifacts/image_000000_5cc561b6e9eb3b89dd3b49a7b20c042d07a427c265b3a969b2835551a121bc7a.png)

Figure: SPU Write-protect registers Each write-protect register in the SPU is associated with a particular peripheral. In the figure, a write-protect register in the SPU module blocks write-attempts to the MMR space of the associated peripheral. The bits in the write-protect register specify from which requesters to block write-access.

NOTE: A SPU write protection register ( SPU\_WP[n] ) exists for the SPU alone. If all defined bits are set in this register for the SPU, any configurations in the SPU are locked and cannot be changed. Only a system reset can restore access to the SPU.

Figure: SPU Write-Protect Register Blocking Access from System Master 0 and Core Master 1 Figure 38-2: SPU Write-Protect Register Blocking Access from System Requester 0 and Core Requester 1

![Image](41_System_Protection_Unit_(SPU)_artifacts/image_000001_451da524a4d9375385fe56905e04389184f9a113ab016eb2aad835af2a21312f.png)

## Global Locking

A write-protect register in the SPU blocks write-attempts to the associated peripheral's MMR  space.  The bits in the write protect register specify which masters to block write-access from.

The SPU also has global locking capability. When enabled by setting SPU\_CTL.GLCK bit field to a value other than 0xAD, a system-wide global lock signal is active. Some peripherals have a lock enable bit in their control register. When this bit is set, the peripheral recognizes the global lock signal and blocks further write-accesses to its own control register. Access to the configuration register of the peripheral is enabled when the global lock is turned off in the SPU.

The Global Locking figure is a conceptual diagram. The diagram shows how the SPU module (or any peripheral) blocks any write attempts to its control register when:

- The global lock signal from the SPU is active, and
- The global lock enable bit is set in the control register of the peripheral

Figure 38-3: Global Locking

![Image](41_System_Protection_Unit_(SPU)_artifacts/image_000002_63b993a64f367dd3d9de5d88d1e5de1e12deea22a753586e265566f78da51cf6.png)

Figure: Conceptual Figure of Global Locking The SPU can write-protect its own registers. When the SPU\_CTL.WPLCK bit is set and global locking is enabled, the SPU blocks accesses to the SPU write-protection registers. To enable write access to the write-protection registers in the SPU, disable the global locking.

Peripheral blocks write attempts to Control MMR if the

Global Lock bit is set in the peripheral's Control MMR

and the Global Lock Signal is active from the SPU

-

## SPU Block Diagram

The SPU System-Level Block Diagram shows a system-level block diagram of where the SPU is located in the system. It resides between the SMMR interface and the system crossbar. Depending on the configuration of the SPU write-protect registers, it can block access to some peripherals from certain SMMR requesters.

Figure 38-4: SPU System-Level Block Diagram

![Image](41_System_Protection_Unit_(SPU)_artifacts/image_000003_b0c529c915c178a52cad6b1c74e2526b4e93ceacf9d5235ac7f3e098cd224cfa.png)

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

Another protection mechanism that the SPU offers is write-protection against the write-protection registers. If the write protect register lock bit ( SPU\_CTL.WPLCK ) is set and the global lock signal is active, writes to the write-protect registers of the SPU are blocked. To re-enable access to the write-protect registers in the SPU, deactivate the global lock signal by writing 0xAD into the SPU\_CTL.GLCK bit field.

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

## ADSP-2184x SPU Register Descriptions

System Protection Unit (SPU) contains the following registers.

Table 38-3: ADSP-2184x SPU Register List

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

Figure 38-5: SPU\_CTL Register Diagram

![Image](41_System_Protection_Unit_(SPU)_artifacts/image_000004_4f2775460ce341fc717350a53bc02b35ae8caf022672a813a6053bcb4c0d6494.png)

Table 38-4: SPU\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | WPLCK      | Write Protect Register Lock. When the SPU_CTL.WPLCK bit is set in combination with the SPU_CTL.GLCK bit, writes to the SPU's write protect registers are blocked and return an error. |
| 14 (R/W)           | PINTEN     | Protection Violation Interrupt Enable. When the SPU_CTL.PINTEN bit is set (=1), a block of any transaction according to the configured settings produces an interrupt.                |
| 13                 | PBETYPE    | 0 Disable 1 Enable Protection Violation Bus Error Type.                                                                                                                               |
| (R/W)              |            | 0 Reserved                                                                                                                                                                            |
| 12 (R/W)           | PBEDIS     | Protection Violation Bus Error Disable. 0 Reserved                                                                                                                                    |

Table 38-4: SPU\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | GLCK       | Global Lock. The SPU_CTL.GLCK controls the global lock signal. The global lock signal provides register-based write protection. Writing 0xAD to this field disables the lock, and writing any other value enables the lock. |

## Secure Check Register

The SPU\_SECURECHK register reads by secure requesters return 0xFFFFFFFF . Reads by non-secure requesters return 0x00000000.

Figure 38-6: SPU\_SECURECHK Register Diagram

![Image](41_System_Protection_Unit_(SPU)_artifacts/image_000005_e1152ffa92e6965bee20390b34d2af8c86a98d511f6417c50cf6e067f2d3103c.png)

Table 38-5: SPU\_SECURECHK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------|
| 31:0               | DATA       | Read Data. The SPU_SECURECHK.DATA bit field performs reads. Reads by secure requesters |
| (R/NW)             |            | return 0xFFFFFFFF. Reads by non-secure requesters return 0x00000000.                   |

## Secure Control Register

The SPU Secure Control Register ( SPU\_SECURECTL ) allows the user to lock write access to all the SPU\_SECUREC[n] and SPU\_SECUREP[n] registers as well as configure the interrupt generation in an event of a security error. It also allows bulk clear of the SSEC bits and/or MSEC bits in the SPU\_SECUREP[n] registers.

Figure 38-7: SPU\_SECURECTL Register Diagram

![Image](41_System_Protection_Unit_(SPU)_artifacts/image_000006_e06c4263d4bb8e38c9704aef40bf13b90fa5dbb61333522bce0b3701a69a1545.png)

Table 38-6: SPU\_SECURECTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | SCRLCK     | Secure Register Lock. When the SPU_SECURECTL.SCRLCK bit is set in combination with the SPU_CTL.GLCK bit, writes to the Security Configuration registers ( SPU_SECUREC[n] and SPU_SECUREP[n] ) are blocked and return an error which is captured in the SPU_STAT.LWERR bit. |
| 14 (R/W)           | SINTEN     | Secure Violation Interrupt Enable. The SPU_SECURECTL.SINTEN bit generates an interrupt if a security violation was captured. Interrupt status is provided in the SPU_STAT.VIRQ bit.                                                                                        |
| 5 (R0/W)           | MSECCLR    | Requester Secure Clear. When the SPU_SECURECTL.MSECCLR bit is set, the SPU_SECUREP[n].MSEC bits in all SPU_SECUREP[n] registers are cleared. The SPU_SECURECTL.MSECCLR bit always reads back as a 0. 0 No Action 1 Clear All Requester Secure Control Bits                 |

Table 38-6: SPU\_SECURECTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R0/W)           | SSECCLR    | Completer Secure Clear. When the SPU_SECURECTL.SSECCLR bit is set, the SPU_SECUREP[n].SSEC bits in all SPU_SECUREP[n] registers are cleared. The SPU_SECURECTL.SSECCLR bit always reads back as a 0. |
| 4 (R0/W)           | SSECCLR    | 0 No Action                                                                                                                                                                                          |
| 4 (R0/W)           | SSECCLR    | 1 Clear All Completer Secure Control Bits                                                                                                                                                            |

## Secure Core Registers

A SPU register exists for every DSP core in the system. The SEC field enables or disables security for features in the core.

Figure 38-8: SPU\_SECUREC[n] Register Diagram

![Image](41_System_Protection_Unit_(SPU)_artifacts/image_000007_4696e3d11e2e4c4133f99cfdbf17fbce01b01d0356ed439f37128bc7ac81adbb.png)

Table 38-7: SPU\_SECUREC[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | CSEC       | Core Secure. The SPU_SECUREC[n].CSEC bit controls whether non-secure accesses are allowed to L1 memory of the processor core. When =1, the core (as a completer) is set as secure meaning only secure transactions are allowed to L1. 0 Disable |
| 0 (R/W)            | CSEC       | 1 Enable                                                                                                                                                                                                                                        |
| 0 (R/W)            | CSEC       |                                                                                                                                                                                                                                                 |

## Secure Peripheral Register

In the system, each SPU\_SECUREP[n] register is assigned to a specific MMR address range associated with one peripheral. Each SPU\_SECUREP[n] has a Completer Secure (SSEC) bit and a Requester Secure (MSEC) bit. When the SSEC bit is set, the SPU will only allow secure requesters generating secure transactions to access the peripheral's MMR address space. When the MSEC bit is set, the associated peripheral will be secure and will generate secure transactions.

Figure 38-9: SPU\_SECUREP[n] Register Diagram

![Image](41_System_Protection_Unit_(SPU)_artifacts/image_000008_1ccb648d546564c5e0874a67c9ba5b4c73cabba56e24b0a812a2fac41d3dd5b6.png)

Table 38-8: SPU\_SECUREP[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | MSEC       | Requester Secure Enable. The SPU_SECUREP[n].MSEC bit controls whether the peripheral generates secure transactions as a requester. When clear (=0), the peripheral generates non-secure trans- actions as a requester (if applicable). When set (=1), the peripheral generates secure transactions as a requester. 0 Disable                                      |
| 0 (R/W)            | SSEC       | Completer Secure Enable. The SPU_SECUREP[n].SSEC bit controls whether the peripheral is protected from non-secure transactions. When clear (=0), the security status of the transaction is ignored. When set (=1), only secure transactions are allowed to access the address space of the peripheral and non-secure transactions are blocked. 0 Disable 1 Enable |

## Status Register

The SPU\_STAT register indicates if there have been any errors, active interrupts and global lock status.

![Image](41_System_Protection_Unit_(SPU)_artifacts/image_000009_8543123da2a2b3dbce75750c5a01edeb932632d59d976278681fd17b41b09d6b.png)

Lock Write Error

Figure 38-10: SPU\_STAT Register Diagram

Table 38-9: SPU\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | Lock Write Error. The SPU_STAT.LWERR indicates whether there has been an attempted write to a register in the SPU with its lock bit ( SPU_CTL.WPLCK or SCRLCK) set while SPU_CTL.GLCK was asserted. This bit is W1C. |
| 30 (R/W1C)         | ADDRERR    | Address Error. The SPU_STAT.ADDRERR indicates whether there has been an attempted write to a read-only register or an access an invalid address in the SPU MMRaddress range. This bit is W1C.                        |
| 12 (R/W1C)         | VIRQ       | Violation Interrupt Request. The SPU_STAT.VIRQ bit indicates that a security and/or protection violation has been detected and interrupt asserted. This is a W1C bit.                                                |
| 0 (R/NW)           |            | Global Lock Status. The SPU_STAT.GLCK indicates whether the global lock is enabled or disabled.                                                                                                                      |
|                    | GLCK       | 0 Disabled (global_lock=0)                                                                                                                                                                                           |

Address Error

## Write Protect Register n

In the system, each SPU\_WP[n] register is assigned to a specific MMR address range associated with one peripheral. When the appropriate bits are set, writes to the peripheral from a specific requester are blocked and an error is returned to the requester. For more information, see the processor specific additional information for the SPU\_WP[n] register.

Figure 38-11: SPU\_WP[n] Register Diagram

![Image](41_System_Protection_Unit_(SPU)_artifacts/image_000010_2a4c598f96b98a3eaddb67ea7ddba1ac0c8eb1bbeb864cda9ec5c8d43e0cdb3c.png)

Table 38-10: SPU\_WP[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19:16 (R/W)        | SM[n]      | System Requester x Write Protect Enable. The SPU_WP[n].SM[n] bits correspond to different system requesters in the sys- tem. When a particular bit is set in this field, the corresponding system requester cannot write to the corresponding peripheral's MMRaddress space. The write attempt is blocked by the SPU. |
| 2:0 (R/W)          | CM[n]      | Core Requester x Write Protect Enable. The SPU_WP[n].CM[n] bits correspond to different cores in the system. When a particular bit is set in this field, the corresponding core cannot write to the correspond- ing peripheral's MMRaddress space. The write attempt is blocked by the SPU.                           |

## Write-Protect, Secure Peripheral, and Secure Core Registers

The SPU consists of a collection of write-protect registers each of which are associated with a specific peripheral or target. The SPU also has a collection of secure peripheral registers which are also associated with specific peripherals. The table gives the write-protect register and secure peripheral number for each of the peripherals that are provided with write protection and security through the SPU. The SPU for the processor is configured with 136 write-protect registers and also 136 secure peripheral registers. The number corresponding to a peripheral correlates to both the Write-Protect register and the Secure Peripheral register.

For each processor, there are different numbers of controllers that are able to access the SMMR space

The SPU\_WP[n] register shows which bits enable the protection against which controller.

For each peripheral, there is a corresponding write-protect register, SPU\_WP[n] , and secure peripheral register, SPU\_SECUREP[n] . The table shows the write-protect register and secure peripheral number for each peripheral.

|   No. | Peripheral/Block Name   | SWPn Mapping   | SECUREPn Mapping   | Controller Capable   |
|-------|-------------------------|----------------|--------------------|----------------------|
|     0 | Reserved                | Reserved       | Reserved           | Reserved             |
|     1 | DMC0_PRG_CDC_BRI DGE    | WP1            | SECURE1            | N/A                  |
|     2 | SPIF GPV SPACE          | WP2            | SECURE2            | N/A                  |
|     3 | LP0                     | WP3            | SECURE3            | N/A                  |
|     4 | LP1                     | WP4            | SECURE4            | N/A                  |
|     5 | LP0 DDE                 | WP5            | SECURE5            | Yes                  |
|     6 | LP1 DDE                 | WP6            | SECURE6            | Yes                  |
|     7 | TWI3                    | WP7            | SECURE7            | N/A                  |
|     8 | TWI4                    | WP8            | SECURE8            | N/A                  |
|     9 | TWI5                    | WP9            | SECURE9            | N/A                  |
|    10 | TWI0                    | WP10           | SECURE10           | N/A                  |
|    11 | TWI1                    | WP11           | SECURE11           | N/A                  |
|    12 | TWI2                    | WP12           | SECURE12           | N/A                  |
|    13 | SPORT0A                 | WP13           | SECURE13           | N/A                  |
|    14 | SPORT0B                 | WP14           | SECURE14           | N/A                  |
|    15 | SPORT1A                 | WP15           | SECURE15           | N/A                  |
|    16 | SPORT1B                 | WP16           | SECURE16           | N/A                  |
|    17 | SPORT2A                 | WP17           | SECURE17           | N/A                  |
|    18 | SPORT2B                 | WP18           | SECURE18           | N/A                  |
|    19 | SPORT3A                 | WP19           | SECURE19           | N/A                  |
|    20 | SPORT3B                 | WP20           | SECURE20           | N/A                  |
|    21 | SPORT4A                 | WP21           | SECURE21           | N/A                  |
|    22 | SPORT4B                 | WP22           | SECURE22           | N/A                  |
|    23 | SPORT5A                 | WP23           | SECURE23           | N/A                  |
|    24 | SPORT5B                 | WP24           | SECURE24           | N/A                  |
|    25 | SPORT6A                 | WP25           | SECURE25           | N/A                  |
|    26 | SPORT6B                 | WP26           | SECURE26           | N/A                  |
|    27 | SPORT7A                 | WP27           | SECURE27           | N/A                  |
|    28 | SPORT7B                 | WP28           | SECURE28           | N/A                  |
|    29 | UART0                   | WP29           | SECURE29           | N/A                  |

|   No. | Peripheral/Block Name   | SWPn Mapping   | SECUREPn Mapping   | Controller Capable   |
|-------|-------------------------|----------------|--------------------|----------------------|
|    30 | UART1                   | WP30           | SECURE30           | N/A                  |
|    31 | UART2                   | WP31           | SECURE31           | N/A                  |
|    32 | PORTA                   | WP32           | SECURE32           | N/A                  |
|    33 | PORTB                   | WP33           | SECURE33           | N/A                  |
|    34 | PORTC                   | WP34           | SECURE34           | N/A                  |
|    35 | PORTD                   | WP35           | SECURE35           | N/A                  |
|    36 | PORTE                   | WP36           | SECURE36           | N/A                  |
|    37 | PORTF                   | WP37           | SECURE37           | N/A                  |
|    38 | PORTG                   | WP38           | SECURE38           | N/A                  |
|    39 | PORTH                   | WP39           | SECURE39           | N/A                  |
|    41 | PADS                    | WP41           | SECURE41           | N/A                  |
|    42 | PINT1                   | WP42           | SECURE42           | N/A                  |
|    43 | PINT1                   | WP43           | SECURE43           | N/A                  |
|    44 | PINT2                   | WP44           | SECURE44           | N/A                  |
|    45 | PINT3                   | WP45           | SECURE45           | N/A                  |
|    46 | PINT4                   | WP46           | SECURE46           | N/A                  |
|    47 | PINT5                   | WP47           | SECURE47           | N/A                  |
|    48 | PINT6                   | WP48           | SECURE48           | N/A                  |
|    49 | PINT7                   | WP49           | SECURE49           | N/A                  |
|    50 | SMC0                    | WP50           | SECURE50           | N/A                  |
|    51 | SMPU_SMC                | WP51           | SECURE51           | N/A                  |
|    52 | WDT0                    | WP52           | SECURE52           | N/A                  |
|    53 | WDT1                    | WP53           | SECURE53           | N/A                  |
|    54 | WDT2                    | WP54           | SECURE54           | N/A                  |
|    55 | WDT3                    | WP55           | SECURE55           | N/A                  |
|    56 | CNT0                    | WP56           | SECURE56           | N/A                  |
|    57 | OTPMMR                  | WP57           | SECURE57           | N/A                  |
|    58 | SMPU-OTP                | WP58           | SECURE58           | N/A                  |
|    59 | SWU SMC                 | WP59           | SECURE59           | N/A                  |
|    60 | HADC0                   | WP60           | SECURE60           | N/A                  |
|    61 | TMU0                    | WP61           | SECURE61           | N/A                  |
|    62 | TMR                     | WP62           | SECURE62           | N/A                  |

|   No. | Peripheral/Block Name   | SWPn Mapping   | SECUREPn Mapping   | Controller Capable   |
|-------|-------------------------|----------------|--------------------|----------------------|
|    63 | xSPI                    | WP63           | SECURE63           | Yes                  |
|    64 | SPORT0A DDE             | WP64           | SECURE64           | Yes                  |
|    65 | SPORT0B DDE             | WP65           | SECURE65           | Yes                  |
|    66 | SPORT1A DDE             | WP66           | SECURE66           | Yes                  |
|    67 | SPORT1B DDE             | WP67           | SECURE67           | Yes                  |
|    68 | SPORT2A DDE             | WP68           | SECURE68           | Yes                  |
|    69 | SPORT2B DDE             | WP69           | SECURE69           | Yes                  |
|    70 | SPORT3A DDE             | WP70           | SECURE70           | Yes                  |
|    71 | SPORT3B DDE             | WP71           | SECURE71           | Yes                  |
|    72 | SPORT4A DDE             | WP72           | SECURE72           | Yes                  |
|    73 | SPORT4B DDE             | WP73           | SECURE73           | Yes                  |
|    74 | SPORT5A DDE             | WP74           | SECURE74           | Yes                  |
|    75 | SPORT5B DDE             | WP75           | SECURE75           | Yes                  |
|    76 | SPORT6A DDE             | WP76           | SECURE76           | Yes                  |
|    77 | SPORT6B DDE             | WP77           | SECURE77           | Yes                  |
|    78 | SPORT7A DDE             | WP78           | SECURE78           | Yes                  |
|    79 | SPORT7B DDE             | WP79           | SECURE79           | Yes                  |
|    80 | UART0RX DDE             | WP80           | SECURE80           | Yes                  |
|    81 | UART0TX DDE             | WP81           | SECURE81           | Yes                  |
|    82 | UART1RX DDE             | WP82           | SECURE82           | Yes                  |
|    83 | UART1TX DDE             | WP83           | SECURE83           | Yes                  |
|    84 | UART2RX DDE             | WP84           | SECURE84           | Yes                  |
|    85 | UART2TX DDE             | WP85           | SECURE85           | Yes                  |
|    86 | Core MBIST              | WP86           | SECURE86           | Yes                  |
|    87 | MEMREPAIR               | WP87           | SECURE87           | Yes                  |
|    88 | SPI0TX DDE              | WP88           | SECURE88           | Yes                  |
|    89 | SPI0RX DDE              | WP89           | SECURE89           | Yes                  |
|    90 | SPI1TX DDE              | WP90           | SECURE90           | Yes                  |
|    91 | SPI1RX DDE              | WP91           | SECURE91           | Yes                  |
|    92 | SPI2TX DDE              | WP92           | SECURE92           | Yes                  |
|    93 | SPI2RX DDE              | WP93           | SECURE93           | Yes                  |
|    94 | SPI5TX DDE              | WP94           | SECURE94           | Yes                  |

|   No. | Peripheral/Block Name   | SWPn Mapping   | SECUREPn Mapping   | Controller Capable   |
|-------|-------------------------|----------------|--------------------|----------------------|
|    95 | SPI5RX DDE              | WP95           | SECURE95           | Yes                  |
|    96 | SPI0                    | WP96           | SECURE96           | N/A                  |
|    97 | SPI1                    | WP97           | SECURE97           | N/A                  |
|    98 | SPI2                    | WP98           | SECURE98           | N/A                  |
|    99 | SPI5                    | WP99           | SECURE99           | N/A                  |
|   100 | EMAC0                   | WP100          | SECURE100          | Yes                  |
|   101 | CAN0                    | WP101          | SECURE102          | N/A                  |
|   102 | CAN1                    | WP102          | SECURE102          | N/A                  |
|   103 | DDR PFB                 | WP103          | SECURE103          | N/A                  |
|   104 | L2CTL0                  | WP104          | SECURE104          | N/A                  |
|   105 | L2CTL1                  | WP105          | SECURE105          | N/A                  |
|   107 | SMPU L2CTL0 CL2 0       | WP107          | SECURE107          | N/A                  |
|   108 | SMPU L2CTL0 DL2 0       | WP108          | SECURE108          | N/A                  |
|   109 | SMPU L2CTL0 CL2 1       | WP109          | SECURE109          | N/A                  |
|   110 | SMPU L2CTL0 DL2 1       | WP110          | SECURE110          | N/A                  |
|   111 | SMPU L2CTL0 CL2 2       | WP111          | SECURE111          | N/A                  |
|   112 | SEC0                    | WP112          | SECURE112          | N/A                  |
|   113 | TRU0                    | WP113          | SECURE113          | N/A                  |
|   114 | SPU0                    | WP114          | SECURE114          | N/A                  |
|   115 | RCU0                    | WP115          | SECURE115          | N/A                  |
|   116 | CGU0                    | WP116          | SECURE116          | N/A                  |
|   117 | CGU1                    | WP117          | SECURE117          | N/A                  |
|   118 | CDU0                    | WP118          | SECURE118          | N/A                  |
|   119 | DPM0                    | WP119          | SECURE119          | N/A                  |
|   120 | PLL0                    | WP120          | SECURE120          | N/A                  |
|   121 | PLL1                    | WP121          | SECURE121          | N/A                  |
|   122 | SWU_L2CTL0_CL2_0        | WP122          | SECURE122          | N/A                  |
|   123 | SWU_L2CTL0_DL2_0        | WP123          | SECURE123          | N/A                  |
|   124 | SWU_L2CTL0_CL2_1        | WP124          | SECURE124          | N/A                  |
|   125 | SWU_L2CTL0_DL2_1        | WP125          | SECURE125          | N/A                  |
|   126 | SWU SMMR                | WP126          | SECURE126          | N/A                  |
|   127 | SWU_L2CTL0_CL2_2        | WP127          | SECURE127          | N/A                  |

|   No. | Peripheral/Block Name   | SWPn Mapping   | SECUREPn Mapping   | Controller Capable   |
|-------|-------------------------|----------------|--------------------|----------------------|
|   128 | EnhancedBWMDMA          | WP128          | SECURE128          | Yes                  |
|   129 | MaxBWMDMA               | WP129          | SECURE129          | Yes                  |
|   130 | MaxBWMDMA1              | WP120          | SECURE130          | Yes                  |
|   131 | MLB0                    | WP131          | SECURE131          | Yes                  |
|   132 | SWU DMC0                | WP132          | SECURE132          | N/A                  |
|   133 | SMPU DMC0               | WP133          | SECURE133          | N/A                  |
|   134 | SMPU SPI2               | WP134          | SECURE134          | N/A                  |
|   135 | MEC0                    | WP135          | SECURE135          | N/A                  |
|   136 | MEC1                    | WP136          | SECURE136          | N/A                  |
|   137 | MEC2                    | WP137          | SECURE137          | N/A                  |
|   138 | CRC0                    | WP138          | SECURE138          | N/A                  |
|   139 | CRC1                    | WP139          | SECURE139          | N/A                  |
|   140 | MDMA0 DDE0 CRC0         | WP140          | SECURE140          | Yes                  |
|   141 | MDMA0 DDE1 CRC0         | WP141          | SECURE141          | Yes                  |
|   142 | MDMA1 DDE0 CRC1         | WP142          | SECURE142          | Yes                  |
|   143 | MDMA1 DDE1 CRC1         | WP143          | SECURE143          | Yes                  |
|   144 | MDMA4 DDE0 CRC2         | WP144          | SECURE144          | Yes                  |
|   145 | MDMA4 DDE1 CRC2         | WP145          | SECURE145          | Yes                  |
|   146 | MDMA5 DDE0 CRC3         | WP146          | SECURE146          | Yes                  |
|   147 | MDMA5 DDE1 CRC3         | WP147          | SECURE147          | Yes                  |
|   148 | SWU SPI2                | WP148          | SECURE148          | N/A                  |
|   149 | MISC REG                | WP149          | SECURE149          | N/A                  |
|   150 | CRC2                    | WP150          | SECURE150          | N/A                  |
|   151 | CRC3                    | WP151          | SECURE151          | N/A                  |
|   152 | EnhancedBWMDMA1         | WP152          | SECURE152          | Yes                  |
|   153 | DAI0                    | WP153          | SECURE153          | N/A                  |
|   154 | DAI1                    | WP154          | SECURE154          | N/A                  |
|   155 | CRYPTO SPE              | WP155          | SECURE155          | Yes                  |
|   156 | CRYPTO PKP              | WP156          | SECURE156          | N/A                  |
|   157 | DLMDMA0                 | WP157          | SECURE157          | Yes                  |
|   158 | DAPROM                  | WP158          | SECURE158          | N/A                  |
|   159 | SHARC-FX0 CTI           | WP159          | SECURE159          | N/A                  |

| No.          | Peripheral/Block Name   | SWPn Mapping   | SECUREPn Mapping   | Controller Capable   |
|--------------|-------------------------|----------------|--------------------|----------------------|
| 160          | SHARC-FX0 TRAX          | WP160          | SECURE160          | N/A                  |
| 161          | SHARC-FX0 PMU           | WP161          | SECURE161          | N/A                  |
| 162          | SHARC-FX0 OCD           | WP162          | SECURE162          | N/A                  |
| 163          | SHARC-FX0 Misc          | WP163          | SECURE163          | N/A                  |
| 164          | CSTF                    | WP164          | SECURE164          | N/A                  |
| 165          | ETF                     | WP165          | SECURE165          | N/A                  |
| 166          | ETR                     | WP166          | SECURE166          | N/A                  |
| 167          | TPIU                    | WP167          | SECURE167          | N/A                  |
| 168          | CTI Trace               | WP168          | SECURE168          | N/A                  |
| 169          | CTI System              | WP169          | SECURE169          | N/A                  |
| 170          | TSGEN                   | WP170          | SECURE170          | N/A                  |
| 171          | A55ROM                  | WP173          | SECURE173          | N/A                  |
| 172          | A55 PE0 Debug           | WP171          | SECURE171          | N/A                  |
| 173          | A55 PE1 Debug           | WP172          | SECURE172          | N/A                  |
| 174          | A55 PE0 PMU             | WP174          | SECURE174          | N/A                  |
| 175          | A55 PE1 PMU             | WP175          | SECURE175          | N/A                  |
| 176          | A55 PE0 CTI             | WP176          | SECURE176          | N/A                  |
| 177          | A55 PE1 CTI             | WP177          | SECURE177          | N/A                  |
| 178          | A55 PE0 ETM             | WP178          | SECURE178          | N/A                  |
| 179          | A55 PE1 ETM             | WP179          | SECURE179          | N/A                  |
| 180          | M85 MISC                | WP180          | SECURE180          | N/A                  |
| 181          | M85 PFB                 | WP181          | SECURE181          | N/A                  |
| 182          | M85 SMPU                | WP182          | SECURE182          | N/A                  |
| 183          | XSPI2_PFB               | WP183          | SECURE183          | N/A                  |
| 184          | XSPI2                   | WP184          | SECURE184          | MSEC184              |
| 185 Reserved | 185 Reserved            | 185 Reserved   | 185 Reserved       | 185 Reserved         |
| 186          | EMMC                    | WP186          | SECURE186          | MSEC186              |
| 187          | HSM                     | WP187          | SECURE187          | N/A                  |
| 188          | M85_2 MISC              | WP188          | SECURE188          | N/A                  |
| 189          | M85_2 PFB               | WP189          | SECURE189          | N/A                  |
| 190          | M85_2 SMPU              | WP190          | SECURE190          | N/A                  |
| 191          | FRAC-PLL2               | WP191          | SECURE191          | N/A                  |

|   No. | Peripheral/Block Name   | SWPn Mapping   | SECUREPn Mapping   | Controller Capable   |
|-------|-------------------------|----------------|--------------------|----------------------|
|   192 | LPDDR4 Ctrl             | WP192          | SECURE192          | N/A                  |
|   193 | LPDDR4 Perf             | WP193          | SECURE193          | N/A                  |
|   194 | HSM_MISC                | WP194          | SECURE194          | N/A                  |
|   195 | LPDDR4 PHY              | WP195          | SECURE195          | N/A                  |

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