## 43   System Protection Unit (SPU)

In a system with multiple system MMR masters, configurations of peripherals can be changed unintentionally leading to bad data or even system malfunctions. The peripherals are shared resources in the system. The SPU restricts access to certain MMRs, similar to the functionality of a semaphore.

The SPU also protects peripherals based on security settings. It is part of the overall security infrastructure of the processor.

## SPU Features

The SPU has the following features:

- Write-protect system MMR from certain system masters and core masters.
- Simultaneously lock multiple peripheral configuration registers through a global lock mechanism.
- Write-protect and block access to its own write-protection registers from other system masters.
- Defined security privileges to peripherals and system resources.
- Security protection to guard secure peripheral MMRs against non-secure accesses.

## SPU Functional Description

The following sections provide information on the function of the SPU.

## ADSP-SC58x SPU Register List

The System Protection Unit (SPU) provides a set of registers that can protect system resources from errant writes. The protection categories are global lock (protects configuration registers) and write protect register lock (protects the write protect register). For more information on SPU functionality, see the SPU register descriptions.

Table 43-1: ADSP-SC58x SPU Register List

| Name          | Description           |
|---------------|-----------------------|
| SPU_CTL       | Control Register      |
| SPU_SECURECHK | Secure Check Register |

Table 43-1: ADSP-SC58x SPU Register List (Continued)

| Name           | Description                |
|----------------|----------------------------|
| SPU_SECURECTL  | Secure Control Register    |
| SPU_SECUREC[n] | Secure Core Registers      |
| SPU_SECUREP[n] | Secure Peripheral Register |
| SPU_STAT       | Status Register            |
| SPU_WP[n]      | Write Protect Register n   |

## ADSP-SC58x SPU Interrupt List

Table 43-2: ADSP-SC58x SPU Interrupt List

|   Interrupt ID | Name     | Description    | Sensitivity   | DMA Channel   |
|----------------|----------|----------------|---------------|---------------|
|            241 | SPU0_INT | SPU0 Interrupt | Level         |               |

## Peripheral Register Write Protection

The SPU has a write-protection register ( SPU\_WP[n] ) associated with each peripheral. Each of these write-protection registers has the exact same bits that correspond to a particular SMMR master (Core 0, Core 1, MDMA, for example). When the bits are set, the SPU locks the corresponding SMMR masters from accessing the register address space of the associated peripheral. The bits in the register can be cleared to allow access to the registers of the peripheral again. When the SPU intiates the write-protection register, any writes that are in-progress complete before the SPU blocks subsequent writes.

In the SPU Write Protect Registers figure, each write-protect register in the SPU is associated with a particular peripheral.

Figure 43-1: SPU Write Protect Registers

![Image](46_System_Protection_Unit_(SPU)_artifacts/image_000000_3c30647a07dcc70004b069a1ecbfae14c9de0577fd929f31e850eab232cc7c2f.png)

Figure: SPU Write-protect registers Each write-protect register in the SPU is associated with a particular peripheral. In the figure, a write-protect register in the SPU module blocks write-attempts to the MMR space of the associated peripheral. The bits in the write-protect register specify from which masters to block write-access.

NOTE: A SPU write protection register ( SPU\_WP[n] ) exists for the SPU alone. If all defined bits are set in this register for the SPU, any configurations in the SPU are locked and cannot be changed. Only a system reset can restore access to the SPU.

Figure: SPU Write-Protect Register Blocking Access from System Master 0 and Core Master 1 Figure 43-2: SPU Write-Protect Register Blocking Access from System Master 0 and Core Master 1

![Image](46_System_Protection_Unit_(SPU)_artifacts/image_000001_8435b03947224e95c3a02003cdb813235a28e8722f2d127dee86f2ebadb4d42d.png)

## Global Locking

A write-protect register in the SPU blocks write-attempts to the associated peripheral's MMR  space.  The bits in the write protect register specify which masters to block write-access from.

The SPU also has global locking capability. When enabled by setting SPU\_CTL.GLCK bit field to a value other than 0xAD, a system-wide global lock signal is active. Some peripherals have a lock enable bit in their control register. When this bit is set, the peripheral recognizes the global lock signal and blocks further write-accesses to its own control register. Access to the configuration register of the peripheral is enabled when the global lock is turned off in the SPU.

The Global Locking figure is a conceptual diagram. The diagram shows how the SPU module (or any peripheral) blocks any write attempts to its control register when:

- The global lock signal from the SPU is active, and
- The global lock enable bit is set in the control register of the peripheral

Figure 43-3: Global Locking

![Image](46_System_Protection_Unit_(SPU)_artifacts/image_000002_be71a51022e08e24cb6a25f87c1457e1a06879226aa3474acb81600ec06d41e5.png)

Figure: Conceptual Figure of Global Locking The SPU can write-protect its own registers. When the SPU\_CTL.WPLCK bit is set and global locking is enabled, the SPU blocks accesses to the SPU write-protection registers. To enable write access to the write-protection registers in the SPU, disable the global locking.

Peripheral blocks write attempts to Control MMR if the

Global Lock bit is set in the peripheral's Control MMR

and the Global Lock Signal is active from the SPU

-

## SPU Block Diagram

The SPU System-Level Block Diagram shows a system-level block diagram of where the SPU is located in the system. It resides between the SMMR interface and the system crossbar. Depending on the configuration of the SPU write-protect registers, it can block access to some peripherals from certain SMMR masters.

Figure 43-4: SPU System-Level Block Diagram

![Image](46_System_Protection_Unit_(SPU)_artifacts/image_000003_b0c529c915c178a52cad6b1c74e2526b4e93ceacf9d5235ac7f3e098cd224cfa.png)

## SPU Architectural Concepts

As shown in the block diagram, the SPU sits between the system crossbar (SCB) and the SMMR interface to the peripherals. The SPU gates any MMR access to any peripheral from any master that comes through the SCB. Depending on the configuration of the write-protection registers in the SPU, the SPU does or does not allow the MMR write to go through.

The SPU also checks whether the transaction is a secure or non-secure transaction and blocks it according to the configured security setting for the target destination. A secure master can generate secure read or secure write transactions which can access secure or non-secure slaves. A non-secure master can generate non-secure read or non-secure write transactions and can only access non-secure slaves.

## SPU Event Control

The system protection unit provides write-protection against MMRs peripherals and its own write-protect registers. If a write attempt is made to any locked MMR peripheral the SPU has write-protected, it blocks the write. The SPU generates a bus error to the master that attempted the write. That master does or does not generate an event, based on the returned error.

The SPU can be configured to generate an interrupt for the write-protection violation by setting the SPU\_CTL.PINTEN bit. The SPU can also be configured to generate an interrupt for a security violation by setting the SPU\_SECURECTL.SINTEN bit. If either one or both bits is triggered, the SPU\_STAT.VIRQ bit is set.

The SPU can also lock its own registers from write attempts. If a write-attempt is made to a locked register in the SPU, the SPU blocks it and records it as an error in the SPU\_STAT.LWERR bit. Again, the SPU generates a bus error to the master that attempted the write.

The master does or does not generate an event, based on the returned error.

The SPU does not generate an event for a blocked write access to an SPU register. If the SPU\_CTL.PINTEN bit is set, the SPU triggers an interrupt for this blocked access attempt.

The global lock is enabled by setting the SPU\_CTL.GLCK bit to something other than 0xAD. If the lock bit is set in that same configuration register, a peripheral can block write access to its configuration register. When the SPU blocks a write attempt, the peripheral logs and reports the failed attempt. The SPU is unaware and therefore does not provide any indication of a failed write attempt to the configuration register of the peripheral.

## SPU Programming Model

The system protection unit (SPU) consists of write-protect registers. Each one corresponds to a different peripheral instance. Bits in the write-protect registers correspond to system masters that can modify the MMR contents of the peripherals. By writing to these write-protect registers, the corresponding memory-mapped registers of the peripheral are write-protected against masters whose bits in the write-protect register are set.

The SPU globally locks the control register of the peripheral. Peripherals that support this feature have a lock enable bit in their control register. The peripheral blocks any additional write attempts to its control register from any master when:

- The global lock signal is active from the SPU, and
- The lock enable bit of the peripheral is set

If the lock enable bit of a peripheral is not set and the global lock signal is active, access to that control register of the peripheral is still allowed. T o grant access again, disable the global lock signal from the SPU by writing the value 0xAD into the SPU\_CTL.GLCK bit field.

Another protection mechanism that the SPU offers is write-protection against the write-protection registers. If the write protect register lock bit ( SPU\_CTL.WPLCK ) is set and the global lock signal is active, writes to the writeprotect registers of the SPU are blocked. To reenable access to the write-protect registers in the SPU, deactivate the global lock signal by writing 0xAD into the SPU\_CTL.GLCK bit field.

For security, the SPU provides a set of SPU\_SECUREC[n] registers (one for each processor core from Analog Devices) to configure their security settings. The SPU also provides a set of SPU\_SECUREP[n] registers (one for each peripheral instance) to configure their security settings.

## Enabling and Disabling the SPU

The SPU is always operating. There are no bits to enable or disable the SPU. The SPU configuration can be updated at any time. Any ongoing transactions finish before a new configuration is in effect. By default, the SPU does not write-protect any of the MMRs.

## Write-Protecting the SPU

The SPU is treated like any other peripheral in the system. As such, the SPU also has an associated write-protection register. If this write-protection register is configured to block all writes from all masters, any SPU configuration remains the same until the next system reset.

## Checking the Security State

In some cases while running a peripheral, an application system master does not know whether they are a secure master generating secure transactions or not. The SPU provides a means for checking the security state of the master through the SPU\_SECURECHK register. When read by a secure master, the register reads 0xFFFFFFFF and when read by a non-secure master, the value is 0x00000000.

## SPU Mode Configuration

The SPU can provide address range-wide protection by write-protecting the peripherals MMR address range from system MMR masters. It can also provide register wide protection using global locking. Peripherals that support this feature can enable it in their respective configuration register. When the SPU enables the global lock signal, all subsequent writes to the configuration register of the peripheral are blocked until the global lock signal is deasserted. Similarly, the write-protection registers of the SPU can be write-protected using the global lock signal as well. The SPU uses all these modes of operation together.

## Locking Write-Protect Registers

Use the following steps to lock (write-protect) a register.

1. Set the SPU\_CTL.WPLCK bit and configure the SPU\_CTL.GLCK field to something other than 0xAD.

The SPU write-protect registers are blocked from further write accesses.

## Protecting a Peripheral

Use the following procedure to protect a peripheral.

1. Determine which peripheral needs protection and locate the corresponding write-protect register ( SPU\_WP[n] ) in the SPU. See the "Write-Protect and Secure Peripheral Registers" section.
2. Determine the SMMR masters from which the peripheral needs protection. Then, set the corresponding bit or bits in the write-protect register ( SPU\_WP[n] ) for the peripheral. See the "Write-Protect and Secure Peripheral Registers" section.

After setting the write-protect register for the particular peripheral, the identified SMMR masters are blocked from writing to any MMR in the address space of the peripheral. This block remains in place until the bits in the writeprotect register are cleared.

## Configuring Security Privileges of a Peripheral

Use the following procedure to configure the security privileges of a peripheral.

1. Determine the peripheral and its corresponding secure peripheral register ( SPU\_SECUREP[n] ) in the SPU. See the "Write-Protect and Secure Peripheral Registers" section.
2. If the peripheral is to be a secure slave only accepting secure transactions, set bit 0 ( SPU\_SECUREP[n].SSEC ).
3. If the peripheral is to be a secure master that generates secure transactions (keeping in mind not all peripherals can be masters), set bit 1 ( SPU\_SECUREP[n].MSEC ).

This procedure sets the security privileges of a peripheral.

NOTE: Only a secure master can set security privileges, keeping the chain of trust intact. If a non-secure master configures the security privileges, it can undermine security protection.

## ADSP-SC58x SPU Register Descriptions

System Protection Unit (SPU) contains the following registers.

Table 43-3: ADSP-SC58x SPU Register List

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

![Image](46_System_Protection_Unit_(SPU)_artifacts/image_000004_235008c1d3ee6e8bc96279649ee37187b2e5bdf57f88d73e2bcd918d1ac027ab.png)

Write Protect Register Lock

Figure 43-5: SPU\_CTL Register Diagram

Table 43-4: SPU\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | WPLCK      | Write Protect Register Lock. When the SPU_CTL.WPLCK bit is set in combination with the SPU_CTL.GLCK bit, writes to the SPU's write protect registers are blocked and return an error. 0 Disable                               |
| 14 (R/W)           | PINTEN     | Protection Violation Interrupt Enable. When the SPU_CTL.PINTEN bit is set (=1), a block of any transaction according to the configured settings produces an interrupt.                                                        |
| 7:0 (R/W)          | GLCK       | Global Lock. The SPU_CTL.GLCK controls the global lock signal. The global lock signal provides register-based write protection. Writing 0xAD to this field disables the lock, and writ- ing any other value enables the lock. |

## Secure Check Register

The SPU\_SECURECHK register reads by secure masters return 0xFFFFFFFF . Reads by non-secure masters return 0x00000000.

Figure 43-6: SPU\_SECURECHK Register Diagram

![Image](46_System_Protection_Unit_(SPU)_artifacts/image_000005_bc75b95359610f8b34a9f196538f8a57f37e1d174912bdaaacb9b98f03d05de4.png)

Table 43-5: SPU\_SECURECHK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | DATA       | Read Data. The SPU_SECURECHK.DATA bit field performs reads. Reads by secure masters re- turn 0xFFFFFFFF. Reads by non-secure masters return 0x00000000. |

## Secure Control Register

The SPU Secure Control Register ( SPU\_SECURECTL ) allows the user to lock write access to all the SPU\_SECUREC[n] and SPU\_SECUREP[n] registers as well as configure the interrupt generation in an event of a security error. It also allows bulk clear of the SSEC bits and/or MSEC bits in the SPU\_SECUREP[n] registers.

![Image](46_System_Protection_Unit_(SPU)_artifacts/image_000006_fb963aadaf1613652db3a80542965db14997ec393f55f0aa6875ea6f4b8307df.png)

Secure Register Lock

Figure 43-7: SPU\_SECURECTL Register Diagram

Table 43-6: SPU\_SECURECTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | SCRLCK     | Secure Register Lock. When the SPU_SECURECTL.SCRLCK bit is set in combination with the SPU_CTL.GLCK bit, writes to the Security Configuration registers ( SPU_SECUREC[n] and SPU_SECUREP[n] ) are blocked and return an error which is captured in the SPU_STAT.LWERR bit. |
| 14 (R/W)           | SINTEN     | Secure Violation Interrupt Enable. The SPU_SECURECTL.SINTEN bit generates an interrupt if a security violation was captured. Interrupt status is provided in the SPU_STAT.VIRQ bit.                                                                                        |
| 5 (R0/W)           | MSECCLR    | 1 Enable Master Secure Clear. When the SPU_SECURECTL.MSECCLR bit is set, the SPU_SECUREP[n].MSEC bits in all SPU_SECUREP[n] registers are cleared. The SPU_SECURECTL.MSECCLR bit always reads back as a 0. 0 No Action 1 Clear All Master Secure Control Bits              |

Table 43-6: SPU\_SECURECTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R0/W)           | SSECCLR    | Slave Secure Clear. When the SPU_SECURECTL.SSECCLR bit is set, the SPU_SECUREP[n].SSEC bits in all SPU_SECUREP[n] registers are cleared. The SPU_SECURECTL.SSECCLR bit always reads back as a 0. |
| 4 (R0/W)           | SSECCLR    | 0 No Action                                                                                                                                                                                      |
| 4 (R0/W)           | SSECCLR    | 1 Clear All Slave Secure Control Bits                                                                                                                                                            |

## Secure Core Registers

A SPU register exists for every DSP core in the system. The bits enable or disable security for features in the core.

Figure 43-8: SPU\_SECUREC[n] Register Diagram

![Image](46_System_Protection_Unit_(SPU)_artifacts/image_000007_b8833a9ea6cb0a7c812349d7a301bf72e15d30fae64c404ef840865b9fd7f4d8.png)

Table 43-7: SPU\_SECUREC[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | CSEC[n]    | Core Secure. The SPU_SECUREC[n].CSEC[n] bit controls whether non-secure accesses are al- lowed to L1 memory of the processor core. When =1, the core (as a slave) is set as se- cure meaning only secure transactions are allowed to L1. |
| 0 (R/W)            | CSEC[n]    | 0 Disable                                                                                                                                                                                                                                |
| 0 (R/W)            | CSEC[n]    | 1 Enable                                                                                                                                                                                                                                 |

## Secure Peripheral Register

In the system, each SPU\_SECUREP[n] register is assigned to a specific MMR address range associated with one peripheral. Each SPU\_SECUREP[n] has a Slave Secure (SSEC) bit and a Master Secure (MSEC) bit. When the Slave Secure (SSEC) bit is set, the SPU will only allow Secure Masters generating secure transactions to access the peripheral's MMR address space. When the Master Secure (MSEC) bit is set, the associated peripheral will be secure and will generate secure transactions.

Figure 43-9: SPU\_SECUREP[n] Register Diagram

![Image](46_System_Protection_Unit_(SPU)_artifacts/image_000008_17d6dde4cf53f02d446399070932b9e73e9a6f16d73569effe6046a2cb5f22c4.png)

Table 43-8: SPU\_SECUREP[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | MSEC       | Master Secure Enable. The SPU_SECUREP[n].MSEC bit controls whether the peripheral generates secure transactions as a master. When clear (=0), the peripheral generates non-secure transac- tions as a master (if applicable). When set (=1), the peripheral generates secure transac- tions as a master. 0 Disable                                            |
| 0 (R/W)            | SSEC       | Slave Secure Enable. The SPU_SECUREP[n].SSEC bit controls whether the peripheral is protected from non-secure transactions. When clear (=0), the security status of the transaction is ignored. When set (=1), only secure transactions are allowed to access the address space of the peripheral and non-secure transactions are blocked. 0 Disable 1 Enable |

## Status Register

The SPU\_STAT register indicates if there have been any errors, active interrupts and global lock status.

Figure 43-10: SPU\_STAT Register Diagram

![Image](46_System_Protection_Unit_(SPU)_artifacts/image_000009_b9c14244ba77cdf67186f0676d10b2e6c2116be3c986a8b969c10c3f8b984ec2.png)

Table 43-9: SPU\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | LWERR      | Lock Write Error. The SPU_STAT.LWERR indicates whether there has been an attempted write to a register in the SPU with its lock bit ( SPU_CTL.WPLCK or SCRLCK) set while SPU_CTL.GLCK was asserted. This bit is W1C. |
| 30 (R/W1C)         | ADDRERR    | Address Error. The SPU_STAT.ADDRERR indicates whether there has been an attempted write to a read-only register or an access an invalid address in the SPU MMRaddress range. This bit is W1C.                        |
| 12 (R/W1C)         | VIRQ       | Violation Interrupt Request. The SPU_STAT.VIRQ bit indicates that a security and/or protection violation has been detected and interrupt asserted. This is a W1C bit. 0 Inactive                                     |
| 0 (R/NW)           | GLCK       | Global Lock Status. The SPU_STAT.GLCK indicates whether the global lock is enabled or disabled. 0 Disabled (global_lock=0)                                                                                           |

## Write Protect Register n

In the system, each SPU\_WP[n] register is assigned to a specific MMR address range associated with one peripheral. When the appropriate bits are set, writes to the peripheral from a specific master are blocked and an error is returned to the master. For more information, see the processor specific additional information for the SPU\_WP[n] register.

Figure 43-11: SPU\_WP[n] Register Diagram

![Image](46_System_Protection_Unit_(SPU)_artifacts/image_000010_5e01b281871fd37858e42aa31e9cb0ba8f369d2576fe6f88e2b388d963f650ba.png)

Table 43-10: SPU\_WP[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19:16 (R/W)        | SM[n]      | System Master x Write Protect Enable. The SPU_WP[n].SM[n] bits correspond to different system masters in the system. When a particular bit is set in this field, the corresponding system master cannot write to the corresponding peripheral's MMRaddress space. The write attempt is blocked by the SPU.                                        |
| 2:0 (R/W)          | CM[n]      | Core Master x Write Protect Enable. The SPU_WP[n].CM[n] bits correspond to different cores in the system. When a particular bit is set in this field, the corresponding core cannot write to the correspond- ing peripheral's MMRaddress space. The write attempt is blocked by the SPU. Core ID 0 = M0 Supervisor and Core ID 1 = M4 Controller. |

## ADSP-SC5xx Write-Protect, Secure Peripheral and Secure Core Registers

The SPU consists of a collection of write-protect registers each of which are associated with a specific peripheral or slave. The SPU also has a collection of secure peripheral registers which are also associated with specific peripherals. The table gives the write-protect register and secure peripheral number for each of the peripherals that are provided with write protection and security through the SPU. The SPU for ADSP-SC5xx is configured with 188 write-protect registers and also 188 secure peripheral registers. The number corresponding to a peripheral correlates to both the Write-Protect register and the Secure Peripheral register.

For each processor, there are different numbers of masters that are able to access the SMMR space. The SPU\_WPn.CMn and SPU\_WPn.SMn Bits table shows which bits enable the protection against which master.

Table 43-11: SPU\_WPn.CMn and SPU\_WPn.SMn Bits

|   Bit Number | Bit Name   | Description                |
|--------------|------------|----------------------------|
|            0 | CM_WP[0]   | ARM Core 0                 |
|            1 | CM_WP[1]   | SHARC Core 1               |
|            2 | CM_WP[2]   | SHARC Core 2               |
|           15 | SM_WP[0]   | PCIe                       |
|           16 | SM_WP[1]   | DBG                        |
|           17 | SM_WP[2]   | Embeded Trace Router (ETR) |
|           18 | SM_WP[3]   | Enhanced BandwidthMDMA     |

For each peripheral, there is a corresponding write-protect register, SPU\_WP[n] , and secure peripheral register, SPU\_SECUREP[n] . The table shows the write-protect register and secure peripheral number for each peripheral.

Table 43-12: Write-Protect Register and Secure Peripheral Number (n)

|   No | Peripheral/Block Name   | SWPn Mapping   | SECUREPn Mapping   | Master Capable   |
|------|-------------------------|----------------|--------------------|------------------|
|    0 | Reserved                | WP0            | SECUREP0           | N/A              |
|    1 | Reserved                | WP1            | SECUREP1           | N/A              |
|    2 | Reserved                | WP2            | SECUREP2           | N/A              |
|    3 | LP0                     | WP3            | SECUREP3           | N/A              |
|    4 | LP1                     | WP4            | SECUREP4           | N/A              |
|    5 | LP0 DMA30               | WP5            | SECUREP5           | Yes              |
|    6 | LP1 DMA36               | WP6            | SECUREP6           | Yes              |
|    7 | CAN0                    | WP7            | SECUREP7           | N/A              |
|    8 | CAN1                    | WP8            | SECUREP8           | N/A              |
|    9 | TIMER0                  | WP9            | SECUREP9           | N/A              |
|   10 | CRC0                    | WP10           | SECUREP10          | N/A              |
|   11 | CRC1                    | WP11           | SECUREP11          | N/A              |
|   12 | TWI0                    | WP12           | SECUREP12          | N/A              |
|   13 | TWI1                    | WP13           | SECUREP13          | N/A              |
|   14 | TWI2                    | WP14           | SECUREP14          | N/A              |
|   15 | SPORT 0A                | WP15           | SECUREP15          | N/A              |
|   16 | SPORT 0B                | WP16           | SECUREP16          | N/A              |
|   17 | SPORT 1A                | WP17           | SECUREP17          | N/A              |

Table 43-12: Write-Protect Register and Secure Peripheral Number (n) (Continued)

|   No | Peripheral/Block Name   | SWPn Mapping   | SECUREPn Mapping   | Master Capable   |
|------|-------------------------|----------------|--------------------|------------------|
|   18 | SPORT 1B                | WP18           | SECUREP18          | N/A              |
|   19 | SPORT 2A                | WP19           | SECUREP19          | N/A              |
|   20 | SPORT 2B                | WP20           | SECUREP20          | N/A              |
|   21 | SPORT 3A                | WP21           | SECUREP21          | N/A              |
|   22 | SPORT 3B                | WP22           | SECUREP22          | N/A              |
|   23 | SPORT 4A                | WP23           | SECUREP23          | N/A              |
|   24 | SPORT 4B                | WP24           | SECUREP24          | N/A              |
|   25 | SPORT 5A                | WP25           | SECUREP25          | N/A              |
|   26 | SPORT 5B                | WP26           | SECUREP26          | N/A              |
|   27 | SPORT 6A                | WP27           | SECUREP27          | N/A              |
|   28 | SPORT 6B                | WP28           | SECUREP28          | N/A              |
|   29 | SPORT 7A                | WP29           | SECUREP29          | N/A              |
|   30 | SPORT 7B                | WP30           | SECUREP30          | N/A              |
|   31 | UART0                   | WP31           | SECUREP31          | N/A              |
|   32 | UART1                   | WP32           | SECUREP32          | N/A              |
|   33 | UART2                   | WP33           | SECUREP33          | N/A              |
|   34 | PORTA                   | WP34           | SECUREP34          | N/A              |
|   35 | PORTB                   | WP35           | SECUREP35          | N/A              |
|   36 | PORTC                   | WP36           | SECUREP36          | N/A              |
|   37 | PORTD                   | WP37           | SECUREP37          | N/A              |
|   38 | PORTE                   | WP38           | SECUREP38          | N/A              |
|   39 | PORTF                   | WP39           | SECUREP39          | N/A              |
|   40 | PORTG                   | WP40           | SECUREP40          | N/A              |
|   41 | PADS                    | WP41           | SECUREP41          | N/A              |
|   42 | PINT0                   | WP42           | SECUREP42          | N/A              |
|   43 | PINT1                   | WP43           | SECUREP43          | N/A              |
|   44 | PINT2                   | WP44           | SECUREP44          | N/A              |
|   45 | PINT3                   | WP45           | SECUREP45          | N/A              |
|   46 | PINT4                   | WP46           | SECUREP46          | N/A              |
|   47 | PINT5                   | WP47           | SECUREP47          | N/A              |
|   48 | SMC0                    | WP48           | SECUREP48          | N/A              |

Table 43-12: Write-Protect Register and Secure Peripheral Number (n) (Continued)

|   No | Peripheral/Block Name   | SWPn Mapping   | SECUREPn Mapping   | Master Capable   |
|------|-------------------------|----------------|--------------------|------------------|
|   49 | SMPU-SMC                | WP49           | SECUREP49          | N/A              |
|   50 | WDT0                    | WP50           | SECUREP50          | N/A              |
|   51 | WDT1                    | WP51           | SECUREP51          | N/A              |
|   52 | EPWM0                   | WP52           | SECUREP52          | N/A              |
|   53 | EPWM1                   | WP53           | SECUREP53          | N/A              |
|   54 | EPWM2                   | WP54           | SECUREP54          | N/A              |
|   55 | EMAC0                   | WP55           | SECUREP55          | Yes              |
|   56 | EMAC1                   | WP56           | SECUREP56          | Yes              |
|   57 | CNT0                    | WP57           | SECUREP57          | N/A              |
|   58 | MSI0                    | WP58           | SECUREP58          | Yes              |
|   59 | OTP-MMR                 | WP59           | SECUREP59          | N/A              |
|   60 | SINC0                   | WP60           | SECUREP60          | Yes              |
|   61 | SWU-SMC                 | WP61           | SECUREP61          | N/A              |
|   62 | TMU0                    | WP62           | SECUREP62          | N/A              |
|   63 | HADC0                   | WP63           | SECUREP63          | N/A              |
|   64 | HAE0                    | WP64           | SECUREP64          | N/A              |
|   65 | ACM0                    | WP65           | SECUREP65          | N/A              |
|   66 | SPORT 0A DMA0           | WP66           | SECUREP66          | Yes              |
|   67 | SPORT 0B DMA1           | WP67           | SECUREP67          | Yes              |
|   68 | SPORT 1A DMA2           | WP68           | SECUREP68          | Yes              |
|   69 | SPORT 1B DMA3           | WP69           | SECUREP69          | Yes              |
|   70 | SPORT 2A DMA4           | WP70           | SECUREP70          | Yes              |
|   71 | SPORT 2B DMA5           | WP71           | SECUREP71          | Yes              |
|   72 | SPORT 3A DMA6           | WP72           | SECUREP72          | Yes              |
|   73 | SPORT 3B DMA7           | WP73           | SECUREP73          | Yes              |
|   74 | SPORT 4A DMA10          | WP74           | SECUREP74          | Yes              |
|   75 | SPORT 4B DMA11          | WP75           | SECUREP75          | Yes              |
|   76 | SPORT 5A DMA12          | WP76           | SECUREP76          | Yes              |
|   77 | SPORT 5B DMA13          | WP77           | SECUREP77          | Yes              |
|   78 | SPORT 6A DMA14          | WP78           | SECUREP78          | Yes              |
|   79 | SPORT 6B DMA15          | WP79           | SECUREP79          | Yes              |

Table 43-12: Write-Protect Register and Secure Peripheral Number (n) (Continued)

|   No | Peripheral/Block Name   | SWPn Mapping   | SECUREPn Mapping   | Master Capable   |
|------|-------------------------|----------------|--------------------|------------------|
|   80 | SPORT 7A DMA16          | WP80           | SECUREP80          | Yes              |
|   81 | SPORT 7B DMA17          | WP81           | SECUREP81          | Yes              |
|   82 | UART0 RX DMA20          | WP82           | SECUREP82          | Yes              |
|   83 | UART0 TX DMA21          | WP83           | SECUREP83          | Yes              |
|   84 | UART1 RX DMA34          | WP84           | SECUREP84          | Yes              |
|   85 | UART1 TX DMA35          | WP85           | SECUREP85          | Yes              |
|   86 | UART2 RX DMA37          | WP86           | SECUREP86          | Yes              |
|   87 | UART2 TX DMA38          | WP87           | SECUREP87          | Yes              |
|   88 | MDMA0 SRC CRC0 DMA8     | WP88           | SECUREP88          | Yes              |
|   89 | MDMA0 DST CRC0 DMA9     | WP89           | SECUREP89          | Yes              |
|   90 | MDMA1 SRC CRC1 DMA18    | WP90           | SECUREP90          | Yes              |
|   91 | MDMA1 DST CRC1 DMA19    | WP91           | SECUREP91          | Yes              |
|   92 | HAE RX0 DMA32           | WP92           | SECUREP92          | Yes              |
|   93 | HAE RX1 DMA33           | WP93           | SECUREP93          | Yes              |
|   94 | HAE TX0 DMA31           | WP94           | SECUREP94          | Yes              |
|   95 | EPPI0                   | WP95           | SECUREP95          | N/A              |
|   96 | SWU-SPIF(SPI2)          | WP96           | SECUREP96          | N/A              |
|   97 | SPI0                    | WP97           | SECUREP97          | N/A              |
|   98 | SPI1                    | WP98           | SECUREP98          | N/A              |
|   99 | SPI2                    | WP99           | SECUREP99          | N/A              |
|  100 | SWU-PCIE-SLV            | WP100          | SECUREP100         | N/A              |
|  101 | SPI0 TX DMA22           | WP101          | SECUREP101         | Yes              |
|  102 | SPI0 RX DMA23           | WP102          | SECUREP102         | Yes              |
|  103 | SPI1 TX DMA24           | WP103          | SECUREP103         | Yes              |
|  104 | SPI1 RX DMA25           | WP104          | SECUREP104         | Yes              |
|  105 | SPI2 TX DMA26           | WP105          | SECUREP105         | Yes              |
|  106 | SPI2 RX DMA27           | WP106          | SECUREP106         | Yes              |
|  107 | EPPI0 Channel 0 DMA28   | WP107          | SECUREP107         | Yes              |
|  108 | EPPI0 Channel 1 DMA29   | WP108          | SECUREP108         | Yes              |
|  109 | DMC0                    | WP109          | SECUREP109         | N/A              |
|  110 | DMC0-PHY                | WP110          | SECUREP110         | N/A              |

Table 43-12: Write-Protect Register and Secure Peripheral Number (n) (Continued)

|   No | Peripheral/Block Name   | SWPn Mapping   | SECUREPn Mapping   | Master Capable   |
|------|-------------------------|----------------|--------------------|------------------|
|  111 | DMC0-DFT                | WP111          | SECUREP111         | N/A              |
|  112 | DMC1                    | WP112          | SECUREP112         | N/A              |
|  113 | DMC1-PHY                | WP113          | SECUREP113         | N/A              |
|  114 | DMC1-DFT                | WP114          | SECUREP114         | N/A              |
|  115 | L2CTLport0              | WP115          | SECUREP115         | N/A              |
|  116 | L2CTL port 1            | WP116          | SECUREP116         | N/A              |
|  117 | L2CTL port 2            | WP117          | SECUREP117         | N/A              |
|  118 | SMPU-L2CTL, Core Port 0 | WP118          | SECUREP118         | N/A              |
|  119 | SMPU-L2CTL, DMAPort 0   | WP119          | SECUREP119         | N/A              |
|  120 | SMPU-L2CTL, Core Port 1 | WP120          | SECUREP120         | N/A              |
|  121 | SMPU-L2CTL, DMAPort1    | WP121          | SECUREP121         | N/A              |
|  122 | SMPU-L2CTL, Core Port 2 | WP122          | SECUREP122         | N/A              |
|  123 | SMPU-L2CTL, DMAPort 2   | WP123          | SECUREP123         | N/A              |
|  124 | SEC0                    | WP124          | SECUREP124         | N/A              |
|  125 | TRU0                    | WP125          | SECUREP125         | N/A              |
|  126 | RCU0                    | WP126          | SECUREP126         | N/A              |
|  127 | SPU0                    | WP127          | SECUREP127         | N/A              |
|  128 | CGU0(PLL-Dig)           | WP128          | SECUREP128         | N/A              |
|  129 | CGU1(PLL-Dig)           | WP129          | SECUREP129         | N/A              |
|  130 | CDU0                    | WP130          | SECUREP130         | N/A              |
|  131 | DPM0                    | WP131          | SECUREP131         | N/A              |
|  132 | SWU_L2CTL,Core Port 0   | WP132          | SECUREP132         | N/A              |
|  133 | SWU_L2CTL, DMAPort 0    | WP133          | SECUREP133         | N/A              |
|  134 | SWU_L2CTL, Core Port 1  | WP134          | SECUREP134         | N/A              |
|  135 | SWU_L2CTL, DMAPort 1    | WP135          | SECUREP135         | N/A              |
|  136 | SWU_L2CTL, Core Port 2  | WP136          | SECUREP136         | N/A              |
|  137 | SWU_L2CTL, DMAPort 2    | WP137          | SECUREP137         | N/A              |
|  138 | SWU-SMMR                | WP138          | SECUREP138         | N/A              |
|  139 | SMPU-PCIE-SLV           | WP139          | SECUREP139         | N/A              |
|  140 | MediumBWMDMA            | WP140          | SECUREP140         | Yes              |
|  141 | MaximumBWMDMA           | WP141          | SECUREP141         | Yes              |

Table 43-12: Write-Protect Register and Secure Peripheral Number (n) (Continued)

|   No | Peripheral/Block Name              | SWPn Mapping   | SECUREPn Mapping   | Master Capable   |
|------|------------------------------------|----------------|--------------------|------------------|
|  142 | FFTA0                              | WP142          | SECUREP142         | Yes              |
|  143 | MLB0                               | WP143          | SECUREP143         | Yes              |
|  144 | SWU-DMC0                           | WP144          | SECUREP144         | N/A              |
|  145 | SWU-DMC1                           | WP145          | SECUREP145         | N/A              |
|  146 | SMPU-DMC0                          | WP146          | SECUREP146         | N/A              |
|  147 | SMPU-DMC1                          | WP147          | SECUREP147         | N/A              |
|  148 | STM0                               | WP148          | SECUREP148         | N/A              |
|  149 | GIC-Port 0 (Cortex A5 Distributor) | WP149          | SECUREP149         | N/A              |
|  150 | GIC-Port 1 (Cortex A5 Core)        | WP150          | SECUREP150         | N/A              |
|  151 | PCIE-DBI+PCIE-RSCKPHY              | WP151          | SECUREP151         | Yes              |
|  153 | USB0                               | WP153          | SECUREP153         | Yes              |
|  154 | USB1                               | WP154          | SECUREP154         | Yes              |
|  155 | FIR0                               | WP155          | SECUREP155         | Yes              |
|  156 | IIR0                               | WP156          | SECUREP156         | Yes              |
|  158 | EMDMA0 (2 channels)                | WP158          | SECUREP158         | Yes              |
|  159 | RTC0                               | WP160          | SECUREP160         | N/A              |
|  160 | DAI0                               | WP161          | SECUREP161         | N/A              |
|  161 | DAI1                               | WP162          | SECUREP162         | N/A              |
|  162 | PKTE                               | WP163          | SECUREP163         | Yes              |
|  163 | PKA                                | WP164          | SECUREP164         | N/A              |
|  164 | DAPROM                             | WP165          | SECUREP165         | N/A              |
|  165 | SHARC1 DBG                         | WP166          | SECUREP166         | N/A              |
|  166 | SHARC1 CTI                         | WP167          | SECUREP167         | N/A              |
|  167 | SHARC1 PTM                         | WP168          | SECUREP168         | N/A              |
|  168 | STM                                | WP169          | SECUREP169         | N/A              |
|  169 | SHARC2 DBG                         | WP170          | SECUREP170         | N/A              |
|  170 | SHARC2 CTI                         | WP171          | SECUREP171         | N/A              |
|  171 | SHARC2 PTM                         | WP172          | SECUREP172         | N/A              |
|  172 | CSTF                               | WP173          | SECUREP173         | N/A              |
|  173 | ETF                                | WP174          | SECUREP174         | N/A              |
|  174 | ETR                                | WP175          | SECUREP175         | N/A              |

Table 43-12: Write-Protect Register and Secure Peripheral Number (n) (Continued)

|   No | Peripheral/Block Name   | SWPn Mapping   | SECUREPn Mapping   | Master Capable   |
|------|-------------------------|----------------|--------------------|------------------|
|  175 | TPIU                    | WP176          | SECUREP176         | N/A              |
|  176 | CTI Trace               | WP177          | SECUREP177         | N/A              |
|  177 | CTI System              | WP178          | SECUREP178         | N/A              |
|  178 | A5 IntegrationROM       | WP179          | SECUREP179         | N/A              |
|  179 | A5 DBG                  | WP180          | SECUREP180         | N/A              |
|  180 | A5 PMU                  | WP181          | SECUREP181         | N/A              |
|  181 | A5 CTI                  | WP182          | SECUREP182         | N/A              |
|  182 | A5 ETM                  | WP183          | SECUREP183         | N/A              |
|  183 | TAPCMMR                 | WP184          | SECUREP184         | N/A              |
|  184 | DebugControl            | WP185          | SECUREP185         | N/A              |
|  185 | SWU Core 1 Slave Port 1 | WP186          | SECUREP186         | N/A              |
|  186 | SWU Core 1 Slave Port 2 | WP187          | SECUREP187         | N/A              |
|  187 | SWU Core 2 Slave Port 1 | WP188          | SECUREP188         | N/A              |
|  188 | SWU Core 2 Slave Port 2 | WP189          | SECUREP189         | N/A              |

## ADSP-SC5xx Specific Information

The information in this section applies specifically to the ADSP-SC5xx processor models

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
- L2 Memory Controller (L2CTL1)
- L2 Memory Controller (L2CTL2)