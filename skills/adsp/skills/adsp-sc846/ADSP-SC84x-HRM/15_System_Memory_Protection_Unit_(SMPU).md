## 12   System Memory Protection Unit (SMPU)

The SMPU provides a flexible way of protecting memory regions against read or write access from any or all bus requester in the system. In addition, it can guard against memory access depending on security privileges of the system bus requester.

## SMPU Features

The system memory protection unit has the following features.

- After reset, the default state of the system is fully open. The SMPUs admit any access to memory spaces by any controller.
- On reset, secure transactions are only allowed; non secure transactions are disabled.
- Each SMPU instance can be configured to monitor multiple regions. Each can be individually enabled.
- Each region can be configured with its own protection settings.
- Provides general read or write protection.
- Read and write transactions are restricted or allowed depending on the transaction ID.

On the ADSP-SC84x/ADSP-2184x processor, 15 SMPU instances are available to protect the L2, external memory (DMC) interface.

Table 12-1: SMPU Instances

| Module                 |   SMPU Instance | Controller Allowed                                              | L2 Memory Address Range                                                                  |
|------------------------|-----------------|-----------------------------------------------------------------|------------------------------------------------------------------------------------------|
| L2-Core Port 0 (CL2_0) |               2 | Data port of both the SHARC-FX and Arm and IIR accelerator port | 0x20400000 - 0x205FFFFF (L2 SRAM Banks 0-19) 0x20200000 - 0x2021FFFF (L2 Boot ROM0and 1) |
| L2 DMAPort 0 (DL2_0)   |               3 | HSMDMA                                                          | 0x20400000 - 0x208FFFFF (L2 SRAM Banks 0-19) 0x20200000 - 0x2021FFFF (L2 Boot ROM0and 1) |
| L2-Core Port 1 (CL2_1) |               4 | Data port of the SHARC-FX and Arm and IIR accelerator port      | 0x20600000 - 0x208FFFFF (L2 SRAM Banks 16-19)                                            |
| L2 DMAPort 1 (DL2_1)   |               5 | Peripheral and remainingDMA                                     | 0x20400000 - 0x208FFFFF (L2 SRAM Banks 0-19) 0x20200000 - 0x2021FFFF (L2 Boot ROM0and 1) |

Table 12-1: SMPU Instances (Continued)

| Module                        |   SMPU Instance | Controller Allowed                                          | L2 Memory Address Range                                                                  |
|-------------------------------|-----------------|-------------------------------------------------------------|------------------------------------------------------------------------------------------|
| L2-Core Port 2 (CL2_2)        |               6 | Instruction port of both SHARC- FX and FIR accelerator port | 0x20400000 - 0x208FFFFF (L2 SRAM Banks 0-19) 0x20200000 - 0x2021FFFF (L2 Boot ROM0and 1) |
| DMC0                          |               9 | All allowed requesters                                      | Entire range                                                                             |
| SPI2/xSPI Flash Address Space |              11 | All allowed requesters                                      | Entire range                                                                             |
| OTP Memory                    |              12 | All allowed requesters                                      | Entire range                                                                             |

All the SMPU instances can be configured up to eight regions.

- Up to eight outstanding read or write transactions supported on SMPU instances for Core\_L2 and DMC0. Up to four outstanding read or write transactions supported on the SMPU instances for DMA\_L2 and SPI flash address space.

## SMPU Functional Description

The following sections provide details on the function of the SMPU module. If the region security settings allow transactions to go through, the ID in the ID-based region protection settings can still filter the transactions.

For the memory that an SMPU protects, programs can configure region-based settings with the SMPU\_RCTL[n] registers. (There can be multiple SMPUs in a system.) The SMPU\_RCTL[n] registers define the ID-based protection for memory regions.

If the target address does not reside in any configured memory region, the transaction permission resorts back to the global configuration setting.

## Protection Units

Each SMPU provides two protection units, A and B for ID-based matching in the region-based memory protection. This feature provides a degree of flexibility for the user to match against multiple IDs.

## Instruction Fetches

When the core executes instructions from memory, this operation is also considered a memory transaction. If the SMPU is configured to protect a memory region from read accesses that contain instructions, the core cannot fetch and execute these instructions.

## Using Cache

When the processor uses both cache and the SMPU, there are a few issues to be aware of. If the SMPU is configured to protect a memory region from write accesses, instruction fetches from a core are still possible since instructions are not updated and replaced during run time.

NOTE: The debugger typically replaces an instruction with a breakpoint instruction for software breakpoints. If a memory region is protected against write accesses, software breakpoints are not possible unless the SMPU is configured with the appropriate system controller ID of the debugger. The configuration allows it to perform a write-access.

In the case where memory is used for data, a read access or cache fill is not possible if the memory is blocked from read accesses. If read accesses are allowed but write accesses are disallowed, then there is an issue with coherency. The cache is filled but when the cache is updated and must be written back to the SMPU protected memory, the write-access is blocked.

In general, exercise caution when using both the SMPU and cache.

## Speculative Reads

If speculative reads are enabled ( SMPU\_CTL.RSDIS =0), the SMPU forwards the read transaction directly to the memory before checking the protection setting corresponding to the addressed memory region. This functionality saves one clock cycle in the clock domain of the SMPU. The SMPU checks the protection setting while the read transaction occurs with the memory. If the protection setting dictates that the target memory address is blocked, the SMPU blocks the read to the controller.

If speculative reads are disabled ( SMPU\_CTL.RSDIS =1), the SMPU checks the protection settings first and forwards the transaction to memory only if it passes the configured protection settings. This functionality incurs a one-cycle latency per read.

NOTE: Reads affect certain memory operations such as automatic clearing of the memory (that is, FIFOs). When the SMPU protects this type of memory, disable read speculation since the blocking can occur without the read transaction reaching the target memory.

## ADSP-2184x SMPU Register List

The System Memory Protection Unit (SMPU) provides selective protection of the processor's memory resources. The SMPU includes a set of processor events that can be monitored during program execution. A set of registers governs SMPU operations. For more information on SMPU functionality, see the SMPU register descriptions.

Table 12-2: ADSP-2184x SMPU Register List

| Name             | Description                  |
|------------------|------------------------------|
| SMPU_BADDR       | Bus Error Address Register   |
| SMPU_BDTLS       | Bus Error Details Register   |
| SMPU_CTL         | SMPU Control Register        |
| SMPU_EXACADD[n]  | Exclusive Access IDn Address |
| SMPU_EXACSTAT[n] | Exclusive Access Status      |
| SMPU_IADDR       | Interrupt Address Register   |

Table 12-2: ADSP-2184x SMPU Register List (Continued)

| Name               | Description                               |
|--------------------|-------------------------------------------|
| SMPU_IDTLS         | Interrupt Details Register                |
| SMPU_RADDR[n]      | Region n Address Register                 |
| SMPU_RCTL[n]       | Region n Control Register                 |
| SMPU_REVID         | SMPU Revision ID Register                 |
| SMPU_RIDA[n]       | Region n ID A Register                    |
| SMPU_RIDB[n]       | Region n ID B Register                    |
| SMPU_RIDMSKA[n]    | Region n ID Mask A Register               |
| SMPU_RIDMSKB[n]    | Region n ID Mask B Register               |
| SMPU_SECURECTL     | SMPU Control Secure Accesses Register     |
| SMPU_SECURERCTL[n] | Region n Control Secure Accesses Register |
| SMPU_STAT          | SMPU Status Register                      |

## SMPU Interrupts

The SMPU has one interrupt with the SEC ID = 142.

See the SEC chapter for complete information on interrupt generation and use.

## Memory Writes

A write transaction to address n is prevented when the following is true.

Address n is in memory region m and memory region m is write-protected ( SMPU\_RCTL[n].WPROTEN =1) and ID is not a match. (See ID Comparison). The block occurs because the memory region is configured for write-protection and the ID comparison does not result in a match. If an ID comparison results in a match, the write transaction is allowed through.

## Memory Reads

A read transaction from address n is prevented when the following is true:

- Address n is in memory region and memory region m is read-protected ( SMPU\_RCTL[n].RPROTEN =1) and
- ID is not a match

The block occurs because the memory region is configured for read protection and the ID comparison does not result in a match. If the ID comparison results in a match, the read transaction is permitted. (See ID Comparison).

## ID Comparison

ID comparison automatically occurs during region-based memory protection. ID matches allow the transaction to bypass the configured memory protection for that region. The following sections describe the calculation of a write ID match and read ID match.

## Write Transaction

The state of the following values determines the ID value that is compared with the ID of an incoming write transaction:

- The SMPU\_RCTL[n].WIDCINV bit
- The SMPU\_RIDA[n].ID and SMPU\_RIDB[n].ID bit fields
- The SMPU\_RIDMSKA[n].MSK and SMPU\_RIDMSKB[n].MSK bit fields

Write IDA match = ((ID of incoming write transaction AND SMPU\_RIDMSKA[n].MSK ) == ( SMPU\_RIDA[n].ID AND SMPU\_RIDMSKA[n].MSK ))

Write IDB match = ((ID of incoming write transaction AND SMPU\_RIDMSKB[n].MSK ) == ( SMPU\_RIDB[n].ID AND SMPU\_RIDMSKB[n].MSK ))

Write ID match = (Write IDA match OR Write IDB match) XOR SMPU\_RCTL[n].WIDCINV bit

## Read Transaction

The state of the following values determines the ID value that is compared with the ID of an incoming read transaction:

- The SMPU\_RCTL[n].RIDCINV bit
- The SMPU\_RIDA[n].ID and SMPU\_RIDB[n].ID bit fields
- The SMPU\_RIDMSKA[n].MSK and SMPU\_RIDMSKB[n].MSK bit fields

Read IDA match = ((ID of incoming read transaction AND SMPU\_RIDMSKA[n].MSK ) == ( SMPU\_RIDA[n].ID AND SMPU\_RIDMSKA[n].MSK ))

Read IDB match = ((ID of incoming read transaction AND SMPU\_RIDMSKB[n].MSK ) == ( SMPU\_RIDB[n].ID AND SMPU\_RIDMSKB[n].MSK ))

Read ID match = (Read IDA match OR Read IDB match) XOR SMPU\_RCTL[n].RIDCINV

In the two cases described above, the incoming transaction (either write or read) ID is AND'ed with the configured mask value in protection unit A. It is then compared to the value of the configured ID value which is also AND'ed with the configured mask value in protection unit A. The mask provides a method to allow a group of IDs to match. This process is also performed for protection unit B. The two outcomes (from A and B) are then OR'ed together.

Depending on the setting of the SMPU\_RCTL[n].RIDCINV or the SMPU\_RCTL[n].WIDCINV bits, the ID match comparison is inverted or not. The final result after applying the inversion, SMPU\_RCTL[n].RIDCINV , or SMPU\_RCTL[n].WIDCINV , determines whether the transaction bypasses the protection.

## Usage

The masks, SMPU\_RIDMSKA[n] and SMPU\_RIDMSKB[n] , are AND'ed with both the incoming transaction ID and the configured ID in SMPU\_RIDA[n].ID and SMPU\_RIDB[n].ID , respectively. By default the masks are zero. If ID-based region protection is enabled by setting the SMPU\_RCTL[n].WPROTEN or SMPU\_RCTL[n].RPROTEN bit fields and the masks are not set, the ID comparison essentially compares zeros. The comparison allows all transactions to bypass (if the region-based security setting is also configured in a way to allow transactions to go through for the region). To have the ID-based region protection to function, the mask registers and ID registers must also be set.

## System IDs

The System controller IDs table provides the IDs for the system controllers. An x means that the bit can be a 0 or a 1. There are multiple IDs associated with that system controller.

Table 12-3: System Controller IDs

| ASIB Name   |   IID |   SIID | ID              |
|-------------|-------|--------|-----------------|
| SP0A        |     0 |      0 | 11'b0000000x000 |
| SP0B        |     0 |      1 | 11'b0000000x001 |
| SP1A        |     0 |      2 | 11'b0000000x010 |
| SP1B        |     0 |      3 | 11'b0000000x011 |
| SP2A        |     0 |      4 | 11'b0000000x100 |
| SP2B        |     0 |      5 | 11'b0000000x101 |
| SP3A        |     0 |      6 | 11'b0000000x110 |
| SP3B        |     0 |      7 | 11'b0000000x111 |
| CRC0_CH0    |     1 |      6 | 11'b0010000x110 |
| CRC0_CH1    |     1 |      7 | 11'b0010000x111 |
| MLB         |     2 |      4 | 11'b00100000100 |
| UART0_TX    |     3 |      0 | 11'b0011000x000 |
| UART0_RX    |     3 |      4 | 11'b0011000x100 |
| SPI0TX      |     4 |      0 | 11'b0100000x000 |
| SPI0RX      |     4 |      1 | 11'b0100000x001 |
| SPI1TX      |     4 |      2 | 11'b0100000x010 |
| SPI1RX      |     4 |      3 | 11'b0100000x011 |

Table 12-3: System Controller IDs (Continued)

| ASIB Name   |   IID |   SIID | ID              |
|-------------|-------|--------|-----------------|
| SPI2TX      |     4 |      5 | 11'b0100000x101 |
| SPI2RX      |     4 |      4 | 11'b0100000x100 |
| UART1_TX    |     3 |      1 | 11'b0011000x001 |
| UART1_RX    |     3 |      2 | 11'b0011000x010 |
| CRYPTO      |     6 |      2 | 11'b01100000010 |
| ACC_CH0     |     6 |      0 | 11'b0110xxxx000 |
| ACC_CH1     |     6 |      1 | 11'b0110xxxx001 |
| EMDMA0_CH0  |     7 |      0 | 11'b01110000000 |
| EMDMA0_CH1  |     7 |      1 | 11'b01110000001 |
| EMDMA1_CH0  |     8 |      1 | 11'b10000000001 |
| EMDMA1_CH1  |     8 |      0 | 11'b10000000000 |
| MSMDMA_CH0  |     9 |      0 | 11'b1001000x000 |
| MSMDMA_CH1  |     9 |      1 | 11'b1001000x001 |
| DBG         |     9 |      2 | 11'b10010000010 |
| CRC1_CH0    |    10 |      4 | 11'b1010000x100 |
| CRC1_CH1    |    10 |      5 | 11'b1010000x101 |
| SH0_DPORT   |    11 |      0 | 11'b1011xxxx000 |
| SH0_IPORT   |    11 |      1 | 11'b10110000001 |
| HSMDMA_CH0  |    12 |      0 | 11'b1100000x000 |
| HSMDMA_CH1  |    12 |      1 | 11'b1100000x001 |
| SP4A        |    13 |      0 | 11'b1101000x000 |
| SP4B        |    13 |      1 | 11'b1101000x001 |
| SP5A        |    13 |      2 | 11'b1101000x010 |
| SP5B        |    13 |      3 | 11'b1101000x011 |
| SP6A        |    13 |      4 | 11'b1101000x100 |
| SP6B        |    13 |      5 | 11'b1101000x101 |
| SP7A        |    13 |      6 | 11'b1101000x110 |
| SP7B        |    13 |      7 | 11'b1101000x111 |
| SH0_MMR     |    14 |      0 | 11'b1110xxxx000 |

## Memory Region

Memory regions can start at address 0x00000000 or at any address that is a multiple of its size. The Supported Memory Region Size and Alignment table shows the memory region sizes that the processor supports and the alignment of the memory region. (X values are do-not-care).

SMPU supports a maximum of eight regions.

Table 12-4: Supported Memory Region Size and Alignment

| Size   | SMPU_RCTLn.SIZE   | Address    | Possible Values forN              |
|--------|-------------------|------------|-----------------------------------|
| 4KB    | 0b00000           | 0xXXXXX000 | -                                 |
| 8KB    | 0b00001           | 0xXXXXN000 | 0x0, 0x2, 0x4, 0x8, 0xA, 0xC, 0xE |
| 16KB   | 0b00010           | 0xXXXXN000 | 0x0, 0x4, 0x8, 0xC                |
| 32KB   | 0b00011           | 0xXXXXN000 | 0x0, 0x8                          |
| 64KB   | 0b00100           | 0xXXXX0000 | -                                 |
| 128KB  | 0b00101           | 0xXXXN0000 | 0x0, 0x2, 0x4, 0x8, 0xA, 0xC, 0xE |
| 256KB  | 0b00110           | 0xXXXN0000 | 0x0, 0x4, 0x8, 0xC                |
| 512KB  | 0b00111           | 0xXXXN0000 | 0x0, 0x8                          |
| 1MB    | 0b01000           | 0xXXX00000 | -                                 |
| 2MB    | 0b01001           | 0xXXN00000 | 0x0, 0x2, 0x4, 0x8, 0xA, 0xC, 0xE |
| 4MB    | 0b01010           | 0xXXN00000 | 0x0, 0x4, 0x8, 0xC                |
| 8MB    | 0b01011           | 0xXXN00000 | 0x0, 0x8                          |
| 16MB   | 0b01100           | 0xXX000000 | -                                 |
| 32MB   | 0b01101           | 0xXN000000 | 0x0, 0x2, 0x4, 0x8, 0xA, 0xC, 0xE |
| 64MB   | 0b01110           | 0xXN000000 | 0x0, 0x4, 0x8, 0xC                |
| 128MB  | 0b01111           | 0xXN000000 | 0x0, 0x8                          |
| 256MB  | 0b10000           | 0xX0000000 | -                                 |
| 512MB  | 0b10001           | 0xN0000000 | 0x0, 0x2, 0x4, 0x8, 0xA, 0xC, 0xE |
| 1GB    | 0b10010           | 0xN0000000 | 0x0, 0x4, 0x8, 0xC                |
| 2GB    | 0b10011           | 0xN0000000 | 0x0, 0x8                          |
| 4GB    | 0b10100           | 0x00000000 | -                                 |

For the case where the region size is selected as 4 GB, the region address must be at address 0x00000000.

NOTE: If a memory region address is not aligned to its size, the memory region start address protected by the SMPU is the configured address with the corresponding least significant bits masked. For example, if the size is configured for 16 KB ( SMPU\_RCTL[n].SIZE =0b00010), and the base address is configured for SMPU\_RADDR[n].BADDR =0x00005018, the actual base address used by the SMPU is 0x00004000. When SMPU\_RADDR[n].BADDR is read back, the program reads 0x00005000. This functionality is

because only bits [11:0] are reserved as 0's. Programs must use care when setting the base address as it is not always the true base address.

## SMPU Definitions

To make the best use of the SMPU, it is useful to understand the terms in this section.

## Global Protection

Guarding of the entire memory space for the SMPU instantiation.

## Region-Based Protection

Guarding individual segments of memory inside the memory space for the SMPU instantiation.

## ID Match

A successful comparison of the ID associated with the incoming transaction and the ID and MASK configured in the SMPU.

## SMPU Block Diagram

The SMPU Top-Level Block Diagram shows the SMPU block.

As seen in the diagram, the SMPU sits between the memory port (SCB requester port) and the SCB fabric (SCB completer port). It acts as a gateway analyzing the transaction requests. It either rejects the transaction request or allows access based on the user-programmed configuration of the SMPU.

Figure 12-1: SMPU Top-Level Block Diagram

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000000_726c0667de8ae95ab0a56dfa8b84c53651dd6edf65fdc5bb815b93f0cb74cfd0.png)

## SMPU Architectural Concepts

The following sections provide brief descriptions of the architecture of the SMPU module.

## Default Setting

At reset, the default state of the system is fully open. The SMPUs admit any access to memory spaces.

## Latency

The SMPU adds latency to all the transactions to the memory except reads when read speculation is enabled ( SMPU\_CTL.RSDIS =0). In this case, read accesses are always forwarded to the memory and read responses are generated according to the SMPU settings. If read speculation is disabled ( SMPU\_CTL.RSDIS =1), reads are blocked if they cause a security or protection violation. The SMPU generates the SCB read response that corresponds to a blocked transaction.

If read speculation is enabled, the SMPU adds 1 clock cycle latency to the read transaction. If read speculation is disabled, the SMPU adds 2 clock cycles latency to the read transaction.

## SMPU Operating Modes

The SMPU does not have any strict modes of operation. However, it can be configured for region-based protection where a controller with a particular ID can be blocked or allowed based on settings in the SMPU\_RCTL[n] register.

Region-based protection is programmed with registers:

- SMPU\_RCTL[n]
- SMPU\_RADDR[n]
- SMPU\_RIDA[n]
- SMPU\_RIDMSKA[n]
- SMPU\_RIDB[n]
- SMPU\_RIDMSKB[n]

## SMPU Interrupt Signals

There is one interrupt signal associated with the SMPU. If interrupts are enabled, the SMPU\_STAT.IRQ bit is set. The SMPU\_IRQ signal is asserted when the SMPU detects a memory access violation. The target address triggering the interrupt is found in the SMPU\_IADDR register. The SMPU\_IDTLS register provides further details about the cause of the interrupt.

Write errors are prioritized over read errors.

Protection violations (an ID-based violation) can trigger the SMPU interrupt and can be enabled independently. The protection violation interrupt is enabled by setting the SMPU\_CTL.PINTEN bit.

The SMPU interrupt is asserted for any of the following conditions:

If a second memory access violation occurs while the SMPU\_STAT.IRQ bit is set, the SMPU\_STAT.IOVR (interrupt overrun) bit is set. The SMPU\_IADDR and the SMPU\_IDTLS registers are not updated until the SMPU\_STAT.IRQ bit is cleared. Any information on the subsequent interrupt is lost. Once the SMPU\_STAT.IRQ bit and the SMPU\_STAT.IOVR bit are cleared, any new memory access violations can trigger an interrupt and its details can be captured.

- NOTE: When a blocked access occurs, the SMPU triggers an interrupt when interrupt generation is enabled. The SMPU can also be configured to generate a bus error that propagates back to the system controller. The system controller can also trigger an interrupt due to this bus error.
- NOTE: On the processor, each SMPU instance has an interrupt. All of the SMPU interrupts are OR'ed and mapped to a single SMPU interrupt on the SEC. While servicing the SMPU interrupt, check all of the SMPU\_STAT registers to determine which triggered the interrupt. The interrupt service routine clears the SMPU\_STAT.IRQ bit of all of the SMPU\_STAT registers for which the interrupt is triggered.

## SMPU Status and Error Signals

If bus errors are enabled ( SMPU\_CTL.PBEDIS =0), the SMPU generates and returns a bus error to the controller initiating the blocked access. This bit also sets the SMPU\_STAT.BERR bit. The SMPU\_BADDR and SMPU\_BDTLS registers can be read to get the address and details of the transaction that caused the SMPU to generate the error.

Write errors are prioritized over read errors.

A bus error status is returned to the system controller if:

- an ID-based violation happened and the SMPU\_CTL.PBEDIS bit =0

If a second memory access violation occurs while the SMPU\_STAT.BERR bit is set, the SMPU\_STAT.BEOVR bit (bus error overrun) is set. The SMPU\_BADDR and the SMPU\_BDTLS registers are not updated until the SMPU\_STAT.IRQ bit is cleared. The information about the transaction that caused the SMPU\_STAT.BEOVR bit to be set is lost.

- NOTE: If both the protection violation interrupt is not enabled ( SMPU\_CTL.PINTEN =0) and the protection bus error is disabled ( SMPU\_CTL.PBEDIS =1), the SMPU blocks invalid transactions. However, it does not provide any status or interrupt information indicating that a transaction is blocked.

## ADSP-2184x SMPU Register Descriptions

The System Memory Protection Unit (SMPU) contains the following registers.

Table 12-5: ADSP-2184x SMPU Register List

| Name               | Description                               |
|--------------------|-------------------------------------------|
| SMPU_BADDR         | Bus Error Address Register                |
| SMPU_BDTLS         | Bus Error Details Register                |
| SMPU_CTL           | SMPU Control Register                     |
| SMPU_EXACADD[n]    | Exclusive Access IDn Address              |
| SMPU_EXACSTAT[n]   | Exclusive Access Status                   |
| SMPU_IADDR         | Interrupt Address Register                |
| SMPU_IDTLS         | Interrupt Details Register                |
| SMPU_RADDR[n]      | Region n Address Register                 |
| SMPU_RCTL[n]       | Region n Control Register                 |
| SMPU_REVID         | SMPU Revision ID Register                 |
| SMPU_RIDA[n]       | Region n ID A Register                    |
| SMPU_RIDB[n]       | Region n ID B Register                    |
| SMPU_RIDMSKA[n]    | Region n ID Mask A Register               |
| SMPU_RIDMSKB[n]    | Region n ID Mask B Register               |
| SMPU_SECURECTL     | SMPU Control Secure Accesses Register     |
| SMPU_SECURERCTL[n] | Region n Control Secure Accesses Register |
| SMPU_STAT          | SMPU Status Register                      |

## Bus Error Address Register

Programs read the SMPU\_BADDR and the SMPU\_BDTLS registers to determine the cause of a bus error. Write errors are prioritized over read errors.

Figure 12-2: SMPU\_BADDR Register Diagram

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000001_9295b34e75f3b802f9b2b63b5e223c28065424c5a85305257f4ed285bff2278d.png)

Table 12-6: SMPU\_BADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                               |
|--------------------|------------|-----------------------------------------------------------------------|
| 31:0               | VALUE      | Bus Error Address.                                                    |
| (R/NW)             |            | The SMPU_BADDR.VALUE bit field contains the address of the bus error. |

## Bus Error Details Register

The SMPU\_BDTLS register indicates the ID of the bus error transaction, whether the transaction that caused the last bus error was a read, a write, secure or non-secure.

Figure 12-3: SMPU\_BDTLS Register Diagram

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000002_c731cdd13fe05af00dde39ae0c903191c7dc27127716721ab36843fc88b6c3b6.png)

Table 12-7: SMPU\_BDTLS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                 | Description/Enumeration                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 20:8 (R/NW)        | ID         | ID of Transaction. The SMPU_BDTLS.ID bit field provides the ID of the transaction that caused the                                       | ID of Transaction. The SMPU_BDTLS.ID bit field provides the ID of the transaction that caused the                                       |
| 1 (R/NW)           | RNW        | Read/Write Status. The SMPU_BDTLS.RNW bit indicates whether the last transaction that caused the bad address error was a read or write. | Read/Write Status. The SMPU_BDTLS.RNW bit indicates whether the last transaction that caused the bad address error was a read or write. |
| 1 (R/NW)           | RNW        | 0                                                                                                                                       | Transaction that caused last bus error was a write                                                                                      |
| 1 (R/NW)           | RNW        | 1                                                                                                                                       | Transaction that caused last bus error was a read                                                                                       |
| 0 (R/NW)           | SECURE     | Secure Status Register. The SMPU_BDTLS.SECURE bit indicates whether the last transaction that caused                                    | Secure Status Register. The SMPU_BDTLS.SECURE bit indicates whether the last transaction that caused                                    |
| 0 (R/NW)           | SECURE     | 0                                                                                                                                       | Transaction that caused last bus error was non-secure                                                                                   |
| 0 (R/NW)           | SECURE     | 1                                                                                                                                       | Transaction that caused last bus error was secure                                                                                       |

## SMPU Control Register

The SMPU\_CTL register provides access to the locking control, error interrupts and SMPU violations.

Figure 12-4: SMPU\_CTL Register Diagram

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000003_51d91b46fc51571788b85a2dd5e248a77dd701e9df2b0288330c2f6bf083ba33.png)

Table 12-8: SMPU\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock Bit. When the SMPU_CTL.LOCK bit is set and the global lock signal is asserted from the SPU, the SMPU_CTL register is write-protected. Write-protection is disabled only when the global lock signal becomes deasserted again.                                                                                                                                                                     | Lock Bit. When the SMPU_CTL.LOCK bit is set and the global lock signal is asserted from the SPU, the SMPU_CTL register is write-protected. Write-protection is disabled only when the global lock signal becomes deasserted again.                                                                                                                                                                     |
| 31 (R/W)           | LOCK       | 0                                                                                                                                                                                                                                                                                                                                                                                                      | CTL Global Lock Disable. The SMPU_CTL register is not write-protected.                                                                                                                                                                                                                                                                                                                                 |
| 31 (R/W)           | LOCK       | 1                                                                                                                                                                                                                                                                                                                                                                                                      | CTL Global Lock Enable. The SMPU_CTL register is write-protected.                                                                                                                                                                                                                                                                                                                                      |
| 4 (R/W)            | RLOCK      | RCTLn, RADDRn, RIDxn and RIDMxn Registers Lock Bit. When the SMPU_CTL.RLOCK bit is set, all the registers associated with region-based control ( SMPU_RCTL[n] , SMPU_RADDR[n] , SMPU_RIDA[n] , SMPU_RIDB[n] , SMPU_RIDMSKA[n] and SMPU_RIDMSKB[n] ) are write-pro- tected when the global lock signal is active from the SPU. Write access is allowed again when the global lock signal is deasserted. | RCTLn, RADDRn, RIDxn and RIDMxn Registers Lock Bit. When the SMPU_CTL.RLOCK bit is set, all the registers associated with region-based control ( SMPU_RCTL[n] , SMPU_RADDR[n] , SMPU_RIDA[n] , SMPU_RIDB[n] , SMPU_RIDMSKA[n] and SMPU_RIDMSKB[n] ) are write-pro- tected when the global lock signal is active from the SPU. Write access is allowed again when the global lock signal is deasserted. |
| 4 (R/W)            | RLOCK      | 0                                                                                                                                                                                                                                                                                                                                                                                                      | Region Registers Write-Protect Enable. All region regis- ters are not write-protected.                                                                                                                                                                                                                                                                                                                 |
| 4 (R/W)            | RLOCK      | 1                                                                                                                                                                                                                                                                                                                                                                                                      | Region Registers Write-Protect Disable. All region regis- ters are write-protected.                                                                                                                                                                                                                                                                                                                    |

Table 12-8: SMPU\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | PINTEN     | Protection Violation Interrupt Enable. The SMPU_CTL.PINTEN bit controls whether or not an interrupt is generated when a protection violation occurs.                           |
| 2 (R/W)            | PBETYPE    | Protection Violation Bus Error Type. The SMPU_CTL.PBETYPE bit controls whether a protection violation produces a decode error or a completer error.                            |
| 1                  | PBEDIS     | which violate the configured protection. Protection Violation Bus Error Disable. If set, the SMPU_CTL.PBEDIS bit blocks protection violations, but does not cause a bus error. |
| (R/W)              |            | 0 Bus Error Generation Enable. Transactions which vio-                                                                                                                         |
| 0 (R/W)            | RSDIS      | Read Speculation Disable. The SMPU_CTL.RSDIS bit controls whether or not the read addresses are checked before being sent to the completer.                                    |
|                    |            | 0 Read Speculation Enable. Read addresses are sent to completer without checking.                                                                                              |
|                    |            | the 1 Read Speculation Disable. Read addresses are checked                                                                                                                     |

## Exclusive Access IDn Address

The SMPU\_EXACADD[n] register provides the address ID of an exclusive access.

Figure 12-5: SMPU\_EXACADD[n] Register Diagram

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000004_fe72aea451c7c62bed531c5cd5831cc910433a8badccfef8d052bdc5270a5776.png)

Table 12-9: SMPU\_EXACADD[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration       |
|--------------------|------------|-------------------------------|
| 31:0               | VALUE      | Exclusive Access IDn Address. |
| (R/NW)             |            |                               |

## Exclusive Access Status

The SMPU\_EXACSTAT[n] register provides the exclusive access ID, read size and read length as well as the indication that the access was valid.

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000005_559593e8b7012e547874670c3bfd0bafb191f4e8289ed3e0d4ee2508ed90dc5e.png)

Exclusive Access ID

Figure 12-6: SMPU\_EXACSTAT[n] Register Diagram

Table 12-10: SMPU\_EXACSTAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration       |
|--------------------|------------|-------------------------------|
| 20:8 (R/NW)        | ARID       | Exclusive Access ID.          |
| 7:5 (R/NW)         | ARSIZE     | Exclusive Access Read Size.   |
| 4:1 (R/NW)         | ARLEN      | Exclusive Access Read Length. |
| 0 (R/NW)           | VALID      | Valid Exclusive Access Read.  |

## Interrupt Address Register

The SMPU\_IADDR register indicates an attempt to make a read or write access to unimplemented addresses or accesses are non-aligned. The SMPU issues a bus error for this condition.

Figure 12-7: SMPU\_IADDR Register Diagram

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000006_3690ab565cf1e65d5cb163c39c469617e32a17c9d6cc900701dfcdc4969e146f.png)

Table 12-11: SMPU\_IADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | Interrupt Address. The SMPU_IADDR.VALUE bit field is the address where an attempt to access an |
| (R/NW)             |            | unimplemented address or a non-aligned access has occurred.                                    |

## Interrupt Details Register

The SMPU\_IDTLS register provides the ID of the last signaled interrupt, whether the interrupt was caused by a read or write, and whether the transaction that caused the last signaled interrupt was secure.

Figure 12-8: SMPU\_IDTLS Register Diagram

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000007_10120fd4bbec591700e805b862f21b9fe0f97cc94af77c624f4f84b52bbb89d1.png)

Table 12-12: SMPU\_IDTLS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                             | Description/Enumeration                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 20:8 (R/NW)        | ID         | ID of Transaction. The SMPU_IDTLS.ID bit field provides the ID of the transaction that caused the interrupt.                        | ID of Transaction. The SMPU_IDTLS.ID bit field provides the ID of the transaction that caused the interrupt.                        |
| 1 (R/NW)           | RNW        | Read/Write Status. The SMPU_IDTLS.RNW bit indicates whether the last transaction that caused the interrupt was a read or write.     | Read/Write Status. The SMPU_IDTLS.RNW bit indicates whether the last transaction that caused the interrupt was a read or write.     |
|                    |            | 0                                                                                                                                   | Transaction that caused last signaled interrupt was a write                                                                         |
|                    |            | 1                                                                                                                                   | Transaction that caused last signaled interrupt was a read                                                                          |
| 0 (R/NW)           | SECURE     | Secure Status. The SMPU_IDTLS.SECURE bit indicates whether the last transaction that caused the interrupt was secure or non-secure. | Secure Status. The SMPU_IDTLS.SECURE bit indicates whether the last transaction that caused the interrupt was secure or non-secure. |
|                    |            | 0                                                                                                                                   | Transaction that caused last signaled interrupt was non- secure                                                                     |
|                    |            | 1                                                                                                                                   | Transaction that caused last signaled interrupt was se- cure                                                                        |

## Region n Address Register

The SMPU\_RADDR[n] register is used to define the base address for a memory region to be protected.

Figure 12-9: SMPU\_RADDR[n] Register Diagram

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000008_1f3f627e169da96b203bb73ad9cd7f5bdf87794a95db11c8512f49c2b6f890af.png)

Table 12-13: SMPU\_RADDR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------|
| 31:12              | BADDR      | Region n Base Address 20 Most Significant Bits. The SMPU_RADDR[n].BADDR bit field defines the base address for a |
| (R/W)              |            | memory region to be protected.                                                                                   |

## Region n Control Register

The SMPU\_RCTL[n] register is used to define the level of protection for a region of memory. The protection of a region is controlled and defined by this register and the SMPU\_RADDR[n] , SMPU\_RIDA[n] , SMPU\_RIDB[n] , SMPU\_RIDMSKA[n] , and SMPU\_RIDMSKB[n] registers.

Figure 12-10: SMPU\_RCTL[n] Register Diagram

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000009_04b9d484ef7a74f695301d674e372d433131902326e3014f6daa14e0251262b4.png)

Table 12-14: SMPU\_RCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                               | Description/Enumeration                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | WIDCINV    | Write Transaction ID Compare Invert. The SMPU_RCTL[n].WIDCINV bit inverts the write ID match result.                  | Write Transaction ID Compare Invert. The SMPU_RCTL[n].WIDCINV bit inverts the write ID match result.                  |
|                    |            | 0                                                                                                                     | Write transaction ID comparison result not inverted                                                                   |
|                    |            | 1                                                                                                                     | Write transaction ID comparison result inverted                                                                       |
| 10 (R/W)           | WPROTEN    | Write Transaction Protection Enable. The SMPU_RCTL[n].WPROTEN bit enables protection against ID-based write           | Write Transaction Protection Enable. The SMPU_RCTL[n].WPROTEN bit enables protection against ID-based write           |
|                    |            | 0                                                                                                                     | Write transaction ID-based protection disabled                                                                        |
|                    |            | 1                                                                                                                     | Write transaction ID-based protection enabled                                                                         |
| 9 (R/W)            | RIDCINV    | Read Transaction ID Compare Invert. When the SMPU_RCTL[n].RIDCINV bit is set, the read ID match result is invert- ed. | Read Transaction ID Compare Invert. When the SMPU_RCTL[n].RIDCINV bit is set, the read ID match result is invert- ed. |
|                    |            | 0                                                                                                                     | Read transaction ID comparison result not inverted                                                                    |
|                    |            | 1                                                                                                                     | Read transaction ID comparison result inverted                                                                        |

Table 12-14: SMPU\_RCTL[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | RPROTEN    | Read Transaction Protection Enable. The SMPU_RCTL[n].RPROTEN bit enable bit to turn on protection against ID- based read transactions for the memory region. |
|                    |            | 0 Read transaction ID-based protection disabled                                                                                                              |
|                    |            | 1 Read transaction ID-based protection enabled                                                                                                               |
| 5:1 (R/W)          | SIZE       | Memory Region Size. The SMPU_RCTL[n].SIZE bit defines the size of the memory region to be protect- ed.                                                       |
|                    |            | 0 4 KB                                                                                                                                                       |
|                    |            | 1 8 KB                                                                                                                                                       |
|                    |            | 2 16 KB                                                                                                                                                      |
|                    |            | 3 32 KB                                                                                                                                                      |
|                    |            | 4 64 KB                                                                                                                                                      |
|                    |            | 5 128 KB                                                                                                                                                     |
|                    |            | 6 256 KB                                                                                                                                                     |
|                    |            | 7 512 KB                                                                                                                                                     |
|                    |            | 8 1MB                                                                                                                                                        |
|                    |            | 9 2MB                                                                                                                                                        |
|                    |            | 10 4MB                                                                                                                                                       |
|                    |            | 11 8MB                                                                                                                                                       |
|                    |            | 12 16MB                                                                                                                                                      |
|                    |            | 13 32MB                                                                                                                                                      |
|                    |            | 14 64MB                                                                                                                                                      |
|                    |            | 15 128MB                                                                                                                                                     |
|                    |            | 16 256MB                                                                                                                                                     |
|                    |            | 17 512MB                                                                                                                                                     |
|                    |            | 18 1 GB                                                                                                                                                      |
|                    |            | 19 2 GB                                                                                                                                                      |
|                    |            | 20 4 GB                                                                                                                                                      |
|                    |            | 21-31 Reserved                                                                                                                                               |

Table 12-14: SMPU\_RCTL[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                    |
|--------------------|------------|----------------------------------------------------------------------------|
| 0 (R/W)            | EN         | Region Enable. The SMPU_RCTL[n].EN bit enables the protection of a region. |
| 0 (R/W)            | EN         | 0 Disabled                                                                 |
| 0 (R/W)            | EN         | 1 Enabled                                                                  |

## SMPU Revision ID Register

The SMPU\_REVID register provides the major and minor revision numbers of this module.

Figure 12-11: SMPU\_REVID Register Diagram

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000010_32ccc8d508cdacbf5b9d7e4970e325c69dc0c35ed419fb1a522690d6bdb22bef.png)

Table 12-15: SMPU\_REVID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:4                | MAJOR      | Major Version ID.         |
| 3:0 (R/NW)         | REV        | Incremental Version ID.   |

## Region n ID A Register

The SMPU\_RIDA[n] register is used for ID comparison 'A'. This comparison is performed after a mask is applied to both the transaction ID (from either the read or write IDs) and the register value. An ID match means that the ID is the exception to the rule and the read or write is allowed even if the region is read or write-protected. For more detail, refer to the ID Comparison section.

Figure 12-12: SMPU\_RIDA[n] Register Diagram

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000011_791c5ad188a2707133680d342c9bb79e5e25e68e7081fea0a1bc37be928f0c7a.png)

Table 12-16: SMPU\_RIDA[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:0 (R/W)         | ID         | Region n ID Register A. The SMPU_RIDA[n].ID bit field, combined with the mask provides the means to bypass the configured memory protection for a region. |

## Region n ID B Register

The SMPU\_RIDB[n] register is used for ID comparison 'B'. This comparison is performed after a mask is applied to both the transaction ID (from either the read or write IDs) and the register value. An ID match means that the ID is the exception to the rule and the read or write is allowed even if the region is read or write-protected. For more details, refer to the ID Comparison section.

Figure 12-13: SMPU\_RIDB[n] Register Diagram

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000012_0d42c0b5c39519d182e9dcd009131e965a21796ea812869b8f8a2b9cb6f00ced.png)

Table 12-17: SMPU\_RIDB[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:0 (R/W)         | ID         | Region n ID Register B. The SMPU_RIDB[n].ID bit field, combined with the mask provides the means to bypass the configured memory protection for a region. |

## Region n ID Mask A Register

The SMPU\_RIDMSKA[n] register is used for ID comparison 'A'. The mask allows or disallows certain IDs from affecting the final result of the ID match. For more details, refer to the ID Comparison section.

Figure 12-14: SMPU\_RIDMSKA[n] Register Diagram

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000013_5cb58f53e49b9cd6e42234708436ad18aea6845cd1570ff59f4b465d8c74c0d3.png)

Table 12-18: SMPU\_RIDMSKA[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:0 (R/W)         | MSK        | Region n ID Mask Register A. The SMPU_RIDMSKA[n].MSK bit field, combined with the incoming transaction, provides the means to bypass the configured memory protection for a region. |

## Region n ID Mask B Register

The SMPU\_RIDMSKB[n] register is used for ID comparison 'B'. The mask allows or disallows certain IDs from affecting the final result of the ID match. For more details, refer to the ID Comparison section.

Figure 12-15: SMPU\_RIDMSKB[n] Register Diagram

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000014_81bd37ae35f9a1530e143fc506995068a0ca8d2b0b65b2290b3663bb2bff075b.png)

Table 12-19: SMPU\_RIDMSKB[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:0               | MSK        | Region n ID Mask Register B.                                                                                                                          |
| (R/W)              |            | The SMPU_RIDMSKB[n].MSK bit field, combined with the incoming transaction provides the means to bypass the configured memory protection for a region. |

## SMPU Control Secure Accesses Register

The SMPU\_SECURECTL register provides the bits required to set up the security settings for the processor. These settings includes error generation and read/write security.

Figure 12-16: SMPU\_SECURECTL Register Diagram

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000015_8aba55b8c6928840513808b712cb2d88e1393d1844f83406d51ebc52be07a34d.png)

Table 12-20: SMPU\_SECURECTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock Bit. When the SMPU_SECURECTL.LOCK bit is set and the global lock signal is asserted from the SPU, the SMPU_SECURECTL register is write-protected. Write-protection is disabled only when the global lock signal becomes deasserted again. |
| 11 (R/W)           | WSECDIS    | Secure Write Transaction Disable. The SMPU_SECURECTL.WSECDIS bit disables secure write transactions.                                                                                                                                           |
| 10 (R/W)           | WNSEN      | Non-secure Write Transaction Enable. The SMPU_SECURECTL.WNSEN bit enables non-secure write transactions. 0 Disable non-secure writes 1 Enable non-secure writes                                                                                |

Table 12-20: SMPU\_SECURECTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | RSECDIS    | Secure Read Transaction Disable. The SMPU_SECURECTL.RSECDIS bit disables secure read transactions.                                                                                                                                                                                      |
| 8 (R/W)            | RNSEN      | Non-secure Read Transaction Enable. The SMPU_SECURECTL.RNSEN bit enables non-secure read transactions. 0 Disable non-secure read transactions                                                                                                                                           |
| 3 (R/W)            | RLOCK      | Secure Region Registers Lock Bit. When the SMPU_SECURECTL.RLOCK bit is set, the secure region control registers, SMPU_SECURERCTL[n] , are write-protected when the global lock signal is active from the SPU. When the global lock signal is deasserted, write access is allowed again. |
| 2 (R/W)            | SINTEN     | 1 Enable write-protection on secure region registers Security Violation Interrupt Enable. The SMPU_SECURECTL.SINTEN bit enables interrupt generation when a security violation occurs. 0 Disable security settings violation interrupt                                                  |
| 1 (R/W)            |            | 1 Enable security settings violation interrupt                                                                                                                                                                                                                                          |
| 0                  | SBETYPE    | Security Violation Bus Error Type. The SMPU_SECURECTL.SBETYPE bit controls whether a decode error or a com- pleter error is returned when a security violation occurs. 0 Decode error 1 Completer error                                                                                 |
| (R/W)              | SBEDIS     | Security Violation Bus Error Disable. The SMPU_SECURECTL.SBEDIS bit controls whether or not a bus error is caused when a security violation occurs. 0 Enable bus error                                                                                                                  |

## Region n Control Secure Accesses Register

The SMPU\_SECURERCTL[n] register contains bits that configure read/write security for a specific region.

Figure 12-17: SMPU\_SECURERCTL[n] Register Diagram

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000016_12eb1a3bf896c8b6c2be446c10f0b62b3d40c3f4a8e91bdbf523ffff76e4eec9.png)

Table 12-21: SMPU\_SECURERCTL[n] Register Fields

| Bit No. (Access)   | Bit Name                  | Description/Enumeration                                                                                                                                            |
|--------------------|---------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | WSECDIS                   | Secure Write Transaction Disable. The SMPU_SECURERCTL[n].WSECDIS bit disables secure write transactions for the memory region.                                     |
| 2 (R/W)            | WNSEN                     | Non-secure Write Transaction Enable. This SMPU_SECURERCTL[n].WNSEN bit enables non-secure write transactions for the memory region.                                |
| 1 (R/W)            | RSECDIS                   | 1 Enable non-secure write transactions to this region Secure Read Transaction Disable. The SMPU_SECURERCTL[n].RSECDIS bit disables secure read transactions for    |
|                    |                           | the memory region. 0 Enable secure read transactions to this region 1 Disable secure read transactions to this region                                              |
| 0 (R/W)            | RNSEN Non-secure Read The | Transaction Enable. SMPU_SECURERCTL[n].RNSEN bit enables non-secure read transactions for the memory region. 0 Disable non-secure read transactions to this region |

## SMPU Status Register

The SMPU\_STAT register provides the state of the SMPU and indicates various errors. All bits in this register are write 1 to clear.

Figure 12-18: SMPU\_STAT Register Diagram

![Image](15_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000017_89fc5d675b4fb81a6147040cf7d5adf7b7c9bffa63342d89aaa432d807c51100.png)

Table 12-22: SMPU\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W1C)         | LWERR      | Lock Write Error. The SMPU_STAT.LWERR bit is set when SMPU_CTL.LOCK bit =1, the global lock signal is asserted from the SPU and a read or write attempt was made to the SMPU_CTL MMR.                                            |
| 16 (R/W1C)         | ADRERR     | Address Error. The SMPU_STAT.ADRERR bit is set when the SMPU MMRis accessed as an unaligned address, or when a read-only MMRis written to.                                                                                       |
| 3 (R/W1C)          | BEOVR      | 1 Address Error Bus Error Overrun. The SMPU_STAT.BEOVR bit indicates that another bus error had occurred. Any new information about the most recent violation which caused the bus error is not captured. 0 No Bus Error overrun |
| 3 (R/W1C)          | 1          | Bus Error overrun has occurred                                                                                                                                                                                                   |
| 3 (R/W1C)          |            |                                                                                                                                                                                                                                  |

Table 12-22: SMPU\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                       | Description/Enumeration                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W1C)          | BERR       | Bus Error. This SMPU_STAT.BERR bit indicates if a bus error was generated.                                                                                                                                                    | Bus Error. This SMPU_STAT.BERR bit indicates if a bus error was generated.                                                                                                                                                    |
| 2 (R/W1C)          | BERR       | 0                                                                                                                                                                                                                             | No Bus Error since this bit has been cleared                                                                                                                                                                                  |
| 2 (R/W1C)          | BERR       | 1                                                                                                                                                                                                                             | Bus Error has been generated                                                                                                                                                                                                  |
| 1 (R/W1C)          | IOVR       | Interrupt Overrun. The SMPU_STAT.IOVR bit indicates if another violation occurred while the previ- ous violation interrupt was not finished being serviced. Information about the most recent violation is then not captured. | Interrupt Overrun. The SMPU_STAT.IOVR bit indicates if another violation occurred while the previ- ous violation interrupt was not finished being serviced. Information about the most recent violation is then not captured. |
| 1 (R/W1C)          | IOVR       | 0                                                                                                                                                                                                                             | No Interrupt overrun                                                                                                                                                                                                          |
| 1 (R/W1C)          | IOVR       | 1                                                                                                                                                                                                                             | Interrupt overrun has occurred                                                                                                                                                                                                |
| 0 (R/W1C)          | IRQ        | Interrupt Request. The SMPU_STAT.IRQ bit provides an indication that an interrupt has been generat- ed.                                                                                                                       | Interrupt Request. The SMPU_STAT.IRQ bit provides an indication that an interrupt has been generat- ed.                                                                                                                       |
| 0 (R/W1C)          | IRQ        | 0                                                                                                                                                                                                                             | No Interrupt since this bit has been cleared                                                                                                                                                                                  |
| 0 (R/W1C)          | IRQ        | 1                                                                                                                                                                                                                             | Interrupt has been generated                                                                                                                                                                                                  |