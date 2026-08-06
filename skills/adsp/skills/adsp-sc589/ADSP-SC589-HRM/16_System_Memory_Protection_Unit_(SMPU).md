## 13   System Memory Protection Unit (SMPU)

The SMPU provides a flexible way of protecting memory regions against read or write access from any or all masters in the system. In addition, it can guard against memory access depending on security privileges of the system master.

## SMPU Features

The system memory protection unit has the following features.

- After reset, the default state of the system is fully open. The SMPUs admit any access to memory spaces by any master.
- Each SMPU instance can be configured to monitor multiple regions. Each can be individually enabled.
- Each region can be configured with its own protection settings.
- Provides general read or write protection.
- Read and write transactions are restricted or allowed depending on the transaction ID.

On the ADSP-SC58x, ten SMPU instances are available to protect the L2, external memory (DMC/SMC), and memory mapped I/O (PCIe) interfaces. Six instances are allotted to protect the L2 memory, two instances for DMC0 and DMC1, one instance for SMC and one instance for PCIe as shown below:

Table 13-1: SMPU Instances

| Module                 |   SMPU Instance |
|------------------------|-----------------|
| SMC                    |               0 |
| Core_L2_RAM_Boot_ROM0  |               2 |
| DMA_L2_RAM_Boot_ROM0   |               3 |
| Core_L2_ROM1_Boot_ROM1 |               4 |
| DMA_L2_ROM1_Boot_ROM1  |               5 |
| Core_L2_ROM2_Boot_ROM2 |               6 |
| DMA_L2_ROM2_Boot_ROM2  |               7 |
| PCIe                   |               8 |

Table 13-1: SMPU Instances (Continued)

| Module   |   SMPU Instance |
|----------|-----------------|
| DMC0     |               9 |
| DMC1     |              10 |

All the SMPU instances can be configured up to eight regions.

- Up to eight outstanding read or write transactions supported on SMPU instances for Core\_L2\_RAM\_Boot\_ROM0, DMC0 and DMC1.
- Up to four outstanding read or write transactions supported on the SMPU instances for DMA\_L2\_RAM\_Boot\_ROM0, Core\_L2\_ROM1\_Boot\_ROM1, DMA\_L2\_ROM1\_Boot\_ROM1, Core\_L2\_ROM2\_Boot\_ROM2, DMA\_L2\_ROM2\_Boot\_ROM2, SMC, and PCIe.
- Exclusive access enabled by hardware for the SMPU instances of Core\_L2\_RAM\_Boot\_ROM0, DMC 0, DMC1, and SMC. Up to four exclusive-access-capable masters can be supported.

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

NOTE: The debugger typically replaces an instruction with a breakpoint instruction for software breakpoints. If a memory region is protected against write accesses, software breakpoints are not possible unless the SMPU is configured with the appropriate system master ID of the debugger. The configuration allows it to perform a write-access.

In the case where memory is used for data, a read access or cache fill is not possible if the memory is blocked from read accesses. If read accesses are allowed but write accesses are disallowed, then there is an issue with coherency. The cache is filled but when the cache is updated and must be written back to the SMPU protected memory, the writeaccess is blocked.

In general, practice caution when using both the SMPU and cache.

## Speculative Reads

If speculative reads are enabled ( SMPU\_CTL.RSDIS =0), the SMPU forwards the read transaction directly to the memory before checking the protection setting corresponding to the addressed memory region. This functionality saves one clock cycle in the clock domain of the SMPU. The SMPU checks the protection setting while the read transaction occurs with the memory. If the protection setting dictates that the target memory address is blocked, the SMPU blocks the read to the master.

If speculative reads are disabled ( SMPU\_CTL.RSDIS =1), the SMPU checks the protection settings first and forwards the transaction to memory only if it passes the configured protection settings. This functionality incurs a onecycle latency per read.

NOTE: Reads affect certain memory operations such as automatic clearing of the memory (that is, FIFOs). When the SMPU protects this type of memory, disable read speculation since the blocking can occur without the read transaction reaching the target memory.

## ADSP-SC58x SMPU Register List

The System Memory Protection Unit (SMPU) provides selective protection of the processor's memory resources. The SMPU includes a set of processor events that can be monitored during program execution. A set of registers governs SMPU operations. For more information on SMPU functionality, see the SMPU register descriptions.

Table 13-2: ADSP-SC58x SMPU Register List

| Name             | Description                  |
|------------------|------------------------------|
| SMPU_BADDR       | Bus Error Address Register   |
| SMPU_BDTLS       | Bus Error Details Register   |
| SMPU_CTL         | SMPU Control Register        |
| SMPU_EXACADD[n]  | Exclusive Access IDn Address |
| SMPU_EXACSTAT[n] | Exclusive Access Status      |
| SMPU_IADDR       | Interrupt Address Register   |

Table 13-2: ADSP-SC58x SMPU Register List (Continued)

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

## ADSP-SC58x SMPU Interrupts

The SMPU has one interrupt with the SEC ID = 242. See the System Event Controller (SEC) and Generic Interrupt Controller (GIC) chapter for complete information on interrupt generation and use.

## Memory Writes

A write transaction to address n is prevented when the following is true.

Address n is in memory region m and memory region m is write-protected ( SMPU\_RCTL[n].WPROTEN =1) and ID is not a match. (See ID Comparison). The block occurs because the memory region is configured for write-protection and the ID comparison does not result in a match. If an ID comparison results in a match, the write transaction is allowed through.

## Memory Reads

A read transaction from address n is prevented when the following is true:

- Address n is in memory region and memory region m is read-protected ( SMPU\_RCTL[n].RPROTEN =1) and
- ID is not a match

The block occurs because the memory region is configured for read-protection and the ID comparison does not result in a match. If the ID comparison results in a match, the read transaction is permitted. (See ID Comparison).

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

Read IDA match = ((ID of incoming read transaction AND SMPU\_RIDMSKA[n].MSK ) == ( SMPU\_RIDA[n].ID AND SMPU\_RIDMSKA[n].MSK )) Read IDB match = ((ID of incoming read transaction AND SMPU\_RIDMSKB[n].MSK ) == ( SMPU\_RIDB[n].ID AND SMPU\_RIDMSKB[n].MSK )) Read ID match = (Read IDA match OR Read IDB match) XOR SMPU\_RCTL[n].RIDCINV

In the two cases described above, the incoming transaction (either write or read) ID is AND'ed with the configured mask value in protection unit A. It is then compared to the value of the configured ID value which is also AND'ed with the configured mask value in protection unit A. The mask provides a method to allow a group of IDs to match. This process is also performed for protection unit B. The two outcomes (from A and B) are then OR'ed together.

Depending on the setting of the SMPU\_RCTL[n].RIDCINV or the SMPU\_RCTL[n].WIDCINV bits, the ID match comparison is inverted or not. The final result after applying the inversion, SMPU\_RCTL[n].RIDCINV , or SMPU\_RCTL[n].WIDCINV , determines whether the transaction bypasses the protection.

## Usage

The masks, SMPU\_RIDMSKA[n] and SMPU\_RIDMSKB[n] , are AND'ed with both the incoming transaction ID and the configured ID in SMPU\_RIDA[n].ID and SMPU\_RIDB[n].ID , respectively. By default the masks are zero. If ID-based region protection is enabled by setting the SMPU\_RCTL[n].WPROTEN or SMPU\_RCTL[n].RPROTEN bit fields and the masks are not set, the ID comparison essentially compares zeros. The comparison allows all transactions to bypass (if the region-based security setting is also configured in a way to allow transactions to go through for the region). To have the ID-based region protection to function, the mask registers and ID registers must also be set.

## System IDs

The System Master IDs table provides the IDs for the system masters. An x means that the bit can be a 0 or a 1. There are multiple IDs associated with that particular system master.

Table 13-3: System Master IDs

| Master Name   |   SCB Input Switch |   Master ID SCB Input | ID                |
|---------------|--------------------|-----------------------|-------------------|
| SPORT0_A      |                  0 |                     0 | 13'b00000000x0000 |
| SPORT0_B      |                  0 |                     1 | 13'b00000000x0001 |
| SPORT1_A      |                  0 |                     2 | 13'b00000000x0010 |
| SPORT1_B      |                  0 |                     3 | 13'b00000000x0011 |
| SPORT2_A      |                  0 |                     4 | 13'b00000000x0100 |
| SPORT2_B      |                  0 |                     5 | 13'b00000000x0101 |
| SPORT3_A      |                  0 |                     6 | 13'b00000000x0110 |
| SPORT3_B      |                  0 |                     7 | 13'b00000000x0111 |
| SPORT4_A      |                  1 |                     0 | 13'b00010000x0000 |
| SPORT4_B      |                  1 |                     1 | 13'b00010000x0001 |
| SPORT5_A      |                  1 |                     2 | 13'b00010000x0010 |
| SPORT5_B      |                  1 |                     3 | 13'b00010000x0011 |
| SPORT6_A      |                  1 |                     4 | 13'b00010000x0100 |
| SPORT6_B      |                  1 |                     5 | 13'b00010000x0101 |
| SPORT7_A      |                  1 |                     6 | 13'b00010000x0110 |
| SPORT7_B      |                  1 |                     7 | 13'b00010000x0111 |
| CRC0_CH0      |                  0 |                     8 | 13'b00000000x1000 |

Table 13-3: System Master IDs (Continued)

| Master Name   |   SCB Input Switch |   Master ID SCB Input | ID                |
|---------------|--------------------|-----------------------|-------------------|
| CRC0_CH1      |                  0 |                     9 | 13'b00000000x1001 |
| CRC1_CH0      |                  1 |                     8 | 13'b00010000x1000 |
| CRC1_CH1      |                  1 |                     9 | 13'b00010000x1001 |
| EMAC0         |                  2 |                     0 | 13'b00100xxxx0000 |
| MLB           |                  3 |                     0 | 13'b0011000000000 |
| UART0_TX      |                  2 |                     1 | 13'b00100000x0001 |
| UART0_RX      |                  2 |                     2 | 13'b00100000x0010 |
| SDIO          |                  2 |                     3 | 13'b0010000000011 |
| SINC          |                  2 |                     4 | 13'b00100000x0100 |
| SPI0TX        |                  4 |                     0 | 13'b01000000x0000 |
| SPI0RX        |                  4 |                     1 | 13'b01000000x0001 |
| SPI1TX        |                  4 |                     2 | 13'b01000000x0010 |
| SPI1RX        |                  4 |                     3 | 13'b01000000x0011 |
| SPI2TX        |                  4 |                     4 | 13'b01000000x0100 |
| SPI2RX        |                  4 |                     5 | 13'b01000000x0101 |
| PPI_F0        |                  4 |                     6 | 13'b01000000x0110 |
| PPI_F1        |                  4 |                     7 | 13'b01000000x0111 |
| LP0           |                  3 |                     1 | 13'b00110000x0001 |
| HAE_IN0       |                  5 |                     0 | 13'b01010000x0000 |
| HAE_IN1       |                  5 |                     1 | 13'b01010000x0001 |
| HAE_OUT       |                  5 |                     2 | 13'b01010000x0010 |
| USB0          |                  2 |                     5 | 13'b0010000000101 |
| UART1_TX      |                  5 |                     3 | 13'b01010000x0011 |
| UART1_RX      |                  5 |                     4 | 13'b01010000x0100 |
| EMAC1         |                  5 |                     5 | 13'b01010xxxx0101 |
| LP1           |                  3 |                     2 | 13'b00110000x0010 |
| USB1          |                  6 |                     0 | 13'b0110000000000 |
| CRYPTO        |                  6 |                     3 | 13'b0110000000011 |
| UART2_TX      |                  6 |                     4 | 13'b01100000x0100 |
| UART2_RX      |                  6 |                     5 | 13'b01100000x0101 |
| FIR_CH0       |                  5 |                     6 | 13'b0101000000110 |

Table 13-3: System Master IDs (Continued)

| Master Name                       |   SCB Input Switch |   Master ID SCB Input | ID                |
|-----------------------------------|--------------------|-----------------------|-------------------|
| FIR_CH1                           |                  5 |                     7 | 13'b0101000000111 |
| IIR_CH0                           |                  6 |                     6 | 13'b0110000000110 |
| IIR_CH1                           |                  6 |                     7 | 13'b0110000000111 |
| EMDMA0_CH0                        |                  7 |                     0 | 13'b0111000000000 |
| EMDMA0_CH1                        |                  7 |                     1 | 13'b0111000000001 |
| EMDMA1_CH0                        |                  7 |                     2 | 13'b0111000000010 |
| EMDMA1_CH1                        |                  7 |                     3 | 13'b0111000000011 |
| STD_BW_MDMA_SRC_CH                |                  8 |                     0 | 13'b10000000x0000 |
| STD_BW_MDMA_DST_CH                |                  8 |                     1 | 13'b10000000x0001 |
| PCIE_M                            |                  8 |                     2 | 13'b1000xxxxx0010 |
| ENH_BW_MDMA_SRC_CH                |                  9 |                     0 | 13'b10010000x0000 |
| ENH_BW_MDMA_DST_CH                |                  9 |                     1 | 13'b10010000x0001 |
| FFT_CH0                           |                  9 |                     2 | 13'b10010000x0010 |
| FFT_CH1                           |                  9 |                     3 | 13'b10010000x0011 |
| SH0_DPORT                         |                 10 |                     0 | 13'b1010000000000 |
| SH0_IPORT                         |                 10 |                     1 | 13'b1010000000001 |
| SH1_DPORT                         |                 10 |                     2 | 13'b1010000000010 |
| SH1_IPORT                         |                 10 |                     3 | 13'b1010000000011 |
| PL310_M0 = L2 cache Master Port 0 |                 10 |                     5 | 13'b1010xxxxx0101 |
| PL310_M0 = L2 cache Master Port 1 |                 10 |                     4 | 13'b1010xxxxx0100 |
| DBG                               |                  3 |                     3 | 13'b0011000000011 |
| ETR                               |                  3 |                     4 | 13'b0011000000100 |

## Memory Region

Memory regions can start at address 0x00000000 or at any address that is a multiple of its size. The Supported Memory Region Size and Alignment table shows the memory region sizes that the processor supports and the alignment of the memory region. (X values are do-not-care).

SMPU0 and SMPU2 support a maximum of four regions. SMPU1 supports a maximum of eight regions.

Table 13-4: Supported Memory Region Size and Alignment

| Size   | SMPU_RCTLn.SIZE   | Address    | Possible Values forN   |
|--------|-------------------|------------|------------------------|
|        | 0b00000           | 0xXXXXX000 | -                      |

Table 13-4: Supported Memory Region Size and Alignment (Continued)

| Size   | SMPU_RCTLn.SIZE   | Address    | Possible Values forN              |
|--------|-------------------|------------|-----------------------------------|
| 4KB    |                   |            |                                   |
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

NOTE: If a memory region address is not aligned to its size, the memory region start address protected by the SMPU is the configured address with the corresponding least significant bits masked. For example, if the size is configured for 16 KB ( SMPU\_RCTL[n].SIZE =0b00010), and the base address is configured for SMPU\_RADDR[n].BADDR =0x00005018, the actual base address used by the SMPU is 0x00004000. When SMPU\_RADDR[n].BADDR is read back, the program reads 0x00005000. This functionality is because only bits [11:0] are reserved as 0's. Programs must use care when setting the base address as it is not always the true base address.

## SMPU Definitions

To make the best use of the SMPU, it is useful to understand the terms in this section.

## Global Protection

Guarding of the entire memory space for the particular SMPU instantiation.

## Region-Based Protection

Guarding individual segments of memory inside the memory space for the particular SMPU instantiation.

## ID Match

A successful comparison of the ID associated with the incoming transaction and the ID and MASK configured in the SMPU.

## SMPU Block Diagram

The SMPU Top-Level Block Diagram shows the SMPU block.

As seen in the diagram, the SMPU sits between the memory port (SCB master port) and the SCB fabric (SCB slave port). It acts as a gateway analyzing the transaction requests. It either rejects the transaction request or allows access based on the user-programmed configuration of the SMPU.

Figure 13-1: SMPU Top-Level Block Diagram

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000000_738a91049cdaec27cfa0062b1f3c917ca74f4e4f815223291fa738add6bcf550.png)

## SMPU Architectural Concepts

The following sections provide brief descriptions of the architecture of the SMPU module.

## Default Setting

At reset, the default state of the system is fully open. The SMPUs admit any access to memory spaces.

## Latency

The SMPU adds latency to all the transactions to the memory except reads when read speculation is enabled ( SMPU\_CTL.RSDIS =0). In this case, read accesses are always forwarded to the memory and read responses are generated according to the SMPU settings. If read speculation is disabled ( SMPU\_CTL.RSDIS =1), reads are blocked if they cause a security or protection violation. The SMPU generates the SCB read response that corresponds to a blocked transaction.

If read speculation is enabled, the SMPU adds 1 clock cycle latency to the read transaction. If read speculation is disabled, the SMPU adds 2 clock cycles latency to the read transaction.

## SMPU Operating Modes

The SMPU does not have any strict modes of operation. However, it can be configured for region-based protection where a master with a particular ID can be blocked or allowed based on settings in the SMPU\_RCTL[n] register.

Region-based protection is programmed with registers:

- SMPU\_RCTL[n]
- SMPU\_RADDR[n]
- SMPU\_RIDA[n]
- SMPU\_RIDMSKA[n]
- SMPU\_RIDB[n]
- SMPU\_RIDMSKB[n]

Exclusive Accesses. On the ADSP-SC5xx processors, the hardware supports the exclusive accesses. The exclusive accesses are supported for the SMPU instances that correspond to SMC, DMC 0, DMC 1 and Core\_L2\_0. The exclusive accesses are automatically enabled when one of the cores (ARM Core 0, SHARC+ Core 1, and SHARC+ Core 2) executes the exclusive access instruction. Refer to the Programming Reference manual for more details on how the exclusive access works.

## SMPU Interrupt Signals

There is one interrupt signal associated with the SMPU. If interrupts are enabled, the SMPU\_STAT.IRQ bit is set. The SMPU\_IRQ signal is asserted when the SMPU detects a memory access violation. The target address triggering the interrupt is found in the SMPU\_IADDR register. The SMPU\_IDTLS register provides further details about the cause of the interrupt.

Write errors are prioritized over read errors.

Protection violations (an ID-based violation) can trigger the SMPU interrupt, and can be enabled independently. The protection violation interrupt is enabled by setting the SMPU\_CTL.PINTEN bit.

The SMPU interrupt is asserted for any of the following conditions:

If a second memory access violation occurs while the SMPU\_STAT.IRQ bit is set, the SMPU\_STAT.IOVR (interrupt overrun) bit is set. The SMPU\_IADDR and the SMPU\_IDTLS registers are not updated until the SMPU\_STAT.IRQ bit is cleared. Any information on the subsequent interrupt is lost. Once the SMPU\_STAT.IRQ bit and the SMPU\_STAT.IOVR bit are cleared, any new memory access violations can trigger an interrupt and its details can be captured.

- NOTE: When a blocked access occurs, the SMPU triggers an interrupt when interrupt generation is enabled. The SMPU can also be configured to generate a bus error that propagates back to the system master. The system master can also trigger an interrupt due to this bus error.
- NOTE: On the processor, each SMPU instance has an interrupt. All of the SMPU interrupts are OR'ed and mapped to a single SMPU interrupt on the SEC/GIC. While servicing the SMPU interrupt, check all of the SMPU\_STAT registers to determine which triggered the interrupt. The interrupt service routine clears the SMPU\_STAT.IRQ bit of the all of the SMPU\_STAT registers for which the interrupt is triggered.

## SMPU Status and Error Signals

If bus errors are enabled ( SMPU\_CTL.PBEDIS =0), the SMPU generates and returns a bus error to the master initiating the blocked access. This bit also sets the SMPU\_STAT.BERR bit. The SMPU\_BADDR and SMPU\_BDTLS registers can be read to get the address and details of the transaction that caused the SMPU to generate the error.

Write errors are prioritized over read errors.

A bus error status is returned to the system master if:

- an ID-based violation happened and the SMPU\_CTL.PBEDIS bit =0

If a second memory access violation occurs while the SMPU\_STAT.BERR bit is set, the SMPU\_STAT.BEOVR bit (bus error overrun) is set. The SMPU\_BADDR and the SMPU\_BDTLS registers are not updated until the SMPU\_STAT.IRQ bit is cleared. The information about the transaction that caused the SMPU\_STAT.BEOVR bit to be set is lost.

NOTE: If both the protection violation interrupt is not enabled ( SMPU\_CTL.PINTEN =0) and the protection bus error is disabled ( SMPU\_CTL.PBEDIS =1), the SMPU blocks invalid transactions. However, it does not provide any status or interrupt information indicating that a transaction is blocked.

## SMPU Programming Example

The following example corresponds to using SMPU1 to protect M4 SRAM from MDMA write accesses. In the example the region configured for protection is 4KB at the start of M4 SRAM Bank A.

1. Enable SMPU1 interrupts by writing to the SMPU\_CTL register.
2. Define the memory region to be protected by writing to the SMPU\_RADDR[n] register.

```
*pREG_SMPU_RADDR0 = 0x20000000;
```

3. Program the SMPU\_RIDA[n] and SMPU\_RIDB[n] registers with the ID which is to be an exception to the rule.
4. Program the SMPU\_RIDMSKA[n] and SMPU\_RIDMSKB[n] registers with the mask to be applied to the ID values programmed in the SMPU\_RIDA[n] and SMPU\_RIDB[n] registers respectively.
5. Program the Region Control register ( SMPU\_RCTL[n] register) with the required fields. Control fields like ID invert, write protection enable, read protection enable and memory region size can be configured in this register.
6. Set the SMPU\_RCTL[n].EN bit.

```
*pREG_SMPU_RIDA0 = 0x0081; //corresponds to MDMA_WR *pREG_SMPU_RIDB0 = 0x0081;
```

```
*pREG_SMPU_RIDMSKA0 = 0xFFFF; *pREG_SMPU_RIDMSKB0 = 0xFFFF;
```

```
*pREG_SMPU_RCTL0 = ENUM_SMPU_RCTL_WPROTEN|ENUM_SMPU_RCTL_RPROTDIS| BITM_SMPU_RCTL_WIDCINV|BITM_SMPU_RCTL_RIDCINV| ((0<<BITP_SMPU_RCTL_SIZE) & BITM_SMPU_RCTL_SIZE)
```

```
*pREG_SMPU_RCTL0  |= BITM_SMPU_RCTL_EN;
```

Any accesses violation to the protection enabled by this procedure will be blocked. If the corresponding interrupt is enabled, an interrupt is generated on protection violations.

## ADSP-SC58x SMPU Register Descriptions

The System Memory Protection Unit (SMPU) contains the following registers.

Table 13-5: ADSP-SC58x SMPU Register List

| Name             | Description                  |
|------------------|------------------------------|
| SMPU_BADDR       | Bus Error Address Register   |
| SMPU_BDTLS       | Bus Error Details Register   |
| SMPU_CTL         | SMPU Control Register        |
| SMPU_EXACADD[n]  | Exclusive Access IDn Address |
| SMPU_EXACSTAT[n] | Exclusive Access Status      |
| SMPU_IADDR       | Interrupt Address Register   |
| SMPU_IDTLS       | Interrupt Details Register   |
| SMPU_RADDR[n]    | Region n Address Register    |

Table 13-5: ADSP-SC58x SMPU Register List (Continued)

| Name               | Description                               |
|--------------------|-------------------------------------------|
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

Figure 13-2: SMPU\_BADDR Register Diagram

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000001_72def6150b3932f3f8ad8027e57609b6568f19cf3fb1d7084afeefa99da1c996.png)

Table 13-6: SMPU\_BADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                               |
|--------------------|------------|-----------------------------------------------------------------------|
| 31:0               | VALUE      | Bus Error Address.                                                    |
| (R/NW)             |            | The SMPU_BADDR.VALUE bit field contains the address of the bus error. |

## Bus Error Details Register

The SMPU\_BDTLS register indicates the ID of the bus error transaction, whether the transaction that caused the last bus error was a read, a write, secure or non-secure.

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000002_ad599fa9b375f41879b0d82411ea2681f7a78798588715d5bb755ebdba278dc8.png)

ID of Transaction

Figure 13-3: SMPU\_BDTLS Register Diagram

Table 13-7: SMPU\_BDTLS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                              | Description/Enumeration                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20:8 (R/NW)        | ID         | ID of Transaction. The SMPU_BDTLS.ID bit field provides the ID of the transaction that caused the                                                    | ID of Transaction. The SMPU_BDTLS.ID bit field provides the ID of the transaction that caused the                                                    |
| 1 (R/NW)           | RNW        | Read/Write Status. The SMPU_BDTLS.RNW bit indicates whether the last transaction that caused the bad address error was a read or write.              | Read/Write Status. The SMPU_BDTLS.RNW bit indicates whether the last transaction that caused the bad address error was a read or write.              |
|                    |            | 0                                                                                                                                                    | Transaction that caused last bus error was a write                                                                                                   |
| 0 (R/NW)           | SECURE     | Secure Status Register. The SMPU_BDTLS.SECURE bit indicates whether the last transaction that caused the bad address error was secure or non-secure. | Secure Status Register. The SMPU_BDTLS.SECURE bit indicates whether the last transaction that caused the bad address error was secure or non-secure. |
|                    |            | 0                                                                                                                                                    | Transaction that caused last bus error was non-secure                                                                                                |
|                    |            | 1                                                                                                                                                    | Transaction that caused last bus error was secure                                                                                                    |

## SMPU Control Register

The SMPU\_CTL register provides access to the locking control, error interrupts and SMPU violations.

Figure 13-4: SMPU\_CTL Register Diagram

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000003_419192ffdd1da7b14e804a6f9ceba327091baadcb0f2bdcfa3d3fd0a287ee0d8.png)

Table 13-8: SMPU\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock Bit. When the SMPU_CTL.LOCK bit is set and the global lock signal is asserted from the SPU, the SMPU_CTL register is write-protected. Write-protection is disabled only when the global lock signal becomes deasserted again.                                                                                                                                                                     | Lock Bit. When the SMPU_CTL.LOCK bit is set and the global lock signal is asserted from the SPU, the SMPU_CTL register is write-protected. Write-protection is disabled only when the global lock signal becomes deasserted again.                                                                                                                                                                     |
| 31 (R/W)           | LOCK       | 0                                                                                                                                                                                                                                                                                                                                                                                                      | CTL Global Lock Disable. The SMPU_CTL register is not write-protected.                                                                                                                                                                                                                                                                                                                                 |
| 31 (R/W)           | LOCK       | 1                                                                                                                                                                                                                                                                                                                                                                                                      | CTL Global Lock Enable. The SMPU_CTL register is write-protected.                                                                                                                                                                                                                                                                                                                                      |
| 4 (R/W)            | RLOCK      | RCTLn, RADDRn, RIDxn and RIDMxn Registers Lock Bit. When the SMPU_CTL.RLOCK bit is set, all the registers associated with region-based control ( SMPU_RCTL[n] , SMPU_RADDR[n] , SMPU_RIDA[n] , SMPU_RIDB[n] , SMPU_RIDMSKA[n] and SMPU_RIDMSKB[n] ) are write-pro- tected when the global lock signal is active from the SPU. Write access is allowed again when the global lock signal is deasserted. | RCTLn, RADDRn, RIDxn and RIDMxn Registers Lock Bit. When the SMPU_CTL.RLOCK bit is set, all the registers associated with region-based control ( SMPU_RCTL[n] , SMPU_RADDR[n] , SMPU_RIDA[n] , SMPU_RIDB[n] , SMPU_RIDMSKA[n] and SMPU_RIDMSKB[n] ) are write-pro- tected when the global lock signal is active from the SPU. Write access is allowed again when the global lock signal is deasserted. |
| 4 (R/W)            | RLOCK      | 0                                                                                                                                                                                                                                                                                                                                                                                                      | Region Registers Write-Protect Enable. All region regis- ters are not write-protected.                                                                                                                                                                                                                                                                                                                 |
| 4 (R/W)            | RLOCK      | 1                                                                                                                                                                                                                                                                                                                                                                                                      | Region Registers Write-Protect Disable. All region regis- ters are write-protected.                                                                                                                                                                                                                                                                                                                    |

Table 13-8: SMPU\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | PINTEN     | Protection Violation Interrupt Enable. The SMPU_CTL.PINTEN bit controls whether or not an interrupt is generated when a protection violation occurs.                 |
| 2 (R/W)            | PBETYPE    | Protection Violation Bus Error Type. The SMPU_CTL.PBETYPE bit controls whether a protection violation produces a decode error or a slave error.                      |
| 1                  | PBEDIS     | late the configured protection Protection Violation Bus Error Disable. If set, the SMPU_CTL.PBEDIS bit blocks protection violations, but does not cause a bus error. |
| (R/W)              |            | 0 Bus Error Generation Enable. Transactions which vio-                                                                                                               |
| 0 (R/W)            | RSDIS      | Read Speculation Disable. The SMPU_CTL.RSDIS bit controls whether or not the read addresses are checked before being sent to the slave.                              |
|                    |            | 0 Read Speculation Enable. Read addresses are sent to the slave without checking.                                                                                    |
|                    |            | 1 Read Speculation Disable. Read addresses are checked                                                                                                               |

## Exclusive Access IDn Address

The SMPU\_EXACADD[n] register provides the address ID of an exclusive access.

Figure 13-5: SMPU\_EXACADD[n] Register Diagram

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000004_06615e2292a9ce1a2415f1fa4ba0e5b3449e9b75519bf4bcef1d890656360d69.png)

Table 13-9: SMPU\_EXACADD[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration       |
|--------------------|------------|-------------------------------|
| 31:0               | VALUE      | Exclusive Access IDn Address. |
| (R/NW)             |            |                               |

## Exclusive Access Status

The SMPU\_EXACSTAT[n] register provides the exclusive access ID, read size and read length as well as the indication that the access was valid.

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000005_72f1c5a313dc1fc1265bebf5150aedec42c2e3e71397fbe76a67dc3e681b4c82.png)

Exclusive Access ID

Figure 13-6: SMPU\_EXACSTAT[n] Register Diagram

Table 13-10: SMPU\_EXACSTAT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration       |
|--------------------|------------|-------------------------------|
| 20:8 (R/NW)        | ARID       | Exclusive Access ID.          |
| 7:5 (R/NW)         | ARSIZE     | Exclusive Access Read Size.   |
| 4:1 (R/NW)         | ARLEN      | Exclusive Access Read Length. |
| 0 (R/NW)           | VALID      | Valid Exclusive Access Read.  |

## Interrupt Address Register

The SMPU\_IADDR register indicates an attempt to make a read or write access to unimplemented addresses or accesses are non-aligned. The SMPU issues a bus error for this condition.

Figure 13-7: SMPU\_IADDR Register Diagram

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000006_0daefdd80a4a1557265995d7f6d0a17a0ec1bc86012611539b9b911019dfab42.png)

Table 13-11: SMPU\_IADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | Interrupt Address. The SMPU_IADDR.VALUE bit field is the address where an attempt to access an un- implemented address or a non-aligned access has occurred. |

## Interrupt Details Register

The SMPU\_IDTLS register provides the ID of the last signaled interrupt, whether the interrupt was caused by a read or write, and whether the transaction that caused the last signaled interrupt was secure.

Figure 13-8: SMPU\_IDTLS Register Diagram

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000007_1a2083cc5de7d6375871353eb37a156227e951bcc5ab2fd21d487b7b612108d5.png)

Table 13-12: SMPU\_IDTLS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                             | Description/Enumeration                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 20:8 (R/NW)        | ID         | ID of Transaction. The SMPU_IDTLS.ID bit field provides the ID of the transaction that caused the interrupt.                        | ID of Transaction. The SMPU_IDTLS.ID bit field provides the ID of the transaction that caused the interrupt.                        |
| 1 (R/NW)           | RNW        | Read/Write Status. The SMPU_IDTLS.RNW bit indicates whether the last transaction that caused the in- terrupt was a read or write.   | Read/Write Status. The SMPU_IDTLS.RNW bit indicates whether the last transaction that caused the in- terrupt was a read or write.   |
|                    |            | 0                                                                                                                                   | Transaction that caused last signaled interrupt was a write                                                                         |
|                    |            | 1                                                                                                                                   | Transaction that caused last signaled interrupt was a read                                                                          |
| 0 (R/NW)           | SECURE     | Secure Status. The SMPU_IDTLS.SECURE bit indicates whether the last transaction that caused the interrupt was secure or non-secure. | Secure Status. The SMPU_IDTLS.SECURE bit indicates whether the last transaction that caused the interrupt was secure or non-secure. |
|                    |            | 0                                                                                                                                   | Transaction that caused last signaled interrupt was non- secure                                                                     |
|                    |            | 1                                                                                                                                   | Transaction that caused last signaled interrupt was se- cure                                                                        |

## Region n Address Register

The SMPU\_RADDR[n] register is used to define the base address for a memory region to be protected.

Figure 13-9: SMPU\_RADDR[n] Register Diagram

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000008_205d6d6fe865998ed04fd7a50191c87f9d84925a28dfab24243b9dcc46140677.png)

Table 13-13: SMPU\_RADDR[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:12 (R/W)        | BADDR      | Region n Base Address 20 Most Significant Bits. The SMPU_RADDR[n].BADDR bit field defines the base address for a memory re- gion to be protected. |

## Region n Control Register

The SMPU\_RCTL[n] register is used to define the level of protection for a region of memory. The protection of a region is controlled and defined by this register and the SMPU\_RADDR[n] , SMPU\_RIDA[n] , SMPU\_RIDB[n] , SMPU\_RIDMSKA[n] , and SMPU\_RIDMSKB[n] registers.

Figure 13-10: SMPU\_RCTL[n] Register Diagram

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000009_b2b1a354ac397f9f9087ac036f683bfa9c9a77759f2a466a3f1fe89bb6f7edd5.png)

Table 13-14: SMPU\_RCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                               | Description/Enumeration                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | WIDCINV    | Write Transaction ID Compare Invert. The SMPU_RCTL[n].WIDCINV bit inverts the write ID match result.                  | Write Transaction ID Compare Invert. The SMPU_RCTL[n].WIDCINV bit inverts the write ID match result.                  |
|                    |            | 0                                                                                                                     | Write transaction ID comparison result not inverted                                                                   |
|                    |            | 1                                                                                                                     | Write transaction ID comparison result inverted                                                                       |
| 10 (R/W)           | WPROTEN    | Write Transaction Protection Enable. SMPU_RCTL[n].WPROTEN                                                             | bit enables protection against ID-based write                                                                         |
|                    |            | 0                                                                                                                     | Write transaction ID-based protection disabled                                                                        |
|                    |            | 1                                                                                                                     | Write transaction ID-based protection enabled                                                                         |
| 9 (R/W)            | RIDCINV    | Read Transaction ID Compare Invert. When the SMPU_RCTL[n].RIDCINV bit is set, the read ID match result is invert- ed. | Read Transaction ID Compare Invert. When the SMPU_RCTL[n].RIDCINV bit is set, the read ID match result is invert- ed. |
|                    |            | 0                                                                                                                     | Read transaction ID comparison result not inverted                                                                    |
|                    |            | 1                                                                                                                     | Read transaction ID comparison result inverted                                                                        |

Table 13-14: SMPU\_RCTL[n] Register Fields (Continued)

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

Table 13-14: SMPU\_RCTL[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                    |
|--------------------|------------|----------------------------------------------------------------------------|
| 0 (R/W)            | EN         | Region Enable. The SMPU_RCTL[n].EN bit enables the protection of a region. |
| 0 (R/W)            | EN         | 0 Disabled                                                                 |
| 0 (R/W)            | EN         | 1 Enabled                                                                  |

## SMPU Revision ID Register

The SMPU\_REVID register provides the major and minor revision numbers of this module.

Figure 13-11: SMPU\_REVID Register Diagram

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000010_686c06ed8f9a563ba4228db882aa10b1d95804dc4b2242b4c4c101e376a6955c.png)

Table 13-15: SMPU\_REVID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:4                | MAJOR      | Major Version ID.         |
| 3:0 (R/NW)         | REV        | Incremental Version ID.   |

## Region n ID A Register

The SMPU\_RIDA[n] register is used for ID comparison 'A'. This comparison is performed after a mask is applied to both the transaction ID (from either the read or write IDs) and the register value. An ID match means that the ID is the exception to the rule and the read or write is allowed even if the region is read or write-protected. For more detail, refer to the ID Comparison section.

Figure 13-12: SMPU\_RIDA[n] Register Diagram

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000011_edff4d6659cd415677c36e0c009820cb24243b2aa127bdcc723b4d7e19778369.png)

Table 13-16: SMPU\_RIDA[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:0 (R/W)         | ID         | Region n ID Register A. The SMPU_RIDA[n].ID bit field, combined with the mask provides the means to bypass the configured memory protection for a region. |

## Region n ID B Register

The SMPU\_RIDB[n] register is used for ID comparison 'B'. This comparison is performed after a mask is applied to both the transaction ID (from either the read or write IDs) and the register value. An ID match means that the ID is the exception to the rule and the read or write is allowed even if the region is read or write-protected. For more details, refer to the ID Comparison section.

Figure 13-13: SMPU\_RIDB[n] Register Diagram

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000012_396293ad1317dfe25b4b98dba83262283a5d3f0b4ba3bca7e483db86e6980297.png)

Table 13-17: SMPU\_RIDB[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:0 (R/W)         | ID         | Region n ID Register B. The SMPU_RIDB[n].ID bit field, combined with the mask provides the means to bypass the configured memory protection for a region. |

## Region n ID Mask A Register

The SMPU\_RIDMSKA[n] register is used for ID comparison 'A'. The mask allows or disallows certain IDs from affecting the final result of the ID match. For more details, refer to the ID Comparison section.

Figure 13-14: SMPU\_RIDMSKA[n] Register Diagram

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000013_cf5f7ae4a44f1af5ef262ae29e5839a8103197615568531f57a7226c0d7312d1.png)

Table 13-18: SMPU\_RIDMSKA[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:0 (R/W)         | MSK        | Region n ID Mask Register A. The SMPU_RIDMSKA[n].MSK bit field, combined with the incoming transaction, provides the means to bypass the configured memory protection for a region. |

## Region n ID Mask B Register

The SMPU\_RIDMSKB[n] register is used for ID comparison 'B'. The mask allows or disallows certain IDs from affecting the final result of the ID match. For more details, refer to the ID Comparison section.

Figure 13-15: SMPU\_RIDMSKB[n] Register Diagram

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000014_ca0048d189fa556d464291bf2df819489b1b4e685a0d6327e865c33ce253bb80.png)

Table 13-19: SMPU\_RIDMSKB[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12:0 (R/W)         | MSK        | Region n ID Mask Register B. The SMPU_RIDMSKB[n].MSK bit field, combined with the incoming transaction provides the means to bypass the configured memory protection for a region. |

## SMPU Control Secure Accesses Register

The SMPU\_SECURECTL register provides the bits required to set up the security settings for the processor. These settings includes error generation and read/write security.

Figure 13-16: SMPU\_SECURECTL Register Diagram

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000015_f20e2cd5a9b2dd2c66ce7b8f1b0463b3d6f6440909db31e786a36dd10449f384.png)

Table 13-20: SMPU\_SECURECTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock Bit. When the SMPU_SECURECTL.LOCK bit is set and the global lock signal is asserted from the SPU, the SMPU_SECURECTL register is write-protected. Write-protection is disabled only when the global lock signal becomes deasserted again. |
| 11 (R/W)           | WSECDIS    | Secure Write Transaction Disable. The SMPU_SECURECTL.WSECDIS bit disables secure write transactions.                                                                                                                                           |
| 10 (R/W)           | WNSEN      | Non-secure Write Transaction Enable. The SMPU_SECURECTL.WNSEN bit enables non-secure write transactions. 0 Disable non-secure writes                                                                                                           |

Table 13-20: SMPU\_SECURECTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | RSECDIS    | Secure Read Transaction Disable. The SMPU_SECURECTL.RSECDIS bit disables secure read transactions. 0 Enable secure read transactions                                                                                                                                                                                                          |
| 8 (R/W)            | RNSEN      | Non-secure Read Transaction Enable. The SMPU_SECURECTL.RNSEN bit enables non-secure read transactions. 0 Disable non-secure read transactions                                                                                                                                                                                                 |
| 3 (R/W)            | RLOCK      | Secure Region Registers Lock Bit. When the SMPU_SECURECTL.RLOCK bit is set, the secure region control registers, SMPU_SECURERCTL[n] , are write-protected when the global lock signal is active from the SPU. When the global lock signal is deasserted, write access is allowed again. 0 Disable write-protection on secure region registers |
| 2 (R/W)            | SINTEN     | 1 Enable write-protection on secure region registers Security Violation Interrupt Enable. The SMPU_SECURECTL.SINTEN bit enables interrupt generation when a security violation occurs.                                                                                                                                                        |
| 1 (R/W)            |            | 0 Disable security settings violation interrupt 1 Enable security settings violation interrupt                                                                                                                                                                                                                                                |
|                    | SBETYPE    | Security Violation Bus Error Type. The SMPU_SECURECTL.SBETYPE bit controls whether a decode error or a slave error is returned when a security violation occurs. 0 Return a decode error error which violates the security settings 1 Return a slave error which violates the security settings                                               |
| 0 (R/W)            | SBEDIS     | Security Violation Bus Error Disable. The SMPU_SECURECTL.SBEDIS bit controls whether or not a bus error is caused when a security violation occurs. 0 Enable bus error                                                                                                                                                                        |

## Region n Control Secure Accesses Register

The SMPU\_SECURERCTL[n] register contains bits that configure read/write security for a specific region.

Figure 13-17: SMPU\_SECURERCTL[n] Register Diagram

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000016_fac60963e9c7b9285b7e01b19cdce4b5c7bf918131482feed03c7cca4cc4e9a3.png)

Table 13-21: SMPU\_SECURERCTL[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                             | Description/Enumeration                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | WSECDIS    | Secure Write Transaction Disable. The SMPU_SECURERCTL[n].WSECDIS bit disables secure write transactions for the memory region.      | Secure Write Transaction Disable. The SMPU_SECURERCTL[n].WSECDIS bit disables secure write transactions for the memory region.      |
| 3 (R/W)            | WSECDIS    | 0                                                                                                                                   | Enable secure write transactions to this region                                                                                     |
| 3 (R/W)            | WSECDIS    | 1                                                                                                                                   | Disable secure write transactions to this region                                                                                    |
| 2 (R/W)            | WNSEN      | Non-secure Write Transaction Enable. This SMPU_SECURERCTL[n].WNSEN bit enables non-secure write transactions for the memory region. | Non-secure Write Transaction Enable. This SMPU_SECURERCTL[n].WNSEN bit enables non-secure write transactions for the memory region. |
| 2 (R/W)            | WNSEN      | 0                                                                                                                                   | Disable non-secure write transactions to this region                                                                                |
| 2 (R/W)            | WNSEN      | 1                                                                                                                                   | Enable non-secure write transactions to this region                                                                                 |
| 1 (R/W)            | RSECDIS    | Secure Read Transaction Disable. The SMPU_SECURERCTL[n].RSECDIS bit disables secure read transactions for the memory region.        | Secure Read Transaction Disable. The SMPU_SECURERCTL[n].RSECDIS bit disables secure read transactions for the memory region.        |
| 1 (R/W)            | RSECDIS    | 0                                                                                                                                   | Enable secure read transactions to this region                                                                                      |
| 1 (R/W)            | RSECDIS    | 1                                                                                                                                   | Disable secure read transactions to this region                                                                                     |
| 0 (R/W)            | RNSEN      | Non-secure Read Transaction Enable. The SMPU_SECURERCTL[n].RNSEN bit enables non-secure read transactions for the memory region.    | Non-secure Read Transaction Enable. The SMPU_SECURERCTL[n].RNSEN bit enables non-secure read transactions for the memory region.    |
| 0 (R/W)            | RNSEN      | 0                                                                                                                                   | Disable non-secure read transactions to this region                                                                                 |
| 0 (R/W)            | RNSEN      | 1                                                                                                                                   | Enable non-secure read transactions to this region                                                                                  |

## SMPU Status Register

The SMPU\_STAT register provides the state of the SMPU and indicates various errors. All bits in this register are write 1 to clear.

Figure 13-18: SMPU\_STAT Register Diagram

![Image](16_System_Memory_Protection_Unit_(SMPU)_artifacts/image_000017_269ae22698367bcff61be508977ca0ca7996fecaf11d64a7ab6e450cb1fdaa36.png)

Table 13-22: SMPU\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W1C)         | LWERR      | Lock Write Error. The SMPU_STAT.LWERR bit is set when SMPU_CTL.LOCK bit =1, the global lock signal is asserted from the SPU and a read or write attempt was made to the SMPU_CTL MMR.                            |
| 16 (R/W1C)         | ADRERR     | Address Error. The SMPU_STAT.ADRERR bit is set when the SMPU MMRis accessed as an un- aligned address, or when a read-only MMRis written to.                                                                     |
| 3 (R/W1C)          | BEOVR      | Bus Error Overrun. The SMPU_STAT.BEOVR bit indicates that another bus error had occurred. Any new information about the most recent violation which caused the bus error is not captured. 0 No Bus Error overrun |
| 3 (R/W1C)          | 1          | Bus Error overrun has occurred                                                                                                                                                                                   |
| 3 (R/W1C)          |            |                                                                                                                                                                                                                  |

Table 13-22: SMPU\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                         | Description/Enumeration                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W1C)          | BERR       | Bus Error. This SMPU_STAT.BERR bit indicates if a bus error was generated.                                                                                                                                                      | Bus Error. This SMPU_STAT.BERR bit indicates if a bus error was generated.                                                                                                                                                      |
| 2 (R/W1C)          | BERR       | 0                                                                                                                                                                                                                               | No Bus Error since this bit has been cleared                                                                                                                                                                                    |
| 2 (R/W1C)          | BERR       | 1                                                                                                                                                                                                                               | Bus Error has been generated                                                                                                                                                                                                    |
| 1 (R/W1C)          | IOVR       | Interrupt Overrun. The SMPU_STAT.IOVR bit indicates if another violation occurred while the previ- ous violation interrupt was not finished being serviced. Information about the most re- cent violation is then not captured. | Interrupt Overrun. The SMPU_STAT.IOVR bit indicates if another violation occurred while the previ- ous violation interrupt was not finished being serviced. Information about the most re- cent violation is then not captured. |
| 1 (R/W1C)          | IOVR       | 0                                                                                                                                                                                                                               | No Interrupt overrun                                                                                                                                                                                                            |
| 1 (R/W1C)          | IOVR       | 1                                                                                                                                                                                                                               | Interrupt overrun has occurred                                                                                                                                                                                                  |
| 0 (R/W1C)          | IRQ        | Interrupt Request. The SMPU_STAT.IRQ bit provides an indication that an interrupt has been generat- ed.                                                                                                                         | Interrupt Request. The SMPU_STAT.IRQ bit provides an indication that an interrupt has been generat- ed.                                                                                                                         |
| 0 (R/W1C)          | IRQ        | 0                                                                                                                                                                                                                               | No Interrupt since this bit has been cleared                                                                                                                                                                                    |
| 0 (R/W1C)          | IRQ        | 1                                                                                                                                                                                                                               | Interrupt has been generated                                                                                                                                                                                                    |