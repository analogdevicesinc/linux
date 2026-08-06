# Boot ROM and Booting the Processor

<!-- source: 053_Boot_ROM_and_Booting_the_Processor.pdf | original pages 2934–3079 -->

## 46    Boot ROM and Booting the Processor

Bootstrapping or booting is the series of events that occur when the system applies power to the processor or when the processor enters a hardware reset state. This section gives an in-depth description of these events and how to integrate an application effectively.

On reset, the processor begins fetching instruction from an internal ROM. The boot code contained within the ROM is designed to facilitate loading an application. The boot code can automatically initialize certain peripherals for communication based on a chosen boot mode, then load an application. For more information on what boot modes are available, see the Boot Modes section. The boot code can efficiently load an entire application, code, and data, into appropriate locations after the development tools repackage the application into a boot stream.

A boot stream is an application or data that the boot-loader tool splits into blocks. A 16-byte header in each block provides instruction to the boot code for processing the associated data. The processor can perform several boot functions, depending on the flags set in the header. For more details on what options are available and a description of the stream format, refer to the Boot Loader Stream section.

The boot ROM provides a mechanism through available non-volatile programmable memory (OTP on this processor) to customize different aspects of the boot process. These customizations include overriding default boot-peripheral instance, overriding default peripheral-timing parameters, and disabling boot modes.

NOTE: In this chapter the term boot ROM describes the boot code that cannot be altered. The terms program and application are used to describe code that is used to customize the boot process.

Several utilities of the boot code are also available to the application. These utilities include features such as copying memory, comparing memory, or loading another boot stream at run time. The APIs may be used to help ensure that application code is more compatible with future products. The boot code also provides the ability to define a custom boot mode. This capability helps when support is not available for a desired boot mode. It allows second stage boot loaders for unsupported boot peripherals to leverage a significant amount of the existing boot ROM functionality.

## SRAM Requirements

The boot process reserves 8K bytes of L2 ECC Protected SRAM for dedicated use. This topic describes how the reserved memory region is used during boot.

The boot process requires SRAM resources for stack use and to store various data items that require read/write access during the boot process. 8K bytes of L2 ECC protected SRAM is reserved for this purpose. The Boot Process SRAM Requirements table describes the various items stored in this memory region.

Table 46-1: Boot Process SRAM Requirements

| Address    |   Size (Bytes) | Item                                             | Description                                                                                                                                                                                                                                                                                               |
|------------|----------------|--------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0x201FE000 |              4 | Reserved                                         | Reserved                                                                                                                                                                                                                                                                                                  |
| 0x201FE004 |              4 | Pointer to the struct ADI_ROM_BOOT_CONFIG object | Pointer to the boot configuration structure that is located on the stack. This location is used to find the location of the boot structure on the stack for debug.                                                                                                                                        |
| 0x201FE008 |              8 | Reserved                                         | Reserved                                                                                                                                                                                                                                                                                                  |
| 0x201FE010 |           1024 | Internal Intermediate Buffer 0                   | The first of two internal buffers used for intermediate storage of boot content when using indirect and page mode accesses and for secure boot operations. Two buffers are used to allow SHA-224 and AES-128 operations to be performed on one buffer while simultaneously loading the other buffer.      |
| 0x201FE410 |           1024 | Internal Intermediate Buffer 1                   | The second of two internal buffers used for intermediate stor- age of boot content when using indirect and page mode access- es and for secure boot operations. Two buffers are used to allow SHA-224 and AES-128 operations to be performed on one buffer while simultaneously loading the other buffer. |
| 0x201FE810 |             16 | struct ADI_ROM_BOOT_HEADER object                | Storage location for all the block headers of the boot stream.                                                                                                                                                                                                                                            |
| 0x201FE820 |           2912 | Storage for Secure Boot related de- scriptors    | Contains a number of buffers for the various descriptors used by the Cryptographic Accelerators and provides storage for the secure header of a secure boot stream.                                                                                                                                       |
| 0x201FF400 |           3072 | Stack for the boot process                       | The primary booting cores stack. The processor core should lo- cate the stack in this region to preserve security in secure boot operations.                                                                                                                                                              |

NOTE: To preserve the security of the product, the 8K byte region described here is not a bootable region of memory. If the boot process determines that a block of data in the boot stream is targeted towards this memory region, the boot process terminates and enters either the default error handler or, if applicable, a user-defined error handler. This reserved memory region is free for use after the boot process completes. To preserve security when using the boot API to boot a secure boot stream, the stack used during the execution of the boot API must be located at the default location in this reserved 8K bytes region of memory.

## Preboot Operations

Preboot is responsible for configuration of all system resources prior to executing the required boot operation.

The steps performed by the preboot process are described here in the order of execution. Numerous stages of the preboot process are conditional based upon the content of the RCU\_BCODE register.

NOTE: When a power on reset, hard reset, and software triggered system reset event completes, the processor operates in PLL Bypass mode (default). Partway through the preboot sequence the processor is brought into Full-On mode at the default settings unless the program has provisioned custom CGU settings in the OTP. The Oscillator Watchdog fault, enabled by default when reset completes, is disabled at this stage unless Oscillator Watchdogs settings are also supplied.

## Start-up Sequence

This section describes the initial start-up sequence of all cores in the processor.

Upon completion of a power-on reset, hardware reset, or system reset event, only a single core is released from reset. It is responsible for managing the boot process. The following sections describe in detail the sequence of events that occur for each of the cores on the various product derivatives.

## Core Reset Sequencing

System and hardware reset events result in the processor state being reset and the boot sequence is executed. Only a single core is initially released from reset and starts execution of the preboot software from the boot ROM. The sequencing between the RCU and the various cores is shown for each of the product derivatives.

For the ADSP-SC59x derivatives, core 0 is responsible for controlling the boot process. Cores 1 and 2 are held in reset until core 0 releases them from reset during the preboot phase of the boot process.

The initial sequence of events after the completion of the reset sequence are shown below.

Figure 46-1: Core Reset Sequencing - ADSP-SC59x

<!-- image -->

The ADSP-2159x derivative contains two cores with IDs of 1 and 2. In this device, core 1 is responsible for controlling the boot process; core 2 is held in reset until released by core 1 during the preboot phase of the boot process. The initial sequence of events after the completion of the reset sequence are shown.

## Core 0 Start-up

Describes the initial operations performed by the core 0 immediately after being released from reset and soft vector processing has completed. This topic is only applicable to the ADSP-SC59x processors.

Upon initial release from reset, the soft vectoring process results in core 0 executing the boot process as long as the RCU\_SVECT0 register contained the default reset value during the soft vector processing.

The core performs the following initial operations:

- Clear RCU\_MSG.C0IDLE
- Check for OTPC boot completion
- Disable MMU
- Disable instruction and data caches
- Invalidate the TLB
- Read RCU\_STAT register and store locally in register set
- Install Faults
- Configure L2 controller
- Initiate hardware controlled L2 memory initialization sequence
- Enter IRQ mode and configure the stack pointer
- Enter FIQ mode and configure the stack pointer
- Enter ABT mode and configure the stack pointer
- Enter UND mode and configure the stack pointer
- Enter SVC mode and configure the stack pointer

## Core 1 Start-up

Describes the initial operations performed by the core 1 immediately after being released from reset and soft vector processing has completed.

Upon initial release from reset, the soft vectoring process results in core 1 executing from the boot ROM as long as the RCU\_SVECT1 register contained the default reset value during the soft vector processing. The operations the core performs varies depending on the product.

ADSP-SC59x Processors - Core 1 is not responsible for the booting of the processor from reset and thus enters a safe IDLE state allowing access to the cores L1 resources by any other core.

The core performs the following initial operations:

- Clear RCU\_MSG.C1IDLE
- Check for OTPC boot completion.

- Disable Software Return feature in the BTB.
- Disable the BTB.
- Disable cache via MODE2 register.
- Flush the cache.
- Set IRPTL to 0x00000000.
- Set RCU\_MSG.C1IDLE
- Enter an endless IDLE loop

ADSP-2159x Processors - Core 1 is responsible for controlling the boot process and the following actions are performed.

- Clear RCU\_MSG.C0IDLE
- Check for OTPC boot completion.
- Disable Software Return feature in the BTB.
- Disable the BTB.
- Disable cache via MODE2 register.
- Flush the cache.
- Set IRPTL to 0x00000000.
- Configure primary and secondary DAG configurations to setup the C run-time environment.
- Clear the CBUFEN, SRD1H, SRD1L, SRD2H, SRD2L, ALUSAT, TRUNCATE bits in the MODE1 register.
- Set the IRPTEN, IPERREN, DPERREN, SPERREN bits in the MODE1 register.
- Read the RCU0\_STAT register and store locally in the register set.
- Install the x.
- Configure L2 Controller
- Initiate hardware controlled L2 memory initialization sequence

Table 46-2: Primary/Secondary DAG Configuration for C Run-Time Setup

| Primary/Secondary DAG Register   |   Value | Description                                  |
|----------------------------------|---------|----------------------------------------------|
| M7, M15                          |      -1 | Dedicated registers must always be set to -1 |
| M6, M14                          |       1 | Dedicated registers must always be set to 1  |
| M5, M13                          |       0 | Dedicated registers must always be set to 0  |

Table 46-2: Primary/Secondary DAG Configuration for C Run-Time Setup (Continued)

| Primary/Secondary DAG Register   | Value      | Description                              |
|----------------------------------|------------|------------------------------------------|
| L0-L5                            | 0          | Preserved registers set initially to 0   |
| L6, L7                           | 0          | Stack Length Register set initially to 0 |
| L8-L15                           | 0          | Preserved registers set initially to 0   |
| B6, B7                           | 0x201FF400 | Stack Base Registers                     |
| I7                               | 0x201FFFFC | Stack Pointer                            |
| L6, L7                           | 0x000002FF | Stack Length Registers                   |
| I6                               | 0x201FFFFC | Frame Pointer                            |

## Core 2 Start-up

Describes the initial operations performed by core 2 immediately after being released from reset and soft vector processing has completed.

Upon initial release from reset the soft vectoring process results in core 2 executing from the boot ROM as long as the RCU\_SVECT2 register contained the default reset value during the soft vector processing. The operations the core performs varies depending on the product.

In the ADSP-SC59x Processors , core 1 is not responsible for the booting of the processor from reset and thus enters a safe IDLE state allowing access to the cores L1 resources by any other core. Core 2 has no entry point into the main boot rom to execute the preboot process or controller boot process from reset of the device.

From reset, core 2 can perform the following operations:

- Clear RCU\_MSG.C2IDLE
- Check for OTPC boot completion.
- Disable Software Return feature in the BTB.
- Disable the BTB.
- Disable cache via MODE2 register.
- Flush the cache.
- Set IRPTL to 0x00000000.
- Set RCU\_MSG.C2IDLE
- Enter an endless IDLE loop

## Fault Configuration

Describes the initial fault sources that are enabled allowing the processor to signal a fault to the system.

The following faults are enabled via the SEC . Only the faults are enabled, the boot process does not install any SEC interrupts.

Table 46-3: Initial Faults installed during Preboot

|   SEC Fault ID | SEC Fault Name      | Description                                                                                      |
|----------------|---------------------|--------------------------------------------------------------------------------------------------|
|              0 | INTR_SEC0_ERR       | SEC Error                                                                                        |
|              3 | INTR_WDOG0_EXP      | WDOGExpire                                                                                       |
|              4 | INTR_WDOG1_EXP      | WDOGExpire                                                                                       |
|              6 | INTR_OTPC0_ERR      | OTPC Expire                                                                                      |
|             10 | INTR_L2CTL0_ECC_ERR | L2 ECC Error                                                                                     |
|             76 | INTR_SOFT3_INT      | Software Driven Interrupt 3. This is raised in the boot ROMerror han- dler should it be entered. |
|            190 | INTR_CRC0_ERR       | CRC Error                                                                                        |
|            191 | INTR_CRC1_ERR       | CRC Error                                                                                        |
|            194 | INTR_MDMA1_SRC_ERR  | MDMA1Source DMAChannel Error                                                                     |
|            195 | INTR_MDMA1_DST_ERR  | MDMA1Destination DMAChannel Error                                                                |

The SEC is configured for a fault delay of 0x100 via the SEC\_FDLY.COUNT and SEC\_FSRDLY.COUNT bit fields. This allows for a delay before the fault assertion is a custom error handler is installed and any SEC interrupts are enabled and handled for more advanced second stage boot scenarios.

The SYSFAULT pin is configured via the SEC to support both incoming and outgoing faults by enabling the SEC\_FCTL.FIEN and SEC\_FCTL.FOEN bits. The boot process captures an incoming fault if that fault is asserted by the external system on handover to the user application after boot.

- NOTE: The installation of the fault sources is enabled by default and may be optionally bypassed using the RCU\_BCODE.NOFAULTS bit.
- NOTE: In the ADSP-SC594, ADSP-SC592 and ADSP-21594 processors the SYSFAULT pin is dedicated. However, in the ADSP-21593 processor, the SYSFAULT appears on the PC\_07 in pin mux. For specific boot modes in the ADSP-21593 processor, the SYSFAULT pin is not available for fault indication due to this pin being used by the booting peripheral.

## L2 Controller Configuration

The L2 controller is configured to ensure all L2 memory banks are ECC enabled and that the L2 memory scrub feature is disabled.

- NOTE: The L2 Controller configuration is enabled by default and may be optionally bypassed by setting the RCU\_BCODE.NOL2CONFIG bit.

## L2 Memory Initialization

The L2 memory subsytem is ECC protected by default. The L2 memory must be initialized to ensure that no read from the L2 memory generates an ECC error. All L2 memory banks are initialized using the L2CTL\_INIT bit. The boot process waits for each memory bank to signal initialization complete using the L2CTL\_ISTAT bit. The RCU\_MSG.L2INIT is set when L2 memory initialization is complete.

NOTE: The L2 memory initialization process is enabled by default and may be optionally bypassed by setting the RCU\_BCODE.NOMEMINIT bit.

## Idle On Entry

The Idle On Entry feature instructs the debugger to halt the boot code before continuing with any further preboot operations.

When this feature is enabled the processor executes a WFI/IDLE instruction and then continues once an event (such as an emulator exception) is serviced.

NOTE: Idle On Entry processing is disabled by default and is enabled by setting the RCU\_BCODE.IDLEONENTRY bit prior to performing a system reset operation.

## SPU Configuration

The SPU is configured differently depending upon the detected security state of the device. The first operation clears any existing security violations that may be indicated in the SPU\_STAT register.

None of the peripherals are configured to ignore security signals and only the following controllers are configured to generate secure transactions.

Table 46-4: SPU Secure Controllers during Boot

|   SPU Endpoint ID | Controller Name              |
|-------------------|------------------------------|
|               146 | MDMA0 Source DMAChannel      |
|               147 | MDMA0 Destination DMAChannel |
|               148 | MDMA1 Source DMAChannel      |
|               149 | MDMA1 Destination DMAchannel |
|               144 | CRC0                         |
|               145 | CRC1                         |
|               175 | PKTE                         |

## SMPU Configuration

The SMPU is used to restrict access to various memory regions in the processor. The configuration applied during boot differs depending on the locked state of the processor.

By default the SMPU instances only allow secure read and write transactions. The SMPU Configuration table describes the configurations for the different security states.

Speculative reads are also disabled by setting the SMPU\_CTL.RSDIS bit.

Table 46-5: SMPU Configuration

| SMPU Instance          |   SMPU Instance | Open SMPU_SECURECTL Value                        | Locked SMPU_SECURECTL Value      |
|------------------------|-----------------|--------------------------------------------------|----------------------------------|
| Core_L2_RAM_Boot_ROM0  |               2 | SMPU_SECURECTL.WNSEN &#124; SMPU_SECURECTL.RNSEN | 0x00000000 (Default Reset Value) |
| DMA_L2_RAM_Boot_ROM0   |               3 | SMPU_SECURECTL.WNSEN &#124; SMPU_SECURECTL.RNSEN | 0x00000000 (Default Reset Value) |
| DMC                    |               9 | SMPU_SECURECTL.WNSEN &#124; SMPU_SECURECTL.RNSEN | 0x00000000 (Default Reset Value) |
| SPI                    |              11 | SMPU_SECURECTL.WNSEN &#124; SMPU_SECURECTL.RNSEN | 0x00000000 (Default Reset Value) |
| Core_L2_ROM1_Boot_ROM1 |               4 | SMPU_SECURECTL.WNSEN &#124; SMPU_SECURECTL.RNSEN | 0x00000000 (Default Reset Value) |
| Core_L2_ROM2_Boot_ROM2 |               6 | SMPU_SECURECTL.WNSEN &#124; SMPU_SECURECTL.RNSEN | 0x00000000 (Default Reset Value) |
| DMA_L2_ROM1_Boot_ROM1  |               5 | SMPU_SECURECTL.WNSEN &#124; SMPU_SECURECTL.RNSEN | 0x00000000 (Default Reset Value) |

## Secure Debug Key Processing

In the event the processor is locked, the debug tools must submit a secure debug key that is matched with a key on the processor. A 128-bit Secure Debug Key must be provided the application program prior to locking the device.

The secure debug key is read from the OTP memory and then written to the corresponding register in the TAPC. After the key has been written the TAPC\_SDBGKEY\_CTL.VALID bit is set. Once the debug tools then submit their key a key compare operation is performed.

Debug tools must wait for the boot software to load the key before setting the TAPC\_SDBGKEY\_CTL.VALID bit before submitting the key for comparison.

The 128-bit Secure Debug Key is loaded as follows from the storage area in OTP .

Table 46-6: Secure Debug Key Load Procedure

| Secure Debug Key[127:0]   | Register      |
|---------------------------|---------------|
| Secure Debug Key[31:0]    | TAPC_SDBGKEY0 |
| Secure Debug Key[63:32]   | TAPC_SDBGKEY1 |
| Secure Debug Key[95:94]   | TAPC_SDBGKEY2 |
| Secure Debug Key[127:96]  | TAPC_SDBGKEY3 |

NOTE: In the ADSP-SC59x processor family, the boot ROM supports two 128-bit secure debug keys, secure\_emu\_key0 and secure\_emu\_key1 , where only one of them is used by the boot ROM at a time. Each of the secure debug keys is associated with a bypass key ( emu\_key0\_disable or

emu\_key1\_disable ) which decides the validity of the corresponding key. Any non-zero value written into the 16-bit bypass key invalidates that particular secure debug key.

The validity of the secure debug keys can be determined as shown in the Secure Debug Key Validation table:

Table 46-7: Secure Debug Key Validation

| Bypass Key 0 (emu_key0_disable) Value   | Bypass Key 1 (emu_key1_disable) Value   | Valid Secure Debug Key   |
|-----------------------------------------|-----------------------------------------|--------------------------|
| Zero                                    | Zero                                    | secure_emu_key0          |
| Zero                                    | Non-zero                                | secure_emu_key0          |
| Non-zero                                | Zero                                    | secure_emu_key1          |
| Non-zero                                | Non-zero                                | Both keys are valid      |

CAUTION: The boot code bypasses the key load operation entirely when a 16-bit non-zero value is programmed on both emu\_key0\_disable and emu\_key1\_disable fields in OTP or a key of 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF is programmed in both secure debug key fields provided in theOTP . When debug access is subsequently needed, the program must load the key to the TAPC. When the processor fails to boot (due to corrupted firmware) then there is no debug access. The only way to gain access is to load an authenticated boot image that can then load the required keys prior to attempting to connect with a debugger.

## CGU Configuration

This step in the preboot process reconfigures the internal clocks on the processor for improved boot performance.

The boot process can optionally configure the CGU in order to improve boot performance. The settings for the CGU are located within the struct ADI\_ROM\_OTP\_BOOT\_CGU\_INFO structure that has storage allocated in the OTP as part of the struct ADI\_ROM\_OTP\_BOOT\_INFO structure.

Typically, CGU configuration is performed using an Init Block in the boot stream. This provides greatest flexibility. In situations where boot time must be kept to a minimum, settings in the OTP can be applied at this stage of preboot as opposed to during the boot process itself. When a processor is locked, the boot process does not support an Init Block in the boot stream. For a locked processor programs must use the OTP to reconfigure the clocks without adopting a multi-stage boot strategy.

On release from reset, the CGU is configured for PLL Bypass mode. To improve boot performance, the boot software reconfigures the CGU so that Full-On mode is entered with the default CGU settings.

If the program doesn't provide settings in OTP to configure the Oscillator Watchdog, then the Oscillator Watchdog fault, enabled by default after reset, is disabled prior to reconfiguring the CGU.

If settings are provided in the OTP to configure the Oscillator Watchdog, the fault is left enabled and, if applicable, the CGU is configured per the provided settings. The remainder of the boot process completes in Full-On mode.

The CGU configuration object (see struct ADI\_ROM\_OTP\_BOOT\_CGU\_INFO) allows the boot software to configure the CGU for a more efficient boot process.

Table 46-8: ADI\_ROM\_OTP\_BOOT\_CGU\_INFO Members

| Type     | Name                              | Description                                                     |
|----------|-----------------------------------|-----------------------------------------------------------------|
| uint32_t | ctl_WEN:1 (bitfield)              | Enable write to the CGU_CTL register                            |
| uint32_t | div_WEN:1 (bitfield)              | Enable write to the CGU_DIV register                            |
| uint32_t | reserved0:1 (bitfield)            | Reserved                                                        |
| uint32_t | div_DSEL:5 (bitfield)             | CGU_DIV.DSEL value                                              |
| uint32_t | div_CSEL:5 (bitfield)             | CGU_DIV.CSEL value                                              |
| uint32_t | div_S0SEL:3 (bitfield)            | CGU_DIV.S0SEL value                                             |
| uint32_t | div_SYSSEL:5 (bitfield)           | CGU_DIV.SYSSEL value                                            |
| uint32_t | div_S1SEL:3 (bitfield)            | CGU_DIV.S1SEL value                                             |
| uint32_t | div_OSEL:7 (bitfield)             | CGU_DIV.OSEL value                                              |
| uint32_t | ctl_DF:1 (bitfield)               | CGU_CTL.DF value                                                |
| uint32_t | ctl_MSEL:7 (bitfield)             | CGU_CTL.MSEL value                                              |
| uint32_t | auto_disable:1 (bitfield)         | Disable polling on auto-alignment of clocks (not recommen- ded) |
| uint32_t | Reserved1:6 (bitfield)            | Reserved                                                        |
| uint32_t | clkoutsel_CLKOUTSEL:5 (bitfield)  | CGU_CLKOUTSEL.CLKOUTSEL value                                   |
| uint32_t | clkoutsel_WEN:1 (bitfield)        | Enable write to the CGU_CLKOUTSEL register                      |
| uint32_t | Reserved2:12 (bitfield)           | Reserved                                                        |
| uint32_t | oscwctl0_WEN:1 (bitfield)         | Enable write to the CGU_OSCWDCTL instance 0 register            |
| uint32_t | oscwctl0_HODF:6 (bitfield)        | CGU_OSCWDCTL.HODF value                                         |
| uint32_t | oscwctl0_HODEN:1 (bitfield)       | CGU_OSCWDCTL.HODEN value                                        |
| uint32_t | oscwctl0_CNGEN:1 (bitfield)       | CGU_OSCWDCTL.CNGEN value                                        |
| uint32_t | oscwctl0_BOUF:5 (bitfield)        | CGU_OSCWDCTL.BOUF value                                         |
| uint32_t | oscwctl0_BOUEN:1 (bitfield)       | CGU_OSCWDCTL.BOUEN value                                        |
| uint32_t | oscwctl0_FAULTEN:1 (bitfield)     | CGU_OSCWDCTL.FAULTEN value                                      |
| uint32_t | oscwctl0_MONDIS:1 (bitfield)      | CGU_OSCWDCTL.MONDIS value                                       |
| uint32_t | oscwctl0_FAULTPINDIS:1 (bitfield) | CGU_OSCWDCTL.FAULTPINDIS value                                  |
| uint32_t | reserved3:14 (bitfield)           | Reserved                                                        |
| uint32_t | oscwctl1_WEN:1 (bitfield)         | Enable write to the CGU_OSCWDCTL instance 1 register            |
| uint32_t | oscwctl1_HODF:6 (bitfield)        | CGU_OSCWDCTL.HODF value                                         |
| uint32_t | oscwctl1_HODEN:1 (bitfield)       | CGU_OSCWDCTL.HODEN value                                        |
| uint32_t | oscwctl1_CNGEN:1 (bitfield)       | CGU_OSCWDCTL.CNGEN value                                        |

Table 46-8: ADI\_ROM\_OTP\_BOOT\_CGU\_INFO Members (Continued)

| Type     | Name                              | Description                    |
|----------|-----------------------------------|--------------------------------|
| uint32_t | oscwctl1_BOUF:5 (bitfield)        | CGU_OSCWDCTL.BOUF value        |
| uint32_t | oscwctl1_BOUEN:1 (bitfield)       | CGU_OSCWDCTL.BOUEN value       |
| uint32_t | oscwctl1_FAULTEN:1 (bitfield)     | CGU_OSCWDCTL.FAULTEN value     |
| uint32_t | oscwctl1_MONDIS:1 (bitfield)      | CGU_OSCWDCTL.MONDIS value      |
| uint32_t | oscwctl1_FAULTPINDIS:1 (bitfield) | CGU_OSCWDCTL.FAULTPINDIS value |
| uint32_t | Reserved4:14 (bitfield)           | Reserved                       |

If a CGU\_STAT.WDIVERR , CGU\_STAT.WDFMSERR , CGU\_STAT.LWERR or CGU\_STAT.ADDRERR occurs at the entry to or completion of the configuration routine, the default error handler is called and the boot process terminates.

NOTE: Programs can bypass CGU configuration by setting the RCU\_BCODE.NOPREBOOT bit when this part of the boot process is reached.

## Releasing All Cores from Reset

The control booting core releases all other cores from reset allowing them to then run by default into a safe endless loop. By releasing all other cores from reset, any dedicated core L1 memories become accessible to all cores via the system address space. For one core to load or read data from dedicated L1 memory of another core, the core must be released from the reset state. By default, the other cores in the processor execute a safe endless loop in the boot ROM.

## L1 Memory Initialization

The processor initializes all parity and ECC protected memories to perform subsequent read operations without generating an ECC or parity error.

The L1 Memory Initialization table describes the methods used to initialize the various parity and ECC supported memories on the processor.

Table 46-9: L1 Memory Initialization

| Resource To Fill Memory   | Memory Type      | Address    | Count       | Fill Value   | Flag Set Upon Completion   |
|---------------------------|------------------|------------|-------------|--------------|----------------------------|
| Core 1                    | Core 1 L1 Bank 0 | 0x00048000 | 0x6000 (LW) | 0x00000000   | RCU_MSG.C1L1INIT           |
| Core 1                    | Core 1 L1 Bank 1 | 0x00058000 | 0x6000 (LW) | 0x00000000   | RCU_MSG.C1L1INIT           |
| Core 1                    | Core 1 L1 Bank 2 | 0x00060000 | 0x4000 (LW) | 0x00000000   | RCU_MSG.C1L1INIT           |
| Core 1                    | Core 1 L1 Bank 3 | 0x00070000 | 0x4000 (LW) | 0x00000000   | RCU_MSG.C1L1INIT           |

Table 46-9: L1 Memory Initialization (Continued)

| Resource To Fill Memory   | Memory Type      | Address    | Count       | Fill Value   | Flag Set Upon Completion   |
|---------------------------|------------------|------------|-------------|--------------|----------------------------|
| Core 2                    | Core 2 L1 Bank 0 | 0x00048000 | 0x6000 (LW) | 0x00000000   | RCU_MSG.C2L1INIT           |
| Core 2                    | Core 2 L1 Bank 1 | 0x00058000 | 0x6000 (LW) | 0x00000000   | RCU_MSG.C2L1INIT           |
| Core 2                    | Core 2 L1 Bank 2 | 0x00060000 | 0x4000 (LW) | 0x00000000   | RCU_MSG.C2L1INIT           |
| Core 2                    | Core 2 L1 Bank 3 | 0x00070000 | 0x4000 (LW) | 0x00000000   | RCU_MSG.C2L1INIT           |

NOTE: The Memory Initialization process is enabled by default and may be optionally bypassed by setting the RCU\_BCODE.NOMEMINIT bit.

## Default Entry Point

The first instruction executed for the core in the boot ROM is a read of the RCU\_SVECT0 register. The boot ROM then vectors to that location.

The Default Entry Point table defines the default entry point for each core in the processor

Table 46-10: Default Entry Point

|   Core ID | Register   | Entry Point   |
|-----------|------------|---------------|
|         0 | RCU_SVECT0 | 0x00000000    |
|         1 | RCU_SVECT1 | 0x00500004    |
|         2 | RCU_SVECT2 | 0x00500004    |

The boot code does not set any initial default application entry points by writing the entry point to the RCU\_SVECT0 register in the processor. It will hold the default values of the power on reset case. The RCU\_SVECT0 register is updated at the end of the boot process. For the secure boot, the register is updated only after the successful authentication of the boot stream.

## NO-BOOT Processing

No-Boot mode is executed when selected by the SYS\_BMODE[n] pins. The boot mode is intended as a recovery boot mode or for debug purposes. The core simply executes in an endless loop in the boot ROM, terminating further execution of the boot process.

This boot mode is primarily intended for debug sessions when no boot source can be configured. It allows for a debugger to safely connect to the device and take control (assuming the debugger has been granted access rights as defined by the processor security implementation).

NOTE: NO-BOOT processing is usually only entered because of the boot mode pin sampling that results in execution of the No-Boot boot mode. This processing can also be optionally enabled by setting RCU\_BCODE.HALT field. The RCU\_BCODE.HALT setting can be especially useful for debug sessions to force the execution of the NO-BOOT mode regardless of the SYS\_BMODE[n] state allowing a user

application to be loadedusing the debug tools without fear of the image being corrupted because of attempting to boot through another source.

## SYS\_RESOUT Signal

To signal to tan external system that the processor is in a configured state and ready to start the boot process, the boot software de-asserts the SYS\_RESOUT pinusing the RCU\_CTL.RSTOUTDSRT bit.

## DMC Configuration

To boot to external DDR memories (to support booting to external memory when the processor is locked) the DMC must be configured.

Typically, DMC configuration is done using the Init Block in the boot stream. When the processor is locked the boot process does not support an Init Block in the boot stream. For a locked processor, programs must use the OTP to configure the DMC without adopting a multi-stage boot strategy.

Table 46-66 ADI\_ROM\_OTP\_DMC\_CONFIG Members provides details of the OTP region that is used to store the DMC configuration. In the table, the ADI\_ROM\_OTP\_BOOT\_CFG::dmcEn must be set for the settings to apply.

Table 46-11: ADI\_ROM\_OTP\_DMC\_CONFIG Members

| Type     | Name                                 | Description                     |
|----------|--------------------------------------|---------------------------------|
| uint32_t | ulDDR_DLLCTLCFG:16 (bit field)       | Content of DMCCFG[15:0]         |
| uint32_t | ulDDR_DLLCTLCFG:16 (bit field)       | Content of DMCDLL CTL[15:0]     |
| uint32_t | ulDDR_EMR2EMR3:16 (bit field)        | Contents of DMC_EMR3 [15:0]     |
| uint32_t | ulDDR_EMR2EMR3:16 (bit field)        | Contents of DMC_EMR2 [15:0]     |
| uint32_t | ulDDR_CTL                            | Content of DMCCTL[31:0]         |
| uint32_t | ulDDR_MREMR1:16 ( bit field)         | Content of DMCEMR1[15:0]        |
| uint32_t | ulDDR_MREMR1:16 ( bit field)         | Content of DMCMR[15:0]          |
| uint32_t | ulDDR_TR0                            | Content of DMC_TR0[31:0]        |
| uint32_t | ulDDR_TR1                            | Content of DMC_TR1[31:0]        |
| uint32_t | ulDDR_TR2                            | Content of DMC_TR2[31:0]        |
| uint32_t | ulDDR_ZQCTL0                         | Content of DMCPHYZQCTL0[31:0]   |
| uint32_t | ulDDR_ZQCTL1                         | Content of DMCPHYZQCTL1[31:0]   |
| uint32_t | ulDDR_ZQCTL2                         | Content of DMCPHYZQCTL2[31:0]   |
| uint32_t | ulDDRPHY_CACTL                       | Content of DMCPHYCA_CTL [31:0]  |
| uint32_t | uBypassDelay_LANE0CTL1:6 (bit field) | Content of DMCLANE0_CTL1[15:10] |
| uint32_t | uBypassDelay_LANE1CTL1:6 (bit field) | Content of DMCLANE1_CTL1[15:10] |

Table 46-11: ADI\_ROM\_OTP\_DMC\_CONFIG Members (Continued)

| Type     | Name                                 | Description                     |
|----------|--------------------------------------|---------------------------------|
| uint32_t | uBypassDelay_LANE0CTL0:6 (bit field) | Content of DMCLANE0_CTL0[15:10] |
| uint32_t | uBypassDelay_LANE1CTL0:6 (bit field) | Content of DMCLANE1_CTL0[15:10] |
| uint32_t | reserved0:8 (bit field)              | Reserved                        |

There is an additional single bit located in OTP , ADI\_ROM\_OTP\_BOOT\_CFG::dmcInv allowing programs to invalidate the DMC settings stored in OTP .

NOTE: Once ADI\_ROM\_OTP\_BOOT\_CFG::dmcInv has been set in OTP there is no means to configure the DMC during the preboot phase.

The configuration of the DMC is bypassed if the RCU\_BCODE.NOPREBOOT bit is set when this part of the boot process is reached.

## Bypassing the Boot Process

The boot process can be bypassed allowing a program to start execution from the address stored in the core's soft vector register. This is useful when working in emulation sessions as it provides a mechanism to be able to execute programs directly from an accessible memory that already contains executable code.

To bypass the boot process, set the RCU\_BCODE.NOKERNEL bit. The processor core vectors to the address stored in the core's corresponding RCU\_SVECT0 / RCU\_SVECT1 / RCU\_SVECT2 register instead of calling the required boot mode.

This feature is used along with other features such as disabling memory initialization.

## Boot Mode Disable

Specific boot modes can be permanently disabled via OTP . If the disabled boot mode is enabled using the SYS\_BMODE[n] pins, a boot error is generated.

A byte of storage is provided in the OTP to disable up to eight boot modes. The boot mode disable field can be programmed using the adi\_rom\_otp\_pgm() routine. The otp\_data::bootModeDisable member is used in the program operation to disable the various boot modes.

Table 46-12: Boot Mode Disable

|   otp_data::bootModeDisable Bit Position | Corresponding Boot Mode   |
|------------------------------------------|---------------------------|
|                                        0 | SPI Controller Boot Mode  |
|                                        1 | SPI Target Boot Mode      |
|                                        2 | UART Target Boot Mode     |
|                                        3 | Linkport Target Boot Mode |

Table 46-12: Boot Mode Disable (Continued)

| otp_data::bootModeDisable Bit Position   | Corresponding Boot Mode   |
|------------------------------------------|---------------------------|
| 4                                        | OSPI Controller Boot Mode |
| 7-5                                      | Reserved                  |

## Boot Command Customization

Boot command customization allows permanent customization of a particular boot mode. For example, it is possible to change the peripheral instance used for boot operation.

Storage is provided in OTP for a command item for each supported boot mode.

Storage is provided in the struct ADI\_ROM\_OTP\_BOOT\_CMD\_INFO member of struct ADI\_ROM\_OTP\_BOOT\_INFO . Refer to the corresponding boot modes boot command description for details on supported command options.

NOTE: Before programming boot command to the OTP , evaluate the boot command using the adi\_rom\_Boot() API and ensure that the boot command provides the desired functionality. Once the command is programmed to OTP , it cannot be reverted to original default settings.

## Boot Mode Specific SPU Configuration

Prior to performing actual boot process, the processors SPU resources specific to the boot mode selected are configured. This is performed in the preboot phase as opposed to within the boot mode itself when calling the boot API, as it isolates the security functionality of the processor allowing it to be handled specifically by a separate process.

The following additional SPU resources are configured as secure controllers according to the boot mode selected.

Table 46-13: Boot Mode Specific SPU Configuration

| Boot Mode                                                            | SPU Endpoint ID   | Controller Name                                                            |
|----------------------------------------------------------------------|-------------------|----------------------------------------------------------------------------|
| SPI Controller Boot (Memory- Mapped Mode) and OSPI Con- troller Boot | 146, 147          | MDMA0 Source DMAChannel, MDMA0 Destination DMAChan- nel                    |
| SPI Controller Boot (Peripheral Mode)                                | 100, 98, 96, 94   | SPI3 Receive DMA, SPI2 Receive DMA, SPI1 Receive DMA, SPI0 Re- ceiveDMA    |
| SPI Target Boot                                                      | 100, 98, 96, 94   | SPI3 Receive DMA, SPI2 Receive DMA, SPI1 Receive DMA, SPI0 Re- ceiveDMA    |
| UART Target Boot                                                     | 79, 81, 83, 85    | UART 0 Receive DMA, UART1 Receive DMA, UART2 Receive DMA, UART3 ReceiveDMA |
| LINKPORT Target Boot                                                 | 5, 6              | LINKPORT 0 DMA, LINKPORT1DMA                                               |

NOTE: In the Boot Mode Specific SPU Configuration table, for a given boot mode, all the SPU resources are not configured. Only a single peripheral instance is enabled according the peripheral instance selected for boot.

For example if the boot command for the boot mode indicates boot from UART0 only the UART0 Receive DMA is configured, the other UART Receive DMAs are not configured for secure access.

## Executing the Boot Mode

The boot mode is called adi\_rom\_Boot() routine. The routine fetches and processes the boot stream from the configured boot source.

The table provides default parameters passed to each of the supported boot modes. For details on the API usage, refer to adi\_rom\_Boot().

Table 46-14: Default Boot ROM API Parameters

| Boot Mode            | pAddress   | flags      | blockCount   | pHook                             | Command    |
|----------------------|------------|------------|--------------|-----------------------------------|------------|
| No Boot              | 0x00000000 | 0x00000000 | 0x00000000   | Points to an empty routine in ROM | 0x00000000 |
| SPI Controller Boot  | 0x60000000 | 0x00040000 | 0x00000000   | Points to an empty routine in ROM | 0x00000207 |
| SPI Target Boot      | 0x00000000 | 0x00000000 | 0x00000000   | Points to an empty routine in ROM | 0x00000212 |
| UART Target Boot     | 0x00000000 | 0x00000000 | 0x00000000   | Points to an empty routine in ROM | 0x00000013 |
| LINKPORT Target Boot | 0x00000000 | 0x00000000 | 0x00000000   | Points to an empty routine in ROM | 0x00000014 |
| OSPI Controller Boot | 0x60000000 | 0x00000000 | 0x00000000   | Points to an empty routine in ROM | 0x00000008 |
| Reserved             | 0x00000000 | 0x00000000 | 0x00000000   | Points to an empty routine in ROM | 0x00000000 |

The hook function installed using pHook on this product does not perform any additional configuration.

NOTE: For SPI controller boot and OSPI controller boot modes, start address (pAddress) is set to 0x60000000, This corresponds to the first memory location in the respective flash device (0x0).

## Boot Modes

The boot implementation provides built-in support for booting from various peripherals.

The Booting Modes table describes the supported boot modes.

In target boot mode, the processor functions as a target to any host device. In these modes, the host device usually controls the processor SYS\_HWRST input. Typically, the host applies the reset sequence and waits until the processor is ready to boot, depending on the peripheral in use, and transmits the boot stream data to the processor. Handshake signals are used to signal to the host that the processor is ready to accept more data.

In controller boot mode, the processor controls the peripheral and requests data via the peripheral as and when required.

Individual boot modes can be disabled. For more information about disabling boot modes, see Boot ROM OTP Customizations.

Table 46-15: Booting Modes

| SYS_BMODE[2:0]   | Boot Source          | Description                                                                                                                                      |
|------------------|----------------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| 000              | No Boot              | The processor does not boot. Rather the boot kernel executes some of the preboot operations then enters an endless WFI / IDLE state.             |
| 001              | SPI Controller Boot  | Boot from integrated Flash memory through the SPI2 peripheral configured for memory-mapped mode.                                                 |
| 010              | SPI Target Boot      | Boot through the SPI2 peripheral configured as a target.                                                                                         |
| 011              | UART Boot            | Boots through UART0 configured as a target receiver.                                                                                             |
| 100              | LINKPORT Boot        | Boot through LINKPORT0 peripheral configured as a target receiver. This boot mode is only applicable to derivatives supporting the LP0 instance. |
| 101              | OSPI Controller Boot | Boot from integrated Flash memory through the OSPI peripheral configured for memory-mapped mode.                                                 |
| 110, 111         | Reserved             | Reserved                                                                                                                                         |

## No-Boot Mode

No-Boot mode is intended for device recovery purposes caused due to incorrect programming of the boot source memory allowing for target connection through an emulator. Emulation tools can also leverage the No-Boot functionality allowing for debug sessions to run the preboot software prior to loading an application while preventing the boot process from continuing and clobbering data loaded by the emulator.

This boot mode results in several preboot operations being performed before placing the core in a safe, endless loop located in the boot ROM.

For a complete list of operations performed when No-Boot is selected, see Preboot Operations. The core terminates at the NO-BOOT Processing stage of the preboot process.

## SPI Master Boot Mode

The SPI master boot routine provides support for booting the processor from SPI flash memories. The SPI boot mode uses a device auto-detection feature that is enabled by default. This lets the boot stream itself instruct updates to the SPI configuration and the read command used allowing for more efficient transactions.

NOTE: For default SPI master mode, the peripheral fault, SPI2\_ERR with fault ID 132 is enabled.

## Boot From External SPI Flash Devices

The SPI boot mode supports booting from 24-bit or 32-bit addressable flash devices. The boot mode uses the MDMA channels by default and configures the SPI flash for memory-mapped functionality. Peripheral DMA mode is also supported when calling the boot mode via adi\_rom\_Boot() .

When auto-device detection is enabled, the SPI memory is initially read using the standard 0x03 SPI read command with a reduced clocking frequency for maximum compatibility. The first nibble of the boot stream is then used to reconfigure the SPI interface and possible the SPI flash. Refer to SPI Device Detection Routine.

NOTE: Support for automatic device detection via the first nibble of the boot stream is not supported when booting secure boot streams. Instead when signing the boot image an attribute can be set in the image header that specifies the configuration to use.

For booting, the SPI memory is connected as shown in the SPI Memory Connections figure.

Figure 46-2: SPI Memory Connections

<!-- image -->

The pull-up resistor on the slave select signal ensures that the memory is deselected when the pin is in a high-impedance mode such as during reset.

Initialization codes are allowed to manipulate the ADI\_ROM\_BOOT\_CONFIG::dBootCommand to extend boot mechanism to a second SPI memory connected to another slave select pin. Updating the field that specifies the slave select signal for use, allows the boot process to manage larger boot streams than fit into a single SPI device.

NOTE: If modifying the slave select signal used during the boot process, configure the pin multiplexing to enable the correct functionality for the pin. Once the boot process has proceeded past the configuration function and the boot process has actually started, the boot kernel will not perform any further pin multiplexing operations.

## SPI Device Detection Routine

Since the boot mode supports booting from various SPI memories, the boot kernel automatically detects what type of memory is connected. To determine whether the SPI memory device requires a 24-bit or 32-bit addressing scheme, the boot kernel performs a device detection sequence prior to booting. The SPI\_MISO signal requires a pull-up resistor. The routine relies on the fact that memories do not drive their data outputs unless the right number of address bytes are received.

Initially, the boot kernel transmits the read command on the SPI\_MOSI line. Once the command has been sent, the boot kernel proceeds to transmit a single address byte and waits until the receive FIFO indicates that the buffer is no longer empty. The first received byte is discarded. The boot code then proceeds to issue another address byte while simultaneously receiving a byte. The process continues until a non 0xFF or 0x00 byte is received or until the full 4 address bytes is sent without any valid data being returned.

The receiving of a non 0x00 or 0xFF byte tells the boot code whether the memory device requires 24 or 32 address bits. The lower nibble of the received byte is then used to further customize the boot mode. This nibble is referred to as the BCODE . The boot code applies settings to the SPI peripheral according to the SPI Master Boot BCODE Descriptions .

If the received value equals 0x00 or 0xFF , it is assumed that the memory device has not driven its data output thus, another zero byte is transmitted and the received data is tested again.

If the value still equals 0xFF , device detection continues. Device detection aborts immediately when a byte different than 0xFF is received. The boot process continues with normal boot operation and it reissues a command to again read from address 0. Two read sequences load the first block header. Separate read sequences load further block headers and block payload fields.

The SPI Device Detection Principle figure illustrates how individual devices behave.

Figure 46-3: SPI Device Detection Principle

<!-- image -->

Table 46-16: SPI Master BCODE Configuration Lookup Table

| Member         | BCODE                                | BCODE                                | BCODE                                | BCODE                                         | BCODE                                     |
|----------------|--------------------------------------|--------------------------------------|--------------------------------------|-----------------------------------------------|-------------------------------------------|
| Member         | 0                                    | 1                                    | 2                                    | 3                                             | 4                                         |
| Transfer Type  | Single bit command, address and data | Single bit command, address and data | Single bit command, address and data | Single bit command, address and dual bit data | Single bit command, address and Quad data |
| ubDummyBytes   | 0x00                                 | 0x00                                 | 0x01                                 | 0x01                                          | 0x01                                      |
| ubReadCommand  | 0x03                                 | 0x03                                 | 0x0B                                 | 0x3B                                          | 0x6B                                      |
| ubDataBits     | 0x00                                 | 0x00                                 | 0x00                                 | 0x01                                          | 0x02                                      |
| ubAddressBytes | 0x03                                 | 0x03                                 | 0x03                                 | 0x03                                          | 0x03                                      |
| uwClkLower     | 0x000F                               | 0x000F                               | 0x0001                               | 0x0001                                        | 0x0001                                    |
| uReserved0     | 0x0000                               | 0x0000                               | 0x0000                               | 0x0000                                        | 0x0000                                    |
| nTxCtl         | 0x00000003                           | 0x00000003                           | 0x00040003                           | 0x00040033                                    | 0x00040033                                |

Table 46-16: SPI Master BCODE Configuration Lookup Table (Continued)

| Member         | BCODE      | BCODE      | BCODE      | BCODE      | BCODE      |
|----------------|------------|------------|------------|------------|------------|
|                | 0          | 1          | 2          | 3          | 4          |
| nRxCtl         | 0x00000003 | 0x00000003 | 0x00040003 | 0x00140003 | 0x00240033 |
| nCmdCtl        | 0x00000003 | 0x00000003 | 0x00000003 | 0x00000033 | 0x00000033 |
| pMIOEnFunction | 0x00000000 | 0x00000000 | 0x00000000 | 0x00000000 | 0x00000000 |
| nDummy         | 0x00       | 0x00       | 0x00       | 0x00       | 0x00       |

Table 46-17: SPI Master BCODE Configuration Lookup Table

| Member           | BCODE                                           | BCODE                                            | BCODE                                            | BCODE                                            | BCODE                                            | BCODE                                            |
|------------------|-------------------------------------------------|--------------------------------------------------|--------------------------------------------------|--------------------------------------------------|--------------------------------------------------|--------------------------------------------------|
| Member           | 5                                               | 6                                                | 7                                                | 8                                                | 9                                                | A                                                |
| Transfer Type    | Single bit com- mand, Dual bit address and data | Single bit com- mand , Dual bit address and data | Single bit com- mand , Dual bit address and data | Single bit com- mand , Quad bit address and data | Single bit com- mand , Quad bit address and data | Single bit com- mand , Quad bit address and data |
| ubDummyBytes     | 0x01                                            | 0x02                                             | 0x03                                             | 0x02                                             | 0x03                                             | 0x05                                             |
| ubReadCom- mand  | 0xBB                                            | 0xBB                                             | 0xBB                                             | 0xEB                                             | 0xEB                                             | 0xEB                                             |
| ubDataBits       | 0x01                                            | 0x01                                             | 0x01                                             | 0x02                                             | 0x02                                             | 0x02                                             |
| ubAddressBytes   | 0x03                                            | 0x03                                             | 0x03                                             | 0x03                                             | 0x03                                             | 0x03                                             |
| uwClkLower       | 0x0001                                          | 0x0001                                           | 0x0001                                           | 0x0001                                           | 0x0001                                           | 0x0001                                           |
| uReserved0       | 0x0000                                          | 0x0000                                           | 0x0000                                           | 0x0000                                           | 0x0000                                           | 0x0000                                           |
| nTxCtl           | 0x00140033                                      | 0x00140033                                       | 0x00140033                                       | 0x00240033                                       | 0x00240033                                       | 0x00240033                                       |
| nRxCtl           | 0x00140033                                      | 0x00140033                                       | 0x00140033                                       | 0x00240033                                       | 0x00240033                                       | 0x00240033                                       |
| nCmdCtl          | 0x00000033                                      | 0x00000033                                       | 0x00000033                                       | 0x00000033                                       | 0x00000033                                       | 0x00000033                                       |
| pMIOEnFunc- tion | 0x00000000                                      | 0x00000000                                       | 0x00000000                                       | 0x00000000                                       | 0x00000000                                       | 0x00000000                                       |
| nDummy           | 0x00                                            | 0x00                                             | 0x00                                             | 0x00                                             | 0x00                                             | 0x00                                             |

NOTE: For the above configurations, the addressing scheme can be 3-bytes or 4-bytes depending on the addressing of flash detected in auto-detection. The SPI mode byte issued for all the SPI master peripheral based configurations is 0x00. The mode byte is the first byte transmitted after the address cycles and is used to control the continuous read mode functionality in which the next read operation is not required to issue a command cycle. Continuous read mode is not supported during the boot process.

## Supported Quad Mode Enable Methods

The boot ROM does not support enabling quad mode on the SPI flash device. To boot in quad mode, the flash device must be configured outside the boot ROM.

NOTE: It is more beneficial to boot initially in Dual mode and use an initcode to enable Quad mode.

## Run-Time API

The following table provides descriptions of the adi\_rom\_Boot() command parameter.

Table 46-18: SPI Master Boot Command Descriptions

| Bits   | Name                  | Setting      | Description                                                                                                                           |
|--------|-----------------------|--------------|---------------------------------------------------------------------------------------------------------------------------------------|
| 31:28  | ROM_BCMD_SPIM_SPEED   | 0 to 1111    | SPI clock divider. This value written to the SPI peripherals clock divider register                                                   |
| 27     | Reserved              | Reserved     | Reserved                                                                                                                              |
| 26:22  | ROM_BCMD_SPIM_DUMMY   | 00000        | No dummy bytes required after the address                                                                                             |
| 26:22  | ROM_BCMD_SPIM_DUMMY   | 00001        | 1 dummy byte required after the address                                                                                               |
| 26:22  | ROM_BCMD_SPIM_DUMMY   | ....         | .....                                                                                                                                 |
| 26:22  | ROM_BCMD_SPIM_DUMMY   | 11111        | 31 dummy bytes required after the address                                                                                             |
| 21:20  | ROM_BCMD_SPIM_ADDR    | 00           | Flash device requires an 8-bit address                                                                                                |
| 21:20  | ROM_BCMD_SPIM_ADDR    | 01           | Flash device requires a 16-bit address                                                                                                |
| 21:20  | ROM_BCMD_SPIM_ADDR    | 10           | Flash device requires a 24-bit address                                                                                                |
| 21:20  | ROM_BCMD_SPIM_ADDR    | 11           | Flash device requires a 32-bit address                                                                                                |
| 19:16  | ROM_BCMD_SPIM_BCODE   | 0000 to 1111 | Boot mode-specific code. Specifies the boot mode-specific code that can further customize and control the boot proc- ess.             |
| 15     | ROM_BCMD_SPIM_CMD     | 0            | Issue the read command over the single bit bus                                                                                        |
| 15     | ROM_BCMD_SPIM_CMD     | 1            | Issue the read command over the multi-bit bus. Not recommended for use in ADSP-2159x processor.                                       |
| 14:12  | ROM_BCMD_SPIM_SSEL    | 000          | Use slave select 1 for SPI chip select                                                                                                |
| 14:12  | ROM_BCMD_SPIM_SSEL    | 001          | Use slave select 2 for SPI chip select                                                                                                |
| 14:12  | ROM_BCMD_SPIM_SSEL    | 010          | Use slave select 3 for SPI chip select                                                                                                |
| 14:12  | ROM_BCMD_SPIM_SSEL    | 011          | Reserved                                                                                                                              |
| 14:12  | ROM_BCMD_SPIM_SSEL    | 100          | Reserved                                                                                                                              |
| 14:12  |                       | 101          | Reserved                                                                                                                              |
| 14:12  |                       | 110          | Reserved                                                                                                                              |
| 14:12  |                       | 111          | Reserved                                                                                                                              |
| 11:8   | ROM_BCMD_SPIM_DEVENUM | 0 to 16      | Boot peripheral enumeration. So, for SPI3 it is set to 3, for SPI2 it is set to 2, for SPI0 it is set to 0. 0x4 to 0xF is re- served. |
| 7      | Reserved              | Reserved     | Reserved                                                                                                                              |

Table 46-18: SPI Master Boot Command Descriptions (Continued)

| Bits   | Name                 | Setting      | Description                                                                                                                                                                      |
|--------|----------------------|--------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6      | ROM_BCMD_SPIM_NOAUTO | 0            | Automatic device detection disable. Perform automatic de- vice detection and peripheral configuration based on the BCODE value (first nibble) of the boot streams block head- er |
| 6      | ROM_BCMD_SPIM_NOAUTO | 1            | Do not perform automatic device detection                                                                                                                                        |
| 5      | ROM_BCMD_SPIM_NOCFG  | 0            | Device configuration enable. Instructs the config routine to perform a pin-muxing and full peripheral configuration                                                              |
| 5      | ROM_BCMD_SPIM_NOCFG  | 1            | Device configuration disable.                                                                                                                                                    |
| 4      | ROM_BCMD_SPIM_HOST   | 0            | Master boot mode enable.                                                                                                                                                         |
| 4      | ROM_BCMD_SPIM_HOST   | 1            | Slave boot mode enable.                                                                                                                                                          |
| 3:0    | ROM_BCMD_SPIM_DEVICE | 0000         | Reserved                                                                                                                                                                         |
| 3:0    | ROM_BCMD_SPIM_DEVICE | 0001         | Reserved                                                                                                                                                                         |
| 3:0    | ROM_BCMD_SPIM_DEVICE | 0010         | SPI Boot (Legacy peripheral DMA)                                                                                                                                                 |
| 3:0    | ROM_BCMD_SPIM_DEVICE | 0011         | Reserved                                                                                                                                                                         |
| 3:0    | ROM_BCMD_SPIM_DEVICE | 0100         | Reserved                                                                                                                                                                         |
| 3:0    | ROM_BCMD_SPIM_DEVICE | 0101         | Reserved                                                                                                                                                                         |
| 3:0    | ROM_BCMD_SPIM_DEVICE | 0110         | Reserved                                                                                                                                                                         |
| 3:0    | ROM_BCMD_SPIM_DEVICE | 0111         | SPI Memory Mapped Boot                                                                                                                                                           |
| 3:0    | ROM_BCMD_SPIM_DEVICE | 1000 to 1111 | Reserved                                                                                                                                                                         |

NOTE: All bits in the above table that are not defined must be set to zero. Supported features may be limited depending on peripheral instance.

NOTE: SPI master secure boot is only supported in memory mapped mode. SPI2 is the default SPI instance that operates in memory mapped mode. To boot from an SPI instance other than SPI2, ensure that the desired SPI instance supports memory-mapped mode. The ROM\_BCMD\_SPIM\_DEVICE field in the SPI master boot command parameter must be set to 0x7 in OTP or passed as an argument in the ROM API call.

## SPI Slave Boot Mode

When using SPI slave mode boot, the processor consumes boot data from an external SPI host device. This mode supports single, dual, and quad-bit modes. The boot kernel always starts in single bit mode and can be changed using the appropriate command. The following figures show the hardware configuration for the modes. As in all slave boot modes, the host device controls the SYS\_HWRST input of the processor.

NOTE: Secure Boot Stream Padding

For slave boot modes, the host must always send data in multiples of 1024 bytes. This requirement is due to the sizing of internal buffers used for DMA.

<!-- image -->

Figure 46-4: Connection between Host (SPI Master) and Processor (SPI Slave)

Figure 46-5: Connection between Host (SPI Master) and Processor (SPI Slave) DIOM

<!-- image -->

Figure 46-6: Connection between Host (SPI Master) and Processor (SPI Slave) QSPI

<!-- image -->

The host drives the SPI clock and is responsible for timing. The host must provide an active-low chip select signal that connects to the processor's SPIx\_SS input signal with each byte transferred or remain low during the entire procedure. 8-bit data is expected and 16-bit mode is not supported.

In SPI slave boot mode, the boot kernel sets the SPI\_CTL.CPHA bit and clears the SPI\_CTL.CPOL bit. Therefore the SPI\_MISO pin is latched on the falling edge of the SPI\_MOSI pin.

The SPI slave processor detects the correct boot mode from the host SPI device by reading the first byte sent, defined as SPICMD . The SPICMD Descriptions table describes the available codes. These additional bytes must be sent prior to transmitting the data to configure the SPI device.

The SPICMD Descriptions table describes the following:

- Host starting in single bit mode
- Host starting in a mode other than single bit

Table 46-19: SPICMD Descriptions

| SPICMD                                | Description             |
|---------------------------------------|-------------------------|
| If host starts in Single bit Mode     |                         |
| 0x3                                   | Keep single-bit mode    |
| 0x7                                   | Switch to dual-bit mode |
| 0xB                                   | Switch to quad-bit mode |
| If host device starts in DIOM or QSPI |                         |
| 0xAA,0xBF                             | Switch to dual-bit mode |
| 0xEE,0xEE,0xFE,0xFF                   | Switch to quad-bit mode |

In SPI slave boot mode, SPIx\_RDY functionality is critical. The SPIx\_RDY output is used for back pressure and requires a pulling resistor. The boot code requires the SPIx\_RDY signal function as active-low. The host is only permitted to transfer data when SPIx\_RDY is in the active state. This functionality allows the processor to hold off the host while the processor is in reset or executing the pre-boot and processor initialization sequences. The SPI is configured to deassert SPIx\_RDY when the receive FIFO is filled to 75% or more.

NOTE: For default SPI slave mode, the peripheral fault, SPI2\_ERR with Fault ID 132 is enabled.

The SPI Program Flow on the Host Side figure illustrates the required program flow on the host side.

d

d

d

Figure 46-7: SPI Program Flow on the Host Side

<!-- image -->

## Run-time API

The SPI slave boot mode can be called through the Boot Routine API function at run-time. Initiating a boot through the run-time API allows for extra customization such as disabling automatic device configuration or specifying a different SPI device other than the default.

When ROM\_BCMD\_NOCFG flag is specified, it is necessary to program pin multiplexing and other SPI configuration as required, while keeping the SPI\_CTL.EN bit cleared.

The ROM\_BCMD\_NOAUTO flag can suppress auto mode detection. In that case, the desired configuration must be passed through the ROM\_BCMD\_SPIS\_BCODE bit field, even if the ROM\_BCMD\_NOCFG flag is set.

The following table provides descriptions of the adi\_rom\_Boot parameter.

Table 46-20: SPI Slave Boot Command Bit Descriptions

| Bit No. (Access)   | Bit Name             | Description/Enumeration                                                                                                                                                                   |
|--------------------|----------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19:16              | ROM_BCMD_S PIS_BCODE | Boot Mode Specific BCODE. Specifies the boot mode-specific code that can further customize and control the boot process.                                                                  |
| 19:16              | ROM_BCMD_S PIS_BCODE | 00xxb Single bit SPI bus                                                                                                                                                                  |
| 19:16              | ROM_BCMD_S PIS_BCODE | 01xxb Dual SPI bus                                                                                                                                                                        |
| 19:16              | ROM_BCMD_S PIS_BCODE | 10xxb Quad SPI bus                                                                                                                                                                        |
| 19:16              | ROM_BCMD_S PIS_BCODE | 11xxb Reserved                                                                                                                                                                            |
| 11:8               | ROM_BCMD_ DEVENUM    | Device enumeration. Specifies the SPI device to use.                                                                                                                                      |
| 11:8               | ROM_BCMD_ DEVENUM    | 0x0 SPI0                                                                                                                                                                                  |
| 11:8               | ROM_BCMD_ DEVENUM    | 0x1 SPI1                                                                                                                                                                                  |
| 11:8               | ROM_BCMD_ DEVENUM    | 0x2 SPI2                                                                                                                                                                                  |
| 11:8               | ROM_BCMD_ DEVENUM    | 0x3 SPI3                                                                                                                                                                                  |
| 11:8               | ROM_BCMD_ DEVENUM    | 0x4 - 0xF Reserved                                                                                                                                                                        |
| 6                  | ROM_BCMD_ NOAUTO     | Automatic device detection disable. When set, this bit disables automatic device detection and uses the setting provided in the other fields of this register to configure the boot mode. |
| 5                  | ROM_BCMD_ NOCFG      | Device configuration disable. When set, this bit disables device configuration. Device configuration includes reconfiguration of the peripherals MMRregisters and device pin muxing.      |
| 4                  | ROM_BCMD_ HOST       | Host boot mode enable. When set, enables SPI slave boot mode. Otherwise, use the master boot mode.                                                                                        |
| 3:0                | ROM_BCMD_ DEVICE     | Boot source device. Specifies the device to boot from.                                                                                                                                    |
| 3:0                | ROM_BCMD_ DEVICE     | 0x2 SPI                                                                                                                                                                                   |

NOTE: All bits in the table that are not defined must be set to zero. Supported features may be limited depending on peripheral instance.

## Link Port Target Boot Mode

This section describes booting from the link port with the processor as a target.

Link port boot is a target boot mode in which the processor receives boot data from an external link port controller through link port 0. The link port is configured for receive mode and all transfers from the link port to memory are performed under the control of DMA. The maximum supported operating frequency of the link port is for which

the controller boot source is responsible for deriving the clock frequency. The link port receiver operates at an asynchronous frequency up to the maximum supported operating frequency.

The link port protocol supports a way to generate link port transmit and receive service requests. The transmit service request is generated on the processor to transmit data when the transmitter is disabled. The receiver drives the LACKx signal high to initiate this activity. The receive service request is generated on a receiver when it is disabled. The transmitter drives the LCLKx signal high to initiate this activity.

Because the transmitter and receivers can be enabled at different times, external pull-down resistors are required on both the LCLKx and LACKx signals to eliminate any false service request assertions.

The link port target boot mode initialization phase waits for the receive service request before passing control back to the main kernel. Once this initial receive service request has been detected, the receiving link port is enabled and the boot process completes. The receiving link port is not disabled again until after boot is complete. Once the link port is enabled, the receive DMA channel controls all transfers. The load function for the link port receive boot mode can then point to the peripheral DMA routine of the main kernel in a similar way to the SPI slave boot mode.

NOTE: For default Link Port target mode, the peripheral DMA fault, LP0\_DMA\_ERR with fault ID 252 is enabled.

## Run-time API

The Link Port target boot mode can be called through the boot routine API function at run time. The run-time API allows for more customization. Both device auto-detection and device configuration can be disabled, and a device other than the default LINKPORT0 can be specified.

If ROM\_BCMD\_NOCFG flag is specified, it is the programs responsibility to configure pin multiplexing as required.

The following table provides descriptions of the adi\_rom\_Boot parameter.

Table 46-21: LINKPORT Target Boot command Bit Descriptions

| Bit No. (Access)   | Bit Name          | Description/Enumeration                                                                                                                                                              |
|--------------------|-------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:8               | ROM_BCMD_ DEVENUM | Device enumeration. Specifies the LINKPORT device to use.                                                                                                                            |
| 11:8               | ROM_BCMD_ DEVENUM | 0x0 LP0                                                                                                                                                                              |
| 11:8               | ROM_BCMD_ DEVENUM | 0x1 LP1                                                                                                                                                                              |
| 11:8               | ROM_BCMD_ DEVENUM | 0x2 - 0xF Reserved                                                                                                                                                                   |
| 6                  | ROM_BCMD_ NOAUTO  | Automatic device detection disable. When set disables automatic device detection and uses the setting provided in the other fields of this register to configure the boot mode.      |
| 5                  | ROM_BCMD_ NOCFG   | Device configuration disable. When set, this bit disables device configuration. Device configuration includes reconfiguration of the peripherals MMRregisters and device pin muxing. |

Table 46-21: LINKPORT Target Boot command Bit Descriptions (Continued)

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                           |
|--------------------|------------------|---------------------------------------------------------------------------------------------------|
| 4                  | ROM_BCMD_ HOST   | Host boot mode enable. When set, enables SPI slave boot. Otherwise, use the controller boot mode. |
| 3:0                | ROM_BCMD_ DEVICE | Boot source device. Specifies the device to boot from.                                            |
|                    |                  | 0x4 LINKPORT                                                                                      |

NOTE: All bits in the above table that are not defined must be set to zero. Supported features may be limited depending on peripheral instance.

## UART Controller Boot Mode

When using UART target mode boot, the processor receives boot data from a UART host device connected to the UART interface. The device connected to UART0 is initially detected using an autobaud detection sequence. After finishing the UART target boot process, all control and status registers of the used resources are restored.

Further customization, such as disabling autobaud detection, and changing the device, use the boot routine API.

During boot operation, the host device usually relies on the RTS output of the UART device. At boot time, the processor does not evaluate RTS signals driven by the host. Since the RTS is in a high impedance state when the processor is in reset, or while executing a pre-boot, an external pull-up resistor to VDDEXT is recommended. The Connection Between Host and Processor figure shows the interconnection required for booting. The figure does not show physical line drivers and level shifters that are typically required to meet the individual UART-compatible standards.

Figure 46-8: Connection Between Host and Processor

<!-- image -->

When the UART is enabled, the RTS immediately transitions low, encouraging the host to send the first boot stream data as shown in the Host Relying on RTS figure. For half-duplex UART connections, the host must avoid this action. The host must wait until it has received the 4 bytes from the target processor before sending any data.

Figure 46-9: Host Relying on RTS

<!-- image -->

When the boot kernel is processing fill or Initcode blocks, it can require extra processing time and must delay the host from sending more data. This request is signaled using the RTS output.

The Host Relying on RTS figure shows RTS timing when an extended Initcode routine executes. Since code execution is distracting from the data loading, the host device must be prevented from sending more data. The timing of the RTS depends on the state of the UART\_CTL.RFRT bit. This bit is cleared during UART target boot mode when RTS is de-asserted, the UART receive FIFO contains 4 or more data words, and another start bit is detected.

## NOTE: Secure Boot Stream Padding

For target boot modes, the host must always send data in multiples of 1024 bytes. This requirement is due to the sizing of internal buffers used for DMA.

## Autobaud Detection

The kernel supports autobaud detection using the '@' character as data. The host is expected to have its clock set to a rate supported in the UART.

To determine the bit rate when performing autobaud detection, use the following steps:

1. The boot kernel expects an '@' character (0x40, eight bits data, one start bit, one stop bit, no parity bit) on the UART RXD input.
2. The UART\_CLK.EDBO and the UART\_CLK register is cleared.
3. The boot kernel acknowledges, and the host then downloads the boot stream. The acknowledgment consists of 4 bytes: 0xBF , UART\_CLK [15:8], UART\_CLK [7:0], 0x00.
4. The host is requested to not send further bytes until it has received the complete acknowledge string.
5. Once the 0x00 byte is received, the host can send the entire boot stream.

The host knows the total byte count of the boot stream, but it is not required to know the content of the boot stream.

UART0\_RX

UART0\_RX

UART0\_RTS

UART0\_CTS

Figure 46-10: UART Autobaud Detection Waveform

<!-- image -->

The UART Autobaud Detection Waveform figure provides timing information for UART booting. After the bit rate is known, the UART is enabled and the kernel transmits the 4 acknowledge bytes.

NOTE: For default UART target mode, the peripheral DMA fault, UART2\_RXDMA\_ERR with fault ID 252 is enabled.

## Run-time API

The UART target boot mode can be called through the boot routine API function at run time. The run-time API allows for more customization. Both autobaud detection and device configuration can be disabled, and a device other than the default UART0 can be specified.

If ROM\_BCMD\_NOCFG flag is specified, it is the programs responsibility to configure pin multiplexing as required.

Autobaud detection can be suppressed using the ROM\_BCMD\_NOAUTO flag. In this case, the desired configuration can be passed through the ROM\_BCMD\_UART\_CLK bit field. If the ROM\_BCMD\_UART\_CLK bit field is zero, UART\_CLK is evaluated. If a value of 0xFFFF was present, the default error routine of the boot kernel is called and the booting process is aborted. Otherwise, the value in UART\_CLK remains untouched.

The following table provides descriptions of the adi\_rom\_Boot parameter.

Table 46-22: UART Target Boot command Bit Descriptions

| Bit No. (Access)   | Bit Name            | Description/Enumeration                                      |
|--------------------|---------------------|--------------------------------------------------------------|
| 31:16              | ROM_BCMD_ UART_CLK  | UART Clock Divider. When set to zero this field is ignored.  |
| 15                 | ROM_BCMD_ UART_EDBO | UART Clock Divider Mode When set enables EDBO functionality. |
| 11:8               | ROM_BCMD_ DEVENUM   | Device enumeration. Specifies the UART device to use.        |
| 11:8               | ROM_BCMD_ DEVENUM   | 0x0 UART0                                                    |
| 11:8               | ROM_BCMD_ DEVENUM   | 0x1 UART1                                                    |

Table 46-22: UART Target Boot command Bit Descriptions (Continued)

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                                                                                              | Description/Enumeration                                                                                                                                                              |
|--------------------|------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |                  | 0x2                                                                                                                                                                                  | UART2                                                                                                                                                                                |
|                    |                  | 0x3                                                                                                                                                                                  | UART3                                                                                                                                                                                |
|                    |                  | 0x4 - 0xF                                                                                                                                                                            | Reserved                                                                                                                                                                             |
| 6                  | ROM_BCMD_ NOAUTO | Automatic device detection disable. When set disables automatic device detection and uses the setting provided in the other fields of this register to configure the boot mode.      | Automatic device detection disable. When set disables automatic device detection and uses the setting provided in the other fields of this register to configure the boot mode.      |
| 5                  | ROM_BCMD_ NOCFG  | Device configuration disable. When set, this bit disables device configuration. Device configuration includes reconfiguration of the peripherals MMRregisters and device pin muxing. | Device configuration disable. When set, this bit disables device configuration. Device configuration includes reconfiguration of the peripherals MMRregisters and device pin muxing. |
| 4                  | ROM_BCMD_ HOST   | Host boot mode enable. When set, enables SPI slave boot. Otherwise, use the controller boot mode.                                                                                    | Host boot mode enable. When set, enables SPI slave boot. Otherwise, use the controller boot mode.                                                                                    |
| 3:0                | ROM_BCMD_ DEVICE | Boot source device. Specifies the device to boot from.                                                                                                                               | Boot source device. Specifies the device to boot from.                                                                                                                               |
| 3:0                | ROM_BCMD_ DEVICE | 0x3                                                                                                                                                                                  | UART                                                                                                                                                                                 |

NOTE: All bits in the above table that are not defined must be set to zero. Supported features may be limited depending on peripheral instance.

## OSPI Controller Boot Mode

OSPI controller boot supports booting from flash devices via the SPI3 controller. Similar to legacy SPI (SPI0/SPI1/ SPI2), it supports boot from many different flash devices provided by different flash vendors further increasing the range of devices due to support for DDR modes of operation. This interface enables to reduce boot times by using faster modes of operation supported by the OSPI controller including DDR modes of operation to achieve higher transfer rates. Most flash vendors support at least a basic and standard 0x03 read command allowing for support for a single bit SPI flash interface.

The OSPI boot mode uses a device auto-detection feature that is enabled by default. This lets the boot stream itself instruct updates to the SPI configuration and the read command used allowing for more efficient transactions such as for enabling quad bus widths and DDR modes and faster SPI clocks.

## Boot from External SPI Flash Devices

The OSPI boot mode supports booting from 24-bit or 32-bit addressable flash devices. The boot mode uses the MDMA channels, which works with OSPI controller to get data from flash memory in memory mapped mode.

When auto-device detection is enabled, the SPI memory is initially read using the standard 0x03 SPI read command with a reduced clocking frequency for maximum compatibility. The first nibble of the boot stream is then used to reconfigure the SPI interface.

NOTE: Support for automatic device detection via the first nibble of the boot stream is not supported when booting secure boot streams. Instead when signing the boot image an attribute can be set in the image header that specifies the configuration to use.

For booting, the SPI memory is connected as shown in the SPI Memory Connections figure.

Figure 46-11: SPI Memory Connections

<!-- image -->

The pull-up resistor on the target select signal ensures that the memory is deselected when the pin is in a highimpedance mode such as during reset.

Initialization codes are allowed to manipulate the ADI\_ROM\_BOOT\_CONFIG::dBootCommand to extend boot mechanism to a second SPI memory connected to another target select pin. Updating the field that specifies the target select signal for use, allows the boot process to manage larger boot streams than fit into a single SPI device.

NOTE: If modifying the target select signal used during the boot process, configure the pin multiplexing to enable the correct functionality for the pin. Once the boot process has proceeded past the configuration function and the boot process has started, the boot kernel does not perform any further pin multiplexing operations.

## OSPI Device Detection Routine

As boot mode supports booting from various SPI memories, the boot kernel automatically detects what type of memory is connected. To determine whether the SPI memory device requires a 24-bit or 32-bit addressing scheme, the boot kernel performs a device detection sequence prior to booting. The OSPI\_MISO signal requires a pull-up resistor. The routine relies on the fact that memories do not drive their data outputs unless the right number of address bytes are received.

The SPI flash autodetection routine determines the SPI configuration to be used by data in the boot stream. The autodetection first determines the number of address cycles needed to address the flash device.

This operation is performed by configuring the controller in STIG mode initially with the number of expected address bytes programmed to 3 and a basic read command (0x3). A read command from address 0x00000000 with a 3-byte address is issued. If the flash returns a value other than 0x00 or 0xFF , the flash is in 3-byte address mode. Otherwise, the same process is repeated with the STIG controller configured with four address bytes. Only three and four byte address cycles are supported.

The lower nibble of the received byte is then used to further customize the boot mode. This nibble is referred to as the BCODE . The boot code applies settings to the OSPI peripheral according to the OSPI controller Boot BCODE Descriptions .

The boot process continues with normal boot operation and it re-issues a command to re-read from address 0. Two read sequences load the first block header. Separate read sequences load further block headers and block payload fields.

The SPI Device Detection Principle figure illustrates how individual devices behave.

Figure 46-12: SPI Device Detection Principle

<!-- image -->

Table 46-23: OSPI Controller BCODE Configuration Lookup Table

| Member Name                                                        | BCODE                                | BCODE                                | BCODE                                | BCODE                                     | BCODE                                     |
|--------------------------------------------------------------------|--------------------------------------|--------------------------------------|--------------------------------------|-------------------------------------------|-------------------------------------------|
| Member Name                                                        | 0                                    | 1                                    | 2                                    | 3                                         | 4                                         |
| Transfer Type                                                      | Single bit command, address and data | Single bit command, address and data | Single bit command, address and data | Single bit command, dual address and data | Single bit command, dual address and data |
| ubDummyCycles                                                      | 0x00                                 | 0x00                                 | 0x08                                 | 0x04                                      | 0x08                                      |
| ubReadCommand                                                      | 0x03                                 | 0x03                                 | 0x0B                                 | 0xBB                                      | 0xBB                                      |
| ubDataBits                                                         | 0x00                                 | 0x00                                 | 0x00                                 | 0x01                                      | 0x01                                      |
| ubAddressBytes (This field is re- tained. Not used by boot kernel) | 0x03                                 | 0x03                                 | 0x03                                 | 0x03                                      | 0x03                                      |
| uwClkLower                                                         | 0x000F                               | 0x000F                               | 0x0003                               | 0x0003                                    | 0x0003                                    |
| uReserved0                                                         | 0x0000                               | 0x0000                               | 0x0000                               | 0x0000                                    | 0x0000                                    |
| nCfg                                                               | 0x00000081                           | 0x00000081                           | 0x00000081                           | 0x00000081                                | 0x00000081                                |
| nDsr                                                               | 0x00000002                           | 0x00000002                           | 0x00000002                           | 0x00000002                                | 0x00000002                                |
| nDrir                                                              | 0x00000000                           | 0x00000000                           | 0x00000000                           | 0x00011000                                | 0x00011000                                |
| uReserved1                                                         | 0x00000000                           | 0x00000000                           | 0x00000000                           | 0x00000000                                | 0x00000000                                |
| nDummy                                                             | 0x00                                 | 0x00                                 | 0x00                                 | 0x00                                      | 0x00                                      |

Table 46-24: OSPI Controller BCODE Configuration Lookup Table

| Member Name                                                        | BCODE                                         | BCODE                                         | BCODE                                                 | BCODE                                                 | BCODE                                         |
|--------------------------------------------------------------------|-----------------------------------------------|-----------------------------------------------|-------------------------------------------------------|-------------------------------------------------------|-----------------------------------------------|
| Member Name                                                        | 5                                             | 6                                             | 7                                                     | 8                                                     | 9                                             |
| Transfer Type                                                      | Single bit command, Quad bit address and data | Single bit command, Quad bit address and data | Single bit command, single bit DDR ad- dress and data | Single bit command, single bit DDR ad- dress and data | Single bit command, dual DTR address and data |
| ubDummyCycles                                                      | 0x06                                          | 0x0A                                          | 0x06                                                  | 0x08                                                  | 0x02                                          |
| ubReadCommand                                                      | 0xEB                                          | 0xEB                                          | 0x0D                                                  | 0x0D                                                  | 0xBD                                          |
| ubDataBits                                                         | 0x02                                          | 0x02                                          | 0x00                                                  | 0x00                                                  | 0x01                                          |
| ubAddressBytes (This field is re- tained. Not used by boot kernel) | 0x03                                          | 0x03                                          | 0x03                                                  | 0x03                                                  | 0x03                                          |
| uwClkLower                                                         | 0x0003                                        | 0x0003                                        | 0x0007                                                | 0x0007                                                | 0x0007                                        |
| uReserved0                                                         | 0x0000                                        | 0x0000                                        | 0x0000                                                | 0x0000                                                | 0x0000                                        |
| nCfg                                                               | 0x00000081                                    | 0x00000081                                    | 0x00000081                                            | 0x00000081                                            | 0x00000081                                    |
| nDsr                                                               | 0x00000002                                    | 0x00000002                                    | 0x00000002                                            | 0x00000002                                            | 0x00000002                                    |
| nDrir                                                              | 0x00022000                                    | 0x00022000                                    | 0x00000400                                            | 0x00000400                                            | 0x00011400                                    |
| uReserved1                                                         | 0x00000000                                    | 0x00000000                                    | 0x00000000                                            | 0x00000000                                            | 0x00000000                                    |
| nDummy                                                             | 0x00                                          | 0x00                                          | 0x00                                                  | 0x00                                                  | 0x00                                          |

Table 46-25: OSPI Controller BCODE Configuration Lookup Table

| Member Name                                                        | BCODE                                           | BCODE                                           | BCODE                                           | BCODE                                           | BCODE                                           | BCODE    |
|--------------------------------------------------------------------|-------------------------------------------------|-------------------------------------------------|-------------------------------------------------|-------------------------------------------------|-------------------------------------------------|----------|
| Member Name                                                        | A                                               | B                                               | C                                               | D                                               | E                                               | F        |
| Transfer Type                                                      | Single bit com- mand, dual DTR address and data | Single bit com- mand, dual DTR address and data | Single bit com- mand, quad DTR address and data | Single bit com- mand, quad DTR address and data | Single bit com- mand, quad DTR address and data | Reserved |
| ubDummyCycles                                                      | 0x04                                            | 0x06                                            | 0x06                                            | 0x07                                            | 0x08                                            | 0x00     |
| ubReadCom- mand                                                    | 0xBD                                            | 0xBD                                            | 0xED                                            | 0xED                                            | 0xED                                            | 0x0B     |
| ubDataBits                                                         | 0x01                                            | 0x01                                            | 0x02                                            | 0x02                                            | 0x02                                            | 0x00     |
| ubAddressBytes (This field is re- tained. Not used by boot kernel) | 0x03                                            | 0x03                                            | 0x03                                            | 0x03                                            | 0x03                                            | 0x03     |
| uwClkLower                                                         | 0x0007                                          | 0x0007                                          | 0x0007                                          | 0x0007                                          | 0x0007                                          | 0x0003   |
| uReserved0                                                         | 0x0000                                          | 0x0000                                          | 0x0000                                          | 0x0000                                          | 0x0000                                          | 0x0000   |

Table 46-25: OSPI Controller BCODE Configuration Lookup Table (Continued)

| Member Name   | BCODE      | BCODE      | BCODE      | BCODE      | BCODE      | BCODE      |
|---------------|------------|------------|------------|------------|------------|------------|
| Member Name   | A          | B          | C          | D          | E          | F          |
| nCfg          | 0x00000081 | 0x00000081 | 0x00000081 | 0x00000081 | 0x00000081 | 0x00000081 |
| nDsr          | 0x00000002 | 0x00000002 | 0x00000002 | 0x00000002 | 0x00000002 | 0x00000002 |
| nDrir         | 0x00011400 | 0x00011400 | 0x00022400 | 0x00022400 | 0x00022400 | 0x00000000 |
| uReserved1    | 0x00000000 | 0x00000000 | 0x00000000 | 0x00000000 | 0x00000000 | 0x00000000 |
| nDummy        | 0x00       | 0x00       | 0x00       | 0x00       | 0x00       | 0x00       |

Table 46-26: OSPI Controller BCODE Configuration Lookup Table (Applicable to Secure Streams Only)

| Member Name                                                         | BCODE                                         | BCODE                                         | BCODE                                             | BCODE                                             | BCODE                                            | BCODE                                            |
|---------------------------------------------------------------------|-----------------------------------------------|-----------------------------------------------|---------------------------------------------------|---------------------------------------------------|--------------------------------------------------|--------------------------------------------------|
| Member Name                                                         | 10                                            | 11                                            | 12                                                | 13                                                | 14                                               | 15                                               |
| Transfer Type                                                       | Single bit com- mand, dual ad- dress and data | Single bit com- mand, quad ad- dress and data | Single bit com- mand, single DTR address and data | Single bit com- mand, single DTR address and data | Single bit com- mand, q uad DTR address and data | Single bit com- mand, q uad DTR address and data |
| ubDummyCycles                                                       | 0x0C                                          | 0x04                                          | 0x04                                              | 0x05                                              | 0x03                                             | 0x09                                             |
| ubReadCom- mand                                                     | 0xBB                                          | 0xEB                                          | 0x0D                                              | 0x0D                                              | 0xED                                             | 0xED                                             |
| ubDataBits                                                          | 0x01                                          | 0x02                                          | 0x00                                              | 0x00                                              | 0x02                                             | 0x02                                             |
| ubAddressBytes (This field is re- tained. Not used by boot kernel ) | 0x03                                          | 0x03                                          | 0x03                                              | 0x03                                              | 0x03                                             | 0x03                                             |
| uwClkLower                                                          | 0x0003                                        | 0x0003                                        | 0x0007                                            | 0x0007                                            | 0x0007                                           | 0x0003                                           |
| uReserved0                                                          | 0x0000                                        | 0x0000                                        | 0x0000                                            | 0x0000                                            | 0x0000                                           | 0x0000                                           |
| nCfg                                                                | 0x00000081                                    | 0x00000081                                    | 0x00000081                                        | 0x00000081                                        | 0x00000081                                       | 0x00000081                                       |
| nDsr                                                                | 0x00000002                                    | 0x00000002                                    | 0x00000002                                        | 0x00000002                                        | 0x00000002                                       | 0x00000002                                       |
| nDrir                                                               | 0x00011000                                    | 0x00022000                                    | 0x00000400                                        | 0x00000400                                        | 0x00022400                                       | 0x00022400                                       |
| uReserved1                                                          | 0x00000000                                    | 0x00000000                                    | 0x00000000                                        | 0x00000000                                        | 0x00000000                                       | 0x00000000                                       |
| nDummy                                                              | 0x00                                          | 0x00                                          | 0x00                                              | 0x00                                              | 0x00                                             | 0x00                                             |

NOTE: For the above configurations, the addressing scheme can be 3-bytes or 4-bytes depending on the addressing of flash detected in auto-detection. The SPI mode byte issued for all the SPI controller peripheral based configurations is 0x00. The mode byte is the first byte transmitted after the address cycles and is used to control the continuous read mode functionality in which the next read operation is not required to issue a command cycle. Continuous read mode is not supported during the boot process.

## Supported Quad Mode Enable Methods

The boot ROM does not support enabling quad mode on the SPI flash device. To boot in quad mode, the flash device must be configured outside the boot ROM.

To enable Quad mode, initially boot in Dual mode and use an initcode.

## Run-Time API

The following table provides descriptions of the adi\_rom\_Boot() command parameter.

Table 46-27: OSPI Controller Boot Command Bit Descriptions

| Bits   | Name                     | Setting      | Description                                                                                                                                                                       |
|--------|--------------------------|--------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:28  | ROM_BCMD_SPIM_SPEED      | 0-1111b      | SPI clock divider used. This value is written to the SPI pe- ripherals clock divider register                                                                                     |
| 27     | ROM_BCMD_SPIM_CMDSKIP_EN | 0            | Issue a read command with every read operation                                                                                                                                    |
|        |                          | 1            | Do not issue a read command with each read operation only issue the read command on the first read.                                                                               |
| 26:22  | ROM_BCMD_SPIM_DUMMY      | 00000b       | No dummy clock cycles required after the address                                                                                                                                  |
| 26:22  |                          | 00001b       | 1 dummy clock cycle required after the address                                                                                                                                    |
| 26:22  |                          | ....         | .....                                                                                                                                                                             |
| 26:22  |                          | 11111b       | 31 dummy clock cycle required after the address                                                                                                                                   |
| 21:20  | ROM_BCMD_SPIM_ADDR       | 00b          | Flash device requires an 8-bit address                                                                                                                                            |
| 21:20  |                          | 01b          | Flash device requires a 16-bit address                                                                                                                                            |
| 21:20  |                          | 10b          | Flash device requires a 24-bit address                                                                                                                                            |
| 21:20  |                          | 11b          | Flash device requires a 32-bit address                                                                                                                                            |
| 19:16  | ROM_BCMD_SPIM_BCODE      | 0000b-1111 b | Applies an initial configuration as described in the SPI Con- troller Boot BCODE configurations. Not all options may be available.                                                |
| 15     | ROM_BCMD_SPIM_CMD        | 0            | Issue the read command over the single bit bus                                                                                                                                    |
| 15     |                          | 1            | Issue the read command over the multi-bit bus. Not recom- mended for use. Intended for use with products with on chip SPI flash only, and even then it can cause complica- tions. |
| 14:12  | ROM_BCMD_SPIM_SSEL       | 000b         | Use target select 1 for SPI chip select                                                                                                                                           |
| 14:12  |                          | 001b         | Use target select 2 for SPI chip select                                                                                                                                           |
| 14:12  |                          | 010b         | Use target select 3 for SPI chip select                                                                                                                                           |
| 14:12  |                          | 011b         | Reserved                                                                                                                                                                          |
| 14:12  |                          | 100b         | Reserved                                                                                                                                                                          |

Table 46-27: OSPI Controller Boot Command Bit Descriptions (Continued)

| Bits   | Name                  | Setting       | Description                                                                                                                                                                                                                                                                 |
|--------|-----------------------|---------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|        |                       | 101b          | Reserved                                                                                                                                                                                                                                                                    |
|        |                       | 110b          | Reserved                                                                                                                                                                                                                                                                    |
|        |                       | 111b          | Reserved                                                                                                                                                                                                                                                                    |
| 11:8   | ROM_BCMD_SPIM_DEVENUM | 0 to 16       | Specifies the boot peripheral enumeration. For OSPI Con- troller boot, configure this field to 0.                                                                                                                                                                           |
| 7      | Reserved              | Reserved      | Reserved                                                                                                                                                                                                                                                                    |
| 6      | ROM_BCMD_SPIM_NOAUTO  | 0             | Perform automatic device detection and peripheral configu- ration based on the BCODE value (first nibble) of the boot streams block header                                                                                                                                  |
| 6      |                       | 1             | Do not perform automatic device detection                                                                                                                                                                                                                                   |
| 5      | ROM_BCMD_SPIM_NOCFG   | 0             | Instructs the config routine to perform a pin-muxing and full peripheral configuration                                                                                                                                                                                      |
| 5      |                       | 1             | The config routine does not change the configuration of the peripheral and idoes not enable pin muxing. This can be used when the boot software does not support specific fea- tures of a peripheral, users can configure the peripheral prior to calling the boot routine. |
| 4      | ROM_BCMD_SPIM_HOST    | 0             | Controller boot mode                                                                                                                                                                                                                                                        |
| 4      |                       | 1             | Not to be used for OSPI Controller Boot                                                                                                                                                                                                                                     |
| 3:0    | ROM_BCMD_SPIM_DEVICE  | 0000b         | Not to be used for OSPI boot                                                                                                                                                                                                                                                |
| 3:0    |                       | 0001b         | Not to be used for OSPI boot                                                                                                                                                                                                                                                |
| 3:0    |                       | 0010b         | Not to be used for OSPI boot                                                                                                                                                                                                                                                |
| 3:0    |                       | 0011b         | Not to be used for OSPI boot                                                                                                                                                                                                                                                |
| 3:0    |                       | 0100b         | Not to be used for OSPI boot                                                                                                                                                                                                                                                |
| 3:0    |                       | 0101b         | Not to be used for OSPI boot                                                                                                                                                                                                                                                |
| 3:0    |                       | 0110b         | Not to be used for OSPI boot                                                                                                                                                                                                                                                |
| 3:0    |                       | 0111b         | Not to be used for OSPI boot                                                                                                                                                                                                                                                |
| 3:0    |                       | 1000b         | OSPI Memory Mapped Boot (Memory mapped mode and MDMA)                                                                                                                                                                                                                       |
| 3:0    |                       | 1001b - 1111b | Not to be used for OSPI boot                                                                                                                                                                                                                                                |

NOTE: All bits in the above table that are not defined must be set to zero.

To support the above lookup table at varying SPICLK frequency range, boot kernel programs the Read data capture register such that sampling time does not go beyond one full cycle of SPI CLK for STR and

half cycle of SPI CLK for DTR mode of operation. When this timing is not sufficient to support all flash vendors, there is a provision to update the Read data capture register from the Customer OTP space with a worst case value corresponding to a different flash device used by user. This value has to be derived based on the minimum-maximum AC timing propagation delay from OSPI controller to pin.

## Default Booting Peripheral Pin-Mux

By default SPI2, UART0, LP0 and OSPI0 boot peripherals support booting through a default pin-mux combinations given in the following tables.

Table 46-28: SPI2 Default Pin-Mux

| Signals    | ADSP-21593   | ADSP-21954/SC592/SC594   |
|------------|--------------|--------------------------|
| SPI2_MOSI  | PA_01        | PA_01                    |
| SPI2_MISO  | PA_00        | PA_00                    |
| SPI2_D2    | PA_02        | PA_02                    |
| SPI2_D3    | PA_03        | PA_03                    |
| SPI2_CLK   | PA_04        | PA_04                    |
| SPI2_SEL1b | PA_05        | PA_05                    |
| SPI2_RDY   | PB_05        | PB_05                    |

Table 46-29: OSPI0 Default Pin-Mux

| Signals     | ADSP-21593   | ADSP-21954/SC592/SC594   |
|-------------|--------------|--------------------------|
| OSPI0_MOSI  | PA_01        | PC_11                    |
| OSPI0_MISO  | PA_00        | PC_12                    |
| OSPI0_D2    | PA_02        | PC_10                    |
| OSPI0_D3    | PA_03        | PC_09                    |
| OSPI0_D4    | PA_06        | PD_00                    |
| OSPI0_D5    | PA_07        | PC_15                    |
| OSPI0_D6    | PA_08        | PC_14                    |
| OSPI0_D7    | PA_09        | PC_13                    |
| OSPI0_CLK   | PA_04        | PC_08                    |
| OSPI0_SEL1b | PA_05        | PD_01                    |

Table 46-30: UART0 Default Pin-Mux

| Signals   | ADSP-21593   | ADSP-21954/SC592/SC594   |
|-----------|--------------|--------------------------|
| UART0_CTS | PA_09        | PA_09                    |

Table 46-30: UART0 Default Pin-Mux (Continued)

| Signals   | ADSP-21593   | ADSP-21954/SC592/SC594   |
|-----------|--------------|--------------------------|
| UART0_RTS | PA_08        | PA_08                    |
| UART0_TX  | PA_06        | PA_06                    |
| UART0_RX  | PA_07        | PA_07                    |

Table 46-31:

| Signals   | ADSP-21593   | ADSP-21954/SC592/SC594   |
|-----------|--------------|--------------------------|
| LP0_DAT0  | PB_07        | PB_07                    |
| LP0_DAT1  | PB_08        | PB_08                    |
| LP0_DAT2  | PB_09        | PB_09                    |
| LP0_DAT3  | PB_10        | PB_10                    |
| LP0_DAT4  | PB_11        | PB_11                    |
| LP0_DAT5  | PB_12        | PB_12                    |
| LP0_DAT6  | PB_13        | PB_13                    |
| LP0_DAT7  | PB_14        | PB_14                    |
| LP0_CLK   | PB_06        | PB_06                    |
| LP0_ACK   | PB_04        | PB_04                    |

## Boot Loader Stream

A loader stream is a set of formatted blocks containing instructions for the boot kernel, as well as the application and data for loading to the chip. This section details the different aspects of the stream, its blocks, some common use cases, and the ROM function.

Each block begins with a block header that contains attributes of the block as well as flags to control its processing by the boot ROM. On power-up or reset, the processor begins executing the on-chip boot ROM. The boot stream is either read from memory or received from a peripheral, depending on the boot mode specified. Each block in the boot stream instructs the boot kernel to perform an action, most commonly to load data to a specified location. For a complete list of actions, refer to the Block Types section.

Some common actions include the following:

- running code that initializes a peripheral
- processing data then loading it to a location

As shown in the Project Flow figure, a utility is required to process the resulting output from the tool chain to create a valid boot stream. This utility can be in the form of a standalone application or script that parses an application

image file, elf output file, or text-based file such as Intel hex. A flash programmer utility can format a boot stream internally.

The elfloader utility parses the application data and creates a set of blocks representing the segmented application. When processing an actual image that must be loaded to a single contiguous memory block, this representation can contain as little as a single header block. The output of the standalone utility is stored in a loader file ( .ldr ). The loader file contains the boot stream and becomes available to the hardware by:

- Programming or burning it into non-volatile external memory.
- Alternatively, sending it through a peripheral such as the UART during boot time

A loader stream must always begin with a first block and end with a final block. The loader file contains the boot stream and becomes available to hardware by:

- programming or burning it into non-volatile external memory, or
- sending it through a peripheral during boot time

Figure 46-13: Project Flow

<!-- image -->

The Booting Process figure shows the parallel or serial boot stream contained in a flash memory device. In host boot scenarios, the non-volatile memory usually connects to the host processor rather than directly to the processor. After reset, the on-chip boot kernel reads and parses the headers, and processes the loader stream block-by-block. Finally, payload data is copied to a destination addresses, either in on-chip L1 and L2 memory, or off-chip to SDRAM or SRAM.

Figure 46-14: Booting Process

<!-- image -->

In some cases (for example, secure boot or when the BFLAG\_INDIRECT flag for any block is set), the boot kernel uses another memory block for intermediate data storage. In order to preserve the security of the device processors do not allow these storage regions to be initialized with boot data. The boot stream is loaded to the intermediate storage then processed by the kernel and loaded to the final destination. The final destination cannot be the intermediate storage location otherwise the boot process terminates in an error.

## Block Header

Figure 46-15: Loader Stream Block Structure

<!-- image -->

A boot stream consists of multiple boot blocks as shown in the figure. A 16-byte block header begins every block. The 16 bytes are functionally grouped into four 32-bit words:

- Block code
- Target address
- Byte count
- Argument field

This section provides a general description describes the fields in general. The use varies depending on the block type and boot mode.

## Block Code

Table 46-32: Block Code Flags

| Bit   | Name       | Description                                                                                                                                                                                                   |
|-------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0-3   | BCODE      | Specific to boot modes (see Boot Modes)                                                                                                                                                                       |
| 4     | BFLAG_SAVE | Intended to allow for a user application to mark blocks for saving the memory of this block to off-chip memory in case of power failure. The on-chip boot kernel does not use this flag.                      |
| 5     | BFLAG_AUX  | When set indicates that the byte address space translation for SHARC core boot blocks requires translation to the 48-bit PM address space. When cleared translation to the 32-bit address space is performed. |

Table 46-32: Block Code Flags (Continued)

| Bit   | Name           | Description                                                                                                                                                                            |
|-------|----------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7-6   | Reserved       | Reserved                                                                                                                                                                               |
| 8     | BFLAG_FILL     | Fill the target location with a specified 32-bit value                                                                                                                                 |
| 9     | Reserved       | Reserved                                                                                                                                                                               |
| 10    | BFLAG_CALLBACK | Calls the previously registered callback function                                                                                                                                      |
| 11    | BFLAG_INIT     | Calls function at target address. If the block contains a payload, the payload is loaded prior to the call.                                                                            |
| 12    | BFLAG_IGNORE   | Block payload is ignored                                                                                                                                                               |
| 13    | BFLAG_INDIRECT | Boots the payload to the intermediate storage location                                                                                                                                 |
| 14    | BFLAG_FIRST    | Indicates the block that is the beginning of a new application                                                                                                                         |
| 15    | BFLAG_FINAL    | Indicates the last block of a loader stream. Booting will complete after processing the block. This flag does not denote the end of an application in a Multi-Application Boot Stream. |
| 16-23 | HDRCHK         | A simple 8-bit XOR checksum of the other 24 bits in the boot block header.                                                                                                             |
| 24-31 | HDRSIGN        | 0xAD, 0xAC or 0xAB. Indicates if the boot block is intended for core 0, core 1 or core 2 respec- tively.                                                                               |

## TARGET\_ADDRESS

The TARGET\_ADDRESS holds the applicable address for the block, (where the code or data is loaded). However, the interpretation of the field differs depending on what specific flags are set in the block code. Refer to the documentation for each block type for details.

The following attributes must be true:

- The target address must be divisible by 4, as the boot kernel uses 32-bit DMA for certain operations.
- The target address must point to valid on-chip or off-chip memory locations.

## BYTE\_COUNT

The BYTE\_COUNT must be divisible by 4, and can also be zero. This 32-bit field generally holds the size of the block. In some cases, it has a different use (such as when BFLAG\_FILL is set). See the Block Types section for information on the variations.

## ARGUMENT

The 32-bit field is a user-variable for most block types. The Initcode or callback routine can access this value and use it for optional instructions.

The different block types use the ARGUMENT field in various ways. See the Block Types descriptions for further information.

## Block Types

A loader stream is a set of linked blocks and each block is responsible for performing a certain function dependent on the block type. The flags in the block header define a block type. Operations include functions such as loading data, filling a memory region with data, and instructing the kernel to stop processing. This section describes each block type and its usage within a boot loader stream.

## Normal Block

The primary function of a block is to load data into a specified location of memory. A normal block instructs the boot kernel to load the data contained in its payload to the location specified in the TARGET\_ADDRESS field. The BYTE\_COUNT defines the size of the payload. Once the correct amount of data has been loaded, the kernel moves on to process the next block in the stream.

Table 46-33: Flags

| Flag           | Required Value   | Init                                            |
|----------------|------------------|-------------------------------------------------|
| TARGET_ADDRESS | Y                | Address where payload is loaded (must be valid) |
| BYTE_COUNT     | Y                | Size of block in bytes                          |

## First Block

A first block indicates the start of a boot stream and is always needed at the beginning of the boot stream. When a loader stream contains Multi-Application Boot Streams, a first block that occurs within the loader stream indicates the beginning of a new application.

When the kernel processes the first block in a loader stream, the boot kernel uses the TARGET\_ADDRESS to determine the application entry location. For more details, refer to Boot Termination and Application Execution .

NOTE: A first block cannot be combined with a fill block.

Table 46-34: Flags

| Flag           | Required Value   | Init                                                                                                                                                                                                    |
|----------------|------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| BFLAG_FIRST    | Y                | 1                                                                                                                                                                                                       |
| ARGUMENT       | Y                | Offset to the next application, or first address following loader stream. Commonly re- ferred to as the NEXTDXE field.                                                                                  |
| TARGET_ADDRESS | Y                | When the block is the first block in a loader stream it defines the start address for the application. If the block is not the first in a loader stream, use the target address as in normal operation. |

## Final Block

The final block marks the last block in a boot stream. After processing a final block, the boot kernel jumps to the start address of the application. For more information on the definition of the start address, refer to Boot Termination and Application Execution .

There is further customization available to alter the kernel behavior. For example, the kernel can be instructed to return from the boot routine rather than jump to the application using initialization codes or the adi\_rom\_Boot() API.

Before the boot kernel passes program control to the application, it does some housekeeping. Most of the registers in use are set to their default state. However, some register values can differ depending on the boot mode. See Boot Modes for more information.

Table 46-35: Flags

| Flag        | Required Value   |   Init |
|-------------|------------------|--------|
| BFLAG_FINAL | Y                |      1 |

## Indirect Block

An indirect block is first loaded to a storage location before being copied to the destination. This block is used in the following situations:

- Some boot modes do not use DMA from the boot peripheral. The core is not always able to access some memory locations directly or efficiently. An intermediate load to a different location improves overall efficiency.
- In some booting situations, the data in the payload must be operated-on or analyzed before it is fully loaded (such as decryption or a checksum calculation). By using an intermediate location, such situations are simplified and are more efficient when loading to off-chip memories.

In some cases, a boot block does not fit into temporary storage memory. Having a larger buffer can improve boot performance. If an entire block cannot fit into the buffer, it is processed in pieces. Initialization code or callback functions can alter the temporary buffer region, including its location and size, by modifying the ADI\_ROM\_BOOT\_CONFIG::pTempBuffer and ADI\_ROM\_BOOT\_CONFIG::dTempByteCount varia- bles in the struct ADI\_ROM\_BOOT\_CONFIG structure.

Table 46-36: Flags

| Flag           | Required Value   | Init                                                                                                |
|----------------|------------------|-----------------------------------------------------------------------------------------------------|
| BFLAG_INDIRECT | Y                | 1                                                                                                   |
| BFLAG_CALLBACK | N                | Defines a callback function to operate on intermediate data. These 2 flags are often used together. |

## Ignore Block

It is a block that is ignored by the loader stream (in several cases). These blocks are useful when it is not possible to pass information in another block header. For example, if the first block contains data such as a firmware revision rather than application code, then BFLAG\_IGNORE can be set with the correct application start address. This step ensures that the loader stream uses the correct start address. Since this block has no other function, identify it as an ignore block. Then, the kernel does not attempt to process any payload.

Ignore blocks clear the following flags, disabling the corresponding blocks from being processed if set along with BFLAG\_IGNORE :

- BFLAG\_INIT
- BFLAG\_CALLBACK
- BFLAG\_FINAL
- BFLAG\_AUX

NOTE: When BFLAG\_IGNORE is set along with BFLAG\_FIRST , only the payload associated with the first block is ignored. The application entry point retrieved from the first block is always processed.

Table 46-37: Flags

| Flag         | Required Value   | Init                                 |
|--------------|------------------|--------------------------------------|
| BFLAG_IGNORE | Y                | 1                                    |
| BYTE_COUNT   | Y                | Size of block to ignore; can be zero |

## Fill Block

A fill block instructs the boot kernel to perform a 32-bit memory fill of the memory region. Fill blocks help minimize the size of a boot stream when an application contains large arrays of data that need are initialized at startup. Zero fill is the most common fill block type. However, any 32-bit fill value is supported.

Table 46-38: Flags

| Flag           | Required Value   | Init                                            |
|----------------|------------------|-------------------------------------------------|
| BFLAG_FILL     | Y                | 1                                               |
| TARGET_ADDRESS | Y                | Address where payload is loaded (must be valid) |
| BYTE_COUNT     | Y                | Size of block in bytes (must be multiple of 4)  |
| ARGUMENT       | Y                | 32-bit fill value                               |

## Init Block

An initialization block instructs the boot kernel to perform a function call to the target address after the entire block has loaded. The function called is referred to as the initialization code (Initcode) routine . Refer to the API reference initcode() for full details.

If the Initcode routine has been previously loaded, the block can declare a zero-size and have no payload.

Initcode routines can be used to speed up and customize booting mechanisms exposed by the boot kernel. Traditionally, an Initcode routine is used to set up the system PLL, bit rates, wait states, and the external memory controllers. Boot time can be significantly reduced when an init block is executed early in the boot process.

Initcode routines are required to follow the C language calling conventions. The expected C prototype is:

```
void initcode(ADI_ROM_BOOT_CONFIG * pBootConfig)
```

NOTE: When programming in assembly, use a return from subroutine instruction for returns.

The structure provided to the Initcode routine by the boot kernel contains information about the block being processed. It includes header information, locations of temporary block data (for indirect blocks), target address, and byte count. See struct ADI\_ROM\_BOOT\_CONFIG for a full list and details on the provided data.

In the simplest case, an Initcode routine consists of only a single block in which the BFLAG\_INIT flag is set. For larger routines, a sequence of blocks can incrementally load the routine, and only the last block sets the BFLAG\_INIT flag. In the latter case, the last block has no payload attached, and simply instructs the boot kernel to issue a call to subroutine instruction.

An Initcode routine can be overwritten by a successive block when it is no longer needed. Otherwise, the routine can be called at multiple points during the boot process, and even remain in memory after the application completes booting.

NOTE: The following list provides requirements for Initcode that is written in C or C++.

- Ensure that the Initcode routine does not contain calls to the run-time libraries
- Do not assume that parts of the run-time environment, such as the heap, are fully functional
- Ensure that all run-time components are loaded and initialized before the routine executes

The loader utility and tool set provide mechanisms to aid in implementing initialization codes and organizing them properly within the boot loader stream. A special project type is provided to allow the creation of Initcode routines as separate projects. Options are available to assign particular pieces of the application to be Initcode routines. For details and more information on the utility, see the Loader and Utilities Manual .

Table 46-39: Flags

| Flag       | Required Value   |   Init |
|------------|------------------|--------|
| BFLAG_INIT | Y                |      1 |

Table 46-39: Flags (Continued)

| Flag           | Required Value   | Init                                                                             |
|----------------|------------------|----------------------------------------------------------------------------------|
| TARGET_ADDRESS | Y                | Location to load payload data. Call to subroutine issued to the same loca- tion. |
| ARGUMENT       | N                | Can be used to supply block specific arguments                                   |
| BYTE_COUNT     | Y                | Size of payload; can be zero                                                     |

NOTE: Init blocks cause software not located in the boot ROM during the boot process to execute. In the case of a secure boot scenario, initcode routines are not supported. The secure boot authentication process is performed at the end of the boot process. Execution of any user software prior to the authentication process violates the secure boot requirements.

## Callback Block

It instructs the boot kernel to call a pre-registered function when the payload of the block is loaded. The purpose of a callback routine is to apply standard processing to the block payload. The callback routine is registered through an Initcode routine prior to loading a block using the routine. Typically, callback routines contain checksum, decryption, decompression or hash algorithms.

To register a callback, create an Init Block whose Initcode modifies the ADI\_ROM\_BOOT\_CONFIG::pCallBackFunction with the address of the callback function to execute. A callback function must be registered prior to processing a callback block.

As callback routines require access to the payload data of the boot blocks, load the block data before processing. Often an Indirect Block is used in combination with a callback block. Indirect blocks in combination with callback blocks allow for post processing of the loaded data before it is then transferred to the final destination.

Callback routines are expected to meet the C language calling conventions. The prototype is as follows:

```
ROM_BOOT_RESULT callback( ADI_ROM_BOOT_CONFIG * pBootConfig, ADI_ROM_BOOT_BUFFER * pBuffer, uint32_t nFlags )
```

The pBootConfig argument contains a pointer to the struct ADI\_ROM\_BOOT\_CONFIG information and pBuffer provides access to the address and size of the block (can vary when using indirect). The nFlags parameter is specifically used when the BFLAG\_INDIRECT flag is also used. The CBFLAG\_DIRECT flag indicates that the BFLAG\_INDIRECT bit is not active so that the program only calls the callback routine once per block. When the CBFLAG\_DIRECT flag is set, the CBFLAG\_FIRST and CBFLAG\_FINAL flags are also set.

NOTE: Callback blocks result in execution of software that is not located in the boot ROM during the boot process. In the case of a secure boot scenario callback routines are not supported because the secure boot authentication process is performed at the end of the boot process and execution of any program prior to the authentication process violates the secure boot requirements.

## Callback Block in Conjunction with Indirect Block

A block using a callback routine is also loaded indirectly and there are slight behavioral differences. The procedure for loading is:

1. Load data into the temporary buffer defined by ADI\_ROM\_BOOT\_CONFIG::pTempBuffer .
2. Issue a call to ADI\_ROM\_BOOT\_CONFIG::pCallBackFunction .
3. After the callback routine returns and when the return value is zero, the memory DMA copies data to the destination.

When a block does not fit entirely into the temporary buffer, loading is performed similar to indirect blocks. The software calls the callback function after each chunk is loaded into the temporary storage. The nFlags parameter provides information on the specific iteration.

When a block does not fit entirely into the temporary storage area, nFlags tells the callback routine whether it is invoked for the first time ( CBFLAG\_FIRST ) or called the last time ( CBFLAG\_FINAL ) for a specific block.

When the software invokes DMA to copy the data, it relies on the supplied pBuffer data, not the ADI\_ROM\_BOOT\_CONFIG::pTempBuffer and ADI\_ROM\_BOOT\_CONFIG::dTempByteCount members of the boot structure.

The callback routine can control the source of the memory DMA by altering the content of the pBuffer structure. This alteration is necessary when the callback routine performs data manipulation such as decompression.

When the software uses an indirect block, the return value of the callback routine determines whether the DMA transfer occurs. If the value is non-zero, then the transfer does not occur.

Table 46-40: Flags

| Flag           | Required Value   |   Init |
|----------------|------------------|--------|
| BFLAG_CALLBACK | Y                |      1 |

NOTE: Callback blocks cause software not located in the boot ROM during the boot process to execute. In the case of a secure boot scenario callback routines are not supported because the secure boot authentication process is performed at the end of the boot process and execution of any user software prior to the authentication process violates the secure boot requirements.

## Save Block

A save block is intended to mark blocks in a boot stream for saving to off-chip memory. The on-chip boot kernel does not use this flag. A program can process the boot steam to find address of regions of memory for saving to

external memory. On a reboot the program may then restore the previously saved contents. This block provides a means of doing a context restore after a reboot.

Table 46-41: Flags

| Flag       | Required Value   |   Init |
|------------|------------------|--------|
| BFLAG_SAVE | Y                |      1 |

## Single-Block Boot Streams

The simplest boot stream consists of a single block header and one contiguous block of instructions and optionally data. When the appropriate flags are set in the block header, the kernel loads the block to the target address, terminates the boot process, and begins executing from the entry address of the application.

The Initial Header for Single-Block Stream table shows an example of a single-block boot stream header with settings that can be loaded using any boot mode. The BFLAG\_FIRST and BFLAG\_FINAL flags are both set at the same time. The desired location and size of the application determines the target address and byte count.

When using single block boot streams on products with multiple cores, the boot stream must be targeted towards the primary booting core that manages the boot process. If core 0 is the primary booting core, the boot stream must contain code that is intended for execution by that core. It is possible to boot a single block boot stream when using the API to load an application to the non-booting core. In this case, the BFLAG\_RETURN flag must be set so the boot process returns to user application on the core controlling the boot process.

Table 46-42: Initial Header for Single-Block Stream

| Field          | Description of Value                                                     |
|----------------|--------------------------------------------------------------------------|
| BLOCK_CODE     | 0xAD000000&#124;XORSUM&#124; BFLAG_FINAL &#124; BFLAG_FIRST              |
| TARGET_ADDRESS | Start address of block                                                   |
| BYTE_COUNT     | Number of bytes in the block                                             |
| ARGUMENT       | Functions as next-application pointer in multi-application boot streams. |

## Direct Code Execution

Applications can avoid long boot times and execute code from flash or SDRAM memory with minimal processing by the boot kernel. This feature is called direct code execution.

An initial boot block header is required for the processor to fetch and execute program code from the boot device as early as possible. The block uses safety mechanisms to avoid unpredictable processor behavior when boot memory is not yet programmed with valid data. Safety mechanisms include the header signature and the byte-wise XOR checksum. Rather than blindly executing code, the boot kernel first executes the pre-boot routine for system customization. It then loads the first block header and checks it for consistency. If the block header is corrupt, the boot kernel calls the error handler and does not start code execution.

If the initial block header check is good, the boot kernel interrogates the block flags. If the BFLAG\_FINAL flag is set, the boot kernel terminates and executes the sequence as described in the Boot Termination and Application Execution section. If the application requires that the boot kernel customize the starting address in advance, the first block must also have the BFLAG\_FIRST flag set. The target address field is then saved as the application start address.

When processing direct code execution blocks, the block instructs the processor executing the boot code to execute from the address specified. It is not possible to have core 0 boot the block and have it instruct core 1 to immediately start execution from the address provided.

For example, when the block header described in the Direct Code Execution Header table is placed at address 0x20000000, the boot kernel is instructed to issue a JUMP command to address 0x20000020.

Table 46-43: Example Direct Code Execution Header

| Field          | Value      | Comments                                                                                                                                                                                       |
|----------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| BLOCK_CODE     | 0xAD7BD006 | 0xAD000000&#124;XORSUM&#124; BFLAG_FINAL &#124; BFLAG_FIRST &#124; BFLAG_IGNORE &#124;BCODE                                                                                                    |
| TARGET_ADDRESS | 0x20000020 | Start address of application code. Provided as an example this would be de- pendent upon the start address required for a given product.                                                       |
| BYTE_COUNT     | 0x00000010 | Ignores 16 bytes to provide space for control data such as version code and build data. This field is optional and can be zero. The payload is ignored due to the BFLAG_IGNORE flag being set. |
| ARGUMENT       | 0x00000010 | Functions as next-application pointer in multi-application boot streams.                                                                                                                       |

## Multi-Application Boot Streams

This section describes loader streams that contain multiple applications.

A boot stream is typically generated from an application file. Loader streams with more than one application are commonly referred to as multi-application booting. A loader utility often accepts multiple application files as input parameters and generates a contiguous boot image. The subsequent applications are appended to the first.

The loader utility must also update the argument field of all BFLAG\_FIRST blocks. The argument field of a BFLAG\_FINAL block is called the next-application pointer.

The next-application pointer of the first application boot stream is a relative pointer to the start address of the second application boot stream. A multi-application boot image can be seen as a linked list of boot streams. The nextapplication pointer of the last application boot stream is a relative pointer to the next free address. The Multi-Application Boot Stream Example figure shows an example.

Figure 46-16: Multi-Application Boot Stream Example

<!-- image -->

The Multi-Application Direct Code Execution Example figure shows a linked list of initial block headers. The blocks instruct the boot kernel to terminate immediately and to start code execution at the address provided by the target address field of the individual blocks. There is nothing in the boot code that prevents multi-application streams from mixing regular boot streams and direct code execution blocks.

Figure 46-17: Multi-Application Direct Code Execution Example

<!-- image -->

## CRC32 Protection

The boot kernel has mechanisms that allow verification of each block's payload using a 32-bit CRC. The boot ROM contains a function that can be called as an initcode to register the CRC callback and initialize the CRC peripheral with a specified polynomial. CRC32 protected loader streams are supported by the programming tools utlities.

To enable this feature an Block Types must be located in the boot stream with a TARGET\_ADDRESS that points to the adi\_rom\_Crc32Init() function in the ROM. The ARGUMENT field contains the CRC32 checksum polynomial that is used to initialize the CRC lookup table. Once the CRC initcode function in the ROM has been executed, CRC verification is enabled for all subsequent blocks except:

- Ignore
- First

NOTE: CRC functionality is dependent upon the use of an Initblock and is not supported in secure booting.

## Secure Boot

The secure boot process provides a way to integrate security in the processor boot sequence. A chain of trust is established within the system by ensuring the integrity and authenticity of the boot image. Confidentially is also supported.

Secure boot increases protection against malicious, unsecured accesses to critical and confidential resources of the processor. The boot stream application code and data must be digitally signed in order to build up a chain of trust in

the system. This requirement allows the processor to distinguish between authentic and trusted code from nonauthentic and untrusted code.

Secure boot also provides confidentiality support. The digitally signed boot image can be optionally encrypted. When loading an encrypted image, the ROM decrypts while loading, then authenticates, before any application code is executed.

Secure boot is an optional feature of the processor that is disabled by default. The feature is enabled using the OTP lock API; secure boot cannot be disabled after it has been enabled. When security is enabled, developers are not dependent on Analog Devices to provision the devices, sign code or provide security certificates. The required tools for signing and encrypting the boot images are provided with the processor development tools.

## Integrity and Authenticity Protection

Integrity protection is based on the secure hash SHA-2 224/256-bit algorithm. Authenticity protection is based on the ECDSA algorithm. The ADSP-SC59x Boot ROM uses P-224 or P-256 curves for the implementation of the ECDSA-224/256 algorithm to exercise secure boot.

ECDSA uses public key cryptography consisting of two keys: a private key and a public key. The public key is stored in OTP memory on the processor so that the secure boot process can verify the authenticity of the signed boot image. Only parties in possession of the private key are able to sign the images.

## Confidentiality Protection

Confidentiality protection uses the AES algorithm. Two variants are supported: wrapped and unwrapped.

The wrapped variant uses a 128-bit Key Encryption Key (KEK) stored on the processor to decrypt the 128-bit AES decryption key embedded in the secure header. The unwrapped variant stores the AES description key on the processor and utilizes it to decrypt the entire image.

The privacy of the key stored on the device (whether AES or KEK) is paramount to the security of the system. Disclosure of this key compromises security of the entire system.

## Anti-Cloning Protection

Anti-cloning protection is based on the confidentiality protection. If each processor in a system uses a unique private key for confidentiality protection, then cloning between these devices can be prevented. The boot image is incompatible with devices using a different private key for decryption.

## Anti-Rollback Protection

The secure boot process supports anti-rollback protection using a 32-bit counter in OTP memory. A value of 0x00000000 in the OTP makes anti-rollback disabled by default. If anti-rollback protection is required, then the program can set the rollback ID when signing the boot image. When the boot image is authenticated, the secure boot software updates the counter in OTP (if the rollback ID in the boot image is greater than the value currently stored in the OTP counter).

The rollback ID stored in the secure boot image header is integrity-protected and cannot be altered.

NOTE: To enable anti-rollback protection for secure boot operations, write a non-zero value to the 32-bit counter in OTP memory. When the counter register remains at the default value of zero, anti-rollback protections are not enabled, regardless of the rollback ID located in a secure boot stream.

CAUTION: There are a number of restrictions for the rollback ID in the OTP module. Programs should only use the OTP boot program ROM API to set the counter. Refer to the OTP counters section for information on the implementation strategy.

## Terminology

## ECDSA

Elliptical Curve Digital Signature Algorithm

## BLp

## BLx

## BLw

## BLe

## SBLS

## SBH

Secure Boot Header

## SBCR

Secure Boot Confidentiality Root

## AES

Boot Loader plain text, Plaintext Format

Boot Loader without key, Keyless Format

Boot Loader wrapped, Wrapped Format

Boot Loader encrypted, Encrypted Format

Secure Boot Loader Stream

Advanced Encryption Standard

## Signing for Secure Boot Images

All boot images must be digitally signed to create secure boot images. The boot image is processed by the security utilities included with the development tools to sign and optionally encrypt the boot image. The security utilities operate with key-pairs consisting of a private and a public key. The private key is used for signing the images, and the public key is used to validate an image being loaded into the processor.

CAUTION: The private key generated from the signing utility, used for signing images, is never required by the processor for successful secure boot. The private key is only ever required by the signing utility and should be made available only within the system responsible for the image signing process.

The image signing utility provides the following functionality:

- Signing and encrypting of images
- Generation of ECDSA key pairs
- Generation of random encryption keys
- Extraction of the public key from an ECDSA key pair
- Setting Secure Boot Image Attributes

For more information on the use of the signing utility, refer to the Loader and Utilities manual.

## Secure Boot Image Types

This section provides an overview of the different image types and how to use them.

## Plaintext (BLp) Format

The BLp format provides integrity plus authentication protection of the boot image. The boot image is produced using 224-bit or 256-bit Elliptical Curve Digital Signature Algorithm (ECDSA) private key. To authenticate the image, program the corresponding public key into the OTP public\_key field using the OTP boot program API.

SBH

## Wrapped (BLw) Format

The BLw format provides the highest level of protection: integrity plus authentication, confidentiality, and anticloning protection. The image contains an AES wrapped image encryption key (denoted by [K]) within the secure header. The image data is encrypted with the wrapped key, preventing cloning. An extra key is required to unwrap the header; program this key into the OTP pvt\_128key field using the OTP boot program API.

| SBH [K]   | Encrypted Boot Loader Stream   |
|-----------|--------------------------------|

Boot Loader Stream

## Keyless (BLx) Format

The BLx format is similar to the BLw format except that the image does not contain the key at all. This format provides anti-cloning protection only if the secure key is unique per device. Program the decryption key for the data into OTP pvt\_128key field using the OTP boot program API.

| SBH   | Encrypted Boot Loader Stream   |
|-------|--------------------------------|

## Secure Boot Image Format

Secure Boot images provide authenticity and integrity protection during the boot process. A secure boot image is comprised of a secure boot header and an optionally encrypted loader stream.

Signed images consist of the following sections to comprise a complete secure boot image:

- Secure Boot Header
- Image Attributes
- Image Section

The Figure 46-18 Secure Boot Image shows that the image attributes are encapsulated within the secure boot header. The image attributes are actually integrity protected along with the image section. The image section contains a standard boot loader stream. Some block types are not allowed as described in Unsupported Boot Stream Blocks.

Figure 46-18: Secure Boot Image

<!-- image -->

## Secure Boot Header

Table 46-44: Secure Boot Header

|          |            |                                                                                                            | Values                                                                                                                                     | Values                                                                                                                                     | Values                                                                                                                                     |
|----------|------------|------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------|
| Bytes    | Name       | Description                                                                                                | Keyless Format BLx                                                                                                                         | Wrapped Format BLw                                                                                                                         | Plain Text Format BLp                                                                                                                      |
| 3:0      | Type       | Format and version of the image. Upper 24 bits are the image format and lower 8 bits are the image version | 0x424c7802                                                                                                                                 | 0x424c7702                                                                                                                                 | 0x424c7002                                                                                                                                 |
| 67:4     | Signature  | The ECDSA signature of the image                                                                           | Two 256-bit numbers                                                                                                                        | Two 256-bit numbers                                                                                                                        | Two 256-bit numbers                                                                                                                        |
| 99:68    | Hash       | Hash of the image                                                                                          | Hash size depends upon whether ECDSA 224 or ECDSA 256 is used. For ECDSA 224 Hash Value: 95:68 bytes For ECDSA 256 Hash Value: 99:68 bytes | Hash size depends upon whether ECDSA 224 or ECDSA 256 is used. For ECDSA 224 Hash Value: 95:68 bytes For ECDSA 256 Hash Value: 99:68 bytes | Hash size depends upon whether ECDSA 224 or ECDSA 256 is used. For ECDSA 224 Hash Value: 95:68 bytes For ECDSA 256 Hash Value: 99:68 bytes |
| 123:100  | Key        | Confidentiality (only appli- cable for certain formats)                                                    | Reserved                                                                                                                                   | 192-bit AES-WRAP data holding a 128-bit AES key                                                                                            | Reserved                                                                                                                                   |
| 139:124  | IV         | Initialization Vector (only applicable for certain for- mats)                                              | 16-bit IV generated during signing process                                                                                                 | 16-bit IV generated during signing process                                                                                                 | Reserved                                                                                                                                   |
| 143:140  | Length     | The length of the image sec- tion in bytes                                                                 | Maximum supported byte count 0x10000000 bytes                                                                                              | Maximum supported byte count 0x10000000 bytes                                                                                              | Maximum supported byte count 0x10000000 bytes                                                                                              |
| 207:144  | Attributes | Image attributes                                                                                           | Support for up to 8 image attributes                                                                                                       | Support for up to 8 image attributes                                                                                                       | Support for up to 8 image attributes                                                                                                       |
| 211 :208 | Reserved   |                                                                                                            |                                                                                                                                            |                                                                                                                                            |                                                                                                                                            |

## Secure Boot Processing Overview

The Secure Boot Processing figure illustrates how the block is processed using secure features (not all block header type details are shown). For details on the various block types and their function, see Block Types. Some image types are decrypted. Decrypt indicates that the data is decrypted when applicable to the Secure Boot Image Types at that particular stage.

Figure 46-19: Secure Boot Processing

<!-- image -->

## Unsupported Boot Stream Blocks

To ensure the security of the processor, the following block types are not supported in a secure boot image. If the boot kernel finds one of these block types, the boot process terminates.

- Init Block: It require a call to the application code prior to the authentication of the boot image. If customizations or optimizations are necessary to improve the load performance, use a second stage loader style implementation. The first application contains only the custom code. Issue a call using the api\_boot () routine to boot using the desired device.
- Call Block: It require a call to a user-defined address prior to the authentication of the boot image, and therefore cannot be supported.

NOTE: Secure boot streams use double buffer Page Mode to optimize the boot process. This functionality allows for the performance of decrypt and hash operations on received data while new data is fetched from the boot source. This host in slave boot mode must ensure that more data is sent after the boot stream to ensure that the temp buffer is filled completely. The size of the secure boot stream minus the size of the secure boot header must be a multiple of the size of the temp buffer. The temp buffer default size is 1024 bytes.

## Secure Boot Image Attributes

Secure boot image attributes form part of the secure boot header. The attributes provide more information about the content of the secure boot image.

All image attributes are integrity protected using the same algorithm as the image section. When the image authentication process completes and the image is successfully authenticated, the image attributes are known to be trustworthy.

Attributes are specified as type value pairs with both the type and value being a 32-bit value. The boot code supports the following image attributes.

Table 46-45: Secure Boot Image Attributes

| ID         | Name        | Description                                                                                                                                                                    | Values                                                                                        | Values                                                                                                |
|------------|-------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------|
| 0x00000000 | Unused      | Unused attribute                                                                                                                                                               | 0x00000000                                                                                    | 0x00000000                                                                                            |
| 0x00000001 | Version     | Version of the attribute format                                                                                                                                                | 0x00000000                                                                                    | 0x00000000                                                                                            |
| 0x00000002 | Rollback ID | Current value of the roll- back counter                                                                                                                                        | 0x00000000                                                                                    | Rollback disabled                                                                                     |
| 0x00000002 | Rollback ID | Current value of the roll- back counter                                                                                                                                        | 0x00000001 - 0x0000001F                                                                       | Current firmware revi- sion. It must be greater than or equal to the value retrieved from OTP.        |
| 0x80000001 | NoRestore   | Controls whether the boot process restored reg- isters back to default val- ues and clears sensitive se- curity related information from the stack and dedi- cated structures. | 0x1                                                                                           | Do not restore registers and clear security infor- mation from stack and dedicated structures.        |
| 0x80000001 | NoRestore   | Controls whether the boot process restored reg- isters back to default val- ues and clears sensitive se- curity related information from the stack and dedi- cated structures. | Other                                                                                         | Restore registers and clear security sensitive informa- tion from the stack and dedicated structures. |
| 0x80000002 | BCODE       | Used by the boot mode drivers that support auto- detection to configure the device from a range of preconfigured settings                                                      | Values between 0x00000000 and 0x0000000F sup- ported.                                         | Values between 0x00000000 and 0x0000000F sup- ported.                                                 |
| 0x80000003 | ECDSA Type  | Used by boot ROMto perform Authentication using ECDSA-224 or ECDSA-256 algorithm.                                                                                              | 224 : Authentication using ECDSA-224 algorithm 256 : Authentication using ECDSA-256 algorithm | 224 : Authentication using ECDSA-224 algorithm 256 : Authentication using ECDSA-256 algorithm         |

## Secure Boot Streams

The secure boot image sections contain the code and data for loading to the various memory regions of the processor. The content is in the form of a boot stream format consisting of block headers that provide a descriptor for the associated block payloads.

The image section within the secure boot stream consists of a standard Boot Loader Stream consisting of block headers and payloads as generated by the supporting tools and utilities.

The secure boot image section can contain application images for just a single core or for multiple cores allowing for a flexible booting strategy.

A single boot image containing program and data for multiple cores permits a full system boot initializing all internal memories without needing a second stage loader approach. If boot images are required to initialize external memories, then a multi-stage loader approach can be required to configure the memory interfaces. The advantage of a single boot image for multiple cores is that authentication and decryption of the boot image requires only one execution. However, it is over a larger boot image.

Multiple single-core boot images permit one core to boot then execute that core's boot image without booting the other cores. This booting strategy can be needed when a single processor must be brought up quickly to deal with some initial tasks before booting the rest of the system. Or, it can be used to initialize extra peripherals to use for external memory interfaces. The core that has previously booted can then control the boot process for the remaining cores.

## Single Core Images

Single core boot images are the result of processing a single core's output from the linker and converting it to a compliant boot stream. The resulting boot stream has a single first block at the beginning of the boot stream and a final block at the end of the boot stream.

The Block Header HDRSIGN field of the resulting boot stream is used to identify which core the image is intended for. This identification is required so that the boot code can set the correct ( RCU\_SVECT0 -RCU\_SVECT2 ) register from the target address in the first block once the authentication is successful. When the boot image is loaded and authentication is successful, the boot code jumps to the location stored in the cores corresponding ( RCU\_SVECT0 -RCU\_SVECT2 ) register.

This boot stream must then be signed using the secure boot utilities resulting in the final secure boot image for a single core.

A second stage loader option is also available giving maximum flexibility both in terms of using ROM functionality, or creating a custom booting strategy. The simplest option is to have the application use adi\_rom\_Boot() to boot the main application image or indeed a third stage loader when one is required.

The boot stream is generated such that all image contents are in the system address space. Core specific L1 memory sections are converted during the boot stream generation process such that they are loaded through the multiprocessor memory space. The core executing the second stage loader can boot an application image intended for another core.

NOTE: The ROM\_BFLAG\_RETURN flag is set when calling the boot routine for the other cores than the core executing the second stage loader then cleared when loading the cores own image. Failure to do so would result in unintentional behavior.

When the core is used to boot a separately generated boot stream for another core, the ROM\_BFLAG\_RETURN flag should be used when calling adi\_rom\_Boot() to instruct the boot kernel to return to the calling application,

which in this case is back to the core that was running the second stage loader. Failure to set this flag results in the core vectoring to the location in its own ( RCU\_SVECT0 -RCU\_SVECT2 ) register which is not the intention.

By default when implementing a scheme such as previously described where one processor is responsible for booting images for another core, the other cores in the system will remain in their existing idle state that they should be placed in prior to the boot commencing. To allow the cores to execute new application, the core that is responsible for booting the other cores must reset the cores via the RCU\_CRCTL then release them again from reset. If there is a requirement to release the core from the reset state, then this must be done within the application code of the running core.

Releasing other cores from reset to run application software while other processors are running a boot process requires careful system design. The drivers used by the boot kernel for the boot process assume the various peripheral and infrastructure resources such as MDMA channels and peripherals are available for use, and are not being used by another core. If the boot routine is being executed while other cores are running applications, then those applications must ensure that all the required boot resources are freed up and remain free for the boot process to complete on the remaining cores.

## Multi-Core Boot Images

Multi-core boot images are generated as a result or processing the multiple linker output files for multiple cores together to create a single compliant boot stream. The resulting boot stream has a first header block at the beginning of each core boot stream and a single final block at the end of the boot stream. The boot stream must only have a single final block to allow the boot kernel to continue processing the entire boot stream. A final block results in the boot kernel terminating triggering the final public key authentication sequence.

The block headers block code HDRSIGN field of the resulting boot stream is used to identify the intended core for the image. This identification is required such that the boot code can set the correct ( RCU\_SVECT0 -RCU\_SVECT2 ) register when the first block is read to set the application start address for that core. When the boot image is loaded and authentication is successful, the core executing the boot sequence jumps to the location stored in the cores corresponding ( RCU\_SVECT0 -RCU\_SVECT2 ) register. However, since the boot stream has multiple first blocks present for each of the cores, the ( RCU\_SVECT0 -RCU\_SVECT2 ) register of the other cores is set for their applications. Upon the booting core running its application, it must release the other cores from reset for them to start running their loaded applications.

When a product supports three or more cores, it is acceptable to create a dual core boot image and then load the other cores later using single core boot images stored elsewhere in the boot source. There are no restrictions on a multi-core boot stream that must contain an application for all the cores in a product.

The resulting multi-core boot stream must then be signed and optionally encrypted resulting in a complaint secure boot stream that can be used to boot a secure device.

Multiple core boot images are advantageous in the fact that applications can be loaded to all cores in the system in a single boot sequence resulting in the requirement to decrypt and authenticate only a single boot image.

## NOTE:

If there is a requirement for a multi-core boot stream to load code to memory spaces that require a memory controller to be initialized that is not supported by the boot process a multi-stage booting approach is required that can initialize the peripheral. This boot stream is then authenticated, and executed prior to the loading of the boot stream.

## Secure Debug Access

The TAPC controller provides a way to restrict access to secure resources of the processor. Secure access through the debug port is protected through a 128-bit security key that must match a key that has been loaded into OTP for access.

To access a locked processor, the TAPC must allow access to the part. The TAPC only allows access to the part if it is provided with a matching key to the data loaded into its TAPC\_USERKEYn registers.

With the processor in a locked state, on initial boot the boot ROM reads either the 128-bit secure\_emu\_key0 or the 128-bit secure\_emu\_key1 from the OTP memory and programs the key into the TAPC\_SDBGKEY0 through the TAPC\_SDBGKEY3 registers before then setting the TAPC\_USERKEY\_CTL.USERKEY\_VALID bit. The TAPC is then able to access a matching outside key to allow access. See Secure Debug Key Processing for more information about secure debug key validation.

- CAUTION: A 16 bit non-zero value programmed on both emu\_key0\_disable and emu\_key1\_disable fields in OTP or a key of 0xFFFFFFFF , 0xFFFFFFFF , 0xFFFFFFFF , 0xFFFFFFFF programmed in both the secure debug key fields provided in OTP results in the boot code bypassing the key load operation entirely. When debug access is ever required, the key must be loaded to the TAPC by the program. When the processor fails to boot (due to corrupted firmware) then there is no debug access. The only way to gain access is to load an authenticated boot image that can then load the required keys prior to attempting to connect with a debugger.

The key is set in OTP using the OTP boot program API to program secure\_emu\_key0 / secure\_emu\_key1 . The key is read and loaded by the ROM in the following sequence:

1. Bits 31:0 of the key in OTP are stored to TAPC\_SDBGKEY0 bits 31:0
2. Bits 63:32 of the key in OTP are stored to TAPC\_SDBGKEY1 bits 31:0
3. Bits 95:64 of the key in OTP are stored to TAPC\_SDBGKEY2 bits 31:0
4. Bits 127:96 of the key in OTP are stored to TAPC\_SDBGKEY3 bits 31:0

Once the ROM has loaded the user key, a test key can be provided to the TAPC through JTAG. Refer to the Emulator manual for details for providing the key.

A key failure indication can be detected through the TAPC\_SDBGKEY\_STAT register. The boot code does not check the key status, nor does it enable any associated interrupts to signal key failure. The boot code continues to boot upon a key failure in a secure manner. The key failure status remains intact so that the application loaded can check for a failed challenge on the debug port.

The boot code can be configured to bypass the loading of the key during the boot sequence by setting the value of secure\_emu\_key0 or secure\_emu\_key1 in the OTP to all ones (see struct OTP\_DATA). In this case, the only way to gain access to the secure resources through the debug port is to load an alternate key using the application. The alternative key must always reside in a secure region of memory. Or, if sent remotely, it should be transmitted over a secure connection.

NOTE: In the ADSP-SC59x processor family, the boot ROM supports two 128-bit secure debug keys, se-cure\_emu\_key0 and secure\_emu\_key1, where only one of them is used by the Boot ROM at a time. Each of the secure debug keys is associated with a bypass key ( emu\_key0\_disable or emu\_key1\_disable ) which decides the validity of the corresponding key. Any non-zero value written into the 16-bit bypass key invalidates that secure debug key.

## Failure Analysis

The boot ROM supports failure analysis on locked parts. Calling adi\_rom\_otp\_fa\_enable() activates this feature. The user must activate this feature on a locked part before sending it for failure analysis. Without activation, failure analysis on a locked part is impossible due to security reasons. Do not enable this feature on open parts because it results in boot failure.

## Boot ROM Errors and Failures

Any errors encountered while processing a secure boot image results in the ROM jumping to the Error Handler with a specific error code. The error can be of any boot specific error or product specific error. The below table shows different types of errors which are supported by the Boot ROM.

Warning: In secured booting, an incorrect key supplied during boot does not cause a boot failure and the processor continues to boot as normal. A program that supplies an incorrect key is unable to gain access to any secure resources of the processor.

Table 46-46: Secure Boot Header

| Error Name                     | Error ID   | Description                                                                                                  |
|--------------------------------|------------|--------------------------------------------------------------------------------------------------------------|
| ROM_BOOT_ERR_GENERAL           | 0x0        | General Boot Error                                                                                           |
| ROM_BOOT_ERR_UNSUPPORTED       | 0x1        | Unsupported configuration (device/mode selection)                                                            |
| ROM_BOOT_ERR_BMODE_INIT        | 0x2        | Initialization failure for the selected boot mode                                                            |
| ROM_BOOT_ERR_BMODE_CONFIG      | 0x3        | Configuration failure for the selected boot mode                                                             |
| ROM_BOOT_ERR_BMODE_HOOK        | 0x4        | Hook function failure                                                                                        |
| ROM_BOOT_ERR_BMODE_REG_FAILURE | 0x5        | Bmode registration failure                                                                                   |
| ROM_BOOT_ERR_SEC_AUTH_FAIL     | 0x6        | Authentication failure                                                                                       |
| ROM_BOOT_ERR_SEC_DECRYPT       | 0x7        | Decryption Failure                                                                                           |
| ROM_BOOT_ERR_ID                | 0x8        | The image being loaded has a lower ID than was pre- viously recorded, indicating the image ID is outda- ted. |

Table 46-46: Secure Boot Header (Continued)

| Error Name                         | Error ID   | Description                                                      |
|------------------------------------|------------|------------------------------------------------------------------|
| ROM_BOOT_ERR_CLEANUP               | 0x9        | Cleanup failure                                                  |
| ROM_BOOT_ERR_HDRSGN                | 0xA        | Block header signature failure                                   |
| ROM_BOOT_ERR_HOOK                  | 0xB        | Hook function failure                                            |
| ROM_BOOT_ERR_HDRCHK                | 0xC        | Header checksum failure                                          |
| ROM_BOOT_SECURE_ERR_HDRCHK         | 0xD        | Secure header failure                                            |
| ROM_BOOT_ERR_NOFINAL               | 0xE        | No final block                                                   |
| ROM_BOOT_ERR_BLKFILL               | 0xF        | Failure to process fill block                                    |
| ROM_BOOT_ERR_BLKCRC                | 0x10       | Failed crc for a block                                           |
| ROM_BOOT_ERR_DMA                   | 0x12       | DMAtransfer failure                                              |
| ROM_BOOT_ERR_CALLBACK              | 0x13       | Callback failure                                                 |
| ROM_BOOT_ERR_NEXTDXE               | 0x14       | Next dxe failure                                                 |
| ROM_BOOT_ERR_PLLCFG                | 0x15       | PLL configuration failure                                        |
| ROM_BOOT_ERR_INT                   | 0x16       | Internal ROMerror                                                |
| ROM_BOOT_ERR_NOKERNEL              | 0x17       | Application returned to the bootrom                              |
| ROM_BOOT_ERR_OTPREAD               | 0x18       | Failure to read OTP                                              |
| ROM_BOOT_ERR_NODEVICE              | 0x19       | No device selected in the bootCommand                            |
| ROM_BOOT_ERR_NOBMODE               | 0x1A       | No boot mode selected                                            |
| ROM_BOOT_ERR_BMODEDIS              | 0x1B       | Boot mode is disabled                                            |
| ROM_BOOT_ERR_NOPUBKEY              | 0x1C       | No public key                                                    |
| ROM_BOOT_ERR_NOPVTKEY              | 0x1D       | All private keys have been disabled                              |
| ROM_BOOT_ERR_SPICFG                | 0x1E       | Failed SPI configuration                                         |
| ROM_BOOT_ERR_UARTCFG               | 0x1F       | Failed UART configuration                                        |
| ROM_BOOT_ERR_SECURITY_FAILURE      | 0x20       | Failure to initialize secure features                            |
| ROM_BOOT_ERR_ECDSATYPE             | 0x21       | Wrong ECDSA Type detected other than 224 and 256                 |
| ROM_BOOT_ERR_OTPLOCKBIT            | 0x22       | Error in OTP area holding the Lock and Guard bits                |
| ROM_BOOT_ERR_INVALID_FAEN          | 0x23       | Error due to Failure Analysis Enable bit being set in open part  |
| ROM_BOOT_ERR_SEC_HASH_COMPARE_FAIL | 0x24       | Error due to hash compare fail between preHash and Computed Hash |
| ROM_BOOT_ERR_A5_UNDEF_IRQ          | 0x25       | Undefined instruction exception                                  |
| ROM_BOOT_ERR_A5_SWI_IRQ            | 0x26       | Software interrupt instruction                                   |

Table 46-46: Secure Boot Header (Continued)

| Error Name                    | Error ID   | Description                                              |
|-------------------------------|------------|----------------------------------------------------------|
| ROM_BOOT_ERR_A5_PREFETCH_IRQ  | 0x27       | Prefetch abort exception                                 |
| ROM_BOOT_ERR_A5_ABORT_IRQ     | 0x28       | Data abort exception                                     |
| ROM_BOOT_ERR_A5_NORM_IRQ      | 0x29       |                                                          |
| ROM_BOOT_ERR_A5_FAST_IRQ      | 0x2A       |                                                          |
| ROM_BOOT_ERR_SH_RCU_SVECT_IRQ | 0x2B       |                                                          |
| ROM_BOOT_ERR_SH_PARI_IRQ      | 0x2C       | L1 Parity Error                                          |
| ROM_BOOT_ERR_SH_ILOPI_IRQ     | 0x2D       | Illegal opcode error                                     |
| ROM_BOOT_ERR_SH_CB7I_IRQ      | 0x2E       | Software stack (Circular Buffer 7) Overflow              |
| ROM_BOOT_ERR_SH_IICDI_IRQ     | 0x2F       | Unaligned LW/BW access + unintentional CMMR/ SMMR access |
| ROM_BOOT_ERR_SH_SOVFI_IRQ     | 0x30       | Status loop or mode stack overflow; or PC stack full     |
| ROM_BOOT_ERR_SH_ILADI_IRQ     | 0x31       | Illegal Address Space detected                           |
| ROM_BOOT_ERR_SH_IIR2_IRQ      | 0x32       | IIR 2 error                                              |
| ROM_BOOT_ERR_SH_IIR3_IRQ      | 0x33       | IIR 3 error                                              |
| ROM_BOOT_ERR_SH_TMZHI_IRQ     | 0x34       | Core Timer error                                         |
| ROM_BOOT_ERR_SH_BKPI_IRQ      | 0x35       | User Hardware Breakpoint                                 |
| ROM_BOOT_ERR_SH_FIR_IRQ       | 0x36       | FIR error                                                |
| ROM_BOOT_ERR_SH_IIR0_IRQ      | 0x37       | IIR 0 error                                              |
| ROM_BOOT_ERR_SH_SECI_IRQ      | 0x38       | System event controller interrupt                        |
| ROM_BOOT_ERR_SH_IIR1_IRQ      | 0x39       | IIR 1 error                                              |
| ROM_BOOT_ERR_SH_DAI0SPT1      | 0x3A       |                                                          |
| ROM_BOOT_ERR_SH_DAI1SPT0      | 0x3B       |                                                          |
| ROM_BOOT_ERR_SH_DAI1SPT1      | 0x3C       |                                                          |
| ROM_BOOT_ERR_SH_RINSEQI_IRQ   | 0x3D       | Restricted Instruction Sequence                          |
| ROM_BOOT_ERR_SH_CB15I_IRQ     | 0x3E       | Circular Buffer 15 Overflow                              |
| ROM_BOOT_ERR_SH_TMZLI_IRQ     | 0x3F       | Core Timer (Low Priority Option)                         |
| ROM_BOOT_ERR_SH_FIXI_IRQ      | 0x40       | Fixed-point overflow exception                           |
| ROM_BOOT_ERR_SH_FLTOI_IRQ     | 0x41       | Floating-point overflow exception                        |
| ROM_BOOT_ERR_SH_FLTUI_IRQ     | 0x42       | Floating-point underflow exception                       |
| ROM_BOOT_ERR_SH_FLTII_IRQ     | 0x43       | Floating-point invalid exception                         |
| ROM_BOOT_ERR_SH_EMULI_IRQ     | 0x44       | Emulator low priority interrupt                          |
| ROM_BOOT_ERR_SH_SFT0I_IRQ     | 0x45       | User software interrupt 0                                |

Table 46-46: Secure Boot Header (Continued)

| Error Name                | Error ID   | Description               |
|---------------------------|------------|---------------------------|
| ROM_BOOT_ERR_SH_SFT1I_IRQ | 0x46       | User software interrupt 1 |
| ROM_BOOT_ERR_SH_SFT2I_IRQ | 0x47       | User software interrupt 2 |
| ROM_BOOT_ERR_SH_SFT3I_IRQ | 0x48       | User software interrupt 3 |
| ROM_BOOT_ERR_OTPBOOT_IRQ  | 0x49       |                           |

Table 46-47: Secure Boot Header

| Error Name                     | Error ID   | Description                                                                                            |
|--------------------------------|------------|--------------------------------------------------------------------------------------------------------|
| ROM_BOOT_ERR_GENERAL           | 0x0        | General Boot Error                                                                                     |
| ROM_BOOT_ERR_UNSUPPORTED       | 0x1        | Unsupported configuration (device/mode selection)                                                      |
| ROM_BOOT_ERR_BMODE_INIT        | 0x2        | Initialization failure for the selected boot mode                                                      |
| ROM_BOOT_ERR_BMODE_CONFIG      | 0x3        | Configuration failure for the selected boot mode                                                       |
| ROM_BOOT_ERR_BMODE_HOOK        | 0x4        | Hook function failure                                                                                  |
| ROM_BOOT_ERR_BMODE_REG_FAILURE | 0x5        | Bmode registration failure                                                                             |
| ROM_BOOT_ERR_SEC_AUTH_FAIL     | 0x6        | Authentication failure                                                                                 |
| ROM_BOOT_ERR_SEC_DECRYPT       | 0x7        | Decryption Failure                                                                                     |
| ROM_BOOT_ERR_ID                | 0x8        | The image being loaded has a lower ID than was pre- viously recorded, indicating image ID is out dated |
| ROM_BOOT_ERR_CLEANUP           | 0x9        | Cleanup failure                                                                                        |
| ROM_BOOT_ERR_HDRSGN            | 0xA        | Block header signature failure                                                                         |
| ROM_BOOT_ERR_HOOK              | 0xB        | Hook function failure                                                                                  |
| ROM_BOOT_ERR_HDRCHK            | 0xC        | Header checksum failure                                                                                |
| ROM_BOOT_SECURE_ERR_HDRCHK     | 0xD        | Secure header failure                                                                                  |
| ROM_BOOT_ERR_NOFINAL           | 0xE        | No final block                                                                                         |
| ROM_BOOT_ERR_BLKFILL           | 0xF        | Failure to process fill block                                                                          |
| ROM_BOOT_ERR_BLKCRC            | 0x10       | Failed crc for a block                                                                                 |
| ROM_BOOT_ERR_DMA               | 0x12       | DMAtransfer failure                                                                                    |
| ROM_BOOT_ERR_CALLBACK          | 0x13       | Callback failure                                                                                       |
| ROM_BOOT_ERR_NEXTDXE           | 0x14       | Next dxe failure                                                                                       |
| ROM_BOOT_ERR_PLLCFG            | 0x15       | PLL configuration failure                                                                              |
| ROM_BOOT_ERR_INT               | 0x16       | Internal ROMerror                                                                                      |
| ROM_BOOT_ERR_NOKERNEL          | 0x17       | Application returned to the bootrom                                                                    |
| ROM_BOOT_ERR_OTPREAD           | 0x18       | Failure to read OTP                                                                                    |

Table 46-47: Secure Boot Header (Continued)

| Error Name                                | Error ID   | Description                                                      |
|-------------------------------------------|------------|------------------------------------------------------------------|
| ROM_BOOT_ERR_NODEVICE                     | 0x19       | No device selected in the bootCommand                            |
| ROM_BOOT_ERR_NOBMODE                      | 0x1A       | No boot mode selected                                            |
| ROM_BOOT_ERR_BMODEDIS                     | 0x1B       | Boot mode is disabled                                            |
| ROM_BOOT_ERR_NOPUBKEY                     | 0x1C       | No public key                                                    |
| ROM_BOOT_ERR_NOPVTKEY                     | 0x1D       | All private keys have been disabled                              |
| ROM_BOOT_ERR_SPICFG                       | 0x1E       | Failed SPI configuration                                         |
| ROM_BOOT_ERR_UARTCFG                      | 0x1F       | failed UART configuration                                        |
| ROM_BOOT_ERR_SECURITY_FAILURE             | 0x20       | Failure to initialize secure features                            |
| ROM_BOOT_ERR_ECDSATYPE                    | 0x21       | Wrong ECDSA Type detected other than 224 and 256                 |
| ROM_BOOT_ERR_OTPLOCKBIT                   | 0x22       | Error in OTP area holding the Lock and Guard bits                |
| ROM_BOOT_ERR_INVALID_FAEN                 | 0x23       | Error due to Failure Analysis Enable bit being set in open part  |
| ROM_BOOT_ERR_SEC_HASH_COMPARE_FAIL        | 0x24       | Error due to hash compare fail between preHash and Computed Hash |
| ROM_BOOT_ERR_EMSI_LOAD_FAIL               | 0x25       | EMSI Load Routine Failure                                        |
| ROM_BOOT_ERR_A55_SYNC_EXCEPTION_SP_EL     | 0x26       | Synchronous Exception at current EL level with SP0               |
| ROM_BOOT_ERR_A55_IRQ_SP_EL                | 0x27       | Normal priority interrupt at current EL level with SP0           |
| ROM_BOOT_ERR_A55_FIQ_SP_EL                | 0x28       | Fast priority interrupt at current EL level with SP0             |
| ROM_BOOT_ERR_A55_SERROR_SP_EL             | 0x29       | SError exception at current EL level with SP0                    |
| ROM_BOOT_ERR_A55_SYNC_EXCEPTION_SP_ELX    | 0x2A       | Synchronous Exception at current EL level with SPx               |
| ROM_BOOT_ERR_A55_IRQ_SP_ELX               | 0x2B       | Normal priority interrupt at current EL level with SPx           |
| ROM_BOOT_ERR_A55_FIQ_SP_ELX               | 0x2C       | Fast priority interrupt at current EL level with SPx             |
| ROM_BOOT_ERR_A55_SERROR_SP_ELX            | 0x2D       | SError exception at current EL level with SPx                    |
| ROM_BOOT_ERR_A55_SYNC_EXCEP- TION_AARCH64 | 0x2E       | Synchronous Exception at lower EL level using Aarch64            |
| ROM_BOOT_ERR_A55_IRQ_AARCH64              | 0x2F       | Normal priority interrupt at lower EL level using Aarch64        |
| ROM_BOOT_ERR_A55_FIQ_AARCH64              | 0x30       | Fast priority interrupt at lower EL level using Aarch64          |
| ROM_BOOT_ERR_A55_SERROR_AARCH64           | 0x31       | SError exception at lower EL level using Aarch64                 |

Table 46-47: Secure Boot Header (Continued)

| Error Name                                | Error ID   | Description                                               |
|-------------------------------------------|------------|-----------------------------------------------------------|
| ROM_BOOT_ERR_A55_SYNC_EXCEP- TION_AARCH32 | 0x32       | Synchronous Exception at lower EL level using Aarch32     |
| ROM_BOOT_ERR_A55_IRQ_AARCH32              | 0x33       | Normal priority interrupt at lower EL level using Aarch32 |
| ROM_BOOT_ERR_A55_FIQ_AARCH32              | 0x34       | Fast priority interrupt at lower EL level using Aarch32   |
| ROM_BOOT_ERR_A55_SERROR_AARCH32           | 0x35       | SError exception at lower EL level using Aarch32          |
| ROM_BOOT_ERR_SH_RCU_SVECT_IRQ             | 0x36       | RCU SVECT Exception                                       |
| ROM_BOOT_ERR_SH_PARI_IRQ                  | 0x37       | L1 Parity Error                                           |
| ROM_BOOT_ERR_SH_ILOPI_IRQ                 | 0x38       | Illegal opcode error                                      |
| ROM_BOOT_ERR_SH_CB7I_IRQ                  | 0x39       | Software stack (Circular Buffer 7) Overflow               |
| ROM_BOOT_ERR_SH_IICDI_IRQ                 | 0x3A       | Unaligned LW/BW access + unintentional CMMR/ SMMR access  |
| ROM_BOOT_ERR_SH_SOVFI_IRQ                 | 0x3B       | Status loop or mode stack overflow; or PC stack full      |
| ROM_BOOT_ERR_SH_ILADI_IRQ                 | 0x3C       | Illegal Address Space detected                            |
| ROM_BOOT_ERR_SH_IIR2_IRQ                  | 0x3D       | IIR 2 error                                               |
| ROM_BOOT_ERR_SH_IIR3_IRQ                  | 0x3E       | IIR 3 error                                               |
| ROM_BOOT_ERR_SH_TMZHI_IRQ                 | 0x3F       | Core Timer error                                          |
| ROM_BOOT_ERR_SH_BKPI_IRQ                  | 0x40       | User Hardware Breakpoint                                  |
| ROM_BOOT_ERR_SH_FIR_IRQ                   | 0x41       | FIR error                                                 |
| ROM_BOOT_ERR_SH_IIR0_IRQ                  | 0x42       | IIR 0 error                                               |
| ROM_BOOT_ERR_SH_SECI_IRQ                  | 0x43       | System event controller interrupt                         |
| ROM_BOOT_ERR_SH_IIR1_IRQ                  | 0x44       | IIR 1 error                                               |
| ROM_BOOT_ERR_SH_DAI0SPT1                  | 0x45       |                                                           |
| ROM_BOOT_ERR_SH_DAI1SPT0                  | 0x46       |                                                           |
| ROM_BOOT_ERR_SH_DAI1SPT1                  | 0x47       |                                                           |
| ROM_BOOT_ERR_SH_RINSEQI_IRQ               | 0x48       | Restricted Instruction Sequence                           |
| ROM_BOOT_ERR_SH_CB15I_IRQ                 | 0x49       | Circular Buffer 15 Overflow                               |
| ROM_BOOT_ERR_SH_TMZLI_IRQ                 | 0x4A       | Core Timer (Low Priority Option)                          |
| ROM_BOOT_ERR_SH_FIXI_IRQ                  | 0x4B       | Fixed-point overflow exception                            |
| ROM_BOOT_ERR_SH_FLTOI_IRQ                 | 0x4C       | Floating-point overflow exception                         |
| ROM_BOOT_ERR_SH_FLTUI_IRQ                 | 0x4D       | Floating-point underflow exception                        |
| ROM_BOOT_ERR_SH_FLTII_IRQ                 | 0x4E       | Floating-point invalid exception                          |

Table 46-47: Secure Boot Header (Continued)

| Error Name                        | Error ID   | Description                     |
|-----------------------------------|------------|---------------------------------|
| ROM_BOOT_ERR_SH_EMULI_IRQ         | 0x4F       | Emulator low priority interrupt |
| ROM_BOOT_ERR_SH_SFT0I_IRQ         | 0x50       | User software interrupt 0       |
| ROM_BOOT_ERR_SH_SFT1I_IRQ         | 0x51       | User software interrupt 1       |
| ROM_BOOT_ERR_SH_SFT2I_IRQ         | 0x52       | User software interrupt 2       |
| ROM_BOOT_ERR_SH_SFT3I_IRQ         | 0x53       | User software interrupt 3       |
| ROM_BOOT_ERR_OTPBOOT_IRQ          | 0x54       | OTP Boot Exception              |
| ROM_BOOT_ERR_OTP_INVALIDSTATE_IRQ | 0x55       | OTP Invalidate State Exception  |

## Boot ROM Programming Model

This section describes the programming model for booting the processor. The programming model includes booting functions, API calls, and data structures.

## Boot Mode Driver API

The kernel provides a mechanism to provide a customization of supported boot modes or for implementation of completely new boot modes as second stage boot loaders. This allows programs to customize booting while still taking advantage of the rest of the booting framework. A custom boot mode may provide support for a peripheral that is not supported for boot by the ROM, or it could support one of the same peripherals but with a different configuration.

All the same security features can be supported when using a custom boot mode.

A full boot mode, as perceived by the boot implementation, is a collection of following functions:

1. Register-installs the driver functions listed below so they can be accessed by the boot process
2. Initialization-initialize the boot source
3. Configuration-configure the boot source
4. Load-read from the boot source
5. Cleanup-called after booting

The boot kernel is only aware of functions 2 through 5 and has a requirement to support the Load function. It is this function that is responsible for fetching the boot stream from the boot peripheral. The other functions are used prior to executing the kernel or for cleaning up after the kernel has completed processing the boot stream.

To install a custom boot mode:

1. Create a first stage boot application to define a Load function

2. Use the adi\_rom\_BootKernel() API to call the boot kernel after the boot peripheral and pin muxing has configured. Ensure all the fields of struct ADI\_ROM\_BOOT\_CONFIG are configured accordingly prior to performing the call.

The boot mode can use the pModeData member of ADI\_ROM\_BOOT\_CONFIG to preserve and access shared data across the different function calls if required.

All functions have the following prototype:

```
void apiFunction(ADI_ROM_BOOT_CONFIG* pBootStruct);
```

Another way to support custom boot mode is:

1. Create a first stage boot application to define all Init, Config, Load and Cleanup routines.
2. Use adi\_rom\_Boot() API with hook function installed to update the Init, Config, Load and Clean up functions after the preregister initialization is complete. This is checked by boot ROM to override the above functions inside the ROM\_HOOK\_CALL\_CAUSE: ROM\_HOOK\_REG\_COMPLETE

NOTE: The boot command for the custom boot method is 0xF.

## Load Function

The load function is required to read data from the source into the specified destination, according to the parameBootters given through the configuration struct parameter. The structure provides all of the required information read from the block header, or specified by the kernel to read the block header. The load function often makes use of the DMA APIs in order to simplify the load function implementation.

As the kernel processes the stream, it calls the load function to request data. Initially, the request is for the header, then the kernel makes requests according to the block flags it parses. The load function must only read from the device, and write where requested.

Relevant fields within the ADI\_ROM\_BOOT\_CONFIG object for the load function are (but not limited to): uwDataWidth , pSource , dByteCount , pDestination , loadType .

Custom load functions must meet the following requirements.

- Protect against dByteCount values of zero
- Use multiple DMA units if dByteCount is greater than 65536 and the peripheral does not support byte count transfers greater than 65536
- The pSource and pDestination pointers must be properly updated after loading.

In slave boot modes, the boot kernel uses the address of the dArgument field in the pHeader block as the destination for the required dummy DMAs when payload data is consumed from ROM\_BFLAG\_IGNORE blocks. The load function must read the ARGUMENT word of the block early in the function (if required).

## Initialization/Configuration Function

The Initialization and Configuration functions are called in sequence when calling a boot operation using an already supported boot peripheral via the adi\_rom\_Boot() API. These functions are used to configure the boot peripheral prior to calling the boot kernel. Both functions are called in sequence separated only by a call to a user-defined hook function. This hook function is useful when using built-in boot modes to further customize their function. The initialization and configuration functions are responsible for applying any required settings to any devices in use. For example, pin multiplexing may be required and data or pointers that are used by the load function must be initialized. The specific actions depend on the device and functionality used.

## Cleanup Function

The cleanup function is called after the entire boot stream is read, and the kernel has completed its boot modespecific function. This is only performed when using the adi\_rom\_Boot() API. Resetting any status registers, or device parameters is done to prepare the environment for the execution of the newly loaded application.

## Error Handler

This section describes the default error handler for the ROM including information on how to customize the error handling.

The default error handler eventually puts the core into an idle state. This functionality can be overridden by using an Init Block (see Block Types) to modify the error function point in the struct ADI\_ROM\_BOOT\_CONFIG structure. The error handler has access to the entire boot info structure and receives the instruction address that triggered the error.

When a part is locked, and the boot type has not disabled secure boot, only the default error handler is called.

The expected prototype is:

```
void ErrorFunction(ADI_ROM_BOOT_CONFIG* pBootStruct, void *pFailingAddress);
```

The error handler saves the failing address to the ADI\_ROM\_BOOT\_CONFIG structure then raises the INTR\_SOFT3 fault signaling a fault condition to the system before then entering an endless loop in the boot rom.

NOTE: When using the adi\_rom\_Boot() function to perform a boot action, programs may need to manually configure the INTR\_SOFT3 fault signaling depending on the previous application software executed. Calling the boot process using adi\_rom\_Boot() does not result in a reset of the SEC and Fault installation and configuration as described in Preboot Operations.

## Page Mode

Page operations are beneficial for page oriented boot source devices, and to improve boot performance for secure boot operations. Page mode optimizes memory reads for block organized devices by always reading a page, rather than reading data on demand. Two 1024 byte buffers are used in page mode where the contents of one buffer are processed by the boot kernel while DMA is used to load the next data into the second buffer.

Blocking DMA is used to load the active buffer, forcing the process to pause until the DMA is complete. Nonblocking DMA is used to load the not active buffer, allowing the active buffer to process while loading the new data in parallel.

Page mode can be enabled when calling a boot mode using adi\_rom\_Boot() . Refer to the API documentation for the various modes supported by this API. Additionally programs can set the flag using the dFlags function (see struct ADI\_ROM\_BOOT\_CONFIG) when using hook functions

NOTE: Do not customize page mode settings from the default installed by the boot process.

## Boot Hook Function

The boot software allows installation of callback hooks using the adi\_rom\_Boot() APIs hook function parameter. Using this feature, it is possible to alter the state of the processor at different stages of the boot process and customize the boot structures to alter the behavior of the boot process.

The hook function must adhere to the following prototype:

```
nt32_t hookFunction(ADI_ROM_BOOT_CONFIG* pBootconfig,
```

```
ROM_HOOK_CALL_CAUSE cause);
```

By modifying settings in the ADI\_ROM\_BOOT\_CONFIG structure, many alterations of the boot process can be achieved. Much of the same functionality that is available in an Init Block can be provided through the hook function, with even more flexibility for customization. The hook function is called once after executing the boot modes Init routine then once again after executing the boot modes Config routine. A flag passed to the hook function allows software to determine at which point the call took place to allow for conditional processing to occur at different stages of the setup phase.

The hook function must return a zero value for normal booting to continue. A non-zero return value causes the ROM to omit loading any data and immediately transfer control according to Boot Termination and Application Execution .

When the hook function is called, a parameter is passed that indicates why the hook function was called. See enum ROM\_HOOK\_CALL\_CAUSE for more details.

## enum ROM\_HOOK\_CALL\_CAUSE

Enumeration Type Declaration: ROM\_HOOK\_CALL\_CAUSE

Passed to a user hook routine to indicate the reason of the call.

An optional hook routine is provided as a callback when calling a boot mode via adi\_rom\_Boot. This hook routine is called by the boot software first after the execution of the boot modes initialization routine then again after execution of the boot modes configuration routine. This parameter allows the users routine to identify at which point the call was made allowing the user to perform different actions for each call.

Table 46-48: ROM\_HOOK\_CALL\_CAUSE Members

| Enumerator                    | Description                                                                        |
|-------------------------------|------------------------------------------------------------------------------------|
| ROM_HOOK_CALL_INIT_COMPLETE   | Call was because of the completion of the boot modes initialization function       |
| ROM_HOOK_CALL_CONFIG_COMPLETE | Call was because of the completion of the boot modes configuration function        |
| ROM_HOOK_REG_COMPLETE         | Call was because of the completion of the boot modes pre-register initi- alization |

## Boot Return Feature

The adi\_rom\_Boot() API provides a feature to bypass calling the loaded application when boot completes, and to return to the routine that made the call instead. The boot software returns the next address after the last loaded application block in the boot source when this feature is enabled.

To enable this feature, set the ROM\_BFLAG\_RETURN flag in the adi\_rom\_Boot() flags argument when calling the API.

## Boot Termination and Application Execution

When the boot kernel completes the processing of the boot stream, a sequence of events is required to then pass control to the loaded application.

When the boot process is complete, the core is required to vector to the application start address and start executing the newly loaded application. Typically, the first block of a boot stream, which is marked with the BFLAG\_FIRST flag, contains the address of the application. In a multi-core system there may be multiple first blocks in the bootstream indicating the start address of the application for each core. The application entry point for each core is loaded into the cores corresponding RCU\_SVECTn register.

Upon boot completion only the core that performed the boot process will vector and start executing the loaded application. This core must then manage the process of resetting then releasing from the reset the other cores in the system in order to make them execute their newly loaded applications.

Execution of the loaded application can be bypassed when calling the boot mode using adi\_rom\_Boot() and setting the ROM\_BFLAG\_RETURN flag.

Table 46-49: Application Entry Point Registers

|   Core ID | Corresponding RCU_SVECTn Register   |
|-----------|-------------------------------------|
|         0 | RCU_SVECT0                          |
|         1 | RCU_SVECT1                          |
|         2 | RCU_SVECT2                          |

## Boot ROM OTP Customizations

The boot ROM provides a mechanism through available non-volatile programmable memory (OTP on this processor) to customize different aspects of the boot process. These customizations include overriding default boot-peripheral instance, overriding default peripheral-timing parameters, and disabling boot modes.

Data in the OTP memory controls all ROM customizations. The struct ADI\_ROM\_OTP\_BOOT\_INFO data structure accounts for most of the options.

## CGU Initializations

Refer to CGU Configuration

## Boot Command Customization

Refer to Boot Command Customization

## DMC Configuration

Refer to DMC Configuration

## Secure Boot Customization

All the public and private keys can be invalidated using the various key invalidation fields provided in the ADI\_ROM\_OTP\_BOOT\_INFO structure. This configuration is useful when a new key is required. The boot ROM always uses the lowest valid key enumeration. If key0 is valid, then it is used, if key0 is invalid and key1 is valid, then key1 is used. Refer to Secure Boot for details on the secure boot functionality.

## Disabling Boot Modes

Refer to Boot Mode Disable

## API Reference

The APIs defined in this section are exposed for widespread use.

## adi\_rom\_Boot()

Provides access to boot an application at run-time through a supported peripheral.

## API Details

```
void * adi_rom_Boot( void * pAddress, uint32_t flags, int32_t blockCount, ROM_BOOT_HOOK_FUNC * pHook, uint32_t command )
```

## pAddress

Pointer to source address of the boot stream.

## flags

Global flags applied to the entire boot process

## blockCount

Number of blocks to boot. Zero results in processing until a final block is reached.

## pHook

Pointer to user implemented hook function for enabling callbacks during the registering of the boot mode with the boot kernel

## command

The boot command defining the boot mode to use, the peripheral instance to boot from as well as some boot mode specific configuration

## Returns

The 32-bit address of the next address in the boot source to process

## Function Description

This function is used for any second-stage boot for a currently supported boot mode. It provides options to boot from any peripheral enumeration. In SPI Master boot the function supports using any SPI slave select signal.

Boot modes may support an auto-detection mechanism to identify the type of connected device. This function provides options to bypass such auto-detection and use custom configuration options. Options are also provided to bypass peripheral configurations such as pinmux settings or a peripheral configuration if an existing peripheral is more appropriately configured to allow communication with the boot source.

These features are all provided via the command parameter which is specific for each particular boot mode.

The source address of the boot stream is required for master boot modes that require that an address is issued in order to request data from the boot source. Slave boot modes are under full control of the host and use a handshake mechanism to indicate that the processor is ready to receive data. For boot modes such as UART Slave and SPI Slave this parameter is of little value in regards to the boot process itself. However it can prove useful in debug to see how far through the boot stream the boot process progressed in the event of a boot failure.

- NOTE: The processor supports both SPI Memory-Mapped boot as well as Peripheral based SPI Boot. When the boot mode is called to boot from the memory-mapped boot mode via the command argument, the address must coincide with the processors memory-mapped SPI address space as defined by the processors internal memory map. When using the peripheral based boot mode use the absolute address of the boot stream in flash.

Flags passed using the flags argument are global flags and the operation is applied throughout the entire boot process. These must not be confused with the boot block specific flags which are part of the boot stream and indicate how a particular block in the boot stream is processed. Internally, the boot kernel takes the global flags supplied using this function call and combines them with a boot block's local flags to determine all the operations to perform on a given block. After processing the boot block the local flags are cleared and are ready to populate from the next boot block while the global flags remain.

Table 46-50: Global Flags

| Bit Position   | Flag Name                 | Description                                                                                                                          |
|----------------|---------------------------|--------------------------------------------------------------------------------------------------------------------------------------|
| 18             | ROM_BFLAG_HOOK            | Calls the application supplied hook function after execution of bootmode init and config routines                                    |
| 19             | ROM_BLAG_PAGEMODE         | Enables page mode processing where blocks of data are fetched and processed from internal memory                                     |
| 20             | ROM_BFLAG_NOFIRS- THEADER | Set if calling the boot mode and the first block header has already been fetched and is present in the block header storage location |
| 21             | ROM_BFLAG_HEADER          | Not set by the application, set by the boot code each time it is fetch- ed by a block header                                         |
| 22-25          | Reserved                  | Reserved                                                                                                                             |
| 26             | ROM_BFLAG_SLAVE           | Slave boot mode. Causes different handling of ignore blocks by the kernel                                                            |
| 27             | ROM_BFLAG_WAKEUP          | Enables conditional processing of boot blocks intended for wakeup events but not exclusively                                         |
| 28             | ROM_BFLAG_NEXTDXE         | Parses the stream using the next DXE pointer                                                                                         |
| 29             | ROM_BFLAG_RETURN          | Returns the application after calling the API instead of running the new application                                                 |
| 30             | ROM_BFLAG_NORESET         | No Reset                                                                                                                             |
| 31             | ROM_BFLAG_NORESTORE       | Do not execute the boot peripherals cleanup routine to restore regis- ter contents                                                   |

The blockCount argument specifies the number of blocks to process before terminating the boot process (default = 0x00000000). This value instructs the boot software to continue processing a boot stream until the ROM\_BFLAG\_FINAL flag is set. T o load only a specified number of blocks, programs can use blockCount argument.

When blockCount is used in combination with the ROM\_BFLAG\_NEXTDXE flag then the block count is re purposed as a next application count. The boot kernel navigates the first blocks of multiple boot streams similar to a linked list. When the requested application count is reached, the program returns the pointer to this application in the boot source. This allows programs to use the boot kernel to find a specific application when multiple application boot streams are stored contiguously in the boot source.

The pHook argument is a function pointer to a hook routine. When used with the ROM\_BFLAG\_HOOK global flag the boot mode calls the hook routine after calling the boot modes init and functions in the boot mode driver so that the boot software can be customized, and the configuration changed.

The pHook argument when used with the ROM\_BFLAG\_HOOK global flag can enable new features not supported by the boot software and to install a custom load function or error handler for example.

The command argument describes the boot peripheral to boot from, the peripheral instance and contains additional boot mode specific configuration information and flags specific to the boot mode.

NOTE: When calling a boot mode via this API the program must first ensure that the boot peripheral is configured in the SPU as a secure master. The boot software does not configure the peripheral security via this API so that device security is fully controlled by a dedicated task.

## adi\_rom\_BootKernel()

Calls the boot kernel allowing for implementation of custom boot modes.

## API Details

void * adi\_rom\_BootKernel(ADI\_ROM\_BOOT\_CONFIG * pBoot)

## pBoot

Pointer to the struct ADI\_ROM\_BOOT\_CONFIG boot structure containing the complete context of the boot configuration.

## Returns

Pointer containing the address of the byte immediately following the end of the boot stream.

## Function Description

The boot kernel performs the core processing of the boot stream. The boot kernel calls a load function to load data from the peripheral to the required destination. The boot kernel itself has no concept of what the boot peripheral is or how that peripheral is configured. The kernel calls the registered load function, which must then analyze the boot structure and provide the requested amount of data to the required destination.

The load function called by the kernel is provided via the struct ADI\_ROM\_BOOT\_REGISTRY member of struct ADI\_ROM\_BOOT\_CONFIG .

The boot kernel fetches a boot stream block header then a payload if one exists. The boot kernel takes care of the size of the data being requested and the destination address.

The load function that is registered with the kernel is required to update the pSource . Keeping this control under the load function as opposed to the boot kernel itself allows load functions to better control where the next block of data is fetched in the event the boot stream is fragmented or split into different areas of the boot source.

This function is used to implement a second stage boot loader for a peripheral in which there is no driver support in the boot ROM. The application is responsible for initializing the peripheral and the complete struct ADI\_ROM\_BOOT\_CONFIG object before calling this function. The application is responsible for performing a vector to the newly loaded application on return from the function.

NOTE: Programs must ensure that when a new application is loaded it does not clobber the load function and the part of the software responsible for making the core jump to the start of the newly loaded application.

## adi\_rom\_Crc32Init()

The CRC32 Initcode function in the boot rom that is called to enable CRC32 support of boot stream payloads.

## API Details

```
ROM_BOOT_RESULT adi_rom_Crc32Init(ADI_ROM_BOOT_CONFIG * pBootConfig)
```

## pBootConfig

Pointer to the struct ADI\_ROM\_BOOT\_CONFIG object containing the complete boot configuration

## Returns

Returns the following results

- enum ROM\_BOOT\_RESULT when pBootConfig or pBootConfig -&gt; pHeader are zero
- enum ROM\_BOOT\_RESULT when the callback is registered and lookup table initialized

## Function Description

The boot process supports CRC32 protection of all boot block payloads. In order to enable this feature a global callback must be registered with the boot process using struct ADI\_ROM\_BOOT\_CONFIG and the CRC peripherals look up table initialized from the application's polynomial.

An init block header where the ROM\_BFLAG\_INIT flag is set must be included in the boot stream to enable CRC functions. The ADI\_ROM\_BOOT\_HEADER::pTargetAddress field must be set to the address of this function and the application polynomial is provided using the block's struct ADI\_ROM\_BOOT\_HEADER member.

When the boot kernel processes the init block, it calls this function in the boot ROM, registers the callback with the kernel, and performs the look up table initialization.

CRC functionality is enabled on MDMA channel 1 interfaced to the CRC0 peripheral instance.

## adi\_rom\_Crc32Poly()

Initializes the CRC peripheral for use with the user supplied polynomial.

## API Details

```
ROM_BOOT_RESULT adi_rom_Crc32Poly( uint32_t CrcPoly,
```

```
ROM_BOOT_MDMA_REGS const *const pDma )
```

## CrcPoly

None

## pDma

None

## Function Description

The CRC lookup table must be initialized for the CRC polynomial of choice to prepare the CRC peripheral for use.

## adi\_rom\_GetAddress()

Used to find the location of various look-up tables and data objects used during the boot process.

## API Details

```
int32_t adi_rom_GetAddress(ROM_GETADDR_VALUE value)
```

## value

The enum ROM\_GETADDR\_VALUE enumeration specifying the object to retrieve the address of in the ROM memory

## Returns

The byte address of the object in memory

## Function Description

The function returns the address of the object specified by the enumerator provided as an argument to the function. Using this function can make software more code compatible with future products and silicon revisions.

## adi\_rom\_MemCompare()

Verifies that a block of data is filled with an application supplied 32-bit value.

## API Details

```
ROM_BOOT_RESULT adi_rom_MemCompare( ROM_DMA_MDMA_CONFIG * pDmaCfg, ROM_BOOT_MDMA_REGS const *const pDma )
```

## pDmaCfg

None

## pDma

None

## Function Description

The CRC peripheral is used in compare mode and a source MDMA channel is used to read data from a buffer and supply each 32-bit value to the CRC. The CRC peripheral checks that the incoming 32-bit value matches the 32-bit value to compare against.

## adi\_rom\_MemCopy()

Performs a Memory-to-Memory DMA (MDMA) operation using a source and destination pair of DMA channels.

## API Details

```
ROM_BOOT_RESULT adi_rom_MemCopy( ROM_DMA_MDMA_CONFIG * pDmaCfg, ROM_BOOT_MDMA_REGS const *const pDma )
```

## pDmaCfg

Pointer to the struct ROM\_DMA\_PDMA\_CONFIG object containing the peripheral DMA configuration

## pDma

Pointer to the struct ROM\_BOOT\_MDMA\_REGS objects that provides access to the DMA channel MMRs and associated CRC peripheral

## Returns

Returns the following results

- enum ROM\_BOOT\_RESULT for a successful operation or when byte count is 0 as no operation is performed
- enum ROM\_BOOT\_RESULT if a configuration error was detected in the DMA channel prior to configuring the channels for the new operation
- if a configuration error occurred in the source MDMA channel
- if a configuration error occurred in the destination MDMA channel

## Function Description

The memory copy routine performs transfers of blocks of data from one memory location to another. The routine takes a basic descriptor providing configuration details of the operation to perform using the struct

ROM\_DMA\_MDMA\_CONFIG object passed as the first argument. The second argument is a descriptor that provides

access to the DMA channel's MMR registers and the associated CRC peripheral. When called from the higher level adi\_rom\_MemDma() routine this object is retrieved from the ROM.

NOTE: Applications are expected to make use the adi\_rom\_MemDma() routine for all MDMA operations as there is little additional optional configuration that is supported by using this routine.

## adi\_rom\_MemCrc()

Performs CRC32 verification of a block of data by reading the contents and comparing them with an expected result.

## API Details

```
ROM_BOOT_RESULT adi_rom_MemCrc( ROM_DMA_MDMA_CONFIG * pDmaCfg, ROM_BOOT_MDMA_REGS const *const pDma )
```

## pDmaCfg

Pointer to the struct ROM\_DMA\_PDMA\_CONFIG object containing the peripheral DMA configuration

## pDma

Pointer to the struct ROM\_BOOT\_MDMA\_REGS objects that provides access to the DMA channel's MMRs and associated CRC peripheral

## Function Description

The routine uses an MDMA channel pair source DMA channel and the CRC peripheral to calculate a CRC32 result of a data block using a previously supplied polynomial.

The polynomial is supplied through the adi\_rom\_Crc32Poly() routine to ensure consistent CRC peripheral configuration for the look up table initialization that uses the polynomial and the

NOTE: Applications are expected to make use the adi\_rom\_MemDma() routine for all MDMA operations, there is little additional optional configuration that is supported by using this routine.

## adi\_rom\_MemDma()

Provides access to all MDMA operations supported by the boot ROM implementation.

## API Details

```
ROM_BOOT_RESULT adi_rom_MemDma(ROM_DMA_MDMA_CONFIG * pDmaCfg)
```

## pDmaCfg

Pointer to the struct ROM\_DMA\_PDMA\_CONFIG object containing the MDMA configuration

## Returns

Returns the following results using the enum ROM\_BOOT\_RESULT enumeration.

- ROM\_BOOT\_SUCCESS Success. General success can be used to indicate any general functional success for an operation during the boot process. This must be the return result for a boot mode drivers initialization, configuration, load and cleanup routines when overriding their functionality in second stage boot loaders to use custom functions. . Successful operation or when byte count is 0 as no operation is performed
- ROM\_BOOT\_MDMA\_ID\_ERR Illegal MDMA Channel ID. Returned by adi\_rom\_MemDma() if the MDMA channel ID is not supported. For supported channel IDs, see ROM\_DMA\_MDMA\_ID . A MDMA channel ID is provided that is not supported
- ROM\_BOOT\_CRC\_SUPPORTED\_ERR CRC Not Supported Error. Returned by adi\_rom\_Mem-Dma(), adi\_rom\_MemFill(), adi\_rom\_MemCompare() and adi\_rom\_Crc32Poly() if the supplied DMA configuration specified a MDMA channel that does not support CRC operations. . A CRC operation was attempted on a MDMA channel that does not support CRC
- ROM\_BOOT\_MDMA\_OPERATION\_ERR Illegal MDMA operation Specified. Returned by adi\_rom\_MemDma() if the MDMA operation is not supported. For supported operations, see ROM\_DMA\_MDMA\_OPERATION . The operation is not supported
- ROM\_BOOT\_DMA\_FAILURE DMA Failure. Returned by the DMA routines if an error was detected in the DMA\_STAT.IRQERR prior to setting up a new DMA operation with the newly supplied configuration. . A configuration error is detected in the DMA channel prior to configuring the channel for the new operation
- ROM\_BOOT\_DMA\_FAILURE DMA Failure. Returned by the DMA routines if an error was detected in the DMA\_STAT.IRQERR prior to setting up a new DMA operation with the newly supplied configuration. . A configuration error was detected in the DMA channel and the operation requested involves only a single DMA channel
- A configuration error occurred in the source MDMA channel
- A configuration error occurred in the destination MDMA channel
- ROM\_BOOT\_CRC\_COUNT\_ERR CRC Byte Count was not a multiple 4. The CRC peripheral operates on 32-bit data only and as such all CRC operations must have a byte count that is a multiple of 4. This result is returned by the higher level adi\_rom\_MemDma() routine and the underlying adi\_rom\_MemCompare() and adi\_rom\_MemFill() routines if the byte count is not a multiple of 4 bytes. . A CRC operation is being requested and the byte count is not a multiple of 4
- ROM\_BOOT\_CRC\_FAILURE MDMA CRC32 Failure. Returned by the higher level adi\_rom\_Mem-Dma() routine and the underlying adi\_rom\_MemCompare() routine if the CRC32 result of the block of data did not match the expected result. . A CRC32 verification fails

- ROM\_BOOT\_CRC\_FAILURE MDMA CRC32 Failure. Returned by the higher level adi\_rom\_Mem-Dma() routine and the underlying adi\_rom\_MemCompare() routine if the CRC32 result of the block of data did not match the expected result. . A 32-bit memory compare fails

## Function Description

The supported MDMA operations are listed in enum ROM\_DMA\_MDMA\_OPERATION .

- NOTE: The MDMA and CRC peripherals support on-the-fly CRC32 calculations during the transfer of data from one location to another-MDMA in the boot ROM software does not. For CRC calculations data is instead read back from its final destination and verified.

This function is the main entry point for all the MDMA functions supported by the boot ROM software. The individual functions that are called for each operation type are also exposed using the public API.

- NOTE: Use this function for all operations. The lower level functions allow for some basic reconfiguration of default parameters but such modifications are not required where these basic MDMA operations are required.

The boot ROM has an MDMA configuration data structure that is used to specify the overall MDMA configuration of the processor. It provides details on the DMA channel ID associated with each MDMA channel's source and destination DMA channel. It provides information about CRC support and the CRC peripheral instance that is used for a given MDMA channel. See struct ROM\_BOOT\_MDMA and struct ROM\_BOOT\_MDMA\_REGS for full details of the content stored.

The configuration provided as the only argument to the function is provided in Table 46-70 ROM\_DMA\_MDMA\_CONFIG Members .

For a basic MDMA transfer from source to destination the program needs to configure:

- The struct ROM\_DMA\_MDMA\_CONFIG type as enum ROM\_DMA\_MDMA\_OPERATION
- The MDMA channel to use with the eId MDMA Channel ID , for example
- Set the address of the source data via pSource Source Pointer
- Set the destination address of the source data via pDestination Destination Pointer
- Set the byte count using the ByteCount Byte Count field
- Set eDoneDetect DMA Done Detection Method to enum ROM\_DMA\_DONE\_DETECT\_METHOD to poll for DMA completion
- NOTE: The implementation of the MDMA operations does not support interrupt driven data transfers. The routines are implemented for polling DMA status during the boot process. The boot stream is restricted where byte counts and source and destination address alignment must all be a multiple of 4 bytes. MDMA routines must comply with these restrictions.

To use the CRC32 function of the MDMA routines, the application must first initialize the CRC lookup table from the application supplied polynomial. This operation can be performed by setting

ROM\_DMA\_MDMA\_CONFIG::eOperation type as ROM\_DMA\_MDMA\_OPERATION::ROM\_DMA\_CRC\_LUT\_INIT . If an MDMA channel is specified that does not support CRC function, an error result is returned.

For further details of the individual operations supported see the following API references:

- adi\_rom\_MemCopy() · adi\_rom\_MemCrc() · adi\_rom\_MemFill() · adi\_rom\_MemCompare() · adi\_rom\_Crc32Poly()

## adi\_rom\_MemFill()

Fills a block of memory with a 32-bit user supplied value.

## API Details

```
ROM_BOOT_RESULT adi_rom_MemFill( ROM_DMA_MDMA_CONFIG * pDmaCfg, ROM_BOOT_MDMA_REGS const *const pDma )
```

```
pDmaCfg None pDma None
```

## Returns

## Returns the following results

- enum ROM\_BOOT\_RESULT for a successful operation or when byte count is 0 as no operation to be performed
- enum ROM\_BOOT\_RESULT if a configuration error was detected in the DMA channel prior to configuring the channel for the new operation
- A configuration error occurred in the source MDMA channel
- A configuration error occurred in the destination MDMA channel

## Function Description

The CRC peripheral is configured for fill mode and the destination MDMA channel is configured to fill a block of memory with a fixed 32-bit value.

## adi\_rom\_PeriphDma()

Provides access to any peripherals dedicated DMA channel for receive operations only.

## API Details

```
ROM_BOOT_RESULT adi_rom_PeriphDma(ROM_DMA_PDMA_CONFIG * pDmaCfg)
```

## pDmaCfg

Pointer to the struct ROM\_DMA\_PDMA\_CONFIG object containing the peripheral DMA configuration

## Returns

## Returns the following results

- ROM\_BOOT\_SUCCESS Success. General success can be used to indicate any general functional success for an operation during the boot process. This must be the return result for a boot mode drivers initialization, configuration, load and cleanup routines when overriding their functionality in second stage boot loaders to use custom functions. for a successful operation or when byte count is 0 as no operation is performed
- ROM\_BOOT\_DMA\_ACTIVE DMA Channel is Active Returned only by the peripheral DMA routine when an attempt to run another peripheral DMA operation is attempted and the DMA channel is already running. This is not currently implemented for MDMA operations. if the DMA channel is currently running
- ROM\_BOOT\_DMA\_FAILURE DMA Failure. Returned by the DMA routines if an error was detected in the DMA\_STAT.IRQERR prior to setting up a new DMA operation with the newly supplied configuration. if a configuration error was detected in the DMA channel after starting the DMA operation

## Function Description

The peripheral DMA routine is used by the load routines of boot peripherals that have dedicated DMA channels and do not support MDMA channel pairs. Examples are the SPI when not configured for memory-mapped mode and UART peripherals.

In the boot implementation this routine is called from the peripheral load function to request data from the boot source. The routine supports both polling on DMA completion and non-blocking operation to allow for immediate return after starting the DMA operation and continuing with further processing.

NOTE: The function only supports read operations from the peripheral to memory. Transmit operations from memory to peripheral are not supported

## adi\_rom\_otp\_cfg()

Configures the OTPC to enable read and program operations.

## API Details

```
bool adi_rom_otp_cfg(void)
```

## Function Description

Programs may call this routine to ensure the OTPC is configured correctly for read and write access.

NOTE: The preboot process configures the OTPC for use and there is no direct requirement to call this function when using the OTP .

## adi\_rom\_otp\_get()

Reads the field from OTP as defined by the supplied enum OTPCMD .

## API Details

```
bool adi_rom_otp_get( uint32_t data[]
```

```
OTPCMD cmd, )
```

## cmd

The enum OTPCMD enumeration describes the OTP content to read

## data[]

Pointer to storage area to store the read OTP contents

## Function Description

Programs can read the various fields (Table 46-75 OTPCMD Members) of the OTP via this routine. The supplied enum OTPCMD object is used to specify the object to read.

## adi\_rom\_otp\_lock()

Locks the processor, enabling all security features.

## API Details

```
bool adi_rom_otp_lock(void)
```

## Function Description

This function is used to lock the processor from unauthorized access. Once this function is called the application must supply a secure debug key in order to gain access to the device with debug tools and the processor may only be booted using a secure boot stream.

WARNING: Programs must ensure that the OTP secure boot fields are all programmed. Secure boot can be verified prior to locking the processor. Programs must also provision a secure debug key.

## adi\_rom\_otp\_fa\_enable()

Enables failure analysis feature for a locked part

## API Details

```
bool adi_rom_otp_fa_enable(void)
```

## Function Description

This function sets the bit in OTP and enables failure analysis on the locked part.

WARNING: This API should not be called or executed on an open part; it will result in boot failure.

## adi\_rom\_otp\_pgm()

Programs the OTP Memory with the contents of the struct OTP\_DATA object.

## API Details

```
bool adi_rom_otp_pgm(otp_data * data)
```

## data

Pointer to the ::otp\_data object containing the complete OTP contents to program

## Function Description

The OTP memory is only programmed with values that are not 0. Any items that are 0 are ignored. Programs are expected to use this function for all OTP program operations.

## callback()

Implements custom callbacks to previously loaded code during boot.

## API Details

```
ROM_BOOT_RESULT callback( ADI_ROM_BOOT_CONFIG * pBootConfig, ADI_ROM_BOOT_BUFFER * pBuffer,
```

```
uint32_t nFlags
```

)

## pBootConfig

Pointer to the struct ADI\_ROM\_BOOT\_CONFIG object containing the complete context of the boot procedure

## pBuffer

Pointer to the struct ADI\_ROM\_BOOT\_BUFFER object containing details of the payload associated with the callback

## nFlags

The callback flags as set by the boot kernel

## Function Description

A single callback function may be registered with the boot kernel using pCallBackFunction Pointer to the callback function that is called when processing boot blocks with the callback flag set . This function is then called whenever a block is processed with the ROM\_BFLAG\_CALLBACK flag set. Only a single callback function can be registered for the complete boot process.

Callbacks may be used alongside indirect blocks when post processing of the received boot data is required before sending the data to the final destination. An example of this is compression applied to block payloads. The compressed payload is loaded indirectly to the intermediate buffer where it is decompressed by the callback. The callback can modify the source address and byte count for the final MDMA transfer of the decompressed payload using the pBuffer parameter. The callback returns the boot kernel then handles the final transfer of the uncompressed data to its destination.

There are restrictions on the amount of data that can be loaded using indirect blocks depending on the size of the intermediate buffer. For this reason the nFlags parameter is used to indicate the status of the callback when handling larger blocks of indirect data. The table below defines the supported flags:

| Bit Position   | Flag Name            | Description                                                                                                |
|----------------|----------------------|------------------------------------------------------------------------------------------------------------|
| 0              | ROM_CBFLAG_ DIRECT   | When set indicates the call was from the processing of a block header with the ROM_BFLAG_CALLBACK flag set |
| 1              | ROM_CBLAG_ PAGESTART | Indicates the callback was a result of a fetch of a page of data to the in- termediate buffers             |
| 2              | ROM_CBFLAG_ FIRST    | Set if the first fetch of payload data                                                                     |
| 3              | ROM_CBFLAG_ FINAL    | Set if the final fetch of payload data                                                                     |
| 31:4           | Reserved             | Reserved                                                                                                   |

When a callback block header is received by the boot kernel, a call to the callback is performed with the ROM\_CBFLAG\_DIRECT flag set. If the ROM\_BFLAG\_INDIRECT flag or the ROM\_BFLAG\_PAGEMODE flags are set they indicate the use of indirect or page mode the ROM\_CBFLAG\_FIRST and ROM\_CBFLAG\_FINAL flags are cleared. If the transfer is a direct transfer straight to the final destination and not via the intermediate buffers, then the ROM\_CBFLAG\_FIRST and ROM\_CBFLAG\_FINAL flags are also set.

This allows software to identify a callback call based on the processing of a block header with the ROM\_BFLAG\_CALLBACK flag set.

Callbacks are also called when processing payloads indirectly or when page mode is enabled.

- The ROM\_CBFLAG\_DIRECT flag is cleared when the callback is a result of processing the payload data using the intermediate buffers.
- The ROM\_CBFLAG\_FIRST flag is set if the callback is a result of fetching the first block of data in the payload. This flag is also set if the complete block of data fits in the intermediate buffer.
- If the payload does not fit completely in the intermediate buffers multiple fetches must take place and thus multiple callbacks are generated.
- If no flags are set it indicates a callback on a payload transfer is neither the first nor the last block of data in the payload, so there is still further data in the payload to fetch.
- If only the ROM\_CBFLAG\_FINAL flag is set then it is the final block in a payload transfer.

The following table provides an overview of the flag states and their meaning for the processing of callbacks.

|   ROM_CBFLAG _DIRECT |   ROM_CBFLAG _PAGESTART |   ROM_CBFLAG _FIRST |   ROM_CBFLAG _FINAL | Description                                                                                 |
|----------------------|-------------------------|---------------------|---------------------|---------------------------------------------------------------------------------------------|
|                    1 |                       0 |                   0 |                   0 | Callback as a result of processing a block header with indirect or page- mode enabled       |
|                    1 |                       0 |                   1 |                   1 | Callback as a result of processing a block header with indirect and pa- gemode disabled     |
|                    0 |                       1 |                   0 |                   0 | Callback as a result of fetching a page of data in pagemode                                 |
|                    0 |                       1 |                   0 |                   1 | Callback as a result of fetching a page of data in pagemode and the final page in the block |
|                    0 |                       0 |                   1 |                   0 | Callback as a result of fetching the first part of payload in an indirect payload           |
|                    0 |                       0 |                   0 |                   0 | Callback as a result of fetching an indirect payload, not first or last transfer in payload |

|   ROM_CBFLAG _DIRECT |   ROM_CBFLAG _PAGESTART |   ROM_CBFLAG _FIRST |   ROM_CBFLAG _FINAL | Description                                                                       |
|----------------------|-------------------------|---------------------|---------------------|-----------------------------------------------------------------------------------|
|                    0 |                       0 |                   0 |                   1 | Callback as a result of fetching the final part of payload in an indirect payload |
|                    0 |                       0 |                   1 |                   1 | Callback as a result of fetching the complete payload in an indirect payload      |

## initcode()

Implements custom callbacks to previously loaded code during boot.

## API Details

```
void initcode(ADI_ROM_BOOT_CONFIG * pBootConfig)
```

## pBootConfig

Pointer to the struct ADI\_ROM\_BOOT\_CONFIG object containing the complete context of the boot procedure

## Function Description

Initcode functions are embedded into the boot stream to execute application code during the boot phase. Initcode functions help to optimally configure the CGU or any external memory interface that requires initialization to boot data to those memories.

A boot stream may have any number of initcodes present. The only requirement is that the code must be loaded prior to processing the BFLAG\_INIT block.

The initcode routine is passed by the pointer to the complete boot context so that extensive boot customization tasks can be supported.

## adi\_rom\_idle\_loop()

Jumps to the IDLE loop inside the boot ROM space.

## API Details

void adi\_rom\_idle\_loop(void)

## Function Description

This function is used to jump to the forever idle loop inside the boot ROM space.

## adi\_rom\_CguInit()

Can be used for CGU initialization.

## API Details

```
bool adi_rom_CguInit(const ADI_ROM_OTP_BOOT_CGU_INFO * const pSettings)
```

## pSettings

Pointer to the struct struct ADI\_ROM\_OTP\_BOOT\_CGU\_INFO object containing complete context of CGU initialization

## Function Description

This function is used for CGU initialization.

## adi\_rom\_DmcPhyCalibration()

Used for DMC PHY calibration.

## API Details

```
void  adi_rom_DmcPhyCalibration(ADI_ROM_OTP_DMC_CONFIG *pConfig,int csel_dsel_r)
```

## pConfig

Pointer to the struct struct ADI\_ROM\_OTP\_DMC\_CONFIG object containing the complete context of the DMC PHY calibration

## csel\_dsel\_r

Holds the CSEL to DSEL ratio

## Function Description

This function is used for DMC PHY calibration.

## adi\_rom\_DmcInit()

Used for DMC initialization.

## API Details

```
void adi_rom_DmcInit(ADI_ROM_OTP_DMC_CONFIG *pConfig, uint32_t csel_dsel_r)
```

## pConfig

Pointer to the struct struct ADI\_ROM\_OTP\_DMC\_CONFIG object containing the complete context of the DMC PHY calibration.

## csel\_dsel\_r

Holds the CSEL to DSEL ratio.

## Function Description

This function is used for DMC PHY calibration.

## adi\_rom\_ShaInit()

Initializes the PKTE module for the SHA-224/256 operation.

## API Details

```
void adi_rom_DmcInit(ADI_ROM_OTP_DMC_CONFIG *pConfig, uint32_t csel_dsel_r)
```

## pCrypto

Pointer to Crypto descriptors

IV

Initialization Vector

## Ecdsa\_Word

ECDSA word count for 224/256 bit operation

## Function Description

This API is used for the PKTE initialization for the SHA-224/256 bit operation

## adi\_rom\_Sha()

Computes SHA-224/256 bit digest from the PKTE module.

## API Details

```
adi_rom_Sha(CRYPTO_DESCRIPTORS * pCrypto,uint32_t Ecdsa_Word, void *output_p, void *input_p, int size, bool finalFlag)
```

## pCrypto

Pointer to Crypto descriptors

## Ecdsa\_Word

ECDSA word count for 224/256 bit operation

## output\_p

Pointer to the output buffer

## input\_p

size

Size of the input buffer. It must be a multiple of 512

## finalFlag

Final block or not

## Function Description

This API is used to compute the message digest using SHA-224/256 using the PKTE module.

/* points to bootrom\_jumptable\_aes128\_cbc\_decrypt */

```
typedef ROM_BOOT_RESULT (*_bootrom_jumptable_aes128_cbc_decrypt_t) (CRYPTO_DESCRIPTORS * pCrypto, void *output_p, void *input_p, int size); inline static ROM_BOOT_RESULT adi_rom_Aes128CbcDecrypt(CRYPTO_DESCRIPTORS * pCrypto, void *output_p, void *input_p, int size){ _bootrom_jumptable_aes128_cbc_decrypt_t pTmp = (_bootrom_jumptable_aes128_cbc_decrypt_t) (FUNC_ROM_AES128_CBC_DECRYPT); return (*pTmp)(pCrypto,output_p,input_p,size); )
```

## /* points to bootrom\_jumptable\_aes128\_key\_unwrap */

```
typedef ROM_BOOT_RESULT (*_bootrom_jumptable_aes128_key_unwrap_t) (CRYPTO_DESCRIPTORS * pCrypto, void *WrappedIn_p, void *KeyOut_p, uint32_t KeyBits, void *KWK_p); inline static ROM_BOOT_RESULT adi_rom_Aes128KeyUnwrap(CRYPTO_DESCRIPTORS * pCrypto, void *WrappedIn_p, void *KeyOut_p, uint32_t KeyBits, void *KWK_p){ _bootrom_jumptable_aes128_key_unwrap_t pTmp = (_bootrom_jumptable_aes128_key_unwrap_t) (FUNC_ROM_AES128_KEY_UNWRAP); return (*pTmp)(pCrypto, WrappedIn_p, KeyOut_p, KeyBits, KWK_p); }
```

/* points to bootrom\_jumptable\_aes128\_cbc\_loadkey */

```
typedef void * (*_bootrom_jumptable_aes128_cbc_loadkey_t)(sa_t * pSARecord, uint32_t *key_p); inline static void * adi_rom_Aes128CbcLoadkey(sa_t * pSARecord,
```

Pointer to the input buffer

```
uint32_t *key_p) { _bootrom_jumptable_aes128_cbc_loadkey_t pTmp = (_bootrom_jumptable_aes128_cbc_loadkey_t) (FUNC_ROM_AES128_CBC_LOADKEY); return (*pTmp)(pSARecord,key_p); }
```

## /* points to bootrom\_jumptable\_pka\_init */

```
typedef PKA_Status_t (*_bootrom_jumptable_pka_init_t)( PKA_IOArea_t * const IOArea_p, Device_Handle_t Device,const uint32_t * const Firmware_p,const uint32_t FirmwareWordCount); inline static PKA_Status_t adi_rom_PKA_Init( PKA_IOArea_t * const IOArea_p, Device_Handle_t Device,const uint32_t * const Firmware_p,const uint32_t FirmwareWordCount) { _bootrom_jumptable_pka_init_t pTmp = (_bootrom_jumptable_pka_init_t)(FUNC_ROM_PKA_INIT); return (*pTmp)(IOArea_p,Device,Firmware_p,FirmwareWordCount); }
```

## /* points to bootrom\_jumptable\_ecdsa\_verify\_init */

```
typedef ROM_BOOT_RESULT (*_bootrom_jumptable_ecdsa_verify_init_t)(SBHYBRID_EcdsaContext_t * const Verify_p, const SBIF_ECDSA_PublicKey_t * const PublicKey_p,const SBIF_ECDSA_Signature_t * const Signature_p,uint32_t EcdsaWord); inline static ROM_BOOT_RESULT adi_rom_EcdsaVerifyInit(SBHYBRID_EcdsaContext_t * const Verify_p, const SBIF_ECDSA_PublicKey_t * const PublicKey_p,const SBIF_ECDSA_Signature_t * const Signature_p,uint32_t EcdsaWord) { _bootrom_jumptable_ecdsa_verify_init_t pTmp = (_bootrom_jumptable_ecdsa_verify_init_t) (FUNC_ROM_ECDSA_VERIFY_INIT); return (*pTmp)(Verify_p,PublicKey_p,Signature_p,EcdsaWord); }
```

## /* points to bootrom\_jumptable\_ecdsa\_verify\_set\_digest */

```
typedef void * (*_bootrom_jumptable_ecdsa_verify_set_digest_t) (SBHYBRID_EcdsaContext_t * const Verify_p, uint8_t * Digest_p);
```

```
inline static void * adi_rom_EcdsaVerifySetDigest(SBHYBRID_EcdsaContext_t * const Verify_p, uint8_t * Digest_p) { _bootrom_jumptable_ecdsa_verify_set_digest_t pTmp = (_bootrom_jumptable_ecdsa_verify_set_digest_t) (FUNC_ROM_ECDSA_VERIFY_SET_DIGEST); return (*pTmp)(Verify_p,Digest_p); }
```

## /* points to bootrom\_jumptable\_ecdsa\_verify */

```
typedef SB_Result_t (*_bootrom_jumptable_ecdsa_verify_t) (SBHYBRID_EcdsaContext_t * const Verify_p, uint32_t EcdsaWord); inline static SB_Result_t adi_rom_EcdsaVerify(SBHYBRID_EcdsaContext_t * const Verify_p, uint32_t EcdsaWord) { _bootrom_jumptable_ecdsa_verify_t pTmp = (_bootrom_jumptable_ecdsa_verify_t)(FUNC_ROM_ECDSA_VERIFY); return (*pTmp)(Verify_p, EcdsaWord); }
```

## Data Structures

The programming model for booting the processor uses the data structures defined in this section.

## struct ADI\_ROM\_BOOT\_BUFFER

Structure Type Declaration: ADI\_ROM\_BOOT\_BUFFER

Boot Buffer.

A basic buffer type consisting of a pointer to the buffer and its size

## Table 46-51: ADI\_ROM\_BOOT\_BUFFER Members

| Type    | Name       | Description           |
|---------|------------|-----------------------|
| void *  | pBuffer    | Pointer to the buffer |
| int32_t | dByteCount | Size of the buffer    |

## pBuffer

Pointer to the buffer

## dByteCount

Size of the buffer

## struct ADI\_ROM\_BOOT\_CONFIG

Structure Type Declaration: ADI\_ROM\_BOOT\_CONFIG

The Boot Configuration Object that contains all context for the boot process.

This structure contains the complete context for the boot process. A pointer to this object is passed through many routines and is presented to customizable routines such as initcodes, custom initialization, configuration, load and cleanup routines. The object is passed to error handlers and callbacks to customize and adapt the boot process for specific applications, especially in regards to multi-stage boot loader development.

Table 46-52: ADI\_ROM\_BOOT\_CONFIG Members

| Type              | Name                | Description                                                                                                                               |
|-------------------|---------------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| void *            | pSource             | Source address from where to fetch the next boot data.                                                                                    |
| void *            | pDestination        | Destination address to store the fetched data.                                                                                            |
| int32_t           | dByteCount          | Number of bytes to fetch from the boot source.                                                                                            |
| int32_t           | dFlags              | Control flags related to the boot kernel processing of blocks.                                                                            |
| uint32_t          | ulBlockCount        | Limit of blocks processed during boot.                                                                                                    |
| uint32_t          | ulBlockCurrent      | The number of blocks currently processed by the boot kernel                                                                               |
| void *            | pNextDxe            | Pointer to the next application in the boot stream or the first free location after the boot stream.                                      |
| uint32_t          | uByteAddress        | The destination address converted to the byte address space.                                                                              |
| uint32_t volatile | pControlRegister    | Pointer to the boot peripherals control register.                                                                                         |
| int32_t           | dControlValue       | Storage for the boot peripheral main control value to ena- ble that peripheral in a required configuration.                               |
| uint32_t volatile | pPeripheralBase     | Pointer to the boot peripherals base MMRaddress                                                                                           |
| uint32_t volatile | pAuxControlRegister | Pointer to any register that may be used for auxiliary op- erations such as a timer control register for UART auto- baud detection        |
| uint32_t volatile | pAuxPeripheralBase  | Pointer to the base address of any peripheral used for aux- iliary operations such as the TIMER block                                     |
| uint32_t volatile | pSecControlRegister | Base MMRaddress of the SEC SSI instance associated with the boot peripheral if required for advanced second stage boot loader development |
| ADI_DMA_TypeDef * | pDmaBaseRegister    | Base MMRaddress of the DMAchannel associated with the boot peripheral.                                                                    |

Table 46-52: ADI\_ROM\_BOOT\_CONFIG Members (Continued)

| Type                       | Name            | Description                                                                                                                                      |
|----------------------------|-----------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| ROM_DMA_DONE_DETECT_METHOD | loadType        | Set by the kernel to specify to the boot peripherals load function if it is requesting a blocking or non-blocking DMA                            |
| ROM_DMA_MDMA_CONFIG        | MdmaCfg         | An MDMAdescriptor that is used by the boot kernel for internal MDMAoperations.                                                                   |
| uint16_t                   | uwDataWidth     | The maximum data width supported by the boot periph- erals DMAchannel. Set to 0 for 8-bit, 1 for 16-bit and 2 for 32-bit                         |
| uint16_t                   | uwSrcModifyMult | The source modify multiplier used to set DMA_XMOD for source MDMAoperations or peripheral DMAtransmit operations                                 |
| uint16_t                   | uwDstModifyMult | The destination modify multiplier used to set DMA_XMOD for destination MDMAoperations or pe- ripheral DMAreceive operations                      |
| uint16_t                   | uwUserShort     | Free to use                                                                                                                                      |
| int32_t                    | dUserLong       | Free to use                                                                                                                                      |
| int32_t                    | dReserved0      | Reserved for future use                                                                                                                          |
| void *                     | pModeData       | Pointer to the boot mode specific data structure.                                                                                                |
| int32_t                    | dBootCommand    | The boot command value supplied during the call to the adi_rom_Boot() routine.                                                                   |
| ADI_ROM_BOOT_HEADER*       | pHeader         | Pointer to the boot header storage location where all boot stream block headers eventually reside for processing by the kernel.                  |
| void *                     | pTempBuffer     | Pointer to the internal intermediate buffer. Used for pro- cessing of indirect blocks.                                                           |
| void                       | dReserved1      | Reserved                                                                                                                                         |
| int32_t                    | dTempByteCount  | Size of the internal intermediate buffer in bytes.                                                                                               |
| void *                     | pTempSource     | Current source address that is processed in the internal intermediate buffer.                                                                    |
| int32_t                    | dPageByteCount  | The page size used for page mode processing. On this product page size is fixed to 1024 bytes and this member is not used for the load requests. |
| ADI_ROM_BOOT_INTER_BUFFERS | bootBuffers     | The internal intermediate buffer descriptors required when using indirect and page mode features.                                                |
| ROM_BOOT_REGISTRY          | bootRegistry    | The registry object that is used to register a boot periph- eral routines with the kernel.                                                       |

Table 46-52: ADI\_ROM\_BOOT\_CONFIG Members (Continued)

| Type                       | Name                 | Description                                                                                                                                                 |
|----------------------------|----------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ROM_BOOT_ERROR_FUNC *      | pErrorFunction       | Pointer to the error handler called in the event of an er- ror.                                                                                             |
| ROM_BOOT_CALLBACK_FUNC *   | pCallBackFunction    | Pointer to the callback function that is called when pro- cessing boot blocks with the callback flag set.                                                   |
| ROM_BOOT_CALLBACK_FUNC *   | pCrcFunction         | Pointer to the CRC function that is used to perform CRC validation of the boot stream payload data.                                                         |
| ROM_BOOT_CALLBACK_FUNC *   | pForwardFunction     | Feature not supported on this product.                                                                                                                      |
| ADI_ROM_BOOT_MODES         | bootModes            | Access to all boot mode specific resources.                                                                                                                 |
| void *                     | pLogBuffer           | Pointer to the log buffer. Logging is disabled by default on this product and must be configured from within in- itcodes or hook routines.                  |
| void *                     | pLogCurrent          | The current position within the log buffer. Logging is dis- abled by default on this product and must be configured from within initcodes or hook routines. |
| int32_t                    | dLogByteCount        | The size of the log buffer. Logging is disabled by default on this product and must be configured from within in- itcodes or hook routines.                 |
| ADI_ROM_OTP_BOOT_INFO*     | pOtpBootInfo         | Pointer to the ADI_ROM_OTP_BOOT_INFO boot in- formation block that is read from OTP and contains boot customization options.                                |
| ADI_ROM_BOOT_KEY_TYPE      | keyType              | When set to a specific value allows keys not stored in OTP to perform secure boot evaluation on an open pro- cessor.                                        |
| ADI_ROM_BOOT_TYPE          | bootType             | A key to indicate if the boot type is secure or non-secure for open parts.                                                                                  |
| ROM_SB_IMAGE_TYPE          | secureBootImageType  | The type of secure boot image.                                                                                                                              |
| SBIF_ECDSA_Header_t*       | pSecureHeader        | Pointer to the secure boot stream header that is loaded by the boot peripheral from the boot source during the con- figuration phase.                       |
| ADI_SBIF_ECDSA_PublicKey_t | publicKey            | The public key used for secure boot image authentica- tion.                                                                                                 |
| CRYPTO_DESCRIPTORS         | cryptoDescriptors    | The descriptor items as required for PKTE operations.                                                                                                       |
| SB_StorageArea_t*          | pSB_Storage          | Storage area reserved for some crypto operations.                                                                                                           |
| int32_t                    | secureBytesRemaining | The number of bytes remaining to be processed in the se- cure boot stream.                                                                                  |
| uint32_t[4]                | aesKey               | The 128-bit AES decryption key.                                                                                                                             |
| uint32_t[6]                | aesWrapKey           | The key wrapped key from the BLw secure boot image.                                                                                                         |

Table 46-52: ADI\_ROM\_BOOT\_CONFIG Members (Continued)

| Type        | Name        | Description                                                                                                            |
|-------------|-------------|------------------------------------------------------------------------------------------------------------------------|
| uint32_t[4] | IV          | The IV as read from the secure boot header, required to initialize the PKTE.                                           |
| uint8_t *   | pHash       | Pointer to the output destination of the SHA-224 result that is required for authentication of the secure boot stream. |
| uint8_t *   | esdsaType   | Type of ECDSA algorithm to sign the image.                                                                             |
| uint32_t    | errorReturn | Storage location for the address of the instruction line fol- lowing a call to the error handler.                      |

## pSource

Source address from where to fetch the next boot data.

The source address must be maintained by the boot peripheral's load function. The kernel does not update the source pointer automatically after requesting data. This lets load routines control and change the source address. It is useful when advanced second stage loaders need to change the source address (due to a fragmented boot stream) or to reset the address (if expanding into a second SPI flash device).

During debug it is useful in identifying the block in the boot stream that is currently being processed.

## pDestination

Destination address to store the fetched data.

Used by the boot kernel to indicate the destination address for the fetched data. A boot peripheral's load function must transfer the data to this location before returning back to the kernel. The boot kernel updates this field depending on whether a block header or payload is being fetched. In normal operation mode the kernel loads this field with the storage area location of the block header. After processing the block header loads pDestination with the pTargetAddress Destination address of payload contents read from the fetched block header. When using page mode the destination points to the internal buffers and is then updated to transfer data to the final destination.

## dByteCount

Number of bytes to fetch from the boot source.

The kernel sets this parameter to indicate to the load function the number of bytes requested by the kernel. The kernel is responsible for adjusting the byte count for page mode based accesses. The peripheral load function must return the required number of bytes to the destination address provided.

## dFlags

Control flags related to the boot kernel block processing.

When calling a boot mode using the adi\_rom\_Boot() routine the flags supplied to that routine are used to initialize dFlags . These become global flags that remain set through the entire boot process. When a

block header is received the lower 16 bits of the block header are OR'ed with the global flags. The boot software may clear some flags if it detects they are not compatible with other flags and then writes the resulting flags back to this member. The boot kernel then processes the block payload as instructed by the combination of global and boot block specific flags. Upon completion of the block processing original set of global flags are restored and the process repeated.

## ulBlockCount

Limit of blocks processed during boot.

When calling the boot process the adi\_rom\_Boot() routine can accept a block limit for the number of blocks to process before terminating the boot process. If the block count is set to zero then the boot process will continue until a final block reached indicating end of the boot stream. This member holds the program's specified limit for the number of blocks to process and is used by the boot kernel after processing each block to compare it against the ADI\_ROM\_BOOT\_CONFIG::ulBlockCurrent value. Boot process terminates when ADI\_ROM\_BOOT\_CONFIG::ulBlockCurrent equals ADI\_ROM\_BOOT\_CONFIG::ulBlockCount

## ulBlockCurrent

The number of blocks currently processed by the boot kernel

## pNextDxe

Pointer to the next application in the boot stream or the first free location after the boot stream.

This member is initialized when processing a first block in the boot stream. The dArgument Argument function varies depending on operation field of a first block contains the number of bytes left in the boot stream before we reach the end of that boot stream. This lets this pointer point to the next boot stream or to the first empty location after the boot stream. This allows for a feature when using the adi\_rom\_Boot() routine to find the address of an application in a linked list of boot streams or to find the first empty location after the boot stream.

## uByteAddress

The destination address converted to the byte address space.

This member is used to store the byte address space equivalent of SHARC L1 memory addresses, allowing any core to load content to any SHARC cores L1 memory. The SHARC core in which the load is targeted is determined by the block header.

## pControlRegister

Pointer to the boot peripherals control register.

This can be used by a boot mode peripherals driver in order to gain efficient access to a control MMR register in the boot peripheral.

NOTE: This is not used in this products boot implementation but may be leveraged by developers of second stage boot loaders if required

## dControlValue

Storage for the boot peripheral main control value to enable that peripheral in a required configuration.

This can be used by a boot modes peripheral driver in order to store a control value that can be used to enable the peripheral for a required configuration.

## pPeripheralBase

Pointer to the boot peripherals base MMR address

## pAuxControlRegister

Pointer to any register that may be used for auxiliary operations such as a timer control register for UART autobaud detection

## pAuxPeripheralBase

Pointer to the base address of any peripheral used for auxiliary operations such as the TIMER block

## pSecControlRegister

Base MMR address of the SEC SSI instance associated with the boot peripheral should they be required for advanced second stage boot loader development

## pDmaBaseRegister

Base MMR address of the DMA channel associated with the boot peripheral.

This is used by the boot kernel to gain access to the DMA channels status when using non-blocking DMA operations when page mode or secure boot is required, so it must be set when implementing custom boot loaders in order for the kernel to get access to that peripherals DMA status.

NOTE: When a custom boot peripheral does not support the standard DMA instance, the custom driver is required to set up a DMA instance in SRAM that this location points to and the load function would need to update the status accordingly to indicate when the DMA was running and when the DMA completed.

## loadType

Set by the kernel to specify to the boot peripherals load function if it is requesting a blocking or non-blocking DMA

## MdmaCfg

An MDMA descriptor that is used by the boot kernel for internal MDMA operations.

The boot kernel may be required to perform internal MDMA operations outside the control of the boot peripheral driver. Such operations include processing of fill blocks CRC callbacks for CRC verification and MDMA operations from the internal intermediate buffers for indirect block and page mode processing.

NOTE: Users must not use this item or reconfigure this item when developing custom boot drivers, it is intended purely for the internal use by the boot kernel

## uwDataWidth

The maximum data width supported by the boot peripherals DMA channel. Set to 0 for 8-bit, 1 for 16-bit and 2 for 32-bit

## uwSrcModifyMult

The source modify multiplier used to set DMA\_XMOD for source MDMA operations or peripheral DMA transmit operations

## uwDstModifyMult

The destination modify multiplier used to set DMA\_XMOD for destination MDMA operations or peripheral DMA receive operations

## uwUserShort

Free to use by the user

## dUserLong

Free to use by the user

## pModeData

Pointer to the boot mode specific data structure.

Can be set by a boot peripheral driver to allow for a single point of access to the boot mode specific object containing control and configuration information specific to that single boot mode.

## dBootCommand

The boot command value supplied during the call to the adi\_rom\_Boot() routine

## pHeader

Pointer to the boot header storage location where all boot stream block headers eventually reside for processing by the kernel

## pTempBuffer

Pointer to the internal intermediate buffer. Used for processing of indirect blocks

## dTempByteCount

Size of the internal intermediate buffer in bytes

## pTempSource

Current source address that is being processed in the internal intermediate buffer

## dPageByteCount

The page size used for page mode processing. On this product page size is fixed to 1024 bytes and this member is not used for the load requests

## bootBuffers

The internal intermediate buffer descriptors required when using indirect and page mode features

## bootRegistry

The registry object that is used to register a boot peripherals routine with the kernel.

When using the adi\_rom\_Boot() function the boot software calls a peripherals initialization, configuration, load, and cleanup routines from the pointers stored in this object. The kernel itself only makes calls to the load function for the peripheral, so when using the adi\_rom\_BootKernel() function the load function that is called by the boot kernel to fetch data from the boot source must be registered using pLoadFunctionPointer to the boot modes Load function .

## pErrorFunction

Pointer to the error handler called in the event of an error

## pCallBackFunction

Pointer to the callback function that is called when processing boot blocks with the callback flag set

## pCrcFunction

Pointer to the CRC function that is used to perform CRC validation of the boot stream payload data

## pForwardFunction

Feature not supported on this product

## bootModes

Access to all boot mode specific resources

## pLogBuffer

Pointer to the log buffer. Logging is disabled by default on this product and thus must be configured from within initcodes or hook routines

## pLogCurrent

The current position within the log buffer. Logging is disabled by default on this product and thus must be configured from within initcodes or hook routines

## dLogByteCount

The size of the log buffer. Logging is disabled by default on this product and thus must be configured from within initcodes or hook routines

## pOtpBootInfo

Pointer to the struct ADI\_ROM\_OTP\_BOOT\_INFO boot information block that gets read from OTP and contains boot customization options

## keyType

When set to a specific value allows keys not stored in OTP to be used for secure boot evaluation on an open processor.

By default when performing boot on an open part the decryption keys and the public key are fetched from OTP . By setting this field to enum ADI\_ROM\_BOOT\_KEY\_TYPE users can disable the fetching of the keys from OTP and instead provision the keys directly in the ADI\_ROM\_BOOT\_CONFIG::publicKey and ADI\_ROM\_BOOT\_CONFIG::aesKey members via hook routines when using the adi\_rom\_Boot() function.

## bootType

A key to indicate if the boot type is secure or non-secure for open parts

## secureBootImageType

The type of secure boot image

## pSecureHeader

Pointer to the secure boot stream header that is loaded by the boot peripheral from the boot source during the configuration phase

## publicKey

The public key used for secure boot image authentication

## cryptoDescriptors

The descriptor items as required for the PKTE operations

## pSB\_Storage

Storage area reserved for some crypto operations

## secureBytesRemaining

The number of bytes remaining to be processed in the secure boot stream

## aesKey

The 128-bit AES decryption key

## aesWrapKey

The key wrapped key from the BLw secure boot image

## IV

The IV as read from the secure boot header as required to initialize the PKTE

## pHash

Pointer to the output destination of the SHA-224 result that is required for authentication of the secure boot stream

## ecdsaType

This holds the ecdsa type to be used for authentication. It can be 224-bit or 256-bit.

## errorReturn

Storage location for the address of the instruction line following a call to the error handler

## struct ADI\_ROM\_BOOT\_HEADER

Structure Type Declaration: ADI\_ROM\_BOOT\_HEADER

Boot Block Header.

Boot block headers control the loading process of the boot stream, For full details on the contents of the block header and supported flags see Boot Stream.

Table 46-53: ADI\_ROM\_BOOT\_HEADER Members

| Type    | Name           | Description                                          |
|---------|----------------|------------------------------------------------------|
| int32_t | dBlockCode     | Instructs the boot kernel how to process the block.  |
| void *  | pTargetAddress | Destination Address of Payload                       |
| int32_t | dByteCount     | Byte Count of the Payload                            |
| int32_t | dArgument      | Argument functionality varies depending on operation |

## dBlockCode

Instructs the boot kernel how to process the block.

Contains a number of fields for verification of the block header and flags to indicate the type of block. This allows the kernel to process the block correctly.

## pTargetAddress

Destination address of payload

## dByteCount

Byte count of the payload

## dArgument

Argument function varies depending on operation

## struct ADI\_ROM\_BOOT\_INTER\_BUFFER

Structure Type Declaration: ADI\_ROM\_BOOT\_INTER\_BUFFER

The buffer object for the internal intermediate buffers used for indirect and page mode operations.

Table 46-54: ADI\_ROM\_BOOT\_INTER\_BUFFER Members

| Type      | Name     | Description                        |
|-----------|----------|------------------------------------|
| uint8_t * | pBuffer  | Pointer to the buffer              |
| uint32_t  | size     | Size of the buffer                 |
| uint32_t  | pageSize | Page size for block based devices. |

## pBuffer

Pointer to the buffer size

Size of the buffer

## pageSize

Page size for block based devices.

NOTE:

This field is not used in this product. A fixed page size of 1024 bytes is used.

## struct ADI\_ROM\_BOOT\_INTER\_BUFFERS

Structure Type Declaration: ADI\_ROM\_BOOT\_INTER\_BUFFERS

The boot kernels internal buffer object used to access the intermediate buffers and obtain buffer status.

Table 46-55: ADI\_ROM\_BOOT\_INTER\_BUFFERS Members

| Type                          | Name    | Description                                                                               |
|-------------------------------|---------|-------------------------------------------------------------------------------------------|
| ADI_ROM_BOOT_INTER_ BUFFER[2] | buffer  | The two buffer descriptors                                                                |
| ADI_ROM_BOOT_BUFFER_ STATE    | state   | Buffer Status Information                                                                 |
| void *                        | pSource | Original source address pointer of data loaded to active buffer. Not used on this product |
| ADI_DMA_TypeDef *             | pDma    | TBD                                                                                       |

## buffer

The two buffer descriptors

## state

Buffer Status Information

## pSource

Original source address pointer of data loaded to active buffer. Not used on this product

## struct ADI\_ROM\_BOOT\_LINKPORT

Structure Type Declaration: ADI\_ROM\_BOOT\_LINKPORT

The linkport slave boot mode specific structure.

This structure contains all the boot context information that is specific to the linkport slave boot mode.

Table 46-56: ADI\_ROM\_BOOT\_LINKPORT Members

| Type              | Name            | Description                                                                                      |
|-------------------|-----------------|--------------------------------------------------------------------------------------------------|
| uint32_t          | nFlags          | Flags related to linkport boot mode                                                              |
| ADI_LP_TypeDef *  | pRegisters      | Pointer to the LP peripherals base MMRaddress, not used on this product                          |
| ADI_DMA_TypeDef * | pRxDmaRegisters | Pointer to the DMAperipherals base MMRaddress, for receive operations, not used on this product  |
| ADI_DMA_TypeDef * | pTxDmaRegisters | Pointer to the DMAperipherals base MMRaddress, for transmit operations, not used on this product |

## nFlags

Flags related to linkport boot mode

## pRegisters

Pointer to the LP peripherals base MMR address, not used on this product

## pRxDmaRegisters

Pointer to the DMA peripherals base MMR address, for receive operations, not used on this product

## pTxDmaRegisters

Pointer to the DMA peripherals base MMR address, for transmit operations, not used on this product

## struct ADI\_ROM\_BOOT\_MODES

Structure Type Declaration: ADI\_ROM\_BOOT\_MODES

Holds all boot mode specific configuration items.

A boot mode may have requirements for some dedicated storage. This object is used to collect all storage items for all the boot modes supported by the boot ROM.

Table 46-57: ADI\_ROM\_BOOT\_MODES Members

| Type                  | Name     | Description                                |
|-----------------------|----------|--------------------------------------------|
| ADI_ROM_BOOT_SPI      | spi      | Access to all SPI boot mode resources      |
| ADI_ROM_BOOT_UART     | uart     | Access to all UART boot mode resources     |
| ADI_ROM_BOOT_LINKPORT | linkport | Access to all LINKPORT boot mode resources |
| ADI_ROM_BOOT_OSPI     | ospi     | Access to all OSPI boot mode resources     |
| ADI_ROM_BOOT_CUSTOM   | custom   | Access to all custom boot mode resources   |

## spi

## uart

Access to all UART boot mode resources

## linkport

Access to all LINKPORT boot mode resources

## custom

Access to all custom boot mode resources

## Spi3

Access to all OSPI boot mode resources

## struct ADI\_ROM\_BOOT\_REGISTRY

Structure Type Declaration: ADI\_ROM\_BOOT\_REGISTRY

Boot Mode Registration.

Used to hold pointers for the boot mode initialization, configuration, load and cleanup functions. Can customize the registered content via hook routines or install load functions or cleanup functions from within init codes.

When using adi\_rom\_Boot() the boot process makes a call to the initialization function and the configuration function before calling the kernel. The kernel then runs and makes calls to the load function. The cleanup function is called when the kernel reaches the end of the boot stream.

When using the adi\_rom\_BootKernel() function only the load function is called during execution of the software in the boot ROM. All the functions here must return a enum ROM\_BOOT\_RESULT result in order for the boot process to continue. All functions expect a single argument that is the pointer to the boot structure object struct ADI\_ROM\_BOOT\_CONFIG .

Access to all SPI boot mode resources

Table 46-58: ADI\_ROM\_BOOT\_REGISTRY Members

| Type                          | Name             | Description                                       |
|-------------------------------|------------------|---------------------------------------------------|
| ROM_BOOT_MODE_INIT _FUNC *    | pInitFunction    | Pointer to the boot modes Initialization function |
| ROM_BOOT_MODE_ CONFIG_FUNC *  | pConfigFunction  | Pointer to the boot modes Configuration function  |
| ROM_BOOT_MODE_LOAD _FUNC *    | pLoadFunction    | Pointer to the boot modes Load function           |
| ROM_BOOT_MODE_ CLEANUP_FUNC * | pCleanUpFunction | Pointer to the boot modes Cleanup function        |
| void *                        | pReserved        | Reserved for future use                           |
| int32_t                       | dReserved        | Reserved for future use                           |

## pInitFunction

Pointer to the boot modes Initialization function

## pConfigFunction

Pointer to the boot modes Configuration function

## pLoadFunction

Pointer to the boot modes Load function

## pCleanUpFunction

Pointer to the boot modes Cleanup function

## struct ADI\_ROM\_BOOT\_SPI

Structure Type Declaration: ADI\_ROM\_BOOT\_SPI

The SPI Master Boot Mode Specific Structure.

This structure contains all the boot context information that is specific for SPI master boot mode. During autodetection information is copied from the required ::ROM\_SPI\_LUTENTRY item into this structure and used to configure the SPI peripheral for the mode of operation.

Table 46-59: ADI\_ROM\_BOOT\_SPI Members

| Type    | Name           | Description                                                                 |
|---------|----------------|-----------------------------------------------------------------------------|
| uint8_t | ubReadCommand  | Read command to read data from the SPI device                               |
| uint8_t | ubDummyBytes   | Number of dummy bytes to issue after the read command                       |
| uint8_t | ubAddressBytes | Number of address bytes required to access the device                       |
| uint8_t | ubDataBits     | Bus width when reading the data. 0 = single bit, 1 = dual bit, 2 = quad bit |

Table 46-59: ADI\_ROM\_BOOT\_SPI Members (Continued)

| Type                            | Name            | Description                                                                                               |
|---------------------------------|-----------------|-----------------------------------------------------------------------------------------------------------|
| uint16_t                        | uwClkLower      | SPI clock divider value                                                                                   |
| uint16_t                        | uReserved0      | Reserved                                                                                                  |
| uint32_t                        | nTxCtl          | The value written to the SPI_TXCTL register used for address transmit oper- ations such as address cycles |
| uint32_t                        | nRxCtl          | The value written to the SPI_RXCTL register that is used for all receive op- erations                     |
| uint32_t                        | nCmdCtl         | The value written to the SPI_TXCTL register that is used for sending the read command to the SPI Flash    |
| ROM_BOOT_SPIM_IO_ ENABLE_FUNC * | pMIOEnFunction  | Pointer to the function used to enable quad mode on the SPI flash                                         |
| uint8_t                         | nDummy          | The Dummy Byte value if dummy byte transfers are required and the bus is not three-stated                 |
| uint8_t                         | nFlags          | Flags used for additional SPI configuration processing.                                                   |
| uint16_t                        | uReserved2      | Reserved                                                                                                  |
| void *                          | pXIPAddress     | The memory-mapped SPI address to boot from                                                                |
| ADI_SPI_TypeDef *               | pRegisters      | Pointer to the SPI peripherals base MMRaddress, not used on this product                                  |
| ADI_DMA_TypeDef *               | pRxDmaRegisters | Pointer to the DMAperipherals base MMRaddress, for receive operations, not used on this product           |
| ADI_DMA_TypeDef *               | pTxDmaRegisters | Pointer to the DMAperipherals base MMRaddress, for transmit operations, not used on this product          |

## ubReadCommand

Read command to use to read data from the SPI device

## ubDummyBytes

Number of dummy bytes to issue after the read command

## ubAddressBytes

Number of address bytes required to access the device

## ubDataBits

The bus width used when reading the data. 0 for single bit, 1 for dual, 2 for quad

## uwClkLower

The SPI clock divider value

## nTxCtl

The value written to the SPI\_TXCTL register that is used for the address transmit operations such as address cycles

## nRxCtl

The value written to the SPI\_RXCTL register that is used for all receive operations

## nCmdCtl

The value written to the SPI\_TXCTL register that is used for sending the read command to the SPI Flash

## pMIOEnFunction

Pointer to the function used to enable quad mode on the SPI flash

## nDummy

The Dummy Byte value if dummy byte transfers are required and the bus is not tri-stated

## nFlags

Flags used for some additional SPI configuration processing. The flags supported are defined as follows:

|   Bit Position | Name                      | Description                                                                                                                  |
|----------------|---------------------------|------------------------------------------------------------------------------------------------------------------------------|
|              0 | ROM_SPI_FLAGS_CMDSKIP_EN  | When set the configuration routine enables command skip mode where the SPI does not issue a read command for read operations |
|              1 | ROM_SPI_FLAGS_MULTICMD_EN | Configuration routine enables sending command cycles over dual or quad bit bus                                               |

## pXIPAddress

The memory-mapped SPI address to boot from

## pRegisters

Pointer to the SPI peripherals base MMR address, not used on this product

## pRxDmaRegisters

Pointer to the DMA peripherals base MMR address, for receive operations, not used on this product

## pTxDmaRegisters

Pointer to the DMA peripherals base MMR address, for transmit operations, not used on this product

## struct ADI\_ROM\_BOOT\_OSPI

Structure Type Declaration: ADI\_ROM\_BOOT\_OSPI

The SPI Master Boot Mode Specific Structure.

This structure contains all the boot context information that is specific for OSPI master boot mode. During autodetection information is copied from the required ::ROM\_OSPI\_LUTENTRY item into this structure and used to configure the OSPI peripheral for the mode of operation.

Table 46-60: ADI\_ROM\_BOOT\_OSPI Members

| Type               | Name            | Description                                                                                      |
|--------------------|-----------------|--------------------------------------------------------------------------------------------------|
| uint8_t            | ubReadCommand   | Read command to read data from the OSPI device                                                   |
| uint8_t            | ubDummyCycles   | Number of dummy cycles to issue after the read command                                           |
| uint8_t            | ubAddressBytes  | Number of address bytes required to access the device                                            |
| uint8_t            | ubDataBits      | Bus width when reading the data                                                                  |
| uint16_t           | uwClkLower      | OSPI clock divider value                                                                         |
| uint16_t           | uReserved0      | Reserved                                                                                         |
| uint32_t           | nCfg            | Value written to lower 16 bits of the OSPI_CFG register                                          |
| uint32_t           | nDsr            | Value written to OSPI device size register                                                       |
| uint32_t           | nDrir           | Value written to OSPI device read instruction control register                                   |
| uint32_t           | reserved1       | Reserved                                                                                         |
| uint8_t            | nDummy          | The dummy byte value to be used                                                                  |
| uint8_t            | nFlags          | Flags used for some additional SPI configuration processing                                      |
| uint16_t           | uReserved2      | Reserved                                                                                         |
| void *             | pXIPAddress     | The memory-mapped OSPI address to boot                                                           |
| ADI_OSPI_TypeDef * | pRegisters      | Pointer to the OSPI peripherals base MMRaddress, not used on this product                        |
| ADI_DMA_TypeDef *  | pRxDmaRegisters | Pointer to the DMAperipherals base MMRaddress, for receive operations, not used on this product  |
| ADI_DMA_TypeDef *  | pTxDmaRegisters | Pointer to the DMAperipherals base MMRaddress, for transmit operations, not used on this product |

## ubReadCommand

Read command to use to read data from the SPI device

## ubDummyCycles

Number of dummy cycles to issue after the read command

## ubAddressBytes

Number of address bytes required to access the device

## ubDataBits

The bus width used when reading the data. 0 for single bit, 1 for dual, 2 for quad

## uwClkLower

The SPI clock divider value

## nCfg

The value written to the Lower 16 bits of the OSPI Control register

## nDsr

The value written to the OSPI Device Size register

## nDrir

The value written to the OSPI Device Read Instruction Control register

## nDummy

The Dummy Byte value if dummy byte transfers are required and the bus is not tri-stated

## nFlags

Flags used for some additional SPI configuration processing. The flags supported are defined as follows:

- ROM\_SPI\_FLAGS\_CMDSKIP\_EN (Bit position 0): When set the configuration routine enables command skip mode where the SPI does not issue a read command for read operations
- ROM\_SPI\_FLAGS\_MULTICMD\_EN (Bit position 1): Configuration routine enables sending command cycles over dual or quad bit bus

## pXIPAddress

The memory-mapped SPI address to boot

## pRegisters

Pointer to the OSPI peripherals base MMR address (not used on this product)

## pRxDmaRegisters

Pointer to the DMA peripherals base MMR address for receive operations (not used on this product)

## pTxDmaRegisters

Pointer to the DMA peripherals base MMR address, for transmit operations (not used on this product)

## struct ADI\_ROM\_BOOT\_UART

Structure Type Declaration: ADI\_ROM\_BOOT\_UART

The UART slave boot mode specific structure.

This structure contains all the boot context information that is specific for the UART slave boot mode.

Table 46-61: ADI\_ROM\_BOOT\_UART Members

| Type              | Name            | Description                                                                                     |
|-------------------|-----------------|-------------------------------------------------------------------------------------------------|
| uint32_t          | nFlags          | Flags related to UART Boot mode                                                                 |
| ADI_UART_TypeDef* | pRegisters      | Pointer to the UART peripherals base MMRaddress, not used on this product                       |
| ADI_DMA_TypeDef*  | pRxDmaRegisters | Pointer to the DMAperipherals base MMRaddress for receive operations, not used on this product  |
| ADI_DMA_TypeDef*  | pTxDmaRegisters | Pointer to the DMAperipherals base MMRaddress for transmit operations, not used on this product |

## nFlags

Flags related to UART Boot mode

## pRegisters

Pointer to the UART peripherals base MMR address, not used on this product.

## pRxDmaRegisters

Pointer to the DMA peripherals base MMR address for receive operations, not used on this product.

## pTxDmaRegisters

Pointer to the DMA peripherals base MMR address for transmit operations, not used on this product.

## struct ADI\_ROM\_OTP\_BOOT\_CFG

Structure Type Declaration: ADI\_ROM\_OTP\_BOOT\_CFG

The boot configuration object for storing further boot customization objects.

This is a 160-bit structure that is allocated to one contiguous region in the OTP memory array. The functionality allows for individual flags to enable or disable specific features of the boot process. Each flag is allocated in a separate 16-bit word so that each flag can be set at different times and the ECC information will not impact the setting of another flag.

Table 46-62: ADI\_ROM\_OTP\_BOOT\_CFG Members

| Type      | Name                            | Description                            |
|-----------|---------------------------------|----------------------------------------|
| uint32_ t | ctl_WEN:1 (bitfield)            | Enable a write to the CGU_CTL register |
| uint32_ t | div_WEN:1 (bitfield) (bitfield) | Enable a write to the CGU_DIV register |
| uint32_ t | reserved0:1 (bitfield)          | Reserved                               |

Table 46-62: ADI\_ROM\_OTP\_BOOT\_CFG Members (Continued)

| Type      | Name                                  | Description                                                   |
|-----------|---------------------------------------|---------------------------------------------------------------|
| uint32_ t | div_DSEL:5 (bitfield)                 | CGU_DIV.DSEL value                                            |
| uint32_ t | div_CSEL:5 (bitfield) (bitfield)      | CGU_DIV.CSEL value                                            |
| uint32_ t | div_S0SEL:3 (bitfield)                | CGU_DIV.S0SEL value                                           |
| uint32_ t | div_SYSSEL:5 (bitfield)               | CGU_DIV.SYSSEL value                                          |
| uint32_ t | div_S1SEL:3 (bitfield)                | CGU_DIV.S1SEL value                                           |
| uint32_ t | div_OSEL:7 (bitfield)                 | CGU_DIV.OSEL value                                            |
| uint32_ t | div_DF:1 (bitfield)                   | CGU_CTL.DF value                                              |
| uint32_ t | div_MSEL:1 (bitfield) (bitfield)      | CGU_CTL.MSEL value                                            |
| uint32_ t | auto_disable:1 (bitfield)             | Disable polling on auto alignment of clocks (not recommended) |
| uint32_ t | reserved1:16 (bitfield)               | Reserved                                                      |
| uint32_ t | clkoutsel_CLKOUTSEL:5 (bitfield)      | CGU_CLKOUTSEL.CLKOUTSEL value                                 |
| uint32_ t | reserved5:15 (bitfield)               | Reserved                                                      |
| uint32_ t | clkoutsel_WEN:1 (bitfield) (bitfield) | Enable write to the CGU_CLKOUTSEL register                    |
| uint32_ t | reserved2:12 (bitfield)               | Reserved                                                      |
| uint32_ t | oscwctl0_WEN:1 (bitfield)             | Enables write to the instance 0 register                      |
| uint32_ t | oscwctl0_HODF:6 (bitfield)            | Value                                                         |
| uint32_ t | oscwctl0_HODEN:1 (bitfield)           | Value                                                         |
| uint32_ t | oscwctl0_CNGEN:1 (bitfield)           | Value                                                         |
| uint32_ t | oscwctl0_BOUF:1 (bitfield)            | Value                                                         |

Table 46-62: ADI\_ROM\_OTP\_BOOT\_CFG Members (Continued)

| Type      | Name                              | Description                              |
|-----------|-----------------------------------|------------------------------------------|
| uint32_ t | oscwctl0_BOUEN:5 (bitfield)       | Value                                    |
| uint32_ t | oscwctl0_FAULTEN:1 (bit- field)   | Value                                    |
| uint32_ t | oscwctl0_MONDIS:1 (bit- field)    | Value                                    |
| uint32_ t | oscwctl0_FAULTPINDIS:1 (bitfield) | Value                                    |
| uint32_ t | reserved3:15 (bitfield)           | Reserved                                 |
| uint32_ t | oscwctl1_WEN:1 (bitfield)         | Enables write to the instance 1 register |
| uint32_ t | oscwctl1_HODF:6 (bitfield)        | Value                                    |
| uint32_ t | oscwctl1_HODEN:1 (bitfield)       | Value                                    |
| uint32_ t | oscwctl1_CNGEN:1 (bitfield)       | Value                                    |
| uint32_ t | oscwctl1_BOUF:1 (bitfield)        | Value                                    |
| uint32_ t | oscwctl1_BOUEN:5 (bitfield)       | Value                                    |
| uint32_ t | oscwctl1_FAULTEN:1 (bit- field)   | Value                                    |
| uint32_ t | oscwctl1_MONDIS:1 (bit- field)    | Value                                    |
| uint32_ t | oscwctl1_FAULTPINDIS:1 (bitfield) | Value                                    |
| uint32_ t | reserved4:14 (bitfield)           | Reserved                                 |

## lockMonitor

Flag to enable the status checking of ADI private OTP area.

## pubkey0Inv

Invalidate Public Key 0, use next public key for secure boot

## pubkey1Inv

Invalidate Public Key 1, Secure boot will no longer be operational as no further public keys

## privkey0Inv

Invalidate Decryption Key 0, use next Decryption key for secure boot

## privkey1Inv

Invalidate Decryption Key 1, use next Decryption key for secure boot

## privkey2Inv

Invalidate Decryption Key 2, use next Decryption key for secure boot

## privkey3Inv

Invalidate Decryption Key 3, once invalidated part will no longer be bootable

## dmcEn

Enables configuration of the DMC from values in OTP stored in the format of the ADI\_ROM\_OTP\_DMC\_CONFIG object

## dmcInv

Invalidates the DMC values in the OTP resulting is bypassing of DMC configuration

## struct ADI\_ROM\_OTP\_BOOT\_CGU\_INFO

Structure Type Declaration: ADI\_ROM\_OTP\_BOOT\_CGU\_INFO

The CGU configuration object located in OTP for configuration of the CGU by the boot software.

This is a 128-bit structure that is allocated to one contiguous region in the OTP memory array. Allows the boot software to configure the CGU for a more efficient boot process.

Table 46-63: ADI\_ROM\_OTP\_BOOT\_CGU\_INFO Members

| Type     | Name                    | Description                          |
|----------|-------------------------|--------------------------------------|
| uint32_t | ctl_WEN:1 (bitfield)    | Enable write to the CGU_CTL register |
| uint32_t | div_WEN:1 (bitfield)    | Enable write to the CGU_DIV register |
| uint32_t | reserved0:1 (bitfield)  | Reserved                             |
| uint32_t | div_DSEL:5 (bitfield)   | CGU_DIV.DSEL value                   |
| uint32_t | div_CSEL:5 (bitfield)   | CGU_DIV.CSEL value                   |
| uint32_t | div_S0SEL:3 (bitfield)  | CGU_DIV.S0SEL value                  |
| uint32_t | div_SYSSEL:5 (bitfield) | CGU_DIV.SYSSEL value                 |
| uint32_t | div_S1SEL:3 (bitfield)  | CGU_DIV.S1SEL value                  |
| uint32_t | div_OSEL:7 (bitfield)   | CGU_DIV.OSEL value                   |
| uint32_t | ctl_DF:1 (bitfield)     | CGU_CTL.DF value                     |

Table 46-63: ADI\_ROM\_OTP\_BOOT\_CGU\_INFO Members (Continued)

| Type     | Name                              | Description                                                     |
|----------|-----------------------------------|-----------------------------------------------------------------|
| uint32_t | ctl_MSEL:7 (bitfield)             | CGU_CTL.MSEL value                                              |
| uint32_t | auto_disable:1 (bitfield)         | disable polling on auto-alignment of clocks (not recommen- ded) |
| uint32_t | Reserved1:6 (bitfield)            | Reserved                                                        |
| uint32_t | clkoutsel_CLKOUTSEL:5 (bitfield)  | CGU_CLKOUTSEL.CLKOUTSEL value                                   |
| uint32_t | clkoutsel_WEN:1 (bitfield)        | Enable write to the CGU_CLKOUTSEL register                      |
| uint32_t | reserved2:12 (bitfield)           | Reserved                                                        |
| uint32_t | oscwctl0_WEN:1 (bitfield)         | Enable write to the CGU_OSCWDCTL instance 0 register            |
| uint32_t | oscwctl0_HODF:6 (bitfield)        | CGU_OSCWDCTL.HODF value                                         |
| uint32_t | oscwctl0_HODEN:1 (bitfield)       | CGU_OSCWDCTL.HODEN value                                        |
| uint32_t | oscwctl0_CNGEN:1 (bitfield)       | CGU_OSCWDCTL.CNGEN value                                        |
| uint32_t | oscwctl0_BOUF:5 (bitfield)        | CGU_OSCWDCTL.BOUF value                                         |
| uint32_t | oscwctl0_BOUEN:1 (bitfield)       | CGU_OSCWDCTL.BOUEN value                                        |
| uint32_t | oscwctl0_FAULTEN:1 (bitfield)     | CGU_OSCWDCTL.FAULTEN value                                      |
| uint32_t | oscwctl0_MONDIS:1 (bitfield)      | CGU_OSCWDCTL.MONDIS value                                       |
| uint32_t | oscwctl0_FAULTPINDIS:1 (bitfield) | CGU_OSCWDCTL.FAULTPINDIS value                                  |
| uint32_t | reserved3:14 (bitfield)           | Reserved                                                        |
| uint32_t | oscwctl1_WEN:1 (bitfield)         | Enable write to the CGU_OSCWDCTL instance 1 register            |
| uint32_t | oscwctl1_HODF:6 (bitfield)        | CGU_OSCWDCTL.HODF value                                         |
| uint32_t | oscwctl1_HODEN:1 (bitfield)       | CGU_OSCWDCTL.HODEN value                                        |
| uint32_t | oscwctl1_CNGEN:1 (bitfield)       | CGU_OSCWDCTL.CNGEN value                                        |
| uint32_t | oscwctl1_BOUF:5 (bitfield)        | CGU_OSCWDCTL.BOUF value                                         |
| uint32_t | oscwctl1_BOUEN:1 (bitfield)       | CGU_OSCWDCTL.BOUEN value                                        |
| uint32_t | oscwctl1_FAULTEN:1 (bitfield)     | CGU_OSCWDCTL.FAULTEN value                                      |
| uint32_t | oscwctl1_MONDIS:1 (bitfield)      | CGU_OSCWDCTL.MONDIS value                                       |
| uint32_t | oscwctl1_FAULTPINDIS:1 (bitfield) | CGU_OSCWDCTL.FAULTPINDIS value                                  |
| uint32_t | reserved4:14 (bitfield)           | Reserved                                                        |

## ctl\_WEN

Enable write to the CGU\_CTL register

## div\_WEN

Enable write to the CGU\_DIV register

## div\_DSEL

```
CGU_DIV.DSEL value div_CSEL CGU_DIV.CSEL value div_S0SEL CGU_DIV.S0SEL value div_SYSSEL CGU_DIV.SYSSEL value div_S1SEL CGU_DIV.S1SEL value div_OSEL CGU_DIV.OSEL value ctl_DF CGU_CTL.DF value ctl_MSEL CGU_CTL.MSEL value auto_disable disable polling on auto-alignment of clocks, NOT RECOMMENDED! clkoutsel_CLKOUTSEL CGU_CLKOUTSEL.CLKOUTSEL value clkoutsel_WEN Enable write to the CGU_CLKOUTSEL register oscwctl0_WEN Enable write to the CGU_OSCWDCTL instance 0 register oscwctl0_HODF CGU_OSCWDCTL.HODF value oscwctl0_HODEN CGU_OSCWDCTL.HODEN value oscwctl0_CNGEN
```

CGU\_OSCWDCTL.CNGEN value

## oscwctl0\_BOUF

CGU\_OSCWDCTL.BOUF value

## oscwctl0\_BOUEN

CGU\_OSCWDCTL.BOUEN value

## oscwctl0\_FAULTEN

CGU\_OSCWDCTL.FAULTEN value

## oscwctl0\_MONDIS

CGU\_OSCWDCTL.MONDIS value oscwctl0\_FAULTPINDIS

CGU\_OSCWDCTL.FAULTPINDIS value

## oscwctl1\_WEN

Enable write to the CGU\_OSCWDCTL instance 1 register

## oscwctl1\_HODF

CGU\_OSCWDCTL.HODF value

## oscwctl1\_HODEN

CGU\_OSCWDCTL.HODEN value oscwctl1\_CNGEN

CGU\_OSCWDCTL.CNGEN value

## oscwctl1\_BOUF

CGU\_OSCWDCTL.BOUF value

## oscwctl1\_BOUEN

CGU\_OSCWDCTL.BOUEN value

## oscwctl1\_FAULTEN

CGU\_OSCWDCTL.FAULTEN value

## oscwctl1\_MONDIS

CGU\_OSCWDCTL.MONDIS value

## oscwctl1\_FAULTPINDIS

CGU\_OSCWDCTL.FAULTPINDIS value

## struct ADI\_ROM\_OTP\_BOOT\_CMD\_INFO

Structure Type Declaration: ADI\_ROM\_OTP\_BOOT\_CMD\_INFO

The boot command object for storing a customized boot command for each boot mode within the OTP memory array.

This is a 160-bit structure that is allocated to one contiguous region in the OTP memory array. Allows the boot ROM software to pass a custom boot command to a specific boot mode which changes the default boot behavior on startup. Can be used to change the default UART instance used for a UART boot operation (for example).

Table 46-64: ADI\_ROM\_OTP\_BOOT\_CMD\_INFO Members

| Type     | Name              | Description                |
|----------|-------------------|----------------------------|
| uint32_t | spiMasterBootCmd  | SPI Master Boot Mode       |
| uint32_t | spiSlaveBootCmd   | SPI Slave Boot Mode        |
| uint32_t | lpBootCmd         | Link Port Target Boot Mode |
| uint32_t | uartBootCmd       | UART Controller Boot Mode  |
| uint32_t | ospiMasterBootCmd | OSPI Controller Boot Mode  |

## spiMasterBootCmd

SPI Master Boot Mode

spiSlaveBootCmd

SPI Slave Boot Mode

lpBootCmd

Link Port Target Boot Mode

uartBootCmd

UART Controller Boot Mode

ospiMasterBootCmd

OSPI Controller Boot Mode

## struct ADI\_ROM\_OTP\_BOOT\_INFO

Structure Type Declaration: ADI\_ROM\_OTP\_BOOT\_INFO

The 576-bit boot info object located in OTP for boot customization.

This is a 576-bit structure that is allocated to one contiguous region in the OTP memory array. The content in OTP is stored in the format of this structure so boot can read the contents directly into this object.

Programs can read this object using the adi\_rom\_otp\_get() routine supplying the enum OTPCMD enumeration

Table 46-65: ADI\_ROM\_OTP\_BOOT\_INFO Members

| Type                             | Name                 | Description                                                                                                  |
|----------------------------------|----------------------|--------------------------------------------------------------------------------------------------------------|
| struct ADI_ROM_OTP_BOOT_CGU_INFO | cgu                  | CGU Configuration information                                                                                |
| uint32_t                         | flashStartAddress    | Flash Start address for the SPI/OSPI master boot mode                                                        |
| struct ADI_ROM_OTP_BOOT_CMD_INFO | bcmd                 | Boot Command customization for each boot mode to change default boot peripheral instance and configu- ration |
| struct ADI_ROM_OTP_BOOT_CMD_INFO | rdcr                 | Additional OSPI configuration used for OSPI boot mode                                                        |
| uint16_t                         | ospi_rdcr_ reserved0 | Reserved                                                                                                     |
| struct ADI_ROM_OTP_BOOT_CFG      | bcfg                 | Additional boot configuration flags for key invalida- tion and to enable DMCconfiguration                    |
| uint32_t                         | val14                | Reserved                                                                                                     |
| uint16_t                         | otpReadTiming        | Reserved, Must always be zero                                                                                |
| uint16_t                         | reserved1            | Reserved                                                                                                     |

## cgu

CGU Configuration information

## flashStartAddress

Flash start address for the SPI/OSPI master boot mode

## bcmd

Boot Command customization for each boot mode to change the default boot peripheral instance and configuration

## rdcr

## bcfg

Additional boot configuration flags for key invalidation and for enabling DMC configuration

## otpReadTiming

Reserved, Must always be zero

Additional OSPI configuration info that is used for OSPI boot mode

## struct ADI\_ROM\_OTP\_DMC\_CONFIG

Structure Type Declaration: ADI\_ROM\_OTP\_DMC\_CONFIG

The 384-bit configuration object located in OTP for configuration of the DMC during preboot.

Makes use of the memories connected to the DMC peripheral during boot without the use of init codes in the boot stream. The settings can be applied to this object in the OTP memory. During preboot the boot software reads the object from OTP and configures the peripheral accordingly.

NOTE: When the device is open programs should avoid using OTP and instead use initcodes to initialize DMC because the initcode method is highly customizable. To lock the device to enable secure boot (to boot to memories interfaced to the DMC) the configuration must be provisioned to OTP because initcodes are not supported in secure boot.

Table 46-66: ADI\_ROM\_OTP\_DMC\_CONFIG Members

| Type     | Name                     | Description                                |
|----------|--------------------------|--------------------------------------------|
| uint32_t | ulDDR_DLLCTLCFG          | Content of DDR DLLCTL and DMC_CFG register |
| uint32_t | ulDDR_MR2MR3             | Content of the DDR MR2 and MR3 register    |
| uint32_t | ulDDR_CTL                | Content of the DDR Control register        |
| uint32_t | ulDDR_MRMR1              | Content of the DDR MRand MR1 register      |
| uint32_t | ulDDR_TR0                | Content of the DDR Timing register         |
| uint32_t | ulDDR_TR1                | Content of the DDR Timing register         |
| uint32_t | ulDDR_TR2                | Content of the DDR Timing register         |
| uint32_t | ulDDR_ZQCTL0             | Content of ZQCTL0 register                 |
| uint32_t | ulDDR_ZQCTL1             | Content of ZQCTL1 register                 |
| uint32_t | ulDDR_ZQCTL2             | Content of ZQCTL2 register                 |
| uint32_t | ulDDRPHY_CACTL           | Content of DDRPHY_DDR_CA_CTL register      |
| uint32_t | uBypassDelay_LANE0CTL1:6 | Content of DDR_LANE0_CTL1[15:10] register  |
| uint32_t | uBypassDelay_LANE1CTL1:6 | Content of DDR_LANE1_CTL1[15:10] register  |
| uint32_t | uBypassDelay_LANE0CTL0:6 | Content of DDR_LANE0_CTL0[15:10] register  |
| uint32_t | uBypassDelay_LANE1CTL0:6 | Content of DDR_LANE1_CTL0[15:10] register  |
| uint32_t | reserved0:8              | Reserved                                   |

## struct ROM\_BOOT\_DMA\_INSTANCE

Structure Type Declaration: ROM\_BOOT\_DMA\_INSTANCE

DMA Channel Instance.

Specifies the base MMR address of the DMA channel as well as trigger and interrupt IDs

Table 46-67: ROM\_BOOT\_DMA\_INSTANCE Members

| Type              | Name          | Description                                                |
|-------------------|---------------|------------------------------------------------------------|
| ADI_DMA_TypeDef * | pReg          | Pointer to the base address of the DMAchannel MMRregisters |
| DMA_CHANn_TypeDef | eDmaChannelId | The actual DMAchannel ID in the system                     |
| uint8_t           | TriggerId     | The trigger ID associated with the DMAchannel              |
| uint8_t           | InterruptId   | The interrupt ID associated with the DMAchannel            |

## pReg

Pointer to the base address of the DMA channel MMR registers

## eDmaChannelId

The actual DMA channel ID in the system

## TriggerId

The trigger ID associated with the DMA channel

## InterruptId

The interrupt ID associated with the DMA channel

## struct ROM\_BOOT\_MDMA

Structure Type Declaration: ROM\_BOOT\_MDMA

MDMA Channels.

Provides access to all the MDMA channels and CRC peripherals supported by the processor

Table 46-68: ROM\_BOOT\_MDMA Members

| Type                                             | Name   | Description                                                    |
|--------------------------------------------------|--------|----------------------------------------------------------------|
| ROM_BOOT_MDMA_REGS[PARAM_ SYS0_NUM_MDMA_STREAMS] | Stream | Array of MDMAchannel configurations supported by the processor |

## Stream

Array of MDMA channel configurations supported by the processor

## struct ROM\_BOOT\_MDMA\_REGS

Structure Type Declaration:

ROM\_BOOT\_MDMA\_REGS

MDMA Channel Registers.

Contains the Source and Destination MDMA channel instances for access to the MMRs and interrupt and trigger information. Information is also provided on the CRC support of the MDMA channel and access is provided to the corresponding CRC peripheral.

Table 46-69: ROM\_BOOT\_MDMA\_REGS Members

| Type                            | Name        | Description                                                        |
|---------------------------------|-------------|--------------------------------------------------------------------|
| struct ROM_BOOT_DMA_INSTANCE    | Src         | The source DMAChannel in the MDMApair                              |
| ROM_BOOT_DMA_INSTANCE           | Dst         | The destination DMAChannel in the MDMApair                         |
| ADI_CRC_TypeDef *               | pCrc        | The base MMRaddress of the associated CRC peripheral if one exists |
| enum ROM_BOOT_MDMA_CRC_ SUPPORT | eCrcSupport | Indicates if the MDMAchannel supports CRC or not                   |

Src

Dst pCrc

The base MMR address of the associated CRC peripheral if one exists eCrcSupport

Indicates if the MDMA channel supports CRC or not

## struct ROM\_DMA\_MDMA\_CONFIG

Structure Type Declaration: ROM\_DMA\_MDMA\_CONFIG

MDMA Configuration Object.

The configurable structure for controlling the MDMA operation supplied to the adi\_rom\_MemDma() routine.

Table 46-70: ROM\_DMA\_MDMA\_CONFIG Members

| Type                          | Name         | Description                  |
|-------------------------------|--------------|------------------------------|
| enum ROM_DMA_MDMA_OP- ERATION | eOperation   | Type of operation to perform |
| enum ROM_DMA_MDMA_ID          | eId          | MDMAChannel ID               |
| void *                        | pSource      | Source Pointer               |
| void *                        | pDestination | Destination Pointer          |

The source DMA Channel in the MDMA pair

The destination DMA Channel in the MDMA pair

Table 46-70: ROM\_DMA\_MDMA\_CONFIG Members (Continued)

| Type                             | Name        | Description                                                         |
|----------------------------------|-------------|---------------------------------------------------------------------|
| uint32_t                         | ByteCount   | Byte Count                                                          |
| enum ROM_DMA_DONE_DETECT_ METHOD | eDoneDetect | DMADone Detection Method                                            |
| uint32_t                         | CrcCtl      | CRC_CTL value when CRC operations are required                      |
| uint32_t                         | FillVal     | Fill value for memory fill operations                               |
| uint32_t                         | CrcPoly     | CRC Polynomial for CRC operations                                   |
| uint32_t                         | CrcCompare  | Value used for CRC compare operations or for a CRC32 result compare |

## eOperation

Type of operation to perform

## eId

MDMA Channel ID

## pSource

Source Pointer

## pDestination

Destination Pointer

## ByteCount

Byte Count

## eDoneDetect

DMA Done Detection Method

## CrcCtl

CRC\_CTL value when CRC operations are required

## FillVal

Fill value for memory fill operations

## CrcPoly

CRC Polynomial for CRC operations

## CrcCompare

Value used for CRC compare operations or for a CRC32 result compare

## struct ROM\_DMA\_PDMA\_CONFIG

Structure Type Declaration: ROM\_DMA\_PDMA\_CONFIG

PDMA Configuration Object.

The user configurable structure for controlling the PDMA operation via the adi\_rom\_PeriphDma() function.

Table 46-71: ROM\_DMA\_PDMA\_CONFIG Members

| Type                             | Name          | Description                                                                                                    |
|----------------------------------|---------------|----------------------------------------------------------------------------------------------------------------|
| enum ROM_DMA_PDMA_ OPERATION     | eOperation    | Type of operation to perform                                                                                   |
| ADI_DMA_TypeDef volatile *       | pRegs         | Pointer to the base address of the DMAchannel MMRregisters                                                     |
| uint16_t                         | dataWidth     | The maximum supported data width of the DMAchannel. Used to configure the DMA_CFG.PSIZE PSIZE field in DMA_CFG |
| uint16_t                         | dstModifyMult | The modify multiplier, usually set to 1                                                                        |
| void *                           | pSource       | Source Pointer used for transmit operations                                                                    |
| void *                           | pDestination  | Destination Pointer used for receive operations                                                                |
| uint32_t                         | byteCount     | Number of bytes to transfer                                                                                    |
| enum ROM_DMA_DONE_ DETECT_METHOD | eDoneDetect   | DMADone Detection method used for the transfer                                                                 |
| enum ROM_DMA_DONE_ DETECT_METHOD | loadType      | Defines whether the load function DMAwaits for the load to complete or re- turns to the kernel                 |

## eOperation

Type of operation to perform

## pRegs

Pointer to the base address of the DMA channel MMR registers

## dataWidth

The maximum supported data width of the DMA channel. Used to configure the DMA\_CFG.PSIZE PSIZE field in DMA\_CFG

## dstModifyMult

The modify multiplier, usually set to 1

## pSource

Source Pointer used for transmit operations

## pDestination

Destination Pointer used for receive operations

## byteCount

Number of bytes to transfer

## eDoneDetect

DMA Done Detection method used for the transfer

## loadType

Defines whether the load function DMA waits for the load to complete or returns to the kernel

## struct OTP\_DATA

Structure Type Declaration: otp\_data

Container for accessing data to be written to OTP via the adi\_rom\_otp\_pgm() routine.

Any pointers that are NULL will result in the object not being written. Any data values of 0 will not be written.

Table 46-72: otp\_data Members

| Type                                     | Name           | Description                                |
|------------------------------------------|----------------|--------------------------------------------|
| void (*)                                 | reserved       |                                            |
| uint32_t(*) [ROM_OTP_SZ_huk]             | huk            | Pointer to 256-bit Hardware Unique Key     |
| uint32_t(*) [ROM_OTP_SZ_rkek]            | rkek           | Pointer to 128-bit Root Key Encryption Key |
| uint 32_t(*) [ROM_OTP_SZ_dek]            | dek            | Pointer to 128-bit Local Encryption Key    |
| uint32_t(*) [ROM_OTP_SZ_oem_public _key] | oem_public_key | Pointer to 512-bit OEMPublic Key           |
| uint32_t(*) [ROM_OTP_SZ_pvt_ 128key0]    | pvt_128key0    | Pointer to 128-bit AES Key                 |
| uint32_t(*) [ROM_OTP_SZ_pvt_ 128key1]    | pvt_128key1    | Pointer to 128-bit AES Key                 |
| uint32_t(*) [ROM_OTP_SZ_pvt_ 128key2]    | pvt_128key2    | Pointer to 128-bit AES Key                 |

Table 46-72: otp\_data Members (Continued)

| Type                                      | Name                            | Description                                                                            |
|-------------------------------------------|---------------------------------|----------------------------------------------------------------------------------------|
| uint32_t(*) [ROM_OTP_SZ_pvt_ 128key3]     | pvt_128key3                     | Pointer to 128-bit AES Key                                                             |
| uint32_t(*) [ROM_OTP_SZ_ek]               | ek                              | Pointer to 256-bit endorsement key                                                     |
| uint32_t(*) [ROM_OTP_SZ_secure_emu _key0] | secure_emu_key0                 | Pointer to 128-bit Secure Debug Key                                                    |
| uint32_t(*) [ROM_OTP_SZ_secure_emu _key1] | secure_emu_key1                 | Pointer to 128-bit Secure Debug Key                                                    |
| uint32_t                                  | emu_key0_disable :16 (bitfield) | Secure Debug key disable                                                               |
| uint32_t                                  | emu_key1_disable :16 (bitfield) | Secure Debug key disable                                                               |
| uint32_t(*) [ROM_OTP_SZ_public_ key0]     | public_key0                     | Pointer to 512-bit public key used for boot stream authentica- tion                    |
| uint32_t(*) [ROM_OTP_SZ_public_ key1]     | public_key1                     | Pointer to 512-bit public key used for boot stream authentica- tion                    |
| uint32_t(*) [ROM_OTP_SZ_boot_info]        | boot_info                       | Pointer to 608-bit boot customization structure, see also struct ADI_ROM_OTP_BOOT_INFO |
| uint8_t                                   | antiroll_nv_cntr                | Anti-rollback counter to prevent loading of older firmware dur- ing secure boot.       |
| uint32_t(*) [ROM_OTP_SZ_gp1]              | gp1                             | Pointer to 512-bit General purpose user space                                          |
| uint32_t                                  | bootModeDisable: 8 (bitfield)   | Boot mode disable for permanently disabling specific boot modes                        |
| uint32_t(*) [ROM_OTP_SZ_preboot_ ddr_cfg] | preboot_ddr_cfg                 | Pointer to 384-bit DMCconfiguration. See also struct ADI_ROM_OTP_BOOT_INFO             |
| uint32_t(*) [ROM_OTP_SZ_stageID]          | stageID                         | Pointer to 64-bit staging ID                                                           |

## huk

Pointer to 256-bit Hardware Unique Key

```
rkek Pointer to 128-bit Root key Encryption Key dek Pointer to 128-bit Local Encryption Key oem_public_key Pointer to 512-bit OEM Public Key pvt_128key0 Pointer to 128-bit AES Key pvt_128key1 Pointer to 128-bit AES Key pvt_128key2 Pointer to 128-bit AES Key pvt_128key3 Pointer to 128-bit AES Key ek Pointer to 256-bit endorsement key secure_emu_key0 Pointer to 128-bit Secure Debug Key secure_emu_key1 Pointer to 128-bit Secure Debug Key emu_key0_disable Secure debug key disable emu_key1_disable Secure debug key disable public_key0 Pointer to 512-bit public key used for boot stream authentication public_key1 Pointer to 512-bit public key used for boot stream authentication boot_info
```

## gp1

Pointer to 608-bit boot customization structure, see also struct ADI\_ROM\_OTP\_BOOT\_INFO

## antiroll\_nv\_cntr

Anti-rollback counter to prevent the loading of older firmware during a secure boot.

The counter supports values of 0 through 31. The counter feature is disabled when the counter is initially set to zero.

Pointer to 512-bit General purpose user space

## bootModeDisable

Boot mode disable for permanently disabling specific boot modes

## preboot\_ddr\_cfg

Pointer to 384-bit DMC Configuration. See also struct ADI\_ROM\_OTP\_DMC\_CONFIG

## stageID

Pointer to 64-bit staging ID

## Enumerations

The programming model for booting the processor uses the enumerations defined in this section.

## enum ADI\_ROM\_BOOT\_KEY\_TYPE

Enumeration Type Declaration: ADI\_ROM\_BOOT\_KEY\_TYPE

Indicates when custom security keys are used for evaluation of secure boot.

By default the boot process fetches all security keys from OTP for use during secure boot. Custom security lets the program set their custom keys in the struct ADI\_ROM\_BOOT\_CONFIG item (not from OTP). Secure boot is evaluated using the adi\_rom\_Boot() function without provisioning keys in OTP .

Table 46-73: ADI\_ROM\_BOOT\_KEY\_TYPE Members

| Enumerator              | Description                                                          |
|-------------------------|----------------------------------------------------------------------|
| ADI_ROM_CUSTOM_SECURITY | Enable use of custom security keys for authentication and decryption |

## ADI\_ROM\_CUSTOM\_SECURITY

Enable use of custom security keys for authentication and decryption

## enum ADI\_ROM\_BOOT\_TYPE

Enumeration Type Declaration: ADI\_ROM\_BOOT\_TYPE

Indicate to the boot kernel (in an open processor) when a secure or non-secure boot is required.

The boot kernel defaults to a secure boot unless the boot structure is configured as a Non-Secure Boot.

Table 46-74: ADI\_ROM\_BOOT\_TYPE Members

| Enumerator                     | Description                                                                                                            |
|--------------------------------|------------------------------------------------------------------------------------------------------------------------|
| ADI_ROM_SECURE_BOOT_DIS        | Non-Secure Boot                                                                                                        |
| ADI_ROM_SECURE_BOOT            | Secure Boot                                                                                                            |
| ADI_ROM_SECURE_BOOT_HASH_ ONLY | Boot kernel supports for hash compare only if this boot type selected using the ROMAPI. ECDSA verification is skipped. |

## ADI\_ROM\_SECURE\_BOOT\_DIS

Non-Secure Boot

## ADI\_ROM\_SECURE\_BOOT

Secure Boot

## ADI\_ROM\_SECURE\_BOOT\_HASH\_ONLY

Boot kernel supports for hash compare only if this boot type selected using ROM API. ECDSA verification is skipped.

## enum OTPCMD

Enumeration Type Declaration: OTPCMD

Commands required by the adi\_rom\_otp\_get() routine to retrieve specific fields from the OTP memory.

Table 46-75: OTPCMD Members

| Enumerator             | Description               |
|------------------------|---------------------------|
| otpcmd_huk             | Hardware Unique Key       |
| otpcmd_rkek            | Root key Encryption Key   |
| otpcmd_dek             | Local Encryption Key      |
| otpcmd_oem_public_key  | OEMPublic Key             |
| otpcmd_pvt_128key0     | Customer Private AES Key0 |
| otpcmd_pvt_128key1     | Customer Private AES Key1 |
| otpcmd_pvt_128key2     | Customer Private AES Key2 |
| otpcmd_pvt_128key3     | Customer Private AES Key3 |
| otpcmd_ek              | Endorsement Key           |
| otpcmd_secure_emu_key0 | Secure Emulation Key 0    |

Table 46-75: OTPCMD Members (Continued)

| Enumerator              | Description                            |
|-------------------------|----------------------------------------|
| otpcmd_secure_emu_key1  | Secure Emulation Key 1                 |
| otpcmd_emu_key0_disable | Secure emulation key 0 disable         |
| otpcmd_emu_key1_disable | Secure emulation key 1 disable         |
| otpcmd_public_key0      | Customer Public Key0                   |
| otpcmd_public_key1      | Customer Public Key1                   |
| otpcmd_boot_info        | Customer Programmable Boot Information |
| otpcmd_otpTiming        | OTP Read timing override               |
| otpcmd_antiroll_nv_cntr | AntiRollback NV Counter                |
| otpcmd_gp1              | General Purpose 1                      |
| otpcmd_bootModeDisable  | Boot Mode Disable Bits                 |
| otpcmd_preboot_ddr_cfg  | User DMCconfiguration                  |
| otpcmd_stageID          | StageID                                |

## otpcmd\_huk

Hardware Unique Key

## otpcmd\_rkek

Root Encryption Key

otpcmd\_dek

Local Encryption Key

otpcmd\_oem\_public\_key

OEM Public Key

otpcmd\_pvt\_128key0

Customer Private AES Key0

otpcmd\_pvt\_128key1

Customer Private AES Key1

otpcmd\_pvt\_128key2

Customer Private AES Key2

otpcmd\_pvt\_128key3

Customer Private AES Key3

```
otpcmd_ek Endorsement Key otpcmd_secure_emu_key0 Secure Emulation Key 0 otpcmd_secure_emu_key1 Secure Emulation Key 1 otpcmd_emu_key0_disable Secure emulation key 0 disable otpcmd_emu_key2_disable Secure emulation key 2 disable otpcmd_public_key0 Customer Public Key0 otpcmd_public_key1 Customer Public Key1 otpcmd_boot_info Customer Programmable Boot Information otpcmd_otpTiming OTP Read timing override otpcmd_antiroll_nv_cntr AntiRollback NV Counter otpcmd_gp1 General Purpose 1 otpcmd_bootModeDisable Boot Mode Disable Bits otpcmd_preboot_ddr_cfg User DMC configuration otpcmd_stageID StageID
```

## enum ROM\_BOOT\_MDMA\_CRC\_SUPPORT

Enumeration Type Declaration:

ROM\_BOOT\_MDMA\_CRC\_SUPPORT

MDMA Channel CRC support.

Specifies whether the MDMA channel supports CRC operations or not

Table 46-76: ROM\_BOOT\_MDMA\_CRC\_SUPPORT Members

| Enumerator                      | Description                      |
|---------------------------------|----------------------------------|
| ROM_BOOT_DMA_CRC_SUPPORTED      | MDMAChannel supports CRC         |
| ROM_BOOT_DMA_CRC_NOT_ SUPPORTED | MDMAChannel does not support CRC |

## ROM\_BOOT\_DMA\_CRC\_SUPPORTED

MDMA Channel supports CRC

ROM\_BOOT\_DMA\_CRC\_NOT\_SUPPORTED

MDMA Channel does not support CRC

## enum ROM\_BOOT\_RESULT

Enumeration Type Declaration: ROM\_BOOT\_RESULT

General Boot ROM Return Types.

Used throughout the boot software to indicate various success and failure events that can occur during boot.

Table 46-77: ROM\_BOOT\_RESULT Members

| Enumerator              | Description                                 |
|-------------------------|---------------------------------------------|
| ROM_BOOT_FAILURE        | Failure                                     |
| ROM_BOOT_SUCCESS        | Success                                     |
| ROM_BOOT_HDR_CHKSUM_ERR | Boot Stream Block Header Checksum Error     |
| ROM_BOOT_HDR_SIGN_ERR   | Boot Stream Block Header Sign Failure       |
| ROM_BOOT_HDR_DEST_ERR   | Boot Stream Block Payload Destination Error |
| RESERVED0               | Reserved                                    |
| RESERVED1               | Reserved                                    |
| RESERVED2               | Reserved                                    |
| RESERVED3               | Reserved                                    |
| ROM_BOOT_CGU_WRITE_ERR  | CGU Write Error                             |
| ROM_BOOT_DMA_SUCCESS    | DMAoperation was successful                 |

Table 46-77: ROM\_BOOT\_RESULT Members (Continued)

| Enumerator                     | Description                                 |
|--------------------------------|---------------------------------------------|
| ROM_BOOT_DMA_FAILURE           | DMAFailure.                                 |
| ROM_BOOT_DMA_ACTIVE            | DMAChannel is Active                        |
| ROM_BOOT_DMA_CONFIG_ERR        | DMAconfiguration error                      |
| ROM_BOOT_MDMA_ID_ERR           | Illegal MDMAChannel ID                      |
| ROM_BOOT_MDMA_OPERATION_ERR    | Illegal MDMAoperation Specified             |
| ROM_BOOT_MDMA_CONFIG_ERR       | Memory DMAconfiguration error               |
| ROM_BOOT_MDMA_SRC_ERR          | MDMASource Channel Configuration Error      |
| ROM_BOOT_MDMA_DST_ERR          | MDMADestination Channel Configuration Error |
| ROM_BOOT_MDMA_DONE_DETECT_ ERR | Memory DMACompleted with errors             |
| ROM_BOOT_MDMA_SUCCESS          | Memory DMACompleted successfully            |
| ROM_BOOT_PDMA_CONFIG_ERR       | Peripheral DMAconfiguration invalid         |
| ROM_BOOT_PDMA_SUCCESS          | Peripheral DMAcompleted successfully        |
| ROM_BOOT_CRC_FAILURE           | MDMACRC32 Failure                           |
| ROM_BOOT_CRC_CONFIG_ERR        | MDMACRC32 CONFIG error                      |
| ROM_BOOT_CRC_COUNT_ERR         | CRC Byte Count was not a multiple 4         |
| ROM_BOOT_CRC_SUPPORTED_ERR     | CRC Not Supported Error.                    |
| ROM_BOOT_CRC_INITCODE_ERR      | CRC32 Enable Failure During Boot            |
| ROM_BOOT_CRC_CALLBACK_ERR      | Error in Execution of the CRC Callback      |
| ROM_BOOT_CRC_SUCCESS           | MDMACRCSuccess                              |

## ROM\_BOOT\_FAILURE

Failure.

General failure can be used to indicate any general failure throughout the boot process

## ROM\_BOOT\_SUCCESS

Success.

General success can be used to indicate any general functional success for an operation during the boot process.

NOTE: This must be the return result for a boot mode drivers initialization, configuration, load and cleanup routines when overriding their functionality in second stage boot loaders to use custom functions.

## ROM\_BOOT\_HDR\_CHKSUM\_ERR

Boot Stream Block Header Checksum Error.

Indicates that the 8-bit XOR checksum of the 16-byte block header failed to generate the expected result.

## ROM\_BOOT\_HDR\_SIGN\_ERR

Boot Stream Block Header Sign Failure.

The 0xAD block required as byte 4 of the boot block header was not found.

## ROM\_BOOT\_HDR\_DEST\_ERR

Boot Stream Block Payload Destination Error.

The target address field of the block header indicates that the payload for the block is destined towards an address that is not supported. Indicates an attempt to load data to the reserved 8 KB non-bootable region of memory reserved by the boot process.

## ROM\_BOOT\_CGU\_WRITE\_ERR

CGU Write Error

Returned by the CGU configuration routine if a CGU error is set during its initialization from settings provisioned in the OTP .

## ROM\_BOOT\_DMA\_SUCCESS

DMA operation was successful

## ROM\_BOOT\_DMA\_FAILURE

DMA Failure.

Returned by the DMA routines if an error was detected in the DMA\_STAT.IRQERR prior to setting up a new DMA operation with the newly supplied configuration.

## ROM\_BOOT\_DMA\_ACTIVE

DMA Channel is Active

Returned only by the peripheral DMA routine when an attempt to run another peripheral DMA operation is attempted and the DMA channel is already running.

NOTE: This is not currently implemented for MDMA operations.

## ROM\_BOOT\_DMA\_CONFIG\_ERR

DMA configuration error

## ROM\_BOOT\_MDMA\_ID\_ERR

Illegal MDMA Channel ID.

Returned by adi\_rom\_MemDma() if the MDMA channel ID is not supported. For supported channel IDs, see enum ROM\_DMA\_MDMA\_ID

## ROM\_BOOT\_MDMA\_OPERATION\_ERR

Illegal MDMA operation Specified.

Returned by adi\_rom\_MemDma() if the MDMA operation is not supported. For supported operations, see enum ROM\_DMA\_MDMA\_OPERATION

## ROM\_BOOT\_MDMA\_CONFIG\_ERR

Memory DMA configuration error

## ROM\_BOOT\_MDMA\_SRC\_ERR

MDMA Source Channel Configuration Error.

Set by the MDMA routines if after configuring the MDMA source channel to start a DMA operation, an error is generated in the source channels DMA\_STAT.IRQERR

## ROM\_BOOT\_MDMA\_DST\_ERR

MDMA Destination Channel Configuration Error.

Set by the MDMA routines when an error is generated in the destination channels DMA\_STAT.IRQERR bit (after configuring the MDMA source channel to start a DMA operation)

## ROM\_BOOT\_MDMA\_DONE\_DETECT\_ERR

Memory DMA Completed with errors

## ROM\_BOOT\_MDMA\_SUCCESS

Memory DMA Completed successfully

## ROM\_BOOT\_PDMA\_CONFIG\_ERR

Peripheral DMA configuration invalid

## ROM\_BOOT\_PDMA\_SUCCESS

Peripheral DMA completed successfully

## ROM\_BOOT\_CRC\_FAILURE

MDMA CRC32 Failure.

Returned by the higher level adi\_rom\_MemDma() routine and the underlying adi\_rom\_MemCompare() routine if the CRC32 result of the block of data did not match the expected result.

## ROM\_BOOT\_CRC\_CONFIG\_ERR

MDMA CRC32 CONFIG error

## ROM\_BOOT\_CRC\_COUNT\_ERR

CRC Byte Count was not a multiple 4.

The CRC peripheral operates on 32-bit data only and as such all CRC operations must have a byte count that is a multiple of 4. This result is returned by the higher level adi\_rom\_MemDma() routine and the underlying adi\_rom\_MemCompare() and adi\_rom\_MemFill() routines if the byte count is not a multiple of 4 bytes.

## ROM\_BOOT\_CRC\_SUPPORTED\_ERR

CRC Not Supported Error.

Returned by adi\_rom\_MemDma() , adi\_rom\_MemFill() , adi\_rom\_MemCompare() and adi\_rom\_Crc32Poly() if the supplied DMA configuration specified a MDMA channel that does not support CRC operations.

## ROM\_BOOT\_CRC\_INITCODE\_ERR

CRC32 Enable Failure During Boot.

Returned by adi\_rom\_Crc32Init() if the boot process cannot enable the CRC32 functionality due to a NULL struct ADI\_ROM\_BOOT\_CONFIG pointer or NULL struct ADI\_ROM\_BOOT\_HEADER pointer located in pHeader Pointer to the boot header storage location where all boot stream block headers eventually reside for processing by the kernel

## ROM\_BOOT\_CRC\_CALLBACK\_ERR

Error in Execution of the CRC Callback.

Returned by the default CRC callback function located in the boot ROM if any of the following conditions are met:

- The struct ADI\_ROM\_BOOT\_CONFIG pointer passed to the callback is a NULL pointer
- The struct ADI\_ROM\_BOOT\_BUFFER pointer to the buffer to run CRC validation is a NULL pointer
- The ROM\_CBFLAG\_DIRECT flag is not set in the supplied flags parameter indicating it was a direct callback
- The struct ADI\_ROM\_BOOT\_HEADER pointer located in pHeader Pointer to the boot header storage location where all boot stream block headers eventually reside for processing by the kernel is a NULL pointer

## ROM\_BOOT\_CRC\_SUCCESS

MDMA CRC Success

## enum ROM\_CORE\_ID

Enumeration Type Declaration: ROM\_CORE\_ID

Core ID.

An enumeration for referencing a particular core

Table 46-78: ROM\_CORE\_ID Members

| Enumerator         | Description     |
|--------------------|-----------------|
| ROM_CORE_ID0       | Core 0          |
| ROM_CORE_ID1       | Core 1          |
| ROM_CORE_ID2       | Core 2          |
| ROM_CORE_NUM_CORES | Number of Cores |

## ROM\_CORE\_ID0

Core 0

## ROM\_CORE\_NUM\_CORES

Number of Cores

## enum ROM\_DMA\_DONE\_DETECT\_METHOD

Enumeration Type Declaration: ROM\_DMA\_DONE\_DETECT\_METHOD

DMA Done Detection Method.

Specifies the method used to detect the completion of the requested DMA operation.

When a program requests a non-blocking DMA operation, separate software is required to check the status of the DMA channel. The boot ROM does not provide an API for this operation.

NOTE: Trigger mode is not supported on this product

Table 46-79: ROM\_DMA\_DONE\_DETECT\_METHOD Members

| Enumerator                  | Description                                           |
|-----------------------------|-------------------------------------------------------|
| ROM_DMA_DONE_NON_BLOCKING   | Return without waiting for the DMAto complete         |
| ROM_DMA_DONE_POLL_IRQDONE   | Poll on the IRQDONE bit in the DMAStatus register     |
| ROM_DMA_DONE_WAKEUP_TRIGGER | Configure a trigger to wake up the core when complete |

## ROM\_DMA\_DONE\_NON\_BLOCKING

Return without waiting for the DMA to complete

## ROM\_DMA\_DONE\_POLL\_IRQDONE

Poll on the IRQDONE bit in the DMA Status register

ROM\_DMA\_DONE\_WAKEUP\_TRIGGER

Configure a trigger to wake up the core on when done

## enum ROM\_DMA\_MDMA\_ID

Enumeration Type Declaration: ROM\_DMA\_MDMA\_ID

MDMA Channel ID.

The ID of the Memory DMA channel. Used in the struct ROM\_DMA\_MDMA\_CONFIG configuration to specify the Memory DMA channel to use for operations accessible using the adi\_rom\_MemDma() routine

Table 46-80: ROM\_DMA\_MDMA\_ID Members

| Enumerator               | Description                 |
|--------------------------|-----------------------------|
| ROM_DMA_MDMA0            | Memory DMAStream 0          |
| ROM_DMA_MDMA1            | Memory DMAStream 1          |
| ROM_DMA_MDMA2            | Memory DMAStream 2          |
| ROM_DMA_MDMA3            | Memory DMAStream 3          |
| ROM_DMA_MEMDMA_END_COUNT | Number of Memory DMAStreams |

## ROM\_DMA\_MDMA0

Memory DMA Stream 0

## ROM\_DMA\_MDMA1

Memory DMA Stream 1

## ROM\_DMA\_MDMA2

Memory DMA Stream 2

## ROM\_DMA\_MDMA3

Memory DMA Stream 3

## ROM\_DMA\_MEMDMA\_END\_COUNT

Number of Memory DMA Streams

## enum ROM\_DMA\_MDMA\_OPERATION

Enumeration Type Declaration: ROM\_DMA\_MDMA\_OPERATION

MDMA Operation.

The operation determines if only an MDMA is required, or whether a CRC operation must be used in conjunction with the MDMA.

Table 46-81: ROM\_DMA\_MDMA\_OPERATION Members

| Enumerator           | Description                                                                         |
|----------------------|-------------------------------------------------------------------------------------|
| ROM_DMA_MEM_COPY     | Standard MDMAtransfer from a source to a destination                                |
| ROM_DMA_MEM_CRC      | Performs a CRC32 MDMAread operation and compares the result with an expected result |
| ROM_DMA_MEM_FILL     | Uses the CRC peripheral to perform a fill operation with a 32-bit value             |
| ROM_DMA_MEM_COMPARE  | Uses the CRC peripheral to compare data with a constant 32-bit value                |
| ROM_DMA_CRC_LUT_INIT | Initializes the CRC LUT from the supplied CRC Polynomial                            |

## ROM\_DMA\_MEM\_COPY

Standard MDMA transfer from a source to a destination

## ROM\_DMA\_MEM\_CRC

Performs a CRC32 MDMA read operation and compares the result with an expected result

## ROM\_DMA\_MEM\_FILL

Uses the CRC peripheral to perform a fill operation with a 32-bit value

## ROM\_DMA\_MEM\_COMPARE

Uses the CRC peripheral to compare data with a constant 32-bit value

## ROM\_DMA\_CRC\_LUT\_INIT

Initializes the CRC LUT from the supplied CRC Polynomial

## enum ROM\_DMA\_PDMA\_OPERATION

Enumeration Type Declaration:

ROM\_DMA\_PDMA\_OPERATION

Table 46-82: ROM\_DMA\_PDMA\_OPERATION Members

| Enumerator      | Description                   |
|-----------------|-------------------------------|
| ROM_DMA_PERI_TX | Peripheral Transmit Operation |
| ROM_DMA_PERI_RX | Peripheral Receive Operation  |

## ROM\_DMA\_PERI\_TX

Peripheral Transmit Operation

## ROM\_DMA\_PERI\_RX

Peripheral Receive Operation

## enum ROM\_GETADDR\_VALUE

Enumeration Type Declaration: ROM\_GETADDR\_VALUE

Parameter for adi\_rom\_GetAddress() function to retrieve the address of a data object stored in the boot ROM.

Table 46-83: ROM\_GETADDR\_VALUE Members

| Enumerator               | Description                                                                                                    |
|--------------------------|----------------------------------------------------------------------------------------------------------------|
| ROM_GETADDR_CONSTANTS    | Retrieve the address of the ROM_CONSTANTS_TYPE object                                                          |
| ROM_GETADDR_BMODE        | Retrieve the address of the lookup table sorting the default adi_rom_boot() parame- ters for each boot mode    |
| ROM_GETADDR_MDMAREGS     | Retrieve the address of the struct ROM_BOOT_MDMA_REGS object                                                   |
| ROM_GETADDR_SPILUT       | Retrieve the address of the lookup table in the ROMdescribing the various SPI master boot BCODE configurations |
| ROM_GETADDR_ECDSA_DOMAIN | Retrieve the address of the domain parameters used for ECDSA                                                   |

## ROM\_GETADDR\_CONSTANTS

Retrieve the address of the ROM\_CONSTANTS\_TYPE object

## ROM\_GETADDR\_BMODE

Retrieve the address of the lookup table storing the default adi\_rom\_boot() parameters for each boot mode

## ROM\_GETADDR\_MDMAREGS

Retrieve the address of the struct ROM\_BOOT\_MDMA\_REGS object

## ROM\_GETADDR\_SPILUT

Retrieve the address of the lookup table in the rom describing the various SPI master boot BCODE configurations

## ROM\_GETADDR\_ECDSA\_DOMAIN

Retrieve the address of the domain parameters used for ECDSA

## enum ROM\_HOOK\_CALL\_CAUSE

Enumeration Type Declaration: ROM\_HOOK\_CALL\_CAUSE

Passed to a user hook routine to indicate the reason of the call.

An optional hook routine is provided as a callback when calling a boot mode via adi\_rom\_Boot. This hook routine is called by the boot software first after the execution of the boot modes initialization routine then again after execution of the boot modes configuration routine. This parameter allows the users routine to identify at which point the call was made allowing the user to perform different actions for each call.

Table 46-84: ROM\_HOOK\_CALL\_CAUSE Members

| Enumerator                    | Description                                                                        |
|-------------------------------|------------------------------------------------------------------------------------|
| ROM_HOOK_CALL_INIT_COMPLETE   | Call was because of the completion of the boot modes initialization function       |
| ROM_HOOK_CALL_CONFIG_COMPLETE | Call was because of the completion of the boot modes configuration function        |
| ROM_HOOK_REG_COMPLETE         | Call was because of the completion of the boot modes pre-register initi- alization |

## enum ROM\_SB\_IMAGE\_TYPE

Enumeration Type Declaration: ROM\_SB\_IMAGE\_TYPE

Secure Boot Image Types.

The secure boot header contains a type field for the secure boot image type, this enumeration provides a complete list of all image types.

NOTE: The secure boot process does not necessarily support all image types defined.

Table 46-85: ROM\_SB\_IMAGE\_TYPE Members

| Enumerator               | Description                                                                                                                      |
|--------------------------|----------------------------------------------------------------------------------------------------------------------------------|
| ROM_SB_IMAGE_UNKNOWN     | Unknown Secure Boot image type, used by software to initialize the type before detection of boot image type takes place          |
| ROM_SB_IMAGE_BLP         | Plain text BLp secure boot image supports authentication only with no decryption                                                 |
| ROM_SB_IMAGE_BLW         | Keywrapped BLw secure boot image supports authentication and decryption, boot stream decryption key wrapped in the secure header |
| ROM_SB_IMAGE_BLE         | Not supported by any Secure boot products. Secure boot image with key stored in plain text form in the secure header             |
| ROM_SB_IMAGE_BLX         | BLx Secure boot image supports authentication and decryption, boot stream decryption key wrapped located in OTP                  |
| ROM_SB_IMAGE_UNSUPPORTED | Used by software to indicate any other unsupported image type                                                                    |

## ROM\_SB\_IMAGE\_UNKNOWN

Unknown Secure Boot image type, used by software to initialize the type before detection of boot image type takes place

## ROM\_SB\_IMAGE\_BLP

Plain text BLp secure boot image supports authentication only with no decryption

## ROM\_SB\_IMAGE\_BLW

Keywrapped BLw secure boot image supports authentication and decryption, boot stream decryption key wrapped in the secure header

## ROM\_SB\_IMAGE\_BLE

Not supported by any Secure boot products. Secure boot image with key stored in plain text form in the secure header

## ROM\_SB\_IMAGE\_BLX

BLx Secure boot image supports authentication and decryption, boot stream decryption key wrapped located in OTP

## ROM\_SB\_IMAGE\_UNSUPPORTED

Used by software to indicate any other unsupported image type

## enum ROM\_SPI\_PROTOCOL

Enumeration Type Declaration: ROM\_SPI\_PROTOCOL

The SPI Protocol to use.

SPI Flash devices can support multiple protocols to send commands to the SPI flash. Commands are usually sent over the single-bit bus; however a number of newer devices also support sending commands over the dual- or quadbit bus.

WARNING: In system reset type events where the processor attempts to reboot and the flash may not have been reset, using the DUALIO or QUADIO protocols for the command cycles risks the boot process being unable to communicate with the SPI flash. Do not enable these features on SPI Flash devices if they are also the primary boot source used for booting from hardware reset and system reset events.

Table 46-86: ROM\_SPI\_PROTOCOL Members

| Enumerator              | Description                                                             |
|-------------------------|-------------------------------------------------------------------------|
| ROM_SPI_EXT_PROTOCOL    | Extended protocol where the command cycle is sent on the single bit bus |
| ROM_SPI_DUALIO_PROTOCOL | DualIO protocol where the command cycle is sent on the dual bit bus     |
| ROM_SPI_QUADIO_PROTOCOL | QuadIO protocol where the command cycle is sent on the quad bit bus     |

## ROM\_SPI\_EXT\_PROTOCOL

Extended protocol where the command cycle is sent on the single bit bus

## ROM\_SPI\_DUALIO\_PROTOCOL

DualIO protocol where the command cycle is sent on the dual bit bus

## ROM\_SPI\_QUADIO\_PROTOCOL

QuadIO protocol where the command cycle is sent on the quad bit bus