## Preface

Thank you for purchasing and developing systems using an ADSP-SC58x SHARC+ processor from Analog Devices, Inc.

The ADSP-SC58x processor features two high-performance, floating-point SHARC+ ®  cores (ADSP-2158x) and one ARM ®  Cortex-A5 core. The cores deliver 24 GFLOPS performance at under 2W for advanced, real-rime audio and industrial applications. The processor is designed for a wide variety of automotive, consumer, professional audio energy distribution systems, and other computationally intensive real-time applications.

## Purpose of This Manual

The ADSP-SC58x SHARC Processor Hardware Reference provides architectural information about the ADSP-SC58x processors. This hardware reference provides the main architectural information about these processors. The architectural descriptions cover functional blocks, buses, and ports, including all features and processes that they support. For information about programming the ARM core in the ADSP-SC58x processor, visit the ARM Information Center at:

http://infocenter.arm.com.

For timing, electrical, and package specifications, see the processor data sheet.

## Intended Audience

The primary audience for this manual is a programmer who is familiar with Analog Devices processors. The manual assumes the audience has a working knowledge of the appropriate processor architecture and instruction set. Programmers who are unfamiliar with Analog Devices processors can use this manual, but should supplement it with other texts, such as programming reference books and data sheets, that describe their target architecture.

## What's New in This Manual

This is the first released to production revision (1.0) of the ADSP-SC58x SHARC+ Processor Hardware Reference .

## Technical or Customer Support

You can reach customer and technical support for processors from Analog Devices in the following ways:

- Post your questions in the processors and DSP support community at EngineerZone : http://ez.analog.com/community/dsp
- Submit your questions to technical support at Connect with ADI Specialists :

http://www.analog.com/support

- E-mail your questions about software/hardware development tools to: processor.tools.support@analog.com
- E-mail your questions about processors and DSPs to: processor.support@analog.com (world wide support) processor.china@analog.com (China support)
- Contact your Analog Devices sales office or authorized distributor. Locate one at: http://www.analog.com/adi-sales
- Send questions by mail to:

Analog Devices, Inc.

Three Technology Way P.O. Box 9106 Norwood, MA 02062-9106 USA

## Product Information

Product information can be obtained from the Analog Devices Web site and CrossCore Embedded Studio online Help system.

## Analog Devices Web Site

The Analog Devices Web site, http://www.analog.com, provides information about a broad range of products-analog integrated circuits, amplifiers, converters, and digital signal processors.

To access a complete technical library for each processor family, go to: http://www.analog.com/processors/techni-cal\_library The manuals selection opens a list of current manuals related to the product as well as a link to the previous revisions of the manuals. When locating your manual title, note a possible errata check mark next to the title that leads to the current correction report against the manual.

Also note, MyAnalog.com is a free feature of the Analog Devices Web site that allows customization of a Web page to display only the latest information about products you are interested in. You can choose to receive weekly e-mail notifications containing updates to the Web pages that meet your interests, including documentation errata against all manuals. MyAnalog.com provides access to books, application notes, data sheets, code examples, and more.

Visit MyAnalog.com to sign up. If you are a registered user, just log on. Your user name is your e-mail address.

## EngineerZone

EngineerZone is a technical support forum from Analog Devices. It allows you direct access to ADI technical support engineers. You can search FAQs and technical information to get quick answers to your embedded processing and DSP design questions.

Use EngineerZone to connect with other DSP developers who face similar design challenges. You can also use this open forum to share knowledge and collaborate with the ADI support team and your peers. Visit http:// ez.analog.com to sign up.

## Supported Processors

The following is the list of Analog Devices, Inc. processors supported by the CrossCore Embedded Studio ®  development tools suite.

## Blackfin+ ®  (ADSP-BF7xx) Processors

The name Blackfin+ refers to the enhanced fixed-point Blackfin core architecture featured by the ADSPBF70x processor product line, which is a family of 16-bit embedded processors.

## Blackfin ®  (ADSP-BF6xx/BF5xx) Processors

The name Blackfin refers to the fixed-point core architecture featured on the following processors: ADSPBF5xx and ADSP-BF6xx.

## SHARC ®  (ADSP-21xxx) Processors

The name SHARC refers to the high-performance, 32-bit, floating-point core architecture featured on the following processors: ADSP-2116x, ADSP-2126x, ADSP-213xx, and ADSP-214xx. These processors can be used in speech, sound, graphics, and imaging applications.

## SHARC+ ®  (ADSP-SC5xx, ADSP-215xx) Processors

The name SHARC+ refers to the enhanced high-performance, 32-bit, floating-point core architecture featured on the following processors: ADSP-215xx/ADSP-SC5xx. The connected SHARC+ ADSP-SC5xx processors also contain an ARM® Cortex-A5® core. These products can be used in speech, sound, graphics, and imaging applications.

The following is the list of Analog Devices, Inc. processors supported by the IAR Embedded WorkBench ®  development tools. For information about the IAR Embedded WorkBench product and software download, go to http:// www.iar.com/en/Products/IAR-Embedded-Workbench .

## Mixed-Signal Control Processors

The ADSP-CM40x processors are based on the ARM Cortex ® -M4 core and are designed for motor control and industrial applications.

The ADSP-CM41x processors are based on the ARM Cortex-M4 and ARM Cortex-M0 cores and are designed for motor control and industrial applications.

## How to Use this Manual

This section describes how this manual is organized and where you can find specific types of information.

This manual is organized such that it aligns with the Figure 1-1 ADSP-SC58x Functional Block Diagram. This structure is:

ARM Cortex Overview

Power and Clock Management

- Clock Generation Unit (CGU)
- Dynmic Power Management (DPM)
- Reset Control Unit (RCU)

System Interrupts and Triggers

- System Event Control (SEC)
- Trigger Routing Unit (TRU)

System Memory (L2CTL/DMC/SMC/OTPC/SMPU)

- L2 Controller (L2CTL)
- Dynamic Memory Congtroller (DMC)
- Static Memory Controller (SMC)
- One-Time Programmable Memory Controller (OTPC)
- System Memory Protection Unit (SMPU)

Peripherals:

- GPIO
- Timers and Counters
- General-Purpose Counter (CNT)
- General-Purpose Timer (TMR)
- Watchdog Timer (WDOG)

- Real-Time Counter (RTC)
- Serial Interfaces
- Serial Ports (SPORTs)
- Serial Peripheral Interface (SPI, SPIHP)
- Universal Asynchronous Receiver/Transmitter (UART)
- Two-Wire Interface (TWI)
- Control Area Network (CAN)
- Universal Serial Bus (USB)
- Parallel Interfaces
- Enhanced Parallel Peripheral Interface (EPPI)
- Mobile Storage Interface (MSI)

## Direct Memory Access

- Dynamic Memory Controller (DMC)
- Memory DMA Controller (MDMA)
- Cyclic Redundancy Check (CRC)

## Analog Subsystem (Housekeeping ADC)

## Security and Protection (SPU/PKTE/PKIC/PKA/TRNG)

- System Protection Unit (SPU)
- Security Packet Engine (PKTE)
- Public Key Interrupt Controller (PKIC)
- Public Key Accelerator (PKA)
- True Random Number Generator (TRNG)

## Booting

## System Debug

- System Crossbars (SCB)
- System Watchpoint Unit (SWU)
- System Debug and Trace Unit (DBG)
- Test Access Port Controller (TAPC)
- Trace Unit (CSPFT)

- System Trace Module (STM)

The following list provides information on the organization and contents of each chapter of this manual. Note that not every chapter follows this organization exactly nor does every chapter contain all of the topics listed.

- Overview. Provides very general information on what the module does. Also may list relevent industry standards that the modules support or comply with.
- Features. Provides a list of module features such as modes of operation, clocking options, and throughput.
- Functional Description. Provides details on how the module functions including descriptions of module blocks, connectivity and a glossary. This section also contains a register list and lists of interrupts, triggers and DMA channels (if applicable).
- Operating Modes. Describes the available modes of operation, how to configure them and how they work.
- Event Control. Provides information on interrupt and fault generation and how to manage them.
- Programming Model. Provides general guidelines on how to configure and enable the module. May also provide specific examples for configuring a mode of operation.
- Debug Features. Describes debugging operations.
- Register Descriptions. Provides detailed descriptions of all of the registers and bits used by the module.

This manual also contains a comprehensive appendix of memory-mapped register addresses and names. The modules in this appendix are presented in alphabetical order.

## Notation Conventions

Text conventions used in this manual are identified and described as follows. Additional conventions, which apply only to specific chapters, may appear throughout this document.

| Example            | Description                                                                                                                                                                           |
|--------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| File > Close       | Titles in reference sections indicate the location of an item within the CrossCore Embedded Studio IDE's menu system (for example, the Close command appears on the File menu).       |
| {this &#124; that} | Alternative required items in syntax descriptions appear within curly brackets and separated by vertical bars; read the example as this or that . One or the other is required.       |
| [this &#124; that] | Optional items in syntax descriptions appear within brackets and separated by vertical bars; read the example as an optional this or that .                                           |
| [this, …]          | Optional item lists in syntax descriptions appear within brackets delimited by commas and terminated with an ellipse; read the example as an optional comma- separated list of this . |
| .SECTION           | Commands, directives, keywords, and feature names are in text with Letter Gothic font.                                                                                                |

| Example                   | Description                                                                                                                                                                                                                                                                                                          |
|---------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| filename                  | Non-keyword placeholders appear in text with italic style format.                                                                                                                                                                                                                                                    |
| NOTE:                     | NOTE: For correct operation, ... A note provides supplementary information on a related topic. In the online ver- sion of this book, the word NOTE: appears instead of this symbol.                                                                                                                                  |
| CAUTION:                  | CAUTION: Incorrect device operation may result if ... CAUTION: Device damage may result if ... A caution identifies conditions or inappropriate usage of the product that could lead to undesirable results or product damage. In the online version of this book, the word CAUTION: appears instead of this symbol. |
| ATTENTION:                | ATTENTION: Injury to device users may result if ... A warning identifies conditions or inappropriate usage of the product that could lead to conditions that are potentially hazardous for devices users. In the online version of this book, the word ATTENTION: appears instead of this symbol.                    |
| Registers/Bits            | All registers and bits in this manual are linked (clickable) to their respective de- scriptions in the "Register Descriptions" of each chapter.                                                                                                                                                                      |
| Miscellaneous Conventions | Interrupt and internal signals are shown in all caps with no other formatting. For example the SPDIFn_RX or SCLK signal or the PKTE0_IRQ interrupt. An overbar denotes an active-low signal as in SYS_FAULT.                                                                                                         |

## Register Documentation Conventions

Register diagrams use the following conventions:

- The descriptive name of the register appears at the top with the short form of the name.
- If a bit has a short name, the short name appears first in the bit description, followed by the long name.
- The reset value appears in binary in the individual bits and in hexadecimal to the left of the register.
- Bits marked X have an unknown reset value. Consequently, the reset value of registers that contain such bits is undefined or dependent on pin values at reset.
- Shaded bits are reserved

NOTE: To ensure upward compatibility with future implementations, write back the value that is read for reserved bits in a register, unless otherwise specified.

Register description tables use the following conventions:

- Each bit's or bit field's access type appears beneath the bit number in the table in the form (read-access/writeaccess). The access types include:
- R = read, RC = read clear, RS = read set, R0 = read zero, R1 = read one, Rx = read undefined
- W = write, NW = no write, W1C = write one to clear, W1S = write one to set, W0C = write zero to clear, W0S = write zero to set, WS = write to set, WC = write to clear, W1A = write one action

- Many bit and bit field descriptions include enumerations, identifying bit values and related functionality. Un- less otherwise indicated (with a prefix), these enumerations are decimal values.