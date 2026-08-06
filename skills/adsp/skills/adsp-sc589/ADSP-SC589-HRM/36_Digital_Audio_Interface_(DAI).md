## 33   Digital Audio Interface (DAI)

The Digital Audio Interfaces (DAIn) are comprised of groups of identical peripherals and their respective Signal Routing Units (SRUn). The SRU connects inputs and outputs of the DAI peripherals with each other and to the external pins. This configuration allows peripherals to be interconnected to accomodate a wide variety of systems without making external pin connections.

In a typical processor, static (multiplexed) pins are assigned to specific peripherals. When certain peripherals are not required for an application, these pins are unnecessary and expensive. The pins may need to be defined as high or low to prevent any illegal conditions. The signal routing units on the SHARC processors addresses this situation by controlling a number of general-purpose pins which can be assigned flexibly (a virtual connectivity between peripherals) depending on system requirements. This virtual connectivity includes pin buffers and routing logic (multiplexer). It also allows the SHARC processors to include an arbitrary number and variety of peripherals while retaining high levels of compatibility without increasing pin count.

## SRU Features

The SRU has the following features and capabilities.

- Flexible connections that can be made through software and during run time; no hard-wiring is required.
- At reset, a default routing scheme is already programmed.
- Connectivity can be made internally between peripherals, externally between pin buffers, or a mix of both.
- Status of the pin buffers can be programmed for conditional execution or interrupts.
- Some pin buffers allow control of signal polarity changes.
- No fan-out limitation, a peripheral or pin buffer output that can be routed to multiple peripheral or pin buffer inputs.
- Two independent routing systems are availablethe DAI0 and the DAI1. Signals can't be interconnected between both routing units with the exception of the following modules under some specific scenarios.
- The precision clock generator (PCG)
- A pair of DAI pin buffers from each DAI unit
- Asynchronous Sample Rate Converter (ASRC) data for daisy chaining across the DAI units

NOTE: The 349-ball CSP BGA package has 2 x 14 DAI pins bonded off-chip (DAIx\_PIN01 through DAIx\_PIN12, DAIx\_PIN19 and DAIx\_PIN20).

## Functional Description

The fundamental timing clock of the DAI modules is SCLK0\_0.

The DAI Block Diagram shows how the DAI pin buffers are connected through the SRUn. This configuration allows for flexible signal routing.

The DAI units are comprised of four primary blocks:

- Peripherals (A/B/C) associated with DAIn
- Signal Routing Units (SRUn)
- DAIn I/O pin buffers
- Miscellaneous buffers

The peripherals shown in DAI Functional Block Diagram can have up to three connections (if master or slave capable); one acts as a signal input, one as a signal output and the third as an output enable. The SRUs are based on a group of multiplexers which are controlled by registers to establish the desired interconnections. The DAI pin buffers have three signals which are used for input and output to or from off-chip and the third for output enable.

The miscellaneous buffers have an input and an output and are used for group interconnection.

The figures are a simplified representation of a DAI system. In a real representation, the SRU and DAI would show several types of data being routed from several sources including the following:

- Serial ports (SPORT)
- Precision clock generators (PCG)
- Asynchronous sample rate converters (SRC)
- S/PDIF transmitter
- S/PDIF receiver
- DAI interrupts (miscellaneous)

## ADSP-SC58x DAI Register List

The Digital Audio Interfaces (DAIn) contain groups of identical peripherals which can be connected internally between peripherals, externally between pin buffers, or a mix of both. This module contains the following registers.

Table 33-1: ADSP-SC58x DAI Register List

| Name     | Description                      |
|----------|----------------------------------|
| DAI_CLK0 | Clock Routing Control Register 0 |

Table 33-1: ADSP-SC58x DAI Register List (Continued)

| Name         | Description                                   |
|--------------|-----------------------------------------------|
| DAI_CLK1     | Clock Routing Control Register 1              |
| DAI_CLK2     | Clock Routing Control Register 2              |
| DAI_CLK3     | Clock Routing Control Register 3              |
| DAI_CLK4     | Clock Routing Control Register 4              |
| DAI_CLK5     | Clock Routing Control Register 5              |
| DAI_DAT0     | Serial Data Routing Control Register 0        |
| DAI_DAT1     | Serial Data Routing Control Register 1        |
| DAI_DAT2     | Serial Data Routing Control Register 2        |
| DAI_DAT3     | Serial Data Routing Control Register 3        |
| DAI_DAT4     | Serial Data Routing Control Register 4        |
| DAI_DAT5     | Serial Data Routing Control Register 5        |
| DAI_DAT6     | Serial Data Routing Control Register 6        |
| DAI_FS0      | Frame Sync Routing Control Register 0         |
| DAI_FS1      | Frame Sync Routing Control Register 1         |
| DAI_FS2      | Frame Sync Routing Control Register 2         |
| DAI_FS4      | Frame Sync Routing Control Register 4         |
| DAI_IMSK_FE  | Falling-Edge Interrupt Mask Register          |
| DAI_IMSK_PRI | Core Interrupt Priority Assignment Register   |
| DAI_IMSK_RE  | Rising-Edge Interrupt Mask Register           |
| DAI_IRPTL_H  | High Priority Interrupt Latch Register        |
| DAI_IRPTL_HS | Shadow High Priority Interrupt Latch Register |
| DAI_IRPTL_L  | Low Priority Interrupt Latch Register         |
| DAI_IRPTL_LS | Shadow Low Priority Interrupt Latch Register  |
| DAI_MISC0    | Miscellaneous Control Register 0              |
| DAI_MISC1    | Miscellaneous Control Register 1              |
| DAI_PBEN0    | Pin Buffer Enable Register 0                  |
| DAI_PBEN1    | Pin Buffer Enable Register 1                  |
| DAI_PBEN2    | Pin Buffer Enable Register 2                  |
| DAI_PBEN3    | Pin Buffer Enable Register 3                  |
| DAI_PIN0     | Pin Buffer Assignment Register 0              |
| DAI_PIN1     | Pin Buffer Assignment Register 1              |

Table 33-1: ADSP-SC58x DAI Register List (Continued)

| Name         | Description                      |
|--------------|----------------------------------|
| DAI_PIN2     | Pin Buffer Assignment Register 2 |
| DAI_PIN3     | Pin Buffer Assignment Register 3 |
| DAI_PIN4     | Pin Buffer Assignment Register 4 |
| DAI_PIN_STAT | Pin Status Register              |

## ADSP-SC58x DAI Interrupt List

Table 33-2: ADSP-SC58x DAI Interrupt List

|   Interrupt ID | Name      | Description                  | Sensitivity   | DMA Channel   |
|----------------|-----------|------------------------------|---------------|---------------|
|             24 | DAI0_IRQH | DAI0 High Priority Interrupt |               |               |
|             25 | DAI1_IRQH | DAI1 High Priority Interrupt |               |               |
|            145 | DAI0_IRQL | DAI0 Low Priority Interrupt  |               |               |
|            146 | DAI1_IRQL | DAI1 Low Priority Interrupt  |               |               |

## DAI Block Diagram

The DAI Functional Block Diagram and the Digital Audio Interconnect Unit figures show the functional blocks within the DAI and the unit connections to the peripherals.

Figure 33-1: DAI Functional Block Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000000_06310fcccec0189fe920dfebb468edb06b5db56024b32ce4ec55f9518ab69e9d.png)

Figure 33-2: Digital Audio Interconnect Unit

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000001_701c3820d1913a5820d34ca90ec763bf3e4bac48eaca5d83f71a84df3a12f8fb.png)

## DAI Signal Naming Conventions

The peripherals associated with the DAI do not have any dedicated I/O pins for off-chip communication. Instead, the I/O pin is only accessible in the chip internally and is known as an internal node . Every internal node of a DAI peripheral (input or output) is given a unique mnemonic. The convention is to begin the name with an identifier for the peripheral that the signal is coming to or from, followed by the function of the signal.

A number is included if the DAI contains more than one peripheral type (for example, serial ports), or if the peripheral has more than one signal that performs this function (for example, serial ports). The mnemonic always ends with \_I if the signal is an input, or with \_O if the signal is an output. An example is shown in the Example DAI Mnemonics figure.

Figure 33-3: Example DAI Mnemonics

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000002_e2843ff96fd374c67573a29e85f6cb8e0b5808ac00be7006c4632df0b82c2cdf.png)

## I/O Pin Buffers

Within the context of the SRU, physical connections to the DAI pins are replaced by a logical interface known as a pin buffer . This three terminal active device is capable of sourcing or sinking output current when its driver is enabled, and passing external input signals when disabled. Each pin has an input, an output, and an enable as shown in the Pin Buffer Example figure. The inputs and the outputs are defined with respect to the pin, similar to a peripheral device. This naming convention is consistent with the SRU naming convention.

## Pin Buffer Signals

The pin buffer is based on three signals shown in the Pin Buffer Example figure and described in the following sections.

Figure 33-4: Pin Buffer Example

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000003_e27833e45b06a8dda5fe7266ef849e6d1a888b37bd1adfcd9d52af0ea87649ed.png)

## Pin Buffer Input Signal

A pin buffer input ( DAIn\_PBxx\_I ) is driven as an output from the processor when the pin buffer enable is set (=1). Each physical pin (connected to a bonded pad) can be connected through the SRU to any of the outputs of the DAI peripherals, based on the bit field values. The SRU can also be used to route signals that control the pins in other ways. Many signals can be configured for use as control signals.

## Pin Buffer Enable Signal

When a pin buffer enable ( DAIn\_PBENxx\_I ) is set (=1), the signal present at the corresponding pin buffer input ( DAIn\_PBxx\_I ) is driven off-chip as an output. When a pin buffer enable is cleared (=0), the signal present at the corresponding pin buffer input is ignored. The pin enable control registers activate the drive buffer for each of the DAI pins. When the pins are not enabled (driven), they can be used as inputs. There are two options to control the pin buffer enable signal; setting the level high for a static solution, or connecting the dedicated peripheral's pin buffer output enable signal to its pin buffer, which automatically enables the pin buffer.

## Pin Buffer Input Level State

The DAI pads are provided with an input enable signal (IE). The signal is used to ensure that the input path ( DAI\_PBxx\_O ) is pushed to a known state (versus a floating state in case of no external drivers on the DAI pad). This state can be considered as an AND gate of the pad signal with the IE as shown in the pin buffer figure. The IE signal is controlled from the PAD logic ( PADS\_DAIx\_IE register) and is provided at a granularity at each DAI pin. (Refer to the GP port chapter.) Every DAI pin can be individually controlled depending on whether the pin is used in a system or not. The PADS register controlling the IE signal must have a reset value of 0 and hence all the pin buffer inputs are ( DAI\_PBxx\_O ) gated to 0.

NOTE: In previous SHARC products, this default value was 1 due to a weak pull-up.

Once the PADS module register is programmed to enable the IE signal on the used DAI pin buffers, the pin-buffer outputs its signal on DAI\_PBxx\_O . The programming model needs to enable all used DAI pin buffers. For unused DAI pin-buffers (no external system connection), keep the corresponding IE-bit at a disabled state.

## Pin Buffer Functions

Pin buffers can be configured as inputs or outputs as described in the following sections.

## Pin Buffers as Signal Input

When the DAI pin is used only as an input, connect the corresponding pin buffer enable to logic low as shown in the Pin Buffer as Input figure. This configuration disables the buffer amplifier and allows an off-chip source to drive the value present on the DAI pin and at the pin buffer output. When the pin buffer enable (for example, DAI\_PBEN0.PB01 ) is cleared (= 0), the pin buffer output ( DAIn\_PBxx\_O ) is the signal driven onto the DAI pin by an external source, and the pin buffer input signal ( DAIn\_PBxx\_I ) is not used.

Figure 33-5: Pin Buffer as Input

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000004_970fee93071e123c56288b4b47a5dbf189178b885cf69a07dff67ae5cecd5045.png)

NOTE: Whether programmed as input or output, a DAI buffer input always routes the same signal to an output internally. The DAI also has an additional programming requirement when it is configured as input. The corresponding pin must be enabled in the PADS\_DAIx\_IE register. The DAI\_PB01\_O pin cannot be routed as an input source for clock (Group A signals) and frame sync (Group C). For more information, see DAI Routing Capabilities.

## Pin Buffers As Signal Output

In a typical embedded system, most pins are designated as either inputs or outputs when the circuit is designed, even though they can be used in either direction. Each of the DAI pins can be used as either an output or an input. Although the direction of a DAI pin is set simply by writing to a memory-mapped register, most often the direction of the pin is dictated by the designated use of that pin.

When the DAI pin is used only as an output, connect the corresponding pin buffer enable to logic high as shown in the Pin Buffer as Output figure. This configuration enables the buffer amplifier to operate as a current source and to drive the value present at the pin buffer input onto the DAI pin and off-chip. When the pin buffer enable bits are set (in the DAIn\_PBxx\_I registers) (=1), the pin buffer output ( DAIn\_PBxx\_O ) is the same signal as the pin buffer input ( DAIn\_PBxx\_I ), and this signal is driven as an output.

Figure 33-6: Pin Buffer as Output

## DAI Pin Buffer Status

The signal levels on the DAI pins can be read with the DAI\_PIN\_STAT registers.

## DAIn Peripherals

There are two categories of peripherals associated with the DAI units. These are described in the following sections.

## Output Signals With Pin Buffer Enable Control

Many peripherals within the DAIn that have bidirectional pins generate a corresponding pin enable signal. Typically, the settings within the control registers of a peripheral determine if a bidirectional pin is an input or an output. The pin is then driven accordingly.

Though most peripherals are capable of operating bidirectionally, it is not required that all of the \_I and \_O signals of a peripheral be connected to the pin buffer. If the system design only uses a signal in one direction, it is simpler to connect the pin buffer accordingly.

NOTE: All available pin buffer output enables must be routed to their pin buffer input enable signals in cases where data streaming connections are used. This arrangement guarantees timing requirements.

NOTE: In some cases, it is necessary to use a peripheral's dedicated pin buffer enable signal instead of static levels. For example, SPORT TDM mode requires the SPORT's dedicated data pin buffer enable signal to be used for the SPORT's data pin to three-state the data pins on inactive channels.

## Output Signals Without Pin Buffer Enable Control

Some peripherals have signal outputs without an automated pin buffer control enable signal. The operation of these peripherals is simplified. The routing to a DAIn pin buffer enable input requires a static high from the SRUn. In order to disable the pin buffer output, software must clear the pin buffer enable input accordingly.

## Signal Routing Units (SRUs)

The following sections provide details specific to the SRUs.

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000005_902e4df78c48e4187c64385431516d4dc0c9ddc9e8c261980ce26c74313f55a3.png)

## Cross Mode Connections

The DAI allows a few signals to be interconnected across DAI units (see Figure 33-2 Digital Audio Interconnect Unit). These connections are commonly referred as cross mode connections . These cross mode connections are useful for the system designer to share signals across both DAI's so that synchronization can be achieved on the peripheral modules across DAI0 and DAI1.

NOTE: The 349-ball CSP BGA package has 2 x 14 DAI pins bonded off-chip and continues to support 2 x 20 pin buffers for internal routings.

Shared Clock. This cross mode connection (see Figure 33-7 Cross Connect Clock Example) can be used to share a clock signal cross DAIs. DAI0\_PIN03 and DAI1\_PIN03 can be used to share clock signal across DAI. When DAI0\_PIN03 is configured as input clock pin then this clock signal is available in Group A of both DAIs. This source signal is DAI0\_PB03\_O in group A of DAI 0 and DAI1\_CRS\_PB03\_O in group A of DAI1.

Peripherals routed through DAI0 like PCG A and SPORT 0A can use DAI0\_PB03\_O as the input clock signal. Peripherals routed through DAI1 like PCG C, SPORT 4A can use DAI1\_CRS\_PB03 as the input clock signal. Similarly when DAI1\_PIN03 is configured as the input clock pin, then this clock signal is available as DAI1\_PB03\_O in group A of DAI1 and DAI0\_CRS\_PB03 in group A of DAI0.

Shared Frame Sync. This cross mode connection can be used to share a frame sync signal cross DAIs. DAI0\_PIN04 and DAI1\_PIN04 can be used to share the frame sync signal across the DAIs. When DAI0\_PIN04 is configured as the input frame sync pin then this frame sync signal is available in Group C of both DAIs. This source signal is DAI0\_PB04\_O in group C of DAI 0 and DAI1\_CRS\_PB04\_O in group C of DAI1.

Peripherals on DAI0 like SPORT 0A can use DAI0\_PB04\_O as the input frame sync signal. The peripherals on DAI1 like SPORT 4A can use DAI1\_CRS\_PB04 as the input sync signal. Similarly when DAI1\_PIN04 is configured as input frame sync pin, then this frame sync signal is available as DAI1\_PB04\_O in group A of DAI1 and DAI0\_CRS\_PB04 in group C of DAI0.

Shared Pin Buffer. This cross mode connection can be used to connect input signal on DAI0\_PIN03 , DAI0\_PIN04 , DAI1\_PIN03 and DAI1\_PIN04 to other DAI pins across DAI. When DAI0\_PIN03 and DAI0\_PIN04 are configured as input signals. These signals can be routed to the pin buffers of DAI1 in Group D using DAI1\_CRS\_PIN03 and DAI1\_CRS\_PIN04 . When DAI1\_PIN03 and DAI1\_PIN04 are configured as input signals they can be routed to the pin buffers of DAI0 in Group D using DAI0\_CRS\_PIN03 and DAI0\_CRS\_PIN04 signals.

PCGs and Pin Buffers. This cross mode connection (see Figure 33-8 Cross Connect PCG Example) can be used to connect PCG output signals of one DAI to pin buffers of other DAI. For example, PCG C outputs of DAI 1 can be connected to pin buffers of DAI 0 using PCG0\_CRS\_CLKC\_O of Group D.

ASRC TDM and Data Signal. This cross mode connection is available in Group B (Data Signals). This allows programs to connect the Data and TDM signals of SRC3 and SRC 7 across DAIs so that eight SRCs can be daisy chained for TDM 16 operation. For example SRC3\_DAT\_OP\_O and SRC3\_TDM\_IP\_O of DAI0 are available as SRC3\_CRS\_DAT\_OP\_O and SRC3\_CRS\_IP\_O in DAI1 so that DAI0 SRC3 can be connected to DAI1's SRC for daisy chaining.

Figure 33-7: Cross Connect Clock Example

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000006_92c1f5f41734aab77086f0b65635a1a106690d40db3dbcf0b74d1ada7718e266.png)

Figure 33-8: Cross Connect PCG Example

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000007_8a927e8117df870f53724f863bbd9fb800e0a97be6206f5c9b1f1f4782d7e7dd.png)

Refer to the DAI Group Routing section for more details on DAI routing.

## Signal Routing Matrix by Groups

The SRU is similar to a set of patch bays, which contain a bank of inputs and a bank of outputs. For each input (destination), there is a set of permissible output (source) options. Outputs can feed to any number of inputs in parallel, but every input must be patched to exactly one valid output source. Together, the set of inputs and outputs are called a group. The signal's inputs and outputs that comprise each group all serve similar purposes. They are compatible such that almost any output-to-input patch makes functional sense. With the grouping, the multiplexing

scheme becomes highly efficient since it does not make sense (for example) to route a frame sync signal to a data signal.

The SRU for the DAI contains six groups named A through F . Each group routes a unique set of signals with a specific purpose:

- Group A routes clock signals
- Group B routes serial data signals
- Group C routes frame sync signals
- Group D routes pin signals
- Group E routes miscellaneous signals
- Group F routes pin output enable signals

Together, the six groups of the SRU include all of the inputs and outputs of the DAI peripherals, a number of additional signals from the core, and all of the connections to the DAI pins.

NOTE: It is not possible to connect a signal in one group directly to a signal in a different group (analogous to wiring from one patch bay to another). However, group D (DAI) is largely devoted to routing in this vein.

## DAI Group Routing

Each group has a unique encoding for its associated output signals and a set of configuration registers. For example, DAI group A is used to route clock signals. The memory-mapped group A registers, DAI\_CLK0 through DAI\_CLK5 , contain bit fields corresponding to the clock inputs of various peripherals. The values written to these bit fields specify a signal source that is an output from another peripheral. All of the possible encodings represent sources that are clock signals (or at least could be clock signals in some systems). The Example DAI Group A Multiplexing (DAI\_CLKx) diagrams the input signals that are controlled by the group A registers. All bit fields in the SRU configuration registers correspond to inputs. The value written to the bit field specifies the signal source. This value is also an output from some other component within the SRU.

The SRU is similar to a set of patch bays. Each bay routes a distinct set of outputs to compatible inputs. These connections are implemented as a set of memory-mapped registers with a bit field for each input. The outputs are implemented as a set of bit encodings. Conceptually, a patch cord is used to connect an output to an input. In the SRU, a bit pattern that is associated with a signal output (shown in the Example DAI Group A Multiplexing (DAI\_CLKx) figure) is written to a bit field corresponding to a signal input.

The same encoding can be written to any number of bit fields in the same group. It is not possible to run out of patch points for an output signal.

Figure 33-9: Example DAI Group A Multiplexing (DAIn\_CLKx)

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000008_41b1c09cb1559994d8a6121e81bc4c877f98e1f19e20a439e8360b928bc7fcfc.png)

Just as group A routes clock signals, each of the other groups route a collection of compatible signals. Group B routes serial data streams while group C routes frame sync signals. All of the groups have an encoding that allow a signal to flow from a pin output to the input being specified by the bit field.

Group D routes signals to pins so that they may be driven off-chip (required to route a signal to the pin input). Group F routes signals to the pin enables, and the value of these signals determines if a DAI pin is used as an output or an input. The input of one pin can be patched to the output of another pin, allowing board-level routing under software control.

## Rules for SRU Connections

There are two rules which apply to all routing:

1. One source (output node) can drive different destinations (input nodes).
2. One destination (input node) can only be assigned to one source (output node).

As an example from the Example DAI Group A Multiplexing (DAI\_CLKx) figure:

- DAI0\_CRS\_PB03\_O is routed to SPT0\_ACLK\_I
- DAI0\_CRS\_PB03\_O is routed to SPT0\_BCLK\_I
- SPT4\_ACLK\_O is routed to SPT3\_ACLK\_I

NOTE: Inputs may only be connected to outputs.

## Miscellaneous Buffers and Functions

The SRU group E provides miscellaneous buffers used for group interconnect.

DAI group E connections are slightly different from the others in that the inputs and outputs being routed vary considerably in function. This group routes control signals and provides a means of connecting signals between groups.

In the DAI MISCAx SRU Signal Connections table, the DAIn\_MISCAx\_I signals appear as inputs in group E (also connected to the DAI interrupt logic), but do not directly feed any peripheral. Rather, the MISCAx\_O signals reappear as outputs in group F .

Table 33-3: DAI MISCAx SRU Signal Connections

| MISCA Source    | DAI Connection   | MISCA Destination   |
|-----------------|------------------|---------------------|
|                 | Group E          | DAIn_MISCA5-0_I     |
| DAIn_MISCA5-0_O | Group F          |                     |

Additional connections among groups provide a great amount of utility. Since the output groups F (DAI) dictate pin direction, these few signal paths enable a number of possible uses and connections for the DAI pins. Other examples include:

- A pin input can be patched to another pin's enable, allowing an off-chip signal to gate an output from the processor.
- Any of the DAI pins can be used as interrupt sources or general-purpose I/O (GPIO) signals.

In summary, the SRU enables many possible functional changes, both within the processor as well as externally. Used creatively, it allows system designers to radically change functionality at run time, and to potentially reuse circuit boards across many products.

## DAI Routing Capabilities

This section describes the routing options to aid in designing a system using the DAI units. The Cross Mode Connections section provides information on connecting signals across DAI0 and DAI1. The DAI Default Routing section provides diagrams that show how that DAIs connect at default. The DAI Group tables provide the source signals and selections codes. Finally, the ADSP-SC58x DAI Register Descriptions provides information about configuring destinations.

The DAI Routing Capabilities tables provide an overview of the different routing capabilities for the DAI unit. For information on an individual peripherals routing, see the 'SRU Programming' section of the specific peripheral chapter.

Table 33-4: DAI0 Routing Capabilities

| Source Signals - Output (xxxx_O)                                                                                                                                   | Source Signals - Output (xxxx_O)                                                                    | DAI0 Group                | Destination Signals - Input (xxxx_I)                                                           |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------|---------------------------|------------------------------------------------------------------------------------------------|
| SPORT2B-0A (clocks) PCG A, B S/PDIF0 Rx (clock, TDMclock)                                                                                                          | DAI0 pin buffer 20-2 DAI0 CRS pin buffer (buffer 3 from other DAI) Logic level high Logic level low | A-clocks                  | SPORT3B-0A (clocks) SRC3-0 (clocks) PCG A-B ext. clock, ext. sync S/PDIF0 Tx (clock, HF clock) |
| SPORT3B-0A (data) SRC3-0 (data, TDMdata) S/PDIF0 Tx/Rx (data) SRC7 from other DAI (data, TDMdata)                                                                  | DAI0 pin buffer 20-1 Logic level high Logic level low                                               | B-data                    | SPORT3B-0A (data) SRC3-0 (data, TDMdata) S/PDIF0 Tx/Rx (data)                                  |
| SPORT2B-0A (FS) PCG A, B (FS) S/PDIF0 Rx (FS)                                                                                                                      | DAI0 pin buffer 20-2 DAI0 CRS pin buffer (buffer 4 from other DAI) Logic level high Logic level low | C-frame sync              | SPORT3B-0A (FS) SRC3-0 (FS)                                                                    |
| SPORT3B-0A (clock, FS, TDV, data) PCG A,B (clock, FS) S/PDIF0 Rx (clock, TDMclock, FS, data) S/PDIF0 Tx (data, block start) PCG C, Dfrom the other DAI (clock, FS) | DAI0 pin buffer 20-1 DAI0 pin buffer 3 and 4 from other DAI Logic level high Logic level low        | D-pin buffer inputs       | DAI0 pin buffer 20-1 Options: DAI0 pin buffer 20-19 Polarity change                            |
| SPORT2B-0A (FS) PCG A (clock) PCG B (clock, FS) S/PDIF0 Tx (block start)                                                                                           | DAI0 pin buffer 20-1 Logic level high Logic level low                                               | E-miscellaneous sig- nals | DAI Interrupt 31-22 MISCA5-0 Options: MISCA5- Polarity change                                  |
| SPORT3B-0A (clock, FS, data, TDV) MISCA5-0                                                                                                                         | DAI0 pin buffer 20-1 Logic level high Logic level low                                               | F-pin buffer enable       | DAI0 pin buffer enable 20-1                                                                    |

Table 33-5: DAI1 Routing Capabilities

| Source Signals - Output (xxxx_O)                                                                                                                                    |                                                                                                     | DAI1 Group                | Destination Signals - Input (xxxx_I)                                                           |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------|---------------------------|------------------------------------------------------------------------------------------------|
| SPORT6B-4A (clocks) PCG C, D(clocks) S/PDIF1 Rx (clock, TDMclock)                                                                                                   | DAI1 Pin Buffer 20-2 DAI1 CRS pin buffer (buffer 3 from other DAI) Logic level high Logic level low | A-Clocks                  | SPORT7B-4A (clocks) SRC4-7 (clocks) PCG C-D Ext. clock, Ext. sync S/PDIF1 Tx (clock, HF clock) |
| SPORT6B-4A (data) SRC4-7 (data, TDMdata) S/PDIF1 Tx/Rx (data) SRC3 from other DAI (data, TDMdata)                                                                   | DAI1 Pin Buffer 20-1 Logic level high Logic level low                                               | B-Data                    | SPORT7B-4A (data) SRC4-7 (data, TDMdata) S/PDIF1 Tx/Rx (data)                                  |
| SPORT6B-4A (FS) PCG C, D(FS) S/PDIF1 Rx (FS)                                                                                                                        | DAI1 Pin Buffer 20-2 DAI1 CRS pin buffer (buffer 4 from other DAI) Logic level high Logic level low | C-Frame Sync              | SPORT7B-4A (FS) SRC4-7 (FS)                                                                    |
| SPORT6B-4A (clock, FS, TDV, data) PCG C, D(clock, FS) S/PDIF1 Rx (clock, TDMclock, FS, data) S/PDIF1 Tx (data, block start) PCG A, B from the other DAI (clock, FS) | DAI1 Pin Buffer 20-1 DAI1 Pin Buffer 3 and 4 from other DAI Logic level high Logic level low        | D-Pin Buffer Inputs       | DAI1 Pin Buffer 20-1 Options: DAI1 Pin Buffer 20-19 Polarity Change                            |
| SPORT6B-4A (FS) PCG C (clock) PCG D(clock, FS) S/PDIF1 Tx (block start)                                                                                             | DAI1 Pin Buffer 20-1 Logic level high Logic level low                                               | E-Miscellaneous Sig- nals | DAI Interrupt 31-22 MISCA5-0 Options: MISCA5-4 Polarity Change                                 |
| SPORT7B-4A (clock, FS, data, TDV) MISCA5-0                                                                                                                          | Logic level high Logic level low                                                                    | F-Pin Buffer Enable       | DAI1 Pin Buffer Enable 20-1                                                                    |

NOTE: For DAI sources (Group A, clocks) and (Group C, FS) only DAI pin buffer 2 through 20 can be used (DAI pin buffer 1 is no longer available and replaced by DAI CRS buffer for the other DAI).

## DAI Default Routing

When the processor comes out of reset, the SPORT junctions are bidirectional to the DAI pin buffers. This configuration allows systems to use the SPORTs as either master or slave (without changing the routing scheme). Therefore, programs only need to use the SPORT control register settings to configure master or slave operations. Note that all DAI inputs which are not routed by default are tied to signal low.

NOTE: All DAI input buffers which are not routed by default are driven low and all DAI pin buffer enable signals are driven low.

Figure 33-10: DAI0 Default Routing Pins 01-08

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000009_6b530d7259015908df4f65b5976ce953eb2b9a260b8378cb0db7a42993ac5acf.png)

Figure 33-11: DAI0 Default Routing Pins 09-20

Figure 33-12: DAI1 Default Routing Pins 01-08

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000010_f055e7ef68fb004ff513f9e092203da5b3e802d0b2a63348c0a430d77aab49aa.png)

Figure 33-13: DAI1 Default Routing Pins 09-20

## Unused DAI Connections

The SRUs have a default general-purpose routing scheme which can be modified to accomodate any number of different system designs. Regardless of the system design, it is good practice to tie all unused inputs to a high or low level to reduce dynamic power consumption.

## Un-Bonded DAI Buffers

Some product variants have some DAI pin buffer not bonded off-chip. Note thee un bonded-DAI buffers can continued to be used for internal signal routing purposes (for example exchange CLK/FS/data between DAI buffers).

## DAI Operating Modes

Some buffers allow polarity changes, described as follows.

## DAI Pin Buffer Polarity

As shown in the Pin Buffer Polarity figure, the DAI pin buffer 20-19 can change the polarity of the input signal if the corresponding control bits ( DAI\_PIN4.INV20 , DAI\_PIN4.INV19 ) are set. These bits can be set during run time and the buffer should not loopback to itself.

Figure 33-14: Pin Buffer Polarity

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000011_7c415367f1a4c9cf62cf9d845fa1c132cd109dcfe4b62ac3c105ef333d9d5c21.png)

## DAI Miscellaneous Buffer Polarity

As shown in the Pin Buffer Polarity figure, the A5-4 miscellaneous buffers can change the polarity of the input signal if the corresponding control bits ( DAI\_MISC1.IN5 , DAI\_MISC1.IN4 ) are set. Both buffers are not connected to the DAI interrupt latch register. These bits can be set during run time.

Figure 33-15: Miscellaneous Buffer Polarity

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000012_f401f0d938185655ed94c317713c3cb9cc01203f864ed2c292124971be50aad9.png)

## DAI System Interrupt Controller (SIC)

The DAI module incorporates a system interrupt controller (SIC) which is connected to the SEC and GIC as seen in the DAI System Interrupt Controller figure.

Figure 33-16: DAI System Interrupt Controller

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000013_0b0c27bd1aacb618154aa38dde68d25c2d7f3db08e4112305d850df0c0e49440.png)

The DAI has its own system interrupt controllers that indicate to the core when DAI audio peripheral-related events have occurred. Since audio events generally occur infrequently relative to the SHARC core, the DAI interrupt controller reduces all of its interrupts onto two interrupt signals within the core's primary interrupt systems. One interrupt is mapped with DAI low priority. The second interrupt is mapped with DAI high priority. This configuration allows programs to broadly indicate priority. In this way, the DAI SIC provides 32 independently configurable sources or channels. The output bus interrupt signals are logically OR'ed into one interrupt line and fed to the interrupt controller logic of the core.

Three registers are used to configure the DAI interrupt controller. Each of the 32 interrupt sources can be independently configured to trigger on a rising edge, falling edge, both edges, or neither edge of an incoming signal. All DAI interrupt control registers are memory-mapped registers and are accessed through the peripheral bus.

## Interrupt Sources

The DAI's five peripheral sources are multiplexed into 32 interrupt sources and are labeled DAI\_INT31-0 ( DAI Interrupt Sources table).

NOTE: There are two naming conventions. The DAI interrupt controller register bits are labeled DAI\_31-0\_INT. Their corresponding SRU routing signals are labeled DAI\_INT\_31-0\_I.

Table 33-6: DAI Interrupt Sources

| Interrupt Source     | Description                          | Signal Response   |
|----------------------|--------------------------------------|-------------------|
| DAI_INT2-0, DAI_INT4 | S/PDIF RX, 4 channels                | Event             |
| DAI_INT2-0, DAI_INT4 | S/PDIF RX, 4 channels                | Waveform          |
| DAI_INT21-18         | ASRC, 4 channels                     | Waveform          |
| DAI_INT31-22         | Miscellaneous, S/PDIF TX, 9 channels | Waveform          |

## Interrupt Latch Priority Option

The DAI\_IMSK\_PRI register specifies the priority for the DAI interrupt channels. DAI system interrupt controller has a pair of interrupt latch registers, DAI\_IRPTL\_H and DAI\_IRPTL\_L . The configuration of the DAI\_IMSK\_PRI register also determines the interrupt latch mapping for a particular DAI interrupt. When a DAI interrupt is configured as low priority ( DAI\_IMSK\_PRI bit cleared, default setting), interrupts are mapped to the DAI\_INTR\_IRQL signal and when an interrupt occurs, the corresponding bit of the DAI\_IRPTL\_L register is set. When a DAI interrupt is configured as high priority ( DAI\_IMSK\_PRI bit set), interrupts are mapped to the DAI\_INTR\_IRQH signal and the interrupt is latched to the DAI\_IRPTL\_H register. The low priority DAI interrupt (INTR\_DAI\_IRQL) and high priority DAI interrupt (INTR\_DAI\_IRQH) are connected to the SEC and GIC.

## Interrupt Mask for Waveforms

The DAI\_IMSK\_RE and DAI\_IMSK\_FE registers allow programs to mask or unmask interrupts for specific edges of a signal mapped to the channel. It can be configured for rising edges, falling edges, both rising and falling edges, or neither rising nor falling edges by masking them separately. Signals from the SRU can be used to generate interrupts. For example, when the DAI\_IMSK\_FE.MISCINT9 bit is set to one, any falling edge signals from the external channel generate an interrupt and the interrupt latch is set.

## Interrupt Mask for Events

The system interrupt controller needs information about a peripheral's interrupt sources that correspond to event signals (see the DAI Interrupt Sources table). As a result, the rising edge is used as an interrupt source only. For DAI peripherals marked as events, programs may unmask an interrupt source on the rising edge only.

## Shadow Interrupt Register

The DAI interrupt controller has shadow registers to simplify debug activities since these registers do not manipulate status control. Any read of the shadow registers ( DAI\_IRPTL\_HS , DAI\_IRPTL\_LS ), provides the same data as a read of the DAI\_IRPTL\_H and DAI\_IRPTL\_L registers. However, reads of the DAI shadow registers do not change the interrupt acknowledge status to the core interrupt controller.

## Interrupt Service

The interrupt acknowledge operates differently when multiple channels are multiplexed into one interrupt output signal. When an interrupt from the DAI must be serviced, any of the two interrupt service routines (INTR\_DAI\_IRQL, INTR\_DAI\_IRQH) must query the SIC to determine the source(s). Sources can be any one or more of the DAI channels (DAI\_INT31-0).

- When the DAI\_IRPTL\_H register is read, the high priority latched interrupts are cleared.
- When the DAI\_IRPTL\_L register is read, the low priority latched interrupts are cleared.

If an interrupt occurs in the same cycle as a latch register is cleared, the clear mechanism has lower priority and the new interrupt is registered.

## Signal Routing Unit Effect Latency

After the DAIy registers are configured the effect latency is 2 SCLK0 cycles minimum and 3 SCLK0 cycles maximum.

## DAI Programming Model

As discussed in the previous sections, the signal routing unit is controlled by writing values that provide a plug-in tool in CCES so that configuring the SRU is done graphically. Analog Devices offers macros that are included with the CrossCore or VisualDSP++ tools, greatly easing code development in the SRU.

There is a macro that has been created to connect peripherals used in a DAI configuration. This code can be used in both assembly and C code. See the INCLUDE file SRU.H. In practice the macro is provided and forms the style SRU(source\_O, destination\_I) for DAI0 and SRU2(source\_O, destination\_I) for DAI1. Example appears as: SRU(DAI0\_PB12\_O, SPT0\_ACLK\_I); or SRU2(DAI1\_PB14\_O, SPT4\_ACLK\_I);

## Debug Features

The following section describes the feature that can be used to help in debugging the DAI.

## Loopback Routing

The SPORT serial peripheral supports an internal loopback mode. If the loopback bit for each peripheral is enabled, it connects the transmitter with the receiver block internally (does not signal off-chip). The SRU can be used for this purpose. The Loopback Routing table describes the different possible routings based on the peripheral.

NOTE: The peripheral's loopback mode for debug is independent from both of the signal routing units.

Table 33-7: Loopback Routing

| Peripheral   | Loopback Mode   | SRU1-0 Internal Routing for Loopback   | SRU1-0 External Routing for Loopback                  |
|--------------|-----------------|----------------------------------------|-------------------------------------------------------|
| SPORT        | Yes             | SPTx_xx_O → SPTx_xx_I                  | SPTx_xx_O → DAIn_PBxx_I DAIn_PBxx_O → SPTx_xx_I       |
| S/PDIF Tx/Rx | No              | SPDIFn_TX_O → SPDIFn_RX_I              | SPDIFn_TX_O → DAI_PBxx_I DAI_PBxx_O → SPDIFn_RX_I     |
| SRC          | No              | SRCx_DAT_OP_O → SRCx_DAT_IP_I          | SRCx_DAT_OP_O → DAI_PBxx_I DAI_PBxx_O → SRCx_DAT_IP_I |

## DAI Sources Overview

The following tables provide information of the various DAI sources sorted by group. The shaded cells in the tables denote connection to the other DAI.

## DAI0 Group A - Clock Routing Source Signals

The group A clock sources are listed in the following table. The Group A clock destinations are configured using Clock Routing Control Register 0 through Clock Routing Control Register 5 .

Table 33-8: DAI0 Group A - Clock Routing

| Selection Code   | Source Signal   | Description (Source Selection)   |
|------------------|-----------------|----------------------------------|
| 00000 (0x0)      | DAI0_CRS_PB03_O | Pin Buffer 3 (DAI1)              |
| 00001 (0x1)      | DAI0_PB02_O     | Pin Buffer 2                     |
| 00010 (0x2)      | DAI0_PB03_O     | Pin Buffer 3                     |
| 00011 (0x3)      | DAI0_PB04_O     | Pin Buffer 4                     |
| 00100 (0x4)      | DAI0_PB05_O     | Pin Buffer 5                     |
| 00101 (0x5)      | DAI0_PB06_O     | Pin Buffer 6                     |
| 00110 (0x6)      | DAI0_PB07_O     | Pin Buffer 7                     |
| 00111 (0x7)      | DAI0_PB08_O     | Pin Buffer 8                     |
| 01000 (0x8)      | DAI0_PB09_O     | Pin Buffer 9                     |

Table 33-8: DAI0 Group A - Clock Routing (Continued)

| Selection Code   | Source Signal      | Description (Source Selection)   |
|------------------|--------------------|----------------------------------|
| 01001 (0x9)      | DAI0_PB10_O        | Pin Buffer 10                    |
| 01010 (0xA)      | DAI0_PB11_O        | Pin Buffer 11                    |
| 01011 (0xB)      | DAI0_PB12_O        | Pin Buffer 12                    |
| 01100 (0xC)      | DAI0_PB13_O        | Pin Buffer 13                    |
| 01101 (0xD)      | DAI0_PB14_O        | Pin Buffer 14                    |
| 01110 (0xE)      | DAI0_PB15_O        | Pin Buffer 15                    |
| 01111 (0xF)      | DAI0_PB16_O        | Pin Buffer 16                    |
| 10000 (0x10)     | DAI0_PB17_O        | Pin Buffer 17                    |
| 10001 (0x11)     | DAI0_PB18_O        | Pin Buffer 18                    |
| 10010 (0x12)     | DAI0_PB19_O        | Pin Buffer 19                    |
| 10011 (0x13)     | DAI0_PB20_O        | Pin Buffer 20                    |
| 10100 (0x14)     | SPT0_ACLK_O        | SPORT 0 Clock A                  |
| 10101 (0x15)     | SPT0_BCLK_O        | SPORT 0 Clock B                  |
| 10110 (0x16)     | SPT1_ACLK_O        | SPORT 1 Clock A                  |
| 10111 (0x17)     | SPT1_BCLK_O        | SPORT 1 Clock B                  |
| 11000 (0x18)     | SPT2_ACLK_O        | SPORT 2 Clock A                  |
| 11001 (0x19)     | SPT2_BCLK_O        | SPORT 2 Clock B                  |
| 11010 (0x1A)     | SPDIF0_RX_CLK_O    | SPDIF 0 Receive Clock Output     |
| 11011 (0x1B)     | SPDIF0_RX_TDMCLK_O | SPDIF 0 Receive TDMClock Output  |
| 11100 (0x1C)     | PCG0_CLKA_O        | Precision Clock A Output         |
| 11101 (0x1D)     | PCG0_CLKB_O        | Precision Clock B Output         |
| 11110 (0x1E)     | LOW                | Logic Level Low (0)              |
| 11111 (0x1F)     | HIGH               | Logic Level High (1)             |

## DAI0 Group B - Serial Data Source Signals

The group B data sources are listed in the following table. The group B data destinations are configured using Serial Data Routing Control Register 0 through Serial Data Routing Control Register 6 .

Table 33-9: DAI0 Group B - Serial Data Signals

| Selection Code   | Source Signal   | Description (Source Selection)   |
|------------------|-----------------|----------------------------------|
| 000000 (0x0)     | DAI0_PB01_O     | Pin Buffer 1                     |
| 000001 (0x1)     | DAI0_PB02_O     | Pin Buffer 2                     |

Table 33-9: DAI0 Group B - Serial Data Signals (Continued)

| Selection Code   | Source Signal   | Description (Source Selection)   |
|------------------|-----------------|----------------------------------|
| 000010 (0x2)     | DAI0_PB03_O     | Pin Buffer 3                     |
| 000011 (0x3)     | DAI0_PB04_O     | Pin Buffer 4                     |
| 000100 (0x4)     | DAI0_PB05_O     | Pin Buffer 5                     |
| 000101 (0x5)     | DAI0_PB06_O     | Pin Buffer 6                     |
| 000110 (0x6)     | DAI0_PB07_O     | Pin Buffer 7                     |
| 000111 (0x7)     | DAI0_PB08_O     | Pin Buffer 8                     |
| 001000 (0x8)     | DAI0_PB09_O     | Pin Buffer 9                     |
| 001001 (0x9)     | DAI0_PB10_O     | Pin Buffer 10                    |
| 001010 (0xA)     | DAI0_PB11_O     | Pin Buffer 11                    |
| 001011 (0xB)     | DAI0_PB12_O     | Pin Buffer 12                    |
| 001100 (0xC)     | DAI0_PB13_O     | Pin Buffer 13                    |
| 001101 (0xD)     | DAI0_PB14_O     | Pin Buffer 14                    |
| 001110 (0xE)     | DAI0_PB15_O     | Pin Buffer 15                    |
| 001111 (0xF)     | DAI0_PB16_O     | Pin Buffer 16                    |
| 010000 (0x10)    | DAI0_PB17_O     | Pin Buffer 17                    |
| 010001 (0x11)    | DAI0_PB18_O     | Pin Buffer 18                    |
| 010010 (0x12)    | DAI0_PB19_O     | Pin Buffer 19                    |
| 010011 (0x13)    | DAI0_PB20_O     | Pin Buffer 20                    |
| 010100 (0x14)    | SPT0_AD0_O      | SPORT 0 Data AD0                 |
| 010101 (0x15)    | SPT0_AD1_O      | SPORT 0 Data AD1                 |
| 010110 (0x16)    | SPT0_BD0_O      | SPORT 0 Data BD0                 |
| 010111 (0x17)    | SPT0_BD1_O      | SPORT 0 Data BD1                 |
| 011000 (0x18)    | SPT1_AD0_O      | SPORT 1 Data AD0                 |
| 011001 (0x19)    | SPT1_AD1_O      | SPORT 1 Data AD1                 |
| 011010 (0x1A)    | SPT1_BD0_O      | SPORT 1 Data BD0                 |
| 011011 (0x1B)    | SPT1_BD1_O      | SPORT 1 Data BD1                 |
| 011100 (0x1C)    | SPT2_AD0_O      | SPORT 2 Data AD0                 |
| 011101 (0x1D)    | SPT2_AD1_O      | SPORT 2 Data AD1                 |
| 011110 (0x1E)    | SPT2_BD0_O      | SPORT 2 Data BD0                 |
| 011111 (0x1F)    | SPT2_BD1_O      | SPORT 2 Data BD1                 |
| 100000 (0x20)    | SRC0_DAT_OP_O   | SRC0 Data Out                    |

Table 33-9: DAI0 Group B - Serial Data Signals (Continued)

| Selection Code            | Source Signal     | Description (Source Selection)   |
|---------------------------|-------------------|----------------------------------|
| 100001 (0x21)             | SRC1_DAT_OP_O     | SRC1 Data Out                    |
| 100010 (0x22)             | SRC2_DAT_OP_O     | SRC2 Data Out                    |
| 100011 (0x23)             | SRC3_DAT_OP_O     | SRC3 Data Out                    |
| 100100 (0x24)             | SRC0_TDM_IP_O     | SRC0 Data Out                    |
| 100101 (0x25)             | SRC1_TDM_IP_O     | SRC1 Data Out                    |
| 100110 (0x26)             | SRC2_TDM_IP_O     | SRC2 Data Out                    |
| 100111 (0x27)             | SRC3_TDM_IP_O     | SRC3 Data Out                    |
| 101000 (0x28)             | SPDIF0_RX_DAT_O   | SPDIF 0 RX Serial Data Out       |
| 101100(0x2C)              | SPT3_AD0_O        | SPORT 3 Data AD0                 |
| 101101(0x2D)              | SPT3_AD1_O        | SPORT 3 Data AD1                 |
| 101110(0x2E)              | SPT3_BD0_O        | SPORT 3 Data BD0                 |
| 101111(0x2F)              | SPT3_BD1_O        | SPORT 3 Data BD1                 |
| 110000(0x30)              | SPDIF0_TX_O       | SPDIF 0 TX Biphase Stream        |
| 110001(0x31)              | SRC7_CRS_DAT_OP_O | SRC7 data out (DAI1)             |
| 110010(0x32)              | SRC7_CRS_TDM_IP_O | SRC7 data out (DAI1)             |
| 110001(0x33)-111101(0x3D) | Reserved          | Reserved                         |
| 111110 (0x3E)             | LOW               | Logic Level Low (0)              |
| 111111 (0x3F)             | HIGH              | Logic Level High (1)             |

## DAI0 Group C - Frame Sync Source Signals

The group C frame sync signal sources are listed in the following table. The frame sync destinations are configured using Frame Sync Routing Control Register 0 through Frame Sync Routing Control Register 4 .

Table 33-10: DAI0 Group C - Frame Sync Signals

| Selection Code   | Source Signal   | Description (Source Selection)   |
|------------------|-----------------|----------------------------------|
| 00000 (0x0)      | DAI0_CRS_PB04_O | Pin Buffer 4 (DAI1)              |
| 00001 (0x1)      | DAI0_PB02_O     | Pin Buffer 2                     |
| 00010 (0x2)      | DAI0_PB03_O     | Pin Buffer 3                     |
| 00011 (0x3)      | DAI0_PB04_O     | Pin Buffer 4                     |
| 00100 (0x4)      | DAI0_PB05_O     | Pin Buffer 5                     |
| 00101 (0x5)      | DAI0_PB06_O     | Pin Buffer 6                     |
| 00110 (0x6)      | DAI0_PB07_O     | Pin Buffer 7                     |

Table 33-10: DAI0 Group C - Frame Sync Signals (Continued)

| Selection Code   | Source Signal   | Description (Source Selection)   |
|------------------|-----------------|----------------------------------|
| 00111 (0x7)      | DAI0_PB08_O     | Pin Buffer 8                     |
| 01000 (0x8)      | DAI0_PB09_O     | Pin Buffer 9                     |
| 01001 (0x9)      | DAI0_PB10_O     | Pin Buffer 10                    |
| 01010 (0xA)      | DAI0_PB11_O     | Pin Buffer 11                    |
| 01011 (0xB)      | DAI0_PB12_O     | Pin Buffer 12                    |
| 01100 (0xC)      | DAI0_PB13_O     | Pin Buffer 13                    |
| 01101 (0xD)      | DAI0_PB14_O     | Pin Buffer 14                    |
| 01110 (0xE)      | DAI0_PB15_O     | Pin Buffer 15                    |
| 01111 (0xF)      | DAI0_PB16_O     | Pin Buffer 16                    |
| 10000 (0x10)     | DAI0_PB17_O     | Pin Buffer 17                    |
| 10001 (0x11)     | DAI0_PB18_O     | Pin Buffer 18                    |
| 10010 (0x12)     | DAI0_PB19_O     | Pin Buffer 19                    |
| 10011 (0x13)     | DAI0_PB20_O     | Pin Buffer 20                    |
| 10100 (0x14)     | SPT0_AFS_O      | SPORT 0 Frame Sync A             |
| 10101 (0x15)     | SPT0_BFS_O      | SPORT 0 Frame Sync B             |
| 10110 (0x16)     | SPT1_AFS_O      | SPORT 1 Frame Sync A             |
| 10111 (0x17)     | SPT1_BFS_O      | SPORT 1 Frame Sync B             |
| 11000 (0x18)     | SPT2_AFS_O      | SPORT 2 Frame Sync A             |
| 11001 (0x19)     | SPT2_BFS_O      | SPORT 2 Frame Sync B             |
| 11010 (0x1A)     | SPDIF0_FS_O     | SPDIF 0 RX Frame Sync Output     |
| 11011 (0x1B)     | Reserved        | Reserved                         |
| 11100 (0x1C)     | PCG0_FSA_O      | Precision Frame Sync A Output    |
| 11101 (0x1D)     | PCG0_FSB_O      | Precision Frame Sync B Output    |
| 11110 (0x1E)     | LOW             | Logic Level Low (0)              |
| 11111 (0x1F)     | HIGH            | Logic Level High (1)             |

## DAI0 Group D - Pin Signal Assignment Source Signals

The group D pin signal assignment sources are listed in the following table. The group D destinations are configured using Pin Buffer Assignment Register 0 through Pin Buffer Assignment Register 4 .

Table 33-11: DAI0 Group D - Pin Signal Assignments

| Selection Code   | Source Signal   | Description (Source Selection)   |
|------------------|-----------------|----------------------------------|
| 0000000 (0x0)    | DAI0_PB01_O     | Pin Buffer 1                     |
| 0000001 (0x1)    | DAI0_PB02_O     | Pin Buffer 2                     |
| 0000010 (0x2)    | DAI0_PB03_O     | Pin Buffer 3                     |
| 0000011 (0x3)    | DAI0_PB04_O     | Pin Buffer 4                     |
| 0000100 (0x4)    | DAI0_PB05_O     | Pin Buffer 5                     |
| 0000101 (0x5)    | DAI0_PB06_O     | Pin Buffer 6                     |
| 0000110 (0x6)    | DAI0_PB07_O     | Pin Buffer 7                     |
| 0000111 (0x7)    | DAI0_PB08_O     | Pin Buffer 8                     |
| 0001000 (0x8)    | DAI0_PB09_O     | Pin Buffer 9                     |
| 0001001 (0x9)    | DAI0_PB10_O     | Pin Buffer 10                    |
| 0001010 (0xA)    | DAI0_PB11_O     | Pin Buffer 11                    |
| 0001011 (0xB)    | DAI0_PB12_O     | Pin Buffer 12                    |
| 0001100 (0xC)    | DAI0_PB13_O     | Pin Buffer 13                    |
| 0001101 (0xD)    | DAI0_PB14_O     | Pin Buffer 14                    |
| 0001110 (0xE)    | DAI0_PB15_O     | Pin Buffer 15                    |
| 0001111 (0xF)    | DAI0_PB16_O     | Pin Buffer 16                    |
| 0010000 (0x10)   | DAI0_PB17_O     | Pin Buffer 17                    |
| 0010001 (0x11)   | DAI0_PB18_O     | Pin Buffer 18                    |
| 0010010 (0x12)   | DAI0_PB19_O     | Pin Buffer 19                    |
| 0010011 (0x13)   | DAI0_PB20_O     | Pin Buffer 20                    |
| 0010100 (0x14)   | SPT0_AD0_O      | SPORT 0 Data AD0                 |
| 0010101 (0x15)   | SPT0_AD1_O      | SPORT 0 Data AD1                 |
| 0010110 (0x16)   | SPT0_BD0_O      | SPORT 0 Data BD0                 |
| 0010111 (0x17)   | SPT0_BD1_O      | SPORT 0 Data BD1                 |
| 0011000 (0x18)   | SPT1_AD0_O      | SPORT 1 Data AD0                 |
| 0011001 (0x19)   | SPT1_AD1_O      | SPORT 1 Data AD1                 |
| 0011010 (0x1A)   | SPT1_BD0_O      | SPORT 1 Data BD0                 |
| 0011011 (0x1B)   | SPT1_BD1_O      | SPORT 1 Data BD1                 |
| 0011100 (0x1C)   | SPT2_AD0_O      | SPORT 2 Data AD0                 |
| 0011101 (0x1D)   | SPT2_AD1_O      | SPORT 2 Data AD1                 |
| 0011110 (0x1E)   | SPT2_BD0_O      | SPORT 2 Data BD0                 |

Table 33-11: DAI0 Group D - Pin Signal Assignments (Continued)

| Selection Code   | Source Signal        | Description (Source Selection)   |
|------------------|----------------------|----------------------------------|
| 0011111 (0x1F)   | SPT2_BD1_O           | SPORT 2 Data BD1                 |
| 0100000 (0x20)   | SPT0_ACLK_O          | SPORT 0 Clock A                  |
| 0100001 (0x21)   | SPT0_BCLK_O          | SPORT 0 Clock B                  |
| 0100010 (0x22)   | SPT1_ACLK_O          | SPORT 1 Clock A                  |
| 0100011 (0x23)   | SPT1_BCLK_O          | SPORT 1 Clock B                  |
| 0100100 (0x24)   | SPT2_ACLK_O          | SPORT 2 Clock A                  |
| 0100101 (0x25)   | SPT2_BCLK_O          | SPORT 2 Clock B                  |
| 0100110 (0x26)   | SPT0_AFS_O           | SPORT 0 Frame Sync A             |
| 0100111 (0x27)   | SPT0_BFS_O           | SPORT 0 Frame Sync B             |
| 0101000 (0x28)   | SPT1_AFS_O           | SPORT 1 Frame Sync A             |
| 0101001 (0x29)   | SPT1_BFS_O           | SPORT 1 Frame Sync B             |
| 0101010 (0x2A)   | SPT2_AFS_O           | SPORT 2 Frame Sync A             |
| 0101011 (0x2B)   | SPT2_BFS_O           | SPORT 2 Frame Sync B             |
| 0101100 (0x2C)   | SPT3_AD0_O           | SPORT 3 Data AD0                 |
| 0101101 (0x2D)   | SPT3_AD1_O           | SPORT 3 Data AD1                 |
| 0101110 (0x2E)   | SPT3_BD0_O           | SPORT 3 Data BD0                 |
| 0101111 (0x2F)   | SPT3_BD1_O           | SPORT 3 Data BD1                 |
| 0110000 (0x30)   | MLB0_CLKOUT          | MLB PLL clock output             |
| 0110001 (0x31)   | SPDIF0_TX_BLKSTART_O | SPDIF 0 TX Block Start Output    |
| 0110100 (0x34)   | SPT3_ACLK_O          | SPORT 3 Clock A                  |
| 0110101 (0x35)   | SPT3_BCLK_O          | SPORT 3 Clock B                  |
| 0110110 (0x36)   | SPT3_AFS_O           | SPORT 3 Frame Sync A             |
| 0110111 (0x37)   | SPT3_BFS_O           | SPORT 3 Frame Sync B             |
| 0111000 (0x38)   | PCG0_CLKA_O          | Precision Clock A                |
| 0111001 (0x39)   | PCG0_CLKB_O          | Precision Clock B                |
| 0111010 (0x3A)   | PCG0_FSA_O           | Precision Frame Sync A           |
| 0111011 (0x3B)   | PCG0_FSB_O           | Precision Frame Sync B           |
| 0111100 (0x3C)   | Reserved             | Reserved                         |
| 0111101 (0x3D)   | SRC0_DAT_OP_O        | SRC0 Data Output                 |
| 0111110 (0x3E)   | SRC1_DAT_OP_O        | SRC1 Data Output                 |
| 0111111 (0x3F)   | SRC2_DAT_OP_O        | SRC2 Data Output                 |

Table 33-11: DAI0 Group D - Pin Signal Assignments (Continued)

| Selection Code    | Source Signal      | Description (Source Selection)         |
|-------------------|--------------------|----------------------------------------|
| 1000000 (0x40)    | SRC3_DAT_OP_O      | SRC3 Data Output                       |
| 1000001 (0x41)    | SPDIF0_RX_DAT_O    | SPDIF 0 RX Data Output                 |
| 1000010 (0x42)    | SPDIF0_FS_O        | SPDIF 0 RX Frame Sync Output           |
| 1000011 (0x43)    | SPDIF0_RXCLK_O     | SPDIF 0 RX Clock Output                |
| 1000100 (0x44)    | SPDIF0_RX_TDMCLK_O | SPDIF 0 RX TDMClock Output             |
| 1000101 (0x45)    | SPDIF0_TX_O        | SPDIF 0 TX Biphase Encoded Data Output |
| 1000110 (0x46)    | SPT0_ATDV_O        | SPORT0 Transmit A Data Valid Output    |
| 1000111 (0x47)    | SPT0_BTDV_O        | SPORT0 Transmit B Data Valid Output    |
| 1001000 (0x48)    | SPT1_ATDV_O        | SPORT1 Transmit A Data Valid Output    |
| 1001001 (0x49)    | SPT1_BTDV_O        | SPORT1 Transmit B Data Valid Output    |
| 1001010 (0x4A)    | SPT2_ATDV_O        | SPORT2 Transmit A Data Valid Output    |
| 1001011 (0x4B)    | SPT2_BTDV_O        | SPORT2 Transmit B Data Valid Output    |
| 1001100 (0x4C)    | SPT3_ATDV_O        | SPORT3 Transmit A Data Valid Output    |
| 1001101 (0x4D)    | SPT3_BTDV_O        | SPORT3 Transmit B Data Valid Output    |
| 1001110 (0x4E)    | Reserved           |                                        |
| 1001111 (0x4F)    | Reserved           |                                        |
| 1010000 (0x50)    | PCG0_CRS_CLKC_O    | Precision Clock C (DAI1)               |
| 1010001 (0x51)    | PCG0_CRS_CLKD_O    | Precision Clock D(DAI1)                |
| 1010010 (0x52)    | PCG0_CRS_FSC_O     | Precision Frame Sync C (DAI1)          |
| 1010011 (0x53)    | PCG0_CRS_FSD_O     | Precision Frame Sync D(DAI1)           |
| 1010100 (0x54)    | DAI0_CRS_PB03_O    | Pin Buffer 3 (DAI1)                    |
| 1010101 (0x55)    | DAI0_CRS_PB04_O    | Pin Buffer 4 (DAI1)                    |
| 1010110 - 1111101 | Reserved           |                                        |
| 1111110 (0x7E)    | LOW                | Logic Level Low (0)                    |
| 1111111 (0x7F)    | HIGH               | Logic Level High (1)                   |

## DAI0 Group E - Miscellaneous Source Signals

The group E miscellaneous signal sources are listed in the following table. The group E destinations are configured using Miscellaneous Control Register 0 through Miscellaneous Control Register 1 .

Table 33-12: DAI0 Group E - Miscellaneous Signals

| Selection Code   | Source Signal        | Description (Source Selection)   |
|------------------|----------------------|----------------------------------|
| 00000 (0x0)      | DAI0_PB01_O          | Pin Buffer 1 Output              |
| 00001 (0x1)      | DAI0_PB02_O          | Pin Buffer 2 Output              |
| 00010 (0x2)      | DAI0_PB03_O          | Pin Buffer 3 Output              |
| 00011 (0x3)      | DAI0_PB04_O          | Pin Buffer 4 Output              |
| 00100 (0x4)      | DAI0_PB05_O          | Pin Buffer 5 Output              |
| 00101 (0x5)      | DAI0_PB06_O          | Pin Buffer 6 Output              |
| 00110 (0x6)      | DAI0_PB07_O          | Pin Buffer 7 Output              |
| 00111 (0x7)      | DAI0_PB08_O          | Pin Buffer 8 Output              |
| 01000 (0x8)      | DAI0_PB09_O          | Pin Buffer 9 Output              |
| 01001 (0x9)      | DAI0_PB10_O          | Pin Buffer 10 Output             |
| 01010 (0xA)      | DAI0_PB11_O          | Pin Buffer 11 Output             |
| 01011 (0xB)      | DAI0_PB12_O          | Pin Buffer 12 Output             |
| 01100 (0xC)      | DAI0_PB13_O          | Pin Buffer 13 Output             |
| 01101 (0xD)      | DAI0_PB14_O          | Pin Buffer 14 Output             |
| 01110 (0xE)      | DAI0_PB15_O          | Pin Buffer 15 Output             |
| 01111 (0xF)      | DAI0_PB16_O          | Pin Buffer 16 Output             |
| 10000 (0x10)     | DAI0_PB17_O          | Pin Buffer 17 Output             |
| 10001 (0x11)     | DAI0_PB18_O          | Pin Buffer 18 Output             |
| 10010 (0x12)     | DAI0_PB19_O          | Pin Buffer 19 Output             |
| 10011 (0x13)     | DAI0_PB20_O          | Pin Buffer 20 Output             |
| 10100 (0x14)     | SPT0_AFS_O           | SPORT 0 Frame Sync A             |
| 10101 (0x15)     | SPT0_BFS_O           | SPORT 0 Frame Sync B             |
| 10110 (0x16)     | SPT1_AFS_O           | SPORT 1 Frame Sync A             |
| 10111 (0x17)     | SPT1_BFS_O           | SPORT 1 Frame Sync B             |
| 11000 (0x18)     | SPT2_AFS_O           | SPORT 2 Frame Sync A             |
| 11001 (0x19)     | SPT2_BFS_O           | SPORT 2 Frame Sync B             |
| 11010 (0x1A)     | SPDIF0_TX_BLKSTART_O | SPDIF 0 TX Block Start Output    |
| 11011 (0x1B)     | PCG0_FSA_O           | Precision Frame Sync A           |
| 11100 (0x1C)0    | PCG0_CLKB_O          | Precision Clock B                |
| 11101 (0x1D)     | PCG0_FSB_O           | Precision Frame Sync B           |
| 11110 (0x1E)     | LOW                  | Logic Level Low (0) as a Source  |

Table 33-12: DAI0 Group E - Miscellaneous Signals (Continued)

| Selection Code   | Source Signal   | Description (Source Selection)   |
|------------------|-----------------|----------------------------------|
| 11111 (0x1F)     | HIGH            | Logic Level High (1) as a Source |

## DAI0 Group F - Pin Output Enable Source Signals

The group F pin output enable signal sources are listed in the following table. The group F destinations are configured using Pin Buffer Enable Register 0 through Pin Buffer Enable Register 3 .

Table 33-13: DAI0 Group F - Pin Output Enable

| Selection Code   | Source Signal    | Description (Source Selection)       |
|------------------|------------------|--------------------------------------|
| 000000 (0x0)     | LOW              | Logic Level Low (0)                  |
| 000001 (0x1)     | HIGH             | Logic Level High (1)                 |
| 000010 (0x2)     | DAI0_MISCA0_O    | DAI0 Miscellaneous Control A0 Output |
| 000011 (0x3)     | DAI0_MISCA1_O    | DAI0 Miscellaneous Control A1 Output |
| 000100 (0x4)     | DAI0_MISCA2_O    | DAI0 Miscellaneous Control A2 Output |
| 000101 (0x5)     | DAI0_MISCA3_O    | DAI0 Miscellaneous Control A3 Output |
| 000110 (0x6)     | DAI0_MISCA4_O    | DAI0 Miscellaneous Control A4 Output |
| 000111 (0x7)     | DAI0_MISCA5_O    | DAI0 Miscellaneous Control A5 Output |
| 001000 (0x8)     | SPT0_ACLK_PBEN_O | SPORT 0 Clock A Output Enable        |
| 001001 (0x9)     | SPT0_AFS_PBEN_O  | SPORT 0 Frame Sync A Output Enable   |
| 001010 (0xA)     | SPT0_AD0_PBEN_O  | SPORT 0 Data AD0 Output Enable       |
| 001011 (0xB)     | SPT0_AD1_PBEN_O  | SPORT 0 Data AD1 Output Enable       |
| 001100 (0xC)     | SPT0_BCLK_PBEN_O | SPORT 0 Clock B Output Enable        |
| 001101 (0xD)     | SPT0_BFS_PBEN_O  | SPORT 0 Frame Sync B Output Enable   |
| 001110 (0xE)     | SPT0_BD0_PBEN_O  | SPORT 0 Data BD0 Output Enable       |
| 001111 (0xF)     | SPT0_BD1_PBEN_O  | SPORT 0 Data BD1 Output Enable       |
| 010000 (0x10)    | SPT1_ACLK_PBEN_O | SPORT 1 Clock A Output Enable        |
| 010001 (0x11)    | SPT1_AFS_PBEN_O  | SPORT 1 Frame Sync A Output Enable   |
| 010010 (0x12)    | SPT1_AD0_PBEN_O  | SPORT 1 Data AD0 Output Enable       |
| 010011 (0x13)    | SPT1_AD1_PBEN_O  | SPORT 1 Data AD1 Output Enable       |
| 010100 (0x14)    | SPT1_BCLK_PBEN_O | SPORT 1 Clock B Output Enable        |
| 010101 (0x15)    | SPT1_BFS_PBEN_O  | SPORT 1 Frame Sync B Output Enable   |
| 010110 (0x16)    | SPT1_BD0_PBEN_O  | SPORT 1 Data BD0 Output Enable       |
| 010111 (0x17)    | SPT1_BD1_PBEN_O  | SPORT 1 Data BD1 Output Enable       |

Table 33-13: DAI0 Group F - Pin Output Enable (Continued)

| Selection Code               | Source Signal    | Description (Source Selection)       |
|------------------------------|------------------|--------------------------------------|
| 011000 (0x18)                | SPT2_ACLK_PBEN_O | SPORT 2 Clock A Output Enable        |
| 011001 (0x19)                | SPT2_AFS_PBEN_O  | SPORT 2 Frame Sync A Output Enable   |
| 011010 (0x1A)                | SPT2_AD0_PBEN_O  | SPORT 2 Data AD0 Output Enable       |
| 011011 (0x1B)                | SPT2_AD1_PBEN_O  | SPORT 2 Data AD1 Output Enable       |
| 011100 (0x1C)                | SPT2_BCLK_PBEN_O | SPORT 2 Clock B Output Enable        |
| 011101 (0x1D)                | SPT2_BFS_PBEN_O  | SPORT 2 Frame Sync B Output Enable   |
| 011110 (0x1E)                | SPT2_BD0_PBEN_O  | SPORT 2 Data BD0 Output Enable       |
| 011111 (0x1F)                | SPT2_BD1_PBEN_O  | SPORT 2 Data BD1 Output Enable       |
| 100000 (0x20)                | SPT3_ACLK_PBEN_O | SPORT 3 Clock A Output Enable        |
| 100001 (0x21)                | SPT3_AFS_PBEN_O  | SPORT 3 Frame Sync A Output Enable   |
| 100010 (0x22)                | SPT3_AD0_PBEN_O  | SPORT 3 Data AD0 Output Enable       |
| 100011 (0x23)                | SPT3_AD1_PBEN_O  | SPORT 3 Data AD1 Output Enable       |
| 100100 (0x24)                | SPT3_BCLK_PBEN_O | SPORT 2 Clock B Output Enable        |
| 100101 (0x25)                | SPT3_BFS_PBEN_O  | SPORT 3 Frame Sync B Output Enable   |
| 100110 (0x26)                | SPT3_BD0_PBEN_O  | SPORT 3 Data BD0 Output Enable       |
| 100111 (0x27)                | SPT3_BD1_PBEN_O  | SPORT 3 Data BD1 Output Enable       |
| 101000 (0x28)                | SPT0_ATDV_PBEN_O | SPORT 0 A Transmit Data Valid Output |
| 101001 (0x29)                | SPT0_BTDV_PBEN_O | SPORT 0 B Transmit Data Valid Output |
| 101010 (0x2A)                | SPT1_ATDV_PBEN_O | SPORT 1 A Transmit Data Valid Output |
| 101011 (0x2B)                | SPT1_BTDV_PBEN_O | SPORT 1 B Transmit Data Valid Output |
| 101100 (0x2C)                | SPT2_ATDV_PBEN_O | SPORT 2 A Transmit Data Valid Output |
| 101101 (0x2D)                | SPT2_BTDV_PBEN_O | SPORT 2 B Transmit Data Valid Output |
| 100111 (0x2E)                | SPT3_ATDV_PBEN_O | SPORT 3 A Transmit Data Valid Output |
| 101110 (0x2F)                | SPT3_BTDV_PBEN_O | SPORT 3 B Transmit Data Valid Output |
| 110000 (0x30)-1111111 (0x3F) | Reserved         |                                      |

## DAI1 Group A - Clock Routing Source Signals

The group A clock routing signal sources are listed in the following table. The Group A clock destinations are configured using Clock Routing Control Register 0 through Clock Routing Control Register 5 .

Table 33-14: DAI1 Group A - Clock Routing Signals

| Selection Code   | Source Signal      | Description (Source Selection)   |
|------------------|--------------------|----------------------------------|
| 00000 (0x0)      | DAI1_CRS_PB03_O    | Pin Buffer 3 (DAI0)              |
| 00001 (0x1)      | DAI1_PB02_O        | Pin Buffer 2                     |
| 00010 (0x2)      | DAI1_PB03_O        | Pin Buffer 3                     |
| 00011 (0x3)      | DAI1_PB04_O        | Pin Buffer 4                     |
| 00100 (0x4)      | DAI1_PB05_O        | Pin Buffer 5                     |
| 00101 (0x5)      | DAI1_PB06_O        | Pin Buffer 6                     |
| 00110 (0x6)      | DAI1_PB07_O        | Pin Buffer 7                     |
| 00111 (0x7)      | DAI1_PB08_O        | Pin Buffer 8                     |
| 01000 (0x8)      | DAI1_PB09_O        | Pin Buffer 9                     |
| 01001 (0x9)      | DAI1_PB10_O        | Pin Buffer 10                    |
| 01010 (0xA)      | DAI1_PB11_O        | Pin Buffer 11                    |
| 01011 (0xB)      | DAI1_PB12_O        | Pin Buffer 12                    |
| 01100 (0xC)      | DAI1_PB13_O        | Pin Buffer 13                    |
| 01101 (0xD)      | DAI1_PB14_O        | Pin Buffer 14                    |
| 01110 (0xE)      | DAI1_PB15_O        | Pin Buffer 15                    |
| 01111 (0xF)      | DAI1_PB16_O        | Pin Buffer 16                    |
| 10000 (0x10)     | DAI1_PB17_O        | Pin Buffer 17                    |
| 10001 (0x11)     | DAI1_PB18_O        | Pin Buffer 18                    |
| 10010 (0x12)     | DAI1_PB19_O        | Pin Buffer 19                    |
| 10011 (0x13)     | DAI1_PB20_O        | Pin Buffer 20                    |
| 10100 (0x14)     | SPT4_ACLK_O        | SPORT 4 Clock A                  |
| 10101 (0x15)     | SPT4_BCLK_O        | SPORT 4 Clock B                  |
| 10110 (0x16)     | SPT5_ACLK_O        | SPORT 5 Clock A                  |
| 10111 (0x17)     | SPT5_BCLK_O        | SPORT 5 Clock B                  |
| 11000 (0x18)     | SPT6_ACLK_O        | SPORT 6 Clock A                  |
| 11001 (0x19)     | SPT6_BCLK_O        | SPORT 6 Clock B                  |
| 11010 (0x1A)     | SPDIF1_RX_CLK_O    | SPDIF 1 Receive Clock Output     |
| 11011 (0x1B)     | SPDIF1_RX_TDMCLK_O | SPDIF 1 Receive TDMClock Output  |
| 11100 (0x1C)     | PCG0_CLKC_O        | Precision Clock C Output         |
| 11101 (0x1D)     | PCG0_CLKD_O        | Precision Clock DOutput          |
| 11110 (0x1E)     | LOW                | Logic Level Low (0)              |

Table 33-14: DAI1 Group A - Clock Routing Signals (Continued)

| Selection Code   | Source Signal   | Description (Source Selection)   |
|------------------|-----------------|----------------------------------|
| 11111 (0x1F)     | HIGH            | Logic Level High (1)             |

## DAI1 Group B - Serial Data Source Signals

The group B data sources are listed in the following table. The group B data destinations are configured using Serial Data Routing Control Register 0 through Serial Data Routing Control Register 6 .

Table 33-15: DAI1 Group B - Serial Data Signals

| Selection Code   | Source Signal   | Description (Source Selection)   |
|------------------|-----------------|----------------------------------|
| 000000 (0x0)     | DAI1_PB01_O     | Pin Buffer 1                     |
| 000001 (0x1)     | DAI1_PB02_O     | Pin Buffer 2                     |
| 000010 (0x2)     | DAI1_PB03_O     | Pin Buffer 3                     |
| 000011 (0x3)     | DAI1_PB04_O     | Pin Buffer 4                     |
| 000100 (0x4)     | DAI1_PB05_O     | Pin Buffer 5                     |
| 000101 (0x5)     | DAI1_PB06_O     | Pin Buffer 6                     |
| 000110 (0x6)     | DAI1_PB07_O     | Pin Buffer 7                     |
| 000111 (0x7)     | DAI1_PB08_O     | Pin Buffer 8                     |
| 001000 (0x8)     | DAI1_PB09_O     | Pin Buffer 9                     |
| 001001 (0x9)     | DAI1_PB10_O     | Pin Buffer 10                    |
| 001010 (0xA)     | DAI1_PB11_O     | Pin Buffer 11                    |
| 001011 (0xB)     | DAI1_PB12_O     | Pin Buffer 12                    |
| 001100 (0xC)     | DAI1_PB13_O     | Pin Buffer 13                    |
| 001101 (0xD)     | DAI1_PB14_O     | Pin Buffer 14                    |
| 001110 (0xE)     | DAI1_PB15_O     | Pin Buffer 15                    |
| 001111 (0xF)     | DAI1_PB16_O     | Pin Buffer 16                    |
| 010000 (0x10)    | DAI1_PB17_O     | Pin Buffer 17                    |
| 010001 (0x11)    | DAI1_PB18_O     | Pin Buffer 18                    |
| 010010 (0x12)    | DAI1_PB19_O     | Pin Buffer 19                    |
| 010011 (0x13)    | DAI1_PB20_O     | Pin Buffer 20                    |
| 010100 (0x14)    | SPT4_AD0_O      | SPORT 4 Data AD0                 |
| 010101 (0x15)    | SPT4_AD1_O      | SPORT 4 Data AD1                 |
| 010110 (0x16)    | SPT4_BD0_O      | SPORT 4 Data BD0                 |
| 010111 (0x17)    | SPT4_BD1_O      | SPORT 4 Data BD1                 |

Table 33-15: DAI1 Group B - Serial Data Signals (Continued)

| Selection Code            | Source Signal     | Description (Source Selection)   |
|---------------------------|-------------------|----------------------------------|
| 011000 (0x18)             | SPT5_AD0_O        | SPORT 5 Data AD0                 |
| 011001 (0x19)             | SPT5_AD1_O        | SPORT 5 Data AD1                 |
| 011010 (0x1A)             | SPT5_BD0_O        | SPORT 5 Data BD0                 |
| 011011 (0x1B)             | SPT5_BD1_O        | SPORT 5 Data BD1                 |
| 011100 (0x1C)             | SPT6_AD0_O        | SPORT 6 Data AD0                 |
| 011101 (0x1D)             | SPT6_AD1_O        | SPORT 6 Data AD1                 |
| 011110 (0x1E)             | SPT6_BD0_O        | SPORT 6 Data BD0                 |
| 011111 (0x1F)             | SPT6_BD1_O        | SPORT 6 Data BD1                 |
| 100000 (0x20)             | SRC4_DAT_OP_O     | SRC4 Data Out                    |
| 100001 (0x21)             | SRC5_DAT_OP_O     | SRC5 Data Out                    |
| 100010 (0x22)             | SRC6_DAT_OP_O     | SRC6 Data Out                    |
| 100011 (0x23)             | SRC7_DAT_OP_O     | SRC7 Data Out                    |
| 100100 (0x24)             | SRC4_TDM_IP_O     | SRC4 Data Out                    |
| 100101 (0x25)             | SRC5_TDM_IP_O     | SRC5 Data Out                    |
| 100110 (0x26)             | SRC6_TDM_IP_O     | SRC6 Data Out                    |
| 100111 (0x27)             | SRC7_TDM_IP_O     | SRC7 Data Out                    |
| 101000 (0x28)             | SPDIF1_RX_DAT_O   | SPDIF 1 RX Serial Data Out       |
| 101100(0x2C)              | SPT7_AD0_O        | SPORT 7 Data AD0                 |
| 101101(0x2D)              | SPT7_AD1_O        | SPORT 7 Data AD1                 |
| 101110(0x2E)              | SPT7_BD0_O        | SPORT 7 Data BD0                 |
| 101111(0x2F)              | SPT7_BD1_O        | SPORT 7 Data BD1                 |
| 110000(0x30)              | SPDIF1_TX_O       | SPDIF 1 TX Biphase Stream        |
| 110001(0x31)              | SRC3_CRS_DAT_OP_O | SRC3 data out (DAI0)             |
| 110010(0x32)              | SRC3_CRS_TDM_IP_O | SRC3 data out (DAI0)             |
| 110001(0x33)-111101(0x3D) | Reserved          |                                  |
| 111110 (0x3E)             | LOW               | Logic Level Low (0)              |
| 111111 (0x3F)             | HIGH              | Logic Level High (1)             |

## DAI1 Group C - Frame Sync Source Signals

The group C frame sync signals are listed in the following table. The frame sync destinations are configured using Frame Sync Routing Control Register 0 through Frame Sync Routing Control Register 4 .

Table 33-16: DAI1 Group C - Frame Sync Signals

| Selection Code   | Source Signal   | Description (Source Selection)   |
|------------------|-----------------|----------------------------------|
| 00000 (0x0)      | DAI1_CRS_PB04_O | Pin Buffer 4 (DAI0)              |
| 00001 (0x1)      | DAI1_PB02_O     | Pin Buffer 2                     |
| 00010 (0x2)      | DAI1_PB03_O     | Pin Buffer 3                     |
| 00011 (0x3)      | DAI1_PB04_O     | Pin Buffer 4                     |
| 00100 (0x4)      | DAI1_PB05_O     | Pin Buffer 5                     |
| 00101 (0x5)      | DAI1_PB06_O     | Pin Buffer 6                     |
| 00110 (0x6)      | DAI1_PB07_O     | Pin Buffer 7                     |
| 00111 (0x7)      | DAI1_PB08_O     | Pin Buffer 8                     |
| 01000 (0x8)      | DAI1_PB09_O     | Pin Buffer 9                     |
| 01001 (0x9)      | DAI1_PB10_O     | Pin Buffer 10                    |
| 01010 (0xA)      | DAI1_PB11_O     | Pin Buffer 11                    |
| 01011 (0xB)      | DAI1_PB12_O     | Pin Buffer 12                    |
| 01100 (0xC)      | DAI1_PB13_O     | Pin Buffer 13                    |
| 01101 (0xD)      | DAI1_PB14_O     | Pin Buffer 14                    |
| 01110 (0xE)      | DAI1_PB15_O     | Pin Buffer 15                    |
| 01111 (0xF)      | DAI1_PB16_O     | Pin Buffer 16                    |
| 10000 (0x10)     | DAI1_PB17_O     | Pin Buffer 17                    |
| 10001 (0x11)     | DAI1_PB18_O     | Pin Buffer 18                    |
| 10010 (0x12)     | DAI1_PB19_O     | Pin Buffer 19                    |
| 10011 (0x13)     | DAI1_PB20_O     | Pin Buffer 20                    |
| 10100 (0x14)     | SPT4_AFS_O      | SPORT 4 Frame Sync A             |
| 10101 (0x15)     | SPT4_BFS_O      | SPORT 4 Frame Sync B             |
| 10110 (0x16)     | SPT5_AFS_O      | SPORT 5 Frame Sync A             |
| 10111 (0x17)     | SPT5_BFS_O      | SPORT 5 Frame Sync B             |
| 11000 (0x18)     | SPT6_AFS_O      | SPORT 6 Frame Sync A             |
| 11001 (0x19)     | SPT6_BFS_O      | SPORT 6 Frame Sync B             |
| 11010 (0x1A)     | SPDIF1_FS_O     | SPDIF 1 RX Frame Sync Output     |
| 11011 (0x1B)     | Reserved        |                                  |
| 11100 (0x1C)     | PCG0_FSC_O      | Precision Frame Sync C Output    |
| 11101 (0x1D)     | PCG0_FSD_O      | Precision Frame Sync DOutput     |
| 11110 (0x1E)     | LOW             | Logic Level Low (0)              |

Table 33-16: DAI1 Group C - Frame Sync Signals (Continued)

| Selection Code   | Source Signal   | Description (Source Selection)   |
|------------------|-----------------|----------------------------------|
| 11111 (0x1F)     | HIGH            | Logic Level High (1)             |

## DAI1 Group D - Pin Signal Assignment Source Signals

The group D pin signal assignments are listed in the following table. The group D destinations are configured using Pin Buffer Assignment Register 0 through Pin Buffer Assignment Register 4 .

Table 33-17: DAI1 Group D - Pin Signal Assignments

| Selection Code   | Source Signal   | Description (Source Selection)   |
|------------------|-----------------|----------------------------------|
| 0000000 (0x0)    | DAI1_PB01_O     | Pin Buffer 1                     |
| 0000001 (0x1)    | DAI1_PB02_O     | Pin Buffer 2                     |
| 0000010 (0x2)    | DAI1_PB03_O     | Pin Buffer 3                     |
| 0000011 (0x3)    | DAI1_PB04_O     | Pin Buffer 4                     |
| 0000100 (0x4)    | DAI1_PB05_O     | Pin Buffer 5                     |
| 0000101 (0x5)    | DAI1_PB06_O     | Pin Buffer 6                     |
| 0000110 (0x6)    | DAI1_PB07_O     | Pin Buffer 7                     |
| 0000111 (0x7)    | DAI1_PB08_O     | Pin Buffer 8                     |
| 0001000 (0x8)    | DAI1_PB09_O     | Pin Buffer 9                     |
| 0001001 (0x9)    | DAI1_PB10_O     | Pin Buffer 10                    |
| 0001010 (0xA)    | DAI1_PB11_O     | Pin Buffer 11                    |
| 0001011 (0xB)    | DAI1_PB12_O     | Pin Buffer 12                    |
| 0001100 (0xC)    | DAI1_PB13_O     | Pin Buffer 13                    |
| 0001101 (0xD)    | DAI1_PB14_O     | Pin Buffer 14                    |
| 0001110 (0xE)    | DAI1_PB15_O     | Pin Buffer 15                    |
| 0001111 (0xF)    | DAI1_PB16_O     | Pin Buffer 16                    |
| 0010000 (0x10)   | DAI1_PB17_O     | Pin Buffer 17                    |
| 0010001 (0x11)   | DAI1_PB18_O     | Pin Buffer 18                    |
| 0010010 (0x12)   | DAI1_PB19_O     | Pin Buffer 19                    |
| 0010011 (0x13)   | DAI1_PB20_O     | Pin Buffer 20                    |
| 0010100 (0x14)   | SPT4_AD0_O      | SPORT 4 Data AD0                 |
| 0010101 (0x15)   | SPT4_AD1_O      | SPORT 4 Data AD1                 |
| 0010110 (0x16)   | SPT4_BD0_O      | SPORT 4 Data BD0                 |
| 0010111 (0x17)   | SPT4_BD1_O      | SPORT 4 Data BD1                 |

Table 33-17: DAI1 Group D - Pin Signal Assignments (Continued)

| Selection Code   | Source Signal        | Description (Source Selection)   |
|------------------|----------------------|----------------------------------|
| 0011000 (0x18)   | SPT5_AD0_O           | SPORT 5 Data AD0                 |
| 0011001 (0x19)   | SPT5_AD1_O           | SPORT 5 Data AD1                 |
| 0011010 (0x1A)   | SPT5_BD0_O           | SPORT 5 Data BD0                 |
| 0011011 (0x1B)   | SPT5_BD1_O           | SPORT 5 Data BD1                 |
| 0011100 (0x1C)   | SPT6_AD0_O           | SPORT 6 Data AD0                 |
| 0011101 (0x1D)   | SPT6_AD1_O           | SPORT 6 Data AD1                 |
| 0011110 (0x1E)   | SPT6_BD0_O           | SPORT 6 Data BD0                 |
| 0011111 (0x1F)   | SPT6_BD1_O           | SPORT 6 Data BD1                 |
| 0100000 (0x20)   | SPT4_ACLK_O          | SPORT 4 Clock A                  |
| 0100001 (0x21)   | SPT4_BCLK_O          | SPORT4 Clock B                   |
| 0100010 (0x22)   | SPT5_ACLK_O          | SPORT 5 Clock A                  |
| 0100011 (0x23)   | SPT5_BCLK_O          | SPORT 5 Clock B                  |
| 0100100 (0x24)   | SPT6_ACLK_O          | SPORT 6 Clock A                  |
| 0100101 (0x25)   | SPT6_BCLK_O          | SPORT 6 Clock B                  |
| 0100110 (0x26)   | SPT4_AFS_O           | SPORT 4 Frame Sync A             |
| 0100111 (0x27)   | SPT4_BFS_O           | SPORT 4 Frame Sync B             |
| 0101000 (0x28)   | SPT5_AFS_O           | SPORT 5 Frame Sync A             |
| 0101001 (0x29)   | SPT5_BFS_O           | SPORT 5 Frame Sync B             |
| 0101010 (0x2A)   | SPT6_AFS_O           | SPORT 5 Frame Sync A             |
| 0101011 (0x2B)   | SPT6_BFS_O           | SPORT 5 Frame Sync B             |
| 0101100 (0x2C)   | SPT7_AD0_O           | SPORT 7 Data AD0                 |
| 0101101 (0x2D)   | SPT7_AD1_O           | SPORT 7 Data AD1                 |
| 0101110 (0x2E)   | SPT7_BD0_O           | SPORT 7 Data BD0                 |
| 0101111 (0x2F)   | SPT7_BD1_O           | SPORT 7 Data BD1                 |
| 0110000 (0x30)   | MLB0_CLKOUT          | MLB PLL clock output             |
| 0110001 (0x31)   | SPDIF1_TX_BLKSTART_O | SPDIF 1 TX Block Start Output    |
| 0110100 (0x34)   | SPT7_ACLK_O          | SPORT 7 Clock A                  |
| 0110101 (0x35)   | SPT7_BCLK_O          | SPORT 7 Clock B                  |
| 0110110 (0x36)   | SPT7_AFS_O           | SPORT 7 Frame Sync A             |
| 0110111 (0x37)   | SPT7_BFS_O           | SPORT 7 Frame Sync B             |
| 0111000 (0x38)   | PCG0_CLKC_O          | Precision Clock C                |

Table 33-17: DAI1 Group D - Pin Signal Assignments (Continued)

| Selection Code    | Source Signal      | Description (Source Selection)         |
|-------------------|--------------------|----------------------------------------|
| 0111001 (0x39)    | PCG0_CLKD_O        | Precision ClockD                       |
| 0111010 (0x3A)    | PCG0_FSC_O         | Precision Frame Sync C                 |
| 0111011 (0x3B)    | PCG0_FSD_O         | Precision Frame SyncD                  |
| 0111100 (0x3C)    | Reserved           |                                        |
| 0111101 (0x3D)    | SRC4_DAT_OP_O      | SRC4 Data Output                       |
| 0111110 (0x3E)    | SRC5_DAT_OP_O      | SRC5 Data Output                       |
| 0111111 (0x3F)    | SRC6_DAT_OP_O      | SRC6 Data Output                       |
| 1000000 (0x40)    | SRC7_DAT_OP_O      | SRC7 Data Output                       |
| 1000001 (0x41)    | SPDIF1_RX_DAT_O    | SPDIF 1 RX Data Output                 |
| 1000010 (0x42)    | SPDIF1_FS_O        | SPDIF 1 RX Frame Sync Output           |
| 1000011 (0x43)    | SPDIF1_RXCLK_O     | SPDIF 1 RX Clock Output                |
| 1000100 (0x44)    | SPDIF1_RX_TDMCLK_O | SPDIF 1 RX TDMClock Output             |
| 1000101 (0x45)    | SPDIF1_TX_O        | SPDIF 1 TX Biphase Encoded Data Output |
| 1000110 (0x46)    | SPT4_ATDV_O        | SPORT4 Transmit A Data Valid Output    |
| 1000111 (0x47)    | SPT4_BTDV_O        | SPORT4 Transmit B Data Valid Output    |
| 1001000 (0x48)    | SPT5_ATDV_O        | SPORT5 Transmit A Data Valid Output    |
| 1001001 (0x49)    | SPT5_BTDV_O        | SPORT5 Transmit B Data Valid Output    |
| 1001010 (0x4A)    | SPT6_ATDV_O        | SPORT6 Transmit A Data Valid Output    |
| 1001011 (0x4B)    | SPT6_BTDV_O        | SPORT6 Transmit B Data Valid Output    |
| 1001100 (0x4C)    | SPT7_ATDV_O        | SPORT7 Transmit A Data Valid Output    |
| 1001101 (0x4D)    | SPT7_BTDV_O        | SPORT7 Transmit B Data Valid Output    |
| 1001110 (0x4E)    | Reserved           |                                        |
| 1001111 (0x4F)    | Reserved           |                                        |
| 1010000 (0x50)    | PCG0_CRS_CLKA_O    | Precision Clock A (DAI0)               |
| 1010001 (0x51)    | PCG0_CRS_CLKB_O    | Precision Clock B (DAI0)               |
| 1010010 (0x52)    | PCG0_CRS_FSA_O     | Precision Frame Sync A (DAI0)          |
| 1010011 (0x53)    | PCG0_CRS_FSB_O     | Precision Frame Sync B (DAI0)          |
| 1010100 (0x54)    | DAI1_CRS_PB03_O    | Pin Buffer 3 (DAI0)                    |
| 1010101 (0x55)    | DAI1_CRS_PB04_O    | Pin Buffer 4 (DAI0)                    |
| 1010110 - 1111101 | Reserved           |                                        |
| 1111110 (0x7E)    | LOW                | Logic Level Low (0)                    |

Table 33-17: DAI1 Group D - Pin Signal Assignments (Continued)

| Selection Code   | Source Signal   | Description (Source Selection)   |
|------------------|-----------------|----------------------------------|
| 1111111 (0x7F)   | HIGH            | Logic Level High (1)             |

## DAI1 Group E - Miscellaneous Source Signals

The group E miscellaneous signals are listed in the following table. The group E destinations are configured using Miscellaneous Control Register 0 and Miscellaneous Control Register 1 .

Table 33-18: DAI1 Group E - Miscellaneous Signals

| Selection Code   | Source Signal   | Description (Source Selection)   |
|------------------|-----------------|----------------------------------|
| 00000 (0x0)      | DAI1_PB01_O     | Pin Buffer 1 Output              |
| 00001 (0x1)      | DAI1_PB02_O     | Pin Buffer 2 Output              |
| 00010 (0x2)      | DAI1_PB03_O     | Pin Buffer 3 Output              |
| 00011 (0x3)      | DAI1_PB04_O     | Pin Buffer 4 Output              |
| 00100 (0x4)      | DAI1_PB05_O     | Pin Buffer 5 Output              |
| 00101 (0x5)      | DAI1_PB06_O     | Pin Buffer 6 Output              |
| 00110 (0x6)      | DAI1_PB07_O     | Pin Buffer 7 Output              |
| 00111 (0x7)      | DAI1_PB08_O     | Pin Buffer 8 Output              |
| 01000 (0x8)      | DAI1_PB09_O     | Pin Buffer 9 Output              |
| 01001 (0x9)      | DAI1_PB10_O     | Pin Buffer 10 Output             |
| 01010 (0xA)      | DAI1_PB11_O     | Pin Buffer 11 Output             |
| 01011 (0xB)      | DAI1_PB12_O     | Pin Buffer 12 Output             |
| 01100 (0xC)      | DAI1_PB13_O     | Pin Buffer 13 Output             |
| 01101 (0xD)      | DAI1_PB14_O     | Pin Buffer 14 Output             |
| 01110 (0xE)      | DAI1_PB15_O     | Pin Buffer 15 Output             |
| 01111 (0xF)      | DAI1_PB16_O     | Pin Buffer 16 Output             |
| 10000 (0x10)     | DAI1_PB17_O     | Pin Buffer 17 Output             |
| 10001 (0x11)     | DAI1_PB18_O     | Pin Buffer 18 Output             |
| 10010 (0x12)     | DAI1_PB19_O     | Pin Buffer 19 Output             |
| 10011 (0x13)     | DAI1_PB20_O     | Pin Buffer 20 Output             |
| 10100 (0x14)     | SPT4_AFS_O      | SPORT 4 Frame Sync A             |
| 10101 (0x15)     | SPT4_BFS_O      | SPORT 4 Frame Sync B             |
| 10110 (0x16)     | SPT5_AFS_O      | SPORT 5 Frame Sync A             |
| 10111 (0x17)     | SPT5_BFS_O      | SPORT 5 Frame Sync B             |

Table 33-18: DAI1 Group E - Miscellaneous Signals (Continued)

| Selection Code   | Source Signal        | Description (Source Selection)   |
|------------------|----------------------|----------------------------------|
| 11000 (0x18)     | SPT6_AFS_O           | SPORT 6 Frame Sync A             |
| 11001 (0x19)     | SPT6_BFS_O           | SPORT 6 Frame Sync B             |
| 11010 (0x1A)     | SPDIF1_TX_BLKSTART_O | SPDIF 1 TX Block Start Output    |
| 11011 (0x1B)     | PCG0_FSC_O           | Precision Frame Sync C           |
| 11100 (0x1C)     | PCG0_CLKD_O          | Precision ClockD                 |
| 11101 (0x1D)     | PCG0_FSD_O           | Precision Frame SyncD            |
| 11110 (0x1E)     | LOW                  | Logic Level Low (0) as a Source  |
| 11111 (0x1F)     | HIGH                 | Logic Level High (1) as a Source |

## DAI1 Group F - Pin Output Enable Source Signals

The group F pin output enable signals are listed in the following table. The group F destinations are configured using Pin Buffer Enable Register 0 through Pin Buffer Enable Register 3 .

Table 33-19: DAI1 Group F - Pin Output Enable

| Selection Code   | Source Signal    | Description (Source Selection)       |
|------------------|------------------|--------------------------------------|
| 000000 (0x0)     | LOW              | Logic Level Low (0)                  |
| 000001 (0x1)     | HIGH             | Logic Level High (1)                 |
| 000010 (0x2)     | DAI1_MISCA0_O    | DAI1 Miscellaneous Control A0 Output |
| 000011 (0x3)     | DAI1_MISCA1_O    | DAI1 Miscellaneous Control A1 Output |
| 000100 (0x4)     | DAI1_MISCA2_O    | DAI1 Miscellaneous Control A2 Output |
| 000101 (0x5)     | DAI1_MISCA3_O    | DAI1 Miscellaneous Control A3 Output |
| 000110 (0x6)     | DAI1_MISCA4_O    | DAI1 Miscellaneous Control A4 Output |
| 000111 (0x7)     | DAI1_MISCA5_O    | DAI1 Miscellaneous Control A5 Output |
| 001000 (0x8)     | SPT4_ACLK_PBEN_O | SPORT 4 Clock A Output Enable        |
| 001001 (0x9)     | SPT4_AFS_PBEN_O  | SPORT 4 Frame Sync A Output Enable   |
| 001010 (0xA)     | SPT4_AD0_PBEN_O  | SPORT 4 Data AD0 Output Enable       |
| 001011 (0xB)     | SPT4_AD1_PBEN_O  | SPORT 4 Data AD1 Output Enable       |
| 001100 (0xC)     | SPT4_BCLK_PBEN_O | SPORT 4 Clock B Output Enable        |
| 001101 (0xD)     | SPT4_BFS_PBEN_O  | SPORT 4 Frame Sync B Output Enable   |
| 001110 (0xE)     | SPT4_BD0_PBEN_O  | SPORT 4 Data BD0 Output Enable       |
| 001111 (0xF)     | SPT4_BD1_PBEN_O  | SPORT 4 Data BD1 Output Enable       |
| 010000 (0x10)    | SPT5_ACLK_PBEN_O | SPORT 5 Clock A Output Enable        |

Table 33-19: DAI1 Group F - Pin Output Enable (Continued)

| Selection Code   | Source Signal    | Description (Source Selection)       |
|------------------|------------------|--------------------------------------|
| 010001 (0x11)    | SPT5_AFS_PBEN_O  | SPORT 5 Frame Sync A Output Enable   |
| 010010 (0x12)    | SPT5_AD0_PBEN_O  | SPORT 5 Data AD0 Output Enable       |
| 010011 (0x13)    | SPT5_AD1_PBEN_O  | SPORT 5 Data AD1 Output Enable       |
| 010100 (0x14)    | SPT5_BCLK_PBEN_O | SPORT 5 Clock B Output Enable        |
| 010101 (0x15)    | SPT5_BFS_PBEN_O  | SPORT 5 Frame Sync B Output Enable   |
| 010110 (0x16)    | SPT5_BD0_PBEN_O  | SPORT 5 Data BD0 Output Enable       |
| 010111 (0x17)    | SPT5_BD1_PBEN_O  | SPORT 5 Data BD1 Output Enable       |
| 011000 (0x18)    | SPT6_ACLK_PBEN_O | SPORT 6 Clock A Output Enable        |
| 011001 (0x19)    | SPT6_AFS_PBEN_O  | SPORT 6 Frame Sync A Output Enable   |
| 011010 (0x1A)    | SPT6_AD0_PBEN_O  | SPORT 6 Data AD0 Output Enable       |
| 011011 (0x1B)    | SPT6_AD1_PBEN_O  | SPORT 6 Data AD1 Output Enable       |
| 011100 (0x1C)    | SPT6_BCLK_PBEN_O | SPORT 6 Clock B Output Enable        |
| 011101 (0x1D)    | SPT6_BFS_PBEN_O  | SPORT 6 Frame Sync B Output Enable   |
| 011110 (0x1E)    | SPT6_BD0_PBEN_O  | SPORT 6 Data BD0 Output Enable       |
| 011111 (0x1F)    | SPT6_BD1_PBEN_O  | SPORT 6 Data BD1 Output Enable       |
| 100000 (0x20)    | SPT7_ACLK_PBEN_O | SPORT 7 Clock A Output Enable        |
| 100001 (0x21)    | SPT7_AFS_PBEN_O  | SPORT 7 Frame Sync A Output Enable   |
| 100010 (0x22)    | SPT7_AD0_PBEN_O  | SPORT 7 Data AD0 Output Enable       |
| 100011 (0x23)    | SPT7_AD1_PBEN_O  | SPORT 7 Data AD1 Output Enable       |
| 100100 (0x24)    | SPT7_BCLK_PBEN_O | SPORT 6 Clock B Output Enable        |
| 100101 (0x25)    | SPT7_BFS_PBEN_O  | SPORT 7 Frame Sync B Output Enable   |
| 100110 (0x26)    | SPT7_BD0_PBEN_O  | SPORT 7 Data BD0 Output Enable       |
| 100111 (0x27)    | SPT7_BD1_PBEN_O  | SPORT 7 Data BD1 Output Enable       |
| 101000 (0x28)    | SPT4_ATDV_PBEN_O | SPORT 4 A Transmit Data Valid Output |
| 101001 (0x29)    | SPT4_BTDV_PBEN_O | SPORT 4 B Transmit Data Valid Output |
| 101010 (0x2A)    | SPT5_ATDV_PBEN_O | SPORT 5 A Transmit Data Valid Output |
| 101011 (0x2B)    | SPT5_BTDV_PBEN_O | SPORT 5 B Transmit Data Valid Output |
| 101100 (0x2C)    | SPT6_ATDV_PBEN_O | SPORT 6 A Transmit Data Valid Output |
| 101101 (0x2D)    | SPT6_BTDV_PBEN_O | SPORT 6 B Transmit Data Valid Output |
| 100111 (0x2E)    | SPT7_ATDV_PBEN_O | SPORT 7 A Transmit Data Valid Output |
| 101110 (0x2F)    | SPT7_BTDV_PBEN_O | SPORT 7 B Transmit Data Valid Output |

Table 33-19: DAI1 Group F - Pin Output Enable (Continued)

| Selection Code               | Source Signal   | Description (Source Selection)   |
|------------------------------|-----------------|----------------------------------|
| 110000 (0x30)-1111111 (0x3F) | Reserved        |                                  |

## DAI Destination Registers Overview

The tables in DAI Routing Capabilities provide high level descriptions that illustrate source (output) connections to destinations (inputs) depending on the routing groups A through F . This sections lists the various input fields (INx) which are described in detail in the ADSP-SC58x DAI Register Descriptions section.

Table 33-20: Clock Destination Registers (Group A)

| DAI Register   | Bit Field Name   | DAI0 Mapping          | DAI1 Mapping          | Description                       |
|----------------|------------------|-----------------------|-----------------------|-----------------------------------|
| DAI_CLK0       | IN0              | SPT0_ACLK_I           | SPT4_ACLK_I           | SPORT0A or SPORT4A Clock          |
| DAI_CLK0       | IN1              | SPT0_BCLK_I           | SPT4_BCLK_I           | SPORT0B or SPORT4B Clock          |
| DAI_CLK0       | IN2              | SPT1_ACLK_I           | SPT5_ACLK_I           | SPORT1A or SPORT5A Clock          |
| DAI_CLK0       | IN3              | SPT1_BCLK_I           | SPT5_BCLK_I           | SPORT1B or SPORT5B Clock          |
| DAI_CLK0       | IN4              | SPT2_ACLK_I           | SPT6_ACLK_I           | SPORT2A or SPORT6A Clock          |
| DAI_CLK0       | IN5              | SPT2_BCLK_I           | SPT6_BCLK_I           | SPORT2B or SPORT6B Clock          |
| DAI_CLK1       | IN0              | SRC0_CLK_IP_I         | SRC4_CLK_IP_I         | SRC0 or SRC4 Clock Input          |
| DAI_CLK1       | IN1              | SRC0_CLK_OP_I         | SRC4_CLK_OP_I         | SRC0 or SRC4 Clock Output         |
| DAI_CLK1       | IN2              | SRC1_CLK_IP_I         | SRC5_CLK_IP_I         | SRC1 or SRC5 Clock Input          |
| DAI_CLK1       | IN3              | SRC1_CLK_OP_I         | SRC5_CLK_OP_I         | SRC1 or SRC5 Clock Output         |
| DAI_CLK1       | IN4              | SRC2_CLK_IP_I         | SRC6_CLK_IP_I         | SRC2 or SRC6 Clock Input          |
| DAI_CLK1       | IN5              | SRC2_CLK_OP_I         | SRC6_CLK_OP_I         | SRC2 or SRC6 Clock Output         |
| DAI_CLK2       | IN0              | SRC3_CLK_IP_I         | SRC7_CLK_IP_I         | SRC3 or SRC7 Clock Input          |
| DAI_CLK2       | IN1              | SRC3_CLK_OP_I         | SRC7_CLK_OP_I         | SRC3 or SRC7 Clock Output         |
| DAI_CLK2       | IN2              | SPDIF0_TX_CLK_I       | SPDIF1_TX_CLK_I       | SPDIF0 or SPDIF1 TX Clock         |
| DAI_CLK3       | IN5              | SPDIF0_TX_HFCLK_I     | SPDIF1_TX_HFCLK_I     | SPDIF0 or SPDIF1 TX HF Clock      |
| DAI_CLK4       | IN0              | PCG0_EXTCLKA_I        | PCG0_EXTCLKC_I        | PCG0 External Clock A or C        |
| DAI_CLK4       | IN1              | PCG0_EXTCLKB_I        | PCG0_EXTCLKD_I        | PCG0 External Clock B orD         |
| DAI_CLK4       | IN3              | SPDIF0_TX_EXT_SYN C_I | SPDIF1_TX_EXT_SYN C_I | SPDIF0 or SPDIF1 TX External Sync |
| DAI_CLK4       | IN4              | PCG0_SYNC_CLKA_I      | PCG0_SYNC_CLKC_I      | PCG0 Sync Clock A or C            |
| DAI_CLK4       | IN5              | PCG0_SYNC_CLKB_I      | PCG0_SYNC_CLKD_I      | PCG0 Sync Clock B orD             |

Table 33-20: Clock Destination Registers (Group A) (Continued)

| DAI Register   | Bit Field Name   | DAI0 Mapping   | DAI1 Mapping   | Description               |
|----------------|------------------|----------------|----------------|---------------------------|
| DAI_CLK5       | IN0              | SPT3_ACLK_I    | SPT7_ACLK_I    | SPORT3A or SPORT 7A Clock |
|                | IN1              | SPT3_BCLK_I    | SPT7_BCLK_I    | SPORT3B or SPORT 7B Clock |

Table 33-21: Data Destination Registers (Group B)

| DAI Regis- ter   | Bit Field Name   | DAI0 Mapping   | DAI1 Mapping   | Description                       |
|------------------|------------------|----------------|----------------|-----------------------------------|
| DAI_DAT0         | IN0              | SPT0_AD0_I     | SPT4_AD0_I     | SPORT0A or SPORT4A Primary Data   |
| DAI_DAT0         | IN1              | SPT0_AD1_I     | SPT4_AD1_I     | SPORT0A or SPORT4A Secondary Data |
| DAI_DAT0         | IN2              | SPT0_BD0_I     | SPT4_BD0_I     | SPORT0B or SPORT4B Primary Data   |
| DAI_DAT0         | IN3              | SPT0_BD1_I     | SPT4_BD1_I     | SPORT0B or SPORT4B Secondary Data |
| DAI_DAT0         | IN4              | SPT1_AD0_I     | SPT5_AD0_I     | SPORT1A or SPORT5A Primary Data   |
| DAI_DAT1         | IN0              | SPT1_AD1_I     | SPT5_AD1_I     | SPORT1A or SPORT5A Secondary Data |
| DAI_DAT1         | IN1              | SPT1_BD0_I     | SPT5_BD0_I     | SPORT1B or SPORT5B Primary Data   |
| DAI_DAT1         | IN2              | SPT1_BD1_I     | SPT5_BD1_I     | SPORT1B or SPORT5B Secondary Data |
| DAI_DAT1         | IN3              | SPT2_AD0_I     | SPT6_AD0_I     | SPORT2A or SPORT6A Primary Data   |
| DAI_DAT1         | IN4              | SPT2_AD1_I     | SPT6_AD1_I     | SPORT2A or SPORT6A Secondary Data |
| DAI_DAT2         | IN0              | SPT2_BD0_I     | SPT6_BD0_I     | SPORT2B or SPORT6B Primary Data   |
| DAI_DAT2         | IN1              | SPT2_BD1_I     | SPT6_BD1_I     | SPORT2B or SPORT6B Secondary Data |
| DAI_DAT2         | IN2              | SRC0_DAT_IP_I  | SRC4_DAT_IP_I  | SRC0 or SRC4 Data                 |
| DAI_DAT2         | IN3              | SRC1_DAT_IP_I  | SRC5_DAT_IP_I  | SRC1 or SRC5 Data                 |
| DAI_DAT2         | IN4              | SRC2_DAT_IP_I  | SRC6_DAT_IP_I  | SRC2 or SRC6 Data                 |

Table 33-21: Data Destination Registers (Group B) (Continued)

| DAI Regis- ter   | Bit Field Name   | DAI0 Mapping      | DAI1 Mapping      | Description                              |
|------------------|------------------|-------------------|-------------------|------------------------------------------|
| DAI_DAT3         | IN0              | SRC3_DAT_IP_I     | SRC7_DAT_IP_I     | SRC3 or SRC7 Data                        |
| DAI_DAT3         | IN1              | SRC0_DAT_TDM_OP_I | SRC4_DAT_TDM_OP_I | SRC0 or SRC4 TDMoutput port              |
| DAI_DAT3         | IN2              | SRC1_DAT_TDM_OP_I | SRC5_DAT_TDM_OP_I | SRC1 or SRC5 TDMOutput port Da- ta       |
| DAI_DAT3         | IN3              | SRC2_DAT_TDM_OP_I | SRC6_DAT_TDM_OP_I | SRC2 or SRC6 TDMOutput port Da- ta       |
| DAI_DAT3         | IN4              | SRC3_DAT_TDM_OP_I | SRC7_DAT_TDM_OP_I | SRC3 or SRC7 TDMOutput port Da- ta       |
| DAI_DAT4         | IN0              | SPDIF0_TX_DAT_I   | SPDIF1_TX_DAT_I   | SPDIF0 or SPDIF1 serial transmitter Data |
| DAI_DAT5         | IN4              | SPDIF0_RX_I       | SPDIF1_RX_I       | SPDIF0 or SPDIF1 receiver bi-phase Data  |
| DAI_DAT6         | IN0              | SPT3_AD0_I        | SPT7_AD0_I        | SPORT3A or SPORT7A Primary Data          |
| DAI_DAT6         | IN1              | SPT3_AD1_I        | SPT7_AD1_I        | SPORT3A or SPORT7A Secondary Data        |
| DAI_DAT6         | IN2              | SPT3_BD0_I        | SPT7_BD0_I        | SPORT3B or SPORT7B Primary Data          |
| DAI_DAT6         | IN3              | SPT3_BD1_I        | SPT7_BD1_I        | SPORT3B or SPORT7B Secondary Data        |
| DAI_DAT6         | IN4              | Reserved          | Reserved          |                                          |

Table 33-22: Frame Sync Destination Registers (Group C)

| DAI Register   | Bit Field Name   | DAI0 Mapping   | DAI1 Mapping   | Description                    |
|----------------|------------------|----------------|----------------|--------------------------------|
| DAI_FS0        | IN0              | SPT0_AFS_I     | SPT4_AFS_I     | SPORT0A or SPORT4A Frame Sync  |
| DAI_FS0        | IN1              | SPT0_BFS_I     | SPT4_BFS_I     | SPORT0B or SPORT4B Frame Sync  |
| DAI_FS0        | IN2              | SPT1_AFS_I     | SPT5_AFS_I     | SPORT1A or SPORT5A Frame Sync  |
| DAI_FS0        | IN3              | SPT1_BFS_I     | SPT5_BFS_I     | SPORT1B or SPORT5B Frame Sync  |
| DAI_FS0        | IN4              | SPT2_AFS_I     | SPT6_AFS_I     | SPORT2A or SPORT6A Frame Sync  |
| DAI_FS0        | IN5              | SPT2_BFS_I     | SPT6_BFS_I     | SPORT2B or SPORT6B Frame Sync  |
| DAI_FS1        | IN0              | SRC0_FS_IP_I   | SRC4_FS_IP_I   | SRC0 or SRC4 Frame Sync Input  |
| DAI_FS1        | IN1              | SRC0_FS_OP_I   | SRC4_FS_OP_I   | SRC0 or SRC4 Frame Sync Output |
| DAI_FS1        | IN2              | SRC1_FS_IP_I   | SRC5_FS_IP_I   | SRC1 or SRC5 Frame Sync Input  |
| DAI_FS1        | IN3              | SRC1_FS_OP_I   | SRC5_FS_OP_I   | SRC1 or SRC5 Frame Sync Output |
| DAI_FS1        | IN4              | SRC2_FS_IP_I   | SRC6_FS_IP_I   | SRC2 or SRC6 Frame Sync Input  |

Table 33-22: Frame Sync Destination Registers (Group C) (Continued)

| DAI Register   | Bit Field Name   | DAI0 Mapping   | DAI1 Mapping   | Description                          |
|----------------|------------------|----------------|----------------|--------------------------------------|
|                | IN5              | SRC2_FS_OP_I   | SRC6_FS_OP_I   | SRC2 or SRC6 Frame Sync Output       |
| DAI_FS2        | IN0              | SRC3_FS_IP_I   | SRC7_FS_IP_I   | SRC3 or SRC7 Frame Sync Input        |
|                | IN1              | SRC3_FS_OP_I   | SRC7_FS_OP_I   | SRC3 or SRC7 Frame Sync Output       |
|                | IN2              | SPDIF0_TX_FS_I | SPDIF1_TX_FS_I | SPDIF0 or SPDIF1 Transmit Frame Sync |
| DAI_FS4        | IN0              | SPT3_AFS_I     | SPT7_AFS_I     | SPORT3A or SPORT7A Frame Sync        |
|                | IN1              | SPT3_BFS_I     | SPT7_BFS_I     | SPORT3B or SPORT7B Frame Sync        |

Table 33-23: Pin Buffer Assignment Destination Registers (Group D)

| DAI Register   | Bit Field Name   | DAI0 Mapping    | DAI1 Mapping    | Description            |
|----------------|------------------|-----------------|-----------------|------------------------|
| DAI_PIN0       | IN0              | DAI0_PB01_I     | DAI1_PB01_I     | Pin Buffer 1           |
| DAI_PIN0       | IN1              | DAI0_PB02_I     | DAI1_PB02_I     | Pin Buffer 2           |
| DAI_PIN0       | IN2              | DAI0_PB03_I     | DAI1_PB03_I     | Pin Buffer 3           |
| DAI_PIN0       | IN3              | DAI0_PB04_I     | DAI1_PB04_I     | Pin Buffer 4           |
| DAI_PIN1       | IN0              | DAI0_PB05_I     | DAI1_PB05_I     | Pin Buffer 5           |
| DAI_PIN1       | IN1              | DAI0_PB06_I     | DAI1_PB06_I     | Pin Buffer 6           |
| DAI_PIN1       | IN2              | DAI0_PB07_I     | DAI1_PB07_I     | Pin Buffer 7           |
| DAI_PIN1       | IN3              | DAI0_PB08_I     | DAI1_PB08_I     | Pin Buffer 8           |
| DAI_PIN2       | IN0              | DAI0_PB09_I     | DAI1_PB09_I     | Pin Buffer 9           |
| DAI_PIN2       | IN1              | DAI0_PB10_I     | DAI1_PB10_I     | Pin Buffer 10          |
| DAI_PIN2       | IN2              | DAI0_PB11_I     | DAI1_PB11_I     | Pin Buffer 11          |
| DAI_PIN2       | IN3              | DAI0_PB12_I     | DAI1_PB12_I     | Pin Buffer 12          |
| DAI_PIN3       | IN0              | DAI0_PB13_I     | DAI1_PB13_I     | Pin Buffer 13          |
| DAI_PIN3       | IN1              | DAI0_PB14_I     | DAI1_PB14_I     | Pin Buffer 14          |
| DAI_PIN3       | IN2              | DAI0_PB15_I     | DAI1_PB15_I     | Pin Buffer 15          |
| DAI_PIN3       | IN3              | DAI0_PB16_I     | DAI1_PB16_I     | Pin Buffer 16          |
| DAI_PIN4       | IN0              | DAI0_PB17_I     | DAI1_PB17_I     | Pin Buffer 17          |
| DAI_PIN4       | IN1              | DAI0_PB18_I     | DAI1_PB18_I     | Pin Buffer 18          |
| DAI_PIN4       | IN2              | DAI0_PB19_I     | DAI1_PB19_I     | Pin Buffer 19          |
| DAI_PIN4       | IN3              | DAI0_PB20_I     | DAI1_PB20_I     | Pin Buffer 20          |
| DAI_PIN4       | IN4              | INV_DAI0_PB19_I | INV_DAI1_PB19_I | Inverted Pin Buffer 19 |

Table 33-23: Pin Buffer Assignment Destination Registers (Group D) (Continued)

| DAI Register   | Bit Field Name   | DAI0 Mapping    | DAI1 Mapping    | Description            |
|----------------|------------------|-----------------|-----------------|------------------------|
|                | IN5              | INV_DAI0_PB20_I | INV_DAI1_PB20_I | Inverted Pin Buffer 20 |

Table 33-24: Pin Buffer Enable Destination Registers (Group E)

| DAI Register   | Bit Field Name   | DAI0 Mapping   | DAI1 Mapping   | Description          |
|----------------|------------------|----------------|----------------|----------------------|
| DAI_PBEN0      | IN0              | DAI0_PBEN01_I  | DAI1_PBEN01_I  | Pin Buffer 1 Enable  |
| DAI_PBEN0      | IN1              | DAI0_PBEN02_I  | DAI1_PBEN02_I  | Pin Buffer 2 Enable  |
| DAI_PBEN0      | IN2              | DAI0_PBEN03_I  | DAI1_PBEN03_I  | Pin Buffer 3 Enable  |
| DAI_PBEN0      | IN3              | DAI0_PBEN04_I  | DAI1_PBEN04_I  | Pin Buffer 4 Enable  |
| DAI_PBEN0      | IN4              | DAI0_PBEN05_I  | DAI1_PBEN05_I  | Pin Buffer 5 Enable  |
| DAI_PBEN1      | IN0              | DAI0_PBEN06_I  | DAI1_PBEN06_I  | Pin Buffer 6 Enable  |
| DAI_PBEN1      | IN1              | DAI0_PBEN07_I  | DAI1_PBEN07_I  | Pin Buffer 7 Enable  |
| DAI_PBEN1      | IN2              | DAI0_PBEN08_I  | DAI1_PBEN08_I  | Pin Buffer 8 Enable  |
| DAI_PBEN1      | IN3              | DAI0_PBEN09_I  | DAI1_PBEN09_I  | Pin Buffer 9 Enable  |
| DAI_PBEN1      | IN4              | DAI0_PBEN10_I  | DAI1_PBEN10_I  | Pin Buffer 10 Enable |
| DAI_PBEN2      | IN0              | DAI0_PBEN11_I  | DAI1_PBEN11_I  | Pin Buffer 11 Enable |
| DAI_PBEN2      | IN1              | DAI0_PBEN12_I  | DAI1_PBEN12_I  | Pin Buffer 12 Enable |
| DAI_PBEN2      | IN2              | DAI0_PBEN13_I  | DAI1_PBEN13_I  | Pin Buffer 13 Enable |
| DAI_PBEN2      | IN3              | DAI0_PBEN14_I  | DAI1_PBEN14_I  | Pin Buffer 14 Enable |
| DAI_PBEN2      | IN4              | DAI0_PBEN15_I  | DAI1_PBEN15_I  | Pin Buffer 15 Enable |
| DAI_PBEN3      | IN0              | DAI0_PBEN16_I  | DAI1_PBEN16_I  | Pin Buffer 16 Enable |
| DAI_PBEN3      | IN1              | DAI0_PBEN17_I  | DAI1_PBEN17_I  | Pin Buffer 17 Enable |
| DAI_PBEN3      | IN2              | DAI0_PBEN18_I  | DAI1_PBEN18_I  | Pin Buffer 18 Enable |
| DAI_PBEN3      | IN3              | DAI0_PBEN19_I  | DAI1_PBEN19_I  | Pin Buffer 19 Enable |
| DAI_PBEN3      | IN4              | DAI0_PBEN20_I  | DAI1_PBEN20_I  | Pin Buffer 20 Enable |

The the Miscellaneous Control Registers table, DAI\_MISCAx\_I is an alias name for DAI\_INT\_x\_I.

Table 33-25: Miscellaneous Control Destination Registers (Group F)

| DAI Register   | Bit Field Name   | DAI0 Mapping                | DAI1 Mapping                | Description                                       |
|----------------|------------------|-----------------------------|-----------------------------|---------------------------------------------------|
| DAI_MISC0      | IN6              | DAI0_MISCA0_I/ DAI0_INT_6_I | DAI1_MISCA0_I/ DAI1_INT_6_I | DAIx miscellaneous A0 input, or DAIx interrupt 28 |

Table 33-25: Miscellaneous Control Destination Registers (Group F) (Continued)

| DAI Register   | Bit Field Name   | DAI0 Mapping                | DAI1 Mapping                | Description                                       |
|----------------|------------------|-----------------------------|-----------------------------|---------------------------------------------------|
|                | IN7              | DAI0_MISCA1_I/ DAI0_INT_7_I | DAI1_MISCA1_I/ DAI1_INT_7_I | DAIx miscellaneous A1 input, or DAIx interrupt 29 |
|                | IN8              | DAI0_MISCA2_I/ DAI0_INT_8_I | DAI1_MISCA2_I/ DAI1_INT_8_I | DAIx miscellaneous A2 input, or DAIx interrupt 30 |
|                | IN9              | DAI0_MISCA3_I/ DAI0_INT_9_I | DAI1_MISCA3_I/ DAI1_INT_9_I | DAIx miscellaneous A3 input, or DAIx interrupt 31 |
|                | IN10             | DAI0_MISCA4_I               | DAI1_MISCA4_I               | DAIx miscellaneous A4 input                       |
|                | IN11             | DAI0_MISCA5_I               | DAI1_MISCA5_I               | DAIx miscellaneous A5 input                       |
|                | INV10            | DAI0_INV_MISCA4_I           | DAI1_INV_MISCA4_I           | DAIx inverted miscellaneous A4 input              |
|                | INV11            | DAI0_INV_MISCA5_I           | DAI1_INV_MISCA5_I           | DAIx inverted miscellaneous A5 input              |
| DAI_MISC1      | IN0              | DAI0_INT_0                  | DAI1_INT_0                  | DAIx interrupt 22                                 |
| DAI_MISC1      | IN1              | DAI0_INT_1                  | DAI1_INT_1                  | DAIx interrupt 23                                 |
| DAI_MISC1      | IN2              | DAI0_INT_2                  | DAI1_INT_2                  | DAIx interrupt 24                                 |
| DAI_MISC1      | IN3              | DAI0_INT_3                  | DAI1_INT_3                  | DAIx interrupt 25                                 |
| DAI_MISC1      | IN4              | DAI0_INT_4                  | DAI1_INT_4                  | DAIx interrupt 26                                 |
| DAI_MISC1      | IN5              | DAI0_INT_5                  | DAI1_INT_5                  | DAIx interrupt 27                                 |

The Interrupt Events table presents the information for the following registers.

- DAI\_IMSK\_PRI (Core Interrupt Priority Assignment) register
- DAI\_IMSK\_FE (Falling-Edge Interrupt Mask) and DAI\_IMSK\_RE (Rising-Edge Interrupt Mask) registers
- DAI\_IRPTL\_H (High-Priority Interrupt Latch) and DAI\_IRPTL\_L (Low-Priority Interrupt Latch) registers
- DAI\_IRPTL\_HS (Shadow High Interrupt Latch) and DAI\_IRPTL\_LS (Shadow Low Interrupt Latch) registers

Table 33-26: Interrupt Events

| Bit Field Name   | DAI0 Mapping            | DAI1 Mapping            | Description                                |
|------------------|-------------------------|-------------------------|--------------------------------------------|
| RXVALID          | SPDIF0_RXVALID_INT      | SPDIF1_RXVALID_INT      | SPDIF0 or SPDIF1 RX Valid inter- rupt      |
| RXLOCK           | SPDIF0_RXLOCK_INT       | SPDIF1_RXLOCK_INT       | SPDIF0 or SPDIF1 RX Lock inter- rupt       |
| RXLOSSOF- LOCK   | SPDIF0_RXLOSSOFLOCK_INT | SPDIF1_RXLOSSOFLOCK_INT | SPDIF0 or SPDIF1 RX Loss of Lock interrupt |

Table 33-26: Interrupt Events (Continued)

| Bit Field Name   | DAI0 Mapping                    | DAI1 Mapping                    | Description                                                                    |
|------------------|---------------------------------|---------------------------------|--------------------------------------------------------------------------------|
| RXNONAU- DIO     | SPDIF0_RXNONAUDIO_INT           | SPDIF1_RXNONAUDIO_INT           | SPDIF0 or SPDIF1 RX Non-audio interrupt                                        |
| SRC0MUTE         | SRC0_MUTE_INT                   | SRC4_MUTE_INT                   | DAI0 or DAI1 ASRC0 Mute inter- rupt                                            |
| SRC1MUTE         | SRC1_MUTE_INT                   | SRC5_MUTE_INT                   | DAI0 or DAI1 ASRC1 Mute inter- rupt                                            |
| SRC2MUTE         | SRC2_MUTE_INT                   | SRC6_MUTE_INT                   | DAI0 or DAI1 ASRC2 Mute inter- rupt                                            |
| SRC3MUTE         | SRC3_MUTE_INT                   | SRC7_MUTE_INT                   | DAI0 or DAI1 ASRC3 Mute inter- rupt                                            |
| MISCINT0         | DAI0_INT_00                     | DAI1_INT_00                     | DAI0 or DAI1 Miscellaneous Inter- rupt 0                                       |
| MISCINT1         | DAI0_INT_01                     | DAI1_INT_01                     | DAI0 or DAI1 Miscellaneous Inter- rupt 1                                       |
| MISCINT2         | DAI0_INT_02                     | DAI1_INT_02                     | DAI0 or DAI1 Miscellaneous Inter- rupt 2                                       |
| MISCINT3         | DAI0_INT_03                     | DAI1_INT_03                     | DAI0 or DAI1 Miscellaneous Inter- rupt 3                                       |
| MISCINT4         | DAI0_INT_04                     | DAI1_INT_04                     | DAI0 or DAI1 Miscellaneous Inter- rupt 4                                       |
| MISCINT5         | DAI0_INT_05                     | DAI1_INT_05                     | DAI0 or DAI1 Miscellaneous Inter- rupt 5                                       |
| MISCINT6         | DAI0_EXTMISCA0_INT/ DAI0_INT_06 | DAI1_EXTMISCA0_INT/ DAI1_INT_06 | DAI0 or DAI1 Miscellaneous Inter- rupt 6 / External Miscellaneous A0 Interrupt |
| MISCINT7         | DAI0_EXTMISCA1_INT/ DAI0_INT_07 | DAI1_EXTMISCA1_INT/ DAI1_INT_07 | DAI0 or DAI1 Miscellaneous Inter- rupt 7 / External Miscellaneous A1 Interrupt |
| MISCINT8         | DAI0_EXTMISCA2_INT/ DAI0_INT_08 | DAI1_EXTMISCA2_INT/ DAI1_INT_08 | DAI0 or DAI1 Miscellaneous Inter- rupt 8 / External Miscellaneous A2 Interrupt |
| MISCINT9         | DAI0_EXTMISCA3_INT/ DAI0_INT_09 | DAI1_EXTMISCA3_INT/ DAI1_INT_09 | DAI0 or DAI1 Miscellaneous Inter- rupt 9 / External Miscellaneous A3 Interrupt |

## ADSP-SC58x DAI Register Descriptions

The Digital Audio Interface (DAIn) registers are used to configure DAI destinations for the DAI sources shown in the Group A - F Routing tables. (DAI) contains the following registers.

Table 33-27: ADSP-SC58x DAI Register List

| Name         | Description                                   |
|--------------|-----------------------------------------------|
| DAI_CLK0     | Clock Routing Control Register 0              |
| DAI_CLK1     | Clock Routing Control Register 1              |
| DAI_CLK2     | Clock Routing Control Register 2              |
| DAI_CLK3     | Clock Routing Control Register 3              |
| DAI_CLK4     | Clock Routing Control Register 4              |
| DAI_CLK5     | Clock Routing Control Register 5              |
| DAI_DAT0     | Serial Data Routing Control Register 0        |
| DAI_DAT1     | Serial Data Routing Control Register 1        |
| DAI_DAT2     | Serial Data Routing Control Register 2        |
| DAI_DAT3     | Serial Data Routing Control Register 3        |
| DAI_DAT4     | Serial Data Routing Control Register 4        |
| DAI_DAT5     | Serial Data Routing Control Register 5        |
| DAI_DAT6     | Serial Data Routing Control Register 6        |
| DAI_FS0      | Frame Sync Routing Control Register 0         |
| DAI_FS1      | Frame Sync Routing Control Register 1         |
| DAI_FS2      | Frame Sync Routing Control Register 2         |
| DAI_FS4      | Frame Sync Routing Control Register 4         |
| DAI_IMSK_FE  | Falling-Edge Interrupt Mask Register          |
| DAI_IMSK_PRI | Core Interrupt Priority Assignment Register   |
| DAI_IMSK_RE  | Rising-Edge Interrupt Mask Register           |
| DAI_IRPTL_H  | High Priority Interrupt Latch Register        |
| DAI_IRPTL_HS | Shadow High Priority Interrupt Latch Register |
| DAI_IRPTL_L  | Low Priority Interrupt Latch Register         |
| DAI_IRPTL_LS | Shadow Low Priority Interrupt Latch Register  |
| DAI_MISC0    | Miscellaneous Control Register 0              |
| DAI_MISC1    | Miscellaneous Control Register 1              |
| DAI_PBEN0    | Pin Buffer Enable Register 0                  |
| DAI_PBEN1    | Pin Buffer Enable Register 1                  |
| DAI_PBEN2    | Pin Buffer Enable Register 2                  |
| DAI_PBEN3    | Pin Buffer Enable Register 3                  |
| DAI_PIN0     | Pin Buffer Assignment Register 0              |

Table 33-27: ADSP-SC58x DAI Register List (Continued)

| Name         | Description                      |
|--------------|----------------------------------|
| DAI_PIN1     | Pin Buffer Assignment Register 1 |
| DAI_PIN2     | Pin Buffer Assignment Register 2 |
| DAI_PIN3     | Pin Buffer Assignment Register 3 |
| DAI_PIN4     | Pin Buffer Assignment Register 4 |
| DAI_PIN_STAT | Pin Status Register              |

## Clock Routing Control Register 0

The DAI\_CLK0 register provides clock routing connections for the serial ports (SPORTs).

Figure 33-17: DAI\_CLK0 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000014_245c943b2d3d19d3ac6203e3c4f9392267708e88ad345ccc51ad78c8c1f253f6.png)

Table 33-28: DAI\_CLK0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:25 (R/W)        | IN5        | Input Clock 5. DAI_CLK0.IN5 holds the Source signal assignment that will be routed to the DAI_CLK0.IN5 Destination. Refer to the Group A Signals table for Source and Destination mappings |
| 24:20 (R/W)        | IN4        | Input Clock 4. DAI_CLK0.IN4 holds the Source signal assignment that will be routed to the DAI_CLK0.IN4 Destination. Refer to the Group A Signals table for Source and Destination mappings |
| 19:15 (R/W)        | IN3        | Input Clock 3. DAI_CLK0.IN3 holds the Source signal assignment that will be routed to the DAI_CLK0.IN3 Destination. Refer to the Group A Signals table for Source and Destination mappings |
| 14:10 (R/W)        | IN2        | Input Clock 2. DAI_CLK0.IN2 holds the Source signal assignment that will be routed to the DAI_CLK0.IN2 Destination. Refer to the Group A Signals table for Source and Destination mappings |
| 9:5 (R/W)          | IN1        | Input Clock 1. DAI_CLK0.IN1 holds the Source signal assignment that will be routed to the DAI_CLK0.IN1 Destination. Refer to the Group A Signals table for Source and Destination mappings |

Table 33-28: DAI\_CLK0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4:0 (R/W)          | IN0        | Input Clock 0. DAI_CLK0.IN0 holds the Source signal assignment that will be routed to the DAI_CLK0.IN0 Destination. Refer to the Group A Signals table for Source and Destination mappings |

## Clock Routing Control Register 1

The DAI\_CLK1 register provides clock routing connections for the asynchronous sample rate converters (ASRC).

Figure 33-18: DAI\_CLK1 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000015_730f25513d24cc99e0a81dfab19ca2dedda72a72c353c1e04e833781dd72c503.png)

Table 33-29: DAI\_CLK1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:25 (R/W)        | IN5        | Input Clock 5. DAI_CLK1.IN5 holds the Source signal assignment that will be routed to the DAI_CLK1.IN5 Destination. Refer to the Group A Signals table for Source and Destination mappings  |
| 24:20 (R/W)        | IN4        | Input Clock 4. DAI_CLK1.IN4 holds the Source signal assignment that will be routed to the DAI_CLK1.IN4 Destination. Refer to the Group A Signals table for Source and Destination mappings  |
| 19:15 (R/W)        | IN3        | Input Clock 3. DAI_CLK1.IN3 holds the Source signal assignment that will be routed to the DAI_CLK1.IN3 Destination. Refer to the Group A Signals table for Source and Destination mappings  |
| 14:10 (R/W)        | IN2        | Input Clock 2. DAI_CLK1.IN2 holds the Source signal assignment that will be routed to the DAI_CLK1.IN2 Destination. Refer to the Group A Signals table for Source and Destination mappings  |
| 9:5 (R/W)          | IN1        | Input Clock 1. DAI_CLK1.IN1 holds the Source signal assignment that will be routed to the DAI_CLK1.IN1 Destination. Refer to the Group A Signals table for Source and Destination mappings. |

Table 33-29: DAI\_CLK1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4:0 (R/W)          | IN0        | Input Clock 0. DAI_CLK1.IN0 holds the Source signal assignment that will be routed to the DAI_CLK1.IN0 Destination. Refer to the Group A Signals table for Source and Destination mappings. |

## Clock Routing Control Register 2

The DAI\_CLK2 register provides clock routing connections for the S/PDIF and ASRC.

Figure 33-19: DAI\_CLK2 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000016_42055eb7b64010d03dc7f45955e021fb50fd7bf3478815aa705e233b58b5abae.png)

Table 33-30: DAI\_CLK2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:10 (R/W)        | IN2        | Input Clock 2. DAI_CLK2.IN2 holds the Source signal assignment that will be routed to the DAI_CLK2.IN2 Destination. Refer to the Group A Signals table for Source and Destination mappings. |
| 9:5 (R/W)          | IN1        | Input Clock 1. DAI_CLK2.IN1 holds the Source signal assignment that will be routed to the DAI_CLK2.IN1 Destination. Refer to the Group A Signals table for Source and Destination mappings. |
| 4:0 (R/W)          | IN0        | Input Clock 0. DAI_CLK2.IN0 holds the Source signal assignment that will be routed to the DAI_CLK2.IN0 Destination. Refer to the Group A Signals table for Source and Destination mappings. |

## Clock Routing Control Register 3

The DAI\_CLK3 register provides clock routing connections for the S/PDIF .

Figure 33-20: DAI\_CLK3 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000017_a06046817f1461ab03f5df107595ae934d994781aeadd8985090ce4853ef1a04.png)

Table 33-31: DAI\_CLK3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                            |
|--------------------|------------|--------------------------------------------------------------------|
| 29:25              | IN5        | Input Clock 5.                                                     |
| (R/W)              |            | The DAI_CLK3.IN5 bit field provides the S/PDIF oversampling clock. |

## Clock Routing Control Register 4

The DAI\_CLK4 register provides clock routing connections for the precision clock generators (PCGs).

Figure 33-21: DAI\_CLK4 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000018_94b0233c8e1a551d1d429e52eacb5b43865c71b5fbedb4f0e8d90b3a452ea3fc.png)

Table 33-32: DAI\_CLK4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:25 (R/W)        | IN5        | Input Clock 5. DAI_CLK4.IN5 holds the Source signal assignment that will be routed to the DAI_CLK4.IN5 Destination. Refer to the Group A Signals table for Source and Destination mappings. |
| 24:20 (R/W)        | IN4        | Input Clock 4. DAI_CLK4.IN4 holds the Source signal assignment that will be routed to the DAI_CLK4.IN4 Destination. Refer to the Group A Signals table for Source and Destination mappings. |
| 19:15 (R/W)        | IN3        | Input Clock 3. DAI_CLK4.IN3 holds the Source signal assignment that will be routed to the DAI_CLK4.IN3 Destination. Refer to the Group A Signals table for Source and Destination mappings. |
| 9:5 (R/W)          | IN1        | Input Clock 1. DAI_CLK4.IN1 holds the Source signal assignment that will be routed to the DAI_CLK4.IN1 Destination. Refer to the Group A Signals table for Source and Destination mappings. |
| 4:0 (R/W)          | IN0        | Input Clock 0. DAI_CLK4.IN0 holds the Source signal assignment that will be routed to the DAI_CLK4.IN0 Destination. Refer to the Group A Signals table for Source and Destination mappings. |

## Clock Routing Control Register 5

The DAI\_CLK5 register provides clock routing connections for the serial ports.

Figure 33-22: DAI\_CLK5 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000019_fb2dd87c9d7a0a3909ca6c8f0d6a942991d1dd6c2179cc0a1ec12f9c9055d8e2.png)

Table 33-33: DAI\_CLK5 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9:5 (R/W)          | IN1        | Input Clock 1. DAI_CLK5.IN1 holds the Source signal assignment that will be routed to the DAI_CLK5.IN1 Destination. Refer to the Group A Signals table for Source and Destination mappings. |
| 4:0 (R/W)          | IN0        | Input Clock 0. DAI_CLK5.IN0 holds the Source signal assignment that will be routed to the DAI_CLK5.IN0 Destination. Refer to the Group A Signals table for Source and Destination mappings. |

## Serial Data Routing Control Register 0

The DAI\_DAT0 register routes serial data to the serial ports.

Figure 33-23: DAI\_DAT0 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000020_630844f1cd8edae62aca51c95106e6a5ab2d649c8fefabfcd0d253a91aac58c2.png)

Table 33-34: DAI\_DAT0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:24 (R/W)        | IN4        | Input Data 4. DAI_DAT0.IN4 holds the Source signal assignment that will be routed to the DAI_DAT0.IN4 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 23:18 (R/W)        | IN3        | Input Data 3. DAI_DAT0.IN3 holds the Source signal assignment that will be routed to the DAI_DAT0.IN3 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 17:12 (R/W)        | IN2        | Input Data 2. DAI_DAT0.IN2 holds the Source signal assignment that will be routed to the DAI_DAT0.IN2 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 11:6 (R/W)         | IN1        | Input Data 1. DAI_DAT0.IN1 holds the Source signal assignment that will be routed to the DAI_DAT0.IN1 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 5:0 (R/W)          | IN0        | Input Data 0. DAI_DAT0.IN0 holds the Source signal assignment that will be routed to the DAI_DAT0.IN0 Destination. Refer to the Group B Signals table for Source and Destination mappings. |

## Serial Data Routing Control Register 1

The DAI\_DAT1 register routes serial data to the serial ports.

Figure 33-24: DAI\_DAT1 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000021_44ba4c6a1649d82624f24490ec4fd75f1c29584fd656db5a87de80a9bf9f8e4b.png)

Table 33-35: DAI\_DAT1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:24 (R/W)        | IN4        | Input Data 4. DAI_DAT1.IN4 holds the Source signal assignment that will be routed to the DAI_DAT1.IN4 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 23:18 (R/W)        | IN3        | Input Data 3. DAI_DAT1.IN3 holds the Source signal assignment that will be routed to the DAI_DAT1.IN3 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 17:12 (R/W)        | IN2        | Input Data 2. DAI_DAT1.IN2 holds the Source signal assignment that will be routed to the DAI_DAT1.IN2 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 11:6 (R/W)         | IN1        | Input Data 1. DAI_DAT1.IN1 holds the Source signal assignment that will be routed to the DAI_DAT1.IN1 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 5:0 (R/W)          | IN0        | Input Data 0. DAI_DAT1.IN0 holds the Source signal assignment that will be routed to the DAI_DAT1.IN0 Destination. Refer to the Group B Signals table for Source and Destination mappings. |

## Serial Data Routing Control Register 2

The DAI\_DAT2 register routes serial data to the serial ports and the asynchronous sample rate converter (ASRC).

Figure 33-25: DAI\_DAT2 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000022_edf1b2d92ff99e73547ef092a92cab48ef1fde6a552ea77ff2808135455511a9.png)

Table 33-36: DAI\_DAT2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:24 (R/W)        | IN4        | Input Data 4. DAI_DAT2.IN4 holds the Source signal assignment that will be routed to the DAI_DAT2.IN4 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 23:18 (R/W)        | IN3        | Input Data 3. DAI_DAT2.IN3 holds the Source signal assignment that will be routed to the DAI_DAT2.IN3 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 17:12 (R/W)        | IN2        | Input Data 2. DAI_DAT2.IN2 holds the Source signal assignment that will be routed to the DAI_DAT2.IN2 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 11:6 (R/W)         | IN1        | Input Data 1. DAI_DAT2.IN1 holds the Source signal assignment that will be routed to the DAI_DAT2.IN1 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 5:0 (R/W)          | IN0        | Input Data 0. DAI_DAT2.IN0 holds the Source signal assignment that will be routed to the DAI_DAT2.IN0 Destination. Refer to the Group B Signals table for Source and Destination mappings. |

## Serial Data Routing Control Register 3

The DAI\_DAT3 register routes serial data to the asynchronous sample rate converter (ASRC).

Figure 33-26: DAI\_DAT3 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000023_2b27f62497126398814593561ecfe53c155cae841006ebf20a3b1207fd26bca0.png)

Table 33-37: DAI\_DAT3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:24 (R/W)        | IN4        | Input Data 4. DAI_DAT3.IN4 holds the Source signal assignment that will be routed to the DAI_DAT3.IN4 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 23:18 (R/W)        | IN3        | Input Data 3. DAI_DAT3.IN3 holds the Source signal assignment that will be routed to the DAI_DAT3.IN3 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 17:12 (R/W)        | IN2        | Input Data 2. DAI_DAT3.IN2 holds the Source signal assignment that will be routed to the DAI_DAT3.IN2 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 11:6 (R/W)         | IN1        | Input Data 1. DAI_DAT3.IN1 holds the Source signal assignment that will be routed to the DAI_DAT3.IN1 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 5:0 (R/W)          | IN0        | Input Data 0. DAI_DAT3.IN0 holds the Source signal assignment that will be routed to the DAI_DAT3.IN0 Destination. Refer to the Group B Signals table for Source and Destination mappings. |

## Serial Data Routing Control Register 4

The DAI\_DAT4 register routes serial data to the S/PDIF .

Figure 33-27: DAI\_DAT4 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000024_09aef0549bc61fc779797c4d19903b88b16c7b24538b2fecb28ec503338a0135.png)

Table 33-38: DAI\_DAT4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5:0 (R/W)          | IN0        | Input Data 0. DAI_DAT4.IN0 holds the Source signal assignment that will be routed to the DAI_DAT4.IN0 Destination. Refer to the Group B Signals table for Source and Destination mappings. |

## Serial Data Routing Control Register 5

The DAI\_DAT5 register routes serial data to the S/PDIF .

Figure 33-28: DAI\_DAT5 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000025_af4ddfa85bfb414ebacc0d18ace95ced3d63e39d9a8baef936abc5990cd3d150.png)

Table 33-39: DAI\_DAT5 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------|
| 29:24 (R/W)        | IN4        | Input Data 4. The DAI_DAT5.IN4 bit field routes input data to the S/PDIF Biphase receiver stream. |

## Serial Data Routing Control Register 6

The DAI\_DAT6 register routes serial data to the serial ports.

Figure 33-29: DAI\_DAT6 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000026_a3536ad62937f83c8044bfd31d3133d6481f063a0860602b00f03c78faaf004f.png)

Table 33-40: DAI\_DAT6 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:24 (R/W)        | IN4        | Input Data 4. DAI_DAT6.IN4 holds the Source signal assignment that will be routed to the DAI_DAT6.IN4 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 23:18 (R/W)        | IN3        | Input Data 3. DAI_DAT6.IN3 holds the Source signal assignment that will be routed to the DAI_DAT6.IN3 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 17:12 (R/W)        | IN2        | Input Data 2. DAI_DAT6.IN2 holds the Source signal assignment that will be routed to the DAI_DAT6.IN2 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 11:6 (R/W)         | IN1        | Input Data 1. DAI_DAT6.IN1 holds the Source signal assignment that will be routed to the DAI_DAT6.IN1 Destination. Refer to the Group B Signals table for Source and Destination mappings. |
| 5:0 (R/W)          | IN0        | Input Data 0. DAI_DAT6.IN0 holds the Source signal assignment that will be routed to the DAI_DAT6.IN0 Destination. Refer to the Group B Signals table for Source and Destination mappings. |

## Frame Sync Routing Control Register 0

The DAI\_FS0 register routes frame syncs to the serial ports.

Figure 33-30: DAI\_FS0 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000027_0247427328cecc88f4cabaf7e9f02504d1db400e3cfcb43291156f2176c86c77.png)

Table 33-41: DAI\_FS0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:25 (R/W)        | IN5        | Input Frame Sync 5. DAI_FS0.IN5 holds the Source signal assignment that will be routed to the DAI_FS0.IN5 Destination. Refer to the Group C Signals table for Source and Des- tination mappings. |
| 24:20 (R/W)        | IN4        | Input Frame Sync 4. DAI_FS0.IN4 holds the Source signal assignment that will be routed to the DAI_FS0.IN4 Destination. Refer to the Group C Signals table for Source and Des- tination mappings. |
| 19:15 (R/W)        | IN3        | Input Frame Sync 3. DAI_FS0.IN3 holds the Source signal assignment that will be routed to the DAI_FS0.IN3 Destination. Refer to the Group C Signals table for Source and Des- tination mappings. |
| 14:10 (R/W)        | IN2        | Input Frame Sync 2. DAI_FS0.IN2 holds the Source signal assignment that will be routed to the DAI_FS0.IN2 Destination. Refer to the Group C Signals table for Source and Des- tination mappings. |
| 9:5 (R/W)          | IN1        | Input Frame Sync 1. DAI_FS0.IN1 holds the Source signal assignment that will be routed to the DAI_FS0.IN1 Destination. Refer to the Group C Signals table for Source and Des- tination mappings. |

Table 33-41: DAI\_FS0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4:0 (R/W)          | IN0        | Input Frame Sync 0. DAI_FS0.IN0 holds the Source signal assignment that will be routed to the DAI_FS0.IN0 Destination. Refer to the Group C Signals table for Source and Des- tination mappings. |

## Frame Sync Routing Control Register 1

The DAI\_FS1 register routes frame syncs to the asynchronous sample rate converter (ASRC).

Figure 33-31: DAI\_FS1 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000028_2068c41e09fe3cbd5b185a945ee57e3c2805b617e18048ad9448010cd2a666da.png)

Table 33-42: DAI\_FS1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:25 (R/W)        | IN5        | Input Frame Sync 5. DAI_FS1.IN5 holds the Source signal assignment that will be routed to the DAI_FS1.IN5 Destination. Refer to the Group C Signals table for Source and Des- tination mappings. |
| 24:20 (R/W)        | IN4        | Input Frame Sync 4. DAI_FS1.IN4 holds the Source signal assignment that will be routed to the DAI_FS1.IN4 Destination. Refer to the Group C Signals table for Source and Des- tination mappings. |
| 19:15 (R/W)        | IN3        | Input Frame Sync 3. DAI_FS1.IN3 holds the Source signal assignment that will be routed to the DAI_FS1.IN3 Destination. Refer to the Group C Signals table for Source and Des- tination mappings. |
| 14:10 (R/W)        | IN2        | Input Frame Sync 2. DAI_FS1.IN2 holds the Source signal assignment that will be routed to the DAI_FS1.IN2 Destination. Refer to the Group C Signals table for Source and Des- tination mappings. |
| 9:5 (R/W)          | IN1        | Input Frame Sync 1. DAI_FS1.IN1 holds the Source signal assignment that will be routed to the DAI_FS1.IN1 Destination. Refer to the Group C Signals table for Source and Des- tination mappings. |

Table 33-42: DAI\_FS1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4:0 (R/W)          | IN0        | Input Frame Sync 0. DAI_FS1.IN0 holds the Source signal assignment that will be routed to the DAI_FS1.IN0 Destination. Refer to the Group C Signals table for Source and Des- tination mappings. |

## Frame Sync Routing Control Register 2

Figure 33-32: DAI\_FS2 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000029_ef2070e2787139530e9e70435dba0f640989febac9a692f70e8f842c3083a270.png)

Table 33-43: DAI\_FS2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:10 (R/W)        | IN2        | Input Frame Sync 2. DAI_FS2.IN2 holds the Source signal assignment that will be routed to the DAI_FS2.IN2 Destination. Refer to the Group C Signals table for Source and Des- tination mappings. |
| 9:5 (R/W)          | IN1        | Input Frame Sync 1. The DAI_FS2.IN1 bit field routes the frame sync output to the ASRC3 frame sync input port.                                                                                   |
| 4:0 (R/W)          | IN0        | Input Frame Sync 0. DAI_FS2.IN0 holds the Source signal assignment that will be routed to the DAI_FS2.IN0 Destination. Refer to the Group C Signals table for Source and Des- tination mappings. |

## Frame Sync Routing Control Register 4

Figure 33-33: DAI\_FS4 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000030_ee41c02aa6265883b6bd9eda0d9c6451053abf847fc1026f0b98c6044a27a66e.png)

Table 33-44: DAI\_FS4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9:5 (R/W)          | IN1        | Input Frame Sync 1. DAI_FS4.IN1 holds the Source signal assignment that will be routed to the DAI_FS4.IN1 Destination. Refer to the Group C Signals table for Source and Des- tination mappings. |
| 4:0 (R/W)          | IN0        | Input Frame Sync 0. DAI_FS4.IN0 holds the Source signal assignment that will be routed to the DAI_FS4.IN0 Destination. Refer to the Group C Signals table for Source and Des- tination mappings. |

## Falling-Edge Interrupt Mask Register

The DAI\_IMSK\_FE register masks and unmasks interrupts generated on the falling edge of a waveform. Note that any of the Group E signals can be mapped to any of the Miscellaneous interrupts (9-0).

Figure 33-34: DAI\_IMSK\_FE Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000031_2139eada5311f068db60f655b73bda38f7482b99ddc6ed14e619a191158713f4.png)

Table 33-45: DAI\_IMSK\_FE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | MISCINT9   | Miscellaneous Interrupt 9. Setting the DAI_IMSK_FE.MISCINT9 bit unmasks the interrupt for the falling edge of the routed signal. |
| 30 (R/W)           | MISCINT8   | Miscellaneous Interrupt 8. Setting the DAI_IMSK_FE.MISCINT8 bit unmasks the interrupt for the falling edge of the routed signal. |
| 29 (R/W)           | MISCINT7   | Miscellaneous Interrupt 7. Setting the DAI_IMSK_FE.MISCINT7 bit unmasks the interrupt for the falling edge of the routed signal. |

Table 33-45: DAI\_IMSK\_FE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 28 (R/W)           | MISCINT6   | Miscellaneous Interrupt 6. Setting the DAI_IMSK_FE.MISCINT6 bit unmasks the interrupt for the falling edge of the routed signal.                                                                                          |
| 27 (R/W)           | MISCINT5   | Miscellaneous Interrupt 5. Setting the DAI_IMSK_FE.MISCINT5 bit unmasks the interrupt for the falling edge of the routed signal.                                                                                          |
| 26 (R/W)           | MISCINT4   | Miscellaneous Interrupt 4. Setting the DAI_IMSK_FE.MISCINT4 bit unmasks the interrupt for the falling edge of the routed signal.                                                                                          |
| 25 (R/W)           | MISCINT3   | Miscellaneous Interrupt 3. Setting the DAI_IMSK_FE.MISCINT3 bit unmasks the interrupt for the falling edge of the routed signal.                                                                                          |
| 24 (R/W)           | MISCINT2   | Miscellaneous Interrupt 2. Setting the DAI_IMSK_FE.MISCINT2 bit unmasks the interrupt for the falling edge of the routed signal.                                                                                          |
| 23 (R/W)           | MISCINT1   | Miscellaneous Interrupt 1. Setting the DAI_IMSK_FE.MISCINT1 bit unmasks the interrupt for the falling edge of the routed signal.                                                                                          |
| 22 (R/W)           | MISCINT0   | Miscellaneous Interrupt 0. Setting the DAI_IMSK_FE.MISCINT0 bit unmasks the interrupt for the falling edge of the routed signal.                                                                                          |
| 21 (R/W)           | SRC3MUTE   | SRC3 Mute. The DAI_IMSK_FE.SRC3MUTE bit masks or unmasks the corresponding SRC mute out interrupt for the falling edge of this interrupt. This interrupt can be generat- ed either entering mute, exiting muting or both. |
| 20 (R/W)           | SRC2MUTE   | SRC2 Mute. The DAI_IMSK_FE.SRC2MUTE bit masks or unmasks the corresponding SRC mute out interrupt for the falling edge of this interrupt. This interrupt can be generat- ed either entering mute, exiting muting or both. |
| 19 (R/W)           | SRC1MUTE   | SRC1 Mute. The DAI_IMSK_FE.SRC1MUTE bit masks or unmasks the corresponding SRC mute out interrupt for the falling edge of this interrupt. This interrupt can be generat- ed either entering mute, exiting muting or both. |

Table 33-45: DAI\_IMSK\_FE Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                  |
|--------------------|--------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (R/W)           | SRC0MUTE     | SRC0 Mute. The DAI_IMSK_FE.SRC0MUTE bit masks or unmasks the corresponding SRC mute out interrupt for the falling edge of this interrupt. This interrupt can be generat- ed either entering mute, exiting muting or both.                                |
| 4 (R/W)            | RXNONAUDIO   | Receive Non Audio. The DAI_IMSK_FE.RXNONAUDIO bit masks or unmasks the non audio frame mode interrupt for the falling edge of this interrupt. If the channel status indicates non-PCM audio, the NONAUDIO bit flag is set.                               |
| 2 (R/W)            | RXLOSSOFLOCK | Receive Loss of Lock. The DAI_IMSK_FE.RXLOSSOFLOCK bit masks or unmasks the emphasis loss of lock interrupt for the falling edge of this interrupt. The loss of lock status is set by the S/PDIF receiver if receiver looses the lock of biphase stream. |
| 1 (R/W)            | RXLOCK       | Receive Lock. The DAI_IMSK_FE.RXLOCK bit masks or unmasks the S/PDIF receiver lock for the falling edge of this interrupt. This interrupt occurs when the S/PDIF receiver locks to the S/PDIF stream.                                                    |
| 0 (R/W)            | RXVALID      | Receive Valid. Setting the DAI_IMSK_FE.RXVALID bit unmasks the interrupt for the falling edge of the routed signal. This interrupt is set based on whether data received by the S/ PDIF is linear PCM or non-linear audio data.                          |

## Core Interrupt Priority Assignment Register

The DAI\_IMSK\_PRI register masks interrupts for DAI high or DAI low interrupt priority.

Figure 33-35: DAI\_IMSK\_PRI Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000032_ed49a7e402cb14a727809cf04ce6c029351dd2bf11b147a8add83c453252d79e.png)

Table 33-46: DAI\_IMSK\_PRI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | MISCINT9   | Miscellaneous Interrupt 9. If the DAI_IMSK_PRI.MISCINT9 bit is cleared (=0), the interrupt is mapped to INTR_DAI_IRQL signal from the SEC. If the DAI_IMSK_PRI.MISCINT9 bit is set (=1) the interrupt is mapped to INTR_DAI_IRQH signal from the SEC. |
| 30 (R/W)           | MISCINT8   | Miscellaneous Interrupt 8. If the DAI_IMSK_PRI.MISCINT8 bit is cleared (=0), the interrupt is mapped to INTR_DAI_IRQL signal from the SEC. If the DAI_IMSK_PRI.MISCINT8 bit is set (=1) the interrupt is mapped to INTR_DAI_IRQH signal from the SEC. |

Table 33-46: DAI\_IMSK\_PRI Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W)           | MISCINT7   | Miscellaneous Interrupt 7. If the DAI_IMSK_PRI.MISCINT7 bit is cleared (=0), the interrupt is mapped to INTR_DAI_IRQL signal from the SEC. If the DAI_IMSK_PRI.MISCINT7 bit is set (=1) the interrupt is mapped to INTR_DAI_IRQH signal from the SEC. |
| 28 (R/W)           | MISCINT6   | Miscellaneous Interrupt 6. If the DAI_IMSK_PRI.MISCINT6 bit is cleared (=0), the interrupt is mapped to INTR_DAI_IRQL signal from the SEC. If the DAI_IMSK_PRI.MISCINT6 bit is set (=1) the interrupt is mapped to INTR_DAI_IRQH signal from the SEC. |
| 27 (R/W)           | MISCINT5   | Miscellaneous Interrupt 5. If the DAI_IMSK_PRI.MISCINT5 bit is cleared (=0), the interrupt is mapped to INTR_DAI_IRQL signal from the SEC. If the DAI_IMSK_PRI.MISCINT5 bit is set (=1) the interrupt is mapped to INTR_DAI_IRQH signal from the SEC. |
| 26 (R/W)           | MISCINT4   | Miscellaneous Interrupt 4. If the DAI_IMSK_PRI.MISCINT4 bit is cleared (=0), the interrupt is mapped to INTR_DAI_IRQL signal from the SEC. If the DAI_IMSK_PRI.MISCINT4 bit is set (=1) the interrupt is mapped to INTR_DAI_IRQH signal from the SEC. |
| 25 (R/W)           | MISCINT3   | Miscellaneous Interrupt 3. If the DAI_IMSK_PRI.MISCINT3 bit is cleared (=0), the interrupt is mapped to INTR_DAI_IRQL signal from the SEC. If the DAI_IMSK_PRI.MISCINT3 bit is set (=1) the interrupt is mapped to INTR_DAI_IRQH signal from the SEC. |
| 24 (R/W)           | MISCINT2   | Miscellaneous Interrupt 2. If the DAI_IMSK_PRI.MISCINT2 bit is cleared (=0), the interrupt is mapped to INTR_DAI_IRQL signal from the SEC. If the DAI_IMSK_PRI.MISCINT2 bit is set (=1) the interrupt is mapped to INTR_DAI_IRQH signal from the SEC. |
| 23 (R/W)           | MISCINT1   | Miscellaneous Interrupt 1. If the DAI_IMSK_PRI.MISCINT1 bit is cleared (=0), the interrupt is mapped to INTR_DAI_IRQL signal from the SEC. If the DAI_IMSK_PRI.MISCINT1 bit is set (=1) the interrupt is mapped to INTR_DAI_IRQH signal from the SEC. |
| 22 (R/W)           | MISCINT0   | Miscellaneous Interrupt 0. If the DAI_IMSK_PRI.MISCINT0 bit is cleared (=0), the interrupt is mapped to INTR_DAI_IRQL signal from the SEC. If the DAI_IMSK_PRI.MISCINT0 bit is set (=1) the interrupt is mapped to INTR_DAI_IRQH signal from the SEC. |
| 21 (R/W)           | SRC3MUTE   | SRC3 Mute. If the DAI_IMSK_PRI.SRC3MUTE bit is cleared (=0), the interrupt is mapped to INTR_DAI_IRQL signal from the SEC. If the DAI_IMSK_PRI.SRC3MUTE bit is set (=1) the interrupt is mapped to INTR_DAI_IRQH signal from the SEC.                 |

Table 33-46: DAI\_IMSK\_PRI Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                   |
|--------------------|--------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W)           | SRC2MUTE     | SRC2 Mute. If the DAI_IMSK_PRI.SRC2MUTE bit is cleared (=0), the interrupt is mapped to INTR_DAI_IRQL signal from the SEC. If the DAI_IMSK_PRI.SRC2MUTE bit is set (=1) the interrupt is mapped to INTR_DAI_IRQH signal from the SEC.                     |
| 19 (R/W)           | SRC1MUTE     | SRC1 Mute. If the DAI_IMSK_PRI.SRC1MUTE bit is cleared (=0), the interrupt is mapped to INTR_DAI_IRQL signal from the SEC. If the DAI_IMSK_PRI.SRC1MUTE bit is set (=1) the interrupt is mapped to INTR_DAI_IRQH signal from the SEC.                     |
| 18 (R/W)           | SRC0MUTE     | SRC0 Mute. If the DAI_IMSK_PRI.SRC0MUTE bit is cleared (=0), the interrupt is mapped to INTR_DAI_IRQL signal from the SEC. If the DAI_IMSK_PRI.SRC0MUTE bit is set (=1) the interrupt is mapped to INTR_DAI_IRQH signal from the SEC.                     |
| 4 (R/W)            | RXNONAUDIO   | Receiver Non-Audio. If the DAI_IMSK_PRI.RXNONAUDIO bit is cleared (=0), the interrupt is mapped to INTR_DAI_IRQL signal from the SEC. If the DAI_IMSK_PRI.RXNONAUDIO bit is set (=1) the interrupt is mapped to INTR_DAI_IRQH signal from the SEC.        |
| 2 (R/W)            | RXLOSSOFLOCK | Receive Loss of Lock. The DAI_IMSK_PRI.RXLOSSOFLOCK bit masks or unmasks the emphasis loss of lock interrupt for the falling edge of this interrupt. The loss of lock status is set by the S/PDIF receiver if receiver looses the lock of biphase stream. |
| 1 (R/W)            | RXLOCK       | Receive Lock. If the DAI_IMSK_PRI.RXLOCK bit is cleared (=0), the interrupt is mapped to INTR_DAI_IRQL signal from the SEC. If the DAI_IMSK_PRI.RXLOCK bit is set (=1) the interrupt is mapped to INTR_DAI_IRQH signal from the SEC.                      |
| 0 (R/W)            | RXVALID      | Receive Valid. If the DAI_IMSK_PRI.RXVALID bit is cleared (=0), the interrupt is mapped to the INTR_DAI_IRQL from the SEC. If the DAI_IMSK_PRI.RXVALID bit is set (=1) the interrupt is mapped to INTR_DAI_IRQH signal from the SEC.                      |

## Rising-Edge Interrupt Mask Register

The DAI\_IMSK\_RE register masks and unmasks interrupts generated on the rising edge of a waveform. Note that any of the Group E signals can be mapped to any of the miscellaneous interrupts (9-0).

Figure 33-36: DAI\_IMSK\_RE Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000033_2139eada5311f068db60f655b73bda38f7482b99ddc6ed14e619a191158713f4.png)

Table 33-47: DAI\_IMSK\_RE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | MISCINT9   | Miscellaneous Interrupt 9. Setting the DAI_IMSK_RE.MISCINT9 bit unmasks the interrupt for the rising edge of the routed signal. |
| 30 (R/W)           | MISCINT8   | Miscellaneous Interrupt 8. Setting the DAI_IMSK_RE.MISCINT8 bit unmasks the interrupt for the rising edge of the routed signal. |
| 29 (R/W)           | MISCINT7   | Miscellaneous Interrupt 7. Setting the DAI_IMSK_RE.MISCINT7 bit unmasks the interrupt for the rising edge of the routed signal. |

Table 33-47: DAI\_IMSK\_RE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 28 (R/W)           | MISCINT6   | Miscellaneous Interrupt 6. Setting the DAI_IMSK_RE.MISCINT6 bit unmasks the interrupt for the rising edge of the routed signal.                                                                                        |
| 27 (R/W)           | MISCINT5   | Miscellaneous Interrupt 5. Setting the DAI_IMSK_RE.MISCINT5 bit unmasks the interrupt for the rising edge of the routed signal.                                                                                        |
| 26 (R/W)           | MISCINT4   | Miscellaneous Interrupt 4. Setting the DAI_IMSK_RE.MISCINT4 bit unmasks the interrupt for the rising edge of the routed signal.                                                                                        |
| 25 (R/W)           | MISCINT3   | Miscellaneous Interrupt 3. Setting the DAI_IMSK_RE.MISCINT3 bit unmasks the interrupt for the rising edge of the routed signal.                                                                                        |
| 24 (R/W)           | MISCINT2   | Miscellaneous Interrupt 2. Setting the DAI_IMSK_RE.MISCINT2 bit unmasks the interrupt for the rising edge of the routed signal.                                                                                        |
| 23 (R/W)           | MISCINT1   | Miscellaneous Interrupt 1. Setting the DAI_IMSK_RE.MISCINT1 bit unmasks the interrupt for the rising edge of the routed signal.                                                                                        |
| 22 (R/W)           | MISCINT0   | Miscellaneous Interrupt 0. Setting the DAI_IMSK_RE.MISCINT0 bit unmasks the interrupt for the rising edge of the routed signal.                                                                                        |
| 21 (R/W)           | SRC3MUTE   | SRC3 Mute. The DAI_IMSK_RE.SRC3MUTE bit masks or unmasks the corresponding SRC mute out interrupt for the rising edge of this interrupt. This interrupt can be generated either entering mute, exiting muting or both. |
| 20 (R/W)           | SRC2MUTE   | SRC2 Mute. The DAI_IMSK_RE.SRC2MUTE bit masks or unmasks the corresponding SRC mute out interrupt for the rising edge of this interrupt. This interrupt can be generated either entering mute, exiting muting or both. |
| 19 (R/W)           | SRC1MUTE   | SRC1 Mute. The DAI_IMSK_RE.SRC1MUTE bit masks or unmasks the corresponding SRC mute out interrupt for the rising edge of this interrupt. This interrupt can be generated either entering mute, exiting muting or both. |

Table 33-47: DAI\_IMSK\_RE Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                 |
|--------------------|--------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (R/W)           | SRC0MUTE     | SRC0 Mute. The DAI_IMSK_RE.SRC0MUTE bit masks or unmasks the corresponding SRC mute out interrupt for the rising edge of this interrupt. This interrupt can be generated either entering mute, exiting muting or both.                                  |
| 4 (R/W)            | RXNONAUDIO   | Receive Non Audio. The DAI_IMSK_RE.RXNONAUDIO bit masks or unmasks the non audio frame mode interrupt for the rising edge of this interrupt. If the channel status indicates non- PCM audio, the NONAUDIO bit flag is set.                              |
| 2 (R/W)            | RXLOSSOFLOCK | Receive Loss of Lock. The DAI_IMSK_RE.RXLOSSOFLOCK bit masks or unmasks the emphasis loss of lock interrupt for the rising edge of this interrupt. The loss of lock status is set by the S/PDIF receiver if receiver looses the lock of biphase stream. |
| 1 (R/W)            | RXLOCK       | Receive Lock. The DAI_IMSK_RE.RXLOCK bit masks or unmasks the S/PDIF receiver lock for the rising edge of this interrupt. This interrupt occurs when the S/PDIF receiver locks to the S/PDIF stream.                                                    |
| 0 (R/W)            | RXVALID      | Receive Valid. Setting the DAI_IMSK_RE.RXVALID bit unmasks the interrupt for the rising edge of the routed signal. This interrupt is set based on whether data received by the S/ PDIF is linear PCM or non-linear audio data.                          |

## High Priority Interrupt Latch Register

The DAI\_IRPTL\_H register holds the high priority latched interrupt status for interrupt requests that have been unmasked (enabled) by the DAI\_IMSK\_FE , DAI\_IMSK\_RE registers and mapped to the INTR\_DAI\_IRQH signal in the SEC by DAI\_IMSK\_PRI registers. If a bit in this register is already set and the corresponding interrupt is masked in the DAI\_IMSK\_FE , DAI\_IMSK\_RE or DAI\_IMSK\_PRI registers, the latch holds its old value, leaving the interrupt asserted until it is reset by software with a W1C operation.

Figure 33-37: DAI\_IRPTL\_H Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000034_0b357b7b2ff6e0a53879fd50633c7b6287831b3cd41a15fe56c0170e087e59cf.png)

Table 33-48: DAI\_IRPTL\_H Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (RC/NW)         | MISCINT9   | Miscellaneous Interrupt 9. The DAI_IRPTL_H.MISCINT9 bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register. |
| 30 (RC/NW)         | MISCINT8   | Miscellaneous Interrupt 8. The DAI_IRPTL_H.MISCINT8 bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register. |

Table 33-48: DAI\_IRPTL\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (RC/NW)         | MISCINT7   | Miscellaneous Interrupt 7. The DAI_IRPTL_H.MISCINT7 bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register. |
| 28 (RC/NW)         | MISCINT6   | Miscellaneous Interrupt 6. The DAI_IRPTL_H.MISCINT6 bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register. |
| 27 (RC/NW)         | MISCINT5   | Miscellaneous Interrupt 5. The DAI_IRPTL_H.MISCINT5 bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register. |
| 26 (RC/NW)         | MISCINT4   | Miscellaneous Interrupt 4. The DAI_IRPTL_H.MISCINT4 bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register. |
| 25 (RC/NW)         | MISCINT3   | Miscellaneous Interrupt 3. The DAI_IRPTL_H.MISCINT3 bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register. |
| 24 (RC/NW)         | MISCINT2   | Miscellaneous Interrupt 2. The DAI_IRPTL_H.MISCINT2 bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register. |
| 23 (RC/NW)         | MISCINT1   | Miscellaneous Interrupt 1. The DAI_IRPTL_H.MISCINT1 bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register. |
| 22 (RC/NW)         | MISCINT0   | Miscellaneous Interrupt 0. The DAI_IRPTL_H.MISCINT0 bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register. |
| 21 (RC/NW)         | SRC3MUTE   | SRC3 Mute. The DAI_IRPTL_H.SRC3MUTE bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register.                 |
| 20 (RC/NW)         | SRC2MUTE   | SRC2 Mute. The DAI_IRPTL_H.SRC2MUTE bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register.                 |
| 19 (RC/NW)         | SRC1MUTE   | SRC1 Mute. The DAI_IRPTL_H.SRC1MUTE bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register.                 |

Table 33-48: DAI\_IRPTL\_H Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                          |
|--------------------|--------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (RC/NW)         | SRC0MUTE     | SRC0 Mute. The DAI_IRPTL_H.SRC0MUTE bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register.                |
| 4 (RC/NW)          | RXNONAUDIO   | Receive Non Audio. The DAI_IRPTL_H.RXNONAUDIO bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register.      |
| 2 (RC/NW)          | RXLOSSOFLOCK | Receive Loss of Lock. The DAI_IRPTL_H.RXLOSSOFLOCK bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register. |
| 1 (RC/NW)          | RXLOCK       | Receive Lock. The DAI_IRPTL_H.RXLOCK bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register.               |
| 0 (RC/NW)          | RXVALID      | Receive Valid. The DAI_IRPTL_H.RXVALID bit is set if the interrupt is mapped to the INTR_DAI_IRQH signal in the SEC using the DAI_IMSK_PRI register.             |

## Shadow High Priority Interrupt Latch Register

The DAI\_IRPTL\_HS register is the shadow register of the DAI\_IRPTL\_H register. Its content is the same as the DAI\_IRPTL\_H register. Reading the DAI\_IRPTL\_HS register does not affect its contents while reading the contents of the DAI\_IRPTL\_H registers clears it.

Figure 33-38: DAI\_IRPTL\_HS Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000035_a231fa1cf13f603213355225eb5df2bb123271b195084bbd774211f07b28986b.png)

Table 33-49: DAI\_IRPTL\_HS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/NW)          | MISCINT9   | Miscellaneous Interrupt 9. The DAI_IRPTL_HS.MISCINT9 bit is the shadow of the DAI_IRPTL_H.MISCINT9 bit and contains the same content. Reading the DAI_IRPTL_HS.MISCINT9 bit does not affect its contents while reading the DAI_IRPTL_H.MISCINT9 bit clears it. |
| 30 (R/NW)          | MISCINT8   | Miscellaneous Interrupt 8. The DAI_IRPTL_HS.MISCINT8 bit is the shadow of the DAI_IRPTL_H.MISCINT8 bit and contains the same content. Reading the DAI_IRPTL_HS.MISCINT8 bit does not affect its contents while reading the DAI_IRPTL_H.MISCINT8 bit clears it. |

Table 33-49: DAI\_IRPTL\_HS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/NW)          | MISCINT7   | Miscellaneous Interrupt 7. The DAI_IRPTL_HS.MISCINT7 bit is the shadow of the DAI_IRPTL_H.MISCINT7 bit and contains the same content. Reading the DAI_IRPTL_HS.MISCINT7 bit does not affect its contents while reading the DAI_IRPTL_H.MISCINT7 bit clears it. |
| 28 (R/NW)          | MISCINT6   | Miscellaneous Interrupt 6. The DAI_IRPTL_HS.MISCINT6 bit is the shadow of the DAI_IRPTL_H.MISCINT6 bit and contains the same content. Reading the DAI_IRPTL_HS.MISCINT6 bit does not affect its contents while reading the DAI_IRPTL_H.MISCINT6 bit clears it. |
| 27 (R/NW)          | MISCINT5   | Miscellaneous Interrupt 5. The DAI_IRPTL_HS.MISCINT5 bit is the shadow of the DAI_IRPTL_H.MISCINT5 bit and contains the same content. Reading the DAI_IRPTL_HS.MISCINT5 bit does not affect its contents while reading the DAI_IRPTL_H.MISCINT5 bit clears it. |
| 26 (R/NW)          | MISCINT4   | Miscellaneous Interrupt 4. The DAI_IRPTL_HS.MISCINT4 bit is the shadow of the DAI_IRPTL_H.MISCINT4 bit and contains the same content. Reading the DAI_IRPTL_HS.MISCINT4 bit does not affect its contents while reading the DAI_IRPTL_H.MISCINT4 bit clears it. |
| 25 (R/NW)          | MISCINT3   | Miscellaneous Interrupt 3. The DAI_IRPTL_HS.MISCINT3 bit is the shadow of the DAI_IRPTL_H.MISCINT3 bit and contains the same content. Reading the DAI_IRPTL_HS.MISCINT3 bit does not affect its contents while reading the DAI_IRPTL_H.MISCINT3 bit clears it. |
| 24 (R/NW)          | MISCINT2   | Miscellaneous Interrupt 2. The DAI_IRPTL_HS.MISCINT2 bit is the shadow of the DAI_IRPTL_H.MISCINT2 bit and contains the same content. Reading the DAI_IRPTL_HS.MISCINT2 bit does not affect its contents while reading the DAI_IRPTL_H.MISCINT2 bit clears it. |
| 23 (R/NW)          | MISCINT1   | Miscellaneous Interrupt 1. The DAI_IRPTL_HS.MISCINT1 bit is the shadow of the DAI_IRPTL_H.MISCINT1 bit and contains the same content. Reading the DAI_IRPTL_HS.MISCINT1 bit does not affect its contents while reading the DAI_IRPTL_H.MISCINT1 bit clears it. |

Table 33-49: DAI\_IRPTL\_HS Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                        |
|--------------------|--------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 22 (R/NW)          | MISCINT0     | Miscellaneous Interrupt 0. The DAI_IRPTL_HS.MISCINT0 bit is the shadow of the DAI_IRPTL_H.MISCINT0 bit and contains the same content. Reading the DAI_IRPTL_HS.MISCINT0 bit does not affect its contents while reading the DAI_IRPTL_H.MISCINT0 bit clears it. |
| 21 (R/NW)          | SRC3MUTE     | SRC3 Mute. The DAI_IRPTL_HS.SRC3MUTE bit is the shadow of the DAI_IRPTL_H.SRC3MUTE bit and contains the same content. Reading the DAI_IRPTL_HS.SRC3MUTE bit does not affect its contents while reading the DAI_IRPTL_H.SRC3MUTE bit clears it.                 |
| 20 (R/NW)          | SRC2MUTE     | SRC2 Mute. The DAI_IRPTL_HS.SRC2MUTE bit is the shadow of the DAI_IRPTL_H.SRC2MUTE bit and contains the same content. Reading the DAI_IRPTL_HS.SRC2MUTE bit does not affect its contents while reading the DAI_IRPTL_H.SRC2MUTE bit clears it.                 |
| 19 (R/NW)          | SRC1MUTE     | SRC1 Mute. The DAI_IRPTL_HS.SRC1MUTE bit is the shadow of the DAI_IRPTL_H.SRC1MUTE bit and contains the same content. Reading the DAI_IRPTL_HS.SRC1MUTE bit does not affect its contents while reading the DAI_IRPTL_H.SRC1MUTE bit clears it.                 |
| 18 (R/NW)          | SRC0MUTE     | SRC0 Mute. The DAI_IRPTL_HS.SRC0MUTE bit is the shadow of the DAI_IRPTL_H.SRC0MUTE bit and contains the same content. Reading the DAI_IRPTL_HS.SRC0MUTE bit does not affect its contents while reading the DAI_IRPTL_H.SRC0MUTE bit clears it.                 |
| 4 (R/NW)           | RXNONAUDIO   | Receive Non Audio. The DAI_IRPTL_HS.RXNONAUDIO bit is the shadow of DAI_IRPTL_H.RXNONAUDIO bit and contains the same content. Reading this bit does not affect its content.                                                                                    |
| 2 (R/NW)           | RXLOSSOFLOCK | Receive Loss of Lock. The DAI_IRPTL_HS.RXLOSSOFLOCK bit is the shadow of DAI_IRPTL_H.RXLOSSOFLOCK bit and contains the same content. Reading this bit does not affect its content.                                                                             |
| 1 (R/NW)           | RXLOCK       | Receive Lock. The DAI_IRPTL_HS.RXLOCK bit is the shadow of DAI_IRPTL_H.RXLOCK bit and contains the same content. Reading this bit does not affect its content.                                                                                                 |

Table 33-49: DAI\_IRPTL\_HS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/NW)           | RXVALID    | Receive Valid. Setting the DAI_IRPTL_HS.RXVALID bit is the shadow of DAI_IRPTL_H.RXVALID bit and contains the same content. Reading this bit does not affect its content. |

## Low Priority Interrupt Latch Register

The DAI\_IRPTL\_L register holds the low priority latched interrupt status for interrupt requests that have been unmasked (enabled) by the DAI\_IMSK\_FE , DAI\_IMSK\_RE or DAI\_IMSK\_PRI registers. If a bit in DAI\_IRPTL\_L is already set and the corresponding interrupt is masked in DAI\_IMSK\_FE , DAI\_IMSK\_RE or DAI\_IMSK\_PRI registers, the latch holds its old value, leaving the interrupt asserted until it is reset by software with a W1C operation.

Figure 33-39: DAI\_IRPTL\_L Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000036_0b357b7b2ff6e0a53879fd50633c7b6287831b3cd41a15fe56c0170e087e59cf.png)

Table 33-50: DAI\_IRPTL\_L Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (RC/NW)         | MISCINT9   | Miscellaneous Interrupt 9. The DAI_IRPTL_L.MISCINT9 bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register. |
| 30 (RC/NW)         | MISCINT8   | Miscellaneous Interrupt 8. The DAI_IRPTL_L.MISCINT8 bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register. |

Table 33-50: DAI\_IRPTL\_L Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (RC/NW)         | MISCINT7   | Miscellaneous Interrupt 7. The DAI_IRPTL_L.MISCINT7 bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register. |
| 28 (RC/NW)         | MISCINT6   | Miscellaneous Interrupt 6. The DAI_IRPTL_L.MISCINT6 bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register. |
| 27 (RC/NW)         | MISCINT5   | Miscellaneous Interrupt 5. The DAI_IRPTL_L.MISCINT5 bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register. |
| 26 (RC/NW)         | MISCINT4   | Miscellaneous Interrupt 4. The DAI_IRPTL_L.MISCINT4 bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register. |
| 25 (RC/NW)         | MISCINT3   | Miscellaneous Interrupt 3. The DAI_IRPTL_L.MISCINT3 bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register. |
| 24 (RC/NW)         | MISCINT2   | Miscellaneous Interrupt 2. The DAI_IRPTL_L.MISCINT2 bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register. |
| 23 (RC/NW)         | MISCINT1   | Miscellaneous Interrupt 1. The DAI_IRPTL_L.MISCINT1 bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register. |
| 22 (RC/NW)         | MISCINT0   | Miscellaneous Interrupt 0. The DAI_IRPTL_L.MISCINT0 bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register. |
| 21 (RC/NW)         | SRC3MUTE   | SRC3 Mute. The DAI_IRPTL_L.SRC3MUTE bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register.                 |
| 20 (RC/NW)         | SRC2MUTE   | SRC2 Mute. The DAI_IRPTL_L.SRC2MUTE bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register.                 |
| 19 (RC/NW)         | SRC1MUTE   | SRC1 Mute. The DAI_IRPTL_L.SRC1MUTE bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register.                 |

Table 33-50: DAI\_IRPTL\_L Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                          |
|--------------------|--------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (RC/NW)         | SRC0MUTE     | SRC0 Mute. The DAI_IRPTL_L.SRC0MUTE bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register.                |
| 4 (RC/NW)          | RXNONAUDIO   | Receive Non Audio. The DAI_IRPTL_L.RXNONAUDIO bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register.      |
| 2 (RC/NW)          | RXLOSSOFLOCK | Receive Loss of Lock. The DAI_IRPTL_L.RXLOSSOFLOCK bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register. |
| 1 (RC/NW)          | RXLOCK       | Receive Lock. The DAI_IRPTL_L.RXLOCK bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register.               |
| 0 (RC/NW)          | RXVALID      | Receive Valid. The DAI_IRPTL_L.RXVALID bit is set if the interrupt is mapped to the INTR_DAI_IRQL signal in the SEC using the DAI_IMSK_PRI register.             |

## Shadow Low Priority Interrupt Latch Register

The DAI\_IRPTL\_LS register is the shadow register of the DAI\_IRPTL\_L register. Its content is the same as the DAI\_IRPTL\_L register. Reading the DAI\_IRPTL\_LS register does not affect its contents while reading the contents of the DAI\_IRPTL\_L registers clears it.

Figure 33-40: DAI\_IRPTL\_LS Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000037_0ab1e7a6a671f44f2c7734cddc806e2b7a1eb271a76585064716771400e6d7b3.png)

Table 33-51: DAI\_IRPTL\_LS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/NW)          | MISCINT9   | Miscellaneous Interrupt 9. The DAI_IRPTL_LS.MISCINT9 bit is the shadow of the DAI_IRPTL_L.MISCINT9 bit and contains the same content. Reading the DAI_IRPTL_LS.MISCINT9 bit does not affect its contents while reading the DAI_IRPTL_L.MISCINT9 bit clears it. |
| 30 (R/NW)          | MISCINT8   | Miscellaneous Interrupt 8. The DAI_IRPTL_LS.MISCINT8 bit is the shadow of the DAI_IRPTL_L.MISCINT8 bit and contains the same content. Reading the DAI_IRPTL_LS.MISCINT8 bit does not affect its contents while reading the DAI_IRPTL_L.MISCINT8 bit clears it. |

Table 33-51: DAI\_IRPTL\_LS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/NW)          | MISCINT7   | Miscellaneous Interrupt 7. The DAI_IRPTL_LS.MISCINT7 bit is the shadow of the DAI_IRPTL_L.MISCINT7 bit and contains the same content. Reading the DAI_IRPTL_LS.MISCINT7 bit does not affect its contents while reading the DAI_IRPTL_L.MISCINT7 bit clears it. |
| 28 (R/NW)          | MISCINT6   | Miscellaneous Interrupt 6. The DAI_IRPTL_LS.MISCINT6 bit is the shadow of the DAI_IRPTL_L.MISCINT6 bit and contains the same content. Reading the DAI_IRPTL_LS.MISCINT6 bit does not affect its contents while reading the DAI_IRPTL_L.MISCINT6 bit clears it. |
| 27 (R/NW)          | MISCINT5   | Miscellaneous Interrupt 5. The DAI_IRPTL_LS.MISCINT5 bit is the shadow of the DAI_IRPTL_L.MISCINT5 bit and contains the same content. Reading the DAI_IRPTL_LS.MISCINT5 bit does not affect its contents while reading the DAI_IRPTL_L.MISCINT5 bit clears it. |
| 26 (R/NW)          | MISCINT4   | Miscellaneous Interrupt 4. The DAI_IRPTL_LS.MISCINT4 bit is the shadow of the DAI_IRPTL_L.MISCINT4 bit and contains the same content. Reading the DAI_IRPTL_LS.MISCINT4 bit does not affect its contents while reading the DAI_IRPTL_L.MISCINT4 bit clears it. |
| 25 (R/NW)          | MISCINT3   | Miscellaneous Interrupt 3. The DAI_IRPTL_LS.MISCINT3 bit is the shadow of the DAI_IRPTL_L.MISCINT3 bit and contains the same content. Reading the DAI_IRPTL_LS.MISCINT3 bit does not affect its contents while reading the DAI_IRPTL_L.MISCINT3 bit clears it. |
| 24 (R/NW)          | MISCINT2   | Miscellaneous Interrupt 2. The DAI_IRPTL_LS.MISCINT2 bit is the shadow of the DAI_IRPTL_L.MISCINT2 bit and contains the same content. Reading the DAI_IRPTL_LS.MISCINT2 bit does not affect its contents while reading the DAI_IRPTL_L.MISCINT2 bit clears it. |
| 23 (R/NW)          | MISCINT1   | Miscellaneous Interrupt 1. The DAI_IRPTL_LS.MISCINT1 bit is the shadow of the DAI_IRPTL_L.MISCINT1 bit and contains the same content. Reading the DAI_IRPTL_LS.MISCINT1 bit does not affect its contents while reading the DAI_IRPTL_L.MISCINT1 bit clears it. |

Table 33-51: DAI\_IRPTL\_LS Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                         |
|--------------------|--------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 22 (R/NW)          | MISCINT0     | Miscellaneous Interrupt 0. The DAI_IRPTL_LS.MISCINT0 bit is the shadow of the DAI_IRPTL_L.MISCINT0 bit and contains the same content. Reading the DAI_IRPTL_LS.MISCINT0 bit does not affect its contents while reading the DAI_IRPTL_L.MISCINT0 bit clears it.  |
| 21 (R/NW)          | SRC3MUTE     | SRC3 Mute. The DAI_IRPTL_LS.SRC3MUTE bit is the shadow of the DAI_IRPTL_L.SRC3MUTE bit and contains the same content. Reading the DAI_IRPTL_LS.SRC3MUTE bit does not affect its contents while reading the DAI_IRPTL_L.SRC3MUTE bit clears it.                  |
| 20 (R/NW)          | SRC2MUTE     | SRC2 Mute. The DAI_IRPTL_LS.SRC2MUTE bit is the shadow of the DAI_IRPTL_L.SRC2MUTE bit and contains the same content. Reading the DAI_IRPTL_LS.SRC2MUTE bit does not affect its contents while reading the DAI_IRPTL_L.SRC2MUTE bit clears it.                  |
| 19 (R/NW)          | SRC1MUTE     | SRC1 Mute. The DAI_IRPTL_LS.SRC1MUTE bit is the shadow of the DAI_IRPTL_L.SRC1MUTE bit and contains the same content. Reading the DAI_IRPTL_LS.SRC1MUTE bit does not affect its contents while reading the DAI_IRPTL_L.SRC1MUTE bit clears it.                  |
| 18 (R/NW)          | SRC0MUTE     | SRC0 Mute. The DAI_IRPTL_LS.SRC0MUTE bit is the shadow of the DAI_IRPTL_L.SRC0MUTE bit and contains the same content. Reading the DAI_IRPTL_LS.SRC0MUTE bit does not affect its contents while reading the DAI_IRPTL_L.SRC0MUTE bit clears it.                  |
| 4 (R/NW)           | RXNONAUDIO   | Receiver Non Audio. The DAI_IRPTL_LS.RXNONAUDIO bit is the shadow of the DAI_IRPTL_L.RXNONAUDIO bit and contains the same content. Reading the DAI_IRPTL_LS.RXNONAUDIO bit does not affect its contents while reading the DAI_IRPTL_L.RXNONAUDIO bit clears it. |
| 2 (R/NW)           | RXLOSSOFLOCK | Receive Emphasis Loss of Lock. The DAI_IRPTL_LS.RXLOSSOFLOCK bit is the shadow of DAI_IRPTL_L.RXLOSSOFLOCK bit and contains the same content. Reading this bit does not affect its content. Reading the DAI_IRPTL_L.RXLOSSOFLOCK bit clears it.                 |
| 1 (R/NW)           | RXLOCK       | Receive Error Lock. The DAI_IRPTL_LS.RXLOCK bit is the shadow of DAI_IRPTL_L.RXLOCK bit and contains the same content. Reading this bit does not affect its content. Reading the DAI_IRPTL_L.RXLOCK bit clears it.                                              |

Table 33-51: DAI\_IRPTL\_LS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/NW)           | RXVALID    | Receive Valid. The DAI_IRPTL_LS.RXVALID bit is the shadow of DAI_IRPTL_L.RXVALID bit and contains the same content. Reading this bit does not affect its content. Reading the DAI_IRPTL_L.RXVALID bit clears it. |

## Miscellaneous Control Register 0

The DAI\_MISC0 register allows programs to route to the DAI interrupt latch, PBEN input routing, or input signal inversion. This register belongs to group E which routes control signals and provides a means of connecting signals between groups.

Figure 33-41: DAI\_MISC0 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000038_be1fbcfb5f9c219ab6128a19716d7aa74a3192d50fbfabe2a0e0fb096ddbd05a.png)

Table 33-52: DAI\_MISC0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | INV11      | Invert 11. The DAI_MISC0.INV11 bit field inverts miscellaneous A5 input.                                              |
| 30 (R/W)           | INV10      | Invert 10. The DAI_MISC0.INV10 bit field inverts miscellaneous A4 input.                                              |
| 29:25 (R/W)        | IN11       | Input 11. The DAI_MISC0.IN11 bit field configures miscellaneous A5 input.                                             |
| 24:20 (R/W)        | IN10       | Input 10. The DAI_MISC0.IN10 bit field configures miscellaneous A4 input.                                             |
| 19:15 (R/W)        | IN9        | Input 9. The DAI_MISC0.IN9 bit field configures miscellaneous A3 input/miscellaneous In- terrupt 9(DAI interrupt 31). |
| 14:10 (R/W)        | IN8        | Input 8. The DAI_MISC0.IN8 bit field configures miscellaneous A2 input/miscellaneous In- terrupt 8(DAI interrupt 30). |

Table 33-52: DAI\_MISC0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------|
| 9:5 (R/W)          | IN7        | Input 7. The DAI_MISC0.IN7 bit field configures miscellaneous A1 input/miscellaneous In- terrupt 7(DAI interrupt 29). |
| 4:0 (R/W)          | IN6        | Input 6. The DAI_MISC0.IN6 bit field configures miscellaneous A0 input/miscellaneous In- terrupt 6(DAI interrupt 28). |

## Miscellaneous Control Register 1

The DAI\_MISC1 register allows programs to route to the DAI interrupt latch, PBEN input routing, or input signal inversion. This register belongs to group E which routes control signals and provides a means of connecting signals between groups.

Figure 33-42: DAI\_MISC1 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000039_fb331a6190988b959493796aec60355c101460656dad907037096a94f078d681.png)

Table 33-53: DAI\_MISC1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------|
| 29:25 (R/W)        | IN5        | Input 5. The DAI_MISC1.IN5 bit field configures miscellaneous Interrupt 5(DAI interrupt 27). |
| 24:20 (R/W)        | IN4        | Input 4. The DAI_MISC1.IN4 bit field configures miscellaneous Interrupt 4(DAI interrupt 26). |
| 19:15 (R/W)        | IN3        | Input 3. The DAI_MISC1.IN3 bit field configures miscellaneous Interrupt 3(DAI interrupt 25). |
| 14:10 (R/W)        | IN2        | Input 2. The DAI_MISC1.IN2 bit field configures miscellaneous Interrupt 2(DAI interrupt 24). |
| 9:5 (R/W)          | IN1        | Input 1. The DAI_MISC1.IN1 bit field configures miscellaneous Interrupt 1(DAI interrupt 23). |

Table 33-53: DAI\_MISC1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------|
| 4:0 (R/W)          | IN0        | Input 0. The DAI_MISC1.IN0 bit field configures miscellaneous Interrupt 0(DAI interrupt 22). |

## Pin Buffer Enable Register 0

The DAI\_PBEN0 register routes signals to the pin enables, and the value of these signals determines if a DAI pin is used as an output or an input.

Figure 33-43: DAI\_PBEN0 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000040_f258572a2e6d380e7bd8db2ff1c393cf21dd764babbb2ad4bddecd98c8feabcd.png)

Table 33-54: DAI\_PBEN0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------|
| 29:24 (R/W)        | PB05       | Pin Buffer Enable 5. The DAI_PBEN0.PB05 bit field is the pin buffer enable for DAI port 5. |
| 23:18 (R/W)        | PB04       | Pin Buffer Enable 4. The DAI_PBEN0.PB04 bit field is the pin buffer enable for DAI port 4. |
| 17:12 (R/W)        | PB03       | Pin Buffer Enable 3. The DAI_PBEN0.PB03 bit field is the pin buffer enable for DAI port 3. |
| 11:6 (R/W)         | PB02       | Pin Buffer Enable 2. The DAI_PBEN0.PB02 bit field is the pin buffer enable for DAI port 2. |
| 5:0 (R/W)          | PB01       | Pin Buffer Enable 1. The DAI_PBEN0.PB01 bit field is the pin buffer enable for DAI port 1. |

## Pin Buffer Enable Register 1

The DAI\_PBEN1 register routes signals to the pin enables, and the value of these signals determines if a DAI pin is used as an output or an input.

Figure 33-44: DAI\_PBEN1 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000041_6b9bc0fb39fb72a37b8475e37b2ff8f90074e63b0eaed0fd01242e169e5b7e54.png)

Table 33-55: DAI\_PBEN1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------|
| 29:24 (R/W)        | PB10       | Pin Buffer Enable 10. The DAI_PBEN1.PB10 bit field is the pin buffer enable for DAI port 10. |
| 23:18 (R/W)        | PB09       | Pin Buffer Enable 9. The DAI_PBEN1.PB09 bit field is the pin buffer enable for DAI port 9.   |
| 17:12 (R/W)        | PB08       | Pin Buffer Enable 8. The DAI_PBEN1.PB08 bit field is the pin buffer enable for DAI port 8.   |
| 11:6 (R/W)         | PB07       | Pin Buffer Enable 7. The DAI_PBEN1.PB07 bit field is the pin buffer enable for DAI port 7.   |
| 5:0 (R/W)          | PB06       | Pin Buffer Enable 6. The DAI_PBEN1.PB06 bit field is the pin buffer enable for DAI port 6.   |

## Pin Buffer Enable Register 2

The DAI\_PBEN2 register routes signals to the pin enables, and the value of these signals determines if a DAI pin is used as an output or an input.

Figure 33-45: DAI\_PBEN2 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000042_df9f3e3c8bf3124c0d5cc302b65a47fa5ca060515b792c5b8f76e9eb33ebff2a.png)

Table 33-56: DAI\_PBEN2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------|
| 29:24 (R/W)        | PB15       | Pin Buffer Enable 15. The DAI_PBEN2.PB15 bit field is the pin buffer enable for DAI port 15. |
| 23:18 (R/W)        | PB14       | Pin Buffer Enable 14. The DAI_PBEN2.PB14 bit field is the pin buffer enable for DAI port 14. |
| 17:12 (R/W)        | PB13       | Pin Buffer Enable 13. The DAI_PBEN2.PB13 bit field is the pin buffer enable for DAI port 13. |
| 11:6 (R/W)         | PB12       | Pin Buffer Enable 12. The DAI_PBEN2.PB12 bit field is the pin buffer enable for DAI port 12. |
| 5:0 (R/W)          | PB11       | Pin Buffer Enable 11. The DAI_PBEN2.PB11 bit field is the pin buffer enable for DAI port 11. |

## Pin Buffer Enable Register 3

The DAI\_PBEN3 register routes signals to the pin enables, and the value of these signals determines if a DAI pin is used as an output or an input.

Figure 33-46: DAI\_PBEN3 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000043_949d73bb140887e803f401f33cfb62d93a7cc03201f2654d5bc270b8d69abcc7.png)

Table 33-57: DAI\_PBEN3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------|
| 29:24 (R/W)        | PB20       | Pin Buffer Enable 20. The DAI_PBEN3.PB20 bit field is the pin buffer enable for DAI port 20. |
| 23:18 (R/W)        | PB19       | Pin Buffer Enable 19. The DAI_PBEN3.PB19 bit field is the pin buffer enable for DAI port 19. |
| 17:12 (R/W)        | PB18       | Pin Buffer Enable 18. The DAI_PBEN3.PB18 bit field is the pin buffer enable for DAI port 18. |
| 11:6 (R/W)         | PB17       | Pin Buffer Enable 17. The DAI_PBEN3.PB17 bit field is the pin buffer enable for DAI port 17. |
| 5:0 (R/W)          | PB16       | Pin Buffer Enable 16. The DAI_PBEN3.PB16 bit field is the pin buffer enable for DAI port 16. |

## Pin Buffer Assignment Register 0

The DAI\_PIN0 register routes physical pins that are connected to a bonded pad.

Figure 33-47: DAI\_PIN0 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000044_b0634da6e66750ba44d94c292cec281149123fe29e4ee5b9f98431cf1f1ea791.png)

Table 33-58: DAI\_PIN0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 27:21 (R/W)        | PB04       | Pin Buffer 4 Input. DAI_PIN0.PB04 holds the Source signal assignment that will be routed to the DAI_PIN0.PB04 Destination. Refer to the Group DSignals table for Source and Destination mappings. |
| 20:14 (R/W)        | PB03       | Pin Buffer 3 Input. DAI_PIN0.PB03 holds the Source signal assignment that will be routed to the DAI_PIN0.PB03 Destination. Refer to the Group DSignals table for Source and Destination mappings. |
| 13:7 (R/W)         | PB02       | Pin Buffer 2 Input. DAI_PIN0.PB02 holds the Source signal assignment that will be routed to the DAI_PIN0.PB02 Destination. Refer to the Group DSignals table for Source and Destination mappings. |
| 6:0 (R/W)          | PB01       | Pin Buffer 1 Input. DAI_PIN0.PB01 holds the Source signal assignment that will be routed to the DAI_PIN0.PB01 Destination. Refer to the Group DSignals table for Source and Destination mappings. |

## Pin Buffer Assignment Register 1

The DAI\_PIN1 register routes physical pins that are connected to a bonded pad.

Figure 33-48: DAI\_PIN1 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000045_b55ba0654694446084a405a0fbf2d61026fec2bc6a668d86795c5ffff3de0320.png)

Table 33-59: DAI\_PIN1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 27:21 (R/W)        | PB08       | Pin Buffer 8 Input. DAI_PIN1.PB08 holds the Source signal assignment that will be routed to the DAI_PIN1.PB08 Destination. Refer to the Group DSignals table for Source and Destination mappings. |
| 20:14 (R/W)        | PB07       | Pin Buffer 7 Input. DAI_PIN1.PB07 holds the Source signal assignment that will be routed to the DAI_PIN1.PB07 Destination. Refer to the Group DSignals table for Source and Destination mappings. |
| 13:7 (R/W)         | PB06       | Pin Buffer 6 Input. DAI_PIN1.PB06 holds the Source signal assignment that will be routed to the DAI_PIN1.PB06 Destination. Refer to the Group DSignals table for Source and Destination mappings. |
| 6:0 (R/W)          | PB05       | Pin Buffer 5 Input. DAI_PIN1.PB05 holds the Source signal assignment that will be routed to the DAI_PIN1.PB05 Destination. Refer to the Group DSignals table for Source and Destination mappings. |

## Pin Buffer Assignment Register 2

The DAI\_PIN2 register routes physical pins that are connected to a bonded pad.

Figure 33-49: DAI\_PIN2 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000046_356d2727eb8d221f4c48a02eaa352c0189b67e3f3061600918c9d30ce94dbf07.png)

Table 33-60: DAI\_PIN2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 27:21 (R/W)        | PB12       | Pin Buffer 12 Input. DAI_PIN2.PB12 holds the Source signal assignment that will be routed to the DAI_PIN2.PB12 Destination. Refer to the Group DSignals table for Source and Destination mappings. |
| 20:14 (R/W)        | PB11       | Pin Buffer 11 Input. DAI_PIN2.PB11 holds the Source signal assignment that will be routed to the DAI_PIN2.PB11 Destination. Refer to the Group DSignals table for Source and Destination mappings. |
| 13:7 (R/W)         | PB10       | Pin Buffer 10 Input. DAI_PIN2.PB10 holds the Source signal assignment that will be routed to the DAI_PIN2.PB10 Destination. Refer to the Group DSignals table for Source and Destination mappings. |
| 6:0 (R/W)          | PB09       | Pin Buffer 9 Input. DAI_PIN2.PB09 holds the Source signal assignment that will be routed to the DAI_PIN2.PB09 Destination. Refer to the Group DSignals table for Source and Destination mappings.  |

## Pin Buffer Assignment Register 3

The DAI\_PIN3 register routes physical pins that are connected to a bonded pad.

Figure 33-50: DAI\_PIN3 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000047_eb4c91565461f9ff11b6ea107adf6052f156a50912b8f930e60d6eddd8662491.png)

Table 33-61: DAI\_PIN3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 27:21 (R/W)        | PB16       | Pin Buffer 16 Input. The DAI_PIN3.PB16 bit field is the pin buffer 16 input.                                                                                                                       |
| 20:14 (R/W)        | PB15       | Pin Buffer 15 Input. DAI_PIN3.PB15 holds the Source signal assignment that will be routed to the DAI_PIN3.PB15 Destination. Refer to the Group DSignals table for Source and Destination mappings. |
| 13:7 (R/W)         | PB14       | Pin Buffer 14 Input. DAI_PIN3.PB14 holds the Source signal assignment that will be routed to the DAI_PIN3.PB14 Destination. Refer to the Group DSignals table for Source and Destination mappings. |
| 6:0 (R/W)          | PB13       | Pin Buffer 13 Input. DAI_PIN3.PB13 holds the Source signal assignment that will be routed to the DAI_PIN3.PB13 Destination. Refer to the Group DSignals table for Source and Destination mappings. |

## Pin Buffer Assignment Register 4

The DAI\_PIN4 register routes physical pins that are connected to a bonded pad.

Figure 33-51: DAI\_PIN4 Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000048_1e7337497e47b16ddceb5611280bbe004b3dff1e6ad9361e151c8b45680b5571.png)

Table 33-62: DAI\_PIN4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W)           | INV20      | Pin Buffer 20 Invert. DAI_PIN4.INV20 holds the Source signal assignment that will be routed to the DAI_PIN4.INV20 Destination. Refer to the Group DSignals table for Source and Destination mappings. |
| 28 (R/W)           | INV19      | Pin Buffer 19 Invert. DAI_PIN4.INV19 holds the Source signal assignment that will be routed to the DAI_PIN4.INV19 Destination. Refer to the Group DSignals table for Source and Destination mappings. |
| 27:21 (R/W)        | PB20       | Pin Buffer 20 Input. DAI_PIN4.PB20 holds the Source signal assignment that will be routed to the DAI_PIN4.PB20 Destination. Refer to the Group DSignals table for Source and Destination mappings.    |
| 20:14 (R/W)        | PB19       | Pin Buffer 19 Input. DAI_PIN4.PB19 holds the Source signal assignment that will be routed to the DAI_PIN4.PB19 Destination. Refer to the Group DSignals table for Source and Destination mappings.    |
| 13:7 (R/W)         | PB18       | Pin Buffer 18 Input. DAI_PIN4.PB18 holds the Source signal assignment that will be routed to the DAI_PIN4.PB18 Destination. Refer to the Group DSignals table for Source and Destination mappings.    |

Table 33-62: DAI\_PIN4 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:0 (R/W)          | PB17       | Pin Buffer 17 Input. DAI_PIN4.PB17 holds the Source signal assignment that will be routed to the DAI_PIN4.PB17 Destination. Refer to the Group DSignals table for Source and Destination mappings. |

## Pin Status Register

The DAI\_PIN\_STAT register bits indicate the status (signal high or low) for each pin. The individual bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low.

Figure 33-52: DAI\_PIN\_STAT Register Diagram

![Image](36_Digital_Audio_Interface_(DAI)_artifacts/image_000049_ff256562a14cd964c9ed8921b217e5ea65c065da00b2f38952639d4714b25cdc.png)

Table 33-63: DAI\_PIN\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 19 (R/NW)          | PB20       | Pin Buffer 20 Status. The DAI_PIN_STAT.PB20 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |
| 18 (R/NW)          | PB19       | Pin Buffer 19 Status. The DAI_PIN_STAT.PB19 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |

Table 33-63: DAI\_PIN\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/NW)          | PB18       | Pin Buffer 18 Status. The DAI_PIN_STAT.PB18 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |
| 16 (R/NW)          | PB17       | Pin Buffer 17 Status. The DAI_PIN_STAT.PB17 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |
| 15 (R/NW)          | PB16       | Pin Buffer 16 Status. The DAI_PIN_STAT.PB16 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |
| 14 (R/NW)          | PB15       | Pin Buffer 15 Status. The DAI_PIN_STAT.PB15 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |
| 13 (R/NW)          | PB14       | Pin Buffer 14 Status. The DAI_PIN_STAT.PB14 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |
| 12 (R/NW)          | PB13       | Pin Buffer 13 Status. The DAI_PIN_STAT.PB13 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |
| 11 (R/NW)          | PB12       | Pin Buffer 12 Status. The DAI_PIN_STAT.PB12 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |
| 10 (R/NW)          | PB11       | Pin Buffer 11 Status. The DAI_PIN_STAT.PB11 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |
| 9 (R/NW)           | PB10       | Pin Buffer 10 Status. The DAI_PIN_STAT.PB10 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |
| 8 (R/NW)           | PB09       | Pin Buffer 09 Status. The DAI_PIN_STAT.PB09 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |
| 7 (R/NW)           | PB08       | Pin Buffer 08 Status. The DAI_PIN_STAT.PB08 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |

Table 33-63: DAI\_PIN\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/NW)           | PB07       | Pin Buffer 07 Status. The DAI_PIN_STAT.PB07 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |
| 5 (R/NW)           | PB06       | Pin Buffer 06 Status. The DAI_PIN_STAT.PB06 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |
| 4 (R/NW)           | PB05       | Pin Buffer 05 Status. The DAI_PIN_STAT.PB05 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |
| 3 (R/NW)           | PB04       | Pin Buffer 04 Status. The DAI_PIN_STAT.PB04 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |
| 2 (R/NW)           | PB03       | Pin Buffer 03 Status. The DAI_PIN_STAT.PB03 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |
| 1 (R/NW)           | PB02       | Pin Buffer 02 Status. The DAI_PIN_STAT.PB02 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |
| 0 (R/NW)           | PB01       | Pin Buffer 01 Status. The DAI_PIN_STAT.PB01 bit reads 1 if the signal to this pin is high and reads 0 if signal to this pin is low. |