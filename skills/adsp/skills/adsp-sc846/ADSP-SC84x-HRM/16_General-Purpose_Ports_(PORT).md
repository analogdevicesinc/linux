## 13   General-Purpose Ports (PORT)

This section describes general-purpose ports, pin multiplexing, general-purpose input/output (GPIO) functionality, and pin interrupts. The general-purpose ports provide the following three functions:

- Pin multiplexing scheme
- GPIO functionality
- Pin interrupt requests

NOTE: In this chapter, the naming convention for registers and bits omits the alphabetic group enumeration to refer to any and all of the ports. For example, PORT\_FER represents registers PORTA\_FER , PORTB\_FER , and so on. Likewise PORT\_FER.PX1 represents bits PA1, PB1, and so on.

Figure 13-1: Simplified GPIO and Pin Interrupt Signal Flow

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000000_b4cc48f57567ba6e1ed7a855dd2dc5ec7d614c557b886d1da105432f8fdb1097.png)

## PORT Features

The PORTs include the following features:

- Input mode, output mode, and open-drain mode of GPIO operation
- Port multiplexing controlled on a pin-by-pin basis
- No external glue hardware required for unused pins
- All port pins provide interrupt request functionality
- Byte-wide pin-to-interrupt request assignment

## PORT Functional Description

The number of ports and their composition is defined in the processor data sheet. Each port has a dedicated set of MMR registers that control pin functions and operates in general-purpose I/O (GPIO) mode by default, as controlled by the port-specific PORT\_FER register. Each bit in this register, as well as the other PORT MMRs, represents a specific GPIO pin on the specified port.

## Input Mode, Output Mode, and Open-Drain Mode of GPIO Operation

At reset, every GPIO pin defaults to input mode with the input drivers disabled. To enable any GPIO input driver, set the bits corresponding to the individual pins in the appropriate input enable register ( PORT\_INEN ).

The GPIO output drivers are enabled by setting the corresponding bits in the direction registers ( PORT\_DIR ).

The PORT can use every GPIO in open-drain mode by clearing the respective bit in the PORT\_DATA register or setting the respective bit in the PORT\_DATA\_CLR register. Then, set the corresponding bit in the PORT\_INEN register. Read from the PORT\_DATA register to obtain the status from the pin.

## Port Multiplexing Controlled on Pin-by-Pin Basis

Each port has two dedicated MMRs that control the port multiplexing, the 16-bit function enable ( PORT\_FER ) registers and the 32-bit port multiplexing ( PORT\_MUX ) registers.

## All Port Pins Provide Interrupt Functionality

Pin interrupts are completely decoupled from GPIO functionality. Pins are connected directly via the PINTx modules, each of which is configurable in terms of which port pins are sensed for interrupt generation.

## ADSP-2184x PORT Register List

The PORT module (PORT) regulates the use of the multiplexable processor pins. Every port pin can operate in general-purpose I/O (GPIO) mode or as an alternate function. This GPIO operation is the default after processor reset and is controlled by a set of registers that control GPIO functionality. Every bit in these registers represents a certain GPIO pin of a specific port. For more information on PORT functionality, see the PORT register descriptions.

Table 13-1: ADSP-2184x PORT Register List

| Name          | Description                                |
|---------------|--------------------------------------------|
| PORT_DATA     | Port x GPIO Data Register                  |
| PORT_DATA_CLR | Port x GPIO Data Clear Register            |
| PORT_DATA_SET | Port x GPIO Data Set Register              |
| PORT_DATA_TGL | Port x GPIO Output Toggle Register         |
| PORT_DIR      | Port x GPIO Direction Register             |
| PORT_DIR_CLR  | Port x GPIO Direction Clear Register       |
| PORT_DIR_SET  | Port x GPIO Direction Set Register         |
| PORT_FER      | Port x Function Enable Register            |
| PORT_FER_CLR  | Port x Function Enable Clear Register      |
| PORT_FER_SET  | Port x Function Enable Set Register        |
| PORT_INEN     | Port x GPIO Input Enable Register          |
| PORT_INEN_CLR | Port x GPIO Input Enable Clear Register    |
| PORT_INEN_SET | Port x GPIO Input Enable Set Register      |
| PORT_LOCK     | Port x GPIO Lock Register                  |
| PORT_MUX      | Port x Multiplexer Control Register        |
| PORT_POL      | Port x GPIO Polarity Invert Register       |
| PORT_POL_CLR  | Port x GPIO Polarity Invert Clear Register |
| PORT_POL_SET  | Port x GPIO Polarity Invert Set Register   |
| PORT_TRIG_TGL | Port x GPIO Trigger Toggle Register        |

## ADSP-2184x PORT Trigger List

Table 13-2: ADSP-2184x PORT Trigger List Generators

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         | None   | None          | None          |

Table 13-3: ADSP-2184x PORT Trigger List Receivers

|   Trigger ID | Name         | Description         | Sensitivity   |
|--------------|--------------|---------------------|---------------|
|          136 | PORTA_TOGGLE | Port Toggle Trigger | Pulse         |
|          137 | PORTB_TOGGLE | Port Toggle Trigger | Pulse         |
|          138 | PORTC_TOGGLE | Port Toggle Trigger | Pulse         |
|          139 | PORTD_TOGGLE | Port Toggle Trigger | Pulse         |
|          140 | PORTE_TOGGLE | Port Toggle Trigger | Pulse         |
|          141 | PORTF_TOGGLE | Port Toggle Trigger | Pulse         |
|          142 | PORTG_TOGGLE | Port Toggle Trigger | Pulse         |
|          143 | PORTH_TOGGLE | Port Toggle Trigger | Pulse         |

## ADSP-2184x PINT Register List

The Pin Interrupt module (PINT) controls the pin-to-interrupt assignment in a byte-wide manner. The pin-interrupt assignment registers do not consist of 32 individual bits. They consist of four control bytes, each functioning as a multiplexer control. For more information, see the PINT register descriptions.

All PINT registers are 32 bits wide and can be accessed by 32-bit load/store instructions. They also support 16-bit operation where the upper 16 bits are ignored and the application uses the lower 16 bits only. Consequently, all PINT registers support 32-bit accesses as well as 16-bit accesses for the lower half words. Applications may use faster 16-bit accesses as long as they do not require functionality of upper register halves.

Table 13-4: ADSP-2184x PINT Register List

| Name          | Description                |
|---------------|----------------------------|
| PINT_ASSIGN   | PINT Assign Register       |
| PINT_EDGE_CLR | PINT Edge Clear Register   |
| PINT_EDGE_SET | PINT Edge Set Register     |
| PINT_INV_CLR  | PINT Invert Clear Register |
| PINT_INV_SET  | PINT Invert Set Register   |
| PINT_LATCH    | PINT Latch Register        |
| PINT_MSK_CLR  | PINT Mask Clear Register   |
| PINT_MSK_SET  | PINT Mask Set Register     |
| PINT_PINSTATE | PINT Pin State Register    |
| PINT_REQ      | PINT Request Register      |

## ADSP-2184x PINT Interrupt List

Table 13-5: ADSP-2184x PINT Interrupt List

|   Interrupt ID | Name        | Description                 | Sensitivity   | DMA Channel   |
|----------------|-------------|-----------------------------|---------------|---------------|
|            217 | PINT0_BLOCK | PINT0 Pin Interrupt Block 0 | Level         |               |
|            218 | PINT1_BLOCK | PINT1 Pin Interrupt Block 1 | Level         |               |
|            219 | PINT2_BLOCK | PINT2 Pin Interrupt Block 2 | Level         |               |
|            220 | PINT3_BLOCK | PINT3 Pin Interrupt Block 3 | Level         |               |
|            221 | PINT4_BLOCK | PINT4 Pin Interrupt Block 4 | Level         |               |
|            222 | PINT5_BLOCK | PINT5 Pin Interrupt Block 5 | Level         |               |
|            223 | PINT6_BLOCK | PINT6 Pin Interrupt Block 6 | Level         |               |
|            224 | PINT7_BLOCK | PINT7 Pin Interrupt Block 7 | Level         |               |

## ADSP-2184x PINT Trigger List

Table 13-6: ADSP-2184x PINT Trigger List Generators

|   Trigger ID | Name        | Description               | Sensitivity   |
|--------------|-------------|---------------------------|---------------|
|          108 | PINT0_BLOCK | PINT0 Pin Interrupt Block | Level         |
|          109 | PINT1_BLOCK | PINT1 Pin Interrupt Block | Level         |
|          110 | PINT2_BLOCK | PINT2 Pin Interrupt Block | Level         |
|          111 | PINT3_BLOCK | PINT3 Pin Interrupt Block | Level         |
|          112 | PINT4_BLOCK | PINT4 Pin Interrupt Block | Level         |
|          113 | PINT5_BLOCK | PINT5 Pin Interrupt Block | Level         |
|          114 | PINT6_BLOCK | PINT6 Pin Interrupt Block | Level         |
|          115 | PINT7_BLOCK | PINT7 Pin Interrupt Block | Level         |

Table 13-7: ADSP-2184x PINT Trigger List Receivers

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         | None   | None          | None          |

## ADSP-2184x PADS Register List

The PADS controls signal hysteresis and other system interface signal features for a number of module interfaces.

Table 13-8: ADSP-2184x PADS Register List

| Name           | Description                                      |
|----------------|--------------------------------------------------|
| PADS_DAI0_0_DS | DAI0 Pins 1 to 8 Drive Strength Control Register |

Table 13-8: ADSP-2184x PADS Register List (Continued)

| Name                | Description                                        |
|---------------------|----------------------------------------------------|
| PADS_DAI0_1_DS      | DAI0 Pins 9 to 16 Drive Strength Control Register  |
| PADS_DAI0_2_DS      | DAI0 Pins 17 to 20 Drive Strength Control Register |
| PADS_DAI0_IE        | DAI0 Port Input Enable Control Register            |
| PADS_DAI0_PE        | DAI0 Pull Enable Register                          |
| PADS_DAI0_PS        | DAI0 Pull Selector Register                        |
| PADS_DAI0_SL        | DAI0 Slew Rate Control Register                    |
| PADS_DAI0_SPU       | DAI0 Strong Pull Up Control Register               |
| PADS_DAI1_0_DS      | DAI1 Pins 1 to 8 Drive Strength Control Register   |
| PADS_DAI1_1_DS      | DAI1 Pins 9 to 16 Drive Strength Control Register  |
| PADS_DAI1_2_DS      | DAI1 Pins 17 to 20 Drive Strength Control Register |
| PADS_DAI1_IE        | DAI1 Port Input Enable Control Register            |
| PADS_DAI1_PE        | DAI1 Pull Enable Register                          |
| PADS_DAI1_PS        | DAI1 Pull Selector Register                        |
| PADS_DAI1_SL        | DAI1 Slew Rate Control Register                    |
| PADS_DAI1_SPU       | DAI1 Strong Pull up Control Register               |
| PADS_NONPORTS_CTL   | Non-GPIO Control Register                          |
| PADS_NONPORTS_DS    | Non-GPIO Drive Strength Register                   |
| PADS_NONPORTS_DS_1  | EMAC Non-GPIO Drive Strength Register              |
| PADS_OTPC_BOOT_STAT | OTPC Boot Status Register                          |
| PADS_PCFG0          | Peripheral PAD Configuration 0 Register            |
| PADS_PCFG1          | Peripheral Configuration 1 Register                |
| PADS_PORTA0_DS      | PORTA Pins 0 to 7 Drive Strength Control Register  |
| PADS_PORTA1_DS      | PORTA Pins 8 to 15 Drive Strength Control Register |
| PADS_PORTB0_DS      | PORTB Pins 0 to 7 Drive Strength Control Register  |
| PADS_PORTB1_DS      | PORTB Pins 8 to 15 Drive Strength Control Register |
| PADS_PORTC0_DS      | PORTC Pins 0 to 7 Drive Strength Control Register  |
| PADS_PORTC1_DS      | PORTC Pins 8 to 15 Drive Strength Control Register |
| PADS_PORTD0_DS      | PORTD Pins 0 to 7 Drive Strength Control Register  |
| PADS_PORTD1_DS      | PORTD Pins 8 to 15 Drive Strength Control Register |
| PADS_PORTE0_DS      | PORTE Pins 0 to 7 Drive Strength Control Register  |
| PADS_PORTE1_DS      | PORTE Pins 8 to 15 Drive Strength Control Register |

Table 13-8: ADSP-2184x PADS Register List (Continued)

| Name             | Description                                        |
|------------------|----------------------------------------------------|
| PADS_PORTF0_DS   | PORTF Pins 0 to 7 Drive Strength Control Register  |
| PADS_PORTF1_DS   | PORTF Pins 8 to 15 Drive Strength Control Register |
| PADS_PORTG0_DS   | PORTG Pins 0 to 7 Drive Strength Control Register  |
| PADS_PORTG1_DS   | PORTG Pins 8 to 15 Drive Strength Control Register |
| PADS_PORTH0_DS   | PORTH Pins 0 to 7 Drive Strength Control Register  |
| PADS_PORTH1_DS   | PORTH Pins 8 and 9 Drive Strength Control Register |
| PADS_PORT_AB_PE  | PORTA and PORTB Pull Enable Register               |
| PADS_PORT_AB_PS  | PORTA and PORTB Pull Selector Register             |
| PADS_PORT_AB_SL  | PORTA and PORTB Slew Rate Control Register         |
| PADS_PORT_AB_SPU | PORTA and PORTB Strong Pull Up Control Register    |
| PADS_PORT_CD_PE  | PORTC and PORTD Pull Enable Register               |
| PADS_PORT_CD_PS  | PORTC and PORTD Pull Selector Register             |
| PADS_PORT_CD_SL  | PORTC and PORTD Slew Rate Control Register         |
| PADS_PORT_CD_SPU | PORTC and PORTD Strong Pull Up Control Register    |
| PADS_PORT_EF_PE  | PORTE and PORTF Pull Enable Register               |
| PADS_PORT_EF_PS  | PORTE and PORTF Pull Selector Register             |
| PADS_PORT_EF_SL  | PORTE and PORTF Slew Rate Control Register         |
| PADS_PORT_EF_SPU | PORTE and PORTF Strong Pull Up Control Register    |
| PADS_PORT_GH_PE  | PORTG and PORTH Pull Enable Register               |
| PADS_PORT_GH_PS  | PORTG and PORTH Pull Selector Register             |
| PADS_PORT_GH_SL  | PORTG and PORTH Slew Rate Control Register         |
| PADS_PORT_GH_SPU | PORTG and PORTH Strong Pull Up Control Register    |
| PADS_STAT        | Status Register                                    |

## PORT Architectural Concepts

These sections describe in more detail how the PORT module connects externally to pins and internally to the MMR bus. Ports are named alphabetically beginning with A.

- Internal Interfaces
- External Interfaces
- GPIO Pin Function
- Port Multiplexing Control

## Internal Interfaces

All of the pin multiplexing, GPIO, and pin interrupt control block MMRs can be accessed through the MMR bus. There is no DMA support. Each of the pin interrupt (PINTx) modules has its own dedicated interrupt request output signal that connects directly to the system core.

## External Interfaces

The pin multiplexing hardware can be seen as a layer between the on-chip peripherals and the silicon pads connecting to the physical pins/balls or the package, as controlled by the PORT unit.

## GPIO Pin Function

By default, the PORT sets every GPIO pin to input mode. The input drivers are not enabled, which avoids the need for unnecessary current sinks and external termination resistors on unused pins.

## Input Mode

The default mode of every GPIO pin after reset is input mode, but the input drivers are not enabled. To enable GPIO input drivers, set the bits corresponding to the PORT pins in the appropriate input enable register ( PORT\_INEN ). When enabled, a read from the PORT\_DATA register returns the logical state of the input pins. However, the input signal does not overwrite the state of the internal flip-flop used for providing output to the same pin. Only software can alter the state. If the input driver is enabled, a write to the PORT\_DATA register can alter the state of the flip-flop, but the change cannot be read back.

## Output Mode

Any GPIO pin can be configured for output mode. The GPIO output drivers are enabled by setting the bits corresponding to the PORT pins in the appropriate direction register. The PORT implements direction registers as a pair of write-1-to-set (W1S) and write-1-to-clear (W1C) MMRs called PORT\_DIR\_SET and PORT\_DIR\_CLR , respectively. As such, software can alter the direction of the signal flow on individual GPIO pins without impacting other GPIOs on the same port.

Both the PORT\_DIR\_SET and PORT\_DIR\_CLR registers return the same value when read, and a logical 1 indicates an enabled output. The PORT\_DATA registers control the state of output pins. A logical 0 drives the output low while a logical 1 drives the output high.

While writes to the PORT\_DATA register can alter all of the GPIOs on a specific port at once, there are also a pair of W1S and W1C MMRs called PORT\_DATA\_SET and PORT\_DATA\_CLR , respectively. These registers enable the manipulation of individual GPIO outputs. The state of the outputs can be obtained by reading the PORT\_DATA registers. Because the state of the GPIO output can be controlled before the output driver is enabled, set or clear the internal flip-flop first by programming this register to avoid volatile levels on the output pin.

## Trigger Toggle Mode

Any GPIO pin that has been configured for output mode can be toggled using a trigger input from the Trigger Routing Unit (TRU). To enable this functionality for a particular GPIO, set the appropriate bit in the PORT\_TRIG\_TGL register. Any subsequent trigger for the designated port causes all GPIO outputs set in the PORT\_TRIG\_TGL register to toggle.

To avoid unpredictable behavior, do not set, clear, or toggle the PORT\_DATA , PORT\_DATA\_SET , PORT\_DATA\_CLR , or PORT\_DATA\_TGL registers when the GPIO output has the corresponding PORT\_TRIG\_TGL bit set. T o change the state of the GPIO output using one of these registers, first clear the corresponding bit in the PORT\_TRIG\_TGL register.

## Open-Drain Mode

Every GPIO can also be used in open-drain mode. First, either clear the respective bit in the PORT\_DATA register or set the respective bit in the PORT\_DATA\_CLR register. Then, set the appropriate bit in the PORT\_INEN register. Read from the PORT\_DATA register to return the status from the pin rather than the state of the internal flip-flop.

By toggling the output driver through the PORT\_DIR\_SET and PORT\_DIR\_CLR register pair, the output signal can be pulled low or three-stated, as required. The polarity of the driven signal can be inverted when the internal flip-flop is set. When using a GPIO port in open-drain mode, take care to not exceed the V IH  operating condition associated with the respective pins.

## Port Multiplexing Control

To configure pins properly, consult the processor data sheet to determine which bits in the PORT\_FER and PORT\_MUX register map to the pin of interest, and then set these registers appropriately for the desired function.

After reset, all port pins default to GPIO input mode with their output and input drivers disabled. As a result, all unused port pins can be left unconnected. Disabled pins appear as a high impedance to external circuits.

Each port has two dedicated MMRs that control the port multiplexing, the 16-bit function enable ( PORT\_FER ) registers and the 32-bit port multiplexing ( PORT\_MUX ) registers.

The function enable register specifies whether the pin is used as a GPIO pin or allocated for use by a specific peripheral, but it does not specify what the peripheral function is. Each bit in the 16-bit PORT\_FER register corresponds to an individual port pin. For example, if bit 1 (PA1) of the PORTA\_FER register is cleared, the PA\_01 pin is configured as a GPIO. When set, one of the available peripheral functions becomes active on the PA\_01 pin instead.

Pairs of bits in the PORT\_MUX register control the multiplexing between the peripheral functions available to an individual pin, as some PORT pins provide up to four possible peripheral functions.

Refer to the Signal Muxing table in the data sheet for the specific PORT\_MUX settings.

## PORT Event Control

The following sections describe event generation in the PORT module.

## PORT Interrupt Signals

The pin interrupts are decoupled from GPIO functionality, providing the following advantages.

- Flexible mapping scheme enables pins from up to four different ports to be grouped into one common interrupt scheme.

- Interrupt requests work on input and output pins regardless of whether the pin is functioning as a GPIO or a peripheral.

The processor has a number of interrupt channels dedicated to pin interrupts, managed by a set of pin interrupt (PINTx) blocks. Each PINTx block can sense up to 32 GPIO pins, as described in the following list and figure.

The processor supports both 32-bit and 16-bit peripheral bus accesses to PINTx registers.

Figure 13-2: GPIO to PINTx Assignment

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000001_12a5ea54eaba1367e6df7901eb7f3528693c767a8b34dc1e1bfe3151ec94ab7c.png)

Pins connect to the PINTx module, as shown in the PINTx Block Diagram .

Figure 13-3: PINTx Block Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000002_c4a7af69ab4b2e6a1ac9506359b5aedd99f37d958aa95b51c28c368eec73d297.png)

As shown in the PINTx Block Diagram , each port is subdivided into two 8-pin half ports, upper ( PxH ) and lower ( PxL ). The PINT\_ASSIGN registers control the 8-bit multiplexers associated with these half ports, where the lower half units (eight pins) can be forwarded to either byte 0 or byte 2 of the PINTx blocks, and the upper half units (eight pins) can be forwarded to either byte 1 or byte 3 of the PINTx blocks.

When a half port is assigned to a byte in any PINTx block, the state of the eight pins appears in the PINT\_PINSTATE register, regardless of whether the pin is enabled for GPIO or peripheral functions (input or output). When neither the input nor output drivers of the pin are enabled, the pin state is read as zero. The PINT\_PINSTATE register reports the inverted state of the pin when the PINT\_INV\_SET register activates the signal inverter. The inverter can be enabled on an individual bit-by-bit basis. Each bit in the PINT\_INV\_SET / PINT\_INV\_CLR register pair represents a pin signal.

By default, PORT interrupt request generation is level-sensitive, and an interrupt request occurs when the enabled pin is sensed as active high. Use the PINT\_EDGE\_SET register to change the interrupt request generation scheme to instead be edge-sensitive (rising edge generates the interrupt request). Use the PINT\_INV\_SET register to invert the polarity such that the PINTx block generates the interrupt request on active-low signals or falling edges.

The PINTx modules also assist when both signal edges must generate unique interrupt requests. If two different interrupt requests are required, the PINT\_ASSIGN registers can route a single input signal to two different PINTx blocks, where one block inverts the signal in the PINT\_INV\_SET register and the other one does not. In this fashion, a unique software routine is associated with the hardware PINTx block that is generating the unique interrupt request for each signal edge. When both signal edges can be serviced by the same interrupt request, each half port can be routed to two separate bytes within a single PINTx block using the PINT\_ASSIGN register, and then one of the half ports needs to have the inversion enabled in the PINT\_INV\_SET register. The servicing software routine can then detect from the PINT\_LATCH register whether a falling, rising, or both edges have occurred.

Regardless of whether level-sensitive or edge-sensitive mode is used, the hardware always latches an interrupt request. Latched signals can be read from the PINT\_LATCH registers. Only a software or hardware reset clears the latches. To clear the latch, a W1C operation must be performed to the PINT\_REQ or PINT\_LATCH register. When in level-sensitive mode, the interrupt request remains asserted if the pin state does not change by the time the interrupt service routine exits.

Because every PINTx block groups up to 32 pin signals, the PINT\_MSK\_SET / PINT\_MSK\_CLR register pair can control which of the signals can request an interrupt at the system level. Software can interrogate the PINT\_REQ register for signaling pins. The PINT\_REQ bits represent a logical AND between the mask and the latch. When any of these bits is set, the block output interrupt request is asserted.

## PORT Programming Model

The GPIO Programming Model Flow (Part 1) , GPIO Programming Model Flow (Part 2) , and GPIO Programming Model Flow (Part 3) figures show the programming model for the general-purpose ports. This programming includes the GPIO input and output operation, open-drain mode, and the pin interrupt PINTx modules.

NOTE: These process flow diagrams connect where call-out letters appear. For example, call-out A in the GPIO Programming Model Flow (Part 1) diagram connects to call-out A in the GPIO Programming Model Flow (Part 2) diagram.

The following flowcharts describe the processes for setting up pins for various functions. Begin the process from the start label in the GPIO Programming Model Flow (Part 1) figure. The first decision (GPIO or peripheral) determines how to program the PORT\_FER register. Set the bit(s) corresponding to the pin(s) to 1 to enable peripheral functionality or to 0 to enable GPIO mode. For more information on setting up for peripheral functions, refer to Port Multiplexing Control.

If the pin is to be a GPIO pin, a subsequent series of decisions must be made that will impact how the PORT\_DATA , PORT\_POL , PORT\_DIR , and PORT\_INEN configuration registers must be programmed. Depending on the type of GPIO pin desired, some configurations do not apply and can have different meanings. The following paragraphs briefly describe the function of the different settings for each of the pin functions in the input, output, and open-drain GPIO modes. It is a best practice to use the SET or CLR versions of the PORT registers, where applicable, to effect changes on a pin-by-pin basis rather than on the full port.

For output mode, first clear the PORT\_DATA register to set all the pins low. Then write the PORT\_DIR register to define the direction of each pin (set the bits associated with the desired output pins to 1). In output mode, the other registers are not significant. The GPIO Programming Model Flow (Part 1) chart shows this flow starting at label 2.

Figure 13-4: GPIO Programming Model Flow (Part 1)

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000003_ac7cb6b1418f5ce8fc134cb3fb430a19250f0850d0520073ee1ee06994fe35ed.png)

For input mode, first decide the polarity for each pin using the PORT\_POL register. Program the PORT\_DIR register to define the appropriate pins as inputs (write a 0 to the bit location associated with the pin). If interrupt requests are desired, configure the PINT module as shown in the GPIO Programming Model Flow (Part 3) figure starting at label B. Finally, write the PORT\_INEN register to enable the associated input drivers. The GPIO Programming Model Flow (Part 2) chart shows this entire flow starting at label 3.

For open-drain mode, set all pins low by clearing the PORT\_DATA register. Then, use the PORT\_INEN register to enable the appropriate input drivers. Set the PORT\_DIR register in this mode to indicate whether the pin is in an active state or not (active being 0). The GPIO Programming Model Flow (Part 2) chart shows this flow starting at label 4.

Figure 13-5: GPIO Programming Model Flow (Part 2)

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000004_f66215be733ab6580bdb3e2c0ea1bce05060cb4818914632ed030fbff8fe0009.png)

Figure 13-6: GPIO Programming Model Flow (Part 3)

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000005_a5f69fd79211414bd7e3f1aeb25af67a12a8968049e202ebc2fb4c706ce62dff.png)

## Programmable Pull Enable and Pull Select Resistors for PORT and DAI

There are programmable weak pull-up resistors for DAI and PORT pins. These resistors can be enabled or disabled by configuring the PADS\_PORT\_AB\_PE , PADS\_PORT\_CD\_PE , PADS\_PORT\_EF\_PE , PADS\_PORT\_GH\_PE ; PADS\_PORT\_AB\_PS , PADS\_PORT\_CD\_PS , PADS\_PORT\_EF\_PS , PADS\_PORT\_GH\_PS ; PADS\_DAI0\_PE , PADS\_DAI1\_PE ; and PADS\_DAI0\_PS , PADS\_DAI1\_PS registers.

## Pull Enable (PE) and Pull Select (PS) Control

The GPIO pad provides optional internal pull resistors to define a stable logic level when the pin is not actively driven. The following controls are configurable:

- Pull Enable (PE): Determines whether a pull resistor is connected. When PE = 0, no pull is applied. When PE = 1, a pull resistor is enabled

- Pull Select (PS): Defines the type of pull resistor when PE is enabled. Setting PS = 0 applies a pull-down, while PS = 1 applies a pull-up.
- Strong Pull-Up (SPU): In input mode, enabling SPU forces the PAD strongly high, overriding weak pull configurations

NOTE: For unused PORT/DAI pins, ensure that any unused pin does not go into a floating state, by clearing (=0) the PE bit of that pin.

## ADSP-2184x PORT Register Descriptions

The General-Purpose Input/Output Port (PORT) contains the following registers.

Table 13-9: ADSP-2184x PORT Register List

| Name          | Description                                |
|---------------|--------------------------------------------|
| PORT_DATA     | Port x GPIO Data Register                  |
| PORT_DATA_CLR | Port x GPIO Data Clear Register            |
| PORT_DATA_SET | Port x GPIO Data Set Register              |
| PORT_DATA_TGL | Port x GPIO Output Toggle Register         |
| PORT_DIR      | Port x GPIO Direction Register             |
| PORT_DIR_CLR  | Port x GPIO Direction Clear Register       |
| PORT_DIR_SET  | Port x GPIO Direction Set Register         |
| PORT_FER      | Port x Function Enable Register            |
| PORT_FER_CLR  | Port x Function Enable Clear Register      |
| PORT_FER_SET  | Port x Function Enable Set Register        |
| PORT_INEN     | Port x GPIO Input Enable Register          |
| PORT_INEN_CLR | Port x GPIO Input Enable Clear Register    |
| PORT_INEN_SET | Port x GPIO Input Enable Set Register      |
| PORT_LOCK     | Port x GPIO Lock Register                  |
| PORT_MUX      | Port x Multiplexer Control Register        |
| PORT_POL      | Port x GPIO Polarity Invert Register       |
| PORT_POL_CLR  | Port x GPIO Polarity Invert Clear Register |
| PORT_POL_SET  | Port x GPIO Polarity Invert Set Register   |
| PORT_TRIG_TGL | Port x GPIO Trigger Toggle Register        |

## Port x GPIO Data Register

The operation of the PORT\_DATA register depends on whether the bit/pin is in output mode or input mode. In both modes, a set bit in the PORT\_DATA register corresponds to a signal high on a GPIO pin. A cleared bit in the PORT\_DATA register corresponds to a signal low on a GPIO pin.

The PORT\_DATA , PORT\_DATA\_SET , and PORT\_DATA\_CLR registers control the state of GPIO pins in output mode. To enable output mode (and output drivers), use the PORT\_DIR\_SET and PORT\_DIR\_CLR registers.

Writes to the PORT\_DATA register affect the state of all pins of the port that are in output mode. T o set or clear specific pins without impacting other pins of the port, use the PORT\_DATA\_SET and PORT\_DATA\_CLR registers.

When the GPIO pins are in input mode (input driver is enabled with the PORT\_INEN register), reads from the PORT\_DATA , PORT\_DATA\_SET , and PORT\_DATA\_CLR registers return the state of the respective GPIO pins.

Note that when the input driver is not enabled, reads from the PORT\_DATA , PORT\_DATA\_SET , and PORT\_DATA\_CLR registers return the value previously written to the registers.

Figure 13-7: PORT\_DATA Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000006_6fc8b6b2300151ed49a23d25255d7e60e19b28502cae1af56dbee7ef31885008.png)

Table 13-10: PORT\_DATA Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | PX15       | Port x Bit 15 Data. The PORT_DATA.PX15 bit indicates a signal on a GPIO pin.                            |
| 14 (R/W)           | PX14       | Port x Bit 14 Data. The PORT_DATA.PX14 bit indicates a signal on a GPIO pin. 0 Signal Low               |
| 13 (R/W)           | PX13       | Port x Bit 13 Data. The PORT_DATA.PX13 bit indicates a signal on a GPIO pin. 0 Signal Low               |
| 12 (R/W)           | PX12       | 1 Signal High Port x Bit 12 Data. The PORT_DATA.PX12 bit indicates a signal on a GPIO pin. 0 Signal Low |
| 11 (R/W)           | PX11       | 1 Signal High Port x Bit 11 Data. The PORT_DATA.PX11 bit indicates a signal on a GPIO pin.              |
| 10 (R/W)           | PX10       | 0 Signal Low 1 Signal High Port x Bit 10 Data.                                                          |
|                    |            | The PORT_DATA.PX10 bit indicates a signal on a GPIO pin. 0 Signal Low 1 Signal High                     |
| 9 (R/W)            | PX9        | Port x Bit 9 Data. The PORT_DATA.PX9 bit indicates a signal on a GPIO pin. 0 Signal Low                 |

Table 13-10: PORT\_DATA Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | PX8        | Port x Bit 8 Data. The PORT_DATA.PX8 bit indicates a signal on a GPIO pin.                            |
| 7 (R/W)            | PX7        | Port x Bit 7 Data. The PORT_DATA.PX7 bit indicates a signal on a GPIO pin. 0 Signal Low               |
| 6 (R/W)            | PX6        | Port x Bit 6 Data. The PORT_DATA.PX6 bit indicates a signal on a GPIO pin. 0 Signal Low               |
| 5 (R/W)            | PX5        | Port x Bit 5 Data. The PORT_DATA.PX5 bit indicates a signal on a GPIO pin.                            |
| 4 (R/W)            | PX4        | 0 Signal Low 1 Signal High                                                                            |
|                    |            | Port x Bit 4 Data. The PORT_DATA.PX4 bit indicates a signal on a GPIO pin. 0 Signal Low 1 Signal High |
| 3 (R/W)            | PX3        | Port x Bit 3 Data. The PORT_DATA.PX3 bit indicates a signal on a GPIO pin. 0 Signal Low 1 Signal High |
| 2 (R/W)            | PX2        | Port x Bit 2 Data. The PORT_DATA.PX2 bit indicates a signal on a GPIO pin. 0 Signal Low               |

Table 13-10: PORT\_DATA Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | PX1        | Port x Bit 1 Data. The PORT_DATA.PX1 bit indicates a signal on a GPIO pin. 0 Signal Low               |
| 0 (R/W)            | PX0        | Port x Bit 0 Data. The PORT_DATA.PX0 bit indicates a signal on a GPIO pin. 0 Signal Low 1 Signal High |

## Port x GPIO Data Clear Register

The PORT\_DATA\_CLR register operates differently for port bits/pins, depending on whether the bit/pin is output mode or input mode. For more information, see the PORT\_DATA register description.

Figure 13-8: PORT\_DATA\_CLR Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000007_12921938ad63e858c4ab206c9bf7bc2829a9fd126e27f07d72cb41eabf3610e7.png)

Table 13-11: PORT\_DATA\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W1C)         | PX15       | Port x Bit 15 Data Clear. The PORT_DATA_CLR.PX15 bit clears the pin without impacting other pins of the port. 0 No Effect                                                                                            |
| 14 (R/W1C)         | PX14       | Port x Bit 14 Data Clear. The PORT_DATA_CLR.PX14 bit clears the pin without impacting other pins of the port. 0 No Effect. Write 0 has no effect in output mode. 1 Clear Bit. Write 1 for signal low in output mode. |

Table 13-11: PORT\_DATA\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------|
| 13 (R/W1C)         | PX13       | Port x Bit 13 Data Clear. The PORT_DATA_CLR.PX13 bit clears the pin without impacting other pins of the port. |
| 13 (R/W1C)         | PX13       | 0 No Effect. Write 0 has no effect in output mode.                                                            |
| 13 (R/W1C)         | PX13       | 1 Clear Bit. Write 1 for signal low in output mode.                                                           |
| 12 (R/W1C)         | PX12       | Port x Bit 12 Data Clear. The PORT_DATA_CLR.PX12 bit clears the pin without impacting other pins of the port. |
| 12 (R/W1C)         | PX12       | 0 No Effect. Write 0 has no effect in output mode.                                                            |
| 12 (R/W1C)         | PX12       | 1 Clear Bit. Write 1 for signal low in output mode.                                                           |
| 11 (R/W1C)         | PX11       | Port x Bit 11 Data Clear. The PORT_DATA_CLR.PX11 bit clears the pin without impacting other pins of the port. |
| 11 (R/W1C)         | PX11       | 0 No Effect                                                                                                   |
| 11 (R/W1C)         | PX11       | 1 Clear Bit. Write 1 for signal low in output mode.                                                           |
| 10 (R/W1C)         | PX10       | Port x Bit 10 Data Clear. The PORT_DATA_CLR.PX10 bit clears the pin without impacting other pins of the port. |
| 10 (R/W1C)         | PX10       | 0 No Effect. Write 0 has no effect in output mode.                                                            |
| 10 (R/W1C)         | PX10       | 1 Clear Bit. Write 1 for signal low in output mode.                                                           |
| 9 (R/W1C)          | PX9        | Port x Bit 9 Data Clear. The PORT_DATA_CLR.PX9 bit clears the pin without impacting other pins of the port.   |
| 9 (R/W1C)          | PX9        | 0 No Effect. Write 0 has no effect in output mode.                                                            |
| 9 (R/W1C)          | PX9        | 1 Clear Bit. Write 1 for signal low in output mode.                                                           |
| 8 (R/W1C)          | PX8        | Port x Bit 8 Data Clear. The PORT_DATA_CLR.PX8 bit clears the pin without impacting other pins of the port.   |
| 8 (R/W1C)          | PX8        | 0 No Effect. Write 0 has no effect in output mode.                                                            |
| 8 (R/W1C)          | PX8        | 1 Clear Bit. Write 1 for signal low in output mode.                                                           |

Table 13-11: PORT\_DATA\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------|
| 7 (R/W1C)          | PX7        | Port x Bit 7 Data Clear. The PORT_DATA_CLR.PX7 bit clears the pin without impacting other pins of the port. |
| 7 (R/W1C)          | PX7        | 0 No Effect. Write 0 has no effect in output mode.                                                          |
| 7 (R/W1C)          | PX7        | 1 Clear Bit. Write 1 for signal low in output mode.                                                         |
| 6 (R/W1C)          | PX6        | Port x Bit 6 Data Clear. The PORT_DATA_CLR.PX6 bit clears the pin without impacting other pins of the port. |
| 6 (R/W1C)          | PX6        | 0 No Effect. Write 0 has no effect in output mode.                                                          |
| 6 (R/W1C)          | PX6        | 1 Clear Bit. Write 1 for signal low in output mode.                                                         |
| 5 (R/W1C)          | PX5        | Port x Bit 5 Data Clear. The PORT_DATA_CLR.PX5 bit clears the pin without impacting other pins of the port. |
| 5 (R/W1C)          | PX5        | 0 No Effect. Write 0 has no effect in output mode.                                                          |
| 4 (R/W1C)          | PX4        | Port x Bit 4 Data Clear. The PORT_DATA_CLR.PX4 bit clears the pin without impacting other pins of the port. |
| 4 (R/W1C)          | PX4        | 0 No Effect. Write 0 has no effect in output mode.                                                          |
| 4 (R/W1C)          | PX4        | 1 Clear Bit. Write 1 for signal low in output mode.                                                         |
| 3 (R/W1C)          | PX3        | Port x Bit 3 Data Clear. The PORT_DATA_CLR.PX3 bit clears the pin without impacting other pins of the port. |
| 3 (R/W1C)          | PX3        | 0 No Effect. Write 0 has no effect in output mode.                                                          |
| 3 (R/W1C)          | PX3        | 1 Clear Bit. Write 1 for signal low in output mode.                                                         |
| 2 (R/W1C)          | PX2        | Port x Bit 2 Data Clear. The PORT_DATA_CLR.PX2 bit clears the pin without impacting other pins of the port. |
| 2 (R/W1C)          | PX2        | 0 No Effect Write 0 has no effect in output mode.                                                           |
| 2 (R/W1C)          | PX2        | 1 Clear Bit Write 1 for signal low in output mode.                                                          |

Table 13-11: PORT\_DATA\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1C)          | PX1        | Port x Bit 1 Data Clear. The PORT_DATA_CLR.PX1 bit clears the pin without impacting other pins of the port. 0 No Effect. Write 0 has no effect in output mode.                                                     |
| 0 (R/W1C)          | PX0        | Port x Bit 0 Data Clear. The PORT_DATA_CLR.PX0 bit clears the pin without impacting other pins of the port. 0 No Effect. Write 0 has no effect in output mode. 1 Clear Bit. Write 1 for signal low in output mode. |

## Port x GPIO Data Set Register

The PORT\_DATA\_SET register operates differently for port bits/pins, depending on whether the bit/pin is output mode or input mode. For more information, see the PORT\_DATA register description.

Figure 13-9: PORT\_DATA\_SET Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000008_f6d30a7b57b5dd5cc43576cf73e78b2d30c30edbc825e7d1388498150cde2519.png)

Table 13-12: PORT\_DATA\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   | Description/Enumeration                          |
|--------------------|------------|---------------------------|--------------------------------------------------|
| 15                 | PX15       | Port x Bit 15 Data Set.   | Port x Bit 15 Data Set.                          |
| (R/W1S)            |            | 0                         | No Effect. Write 0 has no effect in output mode. |
|                    |            | 1                         | Set Bit. Write 1 for signal high in output mode. |
| 14                 | PX14       | Port x Bit 14 Data Set.   | Port x Bit 14 Data Set.                          |
| (R/W1S)            |            | 0                         | No Effect. Write 0 has no effect in output mode. |
|                    |            | 1                         | Set Bit. Write 1 for signal high in output mode. |
| 13                 | PX13       | Port x Bit 13 Data Set.   | Port x Bit 13 Data Set.                          |
| (R/W1S)            |            | 0                         | No Effect. Write 0 has no effect in output mode. |
|                    |            | 1                         | Set Bit. Write 1 for signal high in output mode. |

Table 13-12: PORT\_DATA\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   | Description/Enumeration                          |
|--------------------|------------|---------------------------|--------------------------------------------------|
| 12                 | PX12       | Port x Bit 12 Data Set.   | Port x Bit 12 Data Set.                          |
| (R/W1S)            |            | 0                         | No Effect. Write 0 has no effect in output mode. |
|                    |            | 1                         | Set Bit. Write 1 for signal high in output mode. |
| 11                 | PX11       | Port x Bit 11 Data Set.   | Port x Bit 11 Data Set.                          |
| (R/W1S)            |            | 0                         | No Effect. Write 0 has no effect in output mode. |
|                    |            | 1                         | Set Bit.                                         |
| 10                 | PX10       | Port x Bit 10 Data Set.   | Port x Bit 10 Data Set.                          |
| (R/W1S)            |            | 0                         | No Effect. Write 0 has no effect in output mode. |
|                    |            | 1                         | Set Bit. Write 1 for signal high in output mode. |
| 9                  | PX9        | Port x Bit 9 Data Set.    | Port x Bit 9 Data Set.                           |
| (R/W1S)            |            | 0                         | No Effect. Write 0 has no effect in output mode. |
|                    |            | 1                         | Set Bit. Write 1 for signal high in output mode. |
| 8                  | PX8        | Port x Bit 8 Data Set.    | Port x Bit 8 Data Set.                           |
| (R/W1S)            |            | 0                         | No Effect. Write 0 has no effect in output mode. |
|                    |            | 1                         | Set Bit. Write 1 for signal high in output mode. |
| 7                  | PX7        | Port x Bit 7 Data Set.    | Port x Bit 7 Data Set.                           |
| (R/W1S)            |            | 0                         | No Effect. Write 0 has no effect in output mode. |
|                    |            | 1                         | Set Bit. Write 1 for signal high in output mode. |
| 6                  | PX6        | Port x Bit 6 Data Set.    | Port x Bit 6 Data Set.                           |
| (R/W1S)            |            | 0                         | No Effect. Write 0 has no effect in output mode. |
|                    |            | 1                         | Set Bit. Write 1 for signal high in output mode. |
| 5                  | PX5        | Port x Bit 5 Data Set.    | Port x Bit 5 Data Set.                           |
| (R/W1S)            |            | 0                         | No Effect. Write 0 has no effect in output mode. |
|                    |            | 1                         | Set Bit. Write 1 for signal high in output mode. |
| 4                  | PX4        | Port x Bit 4 Data Set.    | Port x Bit 4 Data Set.                           |
| (R/W1S)            |            | 0                         | No Effect. Write 0 has no effect in output mode. |
|                    |            | 1                         | Set Bit. Write 1 for signal high in output mode. |
| 3                  | PX3        | Port x Bit 3 Data Set.    | Port x Bit 3 Data Set.                           |
| (R/W1S)            |            | 0                         | No Effect. Write 0 has no effect in output mode. |
|                    |            | 1                         | Set Bit. Write 1 for signal high in output mode. |

Table 13-12: PORT\_DATA\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   | Description/Enumeration                          |
|--------------------|------------|---------------------------|--------------------------------------------------|
| 2                  | PX2        | Port x Bit 2 Data Set.    | Port x Bit 2 Data Set.                           |
| (R/W1S)            |            | 0                         | No Effect. Write 0 has no effect in output mode. |
|                    |            | 1                         | Set Bit. Write 1 for signal high in output mode. |
| 1                  | PX1        | Port x Bit 1 Data Set.    | Port x Bit 1 Data Set.                           |
| (R/W1S)            |            | 0                         | No Effect. Write 0 has no effect in output mode. |
|                    |            | 1                         | Set Bit. Write 1 for signal high in output mode. |
| 0                  | PX0        | Port x Bit 0 Data Set.    | Port x Bit 0 Data Set.                           |
| (R/W1S)            |            | 0                         | No Effect. Write 0 has no effect in output mode. |
|                    |            | 1                         | Set Bit. Write 1 for signal high in output mode. |

## Port x GPIO Output Toggle Register

The PORT\_DATA\_TGL register permits toggling the state of output GPIO pins. Setting bits in the PORT\_DATA\_TGL register affects the state of specific pins without impacting other pins of the port.

Reading the PORT\_DATA\_TGL returns the state of the PORT\_DATA register output pin state, but does not return the input pin/signal state.

Figure 13-10: PORT\_DATA\_TGL Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000009_72d504aa9f29d89604be6b33e37c4283ecf9e1988a38f2b2a230f8945805e8d4.png)

Table 13-13: PORT\_DATA\_TGL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | PX15       | Port x Bit 15 Toggle. The PORT_DATA_TGL.PX15 bit toggles the output GPIO bit/pin state.                          |
| 14 (R/W)           | PX14       | Port x Bit 14 Toggle. The PORT_DATA_TGL.PX14 bit toggles the output GPIO bit/pin state. 0 No Effect 1 Toggle Bit |

Table 13-13: PORT\_DATA\_TGL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | PX13       | Port x Bit 13 Toggle. The PORT_DATA_TGL.PX13 bit toggles the output GPIO bit/pin state.                        |
| 12 (R/W)           | PX12       | Port x Bit 12 Toggle. The PORT_DATA_TGL.PX12 bit toggles the output GPIO bit/pin state. 0 No Effect            |
| 11 (R/W)           | PX11       | Port x Bit 11 Toggle. The PORT_DATA_TGL.PX11 bit toggles the output GPIO bit/pin state. 0 No Effect            |
| 10 (R/W)           | PX10       | Port x Bit 10 Toggle. The PORT_DATA_TGL.PX10 bit toggles the output GPIO bit/pin state. 0 No Effect            |
| 9 (R/W)            | PX9        | 1 Toggle Bit                                                                                                   |
|                    |            | Port x Bit 9 Toggle. The PORT_DATA_TGL.PX9 bit toggles the output GPIO bit/pin state. 0 No Effect 1 Toggle Bit |
| 8 (R/W)            | PX8        | Port x Bit 8 Toggle. The PORT_DATA_TGL.PX8 bit toggles the output GPIO bit/pin state. 0 No Effect 1 Toggle Bit |
| 7 (R/W)            | PX7        | Port x Bit 7 Toggle. The PORT_DATA_TGL.PX7 bit toggles the output GPIO bit/pin state. 0 No Effect              |

Table 13-13: PORT\_DATA\_TGL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | PX6        | Port x Bit 6 Toggle. The PORT_DATA_TGL.PX6 bit toggles the output GPIO bit/pin state.                          |
| 5 (R/W)            | PX5        | Port x Bit 5 Toggle. The PORT_DATA_TGL.PX5 bit toggles the output GPIO bit/pin state. 0 No Effect              |
| 4 (R/W)            | PX4        | Port x Bit 4 Toggle. The PORT_DATA_TGL.PX4 bit toggles the output GPIO bit/pin state. 0 No Effect              |
| 3 (R/W)            | PX3        | Port x Bit 3 Toggle. The PORT_DATA_TGL.PX3 bit toggles the output GPIO bit/pin state. 0 No Effect              |
| 2 (R/W)            | PX2        | 1 Toggle Bit                                                                                                   |
|                    |            | Port x Bit 2 Toggle. The PORT_DATA_TGL.PX2 bit toggles the output GPIO bit/pin state. 0 No Effect 1 Toggle Bit |
| 1 (R/W)            | PX1        | Port x Bit 1 Toggle. The PORT_DATA_TGL.PX1 bit toggles the output GPIO bit/pin state. 0 No Effect 1 Toggle Bit |
| 0 (R/W)            | PX0        | Port x Bit 0 Toggle. The PORT_DATA_TGL.PX0 bit toggles the output GPIO bit/pin state. 0 No Effect              |

## Port x GPIO Direction Register

The PORT\_DIR , PORT\_DIR\_SET , and PORT\_DIR\_CLR registers select output or input mode for GPIO pins and enable output drivers. Use the PORT\_INEN , PORT\_INEN\_SET , and PORT\_INEN\_CLR registers to enable or disable input drivers.

Writes to the PORT\_DIR register affect the state of all pins of the port. T o select a direction for specific pins without impacting other pins of the port, use the PORT\_DIR\_SET and PORT\_DIR\_CLR registers.

Setting a bit in the PORT\_DIR register enables output mode on the corresponding a GPIO pin. Clearing a bit in the PORT\_DIR register disables output mode on the corresponding GPIO pin.

Input Mode - The default mode of every GPIO pin after reset is the input mode, but the input drivers are not enabled. To enable GPIO input drivers, set the corresponding bits in the PORT\_INEN register. When enabled, a read from the PORT\_DATA register returns the logical state of the input pin. The input signal does not overwrite the state of the bit used for the output case. That state can only be altered by software. If the input driver is enabled, a write to the PORT\_DATA register can alter the state of the bit, but the change cannot be read back.

Output Mode - Any GPIO pin can be configured for output mode. The GPIO output drivers are enabled by setting the corresponding bits in the PORT\_DIR , PORT\_DIR\_SET , or PORT\_DIR\_CLR registers. By using the PORT\_DIR\_SET and PORT\_DIR\_CLR registers, the direction of the signal flow of individual GPIO pins can be altered by separate software threads without mutually impacting other GPIOs on the same port. Both registers return the same value when read. Because the state of the GPIO output can already be controlled before the output driver is enabled, it is recommended to first set or clear the bit (using the PORT\_DATA , PORT\_DATA\_SET , or PORT\_DATA\_CLR registers) to avoid any volatile levels on the output.

Open-Drain Mode - Every GPIO can also be used in open-drain mode. To accomplish this, first, clear the respective bit in the PORT\_DATA or PORT\_DATA\_CLR register. Then, set the one bit in the PORT\_INEN register. Reads from the PORT\_DATA register then return the status from the pin and do not return the state of the internal flip-flop. By toggling the output driver through the PORT\_DIR\_SET and PORT\_DIR\_CLR register pair, the output signal can be pulled low or three-stated as required. Note that the polarity of the driven signal can be inverted when the internal flip-flop is set instead. When a GPIO port is used in open-drain mode, take care to not exceed the V IH  operating condition associated with the respective pin.

Figure 13-11: PORT\_DIR Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000010_dfd7140323db9086832b66cd5c457bb1d412c8c2d7294f01793379bfb87f22fd.png)

Table 13-14: PORT\_DIR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   | Description/Enumeration                    |
|--------------------|------------|---------------------------|--------------------------------------------|
| 15                 | PX15       | Port x Bit 15 Direction.  | Port x Bit 15 Direction.                   |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
|                    |            | 1                         | Output mode. The output driver is enabled. |
| 14                 | PX14       | Port x Bit 14 Direction.  | Port x Bit 14 Direction.                   |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
|                    |            | 1                         | Output mode. The output driver is enabled. |
| 13                 | PX13       | Port x Bit 13 Direction.  | Port x Bit 13 Direction.                   |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
|                    |            | 1                         | Output mode. The output driver is enabled. |
| 12                 | PX12       | Port x Bit 12 Direction.  | Port x Bit 12 Direction.                   |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
|                    |            | 1                         | Output mode. The output driver is enabled. |

Table 13-14: PORT\_DIR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   | Description/Enumeration                    |
|--------------------|------------|---------------------------|--------------------------------------------|
| 11                 | PX11       | Port x Bit 11 Direction.  | Port x Bit 11 Direction.                   |
| (R/W)              | PX11       | 0                         | Input mode. The output driver is disabled. |
| 11                 | PX11       | 1                         | Output mode. The output driver is enabled. |
| 10                 | PX10       | Port x Bit 10 Direction.  | Port x Bit 10 Direction.                   |
| (R/W)              | PX10       | 0                         | Input mode. The output driver is disabled. |
| (R/W)              | PX10       | 1                         | Output mode. The output driver is enabled. |
| 9                  | PX9        | Port x Bit 9 Direction.   | Port x Bit 9 Direction.                    |
| (R/W)              | PX9        | 0                         | Input mode. The output driver is disabled. |
| (R/W)              | PX9        | 1                         | Output mode. The output driver is enabled. |
| 8                  | PX8        | Port x Bit 8 Direction.   | Port x Bit 8 Direction.                    |
| (R/W)              | PX8        | 0                         | Input mode. The output driver is disabled. |
| (R/W)              | PX8        | 1                         | Output mode. The output driver is enabled. |
| 7                  | PX7        | Port x Bit 7 Direction.   | Port x Bit 7 Direction.                    |
| (R/W)              | PX7        | 0                         | Input mode. The output driver is disabled. |
| (R/W)              | PX7        | 1                         | Output mode. The output driver is enabled. |
| 6                  | PX6        | Port x Bit 6 Direction.   | Port x Bit 6 Direction.                    |
| (R/W)              | PX6        | 0                         | Input mode. The output driver is disabled. |
| (R/W)              | PX6        | 1                         | Output mode. The output driver is enabled. |
| 5                  | PX5        | Port x Bit 5 Direction.   | Port x Bit 5 Direction.                    |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
| (R/W)              |            | 1                         | Output mode. The output driver is enabled. |
| 4                  | PX4        | Port x Bit 4 Direction.   | Port x Bit 4 Direction.                    |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
| (R/W)              |            | 1                         | Output mode. The output driver is enabled. |
| 3                  | PX3        | Port x Bit 3 Direction.   | Port x Bit 3 Direction.                    |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
| (R/W)              |            | 1                         | Output mode. The output driver is enabled. |
| 2                  | PX2        | Port x Bit 2 Direction.   | Port x Bit 2 Direction.                    |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
| (R/W)              |            | 1                         | Output mode. The output driver is enabled. |

Table 13-14: PORT\_DIR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   | Description/Enumeration                    |
|--------------------|------------|---------------------------|--------------------------------------------|
| 1                  | PX1        | Port x Bit 1 Direction.   | Port x Bit 1 Direction.                    |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
|                    |            | 1                         | Output mode. The output driver is enabled. |
| 0                  | PX0        | Port x Bit 0 Direction.   | Port x Bit 0 Direction.                    |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
|                    |            | 1                         | Output mode. The output driver is enabled. |

## Port x GPIO Direction Clear Register

The PORT\_DIR\_CLR register disables output mode and disables the output drivers for GPIO pins. For more information, see the PORT\_DIR register description.

Figure 13-12: PORT\_DIR\_CLR Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000011_63ce8eba608f2c03ea008333e1040db16a823e91651b72f62a1c6de4c8f874de.png)

Table 13-15: PORT\_DIR\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W1C)         | PX15       | Port x Bit 15 Direction Clear. The PORT_DIR_CLR.PX15 bit disables output mode and the output drivers for port x. 0 No Effect                              |
| 14 (R/W1C)         | PX14       | Port x Bit 14 Direction Clear. The PORT_DIR_CLR.PX14 bit disables output mode and the output drivers for port x. 0 No Effect 1 Disable output mode/driver |

Table 13-15: PORT\_DIR\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------|
| 13 (R/W1C)         | PX13       | Port x Bit 13 Direction Clear. The PORT_DIR_CLR.PX13 bit disables output mode and the output drivers for port x. |
| 13 (R/W1C)         | PX13       | 0 No Effect                                                                                                      |
| 13 (R/W1C)         | PX13       | 1 Disable output mode/driver                                                                                     |
| 12 (R/W1C)         | PX12       | Port x Bit 12 Direction Clear. The PORT_DIR_CLR.PX12 bit disables output mode and the output drivers for port x. |
| 12 (R/W1C)         | PX12       | 0 No Effect                                                                                                      |
| 12 (R/W1C)         | PX12       | 1 Disable output mode/driver                                                                                     |
| 11 (R/W1C)         | PX11       | Port x Bit 11 Direction Clear. The PORT_DIR_CLR.PX11 bit disables output mode and the output drivers for port x. |
| 11 (R/W1C)         | PX11       | 0 No Effect                                                                                                      |
| 11 (R/W1C)         | PX11       | 1 Disable output mode/driver                                                                                     |
| 10 (R/W1C)         | PX10       | Port x Bit 10 Direction Clear. The PORT_DIR_CLR.PX10 bit disables output mode and the output drivers for port x. |
| 10 (R/W1C)         | PX10       | 0 No Effect                                                                                                      |
| 10 (R/W1C)         | PX10       | 1 Disable output mode/driver                                                                                     |
| 9 (R/W1C)          | PX9        | Port x Bit 9 Direction Clear. The PORT_DIR_CLR.PX9 bit disables output mode and the output drivers for port x.   |
| 9 (R/W1C)          | PX9        | 0 No Effect                                                                                                      |
| 9 (R/W1C)          | PX9        | 1 Disable output mode/driver                                                                                     |
| 8 (R/W1C)          | PX8        | Port x Bit 8 Direction Clear. The PORT_DIR_CLR.PX8 bit disables output mode and the output drivers for port x.   |
| 8 (R/W1C)          | PX8        | 0 No Effect                                                                                                      |
| 8 (R/W1C)          | PX8        | 1 Disable output mode/driver                                                                                     |

Table 13-15: PORT\_DIR\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 7 (R/W1C)          | PX7        | Port x Bit 7 Direction Clear. The PORT_DIR_CLR.PX7 bit disables output mode and the output drivers for port x. |
| 7 (R/W1C)          | PX7        | 0 No Effect                                                                                                    |
| 7 (R/W1C)          | PX7        | 1 Disable output mode/driver                                                                                   |
| 6 (R/W1C)          | PX6        | Port x Bit 6 Direction Clear. The PORT_DIR_CLR.PX6 bit disables output mode and the output drivers for port x. |
| 6 (R/W1C)          | PX6        | 0 No Effect                                                                                                    |
| 6 (R/W1C)          | PX6        | 1 Disable output mode/driver                                                                                   |
| 5 (R/W1C)          | PX5        | Port x Bit 5 Direction Clear. The PORT_DIR_CLR.PX5 bit disables output mode and the output drivers for port x. |
| 5 (R/W1C)          | PX5        | 0 No Effect                                                                                                    |
| 5 (R/W1C)          | PX5        | 1 Disable output mode/driver                                                                                   |
| 4 (R/W1C)          | PX4        | Port x Bit 4 Direction Clear. The PORT_DIR_CLR.PX4 bit disables output mode and the output drivers for port x. |
| 4 (R/W1C)          | PX4        | 0 No Effect                                                                                                    |
| 4 (R/W1C)          | PX4        | 1 Disable output mode/driver                                                                                   |
| 3 (R/W1C)          | PX3        | Port x Bit 3 Direction Clear. The PORT_DIR_CLR.PX3 bit disables output mode and the output drivers for port x. |
| 3 (R/W1C)          | PX3        | 0 No Effect                                                                                                    |
| 3 (R/W1C)          | PX3        | 1 Disable output mode/driver                                                                                   |
| 2 (R/W1C)          | PX2        | Port x Bit 2 Direction Clear. The PORT_DIR_CLR.PX2 bit disables output mode and the output drivers for port x. |
| 2 (R/W1C)          | PX2        | 0 No Effect                                                                                                    |
| 2 (R/W1C)          | PX2        | 1 Disable output mode/driver                                                                                   |

Table 13-15: PORT\_DIR\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1C)          | PX1        | Port x Bit 1 Direction Clear. The PORT_DIR_CLR.PX1 bit disables output mode and the output drivers for port x. 0 No Effect 1 Disable output mode/driver |
| 0 (R/W1C)          | PX0        | Port x Bit 0 Direction Clear. The PORT_DIR_CLR.PX0 bit disables output mode and the output drivers for port x. 0 No Effect 1 Disable output mode/driver |

## Port x GPIO Direction Set Register

The PORT\_DIR\_SET register enables output mode and output drivers for GPIO pins. For more information, see the PORT\_DIR register description.

Figure 13-13: PORT\_DIR\_SET Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000012_703aac1b5c8c11515ee597fd691419af2bddc0c7e3b003b9a35f437e91217a7f.png)

Table 13-16: PORT\_DIR\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------|
| 15 (R/W1S)         | PX15       | Port x Bit 15 Direction Set. The PORT_DIR_SET.PX15 bit enables the output mode/driver for port x.             |
| 14 (R/W1S)         | PX14       | Port x Bit 14 Direction Set. The PORT_DIR_SET.PX14 bit enables the output mode/driver for port x. 0 No Effect |
|                    |            | 1 Enable output mode/driver                                                                                   |

Table 13-16: PORT\_DIR\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W1S)         | PX13       | Port x Bit 13 Direction Set. The PORT_DIR_SET.PX13 bit enables the output mode/driver for port x. 0 No Effect                             |
| 12 (R/W1S)         | PX12       | Port x Bit 12 Direction Set. The PORT_DIR_SET.PX12 bit enables the output mode/driver for port x. 0 No Effect 1 Enable output mode/driver |
| 11 (R/W1S)         | PX11       | Port x Bit 11 Direction Set. The PORT_DIR_SET.PX11 bit enables the output mode/driver for port x. 0 No Effect                             |
| 10 (R/W1S)         | PX10       | Port x Bit 10 Direction Set. The PORT_DIR_SET.PX10 bit enables the output mode/driver for port x. 0 No Effect                             |
| 9 (R/W1S)          | PX9        | 1 Enable output mode/driver Port x Bit 9 Direction Set. x.                                                                                |
| 8 (R/W1S)          | PX8        | The PORT_DIR_SET.PX9 bit enables the output mode/driver for port 0 No Effect 1 Enable output mode/driver                                  |
|                    |            | Port x Bit 8 Direction Set. The PORT_DIR_SET.PX8 bit enables the output mode/driver for port x. 0 No Effect 1 Enable output mode/driver   |
| 7 (R/W1S)          | PX7        | Port x Bit 7 Direction Set. The PORT_DIR_SET.PX7 bit enables the output mode/driver for port x. 0 No Effect                               |

Table 13-16: PORT\_DIR\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W1S)          | PX6        | Port x Bit 6 Direction Set. The PORT_DIR_SET.PX6 bit enables the output mode/driver for port x. 0 No Effect                             |
| 5 (R/W1S)          | PX5        | Port x Bit 5 Direction Set. The PORT_DIR_SET.PX5 bit enables the output mode/driver for port x. 0 No Effect                             |
| 4 (R/W1S)          | PX4        | Port x Bit 4 Direction Set. The PORT_DIR_SET.PX4 bit enables the output mode/driver for port x. 0 No Effect                             |
| 3 (R/W1S)          | PX3        | Port x Bit 3 Direction Set. The PORT_DIR_SET.PX3 bit enables the output mode/driver for port x. 0 No Effect                             |
| 2 (R/W1S)          | PX2        | 1 Enable output mode/driver                                                                                                             |
|                    |            | Port x Bit 2 Direction Set. The PORT_DIR_SET.PX2 bit enables the output mode/driver for port x. 0 No Effect 1 Enable output mode/driver |
| 1 (R/W1S)          | PX1        | Port x Bit 1 Direction Set. The PORT_DIR_SET.PX1 bit enables the output mode/driver for port x. 0 No Effect 1 Enable output mode/driver |
| 0 (R/W1S)          | PX0        | Port x Bit 0 Direction Set. The PORT_DIR_SET.PX0 bit enables the output mode/driver for port x.                                         |
|                    |            | 0 No Effect                                                                                                                             |

## Port x Function Enable Register

The PORT\_FER register bits indicate each port bit's operating mode: general purpose I/O mode or peripheral mode. After reset, all pins default to GPIO mode. Setting a bit in the PORT\_FER registers enables a peripheral module to take ownership of the pin. The function enable bits impact output control only. Regardless of the setting of the function enable bits, both GPIO and peripherals can still sense the pin input. After a function is enabled, it is up to the PORT\_MUX registers as to which peripheral takes control.

Figure 13-14: PORT\_FER Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000013_9d8a83f634d38276e26a9e02d315d9b3671d84c1b7599ae5c356f8864ff411a6.png)

Table 13-17: PORT\_FER Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | PX15       | Port x Bit 15 Mode. The PORT_FER.PX15 bit indicates the operating mode for port x.                               |
| 14 (R/W)           | PX14       | Port x Bit 14 Mode. The PORT_FER.PX14 bit indicates the operating mode for port x. 0 GPIO Mode 1 Peripheral Mode |

Table 13-17: PORT\_FER Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | PX13       | Port x Bit 13 Mode. The PORT_FER.PX13 bit indicates the operating mode for port x.                             |
| 12 (R/W)           | PX12       | Port x Bit 12 Mode. The PORT_FER.PX12 bit indicates the operating mode for port x. 0 GPIO Mode                 |
| 11 (R/W)           | PX11       | Port x Bit 11 Mode. The PORT_FER.PX11 bit indicates the operating mode for port x. 0 GPIO Mode                 |
| 10 (R/W)           | PX10       | Port x Bit 10 Mode. The PORT_FER.PX10 bit indicates the operating mode for port x. 0 GPIO Mode                 |
| 9 (R/W)            | PX9        | 1 Peripheral Mode                                                                                              |
|                    |            | Port x Bit 9 Mode. The PORT_FER.PX9 bit indicates the operating mode for port x. 0 GPIO Mode 1 Peripheral Mode |
| 8 (R/W)            | PX8        | Port x Bit 8 Mode. The PORT_FER.PX8 bit indicates the operating mode for port x. 0 GPIO Mode 1 Peripheral Mode |
| 7 (R/W)            | PX7        | Port x Bit 7 Mode. The PORT_FER.PX7 bit indicates the operating mode for port x. 0 GPIO Mode                   |

Table 13-17: PORT\_FER Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | PX6        | Port x Bit 6 Mode. The PORT_FER.PX6 bit indicates the operating mode for port x.                               |
| 5 (R/W)            | PX5        | Port x Bit 5 Mode. The PORT_FER.PX5 bit indicates the operating mode for port x. 0 GPIO Mode                   |
| 4 (R/W)            | PX4        | Port x Bit 4 Mode. The PORT_FER.PX4 bit indicates the operating mode for port x. 0 GPIO Mode                   |
| 3 (R/W)            | PX3        | Port x Bit 3 Mode. PORT_FER.PX3 bit indicates the operating mode for port x. 0 GPIO Mode                       |
| 2 (R/W)            |            | The 1 Peripheral Mode                                                                                          |
|                    | PX2        | Port x Bit 2 Mode. The PORT_FER.PX2 bit indicates the operating mode for port x. 0 GPIO Mode 1 Peripheral Mode |
| 1 (R/W)            | PX1        | Port x Bit 1 Mode. The PORT_FER.PX1 bit indicates the operating mode for port x. 0 GPIO Mode 1 Peripheral Mode |
| 0 (R/W)            | PX0        | Port x Bit 0 Mode. The PORT_FER.PX0 bit indicates the operating mode for port x. 0 GPIO Mode                   |

## Port x Function Enable Clear Register

The PORT\_FER\_CLR register permits enabling GPIO mode for each bit and corresponding GPIO pin. Writing 1 to a bit in PORT\_FER\_CLR enables GPIO mode for the corresponding pin.

Figure 13-15: PORT\_FER\_CLR Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000014_e690e04ab322db3d300d320f677979cfc90d9f6f4332cc34940eeae54fcdb0be.png)

Table 13-18: PORT\_FER\_CLR Register Fields

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000015_1dd912289df164516614ab904bc84213bfea169b0cb2adeec66b9b4fc85d2bbe.png)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                |
|--------------------|------------|------------------------------------------------------------------------|
| 15 (R/W1C)         | PX15       | Port x Bit 15 Mode Clear. The PORT_FER_CLR.PX15 bit enables GPIO mode. |
| 14 (R/W1C)         | PX14       | Port x Bit 14 Mode Clear. The PORT_FER_CLR.PX14 bit enables GPIO mode. |
|                    |            | 1 Set Bit for GPIO                                                     |
|                    |            | Mode                                                                   |

Table 13-18: PORT\_FER\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W1C)         | PX13       | Port x Bit 13 Mode Clear. The PORT_FER_CLR.PX13 bit enables GPIO mode. 0 No Effect                                               |
| 12 (R/W1C)         | PX12       | Port x Bit 12 Mode Clear. The PORT_FER_CLR.PX12 bit enables GPIO mode. 0 No Effect                                               |
| 11 (R/W1C)         | PX11       | Port x Bit 11 Mode Clear. The PORT_FER_CLR.PX11 bit enables GPIO mode. 0 No Effect                                               |
| 10 (R/W1C)         | PX10       | Port x Bit 10 Mode Clear. The PORT_FER_CLR.PX10 bit enables GPIO mode. 0 No Effect                                               |
| 9 (R/W1C)          | PX9        | 1 Set Bit for GPIO Mode Port x Bit 9 Mode Clear. The PORT_FER_CLR.PX9 bit enables GPIO mode. 0 No Effect 1 Set Bit for GPIO Mode |
| 8 (R/W1C)          | PX8        | Port x Bit 8 Mode Clear. The PORT_FER_CLR.PX8 bit enables GPIO mode. 0 No Effect                                                 |
| 7 (R/W1C)          | PX7        | Port x Bit 7 Mode Clear. The PORT_FER_CLR.PX7 bit enables GPIO mode. 0 No Effect                                                 |

Table 13-18: PORT\_FER\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------|
| 6 (R/W1C)          | PX6        | Port x Bit 6 Mode Clear. The PORT_FER_CLR.PX6 bit enables GPIO mode. 0 No Effect                         |
| 5 (R/W1C)          | PX5        | Port x Bit 5 Mode Clear. The PORT_FER_CLR.PX5 bit enables GPIO mode. 0 No Effect 1 Set Bit for GPIO Mode |
| 4 (R/W1C)          | PX4        | Port x Bit 4 Mode Clear. The PORT_FER_CLR.PX4 bit enables GPIO mode. 0 No Effect                         |
| 3 (R/W1C)          | PX3        | Port x Bit 3 Mode Clear. The PORT_FER_CLR.PX3 bit enables GPIO mode. 0 No Effect                         |
| 2 (R/W1C)          | PX2        | 1 Set Bit for GPIO Mode Port x Bit 2 Mode Clear. The PORT_FER_CLR.PX2 bit enables GPIO mode. 0 No Effect |
| 1 (R/W1C)          | PX1        | Port x Bit 1 Mode Clear. The PORT_FER_CLR.PX1 bit enables GPIO mode. 0 No Effect                         |
| 0 (R/W1C)          | PX0        | 1 Set Bit for GPIO Mode Port x Bit 0 Mode Clear. The PORT_FER_CLR.PX0 bit enables GPIO mode. 0 No Effect |

## Port x Function Enable Set Register

The PORT\_FER\_SET register permits enabling peripheral mode for each bit and corresponding GPIO pin. Writing 1 to a bit in PORT\_FER\_SET enables peripheral mode for the corresponding pin.

Figure 13-16: PORT\_FER\_SET Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000016_8747dc93ec4b7a1ce9d06cb23668129f0cc9683b8f76227c1b021567c07ac933.png)

Table 13-19: PORT\_FER\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                    |
|--------------------|------------|----------------------------------------------------------------------------|
| 15 (R/W1S)         | PX15       | Port x Bit 15 Mode Set. The PORT_FER_SET.PX15 bit enables peripheral mode. |
| 14 (R/W1S)         | PX14       | Port x Bit 14 Mode Set. The PORT_FER_SET.PX14 bit enables peripheral mode. |
|                    |            | 1 Set Bit for Peripheral Mode                                              |

Table 13-19: PORT\_FER\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------|
| 13 (R/W1S)         | PX13       | Port x Bit 13 Mode Set. The PORT_FER_SET.PX13 bit enables peripheral mode. 0 No Effect                             |
| 12 (R/W1S)         | PX12       | Port x Bit 12 Mode Set. The PORT_FER_SET.PX12 bit enables peripheral mode. 0 No Effect                             |
| 11 (R/W1S)         | PX11       | Port x Bit 11 Mode Set. The PORT_FER_SET.PX11 bit enables peripheral mode. 0 No Effect                             |
| 10 (R/W1S)         | PX10       | Port x Bit 10 Mode Set. The PORT_FER_SET.PX10 bit enables peripheral mode. 0 No Effect                             |
| 9 (R/W1S)          | PX9        | 1 Set Bit for Peripheral Mode Port x Bit 9 Mode Set. The PORT_FER_SET.PX9 bit enables peripheral mode. 0 No Effect |
| 8 (R/W1S)          | PX8        | 1 Set Bit for Peripheral Mode Port x Bit 8 Mode Set. The PORT_FER_SET.PX8 bit enables peripheral mode. 0 No Effect |
| 7 (R/W1S)          | PX7        | 1 Set Bit for Peripheral Mode Port x Bit 7 Mode Set. The PORT_FER_SET.PX7 bit enables peripheral mode. 0 No Effect |

Table 13-19: PORT\_FER\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------|
| 6 (R/W1S)          | PX6        | Port x Bit 6 Mode Set. The PORT_FER_SET.PX6 bit enables peripheral mode. 0 No Effect                               |
| 5 (R/W1S)          | PX5        | Port x Bit 5 Mode Set. The PORT_FER_SET.PX5 bit enables peripheral mode. 0 No Effect 1 Set Bit for Peripheral Mode |
| 4 (R/W1S)          | PX4        | Port x Bit 4 Mode Set. The PORT_FER_SET.PX4 bit enables peripheral mode. 0 No Effect                               |
| 3 (R/W1S)          | PX3        | Port x Bit 3 Mode Set. The PORT_FER_SET.PX3 bit enables peripheral mode. 0 No Effect                               |
| 2 (R/W1S)          | PX2        | 1 Set Bit for Peripheral Mode Port x Bit 2 Mode Set. The PORT_FER_SET.PX2 bit enables peripheral mode. 0 No Effect |
| 1 (R/W1S)          | PX1        | Port x Bit 1 Mode Set. The PORT_FER_SET.PX1 bit enables peripheral mode. 0 No Effect                               |
| 0 (R/W1S)          | PX0        | 1 Set Bit for Peripheral Mode Port x Bit 0 Mode Set. The PORT_FER_SET.PX0 bit enables peripheral mode. 0 No Effect |

## Port x GPIO Input Enable Register

The PORT\_INEN , PORT\_INEN\_SET , and PORT\_INEN\_CLR registers enable or disable input drivers, which are required for using a GPIO pin in input mode.

Writes to the PORT\_INEN register affect the input drivers for all pins of the port. T o set or clear specific pin drivers without impacting other pin drivers of the port, use the PORT\_INEN\_SET and PORT\_INEN\_CLR registers.

If the input is enabled, reads from the PORT\_DATA , PORT\_DATA\_SET , or PORT\_DATA\_CLR registers return the state of the pins. However, the state of the output is not overwritten by the input. It is altered by software writes only. Input and output drivers can be enabled at the same time. In this case, a read of the data register returns the true value of the data register and not the pin state.

For more information, see the PORT\_DATA register description and the PORT\_DIR register description.

Figure 13-17: PORT\_INEN Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000017_229a93fd30a44b9ca4902514cee0c4c730ed6f8465a210ea9940060af702df9a.png)

Table 13-20: PORT\_INEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration     |
|--------------------|------------|-----------------------------|
| 15                 | PX15       | Port x Bit 15 Input Enable. |
| (R/W)              | PX15       | 0 Disable Input Driver      |
| (R/W)              | PX15       | 1 Enable Input Driver       |

Table 13-20: PORT\_INEN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration     |
|--------------------|------------|-----------------------------|
| 14 (R/W)           | PX14       | Port x Bit 14 Input Enable. |
| 14 (R/W)           | PX14       | 0 Disable Input Driver      |
| 14 (R/W)           | PX14       | 1 Enable Input Driver       |
| 13                 | PX13       | Port x Bit 13 Input Enable. |
| (R/W)              | PX13       | 0 Disable Input Driver      |
| 13                 | PX13       | 1 Enable Input Driver       |
| 12 (R/W)           | PX12       | Port x Bit 12 Input Enable. |
| 12 (R/W)           | PX12       | 0 Disable Input Driver      |
| 12 (R/W)           | PX12       | 1 Enable Input Driver       |
| 11 (R/W)           | PX11       | Port x Bit 11 Input Enable. |
| 11 (R/W)           | PX11       | 0 Disable Input Driver      |
| 11 (R/W)           | PX11       | 1 Enable Input Driver       |
| 10 (R/W)           | PX10       | Port x Bit 10 Input Enable. |
| 10 (R/W)           | PX10       | 0 Disable Input Driver      |
| 10 (R/W)           | PX10       | 1 Enable Input Driver       |
| 9                  | PX9        | Port x Bit 9 Input Enable.  |
| (R/W)              | PX9        | 0 Disable Input Driver      |
| (R/W)              | PX9        | 1 Enable Input Driver       |
| 8                  | PX8        | Port x Bit 8 Input Enable.  |
| (R/W)              |            | 0 Disable Input Driver      |
| (R/W)              |            | 1 Enable Input Driver       |
| 7                  | PX7        | Port x Bit 7 Input Enable.  |
| (R/W)              |            | 0 Disable Input Driver      |
| (R/W)              |            | 1 Enable Input Driver       |
| 6                  | PX6        | Port x Bit 6 Input Enable.  |
| (R/W)              |            | 0 Disable Input Driver      |
| (R/W)              |            | 1 Enable Input Driver       |
| 5                  | PX5        | Port x Bit 5 Input Enable.  |
| (R/W)              |            | 0 Disable Input Driver      |
| (R/W)              |            | 1 Enable Input Driver       |

Table 13-20: PORT\_INEN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                 |
|--------------------|------------|-------------------------------------------------------------------------|
| 4                  | PX4        | Port x Bit 4 Input Enable.                                              |
| 3 (R/W)            | PX3        | Port x Bit 3 Input Enable. 0 Disable Input Driver                       |
| 2 (R/W)            | PX2        | Input Enable. 0 Disable Input Driver                                    |
| 1                  | PX1        | Port x Bit 2 1 Enable Input Driver                                      |
| (R/W)              |            | Port x Bit 1 Input Enable. 0 Disable Input Driver 1 Enable Input Driver |
| 0 (R/W)            | PX0        | Port x Bit 0 Input Enable. 0 Disable Input Driver                       |
|                    |            | 1                                                                       |
|                    |            | Enable Input Driver                                                     |

## Port x GPIO Input Enable Clear Register

The PORT\_INEN\_CLR register disables the input drivers for GPIO pins. For more information, see the PORT\_INEN register description.

Figure 13-18: PORT\_INEN\_CLR Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000018_eb6bc84441c00190173622da942cb8e4309d4966e98f912cd18e7c362457c6a6.png)

Table 13-21: PORT\_INEN\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                   |
|--------------------|------------|-----------------------------------------------------------|
| 15                 | PX15       | Port x Bit 15 Input Enable Clear.                         |
| 14 (R/W1C)         | PX14       | Port x Bit 14 Input Enable Clear. 0 No Effect             |
| 13                 | PX13       | Port x Bit 13 Input Enable Clear.                         |
| (R/W1C)            |            | 0 No Effect 1 Clear Bit. Set to disable the input driver. |

Table 13-21: PORT\_INEN\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                       |
|--------------------|------------|-----------------------------------------------|
| 12 (R/W1C)         | PX12       | Port x Bit 12 Input Enable Clear.             |
| 12 (R/W1C)         | PX12       | 0 No Effect                                   |
| 12 (R/W1C)         | PX12       | 1 Clear Bit. Set to disable the input driver. |
| 11 (R/W1C)         | PX11       | Port x Bit 11 Input Enable Clear.             |
| 11 (R/W1C)         | PX11       | 0 No Effect                                   |
| 11 (R/W1C)         | PX11       | 1 Clear Bit. Set to disable the input driver. |
| 10 (R/W1C)         | PX10       | Port x Bit 10 Input Enable Clear.             |
| 10 (R/W1C)         | PX10       | 0 No Effect                                   |
| 10 (R/W1C)         | PX10       | 1 Clear Bit. Set to disable the input driver. |
| 9 (R/W1C)          | PX9        | Port x Bit 9 Input Enable Clear.              |
| 9 (R/W1C)          | PX9        | 0 No Effect                                   |
| 9 (R/W1C)          | PX9        | 1 Clear Bit. Set to disable the input driver. |
| 8 (R/W1C)          | PX8        | Port x Bit 8 Input Enable Clear.              |
| 8 (R/W1C)          | PX8        | 0 No Effect                                   |
| 8 (R/W1C)          | PX8        | 1 Clear Bit. Set to disable the input driver. |
| 7 (R/W1C)          | PX7        | Port x Bit 7 Input Enable Clear.              |
| 7 (R/W1C)          | PX7        | 0 No Effect                                   |
| 7 (R/W1C)          | PX7        | 1 Clear Bit. Set to disable the input driver. |
| 6                  | PX6        | Port x Bit 6 Input Enable Clear.              |
| (R/W1C)            | PX6        | 0 No Effect                                   |
| 6                  | PX6        | 1 Clear Bit. Set to disable the input driver. |
| 5 (R/W1C)          | PX5        | Port x Bit 5 Input Enable Clear.              |
| 5 (R/W1C)          | PX5        | 0 No Effect                                   |
| 4 (R/W1C)          | PX4        | Port x Bit 4 Input Enable Clear.              |
| 4 (R/W1C)          | PX4        | 0 No Effect                                   |
| 4 (R/W1C)          | PX4        | 1 Clear Bit. Set to disable the input driver. |
| 3                  | PX3        | Port x Bit 3 Input Enable Clear.              |
| (R/W1C)            | PX3        | 0 No Effect                                   |
| 3                  | PX3        | 1 Clear Bit. Set to disable the input driver. |

Table 13-21: PORT\_INEN\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                        |
|--------------------|------------|--------------------------------------------------------------------------------|
| 2                  | PX2        | Port x Bit 2 Input Enable Clear.                                               |
| 1 (R/W1C)          | PX1        | Port x Bit 1 Input Enable Clear. 0 No Effect                                   |
| 0                  | PX0        | 1 Clear Bit. Set to disable the input driver. Port x Bit 0 Input Enable Clear. |
| (R/W1C)            |            | 0 No Effect 1 Clear Bit. Set to disable the input driver.                      |

## Port x GPIO Input Enable Set Register

The PORT\_INEN\_SET register enables input drivers for GPIO pins. For more information, see the PORT\_INEN register description.

Figure 13-19: PORT\_INEN\_SET Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000019_144ed72b568c687914eb6fe6f2e745b0cd0b6d1072a24f8dae4d625a2b14d103.png)

Table 13-22: PORT\_INEN\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                    |
|--------------------|------------|----------------------------------------------------------------------------|
| 15                 | PX15       | Port x Bit 15 Input Enable Set.                                            |
| 14 (R/W1S)         | PX14       | Port x Bit 14 Input Enable Set. 0 No Effect                                |
| 13                 | PX13       | 1 Set Bit. Set to enable the input driver. Port x Bit 13 Input Enable Set. |
| (R/W1S)            |            | 0 No Effect 1 Set Bit. Set to enable the input driver.                     |

Table 13-22: PORT\_INEN\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                    |
|--------------------|------------|--------------------------------------------|
| 12 (R/W1S)         | PX12       | Port x Bit 12 Input Enable Set.            |
| 12 (R/W1S)         | PX12       | 0 No Effect                                |
| 12 (R/W1S)         | PX12       | 1 Set Bit. Set to enable the input driver. |
| 11 (R/W1S)         | PX11       | Port x Bit 11 Input Enable Set.            |
| 11 (R/W1S)         | PX11       | 0 No Effect                                |
| 11 (R/W1S)         | PX11       | 1 Set Bit. Set to enable the input driver. |
| 10 (R/W1S)         | PX10       | Port x Bit 10 Input Enable Set.            |
| 10 (R/W1S)         | PX10       | 0 No Effect                                |
| 10 (R/W1S)         | PX10       | 1 Set Bit. Set to enable the input driver. |
| 9                  | PX9        | Port x Bit 9 Input Enable Set.             |
| (R/W1S)            | PX9        | 0 No Effect                                |
| 9                  | PX9        | 1 Set Bit. Set to enable the input driver. |
| 8 (R/W1S)          | PX8        | Port x Bit 8 Input Enable Set.             |
| 8 (R/W1S)          | PX8        | 0 No Effect                                |
| 8 (R/W1S)          | PX8        | 1 Set Bit. Set to enable the input driver. |
| 7 (R/W1S)          | PX7        | Port x Bit 7 Input Enable Set.             |
| 7 (R/W1S)          | PX7        | 0 No Effect                                |
| 7 (R/W1S)          | PX7        | 1 Set Bit. Set to enable the input driver. |
| 6                  | PX6        | Port x Bit 6 Input Enable Set.             |
| (R/W1S)            |            | 0 No Effect                                |
| (R/W1S)            |            | 1 Set Bit. Set to enable the input driver. |
| 5                  | PX5        | Port x Bit 5 Input Enable Set.             |
| (R/W1S)            |            | 0 No Effect                                |
| (R/W1S)            |            | 1 Set Bit. Set to enable the input driver. |
| 4                  | PX4        | Port x Bit 4 Input Enable Set.             |
| (R/W1S)            |            | 0 No Effect                                |
| (R/W1S)            |            | 1 Set Bit. Set to enable the input driver. |
| 3                  | PX3        | Port x Bit 3 Input Enable Set.             |
| (R/W1S)            |            | 0 No Effect                                |
| (R/W1S)            |            | 1 Set Bit. Set to enable the input driver. |

Table 13-22: PORT\_INEN\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                   |
|--------------------|------------|---------------------------------------------------------------------------|
| 2                  | PX2        | Port x Bit 2 Input Enable Set.                                            |
| 1 (R/W1S)          | PX1        | Port x Bit 1 Input Enable Set. 0 No Effect                                |
| 0                  | PX0        | 1 Set Bit. Set to enable the input driver. Port x Bit 0 Input Enable Set. |
| (R/W1S)            |            | 0 No Effect 1 Set Bit. Set to enable the input driver.                    |

## Port x GPIO Lock Register

The PORT\_LOCK register enables (unlocks) or disables (locks) write access selectively for the PORT control registers.

Figure 13-20: PORT\_LOCK Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000020_022c5ce09484720ef5d6e367e84758b474864278ed2c760aa662c8ea6a7e11d7.png)

Table 13-23: PORT\_LOCK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the PORT_LOCK.LOCK bit is set, the PORT_LOCK register is read only (locked). |
| 5 (R/W)            | POLAR      | Polarity Lock. The PORT_LOCK.POLAR disables write access to the PORT_POL , PORT_POL_SET , and PORT_POL_CLR registers.                       |
| 4 (R/W)            | INEN       | Input Enable Lock. The PORT_LOCK.INEN disables write access to the PORT_INEN , PORT_INEN_SET , and PORT_INEN_CLR registers. 0 Unlock INEN   |
| 4 (R/W)            | INEN       | 1 Lock INEN                                                                                                                                 |
| 4 (R/W)            | INEN       |                                                                                                                                             |

Table 13-23: PORT\_LOCK Register Fields (Continued)

| Bit No. (Access)   | Description/Enumeration                                                                                                                  |
|--------------------|------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | Direction Lock. The PORT_LOCK.DIR disables write access to the PORT_DIR , PORT_DIR_SET , PORT_DIR_CLR registers.                         |
| 2 (R/W)            | Data, TGL Lock. The PORT_LOCK.DATA disables write access to the PORT_DATA , PORT_DATA_SET , PORT_DATA_CLR , and PORT_DATA_TGL registers. |
| 1 (R/W)            | Function Multiplexer Lock. The PORT_LOCK.MUX disables write accesses to the PORT_MUX register.                                           |
| 0 (R/W)            | Function Enable Lock. The PORT_LOCK.FER disables write access to the PORT_FER , PORT_FER_SET , and PORT_FER_CLR registers. 0 Unlock FER  |

## Port x Multiplexer Control Register

When a pin is in peripheral mode (not GPIO mode), the PORT\_MUX register controls which peripheral takes ownership of a pin. Ports may have multiple, different peripheral functions. T wo bits are required to describe every multiplexer on an individual pin-by-pin scheme. For example, bit 0 and bit 1 of the PORT\_MUX register control the multiplexer of pin 0, bit 2 and bit 3 of PORT\_MUX control the multiplexer of pin 1, and so on. The value of any PORT\_MUX bit has no effect on the port pins when the associated bit in the PORT\_FER register is 0 (selects GPIO mode). Even if a port has only one function, the PORT\_MUX register is still present. For single function ports (no multiplexing is needed), leave the PORT\_MUX bits at 0 (default). For all PORT\_MUX bit fields: 00 = default/reset peripheral option, 01 = first alternate peripheral option, 10 = second alternate peripheral option, and 11 = third alternate peripheral option.

See the processor data sheet for details regarding the peripheral options associated with each port.

Figure 13-21: PORT\_MUX Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000021_dd7daeba5d496a7c9d2098a1d3d179980d485045f15073979e90f97b7b736a72.png)

Table 13-24: PORT\_MUX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                |
|--------------------|------------|------------------------------------------------------------------------|
| 31:30              | MUX15      | Mux for Port x Bit 15.                                                 |
| (R/W)              |            | The PORT_MUX.MUX15 bit provides multiplexer control for port x bit 15. |

Table 13-24: PORT\_MUX Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                       |
|--------------------|------------|-----------------------------------------------------------------------------------------------|
| 29:28 (R/W)        | MUX14      | Mux for Port x Bit 14. The PORT_MUX.MUX14 bit provides multiplexer control for port x bit 14. |
| 27:26 (R/W)        | MUX13      | Mux for Port x Bit 13. The PORT_MUX.MUX13 bit provides multiplexer control for port x bit 13. |
| 25:24 (R/W)        | MUX12      | Mux for Port x Bit 12. The PORT_MUX.MUX12 bit provides multiplexer control for port x bit 12. |
| 23:22 (R/W)        | MUX11      | Mux for Port x Bit 11. The PORT_MUX.MUX11 bit provides multiplexer control for port x bit 11. |
| 21:20 (R/W)        | MUX10      | Mux for Port x Bit 10. The PORT_MUX.MUX10 bit provides multiplexer control for port x bit 10. |
| 19:18 (R/W)        | MUX9       | Mux for Port x Bit 9. The PORT_MUX.MUX9 bit provides multiplexer control for port x bit 9.    |
| 17:16 (R/W)        | MUX8       | Mux for Port x Bit 8. The PORT_MUX.MUX8 bit provides multiplexer control for port x bit 8.    |
| 15:14 (R/W)        | MUX7       | Mux for Port x Bit 7. The PORT_MUX.MUX7 bit provides multiplexer control for port x bit 7.    |
| 13:12 (R/W)        | MUX6       | Mux for Port x Bit 6. The PORT_MUX.MUX6 bit provides multiplexer control for port x bit 6.    |
| 11:10 (R/W)        | MUX5       | Mux for Port x Bit 5. The PORT_MUX.MUX5 bit provides multiplexer control for port x bit 5.    |
| 9:8 (R/W)          | MUX4       | Mux for Port x Bit 4. The PORT_MUX.MUX4 bit provides multiplexer control for port x bit 4.    |
| 7:6 (R/W)          | MUX3       | Mux for Port x Bit 3. The PORT_MUX.MUX3 bit provides multiplexer control for port x bit 3.    |
| 5:4 (R/W)          | MUX2       | Mux for Port x Bit 2. The PORT_MUX.MUX2 bit provides multiplexer control for port x bit 2.    |
| 3:2 (R/W)          | MUX1       | Mux for Port x Bit 1. The PORT_MUX.MUX1 bit provides multiplexer control for port x bit 1.    |
| 1:0 (R/W)          | MUX0       | Mux for Port x Bit 0. The PORT_MUX.MUX0 bit provides multiplexer control for port x bit 0.    |

## Port x GPIO Polarity Invert Register

The PORT\_POL , PORT\_POL\_SET , and PORT\_POL\_CLR registers enable or disable inverting polarity of GPIO signals. To invert polarity of peripheral signals, use the inversion selection programming in the signal's corresponding module.

Writes to the PORT\_POL register affect the polarity inversion selection of all pins of the port. T o enable or disable polarity inversion for specific pins without impacting other pins of the port, use the PORT\_POL\_SET and PORT\_POL\_CLR registers.

Setting a bit in the PORT\_POL register enables polarity inversion on the corresponding inversion GPIO pin, making the pin active-low or falling-edge sensitive. Clearing a bit in the PORT\_POL register disables polarity (default state) on the corresponding GPIO pin, making it active-high or rising-edge sensitive.

Figure 13-22: PORT\_POL Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000022_8e1e8f3b8ff005d51b5ff7c206b24f907ee7b38ff587397d9ecbb55859a17f77.png)

Table 13-25: PORT\_POL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                          |
|--------------------|------------|----------------------------------------------------------------------------------|
| 15 (R/W)           | PX15       | Port x Bit 15 Polarity Invert. The PORT_POL.PX15 bit enables polarity inversion. |
| 15 (R/W)           | PX15       | 0 No Invert. GPIO is active high or rising edge sensitive.                       |
| 15 (R/W)           | PX15       | 1 Invert. GPIO is active low or falling edge sensitive.                          |

Table 13-25: PORT\_POL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | PX14       | Port x Bit 14 Polarity Invert. The PORT_POL.PX14 bit enables polarity inversion. 0 No Invert. GPIO is active high or rising edge sensitive.                                                         |
| 13 (R/W)           | PX13       | Port x Bit 13 Polarity Invert. The PORT_POL.PX13 bit enables polarity inversion. 0 No Invert. GPIO is active high or rising edge sensitive. 1 Invert. GPIO is active low or falling edge sensitive. |
| 12 (R/W)           | PX12       | Port x Bit 12 Polarity Invert. The PORT_POL.PX12 bit enables polarity inversion. 0 No Invert. GPIO is active high or rising edge sensitive.                                                         |
| 11 (R/W)           | PX11       | 1 Invert. GPIO is active low or falling edge sensitive. Port x Bit 11 Polarity Invert. The PORT_POL.PX11 bit enables polarity inversion. 0 No Invert. GPIO is active high or rising edge sensitive. |
| 10 (R/W)           | PX10       | 1 Invert. GPIO is active low or falling edge sensitive.                                                                                                                                             |
|                    |            | Port x Bit 10 Polarity Invert. The PORT_POL.PX10 bit enables polarity inversion. 0 No Invert. GPIO is active high or rising edge sensitive. 1 Invert. GPIO is active low or falling edge sensitive. |
| 9 (R/W)            | PX9        | Port x Bit 9 Polarity Invert. The PORT_POL.PX9 bit enables polarity inversion. 0 No Invert. GPIO is active high or rising edge sensitive. 1 Invert. GPIO is active low or falling edge sensitive.   |
| 8 (R/W)            | PX8        | Port x Bit 8 Polarity Invert. The PORT_POL.PX8 bit enables polarity inversion. 0 No Invert. GPIO is active high or rising edge sensitive.                                                           |

Table 13-25: PORT\_POL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | PX7        | Port x Bit 7 Polarity Invert. The PORT_POL.PX7 bit enables polarity inversion. 0 No Invert. GPIO is active high or rising edge sensitive.                                                         |
| 6 (R/W)            | PX6        | Port x Bit 6 Polarity Invert. The PORT_POL.PX6 bit enables polarity inversion. 0 No Invert. GPIO is active high or rising edge sensitive.                                                         |
| 5 (R/W)            | PX5        | Port x Bit 5 Polarity Invert. The PORT_POL.PX5 bit enables polarity inversion. 0 No Invert. GPIO is active high or rising edge sensitive.                                                         |
| 4 (R/W)            | PX4        | 1 Invert. GPIO is active low or falling edge sensitive. Port x Bit 4 Polarity Invert. The PORT_POL.PX4 bit enables polarity inversion.                                                            |
| 3 (R/W)            | PX3        | 0 No Invert. GPIO is active high or rising edge sensitive. 1 Invert. GPIO is active low or falling edge sensitive.                                                                                |
|                    |            | Port x Bit 3 Polarity Invert. The PORT_POL.PX3 bit enables polarity inversion. 0 No Invert. GPIO is active high or rising edge sensitive. 1 Invert. GPIO is active low or falling edge sensitive. |
| 2 (R/W)            | PX2        | Port x Bit 2 Polarity Invert. The PORT_POL.PX2 bit enables polarity inversion. 0 No Invert. GPIO is active high or rising edge sensitive. 1 Invert. GPIO is active low or falling edge sensitive. |
| 1 (R/W)            | PX1        | Port x Bit 1 Polarity Invert. The PORT_POL.PX1 bit enables polarity inversion. 0 No Invert. GPIO is active high or rising edge sensitive.                                                         |

Table 13-25: PORT\_POL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                        |
|--------------------|------------|--------------------------------------------------------------------------------|
| 0 (R/W)            | PX0        | Port x Bit 0 Polarity Invert. The PORT_POL.PX0 bit enables polarity inversion. |
| 0 (R/W)            | PX0        | 0 No Invert. GPIO is active high or rising edge sensitive.                     |
| 0 (R/W)            | PX0        | 1 Invert. GPIO is active low or falling edge sensitive.                        |

## Port x GPIO Polarity Invert Clear Register

The PORT\_POL\_CLR register disables polarity inversion for GPIO pins. For more information, see the PORT\_POL register description.

Figure 13-23: PORT\_POL\_CLR Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000023_449dfb9399af4cb05bc05fad00db19ac811b6220b1b8d4eddcdb4b7270b73e4c.png)

Table 13-26: PORT\_POL\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                           |
|--------------------|------------|-------------------------------------------------------------------|
| 15                 | PX15       | Port x Bit 15 Polarity Invert Clear.                              |
| 14 (R/W1C)         | PX14       | Port x Bit 14 Polarity Invert Clear. 0 No Effect                  |
| 13                 | PX13       | Port x Bit 13 Polarity Invert Clear.                              |
| (R/W1C)            |            | 0 No Effect 1 Clear Bit. Set to disable GPIO pin polarity invert. |

Table 13-26: PORT\_POL\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |                                      | Description/Enumeration                             |
|--------------------|------------|--------------------------------------|-----------------------------------------------------|
| 12 (R/W1C)         | PX12       | Port x Bit 12 Polarity Invert Clear. | Port x Bit 12 Polarity Invert Clear.                |
| 12 (R/W1C)         | PX12       | 0                                    | No Effect                                           |
| 12 (R/W1C)         | PX12       | 1                                    | Clear Bit. Set to disable GPIO pin polarity invert. |
| 11 (R/W1C)         | PX11       | Port x Bit 11 Polarity Invert Clear. | Port x Bit 11 Polarity Invert Clear.                |
| 11 (R/W1C)         | PX11       | 0                                    | No Effect                                           |
| 11 (R/W1C)         | PX11       | 1                                    | Clear Bit. Set to disable GPIO pin polarity invert. |
| 10 (R/W1C)         | PX10       | Port x Bit 10 Polarity Invert Clear. | Port x Bit 10 Polarity Invert Clear.                |
| 10 (R/W1C)         | PX10       | 0                                    | No Effect                                           |
| 10 (R/W1C)         | PX10       | 1                                    | Clear Bit. Set to disable GPIO pin polarity invert. |
| 9 (R/W1C)          | PX9        | Port x Bit 9 Polarity Invert Clear.  | Port x Bit 9 Polarity Invert Clear.                 |
| 9 (R/W1C)          | PX9        | 0                                    | No Effect                                           |
| 9 (R/W1C)          | PX9        | 1                                    | Clear Bit. Set to disable GPIO pin polarity invert. |
| 8 (R/W1C)          | PX8        | Port x Bit 8 Polarity Invert Clear.  | Port x Bit 8 Polarity Invert Clear.                 |
| 8 (R/W1C)          | PX8        | 0                                    | No Effect                                           |
| 8 (R/W1C)          | PX8        | 1                                    | Clear Bit. Set to disable GPIO pin polarity invert. |
| 7 (R/W1C)          | PX7        | Port x Bit 7 Polarity Invert Clear.  | Port x Bit 7 Polarity Invert Clear.                 |
| 7 (R/W1C)          | PX7        | 0                                    | No Effect                                           |
| 7 (R/W1C)          | PX7        | 1                                    | Clear Bit. Set to disable GPIO pin polarity invert. |
| 6                  | PX6        | Port x Bit 6 Polarity Invert Clear.  | Port x Bit 6 Polarity Invert Clear.                 |
| (R/W1C)            |            | 0                                    | No Effect                                           |
| (R/W1C)            |            | 1                                    | Clear Bit. Set to disable GPIO pin polarity invert. |
| 5                  | PX5        | Port x Bit 5 Polarity Invert Clear.  | Port x Bit 5 Polarity Invert Clear.                 |
| (R/W1C)            |            | 0                                    | No Effect                                           |
| (R/W1C)            |            | 1                                    | Clear Bit. Set to disable GPIO pin polarity invert. |
| 4                  | PX4        | Port x Bit 4 Polarity Invert Clear.  | Port x Bit 4 Polarity Invert Clear.                 |
| (R/W1C)            |            | 0                                    | No Effect                                           |
| (R/W1C)            |            | 1                                    | Clear Bit. Set to disable GPIO pin polarity invert. |
| 3                  | PX3        | Port x Bit 3 Polarity Invert Clear.  | Port x Bit 3 Polarity Invert Clear.                 |
| (R/W1C)            |            | 0                                    | No Effect                                           |
| (R/W1C)            |            | 1                                    | Clear Bit. Set to disable GPIO pin polarity invert. |

Table 13-26: PORT\_POL\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------|
| 2                  | PX2        | Port x Bit 2 Polarity Invert Clear.                                                       |
| 1 (R/W1C)          | PX1        | Port x Bit 1 Polarity Invert Clear. 0 No Effect                                           |
| 0                  | PX0        | 1 Clear Bit. Set to disable GPIO pin polarity invert. Port x Bit 0 Polarity Invert Clear. |
| (R/W1C)            |            | 0 No Effect 1 Clear Bit. Set to disable GPIO pin polarity invert.                         |

## Port x GPIO Polarity Invert Set Register

The PORT\_POL\_SET register enables polarity inversion for GPIO pins. For more information, see the PORT\_POL register description.

Figure 13-24: PORT\_POL\_SET Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000024_c3f327a14be89a8954c94ff0def79f44359cd402de415b41cd69f64900962527.png)

Table 13-27: PORT\_POL\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------|
| 15 (R/W1S)         | PX15       | Port x Bit 15 Polarity Invert Set. The PORT_POL_SET.PX15 bit enables pin polarity inversion. |
| 14 (R/W1S)         | PX14       | Port x Bit 14 Polarity Invert Set. The PORT_POL_SET.PX14 bit enables pin polarity inversion. |
|                    |            | 0 No Effect                                                                                  |
|                    |            | 1 Set Bit. Set to enable GPIO pin polarity invert.                                           |

Table 13-27: PORT\_POL\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W1S)         | PX13       | Port x Bit 13 Polarity Invert Set. The PORT_POL_SET.PX13 bit enables pin polarity inversion. 0 No Effect                                                    |
| 12 (R/W1S)         | PX12       | Port x Bit 12 Polarity Invert Set. The PORT_POL_SET.PX12 bit enables pin polarity inversion. 0 No Effect 1 Set Bit. Set to enable GPIO pin polarity invert. |
| 11 (R/W1S)         | PX11       | Port x Bit 11 Polarity Invert Set. The PORT_POL_SET.PX11 bit enables pin polarity inversion. 0 No Effect                                                    |
| 10 (R/W1S)         | PX10       | 1 Set Bit. Set to enable GPIO pin polarity invert. Port x Bit 10 Polarity Invert Set. The PORT_POL_SET.PX10 bit enables pin polarity inversion.             |
| 9 (R/W1S)          | PX9        | 0 No Effect 1 Set Bit. Set to enable GPIO pin polarity invert.                                                                                              |
|                    |            | Port x Bit 9 Polarity Invert Set. The PORT_POL_SET.PX9 bit enables pin polarity inversion. 0 No Effect 1 Set Bit. Set to enable GPIO pin polarity invert.   |
| 8 (R/W1S)          | PX8        | Port x Bit 8 Polarity Invert Set. The PORT_POL_SET.PX8 bit enables pin polarity inversion. 0 No Effect 1 Set Bit. Set to enable GPIO pin polarity invert.   |
| 7 (R/W1S)          | PX7        | Port x Bit 7 Polarity Invert Set. The PORT_POL_SET.PX7 bit enables pin polarity inversion. 0 No Effect                                                      |

Table 13-27: PORT\_POL\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W1S)          | PX6        | Port x Bit 6 Polarity Invert Set. The PORT_POL_SET.PX6 bit enables pin polarity inversion. 0 No Effect                                                    |
| 5 (R/W1S)          | PX5        | Port x Bit 5 Polarity Invert Set. The PORT_POL_SET.PX5 bit enables pin polarity inversion. 0 No Effect 1 Set Bit. Set to enable GPIO pin polarity invert. |
| 4 (R/W1S)          | PX4        | Port x Bit 4 Polarity Invert Set. The PORT_POL_SET.PX4 bit enables pin polarity inversion. 0 No Effect                                                    |
| 3 (R/W1S)          | PX3        | 1 Set Bit. Set to enable GPIO pin polarity invert. Port x Bit 3 Polarity Invert Set. The PORT_POL_SET.PX3 bit enables pin polarity inversion.             |
| 2 (R/W1S)          | PX2        | 0 No Effect 1 Set Bit. Set to enable GPIO pin polarity invert.                                                                                            |
|                    |            | Port x Bit 2 Polarity Invert Set. The PORT_POL_SET.PX2 bit enables pin polarity inversion. 0 No Effect 1 Set Bit. Set to enable GPIO pin polarity invert. |
| 1 (R/W1S)          | PX1        | Port x Bit 1 Polarity Invert Set. The PORT_POL_SET.PX1 bit enables pin polarity inversion. 0 No Effect 1 Set Bit. Set to enable GPIO pin polarity invert. |
| 0 (R/W1S)          | PX0        | Port x Bit 0 Polarity Invert Set. The PORT_POL_SET.PX0 bit enables pin polarity inversion. 0 No Effect                                                    |

## Port x GPIO Trigger Toggle Register

The PORT\_TRIG\_TGL register permits toggling the state of output GPIO pins in response to a trigger from the TRU for the corresponding port. Setting bits in the PORT\_TRIG\_TGL register enables triggers to toggle the state of those specific pins without impacting other pins of the port.

Figure 13-25: PORT\_TRIG\_TGL Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000025_56a31dce128ca383408ceef6ee22087ba422689fcd046404bf9c7f2454c5fbf0.png)

Table 13-28: PORT\_TRIG\_TGL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | PX15       | PX15 Data Toggle on Trigger. The PORT_TRIG_TGL.PX15 bit enables triggers to toggle the state of the pin. |
| 14 (R/W)           | PX14       | PX14 Data Toggle on Trigger. The PORT_TRIG_TGL.PX14 bit enables triggers to toggle the state of the pin. |
| 13 (R/W)           | PX13       | PX13 Data Toggle on Trigger. The PORT_TRIG_TGL.PX13 bit enables triggers to toggle the state of the pin. |
| 12 (R/W)           | PX12       | PX12 Data Toggle on Trigger. The PORT_TRIG_TGL.PX12 bit enables triggers to toggle the state of the pin. |
| 11 (R/W)           | PX11       | PX11 Data Toggle on Trigger. The PORT_TRIG_TGL.PX11 bit enables triggers to toggle the state of the pin. |

Table 13-28: PORT\_TRIG\_TGL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------|
| 10 (R/W)           | PX10       | PX10 Data Toggle on Trigger. The PORT_TRIG_TGL.PX10 bit enables triggers to toggle the state of the pin. |
| 9 (R/W)            | PX9        | PX9 Data Toggle on Trigger. The PORT_TRIG_TGL.PX9 bit enables triggers to toggle the state of the pin.   |
| 8 (R/W)            | PX8        | PX8 Data Toggle on Trigger. The PORT_TRIG_TGL.PX8 bit enables triggers to toggle the state of the pin.   |
| 7 (R/W)            | PX7        | PX7 Data Toggle on Trigger. The PORT_TRIG_TGL.PX7 bit enables triggers to toggle the state of the pin.   |
| 6 (R/W)            | PX6        | PX6 Data Toggle on Trigger. The PORT_TRIG_TGL.PX6 bit enables triggers to toggle the state of the pin.   |
| 5 (R/W)            | PX5        | PX5 Data Toggle on Trigger. The PORT_TRIG_TGL.PX5 bit enables triggers to toggle the state of the pin.   |
| 4 (R/W)            | PX4        | PX4 Data Toggle on Trigger. The PORT_TRIG_TGL.PX4 bit enables triggers to toggle the state of the pin.   |
| 3 (R/W)            | PX3        | PX3 Data Toggle on Trigger. The PORT_TRIG_TGL.PX3 bit enables triggers to toggle the state of the pin.   |
| 2 (R/W)            | PX2        | PX2 Data Toggle on Trigger. The PORT_TRIG_TGL.PX2 bit enables triggers to toggle the state of the pin.   |
| 1 (R/W)            | PX1        | PX1 Data Toggle on Trigger. The PORT_TRIG_TGL.PX1 bit enables triggers to toggle the state of the pin.   |
| 0 (R/W)            | PX0        | PX0 Data Toggle on Trigger. The PORT_TRIG_TGL.PX0 bit enables triggers to toggle the state of the pin.   |

## ADSP-2184x PINT Register Descriptions

The Pin Interrupt module (PINT) contains the following registers.

Table 13-29: ADSP-2184x PINT Register List

| Name          | Description                |
|---------------|----------------------------|
| PINT_ASSIGN   | PINT Assign Register       |
| PINT_EDGE_CLR | PINT Edge Clear Register   |
| PINT_EDGE_SET | PINT Edge Set Register     |
| PINT_INV_CLR  | PINT Invert Clear Register |

Table 13-29: ADSP-2184x PINT Register List (Continued)

| Name          | Description              |
|---------------|--------------------------|
| PINT_INV_SET  | PINT Invert Set Register |
| PINT_LATCH    | PINT Latch Register      |
| PINT_MSK_CLR  | PINT Mask Clear Register |
| PINT_MSK_SET  | PINT Mask Set Register   |
| PINT_PINSTATE | PINT Pin State Register  |
| PINT_REQ      | PINT Request Register    |

## PINT Assign Register

The PINT\_ASSIGN register controls the pin-to-interrupt request assignment in a byte-wide manner. This register consists of four control bytes that each function as a multiplexer control.

The PINT ports are subdivided into 8-bit half ports, resulting in lower and upper half 8-bit units. Using the multiplexers controlled by the PINT\_ASSIGN register, the lower half units of eight pins can be forwarded to either byte 0 or byte 2 of either associated PINT block. The upper half units can be forwarded to either byte 1 or byte 3 of the PINT block, without further restrictions.

Figure 13-26: PINT\_ASSIGN Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000026_347d366ba96acaaf9d7a631d2033f19adda80bc1d7f08076a240213f593a905f.png)

Table 13-30: PINT\_ASSIGN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                               |
|--------------------|------------|-----------------------------------------------------------------------|
| 31:25 (R/NW)       | B3MSB0     | Bits [7:1] of B3MAP.                                                  |
| 24 (R/W)           | B3MAP      | Byte 3 Mapping. 0 B3MAP_PAH. Byte 3 = PA.H 1 B3MAP_PBH. Byte 3 = PB.H |
| 23:17 (R/NW)       | B2MSB0     | Bits [7:1] of B2MAP.                                                  |
| 16 (R/W)           | B2MAP      | Byte 2 Mapping. 0 B2MAP_PAL. Byte 2 = PA.L 1 B2MAP_PBL. Byte 2 = PB.L |
| 15:9 (R/NW)        | B1MSB0     | Bits [7:1] of B1MAP.                                                  |

Table 13-30: PINT\_ASSIGN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                    |
|--------------------|------------|--------------------------------------------|
| 8                  | B1MAP      | Byte 1 Mapping.                            |
| 7:1 (R/NW)         | B0MSB0     | Bits [7:1] of B0MAP.                       |
| 0 (R/W)            | B0MAP      | Byte 0 Mapping. 0 B0MAP_PAL. Byte 0 = PA.L |

## PINT Edge Clear Register

The PINT\_EDGE\_CLR register permits selecting level-sensitive interrupts. Writing 1 to a bit in PINT\_EDGE\_CLR enables level sensitivity for the corresponding pin interrupt.

Figure 13-27: PINT\_EDGE\_CLR Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000027_bd277e75b9331e8acbd2b1972dc3021be16cb96b4cff41376ba95ffcc7302488.png)

Table 13-31: PINT\_EDGE\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------|
| 31 (R/W1C)         | PIQ31      | Pin Interrupt 31 Level. Set the PINT_EDGE_CLR.PIQ31 bit to enable level sensitivity. |
| 30 (R/W1C)         | PIQ30      | Pin Interrupt 30 Level. Set the PINT_EDGE_CLR.PIQ30 bit to enable level sensitivity. |
| 29 (R/W1C)         | PIQ29      | Pin Interrupt 29 Level. Set the PINT_EDGE_CLR.PIQ29 bit to enable level sensitivity. |
| 28 (R/W1C)         | PIQ28      | Pin Interrupt 28 Level. Set the PINT_EDGE_CLR.PIQ28 bit to enable level sensitivity. |
| 27 (R/W1C)         | PIQ27      | Pin Interrupt 27 Level. Set the PINT_EDGE_CLR.PIQ27 bit to enable level sensitivity. |
| 26 (R/W1C)         | PIQ26      | Pin Interrupt 26 Level. Set the PINT_EDGE_CLR.PIQ26 bit to enable level sensitivity. |
| 25 (R/W1C)         | PIQ25      | Pin Interrupt 25 Level. Set the PINT_EDGE_CLR.PIQ25 bit to enable level sensitivity. |
| 24 (R/W1C)         | PIQ24      | Pin Interrupt 24 Level. Set the PINT_EDGE_CLR.PIQ24 bit to enable level sensitivity. |
| 23 (R/W1C)         | PIQ23      | Pin Interrupt 23 Level. Set the PINT_EDGE_CLR.PIQ23 bit to enable level sensitivity. |
| 22 (R/W1C)         | PIQ22      | Pin Interrupt 22 Level. Set the PINT_EDGE_CLR.PIQ22 bit to enable level sensitivity. |
| 21 (R/W1C)         | PIQ21      | Pin Interrupt 21 Level. Set the PINT_EDGE_CLR.PIQ21 bit to enable level sensitivity. |
| 20 (R/W1C)         | PIQ20      | Pin Interrupt 20 Level. Set the PINT_EDGE_CLR.PIQ20 bit to enable level sensitivity. |
| 19 (R/W1C)         | PIQ19      | Pin Interrupt 19 Level. Set the PINT_EDGE_CLR.PIQ19 bit to enable level sensitivity. |
| 18 (R/W1C)         | PIQ18      | Pin Interrupt 18 Level. Set the PINT_EDGE_CLR.PIQ18 bit to enable level sensitivity. |
| 17 (R/W1C)         | PIQ17      | Pin Interrupt 17 Level. Set the PINT_EDGE_CLR.PIQ17 bit to enable level sensitivity. |
| 16 (R/W1C)         | PIQ16      | Pin Interrupt 16 Level. Set the PINT_EDGE_CLR.PIQ16 bit to enable level sensitivity. |

Table 13-31: PINT\_EDGE\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------|
| 15 (R/W1C)         | PIQ15      | Pin Interrupt 15 Level. Set the PINT_EDGE_CLR.PIQ15 bit to enable level sensitivity. |
| 14 (R/W1C)         | PIQ14      | Pin Interrupt 14 Level. Set the PINT_EDGE_CLR.PIQ14 bit to enable level sensitivity. |
| 13 (R/W1C)         | PIQ13      | Pin Interrupt 13 Level. Set the PINT_EDGE_CLR.PIQ13 bit to enable level sensitivity. |
| 12 (R/W1C)         | PIQ12      | Pin Interrupt 12 Level. Set the PINT_EDGE_CLR.PIQ12 bit to enable level sensitivity. |
| 11 (R/W1C)         | PIQ11      | Pin Interrupt 11 Level. Set the PINT_EDGE_CLR.PIQ11 bit to enable level sensitivity. |
| 10 (R/W1C)         | PIQ10      | Pin Interrupt 10 Level. Set the PINT_EDGE_CLR.PIQ10 bit to enable level sensitivity. |
| 9 (R/W1C)          | PIQ9       | Pin Interrupt 9 Level. Set the PINT_EDGE_CLR.PIQ9 bit to enable level sensitivity.   |
| 8 (R/W1C)          | PIQ8       | Pin Interrupt 8 Level. Set the PINT_EDGE_CLR.PIQ8 bit to enable level sensitivity.   |
| 7 (R/W1C)          | PIQ7       | Pin Interrupt 7 Level. Set the PINT_EDGE_CLR.PIQ7 bit to enable level sensitivity.   |
| 6 (R/W1C)          | PIQ6       | Pin Interrupt 6 Level. Set the PINT_EDGE_CLR.PIQ6 bit to enable level sensitivity.   |
| 5 (R/W1C)          | PIQ5       | Pin Interrupt 5 Level. Set the PINT_EDGE_CLR.PIQ5 bit to enable level sensitivity.   |
| 4 (R/W1C)          | PIQ4       | Pin Interrupt 4 Level. Set the PINT_EDGE_CLR.PIQ4 bit to enable level sensitivity.   |
| 3 (R/W1C)          | PIQ3       | Pin Interrupt 3 Level. Set the PINT_EDGE_CLR.PIQ3 bit to enable level sensitivity.   |
| 2 (R/W1C)          | PIQ2       | Pin Interrupt 2 Level. Set the PINT_EDGE_CLR.PIQ2 bit to enable level sensitivity.   |
| 1 (R/W1C)          | PIQ1       | Pin Interrupt 1 Level. Set the PINT_EDGE_CLR.PIQ1 bit to enable level sensitivity.   |
| 0 (R/W1C)          | PIQ0       | Pin Interrupt 0 Level. Set the PINT_EDGE_CLR.PIQ0 bit to enable level sensitivity.   |

## PINT Edge Set Register

The PINT\_EDGE\_SET register permits selecting edge-sensitive interrupts. Writing 1 to a bit in PINT\_EDGE\_SET enables edge sensitivity for the corresponding pin interrupt.

Figure 13-28: PINT\_EDGE\_SET Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000028_1300f0b881512222322cb536b9181ae28c276369c9ca56a95b6c42b5fcc2aed9.png)

Table 13-32: PINT\_EDGE\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                            |
|--------------------|------------|------------------------------------------------------------------------------------|
| 31 (R/W1S)         | PIQ31      | Pin Interrupt 31 Edge. Set the PINT_EDGE_SET.PIQ31 bit to enable edge sensitivity. |
| 30 (R/W1S)         | PIQ30      | Pin Interrupt 30 Edge. Set the PINT_EDGE_SET.PIQ30 bit to enable edge sensitivity. |
| 29 (R/W1S)         | PIQ29      | Pin Interrupt 29 Edge. Set the PINT_EDGE_SET.PIQ29 bit to enable edge sensitivity. |
| 28 (R/W1S)         | PIQ28      | Pin Interrupt 28 Edge. Set the PINT_EDGE_SET.PIQ28 bit to enable edge sensitivity. |
| 27 (R/W1S)         | PIQ27      | Pin Interrupt 27 Edge. Set the PINT_EDGE_SET.PIQ27 bit to enable edge sensitivity. |
| 26 (R/W1S)         | PIQ26      | Pin Interrupt 26 Edge. Set the PINT_EDGE_SET.PIQ26 bit to enable edge sensitivity. |
| 25 (R/W1S)         | PIQ25      | Pin Interrupt 25 Edge. Set the PINT_EDGE_SET.PIQ25 bit to enable edge sensitivity. |
| 24 (R/W1S)         | PIQ24      | Pin Interrupt 24 Edge. Set the PINT_EDGE_SET.PIQ24 bit to enable edge sensitivity. |
| 23 (R/W1S)         | PIQ23      | Pin Interrupt 23 Edge. Set the PINT_EDGE_SET.PIQ23 bit to enable edge sensitivity. |
| 22 (R/W1S)         | PIQ22      | Pin Interrupt 22 Edge. Set the PINT_EDGE_SET.PIQ22 bit to enable edge sensitivity. |
| 21 (R/W1S)         | PIQ21      | Pin Interrupt 21 Edge. Set the PINT_EDGE_SET.PIQ21 bit to enable edge sensitivity. |
| 20 (R/W1S)         | PIQ20      | Pin Interrupt 20 Edge. Set the PINT_EDGE_SET.PIQ20 bit to enable edge sensitivity. |
| 19 (R/W1S)         | PIQ19      | Pin Interrupt 19 Edge. Set the PINT_EDGE_SET.PIQ19 bit to enable edge sensitivity. |
| 18 (R/W1S)         | PIQ18      | Pin Interrupt 18 Edge. Set the PINT_EDGE_SET.PIQ18 bit to enable edge sensitivity. |
| 17 (R/W1S)         | PIQ17      | Pin Interrupt 17 Edge. Set the PINT_EDGE_SET.PIQ17 bit to enable edge sensitivity. |
| 16 (R/W1S)         | PIQ16      | Pin Interrupt 16 Edge. Set the PINT_EDGE_SET.PIQ16 bit to enable edge sensitivity. |

Table 13-32: PINT\_EDGE\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                            |
|--------------------|------------|------------------------------------------------------------------------------------|
| 15 (R/W1S)         | PIQ15      | Pin Interrupt 15 Edge. Set the PINT_EDGE_SET.PIQ15 bit to enable edge sensitivity. |
| 14 (R/W1S)         | PIQ14      | Pin Interrupt 14 Edge. Set the PINT_EDGE_SET.PIQ14 bit to enable edge sensitivity. |
| 13 (R/W1S)         | PIQ13      | Pin Interrupt 13 Edge. Set the PINT_EDGE_SET.PIQ13 bit to enable edge sensitivity. |
| 12 (R/W1S)         | PIQ12      | Pin Interrupt 12 Edge. Set the PINT_EDGE_SET.PIQ12 bit to enable edge sensitivity. |
| 11 (R/W1S)         | PIQ11      | Pin Interrupt 11 Edge. Set the PINT_EDGE_SET.PIQ11 bit to enable edge sensitivity. |
| 10 (R/W1S)         | PIQ10      | Pin Interrupt 10 Edge. Set the PINT_EDGE_SET.PIQ10 bit to enable edge sensitivity. |
| 9 (R/W1S)          | PIQ9       | Pin Interrupt 9 Edge. Set the PINT_EDGE_SET.PIQ9 bit to enable edge sensitivity.   |
| 8 (R/W1S)          | PIQ8       | Pin Interrupt 8 Edge. Set the PINT_EDGE_SET.PIQ8 bit to enable edge sensitivity.   |
| 7 (R/W1S)          | PIQ7       | Pin Interrupt 7 Edge. Set the PINT_EDGE_SET.PIQ7 bit to enable edge sensitivity.   |
| 6 (R/W1S)          | PIQ6       | Pin Interrupt 6 Edge. Set the PINT_EDGE_SET.PIQ6 bit to enable edge sensitivity.   |
| 5 (R/W1S)          | PIQ5       | Pin Interrupt 5 Edge. Set the PINT_EDGE_SET.PIQ5 bit to enable edge sensitivity.   |
| 4 (R/W1S)          | PIQ4       | Pin Interrupt 4 Edge. Set the PINT_EDGE_SET.PIQ4 bit to enable edge sensitivity.   |
| 3 (R/W1S)          | PIQ3       | Pin Interrupt 3 Edge. Set the PINT_EDGE_SET.PIQ3 bit to enable edge sensitivity.   |
| 2 (R/W1S)          | PIQ2       | Pin Interrupt 2 Edge. Set the PINT_EDGE_SET.PIQ2 bit to enable edge sensitivity.   |
| 1 (R/W1S)          | PIQ1       | Pin Interrupt 1 Edge. Set the PINT_EDGE_SET.PIQ1 bit to enable edge sensitivity.   |
| 0 (R/W1S)          | PIQ0       | Pin Interrupt 0 Edge. Set the PINT_EDGE_SET.PIQ0 bit to enable edge sensitivity.   |

## PINT Invert Clear Register

The PINT\_INV\_CLR register disables inverting input polarity. Writing 1 to a bit in PINT\_INV\_CLR disables an inverter for input on the corresponding pin.

Figure 13-29: PINT\_INV\_CLR Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000029_45661e7e7dfb5cf27df599dec14b5b34d3737e01949986a1327b7bf11e6294f5.png)

Table 13-33: PINT\_INV\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------|
| 31 (R/W1C)         | PIQ31      | Pin Interrupt 31 No Invert. Set the PINT_INV_CLR.PIQ31 bit to disable inverted input. |
| 30 (R/W1C)         | PIQ30      | Pin Interrupt 30 No Invert. Set the PINT_INV_CLR.PIQ30 bit to disable inverted input. |
| 29 (R/W1C)         | PIQ29      | Pin Interrupt 29 No Invert. Set the PINT_INV_CLR.PIQ29 bit to disable inverted input. |
| 28 (R/W1C)         | PIQ28      | Pin Interrupt 28 No Invert. Set the PINT_INV_CLR.PIQ28 bit to disable inverted input. |
| 27 (R/W1C)         | PIQ27      | Pin Interrupt 27 No Invert. Set the PINT_INV_CLR.PIQ27 bit to disable inverted input. |
| 26 (R/W1C)         | PIQ26      | Pin Interrupt 26 No Invert. Set the PINT_INV_CLR.PIQ26 bit to disable inverted input. |
| 25 (R/W1C)         | PIQ25      | Pin Interrupt 25 No Invert. Set the PINT_INV_CLR.PIQ25 bit to disable inverted input. |
| 24 (R/W1C)         | PIQ24      | Pin Interrupt 24 No Invert. Set the PINT_INV_CLR.PIQ24 bit to disable inverted input. |
| 23 (R/W1C)         | PIQ23      | Pin Interrupt 23 No Invert. Set the PINT_INV_CLR.PIQ23 bit to disable inverted input. |
| 22 (R/W1C)         | PIQ22      | Pin Interrupt 22 No Invert. Set the PINT_INV_CLR.PIQ22 bit to disable inverted input. |
| 21 (R/W1C)         | PIQ21      | Pin Interrupt 21 No Invert. Set the PINT_INV_CLR.PIQ21 bit to disable inverted input. |
| 20 (R/W1C)         | PIQ20      | Pin Interrupt 20 No Invert. Set the PINT_INV_CLR.PIQ20 bit to disable inverted input. |
| 19 (R/W1C)         | PIQ19      | Pin Interrupt 19 No Invert. Set the PINT_INV_CLR.PIQ19 bit to disable inverted input. |
| 18 (R/W1C)         | PIQ18      | Pin Interrupt 18 No Invert. Set the PINT_INV_CLR.PIQ18 bit to disable inverted input. |
| 17 (R/W1C)         | PIQ17      | Pin Interrupt 17 No Invert. Set the PINT_INV_CLR.PIQ17 bit to disable inverted input. |
| 16 (R/W1C)         | PIQ16      | Pin Interrupt 16 No Invert. Set the PINT_INV_CLR.PIQ16 bit to disable inverted input. |

Table 13-33: PINT\_INV\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------|
| 15 (R/W1C)         | PIQ15      | Pin Interrupt 15 No Invert. Set the PINT_INV_CLR.PIQ15 bit to disable inverted input. |
| 14 (R/W1C)         | PIQ14      | Pin Interrupt 14 No Invert. Set the PINT_INV_CLR.PIQ14 bit to disable inverted input. |
| 13 (R/W1C)         | PIQ13      | Pin Interrupt 13 No Invert. Set the PINT_INV_CLR.PIQ13 bit to disable inverted input. |
| 12 (R/W1C)         | PIQ12      | Pin Interrupt 12 No Invert. Set the PINT_INV_CLR.PIQ12 bit to disable inverted input. |
| 11 (R/W1C)         | PIQ11      | Pin Interrupt 11 No Invert. Set the PINT_INV_CLR.PIQ11 bit to disable inverted input. |
| 10 (R/W1C)         | PIQ10      | Pin Interrupt 10 No Invert. Set the PINT_INV_CLR.PIQ10 bit to disable inverted input. |
| 9 (R/W1C)          | PIQ9       | Pin Interrupt 9 No Invert. Set the PINT_INV_CLR.PIQ9 bit to disable inverted input.   |
| 8 (R/W1C)          | PIQ8       | Pin Interrupt 8 No Invert. Set the PINT_INV_CLR.PIQ8 bit to disable inverted input.   |
| 7 (R/W1C)          | PIQ7       | Pin Interrupt 7 No Invert. Set the PINT_INV_CLR.PIQ7 bit to disable inverted input.   |
| 6 (R/W1C)          | PIQ6       | Pin Interrupt 6 No Invert. Set the PINT_INV_CLR.PIQ6 bit to disable inverted input.   |
| 5 (R/W1C)          | PIQ5       | Pin Interrupt 5 No Invert. Set the PINT_INV_CLR.PIQ5 bit to disable inverted input.   |
| 4 (R/W1C)          | PIQ4       | Pin Interrupt 4 No Invert. Set the PINT_INV_CLR.PIQ4 bit to disable inverted input.   |
| 3 (R/W1C)          | PIQ3       | Pin Interrupt 3 No Invert. Set the PINT_INV_CLR.PIQ3 bit to disable inverted input.   |
| 2 (R/W1C)          | PIQ2       | Pin Interrupt 2 No Invert. Set the PINT_INV_CLR.PIQ2 bit to disable inverted input.   |
| 1 (R/W1C)          | PIQ1       | Pin Interrupt 1 No Invert. Set the PINT_INV_CLR.PIQ1 bit to disable inverted input.   |
| 0 (R/W1C)          | PIQ0       | Pin Interrupt 0 No Invert. Set the PINT_INV_CLR.PIQ0 bit to disable inverted input.   |

## PINT Invert Set Register

The PINT\_INV\_SET register enables inverting input polarity. Writing 1 to a bit in PINT\_INV\_SET enables an inverter for input on the corresponding pin.

Figure 13-30: PINT\_INV\_SET Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000030_af6dbc26c8205302c99f135fabbbf9915210df2d5867cd76a4707f95cd093768.png)

Table 13-34: PINT\_INV\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------|
| 31 (R/W1S)         | PIQ31      | Pin Interrupt 31 Invert. Set the PINT_INV_SET.PIQ31 bit to enable inverted input. |
| 30 (R/W1S)         | PIQ30      | Pin Interrupt 30 Invert. Set the PINT_INV_SET.PIQ30 bit to enable inverted input. |
| 29 (R/W1S)         | PIQ29      | Pin Interrupt 29 Invert. Set the PINT_INV_SET.PIQ29 bit to enable inverted input. |
| 28 (R/W1S)         | PIQ28      | Pin Interrupt 28 Invert. Set the PINT_INV_SET.PIQ28 bit to enable inverted input. |
| 27 (R/W1S)         | PIQ27      | Pin Interrupt 27 Invert. Set the PINT_INV_SET.PIQ27 bit to enable inverted input. |
| 26 (R/W1S)         | PIQ26      | Pin Interrupt 26 Invert. Set the PINT_INV_SET.PIQ26 bit to enable inverted input. |
| 25 (R/W1S)         | PIQ25      | Pin Interrupt 25 Invert. Set the PINT_INV_SET.PIQ25 bit to enable inverted input. |
| 24 (R/W1S)         | PIQ24      | Pin Interrupt 24 Invert. Set the PINT_INV_SET.PIQ24 bit to enable inverted input. |
| 23 (R/W1S)         | PIQ23      | Pin Interrupt 23 Invert. Set the PINT_INV_SET.PIQ23 bit to enable inverted input. |
| 22 (R/W1S)         | PIQ22      | Pin Interrupt 22 Invert. Set the PINT_INV_SET.PIQ22 bit to enable inverted input. |
| 21 (R/W1S)         | PIQ21      | Pin Interrupt 21 Invert. Set the PINT_INV_SET.PIQ21 bit to enable inverted input. |
| 20 (R/W1S)         | PIQ20      | Pin Interrupt 20 Invert. Set the PINT_INV_SET.PIQ20 bit to enable inverted input. |
| 19 (R/W1S)         | PIQ19      | Pin Interrupt 19 Invert. Set the PINT_INV_SET.PIQ19 bit to enable inverted input. |
| 18 (R/W1S)         | PIQ18      | Pin Interrupt 18 Invert. Set the PINT_INV_SET.PIQ18 bit to enable inverted input. |
| 17 (R/W1S)         | PIQ17      | Pin Interrupt 17 Invert. Set the PINT_INV_SET.PIQ17 bit to enable inverted input. |
| 16 (R/W1S)         | PIQ16      | Pin Interrupt 16 Invert. Set the PINT_INV_SET.PIQ16 bit to enable inverted input. |

Table 13-34: PINT\_INV\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------|
| 15 (R/W1S)         | PIQ15      | Pin Interrupt 15 Invert. Set the PINT_INV_SET.PIQ15 bit to enable inverted input. |
| 14 (R/W1S)         | PIQ14      | Pin Interrupt 14 Invert. Set the PINT_INV_SET.PIQ14 bit to enable inverted input. |
| 13 (R/W1S)         | PIQ13      | Pin Interrupt 13 Invert. Set the PINT_INV_SET.PIQ13 bit to enable inverted input. |
| 12 (R/W1S)         | PIQ12      | Pin Interrupt 12 Invert. Set the PINT_INV_SET.PIQ12 bit to enable inverted input. |
| 11 (R/W1S)         | PIQ11      | Pin Interrupt 11 Invert. Set the PINT_INV_SET.PIQ11 bit to enable inverted input. |
| 10 (R/W1S)         | PIQ10      | Pin Interrupt 10 Invert. Set the PINT_INV_SET.PIQ10 bit to enable inverted input. |
| 9 (R/W1S)          | PIQ9       | Pin Interrupt 9 Invert. Set the PINT_INV_SET.PIQ9 bit to enable inverted input.   |
| 8 (R/W1S)          | PIQ8       | Pin Interrupt 8 Invert. Set the PINT_INV_SET.PIQ8 bit to enable inverted input.   |
| 7 (R/W1S)          | PIQ7       | Pin Interrupt 7 Invert. Set the PINT_INV_SET.PIQ7 bit to enable inverted input.   |
| 6 (R/W1S)          | PIQ6       | Pin Interrupt 6 Invert. Set the PINT_INV_SET.PIQ6 bit to enable inverted input.   |
| 5 (R/W1S)          | PIQ5       | Pin Interrupt 5 Invert. Set the PINT_INV_SET.PIQ5 bit to enable inverted input.   |
| 4 (R/W1S)          | PIQ4       | Pin Interrupt 4 Invert. Set the PINT_INV_SET.PIQ4 bit to enable inverted input.   |
| 3 (R/W1S)          | PIQ3       | Pin Interrupt 3 Invert. Set the PINT_INV_SET.PIQ3 bit to enable inverted input.   |
| 2 (R/W1S)          | PIQ2       | Pin Interrupt 2 Invert. Set the PINT_INV_SET.PIQ2 bit to enable inverted input.   |
| 1 (R/W1S)          | PIQ1       | Pin Interrupt 1 Invert. Set the PINT_INV_SET.PIQ1 bit to enable inverted input.   |
| 0 (R/W1S)          | PIQ0       | Pin Interrupt 0 Invert. Set the PINT_INV_SET.PIQ0 bit to enable inverted input.   |

## PINT Latch Register

The PINT\_LATCH register indicates the interrupt latch status for pin interrupts. When set, an interrupt request is latched. When cleared, there is no interrupt request latched.

Both the PINT\_REQ and PINT\_LATCH registers indicate whether an interrupt request is latched on the respective pin. The PINT\_LATCH register is a latch that operates regardless of the interrupt masks. Bits of the PINT\_REQ register depend on the mask register. The PINT\_REQ register is a logical AND of the PINT\_LATCH register and the interrupt mask.

Having two separate registers here enables the user to interrogate certain pins in polling mode while others work in interrupt mode. The PINT\_LATCH registers can be used for edge detection or pin activity detection.

Both registers have W1C behavior. Writing a 1 to either register clears the respective bits in both registers. For interrupt operation, the user may prefer to W1C the PINT\_REQ register (address still loaded in Px pointer). In polling mode, it might be cleaner to W1C the PINT\_LATCH register.

Whether in edge-sensitive mode or level-sensitive mode, PINT\_LATCH bits are never cleared by hardware except at system reset. Even in level-sensitive mode, the PINT\_LATCH register functions as latch.

Figure 13-31: PINT\_LATCH Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000031_07a2972015b1fe9331ac9f657345243880ff24f0552a1cd6ec4ec4fa46d9b075.png)

Table 13-35: PINT\_LATCH Register Fields

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000032_ffe76703010e36f0f486c34cd50d116c92c27f7b26a52e7e649e2b29444f3a8c.png)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                     |
|--------------------|------------|-------------------------------------------------------------|
| 31                 | PIQ31      | Pin Interrupt 31 Latch.                                     |
| (R/W1C)            |            | If the PINT_LATCH.PIQ31 bit is set, the request is latched. |

Table 13-35: PINT\_LATCH Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------|
| 30 (R/W1C)         | PIQ30      | Pin Interrupt 30 Latch. If the PINT_LATCH.PIQ30 bit is set, the request is latched. |
| 29 (R/W1C)         | PIQ29      | Pin Interrupt 29 Latch. If the PINT_LATCH.PIQ29 bit is set, the request is latched. |
| 28 (R/W1C)         | PIQ28      | Pin Interrupt 28 Latch. If the PINT_LATCH.PIQ28 bit is set, the request is latched. |
| 27 (R/W1C)         | PIQ27      | Pin Interrupt 27 Latch. If the PINT_LATCH.PIQ27 bit is set, the request is latched. |
| 26 (R/W1C)         | PIQ26      | Pin Interrupt 26 Latch. If the PINT_LATCH.PIQ26 bit is set, the request is latched. |
| 25 (R/W1C)         | PIQ25      | Pin Interrupt 25 Latch. If the PINT_LATCH.PIQ25 bit is set, the request is latched. |
| 24 (R/W1C)         | PIQ24      | Pin Interrupt 24 Latch. If the PINT_LATCH.PIQ24 bit is set, the request is latched. |
| 23 (R/W1C)         | PIQ23      | Pin Interrupt 23 Latch. If the PINT_LATCH.PIQ23 bit is set, the request is latched. |
| 22 (R/W1C)         | PIQ22      | Pin Interrupt 22 Latch. If the PINT_LATCH.PIQ22 bit is set, the request is latched. |
| 21 (R/W1C)         | PIQ21      | Pin Interrupt 21 Latch. If the PINT_LATCH.PIQ21 bit is set, the request is latched. |
| 20 (R/W1C)         | PIQ20      | Pin Interrupt 20 Latch. If the PINT_LATCH.PIQ20 bit is set, the request is latched. |
| 19 (R/W1C)         | PIQ19      | Pin Interrupt 19 Latch. If the PINT_LATCH.PIQ19 bit is set, the request is latched. |
| 18 (R/W1C)         | PIQ18      | Pin Interrupt 18 Latch. If the PINT_LATCH.PIQ18 bit is set, the request is latched. |
| 17 (R/W1C)         | PIQ17      | Pin Interrupt 17 Latch. If the PINT_LATCH.PIQ17 bit is set, the request is latched. |
| 16 (R/W1C)         | PIQ16      | Pin Interrupt 16 Latch. If the PINT_LATCH.PIQ16 bit is set, the request is latched. |
| 15 (R/W1C)         | PIQ15      | Pin Interrupt 15 Latch. If the PINT_LATCH.PIQ15 bit is set, the request is latched. |

Table 13-35: PINT\_LATCH Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------|
| 14 (R/W1C)         | PIQ14      | Pin Interrupt 14 Latch. If the PINT_LATCH.PIQ14 bit is set, the request is latched. |
| 13 (R/W1C)         | PIQ13      | Pin Interrupt 13 Latch. If the PINT_LATCH.PIQ13 bit is set, the request is latched. |
| 12 (R/W1C)         | PIQ12      | Pin Interrupt 12 Latch. If the PINT_LATCH.PIQ12 bit is set, the request is latched. |
| 11 (R/W1C)         | PIQ11      | Pin Interrupt 11 Latch. If the PINT_LATCH.PIQ11 bit is set, the request is latched. |
| 10 (R/W1C)         | PIQ10      | Pin Interrupt 10 Latch. If the PINT_LATCH.PIQ10 bit is set, the request is latched. |
| 9 (R/W1C)          | PIQ9       | Pin Interrupt 9 Latch. If the PINT_LATCH.PIQ9 bit is set, the request is latched.   |
| 8 (R/W1C)          | PIQ8       | Pin Interrupt 8 Latch. If the PINT_LATCH.PIQ8 bit is set, the request is latched.   |
| 7 (R/W1C)          | PIQ7       | Pin Interrupt 7 Latch. If the PINT_LATCH.PIQ7 bit is set, the request is latched.   |
| 6 (R/W1C)          | PIQ6       | Pin Interrupt 6 Latch. If the PINT_LATCH.PIQ6 bit is set, the request is latched.   |
| 5 (R/W1C)          | PIQ5       | Pin Interrupt 5 Latch. If the PINT_LATCH.PIQ5 bit is set, the request is latched.   |
| 4 (R/W1C)          | PIQ4       | Pin Interrupt 4 Latch. If the PINT_LATCH.PIQ4 bit is set, the request is latched.   |
| 3 (R/W1C)          | PIQ3       | Pin Interrupt 3 Latch. If the PINT_LATCH.PIQ3 bit is set, the request is latched.   |
| 2 (R/W1C)          | PIQ2       | Pin Interrupt 2 Latch. If the PINT_LATCH.PIQ2 bit is set, the request is latched.   |
| 1 (R/W1C)          | PIQ1       | Pin Interrupt 1 Latch. If the PINT_LATCH.PIQ1 bit is set, the request is latched.   |
| 0 (R/W1C)          | PIQ0       | Pin Interrupt 0 Latch. If the PINT_LATCH.PIQ0 bit is set, the request is latched.   |

## PINT Mask Clear Register

The PINT\_MSK\_CLR register permits masking (disabling) of interrupt requests. Writing 1 to a bit in PINT\_MSK\_CLR masks the corresponding pin interrupt.

Figure 13-32: PINT\_MSK\_CLR Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000033_d73be53187ece8d9a1e635267267753820b42b438c8413c690876ef3de310c1f.png)

Table 13-36: PINT\_MSK\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                         |
|--------------------|------------|---------------------------------------------------------------------------------|
| 31 (R/W1C)         | PIQ31      | Pin Interrupt 31 Mask. Set the PINT_MSK_CLR.PIQ31 bit to disable the interrupt. |
| 30 (R/W1C)         | PIQ30      | Pin Interrupt 30 Mask. Set the PINT_MSK_CLR.PIQ30 bit to disable the interrupt. |
| 29 (R/W1C)         | PIQ29      | Pin Interrupt 29 Mask. Set the PINT_MSK_CLR.PIQ29 bit to disable the interrupt. |
| 28 (R/W1C)         | PIQ28      | Pin Interrupt 28 Mask. Set the PINT_MSK_CLR.PIQ28 bit to disable the interrupt. |
| 27 (R/W1C)         | PIQ27      | Pin Interrupt 27 Mask. Set the PINT_MSK_CLR.PIQ27 bit to disable the interrupt. |
| 26 (R/W1C)         | PIQ26      | Pin Interrupt 26 Mask. Set the PINT_MSK_CLR.PIQ26 bit to disable the interrupt. |
| 25 (R/W1C)         | PIQ25      | Pin Interrupt 25 Mask. Set the PINT_MSK_CLR.PIQ25 bit to disable the interrupt. |
| 24 (R/W1C)         | PIQ24      | Pin Interrupt 24 Mask. Set the PINT_MSK_CLR.PIQ24 bit to disable the interrupt. |
| 23 (R/W1C)         | PIQ23      | Pin Interrupt 23 Mask. Set the PINT_MSK_CLR.PIQ23 bit to disable the interrupt. |
| 22 (R/W1C)         | PIQ22      | Pin Interrupt 22 Mask. Set the PINT_MSK_CLR.PIQ22 bit to disable the interrupt. |
| 21 (R/W1C)         | PIQ21      | Pin Interrupt 21 Mask. Set the PINT_MSK_CLR.PIQ21 bit to disable the interrupt. |
| 20 (R/W1C)         | PIQ20      | Pin Interrupt 20 Mask. Set the PINT_MSK_CLR.PIQ20 bit to disable the interrupt. |
| 19 (R/W1C)         | PIQ19      | Pin Interrupt 19 Mask. Set the PINT_MSK_CLR.PIQ19 bit to disable the interrupt. |
| 18 (R/W1C)         | PIQ18      | Pin Interrupt 18 Mask. Set the PINT_MSK_CLR.PIQ18 bit to disable the interrupt. |
| 17 (R/W1C)         | PIQ17      | Pin Interrupt 17 Mask. Set the PINT_MSK_CLR.PIQ17 bit to disable the interrupt. |
| 16 (R/W1C)         | PIQ16      | Pin Interrupt 16 Mask. Set the PINT_MSK_CLR.PIQ16 bit to disable the interrupt. |

Table 13-36: PINT\_MSK\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                         |
|--------------------|------------|---------------------------------------------------------------------------------|
| 15 (R/W1C)         | PIQ15      | Pin Interrupt 15 Mask. Set the PINT_MSK_CLR.PIQ15 bit to disable the interrupt. |
| 14 (R/W1C)         | PIQ14      | Pin Interrupt 14 Mask. Set the PINT_MSK_CLR.PIQ14 bit to disable the interrupt. |
| 13 (R/W1C)         | PIQ13      | Pin Interrupt 13 Mask. Set the PINT_MSK_CLR.PIQ13 bit to disable the interrupt. |
| 12 (R/W1C)         | PIQ12      | Pin Interrupt 12 Mask. Set the PINT_MSK_CLR.PIQ12 bit to disable the interrupt. |
| 11 (R/W1C)         | PIQ11      | Pin Interrupt 11 Mask. Set the PINT_MSK_CLR.PIQ11 bit to disable the interrupt. |
| 10 (R/W1C)         | PIQ10      | Pin Interrupt 10 Mask. Set the PINT_MSK_CLR.PIQ10 bit to disable the interrupt. |
| 9 (R/W1C)          | PIQ9       | Pin Interrupt 9 Mask. Set the PINT_MSK_CLR.PIQ9 bit to disable the interrupt.   |
| 8 (R/W1C)          | PIQ8       | Pin Interrupt 8 Mask. Set the PINT_MSK_CLR.PIQ8 bit to disable the interrupt.   |
| 7 (R/W1C)          | PIQ7       | Pin Interrupt 7 Mask. Set the PINT_MSK_CLR.PIQ7 bit to disable the interrupt.   |
| 6 (R/W1C)          | PIQ6       | Pin Interrupt 6 Mask. Set the PINT_MSK_CLR.PIQ6 bit to disable the interrupt.   |
| 5 (R/W1C)          | PIQ5       | Pin Interrupt 5 Mask. Set the PINT_MSK_CLR.PIQ5 bit to disable the interrupt.   |
| 4 (R/W1C)          | PIQ4       | Pin Interrupt 4 Mask. Set the PINT_MSK_CLR.PIQ4 bit to disable the interrupt.   |
| 3 (R/W1C)          | PIQ3       | Pin Interrupt 3 Mask. Set the PINT_MSK_CLR.PIQ3 bit to disable the interrupt.   |
| 2 (R/W1C)          | PIQ2       | Pin Interrupt 2 Mask. Set the PINT_MSK_CLR.PIQ2 bit to disable the interrupt.   |
| 1 (R/W1C)          | PIQ1       | Pin Interrupt 1 Mask. Set the PINT_MSK_CLR.PIQ1 bit to disable the interrupt.   |
| 0 (R/W1C)          | PIQ0       | Pin Interrupt 0 Mask. Set the PINT_MSK_CLR.PIQ0 bit to disable the interrupt.   |

## PINT Mask Set Register

The PINT\_MSK\_SET register permits unmasking (enabling) of interrupt requests. Writing 1 to a bit in PINT\_MSK\_SET unmasks the corresponding pin interrupt.

Figure 13-33: PINT\_MSK\_SET Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000034_74ded2cb2ecae255c15aad197ca0225956dd4010270146240e604192097065db.png)

Table 13-37: PINT\_MSK\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                          |
|--------------------|------------|----------------------------------------------------------------------------------|
| 31 (R/W1S)         | PIQ31      | Pin Interrupt 31 Unmask. Set the PINT_MSK_SET.PIQ31 bit to enable the interrupt. |
| 30 (R/W1S)         | PIQ30      | Pin Interrupt 30 Unmask. Set the PINT_MSK_SET.PIQ30 bit to enable the interrupt. |
| 29 (R/W1S)         | PIQ29      | Pin Interrupt 29 Unmask. Set the PINT_MSK_SET.PIQ29 bit to enable the interrupt. |
| 28 (R/W1S)         | PIQ28      | Pin Interrupt 28 Unmask. Set the PINT_MSK_SET.PIQ28 bit to enable the interrupt. |
| 27 (R/W1S)         | PIQ27      | Pin Interrupt 27 Unmask. Set the PINT_MSK_SET.PIQ27 bit to enable the interrupt. |
| 26 (R/W1S)         | PIQ26      | Pin Interrupt 26 Unmask. Set the PINT_MSK_SET.PIQ26 bit to enable the interrupt. |
| 25 (R/W1S)         | PIQ25      | Pin Interrupt 25 Unmask. Set the PINT_MSK_SET.PIQ25 bit to enable the interrupt. |
| 24 (R/W1S)         | PIQ24      | Pin Interrupt 24 Unmask. Set the PINT_MSK_SET.PIQ24 bit to enable the interrupt. |
| 23 (R/W1S)         | PIQ23      | Pin Interrupt 23 Unmask. Set the PINT_MSK_SET.PIQ23 bit to enable the interrupt. |
| 22 (R/W1S)         | PIQ22      | Pin Interrupt 22 Unmask. Set the PINT_MSK_SET.PIQ22 bit to enable the interrupt. |
| 21 (R/W1S)         | PIQ21      | Pin Interrupt 21 Unmask. Set the PINT_MSK_SET.PIQ21 bit to enable the interrupt. |
| 20 (R/W1S)         | PIQ20      | Pin Interrupt 20 Unmask. Set the PINT_MSK_SET.PIQ20 bit to enable the interrupt. |
| 19 (R/W1S)         | PIQ19      | Pin Interrupt 19 Unmask. Set the PINT_MSK_SET.PIQ19 bit to enable the interrupt. |
| 18 (R/W1S)         | PIQ18      | Pin Interrupt 18 Unmask. Set the PINT_MSK_SET.PIQ18 bit to enable the interrupt. |
| 17 (R/W1S)         | PIQ17      | Pin Interrupt 17 Unmask. Set the PINT_MSK_SET.PIQ17 bit to enable the interrupt. |
| 16 (R/W1S)         | PIQ16      | Pin Interrupt 16 Unmask. Set the PINT_MSK_SET.PIQ16 bit to enable the interrupt. |

Table 13-37: PINT\_MSK\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                          |
|--------------------|------------|----------------------------------------------------------------------------------|
| 15 (R/W1S)         | PIQ15      | Pin Interrupt 15 Unmask. Set the PINT_MSK_SET.PIQ15 bit to enable the interrupt. |
| 14 (R/W1S)         | PIQ14      | Pin Interrupt 14 Unmask. Set the PINT_MSK_SET.PIQ14 bit to enable the interrupt. |
| 13 (R/W1S)         | PIQ13      | Pin Interrupt 13 Unmask. Set the PINT_MSK_SET.PIQ13 bit to enable the interrupt. |
| 12 (R/W1S)         | PIQ12      | Pin Interrupt 12 Unmask. Set the PINT_MSK_SET.PIQ12 bit to enable the interrupt. |
| 11 (R/W1S)         | PIQ11      | Pin Interrupt 11 Unmask. Set the PINT_MSK_SET.PIQ11 bit to enable the interrupt. |
| 10 (R/W1S)         | PIQ10      | Pin Interrupt 10 Unmask. Set the PINT_MSK_SET.PIQ10 bit to enable the interrupt. |
| 9 (R/W1S)          | PIQ9       | Pin Interrupt 9 Unmask. Set the PINT_MSK_SET.PIQ9 bit to enable the interrupt.   |
| 8 (R/W1S)          | PIQ8       | Pin Interrupt 8 Unmask. Set the PINT_MSK_SET.PIQ8 bit to enable the interrupt.   |
| 7 (R/W1S)          | PIQ7       | Pin Interrupt 7 Unmask. Set the PINT_MSK_SET.PIQ7 bit to enable the interrupt.   |
| 6 (R/W1S)          | PIQ6       | Pin Interrupt 6 Unmask. Set the PINT_MSK_SET.PIQ6 bit to enable the interrupt.   |
| 5 (R/W1S)          | PIQ5       | Pin Interrupt 5 Unmask. Set the PINT_MSK_SET.PIQ5 bit to enable the interrupt.   |
| 4 (R/W1S)          | PIQ4       | Pin Interrupt 4 Unmask. Set the PINT_MSK_SET.PIQ4 bit to enable the interrupt.   |
| 3 (R/W1S)          | PIQ3       | Pin Interrupt 3 Unmask. Set the PINT_MSK_SET.PIQ3 bit to enable the interrupt.   |
| 2 (R/W1S)          | PIQ2       | Pin Interrupt 2 Unmask. Set the PINT_MSK_SET.PIQ2 bit to enable the interrupt.   |
| 1 (R/W1S)          | PIQ1       | Pin Interrupt 1 Unmask. Set the PINT_MSK_SET.PIQ1 bit to enable the interrupt.   |
| 0 (R/W1S)          | PIQ0       | Pin Interrupt 0 Unmask. Set the PINT_MSK_SET.PIQ0 bit to enable the interrupt.   |

## PINT Pin State Register

When a half port is assigned to a byte in any PINT block, the state of the eight pins (regardless of GPIO or function, input or output) can be seen in the PINT\_PINSTATE register. While neither input nor output drivers of the pin are enabled, reads of the pin state in PINT\_PINSTATE return zero. The PINT\_PINSTATE register reports the inverted state of the pin if the signal inverter is activated by the PINT\_INV\_SET register. The inverter can be enabled on an individual bit-by-bit basis. Every bit in the PINT\_INV\_SET and PINT\_INV\_CLR register pair represents a pin signal.

The pin interrupt pin state registers enable the service routine to read the current state of the pin without reading from GPIO space. If there was an edge-sensitive interrupt, the service routine can check whether the state of the pin is still high or turned low.

Figure 13-34: PINT\_PINSTATE Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000035_a3c7a854bdb2737e6f40429d459c42d464d1f43de96e298e22ccdda909fd9567.png)

Table 13-38: PINT\_PINSTATE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                      |
|--------------------|------------|--------------------------------------------------------------|
| 31                 | PIQ31      | Pin Interrupt 31 State.                                      |
| (R/NW)             |            | A read of the PINT_PINSTATE.PIQ31 bit returns the pin state. |

Table 13-38: PINT\_PINSTATE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------|
| 30 (R/NW)          | PIQ30      | Pin Interrupt 30 State. A read of the PINT_PINSTATE.PIQ30 bit returns the pin state. |
| 29 (R/NW)          | PIQ29      | Pin Interrupt 29 State. A read of the PINT_PINSTATE.PIQ29 bit returns the pin state. |
| 28 (R/NW)          | PIQ28      | Pin Interrupt 28 State. A read of the PINT_PINSTATE.PIQ28 bit returns the pin state. |
| 27 (R/NW)          | PIQ27      | Pin Interrupt 27 State. A read of the PINT_PINSTATE.PIQ27 bit returns the pin state. |
| 26 (R/NW)          | PIQ26      | Pin Interrupt 26 State. A read of the PINT_PINSTATE.PIQ26 bit returns the pin state. |
| 25 (R/NW)          | PIQ25      | Pin Interrupt 25 State. A read of the PINT_PINSTATE.PIQ25 bit returns the pin state. |
| 24 (R/NW)          | PIQ24      | Pin Interrupt 24 State. A read of the PINT_PINSTATE.PIQ24 bit returns the pin state. |
| 23 (R/NW)          | PIQ23      | Pin Interrupt 23 State. A read of the PINT_PINSTATE.PIQ23 bit returns the pin state. |
| 22 (R/NW)          | PIQ22      | Pin Interrupt 22 State. A read of the PINT_PINSTATE.PIQ22 bit returns the pin state. |
| 21 (R/NW)          | PIQ21      | Pin Interrupt 21 State. A read of the PINT_PINSTATE.PIQ21 bit returns the pin state. |
| 20 (R/NW)          | PIQ20      | Pin Interrupt 20 State. A read of the PINT_PINSTATE.PIQ20 bit returns the pin state. |
| 19 (R/NW)          | PIQ19      | Pin Interrupt 19 State. A read of the PINT_PINSTATE.PIQ19 bit returns the pin state. |
| 18 (R/NW)          | PIQ18      | Pin Interrupt 18 State. A read of the PINT_PINSTATE.PIQ18 bit returns the pin state. |
| 17 (R/NW)          | PIQ17      | Pin Interrupt 17 State. A read of the PINT_PINSTATE.PIQ17 bit returns the pin state. |
| 16 (R/NW)          | PIQ16      | Pin Interrupt 16 State. A read of the PINT_PINSTATE.PIQ16 bit returns the pin state. |
| 15 (R/NW)          | PIQ15      | Pin Interrupt 15 State. A read of the PINT_PINSTATE.PIQ15 bit returns the pin state. |

Table 13-38: PINT\_PINSTATE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------|
| 14 (R/NW)          | PIQ14      | Pin Interrupt 14 State. A read of the PINT_PINSTATE.PIQ14 bit returns the pin state. |
| 13 (R/NW)          | PIQ13      | Pin Interrupt 13 State. A read of the PINT_PINSTATE.PIQ13 bit returns the pin state. |
| 12 (R/NW)          | PIQ12      | Pin Interrupt 12 State. A read of the PINT_PINSTATE.PIQ12 bit returns the pin state. |
| 11 (R/NW)          | PIQ11      | Pin Interrupt 11 State. A read of the PINT_PINSTATE.PIQ11 bit returns the pin state. |
| 10 (R/NW)          | PIQ10      | Pin Interrupt 10 State. A read of the PINT_PINSTATE.PIQ10 bit returns the pin state. |
| 9 (R/NW)           | PIQ9       | Pin Interrupt 9 State. A read of the PINT_PINSTATE.PIQ9 bit returns the pin state.   |
| 8 (R/NW)           | PIQ8       | Pin Interrupt 8 State. A read of the PINT_PINSTATE.PIQ8 bit returns the pin state.   |
| 7 (R/NW)           | PIQ7       | Pin Interrupt 7 State. A read of the PINT_PINSTATE.PIQ7 bit returns the pin state.   |
| 6 (R/NW)           | PIQ6       | Pin Interrupt 6 State. A read of the PINT_PINSTATE.PIQ6 bit returns the pin state.   |
| 5 (R/NW)           | PIQ5       | Pin Interrupt 5 State. A read of the PINT_PINSTATE.PIQ5 bit returns the pin state.   |
| 4 (R/NW)           | PIQ4       | Pin Interrupt 4 State. A read of the PINT_PINSTATE.PIQ4 bit returns the pin state.   |
| 3 (R/NW)           | PIQ3       | Pin Interrupt 3 State. A read of the PINT_PINSTATE.PIQ3 bit returns the pin state.   |
| 2 (R/NW)           | PIQ2       | Pin Interrupt 2 State. A read of the PINT_PINSTATE.PIQ2 bit returns the pin state.   |
| 1 (R/NW)           | PIQ1       | Pin Interrupt 1 State. A read of the PINT_PINSTATE.PIQ1 bit returns the pin state.   |
| 0 (R/NW)           | PIQ0       | Pin Interrupt 0 State. A read of the PINT_PINSTATE.PIQ0 bit returns the pin state.   |

## PINT Request Register

The PINT\_REQ register indicates the interrupt request status for pin interrupts. When set, an interrupt request is pending. When cleared, there is no interrupt request pending.

Both the PINT\_REQ and PINT\_LATCH registers indicate whether an interrupt request is latched on the respective pin. The PINT\_LATCH register is a latch that operates regardless of the interrupt masks. Bits of the PINT\_REQ register depend on the mask register. The PINT\_REQ register is a logical AND of the PINT\_LATCH register and the interrupt mask.

Figure 13-35: PINT\_REQ Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000036_00180af2c2353d89cdfca7dc1bdc351690648c03742c520a6abc845598791b61.png)

Table 13-39: PINT\_REQ Register Fields

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000037_8a9f7eda99cb249b32dd3fee87c6b908dc30d821b925df711ce5671dda1af3c9.png)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                 |
|--------------------|------------|---------------------------------------------------------|
| 31                 | PIQ31      | Pin Interrupt 31 Request.                               |
| (R/W1C)            |            | If the PINT_REQ.PIQ31 bit is set, a request is pending. |

Table 13-39: PINT\_REQ Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------|
| 30 (R/W1C)         | PIQ30      | Pin Interrupt 30 Request. If the PINT_REQ.PIQ30 bit is set, a request is pending. |
| 29 (R/W1C)         | PIQ29      | Pin Interrupt 29 Request. If the PINT_REQ.PIQ29 bit is set, a request is pending. |
| 28 (R/W1C)         | PIQ28      | Pin Interrupt 28 Request. If the PINT_REQ.PIQ28 bit is set, a request is pending. |
| 27 (R/W1C)         | PIQ27      | Pin Interrupt 27 Request. If the PINT_REQ.PIQ27 bit is set, a request is pending. |
| 26 (R/W1C)         | PIQ26      | Pin Interrupt 26 Request. If the PINT_REQ.PIQ26 bit is set, a request is pending. |
| 25 (R/W1C)         | PIQ25      | Pin Interrupt 25 Request. If the PINT_REQ.PIQ25 bit is set, a request is pending. |
| 24 (R/W1C)         | PIQ24      | Pin Interrupt 24 Request. If the PINT_REQ.PIQ24 bit is set, a request is pending. |
| 23 (R/W1C)         | PIQ23      | Pin Interrupt 23 Request. If the PINT_REQ.PIQ23 bit is set, a request is pending. |
| 22 (R/W1C)         | PIQ22      | Pin Interrupt 22 Request. If the PINT_REQ.PIQ22 bit is set, a request is pending. |
| 21 (R/W1C)         | PIQ21      | Pin Interrupt 21 Request. If the PINT_REQ.PIQ21 bit is set, a request is pending. |
| 20 (R/W1C)         | PIQ20      | Pin Interrupt 20 Request. If the PINT_REQ.PIQ20 bit is set, a request is pending. |
| 19 (R/W1C)         | PIQ19      | Pin Interrupt 19 Request. If the PINT_REQ.PIQ19 bit is set, a request is pending. |
| 18 (R/W1C)         | PIQ18      | Pin Interrupt 18 Request. If the PINT_REQ.PIQ18 bit is set, a request is pending. |
| 17 (R/W1C)         | PIQ17      | Pin Interrupt 17 Request. If the PINT_REQ.PIQ17 bit is set, a request is pending. |
| 16 (R/W1C)         | PIQ16      | Pin Interrupt 16 Request. If the PINT_REQ.PIQ16 bit is set, a request is pending. |
| 15 (R/W1C)         | PIQ15      | Pin Interrupt 15 Request. If the PINT_REQ.PIQ15 bit is set, a request is pending. |

Table 13-39: PINT\_REQ Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------|
| 14 (R/W1C)         | PIQ14      | Pin Interrupt 14 Request. If the PINT_REQ.PIQ14 bit is set, a request is pending. |
| 13 (R/W1C)         | PIQ13      | Pin Interrupt 13 Request. If the PINT_REQ.PIQ13 bit is set, a request is pending. |
| 12 (R/W1C)         | PIQ12      | Pin Interrupt 12 Request. If the PINT_REQ.PIQ12 bit is set, a request is pending. |
| 11 (R/W1C)         | PIQ11      | Pin Interrupt 11 Request. If the PINT_REQ.PIQ11 bit is set, a request is pending. |
| 10 (R/W1C)         | PIQ10      | Pin Interrupt 10 Request. If the PINT_REQ.PIQ10 bit is set, a request is pending. |
| 9 (R/W1C)          | PIQ9       | Pin Interrupt 9 Request. If the PINT_REQ.PIQ9 bit is set, a request is pending.   |
| 8 (R/W1C)          | PIQ8       | Pin Interrupt 8 Request. If the PINT_REQ.PIQ8 bit is set, a request is pending.   |
| 7 (R/W1C)          | PIQ7       | Pin Interrupt 7 Request. If the PINT_REQ.PIQ7 bit is set, a request is pending.   |
| 6 (R/W1C)          | PIQ6       | Pin Interrupt 6 Request. If the PINT_REQ.PIQ6 bit is set, a request is pending.   |
| 5 (R/W1C)          | PIQ5       | Pin Interrupt 5 Request. If the PINT_REQ.PIQ5 bit is set, a request is pending.   |
| 4 (R/W1C)          | PIQ4       | Pin Interrupt 4 Request. If the PINT_REQ.PIQ4 bit is set, a request is pending.   |
| 3 (R/W1C)          | PIQ3       | Pin Interrupt 3 Request. If the PINT_REQ.PIQ3 bit is set, a request is pending.   |
| 2 (R/W1C)          | PIQ2       | Pin Interrupt 2 Request. If the PINT_REQ.PIQ2 bit is set, a request is pending.   |
| 1 (R/W1C)          | PIQ1       | Pin Interrupt 1 Request. If the PINT_REQ.PIQ1 bit is set, a request is pending.   |
| 0 (R/W1C)          | PIQ0       | Pin Interrupt 0 Request. If the PINT_REQ.PIQ0 bit is set, a request is pending.   |

## ADSP-2184x PADS Register Descriptions

Pads Controller (PADS) contains the following registers.

Table 13-40: ADSP-2184x PADS Register List

| Name                | Description                                        |
|---------------------|----------------------------------------------------|
| PADS_DAI0_0_DS      | DAI0 Pins 1 to 8 Drive Strength Control Register   |
| PADS_DAI0_1_DS      | DAI0 Pins 9 to 16 Drive Strength Control Register  |
| PADS_DAI0_2_DS      | DAI0 Pins 17 to 20 Drive Strength Control Register |
| PADS_DAI0_IE        | DAI0 Port Input Enable Control Register            |
| PADS_DAI0_PE        | DAI0 Pull Enable Register                          |
| PADS_DAI0_PS        | DAI0 Pull Selector Register                        |
| PADS_DAI0_SL        | DAI0 Slew Rate Control Register                    |
| PADS_DAI0_SPU       | DAI0 Strong Pull Up Control Register               |
| PADS_DAI1_0_DS      | DAI1 Pins 1 to 8 Drive Strength Control Register   |
| PADS_DAI1_1_DS      | DAI1 Pins 9 to 16 Drive Strength Control Register  |
| PADS_DAI1_2_DS      | DAI1 Pins 17 to 20 Drive Strength Control Register |
| PADS_DAI1_IE        | DAI1 Port Input Enable Control Register            |
| PADS_DAI1_PE        | DAI1 Pull Enable Register                          |
| PADS_DAI1_PS        | DAI1 Pull Selector Register                        |
| PADS_DAI1_SL        | DAI1 Slew Rate Control Register                    |
| PADS_DAI1_SPU       | DAI1 Strong Pull up Control Register               |
| PADS_NONPORTS_CTL   | Non-GPIO Control Register                          |
| PADS_NONPORTS_DS    | Non-GPIO Drive Strength Register                   |
| PADS_NONPORTS_DS_1  | EMAC Non-GPIO Drive Strength Register              |
| PADS_OTPC_BOOT_STAT | OTPC Boot Status Register                          |
| PADS_PCFG0          | Peripheral PAD Configuration 0 Register            |
| PADS_PCFG1          | Peripheral Configuration 1 Register                |
| PADS_PORTA0_DS      | PORTA Pins 0 to 7 Drive Strength Control Register  |
| PADS_PORTA1_DS      | PORTA Pins 8 to 15 Drive Strength Control Register |
| PADS_PORTB0_DS      | PORTB Pins 0 to 7 Drive Strength Control Register  |
| PADS_PORTB1_DS      | PORTB Pins 8 to 15 Drive Strength Control Register |
| PADS_PORTC0_DS      | PORTC Pins 0 to 7 Drive Strength Control Register  |
| PADS_PORTC1_DS      | PORTC Pins 8 to 15 Drive Strength Control Register |
| PADS_PORTD0_DS      | PORTD Pins 0 to 7 Drive Strength Control Register  |

Table 13-40: ADSP-2184x PADS Register List (Continued)

| Name             | Description                                        |
|------------------|----------------------------------------------------|
| PADS_PORTD1_DS   | PORTD Pins 8 to 15 Drive Strength Control Register |
| PADS_PORTE0_DS   | PORTE Pins 0 to 7 Drive Strength Control Register  |
| PADS_PORTE1_DS   | PORTE Pins 8 to 15 Drive Strength Control Register |
| PADS_PORTF0_DS   | PORTF Pins 0 to 7 Drive Strength Control Register  |
| PADS_PORTF1_DS   | PORTF Pins 8 to 15 Drive Strength Control Register |
| PADS_PORTG0_DS   | PORTG Pins 0 to 7 Drive Strength Control Register  |
| PADS_PORTG1_DS   | PORTG Pins 8 to 15 Drive Strength Control Register |
| PADS_PORTH0_DS   | PORTH Pins 0 to 7 Drive Strength Control Register  |
| PADS_PORTH1_DS   | PORTH Pins 8 and 9 Drive Strength Control Register |
| PADS_PORT_AB_PE  | PORTA and PORTB Pull Enable Register               |
| PADS_PORT_AB_PS  | PORTA and PORTB Pull Selector Register             |
| PADS_PORT_AB_SL  | PORTA and PORTB Slew Rate Control Register         |
| PADS_PORT_AB_SPU | PORTA and PORTB Strong Pull Up Control Register    |
| PADS_PORT_CD_PE  | PORTC and PORTD Pull Enable Register               |
| PADS_PORT_CD_PS  | PORTC and PORTD Pull Selector Register             |
| PADS_PORT_CD_SL  | PORTC and PORTD Slew Rate Control Register         |
| PADS_PORT_CD_SPU | PORTC and PORTD Strong Pull Up Control Register    |
| PADS_PORT_EF_PE  | PORTE and PORTF Pull Enable Register               |
| PADS_PORT_EF_PS  | PORTE and PORTF Pull Selector Register             |
| PADS_PORT_EF_SL  | PORTE and PORTF Slew Rate Control Register         |
| PADS_PORT_EF_SPU | PORTE and PORTF Strong Pull Up Control Register    |
| PADS_PORT_GH_PE  | PORTG and PORTH Pull Enable Register               |
| PADS_PORT_GH_PS  | PORTG and PORTH Pull Selector Register             |
| PADS_PORT_GH_SL  | PORTG and PORTH Slew Rate Control Register         |
| PADS_PORT_GH_SPU | PORTG and PORTH Strong Pull Up Control Register    |
| PADS_STAT        | Status Register                                    |

## DAI0 Pins 1 to 8 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-36: PADS\_DAI0\_0\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000038_1b42c56d8cfdf6bb7b4011a501dfb3114983c4d3c7f12bab708aa09ae55c118b.png)

Table 13-41: PADS\_DAI0\_0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration             |
|--------------------|------------|-------------------------------------|
| 31:28 (R/W)        | DAI8       | Drive Strength Control for DAI8 IO. |
| 27:24 (R/W)        | DAI7       | Drive Strength Control for DAI7 IO. |
| 23:20 (R/W)        | DAI6       | Drive Strength Control for DAI6 IO. |
| 19:16 (R/W)        | DAI5       | Drive Strength Control for DAI5 IO. |
| 15:12 (R/W)        | DAI4       | Drive Strength Control for DAI4 IO. |
| 11:8 (R/W)         | DAI3       | Drive Strength Control for DAI3 IO. |
| 7:4 (R/W)          | DAI2       | Drive Strength Control for DAI2 IO. |
| 3:0 (R/W)          | DAI1       | Drive Strength Control for DAI1 IO. |

## DAI0 Pins 9 to 16 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-37: PADS\_DAI0\_1\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000039_2d2b0b85538f345650106af489c682f4d3f371a3c5f115e70d4d34ef44af38fc.png)

Table 13-42: PADS\_DAI0\_1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration              |
|--------------------|------------|--------------------------------------|
| 31:28 (R/W)        | DAI16      | Drive Strength Control for DAI16 IO. |
| 27:24 (R/W)        | DAI15      | Drive Strength Control for DAI15 IO. |
| 23:20 (R/W)        | DAI14      | Drive Strength Control for DAI14 IO. |
| 19:16 (R/W)        | DAI13      | Drive Strength Control for DAI13 IO. |
| 15:12 (R/W)        | DAI12      | Drive Strength Control for DAI12 IO. |
| 11:8 (R/W)         | DAI11      | Drive Strength Control for DAI11 IO. |
| 7:4 (R/W)          | DAI10      | Drive Strength Control for DAI10 IO. |
| 3:0 (R/W)          | DAI9       | Drive Strength Control for DAI9 IO.  |

## DAI0 Pins 17 to 20 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-38: PADS\_DAI0\_2\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000040_0b9b8ca62238f22e78fe8fc5b6c08726b1467441485806a015dfe48254fd09a3.png)

Table 13-43: PADS\_DAI0\_2\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration              |
|--------------------|------------|--------------------------------------|
| 15:12 (R/W)        | DAI20      | Drive Strength Control for DAI20 IO. |
| 11:8 (R/W)         | DAI19      | Drive Strength Control for DAI19 IO. |
| 7:4 (R/W)          | DAI18      | Drive Strength Control for DAI18 IO. |
| 3:0 (R/W)          | DAI17      | Drive Strength Control for DAI17 IO. |

## DAI0 Port Input Enable Control Register

The PADS\_DAI0\_IE register configures input enable control of the DAI0 (20 pins) pads. If =0 implies input buffer disable and if =1 implies enable.

Figure 13-39: PADS\_DAI0\_IE Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000041_6aa7688f8218526e116311ea31c8a3496261e1bff0764961ee240977c610f46e.png)

Table 13-44: PADS\_DAI0\_IE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration    |
|--------------------|------------|----------------------------|
| 19:0               | VALUE      | DAI0 Input Enable Control. |

## DAI0 Pull Enable Register

Figure 13-40: PADS\_DAI0\_PE Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000042_7d7cc50677036620b1d67b66485842c68b796605a73604ba92af8f9c3cb5ecb7.png)

Table 13-45: PADS\_DAI0\_PE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 19 (R/W)           | DAI20_PE   | Pull Enable for DAI20 IO. |
| 18 (R/W)           | DAI19_PE   | Pull Enable for DAI19 IO. |
| 17 (R/W)           | DAI18_PE   | Pull Enable for DAI18 IO. |
| 16 (R/W)           | DAI17_PE   | Pull Enable for DAI17 IO. |

Table 13-45: PADS\_DAI0\_PE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 15 (R/W)           | DAI16_PE   | Pull Enable for DAI16 IO. |
| 14 (R/W)           | DAI15_PE   | Pull Enable for DAI15 IO. |
| 13 (R/W)           | DAI14_PE   | Pull Enable for DAI14 IO. |
| 12 (R/W)           | DAI13_PE   | Pull Enable for DAI13 IO. |
| 11 (R/W)           | DAI12_PE   | Pull Enable for DAI12 IO. |
| 10 (R/W)           | DAI11_PE   | Pull Enable for DAI11 IO. |
| 9 (R/W)            | DAI10_PE   | Pull Enable for DAI10 IO. |
| 8 (R/W)            | DAI9_PE    | Pull Enable for DAI9 IO.  |
| 7 (R/W)            | DAI8_PE    | Pull Enable for DAI8 IO.  |
| 6 (R/W)            | DAI7_PE    | Pull Enable for DAI7 IO.  |
| 5 (R/W)            | DAI6_PE    | Pull Enable for DAI6 IO.  |
| 4 (R/W)            | DAI5_PE    | Pull Enable for DAI5 IO.  |
| 3 (R/W)            | DAI4_PE    | Pull Enable for DAI4 IO.  |
| 2 (R/W)            | DAI3_PE    | Pull Enable for DAI3 IO.  |
| 1 (R/W)            | DAI2_PE    | Pull Enable for DAI2 IO.  |
| 0 (R/W)            | DAI1_PE    | Pull Enable for DAI1 IO.  |

## DAI0 Pull Selector Register

Figure 13-41: PADS\_DAI0\_PS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000043_87f5a19b5062d612ece71ab354d647f147a56e510a7815ed9aa23052284ff4d2.png)

Table 13-46: PADS\_DAI0\_PS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration     |
|--------------------|------------|-----------------------------|
| 19 (R/W)           | DAI20_PS   | Pull Selector for DAI20 IO. |
| 18 (R/W)           | DAI19_PS   | Pull Selector for DAI19 IO. |
| 17 (R/W)           | DAI18_PS   | Pull Selector for DAI18 IO. |
| 16 (R/W)           | DAI17_PS   | Pull Selector for DAI17 IO. |

Table 13-46: PADS\_DAI0\_PS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration     |
|--------------------|------------|-----------------------------|
| 15 (R/W)           | DAI16_PS   | Pull Selector for DAI16 IO. |
| 14 (R/W)           | DAI15_PS   | Pull Selector for DAI15 IO. |
| 13 (R/W)           | DAI14_PS   | Pull Selector for DAI14 IO. |
| 12 (R/W)           | DAI13_PS   | Pull Selector for DAI13 IO. |
| 11 (R/W)           | DAI12_PS   | Pull Selector for DAI12 IO. |
| 10 (R/W)           | DAI11_PS   | Pull Selector for DAI11 IO. |
| 9 (R/W)            | DAI10_PS   | Pull Selector for DAI10 IO. |
| 8 (R/W)            | DAI9_PS    | Pull Selector for DAI9 IO.  |
| 7 (R/W)            | DAI8_PS    | Pull Selector for DAI8 IO.  |
| 6 (R/W)            | DAI7_PS    | Pull Selector for DAI7 IO.  |
| 5 (R/W)            | DAI6_PS    | Pull Selector for DAI6 IO.  |
| 4 (R/W)            | DAI5_PS    | Pull Selector for DAI5 IO.  |
| 3 (R/W)            | DAI4_PS    | Pull Selector for DAI4 IO.  |
| 2 (R/W)            | DAI3_PS    | Pull Selector for DAI3 IO.  |
| 1 (R/W)            | DAI2_PS    | Pull Selector for DAI2 IO.  |
| 0 (R/W)            | DAI1_PS    | Pull Selector for DAI1 IO.  |

## DAI0 Slew Rate Control Register

Figure 13-42: PADS\_DAI0\_SL Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000044_873506f0d3b7cde626c5761f43945884573ae762c8c6b519c77a8f5fa488b26d.png)

Table 13-47: PADS\_DAI0\_SL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration         |
|--------------------|------------|---------------------------------|
| 19 (R/W)           | DAI20_SL   | Slew Rate Control for DAI20 IO. |
| 18 (R/W)           | DAI19_SL   | Slew Rate Control for DAI19 IO. |
| 17 (R/W)           | DAI18_SL   | Slew Rate Control for DAI18 IO. |
| 16 (R/W)           | DAI17_SL   | Slew Rate Control for DAI17 IO. |

Table 13-47: PADS\_DAI0\_SL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration         |
|--------------------|------------|---------------------------------|
| 15 (R/W)           | DAI16_SL   | Slew Rate Control for DAI16 IO. |
| 14 (R/W)           | DAI15_SL   | Slew Rate Control for DAI15 IO. |
| 13 (R/W)           | DAI14_SL   | Slew Rate Control for DAI14 IO. |
| 12 (R/W)           | DAI13_SL   | Slew Rate Control for DAI13 IO. |
| 11 (R/W)           | DAI12_SL   | Slew Rate Control for DAI12 IO. |
| 10 (R/W)           | DAI11_SL   | Slew Rate Control for DAI11 IO. |
| 9 (R/W)            | DAI10_SL   | Slew Rate Control for DAI10 IO. |
| 8 (R/W)            | DAI9_SL    | Slew Rate Control for DAI9 IO.  |
| 7 (R/W)            | DAI8_SL    | Slew Rate Control for DAI8 IO.  |
| 6 (R/W)            | DAI7_SL    | Slew Rate Control for DAI7 IO.  |
| 5 (R/W)            | DAI6_SL    | Slew Rate Control for DAI6 IO.  |
| 4 (R/W)            | DAI5_SL    | Slew Rate Control for DAI5 IO.  |
| 3 (R/W)            | DAI4_SL    | Slew Rate Control for DAI4 IO.  |
| 2 (R/W)            | DAI3_SL    | Slew Rate Control for DAI3 IO.  |
| 1 (R/W)            | DAI2_SL    | Slew Rate Control for DAI2 IO.  |
| 0 (R/W)            | DAI1_SL    | Slew Rate Control for DAI1 IO.  |

## DAI0 Strong Pull Up Control Register

Figure 13-43: PADS\_DAI0\_SPU Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000045_187332c6c5770fca51a9baa07d5ea2086f66fda410b6e760d1e87693f6f3c7de.png)

Table 13-48: PADS\_DAI0\_SPU Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration              |
|--------------------|------------|--------------------------------------|
| 19 (R/W)           | DAI20_SPU  | Strong Pull Up Control for DAI20 IO. |
| 18 (R/W)           | DAI19_SPU  | Strong Pull Up Control for DAI19 IO. |
| 17 (R/W)           | DAI18_SPU  | Strong Pull Up Control for DAI18 IO. |
| 16 (R/W)           | DAI17_SPU  | Strong Pull Up Control for DAI17 IO. |

Table 13-48: PADS\_DAI0\_SPU Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration              |
|--------------------|------------|--------------------------------------|
| 15 (R/W)           | DAI16_SPU  | Strong Pull Up Control for DAI16 IO. |
| 14 (R/W)           | DAI15_SPU  | Strong Pull Up Control for DAI15 IO. |
| 13 (R/W)           | DAI14_SPU  | Strong Pull Up Control for DAI14 IO. |
| 12 (R/W)           | DAI13_SPU  | Strong Pull Up Control for DAI13 IO. |
| 11 (R/W)           | DAI12_SPU  | Strong Pull Up Control for DAI12 IO. |
| 10 (R/W)           | DAI11_SPU  | Strong Pull Up Control for DAI11 IO. |
| 9 (R/W)            | DAI10_SPU  | Strong Pull Up Control for DAI10 IO. |
| 8 (R/W)            | DAI9_SPU   | Strong Pull Up Control for DAI9 IO.  |
| 7 (R/W)            | DAI8_SPU   | Strong Pull Up Control for DAI8 IO.  |
| 6 (R/W)            | DAI7_SPU   | Strong Pull Up Control for DAI7 IO.  |
| 5 (R/W)            | DAI6_SPU   | Strong Pull Up Control for DAI6 IO.  |
| 4 (R/W)            | DAI5_SPU   | Strong Pull Up Control for DAI5 IO.  |
| 3 (R/W)            | DAI4_SPU   | Strong Pull Up Control for DAI4 IO.  |
| 2 (R/W)            | DAI3_SPU   | Strong Pull Up Control for DAI3 IO.  |
| 1 (R/W)            | DAI2_SPU   | Strong Pull Up Control for DAI2 IO.  |
| 0 (R/W)            | DAI1_SPU   | Strong Pull Up Control for DAI1 IO.  |

## DAI1 Pins 1 to 8 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-44: PADS\_DAI1\_0\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000046_1b42c56d8cfdf6bb7b4011a501dfb3114983c4d3c7f12bab708aa09ae55c118b.png)

Table 13-49: PADS\_DAI1\_0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration             |
|--------------------|------------|-------------------------------------|
| 31:28 (R/W)        | DAI8       | Drive Strength Control for DAI8 IO. |
| 27:24 (R/W)        | DAI7       | Drive Strength Control for DAI7 IO. |
| 23:20 (R/W)        | DAI6       | Drive Strength Control for DAI6 IO. |
| 19:16 (R/W)        | DAI5       | Drive Strength Control for DAI5 IO. |
| 15:12 (R/W)        | DAI4       | Drive Strength Control for DAI4 IO. |
| 11:8 (R/W)         | DAI3       | Drive Strength Control for DAI3 IO. |
| 7:4 (R/W)          | DAI2       | Drive Strength Control for DAI2 IO. |
| 3:0 (R/W)          | DAI1       | Drive Strength Control for DAI1 IO. |

## DAI1 Pins 9 to 16 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-45: PADS\_DAI1\_1\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000047_2d2b0b85538f345650106af489c682f4d3f371a3c5f115e70d4d34ef44af38fc.png)

Table 13-50: PADS\_DAI1\_1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration              |
|--------------------|------------|--------------------------------------|
| 31:28 (R/W)        | DAI16      | Drive Strength Control for DAI16 IO. |
| 27:24 (R/W)        | DAI15      | Drive Strength Control for DAI15 IO. |
| 23:20 (R/W)        | DAI14      | Drive Strength Control for DAI14 IO. |
| 19:16 (R/W)        | DAI13      | Drive Strength Control for DAI13 IO. |
| 15:12 (R/W)        | DAI12      | Drive Strength Control for DAI12 IO. |
| 11:8 (R/W)         | DAI11      | Drive Strength Control for DAI11 IO. |
| 7:4 (R/W)          | DAI10      | Drive Strength Control for DAI10 IO. |
| 3:0 (R/W)          | DAI9       | Drive Strength Control for DAI9 IO.  |

## DAI1 Pins 17 to 20 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-46: PADS\_DAI1\_2\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000048_0b9b8ca62238f22e78fe8fc5b6c08726b1467441485806a015dfe48254fd09a3.png)

Table 13-51: PADS\_DAI1\_2\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration              |
|--------------------|------------|--------------------------------------|
| 15:12 (R/W)        | DAI20      | Drive Strength Control for DAI20 IO. |
| 11:8 (R/W)         | DAI19      | Drive Strength Control for DAI19 IO. |
| 7:4 (R/W)          | DAI18      | Drive Strength Control for DAI18 IO. |
| 3:0 (R/W)          | DAI17      | Drive Strength Control for DAI17 IO. |

## DAI1 Port Input Enable Control Register

The PADS\_DAI1\_IE register configures input enable control of the DAI1 (20 pins) pads. If =0 implies input buffer disable and if =1 implies enable.

Figure 13-47: PADS\_DAI1\_IE Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000049_161b312a0b76b1860a4dd3c3a2d717c410ccb5754b1cf3d7cdd9f3bedfdbb9a3.png)

Table 13-52: PADS\_DAI1\_IE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration    |
|--------------------|------------|----------------------------|
| 19:0               | VALUE      | DAI1 Input Enable Control. |

## DAI1 Pull Enable Register

Figure 13-48: PADS\_DAI1\_PE Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000050_7d7cc50677036620b1d67b66485842c68b796605a73604ba92af8f9c3cb5ecb7.png)

Table 13-53: PADS\_DAI1\_PE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 19 (R/W)           | DAI20_PE   | Pull Enable for DAI20 IO. |
| 18 (R/W)           | DAI19_PE   | Pull Enable for DAI19 IO. |
| 17 (R/W)           | DAI18_PE   | Pull Enable for DAI18 IO. |
| 16 (R/W)           | DAI17_PE   | Pull Enable for DAI17 IO. |

Table 13-53: PADS\_DAI1\_PE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 15 (R/W)           | DAI16_PE   | Pull Enable for DAI16 IO. |
| 14 (R/W)           | DAI15_PE   | Pull Enable for DAI15 IO. |
| 13 (R/W)           | DAI14_PE   | Pull Enable for DAI14 IO. |
| 12 (R/W)           | DAI13_PE   | Pull Enable for DAI13 IO. |
| 11 (R/W)           | DAI12_PE   | Pull Enable for DAI12 IO. |
| 10 (R/W)           | DAI11_PE   | Pull Enable for DAI11 IO. |
| 9 (R/W)            | DAI10_PE   | Pull Enable for DAI10 IO. |
| 8 (R/W)            | DAI9_PE    | Pull Enable for DAI9 IO.  |
| 7 (R/W)            | DAI8_PE    | Pull Enable for DAI8 IO.  |
| 6 (R/W)            | DAI7_PE    | Pull Enable for DAI7 IO.  |
| 5 (R/W)            | DAI6_PE    | Pull Enable for DAI6 IO.  |
| 4 (R/W)            | DAI5_PE    | Pull Enable for DAI5 IO.  |
| 3 (R/W)            | DAI4_PE    | Pull Enable for DAI4 IO.  |
| 2 (R/W)            | DAI3_PE    | Pull Enable for DAI3 IO.  |
| 1 (R/W)            | DAI2_PE    | Pull Enable for DAI2 IO.  |
| 0 (R/W)            | DAI1_PE    | Pull Enable for DAI1 IO.  |

## DAI1 Pull Selector Register

Figure 13-49: PADS\_DAI1\_PS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000051_87f5a19b5062d612ece71ab354d647f147a56e510a7815ed9aa23052284ff4d2.png)

Table 13-54: PADS\_DAI1\_PS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration     |
|--------------------|------------|-----------------------------|
| 19 (R/W)           | DAI20_PS   | Pull Selector for DAI20 IO. |
| 18 (R/W)           | DAI19_PS   | Pull Selector for DAI19 IO. |
| 17 (R/W)           | DAI18_PS   | Pull Selector for DAI18 IO. |
| 16 (R/W)           | DAI17_PS   | Pull Selector for DAI17 IO. |

Table 13-54: PADS\_DAI1\_PS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration     |
|--------------------|------------|-----------------------------|
| 15 (R/W)           | DAI16_PS   | Pull Selector for DAI16 IO. |
| 14 (R/W)           | DAI15_PS   | Pull Selector for DAI15 IO. |
| 13 (R/W)           | DAI14_PS   | Pull Selector for DAI14 IO. |
| 12 (R/W)           | DAI13_PS   | Pull Selector for DAI13 IO. |
| 11 (R/W)           | DAI12_PS   | Pull Selector for DAI12 IO. |
| 10 (R/W)           | DAI11_PS   | Pull Selector for DAI11 IO. |
| 9 (R/W)            | DAI10_PS   | Pull Selector for DAI10 IO. |
| 8 (R/W)            | DAI9_PS    | Pull Selector for DAI9 IO.  |
| 7 (R/W)            | DAI8_PS    | Pull Selector for DAI8 IO.  |
| 6 (R/W)            | DAI7_PS    | Pull Selector for DAI7 IO.  |
| 5 (R/W)            | DAI6_PS    | Pull Selector for DAI6 IO.  |
| 4 (R/W)            | DAI5_PS    | Pull Selector for DAI5 IO.  |
| 3 (R/W)            | DAI4_PS    | Pull Selector for DAI4 IO.  |
| 2 (R/W)            | DAI3_PS    | Pull Selector for DAI3 IO.  |
| 1 (R/W)            | DAI2_PS    | Pull Selector for DAI2 IO.  |
| 0 (R/W)            | DAI1_PS    | Pull Selector for DAI1 IO.  |

## DAI1 Slew Rate Control Register

Figure 13-50: PADS\_DAI1\_SL Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000052_873506f0d3b7cde626c5761f43945884573ae762c8c6b519c77a8f5fa488b26d.png)

Table 13-55: PADS\_DAI1\_SL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration         |
|--------------------|------------|---------------------------------|
| 19 (R/W)           | DAI20_SL   | Slew Rate Control for DAI20 IO. |
| 18 (R/W)           | DAI19_SL   | Slew Rate Control for DAI19 IO. |
| 17 (R/W)           | DAI18_SL   | Slew Rate Control for DAI18 IO. |
| 16 (R/W)           | DAI17_SL   | Slew Rate Control for DAI17 IO. |

Table 13-55: PADS\_DAI1\_SL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration         |
|--------------------|------------|---------------------------------|
| 15 (R/W)           | DAI16_SL   | Slew Rate Control for DAI16 IO. |
| 14 (R/W)           | DAI15_SL   | Slew Rate Control for DAI15 IO. |
| 13 (R/W)           | DAI14_SL   | Slew Rate Control for DAI14 IO. |
| 12 (R/W)           | DAI13_SL   | Slew Rate Control for DAI13 IO. |
| 11 (R/W)           | DAI12_SL   | Slew Rate Control for DAI12 IO. |
| 10 (R/W)           | DAI11_SL   | Slew Rate Control for DAI11 IO. |
| 9 (R/W)            | DAI10_SL   | Slew Rate Control for DAI10 IO. |
| 8 (R/W)            | DAI9_SL    | Slew Rate Control for DAI9 IO.  |
| 7 (R/W)            | DAI8_SL    | Slew Rate Control for DAI8 IO.  |
| 6 (R/W)            | DAI7_SL    | Slew Rate Control for DAI7 IO.  |
| 5 (R/W)            | DAI6_SL    | Slew Rate Control for DAI6 IO.  |
| 4 (R/W)            | DAI5_SL    | Slew Rate Control for DAI5 IO.  |
| 3 (R/W)            | DAI4_SL    | Slew Rate Control for DAI4 IO.  |
| 2 (R/W)            | DAI3_SL    | Slew Rate Control for DAI3 IO.  |
| 1 (R/W)            | DAI2_SL    | Slew Rate Control for DAI2 IO.  |
| 0 (R/W)            | DAI1_SL    | Slew Rate Control for DAI1 IO.  |

## DAI1 Strong Pull up Control Register

Figure 13-51: PADS\_DAI1\_SPU Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000053_187332c6c5770fca51a9baa07d5ea2086f66fda410b6e760d1e87693f6f3c7de.png)

Table 13-56: PADS\_DAI1\_SPU Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration              |
|--------------------|------------|--------------------------------------|
| 19 (R/W)           | DAI20_SPU  | Strong Pull Up Control for DAI20 IO. |
| 18 (R/W)           | DAI19_SPU  | Strong Pull Up Control for DAI19 IO. |
| 17 (R/W)           | DAI18_SPU  | Strong Pull Up Control for DAI18 IO. |
| 16 (R/W)           | DAI17_SPU  | Strong Pull Up Control for DAI17 IO. |

Table 13-56: PADS\_DAI1\_SPU Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration              |
|--------------------|------------|--------------------------------------|
| 15 (R/W)           | DAI16_SPU  | Strong Pull Up Control for DAI16 IO. |
| 14 (R/W)           | DAI15_SPU  | Strong Pull Up Control for DAI15 IO. |
| 13 (R/W)           | DAI14_SPU  | Strong Pull Up Control for DAI14 IO. |
| 12 (R/W)           | DAI13_SPU  | Strong Pull Up Control for DAI13 IO. |
| 11 (R/W)           | DAI12_SPU  | Strong Pull Up Control for DAI12 IO. |
| 10 (R/W)           | DAI11_SPU  | Strong Pull Up Control for DAI11 IO. |
| 9 (R/W)            | DAI10_SPU  | Strong Pull Up Control for DAI10 IO. |
| 8 (R/W)            | DAI9_SPU   | Strong Pull Up Control for DAI9 IO.  |
| 7 (R/W)            | DAI8_SPU   | Strong Pull Up Control for DAI8 IO.  |
| 6 (R/W)            | DAI7_SPU   | Strong Pull Up Control for DAI7 IO.  |
| 5 (R/W)            | DAI6_SPU   | Strong Pull Up Control for DAI6 IO.  |
| 4 (R/W)            | DAI5_SPU   | Strong Pull Up Control for DAI5 IO.  |
| 3 (R/W)            | DAI4_SPU   | Strong Pull Up Control for DAI4 IO.  |
| 2 (R/W)            | DAI3_SPU   | Strong Pull Up Control for DAI3 IO.  |
| 1 (R/W)            | DAI2_SPU   | Strong Pull Up Control for DAI2 IO.  |
| 0 (R/W)            | DAI1_SPU   | Strong Pull Up Control for DAI1 IO.  |

## Non-GPIO Control Register

Figure 13-52: PADS\_NONPORTS\_CTL Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000054_8ed46463b44cf95831dc9d77861eb8481cfbf09ec23a80debc1c6763ff64ddc4.png)

Table 13-57: PADS\_NONPORTS\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration               |
|--------------------|------------|---------------------------------------|
| 16 (R/W)           | FAULT_SPU  | Strong Pull-up Enable for FAULT Pin.  |
| 15 (R/W)           | FAULT_PS   | Pull Selector for FAULT Pin.          |
| 14 (R/W)           | FAULT_PE   | Pull Enable for FAULT Pin.            |
| 13 (R/W)           | FAULT_SL   | Slew Rate Enable for FAULT Pin.       |
| 12 (R/W)           | FAULT_ST   | Schmitt Trigger Enable for FAULT Pin. |

Table 13-57: PADS\_NONPORTS\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                  |
|--------------------|-------------|------------------------------------------|
| 11 (R/W)           | FAULTB_SPU  | Strong Pull-up Enable for FAULTB Pin.    |
| 10 (R/W)           | FAULTB_PS   | Pull Selector for FAULTB Pin.            |
| 9 (R/W)            | FAULTB_PE   | Pull Enable for FAULTB Pin.              |
| 8 (R/W)            | FAULTB_SL   | Slew Rate Enable for FAULTB Pin.         |
| 7 (R/W)            | FAULTB_ST   | Schmitt Trigger Enable for FAULTB Pin.   |
| 6 (R/W)            | RESOUTB_SPU | Strong Pull-up Enable for RESETOUTB Pin. |
| 5 (R/W)            | CLKOUT_SPU  | Strong Pull-up Enable for CLKOUT Pin.    |
| 3 (R/W)            | RESOUTB_PS  | Pull Selector for RESETOUTB Pin.         |
| 2 (R/W)            | RESOUTB_PE  | Pull Enable for RESETOUTB Pin.           |
| 1 (R/W)            | CLKOUT_PS   | Pull Selector for CLKOUT Pin.            |
| 0 (R/W)            | CLKOUT_PE   | Pull Enable for CLKOUT Pin.              |

## Non-GPIO Drive Strength Register

The PADS\_NONPORTS\_DS register sets the drive strength and tolerance for the non-GPIO pins. The drive strength is only controlled for pin groups classified by the interface type.

Figure 13-53: PADS\_NONPORTS\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000055_42e219dd43b0e6a50e62c1b373e473a6f65fec41583581c8f7bd8b06c4a62eff.png)

Table 13-58: PADS\_NONPORTS\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration            |
|--------------------|------------|------------------------------------|
| 31:28 (R/W)        | XSPI1      | XSPI1 Pins Drive Strength Control. |
| 27:24 (R/W)        | XSPI0      | XSPI0 Pins Drive Strength Control. |
| 23:20 (R/W)        | LP1CLK     | LP1 Clock Drive Strength Control.  |
| 19:16 (R/W)        | LP0CK      | LP0 Clock Drive Strength Control.  |
| 15:12 (R/W)        | FAULT      | FAULT Drive Strength Control.      |
| 11:8 (R/W)         | FAULTB     | FAULTB Drive Strength Control.     |
| 7:4 (R/W)          | RESOUTB    | RESOUTB Drive Strength Control.    |
| 3:0 (R/W)          | CLKOUT     | CLKOUT Drive Strength Control.     |

## EMAC Non-GPIO Drive Strength Register

The PADS\_NONPORTS\_DS\_1 register sets the drive strength and tolerance for the non-GPIO pins. The drive strength is only controlled for pin groups classified by the interface type.

Figure 13-54: PADS\_NONPORTS\_DS\_1 Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000056_25fd901670f581148b9ea85cded969e72077aba5df521e23b484a43423484dad.png)

Table 13-59: PADS\_NONPORTS\_DS\_1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                       |
|--------------------|------------|---------------------------------------------------------------|
| 3:0                | ETH0DCK_DS | ETH0DCK Drive Strength Control.                               |
| (R/W)              |            | The default drive strength is 1001. Do not change this value. |

## OTPC Boot Status Register

Figure 13-55: PADS\_OTPC\_BOOT\_STAT Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000057_d0e4b5b3ff3fe11a36012107a1a57176ebb4111a099740c4450416eae6ca09db.png)

Table 13-60: PADS\_OTPC\_BOOT\_STAT Register Fields

| Bit No. (Access)   | Bit Name                    | Description/Enumeration                                     |
|--------------------|-----------------------------|-------------------------------------------------------------|
| 1 (R/NW)           | HSM_OTP_BOOT_COM PLETE      | Status of HSM OTP Boot Completion. 1> Success 0> Failure    |
| 0 (R/NW)           | SYS- TEM_OTP_BOOT_COM PLETE | Status of System OTP Boot Completion. 1> Success 0> Failure |

## Peripheral PAD Configuration 0 Register

The PADS\_PCFG0 register provides several configuration options for the pads and multiplexing for peripherals.

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000058_c6e6b3e763d4ce080812ed747cdc2398ccc203f02f070221136db89a58053b8c.png)

## EMACAUXIE (R/W)

and ETH\_PTPCLKIN Pins Input Enable Control for ETH\_PTPAUX\_MCG\_IN

Figure 13-56: PADS\_PCFG0 Register Diagram

Table 13-61: PADS\_PCFG0 Register Fields

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                                                                |
|--------------------|------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19 (R/W)           | EMAC0_ENDIANNESS | EMAC0 DMATransfer Format. Configure the PADS_PCFG0.EMAC0_ENDIANNESS bit field to set the EMAC0 DMAtransfers to use big-endian or little-endian format. |
| 17 (R/W)           | EMACAUXIE        | Input Enable Control for ETH_PTPAUX_MCG_IN and ETH_PTPCLKIN Pins. 0 Disable input                                                                      |
| 16 (R/W)           | PUTMS            | Pull-up Enable for TMS/SWDIO (Debug Port). 0 Disable pull-up                                                                                           |
| 15                 |                  | 1 Enable pull-up Pull-up Enable for HSM TMS.                                                                                                           |
| (R/W)              | PUTMS1           | 0 Disable pull-up                                                                                                                                      |

Table 13-61: PADS\_PCFG0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                             |
|--------------------|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | CNT0DGSEL   | CNT0 Down Input Select. The PADS_PCFG0 bit selects the input for the CNT0 counter down and gate.                                                                                                                    |
| 7 (R/W)            | CNT0DGSEL   | 0 GPIO port pins                                                                                                                                                                                                    |
| 7 (R/W)            | CNT0DGSEL   | 1 Trigger connection                                                                                                                                                                                                |
| 6 (R/W)            | CNT0UDSEL   | CNT0 Up Input Select. The PADS_PCFG0 bit selects the input for the CNT0 counter up and direction. port pins                                                                                                         |
| 6 (R/W)            | CNT0UDSEL   | 0 GPIO                                                                                                                                                                                                              |
| 6 (R/W)            | CNT0UDSEL   | 1 Trigger connection                                                                                                                                                                                                |
| 4:3 (R/W)          | EMACPHYISEL | Select PHY Interface RGMII/RMII/MII.                                                                                                                                                                                |
| 4:3 (R/W)          | EMACPHYISEL | 0 MII Interface Selects PHY Interface MII                                                                                                                                                                           |
| 4:3 (R/W)          | EMACPHYISEL | 1 RGMII Interface Selects PHY Interface RGMII                                                                                                                                                                       |
| 4:3 (R/W)          | EMACPHYISEL | 2 RMII Interface Selects PHY Interface RMII                                                                                                                                                                         |
| 2 (R/W)            | EMACRESET   | Reset Enable for RGMII. The PADS_PCFG0.EMACRESET bit asserts the reset on the RGMII interface. To select the PHY interface (RGMII or RMII), set the EMACPHYISEL bit as required and then set PADS_PCFG0.EMACRESET . |
| 2 (R/W)            | EMACRESET   | 0 RGMII reset is asserted                                                                                                                                                                                           |
| 2 (R/W)            | EMACRESET   | 1 RGMII reset reset is deasserted                                                                                                                                                                                   |
| 1:0 (R/W)          | EMAC0       | PTP Clock Source 0. The PADS_PCFG0.EMAC0 selects the clock source for the PTP Block in EMAC0.                                                                                                                       |
| 1:0 (R/W)          | EMAC0       | 0 REFCLK (MII or RMII) or CLKO7 (RGMII)                                                                                                                                                                             |
| 1:0 (R/W)          | EMAC0       | 1 SCLK0                                                                                                                                                                                                             |
| 1:0 (R/W)          | EMAC0       | 2 External PTP clock                                                                                                                                                                                                |
| 1:0 (R/W)          | EMAC0       | 3 SCLK0                                                                                                                                                                                                             |

## Peripheral Configuration 1 Register

The PADS\_PCFG1 register provides bits to configure PWM secondary inputs to timers and enable inputs from the TRU.

Figure 13-57: PADS\_PCFG1 Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000059_6484d63e10092bf80cdaab113b1b4773a966e28d5c54522a049fa765ccf3d9cb.png)

Table 13-62: PADS\_PCFG1 Register Fields

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                                                                                           |
|--------------------|------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | PWM_TRUTRIP1B_EN | PWMTRIP1b Input Enable. The PADS_PCFG1.PWM_TRUTRIP1B_EN bit enables the PWMTRIP1b input from the TRU. 0> TRU Trip1b input to PWMdisabled 1>TRU Trip1b input to PWMenabled         |
| 12 (R/W)           | PWM_TRUSYNC_EN   | PWMSYNC Enable. The PADS_PCFG1.PWM_TRUSYNC_EN bit enables the PWMSYNC input from the TRU. 0> Disable, PWMsync input is from external pin 1> Enable, PWMsync input is from the TRU |

## PORTA Pins 0 to 7 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-58: PADS\_PORTA0\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000060_df2037036735db68f1279c8a2c9153cfac6b588ed438b48fbbf967b9e82b5a7d.png)

Table 13-63: PADS\_PORTA0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration            |
|--------------------|------------|------------------------------------|
| 31:28 (R/W)        | PA7        | Drive Strength Control for PA7 IO. |
| 27:24 (R/W)        | PA6        | Drive Strength Control for PA6 IO. |
| 23:20 (R/W)        | PA5        | Drive Strength Control for PA5 IO. |
| 19:16 (R/W)        | PA4        | Drive Strength Control for PA4 IO. |
| 15:12 (R/W)        | PA3        | Drive Strength Control for PA3 IO. |
| 11:8 (R/W)         | PA2        | Drive Strength Control for PA2 IO. |
| 7:4 (R/W)          | PA1        | Drive Strength Control for PA1 IO. |
| 3:0 (R/W)          | PA0        | Drive Strength Control for PA0 IO. |

## PORTA Pins 8 to 15 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-59: PADS\_PORTA1\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000061_98513a758ea4221dead85f8671aac23ef2240c4ea00c419159fda596ceb1526f.png)

Table 13-64: PADS\_PORTA1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration             |
|--------------------|------------|-------------------------------------|
| 31:28 (R/W)        | PA15       | Drive Strength Control for PA15 IO. |
| 27:24 (R/W)        | PA14       | Drive Strength Control for PA14 IO. |
| 23:20 (R/W)        | PA13       | Drive Strength Control for PA13 IO. |
| 19:16 (R/W)        | PA12       | Drive Strength Control for PA12 IO. |
| 15:12 (R/W)        | PA11       | Drive Strength Control for PA11 IO. |
| 11:8 (R/W)         | PA10       | Drive Strength Control for PA10 IO. |
| 7:4 (R/W)          | PA9        | Drive Strength Control for PA9 IO.  |
| 3:0 (R/W)          | PA8        | Drive Strength Control for PA8 IO.  |

## PORTB Pins 0 to 7 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-60: PADS\_PORTB0\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000062_dcdb1710bdd55181823b2783d2bc223572cd60188be4e631b1bc2eaa421087e2.png)

Table 13-65: PADS\_PORTB0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration            |
|--------------------|------------|------------------------------------|
| 31:28 (R/W)        | PB7        | Drive Strength Control for PB7 IO. |
| 27:24 (R/W)        | PB6        | Drive Strength Control for PB6 IO. |
| 23:20 (R/W)        | PB5        | Drive Strength Control for PB5 IO. |
| 19:16 (R/W)        | PB4        | Drive Strength Control for PB4 IO. |
| 15:12 (R/W)        | PB3        | Drive Strength Control for PB3 IO. |
| 11:8 (R/W)         | PB2        | Drive Strength Control for PB2 IO. |
| 7:4 (R/W)          | PB1        | Drive Strength Control for PB1 IO. |
| 3:0 (R/W)          | PB0        | Drive Strength Control for PB0 IO. |

## PORTB Pins 8 to 15 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-61: PADS\_PORTB1\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000063_f8146ebcab16fb477993873fa9566f9a73368e7e9a8c3a4543841ce9ab8eb52e.png)

Table 13-66: PADS\_PORTB1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration             |
|--------------------|------------|-------------------------------------|
| 31:28 (R/W)        | PB15       | Drive Strength Control for PB15 IO. |
| 27:24 (R/W)        | PB14       | Drive Strength Control for PB14 IO. |
| 23:20 (R/W)        | PB13       | Drive Strength Control for PB13 IO. |
| 19:16 (R/W)        | PB12       | Drive Strength Control for PB12 IO. |
| 15:12 (R/W)        | PB11       | Drive Strength Control for PB11 IO. |
| 11:8 (R/W)         | PB10       | Drive Strength Control for PB10 IO. |
| 7:4 (R/W)          | PB9        | Drive Strength Control for PB9 IO.  |
| 3:0 (R/W)          | PB8        | Drive Strength Control for PB8 IO.  |

## PORTC Pins 0 to 7 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-62: PADS\_PORTC0\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000064_e6da72d17d9c1763a8e5fbc2e2e7c4e57b3a6400659e2ea330243fa03042d59e.png)

Table 13-67: PADS\_PORTC0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration            |
|--------------------|------------|------------------------------------|
| 31:28 (R/W)        | PC7        | Drive Strength Control for PC7 IO. |
| 27:24 (R/W)        | PC6        | Drive Strength Control for PC6 IO. |
| 23:20 (R/W)        | PC5        | Drive Strength Control for PC5 IO. |
| 19:16 (R/W)        | PC4        | Drive Strength Control for PC4 IO. |
| 15:12 (R/W)        | PC3        | Drive Strength Control for PC3 IO. |
| 11:8 (R/W)         | PC2        | Drive Strength Control for PC2 IO. |
| 7:4 (R/W)          | PC1        | Drive Strength Control for PC1 IO. |
| 3:0 (R/W)          | PC0        | Drive Strength Control for PC0 IO. |

## PORTC Pins 8 to 15 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-63: PADS\_PORTC1\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000065_6324215192e3ccd9cbd3f8d10ce2927738288c8ce644e878fbbea52273acc996.png)

Table 13-68: PADS\_PORTC1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration             |
|--------------------|------------|-------------------------------------|
| 31:28 (R/W)        | PC15       | Drive Strength Control for PC15 IO. |
| 27:24 (R/W)        | PC14       | Drive Strength Control for PC14 IO. |
| 23:20 (R/W)        | PC13       | Drive Strength Control for PC13 IO. |
| 19:16 (R/W)        | PC12       | Drive Strength Control for PC12 IO. |
| 15:12 (R/W)        | PC11       | Drive Strength Control for PC11 IO. |
| 11:8 (R/W)         | PC10       | Drive Strength Control for PC10 IO. |
| 7:4 (R/W)          | PC9        | Drive Strength Control for PC9 IO.  |
| 3:0 (R/W)          | PC8        | Drive Strength Control for PC8 IO.  |

## PORTD Pins 0 to 7 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-64: PADS\_PORTD0\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000066_78568d09326f40eb154d5bf2a016770dcc437d99ee487cfe365e90ae57c5f4b7.png)

Table 13-69: PADS\_PORTD0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration            |
|--------------------|------------|------------------------------------|
| 31:28 (R/W)        | PD7        | Drive Strength Control for PD7 IO. |
| 27:24 (R/W)        | PD6        | Drive Strength Control for PD6 IO. |
| 23:20 (R/W)        | PD5        | Drive Strength Control for PD5 IO. |
| 19:16 (R/W)        | PD4        | Drive Strength Control for PD4 IO. |
| 15:12 (R/W)        | PD3        | Drive Strength Control for PD3 IO. |
| 11:8 (R/W)         | PD2        | Drive Strength Control for PD2 IO. |
| 7:4 (R/W)          | PD1        | Drive Strength Control for PD1 IO. |
| 3:0 (R/W)          | PD0        | Drive Strength Control for PD0 IO. |

## PORTD Pins 8 to 15 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-65: PADS\_PORTD1\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000067_30b58fcdde8a975d91a2bd920de6ac2260fd3a8f1121cd1b7308575a1826b8b2.png)

Table 13-70: PADS\_PORTD1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration             |
|--------------------|------------|-------------------------------------|
| 31:28 (R/W)        | PD15       | Drive Strength Control for PD15 IO. |
| 27:24 (R/W)        | PD14       | Drive Strength Control for PD14 IO. |
| 23:20 (R/W)        | PD13       | Drive Strength Control for PD13 IO. |
| 19:16 (R/W)        | PD12       | Drive Strength Control for PD12 IO. |
| 15:12 (R/W)        | PD11       | Drive Strength Control for PD11 IO. |
| 11:8 (R/W)         | PD10       | Drive Strength Control for PD10 IO. |
| 7:4 (R/W)          | PD9        | Drive Strength Control for PD9 IO.  |
| 3:0 (R/W)          | PD8        | Drive Strength Control for PD8 IO.  |

## PORTE Pins 0 to 7 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-66: PADS\_PORTE0\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000068_bda8710ea2936578143d34048e525d0ecd3a5e558cf3d56c01408f422ab4caae.png)

Table 13-71: PADS\_PORTE0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration            |
|--------------------|------------|------------------------------------|
| 31:28 (R/W)        | PE7        | Drive Strength Control for PE7 IO. |
| 27:24 (R/W)        | PE6        | Drive Strength Control for PE6 IO. |
| 23:20 (R/W)        | PE5        | Drive Strength Control for PE5 IO. |
| 19:16 (R/W)        | PE4        | Drive Strength Control for PE4 IO. |
| 15:12 (R/W)        | PE3        | Drive Strength Control for PE3 IO. |
| 11:8 (R/W)         | PE2        | Drive Strength Control for PE2 IO. |
| 7:4 (R/W)          | PE1        | Drive Strength Control for PE1 IO. |
| 3:0 (R/W)          | PE0        | Drive Strength Control for PE0 IO. |

## PORTE Pins 8 to 15 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-67: PADS\_PORTE1\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000069_31fef0d88b72487ab4fc05e64a8d5371ca29bb519ef8a295aefbb65f2e4b2d72.png)

Table 13-72: PADS\_PORTE1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration             |
|--------------------|------------|-------------------------------------|
| 31:28 (R/W)        | PE15       | Drive Strength Control for PE15 IO. |
| 27:24 (R/W)        | PE14       | Drive Strength Control for PE14 IO. |
| 23:20 (R/W)        | PE13       | Drive Strength Control for PE13 IO. |
| 19:16 (R/W)        | PE12       | Drive Strength Control for PE12 IO. |
| 15:12 (R/W)        | PE11       | Drive Strength Control for PE11 IO. |
| 11:8 (R/W)         | PE10       | Drive Strength Control for PE10 IO. |
| 7:4 (R/W)          | PE9        | Drive Strength Control for PE9 IO.  |
| 3:0 (R/W)          | PE8        | Drive Strength Control for PE8 IO.  |

## PORTF Pins 0 to 7 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-68: PADS\_PORTF0\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000070_2a3fc7c08c4ae02a7b35e349cc51bfcbb207ca90c623b2d754bea88d58b94fdb.png)

Table 13-73: PADS\_PORTF0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration            |
|--------------------|------------|------------------------------------|
| 31:28 (R/W)        | PF7        | Drive Strength Control for PF7 IO. |
| 27:24 (R/W)        | PF6        | Drive Strength Control for PF6 IO. |
| 23:20 (R/W)        | PF5        | Drive Strength Control for PF5 IO. |
| 19:16 (R/W)        | PF4        | Drive Strength Control for PF4 IO. |
| 15:12 (R/W)        | PF3        | Drive Strength Control for PF3 IO. |
| 11:8 (R/W)         | PF2        | Drive Strength Control for PF2 IO. |
| 7:4 (R/W)          | PF1        | Drive Strength Control for PF1 IO. |
| 3:0 (R/W)          | PF0        | Drive Strength Control for PF0 IO. |

## PORTF Pins 8 to 15 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-69: PADS\_PORTF1\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000071_4b80e65072e7a10ced2ced375d764b64e1e4b0d5708eba5bfdb78e69cb0547f2.png)

Table 13-74: PADS\_PORTF1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration             |
|--------------------|------------|-------------------------------------|
| 31:28 (R/W)        | PF15       | Drive Strength Control for PF15 IO. |
| 27:24 (R/W)        | PF14       | Drive Strength Control for PF14 IO. |
| 23:20 (R/W)        | PF13       | Drive Strength Control for PF13 IO. |
| 19:16 (R/W)        | PF12       | Drive Strength Control for PF12 IO. |
| 15:12 (R/W)        | PF11       | Drive Strength Control for PF11 IO. |
| 11:8 (R/W)         | PF10       | Drive Strength Control for PF10 IO. |
| 7:4 (R/W)          | PF9        | Drive Strength Control for PF9 IO.  |
| 3:0 (R/W)          | PF8        | Drive Strength Control for PF8 IO.  |

## PORTG Pins 0 to 7 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-70: PADS\_PORTG0\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000072_feb2aeaba2005146ff4aea3d0edfb2bb9ec2d2ddd419e7b2bb8995b10f85a040.png)

Table 13-75: PADS\_PORTG0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration            |
|--------------------|------------|------------------------------------|
| 31:28 (R/W)        | PG7        | Drive Strength Control for PG7 IO. |
| 27:24 (R/W)        | PG6        | Drive Strength Control for PG6 IO. |
| 23:20 (R/W)        | PG5        | Drive Strength Control for PG5 IO. |
| 19:16 (R/W)        | PG4        | Drive Strength Control for PG4 IO. |
| 15:12 (R/W)        | PG3        | Drive Strength Control for PG3 IO. |
| 11:8 (R/W)         | PG2        | Drive Strength Control for PG2 IO. |
| 7:4 (R/W)          | PG1        | Drive Strength Control for PG1 IO. |
| 3:0 (R/W)          | PG0        | Drive Strength Control for PG0 IO. |

## PORTG Pins 8 to 15 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-71: PADS\_PORTG1\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000073_fde0134861f630d6a81a9fca8fe8ec73aad82fb395f0fa38a696731b219ae831.png)

Table 13-76: PADS\_PORTG1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration             |
|--------------------|------------|-------------------------------------|
| 31:28 (R/W)        | PG15       | Drive Strength Control for PG15 IO. |
| 27:24 (R/W)        | PG14       | Drive Strength Control for PG14 IO. |
| 23:20 (R/W)        | PG13       | Drive Strength Control for PG13 IO. |
| 19:16 (R/W)        | PG12       | Drive Strength Control for PG12 IO. |
| 15:12 (R/W)        | PG11       | Drive Strength Control for PG11 IO. |
| 11:8 (R/W)         | PG10       | Drive Strength Control for PG10 IO. |
| 7:4 (R/W)          | PG9        | Drive Strength Control for PG9 IO.  |
| 3:0 (R/W)          | PG8        | Drive Strength Control for PG8 IO.  |

## PORTH Pins 0 to 7 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-72: PADS\_PORTH0\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000074_d10eaa14a72f8fb4a79c7b9722c99e70a0f39c0d84458aeb9ab91bb1b7fbf49d.png)

Table 13-77: PADS\_PORTH0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration            |
|--------------------|------------|------------------------------------|
| 31:28 (R/W)        | PH7        | Drive Strength Control for PH7 IO. |
| 27:24 (R/W)        | PH6        | Drive Strength Control for PH6 IO. |
| 23:20 (R/W)        | PH5        | Drive Strength Control for PH5 IO. |
| 19:16 (R/W)        | PH4        | Drive Strength Control for PH4 IO. |
| 15:12 (R/W)        | PH3        | Drive Strength Control for PH3 IO. |
| 11:8 (R/W)         | PH2        | Drive Strength Control for PH2 IO. |
| 7:4 (R/W)          | PH1        | Drive Strength Control for PH1 IO. |
| 3:0 (R/W)          | PH0        | Drive Strength Control for PH0 IO. |

## PORTH Pins 8 and 9 Drive Strength Control Register

The default drive strength is 1001. Do not change this value.

Figure 13-73: PADS\_PORTH1\_DS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000075_251d4d750c2dc019851185b21defd39b61b3813b730bb082f02f4e342cea76e3.png)

Table 13-78: PADS\_PORTH1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration            |
|--------------------|------------|------------------------------------|
| 31:28 (R/W)        | PH15       | DS control for PH15 IO.            |
| 27:24 (R/W)        | PH14       | DS control for PH14 IO.            |
| 23:20 (R/W)        | PH13       | DS control for PH13 IO.            |
| 19:16 (R/W)        | PH12       | DS control for PH12 IO.            |
| 15:12 (R/W)        | PH11       | DS control for PH11 IO.            |
| 11:8 (R/W)         | PH10       | DS control for PH10 IO.            |
| 7:4 (R/W)          | PH9        | Drive Strength Control for PH9 IO. |
| 3:0 (R/W)          | PH8        | Drive Strength Control for PH8 IO. |

## PORTA and PORTB Pull Enable Register

Figure 13-74: PADS\_PORT\_AB\_PE Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000076_eef5af75edb306a180e82368018f1d6999b0ee3fb5197b32a79059ef0d55a9db.png)

Table 13-79: PADS\_PORT\_AB\_PE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31 (R/W)           | PB15_PE    | Pull Enable for PB15 IO.  |
| 30 (R/W)           | PB14_PE    | Pull Enable for PB14 IO.  |
| 29 (R/W)           | PB13_PE    | Pull Enable for PB13 IO.  |
| 28 (R/W)           | PB12_PE    | Pull Enable for PB12 IO.  |
| 27 (R/W)           | PB11_PE    | Pull Enable for PB11 IO.  |
| 26 (R/W)           | PB10_PE    | Pull Enable for PB10 IO.  |
| 25 (R/W)           | PB9_PE     | Pull Enable for PB9 IO.   |
| 24 (R/W)           | PB8_PE     | Pull Enable for PB8 IO.   |
| 23 (R/W)           | PB7_PE     | Pull Enable for PB7 IO.   |
| 22 (R/W)           | PB6_PE     | Pull Enable for PB6 IO.   |
| 21 (R/W)           | PB5_PE     | Pull Enable for PB5 IO.   |
| 20 (R/W)           | PB4_PE     | Pull Enable for PB4 IO.   |
| 19 (R/W)           | PB3_PE     | Pull Enable for PB3 IO.   |
| 18 (R/W)           | PB2_PE     | Pull Enable for PB2 IO.   |
| 17 (R/W)           | PB1_PE     | Pull Enable for PB1 IO.   |
| 16 (R/W)           | PB0_PE     | Pull Enable for PB0 IO.   |

Table 13-79: PADS\_PORT\_AB\_PE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 15 (R/W)           | PA15_PE    | Pull Enable for PA15 IO.  |
| 14 (R/W)           | PA14_PE    | Pull Enable for PA14 IO.  |
| 13 (R/W)           | PA13_PE    | Pull Enable for PA13 IO.  |
| 12 (R/W)           | PA12_PE    | Pull Enable for PA12 IO.  |
| 11 (R/W)           | PA11_PE    | Pull Enable for PA11 IO.  |
| 10 (R/W)           | PA10_PE    | Pull Enable for PA10 IO.  |
| 9 (R/W)            | PA9_PE     | Pull Enable for PA9 IO.   |
| 8 (R/W)            | PA8_PE     | Pull Enable for PA8 IO.   |
| 7 (R/W)            | PA7_PE     | Pull Enable for PA7 IO.   |
| 6 (R/W)            | PA6_PE     | Pull Enable for PA6 IO.   |
| 5 (R/W)            | PA5_PE     | Pull Enable for PA5 IO.   |
| 4 (R/W)            | PA4_PE     | Pull Enable for PA4 IO.   |
| 3 (R/W)            | PA3_PE     | Pull Enable for PA3 IO.   |
| 2 (R/W)            | PA2_PE     | Pull Enable for PA2 IO.   |
| 1 (R/W)            | PA1_PE     | Pull Enable for PA1 IO.   |
| 0 (R/W)            | PA0_PE     | Pull Enable for PA0 IO.   |

## PORTA and PORTB Pull Selector Register

Figure 13-75: PADS\_PORT\_AB\_PS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000077_7427570798ca32d5fd1b9212712c62c777e1cb61c752147ca148bc07c58af685.png)

Table 13-80: PADS\_PORT\_AB\_PS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration    |
|--------------------|------------|----------------------------|
| 31 (R/W)           | PB15_PS    | Pull Selector for PB15 IO. |
| 30 (R/W)           | PB14_PS    | Pull Selector for PB14 IO. |
| 29 (R/W)           | PB13_PS    | Pull Selector for PB13 IO. |
| 28 (R/W)           | PB12_PS    | Pull Selector for PB12 IO. |
| 27 (R/W)           | PB11_PS    | Pull Selector for PB11 IO. |
| 26 (R/W)           | PB10_PS    | Pull Selector for PB10 IO. |
| 25 (R/W)           | PB9_PS     | Pull Selector for PB9 IO.  |
| 24 (R/W)           | PB8_PS     | Pull Selector for PB8 IO.  |
| 23 (R/W)           | PB7_PS     | Pull Selector for PB7 IO.  |
| 22 (R/W)           | PB6_PS     | Pull Selector for PB6 IO.  |
| 21 (R/W)           | PB5_PS     | Pull Selector for PB5 IO.  |
| 20 (R/W)           | PB4_PS     | Pull Selector for PB4 IO.  |
| 19 (R/W)           | PB3_PS     | Pull Selector for PB3 IO.  |
| 18 (R/W)           | PB2_PS     | Pull Selector for PB2 IO.  |
| 17 (R/W)           | PB1_PS     | Pull Selector for PB1 IO.  |
| 16 (R/W)           | PB0_PS     | Pull Selector for PB0 IO.  |

Table 13-80: PADS\_PORT\_AB\_PS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration    |
|--------------------|------------|----------------------------|
| 15 (R/W)           | PA15_PS    | Pull Selector for PA15 IO. |
| 14 (R/W)           | PA14_PS    | Pull Selector for PA14 IO. |
| 13 (R/W)           | PA13_PS    | Pull Selector for PA13 IO. |
| 12 (R/W)           | PA12_PS    | Pull Selector for PA12 IO. |
| 11 (R/W)           | PA11_PS    | Pull Selector for PA11 IO. |
| 10 (R/W)           | PA10_PS    | Pull Selector for PA10 IO. |
| 9 (R/W)            | PA9_PS     | Pull Selector for PA9 IO.  |
| 8 (R/W)            | PA8_PS     | Pull Selector for PA8 IO.  |
| 7 (R/W)            | PA7_PS     | Pull Selector for PA7 IO.  |
| 6 (R/W)            | PA6_PS     | Pull Selector for PA6 IO.  |
| 5 (R/W)            | PA5_PS     | Pull Selector for PA5 IO.  |
| 4 (R/W)            | PA4_PS     | Pull Selector for PA4 IO.  |
| 3 (R/W)            | PA3_PS     | Pull Selector for PA3 IO.  |
| 2 (R/W)            | PA2_PS     | Pull Selector for PA2 IO.  |
| 1 (R/W)            | PA1_PS     | Pull Selector for PA1 IO.  |
| 0 (R/W)            | PA0_PS     | Pull Selector for PA0 IO.  |

## PORTA and PORTB Slew Rate Control Register

Figure 13-76: PADS\_PORT\_AB\_SL Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000078_3178d573494f3679b886cbc8d10996aabae03649a74fa499e7b7e921805b7d9d.png)

Table 13-81: PADS\_PORT\_AB\_SL Register Fields

|   Bit No. (Access) | Bit Name   | Description/Enumeration        |
|--------------------|------------|--------------------------------|
|                 31 | PB15_SL    | Slew Rate Control for PB15 IO. |

Table 13-81: PADS\_PORT\_AB\_SL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration        |
|--------------------|------------|--------------------------------|
| 30 (R/W)           | PB14_SL    | Slew Rate Control for PB14 IO. |
| 29 (R/W)           | PB13_SL    | Slew Rate Control for PB13 IO. |
| 28 (R/W)           | PB12_SL    | Slew Rate Control for PB12 IO. |
| 27 (R/W)           | PB11_SL    | Slew Rate Control for PB11 IO. |
| 26 (R/W)           | PB10_SL    | Slew Rate Control for PB10 IO. |
| 25 (R/W)           | PB9_SL     | Slew Rate Control for PB9 IO.  |
| 24 (R/W)           | PB8_SL     | Slew Rate Control for PB8 IO.  |
| 23 (R/W)           | PB7_SL     | Slew Rate Control for PB7 IO.  |
| 22 (R/W)           | PB6_SL     | Slew Rate Control for PB6 IO.  |
| 21 (R/W)           | PB5_SL     | Slew Rate Control for PB5 IO.  |
| 20 (R/W)           | PB4_SL     | Slew Rate Control for PB4 IO.  |
| 19 (R/W)           | PB3_SL     | Slew Rate Control for PB3 IO.  |
| 18 (R/W)           | PB2_SL     | Slew Rate Control for PB2 IO.  |
| 17 (R/W)           | PB1_SL     | Slew Rate Control for PB1 IO.  |
| 16 (R/W)           | PB0_SL     | Slew Rate Control for PB0 IO.  |
| 15 (R/W)           | PA15_SL    | Slew Rate Control for PA15 IO. |

Table 13-81: PADS\_PORT\_AB\_SL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration        |
|--------------------|------------|--------------------------------|
| 14 (R/W)           | PA14_SL    | Slew Rate Control for PA14 IO. |
| 13 (R/W)           | PA13_SL    | Slew Rate Control for PA13 IO. |
| 12 (R/W)           | PA12_SL    | Slew Rate Control for PA12 IO. |
| 11 (R/W)           | PA11_SL    | Slew Rate Control for PA11 IO. |
| 10 (R/W)           | PA10_SL    | Slew Rate Control for PA10 IO. |
| 9 (R/W)            | PA9_SL     | Slew Rate Control for PA9 IO.  |
| 8 (R/W)            | PA8_SL     | Slew Rate Control for PA8 IO.  |
| 7 (R/W)            | PA7_SL     | Slew Rate Control for PA7 IO.  |
| 6 (R/W)            | PA6_SL     | Slew Rate Control for PA6 IO.  |
| 5 (R/W)            | PA5_SL     | Slew Rate Control for PA5 IO.  |
| 4 (R/W)            | PA4_SL     | Slew Rate Control for PA4 IO.  |
| 3 (R/W)            | PA3_SL     | Slew Rate Control for PA3 IO.  |
| 2 (R/W)            | PA2_SL     | Slew Rate Control for PA2 IO.  |

## PORTA and PORTB Strong Pull Up Control Register

Figure 13-77: PADS\_PORT\_AB\_SPU Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000079_3b22b122a657ba381bc8089b408c8ff09a142362a36427b6ecdf2cf04485a1c0.png)

Table 13-82: PADS\_PORT\_AB\_SPU Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration             |
|--------------------|------------|-------------------------------------|
| 31 (R/W)           | PB15_SPU   | Strong Pull Up Control for PB15 IO. |
| 30 (R/W)           | PB14_SPU   | Strong Pull Up Control for PB14 IO. |
| 29 (R/W)           | PB13_SPU   | Strong Pull Up Control for PB13 IO. |
| 28 (R/W)           | PB12_SPU   | Strong Pull Up Control for PB12 IO. |
| 27 (R/W)           | PB11_SPU   | Strong Pull Up Control for PB11 IO. |
| 26 (R/W)           | PB10_SPU   | Strong Pull Up Control for PB10 IO. |
| 25 (R/W)           | PB9_SPU    | Strong Pull Up Control for PB9 IO.  |
| 24 (R/W)           | PB8_SPU    | Strong Pull Up Control for PB8 IO.  |
| 23 (R/W)           | PB7_SPU    | Strong Pull Up Control for PB7 IO.  |
| 22 (R/W)           | PB6_SPU    | Strong Pull Up Control for PB6 IO.  |
| 21 (R/W)           | PB5_SPU    | Strong Pull Up Control for PB5 IO.  |
| 20 (R/W)           | PB4_SPU    | Strong Pull Up Control for PB4 IO.  |
| 19 (R/W)           | PB3_SPU    | Strong Pull Up Control for PB3 IO.  |
| 18 (R/W)           | PB2_SPU    | Strong Pull Up Control for PB2 IO.  |
| 17 (R/W)           | PB1_SPU    | Strong Pull Up Control for PB1 IO.  |
| 16 (R/W)           | PB0_SPU    | Strong Pull Up Control for PB0 IO.  |

Table 13-82: PADS\_PORT\_AB\_SPU Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration             |
|--------------------|------------|-------------------------------------|
| 15 (R/W)           | PA15_SPU   | Strong Pull Up Control for PA15 IO. |
| 14 (R/W)           | PA14_SPU   | Strong Pull Up Control for PA14 IO. |
| 13 (R/W)           | PA13_SPU   | Strong Pull Up Control for PA13 IO. |
| 12 (R/W)           | PA12_SPU   | Strong Pull Up Control for PA12 IO. |
| 11 (R/W)           | PA11_SPU   | Strong Pull Up Control for PA11 IO. |
| 10 (R/W)           | PA10_SPU   | Strong Pull Up Control for PA10 IO. |
| 9 (R/W)            | PA9_SPU    | Strong Pull Up Control for PA9 IO.  |
| 8 (R/W)            | PA8_SPU    | Strong Pull Up Control for PA8 IO.  |
| 7 (R/W)            | PA7_SPU    | Strong Pull Up Control for PA7 IO.  |
| 6 (R/W)            | PA6_SPU    | Strong Pull Up Control for PA6 IO.  |
| 5 (R/W)            | PA5_SPU    | Strong Pull Up Control for PA5 IO.  |
| 4 (R/W)            | PA4_SPU    | Strong Pull Up Control for PA4 IO.  |
| 3 (R/W)            | PA3_SPU    | Strong Pull Up Control for PA3 IO.  |
| 2 (R/W)            | PA2_SPU    | Strong Pull Up Control for PA2 IO.  |
| 1 (R/W)            | PA1_SPU    | Strong Pull Up Control for PA1 IO.  |
| 0 (R/W)            | PA0_SPU    | Strong Pull Up Control for PA0 IO.  |

## PORTC and PORTD Pull Enable Register

Figure 13-78: PADS\_PORT\_CD\_PE Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000080_5bf3e9c0a65fe8b47aeea0c9588ce80c55fb1a134ea4893dfb27bf800c99cca0.png)

Table 13-83: PADS\_PORT\_CD\_PE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31 (R/W)           | PD15_PE    | Pull Enable for PD15 IO.  |
| 30 (R/W)           | PD14_PE    | Pull Enable for PD14 IO.  |
| 29 (R/W)           | PD13_PE    | Pull Enable for PD13 IO.  |
| 28 (R/W)           | PD12_PE    | Pull Enable for PD12 IO.  |
| 27 (R/W)           | PD11_PE    | Pull Enable for PD11 IO.  |
| 26 (R/W)           | PD10_PE    | Pull Enable for PD10 IO.  |
| 25 (R/W)           | PD9_PE     | Pull Enable for PD9 IO.   |
| 24 (R/W)           | PD8_PE     | Pull Enable for PD8 IO.   |
| 23 (R/W)           | PD7_PE     | Pull Enable for PD7 IO.   |
| 22 (R/W)           | PD6_PE     | Pull Enable for PD6 IO.   |
| 21 (R/W)           | PD5_PE     | Pull Enable for PD5 IO.   |
| 20 (R/W)           | PD4_PE     | Pull Enable for PD4 IO.   |
| 19 (R/W)           | PD3_PE     | Pull Enable for PD3 IO.   |
| 18 (R/W)           | PD2_PE     | Pull Enable for PD2 IO.   |
| 17 (R/W)           | PD1_PE     | Pull Enable for PD1 IO.   |
| 16 (R/W)           | PD0_PE     | Pull Enable for PD0 IO.   |

Table 13-83: PADS\_PORT\_CD\_PE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 15 (R/W)           | PC15_PE    | Pull Enable for PC15 IO.  |
| 14 (R/W)           | PC14_PE    | Pull Enable for PC14 IO.  |
| 13 (R/W)           | PC13_PE    | Pull Enable for PC13 IO.  |
| 12 (R/W)           | PC12_PE    | Pull Enable for PC12 IO.  |
| 11 (R/W)           | PC11_PE    | Pull Enable for PC11 IO.  |
| 10 (R/W)           | PC10_PE    | Pull Enable for PC10 IO.  |
| 9 (R/W)            | PC9_PE     | Pull Enable for PC9 IO.   |
| 8 (R/W)            | PC8_PE     | Pull Enable for PC8 IO.   |
| 7 (R/W)            | PC7_PE     | Pull Enable for PC7 IO.   |
| 6 (R/W)            | PC6_PE     | Pull Enable for PC6 IO.   |
| 5 (R/W)            | PC5_PE     | Pull Enable for PC5 IO.   |
| 4 (R/W)            | PC4_PE     | Pull Enable for PC4 IO.   |
| 3 (R/W)            | PC3_PE     | Pull Enable for PC3 IO.   |
| 2 (R/W)            | PC2_PE     | Pull Enable for PC2 IO.   |
| 1 (R/W)            | PC1_PE     | Pull Enable for PC1 IO.   |
| 0 (R/W)            | PC0_PE     | Pull Enable for PC0 IO.   |

## PORTC and PORTD Pull Selector Register

Figure 13-79: PADS\_PORT\_CD\_PS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000081_3cb82f5ba14326e9a333e28b6a96ceccba93e9c23b8eceaf296a4f6b0b0fd544.png)

Table 13-84: PADS\_PORT\_CD\_PS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration    |
|--------------------|------------|----------------------------|
| 31 (R/W)           | PD15_PS    | Pull Selector for PD15 IO. |
| 30 (R/W)           | PD14_PS    | Pull Selector for PD14 IO. |
| 29 (R/W)           | PD13_PS    | Pull Selector for PD13 IO. |
| 28 (R/W)           | PD12_PS    | Pull Selector for PD12 IO. |
| 27 (R/W)           | PD11_PS    | Pull Selector for PD11 IO. |
| 26 (R/W)           | PD10_PS    | Pull Selector for PD10 IO. |
| 25 (R/W)           | PD9_PS     | Pull Selector for PD9 IO.  |
| 24 (R/W)           | PD8_PS     | Pull Selector for PD8 IO.  |
| 23 (R/W)           | PD7_PS     | Pull Selector for PD7 IO.  |
| 22 (R/W)           | PD6_PS     | Pull Selector for PD6 IO.  |
| 21 (R/W)           | PD5_PS     | Pull Selector for PD5 IO.  |
| 20 (R/W)           | PD4_PS     | Pull Selector for PD4 IO.  |
| 19 (R/W)           | PD3_PS     | Pull Selector for PD3 IO.  |
| 18 (R/W)           | PD2_PS     | Pull Selector for PD2 IO.  |
| 17 (R/W)           | PD1_PS     | Pull Selector for PD1 IO.  |
| 16 (R/W)           | PD0_PS     | Pull Selector for PD0 IO.  |

Table 13-84: PADS\_PORT\_CD\_PS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration    |
|--------------------|------------|----------------------------|
| 15 (R/W)           | PC15_PS    | Pull Selector for PC15 IO. |
| 14 (R/W)           | PC14_PS    | Pull Selector for PC14 IO. |
| 13 (R/W)           | PC13_PS    | Pull Selector for PC13 IO. |
| 12 (R/W)           | PC12_PS    | Pull Selector for PC12 IO. |
| 11 (R/W)           | PC11_PS    | Pull Selector for PC11 IO. |
| 10 (R/W)           | PC10_PS    | Pull Selector for PC10 IO. |
| 9 (R/W)            | PC9_PS     | Pull Selector for PC9 IO.  |
| 8 (R/W)            | PC8_PS     | Pull Selector for PC8 IO.  |
| 7 (R/W)            | PC7_PS     | Pull Selector for PC7 IO.  |
| 6 (R/W)            | PC6_PS     | Pull Selector for PC6 IO.  |
| 5 (R/W)            | PC5_PS     | Pull Selector for PC5 IO.  |
| 4 (R/W)            | PC4_PS     | Pull Selector for PC4 IO.  |
| 3 (R/W)            | PC3_PS     | Pull Selector for PC3 IO.  |
| 2 (R/W)            | PC2_PS     | Pull Selector for PC2 IO.  |
| 1 (R/W)            | PC1_PS     | Pull Selector for PC1 IO.  |
| 0 (R/W)            | PC0_PS     | Pull Selector for PC0 IO.  |

## PORTC and PORTD Slew Rate Control Register

Figure 13-80: PADS\_PORT\_CD\_SL Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000082_232a5d9921563df078abb266f1d5dc94c2b1937cad33bf02a7aec5931080f50f.png)

Table 13-85: PADS\_PORT\_CD\_SL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration        |
|--------------------|------------|--------------------------------|
| 31 (R/W)           | PD15_SL    | Slew Rate Control for PD15 IO. |
| 30 (R/W)           | PD14_SL    | Slew Rate Control for PD14 IO. |
| 29 (R/W)           | PD13_SL    | Slew Rate Control for PD13 IO. |
| 28 (R/W)           | PD12_SL    | Slew Rate Control for PD12 IO. |
| 27 (R/W)           | PD11_SL    | Slew Rate Control for PD11 IO. |
| 26 (R/W)           | PD10_SL    | Slew Rate Control for PD10 IO. |
| 25 (R/W)           | PD9_SL     | Slew Rate Control for PD9 IO.  |
| 24 (R/W)           | PD8_SL     | Slew Rate Control for PD8 IO.  |
| 23 (R/W)           | PD7_SL     | Slew Rate Control for PD7 IO.  |
| 22 (R/W)           | PD6_SL     | Slew Rate Control for PD6 IO.  |
| 21 (R/W)           | PD5_SL     | Slew Rate Control for PD5 IO.  |
| 20 (R/W)           | PD4_SL     | Slew Rate Control for PD4 IO.  |
| 19 (R/W)           | PD3_SL     | Slew Rate Control for PD3 IO.  |
| 18 (R/W)           | PD2_SL     | Slew Rate Control for PD2 IO.  |
| 17 (R/W)           | PD1_SL     | Slew Rate Control for PD1 IO.  |
| 16 (R/W)           | PD0_SL     | Slew Rate Control for PD0 IO.  |

Table 13-85: PADS\_PORT\_CD\_SL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration        |
|--------------------|------------|--------------------------------|
| 15 (R/W)           | PC15_SL    | Slew Rate Control for PC15 IO. |
| 14 (R/W)           | PC14_SL    | Slew Rate Control for PC14 IO. |
| 13 (R/W)           | PC13_SL    | Slew Rate Control for PC13 IO. |
| 12 (R/W)           | PC12_SL    | Slew Rate Control for PC12 IO. |
| 11 (R/W)           | PC11_SL    | Slew Rate Control for PC11 IO. |
| 10 (R/W)           | PC10_SL    | Slew Rate Control for PC10 IO. |
| 9 (R/W)            | PC9_SL     | Slew Rate Control for PC9 IO.  |
| 8 (R/W)            | PC8_SL     | Slew Rate Control for PC8 IO.  |
| 7 (R/W)            | PC7_SL     | Slew Rate Control for PC7 IO.  |
| 6 (R/W)            | PC6_SL     | Slew Rate Control for PC6 IO.  |
| 5 (R/W)            | PC5_SL     | Slew Rate Control for PC5 IO.  |
| 4 (R/W)            | PC4_SL     | Slew Rate Control for PC4 IO.  |
| 3 (R/W)            | PC3_SL     | Slew Rate Control for PC3 IO.  |
| 2 (R/W)            | PC2_SL     | Slew Rate Control for PC2 IO.  |
| 1 (R/W)            | PC1_SL     | Slew Rate Control for PC1 IO.  |
| 0 (R/W)            | PC0_SL     | Slew Rate Control for PC0 IO.  |

## PORTC and PORTD Strong Pull Up Control Register

Figure 13-81: PADS\_PORT\_CD\_SPU Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000083_f0c39b083f033a301b671d7cfc9a3979f80c17cad4d23263ffe80f0dbdb5b5b0.png)

Table 13-86: PADS\_PORT\_CD\_SPU Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration             |
|--------------------|------------|-------------------------------------|
| 31 (R/W)           | PD15_SPU   | Strong Pull Up Control for PD15 IO. |
| 30 (R/W)           | PD14_SPU   | Strong Pull Up Control for PD14 IO. |
| 29 (R/W)           | PD13_SPU   | Strong Pull Up Control for PD13 IO. |
| 28 (R/W)           | PD12_SPU   | Strong Pull Up Control for PD12 IO. |
| 27 (R/W)           | PD11_SPU   | Strong Pull Up Control for PD11 IO. |
| 26 (R/W)           | PD10_SPU   | Strong Pull Up Control for PD10 IO. |
| 25 (R/W)           | PD9_SPU    | Strong Pull Up Control for PD9 IO.  |
| 24 (R/W)           | PD8_SPU    | Strong Pull Up Control for PD8 IO.  |
| 23 (R/W)           | PD7_SPU    | Strong Pull Up Control for PD7 IO.  |
| 22 (R/W)           | PD6_SPU    | Strong Pull Up Control for PD6 IO.  |
| 21 (R/W)           | PD5_SPU    | Strong Pull Up Control for PD5 IO.  |
| 20 (R/W)           | PD4_SPU    | Strong Pull Up Control for PD4 IO.  |
| 19 (R/W)           | PD3_SPU    | Strong Pull Up Control for PD3 IO.  |
| 18 (R/W)           | PD2_SPU    | Strong Pull Up Control for PD2 IO.  |
| 17 (R/W)           | PD1_SPU    | Strong Pull Up Control for PD1 IO.  |
| 16 (R/W)           | PD0_SPU    | Strong Pull Up Control for PD0 IO.  |

Table 13-86: PADS\_PORT\_CD\_SPU Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration             |
|--------------------|------------|-------------------------------------|
| 15 (R/W)           | PC15_SPU   | Strong Pull Up Control for PC15 IO. |
| 14 (R/W)           | PC14_SPU   | Strong Pull Up Control for PC14 IO. |
| 13 (R/W)           | PC13_SPU   | Strong Pull Up Control for PC13 IO. |
| 12 (R/W)           | PC12_SPU   | Strong Pull Up Control for PC12 IO. |
| 11 (R/W)           | PC11_SPU   | Strong Pull Up Control for PC11 IO. |
| 10 (R/W)           | PC10_SPU   | Strong Pull Up Control for PC10 IO. |
| 9 (R/W)            | PC9_SPU    | Strong Pull Up Control for PC9 IO.  |
| 8 (R/W)            | PC8_SPU    | Strong Pull Up Control for PC8 IO.  |
| 7 (R/W)            | PC7_SPU    | Strong Pull Up Control for PC7 IO.  |
| 6 (R/W)            | PC6_SPU    | Strong Pull Up Control for PC6 IO.  |
| 5 (R/W)            | PC5_SPU    | Strong Pull Up Control for PC5 IO.  |
| 4 (R/W)            | PC4_SPU    | Strong Pull Up Control for PC4 IO.  |
| 3 (R/W)            | PC3_SPU    | Strong Pull Up Control for PC3 IO.  |
| 2 (R/W)            | PC2_SPU    | Strong Pull Up Control for PC2 IO.  |
| 1 (R/W)            | PC1_SPU    | Strong Pull Up Control for PC1 IO.  |
| 0 (R/W)            | PC0_SPU    | Strong Pull Up Control for PC0 IO.  |

## PORTE and PORTF Pull Enable Register

Figure 13-82: PADS\_PORT\_EF\_PE Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000084_2ecef3ca434a16166ee19fd62d0ae3f860380724449249fa7fa699709d497c78.png)

Table 13-87: PADS\_PORT\_EF\_PE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31 (R/W)           | PF15_PE    | Pull Enable for PF15 IO.  |
| 30 (R/W)           | PF14_PE    | Pull Enable for PF14 IO.  |
| 29 (R/W)           | PF13_PE    | Pull Enable for PF13 IO.  |
| 28 (R/W)           | PF12_PE    | Pull Enable for PF12 IO.  |
| 27 (R/W)           | PF11_PE    | Pull Enable for PF11 IO.  |
| 26 (R/W)           | PF10_PE    | Pull Enable for PF10 IO.  |
| 25 (R/W)           | PF9_PE     | Pull Enable for PF9 IO.   |
| 24 (R/W)           | PF8_PE     | Pull Enable for PF8 IO.   |
| 23 (R/W)           | PF7_PE     | Pull Enable for PF7 IO.   |
| 22 (R/W)           | PF6_PE     | Pull Enable for PF6 IO.   |
| 21 (R/W)           | PF5_PE     | Pull Enable for PF5 IO.   |
| 20 (R/W)           | PF4_PE     | Pull Enable for PF4 IO.   |
| 19 (R/W)           | PF3_PE     | Pull Enable for PF3 IO.   |
| 18 (R/W)           | PF2_PE     | Pull Enable for PF2 IO.   |
| 17 (R/W)           | PF1_PE     | Pull Enable for PF1 IO.   |
| 16 (R/W)           | PF0_PE     | Pull Enable for PF0 IO.   |

Table 13-87: PADS\_PORT\_EF\_PE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 15 (R/W)           | PE15_PE    | Pull Enable for PE15 IO.  |
| 14 (R/W)           | PE14_PE    | Pull Enable for PE14 IO.  |
| 13 (R/W)           | PE13_PE    | Pull Enable for PE13 IO.  |
| 12 (R/W)           | PE12_PE    | Pull Enable for PE12 IO.  |
| 11 (R/W)           | PE11_PE    | Pull Enable for PE11 IO.  |
| 10 (R/W)           | PE10_PE    | Pull Enable for PE10 IO.  |
| 9 (R/W)            | PE9_PE     | Pull Enable for PE9 IO.   |
| 8 (R/W)            | PE8_PE     | Pull Enable for PE8 IO.   |
| 7 (R/W)            | PE7_PE     | Pull Enable for PE7 IO.   |
| 6 (R/W)            | PE6_PE     | Pull Enable for PE6 IO.   |
| 5 (R/W)            | PE5_PE     | Pull Enable for PE5 IO.   |
| 4 (R/W)            | PE4_PE     | Pull Enable for PE4 IO.   |
| 3 (R/W)            | PE3_PE     | Pull Enable for PE3 IO.   |
| 2 (R/W)            | PE2_PE     | Pull Enable for PE2 IO.   |
| 1 (R/W)            | PE1_PE     | Pull Enable for PE1 IO.   |
| 0 (R/W)            | PE0_PE     | Pull Enable for PE0 IO.   |

## PORTE and PORTF Pull Selector Register

Figure 13-83: PADS\_PORT\_EF\_PS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000085_766d33979ce57b785f3fdcb47e3e64bc4854863076423321f0ce699f45bb419f.png)

Table 13-88: PADS\_PORT\_EF\_PS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration    |
|--------------------|------------|----------------------------|
| 31 (R/W)           | PF15_PS    | Pull Selector for PF15 IO. |
| 30 (R/W)           | PF14_PS    | Pull Selector for PF14 IO. |
| 29 (R/W)           | PF13_PS    | Pull Selector for PF13 IO. |
| 28 (R/W)           | PF12_PS    | Pull Selector for PF12 IO. |
| 27 (R/W)           | PF11_PS    | Pull Selector for PF11 IO. |
| 26 (R/W)           | PF10_PS    | Pull Selector for PF10 IO. |
| 25 (R/W)           | PF9_PS     | Pull Selector for PF9 IO.  |
| 24 (R/W)           | PF8_PS     | Pull Selector for PF8 IO.  |
| 23 (R/W)           | PF7_PS     | Pull Selector for PF7 IO.  |
| 22 (R/W)           | PF6_PS     | Pull Selector for PF6 IO.  |
| 21 (R/W)           | PF5_PS     | Pull Selector for PF5 IO.  |
| 20 (R/W)           | PF4_PS     | Pull Selector for PF4 IO.  |
| 19 (R/W)           | PF3_PS     | Pull Selector for PF3 IO.  |
| 18 (R/W)           | PF2_PS     | Pull Selector for PF2 IO.  |
| 17 (R/W)           | PF1_PS     | Pull Selector for PF1 IO.  |
| 16 (R/W)           | PF0_PS     | Pull Selector for PF0 IO.  |

Table 13-88: PADS\_PORT\_EF\_PS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration    |
|--------------------|------------|----------------------------|
| 15 (R/W)           | PE15_PS    | Pull Selector for PE15 IO. |
| 14 (R/W)           | PE14_PS    | Pull Selector for PE14 IO. |
| 13 (R/W)           | PE13_PS    | Pull Selector for PE13 IO. |
| 12 (R/W)           | PE12_PS    | Pull Selector for PE12 IO. |
| 11 (R/W)           | PE11_PS    | Pull Selector for PE11 IO. |
| 10 (R/W)           | PE10_PS    | Pull Selector for PE10 IO. |
| 9 (R/W)            | PE9_PS     | Pull Selector for PE9 IO.  |
| 8 (R/W)            | PE8_PS     | Pull Selector for PE8 IO.  |
| 7 (R/W)            | PE7_PS     | Pull Selector for PE7 IO.  |
| 6 (R/W)            | PE6_PS     | Pull Selector for PE6 IO.  |
| 5 (R/W)            | PE5_PS     | Pull Selector for PE5 IO.  |
| 4 (R/W)            | PE4_PS     | Pull Selector for PE4 IO.  |
| 3 (R/W)            | PE3_PS     | Pull Selector for PE3 IO.  |
| 2 (R/W)            | PE2_PS     | Pull Selector for PE2 IO.  |
| 1 (R/W)            | PE1_PS     | Pull Selector for PE1 IO.  |
| 0 (R/W)            | PE0_PS     | Pull Selector for PE0 IO.  |

## PORTE and PORTF Slew Rate Control Register

Figure 13-84: PADS\_PORT\_EF\_SL Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000086_da924f2bbfc16dd27646a115a434c2ddc608d62e605a6763fff702f320e37ddf.png)

Table 13-89: PADS\_PORT\_EF\_SL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration        |
|--------------------|------------|--------------------------------|
| 31 (R/W)           | PF15_SL    | Slew Rate Control for PF15 IO. |
| 30 (R/W)           | PF14_SL    | Slew Rate Control for PF14 IO. |
| 29 (R/W)           | PF13_SL    | Slew Rate Control for PF13 IO. |
| 28 (R/W)           | PF12_SL    | Slew Rate Control for PF12 IO. |
| 27 (R/W)           | PF11_SL    | Slew Rate Control for PF11 IO. |
| 26 (R/W)           | PF10_SL    | Slew Rate Control for PF10 IO. |
| 25 (R/W)           | PF9_SL     | Slew Rate Control for PF9 IO.  |
| 24 (R/W)           | PF8_SL     | Slew Rate Control for PF8 IO.  |
| 23 (R/W)           | PF7_SL     | Slew Rate Control for PF7 IO.  |
| 22 (R/W)           | PF6_SL     | Slew Rate Control for PF6 IO.  |
| 21 (R/W)           | PF5_SL     | Slew Rate Control for PF5 IO.  |
| 20 (R/W)           | PF4_SL     | Slew Rate Control for PF4 IO.  |
| 19 (R/W)           | PF3_SL     | Slew Rate Control for PF3 IO.  |
| 18 (R/W)           | PF2_SL     | Slew Rate Control for PF2 IO.  |
| 17 (R/W)           | PF1_SL     | Slew Rate Control for PF1 IO.  |
| 16 (R/W)           | PF0_SL     | Slew Rate Control for PF0 IO.  |

Table 13-89: PADS\_PORT\_EF\_SL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration        |
|--------------------|------------|--------------------------------|
| 15 (R/W)           | PE15_SL    | Slew Rate Control for PE15 IO. |
| 14 (R/W)           | PE14_SL    | Slew Rate Control for PE14 IO. |
| 13 (R/W)           | PE13_SL    | Slew Rate Control for PE13 IO. |
| 12 (R/W)           | PE12_SL    | Slew Rate Control for PE12 IO. |
| 11 (R/W)           | PE11_SL    | Slew Rate Control for PE11 IO. |
| 10 (R/W)           | PE10_SL    | Slew Rate Control for PE10 IO. |
| 9 (R/W)            | PE9_SL     | Slew Rate Control for PE9 IO.  |
| 8 (R/W)            | PE8_SL     | Slew Rate Control for PE8 IO.  |
| 7 (R/W)            | PE7_SL     | Slew Rate Control for PE7 IO.  |
| 6 (R/W)            | PE6_SL     | Slew Rate Control for PE6 IO.  |
| 5 (R/W)            | PE5_SL     | Slew Rate Control for PE5 IO.  |
| 4 (R/W)            | PE4_SL     | Slew Rate Control for PE4 IO.  |
| 3 (R/W)            | PE3_SL     | Slew Rate Control for PE3 IO.  |
| 2 (R/W)            | PE2_SL     | Slew Rate Control for PE2 IO.  |
| 1 (R/W)            | PE1_SL     | Slew Rate Control for PE1 IO.  |
| 0 (R/W)            | PE0_SL     | Slew Rate Control for PE0 IO.  |

## PORTE and PORTF Strong Pull Up Control Register

Figure 13-85: PADS\_PORT\_EF\_SPU Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000087_c842951d31b7d74f95577893cb39c0af638edcc71d0c4096de4ea82e267b2c30.png)

Table 13-90: PADS\_PORT\_EF\_SPU Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration             |
|--------------------|------------|-------------------------------------|
| 31 (R/W)           | PF15_SPU   | Strong Pull Up Control for PF15 IO. |
| 30 (R/W)           | PF14_SPU   | Strong Pull Up Control for PF14 IO. |
| 29 (R/W)           | PF13_SPU   | Strong Pull Up Control for PF13 IO. |
| 28 (R/W)           | PF12_SPU   | Strong Pull Up Control for PF12 IO. |
| 27 (R/W)           | PF11_SPU   | Strong Pull Up Control for PF11 IO. |
| 26 (R/W)           | PF10_SPU   | Strong Pull Up Control for PF10 IO. |
| 25 (R/W)           | PF9_SPU    | Strong Pull Up Control for PF9 IO.  |
| 24 (R/W)           | PF8_SPU    | Strong Pull Up Control for PF8 IO.  |
| 23 (R/W)           | PF7_SPU    | Strong Pull Up Control for PF7 IO.  |
| 22 (R/W)           | PF6_SPU    | Strong Pull Up Control for PF6 IO.  |
| 21 (R/W)           | PF5_SPU    | Strong Pull Up Control for PF5 IO.  |
| 20 (R/W)           | PF4_SPU    | Strong Pull Up Control for PF4 IO.  |
| 19 (R/W)           | PF3_SPU    | Strong Pull Up Control for PF3 IO.  |
| 18 (R/W)           | PF2_SPU    | Strong Pull Up Control for PF2 IO.  |
| 17 (R/W)           | PF1_SPU    | Strong Pull Up Control for PF1 IO.  |
| 16 (R/W)           | PF0_SPU    | Strong Pull Up Control for PF0 IO.  |

Table 13-90: PADS\_PORT\_EF\_SPU Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration             |
|--------------------|------------|-------------------------------------|
| 15 (R/W)           | PE15_SPU   | Strong Pull Up Control for PE15 IO. |
| 14 (R/W)           | PE14_SPU   | Strong Pull Up Control for PE14 IO. |
| 13 (R/W)           | PE13_SPU   | Strong Pull Up Control for PE13 IO. |
| 12 (R/W)           | PE12_SPU   | Strong Pull Up Control for PE12 IO. |
| 11 (R/W)           | PE11_SPU   | Strong Pull Up Control for PE11 IO. |
| 10 (R/W)           | PE10_SPU   | Strong Pull Up Control for PE10 IO. |
| 9 (R/W)            | PE9_SPU    | Strong Pull Up Control for PE9 IO.  |
| 8 (R/W)            | PE8_SPU    | Strong Pull Up Control for PE8 IO.  |
| 7 (R/W)            | PE7_SPU    | Strong Pull Up Control for PE7 IO.  |
| 6 (R/W)            | PE6_SPU    | Strong Pull Up Control for PE6 IO.  |
| 5 (R/W)            | PE5_SPU    | Strong Pull Up Control for PE5 IO.  |
| 4 (R/W)            | PE4_SPU    | Strong Pull Up Control for PE4 IO.  |
| 3 (R/W)            | PE3_SPU    | Strong Pull Up Control for PE3 IO.  |
| 2 (R/W)            | PE2_SPU    | Strong Pull Up Control for PE2 IO.  |
| 1 (R/W)            | PE1_SPU    | Strong Pull Up Control for PE1 IO.  |
| 0 (R/W)            | PE0_SPU    | Strong Pull Up Control for PE0 IO.  |

## PORTG and PORTH Pull Enable Register

Figure 13-86: PADS\_PORT\_GH\_PE Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000088_34a0fd1e4227759dfc9b8a6d0b4dc5dfd89a722047965a15628b8506e0898bf6.png)

Table 13-91: PADS\_PORT\_GH\_PE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31 (R/W)           | PH15_PE    | Pull Enable for PH15 IO.  |
| 30 (R/W)           | PH14_PE    | Pull Enable for PH14 IO.  |
| 29 (R/W)           | PH13_PE    | Pull Enable for PH13 IO.  |
| 28 (R/W)           | PH12_PE    | Pull Enable for PH12 IO.  |
| 27 (R/W)           | PH11_PE    | Pull Enable for PH11 IO.  |
| 26 (R/W)           | PH10_PE    | Pull Enable for PH10 IO.  |
| 25 (R/W)           | PH9_PE     | Pull Enable for PH9 IO.   |
| 24 (R/W)           | PH8_PE     | Pull Enable for PH8 IO.   |
| 23 (R/W)           | PH7_PE     | Pull Enable for PH7 IO.   |
| 22 (R/W)           | PH6_PE     | Pull Enable for PH6 IO.   |
| 21 (R/W)           | PH5_PE     | Pull Enable for PH5 IO.   |
| 20 (R/W)           | PH4_PE     | Pull Enable for PH4 IO.   |
| 19 (R/W)           | PH3_PE     | Pull Enable for PH3 IO.   |
| 18 (R/W)           | PH2_PE     | Pull Enable for PH2 IO.   |
| 17 (R/W)           | PH1_PE     | Pull Enable for PH1 IO.   |
| 16 (R/W)           | PH0_PE     | Pull Enable for PH0 IO.   |

Table 13-91: PADS\_PORT\_GH\_PE Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 15 (R/W)           | PG15_PE    | Pull Enable for PG15 IO.  |
| 14 (R/W)           | PG14_PE    | Pull Enable for PG14 IO.  |
| 13 (R/W)           | PG13_PE    | Pull Enable for PG13 IO.  |
| 12 (R/W)           | PG12_PE    | Pull Enable for PG12 IO.  |
| 11 (R/W)           | PG11_PE    | Pull Enable for PG11 IO.  |
| 10 (R/W)           | PG10_PE    | Pull Enable for PG10 IO.  |
| 9 (R/W)            | PG9_PE     | Pull Enable for PG9 IO.   |
| 8 (R/W)            | PG8_PE     | Pull Enable for PG8 IO.   |
| 7 (R/W)            | PG7_PE     | Pull Enable for PG7 IO.   |
| 6 (R/W)            | PG6_PE     | Pull Enable for PG6 IO.   |
| 5 (R/W)            | PG5_PE     | Pull Enable for PG5 IO.   |
| 4 (R/W)            | PG4_PE     | Pull Enable for PG4 IO.   |
| 3 (R/W)            | PG3_PE     | Pull Enable for PG3 IO.   |
| 2 (R/W)            | PG2_PE     | Pull Enable for PG2 IO.   |
| 1 (R/W)            | PG1_PE     | Pull Enable for PG1 IO.   |
| 0 (R/W)            | PG0_PE     | Pull Enable for PG0 IO.   |

## PORTG and PORTH Pull Selector Register

Figure 13-87: PADS\_PORT\_GH\_PS Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000089_2adbe0624ad4b91ed24e9aa6fc64a57b2a4984bfcf694fd08b916f3a4bb77ac9.png)

Table 13-92: PADS\_PORT\_GH\_PS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration    |
|--------------------|------------|----------------------------|
| 31 (R/W)           | PH15_PS    | Pull Selector for PH15 IO. |
| 30 (R/W)           | PH14_PS    | Pull Selector for PH14 IO. |
| 29 (R/W)           | PH13_PS    | Pull Selector for PH13 IO. |
| 28 (R/W)           | PH12_PS    | Pull Selector for PH12 IO. |
| 27 (R/W)           | PH11_PS    | Pull Selector for PH11 IO. |
| 26 (R/W)           | PH10_PS    | Pull Selector for PH10 IO. |
| 25 (R/W)           | PH9_PS     | Pull Selector for PH9 IO.  |
| 24 (R/W)           | PH8_PS     | Pull Selector for PH8 IO.  |
| 23 (R/W)           | PH7_PS     | Pull Selector for PH7 IO.  |
| 22 (R/W)           | PH6_PS     | Pull Selector for PH6 IO.  |
| 21 (R/W)           | PH5_PS     | Pull Selector for PH5 IO.  |
| 20 (R/W)           | PH4_PS     | Pull Selector for PH4 IO.  |
| 19 (R/W)           | PH3_PS     | Pull Selector for PH3 IO.  |
| 18 (R/W)           | PH2_PS     | Pull Selector for PH2 IO.  |
| 17 (R/W)           | PH1_PS     | Pull Selector for PH1 IO.  |
| 16 (R/W)           | PH0_PS     | Pull Selector for PH0 IO.  |

Table 13-92: PADS\_PORT\_GH\_PS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration    |
|--------------------|------------|----------------------------|
| 15 (R/W)           | PG15_PS    | Pull Selector for PG15 IO. |
| 14 (R/W)           | PG14_PS    | Pull Selector for PG14 IO. |
| 13 (R/W)           | PG13_PS    | Pull Selector for PG13 IO. |
| 12 (R/W)           | PG12_PS    | Pull Selector for PG12 IO. |
| 11 (R/W)           | PG11_PS    | Pull Selector for PG11 IO. |
| 10 (R/W)           | PG10_PS    | Pull Selector for PG10 IO. |
| 9 (R/W)            | PG9_PS     | Pull Selector for PG9 IO.  |
| 8 (R/W)            | PG8_PS     | Pull Selector for PG8 IO.  |
| 7 (R/W)            | PG7_PS     | Pull Selector for PG7 IO.  |
| 6 (R/W)            | PG6_PS     | Pull Selector for PG6 IO.  |
| 5 (R/W)            | PG5_PS     | Pull Selector for PG5 IO.  |
| 4 (R/W)            | PG4_PS     | Pull Selector for PG4 IO.  |
| 3 (R/W)            | PG3_PS     | Pull Selector for PG3 IO.  |
| 2 (R/W)            | PG2_PS     | Pull Selector for PG2 IO.  |
| 1 (R/W)            | PG1_PS     | Pull Selector for PG1 IO.  |
| 0 (R/W)            | PG0_PS     | Pull Selector for PG0 IO.  |

## PORTG and PORTH Slew Rate Control Register

Figure 13-88: PADS\_PORT\_GH\_SL Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000090_3cc76c62bb65dffe4f5589ffcefec9b9b3e9d1b78259d81f3d1e4625f48ba2fb.png)

Table 13-93: PADS\_PORT\_GH\_SL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration        |
|--------------------|------------|--------------------------------|
| 31 (R/W)           | PH15_SL    | Slew Rate Control for PH15 IO. |
| 30 (R/W)           | PH14_SL    | Slew Rate Control for PH14 IO. |
| 29 (R/W)           | PH13_SL    | Slew Rate Control for PH13 IO. |
| 28 (R/W)           | PH12_SL    | Slew Rate Control for PH12 IO. |
| 27 (R/W)           | PH11_SL    | Slew Rate Control for PH11 IO. |
| 26 (R/W)           | PH10_SL    | Slew Rate Control for PH10 IO. |
| 25 (R/W)           | PH9_SL     | Slew Rate Control for PH9 IO.  |
| 24 (R/W)           | PH8_SL     | Slew Rate Control for PH8 IO.  |
| 23 (R/W)           | PH7_SL     | Slew Rate Control for PH7 IO.  |
| 22 (R/W)           | PH6_SL     | Slew Rate Control for PH6 IO.  |
| 21 (R/W)           | PH5_SL     | Slew Rate Control for PH5 IO.  |
| 20 (R/W)           | PH4_SL     | Slew Rate Control for PH4 IO.  |
| 19 (R/W)           | PH3_SL     | Slew Rate Control for PH3 IO.  |
| 18 (R/W)           | PH2_SL     | Slew Rate Control for PH2 IO.  |
| 17 (R/W)           | PH1_SL     | Slew Rate Control for PH1 IO.  |
| 16 (R/W)           | PH0_SL     | Slew Rate Control for PH0 IO.  |

Table 13-93: PADS\_PORT\_GH\_SL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration        |
|--------------------|------------|--------------------------------|
| 15 (R/W)           | PG15_SL    | Slew Rate Control for PG15 IO. |
| 14 (R/W)           | PG14_SL    | Slew Rate Control for PG14 IO. |
| 13 (R/W)           | PG13_SL    | Slew Rate Control for PG13 IO. |
| 12 (R/W)           | PG12_SL    | Slew Rate Control for PG12 IO. |
| 11 (R/W)           | PG11_SL    | Slew Rate Control for PG11 IO. |
| 10 (R/W)           | PG10_SL    | Slew Rate Control for PG10 IO. |
| 9 (R/W)            | PG9_SL     | Slew Rate Control for PG9 IO.  |
| 8 (R/W)            | PG8_SL     | Slew Rate Control for PG8 IO.  |
| 7 (R/W)            | PG7_SL     | Slew Rate Control for PG7 IO.  |
| 6 (R/W)            | PG6_SL     | Slew Rate Control for PG6 IO.  |
| 5 (R/W)            | PG5_SL     | Slew Rate Control for PG5 IO.  |
| 4 (R/W)            | PG4_SL     | Slew Rate Control for PG4 IO.  |
| 3 (R/W)            | PG3_SL     | Slew Rate Control for PG3 IO.  |
| 2 (R/W)            | PG2_SL     | Slew Rate Control for PG2 IO.  |
| 1 (R/W)            | PG1_SL     | Slew Rate Control for PG1 IO.  |
| 0 (R/W)            | PG0_SL     | Slew Rate Control for PG0 IO.  |

## PORTG and PORTH Strong Pull Up Control Register

Figure 13-89: PADS\_PORT\_GH\_SPU Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000091_4e0a159d02741e213ef8089a3e2f69063f24ff194d5f0467ae505ce093c46bc4.png)

Table 13-94: PADS\_PORT\_GH\_SPU Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration             |
|--------------------|------------|-------------------------------------|
| 31 (R/W)           | PH15_SPU   | Strong Pull Up Control for PH15 IO. |
| 30 (R/W)           | PH14_SPU   | Strong Pull Up Control for PH14 IO. |
| 29 (R/W)           | PH13_SPU   | Strong Pull Up Control for PH13 IO. |
| 28 (R/W)           | PH12_SPU   | Strong Pull Up Control for PH12 IO. |
| 27 (R/W)           | PH11_SPU   | Strong Pull Up Control for PH11 IO. |
| 26 (R/W)           | PH10_SPU   | Strong Pull Up Control for PH10 IO. |
| 25 (R/W)           | PH9_SPU    | Strong Pull Up Control for PH9 IO.  |
| 24 (R/W)           | PH8_SPU    | Strong Pull Up Control for PH8 IO.  |
| 23 (R/W)           | PH7_SPU    | Strong Pull Up Control for PH7 IO.  |
| 22 (R/W)           | PH6_SPU    | Strong Pull Up Control for PH6 IO.  |
| 21 (R/W)           | PH5_SPU    | Strong Pull Up Control for PH5 IO.  |
| 20 (R/W)           | PH4_SPU    | Strong Pull Up Control for PH4 IO.  |
| 19 (R/W)           | PH3_SPU    | Strong Pull Up Control for PH3 IO.  |
| 18 (R/W)           | PH2_SPU    | Strong Pull Up Control for PH2 IO.  |
| 17 (R/W)           | PH1_SPU    | Strong Pull Up Control for PH1 IO.  |
| 16 (R/W)           | PH0_SPU    | Strong Pull Up Control for PH0 IO.  |

Table 13-94: PADS\_PORT\_GH\_SPU Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration             |
|--------------------|------------|-------------------------------------|
| 15 (R/W)           | PG15_SPU   | Strong Pull Up Control for PG15 IO. |
| 14 (R/W)           | PG14_SPU   | Strong Pull Up Control for PG14 IO. |
| 13 (R/W)           | PG13_SPU   | Strong Pull Up Control for PG13 IO. |
| 12 (R/W)           | PG12_SPU   | Strong Pull Up Control for PG12 IO. |
| 11 (R/W)           | PG11_SPU   | Strong Pull Up Control for PG11 IO. |
| 10 (R/W)           | PG10_SPU   | Strong Pull Up Control for PG10 IO. |
| 9 (R/W)            | PG9_SPU    | Strong Pull Up Control for PG9 IO.  |
| 8 (R/W)            | PG8_SPU    | Strong Pull Up Control for PG8 IO.  |
| 7 (R/W)            | PG7_SPU    | Strong Pull Up Control for PG7 IO.  |
| 6 (R/W)            | PG6_SPU    | Strong Pull Up Control for PG6 IO.  |
| 5 (R/W)            | PG5_SPU    | Strong Pull Up Control for PG5 IO.  |
| 4 (R/W)            | PG4_SPU    | Strong Pull Up Control for PG4 IO.  |
| 3 (R/W)            | PG3_SPU    | Strong Pull Up Control for PG3 IO.  |
| 2 (R/W)            | PG2_SPU    | Strong Pull Up Control for PG2 IO.  |
| 1 (R/W)            | PG1_SPU    | Strong Pull Up Control for PG1 IO.  |
| 0 (R/W)            | PG0_SPU    | Strong Pull Up Control for PG0 IO.  |

## Status Register

Figure 13-90: PADS\_STAT Register Diagram

![Image](16_General-Purpose_Ports_(PORT)_artifacts/image_000092_7e7f976071108d530a83e692f0e7ee92450598144fca97976ae81176002ab108.png)

Table 13-95: PADS\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 0                  | RPC        | State of RPC pad.         |
| (R/NW)             |            |                           |