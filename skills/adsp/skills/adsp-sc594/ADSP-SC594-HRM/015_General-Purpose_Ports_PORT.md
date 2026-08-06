# General-Purpose Ports (PORT)

<!-- source: 015_General-Purpose_Ports_PORT.pdf | original pages 512–674 -->

## 13   General-Purpose Ports (PORT)

This section describes general-purpose ports, pin multiplexing, general-purpose input/output (GPIO) functionality, and pin interrupts. The general-purpose ports provide the following three functions:

- Pin multiplexing scheme
- GPIO functionality
- Pin interrupt requests

NOTE: In this chapter, the naming convention for registers and bits omits the alphabetic group enumeration to refer to any and all of the ports. For example, PORT\_FER represents registers PORTA\_FER , PORTB\_FER , and so on. Likewise PORT\_FER.PX1 represents bits PA1, PB1, and so on.

Figure 13-1: Simplified GPIO and Pin Interrupt Signal Flow

<!-- image -->

## PORT Features

The PORTs include the following features:

- Input mode, output mode, and open-drain mode of GPIO operation
- Port multiplexing controlled on a pin-by-pin basis
- No external glue hardware required for unused pins
- All port pins provide interrupt request functionality
- Byte-wide pin-to-interrupt request assignment

## PORT Functional Description

The number of ports and each's composition are defined in the processor data sheet. Each port has a dedicated set of MMR registers that control pin functions and operates in general-purpose I/O (GPIO) mode by default, as controlled by the port-specific PORT\_FER register. Each bit in this register, as well as the other PORT MMRs, represents a specific GPIO pin on the specified port.

## Input Mode, Output Mode, and Open-Drain Mode of GPIO Operation

At reset, every GPIO pin defaults to input mode with the input drivers disabled. To enable any GPIO input driver, set the bits corresponding to the individual pins in the appropriate input enable register ( PORT\_INEN ).

The GPIO output drivers are enabled by setting the corresponding bits in the direction registers ( PORT\_DIR ).

The PORT can use every GPIO in open-drain mode by clearing the respective bit in the PORT\_DATA register or setting the respective bit in the PORT\_DATA\_CLR register. Then, set the corresponding bit in the PORT\_INEN register. Read from the PORT\_DATA register to obtain the status from the pin.

## Port Multiplexing Controlled on Pin-by-Pin Basis

Each port has two dedicated MMRs that control the port multiplexing, the 16-bit function enable ( PORT\_FER ) registers and the 32-bit port multiplexing ( PORT\_MUX ) registers.

## All Port Pins Provide Interrupt Functionality

Pin interrupts are completely decoupled from GPIO functionality. Pins are connected to the system event controller (SEC) via the PINTx modules, each of which is configurable in terms of which port pins are sensed for interrupt generation.

## ADSP-2159x\_SC592\_SC594 PORT Register List

The PORT module (PORT) regulates the use of the multiplexable processor pins. Every port pin can operate in general-purpose I/O (GPIO) mode or as an alternate function. This GPIO operation is the default after processor reset and is controlled by a set of registers that control GPIO functionality. Every bit in these registers represents a certain GPIO pin of a specific port. For more information on PORT functionality, see the PORT register descriptions.

Table 13-1: ADSP-2159x\_SC592\_SC594 PORT Register List

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

## ADSP-2159x\_SC592\_SC594 PORT Trigger List

Table 13-2: ADSP-2159x\_SC592\_SC594 PORT Trigger List Generators

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         | None   | None          | None          |

Table 13-3: ADSP-2159x\_SC592\_SC594 PORT Trigger List Receivers

|   Trigger ID | Name         | Description         | Sensitivity   |
|--------------|--------------|---------------------|---------------|
|           41 | PORTA_TOGGLE | Port Toggle Trigger | Pulse         |
|           42 | PORTB_TOGGLE | Port Toggle Trigger | Pulse         |
|           43 | PORTC_TOGGLE | Port Toggle Trigger | Pulse         |
|           44 | PORTD_TOGGLE | Port Toggle Trigger | Pulse         |
|           45 | PORTE_TOGGLE | Port Toggle Trigger | Pulse         |
|           46 | PORTF_TOGGLE | Port Toggle Trigger | Pulse         |
|           47 | PORTG_TOGGLE | Port Toggle Trigger | Pulse         |
|           48 | PORTH_TOGGLE | Port Toggle Trigger | Pulse         |
|           49 | PORTI_TOGGLE | Port Toggle Trigger | Pulse         |

## ADSP-2159x\_SC592\_SC594 PINT Register List

The Pin Interrupt module (PINT) controls the pin-to-interrupt assignment in a byte-wide manner. The pin-interrupt assignment registers do not consist of 32 individual bits. They consist of four control bytes, each functioning as a multiplexer control. For more information, see the PINT register descriptions.

All PINT registers are 32 bits wide and can be accessed by 32-bit load/store instructions. They also support 16-bit operation where the upper 16 bits are ignored and the application uses the lower 16 bits only. Consequently, all PINT registers support 32-bit accesses as well as 16-bit accesses for the lower half words. Applications may use faster 16-bit accesses as long as they do not require functionality of upper register halves.

Table 13-4: ADSP-2159x\_SC592\_SC594 PINT Register List

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

## ADSP-2159x\_SC592\_SC594 PINT Interrupt List

Table 13-5: ADSP-2159x\_SC592\_SC594 PINT Interrupt List

|   Interrupt ID | Name        | Description                 | Sensitivity   | DMA Channel   |
|----------------|-------------|-----------------------------|---------------|---------------|
|             65 | PINT0_BLOCK | PINT0 Pin Interrupt Block 0 | Level         |               |
|             66 | PINT1_BLOCK | PINT1 Pin Interrupt Block 1 | Level         |               |
|             67 | PINT2_BLOCK | PINT2 Pin Interrupt Block 2 | Level         |               |
|             68 | PINT3_BLOCK | PINT3 Pin Interrupt Block 3 | Level         |               |
|             69 | PINT4_BLOCK | PINT4 Pin Interrupt Block 4 | Level         |               |
|             70 | PINT5_BLOCK | PINT5 Pin Interrupt Block 5 | Level         |               |
|             71 | PINT6_BLOCK | PINT6 Pin Interrupt Block 6 | Level         |               |
|             72 | PINT7_BLOCK | PINT7 Pin Interrupt Block 7 | Level         |               |

## ADSP-2159x\_SC592\_SC594 PINT Trigger List

Table 13-6: ADSP-2159x\_SC592\_SC594 PINT Trigger List Generators

|   Trigger ID | Name        | Description               | Sensitivity   |
|--------------|-------------|---------------------------|---------------|
|           73 | PINT0_BLOCK | PINT0 Pin Interrupt Block | Level         |
|           74 | PINT1_BLOCK | PINT1 Pin Interrupt Block | Level         |
|           75 | PINT2_BLOCK | PINT2 Pin Interrupt Block | Level         |
|           76 | PINT3_BLOCK | PINT3 Pin Interrupt Block | Level         |
|           77 | PINT4_BLOCK | PINT4 Pin Interrupt Block | Level         |
|           78 | PINT5_BLOCK | PINT5 Pin Interrupt Block | Level         |
|           79 | PINT6_BLOCK | PINT6 Pin Interrupt Block | Level         |
|           80 | PINT7_BLOCK | PINT7 Pin Interrupt Block | Level         |

Table 13-7: ADSP-2159x\_SC592\_SC594 PINT Trigger List Receivers

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         | None   | None          | None          |

## ADSP-2159x\_SC592\_SC594 PADS Register List

The PADS controls signal hysteresis and other system interface signal features for a number of module interfaces.

Table 13-8: ADSP-2159x\_SC592\_SC594 PADS Register List

| Name           | Description                  |
|----------------|------------------------------|
| PADS_DAI0_0_DS | DAI0 1 to 10 pins DS control |

Table 13-8: ADSP-2159x\_SC592\_SC594 PADS Register List (Continued)

| Name              | Description                             |
|-------------------|-----------------------------------------|
| PADS_DAI0_1_DS    | DAI0 11 to 20 pins DS control           |
| PADS_DAI0_IE      | DAI0 Port Input Enable Control Register |
| PADS_DAI1_0_DS    | DAI1 1 to 10 pins DS control            |
| PADS_DAI1_1_DS    | DAI1 11 to 20 pins DS control           |
| PADS_DAI1_IE      | DAI1 Port Input Enable Control Register |
| PADS_DAI[n]_PDE   | DAI[n] Pull-Down Enable                 |
| PADS_DAI[n]_PUE   | DAI[n] Pull-Up Enable                   |
| PADS_NONPORTS_CTL | Non-GPIO control Register               |
| PADS_NONPORTS_DS  | Non-GPIO Drive Strength Register        |
| PADS_PCFG0        | Peripheral PAD Configuration0 Register  |
| PADS_PCFG1        | Peripheral Configuration1 Register      |
| PADS_PORTA0_DS    | PORTA 0 to 7 pins DS control            |
| PADS_PORTA1_DS    | PORTA 8 - 15 pins DS control            |
| PADS_PORTB0_DS    | PORTB 0 to 7 pins DS control            |
| PADS_PORTB1_DS    | PORTB 8 - 15 pins DS control            |
| PADS_PORTC0_DS    | PORTC 0 to 7 pins DS control            |
| PADS_PORTC1_DS    | PORTC 8 - 15 pins DS control            |
| PADS_PORTD0_DS    | PORTD 0 to 7 pins DS control            |
| PADS_PORTD1_DS    | PORTD 8 to 15 pins DS control           |
| PADS_PORTE0_DS    | PORTE 0 to 7 pins DS control            |
| PADS_PORTE1_DS    | PORTE 8 to 15 pins DS control           |
| PADS_PORTF0_DS    | PORTF 0 to 7 pins DS control            |
| PADS_PORTF1_DS    | PORTF 8 to 15 pins DS control           |
| PADS_PORTG0_DS    | PORTG 0 to 7 pins DS control            |
| PADS_PORTG1_DS    | PORTG 8 to 15 pins DS control           |
| PADS_PORTH0_DS    | PORTH 0 to 7 pins DS control            |
| PADS_PORTH1_DS    | PORTH 8 to 15 pins DS control           |
| PADS_PORTI0_DS    | PORTI 0 to 7 pins DS control            |
| PADS_PORT[n]_PDE  | PORT[n] Pull-Down Enable                |
| PADS_PORT[n]_PUE  | PORT[n] Pull-Up Enable                  |

## PORT Architectural Concepts

These sections describe in more detail how the PORT module connects externally to pins and internally to the MMR bus. Ports are named alphabetically beginning with A.

- Internal Interfaces
- External Interfaces
- GPIO Pin Function
- Port Multiplexing Control

## Internal Interfaces

All of the pin multiplexing, GPIO, and pin interrupt control block MMRs can be accessed through the MMR bus. There is no DMA support. Each of the pin interrupt (PINTx) modules has its own dedicated interrupt request output signal that connects directly to the system event controller (SEC).

## External Interfaces

The pin multiplexing hardware can be seen as a layer between the on-chip peripherals and the silicon pads connecting to the physical pins/balls or the package, as controlled by the PORT unit.

## GPIO Pin Function

By default, the PORT sets every GPIO pin to input mode. The input drivers are not enabled, which avoids the need for unnecessary current sinks and external termination resistors on unused pins.

## Input Mode

The default mode of every GPIO pin after reset is input mode, but the input drivers are not enabled. To enable GPIO input drivers, set the bits corresponding to the PORT pins in the appropriate input enable register ( PORT\_INEN ). When enabled, a read from the PORT\_DATA register returns the logical state of the input pins. However, the input signal does not overwrite the state of the internal flip-flop used for providing output to the same pin. Only software can alter the state. If the input driver is enabled, a write to the PORT\_DATA register can alter the state of the flip-flop, but the change cannot be read back.

## Output Mode

Any GPIO pin can be configured for output mode. The GPIO output drivers are enabled by setting the bits corresponding to the PORT pins in the appropriate direction register. The PORT implements direction registers as a pair of write-1-to-set (W1S) and write-1-to-clear (W1C) MMRs called PORT\_DIR\_SET and PORT\_DIR\_CLR , respectively. As such, software can alter the direction of the signal flow on individual GPIO pins without impacting other GPIOs on the same port.

Both the PORT\_DIR\_SET and PORT\_DIR\_CLR registers return the same value when read, and a logical 1 indicates an enabled output. The PORT\_DATA registers control the state of output pins. A logical 0 drives the output low while a logical 1 drives the output high.

While writes to the PORT\_DATA register can alter all of the GPIOs on a specific port at once, there are also a pair of W1S and W1C MMRs called PORT\_DATA\_SET and PORT\_DATA\_CLR , respectively. These registers enable

the manipulation of individual GPIO outputs. The state of the outputs can be obtained by reading the PORT\_DATA registers. Because the state of the GPIO output can be controlled before the output driver is enabled, set or clear the internal flip-flop first by programming this register to avoid volatile levels on the output pin.

## Trigger Toggle Mode

Any GPIO pin that has been configured for output mode can be toggled using a trigger input from the Trigger Routing Unit (TRU). To enable this functionality for a particular GPIO, set the appropriate bit in the PORT\_TRIG\_TGL register. Any subsequent trigger for the designated port causes all GPIO outputs set in the PORT\_TRIG\_TGL register to toggle.

To avoid unpredictable behavior, do not set, clear, or toggle the PORT\_DATA , PORT\_DATA\_SET , PORT\_DATA\_CLR , or PORT\_DATA\_TGL registers when the GPIO output has the corresponding PORT\_TRIG\_TGL bit set. T o change the state of the GPIO output using one of these registers, first clear the corresponding bit in the PORT\_TRIG\_TGL register.

## Open-Drain Mode

Every GPIO can also be used in open-drain mode. First, either clear the respective bit in the PORT\_DATA register or set the respective bit in the PORT\_DATA\_CLR register. Then, set the appropriate bit in the PORT\_INEN register. Read from the PORT\_DATA register to return the status from the pin rather than the state of the internal flipflop.

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

- PINT0 can sense pin activity on PORTA and PORTB
- PINT1 can sense pin activity on PORTB and PORTC
- PINT2 can sense pin activity on PORTC and PORTD
- PINT3 can sense pin activity on PORTD and PORTE
- PINT4 can sense pin activity on PORTE and PORTF
- PINT5 can sense pin activity on PORTF and PORTG
- PINT6 can sense pin activity on PORTG and PORTH
- PINT7 can sense pin activity on PORTH and PORTI

The processor supports both 32-bit and 16-bit peripheral bus accesses to PINTx registers.

Figure 13-2: GPIO to PINTx Assignment

<!-- image -->

Pins connect to the PINTx module and then to the system event controller (SEC), as shown in the PINTx Block Diagram .

Figure 13-3: PINTx Block Diagram

<!-- image -->

As shown in the PINTx Block Diagram , each port is subdivided into two 8-pin half ports, upper ( PxH ) and lower ( PxL ). The PINT\_ASSIGN registers control the 8-bit multiplexers associated with these half ports, where the lower half units (eight pins) can be forwarded to either byte 0 or byte 2 of the PINTx blocks, and the upper half units (eight pins) can be forwarded to either byte 1 or byte 3 of the PINTx blocks.

When a half port is assigned to a byte in any PINTx block, the state of the eight pins appears in the PINT\_PINSTATE register, regardless of whether the pin is enabled for GPIO or peripheral functions (input or output). When neither the input nor output drivers of the pin are enabled, the pin state is read as zero. The PINT\_PINSTATE register reports the inverted state of the pin when the PINT\_INV\_SET register activates the signal inverter. The inverter can be enabled on an individual bit-by-bit basis. Each bit in the PINT\_INV\_SET / PINT\_INV\_CLR register pair represents a pin signal.

By default, PORT interrupt request generation is level-sensitive, and an interrupt request occurs when the enabled pin is sensed as active high. Use the PINT\_EDGE\_SET register to change the interrupt request genearion scheme to instead be edge-sensitive (rising edge generates the interrupt request). Use the PINT\_INV\_SET register to invert the polarity such that the PINTx block generates the interrupt request on active-low signals or falling edges.

The PINTx modules also assist when both signal edges must generate unique interrupt requests. If two different interrupt requests are required, the PINT\_ASSIGN registers can route a single input signal to two different PINTx blocks, where one block inverts the signal in the PINT\_INV\_SET register and the other one does not. In this fashion, a unique software routine is associated with the hardware PINTx block that is generating the unique interrupt request for each signal edge. When both signal edges can be serviced by the same interrupt request, each half port can be routed to two separate bytes within a single PINTx block using the PINT\_ASSIGN register, and then one of the half ports needs to have the inversion enabled in the PINT\_INV\_SET register. The servicing software routine can then detect from the PINT\_LATCH register whether a falling, rising, or both edges have occurred.

Regardless of whether level-sensitive or edge-sensitive mode is used, the hardware always latches an interrupt request. Latched signals can be read from the PINT\_LATCH registers. Only a software or hardware reset clears the latches. To clear the latch, a W1C operation must be performed to the PINT\_REQ or PINT\_LATCH register. When in level-sensitive mode, the interrupt request remains asserted if the pin state does not change by the time the interrupt service routine exits.

Because every PINTx block groups up to 32 pin signals, the PINT\_MSK\_SET / PINT\_MSK\_CLR register pair can control which of the signals can request an interrupt at the system level. Software can interrogate the PINT\_REQ register for signaling pins. The PINT\_REQ bits represent a logical AND between the mask and the latch. When any of these bits is set, the block output interrupt request is asserted.

## PORT Programming Model

The GPIO Programming Model Flow (Part 1) , GPIO Programming Model Flow (Part 2) , and GPIO Programming Model Flow (Part 3) figures show the programming model for the general-purpose ports. This programming includes the GPIO input and output operation, open-drain mode, and the pin interrupt PINTx modules.

- NOTE: These process flow diagrams connect where call-out letters appear. For example, call-out A in the GPIO Programming Model Flow (Part 1) diagram connects to call-out A in the GPIO Programming Model Flow (Part 2) diagram.

The following flowcharts describe the processes for setting up pins for various functions. Begin the process from the start label in the GPIO Programming Model Flow (Part 1) figure. The first decision (GPIO or peripheral) determines how to program the PORT\_FER register. Set the bit(s) corresponding to the pin(s) to 1 to enable peripheral functionality or to 0 to enable GPIO mode. For more information on setting up for peripheral functions, refer to Port Multiplexing Control.

If the pin is to be a GPIO pin, a subsequent series of decisions must be made that will impact how the PORT\_DATA , PORT\_POL , PORT\_DIR , and PORT\_INEN configuration registers must be programmed. Depending on the type of GPIO pin desired, some configurations do not apply and can have different meanings. The following paragraphs briefly describe the function of the different settings for each of the pin functions in the input, output, and open-drain GPIO modes. It is a best practice to use the SET or CLR versions of the PORT registers, where applicable, to effect changes on a pin-by-pin basis rather than on the full port.

For output mode, first clear the PORT\_DATA register to set all the pins low. Then write the PORT\_DIR register to define the direction of each pin (set the bits associated with the desired output pins to 1). In output mode, the other registers are not significant. The GPIO Programming Model Flow (Part 1) chart shows this flow starting at label 2.

Figure 13-4: GPIO Programming Model Flow (Part 1)

<!-- image -->

For input mode, first decide the polarity for each pin using the PORT\_POL register. Program the PORT\_DIR register to define the appropriate pins as inputs (write a 0 to the bit location associated with the pin). If interrupt requests are desired, configure the PINT module as shown in the GPIO Programming Model Flow (Part 3) figure starting at label B. Finally, write the PORT\_INEN register to enable the associated input drivers. The GPIO Programming Model Flow (Part 2) chart shows this entire flow starting at label 3.

For open-drain mode, set all pins low by clearing the PORT\_DATA register. Then, use the PORT\_INEN register to enable the appropriate input drivers. Set the PORT\_DIR register in this mode to indicate whether the pin is in an active state or not (active being 0). The GPIO Programming Model Flow (Part 2) chart shows this flow starting at label 4.

Figure 13-5: GPIO Programming Model Flow (Part 2)

<!-- image -->

Figure 13-6: GPIO Programming Model Flow (Part 3)

<!-- image -->

## Programmable Pull-up Resistors for PORT and DAI

There are programmable weak pull-up resistors for DAI and PORT pins. These resistors can be enabled or disabled by configuring the PADS\_DAI[n]\_PUE , PADS\_PORT[n]\_PUE , PADS\_DAI[n]\_PDE , and PADS\_PORT[n]\_PDE registers.

For DAI pins, if both the PADS\_DAI[n]\_PDE and PADS\_DAI[n]\_PUE registers are set (=1) for the same pin, then the PADS\_DAI[n]\_PDE register takes priority over PADS\_DAI[n]\_PUE register.

NOTE: For unused PORT/DAI pins, ensure that any unused pin does not go to floating state. Clear (=0) the PUD bit of that pin.

Refer to the DAI Pull-up Resistor States table. T o enable the pull-up on a DAI pin, the

PADS\_DAI[n]\_PDE.PDE bit must be cleared (=0) AND the PADS\_DAI[n]\_PUE.PUE bit must be set (=1). To disable the pull-up on a DAI pin, the PADS\_DAI[n]\_PDE.PDE bit must be set (=1) and PADS\_DAI[n]\_PUE.PUE bit can be any value.

Table 13-9: DAI Pull-up Resistor States

|   PADS_DAI[n]_PUE.PUE |   PADS_DAI[n]_PDE.PDE | Pull-up Resistor State   |
|-----------------------|-----------------------|--------------------------|
|                     0 |                     0 | Disabled                 |
|                     0 |                     1 | Disabled                 |
|                     1 |                     0 | Enabled                  |
|                     1 |                     1 | Disabled                 |

Similarly, for PORT pins, if both PADS\_PORT[n]\_PDE.PDE and PADS\_PORT[n]\_PUE registers are set (=1) for the same pin, then the PADS\_PORT[n]\_PDE.PDE register takes priority over PADS\_PORT[n]\_PUE register.

Refer to the PORT Pull-up Resistor States table. T o enable the pull-up on a PORT pin, the PADS\_PORT[n]\_PDE.PDE bit must be cleared (=0) AND the PADS\_PORT[n]\_PUE.PUE bit must be set (=1). To disable the pull-up on a PORT pin, PADS\_DAI[n]\_PDE.PDE bit must be set (=1) and PADS\_PORT[n]\_PUE.PUE bit can be any value.

Table 13-10: PORT Pull-up Resistor States

|   PADS_PORT[n]_PUE.PUE |   PADS_PORT[n]_PDE.PDE | Pull-up Resistor State   |
|------------------------|------------------------|--------------------------|
|                      0 |                      0 | Disabled                 |
|                      0 |                      1 | Disabled                 |
|                      1 |                      0 | Enabled                  |
|                      1 |                      1 | Disabled                 |

NOTE: Refer to the Designer Quick Reference section of the data sheet for handling unused PORT and DAI pins.

NOTE: The internal pull-up on GPIO pins are very weak. These pins hold a defined logic level when pins are left floating. The internal pull-up resistance varies over a wide range. If an application needs a specified pull-up on the pin, use an external resistor.

NOTE: For unused PORT/DAI pins, ensure that any unused pin does not go to floating state. Clear (=0) the PUD bit of that pin.

## ADSP-2159x\_SC592\_SC594 PORT Register Descriptions

The General-Purpose Input/Output Port (PORT) contains the following registers.

Table 13-11: ADSP-2159x\_SC592\_SC594 PORT Register List

| Name          | Description                     |
|---------------|---------------------------------|
| PORT_DATA     | Port x GPIO Data Register       |
| PORT_DATA_CLR | Port x GPIO Data Clear Register |

Table 13-11: ADSP-2159x\_SC592\_SC594 PORT Register List (Continued)

| Name          | Description                                |
|---------------|--------------------------------------------|
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

<!-- image -->

Table 13-12: PORT\_DATA Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | PX15       | Port x Bit 15 Data. The PORT_DATA.PX15 bit indicates a signal on a GPIO pin.                            |
| 14 (R/W)           | PX14       | Port x Bit 14 Data. The PORT_DATA.PX14 bit indicates a signal on a GPIO pin. 0 Signal Low               |
| 13 (R/W)           | PX13       | Port x Bit 13 Data. The PORT_DATA.PX13 bit indicates a signal on a GPIO pin. 0 Signal Low               |
| 12 (R/W)           | PX12       | 1 Signal High Port x Bit 12 Data. The PORT_DATA.PX12 bit indicates a signal on a GPIO pin. 0 Signal Low |
| 11 (R/W)           | PX11       | 1 Signal High Port x Bit 11 Data. The PORT_DATA.PX11 bit indicates a signal on a GPIO pin.              |
| 10 (R/W)           | PX10       | 0 Signal Low 1 Signal High                                                                              |
|                    |            | Port x Bit 10 Data. The PORT_DATA.PX10 bit indicates a signal on a GPIO pin. 0 Signal Low 1 Signal High |
| 9 (R/W)            | PX9        | Port x Bit 9 Data. The PORT_DATA.PX9 bit indicates a signal on a GPIO pin. 0 Signal Low                 |

Table 13-12: PORT\_DATA Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | PX8        | Port x Bit 8 Data. The PORT_DATA.PX8 bit indicates a signal on a GPIO pin.                            |
| 7 (R/W)            | PX7        | Port x Bit 7 Data. The PORT_DATA.PX7 bit indicates a signal on a GPIO pin. 0 Signal Low               |
| 6 (R/W)            | PX6        | Port x Bit 6 Data. The PORT_DATA.PX6 bit indicates a signal on a GPIO pin. 0 Signal Low               |
| 5 (R/W)            | PX5        | Port x Bit 5 Data. The PORT_DATA.PX5 bit indicates a signal on a GPIO pin. 0 Signal Low               |
| 4 (R/W)            | PX4        | 1 Signal High                                                                                         |
|                    |            | Port x Bit 4 Data. The PORT_DATA.PX4 bit indicates a signal on a GPIO pin. 0 Signal Low 1 Signal High |
| 3 (R/W)            | PX3        | Port x Bit 3 Data. The PORT_DATA.PX3 bit indicates a signal on a GPIO pin. 0 Signal Low 1 Signal High |
| 2 (R/W)            | PX2        | Port x Bit 2 Data. The PORT_DATA.PX2 bit indicates a signal on a GPIO pin. 0 Signal Low               |

Table 13-12: PORT\_DATA Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | PX1        | Port x Bit 1 Data. The PORT_DATA.PX1 bit indicates a signal on a GPIO pin. 0 Signal Low               |
| 0 (R/W)            | PX0        | Port x Bit 0 Data. The PORT_DATA.PX0 bit indicates a signal on a GPIO pin. 0 Signal Low 1 Signal High |

## Port x GPIO Data Clear Register

The PORT\_DATA\_CLR register operates differently for port bits/pins, depending on whether the bit/pin is output mode or input mode. For more information, see the PORT\_DATA register description.

Figure 13-8: PORT\_DATA\_CLR Register Diagram

<!-- image -->

Table 13-13: PORT\_DATA\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                       | Description/Enumeration                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------|
| 15 (R/W1C)         | PX15       | Port x Bit 15 Data Clear. The PORT_DATA_CLR.PX15 bit clears the pin without impacting other pins of the port. | Port x Bit 15 Data Clear. The PORT_DATA_CLR.PX15 bit clears the pin without impacting other pins of the port. |
| 15 (R/W1C)         | PX15       | 0                                                                                                             | No Effect                                                                                                     |
| 15 (R/W1C)         | PX15       | 1                                                                                                             | Clear Bit. Write 1 for signal low in output mode.                                                             |
| 14 (R/W1C)         | PX14       | Port x Bit 14 Data Clear. The PORT_DATA_CLR.PX14 bit clears the pin without impacting other pins of the port. | Port x Bit 14 Data Clear. The PORT_DATA_CLR.PX14 bit clears the pin without impacting other pins of the port. |
| 14 (R/W1C)         | PX14       | 0                                                                                                             | No Effect. Write 0 has no effect in output mode.                                                              |
| 14 (R/W1C)         | PX14       | 1                                                                                                             | Clear Bit. Write 1 for signal low in output mode.                                                             |

Table 13-13: PORT\_DATA\_CLR Register Fields (Continued)

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

Table 13-13: PORT\_DATA\_CLR Register Fields (Continued)

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
| 5 (R/W1C)          | PX5        | 1 Clear Bit. Write 1 for signal low in output mode.                                                         |
| 4 (R/W1C)          | PX4        | Port x Bit 4 Data Clear. The PORT_DATA_CLR.PX4 bit clears the pin without impacting other pins of the port. |
| 4 (R/W1C)          | PX4        | 0 No Effect. Write 0 has no effect in output mode.                                                          |
| 4 (R/W1C)          | PX4        | 1 Clear Bit. Write 1 for signal low in output mode.                                                         |
| 3 (R/W1C)          | PX3        | Port x Bit 3 Data Clear. The PORT_DATA_CLR.PX3 bit clears the pin without impacting other pins of the port. |
| 3 (R/W1C)          | PX3        | 0 No Effect. Write 0 has no effect in output mode.                                                          |
| 3 (R/W1C)          | PX3        | 1 Clear Bit. Write 1 for signal low in output mode.                                                         |
| 2 (R/W1C)          | PX2        | Port x Bit 2 Data Clear. The PORT_DATA_CLR.PX2 bit clears the pin without impacting other pins of the port. |
| 2 (R/W1C)          | PX2        | 0 No Effect Write 0 has no effect in output mode.                                                           |
| 2 (R/W1C)          | PX2        | 1 Clear Bit Write 1 for signal low in output mode.                                                          |

Table 13-13: PORT\_DATA\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                     | Description/Enumeration                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------|
| 1 (R/W1C)          | PX1        | Port x Bit 1 Data Clear. The PORT_DATA_CLR.PX1 bit clears the pin without impacting other pins of the port. | Port x Bit 1 Data Clear. The PORT_DATA_CLR.PX1 bit clears the pin without impacting other pins of the port. |
| 1 (R/W1C)          | PX1        | 0                                                                                                           | No Effect. Write 0 has no effect in output mode.                                                            |
| 1 (R/W1C)          | PX1        | 1                                                                                                           | Clear Bit. Write 1 for signal low in output mode.                                                           |
| 0 (R/W1C)          | PX0        | Port x Bit 0 Data Clear. The PORT_DATA_CLR.PX0 bit clears the pin without impacting other pins of the port. | Port x Bit 0 Data Clear. The PORT_DATA_CLR.PX0 bit clears the pin without impacting other pins of the port. |
| 0 (R/W1C)          | PX0        | 0                                                                                                           | No Effect. Write 0 has no effect in output mode.                                                            |
| 0 (R/W1C)          | PX0        | 1                                                                                                           | Clear Bit. Write 1 for signal low in output mode.                                                           |

## Port x GPIO Data Set Register

The PORT\_DATA\_SET register operates differently for port bits/pins, depending on whether the bit/pin is output mode or input mode. For more information, see the PORT\_DATA register description.

Figure 13-9: PORT\_DATA\_SET Register Diagram

<!-- image -->

Table 13-14: PORT\_DATA\_SET Register Fields

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

Table 13-14: PORT\_DATA\_SET Register Fields (Continued)

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

Table 13-14: PORT\_DATA\_SET Register Fields (Continued)

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

<!-- image -->

Table 13-15: PORT\_DATA\_TGL Register Fields

<!-- image -->

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | PX15       | Port x Bit 15 Toggle. The PORT_DATA_TGL.PX15 bit toggles the output GPIO bit/pin state.                          |
| 14 (R/W)           | PX14       | Port x Bit 14 Toggle. The PORT_DATA_TGL.PX14 bit toggles the output GPIO bit/pin state. 0 No Effect 1 Toggle Bit |

Table 13-15: PORT\_DATA\_TGL Register Fields (Continued)

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

Table 13-15: PORT\_DATA\_TGL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | PX6        | Port x Bit 6 Toggle. The PORT_DATA_TGL.PX6 bit toggles the output GPIO bit/pin state.                          |
| 5 (R/W)            | PX5        | Port x Bit 5 Toggle. The PORT_DATA_TGL.PX5 bit toggles the output GPIO bit/pin state. 0 No Effect              |
| 4 (R/W)            | PX4        | Port x Bit 4 Toggle. The PORT_DATA_TGL.PX4 bit toggles the output GPIO bit/pin state. 0 No Effect              |
| 3 (R/W)            | PX3        | Port x Bit 3 Toggle. The PORT_DATA_TGL.PX3 bit toggles the output GPIO bit/pin state. 0 No Effect              |
| 2 (R/W)            | PX2        | 1 Toggle Bit Port x Bit 2 Toggle. PORT_DATA_TGL.PX2 bit toggles the output GPIO bit/pin state.                 |
| 1 (R/W)            | PX1        | The 0 No Effect 1 Toggle Bit                                                                                   |
| 0                  |            | Port x Bit 1 Toggle. The PORT_DATA_TGL.PX1 bit toggles the output GPIO bit/pin state. 0 No Effect 1 Toggle Bit |
| (R/W)              | PX0        | Port x Bit 0 Toggle. The PORT_DATA_TGL.PX0 bit toggles the output GPIO bit/pin state. 0 No Effect              |

## Port x GPIO Direction Register

The PORT\_DIR , PORT\_DIR\_SET , and PORT\_DIR\_CLR registers select output or input mode for GPIO pins and enable output drivers. Use the PORT\_INEN , PORT\_INEN\_SET , and PORT\_INEN\_CLR registers to enable or disable input drivers.

Writes to the PORT\_DIR register affect the state of all pins of the port. T o select a direction for specific pins without impacting other pins of the port, use the PORT\_DIR\_SET and PORT\_DIR\_CLR registers.

Setting a bit in the PORT\_DIR register enables output mode on the corresponding a GPIO pin. Clearing a bit in the PORT\_DIR register disables output mode on the corresponding GPIO pin.

Input Mode - The default mode of every GPIO pin after reset is the input mode, but the input drivers are not enabled. To enable GPIO input drivers, set the corresponding bits in the PORT\_INEN register. When enabled, a read from the PORT\_DATA register returns the logical state of the input pin. The input signal does not overwrite the state of the bit used for the output case. That state can only be altered by software. If the input driver is enabled, a write to the PORT\_DATA register can alter the state of the bit, but the change cannot be read back.

Output Mode - Any GPIO pin can be configured for output mode. The GPIO output drivers are enabled by setting the corresponding bits in the PORT\_DIR , PORT\_DIR\_SET , or PORT\_DIR\_CLR registers. By using the PORT\_DIR\_SET and PORT\_DIR\_CLR registers, the direction of the signal flow of individual GPIO pins can be altered by separate software threads without mutually impacting other GPIOs on the same port. Both registers return the same value when read. Because the state of the GPIO output can already be controlled before the output driver is enabled, it is recommended to first set or clear the bit (using the PORT\_DATA , PORT\_DATA\_SET , or PORT\_DATA\_CLR registers) to avoid any volatile levels on the output.

Open-Drain Mode - Every GPIO can also be used in open-drain mode. To accomplish this, first, clear the respective bit in the PORT\_DATA or PORT\_DATA\_CLR register. Then, set the one bit in the PORT\_INEN register. Reads from the PORT\_DATA register then return the status from the pin and do not return the state of the internal flipflop. By toggling the output driver through the PORT\_DIR\_SET and PORT\_DIR\_CLR register pair, the output signal can be pulled low or three-stated as required. Note that the polarity of the driven signal can be inverted when the internal flip-flop is set instead. When a GPIO port is used in open-drain mode, take care to not exceed the V IH operating condition associated with the respective pin.

Figure 13-11: PORT\_DIR Register Diagram

<!-- image -->

Table 13-16: PORT\_DIR Register Fields

| Bit No. (Access)   | Bit Name                      | Description/Enumeration                      |
|--------------------|-------------------------------|----------------------------------------------|
| 15                 | PX15 Port x Bit 15 Direction. | PX15 Port x Bit 15 Direction.                |
| (R/W)              |                               | 0 Input mode. The output driver is disabled. |
|                    |                               | 1 Output mode. The output driver is enabled. |
| 14                 | Port x Bit 14 Direction.      | Port x Bit 14 Direction.                     |
| (R/W)              |                               | 0 Input mode. The output driver is disabled. |
|                    |                               | 1 Output mode. The output driver is enabled. |
| 13                 | Port x Bit 13 Direction.      | Port x Bit 13 Direction.                     |
| (R/W)              |                               | 0 Input mode. The output driver is disabled. |
|                    |                               | 1 Output mode. The output driver is enabled. |
| 12                 | Port x Bit 12 Direction.      | Port x Bit 12 Direction.                     |
| (R/W)              |                               | 0 Input mode. The output driver is disabled. |
|                    |                               | 1 Output mode. The output driver is enabled. |

Table 13-16: PORT\_DIR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   | Description/Enumeration                    |
|--------------------|------------|---------------------------|--------------------------------------------|
| 11                 | PX11       | Port x Bit 11 Direction.  | Port x Bit 11 Direction.                   |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
|                    |            | 1                         | Output mode. The output driver is enabled. |
| 10                 | PX10       | Port x Bit 10 Direction.  | Port x Bit 10 Direction.                   |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
|                    |            | 1                         | Output mode. The output driver is enabled. |
| 9                  | PX9        | Port x Bit 9 Direction.   | Port x Bit 9 Direction.                    |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
|                    |            | 1                         | Output mode. The output driver is enabled. |
| 8                  | PX8        | Port x Bit 8 Direction.   | Port x Bit 8 Direction.                    |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
|                    |            | 1                         | Output mode. The output driver is enabled. |
| 7                  | PX7        | Port x Bit 7 Direction.   | Port x Bit 7 Direction.                    |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
|                    |            | 1                         | Output mode. The output driver is enabled. |
| 6                  | PX6        | Port x Bit 6 Direction.   | Port x Bit 6 Direction.                    |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
|                    |            | 1                         | Output mode. The output driver is enabled. |
| 5                  | PX5        | Port x Bit 5 Direction.   | Port x Bit 5 Direction.                    |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
|                    |            | 1                         | Output mode. The output driver is enabled. |
| 4                  | PX4        | Port x Bit 4 Direction.   | Port x Bit 4 Direction.                    |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
|                    |            | 1                         | Output mode. The output driver is enabled. |
| 3                  | PX3        | Port x Bit 3 Direction.   | Port x Bit 3 Direction.                    |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
|                    |            | 1                         | Output mode. The output driver is enabled. |
| 2                  | PX2        | Port x Bit 2 Direction.   | Port x Bit 2 Direction.                    |
| (R/W)              |            | 0                         | Input mode. The output driver is disabled. |
|                    |            | 1                         | Output mode. The output driver is enabled. |

Table 13-16: PORT\_DIR Register Fields (Continued)

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

<!-- image -->

Table 13-17: PORT\_DIR\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                          | Description/Enumeration                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------|
| 15 (R/W1C)         | PX15       | Port x Bit 15 Direction Clear. The PORT_DIR_CLR.PX15 bit disables output mode and the output drivers for port x. | Port x Bit 15 Direction Clear. The PORT_DIR_CLR.PX15 bit disables output mode and the output drivers for port x. |
| 15 (R/W1C)         | PX15       | 0                                                                                                                | No Effect                                                                                                        |
| 15 (R/W1C)         | PX15       | 1                                                                                                                | Disable output mode/driver                                                                                       |
| 14 (R/W1C)         | PX14       | Port x Bit 14 Direction Clear. The PORT_DIR_CLR.PX14 bit disables output mode and the output drivers for port x. | Port x Bit 14 Direction Clear. The PORT_DIR_CLR.PX14 bit disables output mode and the output drivers for port x. |
| 14 (R/W1C)         | PX14       | 0                                                                                                                | No Effect                                                                                                        |
| 14 (R/W1C)         | PX14       | 1                                                                                                                | Disable output mode/driver                                                                                       |

Table 13-17: PORT\_DIR\_CLR Register Fields (Continued)

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

Table 13-17: PORT\_DIR\_CLR Register Fields (Continued)

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

Table 13-17: PORT\_DIR\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1C)          | PX1        | Port x Bit 1 Direction Clear. The PORT_DIR_CLR.PX1 bit disables output mode and the output drivers for port x. 0 No Effect 1 Disable output mode/driver |
| 0 (R/W1C)          | PX0        | Port x Bit 0 Direction Clear. The PORT_DIR_CLR.PX0 bit disables output mode and the output drivers for port x. 0 No Effect 1 Disable output mode/driver |

## Port x GPIO Direction Set Register

The PORT\_DIR\_SET register enables output mode and output drivers for GPIO pins. For more information, see the PORT\_DIR register description.

Figure 13-13: PORT\_DIR\_SET Register Diagram

<!-- image -->

Table 13-18: PORT\_DIR\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W1S)         | PX15       | Port x Bit 15 Direction Set. The PORT_DIR_SET.PX15 bit enables the output mode/driver for port x.                                         |
| 14 (R/W1S)         | PX14       | Port x Bit 14 Direction Set. The PORT_DIR_SET.PX14 bit enables the output mode/driver for port x. 0 No Effect 1 Enable output mode/driver |

Table 13-18: PORT\_DIR\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W1S)         | PX13       | Port x Bit 13 Direction Set. The PORT_DIR_SET.PX13 bit enables the output mode/driver for port x. 0 No Effect                             |
| 12 (R/W1S)         | PX12       | Port x Bit 12 Direction Set. The PORT_DIR_SET.PX12 bit enables the output mode/driver for port x. 0 No Effect 1 Enable output mode/driver |
| 11 (R/W1S)         | PX11       | Port x Bit 11 Direction Set. The PORT_DIR_SET.PX11 bit enables the output mode/driver for port x. 0 No Effect                             |
| 10 (R/W1S)         | PX10       | Port x Bit 10 Direction Set. The PORT_DIR_SET.PX10 bit enables the output mode/driver for port x. 0 No Effect                             |
| 9 (R/W1S)          | PX9        | 1 Enable output mode/driver                                                                                                               |
|                    |            | Port x Bit 9 Direction Set. The PORT_DIR_SET.PX9 bit enables the output mode/driver for port x. 0 No Effect 1 Enable output mode/driver   |
| 8 (R/W1S)          | PX8        | Port x Bit 8 Direction Set. The PORT_DIR_SET.PX8 bit enables the output mode/driver for port x. 0 No Effect 1 Enable output mode/driver   |
| 7 (R/W1S)          | PX7        | Port x Bit 7 Direction Set. The PORT_DIR_SET.PX7 bit enables the output mode/driver for port x. 0 No Effect                               |

Table 13-18: PORT\_DIR\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                         |                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------|
| 6 (R/W1S)          | PX6        | Port x Bit 6 Direction Set. The PORT_DIR_SET.PX6 bit enables the output mode/driver for port x. | Port x Bit 6 Direction Set. The PORT_DIR_SET.PX6 bit enables the output mode/driver for port x. |
| 6 (R/W1S)          | PX6        | 0                                                                                               | No Effect                                                                                       |
| 6 (R/W1S)          | PX6        | 1                                                                                               | Enable output mode/driver                                                                       |
| 5 (R/W1S)          | PX5        | Port x Bit 5 Direction Set. The PORT_DIR_SET.PX5 bit enables the output mode/driver for port x. | Port x Bit 5 Direction Set. The PORT_DIR_SET.PX5 bit enables the output mode/driver for port x. |
| 5 (R/W1S)          | PX5        | 0                                                                                               | No Effect                                                                                       |
| 5 (R/W1S)          | PX5        | 1                                                                                               | Enable output mode/driver                                                                       |
| 4 (R/W1S)          | PX4        | Port x Bit 4 Direction Set. The PORT_DIR_SET.PX4 bit enables the output mode/driver for port x. | Port x Bit 4 Direction Set. The PORT_DIR_SET.PX4 bit enables the output mode/driver for port x. |
| 4 (R/W1S)          | PX4        | 0                                                                                               | No Effect                                                                                       |
| 4 (R/W1S)          | PX4        | 1                                                                                               | Enable output mode/driver                                                                       |
| 3 (R/W1S)          | PX3        | Port x Bit 3 Direction Set. The PORT_DIR_SET.PX3 bit enables the output mode/driver for port x. | Port x Bit 3 Direction Set. The PORT_DIR_SET.PX3 bit enables the output mode/driver for port x. |
| 3 (R/W1S)          | PX3        | 0                                                                                               | No Effect                                                                                       |
| 3 (R/W1S)          | PX3        | 1                                                                                               | Enable output mode/driver                                                                       |
| 2 (R/W1S)          | PX2        | Port x Bit 2 Direction Set. The PORT_DIR_SET.PX2 bit enables the output mode/driver for port x. | Port x Bit 2 Direction Set. The PORT_DIR_SET.PX2 bit enables the output mode/driver for port x. |
| 2 (R/W1S)          | PX2        | 0                                                                                               | No Effect                                                                                       |
| 2 (R/W1S)          | PX2        | 1                                                                                               | Enable output mode/driver                                                                       |
| 1 (R/W1S)          | PX1        | Port x Bit 1 Direction Set.                                                                     | Port x Bit 1 Direction Set.                                                                     |
|                    |            | The PORT_DIR_SET.PX1 bit enables the output mode/driver for port x.                             | The PORT_DIR_SET.PX1 bit enables the output mode/driver for port x.                             |
|                    |            | 0                                                                                               | No Effect                                                                                       |
|                    |            | 1                                                                                               | Enable output mode/driver                                                                       |
| 0 (R/W1S)          | PX0        | Port x Bit 0 Direction Set. The PORT_DIR_SET.PX0 bit enables the output mode/driver for port x. | Port x Bit 0 Direction Set. The PORT_DIR_SET.PX0 bit enables the output mode/driver for port x. |
| 0 (R/W1S)          | PX0        | 0                                                                                               | No Effect                                                                                       |
| 0 (R/W1S)          | PX0        | 1                                                                                               | Enable output mode/driver                                                                       |

## Port x Function Enable Register

The PORT\_FER register bits indicate each port bit's operating mode: general purpose I/O mode or peripheral mode. After reset, all pins default to GPIO mode. Setting a bit in the PORT\_FER registers enables a peripheral module to take ownership of the pin. The function enable bits impact output control only. Regardless of the setting of the function enable bits, both GPIO and peripherals can still sense the pin input. After a function is enabled, it is up to the PORT\_MUX registers as to which peripheral takes control.

Figure 13-14: PORT\_FER Register Diagram

<!-- image -->

Table 13-19: PORT\_FER Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | PX15       | Port x Bit 15 Mode. The PORT_FER.PX15 bit indicates the operating mode for port x.                               |
| 14 (R/W)           | PX14       | Port x Bit 14 Mode. The PORT_FER.PX14 bit indicates the operating mode for port x. 0 GPIO Mode 1 Peripheral Mode |

Table 13-19: PORT\_FER Register Fields (Continued)

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

Table 13-19: PORT\_FER Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | PX6        | Port x Bit 6 Mode. The PORT_FER.PX6 bit indicates the operating mode for port x.                               |
| 5 (R/W)            | PX5        | Port x Bit 5 Mode. The PORT_FER.PX5 bit indicates the operating mode for port x. 0 GPIO Mode                   |
| 4 (R/W)            | PX4        | Port x Bit 4 Mode. The PORT_FER.PX4 bit indicates the operating mode for port x. 0 GPIO Mode                   |
| 3 (R/W)            | PX3        | Port x Bit 3 Mode. The PORT_FER.PX3 bit indicates the operating mode for port x.                               |
| 2 (R/W)            |            | 0 GPIO Mode 1 Peripheral Mode                                                                                  |
|                    | PX2        | Port x Bit 2 Mode. The PORT_FER.PX2 bit indicates the operating mode for port x. 0 GPIO Mode 1 Peripheral Mode |
| 1 (R/W)            | PX1        | Port x Bit 1 Mode. The PORT_FER.PX1 bit indicates the operating mode for port x. 0 GPIO Mode 1 Peripheral Mode |
| 0 (R/W)            | PX0        | Port x Bit 0 Mode. The PORT_FER.PX0 bit indicates the operating mode for port x. 0 GPIO Mode                   |

## Port x Function Enable Clear Register

The PORT\_FER\_CLR register permits enabling GPIO mode for each bit and corresponding GPIO pin. Writing 1 to a bit in PORT\_FER\_CLR enables GPIO mode for the corresponding pin.

Figure 13-15: PORT\_FER\_CLR Register Diagram

<!-- image -->

Table 13-20: PORT\_FER\_CLR Register Fields

<!-- image -->

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------|
| 15 (R/W1C)         | PX15       | Port x Bit 15 Mode Clear. The PORT_FER_CLR.PX15 bit enables GPIO mode.                                     |
| 14 (R/W1C)         | PX14       | Port x Bit 14 Mode Clear. The PORT_FER_CLR.PX14 bit enables GPIO mode. 0 No Effect 1 Set Bit for GPIO Mode |

Table 13-20: PORT\_FER\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------|
| 13 (R/W1C)         | PX13       | Port x Bit 13 Mode Clear. The PORT_FER_CLR.PX13 bit enables GPIO mode. 0 No Effect                       |
| 12 (R/W1C)         | PX12       | Port x Bit 12 Mode Clear. The PORT_FER_CLR.PX12 bit enables GPIO mode. 0 No Effect                       |
| 11 (R/W1C)         | PX11       | Port x Bit 11 Mode Clear. The PORT_FER_CLR.PX11 bit enables GPIO mode. 0 No Effect                       |
| 10 (R/W1C)         | PX10       | Port x Bit 10 Mode Clear. The PORT_FER_CLR.PX10 bit enables GPIO mode. 0 No Effect                       |
| 9 (R/W1C)          | PX9        | Port x Bit 9 Mode Clear. The PORT_FER_CLR.PX9 bit enables GPIO mode. 0 No Effect 1 Set Bit for GPIO Mode |
| 8 (R/W1C)          | PX8        | Port x Bit 8 Mode Clear. The PORT_FER_CLR.PX8 bit enables GPIO mode. 0 No Effect                         |
| 7 (R/W1C)          | PX7        | 1 Set Bit for GPIO Mode Port x Bit 7 Mode Clear. The PORT_FER_CLR.PX7 bit enables GPIO mode. 0 No Effect |

Table 13-20: PORT\_FER\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------|
| 6 (R/W1C)          | PX6        | Port x Bit 6 Mode Clear. The PORT_FER_CLR.PX6 bit enables GPIO mode. 0 No Effect                         |
| 5 (R/W1C)          | PX5        | Port x Bit 5 Mode Clear. The PORT_FER_CLR.PX5 bit enables GPIO mode. 0 No Effect                         |
| 4 (R/W1C)          | PX4        | Port x Bit 4 Mode Clear. The PORT_FER_CLR.PX4 bit enables GPIO mode. 0 No Effect                         |
| 3 (R/W1C)          | PX3        | Port x Bit 3 Mode Clear. The PORT_FER_CLR.PX3 bit enables GPIO mode. 0 No Effect                         |
| 2 (R/W1C)          | PX2        | 1 Set Bit for GPIO Mode Port x Bit 2 Mode Clear. The PORT_FER_CLR.PX2 bit enables GPIO mode. 0 No Effect |
| 1 (R/W1C)          | PX1        | 1 Set Bit for GPIO Mode Port x Bit 1 Mode Clear. The PORT_FER_CLR.PX1 bit enables GPIO mode. 0 No Effect |
| 0 (R/W1C)          | PX0        | 1 Set Bit for GPIO Mode Port x Bit 0 Mode Clear. The PORT_FER_CLR.PX0 bit enables GPIO mode. 0 No Effect |

## Port x Function Enable Set Register

The PORT\_FER\_SET register permits enabling peripheral mode for each bit and corresponding GPIO pin. Writing 1 to a bit in PORT\_FER\_SET enables peripheral mode for the corresponding pin.

Figure 13-16: PORT\_FER\_SET Register Diagram

<!-- image -->

Table 13-21: PORT\_FER\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------|
| 15 (R/W1S)         | PX15       | Port x Bit 15 Mode Set. The PORT_FER_SET.PX15 bit enables peripheral mode.                                           |
| 14 (R/W1S)         | PX14       | Port x Bit 14 Mode Set. The PORT_FER_SET.PX14 bit enables peripheral mode. 0 No Effect 1 Set Bit for Peripheral Mode |

Table 13-21: PORT\_FER\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------|
| 13 (R/W1S)         | PX13       | Port x Bit 13 Mode Set. The PORT_FER_SET.PX13 bit enables peripheral mode. 0 No Effect                             |
| 12 (R/W1S)         | PX12       | Port x Bit 12 Mode Set. The PORT_FER_SET.PX12 bit enables peripheral mode. 0 No Effect                             |
| 11 (R/W1S)         | PX11       | Port x Bit 11 Mode Set. The PORT_FER_SET.PX11 bit enables peripheral mode. 0 No Effect                             |
| 10 (R/W1S)         | PX10       | Port x Bit 10 Mode Set. The PORT_FER_SET.PX10 bit enables peripheral mode. 0 No Effect                             |
| 9 (R/W1S)          | PX9        | Port x Bit 9 Mode Set. The PORT_FER_SET.PX9 bit enables peripheral mode. 0 No Effect 1 Set Bit for Peripheral Mode |
| 8 (R/W1S)          | PX8        | Port x Bit 8 Mode Set. The PORT_FER_SET.PX8 bit enables peripheral mode. 0 No Effect                               |
| 7 (R/W1S)          | PX7        | 1 Set Bit for Peripheral Mode Port x Bit 7 Mode Set. The PORT_FER_SET.PX7 bit enables peripheral mode. 0 No Effect |

Table 13-21: PORT\_FER\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------|
| 6 (R/W1S)          | PX6        | Port x Bit 6 Mode Set. The PORT_FER_SET.PX6 bit enables peripheral mode. 0 No Effect                               |
| 5 (R/W1S)          | PX5        | Port x Bit 5 Mode Set. The PORT_FER_SET.PX5 bit enables peripheral mode. 0 No Effect                               |
| 4 (R/W1S)          | PX4        | Port x Bit 4 Mode Set. The PORT_FER_SET.PX4 bit enables peripheral mode. 0 No Effect                               |
| 3 (R/W1S)          | PX3        | Port x Bit 3 Mode Set. The PORT_FER_SET.PX3 bit enables peripheral mode. 0 No Effect                               |
| 2 (R/W1S)          | PX2        | 1 Set Bit for Peripheral Mode Port x Bit 2 Mode Set. The PORT_FER_SET.PX2 bit enables peripheral mode. 0 No Effect |
| 1 (R/W1S)          | PX1        | 1 Set Bit for Peripheral Mode Port x Bit 1 Mode Set. The PORT_FER_SET.PX1 bit enables peripheral mode. 0 No Effect |
| 0 (R/W1S)          | PX0        | 1 Set Bit for Peripheral Mode Port x Bit 0 Mode Set. The PORT_FER_SET.PX0 bit enables peripheral mode. 0 No Effect |

## Port x GPIO Input Enable Register

The PORT\_INEN , PORT\_INEN\_SET , and PORT\_INEN\_CLR registers enable or disable input drivers, which are required for using a GPIO pin in input mode.

Writes to the PORT\_INEN register affect the input drivers for all pins of the port. T o set or clear specific pin drivers without impacting other pin drivers of the port, use the PORT\_INEN\_SET and PORT\_INEN\_CLR registers.

If the input is enabled, reads from the PORT\_DATA , PORT\_DATA\_SET , or PORT\_DATA\_CLR registers return the state of the pins. However, the state of the output is not overwritten by the input. It is altered by software writes only. Input and output drivers can be enabled at the same time. In this case, a read of the data register returns the true value of the data register and not the pin state.

For more information, see the PORT\_DATA register description and the PORT\_DIR register description.

Figure 13-17: PORT\_INEN Register Diagram

<!-- image -->

Table 13-22: PORT\_INEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration     |
|--------------------|------------|-----------------------------|
| 15                 | PX15       | Port x Bit 15 Input Enable. |
| (R/W)              | PX15       | 0 Disable Input Driver      |
| (R/W)              | PX15       | 1 Enable Input Driver       |

Table 13-22: PORT\_INEN Register Fields (Continued)

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

Table 13-22: PORT\_INEN Register Fields (Continued)

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

<!-- image -->

Table 13-23: PORT\_INEN\_CLR Register Fields

| Bit No. (Access)   | Description/Enumeration                       | Description/Enumeration                       | Description/Enumeration                       | Description/Enumeration                       | Description/Enumeration                       | Description/Enumeration                       | Description/Enumeration                       | Description/Enumeration                       | Description/Enumeration                       | Description/Enumeration                       |
|--------------------|-----------------------------------------------|-----------------------------------------------|-----------------------------------------------|-----------------------------------------------|-----------------------------------------------|-----------------------------------------------|-----------------------------------------------|-----------------------------------------------|-----------------------------------------------|-----------------------------------------------|
| 15                 | Port x Bit 15 Input Enable Clear.             | Port x Bit 15 Input Enable Clear.             | Port x Bit 15 Input Enable Clear.             | Port x Bit 15 Input Enable Clear.             | Port x Bit 15 Input Enable Clear.             | Port x Bit 15 Input Enable Clear.             | Port x Bit 15 Input Enable Clear.             | Port x Bit 15 Input Enable Clear.             | Port x Bit 15 Input Enable Clear.             | Port x Bit 15 Input Enable Clear.             |
| 14                 | Port x Bit 14 Input Enable Clear. 0 No Effect | Port x Bit 14 Input Enable Clear. 0 No Effect | Port x Bit 14 Input Enable Clear. 0 No Effect | Port x Bit 14 Input Enable Clear. 0 No Effect | Port x Bit 14 Input Enable Clear. 0 No Effect | Port x Bit 14 Input Enable Clear. 0 No Effect | Port x Bit 14 Input Enable Clear. 0 No Effect | Port x Bit 14 Input Enable Clear. 0 No Effect | Port x Bit 14 Input Enable Clear. 0 No Effect | Port x Bit 14 Input Enable Clear. 0 No Effect |
| 13                 | 1 Clear Bit. Set to disable the input driver. | 1 Clear Bit. Set to disable the input driver. | 1 Clear Bit. Set to disable the input driver. | 1 Clear Bit. Set to disable the input driver. | 1 Clear Bit. Set to disable the input driver. | 1 Clear Bit. Set to disable the input driver. | 1 Clear Bit. Set to disable the input driver. | 1 Clear Bit. Set to disable the input driver. | 1 Clear Bit. Set to disable the input driver. | 1 Clear Bit. Set to disable the input driver. |
| (R/W1C)            | 0 No Effect                                   | 0 No Effect                                   | 0 No Effect                                   | 0 No Effect                                   | 0 No Effect                                   | 0 No Effect                                   | 0 No Effect                                   | 0 No Effect                                   | 0 No Effect                                   | 0 No Effect                                   |
|                    | Clear Bit. Set to disable the input driver.   | Clear Bit. Set to disable the input driver.   | Clear Bit. Set to disable the input driver.   | Clear Bit. Set to disable the input driver.   | Clear Bit. Set to disable the input driver.   | Clear Bit. Set to disable the input driver.   | Clear Bit. Set to disable the input driver.   | Clear Bit. Set to disable the input driver.   | Clear Bit. Set to disable the input driver.   | Clear Bit. Set to disable the input driver.   |
|                    | 1                                             | 1                                             | 1                                             | 1                                             | 1                                             | 1                                             | 1                                             | 1                                             | 1                                             | 1                                             |

Table 13-23: PORT\_INEN\_CLR Register Fields (Continued)

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
| 6 (R/W1C)          | PX6        | Port x Bit 6 Input Enable Clear.              |
| 6 (R/W1C)          | PX6        | 0 No Effect                                   |
| 6 (R/W1C)          | PX6        | 1 Clear Bit. Set to disable the input driver. |
| 5 (R/W1C)          | PX5        | Port x Bit 5 Input Enable Clear.              |
| 5 (R/W1C)          | PX5        | 0 No Effect                                   |
| 4 (R/W1C)          | PX4        | Port x Bit 4 Input Enable Clear.              |
| 4 (R/W1C)          | PX4        | 0 No Effect                                   |
| 4 (R/W1C)          | PX4        | 1 Clear Bit. Set to disable the input driver. |
| 3                  | PX3        | Port x Bit 3 Input Enable Clear.              |
| (R/W1C)            | PX3        | 0 No Effect                                   |
| 3                  | PX3        | 1 Clear Bit. Set to disable the input driver. |

Table 13-23: PORT\_INEN\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                        |
|--------------------|------------|--------------------------------------------------------------------------------|
| 2                  | PX2        | Port x Bit 2 Input Enable Clear.                                               |
| 1 (R/W1C)          | PX1        | Port x Bit 1 Input Enable Clear. 0 No Effect                                   |
| 0                  | PX0        | 1 Clear Bit. Set to disable the input driver. Port x Bit 0 Input Enable Clear. |
| (R/W1C)            |            | 0 No Effect 1 Clear Bit. Set to disable the input driver.                      |

## Port x GPIO Input Enable Set Register

The PORT\_INEN\_SET register enables input drivers for GPIO pins. For more information, see the PORT\_INEN register description.

Figure 13-19: PORT\_INEN\_SET Register Diagram

<!-- image -->

Table 13-24: PORT\_INEN\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                    |
|--------------------|------------|----------------------------------------------------------------------------|
| 15                 | PX15       | Port x Bit 15 Input Enable Set.                                            |
| 14 (R/W1S)         | PX14       | Port x Bit 14 Input Enable Set. 0 No Effect                                |
| 13                 | PX13       | 1 Set Bit. Set to enable the input driver. Port x Bit 13 Input Enable Set. |
| (R/W1S)            |            | 0 No Effect 1 Set Bit. Set to enable the input driver.                     |

Table 13-24: PORT\_INEN\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration         |                                          |
|--------------------|------------|---------------------------------|------------------------------------------|
| 12 (R/W1S)         | PX12       | Port x Bit 12 Input Enable Set. | Port x Bit 12 Input Enable Set.          |
| 12 (R/W1S)         | PX12       | 0                               | No Effect                                |
| 12 (R/W1S)         | PX12       | 1                               | Set Bit. Set to enable the input driver. |
| 11                 | PX11       | Port x Bit 11 Input Enable Set. | Port x Bit 11 Input Enable Set.          |
| (R/W1S)            |            | 0                               | No Effect                                |
| (R/W1S)            |            | 1                               | Set Bit. Set to enable the input driver. |
| 10                 | PX10       | Port x Bit 10 Input Enable Set. | Port x Bit 10 Input Enable Set.          |
| (R/W1S)            |            | 0                               | No Effect                                |
| (R/W1S)            |            | 1                               | Set Bit. Set to enable the input driver. |
| 9                  | PX9        | Port x Bit 9 Input Enable Set.  | Port x Bit 9 Input Enable Set.           |
| (R/W1S)            |            | 0                               | No Effect                                |
| (R/W1S)            |            | 1                               | Set Bit. Set to enable the input driver. |
| 8                  | PX8        | Port x Bit 8 Input Enable Set.  | Port x Bit 8 Input Enable Set.           |
| (R/W1S)            |            | 0                               | No Effect                                |
| (R/W1S)            |            | 1                               | Set Bit. Set to enable the input driver. |
| 7                  | PX7        | Port x Bit 7 Input Enable Set.  | Port x Bit 7 Input Enable Set.           |
| (R/W1S)            |            | 0                               | No Effect                                |
| (R/W1S)            |            | 1                               | Set Bit. Set to enable the input driver. |
| 6                  | PX6        | Port x Bit 6 Input Enable Set.  | Port x Bit 6 Input Enable Set.           |
| (R/W1S)            |            | 0                               | No Effect                                |
| (R/W1S)            |            | 1                               | Set Bit. Set to enable the input driver. |
| 5                  | PX5        | Port x Bit 5 Input Enable Set.  | Port x Bit 5 Input Enable Set.           |
| (R/W1S)            |            | 0                               | No Effect                                |
| (R/W1S)            |            | 1                               | Set Bit. Set to enable the input driver. |
| 4                  | PX4        | Port x Bit 4 Input Enable Set.  | Port x Bit 4 Input Enable Set.           |
| (R/W1S)            |            | 0                               | No Effect                                |
| (R/W1S)            |            | 1                               | Set Bit. Set to enable the input driver. |
| 3                  | PX3        | Port x Bit 3 Input Enable Set.  | Port x Bit 3 Input Enable Set.           |
| (R/W1S)            |            | 0                               | No Effect                                |
| (R/W1S)            |            | 1                               | Set Bit. Set to enable the input driver. |

Table 13-24: PORT\_INEN\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                   |
|--------------------|------------|---------------------------------------------------------------------------|
| 2                  | PX2        | Port x Bit 2 Input Enable Set.                                            |
| 1 (R/W1S)          | PX1        | Port x Bit 1 Input Enable Set. 0 No Effect                                |
| 0                  | PX0        | 1 Set Bit. Set to enable the input driver. Port x Bit 0 Input Enable Set. |
| (R/W1S)            |            | 0 No Effect 1 Set Bit. Set to enable the input driver.                    |

## Port x GPIO Lock Register

The PORT\_LOCK register enables (unlocks) or disables (locks) write access selectively for the PORT control registers.

Figure 13-20: PORT\_LOCK Register Diagram

<!-- image -->

Table 13-25: PORT\_LOCK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the PORT_LOCK.LOCK bit is set, the PORT_LOCK register is read only (locked).           |
| 5 (R/W)            | POLAR      | Polarity Lock. The PORT_LOCK.POLAR disables write access to the PORT_POL , PORT_POL_SET , and PORT_POL_CLR registers.                                 |
| 4 (R/W)            | INEN       | Input Enable Lock. The PORT_LOCK.INEN disables write access to the PORT_INEN , PORT_INEN_SET , and PORT_INEN_CLR registers. 0 Unlock INEN 1 Lock INEN |
| 4 (R/W)            |            |                                                                                                                                                       |

Table 13-25: PORT\_LOCK Register Fields (Continued)

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

<!-- image -->

Table 13-26: PORT\_MUX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                |
|--------------------|------------|------------------------------------------------------------------------|
| 31:30              | MUX15      | Mux for Port x Bit 15.                                                 |
| (R/W)              |            | The PORT_MUX.MUX15 bit provides multiplexer control for port x bit 15. |

Table 13-26: PORT\_MUX Register Fields (Continued)

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

<!-- image -->

Table 13-27: PORT\_POL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                          |
|--------------------|------------|----------------------------------------------------------------------------------|
| 15 (R/W)           | PX15       | Port x Bit 15 Polarity Invert. The PORT_POL.PX15 bit enables polarity inversion. |
| 15 (R/W)           | PX15       | 0 No Invert. GPIO is active high or rising edge sensitive.                       |
| 15 (R/W)           | PX15       | 1 Invert. GPIO is active low or falling edge sensitive.                          |

Table 13-27: PORT\_POL Register Fields (Continued)

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

Table 13-27: PORT\_POL Register Fields (Continued)

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

Table 13-27: PORT\_POL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                        |
|--------------------|------------|--------------------------------------------------------------------------------|
| 0 (R/W)            | PX0        | Port x Bit 0 Polarity Invert. The PORT_POL.PX0 bit enables polarity inversion. |

## Port x GPIO Polarity Invert Clear Register

The PORT\_POL\_CLR register disables polarity inversion for GPIO pins. For more information, see the PORT\_POL register description.

Figure 13-23: PORT\_POL\_CLR Register Diagram

<!-- image -->

Table 13-28: PORT\_POL\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------|
| 15                 | PX15       | Port x Bit 15 Polarity Invert Clear.                                                       |
| 14 (R/W1C)         | PX14       | Port x Bit 14 Polarity Invert Clear. 0 No Effect                                           |
| 13                 | PX13       | 1 Clear Bit. Set to disable GPIO pin polarity invert. Port x Bit 13 Polarity Invert Clear. |
| (R/W1C)            |            | 0 No Effect                                                                                |
|                    |            | 1 Clear Bit. Set to disable GPIO pin polarity invert.                                      |

Table 13-28: PORT\_POL\_CLR Register Fields (Continued)

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
| 6 (R/W1C)          | PX6        | Port x Bit 6 Polarity Invert Clear.  | Port x Bit 6 Polarity Invert Clear.                 |
| 6 (R/W1C)          | PX6        | 0                                    | No Effect                                           |
| 6 (R/W1C)          | PX6        | 1                                    | Clear Bit. Set to disable GPIO pin polarity invert. |
| 5                  | PX5        | Port x Bit 5 Polarity Invert Clear.  | Port x Bit 5 Polarity Invert Clear.                 |
| (R/W1C)            |            | 0                                    | No Effect                                           |
| (R/W1C)            |            | 1                                    | Clear Bit. Set to disable GPIO pin polarity invert. |
| 4 (R/W1C)          | PX4        | Port x Bit 4 Polarity Invert Clear.  | Port x Bit 4 Polarity Invert Clear.                 |
| 4 (R/W1C)          | PX4        | 0                                    | No Effect                                           |
| 4 (R/W1C)          | PX4        | 1                                    | Clear Bit. Set to disable GPIO pin polarity invert. |
| 3                  | PX3        | Port x Bit 3 Polarity Invert Clear.  | Port x Bit 3 Polarity Invert Clear.                 |
| (R/W1C)            |            | 0                                    | No Effect                                           |
| (R/W1C)            |            | 1                                    | Clear Bit. Set to disable GPIO pin polarity invert. |

Table 13-28: PORT\_POL\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------|
| 2                  | PX2        | Port x Bit 2 Polarity Invert Clear.                                                       |
| 1 (R/W1C)          | PX1        | Port x Bit 1 Polarity Invert Clear. 0 No Effect                                           |
| 0                  | PX0        | 1 Clear Bit. Set to disable GPIO pin polarity invert. Port x Bit 0 Polarity Invert Clear. |
| (R/W1C)            |            | 0 No Effect 1 Clear Bit. Set to disable GPIO pin polarity invert.                         |

## Port x GPIO Polarity Invert Set Register

The PORT\_POL\_SET register enables polarity inversion for GPIO pins. For more information, see the PORT\_POL register description.

Figure 13-24: PORT\_POL\_SET Register Diagram

<!-- image -->

Table 13-29: PORT\_POL\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W1S)         | PX15       | Port x Bit 15 Polarity Invert Set. The PORT_POL_SET.PX15 bit enables pin polarity inversion.                                                        |
| 14 (R/W1S)         | PX14       | Port x Bit 14 Polarity Invert Set. The PORT_POL_SET.PX14 bit enables pin polarity inversion. 0 No Effect 1 Set Bit. Set to enable GPIO pin polarity |

Table 13-29: PORT\_POL\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W1S)         | PX13       | Port x Bit 13 Polarity Invert Set. The PORT_POL_SET.PX13 bit enables pin polarity inversion.                                                              |
| 12 (R/W1S)         | PX12       | Port x Bit 12 Polarity Invert Set. The PORT_POL_SET.PX12 bit enables pin polarity inversion. 0 No Effect                                                  |
| 11 (R/W1S)         | PX11       | Port x Bit 11 Polarity Invert Set. The PORT_POL_SET.PX11 bit enables pin polarity inversion. 0 No Effect                                                  |
| 10 (R/W1S)         | PX10       | 1 Set Bit. Set to enable GPIO pin polarity invert. Port x Bit 10 Polarity Invert Set. The PORT_POL_SET.PX10 bit enables pin polarity inversion.           |
| 9 (R/W1S)          | PX9        | 0 No Effect 1 Set Bit. Set to enable GPIO pin polarity invert.                                                                                            |
|                    |            | Port x Bit 9 Polarity Invert Set. The PORT_POL_SET.PX9 bit enables pin polarity inversion. 0 No Effect 1 Set Bit. Set to enable GPIO pin polarity invert. |
| 8 (R/W1S)          | PX8        | Port x Bit 8 Polarity Invert Set. The PORT_POL_SET.PX8 bit enables pin polarity inversion. 0 No Effect 1 Set Bit. Set to enable GPIO pin polarity invert. |
| 7 (R/W1S)          | PX7        | Port x Bit 7 Polarity Invert Set. The PORT_POL_SET.PX7 bit enables pin polarity inversion.                                                                |
|                    |            | 0 No Effect                                                                                                                                               |

Table 13-29: PORT\_POL\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W1S)          | PX6        | Port x Bit 6 Polarity Invert Set. The PORT_POL_SET.PX6 bit enables pin polarity inversion.                                                                |
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

<!-- image -->

Table 13-30: PORT\_TRIG\_TGL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | PX15       | PX15 Data Toggle on Trigger. The PORT_TRIG_TGL.PX15 bit enables triggers to toggle the state of the pin. |
| 14 (R/W)           | PX14       | PX14 Data Toggle on Trigger. The PORT_TRIG_TGL.PX14 bit enables triggers to toggle the state of the pin. |
| 13 (R/W)           | PX13       | PX13 Data Toggle on Trigger. The PORT_TRIG_TGL.PX13 bit enables triggers to toggle the state of the pin. |
| 12 (R/W)           | PX12       | PX12 Data Toggle on Trigger. The PORT_TRIG_TGL.PX12 bit enables triggers to toggle the state of the pin. |
| 11 (R/W)           | PX11       | PX11 Data Toggle on Trigger. The PORT_TRIG_TGL.PX11 bit enables triggers to toggle the state of the pin. |

Table 13-30: PORT\_TRIG\_TGL Register Fields (Continued)

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

## ADSP-2159x\_SC592\_SC594 PINT Register Descriptions

The Pin Interrupt module (PINT) contains the following registers.

Table 13-31: ADSP-2159x\_SC592\_SC594 PINT Register List

| Name          | Description                |
|---------------|----------------------------|
| PINT_ASSIGN   | PINT Assign Register       |
| PINT_EDGE_CLR | PINT Edge Clear Register   |
| PINT_EDGE_SET | PINT Edge Set Register     |
| PINT_INV_CLR  | PINT Invert Clear Register |

Table 13-31: ADSP-2159x\_SC592\_SC594 PINT Register List (Continued)

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

<!-- image -->

Table 13-32: PINT\_ASSIGN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                               |
|--------------------|------------|-----------------------------------------------------------------------|
| 31:25 (R/NW)       | B3MSB0     | Bits [7:1] of B3MAP.                                                  |
| 24 (R/W)           | B3MAP      | Byte 3 Mapping. 0 B3MAP_PAH. Byte 3 = PA.H 1 B3MAP_PBH. Byte 3 = PB.H |
| 23:17 (R/NW)       | B2MSB0     | Bits [7:1] of B2MAP.                                                  |
| 16 (R/W)           | B2MAP      | Byte 2 Mapping. 0 B2MAP_PAL. Byte 2 = PA.L 1 B2MAP_PBL. Byte 2 = PB.L |
| 15:9 (R/NW)        | B1MSB0     | Bits [7:1] of B1MAP.                                                  |

Table 13-32: PINT\_ASSIGN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                    |
|--------------------|------------|--------------------------------------------|
| 8                  | B1MAP      | Byte 1 Mapping.                            |
| 7:1 (R/NW)         | B0MSB0     | Bits [7:1] of B0MAP.                       |
| 0 (R/W)            | B0MAP      | Byte 0 Mapping. 0 B0MAP_PAL. Byte 0 = PA.L |

## PINT Edge Clear Register

The PINT\_EDGE\_CLR register permits selecting level-sensitive interrupts. Writing 1 to a bit in PINT\_EDGE\_CLR enables level sensitivity for the corresponding pin interrupt.

Figure 13-27: PINT\_EDGE\_CLR Register Diagram

<!-- image -->

Table 13-33: PINT\_EDGE\_CLR Register Fields

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

Table 13-33: PINT\_EDGE\_CLR Register Fields (Continued)

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

<!-- image -->

Table 13-34: PINT\_EDGE\_SET Register Fields

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

Table 13-34: PINT\_EDGE\_SET Register Fields (Continued)

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

<!-- image -->

Table 13-35: PINT\_INV\_CLR Register Fields

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

Table 13-35: PINT\_INV\_CLR Register Fields (Continued)

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

<!-- image -->

Table 13-36: PINT\_INV\_SET Register Fields

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

Table 13-36: PINT\_INV\_SET Register Fields (Continued)

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

<!-- image -->

Table 13-37: PINT\_LATCH Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                     |
|--------------------|------------|-------------------------------------------------------------|
| 31                 | PIQ31      | Pin Interrupt 31 Latch.                                     |
| (R/W1C)            |            | If the PINT_LATCH.PIQ31 bit is set, the request is latched. |

Table 13-37: PINT\_LATCH Register Fields (Continued)

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

Table 13-37: PINT\_LATCH Register Fields (Continued)

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

<!-- image -->

Table 13-38: PINT\_MSK\_CLR Register Fields

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

Table 13-38: PINT\_MSK\_CLR Register Fields (Continued)

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

<!-- image -->

Table 13-39: PINT\_MSK\_SET Register Fields

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

Table 13-39: PINT\_MSK\_SET Register Fields (Continued)

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

<!-- image -->

Table 13-40: PINT\_PINSTATE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                      |
|--------------------|------------|--------------------------------------------------------------|
| 31                 | PIQ31      | Pin Interrupt 31 State.                                      |
| (R/NW)             |            | A read of the PINT_PINSTATE.PIQ31 bit returns the pin state. |

Table 13-40: PINT\_PINSTATE Register Fields (Continued)

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

Table 13-40: PINT\_PINSTATE Register Fields (Continued)

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

<!-- image -->

Table 13-41: PINT\_REQ Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                 |
|--------------------|------------|---------------------------------------------------------|
| 31                 | PIQ31      | Pin Interrupt 31 Request.                               |
| (R/W1C)            |            | If the PINT_REQ.PIQ31 bit is set, a request is pending. |

Table 13-41: PINT\_REQ Register Fields (Continued)

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

Table 13-41: PINT\_REQ Register Fields (Continued)

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

## ADSP-2159x\_SC592\_SC594 PADS Register Descriptions

Pads Controller (PADS) contains the following registers.

Table 13-42: ADSP-2159x\_SC592\_SC594 PADS Register List

| Name              | Description                             |
|-------------------|-----------------------------------------|
| PADS_DAI0_0_DS    | DAI0 1 to 10 pins DS control            |
| PADS_DAI0_1_DS    | DAI0 11 to 20 pins DS control           |
| PADS_DAI0_IE      | DAI0 Port Input Enable Control Register |
| PADS_DAI1_0_DS    | DAI1 1 to 10 pins DS control            |
| PADS_DAI1_1_DS    | DAI1 11 to 20 pins DS control           |
| PADS_DAI1_IE      | DAI1 Port Input Enable Control Register |
| PADS_DAI[n]_PDE   | DAI[n] Pull-Down Enable                 |
| PADS_DAI[n]_PUE   | DAI[n] Pull-Up Enable                   |
| PADS_NONPORTS_CTL | Non-GPIO control Register               |
| PADS_NONPORTS_DS  | Non-GPIO Drive Strength Register        |
| PADS_PCFG0        | Peripheral PAD Configuration0 Register  |
| PADS_PCFG1        | Peripheral Configuration1 Register      |
| PADS_PORTA0_DS    | PORTA 0 to 7 pins DS control            |
| PADS_PORTA1_DS    | PORTA 8 - 15 pins DS control            |
| PADS_PORTB0_DS    | PORTB 0 to 7 pins DS control            |
| PADS_PORTB1_DS    | PORTB 8 - 15 pins DS control            |
| PADS_PORTC0_DS    | PORTC 0 to 7 pins DS control            |
| PADS_PORTC1_DS    | PORTC 8 - 15 pins DS control            |
| PADS_PORTD0_DS    | PORTD 0 to 7 pins DS control            |
| PADS_PORTD1_DS    | PORTD 8 to 15 pins DS control           |
| PADS_PORTE0_DS    | PORTE 0 to 7 pins DS control            |
| PADS_PORTE1_DS    | PORTE 8 to 15 pins DS control           |
| PADS_PORTF0_DS    | PORTF 0 to 7 pins DS control            |
| PADS_PORTF1_DS    | PORTF 8 to 15 pins DS control           |
| PADS_PORTG0_DS    | PORTG 0 to 7 pins DS control            |
| PADS_PORTG1_DS    | PORTG 8 to 15 pins DS control           |
| PADS_PORTH0_DS    | PORTH 0 to 7 pins DS control            |
| PADS_PORTH1_DS    | PORTH 8 to 15 pins DS control           |
| PADS_PORTI0_DS    | PORTI 0 to 7 pins DS control            |

Table 13-42: ADSP-2159x\_SC592\_SC594 PADS Register List (Continued)

| Name             | Description              |
|------------------|--------------------------|
| PADS_PORT[n]_PDE | PORT[n] Pull-Down Enable |
| PADS_PORT[n]_PUE | PORT[n] Pull-Up Enable   |

## DAI0 1 to 10 pins DS control

Figure 13-36: PADS\_DAI0\_0\_DS Register Diagram

<!-- image -->

Table 13-43: PADS\_DAI0\_0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:27              | DAI10      | DS control for DAI10 IO.                                                                                                                                                                                                                                       |
| (R/W)              | DAI9       | 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010. To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid                    |
| 26:24 (R/W)        |            | DS control for DAI9 IO. 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- |

Table 13-43: PADS\_DAI0\_0\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |                                                                                                                                                                                                                                                                                        |
|--------------------|------------|---------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:21              | DAI8       | DS control for DAI8 IO.   | DS control for DAI8 IO.                                                                                                                                                                                                                                                                |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid                                                                                                                                                               |
| 17:15 (R/W)        | DAI6       | DS control for DAI6 IO. 1 | more than 62.5MHz. Drive Strength 000/1xx are inva- lid Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength are invalid Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid |
| 14:12              | DAI5       | 2                         | 000/1xx                                                                                                                                                                                                                                                                                |
| (R/W)              |            | DS control for DAI5 IO. 1 | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid                                                                                                                                                               |
| 11:9               |            | 2 DS control for DAI4 IO. | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid                                                                                                                                                                          |
| (R/W)              | DAI4       | 1 less                    | Drive strength 001. To be used for operating frequency than or equal to 62.5MHz. Drive Strength 000/1xx are invalid                                                                                                                                                                    |
|                    |            | 2                         |                                                                                                                                                                                                                                                                                        |
|                    |            |                           | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid                                                                                                                                                                          |

Table 13-43: PADS\_DAI0\_0\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   | Description/Enumeration                                                                                                  |
|--------------------|------------|---------------------------|--------------------------------------------------------------------------------------------------------------------------|
| 8:6                | DAI3       | DS control for DAI3 IO.   | DS control for DAI3 IO.                                                                                                  |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
| (R/W)              |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 5:3                | DAI2       | DS control for DAI2 IO.   | DS control for DAI2 IO.                                                                                                  |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
| (R/W)              |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 2:0                | DAI1       | DS control for DAI1 IO.   | DS control for DAI1 IO.                                                                                                  |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
| (R/W)              |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |

## DAI0 11 to 20 pins DS control

Figure 13-37: PADS\_DAI0\_1\_DS Register Diagram

<!-- image -->

Table 13-44: PADS\_DAI0\_1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:27              | DAI20      | DS control for DAI20 IO.                                                                                                                                                                                                                                            |
| (R/W)              | DAI19      | 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid DS control for DAI19 IO. |
| 26:24 (R/W)        |            | 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid                          |

Table 13-44: PADS\_DAI0\_1\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                   |                                                                                                                                                                                |
|--------------------|------------|-------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:21              | DAI18      | DS control for DAI18 IO.                  | DS control for DAI18 IO.                                                                                                                                                       |
| (R/W)              |            | 1                                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid                                                       |
| 17:15 (R/W)        | DAI16      | DS control for DAI16 IO. 1 Drive strength | 001. To be used for operating frequency                                                                                                                                        |
| 20:18 (R/W)        | DAI17      | DS control for DAI17 IO. 1                | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid                                                       |
| 14:12              |            | DS control for DAI15 IO.                  | DS control for DAI15 IO.                                                                                                                                                       |
| (R/W)              | DAI15      |                                           | are invalid 2 Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid                                                    |
| 11:9 (R/W)         | DAI14      |                                           |                                                                                                                                                                                |
|                    |            | 1 2                                       | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid Drive strength 010.To be used for operating frequency |
|                    |            | DS control for DAI14 IO.                  | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid                                                       |
|                    |            | 1                                         |                                                                                                                                                                                |
|                    |            | 2                                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid                                                                  |

Table 13-44: PADS\_DAI0\_1\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |                                                                                                                          |
|--------------------|------------|---------------------------|--------------------------------------------------------------------------------------------------------------------------|
| 8:6                | DAI13      | DS control for DAI13 IO.  | DS control for DAI13 IO.                                                                                                 |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
| (R/W)              |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 5:3                | DAI12      | DS control for DAI12 IO.  | DS control for DAI12 IO.                                                                                                 |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
| (R/W)              |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 2:0                | DAI11      | DS control for DAI11 IO.  | DS control for DAI11 IO.                                                                                                 |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
| (R/W)              |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |

## DAI0 Port Input Enable Control Register

The PADS\_DAI0\_IE register configures input enable control of the DAI0 (20 pins) pads. If =0 implies input buffer disable and if =1 implies enable.

Figure 13-38: PADS\_DAI0\_IE Register Diagram

<!-- image -->

Table 13-45: PADS\_DAI0\_IE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration    |
|--------------------|------------|----------------------------|
| 19:0               | VALUE      | DAI0 Input Enable Control. |
| (R/W)              |            |                            |

## DAI1 1 to 10 pins DS control

Figure 13-39: PADS\_DAI1\_0\_DS Register Diagram

<!-- image -->

Table 13-46: PADS\_DAI1\_0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:27              | DAI10      | DS control for DAI10 IO.                                                                                                                                                                                                                                           |
| (R/W)              | DAI9       | 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid DS control for DAI9 IO. |
| 26:24 (R/W)        |            | 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva-                             |

Table 13-46: PADS\_DAI1\_0\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   | Description/Enumeration                                                                                                  |
|--------------------|------------|---------------------------|--------------------------------------------------------------------------------------------------------------------------|
| 23:21              | DAI8       | DS control for DAI8 IO.   | DS control for DAI8 IO.                                                                                                  |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 20:18              | DAI7       | DS control for DAI7 IO.   | DS control for DAI7 IO.                                                                                                  |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx             |
|                    |            |                           | are invalid                                                                                                              |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 17:15              | DAI6       | DS control for DAI6 IO.   | DS control for DAI6 IO.                                                                                                  |
| (R/W) 14:12        | DAI5       | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are                      |
|                    |            |                           | inva- lid                                                                                                                |
| (R/W)              |            | 1 DS control for DAI4 IO. | Drive strength 001. To be used for operating frequency                                                                   |
| 11:9 (R/W)         | DAI4       | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |

Table 13-46: PADS\_DAI1\_0\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   | Description/Enumeration                                                                                                  |
|--------------------|------------|---------------------------|--------------------------------------------------------------------------------------------------------------------------|
| 8:6                | DAI3       | DS control for DAI3 IO.   | DS control for DAI3 IO.                                                                                                  |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
| (R/W)              |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 5:3                | DAI2       | DS control for DAI2 IO.   | DS control for DAI2 IO.                                                                                                  |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
| (R/W)              |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 2:0                | DAI1       | DS control for DAI1 IO.   | DS control for DAI1 IO.                                                                                                  |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
| (R/W)              |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |

## DAI1 11 to 20 pins DS control

Figure 13-40: PADS\_DAI1\_1\_DS Register Diagram

<!-- image -->

Table 13-47: PADS\_DAI1\_1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:27              | DAI20      | DS control for DAI20 IO.                                                                                                                                                                                                               |
|                    | DAI19      | 2 Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid DS control for DAI19 IO.                                                                                               |
| 26:24 (R/W)        |            | 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- |

Table 13-47: PADS\_DAI1\_1\_DS Register Fields (Continued)

| Bit No. (Access) Bit Name                 | Description/Enumeration                                                                                                                                                                                                                |
|-------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:21 DAI18 DS control for DAI18 IO.      |                                                                                                                                                                                                                                        |
| (R/W) 1                                   | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid                                                                                                               |
| 17:15 DAI16 DS control for DAI16 IO.      |                                                                                                                                                                                                                                        |
| (R/W) 1                                   | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid                                                                                                               |
| 14:12 DAI15 DS control for DAI15 IO.      |                                                                                                                                                                                                                                        |
| (R/W) 1 2 (R/W)                           | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid |
| 11:9 (R/W) DAI14 DS control for DAI14 IO. |                                                                                                                                                                                                                                        |
| 1 2                                       | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva-     |
|                                           | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength are invalid                                                                                                                       |
| 2                                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva-                                                                                                                              |
| lid                                       |                                                                                                                                                                                                                                        |
|                                           | 000/1xx                                                                                                                                                                                                                                |
| 1                                         |                                                                                                                                                                                                                                        |

Table 13-47: PADS\_DAI1\_1\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |                                                                                                                          |
|--------------------|------------|---------------------------|--------------------------------------------------------------------------------------------------------------------------|
| 8:6                | DAI13      | DS control for DAI13 IO.  | DS control for DAI13 IO.                                                                                                 |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
| (R/W)              |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 5:3                | DAI12      | DS control for DAI12 IO.  | DS control for DAI12 IO.                                                                                                 |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
| (R/W)              |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 2:0                | DAI11      | DS control for DAI11 IO.  | DS control for DAI11 IO.                                                                                                 |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
| (R/W)              |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |

## DAI1 Port Input Enable Control Register

The PADS\_DAI1\_IE register configures input enable control of the DAI1 (20 pins) pads. If =0 implies input buffer disable and if =1 implies enable.

Figure 13-41: PADS\_DAI1\_IE Register Diagram

<!-- image -->

Table 13-48: PADS\_DAI1\_IE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration    |
|--------------------|------------|----------------------------|
| 19:0               | VALUE      | DAI1 Input Enable Control. |
| (R/W)              |            |                            |

## DAI[n] Pull-Down Enable

Each bit of the PADS\_DAI[n]\_PDE register is mapped to a corresponding DAI pin pull-down enable.

Do not enable both the pull-up and pull-down at the same time for any pad. Check the Pull-Up Enable register to make sure there is no conflict.

Figure 13-42: PADS\_DAI[n]\_PDE Register Diagram

<!-- image -->

Table 13-49: PADS\_DAI[n]\_PDE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| 19:0 (R/W)         | PDE        | Pull-Down Enable. Set (=1) the PADS_DAI[n]_PDE.PDE bit to enable the pull-down. Clear (=0) the PADS_DAI[n]_PDE.PDE bit to disable the pull-down. |

## DAI[n] Pull-Up Enable

The PADS\_DAI[n]\_PUE register enables the pull-up resistor for DAI pins. Each bit of the PADS\_DAI[n]\_PUE register is mapped to the corresponding DAI pin pull-up enable.

Figure 13-43: PADS\_DAI[n]\_PUE Register Diagram

<!-- image -->

Table 13-50: PADS\_DAI[n]\_PUE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------|
| 19:0 (R/W)         | PUE        | Pull-Up Enable. Set (=1) the PADS_DAI[n]_PUE.PUE bit to enable the pull-up. Clear (=0) the PADS_DAI[n]_PUE.PUE bit to disable the pull-up. |

## Non-GPIO control Register

Figure 13-44: PADS\_NONPORTS\_CTL Register Diagram

<!-- image -->

Table 13-51: PADS\_NONPORTS\_CTL Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                             |
|--------------------|-------------|---------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | PDRESETOUTB | Pull down enable RESEOUTB. The PADS_NONPORTS_CTL.PDRESETOUTB bit enables (=1) and disables (=0) hysteresis on DAI1. |
| 2 (R/W)            | PURESETOUTB | Pull up enable RESEOUTB. The PADS_NONPORTS_CTL.PURESETOUTB bit enables (=1) and disables (=0) hysteresis on DAI1.   |
| 1 (R/W)            | PDCLKOUT    | Pull down enable CLKOUT. The PADS_NONPORTS_CTL.PDCLKOUT bit enables (=1) and disables (=0) hyste- resis on DAI1.    |
| 0 (R/W)            | PUCLKOUT    | Pull up enable CLKOUT. The PADS_NONPORTS_CTL.PUCLKOUT bit enables (=1) and disables (=0) hyste- resis on DAI0.      |

## Non-GPIO Drive Strength Register

The PADS\_NONPORTS\_DS register sets the drive strength and tolerance for the non-GPIO pins. The drive strength is only controlled for pin groups classified by the interface type.

Figure 13-45: PADS\_NONPORTS\_DS Register Diagram

<!-- image -->

Table 13-52: PADS\_NONPORTS\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:21 (R/W)        | EMACDCK    | EMAC Drive Strength Control. 1 Drive Strength 001 For an operating frequency less than                                                                                                                                              |
| 20:18              | FAULT      | 2 Drive Strength 010 For an operating frequency more than 62.5MHz. A drive strength of 000/1xx is invalid. FAULT Drive Strength Control.                                                                                            |
| (R/W)              |            | 1 Drive Strength 001 For an operating frequency less than or equal to 62.5MHz. A drive strength of 000/1xx is in- valid. 2 Drive Strength 010 For an operating frequency more than 62.5MHz. A drive strength of 000/1xx is invalid. |

Table 13-52: PADS\_NONPORTS\_DS Register Fields (Continued)

| Bit No. (Access) Bit Name           | Description/Enumeration                                                                                                                                                           | Description/Enumeration                                                                                                                                                           |
|-------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17:15 OSPI OSPI pins Drive Strength | Control.                                                                                                                                                                          |                                                                                                                                                                                   |
| (R/W)                               | 1                                                                                                                                                                                 |                                                                                                                                                                                   |
| 11:9 LP1CK LP1 Clock Drive Strength | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid Control.                                                 | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid Control.                                                 |
| (R/W)                               | less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid 1 | less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid 1 |
| 8:6 FAULTB FAULTB DS control.       |                                                                                                                                                                                   |                                                                                                                                                                                   |
| (R/W)                               | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid                                                          | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid                                                          |
| 5:3 RESOUTB RESOUTB DS control.     | less than or equal to 62.5MHz. Drive Strength are invalid                                                                                                                         | less than or equal to 62.5MHz. Drive Strength are invalid                                                                                                                         |
|                                     | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid                                                                     | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid                                                                     |
|                                     | Drive strength 001. To be used for operating frequency                                                                                                                            | Drive strength 001. To be used for operating frequency                                                                                                                            |
|                                     | more than 62.5MHz. Drive Strength 000/1xx are inva- lid                                                                                                                           | more than 62.5MHz. Drive Strength 000/1xx are inva- lid                                                                                                                           |
|                                     | 000/1xx                                                                                                                                                                           | 000/1xx                                                                                                                                                                           |
| 2                                   |                                                                                                                                                                                   |                                                                                                                                                                                   |
|                                     | 1                                                                                                                                                                                 |                                                                                                                                                                                   |

Table 13-52: PADS\_NONPORTS\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2:0                | CKOUT      | CLKOUT DS control.                                                                                                                                                                                                                       |
| (R/W)              | 2          | 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid |

## Peripheral PAD Configuration0 Register

The PADS\_PCFG0 register provides several configuration options for the pads and multiplexing for peripherals.

Figure 13-46: PADS\_PCFG0 Register Diagram

<!-- image -->

Table 13-53: PADS\_PCFG0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 18                 | FAULT_DIS  | FAULT does not exist.                                                                                          |
| 17 (R/W)           | EMACAUXIE  | Input enable control for ETH_PTPAUX_MCG_IN and ETH_PTPCLKIN pins. 0 Disable input                              |
| 16 (R/W)           | PUTMS      | Pull-Up Enable for TMS/SWDIO (debug port). 0 Disable pull-up                                                   |
| 7                  | CNT0DGSEL  | 1 Enable pull-up CNT0 Down Input Select.                                                                       |
| (R/W)              |            | The PADS_PCFG0 bit selects the input for the CNT0 counter down and gate. 0 GPIO port pins 1 Trigger connection |

Table 13-53: PADS\_PCFG0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                           |
|--------------------|-------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | CNT0UDSEL   | CNT0 Up Input Select. The PADS_PCFG0 bit selects the input for the CNT0 counter up and direction. 0 GPIO port pins                                                                                                                                                                |
| 4:3 (R/W)          | EMACPHYISEL | Select PHY Interface RGMII/RMII/MII. 0 MII Interface Selects PHY Interface MII                                                                                                                                                                                                    |
| 2 (R/W)            | EMACRESET   | Reset Enable for RGMII. The PADS_PCFG0.EMACRESET bit asserts the reset on the RGMII interface. To se- lect the PHY interface (RGMII or RMII), set the EMACPHYISEL bit as required and then set PADS_PCFG0.EMACRESET . 0 RGMII reset is asserted 1 RGMII reset reset is deasserted |
|                    | EMAC0       |                                                                                                                                                                                                                                                                                   |
| 1:0 (R/W)          |             | PTP Clock Source 0. The PADS_PCFG0.EMAC0 selects the clock source for the PTP Block in EMAC0. 0 REFCLK (MII or RMII) or CLKO7 (RGMII) 1 SCLK0 2 External PTP clock                                                                                                                |

## Peripheral Configuration1 Register

The PADS\_PCFG1 register provides bits to configure PWM secondary inputs to timers and enable inputs from the TRU.

Figure 13-47: PADS\_PCFG1 Register Diagram

<!-- image -->

Table 13-54: PADS\_PCFG1 Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                                   | Description/Enumeration                                                                                                                                   |
|--------------------|---------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25:16 (R/W)        | TIMER_SEC_INP | Secondary Input Select. The PADS_PCFG1.TIMER_SEC_INP field selects secondary inputs to timers. The enumerated bits below define the source for the timer. | Secondary Input Select. The PADS_PCFG1.TIMER_SEC_INP field selects secondary inputs to timers. The enumerated bits below define the source for the timer. |
|                    |               |                                                                                                                                                           | 1 TIMER0_CLK The TIMER0_CLK source is PB_01 when bit[0] is cleared (=0) The TIMER0_CLK source is PF_05 when this bit is set (=1)                          |
|                    |               |                                                                                                                                                           | 2 TIMER0_ACI0 The TIMER0_ACI0 source is PA_07 when bit[1] is cleared (=0) The TIMER0_ACI0 source is PD_08 when this bit is set (=1)                       |
|                    |               |                                                                                                                                                           | 4 TIMER0_ACI1 The TIMER0_ACI1 source is PA_14 when bit[2] is cleared (=0) The TIMER0_ACI1 source is PD_04 when this bit                                   |
|                    |               |                                                                                                                                                           | is set (=1)                                                                                                                                               |
|                    |               |                                                                                                                                                           | 8 TIMER0_ACI2 The TIMER0_ACI2 source is PB_11 when bit[3] is cleared (=0) The TIMER0_ACI2 source is PD_10 when this bit is                                |
|                    |               |                                                                                                                                                           | set (=1)                                                                                                                                                  |

Table 13-54: PADS\_PCFG1 Register Fields (Continued)

| Bit Name                                                                                                                                                                                                                                                                                                                                                                         | Description/Enumeration                                                   |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------|
| TIMER0_ACI3 The TIMER0_ACI3 cleared (=0) The TIMER0_ACI3 set (=1) TIMER0_ACI4 The TIMER0_ACI4 cleared (=0) The TIMER0_ACI4 set (=1) TIMER0_ACLK1 The TIMER0_ACLK1 cleared (=0) The TIMER0_ACLK1 set (=1) TIMER0_ACLK2 The TIMER0_ACLK2 cleared (=0) The TIMER0_ACLK2 set (=1) TIMER0_ACLK3 The TIMER0_ACLK3 cleared (=0) The TIMER0_ACLK3 set (=1) TIMER0_ACLK4 The TIMER0_ACLK4 | 16 source is PB_00 when bit[4] is source is PG_04 when this bit is        |
|                                                                                                                                                                                                                                                                                                                                                                                  | 32 source is PA_11 when bit[5] is source is PF_15 when this bit           |
|                                                                                                                                                                                                                                                                                                                                                                                  | is 64 source is PA_06 when bit[6] is source is PG_09 when this bit is     |
|                                                                                                                                                                                                                                                                                                                                                                                  | 128 source is PA_08 when bit[7] is source is PI_05 when this bit is       |
|                                                                                                                                                                                                                                                                                                                                                                                  | 256 source is PA_02 when bit[8] is source is PG_10 when this bit is       |
| cleared (=0) The set (=1)                                                                                                                                                                                                                                                                                                                                                        | source is PB_02 when bit[9] is TIMER0_ACLK4 source is PE_03 when this bit |
|                                                                                                                                                                                                                                                                                                                                                                                  | 512                                                                       |
|                                                                                                                                                                                                                                                                                                                                                                                  | is                                                                        |

## PORTA 0 to 7 pins DS control

Figure 13-48: PADS\_PORTA0\_DS Register Diagram

<!-- image -->

Table 13-55: PADS\_PORTA0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:21              | PA7        | DS control for PA7 IO.                                                                                                                                                                                                                     |
| (R/W)              | PA6        | 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid |
| 20:18 (R/W)        |            | DS control for PA6 IO. 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010.To be used for operating frequency                                  |

Table 13-55: PADS\_PORTA0\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |                                                                                                                          |
|--------------------|------------|---------------------------|--------------------------------------------------------------------------------------------------------------------------|
| 17:15              | PA5        | DS control for PA5 IO.    | DS control for PA5 IO.                                                                                                   |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 14:12              | PA4        | DS control for PA4 IO.    | DS control for PA4 IO.                                                                                                   |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva-                |
|                    |            |                           | lid                                                                                                                      |
| 11:9               | PA3        | DS control for PA3 IO.    | DS control for PA3 IO.                                                                                                   |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are                      |
|                    |            |                           | inva- lid                                                                                                                |
| 8:6                | PA2        | DS control for PA2 IO.    | DS control for PA2 IO.                                                                                                   |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |

Table 13-55: PADS\_PORTA0\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2:0                | PA0        | DS control for PA0 IO.                                                                                                                                                                                                                   |
| (R/W)              | 2          | 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid |

## PORTA 8 - 15 pins DS control

Figure 13-49: PADS\_PORTA1\_DS Register Diagram

<!-- image -->

Table 13-56: PADS\_PORTA1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:21              | PA15       | DS control for PA15 IO.                                                                                                                                                                                                                |
|                    | PA14       | 2 Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid DS control for PA14 IO.                                                                                                |
| 20:18 (R/W)        |            | 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- |

Table 13-56: PADS\_PORTA1\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   | Description/Enumeration                                                                                                  |
|--------------------|------------|---------------------------|--------------------------------------------------------------------------------------------------------------------------|
| 17:15              | PA13       | DS control for PA13 IO.   | DS control for PA13 IO.                                                                                                  |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 14:12              | PA12       | DS control for PA12 IO.   | DS control for PA12 IO.                                                                                                  |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx             |
|                    |            | 2                         | are invalid                                                                                                              |
|                    |            |                           | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 11:9               | PA11       | DS control for PA11 IO.   | DS control for PA11 IO.                                                                                                  |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 8:6                | PA10       | DS control for PA10 IO.   | DS control for PA10 IO.                                                                                                  |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx             |
|                    |            |                           | are invalid                                                                                                              |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 5:3                | PA9        | DS control for PA9 IO.    | DS control for PA9 IO.                                                                                                   |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx             |
|                    |            | 2                         | are invalid                                                                                                              |
|                    |            |                           | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |

Table 13-56: PADS\_PORTA1\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2:0                | PA8        | DS control for PA8 IO.                                                                                                                                                                                                                   |
| (R/W)              | 2          | 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid |

## PORTB 0 to 7 pins DS control

Figure 13-50: PADS\_PORTB0\_DS Register Diagram

<!-- image -->

Table 13-57: PADS\_PORTB0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:21              | PB7        | DS control for PB7 IO.                                                                                                                                                                                                                     |
| (R/W)              | PB6        | 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid |
| 20:18 (R/W)        |            | DS control for PB6 IO. 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010.To be used for operating frequency                                  |

Table 13-57: PADS\_PORTB0\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |                                                                                                                          |
|--------------------|------------|---------------------------|--------------------------------------------------------------------------------------------------------------------------|
| 17:15              | PB5        | DS control for PB5 IO.    | DS control for PB5 IO.                                                                                                   |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 14:12              | PB4        | DS control for PB4 IO.    | DS control for PB4 IO.                                                                                                   |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva-                |
|                    |            |                           | lid                                                                                                                      |
| 11:9               | PB3        | DS control for PB3 IO.    | DS control for PB3 IO.                                                                                                   |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are                      |
|                    |            |                           | inva- lid                                                                                                                |
| 8:6                | PB2        | DS control for PB2 IO.    | DS control for PB2 IO.                                                                                                   |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |

Table 13-57: PADS\_PORTB0\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2:0                | PB0        | DS control for PB0 IO.                                                                                                                                                                                                                     |
| (R/W)              |            | 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid |

## PORTB 8 - 15 pins DS control

Figure 13-51: PADS\_PORTB1\_DS Register Diagram

<!-- image -->

Table 13-58: PADS\_PORTB1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:21              | PB15       | DS control for PB15 IO.                                                                                                                                                            |
|                    | PB14       | 2 Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid DS control for PB14 IO.                                            |
| 20:18 (R/W)        |            | 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010.To be used for operating frequency |

Table 13-58: PADS\_PORTB1\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   | Description/Enumeration                                                                                                  |
|--------------------|------------|---------------------------|--------------------------------------------------------------------------------------------------------------------------|
| 17:15              | PB13       | DS control for PB13 IO.   | DS control for PB13 IO.                                                                                                  |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 14:12              | PB12       | DS control for PB12 IO.   | DS control for PB12 IO.                                                                                                  |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx             |
|                    |            | 2                         | are invalid                                                                                                              |
|                    |            |                           | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 11:9               | PB11       | DS control for PB11 IO.   | DS control for PB11 IO.                                                                                                  |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 8:6                | PB10       | DS control for PB10 IO.   | DS control for PB10 IO.                                                                                                  |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength                     |
|                    |            |                           | 000/1xx are invalid                                                                                                      |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are lid                  |
|                    |            |                           | inva-                                                                                                                    |
| 5:3                | PB9        | DS control for PB9 IO.    | DS control for PB9 IO.                                                                                                   |
| (R/W)              |            |                           | Drive strength 001. To be used for operating frequency                                                                   |
|                    |            |                           | less than or equal to 62.5MHz. Drive Strength are invalid                                                                |
|                    |            | 2                         | Drive strength 010.To be used for operating more than 62.5MHz. Drive Strength 000/1xx are                                |
|                    |            |                           | frequency                                                                                                                |
|                    |            |                           | inva- lid                                                                                                                |
|                    |            | 1                         |                                                                                                                          |
|                    |            |                           | 000/1xx                                                                                                                  |

Table 13-58: PADS\_PORTB1\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2:0                | PB8        | DS control for PB8 IO.                                                                                                                                                                                                                     |
| (R/W)              |            | 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid |

## PORTC 0 to 7 pins DS control

Figure 13-52: PADS\_PORTC0\_DS Register Diagram

<!-- image -->

Table 13-59: PADS\_PORTC0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:21              | PC7        | DS control for PC7 IO.                                                                                                                                                                                                                     |
| (R/W)              | PC6        | 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid |
| 20:18 (R/W)        |            | DS control for PC6 IO. 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010.To be used for operating frequency                                  |

Table 13-59: PADS\_PORTC0\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |                                                                                                                          |
|--------------------|------------|---------------------------|--------------------------------------------------------------------------------------------------------------------------|
| 17:15              | PC5        | DS control for PC5 IO.    | DS control for PC5 IO.                                                                                                   |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
| 14:12              | PC4        | DS control for PC4 IO.    | DS control for PC4 IO.                                                                                                   |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva-                |
|                    |            |                           | lid                                                                                                                      |
| 11:9               | PC3        | DS control for PC3 IO.    | DS control for PC3 IO.                                                                                                   |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are                      |
|                    |            |                           | inva- lid                                                                                                                |
| 8:6                | PC2        | DS control for PC2 IO.    | DS control for PC2 IO.                                                                                                   |
| (R/W)              |            | 1                         | Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |
|                    |            | 2                         | Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid            |

Table 13-59: PADS\_PORTC0\_DS Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2:0                | PC0        | DS control for PC0 IO.                                                                                                                                                                                                                     |
| (R/W)              |            | 1 Drive strength 001. To be used for operating frequency less than or equal to 62.5MHz. Drive Strength 000/1xx are invalid 2 Drive strength 010.To be used for operating frequency more than 62.5MHz. Drive Strength 000/1xx are inva- lid |

## PORTC 8 - 15 pins DS control

Figure 13-53: PADS\_PORTC1\_DS Register Diagram

<!-- image -->

Table 13-60: PADS\_PORTC1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 23:21 (R/W)        | PC15       | DS control for PC15 IO.   |
| 20:18 (R/W)        | PC14       | DS control for PC14 IO.   |
| 17:15 (R/W)        | PC13       | DS control for PC13 IO.   |
| 14:12 (R/W)        | PC12       | DS control for PC12 IO.   |
| 11:9 (R/W)         | PC11       | DS control for PC11 IO.   |
| 8:6 (R/W)          | PC10       | DS control for PC10 IO.   |
| 5:3 (R/W)          | PC9        | DS control for PC9 IO.    |
| 2:0 (R/W)          | PC8        | DS control for PC8 IO.    |

## PORTD 0 to 7 pins DS control

Figure 13-54: PADS\_PORTD0\_DS Register Diagram

<!-- image -->

Table 13-61: PADS\_PORTD0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 23:21 (R/W)        | PD7        | DS control for PD7 IO.    |
| 20:18 (R/W)        | PD6        | DS control for PD6 IO.    |
| 17:15 (R/W)        | PD5        | DS control for PD5 IO.    |
| 14:12 (R/W)        | PD4        | DS control for PD4 IO.    |
| 11:9 (R/W)         | PD3        | DS control for PD3 IO.    |
| 8:6 (R/W)          | PD2        | DS control for PD2 IO.    |
| 5:3 (R/W)          | PD1        | DS control for PD1 IO.    |
| 2:0 (R/W)          | PD0        | DS control for PD0 IO.    |

## PORTD 8 to 15 pins DS control

Figure 13-55: PADS\_PORTD1\_DS Register Diagram

<!-- image -->

Table 13-62: PADS\_PORTD1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 23:21 (R/W)        | PD15       | DS control for PD15 IO.   |
| 20:18 (R/W)        | PD14       | DS control for PD14 IO.   |
| 17:15 (R/W)        | PD13       | DS control for PD13 IO.   |
| 14:12 (R/W)        | PD12       | DS control for PD12 IO.   |
| 11:9 (R/W)         | PD11       | DS control for PD11 IO.   |
| 8:6 (R/W)          | PD10       | DS control for PD10 IO.   |
| 5:3 (R/W)          | PD9        | DS control for PD9 IO.    |
| 2:0 (R/W)          | PD8        | DS control for PD8 IO.    |

## PORTE 0 to 7 pins DS control

Figure 13-56: PADS\_PORTE0\_DS Register Diagram

<!-- image -->

Table 13-63: PADS\_PORTE0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 23:21 (R/W)        | PE7        | DS control for PE7 IO.    |
| 20:18 (R/W)        | PE6        | DS control for PE6 IO.    |
| 17:15 (R/W)        | PE5        | DS control for PE5 IO.    |
| 14:12 (R/W)        | PE4        | DS control for PE4 IO.    |
| 11:9 (R/W)         | PE3        | DS control for PE3 IO.    |
| 8:6 (R/W)          | PE2        | DS control for PE2 IO.    |
| 5:3 (R/W)          | PE1        | DS control for PE1 IO.    |
| 2:0 (R/W)          | PE0        | DS control for PE0 IO.    |

## PORTE 8 to 15 pins DS control

Figure 13-57: PADS\_PORTE1\_DS Register Diagram

<!-- image -->

Table 13-64: PADS\_PORTE1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 23:21 (R/W)        | PE15       | DS control for PE15 IO.   |
| 20:18 (R/W)        | PE14       | DS control for PE14 IO.   |
| 17:15 (R/W)        | PE13       | DS control for PE13 IO.   |
| 14:12 (R/W)        | PE12       | DS control for PE12 IO.   |
| 11:9 (R/W)         | PE11       | DS control for PE11 IO.   |
| 8:6 (R/W)          | PE10       | DS control for PE10 IO.   |
| 5:3 (R/W)          | PE9        | DS control for PE9 IO.    |
| 2:0 (R/W)          | PE8        | DS control for PE8 IO.    |

## PORTF 0 to 7 pins DS control

Figure 13-58: PADS\_PORTF0\_DS Register Diagram

<!-- image -->

Table 13-65: PADS\_PORTF0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 23:21 (R/W)        | PF7        | DS control for PF7 IO.    |
| 20:18 (R/W)        | PF6        | DS control for PF6 IO.    |
| 17:15 (R/W)        | PF5        | DS control for PF5 IO.    |
| 14:12 (R/W)        | PF4        | DS control for PF4 IO.    |
| 11:9 (R/W)         | PF3        | DS control for PF3 IO.    |
| 8:6 (R/W)          | PF2        | DS control for PF2 IO.    |
| 5:3 (R/W)          | PF1        | DS control for PF1 IO.    |
| 2:0 (R/W)          | PF0        | DS control for PF0 IO.    |

## PORTF 8 to 15 pins DS control

Figure 13-59: PADS\_PORTF1\_DS Register Diagram

<!-- image -->

Table 13-66: PADS\_PORTF1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 23:21 (R/W)        | PF15       | DS control for PF15 IO.   |
| 20:18 (R/W)        | PF14       | DS control for PF14 IO.   |
| 17:15 (R/W)        | PF13       | DS control for PF13 IO.   |
| 14:12 (R/W)        | PF12       | DS control for PF12 IO.   |
| 11:9 (R/W)         | PF11       | DS control for PF11 IO.   |
| 8:6 (R/W)          | PF10       | DS control for PF10 IO.   |
| 5:3 (R/W)          | PF9        | DS control for PF9 IO.    |
| 2:0 (R/W)          | PF8        | DS control for PF8 IO.    |

## PORTG 0 to 7 pins DS control

Figure 13-60: PADS\_PORTG0\_DS Register Diagram

<!-- image -->

Table 13-67: PADS\_PORTG0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 23:21 (R/W)        | PG7        | DS control for PG7 IO.    |
| 20:18 (R/W)        | PG6        | DS control for PG6 IO.    |
| 17:15 (R/W)        | PG5        | DS control for PG5 IO.    |
| 14:12 (R/W)        | PG4        | DS control for PG4 IO.    |
| 11:9 (R/W)         | PG3        | DS control for PG3 IO.    |
| 8:6 (R/W)          | PG2        | DS control for PG2 IO.    |
| 5:3 (R/W)          | PG1        | DS control for PG1 IO.    |
| 2:0 (R/W)          | PG0        | DS control for PG0 IO.    |

## PORTG 8 to 15 pins DS control

Figure 13-61: PADS\_PORTG1\_DS Register Diagram

<!-- image -->

Table 13-68: PADS\_PORTG1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 23:21 (R/W)        | PG15       | DS control for PG15 IO.   |
| 20:18 (R/W)        | PG14       | DS control for PG14 IO.   |
| 17:15 (R/W)        | PG13       | DS control for PG13 IO.   |
| 14:12 (R/W)        | PG12       | DS control for PG12 IO.   |
| 11:9 (R/W)         | PG11       | DS control for PG11 IO.   |
| 8:6 (R/W)          | PG10       | DS control for PG10 IO.   |
| 5:3 (R/W)          | PG9        | DS control for PG9 IO.    |
| 2:0 (R/W)          | PG8        | DS control for PG8 IO.    |

## PORTH 0 to 7 pins DS control

Figure 13-62: PADS\_PORTH0\_DS Register Diagram

<!-- image -->

Table 13-69: PADS\_PORTH0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 23:21 (R/W)        | PH7        | DS control for PH7 IO.    |
| 20:18 (R/W)        | PH6        | DS control for PH6 IO.    |
| 17:15 (R/W)        | PH5        | DS control for PH5 IO.    |
| 14:12 (R/W)        | PH4        | DS control for PH4 IO.    |
| 11:9 (R/W)         | PH3        | DS control for PH3 IO.    |
| 8:6 (R/W)          | PH2        | DS control for PH2 IO.    |
| 5:3 (R/W)          | PH1        | DS control for PH1 IO.    |
| 2:0 (R/W)          | PH0        | DS control for PH0 IO.    |

## PORTH 8 to 15 pins DS control

Figure 13-63: PADS\_PORTH1\_DS Register Diagram

<!-- image -->

Table 13-70: PADS\_PORTH1\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 23:21 (R/W)        | PH15       | DS control for PH15 IO.   |
| 20:18 (R/W)        | PH14       | DS control for PH14 IO.   |
| 17:15 (R/W)        | PH13       | DS control for PH13 IO.   |
| 14:12 (R/W)        | PH12       | DS control for PH12 IO.   |
| 11:9 (R/W)         | PH11       | DS control for PH11 IO.   |
| 8:6 (R/W)          | PH10       | DS control for PH10 IO.   |
| 5:3 (R/W)          | PH9        | DS control for PH9 IO.    |
| 2:0 (R/W)          | PH8        | DS control for PH8 IO.    |

## PORTI 0 to 7 pins DS control

Figure 13-64: PADS\_PORTI0\_DS Register Diagram

<!-- image -->

Table 13-71: PADS\_PORTI0\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 23:21 (R/W)        | PI7        | DS control for PI7 IO.    |
| 20:18 (R/W)        | PI6        | DS control for PI6 IO.    |
| 17:15 (R/W)        | PI5        | DS control for PI5 IO.    |
| 14:12 (R/W)        | PI4        | DS control for PI4 IO.    |
| 11:9 (R/W)         | PI3        | DS control for PI3 IO.    |
| 8:6 (R/W)          | PI2        | DS control for PI2 IO.    |
| 5:3 (R/W)          | PI1        | DS control for PI1 IO.    |
| 2:0 (R/W)          | PI0        | DS control for PI0 IO.    |

## PORT[n] Pull-Down Enable

Each bit of the PADS\_PORT[n]\_PDE register is mapped to a corresponding pull-down enable.

Do not enable both the pull-up and pull-down at the same time for any pad. Check the Pull-Up Enable register to make sure there is no conflict.

Figure 13-65: PADS\_PORT[n]\_PDE Register Diagram

<!-- image -->

Table 13-72: PADS\_PORT[n]\_PDE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | PDE        | Pull-Down Enable. Set (=1) the PADS_PORT[n]_PDE.PDE bit to enable the pull-down. Clear (=0) the PADS_PORT[n]_PDE.PDE bit to disable the pull-down. |

## PORT[n] Pull-Up Enable

Each bit of the PADS\_PORT[n]\_PUE register is mapped to a corresponding pull-up enable.

Do not enable both the pull-up and pull-down at the same time for any pad. Check the Pull-Down Enable register to make sure there is no conflict.

Figure 13-66: PADS\_PORT[n]\_PUE Register Diagram

<!-- image -->

Table 13-73: PADS\_PORT[n]\_PUE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | PUE        | Pull-Up Enable. Set (=1) the PADS_PORT[n]_PUE.PUE bit to enable the pull-up. Clear (=0) the PADS_PORT[n]_PUE.PUE bit to disable the pull-up. |