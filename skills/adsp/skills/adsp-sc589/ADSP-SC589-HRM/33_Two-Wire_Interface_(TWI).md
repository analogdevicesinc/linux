## 30   Two-Wire Interface (TWI)

The processor has a two-wire interface (TWI), that provides a simple exchange method of control data between multiple devices. The TWI module is compatible with the widely used I 2 C bus standard. Additionally, the TWI module is fully compatible with serial camera control bus (SCCB) functionality for easier control of various CMOS camera sensor devices.

The TWI module offers the capabilities of simultaneous master and slave operation and support for both 7-bit addressing and multimedia data arbitration. The TWI interface uses two pins for transferring clock ( TWI\_SCL ) and data ( TWI\_SDA ) and supports the protocol at speeds up to 400K bits/sec. The TWI interface pins are compatible with 5-V logic levels.

To preserve processor bandwidth, the TWI module can be set up with transfer-initiated interrupts to only service FIFO buffer data reads and writes. Protocol-related interrupts are optional. The TWI externally moves 8-bit data while maintaining compliance with the I 2 C bus protocol.

## TWI Features

The TWI is fully compatible with the widely used I 2 C bus standard.

The TWI controller includes the following features.

- Simultaneous master and slave operation on multiple device systems
- Support for multi-master bus arbitration
- 7-bit addressing
- 100K bits/second and 400K bits/second data rates
- General call address support
- Master clock synchronization and support for clock low extension
- Separate multiple-byte receive and transmit FIFOs
- Low interrupt rate
- Individual override control of data and clock lines in the event of bus lock-up
- Input filter for spike suppression

- Serial camera control bus support as specified in the OmniVision Serial Camera Control Bus (SCCB) Functional Specification

## TWI Functional Description

The TWI interface is a shift register that serially transmits and receives data bits. It moves data 1 bit at a time at the SCL rate, to and from other TWI devices. The SCL signal synchronizes the shifting and sampling of the data on the serial data pin.

## ADSP-SC58x TWI Register List

The Two-Wire Interface controller TWI allows a device to interface to an inter-IC bus as specified by the Philips I 2 C Bus Specification version 2.1, dated January 2000. A set of registers governs TWI operations. For more information on TWI functionality, see the TWI register descriptions.

Table 30-1: ADSP-SC58x TWI Register List

| Name         | Description                   |
|--------------|-------------------------------|
| TWI_CLKDIV   | SCL Clock Divider Register    |
| TWI_CTL      | Control Register              |
| TWI_FIFOCTL  | FIFO Control Register         |
| TWI_FIFOSTAT | FIFO Status Register          |
| TWI_IMSK     | Interrupt Mask Register       |
| TWI_ISTAT    | Interrupt Status Register     |
| TWI_MSTRADDR | Master Mode Address Register  |
| TWI_MSTRCTL  | Master Mode Control Registers |
| TWI_MSTRSTAT | Master Mode Status Register   |
| TWI_RXDATA16 | Rx Data Double-Byte Register  |
| TWI_RXDATA8  | Rx Data Single-Byte Register  |
| TWI_SLVADDR  | Slave Mode Address Register   |
| TWI_SLVCTL   | Slave Mode Control Register   |
| TWI_SLVSTAT  | Slave Mode Status Register    |
| TWI_TXDATA16 | Tx Data Double-Byte Register  |
| TWI_TXDATA8  | Tx Data Single-Byte Register  |

## ADSP-SC58x TWI Interrupt List

Table 30-2: ADSP-SC58x TWI Interrupt List

|   Interrupt ID | Name      | Description         | Sensitivity   | DMA Channel   |
|----------------|-----------|---------------------|---------------|---------------|
|            123 | TWI0_DATA | TWI0 Data Interrupt | Level         |               |
|            124 | TWI1_DATA | TWI1 Data Interrupt | Level         |               |
|            125 | TWI2_DATA | TWI2 Data Interrupt | Level         |               |

## TWI Block Diagram

The TWI Block Diagram figure shows the basic blocks of the TWI interface.

Figure 30-1: TWI Block Diagram

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000000_4cc0385c133695384d7775cfbab661f5f8e3e81edb5638d6f3efed8d2ec069da.png)

## External Interface

The TWI\_SDA (serial data) and TWI\_SCL (serial clock) signals are open drain and require pull-up resistors. These bidirectional signals externally interface the TWI controller to the I 2 C bus and no other external connections or logic are necessary.

## Serial Clock Signal (SCL)

The serial clock signal ( TWI\_SCL ) is an input in slave mode. In master mode, the TWI controller must set this signal to the desired frequency.

The TWI controller supports the standard mode of operation (up to 100 kHz) or fast mode (up to 400 kHz). The TWI control register ( TWI\_CTL ) sets the TWI\_CTL.PRESCALE value which sets the relationship between the

system clock (SCLK0\_0) and the internally timed events of the TWI controller. The internal time reference is derived from SCLK0\_0 using a prescaled value. The prescale value is the number of SCLK0\_0 periods used in the generation of one internal time reference. Set the value of prescale to create an internal time reference with a period of 10 MHz. It is represented as a 7-bit binary value as follows.

PRESCALE = f SCLK0\_0/10MHz

NOTE: It is not always possible to achieve 10-MHz accuracy. In such cases, it is safe to round up the PRESCALE value to the next highest integer. For example, if SCLK0\_0 is 100 MHz, the PRESCALE value is calculated as 100 MHz/10 MHz = 10. A prescale value of 14 in this case ensures that all timing requirements are met.

During master mode operation, the TWI module uses the TWI\_CLKDIV register values to create the minimum TWI\_CLKDIV.CLKHI and TWI\_CLKDIV.CLKLO durations of the TWI\_SCL signal. The TWI\_CLKDIV.CLKHI field specifies the minimum number of 10-MHz time reference periods the TWI\_SCL waits before a new clock low period begins, assuming a single master. (The 10-MHz time reference periods are represented as an 8-bit binary value). The TWI uses the TWI\_CLKDIV.CLKLO field to specify the minimum number of internal time reference periods (represented as an 8-bit binary value). The TWI\_SCL signal is held low.

Serial clock frequencies can vary from 400 kHz to less than 20 kHz. The resolution of the clock generated is 1/10 MHz or 100 ns. The following equation describes the frequency.

TWI\_CLKDIV = TWI\_SCL period/10 MHz time reference.

For example, for an TWI\_SCL of 400 kHz (period = 1/400 kHz = 2500 ns) and an internal time reference of 10 MHz (period = 100 ns), the following equation applies:

```
TWI_CLKDIV = 2500 ns/100 ns = 25
```

```
Therefore, a TWI_SCL signal with a 30% duty cycle has TWI_CLKDIV.CLKLO =17 and TWI_CLKDIV.CLKHI =8. Adding TWI_CLKDIV.CLKLO and TWI_CLKDIV.CLKHI equals TWI_CLKDIV .
```

NOTE: The TWI\_CLKDIV.CLKHI and TWI\_CLKDIV.CLKLO fields are not intended to guarantee a certain frequency. Rather, they guarantee a certain minimum high and low duration for the TWI\_SCL signal. Slew rate controls falling edges. The RC time constant governs the rising edges. The pull-up resistor and the TWI\_SCL capacitance form the time constant. See the 'Register Descriptions' section for more details.

## Serial Data Signal (SDA)

The TWI transmits and receives serial data, depending on the direction of the transfer, on the bidirectional serial data signal (SDA).

## Internal Interface

The peripheral bus interface supports the transfer of 16-bit wide data. The processor uses the interface in the support of register and FIFO buffer reads and writes. The TWI internal interface is comprised of the blocks described as follows.

Register block. Contains all control and status bits and reflects what can be written or read as outlined by the programming model. Each function block updates their corresponding status bits.

FIFO buffer. Configured as a 1-byte-wide, 2-deep transmit FIFO buffer and a 1-byte-wide, 2-deep receive FIFO buffer.

Transmit shift register. Serially shifts its data out externally off chip. The output can be controlled for generation of acknowledgments or it can be manually overwritten.

Receive shift register. Receives its data serially from off chip. The receive shift register is 1 byte wide and data received can either be transferred to the FIFO buffer or used in an address comparison.

Address compare block. Supports address comparison in the event the TWI controller module is accessed as a slave.

Prescaler block. Must be programmed to generate a 10-MHz time reference relative to the system clock. The block uses this time base for filtering of data and timing events specified by the electrical data sheet (See the Philips specification). The block uses the time base to generate the TWI\_SCL clock as well.

Clock generation module. Generates an external TWI\_SCL clock when in master mode. It includes the logic necessary for synchronization in a multi-master clock configuration and clock stretching when configured in slave mode.

NOTE: The TWI does not support DMA based operation.

## TWI Architectural Concepts

The TWI controller follows the transfer protocol of the Philips I 2 C Bus specification version 2.1 dated January 2000.

NOTE: The TWI unit does not support DMA-based operation.

## TWI Protocol

The Data Transfer figure shows a simple complete transfer.

Figure 30-2: Data Transfer

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000001_dff97b333ae304273d936562e82c217668a4d822a92f33e8cdb369cad398806a.png)

The TWI controller register contents maps to a basic transfer. The Data Transfer with Bit Illustration figure details the same transfer from the Data Transfer figure noting the corresponding TWI controller bit names. In this illustration, the TWI controller successfully transmits 1 byte of data. The slave has acknowledged both address and data.

Figure 30-3: Data Transfer with Bit Illustration

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000002_ccede948405d5f8719f4b2d6bb95a5321a6141961540674415378e8d74336527.png)

## Clock Generation and Synchronization

The TWI controller implementation only issues a clock during master mode operation and only at the time a transfer initiates. If arbitration for the bus is lost, the serial clock output immediately three-states. If multiple clocks attempt to drive the serial clock line, the TWI controller synchronizes its clock with the other remaining clocks. The Clock Synchronization figure shows this functionality.

Figure 30-4: Clock Synchronization

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000003_db574ed3727c4aa6593e0fafb7bfb5d53aeba25c525f1d8cb398cbfd41587e95.png)

The TWI controller serial clock ( TWI\_SCL ) output follows these rules:

- Once the clock high ( TWI\_CLKDIV.CLKHI ) count is complete, the serial clock output is driven low and the clock low ( TWI\_CLKDIV.CLKLO ) count begins.
- Once the clock low count is complete, the serial clock line is three-stated. This state allows the external pull-up resistor to pull the TWI\_SCL signal high. The clock synchronization logic enters into a delay mode (shaded area) until the TWI\_SCL signal is detected at logic 1 level. Now, the clock high count begins.

## Bus Arbitration

The TWI controller initiates a master mode transmission only when the bus is idle. If the bus is idle and two masters initiate a transfer, arbitration for the bus begins. The Bus Arbitration figure shows the arbitration.

Figure 30-5: Bus Arbitration

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000004_21eb0552bc1dd01fc543d2d19ce0930f6a8d01bb90272f8f9fec5d5f67dfc71f.png)

The TWI controller monitors the serial data bus (SDA) while the TWI\_SCL signal is high. If the TWI\_SDA signal is determined to be an active logic 0 level while the data of the TWI controller is a logic 1 level, the TWI controller has lost arbitration. It stops generating the clock and data signals. Arbitration is not only performed at the serial clock edges, but also during the entire time the TWI\_SCL signal is high.

## Start and Stop Conditions

Start and stop conditions involve serial data transitions while the serial clock is a logic 1 level. The TWI controller generates and recognizes these transitions. Typically, start and stop conditions occur at the beginning and at the conclusion of a transmission, except repeated start combined transfers. The Start and Stop Conditions figure shows the transitions.

Figure 30-6: Start and Stop Conditions

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000005_f140a4565b96f61e9634db92587b22f8232e2bb9f8e46eea15512bc63a7054af.png)

The TWI special case start and stop conditions of the TWI controller include the following.

- Controller addressed as a slave-receiver. If the master asserts a stop condition during the data phase of a transfer, the TWI controller concludes the transfer ( TWI\_ISTAT.SCOMP ).
- Controller addressed as a slave-transmitter. If the master asserts a stop condition during the data phase of a transfer, the TWI controller concludes the transfer ( TWI\_ISTAT.SCOMP ) and indicates a slave transfer error ( TWI\_ISTAT.SERR ).
- Controller as a master-transmitter or master-receiver. If the stop bit ( TWI\_MSTRCTL.STOP ) is set during an active master transfer, the TWI controller issues a stop condition as soon as possible avoiding any error conditions. The TWI controller operates as if data transfer count had been reached.

## General Call Support

The TWI controller always decodes and acknowledges a general call address if:

- The TWI controller is enabled as a slave
- General call is enabled

The TWI\_SLVCTL.GEN bit configures general call addressing (0x00) only when the TWI controller is a slavereceiver.

If the data associated with the transfer is (NAK) not acknowledged, the TWI\_SLVCTL.NAK bit can be set. If the TWI controller issues a general call as a master-transmitter, set the appropriate address ( TWI\_MSTRADDR register) and transfer direction ( TWI\_MSTRCTL.DIR bit) and load the transmit FIFO data.

NOTE: The byte following the general call address usually defines the slaves response to the call. The interpretation of the command in the second byte is based on the value of its LSB. For a TWI slave device, the bytes received after the general call address are considered data.

## Fast Mode

Fast mode essentially uses the same mechanics as the standard mode of operation. Fast mode affects electrical specifications and timing. When fast mode is enabled, (FAST) timing is modified to meet the following electrical requirements.

- Serial data rise times before arbitration evaluation (t r )
- Stop condition set-up time from serial clock to serial data (t SUSTO )
- Bus free time between a stop and start condition (t BUF )

## TWI Operating Modes

The TWI has two modes of operation: repeated start and clock stretching. The following sections describe the operating modes.

## Repeated Start

A repeated start condition is the absence of a stop condition between two transfers. The two transfers can be of any direction type. Examples include a transmit followed by a receive, or a receive followed by a transmit. The following sections guide the programmer in developing a service routine.

## Transmit Receive Repeated Start

The Repeated Start Followed by Data Receive figure shows a repeated start followed by a data receive sequence. The shading in the figure indicates that the slave has control of the bus.

Figure 30-7: Repeated Start Followed by Data Receive

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000006_e3c128c636af97fea16f30ba9f01bbb33a0073f1ba32ee47156e4feabf99cefe.png)

The tasks performed at each interrupt are:

- Transmit FIFO service ( TWI\_ISTAT.TXSERV ) interrupt request. This interrupt is generated due to a FIFO access. Since this byte is the last of this transfer, the TWI uses the TWI\_FIFOSTAT register to indicate that the transmit FIFO is empty. When read, TWI\_MSTRCTL.DCNT bit field=0. Set the TWI\_MSTRCTL.RSTART bit to indicate a repeated start and set the TWI\_MSTRCTL.DIR bit if the following transfer is a data receive.
- Master transfer complete ( TWI\_ISTAT.MCOMP ) interrupt. This interrupt request is generated when all data transfers ( TWI\_MSTRCTL.DCNT bit field=0). If no errors occur, a start condition initiates. Clear the TWI\_MSTRCTL.RSTART bit and program the TWI\_MSTRCTL.DCNT bits with the desired number of bytes to receive.

- Receive FIFO service ( TWI\_ISTAT.RXSERV ) interrupt. This interrupt request is generated due to the arrival of a byte in the receive FIFO. Simple data handling is the only requirement.
- Master transfer complete ( TWI\_ISTAT.MCOMP ) interrupt. This interrupt request is generated when the transfer completes.

## Receive Transmit Repeated Start

The Repeated Start Data Receive Followed by Data Transmit figure illustrates a repeated start data receive followed by a data transmit sequence. The shading in the figure indicates that the slave has control of the bus.

Figure 30-8: Repeated Start Data Receive Followed by Data Transmit

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000007_75fb724a6699c88e56eb9b482de86834fc07b36737c4d9fb1f80906a600d9f99.png)

The tasks performed at each interrupt are:

- Receive FIFO service ( TWI\_ISTAT.RXSERV ) interrupt. This interrupt request is generated due to the arrival of a data byte in the receive FIFO. Set the TWI\_MSTRCTL.RSTART bit to indicate a repeated start and clear the TWI\_MSTRCTL.DIR bit if the following transfer is a data transmit.
- Master transfer complete ( TWI\_ISTAT.MCOMP ) interrupt. This interrupt request has occurred due to the completion of the data receive transfer. If no errors occur, a start condition initiates. Clear the TWI\_MSTRCTL.RSTART bit and program the TWI\_MSTRCTL.DCNT bits with the desired number of bytes to transmit.
- Transmit FIFO service ( TWI\_ISTAT.TXSERV ) interrupt. This interrupt request is generated due to a FIFO access. Simple data handling is the only requirement.
- Master transfer complete ( TWI\_ISTAT.MCOMP ) interrupt. This interrupt request is generated when the transfer completes.

NOTE: There is no timing constraint to meet the conditions-program the bits as required. Refer to Clock Stretching During Repeated Start section for more on how the controller stretches the clock during repeated start transfers.

## Clock Stretching

Clock stretching is an added function of the TWI controller in master mode operation. This behavior uses self-induced stretching of the I 2 C clock while waiting to service interrupts. Hardware initiates stretching automatically. No programming is necessary. The TWI controller as a master supports three modes of clock stretching:

- Clock Stretching During FIFO Underflow
- Clock Stretching During FIFO Overflow

## · Clock Stretching During Repeated Start

## Clock Stretching During FIFO Underflow

During a master mode transmit, an interrupt request occurs the instant the transmit FIFO becomes empty. The most recent byte begins transmission. If the TWI\_ISTAT.TXSERV interrupt request is not serviced, the concluding acknowledge phase of the transfer stretches.

Stretching of the clock continues until new data bytes are written to the transmit FIFO ( TWI\_TXDATA8 or TWI\_TXDATA16 registers). No other action is required to release the clock and continue the transmission. This behavior continues until the transmission completes ( TWI\_MSTRCTL.DCNT =0). The transmission concludes ( TWI\_ISTAT.MCOMP ). The Clock Stretching during FIFO Underflow figure and table show the stretching.

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000008_bd2ac45bbd81bbe67bf5c392ad011a66c8374052c39ed505d8c7a29c0264c3c5.png)

ACKNOWLEDGE "STRETCH" BEGINS SOON AFTER SCL FALL.

Figure 30-9: Clock Stretching during FIFO Underflow

| TWI Controller                                          | Processor                                                                                |
|---------------------------------------------------------|------------------------------------------------------------------------------------------|
| Interrupt: XMTSERV - Transmit FIFO buffer is empty.     | Acknowledge: Clear the interrupt request source bits. Write to the transmit FIFO buffer. |
| ...                                                     | ...                                                                                      |
| Interrupt: MCOMP-Master transmit complete (DCNT= 0x00). | Acknowledge: Clear the interrupt request source bits.                                    |

## Clock Stretching During FIFO Overflow

During a master mode receive operation, an interrupt occurs at the instant the receive FIFO becomes full. It is during the acknowledge phase of this received byte that clock stretching begins. The TWI module makes no attempt to initiate the reception of another byte. Stretching of the clock continues until the data bytes previously received are read from the receive FIFO buffer ( TWI\_RXDATA8 or TWI\_RXDATA16 registers). No other action is required to release the clock and continue the reception of data. This behavior continues until the reception is complete ( TWI\_MSTRCTL.DCNT =0). Reception concludes ( TWI\_ISTAT.MCOMP ). The Clock Stretching During FIFO Overflow figure and table show the clock stretching.

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000009_b056be8fdb85b5ac823c7c5030bfec5452d552a5e3a885be406513e7cc456850.png)

Figure 30-10: Clock Stretching During FIFO Overflow

| TWI Controller                                    | Processor                                                                             |
|---------------------------------------------------|---------------------------------------------------------------------------------------|
| Interrupt: RCVSERV - Receive FIFO buffer is full. | Acknowledge: Clear the interrupt request source bits. Read the re- ceive FIFO buffer. |
| ...                                               | ...                                                                                   |
| Acknowledge: Clear the interrupt source bits.     | Interrupt: MCOMP-Master receive complete.                                             |

## Clock Stretching During Repeated Start

The repeated start feature in I 2 C protocol requires a transition between two subsequent transfers. With the use of clock stretching, the task of managing transitions becomes simpler and common to all transfer types.

Once an initial TWI master transfer completes (transmit or receive), the clock initiates a stretch during the repeated start phase between transfers. Concurrent with this event, the initial transfer generates a TWI\_ISTAT.MCOMP interrupt to signify the initial transfer has completed ( TWI\_MSTRCTL.DCNT =0). This initial transfer is handled without any special bit setting sequences or timing.

The clock stretching logic described applies here. With no system-related timing constraints, the subsequent transfer (receive or transmit) is set up and activated. This sequence can repeat as many times as required to string a series of repeated start transfers together. The Clock Stretching during Repeated Start Condition figure and table show the clock stretching.

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000010_ea045d53ea70cb2910534bd175a48ae2d65fd3735b3c575dd0598581e059e3d9.png)

DUE TO DCNT=0X00 AND RSTART.

Figure 30-11: Clock Stretching during Repeated Start Condition

| TWI Controller                                                                                                      | Processor                                                                                                                                                      |
|---------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Interrupt: MCOMP-Initial transmit has completed and DCNT = 0x00. Note: transfer in progress, RSTART previously set. | Acknowledge: Clear the interrupt request source bits. Write to TWIx_MASTER_CTL, setting MDIR (receive), clearing RSTART, and setting new DCNT value (nonzero). |
| Interrupt: RCVSERV - Receive FIFO is full.                                                                          | Acknowledge: Clear the interrupt request source bits. Read the re- ceive FIFO buffer.                                                                          |
| ...                                                                                                                 | ...                                                                                                                                                            |
| Interrupt: MCOMP-Master receive complete                                                                            | Acknowledge: Clear the interrupt request source bits.                                                                                                          |

## TWI Programming Model

The topics in this section provide information on the basic programming steps required to set up and run the two wire interface.

## General Setup

General setup refers to register writes that are required for both slave mode and master mode operations.

Perform general setup before setting either the master or slave enable bits.

1. Program the TWI\_CTL.EN bit to enable the TWI controller and set the prescale value ( TWI\_CTL.PRESCALE bit).
2. Program the prescale value to the binary representation of f SCLK0 /10 MHz. Round up all values to the next whole number.
3. Set the TWI\_CTL.EN bit to enable the controller.

Once the TWI controller is enabled, a bus busy condition can be detected. This condition clears after t BUF  has expired, assuming no additional bus activity has been detected.

## Slave Mode

When enabled, slave mode operation supports both receive and transmit data transfers.

It is not possible to enable only one data transfer direction and not acknowledge (NAK) the other. The following setup reflects this functionality.

1. Program the TWI\_SLVADDR register. The TWI uses the appropriate 7 bits in determining a match during the address phase of the transfer.
2. Program the TWI\_TXDATA8.VALUE or TWI\_TXDATA16 registers. These values are the initial data values for transmission when the slave is addressed and transmission is needed. This step is optional. If no data is written when the slave is addressed and transmission is needed, the serial clock ( TWI\_SCL ) stretches. An interrupt is generated until data is written to the transmit FIFO.
3. Program the TWI\_IMSK register. There are enable-bits associated with the desired interrupt sources. For example, programming the value 0x000F results in an interrupt request output to the processor, when the TWI module detects a valid address match. An interrupt request also occurs when a valid slave transfer completes or has an error, or a subsequent transfer has begun and the previous transfer has not been serviced.
4. Program the TWI\_SLVCTL register. This step prepares and enables slave mode operation. For example, programming the value 0x0005 enables slave mode operation and requires 7-bit addressing. It indicates that data in the transmit FIFO buffer is for slave mode transmission.

The Slave Mode Interaction table and TWI Slave Mode Program Flow diagram represent the interaction between the TWI controller and the processor using this example.

Figure 30-12: TWI Slave Mode Program Flow

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000011_e40786122176a288a9dc72d86beb5150013b2117355dfdd3e5de26a1b772ac8d.png)

Table 30-3: Slave Mode Interaction

| TWI Controller                                 | Processor                                                                                        |
|------------------------------------------------|--------------------------------------------------------------------------------------------------|
| Interrupt: SINIT - Slave transfer in progress. | Acknowledge: Clear the interrupt source bits.                                                    |
| Interrupt: RCVSERV - Receive buffer is full.   | Acknowledge: Clear the interrupt source bits. Read TWIx_FIFO_STAT. Read the receive FIFO buffer. |
| ...                                            | ...                                                                                              |
| Interrupt: SCOMP - Slave transfer complete.    | Acknowledge: Clear the interrupt source bits. Read the receive FIFO buffer.                      |

## Master Mode Program Flow

The Master Mode Program Flow figure shows the program for the TWI in master mode.

Figure 30-13: Master Mode Program Flow

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000012_9860aeba341e712c7e1ce9bd6871db925ff8da9ef417ff3472fa0789999228bb.png)

## Master Mode Clock Setup

Master mode operation is set up and executed on a per-transfer basis.

An example of programming steps for a receive and for a transmit is given separately in following sections. The programming step for clock setup listed here is common to both transfer types.

1. Program the TWI\_CLKDIV register to define the minimum high and minimum low duration for the clock.

The TWI\_CLKDIV.CLKHI and TWI\_CLKDIV.CLKLO fields do not guarantee a certain frequency. Rather, they guarantee a certain minimum high and low duration for TWI\_SCL . The slew rate controls falling edges. The RC time constant formed by the pull-up resistor and the SCL capacitance govern rising edges. See the 'Register Descriptions' section for more details.

## Master Mode Transmit

Follow these programming steps for a single master mode transmission:

1. Program the TWI\_MSTRADDR register. This step defines the address transmitted during the address phase of the transfer.

2. Program the TWI\_TXDATA8 or TWI\_TXDATA16 register. This step configures the initial data transmitted. It is an error to complete the address phase of the transfer and not have data available in the transmit FIFO buffer.
3. Program the TWI\_FIFOCTL register. The programming indicates if the transmit FIFO buffer interrupt requests occur with each byte transmitted (8-bits) or with every 2 bytes transmitted (16-bits).
4. Program the TWI\_IMSK register. This step enables the bits associated with the desired interrupt request sources. For example, programming the value 0x0030 results in an interrupt output to the processor when the master transfer completes, and the master transfer has an error.
5. Program the TWI\_MSTRCTL register. This step prepares and enables master mode operation. For example, programming the value 0x0201: enables master mode operation, generates a 7-bit address, sets the direction to master-transmit, uses standard mode timing, and transmits 8 data bytes before generating a stop condition.

The Master Mode Transmit Setup Interaction table represents the interaction between the TWI controller and the processor using this example.

Table 30-4: Master Mode Transmit Setup Interaction

| TWI Controller                                 | Processor                                                                                |
|------------------------------------------------|------------------------------------------------------------------------------------------|
| Interrupt: XMTSERV - Transmit buffer is empty. | Acknowledge: Clear the interrupt request source bits. Write to the transmit FIFO buffer. |
| ...                                            | ...                                                                                      |
| Interrupt: MCOMP-Master transfer complete.     | Acknowledge: Clear the interrupt request source bits.                                    |

## Master Mode Receive

Follow these programming steps for a single master mode receive.

1. Program the TWI\_MSTRADDR register. This step defines the address transmitted during the address phase of the transfer.
2. Program the TWI\_FIFOCTL register. This step indicates if the receive FIFO buffer interrupt requests occur with each byte received (8-bits) or with every 2 bytes received (16-bits).
3. Program the TWI\_IMSK register. This step configures the enable bits associated with the desired interrupt sources. For example, programming the value 0x0030 results in an interrupt request output to the processor when the master transfer completes, and the master transfer has an error.
4. Program the TWI\_MSTRCTL register. This step prepares and enables master mode operation. For example, programming the value 0x0205: enables master mode operation, generates a 7-bit address, sets the direction to master-receive, uses standard mode timing, and receives 8 data bytes before generating a stop condition.

The Master Mode Receive Setup Interaction table shows the interaction between the TWI controller and the processor using this example.

Table 30-5: Master Mode Receive Setup Interaction

| TWI Controller                               | Processor                                                                             |
|----------------------------------------------|---------------------------------------------------------------------------------------|
| Interrupt: RCVSERV - Receive buffer is full. | Acknowledge: Clear the interrupt request source bits. Read the re- ceive FIFO buffer. |
| ...                                          | ...                                                                                   |
| Interrupt: MCOMP-Master transfer complete.   | Acknowledge: Clear the interrupt request source bits. Read the re- ceive FIFO buffer. |

NOTE: After the TWI\_MSTRCTL.DCNT bit decrements to zero, the TWI master device sends a NAK to indicate to the slave transmitter to release the bus. This operation allows the master to send the stop signal to terminate the transfer.

## ADSP-SC58x TWI Register Descriptions

Two-Wire Interface (TWI) contains the following registers.

Table 30-6: ADSP-SC58x TWI Register List

| Name         | Description                   |
|--------------|-------------------------------|
| TWI_CLKDIV   | SCL Clock Divider Register    |
| TWI_CTL      | Control Register              |
| TWI_FIFOCTL  | FIFO Control Register         |
| TWI_FIFOSTAT | FIFO Status Register          |
| TWI_IMSK     | Interrupt Mask Register       |
| TWI_ISTAT    | Interrupt Status Register     |
| TWI_MSTRADDR | Master Mode Address Register  |
| TWI_MSTRCTL  | Master Mode Control Registers |
| TWI_MSTRSTAT | Master Mode Status Register   |
| TWI_RXDATA16 | Rx Data Double-Byte Register  |
| TWI_RXDATA8  | Rx Data Single-Byte Register  |
| TWI_SLVADDR  | Slave Mode Address Register   |
| TWI_SLVCTL   | Slave Mode Control Register   |
| TWI_SLVSTAT  | Slave Mode Status Register    |
| TWI_TXDATA16 | Tx Data Double-Byte Register  |
| TWI_TXDATA8  | Tx Data Single-Byte Register  |

## SCL Clock Divider Register

During master mode operation, the TWI\_CLKDIV holds values, which the TWI uses to create the high and low durations of the serial clock (SCL). The clock signal SCL is an output in master mode and an input in slave mode. The values in the TWI\_CLKDIV.CLKLO and TWI\_CLKDIV.CLKHI fields add up to the CLKDIV value the following equation.

CLKDIV = TWI SCL period / 10 MHz time reference

Serial clock frequencies can vary from 400 KHz to less than 20 KHz. The resolution of the clock generated is 1/10 MHz or 100 ns. For example, for an SCL of 400 KHz (period = 1/400 KHz = 2500 ns) and an internal time reference of 10 MHz (period = 100 ns):

CLKDIV = 2500 ns / 100 ns = 25

For an SCL with a 30% duty cycle, use TWI\_CLKDIV.CLKLO = 17 and TWI\_CLKDIV.CLKHI = 8.

Figure 30-14: TWI\_CLKDIV Register Diagram

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000013_9fcbe3c49a3ea43160b2fdb22e05a7d616061de16f72b20a7d1d9ce1caffdabd.png)

Table 30-7: TWI\_CLKDIV Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/W)         | CLKHI      | SCL Clock High Periods. The TWI_CLKDIV.CLKHI specifies the number of 10 MHz time reference periods the serial clock (SCL) waits before a new clock low period begins, assuming a single master. |
| 7:0 (R/W)          | CLKLO      | SCL Clock Low Periods. The TWI_CLKDIV.CLKLO specifies the number of internal time reference periods the serial clock (SCL) is held low.                                                         |

## Control Register

The TWI\_CTL enables the TWI, establishes a relationship between the system clock ( SCLK0\_0) and the TWI controller's internally timed events, and enables SCCB compatibility.

Figure 30-15: TWI\_CTL Register Diagram

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000014_8185761cb9cdca46900f744e0d08d7c27bf30f6fc2dab308d1a0b2a6829eefb9.png)

Table 30-8: TWI\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9                  | SCCB       | SCCB Compatibility. The TWI_CTL.SCCB enables SCCB compatible operation for the TWI. SCCB com- patibility is an optional feature and should not be used in an I 2 C bus system. When this feature is enabled, all slave asserted acknowledgement bits are ignored by this mas- ter. This feature is valid only during transfers where the TWI is mastering an SCCB bus. Slave mode transfers should be avoided when this feature is enabled because the TWI controller always generates an acknowledge in slave mode. | SCCB Compatibility. The TWI_CTL.SCCB enables SCCB compatible operation for the TWI. SCCB com- patibility is an optional feature and should not be used in an I 2 C bus system. When this feature is enabled, all slave asserted acknowledgement bits are ignored by this mas- ter. This feature is valid only during transfers where the TWI is mastering an SCCB bus. Slave mode transfers should be avoided when this feature is enabled because the TWI controller always generates an acknowledge in slave mode. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Disable SCCB compatibility. When disabled, master transfers are not SCCB compatible.                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 7 (R/W)            | EN         | Enable Module. The TWI_CTL.EN enables TWI controller operation for either master and/or slave mode of operation. It is recommended that this bit be set at the time TWI_CTL.PRESCALE is initialized and remain set. This method guarantees accurate                                                                                                                                                                                                                                                                  | Enable Module. The TWI_CTL.EN enables TWI controller operation for either master and/or slave mode of operation. It is recommended that this bit be set at the time TWI_CTL.PRESCALE is initialized and remain set. This method guarantees accurate                                                                                                                                                                                                                                                                  |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |

Table 30-8: TWI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:0 (R/W)          | PRESCALE   | SCLK0_0 Prescale Value. The TWI_CTL.PRESCALE holds the pre-scaled value for the TWI internal time ref- erence. This reference is derived from SCLK0_0 according to the formula: TWI_CTL.PRESCALE = f SCLK0_0 /10MHz The TWI_CTL.PRESCALE specifies the number of system clock (SCLK0_0) periods used in the generation of one internal time reference. The value of TWI_CTL.PRESCALE must be set to create an internal time reference with a period of 10 MHz. It is represented as a 7-bit binary value. |

## FIFO Control Register

The TWI\_FIFOCTL control bits affect only the FIFO and are not tied in any way with master or slave mode operation.

Figure 30-16: TWI\_FIFOCTL Register Diagram

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000015_6992c7381c41b05fbb27ae09eed632a6a524de90f412370ddc5085dad704c38e.png)

Table 30-9: TWI\_FIFOCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                       | Description/Enumeration                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | RXILEN     | Rx Buffer Interrupt Length. The TWI_FIFOCTL.RXILEN determines the rate at which receive buffer interrupts are to be generated. Interrupts may be generated with each byte received or after two bytes are received. Interrupt status is available in TWI_FIFOSTAT.RXSTAT .                                                    | Rx Buffer Interrupt Length. The TWI_FIFOCTL.RXILEN determines the rate at which receive buffer interrupts are to be generated. Interrupts may be generated with each byte received or after two bytes are received. Interrupt status is available in TWI_FIFOSTAT.RXSTAT .                                                    |
| 3 (R/W)            | RXILEN     | 0                                                                                                                                                                                                                                                                                                                             | RXSERVI on 1 or 2 Bytes in FIFO                                                                                                                                                                                                                                                                                               |
| 3 (R/W)            | RXILEN     | 1                                                                                                                                                                                                                                                                                                                             | RXSERVI on 2 Bytes in FIFO                                                                                                                                                                                                                                                                                                    |
| 2 (R/W)            | TXILEN     | Tx Buffer Interrupt Length. The TWI_FIFOCTL.TXILEN determines the rate at which transmit buffer inter- rupts are to be generated. Interrupts may be generated with each byte transmitted or after two bytes are transmitted. Interrupt status is available in TWI_FIFOSTAT.TXSTAT .                                           | Tx Buffer Interrupt Length. The TWI_FIFOCTL.TXILEN determines the rate at which transmit buffer inter- rupts are to be generated. Interrupts may be generated with each byte transmitted or after two bytes are transmitted. Interrupt status is available in TWI_FIFOSTAT.TXSTAT .                                           |
| 2 (R/W)            | TXILEN     | 0                                                                                                                                                                                                                                                                                                                             | TXSERVI on 1 Byte of FIFO Empty                                                                                                                                                                                                                                                                                               |
| 2 (R/W)            | TXILEN     | 1                                                                                                                                                                                                                                                                                                                             | TXSERVI on 2 Bytes of FIFO Empty                                                                                                                                                                                                                                                                                              |
| 1 (R/W)            | RXFLUSH    | Rx Buffer Flush. The TWI_FIFOCTL.RXFLUSH directs the TWI to flush the contents of the receive buffer and update TWI_FIFOSTAT.RXSTAT to indicate the buffer is empty. This state is held until this bit is cleared. During an active receive, the receive buffer in this state responds to the receive logic as if it is full. | Rx Buffer Flush. The TWI_FIFOCTL.RXFLUSH directs the TWI to flush the contents of the receive buffer and update TWI_FIFOSTAT.RXSTAT to indicate the buffer is empty. This state is held until this bit is cleared. During an active receive, the receive buffer in this state responds to the receive logic as if it is full. |
| 1 (R/W)            | RXFLUSH    | 0                                                                                                                                                                                                                                                                                                                             | Normal Operation of Rx Buffer                                                                                                                                                                                                                                                                                                 |
| 1 (R/W)            | RXFLUSH    | 1                                                                                                                                                                                                                                                                                                                             | Flush Rx Buffer                                                                                                                                                                                                                                                                                                               |

Table 30-9: TWI\_FIFOCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | TXFLUSH    | Tx Buffer Flush. The TWI_FIFOCTL.TXFLUSH directs the TWI to flush the contents of the trans- mit buffer and update TWI_FIFOSTAT.TXSTAT to indicate the buffer is empty. This state is held until this bit is cleared. During an active transmit, the transmit buffer in this state responds to the transmit logic as if it is empty. |
| 0 (R/W)            | TXFLUSH    | 0 Normal Operation of Tx Buffer                                                                                                                                                                                                                                                                                                      |
| 0 (R/W)            | TXFLUSH    | 1 Flush Tx Buffer                                                                                                                                                                                                                                                                                                                    |

## FIFO Status Register

The TWI\_FIFOSTAT fields indicate the state of the FIFO buffers' receive and transmit contents. The FIFO buffers do not discriminate between master data and slave data. By using the status and control bits provided, the FIFO can be managed to allow simultaneous master and slave operation.

Figure 30-17: TWI\_FIFOSTAT Register Diagram

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000016_e2a4a8eb210024d5626c455616ebe29da2654086c429488b9b88f32762692c51.png)

Table 30-10: TWI\_FIFOSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                 | Description/Enumeration                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:2 (R/NW)         | RXSTAT     | Rx FIFO Status. The read-only TWI_FIFOSTAT.RXSTAT indicates the number of valid data bytes in the receive FIFO buffer. The status is updated with each FIFO buffer read using the peripheral data bus or write access by the receive shift register. Simultaneous accesses are allowed. | Rx FIFO Status. The read-only TWI_FIFOSTAT.RXSTAT indicates the number of valid data bytes in the receive FIFO buffer. The status is updated with each FIFO buffer read using the peripheral data bus or write access by the receive shift register. Simultaneous accesses are allowed. |
|                    |            | 0                                                                                                                                                                                                                                                                                       | Empty. The FIFO is empty.                                                                                                                                                                                                                                                               |
|                    |            | 1                                                                                                                                                                                                                                                                                       | Contains 1 Byte. The FIFO contains one byte of data. A single byte peripheral read of the FIFO is allowed.                                                                                                                                                                              |
|                    |            | 2                                                                                                                                                                                                                                                                                       | Reserved                                                                                                                                                                                                                                                                                |
|                    |            | 3                                                                                                                                                                                                                                                                                       | Full. The FIFO is full and contains two bytes of data. Either a single or double byte peripheral read of the FIFO is allowed.                                                                                                                                                           |
| 1:0 (R/NW)         | TXSTAT     | Tx FIFO Status. The read-only TWI_FIFOSTAT.TXSTAT field indicates the number of valid data bytes in the FIFO buffer. The status is updated with each FIFO buffer write using the peripheral data bus or read access by the transmit shift register. Simultaneous accesses are allowed.  | Tx FIFO Status. The read-only TWI_FIFOSTAT.TXSTAT field indicates the number of valid data bytes in the FIFO buffer. The status is updated with each FIFO buffer write using the peripheral data bus or read access by the transmit shift register. Simultaneous accesses are allowed.  |
|                    |            | 0                                                                                                                                                                                                                                                                                       | Empty. The FIFO is empty. Either a single or double byte peripheral write of the FIFO is allowed.                                                                                                                                                                                       |
|                    |            | 1                                                                                                                                                                                                                                                                                       | Contains 1 Byte. The FIFO contains one byte of data. A single byte peripheral write of the FIFO is allowed.                                                                                                                                                                             |
|                    |            | 2                                                                                                                                                                                                                                                                                       | Reserved                                                                                                                                                                                                                                                                                |
|                    |            | 3                                                                                                                                                                                                                                                                                       | Full. The FIFO is full and contains two bytes of data.                                                                                                                                                                                                                                  |

## Interrupt Mask Register

The TWI\_IMSK enables interrupt sources to assert the interrupt output. Each mask bit corresponds with one interrupt request source bit in TWI\_ISTAT . Reading and writing TWI\_IMSK does not affect the contents of the TWI\_ISTAT .

Figure 30-18: TWI\_IMSK Register Diagram

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000017_eea8f39525086e39a96961b6991b0dd24abe407f10544cefa899d73d4a440922.png)

Table 30-11: TWI\_IMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                               |
|--------------------|------------|-----------------------------------------------------------------------|
| 15                 | SCLI       | Serial Clock Interrupt Mask.                                          |
| 14 (R/W)           | SDAI       | Serial Data Interrupt Mask. 0 Mask (Disable) Interrupt                |
| 7 (R/W)            | RXSERV     | Rx FIFO Service Interrupt Mask. 0 Mask (Disable)                      |
| 6                  | TXSERV     | Interrupt 1 Unmask (Enable) Interrupt Tx FIFO Service Interrupt Mask. |
| (R/W)              |            | 0 Mask (Disable) Interrupt 1 Unmask (Enable) Interrupt                |

Table 30-11: TWI\_IMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------|
| 5                  | MERR       | Master Transfer Error Interrupt Mask.                                                           |
| 4 (R/W)            | MCOMP      | Master Transfer Complete Interrupt Mask. 0 Mask (Disable) Interrupt 1 Unmask (Enable) Interrupt |
| 3 (R/W)            | SOVF SERR  | Slave Overflow Interrupt Mask. 0 Mask (Disable) Interrupt 1 Unmask (Enable) Interrupt           |
| 2 (R/W)            |            | Slave Transfer Error Interrupt Mask. 0 Mask (Disable) Interrupt                                 |
| 1 (R/W)            | SCOMP      | 1 Unmask (Enable) Interrupt                                                                     |
| 0                  |            | Slave Transfer Complete Interrupt Mask. 0 Mask (Disable) Interrupt 1 Unmask (Enable) Interrupt  |
| (R/W)              | SINIT      | Slave Transfer Initiated Interrupt Mask. 0 Mask (Disable) Interrupt                             |
|                    |            | 1                                                                                               |
|                    |            | Unmask (Enable) Interrupt                                                                       |

## Interrupt Status Register

The TWI\_ISTAT contains information about functional areas requiring servicing. Many of the bits serve as an indicator to further read and service various status registers. After servicing the interrupt source associated with a bit, the user must clear that interrupt source bit by writing a 1 to it.

Figure 30-19: TWI\_ISTAT Register Diagram

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000018_9328e3910f8d3003b37bbb9cee5c0cb393759d8d86dfc1e1595a55b08c3e32e4.png)

Table 30-12: TWI\_ISTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                        | Description/Enumeration                                                                                                                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W1C)         | SCLI       | Serial Clock Interrupt. If the TWI is enabled ( TWI_CTL.EN ), SCLI is set on a high-to-low transition of the serial clock pin (SCLx). Normally, this bit is not required for I 2 C bus transfers. It will be initially set on an I 2 C transfer and does not require clearing. | Serial Clock Interrupt. If the TWI is enabled ( TWI_CTL.EN ), SCLI is set on a high-to-low transition of the serial clock pin (SCLx). Normally, this bit is not required for I 2 C bus transfers. It will be initially set on an I 2 C transfer and does not require clearing. |
| 15 (R/W1C)         | SCLI       | 0                                                                                                                                                                                                                                                                              | No Interrupt. No transition was detected on the SCLx pin.                                                                                                                                                                                                                      |
| 15 (R/W1C)         | SCLI       | 1                                                                                                                                                                                                                                                                              | Interrupt Detected. A high-to-low transition was detect- ed on the SCLx pin. This bit is W1C.                                                                                                                                                                                  |
| 14 (R/W1C)         | SDAI       | Serial Data Interrupt. If the TWI is enabled ( TWI_CTL.EN ), SDAI is set on a high-to-low transition of the serial data pin (SDAx). Normally, this bit is not required for I 2 C bus transfers. It will be initially set on an I 2 C transfer and does not require clearing.   | Serial Data Interrupt. If the TWI is enabled ( TWI_CTL.EN ), SDAI is set on a high-to-low transition of the serial data pin (SDAx). Normally, this bit is not required for I 2 C bus transfers. It will be initially set on an I 2 C transfer and does not require clearing.   |
| 14 (R/W1C)         | SDAI       | 0                                                                                                                                                                                                                                                                              | No Interrupt. No transition was detected on the SDAx pin.                                                                                                                                                                                                                      |
| 14 (R/W1C)         | SDAI       | 1                                                                                                                                                                                                                                                                              | Interrupt Detected. A high-to-low transition was detect- ed on the SDAx pin. This bit is W1C.                                                                                                                                                                                  |

Table 30-12: TWI\_ISTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W1C)          | RXSERV     | Rx FIFO Service. If TWI_FIFOCTL.RXILEN =0, the TWI_ISTAT.RXSERV is set each time the TWI_FIFOSTAT.RXSTAT field is updated to either 01 or 11. If TWI_FIFOCTL.RXILEN =1, the TWI_ISTAT.RXSERV is set each time TWI_FIFOSTAT.RXSTAT is updated to 11.                               |
| 6 (R/W1C)          | TXSERV     | Tx FIFO Service. If TWI_FIFOCTL.TXILEN =0, the TWI_ISTAT.TXSERV is set each time the TWI_FIFOSTAT.TXSTAT field is updated to either 01 or 00. If TWI_FIFOCTL.TXILEN =1, the TWI_ISTAT.TXSERV is set each time TWI_FIFOSTAT.TXSTAT is updated to 00.                               |
| 5 (R/W1C)          | MERR       | 1 Interrupt Detected. The transmit FIFO buffer has one or two 8-bit locations available to be written. Master Transfer Error. The TWI_ISTAT.MERR indicates that a master error has occurred. The surrounding the error are indicated by the master status register ( TWI_MSTRSTAT |
| 4 (R/W1C)          |            | conditions ). 0 No Interrupt 1 Interrupt Detected                                                                                                                                                                                                                                 |
|                    | MCOMP      | Master Transfer Complete. The TWI_ISTAT.MCOMP indicates that the initiated master transfer has completed. In the absence of a repeat start, the bus has been released. 0 No Interrupt 1 Interrupt Detected                                                                        |
| 3 (R/W1C)          | SOVF       | Slave Overflow. The TWI_ISTAT.SOVF indicates that the TWI_ISTAT.SCOMP bit was set at the time a subsequent transfer has acknowledged an address phase. The transfer continues, however, it may be difficult to delineate data of one transfer from another.                       |

Table 30-12: TWI\_ISTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                   | Description/Enumeration                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W1C)          | SERR       | Slave Transfer Error. The TWI_ISTAT.SERR indicates that a slave error has occurred. A restart or stop condition has occurred during the data receive phase of a transfer. | Slave Transfer Error. The TWI_ISTAT.SERR indicates that a slave error has occurred. A restart or stop condition has occurred during the data receive phase of a transfer. |
| 2 (R/W1C)          | SERR       | 0                                                                                                                                                                         | No Interrupt                                                                                                                                                              |
| 2 (R/W1C)          | SERR       | 1                                                                                                                                                                         | Interrupt Detected                                                                                                                                                        |
| 1 (R/W1C)          | SCOMP      | Slave Transfer Complete. The TWI_ISTAT.SCOMP indicates that the transfer is complete and either a stop, or a restart was detected.                                        | Slave Transfer Complete. The TWI_ISTAT.SCOMP indicates that the transfer is complete and either a stop, or a restart was detected.                                        |
| 1 (R/W1C)          | SCOMP      | 0                                                                                                                                                                         | No Interrupt                                                                                                                                                              |
| 1 (R/W1C)          | SCOMP      | 1                                                                                                                                                                         | Interrupt Detected                                                                                                                                                        |
| 0 (R/W1C)          | SINIT      | Slave Transfer Initiated. The TWI_ISTAT.SINIT indicates whether or not a slave transfer is in progress.                                                                   | Slave Transfer Initiated. The TWI_ISTAT.SINIT indicates whether or not a slave transfer is in progress.                                                                   |
| 0 (R/W1C)          | SINIT      | 0                                                                                                                                                                         | No Interrupt. A transfer is not in progress, or an address match has not occurred since the last time this bit was cleared.                                               |
| 0 (R/W1C)          | SINIT      | 1                                                                                                                                                                         | Interrupt Detected. The slave has detected an address match, and a transfer has been initiated.                                                                           |

## Master Mode Address Register

During the addressing phase of a transfer, the TWI controller, with its master enabled, transmits the contents of TWI\_MSTRADDR . When programming this register, omit the read/write bit. That is, only the upper 7 bits that make up the slave address should be written to this register. For example, if the slave address is b#1010000X, where X is the read/write bit, the TWI\_MSTRADDR is programmed with b#1010000, which corresponds to 0x50. When sending out the address on the bus, the TWI controller appends the read/write bit as appropriate based on the state of the TWI\_MSTRCTL.DIR bit.

Figure 30-20: TWI\_MSTRADDR Register Diagram

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000019_cf252eac1839065f61e3d0d8a30f534df3ae880d24c9fe003ae48e60afb035a6.png)

Table 30-13: TWI\_MSTRADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 6:0                | ADDR       | Master Mode Address.      |
| (R/W)              |            |                           |

## Master Mode Control Registers

The TWI\_MSTRCTL controls the logic associated with master mode operation. Bits in this register do not affect slave mode operation and should not be modified to control slave mode functionality.

Figure 30-21: TWI\_MSTRCTL Register Diagram

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000020_51b0c872cfe8a12b11f988680cacfc63c541ce6d163a3bb7e60e6d629ad7b54f.png)

Table 30-14: TWI\_MSTRCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | SCLOVR     | Serial Clock Override. The TWI_MSTRCTL.SCLOVR provides direct control of the serial clock line when required. Normal master and slave mode operation should not require override opera- tion. When TWI_MSTRCTL.SCLOVR is set, the TWI overrides normal serial clock output, driving it to an active 0 level and overriding all other logic. This state is held until this bit is cleared. When TWI_MSTRCTL.SCLOVR is cleared, the TWI permits normal serial clock operation under the control of master mode clock generation and slave mode clock stretching logic. | Serial Clock Override. The TWI_MSTRCTL.SCLOVR provides direct control of the serial clock line when required. Normal master and slave mode operation should not require override opera- tion. When TWI_MSTRCTL.SCLOVR is set, the TWI overrides normal serial clock output, driving it to an active 0 level and overriding all other logic. This state is held until this bit is cleared. When TWI_MSTRCTL.SCLOVR is cleared, the TWI permits normal serial clock operation under the control of master mode clock generation and slave mode clock stretching logic. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Permit Normal SCL Operation                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 14 (R/W)           | SDAOVR     | Serial Data Override. The TWI_MSTRCTL.SDAOVR provides direct control of the serial data line when re- quired. Normal master and slave mode operation should not require override opera- tion. When TWI_MSTRCTL.SDAOVR is set, the TWI overrides normal serial data operation under the control of the transmit shift register and acknowledge logic, driv- ing serial data output to an active 0 level and overriding all other logic. This state is held until this bit is cleared. When TWI_MSTRCTL.SDAOVR is cleared, the TWI                                     | Serial Data Override. The TWI_MSTRCTL.SDAOVR provides direct control of the serial data line when re- quired. Normal master and slave mode operation should not require override opera- tion. When TWI_MSTRCTL.SDAOVR is set, the TWI overrides normal serial data operation under the control of the transmit shift register and acknowledge logic, driv- ing serial data output to an active 0 level and overriding all other logic. This state is held until this bit is cleared. When TWI_MSTRCTL.SDAOVR is cleared, the TWI                                     |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Permit Normal SDA Operation                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Override Normal SDA Operation                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

Table 30-14: TWI\_MSTRCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:6 (R/W)         | DCNT       | Data Transfer Count. The TWI_MSTRCTL.DCNT indicates the number of data bytes to transfer. As each data word is transferred, the TWI decrements this counter. When TWI_MSTRCTL.DCNT decrements to 0, a stop condition is generated. Setting TWI_MSTRCTL.DCNT to 0xFF disables the counter. In this transfer mode, data con- tinues to be transferred until it is concluded by setting the TWI_MSTRCTL.STOP bit. In the event a master transmit is aborted due to a slave data NAK, the value of TWI_MSTRCTL.DCNT equals the number of bytes not sent. The byte which was | Data Transfer Count. The TWI_MSTRCTL.DCNT indicates the number of data bytes to transfer. As each data word is transferred, the TWI decrements this counter. When TWI_MSTRCTL.DCNT decrements to 0, a stop condition is generated. Setting TWI_MSTRCTL.DCNT to 0xFF disables the counter. In this transfer mode, data con- tinues to be transferred until it is concluded by setting the TWI_MSTRCTL.STOP bit. In the event a master transmit is aborted due to a slave data NAK, the value of TWI_MSTRCTL.DCNT equals the number of bytes not sent. The byte which was |
| 5 (R/W)            | RSTART     | NAK'ed by the slave is counted as a sent byte. Repeat Start. The TWI_MSTRCTL.RSTART enables the TWI to issue a repeat start condition at the conclusion of the current transfer ( TWI_MSTRCTL.DCNT =0) and begin the next transfer. The current transfer concludes with updates to the appropriate status and in- terrupt bits. If errors occurred during the previous transfer, a repeat start does not oc-                                                                                                                                                            | NAK'ed by the slave is counted as a sent byte. Repeat Start. The TWI_MSTRCTL.RSTART enables the TWI to issue a repeat start condition at the conclusion of the current transfer ( TWI_MSTRCTL.DCNT =0) and begin the next transfer. The current transfer concludes with updates to the appropriate status and in- terrupt bits. If errors occurred during the previous transfer, a repeat start does not oc-                                                                                                                                                            |
| 5 (R/W)            | RSTART     | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Disable Repeat Start                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 4 (R/W)            | STOP       | Issue Stop Condition. The TWI_MSTRCTL.STOP directs the TWI to issue a stop condition. The transfer concludes as soon as possible avoiding any error conditions (as if data transfer count had been reached). At that time, the TWI_IMSK is updated along with any associated status bits.                                                                                                                                                                                                                                                                               | Issue Stop Condition. The TWI_MSTRCTL.STOP directs the TWI to issue a stop condition. The transfer concludes as soon as possible avoiding any error conditions (as if data transfer count had been reached). At that time, the TWI_IMSK is updated along with any associated status bits.                                                                                                                                                                                                                                                                               |
| 4 (R/W)            | STOP       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Permit Normal Operation                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 4 (R/W)            | STOP       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Issue Stop                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 3 (R/W)            | FAST       | Fast Mode. The TWI_MSTRCTL.FAST selects whether the TWI operates in fast mode or stand- ard mode. In fast mode, the TWI uses timing specifications for transfers at up to 400K bits/s. In standard mode, the TWI uses timing specifications for transfers at up to 100K bits/s.                                                                                                                                                                                                                                                                                         | Fast Mode. The TWI_MSTRCTL.FAST selects whether the TWI operates in fast mode or stand- ard mode. In fast mode, the TWI uses timing specifications for transfers at up to 400K bits/s. In standard mode, the TWI uses timing specifications for transfers at up to 100K bits/s.                                                                                                                                                                                                                                                                                         |
| 3 (R/W)            | FAST       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Select Standard Mode                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 3 (R/W)            | FAST       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Select Fast Mode                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 2 (R/W)            | DIR        | Transfer Direction for Master. The TWI_MSTRCTL.DIR selects the transfer direction for the TWI as master initiat- ed receive or transmit.                                                                                                                                                                                                                                                                                                                                                                                                                                | Transfer Direction for Master. The TWI_MSTRCTL.DIR selects the transfer direction for the TWI as master initiat- ed receive or transmit.                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 2 (R/W)            | DIR        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Master Transmit                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 2 (R/W)            | DIR        | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Master Receive                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |

Table 30-14: TWI\_MSTRCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | EN         | Enable Master Mode. The TWI_MSTRCTL.EN enables master mode functionality. A start condition is gen- erated if the bus is idle. This bit self clears at the completion of a transfer (after TWI_MSTRCTL.DCNT decrements to zero), including transfers terminated due to er- rors. If disabled (=0) during operation, the transfer is aborted, and all logic associated with master mode transfers are reset. Serial data and serial clock (SDA, SCL) are no longer driven. Write-1-to-clear status bits are not affected. 0 Disable |
| 0 (R/W)            | EN         | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 0 (R/W)            | EN         |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |

## Master Mode Status Register

The TWI\_MSTRSTAT holds information during master mode transfers and at their conclusion. Generally, master mode status bits are not directly associated with the generation of interrupt requests, but these bits offer information on the current transfer. Slave mode operation does not affect master mode status bits.

Note that while TWI\_MSTRSTAT.SCLSEN is set (this condition could be due to having no pull-up resistor on TWI\_SCL or another agent is driving TWI\_SCL low), the acknowledge bits ( TWI\_MSTRSTAT.ANAK and TWI\_MSTRSTAT.DNAK ) do not update. This result occurs because the acknowledge conditions are sampled during the high phase of TWI\_SCL .

Figure 30-22: TWI\_MSTRSTAT Register Diagram

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000021_e72c9b0e514f355b74917301e1c9bf120d8477764a00497b2931c11e9186df96.png)

Table 30-15: TWI\_MSTRSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/NW)           | BUSBUSY    | Bus Busy. The TWI_MSTRSTAT.BUSBUSY indicates whether the bus is currently busy or free. This indication is not limited to only this device but is for all devices. On a start con- dition, the setting of the register value is delayed due to the input filtering. On a stop condition the clearing of the register value occurs after t BUF . |
| 8 (R/NW)           | BUSBUSY    | 0 Bus Free. The bus is free. The clock and data bus signals have been inactive for the appropriate bus free time.                                                                                                                                                                                                                               |
| 8 (R/NW)           | BUSBUSY    | 1 Bus Busy. The bus is busy. Clock or data activity has been detected.                                                                                                                                                                                                                                                                          |

Table 30-15: TWI\_MSTRSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/NW)           | SCLSEN     | Serial Clock Sense. The TWI_MSTRSTAT.SCLSEN indicates the active or inactive state of the serial clock. Use this status bit when direct sensing of the serial clock line is required. The register value is delayed due to the input filter (nominally 50 ns). Normal master and slave mode operation should not require this feature. |
| 6 (R/NW)           | SDASEN     | Serial Data Sense. The TWI_MSTRSTAT.SDASEN indicates the active or inactive status of the serial data. Use this status bit when direct sensing of the serial data line is required. The reg- ister value is delayed due to the input filter (nominally 50 ns). Normal master and slave mode operation should not require this feature. |
| 5 (R/W1C)          | BUFWRERR   | driver is not known and can be internal or external. Buffer Write Error. The TWI_MSTRSTAT.BUFWRERR indicates whether the current master transfer was aborted due to a receive buffer write error. The receive buffer and receive shift register were both full at the same time. This bit is W1C. 0 No Status                          |
| 4 (R/W1C)          | BUFRDERR   | 1 Buffer Write Error Buffer Read Error. The TWI_MSTRSTAT.BUFRDERR indicates whether the current master transfer was aborted due to the detection of a NAK during data transmission. This bit is W1C.                                                                                                                                   |
| 3 (R/W1C)          |            | 0 No Status 1 Buffer Read Error                                                                                                                                                                                                                                                                                                        |
|                    | DNAK       | Data Not Acknowledged. The TWI_MSTRSTAT.DNAK indicates whether the current master transfer was aborted due to the detection of a NAK during data transmission. This bit is W1C.                                                                                                                                                        |

Table 30-15: TWI\_MSTRSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W1C)          | ANAK       | Address Not Acknowledged. The TWI_MSTRSTAT.ANAK indicates whether the current master transfer was aborted due to the detection of a NAK during the address phase of the transfer. This bit is W1C.                                                                                                        |
| 1 (R/W1C)          | LOSTARB    | Lost Arbitration. The TWI_MSTRSTAT.LOSTARB indicates whether the current transfer was aborted due to the loss of arbitration with another master. This bit is W1C. 0 No Status                                                                                                                            |
| 0 (R/NW)           | MPROG      | Master Transfer in Progress. The TWI_MSTRSTAT.MPROG indicates whether or not a master transfer is in prog- ress. If clear ( TWI_MSTRSTAT.MPROG =0), currently no transfer is taking place. This can occur after a transfer is complete or while an enabled master is waiting for an idle bus. 0 No Status |
| 0 (R/NW)           |            | 1 Master Transfer in Progress                                                                                                                                                                                                                                                                             |
| 0 (R/NW)           |            |                                                                                                                                                                                                                                                                                                           |

## Rx Data Double-Byte Register

The TWI\_RXDATA16 holds a 16-bit data value read from the FIFO buffer. To reduce interrupt output rates and peripheral bus access times, a double byte receive data access can be performed. T wo data bytes can be read, effectively emptying the receive FIFO buffer with a single access.

The data is read in little endian byte order, where byte 0 is the first byte received and byte 1 is the second byte received. With each access, the receive status ( TWI\_FIFOSTAT.RXSTAT ) field is updated to indicate it is empty. If an access is performed while the FIFO buffer is not full, the read data is unknown and the existing FIFO buffer data and its status remains unchanged.

Figure 30-23: TWI\_RXDATA16 Register Diagram

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000022_e0010cb9da66f7cb9d6d422dd05cebe0f8559715bfea588e5149378bd203ac85.png)

Table 30-16: TWI\_RXDATA16 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 15:0               | VALUE      | Rx Data 16-Bit Value.     |
| (R0/W)             |            |                           |

## Rx Data Single-Byte Register

The TWI\_RXDATA8 holds an 8-bit data value read from the FIFO buffer. Receive data is read from the corresponding receive buffer in a first-in first-out order. Although peripheral bus reads are 16 bits, a read access to TWI\_RXDATA8 accesses only one transmit data byte from the FIFO buffer. With each access, the receive status ( TWI\_FIFOSTAT.RXSTAT ) field is updated. If an access is performed while the FIFO buffer is empty, the data is unknown and the FIFO buffer status remains indicating it is empty.

Figure 30-24: TWI\_RXDATA8 Register Diagram

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000023_fa3a1acbab431d1527f757d3aff87bf4557f8ca34cbe35bf9fd2019d118495a8.png)

Table 30-17: TWI\_RXDATA8 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | VALUE      | Rx Data 8-Bit Value.      |
| (R0/W)             |            |                           |

## Slave Mode Address Register

The TWI\_SLVADDR holds the slave mode address, which is the valid address to which the slave-enabled TWI controller responds. The TWI controller compares this value with the received address during the addressing phase of a transfer.

Figure 30-25: TWI\_SLVADDR Register Diagram

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000024_8c9c1a25f47893b94e021c4d4fbf62069f8d7de4d7fe6dea4f7fb8b998fe94ec.png)

Table 30-18: TWI\_SLVADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 6:0                | ADDR       | Slave Mode Address.       |
| (R/W)              |            |                           |

## Slave Mode Control Register

The TWI\_SLVCTL controls the logic associated with slave mode operation. Settings in this register do not affect master mode operation and should not be modified to control master mode functionality.

Figure 30-26: TWI\_SLVCTL Register Diagram

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000025_6b3b2c75ad77e8a7f6d3fce02b4932e4e784fee73c74f67ea5001775160bce29.png)

Table 30-19: TWI\_SLVCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                            | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | GEN        | General Call Enable. The TWI_SLVCTL.GEN enables general call address matching. When enabled, a general call slave receive transfer is accepted. All status and interrupt source bits associ- ated with transfers are updated. Note that general call address detection is available on- ly when slave mode is enabled.                                                                                                                             | General Call Enable. The TWI_SLVCTL.GEN enables general call address matching. When enabled, a general call slave receive transfer is accepted. All status and interrupt source bits associ- ated with transfers are updated. Note that general call address detection is available on- ly when slave mode is enabled.                                                                                                                             |
| 4 (R/W)            | GEN        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Disable General Call Matching                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 4 (R/W)            | GEN        | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Enable General Call Matching                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 3 (R/W)            | NAK        | Not Acknowledge. The TWI_SLVCTL.NAK directs the TWI to generate a NAK (if set) or an ACK (if cleared) at the conclusion of data transfer for slave receive. For NAK, the slave is still considered to be addressed at the conclusion of transfer.                                                                                                                                                                                                  | Not Acknowledge. The TWI_SLVCTL.NAK directs the TWI to generate a NAK (if set) or an ACK (if cleared) at the conclusion of data transfer for slave receive. For NAK, the slave is still considered to be addressed at the conclusion of transfer.                                                                                                                                                                                                  |
| 3 (R/W)            | NAK        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Generate ACK                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 3 (R/W)            | NAK        | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Generate NAK                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 2 (R/W)            | TDVAL      | Transmit Data Valid for Slave. The TWI_SLVCTL.TDVAL selects whether the data in the transmit FIFO is available (valid) for slave transmission ( TWI_SLVCTL.TDVAL set). If the FIFO data is not available (invalid) for slave transmission ( TWI_SLVCTL.TDVAL cleared), the data in the transmit FIFO is for master mode transmits, and the data is not allowed to be used during a slave transmit; the transmit FIFO is treated as if it is empty. | Transmit Data Valid for Slave. The TWI_SLVCTL.TDVAL selects whether the data in the transmit FIFO is available (valid) for slave transmission ( TWI_SLVCTL.TDVAL set). If the FIFO data is not available (invalid) for slave transmission ( TWI_SLVCTL.TDVAL cleared), the data in the transmit FIFO is for master mode transmits, and the data is not allowed to be used during a slave transmit; the transmit FIFO is treated as if it is empty. |
| 2 (R/W)            | TDVAL      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Data Invalid for Slave Tx                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 2 (R/W)            | TDVAL      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Data Valid for Slave Tx                                                                                                                                                                                                                                                                                                                                                                                                                            |

Table 30-19: TWI\_SLVCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | EN         | Enable Slave Mode. The TWI_SLVCTL.EN enables slave operation. Enabling slave and master modes of operation concurrently is allowed. If disabled, no attempt is made to identify a valid address. If TWI_SLVCTL.EN is cleared during a valid transfer, clock stretching ceases, the serial data line is released, and the current byte is not acknowledged. |
| 0 (R/W)            | EN         | 0 Disable                                                                                                                                                                                                                                                                                                                                                  |
| 0 (R/W)            | EN         | 1 Enable                                                                                                                                                                                                                                                                                                                                                   |

## Slave Mode Status Register

During and at the conclusion of register slave mode transfers, the TWI\_SLVSTAT holds information on the current transfer. Generally slave mode status bits are not associated with the generation of interrupt requests. Master mode operation does not affect slave mode status bits.

Figure 30-27: TWI\_SLVSTAT Register Diagram

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000026_8c1886a70d6a09abc62e2f5c0fadf0d623d174c2cf6362f34e91826b18f4c0e6.png)

Table 30-20: TWI\_SLVSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                    | Description/Enumeration                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/NW)           | GCALL      | General Call. The TWI_SLVSTAT.GCALL indicates whether or not--at the time of addressing-- the address was determined to be a general call. This bit self clears if slave mode is disa- bled ( TWI_SLVCTL.EN =0).                           | General Call. The TWI_SLVSTAT.GCALL indicates whether or not--at the time of addressing-- the address was determined to be a general call. This bit self clears if slave mode is disa- bled ( TWI_SLVCTL.EN =0).                           |
| 0 (R/NW)           | DIR        | Transfer Direction for Slave. The TWI_SLVSTAT.DIR indicates whether--at the time of addressing--the transfer direction was determined to be slave transmit or receive. This bit self clears if slave mode is disabled ( TWI_SLVCTL.EN =0). | Transfer Direction for Slave. The TWI_SLVSTAT.DIR indicates whether--at the time of addressing--the transfer direction was determined to be slave transmit or receive. This bit self clears if slave mode is disabled ( TWI_SLVCTL.EN =0). |
|                    |            | 0                                                                                                                                                                                                                                          | Slave Receive                                                                                                                                                                                                                              |
|                    |            | 1                                                                                                                                                                                                                                          | Slave Transmit                                                                                                                                                                                                                             |

## Tx Data Double-Byte Register

The TWI\_TXDATA16 register holds a 16-bit data value written into the FIFO buffer. To reduce interrupt latency output rates and peripheral bus access times, a double byte transfer data access can be done. T wo data bytes can be written, effectively filling the transmit FIFO buffer with a single access.

The data is written in little endian byte order, where byte 0 is the first byte to be transferred and byte 1 is the second byte to be transferred. With each access, the transmit status ( TWI\_FIFOSTAT.TXSTAT ) field is updated. If an access is performed while the FIFO buffer is not empty, the write is ignored and the existing FIFO buffer data and its status remains unchanged. This register when read back returns zero.

Figure 30-28: TWI\_TXDATA16 Register Diagram

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000027_80ddcf75a4b7e2da88e745d9e1d87711c0533e154fbfc820f86345156aee30d8.png)

Table 30-21: TWI\_TXDATA16 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 15:0               | VALUE      | Tx Data 16-Bit Value.     |
| (R0/W)             |            |                           |

## Tx Data Single-Byte Register

The TWI\_TXDATA8 register holds an 8-bit data value written into the FIFO buffer. T ransmit data is entered into the corresponding transmit buffer in a first-in first-out order. For 16-bit peripheral bus writes, a write access to this register adds only one transmit data byte to the FIFO buffer. With each access, the transmit status

( TWI\_FIFOSTAT.TXSTAT ) field is updated. If an access is performed while the FIFO buffer is full, the write is ignored and the existing FIFO buffer data and its status remains unchanged. This register returns zero when read back.

Figure 30-29: TWI\_TXDATA8 Register Diagram

![Image](33_Two-Wire_Interface_(TWI)_artifacts/image_000028_7d31d6e1d68cc2b9a12a93703ecf591251bbcac8fec3c6721744b30f0223b9e5.png)

Table 30-22: TWI\_TXDATA8 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | VALUE      | Tx Data 8-Bit Value.      |
| (R0/W)             |            |                           |