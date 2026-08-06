# True Random Number Generator (TRNG)

<!-- source: 050_True_Random_Number_Generator_TRNG.pdf | original pages 2790–2832 -->

## 43   True Random Number Generator (TRNG)

The TRNG engine provides a true, non-deterministic, noise source for generating keys, Initialization Vectors (IVs), and other random number requirements. Other non-cryptographic purposes include statistical sampling, retry timers for communications protocols and noise generation.

## TRNG Features

The TRNG features include:

- Hardware-based non-deterministic random number generator
- ANSI X9.31 postprocessing (depending on processor)
- Redundant 'fail-safe' design with self-test circuits
- Reliable shot noise oscillator implementation with auto-tuning
- Debug output to allow monitoring of internal operation
- Alarm count overflow and auto-tuning error interrupts
- Buffer to allow generation of large blocks of random data in the background

## TRNG Functional Description

The following sections provide details on the function of the TRNG module.

## ADSP-2159x\_SC592\_SC594 TRNG Register List

The True Random Number Generator (TRNG) provides random number generation, intended mainly for securityrelated applications. A set of registers governs TRNG operations. For more information on TRNG functionality, see the TRNG register descriptions.

Table 43-1: ADSP-2159x\_SC592\_SC594 TRNG Register List

| Name        | Description                 |
|-------------|-----------------------------|
| TRNG_ALMCNT | TRNG Alarm Counter Register |

Table 43-1: ADSP-2159x\_SC592\_SC594 TRNG Register List (Continued)

| Name            | Description                              |
|-----------------|------------------------------------------|
| TRNG_ALMMSK     | TRNG Alarm Mask Register                 |
| TRNG_ALMSTP     | TRNG Alarm Stop Register                 |
| TRNG_BLKCNT     | TRNG Block Count Register                |
| TRNG_CFG        | TRNG Configuration Register              |
| TRNG_CNT        | Counter Register                         |
| TRNG_CTL        | TRNG Control Register                    |
| TRNG_FRODETUNE  | TRNG FRO De-tune Register                |
| TRNG_FROEN      | TRNG FRO Enable Register                 |
| TRNG_INPUT[n]   | TRNG Input Registers                     |
| TRNG_INTACK     | TRNG Interrupt Acknowledge Register      |
| TRNG_KEY[n]     | Post-Process Key Registers               |
| TRNG_LFSR_H     | TRNG LFSR Access Register                |
| TRNG_LFSR_L     | TRNG LFSR Access Register                |
| TRNG_LFSR_M     | TRNG LFSR Access Register                |
| TRNG_MONOBITCNT | TRNG Monobit Test Result Register        |
| TRNG_OUTPUT[n]  | TRNG Output Registers                    |
| TRNG_POKER[n]   | TRNG Poker Test Result Registers         |
| TRNG_RUNCNT     | TRNG Run Count Registers                 |
| TRNG_RUN[n]     | TRNG Run Test State and Result Registers |
| TRNG_STAT       | TRNG Status Register                     |
| TRNG_TEST       | TRNG Test Register                       |
| TRNG_V[n]       | TRNG Post-Process "V" Value Registers    |

## Random Number Generation

The random numbers that the TRNG generates are produced by sampling Free Running Oscillators (FRO). The ( TRNG\_CTL.STARTUPCYC ) bit field along with TRNG\_CFG.MINREFCYC (minimum refill cycles) and TRNG\_CFG.MAXREFCYC (maximum refill cycles) bit fields determine the number of samples taken to generate the first random value and subsequent random values.

1. After enabling the TRNG ( TRNG\_CTL.TRNGEN bit =1), a number of FRO output samples defined by the TRNG\_CTL.STARTUPCYC bit field are gathered in the main linear-feedback shift register (LFSR) before taking a snapshot of the LFSR and storing that snapshot in the random data buffer (after optional post-processing).

2. After taking a snapshot of the LFSR, a number of FRO output samples defined by the TRNG\_CFG.MINREFCYC bit field, are gathered in the main LFSR. If the random data buffer is full, sampletaking continues until the number of samples (counting from the snapshot) matches the number of samples defined by the TRNG\_CFG.MAXREFCYC bit field. At that point, the TRNG switches off the FROs and powers down.
3. If, after the TRNG\_CFG.MINREFCYC sampling period, the random data buffer is not completely filled, a new snapshot of the LFSR is taken and stored in the random data buffer (after optional postprocessing). Control branches back to point step 2 above and a new TRNG\_CFG.MINREFCYC sample period starts.

## Locking Detection and Prevention

Lock detection in functional mode uses the sampled outputs of the individual FROs. A FRO alarm event is declared when a repeating pattern (of up to four sample lengths) is detected continuously for the number of samples defined in the alarm threshold TRNG\_ALMCNT.ALMTHRESH bit field. The alarm event is logged by setting the bit that corresponds to the FRO that caused the alarm in the TRNG\_ALMMSK register. If that bit was already set, the corresponding bit in the TRNG\_ALMSTP register is set. The FRO is switched off to prevent further alarm events from that FRO. If the TRNG\_ALMMSK register bit was not yet set, the FRO is restarted automatically in an attempt to break locking.

The shutdown count field in the alarm count TRNG\_ALMCNT.SHDNCNT register monitors the number of FROs switched off. (It counts the number of 1 bits in the TRNG\_ALMSTP register.) The shutdown threshold field ( TRNG\_ALMCNT.SHDNTHRESH ) can be configured to generate the shutdown overflow interrupt ( TRNG\_STAT.SHDNOVR ). When the shutdown count in the TRNG\_ALMCNT.SHDNCNT bit field exceeds the shutdown threshold in the TRNG\_ALMCNT.SHDNTHRESH bit field, the shutdown overflow bit ( TRNG\_STAT.SHDNOVR ) is set to 1 (which can be used to generate an interrupt).

Software can use two strategies for the TRNG operation:

- Monitored Operation . Software checks the TRNG\_ALMMSK register at regular intervals (on the order of seconds). If a bit is set in that register, then the program must also check the TRNG\_ALMSTP register to determine if a FRO was shut down due to multiple alarm events. If no FROs are shut down, the program clears the TRNG\_ALMMSK register to remove the incidental alarm events. If one or more FROs are shut down, the host processor can modify the delay selection of those FROs using the TRNG\_FRODETUNE register to prevent further locking. For this type of operation, the shutdown threshold is normally set to a low value (two, for example). The shutdown overflow interrupt can then be used to signal abnormal operation conditions or the breakdown of FROs.
- Unmonitored Operation. Software sets the shutdown threshold to the acceptable number of FROs to be shut down before taking corrective actions. It then uses the shutdown overflow interrupt to initiate corrective actions (clearing the TRNG\_ALMMSK and TRNG\_ALMSTP registers, toggling bits in the TRNG\_FRODETUNE register). The software must monitor the time interval between these interrupts. If they occur too often (for example, within a minute after each other), this frequency indicates abnormal operating conditions or the breakdown of FROs.

## Run Testing

## Run Test

The TRNG block counts the number of consecutive zeros and ones (runs) in the data stream shifted into the main LFSR. The run length and bit value is then used to increment a specific bucket counter for these values. After 20,000 bits, the bucket counters must be within specified limits for this test to pass. If not, a run fail interrupt (RUNFAIL) is generated.

Table 43-2: Allowable Limits on Runs of 0's and 1's

| Run Count   |   Bit Value |   Min (inclusive) |   Max (inclusive) |
|-------------|-------------|-------------------|-------------------|
| 1           |           0 |              2267 |              2733 |
| 1           |           1 |              2267 |              2733 |
| 2           |           0 |              1079 |              1421 |
| 2           |           1 |              1079 |              1421 |
| 3           |           0 |               502 |               748 |
| 3           |           1 |               502 |               748 |
| 4           |           0 |               233 |               402 |
| 4           |           1 |               233 |               402 |
| 5           |           0 |                90 |               223 |
| 5           |           1 |                90 |               223 |
| 6 and up    |           0 |                90 |               233 |
| 6 and up    |           1 |                90 |               233 |

## Long Run Test

The long run test fails immediately when a run longer than 33 bits is found and a long run fail ( TRNG\_STAT.LRUNFAIL ) interrupt is generated.

## Noise Source Test

A noise source failure is declared when a run of 48 or more identical bits is found and a noise fail interrupt ( TRNG\_STAT.NOISEFAIL ) is generated.

The status and counts are stored in the TRNG\_RUNCNT and TRNG\_RUN[n] registers. Unless otherwise indicated, all counters and state bits in these registers are reset when writing a 1 to either the monobit fail acknowledge ( TRNG\_INTACK.MBITFAIL ), the run fail acknowledge ( TRNG\_INTACK.RUNFAIL ), or the poker fail acknowledge ( TRNG\_INTACK.PKRFAIL ) bits.

## Monobit Testing

The TRNG block performs the monobit test on blocks of 20,000 bits (in parallel with the run test and poker test). It monitors the number of zeros and ones in the data stream shifted into the main LFSR. At the start of the block,

the counter is initialized to 10,000. Each 1 value increments the counter, each 0 value decrements the counter. After 20,000 bits, the counter value must be within 9310-10690 (inclusive) for this test to pass. If not, a monobit fail interrupt ( TRNG\_STAT.MBITFAIL ) is generated. The AIS-31 standard (test T1, ref 0) specifies this run time testing of the TRNG and the parameters.

- NOTE: The actual limits stated here are different than the limits stated in the AIS-31 standard due to the implementation. The circuitry in the TRNG uses an up-down counter while the standard just evaluates the sum of the 20,000 bits.

When the continue poker ( TRNG\_TEST.CONTPKR ) bit is set to 1, the test is not stopped after 20,000 bits. The counter keeps incrementing and decrementing (protected against overflow and underflow). But, no actual limit checking happens. The offset from starting value 10,000 indicates the balance of 0 and 1 bits that were checked since the start of the continuous test. The offset is twice the number of missing or extra 1 bits. (An extra 1 bit adds an increment operation and removes one decrement operation. So, having 10,001 1 bits in the block gives a counter value of 10,002.)

## Poker Testing

The poker test is run in parallel with the monobit test and run test. Counters in the TRNG\_POKER[n] registers are used to count the occurrences of one specific 4-bit value in the data stream fed into the main LFSR. All of the counters are decremented by one every 64 data bits and reset to their start value every 20,000 bits. All counters start at a value of -1 and are decremented 312 times during the 20,000-bit test run.

Each 8-bit counter holds a two's complement value and does not overflow past the range of -128 to +127. At the end of the 20,000-bits block, the values of the counters that contain a single 1 bit appended at the least significant bit side are individually squared and then added. The poker test fails with a poker fail (PKRFAIL) interrupt when:

- the resulting sum (accumulated in the TRNG\_MONOBITCNT register) is outside the range 1288 - 71750 (inclusive), or
- one of the counters tries to increment or decrement outside its limit range
- NOTE: The poker test fails when the 4-bit values of the data stream are distributed too evenly (with eight counters having incremented 312 times and the others incremented 313 times). This failure is intentional. The minimum mean deviation from the expected value of 312.5 is 4.5. Failure at counter overflow is not an official part of the poker test as specified in the AIS-31 standard (ref 0). It can be shown that the maximum deviation for one counter's value from the mean value of 312.5 (without the poker test failing) is 129.5. Since this deviation is more than 40% of the mean value, it indicates that something is wrong. Here, the counter overflow failure is combined with the official poker test failure.

When the continue poker ( TRNG\_TEST.CONTPKR ) bit is set to 1, the test does not stop after 20,000 bits. The counters keep incrementing and decrementing (the latter every 64 bits). A PKRFAIL interrupt is generated when one of the counters tries to increment or decrement outside its limit range.

## Data for Tests

For the monobit test, run test and poker test circuits self-test, the test data written to the TRNG\_INPUT[n] 0 register is used for the monobit test and run test bit-by-bit. It executes from bit 31 down to bit 0. For the poker test, the written test data is used in eight blocks of 4 bits, starting from bits 31:28 down to bits 3:0.

When the TRNG\_TEST.CONTPKR bit =0 during run test and poker test circuits self-tests, the state of the poker and runs test status registers is frozen after all calculations are made and after inputting 20,000 test bits. This state allows time to read the test results. These calculations take around 20 clock cycles to complete. The status registers are reset to their starting states when the first word for the next test block of test bits is written to the TRNG\_INPUT[n] 0 register. Then, the contents of this word are processed.

## X9.31 Postprocessing

Postprocessing is available on some parts using the TRNG. If available, the postprocessing block is situated after the main LFSRs and before the output buffer that stores the random numbers for consumption. The online test logic for monobit, run, and poker testing is before the postprocessor block. The bits used for testing are from the main LFSR.

The postprocessor is based on the ANSI X9.31 specification using 3-Key 3DES cipher algorithm. It does not provide any more entropy than is what is already achieved from sampling the Free Running Oscillators (FROs). The use of the postprocessor only helps with applications requiring the use of an X9.31 compliant RNG.

The following section is an example of postprocessing.

The variables include:

- DT is a 64-bit date/time vector. This value is the input coming from random bits from the main LFSR.
- I is an intermediate value, a 64-bit value
- R is the final result, a 64-bit value
- V is a 64-bit seed value that is to be kept secret
- K is the 3-key for 3DES, each being 64-bits

The postprocessing uses the steps:

1. The intermediate value I is calculated: I = 3DES EDE (K, DT) with 3DES.
2. The result R is calculated: R = 3DES EDE (K, I XOR V).
3. Finally, V is updated: V = 3DES EDE (K, R XOR I).

## TRNG Block Diagram

The System Block Diagram of the TRNG diagram shows the system block for the TRNG. The system includes:

- Free Running Oscillators (FROs) that are the source of sampled bits
- A post-processing unit (processor dependent) for standards compliance

- Test circuitry to detect non-randomness due to failures in the system

Figure 43-1: System Block Diagram of the TRNG

<!-- image -->

## TRNG Architectural Concepts

The random numbers are accessible to the host processor in four 32-bit registers allowing a single burst read of a 128-bit random number. Acknowledging the data ready (interrupt) state causes the TRNG to move a new value (when available in the data buffer) to the TRNG output register. The TRNG always tries to keep the data buffer completely full. Pulling out data starts the regeneration of a new number by:

- Enabling the FROs
- Capturing their outputs in the LFSR, and
- Cryptographically postprocessing the values from the LFSR

The process produces new random values to replenish the buffer.

The major functional blocks of the TRNG module are:

- The actual TRNG core with control and test circuits, optional post-processor, and optional data buffer control logic
- Free Running Oscillators (FROs) instantiated outside the TRNG core
- A 128-byte buffer RAM instantiated outside the TRNG core

In the TRNG core, the true entropy source uses FROs as the basic building block. The accumulation of timing jitter, caused (for the largest part) by shot noise, creates uncertainty intervals for the output transitions of each FRO. Sampling within an uncertainty interval generates a single bit of entropy, which is 'accumulated' in a LFSR. As the uncertainty interval is very narrow compared to the cycle time of a FRO, the mean amount of entropy generated per sample is small (less than 1/100 bit per sample). To increase the entropy generation rate, multiple FROs are used in parallel.

The FROs are asynchronous to one another and asynchronous to the sampling clock to make their behavior truly non-deterministic.

The output signals of the FROs are sampled at regular intervals (in general, at the TRNG core module clock frequency). The samples feed into an error detection circuit that checks for repeating patterns coming out of a FRO. If a repeating pattern persists for a configurable number of samples, the FRO is suspect of having synchronized to (a harmonic of) the sampling interval. This activity drastically reduces the amount of entropy generated by that FRO. The error detection circuit signals this activity as a FRO error event.

Error events can occur during normal operation. The FRO control circuit attempts to restart a FRO that on a first error event. A second error event causes an automatic shutdown of the FRO. Because there are multiple FROs, shutting down a FRO reduces the amount of entropy generated, but it does not immediately jeopardize the TRNG operation. A limit can be configured below which the number of operational FROs does not drop. If this limit is crossed, an interrupt can be generated on the host processor. Software on the host processor can then attempt to prevent frequent locking of a FRO by de-tuning it to a slightly different frequency.

An XOR tree combines the sampled outputs and feeds them into an 81-bit LFSR to accumulate entropy and whiten the random bits stream.

NOTE: Here, whitening means balancing the number of one and zero bits in the stream.

## TRNG Operating Modes

The TRNG has the following operating modes.

## Normal Reading Mode

In normal reading mode, random data can only be read out of the output registers of the TRNG module when the ready bit ( TRNG\_STAT.RDY ) =1. Acknowledging the data by writing a 1 to the ready acknowledge bit in the TRNG\_INTACK.RDY register clears the ready bit from the status register and clears the output registers. The registers remain at zero until the next 128-bit data block is available.

## Secure Reading Mode

An attacker can try to read the output registers (without acknowledging the data) to obtain a copy of data to be read later by an application. To block this attack, secure reading mode is used. In this mode, enable reading from the output registers (by writing 0x00 to the open read gate bits ( TRNG\_INTACK.OPENRDGATE ) before it is possible to access the output registers. Enabling the reading starts a timeout (controlled by the TRNG\_CFG.RDTIMEOUT bit field). When this timeout expires, reading is disabled, and the offered data is acknowledged so that it is not offered again. The host processor must set this timeout such that there is enough time to read the output registers and perform a normal data acknowledge (which stops the timeout).

## Test Mode

In addition to the test circuitry that operates during normal operation, the TRNG has a test mode that allows further diagnosis when errors occur. In test mode, programs have access to the main LFSR through the

TRNG\_LFSR\_L , TRNG\_LFSR\_M , and TRNG\_LFSR\_H registers. Programs can also control the finite state machine sample counter using the TRNG\_CNT register.

In test mode, the TRNG can be configured to test individual or a chosen set of FROs. It can also be configured to feed test patterns to the delay chain.

## TRNG Data Transfer Modes

The host processor reads four 32-bit registers to access the random numbers. Once the registers are read and the data ready interrupt has been acknowledged, the TRNG moves more data from the internal buffer to the output registers ( TRNG\_OUTPUT[n] ).

## TRNG Event Control

There are eight events that the TRNG generates. The events are common error events from the run-time testing of the TRNG. While the TRNG is operating and generating random bits, test circuitry is also running statistical tests. The tests determine if the sources of the random bits have started to fail and are not truly producing random bits. There is also a single event to signal when data is ready in the output registers.

These events are captured in the TRNG\_STAT register and can generate a single interrupt in the Public Key Interrupt Controller (PKIC). The individual events can be masked so the interrupt is not triggered. This configuration uses the TRNG\_CTL register. The event and associated interrupt can be acknowledged in the TRNG\_INTACK register.

The TRNG Interrupt Signals table lists all events or interrupts that the TRNG can generate.

## TRNG Interrupt Signals

The TRNG provides a total of eight interrupts multiplexed into one output.

Table 43-3: TRNG Interrupt Signals

|   Interrupt | Name      | Description                                                                                                                                                                        |
|-------------|-----------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|           0 | RDY       | Ready. Random number is ready to be read from registers                                                                                                                            |
|           1 | SHDNOVR   | Shutdown Overflow. The number of FRO's automatically shut down due to failures or er- rors have gone above the threshold specified in TRNG_ALMCNT.SHDNTHRESH .                     |
|           2 | STUCKOUT  | Stuck Out. Logic circuitry around the output registers has detected the same output has been provided twice.                                                                       |
|           3 | NOISEFAIL | Noise Fail. Logic circuitry monitoring the data shifted into the main LFSR detected 48 identical bits, which is considered a noise source failure.                                 |
|           4 | RUNFAIL   | Run Fail. Logic circuitry monitoring the data shifted into the main LFSR detected an out- of-bounds value for at least one of the TRNG_RUN[n] counters after checking 20,000 bits. |

Table 43-3: TRNG Interrupt Signals (Continued)

|   Interrupt | Name     | Description                                                                                                                                                                                                                        |
|-------------|----------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|           5 | LRUNFAIL | Long Run Fail. Logic circuitry monitoring the data shifted into the main LFSR detected 34 identical bits.                                                                                                                          |
|           6 | PKRFAIL  | Poker Fail. Logic circuitry monitoring the data shifted into the main LFSR detected an out- of-bounds value in at least one of the 16 TRNG_POKER[n] counters or an out-of-bounds sum of squares values after checking 20,000 bits. |
|           7 | MBITFAIL | Monobit Fail. Logic circuitry monitoring the data shifted into the main LFSR detected an out-of-bounds number of 1's after checking 20,000 bits.                                                                                   |

## ADSP-2159x\_SC592\_SC594 TRNG Register Descriptions

True Random Number Generator (TRNG) contains the following registers.

Table 43-4: ADSP-2159x\_SC592\_SC594 TRNG Register List

| Name            | Description                              |
|-----------------|------------------------------------------|
| TRNG_ALMCNT     | TRNG Alarm Counter Register              |
| TRNG_ALMMSK     | TRNG Alarm Mask Register                 |
| TRNG_ALMSTP     | TRNG Alarm Stop Register                 |
| TRNG_BLKCNT     | TRNG Block Count Register                |
| TRNG_CFG        | TRNG Configuration Register              |
| TRNG_CNT        | Counter Register                         |
| TRNG_CTL        | TRNG Control Register                    |
| TRNG_FRODETUNE  | TRNG FRO De-tune Register                |
| TRNG_FROEN      | TRNG FRO Enable Register                 |
| TRNG_INPUT[n]   | TRNG Input Registers                     |
| TRNG_INTACK     | TRNG Interrupt Acknowledge Register      |
| TRNG_KEY[n]     | Post-Process Key Registers               |
| TRNG_LFSR_H     | TRNG LFSR Access Register                |
| TRNG_LFSR_L     | TRNG LFSR Access Register                |
| TRNG_LFSR_M     | TRNG LFSR Access Register                |
| TRNG_MONOBITCNT | TRNG Monobit Test Result Register        |
| TRNG_OUTPUT[n]  | TRNG Output Registers                    |
| TRNG_POKER[n]   | TRNG Poker Test Result Registers         |
| TRNG_RUNCNT     | TRNG Run Count Registers                 |
| TRNG_RUN[n]     | TRNG Run Test State and Result Registers |

Table 43-4: ADSP-2159x\_SC592\_SC594 TRNG Register List (Continued)

| Name      | Description                           |
|-----------|---------------------------------------|
| TRNG_STAT | TRNG Status Register                  |
| TRNG_TEST | TRNG Test Register                    |
| TRNG_V[n] | TRNG Post-Process "V" Value Registers |

## TRNG Alarm Counter Register

The TRNG\_ALMCNT register, together with the TRNG\_ALMMSK and TRNG\_ALMSTP registers, can be used by the host processor to determine if the FRO/sample cycle locking is a problem. Note that incidental alarm events are expected to occur during normal operation. This register also controls the way the monobit test and poker test circuits operate (using the standard 20,000 bit blocks or running continuously).

Figure 43-2: TRNG\_ALMCNT Register Diagram

<!-- image -->

Table 43-5: TRNG\_ALMCNT Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                        |
|--------------------|-------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:24 (R/NW)       | SHDNCNT     | Shutdown Count. This read-only field indicates the number of 1 bits in the TRNG_ALMSTP register, the number of FRO's that's been turned off.                                                                                                                                                                                                                   |
| 23 (R/W)           | SHDNFATAL   | Shutdown Fatal. When the TRNG_ALMCNT.SHDNFATAL bit field is set, the shutdown overflow (SHDNOVR) interrupt is considered a fatal error requiring taking the complete TRNG engine off-line.                                                                                                                                                                     |
| 20:16 (R/W)        | SHDNTHRESH  | Shutdown Threshold. The TRNG_ALMCNT.SHDNTHRESH bit field provides the threshold setting for gen- erating the shutdown overflow (SHDNOVR) interrupt, which is activated when the shutdown count ( TRNG_ALMCNT.SHDNCNT ) value in this register exceeds the threshold value set here.                                                                            |
| 15 (R/W)           | STALLRUNPKR | Stall Run Poker. When the TRNG_ALMCNT.STALLRUNPKR bit is set, stalls the Monobit Test, Run Test and Poker Test circuits when either the TRNG_STAT.MBITFAIL , TRNG_STAT.RUNFAIL or TRNG_STAT.PKRFAIL bits =1. This allows inspec- tion of the state of the result counters (which would otherwise be reset immediately for the next 20,000 bits block to test). |

Table 43-5: TRNG\_ALMCNT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | ALMTHRESH  | Alarm Threshold. The TRNG_ALMCNT.ALMTHRESH bit field sets the alarm detection threshold for the repeating pattern detectors on each FRO. A FRO alarm event is declared when a repeating pattern (of up to four samples length) is detected continuously for the num- ber of samples defined by this fields value. Reset value 255 (decimal) should keep the number of alarm events to a manageable level. |

## TRNG Alarm Mask Register

A set bit (=1) in the TRNG\_ALMMSK register signifies an alarm event and is used by the host processor to determine which of the individual FROs generated the alarm. If a bit in this register is set, the corresponding bit in the TRNG\_ALMSTP register is set and the FRO is turned off by clearing the corresponding bit in the TRNG\_FROEN register. If a bit is not set, the FRO restarts automatically to try to break sample cycle locking that could have caused the alarm event.

Figure 43-3: TRNG\_ALMMSK Register Diagram

<!-- image -->

Table 43-6: TRNG\_ALMMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0                | FROS       | FRO Alarm Mask. The TRNG_ALMMSK.FROS bit field provides logging for the alarm events of individ- ual FROs. A 1 in bit [n] indicates FRO n experienced an alarm event. |
| (R/W)              |            |                                                                                                                                                                       |

## TRNG Alarm Stop Register

The TRNG\_ALMSTP register is used by the host processor to determine which of the individual FROs generated more than one alarm event in quick succession. If a FRO generates an alarm event while a previous event is still logged in the TRNG\_ALMMSK register, the corresponding bit in this register is set (=1) and the FRO is turned off by clearing (=0) the corresponding bit in the TRNG\_FROEN register. The TRNG\_ALMCNT.SHDNCNT bit field keeps track of the number of bits that are set in this register.

Figure 43-4: TRNG\_ALMSTP Register Diagram

<!-- image -->

Table 43-7: TRNG\_ALMSTP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | FROS       | FRO Alarm Stop Bits. The TRNG_ALMSTP.FROS bit field provides logging for the alarm events of individ- ual FROs. A 1 in bit [n] indicates FRO n experienced more than one alarm event in quick succession and has been turned off. A 1 in this field forces the corresponding bit in the TRNG_FROEN register to 0. |

## TRNG Block Count Register

The TRNG\_BLKCNT register is the counter for the 128-bit blocks generated by the post-processor. These bits are forced to zero when the post-processor is disabled and are cleared to zero when an internal re-seed operation has finished. This register can be used by driver software to determine when to re-seed the post-processor.

The whole 32 bits of this register represent the amount of data (in bytes) generated since a re-seed. The TRNG\_BLKCNT register is only present when a post-processor is available.

Figure 43-5: TRNG\_BLKCNT Register Diagram

<!-- image -->

Table 43-8: TRNG\_BLKCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:4 (R/W)         | VALUE      | Block Count. The TRNG_BLKCNT.VALUE bit field is the counter for the 128-bit blocks generated by the post-processor. These bits are forced to zero when the post-processor is disabled and are cleared to zero when an internal re-seed operation has finished. |

## TRNG Configuration Register

The TRNG\_CFG register holds the lower and upper limits of the samples taken from the FROs in order to refill the random data buffer. This register also holds the time out value used for secure reading mode.

Figure 43-6: TRNG\_CFG Register Diagram

<!-- image -->

Table 43-9: TRNG\_CFG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | MAXREFCYC  | Max Refill Cycles. The TRNG_CFG.MAXREFCYC bit field determines the maximum number of sam- ples (between 2 8 and 2 24 ) taken to re-generate entropy from the FROs after reading out a 64-bit random number. If the written value of this field is zero, the number of samples is 2 24 , otherwise the number of samples equals the written value times 2 8 . This field can only be modified while the TRNG_CTL.TRNGEN bit =0.                                                                                                                    |
| 15:12 (R/W)        | RDTIMEOUT  | Read Timeout. The TRNG_CFG.RDTIMEOUT bit field controls the secure reading mode. When this field is 0, secure reading mode is disabled. Values in the range 1-15 enable secure read- ing and set a read gate closure timeout of approximately (read_timeout + 1) x 16 clock input cycles. This field can only be modified while the TRNG_CTL.TRNGEN bit =0.                                                                                                                                                                                       |
| 11:8 (R/W)         | SAMPLEDIV  | Sample Div. The TRNG_CFG.SAMPLEDIV bit field directly controls the number of input cycles between samples taken from the FROs. The default value 0 indicates that samples are taken every cycle, maximum value 15 (decimal) takes one sample every 16 cycles. This field must be set to a value such that the slowest FRO (even under worst-case condi- tions) has a cycle time less than twice the sample period. The default configuration of the FROs allows this field to remain 0. This field can only be modified while TRNG_CTL.TRNGEN =0. |

Table 43-9: TRNG\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | MINREFCYC  | Min Refill Cycles. The TRNG_CFG.MINREFCYC bit field determines the minimum number of samples (between 2 6 and 2 24 ) taken to re-generate entropy from the FROs after reading out a 64-bit random number. If the value of this field is zero, the number of samples is fixed to the value determined by the maximum refill cycles ( TRNG_CFG.MAXREFCYC ) field, otherwise the mini- mum number of samples equals the written value times 64 (which can be up to 2 14 ). The number of samples defined here cannot be higher than the number defined by the TRNG_CFG.MAXREFCYC field (i.e. that field takes precedence). This field can only be modified while the TRNG_CTL.TRNGEN bit =0. |

## Counter Register

The TRNG\_CNT register is used to access the main control Finite State Machine's (FSM) sample counter while the TRNG\_CTL.TSTMODE bit =1.

Figure 43-7: TRNG\_CNT Register Diagram

<!-- image -->

Table 43-10: TRNG\_CNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:0 (R/W)         | VALUE      | Sample Counter. The TRNG_CNT.VALUE bit field is the sample counter used by control finite state machine. This counter can only be accessed when the TRNG_CTL.TSTMODE bit =1. |

## TRNG Control Register

The TRNG\_CTL register must be written to start accumulating entropy before random numbers can be generated. In most cases, the TRNG\_CFG register must also be written prior to writing the TRNG\_CTL register. T o enable the TRNG, set the TRNG\_CTL.TRNGEN bit. This register also controls post-processing (if available). Note that when the TRNG\_CTL.TRNGEN bit =1, the start up cycles field ( TRNG\_CTL.STARTUPCYC ) and the post-processing enable bit (if available) are locked. Any writes to these fields are ignored.

Figure 43-8: TRNG\_CTL Register Diagram

<!-- image -->

Table 43-11: TRNG\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/W)        | STARTUPCYC | Startup Cycles. The TRNG_CTL.STARTUPCYC bit field determines the number of samples (be- tween 2 8 and 2 24 ) taken to gather entropy from the FROs during startup. If the writ- ten value of this field is zero, the number of samples is 2 24 , otherwise the number of samples equals the written value times 2 8 . This field can only be written when TRNG_CTL.TRNGEN =0 before the write. |

Table 43-11: TRNG\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | RESEED     | Re-seed. The TRNG_CTL.RESEED bit is set-only, writing a 1 starts a re-seed cycle and writing a 0 has no effect. A re-seed cycle entails loading the TRNG_KEY[n] and TRNG_V[n] registers with random values generated internally; these values are not visible outside the TRNG core. This bit falls back to 0 automatically after the re-seed operation is complete; at that time the TRNG_BLKCNT register is reset to zero and the random data buffer is zero- ized so that any new data read from the TRNG will use the new seed values. This bit is only present when post-processing is available and can only be set to 1 when TRNG_CTL.TRNGEN =1 before the write. Note that re-seeding can be done with the post-processor disabled (normally used to seed the post-processor before enabling it). When writing a 1 to this bit, all other bits in this register remain unchanged.                                                                                                                                                                                                          |
| 12 (R/W)           | PPROCEN    | Post Processor Enable. Setting the TRNG_CTL.PPROCEN bit enables the FIPS post-processor. If this bit is reset to 0, the post-processor is forced back into the idle state immediately. This bit is only present when post-processing is available and can only be changed when the TRNG_CTL.TRNGEN bit (enable TRNG) was 0 before the write. To change the TRNG_CTL.PPROCEN bit during operation, first put the TRNG into reset ( TRNG_CTL.TRNGEN =0). If the post-processor is enabled, it can be disabled by a subsequent write of 0 to this bit (writing this bit when the TRNG_CTL.TRNGEN bit is still 1 has no effect). The post-processor then stops immediately. To enable it, this bit bit must be written with 1. Changing the enabled/disabled state does not affect the contents of the TRNG_KEY[n] and TRNG_V[n] registers. After changing the state, the TRNG must be started again by setting the TRNG_CTL.TRNGEN bit to 1. Note that it is required to re-gather entropy, so the same number of start-up cycles must be used as when starting the TRNG out of a system reset state. |
| 10 (R/W)           | TRNGEN     | Enable TRNG. Setting the TRNG_CTL.TRNGEN bit to 1 starts the TRNG, gathering entropy from the FROs for the number of samples determined by the value in the TRNG_CTL.STARTUPCYC (Startup Cycles) field. Resetting this bit to 0 forces all TRNG logic back into the idle state immediately. Resetting this bit to 0 also performs the Un-instantiate operation, clearing all internal post-processor registers.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 8 (R/W)            | TSTMODE    | Test Mode. When the TRNG_CTL.TSTMODE bit is set, access is enabled to the TRNG_CNT and TRNG_LFSR_L , TRNG_LFSR_M and TRNG_LFSR_H registers (the latter are cleared before enabling access) and sets the TRNG_STAT.NEEDCLK bit for testing purposes. This bit must be set to 1 before various test modes in the TRNG_TEST reg- ister can be enabled.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

Table 43-11: TRNG\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                     |
|--------------------|--------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | MBITFAILMSK  | Monobit Fail Mask. When the TRNG_CTL.MBITFAILMSK bit is set, this mask allows the TRNG_STAT.MBITFAIL bit to activate the (active HIGH) interrupt output.    |
| 6 (R/W)            | PKRFAILMSK   | Poker Fail Mask. When the TRNG_CTL.PKRFAILMSK bit is set, this mask allows the TRNG_STAT.PKRFAIL bit to activate the (active HIGH) interrupt output.        |
| 5 (R/W)            | LRUNFAILMSK  | Long Run Fail Mask. When the TRNG_CTL.LRUNFAILMSK bit is set, this mask allows the TRNG_STAT.LRUNFAIL bit to activate the (active HIGH) interrupt output.   |
| 4 (R/W)            | RUNFAILMSK   | Run Fail Mask. When the TRNG_CTL.RUNFAILMSK bit is set, this mask allows the TRNG_STAT.RUNFAIL bit to activate the (active HIGH) interrupt output.          |
| 3 (R/W)            | NOISEFAILMSK | Noise Fail Mask. When the TRNG_CTL.NOISEFAILMSK bit is set, this mask allows the TRNG_STAT.NOISEFAIL bit to activate the (active HIGH) interrupt output.    |
| 2 (R/W)            | STUCKOUTMSK  | Stuck Out Mask. When the TRNG_CTL.STUCKOUTMSK bit is set, this mask allows the TRNG_STAT.STUCKOUT bit to activate the (active HIGH) interrupt output.       |
| 1 (R/W)            | SHDNOVRMSK   | Shutdown Overflow Mask. When the TRNG_CTL.SHDNOVRMSK bit is set, this mask allows the TRNG_STAT.SHDNOVR bit to activate the (active HIGH) interrupt output. |
| 0 (R/W)            | RDYMSK       | Ready Mask. When the TRNG_CTL.RDYMSK bit is set, this mask allows the TRNG_STAT.RDY bit to activate the (active HIGH) interrupt output.                     |

## TRNG FRO De-tune Register

The TRNG\_FRODETUNE register is used by the host processor to change the frequencies of individual FROs. This can reduce the number of alarm events generated by a specific FRO.

Figure 43-9: TRNG\_FRODETUNE Register Diagram

<!-- image -->

Table 43-12: TRNG\_FRODETUNE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | FROS       | FRO De-tune Bits. The TRNG_FRODETUNE.FROS bits De-tune the FROs. A 1 in bit [n] lets FRO n run approximately 5% faster. The value of one of these bits may only be changed while the corresponding FRO is turned off (by temporarily writing a 0 in the corresponding bit of the TRNG_FROEN register). |

## TRNG FRO Enable Register

The TRNG\_FROEN register can be used by the host processor to enable and disable FROs individually. Only enabled FROs contribute to entropy generation, but require power to do so. Disabled FROs cannot generate alarm events.

Figure 43-10: TRNG\_FROEN Register Diagram

<!-- image -->

Table 43-13: TRNG\_FROEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | FROS       | Enable Free-Running Oscillators. The TRNG_FROEN.FROS bits are the enables for the individual FROs. A 1 in bit [n] enables FRO n. The default state is all ones to enable all FROs after power-up. Note that the FROs are not actually started up before the TRNG_CTL.TRNGEN bit is set to 1. These bits are automatically forced to 0 (and cannot be written to 1) when the cor- responding bit in the TRNG_ALMSTP register has value 1. |

## TRNG Input Registers

The TRNG\_INPUT[n] registers are used as input for post-processor testing (if post processing is available) and as input for Monobit Test, Run Test and Poker Test functionality tests (TRNG\_INPUT0 only). They share their addresses with the corresponding TRNG\_OUTPUT[n] registers. The least significant word is contained in the TRNG\_INPUT0 register.

Figure 43-11: TRNG\_INPUT[n] Register Diagram

<!-- image -->

Table 43-14: TRNG\_INPUT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Input Bits for 32-bit Word n. The TRNG_INPUT[n].VALUE bit field is used to hold 32-bits of the 64-bit word of test data for 3-DES post-processor (if available). Or, the TRNG_INPUT[n].VALUE bit field is used to hold 32-bit data word for Run Test and Poker Test circuits self test. Can only be written to when the TRNG_STAT.TSTRDY bit =1. |

## TRNG Interrupt Acknowledge Register

The TRNG\_INTACK register is written to acknowledge interrupts indicated in bits [7:0] of the TRNG\_STAT register. Writing a 1 to any of the bits [7:2] has side effects in resetting various parts of the TRNG core logic which can also be used even if no interrupts are actually active.

When acknowledging the interrupts, these bits are write '1' to clear the associated bit in TRNG\_STAT register. The bit in this register will also automatically be reset to zero.

When secure reading mode is enabled, write bits [7:0] of this register with zeros to enable TRNG data reads from the TRNG\_OUTPUT[n] registers. Writing bits [7:0] also starts the (configurable) timeout counter that automatically acknowledges the TRNG data (and disables reads) if the TRNG\_INTACK.RDY bit is not written with a 1 within that timeout period.

Figure 43-12: TRNG\_INTACK Register Diagram

<!-- image -->

Table 43-15: TRNG\_INTACK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (RX/W1C)         | MBITFAIL   | Monobit Fail Acknowledge. Set the TRNG_INTACK.MBITFAIL bit to acknowledge the Monobit Fail Interrupt. This also resets all counter and state bits in the TRNG_RUN[n] , TRNG_MONOBITCNT and the TRNG_POKER[n] registers (except for the TRNG_RUNCNT.LENMAX field). |
| 6 (RX/W1C)         | PKRFAIL    | Poker Fail Acknowledge. Set the TRNG_INTACK.PKRFAIL bit to acknowledge the Poker Fail Interrupt. This also resets all counter and state bits in the TRNG_RUN[n] , TRNG_MONOBITCNT and the TRNG_POKER[n] registers (except for the TRNG_RUNCNT.LENMAX field).      |

Table 43-15: TRNG\_INTACK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (RX/W1C)         | LRUNFAIL   | Long Run Fail Acknowledge. Set the TRNG_INTACK.LRUNFAIL bit to acknowledge the Long Run Fail Inter- rupt. Also clears the TRNG_RUNCNT.LENMAX field.                                                                                                                                                                                                                  |
| 4 (RX/W1C)         | RUNFAIL    | Run Fail Acknowledge. Set the TRNG_INTACK.RUNFAIL bit to acknowledge the Run Fail Interrupt. Also resets all counter and state bits in the TRNG_RUN[n] , TRNG_MONOBITCNT and the TRNG_POKER[n] registers (except for the TRNG_RUNCNT.LENMAX field).                                                                                                                  |
| 3 (RX/W1C)         | NOISEFAIL  | Noise Fail Acknowledge. Set the TRNG_INTACK.NOISEFAIL bit to acknowledge the Noise Fail Interrupt. Setting this bit also clears the TRNG_RUNCNT.LENMAX (Run Length Max) field, the random data buffer, the TRNG_OUTPUT[n] registers and the TRNG_STAT.RDY bit.                                                                                                       |
| 2 (RX/W1C)         | STUCKOUT   | Stuck Out Acknowledge. Set the TRNG_INTACK.STUCKOUT bit to acknowledge the Stuck Out Interrupt. Setting this bit also clears the random data buffer, the TRNG_OUTPUT[n] registers and the TRNG_STAT.RDY bit.                                                                                                                                                         |
| 1 (RX/W1C)         | SHDNOVR    | Shutdown Overflow Acknowledge. Set the TRNG_INTACK.SHDNOVR bit to acknowledge the Shutdown Overflow In- terrupt.                                                                                                                                                                                                                                                     |
| 0 (RX/W1C)         | RDY        | Ready Acknowledge. The TRNG_INTACK.RDY bit allows a new number (if it is ready in the random data buffer), to directly move into the result register. Once done, the TRNG_STAT.RDY bit is reset, after at most size clock cycles.                                                                                                                                    |
| 7:0 (RX/W)         | OPENRDGATE | Open Read Gate. In Secure Reading Mode, the TRNG_INTACK.OPENRDGATE bit writes an all zeros value to bits [7:0] to enable reading of TRNG data from the TRNG_OUTPUT[n] registers. This starts the timeout counter that automatically acknowledges the TRNG data (and disables reading) if the TRNG_INTACK.RDY bit is not written with a 1 within that timeout period. |

## Post-Process Key Registers

The TRNG\_KEY[n] registers are used to load the key used for post-processing (if available). These registers are write-only. Reads return the values of the other registers mapped at the same addresses.

Figure 43-13: TRNG\_KEY[n] Register Diagram

<!-- image -->

Table 43-16: TRNG\_KEY[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                         |
|--------------------|------------|-------------------------------------------------|
| 31:0               | VALUE      | Key bits.                                       |
| (R/W)              |            | Bits for cipher key used in the post-processor. |

## TRNG LFSR Access Register

The TRNG\_LFSR\_H register is used to access bits [80:64] of the main entropy accumulation LFSR while in test mode ( TRNG\_CTL.TSTMODE =1).

For security reasons, the LFSR contents are zeroed before enabling access.

Figure 43-14: TRNG\_LFSR\_H Register Diagram

<!-- image -->

Table 43-17: TRNG\_LFSR\_H Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16:0 (R/W)         | VALUE      | LFSR[80:64]. The TRNG_LFSR_H.VALUE bit field contains bits [80:64] of the main entropy ac- cumulation LFSR. This field can only be accessed when the TRNG_CTL.TSTMODE bit =1. Contents are cleared (=0) before access is enabled. |

## TRNG LFSR Access Register

The TRNG\_LFSR\_L register is used to access bits [31:0] of the main entropy accumulation LFSR while in test mode ( TRNG\_CTL.TSTMODE =1).

For security reasons, the LFSR contents are zeroed before enabling access.

Figure 43-15: TRNG\_LFSR\_L Register Diagram

<!-- image -->

Table 43-18: TRNG\_LFSR\_L Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | LFSR[31:0]. The TRNG_LFSR_L.VALUE bit field contains bits [31:0] of the main entropy accu- mulation LFSR. This field can only be accessed when the TRNG_CTL.TSTMODE bit =1. Contents are cleared (=0) before access is enabled. |

## TRNG LFSR Access Register

The TRNG\_LFSR\_M register is used to access bits [63:32] of the main entropy accumulation LFSR while in test mode ( TRNG\_CTL.TSTMODE =1).

For security reasons, the LFSR contents are zeroed before enabling access.

Figure 43-16: TRNG\_LFSR\_M Register Diagram

<!-- image -->

Table 43-19: TRNG\_LFSR\_M Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | LFSR[63:32]. The TRNG_LFSR_M.VALUE bit field contains bits [63:32] of the main entropy ac- cumulation LFSR. This field can only be accessed when the TRNG_CTL.TSTMODE bit =1. Contents are cleared (=0) before access is enabled. |

## TRNG Monobit Test Result Register

The TRNG\_MONOBITCNT register accesses the counter used perform a Monobit Test as specified by the AIS-31 standard (test T1, ref 4). This test is performed on blocks of 20,000 bits (in parallel to the run test and Poker Test).

Note: Immediately after performing the actual Monobit Test at the end of the 20,000 bits block, the counter is used to accumulate the Poker Test results. As a result, the actual Monobit Test count result value can only be read in the TRNG\_MONOBITCNT register if the test fails and the stall run Poker ( TRNG\_ALMCNT.STALLRUNPKR ) bit =1.

The monobit test result register is read-only; writing it accesses the registers mapped at the same address. The counter in this register is reset when writing a 1 to either the monobit fail acknowledge ( TRNG\_INTACK.MBITFAIL ), run fail acknowledge ( TRNG\_INTACK.RUNFAIL ) or the poker fail acknowledge ( TRNG\_INTACK.PKRFAIL ) bits.

Figure 43-17: TRNG\_MONOBITCNT Register Diagram

<!-- image -->

Table 43-20: TRNG\_MONOBITCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16:0 (R/NW)        | VALUE      | Monobit Count. The TRNG_MONOBITCNT.VALUE bit field is the up/down counter which monitors 1 and 0 bits. After 20,000 bits, this counter should have a value in the range 9310 through 10690 (inclusive) to pass the Monobit Test. This counter is protected against overflow and underflow. |

## TRNG Output Registers

The TRNG\_OUTPUT[n] registers provide read access to the 128-bit random number output. A subset of these registers are also used as output for post-processor testing (if available). They share their addresses with the TRNG\_INPUT0 through TRNG\_INPUT3 registers. The least significant word is contained in the TRNG\_OUT-PUT0 register.

Figure 43-18: TRNG\_OUTPUT[n] Register Diagram

<!-- image -->

Table 43-21: TRNG\_OUTPUT[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Output Bits for 32-bit Word n. The TRNG_OUTPUT[n].VALUE bit field is used to holds 32 bits of the 128-bit word of random data. Only valid when the TRNG_STAT.RDY bit =1. Alternatively, this register holds the 32-bits of the 42-bit word of result data for 3-DES post-process- ing testing. Only valid when the TRNG_STAT.TSTRDY bit =1. |

## TRNG Poker Test Result Registers

The TRNG\_POKER[n] registers are used to access the 16 counters used perform a poker test on blocks of 20,000 bits (in parallel to the monobit and run tests).

Poker test result registers are read-only; writing them accesses the registers mapped at these same addresses. All counters in these registers are reset when writing a 1 to either the monobit fail acknowledge

( TRNG\_INTACK.MBITFAIL ), run fail acknowledge ( TRNG\_INTACK.RUNFAIL ) or the poker fail acknowledge ( TRNG\_INTACK.PKRFAIL ) bits.

Figure 43-19: TRNG\_POKER[n] Register Diagram

<!-- image -->

Table 43-22: TRNG\_POKER[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/NW)       | CNT3       | Poker Count n + 3. The TRNG_POKER[n].CNT3 bit field provides the counter for 4-bit value 0x3, 0x7, 0xB, 0xF in TRNG_POKER0, TRNG_POKER1, TRNG_POKER2, TRNG_POKER3, respectively. |
| 23:16 (R/NW)       | CNT2       | Poker Count n + 2. The TRNG_POKER[n].CNT2 bit field provides the counter for 4-bit value 0x2, 0x6, 0xA, 0xE in TRNG_POKER0, TRNG_POKER1, TRNG_POKER2, TRNG_POKER3, respectively. |
| 15:8 (R/NW)        | CNT1       | Poker Count n + 1. The TRNG_POKER[n].CNT1 bit field provides the counter for 4-bit value 0x1, 0x5, 0x9, 0xD in TRNG_POKER0, TRNG_POKER1, TRNG_POKER2, TRNG_POKER3, respectively. |
| 7:0 (R/NW)         | CNT0       | Poker Count n. The TRNG_POKER[n].CNT0 bit field provides the counter for 4-bit value 0x0, 0x4, 0x8, 0xC in the TRNG_POKER0, TRNG_POKER1, TRNG_POKER2, TRNG_POKER3, respectively. |

## TRNG Run Count Registers

The TRNG\_RUNCNT registers are used to access the 10 counters that perform a run test and long run test as specified by the AIS-31 standard (tests T3 and T4, ref 4). They are also used to perform the noise source failure test proposed in section E.5 of that same standard.

The TRNG\_RUNCNT registers are read-only; writing them accesses the other registers which are mapped at the same addresses. Unless otherwise indicated, all counters and state bits in these registers are reset when writing a 1 to either the Monobit Fail acknowledge ( TRNG\_INTACK.MBITFAIL ), Run Fail acknowledge

( TRNG\_INTACK.RUNFAIL ) or the Poker Fail acknowledge ( TRNG\_INTACK.PKRFAIL ) bits.

Figure 43-20: TRNG\_RUNCNT Register Diagram

<!-- image -->

Table 43-23: TRNG\_RUNCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:24 (R/NW)       | LENMAX     | Run Length Max. The TRNG_RUNCNT.LENMAX bit field configures the maximum run length count value encountered since start of test. This value is reset back to zero when writing a 1 to either the Noise Fail acknowledge ( TRNG_INTACK.NOISEFAIL ) or the Long Run Fail acknowledge ( TRNG_INTACK.LRUNFAIL ) bits.                |
| 21:16 (R/NW)       | LENCNT     | Run Length Br Count. The TRNG_RUNCNT.LENCNT bit field configures the counter for the current run of consecutive 0/1 bits; cannot increment past its maximum value of 63.                                                                                                                                                        |
| 15 (R/NW)          | STATE      | Run State. The TRNG_RUNCNT.STATE bit field provides th state of bits in the current run.                                                                                                                                                                                                                                        |
| 14:0 (R/NW)        | TSTCNT     | Run Test Count. The TRNG_RUNCNT.TSTCNT bit field configures the block length counter for the run and poker tests - counts up for 20,000 tested bits and then controls testing of the run_X_count_... and poker_count_X counters to contain expected values, after which they - and this counter - are reset for the next block. |

## TRNG Run Test State and Result Registers

The TRNG\_RUN[n] registers holds the counts for the associated run bucket for 1's and 0's.

Figure 43-21: TRNG\_RUN[n] Register Diagram

<!-- image -->

Table 43-24: TRNG\_RUN[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 27:16 (R/NW)       | CNTONES    | Count Ones. In TRNG_RUN1, this counter is for single bit runs of value one bits. After 20,000 bits, this counter should have a value in the range 2267 to 2733 (inclusive) to pass the run test. This counter cannot increment past its maximum value of 4095. In TRNG_RUN2, this counter is for two bit runs of value one bits. After 20,000 bits, this counter should have a value in the range 1079 to 1421 (inclusive) to pass the run test. This counter cannot increment past its maximum value of 2047. In TRNG_RUN3, this counter is for three bit runs of value one bits. After 20,000 bits, this counter should have a value in the range 502 to 748 (inclusive) to pass the run test. This counter cannot increment past its maximum value of 1023. In TRNG_RUN4, this counter for four bit runs of value one bits. After 20,000 bits, this counter should have a value in the range 233 to 402 (inclusive) to pass the run test. This counter cannot increment past its maximum value of 511. In TRNG_RUN5, this counter is for five bit runs of value one bits. After 20,000 bits, this counter should have a value in the range 90 to 223 (inclusive) to pass the run test. This counter cannot increment past its maximum value of 255. In TRNG_RUN6, this counter for six and higher bit runs of value one bits. After 20,000 bits, this counter should have a value in the range 90 to 233 (inclusive) to pass the run test. This counter cannot increment past its maximum value of 255. |

Table 43-24: TRNG\_RUN[n] Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:0 (R/NW)        | CNTZEROS   | Count Zeros. In TRNG_RUN1, this counter is for single bit runs of value zero bits. After 20,000 bits, this counter should have a value in the range 2267 to 2733 (inclusive) to pass the run test. This counter cannot increment past its maximum value of 4095. In TRNG_RUN2, this counter is for two bit runs of value zero bits. After 20,000 bits, this counter should have a value in the range 1079 to 1421 (inclusive) to pass the run test. This counter cannot increment past its maximum value of 2047. In TRNG_RUN3, this counter is for three bit runs of value zero bits. After 20,000 bits, this counter should have a value in the range 502 to 748 (inclusive) to pass the run test. This counter cannot increment past its maximum value of 1023. In TRNG_RUN4, this counter is for four bit runs of value zero bits. After 20,000 bits, this counter should have a value in the range 233 to 402 (inclusive) to pass the run test. This counter cannot increment past its maximum value of 511. In TRNG_RUN5, this counter is for five bit runs of value zero bits. After 20,000 bits, this counter should have a value in the range 90 to 223 (inclusive) to pass the run test. This counter cannot increment past its maximum value of 255. In TRNG_RUN6, this counter is for six and higher bit runs of value zero bits. After 20,000 bits, this counter should have a value in the range 90 to 233 (inclusive) to pass the run test. This counter cannot increment past its maximum value of 255. |

## TRNG Status Register

The TRNG\_STAT register provides status results. This register shares the same address as the Interrupt Acknowledge ( TRNG\_INTACK ) register.

Figure 43-22: TRNG\_STAT Register Diagram

<!-- image -->

Table 43-25: TRNG\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 24 (R/NW)          | NEEDCLK    | Need Clock. When the TRNG_STAT.NEEDCLK bit is set, it indicates that the TRNG is busy gen- erating entropy or is in one of its test modes; the module clock may not be turned off.                                                                                                                                                                                 |
| 23:16 (R/NW)       | BLKAVAIL   | Blocks Available. This field indicates the number of 128-bit blocks of random data that are available in the random data buffer. If this value is non-zero, the output registers will be refilled from the random data buffer immediately after acknowledging the TRNG_STAT.RDY by writing a 1 to TRNG_INTACK.RDY.                                                 |
| 8 (R/NW)           | TSTRDY     | Test Ready. When the TRNG_STAT.TSTRDY bit is set, it indicates that data for known-answer tests on the monobit test, run test, poker test and post-processor functions can be writ- ten to the TRNG_INPUT[n] registers. When testing the post-processor, result data can be read from those same registers when this bit has become 1 again (after dropping to 0). |

Table 43-25: TRNG\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/NW)           | MBITFAIL   | Monobit Fail. When the TRNG_STAT.MBITFAIL bit is set, the monobit test logic monitoring da- ta, shifted into the main LFSR, detected an out-of-bounds number of 1s after checking 20,000 bits (test T1 as specified in the AIS-31 standard).                                                                                   |
| 6 (R/NW)           | PKRFAIL    | Poker Fail. When the TRNG_STAT.PKRFAIL bit is set, the poker test logic monitoring data shifted into the main LFSR detected an out-of-bounds value in at least one of the 16 poker counters or an out of bounds "sum of squares" value after checking 20,000 bits (test T2 as specified in the AIS-31 standard).               |
| 5 (R/NW)           | LRUNFAIL   | Long Run Fail. When the TRNG_STAT.LRUNFAIL bit is set, the run test logic monitoring data shifted into the main LFSR detected a sequence of 34 identical bits (test T4 as speci- fied in the AIS-31 standard).                                                                                                                 |
| 4 (R/NW)           | RUNFAIL    | Run Fail. When the TRNG_STAT.RUNFAIL bit is set, the run test logic monitoring data shift- ed into the main LFSR detected an out-of-bounds value for at least one of the TRNG_RUN[n].CNTZEROS or TRNG_RUN[n].CNTONES counters after check- ing 20,000 bits (test T3 as specified in the AIS-31 standard).                      |
| 3 (R/NW)           | NOISEFAIL  | Noise Fail. When the TRNG_STAT.NOISEFAIL bit is set, the Run Test logic monitoring data shifted into the main LFSR detected a sequence of 48 identical bits, which is consid- ered a noise source failure as proposed in section E.5 of the AIS-31 standard.                                                                   |
| 2 (R/NW)           | STUCKOUT   | Stuck Out. When the TRNG_STAT.STUCKOUT bit is set, the logic around the output data reg- isters detected that the TRNG generates the same value twice in a row.                                                                                                                                                                |
| 1 (R/NW)           | SHDNOVR    | Shutdown Overflow. When the TRNG_STAT.SHDNOVR bit is set, the number of FROs shut down after a second error event (the number of 1 bits in the TRNG_ALMSTP register) has exceeded the threshold set by the TRNG_ALMCNT.SHDNTHRESH bit field.                                                                                   |
| 0 (R/NW)           | RDY        | Ready. When the TRNG_STAT.RDY bit is set, data is available in the TRNG_OUTPUT0 to TRNG_OUTPUT3 registers. If a new number is already available in the random data buffer, that number is directly moved into the result register. In this case the ready status bit is asserted again, after at most six module clock cycles. |

## TRNG Test Register

The TRNG\_TEST register can be used by the host processor to perform a number of tests on the TRNG logic including:

- Register controlled characterization by connecting the tst\_fro\_clk\_out output to a selected FRO clock output
- FRO logic connectivity and error event detection checking by feeding known patterns through the FRO delay line and error event detection circuits
- Direct XOR-ed FRO outputs capture by disabling the main LFSR feedback logic
- Extend the Monobit Test and Poker Test by not resetting the Monobit count and Poker Test X counters after each 20,000 bits block
- Perform known answer tests on the Run Test, Poker Test and post-processor functions.

Figure 43-23: TRNG\_TEST Register Diagram

<!-- image -->

Table 43-26: TRNG\_TEST Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | IRQ        | Test IRQ. When the TRNG_TEST.IRQ bit is set force irq output HIGH for interrupt signal connectivity testing. |

Table 43-26: TRNG\_TEST Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 27:16 (R/W)        | PATTERN    | Test Pattern. The TRNG_TEST.PATTERN bit field sets up a repeating sequence of bits to be fed into the selected FRO delay chain TRNG_TEST.PATTFRO =1 and/or the selected FRO error detection circuit TRNG_TEST.PATTDET =1. This field is rotated right over one bit, once every sample period, when either of these control bits is 1. There- fore, bit [16] is the actual pattern bit fed into the test target.                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 12:8 (R/W)         | SEL        | Test Select. The TRNG_TEST.SEL bit field configures the number of the FRO to be tested, the value should be in the range of 0 to 7.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 6 (R/W)            | PPROC      | Test Post Proc. When the TRNG_TEST.PPROC bit is set, it provides direct access to the post-pro- cessor for known-answer tests (writing input data to the TRNG_INPUT[n] registers). While this bit is set, the TRNG can continue to generate entropy in the main LFSR and any buffered random data is preserved, to be loaded into the output registers as soon as this bit is reset to 0 again. The need clock output is forced active while this bit is set. For X9.31 post-processors, it is advisable to re-seed the post-processor after running known-answer tests as the original key and V values are modified to known values. This bit is only present when post-processing is available and can only be set to 1 when the TRNG_CTL.PPROCEN bit =1 and the test run poker ( TRNG_TEST.RUNPKR ) bit in this register is 0. |
| 5 (R/W)            | RUNPKR     | Test Run Poker. When the TRNG_TEST.RUNPKR bit is set, it provides direct access to the inputs of the Monobit, Run and Poker Test circuits (writing input data in chunks of 32 bits to the TRNG_INPUT0 register). While this bit is 1, the TRNG is not allowed to gener- ate entropy but any buffered random data is preserved, to be loaded into the output registers as soon as this bit is reset to 0 again. The TRNG_STAT.NEEDCLK bit is forced active while this bit is 1. The Monobit, Run and Poker Test circuits are reset to their initial states on any change of this bit.                                                                                                                                                                                                                                               |
| 4 (R/W)            | CONTPKR    | Continue Poker. When the TRNG_TEST.CONTPKR bit is set, Monobit Test and Poker Test keep run- ning continuously by not resetting the Monobit count ( TRNG_MONOBITCNT ) and poker counters ( TRNG_POKER[n] register) at the end of each 20,000 bits test block. This bit can only be set to 1 when TRNG_CTL.TSTMODE =1. 0 Do not continue poker test                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 4 (R/W)            | CONTPKR    | 1 Continue poker test                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

Table 43-26: TRNG\_TEST Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                             | Description/Enumeration                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | NOLFSRFB   | No LFSR Feedback. When the TRNG_TEST.NOLFSRFB bit is set, it removes XNOR feedback from the main LFSR, converting it into a normal shift register for the XOR-ed outputs of the FROs (shifting data in on the LSB side). A 1 also forces the LFSR to sample continu- ously. This bit can only be set to 1 when TRNG_CTL.TSTMODE =1. | No LFSR Feedback. When the TRNG_TEST.NOLFSRFB bit is set, it removes XNOR feedback from the main LFSR, converting it into a normal shift register for the XOR-ed outputs of the FROs (shifting data in on the LSB side). A 1 also forces the LFSR to sample continu- ously. This bit can only be set to 1 when TRNG_CTL.TSTMODE =1. |
| 3 (R/W)            | NOLFSRFB   | 0                                                                                                                                                                                                                                                                                                                                   | Keep XNOR feedback                                                                                                                                                                                                                                                                                                                  |
| 3 (R/W)            | NOLFSRFB   | 1                                                                                                                                                                                                                                                                                                                                   | Remove XNOR feedback                                                                                                                                                                                                                                                                                                                |
| 2 (R/W)            | PATTDET    | Test Pattern Detect. When the TRNG_TEST.PATTDET bit is set, it repeatedly feeds test pattern (PAT- TERN) into the error detection circuit of the FRO selected by the test select ( TRNG_TEST.SEL ) field. This bit can only be set to 1 when TRNG_CTL.TSTMODE =1.                                                                   | Test Pattern Detect. When the TRNG_TEST.PATTDET bit is set, it repeatedly feeds test pattern (PAT- TERN) into the error detection circuit of the FRO selected by the test select ( TRNG_TEST.SEL ) field. This bit can only be set to 1 when TRNG_CTL.TSTMODE =1.                                                                   |
| 2 (R/W)            | PATTDET    | 0                                                                                                                                                                                                                                                                                                                                   | Do not repeat feed test pattern                                                                                                                                                                                                                                                                                                     |
| 2 (R/W)            | PATTDET    | 1                                                                                                                                                                                                                                                                                                                                   | Repeat feed test pattern                                                                                                                                                                                                                                                                                                            |
| 1 (R/W)            | PATTFRO    | Test Pattern FRO. When the TRNG_TEST.PATTFRO bit is set, it repeatedly feeds test pattern (PAT- TERN) into the delay chain of the FRO selected by the test select ( TRNG_TEST.SEL ) field by forcing the corresponding FRO enable (FROEN) out- put LOW. This bit can only be set to 1 when TRNG_CTL.TSTMODE =1.                     | Test Pattern FRO. When the TRNG_TEST.PATTFRO bit is set, it repeatedly feeds test pattern (PAT- TERN) into the delay chain of the FRO selected by the test select ( TRNG_TEST.SEL ) field by forcing the corresponding FRO enable (FROEN) out- put LOW. This bit can only be set to 1 when TRNG_CTL.TSTMODE =1.                     |
| 1 (R/W)            | PATTFRO    | 0                                                                                                                                                                                                                                                                                                                                   | Do not repeat feed test pattern                                                                                                                                                                                                                                                                                                     |
| 1 (R/W)            | PATTFRO    | 1                                                                                                                                                                                                                                                                                                                                   | Repeat feed test pattern                                                                                                                                                                                                                                                                                                            |
| 0 (R/W)            | ENOUT      | Test Enable Out. When the TRNG_TEST.ENOUT bit is set, it enables the tst_fro_clk_out output, con- necting to the FRO selected by the test select ( TRNG_TEST.SEL ) field. This bit can only be set to 1 when TRNG_CTL.TSTMODE =1.                                                                                                   | Test Enable Out. When the TRNG_TEST.ENOUT bit is set, it enables the tst_fro_clk_out output, con- necting to the FRO selected by the test select ( TRNG_TEST.SEL ) field. This bit can only be set to 1 when TRNG_CTL.TSTMODE =1.                                                                                                   |
| 0 (R/W)            | ENOUT      | 0                                                                                                                                                                                                                                                                                                                                   | Disable tst_fro_clk_out                                                                                                                                                                                                                                                                                                             |
| 0 (R/W)            | ENOUT      | 1                                                                                                                                                                                                                                                                                                                                   | Enable tst_fro_clk_out                                                                                                                                                                                                                                                                                                              |

## TRNG Post-Process "V" Value Registers

The TRNG\_V[n] registers are used to load the V value used for post-processing (if available). These registers are write-only. Reads return the values of the other registers mapped at the same addresses.

Figure 43-24: TRNG\_V[n] Register Diagram

<!-- image -->

Table 43-27: TRNG\_V[n] Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                |
|--------------------|------------|----------------------------------------|
| 31:0               | VALUE      | Holds "V" value for post-processing.   |
| (RX/W)             |            | Bits of the post-processing 'V' value. |