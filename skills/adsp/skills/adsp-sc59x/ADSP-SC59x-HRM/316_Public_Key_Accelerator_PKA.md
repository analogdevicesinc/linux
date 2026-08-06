# Public Key Accelerator (PKA)

<!-- source: 316_Public_Key_Accelerator_PKA.pdf | original pages 3295–3328 -->

## 42   Public Key Accelerator (PKA)

The PKA helps offload computationally-intensive operations commonly found in public key cryptography algorithms.

## PKA Features

The PKA engine provides the following basic operations:

- Large vector addition, subtraction, and combined addition/subtraction
- Large vector shift right or left
- Large vector multiplication, division (with and without quotient)
- Large vector compare and copy

The PKA engine provides the following complex operations:

- Large vector unsigned value modular exponentiation
- Large vector unsigned value modular exponentiation using the 'Chinese Remainders Theorem' (CRT) method with pre-calculated Q inverse vector
- Modular inversion: Given A and M, calculate B such that ((A × B) MOD M) = 1
- ECC point addition/doubling on elliptic curve y 2  = x 3  + ax + b (mod p) with prime number p and input values a and b to the operation. Adding two identical points automatically performs point doubling.
- ECC point multiplication on elliptic curve y 2  = x 3  + ax + b (mod p) with prime number p and input values a and b to the operation. A version of the 'Montgomery ladder' algorithm is used to provide side channel attack resistance.

The PKA also contains hardware logic to automatically zero out the PKA RAM buffer to clear out any information that is considered sensitive or secure.

## PKA Functional Description

The following sections provide details on the function of the PKA module.

## ADSP-SC59x PKA Register List

The Public Key Accelerator module (PKA) provides security-related features. A set of registers governs PKA operations. For more information on PKA functionality, see the PKA register descriptions.

Table 42-1: ADSP-SC59x PKA Register List

| Name          | Description                                   |
|---------------|-----------------------------------------------|
| PKA_ALEN      | PKA Vector_A Length                           |
| PKA_APTR      | PKA Vector_A Address                          |
| PKA_BLEN      | PKA Vector_B Length                           |
| PKA_BPTR      | PKA Vector_B Address                          |
| PKA_COMPARE   | PKA Compare Result                            |
| PKA_CPTR      | PKA Vector_C Address                          |
| PKA_DIVMSW    | PKA Most-Significant-Word of Divide Remainder |
| PKA_DPTR      | PKA Vector_D Address                          |
| PKA_FUNC      | PKA Function                                  |
| PKA_RAM       | Start of PKA RAM space                        |
| PKA_RESULTMSW | PKA Most-Significant-Word of Result Vector    |
| PKA_SHIFT     | PKA Bit Shift Value                           |

## PKA Definitions

The following definitions are helpful when using the PKA module.

## Elliptic Curve Cryptography (ECC)

A form of public key cryptography based on elliptic curves over finite fields.

## RSA

An acronym for Ron Rivest, Adi Shamir, and Leonard Adleman. It is another form of a public key cryptosystem.

## Chinese Remainder Theorem (CRT)

A mathematical theorem used for simplifying time-consuming arithmetic used in public key algorithm computations.

## Addition Chaining Table (ACT)

A method of speeding up exponentiation by repeatedly squaring the input, storing the result, and reusing the result as input. ACT2 uses a table with 2 address bits (4 entries) and ACT4 uses a table with 4 address bits (16 entries).

## PKA Architectural Concepts

The following sections describe the PKA architecture.

## Public Key Co-Processor (PKCP)

The Public Key Co-Processor (PKCP) handles the basic large vector processing such as addition, subtraction, multiplication, etc.

## Sequencer

The sequencer is small processor that is part of the PKA which handles the more complicated vector processing for public key algorithms. Algorithms include modular exponentiation and the ECC addition and ECC multiply used in Elliptic Curve Cipher algorithms. It executes instructions stored from an internal pre-programmed ROM that handles these operations.

## RAM

Input and output vectors are stored in a 4 kB RAM buffer that is part of the MMR space. The address of PKA\_RAM is the beginning of the RAM space. This memory is also used as a scratchpad or workspace for the sequencer and PKCP . Programs must place the vectors appropriately following the constraints described in the Functional Description section of this chapter.

## PKA Block Diagram

The PKA Block Diagram shows the top-level block diagram of the PKA engine. The PKA engine is comprised of five parts:

1. Registers for input, output, status, and control
2. Public Key Co-processor (PKCP) module which performs the basic suite of big number (vector) operations typically found in public key cryptography applications
3. Sequencer which controls modular exponentiation, elliptic curve cryptography, and modular inversion operations.
4. Program ROM associated with the PKA engine exclusively for the sequencer
5. PKA RAM holds the large input and output values as well as the workspace/scratchpad required from the sequencer and PKCP for operations.

Figure 42-1: PKA Block Diagram

<!-- image -->

## PKCP Vector Operations

The Summary of PKCP Vector Operations table lists the arguments and results for each PKCP vector operation.

Table 42-2: Summary of PKCP Vector Operations

| Function    | Mathematical Operation   | Vector A     | Vector B     | Vector C   | VectorD   |
|-------------|--------------------------|--------------|--------------|------------|-----------|
| Multiply    | A x B → C                | Multiplicand | Multiplier   | Product    | N/A       |
| Add         | A + B → C                | Addend       | Addend       | Sum        | N/A       |
| Subtract    | A - B → C                | Minuend      | Subtracthend | Difference | N/A       |
| AddSub      | A + C - B → D            | Addend       | Subtracthend | Addend     | Result    |
| Right Shift | A >> Shift → C           | Input        | N/A          | Result     | N/A       |
| Left Shift  | A << Shift → C           | Input        | N/A          | Result     | N/A       |
| Divide      | A mod B → C,             | Dividend     | Divisor      | Remainder  | Quotient  |
| Modulo      | A mod B → C              | Dividend     | Divisor      | Remainder  | N/A       |
| Compare     | A = B, A < B, A > B      | Input 1      | Input 2      | N/A        | N/A       |
| Copy        | A → C                    | Input        | N/A          | Result     | N/A       |

To obtain correct result, the input vectors must meet the requirements presented in the Operational Restrictions on Input Vectors for PKCP Operations table.

## Note the following:

- The PKCP does not check input restrictions
- A\_Len and B\_Len indicate the size of vectors A and B in 32-bit words
- Max\_Len equals 128 (32-bit) words, for example, the standard maximum vector size is 4096 bits

Table 42-3: Operational Restrictions on Input Vectors for PKCP Operations

| Function       | Requirement                                                                                                        |
|----------------|--------------------------------------------------------------------------------------------------------------------|
| Multiply       | 0 < A_Len, B_Len <= Max_Len                                                                                        |
| Add            | 0 < A_Len, B_Len <= Max_Len                                                                                        |
| Subtract       | 0 < A_Len, B_Len <= Max_Len Result must be positive (A>=B)                                                         |
| AddSub         | 0 < A_Len <= Max_Len (B and C operands have A_Len as length, B_Len ignored) Result must be positive ((A + C) >= B) |
| Right Shift    | 0 < A_Len <= Max_Len                                                                                               |
| Left Shift     | 0 < A_Len <= Max_Len                                                                                               |
| Divide, Modulo | 1 < B_Len <= A_Len <= Max_Len Most significant 32-bit word of B operand cannot be zero                             |
| Compare        | 0 < A_Len <= Max_Len (B operand has A_Len as length, B_Len ignored)                                                |
| Copy           | 0 < A_Len <= Max_Len                                                                                               |

The host processor is responsible for allocating a block of contiguous memory in PKA RAM for the result vectors. The PKCP Result Vector Memory Allocation table indicates how much memory is allocated for the result vectors.

Table 42-4: PKCP Result Vector Memory Allocation

| Function    | Result Vector   | Result Vector Length (in 32-bit words)                                    |
|-------------|-----------------|---------------------------------------------------------------------------|
| Multiply    | C               | A_Len + B_Len + 6 (the 6 'scratchpad' words should be discarded)          |
| Add         | C               | Max(A_Len, B_Len) + 1                                                     |
| Subtract    | C               | Max(A_Len, B_Len)                                                         |
| AddSub      | D               | A_Len + 1                                                                 |
| Right Shift | C               | A_Len                                                                     |
| Left Shift  | C               | A_Len + 1 (when Shift Value is non-zero) A_Len (when Shift Value is zero) |
| Divide      | C               | Remainder → B_Len + 1 (one 'scratchpad' word should be discarded)         |
| Divide      | D               | Quotient → A_Len - B_Len + 1                                              |
| Modulo      | C               | Remainder → B_Len + 1 (one 'scratchpad' word should be discarded)         |
| Compare     | None            | Compare updates the PKA_COMPARE register                                  |
| Copy        | C               | A_Len                                                                     |

Input vectors for an operation are always allowed to overlap in memory (partially or completely). The PKCP Result Vector/Input Overlap Restrictions table gives restrictions for the overlap of output and input vectors of the operations.

Table 42-5: PKCP Result Vector/Input Overlap Restrictions

| Function                | Result Vector   | Restrictions                                                                                                                                                |
|-------------------------|-----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Multiply                | C               | No overlap with A or B vectors allowed                                                                                                                      |
| Add, Subtract           | C               | May overlap with A and/or B vector, provided the start address of the C vector does not lie above the start address of the vectors with which it overlaps   |
| AddSub                  | D               | May overlap with A, B and/or C vector, provided the start address of the Dvector does not lie above the start address of the vectors with which it overlaps |
| Right Shift, Left Shift | C               | May overlap with A vector, provided the start address of the C vector does not lie above the start address of the A vector                                  |
| Divide                  | C               | No overlap with A, B, or Dvectors allowed                                                                                                                   |
| Divide                  | D               | No overlap with A, B, or C vectors allowed                                                                                                                  |
| Modulo                  | C               | No overlap with A or B vectors allowed                                                                                                                      |
| Compare                 | None            | Compare does not write a result vector                                                                                                                      |
| Copy                    | None            | Same restrictions as for right or left shift, copy of a vector to a lower address is always al- lowed even if source and destination overlap†               |

†The copy operation can be used to fill memory by breaking the overlap restrictions, but it requires setting up TWO initial (32-bit) words. To zero a block of memory, set the A vector pointer to the block start, set the C vector pointer two words higher and the A vector length to the block length minus two (words). Fill the first two words of the block with constant zero and perform a PKCP copy operation to zero the remainder of the block.

## Modular Exponentiation Operations

The Summary of ExpMod Operations table summarizes the modular exponentiation operations that the PKA supports.

Table 42-6: Summary of ExpMod Operations

| Function                                | Mathematical Opera- tion   | Vector A                                                                  | Vector B                                                                                | Vector C                 | VectorD                                         |
|-----------------------------------------|----------------------------|---------------------------------------------------------------------------|-----------------------------------------------------------------------------------------|--------------------------|-------------------------------------------------|
| ExpMod-ACT2 ExpMod-ACT4 ExpMod-variable | C A mod B → D              | Exponent, length = A_Len                                                  | Modulus, length = B_Len                                                                 | Base, length = B_Len     | Result and Workspace                            |
| ExpMod-CRT                              | See below                  | Exp P followed by Exp Qat next higher even word address†, both A_Len long | Mod P + buffer word followed by Mod Qat next higher even word address‡, both B_Len long | Qinverse, length = B_Len | Input, Result (both 2xB-Len long) and Workspace |

† If A\_Len is even, Exp Q follows Exp P immediately - if A\_Len is odd, there is one empty word between Exp Q and Exp P .

‡ If B\_Len is even, there are two empty words between Mod P and Mod Q - if B\_Len is odd, there is one empty (buffer) word between Mod Q and Mod P . Note that the engine may zero the words following Mod P and Mod Q.

The ExpMod-CRT operation performs the following computation steps. (These steps implement Garner's recombination algorithm after the basic exponentiations.)

- X &lt;- (Input mod Mod P)Exp P mod Mod P
- Y &lt;- (Input mod Mod Q)Exp Q mod Mod Q
- Z &lt;- ((((X - Y) mod Mod P) · Q inverse) mod Mod P) · Mod Q
- Result &lt;- Y + Z

The ExpMod-ACT2, -ACT4, and -variable functions implement the same mathematical operation but with a differently sized table with pre-calculated odd powers . The ExpMod-ACT2 function uses a table with two entries whereas ExpMod-ACT4 uses a table with eight entries. The ACT4 version gives better performance but needs more memory. ExpMod-variable and ExpMod-CRT operations allow the selection of a variable number (from 1 up to and including 16) of odd powers through the register normally used to specify the number of bits to shift for shift operations.

The exponentiation functions are extensions of the set of PKA functions. Input and result vectors are passed the same way as basic PKCP operations. The Restrictions on Input Vectors for ExpMod Operations table shows the restrictions on the input and result vectors for the exponentiation operations.

Table 42-7: Restrictions on Input Vectors for ExpMod Operations

| Function                                | Requirements                                                                                                                                                                                                                                                                                                                                                                                                           |
|-----------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ExpMod-ACT2 ExpMod-ACT4 ExpMod-variable | 0 < A_Len <= Max_Len 1 < B_Len <= Max_Len Modulus B must be odd (for example, the least significant bit must be ONE) Modulus B > 2 32 Base C < Modulus B Vectors B and C must be followed by an empty 32-bit buffer word                                                                                                                                                                                               |
| ExpMod-CRT                              | 0 < A_Len <= Max_Len 1 < B_Len <= Max_Len Mod P and Mod Qmust be odd (for example, the least significant bits must be ONE) Mod P > ModQ>2 32 (note that Mod P must be larger than Mod Q) Mod P and Mod Qmust be co-prime (their GCD must be 1) 0 < Exp P < (Mod P - 1) 0 < Exp Q<(Mod Q-1) (Q inverse · Mod Q) 1 (modulo Mod P) Input < (Mod P · Mod Q) Mod P and Mod Qmust be followed by an empty 32-bit buffer word |

The ExpMod Result Vector/Scratchpad Area Memory Allocation Starting at PKA\_DPTR table shows the required scratchpad sizes for the exponentiation operations. These sizes depend on the PKA type. The 'M\_Len' used in the table is the 'real' Modulus length in 32-bit words, for example, without trailing zero words at the end. (This description also applies to Mod P in an ExpMod-CRT operation and Modulus B in the other operations.) If the last word of the modulus vector as given is non-zero, 'M\_Len' equals B\_Len.

Table 42-8: ExpMod Result Vector/Scratchpad Area Memory Allocation Starting at PKA\_DPTR

| Function        | Scratchpad Area Size (in 32-bit words), Result Vector is either M_Len or 2xM_Len 32-bit words long   |
|-----------------|------------------------------------------------------------------------------------------------------|
| ExpMod-ACT2     | 5 x (M_Len + 2)                                                                                      |
| ExpMod-ACT4     | 11 x (M_Len + 2)                                                                                     |
| ExpMod-variable | (# odd powers + 3) x (M_Len + 2)                                                                     |
| ExpMod-CRT      | (# odd powers + 3) x (M_Len + 2) + (M_Len + 2 - (M_LenMOD 2))                                        |

NOTE: During execution of an ExpMod-ACT2, -ACT4 or -variable operation, the last 34 bytes of the PKA RAM are used as the general scratchpad for the sequencer program execution. The ExpMod-CRT operation requires the last 72 bytes of the PKA RAM as the scratchpad. These (fixed location) areas may not overlap with any of the input vectors and/or the D vector scratchpad area. They can be used freely when executing basic PKCP operations.

Table 42-9: ExpMod Scratchpad Area / Input Vector Overlap Restrictions

| Function                                | Result Vector   | Restrictions                                                                                                                                                                                        |
|-----------------------------------------|-----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ExpMod-ACT2 ExpMod-ACT4 ExpMod-variable | D               | Scratchpad area starting at Dmay not overlap with any of the other vec- tors, except that base C may be co-located with result vector Dto save space (for example, PKA_CPTR = PKA_DPTR is allowed). |
| ExpMod-CRT                              | D               | Scratchpad area starting at Dmay not overlap with any of the other vec- tors. This area is also the location of the main input vector (with length 2 x B_Len)                                       |

The Maximum Number of Odd Powers table indicates the maximum number of odd powers that can be used for different standard PKA RAM sizes and PKA types (non-CRT operations using PKA\_CPTR = PKA\_DPTR ). As a rule of thumb, for optimal performance, use one odd power for Verify operations and 4 (or as many as the implemented PKA RAM size allows) for Sign operations. Note the following points about odd powers:

- Using more than eight odd powers is not advisable as the speed advantage for each extra odd power decreases rapidly (and can even become negative for short exponent vector lengths due to the extra pre-processing required).
- The maximum number of odd powers is 16 (limited by the firmware). All '16 odd powers' entries in the table above hit this limit - they are not limited by the PKA RAM size.

Table 42-10: Maximum Number of Odd Powers

| Operation   | Modulus and Exponent Sizes   |   Maximum Number of Odd Powers |
|-------------|------------------------------|--------------------------------|
| Non-CRT     | 1024 bits                    |                             16 |
| Non-CRT     | 2048 bits                    |                             10 |
| Non-CRT     | 4096 bits                    |                              2 |
| CRT         | 2 × 512 bits                 |                             16 |
| CRT         | 2 ×1024 bits                 |                             16 |
| CRT         | 2 × 2048 bits                |                              6 |

The 2K-bit Modular Exponentiation PKA RAM Allocation Examples table shows example PKA RAM vector allocations for modular exponentiation operations with and without using CRT. The free space start address is the first free byte following the vector workspace. The sequencer execution scratchpad of 34 bytes (non-CRT) or 72 bytes (using CRT) must fit between this address and the end of the PKA RAM. Note that the non-CRT operations use PKA\_CPTR = PKA\_DPTR to save space.

Table 42-11: 2K-bit Modular Exponentiation PKA RAM Allocation Examples

| Operation                                                     | (sub-)vector     | Start address (Byte Offset)   | Size (words)            | Buffer (words)   |
|---------------------------------------------------------------|------------------|-------------------------------|-------------------------|------------------|
| non-CRT ( PKA_ALEN = 0x040, PKA_BLEN = 0x040, 4 odd-powers)   | Exponent         | 0x000 ( PKA_APTR = 0x000)     | 64                      | 0                |
| non-CRT ( PKA_ALEN = 0x040, PKA_BLEN = 0x040, 4 odd-powers)   | Modulus          | 0x100 ( PKA_BPTR = 0x040)     | 64                      | 2                |
| non-CRT ( PKA_ALEN = 0x040, PKA_BLEN = 0x040, 4 odd-powers)   | Base             | 0x208 ( PKA_CPTR = 0x082)     | 64                      | 2                |
| non-CRT ( PKA_ALEN = 0x040, PKA_BLEN = 0x040, 4 odd-powers)   | Result           | 0x208 ( PKA_DPTR = 0x082)     | 64                      | 2                |
| non-CRT ( PKA_ALEN = 0x040, PKA_BLEN = 0x040, 4 odd-powers)   | Vector Workspace | 0x208 (= Result)              | 7 × (64+2)=462          | 0                |
| non-CRT ( PKA_ALEN = 0x040, PKA_BLEN = 0x040, 4 odd-powers)   | Free space       | 0x940 (2368 bytes used)       | -                       | -                |
| using CRT ( PKA_ALEN = 0x020, PKA_BLEN = 0x020, 4 odd-powers) | Exp P            | 0x000 ( PKA_APTR = 0x000)     | 32                      | 0                |
| using CRT ( PKA_ALEN = 0x020, PKA_BLEN = 0x020, 4 odd-powers) | ExpQ             | 0x080                         | 32                      | 0                |
| using CRT ( PKA_ALEN = 0x020, PKA_BLEN = 0x020, 4 odd-powers) | Mod P            | 0x100 ( PKA_BPTR = 0x040)     | 32                      | 2                |
| using CRT ( PKA_ALEN = 0x020, PKA_BLEN = 0x020, 4 odd-powers) | ModQ             | 0x188                         | 32                      | 2                |
| using CRT ( PKA_ALEN = 0x020, PKA_BLEN = 0x020, 4 odd-powers) | Qinverse         | 0x210 ( PKA_CPTR = 0x084)     | 32                      | 0                |
| using CRT ( PKA_ALEN = 0x020, PKA_BLEN = 0x020, 4 odd-powers) | Input, Result    | 0x290 ( PKA_DPTR = 0x0A4)     | 64                      | 0                |
| using CRT ( PKA_ALEN = 0x020, PKA_BLEN = 0x020, 4 odd-powers) | Vector workspace | 0x290 (= Result)              | 7 × (32+2)+32+2-0 = 272 | 0                |
| using CRT ( PKA_ALEN = 0x020, PKA_BLEN = 0x020, 4 odd-powers) | Free space       | 0x6D0 (1744 bytes used)       | -                       | -                |

The following example in pseudo-code describes the execution of a non-CRT modular exponentiation operation using a 512-bit modulus and a 160-bit exponent, using actual test vectors:

```
// Perform a 512/160 bit modular exponentiation without CRT (using 4 'odd-powers') // Exponent equals value 0x8FD84098_8A0930CC_9CDC1E8A_B246EB46_2D39F064 // write as vector A to PKA
```

```
RAM Byte offset 0x000: Write PKA_RAM_BASE+0x000+0x00 0x2D39F064 Write PKA_RAM_BASE+0x000+0x04 0xB246EB46 Write PKA_RAM_BASE+0x000+0x08 0x9CDC1E8A Write PKA_RAM_BASE+0x000+0x0C 0x8A0930CC Write PKA_RAM_BASE+0x000+0x10 0x8FD84098 // Modulus equals value 0xF42F559D 1877CA5F_449492B9_42DC7C01_... // A3C9085B_7236A085_2102B000_A093C6B4_... // 9D0EDA0C_292DE841_29C23723_4048BDA3_... // 373C4C9F_45CF15A7_5F049ABF_D8A01B9B // write as vector B to PKA RAM Byte offset 0x018 (following exp at next aligned 64-bit word): Write PKA_RAM_BASE+0x018+0x00 0xD8A01B9B Write PKA_RAM_BASE+0x018+0x04 0x5F049ABF Write PKA_RAM_BASE+0x018+0x08 0x45CF15A7 Write PKA_RAM_BASE+0x018+0x0C 0x373C4C9F Write PKA_RAM_BASE+0x018+0x10 0x4048BDA3 Write PKA_RAM_BASE+0x018+0x14 0x29C23723 Write PKA_RAM_BASE+0x018+0x18 0x292DE841 Write PKA_RAM_BASE+0x018+0x1C 0x9D0EDA0C Write PKA_RAM_BASE+0x018+0x20 0xA093C6B4 Write PKA_RAM_BASE+0x018+0x24 0x2102B000 Write PKA_RAM_BASE+0x018+0x28 0x7236A085 Write PKA_RAM_BASE+0x018+0x2C 0xA3C9085B Write PKA_RAM_BASE+0x018+0x30 0x42DC7C01 Write PKA_RAM_BASE+0x018+0x34 0x449492B9 Write PKA_RAM_BASE+0x018+0x38 0x1877CA5F Write PKA_RAM_BASE+0x018+0x3C 0xF42F559D
```

```
// Base equals value 0x3D291F48_49064887_1149594B_67935110_... // 14EB8FF0_AB291F3A_54A1B4D1_5E611E44_... // C989251B_44904B45_0B060482_317F8352_... // 18CE440E_9BF509F1_6EAF26F2_95F19F12 // write as vector C to PKA RAM Byte offset 0x060 (following mod after buffer + align words): Write PKA_RAM_BASE+0x060+0x00 0x95F19F12 Write PKA_RAM_BASE+0x060+0x04 0x6EAF26F2 Write PKA_RAM_BASE+0x060+0x08 0x9BF509F1 Write PKA_RAM_BASE+0x060+0x0C 0x18CE440E Write PKA_RAM_BASE+0x060+0x10 0x317F8352 Write PKA_RAM_BASE+0x060+0x14 0x0B060482 Write PKA_RAM_BASE+0x060+0x18 0x44904B45 Write PKA_RAM_BASE+0x060+0x1C 0xC989251B Write PKA_RAM_BASE+0x060+0x20 0x5E611E44 Write PKA_RAM_BASE+0x060+0x24 0x54A1B4D1 Write PKA_RAM_BASE+0x060+0x28 0xAB291F3A Write PKA_RAM_BASE+0x060+0x2C 0x14EB8FF0 Write PKA_RAM_BASE+0x060+0x30 0x67935110 Write PKA_RAM_BASE+0x060+0x34 0x1149594B Write PKA_RAM_BASE+0x060+0x38 0x49064887 Write PKA_RAM_BASE+0x060+0x3C 0x3D291F48 // The result value and scratchpad (vector D) may be co-located with the base vector C for // a normal modular exponentiation, so these are located at PKA RAM Byte offset 0x060 too. // Load pointer and length registers: Write PKA_APTR 0x000>>2 // Exponent pointer Write PKA_BPTR 0x018>>2 // Modulus pointer Write PKA_CPTR 0x060>>2 // Base pointer
```

```
Write PKA_DPTR 0x060>>2 // Result/scratchpad pointer Write PKA_ALENGTH 0x00000005 // Exponent length in 32-bit words Write PKA_BLENGTH 0x00000010 // Mod/base/result length in 32-bit words // Start modular exponentiation and wait until it's done: Write PKA_SHIFT 0x00000004 // Number of 'odd powers' Write PKA_FUNCTION 0x0000E000 // 'Run' bit set, 'Sequencer Operations' = 0b110 Wait PKA_FUNCTION[15] == '0' // 'Run' bit clears itself - Host can also use interrupt! // Result value equals 0xA497BF8B_DB729088_954005B0_B5CA6691_... // A3EC491B_091A3D62_03C24214_0863A389_... // 0C7C03CD_2333E231_35EC10ED_8F91281C_... // 30F4253B_FE38FAFB_BB4A39DB_C14F2661 // written as vector D at PKA RAM Byte offset 0x060: Check PKA_RAM_BASE+0x060+0x00 == 0xC14F2661 Check PKA_RAM_BASE+0x060+0x04 == 0xBB4A39DB Check PKA_RAM_BASE+0x060+0x08 == 0xFE38FAFB Check PKA_RAM_BASE+0x060+0x0C == 0x30F4253B Check PKA_RAM_BASE+0x060+0x10 == 0x8F91281C Check PKA_RAM_BASE+0x060+0x14 == 0x35EC10ED Check PKA_RAM_BASE+0x060+0x18 == 0x2333E231 Check PKA_RAM_BASE+0x060+0x1C == 0x0C7C03CD Check PKA_RAM_BASE+0x060+0x20 == 0x0863A389 Check PKA_RAM_BASE+0x060+0x24 == 0x03C24214 Check PKA_RAM_BASE+0x060+0x28 == 0x091A3D62 Check PKA_RAM_BASE+0x060+0x2C == 0xA3EC491B Check PKA_RAM_BASE+0x060+0x30 == 0xB5CA6691 Check PKA_RAM_BASE+0x060+0x34 == 0x954005B0 Check PKA_RAM_BASE+0x060+0x38 == 0xDB729088
```

Check PKA\_RAM\_BASE+0x060+0x3C == 0xA497BF8B

## Modular Inversion

Besides modular exponentiation, the sequencer also controls modular inversion operations.

Table 42-12: Summary of ModInv Operation

| Function   | Mathematical Operation   | Vector A                    | Vector B                | Vector C   | VectorD              |
|------------|--------------------------|-----------------------------|-------------------------|------------|----------------------|
| ModInv     | A -1 mod B → D           | NumToInvert, length = A_Len | Modulus, length = B_Len | Not Used   | Result and Workspace |

The above function is an extension of the set of basic PKCP functions with the following exceptions:

- Vector D not only addresses the result, but also a workspace.
- The PKA\_SHIFT register field is used to return information on the operation's result.

Table 42-13: PKA\_SHIFT Result Values for ModInv Operation

| Function   | PKA_SHIFT Register Field Value At Conclusion                                                                                                                                                              |
|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ModInv     | 0 → success; VectorD holds result 7 → no inverse exists (GCD(A, B) != 1, for example, A and B have common factors); result undefined 31 → error, modulus even; result undefined other values are reserved |

The following tables list the restrictions on the input and result vectors for the ModInv operation.

Table 42-14: Operational Restrictions on Input Vectors for the ModInv Operation

| Function   | Requirements                                                                                                                                                                                                                                                                  |
|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ModInv     | 0 < A_Len <= Max_Len 0 < B_Len <= Max_Len Modulus B must be odd (for example, the least significant bit must be ONE) Modulus B may not have value 1 (result is undefined, no error indicated) The highest word of the modulus vector, as indicated by B_Len, may not be zero. |

Table 42-15: ModInv Scratchpad Area/Input Vector Overlap Restrictions

| Function   | Result Vector   | Restrictions                                                               |
|------------|-----------------|----------------------------------------------------------------------------|
| ModInv     | D               | Scratchpad area starting at Dmay not overlap with any of the other vectors |

The following table shows the required scratchpad sizes for the ModInv Operation.

Table 42-16: ModInv Result Vector/Scratchpad Area Memory Allocation (Both Starting at PKA\_DPTR)

| Function   | Scratchpad area size (in 32-bit words), Result Vector is B_Length 32-bit words long                                |
|------------|--------------------------------------------------------------------------------------------------------------------|
| ModInv     | 5 x (M + ε (M)), with M=Max(A_Length, B_Length) ε (n) = 2 + (n MOD2), for example, 2 (for n even) or 3 (for n odd) |

NOTE: During execution of a ModInv operation, the last 34 bytes of the PKA RAM are used as general scratchpad for the sequencer program execution. This (fixed location) area may not overlap with any of the input vectors and/or the D vector scratchpad area during execution.

## Modular Inversion with an Even Modulus

The ModInv operation requires the modulus to be odd. At first, this requirement appears to make the operation useless in the case of RSA key generation where the private key exponent d is derived from a chosen public exponent e as follows:

d = ModInv(e, φ ); where φ = (p-1) × (q-1) and p and q both prime

Note that φ is even. However, since e must be odd (otherwise no inverse exists), d can be calculated as:

d = (1 + ( φ × (e - ModInv( φ , e))) / e

With four more basic PKCP operations, ModInv can also be used to find inverse values in case the modulus is even.

## Modular Inversion with a Prime Modulus

Modular inversion can be performed with a modular exponentiation using the modulus value minus two as exponent, provided that the modulus value is a prime. This is due to the following:

(A M ) mod M = A =&gt;

(A M-1 ) mod M = 1 =&gt;

(A M-2 ) mod M = A-1 (mod M)

Under the constraint that M is a prime value.

Especially with the large PKA engines containing an LNME, it is worthwhile to check whether this method is faster than using the ModInv operation directly. The modulus values for the ECC curves supported by this PKA engine must be prime, so this method can be used in ECDSA operations.

## ECC Operations

Besides modular exponentiation and modular inversion, the sequencer also controls ECC operations.

Table 42-17: Summary of ECC Operations

| Function   | Mathematical Opera- tion                                                                    | Vector A                                                    | Vector B                                                             | Vector C                                   | VectorD                                                         |
|------------|---------------------------------------------------------------------------------------------|-------------------------------------------------------------|----------------------------------------------------------------------|--------------------------------------------|-----------------------------------------------------------------|
| ECC-ADD    | Point addition/ doubling† on elliptic curve: y 2 = x 3 + a x+ b (mod p ) pntA + pntC → pntD | pntA.x followed‡ by pntA.y both B_Len long (A_Len not used) | Curve parameter p fol- lowed‡ by a ( b is not needed) all B_Len long | pntC.x followed by pntC.y both B_Len long  | Result, for example, pntD.x followed‡ by pntD.y and work- space |
| ECC-MUL    | Point multiplication on elliptic curve: y 2 = x 3 + a x+b (mod p ) k x pntC → pntD          | Scalar k A_Len long                                         | Curve parameter p fol- lowed‡ by a and b all B_Len long.             | pntC.x followed‡ by pntC.y both B_Len long | Result, for example pntD.x followed‡ by pntD.y and work- space  |

‡ All input components must be located on a 64-bit boundary and must have extra 'buffer' words (of 32 bits each) after their most significant word. ε must be 3 (B\_Len odd) or 2 (B\_Len even). Each result component (for example, pntD.x, pntD.y) is followed by ε buffer (zero) words.

The above functions are extensions of the set of PKCP basic functions with the following exceptions:

- Input and result vectors can now be composite (for example, consist of two or three equal-sized subvectors).
- Vector D not only addresses the result, but also a workspace.
- The PKA\_SHIFT register is used to return information on the result of the operation.

Table 42-18: PKA\_SHIFT Result Values for ECC Operations

| Function   | PKA_SHIFT Register Field Value at Conclusion                                                                                                                        |
|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ECC-ADD    | 0 → success; VectorD holds result point                                                                                                                             |
| ECC-MUL    | 7 → result is point-at-infinity; VectorD result point undefined 31 → error, (p not odd, p too short, etc); VectorD result point undefined Other values are reserved |

The following tables below list the restrictions on the input and result vectors for the ECC operations.

Table 42-19: Operational Restrictions on Input Vectors for ECC Operations

| Function   | Requirements                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ECC-ADD    | 1 < B_Len <= 24 (maximum vector length is 768 bits) Modulus p must be a prime > 2 63 Effective modulus size (in bits) must be a multiple of 32† The highest word of the modulus vector, as indicated by B_Len, may not be zero. a < p and b < p pntA and pntC must be on the curve (this condition is not checked) Neither pntA nor pntC can be the point-at-infinity, although ECC-ADD can return this point as a result                              |
| ECC-MUL    | 0 < A_Len <= 24 (maximum vector length is 768 bits) 1 < B_Len <= 24 (maximum vector length is 768 bits) Modulus p must be a prime > 2 63 Effective modulus size (in bits) must be a multiple of 32† The highest word of the modulus vector, as indicated by B_Len, may not be zero. a < p and b < p pntC must be on the curve (this condition is not checked) pntC cannot be the point-at-infinity, although ECC-MUL can return this point as a result |

## † Modulus lengths of 112 and 521 bits are exceptions to this rule.

Table 42-20: ECC Scratchpad Area/Input Vector Overlap Restrictions

| Function   | Result Vector   | Restrictions                                                               |
|------------|-----------------|----------------------------------------------------------------------------|
| ECC-ADD    | D               | Scratchpad area starting at Dmay not overlap with any of the other vectors |
| ECC-MUL    |                 |                                                                            |

The &gt;ECC Result Vector/Scratchpad Area Memory Allocation table shows the required scratchpad sizes for the ECC operations:

Table 42-21: ECC Result Vector/Scratchpad Area Memory Allocation (Both Starting at PKA\_DPTR)

| Function   | Scratchpad area size (in 32-bit words), Result Vector is 2x(B_Length+ ε † (B_Length)) 32-bit words long   |
|------------|-----------------------------------------------------------------------------------------------------------|
| ECC-ADD    | 2 × L + 5 x M, where L = B_Length + ε (B_Length) M=B_Length + 1 + ε (B_Length + 1)                        |
| ECC-MUL    | 18 x L + Max(8, L), where L = (B_Length + ε (B_Length))                                                   |

NOTE: During execution of an ECC-ADD or ECC-MUL operation, the last 72 bytes of the PKA RAM are used as a general scratchpad for the sequencer program execution. These (fixed location) areas must not overlap with any of the input vectors and the D vector scratchpad area during execution.

The ECC Point Multiplication PKA RAM Allocation Examples table shows example PKA RAM vector allocations for ECC point multiplication operations. The free space start address is the first free byte following the vector scratchpad. The sequencer execution scratchpad of 72 bytes must fit between this address and the end of the PKA RAM. Because of this requirement, a 521-bit ECC point multiplication cannot be performed with 2K byte PKA RAM.

Table 42-22: ECC Point Multiplication PKA RAM Allocation Examples

| Modulus Length                                        | (sub-)vector      | Start Address (byte offset)   | Size (words)         | Buffer (words)   |
|-------------------------------------------------------|-------------------|-------------------------------|----------------------|------------------|
| 192 bits (=6 words, PKA_ALEN =0x006, PKA_BLEN =0x006) | Scalar k          | 0x000 ( PKA_APTR = 0x000)     | 6                    | 0                |
| 192 bits (=6 words, PKA_ALEN =0x006, PKA_BLEN =0x006) | p                 | 0x018 ( PKA_BPTR = 0x006)     | 6                    | 2                |
| 192 bits (=6 words, PKA_ALEN =0x006, PKA_BLEN =0x006) | a                 | 0x038                         | 6                    | 2                |
| 192 bits (=6 words, PKA_ALEN =0x006, PKA_BLEN =0x006) | b                 | 0x058                         | 6                    | 2                |
| 192 bits (=6 words, PKA_ALEN =0x006, PKA_BLEN =0x006) | PntC.x (base)     | 0x078 ( PKA_CPTR = 0x01E)     | 6                    | 2                |
| 192 bits (=6 words, PKA_ALEN =0x006, PKA_BLEN =0x006) | PntC.y (base)     | 0x098                         | 6                    | 2                |
| 192 bits (=6 words, PKA_ALEN =0x006, PKA_BLEN =0x006) | PntD.x (result)   | 0x0B8 ( PKA_DPTR = 0x02E)     | 6                    | 2                |
| 192 bits (=6 words, PKA_ALEN =0x006, PKA_BLEN =0x006) | PntD.y (result)   | 0x0D8                         | 6                    | 0                |
| 192 bits (=6 words, PKA_ALEN =0x006, PKA_BLEN =0x006) | Vector scratchpad | 0x0B8 (= PntD.x)              | (18 × 8) + 8 = 152   | 0                |
| 192 bits (=6 words, PKA_ALEN =0x006, PKA_BLEN =0x006) | Free space        | 0x318 (792 bytes used)        | -                    | -                |
| 384 bits (=12 words, ALENGTH=0x00C, BLENGTH=0x00C)    | Scalar k          | 0x000 ( PKA_APTR = 0x000)     | 12                   | 0                |
| 384 bits (=12 words, ALENGTH=0x00C, BLENGTH=0x00C)    | p                 | 0x030 ( PKA_BPTR = 0x00C)     | 12                   | 2                |
| 384 bits (=12 words, ALENGTH=0x00C, BLENGTH=0x00C)    | a                 | 0x068                         | 12                   | 2                |
| 384 bits (=12 words, ALENGTH=0x00C, BLENGTH=0x00C)    | b                 | 0x0A0                         | 12                   | 2                |
| 384 bits (=12 words, ALENGTH=0x00C, BLENGTH=0x00C)    | PntC.x (base)     | 0x0D8 ( PKA_CPTR = 0x036)     | 12                   | 2                |
| 384 bits (=12 words, ALENGTH=0x00C, BLENGTH=0x00C)    | PntC.y (base)     | 0x110                         | 12                   | 2                |
| 384 bits (=12 words, ALENGTH=0x00C, BLENGTH=0x00C)    | PntD.x (result)   | 0x148 ( PKA_DPTR = 0x052)     | 12                   | 2                |
| 384 bits (=12 words, ALENGTH=0x00C, BLENGTH=0x00C)    | PntD.y (result)   | 0x180                         | 12                   | 0                |
| 384 bits (=12 words, ALENGTH=0x00C, BLENGTH=0x00C)    | Vector scratchpad | 0x148 (= PntD.x)              | (18 × 14) + 14 = 266 | 0                |
| 384 bits (=12 words, ALENGTH=0x00C, BLENGTH=0x00C)    | Free space        | 0x570 (1392 bytes used)       | -                    | -                |
| 521 bits (=17 words, ALENGTH=0x011, BLENGTH=0x011)    | Scalar k          | 0x000 ( PKA_APTR = 0x000)     | 17                   | 1 (to align p )  |
| 521 bits (=17 words, ALENGTH=0x011, BLENGTH=0x011)    | p                 | 0x048 ( PKA_BPTR = 0x012)     | 17                   | 3                |
| 521 bits (=17 words, ALENGTH=0x011, BLENGTH=0x011)    | a                 | 0x098                         | 17                   | 3                |
| 521 bits (=17 words, ALENGTH=0x011, BLENGTH=0x011)    | b                 | 0x0E8                         | 17                   | 3                |
| 521 bits (=17 words, ALENGTH=0x011, BLENGTH=0x011)    | PntC.x (base)     | 0x138 ( PKA_CPTR = 0x04E)     | 17                   | 3                |
| 521 bits (=17 words, ALENGTH=0x011, BLENGTH=0x011)    | PntC.y (base)     | 0x188                         | 17                   | 3                |
| 521 bits (=17 words, ALENGTH=0x011, BLENGTH=0x011)    | PntD.x (result)   | 0x1D8 ( PKA_DPTR = 0x076)     | 17                   | 3                |

Table 42-22: ECC Point Multiplication PKA RAM Allocation Examples (Continued)

| Modulus Length   | (sub-)vector      | Start Address (byte offset)   | Size (words)         | Buffer (words)   |
|------------------|-------------------|-------------------------------|----------------------|------------------|
|                  | PntD.y (result)   | 0x228                         | 17                   | 0                |
|                  | Vector scratchpad | 0x1D8 (= PntD.x)              | (18 × 20) + 20 = 380 | 0                |
|                  | Free space        | 0x7C8 (1992 bytes used)       | -                    | -                |

The following example in pseudo-code describes the execution of a 192 bits ECC point multiplication, using actual test vectors (the curve parameters and generator point are from standard curve 'secp192r1').

```
// Perform a 192 bits ECC point multiplication using PKA RAM layout from table above. // Scalar 'k' equals value 0x8D98D058_9EFD018A_C9BCF3CF_2C33AEC0_24867D7F_6ADACBFF // write as vector A to PKA RAM Byte offset 0x000: Write PKA_RAM_BASE+0x000+0x00 0x6ADACBFF Write PKA_RAM_BASE+0x000+0x04 0x24867D7F Write PKA_RAM_BASE+0x000+0x08 0x2C33AEC0 Write PKA_RAM_BASE+0x000+0x0C 0xC9BCF3CF Write PKA_RAM_BASE+0x000+0x10 0x9EFD018A Write PKA_RAM_BASE+0x000+0x14 0x8D98D058 // Curve parameter 'p' equals value 0xFFFFFFFF_FFFFFFFF_FFFFFFFF_FFFFFFFE_FFFFFFFF_FFFFFFFF // write as 1st part of vector B immediately following vector A at PKA RAM Byte offset 0x018 // (no buffer word needed after 'k' vector, 64-bit alignment is OK): Write PKA_RAM_BASE+0x018+0x00 0xFFFFFFFF Write PKA_RAM_BASE+0x018+0x04 0xFFFFFFFF Write PKA_RAM_BASE+0x018+0x08 0xFFFFFFFE Write PKA_RAM_BASE+0x018+0x0C 0xFFFFFFFF Write PKA_RAM_BASE+0x018+0x10 0xFFFFFFFF Write PKA_RAM_BASE+0x018+0x14 0xFFFFFFFF // Curve parameter 'a' equals value 0xFFFFFFFF_FFFFFFFF_FFFFFFFF_FFFFFFFE_FFFFFFFF_FFFFFFFC // write as 2nd part of vector B after one buffer word and one re-alignment word at 0x038: Write PKA_RAM_BASE+0x038+0x00 0xFFFFFFFC Write PKA_RAM_BASE+0x038+0x04 0xFFFFFFFF Write PKA_RAM_BASE+0x038+0x08 0xFFFFFFFE Write PKA_RAM_BASE+0x038+0x0C 0xFFFFFFFF Write PKA_RAM_BASE+0x038+0x10 0xFFFFFFFF Write PKA_RAM_BASE+0x038+0x14 0xFFFFFFFF // Curve parameter 'b' equals value 0x64210519_E59C80E7_0FA7E9AB_72243049_FEB8DEEC_C146B9B1 // write as 3rd part of vector B after one buffer word and one re-alignment word at 0x058: Write PKA_RAM_BASE+0x058+0x00 0xC146B9B1
```

```
Write PKA_RAM_BASE+0x058+0x04 0xFEB8DEEC Write PKA_RAM_BASE+0x058+0x08 0x72243049 Write PKA_RAM_BASE+0x058+0x0C 0x0FA7E9AB Write PKA_RAM_BASE+0x058+0x10 0xE59C80E7 Write PKA_RAM_BASE+0x058+0x14 0x64210519 // X-coord of generator point is value 0x188DA80E_B03090F6_7CBF20EB_43A18800_F4FF0AFD_82FF1012 // write as 1st part of vector C following vector B after buffer + alignment words at 0x078: Write PKA_RAM_BASE+0x078+0x00 0x82FF1012 Write PKA_RAM_BASE+0x078+0x04 0xF4FF0AFD Write PKA_RAM_BASE+0x078+0x08 0x43A18800 Write PKA_RAM_BASE+0x078+0x0C 0x7CBF20EB Write PKA_RAM_BASE+0x078+0x10 0xB03090F6 Write PKA_RAM_BASE+0x078+0x14 0x188DA80E // Y-coord of generator point is value 0x07192B95_FFC8DA78_631011ED_6B24CDD5_73F977A1_1E794811 // write as 2nd part of vector C after one buffer word and one re-alignment word at 0x098: Write PKA_RAM_BASE+0x098+0x00 0x1E794811 Write PKA_RAM_BASE+0x098+0x04 0x73F977A1 Write PKA_RAM_BASE+0x098+0x08 0x6B24CDD5 Write PKA_RAM_BASE+0x098+0x0C 0x631011ED Write PKA_RAM_BASE+0x098+0x10 0xFFC8DA78 Write PKA_RAM_BASE+0x098+0x14 0x07192B95 // The result point and scratchpad (vector D) follow vector C after one buffer word and one // re-alignment word, so these are located at PKA RAM Byte offset 0x0B8. // Load pointer and length registers: Write PKA_APTR 0x000>>2 // Scalar 'k' pointer Write PKA_BPTR 0x018>>2 // Curve parameters 'p', 'a' & 'b' pointer Write PKA_CPTR 0x078>>2 // Generator point X & Y coordinates pointer Write PKA_DPTR 0x0B8>>2 // Result point X & Y coordinates/scratchpad pointer Write PKA_ALENGTH 0x00000006 // Scalar 'k' length in 32-bit words Write PKA_BLENGTH 0x00000006 // Curve parameters and coordinate lengths in 32bit words // Start ECC point multiplication and wait until it's done: Write PKA_FUNCTION 0x0000D000 // 'Run' bit set, 'Sequencer Operations' = 0b101 Wait PKA_FUNCTION[15] == '0' // 'Run' bit clears itself - Host can also use interrupt! Check PKA_SHIFT == 0x00000000 // Shift field value 0 indicates success - check this // X-coord of result point is value 0x759B9F39_0E81D268_18C82BB9_CB42BCF5_0E0AE958_85BA3097 // written as 1st part of vector D at PKA RAM Byte offset 0x0B8: Check PKA_RAM_BASE+0x0B8+0x00== 0x85BA3097 Check PKA_RAM_BASE+0x0B8+0x04== 0x0E0AE958 Check PKA_RAM_BASE+0x0B8+0x08== 0xCB42BCF5 Check PKA_RAM_BASE+0x0B8+0x0C== 0x18C82BB9
```

```
Check PKA_RAM_BASE+0x0B8+0x10== 0x0E81D268 Check PKA_RAM_BASE+0x0B8+0x14== 0x759B9F39 // Y-coord of result point is value 0xECA14640_F92EFF07_CAF2BD55_3FBE28EF_D043F28E_1CC3D238 // written as 2nd part of vector D at PKA RAM Byte offset 0x0D8: Check PKA_RAM_BASE+0x0D8+0x00== 0x1CC3D238 Check PKA_RAM_BASE+0x0D8+0x04== 0xD043F28E Check PKA_RAM_BASE+0x0D8+0x08== 0x3FBE28EF Check PKA_RAM_BASE+0x0D8+0x0C== 0xCAF2BD55 Check PKA_RAM_BASE+0x0D8+0x10== 0xF92EFF07 Check PKA_RAM_BASE+0x0D8+0x14== 0xECA14640
```

## ADSP-SC59x PKA Register Descriptions

Public Key Accelerator (PKA) contains the following registers.

Table 42-23: ADSP-SC59x PKA Register List

| Name          | Description                                   |
|---------------|-----------------------------------------------|
| PKA_ALEN      | PKA Vector_A Length                           |
| PKA_APTR      | PKA Vector_A Address                          |
| PKA_BLEN      | PKA Vector_B Length                           |
| PKA_BPTR      | PKA Vector_B Address                          |
| PKA_COMPARE   | PKA Compare Result                            |
| PKA_CPTR      | PKA Vector_C Address                          |
| PKA_DIVMSW    | PKA Most-Significant-Word of Divide Remainder |
| PKA_DPTR      | PKA Vector_D Address                          |
| PKA_FUNC      | PKA Function                                  |
| PKA_RAM       | Start of PKA RAM space                        |
| PKA_RESULTMSW | PKA Most-Significant-Word of Result Vector    |
| PKA_SHIFT     | PKA Bit Shift Value                           |

## PKA Vector\_A Length

During execution of basic PKCP operations, the PKA\_ALEN register is double buffered and can be written with a new value for the next operation. When not written, the value remains intact. During the execution of sequencer controlled complex operations, the PKA\_ALEN register may not be written and its value is undefined at the conclusion of the operation. The driver software cannot rely on the written value to remain intact.

Figure 42-2: PKA\_ALEN Register Diagram

<!-- image -->

Table 42-24: PKA\_ALEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration               |
|--------------------|------------|---------------------------------------|
| 8:0                | VALUE      | Length of Vector A.                   |
| (R/W)              |            | Length (in 32-bit words) of Vector A. |

## PKA Vector\_A Address

During execution of basic PKCP operations, the PKA\_APTR register is double buffered and can be written with a new value for the next operation. When not written, the value remains intact. During the execution of sequencer controlled complex operations, the PKA\_APTR register may not be written and its value is undefined at the conclusion of the operation. The driver software cannot rely on the written value to remain intact.

Figure 42-3: PKA\_APTR Register Diagram

<!-- image -->

Table 42-25: PKA\_APTR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10:0 (R/W)         | VALUE      | Pointer to Vector A. The PKA_APTR.VALUE bit field is the location of Vector A within the PKA RAM. Vectors are identified through the location of their least-significant 32-bit word. Note that bit [0] must be zero to ensure that the vector starts at an 8-byte boundary. |

## PKA Vector\_B Length

During execution of basic PKCP operations, the PKA\_BLEN register is double buffered and can be written with a new value for the next operation. When not written, the value remains intact. During the execution of sequencer controlled complex operations, the PKA\_BLEN register may not be written and its value is undefined at the conclusion of the operation. The driver software cannot rely on the written value to remain intact.

Figure 42-4: PKA\_BLEN Register Diagram

<!-- image -->

Table 42-26: PKA\_BLEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration               |
|--------------------|------------|---------------------------------------|
| 8:0                | VALUE      | Length of Vector B.                   |
| (R/W)              |            | Length (in 32-bit words) of Vector B. |

## PKA Vector\_B Address

During execution of basic PKCP operations, the PKA\_BPTR register is double buffered and can be written with a new value for the next operation. When not written, the value remains intact. During the execution of sequencer controlled complex operations, the PKA\_BPTR register may not be written and its value is undefined at the conclusion of the operation. The driver software cannot rely on the written value to remain intact.

Figure 42-5: PKA\_BPTR Register Diagram

<!-- image -->

Table 42-27: PKA\_BPTR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10:0 (R/W)         | VALUE      | Pointer to Vector B. The PKA_BPTR.VALUE bit field is the location of Vector B within the PKA RAM. Vectors are identified through the location of their least-significant 32-bit word. Note that bit [0] must be zero to ensure that the vector starts at an 8-byte boundary. |

## PKA Compare Result

The PKA\_COMPARE register provides the result of a basic PKCP Compare operation. It is updated when the PKA\_FUNC.RUN bit is reset at the end of that operation. The status after a complex sequencer operation is unknown.

Figure 42-6: PKA\_COMPARE Register Diagram

<!-- image -->

Table 42-28: PKA\_COMPARE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/NW)           | AGTB       | Vector A is Greater Than Vector B. The PKA_COMPARE.AGTB bit shows the result of the basic compare operation is PKCP Vector_A is greater than Vector_B. |
| 1 (R/NW)           | ALTB       | Vector A is Less Than Vector B. The PKA_COMPARE.ALTB bit shows the result of the basic compare operation is Vector_A is less than Vector_B.            |
| 0 (R/NW)           | AEQB       | Vector A is equal to Vector B. The PKA_COMPARE.AEQB bit shows the result of the basic compare operation is Vector_A is equal to Vector_B.              |

## PKA Vector\_C Address

During execution of basic PKCP operations, the PKA\_CPTR register is double buffered and can be written with a new value for the next operation. When not written, the value remains intact. During the execution of sequencer controlled complex operations, the PKA\_CPTR register may not be written and its value is undefined at the conclusion of the operation. The driver software cannot rely on the written value to remain intact.

Figure 42-7: PKA\_CPTR Register Diagram

<!-- image -->

Table 42-29: PKA\_CPTR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10:0 (R/W)         | CPTR       | Pointer to Vector C. The PKA_CPTR.CPTR bit field is the location of Vector C within the PKA RAM. Vectors are identified through the location of their least-significant 32-bit word. Note that bit [0] must be zero to ensure that the vector starts at an 8-byte boundary. |

## PKA Most-Significant-Word of Divide Remainder

The PKA\_DIVMSW register indicates the (32-bit word) address in the PKA RAM where the most significant nonzero 32-bit word of the Remainder result for the basic Divide and Modulo operations is stored. Bits [4:0] are loaded with the bit number of the most significant non-zero bit in the most significant non-zero word when MS one control bit is set. For Divide, Modulo and MS one reporting, this register is updated when the PKA\_FUNC.RUN bit is reset at the end of the operation.

For the complex sequencer controlled operations, updating bits [4:0] of this register with the actual result's most significant bit location is done near the end of the operation. Note that the result is only meaningful if no errors were detected and that for ECC operations, the PKA\_DIVMSW register provides information for the x-coordinate of the result point only.

Figure 42-8: PKA\_DIVMSW Register Diagram

<!-- image -->

Table 42-30: PKA\_DIVMSW Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/NW)          | ZERO       | Remainder Result Vector is Zeros. The PKA_DIVMSW.ZERO bit shows the remainder result vector is all zeros, ignore the address returned in bits [10:0].                        |
| 10:0 (R/NW)        | ADDR       | Address of Most-significant Nonzero Word. The PKA_DIVMSW.ADDR bit shows the address of the most significant non-zero 32- bit word of the remainder result vector in PKA RAM. |

## PKA Vector\_D Address

During execution of basic PKCP operations, the PKA\_DPTR register is double buffered and can be written with a new value for the next operation. When not written, the value remains intact. During the execution of sequencer controlled complex operations, the PKA\_DPTR register may not be written and its value is undefined at the conclusion of the operation. The driver software cannot rely on the written value to remain intact.

Figure 42-9: PKA\_DPTR Register Diagram

<!-- image -->

Table 42-31: PKA\_DPTR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10:0 (R/W)         | VALUE      | Pointer to Vector D. The PKA_DPTR.VALUE bit field is the location of Vector Dwithin the PKA RAM. Vectors are identified through the location of their least-significant 32-bit word. Note that bit [0] must be zero to ensure that the vector starts at an 8-byte boundary. |

## PKA Function

The PKA\_FUNC register contains the control bits to start basic PKCP as well as complex sequencer operations. The PKA\_FUNC.RUN bit can be used to poll for the completion of the operation. Modifying bits [11:0] is made impossible during the execution of a basic PKCP operation.

During the execution of Sequencer controlled complex operations, this register is modified - the PKA\_FUNC.RUN and PKA\_FUNC.STALLRSLT bits are set to zero at the conclusion, but other bits are undefined.

Continuously reading this register to poll the PKA\_FUNC.RUN bit is NOT allowed when executing complex sequencer operations (the sequencer cannot access the PKCP when this is done).

Leave at least one SCLK cycle between poll operations.

<!-- image -->

Stall Result

Figure 42-10: PKA\_FUNC Register Diagram

Table 42-32: PKA\_FUNC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 24                 | STALLRSLT  | Stall Result.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Stall Result.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 15 (R/W)           | RUN        | operations. Run. Set the PKA_FUNC.RUN bit to instruct the PKA module to begin processing the ba- sic PKCP or complex Sequencer operation. This bit is reset low automatically when the operation is complete. The complement of this bit is output as the pkaint1 inter- rupt. After a reset, the Run bit is always set to 1b but the first Sequencer firmware instruc- tion sets this bit to 0 immediately after the hardware reset is released. A few clock cy- cles are needed before the first instruction is executed and the Run bit state has been propagated. | operations. Run. Set the PKA_FUNC.RUN bit to instruct the PKA module to begin processing the ba- sic PKCP or complex Sequencer operation. This bit is reset low automatically when the operation is complete. The complement of this bit is output as the pkaint1 inter- rupt. After a reset, the Run bit is always set to 1b but the first Sequencer firmware instruc- tion sets this bit to 0 immediately after the hardware reset is released. A few clock cy- cles are needed before the first instruction is executed and the Run bit state has been propagated. |
| 14:12 (R/W)        | SEQOPS     | Sequencer Operation Select. The PKA_FUNC.SEQOPS bit field select the complex Sequencer operation to per- form.                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Sequencer Operation Select. The PKA_FUNC.SEQOPS bit field select the complex Sequencer operation to per- form.                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 14:12 (R/W)        | SEQOPS     | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | None                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 14:12 (R/W)        | SEQOPS     | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | ExpMod-CRT                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 14:12 (R/W)        | SEQOPS     | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | ExpMod-ACT4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 14:12 (R/W)        | SEQOPS     | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | ECC-ADD                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 14:12 (R/W)        | SEQOPS     | 4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | ExpMod-ACT2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 14:12 (R/W)        | SEQOPS     | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | ECC-MUL                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 14:12 (R/W)        | SEQOPS     | 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | ExpMod-variable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 14:12 (R/W)        | SEQOPS     | 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | ModInv                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 11 (R/W)           | CPY        | Copy. PKA_FUNC.CPY                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | bit performs the copy                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 10 (R/W)           | CMP        | Perform Compare Operation. Setting the PKA_FUNC.CMP bit performs the compare operation. For more informa-                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Perform Compare Operation. Setting the PKA_FUNC.CMP bit performs the compare operation. For more informa-                                                                                                                                                                                                                                                                                                                                                                                                                                                             |

Table 42-32: PKA\_FUNC Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | MODULO     | Perform Modulo Operation. Setting the PKA_FUNC.MODULO bit performs the modulo operation. For more in- formation, see the "PKCP Vector Operations" section.                                                                                                                                            |
| 8 (R/W)            | DIV        | Perform Divide Operation. Setting the PKA_FUNC.DIV bit performs the divide operation. For more informa- tion, see the "PKCP Vector Operations" section.                                                                                                                                               |
| 7 (R/W)            | LSHFT      | Perform Left Shift Operation. Setting the PKA_FUNC.LSHFT bit performs the Left shift operation. For more infor- mation, see the "PKCP Vector Operations" section.                                                                                                                                     |
| 6 (R/W)            | RSHFT      | Perform Right Shift Operation. Setting the PKA_FUNC.RSHFT bit performs the right shift operation. For more in- formation, see the "PKCP Vector Operations" section.                                                                                                                                   |
| 5 (R/W)            | SUB        | Perform Subtract Operation. Setting the PKA_FUNC.SUB bit performs the subtract operation. For more informa- tion, see the "PKCP Vector Operations" section.                                                                                                                                           |
| 4 (R/W)            | ADD        | Perform Add Operation. Setting the PKA_FUNC.ADD bit performs the add operation. For more information, see the "PKCP Vector Operations" section.                                                                                                                                                       |
| 3 (R/W)            | MSONE      | Most Significant One. Setting the PKA_FUNC.MSONE bit loads the location of the Most Significant one bit within the result word indicated in the PKA_RESULTMSW register into bits [4:0] of the PKA_DIVMSW register can only be used with basic PKCP operations, except for Divide, Modulo and Compare. |
| 1 (R/W)            | ADDSUB     | Perform Combined Add/Subtract Operation. Setting the PKA_FUNC.ADDSUB bit performs the combined Add/Subtract opera- tion. For more information, see the "PKCP Vector Operations" section.                                                                                                              |
| 0 (R/W)            | MULT       | Perform Multiply Operation. Setting the PKA_FUNC.MULT bit performs the multiply operation. For more infor- mation, see the "PKCP Vector Operations" section.                                                                                                                                          |

## Start of PKA RAM space

The PKA\_RAM register provides the starting location of the RAM space to hold the input, output and other vectors.

Figure 42-11: PKA\_RAM Register Diagram

<!-- image -->

Table 42-33: PKA\_RAM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | First Location in PKA RAM. The PKA_RAM.VALUE bit field provides the starting location of the RAM space to hold the input, output and other vectors. |

## PKA Most-Significant-Word of Result Vector

The PKA\_RESULTMSW register indicates the (word) address in the PKA RAM where the most significant non-zero 32-bit word of the result is stored and should be ignored for modulo operations. For basic PKCP operations, the PKA\_RESULTMSW register is updated when the PKA\_FUNC.RUN bit is reset at the end of the operation.

For the complex sequencer controlled operations, updating the final value matching the actual result is done near the end of the operation. Note that the result is only meaningful if no errors are detected and that for ECC operations, the PKA\_DIVMSW register provides information for the x-coordinate of the result point only.

Figure 42-12: PKA\_RESULTMSW Register Diagram

<!-- image -->

Table 42-34: PKA\_RESULTMSW Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/NW)          | ZERO       | Result Is Zero. The PKA_RESULTMSW.ZERO bit indicates the result vector is all zeros, ignore the address returned in bits [10:0].                                   |
| 10:0 (R/NW)        | ADDR       | Address of Most-significant Nonzero Word. The PKA_RESULTMSW.ADDR bit is the address of the most significant non-zero 32- bit word of the result vector in PKA RAM. |

## PKA Bit Shift Value

For basic PKCP operations, modifying the contents of the PKA\_SHIFT register is made impossible while the operation is being performed. For the ExpMod-variable and ExpMod-CRT operations, the PKA\_SHIFT register is used to indicate the number of odd powers to use (directly as a value in the range 1-16). For the ModInv and ECC operations, this register is used to hold a completion code.

Figure 42-13: PKA\_SHIFT Register Diagram

<!-- image -->

Table 42-35: PKA\_SHIFT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4:0                | VALUE      | Bits to Shift. The PKA_SHIFT.VALUE bit field is the number of bits to shift the input vector (in the range 0-31) during a Rshift or Lshift operation. |
| (R/W)              |            |                                                                                                                                                       |