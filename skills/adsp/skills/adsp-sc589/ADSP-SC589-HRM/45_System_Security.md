## 42   System Security

The requirement to protect content, keys, IP and other sensitive information have become increasingly prevalent. The processor contains several modules and system elements that contribute to creating a secure operating environment for trusted code to execute. The modules and system elements that can be used are:

- Boot Kernel
- Secure Core
- System Protection Unit (SPU)
- System Memory Protection Unit (SMPU)
- DEBUG through the Test Access Port Controller (TAPC)
- One-Time Programmable (OTP) memory
- Cryptographic Accelerators (optionally)

NOTE: This product includes security features that can be used to protect embedded non-volatile memory contents and prevent execution of non-authorized code. When security is enabled on this device (either by the ordering party or the subsequent receiving parties), Analog Devices' ability to conduct Failure Analysis on returned devices will be limited. Please contact Analog Devices, Inc. for details on the Failure Analysis limitations for this device.

## Security Features

The security infrastructure items in the system provide the following features:

- Secure operating environment for secure code execution
- Protect sensitive IP from theft by malicious users or competitors
- Protect sensitive data (for example cipher keys)
- Allow debugging while still maintaining security

## Security Functional Description

In order to provide secure operating environment in a system, it requires the involvement of multiple elements.

## Boot Kernel

The Boot Kernel is the root of trust for a secure system. Since the boot kernel is developed by Analog Devices, Inc and is stored in ROM and can't be changed, it can be trusted. The boot kernel validates the authenticity of the application binary image that needs to be booted in. It also handles decrypting the binary image if it's encrypted.

Verifying the authenticity of the application binary asserts that

1. The image hasn't been tampered with or altered
2. The image came from a trusted developer

If the image is encrypted, it ensures confidentiality since the boot image is stored on an external storage device and can be more easily read or stolen than in the part.

Once the boot kernel can verify and optionally decrypt the boot image, it can be loaded into the processor for execution. At this point, the chain of trust is continued with the verified application.

## Secure Booting

Secure booting is when the boot kernel uses cryptographic algorithms to perform checks on the application binary and to decrypt it. Secure booting is done when security is enabled. If security is not enabled the boot kernel does not verify any signatures nor does it perform any decryption on the application binary. See Security Mode Configuration.

## Secure Core

In a system with multiple master and slave resources, not everything is considered secure. There are secure and nonsecure peripherals and secure and non-secure segments of memory.

For a system to be considerd secure, a secure core that can access and execute instructions (verified application) from secure memory must be configured. Typically, in a single core processor, either the core is hardwired to be secure or the core can switch between secure and non-secure modes.

## System Protection Unit (SPU)

The SPU in the system serves two functions. First, it acts as a gatekeeper, guarding against non-secure accesses to secure resources (peripherals). Second, it is used to define which resources in the system are secure or non-secure masters and which resources in the system are secure or non-secure slaves.

- NOTE: Even thought the SPU can be configured by secure or non-secure master, the first steps that the verified secure application should perform are:
1. Configure the SPU as a secure slave itself so only other secure masters can configure it

2. Define and configure the secure masters and slaves using the SPU

This way, once the SPU is secure and other secure masters and slaves are configured, non-secure masters cannot tamper with the security privileges of secure masters and slaves nor can non-secure resources be changed to a secure resource.

## System Memory Protection Unit (SMPU)

Similar to the SPU protecting the MMR address range for peripherals, the SMPU can guard memory ranges or pages. Memory pages can be configured as secure or non-secure. Again, like in the case of the SPU, the SMPU can guard against non-secure transaction attempts to secure memory.

NOTE: A verified application should reside in memory configured as secure. By default the SMPU has all memory configured as secure. If the program needs to update the memory protections via the SMPU, the application and it's data should remain in secure memory.

## Debug

The typical way of accessing a system is through the debug port via a JTAG or serial wire interface. If these access points are not secure sensitive IP such as cipher keys can be exposed and code can be changed to disable other security settings. T o guard against this type of attack or security hole and still provide debugging capabilities for a developer, a debug unit with security features is used.

When the developer first receives the part, they can define a JTAG/DEBUG key that is programmed into OTP memory. Once security is enabled, the debug unit compares the key sent from the host debugger with the key inside the system. If a match occurs, debug access to secure resources are allowed.

## One-Time-Programmable (OTP) Memory

Customer programmable OTP memory is used to safely and securely store sensitive information such as cipher keys.

In a public key algorithm (for example ECDSA which is used in secure boot), the public key is used to verify the digital signature that accompanies the application binary. The private key is used by the developer on the host development machine to create the digital signature. The public and private key pair is unique and the public key needs to be stored in non-volatile, one-time-programmable memory. If the public key is allowed to be changed than users can generate their own public/private key pairs and successfully boot in malicious code. This can either re-purpose the part or change security configurations to allow easier access to sensitive information stored elsewhere in OTP memory.

When security is not enabled, OTP can be accessed freely. When a user decides to set the LOCK bit in OTP memory to enable security, portions of the OTP , specifically the first 6K bits and another 1K bits for the customer boot information are will also be locked. From then on, only secure masters will be allowed to access those memory regions.

## Cryptographic Accelerators

Cryptographic algorithms are mathematical tools to help provide security. Hardware engines provide some advantages but are not necessarily required. For computationally expensive operations like those used in Elliptic Curve Cryptography, like ECDSA used in secure boot, the operations can be accelerated while the core performs other tasks. Also, it's less likely that the hardware engine can be hacked to change the results.

## Security Mode Configuration

Because the processor is unsecure by default, if security is not required then no steps need to be done.

To enable security features, use the following procedure.

- Generate the public/private key pair on the host development machine. 1
- Program OTP memory with the public key. 2
- Program OTP memory with the decryption key if the application binary needs to be encrypted. 2
- Program OTP memory with the debug/JTAG key. 2
- Develop the application and sign it, creating the digital signature with the private key. 1
- If confidentiality is required, encrypt the application binary before signing it.
- Set the LOCK bit in OTP memory to enable security. After this, subsequent boots are secure boots. 2, 3

1  Software tools are provided with developments tools to generate keys, sign boot streams and also perform encryption. Refer to the development tools manuals for information on usage.

2 Refer to the OTP Chapter for programming the OTP and other related information.

3 Refer to the Booting Chapter for more information on Secure Booting and other related information.

## Status and Error Signals

In a fully functional secure system, non-secure resources should not even attempt to access a secure resource. If this does occur, then either the code has been altered or replaced with malicious code or the system contains a bug.

Errors or error events are dependant on the configuration of the SPU and SMPU, the protection units which guard against security violations. In the case of the SMPU, the error can simply be captured, captured and interrupt generated, or the access prevented without capturing any error. It is the developer's responsibility to:

1. Determine if there was an error due to a blocked access.
2. Determine how to handle a blocked access (for example fix the bug (offline), or try to use a different resource (run time)).