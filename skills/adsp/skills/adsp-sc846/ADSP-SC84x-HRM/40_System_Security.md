## 37   System Security

The requirements to protect content, keys, intellectual property, and other sensitive information have become increasingly prevalent. The processor contains several modules and system elements that contribute to creating a secure operating environment for trusted code to execute.

The modules and system elements for security are:

- Boot Kernel
- Secure Core
- System Protection Unit (SPU)
- Security Packet Engine (PKTE)
- Public Key Accelerator (PKA)
- Public Key Interrupt Controller (PKIC)
- System Memory Protection Unit (SMPU)
- DEBUG through the Test Access Port Controller (TAPC)
- One-Time Programmable (OTP) memory
- Cryptographic Accelerators (optionally) and Hardware Security Module (HSM)

NOTE: This product includes security features for protecting embedded non-volatile memory contents and preventing execution of non-authorized code. Enabling security on this device (either by the ordering party or the subsequent receiving parties), limits the ability of Analog Devices to conduct failure analysis on returned devices. Contact Analog Devices, Inc. for details on the failure analysis limitations for this device.

## Security Features

The security infrastructure in the system provides the following features:

- Secure operating environment for secure code execution
- Protecting sensitive IP from theft by malicious users or competitors

- Protecting sensitive data (for example cipher keys)
- Allowing debugging while still maintaining security

## Security Functional Description

In order to provide a secure operating environment in a system, it requires the involvement of multiple elements.

## Boot Kernel

The boot kernel is the root of trust for a secure system. Since the boot kernel is developed by Analog Devices, Inc., is stored in ROM, and can't be changed, it can be trusted. The boot kernel validates the authenticity of the application binary image that needs to be booted in. It also handles decrypting the binary image if it's encrypted.

Verifying the authenticity of the application binary asserts that

1. The image is not tampered with or altered
2. The image came from a trusted developer

If the image is encrypted, it ensures confidentiality since the boot image is stored on an external storage device and can be more easily read or stolen than in the part.

Once the boot kernel can verify and optionally decrypt the boot image, it can be loaded into the processor for execution. At this point, the chain of trust is continued with the verified application.

## Secure Booting

Secure booting is when the boot kernel uses cryptographic algorithms to perform checks on the application binary and to decrypt it. When security is enabled, signature verification and optional decryption on the application binary are both performed by secure boot. See Security Mode Configuration.

## Secure Core

In a system with multiple requester and completer resources, not everything is considered secure. There are secure and non-secure peripherals and secure and non-secure segments of memory.

For a system to be considered secure, a secure core that can access and execute instructions (verified application) from secure memory must be configured. Typically, in a single core processor, either the core security is hardwired, or the core can switch between secure and non-secure modes.

## System Protection Unit (SPU)

The SPU in the system serves two functions. First, it acts as a gatekeeper, guarding against non-secure accesses to secure resources (peripherals). Second, it is used to define which resources in the system are secure or non-secure requesters and which resources in the system are secure or non-secure completers.

NOTE: Though the SPU can be configured by secure or non-secure requester, the first steps that the verified secure application must perform are:

1. Configure the SPU as a secure completer itself so only other secure requesters can configure it
2. Define and configure the secure requesters and completers using the SPU

This way, once the SPU is secure and other secure requesters and completers are configured, non-secure requesters cannot tamper with the security privileges of secure requesters and completers nor can non-secure resources be changed to a secure resource.

## System Memory Protection Unit (SMPU)

Similar to the SPU protecting the MMR address range for peripherals, the SMPU can guard memory ranges or pages. Memory pages can be configured as secure or non-secure. Again, like in the case of the SPU, the SMPU can guard against non-secure transaction attempts to secure memory.

NOTE: A verified application should reside in memory configured as secure. By default the SMPU has all memory configured as secure. When the program needs to update the memory protections using the SMPU, the application and its data should remain in secure memory.

## Debug

The typical way of accessing a system is through the debug port via a JTAG or serial wire interface. When these access points are not secure, sensitive IP such as cipher keys can be exposed and code can be changed to disable other security settings. T o guard against this type of attack or security hole and still provide debugging capabilities for a developer, a debug unit with security features is used.

When the developer first receives the part, they can define a JTAG/DEBUG key that is programmed into OTP memory. Once security is enabled, the debug unit compares the key sent from the host debugger with the key inside the system. If a match occurs, debug access to secure resources are allowed.

## One-Time-Programmable (OTP) Memory

Customer programmable OTP memory is used to safely and securely store sensitive information such as cipher keys.

In a public key algorithm (for example, ECDSA which is used in secure boot), the public key is used to verify the digital signature that accompanies the application binary. The private key is used by the developer on the host development machine to create the digital signature. The public and private key pair is unique, plus the public key needs to be stored in non-volatile, one-time-programmable memory. When the public key can be changed then users can generate their own public/private key pairs and successfully boot in malicious code. This can either re-purpose the part or change security configurations to allow easier access to sensitive information stored elsewhere in the OTP memory.

## Cryptographic Accelerators

Cryptographic algorithms are mathematical tools to help provide security. Hardware engines provide some advantages but are not necessarily required. For computationally expensive operations like those used in Elliptic Curve

Cryptography, like ECDSA used in secure boot, the operations can be accelerated while the core performs other tasks. Also, it's less likely that the hardware engine can be hacked to change the results.

## Hardware Security Module (HSM)

While cryptographic accelerators primarily optimize performance, the Hardware Security Module (HSM) can also serve as the dedicated root of trust for the SoC. The HSM is an autonomous subsystem that includes its own processor, isolated memory, and cryptographic engines. Its primary function is to manage the lifecycle of high-value assets-such as encryption keys and certificates-within a secure boundary that is inaccessible to the main application cores. By performing sensitive operations (like key generation and signing) internally, the HSM ensures that secret keys are never exposed to the host operating system, significantly reducing the attack surface against software-based vulnerabilities.

## Security Mode Configuration

Security as a feature does not necessarily be employed by the user. There are no steps to disable security, if security protection is not required. The part does not have security enabled.

To use the security features using cryptographic accelerators, perform the following steps:

- Generate the public/private key pair on the host development machine. 1
- Program OTP memory with the public key. 2
- Program OTP memory with the decryption key if the application binary needs to be encrypted. 2
- Program OTP memory with the debug/JTAG key. 2
- Develop the application and sign it, creating the digital signature with the private key. 1
- If confidentiality is required, encrypt the application binary before signing it.
- Set the LOCK bit in OTP memory to enable security. After this, subsequent boots are secure boots. 2, 3

1  Software tools are provided with developments tools to generate keys, sign boot streams, and also perform encryption. Refer to the development tools manuals for details on how to use.

2 Refer to the OTP Chapter for programming the OTP and other related information.

3 Refer to the Booting Chapter for more information on Secure Booting and other related information.

## HSM Assisted Secure Boot Flow

In this implementation, the SoC retains the role of the primary boot master upon coming out of reset. The secure boot sequence is a staged process where the SoC first provisions the security subsystem before delegating sensitive tasks.

1. HSM Initialization and Staging-The SoC loads the HSM application firmware from external storage into the intermediate memory. Once the image is staged, the SoC releases the HSM from reset. The HSM then boots from the intermediate memory, initializing its isolated environment and authenticating the HSM firmware integrity.
2. Runtime Security Services (SIC Protocol)-Once operational, the HSM transitions to a service-provider role, communicating with the SoC via the Secure Inter-Processor Communication (SIC) protocol. For subsequent boot stages, specifically the incoming Secure Image, the SoC offloads all cryptographic validation to the HSM. Tasks such as ECDSA signature verification and AES decryption, previously handled by direct crypto accelerator access, are now executed within the secure boundary of the HSM, ensuring that the verification keys and decrypted code remain isolated from the main application core.

All the relevant asset specific to HSM boot are programmed in HSM OTP space.

## Status and Error Signals

In a fully functional secure system, non-secure resources should not even attempt to access a secure resource. When this happens, either the code has been altered or replaced with malicious code or the system contains a coding error.

Errors or error events are dependent on the configuration of the SPU and SMPU, the protection units which guard against security violations. In the case of the SMPU, the error can simply be captured, captured and interrupt generated, or the access prevented without capturing any error. It is the developer's responsibility to:

1. Determine when there was an error because of a blocked access.
2. Determine how to handle a blocked access. For example fix the bug (offline) or try to use a different resource (run time).