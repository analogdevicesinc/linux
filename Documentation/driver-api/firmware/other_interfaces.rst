Other Firmware Interfaces
=========================

DMI Interfaces
--------------

.. kernel-doc:: drivers/firmware/dmi_scan.c
   :export:

EDD Interfaces
--------------

.. kernel-doc:: drivers/firmware/edd.c
   :internal:

Generic System Framebuffers Interface
-------------------------------------

.. kernel-doc:: drivers/firmware/sysfb.c
   :export:

Intel Stratix10 SoC Service Layer
---------------------------------
Some features of the Intel Stratix10 SoC require a level of privilege
higher than the kernel is granted. Such secure features include
FPGA programming. In terms of the ARMv8 architecture, the kernel runs
at Exception Level 1 (EL1), access to the features requires
Exception Level 3 (EL3).

The Intel Stratix10 SoC service layer provides an in kernel API for
drivers to request access to the secure features. The requests are queued
and processed one by one. ARM’s SMCCC is used to pass the execution
of the requests on to a secure monitor (EL3).

.. kernel-doc:: include/linux/firmware/intel/stratix10-svc-client.h
   :functions: stratix10_svc_command_code

.. kernel-doc:: include/linux/firmware/intel/stratix10-svc-client.h
   :functions: stratix10_svc_client_msg

.. kernel-doc:: include/linux/firmware/intel/stratix10-svc-client.h
   :functions: stratix10_svc_command_config_type

.. kernel-doc:: include/linux/firmware/intel/stratix10-svc-client.h
   :functions: stratix10_svc_cb_data

.. kernel-doc:: include/linux/firmware/intel/stratix10-svc-client.h
   :functions: stratix10_svc_client

.. kernel-doc:: drivers/firmware/stratix10-svc.c
   :export:

NXP Secure Enclave Firmware Interface
=====================================

Introduction
------------
The NXP's i.MX HW IP like EdgeLock Enclave, V2X etc., creates an embedded secure
enclave within the SoC boundary to enable features like
 - Hardware Security Module (HSM)
 - Security Hardware Extension (SHE)
 - Vehicular to Anything (V2X)

Each of the above feature is enabled through dedicated NXP H/W IP on the SoC.
On a single SoC, multiple hardware IP (or can say more than one secure enclave)
can exist.

NXP SoCs enabled with the such secure enclaves(SEs) IPs are:
i.MX93, i.MX8ULP

To communicate with one or more co-existing SE(s) on SoC, there is/are dedicated
messaging units(MU) per SE. Each co-existing SE can have one or multiple exclusive
MUs, dedicated to itself. None of the MU is shared between two SEs. Communication
of the MU is realized using the mailbox driver. Each secure enclave can cater to
multiple clients by virtue of these exclusive MUs. Also, they can distinguish
transactions originating from these clients based on the MU used and core security
state. The communication between the clients and secure enclaves is in form of
command/response mechanism. Each client could expose specific set of secure enclave
features to the higher layers, based on the commands supported by that client. For
example, the secure enclave could simultaneously support an OPTEE TA and Linux
middleware as clients. Each of these clients can expose specific set of secure
enclave features based on the command set supported by them.

NXP Secure Enclave(SE) Interface
--------------------------------
Although MU(s) is/are not shared between SE(s). But for SoC like i.MX95 which has
multiple SE(s) like HSM, V2X-HSM, V2X-SHE; all the SE(s) and their interfaces 'se-if'
that is/are dedicated to a particular SE will be enumerated and provisioned using the
single compatible node("fsl,imx95-se").

Each 'se-if' comprise of twp layers:
- (C_DEV Layer) User-Space software-access interface.
- (Service Layer) OS-level software-access interface.

::
   +--------------------------------------------+
   |            Character Device(C_DEV)         |
   |                                            |
   |   +---------+ +---------+     +---------+  |
   |   | misc #1 | | misc #2 | ... | misc #n |  |
   |   |  dev    | |  dev    |     | dev     |  |
   |   +---------+ +---------+     +---------+  |
   |        +-------------------------+         |
   |        | Misc. Dev Synchr. Logic |         |
   |        +-------------------------+         |
   |                                            |
   +--------------------------------------------+

   +--------------------------------------------+
   |               Service Layer                |
   |                                            |
   |      +-----------------------------+       |
   |      | Message Serialization Logic |       |
   |      +-----------------------------+       |
   |          +---------------+                 |
   |          |  imx-mailbox  |                 |
   |          |   mailbox.c   |                 |
   |          +---------------+                 |
   |                                            |
   +--------------------------------------------+

- service layer:
  This layer is responsible for ensuring the communication protocol that is defined
  for communication with firmware.

  FW Communication protocol ensures two things:
  - Serializing the messages to be sent over an MU.

  - FW can handle one command message at a time.

- c_dev:
  This layer offers character device contexts, created as '/dev/<se>_mux_chx'.
  Using these multiple device contexts that are getting multiplexed over a single MU,
  userspace application(s) can call fops like write/read to send the command message,
  and read back the command response message to/from Firmware.
  fops like read & write use the above defined service layer API(s) to communicate with
  Firmware.

  Misc-device(/dev/<se>_mux_chn) synchronization protocol:
::

                                Non-Secure               +   Secure
                                                         |
                                                         |
                +-----------+      +-------------+       |
                | se_ctrl.c +<---->+imx-mailbox.c|       |
                |           |      |  mailbox.c  +<-->+------+    +------+
                +-----+-----+      +-------------+    | MU X +<-->+ ELE |
                      |                               +------+    +------+
                      +----------------+                 |
                      |                |                 |
                      v                v                 |
                  logical           logical              |
                  receiver          waiter               |
                     +                 +                 |
                     |                 |                 |
                     |                 |                 |
                     |            +----+------+          |
                     |            |           |          |
                     |            |           |          |
              device_ctx     device_ctx     device_ctx   |
                                                         |
                User 0        User 1       User Y        |
                +------+      +------+     +------+      |
                |misc.c|      |misc.c|     |misc.c|      |
 kernel space   +------+      +------+     +------+      |
                                                         |
 +------------------------------------------------------ |
                    |             |           |          |
 userspace     /dev/ele_muXch0    |           |          |
                          /dev/ele_muXch1     |          |
                                        /dev/ele_muXchY  |
                                                         |

When a user sends a command to the firmware, it registers its device_ctx
as waiter of a response from firmware.

Enclave's Firmware owns the storage management, over Linux filesystem.
For this c_dev provisions a dedicated slave device called "receiver".

.. kernel-doc:: drivers/firmware/imx/se_ctrl.c
   :export:
