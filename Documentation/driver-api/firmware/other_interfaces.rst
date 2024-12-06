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

NXP i.MX Secure Enclave Enabled SoC Service layer and C_DEV driver
------------------------------------------------------------------
The NXP's i.MX HW IP like EdgeLock-Enclave, creating an embedded secure
enclave within the SoC boundary to enable features like
 - HSM
 - SHE
 - V2X

SoC enabled with the NXP i.MX secure enclave IP(s) like EdgeLock-Enclave(ELE),
are: i.MX93, i.MX8ULP.

This driver exposes two interfaces:
- service layer: This layer takes the two mutex locks:
  "mu_cmd_lock" is taken to ensure one service is processed at a time. This
  lock is not unlocked, till one service processing is complete. Multiple
  messages can be exchanged with FW as part of one service processing.
  "mu_lock" is taken to ensure one message is sent over MU at a time. This
  lock is unlocked, post sending the message.

- c_dev:
  This driver configures multiple misc-devices on the MU, to exchange
  messages from User-space application and NXP's Edgelocke Enclave firmware.
  The driver ensures that the messages must follow the following protocol
  defined.

                                Non-Secure               +   Secure
                                                         |
                                                         |
                  +---------+      +-------------+       |
                  | se_fw.c +<---->+imx-mailbox.c|       |
                  |         |      |  mailbox.c  +<-->+------+    +------+
                  +---+-----+      +-------------+    | MU X +<-->+ ELE |
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

Enclave's Firmware owns the storage management, over linux filesystem.
For this c_dev provisions a dedicated slave device called "receiver".

.. kernel-doc:: drivers/firmware/imx/se_fw.c
   :export:
