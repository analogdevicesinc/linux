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
--------------------------------------

Introduction
~~~~~~~~~~~~
NXP i.MX hardware IPs such as EdgeLock Enclave (ELE) and V2X create an
embedded secure enclave within the SoC boundary to enable features like:

- Hardware Security Module (HSM)
- Security Hardware Extension (SHE)
- Vehicular to Anything (V2X)

Each of the above features is enabled through a dedicated NXP hardware IP
on the SoC. A single SoC may contain more than one such hardware IP, that
is, more than one secure enclave can coexist.

NXP SoCs with such secure enclave (SE) IPs are:
i.MX93, i.MX8ULP

To communicate with one or more coexisting SEs on the SoC, there are
dedicated messaging units (MU) per SE. Each coexisting SE can have one or
more exclusive MUs dedicated to itself. No MU is shared between two SEs.
MU communication is realized using the mailbox driver. Each secure enclave
can serve multiple clients by virtue of these exclusive MUs, and can
distinguish transactions from different clients based on the MU used and
the core security state. The communication between clients and secure
enclaves uses a command/response mechanism. Each client can expose a
specific set of secure enclave features to higher layers, based on the
commands it supports. For example, a secure enclave can simultaneously
serve an OP-TEE TA and a Linux middleware client. Each client exposes a
specific set of secure enclave features based on its supported command set.

NXP Secure Enclave (SE) Interface
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
MUs are not shared between SEs. For an SoC like i.MX95, which has multiple
SEs such as HSM, V2X-HSM, and V2X-SHE, all SEs and their ``se-if``
interfaces dedicated to a particular SE are enumerated and provisioned
using the single compatible node ``fsl,imx95-se``.

Each ``se-if`` comprises two layers:

- (C_DEV Layer) User-space software-access interface.
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
   |        |   Misc. Dev Sync Logic  |         |
   |        +-------------------------+         |
   |                                            |
   +--------------------------------------------+

::

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
  This layer ensures the communication protocol defined for interaction
  with firmware.

  The firmware communication protocol provides two guarantees:

  - Serializing the messages to be sent over an MU.
  - Firmware can handle one command message at a time.

- c_dev:
  This layer offers character device contexts, created as
  ``/dev/<se>_mux_chx``. Using multiple device contexts multiplexed over
  a single MU, userspace applications can use file operations (fops) such
  as ``write`` and ``read`` to send a command message and read back the
  response to/from firmware. These fops use the service layer API to
  communicate with firmware.

  Misc-device (``/dev/<se>_mux_chn``) synchronization protocol::

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
   kernel space +------+      +------+     +------+      |
                                                         |
   +---------------------------------------------------- |
                    |             |           |          |
   userspace   /dev/ele_muXch0    |           |          |
                          /dev/ele_muXch1     |          |
                                        /dev/ele_muXchY  |
                                                         |

When a user sends a command to the firmware, it registers its
``device_ctx`` as a waiter of a response from firmware.

The secure enclave firmware manages storage over a Linux filesystem.
For this, ``c_dev`` provisions a dedicated device context called the
command-receiver.


ELE_STORAGE_OPEN_REQ concurrency and command-receiver exclusivity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A userspace-created ``dev_ctx`` becomes the storage subordinate to FW by
being registered as the command-receiver via
``set_dev_ctx_as_command_receiver()``. FW sends NVM callback commands to
this ``dev_ctx`` via that priv's MU, on which the ``dev_ctx`` was created.
Once a userspace ``dev_ctx`` on one priv is registered as the
command-receiver, after opening the storage handle with FW, FW enforces a
global one-storage-instance limit: a concurrent ELE_STORAGE_OPEN_REQ
arriving over this or any other priv's MU is rejected by FW itself, before
any storage handle is allocated. The driver uses three layers of
protection; the sequence below shows how a concurrent race is handled
safely::

  Userspace A        Kernel (se_ctrl)          FW (ELE)        Userspace B
      |                    |                      |                  |
      |--ioctl(SEND_RCV)-->|                      |                  |
      |            [advisory check under          |                  |
      |             modify_lock: no receiver]     |                  |
      |            [acquire se_if_cmd_lock]       |--ioctl(SEND_RCV)->
      |                    |                      |  [advisory check  |
      |                    |                      |   passes: TOCTOU] |
      |                    |                      |  [blocks on       |
      |                    |                      |   se_if_cmd_lock] |
      |            ele_msg_send_rcv()             |                  |
      |                    |--STORAGE_OPEN_REQ--->|                  |
      |                    |<--STORAGE_OPEN_RSP---|                  |
      |            fw_api_specific_ops():         |                  |
      |            set_dev_ctx_as_command_receiver|                  |
      |            [re-check under modify_lock]   |                  |
      |            [release se_if_cmd_lock]       |                  |
      |<--ioctl 0----------|                      |                  |
      |                    |                      |  [B gets lock]   |
      |                    |                      |--STORAGE_OPEN_REQ->
      |                    |                      |  [FW rejects:    |
      |                    |                      |   one storage    |
      |                    |                      |   at a time]     |
      |                    |                      |<--ERROR_RSP------|
      |                    |  se_val_rsp_hdr_n_status: -EPERM        |
      |                    |  fw_api_specific_ops not called         |
      |                    |<--ioctl -EPERM-to B------------------->|

The three protection layers are:

1. Advisory early check (under modify_lock, before send): fast-path
   rejection if a receiver is already registered. Not the final gate
   because modify_lock is released before the MU send (TOCTOU window).

2. ``se_if_cmd_lock``: held for the entire send+receive cycle, so only
   one ioctl command is in flight on a given MU at a time.

3. ``set_dev_ctx_as_command_receiver()`` re-checks under modify_lock
   after the response arrives. If two callers race past layer 1, FW
   itself rejects the second ELE_STORAGE_OPEN_REQ before any handle is
   allocated.

Signal handling after a completed hardware operation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A signal may arrive after the firmware has already executed the command
and delivered its response into the MU receive buffer. The driver detects
this case and preserves state before returning ``-EINTR``::

  Userspace            Kernel (se_ctrl)          FW (ELE)
      |                    |                      |
      |--ioctl(SEND_RCV)-->|                      |
      |            ele_msg_send_rcv()             |
      |                    |--CMD_REQ------------>|
      |  [signal arrives]  |                      | (FW executes,
      |                    |                      |  allocates handle,
      |                    |                      |  writes response)
      |                    |<--CMD_RSP------------|
      |  wait_for_completion_interruptible()      |
      |  wakes: signal seen -> -ERESTARTSYS       |
      |                    |                      |
      |            [response is in rx_msg:        |
      |             validate with                 |
      |             se_val_rsp_hdr_n_status()]    |
      |            [fw_api_specific_ops()         |
      |             (is_cmd_interrupted=true):    |
      |             for SESSION_OPEN: record      |
      |             handle, close session via     |
      |             se_close_session(), clear;    |
      |             for STORAGE_OPEN: record      |
      |             handle, close storage via     |
      |             se_close_storage(), return 0] |
      |            err = -EINTR                   |
      |            (not -ERESTARTSYS:             |
      |             prevents VFS auto-restart)    |
      |<--ioctl -EINTR-----|                      |
      |                    |                      |
      | [signal handler runs; userspace decides   |
      |  whether to re-issue; FW handle tracked   |
      |  or cleaned up; no firmware resource leak]|

Returning ``-EINTR`` instead of ``-ERESTARTSYS`` is intentional: the VFS
would transparently restart an ioctl on ``-ERESTARTSYS``, re-running the
command with already-zeroed shared input buffers. ``-EINTR`` lets userspace
enter its signal handler and decide whether to reissue the command.

.. kernel-doc:: drivers/firmware/imx/se_ctrl.c
   :export:
