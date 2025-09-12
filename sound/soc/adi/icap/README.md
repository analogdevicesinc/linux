# Inter Core Audio Protocol

Inter Core Audio Protocol (ICAP) to exchange playback and record audio data
between audio device and audio application on different cores. Currently it is
using rpmsg as a transport layer but can be expanded to other transport layers.

Current ICAP implementations include bare metal application using rpmsg-lite
library and Linux kernel implementation. Future implementations will cover
Linux user space library for user space applications.

ICAP has GPLv2 license when distributed with Linux kernel, otherwise it has
Apache 2.0 license. For details see the LICENSE file.

## Overview
Software running on different cores communicate with each using ICAP in
application-device relation. One end of the ICAP communication must be
application, other end must be device - application creates audio data stream
for device to playback and reads audio data recorded by device.

Application side includes icap_application.h with application specific functions
and callbacks. Device side includes icap_device.h with device specific functions
and callbacks. Each ICAP function call sends appropriate message to the other
side which triggers corresponding callback (#icap_device_callbacks or
#icap_application_callbacks) for the message. The other side sends back a
positive response message (#ICAP_ACK) with or without payload. In case of
failure the other side can send back a negative response (#ICAP_NAK) with error
code. ICAP application functions are synchronous, they wait for response until
#ICAP_MSG_TIMEOUT_US. Application functions work like Remote Function Calls
(RFC). ICAP device functions are asynchronous, they don't wait for corresponding
response message therefore it is possible to call them in interrupt context
which may be required to implement proper playback and record audio streams.
When an ICAP device receives a response the proper callback is executed.

It is possible to cascade ICAP communication by calling application functions
inside device callbacks creating a proxy between first ICAP application side and
final ICAP device side. E.G on SC584 SOC it's possible to establish ICAP
communication between:<br>
`ICAP application on ARM <-> ICAP proxy on SHARC0 <-> ICAP device on SHARC1`

## Simplified usage
### ICAP application
1. Include icap_application.h and allocate statically or dynamically
`struct icap_instance` and `struct icap_application_callbacks`.
2. Initialize `icap_application_callbacks` with proper callback functions.
3. Set appropriate field of the `icap_instance.transport`:
 * for bare metal + rpmsg-lite set the `icap_transport.rpmsg_instance` and
 `icap_transport.rpmsg_ept` fields.
 * for linux kernel set the `icap_transport.rpdev` field.
 * for linux user space set the `icap_transport.fd` field.
4. Initialize the ICAP instance with `icap_application_init()`.
5. Get number of subdevices from ICAP device using `icap_get_subdevices()`.
6. Get features of each device using `icap_get_subdevice_features()`.
7. For a playback subdevice (`#ICAP_DEV_PLAYBACK`) allocate source buffer and
attach the buffer to the `subdevice using icap_add_src()`.
8. For a record subdevice `(#ICAP_DEV_RECORD)` allocate destination buffer and
attach the buffer to the subdevice `using icap_add_dst()`.
9. Fill the playback buffer with audio data.
10. Start subdevices with `icap_start()`.
11. Monitor buffer levels with `icap_application_callbacks.frag_ready()`,
fill more playback audio data if necessary and read recorded audio data.

### ICAP device
1. Include icap_device.h and allocate statically or dynamically
`struct icap_instance` and `struct icap_device_callbacks`.
2. Initialize icap_device_callbacks with proper callback functions.
2. Set appropriate field of the `icap_instance.transport`:
 * for bare metal + rpmsg-lite set the `icap_transport.rpmsg_instance` and
 `icap_transport.rpmsg_ept` fields
 * for linux kernel set the `icap_transport.rpdev` field
 * for linux user space set the `icap_transport.fd` field
3. Initialize the ICAP instance with `icap_device_init()`
4. Wait until playback and record buffers are attached by `add_src()` and
`add_dst()` callbacks.
5. Wait until a subdevice is started by `start()` callback.
6. Read audio data from playback buffer and write the data to audio hardware.
7. Read audio data from audio hardware and write to record buffer.
8. Notify application side about audio fragments consumed from the buffers
using `icap_frag_ready()`.
