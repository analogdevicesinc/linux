.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/net-phy/adin

.. _adin:

ADIN1300/ADIN1200
=================

ADIN1300/ADIN1200 PHY Linux Driver.

Supported Devices
-----------------

- :adi:`ADIN1200`
- :adi:`ADIN1300`

Supported Boards
----------------

Description
-----------

The ADIN1200 is a low power single port 10/100 Mb Ethernet transceiver with low
latency specifications primarily designed for industrial Ethernet applications.

The ADIN1300 is a low power single port Gigabit Ethernet transceiver with
industry leading latency specifications primarily designed for industrial
Ethernet applications.

This design integrates an Energy Efficient Ethernet PHY core plus all the
associated common analog circuitry, input and output clock buffering, the
management interface and subsystem registers as well as the MAC interface and
control logic to manage the reset and clock control and pin configuration.

The 2 PHYs are similar from an interfacing perspective, meaning that the HW & SW
interfaces are identical. The main difference is that the ADIN1300 supports
gigabit speeds.

The PHY can be interfaced with the following: MII, RGMII, RGMII with internal
delays (on RX, TX or both), RMII.

.. important::

   The RMII interface requires an external 50 MHz clock reference, and as such
   requires a bit more consideration when designing the layout. This makes it
   more difficult to configure it entirely from software

Source Code
-----------

Status
~~~~~~

.. list-table::
   :header-rows: 1

   * - Source
     - Mainlined?
     -
     -
   * - :git+linux:`main:drivers/net/phy/adin.c`
     - `Yes <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/net/phy/adin.c>`__
     -
     -

Files
~~~~~

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - driver
     - :git+linux:`main:drivers/net/phy/adin.c`
     -
   * - Documentation
     - :git+linux:`adi,adin.yaml <main:Documentation/devicetree/bindings/net/adi,adin.yaml>`
     -

**Compatibility layer for Linux kernel version < 5.4 (Not required otherwise)**

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - compatibility layer 4.19
     - :git+linux:`2019_R2:drivers/net/phy/adin-compat.h`
     -

Example platform device initialization
--------------------------------------

The ADIN PHY driver instantiates via Linux’s phylib framework, which is
typically enabled on most systems.

Depending on the MAC driver that is used and the operating mode (MII, RGMII,
RMII), a device-tree entry for the PHY may or may-not be needed. The PHY can be
configured via HW pins
(:adi:`see datasheet <media/en/technical-documentation/data-sheets/ADIN1300.pdf>`),
or via SW.

**Optional properties (for MAC):**

- **phy-mode**: this is a standard Linux property for ethernet devices to select
  an operating mode for the PHY, it is typically configured in the MAC
  configuration, and the MAC uses it to configure the mode of the PHY. For the
  ADIN PHY, **accepted values** are: mii, rgmii, rgmii-id, rgmii-txid,
  rgmii-rxid, rmii

**Optional properties (for PHY):**

- **adi,rx-internal-delay-ps**: RGMII RX Clock Delay used only when PHY operates
  in RGMII mode with internal delay (phy-mode is ‘rgmii-id’ or ‘rgmii-rxid’) in
  pico-seconds. **Accepted values:** 1600, 1800, 2000, 2200, 2400, **Default:**
  2000.
- **adi,tx-internal-delay-ps**: RGMII TX Clock Delay used only when PHY operates
  in RGMII mode with internal delay (phy-mode is ‘rgmii-id’ or ‘rgmii-txid’) in
  pico-seconds. **Accepted values:** 1600, 1800, 2000, 2200, 2400, **Default:**
  2000.
- **adi,fifo-depth-bits**: When operating in RMII mode, this option configures
  the FIFO depth. **Accepted values:** 4, 8, 12, 16, 20, 24, **Default:** 8

::

   Example:

       ethernet-mac0 {
           #address-cells = <1>;
           #size-cells = <0>;

           phy-mode = "rgmii-id";

           ethernet-phy@0 {
               reg = <0>;

               adi,rx-internal-delay-ps = <1800>;
               adi,tx-internal-delay-ps = <2200>;
           };
       };

       ethernet-mac1 {
           #address-cells = <1>;
           #size-cells = <0>;

           phy-mode = "rmii";

           ethernet-phy@1 {
               reg = <1>;

               adi,fifo-depth-bits = <16>;
           };
       };

Enabling Linux driver support
-----------------------------

Configure kernel with ``make menuconfig`` (alternatively use ``make xconfig`` or
``make qconfig``)

#. Hit the search button (typically the slash ``/`` key)
#. Type ADIN_PHY, then hit Enter; if nothing shows up, the driver is not
   available in your kernel tree, please use
   `the ADI linux tree <https://github.com/analogdevicesinc/linux/>`__
#. Press 1 (the key), then hit Enter
#. You should see the location + dependencies for enabling the driver

::

   Linux Kernel Configuration
   Symbol: ADIN_PHY [=y]
   Type  : tristate
   Prompt: Analog Devices Industrial Ethernet PHYs
      Location:
        -> Device Drivers
          -> Network device support (NETDEVICES [=y])│
    (1)     -> PHY Device support and infrastructure (PHYLIB [=y])
      Defined at drivers/net/phy/Kconfig:208
      Depends on: NETDEVICES [=y] && PHYLIB [=y]

Driver testing
--------------

Short version is: a cable is plugged into the ethernet port (to which the PHY is
associated) and traffic starts to happen, as in a normal Linux OS environment.

But on the slightly more advanced side, see the sections below.

.. important::

   Depending on the userspace image that is being used, the network cards may
   [or may not get a DHCP address]. This is because there is a NetworkManager
   instance that handles this automatically.

   If NetworkManager isn’t running, it sometimes works to obtain a DHCP lease
   via **dhclient eth0** [or **eth1** or **eth2**, depending which MAC is
   attached to the ADIN PHY].

ifconfig
~~~~~~~~

This tool will display the general status of the available network interfaces.
If they’ve obtained an IP address, RX packets/errors/dropped/etc, TX
packets/errors/dropped/etc, MAC address, etc.

Typically, if both TX & RX values are incremented, it means that it is working.
Also note that there are error counters; if only the TX/RX counters increment,
something may be wrong in the configuration between MAC & PHY, or sometimes at
the physical configuration (i.e. clocks not working, pins not connected
properly, etc).

::

   root@analog:~# ifconfig
   eth0      Link encap:Ethernet  HWaddr d6:41:50:ed:3b:65
             UP BROADCAST MULTICAST  MTU:1500  Metric:1
             RX packets:609895 errors:5 dropped:0 overruns:0 frame:0
             TX packets:286926 errors:0 dropped:0 overruns:0 carrier:0
             collisions:0 txqueuelen:1000
             RX bytes:0 (647.6 MB)  TX bytes:0 (38.7 MB)
             Interrupt:23

   eth1      Link encap:Ethernet  HWaddr 00:0a:35:03:73:d9
             UP BROADCAST MULTICAST  MTU:1500  Metric:1
             RX packets:0 errors:0 dropped:0 overruns:0 frame:0
             TX packets:0 errors:0 dropped:0 overruns:0 carrier:0
             collisions:0 txqueuelen:1000
             RX bytes:0 (0.0 B)  TX bytes:0 (0.0 B)
             Interrupt:24

   lo        Link encap:Local Loopback
             inet addr:127.0.0.1  Mask:255.0.0.0
             inet6 addr: ::1/128 Scope:Host
             UP LOOPBACK RUNNING  MTU:65536  Metric:1
             RX packets:8 errors:0 dropped:0 overruns:0 frame:0
             TX packets:8 errors:0 dropped:0 overruns:0 carrier:0
             collisions:0 txqueuelen:1000
             RX bytes:480 (480.0 B)  TX bytes:480 (480.0 B)

ethtool
~~~~~~~

This tool queries the MAC & PHY via the MAC driver. The MAC driver should also
allows access to the PHY registers.

.. important::

   If ethtool does not work, it should be resolved in the MAC driver. The MAC
   driver is not in the direct scope of Analog Devices support, since MACs are
   typically manufactured & supported by other companies

ethtool can be used to show & override link settings and other parameters for
the MAC & PHY.

Links for the tool:

- Official page with source code:
  https://mirrors.edge.kernel.org/pub/software/network/ethtool/
- Manual page: http://man7.org/linux/man-pages/man8/ethtool.8.html

.. tip::

   Some features of ethtool described here are available in newer versions of
   ethtool. If some of them don’t work, consider upgrading or getting a newer
   version

Example: Seeing MAC & PHY info
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

   root@analog:~# ethtool eth0
   Settings for eth0:
           Supported ports: [ TP MII ]
           Supported link modes:   10baseT/Half 10baseT/Full
                                   100baseT/Half 100baseT/Full ==== Example: Seeing MAC & PHY info ====

                                   1000baseT/Half 1000baseT/Full
           Supported pause frame use: No
           Supports auto-negotiation: Yes
           Supported FEC modes: Not reported
           Advertised link modes:  10baseT/Half 10baseT/Full
                                   100baseT/Half 100baseT/Full
                                   1000baseT/Half 1000baseT/Full
           Advertised pause frame use: No
           Advertised auto-negotiation: Yes
           Advertised FEC modes: Not reported
           Link partner advertised link modes:  10baseT/Half 10baseT/Full
                                                100baseT/Half 100baseT/Full
           Link partner advertised pause frame use: Symmetric
           Link partner advertised auto-negotiation: Yes
           Link partner advertised FEC modes: Not reported
           Speed: 100Mb/s
           Duplex: Full
           Port: MII
           PHYAD: 0
           Transceiver: internal
           Auto-negotiation: on
           Link detected: yes

Example: changing link speeds
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

   ethtool -s eth0 speed 10 duplex full autoneg off
   ethtool -s eth0 speed 100 duplex full autoneg off
   ethtool -s eth0 autoneg on

Example: MDI/MDIX/Auto-MDIX configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The PHY supports configuration of the MDI/MDI-X auto-switch for twisted pairs.
This can also be configured via ethtool.

::

   ethtool -s eth0 mdix auto # auto-mdix (there is no way to set preference; # default MDI is preferred)
   ethtool -s eth0 mdix off # MDI mode manual
   ethtool -s eth0 mdix on # MDIX mode manual

Example: PHY statistics
^^^^^^^^^^^^^^^^^^^^^^^

.. important::

   make sure to not confuse PHY stats with default stats. Ethtool provides stats
   from the MAC, via **ethtool –statistics eth0** or **ethtool -S eth0**

::

   root@analog:~# ethtool --phy-statistics eth0
   PHY statistics:
        total_frames_checked_count: 1870
        length_error_frames_count: 342
        alignment_error_frames_count: 0
        symbol_error_count: 0
        oversized_frames_count: 0
        undersized_frames_count: 0
        odd_nibble_frames_count: 0
        odd_preamble_packet_count: 0
        dribble_bits_frames_count: 0
        false_carrier_events_count: 0

phytool
~~~~~~~

This tool is not very official, but it is very powerful for PHYs. It’s currently
hosted here: https://github.com/wkz/phytool

.. important::

   Can only access registers via Clause 22. Clause 45 does not work with the PHY
   driver.

Format of the command is:

::

   phytool read eth0/0/<reg-addr>
   phytool write eth0/0/<reg-addr> <16-bit-hex-value>

Example for reading the PHY ID:

::

   root@analog:~# phytool read eth0/0/0x2
   0x0283
   root@analog:~# phytool read eth0/0/0x3
   0xbc30

Accessing PHY-core registers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

All registers from 0x00 to 0x1f (which are defined by the IEEE standard) can be
accessed with just

::

   phytool read eth0/0/<reg-addr>
   phytool write eth0/0/<reg-addr>

Reading an MMD reg
^^^^^^^^^^^^^^^^^^

Since only Clause 22 is available accessing MMD regs needs to be done via
registers 0x10 & 0x11.

All registers above register address 0x1f are MMD registers.

Example reading FcFrmCntL reg (lower half of the **total_frames_checked_count**
value):

::

   root@analog:~# phytool write eth0/0/0x10 0x940B
   root@analog:~# phytool read eth0/0/0x11
   0x01c5

Writing an MMD reg
^^^^^^^^^^^^^^^^^^

Example resetting the PHY:

::

   root@analog:~# phytool write eth0/0/0x10 0xFF0C
   root@analog:~# phytool write eth0/0/0x11 0x1

adintool.sh - convenience phytool wrapper
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. warning::

   This tool requires sudo/root access and will do changes to the system. Use it
   with caution on a development system. Also, this tool can leave the PHY into
   a undetermined state of functioning, which may require a reset of the chip,
   or a reboot of the system to put the PHY back into a working state together
   with the attached MAC.

Link: :git+wiki-scripts:`linux/adintool.sh`

When downloading, make sure the script is made executable.

Usage
^^^^^

::

   Usage: adintool.sh <command> [args]
     setup - setup phytool and ethtool required for demo
             WARNING: will override system tools
     dump_regs <eth> - show all reg values
                       WARNING: some registers will be cleared on read
     phy_read_mmd <eth> <reg-addr> - read value from a MMD register
     phy_write_mmd <eth> <reg-addr> <val> - write value to MMD register
     cable_diagnostics <eth> - run cable diagnostics on cable
                               WARNING: puts device into special mode.
                                        device won't send data during this mode

First setup
^^^^^^^^^^^

Needs be run the first time. Make sure that there is internet connectivity and
then install basic things:

::

   ./adintool.sh setup

dump_regs
^^^^^^^^^

The most useful command is to dump registers. This will read all registers that
are specified in the datasheet and show their current value.

Should be used with caution as some registers get cleared on read, and can
interfere with some internal operation of the Linux driver (for EEE for
example).

Example:

::

   ./adintool.sh dump_regs eth0 [could be eth1]

cable_diagnostics
^^^^^^^^^^^^^^^^^

This runs a sequence of register changes to put the PHY into diagnostics mode,
which interrupts normal operation.

Then a diagnostics will be run on the cable and the results of the registers
will be read back.

::

   ./adintool.sh cable_diagnostics eth0

Throughput testing - iperf
~~~~~~~~~~~~~~~~~~~~~~~~~~

This is a more system-general test but it also validates the PHY.

.. tip::

   More tools are available for this sort of testing (iperf3, netperf, etc), but
   iperf is one of the more basic/simple ones to do this validation. If this one
   achieves expected results, others should too

On one of the endpoints with the ADIN1300, run:

::

   iperf -s

and on another system

::

   iperf -c <ip-addr-of-the-other-system>

Then reverse the commands on the hosts. iperf only works in one direction.

Data integrity testing
~~~~~~~~~~~~~~~~~~~~~~

One one side, generate a file with random data (say 1GB)

::

   dd if=/dev/urandom of=test.data bs=1M count=1000

   sha256sum test.data
   <SHA256-hash-of-data>

Then transfer the data to the other side with scp,ftp,etc:

::

   scp test.data root@<ip-addr-of-the-other-host>

On the other host check the hash

::

   sha256sum test.data
   <SHA256-hash-of-data> == should be identical with the first hash

Ethernet pointers
-----------------

- netdev mailing list: netdev@vger.kernel.org
- `Ethernet Controller bindings doc <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/Documentation/devicetree/bindings/net/ethernet-controller.yaml>`__
- `Ethernet PHY bindings doc <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/Documentation/devicetree/bindings/net/ethernet-phy.yaml>`__

.. include:: /include/need-help.rst
   :start-after: .. start-need-help
   :end-before: .. end-need-help
