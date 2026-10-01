.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/net-mac-phy/open_alliance

.. _open_alliance:

TC6-10BASE-T1x
==============

Open Alliance TC6-10BASE-T1x MAC-PHY Serial Interface Linux Driver.

Supported Devices
-----------------

- :adi:`ADIN1110`
- :adi:`ADIN2111`

Supported Boards
----------------

Description
-----------

The IEEE 802.3cg project defines two 10 Mbit/s PHYs operating over a single pair
of conductors. The 10BASE-T1L (Clause 146) is a long reach PHY supporting full
duplex point-to-point operation over 1 km of single balanced pair of conductors.
The 10BASE-T1S (Clause 147) is a short reach PHY supporting full / half duplex
point-to-point operation over 15 m of single balanced pair of conductors, or
half duplex multidrop bus operation over 25 m of single balanced pair of
conductors.

Furthermore, the IEEE 802.3cg project defines the new Physical Layer Collision
Avoidance (PLCA) Reconciliation Sublayer (Clause 148) meant to provide improved
determinism to the CSMA/CD media access method. PLCA works in conjunction with
the 10BASE-T1S PHY operating in multidrop mode.

The aforementioned PHYs are intended to cover the low-speed / low-cost
applications in industrial and automotive environment. The large number of pins
(16) required by the MII interface, which is specified by the IEEE 802.3 in
Clause 22, is one of the major cost factors that need to be addressed to fulfil
this objective.

The MAC-PHY solution integrates an IEEE Clause 4 MAC and a 10BASE-T1x PHY
exposing a low pin count Serial Peripheral Interface (SPI) to the host
microcontroller. This also enables the addition of Ethernet functionality to
existing low-end microcontrollers which do not integrate a MAC controller.

Source Code
-----------

Status
~~~~~~

.. list-table::

   * - Source
     - Mainlined?
     -
   * - :git+linux:`open_alliance.c <adin1110-open-alliance:drivers/net/ethernet/open_alliance.c>`
     - No
     -
   * - :git+linux:`open_alliance.h <adin1110-open-alliance:include/linux/open_alliance.h>`
     - No
     -
   * - :git+linux:`adin1110.c <adin1110-open-alliance:drivers/net/ethernet/adi/adin1110.c>`
     - No
     -

Files
~~~~~

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - API
     - :git+linux:`drivers/net/ethernet/open_alliance.c <adin1110-open-alliance:drivers/net>`
     -
   * - API
     - :git+linux:`include/linux/open_alliance.h <adin1110-open-alliance:include/linux>`
     -
   * - Driver
     - :git+linux:`drivers/net/ethernet/adi/adin1110.c <adin1110-open-alliance:drivers/net/ethernet/adi/>`
     -

Enabling Linux driver support
-----------------------------

Configure kernel with ``make menuconfig`` (alternatively use ``make xconfig`` or
``make qconfig``)

#. Hit the search button (typically the slash ``/`` key)
#. Type open_alliance, then hit Enter; if nothing shows up, the driver is not
   available in your kernel tree, please use
   `the ADI linux tree <https://github.com/analogdevicesinc/linux/>`__
#. Press 1 (the key), then hit Enter
#. You should see the location + dependencies for enabling the driver

::

   CONFIG_OPEN_ALLIANCE:
   The Open Alliance SPI protocol is used on 10BASE-T1x
   MACPHY Serial Interface for communication
   with the host controller.

   Say Y to include support for the OPEN_ALLIANCE.
   To compile this driver as a module, choose M here: the module
   will be called dnet.

   Symbol: OPEN_ALLIANCE [=y]
   Type  : tristate
   Defined at drivers/net/ethernet/Kconfig:147
       Prompt: Open Alliance 10BASE-T1x support.
       Depends on: NETDEVICES [=y] && ETHERNET [=y] && SPI [=y]
       Location:
           -> Device Drivers
               -> Network device support (NETDEVICES [=y])
                   -> Ethernet driver support (ETHERNET [=y])
       Selects: CRC8 [=y]

Driver testing
--------------

This requires that another 10BASE-T1L PHY be connected to the other end of the
network cable, or that a media converter be used to convert to normal
twisted-pair ethernet that standard ethernet cables use.

ADIN1110 communicates with the host via SPI. For 10 Mbps bandwidth, SPI
frequency needs to be around 23 MHz. Lower SPI frequencies are supported but
will result in a lower bandwidth. At 1 MHz the MAC will provide aprox. 0.4 Mbps
of bandwidth.

Connect to host the SCLK, CS_N, SDI, SDO and INT_N. (The INT_N is mandatory, see
DT bindings). RX frames and sent TX frames are signaled to the host by INT_N IRQ
pin.

Device Tree
~~~~~~~~~~~

ADIN1110 probes via devicetree.

::

   ethernet@0 {
       compatible = "adi,adin1110";

           /* SPI CS number */
           reg = <0>;

           /* will need 13.5 MHz for 10 Mbps, lower speeds will result in lower bandwidth */
       spi-max-frequency = <13500000>;

           /* Enables Open Alliance Mode */
           open-alliance;

           /* optional, will check all control read/writes over SPI */
           open-alliance-protected;

       #address-cells = <1>;
       #size-cells = <0>;

           /* an IRQ is required, INT_N pin is configured to signal RX/TX frames */
       interrupt-parent = <&gpio>;
       interrupts = <25 2>;

       mac-address = [ CA 2F B7 10 23 63 ];

       phy@0 {
           compatible = "ethernet-phy-id0283.bc91";
           reg = <0x0>;
       };
   };

ifconfig
~~~~~~~~

This tool will display the general status of the available network interfaces.
If they’ve obtained an IP address, RX packets/errors/dropped/etc, TX
packets/errors/dropped/etc, MAC address, etc.

Typically, if both TX & RX values are incremented, it means that it is working.
Also note that there are error counters; if only the TX/RX counters increment,
something may be wrong with the network connection. Check error/dropped counters
too.

::

   root@analog:~# ifconfig eth1
   eth1: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
           inet 192.168.160.160  netmask 255.255.255.0  broadcast 192.168.160.255
           inet6 fe80::c82f:b7ff:fe10:2363  prefixlen 64  scopeid 0x20<link>
           ether ca:2f:b7:10:23:63  txqueuelen 1000  (Ethernet)
           RX packets 132  bytes 8548 (8.3 KiB)
           RX errors 0  dropped 0  overruns 0  frame 0
           TX packets 79  bytes 9943 (9.7 KiB)
           TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
           device interrupt 55

ethtool
~~~~~~~

This tool queries the MAC & PHY via the MAC driver. The MAC driver also allows
access to the PHY registers. ethtool can be used to show & override link
settings and other parameters for the MAC & PHY.

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

   Settings for eth1:
       Supported ports: [ TP MII ]
       Supported link modes:   10baseT/Full
       Supported pause frame use: Symmetric Receive-only
       Supports auto-negotiation: Yes
       Supported FEC modes: Not reported
       Advertised link modes:  10baseT/Full
       Advertised pause frame use: No
       Advertised auto-negotiation: Yes
       Advertised FEC modes: Not reported
       Link partner advertised link modes:  10baseT/Full
       Link partner advertised pause frame use: No
       Link partner advertised auto-negotiation: Yes
       Link partner advertised FEC modes: Not reported
       Speed: 10Mb/s
       Duplex: Full
       Port: MII
       PHYAD: 0
       Transceiver: internal
       Auto-negotiation: on
       Link detected: yes

Open Alliance also exposes Chunk level statistics:

::

   root@analog:~/ethtool-5.15# ethtool -S adin2111-0-p0
   NIC statistics:
        RX valid data chunks received: 362
        TX chunks sent: 1440
        RX end valid chunks received: 61
        RX chunks to frames: 362
        OA chunks transfered: 1802
