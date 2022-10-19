.. SPDX-License-Identifier: GPL-2.0

======================================================================
This document provides the example about A2B device tree configuration
======================================================================

In general about the A2B transceiver technical reference manual can be found in the below link
https://www.analog.com/media/en/technical-documentation/user-guides/AD242x_TRM_Rev1.1.pdf

AD242x Part Datasheet can be found in the below link
https://www.analog.com/media/en/technical-documentation/data-sheets/ad2420-ad2426-ad2427-ad2428-ad2429.pdf

For example, single chain with 3 node schematic will look like below with one main a2b transceiver node & 2 sub nodes:
::

   ---------          ---------          ---------          ---------
  |         |  i2c   |         |  utp   |         |  utp   |         |
      host   ------->   main    ------->   sub0    ------->   sub1
  |         |  i2s   |         |        |         |        |         |
   ---------          ---------          ---------          ---------
   
Configuration for above schematic in the device tree looks like below (ex:a2b main node address : 0x68):
a2b@68 {
	compatible = "adi,ad2428w";
	reg = <0x68>;
	.. Here main node configuration to be added
	nodes {
		node@0 {
			.. Here sub0 configuration to be added
		}
		node@1 {
			.. Here sub1 configuration to be added
		}
	}
}

For example, multi main node schematic will look like below with 2 main nodes & 2 sub nodes each:
::

   ---------          ---------          ---------          ---------
  |         |  i2c   |         |  utp   |         |  utp   |         |
  |         | ------>   main0   ------->   sub0    ------->   sub1
  |         |  i2s   |         |        |         |        |         |
  |         |         ---------          ---------          ---------
  |  host   | 
  |         |         ---------          ---------          ---------
  |         |  i2c   |         |  utp   |         |  utp   |         |
  |         | ------>   main1   ------->   sub0    ------->   sub1
  |         |  i2s   |         |        |         |        |         |
   ---------          ---------          ---------          ---------

Configuration for above schematic in the device tree looks like below (ex:a2b main node's addresses : 0x68/6a):
a2b@68 {
	compatible = "adi,ad2428w";
	reg = <0x68>;
	.. Here main0 configuration to be added
	nodes {
		node@0 {
			.. Here sub0 configuration to be added
		}
		node@1 {
			.. Here sub1 configuration to be added
		}
	}
}
a2b@6a {
	compatible = "adi,ad2428w";
	reg = <0x6a>;
	.. Here main1 configuration to be added
	nodes {
		node@0 {
			.. Here sub0 configuration to be added
		}
		node@1 {
			.. Here sub1 configuration to be added
		}
	}
}
