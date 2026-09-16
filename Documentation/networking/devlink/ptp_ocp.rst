.. SPDX-License-Identifier: GPL-2.0

========================
ptp_ocp devlink support
========================

This document describes the devlink features implemented by the ``ptp_ocp``
device driver.

Info versions
=============

The ``ptp_ocp`` driver reports the following versions

.. list-table:: devlink info versions implemented
   :widths: 5 5 90

   * - Name
     - Type
     - Description
   * - ``fw``
     - running
     - Version of the firmware running on the card.  Reported as ``loader``
       instead when the card is running the firmware loader.
   * - ``board.id``
     - fixed
     - Board identifier, read from the on-card EEPROM.
   * - ``cpld.id``
     - fixed
     - Lattice device ID (IDCODE) of the TAP CPLD, formatted as ``0x%08x``.
       Only present on ADVA TimeCard X1 boards, which are the only ones
       carrying that CPLD.  Reading it claims the shared I2C bus and
       reprograms the on-card mux, so the driver does that once from its
       own worker and reports the cached value here; the version is
       omitted until that read has succeeded.
