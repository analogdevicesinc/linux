.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/demo/pmod-ad1-da1-demo

.. _pmod-ad1-da1-demo:

PmodAD1 - PmodDA1
=================

PmodAD1IIO Oscilloscope Demo.

Introduction
------------

The ADI IIO Oscilloscope is a example application, which demonstrates how to
interface to the ADI evaluation boards from within a Linux system.

Quick Start Guide
-----------------

Required Hardware
~~~~~~~~~~~~~~~~~

- `ZedBoard <http://www.zedboard.org>`__
- `PmodAD1 <http://www.digilentinc.com/Products/Detail.cfm?Prod=PMOD-AD1>`__
- `PmodDA1 <http://www.digilentinc.com/Products/Detail.cfm?Prod=PMOD-DA1>`__

Prepare the SD Card
~~~~~~~~~~~~~~~~~~~

Please use the image/description from
:dokuwiki:`Create SD Image for Zynq Boards <resources/tools-software/linux-software/zynq_images>`.

Hardware Setup
~~~~~~~~~~~~~~

- The PmodAD1 has to be connected to the PMOD JA1 connector (upper row - see the
  image below).
- The PmodDA1 has to be connected to the PMOD JB1 connector (upper row - see the
  image below).
- Connect the PmodDA1 J2-A1 output pin to PmodAD1 J2-3 input pin (see the image
  below).

.. figure:: https://wiki.analog.com/_media/resources/tools-software/linux-drivers/demo/hardware_setup.jpg
   :width: 300px

- Insert the created SD Card into the ZedBoard SD Card slot (J12 connector).
- Power up the ZedBoard.

Software Setup
~~~~~~~~~~~~~~

Launching the application
~~~~~~~~~~~~~~~~~~~~~~~~~

To start the IIO Oscilloscope open up the start menu of your system search for
``IIO Oscilloscope``. E.g. if you are using a Ubuntu Linux system move your
mouse cursor to the left side of your screen and ``Dash home`` button and type
``IIO Oscilloscope`` into the search box.

.. figure:: https://wiki.analog.com/_media/resources/tools-software/linux-drivers/demo/launching_osc.png
   :width: 600px

IIO Oscilloscope Users Guide
----------------------------

Open the **Settings** menu and select **Impulse generator** submenu. A popup
window will appear and allow you to select an impulse generator (a high
resolution timer) and its frequency. The conversions for both the ADC and the
DAC are started by the impulses of the generator. Click the **OK** button.

.. figure:: https://wiki.analog.com/_media/resources/tools-software/linux-drivers/demo/input_generator.png
   :width: 500px

Select the **AD7303** tab in order to configure the DAC. You can choose to
output a constant value between 0 and 255 by selecting the **Single value
output** radio button or you can select the **Waveform output** radio button for
time-varying signals. After the configuration is done, click on the **Save**
button for the changes to apply.

.. figure:: https://wiki.analog.com/_media/resources/tools-software/linux-drivers/demo/dac_panel.png
   :width: 500px

Once the Impulse Generator and the DAC are configured you can switch back to the
**Capture** tab, select the **ad7476a** device from the **Device** list and
select one of the available channels (**in_voltage0**). The final step is to
start the capture process by clicking the right arrow button. You can stop the
sampling by clicking the same button again.

.. figure:: https://wiki.analog.com/_media/resources/tools-software/linux-drivers/demo/adc_panel.png
   :width: 500px
