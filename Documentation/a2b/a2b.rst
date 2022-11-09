==============
A2B Framework
==============

This driver is intended to be used for ADI AD242x series transceivers bus
topology. The bus topology can be described in device tree. The bus consists
of one main and one or more sub nodes. Main node is an i2c device which
configures each sub nodes on the bus. Each sub node is detected using
"discovery" process.
For each sub node the following platform device driver are available.

						+---------------+
						|   Main node	|
						|  I2C client	|
						+---------------+
							|
	-------------------------------------------------------------------------------------------------
	|			|			|			|			|
+---------------+	+---------------+	+---------------+	+---------------+	+---------------+
|   Sub node	|	|   Sub node	|	|   Sub node	|	|   Sub node	|	|   Sub node	|
|GPIO controller|	|     CCF	|	|    Codec	|	|    Mailbox	|	|  I2C client	|
+---------------+	+---------------+	+---------------+	+---------------+	+---------------+

I2C
====
It provides I2C and SMbus functionality to access a peripheral attached to a
sub node.

GPIO
=====
It provides gpio controller functionality to control gpios on a sub node. Each
pin can be configured as input or output.

Clock
======
It provides common clock framework apis to control the two output clocks of a sub node.

Codec
======
This is a codec driver to be used with sound framework like ALSA. It configures I2S/PDM
interface of a sub node.

Mailbox
========
It creates a mailbox device for a sub node. Mailbox is used to exchange data between
host processors of main and sub node. 
