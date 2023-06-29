// SPDX-License-Identifier: GPL-2.0

// Binds a metavariable (WF) to the write function assigned to the converter.
@ ruleW @ 
idexpression struct cf_axi_converter *C;
identifier WF;
@@

{
	...
	C->write = WF;
	...
}

// Updates the converter write function to receive a device instead of spi_device.
@ ruleWU depends on ever ruleW @ 
identifier D, ruleW.WF;
@@

WF(...,
-		struct spi_device *D,
+		struct device *dev,
	...)
{
+	struct spi_device *D = to_spi_device(dev);
	...
}

// Update calls to the converter write function passing a device instead of spi_device.
@ ruleWU2 depends on ever ruleW @ 
idexpression struct cf_axi_converter *C;
identifier ruleW.WF;
//identifier ruleW.C;
//idexpression ruleW.C;
@@

	WF(...,
-		C->spi,
+		C->dev,
	...)

// Update calls to the converter write function passing a device instead of spi_device.
@ ruleWU3 depends on ever ruleW @ 
idexpression struct spi_device *D;
identifier ruleW.WF;
expression E;
@@

	WF(...,
-		D,
+		&D->dev,
	...)

// Binds a metavariable (RF) to the read function assigned to the converter.
@ ruleR @ 
idexpression struct cf_axi_converter *C;
//identifier C, RF;
identifier RF;
@@

{
	...
	C->read = RF;
	...
}

// Updates the converter read function to receive a device instead of spi_device.
@ ruleRU depends on ever ruleR @ 
identifier D, ruleR.RF;
@@

RF(...,
-		struct spi_device *D,
+		struct device *dev,
	...)
{
+	struct spi_device *D = to_spi_device(dev);
	...
}

// Update calls to the converter read function passing a device instead of spi_device.
@ ruleRU2 depends on ever ruleR @ 
idexpression struct cf_axi_converter *C;
identifier ruleR.RF;
@@

	RF(...,
-		C->spi,
+		C->dev,
	...)

// Update calls to the converter read function passing a device instead of spi_device.
@ ruleRU3 depends on ever ruleR @ 
idexpression struct spi_device *D;
identifier ruleR.RF;
@@

	RF(...,
-		D,
+		&D->dev,
	...)



// Renames cf_axi_converter field from spi to dev.
@ rule_rename3 exists @ 
idexpression struct cf_axi_converter *C;
@@

-		&C->spi->dev
+		C->dev

// Get spi device from converter device.
@ rule_rename4 exists @ 
idexpression struct cf_axi_converter *C;
identifier I;
@@

-		I = C->spi
+		I = to_spi_device(C->dev)

// Unwrap the device from struct spi_device when assigning to converter.
@ rule_rename5 exists @ 
idexpression struct cf_axi_converter *C;
expression E;
@@

-		C->spi = E
+		C->dev = &E->dev

// Update the way drvdata is set.
@ rule_rename6 exists @ 
idexpression struct cf_axi_converter *C;
expression E;
@@

-       spi_set_drvdata(E, C);
+       dev_set_drvdata(&E->dev, C);


// TODO  		dev_err(&st->conv.spi->dev,


// TODO handle:
//-			of_clk_get_scale(conv->spi->dev.of_node, ad9162_clks[i].name,
//+ 		of_clk_get_scale(conv->dev->of_node, ad9162_clks[i].name,


// dds.c
//                        else
//-                               ret = conv->write(conv->spi,
//+                               ret = conv->write(conv->dev,
//                                                  reg, writeval & 0xFF);
//                }
//        } else {
//@@ -996,7 +996,7 @@ static int cf_axi_dds_reg_access(struct iio_dev *indio_dev,
//                        else if (!conv->read)
//                                ret = -ENODEV;
//                        else
//-                               ret = conv->read(conv->spi, reg);
//+                               ret = conv->read(conv->dev, reg);
//                        if (ret < 0)

