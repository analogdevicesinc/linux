#
# Makefile for ccic devices
#
subdir-ccflags-y := -Wformat

max77978-y := max77978-core.o max77978-irq.o \
              max77978-cc.o max77978-bc12.o max77978-pd.o \
              max77978-usbc.o max77978-alternate.o
max77978-$(CONFIG_MAX77978_DEBUG) += max77978-debug.o
obj-$(CONFIG_MAX77978) += max77978.o
