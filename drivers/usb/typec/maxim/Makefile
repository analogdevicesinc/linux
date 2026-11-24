#
# Makefile for ccic devices
#
subdir-ccflags-y := -Wformat
max77958-y := max77958-core.o max77958-irq.o \
              max77958-cc.o max77958-bc12.o max77958-pd.o \
              max77958-usbc.o max77958-alternate.o
max77958-$(CONFIG_MAX77958_DEBUG) += max77958-debug.o
obj-$(CONFIG_MAX77958) += max77958.o
