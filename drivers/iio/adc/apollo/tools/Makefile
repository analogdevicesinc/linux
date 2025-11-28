# SPDX-License-Identifier: GPL-2.0
#
# Makefile for AD9088 Calibration Dump Tool
#

CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c99
LDFLAGS = -lz

TARGET = ad9088_cal_dump
SOURCES = ad9088_cal_dump.c

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(TARGET)

install: $(TARGET)
	install -m 755 $(TARGET) /usr/local/bin/

.PHONY: all clean install
