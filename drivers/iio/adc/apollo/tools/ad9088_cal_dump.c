// SPDX-License-Identifier: GPL-2.0
/*
 * AD9088 Calibration Data Dump Tool
 *
 * Copyright 2025 Analog Devices Inc.
 *
 * Usage: ad9088_cal_dump <calibration_file>
 *
 * This tool reads an AD9088 calibration data file and displays:
 * - Header information (magic, version, chip ID, configuration)
 * - Section offsets and sizes
 * - CRC validation
 * - Calibration data summary
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <zlib.h>  /* For CRC32 calculation */

/* Magic number for AD9088 calibration files */
#define AD9088_CAL_MAGIC    0x41443930  /* "AD90" */
#define AD9088_CAL_VERSION  1

/* Chip IDs */
#define CHIPID_AD9084 0x9084
#define CHIPID_AD9088 0x9088

/* Number of devices */
#define ADI_APOLLO_NUM_ADC_CAL_MODES           2
#define ADI_APOLLO_NUM_JRX_SERDES_12PACKS      4
#define ADI_APOLLO_NUM_JTX_SERDES_12PACKS      4

/* Calibration file header structure (must match kernel driver) */
struct ad9088_cal_header {
	uint32_t magic;			/* Magic number "AD90" */
	uint32_t version;		/* File format version */
	uint32_t chip_id;		/* Chip ID (0x9084 or 0x9088) */
	uint8_t  is_8t8r;		/* 1 = 8T8R, 0 = 4T4R */
	uint8_t  num_adcs;		/* Number of ADCs */
	uint8_t  num_dacs;		/* Number of DACs */
	uint8_t  num_serdes_rx;		/* Number of SERDES RX 12-packs */
	uint8_t  num_serdes_tx;		/* Number of SERDES TX 12-packs */
	uint8_t  reserved1[3];		/* Padding to 4-byte boundary */

	/* Section offsets from start of file */
	uint32_t adc_cal_offset;	/* Offset to ADC cal data */
	uint32_t dac_cal_offset;	/* Offset to DAC cal data */
	uint32_t serdes_rx_cal_offset;	/* Offset to SERDES RX cal data */
	uint32_t serdes_tx_cal_offset;	/* Offset to SERDES TX cal data */

	/* Section sizes */
	uint32_t adc_cal_size;		/* Total size of all ADC cal data */
	uint32_t dac_cal_size;		/* Total size of all DAC cal data */
	uint32_t serdes_rx_cal_size;	/* Total size of all SERDES RX cal data */
	uint32_t serdes_tx_cal_size;	/* Total size of all SERDES TX cal data */

	uint32_t total_size;		/* Total file size including CRC */
	uint32_t reserved2[2];		/* Reserved for future use */
} __attribute__((packed));

static const char *chip_id_to_string(uint32_t chip_id)
{
	switch (chip_id) {
	case CHIPID_AD9084:
		return "AD9084";
	case CHIPID_AD9088:
		return "AD9088";
	default:
		return "Unknown";
	}
}

static void print_header(const struct ad9088_cal_header *hdr)
{
	printf("=== AD9088 Calibration Data Header ===\n\n");

	printf("Magic Number:        0x%08X ('%c%c%c%c') %s\n",
	       hdr->magic,
	       (hdr->magic >> 0) & 0xFF,
	       (hdr->magic >> 8) & 0xFF,
	       (hdr->magic >> 16) & 0xFF,
	       (hdr->magic >> 24) & 0xFF,
	       hdr->magic == AD9088_CAL_MAGIC ? "[OK]" : "[INVALID]");

	printf("Version:             %u %s\n",
	       hdr->version,
	       hdr->version == AD9088_CAL_VERSION ? "[OK]" : "[UNSUPPORTED]");

	printf("Chip ID:             0x%04X (%s)\n",
	       hdr->chip_id,
	       chip_id_to_string(hdr->chip_id));

	printf("Configuration:       %s\n",
	       hdr->is_8t8r ? "8T8R (8 TX, 8 RX)" : "4T4R (4 TX, 4 RX)");

	printf("Number of ADCs:      %u\n", hdr->num_adcs);
	printf("Number of DACs:      %u\n", hdr->num_dacs);
	printf("Number of SERDES RX: %u\n", hdr->num_serdes_rx);
	printf("Number of SERDES TX: %u\n", hdr->num_serdes_tx);

	printf("\n=== Calibration Sections ===\n\n");

	printf("ADC Calibration:\n");
	printf("  Offset: 0x%08X (%u bytes)\n", hdr->adc_cal_offset, hdr->adc_cal_offset);
	printf("  Size:   0x%08X (%u bytes)\n", hdr->adc_cal_size, hdr->adc_cal_size);
	if (hdr->num_adcs > 0 && hdr->adc_cal_size > 0) {
		uint32_t per_mode = hdr->adc_cal_size / ADI_APOLLO_NUM_ADC_CAL_MODES;
		uint32_t per_adc = per_mode / hdr->num_adcs;
		printf("  Per Mode: %u bytes\n", per_mode);
		printf("  Per ADC:  %u bytes\n", per_adc);
	}

	printf("\nDAC Calibration:\n");
	printf("  Offset: 0x%08X (%u bytes)\n", hdr->dac_cal_offset, hdr->dac_cal_offset);
	printf("  Size:   0x%08X (%u bytes)\n", hdr->dac_cal_size, hdr->dac_cal_size);
	if (hdr->num_dacs > 0 && hdr->dac_cal_size > 0) {
		uint32_t per_dac = hdr->dac_cal_size / hdr->num_dacs;
		printf("  Per DAC:  %u bytes\n", per_dac);
	}

	printf("\nSERDES RX Calibration:\n");
	printf("  Offset: 0x%08X (%u bytes)\n", hdr->serdes_rx_cal_offset, hdr->serdes_rx_cal_offset);
	printf("  Size:   0x%08X (%u bytes)\n", hdr->serdes_rx_cal_size, hdr->serdes_rx_cal_size);
	if (hdr->num_serdes_rx > 0 && hdr->serdes_rx_cal_size > 0) {
		uint32_t per_serdes = hdr->serdes_rx_cal_size / hdr->num_serdes_rx;
		printf("  Per Pack: %u bytes\n", per_serdes);
	}

	printf("\nSERDES TX Calibration:\n");
	printf("  Offset: 0x%08X (%u bytes)\n", hdr->serdes_tx_cal_offset, hdr->serdes_tx_cal_offset);
	printf("  Size:   0x%08X (%u bytes)\n", hdr->serdes_tx_cal_size, hdr->serdes_tx_cal_size);
	if (hdr->num_serdes_tx > 0 && hdr->serdes_tx_cal_size > 0) {
		uint32_t per_serdes = hdr->serdes_tx_cal_size / hdr->num_serdes_tx;
		printf("  Per Pack: %u bytes\n", per_serdes);
	}

	printf("\nTotal Size:          0x%08X (%u bytes)\n",
	       hdr->total_size, hdr->total_size);
}

static void print_section_summary(const char *section_name, const uint8_t *data,
				   uint32_t offset, uint32_t size,
				   uint32_t num_items, const char *item_name)
{
	uint32_t i, per_item;
	bool all_zero = true;
	bool all_ff = true;

	if (size == 0 || num_items == 0) {
		printf("\n=== %s: Empty ===\n", section_name);
		return;
	}

	per_item = size / num_items;

	printf("\n=== %s ===\n", section_name);
	printf("Total size: %u bytes (%u items × %u bytes)\n\n",
	       size, num_items, per_item);

	/* Check for all zeros or all 0xFF (likely uninitialized) */
	for (i = 0; i < size && (all_zero || all_ff); i++) {
		if (data[i] != 0x00)
			all_zero = false;
		if (data[i] != 0xFF)
			all_ff = false;
	}

	if (all_zero) {
		printf("WARNING: All data is zero (possibly uninitialized)\n");
	} else if (all_ff) {
		printf("WARNING: All data is 0xFF (possibly uninitialized)\n");
	}

	/* Print first 16 bytes of each item */
	for (i = 0; i < num_items; i++) {
		uint32_t item_offset = i * per_item;
		uint32_t j, bytes_to_show;

		printf("%s %u (offset 0x%08X):\n", item_name, i,
		       offset + item_offset);

		bytes_to_show = per_item < 16 ? per_item : 16;
		printf("  ");
		for (j = 0; j < bytes_to_show; j++) {
			printf("%02X ", data[item_offset + j]);
			if ((j + 1) % 16 == 0)
				printf("\n  ");
		}
		if (bytes_to_show < per_item)
			printf("... (%u more bytes)", per_item - bytes_to_show);
		printf("\n");
	}
}

static int validate_and_dump(const char *filename)
{
	FILE *fp;
	uint8_t *data;
	size_t file_size, read_size;
	struct ad9088_cal_header *hdr;
	uint32_t crc_stored = 0, crc_calc = 0;
	int ret = 0;

	/* Open file */
	fp = fopen(filename, "rb");
	if (!fp) {
		fprintf(stderr, "Error: Cannot open file '%s': %s\n",
			filename, strerror(errno));
		return 1;
	}

	/* Get file size */
	fseek(fp, 0, SEEK_END);
	file_size = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	printf("File: %s\n", filename);
	printf("Size: %zu bytes\n\n", file_size);

	/* Check minimum size */
	if (file_size < sizeof(struct ad9088_cal_header) + 4) {
		fprintf(stderr, "Error: File too small (%zu bytes)\n", file_size);
		fclose(fp);
		return 1;
	}

	/* Read entire file */
	data = malloc(file_size);
	if (!data) {
		fprintf(stderr, "Error: Cannot allocate memory\n");
		fclose(fp);
		return 1;
	}

	read_size = fread(data, 1, file_size, fp);
	fclose(fp);

	if (read_size != file_size) {
		fprintf(stderr, "Error: Read only %zu of %zu bytes\n",
			read_size, file_size);
		free(data);
		return 1;
	}

	/* Parse header */
	hdr = (struct ad9088_cal_header *)data;

	/* Validate magic number */
	if (hdr->magic != AD9088_CAL_MAGIC) {
		fprintf(stderr, "Error: Invalid magic number 0x%08X (expected 0x%08X)\n",
			hdr->magic, AD9088_CAL_MAGIC);
		ret = 1;
		goto out_print_header;
	}

	/* Validate version */
	if (hdr->version != AD9088_CAL_VERSION) {
		fprintf(stderr, "Warning: Unsupported version %u (expected %u)\n",
			hdr->version, AD9088_CAL_VERSION);
	}

	/* Validate total size */
	if (hdr->total_size != file_size) {
		fprintf(stderr, "Warning: Size mismatch - header says %u, file is %zu\n",
			hdr->total_size, file_size);
	}

	/* Extract and verify CRC */
	memcpy(&crc_stored, data + file_size - 4, 4);
	crc_calc = crc32(0L, data, file_size - 4);

	printf("=== CRC Validation ===\n\n");
	printf("Stored CRC:     0x%08X\n", crc_stored);
	printf("Calculated CRC: 0x%08X\n", crc_calc);
	printf("Status:         %s\n\n", crc_stored == crc_calc ? "[OK]" : "[FAILED]");

	if (crc_stored != crc_calc) {
		fprintf(stderr, "Error: CRC mismatch!\n");
		ret = 1;
	}

out_print_header:
	/* Print header information */
	print_header(hdr);

	/* Print section summaries if CRC is valid */
	if (crc_stored == crc_calc && ret == 0) {
		/* ADC calibration */
		if (hdr->adc_cal_size > 0 && hdr->adc_cal_offset < file_size) {
			uint32_t num_items = hdr->num_adcs * ADI_APOLLO_NUM_ADC_CAL_MODES;
			print_section_summary("ADC Calibration Data",
					      data + hdr->adc_cal_offset,
					      hdr->adc_cal_offset,
					      hdr->adc_cal_size,
					      num_items,
					      "ADC Chan/Mode");
		}

		/* DAC calibration */
		if (hdr->dac_cal_size > 0 && hdr->dac_cal_offset < file_size) {
			print_section_summary("DAC Calibration Data",
					      data + hdr->dac_cal_offset,
					      hdr->dac_cal_offset,
					      hdr->dac_cal_size,
					      hdr->num_dacs,
					      "DAC");
		}

		/* SERDES RX calibration */
		if (hdr->serdes_rx_cal_size > 0 && hdr->serdes_rx_cal_offset < file_size) {
			print_section_summary("SERDES RX Calibration Data",
					      data + hdr->serdes_rx_cal_offset,
					      hdr->serdes_rx_cal_offset,
					      hdr->serdes_rx_cal_size,
					      hdr->num_serdes_rx,
					      "SERDES RX Pack");
		}

		/* SERDES TX calibration */
		if (hdr->serdes_tx_cal_size > 0 && hdr->serdes_tx_cal_offset < file_size) {
			print_section_summary("SERDES TX Calibration Data",
					      data + hdr->serdes_tx_cal_offset,
					      hdr->serdes_tx_cal_offset,
					      hdr->serdes_tx_cal_size,
					      hdr->num_serdes_tx,
					      "SERDES TX Pack");
		}
	}

	printf("\n");
	free(data);
	return ret;
}

int main(int argc, char *argv[])
{
	if (argc != 2) {
		fprintf(stderr, "Usage: %s <calibration_file>\n", argv[0]);
		fprintf(stderr, "\n");
		fprintf(stderr, "Example:\n");
		fprintf(stderr, "  %s /lib/firmware/ad9088_cal.bin\n", argv[0]);
		fprintf(stderr, "\n");
		return 1;
	}

	return validate_and_dump(argv[1]);
}
