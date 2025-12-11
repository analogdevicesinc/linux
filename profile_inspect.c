/*
 * Profile Inspector - Read and display AD9088 profile binary attributes
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Include Apollo API types */
#include "apollo_cpu_device_profile_types.h"

int main(int argc, char *argv[])
{
	FILE *fp;
	adi_apollo_top_t profile;
	size_t bytes_read;
	int i;
	const char *profile_file = "firmware/204C_M4_L4_NP16_20p0_4x4.bin";

	if (argc > 1) {
		profile_file = argv[1];
	}

	printf("Profile Inspector\n");
	printf("=================\n");
	printf("Reading profile: %s\n", profile_file);
	printf("Profile structure size: %zu bytes\n\n", sizeof(adi_apollo_top_t));

	fp = fopen(profile_file, "rb");
	if (!fp) {
		fprintf(stderr, "Error: Cannot open %s\n", profile_file);
		return 1;
	}

	/* Read the entire profile structure */
	memset(&profile, 0, sizeof(profile));
	bytes_read = fread(&profile, 1, sizeof(adi_apollo_top_t), fp);
	fclose(fp);

	if (bytes_read < sizeof(adi_apollo_top_t)) {
		fprintf(stderr, "Warning: Read only %zu bytes, expected %zu bytes\n",
			bytes_read, sizeof(adi_apollo_top_t));
	}

	printf("Profile Header:\n");
	printf("  Profile Checksum: 0x%08x\n", profile.profile_checksum);
	printf("\n");

	/* Display RX FSRC configuration */
	printf("RX FSRC Configuration (Side A):\n");
	printf("  Index | Enable | Mode 1x | Bypass | Rate Int        | Rate Frac A     | Rate Frac B\n");
	printf("  ------|--------|---------|--------|-----------------|-----------------|----------------\n");
	for (i = 0; i < ADI_APOLLO_FSRCS_PER_SIDE; i++) {
		adi_apollo_fsrc_cfg_t *fsrc = &profile.rx_path[0].rx_fsrc[i];
		printf("  %5d | %6d | %7d | %6s | 0x%012llx | 0x%012llx | 0x%012llx\n",
		       i,
		       fsrc->enable,
		       fsrc->mode_1x,
		       fsrc->enable ? "NO" : "YES",
		       (unsigned long long)fsrc->fsrc_rate_int,
		       (unsigned long long)fsrc->fsrc_rate_frac_a,
		       (unsigned long long)fsrc->fsrc_rate_frac_b);
	}
	printf("\n");

	/* Display TX FSRC configuration */
	printf("TX FSRC Configuration (Side A):\n");
	printf("  Index | Enable | Mode 1x | Bypass | Rate Int        | Rate Frac A     | Rate Frac B\n");
	printf("  ------|--------|---------|--------|-----------------|-----------------|----------------\n");
	for (i = 0; i < ADI_APOLLO_FSRCS_PER_SIDE; i++) {
		adi_apollo_fsrc_cfg_t *fsrc = &profile.tx_path[0].tx_fsrc[i];
		printf("  %5d | %6d | %7d | %6s | 0x%012llx | 0x%012llx | 0x%012llx\n",
		       i,
		       fsrc->enable,
		       fsrc->mode_1x,
		       fsrc->enable ? "NO" : "YES",
		       (unsigned long long)fsrc->fsrc_rate_int,
		       (unsigned long long)fsrc->fsrc_rate_frac_a,
		       (unsigned long long)fsrc->fsrc_rate_frac_b);
	}
	printf("\n");

	/* Display additional FSRC parameters for first FSRC */
	printf("RX FSRC[0] Detailed Configuration:\n");
	printf("  Enable: %d\n", profile.rx_path[0].rx_fsrc[0].enable);
	printf("  Mode 1x: %d\n", profile.rx_path[0].rx_fsrc[0].mode_1x);
	printf("  Sample Delay: %u\n", profile.rx_path[0].rx_fsrc[0].fsrc_delay);
	printf("  Gain Reduction: %u\n", profile.rx_path[0].rx_fsrc[0].gain_reduction);
	printf("  Data Mult Dither Enable: %d\n", profile.rx_path[0].rx_fsrc[0].data_mult_dither_en);
	printf("  Dither Enable: %d\n", profile.rx_path[0].rx_fsrc[0].dither_en);
	printf("  Split 4T4R: %d\n", profile.rx_path[0].rx_fsrc[0].split_4t4r);
	printf("\n");

	printf("TX FSRC[0] Detailed Configuration:\n");
	printf("  Enable: %d\n", profile.tx_path[0].tx_fsrc[0].enable);
	printf("  Mode 1x: %d\n", profile.tx_path[0].tx_fsrc[0].mode_1x);
	printf("  Sample Delay: %u\n", profile.tx_path[0].tx_fsrc[0].fsrc_delay);
	printf("  Gain Reduction: %u\n", profile.tx_path[0].tx_fsrc[0].gain_reduction);
	printf("  Data Mult Dither Enable: %d\n", profile.tx_path[0].tx_fsrc[0].data_mult_dither_en);
	printf("  Dither Enable: %d\n", profile.tx_path[0].tx_fsrc[0].dither_en);
	printf("  Split 4T4R: %d\n", profile.tx_path[0].tx_fsrc[0].split_4t4r);
	printf("\n");

	/* Summary */
	int rx_enabled = 0, tx_enabled = 0;
	for (i = 0; i < ADI_APOLLO_FSRCS_PER_SIDE; i++) {
		if (profile.rx_path[0].rx_fsrc[i].enable) rx_enabled++;
		if (profile.tx_path[0].tx_fsrc[i].enable) tx_enabled++;
	}

	printf("Summary:\n");
	printf("  RX FSRC blocks enabled: %d/%d\n", rx_enabled, ADI_APOLLO_FSRCS_PER_SIDE);
	printf("  TX FSRC blocks enabled: %d/%d\n", tx_enabled, ADI_APOLLO_FSRCS_PER_SIDE);

	if (rx_enabled == 0 && tx_enabled == 0) {
		printf("\n*** WARNING: All FSRC blocks are BYPASSED in this profile! ***\n");
		printf("*** FSRC reconfiguration will NOT work unless profile is changed. ***\n");
	} else if (rx_enabled > 0 || tx_enabled > 0) {
		printf("\n*** FSRC blocks are ENABLED - Dynamic reconfiguration will work! ***\n");
	}

	return 0;
}
