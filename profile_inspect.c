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

static void print_fsrc(const char *dir, int side, const adi_apollo_fsrc_cfg_t *fsrc)
{
	printf("%s FSRC Configuration (Side %c):\n", dir, 'A' + side);
	printf("  Enable0: %d\n", fsrc->enable0);
	printf("  Enable1: %d\n", fsrc->enable1);
	printf("  Bypass: %s\n", (fsrc->enable0 || fsrc->enable1) ? "NO" : "YES");
	printf("  Mode 1x: %d\n", fsrc->mode_1x);
	printf("  Rate Int: 0x%012llx\n", (unsigned long long)fsrc->fsrc_rate_int);
	printf("  Rate Frac A: 0x%012llx\n", (unsigned long long)fsrc->fsrc_rate_frac_a);
	printf("  Rate Frac B: 0x%012llx\n", (unsigned long long)fsrc->fsrc_rate_frac_b);
	printf("  Sample Delay: %u\n", fsrc->fsrc_delay);
	printf("  Gain Reduction: %u\n", fsrc->gain_reduction);
	printf("  Data Mult Dither Enable: %d\n", fsrc->data_mult_dither_en);
	printf("  Dither Enable: %d\n", fsrc->dither_en);
	printf("  Split 4T4R: %d\n", fsrc->split_4t4r);
	printf("\n");
}

int main(int argc, char *argv[])
{
	FILE *fp;
	adi_apollo_top_t profile;
	size_t bytes_read;
	int rx_enabled = 0, tx_enabled = 0;
	int i;
	const char *profile_file = "firmware/204C_L4_M4_NP16_20p0_4x4_fsrc_5_4.bin";

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

	/* Display FSRC configuration, one entry per side, FSRC0/FSRC1 enables */
	for (i = 0; i < ADI_APOLLO_NUM_SIDES; i++) {
		print_fsrc("RX", i, &profile.rx_path[i].rx_fsrc);
		print_fsrc("TX", i, &profile.tx_path[i].tx_fsrc);
		rx_enabled += profile.rx_path[i].rx_fsrc.enable0 + profile.rx_path[i].rx_fsrc.enable1;
		tx_enabled += profile.tx_path[i].tx_fsrc.enable0 + profile.tx_path[i].tx_fsrc.enable1;
	}

	printf("Summary:\n");
	printf("  RX FSRC blocks enabled: %d/%d\n", rx_enabled, 2 * ADI_APOLLO_NUM_SIDES);
	printf("  TX FSRC blocks enabled: %d/%d\n", tx_enabled, 2 * ADI_APOLLO_NUM_SIDES);

	if (rx_enabled == 0 && tx_enabled == 0) {
		printf("\n*** WARNING: All FSRC blocks are BYPASSED in this profile! ***\n");
		printf("*** FSRC reconfiguration will NOT work unless profile is changed. ***\n");
	} else {
		printf("\n*** FSRC blocks are ENABLED - Dynamic reconfiguration will work! ***\n");
	}

	return 0;
}
