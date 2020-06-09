#include <linux/math64.h>

#include "utils.h"

#define LOWER_16(A) ((A) & 0xFFFF)
#define UPPER_16(A) (((A) >> 16) & 0xFFFF)
#define LOWER_32(A) ((A) & (uint32_t) 0xFFFFFFFF)

int64_t adi_api_utils_gcd(int64_t u, int64_t v)
{
	int64_t t;
	while (v != 0) {
		t = u;
		u = v;
		div64_u64_rem(t, v, &v);
	}
	return u < 0 ? -u : u; /* abs(u) */
}

void adi_api_utils_mult_64(uint32_t a, uint32_t b, uint32_t *hi, uint32_t *lo)
{
	uint32_t    ah = (a >> 16), al = a & 0xffff,
				bh = (b >> 16), bl = b & 0xffff,
				rh = (ah * bh), rl = (al * bl),
				rm1  = ah * bl, rm2  = al * bh,
				rm1h = rm1 >> 16, rm2h = rm2 >> 16,
				rm1l = rm1 & 0xffff, rm2l = rm2 & 0xffff,
				rmh  = rm1h + rm2h, rml  = rm1l + rm2l,
				c = ((rl >> 16) + rml) >> 16;

	rl = rl + (rml << 16);
	rh = rh + rmh + c;
	*lo = rl;
	*hi = rh;
}

void adi_api_utils_lshift_128(uint64_t *hi, uint64_t *lo)
{
	*hi <<= 1;
	if (*lo & U64MSB)
	{
		*hi |= 1ul;
	}
	*lo <<= 1;
}

void adi_api_utils_rshift_128(uint64_t *hi, uint64_t *lo)
{
	*lo >>= 1;
	if (*hi & 1u) {
		*lo |= U64MSB;
	}
	*hi >>= 1;
}

void adi_api_utils_mult_128(uint64_t a, uint64_t b, uint64_t *hi, uint64_t *lo)
{
	uint64_t    ah = (a >> 32), al = a & 0xffffffff,
				bh = (b >> 32), bl = b & 0xffffffff,
				rh = (ah * bh), rl = (al * bl),
				rm1  = ah * bl, rm2  = al * bh,
				rm1h = rm1 >> 32, rm2h = rm2 >> 32,
				rm1l = rm1 & 0xffffffff, rm2l = rm2 & 0xffffffff,
				rmh  = rm1h + rm2h, rml  = rm1l + rm2l,
				c = ((rl >> 32) + rml) >> 32;

	rl = rl + (rml << 32);
	rh = rh + rmh + c;
	*lo = rl;
	*hi = rh;
}

void adi_api_utils_div_128(uint64_t a_hi, uint64_t a_lo,
							uint64_t b_hi, uint64_t b_lo,
							uint64_t *hi, uint64_t *lo)
{
	uint64_t remain_lo  = a_lo; /* The left-hand side of division, i.e. what is being divided */
	uint64_t remain_hi  = a_hi; /* The left-hand side of division, i.e. what is being divided */
	uint64_t part1_lo   = b_lo; /* The right-hand side of division */
	uint64_t part1_hi   = b_hi; /* The right-hand side of division */
	uint64_t result_lo  = 0;
	uint64_t result_hi  = 0;
	uint64_t mask_lo    = 1;
	uint64_t mask_hi    = 0;
	
	if ((part1_lo == 0) && (part1_hi == 0)) {
		/* Do whatever should happen when dividing by zero. */
		return;
	}

	/* while(part1_lo < remain_lo)
	 * Alternative: while(!(part1 & 0x8000)) - For 16-bit, test highest order bit.
	 * Alternative: while(not_signed(part1)) - Same as above: As long as sign bit is not set in part1. */
	while (!(part1_hi & U64MSB)) {
		adi_api_utils_lshift_128(&part1_hi, &part1_lo);
		adi_api_utils_lshift_128(&mask_hi, &mask_lo);
	}

	do {
		if ((remain_hi > part1_hi) || ((remain_hi == part1_hi) && (remain_lo >= part1_lo))) {
			/* remain_lo = remain_lo - part1_lo; */
			adi_api_utils_subt_128(remain_hi, remain_lo, part1_hi, part1_lo, &remain_hi, &remain_lo);
			/* result = result + mask; */
			adi_api_utils_add_128(result_hi, result_lo, mask_hi, mask_lo, &result_hi, &result_lo);
		}
		/* part1 = part1 >> 1; */
		adi_api_utils_rshift_128(&part1_hi, &part1_lo);
		/* mask  = mask  >> 1; */
		adi_api_utils_rshift_128(&mask_hi, &mask_lo);
	} while ((mask_hi != 0) || (mask_lo != 0));
	
	/* Now: result = division result (quotient)
	 *      remain_lo = division remain_loder (modulo) */
	*lo = result_lo;
	*hi = result_hi;
}

void adi_api_utils_add_128(uint64_t ah, uint64_t al,
							uint64_t bh, uint64_t bl,
							uint64_t *hi, uint64_t *lo)
{
	/* r = a - b*/
	uint64_t rl, rh;
	rl = al + bl;
	rh = ah + bh;

	if (rl < al)
	{
		rh++;
	}

	*lo = rl;
	*hi = rh;
}

void adi_api_utils_subt_128(uint64_t ah, uint64_t al, 
							uint64_t bh, uint64_t bl, 
							uint64_t *hi, uint64_t *lo)
{
	/* r = a - b*/
	uint64_t rl, rh;
	if (bl <= al) {
		rl = al - bl;
		rh = ah - bh;
	} else {
		rl = bl - al - 1;
		rl = 0xFFFFFFFFFFFFFFFFll - rl;
		ah--;
		rh = ah - bh;
	}

	*lo = rl;
	*hi = rh;
}

int is_power_of_two(uint64_t x)
{
	return ((x != 0) && !(x & (x - 1)));
}
