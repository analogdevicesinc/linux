// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests for the general purpose allocator in lib/genalloc.c
 */

#include <kunit/device.h>
#include <kunit/test.h>
#include <linux/bitmap.h>
#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/module.h>
#include <linux/numa.h>
#include <linux/string.h>

/* Allocation granule used by most tests: 16 bytes. */
#define GP_ORDER	4
#define GP_UNIT		BIT(GP_ORDER)
#define GP_CHUNK	1024

static void gp_clear_chunk(struct gen_pool *pool, struct gen_pool_chunk *chunk,
			   void *data)
{
	size_t size = chunk->end_addr - chunk->start_addr + 1;

	bitmap_zero(chunk->bits, size >> pool->min_alloc_order);
}

/*
 * gen_pool_destroy() BUGs on outstanding allocations. Clear them first so
 * that a test which fails half way does not take the whole run down.
 */
static void gp_destroy(void *data)
{
	struct gen_pool *pool = data;

	gen_pool_for_each_chunk(pool, gp_clear_chunk, NULL);
	gen_pool_destroy(pool);
}

static struct gen_pool *gp_create(struct kunit *test, int order)
{
	struct gen_pool *pool = gen_pool_create(order, NUMA_NO_NODE);

	KUNIT_ASSERT_NOT_NULL(test, pool);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, gp_destroy, pool),
			0);
	return pool;
}

/* Backing memory for a chunk, aligned to @size (a power of two). */
static unsigned long gp_mem(struct kunit *test, size_t size)
{
	void *mem = kunit_kzalloc(test, size, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, mem);
	return (unsigned long)mem;
}

static void genalloc_size_avail_test(struct kunit *test)
{
	struct gen_pool *pool = gp_create(test, GP_ORDER);
	unsigned long base = gp_mem(test, GP_CHUNK);

	KUNIT_EXPECT_EQ(test, gen_pool_size(pool), 0);
	KUNIT_EXPECT_EQ(test, gen_pool_avail(pool), 0);
	KUNIT_EXPECT_EQ(test, gen_pool_alloc(pool, GP_UNIT), 0);

	KUNIT_ASSERT_EQ(test, gen_pool_add(pool, base, GP_CHUNK, NUMA_NO_NODE),
			0);
	KUNIT_EXPECT_EQ(test, gen_pool_size(pool), GP_CHUNK);
	KUNIT_EXPECT_EQ(test, gen_pool_avail(pool), GP_CHUNK);

	KUNIT_EXPECT_TRUE(test, gen_pool_has_addr(pool, base, GP_CHUNK));
	KUNIT_EXPECT_TRUE(test, gen_pool_has_addr(pool, base + GP_CHUNK - 1, 1));
	KUNIT_EXPECT_FALSE(test, gen_pool_has_addr(pool, base + GP_CHUNK - 4, 8));
	KUNIT_EXPECT_FALSE(test, gen_pool_has_addr(pool, base + GP_CHUNK, 1));
	KUNIT_EXPECT_FALSE(test, gen_pool_has_addr(pool, base - 1, 2));
}

static void genalloc_alloc_free_test(struct kunit *test)
{
	struct gen_pool *pool = gp_create(test, GP_ORDER);
	unsigned long base = gp_mem(test, GP_CHUNK);
	unsigned long a, b, c;

	KUNIT_ASSERT_EQ(test, gen_pool_add(pool, base, GP_CHUNK, NUMA_NO_NODE),
			0);

	/* A zero-sized request fails. */
	KUNIT_EXPECT_EQ(test, gen_pool_alloc(pool, 0), 0);

	a = gen_pool_alloc(pool, GP_UNIT);
	KUNIT_ASSERT_EQ(test, a, base);

	/* Sizes are rounded up to whole granules. */
	b = gen_pool_alloc(pool, 1);
	KUNIT_ASSERT_EQ(test, b, base + GP_UNIT);
	KUNIT_EXPECT_EQ(test, gen_pool_avail(pool), GP_CHUNK - 2 * GP_UNIT);

	c = gen_pool_alloc(pool, GP_UNIT + 1);
	KUNIT_ASSERT_EQ(test, c, base + 2 * GP_UNIT);
	KUNIT_EXPECT_EQ(test, gen_pool_avail(pool), GP_CHUNK - 4 * GP_UNIT);

	/* The pool size never changes. */
	KUNIT_EXPECT_EQ(test, gen_pool_size(pool), GP_CHUNK);

	/* Freeing with the size used to allocate returns the whole granules. */
	gen_pool_free(pool, b, 1);
	gen_pool_free(pool, c, GP_UNIT + 1);
	gen_pool_free(pool, a, GP_UNIT);
	KUNIT_EXPECT_EQ(test, gen_pool_avail(pool), GP_CHUNK);

	/* Too large a request fails; an exact fit succeeds. */
	KUNIT_EXPECT_EQ(test, gen_pool_alloc(pool, GP_CHUNK + 1), 0);
	a = gen_pool_alloc(pool, GP_CHUNK);
	KUNIT_ASSERT_EQ(test, a, base);
	KUNIT_EXPECT_EQ(test, gen_pool_avail(pool), 0);
	KUNIT_EXPECT_EQ(test, gen_pool_alloc(pool, 1), 0);
	gen_pool_free(pool, a, GP_CHUNK);
	KUNIT_EXPECT_EQ(test, gen_pool_avail(pool), GP_CHUNK);
}

static void genalloc_first_fit_test(struct kunit *test)
{
	struct gen_pool *pool = gp_create(test, GP_ORDER);
	unsigned long base = gp_mem(test, GP_CHUNK);
	unsigned long a, b, c, d;

	KUNIT_ASSERT_EQ(test, gen_pool_add(pool, base, GP_CHUNK, NUMA_NO_NODE),
			0);

	a = gen_pool_alloc(pool, 4 * GP_UNIT);
	b = gen_pool_alloc(pool, 4 * GP_UNIT);
	c = gen_pool_alloc(pool, 4 * GP_UNIT);
	KUNIT_ASSERT_EQ(test, a, base);
	KUNIT_ASSERT_EQ(test, b, base + 4 * GP_UNIT);
	KUNIT_ASSERT_EQ(test, c, base + 8 * GP_UNIT);

	/* A hole left by a free is reused by the next request that fits. */
	gen_pool_free(pool, b, 4 * GP_UNIT);
	d = gen_pool_alloc(pool, 2 * GP_UNIT);
	KUNIT_ASSERT_EQ(test, d, b);

	/* A request too large for the hole goes after the last allocation. */
	b = gen_pool_alloc(pool, 3 * GP_UNIT);
	KUNIT_ASSERT_EQ(test, b, base + 12 * GP_UNIT);

	gen_pool_free(pool, a, 4 * GP_UNIT);
	gen_pool_free(pool, b, 3 * GP_UNIT);
	gen_pool_free(pool, c, 4 * GP_UNIT);
	gen_pool_free(pool, d, 2 * GP_UNIT);
	KUNIT_EXPECT_EQ(test, gen_pool_avail(pool), GP_CHUNK);
}

static void genalloc_best_fit_test(struct kunit *test)
{
	struct gen_pool *pool = gp_create(test, GP_ORDER);
	unsigned long base = gp_mem(test, GP_CHUNK);
	unsigned long big, sep, small, rest, addr;
	size_t rest_size = GP_CHUNK - 14 * GP_UNIT;

	KUNIT_ASSERT_EQ(test, gen_pool_add(pool, base, GP_CHUNK, NUMA_NO_NODE),
			0);

	/* Leave an 8 granule hole at the start and a 2 granule hole later. */
	big = gen_pool_alloc(pool, 8 * GP_UNIT);
	sep = gen_pool_alloc(pool, 4 * GP_UNIT);
	small = gen_pool_alloc(pool, 2 * GP_UNIT);
	rest = gen_pool_alloc(pool, rest_size);
	KUNIT_ASSERT_EQ(test, small, base + 12 * GP_UNIT);
	KUNIT_ASSERT_EQ(test, rest, base + 14 * GP_UNIT);
	gen_pool_free(pool, big, 8 * GP_UNIT);
	gen_pool_free(pool, small, 2 * GP_UNIT);

	/* First fit takes the first hole... */
	addr = gen_pool_alloc(pool, 2 * GP_UNIT);
	KUNIT_ASSERT_EQ(test, addr, big);
	gen_pool_free(pool, addr, 2 * GP_UNIT);

	/* ...best fit takes the smallest hole that is large enough. */
	gen_pool_set_algo(pool, gen_pool_best_fit, NULL);
	addr = gen_pool_alloc(pool, 2 * GP_UNIT);
	KUNIT_ASSERT_EQ(test, addr, small);

	/* With that hole used, best fit falls back to the larger one. */
	KUNIT_ASSERT_EQ(test, gen_pool_alloc(pool, GP_UNIT), big);

	gen_pool_free(pool, addr, 2 * GP_UNIT);
	gen_pool_free(pool, big, GP_UNIT);

	/*
	 * Of two equally good holes, best fit takes the first. Free the
	 * separator so granules 0-13 are free, then carve them into two
	 * three granule holes (0-2 and 4-6) with allocations in between.
	 */
	gen_pool_free(pool, sep, 4 * GP_UNIT);
	big = gen_pool_alloc(pool, 3 * GP_UNIT);
	sep = gen_pool_alloc(pool, GP_UNIT);
	small = gen_pool_alloc(pool, 3 * GP_UNIT);
	addr = gen_pool_alloc(pool, 7 * GP_UNIT);
	KUNIT_ASSERT_EQ(test, big, base);
	KUNIT_ASSERT_EQ(test, sep, base + 3 * GP_UNIT);
	KUNIT_ASSERT_EQ(test, small, base + 4 * GP_UNIT);
	KUNIT_ASSERT_EQ(test, addr, base + 7 * GP_UNIT);
	gen_pool_free(pool, big, 3 * GP_UNIT);
	gen_pool_free(pool, small, 3 * GP_UNIT);

	small = gen_pool_alloc(pool, 2 * GP_UNIT);
	KUNIT_ASSERT_EQ(test, small, base);

	gen_pool_free(pool, small, 2 * GP_UNIT);
	gen_pool_free(pool, sep, GP_UNIT);
	gen_pool_free(pool, addr, 7 * GP_UNIT);
	gen_pool_free(pool, rest, rest_size);
	KUNIT_EXPECT_EQ(test, gen_pool_avail(pool), GP_CHUNK);

	/* Setting a NULL algorithm restores first fit. */
	gen_pool_set_algo(pool, NULL, NULL);
	KUNIT_EXPECT_TRUE(test, pool->algo == gen_pool_first_fit);
}

static void genalloc_first_fit_align_test(struct kunit *test)
{
	struct gen_pool *pool = gp_create(test, GP_ORDER);
	unsigned long mem = gp_mem(test, 2 * GP_CHUNK);
	/* A chunk that starts one granule past a 256 byte boundary. */
	unsigned long base = mem + GP_UNIT;
	struct genpool_data_align align = { .align = 256 };
	unsigned long a, b;

	KUNIT_ASSERT_EQ(test, mem & 255, 0);
	KUNIT_ASSERT_EQ(test, gen_pool_add(pool, base, GP_CHUNK, NUMA_NO_NODE),
			0);

	/* Alignment is of the address, not of the offset in the chunk. */
	a = gen_pool_alloc_algo(pool, GP_UNIT, gen_pool_first_fit_align,
				&align);
	KUNIT_ASSERT_EQ(test, a, mem + 256);

	b = gen_pool_alloc_algo(pool, GP_UNIT, gen_pool_first_fit_align,
				&align);
	KUNIT_ASSERT_EQ(test, b, mem + 512);

	/* The space skipped for alignment is still free for plain requests. */
	KUNIT_ASSERT_EQ(test, gen_pool_alloc(pool, GP_UNIT), base);

	gen_pool_free(pool, a, GP_UNIT);
	gen_pool_free(pool, b, GP_UNIT);
	gen_pool_free(pool, base, GP_UNIT);
}

static void genalloc_order_align_test(struct kunit *test)
{
	struct gen_pool *pool = gp_create(test, GP_ORDER);
	unsigned long base = gp_mem(test, GP_CHUNK);
	unsigned long a, b, c;

	KUNIT_ASSERT_EQ(test, gen_pool_add(pool, base, GP_CHUNK, NUMA_NO_NODE),
			0);
	gen_pool_set_algo(pool, gen_pool_first_fit_order_align, NULL);

	a = gen_pool_alloc(pool, GP_UNIT);
	KUNIT_ASSERT_EQ(test, a, base);

	/* Three granules round up to an alignment of four granules. */
	b = gen_pool_alloc(pool, 3 * GP_UNIT);
	KUNIT_ASSERT_EQ(test, b, base + 4 * GP_UNIT);

	/* A single granule needs no alignment and fills the first gap. */
	c = gen_pool_alloc(pool, GP_UNIT);
	KUNIT_ASSERT_EQ(test, c, base + GP_UNIT);

	gen_pool_free(pool, a, GP_UNIT);
	gen_pool_free(pool, b, 3 * GP_UNIT);
	gen_pool_free(pool, c, GP_UNIT);
}

static void genalloc_fixed_alloc_test(struct kunit *test)
{
	struct gen_pool *pool = gp_create(test, GP_ORDER);
	unsigned long base = gp_mem(test, GP_CHUNK);
	struct genpool_data_fixed fixed = { .offset = 8 * GP_UNIT };
	unsigned long a;

	KUNIT_ASSERT_EQ(test, gen_pool_add(pool, base, GP_CHUNK, NUMA_NO_NODE),
			0);

	a = gen_pool_alloc_algo(pool, 2 * GP_UNIT, gen_pool_fixed_alloc, &fixed);
	KUNIT_ASSERT_EQ(test, a, base + 8 * GP_UNIT);

	/* The same offset cannot be handed out twice... */
	KUNIT_EXPECT_EQ(test, gen_pool_alloc_algo(pool, GP_UNIT,
						  gen_pool_fixed_alloc, &fixed),
			0);

	/* ...nor an offset whose range overlaps it. */
	fixed.offset = 7 * GP_UNIT;
	KUNIT_EXPECT_EQ(test, gen_pool_alloc_algo(pool, 2 * GP_UNIT,
						  gen_pool_fixed_alloc, &fixed),
			0);

	/* An offset just before it, that fits, works. */
	KUNIT_ASSERT_EQ(test, gen_pool_alloc_algo(pool, GP_UNIT,
						  gen_pool_fixed_alloc, &fixed),
			base + 7 * GP_UNIT);

	gen_pool_free(pool, a, 2 * GP_UNIT);
	gen_pool_free(pool, base + 7 * GP_UNIT, GP_UNIT);
}

static void genalloc_chunks_test(struct kunit *test)
{
	struct gen_pool *pool = gp_create(test, GP_ORDER);
	unsigned long m1 = gp_mem(test, GP_CHUNK);
	unsigned long m2 = gp_mem(test, GP_CHUNK / 4);
	const phys_addr_t p1 = 0x100000, p2 = 0x200000;
	unsigned long a;

	KUNIT_ASSERT_EQ(test, gen_pool_add_virt(pool, m1, p1, GP_CHUNK,
						NUMA_NO_NODE), 0);
	KUNIT_ASSERT_EQ(test, gen_pool_add_virt(pool, m2, p2, GP_CHUNK / 4,
						NUMA_NO_NODE), 0);

	KUNIT_EXPECT_EQ(test, gen_pool_size(pool), GP_CHUNK + GP_CHUNK / 4);
	KUNIT_EXPECT_EQ(test, gen_pool_avail(pool), GP_CHUNK + GP_CHUNK / 4);

	KUNIT_EXPECT_EQ(test, gen_pool_virt_to_phys(pool, m1), p1);
	KUNIT_EXPECT_EQ(test, gen_pool_virt_to_phys(pool, m1 + 100), p1 + 100);
	KUNIT_EXPECT_EQ(test, gen_pool_virt_to_phys(pool, m2 + GP_CHUNK / 4 - 1),
			p2 + GP_CHUNK / 4 - 1);
	KUNIT_EXPECT_EQ(test, gen_pool_virt_to_phys(pool, m2 + GP_CHUNK / 4),
			(phys_addr_t)-1);

	/* A request that only fits in the larger chunk is served from it. */
	a = gen_pool_alloc(pool, GP_CHUNK / 2);
	KUNIT_ASSERT_EQ(test, a, m1);
	KUNIT_EXPECT_EQ(test, gen_pool_avail(pool), GP_CHUNK / 2 + GP_CHUNK / 4);

	KUNIT_EXPECT_TRUE(test, gen_pool_has_addr(pool, m2, GP_CHUNK / 4));
	/* A range spanning two chunks is not in the pool. */
	KUNIT_EXPECT_FALSE(test, gen_pool_has_addr(pool, m1, 2 * GP_CHUNK));

	gen_pool_free(pool, a, GP_CHUNK / 2);
	KUNIT_EXPECT_EQ(test, gen_pool_avail(pool), GP_CHUNK + GP_CHUNK / 4);
}

static void genalloc_count_chunk(struct gen_pool *pool,
				 struct gen_pool_chunk *chunk, void *data)
{
	int *count = data;

	(*count)++;
}

static void genalloc_chunk_walk_test(struct kunit *test)
{
	struct gen_pool *pool = gp_create(test, GP_ORDER);
	int i, count = 0;

	for (i = 0; i < 3; i++)
		KUNIT_ASSERT_EQ(test, gen_pool_add(pool, gp_mem(test, GP_CHUNK),
						   GP_CHUNK, NUMA_NO_NODE), 0);

	gen_pool_for_each_chunk(pool, genalloc_count_chunk, &count);
	KUNIT_EXPECT_EQ(test, count, 3);
}

static void genalloc_owner_test(struct kunit *test)
{
	struct gen_pool *pool = gp_create(test, GP_ORDER);
	unsigned long base = gp_mem(test, GP_CHUNK);
	int token;
	void *owner = NULL;
	unsigned long a;

	KUNIT_ASSERT_EQ(test, gen_pool_add_owner(pool, base, -1, GP_CHUNK,
						 NUMA_NO_NODE, &token), 0);

	a = gen_pool_alloc_owner(pool, GP_UNIT, &owner);
	KUNIT_ASSERT_EQ(test, a, base);
	KUNIT_EXPECT_PTR_EQ(test, owner, (void *)&token);

	owner = NULL;
	gen_pool_free_owner(pool, a, GP_UNIT, &owner);
	KUNIT_EXPECT_PTR_EQ(test, owner, (void *)&token);

	/* A failed allocation reports no owner. */
	owner = &owner;
	KUNIT_EXPECT_EQ(test, gen_pool_alloc_owner(pool, 2 * GP_CHUNK, &owner),
			0);
	KUNIT_EXPECT_NULL(test, owner);
}

static void genalloc_dma_test(struct kunit *test)
{
	struct gen_pool *pool = gp_create(test, GP_ORDER);
	unsigned long base = gp_mem(test, GP_CHUNK);
	const phys_addr_t phys = 0x40000000;
	dma_addr_t dma = 0;
	u8 *a, *b;
	int i;

	memset((void *)base, 0xa5, GP_CHUNK);
	KUNIT_ASSERT_EQ(test, gen_pool_add_virt(pool, base, phys, GP_CHUNK,
						NUMA_NO_NODE), 0);

	a = gen_pool_dma_alloc(pool, 2 * GP_UNIT, &dma);
	KUNIT_ASSERT_PTR_EQ(test, a, (u8 *)base);
	KUNIT_EXPECT_EQ(test, dma, (dma_addr_t)phys);
	/* gen_pool_dma_alloc() does not clear the memory. */
	KUNIT_EXPECT_EQ(test, a[0], 0xa5);

	b = gen_pool_dma_zalloc(pool, 2 * GP_UNIT, &dma);
	KUNIT_ASSERT_PTR_EQ(test, b, (u8 *)base + 2 * GP_UNIT);
	KUNIT_EXPECT_EQ(test, dma, (dma_addr_t)(phys + 2 * GP_UNIT));
	for (i = 0; i < 2 * GP_UNIT; i++)
		KUNIT_EXPECT_EQ(test, b[i], 0);
	/* Only the requested memory is cleared. */
	KUNIT_EXPECT_EQ(test, b[2 * GP_UNIT], 0xa5);

	gen_pool_free(pool, (unsigned long)a, 2 * GP_UNIT);
	gen_pool_free(pool, (unsigned long)b, 2 * GP_UNIT);

	/* With the start in use, the aligned variant skips to 128 bytes. */
	a = gen_pool_dma_alloc(pool, GP_UNIT, NULL);
	KUNIT_ASSERT_PTR_EQ(test, a, (u8 *)base);
	b = gen_pool_dma_alloc_align(pool, GP_UNIT, &dma, 128);
	KUNIT_ASSERT_PTR_EQ(test, b, (u8 *)base + 128);
	KUNIT_EXPECT_EQ(test, dma, (dma_addr_t)(phys + 128));
	gen_pool_free(pool, (unsigned long)a, GP_UNIT);
	gen_pool_free(pool, (unsigned long)b, GP_UNIT);

	/* A NULL @dma is allowed, and so is a NULL pool. */
	a = gen_pool_dma_alloc(pool, GP_UNIT, NULL);
	KUNIT_ASSERT_PTR_EQ(test, a, (u8 *)base);
	gen_pool_free(pool, (unsigned long)a, GP_UNIT);
	KUNIT_EXPECT_NULL(test, gen_pool_dma_alloc(NULL, GP_UNIT, &dma));
	KUNIT_EXPECT_NULL(test, gen_pool_dma_zalloc(NULL, GP_UNIT, &dma));
	KUNIT_EXPECT_NULL(test, gen_pool_dma_alloc_align(NULL, GP_UNIT, &dma,
							 128));

	/* Exhaustion returns NULL. */
	KUNIT_EXPECT_NULL(test, gen_pool_dma_zalloc(pool, 2 * GP_CHUNK, &dma));
}

static void genalloc_devm_test(struct kunit *test)
{
	struct gen_pool *named, *unnamed;
	struct device *dev;

	dev = kunit_device_register(test, "genalloc-kunit");
	KUNIT_ASSERT_FALSE(test, IS_ERR(dev));

	KUNIT_EXPECT_NULL(test, gen_pool_get(dev, NULL));
	KUNIT_EXPECT_NULL(test, gen_pool_get(dev, "sram"));

	named = devm_gen_pool_create(dev, GP_ORDER, NUMA_NO_NODE, "sram");
	KUNIT_ASSERT_FALSE(test, IS_ERR(named));
	KUNIT_EXPECT_STREQ(test, named->name, "sram");
	KUNIT_EXPECT_EQ(test, named->min_alloc_order, GP_ORDER);
	KUNIT_EXPECT_PTR_EQ(test, gen_pool_get(dev, "sram"), named);

	/* An unnamed lookup does not match a named pool, nor the reverse. */
	KUNIT_EXPECT_NULL(test, gen_pool_get(dev, NULL));
	KUNIT_EXPECT_NULL(test, gen_pool_get(dev, "other"));

	unnamed = devm_gen_pool_create(dev, GP_ORDER, NUMA_NO_NODE, NULL);
	KUNIT_ASSERT_FALSE(test, IS_ERR(unnamed));
	KUNIT_EXPECT_PTR_EQ(test, gen_pool_get(dev, NULL), unnamed);
	KUNIT_EXPECT_PTR_EQ(test, gen_pool_get(dev, "sram"), named);

	/* Each name, including no name, can only be used once per device. */
	KUNIT_EXPECT_EQ(test, PTR_ERR(devm_gen_pool_create(dev, GP_ORDER,
							   NUMA_NO_NODE,
							   "sram")),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, PTR_ERR(devm_gen_pool_create(dev, GP_ORDER,
							   NUMA_NO_NODE, NULL)),
			-EINVAL);

	/* Both pools are destroyed when the device goes away. */
}

static struct kunit_case genalloc_test_cases[] = {
	KUNIT_CASE(genalloc_size_avail_test),
	KUNIT_CASE(genalloc_alloc_free_test),
	KUNIT_CASE(genalloc_first_fit_test),
	KUNIT_CASE(genalloc_best_fit_test),
	KUNIT_CASE(genalloc_first_fit_align_test),
	KUNIT_CASE(genalloc_order_align_test),
	KUNIT_CASE(genalloc_fixed_alloc_test),
	KUNIT_CASE(genalloc_chunks_test),
	KUNIT_CASE(genalloc_chunk_walk_test),
	KUNIT_CASE(genalloc_owner_test),
	KUNIT_CASE(genalloc_dma_test),
	KUNIT_CASE(genalloc_devm_test),
	{}
};

static struct kunit_suite genalloc_test_suite = {
	.name = "genalloc",
	.test_cases = genalloc_test_cases,
};

kunit_test_suite(genalloc_test_suite);

MODULE_DESCRIPTION("KUnit tests for the general purpose allocator");
MODULE_LICENSE("GPL");
