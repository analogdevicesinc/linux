#include <test_progs.h>

#include "struct_ops_refcounted.skel.h"
#include "struct_ops_refcounted_subprog.skel.h"
#include "struct_ops_refcounted_fail__ref_leak.skel.h"
#include "struct_ops_refcounted_fail__global_subprog.skel.h"
#include "struct_ops_refcounted_fail__tail_call.skel.h"
#include "struct_ops_refcounted_fail__arena_stack.skel.h"

void test_struct_ops_refcounted(void)
{
	RUN_TESTS(struct_ops_refcounted);
	RUN_TESTS(struct_ops_refcounted_subprog);
	RUN_TESTS(struct_ops_refcounted_fail__ref_leak);
	RUN_TESTS(struct_ops_refcounted_fail__global_subprog);
	RUN_TESTS(struct_ops_refcounted_fail__tail_call);
	RUN_TESTS(struct_ops_refcounted_fail__arena_stack);
}
