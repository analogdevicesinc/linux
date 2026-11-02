// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for cxl_internal_send_cmd() using a stub mailbox
 * transport in place of hardware.
 */
#include <kunit/test.h>
#include <cxl/mailbox.h>
#include "cxlmem.h"

#define TEST_PAYLOAD_SIZE SZ_4K

/*
 * Stub transport state: what the "device" returns, and a record of
 * what the core passed down.
 */
struct mock_mbox {
	struct cxl_mailbox cxl_mbox;
	int send_rc;
	u16 return_code;
	size_t size_out;
	int calls;
	struct cxl_mbox_cmd seen;
};

static int mock_mbox_send(struct cxl_mailbox *cxl_mbox,
			  struct cxl_mbox_cmd *cmd)
{
	struct mock_mbox *mock;

	mock = container_of(cxl_mbox, struct mock_mbox, cxl_mbox);
	mock->calls++;
	mock->seen = *cmd;
	if (mock->send_rc)
		return mock->send_rc;

	cmd->return_code = mock->return_code;
	cmd->size_out = mock->size_out;
	return 0;
}

static int cxl_mbox_test_init(struct kunit *test)
{
	struct mock_mbox *mock;

	mock = kunit_kzalloc(test, sizeof(*mock), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, mock);

	mock->cxl_mbox.payload_size = TEST_PAYLOAD_SIZE;
	mock->cxl_mbox.mbox_send = mock_mbox_send;
	mock->return_code = CXL_MBOX_CMD_RC_SUCCESS;
	test->priv = mock;

	return 0;
}

static void send_success(struct kunit *test)
{
	struct mock_mbox *mock = test->priv;
	u8 in[16], out[64];
	struct cxl_mbox_cmd cmd = {
		.opcode = CXL_MBOX_OP_IDENTIFY,
		.payload_in = in,
		.size_in = sizeof(in),
		.payload_out = out,
		.size_out = sizeof(out),
	};

	mock->size_out = sizeof(out);
	KUNIT_EXPECT_EQ(test, cxl_internal_send_cmd(&mock->cxl_mbox, &cmd), 0);
	KUNIT_EXPECT_EQ(test, mock->calls, 1);

	/* The command reaches the transport as the caller built it */
	KUNIT_EXPECT_EQ(test, mock->seen.opcode, CXL_MBOX_OP_IDENTIFY);
	KUNIT_EXPECT_PTR_EQ(test, mock->seen.payload_in, (void *)in);
	KUNIT_EXPECT_EQ(test, mock->seen.size_in, sizeof(in));
	KUNIT_EXPECT_PTR_EQ(test, mock->seen.payload_out, (void *)out);
	KUNIT_EXPECT_EQ(test, mock->seen.size_out, sizeof(out));
}

static void send_no_output(struct kunit *test)
{
	struct mock_mbox *mock = test->priv;
	struct cxl_mbox_cmd cmd = {};

	/* No output was requested, so the returned size is not checked */
	mock->size_out = 0;
	KUNIT_EXPECT_EQ(test, cxl_internal_send_cmd(&mock->cxl_mbox, &cmd), 0);
}

static void send_size_in_too_big(struct kunit *test)
{
	struct mock_mbox *mock = test->priv;
	struct cxl_mbox_cmd cmd = {
		.size_in = TEST_PAYLOAD_SIZE + 1,
	};

	KUNIT_EXPECT_EQ(test, cxl_internal_send_cmd(&mock->cxl_mbox, &cmd),
			-E2BIG);
	KUNIT_EXPECT_EQ(test, mock->calls, 0);
}

static void send_size_out_too_big(struct kunit *test)
{
	struct mock_mbox *mock = test->priv;
	struct cxl_mbox_cmd cmd = {
		.size_out = TEST_PAYLOAD_SIZE + 1,
	};

	KUNIT_EXPECT_EQ(test, cxl_internal_send_cmd(&mock->cxl_mbox, &cmd),
			-E2BIG);
	KUNIT_EXPECT_EQ(test, mock->calls, 0);
}

static void send_full_payload(struct kunit *test)
{
	struct mock_mbox *mock = test->priv;
	struct cxl_mbox_cmd cmd = {
		.size_in = TEST_PAYLOAD_SIZE,
		.size_out = TEST_PAYLOAD_SIZE,
	};

	mock->size_out = TEST_PAYLOAD_SIZE;
	KUNIT_EXPECT_EQ(test, cxl_internal_send_cmd(&mock->cxl_mbox, &cmd), 0);
}

static void send_transport_error(struct kunit *test)
{
	struct mock_mbox *mock = test->priv;
	struct cxl_mbox_cmd cmd = {};

	mock->send_rc = -ETIMEDOUT;
	KUNIT_EXPECT_EQ(test, cxl_internal_send_cmd(&mock->cxl_mbox, &cmd),
			-ETIMEDOUT);
}

static void send_device_error(struct kunit *test)
{
	struct mock_mbox *mock = test->priv;

	/* Every defined error code fails the command */
	for (u16 code = CXL_MBOX_CMD_RC_INPUT;
	     code < ARRAY_SIZE(cxl_mbox_cmd_rctable); code++) {
		struct cxl_mbox_cmd cmd = {};
		int expect, rc;

		switch (code) {
		case CXL_MBOX_CMD_RC_PADDR:
			expect = -EFAULT;
			break;
		case CXL_MBOX_CMD_RC_POISONLMT:
			expect = -EBUSY;
			break;
		default:
			expect = -ENXIO;
			break;
		}

		mock->return_code = code;
		rc = cxl_internal_send_cmd(&mock->cxl_mbox, &cmd);
		KUNIT_EXPECT_EQ_MSG(test, rc, expect, "return_code=%#x", code);
	}
}

static void send_unknown_return_code(struct kunit *test)
{
	static const u16 codes[] = { ARRAY_SIZE(cxl_mbox_cmd_rctable), 0x100,
				     0xffff };
	struct mock_mbox *mock = test->priv;

	/* Codes the driver does not know are still device errors */
	for (int i = 0; i < ARRAY_SIZE(codes); i++) {
		struct cxl_mbox_cmd cmd = {};
		int rc;

		mock->return_code = codes[i];
		rc = cxl_internal_send_cmd(&mock->cxl_mbox, &cmd);
		KUNIT_EXPECT_EQ_MSG(test, rc, -ENXIO, "return_code=%#x",
				    codes[i]);
		KUNIT_EXPECT_STREQ_MSG(test, cxl_mbox_cmd_rc2str(&cmd),
				       "unknown return code", "return_code=%#x",
				       codes[i]);
	}
}

static void send_background_started(struct kunit *test)
{
	struct mock_mbox *mock = test->priv;
	struct cxl_mbox_cmd cmd = {};

	/* A background command that started is not an error */
	mock->return_code = CXL_MBOX_CMD_RC_BACKGROUND;
	KUNIT_EXPECT_EQ(test, cxl_internal_send_cmd(&mock->cxl_mbox, &cmd), 0);
}

static void send_short_fixed_output(struct kunit *test)
{
	struct mock_mbox *mock = test->priv;
	struct cxl_mbox_cmd cmd = {
		.size_out = 64,
	};

	/* Without min_out, the full requested size is required */
	mock->size_out = 63;
	KUNIT_EXPECT_EQ(test, cxl_internal_send_cmd(&mock->cxl_mbox, &cmd),
			-EIO);
}

static void send_variable_output(struct kunit *test)
{
	struct mock_mbox *mock = test->priv;
	struct cxl_mbox_cmd cmd;

	/* Output at or above min_out is accepted */
	cmd = (struct cxl_mbox_cmd){ .size_out = 64, .min_out = 8 };
	mock->size_out = 8;
	KUNIT_EXPECT_EQ(test, cxl_internal_send_cmd(&mock->cxl_mbox, &cmd), 0);

	/* Output below min_out is rejected */
	cmd = (struct cxl_mbox_cmd){ .size_out = 64, .min_out = 8 };
	mock->size_out = 7;
	KUNIT_EXPECT_EQ(test, cxl_internal_send_cmd(&mock->cxl_mbox, &cmd),
			-EIO);
}

static struct kunit_case cxl_core_mbox_cases[] = {
	KUNIT_CASE(send_success),
	KUNIT_CASE(send_no_output),
	KUNIT_CASE(send_size_in_too_big),
	KUNIT_CASE(send_size_out_too_big),
	KUNIT_CASE(send_full_payload),
	KUNIT_CASE(send_transport_error),
	KUNIT_CASE(send_device_error),
	KUNIT_CASE(send_unknown_return_code),
	KUNIT_CASE(send_background_started),
	KUNIT_CASE(send_short_fixed_output),
	KUNIT_CASE(send_variable_output),
	{}
};

static struct kunit_suite cxl_core_mbox_suite = {
	.name = "cxl_core_mbox",
	.init = cxl_mbox_test_init,
	.test_cases = cxl_core_mbox_cases,
};

kunit_test_suite(cxl_core_mbox_suite);

MODULE_DESCRIPTION("KUnit tests for the CXL mailbox command core");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("CXL");
