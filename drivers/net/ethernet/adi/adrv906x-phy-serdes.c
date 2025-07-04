// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/delay.h>
#include <net/netlink.h>
#include <net/genetlink.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/kfifo.h>
#include <linux/atomic.h>
#include "adrv906x-phy-serdes.h"
#include "adrv906x-cmn.h"

#define ADRV906X_GENL_NAME          "adrv906x"
#define ADRV906X_GENL_VERSION       1
#define ADRV906X_GENL_MC_GRP_NAME   "adrv906x_mcgrp"
#define ADRV906X_PHY_MAX_PLLS       2
#define ADRV906X_PHY_MAX_LANES      4
#define ADRV906X_PHY_DEV_IDX_MSK    GENMASK(3, 0)
#define ADRV906X_PHY_DEV_SPEED_MSK  GENMASK(31, 16)

#define APP_HEARTBEAT_TIMEOUT_MS    1500

typedef void (*adrv906x_phy_fsm_action)(void *param);
typedef char * (*adrv906x_phy_fsm_state_to_str)(u32 state);
typedef char * (*adrv906x_phy_fsm_event_to_str)(u32 event);

enum adrv906x_nl_attrs {
	ATTR_UNSPEC,
	ATTR_CMD_PAYLOAD, /* u32, link speed (bits 31:16), dev_id (bits 3:0) */
	__ATTR_MAX,
	NUM_ATTR	= __ATTR_MAX,
	ATTR_MAX	= __ATTR_MAX - 1,
};

enum adrv906x_nl_commands {
	NL_CMD_PLL_CFG_REQ,
	NL_CMD_PLL_CFG_DONE,
	NL_CMD_PLL_RELOCK_SUCCEED,
	NL_CMD_PLL_RELOCK_FAILED,
	NL_CMD_SER_CFG_REQ,
	NL_CMD_SER_CFG_DONE,
	NL_CMD_DESER_CFG_REQ,
	NL_CMD_DESER_CFG_DONE,
	NL_CMD_DESER_INIT_CAL_REQ,
	NL_CMD_DESER_SIGNAL_OK,
	NL_CMD_DESER_LOS,
	NL_CMD_SERDES_PWR_DOWN_REQ,
	NL_CMD_SERDES_PWR_DOWN_RDY,
	NL_CMD_SERDES_APP_HEARTBEAT,
};

enum adrv906x_serdes_states {
	SD_ST_IDLE,
	SD_ST_LNK_UP_PEND,
	SD_ST_APP_ACTV,
	SD_ST_PLL_UNLOCKED,
	SD_ST_PLL_CFG,
	SD_ST_RATE_CHANGED,
	SD_ST_SER_CFG,
	SD_ST_DESER_CFG,
	SD_ST_CAL_STARTED,
	SD_ST_LOS,
	SD_ST_SIGNAL_OK,
	SD_ST_PWR_DOWN,
};

enum adrv906x_serdes_events {
	SD_EVT_UNKNOWN,
	SD_EVT_APP_ACTV,
	SD_EVT_APP_INACT,
	SD_EVT_LNK_UP,
	SD_EVT_PLL_LOCKED,
	SD_EVT_PLL_UNLOCKED,
	SD_EVT_SER_RDY,
	SD_EVT_DESER_RDY,
	SD_EVT_LNK_DOWN,
	SD_EVT_PWR_DOWN_DONE,
	SD_EVT_SIGNAL_OK,
	SD_EVT_LOS_DETECTED,
};

enum adrv906x_pll_states {
	PLL_ST_UNLOCKED,
	PLL_ST_10G_RUN,
	PLL_ST_25G_RUN,
	PLL_ST_LNK0_10G_REQ,
	PLL_ST_LNK1_10G_REQ,
	PLL_ST_LNK0_25G_REQ,
	PLL_ST_LNK1_25G_REQ,
	PLL_ST_LNK0_10G_PEND,
	PLL_ST_LNK1_10G_PEND,
	PLL_ST_LNK01_10G_PEND,
	PLL_ST_LNK0_25G_PEND,
	PLL_ST_LNK1_25G_PEND,
	PLL_ST_LNK01_25G_PEND,
};

enum adrv906x_pll_events {
	PLL_EVT_UNKNOWN,
	PLL_EVT_UNLOCKED,
	PLL_EVT_APP_ACTV,
	PLL_EVT_APP_INACT,
	PLL_EVT_LNK0_10G_REQ,
	PLL_EVT_LNK1_10G_REQ,
	PLL_EVT_LNK0_25G_REQ,
	PLL_EVT_LNK1_25G_REQ,
	PLL_EVT_CFG_DONE,
	PLL_EVT_LNK0_DOWN,
	PLL_EVT_LNK1_DOWN,
};

struct adrv906x_phy_fsm {
	char name[16];
	atomic_t state;
	struct task_struct *task;
	struct adrv906x_phy_fsm_tran *tran_tbl;
	int tran_tbl_size;
	struct completion comp_tran;
	struct kfifo event_fifo;
	spinlock_t event_fifo_lock;
	adrv906x_phy_fsm_state_to_str state_to_str;
	adrv906x_phy_fsm_event_to_str event_to_str;
};

struct adrv906x_phy_fsm_tran {
	int src_state;
	int event;
	adrv906x_phy_fsm_action action;
	int dst_state;
};

struct adrv906x_serdes {
	struct phy_device *phydev;
	adrv906x_serdes_cb tx_path_en;
	adrv906x_serdes_cb rx_path_en;
	struct adrv906x_phy_fsm fsm;
	int dev_id;
	int speed;
};

struct adrv906x_pll {
	struct adrv906x_phy_fsm fsm;
	struct mutex mtx; /* protects struct access */
	int dev_id;
	bool started;
};

static int __pll_cfg_done_recv(struct sk_buff *skb, struct genl_info *info);
static int __pll_relock_succeed_recv(struct sk_buff *skb, struct genl_info *info);
static int __pll_relock_failed_recv(struct sk_buff *skb, struct genl_info *info);
static int __sd_ser_cfg_done_recv(struct sk_buff *skb, struct genl_info *info);
static int __sd_deser_los_detected_recv(struct sk_buff *skb, struct genl_info *info);
static int __sd_deser_cfg_done_recv(struct sk_buff *skb, struct genl_info *info);
static int __sd_deser_signal_ok_recv(struct sk_buff *skb, struct genl_info *info);
static int __sd_app_pwr_down_rdy_recv(struct sk_buff *skb, struct genl_info *info);
static int __sd_app_heartbeat_recv(struct sk_buff *skb, struct genl_info *info);
static void __sd_pwr_down(void *param);
static void __sd_cfg_pll_req(void *param);
static void __sd_lnk_down_notif(void *param);
static void __sd_pwr_down_notif_send(void *param);
static void __sd_ser_cfg_send(void *param);
static void __sd_deser_cfg_send(void *param);
static void __sd_deser_init_cal_send(void *param);
static void __pll_unlkd_notif_lnk01(void *param);
static void __pll_lkd_notif_lnk0(void *param);
static void __pll_lkd_notif_lnk1(void *param);
static void __pll_lkd_notif_lnk01(void *param);
static void __pll_check_actv_lnks(void *param);
static void __pll_cfg_10G_send(void *param);
static void __pll_cfg_25G_send(void *param);
static void __do_nothing(void *param);
static void __sd_app_watchdog_expired(struct work_struct *work);

static DEFINE_MUTEX(genl_mutex);

static struct adrv906x_phy_fsm_tran adrv906x_phy_fsm_serdes_trans[] = {
	/* Source State       Event                 Action        	  	Destination State */
	{ SD_ST_IDLE,	      SD_EVT_APP_ACTV,	    __do_nothing,	      SD_ST_APP_ACTV	 },
	{ SD_ST_IDLE,	      SD_EVT_LNK_UP,	    __do_nothing,	      SD_ST_LNK_UP_PEND	 },

	{ SD_ST_LNK_UP_PEND,  SD_EVT_APP_ACTV,	    __sd_cfg_pll_req,	      SD_ST_PLL_CFG	 },
	{ SD_ST_LNK_UP_PEND,  SD_EVT_LNK_DOWN,	    __do_nothing,	      SD_ST_IDLE	 },

	{ SD_ST_APP_ACTV,     SD_EVT_APP_INACT,	    __do_nothing,	      SD_ST_IDLE	 },
	{ SD_ST_APP_ACTV,     SD_EVT_LNK_UP,	    __sd_cfg_pll_req,	      SD_ST_PLL_CFG	 },

	{ SD_ST_PLL_UNLOCKED, SD_EVT_PWR_DOWN_DONE, __sd_cfg_pll_req,	      SD_ST_PLL_CFG	 },
	{ SD_ST_PLL_UNLOCKED, SD_EVT_APP_INACT,	    __do_nothing,	      SD_ST_LNK_UP_PEND	 },

	{ SD_ST_PLL_CFG,      SD_EVT_PLL_LOCKED,    __sd_ser_cfg_send,	      SD_ST_SER_CFG	 },
	{ SD_ST_PLL_CFG,      SD_EVT_PLL_UNLOCKED,  __sd_cfg_pll_req,	      SD_ST_PLL_CFG	 },
	{ SD_ST_PLL_CFG,      SD_EVT_LNK_DOWN,	    __sd_lnk_down_notif,      SD_ST_APP_ACTV	 },
	{ SD_ST_PLL_CFG,      SD_EVT_APP_INACT,	    __do_nothing,	      SD_ST_LNK_UP_PEND	 },
	{ SD_ST_PLL_CFG,      SD_EVT_LNK_UP,	    __sd_cfg_pll_req,	      SD_ST_PLL_CFG	 },
	{ SD_ST_PLL_CFG,      SD_EVT_PWR_DOWN_DONE, __sd_ser_cfg_send,	      SD_ST_SER_CFG	 },

	{ SD_ST_SER_CFG,      SD_EVT_SER_RDY,	    __sd_deser_cfg_send,      SD_ST_DESER_CFG	 },
	{ SD_ST_SER_CFG,      SD_EVT_APP_INACT,	    __sd_pwr_down,	      SD_ST_LNK_UP_PEND	 },
	{ SD_ST_SER_CFG,      SD_EVT_LNK_DOWN,	    __sd_pwr_down_notif_send, SD_ST_PWR_DOWN	 },
	{ SD_ST_SER_CFG,      SD_EVT_PLL_LOCKED,    __sd_pwr_down_notif_send, SD_ST_PLL_CFG	 },
	{ SD_ST_SER_CFG,      SD_EVT_PLL_UNLOCKED,  __sd_pwr_down_notif_send, SD_ST_PLL_UNLOCKED },
	{ SD_ST_SER_CFG,      SD_EVT_LNK_UP,	    __sd_pwr_down_notif_send, SD_ST_RATE_CHANGED },

	{ SD_ST_DESER_CFG,    SD_EVT_DESER_RDY,	    __sd_deser_init_cal_send, SD_ST_CAL_STARTED	 },
	{ SD_ST_DESER_CFG,    SD_EVT_APP_INACT,	    __sd_pwr_down,	      SD_ST_LNK_UP_PEND	 },
	{ SD_ST_DESER_CFG,    SD_EVT_LNK_DOWN,	    __sd_pwr_down_notif_send, SD_ST_PWR_DOWN	 },
	{ SD_ST_DESER_CFG,    SD_EVT_PLL_LOCKED,    __sd_pwr_down_notif_send, SD_ST_PLL_CFG	 },
	{ SD_ST_DESER_CFG,    SD_EVT_PLL_UNLOCKED,  __sd_pwr_down_notif_send, SD_ST_PLL_UNLOCKED },
	{ SD_ST_DESER_CFG,    SD_EVT_LNK_UP,	    __sd_pwr_down_notif_send, SD_ST_RATE_CHANGED },

	{ SD_ST_CAL_STARTED,  SD_EVT_SIGNAL_OK,	    __do_nothing,	      SD_ST_SIGNAL_OK	 },
	{ SD_ST_CAL_STARTED,  SD_EVT_APP_INACT,	    __sd_pwr_down,	      SD_ST_LNK_UP_PEND	 },
	{ SD_ST_CAL_STARTED,  SD_EVT_LNK_DOWN,	    __sd_pwr_down_notif_send, SD_ST_PWR_DOWN	 },
	{ SD_ST_CAL_STARTED,  SD_EVT_PLL_LOCKED,    __sd_pwr_down_notif_send, SD_ST_PLL_CFG	 },
	{ SD_ST_CAL_STARTED,  SD_EVT_PLL_UNLOCKED,  __sd_pwr_down_notif_send, SD_ST_PLL_UNLOCKED },
	{ SD_ST_CAL_STARTED,  SD_EVT_LNK_UP,	    __sd_pwr_down_notif_send, SD_ST_RATE_CHANGED },

	{ SD_ST_SIGNAL_OK,    SD_EVT_LOS_DETECTED,  __do_nothing,	      SD_ST_LOS		 },
	{ SD_ST_SIGNAL_OK,    SD_EVT_APP_INACT,	    __sd_pwr_down,	      SD_ST_LNK_UP_PEND	 },
	{ SD_ST_SIGNAL_OK,    SD_EVT_LNK_DOWN,	    __sd_pwr_down_notif_send, SD_ST_PWR_DOWN	 },
	{ SD_ST_SIGNAL_OK,    SD_EVT_PLL_LOCKED,    __sd_pwr_down_notif_send, SD_ST_PLL_CFG	 },
	{ SD_ST_SIGNAL_OK,    SD_EVT_PLL_UNLOCKED,  __sd_pwr_down_notif_send, SD_ST_PLL_UNLOCKED },
	{ SD_ST_SIGNAL_OK,    SD_EVT_LNK_UP,	    __sd_pwr_down_notif_send, SD_ST_RATE_CHANGED },

	{ SD_ST_LOS,	      SD_EVT_SIGNAL_OK,	    __do_nothing,	      SD_ST_SIGNAL_OK	 },
	{ SD_ST_LOS,	      SD_EVT_APP_INACT,	    __sd_pwr_down,	      SD_ST_LNK_UP_PEND	 },
	{ SD_ST_LOS,	      SD_EVT_LNK_DOWN,	    __sd_pwr_down_notif_send, SD_ST_PWR_DOWN	 },
	{ SD_ST_LOS,	      SD_EVT_PLL_LOCKED,    __sd_pwr_down_notif_send, SD_ST_PLL_CFG	 },
	{ SD_ST_LOS,	      SD_EVT_PLL_UNLOCKED,  __sd_pwr_down_notif_send, SD_ST_PLL_UNLOCKED },
	{ SD_ST_LOS,	      SD_EVT_LNK_UP,	    __sd_pwr_down_notif_send, SD_ST_RATE_CHANGED },

	{ SD_ST_PWR_DOWN,     SD_EVT_APP_INACT,	    __sd_pwr_down,	      SD_ST_IDLE	 },
	{ SD_ST_PWR_DOWN,     SD_EVT_PWR_DOWN_DONE, __sd_lnk_down_notif,      SD_ST_APP_ACTV	 },
	{ SD_ST_PWR_DOWN,     SD_EVT_LNK_UP,	    __sd_cfg_pll_req,	      SD_ST_PLL_CFG	 },

	{ SD_ST_RATE_CHANGED, SD_EVT_APP_INACT,	    __sd_pwr_down,	      SD_ST_IDLE	 },
	{ SD_ST_RATE_CHANGED, SD_EVT_PWR_DOWN_DONE, __sd_cfg_pll_req,	      SD_ST_PLL_CFG	 },
};

static struct adrv906x_phy_fsm_tran adrv906x_phy_fsm_pll_trans[] = {
	/* Source State          Event                 Action               	Destination State */
	{ PLL_ST_UNLOCKED,	 PLL_EVT_LNK0_10G_REQ, __pll_cfg_10G_send,	PLL_ST_LNK0_10G_PEND  },
	{ PLL_ST_UNLOCKED,	 PLL_EVT_LNK1_10G_REQ, __pll_cfg_10G_send,	PLL_ST_LNK1_10G_PEND  },
	{ PLL_ST_UNLOCKED,	 PLL_EVT_LNK0_25G_REQ, __pll_cfg_25G_send,	PLL_ST_LNK0_25G_PEND  },
	{ PLL_ST_UNLOCKED,	 PLL_EVT_LNK1_25G_REQ, __pll_cfg_25G_send,	PLL_ST_LNK1_25G_PEND  },

	{ PLL_ST_10G_RUN,	 PLL_EVT_LNK0_10G_REQ, __pll_lkd_notif_lnk0,	PLL_ST_10G_RUN	      },
	{ PLL_ST_10G_RUN,	 PLL_EVT_LNK1_10G_REQ, __pll_lkd_notif_lnk1,	PLL_ST_10G_RUN	      },
	{ PLL_ST_10G_RUN,	 PLL_EVT_LNK0_25G_REQ, __pll_check_actv_lnks,	PLL_ST_LNK0_25G_REQ   },
	{ PLL_ST_10G_RUN,	 PLL_EVT_LNK1_25G_REQ, __pll_check_actv_lnks,	PLL_ST_LNK1_25G_REQ   },
	{ PLL_ST_10G_RUN,	 PLL_EVT_UNLOCKED,     __pll_unlkd_notif_lnk01, PLL_ST_UNLOCKED	      },
	{ PLL_ST_10G_RUN,	 PLL_EVT_APP_INACT,    __do_nothing,		PLL_ST_UNLOCKED	      },

	{ PLL_ST_25G_RUN,	 PLL_EVT_LNK0_25G_REQ, __pll_lkd_notif_lnk0,	PLL_ST_25G_RUN	      },
	{ PLL_ST_25G_RUN,	 PLL_EVT_LNK1_25G_REQ, __pll_lkd_notif_lnk1,	PLL_ST_25G_RUN	      },
	{ PLL_ST_25G_RUN,	 PLL_EVT_LNK0_10G_REQ, __pll_check_actv_lnks,	PLL_ST_LNK0_10G_REQ   },
	{ PLL_ST_25G_RUN,	 PLL_EVT_LNK1_10G_REQ, __pll_check_actv_lnks,	PLL_ST_LNK1_10G_REQ   },
	{ PLL_ST_25G_RUN,	 PLL_EVT_UNLOCKED,     __pll_unlkd_notif_lnk01, PLL_ST_UNLOCKED	      },
	{ PLL_ST_25G_RUN,	 PLL_EVT_APP_INACT,    __do_nothing,		PLL_ST_UNLOCKED	      },

	{ PLL_ST_LNK0_10G_REQ,	 PLL_EVT_LNK0_DOWN,    __do_nothing,		PLL_ST_25G_RUN	      },
	{ PLL_ST_LNK0_10G_REQ,	 PLL_EVT_LNK0_25G_REQ, __pll_lkd_notif_lnk0,	PLL_ST_25G_RUN	      },
	{ PLL_ST_LNK0_10G_REQ,	 PLL_EVT_LNK1_DOWN,    __pll_cfg_10G_send,	PLL_ST_LNK0_10G_PEND  },
	{ PLL_ST_LNK0_10G_REQ,	 PLL_EVT_LNK1_10G_REQ, __pll_cfg_10G_send,	PLL_ST_LNK01_10G_PEND },
	{ PLL_ST_LNK0_10G_REQ,	 PLL_EVT_UNLOCKED,     __pll_unlkd_notif_lnk01, PLL_ST_UNLOCKED	      },
	{ PLL_ST_LNK0_10G_REQ,	 PLL_EVT_CFG_DONE,     __pll_lkd_notif_lnk1,	PLL_ST_LNK0_10G_REQ   },

	{ PLL_ST_LNK1_10G_REQ,	 PLL_EVT_LNK1_DOWN,    __do_nothing,		PLL_ST_25G_RUN	      },
	{ PLL_ST_LNK1_10G_REQ,	 PLL_EVT_LNK1_25G_REQ, __pll_lkd_notif_lnk1,	PLL_ST_25G_RUN	      },
	{ PLL_ST_LNK1_10G_REQ,	 PLL_EVT_LNK0_DOWN,    __pll_cfg_10G_send,	PLL_ST_LNK1_10G_PEND  },
	{ PLL_ST_LNK1_10G_REQ,	 PLL_EVT_LNK0_10G_REQ, __pll_cfg_10G_send,	PLL_ST_LNK01_10G_PEND },
	{ PLL_ST_LNK1_10G_REQ,	 PLL_EVT_UNLOCKED,     __pll_unlkd_notif_lnk01, PLL_ST_UNLOCKED	      },
	{ PLL_ST_LNK1_10G_REQ,	 PLL_EVT_CFG_DONE,     __pll_lkd_notif_lnk0,	PLL_ST_LNK1_10G_REQ   },

	{ PLL_ST_LNK0_25G_REQ,	 PLL_EVT_LNK0_DOWN,    __do_nothing,		PLL_ST_10G_RUN	      },
	{ PLL_ST_LNK0_25G_REQ,	 PLL_EVT_LNK0_10G_REQ, __pll_lkd_notif_lnk0,	PLL_ST_10G_RUN	      },
	{ PLL_ST_LNK0_25G_REQ,	 PLL_EVT_LNK1_DOWN,    __pll_cfg_25G_send,	PLL_ST_LNK0_25G_PEND  },
	{ PLL_ST_LNK0_25G_REQ,	 PLL_EVT_LNK1_25G_REQ, __pll_cfg_25G_send,	PLL_ST_LNK01_25G_PEND },
	{ PLL_ST_LNK0_25G_REQ,	 PLL_EVT_UNLOCKED,     __pll_unlkd_notif_lnk01, PLL_ST_UNLOCKED	      },
	{ PLL_ST_LNK0_25G_REQ,	 PLL_EVT_CFG_DONE,     __pll_lkd_notif_lnk1,	PLL_ST_LNK0_25G_REQ   },

	{ PLL_ST_LNK1_25G_REQ,	 PLL_EVT_LNK1_DOWN,    __do_nothing,		PLL_ST_10G_RUN	      },
	{ PLL_ST_LNK1_25G_REQ,	 PLL_EVT_LNK1_10G_REQ, __pll_lkd_notif_lnk1,	PLL_ST_10G_RUN	      },
	{ PLL_ST_LNK1_25G_REQ,	 PLL_EVT_LNK0_DOWN,    __pll_cfg_25G_send,	PLL_ST_LNK1_25G_PEND  },
	{ PLL_ST_LNK1_25G_REQ,	 PLL_EVT_LNK0_25G_REQ, __pll_cfg_25G_send,	PLL_ST_LNK01_25G_PEND },
	{ PLL_ST_LNK1_25G_REQ,	 PLL_EVT_UNLOCKED,     __pll_unlkd_notif_lnk01, PLL_ST_UNLOCKED	      },
	{ PLL_ST_LNK1_25G_REQ,	 PLL_EVT_CFG_DONE,     __pll_lkd_notif_lnk0,	PLL_ST_LNK1_25G_REQ   },

	{ PLL_ST_LNK0_10G_PEND,	 PLL_EVT_CFG_DONE,     __pll_lkd_notif_lnk0,	PLL_ST_10G_RUN	      },
	{ PLL_ST_LNK0_10G_PEND,	 PLL_EVT_APP_INACT,    __do_nothing,		PLL_ST_UNLOCKED	      },
	{ PLL_ST_LNK0_10G_PEND,	 PLL_EVT_UNLOCKED,     __pll_unlkd_notif_lnk01, PLL_ST_UNLOCKED	      },
	{ PLL_ST_LNK0_10G_PEND,	 PLL_EVT_LNK1_10G_REQ, __do_nothing,		PLL_ST_LNK01_10G_PEND },
	{ PLL_ST_LNK0_10G_PEND,	 PLL_EVT_LNK1_25G_REQ, __do_nothing,		PLL_ST_LNK1_25G_REQ   },

	{ PLL_ST_LNK1_10G_PEND,	 PLL_EVT_CFG_DONE,     __pll_lkd_notif_lnk1,	PLL_ST_10G_RUN	      },
	{ PLL_ST_LNK1_10G_PEND,	 PLL_EVT_APP_INACT,    __do_nothing,		PLL_ST_UNLOCKED	      },
	{ PLL_ST_LNK1_10G_PEND,	 PLL_EVT_UNLOCKED,     __pll_unlkd_notif_lnk01, PLL_ST_UNLOCKED	      },
	{ PLL_ST_LNK1_10G_PEND,	 PLL_EVT_LNK0_10G_REQ, __do_nothing,		PLL_ST_LNK01_10G_PEND },
	{ PLL_ST_LNK1_10G_PEND,	 PLL_EVT_LNK0_25G_REQ, __do_nothing,		PLL_ST_LNK0_25G_REQ   },

	{ PLL_ST_LNK01_10G_PEND, PLL_EVT_CFG_DONE,     __pll_lkd_notif_lnk01,	PLL_ST_10G_RUN	      },
	{ PLL_ST_LNK01_10G_PEND, PLL_EVT_APP_INACT,    __do_nothing,		PLL_ST_UNLOCKED	      },
	{ PLL_ST_LNK01_10G_PEND, PLL_EVT_UNLOCKED,     __pll_unlkd_notif_lnk01, PLL_ST_UNLOCKED	      },
	{ PLL_ST_LNK01_10G_PEND, PLL_EVT_LNK1_25G_REQ, __do_nothing,		PLL_ST_LNK1_25G_REQ   },
	{ PLL_ST_LNK01_10G_PEND, PLL_EVT_LNK0_25G_REQ, __do_nothing,		PLL_ST_LNK0_25G_REQ   },

	{ PLL_ST_LNK0_25G_PEND,	 PLL_EVT_CFG_DONE,     __pll_lkd_notif_lnk0,	PLL_ST_25G_RUN	      },
	{ PLL_ST_LNK0_25G_PEND,	 PLL_EVT_APP_INACT,    __do_nothing,		PLL_ST_UNLOCKED	      },
	{ PLL_ST_LNK0_25G_PEND,	 PLL_EVT_UNLOCKED,     __pll_unlkd_notif_lnk01, PLL_ST_UNLOCKED	      },
	{ PLL_ST_LNK0_25G_PEND,	 PLL_EVT_LNK1_25G_REQ, __do_nothing,		PLL_ST_LNK01_25G_PEND },
	{ PLL_ST_LNK0_25G_PEND,	 PLL_EVT_LNK1_10G_REQ, __do_nothing,		PLL_ST_LNK1_10G_REQ   },

	{ PLL_ST_LNK1_25G_PEND,	 PLL_EVT_CFG_DONE,     __pll_lkd_notif_lnk1,	PLL_ST_25G_RUN	      },
	{ PLL_ST_LNK1_25G_PEND,	 PLL_EVT_APP_INACT,    __do_nothing,		PLL_ST_UNLOCKED	      },
	{ PLL_ST_LNK1_25G_PEND,	 PLL_EVT_UNLOCKED,     __pll_unlkd_notif_lnk01, PLL_ST_UNLOCKED	      },
	{ PLL_ST_LNK1_25G_PEND,	 PLL_EVT_LNK0_25G_REQ, __do_nothing,		PLL_ST_LNK01_25G_PEND },
	{ PLL_ST_LNK1_25G_PEND,	 PLL_EVT_LNK0_10G_REQ, __do_nothing,		PLL_ST_LNK0_10G_REQ   },

	{ PLL_ST_LNK01_25G_PEND, PLL_EVT_CFG_DONE,     __pll_lkd_notif_lnk01,	PLL_ST_25G_RUN	      },
	{ PLL_ST_LNK01_25G_PEND, PLL_EVT_APP_INACT,    __do_nothing,		PLL_ST_UNLOCKED	      },
	{ PLL_ST_LNK01_25G_PEND, PLL_EVT_UNLOCKED,     __pll_unlkd_notif_lnk01, PLL_ST_UNLOCKED	      },
	{ PLL_ST_LNK01_25G_PEND, PLL_EVT_LNK1_10G_REQ, __do_nothing,		PLL_ST_LNK1_10G_REQ   },
	{ PLL_ST_LNK01_25G_PEND, PLL_EVT_LNK0_10G_REQ, __do_nothing,		PLL_ST_LNK0_10G_REQ   },
};

static struct nla_policy adrv906x_phy_genl_policy[ATTR_MAX + 1] = {
	[ATTR_CMD_PAYLOAD] = { .type = NLA_U32 },
};

static const struct genl_small_ops adrv906x_phy_genl_ops[] = {
	{
		.cmd = NL_CMD_PLL_CFG_DONE,
		.doit = __pll_cfg_done_recv,
	},
	{
		.cmd = NL_CMD_PLL_RELOCK_SUCCEED,
		.doit = __pll_relock_succeed_recv,
	},
	{
		.cmd = NL_CMD_PLL_RELOCK_FAILED,
		.doit = __pll_relock_failed_recv,
	},
	{
		.cmd = NL_CMD_SER_CFG_DONE,
		.doit = __sd_ser_cfg_done_recv,
	},
	{
		.cmd = NL_CMD_DESER_CFG_DONE,
		.doit = __sd_deser_cfg_done_recv,
	},
	{
		.cmd = NL_CMD_DESER_SIGNAL_OK,
		.doit = __sd_deser_signal_ok_recv,
	},
	{
		.cmd = NL_CMD_DESER_LOS,
		.doit = __sd_deser_los_detected_recv,
	},
	{
		.cmd = NL_CMD_SERDES_PWR_DOWN_RDY,
		.doit = __sd_app_pwr_down_rdy_recv,
	},
	{
		.cmd = NL_CMD_SERDES_APP_HEARTBEAT,
		.doit = __sd_app_heartbeat_recv,
	},
};

static const struct genl_multicast_group adrv906x_phy_genl_mcgrps[] = {
	{ .name = ADRV906X_GENL_MC_GRP_NAME },
};

static struct genl_family adrv906x_phy_genl_fam __ro_after_init = {
	.name		= ADRV906X_GENL_NAME,
	.hdrsize	= 0,
	.version	= ADRV906X_GENL_VERSION,
	.maxattr	= ATTR_MAX,
	.policy		= adrv906x_phy_genl_policy,
	.module		= THIS_MODULE,
	.small_ops	= adrv906x_phy_genl_ops,
	.n_small_ops	= ARRAY_SIZE(adrv906x_phy_genl_ops),
	.mcgrps		= adrv906x_phy_genl_mcgrps,
	.n_mcgrps	= ARRAY_SIZE(adrv906x_phy_genl_mcgrps),
};

static DECLARE_DELAYED_WORK(adrv_906x_app_watchdog_task, __sd_app_watchdog_expired);

static struct adrv906x_serdes adrv906x_serdes_devs[ADRV906X_PHY_MAX_LANES] = {
	[0] = { .dev_id = -1 },
	[1] = { .dev_id = -1 },
	[2] = { .dev_id = -1 },
	[3] = { .dev_id = -1 },
};

static struct adrv906x_pll adrv906x_pll_dev[ADRV906X_PHY_MAX_PLLS] = {
	[0] = { .dev_id = -1, .mtx = __MUTEX_INITIALIZER(adrv906x_pll_dev[0].mtx) },
	[1] = { .dev_id = -1, .mtx = __MUTEX_INITIALIZER(adrv906x_pll_dev[1].mtx) },
};

static struct adrv906x_serdes *adrv906x_serdes_instance_get(int dev_id)
{
	if (dev_id >= ADRV906X_PHY_MAX_LANES)
		return NULL;

	return &adrv906x_serdes_devs[dev_id];
}

static struct adrv906x_pll *adrv906x_pll_instance_get(int dev_id)
{
	if (dev_id >= ADRV906X_PHY_MAX_PLLS)
		return NULL;

	return &adrv906x_pll_dev[dev_id];
}

static char *adrv906x_serdes_state_to_str(u32 state)
{
	switch (state) {
	case SD_ST_IDLE:         return "IDLE";
	case SD_ST_LNK_UP_PEND:  return "LNK_UP_PEND";
	case SD_ST_APP_ACTV:     return "APP_ACTV";
	case SD_ST_PLL_UNLOCKED: return "PLL_UNLOCKED";
	case SD_ST_PLL_CFG:      return "PLL_CFG";
	case SD_ST_RATE_CHANGED: return "RATE_CHANGED";
	case SD_ST_SER_CFG:      return "SER_CFG";
	case SD_ST_DESER_CFG:    return "DESER_CFG";
	case SD_ST_CAL_STARTED:  return "CAL_STARTED";
	case SD_ST_LOS:          return "LOS";
	case SD_ST_SIGNAL_OK:    return "SIGNAL_OK";
	case SD_ST_PWR_DOWN:     return "PWR_DOWN";
	default:                 return "UNKNOWN";
	}
}

static char *adrv906x_serdes_event_to_str(u32 event)
{
	switch (event) {
	case SD_EVT_APP_ACTV:      return "APP_ACTV";
	case SD_EVT_APP_INACT:     return "APP_INACT";
	case SD_EVT_LNK_UP:        return "LNK_UP";
	case SD_EVT_PLL_LOCKED:    return "PLL_LOCKED";
	case SD_EVT_PLL_UNLOCKED:  return "PLL_UNLOCKED";
	case SD_EVT_SER_RDY:       return "SER_RDY";
	case SD_EVT_DESER_RDY:     return "DESER_RDY";
	case SD_EVT_LNK_DOWN:      return "LNK_DOWN";
	case SD_EVT_PWR_DOWN_DONE: return "PWR_DOWN_DONE";
	case SD_EVT_SIGNAL_OK:     return "SIGNAL_OK";
	case SD_EVT_LOS_DETECTED:  return "LOS_DETECTED";
	default:                   return "UNKNOWN";
	}
}

static char *adrv906x_pll_state_to_str(u32 state)
{
	switch (state) {
	case PLL_ST_UNLOCKED:       return "UNLOCKED";
	case PLL_ST_10G_RUN:        return "10G_RUN";
	case PLL_ST_25G_RUN:        return "25G_RUN";
	case PLL_ST_LNK0_10G_REQ:   return "LNK0_10G_REQ";
	case PLL_ST_LNK1_10G_REQ:   return "LNK1_10G_REQ";
	case PLL_ST_LNK0_25G_REQ:   return "LNK0_25G_REQ";
	case PLL_ST_LNK1_25G_REQ:   return "LNK1_25G_REQ";
	case PLL_ST_LNK0_10G_PEND:  return "LNK0_10G_PEND";
	case PLL_ST_LNK01_10G_PEND: return "LNK01_10G_PEND";
	case PLL_ST_LNK1_10G_PEND:  return "LNK1_10G_PEND";
	case PLL_ST_LNK0_25G_PEND:  return "LNK0_25G_PEND";
	case PLL_ST_LNK1_25G_PEND:  return "LNK1_25G_PEND";
	case PLL_ST_LNK01_25G_PEND: return "LNK01_25G_PEND";
	default:                    return "UNKNOWN";
	}
}

static char *adrv906x_pll_event_to_str(u32 event)
{
	switch (event) {
	case PLL_EVT_UNLOCKED:     return "UNLOCKED";
	case PLL_EVT_APP_ACTV:     return "APP_ACTV";
	case PLL_EVT_APP_INACT:    return "APP_INACT";
	case PLL_EVT_LNK0_10G_REQ: return "LNK0_10G_REQ";
	case PLL_EVT_LNK1_10G_REQ: return "LNK1_10G_REQ";
	case PLL_EVT_LNK0_25G_REQ: return "LNK0_25G_REQ";
	case PLL_EVT_LNK1_25G_REQ: return "LNK1_25G_REQ";
	case PLL_EVT_CFG_DONE:     return "CFG_DONE";
	case PLL_EVT_LNK0_DOWN:    return "LNK0_DOWN";
	case PLL_EVT_LNK1_DOWN:    return "LNK1_DOWN";
	default:                   return "UNKNOWN";
	}
}

int adrv906x_serdes_genl_register_family(void)
{
	return genl_register_family(&adrv906x_phy_genl_fam);
}

int adrv906x_serdes_genl_unregister_family(void)
{
	return genl_unregister_family(&adrv906x_phy_genl_fam);
}

static int adrv906x_phy_send_message(u32 cmd, u32 dev_id, u32 speed)
{
	struct sk_buff *skb;
	void *hdr;
	u32 data;
	int ret;

	data = FIELD_PREP(ADRV906X_PHY_DEV_IDX_MSK, dev_id)
	       | FIELD_PREP(ADRV906X_PHY_DEV_SPEED_MSK, speed);

	skb = genlmsg_new(NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (unlikely(!skb))
		return -ENOMEM;

	hdr = genlmsg_put(skb, 0, 0, &adrv906x_phy_genl_fam, 0, cmd);
	if (unlikely(!hdr)) {
		nlmsg_free(skb);
		return -ENOMEM;
	}

	ret = nla_put_u32(skb, ATTR_CMD_PAYLOAD, data);
	if (ret) {
		genlmsg_cancel(skb, hdr);
		nlmsg_free(skb);
		return ret;
	}

	genlmsg_end(skb, hdr);

	mutex_lock(&genl_mutex);
	ret = genlmsg_multicast(&adrv906x_phy_genl_fam, skb, 0, 0, GFP_KERNEL);
	mutex_unlock(&genl_mutex);

	return ret;
}

static void adrv906x_phy_fsm_trigger_transition(struct adrv906x_phy_fsm *fsm, int event)
{
	int ret;

	if (!kfifo_initialized(&fsm->event_fifo))
		return;

	ret = kfifo_in_spinlocked(&fsm->event_fifo, &event, sizeof(event), &fsm->event_fifo_lock);
	if (!ret)
		return;

	complete(&fsm->comp_tran);
}

static int adrv906x_phy_fsm_handle_transition(void *data)
{
	struct adrv906x_phy_fsm *fsm = data;
	struct adrv906x_phy_fsm_tran *tran;
	adrv906x_phy_fsm_action action;
	int i, event, ret;

	while (!kthread_should_stop()) {
		if (!wait_for_completion_timeout(&fsm->comp_tran, msecs_to_jiffies(50)))
			continue;

		action = NULL;

		ret = kfifo_out_spinlocked(&fsm->event_fifo, &event, sizeof(event), &fsm->event_fifo_lock);
		if (!ret)
			continue;

		for (i = 0; i < fsm->tran_tbl_size; i++) {
			tran = &fsm->tran_tbl[i];

			if (tran->src_state == atomic_read(&fsm->state) && tran->event == event) {
				pr_debug("[adrv906x] %s, event: %s, transition: %s -> %s",
					 fsm->name,
					 fsm->event_to_str(event),
					 fsm->state_to_str(atomic_read(&fsm->state)),
					 fsm->state_to_str(tran->dst_state));
				atomic_set(&fsm->state, tran->dst_state);
				action = tran->action;
				break;
			}
		}

		if (action)
			action(fsm);
	}

	return 0;
}

static int adrv906x_phy_parse_message(struct genl_info *info, u32 *dev_id, u32 *speed)
{
	u32 data;

	if (!info->attrs[ATTR_CMD_PAYLOAD])
		return -EINVAL;

	data = nla_get_u32(info->attrs[ATTR_CMD_PAYLOAD]);
	*dev_id = FIELD_GET(ADRV906X_PHY_DEV_IDX_MSK, data);
	*speed = FIELD_GET(ADRV906X_PHY_DEV_SPEED_MSK, data);

	return 0;
}

static int __pll_cfg_done_recv(struct sk_buff *skb, struct genl_info *info)
{
	struct adrv906x_pll *pll;
	u32 dev_id, speed;
	int ret;

	ret = adrv906x_phy_parse_message(info, &dev_id, &speed);
	if (ret)
		return ret;

	if (dev_id >= ADRV906X_PHY_MAX_PLLS)
		return -EINVAL;

	pll = adrv906x_pll_instance_get(dev_id);
	adrv906x_phy_fsm_trigger_transition(&pll->fsm, PLL_EVT_CFG_DONE);

	return 0;
}

static int __pll_relock_succeed_recv(struct sk_buff *skb, struct genl_info *info)
{
	struct adrv906x_serdes *serdes0;
	struct adrv906x_serdes *serdes1;
	u32 dev_id, speed;
	int ret;

	ret = adrv906x_phy_parse_message(info, &dev_id, &speed);
	if (ret)
		return ret;

	if (dev_id >= ADRV906X_PHY_MAX_PLLS)
		return -EINVAL;

	if (speed != SPEED_10000 && speed != SPEED_25000)
		return -EINVAL;

	serdes0 = adrv906x_serdes_instance_get(2 * dev_id);
	serdes1 = adrv906x_serdes_instance_get(2 * dev_id + 1);

	adrv906x_phy_fsm_trigger_transition(&serdes0->fsm, SD_EVT_PLL_LOCKED);
	adrv906x_phy_fsm_trigger_transition(&serdes1->fsm, SD_EVT_PLL_LOCKED);

	return 0;
}

static int __pll_relock_failed_recv(struct sk_buff *skb, struct genl_info *info)
{
	struct adrv906x_pll *pll;
	u32 dev_id, speed;
	int ret;

	ret = adrv906x_phy_parse_message(info, &dev_id, &speed);
	if (ret)
		return ret;

	if (dev_id >= ADRV906X_PHY_MAX_PLLS)
		return -EINVAL;

	pll = adrv906x_pll_instance_get(dev_id);
	adrv906x_phy_fsm_trigger_transition(&pll->fsm, PLL_EVT_UNLOCKED);

	return 0;
}

static int __sd_app_heartbeat_recv(struct sk_buff *skb, struct genl_info *info)
{
	struct adrv906x_serdes *serdes0 = adrv906x_serdes_instance_get(0);
	struct adrv906x_serdes *serdes1 = adrv906x_serdes_instance_get(1);
	struct adrv906x_serdes *serdes2 = adrv906x_serdes_instance_get(2);
	struct adrv906x_serdes *serdes3 = adrv906x_serdes_instance_get(3);
	struct adrv906x_pll *pll0 = adrv906x_pll_instance_get(0);
	struct adrv906x_pll *pll1 = adrv906x_pll_instance_get(1);

	mod_delayed_work(system_long_wq, &adrv_906x_app_watchdog_task,
			 msecs_to_jiffies(APP_HEARTBEAT_TIMEOUT_MS));

	adrv906x_phy_fsm_trigger_transition(&pll0->fsm, PLL_EVT_APP_ACTV);
	adrv906x_phy_fsm_trigger_transition(&pll1->fsm, PLL_EVT_APP_ACTV);
	adrv906x_phy_fsm_trigger_transition(&serdes0->fsm, SD_EVT_APP_ACTV);
	adrv906x_phy_fsm_trigger_transition(&serdes1->fsm, SD_EVT_APP_ACTV);
	adrv906x_phy_fsm_trigger_transition(&serdes2->fsm, SD_EVT_APP_ACTV);
	adrv906x_phy_fsm_trigger_transition(&serdes3->fsm, SD_EVT_APP_ACTV);

	return 0;
}

static void __sd_app_watchdog_expired(struct work_struct *work)
{
	struct adrv906x_serdes *serdes0 = adrv906x_serdes_instance_get(0);
	struct adrv906x_serdes *serdes1 = adrv906x_serdes_instance_get(1);
	struct adrv906x_serdes *serdes2 = adrv906x_serdes_instance_get(2);
	struct adrv906x_serdes *serdes3 = adrv906x_serdes_instance_get(3);
	struct adrv906x_pll *pll0 = adrv906x_pll_instance_get(0);
	struct adrv906x_pll *pll1 = adrv906x_pll_instance_get(1);

	adrv906x_phy_fsm_trigger_transition(&pll0->fsm, PLL_EVT_APP_INACT);
	adrv906x_phy_fsm_trigger_transition(&pll1->fsm, PLL_EVT_APP_INACT);
	adrv906x_phy_fsm_trigger_transition(&serdes0->fsm, SD_EVT_APP_INACT);
	adrv906x_phy_fsm_trigger_transition(&serdes1->fsm, SD_EVT_APP_INACT);
	adrv906x_phy_fsm_trigger_transition(&serdes2->fsm, SD_EVT_APP_INACT);
	adrv906x_phy_fsm_trigger_transition(&serdes3->fsm, SD_EVT_APP_INACT);
}

static int __sd_ser_cfg_done_recv(struct sk_buff *skb, struct genl_info *info)
{
	struct adrv906x_serdes *serdes;
	struct phy_device *phydev;
	struct net_device *netdev;
	u32 dev_id, speed;
	int ret;

	ret = adrv906x_phy_parse_message(info, &dev_id, &speed);
	if (ret)
		return ret;

	if (dev_id >= ADRV906X_PHY_MAX_LANES)
		return -EINVAL;

	if (speed != SPEED_10000 && speed != SPEED_25000)
		return -EINVAL;

	serdes = adrv906x_serdes_instance_get(dev_id);
	phydev = serdes->phydev;
	netdev = phydev->attached_dev;

	serdes->tx_path_en(phydev, true);
	adrv906x_eth_cmn_ser_tx_sync_trigger(netdev);
	adrv906x_phy_fsm_trigger_transition(&serdes->fsm, SD_EVT_SER_RDY);

	return 0;
}

static int __sd_deser_cfg_done_recv(struct sk_buff *skb, struct genl_info *info)
{
	struct adrv906x_serdes *serdes;
	u32 dev_id, speed;
	int ret;

	ret = adrv906x_phy_parse_message(info, &dev_id, &speed);
	if (ret)
		return ret;

	if (dev_id >= ADRV906X_PHY_MAX_LANES)
		return -EINVAL;

	if (speed != SPEED_10000 && speed != SPEED_25000)
		return -EINVAL;

	serdes = adrv906x_serdes_instance_get(dev_id);
	adrv906x_phy_fsm_trigger_transition(&serdes->fsm, SD_EVT_DESER_RDY);

	return 0;
}

static int __sd_deser_signal_ok_recv(struct sk_buff *skb, struct genl_info *info)
{
	struct adrv906x_serdes *serdes;
	struct phy_device *phydev;
	struct net_device *netdev;
	u32 dev_id, speed;
	int ret;

	ret = adrv906x_phy_parse_message(info, &dev_id, &speed);
	if (ret)
		return ret;

	if (dev_id >= ADRV906X_PHY_MAX_LANES)
		return -EINVAL;

	if (speed != SPEED_10000 && speed != SPEED_25000)
		return -EINVAL;

	serdes = adrv906x_serdes_instance_get(dev_id);
	phydev = serdes->phydev;
	netdev = phydev->attached_dev;

	/* Reset PCS RX/TX data path */
	serdes->rx_path_en(phydev, false);
	serdes->rx_path_en(phydev, true);
	serdes->tx_path_en(phydev, false);
	serdes->tx_path_en(phydev, true);

	adrv906x_phy_fsm_trigger_transition(&serdes->fsm, SD_EVT_SIGNAL_OK);

	return 0;
}

static int __sd_app_pwr_down_rdy_recv(struct sk_buff *skb, struct genl_info *info)
{
	struct adrv906x_serdes *serdes;
	struct phy_device *phydev;
	struct net_device *netdev;
	u32 dev_id, speed;
	int ret;

	ret = adrv906x_phy_parse_message(info, &dev_id, &speed);
	if (ret)
		return ret;

	if (dev_id >= ADRV906X_PHY_MAX_LANES)
		return -EINVAL;

	if (speed != SPEED_10000 && speed != SPEED_25000)
		return -EINVAL;

	serdes = adrv906x_serdes_instance_get(dev_id);
	phydev = serdes->phydev;
	netdev = phydev->attached_dev;

	adrv906x_phy_fsm_trigger_transition(&serdes->fsm, SD_EVT_PWR_DOWN_DONE);

	return 0;
}

static int __sd_deser_los_detected_recv(struct sk_buff *skb, struct genl_info *info)
{
	struct adrv906x_serdes *serdes;
	struct phy_device *phydev;
	struct net_device *netdev;
	u32 dev_id, speed;
	int ret;

	ret = adrv906x_phy_parse_message(info, &dev_id, &speed);
	if (ret)
		return ret;

	if (dev_id >= ADRV906X_PHY_MAX_LANES)
		return -EINVAL;

	if (speed != SPEED_10000 && speed != SPEED_25000)
		return -EINVAL;

	serdes = adrv906x_serdes_instance_get(dev_id);
	phydev = serdes->phydev;
	netdev = phydev->attached_dev;
	serdes->rx_path_en(phydev, false);

	adrv906x_phy_fsm_trigger_transition(&serdes->fsm, SD_EVT_LOS_DETECTED);

	return 0;
}

static void __sd_pwr_down(void *param)
{
	struct adrv906x_phy_fsm *serdes_fsm = param;
	struct adrv906x_serdes *serdes = container_of(serdes_fsm, struct adrv906x_serdes, fsm);
	struct phy_device *phydev = serdes->phydev;
	struct net_device *netdev = phydev->attached_dev;

	adrv906x_eth_cmn_ser_pwr_down(netdev);
	adrv906x_eth_cmn_deser_pwr_down(netdev);
	serdes->rx_path_en(phydev, false);
	serdes->tx_path_en(phydev, false);
}

static void __sd_cfg_pll_req(void *param)
{
	struct adrv906x_phy_fsm *serdes_fsm = param;
	struct adrv906x_serdes *serdes = container_of(serdes_fsm, struct adrv906x_serdes, fsm);
	struct adrv906x_pll *pll;
	int event = PLL_EVT_UNKNOWN;

	if (serdes->speed == SPEED_10000 && serdes->dev_id % 2 == 0)
		event = PLL_EVT_LNK0_10G_REQ;
	else if (serdes->speed == SPEED_10000 && serdes->dev_id % 2 == 1)
		event = PLL_EVT_LNK1_10G_REQ;
	else if (serdes->speed == SPEED_25000 && serdes->dev_id % 2 == 0)
		event = PLL_EVT_LNK0_25G_REQ;
	else if (serdes->speed == SPEED_25000 && serdes->dev_id % 2 == 1)
		event = PLL_EVT_LNK1_25G_REQ;

	pll = adrv906x_pll_instance_get(serdes->dev_id / 2);
	adrv906x_phy_fsm_trigger_transition(&pll->fsm, event);
}

static void __sd_lnk_down_notif(void *param)
{
	struct adrv906x_phy_fsm *serdes_fsm = param;
	struct adrv906x_serdes *serdes = container_of(serdes_fsm, struct adrv906x_serdes, fsm);
	struct adrv906x_pll *pll;
	int event = PLL_EVT_UNKNOWN;

	if (serdes->dev_id % 2 == 0)
		event = PLL_EVT_LNK0_DOWN;
	else if (serdes->dev_id % 2 == 1)
		event = PLL_EVT_LNK1_DOWN;

	__sd_pwr_down(&serdes->fsm);
	pll = adrv906x_pll_instance_get(serdes->dev_id / 2);
	adrv906x_phy_fsm_trigger_transition(&pll->fsm, event);
}

static void __sd_ser_cfg_send(void *param)
{
	struct adrv906x_phy_fsm *fsm = param;
	struct adrv906x_serdes *serdes = container_of(fsm, struct adrv906x_serdes, fsm);
	struct phy_device *phydev = serdes->phydev;
	struct net_device *netdev = phydev->attached_dev;
	int ret;

	adrv906x_eth_cmn_ser_pwr_up_and_reset(netdev);
	ret = adrv906x_phy_send_message(NL_CMD_SER_CFG_REQ, serdes->dev_id, serdes->speed);
	if (ret)
		adrv906x_phy_fsm_trigger_transition(fsm, SD_EVT_APP_INACT);
}

static void __sd_deser_cfg_send(void *param)
{
	struct adrv906x_phy_fsm *fsm = param;
	struct adrv906x_serdes *serdes = container_of(fsm, struct adrv906x_serdes, fsm);
	struct phy_device *phydev = serdes->phydev;
	struct net_device *netdev = phydev->attached_dev;
	int ret;

	adrv906x_eth_cmn_deser_pwr_up_and_reset(netdev);
	ret = adrv906x_phy_send_message(NL_CMD_DESER_CFG_REQ, serdes->dev_id, serdes->speed);
	if (ret)
		adrv906x_phy_fsm_trigger_transition(fsm, SD_EVT_APP_INACT);
}

static void __sd_deser_init_cal_send(void *param)
{
	struct adrv906x_phy_fsm *fsm = param;
	struct adrv906x_serdes *serdes = container_of(fsm, struct adrv906x_serdes, fsm);
	int ret;

	ret = adrv906x_phy_send_message(NL_CMD_DESER_INIT_CAL_REQ, serdes->dev_id, serdes->speed);
	if (ret)
		adrv906x_phy_fsm_trigger_transition(fsm, SD_EVT_APP_INACT);
}

static void __sd_pwr_down_notif_send(void *param)
{
	struct adrv906x_phy_fsm *fsm = param;
	struct adrv906x_serdes *serdes = container_of(fsm, struct adrv906x_serdes, fsm);
	int ret;

	ret = adrv906x_phy_send_message(NL_CMD_SERDES_PWR_DOWN_REQ, serdes->dev_id, serdes->speed);
	if (ret)
		adrv906x_phy_fsm_trigger_transition(fsm, SD_EVT_APP_INACT);
}

static void __do_nothing(void *param)
{
}

int adrv906x_serdes_lnk_up_req(struct phy_device *phydev)
{
	struct adrv906x_serdes *serdes;
	struct net_device *netdev;
	int dev_id = phydev->mdio.addr;

	netdev = phydev->attached_dev;
	serdes = adrv906x_serdes_instance_get(dev_id);
	if (!serdes)
		return -EINVAL;

	serdes->rx_path_en(phydev, false);
	serdes->tx_path_en(phydev, false);
	serdes->speed = phydev->speed;

	adrv906x_phy_fsm_trigger_transition(&serdes->fsm, SD_EVT_LNK_UP);

	return 0;
}

int adrv906x_serdes_lnk_down_req(struct phy_device *phydev)
{
	struct adrv906x_serdes *serdes;
	int dev_id = phydev->mdio.addr;

	serdes = adrv906x_serdes_instance_get(dev_id);
	if (!serdes)
		return -EINVAL;

	adrv906x_phy_fsm_trigger_transition(&serdes->fsm, SD_EVT_LNK_DOWN);

	return 0;
}

static bool adrv906x_serdes_disabled(int dev_id)
{
	struct adrv906x_serdes *serdes = adrv906x_serdes_instance_get(dev_id);

	switch (atomic_read(&serdes->fsm.state)) {
	case SD_ST_SER_CFG:
	case SD_ST_DESER_CFG:
	case SD_ST_CAL_STARTED:
	case SD_ST_LOS:
	case SD_ST_SIGNAL_OK:
		return false;
	default:
		return true;
	}
}

static void __pll_unlkd_notif_lnk01(void *param)
{
	struct adrv906x_phy_fsm *fsm = param;
	struct adrv906x_pll *pll = container_of(fsm, struct adrv906x_pll, fsm);
	struct adrv906x_serdes *serdes0;
	struct adrv906x_serdes *serdes1;

	serdes0 = adrv906x_serdes_instance_get(2 * pll->dev_id);
	serdes1 = adrv906x_serdes_instance_get(2 * pll->dev_id + 1);

	adrv906x_phy_fsm_trigger_transition(&serdes0->fsm, SD_EVT_PLL_UNLOCKED);
	adrv906x_phy_fsm_trigger_transition(&serdes1->fsm, SD_EVT_PLL_UNLOCKED);
}

static void __pll_lkd_notif_lnk0(void *param)
{
	struct adrv906x_phy_fsm *fsm = param;
	struct adrv906x_pll *pll = container_of(fsm, struct adrv906x_pll, fsm);
	struct adrv906x_serdes *serdes;

	serdes = adrv906x_serdes_instance_get(2 * pll->dev_id);

	adrv906x_phy_fsm_trigger_transition(&serdes->fsm, SD_EVT_PLL_LOCKED);
}

static void __pll_lkd_notif_lnk1(void *param)
{
	struct adrv906x_phy_fsm *fsm = param;
	struct adrv906x_pll *pll = container_of(fsm, struct adrv906x_pll, fsm);
	struct adrv906x_serdes *serdes;

	serdes = adrv906x_serdes_instance_get(2 * pll->dev_id + 1);

	adrv906x_phy_fsm_trigger_transition(&serdes->fsm, SD_EVT_PLL_LOCKED);
}

static void __pll_lkd_notif_lnk01(void *param)
{
	__pll_lkd_notif_lnk0(param);
	__pll_lkd_notif_lnk1(param);
}

static void __pll_check_actv_lnks(void *param)
{
	struct adrv906x_phy_fsm *fsm = param;
	struct adrv906x_pll *pll = container_of(fsm, struct adrv906x_pll, fsm);
	int event = PLL_EVT_UNKNOWN;

	if ((atomic_read(&fsm->state) == PLL_ST_LNK0_10G_REQ || atomic_read(&fsm->state) == PLL_ST_LNK0_25G_REQ)
	    && adrv906x_serdes_disabled(2 * pll->dev_id + 1))
		event = PLL_EVT_LNK1_DOWN;
	else if ((atomic_read(&fsm->state) == PLL_ST_LNK1_10G_REQ || atomic_read(&fsm->state) == PLL_ST_LNK1_25G_REQ)
		 && adrv906x_serdes_disabled(2 * pll->dev_id))
		event = PLL_EVT_LNK0_DOWN;

	adrv906x_phy_fsm_trigger_transition(fsm, event);
}

static void __pll_cfg_10G_send(void *param)
{
	struct adrv906x_phy_fsm *fsm = param;
	struct adrv906x_pll *pll = container_of(fsm, struct adrv906x_pll, fsm);
	struct adrv906x_serdes *serdes;
	struct phy_device *phydev;
	struct net_device *netdev;
	int ret;

	if (adrv906x_serdes_instance_get(2 * pll->dev_id))
		serdes = adrv906x_serdes_instance_get(2 * pll->dev_id);
	else
		serdes = adrv906x_serdes_instance_get(2 * pll->dev_id + 1);

	phydev = serdes->phydev;
	netdev = phydev->attached_dev;

	adrv906x_eth_cmn_pll_reset(netdev);
	adrv906x_eth_cmn_mode_cfg(netdev);
	ret = adrv906x_phy_send_message(NL_CMD_PLL_CFG_REQ, pll->dev_id, SPEED_10000);
	if (ret)
		adrv906x_phy_fsm_trigger_transition(fsm, PLL_EVT_APP_INACT);
}

static void __pll_cfg_25G_send(void *param)
{
	struct adrv906x_phy_fsm *fsm = param;
	struct adrv906x_pll *pll = container_of(fsm, struct adrv906x_pll, fsm);
	struct adrv906x_serdes *serdes;
	struct phy_device *phydev;
	struct net_device *netdev;
	int ret;

	if (adrv906x_serdes_instance_get(2 * pll->dev_id))
		serdes = adrv906x_serdes_instance_get(2 * pll->dev_id);
	else
		serdes = adrv906x_serdes_instance_get(2 * pll->dev_id + 1);

	phydev = serdes->phydev;
	netdev = phydev->attached_dev;

	adrv906x_eth_cmn_pll_reset(netdev);
	adrv906x_eth_cmn_mode_cfg(netdev);
	ret = adrv906x_phy_send_message(NL_CMD_PLL_CFG_REQ, pll->dev_id, SPEED_25000);
	if (ret)
		adrv906x_phy_fsm_trigger_transition(fsm, PLL_EVT_APP_INACT);
}

static int adrv906x_pll_open(int dev_id)
{
	struct adrv906x_pll *pll = adrv906x_pll_instance_get(dev_id);
	int ret;

	mutex_lock(&pll->mtx);
	if (!pll->started) {
		init_completion(&pll->fsm.comp_tran);
		spin_lock_init(&pll->fsm.event_fifo_lock);
		ret = kfifo_alloc(&pll->fsm.event_fifo, 32, GFP_KERNEL);
		if (ret) {
			pr_err("failed to allocate fifo");
			return ret;
		}
		pll->fsm.task = kthread_run(adrv906x_phy_fsm_handle_transition,
					    &pll->fsm, "pll%d-fsm", dev_id);
		if (IS_ERR(pll->fsm.task)) {
			pr_err("kthread_run() failed");
			mutex_unlock(&pll->mtx);
			return PTR_ERR(pll->fsm.task);
		}
		snprintf(pll->fsm.name, sizeof(pll->fsm.name), "pll%d-fsm", dev_id);
		atomic_set(&pll->fsm.state, PLL_ST_UNLOCKED);
		pll->fsm.tran_tbl = adrv906x_phy_fsm_pll_trans;
		pll->fsm.tran_tbl_size = ARRAY_SIZE(adrv906x_phy_fsm_pll_trans);
		pll->fsm.state_to_str = adrv906x_pll_state_to_str;
		pll->fsm.event_to_str = adrv906x_pll_event_to_str;
		pll->dev_id = dev_id;
		pll->started = true;
	}
	mutex_unlock(&pll->mtx);

	return 0;
}

static void adrv906x_pll_close(int dev_id)
{
	struct adrv906x_pll *pll = adrv906x_pll_instance_get(dev_id);

	mutex_lock(&pll->mtx);
	if (pll->started) {
		kthread_stop(pll->fsm.task);
		kfifo_free(&pll->fsm.event_fifo);
		pll->started = false;
	}
	mutex_unlock(&pll->mtx);
}

int adrv906x_serdes_open(struct phy_device *phydev,
			 adrv906x_serdes_cb tx_cb, adrv906x_serdes_cb rx_cb)
{
	struct adrv906x_serdes *serdes;
	int dev_id = phydev->mdio.addr;
	int ret;

	serdes = adrv906x_serdes_instance_get(dev_id);
	if (!serdes)
		return -EINVAL;

	serdes->phydev = phydev;
	serdes->dev_id = dev_id;
	serdes->tx_path_en = tx_cb;
	serdes->rx_path_en = rx_cb;

	init_completion(&serdes->fsm.comp_tran);
	spin_lock_init(&serdes->fsm.event_fifo_lock);

	ret = kfifo_alloc(&serdes->fsm.event_fifo, 32, GFP_KERNEL);
	if (ret) {
		pr_err("failed to allocate fifo");
		return ret;
	}

	serdes->fsm.task = kthread_run(adrv906x_phy_fsm_handle_transition,
				       &serdes->fsm, "serdes%d-fsm", dev_id);
	if (IS_ERR(serdes->fsm.task)) {
		pr_err("kthread_run() failed");
		return PTR_ERR(serdes->fsm.task);
	}
	snprintf(serdes->fsm.name, sizeof(serdes->fsm.name), "serdes%d-fsm", dev_id);
	atomic_set(&serdes->fsm.state, SD_ST_IDLE);
	serdes->fsm.tran_tbl = adrv906x_phy_fsm_serdes_trans;
	serdes->fsm.tran_tbl_size = ARRAY_SIZE(adrv906x_phy_fsm_serdes_trans);
	serdes->fsm.state_to_str = adrv906x_serdes_state_to_str;
	serdes->fsm.event_to_str = adrv906x_serdes_event_to_str;

	ret = adrv906x_pll_open(dev_id / 2);
	if (ret)
		return ret;

	return 0;
}

int adrv906x_serdes_close(struct phy_device *phydev)
{
	struct adrv906x_serdes *serdes;
	int dev_id = phydev->mdio.addr;

	serdes = adrv906x_serdes_instance_get(dev_id);
	if (!serdes)
		return -EINVAL;

	kthread_stop(serdes->fsm.task);
	kfifo_free(&serdes->fsm.event_fifo);
	adrv906x_pll_close(dev_id / 2);

	return 0;
}
