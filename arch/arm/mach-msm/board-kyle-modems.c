/* linux/arch/arm/mach-xxxx/board-amazing-modems.c
 * Copyright (C) 2010 Samsung Electronics. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/clk.h>

/* inlcude platform specific file */
#include <linux/platform_data/modem.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smsm.h>

#define INT_MASK_REQ_ACK_F	0x0020
#define INT_MASK_REQ_ACK_R	0x0010
#define INT_MASK_RES_ACK_F	0x0008
#define INT_MASK_RES_ACK_R	0x0004
#define INT_MASK_SEND_F		0x0002
#define INT_MASK_SEND_R		0x0001

#define INT_MASK_REQ_ACK_RFS	0x0400 /* Request RES_ACK_RFS		*/
#define INT_MASK_RES_ACK_RFS	0x0200 /* Response of REQ_ACK_RFS	*/
#define INT_MASK_SEND_RFS	0x0100 /* Indicate sending RFS data	*/

#define MSM_A2M_INT(n) (MSM_CSR_BASE + 0x400 + (n) * 4)
#if 0
#define MSM_SM_SIZE		0x8000
#define MSM_SM_FMT_TX_BUFF_SZ	1020
#define MSM_SM_RAW_TX_BUFF_SZ	7160
#define MSM_SM_FMT_RX_BUFF_SZ	1020
#define MSM_SM_RAW_RX_BUFF_SZ	23544
#else
#define MSM_SM_SIZE		0x10000
#define MSM_SM_FMT_TX_BUFF_SZ	1020
#define MSM_SM_RAW_TX_BUFF_SZ	31736
#define MSM_SM_FMT_RX_BUFF_SZ	1020
#define MSM_SM_RAW_RX_BUFF_SZ	31736
#endif

#define MAX_MSM_SHAREDMEM_IPC_DEV	(IPC_RAW + 1)	/* FMT, RAW */

/* Shared Memory configuration */
struct sromc_cfg {
	unsigned attr;
	unsigned size;
	unsigned addr;		/* Start address (physical)	*/
	unsigned end;		/* End address (physical)	*/
};

static struct sromc_cfg msm_sharedmem_cfg = {
	.attr = SMEM_ID_VENDOR0,
	.size = MSM_SM_SIZE,
};

struct msm_sharedmem_ipc_cfg {
	u16 magic;
	u16 access;

	u16 fmt_tx_head;
	u16 fmt_tx_tail;
	u8  fmt_tx_buff[MSM_SM_FMT_TX_BUFF_SZ];

	u16 raw_tx_head;
	u16 raw_tx_tail;
	u8  raw_tx_buff[MSM_SM_RAW_TX_BUFF_SZ];

	u16 fmt_rx_head;
	u16 fmt_rx_tail;
	u8  fmt_rx_buff[MSM_SM_FMT_RX_BUFF_SZ];

	u16 raw_rx_head;
	u16 raw_rx_tail;
	u8  raw_rx_buff[MSM_SM_RAW_RX_BUFF_SZ];

	u16 mbx_ap2cp;
	u16 mbx_cp2ap;
};

struct msm_sharedmem_circ {
	u16 __iomem *head;
	u16 __iomem *tail;
	u8  __iomem *buff;
	u32          size;
};

struct msm_sharedmem_ipc_device {
	char name[16];
	int  id;

	struct msm_sharedmem_circ txq;
	struct msm_sharedmem_circ rxq;

	u16 mask_req_ack;
	u16 mask_res_ack;
	u16 mask_send;
};

struct msm_sharedmem_ipc_map {
	u16 __iomem *magic;
	u16 __iomem *access;

	struct msm_sharedmem_ipc_device dev[MAX_MSM_SHAREDMEM_IPC_DEV];

	u16 __iomem *mbx_ap2cp;
	u16 __iomem *mbx_cp2ap;
};

static struct msm_sharedmem_ipc_map msm_ipc_map;

static void msm_sharedmem_reset(void);
static void msm_sharedmem_clr_intr(void);
static u16  msm_sharedmem_recv_intr(void);
static void msm_sharedmem_send_intr(u16 irq_mask);
static u16  msm_sharedmem_recv_msg(void);
static void msm_sharedmem_send_msg(u16 msg);

static u16  msm_sharedmem_get_magic(void);
static void msm_sharedmem_set_magic(u16 value);
static u16  msm_sharedmem_get_access(void);
static void msm_sharedmem_set_access(u16 value);

static u32  msm_sharedmem_get_tx_head(int dev_id);
static u32  msm_sharedmem_get_tx_tail(int dev_id);
static void msm_sharedmem_set_tx_head(int dev_id, u32 head);
static void msm_sharedmem_set_tx_tail(int dev_id, u32 tail);
static u8 __iomem *msm_sharedmem_get_tx_buff(int dev_id);
static u32  msm_sharedmem_get_tx_buff_size(int dev_id);

static u32  msm_sharedmem_get_rx_head(int dev_id);
static u32  msm_sharedmem_get_rx_tail(int dev_id);
static void msm_sharedmem_set_rx_head(int dev_id, u32 head);
static void msm_sharedmem_set_rx_tail(int dev_id, u32 tail);
static u8 __iomem *msm_sharedmem_get_rx_buff(int dev_id);
static u32  msm_sharedmem_get_rx_buff_size(int dev_id);

static u16  msm_sharedmem_get_mask_req_ack(int dev_id);
static u16  msm_sharedmem_get_mask_res_ack(int dev_id);
static u16  msm_sharedmem_get_mask_send(int dev_id);

static struct modemlink_dpram_control msm_sharedmem_ctrl = {
	.reset      = msm_sharedmem_reset,

	.clear_intr = msm_sharedmem_clr_intr,
	.recv_intr  = msm_sharedmem_recv_intr,
	.send_intr  = msm_sharedmem_send_intr,
	.recv_msg   = msm_sharedmem_recv_msg,
	.send_msg   = msm_sharedmem_send_msg,

	.get_magic  = msm_sharedmem_get_magic,
	.set_magic  = msm_sharedmem_set_magic,
	.get_access = msm_sharedmem_get_access,
	.set_access = msm_sharedmem_set_access,

	.get_tx_head = msm_sharedmem_get_tx_head,
	.get_tx_tail = msm_sharedmem_get_tx_tail,
	.set_tx_head = msm_sharedmem_set_tx_head,
	.set_tx_tail = msm_sharedmem_set_tx_tail,
	.get_tx_buff = msm_sharedmem_get_tx_buff,
	.get_tx_buff_size = msm_sharedmem_get_tx_buff_size,

	.get_rx_head = msm_sharedmem_get_rx_head,
	.get_rx_tail = msm_sharedmem_get_rx_tail,
	.set_rx_head = msm_sharedmem_set_rx_head,
	.set_rx_tail = msm_sharedmem_set_rx_tail,
	.get_rx_buff = msm_sharedmem_get_rx_buff,
	.get_rx_buff_size = msm_sharedmem_get_rx_buff_size,

	.get_mask_req_ack = msm_sharedmem_get_mask_req_ack,
	.get_mask_res_ack = msm_sharedmem_get_mask_res_ack,
	.get_mask_send    = msm_sharedmem_get_mask_send,

	.dp_base = NULL,
	.dp_size = 0,
	.dp_type = SHARED_MEM,

	.dpram_irq        = INT_A9_M2A_3,
	.dpram_irq_flags  = IRQ_TYPE_EDGE_RISING,
	.dpram_irq_name   = "MSM_SHAREDMEM__IRQ",
	.dpram_wlock_name = "MSM_SHAREDMEM_WLOCK",

	.max_ipc_dev = MAX_MSM_SHAREDMEM_IPC_DEV,
};

/*
** CDMA target platform data
*/
static struct modem_io_t umts_io_devices[] = {
	[0] = {
		.name = "umts_ipc0",
		.id = 0x1,
		.format = IPC_FMT,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[1] = {
		.name = "multipdp",
		.id = 0x1,
		.format = IPC_MULTI_RAW,
		.io_type = IODEV_DUMMY,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[2] = {
		.name = "rmnet0",
		.id = 0x2A,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[3] = {
		.name = "rmnet1",
		.id = 0x2B,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[4] = {
		.name = "rmnet2",
		.id = 0x2C,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[5] = {
		.name = "ttyCSD0",
		.id = 0x1,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[6] = {
		.name = "ttyEFS",
		.id = 0x8,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[7] = {
		.name = "ttyGPS0",
		.id = 0x5,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[8] = {
		.name = "ttyXTRA0",
		.id = 0x6,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[9] = {
		.name = "ttySMD0",
		.id = 0x19,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[10] = {
		.name = "ttyCPLOG0",
		.id = 0x1D,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
};

static struct modem_data umts_modem_data = {
	.name = "msm7x27",

	.modem_net  = UMTS_NETWORK,
	.modem_type = QC_MSM7x27,
	.link_types = LINKTYPE(LINKDEV_DPRAM),
	.link_name  = "msm_sharedmem",
	.dpram_ctl  = &msm_sharedmem_ctrl,

	.num_iodevs = ARRAY_SIZE(umts_io_devices),
	.iodevs     = umts_io_devices,

	.use_handover = false,
	.ipc_version = SIPC_VER_41,
};

static struct resource umts_modem_res[] = {
	/* Add something */
};

static struct platform_device umts_modem = {
	.name = "modem_if",
	.id = -1,
	.num_resources = ARRAY_SIZE(umts_modem_res),
	.resource = umts_modem_res,
	.dev = {
		.platform_data = &umts_modem_data,
	},
};

static void msm_sharedmem_reset(void)
{
	iowrite16(0, msm_ipc_map.dev[IPC_FMT].txq.head);
	iowrite16(0, msm_ipc_map.dev[IPC_FMT].txq.tail);
	iowrite16(0, msm_ipc_map.dev[IPC_FMT].rxq.head);
	iowrite16(0, msm_ipc_map.dev[IPC_FMT].rxq.tail);

	iowrite16(0, msm_ipc_map.dev[IPC_RAW].txq.head);
	iowrite16(0, msm_ipc_map.dev[IPC_RAW].txq.tail);
	iowrite16(0, msm_ipc_map.dev[IPC_RAW].rxq.head);
	iowrite16(0, msm_ipc_map.dev[IPC_RAW].rxq.tail);
}

static void msm_sharedmem_clr_intr(void)
{
	ioread16(msm_ipc_map.mbx_cp2ap);
}

static u16 msm_sharedmem_recv_intr(void)
{
	return ioread16(msm_ipc_map.mbx_cp2ap);
}

static void msm_sharedmem_send_intr(u16 irq_mask)
{
	iowrite16(irq_mask, msm_ipc_map.mbx_ap2cp);
	writel(1, MSM_A2M_INT(3));
}

static u16 msm_sharedmem_recv_msg(void)
{
	return ioread16(msm_ipc_map.mbx_cp2ap);
}

static void msm_sharedmem_send_msg(u16 msg)
{
	iowrite16(msg, msm_ipc_map.mbx_ap2cp);
}

static u16 msm_sharedmem_get_magic(void)
{
	return ioread16(msm_ipc_map.magic);
}

static void msm_sharedmem_set_magic(u16 value)
{
	iowrite16(value, msm_ipc_map.magic);
}

static u16 msm_sharedmem_get_access(void)
{
	return ioread16(msm_ipc_map.access);
}

static void msm_sharedmem_set_access(u16 value)
{
	iowrite16(value, msm_ipc_map.access);
}

static u32 msm_sharedmem_get_tx_head(int dev_id)
{
	return ioread16(msm_ipc_map.dev[dev_id].txq.head);
}

static u32 msm_sharedmem_get_tx_tail(int dev_id)
{
	return ioread16(msm_ipc_map.dev[dev_id].txq.tail);
}

static void msm_sharedmem_set_tx_head(int dev_id, u32 head)
{
	iowrite16((u16)head, msm_ipc_map.dev[dev_id].txq.head);
}

static void msm_sharedmem_set_tx_tail(int dev_id, u32 tail)
{
	iowrite16((u16)tail, msm_ipc_map.dev[dev_id].txq.tail);
}

static u8 __iomem *msm_sharedmem_get_tx_buff(int dev_id)
{
	return msm_ipc_map.dev[dev_id].txq.buff;
}

static u32 msm_sharedmem_get_tx_buff_size(int dev_id)
{
	return msm_ipc_map.dev[dev_id].txq.size;
}

static u32 msm_sharedmem_get_rx_head(int dev_id)
{
	return ioread16(msm_ipc_map.dev[dev_id].rxq.head);
}

static u32 msm_sharedmem_get_rx_tail(int dev_id)
{
	return ioread16(msm_ipc_map.dev[dev_id].rxq.tail);
}

static void msm_sharedmem_set_rx_head(int dev_id, u32 head)
{
	return iowrite16((u16)head, msm_ipc_map.dev[dev_id].rxq.head);
}

static void msm_sharedmem_set_rx_tail(int dev_id, u32 tail)
{
	return iowrite16((u16)tail, msm_ipc_map.dev[dev_id].rxq.tail);
}

static u8 __iomem *msm_sharedmem_get_rx_buff(int dev_id)
{
	return msm_ipc_map.dev[dev_id].rxq.buff;
}

static u32 msm_sharedmem_get_rx_buff_size(int dev_id)
{
	return msm_ipc_map.dev[dev_id].rxq.size;
}

static u16 msm_sharedmem_get_mask_req_ack(int dev_id)
{
	return msm_ipc_map.dev[dev_id].mask_req_ack;
}

static u16 msm_sharedmem_get_mask_res_ack(int dev_id)
{
	return msm_ipc_map.dev[dev_id].mask_res_ack;
}

static u16 msm_sharedmem_get_mask_send(int dev_id)
{
	return msm_ipc_map.dev[dev_id].mask_send;
}

/* Set dynamic environment for a modem */
static int setup_umts_modem_env(void)
{
	msm_sharedmem_cfg.addr = (volatile unsigned char *)
		(smem_alloc2(msm_sharedmem_cfg.attr, msm_sharedmem_cfg.size));
	if (!msm_sharedmem_cfg.addr) {
		pr_err("[MDM] <%s> smem_do_alloc failed!!\n", __func__);
		return -ENOMEM;
	}
	msm_sharedmem_cfg.end  = msm_sharedmem_cfg.addr + msm_sharedmem_cfg.size - 1;
	msm_sharedmem_ctrl.dpram_irq = INT_A9_M2A_3;
	return 0;
}

static u8 *msm_sharedmem_remap_mem_region(struct sromc_cfg *cfg)
{
	int			      dp_addr = 0;
	int			      dp_size = 0;
	u8 __iomem                   *dp_base = NULL;
	struct msm_sharedmem_ipc_cfg    *ipc_map = NULL;
	struct msm_sharedmem_ipc_device *dev = NULL;

	dp_addr = cfg->addr;
	dp_size = cfg->size;
	dp_base = (u8 __iomem *)cfg->addr;

	pr_info("[MDM] <%s> DPRAM VA=0x%08X\n", __func__, (int)dp_base);

	msm_sharedmem_ctrl.dp_base = (u8 __iomem *)dp_base;
	msm_sharedmem_ctrl.dp_size = dp_size;

	/* Map for IPC */
	ipc_map = (struct msm_sharedmem_ipc_cfg *)dp_base;

	/* Magic code and access enable fields */
	msm_ipc_map.magic  = (u16 __iomem *)&ipc_map->magic;
	msm_ipc_map.access = (u16 __iomem *)&ipc_map->access;

	/* FMT */
	dev = &msm_ipc_map.dev[IPC_FMT];

	strcpy(dev->name, "FMT");
	dev->id = IPC_FMT;

	dev->txq.head = (u16 __iomem *)&ipc_map->fmt_tx_head;
	dev->txq.tail = (u16 __iomem *)&ipc_map->fmt_tx_tail;
	dev->txq.buff = (u8 __iomem *)&ipc_map->fmt_tx_buff[0];
	dev->txq.size = MSM_SM_FMT_TX_BUFF_SZ;

	dev->rxq.head = (u16 __iomem *)&ipc_map->fmt_rx_head;
	dev->rxq.tail = (u16 __iomem *)&ipc_map->fmt_rx_tail;
	dev->rxq.buff = (u8 __iomem *)&ipc_map->fmt_rx_buff[0];
	dev->rxq.size = MSM_SM_FMT_RX_BUFF_SZ;

	dev->mask_req_ack = INT_MASK_REQ_ACK_F;
	dev->mask_res_ack = INT_MASK_RES_ACK_F;
	dev->mask_send    = INT_MASK_SEND_F;

	/* RAW */
	dev = &msm_ipc_map.dev[IPC_RAW];

	strcpy(dev->name, "RAW");
	dev->id = IPC_RAW;

	dev->txq.head = (u16 __iomem *)&ipc_map->raw_tx_head;
	dev->txq.tail = (u16 __iomem *)&ipc_map->raw_tx_tail;
	dev->txq.buff = (u8 __iomem *)&ipc_map->raw_tx_buff[0];
	dev->txq.size = MSM_SM_RAW_TX_BUFF_SZ;

	dev->rxq.head = (u16 __iomem *)&ipc_map->raw_rx_head;
	dev->rxq.tail = (u16 __iomem *)&ipc_map->raw_rx_tail;
	dev->rxq.buff = (u8 __iomem *)&ipc_map->raw_rx_buff[0];
	dev->rxq.size = MSM_SM_RAW_RX_BUFF_SZ;

	dev->mask_req_ack = INT_MASK_REQ_ACK_R;
	dev->mask_res_ack = INT_MASK_RES_ACK_R;
	dev->mask_send    = INT_MASK_SEND_R;

	/* Mailboxes */
	msm_ipc_map.mbx_ap2cp = (u16 __iomem *)&ipc_map->mbx_ap2cp;
	msm_ipc_map.mbx_cp2ap = (u16 __iomem *)&ipc_map->mbx_cp2ap;

	return dp_base;
}

static int __init init_modem(void)
{
	int ret;
	pr_err("[MDM] <%s> System Revision = %d\n", __func__, system_rev);

	ret = setup_umts_modem_env();
	if (ret < 0)
		return ret;

	if (!msm_sharedmem_remap_mem_region(&msm_sharedmem_cfg))
		return -1;

	platform_device_register(&umts_modem);

	return 0;
}
late_initcall(init_modem);
/* module_initcall(init_modem); */
/*device_initcall(init_modem);*/
