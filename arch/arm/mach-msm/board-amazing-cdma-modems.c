/* linux/arch/arm/mach-xxxx/board-amazing-cdma-modems.c
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
#define DEBUG
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <mach/msm_smsm.h>
#include <mach/msm_iomap.h>
#include <linux/platform_data/modem.h>

#define DPRAM_START_ADDRESS	0x0
#define DPRAM_SIZE              0x8000
#define DPRAM_END_ADDRESS	(DPRAM_START_ADDRESS + DPRAM_SIZE - 1)

#define MSM_A2M_INT(n) (MSM_CSR_BASE + 0x400 + (n) * 4)

static struct modem_io_t umts_io_devices[] = {
	[0] = {
		.name = "cdma_ipc0",
		.id = 0x1,
		.format = IPC_FMT,
		.io_type = IODEV_MISC,
		.link = LINKDEV_DPRAM,
	},
	[1] = {
		.name = "multipdp",
		.id = 0x1,
		.format = IPC_MULTI_RAW,
		.io_type = IODEV_DUMMY,
		.link = LINKDEV_DPRAM,
	},
	[2] = {
		.name = "ppp0",
		.id = 0x2A,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_DPRAM,
	},
	[3] = {
		.name = "ppp1",
		.id = 0x2B,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_DPRAM,
	},
	[4] = {
		.name = "ppp2",
		.id = 0x2C,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_DPRAM,
	},
	[5] = {
		.name = "ttyCPLOG0",
		.id = 0x1D,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.link = LINKDEV_DPRAM,
	},
	[6] = {
		.name = "umts_router",
		.id = 0x19,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.link = LINKDEV_DPRAM,
	},
	[7] = {
		.name = "cdma_EFS",
		.id = 0x8,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.link = LINKDEV_DPRAM,
	},
};

/* cdma target platform data */
static struct modem_data umts_modem_data = {
	.name = "msm7x27",

	.modem_type = QC_MSM7x27,
	.link_type = LINKDEV_DPRAM,
	.modem_net = UMTS_NETWORK,

	.num_iodevs = ARRAY_SIZE(umts_io_devices),
	.iodevs = umts_io_devices,
};

static struct resource umts_modem_res[] = {
	[0] = {
		.name = "smd_info",
		.start = SMEM_ID_VENDOR0, /* SMEM ID */
		.end = DPRAM_SIZE, /* Interface Memory size (SMD) */
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "smd_irq_info",
		.start = INT_A9_M2A_3, /* IRQ number */
		.end = IRQ_TYPE_EDGE_RISING, /* IRQ Trigger type */
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.name = "smd_irq_arg",
		.start = 0,
		.end = MSM_A2M_INT(3),
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device umts_modem = {
	.name = "modem_if",
	.id = 1,
	.num_resources = ARRAY_SIZE(umts_modem_res),
	.resource = umts_modem_res,
	.dev = {
		.platform_data = &umts_modem_data,
	},
};

static int __init init_modem(void)
{
	int ret;
	pr_debug("[MIF] <%s> init_modem\n", __func__);

	ret = platform_device_register(&umts_modem);
	if (ret < 0)
		pr_err("[MIF] <%s> init_modem failed!!\n", __func__);

	return ret;
}
late_initcall(init_modem);
