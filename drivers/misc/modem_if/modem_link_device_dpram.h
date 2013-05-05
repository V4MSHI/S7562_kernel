/*
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2010 Samsung Electronics.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/wakelock.h>

#ifndef __MODEM_LINK_DEVICE_DPRAM_H__
#define __MODEM_LINK_DEVICE_DPRAM_H__

#define FMT_IDX			0
#define RAW_IDX			1
#define MAX_IDX			2

#if defined(CONFIG_MACH_AMAZING) /* for MSM7x25(Amazing) */
#define DP_DPRAM_SIZE			0x10000

#define DP_FMT_OUT_BUFF_SIZE            1020
#define DP_RAW_OUT_BUFF_SIZE            31736
#define DP_FMT_IN_BUFF_SIZE             1020
#define DP_RAW_IN_BUFF_SIZE             31736
#elif defined(CONFIG_MACH_PREVAIL2) /* for MSM8x55(Prevail2, Icon) */
#define DP_DPRAM_SIZE			0x8000

#define DP_FMT_OUT_BUFF_SIZE		8186
#define DP_RAW_OUT_BUFF_SIZE		8186
#define DP_FMT_IN_BUFF_SIZE		8186
#define DP_RAW_IN_BUFF_SIZE		8186
#else /* for Trebon, Amazing_cdma */
#define DP_DPRAM_SIZE			0x8000

#define DP_FMT_OUT_BUFF_SIZE		1020
#define DP_RAW_OUT_BUFF_SIZE		7160
#define DP_FMT_IN_BUFF_SIZE		1020
#define DP_RAW_IN_BUFF_SIZE		23544
#endif

/* interrupt masks.*/
#define INT_MASK_VALID			0x0080
#define INT_MASK_CMD			0x0040
#define INT_MASK_REQ_ACK_F		0x0020
#define INT_MASK_REQ_ACK_R		0x0010
#define INT_MASK_RES_ACK_F		0x0008
#define INT_MASK_RES_ACK_R		0x0004
#define INT_MASK_SEND_F			0x0002
#define INT_MASK_SEND_R			0x0001
#define INT_VALID(x)			((x) & INT_MASK_VALID)
#define INT_CMD_VALID(x)		((x) & INT_MASK_CMD)
#define INT_NON_CMD(x)			(INT_MASK_VALID | (x))
#define INT_CMD(x)			(INT_MASK_VALID | INT_MASK_CMD | (x))

#define INT_CMD_MASK(x)			((x) & 0xF)
#define INT_CMD_INIT_START		0x1
#define INT_CMD_INIT_END		0x2
#define INT_CMD_REQ_ACTIVE		0x3
#define INT_CMD_RES_ACTIVE		0x4
#define INT_CMD_REQ_TIME_SYNC		0x5
#define INT_CMD_PHONE_START		0x8
#define INT_CMD_ERR_DISPLAY		0x9
#define INT_CMD_PHONE_DEEP_SLEEP	0xA
#define INT_CMD_NV_REBUILDING		0xB
#define INT_CMD_EMER_DOWN		0xC
#define INT_CMD_PIF_INIT_DONE		0xD
#define INT_CMD_SILENT_NV_REBUILDING	0xE
#define INT_CMD_NORMAL_POWER_OFF	0xF

/* special interrupt cmd indicating modem boot failure. */
#define INT_POWERSAFE_FAIL              0xDEAD

#define GOTA_CMD_VALID(x)		(((x) & 0xA000) == 0xA000)
#define GOTA_RESULT_FAIL		0x2
#define GOTA_RESULT_SUCCESS		0x1
#define GOTA_CMD_MASK(x)		(((x) >> 8) & 0xF)
#define GOTA_CMD_RECEIVE_READY		0x1
#define GOTA_CMD_DOWNLOAD_START_REQ	0x2
#define GOTA_CMD_DOWNLOAD_START_RESP	0x3
#define GOTA_CMD_IMAGE_SEND_REQ		0x4
#define GOTA_CMD_IMAGE_SEND_RESP	0x5
#define GOTA_CMD_SEND_DONE_REQ		0x6
#define GOTA_CMD_SEND_DONE_RESP		0x7
#define GOTA_CMD_STATUS_UPDATE		0x8
#define GOTA_CMD_UPDATE_DONE		0x9
#define GOTA_CMD_EFS_CLEAR_RESP		0xB
#define GOTA_CMD_ALARM_BOOT_OK		0xC
#define GOTA_CMD_ALARM_BOOT_FAIL	0xD

#define CMD_DL_START_REQ		0x9200
#define CMD_IMG_SEND_REQ		0x9400
#define CMD_DL_SEND_DONE_REQ		0x9600
#define CMD_UL_RECEIVE_RESP		0x9601
#define CMD_UL_RECEIVE_DONE_RESP	0x9801

#define START_INDEX			0x7F
#define END_INDEX			0x7E

#define DP_MAGIC_CODE			0xAA
#define DP_MAGIC_DMDL			0x4445444C
#define DP_MAGIC_UMDL			0x4445444D

#define DP_DEFAULT_WRITE_LEN		8168
#define DP_DEFAULT_DUMP_LEN		16366
#define DP_DUMP_HEADER_SIZE		7

#define GOTA_TIMEOUT			(50 * HZ)
#define GOTA_SEND_TIMEOUT		(200 * HZ)
#define DUMP_TIMEOUT			(30 * HZ)
#define DUMP_START_TIMEOUT		(100 * HZ)

struct dpram_circ {
	u16 head;
	u16 tail;
};

struct dpram_ota_header {
	u8 start_index;
	u16 nframes;
	u16 curframe;
	u16 len;

} __packed;

struct dpram_map {
	u16	magic;
	u16	enable;

	struct dpram_circ fmt_out;
	u8	fmt_out_buff[DP_FMT_OUT_BUFF_SIZE];

	struct dpram_circ raw_out;
	u8	raw_out_buff[DP_RAW_OUT_BUFF_SIZE];

	struct dpram_circ fmt_in;
	u8	fmt_in_buff[DP_FMT_IN_BUFF_SIZE];

	struct dpram_circ raw_in;
	u8	raw_in_buff[DP_RAW_IN_BUFF_SIZE];

	/* u8	padding[16]; */
	u16	mbx_ap2cp;
	u16	mbx_cp2ap;

} __packed;

struct dpram_device {
	struct dpram_circ __iomem *in;
	u8 __iomem	*in_buff_addr;
	int		in_buff_size;

	struct dpram_circ __iomem *out;
	u8 __iomem	*out_buff_addr;
	int		out_buff_size;

	u16            mask_req_ack;
	u16            mask_res_ack;
	u16            mask_send;
};

struct ul_header {
	u8 bop;
	u16 total_frame;
	u16 curr_frame;
	u16 len;
};

struct dpram_link_device {
	struct link_device ld;

	/* maybe -list of io devices for the link device to use
	 * to find where to send incoming packets to */
	struct list_head list_of_io_devices;

	atomic_t raw_txq_req_ack_rcvd;
	atomic_t fmt_txq_req_ack_rcvd;

	struct dpram_map __iomem *dpram;
	struct dpram_device dev_map[MAX_IDX];

	struct wake_lock dpram_wake_lock;

	struct completion dpram_init_cmd;
	struct completion modem_pif_init_done;
	struct completion gota_download_start_complete;
	struct completion gota_send_done;
	struct completion gota_update_done;
	struct completion dump_receive_done;

	int irq;
	u32 irq_arg_b;
	void __iomem *irq_arg_addr;

	void (*clear_interrupt)(struct dpram_link_device *);
	void (*cmd_phone_start_handler)(struct dpram_link_device *);

	char dpram_err_buf[128];
	unsigned int is_dpram_err;
	struct fasync_struct *dpram_err_async_q;
};

/* converts from struct link_device* to struct xxx_link_device* */
#define to_dpram_link_device(linkdev) \
			container_of(linkdev, struct dpram_link_device, ld)

#endif
