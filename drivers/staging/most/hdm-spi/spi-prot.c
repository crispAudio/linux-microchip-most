/*
 * spi_hdm.c - SPI protocol driver as a Hardware Dependent Module
 *
 * Copyright (C) 2017, Microchip Technology Germany II GmbH & Co. KG
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This file is licensed under GPLv2.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/of_irq.h>
#include <linux/ctype.h>
#include <linux/if_ether.h>

#include "mostcore.h"

#define SPI_WR 0x00
#define SPI_RD 0x80

#define DR_CONFIG_ADDR 0x00
#define GINT_CHSTS_ADDR 0x01
#define DCI_CTRL_ADDR 0x03
#define DCI_ADDR 0x10
#define ASYNC_ADDR 0x12
#define CTRL_ADDR 0x14

#define GINT_CHSTS_INTM \
	(BIT(GINT_CHSTS_DCITSM_B) | \
	 BIT(GINT_CHSTS_CTISM_B) | \
	 BIT(GINT_CHSTS_CRISM_B) | \
	 BIT(GINT_CHSTS_ATISM_B) | \
	 BIT(GINT_CHSTS_ARISM_B))

/* Interrupt Status Mask R/W-0 */
#define GINT_CHSTS_DCITSM_B 28 /* dci */
#define GINT_CHSTS_CTISM_B 27 /* control tx */
#define GINT_CHSTS_CRISM_B 26 /* control rx */
#define GINT_CHSTS_ATISM_B 25 /* asynchronous tx */
#define GINT_CHSTS_ARISM_B 24 /* asynchronous rx */

/* Interrupt Status R-0 */
#define GINT_CHSTS_DCITS_B 20 /* dci */
#define GINT_CHSTS_CTIS_B 19 /* control tx */
#define GINT_CHSTS_CRIS_B 18 /* control rx */
#define GINT_CHSTS_ATIS_B 17 /* asynchronous tx */
#define GINT_CHSTS_ARIS_B 16 /* asynchronous rx */

/* DCI_CTRL */
#define DCI_CTRL_ERRDM_BM BIT(26) /* ERRDM */
#define DCI_CTRL_ONTFM_BM BIT(25) /* ONTFM */
#define DCI_CTRL_CMDDONEM_BM BIT(24) /* CMDDONEM */
#define DCI_CTRL_ERRD_BM BIT(18) /* ERRD */
#define DCI_CTRL_ONTF_BM BIT(17) /* ONTF */
#define DCI_CTRL_CMDDONE_BM BIT(16) /* CMDDONE */
#define DCI_CTRL_CMDREQ_BM BIT(1) /* CMDREQ */
#define DCI_CTRL_SYNCREQ_BM BIT(0) /* SYNCREQ */

/* DCI channel index */
#define ARX_DCI_CH_IDX 0
#define ATX_DCI_CH_IDX 1
#define CRX_DCI_CH_IDX 2
#define CTX_DCI_CH_IDX 3

enum { FPH_IDX = 3, MDP_FPH = 0x0C, MEP_FPH = 0x24 };

#define ROUND_UP(x)  ALIGN(x, 4)

enum { CH_ASYNC_TX, CH_ASYNC_RX, CH_CTRL_TX, CH_CTRL_RX, CH_NUM };
enum { EOP_SIZE = 1 }; /* size for end of packet */

struct channel_class;
struct hdm_device;

struct hdm_channel {
	const struct channel_class *cl;
	struct mutex cl_lock; /* cl && mbo_list */
	u16 spi_buf_sz;
	u8 *buf;
	u16 buf_sz;
	u8 *data;
	u16 data_sz;
	u16 spi_data_gap;
	struct list_head mbo_list;
};

/*
 * constant channel parameters same for all devices
 */
struct channel_class {
	const char *name;
	enum most_channel_direction dir;
	enum most_channel_data_type type;
	u8 xch_cmd;
	u8 buf_info_cmd;
	u8 int_mask_bit;
	u8 int_status_bit;
	u16 (*get_spi_buf_sz)(struct hdm_device *, struct hdm_channel *);
	void (*xfer)(struct hdm_device *, struct hdm_channel *);
	bool (*xfer_mbo)(struct hdm_device *, struct hdm_channel *,
			 struct mbo *);
};

enum dci_cmd { DCI_READ_REQ = 0xA0, DCI_WRITE_REQ = 0xA1 };

struct dci_job {
	struct list_head list;
	u8 cmd;
	const u16 *reg;
	const u16 *val;
	void (*complete_fn)(struct hdm_device *, int *);
	int *regs_num;
};

struct dci_params {
	struct kobject kobj_group;
};

struct global_ch_int {
	unsigned long spi_mask;
	unsigned long aim_mask;
};

struct dci_ctrl_reg {
	u32 active_int;
};

struct hdm_device {
	struct spi_device *spi;
	struct mutex spi_lock; /* spi_sync() */
	struct hdm_channel ch[CH_NUM];
	struct most_channel_capability capabilities[CH_NUM];
	struct most_interface most_iface;
	struct work_struct sint_work;
	int irq;
	struct global_ch_int gint;
	struct dci_ctrl_reg dci_reg;
	void (*service_dci)(struct hdm_device *, u32);
	struct dci_params dci;

	/*
	 * protects dci_reg.active_int, service_dci and as a side effect,
	 * active_dci_jobs and dci_jobs_heap
	 */
	struct mutex dci_mt;
	struct mutex ni_mt; /* on_netinfo */

	struct dci_job dci_jobs[14];
	struct list_head active_dci_jobs;
	struct list_head dci_jobs_heap;

	void (*on_netinfo)(struct most_interface *, unsigned char,
			   unsigned char *);

	u8 mac[ETH_ALEN];
	u8 link_stat;

	/*
	 * the size of the software tx buffer must not be bigger than
	 * the interrupt threshold for the corresponding channel
	 * plus 4 bytes for the spi protocol header
	 */
	u8 ctx_buf[4 + 64];
	u8 atx_buf[4 + 1536];

	/*
	 * the size of the software rx buffer must not be smaller than
	 * the full size of the internal spi buffer for
	 * the corresponding channel
	 * plus 4 bytes for the spi protocol header
	 */
	u8 crx_buf[4 + 512];
	u8 arx_buf[4 + 4096];

	u8 dci_buf[4 + 2 + 4 * 7];
};

inline struct hdm_device *to_device(struct most_interface *most_iface)
{
	return container_of(most_iface, struct hdm_device, most_iface);
}

static int spi_hdm_xch(struct hdm_device *mdev, const void *tx_buf,
		       void *rx_buf, size_t count)
{
	struct spi_message m;
	struct spi_transfer t;
	int err;

	spi_message_init(&m);

	memset(&t, 0, sizeof(t));

	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	t.len = count;
	spi_message_add_tail(&t, &m);

	mutex_lock(&mdev->spi_lock);
	err = spi_sync(mdev->spi, &m);
	mutex_unlock(&mdev->spi_lock);

	WARN_ONCE(err, "spi_sync failed (%d)\n", err);
	return err;
}

static inline void write_spi_reg(struct hdm_device *mdev, u8 reg, u32 v)
{
	u8 buf[8] = { SPI_WR | reg, 0, 0, 4, v >> 24, v >> 16, v >> 8, v };

	spi_hdm_xch(mdev, buf, NULL, sizeof(buf));
}

static inline u32 read_spi_reg(struct hdm_device *mdev, u8 reg)
{
	u8 buf[8] = { SPI_RD | reg, 0, 0, 4 };

	if (spi_hdm_xch(mdev, buf, buf, sizeof(buf)))
		return 0;

	return buf[4] << 24 | buf[5] << 16 | buf[6] << 8 | buf[7];
}

static inline bool ch_int_enabled(struct global_ch_int *gint, u8 int_mask_bit)
{
	return (gint->aim_mask & BIT(int_mask_bit)) == 0;
}

static inline void enable_ch_int(struct global_ch_int *gint, u8 int_mask_bit)
{
	clear_bit(int_mask_bit, &gint->aim_mask);
}

static inline void disable_ch_int(struct global_ch_int *gint, u8 int_mask_bit)
{
	set_bit(int_mask_bit, &gint->aim_mask);
}

static inline void flush_int_mask(struct global_ch_int *gint)
{
	struct hdm_device *mdev = container_of(gint, struct hdm_device, gint);

	if (gint->spi_mask == gint->aim_mask)
		return;

	gint->spi_mask = gint->aim_mask;
	write_spi_reg(mdev, GINT_CHSTS_ADDR, gint->aim_mask);
}

static void ch_init(struct hdm_channel *c, const struct channel_class *cl,
		    u8 *buf, u16 buf_size)
{
	mutex_lock(&c->cl_lock);
	c->cl = cl;
	c->spi_buf_sz = 0;
	c->buf = buf;
	c->buf_sz = buf_size;
	c->data = buf + 4;
	c->data_sz = 0;
	c->spi_data_gap = 0;
	mutex_unlock(&c->cl_lock);
}

static void xfer_mbos(struct hdm_device *mdev, struct hdm_channel *c)
{
	struct mbo *mbo, *t;

	list_for_each_entry_safe(mbo, t, &c->mbo_list, list) {
		if (!c->cl->xfer_mbo(mdev, c, mbo))
			break;

		list_del(&mbo->list);
		mbo->status = MBO_SUCCESS;
		mbo->complete(mbo);
	}
}

/* rx path { */

static void spi_to_buf(struct hdm_device *mdev, struct hdm_channel *c)
{
	u16 xch_size = ROUND_UP(4 + c->spi_buf_sz);

	if (!c->spi_buf_sz)
		return; /* nothing to get */

	if (c->data_sz)
		return; /* buffer is occupied */

	if (WARN_ONCE(xch_size > c->buf_sz, "%s: too small buffer (%d)\n",
		      c->cl->name, c->spi_buf_sz))
		return;

	c->buf[0] = c->cl->xch_cmd;
	c->buf[1] = 0;
	c->buf[2] = c->spi_buf_sz >> 8;
	c->buf[3] = c->spi_buf_sz;

	if (spi_hdm_xch(mdev, c->buf, c->buf, xch_size))
		return;

	c->data_sz = c->spi_buf_sz;
	c->spi_buf_sz = 0;
}

static bool pl_to_mbo(struct hdm_device *mdev, struct hdm_channel *c,
		      struct mbo *mbo)
{
	u32 msg_len = (c->data[0] << 8 | c->data[1]) + 2;

	if (WARN_ONCE(msg_len > c->data_sz,
		      "%s: length is out of buffer size (%d,%d)\n",
		      c->cl->name, msg_len, c->data_sz))
		return false;

	if (WARN_ONCE(msg_len > mbo->buffer_length,
		      "%s: too big message for mbo (%d)\n",
		      c->cl->name, msg_len))
		return false;

	memcpy(mbo->virt_address, c->data, msg_len);
	c->data_sz -= msg_len;
	if (!c->data_sz)
		c->data = c->buf + 4;
	else
		c->data += msg_len;
	mbo->processed_length = msg_len;
	return true;
}

static bool padded_pl_to_mbo(struct hdm_device *mdev, struct hdm_channel *c,
			     struct mbo *mbo)
{
	enum { MDP_HDR_LEN = 10, MDP_PAD = 3 };

	u32 msg_len = (c->data[0] << 8 | c->data[1]) + 2;
	u32 after_pad_len = msg_len - MDP_HDR_LEN;
	u32 padded_len = msg_len + MDP_PAD;

	if (WARN_ONCE(padded_len > c->data_sz,
		      "%s: length is out of buffer size (%d,%d)\n",
		      c->cl->name, padded_len, c->data_sz))
		return false;

	if (WARN_ONCE(msg_len > mbo->buffer_length,
		      "%s: too big message for mbo (%d)\n",
		      c->cl->name, msg_len))
		return false;

	memcpy(mbo->virt_address, c->data, MDP_HDR_LEN);
	c->data += MDP_HDR_LEN + MDP_PAD;
	memcpy(mbo->virt_address + MDP_HDR_LEN, c->data, after_pad_len);
	c->data_sz -= padded_len;
	if (!c->data_sz)
		c->data = c->buf + 4;
	else
		c->data += after_pad_len;
	mbo->processed_length = msg_len;
	return true;
}

/* implements xfer_mbo */
static bool ctrl_buf_to_mbo(struct hdm_device *mdev, struct hdm_channel *c,
			    struct mbo *mbo)
{
	if (!c->data_sz)
		return false;

	if (WARN_ONCE(c->data_sz < 2,
		      "%s: too small payload (%d)\n", c->cl->name, c->data_sz))
		return false;

	return pl_to_mbo(mdev, c, mbo);
}

/* implements xfer_mbo */
static bool async_buf_to_mbo(struct hdm_device *mdev, struct hdm_channel *c,
			     struct mbo *mbo)
{
	enum { MIN_HDR_LEN = 10 + 3 };

	u8 fph;

	if (!c->data_sz)
		return false;

	if (WARN_ONCE(c->data_sz < MIN_HDR_LEN,
		      "%s: too small payload (%d)\n", c->cl->name, c->data_sz))
		return false;

	fph = c->data[FPH_IDX];
	if (fph == MEP_FPH)
		return pl_to_mbo(mdev, c, mbo);

	if (WARN_ONCE(fph != MDP_FPH, "%s: false FPH (%u)\n", c->cl->name, fph))
		return false;

	return padded_pl_to_mbo(mdev, c, mbo);
}

/* implements channel_class.get_spi_buf_sz */
static u16 get_rx_buf_sz(struct hdm_device *mdev, struct hdm_channel *c)
{
	return (u16)read_spi_reg(mdev, c->cl->buf_info_cmd);
}

/* implements channel_class.xfer */
static void xfer_rx(struct hdm_device *mdev, struct hdm_channel *c)
{
	spi_to_buf(mdev, c);
	xfer_mbos(mdev, c);
	if (c->data_sz)
		disable_ch_int(&mdev->gint, c->cl->int_mask_bit);
	else
		enable_ch_int(&mdev->gint, c->cl->int_mask_bit);
}

/* } rx path */

/* tx path { */

/* implements xfer_mbo */
static bool mbo_to_buf(struct hdm_device *mdev, struct hdm_channel *c,
		       struct mbo *mbo)
{
	u32 msg_len = mbo->buffer_length;
	u16 spi_data_sz = c->data_sz + c->spi_data_gap + msg_len + EOP_SIZE;

	if (spi_data_sz > c->buf_sz - 4) {
		WARN_ONCE(c->data_sz == 0,
			  "%s: too big message (%d)\n", c->cl->name, msg_len);
		return false;
	}

	if (c->cl->type == MOST_CH_CONTROL && c->data_sz)
		return false;

	memcpy(c->data + c->data_sz, mbo->virt_address, msg_len);
	c->data_sz += msg_len;
	c->spi_data_gap += EOP_SIZE;
	mbo->processed_length = msg_len;
	return true;
}

static void buf_to_spi(struct hdm_device *mdev, struct hdm_channel *c)
{
	if (!c->data_sz)
		return;

	if (!c->spi_buf_sz)
		return;

	c->buf[0] = c->cl->xch_cmd;
	c->buf[1] = 0;
	c->buf[2] = c->data_sz >> 8;
	c->buf[3] = c->data_sz;

	if (spi_hdm_xch(mdev, c->buf, NULL, ROUND_UP(4 + c->data_sz)))
		return;

	c->spi_buf_sz = 0;
	c->data_sz = 0;
	c->spi_data_gap = 0;
}

/* implements channel_class.get_spi_buf_sz */
static u16 get_tx_buf_sz(struct hdm_device *mdev, struct hdm_channel *c)
{
	/*
	 * After the interrupt, free space in the spi tx buffer
	 * is big enough to transfer the full software buffer which size
	 * is not bigger than the threshold for the tx interrupt.
	 *
	 * Any non-zero value signals the space avalability.
	 */
	return 1;
}

/* implements channel_class.xfer */
static void xfer_tx(struct hdm_device *mdev, struct hdm_channel *c)
{
	buf_to_spi(mdev, c);
	xfer_mbos(mdev, c);
	if (c->data_sz)
		enable_ch_int(&mdev->gint, c->cl->int_mask_bit);
	else
		disable_ch_int(&mdev->gint, c->cl->int_mask_bit);
}

/* } tx path */

/* Driver Control Interface { */

static inline void dci_ctrl_write(struct dci_ctrl_reg *dci_reg, u32 cmd)
{
	struct hdm_device *mdev;

	cmd |= DCI_CTRL_ERRDM_BM | DCI_CTRL_ONTFM_BM | DCI_CTRL_CMDDONEM_BM;
	mdev = container_of(dci_reg, struct hdm_device, dci_reg);
	write_spi_reg(mdev, DCI_CTRL_ADDR, cmd & ~dci_reg->active_int);
}

static inline struct dci_job *first_dci_job(struct list_head *list)
{
	return list_first_entry(list, struct dci_job, list);
}

static const u16 zero_val[8];

static void dci_service_cmd(struct hdm_device *mdev, u32 dci_cmd);
static void dci_service_ntf(struct hdm_device *mdev, u32 dci_cmd);
static void dci_request(struct hdm_device *mdev);

static void add_dci_job(struct hdm_device *mdev, u8 cmd, const u16 *reg,
			const u16 *val,
			void (*complete_fn)(struct hdm_device *, int *),
			int *regs_num)
{
	struct dci_job *job;

	if (WARN_ONCE(list_empty(&mdev->dci_jobs_heap), "dci jobs overflow\n"))
		return;

	job = first_dci_job(&mdev->dci_jobs_heap);
	job->cmd = cmd;
	job->reg = reg;
	job->val = val;
	job->complete_fn = complete_fn;
	job->regs_num = regs_num;
	list_move(&job->list, &mdev->active_dci_jobs);
}

static void start_dci_request(struct hdm_device *mdev)
{
	if (mdev->service_dci == dci_service_cmd)
		return;

	mdev->service_dci = dci_service_cmd;
	mdev->dci_reg.active_int = DCI_CTRL_ERRDM_BM | DCI_CTRL_CMDDONEM_BM;
	dci_ctrl_write(&mdev->dci_reg, 0);

	dci_request(mdev);
}

static inline void add_dci_read(struct hdm_device *mdev, const u16 *reg,
				void (*complete_fn)(struct hdm_device *, int *))
{
	add_dci_job(mdev, DCI_READ_REQ, reg, zero_val, complete_fn, NULL);
	start_dci_request(mdev);
}

static inline void add_dci_write(struct hdm_device *mdev, const u16 *reg,
				 const u16 *val)
{
	add_dci_job(mdev, DCI_WRITE_REQ, reg, val, NULL, NULL);
	start_dci_request(mdev);
}

static void dci_request(struct hdm_device *mdev)
{
	struct dci_job *job = first_dci_job(&mdev->active_dci_jobs);
	const u16 *reg = job->reg;
	const u16 *val = job->val;
	u8 *const buf = mdev->dci_buf;
	u16 data_sz = 0;

	for (; *reg; reg++, val++) {
		buf[data_sz + 6] = *reg >> 8;
		buf[data_sz + 7] = *reg;
		buf[data_sz + 8] = *val >> 8;
		buf[data_sz + 9] = *val;
		data_sz += 4;
	}

	if (job->regs_num)
		*job->regs_num = data_sz / 4;

	buf[data_sz + 6] = 0;
	buf[data_sz + 7] = 0;

	buf[4] = job->cmd;
	buf[5] = data_sz / 4;
	data_sz += 2;

	buf[0] = DCI_ADDR | SPI_WR;
	buf[1] = 0;
	buf[2] = data_sz >> 8;
	buf[3] = data_sz;

	spi_hdm_xch(mdev, buf, NULL, ROUND_UP(4 + data_sz));
	dci_ctrl_write(&mdev->dci_reg, DCI_CTRL_CMDREQ_BM);
}

static bool dci_get_ack(struct hdm_device *mdev, u16 val[], u8 num)
{
	u8 *const buf = mdev->dci_buf;
	u16 data_sz = num * 4 + 2;
	u16 i;

	buf[0] = DCI_ADDR | SPI_RD;
	buf[1] = 0;
	buf[2] = data_sz >> 8;
	buf[3] = data_sz;

	memset(buf + 4, 0, data_sz + 2);

	if (spi_hdm_xch(mdev, buf, buf, ROUND_UP(4 + data_sz)))
		return false;

	for (i = 0; i < num; i++)
		val[i] = buf[i * 4 + 8] << 8 | buf[i * 4 + 9];

	return true;
}

#define DCI_REG_EVENTS(ch)  (0x3100 | (ch##_DCI_CH_IDX) << 4)
#define DCI_REG_CONFIG(ch)  (0x3102 | (ch##_DCI_CH_IDX) << 4)
#define DCI_REG_ROUTE_STATE(ch)  (0x3103 | (ch##_DCI_CH_IDX) << 4)
#define DCI_REG_DATA_TYPE(ch)  (0x3104 | (ch##_DCI_CH_IDX) << 4)
#define DCI_REG_ONOFF_STATE(ch)  (0x3107 | (ch##_DCI_CH_IDX) << 4)

static const u16 net_params[8] = {
	0x145, 0x146, 0x147,
	DCI_REG_ROUTE_STATE(ATX),
	DCI_REG_ROUTE_STATE(ARX),
};

static const u16 ntf_clear[8] = {
	0x3000,
	DCI_REG_EVENTS(ATX),
	DCI_REG_EVENTS(ARX),
};

static const u16 ntf_clear_val[8] = {
	BIT(2), /* eui48 */
	BIT(1) | BIT(2), /* sync needed | route state */
	BIT(1) | BIT(2), /* sync needed | route state */
};

static const u16 atx_cfg[8] = { DCI_REG_CONFIG(ATX) };
static const u16 arx_cfg[8] = { DCI_REG_CONFIG(ARX) };

static const u16 set_cfg_val[8] = { BIT(0) | BIT(1) | BIT(2) };

static const u16 dci_notify[8] = { 0x3002 };
static const u16 notify_eui48_val[8] = { BIT(2) };

static void net_params_read_complete(struct hdm_device *mdev, int *regs_num)
{
	u16 val[5];

	if (!dci_get_ack(mdev, val, ARRAY_SIZE(val)))
		return;

	mdev->mac[0] = val[0] >> 8;
	mdev->mac[1] = val[0];
	mdev->mac[2] = val[1] >> 8;
	mdev->mac[3] = val[1];
	mdev->mac[4] = val[2] >> 8;
	mdev->mac[5] = val[2];

	mdev->link_stat = val[3] == 1 && val[4] == 1;

	mutex_lock(&mdev->ni_mt);
	if (mdev->on_netinfo)
		mdev->on_netinfo(&mdev->most_iface, mdev->link_stat, mdev->mac);
	mutex_unlock(&mdev->ni_mt);
}

static void dci_service_err(struct hdm_device *mdev, u32 dci_ctrl)
{
	u32 error = dci_ctrl & DCI_CTRL_ERRD_BM;

	if (!error)
		return;

	dev_warn(&mdev->spi->dev, "%s(): dci critical error\n", __func__);
	dci_ctrl_write(&mdev->dci_reg, error);
}

/* implements hdm_device.service_dci, mdev->dci_mt is locked */
static void dci_service_ntf(struct hdm_device *mdev, u32 dci_ctrl)
{
	u32 ntf = dci_ctrl & DCI_CTRL_ONTF_BM;

	if (!ntf)
		return;

	dci_ctrl_write(&mdev->dci_reg, ntf);
	add_dci_write(mdev, ntf_clear, ntf_clear_val);
	add_dci_read(mdev, net_params, net_params_read_complete);
}

/* implements hdm_device.service_dci, mdev->dci_mt is locked */
static void dci_service_cmd(struct hdm_device *mdev, u32 dci_ctrl)
{
	u32 cmd_done = dci_ctrl & DCI_CTRL_CMDDONE_BM;
	struct dci_job *job;

	if (!cmd_done)
		return;

	job = first_dci_job(&mdev->active_dci_jobs);
	if (job->complete_fn)
		job->complete_fn(mdev, job->regs_num);

	list_move(&job->list, &mdev->dci_jobs_heap);

	if (!list_empty(&mdev->active_dci_jobs)) {
		dci_ctrl_write(&mdev->dci_reg, cmd_done);
		dci_request(mdev);
	} else {
		mdev->service_dci = dci_service_ntf;
		mdev->dci_reg.active_int = DCI_CTRL_ERRDM_BM | DCI_CTRL_ONTFM_BM;
		dci_ctrl_write(&mdev->dci_reg, cmd_done);
	}
}

/* } Driver Control Interface */

/* sysfs { */

struct dci_attr;

struct dci_attr_ {
	struct attribute attr;
	ssize_t (*show)(struct hdm_device *mdev, struct dci_attr *attr,
			char *buf);
	ssize_t (*store)(struct hdm_device *mdev, struct dci_attr *attr,
			 const char *buf, size_t count);
};

struct dci_attr {
	struct dci_attr_ a;
	u16 reg[8];
};

struct dci_wait_param {
	struct completion compl;
	int regs_num;
	u16 val[8];
};

static int get_xvalue(unsigned char x)
{
	if (x >= '0' && x <= '9')
		return x - '0';

	x = tolower(x);
	if (x >= 'a' && x <= 'f')
		return x - 'a' + 10;

	return -EINVAL;
}

static inline int dbg_read_u16(const char *buf, size_t count, u16 *res)
{
	int err;
	u16 val;

	/*
	 * kstrtouint() expects '\0' at the end of the buffer,
	 * that is put by configfs_write_file()
	 */
	if (WARN_ONCE(buf[count] != 0, "bad buffer for xx_store\n"))
		return -EFAULT;

	err = kstrtou16(buf, 0, &val);
	if (err)
		return err;

	*res = val;
	return 0;
}

static void dci_read_complete(struct hdm_device *mdev, int *regs_num)
{
	struct dci_wait_param *p;
	int i;

	p = container_of(regs_num, struct dci_wait_param, regs_num);
	if (!dci_get_ack(mdev, p->val, p->regs_num)) {
		for (i = 0; i < p->regs_num; i++)
			p->val[i] = 0xFFFF;
	}
	complete(&p->compl);
}

static void dci_write_complete(struct hdm_device *mdev, int *regs_num)
{
	struct dci_wait_param *p;

	p = container_of(regs_num, struct dci_wait_param, regs_num);
	complete(&p->compl);
}

static int dci_wait(struct completion *compl)
{
	long ret = wait_for_completion_interruptible_timeout(
			compl, msecs_to_jiffies(1000));

	if (WARN_ONCE(!ret, "%s(): timeout\n", __func__))
		return -EBUSY;

	return ret < 0 ? ret : 0;
}

static int dci_read_sync(struct dci_wait_param *p, struct hdm_device *mdev,
			 const u16 *reg)
{
	init_completion(&p->compl);

	mutex_lock(&mdev->dci_mt);
	add_dci_job(mdev, DCI_READ_REQ, reg, zero_val, dci_read_complete,
		    &p->regs_num);
	start_dci_request(mdev);
	mutex_unlock(&mdev->dci_mt);

	return dci_wait(&p->compl);
}

static int dci_write_sync(struct dci_wait_param *p, struct hdm_device *mdev,
			  const u16 *reg)
{
	init_completion(&p->compl);

	mutex_lock(&mdev->dci_mt);
	add_dci_job(mdev, DCI_WRITE_REQ, reg, p->val, dci_write_complete,
		    &p->regs_num);
	start_dci_request(mdev);
	mutex_unlock(&mdev->dci_mt);

	return dci_wait(&p->compl);
}

static ssize_t dci_show_u16(struct hdm_device *mdev, struct dci_attr *attr,
			    char *buf)
{
	struct dci_wait_param p;
	int ret = dci_read_sync(&p, mdev, attr->reg);

	if (ret)
		return ret;

	return sprintf(buf, "0x%04X\n", p.val[0]);
}

static ssize_t dci_store_u16(struct hdm_device *mdev, struct dci_attr *attr,
			     const char *buf, size_t count)
{
	int ret;
	struct dci_wait_param p;
	int err = dbg_read_u16(buf, count, &p.val[0]);

	if (err)
		return err;

	ret = dci_write_sync(&p, mdev, attr->reg);
	if (ret)
		return ret;

	return count;
}

static ssize_t packet_hash_show(struct hdm_device *mdev, struct dci_attr *attr,
				char *buf)
{
	struct dci_wait_param p;
	int ret = dci_read_sync(&p, mdev, attr->reg);

	if (ret)
		return ret;

	return sprintf(buf, "%04X %04X %04X %04X\n",
		       p.val[0], p.val[1], p.val[2], p.val[3]);
}

static ssize_t packet_hash_store(struct hdm_device *mdev, struct dci_attr *attr,
				 const char *buf, size_t count)
{
	int ret;
	struct dci_wait_param p;
	size_t rest = count;
	int i, v0, v1, v2, v3;

	for (i = 0; i < 4; i++) {
		if (rest < 4)
			return -EINVAL;

		v0 = get_xvalue(buf[0]);
		v1 = get_xvalue(buf[1]);
		v2 = get_xvalue(buf[2]);
		v3 = get_xvalue(buf[3]);
		if (v0 < 0 || v1 < 0 || v2 < 0 || v3 < 0)
			return -EINVAL;

		p.val[i] = v0 << 12 | v1 << 8 | v2 << 4 | v3;
		buf += 4;
		rest -= 4;
		/* allow delimiter behind the table to get simpler code */
		if (rest && *buf == ' ') {
			buf++;
			rest--;
		}
	}
	if (rest != 0 && *buf != '\n')
		return -EINVAL;

	ret = dci_write_sync(&p, mdev, attr->reg);
	if (ret)
		return ret;

	return count;
}

static ssize_t packet_eui48_show(struct hdm_device *mdev, struct dci_attr *attr,
				 char *buf)
{
	struct dci_wait_param p;
	int ret = dci_read_sync(&p, mdev, attr->reg);

	if (ret)
		return ret;

	return sprintf(buf, "%02X-%02X-%02X-%02X-%02X-%02X\n",
		       p.val[0] >> 8, p.val[0] & 255,
		       p.val[1] >> 8, p.val[1] & 255,
		       p.val[2] >> 8, p.val[2] & 255);
}

static ssize_t packet_eui48_store(struct hdm_device *mdev,
				  struct dci_attr *attr,
				  const char *buf, size_t count)
{
	int ret;
	struct dci_wait_param p;
	size_t rest = count;
	int i, v0, v1;

	for (i = 0; i < ETH_ALEN; i++) {
		if (rest < 2)
			return -EINVAL;

		v0 = get_xvalue(buf[0]);
		v1 = get_xvalue(buf[1]);
		if (v0 < 0 || v1 < 0)
			return -EINVAL;

		p.val[i / 2] = p.val[i / 2] << 8 | v0 << 4 | v1;
		buf += 2;
		rest -= 2;
		/* allow delimiter behind the address to get simpler code */
		if (rest && *buf == '-') {
			buf++;
			rest--;
		}
	}
	if (rest != 0 && *buf != '\n')
		return -EINVAL;

	ret = dci_write_sync(&p, mdev, attr->reg);
	if (ret)
		return ret;

	return count;
}

#define U16_ATTR_RW(name) __ATTR(name, 0644, dci_show_u16, dci_store_u16)
#define U16_ATTR_RO(name) __ATTR(name, 0444, dci_show_u16, NULL)

/*
 * 0x101 R   PacketBW
 * 0x102 R   NodeAddress
 * 0x103 R   NodePosition
 * 0x120 R   LastResetReason
 * 0x140 R/W PacketFilterMode
 *
 * 0x141 R/W PacketHash_63to48
 * 0x142 R/W PacketHash_47to32
 * 0x143 R/W PacketHash_31to16
 * 0x144 R/W PacketHash_15to0
 *
 * 0x145 R/W PacketEUI48_47to32
 * 0x146 R/W PacketEUI48_31to16
 * 0x147 R/W PacketEUI48_15to0
 *
 * 0x148 R/W PacketLLRTime
 */

static struct dci_attr dci_attrs[] = {
	{ .a = U16_ATTR_RO(packet_bw), .reg = { 0x101 } },
	{ .a = U16_ATTR_RO(node_address), .reg = { 0x102 } },
	{ .a = U16_ATTR_RO(node_position), .reg = { 0x103 } },
	{ .a = U16_ATTR_RO(last_reset_reason), .reg = { 0x120 } },
	{ .a = U16_ATTR_RW(packet_filter_mode), .reg = { 0x140 } },
	{ .a = __ATTR_RW(packet_hash), .reg = { 0x141, 0x142, 0x143, 0x144 } },
	{ .a = __ATTR_RW(packet_eui48), .reg = { 0x145, 0x146, 0x147 } },
	{ .a = U16_ATTR_RW(packet_llr_time), .reg = { 0x148 } },
};

static struct attribute *bus_default_attrs[] = {
	&dci_attrs[0].a.attr,
	&dci_attrs[1].a.attr,
	&dci_attrs[2].a.attr,
	&dci_attrs[3].a.attr,
	&dci_attrs[4].a.attr,
	&dci_attrs[5].a.attr,
	&dci_attrs[6].a.attr,
	&dci_attrs[7].a.attr,
	NULL,
};

static const struct attribute_group bus_attr_group = {
	.attrs = bus_default_attrs,
};

static void bus_kobj_release(struct kobject *kobj)
{
}

static ssize_t bus_kobj_attr_show(struct kobject *kobj, struct attribute *attr,
				  char *buf)
{
	struct dci_attr *xattr = container_of(attr, struct dci_attr, a.attr);
	struct hdm_device *mdev;

	if (!xattr->a.show)
		return -EIO;

	mdev = container_of(kobj, struct hdm_device, dci.kobj_group);
	return xattr->a.show(mdev, xattr, buf);
}

static ssize_t bus_kobj_attr_store(struct kobject *kobj, struct attribute *attr,
				   const char *buf, size_t count)
{
	struct dci_attr *xattr = container_of(attr, struct dci_attr, a.attr);
	struct hdm_device *mdev;

	if (!xattr->a.store)
		return -EIO;

	mdev = container_of(kobj, struct hdm_device, dci.kobj_group);
	return xattr->a.store(mdev, xattr, buf, count);
}

static struct sysfs_ops const bus_kobj_sysfs_ops = {
	.show = bus_kobj_attr_show,
	.store = bus_kobj_attr_store,
};

static struct kobj_type bus_ktype = {
	.release = bus_kobj_release,
	.sysfs_ops = &bus_kobj_sysfs_ops,
};

/* } sysfs */

static const struct channel_class ch_class[CH_NUM] = {
	[CH_ASYNC_TX] = {
		.name = "atx",
		.dir = MOST_CH_TX,
		.type = MOST_CH_ASYNC,
		.xch_cmd = SPI_WR | ASYNC_ADDR,
		.buf_info_cmd = 0x4,
		.int_status_bit = GINT_CHSTS_ARIS_B,
		.int_mask_bit = GINT_CHSTS_ARISM_B,
		.get_spi_buf_sz = get_tx_buf_sz,
		.xfer = xfer_tx,
		.xfer_mbo = mbo_to_buf,
	},
	[CH_ASYNC_RX] = {
		.name = "arx",
		.dir = MOST_CH_RX,
		.type = MOST_CH_ASYNC,
		.xch_cmd = SPI_RD | ASYNC_ADDR,
		.buf_info_cmd = 0x5,
		.int_status_bit = GINT_CHSTS_ATIS_B,
		.int_mask_bit = GINT_CHSTS_ATISM_B,
		.get_spi_buf_sz = get_rx_buf_sz,
		.xfer = xfer_rx,
		.xfer_mbo = async_buf_to_mbo,
	},
	[CH_CTRL_TX] = {
		.name = "ctx",
		.dir = MOST_CH_TX,
		.type = MOST_CH_CONTROL,
		.xch_cmd = SPI_WR | CTRL_ADDR,
		.buf_info_cmd = 0x6,
		.int_status_bit = GINT_CHSTS_CRIS_B,
		.int_mask_bit = GINT_CHSTS_CRISM_B,
		.get_spi_buf_sz = get_tx_buf_sz,
		.xfer = xfer_tx,
		.xfer_mbo = mbo_to_buf,
	},
	[CH_CTRL_RX] = {
		.name = "crx",
		.dir = MOST_CH_RX,
		.type = MOST_CH_CONTROL,
		.xch_cmd = SPI_RD | CTRL_ADDR,
		.buf_info_cmd = 0x7,
		.int_status_bit = GINT_CHSTS_CTIS_B,
		.int_mask_bit = GINT_CHSTS_CTISM_B,
		.get_spi_buf_sz = get_rx_buf_sz,
		.xfer = xfer_rx,
		.xfer_mbo = ctrl_buf_to_mbo,
	},
};

static int configure_channel(struct most_interface *most_iface, int ch_idx,
			     struct most_channel_config *ccfg)
{
	struct hdm_device *mdev = to_device(most_iface);
	struct hdm_channel *c = mdev->ch + ch_idx;
	const struct channel_class *cl = ch_class + ch_idx;

	if (ccfg->data_type != cl->type) {
		dev_err(&mdev->spi->dev, "%s: wrong data type\n", cl->name);
		return -EINVAL;
	}

	if (ccfg->direction != cl->dir) {
		dev_err(&mdev->spi->dev, "%s: wrong direction\n", cl->name);
		return -EINVAL;
	}

	switch (ch_idx) {
	case CH_ASYNC_TX:
		ch_init(c, cl, mdev->atx_buf, sizeof(mdev->atx_buf));
		disable_ch_int(&mdev->gint, cl->int_mask_bit);

		mutex_lock(&mdev->dci_mt);
		add_dci_write(mdev, atx_cfg, set_cfg_val);
		mutex_unlock(&mdev->dci_mt);
		break;
	case CH_ASYNC_RX:
		ch_init(c, cl, mdev->arx_buf, sizeof(mdev->arx_buf));
		enable_ch_int(&mdev->gint, cl->int_mask_bit);

		mutex_lock(&mdev->dci_mt);
		add_dci_write(mdev, arx_cfg, set_cfg_val);
		mutex_unlock(&mdev->dci_mt);
		break;
	case CH_CTRL_TX:
		ch_init(c, cl, mdev->ctx_buf, sizeof(mdev->ctx_buf));
		disable_ch_int(&mdev->gint, cl->int_mask_bit);
		break;
	case CH_CTRL_RX:
		ch_init(c, cl, mdev->crx_buf, sizeof(mdev->crx_buf));
		enable_ch_int(&mdev->gint, cl->int_mask_bit);
		break;
	default:
		dev_err(&mdev->spi->dev, "%s: configure failed, bad channel index: %d\n",
			cl->name, ch_idx);
		return -EINVAL;
	}

	flush_int_mask(&mdev->gint);
	return 0;
}

static int enqueue(struct most_interface *most_iface, int ch_idx,
		   struct mbo *mbo)
{
	struct hdm_device *mdev = to_device(most_iface);
	struct hdm_channel *c = mdev->ch + ch_idx;

	mutex_lock(&c->cl_lock);
	list_add_tail(&mbo->list, &c->mbo_list);
	c->cl->xfer(mdev, c);
	flush_int_mask(&mdev->gint);
	mutex_unlock(&c->cl_lock);
	return 0;
}

static int poison_channel(struct most_interface *most_iface, int ch_idx)
{
	struct hdm_device *mdev = to_device(most_iface);
	struct hdm_channel *c = mdev->ch + ch_idx;
	struct mbo *mbo, *t;

	if (ch_idx == CH_ASYNC_TX) {
		mutex_lock(&mdev->dci_mt);
		add_dci_write(mdev, dci_notify, zero_val);
		add_dci_write(mdev, atx_cfg, zero_val);
		mutex_unlock(&mdev->dci_mt);
	} else if (ch_idx == CH_ASYNC_RX) {
		mutex_lock(&mdev->dci_mt);
		add_dci_write(mdev, dci_notify, zero_val);
		add_dci_write(mdev, arx_cfg, zero_val);
		mutex_unlock(&mdev->dci_mt);
	}

	mutex_lock(&c->cl_lock);
	disable_ch_int(&mdev->gint, c->cl->int_mask_bit);
	flush_int_mask(&mdev->gint);
	c->cl = NULL;
	list_for_each_entry_safe(mbo, t, &c->mbo_list, list) {
		list_del(&mbo->list);
		mbo->processed_length = 0;
		mbo->status = MBO_E_CLOSE;
		mbo->complete(mbo);
	}
	mutex_unlock(&c->cl_lock);

	return 0;
}

static void request_netinfo(struct most_interface *most_iface, int ch_idx,
			    void (*on_netinfo)(struct most_interface *,
					       unsigned char,
					       unsigned char *))
{
	struct hdm_device *mdev = to_device(most_iface);

	mutex_lock(&mdev->ni_mt);
	mdev->on_netinfo = on_netinfo;
	mutex_unlock(&mdev->ni_mt);

	if (!on_netinfo)
		return;

	mutex_lock(&mdev->dci_mt);
	add_dci_read(mdev, net_params, net_params_read_complete);
	add_dci_write(mdev, dci_notify, notify_eui48_val);
	mutex_unlock(&mdev->dci_mt);
}

static void sint_work_fn(struct work_struct *ws)
{
	int i;
	struct hdm_device *mdev =
		container_of(ws, struct hdm_device, sint_work);
	u32 status_reg = read_spi_reg(mdev, GINT_CHSTS_ADDR);

	mdev->gint.spi_mask = status_reg & GINT_CHSTS_INTM;

	if ((status_reg & BIT(GINT_CHSTS_DCITS_B))) {
		u32 dci_ctrl = read_spi_reg(mdev, DCI_CTRL_ADDR);

		dci_service_err(mdev, dci_ctrl);

		mutex_lock(&mdev->dci_mt);
		mdev->service_dci(mdev, dci_ctrl);
		mutex_unlock(&mdev->dci_mt);
	}

	for (i = 0; i < CH_NUM; i++) {
		struct hdm_channel *c = mdev->ch + i;
		const struct channel_class *cl = ch_class + i;

		if (!(status_reg & BIT(cl->int_status_bit)))
			continue; /* not for this channel */

		mutex_lock(&c->cl_lock);
		if (c->cl) {
			c->spi_buf_sz = cl->get_spi_buf_sz(mdev, c);
			cl->xfer(mdev, c);
		}
		mutex_unlock(&c->cl_lock);
	}
	flush_int_mask(&mdev->gint);
	enable_irq(mdev->irq);
}

static irqreturn_t sint_isr(int irq, void *dev)
{
	struct hdm_device *mdev = dev;

	disable_irq_nosync(irq);
	schedule_work(&mdev->sint_work);
	return IRQ_HANDLED;
}

static int spi_hdm_probe(struct spi_device *spi)
{
	struct hdm_device *mdev;
	struct kobject *kobj;
	int err, i;

	mdev = devm_kzalloc(&spi->dev, sizeof(*mdev), GFP_KERNEL);
	if (!mdev)
		return -ENOMEM;

	mutex_init(&mdev->spi_lock);
	mdev->spi = spi_dev_get(spi);
	spi_set_drvdata(spi, mdev);

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;

	err = spi_setup(spi);
	if (err) {
		dev_err(&spi->dev, "spi_setup failed, err=%d\n", err);
		goto err_put_spi;
	}

	for (i = 0; i < CH_NUM; i++) {
		struct most_channel_capability *cap = mdev->capabilities + i;

		mutex_init(&mdev->ch[i].cl_lock);
		INIT_LIST_HEAD(&mdev->ch[i].mbo_list);

		cap->name_suffix = ch_class[i].name;
		cap->direction = ch_class[i].dir;
		cap->data_type = ch_class[i].type;
	}

	mdev->most_iface.interface = ITYPE_MEDIALB_DIM2;
	mdev->most_iface.description = "spi";
	mdev->most_iface.num_channels = CH_NUM;
	mdev->most_iface.channel_vector = mdev->capabilities;
	mdev->most_iface.configure = configure_channel;
	mdev->most_iface.enqueue = enqueue;
	mdev->most_iface.poison_channel = poison_channel;
	mdev->most_iface.request_netinfo = request_netinfo;

	INIT_WORK(&mdev->sint_work, sint_work_fn);

	 /* enable SINT, mask all interrupts */
	mdev->gint.aim_mask = GINT_CHSTS_INTM;
	flush_int_mask(&mdev->gint);

	/*
	 * no interrupt delay,
	 * threshold: 64 bytes for ctx and 1536 bytes for atx
	 */
	write_spi_reg(mdev, DR_CONFIG_ADDR, 5u << 28 | 9u << 24);

	mutex_init(&mdev->dci_mt);
	mutex_init(&mdev->ni_mt);
	mdev->service_dci = dci_service_ntf;
	mdev->dci_reg.active_int = DCI_CTRL_ERRDM_BM | DCI_CTRL_ONTFM_BM;
	dci_ctrl_write(&mdev->dci_reg, 0);

	INIT_LIST_HEAD(&mdev->active_dci_jobs);
	INIT_LIST_HEAD(&mdev->dci_jobs_heap);
	for (i = 0; i < ARRAY_SIZE(mdev->dci_jobs); i++)
		list_add_tail(&mdev->dci_jobs[i].list, &mdev->dci_jobs_heap);

	mdev->irq = irq_of_parse_and_map(spi->dev.of_node, 0);
	if (mdev->irq <= 0) {
		pr_err("failed to get IRQ\n");
		err = -ENODEV;
		goto err_put_spi;
	}

	err = devm_request_irq(&spi->dev, mdev->irq, sint_isr, 0,
			       "spi-hdm", mdev);
	if (err) {
		pr_err("failed to request IRQ: %d, err: %d\n", mdev->irq, err);
		goto err_put_spi;
	}

	kobj = most_register_interface(&mdev->most_iface);
	if (IS_ERR(kobj)) {
		dev_err(&spi->dev, "failed to register MOST interface\n");
		err = PTR_ERR(kobj);
		goto err_put_spi;
	}

	kobject_init(&mdev->dci.kobj_group, &bus_ktype);
	err = kobject_add(&mdev->dci.kobj_group, kobj, "dci");
	if (err) {
		pr_err("kobject_add() failed: %d\n", err);
		goto err_unreg_iface;
	}

	err = sysfs_create_group(&mdev->dci.kobj_group, &bus_attr_group);
	if (err) {
		pr_err("sysfs_create_group() failed: %d\n", err);
		goto err_put_kobject;
	}

	enable_ch_int(&mdev->gint, GINT_CHSTS_DCITSM_B);
	flush_int_mask(&mdev->gint);
	return 0;

err_put_kobject:
	kobject_put(&mdev->dci.kobj_group);

err_unreg_iface:
	most_deregister_interface(&mdev->most_iface);

err_put_spi:
	spi_dev_put(mdev->spi);
	return err;
}

static int spi_hdm_remove(struct spi_device *spi)
{
	struct hdm_device *mdev = spi_get_drvdata(spi);

	sysfs_remove_group(&mdev->dci.kobj_group, &bus_attr_group);
	kobject_put(&mdev->dci.kobj_group);
	most_deregister_interface(&mdev->most_iface);
	mdev->gint.aim_mask = GINT_CHSTS_INTM;
	flush_int_mask(&mdev->gint);
	cancel_work_sync(&mdev->sint_work);
	spi_dev_put(mdev->spi);
	return 0;
}

static const struct of_device_id pd_spi_of_match[] = {
	{ .compatible = "microchip,inic-spi-prot" },
	{}
};
MODULE_DEVICE_TABLE(of, pd_spi_of_match);

static struct spi_driver spi_hdm_driver = {
	.driver = {
		.name = "inic-spi-prot",
		.of_match_table = pd_spi_of_match,
	},
	.probe = spi_hdm_probe,
	.remove = spi_hdm_remove,
};

module_spi_driver(spi_hdm_driver);

MODULE_AUTHOR("Andrey Shvetsov <andrey.shvetsov@k2l.de>");
MODULE_DESCRIPTION("SPI protocol driver as a Hardware Dependent Module");
MODULE_LICENSE("GPL");
