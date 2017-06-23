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

#include "mostcore.h"

#define SPI_WR 0x00
#define SPI_RD 0x80

#define DR_CONFIG_ADDR 0x00
#define GINT_CHSTS_ADDR 0x01
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

struct global_ch_int {
	unsigned long spi_mask;
	unsigned long aim_mask;
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
		break;
	case CH_ASYNC_RX:
		ch_init(c, cl, mdev->arx_buf, sizeof(mdev->arx_buf));
		enable_ch_int(&mdev->gint, cl->int_mask_bit);
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

static void sint_work_fn(struct work_struct *ws)
{
	int i;
	struct hdm_device *mdev =
		container_of(ws, struct hdm_device, sint_work);
	u32 status_reg = read_spi_reg(mdev, GINT_CHSTS_ADDR);

	mdev->gint.spi_mask = status_reg & GINT_CHSTS_INTM;

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

	INIT_WORK(&mdev->sint_work, sint_work_fn);

	 /* enable SINT, mask all interrupts */
	mdev->gint.aim_mask = GINT_CHSTS_INTM;
	flush_int_mask(&mdev->gint);

	/*
	 * no interrupt delay,
	 * threshold: 64 bytes for ctx and 1536 bytes for atx
	 */
	write_spi_reg(mdev, DR_CONFIG_ADDR, 5u << 28 | 9u << 24);

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

	return 0;

err_put_spi:
	spi_dev_put(mdev->spi);
	return err;
}

static int spi_hdm_remove(struct spi_device *spi)
{
	struct hdm_device *mdev = spi_get_drvdata(spi);

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
