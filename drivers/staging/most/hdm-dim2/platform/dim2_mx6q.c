/*
 * dim2_mx6q.c - Platform device for MediaLB DIM2 interface
 * on Freescale IMX6Q
 *
 * Copyright (C) 2015-2017, Microchip Technology Germany II GmbH & Co. KG
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This file is licensed under GPLv2.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/printk.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>

#include "../dim2_hdm.h"
#include "../dim2_hal.h"

#define DIM2_IOREG_BASE  0x218C000
#define DIM2_IOREG_END   0x218CFFF
#define DIM2_AHB0_INT    149
#define DIM2_MLB_INT     85

#define REG_MLBPC1      0x38
#define MLBPC1_VAL      0x888

/* command line parameter to select the clock speed */
static char *clock_speed;
module_param(clock_speed, charp, 0000);
MODULE_PARM_DESC(clock_speed, "MediaLB Clock Speed");

struct dim2_platform_data_ext {
	struct dim2_platform_data pdata;
	struct platform_device *pdev;
	struct clk *clk_mlb3p;
	struct clk *clk_mlb6p;
};

static int init(struct dim2_platform_data *pdata, void *io_base);
static void destroy(struct dim2_platform_data *pdata);

static struct dim2_platform_data_ext pd = {
	.pdata = {
		.init = init,
		.destroy = destroy,
	}
};

static int init(struct dim2_platform_data *pdata, void *io_base)
{
	struct device *dev = &pd.pdev->dev;

	pd.clk_mlb3p = clk_get(dev, "mlb150_clk");
	if (IS_ERR_OR_NULL(pd.clk_mlb3p)) {
		pr_err("unable to get mlb clock\n");
		return -EFAULT;
	}
	clk_prepare_enable(pd.clk_mlb3p);

	if (pdata->clk_speed >= CLK_2048FS) { /* enable pll */
		pd.clk_mlb6p = clk_get(dev, "pll6");
		if (IS_ERR_OR_NULL(pd.clk_mlb6p)) {
			pr_err("unable to get mlb pll clock\n");
			clk_disable_unprepare(pd.clk_mlb3p);
			clk_put(pd.clk_mlb3p);
			return -EFAULT;
		}

		writel(MLBPC1_VAL, io_base + REG_MLBPC1);
		clk_prepare_enable(pd.clk_mlb6p);
	}

	return  0;
}

static void destroy(struct dim2_platform_data *pdata)
{
	if (pdata->clk_speed >= CLK_2048FS) {
		clk_disable_unprepare(pd.clk_mlb6p);
		clk_put(pd.clk_mlb6p);
	}

	clk_disable_unprepare(pd.clk_mlb3p);
	clk_put(pd.clk_mlb3p);
}

static int __init mlb_platform_init(void)
{
	struct platform_device *pdev;
	int ret;
	struct resource res[] = {
		{
			.flags = IORESOURCE_MEM,
			.start = DIM2_IOREG_BASE,
			.end = DIM2_IOREG_END,
		},
		{
			.flags = IORESOURCE_IRQ,
			.start = DIM2_AHB0_INT,
			.end = DIM2_AHB0_INT,
		},
		{
			.flags = IORESOURCE_IRQ,
			.start = DIM2_MLB_INT,
			.end = DIM2_MLB_INT,
		},
	};

	pr_info("%s()\n", __func__);

	if (!clock_speed) {
		pr_err("missing clock speed parameter\n");
		return -EFAULT;
	}

	if (!strcmp(clock_speed, "256fs"))
		pd.pdata.clk_speed = CLK_256FS;
	else if (!strcmp(clock_speed, "512fs"))
		pd.pdata.clk_speed = CLK_512FS;
	else if (!strcmp(clock_speed, "1024fs"))
		pd.pdata.clk_speed = CLK_1024FS;
	else if (!strcmp(clock_speed, "2048fs"))
		pd.pdata.clk_speed = CLK_2048FS;
	else if (!strcmp(clock_speed, "3072fs"))
		pd.pdata.clk_speed = CLK_3072FS;
	else if (!strcmp(clock_speed, "4096fs"))
		pd.pdata.clk_speed = CLK_4096FS;
	else if (!strcmp(clock_speed, "6144fs"))
		pd.pdata.clk_speed = CLK_6144FS;
	else if (!strcmp(clock_speed, "8192fs"))
		pd.pdata.clk_speed = CLK_8192FS;
	else {
		pr_err("bad clock speed parameter\n");
		return -EFAULT;
	}

	pr_info("clock speed: %s\n", clock_speed);

	pdev = platform_device_alloc("medialb_dim2", 0);
	if (!pdev) {
		pr_err("failed to allocate platform device\n");
		return -ENOMEM;
	}

	ret = platform_device_add_resources(pdev, res, ARRAY_SIZE(res));
	if (ret) {
		pr_err("failed to add resources\n");
		goto out_free_pdev;
	}

	pd.pdev = pdev;
	ret = platform_device_add_data(pdev, &pd.pdata, sizeof(pd.pdata));
	if (ret) {
		pr_err("failed to add platform data\n");
		goto out_free_pdev;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add platform device\n");
		goto out_free_pdev;
	}

	return 0;

out_free_pdev:
	platform_device_put(pdev);
	return ret;
}

static void __exit mlb_platform_exit(void)
{
	pr_info("%s()\n", __func__);
	platform_device_unregister(pd.pdev);
}

module_init(mlb_platform_init);
module_exit(mlb_platform_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Shvetsov <andrey.shvetsov@k2l.de>");
MODULE_DESCRIPTION("IMX6Q MediaLB DIM2 Platform Device");
