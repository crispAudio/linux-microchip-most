/*
 * dim2_mx6q_dt.c - Platform device for MediaLB DIM2 interface
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
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include "../dim2_hdm.h"
#include "../dim2_hal.h"

#define REG_MLBPC1      0x38
#define MLBPC1_VAL      0x888

/* command line parameter to select the clock speed */
static char *clock_speed;
module_param(clock_speed, charp, 0000);
MODULE_PARM_DESC(clock_speed, "MediaLB Clock Speed");

struct dim2_platform_data_ext {
	struct dim2_platform_data pdata;
	struct platform_device *pdev;
	struct device *dev;
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
	pd.clk_mlb3p = clk_get(pd.dev, "mlb");
	if (IS_ERR_OR_NULL(pd.clk_mlb3p)) {
		pr_err("unable to get mlb clock\n");
		return -EFAULT;
	}
	clk_prepare_enable(pd.clk_mlb3p);

	if (pdata->clk_speed >= CLK_2048FS) { /* enable pll */
		pd.clk_mlb6p = clk_get(pd.dev, "pll8_mlb");
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

static int dim2_dt_probe(struct platform_device *pdev_dt)
{
	struct device_node *const node = pdev_dt->dev.of_node;
	struct platform_device *pdev;
	struct resource res[3];
	int ret;

	enum dt_interrupts_order { MLB_INT_DT_IDX, AHB0_INT_DT_IDX };

	if (pd.pdev)
		return -ENOMEM;

	ret = of_address_to_resource(node, 0, &res[0]);
	if (ret) {
		pr_err("failed to get memory region\n");
		return ret;
	}

	if (!of_irq_to_resource(node, AHB0_INT_DT_IDX, &res[1])) {
		pr_err("failed to get ahb0_int resource\n");
		return -ENOENT;
	}

	if (!of_irq_to_resource(node, MLB_INT_DT_IDX, &res[2])) {
		pr_err("failed to get mlb_int resource\n");
		return -ENOENT;
	}

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
	pd.dev = &pdev_dt->dev;
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
	pd.pdev = 0;
	return ret;
}

static int dim2_dt_remove(struct platform_device *pdev_dt)
{
	platform_device_unregister(pd.pdev);
	pd.pdev = 0;
	return 0;
}

static const struct of_device_id dim2_imx_dt_ids[] = {
	{ .compatible = "fsl,imx6q-mlb150" },
	{},
};

static struct platform_driver dim2_driver = {
	.probe = dim2_dt_probe,
	.remove = dim2_dt_remove,
	.driver = {
		.name = "dim2-dt-stub-driver",
		.owner = THIS_MODULE,
		.of_match_table = dim2_imx_dt_ids,
	},
};

MODULE_DEVICE_TABLE(of, dim2_imx_dt_ids);

static int __init mlb_platform_init(void)
{
	pr_info("%s()\n", __func__);
	return platform_driver_register(&dim2_driver);
}

static void __exit mlb_platform_exit(void)
{
	pr_info("%s()\n", __func__);
	platform_driver_unregister(&dim2_driver);
}

module_init(mlb_platform_init);
module_exit(mlb_platform_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Shvetsov <andrey.shvetsov@k2l.de>");
MODULE_DESCRIPTION("IMX6Q MediaLB DIM2 dt-friendly Platform Device");
