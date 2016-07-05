/*
 * dim2_arwen_dt.c - Arwen Platform dependent module for MediaLB DIM2 Interface
 *
 * Copyright (C) 2015, Microchip Technology Germany II GmbH & Co. KG
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
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
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include "../dim2_hdm.h"
#include "../dim2_hal.h"

#define DRIVER_VERSION "1.0.0"

/* command line parameter to select clock speed */
static char *mlb_pin_cfg;
module_param(mlb_pin_cfg, charp, 0);
MODULE_PARM_DESC(mlb_pin_cfg, "MediaLB Pin Configuration");


struct dim2_arwen_platform_data {
	struct dim2_platform_data pdata;
	struct platform_device *pdev;
	struct device *dev;
	struct clk *clk_mlb3p;
	struct clk *clk_mlb6p;
	int clk_speed;
};

static const struct of_device_id dim2_arwen_dt_ids[] = {
	{ .compatible = "xlnx,axi4-os62420_3pin-1.00.a" },
	{},
};

static int init(struct dim2_platform_data *pdata, void *io_base, int clk_speed)
{
	struct dim2_arwen_platform_data *pd = pdata->priv;
	pd->clk_speed = clk_speed;
	return  0;
}

static void destroy(struct dim2_platform_data *pdata)
{
}


static struct dim2_arwen_platform_data arwen_pdata = {
	.pdata = {
		.init = init,
		.destroy = destroy,
		.priv = &arwen_pdata,
	},
};


static int dim2_dt_probe(struct platform_device *pdev_dt)
{
    struct platform_device *pdev;
    const struct of_device_id *match;
    struct resource res[2];
    int ret;
    int rc = 0;

    //memset(res, 0, sizeof(res));

    match = of_match_device(dim2_arwen_dt_ids, &pdev_dt->dev);

    if (!match)
    {
            pr_err("Could not find drivers node in device tree \n");
            ret = -EINVAL;
            goto out;
    }
	
    rc = of_address_to_resource(pdev_dt->dev.of_node, 0, &res[0]);
    if (rc) {
	  
	  pr_err("Could not read dim register address from device tree\n");
	  ret = -EFAULT;
	  goto out;
    }

    pr_info("Found MLB3 Pin IP at address %x\n",res[0].start);


    if (!of_irq_to_resource(pdev_dt->dev.of_node, 0, &res[1])) {
	   pr_err("Could not read interrupt number from device tree\n");
	  ret = -EFAULT;
	  goto out;
    }

    pr_info("Found interrupt number %d for MLB3 Pin IP\n",res[1].start);
    // struct resource res[] = {
            // {
                    // .start	= MLB_IOREG_BASE,
                    // .end	= MLB_IOREG_END,
                    // .flags	= IORESOURCE_MEM,
            // },
            // {
                    // .start	= MLB_AHB0_INT,
                    // .end	= MLB_AHB0_INT,
                    // .flags	= IORESOURCE_IRQ,
            // },
    // };

    if (arwen_pdata.pdev)
    {
            return -ENOMEM;
    }

    pdev = platform_device_alloc("medialb_dim2", 0);
    if (!pdev) {
            pr_err("Failed to allocate platform device\n");
            ret = -ENOMEM;
            goto out;
    }

    ret = platform_device_add_resources(pdev, res, ARRAY_SIZE(res));
    if (ret) {
            pr_err("Failed to add resources\n");
            goto out_free_pdev;
    }

    arwen_pdata.pdev = pdev;
    arwen_pdata.dev = &pdev_dt->dev;
    ret = platform_device_add_data(pdev, &arwen_pdata.pdata,
                                   sizeof arwen_pdata.pdata);
    if (ret) {
            pr_err("Failed to add platform data\n");
            goto out_free_pdev;
    }

    ret = platform_device_add(pdev);
    if (ret) {
            pr_err("Failed to add platform device\n");
            goto out_free_pdev;
    }

    return 0;

out_free_pdev:
	platform_device_put(pdev);
out:
	return ret;
}

static int dim2_dt_remove(struct platform_device *pdev_dt)
{
	platform_device_unregister(arwen_pdata.pdev);
	arwen_pdata.pdev = 0;
	return 0;
}

static struct platform_driver dim2_driver = {
	.probe = dim2_dt_probe,
	.remove = dim2_dt_remove,
	.driver = {
		.name = "dim2-dt-mlb3",
		.owner = THIS_MODULE,
		.of_match_table = dim2_arwen_dt_ids,
	},
};
MODULE_DEVICE_TABLE(of, dim2_arwen_dt_ids);

static int __init arwen_mlb_init(void)
{
	pr_info("arwen_mlb_init()\n");
	return platform_driver_register(&dim2_driver);
}

static void __exit arwen_mlb_exit(void)
{
	pr_info("arwen_mlb_exit()\n");
	platform_driver_unregister(&dim2_driver);
}

module_init(arwen_mlb_init);
module_exit(arwen_mlb_exit);
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_AUTHOR("Jain Roy Ambi <JainRoy.Ambi@microchip.com>");
MODULE_AUTHOR("Andrey Shvetsov <andrey.shvetsov@k2l.de>");
MODULE_AUTHOR("Robert Korn <robert.korn@microchip.com>");
MODULE_DESCRIPTION("Arwen MediaLB DIM2 3 PIN Configuration dt-friendly Platform Device");
