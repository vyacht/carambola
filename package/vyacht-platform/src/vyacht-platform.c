/*
 * vYacht module to set platform devices which is more convenient
 * from a loadable module than in the arch-platform file.
 *
 * Copyright (C) 2013 Bernd Ocklin <bernd@vyacht.net>
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include <linux/platform_device.h>

#include <linux/input.h>
#include <linux/gpio_keys.h>

static struct gpio_keys_button vyacht_platform_gpio_buttons[] __initdata = {
        {
	        .desc           = "RIN",
                .type           = EV_KEY,
                .code           = BTN_0,
                .debounce_interval = 60,
                .gpio           = 14,
                .active_low     = 1
        },
        {
                .desc           = "DCD",
                .type           = EV_KEY,
                .code           = BTN_1,
                .debounce_interval = 60,
                .gpio           = 12,
                .active_low     = 1
        },
};

static struct gpio_keys_platform_data vyacht_platform_gpio_keys_data = {
	.buttons 	= vyacht_platform_gpio_buttons,
	.nbuttons 	= ARRAY_SIZE(vyacht_platform_gpio_buttons),
	.name           = "reset-keys",
};

static struct platform_device vyacht_platform_gpio_btn = {
	.name           = "gpio-keys",
	.id           	= -1,
	.dev     	= {
		.platform_data  = &vyacht_platform_gpio_keys_data,
	},
	.resource       = NULL,
	.num_resources  = 0,
};

void __init vyacht_platform_register_gpio_buttons(int id,
					 unsigned poll_interval,
					 unsigned nbuttons,
					 struct gpio_keys_button *buttons)
{
	struct platform_device *pdev;
	struct gpio_keys_platform_data pdata;
	struct gpio_keys_button *p;
	int err;

	p = kmalloc(nbuttons * sizeof(*p), GFP_KERNEL);
	if (!p)
		return;

	memcpy(p, buttons, nbuttons * sizeof(*p));

	pdev = platform_device_alloc("gpio-keys", id);
	if (!pdev)
		goto err_free_buttons;

	memset(&pdata, 0, sizeof(pdata));
	pdata.poll_interval = poll_interval;
	pdata.nbuttons = nbuttons;
	pdata.buttons = p;

	err = platform_device_add_data(pdev, &pdata, sizeof(pdata));
	if (err)
		goto err_put_pdev;

	err = platform_device_add(pdev);
	if (err)
		goto err_put_pdev;

	return;

err_put_pdev:
	platform_device_put(pdev);

err_free_buttons:
	kfree(p);
}



static int __devinit vyacht_platform_probe()
{
	int	status;
	return status;
}

static int __devexit vyspi_platform_remove()
{
	return 0;
}

static int __init vyacht_platform_init(void)
{
	int status= 0;
	printk("vyacht-platform init()\n");

        vyacht_platform_register_gpio_buttons(-1, 20,
                                     ARRAY_SIZE(vyacht_platform_gpio_buttons),
                                     vyacht_platform_gpio_buttons);
	return status;
}
module_init(vyacht_platform_init);

static void __exit vyacht_platform_exit(void)
{
	printk("vyacht-platform exit()\n");
}
module_exit(vyacht_platform_exit);

MODULE_AUTHOR("Bernd Ocklin, <bernd@vyacht.net>");
MODULE_DESCRIPTION("vYacht platform device registration.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("vyacht-platform");
