/*
 *  CARAMBOLA board support
 *
 *  Copyright (C) 2011 Darius Augulis <darius@8devices.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>

#include <asm/mach-ralink/machine.h>
#include <asm/mach-ralink/rt305x.h>
#include <asm/mach-ralink/rt305x_regs.h>
#include <asm/sizes.h>

#include "devices.h"

#ifdef CONFIG_MTD_PARTITIONS
static struct mtd_partition carambola_partitions[] = {
	{
		.name   = "u-boot",
		.offset = 0,
		.size   = SZ_128K + SZ_64K,
	}, {
		.name   = "u-boot-env",
		.offset = MTDPART_OFS_APPEND,
		.size   = SZ_64K,
	}, {
		.name   = "factory",
		.offset = MTDPART_OFS_APPEND,
		.size   = SZ_64K,
	}, {
		.name   = "kernel",
		.offset = SZ_256K + SZ_64K,
		.size   = SZ_1M,
	}, {
		.name   = "rootfs",
		.offset = SZ_256K + SZ_64K + SZ_1M,
		.size   = SZ_4M + SZ_2M,
	}, {
		.name   = "openwrt",
		.offset = SZ_256K + SZ_64K,
		.size   = SZ_4M + SZ_2M + SZ_1M,
	}
};
#endif /* CONFIG_MTD_PARTITIONS */

static struct physmap_flash_data carambola_flash_data = {
#ifdef CONFIG_MTD_PARTITIONS
	.nr_parts	= ARRAY_SIZE(carambola_partitions),
	.parts		= carambola_partitions,
#endif
};

static void __init carambola_init(void)
{
	rt305x_register_flash(0, &carambola_flash_data);
	rt305x_register_ethernet();
	rt305x_register_wifi();
	rt305x_register_usb();
}

MIPS_MACHINE(RAMIPS_MACH_CARAMBOLA, "CARAMBOLA", "CARAMBOLA",
	     carambola_init);
