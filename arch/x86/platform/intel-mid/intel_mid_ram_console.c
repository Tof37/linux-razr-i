/*
 * Intel mid ram console support
 *
 * Copyright (C) 2012 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/platform_data/ram_console.h>
#include <linux/memblock.h>
#include "intel_mid_ram_console.h"

static struct resource ram_console_resources[] = {
	{
		.flags  = IORESOURCE_MEM,
		.start  = INTEL_MID_RAM_CONSOLE_START_DEFAULT,
		.end    = INTEL_MID_RAM_CONSOLE_START_DEFAULT +
			  INTEL_MID_RAM_CONSOLE_SIZE_DEFAULT - 1,
	},
};

static struct ram_console_platform_data ram_console_pdata;

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
	.dev		= {
	.platform_data	= &ram_console_pdata,
	},
};

static __initdata bool intel_mid_ramconsole_inited;

/**
 * intel_mid_ram_console_register() - device_initcall to register ramconsole device
 */
static int __init intel_mid_ram_console_register(void)
{
	int ret;

	if (!intel_mid_ramconsole_inited)
		return -ENODEV;

	ret = platform_device_register(&ram_console_device);
	if (ret) {
		pr_err("%s: unable to register ram console device:"
			"start=0x%08x, end=0x%08x, ret=%d\n",
			__func__, (u32)ram_console_resources[0].start,
			(u32)ram_console_resources[0].end, ret);
	}

	return ret;
}
device_initcall(intel_mid_ram_console_register);

void __init ram_consle_reserve_memory(void)
{
	phys_addr_t mem;
	size_t size;

	size = INTEL_MID_RAM_CONSOLE_SIZE_DEFAULT;
	size = ALIGN(size, PAGE_SIZE);

	mem = memblock_find_in_range(0, 1<<28, size, PAGE_SIZE);
	if (mem == MEMBLOCK_ERROR)
		panic("Cannot allocate \n");

	ram_console_resources[0].start = mem;
	ram_console_resources[0].end = mem + size - 1;
	memblock_x86_reserve_range(mem, mem + size, "ram_console");

	intel_mid_ramconsole_inited = true;
}

