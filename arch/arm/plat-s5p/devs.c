/* linux/arch/arm/plat-s5p/devs.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * Base S5P platform device definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <mach/hardware.h>
#include <mach/map.h>
#include <mach/dma.h>

#include <plat/devs.h>
#include <plat/gpio-cfg.h>
#include <plat/irqs.h>

#ifdef CONFIG_VIDEO_JPEG
static struct resource s5p_jpeg_resource[] = {
	[0] = {
		.start	= S5P_PA_JPEG,
		.end	= S5P_PA_JPEG + S5P_SZ_JPEG - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_JPEG,
		.end	= IRQ_JPEG,
		.flags	= IORESOURCE_IRQ,
	}
};

struct platform_device s5p_device_jpeg = {
	.name		= "s5p-jpeg",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(s5p_jpeg_resource),
	.resource	= s5p_jpeg_resource,
};
EXPORT_SYMBOL(s5p_device_jpeg);
#endif

