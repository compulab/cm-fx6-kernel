/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <linux/compiler.h>
#include <linux/err.h>
#include <linux/init.h>

#include <mach/mx50.h>
#include <mach/devices-common.h>

#ifdef CONFIG_SOC_IMX50
const struct imx_dma_data imx50_dma_data __initconst = {
	.iobase = MX50_APBHDMA_BASE_ADDR,
};
#endif

struct platform_device *__init imx_add_dma(
			const struct imx_dma_data *data)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + SZ_8K - 1,
			.flags = IORESOURCE_MEM,
		},
	};

	return imx_add_platform_device_dmamask("mxs-dma-apbh", -1,
				res, ARRAY_SIZE(res), NULL, 0,
				DMA_BIT_MASK(32));
}
