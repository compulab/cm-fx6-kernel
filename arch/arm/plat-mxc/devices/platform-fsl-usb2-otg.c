/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Based on Uwe Kleine-Koenig's platform-fsl-usb2-udc.c
 * Copyright (C) 2010 Pengutronix
 * Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <mach/hardware.h>
#include <mach/devices-common.h>

#define imx_fsl_usb2_otg_data_entry_single(soc)				\
	{								\
		.iobase = soc ## _USB_OTG_BASE_ADDR,			\
		.irq = soc ## _INT_USB_OTG,				\
	}


#ifdef CONFIG_SOC_IMX6Q
const struct imx_fsl_usb2_otg_data imx6q_fsl_usb2_otg_data __initconst =
	imx_fsl_usb2_otg_data_entry_single(MX6Q);
#endif /* ifdef CONFIG_SOC_IMX6Q */

struct platform_device *__init imx_add_fsl_usb2_otg(
		const struct imx_fsl_usb2_otg_data *data,
		const struct fsl_usb2_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + SZ_512 - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};
	return imx_add_platform_device_dmamask("fsl-usb2-otg", -1,
			res, ARRAY_SIZE(res),
			pdata, sizeof(*pdata), DMA_BIT_MASK(32));
}
