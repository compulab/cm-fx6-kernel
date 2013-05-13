/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <mach/common.h>

extern int usbotg_init(struct platform_device *pdev);
extern void usbotg_uninit(struct fsl_usb2_platform_data *pdata);
extern struct platform_device *host_pdev_register(struct resource *res,
		  int n_res, struct fsl_usb2_platform_data *config);

extern int fsl_usb_host_init(struct platform_device *pdev);
extern void fsl_usb_host_uninit(struct fsl_usb2_platform_data *pdata);
extern int gpio_usbotg_utmi_active(void);
extern void gpio_usbotg_utmi_inactive(void);

extern void __init mx6_usb_dr_init(void);
extern void __init mx6_usb_h1_init(void);

typedef void (*driver_vbus_func)(bool);
extern void mx6_set_host1_vbus_func(driver_vbus_func);
extern void mx6_set_otghost_vbus_func(driver_vbus_func);
extern struct platform_device anatop_thermal_device;
extern struct platform_device mxc_usbdr_otg_device;
extern struct platform_device mxc_usbdr_udc_device;
extern struct platform_device mxc_usbdr_host_device;
extern struct platform_device mxc_usbdr_wakeup_device;
extern struct platform_device mxc_usbh1_device;
extern struct platform_device mxc_usbh1_wakeup_device;

/*
 * Used to set pdata->operating_mode before registering the platform_device.
 * If OTG is configured, the controller operates in OTG mode,
 * otherwise it's either host or device.
 */
#ifdef CONFIG_USB_OTG
#define DR_UDC_MODE	FSL_USB2_DR_OTG
#define DR_HOST_MODE	FSL_USB2_DR_OTG
#else
#define DR_UDC_MODE	FSL_USB2_DR_DEVICE
#define DR_HOST_MODE	FSL_USB2_DR_HOST
#endif

extern void __iomem *imx_otg_base;
