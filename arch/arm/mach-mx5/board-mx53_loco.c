/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
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

#include <linux/init.h>
#include <linux/clk.h>
#include <linux/fec.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/mxcfb.h>
#include <linux/ipu.h>
#include <linux/pwm_backlight.h>
#include <linux/ahci_platform.h>
#include <linux/gpio_keys.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/imx-uart.h>
#include <mach/iomux-mx53.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_dvfs.h>
#include <mach/ahci_sata.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include "crm_regs.h"
#include "devices-imx53.h"
#include "devices.h"
#include "usb.h"

#define LOCO_DISP0_PWR			IMX_GPIO_NR(3, 24)
#define LOCO_DISP0_DET_INT		IMX_GPIO_NR(3, 31)
#define LOCO_DISP0_RESET		IMX_GPIO_NR(5, 0)
#define MX53_LOCO_POWER			IMX_GPIO_NR(1, 8)
#define MX53_LOCO_UI1			IMX_GPIO_NR(2, 14)
#define MX53_LOCO_UI2			IMX_GPIO_NR(2, 15)
#define LOCO_FEC_PHY_RST		IMX_GPIO_NR(7, 6)
#define LOCO_USBH1_VBUS			IMX_GPIO_NR(7, 8)

extern void __iomem *arm_plat_base;
extern void __iomem *gpc_base;
extern void __iomem *ccm_base;
extern void __iomem *imx_otg_base;

extern int __init mx53_loco_init_da9052(void);

static iomux_v3_cfg_t mx53_loco_pads[] = {
	/* FEC */
	MX53_PAD_FEC_MDC__FEC_MDC,
	MX53_PAD_FEC_MDIO__FEC_MDIO,
	MX53_PAD_FEC_REF_CLK__FEC_TX_CLK,
	MX53_PAD_FEC_RX_ER__FEC_RX_ER,
	MX53_PAD_FEC_CRS_DV__FEC_RX_DV,
	MX53_PAD_FEC_RXD1__FEC_RDATA_1,
	MX53_PAD_FEC_RXD0__FEC_RDATA_0,
	MX53_PAD_FEC_TX_EN__FEC_TX_EN,
	MX53_PAD_FEC_TXD1__FEC_TDATA_1,
	MX53_PAD_FEC_TXD0__FEC_TDATA_0,
	/* FEC_nRST */
	MX53_PAD_PATA_DA_0__GPIO7_6,
	/* FEC_nINT */
	MX53_PAD_PATA_DATA4__GPIO2_4,
	/* AUDMUX5 */
	MX53_PAD_KEY_COL0__AUDMUX_AUD5_TXC,
	MX53_PAD_KEY_ROW0__AUDMUX_AUD5_TXD,
	MX53_PAD_KEY_COL1__AUDMUX_AUD5_TXFS,
	MX53_PAD_KEY_ROW1__AUDMUX_AUD5_RXD,
	/* I2C2 */
	MX53_PAD_KEY_COL3__I2C2_SCL,
	MX53_PAD_KEY_ROW3__I2C2_SDA,
	/* SD1 */
	MX53_PAD_SD1_CMD__ESDHC1_CMD,
	MX53_PAD_SD1_CLK__ESDHC1_CLK,
	MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX53_PAD_SD1_DATA3__ESDHC1_DAT3,
	/* SD3 */
	MX53_PAD_PATA_DATA8__ESDHC3_DAT0,
	MX53_PAD_PATA_DATA9__ESDHC3_DAT1,
	MX53_PAD_PATA_DATA10__ESDHC3_DAT2,
	MX53_PAD_PATA_DATA11__ESDHC3_DAT3,
	MX53_PAD_PATA_DATA0__ESDHC3_DAT4,
	MX53_PAD_PATA_DATA1__ESDHC3_DAT5,
	MX53_PAD_PATA_DATA2__ESDHC3_DAT6,
	MX53_PAD_PATA_DATA3__ESDHC3_DAT7,
	MX53_PAD_PATA_IORDY__ESDHC3_CLK,
	MX53_PAD_PATA_RESET_B__ESDHC3_CMD,
	/* SD3_CD */
	MX53_PAD_EIM_DA11__GPIO3_11,
	/* SD3_WP */
	MX53_PAD_EIM_DA12__GPIO3_12,
	/* VGA */
	MX53_PAD_EIM_OE__IPU_DI1_PIN7,
	MX53_PAD_EIM_RW__IPU_DI1_PIN8,
	/* DISPLB */
	MX53_PAD_EIM_D20__IPU_SER_DISP0_CS,
	MX53_PAD_EIM_D21__IPU_DISPB0_SER_CLK,
	MX53_PAD_EIM_D22__IPU_DISPB0_SER_DIN,
	MX53_PAD_EIM_D23__IPU_DI0_D0_CS,
	/* DISP0_POWER_EN */
	MX53_PAD_EIM_D24__GPIO3_24,
	/* DISP0 DET INT */
	MX53_PAD_EIM_D31__GPIO3_31,
	/* LVDS */
	MX53_PAD_LVDS0_TX3_P__LDB_LVDS0_TX3,
	MX53_PAD_LVDS0_CLK_P__LDB_LVDS0_CLK,
	MX53_PAD_LVDS0_TX2_P__LDB_LVDS0_TX2,
	MX53_PAD_LVDS0_TX1_P__LDB_LVDS0_TX1,
	MX53_PAD_LVDS0_TX0_P__LDB_LVDS0_TX0,
	MX53_PAD_LVDS1_TX3_P__LDB_LVDS1_TX3,
	MX53_PAD_LVDS1_TX2_P__LDB_LVDS1_TX2,
	MX53_PAD_LVDS1_CLK_P__LDB_LVDS1_CLK,
	MX53_PAD_LVDS1_TX1_P__LDB_LVDS1_TX1,
	MX53_PAD_LVDS1_TX0_P__LDB_LVDS1_TX0,
	/* I2C1 */
	MX53_PAD_CSI0_DAT8__I2C1_SDA,
	MX53_PAD_CSI0_DAT9__I2C1_SCL,
	/* UART1 */
	MX53_PAD_CSI0_DAT10__UART1_TXD_MUX,
	MX53_PAD_CSI0_DAT11__UART1_RXD_MUX,
	/* CSI0 */
	MX53_PAD_CSI0_DAT12__IPU_CSI0_D_12,
	MX53_PAD_CSI0_DAT13__IPU_CSI0_D_13,
	MX53_PAD_CSI0_DAT14__IPU_CSI0_D_14,
	MX53_PAD_CSI0_DAT15__IPU_CSI0_D_15,
	MX53_PAD_CSI0_DAT16__IPU_CSI0_D_16,
	MX53_PAD_CSI0_DAT17__IPU_CSI0_D_17,
	MX53_PAD_CSI0_DAT18__IPU_CSI0_D_18,
	MX53_PAD_CSI0_DAT19__IPU_CSI0_D_19,
	MX53_PAD_CSI0_VSYNC__IPU_CSI0_VSYNC,
	MX53_PAD_CSI0_MCLK__IPU_CSI0_HSYNC,
	MX53_PAD_CSI0_PIXCLK__IPU_CSI0_PIXCLK,
	/* DISPLAY */
	MX53_PAD_DI0_DISP_CLK__IPU_DI0_DISP_CLK,
	MX53_PAD_DI0_PIN15__IPU_DI0_PIN15,
	MX53_PAD_DI0_PIN2__IPU_DI0_PIN2,
	MX53_PAD_DI0_PIN3__IPU_DI0_PIN3,
	MX53_PAD_DISP0_DAT0__IPU_DISP0_DAT_0,
	MX53_PAD_DISP0_DAT1__IPU_DISP0_DAT_1,
	MX53_PAD_DISP0_DAT2__IPU_DISP0_DAT_2,
	MX53_PAD_DISP0_DAT3__IPU_DISP0_DAT_3,
	MX53_PAD_DISP0_DAT4__IPU_DISP0_DAT_4,
	MX53_PAD_DISP0_DAT5__IPU_DISP0_DAT_5,
	MX53_PAD_DISP0_DAT6__IPU_DISP0_DAT_6,
	MX53_PAD_DISP0_DAT7__IPU_DISP0_DAT_7,
	MX53_PAD_DISP0_DAT8__IPU_DISP0_DAT_8,
	MX53_PAD_DISP0_DAT9__IPU_DISP0_DAT_9,
	MX53_PAD_DISP0_DAT10__IPU_DISP0_DAT_10,
	MX53_PAD_DISP0_DAT11__IPU_DISP0_DAT_11,
	MX53_PAD_DISP0_DAT12__IPU_DISP0_DAT_12,
	MX53_PAD_DISP0_DAT13__IPU_DISP0_DAT_13,
	MX53_PAD_DISP0_DAT14__IPU_DISP0_DAT_14,
	MX53_PAD_DISP0_DAT15__IPU_DISP0_DAT_15,
	MX53_PAD_DISP0_DAT16__IPU_DISP0_DAT_16,
	MX53_PAD_DISP0_DAT17__IPU_DISP0_DAT_17,
	MX53_PAD_DISP0_DAT18__IPU_DISP0_DAT_18,
	MX53_PAD_DISP0_DAT19__IPU_DISP0_DAT_19,
	MX53_PAD_DISP0_DAT20__IPU_DISP0_DAT_20,
	MX53_PAD_DISP0_DAT21__IPU_DISP0_DAT_21,
	MX53_PAD_DISP0_DAT22__IPU_DISP0_DAT_22,
	MX53_PAD_DISP0_DAT23__IPU_DISP0_DAT_23,
	/* Audio CLK*/
	MX53_PAD_GPIO_0__CCM_SSI_EXT1_CLK,
	/* PWM */
	MX53_PAD_GPIO_1__PWM2_PWMO,
	/* SPDIF */
	MX53_PAD_GPIO_7__SPDIF_PLOCK,
	MX53_PAD_GPIO_17__SPDIF_OUT1,
	/* GPIO */
	MX53_PAD_PATA_DA_1__GPIO7_7,
	MX53_PAD_PATA_DA_2__GPIO7_8,
	MX53_PAD_PATA_DATA5__GPIO2_5,
	MX53_PAD_PATA_DATA6__GPIO2_6,
	MX53_PAD_PATA_DATA14__GPIO2_14,
	MX53_PAD_PATA_DATA15__GPIO2_15,
	MX53_PAD_PATA_INTRQ__GPIO7_2,
	MX53_PAD_EIM_WAIT__GPIO5_0,
	MX53_PAD_NANDF_WP_B__GPIO6_9,
	MX53_PAD_NANDF_RB0__GPIO6_10,
	MX53_PAD_NANDF_CS1__GPIO6_14,
	MX53_PAD_NANDF_CS2__GPIO6_15,
	MX53_PAD_NANDF_CS3__GPIO6_16,
	MX53_PAD_GPIO_5__GPIO1_5,
	MX53_PAD_GPIO_16__GPIO7_11,
	MX53_PAD_GPIO_8__GPIO1_8,
};

#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
}

static const struct gpio_keys_button loco_buttons[] __initconst = {
	GPIO_BUTTON(MX53_LOCO_POWER, KEY_POWER, 1, "power", 1),
	GPIO_BUTTON(MX53_LOCO_UI1, KEY_VOLUMEUP, 1, "volume-up", 0),
	GPIO_BUTTON(MX53_LOCO_UI2, KEY_VOLUMEDOWN, 1, "volume-down", 0),
};

static const struct gpio_keys_platform_data loco_button_data __initconst = {
	.buttons        = (struct gpio_keys_button *)loco_buttons,
	.nbuttons       = ARRAY_SIZE(loco_buttons),
};

static inline void mx53_loco_fec_reset(void)
{
	int ret;

	/* reset FEC PHY */
	ret = gpio_request(LOCO_FEC_PHY_RST, "fec-phy-reset");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO_FEC_PHY_RESET: %d\n", ret);
		return;
	}
	gpio_direction_output(LOCO_FEC_PHY_RST, 0);
	msleep(1);
	gpio_set_value(LOCO_FEC_PHY_RST, 1);
}

static struct fec_platform_data mx53_loco_fec_data = {
	.phy = PHY_INTERFACE_MODE_RMII,
};

static const struct imxi2c_platform_data mx53_loco_i2c_data __initconst = {
	.bitrate = 100000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
	.type = "mma8450",
	.addr = 0x1C,
	},
};

static void sii902x_hdmi_reset(void)
{
	gpio_set_value(LOCO_DISP0_RESET, 0);
	msleep(10);
	gpio_set_value(LOCO_DISP0_RESET, 1);
	msleep(10);
}

static struct fsl_mxc_lcd_platform_data sii902x_hdmi_data = {
       .reset = sii902x_hdmi_reset,
};

static void loco_suspend_enter(void)
{
	/* da9053 suspend preparation */
}

static void loco_suspend_exit(void)
{
	/*clear the EMPGC0/1 bits */
	__raw_writel(0, MXC_SRPG_EMPGC0_SRPGCR);
	__raw_writel(0, MXC_SRPG_EMPGC1_SRPGCR);
	/* da9053 resmue resore */
}

static struct mxc_pm_platform_data loco_pm_data = {
	.suspend_enter = loco_suspend_enter,
	.suspend_exit = loco_suspend_exit,
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
	.type = "sgtl5000",
	.addr = 0x0a,
	},
	{
	.type = "sii902x",
	.addr = 0x39,
	.irq = gpio_to_irq(LOCO_DISP0_DET_INT),
	.platform_data = &sii902x_hdmi_data,
	},
};

static struct fb_videomode video_modes[] = {
	{
	/* 800x480 @ 57 Hz , pixel clk @ 27MHz */
	"CLAA-WVGA", 57, 800, 480, 37037, 40, 60, 10, 10, 20, 10,
	FB_SYNC_CLK_LAT_FALL,
	FB_VMODE_NONINTERLACED,
	0,},
	{
	/* 800x480 @ 60 Hz , pixel clk @ 32MHz */
	"SEIKO-WVGA", 60, 800, 480, 29850, 89, 164, 23, 10, 10, 10,
	FB_SYNC_CLK_LAT_FALL,
	FB_VMODE_NONINTERLACED,
	0,},
	{
	/* 1600x1200 @ 60 Hz 162M pixel clk*/
	"UXGA", 60, 1600, 1200, 6172,
	304, 64,
	1, 46,
	192, 3,
	FB_SYNC_HOR_HIGH_ACT|FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_NONINTERLACED,
	0,},
};

static struct ipuv3_fb_platform_data loco_fb_di0_data = {
	.interface_pix_fmt = IPU_PIX_FMT_RGB565,
	.mode_str = "CLAA-WVGA",
	.modes = video_modes,
	.num_modes = ARRAY_SIZE(video_modes),
};

static struct ipuv3_fb_platform_data loco_fb_di1_data = {
	.interface_pix_fmt = IPU_PIX_FMT_GBR24,
	.mode_str = "1024x768M-16@60",
	.modes = video_modes,
	.num_modes = ARRAY_SIZE(video_modes),
};

static struct imx_ipuv3_platform_data ipu_data = {
	.rev = 3,
	.fb_head0_platform_data = &loco_fb_di0_data,
	.fb_head1_platform_data = &loco_fb_di1_data,
};

static struct platform_pwm_backlight_data loco_pwm_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct fsl_mxc_tve_platform_data tve_data = {
	.dac_reg = "DA9052_LDO7",
};

static struct mxc_dvfs_platform_data loco_dvfs_core_data = {
	.reg_id = "DA9052_BUCK_CORE",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.gpc_vcr_offset = MXC_GPC_VCR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 30,
};

static struct mxc_bus_freq_platform_data loco_bus_freq_data = {
	.gp_reg_id = "DA9052_BUCK_CORE",
	.lp_reg_id = "DA9052_BUCK_PRO",
};

static void mx53_loco_usbh1_vbus(bool on)
{
	if (on)
		gpio_set_value(LOCO_USBH1_VBUS, 1);
	else
		gpio_set_value(LOCO_USBH1_VBUS, 0);
}

static void __init mx53_loco_io_init(void)
{
	int ret;

	arm_plat_base = MX53_IO_ADDRESS(MX53_ARM_BASE_ADDR);
	gpc_base = MX53_IO_ADDRESS(MX53_GPC_BASE_ADDR);
	ccm_base = MX53_IO_ADDRESS(MX53_CCM_BASE_ADDR);
	imx_otg_base = MX53_IO_ADDRESS(MX53_OTG_BASE_ADDR);

	mxc_iomux_v3_setup_multiple_pads(mx53_loco_pads,
					ARRAY_SIZE(mx53_loco_pads));

	/* Sii902x HDMI controller */
	ret = gpio_request(LOCO_DISP0_RESET, "disp0-reset");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO_LOCO_DISP0_RESET: %d\n", ret);
		return;
	}
	gpio_direction_output(LOCO_DISP0_RESET, 0);

	ret = gpio_request(LOCO_DISP0_DET_INT, "disp0-detect");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO_LOCO_DISP0_DET_INT: %d\n", ret);
		return;
	}
	gpio_direction_input(LOCO_DISP0_DET_INT);

	/* enable disp0 power */
	ret = gpio_request(LOCO_DISP0_PWR, "disp0-power-en");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO_LOCO_DISP0_PWR: %d\n", ret);
		return;
	}
	gpio_direction_output(LOCO_DISP0_PWR, 1);

	/* usb host1 vbus */
	ret = gpio_request(LOCO_USBH1_VBUS, "usbh1-vbus");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO LOCO_USBH1_VBUS: %d\n", ret);
		return;
	}
	gpio_direction_output(LOCO_USBH1_VBUS, 0);
}

/* HW Initialization, if return 0, initialization is successful. */
static int sata_init(struct device *dev, void __iomem *addr)
{
	void __iomem *mmio;
	struct clk *clk;
	int ret = 0;
	u32 tmpdata;

	clk = clk_get(dev, "imx_sata_clk");
	ret = IS_ERR(clk);
	if (ret) {
		printk(KERN_ERR "AHCI can't get clock.\n");
		return ret;
	}
	ret = clk_enable(clk);
	if (ret) {
		printk(KERN_ERR "AHCI can't enable clock.\n");
		clk_put(clk);
		return ret;
	}

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb_clk");
	ret = IS_ERR(clk);
	if (ret) {
		printk(KERN_ERR "AHCI can't get AHB clock.\n");
		goto no_ahb_clk;
	}

	mmio = ioremap(MX53_SATA_BASE_ADDR, SZ_2K);
	if (mmio == NULL) {
		printk(KERN_ERR "Failed to map SATA REGS\n");
		goto no_ahb_clk;
	}

	tmpdata = readl(mmio + HOST_CAP);
	if (!(tmpdata & HOST_CAP_SSS)) {
		tmpdata |= HOST_CAP_SSS;
		writel(tmpdata, mmio + HOST_CAP);
	}

	if (!(readl(mmio + HOST_PORTS_IMPL) & 0x1))
		writel((readl(mmio + HOST_PORTS_IMPL) | 0x1),
			mmio + HOST_PORTS_IMPL);

	tmpdata = clk_get_rate(clk) / 1000;
	writel(tmpdata, mmio + HOST_TIMER1MS);

	clk = clk_get(dev, "usb_phy1_clk");
	ret = IS_ERR(clk);
	if (ret) {
		printk(KERN_ERR "AHCI can't get USB PHY1 CLK.\n");
		goto no_ahb_clk;
	}
	ret = clk_enable(clk);
	if (ret) {
		printk(KERN_ERR "AHCI Can't enable USB PHY1 clock.\n");
		clk_put(clk);
		goto no_ahb_clk;
	}

	/* Release resources when there is no device on the port */
	if ((readl(mmio + PORT_SATA_SR) & 0xF) == 0) {
		iounmap(mmio);
		ret = -ENODEV;
		goto no_device;
	}

	iounmap(mmio);

	return ret;

no_device:
	printk(KERN_INFO "NO SATA device is found, relase resource!\n");
	clk = clk_get(dev, "usb_phy1_clk");
	if (IS_ERR(clk)) {
		clk = NULL;
		printk(KERN_ERR "AHCI can't get USB PHY1 CLK.\n");
	} else {
		clk_disable(clk);
		clk_put(clk);
	}

no_ahb_clk:
	clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(clk)) {
		clk = NULL;
		printk(KERN_ERR "IMX SATA can't get clock.\n");
	} else {
		clk_disable(clk);
		clk_put(clk);
	}

	return ret;
}

static void sata_exit(struct device *dev)
{
	struct clk *clk;

	clk = clk_get(dev, "usb_phy1_clk");
	if (IS_ERR(clk)) {
		clk = NULL;
		printk(KERN_ERR "AHCI can't get USB PHY1 CLK.\n");
	} else {
		clk_disable(clk);
		clk_put(clk);
	}

	clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(clk)) {
		clk = NULL;
		printk(KERN_ERR "IMX SATA can't get clock.\n");
	} else {
		clk_disable(clk);
		clk_put(clk);
	}
}

static struct ahci_platform_data sata_data = {
	.init = sata_init,
	.exit = sata_exit,
};

static struct mxc_audio_platform_data loco_audio_data;

static int loco_sgtl5000_init(void)
{
	struct clk *ssi_ext1;
	int rate;

	ssi_ext1 = clk_get(NULL, "ssi_ext1_clk");
	if (IS_ERR(ssi_ext1)) {
		return -1;
	}
	rate = clk_round_rate(ssi_ext1, 24000000);
	if (rate < 8000000 || rate > 27000000) {
			printk(KERN_ERR "Error: SGTL5000 mclk freq %d out of range!\n",
				   rate);
			clk_put(ssi_ext1);
			return -1;
	}

	loco_audio_data.sysclk = rate;
	clk_set_rate(ssi_ext1, rate);
	clk_enable(ssi_ext1);

	return 0;
}

static struct imx_ssi_platform_data loco_ssi_pdata = {
	.flags = IMX_SSI_DMA,
};

static struct mxc_audio_platform_data loco_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,
	.init = loco_sgtl5000_init,
};

static struct platform_device loco_audio_device = {
	.name = "imx-sgtl5000",
};

static void mxc_iim_enable_fuse(void)
{
	u32 reg;
	if (!ccm_base)
		return;
	/* enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg |= 0x10;
	writel(reg, ccm_base + 0x64);
}

static void mxc_iim_disable_fuse(void)
{
	u32 reg;
	if (!ccm_base)
		return;
	/* enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg &= ~0x10;
	writel(reg, ccm_base + 0x64);
}

static struct mxc_iim_platform_data iim_data = {
	.bank_start = MXC_IIM_MX53_BANK_START_ADDR,
	.bank_end   = MXC_IIM_MX53_BANK_END_ADDR,
	.enable_fuse = mxc_iim_enable_fuse,
	.disable_fuse = mxc_iim_disable_fuse,
};

static struct mxc_gpu_platform_data gpu_data __initdata;

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ext_ref = 1,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	struct tag *t;
	struct tag *mem_tag = 0;
	int total_mem = SZ_1G;
	int left_mem = 0;
	int gpu_mem = SZ_128M;
	int fb_mem = SZ_32M;
	char *str;

	for_each_tag(mem_tag, tags) {
		if (mem_tag->hdr.tag == ATAG_MEM) {
			total_mem = mem_tag->u.mem.size;
			left_mem = total_mem - gpu_mem - fb_mem;
			break;
		}
	}

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "mem=");
			if (str != NULL) {
				str += 4;
				left_mem = memparse(str, &str);
				if (left_mem == 0 || left_mem > total_mem)
					left_mem = total_mem - gpu_mem - fb_mem;
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				gpu_mem = memparse(str, &str);
			}

			break;
		}
	}

	if (mem_tag) {
		fb_mem = total_mem - left_mem - gpu_mem;
		if (fb_mem < 0) {
			gpu_mem = total_mem - left_mem;
			fb_mem = 0;
		}
		mem_tag->u.mem.size = left_mem;

		/* reserve memory for gpu */
		gpu_data.reserved_mem_base =
				mem_tag->u.mem.start + left_mem;
		gpu_data.reserved_mem_size = gpu_mem;

		/* reserver memory for fb */
		loco_fb_di0_data.res_base = gpu_data.reserved_mem_base
					+ gpu_data.reserved_mem_size;
		loco_fb_di0_data.res_size = fb_mem;
		loco_fb_di1_data.res_base = loco_fb_di0_data.res_base;
		loco_fb_di1_data.res_size = loco_fb_di0_data.res_size;
	}
}

static void __init mx53_loco_board_init(void)
{
	mx53_loco_io_init();

	imx53_add_imx_uart(0, NULL);
	mx53_loco_fec_reset();
	imx53_add_fec(&mx53_loco_fec_data);

	mxc_register_device(&mxc_pm_device, &loco_pm_data);

	imx53_add_ipuv3(&ipu_data);
	imx53_add_vpu();
	imx53_add_ldb(&ldb_data);
	imx53_add_tve(&tve_data);
	imx53_add_v4l2_output(0);

	imx53_add_mxc_pwm(1);
	imx53_add_mxc_pwm_backlight(0, &loco_pwm_backlight_data);

	imx53_add_imx2_wdt(0, NULL);
	imx53_add_srtc();
	imx53_add_dvfs_core(&loco_dvfs_core_data);
	imx53_add_busfreq(&loco_bus_freq_data);
	imx53_add_imx_i2c(0, &mx53_loco_i2c_data);
	imx53_add_imx_i2c(1, &mx53_loco_i2c_data);

	mx53_loco_init_da9052();
	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));

	imx53_add_sdhci_esdhc_imx(0, NULL);
	imx53_add_sdhci_esdhc_imx(2, NULL);
	imx53_add_ahci_imx(0, &sata_data);
	imx53_add_iim(&iim_data);

	/* USB */
	mx5_usb_dr_init();
	mx5_set_host1_vbus_func(mx53_loco_usbh1_vbus);
	mx5_usbh1_init();

	mxc_register_device(&loco_audio_device, &loco_audio_data);
	imx53_add_imx_ssi(1, &loco_ssi_pdata);

	/*GPU*/
	if (mx53_revision() >= IMX_CHIP_REVISION_2_0)
		gpu_data.z160_revision = 1;
	else
		gpu_data.z160_revision = 0;
	imx53_add_mxc_gpu(&gpu_data);
	imx_add_gpio_keys(&loco_button_data);

	/* this call required to release SCC RAM partition held by ROM
	  * during boot, even if SCC2 driver is not part of the image
	  */
	imx53_add_mxc_scc2();
}

static void __init mx53_loco_timer_init(void)
{
	mx53_clocks_init(32768, 24000000, 0, 0);
}

static struct sys_timer mx53_loco_timer = {
	.init	= mx53_loco_timer_init,
};

MACHINE_START(MX53_LOCO, "Freescale MX53 LOCO Board")
	.fixup = fixup_mxc_board,
	.map_io = mx53_map_io,
	.init_early = imx53_init_early,
	.init_irq = mx53_init_irq,
	.timer = &mx53_loco_timer,
	.init_machine = mx53_loco_board_init,
MACHINE_END
