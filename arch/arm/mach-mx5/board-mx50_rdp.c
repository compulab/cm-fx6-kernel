/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/io.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/iomux-mx50.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "devices-imx50.h"
#include "cpu_op-mx50.h"
#include "devices.h"

#define FEC_EN		IMX_GPIO_NR(6, 23)
#define FEC_RESET_B	IMX_GPIO_NR(4, 12)
#define MX50_RDP_CSPI_CS0	IMX_GPIO_NR(4, 13)
#define MX50_RDP_CSPI_CS1	IMX_GPIO_NR(4, 11)

extern int mx50_rdp_init_mc13892(void);

static iomux_v3_cfg_t mx50_rdp_pads[] __initdata = {
	/* SD1 */
	MX50_PAD_ECSPI2_SS0__GPIO_4_19,
	MX50_PAD_EIM_CRE__GPIO_1_27,
	MX50_PAD_SD1_CMD__SD1_CMD,

	MX50_PAD_SD1_CLK__SD1_CLK,
	MX50_PAD_SD1_D0__SD1_D0,
	MX50_PAD_SD1_D1__SD1_D1,
	MX50_PAD_SD1_D2__SD1_D2,
	MX50_PAD_SD1_D3__SD1_D3,

	/* SD2 */
	MX50_PAD_SD2_CD__GPIO_5_17,
	MX50_PAD_SD2_WP__GPIO_5_16,
	MX50_PAD_SD2_CMD__SD2_CMD,
	MX50_PAD_SD2_CLK__SD2_CLK,
	MX50_PAD_SD2_D0__SD2_D0,
	MX50_PAD_SD2_D1__SD2_D1,
	MX50_PAD_SD2_D2__SD2_D2,
	MX50_PAD_SD2_D3__SD2_D3,
	MX50_PAD_SD2_D4__SD2_D4,
	MX50_PAD_SD2_D5__SD2_D5,
	MX50_PAD_SD2_D6__SD2_D6,
	MX50_PAD_SD2_D7__SD2_D7,

	/* SD3 */
	MX50_PAD_SD3_CMD__SD3_CMD,
	MX50_PAD_SD3_CLK__SD3_CLK,
	MX50_PAD_SD3_D0__SD3_D0,
	MX50_PAD_SD3_D1__SD3_D1,
	MX50_PAD_SD3_D2__SD3_D2,
	MX50_PAD_SD3_D3__SD3_D3,
	MX50_PAD_SD3_D4__SD3_D4,
	MX50_PAD_SD3_D5__SD3_D5,
	MX50_PAD_SD3_D6__SD3_D6,
	MX50_PAD_SD3_D7__SD3_D7,

	/* PWR_INT */
	MX50_PAD_ECSPI2_MISO__GPIO_4_18,

	/* UART pad setting */
	MX50_PAD_UART1_TXD__UART1_TXD,
	MX50_PAD_UART1_RXD__UART1_RXD,
	MX50_PAD_UART1_RTS__UART1_RTS,
	MX50_PAD_UART2_TXD__UART2_TXD,
	MX50_PAD_UART2_RXD__UART2_RXD,
	MX50_PAD_UART2_CTS__UART2_CTS,
	MX50_PAD_UART2_RTS__UART2_RTS,

	MX50_PAD_I2C1_SCL__I2C1_SCL,
	MX50_PAD_I2C1_SDA__I2C1_SDA,
	MX50_PAD_I2C2_SCL__I2C2_SCL,
	MX50_PAD_I2C2_SDA__I2C2_SDA,

	MX50_PAD_EPITO__USBH1_PWR,
	/* Need to comment below line if
	 * one needs to debug owire.
	 */
	MX50_PAD_OWIRE__USBH1_OC,
	/* using gpio to control otg pwr */
	MX50_PAD_PWM2__GPIO_6_25,
	MX50_PAD_I2C3_SCL__USBOTG_OC,

	MX50_PAD_SSI_RXC__FEC_MDIO,
	MX50_PAD_SSI_RXFS__FEC_MDC,
	MX50_PAD_DISP_D0__FEC_TXCLK,
	MX50_PAD_DISP_D1__FEC_RX_ER,
	MX50_PAD_DISP_D2__FEC_RX_DV,
	MX50_PAD_DISP_D3__FEC_RXD1,
	MX50_PAD_DISP_D4__FEC_RXD0,
	MX50_PAD_DISP_D5__FEC_TX_EN,
	MX50_PAD_DISP_D6__FEC_TXD1,
	MX50_PAD_DISP_D7__FEC_TXD0,
	MX50_PAD_I2C3_SDA__GPIO_6_23,
	MX50_PAD_ECSPI1_SCLK__GPIO_4_12,

	MX50_PAD_CSPI_SS0__CSPI_SS0,
	MX50_PAD_ECSPI1_MOSI__CSPI_SS1,
	MX50_PAD_CSPI_MOSI__CSPI_MOSI,
	MX50_PAD_CSPI_MISO__CSPI_MISO,

	/* SGTL500_OSC_EN */
	MX50_PAD_UART1_CTS__GPIO_6_8,

	/* SGTL_AMP_SHDN */
	MX50_PAD_UART3_RXD__GPIO_6_15,

	/* Keypad */
	MX50_PAD_KEY_COL0__KEY_COL0,
	MX50_PAD_KEY_ROW0__KEY_ROW0,
	MX50_PAD_KEY_COL1__KEY_COL1,
	MX50_PAD_KEY_ROW1__KEY_ROW1,
	MX50_PAD_KEY_COL2__KEY_COL2,
	MX50_PAD_KEY_ROW2__KEY_ROW2,
	MX50_PAD_KEY_COL3__KEY_COL3,
	MX50_PAD_KEY_ROW3__KEY_ROW3,
	MX50_PAD_EIM_DA0__KEY_COL4,
	MX50_PAD_EIM_DA1__KEY_ROW4,
	MX50_PAD_EIM_DA2__KEY_COL5,
	MX50_PAD_EIM_DA3__KEY_ROW5,
	MX50_PAD_EIM_DA4__KEY_COL6,
	MX50_PAD_EIM_DA5__KEY_ROW6,
	MX50_PAD_EIM_DA6__KEY_COL7,
	MX50_PAD_EIM_DA7__KEY_ROW7,
	/*EIM pads */
	MX50_PAD_EIM_DA8__GPIO_1_8,
	MX50_PAD_EIM_DA9__GPIO_1_9,
	MX50_PAD_EIM_DA10__GPIO_1_10,
	MX50_PAD_EIM_DA11__GPIO_1_11,
	MX50_PAD_EIM_DA12__GPIO_1_12,
	MX50_PAD_EIM_DA13__GPIO_1_13,
	MX50_PAD_EIM_DA14__GPIO_1_14,
	MX50_PAD_EIM_DA15__GPIO_1_15,
	MX50_PAD_EIM_CS2__GPIO_1_16,
	MX50_PAD_EIM_CS1__GPIO_1_17,
	MX50_PAD_EIM_CS0__GPIO_1_18,
	MX50_PAD_EIM_EB0__GPIO_1_19,
	MX50_PAD_EIM_EB1__GPIO_1_20,
	MX50_PAD_EIM_WAIT__GPIO_1_21,
	MX50_PAD_EIM_BCLK__GPIO_1_22,
	MX50_PAD_EIM_RDY__GPIO_1_23,
	MX50_PAD_EIM_OE__GPIO_1_24,
};

/* Serial ports */
static const struct imxuart_platform_data uart_pdata __initconst = {
	.flags = IMXUART_HAVE_RTSCTS,
};

static const struct fec_platform_data fec_data __initconst = {
	.phy = PHY_INTERFACE_MODE_RMII,
};

static inline void mx50_rdp_fec_reset(void)
{
	gpio_request(FEC_EN, "fec-en");
	gpio_direction_output(FEC_EN, 0);
	gpio_request(FEC_RESET_B, "fec-reset_b");
	gpio_direction_output(FEC_RESET_B, 0);
	msleep(1);
	gpio_set_value(FEC_RESET_B, 1);
}

static const struct imxi2c_platform_data i2c_data __initconst = {
	.bitrate = 100000,
};

static struct mxc_gpu_platform_data gpu_data __initdata = {
	.z160_revision = 1,
};

static int mx50_rdp_spi_cs[] = {
	MX50_RDP_CSPI_CS0,
	MX50_RDP_CSPI_CS1,
};

static const struct spi_imx_master mx50_rdp_spi_pdata __initconst = {
	.chipselect     = mx50_rdp_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx50_rdp_spi_cs),
};

/* The GPMI is conflicted with SD3, so init this in the driver. */
static iomux_v3_cfg_t mx50_gpmi_nand[] __initdata = {
	MX50_PIN_EIM_DA8__NANDF_CLE,
	MX50_PIN_EIM_DA9__NANDF_ALE,
	MX50_PIN_EIM_DA10__NANDF_CE0,
	MX50_PIN_EIM_DA11__NANDF_CE1,
	MX50_PIN_EIM_DA12__NANDF_CE2,
	MX50_PIN_EIM_DA13__NANDF_CE3,
	MX50_PAD_EIM_DA14__NANDF_READY,
	MX50_PIN_EIM_DA15__NANDF_DQS,
	MX50_PIN_SD3_D4__NANDF_D0,
	MX50_PIN_SD3_D5__NANDF_D1,
	MX50_PIN_SD3_D6__NANDF_D2,
	MX50_PIN_SD3_D7__NANDF_D3,
	MX50_PIN_SD3_D0__NANDF_D4,
	MX50_PIN_SD3_D1__NANDF_D5,
	MX50_PIN_SD3_D2__NANDF_D6,
	MX50_PIN_SD3_D3__NANDF_D7,
	MX50_PIN_SD3_CLK__NANDF_RDN,
	MX50_PIN_SD3_CMD__NANDF_WRN,
	MX50_PIN_SD3_WP__NANDF_RESETN,
};

static int gpmi_nfc_platform_init(void)
{
	return mxc_iomux_v3_setup_multiple_pads(mx50_gpmi_nand,
					ARRAY_SIZE(mx50_gpmi_nand));
}

static struct gpmi_nfc_platform_data  mx50_gpmi_nfc_platform_data __initdata = {
	.platform_init           = gpmi_nfc_platform_init,
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          = 1,
};

/*
 * Board specific initialization.
 */
static void __init mx50_rdp_board_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx50_rdp_pads,
					ARRAY_SIZE(mx50_rdp_pads));

#if defined(CONFIG_CPU_FREQ_IMX)
	get_cpu_op = mx50_get_cpu_op;
#endif
	pr_info("CPU is iMX50 Revision %u\n",
		mx50_revision());

	imx50_add_cspi(3, &mx50_rdp_spi_pdata);

	imx50_add_dma();
	imx50_add_imx_uart(0, NULL);
	imx50_add_imx_uart(1, NULL);
	imx50_add_srtc();
	mx50_rdp_fec_reset();
	imx50_add_fec(&fec_data);
	imx50_add_gpmi(&mx50_gpmi_nfc_platform_data);
	imx50_add_imx_i2c(0, &i2c_data);
	imx50_add_imx_i2c(1, &i2c_data);
	imx50_add_imx_i2c(2, &i2c_data);
	imx50_add_mxc_gpu(&gpu_data);
	imx50_add_sdhci_esdhc_imx(0, NULL);
	imx50_add_sdhci_esdhc_imx(1, NULL);
	imx50_add_sdhci_esdhc_imx(2, NULL);
	imx50_add_otp();
	imx50_add_dcp();
	imx50_add_rngb();
	imx50_add_perfmon();
	mx50_rdp_init_mc13892();
}

static void __init mx50_rdp_timer_init(void)
{
	mx50_clocks_init(32768, 24000000, 22579200);
}

static struct sys_timer mx50_rdp_timer = {
	.init	= mx50_rdp_timer_init,
};

MACHINE_START(MX50_RDP, "Freescale MX50 Reference Design Platform")
	.map_io = mx50_map_io,
	.init_early = imx50_init_early,
	.init_irq = mx50_init_irq,
	.timer = &mx50_rdp_timer,
	.init_machine = mx50_rdp_board_init,
MACHINE_END
