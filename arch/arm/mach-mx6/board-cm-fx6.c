/*
 * CM-FX6 Module board file
 *
 * Copyright (C) 2013 CompuLab, Ltd.
 * Author: Igor Grinberg <grinberg@compulab.co.il>
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
 * with this program; if not, write to the Free Software Foundation, Inc.
 */

#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/i2c/at24.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/ion.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mxc_asrc.h>
#include <linux/mfd/mxc-hdmi-core.h>
#include <linux/ds2782_battery.h>
#include <linux/spi/scf0403.h>
#include <linux/spi/ads7846.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-cm-fx6-q.h"
#include "board-cm-fx6-dl.h"

/* GPIO PIN, sort by PORT/BIT */
#define CM_FX6_USBH1_PWR		IMX_GPIO_NR(1, 0)
#define SB_FX6_HIMAX_PENDOWN		IMX_GPIO_NR(1, 4)
#define CM_FX6_iSSD_SATA_PWREN		IMX_GPIO_NR(1, 28)
#define SB_FX6_GPIO_PWRBTN		IMX_GPIO_NR(1, 29)
#define CM_FX6_iSSD_SATA_VDDC_CTRL	IMX_GPIO_NR(1, 30)
#define CM_FX6_ADS7846_PENDOWN		IMX_GPIO_NR(2, 15)
#define CM_FX6_iSSD_SATA_LDO_EN		IMX_GPIO_NR(2, 16)
#define CM_FX6_ECSPI1_CS0		IMX_GPIO_NR(2, 30)
#define CM_FX6_GREEN_LED		IMX_GPIO_NR(2, 31)
#define CM_FX6_ECSPI1_CS1		IMX_GPIO_NR(3, 19)
#define CM_FX6_iSSD_SATA_nSTDBY1	IMX_GPIO_NR(3, 20)
#define CM_FX6_USB_OTG_PWR		IMX_GPIO_NR(3, 22)
#define CM_FX6_iSSD_SATA_PHY_SLP	IMX_GPIO_NR(3, 23)
#define CM_FX6_ECSPI2_CS2		IMX_GPIO_NR(3, 24)
#define CM_FX6_ECSPI2_CS3		IMX_GPIO_NR(3, 25)
#define CM_FX6_iSSD_SATA_STBY_REQ	IMX_GPIO_NR(3, 29)
#define CM_FX6_iSSD_SATA_nSTDBY2	IMX_GPIO_NR(5, 2)
#define CM_FX6_iSSD_SATA_nRSTDLY	IMX_GPIO_NR(6, 6)
#define CM_FX6_WIFI_nRESET		IMX_GPIO_NR(6, 16)
#define CM_FX6_iSSD_SATA_PWLOSS_INT	IMX_GPIO_NR(6, 31)
#define CM_FX6_SD3_WP			IMX_GPIO_NR(7, 0)
#define CM_FX6_SD3_CD			IMX_GPIO_NR(7, 1)
#define CM_FX6_USBHUB_nRST		IMX_GPIO_NR(7, 8)
#define CM_FX6_WIFI_nPD			IMX_GPIO_NR(7, 12)
#define SB_FX6_GPIO_HOMEBTN		IMX_GPIO_NR(7, 13)

#define SB_FX6_PCA9555_BASE_ADDR	IMX_GPIO_NR(8, 0)

#define SB_FX6_IO_EXP_GPIO(x)		(SB_FX6_PCA9555_BASE_ADDR + (x))

#define SB_FX6_DVIT_nPD			SB_FX6_IO_EXP_GPIO(2)
#define SB_FX6_DVIT_MSEN		SB_FX6_IO_EXP_GPIO(3)
#define SB_FX6_DVI_HPD			SB_FX6_DVIT_MSEN
#define SB_FX6_LCD_RST			SB_FX6_IO_EXP_GPIO(11)

#define CM_FX6_PCIE_PWR_EN		SB_FX6_IO_EXP_GPIO(14)
#define CM_FX6_PCIE_RESET		SB_FX6_IO_EXP_GPIO(14)


#define BMCR_PDOWN			0x0800 /* PHY Powerdown */
#define DS2786_RSNS			18	/* [Ohm] - sense resistor value */
#define EEPROM_1ST_MAC_OFF		4

#define MX6_SNVS_LPCR_REG		0x38

static struct clk *sata_clk;
static int gpmi_en;

extern char *soc_reg_id;
extern char *pu_reg_id;
extern unsigned int system_rev;

static u32 fsl_system_rev(void);


static const struct esdhc_platform_data cm_fx6_sd1_data = {
	.always_present         = 1,
	.keep_power_at_suspend  = 1,
};

static const struct esdhc_platform_data cm_fx6_sd3_data = {
	.cd_gpio		= CM_FX6_SD3_CD,
	.wp_gpio		= CM_FX6_SD3_WP,
	.support_8bit		= 0,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
	.cd_type		= ESDHC_CD_GPIO,
};

static void sb_fx6_sd_init(void)
{
	iomux_v3_cfg_t *sd3_pads;
	unsigned int sd3_pads_cnt;
	iomux_v3_cfg_t *sd1_pads;
	unsigned int sd1_pads_cnt;

	if (cpu_is_mx6q()) {
		sd3_pads = mx6q_sd3_200mhz;
		sd3_pads_cnt = ARRAY_SIZE(mx6q_sd3_200mhz);

		sd1_pads = mx6q_sd1_200mhz;
		sd1_pads_cnt = ARRAY_SIZE(mx6q_sd1_200mhz);
	} else if (cpu_is_mx6dl()) {
		sd3_pads = mx6dl_sd3_200mhz;
		sd3_pads_cnt = ARRAY_SIZE(mx6dl_sd3_200mhz);

		sd1_pads = mx6dl_sd1_200mhz;
		sd1_pads_cnt = ARRAY_SIZE(mx6dl_sd1_200mhz);
	}

	mxc_iomux_v3_setup_multiple_pads(sd3_pads, sd3_pads_cnt);
	mxc_iomux_v3_setup_multiple_pads(sd1_pads, sd1_pads_cnt);

	imx6q_add_sdhci_usdhc_imx(2, &cm_fx6_sd3_data);
	imx6q_add_sdhci_usdhc_imx(0, &cm_fx6_sd1_data);
}

static int gpmi_nand_platform_init(void)
{
	iomux_v3_cfg_t *nand_pads = NULL;
	u32 nand_pads_cnt;

	if (cpu_is_mx6q()) {
		nand_pads = mx6q_gpmi_nand;
		nand_pads_cnt = ARRAY_SIZE(mx6q_gpmi_nand);
	} else if (cpu_is_mx6dl()) {
		nand_pads = mx6dl_gpmi_nand;
		nand_pads_cnt = ARRAY_SIZE(mx6dl_gpmi_nand);

	}
	BUG_ON(!nand_pads);
	return mxc_iomux_v3_setup_multiple_pads(nand_pads, nand_pads_cnt);
}

static struct mtd_partition gpmi_nand_partitions[] = {
	{
		.name	= "nand",
		.offset	= 0,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct gpmi_nand_platform_data mx6_gpmi_nand_platform_data = {
	.platform_init           = gpmi_nand_platform_init,
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          = 1,
	.partitions		 = gpmi_nand_partitions,
	.partition_count	 = ARRAY_SIZE(gpmi_nand_partitions),
	.enable_bbt              = 1,
	.enable_ddr              = 0,
};

static const struct anatop_thermal_platform_data cm_fx6_anatop_thermal_data = {
	.name = "anatop_thermal",
};

static inline void cm_fx6_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(3, NULL);
	imx6q_add_imx_uart(4, NULL);
}

static int cm_fx6_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x8016);
	phy_write(phydev, 0xd, 0x4007);
	val = phy_read(phydev, 0xe);

	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, 0x1d, 0x5);
	val = phy_read(phydev, 0x1e);
	val |= 0x0100;
	phy_write(phydev, 0x1e, val);

	/* check phy power */
	val = phy_read(phydev, 0x0);
	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));
	return 0;
}

static int cm_fx6_fec_power_hibernate(struct phy_device *phydev)
{
	unsigned short val;

	/*set AR8031 debug reg 0xb to hibernate power*/
	phy_write(phydev, 0x1d, 0xb);
	val = phy_read(phydev, 0x1e);

	val |= 0x8000;
	phy_write(phydev, 0x1e, val);

	return 0;
}

static struct fec_platform_data fec_data = {
	.init			= cm_fx6_fec_phy_init,
	.power_hibernate	= cm_fx6_fec_power_hibernate,
	.phy			= PHY_INTERFACE_MODE_RGMII,
};

static int cm_fx6_spi0_cs[] = {
	CM_FX6_ECSPI1_CS0,
	CM_FX6_ECSPI1_CS1,
};

static int cm_fx6_spi1_cs[] = {
	CM_FX6_ECSPI2_CS2,
	CM_FX6_ECSPI2_CS3,
};

static const struct spi_imx_master cm_fx6_spi0_data __initconst = {
	.chipselect     = cm_fx6_spi0_cs,
	.num_chipselect = ARRAY_SIZE(cm_fx6_spi0_cs),
};

static const struct spi_imx_master cm_fx6_spi1_data __initconst = {
	.chipselect     = cm_fx6_spi1_cs,
	.num_chipselect = ARRAY_SIZE(cm_fx6_spi1_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition cm_fx6_spi_flash_partitions[] = {
	{
		.name	= "uboot",
		.offset	= 0,
		.size	= SZ_512K + SZ_256K,
	}, {
		.name	= "uboot environment",
		.offset	= MTDPART_OFS_APPEND,
		.size	= SZ_256K,
	}, {
		.name	= "reserved",
		.offset	= MTDPART_OFS_APPEND,
		.size	= MTDPART_SIZ_FULL,
	},
};

/*
 * The default cm-fx6 flash chip is 'sst25vf016b'.
 * It is JEDEC compliant, so we do not specify the .type field below.
 */
static struct flash_platform_data cm_fx6_spi_flash_data = {
	.name		= "spi_flash",
	.parts		= cm_fx6_spi_flash_partitions,
	.nr_parts	= ARRAY_SIZE(cm_fx6_spi_flash_partitions),
};
#endif

#if (defined CONFIG_TOUCHSCREEN_ADS7846) || (defined CONFIG_TOUCHSCREEN_ADS7846_MODULE)
static struct ads7846_platform_data ads7846_config = {
	.x_min			= 0x10a0,
	.x_max			= 0x1ef0,
	.y_min			= 0x1100,
	.y_max			= 0x1ef0,
	.reverse_y		= true,
	.x_plate_ohms		= 180,
	.pressure_max		= 255,
	.debounce_max		= 30,
	.debounce_tol		= 10,
	.debounce_rep		= 1,
	.gpio_pendown		= CM_FX6_ADS7846_PENDOWN,
	.keep_vref_on		= true,
};
#endif

static struct scf0403_pdata scf0403_config = {
	.reset_gpio	= SB_FX6_LCD_RST,
};

static struct spi_board_info cm_fx6_spi0_board_info[] = {
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
	{
		/* The modalias must be the same as spi device driver name */
		.modalias	= "m25p80",
		.max_speed_hz	= 20000000,
		.bus_num	= 0,
		.chip_select	= 0,
		.platform_data	= &cm_fx6_spi_flash_data,
	},
#endif
#if (defined CONFIG_TOUCHSCREEN_ADS7846) || (defined CONFIG_TOUCHSCREEN_ADS7846_MODULE)
	{
		.modalias	= "ads7846",
		.max_speed_hz	= 1500000,
		.bus_num	= 0,
		.chip_select	= 1,
		.irq		= gpio_to_irq(CM_FX6_ADS7846_PENDOWN),
		.platform_data	= &ads7846_config,
	},
#endif
};

static struct spi_board_info cm_fx6_spi1_board_info[] = {
#if defined(CONFIG_LCD_SCF0403) || defined(CONFIG_LCD_SCF0403_MODULE)
	{
		.modalias               = "scf0403",
		.max_speed_hz           = 1000000,
		.bus_num                = 1,
		.chip_select            = 1,
		.platform_data          = &scf0403_config,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(cm_fx6_spi0_board_info,
				ARRAY_SIZE(cm_fx6_spi0_board_info));
	spi_register_board_info(cm_fx6_spi1_board_info,
				ARRAY_SIZE(cm_fx6_spi1_board_info));
}

#if defined(CONFIG_I2C_IMX)
static void eeprom_read_mac_address(struct memory_accessor *ma, unsigned char *mac)
{
	int err;

	err = ma->read(ma, mac, EEPROM_1ST_MAC_OFF, ETH_ALEN);
	if (err < ETH_ALEN) {
		pr_warn("cm-fx6: could not read ID EEPROM: %d \n", err);
		memset(mac, 0, ETH_ALEN);
	}
}

static void cm_fx6_id_eeprom_setup(struct memory_accessor *ma, void *context)
{
	eeprom_read_mac_address(ma, fec_data.mac);
	imx6_init_fec(fec_data);
}

static struct at24_platform_data cm_fx6_id_eeprom_data = {
	.byte_len	= 256,
	.page_size	= 8,
	.flags		= AT24_FLAG_IRUGO,
	.setup		= cm_fx6_id_eeprom_setup,
	.context	= NULL,
};

static struct at24_platform_data eeprom_24c02 = {
	.byte_len	= 256,
	.page_size	= 8,
	.flags		= AT24_FLAG_IRUGO,
	.setup		= NULL,
	.context	= NULL,
};

static void cm_fx6_dvi_init(void)
{
	int err;

	err = gpio_request(SB_FX6_DVI_HPD, "dvi detect");
	if (err)
		return;

	gpio_direction_input(SB_FX6_DVI_HPD);
	gpio_export(SB_FX6_DVI_HPD, false);
}

static int cm_fx6_dvi_update(void)
{
	int value;

	/* value = gpio_get_value(SB_FX6_DVI_HPD); */
	value = 1;
	pr_info("DVI display: %s \n", (value ? "attach" : "detach"));
	return value;
}

static struct fsl_mxc_dvi_platform_data cm_fx6_dvi_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
	.init		= cm_fx6_dvi_init,
	.update		= cm_fx6_dvi_update,
};


static int pca9555_setup(struct i2c_client *client,
			 unsigned gpio_base, unsigned ngpio,
			 void *context)
{
	int i;
	/* GPIO value:
	 * 0   - skip
	 * < 0 - set to input
	 * > 0 - set to output, with value (gpio_value - 1)
	 */
	static int gpio_value[] = {2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2};

	pr_info("%s: init GPIO range %u..%u \n", __FUNCTION__, gpio_base, (gpio_base + ngpio));
	for (i = 0; i < ARRAY_SIZE(gpio_value); ++i) {
		if (gpio_value[i] == 0)
			continue;

		gpio_request((gpio_base + i), "PCA9555 GPIO Expander");
		if (gpio_value[i] < 0) {
			gpio_direction_input(gpio_base + i);
		}
		else {
			gpio_direction_output((gpio_base + i), (gpio_value[i] - 1));
		}
		gpio_export((gpio_base + i), 0);
	}

	return 0;
}

static struct pca953x_platform_data sb_fx6_gpio_expander_data = {
	.gpio_base	= SB_FX6_PCA9555_BASE_ADDR,
	.invert		= 0,
	.setup		= pca9555_setup,
};

struct ds278x_platform_data ds2786_volt_gauge_data = {
	.rsns = DS2786_RSNS,
};

static struct imxi2c_platform_data cm_fx6_i2c0_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data cm_fx6_i2c1_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data cm_fx6_i2c2_data = {
	.bitrate = 400000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
#ifdef CONFIG_EEPROM_AT24
	{
		/* EEPROM on the base board: 24c02 */
		I2C_BOARD_INFO("24c02", 0x50),
		.platform_data = &eeprom_24c02,
	},
#endif
#ifdef CONFIG_GPIO_PCA953X
	{
		I2C_BOARD_INFO("pca9555", 0x26),
		.platform_data = &sb_fx6_gpio_expander_data,
	},
#endif
#ifdef CONFIG_BATTERY_DS2782
	{
		I2C_BOARD_INFO("ds2786", 0x36),
		.platform_data = &ds2786_volt_gauge_data,
	},
#endif
#if (defined CONFIG_TOUCHSCREEN_HIMAX) || (defined CONFIG_TOUCHSCREEN_HIMAX_MODULE)
	{
		I2C_BOARD_INFO("hx8526-a", 0x4a),
		.irq = gpio_to_irq(SB_FX6_HIMAX_PENDOWN),
	},
#endif
#ifdef CONFIG_FB_MXC_EDID
	{
		I2C_BOARD_INFO("mxc_dvi", 0x1f),
		.irq = 0,	/* gpio_to_irq(SB_FX6_DVI_HPD) */
		.platform_data = &cm_fx6_dvi_data,
	},
#endif
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
#ifdef CONFIG_EEPROM_AT24
	{
		/* ID EEPROM on the module board: 24c02 */
		I2C_BOARD_INFO("24c02", 0x50),
		.platform_data = &cm_fx6_id_eeprom_data,
	},
#endif
#ifdef CONFIG_SND_SOC_IMX_WM8731
	{
		/* wm8731 audio codec */
		I2C_BOARD_INFO("wm8731", 0x1a),
	}
#endif
};

#if (defined CONFIG_TOUCHSCREEN_HIMAX) || (defined CONFIG_TOUCHSCREEN_HIMAX_MODULE)
static void __init sb_fx6_touchscreen_himax_init(void)
{
	int err;

	err = gpio_request_one(SB_FX6_HIMAX_PENDOWN, GPIOF_IN, "himax_pen");
	if (err) {
		pr_err("could not acquire gpio %u (himax_pen): %d \n", SB_FX6_HIMAX_PENDOWN, err);
		return;
	}

	gpio_export(SB_FX6_HIMAX_PENDOWN, false);
}

#else

static inline void sb_fx6_touchscreen_himax_init(void) {}
#endif	// CONFIG_TOUCHSCREEN_HIMAX

static void __init cm_fx6_init_i2c(void)
{
	sb_fx6_touchscreen_himax_init();

	imx6q_add_imx_i2c(0, &cm_fx6_i2c0_data);
	imx6q_add_imx_i2c(1, &cm_fx6_i2c1_data);
	imx6q_add_imx_i2c(2, &cm_fx6_i2c2_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));
}

#else

static void cm_fx6_init_i2c(void) {}
#endif	// CONFIG_I2C_IMX

static void icm_fx6_usbotg_vbus(bool on)
{
	pr_info("USB-OTG VBus %s \n", (on ? "ON" : "OFF"));
	gpio_set_value(CM_FX6_USB_OTG_PWR, (on ? 1 : 0));
}

static void __init cm_fx6_init_usb(void)
{
	int ret = 0;


	/* reset USB hub */
	ret = gpio_request(CM_FX6_USBHUB_nRST, "usb-hub-reset");
	if (ret) {
		pr_err("failed to get USBHUB_nRST GPIO: %d \n", ret);
		return;
	}
	gpio_direction_output(CM_FX6_USBHUB_nRST, 0);
	udelay(1);
	gpio_set_value(CM_FX6_USBHUB_nRST, 1);


	/* if no USB hub  -  power USB1 VBUS */
	ret = gpio_request(CM_FX6_USBH1_PWR, "usb1 cpen");
	if (ret) {
		pr_err("failed to get USBH1_PWR GPIO: %d \n", ret);
		return;
	}
	gpio_direction_output(CM_FX6_USBH1_PWR, 1);


	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	/* disable external charger detect,
	 * or it will affect signal quality at dp.
	 */

	ret = gpio_request(CM_FX6_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get USB_OTG_PWR GPIO: %d \n", ret);
		return;
	}
	gpio_direction_output(CM_FX6_USB_OTG_PWR, 0);

	/* usb-otg-id pin iomux select */
	mxc_iomux_set_gpr_register(1, 13, 1, 0);

	mx6_set_otghost_vbus_func(icm_fx6_usbotg_vbus);
	mx6_usb_dr_init();
}

static struct viv_gpu_platform_data imx6_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M + SZ_64M,
};


static struct gpio sata_issd_gpios[] = {

	{ CM_FX6_iSSD_SATA_VDDC_CTRL,	GPIOF_IN,		"sata vddc ctrl" },
	{ CM_FX6_iSSD_SATA_STBY_REQ,	GPIOF_IN,		"sata stby req" },

	{ CM_FX6_iSSD_SATA_PWLOSS_INT,	GPIOF_OUT_INIT_LOW,	"sata pwloss int" },
	{ CM_FX6_iSSD_SATA_PHY_SLP,	GPIOF_OUT_INIT_HIGH,	"sata phy slp" },
	{ CM_FX6_iSSD_SATA_nRSTDLY,	GPIOF_OUT_INIT_HIGH,	"sata nrst" },

	/* keep the order of power sequence ! */
	{ CM_FX6_iSSD_SATA_PWREN,	GPIOF_OUT_INIT_LOW,	"sata pwren" },		// VCC_IO
	{ CM_FX6_iSSD_SATA_nSTDBY1,	GPIOF_OUT_INIT_LOW,	"sata nstdby1" },	// VCC_FLASH
	{ CM_FX6_iSSD_SATA_nSTDBY2,	GPIOF_OUT_INIT_LOW,	"sata nstdby2" },	// VCCQ
	{ CM_FX6_iSSD_SATA_LDO_EN,	GPIOF_OUT_INIT_LOW,	"sata ldo en" },	// VDCC
};

/*
 * SSD i100 initialization
 * 
 * @signals
 * PWLOSS_INT - positive edge directs iSSD to perform quick shutdown
 * PHY_SLP - when initialized HIGH, iSSD should proceed to deep sleep when possible,
 *	when initialized LOW, it should be toggled HIGH by PM logic
 */
static int cm_fx6_iSSD_init(void)
{
	int i;
	int err;

	err = gpio_request_array(sata_issd_gpios, ARRAY_SIZE(sata_issd_gpios));
	if (err)
		return err;

	for (i = (ARRAY_SIZE(sata_issd_gpios) - 4); i < ARRAY_SIZE(sata_issd_gpios); ++i) {
		udelay(100);
		gpio_set_value(sata_issd_gpios[i].gpio, 1);
	}

	// DEBUG
	gpio_export(CM_FX6_iSSD_SATA_STBY_REQ, 0);	// in:  gpio93

	gpio_export(CM_FX6_iSSD_SATA_PWLOSS_INT, 0);	// out: gpio191
	gpio_export(CM_FX6_iSSD_SATA_PHY_SLP, 0);	// out: gpio87

	/*
	 * TODO:
	 * implement logic associated with input signals
	 */

	return 0;
}

static void cm_fx6_iSSD_cleanup(void)
{
	int i;

	for (i = (ARRAY_SIZE(sata_issd_gpios) - 4); i < ARRAY_SIZE(sata_issd_gpios); ++i) {
		gpio_set_value(sata_issd_gpios[i].gpio, 0);
	}

	gpio_free_array(sata_issd_gpios, ARRAY_SIZE(sata_issd_gpios));
}

/* HW Initialization, if return 0, initialization is successful. */
static int cm_fx6_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	ret = cm_fx6_iSSD_init();
	if (ret != 0) {
		dev_err(dev, "could not acquire iSSD GPIO: %d \n", ret);
		return ret;
	}

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		ret = PTR_ERR(sata_clk);
		goto release_drive;
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	tmpdata &= ~0x07FFFFFF;
	tmpdata |= 0x0593A4C4;		// 3.0 Gbps
	// tmpdata |= 0x059324C4;	// 1.5 Gbps
	writel(tmpdata, IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	tmpdata |= 0x2;
	writel(tmpdata, IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;

release_sata_clk:
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);
release_drive:
	cm_fx6_iSSD_cleanup();

	dev_err(dev, "disable SATA controller \n");
	return ret;
}

static void cm_fx6_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);

	cm_fx6_iSSD_cleanup();
}

static struct ahci_platform_data cm_fx6_sata_data = {
	.init	= cm_fx6_sata_init,
	.exit	= cm_fx6_sata_exit,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits	= 4,
	.clk_map_ver	= 2,
};

static struct ipuv3_fb_platform_data cm_fx6_fb_data[] = {
	{ /*fb0*/
	.disp_dev		= "ldb",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB24,
	.mode_str		= "LDB-XGA",
	.default_bpp		= 24,
	.int_clk		= false,
	}, {
	.disp_dev		= "hdmi",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB24,
	.mode_str		= "1920x1080M@60",
	.default_bpp		= 24,
	.int_clk		= false,
	}, {
	.disp_dev		= "dvi",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB24,
	.mode_str		= "1920x1080M@60",
	.default_bpp		= 24,
	.int_clk		= false,
	}, {
	/* Startek 800x480 LCD */
	.disp_dev		= "lcd",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB24,
	.mode_str		= "KD050C-WVGA",
	.default_bpp		= 24,
	.int_clk		= false,
	},

};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;
	int max_ipu_id = cpu_is_mx6q() ? 1 : 0;

	pr_info("%s: [ipu:disp] = [%d:%d] \n", __FUNCTION__, ipu_id, disp_id);
	if ((ipu_id > max_ipu_id) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2 * ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
};

static struct platform_device mxc_hdmi_audio_device = {
	.name           = "mxc_hdmi_audio",
	.id             = -1,
};

static void __init cm_fx6_init_hdmi(void)
{
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);
	imx6q_add_mxc_hdmi(&hdmi_data);
}

static void __init cm_fx6_init_hdmi_audio(void)
{
	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();
	platform_device_register(&mxc_hdmi_audio_device);
}

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
	.default_ifmt	= IPU_PIX_FMT_RGB666,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
	.ext_ref	= 1,
	.mode		= LDB_SEP0,
	.sec_ipu_id	= 0,
	.sec_disp_id	= 1,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
		.rev		= 4,
		.csi_clk[0]	= "clko_clk",
	}, {
		.rev		= 4,
		.csi_clk[0]	= "clko_clk",
	},
};

#if defined(CONFIG_BACKLIGHT_PWM)
static struct platform_pwm_backlight_data sb_fx6_pwm_backlight_data = {
	.pwm_id		= 2,
	.max_brightness	= 255,
	.dft_brightness	= 255,
	.pwm_period_ns	= 100000,
};

static void cm_fx6_pwm_init(void)
{
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm_backlight(2, &sb_fx6_pwm_backlight_data);
}

#else

static void cm_fx6_pwm_init(void) {}
#endif	// CONFIG_BACKLIGHT_PWM

static struct ion_platform_data imx_ion_data = {
	.nr = 1,
	.heaps = {
		{
		.type = ION_HEAP_TYPE_CARVEOUT,
		.name = "vpu_ion",
		.size = SZ_64M,
		},
	},
};

static void arm2_suspend_enter(void)
{
	/* suspend preparation */
	pr_info("----- suspend ----- \n");
}

static void arm2_suspend_exit(void)
{
	/* resume resore */
	pr_info("----- resume ------ \n");
}
static const struct pm_platform_data cm_fx6_pm_data __initconst = {
	.name		= "imx_pm",
	.suspend_enter	= arm2_suspend_enter,
	.suspend_exit	= arm2_suspend_exit,
};

static struct regulator_consumer_supply arm2_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data arm2_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(arm2_vmmc_consumers),
	.consumer_supplies = arm2_vmmc_consumers,
};

static struct fixed_voltage_config arm2_vmmc_reg_config = {
	.supply_name	= "vmmc",
	.microvolts	= 3300000,
	.gpio		= -1,
	.init_data	= &arm2_vmmc_init,
};

static struct platform_device arm2_vmmc_reg_devices = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev		= {
		.platform_data = &arm2_vmmc_reg_config,
	},
};

static struct imx_ssi_platform_data cm_fx6_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data cm_fx6_audio_data;
static struct {
	struct clk *pll;
	struct clk *clock_root;
	long current_rate;

} cm_fx6_audio_clocking_data;

static int wm8731_init(void)
{
	struct clk *new_parent;
	struct clk *ssi_clk;

	new_parent = clk_get(NULL, "pll4");
	if (IS_ERR(new_parent)) {
		pr_err("Could not get \"pll4\" clock \n");
		return PTR_ERR(new_parent);
	}

	ssi_clk = clk_get_sys("imx-ssi.1", NULL);
	if (IS_ERR(ssi_clk)) {
		pr_err("Could not get \"imx-ssi.1\" clock \n");
		return PTR_ERR(ssi_clk);
	}

	clk_set_parent(ssi_clk, new_parent);

	cm_fx6_audio_clocking_data.pll = new_parent;
	cm_fx6_audio_clocking_data.clock_root = ssi_clk;
	cm_fx6_audio_clocking_data.current_rate = 0;

	return 0;
}

static int wm8731_clock_enable(int enable)
{
	long pll_rate;
	long rate_req;
	long rate_avail;

	if ( !enable )
		return 0;

	if (cm_fx6_audio_data.sysclk == cm_fx6_audio_clocking_data.current_rate)
		return 0;

	switch (cm_fx6_audio_data.sysclk)
	{
	case 11289600:
		pll_rate = 632217600;
		break;

	case 12288000:
		pll_rate = 688128000;
		break;

	default:
		return -EINVAL;
	}

	rate_req = pll_rate;
	rate_avail = clk_round_rate(cm_fx6_audio_clocking_data.pll, rate_req);
	clk_set_rate(cm_fx6_audio_clocking_data.pll, rate_avail);

	rate_req = cm_fx6_audio_data.sysclk;
	rate_avail = clk_round_rate(cm_fx6_audio_clocking_data.clock_root, rate_req);
	clk_set_rate(cm_fx6_audio_clocking_data.clock_root, rate_avail);

	pr_info("%s: \"imx-ssi.1\" rate = %ld (= %ld) \n", __FUNCTION__, rate_avail, rate_req);
	cm_fx6_audio_clocking_data.current_rate = cm_fx6_audio_data.sysclk;
	return 0;
}

static struct platform_device cm_fx6_audio_device = {
	.name	= "imx-wm8731",
	.id	= -1,
};

static struct mxc_audio_platform_data cm_fx6_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 4,	/* AUDMUX: port[2] -> port[4] */
	.hp_gpio = -1,
	.mic_gpio = -1,
	.sysclk = 0,
	.init = wm8731_init,
	.clock_enable = wm8731_clock_enable,
};

static int __init cm_fx6_init_audio(void)
{
	iomux_v3_cfg_t *audmux_pads;
	int audmux_pads_cnt;

	if (cpu_is_mx6q()) {
		audmux_pads = cm_fx6_q_audmux_pads;
		audmux_pads_cnt = ARRAY_SIZE(cm_fx6_q_audmux_pads);
	}
	else if (cpu_is_mx6dl()) {
		audmux_pads = cm_fx6_dl_audmux_pads;
		audmux_pads_cnt = ARRAY_SIZE(cm_fx6_dl_audmux_pads);
	}
	mxc_iomux_v3_setup_multiple_pads(audmux_pads, audmux_pads_cnt);

	mxc_register_device(&cm_fx6_audio_device, &cm_fx6_audio_data);
	imx6q_add_imx_ssi(1, &cm_fx6_ssi_pdata);

	return 0;
}

static struct mxc_mlb_platform_data cm_fx6_mlb150_data = {
	.reg_nvcc		= NULL,
	.mlb_clk		= "mlb150_clk",
	.mlb_pll_clk		= "pll6",
};

static struct mxc_dvfs_platform_data arm2_dvfscore_data = {
	.reg_id			= "cpu_vddgp",
	.soc_id			= "cpu_vddsoc",
	.pu_id			= "cpu_vddvpu",
	.clk1_id		= "cpu_clk",
	.clk2_id		= "gpc_dvfs_clk",
	.gpc_cntr_offset	= MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset	= MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset	= MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset	= MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask		= 0x1F800,
	.prediv_offset		= 11,
	.prediv_val		= 3,
	.div3ck_mask		= 0xE0000000,
	.div3ck_offset		= 29,
	.div3ck_val		= 2,
	.emac_val		= 0x08,
	.upthr_val		= 25,
	.dnthr_val		= 9,
	.pncthr_val		= 33,
	.upcnt_val		= 10,
	.dncnt_val		= 10,
	.delay_time		= 80,
};

static void __init cm_fx6_fixup(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			/* GPU reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpumem=");
			if (str != NULL) {
				str += 7;
				imx6_gpu_pdata.reserved_mem_size = memparse(str, &str);
			}
			break;
		}
	}
}

static int __init early_enable_gpmi(char *p)
{
	gpmi_en = 1;
	return 0;
}

early_param("gpmi", early_enable_gpmi);

#ifdef CONFIG_SND_SOC_IMX_SPDIF
static int spdif_clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long rate_actual;
	rate_actual = clk_round_rate(clk, rate);
	clk_set_rate(clk, rate_actual);
	return 0;
}

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx		= 1,		/* enable tx */
	.spdif_rx		= 1,		/* enable rx */
	/*
	 * spdif0_clk will be 454.7MHz divided by ccm dividers.
	 *
	 * 44.1KHz: 454.7MHz / 7 (ccm) / 23 (spdif) = 44,128 Hz ~ 0.06% error
	 * 48KHz:   454.7MHz / 4 (ccm) / 37 (spdif) = 48,004 Hz ~ 0.01% error
	 * 32KHz:   454.7MHz / 6 (ccm) / 37 (spdif) = 32,003 Hz ~ 0.01% error
	 */
	.spdif_clk_44100	= 1,    /* tx clk from spdif0_clk_root */
	.spdif_clk_48000	= 1,    /* tx clk from spdif0_clk_root */
	.spdif_div_44100	= 23,
	.spdif_div_48000	= 37,
	.spdif_div_32000	= 37,
	.spdif_rx_clk		= 0,    /* rx clk from spdif stream */
	.spdif_clk_set_rate	= spdif_clk_set_rate,
	.spdif_clk		= NULL, /* spdif bus clk */
};

static void __init cm_fx6_spdif_init(void)
{
	iomux_v3_cfg_t *spdif_pads;
	int spdif_pads_cnt;

	if (cpu_is_mx6q()) {
		spdif_pads = cm_fx6_q_spdif_pads;
		spdif_pads_cnt = ARRAY_SIZE(cm_fx6_q_spdif_pads);
	}
	else if (cpu_is_mx6dl()) {
		spdif_pads = cm_fx6_dl_spdif_pads;
		spdif_pads_cnt = ARRAY_SIZE(cm_fx6_dl_spdif_pads);
	}
	mxc_iomux_v3_setup_multiple_pads(spdif_pads, spdif_pads_cnt);

	mxc_spdif_data.spdif_core_clk = clk_get_sys("mxc_spdif.0", NULL);
	clk_put(mxc_spdif_data.spdif_core_clk);
	imx6q_add_spdif(&mxc_spdif_data);
	imx6q_add_spdif_dai();
	imx6q_add_spdif_audio_device();
}

#else

static void cm_fx6_spdif_init(void) {}
#endif	// CONFIG_SND_SOC_IMX_SPDIF

static const struct imx_pcie_platform_data cm_fx6_pcie_data = {
	.pcie_pwr_en	= CM_FX6_PCIE_PWR_EN,
	.pcie_rst	= CM_FX6_PCIE_RESET,
	.pcie_wake_up	= -EINVAL,
	.pcie_dis	= -EINVAL,
};


#if defined(CONFIG_LEDS_GPIO) && defined(CONFIG_LEDS_TRIGGER_HEARTBEAT)
static struct gpio_led cm_fx6_leds[] = {
	[0] = {
		.gpio			= CM_FX6_GREEN_LED,
		.name			= "cm_fx6:load",
		.default_trigger	= "heartbeat",
		.active_low		= 0,
	},
};

static struct gpio_led_platform_data cm_fx6_led_pdata = {
	.num_leds	= ARRAY_SIZE(cm_fx6_leds),
	.leds		= cm_fx6_leds,
};

static struct platform_device cm_fx6_led_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &cm_fx6_led_pdata,
	},
};

static void __init cm_fx6_init_led(void)
{
	platform_device_register(&cm_fx6_led_device);
}
#else
static inline void cm_fx6_init_led(void) {}
#endif


#if defined(CONFIG_KEYBOARD_GPIO)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake, debounce)	\
{									\
	.gpio			= gpio_num,				\
	.type			= EV_KEY,				\
	.code			= ev_code,				\
	.active_low		= act_low,				\
	.desc			= "gpio " descr,			\
	.wakeup			= wake,					\
	.debounce_interval	= debounce,				\
}

static struct gpio_keys_button cm_fx6_buttons[] = {
	GPIO_BUTTON(SB_FX6_GPIO_PWRBTN, KEY_POWER, 1, "power-button", 1, 1),
	GPIO_BUTTON(SB_FX6_GPIO_HOMEBTN, KEY_HOMEPAGE, 1, "home-button", 0, 1),
};

static struct gpio_keys_platform_data cm_fx6_button_data = {
	.buttons		= cm_fx6_buttons,
	.nbuttons		= ARRAY_SIZE(cm_fx6_buttons),
};

static struct platform_device cm_fx6_button_device = {
	.name			= "gpio-keys",
	.id			= -1,
	.num_resources	 	= 0,
};

static void __init imx6q_add_device_buttons(void)
{
	platform_device_add_data(&cm_fx6_button_device,
				 &cm_fx6_button_data,
				 sizeof(cm_fx6_button_data));
	platform_device_register(&cm_fx6_button_device);
}
#else
static void __init imx6q_add_device_buttons(void) {}
#endif


static struct gpio cm_fx6_wifi_gpios[] = {

	{ CM_FX6_WIFI_nPD,	GPIOF_OUT_INIT_HIGH,	"wifi pdn" },
	{ CM_FX6_WIFI_nRESET,	GPIOF_OUT_INIT_LOW,	"wifi rstn" },
};

static void cm_fx6_init_wifi(void)
{
	int err;

	err = gpio_request_array(cm_fx6_wifi_gpios, ARRAY_SIZE(cm_fx6_wifi_gpios));
	if (err) {
		pr_err("CM-FX6: failed to request wifi gpios: %d \n", err);
	} else {
		msleep(1);
		gpio_set_value(CM_FX6_WIFI_nRESET, 1);
	}

	gpio_export(CM_FX6_WIFI_nPD, false);
	gpio_export(CM_FX6_WIFI_nRESET, false);
}


static void __init cm_fx6_init_ipu(void)
{
	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q())
		imx6q_add_ipuv3(1, &ipu_data[1]);
}

static void __init cm_fx6_init_display(void)
{
	int i;
	int count;

	count = ARRAY_SIZE(cm_fx6_fb_data);
	if (cpu_is_mx6dl())
		count /= 2;

	imx6q_add_vdoa();
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);

	for (i = 0; i < count; ++i)
		imx6q_add_ipuv3fb(i, &cm_fx6_fb_data[i]);
}


static void mx6_snvs_poweroff(void)
{
	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;

	pr_info("Turn off system power \n");
	value = readl(mx6_snvs_base + MX6_SNVS_LPCR_REG);
	/* set TOP and DP_EN bits */
	value |= 0x0060;
	writel(value, (mx6_snvs_base + MX6_SNVS_LPCR_REG));
}


/*!
 * Board specific initialization.
 */
static void __init cm_fx6_init(void)
{
	iomux_v3_cfg_t *common_pads = NULL;
	int common_pads_cnt;


	/* 
	 * Override system revision number to meet conventions set by Freescale. 
	 * According to Freescale, system revision should be set in this very 
	 * way by U-Boot and be passed to the kernel via atags. 
	 */
	system_rev = fsl_system_rev();


	/*
	 * common pads: pads are non-shared with others on this board
	 * feature_pds: pads are shared with others on this board
	 */

	if (cpu_is_mx6q()) {
		common_pads = cm_fx6_q_pads;
		common_pads_cnt = ARRAY_SIZE(cm_fx6_q_pads);
	} else if (cpu_is_mx6dl()) {
		common_pads = cm_fx6_dl_pads;
		common_pads_cnt = ARRAY_SIZE(cm_fx6_dl_pads);
	}

	BUG_ON(!common_pads);
	mxc_iomux_v3_setup_multiple_pads(common_pads, common_pads_cnt);

	/*
	 * the following is the common devices support on the shared ARM2 boards
	 * Since i.MX6DQ/DL share the same memory/Register layout, we don't
	 * need to diff the i.MX6DQ or i.MX6DL here. We can simply use the
	 * mx6q_add_features() for the shared devices. For which only exist
	 * on each indivual SOC, we can use cpu_is_mx6q/6dl() to diff it.
	 */

	gp_reg_id = arm2_dvfscore_data.reg_id;
	soc_reg_id = arm2_dvfscore_data.soc_id;
	pu_reg_id = arm2_dvfscore_data.pu_id;
	cm_fx6_init_uart();

	cm_fx6_init_ipu();

	imx6q_add_imx_snvs_rtc();
	imx6q_add_imx_snvs_pwrkey();
	pm_power_off = mx6_snvs_poweroff;

	imx6q_add_imx_caam();

	cm_fx6_init_i2c();
	cm_fx6_init_led();
	imx6q_add_device_buttons();
	cm_fx6_init_wifi();

	/* SPI */
	imx6q_add_ecspi(0, &cm_fx6_spi0_data);
	imx6q_add_ecspi(1, &cm_fx6_spi1_data);
	spi_device_init();

	imx6q_add_anatop_thermal_imx(1, &cm_fx6_anatop_thermal_data);

	// imx6_init_fec(fec_data); -- called asynchronously by cm_fx6_id_eeprom_setup()

	imx6q_add_pm_imx(0, &cm_fx6_pm_data);
	sb_fx6_sd_init();
	imx_add_viv_gpu(&imx6_gpu_data, &imx6_gpu_pdata);
	if (cpu_is_mx6q())
		imx6q_add_ahci(0, &cm_fx6_sata_data);
	imx6q_add_vpu();
	cm_fx6_init_usb();
	cm_fx6_init_audio();
	platform_device_register(&arm2_vmmc_reg_devices);
	mx6_cpu_regulator_init();

	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();
	if (gpmi_en)
		imx6q_add_gpmi(&mx6_gpmi_nand_platform_data);

	imx6q_add_dvfs_core(&arm2_dvfscore_data);

	imx6q_add_ion(0, &imx_ion_data,
		sizeof(imx_ion_data) + sizeof(struct ion_platform_heap));

	cm_fx6_pwm_init();

	cm_fx6_spdif_init();

	/* can1 can optionally be supported */
	imx6q_add_flexcan0(NULL);

	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
	imx6q_add_mlb150(&cm_fx6_mlb150_data);

	/* Add PCIe RC interface support */
	imx6q_add_pcie(&cm_fx6_pcie_data);
	imx6q_add_busfreq();
}

static int __init cm_fx6_init_late(void)
{
	if (!machine_is_cm_fx6())
		return -ENODEV;

	cm_fx6_init_hdmi();
	cm_fx6_init_display();
	cm_fx6_init_hdmi_audio();
	return 0;
}
device_initcall_sync(cm_fx6_init_late);

#define CM_FX6_MIN_SOC_VOLTAGE	1250000
#define CM_FX6_MIN_PU_VOLTAGE	1250000

static void cm_fx6_adjust_cpu_op(void)
{
	struct cpu_op *op;
	int num;

	if (cpu_is_mx6q()) {
		op = mx6_get_cpu_op(&num);
		if (!op)
			return;

		while (--num >= 0) {
			if (op[num].soc_voltage < CM_FX6_MIN_SOC_VOLTAGE)
				op[num].soc_voltage = CM_FX6_MIN_SOC_VOLTAGE;
			if (op[num].pu_voltage < CM_FX6_MIN_PU_VOLTAGE)
				op[num].pu_voltage = CM_FX6_MIN_PU_VOLTAGE;
		}
	}
}

extern void __iomem *twd_base;
static void __init mx6_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	cm_fx6_adjust_cpu_op();
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.3", NULL);
	early_console_setup(UART4_BASE_ADDR, uart_clk);
}

static struct sys_timer mxc_timer = {
	.init   = mx6_timer_init,
};

static void __init cm_fx6_reserve(void)
{
	phys_addr_t phys;
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	if (imx6_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(
			imx6_gpu_pdata.reserved_mem_size, SZ_4K, SZ_2G);
		memblock_remove(phys, imx6_gpu_pdata.reserved_mem_size);
		imx6_gpu_pdata.reserved_mem_base = phys;
	}
#endif

#if defined(CONFIG_ION)
	if (imx_ion_data.heaps[0].size) {
		phys = memblock_alloc(imx_ion_data.heaps[0].size, SZ_4K);
		memblock_free(phys, imx_ion_data.heaps[0].size);
		memblock_remove(phys, imx_ion_data.heaps[0].size);
		imx_ion_data.heaps[0].base = phys;
	}
#endif
}


/*
 * Set fsl_system_rev:
 * bit 0-7: Chip Revision ID
 * bit 8-11: Board Revision ID
 *     0: Unknown or latest revision
 *     1: RevA Board
 *     2: RevB board
 *     3: RevC board
 * bit 12-19: Chip Silicon ID
 *     0x63: i.MX6 Dual/Quad
 *     0x61: i.MX6 Solo/DualLite
 *     0x60: i.MX6 SoloLite
 */
static u32 fsl_system_rev(void)
{
	/* Read Silicon information from Anatop register */
	/* The register layout:
	 * bit 16-23: Chip Silicon ID
	 * 0x60: i.MX6 SoloLite
	 * 0x61: i.MX6 Solo/DualLite
	 * 0x63: i.MX6 Dual/Quad
	 *
	 * bit 0-7: Chip Revision ID
	 * 0x00: TO1.0
	 * 0x01: TO1.1
	 * 0x02: TO1.2
	 *
	 * exp:
	 * Chip             Major    Minor
	 * i.MX6Q1.0:       6300     00
	 * i.MX6Q1.1:       6300     01
	 * i.MX6Solo1.0:    6100     00

	 * Thus the system_rev will be the following layout:
	 * | 31 - 20 | 19 - 12 | 11 - 8 | 7 - 0 |
	 * | resverd | CHIP ID | BD REV | SI REV |
	 */
	u32 fsl_system_rev = 0;

	u32 cpu_type = readl(IO_ADDRESS(ANATOP_BASE_ADDR + 0x260));

	/* Chip Silicon ID */
	fsl_system_rev = ((cpu_type >> 16) & 0xFF) << 12;
	/* Chip silicon major revision */
	fsl_system_rev |= ((cpu_type >> 8) & 0xFF) << 4;
	fsl_system_rev += 0x10;
	/* Chip silicon minor revision */
	fsl_system_rev |= cpu_type & 0xFF;

	return fsl_system_rev;
}


MACHINE_START(CM_FX6, "CompuLab i.MX6 CM-FX6 Module")
	.boot_params	= MX6_PHYS_OFFSET + 0x100,
	.fixup		= cm_fx6_fixup,
	.map_io		= mx6_map_io,
	.init_irq	= mx6_init_irq,
	.init_machine	= cm_fx6_init,
	.timer		= &mxc_timer,
	.reserve	= cm_fx6_reserve,
MACHINE_END

