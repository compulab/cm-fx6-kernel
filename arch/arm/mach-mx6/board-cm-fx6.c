/*
 * Copyright (C) 2013 CompuLab, Ltd.
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
#include <linux/gpio-i2cmux.h>
#include <linux/spi/scf0403.h>
#include <linux/spi/ads7846.h>

#include <mach/common.h>
#include <mach/hardware.h>
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
#define SB_FX6_DVI_DDC_SEL		IMX_GPIO_NR(1, 2)
#define SB_FX6_HIMAX_PENDOWN		IMX_GPIO_NR(1, 4)
#define CM_FX6_iSSD_SATA_PWREN		IMX_GPIO_NR(1, 28)
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
#define CM_FX6_CAN2_EN			IMX_GPIO_NR(5, 24)
#define CM_FX6_iSSD_SATA_nRSTDLY	IMX_GPIO_NR(6, 6)
#define CM_FX6_iSSD_SATA_PWLOSS_INT	IMX_GPIO_NR(6, 31)
#define SB_FX6_SD3_WP			IMX_GPIO_NR(7, 0)
#define SB_FX6_SD3_CD			IMX_GPIO_NR(7, 1)
#define CM_FX6_USBHUB_nRST		IMX_GPIO_NR(7, 8)
#define CM_FX6_CAN1_STBY		IMX_GPIO_NR(7, 12)
#define CM_FX6_CAN1_EN			IMX_GPIO_NR(7, 13)

#define SB_FX6_PCA9555_BASE_ADDR	IMX_GPIO_NR(8, 0)

#define SB_FX6_IO_EXP_GPIO(x)		(SB_FX6_PCA9555_BASE_ADDR + (x))

#define CM_FX6_PCIE_PWR_EN		-1
#define CM_FX6_PCIE_RESET		-1
#define CM_FX6_CAN2_STBY		-1
#define SB_FX6_LCD_RST			SB_FX6_IO_EXP_GPIO(11)

#define BMCR_PDOWN			0x0800 /* PHY Powerdown */
#define EEPROM_1ST_MAC_OFF		4
#define EEPROM_BOARD_NAME_OFF           128
#define EEPROM_BOARD_NAME_LEN           16

#define MX6_SNVS_LPCR_REG		0x38

static struct clk *sata_clk;
static int spdif_en;
static int flexcan_en;
static unsigned int board_rev;

extern char *soc_reg_id;
extern char *pu_reg_id;

static void fsl_system_rev(void);
static void sb_fx6_gpio_expander_register(void);
static void sb_fx6_touchscreen_himax_register(void);

static const struct esdhc_platform_data cm_fx6_sd1_data __initconst = {
	.always_present         = 1,
	.keep_power_at_suspend  = 1,
};

static struct esdhc_platform_data cm_fx6_sd3_data __initdata = {
	.cd_gpio		= -1,
	.wp_gpio		= -1,
	.cd_type		= ESDHC_CD_NONE,
	.always_present		= 1,	/* ! */
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
};

static void sb_fx6_sd_init(void)
{
	iomux_v3_cfg_t *sd3_pads = NULL;
	unsigned int sd3_pads_cnt = 0;
	iomux_v3_cfg_t *sd1_pads = NULL;
	unsigned int sd1_pads_cnt = 0;

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
		nand_pads = cm_fx6_q_gpmi_nand;
		nand_pads_cnt = ARRAY_SIZE(cm_fx6_q_gpmi_nand);
	} else if (cpu_is_mx6dl()) {
		nand_pads = cm_fx6_dl_gpmi_nand;
		nand_pads_cnt = ARRAY_SIZE(cm_fx6_dl_gpmi_nand);

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

static struct gpmi_nand_platform_data cm_fx6_gpmi_nand_pdata = {
	.platform_init           = gpmi_nand_platform_init,
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          = 1,
	.partitions		 = gpmi_nand_partitions,
	.partition_count	 = ARRAY_SIZE(gpmi_nand_partitions),
	.enable_bbt              = 1,
	.enable_ddr              = 0,
};

static const struct anatop_thermal_platform_data
	cm_fx6_anatop_thermal_data __initconst = {
	.name = "anatop_thermal",
};

static const struct imxuart_platform_data cm_fx6_uart1_data __initconst = {
	.flags      = IMXUART_HAVE_RTSCTS | IMXUART_USE_DCEDTE | IMXUART_SDMA,
	.dma_req_rx = MX6Q_DMA_REQ_UART2_RX,
	.dma_req_tx = MX6Q_DMA_REQ_UART2_TX,
};

static inline void cm_fx6_init_uart(void)
{
	imx6q_add_imx_uart(3, NULL);
	imx6q_add_imx_uart(1, &cm_fx6_uart1_data);
}

static int cm_fx6_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	/* Ar8031 phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(phydev, 0xd, 0x3);
	phy_write(phydev, 0xe, 0x805d);
	phy_write(phydev, 0xd, 0x4003);
	val = phy_read(phydev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(phydev, 0xe, val);

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

static struct fec_platform_data fec_data __initdata = {
	.init			= cm_fx6_fec_phy_init,
	.power_hibernate	= cm_fx6_fec_power_hibernate,
	.phy			= PHY_INTERFACE_MODE_RGMII,
	.gpio_irq		= -1,
};

static void cm_fx6_spi_device_register(int busnum, struct spi_board_info *info,
				       const char *name)
{
	struct spi_master *master = spi_busnum_to_master(busnum);
	if (!master) {
		pr_err("%s: SPI-%d: could not get master \n", __func__, busnum);
		return;
	}

	if (!spi_new_device(master, info)) {
		pr_err("%s: SPI-%d: could not instantiate device %s \n",
		       __func__, busnum, name);
	}

	spi_master_put(master);
}

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
 * The default cm-fx6 flash chip is 'sst25vf016b',
 * which is JEDEC compliant, so we do not specify the 'type' field below.
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

#if defined(CONFIG_LCD_SCF0403) || defined(CONFIG_LCD_SCF0403_MODULE)
static struct scf0403_pdata scf0403_config = {
	.reset_gpio	= SB_FX6_LCD_RST,
};

static struct spi_board_info scf0403_board_info = {
	.modalias               = "scf0403",
	.max_speed_hz           = 1000000,
	.bus_num                = 1,
	.chip_select            = 1,
	.platform_data          = &scf0403_config,
};

static void scf0403_lcd_register(void)
{
	cm_fx6_spi_device_register(1, &scf0403_board_info, "DataImage LCD");
}

#else

static void scf0403_lcd_register(void) {}
#endif

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
	/* FIXME */
};

static void cm_fx6_init_spi(void)
{
	imx6q_add_ecspi(0, &cm_fx6_spi0_data);
	imx6q_add_ecspi(1, &cm_fx6_spi1_data);

	spi_register_board_info(cm_fx6_spi0_board_info,
				ARRAY_SIZE(cm_fx6_spi0_board_info));
	spi_register_board_info(cm_fx6_spi1_board_info,
				ARRAY_SIZE(cm_fx6_spi1_board_info));
}

#if defined(CONFIG_I2C_IMX)
static void cm_fx6_i2c_device_register(int busnum, struct i2c_board_info *info,
				       const char *name)
{
	struct i2c_adapter *i2c_adapter;

	i2c_adapter = i2c_get_adapter(busnum);
	if (!i2c_adapter) {
		pr_err("%s: I2C-%d: could not get adapter \n", __func__, busnum);
		return;
	}

	if (!i2c_new_device(i2c_adapter, info)) {
		pr_err("%s: I2C-%d: could not instantiate device %s \n",
		       __func__, busnum, name);
	}

	i2c_put_adapter(i2c_adapter);
}

static void __init sb_fx6_init(void)
{
	pr_info("CM-FX6: set up SB-FX6 evaluation board \n");

	/* re-configure SD3 for insertion / removal detection */
	cm_fx6_sd3_data.cd_gpio		= SB_FX6_SD3_CD;
	cm_fx6_sd3_data.wp_gpio		= SB_FX6_SD3_WP;
	cm_fx6_sd3_data.cd_type		= ESDHC_CD_GPIO;
	cm_fx6_sd3_data.always_present	= 0;

	sb_fx6_gpio_expander_register();
	scf0403_lcd_register();
	sb_fx6_touchscreen_himax_register();
}

static void __init sb_fx6m_init(void)
{
	pr_info("CM-FX6: set up SB-FX6m - Utilite device \n");
}

static void eeprom_read_mac_address(struct memory_accessor *ma, unsigned char *mac)
{
	int err;

	err = ma->read(ma, mac, EEPROM_1ST_MAC_OFF, ETH_ALEN);
	if (err < ETH_ALEN) {
		pr_warn("%s: could not read MAC address: %d \n", __func__, err);
		memset(mac, 0, ETH_ALEN);
	}
}

static void cm_fx6_id_eeprom_setup(struct memory_accessor *ma, void *context)
{
	eeprom_read_mac_address(ma, fec_data.mac);
	imx6_init_fec(fec_data);
}

static void sb_fx6_id_eeprom_setup(struct memory_accessor *ma, void *context)
{
	char baseboard[EEPROM_BOARD_NAME_LEN];
	int err;

	/* read base-board ID */
	err = ma->read(ma, baseboard, EEPROM_BOARD_NAME_OFF,
		       EEPROM_BOARD_NAME_LEN);
	if (err < 0) {
		pr_warn("%s: could not read base-board ID: %d \n",
			__func__, err);
		memcpy(baseboard, "undefined", 10);
	}
	baseboard[EEPROM_BOARD_NAME_LEN - 1] = '\0';
	pr_info("CM-FX6: start on %s board \n", baseboard);
	if (!strncmp(baseboard, "SB-FX6m", EEPROM_BOARD_NAME_LEN))
		sb_fx6m_init();
	else
		sb_fx6_init();

	sb_fx6_sd_init();
}

static struct at24_platform_data cm_fx6_id_eeprom_data = {
	.byte_len	= 256,
	.page_size	= 16,
	.flags		= AT24_FLAG_IRUGO,
	.setup		= cm_fx6_id_eeprom_setup,
	.context	= NULL,
};

static struct at24_platform_data sb_fx6_id_eeprom_data = {
	.byte_len	= 256,
	.page_size	= 16,
	.flags		= AT24_FLAG_IRUGO,
	.setup		= sb_fx6_id_eeprom_setup,
	.context	= NULL,
};

#ifdef CONFIG_GPIO_PCA953X
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

static struct i2c_board_info sb_fx6_gpio_expander_info = {
	I2C_BOARD_INFO("pca9555", 0x26),
	.platform_data = &sb_fx6_gpio_expander_data,
};

static void sb_fx6_gpio_expander_register(void)
{
	cm_fx6_i2c_device_register(3, &sb_fx6_gpio_expander_info, "GPIO expander");
}

#else

static void sb_fx6_gpio_expander_register(void) {}
#endif

#if (defined CONFIG_TOUCHSCREEN_HIMAX) || (defined CONFIG_TOUCHSCREEN_HIMAX_MODULE)
static struct i2c_board_info sb_fx6_ts_himax_info = {
	I2C_BOARD_INFO("hx8526-a", 0x4a),
	.irq = gpio_to_irq(SB_FX6_HIMAX_PENDOWN),
};

static void sb_fx6_touchscreen_himax_register(void)
{
	int err;

	err = gpio_request_one(SB_FX6_HIMAX_PENDOWN, GPIOF_IN, "himax_pen");
	if (err) {
		pr_err("could not acquire gpio %u (himax_pen): %d \n", SB_FX6_HIMAX_PENDOWN, err);
		return;
	}

	gpio_export(SB_FX6_HIMAX_PENDOWN, false);

	cm_fx6_i2c_device_register(3, &sb_fx6_ts_himax_info, "HIMAX touch controller");
}

#else

static inline void sb_fx6_touchscreen_himax_register(void) {}
#endif

static struct imxi2c_platform_data cm_fx6_i2c0_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data cm_fx6_i2c1_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data cm_fx6_i2c2_data = {
	.bitrate = 400000,
};

static struct i2c_board_info mxc_i2c0_3_board_info[] __initdata = {
#ifdef CONFIG_EEPROM_AT24
	{
		/* EEPROM on the base board: 24c02 */
		I2C_BOARD_INFO("24c02", 0x50),
		.platform_data = &sb_fx6_id_eeprom_data,
	},
#endif
};

static struct i2c_board_info mxc_i2c0_4_board_info[] __initdata = {
	/* FIXME */
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
};


static const unsigned sb_fx6_i2cmux_gpios[] = {
	SB_FX6_DVI_DDC_SEL
};

static const unsigned sb_fx6_i2cmux_values[] = {
	0, 1
};

static struct gpio_i2cmux_platform_data sb_fx6_i2cmux_data = {
	.parent		= 0,			/* multiplex I2C-0 */
	.base_nr	= 3,			/* create I2C-3+ */
	.gpios		= sb_fx6_i2cmux_gpios,
	.n_gpios	= ARRAY_SIZE(sb_fx6_i2cmux_gpios),
	.values		= sb_fx6_i2cmux_values,
	.n_values	= ARRAY_SIZE(sb_fx6_i2cmux_values),
	.idle		= GPIO_I2CMUX_NO_IDLE,
};

static struct platform_device sb_fx6_i2cmux = {
	.name		= "gpio-i2cmux",
	.id		= -1,
	.dev		= {
		.platform_data = &sb_fx6_i2cmux_data,
	},
};


static void __init cm_fx6_init_i2c(void)
{
	imx6q_add_imx_i2c(0, &cm_fx6_i2c0_data);
	imx6q_add_imx_i2c(1, &cm_fx6_i2c1_data);
	imx6q_add_imx_i2c(2, &cm_fx6_i2c2_data);
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));

	/* I2C multiplexing: I2C-0 --> I2C-3, I2C-4 */
	platform_device_register(&sb_fx6_i2cmux);
	i2c_register_board_info(3, mxc_i2c0_3_board_info,
				ARRAY_SIZE(mxc_i2c0_3_board_info));
	i2c_register_board_info(4, mxc_i2c0_4_board_info,
				ARRAY_SIZE(mxc_i2c0_4_board_info));
}

#else

static void cm_fx6_init_i2c(void) {}
#endif	// CONFIG_I2C_IMX

static void cm_fx6_usbotg_vbus(bool on)
{
	pr_info("USB-OTG VBus %s \n", (on ? "ON" : "OFF"));
	gpio_set_value(CM_FX6_USB_OTG_PWR, (on ? 1 : 0));
}

static void __init cm_fx6_init_usb(void)
{
	int ret = 0;


	/* reset USB hub */
	gpio_request_one(CM_FX6_USBHUB_nRST, GPIOF_INIT_LOW, "usb-hub-reset");
	if (ret) {
		pr_err("failed to request USBHUB_nRST GPIO: %d \n", ret);
		return;
	}
	udelay(1);
	gpio_set_value(CM_FX6_USBHUB_nRST, 1);


	/* if no USB hub  -  power USB1 VBUS */
	ret = gpio_request_one(CM_FX6_USBH1_PWR, GPIOF_INIT_HIGH, "usb1 cpen");
	if (ret) {
		pr_err("failed to request USBH1_PWR GPIO: %d \n", ret);
		return;
	}


	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	/* disable external charger detect,
	 * or it will affect signal quality at dp.
	 */

	ret = gpio_request_one(CM_FX6_USB_OTG_PWR, GPIOF_INIT_LOW, "usb-pwr");
	if (ret) {
		pr_err("failed to request USB_OTG_PWR GPIO: %d \n", ret);
		return;
	}

	/* usb-otg-id pin iomux select */
	mxc_iomux_set_gpr_register(1, 13, 1, 0);

	mx6_set_otghost_vbus_func(cm_fx6_usbotg_vbus);
}

static struct viv_gpu_platform_data imx6_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M + SZ_64M,
};

#ifdef CONFIG_SATA_AHCI_PLATFORM
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
	gpio_export(CM_FX6_iSSD_SATA_STBY_REQ, false);		// in:  gpio93

	gpio_export(CM_FX6_iSSD_SATA_PWLOSS_INT, false);	// out: gpio191
	gpio_export(CM_FX6_iSSD_SATA_PHY_SLP, false);		// out: gpio87

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
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
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

static void __init cm_fx6_init_sata(void)
{
	imx6q_add_ahci(0, &cm_fx6_sata_data);
}

#else

/*
 * When SATA is not enabled, disable AHCI phy by inserting it into PDDQ mode.
 */
static void __init cm_fx6_init_sata(void)
{
	void __iomem *addr = (void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K);
	u32 tmpdata;

	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI configured. Save power. PDDQ: %s \n",
		((readl(addr + PORT_PHY_CTL) >> 20) & 1) ? "enabled" : "disabled");
}
#endif

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits	= 4,
	.clk_map_ver	= 2,
};

static struct ipuv3_fb_platform_data cm_fx6_fb_data[] = {
	{ /* fb0 */
	/* Startek 800x480 LCD */
	.disp_dev		= "lcd",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB24,
	.mode_str		= "KD050C-WVGA",
	.default_bpp		= 24,
	.int_clk		= false,
	}, {
	.disp_dev		= "ldb",
	.interface_pix_fmt	= IPU_PIX_FMT_RGB666,
	.mode_str		= "LDB-XGA",
	.default_bpp		= 16,
	.int_clk		= false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;
	int max_ipu_id = cpu_is_mx6q() ? 1 : 0;

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

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
	.default_ifmt	= IPU_PIX_FMT_RGB24,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id		= 1,
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

static struct gpio mx6_flexcan_gpios[] = {
	{ CM_FX6_CAN1_EN, GPIOF_OUT_INIT_LOW, "flexcan1-en" },
	{ CM_FX6_CAN1_STBY, GPIOF_OUT_INIT_LOW, "flexcan1-stby" },
	{ CM_FX6_CAN2_EN, GPIOF_OUT_INIT_LOW, "flexcan2-en" },
};

static void mx6_flexcan0_switch(int enable)
{
	if (enable) {
		gpio_set_value(CM_FX6_CAN1_EN, 1);
		gpio_set_value(CM_FX6_CAN1_STBY, 1);
	} else {
		gpio_set_value(CM_FX6_CAN1_EN, 0);
		gpio_set_value(CM_FX6_CAN1_STBY, 0);
	}
}

static void mx6_flexcan1_switch(int enable)
{
	if (enable) {
		gpio_set_value(CM_FX6_CAN2_EN, 1);
		gpio_set_value_cansleep(CM_FX6_CAN2_STBY, 1);
	} else {
		gpio_set_value(CM_FX6_CAN2_EN, 0);
		gpio_set_value_cansleep(CM_FX6_CAN2_STBY, 0);
	}
}

static const struct flexcan_platform_data
		cm_fx6_flexcan_pdata[] __initconst = {
	{
		.transceiver_switch = mx6_flexcan0_switch,
	}, {
		.transceiver_switch = mx6_flexcan1_switch,
	}
};

static void cm_fx6_suspend_enter(void)
{
	/* suspend preparation */
}

static void cm_fx6_suspend_exit(void)
{
	/* resume restore */
}
static const struct pm_platform_data cm_fx6_pm_data __initconst = {
	.name		= "imx_pm",
	.suspend_enter	= cm_fx6_suspend_enter,
	.suspend_exit	= cm_fx6_suspend_exit,
};

static struct regulator_consumer_supply cm_fx6_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data cm_fx6_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(cm_fx6_vmmc_consumers),
	.consumer_supplies = cm_fx6_vmmc_consumers,
};

static struct fixed_voltage_config cm_fx6_vmmc_reg_config = {
	.supply_name	= "vmmc",
	.microvolts	= 3300000,
	.gpio		= -1,
	.init_data	= &cm_fx6_vmmc_init,
};

static struct platform_device cm_fx6_vmmc_reg_devices = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev		= {
		.platform_data = &cm_fx6_vmmc_reg_config,
	},
};

static struct imx_ssi_platform_data cm_fx6_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static int __init cm_fx6_init_audio(void)
{
	/* FIXME */
	return 0;
}

static struct mxc_mlb_platform_data cm_fx6_mlb150_data = {
	.reg_nvcc		= NULL,
	.mlb_clk		= "mlb150_clk",
	.mlb_pll_clk		= "pll6",
};

static struct mxc_dvfs_platform_data cm_fx6_dvfscore_data = {
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

static int __init early_enable_spdif(char *p)
{
	spdif_en = 1;
	return 0;
}

early_param("spdif", early_enable_spdif);

static int __init early_enable_can(char *p)
{
	flexcan_en = 1;
	return 0;
}

early_param("flexcan", early_enable_can);

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

static const struct imx_pcie_platform_data cm_fx6_pcie_data  __initconst = {
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


static void mx6_snvs_poweroff(void)
{
	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;

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
	int i;
	int ret;

	iomux_v3_cfg_t *common_pads = NULL;
	iomux_v3_cfg_t *spdif_pads = NULL;
	iomux_v3_cfg_t *flexcan_pads = NULL;

	int common_pads_cnt;
	int spdif_pads_cnt;
	int flexcan_pads_cnt;


	/*
	 * Override system revision number to meet conventions set by Freescale.
	 * According to Freescale, system revision should be set in this very
	 * way by U-Boot and be passed to the kernel via atags.
	 */
	fsl_system_rev();

	/* power off handler */
	pm_power_off = mx6_snvs_poweroff;


	/*
	 * common pads: pads are non-shared with others on this board
	 * feature_pds: pads are shared with others on this board
	 */

	if (cpu_is_mx6q()) {
		common_pads = cm_fx6_q_pads;
		spdif_pads = cm_fx6_q_spdif_pads;
		flexcan_pads = cm_fx6_q_can_pads;

		common_pads_cnt = ARRAY_SIZE(cm_fx6_q_pads);
		spdif_pads_cnt =  ARRAY_SIZE(cm_fx6_q_spdif_pads);
		flexcan_pads_cnt = ARRAY_SIZE(cm_fx6_q_can_pads);
	} else if (cpu_is_mx6dl()) {
		common_pads = cm_fx6_dl_pads;
		spdif_pads = cm_fx6_dl_spdif_pads;
		flexcan_pads = cm_fx6_dl_can_pads;

		common_pads_cnt = ARRAY_SIZE(cm_fx6_dl_pads);
		spdif_pads_cnt =  ARRAY_SIZE(cm_fx6_dl_spdif_pads);
		flexcan_pads_cnt = ARRAY_SIZE(cm_fx6_dl_can_pads);
	}

	BUG_ON(!common_pads);
	mxc_iomux_v3_setup_multiple_pads(common_pads, common_pads_cnt);

	/*
	 * S/PDIF out and can1 stby are mutually exclusive because both
	 * use GPIO_17.
	 */
	if (spdif_en) {
		BUG_ON(!spdif_pads);
		mxc_iomux_v3_setup_multiple_pads(spdif_pads, spdif_pads_cnt);
	}
	else if (flexcan_en) {
		BUG_ON(!flexcan_pads);
		mxc_iomux_v3_setup_multiple_pads(flexcan_pads,
						flexcan_pads_cnt);
	}

	/*
	 * the following is the common devices support on the shared ARM2 boards
	 * Since i.MX6DQ/DL share the same memory/Register layout, we don't
	 * need to diff the i.MX6DQ or i.MX6DL here. We can simply use the
	 * mx6q_add_features() for the shared devices. For which only exist
	 * on each indivual SOC, we can use cpu_is_mx6q/6dl() to diff it.
	 */

	gp_reg_id = cm_fx6_dvfscore_data.reg_id;
	soc_reg_id = cm_fx6_dvfscore_data.soc_id;
	pu_reg_id = cm_fx6_dvfscore_data.pu_id;
	cm_fx6_init_uart();

	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q())
		imx6q_add_ipuv3(1, &ipu_data[1]);

	if (cpu_is_mx6dl()) {
		ldb_data.ipu_id = 0;
		ldb_data.disp_id = 0;
		for (i = 0; i < ARRAY_SIZE(cm_fx6_fb_data) / 2; i++)
			imx6q_add_ipuv3fb(i, &cm_fx6_fb_data[i]);
	} else {
		for (i = 0; i < ARRAY_SIZE(cm_fx6_fb_data); i++)
			imx6q_add_ipuv3fb(i, &cm_fx6_fb_data[i]);
	}

	imx6q_add_vdoa();
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);

	imx6q_add_imx_snvs_rtc();

	imx6q_add_imx_caam();

	cm_fx6_init_i2c();
	cm_fx6_init_led();
	cm_fx6_init_spi();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &cm_fx6_anatop_thermal_data);

	// imx6_init_fec(fec_data); -- called asynchronously by cm_fx6_id_eeprom_setup()

	imx6q_add_pm_imx(0, &cm_fx6_pm_data);
	// sb_fx6_sd_init(); -- called asynchronously by sb_fx6_id_eeprom_setup()
	imx_add_viv_gpu(&imx6_gpu_data, &imx6_gpu_pdata);
	if (cpu_is_mx6q()) {
		cm_fx6_init_sata();
	}
	imx6q_add_vpu();
	cm_fx6_init_usb();
	cm_fx6_init_audio();
	platform_device_register(&cm_fx6_vmmc_reg_devices);

	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();
	imx6q_add_gpmi(&cm_fx6_gpmi_nand_pdata);

	imx6q_add_dvfs_core(&cm_fx6_dvfscore_data);

	imx6q_add_ion(0, &imx_ion_data,
		sizeof(imx_ion_data) + sizeof(struct ion_platform_heap));

	cm_fx6_pwm_init();

	if (spdif_en) {
		mxc_spdif_data.spdif_core_clk = clk_get_sys("mxc_spdif.0", NULL);
		clk_put(mxc_spdif_data.spdif_core_clk);
		imx6q_add_spdif(&mxc_spdif_data);
		imx6q_add_spdif_dai();
		imx6q_add_spdif_audio_device();
	} else if (flexcan_en) {
		ret = gpio_request_array(mx6_flexcan_gpios,
				ARRAY_SIZE(mx6_flexcan_gpios));
		if (ret) {
			pr_err("failed to request flexcan-gpios: %d\n", ret);
		} else {
			imx6q_add_flexcan0(&cm_fx6_flexcan_pdata[0]);
			imx6q_add_flexcan1(&cm_fx6_flexcan_pdata[1]);
		}
	}

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
	imx6q_add_mlb150(&cm_fx6_mlb150_data);

	/* Add PCIe RC interface support */
	imx6q_add_pcie(&cm_fx6_pcie_data);
	imx6q_add_busfreq();
}

extern void __iomem *twd_base;
static void __init mx6_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
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
static void fsl_system_rev(void)
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
	/* 'Revision' in /proc/cpuinfo */
	extern unsigned int system_rev;

	u32 fsl_system_rev = 0;

	u32 cpu_type = readl(IO_ADDRESS(ANATOP_BASE_ADDR + 0x260));

	/* Chip Silicon ID */
	fsl_system_rev = ((cpu_type >> 16) & 0xFF) << 12;
	/* Chip silicon major revision */
	fsl_system_rev |= ((cpu_type >> 8) & 0xFF) << 4;
	fsl_system_rev += 0x10;
	/* Chip silicon minor revision */
	fsl_system_rev |= cpu_type & 0xFF;

	/* store aside CompuLab board revision */
	board_rev = system_rev;
	system_rev = fsl_system_rev;
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

