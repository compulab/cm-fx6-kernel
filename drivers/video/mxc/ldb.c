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

/*!
 * @file mxc_ldb.c
 *
 * @brief This file contains the LDB driver device interface and fops
 * functions.
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/fsl_devices.h>
#include <mach/hardware.h>
#include <mach/clock.h>
#include "mxc_dispdrv.h"

#define DISPDRV_LDB	"ldb"

#define LDB_BGREF_RMODE_MASK		0x00008000
#define LDB_BGREF_RMODE_INT		0x00008000
#define LDB_BGREF_RMODE_EXT		0x0

#define LDB_DI1_VS_POL_MASK		0x00000400
#define LDB_DI1_VS_POL_ACT_LOW		0x00000400
#define LDB_DI1_VS_POL_ACT_HIGH		0x0
#define LDB_DI0_VS_POL_MASK		0x00000200
#define LDB_DI0_VS_POL_ACT_LOW		0x00000200
#define LDB_DI0_VS_POL_ACT_HIGH		0x0

#define LDB_BIT_MAP_CH1_MASK		0x00000100
#define LDB_BIT_MAP_CH1_JEIDA		0x00000100
#define LDB_BIT_MAP_CH1_SPWG		0x0
#define LDB_BIT_MAP_CH0_MASK		0x00000040
#define LDB_BIT_MAP_CH0_JEIDA		0x00000040
#define LDB_BIT_MAP_CH0_SPWG		0x0

#define LDB_DATA_WIDTH_CH1_MASK		0x00000080
#define LDB_DATA_WIDTH_CH1_24		0x00000080
#define LDB_DATA_WIDTH_CH1_18		0x0
#define LDB_DATA_WIDTH_CH0_MASK		0x00000020
#define LDB_DATA_WIDTH_CH0_24		0x00000020
#define LDB_DATA_WIDTH_CH0_18		0x0

#define LDB_CH1_MODE_MASK		0x0000000C
#define LDB_CH1_MODE_EN_TO_DI1		0x0000000C
#define LDB_CH1_MODE_EN_TO_DI0		0x00000004
#define LDB_CH1_MODE_DISABLE		0x0
#define LDB_CH0_MODE_MASK		0x00000003
#define LDB_CH0_MODE_EN_TO_DI1		0x00000003
#define LDB_CH0_MODE_EN_TO_DI0		0x00000001
#define LDB_CH0_MODE_DISABLE		0x0

#define LDB_SPLIT_MODE_EN		0x00000010

struct ldb_data {
	struct platform_device *pdev;
	struct mxc_dispdrv_entry *disp_ldb;
	uint32_t *reg;
	uint32_t *control_reg;
	uint32_t *gpr3_reg;
	struct regulator *lvds_bg_reg;
	int mode;
	bool inited;
	struct clk *di_clk[2];
	struct clk *ldb_di_clk[2];
	struct ldb_setting {
		bool active;
		bool clk_en;
		int ipu;
		int di;
	} setting[2];
	struct notifier_block nb;
};

static int g_ldb_mode;

static struct fb_videomode ldb_modedb[] = {
	{
	 "LDB-XGA", 60, 1024, 768, 15385,
	 220, 40,
	 21, 7,
	 60, 10,
	 0,
	 FB_VMODE_NONINTERLACED,
	 FB_MODE_IS_DETAILED,},
	{
	 "LDB-1080P60", 60, 1920, 1080, 7692,
	 100, 40,
	 30, 3,
	 10, 2,
	 0,
	 FB_VMODE_NONINTERLACED,
	 FB_MODE_IS_DETAILED,},
};
static int ldb_modedb_sz = ARRAY_SIZE(ldb_modedb);

static int bits_per_pixel(int pixel_fmt)
{
	switch (pixel_fmt) {
	case IPU_PIX_FMT_BGR24:
	case IPU_PIX_FMT_RGB24:
		return 24;
		break;
	case IPU_PIX_FMT_BGR666:
	case IPU_PIX_FMT_RGB666:
	case IPU_PIX_FMT_LVDS666:
		return 18;
		break;
	default:
		break;
	}
	return 0;
}

static int valid_mode(int pixel_fmt)
{
	return ((pixel_fmt == IPU_PIX_FMT_RGB24) ||
		(pixel_fmt == IPU_PIX_FMT_BGR24) ||
		(pixel_fmt == IPU_PIX_FMT_LVDS666) ||
		(pixel_fmt == IPU_PIX_FMT_RGB666) ||
		(pixel_fmt == IPU_PIX_FMT_BGR666));
}

static int __init ldb_setup(char *options)
{
	if (!strcmp(options, "spl0"))
		g_ldb_mode = LDB_SPL_DI0;
	else if (!strcmp(options, "spl1"))
		g_ldb_mode = LDB_SPL_DI1;
	else if (!strcmp(options, "dul0"))
		g_ldb_mode = LDB_DUL_DI0;
	else if (!strcmp(options, "dul1"))
		g_ldb_mode = LDB_DUL_DI1;
	else if (!strcmp(options, "sin0"))
		g_ldb_mode = LDB_SIN_DI0;
	else if (!strcmp(options, "sin1"))
		g_ldb_mode = LDB_SIN_DI1;
	else if (!strcmp(options, "sep"))
		g_ldb_mode = LDB_SEP;

	return 1;
}
__setup("ldb=", ldb_setup);

static int find_ldb_setting(struct ldb_data *ldb, struct fb_info *fbi)
{
	char *id_di[] = {
		 "DISP3 BG",
		 "DISP3 BG - DI1",
		};
	char id[16];
	int i;

	for (i = 0; i < 2; i++) {
		if (ldb->setting[i].active) {
			memset(id, 0, 16);
			memcpy(id, id_di[ldb->setting[i].di],
				strlen(id_di[ldb->setting[i].di]));
			id[4] += ldb->setting[i].ipu;
			if (!strcmp(id, fbi->fix.id))
				return i;
		}
	}
	return -EINVAL;
}

int ldb_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct ldb_data *ldb = container_of(nb, struct ldb_data, nb);
	struct fb_event *event = v;
	struct fb_info *fbi = event->info;
	int setting_idx, di;

	setting_idx = find_ldb_setting(ldb, fbi);
	if (setting_idx < 0)
		return 0;

	fbi->mode = (struct fb_videomode *)fb_match_mode(&fbi->var,
			&fbi->modelist);

	if (!fbi->mode) {
		dev_warn(&ldb->pdev->dev,
				"LDB: can not find mode for xres=%d, yres=%d\n",
				fbi->var.xres, fbi->var.yres);
		if (ldb->setting[setting_idx].clk_en) {
			clk_disable(ldb->ldb_di_clk[di]);
			ldb->setting[setting_idx].clk_en = false;
		}
		return 0;
	}

	di = ldb->setting[setting_idx].di;

	switch (val) {
	case FB_EVENT_PREMODE_CHANGE:
	{
		uint32_t reg;
		uint32_t pixel_clk, rounded_pixel_clk;

		/* vsync setup */
		reg = readl(ldb->control_reg);
		if (fbi->var.sync & FB_SYNC_VERT_HIGH_ACT) {
			if (di == 0)
				reg = (reg & ~LDB_DI0_VS_POL_MASK)
					| LDB_DI0_VS_POL_ACT_HIGH;
			else
				reg = (reg & ~LDB_DI1_VS_POL_MASK)
					| LDB_DI1_VS_POL_ACT_HIGH;
		} else {
			if (di == 0)
				reg = (reg & ~LDB_DI0_VS_POL_MASK)
					| LDB_DI0_VS_POL_ACT_LOW;
			else
				reg = (reg & ~LDB_DI1_VS_POL_MASK)
					| LDB_DI1_VS_POL_ACT_LOW;
		}
		writel(reg, ldb->control_reg);

		/* clk setup */
		pixel_clk = (PICOS2KHZ(fbi->var.pixclock)) * 1000UL;
		rounded_pixel_clk = clk_round_rate(ldb->ldb_di_clk[di],
				pixel_clk);
		clk_set_rate(ldb->ldb_di_clk[di], rounded_pixel_clk);
		clk_enable(ldb->ldb_di_clk[di]);
		ldb->setting[setting_idx].clk_en = true;
		break;
	}
	case FB_EVENT_BLANK:
	{
		if (*((int *)event->data) == FB_BLANK_UNBLANK) {
			if (!ldb->setting[setting_idx].clk_en) {
				clk_enable(ldb->ldb_di_clk[di]);
				ldb->setting[setting_idx].clk_en = true;
			}
		} else {
			if (ldb->setting[setting_idx].clk_en) {
				clk_disable(ldb->ldb_di_clk[di]);
				ldb->setting[setting_idx].clk_en = false;
			}
		}
	}
	default:
		break;
	}
	return 0;
}

#define LVDS0_MUX_CTL_MASK	(3 << 6)
#define LVDS1_MUX_CTL_MASK	(3 << 8)
#define LVDS0_MUX_CTL_OFFS	6
#define LVDS1_MUX_CTL_OFFS	8
#define ROUTE_IPU0_DI0		0
#define ROUTE_IPU0_DI1		1
#define ROUTE_IPU1_DI0		2
#define ROUTE_IPU1_DI1		3
static int ldb_ipu_ldb_route(int ipu, int di, struct ldb_data *ldb)
{
	uint32_t reg;
	int mode = ldb->mode;

	reg = readl(ldb->gpr3_reg);
	if ((mode == LDB_SPL_DI0) || (mode == LDB_DUL_DI0)) {
		reg &= ~(LVDS0_MUX_CTL_MASK | LVDS1_MUX_CTL_MASK);
		if (ipu == 0)
			reg |= (ROUTE_IPU0_DI0 << LVDS0_MUX_CTL_OFFS) |
				(ROUTE_IPU0_DI0 << LVDS1_MUX_CTL_OFFS);
		else
			reg |= (ROUTE_IPU1_DI0 << LVDS0_MUX_CTL_OFFS) |
				(ROUTE_IPU1_DI0 << LVDS1_MUX_CTL_OFFS);
		dev_dbg(&ldb->pdev->dev,
			"Dual/Split mode both channels route to IPU%d-DI0\n", ipu);
	} else if ((mode == LDB_SPL_DI1) || (mode == LDB_DUL_DI1)) {
		reg &= ~(LVDS0_MUX_CTL_MASK | LVDS1_MUX_CTL_MASK);
		if (ipu == 0)
			reg |= (ROUTE_IPU0_DI1 << LVDS0_MUX_CTL_OFFS) |
				(ROUTE_IPU0_DI1 << LVDS1_MUX_CTL_OFFS);
		else
			reg |= (ROUTE_IPU1_DI1 << LVDS0_MUX_CTL_OFFS) |
				(ROUTE_IPU1_DI1 << LVDS1_MUX_CTL_OFFS);
		dev_dbg(&ldb->pdev->dev,
			"Dual/Split mode both channels route to IPU%d-DI1\n", ipu);
	} else if (mode == LDB_SIN_DI0) {
		reg &= ~LVDS0_MUX_CTL_MASK;
		if (ipu == 0)
			reg |= ROUTE_IPU0_DI0 << LVDS0_MUX_CTL_OFFS;
		else
			reg |= ROUTE_IPU1_DI0 << LVDS0_MUX_CTL_OFFS;
		dev_dbg(&ldb->pdev->dev,
			"Single mode channel 0 route to IPU%d-DI0\n", ipu);
	} else if (mode == LDB_SIN_DI1) {
		reg &= ~LVDS1_MUX_CTL_MASK;
		if (ipu == 0)
			reg |= ROUTE_IPU0_DI1 << LVDS1_MUX_CTL_OFFS;
		else
			reg |= ROUTE_IPU1_DI1 << LVDS1_MUX_CTL_OFFS;
		dev_dbg(&ldb->pdev->dev,
			"Single mode channel 1 route to IPU%d-DI1\n", ipu);
	} else if (mode == LDB_SEP) {
		if (di == 0)
			reg &= ~LVDS0_MUX_CTL_MASK;
		else
			reg &= ~LVDS1_MUX_CTL_MASK;
		if ((ipu == 0) && (di == 0))
			reg |= ROUTE_IPU0_DI0 << LVDS0_MUX_CTL_OFFS;
		else if ((ipu == 0) && (di == 1))
			reg |= ROUTE_IPU0_DI1 << LVDS1_MUX_CTL_OFFS;
		else if ((ipu == 1) && (di == 0))
			reg |= ROUTE_IPU1_DI0 << LVDS0_MUX_CTL_OFFS;
		else
			reg |= ROUTE_IPU1_DI1 << LVDS1_MUX_CTL_OFFS;

		dev_dbg(&ldb->pdev->dev,
			"Separate mode channel %d route to IPU%d-DI%d\n", di, ipu, di);
	}
	writel(reg, ldb->gpr3_reg);

	return 0;
}

static int ldb_disp_init(struct mxc_dispdrv_entry *disp)
{
	int ret = 0, i;
	struct ldb_data *ldb = mxc_dispdrv_getdata(disp);
	struct mxc_dispdrv_setting *setting = mxc_dispdrv_getsetting(disp);
	struct fsl_mxc_ldb_platform_data *plat_data = ldb->pdev->dev.platform_data;
	struct resource *res;
	uint32_t base_addr;
	uint32_t reg, setting_idx;

	/* ipu selected by platform data setting */
	setting->dev_id = plat_data->ipu_id;

	/* if input format not valid, make RGB666 as default*/
	if (!valid_mode(setting->if_fmt)) {
		dev_warn(&ldb->pdev->dev, "Input pixel format not valid"
					"use default RGB666\n");
		setting->if_fmt = IPU_PIX_FMT_RGB666;
	}

	if (!ldb->inited) {
		struct clk *ldb_clk_parent;
		char di0_clk[] = "ipu1_di0_clk";
		char di1_clk[] = "ipu1_di1_clk";
		unsigned long ldb_clk_prate = 455000000;

		ldb->ldb_di_clk[0] = clk_get(&ldb->pdev->dev, "ldb_di0_clk");
		if (IS_ERR(ldb->ldb_di_clk[0])) {
			dev_err(&ldb->pdev->dev, "get ldb clk0 failed\n");
			return PTR_ERR(ldb->ldb_di_clk[0]);
		}
		ldb->ldb_di_clk[1] = clk_get(&ldb->pdev->dev, "ldb_di1_clk");
		if (IS_ERR(ldb->ldb_di_clk[1])) {
			dev_err(&ldb->pdev->dev, "get ldb clk1 failed\n");
			return PTR_ERR(ldb->ldb_di_clk[1]);
		}
		di0_clk[3] += plat_data->ipu_id;
		di1_clk[3] += plat_data->ipu_id;

		ldb->di_clk[0] = clk_get(&ldb->pdev->dev, di0_clk);
		if (IS_ERR(ldb->di_clk[0])) {
			dev_err(&ldb->pdev->dev, "get di clk0 failed\n");
			return PTR_ERR(ldb->di_clk[0]);
		}
		ldb->di_clk[1] = clk_get(&ldb->pdev->dev, di1_clk);
		if (IS_ERR(ldb->di_clk[1])) {
			dev_err(&ldb->pdev->dev, "get di clk1 failed\n");
			return PTR_ERR(ldb->di_clk[1]);
		}

		/* FIXME: set ldb_di_clk parent to 455M to fit both XGA/1080P mode*/
		ldb_clk_parent =
			clk_get_parent(ldb->ldb_di_clk[0]);
		clk_set_rate(ldb_clk_parent, ldb_clk_prate);
		ldb_clk_parent =
			clk_get_parent(ldb->ldb_di_clk[1]);
		clk_set_rate(ldb_clk_parent, ldb_clk_prate);

		res = platform_get_resource(ldb->pdev, IORESOURCE_MEM, 0);
		if (IS_ERR(res)) {
			ret = -ENOMEM;
			goto get_res_failed;
		}

		base_addr = res->start;
		ldb->reg = ioremap(base_addr, res->end - res->start + 1);
		ldb->control_reg = ldb->reg + 2;
		ldb->gpr3_reg = ldb->reg + 3;

		ldb->lvds_bg_reg = regulator_get(&ldb->pdev->dev, plat_data->lvds_bg_reg);
		if (!IS_ERR(ldb->lvds_bg_reg)) {
			regulator_set_voltage(ldb->lvds_bg_reg, 2500000, 2500000);
			regulator_enable(ldb->lvds_bg_reg);
		}

		reg = readl(ldb->control_reg);

		/* refrence resistor select */
		reg &= ~LDB_BGREF_RMODE_MASK;
		if (plat_data->ext_ref)
			reg |= LDB_BGREF_RMODE_EXT;
		else
			reg |= LDB_BGREF_RMODE_INT;

		/* TODO: now only use SPWG data mapping for both channel */
		reg &= ~(LDB_BIT_MAP_CH0_MASK | LDB_BIT_MAP_CH1_MASK);
		reg |= LDB_BIT_MAP_CH0_SPWG | LDB_BIT_MAP_CH1_SPWG;

		/* channel mode setting */
		reg &= ~(LDB_CH0_MODE_MASK | LDB_CH1_MODE_MASK);
		reg &= ~(LDB_DATA_WIDTH_CH0_MASK | LDB_DATA_WIDTH_CH1_MASK);

		if (bits_per_pixel(setting->if_fmt) == 24)
			reg |= LDB_DATA_WIDTH_CH0_24 | LDB_DATA_WIDTH_CH1_24;
		else
			reg |= LDB_DATA_WIDTH_CH0_18 | LDB_DATA_WIDTH_CH1_18;

		if (g_ldb_mode)
			ldb->mode = g_ldb_mode;
		else
			ldb->mode = plat_data->mode;

		if (ldb->mode == LDB_SPL_DI0) {
			reg |= LDB_SPLIT_MODE_EN | LDB_CH0_MODE_EN_TO_DI0
				| LDB_CH1_MODE_EN_TO_DI0;
			setting->disp_id = 0;
		} else if (ldb->mode == LDB_SPL_DI1) {
			reg |= LDB_SPLIT_MODE_EN | LDB_CH0_MODE_EN_TO_DI1
				| LDB_CH1_MODE_EN_TO_DI1;
			setting->disp_id = 1;
		} else if (ldb->mode == LDB_DUL_DI0) {
			reg &= ~LDB_SPLIT_MODE_EN;
			reg |= LDB_CH0_MODE_EN_TO_DI0 | LDB_CH1_MODE_EN_TO_DI0;
			setting->disp_id = 0;
		} else if (ldb->mode == LDB_DUL_DI1) {
			reg &= ~LDB_SPLIT_MODE_EN;
			reg |= LDB_CH0_MODE_EN_TO_DI1 | LDB_CH1_MODE_EN_TO_DI1;
			setting->disp_id = 1;
		} else if (ldb->mode == LDB_SIN_DI0) {
			reg &= ~LDB_SPLIT_MODE_EN;
			reg |= LDB_CH0_MODE_EN_TO_DI0;
			setting->disp_id = 0;
		} else if (ldb->mode == LDB_SIN_DI1) {
			reg &= ~LDB_SPLIT_MODE_EN;
			reg |= LDB_CH1_MODE_EN_TO_DI1;
			setting->disp_id = 1;
		} else { /* separate mode*/
			reg &= ~LDB_SPLIT_MODE_EN;
			reg |= LDB_CH0_MODE_EN_TO_DI0 | LDB_CH1_MODE_EN_TO_DI1;
			setting->disp_id = plat_data->disp_id;
			if (bits_per_pixel(setting->if_fmt) == 24) {
				if (setting->disp_id == 0)
					reg &= ~LDB_DATA_WIDTH_CH1_24;
				else
					reg &= ~LDB_DATA_WIDTH_CH0_24;
			} else {
				if (setting->disp_id == 0)
					reg &= ~LDB_DATA_WIDTH_CH1_18;
				else
					reg &= ~LDB_DATA_WIDTH_CH0_18;
			}
		}

		writel(reg, ldb->control_reg);

		/* fb notifier for clk setting */
		ldb->nb.notifier_call = ldb_fb_event,
		ret = fb_register_client(&ldb->nb);
		if (ret < 0)
			goto fb_register_nb_failed;

		setting_idx = 0;
		ldb->inited = true;
	} else { /* second time for separate mode */
		int disp_id;

		if ((ldb->mode == LDB_SPL_DI0) ||
			(ldb->mode == LDB_SPL_DI1) ||
			(ldb->mode == LDB_DUL_DI0) ||
			(ldb->mode == LDB_DUL_DI1) ||
			(ldb->mode == LDB_SIN_DI0) ||
			(ldb->mode == LDB_SIN_DI1)) {
			dev_err(&ldb->pdev->dev, "for second ldb disp"
					"ldb mode should in separate mode\n");
			return -EINVAL;
		}

		disp_id = setting->disp_id = !plat_data->disp_id;

		reg = readl(ldb->control_reg);
		if (bits_per_pixel(setting->if_fmt) == 24) {
			if (disp_id == 0)
				reg |= LDB_DATA_WIDTH_CH0_24;
			else
				reg |= LDB_DATA_WIDTH_CH1_24;
		} else {
			if (disp_id == 0)
				reg |= LDB_DATA_WIDTH_CH0_18;
			else
				reg |= LDB_DATA_WIDTH_CH1_18;
		}
		writel(reg, ldb->control_reg);
		setting_idx = 1;
	}

	if (cpu_is_mx6q())
		ldb_ipu_ldb_route(setting->dev_id, setting->disp_id, ldb);

	/*
	 * ldb_di0_clk -> ipux_di0_clk
	 * ldb_di1_clk -> ipux_di1_clk
	 */
	clk_set_parent(ldb->di_clk[setting->disp_id],
			ldb->ldb_di_clk[setting->disp_id]);

	/* must use spec video mode defined by driver */
	ret = fb_find_mode(&setting->fbi->var, setting->fbi, setting->dft_mode_str,
				ldb_modedb, ldb_modedb_sz, NULL, setting->default_bpp);
	if (ret != 1)
		fb_videomode_to_var(&setting->fbi->var, &ldb_modedb[0]);

	INIT_LIST_HEAD(&setting->fbi->modelist);
	for (i = 0; i < ldb_modedb_sz; i++) {
		struct fb_videomode m;
		fb_var_to_videomode(&m, &setting->fbi->var);
		if (fb_mode_is_equal(&m, &ldb_modedb[i])) {
			fb_add_videomode(&ldb_modedb[i],
					&setting->fbi->modelist);
			break;
		}
	}

	/* save current ldb setting for fb notifier */
	ldb->setting[setting_idx].active = true;
	ldb->setting[setting_idx].ipu = setting->dev_id;
	ldb->setting[setting_idx].di = setting->disp_id;

	return ret;

fb_register_nb_failed:
	iounmap(ldb->reg);
get_res_failed:
	return ret;
}

static void ldb_disp_deinit(struct mxc_dispdrv_entry *disp)
{
	struct ldb_data *ldb = mxc_dispdrv_getdata(disp);
	int i;

	writel(0, ldb->control_reg);

	for (i = 0; i < 2; i++) {
		clk_disable(ldb->ldb_di_clk[i]);
		clk_put(ldb->ldb_di_clk[i]);
	}

	fb_unregister_client(&ldb->nb);

	iounmap(ldb->reg);
}

static struct mxc_dispdrv_driver ldb_drv = {
	.name 	= DISPDRV_LDB,
	.init 	= ldb_disp_init,
	.deinit	= ldb_disp_deinit,
};

/*!
 * This function is called by the driver framework to initialize the LDB
 * device.
 *
 * @param	dev	The device structure for the LDB passed in by the
 *			driver framework.
 *
 * @return      Returns 0 on success or negative error code on error
 */
static int ldb_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct ldb_data *ldb;

	ldb = kzalloc(sizeof(struct ldb_data), GFP_KERNEL);
	if (!ldb) {
		ret = -ENOMEM;
		goto alloc_failed;
	}

	ldb->pdev = pdev;
	ldb->disp_ldb = mxc_dispdrv_register(&ldb_drv);
	mxc_dispdrv_setdata(ldb->disp_ldb, ldb);

	dev_set_drvdata(&pdev->dev, ldb);

alloc_failed:
	return ret;
}

static int ldb_remove(struct platform_device *pdev)
{
	struct ldb_data *ldb = dev_get_drvdata(&pdev->dev);

	mxc_dispdrv_unregister(ldb->disp_ldb);
	kfree(ldb);
	return 0;
}

static struct platform_driver mxcldb_driver = {
	.driver = {
		   .name = "mxc_ldb",
		   },
	.probe = ldb_probe,
	.remove = ldb_remove,
};

static int __init ldb_init(void)
{
	return platform_driver_register(&mxcldb_driver);
}

static void __exit ldb_uninit(void)
{
	platform_driver_unregister(&mxcldb_driver);
}

module_init(ldb_init);
module_exit(ldb_uninit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC LDB driver");
MODULE_LICENSE("GPL");
