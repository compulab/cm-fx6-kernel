/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @defgroup Framebuffer Framebuffer Driver for SDC and ADC.
 */

/*!
 * @file mxc_edid.c
 *
 * @brief MXC EDID driver
 *
 * @ingroup Framebuffer
 */

/*!
 * Include files
 */
#include <linux/i2c.h>
#include <linux/fb.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mxcfb.h>
#include <linux/fsl_devices.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regulator/consumer.h>
#include <mach/mxc_edid.h>
#include "../edid.h"

#define MXC_EDID_LENGTH (EDID_LENGTH*4)

struct mxc_ddc_data {
	struct platform_device *pdev;
	struct i2c_client *client;
	struct delayed_work det_work;
	struct fb_info *fbi;
	struct mxc_edid_cfg edid_cfg;
	u8 cable_plugin;
	u8 edid[MXC_EDID_LENGTH];

	u32 di;
	void (*init)(void);
	int (*update)(void);
	struct regulator *analog_reg;
} mxc_ddc;

#define MXC_ENABLE	1
#define MXC_DISABLE	2
static int g_enable_ddc;

static ssize_t mxc_ddc_show_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (mxc_ddc.cable_plugin == 0)
		strcpy(buf, "plugout\n");
	else
		strcpy(buf, "plugin\n");

	return strlen(buf);
}

static DEVICE_ATTR(cable_state, S_IRUGO, mxc_ddc_show_state, NULL);

static ssize_t mxc_ddc_show_name(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	strcpy(buf, mxc_ddc.fbi->fix.id);
	sprintf(buf+strlen(buf), "\n");

	return strlen(buf);
}

static DEVICE_ATTR(fb_name, S_IRUGO, mxc_ddc_show_name, NULL);

static ssize_t mxc_ddc_show_edid(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i, j, len = 0;

	for (j = 0; j < MXC_EDID_LENGTH/16; j++) {
		for (i = 0; i < 16; i++)
			len += sprintf(buf+len, "0x%02X ",
					mxc_ddc.edid[j*16 + i]);
		len += sprintf(buf+len, "\n");
	}

	return len;
}

static DEVICE_ATTR(edid, S_IRUGO, mxc_ddc_show_edid, NULL);

static void det_worker(struct work_struct *work)
{
	char event_string[16];
	char *envp[] = { event_string, NULL };

	/* cable connection changes */
	if (mxc_ddc.update()) {
		u8 edid_old[MXC_EDID_LENGTH];
		mxc_ddc.cable_plugin = 1;
		sprintf(event_string, "EVENT=plugin");

		memcpy(edid_old, mxc_ddc.edid, MXC_EDID_LENGTH);

		if (mxc_edid_read(mxc_ddc.client->adapter, mxc_ddc.client->addr,
				mxc_ddc.edid, &mxc_ddc.edid_cfg, mxc_ddc.fbi) < 0)
			dev_err(&mxc_ddc.client->dev,
					"MXC ddc: read edid fail\n");
		else {
			if (!memcmp(edid_old, mxc_ddc.edid, MXC_EDID_LENGTH))
				dev_info(&mxc_ddc.client->dev,
					"Sii902x: same edid\n");
			else if (mxc_ddc.fbi->monspecs.modedb_len > 0) {
				int i;
				const struct fb_videomode *mode;
				struct fb_videomode m;

				/* make sure fb is powerdown */
				acquire_console_sem();
				fb_blank(mxc_ddc.fbi, FB_BLANK_POWERDOWN);
				release_console_sem();

				fb_destroy_modelist(&mxc_ddc.fbi->modelist);

				for (i = 0; i < mxc_ddc.fbi->monspecs.modedb_len; i++)
					/*FIXME now we do not support interlaced mode */
					if (!(mxc_ddc.fbi->monspecs.modedb[i].vmode & FB_VMODE_INTERLACED))
						fb_add_videomode(&mxc_ddc.fbi->monspecs.modedb[i],
								&mxc_ddc.fbi->modelist);

				fb_var_to_videomode(&m, &mxc_ddc.fbi->var);
				mode = fb_find_nearest_mode(&m,
						&mxc_ddc.fbi->modelist);

				fb_videomode_to_var(&mxc_ddc.fbi->var, mode);

				mxc_ddc.fbi->var.activate |= FB_ACTIVATE_FORCE;
				acquire_console_sem();
				mxc_ddc.fbi->flags |= FBINFO_MISC_USEREVENT;
				fb_set_var(mxc_ddc.fbi, &mxc_ddc.fbi->var);
				mxc_ddc.fbi->flags &= ~FBINFO_MISC_USEREVENT;
				release_console_sem();

				acquire_console_sem();
				fb_blank(mxc_ddc.fbi, FB_BLANK_UNBLANK);
				release_console_sem();
			}
		}
	} else {
		mxc_ddc.cable_plugin = 0;
		sprintf(event_string, "EVENT=plugout");
	}

	kobject_uevent_env(&mxc_ddc.pdev->dev.kobj, KOBJ_CHANGE, envp);
}

static irqreturn_t mxc_ddc_detect_handler(int irq, void *data)
{
	if (mxc_ddc.fbi)
		schedule_delayed_work(&(mxc_ddc.det_work), msecs_to_jiffies(300));
	return IRQ_HANDLED;
}

static int mxc_ddc_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct fb_event *event = v;
	struct fb_info *fbi = event->info;

	if ((mxc_ddc.di)) {
		if (strcmp(event->info->fix.id, "DISP3 BG - DI1"))
			return 0;
	} else {
		if (strcmp(event->info->fix.id, "DISP3 BG"))
			return 0;
	}

	switch (val) {
	case FB_EVENT_FB_REGISTERED:
		if (mxc_ddc.fbi != NULL)
			break;
		mxc_ddc.fbi = fbi;
		break;
	}
	return 0;
}

static struct notifier_block nb = {
	.notifier_call = mxc_ddc_fb_event,
};

static int __devinit mxc_ddc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct fb_info edid_fbi;
	struct mxc_ddc_platform_data *plat = client->dev.platform_data;

	if (plat->boot_enable && !g_enable_ddc)
		g_enable_ddc = MXC_ENABLE;
	if (!g_enable_ddc)
		g_enable_ddc = MXC_DISABLE;

	if (g_enable_ddc == MXC_DISABLE) {
		printk(KERN_WARNING "By setting, DDC driver will not be enabled\n");
		return -ENODEV;
	}

	mxc_ddc.client = client;
	mxc_ddc.di = plat->di;
	mxc_ddc.init = plat->init;
	mxc_ddc.update = plat->update;

	if (!mxc_ddc.update)
		return -EINVAL;

	mxc_ddc.analog_reg = regulator_get(&mxc_ddc.pdev->dev, plat->analog_regulator);
	if (!IS_ERR(mxc_ddc.analog_reg)) {
		regulator_set_voltage(mxc_ddc.analog_reg, 2775000, 2775000);
		regulator_enable(mxc_ddc.analog_reg);
	}

	if (mxc_ddc.init)
		mxc_ddc.init();

	if (mxc_ddc.update()) {
		mxc_ddc.cable_plugin = 1;
		/* try to read edid */
		if (mxc_edid_read(client->adapter, client->addr,
					mxc_ddc.edid, &mxc_ddc.edid_cfg, &edid_fbi) < 0)
			dev_warn(&client->dev, "Can not read edid\n");
#if defined(CONFIG_MXC_IPU_V3) && defined(CONFIG_FB_MXC_SYNC_PANEL)
		else
			mxcfb_register_mode(mxc_ddc.di, edid_fbi.monspecs.modedb,
					edid_fbi.monspecs.modedb_len, MXC_DISP_DDC_DEV);
#endif
	} else
		mxc_ddc.cable_plugin = 0;

	if (client->irq) {
		ret = request_irq(client->irq, mxc_ddc_detect_handler,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"ddc_det", &mxc_ddc);
		if (ret < 0) {
			dev_warn(&client->dev,
				"MXC ddc: cound not request det irq %d\n",
				client->irq);
			goto err;
		} else {
			INIT_DELAYED_WORK(&(mxc_ddc.det_work), det_worker);
			ret = device_create_file(&mxc_ddc.pdev->dev, &dev_attr_fb_name);
			if (ret < 0)
				dev_warn(&client->dev,
					"MXC ddc: cound not create sys node for fb name\n");
			ret = device_create_file(&mxc_ddc.pdev->dev, &dev_attr_cable_state);
			if (ret < 0)
				dev_warn(&client->dev,
					"MXC ddc: cound not create sys node for cable state\n");
			ret = device_create_file(&mxc_ddc.pdev->dev, &dev_attr_edid);
			if (ret < 0)
				dev_warn(&client->dev,
					"MXC ddc: cound not create sys node for edid\n");
		}
	}

	fb_register_client(&nb);

err:
	return ret;
}

static int __devexit mxc_ddc_remove(struct i2c_client *client)
{
	fb_unregister_client(&nb);
	if (!IS_ERR(mxc_ddc.analog_reg))
		regulator_disable(mxc_ddc.analog_reg);
	return 0;
}

static int __init enable_ddc_setup(char *options)
{
	if (!strcmp(options, "=off"))
		g_enable_ddc = MXC_DISABLE;
	else
		g_enable_ddc = MXC_ENABLE;

	return 1;
}
__setup("ddc", enable_ddc_setup);

static const struct i2c_device_id mxc_ddc_id[] = {
	{ "mxc_ddc", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, mxc_ddc_id);

static struct i2c_driver mxc_ddc_i2c_driver = {
	.driver = {
		   .name = "mxc_ddc",
		   },
	.probe = mxc_ddc_probe,
	.remove = mxc_ddc_remove,
	.id_table = mxc_ddc_id,
};

static int __init mxc_ddc_init(void)
{
	int ret;

	memset(&mxc_ddc, 0, sizeof(mxc_ddc));

	mxc_ddc.pdev = platform_device_register_simple("mxc_ddc", 0, NULL, 0);
	if (IS_ERR(mxc_ddc.pdev)) {
		printk(KERN_ERR
				"Unable to register MXC DDC as a platform device\n");
		ret = PTR_ERR(mxc_ddc.pdev);
		goto err;
	}

	return i2c_add_driver(&mxc_ddc_i2c_driver);
err:
	return ret;
}

static void __exit mxc_ddc_exit(void)
{
	i2c_del_driver(&mxc_ddc_i2c_driver);
	platform_device_unregister(mxc_ddc.pdev);
}

module_init(mxc_ddc_init);
module_exit(mxc_ddc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC DDC driver");
MODULE_LICENSE("GPL");
