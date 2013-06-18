/*
 * Touch Screen driver for DataImage's I2C connected touch screen panels
 *   Copyright (c) 2012 Anders Electronics
 *   Copyright 2012 CompuLab Ltd, Dmitry Lifshitz <lifshitz@compulab.co.il>
 *
 * Based on migor_ts.c
 *   Copyright (c) 2008 Magnus Damm
 *   Copyright (c) 2007 Ujjwal Pande <ujjwal@kenati.com>
 *
 * This file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU  General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/timer.h>

#define MAX_X 480
#define MAX_Y 800

#define FINGERS 2
#define FINGER_SIZE 4

struct di_ts_priv {
	struct i2c_client *client;
	struct input_dev *input[FINGERS];
	char phys[FINGERS][32];
	char name[FINGERS][32];
	int irq;
	int opened;
};

static irqreturn_t di_ts_isr(int irq, void *dev_id)
{
	struct di_ts_priv *priv = dev_id;
	const u8 NO_EVENT = 0xFF;
	unsigned short i = 0, xpos, ypos;
	u8 getEvent = 0x85;
	u8 buf[16];

	memset(buf, 0, sizeof(buf));

	if (i2c_master_send(priv->client, &getEvent, 1) != 1) {
		dev_err(&priv->client->dev, "Unable to write i2c\n");
		return IRQ_HANDLED;
	}

	/* Now do Page Read */
	if (i2c_master_recv(priv->client, buf, sizeof(buf)) != sizeof(buf)) {
		dev_err(&priv->client->dev, "Unable to read data from i2c\n");
		return IRQ_HANDLED;
	}

	for (i = 0; i < FINGERS; i++) {
		dev_dbg(&priv->client->dev,
			"Buffer [%d]: %02X %02X %02X %02X\n",
			i, buf[i * FINGER_SIZE], buf[i * FINGER_SIZE + 1],
			buf[i * FINGER_SIZE + 2], buf[i * FINGER_SIZE + 3]);

		if (buf[i * FINGER_SIZE] == NO_EVENT) {
			dev_dbg(&priv->client->dev, "Untouched");

			input_report_key(priv->input[i], BTN_TOUCH, 0);
			input_report_abs(priv->input[i], ABS_PRESSURE, 0);
			input_sync(priv->input[i]);
		} else {
			ypos = MAX_Y - (buf[i * FINGER_SIZE + 1] |
					(buf[i * FINGER_SIZE + 0] << 8));

			xpos = buf[i * FINGER_SIZE + 3] |
				(buf[i * FINGER_SIZE + 2] << 8);

			dev_dbg(&priv->client->dev, "Touched: x = %d, y = %d\n",
				xpos, ypos);

			input_report_key(priv->input[i], BTN_TOUCH, 1);
			input_report_abs(priv->input[i], ABS_PRESSURE, 0xFF);
			input_report_abs(priv->input[i], ABS_X, xpos);
			input_report_abs(priv->input[i], ABS_Y, ypos);
			input_sync(priv->input[i]);
		}
	}

	return IRQ_HANDLED;
}

static int di_ts_open(struct input_dev *dev)
{
	struct di_ts_priv *priv = input_get_drvdata(dev);
	struct i2c_client *client = priv->client;
	char sleepOut = 0x81;
	char senseOn = 0x83;
	char clearStack = 0x88;
	char speedMode[] = { 0x9D, 0x80 };
	char mcuTurnOn[] = { 0x35, 0x02 };
	char flashTurnOn[] = { 0x36, 0x01 };

	if (priv->opened)
		return 0;

	priv->opened = 1;

	if (i2c_master_send(client, &sleepOut, 1) != 1)
		dev_err(&client->dev, "Unable to sleep out\n");

	msleep(120);

	if (i2c_master_send(client, speedMode, 2) != 2)
		dev_err(&client->dev, "Unable to set speed mode\n");

	if (i2c_master_send(client, mcuTurnOn, 2) != 2)
		dev_err(&client->dev, "Unable to turn on the MCU\n");

	if (i2c_master_send(client, flashTurnOn, 2) != 2)
		dev_err(&client->dev, "Unable to turn on the flash\n");

	if (i2c_master_send(client, &senseOn, 1) != 1)
		dev_err(&client->dev, "Unable to set sense on\n");

	msleep(100);

	if (i2c_master_send(client, &clearStack, 1) != 1)
		dev_err(&client->dev, "Unable to clear stack\n");

	return 0;
}

static int di_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *idp)
{
	struct di_ts_priv *priv;
	struct input_dev *input[FINGERS];
	int error, i = 0;
	char buf[3];
	char getDevId = 0x31;

	if (i2c_master_send(client, &getDevId, 1) == 1 &&
		i2c_master_recv(client, buf, sizeof(buf)) == sizeof(buf)) {
		dev_info(&client->dev, "Device ID: 0x%02X%02X%02X\n",
			 buf[0], buf[1], buf[2]);
	} else {
		dev_err(&client->dev, "Unable to get DevId\n");
		return -ENODEV;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	priv->client = client;
	priv->irq = client->irq;

	for (i = 0; i < FINGERS; i++) {
		input[i] = input_allocate_device();
		if (!input[i]) {
			dev_err(&client->dev,
				"Failed to allocate input device.\n");
			error = -ENOMEM;
			goto err_free_mem;
		}

		priv->input[i] = input[i];

		__set_bit(EV_KEY, input[i]->evbit);
		__set_bit(EV_ABS, input[i]->evbit);
		__set_bit(EV_SYN, input[i]->evbit);
		__set_bit(BTN_TOUCH, input[i]->keybit);

		input_set_abs_params(input[i], ABS_X, 0, MAX_X, 0, 0);
		input_set_abs_params(input[i], ABS_Y, 0, MAX_Y, 0, 0);
		input_set_abs_params(input[i], ABS_PRESSURE, 0, 0xff, 0, 0);

		snprintf(priv->phys[i], sizeof(priv->phys[i]),
				"%s/input%d", client->name, i);
		snprintf(priv->name[i], sizeof(priv->name[i]),
				"%s%d", client->name, i);

		input[i]->phys = priv->phys[i];
		input[i]->name = priv->name[i];
		input[i]->id.bustype = BUS_I2C;
		input[i]->dev.parent = &client->dev;

		input[i]->open = di_ts_open;

		input_set_drvdata(input[i], priv);
	}

	error = request_threaded_irq(priv->irq, NULL, di_ts_isr,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					client->name, priv);
	if (error) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		goto err_free_mem;
	}

	for (i = 0; i < FINGERS; i++) {
		error = input_register_device(input[i]);
		if (error)
			goto err_free_irq;
	}

	i2c_set_clientdata(client, priv);
	device_init_wakeup(&client->dev, 1);

	return 0;

err_free_irq:
	free_irq(priv->irq, priv);
err_free_mem:
	for ( ; i >= 0; i--)
		input_free_device(input[i]);

	i2c_set_clientdata(client, NULL);
	kfree(priv);

	return error;
}

static int di_ts_remove(struct i2c_client *client)
{
	struct di_ts_priv *priv = i2c_get_clientdata(client);
	int i;

	free_irq(priv->irq, priv);
	for (i = FINGERS; i >= 0; i--)
		input_unregister_device(priv->input[i]);

	i2c_set_clientdata(client, NULL);
	kfree(priv);

	return 0;
}

static int di_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct di_ts_priv *priv = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(priv->irq);

	return 0;
}

static int di_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct di_ts_priv *priv = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(priv->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(di_ts_pm, di_ts_suspend, di_ts_resume);

static const struct i2c_device_id di_ts_id[] = {
	{ "hx8520-c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, di_ts_id);

static struct i2c_driver di_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "hx8520-c",
		.pm = &di_ts_pm,
	},
	.probe = di_ts_probe,
	.remove = di_ts_remove,
	.id_table = di_ts_id,
};

static int __init di_ts_init(void)
{
	return i2c_add_driver(&di_ts_driver);
}

static void __exit di_ts_exit(void)
{
	i2c_del_driver(&di_ts_driver);
}

module_init(di_ts_init);
module_exit(di_ts_exit);

MODULE_DESCRIPTION("HX8520-C Touchscreen driver");
MODULE_LICENSE("GPL v2");
