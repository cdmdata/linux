/*
 * LED Flash driver for XRP6840
 *
 * Copyright 2012 Steve Jardine CDM Data.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include<linux/module.h>
#include <linux/slab.h>
#include<linux/delay.h>
#include<linux/err.h>
#include<linux/i2c.h>
#include<linux/interrupt.h>
#include<linux/kthread.h>
#include<linux/platform_device.h>

struct xrp6840_priv
{
	struct i2c_client	*client;
	int			enabled;
};

static char on_str[] = {0xe1, 0x60, 0xec, 0x0c};	/* Configure device */
static char off_str[] = {0x00};

static void xrp6840_notify(void *pdata, int on)
{
	int ret;
	struct i2c_client *client = (struct i2c_client *)pdata;
	struct xrp6840_priv *xrp = i2c_get_clientdata(client);
	char *p = on ? on_str : off_str;
	char cnt = on ? sizeof(on_str) : sizeof(off_str);

	xrp->enabled = on;
	ret = i2c_master_send(client, p, cnt);
	if (ret != cnt) {
		/* ERROR HANDLING: i2c transaction failed */
		pr_info("%s: Failed to write to the i2c bus.\n", __func__);
		return;
	}
	pr_info("%s: on=%d cnt=%d\n", __func__, on, cnt);
}


static ssize_t xrp6840_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 tmp[8];
	int result;
	struct xrp6840_priv *xrp = dev_get_drvdata(dev);

	tmp[0] = 0;
	tmp[1] = 0;
	tmp[2] = 0;
	tmp[3] = 0;
	result = i2c_master_recv(xrp->client, tmp, 4);
	if (result != 4) {
		dev_err(dev, "i2c_master_recv failed(%i)\n", result);
	}
	return sprintf(buf, "%d, %02x %02x %02x %02x\n", xrp->enabled, tmp[0], tmp[1], tmp[2], tmp[3]);
}

static ssize_t xrp6840_enable_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int val;
	struct xrp6840_priv *xrp = dev_get_drvdata(dev);
	val = simple_strtol(buf, NULL, 10);
	if (val < 0)
		return count;

	xrp6840_notify(xrp->client, val);
	return count;
}

static DEVICE_ATTR(xrp6840_enable, 0644, xrp6840_enable_show, xrp6840_enable_store);


void set_camera_active_notify(void *pdata, void (*notify_callback)(void * pdata, int on));

/*!
 * xrp6840 I2C initialize XRP6840 function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int xrp6840_probe(struct i2c_client *client,
		const struct i2c_device_id *id) 
{
	struct xrp6840_priv *xrp = kzalloc(sizeof(struct xrp6840_priv),GFP_KERNEL);
	int ret;

	if (!xrp) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}
	xrp->client = client;
	i2c_set_clientdata(client, xrp);

	set_camera_active_notify(client, xrp6840_notify);
	ret = device_create_file(&client->dev, &dev_attr_xrp6840_enable);
	if (ret < 0)
		pr_warn("failed to add xrp6840 sysfs enable file\n");
	pr_info("Leaving %s", __func__);
	return 0;
}

/*!
 * xrp6840 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int xrp6840_remove(struct i2c_client *client)
{
	struct xrp6840_priv *xrp = i2c_get_clientdata(client);

	printk(KERN_INFO "In xrp6840 remove");
	set_camera_active_notify(NULL, NULL);
	if (xrp && xrp->enabled)
		xrp6840_notify(client, 0);
	device_remove_file(&client->dev, &dev_attr_xrp6840_enable);
	if (xrp)
		kfree(xrp);
	return 0;
}

static const struct i2c_device_id xrp6840_id[] = { { "xrp6840", 0 }, { } };

static struct i2c_driver xrp6840_driver = {
          .driver = {
                     .name = "xrp6840",
                     .owner = THIS_MODULE,
                     },
          .probe = xrp6840_probe,
          .remove = __devexit_p(xrp6840_remove),
          .id_table = xrp6840_id,
 };

static int __init xrp6840_init(void)
 {
          /* register driver */
          int res = i2c_add_driver(&xrp6840_driver);
          if (res<0 ) {
                  printk(KERN_INFO "add xrp6840 i2c driver failed\n");
                  return -ENODEV;
          }
          printk(KERN_INFO "added xrp6840 i2c driver\n");
          return (res);
 }

 static void __exit xrp6840_exit(void)
 {
          printk(KERN_INFO "remove xrp6840 i2c driver.\n");
          /* reset the xrp6840 */
          i2c_del_driver(&xrp6840_driver);
 }

 module_init(xrp6840_init);
 module_exit(xrp6840_exit);


 MODULE_AUTHOR("Steve Jardine CDMData, Inc.");
 MODULE_LICENSE("GPL");
 MODULE_DESCRIPTION("LED Flash Driver for Exar XRP6840A");
