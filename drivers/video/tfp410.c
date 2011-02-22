/*
 *  tfp410.c - DVI output chip
 *
 *  Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/input-polldev.h>
#include <linux/gpio.h>

/* register definitions according to the TFP410 data sheet */
#define TFP410_VID		0x014C
#define TFP410_DID		0x0410
#define TFP410_VID_DID		(TFP410_VID | (TFP410_DID << 16))

#define TFP410_VID_LO		0x00
#define TFP410_VID_HI		0x01
#define TFP410_DID_LO		0x02
#define TFP410_DID_HI		0x03
#define TFP410_REV		0x04

#define TFP410_CTL_1		0x08
#define TFP410_CTL_1_RSVD	(1<<7)
#define TFP410_CTL_1_TDIS	(1<<6)
#define TFP410_CTL_1_VEN	(1<<5)
#define TFP410_CTL_1_HEN	(1<<4)
#define TFP410_CTL_1_DSEL	(1<<3)
#define TFP410_CTL_1_BSEL	(1<<2)
#define TFP410_CTL_1_EDGE	(1<<1)
#define TFP410_CTL_1_PD		(1<<0)

#define DRV_NAME	"tfp410"
//static const char *client_name = DRV_NAME;


struct tfp410_priv
{
	struct i2c_client	*client;
	int gp;
};



static unsigned char cmd_off[] = {
	TFP410_CTL_1, TFP410_CTL_1_RSVD | TFP410_CTL_1_VEN | TFP410_CTL_1_HEN |
		TFP410_CTL_1_DSEL | TFP410_CTL_1_BSEL,
		0
};
static unsigned char cmd_on[] = {
	TFP410_CTL_1, TFP410_CTL_1_RSVD | TFP410_CTL_1_VEN | TFP410_CTL_1_HEN |
		TFP410_CTL_1_DSEL | TFP410_CTL_1_BSEL | TFP410_CTL_1_PD,
		0
};
/*
 * Initialization function
 */
static int tfp410_send_cmds(struct i2c_client *client, unsigned char *p)
{
	int result;
	while (*p) {
		result = i2c_smbus_write_byte_data(client, p[0], p[1]);
		if (result) {
			pr_err("%s: failed(%i) %x=%x\n", __func__, result, p[0], p[1]);
			return result;
		}
		p += 2;
	}
	return result;
}


static int __devinit tfp_init(struct tfp410_priv *tfp, struct i2c_client *client)
{
	/* Initialize the tfp410 chip */
	int result = tfp410_send_cmds(client, cmd_on);
	if (result)
		dev_err(&client->dev, "init failed\n");
	return result;
}

static void tfp_deinit(struct tfp410_priv *tfp)
{
}

static ssize_t tfp410_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 tmp[16];
	struct tfp410_priv *tfp = dev_get_drvdata(dev);

	if (i2c_smbus_read_i2c_block_data(tfp->client, 0, 11, tmp) < 11) {
			dev_err(dev, "i2c block read failed\n");
			return -EIO;
	}
	return sprintf(buf, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", tmp[0], tmp[1], tmp[2], tmp[3], tmp[4],
			tmp[5], tmp[6], tmp[7], tmp[8], tmp[9], tmp[10]);
}

static DEVICE_ATTR(tfp410_reg, 0444, tfp410_reg_show, NULL);

struct plat_i2c_tfp410_data {
	unsigned gp;
};

/*
 * I2C init/probing/exit functions
 */
static int __devinit tfp410_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	unsigned vid_did;
	int result;
	int gp;
	int ret;
	struct tfp410_priv *tfp;
	struct i2c_adapter *adapter;
	struct plat_i2c_tfp410_data *plat = client->dev.platform_data;

	adapter = to_i2c_adapter(client->dev.parent);

	result = i2c_check_functionality(adapter,
					 I2C_FUNC_SMBUS_BYTE |
					 I2C_FUNC_SMBUS_BYTE_DATA);
	if (!result) {
		dev_err(&client->dev, "i2c_check_functionality failed\n");
		return -ENODEV;
	}

	gp = (plat) ? plat->gp : -1;
	printk(KERN_INFO "%s: tfp410 gp=%i\n", __func__, gp);
	if (gp >= 0) {
		if (gpio_request(gp, "tfp410 i2c mode")) {
			dev_err(&client->dev, "gp for i2c mode unavailable\n");
			gp = -1;
		} else {
			gpio_set_value(gp, 1);	/* enable i2c mode */
		}
	}
	if (i2c_smbus_read_i2c_block_data(client, TFP410_VID_LO, 4, (unsigned char *)&vid_did) < 4) {
		dev_err(&client->dev, "i2c block read failed\n");
		result = -EIO;
		goto release_gpio;
	}
	if (vid_did != TFP410_VID_DID) {
		dev_err(&client->dev, "id match failed %x != %x\n", vid_did, TFP410_VID_DID);
		result = -EIO;
		goto release_gpio;
	}

	tfp = kzalloc(sizeof(struct tfp410_priv),GFP_KERNEL);
	if (!tfp) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}
	tfp->client = client;
	tfp->gp = gp;

	i2c_set_clientdata(client, tfp);
	result = tfp_init(tfp, client);
	if (result)
		goto free_tfp;
	ret = device_create_file(&client->dev, &dev_attr_tfp410_reg);
	if (ret < 0)
		printk(KERN_WARNING "failed to add tfp410 sysfs files\n");
	return result;
free_tfp:
	tfp_deinit(tfp);
	kfree(tfp);
release_gpio:
	if (gp >= 0)
		gpio_free(gp);
	return result;
}

static int __devexit tfp410_remove(struct i2c_client *client)
{
	struct tfp410_priv *tfp = i2c_get_clientdata(client);
	int result = tfp410_send_cmds(client, cmd_off);
	device_remove_file(&client->dev, &dev_attr_tfp410_reg);
	if (tfp) {
		if (tfp->gp >=0)
			gpio_free(tfp->gp);
		tfp_deinit(tfp);
		kfree(tfp);
	}
	return result;
}

static int tfp410_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int result = tfp410_send_cmds(client, cmd_off);
	return result;
}

static int tfp410_resume(struct i2c_client *client)
{
	int result = tfp410_send_cmds(client, cmd_on);
	return result;
}

static const struct i2c_device_id tfp410_id[] = {
	{DRV_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, tfp410_id);

static struct i2c_driver tfp410_driver = {
	.driver = {
		   .name = DRV_NAME,
		   .owner = THIS_MODULE,
		   },
	.suspend = tfp410_suspend,
	.resume = tfp410_resume,
	.probe = tfp410_probe,
	.remove = __devexit_p(tfp410_remove),
	.id_table = tfp410_id,
};

static int __init tfp410_init(void)
{
	/* register driver */
	int res = i2c_add_driver(&tfp410_driver);
	if (res < 0) {
		printk(KERN_INFO "add tfp410 i2c driver failed\n");
		return -ENODEV;
	}
	printk(KERN_INFO "add tfp410 i2c driver\n");
	return res;
}

static void __exit tfp410_exit(void)
{
	printk(KERN_INFO "remove tfp410 i2c driver.\n");
	i2c_del_driver(&tfp410_driver);
}

MODULE_AUTHOR("Boundary Devices, Inc.");
MODULE_DESCRIPTION("tfp410 DVI output driver");
MODULE_LICENSE("GPL");

module_init(tfp410_init);
module_exit(tfp410_exit);
