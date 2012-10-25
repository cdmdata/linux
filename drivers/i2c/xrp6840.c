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
#include<linux/delay.h>
#include<linux/err.h>
#include<linux/i2c.h>
#include<linux/interrupt.h>
#include<linux/kthread.h>
#include<linux/platform_device.h>


static int xrp6840_probe(struct i2c_client *adapter,
		const struct i2c_device_id *device_id);
static int xrp6840_remove(struct i2c_client *client);

/*!
 * xrp6840 I2C initialize XRP6840 function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int xrp6840_probe(struct i2c_client *client,
		const struct i2c_device_id *id) 
{
	char buf[10] = {0};
	int res;

	printk(KERN_INFO "In xrp6840 probe");


	//Configure device
    printk(KERN_INFO "Sending init to xrp6840\n");
    buf[0]=0xE2;
    buf[1]=0x60;
    buf[2]=0xFC;
    buf[3]=0x0C;
    res=i2c_master_send(client, buf, 4);
 	if (res!= 4) {
        /* ERROR HANDLING: i2c transaction failed */
        printk(KERN_INFO "Failed to write to the i2c bus.\n");
        return(res);
    }

    printk(KERN_INFO "Leaving xrp6840_probe");
    return(0);
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


/*!
 * xrp6840 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int xrp6840_remove(struct i2c_client *client) {
	char buf[10] = {0};
	int res;

	printk(KERN_INFO "In xrp6840 remove");
    buf[0]=0x00; // Shutdown 

    res=i2c_master_send(client, buf, 1);
 	if (res != 1) {
        /* ERROR HANDLING: i2c transaction failed */
        printk(KERN_INFO "Failed to write to the i2c bus.\n %x",res);
    }

    return(res);
}

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
