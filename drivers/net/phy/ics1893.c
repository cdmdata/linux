/*
 * drivers/net/phy/ics1893.c
 *
 * Driver for ICS1893 PHY
 *
 * Author: Herbert Valerio Riedel
 *
 * Copyright (c) 2006 Herbert Valerio Riedel <hvr@gnu.org>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Support added for SMSC LAN8187 and LAN8700 by steve.glendinning@smsc.com
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/netdevice.h>

static struct phy_driver ics1893_driver = {
	.phy_id 	= 0x00015f45,
	.phy_id_mask	= 0xfffffff0,
	.name 		= "ICS1893",
	.features	= (PHY_BASIC_FEATURES),
	.flags		= 0,

	/* basic functions */
	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.config_init	= genphy_config_init,

	.suspend	= genphy_suspend,
	.resume		= genphy_resume,

	.driver		= { .owner = THIS_MODULE, }
};


static int __init ics_init(void)
{
	int ret;

	ret = phy_driver_register (&ics1893_driver);
	return ret;
}

static void __exit smsc_exit(void)
{
	phy_driver_unregister (&ics1893_driver);
}

MODULE_DESCRIPTION("ICS1893 PHY driver");
MODULE_AUTHOR("Herbert Valerio Riedel");
MODULE_LICENSE("GPL");

module_init(ics_init);
module_exit(ics_exit);
