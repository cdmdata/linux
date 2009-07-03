/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2005 David Brownell
 * (C) Copyright 2002 Hewlett-Packard Company
 * (C) Copyright 2008 Magnus Damm
 *
 * SM501 Bus Glue - based on ohci-omap.c
 *
 * This file is licenced under the GPL.
 */
#include <linux/platform_device.h>
#include <linux/sm501-int.h>

/////////////////////////////////////////////////////////////
struct SM501_usb_data
{
	port_t cfgBase;
	port_t bufBase;
};
#define hcd_to_sm501(hcd)	((struct SM501_usb_data *)(hcd_to_ohci(hcd) + 1))


void sm501_usb_int(int slotnum, void * hdata)
{
	struct usb_hcd *hcd = (struct usb_hcd *)hdata;
	struct SM501_usb_data *sm = hcd_to_sm501(hcd);
	unsigned long usbsts;
	port_t usbBase = hcd->regs;
	port_t bufBase = sm->bufBase;
#if 1
	//Dummy read, flush the SM501 cache
//	SMI_DUMMY(bufBase,0x000);
	SMI_DUMMY(bufBase,0x800);
#endif
	//save USB host interrupt status
	usbsts = SMI_USBREG(usbBase,SMIR_USB_INT_STATUS_REG);
	if (usbsts & SMI_USBREG(usbBase,SMIR_USB_INT_MASK_REG)) {
#if 1
		//Dummy read, flush the SM501 cache
		SMI_DUMMY(bufBase,0x000);
//		SMI_DUMMY(bufBase,0x800);
#endif
//		pr_info("%s\n", __func__);
		usb_hcd_irq(-1,hdata);
	}
}

extern int usb_disabled(void);

/*-------------------------------------------------------------------------*/

static void sm501_start_hc(port_t cfgBase,port_t usbioport)
{
	// Patch for the PW mode 0
	SMI_REG(cfgBase,SMIR_MISCELLANEOUS_TIMING) = 0x00080800;

	SMI_REG(cfgBase, SMIR_INT_MASK_REG) |= 0x00000040;	// turn on USB Host Interrupt
	if ((SMI_REG(cfgBase, SMIR_INT_MASK_REG) & 0x00000040) == 0)
	{
		printk("VGX_USB::Initialize USB Host Interrupt Failed\r\n");
	}

	// Power Mode 0 Gate
	SMI_REG(cfgBase, SMIR_PWRM0_GATE) |= 0x800;		// turn on USB Host Clock
	// Power Mode 1 Gate
	SMI_REG(cfgBase, SMIR_PWRM1_GATE) |= 0x800;		// turn on USB Host Clock

	mdelay(50);
}

static void sm501_stop_hc(struct platform_device *dev)
{
	struct usb_hcd *hcd = platform_get_drvdata(dev);
	struct SM501_usb_data *sm = hcd_to_sm501(hcd);
	port_t cfgBase = sm->cfgBase;
	port_t usbBase = hcd->regs;
	if (usbBase) {
		SMI_USBREG(usbBase, HcCommandStatus) |= MHcCommandStatusHCRmask;	// reset host controller
	}
	mdelay(1);
	if (cfgBase) {
		// Power Mode 0 Gate
		SMI_REG(cfgBase, SMIR_PWRM0_GATE) &= ~0x800;		// turn off USB Host Clock
		// Power Mode 1 Gate
		SMI_REG(cfgBase, SMIR_PWRM1_GATE) &= ~0x800;		// turn off USB Host Clock
	}
}


/*-------------------------------------------------------------------------*/

static int __devinit
ohci_sm501_start (struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci(hcd);
	int ret;

	ohci_dbg(ohci, "ohci_sm501_start, ohci:%p", ohci);

	if ((ret = ohci_init(ohci)) < 0)
		return ret;

	if ((ret = ohci_run (ohci)) < 0) {
		err ("can't start %s", hcd->self.bus_name);
		ohci_stop (hcd);
		return ret;
	}

	return 0;
}

/*-------------------------------------------------------------------------*/

static const struct hc_driver ohci_sm501_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"sm501 OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd) + sizeof (struct SM501_usb_data),

	/* generic hardware linkage */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY | HCD_LOCAL_MEM,

	/* basic lifecycle operations */
	.start =		ohci_sm501_start,
	.stop =			ohci_stop,
	.shutdown =		ohci_shutdown,

	/* managing i/o requests and associated device resources */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/* scheduling support */
	.get_frame_number =	ohci_get_frame,

	/* root hub support */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
#ifdef	CONFIG_PM
	.hub_suspend =		ohci_hub_suspend,
	.hub_resume =		ohci_hub_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

/*-------------------------------------------------------------------------*/

static int ohci_hcd_sm501_drv_probe(struct platform_device *dev)
{
	int retval;
	struct SM501_usb_data *sm = NULL;
	struct usb_hcd *hcd = NULL;
//	struct resource *cfg = platform_get_resource(dev, IORESOURCE_MEM, 0);
	struct resource *usbregs = platform_get_resource(dev, IORESOURCE_MEM, 1);
	struct resource *usb_dma = platform_get_resource(dev, IORESOURCE_MEM, 2);
//	int irq = platform_get_irq(dev, 0);

	port_t cfgBase = NULL;
	port_t usbBase = NULL;
	port_t bufBase = NULL;

	if (usb_disabled())
		return -ENODEV;

	cfgBase = get_mmioVirtual();
	usbBase = ioremap(usbregs->start, usbregs->end - usbregs->start + 1);
	bufBase = ioremap(usb_dma->start, 0x1000);	//1 page needed for errata workaround

	if ( (!cfgBase)||(!usbBase)) {
		pr_debug("ioremap failed");
		retval = -ENOMEM;
		goto err1;
	}

	hcd = usb_create_hcd(&ohci_sm501_hc_driver, &dev->dev, dev_name(&dev->dev));
	if (!hcd) {
		pr_debug ("hcd_alloc failed");
		retval = -ENOMEM;
		goto err1;
	}
	hcd->regs=usbBase;
	hcd->rsrc_start = usbregs->start;
	hcd->rsrc_len = usbregs->end - usbregs->start + 1;

	sm = hcd_to_sm501(hcd);
	sm->cfgBase=cfgBase;
	sm->bufBase=bufBase;
//	hcd->irq = ~0; // we'll deliver irq
//	hcd->self.controller = &dev->dev;

	sm501_start_hc(cfgBase,usbBase);
	pr_info("%s dma start:0x%x, chip start %x\n", __func__, usb_dma->start, ((unsigned int)(usb_dma->start)) & 0x03ffffff);
	if (!dma_declare_coherent_memory(&dev->dev, usb_dma->start,
			/* grab offset from chip select */
			((unsigned int)(usb_dma->start)) & 0x03ffffff,
			usb_dma->end - usb_dma->start + 1,
			DMA_MEMORY_MAP | DMA_MEMORY_EXCLUSIVE)) {
		retval = -EBUSY;
		goto err2;
	}
	ohci_hcd_init(hcd_to_ohci(hcd));

	retval = SM501_grab_int_slot(cfgBase, SMI_INTERRUPT_USB, sm501_usb_int, hcd);
        if (retval) {
		pr_debug("request_irq(%d) failed with retval %d\n", SMI_INTERRUPT_USB, retval);
		retval = -EBUSY;
		goto err3;
	}
	pr_info("%s (sm501) at 0x%p, cfgBase = %p\n",
		hcd->driver->description, hcd->regs, cfgBase);

	retval = usb_add_hcd(hcd, ~0, IRQF_DISABLED);
	if (retval){
		printk( KERN_ERR "%s: usb_add_hcd error \n", __FUNCTION__ );
		goto err3;
	}
	return 0;
//	usb_remove_hcd(hcd);
err3:
	dma_release_declared_memory(&dev->dev);
err2:
	sm501_stop_hc(dev);
err1:
	if (hcd) {
		usb_put_hcd(hcd);
		hcd->regs = NULL;
	}
	if (cfgBase)
		iounmap(cfgBase);
	if (usbBase)
		iounmap(usbBase);
	if (bufBase)
		iounmap(bufBase);
	if (sm) {
		sm->cfgBase = NULL;
		sm->bufBase = NULL;
	}
	return retval;
}

/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * ohci_hcd_sm501_drv_remove - shutdown processing for sm501-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of ohci_hcd_sm501_drv_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
static int ohci_hcd_sm501_drv_remove(struct platform_device *dev)
{
	struct usb_hcd *hcd = platform_get_drvdata(dev);
	struct SM501_usb_data *sm = hcd_to_sm501(hcd);
	port_t usbBase = hcd->regs;
	port_t cfgBase = sm->cfgBase;
	port_t bufBase = sm->bufBase;
	pr_debug ("remove: %s, state %x", hcd->self.bus_name, hcd->state);

	if (in_interrupt ())
		BUG ();
//	hcd->state = USB_STATE_QUIESCING;
	usb_remove_hcd(hcd);
	sm501_stop_hc(dev);
	
	dma_release_declared_memory(&dev->dev);
	sm->cfgBase = NULL;
	hcd->regs = NULL;
	sm->bufBase = NULL;
	if (usbBase)
		iounmap(usbBase);
	// Put the USB host controller into reset.
	if (cfgBase)
		iounmap(cfgBase);	// Release memory resources.
	if (bufBase)
		iounmap(bufBase);	// Release memory resources.
	usb_put_hcd(hcd);
	platform_set_drvdata(dev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int ohci_hcd_sm501_drv_suspend(struct device *dev, u32 state)
{
//	struct platform_device *pdev = to_platform_device(dev);
//	struct usb_hcd *hcd = dev_get_drvdata(dev);
	printk("%s: not implemented yet\n", __FUNCTION__);
	return 0;
}

static int ohci_hcd_sm501_drv_resume(struct device *dev)
{
//	struct platform_device *pdev = to_platform_device(dev);
//	struct usb_hcd *hcd = dev_get_drvdata(dev);
	printk("%s: not implemented yet\n", __FUNCTION__);
	return 0;
}
#else
#define ohci_hcd_sm501_drv_suspend NULL
#define ohci_hcd_sm501_drv_resume NULL
#endif

static struct platform_driver ohci_hcd_sm501_driver = {
	.probe		= ohci_hcd_sm501_drv_probe,
	.remove		= __devexit_p(ohci_hcd_sm501_drv_remove),
	.shutdown	= usb_hcd_platform_shutdown,
	.suspend	= ohci_hcd_sm501_drv_suspend, 
	.resume		= ohci_hcd_sm501_drv_resume, 
	.driver		= {
		.name	= "sm501-ohci",
		.owner	= THIS_MODULE,
	},
};
