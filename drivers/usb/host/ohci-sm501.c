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

#include <linux/device.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <linux/sm501-int.h>
#include <linux/platform_device.h>

static void ohci_hcd_init (struct ohci_hcd *ohci);
static int ohci_init (struct ohci_hcd *ohci);
static int ohci_run (struct ohci_hcd *ohci);
static void ohci_stop (struct usb_hcd *hcd);

/////////////////////////////////////////////////////////////
struct SM501_usb_data
{
	port_t cfgBase;
	port_t usbBase;
	port_t bufBase;
};

struct SM501_usb_data sm501_usb_Data;

void sm501_usb_int(int slotnum, void * hdata)
{
	unsigned long usbsts;
	port_t usbBase = sm501_usb_Data.usbBase;
	port_t bufBase = sm501_usb_Data.bufBase;
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
	port_t cfgBase = sm501_usb_Data.cfgBase;
	port_t usbBase = sm501_usb_Data.usbBase;
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
void init_sm501_pool_mapping(port_t bufBase,unsigned int size,unsigned int base_mapping);
void release_sm501_pool_mapping(void);

void usb_hcd_sm501_remove (struct usb_hcd *, struct platform_device *);

/**
 * usb_hcd_sm501_probe - initialize sm501-based HCDs
 * Context: !in_interrupt()
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 */
int usb_hcd_sm501_probe( const struct hc_driver *driver, struct platform_device *dev)
{
	int retval;
	struct usb_hcd *hcd = 0;
	struct resource *r = dev->resource ;

	port_t cfgBase = NULL;
	port_t usbBase = NULL;
	port_t bufBase = NULL;
	unsigned int bufbase_mapping;
	unsigned int bufSize;
//	if (!request_mem_region(r-start,r->.end - r->start + 1, hcd_name)) {
//		pr_debug("request_mem_region failed");
//		return -EBUSY;
//	}

	cfgBase = get_mmioVirtual();
	r++;
	usbBase = ioremap(r->start,r->end - r->start + 1);
	r++;
	bufbase_mapping = ((unsigned int)(r->start)) & 0x03ffffff; //grab offset from chip select
	bufSize = r->end - r->start + 1;
	bufBase = ioremap(r->start,bufSize);
	r++;

	if ( (!cfgBase)||(!usbBase)||(!bufBase)) {
		pr_debug("ioremap failed");
		retval = -ENOMEM;
		goto err1;
	}

	if(r->flags != IORESOURCE_IRQ){
		pr_debug ("resource is not IORESOURCE_IRQ");
		retval = -ENOMEM;
		goto err1;
	}

	sm501_usb_Data.cfgBase=cfgBase;
	sm501_usb_Data.bufBase=bufBase;	//needed for allocation routines
	sm501_usb_Data.usbBase=usbBase;

	sm501_start_hc(cfgBase,usbBase);

	hcd = usb_create_hcd( driver, &dev->dev, "sm501" );
	if (hcd == NULL){
		pr_debug ("hcd_alloc failed");
		retval = -ENOMEM;
		goto err1;
	}
	init_sm501_pool_mapping(bufBase,bufSize,bufbase_mapping);
	ohci_hcd_init(hcd_to_ohci(hcd));

	hcd->irq = 0 ; // we'll deliver irq
	hcd->regs = usbBase;
	hcd->self.controller = &dev->dev;

	retval = hcd_buffer_create (hcd);
	if (retval != 0) {
		pr_debug ("pool alloc fail");
		goto err2;
	}

	retval = SM501_grab_int_slot(cfgBase,SMI_INTERRUPT_USB,sm501_usb_int,hcd);
	
        if (retval != 0) {
		pr_debug("request_irq(%d/%d) failed with retval %d\n",hcd->irq,SMI_INTERRUPT_USB, retval);
		retval = -EBUSY;
		goto err3;
	}
	

	pr_debug ("%s (sm501) at 0x%p, irq %d",
		hcd->driver->description, hcd->regs, hcd->irq);

	hcd->self.bus_name = "sm501";
//	usb_register_bus (&hcd->self);

	retval = usb_add_hcd(hcd, 0, IRQF_DISABLED);
	if (retval != 0){
		printk( KERN_ERR "%s: usb_add_hcd error \n", __FUNCTION__ );
		return retval;
	}

	printk( KERN_ERR "dump ohci data\n" );
	return 0;

 err3:
	hcd_buffer_destroy (hcd);
 err2:
	usb_put_hcd(hcd);
 err1:
	sm501_stop_hc(dev);
	if (cfgBase) iounmap(cfgBase);
	if (usbBase) iounmap(usbBase);
	if (bufBase) iounmap(bufBase);
//	r = &dev->resource[0];
//	release_mem_region(r->start,r->end - r->start + 1);
	return retval;
}

/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_sm501_remove - shutdown processing for sm501-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_sm501_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
void usb_hcd_sm501_remove (struct usb_hcd *hcd, struct platform_device *dev)
{
	port_t cfgBase = sm501_usb_Data.cfgBase;
	port_t usbBase = sm501_usb_Data.usbBase;
	port_t bufBase = sm501_usb_Data.bufBase;
	pr_debug ("remove: %s, state %x", hcd->self.bus_name, hcd->state);

	if (in_interrupt ())
		BUG ();

//	hcd->state = USB_STATE_QUIESCING;

	usb_remove_hcd(hcd);
	
	release_sm501_pool_mapping();
	if (usbBase) {
		iounmap(usbBase);
	}
	// Put the USB host controller into reset.
	if (cfgBase) {
		iounmap(cfgBase);									// Release memory resources.
	}
	if (bufBase) {
		iounmap(bufBase);									// Release memory resources.
	}
	usb_put_hcd(hcd);
	sm501_stop_hc(dev);
}

/*-------------------------------------------------------------------------*/

static int __devinit
ohci_sm501_start (struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
	int		ret;

	ohci_dbg (ohci, "ohci_sm501_start, ohci:%p", ohci);

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
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11,

	/*
	 * basic lifecycle operations
	 */
	.start =		ohci_sm501_start,
	.stop =			ohci_stop,
	.shutdown =		ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
#ifdef	CONFIG_PM
	.hub_suspend =		ohci_hub_suspend,
	.hub_resume =		ohci_hub_resume,
#endif
};

/*-------------------------------------------------------------------------*/

static int ohci_hcd_sm501_drv_probe(struct platform_device *pdev)
{
	int ret;

	pr_debug ("In ohci_hcd_sm501_drv_probe");

	if (usb_disabled())
		return -ENODEV;

        pdev->dev.devIsSm501=1;

	ret = usb_hcd_sm501_probe(&ohci_sm501_hc_driver, pdev);
	printk( KERN_ERR "%s: %s\n", __FUNCTION__, 
		pdev->dev.devIsSm501 ? "IsSM501" : "Not SM501" );
	pdev->dev.devIsSm501=1;

	if (ret != 0)
		printk( KERN_ERR "%s: %d\n", __FUNCTION__, ret );
	return ret;
}

static int ohci_hcd_sm501_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_hcd_sm501_remove(hcd, pdev);
	platform_set_drvdata(pdev, NULL);
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
#endif

static struct platform_driver ohci_hcd_sm501_driver = {
	.probe		= ohci_hcd_sm501_drv_probe,
	.remove		= ohci_hcd_sm501_drv_remove,
#ifdef CONFIG_PM
	.suspend	= ohci_hcd_sm501_drv_suspend, 
	.resume		= ohci_hcd_sm501_drv_resume, 
#endif
	.driver		= {
		.name	= "sm501-ohci",
	},
};

static int __init ohci_hcd_sm501_init (void)
{
	//pr_debug (DRIVER_INFO " (sm501)");
	pr_debug ("block sizes: ed %d td %d\n",
		sizeof (struct ed), sizeof (struct td));

	return platform_driver_register(&ohci_hcd_sm501_driver);
}

static void __exit ohci_hcd_sm501_cleanup (void)
{
	platform_driver_unregister(&ohci_hcd_sm501_driver);
}

module_init (ohci_hcd_sm501_init);
module_exit (ohci_hcd_sm501_cleanup);

/////////////////////////////////////////////////////////////////////

#define ALLOC_UNIT_POWER 10
#define ALLOC_UNIT_SIZE (1<<ALLOC_UNIT_POWER)
#define ALLOC_UNIT_MASK (ALLOC_UNIT_SIZE-1)
#define ALLOC_UNIT_CNT(size) ((size+ALLOC_UNIT_MASK)>>ALLOC_UNIT_POWER)

struct sm501_pool_mapping {
	port_t bufBase;
	unsigned int base_mapping;
	unsigned int buffer_cnt;
	unsigned int freeBufStart;
	void* mapping[0];	//buffer_cnt entries
};

struct sm501_pool_mapping* sm501_pool = NULL;

void init_sm501_pool_mapping(port_t bufBase,unsigned int size,unsigned int base_mapping)
{
	struct sm501_pool_mapping* sm_pool = NULL;
	unsigned int buffer_cnt = (size>>ALLOC_UNIT_POWER);
	sm_pool = (struct sm501_pool_mapping*)kmalloc(sizeof(struct sm501_pool_mapping)+(buffer_cnt<<2), GFP_KERNEL);
	if (sm_pool) {
		sm_pool->bufBase = bufBase;
		sm_pool->base_mapping = base_mapping;
		sm_pool->buffer_cnt = buffer_cnt;
		sm_pool->freeBufStart = 0;
		memset(&sm_pool->mapping[0],0,buffer_cnt<<2);
		sm501_pool = sm_pool;
	}
}
void release_sm501_pool_mapping(void)
{
	struct sm501_pool_mapping* sm_pool = sm501_pool;
	if (sm_pool) {
		sm501_pool = NULL;
		kfree(sm_pool);
	}
}


int FindFreeBuffers(struct sm501_pool_mapping* sm_pool,int cnt)
{
	void **m = &sm_pool->mapping[0];
	int remaining = sm_pool->buffer_cnt;
	int cur = 0;
	while (remaining) {
		int freeCnt = 0;
		int start;
		while (*m) { m++; cur++; remaining--; if (remaining==0) return -1;}
		if (remaining < cnt) return -1;
		start = cur;
		while (*m==NULL) {
			freeCnt++;
			if (freeCnt>=cnt) return start;
			m++;
			cur++;
			remaining--;
		}
	}
	return -1;
}

int sm501_alloc_buffer(size_t size)
{
	struct sm501_pool_mapping* sm_pool = sm501_pool;
	if (sm_pool) {
		void* ptr;
		unsigned long flags;
		int bufNum;
		unsigned int next;
		unsigned int cnt= ALLOC_UNIT_CNT(size);
		local_irq_save(flags);
		bufNum=sm_pool->freeBufStart;
		next=bufNum+cnt;
		if (next>sm_pool->buffer_cnt) {
			//search for free spot
			bufNum = FindFreeBuffers(sm_pool,cnt);
		} else {
			sm_pool->freeBufStart = next;
		}
		if (bufNum>=0) {
			void **m = &sm_pool->mapping[bufNum];
			ptr = sm_pool->bufBase + (bufNum<<ALLOC_UNIT_POWER);
			while (cnt) {
				*m++=ptr;
				ptr = (void*)( ((unsigned char*)ptr) + ALLOC_UNIT_SIZE);
				cnt--;
			}
		}
		local_irq_restore(flags);

		if (bufNum<0) {
			printk(KERN_ERR "%s: out of buffers!!! cnt:%i\n", __func__, cnt);
		}
		return bufNum;
	}
	return -1;
}

void sm501_free_buffer(dma_addr_t dma_addr, size_t size)
{
	struct sm501_pool_mapping* sm_pool = sm501_pool;
	if (sm_pool) {
		unsigned int bufNum = (dma_addr - sm_pool->base_mapping)>>ALLOC_UNIT_POWER;
		unsigned int cnt= ALLOC_UNIT_CNT(size);
		unsigned int end=bufNum+cnt;
		if ((bufNum<end)&&(end<=sm_pool->freeBufStart)) {
			void **m = &sm_pool->mapping[bufNum];
			unsigned long flags;
			local_irq_save(flags);
			if (end==sm_pool->freeBufStart) {
				while (bufNum>0) {
					m--;
					if (*m) {break;}
					bufNum--;
				}
				sm_pool->freeBufStart = bufNum;
			} else {
				while (cnt) {
					if (*m==NULL) {
			printk(KERN_ERR "%s: already free dma_addr 0x%x! size:0x%x, cnt:0x%x, freestart:0x%x\n", __func__, dma_addr,size,cnt,sm_pool->freeBufStart);
					} else {
						*m++=NULL;
					}
					cnt--;
				}
			}
			local_irq_restore(flags);
		} else {
			printk(KERN_ERR "%s: invalid dma_addr 0x%x! size:0x%x, freestart:0x%x\n", __func__, dma_addr,size,sm_pool->freeBufStart);
		}
	}
}

dma_addr_t sm501_map_single(void * ptr, size_t size, enum dma_data_direction dir)
{
	struct sm501_pool_mapping* sm_pool = sm501_pool;
	if (sm_pool) {
		int bufNum = sm501_alloc_buffer(size);
//		printk("sm501_map_single: %i\r\n",size);
		if (bufNum < 0) {
			printk(KERN_ERR "%s: unable to map buffer %i!\n", __func__, size);
			return 0;
		}
		sm_pool->mapping[bufNum]=ptr;

		if ((dir == DMA_TO_DEVICE) || (dir == DMA_BIDIRECTIONAL)) {
			port_t virt = sm_pool->bufBase + (bufNum<<ALLOC_UNIT_POWER);
			memcpy(virt, ptr, size);
		}
		return sm_pool->base_mapping+(bufNum<<ALLOC_UNIT_POWER);
	} else {
			printk(KERN_ERR "%s: pool_mapping not initialized\n", __func__);
			return 0;
	}
}

void sm501_unmap_single(dma_addr_t dma_addr, size_t size, enum dma_data_direction dir)
{
	struct sm501_pool_mapping* sm_pool = sm501_pool;
	if (sm_pool) {
//		printk("sm501_unmap_single: %i\r\n",size);
		if (dma_addr & ALLOC_UNIT_MASK) {
			printk(KERN_ERR "%s: invalid dma_addr 0x%x!\n", __func__, dma_addr);
		} else {
			if ((dir == DMA_FROM_DEVICE) || (dir == DMA_BIDIRECTIONAL)) {
				unsigned int bufNum = (dma_addr - sm_pool->base_mapping)>>ALLOC_UNIT_POWER;
				port_t virt = sm_pool->bufBase + (bufNum<<ALLOC_UNIT_POWER);
				if (bufNum<sm_pool->buffer_cnt) {
					void* p = sm_pool->mapping[bufNum];
					if (p) memcpy(p, virt, size);
					else {
						printk(KERN_ERR "%s: unable to find mapping for num %i!\n", __func__, bufNum);
					}
				} else {
					printk(KERN_ERR "%s: invalid dma_addr 0x%x!\n", __func__, dma_addr);
				}
			}
			sm501_free_buffer(dma_addr,size);
		}
	}
}

void * sm501_alloc_coherent(size_t size, dma_addr_t *handle)
{
	struct sm501_pool_mapping* sm_pool = sm501_pool;
	if (sm_pool) {
		void* ptr;
		int bufNum = sm501_alloc_buffer(size);
		if (bufNum >= 0) {
//			printk("sm501_alloc_coherent: %i\r\n",size);
			*handle = sm_pool->base_mapping+(bufNum<<ALLOC_UNIT_POWER);
			ptr = sm_pool->bufBase + (bufNum<<ALLOC_UNIT_POWER);
			sm_pool->mapping[bufNum]=ptr;
			return ptr;
		}
	}
	return NULL;
}

void sm501_free_coherent(size_t size, void *vaddr, dma_addr_t dma_handle)
{
	printk("sm501_free_coherent: %i\r\n",size);
	sm501_free_buffer(dma_handle,size);
}

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
#if 0
struct dma_pool {
	struct dma_pool* next;
	unsigned long free_mask;
	size_t unit_size;
	dma_addr_t handle;
	void * bufStart;
};

struct dma_pool * get_pool_buffer(size_t unit_size)
{
	struct dma_pool* p = (struct dma_pool*)kmalloc(sizeof(*p), GFP_KERNEL);
	if (p) {
		p->next = NULL;
		p->free_mask = 0xffffffff;
		p->unit_size = unit_size;
		p->bufStart = sm501_alloc_coherent(unit_size<<5,&p->handle);
		if (p->bufStart==NULL) {
			kfree(p);
			p=NULL;
		}
	}
	if (p==NULL) {
		printk(KERN_ERR "get_pool_buffer failure, size:%i\r\n",unit_size);
	}
	return p;
}


struct dma_pool *dma_pool_create (const char *name, struct device *dev,
		size_t size, size_t align, size_t allocation)
{
	size_t unit_size;
	if (align<1) align=1;
	unit_size = (size+align-1) & ~(align-1);

	printk("dma_pool_create: %i %i\r\n",size,unit_size);
	return get_pool_buffer(unit_size);
}

void dma_pool_destroy (struct dma_pool *pool)
{
	struct dma_pool* next;
	printk("dma_pool_destroy\n");
	while (pool) {
		next = pool->next;
		pool->next = NULL;
		sm501_free_coherent(pool->unit_size<<5,pool->bufStart,pool->handle);
		kfree(pool);
		pool = next;
	}
}

void *dma_pool_alloc (struct dma_pool *pool, int flags1, dma_addr_t *handle)
{
	unsigned long flags;
	void* p =NULL;
	int i;
//	printk("dma_pool_alloc\n");
	local_irq_save(flags);
	while (pool->free_mask==0) {
		if (pool->next==NULL) {
			pool->next = get_pool_buffer(pool->unit_size);
			pool= pool->next;
			break;
		}
		pool = pool->next;
	}
	if (pool) {
		unsigned int offset;
		i = __ffs(pool->free_mask);
		pool->free_mask &= ~(1<<i);
		offset = (pool->unit_size * i);
		p = pool->bufStart + offset;
		*handle = pool->handle + offset;
	} else {
		*handle = -1;
	}
	local_irq_restore(flags);
	return p;
}

void dma_pool_free (struct dma_pool *pool, void *vaddr, dma_addr_t addr)
{
	unsigned long flags;
	unsigned int size = pool->unit_size<<5;
//	printk("dma_pool_free\n");
	local_irq_save(flags);
	while (pool) {
		unsigned int diff = vaddr - pool->bufStart;
		if (diff < size) {
			unsigned int i = diff/pool->unit_size;
			pool->free_mask |= (1<<i);
			break;
		}
		pool = pool->next;
	}
	local_irq_restore(flags);
	if (pool==NULL) {
		printk(KERN_ERR "dma_pool_free failure\n");
	}
}
#endif

int sm501_map_sg(struct scatterlist * sg, int nents, enum dma_data_direction dir)
{
	int i ;
	for (i = 0; i < nents; i++, sg++) {
		char *virt = sg_virt(sg) + sg->offset ;
		sg->dma_address = sm501_map_single( virt, sg->length, dir );
		dma_cache_maint(virt, sg->length, dir);
	}

	return nents;
}
void sm501_unmap_sg(struct scatterlist * sg, int nents, enum dma_data_direction dir)
{
	int i ;
	for (i = 0; i < nents; i++, sg++) {
           sm501_unmap_single(sg->dma_address, sg->length, dir );
	}

}
void sm501_dma_sync_single(dma_addr_t dma_handle, size_t size, enum dma_data_direction dir)
{
	panic("sm501_dma_sync_single");
}
void sm501_dma_sync_sg(struct scatterlist * sg, int nelems, enum dma_data_direction dir)
{
	panic("sm501_dma_sync_sg");
}
