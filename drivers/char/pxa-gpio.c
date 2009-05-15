//
//	Boundary Devices GPIO Driver-
//	Copyright (C) Troy Kisky <troy.kisky@boundarydevices.com> 2002.
//


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/poll.h>

#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/pxa-gpio.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <mach/hardware.h>
#include <asm/system.h>
#include <asm/mach/map.h>
#include <asm/bitops.h>
#include <asm/ioctls.h>
#include <mach/pxa2xx-gpio.h>

//#define INPUT_EVENT_INTERFACE
#ifdef INPUT_EVENT_INTERFACE
#include <linux/input.h>
#endif

//#define CALLBACK_INTERFACE
#define DEVICE_INTERFACE
#define CONFIG_SINGLE_DATA
#define CONFIG_QUEUED_DATA

#define ASYNC_INTERFACE
// #define CONFIG_ALLOW_FIQ		-- enable after porting arch/arm/kernel/fiq.c


#if defined(CALLBACK_INTERFACE) || defined(CONFIG_SINGLE_DATA) || defined(ASYNC_INTERFACE)
#define DEBOUNCE_LOGIC
#endif


#define MAX_GPIO 120	//0-80
#define DEBOUNCE_LOW_TIME 100		//in milliseconds
#define DEBOUNCE_HIGH_TIME 0

#define GPIO_TO_IRQ(a) IRQ_GPIO(a)


#ifdef CONFIG_ALLOW_FIQ
#include <asm/fiq.h>
#include <asm/arch/dma.h>
#define FIQ_STACK_SIZE (0x100-16)
struct fiq_struct {
	unsigned char stack[FIQ_STACK_SIZE]  __attribute__ ((aligned (16)));
	irqreturn_t (*callback)(int irq, void *dev_id);
	void* priv;
	unsigned char activeMask;	//3 fields, bit0 : gp0, bit1 : gp1, bit2-7 : gp2-80
	unsigned char available;
	unsigned char spare1;
	unsigned char spare2;
	unsigned long happenedMask[3];
	volatile u32 * dcsr;
	int dmaChannel;
	unsigned long fiqMask[3];
	unsigned int gpioMask;
	spinlock_t lock;
	struct fiq_handler fiq_st;
};
#endif

#ifdef CALLBACK_INTERFACE
struct gpio_callback;
struct gpio_callback {
	struct gpio_callback* next;
	void (*callback) (unsigned gpio,int level);
};
#endif

#ifdef CONFIG_QUEUED_DATA


struct gpWork {
	unsigned char sampleGp;
	unsigned char sampleFlags;
};
struct gpWorkArray {
	int m_cnt;
	struct gpWork m_gpWork[0];
};

#define DEFAULT_GPEVENTS 16
#define MAX_GPEVENTS 8192
struct gp_queued_def {
	struct gpWorkArray* m_gpWorkArray[MAX_GPIO];
	wait_queue_head_t m_wait_queue; // Used for blocking read
	unsigned long m_cleanupFiqIndex;
	unsigned long m_insertIndex;
	unsigned long m_removeIndex;
	unsigned long m_allocCnt;
	char overrun;
	struct gpEvent m_gpEvent[0];
};
#endif


struct dev_gpio;
struct dev_gpio
{
#ifdef DEBOUNCE_LOGIC
#define NOT_IN_QUEUE (struct dev_gpio*)1
	struct dev_gpio* gpio_next;
	unsigned long gpio_transitions;		//count of # of transitions
        unsigned long gpio_transitions_read;    // number of transitions read
	unsigned long gpio_debounce_low;
	unsigned long gpio_debounce_high;
	unsigned long gpio_debounce_expiration;
#endif

#ifdef DEVICE_INTERFACE
	wait_queue_head_t gpio_wait_queue; // Used for blocking read
#endif

#ifdef ASYNC_INTERFACE
	struct fasync_struct *gpio_async_queue;
#endif

#ifdef CALLBACK_INTERFACE
	struct gpio_callback* gpio_callback_list;
	spinlock_t gpio_callbacklist_lock;
#endif
	unsigned char gpio_direction;
	unsigned char gpio_level;
	unsigned char gpio_gpio;
	unsigned char gpio_regMask;		//bit 0,1,2 - which gp regs need read upon transition, bit 5 - read timestamp
	unsigned char gpio_edges;
	struct timer_list pulse_timer;	/* Times for the end of a pulse */

#ifdef CONFIG_QUEUED_DATA
//	int	prevJiffies;		//fiq uses difference between jiffies to update timestamp
	struct timeval gpio_timeStamp;
	unsigned gpio_tgplr[4];
	spinlock_t gpio_open_defs_lock;
#define MAX_OPEN_DEFS 16

#define NOT_IN_LIST (struct gp_queued_def*)1
	struct gp_queued_def* gpio_open_defs[MAX_OPEN_DEFS];
#endif
};

struct gpioData
{
#ifdef INPUT_EVENT_INTERFACE
	struct input_dev	g_idev;
#endif
#ifdef DEBOUNCE_LOGIC
	struct timer_list g_timer;	/* Times for the end of a sequence */
	spinlock_t g_debounce_lock;
	struct dev_gpio * volatile g_debounce_head;
	struct dev_gpio * volatile g_debounce_tail;
#endif
	struct dev_gpio * volatile g_dev_gpios[MAX_GPIO];
#ifdef CONFIG_ALLOW_FIQ
	struct fiq_struct fiq;
#endif
};


static __u8 invert_bytes[INVERT_BYTES] = {0};
static inline int invert_bit(int gpio)
{
	return 0 != (invert_bytes[(gpio/8)] & (1<<(gpio&7)));
}

static struct gpioData * g_gpioData;

void Do_Expiration(struct dev_gpio* dev)
{
	dev->gpio_transitions++;
#ifdef CALLBACK_INTERFACE
	{
		struct gpio_callback* p = dev->gpio_callback_list;
		while (p) {
			p->callback(gpio,dev->gpio_level);
			p = p->next;
		}
	}
#endif
#ifdef CONFIG_SINGLE_DATA
	wake_up_interruptible (&dev->gpio_wait_queue);
#endif

#ifdef ASYNC_INTERFACE
	kill_fasync (&dev->gpio_async_queue, SIGIO, POLL_IN);
#endif
}

#ifdef DEBOUNCE_LOGIC
void CheckExpirationTimerCallback(unsigned long parameters)
{
	struct gpioData * g = g_gpioData;
	if (g) {
		unsigned long flags;
		int jif;
		struct dev_gpio* dev;
		struct dev_gpio* prevdev = NULL;
		int exp=0x7fffffff;
		//spin may not be needed, but play it safe
		spin_lock_irqsave(&g->g_debounce_lock,flags);
		jif = jiffies;
		dev = g->g_debounce_head;
		while (dev) {
			int delay= (int)(dev->gpio_debounce_expiration - jif);
			if ( delay <= 0) {
				struct dev_gpio* n = dev->gpio_next;
				if (prevdev) prevdev->gpio_next = n;
				else g->g_debounce_head = n;
				if (!n) g->g_debounce_tail = prevdev;
				dev->gpio_next = NOT_IN_QUEUE;
				Do_Expiration(dev);
				dev = n;
			} else {
				if (exp > delay) exp = delay;
				prevdev = dev;
				dev = dev->gpio_next;
			}
		}
		if (g->g_debounce_head) {
			g->g_timer.expires = exp + jif;
			add_timer(&g->g_timer);
		}
		spin_unlock_irqrestore(&g->g_debounce_lock,flags);
	}
}
#endif

static void endPulse(unsigned long parameters)
{
        unsigned gpio = MINOR (parameters);
	unsigned high = (0 != (parameters&0x80000000));
	if (high) {
		GPCR(gpio) = GPIO_bit(gpio);
	} else {
		GPSR(gpio) = GPIO_bit(gpio);
	}
}

void CheckExpired(struct dev_gpio* dev)
{
#ifdef DEBOUNCE_LOGIC
	int expiration = (dev->gpio_level)? dev->gpio_debounce_high : dev->gpio_debounce_low;
	if (expiration) {
		int jif = jiffies;
		int n_exp = dev->gpio_debounce_expiration = expiration+jif;
		struct gpioData * g = g_gpioData;
		if (g) {
			if (dev->gpio_next==NOT_IN_QUEUE) {
				unsigned long flags;
				dev->gpio_next = NULL;
//this may not be needed when called from gpio_handler
//but it can also be called when a new input pin is defined.
				spin_lock_irqsave(&g->g_debounce_lock,flags);
				if (g->g_debounce_head) {
					g->g_debounce_tail->gpio_next = dev;
					g->g_debounce_tail = dev;
					if (expiration < (int)(g->g_timer.expires-jif)) {
						mod_timer(&g->g_timer,n_exp);
					}
				} else {
					g->g_debounce_head = g->g_debounce_tail = dev;
					g->g_timer.expires = n_exp;
					add_timer (&g->g_timer);
				}
				spin_unlock_irqrestore(&g->g_debounce_lock,flags);
			} else {
				if (expiration < (int)(g->g_timer.expires-jif)) {
					mod_timer(&g->g_timer,n_exp);
				}
			}
		}
	}
	else
#endif
		Do_Expiration(dev);
}



#ifdef CONFIG_QUEUED_DATA
#define local_irqfiq_save(x)					\
	({							\
		unsigned long temp;				\
	__asm__ __volatile__(					\
	"mrs    %0, cpsr                @ local_irqfiq_save\n"	\
"       orr     %1, %0, #128+64\n"				\
"       msr     cpsr_c, %1"					\
	: "=r" (x), "=r" (temp)					\
	:							\
	: "memory");						\
	})


static inline int SetOnMatch(volatile unsigned long *ptr,unsigned long match,unsigned long new)
{
	unsigned long flags;
	int ret;
	local_irqfiq_save(flags);
	ret = (*ptr==match);
	if (ret) *ptr=new;
	local_irq_restore(flags);
	return ret;
}
static inline unsigned long xchng(volatile unsigned long *ptr,unsigned long new)
{
	if (*ptr==new) return new;
	{
		unsigned long flags;
		unsigned long ret;
		local_irqfiq_save(flags);
		ret = *ptr;
		*ptr = new;
		local_irq_restore(flags);
		return ret;
	}
}
//static inline void UpdateTimeStamp(struct timespec *value,int amount)
//{
//	if (amount>0) {
//		value->tv_nsec = (amount % HZ) * (1000000000L / HZ);
//		value->tv_sec = amount / HZ;
//	}
//}
static int scanQueuedWork(struct dev_gpio* dev,unsigned gpio,int level,int irq)
{
	unsigned int tgpsr[3];
	unsigned int tgpcr[3];
	int outputUpdate=0;
	int i;
	int retVal = 0;
#ifdef CONFIG_ALLOW_FIQ
	unsigned char fiqFlag=0;
	if (irq<0) {
//		int jiff = jiffies;
//		int amount = jiff - dev->prevJiffies;
//		dev->prevJiffies = jiff;
//		UpdateTimeStamp(&dev->gpio_timeStamp,amount)
		fiqFlag = SAMPLE_FIQ;
	} else
#endif
	{
		if (dev->gpio_regMask & SAMPLE_TIMESTAMP) {
			do_gettimeofday(&dev->gpio_timeStamp);
//			prevJiffies = jiffies;
		}
	}
	for (i=0; i<MAX_OPEN_DEFS; i++) {
		struct gp_queued_def* q = dev->gpio_open_defs[i];
		if (!q) break;
		if (q != NOT_IN_LIST) {
			struct gpWorkArray* gpA = q->m_gpWorkArray[gpio];
			if (gpA) {
				int cnt = gpA->m_cnt;
				int updated = 0;
				struct gpWork* gpW =  &gpA->m_gpWork[0];

				while (cnt) {
					unsigned char sampleFlags = gpW->sampleFlags
#ifdef CONFIG_ALLOW_FIQ
					| fiqFlag
#endif
					   ;
					int sampleGp = gpW->sampleGp;
					if (sampleFlags & (SAMPLE_INPUT_RISING | SAMPLE_INPUT_FALLING)) {
						if ( (level && (sampleFlags&SAMPLE_INPUT_RISING)) ||
						     ((!level) && (sampleFlags&SAMPLE_INPUT_FALLING)) ) {
							do {
								unsigned long match = q->m_insertIndex;
								unsigned long newVal = match+1;
								if (newVal >= q->m_allocCnt) newVal = 0;
								if (newVal == q->m_removeIndex) {
									gpW->sampleFlags |= SAMPLE_OVERRUN;
									q->overrun=1;
									updated = 1;
									break;
								}
								if (SetOnMatch(&q->m_insertIndex,match,newVal)) {
									struct gpEvent* gpE = &q->m_gpEvent[match];
									gpE->timeStamp.tv_sec = dev->gpio_timeStamp.tv_sec;
									gpE->timeStamp.tv_usec = dev->gpio_timeStamp.tv_usec;
									gpE->inputGp = gpio;
									gpE->sampleGp = sampleGp;
									gpE->sampleFlags = sampleFlags;
									gpE->level = ((dev->gpio_tgplr[sampleGp>>5] & GPIO_bit(sampleGp))? 1 : 0);
									gpW->sampleFlags &= ~SAMPLE_OVERRUN;
									updated = 1;
									break;
								}
							} while (1);
						}
					} else {
						if (sampleFlags>>2) {
							if ( (level && (sampleFlags&SAMPLE_OUTPUT_RISING)) ||
							     ((!level) && (sampleFlags&SAMPLE_OUTPUT_FALLING)) ) {
								if (outputUpdate==0) {
									outputUpdate = 1;
									tgpsr[0] = tgpsr[1] = tgpsr[2] = tgpcr[0] = tgpcr[1] = tgpcr[2] = 0;
								}
								if (level ^ ((sampleFlags&SAMPLE_OUTPUT_INVERT)? 1: 0)) {
									tgpsr[sampleGp>>5] |= GPIO_bit(sampleGp);
								} else {
									tgpcr[sampleGp>>5] |= GPIO_bit(sampleGp);
								}
							}
						}
					}
					gpW++;
					cnt--;
				}
				if (updated) {
					retVal++;
#ifdef CONFIG_ALLOW_FIQ
					if (irq>=0)
#endif
					{
						wake_up_interruptible (&q->m_wait_queue);
					}
				}
			}
		}
	}
	if (outputUpdate) {
		if (tgpsr[0]) GPSR(0)  = tgpsr[0];
		if (tgpsr[1]) GPSR(32) = tgpsr[1];
		if (tgpsr[2]) GPSR(64) = tgpsr[2];
		if (tgpcr[0]) GPCR(0)  = tgpcr[0];
		if (tgpcr[1]) GPCR(32) = tgpcr[1];
		if (tgpcr[2]) GPCR(64) = tgpcr[2];
	}
	return retVal;
}
#endif

#ifdef CONFIG_ALLOW_FIQ
//fix timestamp for fiqs
static void scanDev(struct dev_gpio* dev)
{
	int i;
	if (dev->gpio_regMask & SAMPLE_TIMESTAMP) do_gettimeofday(&dev->gpio_timeStamp);

	for (i=0; i<MAX_OPEN_DEFS; i++) {
		struct gp_queued_def* q = dev->gpio_open_defs[i];
		if (!q) break;
		if (q != NOT_IN_LIST) {
			int startIndex = q->m_cleanupFiqIndex;
			int endIndex = q->m_insertIndex;
			if (startIndex != endIndex) {
				q->m_cleanupFiqIndex = endIndex;
				if (dev->gpio_regMask & SAMPLE_TIMESTAMP) {
					while (startIndex != endIndex) {
						struct gpEvent* gpE = &q->m_gpEvent[startIndex];
						if (gpE->sampleFlags & SAMPLE_FIQ) {
							gpE->timeStamp.tv_sec = dev->gpio_timeStamp.tv_sec;
							gpE->timeStamp.tv_usec = dev->gpio_timeStamp.tv_usec;
						}
						startIndex++;
						if (startIndex >= q->m_allocCnt) startIndex = 0;
					}
				}
				wake_up_interruptible (&q->m_wait_queue);
			}
		}
	}
}

void add_to_pending_list(int irq);

//this is used to convert FIQs into IRQs
static void gpio_dma_interrupt(int ch, void *g_id)
{
	struct gpioData * g = g_id;
	int gpio;
	unsigned long* pmask = &g->fiq.happenedMask[0];
	if (g->fiq.dcsr) {
		*g->fiq.dcsr = DCSR_NODESC;
	}
	for (gpio=0; gpio<MAX_GPIO; gpio+=32) {
		struct dev_gpio* dev;
		int bitno;
		unsigned long mask = xchng(pmask,0);
		do {
			bitno = fls(mask)-1;
			if (bitno<0) break;
			mask &= ~(1<<bitno);
			bitno += gpio;
			dev = g->g_dev_gpios[bitno];
			if (dev) {
				scanDev(dev);
				CheckExpired(dev);
			} else {
				add_to_pending_list(GPIO_TO_IRQ(bitno));
			}
		} while (1);
		pmask++;
	}
}
#endif
//  This handler is called on gpio transitions
//if irq<0 then this is called from my FIQ routine
static irqreturn_t gpio_handler (int irq, void *dev_id)
{
	struct dev_gpio* dev= (struct dev_gpio*)dev_id;
	unsigned gpio = dev->gpio_gpio;
	if (gpio<MAX_GPIO) {
		struct gpioData * g = g_gpioData;
		struct dev_gpio * d = NULL;
		int level;
		if (g) d = g->g_dev_gpios[gpio];
		if (!dev || (dev != d)) {
			printk(KERN_INFO __FILE__ ": spurious interrupt %d, gpio %d\n\r", irq,gpio);
			return IRQ_HANDLED; /* spurious interrupt */
		}
		dev->gpio_tgplr[0] = (dev->gpio_regMask & 1) ? GPLR(0) :0;
		dev->gpio_tgplr[1] = (dev->gpio_regMask & 2) ? GPLR(32) : 0;
		dev->gpio_tgplr[2] = (dev->gpio_regMask & 4) ? GPLR(64) : 0;
		dev->gpio_tgplr[3] = (dev->gpio_regMask & 8) ? GPLR(96) : 0;

		if ( (dev->gpio_edges & SAMPLE_INPUT_BOTH) == SAMPLE_INPUT_BOTH)
			dev->gpio_level = level = ((dev->gpio_tgplr[gpio>>5] & GPIO_bit(gpio))? 1 : 0);
		else {
			dev->gpio_level = level = ((dev->gpio_edges & SAMPLE_INPUT_RISING)==0)  ? 0 : 1;
			if (level) dev->gpio_tgplr[gpio>>5] |= GPIO_bit(gpio);
			else  dev->gpio_tgplr[gpio>>5] &= ~(GPIO_bit(gpio));
		}
#ifdef CONFIG_QUEUED_DATA
		scanQueuedWork(dev,gpio,level,irq);
#endif


#ifdef CONFIG_ALLOW_FIQ
		if (irq>=0)
#endif
			CheckExpired(dev);
	}
	return IRQ_HANDLED;
}

#define IN 0	//values direction can take
#define OUT 1
struct dev_gpio* GetAllocGpioDev(unsigned gpio,int direction,int dirMask,int edges)
{
	struct dev_gpio* dev = NULL;
	struct gpioData * g = g_gpioData;
	if ((!g) || (gpio>=MAX_GPIO)) return NULL;
	dev = g->g_dev_gpios[gpio];

	if (dev) {
		if ((dev->gpio_direction&dirMask) != direction) dev = NULL;
		else {
			if ((dev->gpio_direction == IN) && (edges & ~dev->gpio_edges)) {
				edges = dev->gpio_edges |= edges;
				set_irq_type(IRQ_GPIO(gpio), edges);
				dev->gpio_level =(GPLR(gpio) & GPIO_bit(gpio)) ? 1 : 0;
			}
		}
	} else {
		//this is only valid if GPIO is setup as same direction with NO alternate function
		int dir=(GPDR(gpio) & (GPIO_bit(gpio))) ? 1:0;
		if ( ((dir&dirMask)==direction) && ((GAFR(gpio)&(3<<((gpio&0xf)<<1)))==0) ) {
			//initialize this as an input
			dev = (struct dev_gpio*) kmalloc(sizeof(struct dev_gpio),GFP_KERNEL);
			if (dev) {
				memset(dev,0,sizeof(struct dev_gpio));
				dev->gpio_next = NOT_IN_QUEUE;
#ifdef DEVICE_INTERFACE
				init_waitqueue_head(&dev->gpio_wait_queue);
#endif
#ifdef CONFIG_QUEUED_DATA
				spin_lock_init(&dev->gpio_open_defs_lock);
#endif
				dev->gpio_direction = dir;
                                init_timer (&dev->pulse_timer);
                                dev->pulse_timer.function = endPulse ;
				dev->gpio_gpio = gpio;
				dev->gpio_transitions_read = dev->gpio_transitions-1 ; // force successful read
				dev->gpio_level =(GPLR(gpio) & GPIO_bit(gpio)) ? 1 : 0;
#ifdef DEBOUNCE_LOGIC
				dev->gpio_debounce_low = (DEBOUNCE_LOW_TIME * HZ + 999)/1000;
				dev->gpio_debounce_high = (DEBOUNCE_HIGH_TIME * HZ + 999)/1000;
#endif
#ifdef CONFIG_SINGLE_DATA
				dev->gpio_regMask = 1 << (gpio >> 5);
#endif
				if (dir==IN) {
					dev->gpio_edges = edges;
					set_irq_type(IRQ_GPIO(gpio), edges);
					if (request_irq(GPIO_TO_IRQ(gpio), gpio_handler, IRQF_DISABLED, "PXA gpio pins", dev)<0) {
						kfree(dev);
						dev=NULL;
						printk(KERN_INFO __FILE__ ": irq %d unavailable\n\r", GPIO_TO_IRQ(gpio));
					}
				}
				if (g->g_dev_gpios[gpio]==0) {
					g->g_dev_gpios[gpio]=dev;
					if (dir==IN) 
						if (dev){
							CheckExpired(dev);
						}
				} else {
					printk( KERN_ERR "someone else got our GPIO (handler is screwed)\n" );
					if (dev) kfree(dev);
					dev=g->g_dev_gpios[gpio];
					if (dev) if ((dev->gpio_direction&dirMask) != direction) dev = NULL;
				}
			}
		}
	}
	return dev;
}

#ifdef CALLBACK_INTERFACE
// This function is called by other drivers to register a callback function.
int gpio_add_callback (void (*callback) (unsigned ,int), unsigned gpio)
{
	int return_val=-EINVAL;
	if (callback) {
		struct dev_gpio* dev= GetAllocGpioDev(gpio,IN,1,IRQ_TYPE_EDGE_BOTH);
		if (dev) {
			struct gpio_callback* gp_call = kmalloc(sizeof(struct gpio_callback),GFP_KERNEL);
			if (gp_call) {
				struct gpio_callback** p;
				gp_call->next = NULL;
				gp_call->callback = callback;

				spin_lock(&dev->gpio_callbacklist_lock);
				p = &dev->gpio_callback_list;
				while (*p) p = &(*p)->next;
				*p = gp_call;
				spin_unlock(&dev->gpio_callbacklist_lock);
				return_val=0;
			} else return_val= -ENOMEM;
		}
	}
	return return_val;
}

// This function is called by other drivers to deregister a callback function.
int gpio_del_callback (void (*callback) (unsigned ,int), unsigned gpio)
{
	int return_val = -EINVAL;
	if ( (gpio<MAX_GPIO) && callback ) {
		struct gpioData * g = g_gpioData;
		struct dev_gpio* dev= (g && (gpio<MAX_GPIO)) ? g->g_dev_gpios[gpio] : NULL;
		if (dev) {
			struct gpio_callback** pp = &dev->gpio_callback_list;
			struct gpio_callback* p;
			spin_lock(&dev->gpio_callbacklist_lock);
			p = *pp;
			while (p) {
				if (p->callback == callback) {
					*pp = p->next;
					kfree(p);
					return_val=0;
					break;
				}
				pp = &p->next;
				p = *pp;
			}
			spin_unlock(&dev->gpio_callbacklist_lock);
		}
	}
	return return_val;
}
#endif

#ifdef DEVICE_INTERFACE
// This function is called when a user space program attempts to write /dev/gpio
static ssize_t gpio_write (struct file *filp, const char *buffer,size_t count, loff_t *ppos)
{
	int return_val=-EINVAL;
#ifdef CONFIG_QUEUED_DATA
	struct gp_queued_def* qdef = (struct gp_queued_def*)filp->private_data;
	if (qdef) {
		char ch;
		unsigned int gpio;
		return_val = 0;
		while (count) {
			if (copy_from_user (&ch, buffer, 1)) {
				if (return_val==0) return_val = -EFAULT;
				break;
			}
			gpio = (ch&0x7f);
			if (gpio < MAX_GPIO) {
				if (ch&0x80) {
					GPSR(gpio) = GPIO_bit(gpio);
				} else {
					GPCR(gpio) = GPIO_bit(gpio);
				}
			}
			return_val++;
			buffer++;
			count--;
		}
		return return_val;
	}
#endif
#ifdef CONFIG_SINGLE_DATA
	{
		unsigned int gpio = MINOR (filp->f_dentry->d_inode->i_rdev);
		struct dev_gpio* dev= GetAllocGpioDev(gpio,OUT,1,0);
		if (dev) {
			char ch;
			return_val = (copy_from_user (&ch, buffer, 1))? -EFAULT : 1;
			if (return_val>=0) {
				ch ^= invert_bit(gpio);
				if (ch&1) {
					GPSR(gpio) = GPIO_bit(gpio);
				} else {
					GPCR(gpio) = GPIO_bit(gpio);
				}
				dev->gpio_level=(ch&1);
				dev->gpio_transitions++;
			}
		}
	}
#endif
	return return_val;
}

static unsigned int gpio_poll(struct file *filp, poll_table *wait)
{
	int return_val=-EINVAL;
#ifdef CONFIG_QUEUED_DATA
	struct gp_queued_def* qdef = (struct gp_queued_def*)filp->private_data;
	if (qdef) {
		return_val = 0;
		poll_wait (filp, &qdef->m_wait_queue, wait);
		return ((qdef->m_insertIndex==qdef->m_removeIndex) && (qdef->overrun==0)) ? 0 : POLLIN | POLLRDNORM;
	}
#endif
#ifdef CONFIG_SINGLE_DATA
	{
		unsigned int gpio = MINOR (filp->f_dentry->d_inode->i_rdev);
		struct dev_gpio* dev= GetAllocGpioDev(gpio,0,0,IRQ_TYPE_EDGE_BOTH);
		if (dev) {
			poll_wait (filp, &dev->gpio_wait_queue, wait);
			return (dev->gpio_transitions_read==dev->gpio_transitions) ? 0 : POLLIN | POLLRDNORM;
		}
	}
#endif
	return return_val;
}

#ifdef CONFIG_QUEUED_DATA
static int OverrunSync(struct gp_queued_def* qdef,char *buffer,size_t count)
{
	int return_val = 0;
	struct gpioData * g = g_gpioData;
	int gpio;
	const int size = sizeof(struct gpEvent);
	qdef->overrun = 0;
	if (g) {
		for (gpio=0; gpio<MAX_GPIO; gpio++) {
			struct gpWorkArray* wa = qdef->m_gpWorkArray[gpio];
			if (wa) {
				struct dev_gpio * dev = g->g_dev_gpios[gpio];
				if (dev) {
					int cnt = wa->m_cnt;
					struct gpWork* w = &wa->m_gpWork[0];
					struct gpEvent gpE;
					gpE.timeStamp.tv_sec = dev->gpio_timeStamp.tv_sec;
					gpE.timeStamp.tv_usec = dev->gpio_timeStamp.tv_usec;
					gpE.inputGp = gpio;
					while (cnt) {
						int sampleGp = w->sampleGp;
						int edge = w->sampleFlags;
						if ((edge & SAMPLE_OVERRUN) && (sampleGp<MAX_GPIO)) {
							if (count < size) {
								qdef->overrun = 1;	//need to check for more
								return return_val;
							}
							gpE.sampleGp = sampleGp;
							gpE.sampleFlags = edge;
							gpE.level = ((dev->gpio_tgplr[sampleGp>>5] & GPIO_bit(sampleGp))? 1 : 0);
							if (copy_to_user (buffer, &gpE, size)) {
								qdef->overrun = 1;	//need to check for more
								return (return_val) ? return_val : -EFAULT;
							}
							w->sampleFlags &= ~SAMPLE_OVERRUN;
							return_val += size;
							buffer += size;
							count -= size;
						}
						cnt--;
						w++;
					}
				}
			}
		}
	}
	return return_val;
}
static int OverrunCnt(struct gp_queued_def* qdef)
{
	int return_val = 0;
	struct gpioData * g = g_gpioData;
	int gpio;
	if (g) {
		for (gpio=0; gpio<MAX_GPIO; gpio++) {
			struct gpWorkArray* wa = qdef->m_gpWorkArray[gpio];
			if (wa) {
				struct dev_gpio * dev = g->g_dev_gpios[gpio];
				if (dev) {
					int cnt = wa->m_cnt;
					struct gpWork* w = &wa->m_gpWork[0];
					while (cnt) {
						int sampleGp = w->sampleGp;
						int edge = w->sampleFlags;
						if ((edge & SAMPLE_OVERRUN) && (sampleGp<MAX_GPIO)) {
							return_val++;
						}
						cnt--;
						w++;
					}
				}
			}
		}
	}
	return return_val;
}
#endif
// This function is called when a user space program attempts to read /dev/gpio
static ssize_t gpio_read (struct file *filp, char *buffer,size_t count, loff_t *ppos)
{
	int return_val=-EINVAL;
#ifdef CONFIG_QUEUED_DATA
	struct gp_queued_def* qdef = (struct gp_queued_def*)filp->private_data;
	if (qdef) {
		return_val = 0;
		do {
			while (qdef->m_insertIndex != qdef->m_removeIndex) {
				int nextRemove;
				int past;
				int cnt;
				int size;
				if (qdef->m_insertIndex >= qdef->m_removeIndex) {
					nextRemove = past = qdef->m_insertIndex;
				} else {
					past = qdef->m_allocCnt;
					nextRemove = 0;
				}
				cnt = past - qdef->m_removeIndex;
				size = cnt*sizeof(struct gpEvent);
				if (size > count) {
					cnt = count / sizeof(struct gpEvent);
					if (cnt==0) return return_val;
					size = cnt*sizeof(struct gpEvent);
					nextRemove = qdef->m_removeIndex + cnt;
				}
				if (copy_to_user (buffer, &qdef->m_gpEvent[qdef->m_removeIndex], size))
					return (return_val) ? return_val : -EFAULT;
				qdef->m_removeIndex = nextRemove;
				return_val += size;
				buffer += size;
				count -= size;
				mb();
			}
			if (qdef->overrun) {
				return_val += OverrunSync(qdef,buffer,count);
			}
			if (return_val) break;
			if (filp->f_flags & O_NONBLOCK) break;
			return_val = wait_event_interruptible(qdef->m_wait_queue,
					 (qdef->m_insertIndex != qdef->m_removeIndex));
			if (return_val) break;
		} while (1);
		return return_val;
	}
#endif
#ifdef CONFIG_SINGLE_DATA
	{
		unsigned int gpio = MINOR (filp->f_dentry->d_inode->i_rdev);
		struct dev_gpio* dev= GetAllocGpioDev(gpio,0,0,IRQ_TYPE_EDGE_BOTH);
		if (dev) {
			char ch;
			if (dev->gpio_direction!=IN){
				printk( KERN_ERR "%s: gpio %d not an input\n", __func__, gpio );
				return -EFAULT ;
			}

			if ((filp->f_flags & O_NONBLOCK)==0) {
				if (dev->gpio_transitions_read==dev->gpio_transitions) {
					return_val = wait_event_interruptible(dev->gpio_wait_queue,
							(dev->gpio_transitions_read!=dev->gpio_transitions));
					if (return_val) return return_val;
				}
			}
			dev->gpio_transitions_read=dev->gpio_transitions;
			ch = ((dev->gpio_level) ? '1' : '0') ^ invert_bit(gpio);
			return_val = (copy_to_user (buffer, &ch, 1))? -EFAULT : 1;
		}
	}
#endif
	return return_val;
}


#ifdef CONFIG_ALLOW_FIQ
void * gpioGetPxaFiq(int* length,int* lockedLength);

static void gpioSetupFiq(struct fiq_struct* fiq)
{
	void* start;
	int length;
	int lockedLength;
	struct pt_regs regs;
	struct gpioData * g = (struct gpioData *)fiq->priv;
	memset(&regs,0,sizeof(regs));

	spin_lock_init(&fiq->lock);

	start = gpioGetPxaFiq(&length,&lockedLength);
	set_fiq_handler(start,lockedLength);
	regs.ARM_r9 = (int)((g) ? &g->g_dev_gpios[0]: 0);
	regs.ARM_r10 =(int)fiq->dcsr;
	regs.ARM_fp = (int)(&GPLR0);	//GPIO_BASE, GPLR0 is offset 0
	regs.ARM_sp = (int)(&fiq->stack[FIQ_STACK_SIZE]);

	set_fiq_regs(&regs);
	fiq->available = 1;
}

static int gpioFiqOp(void* ref, int relinquish)
{
	int ret=0;
	struct fiq_struct* fiq = ref;
	if (relinquish) {
		if (fiq->activeMask == 0) {
			fiq->available = 0;
		} else if (fiq->available) ret = -EBUSY;
	} else {
		gpioSetupFiq(fiq);
	}
	return ret;
}

static int gpioRequestFIQ( irqreturn_t (*callback)(int irq, void *dev_id),  struct gpioData * g)
{
	int ret=0;
	struct fiq_struct* fiq = &g->fiq;
	fiq->callback = callback;
	fiq->priv = g;

	if (fiq->available == 0) {
		fiq->fiq_st.dev_id = fiq;
		fiq->fiq_st.name = "pxa-gpio";
		fiq->fiq_st.fiq_op = gpioFiqOp;
		ret = claim_fiq(&fiq->fiq_st);
		if (ret==0) {
			//success
			gpioSetupFiq(fiq);
		}
	}
	return ret;
}
static void gpioReleaseFIQ(struct fiq_struct* fiq)
{
	if (fiq->dmaChannel >=0) {
		pxa_free_dma(fiq->dmaChannel);
		fiq->dmaChannel = -1;
	}
	fiq->available = 0;
	release_fiq(&fiq->fiq_st);
}


static void initGpioFiqState(struct gpioData * g, int gpio, int enableFiq)
{
	unsigned long flags;
	struct fiq_struct* fiq = &g->fiq;
	int irq = gpio+8;
	unsigned char activeMask;
	spin_lock_irqsave(&fiq->lock,flags);
	if (enableFiq)
		fiq->fiqMask[gpio>>5] |= (1<<(gpio&0x1f));
	else
		fiq->fiqMask[gpio>>5] &= ~(1<<(gpio&0x1f));

	activeMask = (fiq->fiqMask[0] & 3);
	if ( (fiq->fiqMask[0]>>2) || fiq->fiqMask[1] || fiq->fiqMask[2]) activeMask |= ~3;

	if (gpio>=2) {
		irq = 10;
		enableFiq = activeMask&4;
	}
	fiq->activeMask = activeMask;

	if (enableFiq) {
		if (fiq->dmaChannel==-1) {
			fiq->dmaChannel = pxa_request_dma("gpio fiq->irq conversion", DMA_PRIO_LOW,  gpio_dma_interrupt, g);
			if (fiq->dmaChannel<0) {
				fiq->dmaChannel = -2;
				fiq->dcsr = NULL;
				printk(KERN_ERR "gpio request dma failed");
			} else {
				fiq->dcsr = &DCSR0 + fiq->dmaChannel;
				*fiq->dcsr = DCSR_NODESC;
			}
		}
		if (fiq->dmaChannel>=0) gpioRequestFIQ(gpio_handler, g);	//try to get it back if lost
	}
//	local_irq_disable();	//this is not needed for xscale, the spin does it instead
	if (enableFiq)
		ICLR |= (1<<irq); //switch to fiq mode
	else
		ICLR &= ~(1<<irq); //switch back to irq mode
//	local_irq_enable();
	spin_unlock_irqrestore(&fiq->lock,flags);
}
#endif

#ifdef CONFIG_QUEUED_DATA
static void alloc_gp_queued_def(struct file *filp,unsigned eventCnt);
static int GetFlags(struct gpWorkArray* wa,int inputGp)
{
	struct gpWork* w = &wa->m_gpWork[0];
	int wcnt = wa->m_cnt;
	int flags = 0;
	while (wcnt) {
		flags |= w->sampleFlags;
		if (w->sampleGp!=inputGp) flags |= (0x100<<(w->sampleGp>>5));
		w++;
		wcnt--;
	}
	return flags;
}
static void SetEdgeAndRegList(struct dev_gpio * dev)
{
	int i;
	int gpio = dev->gpio_gpio;
	int edges = 0;
	int flags = 0;
	int regList = 0;
	for (i=0; i<MAX_OPEN_DEFS; i++) {
		struct gp_queued_def* q = dev->gpio_open_defs[i];
		if (!q) break;
		if (q!=NOT_IN_LIST) {
			struct gpWorkArray* w = q->m_gpWorkArray[gpio];
			flags |= GetFlags(w,gpio);
		}
	}
	edges = (flags | ((flags>>2) & IRQ_TYPE_EDGE_BOTH));
	edges &= (IRQ_TYPE_EDGE_BOTH|SAMPLE_FIQ);

	regList = flags >> 8;
	regList |= (flags&SAMPLE_TIMESTAMP);
	i = dev->gpio_edges ^ edges;
	if (i) {
		dev->gpio_edges = edges;
		if (i&IRQ_TYPE_EDGE_BOTH)
		set_irq_type(IRQ_GPIO(gpio), edges&IRQ_TYPE_EDGE_BOTH);
#ifdef CONFIG_ALLOW_FIQ
		if (i&SAMPLE_FIQ) {
			struct gpioData * g = g_gpioData;
			if (g) initGpioFiqState(g,gpio,edges&SAMPLE_FIQ);
		}
#endif
	}
	edges &= IRQ_TYPE_EDGE_BOTH;
	if (edges == IRQ_TYPE_EDGE_BOTH) regList |= (1<<(gpio>>5));
	dev->gpio_regMask = regList;

	regList |= (1<<(gpio>>5));
	dev->gpio_tgplr[0] = (regList & 1) ? GPLR(0) :0;
	dev->gpio_tgplr[1] = (regList & 2) ? GPLR(32) : 0;
	dev->gpio_tgplr[2] = (regList & 4) ? GPLR(64) : 0;
	dev->gpio_tgplr[3] = (regList & 8) ? GPLR(96) : 0;
//	if ((regList & (1<<(gpio>>5)) == 0) && (flags & SAMPLE_INPUT_RISING)) {
//		dev->gpio_tgplr[gpio>>5] |= GPIO_bit(gpio);
//	}
	if (regList & SAMPLE_TIMESTAMP) do_gettimeofday(&dev->gpio_timeStamp);
}

static void NoteQdef(struct gp_queued_def* qdef)
{
	struct gpioData * g = g_gpioData;
	int gpio;
	if (g) {
		for (gpio=0; gpio<MAX_GPIO; gpio++) {
			struct gpWorkArray* w = qdef->m_gpWorkArray[gpio];
			struct dev_gpio * dev = g->g_dev_gpios[gpio];
			if (w) {
				int flags = GetFlags(w,gpio);
				flags = (flags | (flags>>2)) & IRQ_TYPE_EDGE_BOTH;
				if (!dev) dev= GetAllocGpioDev(gpio,IN,1,flags);
			}
			if (dev) if (dev->gpio_direction != IN) dev = NULL;
			if (dev) {
				int i;
				int empty=MAX_OPEN_DEFS;
				int bInsert = (w)? 1 : 0;
				unsigned long flags;
				for (i=0; i<MAX_OPEN_DEFS; i++) {
					struct gp_queued_def* q = dev->gpio_open_defs[i];
					if (!q) {
						if (empty>i) empty=i;
						break;
					}
					if (q == qdef) {
						if (!bInsert) dev->gpio_open_defs[i] = q = NOT_IN_LIST;
						else bInsert = 0;
					}
					if ( q == NOT_IN_LIST) if (empty>i) empty = i;
				}
				if (bInsert || (empty!=i)) {
					struct gp_queued_def* q;
					spin_lock_irqsave(&dev->gpio_open_defs_lock, flags);
					if (bInsert) {
						empty--;
						while (empty >= 0) {
							q = dev->gpio_open_defs[empty];
							if ((q!=NULL)&&(q!=NOT_IN_LIST)) break;
							empty--;
						}
						while (empty<MAX_OPEN_DEFS-1) {
							empty++;
							q = dev->gpio_open_defs[empty];
							if ((q==NULL)||(q==NOT_IN_LIST)) {
								dev->gpio_open_defs[empty] = qdef;
								break;
							}
						}
					}
					while (i<MAX_OPEN_DEFS) {
						q = dev->gpio_open_defs[i];
						if (q==NULL) break;
						i++;
					}
					while (i>0) {
						i--;
						q = dev->gpio_open_defs[i];
						if (q==NOT_IN_LIST) dev->gpio_open_defs[i] = NULL;
						else if (q) break;
					}
					spin_unlock_irqrestore(&dev->gpio_open_defs_lock, flags);
				}
				SetEdgeAndRegList(dev);
			}
		}
	}
}
static int CntBits(unsigned char* p,int length)
{
	int cnt=0;
	int val;
	while (length) {
		val = *p++;
		while (val) { cnt++; val>>=1;}
		length--;
	}
	return cnt;
}
static void InsertEntry(struct gpWorkArray* wa,int gp,int sampleFlags)
{
	struct gpWork* w = &wa->m_gpWork[0];
	int wcnt = wa->m_cnt;
	while (wcnt) {
		if (w->sampleGp == gp) {w->sampleFlags = sampleFlags; return;}
		w++;
		wcnt--;
	}
	w->sampleGp = gp;
	w->sampleFlags = sampleFlags;
	wa->m_cnt++;
}

static void ProcessEntries(struct gp_queued_def* qdef, int gpio, struct gpConfigSample* s, unsigned cnt)
{
	struct gpConfigSample* s2 = s;
	unsigned i = cnt;
	struct gpWorkArray* gpWA = qdef->m_gpWorkArray[gpio];
	struct gpWorkArray* gpWANew = NULL;
//figure out how many entries are required
#define MAX_GP_MASK_BYTES	((MAX_GPIO+7)>>3)
	unsigned char gpMask[MAX_GP_MASK_BYTES];
	memset(gpMask,0,MAX_GP_MASK_BYTES);

	if (gpWA) {
		int wcnt = gpWA->m_cnt;
		struct gpWork* gpW = &gpWA->m_gpWork[0];

		while (wcnt) {
			unsigned gp = gpW->sampleGp;
			if (gp<MAX_GPIO) {
				if (gpW->sampleFlags) gpMask[gp>>3] |= 1<<(gp&7);
				else gpMask[gp>>3] &= ~(1<<(gp&7));
			}
			gpW++;
			wcnt--;
		}
	}

	while (i) {
		unsigned gp = s2->sampleGp;
		if (gpio==s2->inputGp) if (gp<MAX_GPIO) {
			if (s2->sampleFlags) gpMask[gp>>3] |= 1<<(gp&7);
			else gpMask[gp>>3] &= ~(1<<(gp&7));
		}
		s2++;
		i--;
	}
	i = CntBits(gpMask,MAX_GP_MASK_BYTES);
	if (i) {
		if (gpWA) if (gpWA->m_cnt==i) gpWANew = gpWA;
		if (!gpWANew) gpWANew = kmalloc(sizeof(struct gpWork)*i + sizeof(struct gpWorkArray),GFP_KERNEL);
		if (gpWANew) {
			int wcnt = 0;
			struct gpWork* gpW = NULL;

			if (gpWA) { wcnt = gpWA->m_cnt; gpW = &gpWA->m_gpWork[0];}
			gpWANew->m_cnt = 0;
			while (wcnt) {
				unsigned gp = gpW->sampleGp;
				if (gp<MAX_GPIO) {
					if (gpMask[gp>>3] & (1<<(gp&7))) {
						if (gpW->sampleFlags) InsertEntry(gpWANew,gp,gpW->sampleFlags);
					}
				}
				gpW++;
				wcnt--;
			}
			while (cnt) {
				unsigned gp = s->sampleGp;
				if (gpio==s->inputGp) if (gp<MAX_GPIO) {
					if (gpMask[gp>>3] & (1<<(gp&7))) {
						int flags = s->sampleFlags;
#define SAMPLE_ACTIVE_FLAGS (SAMPLE_INPUT_BOTH|SAMPLE_OUTPUT_BOTH)
						if (flags & SAMPLE_ACTIVE_FLAGS) {
							if (flags & SAMPLE_INPUT_BOTH) flags|=SAMPLE_OVERRUN;
							InsertEntry(gpWANew,gp,flags);	//mark as needing initial value
							qdef->overrun=1;
						}
					}
				}
				s++;
				cnt--;
			}
		}
	}
	if (gpWA != gpWANew) {
		qdef->m_gpWorkArray[gpio] = gpWANew;
		if (gpWA) kfree(gpWA);
	}
}
#endif

// This routine allows the driver to implement device-
// specific ioctl's.  If the ioctl number passed in cmd is
// not recognized by the driver, it should return ENOIOCTLCMD.

static int gpio_ioctl(struct inode *inode, struct file *filp,
			 unsigned int cmd, unsigned long arg)
{
	int return_val=-ENODEV;
	unsigned int gpio;
	struct dev_gpio* dev;
#ifdef CONFIG_QUEUED_DATA
	struct gp_queued_def* qdef = (struct gp_queued_def*)filp->private_data;
	if (qdef) {
		switch (cmd) {
			case GPIO_SET_CONFIG:
			{
				struct gpConfig cfg;
				if (copy_from_user(&cfg, (void *)arg, 8)) {return_val = -EFAULT; break;}
				return_val = 0;
				if (cfg.changeGpEventCnt) {
					if (cfg.changeGpEventCnt>MAX_GPEVENTS) {return_val = -EFAULT; break;}
					if (cfg.changeGpEventCnt != qdef->m_allocCnt) {
						alloc_gp_queued_def(filp,cfg.changeGpEventCnt);
						qdef = (struct gp_queued_def*)filp->private_data;
					}
				}
				if (cfg.sampleCnt) {
					struct gpConfigSample* s;
					int size;
					if (cfg.sampleCnt>200) {return_val = -EFAULT; break;}
					size = sizeof(struct gpConfigSample)*cfg.sampleCnt;
					s = kmalloc(size,GFP_KERNEL);
					if (!s) { return_val = -ENOMEM; break;}

					if (copy_from_user(s, (void *)(arg+8), size)==0) {
						unsigned i=cfg.sampleCnt;
						struct gpConfigSample* s2 = s;
						unsigned char gpMask[MAX_GP_MASK_BYTES];
						memset(gpMask,0,MAX_GP_MASK_BYTES);
						while (i) {
							int gpio = s2->inputGp;
							if (gpio<MAX_GPIO) {
								if ((gpMask[gpio>>3] & (1<<(gpio&7)))==0) {
									gpMask[gpio>>3] |= 1<<(gpio&7);
									ProcessEntries(qdef,gpio,s2,i);
								}
							}
							s2++;
							i--;
						}
						NoteQdef(qdef);
					} else return_val = -EFAULT;

					kfree(s);
				}
				break;
			}
			case FIONREAD:
			{
				int size;
				int cnt = qdef->m_insertIndex - qdef->m_removeIndex;
				if (cnt<0) cnt += qdef->m_allocCnt;
				if (qdef->overrun) cnt += OverrunCnt(qdef);
				size = cnt*sizeof(struct gpEvent);

				return_val = put_user( size, (unsigned long *)arg );
				break;
			}
			default:
				printk( KERN_ERR "unknown GPIO ioctl %u\n", cmd );
				return_val = -ENOIOCTLCMD;
		}
		return return_val;
	}
#endif
	gpio = MINOR (filp->f_dentry->d_inode->i_rdev);
	dev= GetAllocGpioDev(gpio,0,0,IRQ_TYPE_EDGE_BOTH);
	if (dev) {
		switch (cmd) {
#ifdef DEBOUNCE_LOGIC
			case GPIO_SET_DEBOUNCE_DELAY_LOW:		//arg is in milliseconds, convert to jiffies
			{
				int ms = 0 ;
				return_val = copy_from_user(&ms, (void *)arg, sizeof(ms));
				dev->gpio_debounce_low = (ms * HZ + 999)/1000; //amount of time low must be steady before valid
				return_val = 0;
				break;
			}
			case GPIO_SET_DEBOUNCE_DELAY_HIGH:
			{
				int ms = 0 ;
				return_val = copy_from_user(&ms, (void *)arg, sizeof(ms));
				dev->gpio_debounce_high = (ms * HZ + 999)/1000; //amount of time high must be steady before valid
				return_val = 0;
				break;
			}

			// get debounce settings
			case GPIO_GET_DEBOUNCE_DELAY_LOW:
			{
				unsigned long v = (dev->gpio_debounce_low * 1000)/HZ;
				return_val = put_user(v, (unsigned long *)arg);
				break;
			}
			case GPIO_GET_DEBOUNCE_DELAY_HIGH:
			{
				unsigned long v = (dev->gpio_debounce_high * 1000)/HZ;
				return_val = put_user(v, (unsigned long *)arg);
				break;
			}
#endif
			case FIONREAD:
			{
//				unsigned long const one = 1 ;
				return_val = put_user( 1UL, (unsigned long *)arg );
				break;
			}
			case TCGETS:	//get termios structure, ie tcgetattr(STDIN_FILENO,&oldTermState);
			{
				return_val = -ENOIOCTLCMD;
				break;
			}
			case GPIO_SET_INVERT:
			{
				return_val = copy_from_user(invert_bytes, (void *)arg, sizeof(invert_bytes));
				break;
			}
			case GPIO_GET_INVERT:
			{
				return_val = copy_to_user((void *)arg,invert_bytes, sizeof(invert_bytes));
				break;
			}
			case GPIO_PULSE:
			{
				__u32 param ;
				__u32 duration ;
				__u32 direction ;
				
				if( OUT != dev->gpio_direction ){
					printk( KERN_ERR "%s: pulse pin %u not output\n", __func__, gpio );
					return_val = -EFAULT ;
					break;
				}
                                if (copy_from_user(&param, (void *)arg, sizeof(param))) {return_val = -EFAULT; break;}
				duration = param & 0x7fffffff ;
				direction = param & 0x80000000 ;
				if(invert_bit(gpio))
					direction ^= 0x80000000 ;
				dev->pulse_timer.expires = jiffies + duration ;
				dev->pulse_timer.data = direction | gpio ;
                                add_timer (&dev->pulse_timer);

				if (direction) {
					GPSR(gpio) = GPIO_bit(gpio);
				} else {
					GPCR(gpio) = GPIO_bit(gpio);
				}

				return_val = 0 ;
				break;
			}
			default:
				printk( KERN_ERR "unknown GPIO ioctl %u\n", cmd );
				return_val = -ENOIOCTLCMD;
		}
	}
	return return_val;
}



#ifdef ASYNC_INTERFACE
static int gpio_fasync (int fd, struct file *filp, int on)
{
	unsigned int gpio = MINOR (filp->f_dentry->d_inode->i_rdev);
	struct dev_gpio* dev= GetAllocGpioDev(gpio,IN,1,IRQ_TYPE_EDGE_BOTH);
	if (dev) return fasync_helper (fd, filp, on, &dev->gpio_async_queue);
	else return -EINVAL;
}
#endif

#ifdef CONFIG_QUEUED_DATA
static void ChangeQdef(struct gp_queued_def* qdef,struct gp_queued_def* qdefNew)
{
	struct gpioData * g = g_gpioData;
	int gpio;
	for (gpio=0; gpio<MAX_GPIO; gpio++) {
		struct gpWorkArray* w = qdef->m_gpWorkArray[gpio];
		if (w) {
			qdef->m_gpWorkArray[gpio] = NULL;
			if (qdefNew == NOT_IN_LIST) kfree(w);	//ownership was not transferred
			if (g) {
				struct dev_gpio * dev = g->g_dev_gpios[gpio];
				if (dev) {
					int i;
					for (i=0; i<MAX_OPEN_DEFS; i++) {
						struct gp_queued_def* q = dev->gpio_open_defs[i];
						if (!q) break;
						if (q == qdef) {dev->gpio_open_defs[i] = qdefNew; break;}
					}
				}
			}
		}
	}
}
static void alloc_gp_queued_def(struct file *filp,unsigned eventCnt)
{
	struct gp_queued_def* qdef;
	struct gp_queued_def* qdefPrev = (struct gp_queued_def*)filp->private_data;
	int size;
	if (eventCnt > MAX_GPEVENTS) eventCnt = MAX_GPEVENTS;
	else if (eventCnt < 2) eventCnt = 2;

	if (qdefPrev) if (eventCnt == qdefPrev->m_allocCnt) return;

	size = sizeof(struct gp_queued_def) + (eventCnt*sizeof(struct gpEvent));
	qdef = kmalloc(size,GFP_KERNEL);
	if (qdef) {
		if (qdefPrev) {
			int cnt1 = qdefPrev->m_insertIndex - qdefPrev->m_removeIndex;
			int cnt2 = 0;
			memcpy(qdef,qdefPrev,sizeof(struct gp_queued_def));
			qdef->m_removeIndex = 0;
			if (cnt1<0) {
				cnt1 = qdef->m_allocCnt - qdefPrev->m_removeIndex;
				cnt2 = qdefPrev->m_insertIndex;
			}
			if (cnt1 > eventCnt) {cnt1 = eventCnt; cnt2 = 0;}
			if (cnt2 > eventCnt-cnt1) cnt2 = eventCnt-cnt1;
			if (cnt1) memcpy(&qdef->m_gpEvent[0],&qdefPrev->m_gpEvent[qdefPrev->m_removeIndex],cnt1 * sizeof(struct gpEvent));
			if (cnt2) memcpy(&qdef->m_gpEvent[cnt1],&qdefPrev->m_gpEvent[0],cnt2 * sizeof(struct gpEvent));
			qdef->m_insertIndex = cnt1+cnt2;
			qdef->m_allocCnt = eventCnt;
			init_waitqueue_head(&qdef->m_wait_queue);
			filp->private_data = qdef;
			ChangeQdef(qdefPrev,qdef);
			while (1) {
				if (waitqueue_active(&qdefPrev->m_wait_queue)) {
					wake_up(&qdefPrev->m_wait_queue);
                        	} else break;
				schedule();
			}
			kfree(qdefPrev);
		} else {
			memset(qdef, 0, size);
			qdef->m_allocCnt = eventCnt;
			init_waitqueue_head(&qdef->m_wait_queue);
			filp->private_data = qdef;
		}
	}
}
static int gpio_open (struct inode* inode, struct file *filp)
{
	unsigned int minor = MINOR (filp->f_dentry->d_inode->i_rdev);
	if (minor==255) {
		alloc_gp_queued_def(filp,DEFAULT_GPEVENTS);
	} else {
            unsigned int gpio = MINOR (filp->f_dentry->d_inode->i_rdev);
            struct dev_gpio* dev= GetAllocGpioDev(gpio,0,0,IRQ_TYPE_EDGE_BOTH);
            if (dev) {
               dev->gpio_transitions_read = dev->gpio_transitions-1 ; // force first read success
            }
        }
	return 0;
}
#endif

#if defined(ASYNC_INTERFACE) || defined(CONFIG_QUEUED_DATA)
static int gpio_release (struct inode* inode, struct file *filp)
{
        unsigned int minor = MINOR (filp->f_dentry->d_inode->i_rdev);
        struct dev_gpio* dev= GetAllocGpioDev(minor,0,0,IRQ_TYPE_EDGE_BOTH);
#ifdef CONFIG_QUEUED_DATA
	struct gp_queued_def* qdef = filp->private_data;
	if (qdef) {
		while (1) {
			if (waitqueue_active(&qdef->m_wait_queue)) {
				wake_up(&qdef->m_wait_queue);
                        } else break;
			schedule();
		}
		filp->private_data = 0;

		ChangeQdef(qdef,NOT_IN_LIST);
		kfree(qdef);
	}
#endif
	del_timer_sync(&dev->pulse_timer);

#ifdef ASYNC_INTERFACE
	gpio_fasync(-1,filp,0); //remove from the notify list
#endif
	return 0;
}
#endif


// This structure is the file operations structure, which specifies what
// callbacks functions the kernel should call when a user mode process
// attempts to perform these operations on the device.


static struct file_operations gpio_fops = {
	owner:		THIS_MODULE,
	read:		gpio_read,
	poll:		gpio_poll,
	write:		gpio_write,
	ioctl:		gpio_ioctl,
#ifdef ASYNC_INTERFACE
	fasync:		gpio_fasync,
#endif
#ifdef CONFIG_QUEUED_DATA
	open:		gpio_open,
#endif
#if defined(ASYNC_INTERFACE) || defined(CONFIG_QUEUED_DATA)
	release:	gpio_release
#endif
};


#ifdef INPUT_EVENT_INTERFACE
static int ie_gpio_open(struct input_dev *idev)
{
	struct gpioData *ts = (struct gpioData *)idev;

	return ie_gpio_startup(ts);
}
static void ie_gpio_close(struct input_dev *idev)
{
	struct gpioData *ts = (struct gpioData *)idev;

	down(&ts->sem);
	ie_gpio_shutdown(ts);
	up(&ts->sem);
}
static inline int ie_gpio_register(struct gpioData *ts)
{
	ts->g_idev.name      = "Touchscreen panel";
	ts->g_idev.idproduct = ts->ucb->id;
	ts->g_idev.open      = ie_gpio_open;
	ts->g_idev.close     = ie_gpio_close;

	__set_bit(EV_ABS, ts->g_idev.evbit);
	__set_bit(ABS_X, ts->g_idev.absbit);
	__set_bit(ABS_Y, ts->g_idev.absbit);
	__set_bit(ABS_PRESSURE, ts->g_idev.absbit);

	input_register_device(&ts->g_idev);

	return 0;
}
static inline void ie_gpio_deregister(struct gpioData *ts)
{
	input_unregister_device(&ts->g_idev);
}
	input_report_abs(&g->g_idev, ABS_X, x);
#endif




//#define GPIO_MAJOR 0 //0 means dynmaic assignment
#define GPIO_MAJOR 252 //0 means dynmaic assignment
int gpio_major = 0;

module_param(gpio_major, uint,0);
MODULE_PARM_DESC(gpio_major, "Choose major device number");

#endif

#if defined(CONFIG_MACH_ARGON)

static int const inputs[] = {
   24
,  25
};

static int const outputs[] = {
   12
,  13
,  22
,  26
,  27
};

static int const outputLevels[] = {
   0	// 12
,  0	// 13
,  0	// 22
,  0	// 26
,  0	// 27
};

#define HAVE_PINS
#elif defined(CONFIG_MACH_NEON)
#endif

#ifndef ARRAYSIZE
#define ARRAYSIZE(_arr) (sizeof(_arr)/sizeof(_arr[0]))
#endif

#define VERSION "0.2"		/* Driver version number */

#ifdef HAVE_PINS
static void initPins( void )
{
		int i ;
		for( i = 0 ; i < ARRAYSIZE(inputs); i++ ){
			int pinNum = inputs[i];
			u32 volatile *gafr = &_GAFR(pinNum);
			u32 volatile *gpdr = &_GPDR(pinNum);
			*gafr &= ~(3<<(2*(pinNum&15))); // Alternate function zero
			*gpdr &= ~(1<<(pinNum&31)); 	// input=0
		}
		for( i = 0 ; i < ARRAYSIZE(outputs); i++ ){
			int pinNum = outputs[i];
			u32 volatile *gafr = &_GAFR(pinNum);
			u32 volatile *gpdr = &_GPDR(pinNum);
			int const level = outputLevels[i];
			*gafr &= ~(3<<(2*(pinNum&15))); // Alternate function zero
			*gpdr |= (1<<(pinNum&31)); 	// output=1
			if( level ){
				_GPSR(pinNum) = (1<<(pinNum&31));
			} else {
				_GPCR(pinNum) = (1<<(pinNum&31));
			}
		}
}
#endif

// This function is called to initialise the driver, either from misc.c at
// bootup if the driver is compiled into the kernel, or from init_module
// below at module insert time. It attempts to register the device node.

static int __init gpio_init(void)
{
	struct gpioData * g = g_gpioData;

        memset(invert_bytes,0,sizeof(invert_bytes));
	if (!g) {
		g = (struct gpioData*) kmalloc(sizeof(struct gpioData),GFP_KERNEL);
		if (g) {
			memset(g,0,sizeof(struct gpioData));
#ifdef CONFIG_ALLOW_FIQ
			g->fiq.dmaChannel = -1;
#endif
			if (!g_gpioData) g_gpioData = g;
			else {
				kfree(g);
				g = g_gpioData;
			}
		}
	}
	if (g) {
                int result ;

#ifdef CALLBACK_INTERFACE
                spin_lock_init(&g->gpio_callbacklist_lock);
#endif
#ifdef DEBOUNCE_LOGIC
                spin_lock_init(&g->g_debounce_lock);
#endif

#ifdef HAVE_PINS
		initPins();
#endif

#ifdef DEVICE_INTERFACE
		result = register_chrdev(gpio_major,"gpio",&gpio_fops);
		if (result<0) {
			printk (KERN_WARNING __FILE__": Couldn't register device %d.\n", gpio_major);
			return result;
		}
		if (gpio_major==0) gpio_major = result; //dynamic assignment
#endif

                printk (KERN_INFO "GPIO signals driver version " VERSION " from Boundary Devices, 2002\n");
#ifdef DEBOUNCE_LOGIC
		init_timer (&g->g_timer);
		g->g_timer.function = CheckExpirationTimerCallback;
#endif
#ifdef INPUT_EVENT_INTERFACE
		return ie_gpio_register(g);
#endif
	}
	return 0;
}


static void __exit gpio_exit (void)
{
	unsigned gpio=0;
	struct gpioData * g = g_gpioData;
	if (g) {
                g_gpioData = NULL;
#ifdef INPUT_EVENT_INTERFACE
		ie_gpio_deregister(g);
#endif
		for (;gpio<MAX_GPIO;gpio++) {
			struct dev_gpio* dev= g->g_dev_gpios[gpio];
			if (dev) {
				g->g_dev_gpios[gpio] = NULL;
				if (dev->gpio_direction==IN) {
					free_irq(GPIO_TO_IRQ(gpio),dev);
#ifdef DEVICE_INTERFACE
					wake_up_interruptible (&dev->gpio_wait_queue);
#endif

#ifdef CALLBACK_INTERFACE
					{
						struct gpio_callback** pp = &dev->gpio_callback_list;
						struct gpio_callback* p;
						spin_lock(&dev->gpio_callbacklist_lock);
						p = *pp;
						while (p) {
							*pp = p->next;
							kfree(p);
							p = *pp;
						}
						spin_unlock(&dev->gpio_callbacklist_lock);
					}
#endif
				}
				kfree(dev);
			}
		}
#ifdef CONFIG_ALLOW_FIQ
		gpioReleaseFIQ(&g->fiq);
#endif
		kfree(g);
	}
#ifdef DEVICE_INTERFACE
	unregister_chrdev(gpio_major,"gpio");
#endif
}


MODULE_AUTHOR("Troy Kisky<troy.kisky@BoundaryDevices.com>");
MODULE_LICENSE("GPL");

module_init(gpio_init);
module_exit(gpio_exit);
