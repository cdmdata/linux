/*
 * MX51 2XX GPIO driver
 *
 * Copyright (C) 2008, Boundary Devices
 *
 */
#include <linux/module.h>
#include <linux/init.h>

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/console.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/gpio-trig.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/mach/irq.h>
#include <linux/proc_fs.h>

#define GPIO_MAJOR 0 //0 means dynmaic assignment
static int gpio_major = GPIO_MAJOR ;

static char const driverName[] = {
	"gpio"
};

struct gpio_data {
	unsigned		trigger_pin ;
        wait_queue_head_t 	wait_queue ; // Used for blocking read, poll
	unsigned char		take ;
	unsigned char		add ;
	unsigned char		data[256];
};

static int total_ints = 0 ;

static ssize_t gpio_write(struct file * file, const char __user * buf,
		        size_t count, loff_t *ppos)
{
	printk( KERN_ERR "%s\n", __func__ );
	return count ;
}

/* Status readback conforming to ieee1284 */
static ssize_t gpio_read(struct file * file, char __user * buf,
		       size_t count, loff_t *ppos)
{
	struct gpio_data *data = (struct gpio_data *)file->private_data ;
	if( data ){
		ssize_t rval = 0 ;
		do {
			while( (0 < count) && (data->add != data->take) ){
				unsigned char left = (data->add-data->take);
				if( left > count ){
					left = count ;
				}
				if( copy_to_user(buf,data->data+data->take,1) )
					return -EFAULT ;
				data->take++ ;
				count-- ;
				buf++ ;
				rval++ ;
			}
			if( (0 == rval) && (0==(file->f_flags & O_NONBLOCK)) ){
                                if( wait_event_interruptible(data->wait_queue, data->add != data->take) ){
					break;
				}
			} // wait for first
			else
				break;
		} while( 1 );

		return (0 == rval) ? -EINTR : rval ;
	}
	else
		return -EFAULT ;
}

static unsigned int gpio_poll(struct file *filp, poll_table *wait)
{
	struct gpio_data *data = (struct gpio_data *)filp->private_data ;
	if( data ){
		poll_wait(filp, &data->wait_queue, wait);
		return (data->add != data->take) ? POLLIN | POLLRDNORM : 0 ;
	}
	else
		return -EINVAL ;
}

static void sample_data (struct gpio_data *data) {
	data->data[data->add++] = gpio_get_value(data->trigger_pin);
	wake_up(&data->wait_queue);
}

static irqreturn_t int_handler( int irq, void *param)
{
        ++total_ints ;
	if( param && 1 ){
		struct file *file = (struct file *)param ;
                struct gpio_data *data = (struct gpio_data *)file->private_data ;
                sample_data (data);
	}
	return IRQ_HANDLED ;
}

static int gpio_open(struct inode * inode, struct file * file)
{
        unsigned int gpio ;
	struct gpio_data *data = kmalloc(sizeof(struct gpio_data),GFP_KERNEL);
        file->private_data = data ;
	memset(file->private_data,0,sizeof(*data));
	init_waitqueue_head(&data->wait_queue);
        gpio = MINOR (file->f_dentry->d_inode->i_rdev);
	if (0 != gpio) {
		set_irq_type(gpio_to_irq(gpio), IRQ_TYPE_EDGE_BOTH);
		printk (KERN_ERR "%s: requesting %u, irq %d\n", __func__, gpio, gpio_to_irq(gpio));
		if( 0 == request_irq(gpio_to_irq(gpio), int_handler, IRQF_DISABLED, driverName, file) ){
			data->trigger_pin = gpio ;
			sample_data (data);
		}
		else {
			printk( KERN_ERR "Error grabbing interrupt for GPIO %u\n", gpio );
		}
	}
	return 0 ;
}

static int gpio_release(struct inode * inode, struct file * file)
{
        if(file->private_data){
		struct gpio_data *data = (struct gpio_data *)file->private_data ;
		printk (KERN_ERR "%s: releasing %u, irq %d\n", __func__, data->trigger_pin, gpio_to_irq(data->trigger_pin));
		free_irq(gpio_to_irq(data->trigger_pin), file);
		kfree(file->private_data);
                file->private_data = 0 ;
	}
	return 0 ;
}

static int gpio_ioctl(struct inode *inode, struct file *file,
		    unsigned int cmd, unsigned long arg)
{
	switch (cmd){
		case GPIO_TRIG_CFG:{
			struct gpio_data *data = (struct gpio_data *)file->private_data ;
			unsigned new_pin ;
			if( 0 == copy_from_user(&new_pin, (void *)arg, sizeof(new_pin)) ){
				if( data->trigger_pin ){
					printk (KERN_ERR "%s: releasing %u, irq %d\n", __func__, data->trigger_pin, gpio_to_irq(data->trigger_pin));
					free_irq(gpio_to_irq(data->trigger_pin), file);
					data->trigger_pin = 0 ;
				} // trigger previously set

				set_irq_type(gpio_to_irq(new_pin), IRQ_TYPE_EDGE_BOTH);
				printk (KERN_ERR "%s: requesting %u, irq %d\n", __func__, new_pin, gpio_to_irq(new_pin));
			        if( 0 == request_irq(gpio_to_irq(new_pin), int_handler, IRQF_DISABLED, driverName, file) ){
					data->trigger_pin = new_pin ;
					return 0 ;
				}
				else {
					printk( KERN_ERR "Error grabbing interrupt for GPIO %u\n", new_pin );
				}
			}

			break;
		}
		default:
			printk( KERN_ERR "%s: Unknown cmd 0x%x\n", __func__, cmd );
	}
	return -EFAULT ;
}

static const struct file_operations gpio_fops = {
	.owner		= THIS_MODULE,
	.write		= gpio_write,
	.ioctl		= gpio_ioctl,
	.open		= gpio_open,
	.release	= gpio_release,
	.read		= gpio_read,
	.poll		= gpio_poll,
};

#ifndef MODULE
static int __init gpio_setup (char *str)
{
	return 1;
}
#endif

static int __init gpio_init_module (void)
{
	int result ;
        struct proc_dir_entry *pde ;

	result = register_chrdev(gpio_major,driverName,&gpio_fops);
	if (result<0) {
		printk (KERN_WARNING __FILE__": Couldn't register device %d.\n", gpio_major);
		return result;
	}
	if (gpio_major==0)
		gpio_major = result; //dynamic assignment

	printk (KERN_INFO "MX51 GPIO driver. Boundary Devices\n");
	return 0 ;
}

static void gpio_cleanup_module (void)
{
        remove_proc_entry("triggers", 0 );
	unregister_chrdev(gpio_major,driverName);
}

__setup("pxagpio=", gpio_setup);
module_init(gpio_init_module);
module_exit(gpio_cleanup_module);

MODULE_ALIAS_CHARDEV_MAJOR(LP_MAJOR);
MODULE_LICENSE("GPL");