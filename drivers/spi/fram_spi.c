#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/mfp-pxa27x.h>
#include <mach/pxa2xx_spi.h>
#include <mach/gpio.h>
#include <linux/proc_fs.h>
#include <linux/spi/fram_spi.h>

#include <asm/uaccess.h>

#define CE_WAIT_MSEC 	12
#define DEVICE_NAME	"fram"
#define MAX_SIZE	65536 /*64 KB or 512 Kbits*/

static int fram_major = 0;

extern int pxa_gpio_mode(int);

static struct framdev_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	u8			*in_buffer;	
	u8			*out_buffer;
}g_fram_dev;

static unsigned bufsiz = 4096+3;

static int set_pxa_spi_gpio_config(void)
{
	int modes_num = 4, ret = 0, i = 0;
	int mode[] = {	NEON270_FRAM_CLOCK,
			NEON270_FRAM_CS,
			NEON270_FRAM_SI,
			NEON270_FRAM_SO };

	for (i = 0; i < modes_num; i++)
		if ((ret = pxa_gpio_mode(mode[i])) != 0)
			return ret;
	return 0;
}

static inline void fram_write_enable(struct framdev_data *dev_data)
{
	int ret = 0;
	u8 in[2] = { 0 };
	in[0] = FRAM_WREN;
	ret = spi_write(dev_data->spi,in,1);
}

static inline u8 fram_read_status(struct framdev_data *dev_data)
{
	int ret = 0;
	u8 in[2] = { 0 };
	u8 out[2] = { 0 };

	in[0] = FRAM_RDSR;
	ret = spi_write_then_read(dev_data->spi, in, 1, out, 1);
	return out[0];
}

static inline void fram_write_status(struct framdev_data *dev_data,
					int enable_wpen,
					int block_protect)
{
	int ret = 0;
	u8 status_reg = 0x40;
	u8 in[4] = { 0 };

	status_reg |= (enable_wpen << WPEN);
	status_reg |= block_protect;

	in[0] = FRAM_WRSR; in[1] = status_reg;
	ret = spi_write(dev_data->spi,in,2);
}

static inline ssize_t fram_write_data(struct framdev_data *dev_data,
					u32 offset,
					size_t count)
{
	int ret = count;

	dev_data->out_buffer[0] = FRAM_WRITE;
	dev_data->out_buffer[1] = ((offset & 0x0000FF00) >> 8);
	dev_data->out_buffer[2] = (offset & 0x000000FF);

	/*
	 * check if offset + count is within the FRAM boundary
	 * if not modify the count value to adher to boundary
	 * restriction
	 */
	if((offset + ret) > MAX_SIZE)
		ret = (MAX_SIZE - offset);

	if(ret > 0) {
		if(spi_write(dev_data->spi, dev_data->out_buffer, ret+3) < 0) {
			printk(KERN_ERR "Problem writing data to fram\n");
			ret = 0;
		}
	}
	else
		ret = 0;

	return ret;
}

static inline ssize_t fram_read_data(struct framdev_data *dev_data,
					u32 offset,
					size_t count,
					int *read_offset)
{
#define READ_OFFSET	3
	DECLARE_COMPLETION_ONSTACK(done);
	int ret = count;
	/* len = rx + tx. here tx = 3 but during transmission it
	 * also receives data. the first 3 bytes would be undefined
	 * we need to accout for these too. so we define length as
	 * len = rx + 2 * tx
	 */
	struct spi_transfer t = {
			.tx_buf = dev_data->out_buffer,
			.rx_buf = dev_data->in_buffer,
			.len = ((2 * READ_OFFSET) + count)
		};
	struct spi_message m;
	int status;
	
	*read_offset = READ_OFFSET;

	dev_data->out_buffer[0] = FRAM_READ;
	dev_data->out_buffer[1] = ((offset & 0x0000FF00) >> 8);
	dev_data->out_buffer[2] = (offset & 0x000000FF);

	/*
	 * check if offset + count is within the FRAM boundary
	 * if not modify the count value to adher to boundary
	 * restriction
	 */
	if((offset + ret) > MAX_SIZE)
		ret = (MAX_SIZE - offset);

	if(ret > 0) {
		t.len = (2 * READ_OFFSET) + ret;

		spi_message_init(&m);
		spi_message_add_tail(&t, &m);

		if((status = spi_sync(dev_data->spi, &m)) < 0) {
			ret = status;
			printk("Unable to read data. Errno: %d\n", ret);
		}
	}
	else
		ret = 0;

	return ret;
}

static ssize_t
fram_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct framdev_data *dev_data;
	ssize_t status = 0;
	int read_offset;

	if(count > bufsiz)
		return -EMSGSIZE;

	dev_data = filp->private_data;

	status = fram_read_data(dev_data, (u32)(*f_pos), count, &read_offset);

	if(status > 0) {
		char *read_loc = dev_data->in_buffer + read_offset;
		unsigned long missing_bytes = copy_to_user(buf, read_loc, count);
		*f_pos += status;
		if(missing_bytes == status)
			status = -EFAULT;
		else
			status = status - missing_bytes;
	}
	return status;
}

static ssize_t
fram_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct framdev_data *dev_data;
	ssize_t status = 0;
	unsigned long missing_bytes;

	if(count > bufsiz)
		return -EMSGSIZE;

	dev_data = filp->private_data;

	missing_bytes = copy_from_user(dev_data->out_buffer+3, buf, count);

	if(missing_bytes == 0) {
		/* write enable */
		fram_write_enable(dev_data);
		status = fram_write_data(dev_data, (u32)(*f_pos), count); /* written count characters */
		*f_pos += status;
	}
	else
		status = -EFAULT;

	return status;
}

static int fram_open(struct inode *inode, struct file *filp)
{
	int status = 0;
	if(!g_fram_dev.in_buffer) {
		g_fram_dev.in_buffer = kmalloc(bufsiz, GFP_KERNEL);
		if(!g_fram_dev.in_buffer) {
			printk("%s: ENOMEM\n", __func__);
			status = -ENOMEM;
		}
	}
	if(!g_fram_dev.out_buffer) {
		g_fram_dev.out_buffer = kmalloc(bufsiz, GFP_KERNEL);
		if(!g_fram_dev.out_buffer) {
			printk("%s: ENOMEM\n", __func__);
			status = -ENOMEM;
		}
	}
	if(status == 0) {
		filp->private_data = &g_fram_dev;
	}
	return status;
}

static int fram_release(struct inode *inode, struct file *filp)
{
	kfree(g_fram_dev.in_buffer);
	g_fram_dev.in_buffer = NULL;
	kfree(g_fram_dev.out_buffer);
	g_fram_dev.out_buffer = NULL;
	return 0;
}

static loff_t fram_llseek(struct file *file, loff_t offset, int origin)
{
	loff_t new_offset = 0;
	switch(origin) {
		case SEEK_SET:
			new_offset = offset;
			break;
		case SEEK_CUR:
			new_offset = file->f_pos + offset;
			break;
		case SEEK_END:
			new_offset = (MAX_SIZE - 1) + offset;
			break;
		default:
			return -EINVAL;
	}

	if((new_offset < 0) || (new_offset >= MAX_SIZE)) {
		return -EINVAL;
	}

	file->f_pos = new_offset;
	return new_offset;
}

static struct file_operations fram_fops = {
	.owner =	THIS_MODULE,
	.llseek =	fram_llseek,
	.write =	fram_write,
	.read =		fram_read,
	.open =		fram_open,
	.release =	fram_release,
};

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *framdev_class;

static int __devinit fram_probe(struct spi_device *spi)
{
	int ret = 0;
	unsigned long minor = 0;
	u8 status_reg;
	struct device *dev;

	/* Setup spi */
	ret = spi_setup(spi);

	mdelay(CE_WAIT_MSEC);

	GPDR(0) |= ((1 << NEON270_FRAM_CS) | (1 << NEON270_FRAM_CLOCK) | (1 << NEON270_FRAM_SI));
	GPDR(0) &= ~(1 << NEON270_FRAM_SO);
	GAFR0_U |= (0x02 << ((NEON270_FRAM_CLOCK & 0xf) << 1)) |
			(0x02 << ((NEON270_FRAM_CS & 0xf) << 1)) |
			(0x02 << ((NEON270_FRAM_SI & 0xf) << 1)) |
			(0x01 << ((NEON270_FRAM_SO & 0xf) << 1));

	g_fram_dev.devt = MKDEV(fram_major, minor);
	g_fram_dev.spi = spi;
	dev = device_create(framdev_class, &spi->dev, g_fram_dev.devt,
				&g_fram_dev, "fram");

	if(IS_ERR(dev))
		return PTR_ERR(dev);

	status_reg = fram_read_status(&g_fram_dev);
	printk(KERN_INFO "\n%s: Status Register Value\t\t\t0x%x\n", __func__, status_reg);
	fram_write_enable(&g_fram_dev);
	fram_write_status(&g_fram_dev, 0, BP_NONE);
	status_reg = fram_read_status(&g_fram_dev);
	printk(KERN_INFO "\n%s: Status Register Value\t\t\t0x%x\n", __func__, status_reg);

	spi_set_drvdata(spi, &g_fram_dev);

	return 0;
}

static int __devexit fram_remove(struct spi_device *spi)
{
	struct framdev_data	*dev_data = spi_get_drvdata(spi);
	dev_data->spi = NULL;
	spi_set_drvdata(spi, NULL);
	device_destroy(framdev_class, dev_data->devt);
	return 0;
}

static struct spi_driver fram_driver = {
	.driver = {
		.name	= "fram",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= fram_probe,
	.remove		= __devexit_p(fram_remove),
};

static int __init fram_init(void)
{
	int ret = set_pxa_spi_gpio_config();
	if(ret < 0)
		return ret;
	
	ret = register_chrdev(0, DEVICE_NAME, &fram_fops);
	if (ret < 0)
		return ret;
	fram_major = ret;
	printk(KERN_INFO "Successfully registered a %d as the major number for fram\n", fram_major);

	framdev_class = class_create(THIS_MODULE, "fram");
	if (IS_ERR(framdev_class)) {
		unregister_chrdev(fram_major, DEVICE_NAME);
		return PTR_ERR(framdev_class);
	}

	ret = spi_register_driver(&fram_driver);
	if(ret < 0) {
		class_destroy(framdev_class);
		unregister_chrdev(fram_major, DEVICE_NAME);
	}
	return ret;
}
module_init(fram_init);

static void __exit fram_exit(void)
{
	spi_unregister_driver(&fram_driver);
	unregister_chrdev(fram_major, DEVICE_NAME);
}
module_exit(fram_exit);

MODULE_DESCRIPTION("RAMTRON FM25L512 FRAM Driver");
MODULE_LICENSE("GPL");

