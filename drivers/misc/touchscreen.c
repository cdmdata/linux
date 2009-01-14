#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>

#include <sound/core.h>
#include <sound/ac97_codec.h>

MODULE_LICENSE("Dual BSD/GPL");

static int ts_irq = -1;
module_param(ts_irq, int, S_IRUGO);
MODULE_PARM_DESC(ts_irq, "Touchscreen driver IRQ number.");
static int ts_minor = 14;
module_param(ts_minor, int, S_IRUGO);
MODULE_PARM_DESC(ts_minor, "Touchscreen driver minor number.");

/*
 * Interesting UCB1400 AC-link registers
 */

#define UCB_IO_DATA		0x5a
#define UCB_IO_DIR		0x5c
#define UCB_IE_RIS		0x5e
#define UCB_IE_FAL		0x60
#define UCB_IE_STATUS		0x62
#define UCB_IE_CLEAR		0x62
#define UCB_IE_ADC		(1 << 11)
#define UCB_IE_TSPX		(1 << 12)
#define UCB_IE_TSMX		(1 << 13)

#define UCB_TS_CR		0x64
#define UCB_TS_CR_TSMX_POW	(1 << 0)
#define UCB_TS_CR_TSPX_POW	(1 << 1)
#define UCB_TS_CR_TSMY_POW	(1 << 2)
#define UCB_TS_CR_TSPY_POW	(1 << 3)
#define UCB_TS_CR_TSMX_GND	(1 << 4)
#define UCB_TS_CR_TSPX_GND	(1 << 5)
#define UCB_TS_CR_TSMY_GND	(1 << 6)
#define UCB_TS_CR_TSPY_GND	(1 << 7)
#define UCB_TS_CR_MODE_INT	(0 << 8)
#define UCB_TS_CR_MODE_PRES	(1 << 8)
#define UCB_TS_CR_MODE_POS	(2 << 8)
#define UCB_TS_CR_HYST_DIS	(1 << 10)
#define UCB_TS_CR_BIAS_ENA	(1 << 11)
#define UCB_TS_CR_TSPX_LOW	(1 << 12)
#define UCB_TS_CR_TSMX_LOW	(1 << 13)

#define UCB_ADC_CR		0x66
#define UCB_ADC_SYNC_ENA	(1 << 0)
#define UCB_ADC_VREFBYP_CON	(1 << 1)
#define UCB_ADC_INP_TSPX	(0 << 2)
#define UCB_ADC_INP_TSMX	(1 << 2)
#define UCB_ADC_INP_TSPY	(2 << 2)
#define UCB_ADC_INP_TSMY	(3 << 2)
#define UCB_ADC_INP_AD0		(4 << 2)
#define UCB_ADC_INP_AD1		(5 << 2)
#define UCB_ADC_INP_AD2		(6 << 2)
#define UCB_ADC_INP_AD3		(7 << 2)
#define UCB_ADC_EXT_REF		(1 << 5)
#define UCB_ADC_START		(1 << 7)
#define UCB_ADC_ENA		(1 << 15)

#define UCB_ADC_DATA		0x68
#define UCB_ADC_DAT_VALID	(1 << 15)
#define UCB_MAX_AD		0x3ff
#define UCB_ADC_DAT_VALUE(x)	((x) & UCB_MAX_AD)

#define UCB_ID			0x7e
#define UCB_ID_1400             0x4304

#define TSTYPE_PROCNAME "tstype"

/*
 * using io0 of UCB1400 as the 5th wire
 */
#define MASK_TSX_5 (1<<0)

#define TOUCH_NOT_VALID -1
#define TOUCH_4W 0
#define TOUCH_5W 1

static char const * const touch_type_names[] = {
	"Unknown",
	"4-wire resistive",
	"5-wire resistive"
};

static int touch_type = TOUCH_NOT_VALID ;

struct ts_event {
	unsigned short		pressure;
	unsigned short		x;
	unsigned short		y;
	unsigned short		pad;
	struct timeval	stamp;
};

/* This value has to be a power of 2 for the eventq to work properly */
#define MAX_EVENTS	32

struct ucb1400 {
	struct snd_ac97		*ac97;

	struct miscdevice       miscdev;

	int			irq;

	unsigned short		eventq_head; /* holds the head index */
	unsigned short		eventq_tail; /* holds the tail index */
	/*
	The following methods are used to manipulate the event
	queue.
		- eventq_push   : Pushes an event object onto the tail of the
				queue and is responsible for updating the tail
				of the queue.
		- eventq_pop    : Pops an event object from the head of the
				queue and is responsible for updating the head.
		- eventq_clear  : Deletes all the elements in the queue and
				resets the head and tail indicies.
		- eventq_isempty: Returns true if queue is empty.
	 */
	struct ts_event		event_queue[MAX_EVENTS];
	wait_queue_head_t	sample_wait;
	wait_queue_head_t	read_wait;
	struct task_struct	*ts_task;
	struct proc_dir_entry   *pde ;

	unsigned int		irq_pending;	/* not bit field shared */
	int			touch_type ;
	unsigned int		num_pressed ;
	unsigned int		ts_restart:1;
	unsigned int		adcsync:1;
};

struct ucb1400 *gucb;

#define INC_IDX(idx) (idx+1)&(MAX_EVENTS-1)

/* event queue method definitions */
static inline void eventq_clear(struct ucb1400 *ucb)
{
	ucb->eventq_head = ucb->eventq_tail = 0;
}

static inline bool eventq_isempty(struct ucb1400 *ucb)
{
	return (ucb->eventq_head == ucb->eventq_tail);
}

static inline void eventq_push(struct ucb1400 *ucb, struct ts_event *elem)
{
	unsigned short next_idx = INC_IDX(ucb->eventq_tail);
	ucb->event_queue[ucb->eventq_tail] = *elem;
	if (next_idx != ucb->eventq_head)
		ucb->eventq_tail = next_idx;
}

static inline bool eventq_pop(struct ucb1400 *ucb, struct ts_event *elem)
{
	bool empty_list = eventq_isempty(ucb);
	if (!empty_list) {
		(*elem) = ucb->event_queue[ucb->eventq_head];
		ucb->eventq_head = INC_IDX(ucb->eventq_head);
	}
	return empty_list;
}

static inline void add_event(struct ucb1400 *ucb, unsigned short x,
				unsigned short y, unsigned pressure)
{
	struct ts_event elem;
	elem.pressure = pressure;
	elem.x = x;
	elem.y = y;
	elem.pad = 0;
	do_gettimeofday(&elem.stamp);
	eventq_push(ucb, &elem);
}

static int adcsync;
static int ts_delay = 55; /* us */
static int ts_delay_pressure;	/* us */
static unsigned x5_bits = UCB_TS_CR_TSPX_POW
			 |UCB_TS_CR_TSPY_GND
			 |UCB_TS_CR_TSMY_POW ;
static unsigned y5_bits = UCB_TS_CR_TSPX_POW
			 |UCB_TS_CR_TSPY_POW
			 |UCB_TS_CR_TSMY_GND ;

static inline u16 ucb1400_reg_read(struct ucb1400 *ucb, u16 reg)
{
	return ucb->ac97->bus->ops->read(ucb->ac97, reg);
}

static inline void ucb1400_reg_write(struct ucb1400 *ucb, u16 reg, u16 val)
{
	ucb->ac97->bus->ops->write(ucb->ac97, reg, val);
}

static inline void ucb1400_adc_enable(struct ucb1400 *ucb)
{
	ucb1400_reg_write(ucb, UCB_ADC_CR, UCB_ADC_ENA);
}

static inline void ucb1400_ts_irq_disable(struct ucb1400 *ucb)
{
	ucb1400_reg_write(ucb, UCB_IE_FAL, 0);
}

static void ucb1400_handle_pending_irq(struct ucb1400 *ucb)
{
	unsigned int isr;

	isr = ucb1400_reg_read(ucb, UCB_IE_STATUS);
	ucb1400_reg_write(ucb, UCB_IE_CLEAR, isr);
	ucb1400_reg_write(ucb, UCB_IE_CLEAR, 0);

	if (ucb->touch_type == TOUCH_4W) {
		if (isr & UCB_IE_TSPX)
			ucb1400_ts_irq_disable(ucb);
	} else {
		if (isr & UCB_IE_TSMX)
			ucb1400_ts_irq_disable(ucb);
	}

	enable_irq(ucb->irq);
}

static unsigned int ucb1400_adc_read(struct ucb1400 *ucb, u16 adc_channel)
{
	unsigned int val;

	if (ucb->adcsync)
		adc_channel |= UCB_ADC_SYNC_ENA;

	ucb1400_reg_write(ucb, UCB_ADC_CR, UCB_ADC_ENA | adc_channel);
	ucb1400_reg_write(ucb, UCB_ADC_CR, UCB_ADC_ENA |
						adc_channel |
						UCB_ADC_START);

	for (;;) {
		val = ucb1400_reg_read(ucb, UCB_ADC_DATA);
		if (val & UCB_ADC_DAT_VALID)
			break;
		/* yield to other processes */
		schedule_timeout_uninterruptible(1);
	}

	return UCB_ADC_DAT_VALUE(val);
}

static inline void ucb1400_adc_disable(struct ucb1400 *ucb)
{
	ucb1400_reg_write(ucb, UCB_ADC_CR, 0);
}

static void ucb1x00_io_write(struct ucb1400 *ucb, unsigned int set,
				unsigned int clear)
{
	u16 prev = ucb1400_reg_read(ucb, UCB_IO_DATA);
	u16 curr = (prev | set) & ~clear;
	if (prev != curr)
		ucb1400_reg_write(ucb, UCB_IO_DATA, curr);
}

static void ucb1x00_io_set_dir(struct ucb1400 *ucb, unsigned int in,
				unsigned int out)
{
	unsigned int prev = ucb1400_reg_read(ucb, UCB_IO_DIR);
	unsigned int curr = (prev | out) & ~in;
	if (prev != curr)
		ucb1400_reg_write(ucb, UCB_IO_DIR, curr);
}

static inline unsigned int ucb1400_ts_read_xres(struct ucb1400 *ucb)
{
	ucb1400_reg_write(ucb, UCB_TS_CR,
			UCB_TS_CR_TSMX_GND | UCB_TS_CR_TSPX_POW |
			UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);
	return ucb1400_adc_read(ucb, 0);
}

static inline unsigned int ucb1400_ts_read_yres(struct ucb1400 *ucb)
{
	ucb1400_reg_write(ucb, UCB_TS_CR,
			UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_POW |
			UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);
	return ucb1400_adc_read(ucb, 0);
}

static inline int ucb1400_ts_pen_down(struct ucb1400 *ucb)
{
	unsigned short val = ucb1400_reg_read(ucb, UCB_TS_CR);
	if (TOUCH_4W == ucb->touch_type)
		return val & (UCB_TS_CR_TSPX_LOW|UCB_TS_CR_TSMX_LOW);
	else
		return (0 != (val & UCB_TS_CR_TSMX_LOW));
}

static inline void ucb1400_ts_irq_enable(struct ucb1400 *ucb)
{
	ucb1400_reg_write(ucb, UCB_IE_CLEAR, UCB_IE_TSPX);
	ucb1400_reg_write(ucb, UCB_IE_CLEAR, 0);
	ucb1400_reg_write(ucb, UCB_IE_FAL, UCB_IE_TSPX);
}

static inline void ucb1400_ts_irq_enable5(struct ucb1400 *ucb)
{
	ucb1400_reg_write(ucb, UCB_IE_CLEAR, UCB_IE_TSMX);
	ucb1400_reg_write(ucb, UCB_IE_CLEAR, 0);
	ucb1400_reg_write(ucb, UCB_IE_FAL, UCB_IE_TSMX);
}

int discover_touch_type(struct ucb1400 *ucb)
{
	int type = TOUCH_NOT_VALID;
	int tcr ;

	printk(KERN_ALERT "Inside discover_touch_type function\n");
	ucb1400_reg_write(ucb, UCB_TS_CR, UCB_TS_CR_TSPX_POW);
	ucb1x00_io_write(ucb, 0, MASK_TSX_5); /*clear bit*/
	ucb1x00_io_set_dir(ucb, 0, MASK_TSX_5);	/*set as output low*/
	msleep(2);
	tcr = ucb1400_reg_read(ucb, UCB_TS_CR);
	ucb1x00_io_set_dir(ucb, MASK_TSX_5, 0);	/*set as input*/
	if (tcr & (1<<12)) {
		/*TSPX is still high, so TSMXX is NOT involved in touch
		 * screen or NO cable plugged in*/
		ucb1400_reg_write(ucb, UCB_TS_CR, UCB_TS_CR_TSPX_POW |
							UCB_TS_CR_TSMX_GND);
		msleep(2);
		tcr = ucb1400_reg_read(ucb, UCB_TS_CR);
		if (tcr & (1<<12)) {
			/*TSPX is still high, NO cable plugged in*/
			printk(KERN_ALERT "TSPX is still high,\
						NO cable plugged in.\n");
		} else
			type = TOUCH_4W;
	} else {
		/*TSPX is LOW, so TSMXX is involved in touch screen*/
		ucb1400_reg_write(ucb, UCB_TS_CR, UCB_TS_CR_TSPX_POW |
							UCB_TS_CR_TSMY_GND);
		msleep(2);
		tcr = ucb1400_reg_read(ucb, UCB_TS_CR);
		if (tcr & (1<<12)) {
			/*TSPX is still high, Independent X and Y*/
			type = TOUCH_4W;
		} else {
			/*TSPX is low, X plane connected to Y plane*/
			type = TOUCH_5W;
		}
	}
	ucb1400_reg_write(ucb, UCB_TS_CR, 0);
	if (TOUCH_5W == type) {
		ucb1x00_io_set_dir(ucb, 0, 1);
		ucb1x00_io_write(ucb, 0, MASK_TSX_5);	/*set as output low*/
	}

	return type;
}

/* Switch to interrupt mode. */
static inline void ucb1400_ts_mode_int(struct ucb1400 *ucb)
{
	if (TOUCH_4W == ucb->touch_type) {
		ucb1400_reg_write(ucb, UCB_TS_CR,
					UCB_TS_CR_TSMX_POW |
					UCB_TS_CR_TSPX_POW |
					UCB_TS_CR_TSMY_GND |
					UCB_TS_CR_TSPY_GND |
					UCB_TS_CR_MODE_INT);
	} else {
		ucb1400_reg_write(ucb, UCB_TS_CR,
					UCB_TS_CR_TSPX_GND |
					UCB_TS_CR_TSMY_GND |
					UCB_TS_CR_TSPY_GND |
					UCB_TS_CR_TSMX_POW |
					UCB_TS_CR_MODE_INT);
	}
}

/*
 * Switch to pressure mode, and read pressure.  We don't need to wait
 * here, since both plates are being driven.
 */
static inline unsigned int ucb1400_ts_read_pressure(struct ucb1400 *ucb)
{
	ucb1400_reg_write(ucb, UCB_TS_CR,
			UCB_TS_CR_TSMX_POW | UCB_TS_CR_TSPX_POW |
			UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_GND |
			UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);
	udelay(ts_delay_pressure);
	return ucb1400_adc_read(ucb, UCB_ADC_INP_TSPY);
}

/*
 * Switch to X position mode and measure Y plate.  We switch the plate
 * configuration in pressure mode, then switch to position mode.  This
 * gives a faster response time.  Even so, we need to wait about 55us
 * for things to stabilise.
 */
static inline unsigned int ucb1400_ts_read_xpos(struct ucb1400 *ucb)
{
	ucb1400_reg_write(ucb, UCB_TS_CR,
			UCB_TS_CR_TSMX_GND | UCB_TS_CR_TSPX_POW |
			UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);
	ucb1400_reg_write(ucb, UCB_TS_CR,
			UCB_TS_CR_TSMX_GND | UCB_TS_CR_TSPX_POW |
			UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);
	ucb1400_reg_write(ucb, UCB_TS_CR,
			UCB_TS_CR_TSMX_GND | UCB_TS_CR_TSPX_POW |
			UCB_TS_CR_MODE_POS | UCB_TS_CR_BIAS_ENA);

	udelay(ts_delay);

	return ucb1400_adc_read(ucb, UCB_ADC_INP_TSPY);
}

#define MAX_XDELTA 16
#define MAX_YDELTA 16

/*
 * Switch to Y position mode and measure X plate.  We switch the plate
 * configuration in pressure mode, then switch to position mode.  This
 * gives a faster response time.  Even so, we need to wait about 55us
 * for things to stabilise.
 */
static inline unsigned int ucb1400_ts_read_ypos(struct ucb1400 *ucb)
{
	ucb1400_reg_write(ucb, UCB_TS_CR,
			UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_POW |
			UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);
	ucb1400_reg_write(ucb, UCB_TS_CR,
			UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_POW |
			UCB_TS_CR_MODE_PRES | UCB_TS_CR_BIAS_ENA);
	ucb1400_reg_write(ucb, UCB_TS_CR,
			UCB_TS_CR_TSMY_GND | UCB_TS_CR_TSPY_POW |
			UCB_TS_CR_MODE_POS | UCB_TS_CR_BIAS_ENA);

	udelay(ts_delay);
	return ucb1400_adc_read(ucb, UCB_ADC_INP_TSPX);
}

/*
 * In five wire mode, IN0 is connnected to what would normally be X- (TSMX)
 * and always grounded (since it has a 'high' voltage higher than VBIAS).
 *
 * TSPX is set to always high
 *
 * TSMY and TSPY are swapped (one always high, one always low)
 *
 * TSMX is connected to the wiper and used for the ADC
 */
static inline unsigned int ucb1400_ts_read_xpos5(struct ucb1400 *ucb)
{
	unsigned prev = 0 ;
	unsigned curr ;
	int first = 1 ;
	int delta ;

	ucb1400_reg_write(ucb, UCB_TS_CR,
			x5_bits
			|UCB_TS_CR_MODE_POS
			|UCB_TS_CR_BIAS_ENA);

	udelay(ts_delay);

	return ucb1400_adc_read(ucb, UCB_ADC_INP_TSMX);

	do {
		curr = ucb1400_adc_read(ucb, UCB_ADC_INP_TSMX);
		if (first)
			first = 0 ; /* prev not valid. sample again */
		else {
			delta = curr-prev ;
			if (0 > delta)
				delta = 0-delta ;
		} /* prev is valid */
		prev = curr ;
	} while (MAX_XDELTA < delta);

	return curr ;
}

static inline unsigned int ucb1400_ts_read_ypos5(struct ucb1400 *ucb)
{
	unsigned prev = 0 ;
	unsigned curr ;
	int first = 1 ;
	int delta ;

	ucb1400_reg_write(ucb, UCB_TS_CR,
			y5_bits
			|UCB_TS_CR_MODE_POS
			|UCB_TS_CR_BIAS_ENA);

	udelay(ts_delay);

	return ucb1400_adc_read(ucb, UCB_ADC_INP_TSMX);
	do {
		curr = ucb1400_adc_read(ucb, UCB_ADC_INP_TSMX);
		if (first)
			first = 0 ; /* prev not valid. sample again */
		else {
			delta = curr-prev ;
			if (0 > delta)
				delta = 0-delta ;
		} /* prev is valid */
		prev = curr ;
	} while (MAX_YDELTA < delta);

	return curr ;
}

static int ucb1400_detect_irq(struct ucb1400 *ucb)
{
	unsigned long mask, timeout;

	mask = probe_irq_on();

	/* Enable the ADC interrupt. */
	ucb1400_reg_write(ucb, UCB_IE_RIS, UCB_IE_ADC);
	ucb1400_reg_write(ucb, UCB_IE_FAL, UCB_IE_ADC);
	ucb1400_reg_write(ucb, UCB_IE_CLEAR, 0xffff);
	ucb1400_reg_write(ucb, UCB_IE_CLEAR, 0);

	/* Cause an ADC interrupt. */
	ucb1400_reg_write(ucb, UCB_ADC_CR, UCB_ADC_ENA);
	ucb1400_reg_write(ucb, UCB_ADC_CR, UCB_ADC_ENA | UCB_ADC_START);

	/* Wait for the conversion to complete. */
	timeout = jiffies + HZ/2;
	while (!(ucb1400_reg_read(ucb, UCB_ADC_DATA) & UCB_ADC_DAT_VALID)) {
		cpu_relax();
		if (time_after(jiffies, timeout)) {
			printk(KERN_ERR "ucb1400: timed out in IRQ probe\n");
			probe_irq_off(mask);
			return -ENODEV;
		}
	}
	ucb1400_reg_write(ucb, UCB_ADC_CR, 0);

	/* Disable and clear interrupt. */
	ucb1400_reg_write(ucb, UCB_IE_RIS, 0);
	ucb1400_reg_write(ucb, UCB_IE_FAL, 0);
	ucb1400_reg_write(ucb, UCB_IE_CLEAR, 0xffff);
	ucb1400_reg_write(ucb, UCB_IE_CLEAR, 0);

	/* Read triggered interrupt. */
	ucb->irq = probe_irq_off(mask);
	if (ucb->irq < 0 || ucb->irq == NO_IRQ)
		return -ENODEV;

	return 0;
}

static int ucb1400_ts_thread(void *_ucb)
{
	struct ucb1400 *ucb = _ucb;
	struct task_struct *tsk = current;
	int valid = 0;
	struct sched_param param = { .sched_priority = 1 };

	sched_setscheduler(tsk, SCHED_FIFO, &param);

	set_freezable();
	while (!kthread_should_stop()) {
		switch (ucb->touch_type) {
			case TOUCH_4W: {
				unsigned int x, y, p;
				long timeout;

				ucb->ts_restart = 0;

				if (ucb->irq_pending) {
					ucb->irq_pending = 0;
					ucb1400_handle_pending_irq(ucb);
				}

				ucb1400_adc_enable(ucb);
				x = ucb1400_ts_read_xpos(ucb);
				y = ucb1400_ts_read_ypos(ucb);
				p = ucb1400_ts_read_pressure(ucb);
				ucb1400_adc_disable(ucb);

				/* Switch back to interrupt mode. */
				ucb1400_ts_mode_int(ucb);

				if (ucb1400_ts_pen_down(ucb)) {
					ucb1400_ts_irq_enable(ucb);

					/*
					 * If we spat out a valid sample set
					 * last time, spit out a "pen off"
					 * sample here.
					 */
					if (valid) {
						x = 0;
						y = 0;
						p = 0;
						valid = 0;
					}

					timeout = MAX_SCHEDULE_TIMEOUT;
				} else {
					valid = 1;
					timeout = msecs_to_jiffies(10);
				}

				add_event(ucb, x, y, p);
				wake_up_interruptible(&ucb->read_wait);
				wait_event_interruptible_timeout(
							ucb->sample_wait,
							ucb->irq_pending ||
							ucb->ts_restart ||
							kthread_should_stop(),
							timeout);
				break;
			}
			case TOUCH_5W: {
				unsigned int x, y, p;
				long timeout;

				ucb->ts_restart = 0;

				if (ucb->irq_pending) {
					ucb->irq_pending = 0;
					ucb1400_handle_pending_irq(ucb);
				}

				ucb1400_adc_enable(ucb);
				x = ucb1400_ts_read_xpos5(ucb);
				y = ucb1400_ts_read_ypos5(ucb);
				/*p = ucb1400_ts_read_pressure(ucb);*/
				ucb1400_adc_disable(ucb);

				/* Switch back to interrupt mode. */
				ucb1400_ts_mode_int(ucb);

				udelay(150);

				if (ucb1400_ts_pen_down(ucb)) {
					p = 0 ;
					ucb1400_ts_irq_enable5(ucb);

					/*
					 * If we spat out a valid sample set
					 * last time, spit out a "pen off"
					 * sample here.
					 */
					if (valid) {
						x = 0;
						y = 0;
						valid = 0;
					}

					timeout = MAX_SCHEDULE_TIMEOUT;
				} else {
					p = 1 ;
					valid = 1;
					timeout = msecs_to_jiffies(10);
				}

				add_event(ucb, x, y, p);
				wake_up_interruptible(&ucb->read_wait);
				wait_event_interruptible_timeout(
							ucb->sample_wait,
							ucb->irq_pending ||
							ucb->ts_restart ||
							kthread_should_stop(),
							timeout);
				break;
			}
			default : {
				touch_type = discover_touch_type(ucb);
				ucb->touch_type = touch_type;
				if (0 > ucb->touch_type) {
					msleep(100);
				} else {
					printk(KERN_INFO "%s: detected touch\
							screen type %d\n",
							__FILE__,
							ucb->touch_type);
				}
				break;
			}
		}
	}

	/* Send the "pen off" if we are stopping with the pen still active */
	if (valid)
		printk(KERN_INFO "write data regarding release event");

	ucb->ts_task = NULL;
	return 0;
}

static irqreturn_t ucb1400_hard_irq(int irqnr, void *devid)
{
	struct ucb1400 *ucb = devid;

	if (irqnr == ucb->irq) {
		disable_irq(ucb->irq);
		ucb->irq_pending = 1;
		wake_up(&ucb->sample_wait);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

static int ucb1400_read_proc(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
	return snprintf(page, 512, "%s", touch_type_names[touch_type+1]);
}

static unsigned int ucb1400_ts_poll(struct file *filp, poll_table *wait)
{
	struct ucb1400 *ucb = (struct ucb1400 *)filp->private_data;
	if (ucb) {
		poll_wait(filp, &ucb->read_wait, wait);
		return (eventq_isempty(ucb)) ? 0 : POLLIN | POLLRDNORM;
	}
	return -EINVAL;
}

ssize_t ucb1400_ts_read(struct file *filp, char __user *buf, size_t count,
			loff_t *f_pos)
{
	struct ucb1400 *ucb = filp->private_data;
	char *ptr = buf;
	size_t l_count = count;
	int error = 0;

	/*we have data available. read data.*/
	while (l_count >= sizeof(struct ts_event)) {
		error = -ERESTARTSYS;
		if (signal_pending(current))
			break;

		if (!eventq_isempty(ucb)) {
			struct ts_event elem;
			eventq_pop(ucb, &elem);

			error = copy_to_user(ptr, &elem,
						sizeof(struct ts_event));

			if (error)
				break;

			ptr += sizeof(struct ts_event);
			l_count -= sizeof(struct ts_event);
			continue;
		}

		if (filp->f_flags & O_NONBLOCK) {
			error = -EAGAIN;
			break;
		}
		wait_event_interruptible(ucb->read_wait, !eventq_isempty(ucb));
	}

	return (ptr == buf)? error : ptr - buf;
}

ssize_t ucb1400_ts_write(struct file *filp, const char __user *buf,
			size_t count, loff_t *f_pos)
{
	return -EINVAL;
}

static int ucb1400_ts_open(struct inode *inode, struct file *filp)
{
	struct ucb1400 *ucb = gucb;
	int ret = 0;

	BUG_ON(ucb->ts_task);

	ucb->ts_task = kthread_run(ucb1400_ts_thread, ucb, "UCB1400_ts");

	if (IS_ERR(ucb->ts_task)) {
		ret = PTR_ERR(ucb->ts_task);
		ucb->ts_task = NULL;
	}

	if (ret == 0)
		filp->private_data = ucb;

	return 0;
}

static int ucb1400_ts_release(struct inode *inode, struct file *filp)
{
	struct ucb1400 *ucb = gucb;

	if (ucb->ts_task)
		kthread_stop(ucb->ts_task);

	ucb1400_ts_irq_disable(ucb);
	ucb1400_reg_write(ucb, UCB_TS_CR, 0);
	return 0;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = ucb1400_ts_read,
	.poll = ucb1400_ts_poll,
	.write = ucb1400_ts_write,
	.open = ucb1400_ts_open,
	.release = ucb1400_ts_release,
};

static inline int ts_register(struct ucb1400 *ucb)
{
	init_waitqueue_head(&ucb->read_wait);

	ucb->miscdev.minor = ts_minor;
	ucb->miscdev.name = "ucb1x00";
	ucb->miscdev.fops = &fops;

	return misc_register(&ucb->miscdev);
}

static inline void ts_unregister(struct ucb1400 *ucb)
{
	misc_deregister(&ucb->miscdev);
}

static int ucb1400_ts_probe(struct device *dev)
{
	struct ucb1400 *ucb = kzalloc(sizeof(struct ucb1400), GFP_KERNEL);
	int error, id, x_res, y_res;

	if (!ucb) {
		error = -ENOMEM;
		goto err_free_devs;
	}

	ucb->touch_type = TOUCH_NOT_VALID ;
	ucb->adcsync = adcsync;
	ucb->ac97 = to_ac97_t(dev);
	init_waitqueue_head(&ucb->sample_wait);

	id = ucb1400_reg_read(ucb, UCB_ID);
	if (id != UCB_ID_1400) {
		error = -ENODEV;
		goto err_free_devs;
	}

	if (ts_irq != -1) {
		ucb->irq = ts_irq;
		printk(KERN_INFO "Using IRQ value that was passed as a\
				parameter to this driver\n");
	} else {
		error = ucb1400_detect_irq(ucb);
		if (error) {
			printk(KERN_ERR "UCB1400: IRQ probe failed\n");
			goto err_free_devs;
		}
	}

	printk(KERN_ALERT "UCB1400: found IRQ %d\n", ucb->irq);

	error = request_irq(ucb->irq, ucb1400_hard_irq, IRQF_TRIGGER_RISING,
				"UCB1400", ucb);
	if (error) {
		printk(KERN_ERR "ucb1400: unable to grab irq%d: %d\n",
				ucb->irq, error);
		goto err_free_devs;
	}

	ucb1400_adc_enable(ucb);
	x_res = ucb1400_ts_read_xres(ucb);
	y_res = ucb1400_ts_read_yres(ucb);
	ucb1400_adc_disable(ucb);
	printk(KERN_ALERT "UCB1400: x/y = %d/%d\n", x_res, y_res);

	touch_type = ucb->touch_type = discover_touch_type(ucb);
	printk(KERN_ALERT "----> Touch screen is type %d\n", ucb->touch_type);

	dev_set_drvdata(dev, ucb);
	error = ts_register(ucb);

	if (error == 0) {
		gucb = ucb;
		ucb->pde = create_proc_entry(TSTYPE_PROCNAME, 0, 0);
		if (ucb->pde)
			ucb->pde->read_proc  = ucb1400_read_proc ;
	} else {
		printk(KERN_ALERT "ts_register failed");
		ts_unregister(ucb);
	}

	return 0;

err_free_devs:
	kfree(ucb);
	return error;
}

static int ucb1400_ts_remove(struct device *dev)
{
	struct ucb1400 *ucb = dev_get_drvdata(dev);

	if (ucb->pde)
		remove_proc_entry(TSTYPE_PROCNAME, 0);
	printk(KERN_INFO "Freeing IRQ: %d\n", ucb->irq);
	free_irq(ucb->irq, ucb);
	dev_set_drvdata(dev, NULL);
	ts_unregister(ucb);
	gucb = NULL;
	kfree(ucb);
	return 0;
}

static struct device_driver ts_driver = {
	.name		= "ucb1400_ts",
	.owner		= THIS_MODULE,
	.bus		= &ac97_bus_type,
	.probe		= ucb1400_ts_probe,
	.remove		= ucb1400_ts_remove,
};

static int ts_init(void)
{
	/* sanity on MAX_EVENTS defined value to ensure that the queue
	 * manipulation functions would work properly */

	if (MAX_EVENTS) {
		/* ensure MAX_EVENTS is a power of 2 */
		if (MAX_EVENTS & (MAX_EVENTS - 1)) {
			printk(KERN_ALERT "MAX_EVENTS which defines the queue\
					size has to be a power of 2\n");
			return -1;
		}
	} else {
		printk(KERN_ALERT "Event queue size cannot be 0. Please set\
				the MAX_EVENTS value");
		return -1;
	}
	return driver_register(&ts_driver);
}

static void ts_exit(void)
{
	driver_unregister(&ts_driver);
}

module_init(ts_init);
module_exit(ts_exit);
