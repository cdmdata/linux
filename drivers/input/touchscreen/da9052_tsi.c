/*
 * da9052_tsi.c  --  TSI driver for Dialog DA9052
 *
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 *
 * Author: Dialog Semiconductor Ltd <dchen@diasemi.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
#include <linux/module.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/freezer.h>
#include <linux/kthread.h>

#include <linux/mfd/da9052/reg.h>
#include <linux/mfd/da9052/tsi_cfg.h>
#include <linux/mfd/da9052/tsi.h>
#include <linux/mfd/da9052/gpio.h>
#include <linux/mfd/da9052/adc.h>

//#define CONFIG_FIVE_WIRE
#define WAIT_FOR_PEN_DOWN	0
#define WAIT_FOR_SAMPLING	1
#define SAMPLING_ACTIVE		2

static int calibration[7];
module_param_array(calibration, int, NULL, S_IRUGO | S_IWUSR);

int *da9052_get_calibration(void)
{
	return calibration ;
}

static void da9052_tsi_reg_pendwn_event(struct da9052_ts_priv *priv);
static void da9052_tsi_reg_datardy_event(struct da9052_ts_priv *priv);
static ssize_t da9052_tsi_config_power_supply(struct da9052_ts_priv *priv,
					u8 state);
static void da9052_tsi_penup_event(struct da9052_ts_priv *priv);
static ssize_t da9052_tsi_reg_proc_thread(void *ptr);

u32 da9052_tsi_get_input_dev(struct da9052_tsi_info *ts, u8 off)
{
	if (off > NUM_INPUT_DEVS-1)
		return -EINVAL;
	return (u32)ts->input_devs[off];
}

static s32 da9052_tsi_get_rawdata(struct da9052_tsi *tsi, struct da9052_tsi_reg *buf, u8 cnt) {
	u32 data_cnt = 0;
	u32 rem_data_cnt = 0;

	mutex_lock(&tsi->tsi_fifo_lock);

	if (tsi->tsi_fifo_start < tsi->tsi_fifo_end) {
		data_cnt = (tsi->tsi_fifo_end - tsi->tsi_fifo_start);

		if (cnt < data_cnt)
			data_cnt = cnt;

		memcpy(buf, &tsi->tsi_fifo[tsi->tsi_fifo_start],
				sizeof(struct da9052_tsi_reg) * data_cnt);

		tsi->tsi_fifo_start += data_cnt;

		if (tsi->tsi_fifo_start == tsi->tsi_fifo_end) {
			tsi->tsi_fifo_start = 0;
			tsi->tsi_fifo_end = 0;
		}
	} else if (tsi->tsi_fifo_start > tsi->tsi_fifo_end) {
		data_cnt = ((TSI_FIFO_SIZE - tsi->tsi_fifo_start)
		+ tsi->tsi_fifo_end);

		if (cnt < data_cnt)
			data_cnt = cnt;

		if (data_cnt <= (TSI_FIFO_SIZE - tsi->tsi_fifo_start)) {
			memcpy(buf, &tsi->tsi_fifo[tsi->tsi_fifo_start],
				sizeof(struct da9052_tsi_reg) * data_cnt);

			tsi->tsi_fifo_start += data_cnt;
			if (tsi->tsi_fifo_start >= TSI_FIFO_SIZE)
				tsi->tsi_fifo_start = 0;
		} else {
			memcpy(buf, &tsi->tsi_fifo[tsi->tsi_fifo_start],
				sizeof(struct da9052_tsi_reg)
				* (TSI_FIFO_SIZE - tsi->tsi_fifo_start));

			rem_data_cnt = (data_cnt -
				(TSI_FIFO_SIZE - tsi->tsi_fifo_start));

			memcpy(buf, &tsi->tsi_fifo[0],
				sizeof(struct da9052_tsi_reg) * rem_data_cnt);

			tsi->tsi_fifo_start = rem_data_cnt;
		}

		if (tsi->tsi_fifo_start == tsi->tsi_fifo_end) {
				tsi->tsi_fifo_start = 0;
			tsi->tsi_fifo_end = 0;
		}
	} else
		data_cnt = 0;

	mutex_unlock(&tsi->tsi_fifo_lock);

	return data_cnt;
}

static ssize_t write_da9052_reg(struct da9052 *da9052, u8 reg_addr, u8 data)
{
	ssize_t ret = 0;
	struct da9052_ssc_msg ssc_msg;

	ssc_msg.addr =  reg_addr;
	ssc_msg.data =  data;
	ret = da9052->write(da9052, &ssc_msg);
	if (ret) {
		DA9052_DEBUG("%s: ", __func__);
		DA9052_DEBUG("da9052_ssc_write Failed %d\n", ret);
	}

	return ret;
}

static ssize_t da9052_tsi_config_auto_mode(struct da9052_ts_priv *priv,
						u8 state)
{
	unsigned set_mask;
	if (state == ENABLE)
		set_mask = DA9052_TSICONTA_AUTOTSIEN;
	else if (state == DISABLE)
		set_mask = 0;
	else {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG("Invalid Parameter Passed \n" );
		return -EINVAL;
	}
	return priv->da9052->register_modify(priv->da9052, DA9052_TSICONTA_REG,
			DA9052_TSICONTA_AUTOTSIEN, set_mask);
}

static ssize_t da9052_tsi_config_manual_mode(struct da9052_ts_priv *priv,
						u8 state)
{
	ssize_t ret=0;
	struct da9052_tsi_info *ts = &priv->tsi_info;
	unsigned set_mask;
	if (state == ENABLE)
		set_mask = DA9052_TSICONTB_TSIMAN;
	else if (state == DISABLE)
		set_mask = 0;
	else {
		DA9052_DEBUG("DA9052_TSI: %s: ", __FUNCTION__);
		DA9052_DEBUG("Invalid state Passed\n" );
		return -EINVAL;
	}
	ret = priv->da9052->register_modify(priv->da9052, DA9052_TSICONTB_REG,
			DA9052_TSICONTB_TSIMAN, set_mask);
	if (ret)
		return ret;

	if (state == DISABLE)
		ts->tsi_conf.man_cont.tsi_cont_b.tsi_man = RESET;
	else
		ts->tsi_conf.man_cont.tsi_cont_b.tsi_man = SET;

	ret = write_da9052_reg(priv->da9052, DA9052_ADCMAN_REG, DA9052_ADC_TSI);
	if (ret) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG("ADC write Failed \n" );
	}
	return ret;
}

static ssize_t da9052_tsi_config_pen_detect(struct da9052_ts_priv *priv,
						u8 flag)
{
	unsigned set_mask;
	if (flag == ENABLE)
		set_mask = DA9052_TSICONTA_PENDETEN;
	else if (flag == DISABLE)
		set_mask = 0;
	else {
		DA9052_DEBUG("%s:", __FUNCTION__);
		DA9052_DEBUG(" Invalid flag passed \n" );
		return -EINVAL;
	}
	return priv->da9052->register_modify(priv->da9052, DA9052_TSICONTA_REG,
			DA9052_TSICONTA_PENDETEN, set_mask);
}

static ssize_t da9052_tsi_config_gpio(struct da9052_ts_priv *priv)
{
	ssize_t ret = 0;
	struct da9052_ssc_msg ssc_msg[3];
	struct da9052_modify_msg mod_msg[3];

	ssc_msg[0].addr  = DA9052_GPIO0203_REG;
	ssc_msg[1].addr  = DA9052_GPIO0405_REG;
	ssc_msg[2].addr  = DA9052_GPIO0607_REG;

	mod_msg[0].clear_mask = DA9052_GPIO0203_GPIO3PIN;
	mod_msg[0].set_mask = 0;
	mod_msg[1].clear_mask = (DA9052_GPIO0405_GPIO4PIN | DA9052_GPIO0405_GPIO5PIN);
	mod_msg[1].set_mask = 0;
	mod_msg[2].clear_mask = (DA9052_GPIO0607_GPIO6PIN | DA9052_GPIO0607_GPIO7PIN);
	mod_msg[2].set_mask = 0;

	ret = priv->da9052->modify_many(priv->da9052, ssc_msg, mod_msg, 3);
	if (ret) {
		DA9052_DEBUG("%s: da9052_ssc_modify_many Failed\n", __FUNCTION__);
	}
	return ret;
}

static ssize_t da9052_tsi_config_measure_seq(struct da9052_ts_priv *priv,
						enum TSI_MEASURE_SEQ seq)
{
	unsigned set_mask;
	if (seq == XYZP_MODE)
		set_mask = 0;
	else if (seq == XP_MODE)
		set_mask = DA9052_TSICONTA_TSIMODE;
	else {
		DA9052_DEBUG("Invalid Value passed \n" );
		return -EINVAL;
	}
	return priv->da9052->register_modify(priv->da9052, DA9052_TSICONTA_REG,
			DA9052_TSICONTA_TSIMODE, set_mask);
}

static ssize_t da9052_tsi_set_sampling_mode(struct da9052_ts_priv *priv,
					u8 mode)
{
	struct da9052_tsi_info *ts = &priv->tsi_info;
	ssize_t ret = 0;
	unsigned set_mask;
	if (mode == ECONOMY_MODE)
		set_mask = 0;
	else if (mode == FAST_MODE)
		set_mask = DA9052_ADCCONT_ADCMODE;
	else {
		DA9052_DEBUG("DA9052_TSI:%s:", __FUNCTION__);
		DA9052_DEBUG("Invalid interval passed \n" );
		return -EINVAL;
	}

	ret = priv->da9052->register_modify(priv->da9052, DA9052_ADCCONT_REG,
			DA9052_ADCCONT_ADCMODE, set_mask);
	if (ret) {
		DA9052_DEBUG("DA9052_TSI:%s:", __FUNCTION__);
		DA9052_DEBUG("write_da9052_reg Failed \n" );
		return ret;
	}

	switch (mode) {
	case ECONOMY_MODE:
		priv->tsi_reg_data_poll_interval =
			TSI_ECO_MODE_REG_DATA_PROCESSING_INTERVAL;
		priv->tsi_raw_data_poll_interval =
			TSI_ECO_MODE_RAW_DATA_PROCESSING_INTERVAL;
		break;
	case FAST_MODE:
		priv->tsi_reg_data_poll_interval =
			TSI_FAST_MODE_REG_DATA_PROCESSING_INTERVAL;
		priv->tsi_raw_data_poll_interval =
			TSI_FAST_MODE_RAW_DATA_PROCESSING_INTERVAL;
		break;
	}

	ts->tsi_penup_count =
		(u32)priv->tsi_pdata->pen_up_interval /
		priv->tsi_reg_data_poll_interval;

	return 0;
}

static ssize_t da9052_tsi_config_delay(struct da9052_ts_priv *priv,
					enum TSI_DELAY delay)
{
	if (delay > priv->tsi_pdata->max_tsi_delay) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG(" invalid value for tsi delay!!!\n" );
		return -EINVAL;
	}
	return priv->da9052->register_modify(priv->da9052, DA9052_TSICONTA_REG,
			DA9052_TSICONTA_TSIDELAY,
			(delay << priv->tsi_pdata->tsi_delay_bit_shift));
}

static ssize_t da9052_tsi_config_skip_slots(struct da9052_ts_priv *priv,
					enum TSI_SLOT_SKIP skip)
{
	if (skip > priv->tsi_pdata->max_tsi_skip_slot) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG(" invalid value for tsi skip slots!!!\n" );
		return -EINVAL;
	}
	return priv->da9052->register_modify(priv->da9052, DA9052_TSICONTA_REG,
			DA9052_TSICONTA_TSISKIP,
			(skip << priv->tsi_pdata->tsi_skip_bit_shift));
}

static void da9052_tsi_5w_pen_down_handler(struct da9052_eh_nb *eh_data, u32 event)
{

}

static void da9052_tsi_pen_down_handler(struct da9052_eh_nb *eh_data, u32 event)
{
	ssize_t ret = 0;
	struct da9052_ts_priv *priv =
		container_of(eh_data, struct da9052_ts_priv, pd_nb);
	struct da9052_tsi_info *ts = &priv->tsi_info;
	struct input_dev *ip_dev =
		(struct input_dev*)da9052_tsi_get_input_dev(&priv->tsi_info,
		(u8)TSI_INPUT_DEVICE_OFF);

	if (priv->tsi_reg.tsi_state !=  WAIT_FOR_PEN_DOWN)
		return;
	DA9052_DEBUG("EH notified the pendown event 0x%x\n", event);

	priv->tsi_reg.tsi_state = WAIT_FOR_SAMPLING;

	if (ts->tsi_conf.state != TSI_AUTO_MODE) {
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG(" Configure TSI to auto mode.\n" );
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG(" Then call this API.\n" );
		goto fail;
	}

	if (da9052_tsi_config_power_supply(priv, ENABLE))
		goto fail;

	if (priv->da9052->event_disable(priv->da9052, priv->pd_nb.eve_type))
		goto fail;

	if (priv->da9052->event_enable(priv->da9052, priv->datardy_nb.eve_type))
		goto fail;

	if (da9052_tsi_config_auto_mode(priv, ENABLE))
		goto fail;

	input_sync(ip_dev);

	ts->pen_dwn_event = 1;

	priv->tsi_reg.tsi_state =  SAMPLING_ACTIVE;

	goto success;

fail:
	if (ts->pd_reg_status) {
		priv->da9052->unregister_event_notifier(priv->da9052,
						&priv->pd_nb);
		ts->pd_reg_status = RESET;

		priv->da9052->register_event_notifier(priv->da9052,
					&priv->datardy_nb);
		da9052_tsi_reg_pendwn_event(priv);
	}

success:
	ret = 0;
	DA9052_DEBUG ("Exiting PEN DOWN HANDLER \n");
}

static void da9052_tsi_reg_pendwn_event(struct da9052_ts_priv *priv)
{
	ssize_t ret = 0;
	struct da9052_tsi_info  *ts = &priv->tsi_info;

	if (ts->pd_reg_status) {
		DA9052_DEBUG("%s: Pen down ",__FUNCTION__);
		DA9052_DEBUG("Registeration is already done \n");
		return;
	}

#ifdef CONFIG_FIVE_WIRE
	priv->pd_nb.eve_type = GPI2_EVE;
	priv->pd_nb.call_back = &da9052_tsi_5w_pen_down_handler;
#else
	priv->pd_nb.eve_type = PEN_DOWN_EVE;
	priv->pd_nb.call_back = &da9052_tsi_pen_down_handler;
#endif
	ret = priv->da9052->register_event_notifier(priv->da9052, &priv->pd_nb);
	if (ret) {
		DA9052_DEBUG("%s: EH Registeration",__FUNCTION__);
		DA9052_DEBUG(" Failed: ret = %d\n",ret );
		ts->pd_reg_status = RESET;
	} else
		ts->pd_reg_status = SET;

	priv->os_data_cnt = 0;
	priv->raw_data_cnt = 0;

	return;
}

static void da9052_tsi_5w_data_ready_handler(struct da9052_eh_nb *eh_data, u32 event)
{

}

static void da9052_tsi_data_ready_handler(struct da9052_eh_nb *eh_data, u32 event)
{
	struct da9052_ssc_msg tsi_data[4];
	s32 ret;
	struct da9052_ts_priv *priv =
		container_of(eh_data, struct da9052_ts_priv, datardy_nb);

	if (priv->tsi_reg.tsi_state !=  SAMPLING_ACTIVE)
		return;

	tsi_data[0].addr  = DA9052_TSIXMSB_REG;
	tsi_data[1].addr  = DA9052_TSIYMSB_REG;
	tsi_data[2].addr  = DA9052_TSILSB_REG;
	tsi_data[3].addr  = DA9052_TSIZMSB_REG;

	tsi_data[0].data  = 0;
	tsi_data[1].data  = 0;
	tsi_data[2].data  = 0;
	tsi_data[3].data  = 0;

	da9052_lock(priv->da9052);

	ret = priv->da9052->read_many(priv->da9052, tsi_data, 4);
	da9052_unlock(priv->da9052);
	if (ret) {
		DA9052_DEBUG("Error in reading TSI data \n" );
		return;
	}

	mutex_lock(&priv->tsi_reg.tsi_fifo_lock);

	priv->tsi_reg.tsi_fifo[priv->tsi_reg.tsi_fifo_end].x_msb = tsi_data[0].data;
	priv->tsi_reg.tsi_fifo[priv->tsi_reg.tsi_fifo_end].y_msb = tsi_data[1].data;
	priv->tsi_reg.tsi_fifo[priv->tsi_reg.tsi_fifo_end].lsb   = tsi_data[2].data;
	priv->tsi_reg.tsi_fifo[priv->tsi_reg.tsi_fifo_end].z_msb = tsi_data[3].data;
	incr_with_wrap(priv->tsi_reg.tsi_fifo_end);

	if (priv->tsi_reg.tsi_fifo_end == priv->tsi_reg.tsi_fifo_start)
		priv->tsi_reg.tsi_fifo_start++;

	mutex_unlock(&priv->tsi_reg.tsi_fifo_lock);

}

static void da9052_tsi_reg_datardy_event(struct da9052_ts_priv *priv)
{
	ssize_t ret = 0;
	struct da9052_tsi_info  *ts = &priv->tsi_info;

	if(ts->datardy_reg_status)
	{
		DA9052_DEBUG("%s: Data Ready ",__FUNCTION__);
		DA9052_DEBUG("Registeration is already done \n");
		return;
	}

#ifdef CONFIG_FIVE_WIRE
	priv->datardy_nb.eve_type = ADC_EOM_EVE;
	priv->datardy_nb.call_back = &da9052_tsi_5w_data_ready_handler;
#else
	priv->datardy_nb.eve_type = TSI_READY_EVE;
	priv->datardy_nb.call_back = &da9052_tsi_data_ready_handler;
#endif
	ret = priv->da9052->register_event_notifier(priv->da9052,
						&priv->datardy_nb);

	if(ret)
	{
		DA9052_DEBUG("%s: EH Registeration",__FUNCTION__);
		DA9052_DEBUG(" Failed: ret = %d\n",ret );
		ts->datardy_reg_status = RESET;
	} else
		ts->datardy_reg_status = SET;

	return;
}

static ssize_t da9052_tsi_config_state(struct da9052_ts_priv *priv,
					enum TSI_STATE state)
{
	s32 ret;
	struct da9052_tsi_info *ts = &priv->tsi_info;

	if (ts->tsi_conf.state == state)
		return 0;

	switch (state) {
	case TSI_AUTO_MODE:
		ts->tsi_zero_data_cnt = 0;
		priv->early_data_flag = TRUE;
		priv->debounce_over = FALSE;
		priv->win_reference_valid = FALSE;

		clean_tsi_fifos(priv);

		ret = da9052_tsi_config_auto_mode(priv, DISABLE);
		if (ret)
			return ret;

		ret = da9052_tsi_config_manual_mode(priv, DISABLE);
		if (ret)
			return ret;

		ret = da9052_tsi_config_power_supply(priv, DISABLE);
		if (ret)
			return ret;

		ret = priv->da9052->event_enable(priv->da9052, priv->pd_nb.eve_type);
		if (ret)
			return ret;

		ret = priv->da9052->event_disable(priv->da9052, priv->datardy_nb.eve_type);
		if (ret)
			return ret;

		da9052_tsi_reg_pendwn_event(priv);
		da9052_tsi_reg_datardy_event(priv);

		ret = da9052_tsi_config_pen_detect(priv, ENABLE);
		if (ret)
			return ret;
		break;

	case TSI_IDLE:
		ts->pen_dwn_event = RESET;

		ret = da9052_tsi_config_pen_detect(priv, DISABLE);
		if (ret)
			return ret;

		ret = da9052_tsi_config_auto_mode(priv, DISABLE);
		if (ret)
			return ret;

		ret = da9052_tsi_config_manual_mode(priv, DISABLE);
		if (ret)
			return ret;

		ret = da9052_tsi_config_power_supply(priv, DISABLE);
		if (ret)
			return ret;

		if (ts->pd_reg_status) {
			priv->da9052->unregister_event_notifier(priv->da9052,
								&priv->pd_nb);
			ts->pd_reg_status = RESET;
		}
		break;

	default:
		DA9052_DEBUG("DA9052_TSI: %s:", __FUNCTION__);
		DA9052_DEBUG(" Invalid state passed");
		return -EINVAL;
	}

	ts->tsi_conf.state = state;

	return 0;
}


static ssize_t __init da9052_tsi_create_input_dev(struct input_dev **ip_dev,
							u8 n)
{
	u8 i;
	s32 ret;
	struct input_dev *dev = NULL;

	if (!n)
		return -EINVAL;

	for (i = 0; i < n; i++) {
		dev = input_allocate_device();
		if (!dev) {
			DA9052_DEBUG(KERN_ERR "%s:%s():memory allocation for \
					inputdevice failed\n", __FILE__,
								__FUNCTION__);
			return -ENOMEM;
		}

		ip_dev[i] = dev;
		switch (i) {
		case TSI_INPUT_DEVICE_OFF:
			dev->name = DA9052_TSI_INPUT_DEV;
			dev->phys = "input(tsi)";
			break;
		default:
			break;
		}
	}
	dev->id.vendor = DA9052_VENDOR_ID;
	dev->id.product = DA9052_PRODUCT_ID;
	dev->id.bustype = BUS_RS232;
	dev->id.version = TSI_VERSION;
	dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	dev->evbit[0] = (BIT_MASK(EV_SYN) |
			BIT_MASK(EV_KEY) |
			BIT_MASK(EV_ABS));

	input_set_abs_params(dev, ABS_X, 0, DA9052_DISPLAY_X_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, 0, DA9052_DISPLAY_Y_MAX, 0, 0);
	input_set_abs_params(dev, ABS_PRESSURE, 0, DA9052_TOUCH_PRESSURE_MAX,
				0, 0);

	ret = input_register_device(dev);
	if (ret) {
		DA9052_DEBUG(KERN_ERR "%s: Could ", __FUNCTION__);
		DA9052_DEBUG("not register input device(touchscreen)!\n");
		ret = -EIO;
		goto fail;
	}
	return 0;

fail:
	for (; i-- != 0; )
		input_free_device(ip_dev[i]);
	return -EINVAL;
}

static ssize_t __init da9052_tsi_init_drv(struct da9052_ts_priv *priv)
{
	u8 cnt = 0;
	ssize_t ret = 0;
	struct da9052_tsi_info  *ts = &priv->tsi_info;

	if ((DA9052_GPIO_PIN_3 != DA9052_GPIO_CONFIG_TSI) ||
		(DA9052_GPIO_PIN_4 != DA9052_GPIO_CONFIG_TSI) ||
		(DA9052_GPIO_PIN_5 != DA9052_GPIO_CONFIG_TSI) ||
		(DA9052_GPIO_PIN_6 != DA9052_GPIO_CONFIG_TSI) ||
		(DA9052_GPIO_PIN_7 != DA9052_GPIO_CONFIG_TSI)) {
		printk(KERN_ERR"DA9052_TSI: Configure DA9052 GPIO ");
		printk(KERN_ERR"pins for TSI\n");
		return -EINVAL;
	}

	ret = da9052_tsi_config_gpio(priv);

	ret = da9052_tsi_config_state(priv, TSI_IDLE);
	ts->tsi_conf.state = TSI_IDLE;

	da9052_tsi_config_measure_seq(priv, TSI_MODE_VALUE);

	da9052_tsi_config_skip_slots(priv, TSI_SLOT_SKIP_VALUE);

	da9052_tsi_config_delay(priv, TSI_DELAY_VALUE);

	da9052_tsi_set_sampling_mode(priv, DEFAULT_TSI_SAMPLING_MODE);

	ts->tsi_calib = get_calib_config();

	ret = da9052_tsi_create_input_dev(ts->input_devs, NUM_INPUT_DEVS);
	if (ret) {
		DA9052_DEBUG("DA9052_TSI: %s: ", __FUNCTION__);
		DA9052_DEBUG("da9052_tsi_create_input_dev Failed \n" );
		return ret;
	}

	da9052_init_tsi_fifos(priv);

	init_completion(&priv->tsi_reg_proc_thread.notifier);
	priv->tsi_reg_proc_thread.state = ACTIVE;
	priv->tsi_reg_proc_thread.pid =
				kernel_thread(da9052_tsi_reg_proc_thread,
					priv, CLONE_KERNEL | SIGCHLD);

	init_completion(&priv->tsi_raw_proc_thread.notifier);
	priv->tsi_raw_proc_thread.state = ACTIVE;
	priv->tsi_raw_proc_thread.pid =
				kernel_thread(da9052_tsi_raw_proc_thread,
					priv, CLONE_KERNEL | SIGCHLD);

	ret = da9052_tsi_config_state(priv, DEFAULT_TSI_STATE);
	if (ret) {
		for (cnt = 0; cnt < NUM_INPUT_DEVS; cnt++) {
			if (ts->input_devs[cnt] != NULL)
				input_free_device(ts->input_devs[cnt]);
		}
	}

	return 0;
}

s32 da9052_pm_configure_ldo(struct da9052_ts_priv *priv,
				struct da9052_ldo_config ldo_config)
{
	u8 ldo_volt;
	s8 ldo_pd_bit = -1;
	s32 ret = 0;
	unsigned sum;

	DA9052_DEBUG("I am in function: %s\n", __FUNCTION__);
	if (validate_ldo9_mV(ldo_config.ldo_volt))
		return INVALID_LDO9_VOLT_VALUE;

	ldo_volt = ldo9_mV_to_reg(ldo_config.ldo_volt);

	sum = ldo_volt | (ldo_config.ldo_conf ? DA9052_LDO9_LDO9CONF : 0);
	ret =  priv->da9052->register_modify(priv->da9052, DA9052_LDO9_REG,
			~DA9052_LDO9_LDO9EN, sum);
	if ((!ret) && (-1 != ldo_pd_bit)) {
		/* Never executes */
		ret =  priv->da9052->register_modify(priv->da9052, DA9052_PULLDOWN_REG,
				ldo_pd_bit, ldo_config.ldo_pd ? ldo_pd_bit : 0);

	}
	return ret;
}


s32 da9052_pm_set_ldo(struct da9052_ts_priv *priv, u8 ldo_num, u8 flag)
{
	DA9052_DEBUG("I am in function: %s\n", __FUNCTION__);

	return priv->da9052->register_modify(priv->da9052, DA9052_LDO9_REG,
			DA9052_LDO9_LDO9EN, (flag) ? DA9052_LDO9_LDO9EN : 0);
}

static ssize_t da9052_tsi_config_power_supply(struct da9052_ts_priv *priv,
						u8 state)
{
	struct da9052_ldo_config ldo_config;
	struct da9052_tsi_info *ts = &priv->tsi_info;

	if (state != ENABLE && state != DISABLE) {
		DA9052_DEBUG("DA9052_TSI: %s: ", __FUNCTION__);
		DA9052_DEBUG("Invalid state Passed\n" );
		return -EINVAL;
	}

	ldo_config.ldo_volt = priv->tsi_pdata->tsi_supply_voltage;
	ldo_config.ldo_num =  priv->tsi_pdata->tsi_ref_source;
	ldo_config.ldo_conf = RESET;

	if (da9052_pm_configure_ldo(priv, ldo_config))
		return -EINVAL;

	if (da9052_pm_set_ldo(priv, priv->tsi_pdata->tsi_ref_source, state))
		return -EINVAL;

	if (state == ENABLE)
		ts->tsi_conf.ldo9_en = SET;
	else
		ts->tsi_conf.ldo9_en = RESET;

	return 0;
}

static u32 da9052_tsi_get_reg_data(struct da9052_ts_priv *priv)
{
	u32 free_cnt, copy_cnt, cnt;

	if (down_interruptible(&priv->tsi_reg_fifo.lock))
		return 0;

	copy_cnt = 0;

	if ((priv->tsi_reg_fifo.head - priv->tsi_reg_fifo.tail) > 1) {
		free_cnt = get_reg_free_space_cnt(priv);
		if (free_cnt > TSI_POLL_SAMPLE_CNT)
			free_cnt = TSI_POLL_SAMPLE_CNT;

		cnt = da9052_tsi_get_rawdata(&priv->tsi_reg,
			&priv->tsi_reg_fifo.data[priv->tsi_reg_fifo.tail],
			free_cnt);

		if (cnt > free_cnt) {
			DA9052_DEBUG("EH copied more data");
			return -EINVAL;
		}

		copy_cnt = cnt;

		while (cnt--)
			incr_with_wrap_reg_fifo(priv->tsi_reg_fifo.tail);

	} else if ((priv->tsi_reg_fifo.head - priv->tsi_reg_fifo.tail) <= 0) {

		free_cnt = (TSI_REG_DATA_BUF_SIZE - priv->tsi_reg_fifo.tail);
		if (free_cnt > TSI_POLL_SAMPLE_CNT) {
			free_cnt = TSI_POLL_SAMPLE_CNT;

			cnt = da9052_tsi_get_rawdata(&priv->tsi_reg,
			&priv->tsi_reg_fifo.data[priv->tsi_reg_fifo.tail],
			free_cnt);
			if (cnt > free_cnt) {
				DA9052_DEBUG("EH copied more data");
				return -EINVAL;
			}
			copy_cnt = cnt;

			while (cnt--)
				incr_with_wrap_reg_fifo(
						priv->tsi_reg_fifo.tail);
		} else {
			if (free_cnt) {
				cnt = da9052_tsi_get_rawdata(&priv->tsi_reg,
					&priv->tsi_reg_fifo.data[priv->
					tsi_reg_fifo.tail], free_cnt);
				if (cnt > free_cnt) {
					DA9052_DEBUG("EH copied more data");
					return -EINVAL;
				}
				copy_cnt = cnt;
				while (cnt--)
					incr_with_wrap_reg_fifo(
						priv->tsi_reg_fifo.tail);
			}
			free_cnt = priv->tsi_reg_fifo.head;
			if (free_cnt > TSI_POLL_SAMPLE_CNT - copy_cnt)
				free_cnt = TSI_POLL_SAMPLE_CNT - copy_cnt;
			if (free_cnt) {
				cnt = da9052_tsi_get_rawdata(&priv->tsi_reg,
					&priv->tsi_reg_fifo.data[priv->
					tsi_reg_fifo.tail], free_cnt);
				if (cnt > free_cnt) {
					DA9052_DEBUG("EH copied more data");
					return -EINVAL;
				}

				copy_cnt += cnt;

				while (cnt--)
					incr_with_wrap_reg_fifo(
						priv->tsi_reg_fifo.tail);
			}
		}
	} else
		copy_cnt = 0;

	up(&priv->tsi_reg_fifo.lock);

	return copy_cnt;
}


static ssize_t da9052_tsi_reg_proc_thread(void *ptr)
{
	u32 data_cnt;
	struct da9052_tsi_info *ts;
	struct da9052_ts_priv *priv = (struct da9052_ts_priv *)ptr;

	set_freezable();

	while (priv->tsi_reg_proc_thread.state == ACTIVE) {

		try_to_freeze();

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(priv->
					tsi_reg_data_poll_interval));

		ts = &priv->tsi_info;

		if (!ts->pen_dwn_event)
			continue;

		data_cnt = da9052_tsi_get_reg_data(priv);

		da9052_tsi_process_reg_data(priv);

		if (data_cnt)
			ts->tsi_zero_data_cnt = 0;
		else {
			if ((++(ts->tsi_zero_data_cnt)) >
						     ts->tsi_penup_count) {
				ts->pen_dwn_event = RESET;
				da9052_tsi_penup_event(priv);
			}
		}
	}

	complete_and_exit(&priv->tsi_reg_proc_thread.notifier, 0);
	return 0;
}


static void da9052_tsi_penup_event(struct da9052_ts_priv *priv)
{

	struct da9052_tsi_info *ts = &priv->tsi_info;
	struct input_dev *ip_dev =
		(struct input_dev *)da9052_tsi_get_input_dev(&priv->tsi_info,
		(u8)TSI_INPUT_DEVICE_OFF);

	if (da9052_tsi_config_auto_mode(priv, DISABLE))
		goto exit;

	if (da9052_tsi_config_power_supply(priv, ENABLE))
		goto exit;

	ts->tsi_conf.ldo9_en = RESET;

	if (priv->da9052->event_enable(priv->da9052, priv->pd_nb.eve_type))
		goto exit;


	priv->tsi_reg.tsi_state =  WAIT_FOR_PEN_DOWN;

	ts->tsi_zero_data_cnt = 0;
	priv->early_data_flag = TRUE;
	priv->debounce_over = FALSE;
	priv->win_reference_valid = FALSE;

	DA9052_DEBUG ("The raw data count is %d \n", priv->raw_data_cnt);
	DA9052_DEBUG ("The OS data count is %d \n", priv->os_data_cnt);
	DA9052_DEBUG ("PEN UP DECLARED \n");
	input_report_abs(ip_dev, ABS_PRESSURE, 0);
	input_report_key(ip_dev, BTN_TOUCH, 0);
	input_sync(ip_dev);
	priv->os_data_cnt = 0;
	priv->raw_data_cnt = 0;

exit:
	clean_tsi_fifos(priv);
	return;
}

static ssize_t da9052_tsi_suspend(struct platform_device *dev, 
							pm_message_t state)
{
	printk(KERN_INFO "%s: called\n", __FUNCTION__);
	return 0;
}

static ssize_t da9052_tsi_resume(struct platform_device *dev)
{
	printk(KERN_INFO "%s: called\n", __FUNCTION__);
	return 0;
}

static s32 __devinit da9052_tsi_probe(struct platform_device *pdev)
{

	struct da9052_ts_priv *priv;
	struct da9052_tsi_platform_data *pdata = pdev->dev.platform_data;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	mutex_init(&priv->tsi_reg.tsi_fifo_lock);
	priv->tsi_reg.tsi_state = WAIT_FOR_PEN_DOWN;
	priv->da9052 = dev_get_drvdata(pdev->dev.parent);
	platform_set_drvdata(pdev, priv);
	priv->tsi_pdata = pdata;

	if (da9052_tsi_init_drv(priv))
			return -EFAULT;


	printk(KERN_INFO "TSI Drv Successfully Inserted %s\n",
					DA9052_TSI_DEVICE_NAME);
	return 0;
}

static int __devexit da9052_tsi_remove(struct platform_device *pdev)
{
	struct da9052_ts_priv *priv = platform_get_drvdata(pdev);
	struct da9052_tsi_info *ts = &priv->tsi_info;
	s32 ret = 0, i = 0;

	ret = da9052_tsi_config_state(priv, TSI_IDLE);
	if (!ret)
		return -EINVAL;

	if (ts->pd_reg_status) {
		priv->da9052->unregister_event_notifier(priv->da9052,
							&priv->pd_nb);
		ts->pd_reg_status = RESET;
	}

	if (ts->datardy_reg_status) {
		priv->da9052->unregister_event_notifier(priv->da9052,
							&priv->datardy_nb);
		ts->datardy_reg_status = RESET;
	}

	mutex_destroy(&priv->tsi_reg.tsi_fifo_lock);

	priv->tsi_reg_proc_thread.state = INACTIVE;
	wait_for_completion(&priv->tsi_reg_proc_thread.notifier);

	priv->tsi_raw_proc_thread.state = INACTIVE;
	wait_for_completion(&priv->tsi_raw_proc_thread.notifier);

	for (i = 0; i < NUM_INPUT_DEVS; i++) {
		input_unregister_device(ts->input_devs[i]);
	}

	platform_set_drvdata(pdev, NULL);
	DA9052_DEBUG(KERN_DEBUG "Removing %s \n", DA9052_TSI_DEVICE_NAME);

	return 0;
}

static struct platform_driver da9052_tsi_driver = {
	.probe		= da9052_tsi_probe,
	.remove		= __devexit_p(da9052_tsi_remove),
	.suspend	= da9052_tsi_suspend,
	.resume		= da9052_tsi_resume,
	.driver		= {
				.name	= DA9052_TSI_DEVICE_NAME,
				.owner	= THIS_MODULE,
			},
};

static int __init da9052_tsi_init(void)
{
	printk("DA9052 TSI Device Driver, v1.0\n");
	return platform_driver_register(&da9052_tsi_driver);
}
module_init(da9052_tsi_init);

static void __exit da9052_tsi_exit(void)
{
	printk(KERN_ERR "TSI Driver %s Successfully Removed \n",
					DA9052_TSI_DEVICE_NAME);
	return;

}
module_exit(da9052_tsi_exit);

MODULE_DESCRIPTION("Touchscreen driver for Dialog Semiconductor DA9052");
MODULE_AUTHOR("Dialog Semiconductor Ltd <dchen@diasemi.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
