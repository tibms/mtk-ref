/*
 * BQ2588x battery charging driver
 *
 * Copyright (C) 2017 Texas Instruments *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


#define pr_fmt(fmt)	"[bq2588x] %s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/alarmtimer.h>

#include "bq2588x_reg.h"
#include "bq2588x.h"

#define	bq_info	pr_info
#define bq_dbg	pr_debug
#define bq_err	pr_err
#define bq_log	pr_err


enum {
	BQ25880 = 0x01,
	BQ25882 = 0x02
};

enum {
	CHARGE_STATE_NOT_CHARGING,
	CHARGE_STATE_PRECHARGE,
	CHARGE_STATE_FASTCHARGE,
	CHARGE_STATE_OTG,
};

enum int_mask {
	INT_MASK_ADC_DONE	= 0x00000080,
	INT_MASK_IINDPM		= 0x00000040,
	INT_MASK_VINDPM		= 0x00000020,
	INT_MASK_TREG		= 0x00000010,
	INT_MASK_WDT		= 0x00000008,

	INT_MASK_PG		= 0x00008000,
	INT_MASK_VBUS		= 0x00001000,
	INT_MASK_TS		= 0x00000400,
	INT_MASK_ICO		= 0x00000200,
	INT_MASK_VSYS		= 0x00000100,

	INT_MASK_VBUS_OVP	= 0x00800000,
	INT_MASK_TSHUT		= 0x00400000,
	INT_MASK_BAT_OVP	= 0x00200000,
	INT_MASK_TMR		= 0x00100000,
	INT_MASK_SYS_SHORT	= 0x00080000,
	INT_MASK_OTG		= 0x00010000,
};

struct bq2588x {
	struct device *dev;
	struct i2c_client *client;

	int part_no;
	int revision;

	struct mutex i2c_rw_lock;

	bool power_good;

	int vbus_volt;
	int vbat_volt;
	int vsys_volt;
	int ibus_curr;
	int ichg_curr;
	int die_temp;
	int ts_temp;

	bool saved_charge_enable;
	bool charge_enabled;	/* Register bit status */
	bool otg_enabled;
	bool vindpm_triggered;
	bool iindpm_triggered;

	struct delayed_work monitor_work;

};

static const struct charger_properties bq2588x_chg_props = {
	.alias_name = "bq2588x",
};


enum {
	ADC_IBUS,
	ADC_ICHG,
	ADC_VBUS,
	ADC_VBAT,
	ADC_VSYS,
	ADC_TS,
	ADC_TDIE,
	ADC_MAX_NUM
};


static int __bq2588x_read_reg(struct bq2588x *bq, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		bq_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		pm_relax(bq->dev);
		return ret;
	}

	*data = (u8)ret;


	return 0;
}

static int __bq2588x_write_reg(struct bq2588x *bq, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(bq->client, reg, val);
	if (ret < 0) {
		bq_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
				val, reg, ret);
		pm_relax(bq->dev);
		return ret;
	}


	return 0;
}

static int bq2588x_read_reg(struct bq2588x *bq, u8 reg, u8 *data)
{
	int ret;

	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2588x_read_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2588x_read_word(struct bq2588x *bq, u8 reg, u16 *data)
{
	s32 ret;

	mutex_lock(&bq->i2c_rw_lock);
	ret = i2c_smbus_read_word_data(bq->client, reg);
	mutex_unlock(&bq->i2c_rw_lock);
	if (ret >= 0)
		*data = (u16)ret;

	return ret;
}

static int bq2588x_write_reg(struct bq2588x *bq, u8 reg, u8 data)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2588x_write_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	if (ret)
		bq_err("Failed: reg=%02X, ret=%d\n", reg, ret);

	return ret;
}


static int bq2588x_update_bits(struct bq2588x *bq, u8 reg,
					u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	if (bq->skip_reads || bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2588x_read_reg(bq, reg, &tmp);
	if (ret) {
		bq_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __bq2588x_write_reg(bq, reg, tmp);
	if (ret)
		bq_err("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}


static int bq2588x_set_charge_voltage(struct bq2588x *bq, int volt)
{
	int ret;
	u8 reg_val;

	if (volt < BQ2588X_VREG_BASE)
		volt = BQ2588X_VREG_BASE;

	volt -= BQ2588X_VREG_BASE;
	reg_val = volt / BQ2588X_VREG_LSB;
	reg_val <<= BQ2588X_VREG_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHARGE_VOLT,
				BQ2588X_VREG_MASK, reg_val);

	return ret;
}

static int bq2588x_set_hiz_mode(struct bq2588x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2588X_HIZ_DISABLE;
	else
		reg_val = BQ2588X_HIZ_ENABLE;

	reg_val <<= BQ2588X_EN_HIZ_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHARGE_CURRENT,
				BQ2588X_EN_HIZ_MASK, reg_val);

	return ret;
}


static int bq2588x_get_hiz_mode(struct bq2588x *bq, u8 *status)
{
	int ret;
	u8 reg_val;

	ret = bq2588x_read_reg(bq, BQ2588X_REG_CHARGE_CURRENT, &reg_val);

	if (!ret)
		*status = reg_val & BQ2588X_EN_HIZ_MASK;

	return ret;
}

static int bq2588x_set_ilim_pin(struct bq2588x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2588X_ILIM_PIN_DISABLE;
	else
		reg_val = BQ2588X_ILIM_PIN_ENABLE;

	reg_val <<= BQ2588X_EN_ILIM_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHARGE_CURRENT,
				BQ2588X_EN_ILIM_MASK, reg_val);

	return ret;
}

static int bq2588x_set_charge_current(struct bq2588x *bq, int curr)
{
	int ret;
	u8 reg_val;

	if (curr < BQ2588X_ICHG_BASE)
		curr = BQ2588X_ICHG_BASE;

	curr -= BQ2588X_ICHG_BASE;
	reg_val = curr / BQ2588X_ICHG_LSB;
	reg_val <<= BQ2588X_ICHG_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHARGE_CURRENT,
				BQ2588X_ICHG_MASK, reg_val);

	return ret;
}


static int bq2588x_set_input_volt_limit(struct bq2588x *bq, int volt)
{
	int ret;
	u8 reg_val;

	if (volt < BQ2588X_VINDPM_TH_BASE)
		volt = BQ2588X_VINDPM_TH_BASE;

	volt -= BQ2588X_VINDPM_TH_BASE;
	reg_val = volt / BQ2588X_VINDPM_TH_LSB;
	reg_val <<= BQ2588X_VINDPM_TH_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_VINDPM,
				BQ2588X_VINDPM_TH_MASK, reg_val);

	return ret;

}


static int bq2588x_set_ico_mode(struct bq2588x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2588X_ICO_DISABLE;
	else
		reg_val = BQ2588X_ICO_ENABLE;

	reg_val <<= BQ2588X_EN_ICO_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_IINDPM,
				BQ2588X_EN_ICO_MASK, reg_val);

	return ret;
}

static int bq2588x_set_input_current_limit(struct bq2588x *bq, int curr)
{
	int ret;
	u8 reg_val;

	if (curr < BQ2588X_IINDPM_TH_BASE)
		curr = BQ2588X_IINDPM_TH_BASE;

	curr -= BQ2588X_IINDPM_TH_BASE;
	reg_val = curr / BQ2588X_IINDPM_TH_LSB;
	reg_val <<= BQ2588X_IINDPM_TH_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_IINDPM,
				BQ2588X_IINDPM_TH_MASK, reg_val);
	return ret;
}


static int bq2588x_set_prechg_current(struct bq2588x *bq, int curr)
{
	int ret;
	u8 reg_val;

	if (curr < BQ2588X_IPRECHG_BASE)
		curr = BQ2588X_IPRECHG_BASE;

	curr -= BQ2588X_IPRECHG_BASE;
	reg_val = curr / BQ2588X_IPRECHG_LSB;
	reg_val <<= BQ2588X_IPRECHG_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_PRECHG_TERM,
				BQ2588X_IPRECHG_MASK, reg_val);

	return ret;
}

static int bq2588x_set_term_current(struct bq2588x *bq, int curr)
{
	int ret;
	u8 reg_val;

	if (curr < BQ2588X_ITERM_BASE)
		curr = BQ2588X_ITERM_BASE;

	curr -= BQ2588X_ITERM_BASE;
	reg_val = curr / BQ2588X_ITERM_LSB;
	reg_val <<= BQ2588X_ITERM_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_PRECHG_TERM,
				BQ2588X_ITERM_MASK, reg_val);

	return ret;
}

static int bq2588x_set_watchdog_timer(struct bq2588x *bq, int time)
{
	int ret;
	u8 reg_val;

	if (time == 0)
		reg_val = BQ2588X_WDT_TIMER_DISABLE;
	else if (time == 40)
		reg_val = BQ2588X_WDT_TIMER_40S;
	else if (time == 80)
		reg_val = BQ2588X_WDT_TIMER_80S;
	else
		reg_val = BQ2588X_WDT_TIMER_160S;

	reg_val <<= BQ2588X_WDT_TIMER_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL1,
				BQ2588X_WDT_TIMER_MASK, reg_val);

	return ret;
}

static int bq2588x_enable_safety_timer(struct bq2588x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2588X_SAFETY_TIMER_DISABLE;
	else
		reg_val = BQ2588X_SAFETY_TIMER_ENABLE;

	reg_val <<= BQ2588X_SAFETY_TIMER_EN_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL1,
				BQ2588X_SAFETY_TIMER_EN_MASK, reg_val);

	return ret;
}

static int bq2588x_set_safety_timer(struct bq2588x *bq, int time)
{
	int ret;
	u8 reg_val;

	if (time == 5)
		reg_val = BQ2588X_SAFETY_TIMER_5H;
	else if (time == 8)
		reg_val = BQ2588X_SAFETY_TIMER_8H;
	else if (time == 12)
		reg_val = BQ2588X_SAFETY_TIMER_12H;
	else
		reg_val = BQ2588X_SAFETY_TIMER_20H;

	reg_val <<= BQ2588X_SAFETY_TIMER_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL1,
				BQ2588X_SAFETY_TIMER_MASK, reg_val);

	return ret;
}

static int bq2588x_charge_enable(struct bq2588x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2588X_CHARGE_DISABLE;
	else
		reg_val = BQ2588X_CHARGE_ENABLE;

	reg_val <<= BQ2588X_CHARGE_EN_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL2,
				BQ2588X_CHARGE_EN_MASK, reg_val);

	return ret;
}


static int bq2588x_otg_enable(struct bq2588x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2588X_OTG_DISABLE;
	else
		reg_val = BQ2588X_OTG_ENABLE;

	reg_val <<= BQ2588X_OTG_EN_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL2,
				BQ2588X_OTG_EN_MASK, reg_val);

	return ret;

}

static int bq2588x_term_enable(struct bq2588x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2588X_TERM_DISABLE;
	else
		reg_val = BQ2588X_TERM_ENABLE; 

	reg_val <<= BQ2588X_TERM_EN_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL1,
				BQ2588X_TERM_EN_MASK, reg_val);

	return ret;
}

static int bq2588x_reset_watchdog_timer(struct bq2588x *bq)
{
	int ret;
	u8 reg_val;

	reg_val = BQ2588X_WDT_RESET << BQ2588X_WDT_RESET_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL3,
				BQ2588X_WDT_RESET_MASK, reg_val);

	return ret;
}

static int bq2588x_set_topoff_timer(struct bq2588x *bq, int time)
{
	int ret;
	u8 reg_val;

	if (time == 0)
		reg_val = BQ2588X_TOPOFF_TIMER_DISABLE;
	else if (time == 15)
		reg_val = BQ2588X_TOPOFF_TIMER_15M;
	else if (time == 30)
		reg_val = BQ2588X_TOPOFF_TIMER_30M;
	else
		reg_val = BQ2588X_TOPOFF_TIMER_45M;

	reg_val <<= BQ2588X_TOPOFF_TIMER_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL3,
				BQ2588X_TOPOFF_TIMER_MASK, reg_val);

	return ret;
}

static int bq2588x_set_sys_min_volt(struct bq2588x *bq, int volt)
{
	int ret;
	u8 reg_val;

	if (volt < BQ2588X_SYS_MIN_VOLT_BASE)
		volt = BQ2588X_SYS_MIN_VOLT_BASE;

	volt -= BQ2588X_SYS_MIN_VOLT_BASE;
	reg_val = volt / BQ2588X_SYS_MIN_VOLT_LSB;
	reg_val <<= BQ2588X_SYS_MIN_VOLT_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL3,
				BQ2588X_SYS_MIN_VOLT_MASK, reg_val);

	return ret;
}

static int bq2588x_set_otg_current_limit(struct bq2588x *bq, int curr)
{
	int ret;
	u8 reg_val;

	if (curr < BQ2588X_OTG_ILIM_BASE)
		curr = BQ2588X_OTG_ILIM_BASE;

	curr -= BQ2588X_OTG_ILIM_BASE;
	reg_val = curr / BQ2588X_OTG_ILIM_LSB;
	reg_val <<= BQ2588X_OTG_ILIM_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_OTG_CTRL,
				BQ2588X_OTG_ILIM_MASK, reg_val);

	return ret;
}

static int bq2588x_set_otg_volt_limit(struct bq2588x *bq, int volt)
{
	int ret;
	u8 reg_val;

	if (volt < BQ2588X_OTG_VLIM_BASE)
		volt = BQ2588X_OTG_VLIM_BASE;

	volt -= BQ2588X_OTG_VLIM_BASE;
	reg_val = volt / BQ2588X_OTG_VLIM_LSB;
	reg_val <<= BQ2588X_OTG_VLIM_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_OTG_CTRL,
				BQ2588X_OTG_VLIM_MASK, reg_val);

	return ret;
}

static int bq2588x_get_ico_limit(struct bq2588x *bq, int *curr)
{
	int ret;
	u8 reg_val;

	ret = bq2588x_read_reg(bq, BQ2588X_REG_ICO_LIMIT, &reg_val);
	if (ret)
		return ret;

	*curr = reg_val & BQ2588X_ICO_ILIM_MASK;
	*curr >>= BQ2588X_ICO_ILIM_SHIFT;
	*curr *= BQ2588X_ICO_ILIM_LSB;
	*curr += BQ2588X_ICO_ILIM_BASE;

	return 0;
}


static int bq2588x_set_int_mask(struct bq2588x *bq, unsigned int mask)
{
	int ret;
	u8 reg_val;

	reg_val = mask & 0xFF;
	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_INT_MASK1, 0xFF, reg_val);
	if (!ret)
		return ret;

	reg_val = (mask >> 8) & 0xFF;
	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_INT_MASK2, 0xFF, reg_val);
	if (!ret)
		return ret;

	reg_val = (mask >> 16) & 0xFF;
	ret = bq2588x_update_bits(bq, BQ2588X_REG_FAULT_INT_MASK, 0xFF, reg_val);

	return ret;
}

static int bq2588x_enable_adc_scan(struct bq2588x *bq, bool enable)
{
	int ret;
	u8 reg_val;

	if (enable == false)
		reg_val = BQ2588X_ADC_SCAN_DISABLE;
	else
		reg_val = BQ2588X_ADC_SCAN_ENABLE;

	reg_val <<= BQ2588X_ADC_SCAN_EN_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_ADC_CTRL,
				BQ2588X_ADC_SCAN_EN_MASK, reg_val);

	return ret;
}

static int bq2588x_set_adc_scan_mode(struct bq2588x *bq, bool oneshot)
{
	int ret;
	u8 reg_val;

	if (oneshot == false)
		reg_val = BQ2588X_ADC_SCAN_CONTINUOUS;
	else
		reg_val = BQ2588X_ADC_SCAN_ONESHOT;

	reg_val <<= BQ2588X_ADC_SCAN_RATE_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_ADC_CTRL,
				BQ2588X_ADC_SCAN_RATE_MASK, reg_val);

	return ret;
}

static int bq2588x_set_adc_scan_bits(struct bq2588x *bq, int bits)
{
	int ret;
	u8 reg_val;

	if (bits == 15)
		reg_val = BQ2588X_ADC_SCAN_15BITS;
	else if (bits == 14)
		reg_val = BQ2588X_ADC_SCAN_14BITS;
	else if (bits == 13)
		reg_val = BQ2588X_ADC_SCAN_13BITS;
	else
		reg_val = BQ2588X_ADC_SCAN_12BITS;

	reg_val <<= BQ2588X_ADC_SCAN_BITS_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_ADC_CTRL,
				BQ2588X_ADC_SCAN_BITS_MASK, reg_val);

	return ret;
}

static int bq2588x_check_adc_scan_done(struct bq2588x *bq, bool *done)
{
	int ret;
	u8 reg_val;

	ret = bq2588x_read_reg(bq, BQ2588X_REG_CHG_STATUS1, &reg_val);
	if (!ret) 
		*done = !!(reg_val & BQ2588X_ADC_DONE_STAT_MASK);

	return ret;
}

static int bq2588x_wait_adc_scan_done(struct bq2588x *bq)
{
	int ret;
	bool done = false;
	int retry = 0;

	while(retry++ < 20)
		ret = bq2588x_check_adc_scan_done(bq, &done);
		if (!ret && done)
			return 1;
		msleep(100);
	}

	return 0;
}


#define ADC_RES_REG_BASE	0x17

static int bq2588x_read_adc_data(struct bq2588x *bq, u8 channel, int *val)
{
	int ret;
	u16 res;
	u8 reg;

	if (channel >= ADC_MAX_NUM)
		return -EINVAL;
	reg = ADC_RES_REG_BASE + (channel << 1);

	ret = bq2588x_read_word(bq, reg, &res);
	if (ret >= 0)
		*val = (int)res;

	return ret;
}

static int bq2588x_read_bus_volt(struct bq2588x *bq, int *volt)
{
	int ret;
	int reg_val;

	ret = bq2588x_read_adc_data(bq, ADC_VBUS, &reg_val);

	if (ret >= 0) {
		*volt = reg_val * BQ2588X_VBUS_ADC_LB_LSB;
		*volt += BQ2588X_VBUS_ADC_LB_BASE;
	}

	return ret;
}

static int bq2588x_read_bus_curr(struct bq2588x *bq, int *curr)
{
	int ret;
	int reg_val;

	ret = bq2588x_read_adc_data(bq, ADC_IBUS, &reg_val);

	if (ret >= 0) {
		*curr = reg_val * BQ2588X_IBUS_ADC_LB_LSB;
		*curr += BQ2588X_IBUS_ADC_LB_BASE;
	}

	return ret;
}

static int bq2588x_read_bat_volt(struct bq2588x *bq, int *volt)
{
	int ret;
	int reg_val;

	ret = bq2588x_read_adc_data(bq, ADC_VBAT, &reg_val);

	if (ret >= 0) {
		*volt = reg_val * BQ2588X_VBAT_ADC_LB_LSB;
		*volt += BQ2588X_VBAT_ADC_LB_BASE;
	}

	return ret;
}

static int bq2588x_read_bat_curr(struct bq2588x *bq, int *curr)
{
	int ret;
	int reg_val;

	ret = bq2588x_read_adc_data(bq, ADC_ICHG, &reg_val);

	if (ret >= 0) {
		*curr = reg_val * BQ2588X_ICHG_ADC_LB_LSB;
		*curr += BQ2588X_ICHG_ADC_LB_BASE;
	}

	return ret;
}

static int bq2588x_read_sys_volt(struct bq2588x *bq, int *volt)
{
	int ret;
	int reg_val;

	ret = bq2588x_read_adc_data(bq, ADC_VSYS, &reg_val);

	if (ret >= 0) {
		*volt = reg_val * BQ2588X_VSYS_ADC_LB_LSB;
		*volt += BQ2588X_VSYS_ADC_LB_BASE;
	}

	return ret;
}

static int bq2588x_read_ts_temp(struct bq2588x *bq, int *temp)
{
	int ret;
	int reg_val;

	ret = bq2588x_read_adc_data(bq, ADC_TS, &reg_val);

	if (ret >= 0) {
		*temp = reg_val * BQ2588X_TS_ADC_LB_LSB;
		*temp += BQ2588X_TS_ADC_LB_BASE;
	}

	return ret;
}

static int bq2588x_read_die_temp(struct bq2588x *bq, int *temp)
{
	int ret;
	int reg_val;

	ret = bq2588x_read_adc_data(bq, ADC_TDIE, &reg_val);

	if (ret >= 0) {
		*temp = reg_val * BQ2588X_TDIE_ADC_LB_LSB;
		*temp += BQ2588X_TDIE_ADC_LB_BASE;
	}

	return ret;
}

static struct bq2588x_platform_data *bq2588x_parse_dt(struct device *dev,
						struct bq2588x *bq)
{
	int ret;
	struct device_node *np = dev->of_node;
	struct bq2588x_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(struct bq2588x_platform_data),
						GFP_KERNEL);
	if (!pdata)
		return NULL;

	ret = of_property_read_u32(np, "ti,bq2588x,usb-vlim", &pdata->usb.vlim);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,usb-vlim\n");

	ret = of_property_read_u32(np, "ti,bq2588x,usb-ilim", &pdata->usb.ilim);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,usb-ilim\n");

	ret = of_property_read_u32(np, "ti,bq2588x,usb-vreg", &pdata->usb.vreg);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,usb-vreg\n");

	ret = of_property_read_u32(np, "ti,bq2588x,usb-ichg", &pdata->usb.ichg);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,usb-ichg\n");

	ret = of_property_read_u32(np, "ti,bq2588x,precharge-current", &pdata->iprechg);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,precharge-current\n");

	ret = of_property_read_u32(np, "ti,bq2588x,termination-current", &pdata->iterm);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,termination-current\n");

	ret = of_property_read_u32(np, "ti,bq2588x,otg-voltage", &pdata->otg_volt);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,otg-voltage\n");

	ret = of_property_read_u32(np, "ti,bq2588x,otg-current", &pdata->otg_current);
	if (ret)
		bq_err("Failed to read node of ti,bq2588x,otg-current\n");
	
	if (of_property_read_bool(np, "ti,bq2588x,enable_term"))
		pdata->en_term = true;
	else
		pdata->en_term = false;

	return pdata;
}


static int bq2588x_init_device(struct bq2588x *bq)
{
	int ret;
	unsigned int mask;

	ret = bq2588x_set_prechg_current(bq, bq->platform_data->iprechg);
	if (ret)
		bq_err("Failed to set prechg current, ret = %d\n", ret);

	ret = bq2588x_set_term_current(bq, bq->platform_data->iterm);
	if (ret)
		bq_err("Failed to set termination current, ret = %d\n", ret);

	ret = bq2588x_set_otg_volt_limit(bq, bq->platform_data->otg_volt);
	if (ret)
		bq_err("Failed to set otg voltage, ret = %d\n", ret);

	ret = bq2588x_set_otg_current_limit(bq, bq->platform_data->otg_current);
	if (ret)
		bq_err("Failed to set otg current, ret = %d\n", ret);

	ret = bq2588x_charge_enable(bq, true);
	if (ret) {
		bq_err("Failed to enable charger, ret = %d\n", ret);
	} else {
		bq->charge_enabled = true;
		bq_log("Charger Enabled Successfully!\n");
	}

	bq2588x_term_enable(bq, bq->platform_data->en_term);

	bq2588x_set_ilim_pin(bq, true);

	mask = INT_MASK_ADC_DONE | INT_MASK_VSYS | INT_MASK_TREG |
		INT_MASK_TS | INT_MASK_TSHUT | INT_MASK_SYS_SHORT |
		INT_MASK_OTG | INT_MASK_IINDPM | INT_MASK_VINDPM;
	bq2588x_set_int_mask(bq, mask);

	bq2588x_set_adc_scan_mode(bq, true);
	bq2588x_set_adc_scan_bits(bq, 15);
	bq2588x_enable_adc_scan(bq, true);

	return 0;
}

static int bq2588x_detect_device(struct bq2588x *bq)
{
	int ret;
	u8 data;

	ret = bq2588x_read_reg(bq, BQ2588X_REG_PART_NUM, &data);

	if (ret == 0) {
		bq->part_no = data & BQ2588X_PART_NO_MASK;
		bq->part_no >>= BQ2588X_PART_NO_SHIFT;
		bq->revision = data & BQ2588X_REVISION_MASK;
		bq->revision >>= BQ2588X_REVISION_SHIFT;
	}

	return ret;
}

static void bq2588x_update_status(struct bq2588x *bq)
{

	bq2588x_enable_adc_scan(bq, true);
	/* do one-shot scan */
	bq2588x_set_adc_scan_mode(bq, true);

	bq2588x_wait_adc_scan_done(bq);
	
	bq2588x_read_bus_volt(bq, &bq->vbus_volt);
	bq2588x_read_bat_volt(bq, &bq->vbat_volt);
	bq2588x_read_bus_curr(bq, &bq->ibus_curr);
	bq2588x_read_bat_curr(bq, &bq->ichg_curr);
	bq2588x_read_ts_temp(bq, &bq->ts_temp);
	bq2588x_read_die_temp(bq, &bq->die_temp);

	bq_log("vbus:%d, vbat:%d, ibus:%d, ichg:%d\n",
		bq->vbus_volt, bq->vbat_volt,
		bq->ibus_curr, bq->ichg_curr);

}

static void bq2588x_dump_status(struct bq2588x *bq)
{
	bq2588x_update_status(bq);
}



static ssize_t bq2588x_show_registers(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq2588x *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[200];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "bq2588x Reg");
	for (addr = 0x0; addr <= 0x25; addr++) {
		ret = bq2588x_read_reg(bq, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
				"Reg[%02X] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t bq2588x_store_registers(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq2588x *bq = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x16)
		bq2588x_write_reg(bq, (unsigned char)reg, (unsigned char)val);

	return count;
}

static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR,
			bq2588x_show_registers,
			bq2588x_store_registers);

static struct attribute *bq2588x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2588x_attr_group = {
	.attrs = bq2588x_attributes,
};

static int bq2588x_charging(struct charger_device *chg_dev, bool enable)
{
	
	struct bq2588x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret = 0;
	u8 val;

	ret = bq2588x_charge_enable(bq, enable);

	pr_err("%s charger %s\n", enable ? "enable" : "disable",
				  !ret ? "successfully" : "failed");
	
	ret = bq2588x_read_reg(bq, BQ2588X_REG_CHG_CTRL2, &val);

	if (!ret)
		bq->charge_enabled = !!(val & BQ2588X_CHARGE_EN_MASK);
	
	return ret;
}

static int bq2588x_plug_in(struct charger_device *chg_dev)
{

	int ret;

	ret = bq2588x_charging(chg_dev, true);

	if (ret)
		pr_err("Failed to enable charging:%d\n", ret);
	
	return ret;
}

static int bq2588x_plug_out(struct charger_device *chg_dev)
{
	int ret;

	ret = bq2588x_charging(chg_dev, false);

	if (ret)
		pr_err("Failed to disable charging:%d\n", ret);
	
	return ret;
}

static int bq2588x_dump_register(struct charger_device *chg_dev)
{
	struct bq2588x *bq = dev_get_drvdata(&chg_dev->dev);

	bq2588x_dump_regs(bq);

	return 0;
}

static int bq2588x_is_charging_enable(struct charger_device *chg_dev, bool *en)
{
	struct bq2588x *bq = dev_get_drvdata(&chg_dev->dev);
	
	*en = bq->charge_enabled;
	
	return 0;
}

static int bq2588x_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	struct bq2588x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 val;
	
	ret = bq2588x_read_reg(bq, BQ2588X_REG_CHG_STATUS1, &val);
	if (!ret) {
		val = val & BQ2588X_CHRG_STAT_MASK;
		val = val >> BQ2588X_CHRG_STAT_SHIFT;
		*done = (val == BQ2588X_CHRG_STAT_DONE);	
	}
	
	return ret;
}

static int bq2588x_set_ichg(struct charger_device *chg_dev, u32 curr)
{
	struct bq2588x *bq = dev_get_drvdata(&chg_dev->dev);

	return bq2588x_set_charge_current(bq, curr/1000);
}


static int bq2588x_get_ichg(struct charger_device *chg_dev, u32 *curr)
{
	struct bq2588x *bq = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int ichg;
	int ret;

	ret = bq2588x_read_reg(bq, BQ2588X_REG_CHARGE_CURRENT, &reg_val);
	if (!ret) {
		ichg = (reg_val & BQ2588X_ICHG_MASK) >> BQ2588X_ICHG_SHIFT;
		ichg = ichg * BQ2588X__ICHG_LSB + BQ2588X_ICHG_BASE;
		*curr = ichg * 1000;
	}

	return ret;
}

static int bq2588x_get_min_ichg(struct charger_device *chg_dev, u32 *curr)
{

	*curr = 50 * 1000;

	return 0;
}

static int bq2588x_set_vchg(struct charger_device *chg_dev, u32 volt)
{
	
	struct bq2588x *bq = dev_get_drvdata(&chg_dev->dev);

	return bq2588x_set_charge_voltage(bq, volt/1000);	
}

static int bq2588x_get_vchg(struct charger_device *chg_dev, u32 *volt)
{
	struct bq2588x *bq = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int vchg;
	int ret;

	ret = bq2588x_read_reg(bq, BQ2588X_REG_CHARGE_VOLT, &reg_val);
	if (!ret) {
		vchg = (reg_val & BQ2588X_VREG_MASK) >> BQ2588X_VREG_SHIFT;
		vchg = vchg * BQ2588X_VREG_LSB + BQ2588X_VREG_BASE;
		*volt = vchg * 1000;
	}

	return ret;
}

static int bq2588x_set_ivl(struct charger_device *chg_dev, u32 volt)
{

	struct bq2588x *bq = dev_get_drvdata(&chg_dev->dev);

	return bq2588x_set_input_volt_limit(bq, volt/1000);

}

static int bq2588x_set_icl(struct charger_device *chg_dev, u32 curr)
{

	struct bq2588x *bq = dev_get_drvdata(&chg_dev->dev);

	return bq2588x_set_input_current_limit(bq, curr/1000);
}

static int bq2588x_get_icl(struct charger_device *chg_dev, u32 *curr)
{
	struct bq2588x *bq = dev_get_drvdata(&chg_dev->dev);
	u8 reg_val;
	int icl;
	int ret;

	ret = bq2588x_read_reg(bq, BQ2588X_REG_IINDPM, &reg_val);
	if (!ret) {
		icl = (reg_val & BQ2588X_IINDPM_TH_MASK) >> BQ2588X_IINDPM_SHIFT;
		icl = icl * BQ2588X_IINDPM_TH_LSB + BQ2588X_IINDPM_TH_BASE;
		*curr = icl * 1000;
	}

	return ret;

}

static int bq2588x_kick_wdt(struct charger_device *chg_dev)
{
	struct bq2588x *bq = dev_get_drvdata(&chg_dev->dev);

	return bq2588x_reset_watchdog_timer(bq);
}

static int bq2588x_set_otg(struct charger_device *chg_dev, bool en)
{
	int ret;
	struct bq2588x *bq = dev_get_drvdata(&chg_dev);
	
	if (en) {
		bq->saved_charge_enable = bq->charge_enabled;
		if (bq->charge_enabled)
			bq2588x_charging(chg_dev, false);
	} else {
		bq2588x_charging(chg_dev, bq->saved_charge_enable);
	}

	ret = bq2588x_otg_enable(bq, en);

	return ret;
}

static int bq2588x_set_safety_timer(struct charger_device *chg_dev, bool en)
{
	struct bq2588x *bq = dev_get_drvdata(&chg_dev);
	int ret;
		
	ret = bq2588x_enable_safety_timer(bq);
		
	return ret;
}

static int bq2588x_is_safety_timer_enabled(struct charger_device *chg_dev, bool *en)
{
	struct bq2588x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 reg_val;

	ret = bq2588x_read_reg(bq, BQ2588X_REG_CHG_CTRL1, &reg_val);

	if (!ret) 
		*en = !!(reg_val & BQ2588X_SAFETY_TIMER_EN_MASK);
	
	return ret;
}


static int bq2588x_set_otg_ilmt(struct charger_device *chg_dev, u32 curr)
{
	struct bq2588x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;

	ret = bq2588x_set_otg_current_limit(bq, curr/1000);

	return ret;
}

static struct charger_ops bq2588x_chg_ops = {
	/* Normal charging */
	.plug_in = bq2588x_plug_in,
	.plug_out = bq2588x_plug_out,
	.dump_registers = bq2588x_dump_register,
	.enable = bq2588x_charging,
	.is_enabled = bq2588x_is_charging_enable,
	.get_charging_current = bq2588x_get_ichg,
	.set_charging_current = bq2588x_set_ichg,
	.get_input_current = bq2588x_get_icl,
	.set_input_current = bq2588x_set_icl,
	.get_constant_voltage = bq2588x_get_vchg,
	.set_constant_voltage = bq2588x_set_vchg,
	.kick_wdt = bq2588x_kick_wdt,
	.set_mivr = bq2588x_set_ivl,
	.is_charging_done = bq2588x_is_charging_done,
	.get_min_charging_current = bq2588x_get_min_ichg,

	/* Safety timer */
	.enable_safety_timer = bq2588x_set_safety_timer,
	.is_safety_timer_enabled = bq2588x_is_safety_timer_enabled,

	/* Power path */
	.enable_powerpath = NULL,
	.is_powerpath_enabled = NULL,

	/* OTG */
	.enable_otg = bq2588x_set_otg,
	.set_boost_current_limit = bq2588x_set_otg_ilmt,
	.enable_discharge = NULL,

	/* PE+/PE+20 */
	.send_ta_current_pattern = NULL,
	.set_pe20_efficiency_table = NULL,
	.send_ta20_current_pattern = NULL,
//	.set_ta20_reset = NULL,
	.enable_cable_drop_comp = NULL,

	/* ADC */
	.get_tchg_adc = NULL,
};



static int bq2588x_charger_probe(struct i2c_client *client, 
					const struct i2c_device_id *id)
{
	struct bq2588x *bq;

	int ret;
	

	bq = devm_kzalloc(&client->dev, sizeof(struct bq2588x), GFP_KERNEL);
	if (!bq) {
		pr_err("Out of memory\n");
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->client = client;

	i2c_set_clientdata(client, bq);
	
	mutex_init(&bq->i2c_rw_lock);
	
	ret = bq2588x_detect_device(bq);
	if(ret) {
		pr_err("No bq2588x device found!\n");
		return -ENODEV;
	}
	
	if (client->dev.of_node)
		bq->platform_data = bq2588x_parse_dt(&client->dev, bq);
	else
		bq->platform_data = client->dev.platform_data;
	
	if (!bq->platform_data) {
		pr_err("No platform data provided.\n");
		return -EINVAL;
	}

	bq->chg_dev = charger_device_register("bq2588x",
			&client->dev, bq, 
			&bq2588x_chg_ops,
			&bq2588x_chg_props);
	if (IS_ERR_OR_NULL(bq->chg_dev)) {
		ret = PTR_ERR(bq->chg_dev);
		goto err_0;
	}

	ret = bq2588x_init_device(bq);
	if (ret) {
		pr_err("Failed to init device\n");
		return ret;
	}

	
	ret = sysfs_create_group(&bq->dev->kobj, &bq2588x_attr_group);
	if (ret) {
		dev_err(bq->dev, "failed to register sysfs. err: %d\n", ret);
	}


	pr_err("bq2588x probe successfully, Part Num:%d, Revision:%d\n!", 
				bq->part_no, bq->revision);
	
	return 0;
	
err_0:
	
	return ret;
}

static int bq2588x_charger_remove(struct i2c_client *client)
{
	struct bq2588x *bq = i2c_get_clientdata(client);


	mutex_destroy(&bq->i2c_rw_lock);

	sysfs_remove_group(&bq->dev->kobj, &bq2588x_attr_group);


	return 0;
}


static void bq2588x_charger_shutdown(struct i2c_client *client)
{
}

static struct of_device_id bq2588x_charger_match_table[] = {
	{.compatible = "ti,bq2588x",},
	{},
};
MODULE_DEVICE_TABLE(of,bq2588x_charger_match_table);

static const struct i2c_device_id bq2588x_charger_id[] = {
	{ "bq25800-charger", BQ25800 },
	{ "bq25802-charger", BQ25802 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2588x_charger_id);

static struct i2c_driver bq2588x_charger_driver = {
	.driver 	= {
		.name 	= "bq2588x-charger",
		.owner 	= THIS_MODULE,
		.of_match_table = bq2588x_charger_match_table,
	},
	.id_table	= bq2588x_charger_id,
	
	.probe		= bq2588x_charger_probe,
	.remove		= bq2588x_charger_remove,
	.shutdown	= bq2588x_charger_shutdown,
	
};

module_i2c_driver(bq2588x_charger_driver);

MODULE_DESCRIPTION("TI BQ2588x Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");
