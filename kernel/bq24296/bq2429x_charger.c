/*
 * BQ2591x battery charging driver
 *
 * Copyright (C) 2017 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#define pr_fmt(fmt) "[bq2429x]:%s: " fmt, __func__

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
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>

#include "mtk_charger_intf.h"
#include "bq2429x_reg.h"

enum bq2429x_part_no {
	BQ24296 = 0x01,
};


struct bq2429x_config {
	int chg_mv;
	int chg_ma;

	int ivl_mv;
	int icl_ma;
	
	int iterm_ma;
	
	bool enable_term;
};


struct bq2429x {
	struct device	*dev;
	struct i2c_client *client;
	struct charger_device *chg_dev;
	struct charger_properties chg_props;
	
	enum bq2429x_part_no part_no;
	int revision;

	struct bq2429x_config cfg;

	struct delayed_work monitor_work;

	bool charge_enabled;

	int charge_state;

	int chg_mv;
	int chg_ma;
	int ivl_mv;
	int icl_ma;
	
	struct mutex i2c_rw_lock;
};


static int __bq2429x_read_reg(struct bq2429x *bq, u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}
	*data = (u8)ret;
	return 0;
}

static int __bq2429x_write_reg(struct bq2429x *bq, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(bq->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
				val, reg, ret);
		return ret;
	}
	return 0;
}

static int bq2429x_read_byte(struct bq2429x *bq, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2429x_read_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}


static int bq2429x_write_byte(struct bq2429x *bq, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2429x_write_reg(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}


static int bq2429x_update_bits(struct bq2429x *bq, u8 reg,
					u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2429x_read_reg(bq, reg, &tmp);
	if (ret)
		goto out;

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __bq2429x_write_reg(bq, reg, tmp);

out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}


static int bq2429x_enable_otg(struct bq2429x *bq)
{
	u8 val = REG01_OTG_ENABLE << REG01_OTG_CONFIG_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_01,
				   REG01_OTG_CONFIG_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2429x_enable_otg);

static int bq2429x_disable_otg(struct bq2429x *bq)
{
	u8 val = REG01_OTG_DISABLE << REG01_OTG_CONFIG_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_01,
				   REG01_OTG_CONFIG_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2429x_disable_otg);


static int bq2429x_enable_charger(struct bq2429x *bq)
{
	int ret;
	u8 val = REG01_CHG_ENABLE << REG01_CHG_CONFIG_SHIFT;

	ret = bq2429x_update_bits(bq, BQ2429X_REG_01, REG01_CHG_CONFIG_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2429x_enable_charger);

static int bq2429x_disable_charger(struct bq2429x *bq)
{
	int ret;
	u8 val = REG01_CHG_DISABLE << REG01_CHG_CONFIG_SHIFT;

	ret = bq2429x_update_bits(bq, BQ2429X_REG_01, REG01_CHG_CONFIG_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2429x_disable_charger);

int bq2429x_set_chargecurrent(struct bq2429x *bq, int curr)
{
	u8 ichg;
	if (curr < REG02_ICHG_BASE)
		curr = REG02_ICHG_BASE;

	ichg = (curr - REG02_ICHG_BASE)/REG02_ICHG_LSB;
	ichg <<=  REG02_ICHG_SHIFT;
	return bq2429x_update_bits(bq, BQ2429X_REG_02, REG02_ICHG_MASK, ichg);

}

int bq2429x_set_term_current(struct bq2429x *bq, int curr)
{
	u8 iterm;

	if (curr <  REG03_ITERM_BASE)
		curr = REG03_ITERM_BASE;
		
	iterm = (curr - REG03_ITERM_BASE) / REG03_ITERM_LSB;
	iterm <<= REG03_ITERM_SHIFT;
	return bq2429x_update_bits(bq, BQ2429X_REG_03, REG03_ITERM_MASK, iterm);
}

static int bq2429x_enable_term(struct bq2429x *bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = REG05_TERM_ENABLE;
	else
		val = REG05_TERM_DISABLE;

	val <<= REG05_EN_TERM_SHIFT;

	ret = bq2429x_update_bits(bq, BQ2429X_REG_05,
				REG05_EN_TERM_MASK, val);

	return ret;

}


int bq2429x_set_prechg_current(struct bq2429x *bq, int curr)
{
	u8 iprechg;

	if (iprechg < REG03_IPRECHG_BASE)
		iprechg = REG03_IPRECHG_BASE;

	iprechg = (curr - REG03_IPRECHG_BASE) / REG03_IPRECHG_LSB;
	iprechg <<= REG03_IPRECHG_SHIFT; 
	return bq2429x_update_bits(bq, BQ2429X_REG_03, REG03_IPRECHG_MASK, iprechg);
}

int bq2429x_set_chargevoltage(struct bq2429x *bq, int volt)
{
	u8 val;
	
	if (volt < REG04_VREG_BASE)
		volt = REG04_VREG_BASE;

	val = (volt - REG04_VREG_BASE)/REG04_VREG_LSB;
	val <<= REG04_VREG_SHIFT; 

	return bq2429x_update_bits(bq, BQ2429X_REG_04, REG04_VREG_MASK, val);
}


int bq2429x_set_input_volt_limit(struct bq2429x *bq, int volt)
{
	u8 val;

	if (volt < REG00_VINDPM_BASE)
		volt = REG00_VINDPM_BASE;

	val = (volt - REG00_VINDPM_BASE) / REG00_VINDPM_LSB;
	val <<=  REG00_VINDPM_SHIFT;
	return bq2429x_update_bits(bq, BQ2429X_REG_00, REG00_VINDPM_MASK, val);
}

int bq2429x_set_input_current_limit(struct bq2429x *bq, int curr)
{
	u8 val;

	switch (curr){
	case BQ2429X_ILIM_100mA:
		val = REG00_IINLIM_100MA;
		break;
	case BQ2429X_ILIM_150mA:
		val = REG00_IINLIM_150MA;	
		break;
	case BQ2429X_ILIM_900mA:
		val = REG00_IINLIM_900MA;
		break;
	case BQ2429X_ILIM_1000mA:
		val = REG00_IINLIM_1000MA;
		break;
	case BQ2429X_ILIM_1500mA:
		val = REG00_IINLIM_1500MA;
		break;
	case BQ2429X_ILIM_2000mA:
		val = REG00_IINLIM_2000MA;
		break;
	case BQ2429X_ILIM_3000mA:
		val = REG00_IINLIM_3000MA;
		break;
	default:
		val = REG00_IINLIM_500MA;
	}
	
	val <<=  REG00_IINLIM_SHIFT;
	return bq2429x_update_bits(bq, BQ2429X_REG_00, REG00_IINLIM_MASK, val);
}


int bq2429x_set_watchdog_timer(struct bq2429x *bq, u8 timeout)
{
	u8 val;

	val = (u8)((timeout - REG05_WDT_BASE) / REG05_WDT_LSB);
	val <<= REG05_WDT_SHIFT;
	return bq2429x_update_bits(bq, BQ2429X_REG_05, REG05_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2429x_set_watchdog_timer);

int bq2429x_disable_watchdog_timer(struct bq2429x *bq)
{
	u8 val = REG05_WDT_DISABLE << REG05_WDT_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_05, REG05_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2429x_disable_watchdog_timer);

int bq2429x_reset_watchdog_timer(struct bq2429x *bq)
{
	u8 val = REG01_WDT_RESET << REG01_WDT_RESET_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_01, REG01_WDT_RESET_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2429x_reset_watchdog_timer);

int bq2429x_reset_chip(struct bq2429x *bq)
{
	int ret;
	u8 val = REG01_REG_RESET << REG01_REG_RESET_SHIFT;

	ret = bq2429x_update_bits(bq, BQ2429X_REG_01, REG01_REG_RESET_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2429x_reset_chip);

int bq2429x_enter_hiz_mode(struct bq2429x *bq)
{
	u8 val = REG00_HIZ_ENABLE << REG00_ENHIZ_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_00, REG00_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2429x_enter_hiz_mode);

int bq2429x_exit_hiz_mode(struct bq2429x *bq)
{

	u8 val = REG00_HIZ_DISABLE << REG00_ENHIZ_SHIFT;

	return bq2429x_update_bits(bq, BQ2429X_REG_00, REG00_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2429x_exit_hiz_mode);

static int bq2429x_enable_safety_timer(struct bq2429x *bq)
{
	const u8 val = REG05_CHG_TIMER_ENABLE << REG05_EN_TIMER_SHIFT;
	
	return bq2429x_update_bits(bq, BQ2429X_REG_05, REG05_EN_TIMER_MASK,
				val);
}
EXPORT_SYMBOL_GPL(bq2429x_enable_safety_timer);


static int bq2429x_disable_safety_timer(struct bq2429x *bq)
{
	const u8 val = REG05_CHG_TIMER_DISABLE << REG05_EN_TIMER_SHIFT;
	
	return bq2429x_update_bits(bq, BQ2429X_REG_05, REG05_EN_TIMER_MASK,
				val);
}
EXPORT_SYMBOL_GPL(bq2429x_disable_safety_timer);

static ssize_t bq2429x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bq2429x *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[100];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "bq2429x Reg");
	for (addr = 0x0; addr <= 0x0A; addr++) {
		ret = bq2429x_read_byte(bq, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
					"Reg[%02X] = 0x%02X\n",	addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t bq2429x_store_registers(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq2429x *bq = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x0A)
		bq2429x_write_byte(bq, (unsigned char)reg, (unsigned char)val);

	return count;
}

static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, bq2429x_show_registers,
						bq2429x_store_registers);

static struct attribute *bq2429x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2429x_attr_group = {
	.attrs = bq2429x_attributes,
};


static int bq2429x_parse_dt(struct device *dev, struct bq2429x *bq)
{
	int ret;
	struct device_node *np = dev->of_node;

	bq->charge_enabled = !(of_property_read_bool(np, "ti,charging-disabled"));

	bq->cfg.enable_term = of_property_read_bool(np, "ti,bq2429x,enable-term");

	ret = of_property_read_u32(np, "ti,bq2429x,charge-voltage",
					&bq->cfg.chg_mv);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2429x,charge-current",
					&bq->cfg.chg_ma);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2429x,input-current-limit",
					&bq->cfg.icl_ma);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2429x,input-voltage-limit",
					&bq->cfg.ivl_mv);
	if (ret)
		return ret;
	
	ret = of_property_read_u32(np, "ti,bq2429x,term-current",
					&bq->cfg.iterm_ma);

	return ret;
}

static int bq2429x_detect_device(struct bq2429x* bq)
{
    int ret;
    u8 data;

    ret = bq2429x_read_byte(bq, &data, BQ2429X_REG_0A);
    if(ret == 0){
        bq->part_no = (data & REG0A_PN_MASK) >> REG0A_PN_SHIFT;
        bq->revision = (data & REG0A_DEV_REV_MASK) >> REG0A_DEV_REV_SHIFT;
    }

    return ret;
}


static int bq2429x_set_charge_profile(struct bq2429x *bq)
{
	int ret;

	pr_err("chg_mv:%d, chg_ma:%d, icl_ma:%d, ivl_mv:%d\n",
			bq->cfg.chg_mv, bq->cfg.chg_ma, bq->cfg.icl_ma, bq->cfg.ivl_mv);
			
	ret = bq2429x_set_chargevoltage(bq->chg_dev, bq->cfg.chg_mv * 1000);
	if (ret < 0) {
		pr_err("Failed to set charge voltage:%d\n", ret);
		return ret;
	}

	ret = bq2429x_set_chargecurrent(bq->chg_dev, bq->cfg.chg_ma * 1000);
	if (ret < 0) {
		pr_err("Failed to set charge current:%d\n", ret);
		return ret;
	}

	ret = bq2429x_set_input_current_limit(bq->chg_dev, bq->cfg.icl_ma * 1000);
	if (ret < 0) {
		pr_err("Failed to set input current limit:%d\n", ret);
		return ret;
	}

	ret = bq2429x_set_input_volt_limit(bq->chg_dev, bq->cfg.ivl_mv * 1000);
	if (ret < 0) {
		pr_err("Failed to set input voltage limit:%d\n", ret);
		return ret;
	}
	return 0;
}

static int bq2429x_init_device(struct bq2429x *bq)
{
	int ret;

	bq->chg_mv = bq->cfg.chg_mv;
	bq->chg_ma = bq->cfg.chg_ma;
	bq->ivl_mv = bq->cfg.ivl_mv;
	bq->icl_ma = bq->cfg.icl_ma;


	ret = bq2429x_enable_term(bq, bq->cfg.enable_term);
	if (ret < 0)
		pr_err("Failed to %s termination:%d\n",
			bq->cfg.enable_term ? "enable" : "disable", ret);

	bq2429x_set_term_current(bq, bq->cfg.iterm_ma);
	
	bq2429x_set_charge_profile(bq);

	if (bq->charge_enabled)
		ret = bq2429x_enable_charger(bq);
	else
		ret = bq2429x_disable_charger(bq);

	if (ret < 0)
		pr_err("Failed to %s charger:%d\n",
			bq->charge_enabled ? "enable" : "disable", ret);

	return 0;
}

static void bq2429x_dump_regs(struct bq2429x *bq)
{
	int ret;
	u8 addr;
	u8 val;

	for (addr = 0x00; addr <= 0x0A; addr++) {
		msleep(2);
		ret = bq2429x_read_byte(bq, addr, &val);
		if (!ret)
			pr_err("Reg[%02X] = 0x%02X\n", addr, val);
	}

}


static const unsigned char * chg_state_str[] = {
	"Ready", "Charging", "Charge Done", "Charge Fault"
};

static void bq2429x_check_status(struct bq2429x *bq)
{
	int ret;
	u8 val = 0;
	
	ret = bq2429x_read_byte(bq, BQ2415X_REG_00, &val);
	if (!ret) {
		bq->charge_state = (val & BQ2415X_STAT_MASK)
							>> BQ2415X_STAT_SHIFT;
		pr_err("Charge State:%s\n", chg_state_str[bq->charge_state]);
	}
}

static void bq2429x_monitor_workfunc(struct work_struct *work)
{
	struct bq2429x *bq = container_of(work, struct bq2429x, monitor_work.work);

	bq2429x_dump_regs(bq);
	bq2429x_check_status(bq);
	
	schedule_delayed_work(&bq->monitor_work, 5 * HZ);
}

/*
 *  Charger device interface
 */
static int bq2429x_plug_in(struct charger_device *chg_dev)
{
	int ret;
	
	ret = bq2429x_charging(chg_dev, true);
	if (!ret)
		pr_err("Failed to enable charging:%d\n", ret);
	
	return ret;
	
}

static int bq2429x_plug_out(struct charger_device *chg_dev)
{
	int ret;
	
	ret = bq2429x_charging(chg_dev, false);
	if (!ret)
		pr_err("Failed to enable charging:%d\n", ret);
	
	return ret;
}


static int bq2429x_dump_register(struct charger_device *chg_dev)
{

	struct bq2429x *bq = dev_get_drvdata(&chg_dev->dev);
	bq2429x_dump_regs(bq);
	
	return 0;
}


static int bq2429x_charging(struct charger_device *chg_dev, bool enable)
{
	struct bq2429x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret = 0;
	u8 val;

	if (enable)
		ret = bq2429x_enable_charger(bq);
	else
		ret = bq2429x_disable_charger(bq);

	pr_err("%s charger %s\n", enable ? "enable" : "disable",
				  !ret ? "successfully" : "failed");
	
	ret = bq2429x_read_byte(bq, BQ2429X_REG_01, &val);

	if (!ret)
		bq->charge_enabled = !!(val & REG01_CHG_CONFIG_MASK);
	
	return ret;
}

static int bq2429x_is_charging_enable(struct charger_device *chg_dev, bool *en)
{
	struct bq2429x *bq = dev_get_drvdata(&chg_dev->dev);
	
	*en = bq->charge_enabled;
	
	return 0;
}

static int bq2429x_is_charging_done(struct charger_device *chg_dev, bool *done)
{
	struct bq2429x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 val;
	
	ret = bq2429x_read_byte(bq, BQ2429X_REG_08, &val);
	if (!ret) {
		val = val & REG08_CHRG_STAT_MASK;
		val = val >> REG08_CHRG_STAT_SHIFT;
		*done = (val == CHARGE_STATE_CHGDONE);	
	}
	
	return ret;
}

static int bq2429x_set_ichg(struct charger_device *chg_dev, u32 curr)
{
	struct bq2429x *bq = dev_get_drvdata(&chg_dev->dev);
	
	curr /= 1000; /*to mA */
	
	return bq2429x_set_chargecurrent(bq, curr);
}

static int bq2429x_get_ichg(struct charger_device *chg_dev, u32 *curr)
{
	struct bq2429x *bq = dev_get_drvdata(&chg_dev->dev);
	u8 val;
	int ret;
	int ichg;
	
	ret = bq2429x_read_byte(bq, BQ2429X_REG_02, &val);
	if (!ret) {
		ichg = ((u32)(val & REG02_ICHG_MASK ) >> REG02_ICHG_SHIFT);
		ichg = ichg * REG02_ICHG_LSB;
		ichg +=  REG02_ICHG_BASE;
		
		*curr = ichg * 1000; /*to uA*/
	}
	
	return ret;
}

static int bq2429x_get_min_ichg(struct charger_device *chg_dev, u32 *curr)
{
	int ret = 0;
	
	*curr = 64 * 1000;/*64mA*/
	
	return ret;
}

static int bq2429x_set_vchg(struct charger_device *chg_dev, u32 volt)
{
	struct bq2429x *bq = dev_get_drvdata(&chg_dev->dev);

	volt /= 1000; /*to mV*/

	return bq2429x_set_chargevoltage(bq, volt);
}

static int bq2429x_get_vchg(struct charger_device *chg_dev, u32 *cv)
{
	struct bq2429x *bq = dev_get_drvdata(&chg_dev->dev);
	u8 val;
	int ret;
	int volt;
	
	ret = bq2429x_read_byte(bq, BQ2429X_REG_04, &val);
	if (!ret) {
		volt = val & REG04_VREG_MASK;
		volt = (volt >> REG04_VREG_SHIFT) * REG04_VREG_LSB;
		volt = volt + REG04_VREG_BASE;
		*cv = volt * 1000; /*to uV*/
	}
	
	return ret;
}

static int bq2429x_set_ivl(struct charger_device *chg_dev, u32 volt)
{
	struct bq2429x *bq = dev_get_drvdata(&chg_dev->dev);
	u8 val;

	volt /= 1000; /*to mV*/
	return bq2429x_set_input_volt_limit(bq, volt);
}


static int bq2429x_set_icl(struct charger_device *chg_dev, u32 curr)
{
	struct bq2429x *bq = dev_get_drvdata(&chg_dev->dev);
	u8 val;
	
	curr /= 1000;/*to mA*/
	
	return bq2429x_set_input_current_limit(bq, curr);
}

static int bq2429x_get_icl(struct charger_device *chg_dev, u32 *curr)
{
	struct bq2429x *bq = dev_get_drvdata(&chg_dev->dev);
	u8 val;
	int ret;
	int ilim = 0;
	
	ret = bq2429x_read_byte(bq, BQ2415X_REG_00, &val);
	if (!ret) {
		val = val & REG00_IINLIM_MASK;
		val = val >> REG00_IINLIM_SHIFT;
		switch (val) {
		case REG00_IINLIM_100MA:
			ilim = 100;
			break;
		case REG00_IINLIM_150MA:
			ilim = 150;
			break;
		case REG00_IINLIM_500MA:
			ilim = 500;
			break;
		case REG00_IINLIM_900MA:
			ilim = 900;
			break;
		case REG00_IINLIM_1000MA:
			ilim = 1000;
			break;
		case REG00_IINLIM_1500MA:
			ilim = 1500;
			break;
		case REG00_IINLIM_2000MA:
			ilim = 2000;
			break;
		case REG00_IINLIM_3000MA:
			ilim = 3000;
			break;
		default:
			ilim = 500;
		}
	}
	
	*curr = ilim * 1000; /*to uA*/

	return ret;								 
}

static int bq2429x_kick_wdt(struct charger_device *chg_dev)
{
	struct bq2429x *bq = dev_get_drvdata(&chg_dev->dev);

	return bq2429x_reset_watchdog_timer(bq);
}


static int bq2429x_set_otg(struct charger_device *chg_dev, bool en)
{
	struct bq2429x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 val;
	
	if (en)
		ret = bq2429x_enable_otg(bq);
	else
		ret = bq2429x_disable_otg(bq);
	
	return ret;
}

static int bq2429x_set_safety_timer(struct charger_device *chg_dev, bool en)
{
	
	struct bq2429x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;
	
	if (en)
		ret = bq2429x_enable_safety_timer(bq);
	else
		ret = bq2429x_disable_safety_timer(bq);

	return ret;

}

static int bq2429x_is_safety_timer_enabled(struct charger_device *chg_dev, bool *en)
{
	struct bq2429x *bq = dev_get_drvdata(&chg_dev->dev);
	int ret;
	u8 reg_val;

	ret = bq2429x_read_byte(bq, &reg_val, BQ2560X_REG_05);

	if (!ret) 
		*en = !!(reg_val & REG05_EN_TIMER_MASK);
	
	return ret;
}


static struct charger_ops bq2429x_chg_ops = {
	/* Normal charging */
	.plug_in = bq2429x_plug_in,
	.plug_out = bq2429x_plug_out,
	.dump_registers = bq2429x_dump_register,
	.enable = bq2429x_charging,
	.is_enabled = bq2429x_is_charging_enable,
	.get_charging_current = bq2429x_get_ichg,
	.set_charging_current = bq2429x_set_ichg,
	.get_input_current = bq2429x_get_icl,
	.set_input_current = bq2429x_set_icl,
	.get_constant_voltage = bq2429x_get_vchg,
	.set_constant_voltage = bq2429x_set_vchg,
	.kick_wdt = bq2429x_kick_wdt,
	.set_mivr = bq2429x_set_ivl,
	.is_charging_done = bq2429x_is_charging_done,
	.get_min_charging_current = bq2429x_get_min_ichg,

	/* Safety timer */
	.enable_safety_timer = bq2429x_set_safety_timer,
	.is_safety_timer_enabled = bq2429x_is_safety_timer_enabled,

	/* Power path */
	.enable_powerpath = NULL,
	.is_powerpath_enabled = NULL,

	/* OTG */
	.enable_otg = bq2429x_set_otg,
	.set_boost_current_limit = NULL,
	.enable_discharge = NULL,

	/* PE+/PE+20 */
	.send_ta_current_pattern = NULL,
	.set_pe20_efficiency_table = NULL,
	.send_ta20_current_pattern = NULL,
	.set_ta20_reset = NULL,
	.enable_cable_drop_comp = NULL,

	/* ADC */
	.get_tchg_adc = NULL,
};

static int bq2429x_charger_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct bq2429x *bq = NULL;
	int ret;

	bq = devm_kzalloc(&client->dev, sizeof(struct bq2429x), GFP_KERNEL);
	if (!bq) {
		pr_err("out of memory\n");
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->client = client;

	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);

	ret = bq2429x_detect_device(bq);
	if (!ret && bq->part_no == BQ24296) {
		pr_info("charger device bq2429x detected, revision:%d\n",
							bq->revision);
	} else {
		pr_info("no bq2429x charger device found:%d\n", ret);
		return -ENODEV;
	}

	if (client->dev.of_node) {
		ret = bq2429x_parse_dt(&client->dev, bq);
		if (!ret) {
			pr_err("device tree parse error!\n");
			return ret;
		}
	} else {
		pr_err("No device tree node for bq24157\n");
		return -ENODEV;
	}

		/* Register charger device */
	bq->chg_dev = charger_device_register(
		"bq2429x", &client->dev, bq, &bq2429x_chg_ops,
		&bq->chg_props);
	if (IS_ERR_OR_NULL(bq->chg_dev)) {
		ret = PTR_ERR(bq->chg_dev);
		goto err_0;
	}

	
	ret = bq2429x_init_device(bq);
	if (ret) {
		pr_err("device init failure: %d\n", ret);
		goto err_0;
	}

	INIT_DELAYED_WORK(&bq->monitor_work, bq2429x_monitor_workfunc);

	ret = sysfs_create_group(&bq->dev->kobj, &bq2429x_attr_group);
	if (ret)
		dev_err(bq->dev, "failed to register sysfs. err: %d\n", ret);

	pr_info("BQ2429X charger driver probe successfully\n");

	return 0;

err_0:

	return ret;
}

static int bq2429x_charger_remove(struct i2c_client *client)
{
	struct bq2429x *bq = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&bq->monitor_work);

	mutex_destroy(&bq->i2c_rw_lock);

	sysfs_remove_group(&bq->dev->kobj, &bq2429x_attr_group);

	return 0;
}


static void bq2429x_charger_shutdown(struct i2c_client *client)
{
	pr_info("shutdown\n");

}

static struct of_device_id bq2429x_charger_match_table[] = {
	{.compatible = "ti,bq2429x"},
	{},
};


static const struct i2c_device_id bq2429x_charger_id[] = {
	{ "bq2429x", BQ24296 },
	{},
};

MODULE_DEVICE_TABLE(i2c, bq2429x_charger_id);

static struct i2c_driver bq2429x_charger_driver = {
	.driver		= {
		.name	= "bq2429x",
		.of_match_table = bq2429x_charger_match_table,
	},
	.id_table	= bq2429x_charger_id,

	.probe		= bq2429x_charger_probe,
	.remove		= bq2429x_charger_remove,
	.shutdown   = bq2429x_charger_shutdown,
};

module_i2c_driver(bq2429x_charger_driver);

MODULE_DESCRIPTION("TI BQ2415x Charger Driver");
MODULE_LICENSE("GPL2");
MODULE_AUTHOR("Texas Instruments");
