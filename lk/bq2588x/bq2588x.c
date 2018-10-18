#include <platform/mt_typedefs.h>
#include <platform/mt_reg_base.h>
#include <platform/errno.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <platform/mtk_charger_intf.h>
#include <printf.h>

#if !defined(CONFIG_POWER_EXT)
#include <platform/upmu_common.h>
#endif
#include "bq2588x_reg.h"

#define BQ2588X_LK_DRV_VERSION "1.0.1_MTK"

/* ================= */
/* Internal variable */
/* ================= */


struct bq2588x_config {
	int prechg_ma;
	int chg_ma;
	int chg_mv;
	int ivl_mv;
	int icl_ma;

	int term_ma;
	int otg_mv;
	int otg_ma;
};


struct bq2588x {
	struct mtk_charger_info mchr_info;
	struct mt_i2c_t i2c;
	struct bq2588x_config cfg;
	int i2c_log_level;
};



static int bq2588x_write_reg(struct bq2588x *bq, u8 reg, u8 data)
{
	unsigned int ret = I2C_OK;
	unsigned char write_buf[2] = {reg, data};
	struct mt_i2c_t *i2c = &bq->i2c;

	ret = i2c_write(i2c, write_buf, 2);

	if (ret != I2C_OK)
		dprintf(CRITICAL,
			"%s: I2CW[0x%02X] = 0x%02X failed, code = %d\n",
			__func__, reg, data, ret);
	else
		dprintf(bq->i2c_log_level, "%s: I2CW[0x%02X] = 0x%02X\n",
			__func__, reg, data);

	return ret;
}
static int bq2588x_read_reg(struct bq2588x *bq, u8 reg, u8 *data)
{
	int ret = I2C_OK;
	u8 reg_data = reg;
	struct mt_i2c_t *i2c = &bq->i2c;

	ret = i2c_write_read(i2c, &reg_data, 1, 1);

	if (ret != I2C_OK)
		dprintf(CRITICAL, "%s: I2CR[0x%02X] failed, code = %d\n",
			__func__, reg, ret);
	else {
		dprintf(bq->i2c_log_level, "%s: I2CR[0x%02X] = 0x%02X\n",
			__func__, reg, reg_data);
		*data = reg_data;
	}

	return ret;
}

static int bq2588x_update_bits(struct bq2588x *bq, u8 reg,
					u8 mask, u8 data)
{
	int ret;
	u8 reg_data;

	ret = bq2588x_read_reg(bq, reg, &reg_data);
	if (ret != I2C_OK) {
		dprintf(CRITICAL, "Failed: reg=%02X, ret=%d\n", reg, ret);
		return ret;
	}

	reg_data &= ~mask;
	reg_data |= data & mask;

	ret = bq2588x_write_reg(bq, reg, reg_data);
	if (ret)
		dprintf(CRITICAL, "Failed: reg=%02X, ret=%d\n", reg, ret);

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

static int bq2588x_reset_watchdog_timer(struct bq2588x *bq)
{
	int ret;
	u8 reg_val;

	reg_val = BQ2588X_WDT_RESET << BQ2588X_WDT_RESET_SHIFT;

	ret = bq2588x_update_bits(bq, BQ2588X_REG_CHG_CTRL3,
				BQ2588X_WDT_RESET_MASK, reg_val);

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

static bool bq2588x_is_hw_exist(struct bq2588x *bq)
{
	int ret;
	u8 data;
	u8 partno, rev;

	ret = bq2588x_read_reg(bq, BQ2588X_REG_PART_NUM, &data);
	
	if (ret < 0)
		return false;
		
	partno = data & BQ2588X_PART_NO_MASK;
	partno >>= BQ2588X_PART_NO_SHIFT;

	rev = data & BQ2588X_REVISION_MASK;
	rev >>= BQ2588X_REVISION_SHIFT;

	dprintf(CRITICAL, "%s: chip PN:%d, rev:%d\n", __func__, partno,
		rev);
	bq->mchr_info.device_id = rev;

	return true;
}

static int bq2588x_init_device(struct bq2588x *bq)
{
	int ret;

	ret = bq2588x_set_prechg_current(bq, bq->cfg.prechg_ma);
	if (ret)
		dprintf(CRITICAL, "Failed to set prechg current, ret = %d\n", ret);

	ret = bq2588x_set_charge_voltage(bq, bq->cfg.chg_mv);
	if (ret)
		dprintf(CRITICAL, "Failed to set charge voltage, ret = %d\n", ret);

	ret = bq2588x_set_charge_current(bq, bq->cfg.chg_ma);
	if (ret)
		dprintf(CRITICAL, "Failed to set charge current, ret = %d\n", ret);

	ret = bq2588x_set_charge_volt_limit(bq, bq->cfg.ivl_mv);
	if (ret)
		dprintf(CRITICAL, "Failed to set input volt limit, ret = %d\n", ret);

	ret = bq2588x_set_input_current_limit(bq, bq->cfg.icl_ma);
	if (ret)
		dprintf(CRITICAL, "Failed to set input current limit, ret = %d\n", ret);

	ret = bq2588x_set_term_current(bq, bq->cfg.term_ma);
	if (ret)
		dprintf(CRITICAL, "Failed to set termination current, ret = %d\n", ret);

	ret = bq2588x_set_otg_volt_limit(bq, bq->cfg.otg_mv);
	if (ret)
		dprintf(CRITICAL, "Failed to set otg voltage, ret = %d\n", ret);

	ret = bq2588x_set_otg_current_limit(bq, bq->cfg.otg_ma);
	if (ret)
		dprintf(CRITICAL, "Failed to set otg current, ret = %d\n", ret);

	ret = bq2588x_set_watchdog_timer(bq, 0);
	if (ret)
		dprintf(CRITICAL, "Failed to set watchdog_timer, ret = %d\n", ret);

	ret = bq2588x_charge_enable(bq, true);
	if (ret) {
		dprintf(CRITICAL, "Failed to enable charger, ret = %d\n", ret);
	} else {
		dprintf(CRITICAL, "Charger Enabled Successfully!\n");
	}

	bq2588x_term_enable(bq, true);

	bq2588x_set_ilim_pin(bq, true);

	bq2588x_set_adc_scan_mode(bq, true);
	bq2588x_set_adc_scan_bits(bq, 15);
	bq2588x_enable_adc_scan(bq, true);

	return 0;
}

/* =========================================================== */
/* The following is implementation for interface of bq2588x */
/* =========================================================== */

static int bq2588x_dump_regs(struct mtk_charger_info *mchr_info)
{
	struct bq2588x *bq = (struct bq2588x *)mchr_info;
	int ret = 0;
	u8 reg = 0;
	u8 val = 0;

	dprintf(CRITICAL, "%s: enter\n", __func__);
	for(reg = 0x0; reg <= 0x14; reg++)
	{
		ret = bq2588x_read_reg(bq, reg, &val);
		if (ret < 0)
		{
			dprintf(CRITICAL, "%s: read reg=0x%x error\n", __func__,reg);
		}else{
			dprintf(CRITICAL, "%s: reg(0x%x)== 0x%x\n", __func__,reg,val);
		}
	}
	dprintf(CRITICAL, "%s: exit\n", __func__);
	return 0;
}
static int bq2588x_enable_charging(struct mtk_charger_info *mchr_info,
	bool enable)
{
	struct bq2588x *bq = (struct bq2588x *)mchr_info;

	return bq2588x_charge_enable(bq, enable);
}

static int bq2588x_set_ichg(struct mtk_charger_info *mchr_info, u32 ichg)
{
	struct bq2588x *bq = (struct bq2588x *)mchr_info;

	return bq2588x_set_charge_current(bq,ichg/1000); //uA-->mA
}

static int bq2588x_get_ichg(struct mtk_charger_info *mchr_info, u32 *ichg)
{
	struct bq2588x *bq = (struct bq2588x *)mchr_info;
	u8 reg_val;
	int ret;

	ret = bq2588x_read_reg(bq, BQ2588X_REG_CHARGE_CURRENT, &reg_val);
	if (!ret) {
		*ichg = (reg_val & BQ2588X_ICHG_MASK) >> BQ2588X_ICHG_SHIFT;
		*ichg = (*ichg) * BQ2588X_ICHG_LSB + BQ2588X_ICHG_BASE;
		*ichg = (*ichg) * 1000; //mA-->uA
	}

	return ret;
}
static int bq2588x_get_input_current(struct mtk_charger_info *mchr_info, u32 *aicr)
{
	struct bq2588x *bq = (struct bq2588x *)mchr_info;
	u8 reg_val;
	int icl;
	int ret;

	ret = bq2588x_read_reg(bq, BQ2588X_REG_IINDPM, &reg_val);
	if (!ret) {
		icl = (reg_val & BQ2588X_IINDPM_TH_MASK) >> BQ2588X_IINDPM_TH_SHIFT;
		icl = icl * BQ2588X_IINDPM_TH_LSB + BQ2588X_IINDPM_TH_BASE;
		*aicr = icl * 1000;
	}

	return ret;

}
static int bq2588x_set_input_current(struct mtk_charger_info *mchr_info, u32 aicr)
{
	struct bq2588x *bq = (struct bq2588x *)mchr_info;

	return bq2588x_set_input_current_limit(bq, aicr/1000);
}

static int bq2588x_set_mivr(struct mtk_charger_info *mchr_info, u32 mivr)
{

	struct bq2588x *bq = (struct bq2588x *)mchr_info;

	return bq2588x_set_input_volt_limit(bq, mivr/1000);

}
static struct mtk_charger_ops bq2588x_mchr_ops = {
	.dump_register = bq2588x_dump_regs,
	.enable_charging = bq2588x_enable_charging,
	.get_ichg = bq2588x_get_ichg,
	.set_ichg = bq2588x_set_ichg,
	.set_aicr = bq2588x_set_input_current,
	.set_mivr = bq2588x_set_mivr,
	.get_aicr = bq2588x_get_input_current,
};
/* Info of primary charger */
static struct bq2588x g_bq2588x = {
	.mchr_info = {
		.name = "primary_charger",
		.alias_name = "bq2588x",
		.device_id = -1,
		.mchr_ops = &bq2588x_mchr_ops,
	},
	.i2c = {
		.id = I2C1,
		.addr = 0x6B,
		.mode = ST_MODE,
		.speed = 100,
	},
	.cfg = {
		.prechg_ma = 150,
		.term_ma  = 150,
		.chg_ma = 2000,
		.chg_mv = 8400,
		.ivl_mv = 4400,
		.icl_ma = 3000,
		.otg_mv = 5100,
		.otg_ma = 2000,
	}
	.i2c_log_level = INFO,
};

int bq2588x_probe(void)
{
	int ret = 0;

	/* Check primary charger */
	if (bq2588x_is_hw_exist(&g_bq2588x)) {
		ret = bq2588x_init_device(&g_bq2588x);
		mtk_charger_set_info(&(g_bq2588x.mchr_info));
		dprintf(CRITICAL, "%s: %s\n", __func__,
                    BQ2588X_LK_DRV_VERSION);
	}

	return ret;
}
