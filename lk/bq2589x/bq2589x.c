#include <platform/mt_typedefs.h>
#include <platform/mt_reg_base.h>
#include <platform/errno.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <platform/mtk_charger_intf.h>
#include <printf.h>

#define __BQ25890H__	1
#include "bq2589x_reg.h"

#if !defined(CONFIG_POWER_EXT)
#include <platform/upmu_common.h>
#endif

#define BQ2589X_LK_DRV_VERSION "1.0.0_MTK"

enum bq2589x_part_no {
	BQ25890H = 0x03,
};

struct bq2589x_config {
	int chg_mv;
	int chg_ma;

	int ivl_mv;
	int icl_ma;
	
	int iterm_ma;
	
	bool enable_term;
};


struct bq2589x {
	struct mtk_charger_info mchr_info;
	struct mt_i2c_t i2c;
	struct bq2589x_config cfg;
	int i2c_log_level;
};


static int bq2589x_read_byte(struct bq2589x *bq, u8 reg, u8 *data)
{
	int ret = I2C_OK;
	u8 ret_data = reg;
	struct mt_i2c_t *i2c = &bq->i2c;

	ret = i2c_write_read(i2c, &ret_data, 1, 1);

	if (ret != I2C_OK)
		dprintf(CRITICAL, "%s: I2CR[0x%02X] failed, code = %d\n",
			__func__, reg, ret);
	else {
		dprintf(bq->i2c_log_level, "%s: I2CR[0x%02X] = 0x%02X\n",
			__func__, reg, ret_data);
		*data = ret_data;
	}

	return ret;
}


static int bq2589x_write_byte(struct bq2589x *bq, u8 reg, u8 data)
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


static int bq2589x_update_bits(struct bq2589x *bq, u8 reg,
					u8 mask, u8 data)
{
	int ret = 0;
	u8 reg_data = 0;

	ret = bq2589x_read_byte(bq, reg, &reg_data);
	if (ret != I2C_OK)
		return ret;

	reg_data &= ~mask;
	reg_data |= (data & mask);
	ret = bq2589x_write_byte(bq, reg, reg_data);
	
	return ret;
}

static bool bq2589x_is_hw_exist(struct bq2589x *bq)
{
	int ret;
	u8 data;
	u8 partno, rev;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_14, &data);
	if (ret < 0)
			return 0;
	partno = (data & BQ2589X_PN_MASK) >> BQ2589X_PN_SHIFT;
	rev = (data & BQ2589X_DEV_REV_MASK) >> BQ2589X_DEV_REV_SHIFT;
	if (partno != BQ25890H) {
		dprintf(CRITICAL, "%s: incorrect part number, not bq2589x\n", partno);
		return false;
	}
	dprintf(CRITICAL, "%s: chip PN:%d, rev:%d\n", __func__, partno,
		rev);
	bq->mchr_info.device_id = rev;

	return true;	
}

static int bq2589x_enable_term(struct bq2589x *bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_TERM_ENABLE;
	else
		val = BQ2589X_TERM_DISABLE;

	val <<= BQ2589X_EN_TERM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_07,
				BQ2589X_EN_TERM_MASK, val);

	return ret;
}


static int bq2589x_enable_charger(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_CHG_ENABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_03,
				BQ2589X_CHG_CONFIG_MASK, val);
	return ret;
}

static int bq2589x_disable_charger(struct bq2589x *bq)
{
	int ret;

	u8 val = BQ2589X_CHG_DISABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_03,
				BQ2589X_CHG_CONFIG_MASK, val);

	return ret;
}

/* =========================================================== */
/* The following is implementation for interface of bq2589x */
/* =========================================================== */

static int bq2589x_dump_register(struct mtk_charger_info *mchr_info)
{
	struct bq2589x *bq = (struct bq2589x *)mchr_info;
	int ret;
	u8 addr;
	u8 val;

	for (addr = 0x00; addr <= 0x14; addr++) {
		mdelay(2);
		ret = bq2589x_read_byte(bq, addr, &val);
		if (!ret)
			dprintf(CRITICAL,"%s:Reg[%02X] = 0x%02X\n", __func__, addr, val);
	}
	
	return ret;
}

static int bq2589x_enable_charging(struct mtk_charger_info *mchr_info,
	bool enable)
{
	int ret = 0;
	struct bq2589x *bq = (struct bq2589x *)mchr_info;

	dprintf(CRITICAL, "%s: enable = %d\n", __func__, enable);	

	if (enable)
		ret = bq2589x_enable_charger(bq);
	else
		ret = bq2589x_disable_charger(bq);
	
	return ret;
}

static int bq2589x_set_vchg(struct mtk_charger_info *mchr_info, u32 vchg)
{
	int ret = 0;
	struct bq2589x *bq = (struct bq2589x *)mchr_info;
	u8 reg_vchg;

	vchg /= 1000; /* to mV */

	if (vchg < BQ2589X_VREG_BASE)
		vchg = BQ2589X_VREG_BASE;
	
	reg_vchg = (vchg - BQ2589X_VREG_BASE) / BQ2589X_VREG_LSB;
	reg_vchg <<= BQ2589X_VREG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_06,
				BQ2589X_VREG_MASK, reg_vchg);
}


static int bq2589x_set_ichg(struct mtk_charger_info *mchr_info, u32 ichg)
{
	int ret = 0;
	struct bq2589x *bq = (struct bq2589x *)mchr_info;
	
	u8 reg_ichg;
	
	ichg /= 1000; /*to mA */
#if 0
	if (ichg < REG02_ICHG_BASE)
		ichg = REG02_ICHG_BASE;
#endif	
	reg_ichg = (ichg - BQ2589X_ICHG_BASE) / BQ2589X_ICHG_LSB;

	reg_ichg <<= BQ2589X_ICHG_SHIFT;
	
	return bq2589x_update_bits(bq, BQ2589X_REG_04,
				BQ2589X_ICHG_MASK, reg_ichg);
	
}

static int bq2589x_get_ichg(struct mtk_charger_info *mchr_info, u32 *ichg)
{
	int ret = 0;
	u8 val = 0;
	int curr;
	struct bq2589x *bq = (struct bq2589x *)mchr_info;

		
	ret = bq2589x_read_byte(bq, BQ2589X_REG_04, &val);
	if (!ret) {
		curr = ((u32)(val & BQ2589X_ICHG_MASK ) >> BQ2589X_ICHG_SHIFT) * BQ2589X_ICHG_LSB;
		curr +=  BQ2589X_ICHG_BASE;
		
		*ichg = curr * 1000; /*to uA*/
	}
	
	return ret;
}

static int bq2589x_set_aicr(struct mtk_charger_info *mchr_info, u32 curr)
{
	struct bq2589x *bq = (struct bq2589x *)mchr_info;

	u8 val;
	
	curr /= 1000;/*to mA*/
	
	if (curr < BQ2589X_IINLIM_BASE)
		curr = BQ2589X_IINLIM_BASE;

	val = (curr - BQ2589X_IINLIM_BASE) /  BQ2589X_IINLIM_LSB;
	val <<= BQ2589X_IINLIM_SHIFT;
		
	return bq2589x_update_bits(bq, BQ2589X_REG_00,
				BQ2589X_IINLIM_MASK, val);	
}


static int bq2589x_get_aicr(struct mtk_charger_info *mchr_info, u32 *curr)
{
	struct bq2589x *bq = (struct bq2589x *)mchr_info;
	int ret = 0;
	u8 val;
	int ilim;
	
	ret = bq2589x_read_byte(bq, BQ2589X_REG_00, &val);
	if (!ret) {
		val = val & BQ2589X_IINLIM_MASK;
		val = val >> BQ2589X_IINLIM_SHIFT;
		ilim = val * BQ2589X_IINLIM_LSB + BQ2589X_IINLIM_BASE;	
		*curr = ilim * 1000; /*to uA*/
	}
	
	return ret;		
}


static int bq2589x_set_mivr(struct mtk_charger_info *mchr_info, u32 mivr)
{
	u8 reg_mivr = 0;
	struct bq2589x *bq = (struct bq2589x *)mchr_info;


	mivr /= 1000; /*to mV*/

	if (mivr < BQ2589X_VINDPM_BASE)
		mivr = BQ2589X_VINDPM_BASE;
		
	reg_mivr = (mivr - BQ2589X_VINDPM_BASE) / BQ2589X_VINDPM_LSB;
	reg_mivr <<= BQ2589X_VINDPM_SHIFT;

	dprintf(CRITICAL, "%s: mivr = %d(0x%02X)\n", __func__, mivr, reg_mivr);
	
	return bq2589x_update_bits(bq, BQ2589X_REG_0D,
				BQ2589X_VINDPM_MASK, reg_mivr);

}

static int bq2589x_init_setting(struct bq2589x *bq)
{
	int ret = 0;

	dprintf(CRITICAL, "%s\n", __func__);
	
	ret = bq2589x_set_vchg(bq, bq->cfg.chg_mv * 1000);
	if (ret < 0)
		dprintf(CRITICAL, "%s: set chargevolt failed\n", __func__);
	
	ret = bq2589x_set_ichg(&bq->mchr_info, bq->cfg.chg_ma * 1000);
	if (ret < 0)
		dprintf(CRITICAL, "%s: set ichg failed\n", __func__);

	ret = bq2589x_set_aicr(&bq->mchr_info, bq->cfg.icl_ma * 1000);
	if (ret < 0)
		dprintf(CRITICAL, "%s: set aicr failed\n", __func__);

	ret = bq2589x_set_mivr(&bq->mchr_info, bq->cfg.ivl_mv * 1000);
	if (ret < 0)
		dprintf(CRITICAL, "%s: set mivr failed\n", __func__);
	
	ret = bq2589x_enable_term(bq, true);
	if (ret < 0)
		dprintf(CRITICAL, "%s: failed to enable termination\n", __func__);

	return ret;
}


static struct mtk_charger_ops bq2589x_mchr_ops = {
	.dump_register = bq2589x_dump_register,
	.enable_charging = bq2589x_enable_charging,
	.get_ichg = bq2589x_get_ichg,
	.set_ichg = bq2589x_set_ichg,
	.set_aicr = bq2589x_set_aicr,
	.get_aicr = bq2589x_get_aicr,
	.set_mivr = bq2589x_set_mivr,
};


/* Info of primary charger */
static struct bq2589x g_bq2589x = {
	.mchr_info = {
		.name = "primary_charger",
		.alias_name = "bq2589x",
		.device_id = -1,
		.mchr_ops = &bq2589x_mchr_ops,
	},
	.i2c = {
		.id = I2C1,
		.addr = 0x6B,
		.mode = ST_MODE,
		.speed = 100,
	},
	.cfg = {
		.chg_ma = 2000,
		.chg_mv = 4400,
		.ivl_mv = 4500,
		.icl_ma = 1000,
	},
	.i2c_log_level = INFO,
};

int bq2589x_probe(void)
{
	int ret = 0;

	
	/* Check primary charger */
	if (bq2589x_is_hw_exist(&g_bq2589x)) {
		ret = bq2589x_init_setting(&g_bq2589x);
		mtk_charger_set_info(&(g_bq2589x.mchr_info));
		dprintf(CRITICAL, "%s: %s\n", __func__, BQ2589X_LK_DRV_VERSION);
	}

	return ret;
}
