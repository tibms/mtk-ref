
#include <platform/mt_typedefs.h>
#include <platform/mt_reg_base.h>
#include <platform/errno.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <platform/bq24157.h>
#include <platform/mtk_charger_intf.h>
#include <printf.h>

#if !defined(CONFIG_POWER_EXT)
#include <platform/upmu_common.h>
#endif

#define BQ2415X_LK_DRV_VERSION "1.0.0_MTK"

enum bq2415x_part_no {
	BQ24157 = 0x02,
};

struct bq2415x_config {
	int chg_mv;
	int chg_ma;

	int ivl_mv;
	int icl_ma;
	
	int safety_chg_mv;
	int safety_chg_ma;

	
	int iterm_ma;
	int batlow_mv;
	
	int sensor_mohm;
	
	bool enable_term;
};


struct struct bq2415x {
	struct mtk_charger_info mchr_info;
	struct mt_i2c_t i2c;
	struct bq2415x_config cfg;
	int i2c_log_level;
};


static int bq2415x_read_byte(struct bq2415x *bq, u8 reg, u8 *data)
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


static int bq2415x_write_byte(struct bq2415x *bq, u8 reg, u8 data)
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


static int bq2415x_update_bits(struct bq2415x *bq, u8 reg,
					u8 mask, u8 data)
{
	int ret = 0;
	u8 reg_data = 0;

	ret = bq2415x_read_byte(bq, reg, &reg_data);
	if (ret != I2C_OK)
		return ret;

	reg_data &= ~mask;
	reg_data |= (data & mask);
	ret = bq2415x_write_byte(bq, reg, reg_data);
	
	return ret;
}

static bool bq2415x_is_hw_exist(struct bq2415x *info)
{
	int ret;
	u8 data;
	u8 partno, rev;

	ret = bq2415x_read_byte(bq, BQ2415X_REG_03, &data);
	if (ret < 0)
			return 0;
	partno = (data & BQ2415X_PN_MASK) >> BQ2415X_PN_SHIFT;
	rev = (data & BQ2415X_REVISION_MASK) >> BQ2415X_REVISION_SHIFT;
	if (partno != BQ24157) {
		dprintf(CRITICAL, "%s: incorrect part number, not bq24157\n");
		return false;
	}
	dprintf(CRITICAL, "%s: chip PN:%d, rev:%d\n", __func__, partno,
		rev);
	bq->mchr_info.device_id = rev;

	return true;	
}

static int bq2415x_set_safety_reg(struct bq2415x *bq, int volt, int curr)
{
	u8 ichg;
	u8 vchg;
	u8 val;
	
	ichg = (curr * bq->cfg.sensor_mohm / 100 -  BQ2415X_MAX_ICHG_BASE) / BQ2415X_MAX_ICHG_LSB;

	ichg <<= BQ2415X_MAX_ICHG_SHIFT;
	
	vchg = (volt - BQ2415X_MAX_VREG_BASE) / BQ2415X_MAX_VREG_LSB;
	vchg <<= BQ2415X_MAX_VREG_SHIFT;
	
	val = ichg | vchg;
	
	return bq2415x_update_bits(bq, BQ2415X_REG_06,
				BQ2415X_MAX_VREG_MASK | BQ2415X_MAX_ICHG_MASK, val);	
	
}

static int bq2415x_set_battery_voreg(struct bq2415x *bq, u32 voreg)
{
	int ret = 0;
	u8 reg_voreg = 0;

	voreg /= 1000; /*to mV*/
	reg_voreg = (volt - BQ2415X_VREG_BASE)/BQ2415X_VREG_LSB;
	reg_voreg <<= BQ2415X_VREG_SHIFT;
	
	dprintf(CRITICAL, "%s: voreg = %d(0x%02X)\n", __func__, voreg, reg_voreg);
		
	return bq2415x_update_bits(bq, BQ2415X_REG_02,
				BQ2415X_VREG_MASK, reg_voreg);
}


static int bq2415x_enable_term(struct bq2415x *bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2415X_TERM_ENABLE;
	else
		val = BQ2415X_TERM_DISABLE;

	val <<= BQ2415X_TERM_ENABLE_SHIFT;

	ret = bq2415x_update_bits(bq, BQ2425X_REG_01,
				BQ2415X_TERM_ENABLE_MASK, val);

	return ret;
}

static int bq2415x_init_setting(struct bq2415x *bq)
{
	int ret = 0;

	dprintf(CRITICAL, "%s\n", __func__);
	/*safety register must be written before any other register is set*/
	ret = bq2415x_set_safety_reg(bq, bq->cfg.safety_chg_mv, bq->cfg.safety_chg_ma);
	if (ret < 0)
		dprintf(CRITICAL,"Failed to set safety register:%d\n", ret);
	
	ret = bq2415x_set_battery_voreg(bq, bq->cfg.chg_mv * 1000);
	if (ret < 0)
		dprintf(CRITICAL, "%s: set chargevolt failed\n", __func__);
	
	ret = bq2415x_set_ichg(&bq->mchr_info, bq->cfg.chg_ma * 1000);
	if (ret < 0)
		dprintf(CRITICAL, "%s: set ichg failed\n", __func__);

	ret = bq2415x_set_aicr(&bq->mchr_info, bq->cfg.icl_ma * 1000);
	if (ret < 0)
		dprintf(CRITICAL, "%s: set aicr failed\n", __func__);

	ret = bq2415x_set_mivr(&bq->mchr_info, bq->cfg.ivl_mv * 1000);
	if (ret < 0)
		dprintf(CRITICAL, "%s: set mivr failed\n", __func__);
	
	ret = bq2415x_enable_term(bq, true);
	if (ret < 0)
		dprintf(CRITICAL, "%s: failed to enable termination\n", __func__);
	
}


static int bq2415x_enable_charger(struct bq2415x *bq)
{
	int ret;
	u8 val = BQ2415X_CHARGE_ENABLE << BQ2415X_CHARGE_ENABLE_SHIFT;

	ret = bq2415x_update_bits(bq, BQ2425X_REG_01,
				BQ2415X_CHARGE_ENABLE_MASK, val);
	return ret;
}

static int bq2415x_disable_charger(struct bq2415x *bq)
{
	int ret;
	u8 val = BQ2415X_CHARGE_DISABLE << BQ2415X_CHARGE_ENABLE_SHIFT;

	ret = bq2415x_update_bits(bq, BQ2425X_REG_01,
				BQ2415X_CHARGE_ENABLE_MASK, val);

	return ret;
}

/* =========================================================== */
/* The following is implementation for interface of bq2415x */
/* =========================================================== */

static int bq2415x_dump_register(struct mtk_charger_info *mchr_info)
{
	struct bq2415x *bq = (struct bq2415x *)mchr_info;
	int ret;
	u8 addr;
	u8 val;

	for (addr = 0x00; addr <= 0x06; addr++) {
		msleep(2);
		ret = bq2415x_read_byte(bq, addr, &val);
		if (!ret)
			dprintf(CRITICAL,"%s:Reg[%02X] = 0x%02X\n", __func__, addr, val);
	}

}

static int bq2415x_enable_charging(struct mtk_charger_info *mchr_info,
	bool enable)
{
	int ret = 0;
	struct bq2415x *bq = (struct bq2415x *)mchr_info;

	dprintf(CRITICAL, "%s: enable = %d\n", __func__, enable);	

	if (enable)
		ret = bq2415x_enable_charger(bq);
	else
		ret = bq2415x_disable_charger(bq);
	
	return ret;
}


static int bq2415x_set_ichg(struct mtk_charger_info *mchr_info, u32 ichg)
{
	int ret = 0;
	struct bq2415x *bq = (struct bq2415x *)mchr_info;
	
	u8 reg_ichg;
	
	ichg /= 1000; /*to mA */
	
	reg_ichg = (curr * bq->cfg.sensor_mohm / 100 -  BQ2415X_ICHG_BASE) / BQ2415X_ICHG_LSB;

	reg_ichg <<= BQ2415X_ICHG_SHIFT;
	
	return bq2415x_update_bits(bq, BQ2415X_REG_04,
				BQ2415X_ICHG_MASK, reg_ichg);
	
	
}

static int bq2415x_get_ichg(struct mtk_charger_info *mchr_info, u32 *ichg)
{
	int ret = 0;
	u8 val = 0;
	int curr;
	struct bq2415x *bq = (struct bq2415x *)mchr_info;

		
	ret = bq2415x_read_byte(bq, BQ2415X_REG_04, &val);
	if (!ret) {
		curr = ((u32)(val & BQ2415X_ICHG_MASK ) >> BQ2415X_ICHG_SHIFT) * BQ2415X_ICHG_LSB;
		curr +=  BQ2415X_ICHG_BASE;
		curr = curr * 100 / bq->cfg.sensor_mohm;
		
		*ichg = curr * 1000; /*to uA*/
	}
	
	return ret;
}

static int bq2415x_set_aicr(struct mtk_charger_info *mchr_info, u32 curr)
{
	struct bq2415x *bq = (struct bq2415x *)mchr_info;

	u8 val;
	
	curr /= 1000;/*to mA*/
	if (curr == 100)
		val = BQ2415X_IINLIM_100MA;
	else if (curr == 500)
		val = BQ2415X_IINLIM_500MA;
	else if (curr == 800)
		val = BQ2415X_IINLIM_800MA;
	else if (curr == 0)
		val = BQ2415X_IINLIM_NOLIM;
	
	val <<= BQ2415X_IINLIM_SHIFT;

		
	return bq2415x_update_bits(bq, BQ2425X_REG_01,
				BQ2415X_IINLIM_MASK, val);	
}


static int bq2415x_get_aicr(struct mtk_charger_info *mchr_info, u32 *curr)
{
	struct bq2415x *bq = (struct rt9458_info *)mchr_info;
	int ret = 0;
	u8 val;
	int ilim;
	
	ret = bq2415x_read_byte(bq, BQ2425X_REG_01, &val);
	if (!ret) {
		val = val & BQ2415X_IINLIM_MASK;
		val = val >> BQ2415X_IINLIM_SHIFT;
		if (val == BQ2415X_IINLIM_100MA)
			ilim = 100;
		else if (val == BQ2415X_IINLIM_500MA)
			ilim = 500;
		else if (val == BQ2415X_IINLIM_800MA)
			ilim = 800;
		else
			ilim = 0;
		
		*curr = ilim * 1000; /*to uA*/
	}
	
	return ret;		
}


static int bq2415x_set_mivr(struct mtk_charger_info *mchr_info, u32 mivr)
{
	u8 reg_mivr = 0;
	struct bq2415x *bq = (struct bq2415x *)mchr_info;


	mivr /= 1000; /*to mV*/
	reg_mivr = (mivr - BQ2415X_VSREG_BASE) / BQ2415X_VSREG_LSB;
	reg_mivr <<= BQ2415X_VSREG_SHIFT;

	dprintf(CRITICAL, "%s: mivr = %d(0x%02X)\n", __func__, mivr, reg_mivr);
	
	return bq2415x_update_bits(bq, BQ2415X_REG_06,
				BQ2415X_VSREG_MASK, reg_mivr);
}	
}

static struct mtk_charger_ops bq2415x_mchr_ops = {
	.dump_register = bq2415x_dump_register,
	.enable_charging = bq2415x_enable_charging,
	.get_ichg = bq2415x_get_ichg,
	.set_ichg = bq2415x_set_ichg,
	.set_aicr = bq2415x_set_aicr,
	.get_aicr = bq2415x_get_aicr,
	.set_mivr = bq2415x_set_mivr,
};


/* Info of primary charger */
static struct bq2415x g_bq2415x = {
	.mchr_info = {
		.name = "primary_charger",
		.alias_name = "bq24157",
		.device_id = -1,
		.mchr_ops = &bq2415x_mchr_ops,
	},
	.i2c = {
		.id = I2C1,
		.addr = 0x6A,
		.mode = ST_MODE,
		.speed = 100,
	},
	.cfg = {
		.chg_ma = 1550,
		.chg_mv = 4350,
		.ivl_mv = 4500,
		.icl_ma = 500,
		.sensor_mohm = 55,
		.safety_chg_ma = 1550,
		.safety_chg_mv = 4400,
	},
	.i2c_log_level = INFO,
};

int bq2415x_probe(void)
{
	int ret = 0;

	
	/* Check primary charger */
	if (bq2415x_is_hw_exist(&g_bq2415x)) {
		ret = bq2415x_init_setting(&g_bq2415x);
		mtk_charger_set_info(&(g_bq2415x.mchr_info));
		dprintf(CRITICAL, "%s: %s\n", __func__, BQ2415X_LK_DRV_VERSION);
	}

	return ret;
}
