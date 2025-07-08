/*
 * bno055.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "bno055.h"

t_bno055_err	bno055_init(t_bno055 *bno055)
{
    uint8_t			id;
    t_bno055_err	err;

    id = 0;
    bno055->addr = (bno055->addr << 1);
    bno055->linear_acc = (t_bno055_vec){0};
    bno055->gyro = (t_bno055_vec){0};
    err = bno055_read_regs(*bno055, BNO055_CHIP_ID, &id, 1);
    if (err != BNO055_OK)
        return (err);
    if (id != BNO055_DEF_CHIP_ID)
        return (BNO055_ERR_WRONG_CHIP_ID);
    if ((err = bno055_set_opr_mode(bno055, BNO055_OPR_MODE_CONFIG)) != BNO055_OK)
        return (err);
    HAL_Delay(BNO055_MINI_TIME_DELAY);
    bno055_reset(bno055);
    HAL_Delay(5000);
    if ((err = bno055_set_pwr_mode(bno055, BNO055_PWR_MODE_NORMAL)) != BNO055_OK)
        return (err);
    HAL_Delay(10);
    if ((err = bno055_set_page(bno055, BNO055_PAGE_0)) != BNO055_OK)
        return (err);
    HAL_Delay(BNO055_CONFIG_TIME_DELAY + 5);
    bno055_on(bno055);
    if ((err = bno055_set_opr_mode(bno055, bno055->opr_mode)) != BNO055_OK)
        return (err);
    HAL_Delay(BNO055_ANY_TIME_DELAY + 5);
    return (BNO055_OK);
}

t_bno055_err	bno055_reset(t_bno055 *bno055)
{
    uint8_t	data;

    data = 0x20U;
    if (bno055_write_regs(*bno055, BNO055_SYS_TRIGGER, &data, 1) != BNO055_OK)
        return (BNO055_ERR_I2C);
    return (BNO055_OK);
}

t_bno055_err	bno055_on(t_bno055 *bno055)
{
    uint8_t	data;

    data = 0x00U;
    if (bno055_write_regs(*bno055, BNO055_SYS_TRIGGER, &data, 1) != BNO055_OK)
        return (BNO055_ERR_I2C);
    return (BNO055_OK);
}

t_bno055_err	bno055_linear_acc(t_bno055 *bno055, t_bno055_vec *xyz)
{
    t_bno055_err	err;
    uint8_t			data[6];
    float			scale;

    if ((err = bno055_read_regs(*bno055, BNO055_LIA_DATA_X_LSB, data, 6)) != BNO055_OK)
        return (err);
    scale = (bno055->accel_unit == BNO055_ACCEL_UNITSEL_M_S2) ? BNO055_ACCEL_SCALE_M_2 : BNO055_ACCEL_SCALE_MG;
    xyz->x = (int16_t)((data[1] << 8) | data[0]) / scale;
    xyz->y = (int16_t)((data[3] << 8) | data[2]) / scale;
    xyz->z = (int16_t)((data[5] << 8) | data[4]) / scale;
    return (BNO055_OK);
};

t_bno055_err	bno055_gyro(t_bno055 *bno055, t_bno055_vec *xyz)
{
    t_bno055_err	err;
    uint8_t			data[6];
    float			scale;

    if ((err = bno055_read_regs(*bno055, BNO055_GYRO_DATA_X_LSB, data, 6)) != BNO055_OK)
        return (err);
    scale = (bno055->gyro_unit == BNO055_GYRO_UNIT_DPS) ? BNO055_GYRO_SCALE_DPS : BNO055_GYRO_SCALE_RPS;
    xyz->x = (int16_t)((data[1] << 8) | data[0]) / scale;
    xyz->y = (int16_t)((data[3] << 8) | data[2]) / scale;
    xyz->z = (int16_t)((data[5] << 8) | data[4]) / scale;
    return (BNO055_OK);
}

t_bno055_err	bno055_read_regs(t_bno055 bno055, uint8_t addr, uint8_t* buf, uint32_t buf_size)
{
    HAL_StatusTypeDef	err;

    err = HAL_I2C_Master_Transmit(bno055.i2c, bno055.addr, &addr, 1, HAL_MAX_DELAY);
    if (err != HAL_OK)
        return (BNO055_ERR_I2C);
    err = HAL_I2C_Master_Receive(bno055.i2c, bno055.addr, buf, buf_size, HAL_MAX_DELAY);
    if (err != HAL_OK)
        return (BNO055_ERR_I2C);
    return (BNO055_OK);
}

t_bno055_err	bno055_write_regs(t_bno055 bno055, uint32_t addr, uint8_t* buf, uint32_t buf_size)
{
    HAL_StatusTypeDef	err;

    err = HAL_I2C_Mem_Write(bno055.i2c, bno055.addr, addr, buf_size, buf, buf_size, HAL_MAX_DELAY);
    if (err != HAL_OK)
        return (BNO055_ERR_I2C);
    return (BNO055_OK);
}

t_bno055_err	bno055_set_opr_mode(t_bno055* bno055, const t_bno055_opr_mode opr_mode)
{
    t_bno055_err	err;

    if ((err = bno055_write_regs(*bno055, BNO055_OPR_MODE, (uint8_t*)&opr_mode, 1)) != BNO055_OK)
        return (err);
    HAL_Delay(BNO055_ANY_TIME_DELAY + 5);
    return (BNO055_OK);
}

t_bno055_err	bno055_set_pwr_mode(t_bno055 *bno055, t_bno055_pwr_mode pwr_mode)
{
    t_bno055_err	err;

    if (!bno055)
        return (BNO055_ERR_NULL_PTR);
    if ((err = bno055_set_opr_mode(bno055, BNO055_OPR_MODE_CONFIG)) != BNO055_OK)
        return (err);
    if ((err = bno055_set_page(bno055, BNO055_PAGE_0)) != BNO055_OK)
        return (err);
    if ((err = bno055_write_regs(*bno055, BNO055_PWR_MODE, (uint8_t*)&pwr_mode, 1)) != BNO055_OK)
        return (err);
    bno055->pwr_mode = pwr_mode;
    if ((err = bno055_set_page(bno055, BNO055_PAGE_0)) != BNO055_OK)
        return (err);
    if ((err = bno055_set_opr_mode(bno055, bno055->opr_mode)) != BNO055_OK)
        return (err);
    HAL_Delay(BNO055_MINI_TIME_DELAY);
    return (BNO055_OK);
}

t_bno055_err	bno055_set_page(t_bno055* bno055, const t_bno055_page page)
{
    t_bno055_err	err;

    if (bno055->page != page)
        return (BNO055_OK);
    if (page > 0x01)
        return (BNO055_ERR_PAGE_TOO_HIGH);
    err = bno055_write_regs(*bno055, BNO055_PAGE_ID, (uint8_t*)&page, 1);
    if (err != BNO055_OK)
        return (err);
    bno055->page = page;
    HAL_Delay(BNO055_MINI_TIME_DELAY);
    return (BNO055_OK);
}

t_bno055_err	bno055_set_unit(t_bno055* bno055, const t_bno055_gyro_unitsel gyro_unit, const t_bno055_accel_unitsel accel_unit, const t_bno055_eul_unitsel eul_unit)
{
    t_bno055_err	err;
    uint8_t			data;

    if ((err = bno055_set_opr_mode(bno055, BNO055_OPR_MODE_CONFIG)) != BNO055_OK)
        return (err);
    if ((err = bno055_set_page(bno055, BNO055_PAGE_0)) != BNO055_OK)
        return (err);
    data = gyro_unit | accel_unit | eul_unit;
    if ((err = bno055_write_regs(*bno055, BNO055_UNIT_SEL, &data, 1)) != BNO055_OK)
        return (err);
    bno055->gyro_unit = gyro_unit;
    bno055->accel_unit = accel_unit;
    bno055->eul_unit = eul_unit;
    if ((err = bno055_set_opr_mode(bno055, bno055->opr_mode)) != BNO055_OK)
        return (err);
    return (BNO055_OK);
}

char	*bno055_err_str(const t_bno055_err err)
{
    switch (err)
    {
        case BNO055_OK:
            return ("[BNO055] Ok!");
        case BNO055_ERR_I2C:
            return ("[BNO055] I2C error!");
        case BNO055_ERR_PAGE_TOO_HIGH:
            return ("[BNO055] Page setting to high.");
        case BNO055_ERR_NULL_PTR:
            return ("[BNO055] BNO055 struct is nullpointer.");
        case BNO055_ERR_AXIS_REMAP:
            return ("[BNO055] Axis remap error!");
        case BNO055_ERR_SETTING_PAGE:
            return ("[BNO055] TODO");
        case BNO055_ERR_WRONG_CHIP_ID:
            return ("[BNO055] Wrong Chip ID.");
    }
    return ("[BNO] Ok!");
}
