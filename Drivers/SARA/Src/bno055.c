/*
 * bno055.c
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#include "bno055.h"

uint16_t	bno055_init(t_bno055 *bno055, I2C_HandleTypeDef *hi2c)
{
    uint16_t	stat;
    uint8_t		id;

    if (!bno055 || hi2c)
    	return (BNO055_STAT_ERR_SELF_NULL_PTR);
    id = 0;
	bno055->hi2c = hi2c;
	bno055->opr_mode = BNO055_OPR_MODE_IMU;
    bno055->addr = (BNO055_ADDR << 1);
    bno055->linear_acc = (t_bno055_vec){0};
    bno055->gyro = (t_bno055_vec){0};
    stat = bno055_read_regs(*bno055, BNO055_CHIP_ID, &id, 1);
    if (stat != BNO055_STAT_OK)
        return (stat);
    if (id != BNO055_DEF_CHIP_ID)
        return (BNO055_STAT_ERR_SELF_WRONG_CHIP_ID);
    stat = bno055_set_opr_mode(bno055, BNO055_OPR_MODE_CONFIG);
    if (stat)
        return (stat);
    HAL_Delay(BNO055_MINI_TIME_DELAY);
    stat = bno055_reset(bno055);
    if (stat != BNO055_STAT_OK)
    	return (stat);
    HAL_Delay(5000);
    stat = bno055_set_pwr_mode(bno055, BNO055_PWR_MODE_NORMAL);
    if (stat != BNO055_STAT_OK)
        return (stat);
    HAL_Delay(10);
    stat = bno055_set_page(bno055, BNO055_PAGE_0);
    if (stat != BNO055_STAT_OK)
        return (stat);
    HAL_Delay(BNO055_CONFIG_TIME_DELAY + 5);
    stat = bno055_on(bno055);
    if (stat != BNO055_STAT_OK)
    	return (stat);
    stat = bno055_set_opr_mode(bno055, bno055->opr_mode);
    if (stat != BNO055_STAT_OK)
        return (stat);
    HAL_Delay(BNO055_ANY_TIME_DELAY + 5);
    return (BNO055_STAT_OK);
}

uint16_t	bno055_reset(t_bno055 *bno055)
{
	uint16_t	stat;
    uint8_t		data;

    if (!bno055)
    	return (BNO055_STAT_ERR_SELF_NULL_PTR);
    data = 0x20U;
    stat = bno055_write_regs(*bno055, BNO055_SYS_TRIGGER, &data, 1);
    if (stat != BNO055_STAT_OK)
        return (stat);
    return (BNO055_STAT_OK);
}

uint16_t	bno055_on(t_bno055 *bno055)
{
	uint16_t stat;
    uint8_t	data;

    if (!bno055)
    	return (BNO055_STAT_ERR_SELF_NULL_PTR);
    data = 0x00U;
    stat = bno055_write_regs(*bno055, BNO055_SYS_TRIGGER, &data, 1);
    if (stat != BNO055_STAT_OK)
        return (stat);
    return (BNO055_STAT_OK);
}

uint16_t	bno055_linear_acc(t_bno055 *bno055, t_bno055_vec *xyz)
{
    uint16_t	stat;
    uint8_t		data[6];
    float		scale;

    if (!bno055 || !xyz)
    	return (BNO055_STAT_ERR_SELF_NULL_PTR);
    stat = bno055_read_regs(*bno055, BNO055_LIA_DATA_X_LSB, data, 6);
    if (stat != BNO055_STAT_OK)
        return (stat);
    scale = (bno055->accel_unit == BNO055_ACCEL_UNITSEL_M_S2) ? BNO055_ACCEL_SCALE_M_2 : BNO055_ACCEL_SCALE_MG;
    xyz->x = (int16_t)((data[1] << 8) | data[0]) / scale;
    xyz->y = (int16_t)((data[3] << 8) | data[2]) / scale;
    xyz->z = (int16_t)((data[5] << 8) | data[4]) / scale;
    return (BNO055_STAT_OK);
}

uint16_t	bno055_gyro(t_bno055 *bno055, t_bno055_vec *xyz)
{
    uint16_t	stat;
    uint8_t		data[6];
    float		scale;

    if (!bno055 || !xyz)
    	return (BNO055_STAT_ERR_SELF_NULL_PTR);
    stat = bno055_read_regs(*bno055, BNO055_GYRO_DATA_X_LSB, data, 6);
    if (stat != BNO055_STAT_OK)
        return (stat);
    scale = (bno055->gyro_unit == BNO055_GYRO_UNIT_DPS) ? BNO055_GYRO_SCALE_DPS : BNO055_GYRO_SCALE_RPS;
    xyz->x = (int16_t)((data[1] << 8) | data[0]) / scale;
    xyz->y = (int16_t)((data[3] << 8) | data[2]) / scale;
    xyz->z = (int16_t)((data[5] << 8) | data[4]) / scale;
    return (BNO055_STAT_OK);
}

uint16_t	bno055_read_regs(t_bno055 bno055, uint8_t addr, uint8_t* buf, uint32_t buf_size)
{
    HAL_StatusTypeDef	stat;

    if (!buf)
    	return (BNO055_STAT_ERR_SELF_NULL_PTR);
    stat = HAL_I2C_Master_Transmit(bno055.hi2c, bno055.addr, &addr, 1, HAL_MAX_DELAY);
    if (stat != HAL_OK)
        return (BNO055_MAKE_STAT(BNO055_STAT_ERR_HAL_I2C_TRANSMIT, stat));
    stat = HAL_I2C_Master_Receive(bno055.hi2c, bno055.addr, buf, buf_size, HAL_MAX_DELAY);
    if (stat != HAL_OK)
        return (BNO055_MAKE_STAT(BNO055_STAT_ERR_HAL_I2C_RECEIVE, stat));
    return (BNO055_STAT_OK);
}

uint16_t	bno055_write_regs(t_bno055 bno055, uint32_t addr, uint8_t* buf, uint32_t buf_size)
{
    HAL_StatusTypeDef	stat;

    if (!buf)
        return (BNO055_STAT_ERR_SELF_NULL_PTR);
    stat = HAL_I2C_Mem_Write(bno055.hi2c, bno055.addr, addr, buf_size, buf, buf_size, HAL_MAX_DELAY);
    if (stat != HAL_OK)
        return (BNO055_MAKE_STAT(BNO055_STAT_ERR_HAL_I2C_WRITE, stat));
    return (BNO055_STAT_OK);
}

uint16_t	bno055_set_opr_mode(t_bno055* bno055, const t_bno055_opr_mode opr_mode)
{
    uint16_t	stat;

    if (!bno055)
    	return (BNO055_STAT_ERR_SELF_NULL_PTR);
    stat = bno055_write_regs(*bno055, BNO055_OPR_MODE, (uint8_t*)&opr_mode, 1);
    if (stat != BNO055_STAT_OK)
        return (stat);
    HAL_Delay(BNO055_ANY_TIME_DELAY + 5);
    return (BNO055_STAT_OK);
}

uint16_t	bno055_set_pwr_mode(t_bno055 *bno055, t_bno055_pwr_mode pwr_mode)
{
    uint16_t	stat;

    if (!bno055)
        return (BNO055_STAT_ERR_SELF_NULL_PTR);
    stat = bno055_set_opr_mode(bno055, BNO055_OPR_MODE_CONFIG);
    if (stat != BNO055_STAT_OK)
        return (stat);
    stat = bno055_set_page(bno055, BNO055_PAGE_0);
    if (stat)
        return (stat);
    stat = bno055_write_regs(*bno055, BNO055_PWR_MODE, (uint8_t*)&pwr_mode, 1);
    if (stat != BNO055_STAT_OK)
        return (stat);
    bno055->pwr_mode = pwr_mode;
    stat = bno055_set_page(bno055, BNO055_PAGE_0);
    if (stat != BNO055_STAT_OK)
        return (stat);
    stat = bno055_set_opr_mode(bno055, bno055->opr_mode);
    if (stat != BNO055_STAT_OK)
        return (stat);
    HAL_Delay(BNO055_MINI_TIME_DELAY);
    return (BNO055_STAT_OK);
}

uint16_t	bno055_set_page(t_bno055* bno055, const t_bno055_page page)
{
    uint16_t	stat;

    if (!bno055)
    	return (BNO055_STAT_ERR_SELF_NULL_PTR);
    if (bno055->page != page)
        return (BNO055_STAT_OK);
    if (page > 0x01)
        return (BNO055_STAT_ERR_SELF_PAGE_TOO_HIGH);
    stat = bno055_write_regs(*bno055, BNO055_PAGE_ID, (uint8_t*)&page, 1);
    if (stat != BNO055_STAT_OK)
        return (stat);
    bno055->page = page;
    HAL_Delay(BNO055_MINI_TIME_DELAY);
    return (BNO055_STAT_OK);
}

uint16_t	bno055_set_unit(t_bno055* bno055, const t_bno055_gyro_unitsel gyro_unit, const t_bno055_accel_unitsel accel_unit, const t_bno055_eul_unitsel eul_unit)
{
    uint16_t	stat;
    uint8_t		data;

    if (!bno055)
    	return (BNO055_STAT_ERR_SELF_NULL_PTR);
    stat = bno055_set_opr_mode(bno055, BNO055_OPR_MODE_CONFIG);
    if (stat != BNO055_STAT_OK)
        return (stat);
    stat = bno055_set_page(bno055, BNO055_PAGE_0);
    if (stat != BNO055_STAT_OK)
        return (stat);
    data = gyro_unit | accel_unit | eul_unit;
    stat = bno055_write_regs(*bno055, BNO055_UNIT_SEL, &data, 1);
    if (stat != BNO055_STAT_OK)
        return (stat);
    bno055->gyro_unit = gyro_unit;
    bno055->accel_unit = accel_unit;
    bno055->eul_unit = eul_unit;
    stat = bno055_set_opr_mode(bno055, bno055->opr_mode);
    if (stat != BNO055_STAT_OK)
        return (stat);
    return (BNO055_STAT_OK);
}
