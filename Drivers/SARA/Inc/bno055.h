/*
 * bno055.h
 *
 *  Created on: Jun 25, 2025
 *      Author: yzeybek
 */

#ifndef SARA_INC_BNO055_H_
#define SARA_INC_BNO055_H_

# include "stm32f4xx.h"

# define BNO055_ADDR (0x29)
# define BNO055_CHIP_ID (0x00)
# define BNO055_DEF_CHIP_ID (0xA0)
# define BNO055_SYS_TRIGGER (0x3F)

# define BNO055_ANY_TIME_DELAY 19
# define BNO055_MINI_TIME_DELAY 2
# define BNO055_CONFIG_TIME_DELAY 7

# define BNO055_PAGE_ID (0x07)
# define BNO055_OPR_MODE (0x3D)
# define BNO055_PWR_MODE (0x3E)

# define BNO055_UNIT_SEL (0x3B)
# define BNO055_GYRO_UNITSEL_OFFSET (0x01)
# define BNO055_EUL_UNITSEL_OFFSET (0x02)

# define BNO055_LIA_DATA_X_LSB (0x28)
# define BNO055_ACCEL_SCALE_M_2 100.0f
# define BNO055_ACCEL_SCALE_MG  1.0f

# define BNO055_GYRO_DATA_X_LSB (0x14)
# define BNO055_GYRO_SCALE_DPS 16.0f
# define BNO055_GYRO_SCALE_RPS 900.0f

# define BNO055_MAKE_STAT(base_stat, sub_stat) \
    (uint16_t)( \
       ( ((uint16_t)((sub_stat) & 0xFF)) << 8 ) \
     |   ((uint16_t)((base_stat) & 0xFF)) )

typedef enum e_bno055_stat
{
    BNO055_STAT_OK,
    BNO055_STAT_ERR_SELF_PAGE_TOO_HIGH,
    BNO055_STAT_ERR_SELF_SETTING_PAGE,
    BNO055_STAT_ERR_SELF_NULL_PTR,
    BNO055_STAT_ERR_SELF_AXIS_REMAP,
    BNO055_STAT_ERR_SELF_WRONG_CHIP_ID,
	BNO055_STAT_ERR_HAL_I2C_TRANSMIT,
	BNO055_STAT_ERR_HAL_I2C_RECEIVE,
	BNO055_STAT_ERR_HAL_I2C_WRITE,

}	t_bno055_stat;

typedef enum e_bno055_opr_mode
{
    BNO055_OPR_MODE_CONFIG,
    BNO055_OPR_MODE_IMU,

}	t_bno055_opr_mode;

typedef enum e_bno055_pwr_mode
{
    BNO055_PWR_MODE_NORMAL,
    BNO055_PWR_MODE_LOW,
    BNO055_PWR_MODE_SUSPEND,

}	t_bno055_pwr_mode;

typedef enum e_bno055_page
{
    BNO055_PAGE_0 = 0x00U,
    BNO055_PAGE_1 = 0x01U,

}	t_bno055_page;

typedef enum e_bno055_accel_unitsel
{
    BNO055_ACCEL_UNITSEL_M_S2,
    BNO055_ACCEL_UNITSEL_MG,

}	t_bno055_accel_unitsel;

typedef enum e_bno055_gyro_unitsel
{
    BNO055_GYRO_UNIT_DPS = (0 << BNO055_GYRO_UNITSEL_OFFSET),
    BNO055_GYRO_UNIT_RPS = (1 << BNO055_GYRO_UNITSEL_OFFSET),

}	t_bno055_gyro_unitsel;

typedef enum e_bno055_eul_unitsel
{
    BNO055_EUL_UNIT_DEG = (0 << BNO055_EUL_UNITSEL_OFFSET),
    BNO055_EUL_UNIT_RAD = (1 << BNO055_EUL_UNITSEL_OFFSET),

}	t_bno055_eul_unitsel;

typedef struct s_bno055_vec
{
    float	x;
    float	y;
    float	z;

}	t_bno055_vec;

typedef struct s_bno055
{
    I2C_HandleTypeDef		*hi2c;
    uint8_t					addr;
    t_bno055_stat			stat;
    t_bno055_opr_mode		opr_mode;
    t_bno055_pwr_mode		pwr_mode;
    t_bno055_page			page;
    t_bno055_accel_unitsel	accel_unit;
    t_bno055_gyro_unitsel	gyro_unit;
    t_bno055_eul_unitsel	eul_unit;
    t_bno055_vec			linear_acc;
    t_bno055_vec			gyro;

}	t_bno055;

uint16_t	bno055_init(t_bno055 *bno055, I2C_HandleTypeDef *hi2c);
uint16_t	bno055_reset(t_bno055 *bno055);
uint16_t	bno055_on(t_bno055 *bno055);
uint16_t	bno055_linear_acc(t_bno055 *bno055, t_bno055_vec *xyz);
uint16_t	bno055_gyro(t_bno055 *bno055, t_bno055_vec *xyz);
uint16_t	bno055_read_regs(t_bno055 bno055, uint8_t addr, uint8_t* buf, uint32_t buf_size);
uint16_t	bno055_write_regs(t_bno055 bno055, uint32_t addr, uint8_t* buf, uint32_t buf_size);
uint16_t	bno055_set_opr_mode(t_bno055* bno055, const t_bno055_opr_mode opr_mode);
uint16_t	bno055_set_pwr_mode(t_bno055 *bno055, t_bno055_pwr_mode pwr_mode);
uint16_t	bno055_set_page(t_bno055* bno055, const t_bno055_page page);
uint16_t	bno055_set_unit(t_bno055* bno055, const t_bno055_gyro_unitsel gyro_unit, const t_bno055_accel_unitsel accel_unit, const t_bno055_eul_unitsel eul_unit);

#endif /* SARA_INC_BNO055_H_ */
