/**
 * @file ak09916.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief AK09916 driver
 * @date 2023-03-17
 */

#include "ak09916.h"
#include "i2c.h"

#define read_reg_multi(reg, pData, size)        \
        HAL_I2C_Mem_Read(&hi2c1, AK09916_ADDR << 1, reg,       \
                         I2C_MEMADD_SIZE_8BIT, pData, size, 10)
#define write_reg_multi(reg, pData, size)     \
        HAL_I2C_Mem_Write(&hi2c1, AK09916_ADDR << 1, reg,      \
                         I2C_MEMADD_SIZE_8BIT, pData, size, 10)
#define delay_ms(t)     HAL_Delay(t)

/* AK09916 registers */
#define AK09916_REG_WIA1        0x00
#define AK09916_REG_WIA2        0x01
#define AK09916_REG_ST1         0x10
#define AK09916_REG_HXL         0x11
#define AK09916_REG_HXH         0x12
#define AK09916_REG_HYL         0x13
#define AK09916_REG_HYH         0x14
#define AK09916_REG_HZL         0x15
#define AK09916_REG_HZH         0x16
#define AK09916_REG_ST2         0x18
#define AK09916_REG_CNTL2       0x31
#define AK09916_REG_CNTL3       0x32

int ak09916_init(enum ak09916_mode mode)
{
        uint8_t val = 0x01;
        
        delay_ms(50);
        write_reg_multi(AK09916_REG_CNTL3, &val, 1); /* reset */
        read_reg_multi(AK09916_REG_WIA2, &val, 1); /* get who I am */
        if (val != 0x09)
                return 1;
        val = mode;
        write_reg_multi(AK09916_REG_CNTL2, &val, 1); /* set mode */
        delay_ms(1);
        
        return 0;
}

void ak09916_read_data(float *mx, float *my, float *mz)
{
        uint8_t data[8];
        int16_t *ptr = (int16_t *)data;

        read_reg_multi(AK09916_REG_ST1, data, 1);
        if (!(data[0] & 0x01)) /* check data ready */
                return;
        read_reg_multi(AK09916_REG_HXL, data, 8);
        if (data[7] & 0x08) /* check data overflow */
                return;
        *mx = (float)(*ptr++) * 0.15;
        *mx -= MAG_CX;
        *my = -(float)(*ptr++) * 0.15;
        *my -= MAG_CY;
        *mz = -(float)(*ptr++) * 0.15;
        *mz -= MAG_CZ;
}