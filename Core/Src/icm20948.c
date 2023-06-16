/**
 * @file icm20948.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief ICM20948 driver
 * @date 2023-03-15
 */

#include "icm20948.h"
#include "ak09916.h"
#include "i2c.h"

#define read_reg_multi(reg, pData, size)      \
        HAL_I2C_Mem_Read(&hi2c1, ICM20948_ADDR << 1, reg,       \
                         I2C_MEMADD_SIZE_8BIT, pData, size, 10)
#define write_reg_multi(reg, pData, size)     \
        HAL_I2C_Mem_Write(&hi2c1, ICM20948_ADDR << 1, reg,      \
                         I2C_MEMADD_SIZE_8BIT, pData, size, 10)
#define delay_ms(t)     HAL_Delay(t)

/* static variable */
static float accel_unit;
static float gyro_unit;

#define REG_BANK_SEL            0x7F

/* bank 0 registers */
#define REG_B0_WHO_AM_I         0x00
#define REG_B0_USER_CTRL        0x03
#define REG_B0_LP_CONFIG        0x05
#define REG_B0_PWR_MGMT_1       0x06
#define REG_B0_PWR_MGMT_2       0x07
#define REG_B0_INT_PIN_CFG      0x0F
#define REG_B0_INT_ENABLE       0x10
#define REG_B0_INT_ENABLE_1     0x11
#define REG_B0_INT_ENABLE_2     0x12
#define REG_B0_INT_ENABLE_3     0x13
#define REG_B0_I2C_MST_STATUS   0x17
#define REG_B0_INT_STATUS       0x19
#define REG_B0_INT_STATUS_1     0x1A
#define REG_B0_INT_STATUS_2     0x1B
#define REG_B0_INT_STATUS_3     0x1C
#define REG_B0_DELAY_TIMEH      0x28
#define REG_B0_DELAY_TIMEL      0x29
#define REG_B0_ACCEL_XOUT_H     0x2D
#define REG_B0_ACCEL_XOUT_L     0x2E
#define REG_B0_ACCEL_YOUT_H     0x2F
#define REG_B0_ACCEL_YOUT_L     0x30
#define REG_B0_ACCEL_ZOUT_H     0x31
#define REG_B0_ACCEL_ZOUT_L     0x32
#define REG_B0_GYRO_XOUT_H      0x33
#define REG_B0_GYRO_XOUT_L      0x34
#define REG_B0_GYRO_YOUT_H      0x35
#define REG_B0_GYRO_YOUT_L      0x36
#define REG_B0_GYRO_ZOUT_H      0x37
#define REG_B0_GYRO_ZOUT_L      0x38
#define REG_B0_TEMP_OUT_H       0x39
#define REG_B0_TEMP_OUT_L       0x3A
/* 0x3B - 0x52 EXT_SLV_SEN_DATA */
#define REG_B0_FIFO_EN_1        0x66
#define REG_B0_FIFO_EN_2        0x67
#define REG_B0_FIFO_RST         0x68
#define REG_B0_FIFO_MODE        0x69
#define REG_B0_FIFO_COUNTH      0x70
#define REG_B0_FIFO_COUNTL      0x71
#define REG_B0_FIFO_R_W         0x72
#define REG_B0_DATA_RDY_STATUS  0x74
#define REG_B0_FIFO_CFG         0x76

/* bank 1 registers */
#define REG_B1_SELF_TEST_X_GYRO         0x02
#define REG_B1_SELF_TEST_Y_GYRO         0x03
#define REG_B1_SELF_TEST_Z_GYRO         0x04
#define REG_B1_SELF_TEST_X_ACCEL        0x0E
#define REG_B1_SELF_TEST_Y_ACCEL        0x0F
#define REG_B1_SELF_TEST_Z_ACCEL        0x10
#define REG_B1_XA_OFFS_H        0x14
#define REG_B1_XA_OFFS_L        0x15
#define REG_B1_YA_OFFS_H        0x17
#define REG_B1_YA_OFFS_L        0x18
#define REG_B1_ZA_OFFS_H        0x1A
#define REG_B1_ZA_OFFS_L        0x1B
#define REG_B1_TIMEBASE_CORRECTION_PLL  0x28

/* bank 2 registers */
#define REG_B2_GYRO_SMPLRT_DIV  0x00
#define REG_B2_GYRO_CONFIG_1    0x01
#define REG_B2_GYRO_CONFIG_2    0x02
#define REG_B2_XG_OFFS_USRH     0x03
#define REG_B2_XG_OFFS_USRL     0x04
#define REG_B2_YG_OFFS_USRH     0x05
#define REG_B2_YG_OFFS_USRL     0x06
#define REG_B2_ZG_OFFS_USRH     0x07
#define REG_B2_ZG_OFFS_USRL     0x08
#define REG_B2_ODR_ALIGN_EN     0x09
#define REG_B2_ACCEL_SMPLRT_DIV_1       0x10
#define REG_B2_ACCEL_SMPLRT_DIV_2       0x11
#define REG_B2_ACCEL_INTEL_CTRL         0x12
#define REG_B2_ACCEL_WOM_THR            0x13
#define REG_B2_ACCEL_CONFIG             0x14
#define REG_B2_ACCEL_CONFIG_2           0x15
#define REG_B2_FSYNC_CONFIG             0x52
#define REG_B2_TEMP_CONFIG              0x53
#define REG_B2_MOD_CTRL_USR             0x54

/* bank 3 registers not use in this project */

static inline uint8_t read_reg(uint8_t reg)
{
        uint8_t val;

        read_reg_multi(reg, &val, 1);

        return val;
}

static inline void write_reg(uint8_t reg, uint8_t val)
{
        write_reg_multi(reg, &val, 1);
}

static inline int write_check_reg(uint8_t reg, uint8_t val)
{
        write_reg_multi(reg, &val, 1);
        if (read_reg(reg) != val)
                return 1;
        return 0;
}

static inline void set_bank(uint8_t bank)
{
        bank = bank << 4;
        write_reg(REG_BANK_SEL, bank);
}

/* Condition:
 * Gyro: 250 dps, Accel: 2 g, Sample rate: 1125 kHz
 */
void calibrate(void)
{
        uint16_t fifo_cnt;
        uint8_t packet_cnt;
        uint8_t data[12];
        int32_t tmp;
        int32_t bias[6] = {0};
        uint8_t i, j;

        set_bank(2);
        /* set gyro low pass filter to 12Hz */
        write_reg(REG_B2_GYRO_CONFIG_1, 0x29);
        delay_ms(50); /* wait gyro start */

        set_bank(0);
        /* reset FIFO */
        write_reg(REG_B0_FIFO_RST, 0x1F);
        delay_ms(10);
        write_reg(REG_B0_FIFO_RST, 0x00);
        delay_ms(15);
        /* set FIFO to snapshot */
        write_reg(REG_B0_FIFO_MODE, 0x1F);
        /* enable FIFO */
        write_reg(REG_B0_USER_CTRL, 0x40);
        /* enable sensor FIFO */
        write_reg(REG_B0_FIFO_EN_2, 0x1E);
        delay_ms(250); /* 280 samples =  3360 bytes */
        /* disable sensor FIFO */
        write_reg(REG_B0_FIFO_EN_2, 0x00);
        /* get FIFO count */
        read_reg_multi(REG_B0_FIFO_COUNTH, data, 2);
        fifo_cnt = (data[0] << 8) | data[1];
        packet_cnt = fifo_cnt / 12;

        for (i = 0; i < packet_cnt; i++) {
                read_reg_multi(REG_B0_FIFO_R_W, data, 12);
                for (j = 0; j < 6; j++) {
                        tmp = (int16_t)(data[j * 2] << 8) | data[j * 2 + 1];
                        bias[j] += tmp;
                }
        }
        for (i = 0; i < 6; i++) {
                bias[i] /= (int32_t)packet_cnt;
        }
        bias[2] -= 16384;
#ifdef ACCEL_CALIBRATION_BIAS
        bias[0] = (int32_t)(ACCEL_X_BIAS * 16384.0);
        bias[1] = (int32_t)(ACCEL_Y_BIAS * 16384.0);
        bias[2] = (int32_t)(ACCEL_Z_BIAS * 16384.0);
#endif
#ifdef ACCEL_CALIBRATION
        uint8_t mask_bit;
        set_bank(1); /* set accel bias */
        for (i = 0; i < 3; i++) { /* 3 bytes: H(15:8), L(7:1), reserve */
                read_reg_multi(REG_B1_XA_OFFS_H + i * 3, data, 2);
                tmp = (int16_t)(data[0] << 8) | data[1];
                mask_bit = tmp & 0x1UL;
                tmp -= bias[i] >> 3; /* 2048 LSB/g*/
                data[0] = (tmp >> 8) & 0xFF;
                data[1] = (tmp & 0xFE) | mask_bit;
                write_reg_multi(REG_B1_XA_OFFS_H + i * 3, data, 2);
        }
#endif
        set_bank(2); /* set gyro bias */
        for (i = 0; i < 3; i++) {
                tmp = -(bias[3 + i] >> 2); /* divide by 1000/250dps = 4 */
                data[0] = (tmp >> 8) & 0xFF;
                data[1] = tmp & 0xFF;
                write_reg_multi(REG_B2_XG_OFFS_USRH + i * 2, data, 2);
        }
}

/**
 * @brief initialize ICM20948 and AK09916
 * 
 * @param rate (5 - 500 Hz), note: max magnetometer rate: 100 Hz
 * @param g_fs 250/500/1000/2000 dps
 * @param a_fs 2/4/8/16 g
 * @return int 0: successful / 1: failed
 */
int icm20948_init(uint16_t rate, enum gyro_fs g_fs, enum accel_fs a_fs,
                  enum low_pass lp)
{
        uint8_t val;

        delay_ms(100);
        set_bank(0);
        write_reg(REG_B0_PWR_MGMT_1, 0x80); /* reset */
        delay_ms(100);
        write_reg(REG_B0_PWR_MGMT_1, 0x01); /* power up */
        delay_ms(100);
        /* get who am I */
        if (read_reg(REG_B0_WHO_AM_I) != 0xEA)
                return 1;
#ifdef AK09916_ENABLE
        /* enable bypass i2c */
        if (write_check_reg(REG_B0_INT_PIN_CFG, 0x02))
                return 1;

        if (ak09916_init(AK09916_100HZ_MODE))
                return 1;
#endif
        calibrate();

        set_bank(2);
        /* set gyro low pass filter and full scale range */
        gyro_unit = (float)(1 << g_fs) / 131.0;
        val = 0x01 | (lp << 3);
        if (write_check_reg(REG_B2_GYRO_CONFIG_1, val | (g_fs << 1)))
                return 1;
        /* set accel low pass filter and full scale range */
        accel_unit = (float)(1 << a_fs) / 16384.0;
        if (write_check_reg(REG_B2_ACCEL_CONFIG, val | (a_fs << 1)))
                return 1;
        val = 1125 / rate - 1; /* set gyro sampling rate */
        if (write_check_reg(REG_B2_GYRO_SMPLRT_DIV, val))
                return 1;
        /* enable odr start time alignment */
        if (write_check_reg(REG_B2_ODR_ALIGN_EN, 0x01))
                return 1;
        /* set accel sampling rate */
        if (write_check_reg(REG_B2_ACCEL_SMPLRT_DIV_2, val))
                return 1;
        
        set_bank(0);
        /* reset FIFO */
        write_reg(REG_B0_FIFO_RST, 0x1F);
        delay_ms(10);
        write_reg(REG_B0_FIFO_RST, 0x00);
        delay_ms(15);
        /* disble FIFO */
        write_reg(REG_B0_USER_CTRL, 0x00);
#ifdef ICM20948_FIFO_EN
        /* set FIFO to stream */
        if (write_check_reg(REG_B0_FIFO_MODE, 0x00))
                return 1;
        /* enable FIFO */
        if (write_check_reg(REG_B0_USER_CTRL, 0x40))
                return 1;
        /* enable accel and gyro FIFO */
        if (write_check_reg(REG_B0_FIFO_EN_2, 0x1E))
                return 1;
#endif
        return 0;
}

#ifdef ICM20948_FIFO_EN
int icm20948_read_axis6(float *ax, float *ay, float *az,
                        float *gx, float *gy, float *gz)
{
        uint16_t fifo_cnt;
        read_reg_multi(REG_B0_FIFO_COUNTH, (uint8_t *)&fifo_cnt, 2);
        fifo_cnt = (fifo_cnt >> 8) | (fifo_cnt << 8);
        const uint16_t packet_cnt = fifo_cnt / 12;
        if (packet_cnt < 12 || packet_cnt > 4080) /* FIFO empty / overflow */
                return 1;
        uint8_t raw_data[packet_cnt][12];
        uint8_t *data;
        int16_t tmp;
        float avg[6] = {0};

        if (read_reg_multi(REG_B0_FIFO_R_W, raw_data[0],
                           fifo_cnt - fifo_cnt % 12))
                return 1;

        for (uint16_t i = 0; i < packet_cnt; i++) {
                data = &raw_data[i][0];
                for (uint8_t j = 0; j < 6; j++) {
                        tmp = (int16_t)(data[j * 2] << 8) | (data[j * 2 + 1]);
                        avg[j] += (float)tmp;
                }
        }
        *ax = avg[0] / packet_cnt * accel_unit;
        *ay = avg[1] / packet_cnt * accel_unit;
        *az = avg[2] / packet_cnt * accel_unit;
        *gx = avg[3] / packet_cnt * gyro_unit;
        *gy = avg[4] / packet_cnt * gyro_unit;
        *gz = avg[5] / packet_cnt * gyro_unit;

        return 0;
}
#else
int icm20948_read_axis6(float *ax, float *ay, float *az,
                        float *gx, float *gy, float *gz)
{       
        uint8_t raw_data[12];

        if (read_reg_multi(REG_B0_ACCEL_XOUT_H, raw_data, 12))
                return 1;
        *ax = (float)((int16_t)(raw_data[0] << 8) | (raw_data[1]))
                * accel_unit;
        *ay = (float)((int16_t)(raw_data[2] << 8) | (raw_data[3]))
                * accel_unit;
        *az = (float)((int16_t)(raw_data[4] << 8) | (raw_data[5]))
                * accel_unit;
#ifdef ACCEL_CALIBRATION_SCALE
        *ax *= ACCEL_X_SCALE;
        *ay *= ACCEL_Y_SCALE;
        *az *= ACCEL_Z_SCALE;
#endif
        *gx = (float)((int16_t)(raw_data[6] << 8) | (raw_data[7]))
                * gyro_unit;
        *gy = (float)((int16_t)(raw_data[8] << 8) | (raw_data[9]))
                * gyro_unit;
        *gz = (float)((int16_t)(raw_data[10] << 8) | (raw_data[11]))
                * gyro_unit;
        
        return 0;
}
#endif
#ifdef AK09916_ENABLE
/**
 * @brief read AK09916 data
 * 
 * @param mx 
 * @param my 
 * @param mz 
 * @return int 0: successful /  1: failed (not ready)
 */
int icm20948_read_mag(float *mx, float *my, float *mz)
{
        return ak09916_read_data(mx, my, mz);
}
#endif