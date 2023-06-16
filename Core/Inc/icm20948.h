/**
 * @file icm20948.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief icm20948 driver
 * @date 2023-03-15
 */

#ifndef ICM20948_H
#define ICM20948_H

#include "stdint.h"

#define ICM20948_AD0    0
#define ICM20948_ADDR   (0b1101000 | ICM20948_AD0)

// #define ACCEL_CALIBRATION

// #define ACCEL_CALIBRATION_BIAS
// #define ACCEL_X_BIAS    -0.0075F
// #define ACCEL_Y_BIAS    -0.019F
// #define ACCEL_Z_BIAS    0.0065F
// #define ACCEL_CALIBRATION_SCALE
// #define ACCEL_X_SCALE   0.99F
// #define ACCEL_Y_SCALE   0.9862F
// #define ACCEL_Z_SCALE   0.9756F

#define AK09916_ENABLE

/* 
 * When data rate is higher than read data frequency,
 * the read data function will get multiple data from FIFO
 * and use average filter.
 * Note that the frequency of calling read data function
 * must not lower than data rate / 200.
 * 4096 bytes(buffer size) / 12(6-axis data) = 341
 * ex: data rate: 200, read freq > 1 Hz
 */
// #define ICM20948_FIFO_EN

enum gyro_fs {
        GYRO_250_DPS,
        GYRO_500_DPS,
        GYRO_1000_DPS,
        GYRO_2000_DPS
};

enum accel_fs {
        ACCEL_2G,
        ACCEL_4G,
        ACCEL_8G,
        ACCEL_16G
};

/* gyro low pass freqency */
enum low_pass {
        LP_BW_196HZ, /* accel: 246 Hz */
        LP_BW_151HZ, /* accel: 246 Hz */
        LP_BW_119HZ, /* accel: 114 Hz */
        LP_BW_51HZ,
        LP_BW_23HZ,
        LP_BW_11HZ,
        LP_BW_5HZ,
        LP_BW_361HZ
};

int icm20948_init(uint16_t rate, enum gyro_fs g_fs, enum accel_fs a_fs,
                  enum low_pass lp);
int icm20948_read_axis6(float *ax, float *ay, float *az,
                        float *gx, float *gy, float *gz);
#ifdef AK09916_ENABLE
int icm20948_read_mag(float *mx, float *my, float *mz);
#endif

#endif /* ICM20948_H */