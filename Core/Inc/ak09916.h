/**
 * @file ak09916.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief AK09916 driver
 * @date 2023-03-17
 */
#ifndef AK09916_H
#define AK09916_H

#define AK09916_ADDR    0x0C

/* calibration parameter */
#define MAG_CX          22
#define MAG_CY          47
#define MAG_CZ          6

enum ak09916_mode {
        AK09916_POWER_DOWN_MODE,
        AK09916_SINGLE_MEASUREMENT_MODE,
        AK09916_10HZ_MODE,
        AK09916_20HZ_MODE = 4,
        AK09916_50HZ_MODE,
        AK09916_100HZ_MODE = 8,
        AK09916_SELF_TEST_MODE = 16
};

int ak09916_init(enum ak09916_mode mode);
void ak09916_read_data(float *mx, float *my, float *mz);

#endif /* AK09916_H */