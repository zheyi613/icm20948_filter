/**
 * @file ahrs.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief Mahony's AHRS algorithm
 * @date 2023-04-26
 */

#ifndef AHRS_H
#define AHRS_H

#define NED_FRAME

void ahrs_init(float ax, float ay, float az,
               float mx, float my, float mz);
void ahrs_init_imu(float ax, float ay, float az);
void ahrs_update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float dt);
void ahrs_update_imu(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt);
void ahrs2euler(float *r, float *p, float *y);
void ahrs2quat(float q[4]);

#endif