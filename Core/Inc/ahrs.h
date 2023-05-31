/**
 * @file ahrs.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief Mahony's AHRS algorithm
 * @date 2023-04-26
 */

#ifndef AHRS_H
#define AHRS_H

#define NED_FRAME

void AHRSupdate(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float dt);
void AHRSupdateIMU(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt);
void AHRS2euler(float *r, float *p, float *y);
void AHRS2quat(float q[4]);

#endif