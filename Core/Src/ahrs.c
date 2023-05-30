/**
 * @file ahrs.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief Mahony's complementary filter
 * @date 2023-03-31
 */

#include "ahrs.h"
#include "math.h"

#define DOUBLE_KP    1.F   // P gain governs rate of convergence of accel/mag
#define DOUBLE_KI    0.005F // I gain governs rate of convergence of gyro biases

static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
static float ex_int = 0, ey_int = 0, ez_int = 0; // scaled integral error

static float inv_sqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *((int *)&x);           // get bits for floating value
    i = 0x5f375a86 - (i >> 1);      // gives initial guess y0
    x = *((float*)&i);              // convert bits back to float
    x = x * (1.5f - xhalf * x * x); // Newton 1st iteration
    return x;
}

void AHRSupdate(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float dt)
{
        float recip_norm;
        float hx, hy, bx, bz;
        float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
        float halfex, halfey, halfez;
        // auxiliary variables to reduce number of repeated operations
        float q0q0 = q0*q0;
        float q0q1 = q0*q1;
        float q0q2 = q0*q2;
        float q0q3 = q0*q3;
        float q1q1 = q1*q1;
        float q1q2 = q1*q2;
        float q1q3 = q1*q3;
        float q2q2 = q2*q2;
        float q2q3 = q2*q3;
        float q3q3 = q3*q3;
        float qa = q0, qb = q1, qc = q2;

        // Convert gyroscope deg/s to rad/s
        gx *= 0.0174533f;
        gy *= 0.0174533f;
        gz *= 0.0174533f;

        // normalise the measurements
        recip_norm = inv_sqrt(ax*ax + ay*ay + az*az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;
        recip_norm = inv_sqrt(mx*mx + my*my + mz*mz);
        mx *= recip_norm;
        my *= recip_norm;
        mz *= recip_norm;
        // compute reference direction of magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) +
                     mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) +
                     mz * (q2q3 - q0q1));
        bx = sqrt((hx * hx) + (hy * hy)); // assume by = 0
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) +
                     mz * (0.5f - q1q1 - q2q2));
        // estimated direction of gravity and magnetic field (v and w)
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2);
        // error is sum of cross product between reference direction of fields
        // and direction measured by sensors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
        // integral error scaled integral gain
        ex_int += halfex * DOUBLE_KI * dt;
        ey_int += halfey * DOUBLE_KI * dt;
        ez_int += halfez * DOUBLE_KI * dt;
        // adjusted gyroscope measurements
        gx += DOUBLE_KP * halfex + ex_int;
        gy += DOUBLE_KP * halfey + ey_int;
        gz += DOUBLE_KP * halfez + ez_int;
        // integrate quaternion rate and normalize (1st-order Rouge-Kutta)
        gx *= (0.5f * dt);
        gy *= (0.5f * dt);
        gz *= (0.5f * dt);
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);
        // normalise quaternion  
        recip_norm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);  
        q0 *= recip_norm;  
        q1 *= recip_norm;  
        q2 *= recip_norm;  
        q3 *= recip_norm;
}

void AHRSupdateIMU(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt)
{
        float recip_norm;
        float halfvx, halfvy, halfvz;
        float halfex, halfey, halfez;
        float qa = q0, qb = q1, qc = q2;

        gx *= 0.0174533f;
        gy *= 0.0174533f;
        gz *= 0.0174533f;

        recip_norm = inv_sqrt(ax*ax + ay*ay + az*az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        halfex = ay * halfvz - az * halfvy;
        halfey = az * halfvx - ax * halfvz;
        halfez = ax * halfvy - ay * halfvx;

        ex_int += halfex * DOUBLE_KI * dt;
        ey_int += halfey * DOUBLE_KI * dt;
        ez_int += halfez * DOUBLE_KI * dt;

        gx += DOUBLE_KP * halfex + ex_int;
        gy += DOUBLE_KP * halfey + ey_int;
        gz += DOUBLE_KP * halfez + ez_int;

        gx *= (0.5f * dt);
        gy *= (0.5f * dt);
        gz *= (0.5f * dt);
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);

        recip_norm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);  
        q0 *= recip_norm;  
        q1 *= recip_norm;  
        q2 *= recip_norm;  
        q3 *= recip_norm;
}

void AHRS2euler(float *r, float *p, float *y)
{
        float q0q1 = q0 * q1;
        float q0q2 = q0 * q2;
        float q0q3 = q0 * q3;
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q3q3 = q3 * q3;

        *r = atan2f(q0q1 + q2q3, 0.5f - q1q1 - q2q2) * 57.29577951f;
        *p = asinf(2.0f * (q0q2 - q1q3)) * 57.29577951f;
        *y = atan2f(q0q3 + q1q2, 0.5f - q2q2 - q3q3) * 57.29577951f;
}

void AHRS2quat(float q[4])
{
        q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = q3;
}