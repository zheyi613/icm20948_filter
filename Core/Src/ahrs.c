/**
 * @file ahrs.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief Mahony's complementary filter
 * @date 2023-03-31
 */

#include "ahrs.h"
#include "math.h"

#define DOUBLE_KP    2.F   // P gain governs rate of convergence of accel/mag
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

/**
 * @brief Update attitute with 9-dof sensor data
 * 
 * @param gx gyroscope x (ras/s)
 * @param gy gyroscope y (ras/s)
 * @param gz gyroscope z (ras/s)
 * @param ax acceleration x
 * @param ay acceleration y
 * @param az acceleration z
 * @param mx magnetometer x
 * @param my magnetometer y
 * @param mz magnetometer z
 * @param dt time between two measurement
 */
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
        bx = sqrtf((hx * hx) + (hy * hy)); // assume by = 0
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) +
                     mz * (0.5f - q1q1 - q2q2));
        // estimated direction of gravity and magnetic field (v and w)
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
#ifdef NED_FRAME       /* gravity: (0, 0, -1) */ 
        halfvx = -halfvx;
        halfvy = -halfvy;
        halfvz = -halfvz;
#endif
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

/**
 * @brief Update attitute with 6-dof sensor data
 * 
 * @param gx gyroscope x (ras/s)
 * @param gy gyroscope y (ras/s)
 * @param gz gyroscope z (ras/s)
 * @param ax acceleration x
 * @param ay acceleration y
 * @param az acceleration z
 * @param dt time between two measurement
 */
void AHRSupdateIMU(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt)
{
        float recip_norm;
        float halfvx, halfvy, halfvz;
        float halfex, halfey, halfez;
        float qa = q0, qb = q1, qc = q2;

        recip_norm = inv_sqrt(ax*ax + ay*ay + az*az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;
#ifdef NED_FRAME       /* gravity: (0, 0, -1) */ 
        halfvx = -halfvx;
        halfvy = -halfvy;
        halfvz = -halfvz;
#endif
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

/**
 * @brief get Enler angle in rad/s
 *        ENU frame: ZXY, NED frame: ZYX
 * 
 * @param r roll  (rad/s)
 * @param p pitch (rad/s)
 * @param y yaw   (rad/s)
 */
void AHRS2euler(float *r, float *p, float *y)
{
#ifdef NED_FRAME
        *r = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
        *p = asinf(2.0f * (q0 * q2 - q1 * q3));
#else
        *p = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
        *r = asinf(2.0f * (q0 * q2 - q1 * q3));
#endif
        *y = atan2f(q0 * q3 + q1 * q2, 0.5f - q2 * q2 - q3 * q3);
}

void AHRS2quat(float q[4])
{
        q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = q3;
}