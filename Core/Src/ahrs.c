/* mahony AHRS */

#include "math.h"

#define KP     2.0F   // P gain governs rate of convergence of accel/mag
#define KI     0.005F // I gain governs rate of convergence of gyro biases
#define SAMPLE_FREQ     50
#define HALF_T 0.01  // half the sample period
#define DEG2RAD 57.29577951

static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
static float ex_int = 0, ey_int = 0, ez_int = 0; // scaled integral error

void AHRSupdate(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz)
{
        float norm;
        float hx, hy, hz, bx, bz;
        float vx, vy, vz, wx, wy, wz;
        float ex, ey, ez;
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
        float tmp_q0 = q0, tmp_q1 = q1, tmp_q2 = q2;
        // normalise the measurements
        norm = sqrt(ax*ax + ay*ay + az*az);
        ax = ax / norm;
        ay = ay / norm;
        az = az / norm;
        norm = sqrt(mx*mx + my*my + mz*mz);
        mx = mx / norm;
        my = my / norm;
        mz = mz / norm;
        // compute reference direction of magnetic field
        hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
        hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
        hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 -q2q2);
        bx = sqrt((hx*hx) + (hy*hy));
        bz = hz;
        // estimated direction of gravity and magnetic field (v and w)
        vx = 2*(q1q3 - q0q2);
        vy = 2*(q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;
        wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
        wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
        wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
        // error is sum of cross product between reference direction of fields
        // and direction measured by sensors
        ex = (ay*vz - az*vy) + (my*wz - mz*wy);
        ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
        ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
        // integral error scaled integral gain
        ex_int = ex_int + ex*KI* (1.0f / SAMPLE_FREQ);
        ey_int = ey_int + ey*KI* (1.0f / SAMPLE_FREQ);
        ez_int = ez_int + ez*KI* (1.0f / SAMPLE_FREQ);
        // adjusted gyroscope measurements
        gx = gx + KP*ex + ex_int;
        gy = gy + KP*ey + ey_int;
        gz = gz + KP*ez + ez_int;
        // integrate quaternion rate and normalize (1st-order Rouge-Kutta)
        q0 += (-tmp_q1*gx - tmp_q2*gy - q3*gz)*HALF_T;
        q1 += (tmp_q0*gx + tmp_q2*gz - q3*gy)*HALF_T;
        q2 += (tmp_q0*gy - tmp_q1*gz + q3*gx)*HALF_T;
        q3 += (tmp_q0*gz + tmp_q1*gy - tmp_q2*gx)*HALF_T;
        // normalise quaternion  
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);  
        q0 = q0 / norm;  
        q1 = q1 / norm;  
        q2 = q2 / norm;  
        q3 = q3 / norm;  
}

void AHRS2euler(float *r, float *p, float *y)
{
        float q0q1 = q0*q1;
        float q0q2 = q0*q2;
        float q0q3 = q0*q3;
        float q1q1 = q1*q1;
        float q1q2 = q1*q2;
        float q1q3 = q1*q3;
        float q2q2 = q2*q2;
        float q2q3 = q2*q3;
        float q3q3 = q3*q3;
        *r = atan2(2*(q2q3 + q0q1), 2*(0.5 - q1q1 - q2q2)) * DEG2RAD;
        *p = asinf(2*(q1q3 - q0q2)) * DEG2RAD;
        *y = atan2(2*(q1q2 + q0q3), 2*(0.5 - q2q2 - q3q3)) * DEG2RAD;
}