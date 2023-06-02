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

static inline void normalize_quat(float *a, float *b, float *c, float *d)
{
        float recip_norm;

        recip_norm = inv_sqrt((*a) * (*a) + (*b) * (*b) + (*c) * (*c) +
                              (*d) * (*d));
        *a *= recip_norm;
        *b *= recip_norm;
        *c *= recip_norm;
        *d *= recip_norm;        
}

static inline void normalize_vec(float *x, float *y, float *z)
{
        float recip_norm;

        recip_norm = inv_sqrt((*x) * (*x) + (*y) * (*y) + (*z) * (*z));
        *x *= recip_norm;
        *y *= recip_norm;
        *z *= recip_norm;
}

static inline void cross_vec(float ax, float ay, float az,
                             float bx, float by, float bz,
                             float *cx, float *cy, float *cz)
{
        *cx = ay * bz - az * by;
        *cy = az * bx - ax * bz;
        *cz = ax * by - ay * bx;
}

static inline float dot_vec(float ax, float ay, float az,
                            float bx, float by, float bz)
{
        float dot;

        dot = ax * bx;
        dot += ay * by;
        dot += az * bz;

        return dot;
}

/**
 * @brief Cross two unit vector to get quaternion
 * 
 * @param ax
 * @param ay 
 * @param az 
 * @param bx 
 * @param by 
 * @param bz 
 * @param qa 
 * @param qb 
 * @param qc 
 * @param qd 
 */
static void cross_vec2quat(float ax, float ay, float az,
                           float bx, float by, float bz,
                           float *qa, float *qb, float *qc, float *qd)
{
        float cx, cy, cz;
        float dot, cross_norm_square;
        float half_x, half_y, half_z;

        dot = dot_vec(ax, ay, az, bx, by, bz);
        cross_vec(ax, ay, az, bx, by, bz, &cx, &cy, &cz);
        cross_norm_square = cx * cx + cy * cy + cz * cz;
        if (cross_norm_square < 0.000001f && dot < 0.f) { /* angle ~ 180 deg */
                cx = fabs(cx);
                cy = fabs(cy);
                cz = fabs(cz);
                if (cx < cy && cx < cz) {
                        *qb = 0.f;
                        *qc = az;
                        *qd = -ay;
                } else if (cy < cx && cy < cz) {
                        *qb = -az;
                        *qc = 0.f;
                        *qd = ax;
                } else {
                        *qb = ay;
                        *qc = -ax;
                        *qd = 0.f;
                }
                *qa = 0.f;
        } else {
                half_x = ax + bx;
                half_y = ay + by;
                half_z = az + bz;
                normalize_vec(&half_x, &half_y, &half_z);
                *qa = dot_vec(ax, ay, az, half_x, half_y, half_z);
                cross_vec(ax, ay, az, half_x, half_y, half_z, &cx, &cy, &cz);
                *qb = cx;
                *qc = cy;
                *qd = cz;
        }
}

/**
 * @brief Initialize quaternion of attitude with acceleration and
 *        magnetic field
 * 
 * @param ax 
 * @param ay 
 * @param az 
 * @param mx 
 * @param my 
 * @param mz 
 */
void ahrs_init(float ax, float ay, float az,
               float mx, float my, float mz)
{
        float y_x, y_y, y_z;
        float by_x, by_y, by_z;
        float qz0, qz1, qz2, qz3;
        float qy0, qy1, qy2, qy3;
#ifdef NED_FRAME
        ax = -ax;
        ay = -ay;
        az = -az;
#endif
        /* normalize measurement data */
        normalize_vec(&ax, &ay, &az);
        normalize_vec(&mx, &my, &mz);
        /* get rotation of world z to body z */
        cross_vec2quat(ax, ay, az, 0, 0, 1, &qz0, &qz1, &qz2, &qz3);
        /* rotate body y to make world z coincide with body z */
        by_x = 2.f * (qz1 * qz2 + qz0 * qz3);
        by_y = 2.f * (0.5f - qz1 * qz1 + qz3 * qz3);
        by_z = 2.f * (qz2 * qz3 - qz0 * qz1);
        /* acceleration (wz) cross magnetic field to get world y */
        cross_vec(ax, ay, az, mx, my, mz, &y_x, &y_y, &y_z);
        normalize_vec(&y_x, &y_y, &y_z);
        /* compute quaternion between world y and body y */
        cross_vec2quat(y_x, y_y, y_z, by_x, by_y, by_z,
                       &qy0, &qy1, &qy2, &qy3);
        /* product two quaternion */
        q0 = qz0 * qy0 - qz1 * qy1 - qz2 * qy2 - qz3 * qy3;
        q1 = qz1 * qy0 + qz0 * qy1 - qz3 * qy2 + qz2 * qy3;
        q2 = qz2 * qy0 + qz3 * qy1 + qz0 * qy2 - qz1 * qy3;
        q3 = qz3 * qy0 - qz2 * qy1 + qz1 * qy2 + qz0 * qy3;
        /* normalize quaternion */
        normalize_quat(&q0, &q1, &q2, &q3);
}

/**
 * @brief Initialize quaternion of attitude with acceleration
 *        Assume yaw is zero.
 * 
 * @param ax 
 * @param ay 
 * @param az 
 */
void ahrs_init_imu(float ax, float ay, float az)
{
#ifdef NED_FRAME
        ax = -ax;
        ay = -ay;
        az = -az;
#endif
        normalize_vec(&ax, &ay, &az);
        /* get rotation of world z to body z */
        cross_vec2quat(ax, ay, az, 0, 0, 1, &q0, &q1, &q2, &q3);
        normalize_quat(&q0, &q1, &q2, &q3);
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
void ahrs_update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float dt)
{
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
        normalize_vec(&ax, &ay, &az);
        normalize_vec(&mx, &my, &mz);
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
        normalize_quat(&q0, &q1, &q2, &q3);
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
void ahrs_update_imu(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt)
{
        float halfvx, halfvy, halfvz;
        float halfex, halfey, halfez;
        float qa = q0, qb = q1, qc = q2;

        normalize_vec(&ax, &ay, &az);

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

        normalize_quat(&q0, &q1, &q2, &q3);
}

/**
 * @brief get Enler angle in rad/s
 *        ENU frame: ZXY, NED frame: ZYX
 * 
 * @param r roll  (rad/s)
 * @param p pitch (rad/s)
 * @param y yaw   (rad/s)
 */
void ahrs2euler(float *r, float *p, float *y)
{
#ifdef NED_FRAME
        *r = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
        *p = asinf(2.0f * (q0*q2 - q1*q3));
#else
        *p = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
        *r = asinf(2.0f * (q0 * q2 - q1 * q3));
#endif
        *y = atan2f(q0 * q3 + q1 * q2, 0.5f - q2 * q2 - q3 * q3);
}

void ahrs2quat(float q[4])
{
        q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = q3;
}