#ifndef EKF_VW_H
#define EKF_VW_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

enum {
    EKF_STATE_DIM = 6,
    EKF_MEAS_DIM = 3
};

typedef struct {
    /* X = [vx, vy, wz, a_bx, a_by, w_bz]^T */
    float x[EKF_STATE_DIM];

    /* Covariances */
    float P[EKF_STATE_DIM][EKF_STATE_DIM];
    float Q[EKF_STATE_DIM][EKF_STATE_DIM];
    float R[EKF_MEAS_DIM][EKF_MEAS_DIM];
} EKF_t;

typedef struct {
    float p_vx;
    float p_vy;
    float p_w;
    float p_abx;
    float p_aby;
    float p_wbz;

    float q_vx;
    float q_vy;
    float q_w;
    float q_abx;
    float q_aby;
    float q_wbz;

    float r_vx;
    float r_vy;
    float r_w;
} EKF_InitParams_t;

/*
 * Initialize with safe defaults:
 * - state and bias are zero
 * - P is a small diagonal matrix
 * - Q/R are moderate defaults and can be tuned later
 */
void EKF_InitCtx(EKF_t *ekf);

/* Initialize with caller-provided diagonal parameters. */
void EKF_InitWithParamsCtx(EKF_t *ekf, const EKF_InitParams_t *params);

/* Runtime tuning interfaces (diagonal terms only). */
void EKF_SetQDiagCtx(EKF_t *ekf,
                     float q_vx,
                     float q_vy,
                     float q_w,
                     float q_abx,
                     float q_aby,
                     float q_wbz);
void EKF_SetRDiagCtx(EKF_t *ekf, float r_vx, float r_vy, float r_w);

/*
 * Predict (high-rate IMU driven, asynchronous):
 *   vx_k   = vx_{k-1} + (ax - a_bx) * dt
 *   vy_k   = vy_{k-1} + (ay - a_by) * dt
 *   wz_k   = wz_imu - w_bz
 *   a_bx_k = a_bx_{k-1}
 *   a_by_k = a_by_{k-1}
 *   w_bz_k = w_bz_{k-1}
 */
void EKF_PredictCtx(EKF_t *ekf, float ax, float ay, float wz, float dt);

/*
 * Update (low-rate odometry driven, asynchronous):
 *   z = [vx_odom, vy_odom, wz_odom]^T
 */
void EKF_UpdateCtx(EKF_t *ekf, float vx_odom, float vy_odom, float w_odom);

/*
 * Default singleton-style APIs for quick integration.
 * These signatures match common embedded usage in task callbacks.
 */
void EKF_Init(void);
void EKF_InitWithParams(const EKF_InitParams_t *params);
void EKF_SetQDiag(float q_vx,
                  float q_vy,
                  float q_w,
                  float q_abx,
                  float q_aby,
                  float q_wbz);
void EKF_SetRDiag(float r_vx, float r_vy, float r_w);
void EKF_Predict(float ax, float ay, float wz, float dt);
void EKF_Update(float vx_odom, float vy_odom, float w_odom);
EKF_t *EKF_GetHandle(void);

/* Convenience getters */
static inline float EKF_GetVx(const EKF_t *ekf) { return ekf->x[0]; }
static inline float EKF_GetVy(const EKF_t *ekf) { return ekf->x[1]; }
static inline float EKF_GetW(const EKF_t *ekf) { return ekf->x[2]; }

#ifdef __cplusplus
}
#endif

#endif /* EKF_VW_H */
