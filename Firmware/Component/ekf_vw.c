#include "ekf_vw.h"

#include <string.h>

static EKF_t g_ekf;

static void zero_6x6(float m[EKF_STATE_DIM][EKF_STATE_DIM]) {
    (void)memset(m, 0, sizeof(float) * EKF_STATE_DIM * EKF_STATE_DIM);
}

static void zero_3x3(float m[EKF_MEAS_DIM][EKF_MEAS_DIM]) {
    (void)memset(m, 0, sizeof(float) * EKF_MEAS_DIM * EKF_MEAS_DIM);
}

static void set_diag_6x6(float m[EKF_STATE_DIM][EKF_STATE_DIM],
                         float d0,
                         float d1,
                         float d2,
                         float d3,
                         float d4,
                         float d5) {
    zero_6x6(m);
    m[0][0] = d0;
    m[1][1] = d1;
    m[2][2] = d2;
    m[3][3] = d3;
    m[4][4] = d4;
    m[5][5] = d5;
}

static void set_diag_3x3(float m[EKF_MEAS_DIM][EKF_MEAS_DIM],
                         float d0,
                         float d1,
                         float d2) {
    zero_3x3(m);
    m[0][0] = d0;
    m[1][1] = d1;
    m[2][2] = d2;
}

void EKF_InitCtx(EKF_t *ekf) {
    const EKF_InitParams_t defaults = {
        .p_vx = 1e-3f,
        .p_vy = 1e-3f,
        .p_w = 1e-3f,
        .p_abx = 1e-3f,
        .p_aby = 1e-3f,
        .p_wbz = 1e-3f,

        .q_vx = 7e-2f,
        .q_vy = 7e-2f,
        .q_w = 5e-2f,
        .q_abx = 1e-5f,
        .q_aby = 1e-5f,
        .q_wbz = 1e-5f,

        .r_vx = 2e-2f,
        .r_vy = 2e-2f,
        .r_w = 2e-2f,
    };

    EKF_InitWithParamsCtx(ekf, &defaults);
}

void EKF_InitWithParamsCtx(EKF_t *ekf, const EKF_InitParams_t *params) {
    if ((ekf == (void *)0) || (params == (void *)0)) {
        return;
    }

    (void)memset(ekf->x, 0, sizeof(ekf->x));

    /* Bias initial values are explicitly zero by request. */
    ekf->x[3] = 0.0f;
    ekf->x[4] = 0.0f;
    ekf->x[5] = 0.0f;

    set_diag_6x6(ekf->P,
                 params->p_vx,
                 params->p_vy,
                 params->p_w,
                 params->p_abx,
                 params->p_aby,
                 params->p_wbz);

    set_diag_6x6(ekf->Q,
                 params->q_vx,
                 params->q_vy,
                 params->q_w,
                 params->q_abx,
                 params->q_aby,
                 params->q_wbz);

    set_diag_3x3(ekf->R, params->r_vx, params->r_vy, params->r_w);
}

void EKF_SetQDiagCtx(EKF_t *ekf,
                     float q_vx,
                     float q_vy,
                     float q_w,
                     float q_abx,
                     float q_aby,
                     float q_wbz) {
    if (ekf == (void *)0) {
        return;
    }

    set_diag_6x6(ekf->Q, q_vx, q_vy, q_w, q_abx, q_aby, q_wbz);
}

void EKF_SetRDiagCtx(EKF_t *ekf, float r_vx, float r_vy, float r_w) {
    if (ekf == (void *)0) {
        return;
    }

    set_diag_3x3(ekf->R, r_vx, r_vy, r_w);
}

void EKF_PredictCtx(EKF_t *ekf, float ax, float ay, float wz, float dt) {
    float F[EKF_STATE_DIM][EKF_STATE_DIM];
    float FP[EKF_STATE_DIM][EKF_STATE_DIM];
    float FPFt[EKF_STATE_DIM][EKF_STATE_DIM];
    float Pnew[EKF_STATE_DIM][EKF_STATE_DIM];

    if (ekf == (void *)0) {
        return;
    }

    if (dt <= 0.0f) {
        return;
    }

    /*
     * State prediction:
     * vx = vx + (ax - abx) * dt
     * vy = vy + (ay - aby) * dt
     * wz = wz_imu - wbz
     */
    ekf->x[0] = ekf->x[0] + (ax - ekf->x[3]) * dt;
    ekf->x[1] = ekf->x[1] + (ay - ekf->x[4]) * dt;
    ekf->x[2] = wz - ekf->x[5];

    /* Jacobian F */
    zero_6x6(F);
    F[0][0] = 1.0f;
    F[0][3] = -dt;

    F[1][1] = 1.0f;
    F[1][4] = -dt;

    F[2][5] = -1.0f;

    F[3][3] = 1.0f;
    F[4][4] = 1.0f;
    F[5][5] = 1.0f;

    /* FP = F * P */
    for (int i = 0; i < EKF_STATE_DIM; ++i) {
        for (int j = 0; j < EKF_STATE_DIM; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < EKF_STATE_DIM; ++k) {
                sum += F[i][k] * ekf->P[k][j];
            }
            FP[i][j] = sum;
        }
    }

    /* FPFt = FP * F^T */
    for (int i = 0; i < EKF_STATE_DIM; ++i) {
        for (int j = 0; j < EKF_STATE_DIM; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < EKF_STATE_DIM; ++k) {
                sum += FP[i][k] * F[j][k];
            }
            FPFt[i][j] = sum;
        }
    }

    /*
     * Process noise injection:
     * To reduce phase lag, raise q_vx/q_vy/q_w appropriately so predict keeps up with IMU dynamics.
     */
    (void)memcpy(Pnew, FPFt, sizeof(Pnew));
    for (int i = 0; i < EKF_STATE_DIM; ++i) {
        Pnew[i][i] += ekf->Q[i][i] * dt;
    }

    for (int i = 0; i < EKF_STATE_DIM; ++i) {
        for (int j = i + 1; j < EKF_STATE_DIM; ++j) {
            float s = 0.5f * (Pnew[i][j] + Pnew[j][i]);
            Pnew[i][j] = s;
            Pnew[j][i] = s;
        }
    }

    (void)memcpy(ekf->P, Pnew, sizeof(ekf->P));
}

void EKF_UpdateCtx(EKF_t *ekf, float vx_odom, float vy_odom, float w_odom) {
    float y[EKF_MEAS_DIM];
    float S[EKF_MEAS_DIM][EKF_MEAS_DIM];
    float invS[EKF_MEAS_DIM][EKF_MEAS_DIM];
    float PHt[EKF_STATE_DIM][EKF_MEAS_DIM];
    float K[EKF_STATE_DIM][EKF_MEAS_DIM];
    float Pnew[EKF_STATE_DIM][EKF_STATE_DIM];
    float KH[EKF_STATE_DIM][EKF_STATE_DIM];
    float I_KH[EKF_STATE_DIM][EKF_STATE_DIM];

    if (ekf == (void *)0) {
        return;
    }

    y[0] = vx_odom - ekf->x[0];
    y[1] = vy_odom - ekf->x[1];
    y[2] = w_odom - ekf->x[2];

    /* H selects first three states, so S is top-left 3x3 of P + R */
    for (int i = 0; i < EKF_MEAS_DIM; ++i) {
        for (int j = 0; j < EKF_MEAS_DIM; ++j) {
            S[i][j] = ekf->P[i][j] + ekf->R[i][j];
        }
    }

    /* Invert S (3x3) */
    {
        float a = S[0][0], b = S[0][1], c = S[0][2];
        float d = S[1][0], e = S[1][1], f = S[1][2];
        float g = S[2][0], h = S[2][1], i = S[2][2];

        float A = e * i - f * h;
        float B = -(d * i - f * g);
        float C = d * h - e * g;
        float D = -(b * i - c * h);
        float E = a * i - c * g;
        float Fv = -(a * h - b * g);
        float G = b * f - c * e;
        float H = -(a * f - c * d);
        float I = a * e - b * d;

        float det = a * A + b * B + c * C;
        if (det > -1e-12f && det < 1e-12f) {
            return;
        }

        invS[0][0] = A / det;
        invS[0][1] = D / det;
        invS[0][2] = G / det;
        invS[1][0] = B / det;
        invS[1][1] = E / det;
        invS[1][2] = H / det;
        invS[2][0] = C / det;
        invS[2][1] = Fv / det;
        invS[2][2] = I / det;
    }

    /* PHt = P * H^T => first three columns of P */
    for (int r = 0; r < EKF_STATE_DIM; ++r) {
        for (int c = 0; c < EKF_MEAS_DIM; ++c) {
            PHt[r][c] = ekf->P[r][c];
        }
    }

    /* K = PHt * invS */
    for (int r = 0; r < EKF_STATE_DIM; ++r) {
        for (int c = 0; c < EKF_MEAS_DIM; ++c) {
            float sum = 0.0f;
            for (int k = 0; k < EKF_MEAS_DIM; ++k) {
                sum += PHt[r][k] * invS[k][c];
            }
            K[r][c] = sum;
        }
    }

    /* x = x + K y */
    for (int r = 0; r < EKF_STATE_DIM; ++r) {
        float delta = 0.0f;
        for (int c = 0; c < EKF_MEAS_DIM; ++c) {
            delta += K[r][c] * y[c];
        }
        ekf->x[r] += delta;
    }

    /* KH = K * H (H selects first 3 states) */
    zero_6x6(KH);
    for (int r = 0; r < EKF_STATE_DIM; ++r) {
        KH[r][0] = K[r][0];
        KH[r][1] = K[r][1];
        KH[r][2] = K[r][2];
    }

    zero_6x6(I_KH);
    for (int i = 0; i < EKF_STATE_DIM; ++i) {
        I_KH[i][i] = 1.0f;
    }
    for (int i = 0; i < EKF_STATE_DIM; ++i) {
        for (int j = 0; j < EKF_STATE_DIM; ++j) {
            I_KH[i][j] -= KH[i][j];
        }
    }

    /* P = (I - KH) P */
    for (int i = 0; i < EKF_STATE_DIM; ++i) {
        for (int j = 0; j < EKF_STATE_DIM; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < EKF_STATE_DIM; ++k) {
                sum += I_KH[i][k] * ekf->P[k][j];
            }
            Pnew[i][j] = sum;
        }
    }

    for (int i = 0; i < EKF_STATE_DIM; ++i) {
        for (int j = i + 1; j < EKF_STATE_DIM; ++j) {
            float s = 0.5f * (Pnew[i][j] + Pnew[j][i]);
            Pnew[i][j] = s;
            Pnew[j][i] = s;
        }
    }

    (void)memcpy(ekf->P, Pnew, sizeof(ekf->P));
}

void EKF_Init(void) {
    EKF_InitCtx(&g_ekf);
}

void EKF_InitWithParams(const EKF_InitParams_t *params) {
    EKF_InitWithParamsCtx(&g_ekf, params);
}

void EKF_SetQDiag(float q_vx,
                  float q_vy,
                  float q_w,
                  float q_abx,
                  float q_aby,
                  float q_wbz) {
    EKF_SetQDiagCtx(&g_ekf, q_vx, q_vy, q_w, q_abx, q_aby, q_wbz);
}

void EKF_SetRDiag(float r_vx, float r_vy, float r_w) {
    EKF_SetRDiagCtx(&g_ekf, r_vx, r_vy, r_w);
}

void EKF_Predict(float ax, float ay, float wz, float dt) {
    EKF_PredictCtx(&g_ekf, ax, ay, wz, dt);
}

void EKF_Update(float vx_odom, float vy_odom, float w_odom) {
    EKF_UpdateCtx(&g_ekf, vx_odom, vy_odom, w_odom);
}

EKF_t *EKF_GetHandle(void) {
    return &g_ekf;
}
