#include "opt_flow.hpp"
void OptFlow::process(const OptFlow::Data_t& sensor_data, float imu_omega_z, float imu_angle_z) {
    this->process(sensor_data, imu_omega_z, imu_angle_z, 0.0f, 0.0f);
}
#include <cmath>
#include <cstring>
#include "stm32f4xx_hal.h"


//Kalman1D & Kalman2DPosVel Implementation

// 一维卡尔曼滤波器实现
Kalman1D::Kalman1D(float Q_, float R_, float P_, float x0)
    : Q(Q_), R(R_), P(P_), x(x0) {}

void Kalman1D::init(float Q_, float R_, float P_, float x0) {
    Q = Q_;
    R = R_;
    P = P_;
    x = x0;
}

float Kalman1D::update(float z) {
    P += Q;
    float K = P / (P + R);
    x = x + K * (z - x);
    P = (1.0f - K) * P;
    return x;
}

// 二维位置-速度卡尔曼滤波器实现
Kalman2DPosVel::Kalman2DPosVel() {
    setNoise(0.01f, 1.0f, 100.0f, 1e6f);
    init(0.0f, 0.0f, 0.0f, 0.0f, 100.0f);
}

void Kalman2DPosVel::setNoise(float q_pos, float q_vel, float r_vel, float r_pos) {
    Qd_[0] = q_pos; Qd_[1] = q_pos; Qd_[2] = q_vel; Qd_[3] = q_vel;
    Rv_[0] = r_vel; Rv_[1] = r_vel;
    Rp_[0] = r_pos; Rp_[1] = r_pos;
}

void Kalman2DPosVel::init(float px0, float py0, float vx0, float vy0, float p0) {
    x_[0] = px0; x_[1] = py0; x_[2] = vx0; x_[3] = vy0;
    for (int i = 0; i < 16; ++i) P_[i] = 0.0f;
    P_[0] = p0; P_[5] = p0; P_[10] = p0; P_[15] = p0;
}

void Kalman2DPosVel::predict(float ax, float ay, float dt) {
    if (dt <= 0.0f) return;
    const float dt2_2 = 0.5f * dt * dt;
    float px = x_[0] + dt * x_[2] + dt2_2 * ax;
    float py = x_[1] + dt * x_[3] + dt2_2 * ay;
    float vx = x_[2] + dt * ax;
    float vy = x_[3] + dt * ay;
    x_[0] = px; x_[1] = py; x_[2] = vx; x_[3] = vy;
    float AP[16];
    AP[0]  = P_[0] + dt * P_[8];
    AP[1]  = P_[1] + dt * P_[9];
    AP[2]  = P_[2] + dt * P_[10];
    AP[3]  = P_[3] + dt * P_[11];
    AP[4]  = P_[4] + dt * P_[12];
    AP[5]  = P_[5] + dt * P_[13];
    AP[6]  = P_[6] + dt * P_[14];
    AP[7]  = P_[7] + dt * P_[15];
    AP[8]  = P_[8];
    AP[9]  = P_[9];
    AP[10] = P_[10];
    AP[11] = P_[11];
    AP[12] = P_[12];
    AP[13] = P_[13];
    AP[14] = P_[14];
    AP[15] = P_[15];
    float Pn[16];
    Pn[0]  = AP[0];
    Pn[4]  = AP[4];
    Pn[8]  = AP[8];
    Pn[12] = AP[12];
    Pn[1]  = AP[1];
    Pn[5]  = AP[5];
    Pn[9]  = AP[9];
    Pn[13] = AP[13];
    Pn[2]  = AP[0] * dt + AP[2];
    Pn[6]  = AP[4] * dt + AP[6];
    Pn[10] = AP[8] * dt + AP[10];
    Pn[14] = AP[12] * dt + AP[14];
    Pn[3]  = AP[1] * dt + AP[3];
    Pn[7]  = AP[5] * dt + AP[7];
    Pn[11] = AP[9] * dt + AP[11];
    Pn[15] = AP[13] * dt + AP[15];
    Pn[0]  += Qd_[0];
    Pn[5]  += Qd_[1];
    Pn[10] += Qd_[2];
    Pn[15] += Qd_[3];
    for (int i = 0; i < 16; ++i) P_[i] = Pn[i];
}

void Kalman2DPosVel::updateVel(float vx_meas, float vy_meas) {
    float y0 = vx_meas - x_[2];
    float y1 = vy_meas - x_[3];
    float S00 = P_[10] + Rv_[0];
    float S01 = P_[11];
    float S10 = P_[14];
    float S11 = P_[15] + Rv_[1];
    float det = S00 * S11 - S01 * S10;
    if (det == 0.0f) return;
    float invS00 =  S11 / det;
    float invS01 = -S01 / det;
    float invS10 = -S10 / det;
    float invS11 =  S00 / det;
    float PHt[8];
    PHt[0] = P_[2];  PHt[1] = P_[3];
    PHt[2] = P_[6];  PHt[3] = P_[7];
    PHt[4] = P_[10]; PHt[5] = P_[11];
    PHt[6] = P_[14]; PHt[7] = P_[15];
    float K0 = PHt[0] * invS00 + PHt[1] * invS10;
    float K1 = PHt[0] * invS01 + PHt[1] * invS11;
    float K2 = PHt[2] * invS00 + PHt[3] * invS10;
    float K3 = PHt[2] * invS01 + PHt[3] * invS11;
    float K4 = PHt[4] * invS00 + PHt[5] * invS10;
    float K5 = PHt[4] * invS01 + PHt[5] * invS11;
    float K6 = PHt[6] * invS00 + PHt[7] * invS10;
    float K7 = PHt[6] * invS01 + PHt[7] * invS11;
    x_[0] += K0 * y0 + K1 * y1;
    x_[1] += K2 * y0 + K3 * y1;
    x_[2] += K4 * y0 + K5 * y1;
    x_[3] += K6 * y0 + K7 * y1;
    float KH_col2_0 = K0; float KH_col3_0 = K1;
    float KH_col2_1 = K2; float KH_col3_1 = K3;
    float KH_col2_2 = K4; float KH_col3_2 = K5;
    float KH_col2_3 = K6; float KH_col3_3 = K7;
    float Pn[16];
    for (int i = 0; i < 16; ++i) Pn[i] = P_[i];
    Pn[0*4 + 2] = P_[2]  - (KH_col2_0 * P_[10] + KH_col3_0 * P_[14]);
    Pn[1*4 + 2] = P_[6]  - (KH_col2_1 * P_[10] + KH_col3_1 * P_[14]);
    Pn[2*4 + 2] = P_[10] - (KH_col2_2 * P_[10] + KH_col3_2 * P_[14]);
    Pn[3*4 + 2] = P_[14] - (KH_col2_3 * P_[10] + KH_col3_3 * P_[14]);
    Pn[0*4 + 3] = P_[3]  - (KH_col2_0 * P_[11] + KH_col3_0 * P_[15]);
    Pn[1*4 + 3] = P_[7]  - (KH_col2_1 * P_[11] + KH_col3_1 * P_[15]);
    Pn[2*4 + 3] = P_[11] - (KH_col2_2 * P_[11] + KH_col3_2 * P_[15]);
    Pn[3*4 + 3] = P_[15] - (KH_col2_3 * P_[11] + KH_col3_3 * P_[15]);
    Pn[0] = P_[0] - (K0 * P_[2] + K1 * P_[3]);
    Pn[1] = P_[1] - (K0 * P_[6] + K1 * P_[7]);
    Pn[4] = P_[4] - (K2 * P_[2] + K3 * P_[3]);
    Pn[5] = P_[5] - (K2 * P_[6] + K3 * P_[7]);
    for (int i = 0; i < 16; ++i) P_[i] = Pn[i];
}

void Kalman2DPosVel::updatePos(float px_meas, float py_meas) {
    float y0 = px_meas - x_[0];
    float y1 = py_meas - x_[1];
    float S00 = P_[0] + Rp_[0];
    float S01 = P_[1];
    float S10 = P_[4];
    float S11 = P_[5] + Rp_[1];
    float det = S00 * S11 - S01 * S10;
    if (det == 0.0f) return;
    float invS00 =  S11 / det;
    float invS01 = -S01 / det;
    float invS10 = -S10 / det;
    float invS11 =  S00 / det;
    float PHt[8];
    PHt[0] = P_[0];  PHt[1] = P_[1];
    PHt[2] = P_[4];  PHt[3] = P_[5];
    PHt[4] = P_[8];  PHt[5] = P_[9];
    PHt[6] = P_[12]; PHt[7] = P_[13];
    float K0 = PHt[0] * invS00 + PHt[1] * invS10;
    float K1 = PHt[0] * invS01 + PHt[1] * invS11;
    float K2 = PHt[2] * invS00 + PHt[3] * invS10;
    float K3 = PHt[2] * invS01 + PHt[3] * invS11;
    float K4 = PHt[4] * invS00 + PHt[5] * invS10;
    float K5 = PHt[4] * invS01 + PHt[5] * invS11;
    float K6 = PHt[6] * invS00 + PHt[7] * invS10;
    float K7 = PHt[6] * invS01 + PHt[7] * invS11;
    x_[0] += K0 * y0 + K1 * y1;
    x_[1] += K2 * y0 + K3 * y1;
    x_[2] += K4 * y0 + K5 * y1;
    x_[3] += K6 * y0 + K7 * y1;
    float Pn[16];
    for (int i = 0; i < 16; ++i) Pn[i] = P_[i];
    Pn[0]  = P_[0]  - (K0 * P_[0] + K1 * P_[4]);
    Pn[4]  = P_[4]  - (K2 * P_[0] + K3 * P_[4]);
    Pn[8]  = P_[8]  - (K4 * P_[0] + K5 * P_[4]);
    Pn[12] = P_[12] - (K6 * P_[0] + K7 * P_[4]);
    Pn[1]  = P_[1]  - (K0 * P_[1] + K1 * P_[5]);
    Pn[5]  = P_[5]  - (K2 * P_[1] + K3 * P_[5]);
    Pn[9]  = P_[9]  - (K4 * P_[1] + K5 * P_[5]);
    Pn[13] = P_[13] - (K6 * P_[1] + K7 * P_[5]);
    for (int i = 0; i < 16; ++i) P_[i] = Pn[i];
}



// 3阶滑动中值滤波器：去除尖刺神器，且延迟极低(1个采样周期)
class MedianFilter3 {
public:
    MedianFilter3() { reset(); }

    void reset() {
        buffer_[0] = 0.0f;
        buffer_[1] = 0.0f;
        buffer_[2] = 0.0f;
        idx_ = 0;
    }

    float update(float input) {
        // 存入环形缓冲区
        buffer_[idx_] = input;
        idx_ = (idx_ + 1) % 3;

        // 复制数据用于排序
        float a = buffer_[0];
        float b = buffer_[1];
        float c = buffer_[2];

        // 排序网络 (Sorting Network) 找出中值
        // 只需要找出中间那个数，不需要完全排序
        if (a > b) { float t = a; a = b; b = t; } // swap(a,b)
        if (b > c) { float t = b; b = c; c = t; } // swap(b,c)
        if (a > b) { float t = a; a = b; b = t; } // swap(a,b)
        
        return b; // b 就是中值
    }

private:
    float buffer_[3];
    uint8_t idx_;
};

static MedianFilter3 mf_vx, mf_vy;

// IMU加速度与光流速度融合的卡尔曼滤波
static Kalman2DPosVel kf_pose;
static bool kf_pose_inited = false;
static float kf_last_px = 0.0f;
static float kf_last_py = 0.0f;
static float last_imu_angle_z = 0.0f;

OptFlow::OptFlow() : initialized_(false) {
    memset(&state_, 0, sizeof(state_));
}

void OptFlow::reset() {
    memset(&state_, 0, sizeof(state_));
    initialized_ = false;
    kf_pose_inited = false;
    mf_vx.reset();
    mf_vy.reset();
    last_imu_angle_z = 0.0f;
}

void OptFlow::process(const Data_t& sensor_data, float imu_omega_z, float imu_angle_z, float imu_acc_x, float imu_acc_y) {
    // Update timestamp
    state_.time_ms = HAL_GetTick();
    state_.time_us = TIM13->CNT;
    
    // Store raw data (axes swapped in hardware)
    state_.raw_x = sensor_data.y;
    state_.raw_y = sensor_data.x;
    
    // First sample guard
    if (!initialized_) {
        state_.last_x = state_.raw_x;
        state_.last_y = state_.raw_y;
        state_.last_time_ms = state_.time_ms;
        state_.last_time_us = state_.time_us;
        kf_pose_inited = false;
        mf_vx.reset();
        mf_vy.reset();
        last_imu_angle_z = imu_angle_z;
        initialized_ = true;
        return;
    }
    
    // Compute delta
    state_.delta_x = state_.raw_x - state_.last_x;
    state_.delta_y = state_.raw_y - state_.last_y;
    state_.all_distance += sqrtf(state_.delta_x * state_.delta_x + state_.delta_y * state_.delta_y);
    
    // Compute dt in seconds
    // uint32_t t0 = state_.last_time_ms;
    // uint32_t t1 = state_.time_ms;
    // uint32_t dt_ms = (t1 >= t0) ? (t1 - t0) : (t1 + (0xFFFFFFFFu - t0) + 1u);
    // float dt_s = static_cast<float>(dt_ms) / 1000.0f;

    uint32_t t0 = state_.last_time_us;
    uint32_t t1 = state_.time_us;
    uint32_t dt_us = (t1 >= t0) ? (t1 - t0) : (t1 + ((1<<16) - t0) + 1u);
    float dt_s = static_cast<float>(dt_us) / 1000000.0f;
    
    // Clamp dt
    if (dt_s < MIN_DT) dt_s = MIN_DT;
    if (dt_s > MAX_DT) dt_s = MAX_DT;
    state_.dt_s = dt_s;
    
    // Get IMU yaw rate for rigid-body correction
    // 使用角速度积分计算偏航角变化量，比角度差分更平滑
    state_.delta_yaw = imu_omega_z * dt_s * PI / 180.0f;
    
    last_imu_angle_z = imu_angle_z;
    
    // Rigid-body correction
    // float cosdt = cosf(state_.delta_yaw);
    // float sindt = sinf(state_.delta_yaw);
    float sindt = state_.delta_yaw - (state_.delta_yaw * state_.delta_yaw * state_.delta_yaw) / 6.0f; // 小角度近似sin(x)=x
    float cosdt = 1.0f - (state_.delta_yaw * state_.delta_yaw) / 2.0f + (state_.delta_yaw * state_.delta_yaw * state_.delta_yaw * state_.delta_yaw) / 24.0f;        // 小角度近似cos(x)=1
    float dx_rot = (cosdt - 1.0f) * OFFSET_X - sindt * OFFSET_Y;
    float dy_rot = sindt * OFFSET_X + (cosdt - 1.0f) * OFFSET_Y;
    state_.dx_rot = dx_rot;
    state_.dy_rot = dy_rot;
    state_.e = state_.delta_x - dx_rot;
    state_.f = state_.delta_y - dy_rot;
    if (fabsf(state_.delta_yaw) > 0) {
        float L_X = state_.delta_x/state_.delta_yaw;
        float L_Y = state_.delta_y/state_.delta_yaw;
        state_.L_X = L_X;
        state_.L_Y = L_Y;
    }
    
    
    // Compute velocities
    state_.raw_vx = state_.e / dt_s;
    state_.raw_vy = state_.f / dt_s;
    state_.raw_omega = state_.delta_yaw / dt_s;
    
    // 对原始速度进行低通滤波，减少尖刺
    // 使用3点中值滤波去除尖刺，延迟极低（仅1ms左右），相位滞后可以忽略不计
    float filtered_vx = mf_vx.update(state_.raw_vx);
    float filtered_vy = mf_vy.update(state_.raw_vy);
    
    state_.angle = atan2f(state_.e, state_.f);

    //卡尔曼滤波融合
    if (!kf_pose_inited) {
        // q_pos: 位置过程噪声, q_vel: 速度过程噪声, r_vel: 光流速度观测噪声
        kf_pose.setNoise(1e-3f, 5.0f, 200.0f, 1e6f);
        kf_pose.init(0.0f, 0.0f, filtered_vx, filtered_vy, 100.0f);
        kf_last_px = kf_pose.px();
        kf_last_py = kf_pose.py();
        kf_pose_inited = true;
    }

    // 预测：用 IMU 加速度
    kf_pose.predict(imu_acc_x, imu_acc_y, dt_s);
    // 用滤波后的光流速度测量
    kf_pose.updateVel(filtered_vx, filtered_vy);

    // 机器人坐标系下的位移增量（由卡尔曼位置状态差分得到）
    float de_est = kf_pose.px() - kf_last_px;
    float df_est = kf_pose.py() - kf_last_py;
    kf_last_px = kf_pose.px();
    kf_last_py = kf_pose.py();
    
    // Update global position
    state_.global_x = kf_pose.px();
    state_.global_y = kf_pose.py();
    
    float yaw_now_rad = (imu_angle_z - (-108.67f)) / 180.0f * PI;
    float yaw_mid_rad = yaw_now_rad + 0.5f * state_.delta_yaw;
    //state_.global_x += state_.e * cosf(yaw_mid_rad) + state_.f * sinf(yaw_mid_rad);
    //state_.global_y += -state_.e * sinf(yaw_mid_rad) + state_.f * cosf(yaw_mid_rad);
    state_.global_x += de_est * cosf(yaw_mid_rad) + df_est * sinf(yaw_mid_rad);
    state_.global_y += -de_est * sinf(yaw_mid_rad) + df_est * cosf(yaw_mid_rad);
    state_.global_vx = kf_pose.vx();
    state_.global_vy = kf_pose.vy();

    state_.last_x = state_.raw_x;
    state_.last_y = state_.raw_y;
    state_.last_time_ms = state_.time_ms;
    state_.last_time_us = state_.time_us;
}
