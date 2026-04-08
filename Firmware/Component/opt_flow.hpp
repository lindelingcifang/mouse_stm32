#ifndef __OPT_FLOW_HPP
#define __OPT_FLOW_HPP

#include "Task/utils.hpp"

#include <cstdint>
#include <cstddef>

// ============================================================
// [MODIFIED] 双光流传感器间距参数
//   原: OPTFLOW_OFFSET_X / OPTFLOW_OFFSET_Y (单光流安装偏移)
//   现: 双光流基线半长，单位 mm，对应 md 推导中的 l
// ============================================================
#define OPTFLOW_HALF_BASELINE_MM (34.0f)

// [UNCHANGED] valid_mask 位定义
#define OPTFLOW_MASK_LEFT  (0x01u)
#define OPTFLOW_MASK_RIGHT (0x02u)

class Kalman2DPosVel {
public:
    Kalman2DPosVel();
    void setNoise(float q_pos, float q_vel, float r_vel, float r_pos);
    void init(float px0, float py0, float vx0, float vy0, float p0);
    void predict(float ax, float ay, float dt);   // IMU 加速度预测
    void updateVel(float vx_meas, float vy_meas); // 光流速度更新
    void updatePos(float px_meas, float py_meas); // 光流位置更新（可选）
    float px() const { return x_[0]; }
    float py() const { return x_[1]; }
    float vx() const { return x_[2]; }
    float vy() const { return x_[3]; }
private:
    float Q_[4];   // 过程噪声对角线
    float Rv_[2];  // 速度观测噪声
    float Rp_[2];  // 位置观测噪声
    float x_[4];   // 状态 [px, py, vx, vy]（单位：mm 和 mm/s）
    float P_[16];  // 协方差矩阵（行主序）
    // 注意：原代码用 Qd_ 命名，统一改为 Q_ 避免混淆
};

class OptFlow {
public:
    // ============================================================
    // [MODIFIED] Data_t: 单光流单路输入 -> 双光流双路输入
    //   原字段: float x, y
    //   现字段: left_x/y, right_x/y, tick_ms, valid_mask
    // ============================================================
    struct Data_t {
        float        left_x;
        float        left_y;
        float        right_x;
        float        right_y;
        unsigned int tick_ms;
        unsigned int valid_mask;
        unsigned int left_tick_ms;   // Left separate timestamp 
        unsigned int right_tick_ms;  // Right separate timestamp 
        // --- 新增 IMU 字段 ---
        float imu_acc_x;   // 本体坐标系加速度 X，m/s²（BMI088 输出）
        float imu_acc_y;   // 本体坐标系加速度 Y，m/s²
        float imu_omega_z; // 陀螺仪 Z 轴角速度，rad/s（DPS 转换后）
        bool  imu_valid;   // IMU 数据是否有效
    };

    // ============================================================
    // [MODIFIED] State_t: 重构为双光流输出
    //   保留: time_ms, last_time_ms, dt_s（时间相关）
    //   新增: left_vx/vy, right_vx/vy（左右传感器各自速度）
    //         body_vx/vy（机器人本体速度，由 md 推导 dx_robot/dt）
    //         omega_z（机器人角速度，dtheta/dt）
    //         raw_dtheta（每帧角度增量，调试用）
    //   注释掉: 单光流相关字段（raw_x/y, delta_x/y, global_x/y 等）
    //           卡尔曼相关字段（global_vx/vy 等）
    //           待 IMU 引入后可恢复
    // ============================================================
    struct State_t {
        // 左右传感器各自速度（调试用）
        float left_vx;
        float left_vy;
        float right_vx;
        float right_vy;

        // 机器人本体速度（md 推导结果）
        float body_vx;   // dx_robot / dt，前进方向
        float body_vy;   // dy_robot / dt，横移方向
        float omega_z;   // dtheta / dt，偏航角速度 rad/s

        // 每帧原始角度增量（调试用）
        float raw_dtheta;

        // 纯双光流积分得到的中心位姿（不依赖 IMU/KF）
        float flow_px;
        float flow_py;
        float flow_yaw;

        // 时间
        unsigned int time_ms;
        unsigned int last_time_ms;
        float        dt_s;

        // --- 新增：卡尔曼融合输出 ---
        float kf_vx;              // 融合后本体速度 X，mm/s
        float kf_vy;              // 融合后本体速度 Y，mm/s
        float kf_px;              // 融合后位置 X，mm（可用于里程计）
        float kf_py;              // 融合后位置 Y，mm
        float kf_omega_z;         // 融合后偏航角速度 rad/s（互补滤波）
        float flow_quality;       // 光流质量评分 [0,1]，用于调试动态权重
        float flow_weight;        // 实际光流权重 [0,1]（越小越依赖 IMU）

        /* -------------------------------------------------------
         * [COMMENTED OUT] 单光流时代的状态字段，暂时不用
         * 待后续加入 IMU / 全局位姿估计时恢复
         * -------------------------------------------------------
        float raw_x;
        float raw_y;
        float last_x;
        float last_y;
        float delta_x;
        float delta_y;
        float global_x;
        float global_y;
        float global_vx;
        float global_vy;
        float raw_vx;
        float raw_vy;
        float raw_omega;
        uint32_t time_us;
        uint32_t last_time_us;
        float delta_yaw;
        float angle;
        float e;
        float f;
        float all_distance;
        float L_X;
        float L_Y;
        float dx_rot;
        float dy_rot;
         * ------------------------------------------------------- */
    };

    OptFlow();
    ~OptFlow() = default;

    // ============================================================
    // [MODIFIED] process 接口
    //   原: process(Data_t, imu_omega_z, imu_angle_z [, imu_acc_x, imu_acc_y])
    //   现: process(Data_t)，暂不需要 IMU 参数
    // ============================================================
    void process(const Data_t& sensor_data);

    const State_t& get_state() const { return state_; }
    void reset();

private:
    // ============================================================
    // [MODIFIED] 常量
    //   原: OFFSET_X/Y（单光流安装偏移，用于刚体修正）
    //   现: HALF_BASELINE（双光流基线半长）
    // ============================================================
    static constexpr float HALF_BASELINE = OPTFLOW_HALF_BASELINE_MM;
    static constexpr float MIN_DT = 0.001f;   // [UNCHANGED]
    static constexpr float MAX_DT = 0.1f;     // [UNCHANGED]
    static constexpr float kQPos = 0.1f;
    static constexpr float kQVel = 5.0f;
    static constexpr float kRVelMin = 300.0f;
    static constexpr float kRVelMax = 12000.0f;
    static constexpr float kRPos = 1e6f;
    static constexpr float kMinUpdateQuality = 0.08f;
    static constexpr float kResidualBadMmPerS = 1800.0f;
    static constexpr float kZeroSpeedMmPerS = 25.0f;
    static constexpr float kAccelActiveMmPerS2 = 400.0f;
    static constexpr uint8_t kZeroStreakBad = 6u;

    State_t      state_;

    // ============================================================
    // [MODIFIED] 初始化标志：拆分为左右独立
    //   原: bool initialized_（单个共享标志，有突变风险）
    //   现: 左右各自独立，某侧第一帧只记录位置不输出速度，
    //       避免另一侧延迟上线时 last 值为 0 导致速度尖峰
    // ============================================================
    bool         left_initialized_;
    bool         right_initialized_;

    // [NEW] 上一帧左右传感器位置和时间戳
    float        left_last_x_;
    float        left_last_y_;
    float        right_last_x_;
    float        right_last_y_;
    unsigned int last_time_ms_;

     // --- 新增卡尔曼私有成员 ---
    Kalman2DPosVel kf_;
    bool kf_inited_;
    float kf_last_px_;
    float kf_last_py_;
    uint8_t flow_zero_streak_;

    // --- 新增互补滤波私有成员（用于 omega_z）---
    float cf_omega_z_;        // 互补滤波后的 omega_z
    static constexpr float kCfAlpha = 0.7f; // 光流权重
};

// ============================================================
// [COMMENTED OUT] Kalman1D —— 单光流 IMU 融合用，暂不启用
//   待引入 IMU 后取消注释
// ============================================================
/*
class Kalman1D {
public:
    Kalman1D(float Q = 0.01f, float R = 1.0f, float P = 1.0f, float x0 = 0.0f);
    void init(float Q, float R, float P, float x0);
    float update(float z);
    float state() const { return x; }
private:
    float Q;
    float R;
    float P;
    float x;
};
*/

// ============================================================
// [COMMENTED OUT] Kalman2DPosVel —— 单光流 IMU 融合用，暂不启用
//   待引入 IMU 后取消注释
// ============================================================
/*

*/

#endif // __OPT_FLOW_HPP