#ifndef __OPT_FLOW_HPP
#define __OPT_FLOW_HPP

#include "Task/utils.hpp"

#include <cstdint>
#include <cstddef>

#define OPTFLOW_OFFSET_X (115.0f)
#define OPTFLOW_OFFSET_Y (10.5f)

class OptFlow {
public:
    struct Data_t {
        float x;
        float y;
    };
    
    struct State_t {
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
        uint32_t time_ms;
        uint32_t last_time_ms;
        uint32_t time_us;
        uint32_t last_time_us;
        float dt_s;
        float delta_yaw;
        float angle;
        float e;
        float f;
        float all_distance;
    };
    
    OptFlow();
    ~OptFlow() = default;
    
    void process(const Data_t& sensor_data, float imu_omega_z, float imu_angle_z);
    void process(const Data_t& sensor_data, float imu_omega_z, float imu_angle_z, float imu_acc_x, float imu_acc_y);
    const State_t& get_state() const { return state_; }
    void reset();
    
private:
    static constexpr float OFFSET_X = -115.0f;
    static constexpr float OFFSET_Y = 10.5f;
    static constexpr float MIN_DT = 0.001f;
    static constexpr float MAX_DT = 0.1f;
    
    State_t state_;
    bool initialized_;
};

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

class Kalman2DPosVel {
public:
    Kalman2DPosVel();
    void setNoise(float q_pos, float q_vel, float r_vel, float r_pos = 1e6f);
    void init(float px0, float py0, float vx0, float vy0, float p0 = 1.0f);
    void predict(float ax, float ay, float dt);
    void updateVel(float vx_meas, float vy_meas);
    void updatePos(float px_meas, float py_meas);
    float px() const { return x_[0]; }
    float py() const { return x_[1]; }
    float vx() const { return x_[2]; }
    float vy() const { return x_[3]; }
private:
    float x_[4];
    float P_[16];
    float Qd_[4];
    float Rv_[2];
    float Rp_[2];
};

#endif // __OPT_FLOW_HPP
