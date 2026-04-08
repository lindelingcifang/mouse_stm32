// ============================================================
// [MODIFIED] optflow_task1.cpp
//   原: Task 层自己计算速度（维护 OptFlowSideState、手动差分）
//   现: Task 层只负责队列接收 + 数据转发，所有速度计算移入
//        Component (OptFlow::process)，与单光流架构保持一致
// ============================================================

#include "cmsis_os.h"
#include "freertos_vars.h"
#include "can_callbacks.h"
#include "z_main.h"
// [NEW] 引入 Component 头文件
#include "Component/opt_flow.hpp"
#include <cstring>

extern volatile uint8_t bmi088_init_ok;  // 声明 imu_task.cpp 中定义的变量
// ============================================================
// [COMMENTED OUT] 原 Task 层内嵌的速度计算辅助结构和函数
//   逻辑已整体移入 OptFlow::process()，此处完整保留供参考
// ============================================================
/*
namespace {

constexpr unsigned int kOptFlowMaskLeft = 0x01u;
constexpr unsigned int kOptFlowMaskRight = 0x02u;
constexpr float kOptFlowHalfBaselineMm = 16.3f;
constexpr float kDualOptFlowBaselineMm = 2.0f * kOptFlowHalfBaselineMm;

struct OptFlowSideState {
    float x = 0.0f;
    float y = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;
    unsigned int tick_ms = 0;
    bool initialized = false;
};

float compute_velocity(float current_value, float previous_value,
                       unsigned int current_tick_ms, unsigned int previous_tick_ms) {
    unsigned int dt_ms = (current_tick_ms >= previous_tick_ms) ?
        (current_tick_ms - previous_tick_ms) :
        (current_tick_ms + (0xFFFFFFFFu - previous_tick_ms) + 1u);
    if (dt_ms == 0) return 0.0f;
    return (current_value - previous_value) * (1000.0f / static_cast<float>(dt_ms));
}

void update_side_state(OptFlowSideState& side_state,
                       float new_x, float new_y, unsigned int tick_ms) {
    if (!side_state.initialized) {
        side_state.x = new_x; side_state.y = new_y;
        side_state.tick_ms = tick_ms;
        side_state.vx = 0.0f; side_state.vy = 0.0f;
        side_state.initialized = true;
        return;
    }
    if (new_x == side_state.x && new_y == side_state.y) return;
    side_state.vx = compute_velocity(new_x, side_state.x, tick_ms, side_state.tick_ms);
    side_state.vy = compute_velocity(new_y, side_state.y, tick_ms, side_state.tick_ms);
    side_state.x = new_x; side_state.y = new_y;
    side_state.tick_ms = tick_ms;
}

OptFlowSideState left_flow_state;
OptFlowSideState right_flow_state;

} // namespace
*/

// ============================================================
// [NEW] Component 实例（对应单光流中的 opt_flow 全局对象）
// ============================================================
// OptFlow opt_flow; // Already defined in main.cpp

// ============================================================
// [UNCHANGED] 对外暴露的全局变量（供其他模块读取）
//   raw_vx/vy、body_vx/vy、omega_z 语义与原代码一致
//   新增 left/right 各自速度供调试
// ============================================================
float dual_flow_left_x;
float dual_flow_left_y;
float dual_flow_right_x;
float dual_flow_right_y;
// [MODIFIED] 左右速度现在从 Component state 读取，不再由 Task 自算
float dual_flow_left_vx;
float dual_flow_left_vy;
float dual_flow_right_vx;
float dual_flow_right_vy;
float raw_vx, raw_vy;
float body_vx, body_vy, omega_z;
float robot_pos_x_mm, robot_pos_y_mm;
float flow_px, flow_py, flow_yaw;
unsigned int mouse_time_ms;
unsigned int optflow_valid_mask;


extern "C" {

// ============================================================
// [MODIFIED] StartOptFlowRxTask
//   原: 收到快照 -> 调 update_side_state -> 手动计算 raw_vx/vy/omega_z
//   现: 收到快照 -> 填充 OptFlow::Data_t -> opt_flow.process()
//        -> 从 state 读取结果，赋值给全局变量
// ============================================================
void StartOptFlowRxTask(void *argument) {
    osDelay(100);  // [UNCHANGED] 等待初始化完成

    DualOptFlowSnapshot_t snapshot;

    for(;;) {
        if (osMessageQueueGet(q_optflow_dataHandle, &snapshot, NULL, osWaitForever) == osOK) {

            // --------------------------------------------------
            // [UNCHANGED] 原始快照数据透传给全局变量（调试用）
            // --------------------------------------------------
            dual_flow_left_x   = snapshot.left_x;
            dual_flow_left_y   = snapshot.left_y;
            dual_flow_right_x  = snapshot.right_x;
            dual_flow_right_y  = snapshot.right_y;
            optflow_valid_mask = snapshot.valid_mask;
            mouse_time_ms      = snapshot.tick_ms;

            // --------------------------------------------------
            // [NEW] 组装 Component 输入，调用 process
            // --------------------------------------------------
            OptFlow::Data_t data;
            data.left_x     = snapshot.left_x;
            data.left_y     = snapshot.left_y;
            data.right_x    = snapshot.right_x;
            data.right_y    = snapshot.right_y;
            data.tick_ms    = snapshot.tick_ms;
            data.valid_mask = snapshot.valid_mask;

            // 新增：填充 IMU 字段
            // 注意：imu 对象由 imu_task 持续更新，这里直接读取最新值
            // opt_flow 任务不需要等信号量，直接拿当前帧 IMU 数据与当前帧光流融合
            float imu_frame[9];
            imu.get_data(imu_frame);
            // imu_frame[0/1] = acc_x/y (m/s²)
            // imu_frame[5]   = gyro_z  (DPS) → 转为 rad/s
            data.imu_acc_x   = imu_frame[0];
            data.imu_acc_y   = imu_frame[1];
            data.imu_omega_z = imu_frame[5] * (3.14159f / 180.0f);
            data.imu_valid   = (bmi088_init_ok != 0);  // bmi088_init_ok 在 imu_task 中声明

            opt_flow.process(data);

            // --------------------------------------------------
            // [NEW] 从 Component state 读取结果，赋值给全局变量
            // --------------------------------------------------
            const OptFlow::State_t& s = opt_flow.get_state();

            dual_flow_left_vx  = s.left_vx;
            dual_flow_left_vy  = s.left_vy;
            dual_flow_right_vx = s.right_vx;
            dual_flow_right_vy = s.right_vy;

            // body_vx/vy/omega_z：机器人本体速度，与单光流变量命名保持一致
            body_vx = s.kf_vx;
            body_vy = s.kf_vy;
            omega_z = s.omega_z;
            robot_pos_x_mm = s.flow_px;
            robot_pos_y_mm = s.flow_py;
            flow_px = s.flow_px;
            flow_py = s.flow_py;
            flow_yaw = s.flow_yaw;

            // raw 保留原始光流，方便调试对比
            raw_vx = s.body_vx;
            raw_vy = s.body_vy;

            // --------------------------------------------------
            // [COMMENTED OUT] 原 Task 层速度计算逻辑，完整保留供参考
            // --------------------------------------------------
            /*
            if (optflow_valid_mask & kOptFlowMaskLeft) {
                update_side_state(left_flow_state,
                    dual_flow_left_x, dual_flow_left_y, mouse_time_ms);
                dual_flow_left_vx = left_flow_state.vx;
                dual_flow_left_vy = left_flow_state.vy;
            }
            if (optflow_valid_mask & kOptFlowMaskRight) {
                update_side_state(right_flow_state,
                    dual_flow_right_x, dual_flow_right_y, mouse_time_ms);
                dual_flow_right_vx = right_flow_state.vx;
                dual_flow_right_vy = right_flow_state.vy;
            }
            if ((optflow_valid_mask & (kOptFlowMaskLeft | kOptFlowMaskRight))
                    == (kOptFlowMaskLeft | kOptFlowMaskRight)) {
                raw_vx  = 0.5f * (dual_flow_left_vx + dual_flow_right_vx);
                raw_vy  = 0.5f * (dual_flow_left_vy + dual_flow_right_vy);
                omega_z = (dual_flow_right_vx - dual_flow_left_vx)
                          / kDualOptFlowBaselineMm;
            } else if (optflow_valid_mask & kOptFlowMaskLeft) {
                raw_vx = dual_flow_left_vx;
                raw_vy = dual_flow_left_vy;
                omega_z = 0.0f;
            } else if (optflow_valid_mask & kOptFlowMaskRight) {
                raw_vx = dual_flow_right_vx;
                raw_vy = dual_flow_right_vy;
                omega_z = 0.0f;
            } else {
                raw_vx = 0.0f; raw_vy = 0.0f; omega_z = 0.0f;
            }
            body_vx = raw_vx;
            body_vy = raw_vy;
            */
        }
    }
}

} // extern "C"