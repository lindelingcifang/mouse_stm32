#include "cmsis_os.h"
#include "freertos_vars.h"
#include "can_callbacks.h"
#include "z_main.h"

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

float compute_velocity(float current_value, float previous_value, unsigned int current_tick_ms, unsigned int previous_tick_ms) {
    unsigned int dt_ms = (current_tick_ms >= previous_tick_ms) ?
        (current_tick_ms - previous_tick_ms) :
        (current_tick_ms + (0xFFFFFFFFu - previous_tick_ms) + 1u);

    if (dt_ms == 0) {
        return 0.0f;
    }

    return (current_value - previous_value) * (1000.0f / static_cast<float>(dt_ms));
}

void update_side_state(OptFlowSideState& side_state, float new_x, float new_y, unsigned int tick_ms) {
    if (!side_state.initialized) {
        side_state.x = new_x;
        side_state.y = new_y;
        side_state.tick_ms = tick_ms;
        side_state.vx = 0.0f;
        side_state.vy = 0.0f;
        side_state.initialized = true;
        return;
    }

    if (new_x == side_state.x && new_y == side_state.y) {
        return;
    }

    side_state.vx = compute_velocity(new_x, side_state.x, tick_ms, side_state.tick_ms);
    side_state.vy = compute_velocity(new_y, side_state.y, tick_ms, side_state.tick_ms);
    side_state.x = new_x;
    side_state.y = new_y;
    side_state.tick_ms = tick_ms;
}

OptFlowSideState left_flow_state;
OptFlowSideState right_flow_state;

}

float dual_flow_left_x;
float dual_flow_left_y;
float dual_flow_right_x;
float dual_flow_right_y;
float dual_flow_left_vx;
float dual_flow_left_vy;
float dual_flow_right_vx;
float dual_flow_right_vy;
float raw_vx, raw_vy;
float body_vx, body_vy, omega_z;
unsigned int mouse_time_ms;
unsigned int optflow_valid_mask;


extern "C" {

// OptFlowRxTask - Process dual optical flow CAN snapshots and estimate body velocity.
void StartOptFlowRxTask(void *argument) {
    osDelay(100);  // Wait for initialization

    DualOptFlowSnapshot_t snapshot;
    
    for(;;) {
        // Block waiting for the latest dual optical flow CAN snapshot.
        if (osMessageQueueGet(q_optflow_dataHandle, &snapshot, NULL, osWaitForever) == osOK) {
            dual_flow_left_x = snapshot.left_x;
            dual_flow_left_y = snapshot.left_y;
            dual_flow_right_x = snapshot.right_x;
            dual_flow_right_y = snapshot.right_y;
            optflow_valid_mask = snapshot.valid_mask;
            mouse_time_ms = snapshot.tick_ms;

            if (optflow_valid_mask & kOptFlowMaskLeft) {
                update_side_state(left_flow_state, dual_flow_left_x, dual_flow_left_y, mouse_time_ms);
                dual_flow_left_vx = left_flow_state.vx;
                dual_flow_left_vy = left_flow_state.vy;
            }

            if (optflow_valid_mask & kOptFlowMaskRight) {
                update_side_state(right_flow_state, dual_flow_right_x, dual_flow_right_y, mouse_time_ms);
                dual_flow_right_vx = right_flow_state.vx;
                dual_flow_right_vy = right_flow_state.vy;
            }

            if ((optflow_valid_mask & (kOptFlowMaskLeft | kOptFlowMaskRight)) == (kOptFlowMaskLeft | kOptFlowMaskRight)) {
                raw_vx = 0.5f * (dual_flow_left_vx + dual_flow_right_vx);
                raw_vy = 0.5f * (dual_flow_left_vy + dual_flow_right_vy);
                omega_z = (dual_flow_right_vx - dual_flow_left_vx) / kDualOptFlowBaselineMm;
            } else if (optflow_valid_mask & kOptFlowMaskLeft) {
                raw_vx = dual_flow_left_vx;
                raw_vy = dual_flow_left_vy;
                omega_z = 0.0f;
            } else if (optflow_valid_mask & kOptFlowMaskRight) {
                raw_vx = dual_flow_right_vx;
                raw_vy = dual_flow_right_vy;
                omega_z = 0.0f;
            } else {
                raw_vx = 0.0f;
                raw_vy = 0.0f;
                omega_z = 0.0f;
            }

            body_vx = raw_vx;
            body_vy = raw_vy;
        }
    }
}

} // extern "C"

