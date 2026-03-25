#include "cmsis_os.h"
#include "freertos_vars.h"
#include "z_main.h"
#include <cstring>
#include <stdint.h>
#include "Component/ekf_vw.h"

// Include BMI088 driver
#include "bmi088_probe.h"

// IMU data structure
struct ImuData {
    float data[9];  // ax, ay, az, gx, gy, gz, roll, pitch, yaw
};

// Ozone graph debug symbols (global + volatile for stable live watch).
volatile float imu_dbg_acc_x = 0.0f;
volatile float imu_dbg_acc_y = 0.0f;
volatile float imu_dbg_acc_z = 0.0f;
volatile float imu_dbg_gyro_x = 0.0f;
volatile float imu_dbg_gyro_y = 0.0f;
volatile float imu_dbg_gyro_z = 0.0f;
volatile float imu_dbg_angle_x = 0.0f;
volatile float imu_dbg_angle_y = 0.0f;
volatile float imu_dbg_angle_z = 0.0f;
volatile uint32_t imu_dbg_update_tick_ms = 0u;
volatile uint32_t imu_dbg_update_count = 0u;

volatile float ekf_dbg_v = 0.0f;
volatile float ekf_dbg_w = 0.0f;
volatile float ekf_dbg_abx = 0.0f;
volatile float ekf_dbg_wbz = 0.0f;

// BMI088 initialization status
volatile uint8_t bmi088_acc_id = 0xFF;
volatile uint8_t bmi088_gyro_id = 0xFF;
volatile uint8_t bmi088_init_ok = 0;

extern "C" {

// ImuRxTask - Process BMI088 data via SPI
void StartImuRxTask(void *argument) {
    osDelay(100);  // Wait for initialization

    constexpr float kDegToRad = 0.01745329251994329577f;
    uint32_t last_predict_tick_ms = 0u;
    
    // Wait a bit for system stabilization
    HAL_Delay(50);
    
    // Read BMI088 chip IDs using local non-volatile buffers,
    // then publish to volatile debug globals for Ozone watch.
    uint8_t acc_id = 0xFF;
    uint8_t gyro_id = 0xFF;
    bmi088_probe_ids(&acc_id, &gyro_id);
    bmi088_acc_id = acc_id;
    bmi088_gyro_id = gyro_id;
    
    // Initialize BMI088 once at startup. If it fails, task will retry.
    bmi088_init_ok = bmi088_init_minimal();
    
    // BMI088 raw data buffers
    bmi088_raw_data_t acc_data, gyro_data;
    
    for(;;) {
        // Retry init until sensor is online to avoid permanent startup failure.
        if (bmi088_init_ok == 0U) {
            uint8_t acc_id = 0xFF;
            uint8_t gyro_id = 0xFF;
            bmi088_probe_ids(&acc_id, &gyro_id);
            bmi088_acc_id = acc_id;
            bmi088_gyro_id = gyro_id;
            bmi088_init_ok = bmi088_init_minimal();

            // Back off to reduce bus pressure when device is not ready.
            osDelay(20);
            continue;
        }

        // Acquire robot state mutex
        if (osMutexAcquire(mtx_robot_stateHandle, 10) == osOK) {
            
            // Read raw data from BMI088 sensors
            if (bmi088_read_raw(&acc_data, &gyro_data)) {
                // Decode BMI088 raw data to standard IMU format
                imu.decode_bmi088(&acc_data, &gyro_data);

                float imu_frame[9];
                imu.get_data(imu_frame);
                imu_dbg_acc_x = imu_frame[0];
                imu_dbg_acc_y = imu_frame[1];
                imu_dbg_acc_z = imu_frame[2] - 39.7f;  // Remove gravity offset (24g * 9.8m/s²)
                imu_dbg_gyro_x = imu_frame[3];
                imu_dbg_gyro_y = imu_frame[4];
                imu_dbg_gyro_z = imu_frame[5];
                imu_dbg_angle_x = imu_frame[6];
                imu_dbg_angle_y = imu_frame[7];
                imu_dbg_angle_z = imu_frame[8];
                imu_dbg_update_tick_ms = HAL_GetTick();
                imu_dbg_update_count++;

                // EKF predict: use IMU high-rate feedforward with measured dt.
                {
                    uint32_t now_tick_ms = imu_dbg_update_tick_ms;
                    if (last_predict_tick_ms != 0u) {
                        uint32_t dt_ms = (now_tick_ms >= last_predict_tick_ms)
                            ? (now_tick_ms - last_predict_tick_ms)
                            : (now_tick_ms + (0xFFFFFFFFu - last_predict_tick_ms) + 1u);

                        if (dt_ms > 0u) {
                            float dt = static_cast<float>(dt_ms) * 0.001f;
                            float ax_mmps2 = imu_frame[0] * 1000.0f;
                            float ay_mmps2 = imu_frame[1] * 1000.0f;
                            float wz_radps = imu_frame[5] * kDegToRad;

                            EKF_Predict(ax_mmps2, ay_mmps2, wz_radps, dt);
                        }
                    }
                    last_predict_tick_ms = now_tick_ms;

                    EKF_t *ekf = EKF_GetHandle();
                    ekf_dbg_v = ekf->x[0];
                    ekf_dbg_w = ekf->x[2];
                    ekf_dbg_abx = ekf->x[3];
                    ekf_dbg_wbz = ekf->x[5];
                }
            }
            
            // Signal IMU data ready
            osSemaphoreRelease(sem_imu_readyHandle);
            
            osMutexRelease(mtx_robot_stateHandle);
        }
        
        // BMI088 runs at 1600Hz (accel) / 2000Hz (gyro)
        // Update at reasonable rate (e.g., ~200Hz)
        osDelay(5);
    }
}

} // extern "C"
