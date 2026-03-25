#include "cmsis_os.h"
#include "freertos_vars.h"
#include "z_main.h"
#include <cstring>

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

// BMI088 initialization status
volatile uint8_t bmi088_acc_id = 0xFF;
volatile uint8_t bmi088_gyro_id = 0xFF;
volatile uint8_t bmi088_init_ok = 0;

extern "C" {

// ImuRxTask - Process BMI088 data via SPI
void StartImuRxTask(void *argument) {
    osDelay(100);  // Wait for initialization
    
    // Wait a bit for system stabilization
    HAL_Delay(50);
    
    // Read BMI088 chip IDs
    uint8_t acc_id = 0xFF;
    uint8_t gyro_id = 0xFF;
    bmi088_probe_ids(&acc_id, &gyro_id);
    bmi088_acc_id = acc_id;
    bmi088_gyro_id = gyro_id;
    
    // Initialize BMI088
    bmi088_init_ok = bmi088_init_minimal();
    
    // BMI088 raw data buffers
    bmi088_raw_data_t acc_data, gyro_data;
    
    for(;;) {
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
                imu_dbg_acc_z = imu_frame[2];
                imu_dbg_gyro_x = imu_frame[3];
                imu_dbg_gyro_y = imu_frame[4];
                imu_dbg_gyro_z = imu_frame[5];
                imu_dbg_angle_x = imu_frame[6];
                imu_dbg_angle_y = imu_frame[7];
                imu_dbg_angle_z = imu_frame[8];
                imu_dbg_update_tick_ms = HAL_GetTick();
                imu_dbg_update_count++;

                osSemaphoreRelease(sem_imu_readyHandle);//修改到if里面
            }
            
            // Signal IMU data ready
            
            
            osMutexRelease(mtx_robot_stateHandle);
        }
        
        // BMI088 runs at 1600Hz (accel) / 2000Hz (gyro)
        // Update at reasonable rate (e.g., ~200Hz)
        osDelay(5);
    }
}

} // extern "C"
