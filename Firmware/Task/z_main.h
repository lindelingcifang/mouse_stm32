#ifndef __Z_MAIN_H
#define __Z_MAIN_H

// Hardware configuration
#include "board.h"

#include "Task/can_callbacks.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#define PI_UART_TX_DATA_LENGTH 3
#define PI_UART_RX_DATA_LENGTH 7
#define SPI_LENGTH 38

#define INFRARED_THRESHOLD (3.3f)

const uint8_t piRxFrameHeader = 0xbb;
const float bat_k = 12.27;
const float cap_k = 113.73;

#ifdef __cplusplus
}

#include "interfaces.hpp"

// Component includes
#include "Component/motor.hpp"
#include "Component/imu.hpp"
#include "Component/opt_flow.hpp"
#include "Component/robot.hpp"

// Task includes
#include "Task/utils.hpp"
#include "Communication/can/z_can.hpp"

// Forward declarations to ensure types are known before externs
class Robot;

extern Robot robot;

extern ZCAN can1_bus;
extern ZCAN can2_bus;
extern IMU imu;
extern OptFlow opt_flow;

extern volatile float imu_dbg_acc_x;
extern volatile float imu_dbg_acc_y;
extern volatile float imu_dbg_acc_z;
extern volatile float imu_dbg_gyro_x;
extern volatile float imu_dbg_gyro_y;
extern volatile float imu_dbg_gyro_z;
extern volatile float imu_dbg_angle_x;
extern volatile float imu_dbg_angle_y;
extern volatile float imu_dbg_angle_z;
extern volatile uint32_t imu_dbg_update_tick_ms;
extern volatile uint32_t imu_dbg_update_count;

extern volatile uint8_t bmi088_acc_id;
extern volatile uint8_t bmi088_gyro_id;
extern volatile uint8_t bmi088_init_ok;

extern volatile float ekf_dbg_v;
extern volatile float ekf_dbg_w;
extern volatile float ekf_dbg_abx;
extern volatile float ekf_dbg_wbz;

extern float ekf_vx;
extern float ekf_vy;
extern float ekf_w;


#endif // __cplusplus

#endif // __Z_MAIN_H