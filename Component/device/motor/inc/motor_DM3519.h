#ifndef _MOTOR_DM3519_H_
#define _MOTOR_DM3519_H_

#include "motor_base.h"

namespace motor
{
    static const float redu_rat = 3519.0 / 187.0;

    const MotorInfo MotorInfo_DM3519{
        .rx_id = 0x200,
        .tx_id = 0x200,
        .raw_input_limit = 16384,
        .motor_curr_limit = 20.5,
        .broad_curr_limit = 1,
    };

    enum DM3519_State : uint8_t
    {
        kStateCodeMotorDisable = 0x0,        // 电机失能
        kStateCodeMotorEnable = 0x1,         // 电机使能
        kStateCodeMotorSensorError = 0x5,    // 读取传感器错误
        kStateCodeMotorParameterError = 0x6, // 读取电机参数错误
        kStateCodeOverVolt = 0x8,            // 过压
        kStateCodeUnderVolt = 0x9,           // 欠压
        kStateCodeOverCurr = 0xA,            // 过流
        kStateCodeMosOverTemp = 0xB,         // MOS过温
        kStateCodeCoilOverTemp = 0xC,        // 电机线圈过温
        kStateCodeCommLoss = 0xD,            // 通信丢失
        kStateCodeOverload = 0xE,            // 过载
    };

    enum DM3519_Cmd
    {
        kCmdNormal = 0x0,   // 正常指令
        kCmdClearErr = 0x1, // 清除错误
    };

    class Motor_DM3519 : public Motor
    {
    public:
        explicit Motor_DM3519(const MotorParameter motor_parameter_t);
        ~Motor_DM3519() {};
        void encode(uint8_t *tx_data);
        void decode(const uint8_t rx_data[8]);
        float temp(void) const { return temp_; };
        DM3519_State state(void) const { return state_; };

    private:
        float temp_ = 0;
        DM3519_State state_ = kStateCodeMotorEnable;
        DM3519_Cmd cmd_ = kCmdNormal;
        using Motor::torq;
    };
}

#endif