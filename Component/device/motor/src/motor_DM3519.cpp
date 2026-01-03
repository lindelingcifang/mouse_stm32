#include "motor_DM3519.h"

namespace motor
{

    Motor_DM3519::Motor_DM3519(const MotorParameter motor_parameter_t) : Motor()
    {
        motor_info = MotorInfo_DM3519;
        motor_info.rx_id += motor_parameter_t.id;

        motor_parameter = motor_parameter_t;
    }

    void Motor_DM3519::encode(uint8_t *tx_data)
    {
        uint8_t data_H = 0x00;
        uint8_t data_L = 0x00;

        data_H = raw_input_ >> 8;
        data_L = raw_input_;

        if (motor_parameter.id <= 4)
        {
            tx_data[(motor_parameter.id - 1) * 2] = data_H;
            tx_data[(motor_parameter.id - 1) * 2 + 1] = data_L;
        }
    }

    void Motor_DM3519::decode(const uint8_t rx_data[8])
    {
        float raw_angle_ = 0;
        float raw_vel_ = 0;
        float raw_curr_ = 0;

        raw_angle_ = 360 * ((((float)((((int16_t)rx_data[0]) << 8) | (int16_t)rx_data[1]))) / 8191);
        raw_vel_ = (int16_t)(((rx_data[2]) << 8) | rx_data[3]);
        raw_curr_ = 20.5 * ((int16_t)((rx_data[4] << 8) | rx_data[5])) / 16384.0;

        if (motor_parameter.remove_build_in_reducer)
        {
            raw_angle_ = raw_angle_ * motor_parameter.ex_redu_rat;
            raw_vel_ = raw_vel_ * motor_parameter.ex_redu_rat;
        }

        angle_ = (float)motor_parameter.dir * normalize<float>(raw_angle_ * motor_parameter.ex_redu_rat, 360);
        vel_ = (float)motor_parameter.dir * raw_vel_ * motor_parameter.ex_redu_rat;
        curr_ = (float)motor_parameter.dir * raw_curr_;
        temp_ = (float)rx_data[6];
        state_ = (DM3519_State)rx_data[7];
    }

} // namespace motor
