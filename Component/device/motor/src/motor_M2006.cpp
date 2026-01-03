#include "motor_M2006.h"

namespace motor
{

    Motor_M2006::Motor_M2006(const MotorParameter motor_parameter_t) : Motor()
    {
        motor_info = MotorInfo_M2009;
        motor_info.rx_id += motor_parameter_t.id;
        motor_parameter = motor_parameter_t;
    }

    void Motor_M2006::encode(uint8_t *tx_data)
    {
        uint8_t data_H = 0x00;
        uint8_t data_L = 0x00;

        data_H = raw_input_ >> 8;
        data_L = raw_input_;

        if (motor_parameter.id <= 8)
        {
            tx_data[(motor_parameter.id - 5) * 2] = data_H;
            tx_data[(motor_parameter.id - 5) * 2 + 1] = data_L;
        }
    }

    void Motor_M2006::decode(const uint8_t rx_data[8])
    {
        float raw_angle_ = 0;
        float raw_vel_ = 0;
        float raw_torq_ = 0;

        raw_angle_ = 360 * ((((float)((((int16_t)rx_data[0]) << 8) | (int16_t)rx_data[1]))) / 8191);
        raw_vel_ = (int16_t)(((rx_data[2]) << 8) | rx_data[3]);
        raw_torq_ = (int16_t)((rx_data[4] << 8) | rx_data[5]);

        if (motor_parameter.remove_build_in_reducer)
        {
            raw_angle_ = raw_angle_ * motor_parameter.ex_redu_rat;
            raw_vel_ = raw_vel_ * motor_parameter.ex_redu_rat;
        }

        angle_ = (float)motor_parameter.dir * normalize<float>(raw_angle_ * motor_parameter.ex_redu_rat, 360);
        vel_ = (float)motor_parameter.dir * raw_vel_ * motor_parameter.ex_redu_rat;
        torq_ = (float)motor_parameter.dir * raw_torq_;
    }

} // namespace motor
