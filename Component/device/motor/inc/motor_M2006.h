#ifndef _MOTOR_M2006_H_
#define _MOTOR_M2006_H_

#include "motor_base.h"

namespace motor
{
    const MotorInfo MotorInfo_M2009{
        .rx_id = 0x200,
        .tx_id = 0x1FF,
        .raw_input_limit = 10000,
        .motor_curr_limit = 10,
        .broad_curr_limit = 1,
    };

    class Motor_M2006 : public Motor
    {
    public:
        explicit Motor_M2006(const MotorParameter motor_parameter_t);
        ~Motor_M2006() {};
        void encode(uint8_t *tx_data);
        void decode(const uint8_t rx_data[8]);
    private:
        using Motor::curr;
    };
}

#endif