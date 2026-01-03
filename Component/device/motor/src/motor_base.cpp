#include "motor_base.h"

namespace motor
{
    // µÁ¡˜ ‰»Î
    void Motor::setInput(float curr_input_t)
    {
        curr_input_t = limit<float>(curr_input_t, motor_info.broad_curr_limit);

        raw_input_ = motor_parameter.dir * (int16_t)(curr_input_t / motor_info.motor_curr_limit * motor_info.raw_input_limit);
    }
} // namespace motor
