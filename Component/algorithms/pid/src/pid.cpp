#include "pid.h"

namespace pid
{
    float PID::calc(float ref, float fdb)
    {
        last_err = err;
        err = ref - fdb;

        diff = (err - last_err) / parameter_.dt;
        integ = limit<float>(integ + err * parameter_.dt, parameter_.integLimit);

        output = limit<float>(parameter_.kp * err + parameter_.ki * integ + parameter_.kd * diff,
                              parameter_.outputLimit);

        return output;
    }
}