#ifndef _PID_H_
#define _PID_H_

#include "base.h"

namespace pid
{
    typedef struct _parameter_
    {
        float kp;
        float ki;
        float kd;
        float outputLimit;
        float integLimit;
        float dt;
    }parameter;
    

    class PID
    {
    public:    
        PID(parameter parameter_t):parameter_(parameter_t){};
        ~PID() = default;
        float calc(float ref , float fdb);

    private:
        parameter parameter_;
        float err = 0;
        float last_err = 0;
        float integ = 0;
        float diff = 0;
        float output = 0;

    };
}

#endif