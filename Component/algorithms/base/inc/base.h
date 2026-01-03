#ifndef _BASE_H_
#define _BASE_H_

#include "string.h"
#include "stm32f4xx_hal.h"
#include "math.h"


template <typename T>
T limit(T data_t, T limit_t)
{
    if (limit_t < 0)
    {
        limit_t = -limit_t;
    }

    if (data_t > limit_t)
    {
        return limit_t;
    }
    else if (data_t < -limit_t)
    {
        return -limit_t;
    }
    else
    {
        return data_t;
    }
}

template <typename T>
T normalize(T data_t,T cycle_t)
{
    if(cycle_t<0)
    {
        cycle_t = -cycle_t;
    }

    while(data_t>cycle_t)
    {
        data_t -=cycle_t;
    }

    while(data_t<-cycle_t)
    {
        data_t +=cycle_t;
    }

    return data_t;
}





#endif