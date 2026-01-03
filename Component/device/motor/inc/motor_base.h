#ifndef _MOTOR_BASE_H_
#define _MOTOR_BASE_H_

#include "base.h"

namespace motor
{
    enum Dir : int8_t
    {
        kDirFwd = 1,  // 正向
        kDirRev = -1, // 反向
    };

    typedef struct _motor_parameter_
    {
        uint8_t id; // 1~4
        Dir dir;
        int16_t ex_redu_rat; // 额外减速比
        bool remove_build_in_reducer;
    } MotorParameter;

    typedef struct _motor_info_
    {
        uint32_t rx_id;
        uint32_t tx_id;
        float raw_input_limit;
        float motor_curr_limit;
        float broad_curr_limit;
    } MotorInfo;

    class Motor
    {
    public:
        Motor() {};
        virtual ~Motor() {};
        void setInput(float);
        virtual void encode(uint8_t *tx_data) = 0;
        virtual void decode(const uint8_t rx_data[8]) = 0;

        float angle() const { return angle_; };
        float vel() const { return vel_; };
        float curr() const { return curr_; };
        float torq() const { return torq_; };
        uint32_t rx_id() const { return motor_info.rx_id; };
        uint32_t tx_id() const { return motor_info.tx_id; };

    protected:
        float angle_ = 0;
        float vel_ = 0;
        float curr_ = 0;
        float torq_ = 0;
        int16_t raw_input_ = 0;
        MotorInfo motor_info;
        MotorParameter motor_parameter;
    };

} // namespace Motor

#endif