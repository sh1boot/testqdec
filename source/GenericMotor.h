#include "mbed.h"
#include "MicroBitPin.h"

#ifndef GENERIC_MOTOR_H
#define GENERIC_MOTOR_H

class GenericMotor : public MicroBitComponent
{
    MicroBitPin& forward;
    MicroBitPin& reverse;
    const uint32_t dutyCyclePeriod;

    public:

    GenericMotor(MicroBitPin& fwd, MicroBitPin& rev, uint32_t period_us = 100) : forward(fwd), reverse(rev), dutyCyclePeriod(period_us) {}

    virtual void sleep(void);
    virtual void brake(void);
    virtual void coast(void);
    virtual void powerFastDecay(int8_t duty_percent);
    virtual void powerSlowDecay(int8_t duty_percent);

    uint32_t getDutyCyclePeriod(void) const { return dutyCyclePeriod; }
};

#endif
