#include "mbed.h"
#include "MicroBitPin.h"
#include "MicroBitQDec.h"
#include "GenericMotor.h"

#include <limits.h>

#ifndef MICROBIT_TACHOMOTOR_H
#define MICROBIT_TACHOMOTOR_H

class QDecSpeed
{
    static const int  taps = 8;
    int64_t           positionHistory[taps];
    uint32_t          tickHistory[taps];
    const uint32_t    sampleInterval = 5000;
    int32_t           positionDelta;
    int32_t           timeDelta;
    uint8_t           windowPos;

    public:

    void reset(int64_t position, uint32_t tick);
    void update(int64_t position, uint32_t tick);
    int32_t getSpeed(void) const;
};

class TachoMotor : public MicroBitComponent
{
    static const int pollPeriod = 2000;
    static const int hysteresis = 3;

    GenericMotor& motor;
    MicroBitQDec& qdec;
    QDecSpeed speed;
    Ticker ticker;

    public:

    enum Mode {
        MOTOR_SLEEP = 0,
        MOTOR_COAST,
        MOTOR_BRAKE,
        MOTOR_POWER,                    // set specific power
        MOTOR_SPEED,                    // set specific speed
        MOTOR_TRACK,                    // follow constantly-changing position
        MOTOR_POSITION                  // active feedback to maintain position
    };

    TachoMotor(uint16_t id, GenericMotor& mtr, MicroBitQDec& qd)
        : motor(mtr), qdec(qd) { this->id = id; }

    protected:

    int start(void);

    void stop(void);

    struct PIDState {
        int32_t error;
        int64_t sigma;
        int32_t delta;

        void reset(void) {
            error = 0;
            sigma = 0;
            delta = 0;
        }

        void update(int64_t target, int64_t current);
        int32_t output(int32_t p, int32_t i, int32_t d) const;
    };

    private:

    Mode state = MOTOR_SLEEP;
    Mode nextState = MOTOR_SLEEP;
    int64_t targetPosition;
    int32_t targetSpeed;
    PIDState pid;
    int8_t duty;

    void setState(Mode s);
    void setNextState(int64_t where, Mode s);
    virtual void pidTick(void);

    protected:
    virtual int followSpeed(PIDState& pid, int8_t duty) const;
    virtual int followPosition(PIDState& pid, int8_t duty) const;

    public:
    int32_t speedP = 1576;
    int32_t speedI = 100;
    int32_t speedD = 0;
    int32_t positionP = 6 * 65536;
    int32_t positionI = 0;
    int32_t positionD = 0;

    void goTo(int64_t target, Mode andThen = MOTOR_BRAKE);
    void sleep(void) { setState(MOTOR_SLEEP); }
    void coast(void) { setState(MOTOR_COAST); }
    void brake(void) { setState(MOTOR_BRAKE); }
    void go(uint8_t duty_percent) {
        duty = duty_percent;
        setState(MOTOR_POWER);
    }
    void goAt(int32_t speed) {
        targetSpeed = speed;
        setState(MOTOR_SPEED);
    }

    int64_t getPosition(void) { return qdec.getPosition(); }
    int64_t getSpeed(void) { return speed.getSpeed(); }


    int64_t getPosition(void) const { return qdec.getPosition(); }
    int64_t getSpeed(void) const { return speed.getSpeed(); }

#if 1 /* debug fluff */
    void peek(int64_t& target, int32_t& speed, int8_t& duty, char const*& mode) {
        target = targetPosition;
        speed = targetSpeed;
        duty = this->duty;
        switch (state) {
        case MOTOR_SLEEP:     mode = "SLEEP"; break;
        case MOTOR_COAST:     mode = "COAST"; break;
        case MOTOR_BRAKE:     mode = "BRAKE"; break;
        case MOTOR_POWER:     mode = "POWER"; break;
        case MOTOR_SPEED:     mode = "SPEED"; break;
        case MOTOR_TRACK:     mode = "TRACK"; break;
        case MOTOR_POSITION:  mode = "POSITION"; break;
        default:              mode = "???";
        }
    }
    void pidpeek(int32_t& e, int32_t& s, int32_t& d) {
        e = pid.error;
        s = pid.sigma;
        d = pid.delta;
    }
    int64_t triggerPosition = 0;
#endif
};

#endif
