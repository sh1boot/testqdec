#include "mbed.h"
#include "MicroBitSerial.h"
#include "MicroBitPin.h"
#include "MicroBitQDec.h"

#include <limits.h>

struct MicroBitMotor : public MicroBitComponent
{
    MicroBitPin& forward;
    MicroBitPin& reverse;

    MicroBitMotor(uint16_t id, MicroBitPin& fwd, MicroBitPin& rev) : forward(fwd), reverse(rev) { this->id = id; }

    void brake() {
        forward.setDigitalValue(1);
        reverse.setDigitalValue(1);
    }
    void coast() {
        forward.setDigitalValue(0);
        reverse.setDigitalValue(0);
    }
    void power(int value) {
        if (value > 0) forward.setAnalogValue(value);
        else forward.setDigitalValue(0);
        if (value < 0) reverse.setAnalogValue(-value);
        else reverse.setDigitalValue(0);
    }
    void powerSlowDecay(int value) {
        // Switch between drive and brake.  This means that the desired
        // direction is always high, and the other direction is modulated to
        // stay _low_ in proportion to the requested power.
        if (value > 0) reverse.setAnalogValue(MICROBIT_PIN_MAX_OUTPUT - value);
        else reverse.setDigitalValue(1);
        if (value < 0) forward.setAnalogValue(MICROBIT_PIN_MAX_OUTPUT + value);
        else forward.setDigitalValue(1);
    }
};

class TachoMotor : public MicroBitQDec
{
    MicroBitMotor& motor;

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

    TachoMotor(uint16_t id, MicroBitMotor& mtr, MicroBitPin& phaseA, MicroBitPin& phaseB)
        : MicroBitQDec(id, phaseA, phaseB), motor(mtr) {}
    TachoMotor(uint16_t id, MicroBitMotor& mtr, MicroBitPin& phaseA, MicroBitPin& phaseB, MicroBitPin& LED)
        : MicroBitQDec(id, phaseA, phaseB, LED), motor(mtr) {}

    protected:

    public:

    struct PIDState {
        int32_t error;
        int64_t sigma;
        int32_t delta;

        void reset(void) {
            error = 0;
            sigma = 0;
            delta = 0;
        }

        void update(int64_t target, int64_t current) {
            int32_t oldError = error;
            error = target - current;
            sigma += error;
            delta = error - oldError;
        }
        int32_t output(int32_t p, int32_t i, int32_t d) {
            int64_t sum = (int64_t)p * error;
            sum += (int64_t)i * sigma;
            sum += (int64_t)d * delta;
            sum >>= 16;
            if (sum < INT_MIN) return INT_MIN;
            if (sum > INT_MAX) return INT_MAX;
            return (int32_t)sum;
        }
    };

    private:

    Mode state = MOTOR_SLEEP;
    Mode nextState = MOTOR_SLEEP;
    int64_t targetPosition;
    int32_t targetSpeed;
    PIDState pid;
    int power;

    void setState(Mode s) {
        Mode oldState = state;
        nextState = state = s;
        if (oldState == MOTOR_SLEEP && state != MOTOR_SLEEP)
            start();
        switch (s) {
        case MOTOR_SLEEP:
            if (oldState != MOTOR_SLEEP) {
                power = 0;
                motor.coast();
                stop();
            }
            break;
        case MOTOR_COAST:
            power = 0;
            motor.coast();
            break;
        case MOTOR_BRAKE:
            power = 0;
            motor.brake();
            break;
        case MOTOR_POWER:
            motor.power(power);
            break;
        case MOTOR_SPEED:
        case MOTOR_TRACK:
        case MOTOR_POSITION:
            pid.reset();
            break;
        }
        state = s;
    }

    void setNextState(int64_t where, Mode s) {
        targetPosition = where;
        nextState = s;
    }

    virtual void systemTick(void) {
        int64_t p = getPosition();
        int32_t q = getSpeed();
        if (state != nextState) {
            if ((power > 0 && p >= targetPosition) || (power < 0 && p <= targetPosition)) {
                triggerPosition = p;
                setState(nextState);
            }
        }
        switch (state) {
        case MOTOR_SPEED:
            pid.update(targetSpeed, q);
            power = followSpeed(pid, power);
            if (power > MICROBIT_PIN_MAX_OUTPUT) power = MICROBIT_PIN_MAX_OUTPUT;
            if (power < -MICROBIT_PIN_MAX_OUTPUT) power = -MICROBIT_PIN_MAX_OUTPUT;
            motor.power(power);
            break;
        case MOTOR_TRACK:
            pid.update(targetPosition, p);
            power = followPosition(pid, power);
            if (power > MICROBIT_PIN_MAX_OUTPUT) power = MICROBIT_PIN_MAX_OUTPUT;
            if (power < -MICROBIT_PIN_MAX_OUTPUT) power = -MICROBIT_PIN_MAX_OUTPUT;
            motor.power(power);
            break;
        case MOTOR_POSITION:
            pid.update(targetPosition, p);
            power = followPosition(pid, power);
            if (power > MICROBIT_PIN_MAX_OUTPUT) power = MICROBIT_PIN_MAX_OUTPUT;
            if (power < -MICROBIT_PIN_MAX_OUTPUT) power = -MICROBIT_PIN_MAX_OUTPUT;
            motor.powerSlowDecay(power);
            break;
        default:
            /* no-op */
            break;
        }
    }

    protected:
    virtual int followSpeed(PIDState& pid, int power) {
        power = pid.output(speedP, speedI, speedD);
        return power;
    }
    virtual int followPosition(PIDState& pid, int power) const {
        power = pid.output(positionP, positionI, positionD);
        return power;
    }

    public:
    int32_t speedP = 16384;
    int32_t speedI = 1000;
    int32_t speedD = 10;
    int32_t positionP = 1 << 20;
    int32_t positionI = 10000;
    int32_t positionD = 100;

    void sleep(void) {
        setState(MOTOR_SLEEP);
    }
    void coast(void) {
        setState(MOTOR_COAST);
    }
    void brake(void) {
        setState(MOTOR_BRAKE);
    }
    void go(int value) {
        power = value;
        setState(MOTOR_POWER);
    }
    void goAt(int32_t speed) {
        targetSpeed = speed;
        setState(MOTOR_SPEED);
    }
    void goTo(int64_t target, Mode andThen = MOTOR_BRAKE) {
        int64_t p = getPosition();
        if (p < target) {
            go(MICROBIT_PIN_MAX_OUTPUT);
        } else if (target < p) {
            go(-MICROBIT_PIN_MAX_OUTPUT);
        }
        setNextState(target, andThen);
    }

    void peek(int64_t& target, int32_t& speed, int& power, char const*& mode) {
        target = targetPosition;
        speed = targetSpeed;
        power = this->power;
        switch (state) {
        case MOTOR_SLEEP:     mode = "SLEEP"; break;
        case MOTOR_COAST:     mode = "COAST"; break;
        case MOTOR_BRAKE:     mode = "BRAKE"; break;
        case MOTOR_POWER:     mode = "POWER"; break;
        case MOTOR_SPEED:     mode = "SPEED"; break;
        case MOTOR_TRACK:     mode = "TRACK"; break;
        case MOTOR_POSITION:  mode = "POSITION"; break;
        default:              mode = "OOPS";
        }
    }
    void pidpeek(int32_t& e, int32_t& s, int32_t& d) {
        e = pid.error;
        s = pid.sigma;
        d = pid.delta;
    }
    int64_t triggerPosition = 0;
};


MicroBitSerial serial(USBTX, USBRX);
#if 0 // Kitronik motor driver board
MicroBitPin P1(MICROBIT_ID_IO_P1, MICROBIT_PIN_P1, PIN_CAPABILITY_ALL);
MicroBitPin P0(MICROBIT_ID_IO_P11, MICROBIT_PIN_P11, PIN_CAPABILITY_DIGITAL);
MicroBitQDec qdec(MICROBIT_ID_IO_P1, MICROBIT_PIN_P1, MICROBIT_PIN_P11);
#else
MicroBitPin P0(MICROBIT_ID_IO_P0, MICROBIT_PIN_P0, PIN_CAPABILITY_ALL);
MicroBitPin P1(MICROBIT_ID_IO_P1, MICROBIT_PIN_P1, PIN_CAPABILITY_ALL);

MicroBitPin P15(MICROBIT_ID_IO_P15, MICROBIT_PIN_P15, PIN_CAPABILITY_AD);
MicroBitPin P16(MICROBIT_ID_IO_P16, MICROBIT_PIN_P16, PIN_CAPABILITY_AD);

MicroBitMotor motor(MICROBIT_ID_IO_P15, P15, P16);
#if 0
MicroBitQDec qdec(MICROBIT_ID_IO_P0, P0, P1);
#else
TachoMotor qdec(MICROBIT_ID_IO_P0, motor, P0, P1);
#endif
#endif


int main()
{
    qdec.start();
    int mode = 0;
    for (;;)
    {
        int delay = 2000;
        switch (++mode) {
        default:
            mode = 0;
            /*@fallthrough@*/
        case 0: qdec.sleep();
            break;
        case 1: qdec.coast();
            break;
        case 2: qdec.brake();
            break;
        case 3: qdec.go(MICROBIT_PIN_MAX_OUTPUT * 2 / 7);
            delay = 2000;
            break;
        case 4: qdec.goTo(0);
            delay = 5000;
            break;
        case 5: qdec.goAt(-720);
            delay = 20000;
            break;
        case 6: qdec.goTo(720, qdec.MOTOR_POSITION);
            delay = 25000;
            break;
        }
        for (int i = 0; i < delay; i += 50) {
            int64_t target;
            int32_t tspeed;
            int power;
            char const* mode;
            int64_t position = qdec.getPosition();
            int32_t speed = qdec.getSpeed();
            int32_t error, sigma, delta;
            qdec.peek(target, tspeed, power, mode);
            qdec.pidpeek(error, sigma, delta);
            printf("\033[1;1H"
                    "position: %6d       speed: %5d      error: %6d        mode: %s  \033[K\r\n"
                    "  target: %6d      target: %5d      sigma: %6d       power: %5d  \033[K\r\n"
                    "   error: %6d       error: %5d      delta: %6d  \033[K\r\n"
                    " tripped: %6d  \033[K\r\n",
                    (int)position, (int)speed, (int)error, mode,
                    (int)target, (int)tspeed, (int)sigma, power,
                    (int)(position - target), (int)(speed - tspeed), (int)delta,
                    (int)qdec.triggerPosition);
            wait_ms(49);
        }
    }
}
