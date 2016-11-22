#include "mbed.h"
#include "MicroBitSerial.h"
#include "MicroBitPin.h"
#include "MicroBitQDec.h"

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
        if (value > 0) forward.setAnalogValue(value);
        else forward.setDigitalValue(1);
        if (value < 0) reverse.setAnalogValue(-value);
        else reverse.setDigitalValue(1);
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

    private:

    Mode state = MOTOR_SLEEP;
    Mode nextState = MOTOR_SLEEP;
    int64_t targetPosition;
    int32_t targetSpeed;
    int power;

    void setState(Mode s) {
        nextState = state = s;
        if (state == MOTOR_SLEEP && s != MOTOR_SLEEP)
            start();
        switch (s) {
        case MOTOR_SLEEP:
            if (state != MOTOR_SLEEP) {
                power = 0;
                motor.coast();
                stop();
            }
            break;
        case MOTOR_COAST:
            motor.coast();
            break;
        case MOTOR_BRAKE:
            motor.brake();
            break;
        case MOTOR_POWER:
            motor.power(power);
            break;
        case MOTOR_SPEED:
        case MOTOR_TRACK:
        case MOTOR_POSITION:
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
            if ((q > 0 && p >= targetPosition) || (q < 0 && p <= targetPosition))
                setState(nextState);
        }
        switch (state) {
        case MOTOR_SPEED:
            power += speedProfile(targetSpeed - q);
            if (power > MICROBIT_PIN_MAX_OUTPUT) power = MICROBIT_PIN_MAX_OUTPUT;
            if (power < -MICROBIT_PIN_MAX_OUTPUT) power = -MICROBIT_PIN_MAX_OUTPUT;
            motor.power(power);
        case MOTOR_TRACK:
            power += positionProfile(targetPosition - p);
            if (power > MICROBIT_PIN_MAX_OUTPUT) power = MICROBIT_PIN_MAX_OUTPUT;
            if (power < -MICROBIT_PIN_MAX_OUTPUT) power = -MICROBIT_PIN_MAX_OUTPUT;
            motor.power(power);
            break;
            break;
        case MOTOR_POSITION:
            power += positionProfile(targetPosition - p);
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
    virtual int positionProfile(int64_t d) {
        if (d < 0) return -10;
        if (d > 0) return 10;
        return 0;
    }
    virtual int speedProfile(int32_t d) {
        if (d < 0) return -10;
        if (d > 0) return 10;
        return 0;
    }

    public:

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

    void peek(int64_t& target, int32_t& speed, int& power) {
        target = targetPosition;
        speed = targetSpeed;
        power = this->power;
    }
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
    char const *modestr = "";
    for (;;)
    {
        switch (++mode) {
        default:
            mode = 0;
            /*@fallthrough@*/
        case 0: modestr = "SLEEP",
            qdec.sleep();
            break;
        case 1: modestr = "COAST",
            qdec.coast();
            break;
        case 2: modestr = "BRAKE",
            qdec.brake();
            break;
        case 3: modestr = "POWER",
            qdec.go(MICROBIT_PIN_MAX_OUTPUT / 3);
            break;
        case 4: modestr = "SPEED",
            qdec.goAt(-300);
            break;
        case 5: modestr = "GOTOK",
            qdec.goTo(0);
            break;
        case 6: modestr = "GOTOP",
            qdec.goTo(720, qdec.MOTOR_POSITION);
            break;
        }
        for (int i = 0; i < 100; i++) {
            int64_t target;
            int32_t tspeed;
            int power;
            int64_t position = qdec.getPosition();
            int32_t speed = qdec.getSpeed();
            qdec.peek(target, tspeed, power);
            printf("%6s: tp:%8d p:%8d    ts:%6d: s:%8d   pwr:%4d    \r",
                    modestr,
                    (int)target, (int)position,
                    (int)tspeed, (int)speed,
                    power);
            fflush(stdout);
            wait_ms(49);
        }
    }
}
