#include "mbed.h"
#include "TachoMotor.h"
#include "ErrorNo.h"

#include <limits.h>

void QDecSpeed::reset(int64_t position, uint32_t tick) {
    for (int i = 0; i < taps; i++) {
        positionHistory[i] = position;
        tickHistory[i] = tick;
    }
    windowPos = 0;
    positionDelta = 0;
    timeDelta = 0;
}

void QDecSpeed::update(int64_t position, uint32_t tick) {
    int32_t elapsed = tick - tickHistory[windowPos];
    if ((uint32_t)elapsed > sampleInterval * taps * 10) reset(position, tick);
    else if ((uint32_t)elapsed >= sampleInterval) {
        windowPos = (windowPos + 1) & (taps - 1);
        positionDelta = position - positionHistory[windowPos];
        timeDelta = tick - tickHistory[windowPos];
        positionHistory[windowPos] = position;
        tickHistory[windowPos] = tick;
    }
}

int32_t QDecSpeed::getSpeed(void) const {
    int32_t positionDelta = this->positionDelta;
    int32_t timeDelta = this->timeDelta;
    if (timeDelta == 0)
        return 0;
    return (1000000LL * positionDelta) / timeDelta;
}

void TachoMotor::PIDState::update(int64_t target, int64_t current) {
    int32_t oldError = error;
    error = target - current;
    if (-hysteresis < error && error < hysteresis) error = 0;
    sigma += error;
    delta = error - oldError;
}
int32_t TachoMotor::PIDState::output(int32_t p, int32_t i, int32_t d) const {
    int64_t sum = (int64_t)p * error;
    sum += (int64_t)i * sigma;
    sum += (int64_t)d * delta;
    sum >>= 16;
    if (sum < INT_MIN) return INT_MIN;
    if (sum > INT_MAX) return INT_MAX;
    return (int32_t)sum;
}

int TachoMotor::start(void) {
    int result = qdec.start();
    if (result == MICROBIT_OK)
        ticker.attach_us(this, &TachoMotor::pidTick, pollPeriod);
    return result;
}

void TachoMotor::stop(void) {
    ticker.detach();
    qdec.stop();
}

void TachoMotor::setState(Mode s) {
    Mode oldState = state;
    nextState = state = s;
    if (oldState == MOTOR_SLEEP && state != MOTOR_SLEEP)
        start();
    switch (s) {
    case MOTOR_SLEEP:
        if (oldState != MOTOR_SLEEP) {
            duty = 0;
            motor.sleep();
            stop();
        }
        break;
    case MOTOR_COAST:
        duty = 0;
        motor.coast();
        break;
    case MOTOR_BRAKE:
        duty = 0;
        motor.brake();
        break;
    case MOTOR_POWER:
        motor.powerSlowDecay(duty);
        break;
    case MOTOR_SPEED:
    case MOTOR_TRACK:
        if (oldState != state)
            pid.reset();
        break;
    case MOTOR_POSITION:
        motor.brake();
        pid.reset();
        break;
    }
    state = s;
}

void TachoMotor::setNextState(int64_t where, TachoMotor::Mode s) {
    targetPosition = where;
    nextState = s;
}

void TachoMotor::pidTick(void) {
    qdec.poll();
    int64_t p = qdec.getPosition();
    speed.update(p, us_ticker_read());
    int32_t q = speed.getSpeed();
    if (state != nextState) {
        if ((duty > 0 && p >= targetPosition) || (duty < 0 && p <= targetPosition)) {
            triggerPosition = p;
            setState(nextState);
        }
    }
    switch (state) {
    case MOTOR_SPEED:
        pid.update(targetSpeed, q);
        duty = followSpeed(pid, duty);
        motor.powerSlowDecay(duty);
        break;
    case MOTOR_TRACK:
        pid.update(targetPosition, p);
        duty = followPosition(pid, duty);
        motor.powerSlowDecay(duty);
        break;
    case MOTOR_POSITION:
        pid.update(targetPosition, p);
        duty = followPosition(pid, duty);
        motor.powerSlowDecay(duty);
        break;
    default:
        /* no-op */
        break;
    }
}

int TachoMotor::followSpeed(PIDState& pid, int8_t) const {
    return pid.output(speedP, speedI, speedD);
}

int TachoMotor::followPosition(PIDState& pid, int8_t) const {
    return pid.output(positionP, positionI, positionD);
}

void TachoMotor::goTo(int64_t target, TachoMotor::Mode andThen) {
    int64_t p = qdec.getPosition();
    if (p < target) {
        go(100);
    } else if (target < p) {
        go(-100);
    }
    setNextState(target, andThen);
}
