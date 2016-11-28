#include "mbed.h"
#include "GenericMotor.h"

void GenericMotor::sleep(void) {
    coast();
}

void GenericMotor::brake(void) {
    forward.setDigitalValue(1);
    reverse.setDigitalValue(1);
}

void GenericMotor::coast(void) {
    forward.setDigitalValue(0);
    reverse.setDigitalValue(0);
}

void GenericMotor::powerFastDecay(int8_t duty_percent) {
    int32_t pwmvalue = duty_percent * MICROBIT_PIN_MAX_OUTPUT / 100;

    if (pwmvalue >= MICROBIT_PIN_MAX_OUTPUT) {
        forward.setDigitalValue(1);
    } else if (pwmvalue > 0) {
        forward.setAnalogPeriodUs(dutyCyclePeriod);
        forward.setAnalogValue(pwmvalue);
    } else {
        forward.setDigitalValue(0);
    }
    if (pwmvalue <= -MICROBIT_PIN_MAX_OUTPUT) {
        reverse.setDigitalValue(1);
    } else if (pwmvalue < 0) {
        reverse.setAnalogPeriodUs(dutyCyclePeriod);
        reverse.setAnalogValue(-pwmvalue);
    } else {
        reverse.setDigitalValue(0);
    }
}

void GenericMotor::powerSlowDecay(int8_t duty_percent) {
    int32_t pwmvalue = duty_percent * MICROBIT_PIN_MAX_OUTPUT / 100;

    if (pwmvalue >= MICROBIT_PIN_MAX_OUTPUT) {
        reverse.setDigitalValue(0);
    } else if (pwmvalue > 0) {
        reverse.setAnalogPeriodUs(dutyCyclePeriod);
        reverse.setAnalogValue(MICROBIT_PIN_MAX_OUTPUT - pwmvalue);
    } else {
        reverse.setDigitalValue(1);
    }
    if (pwmvalue <= -MICROBIT_PIN_MAX_OUTPUT) {
        forward.setDigitalValue(0);
    } else if (pwmvalue < 0) {
        forward.setAnalogPeriodUs(dutyCyclePeriod);
        forward.setAnalogValue(MICROBIT_PIN_MAX_OUTPUT + pwmvalue);
    } else {
        forward.setDigitalValue(1);
    }
}
