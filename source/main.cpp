#include "mbed.h"
#include "MicroBitSerial.h"
#include "MicroBitPin.h"
#include "TachoMotor.h"
#include "SoftQDec.h"
#include "ErrorNo.h"

MicroBitSerial serial(USBTX, USBRX);

MicroBitMessageBus bus;

MicroBitPin P0(MICROBIT_ID_IO_P0, MICROBIT_PIN_P0, PIN_CAPABILITY_ALL);
MicroBitPin P1(MICROBIT_ID_IO_P1, MICROBIT_PIN_P1, PIN_CAPABILITY_ALL);
MicroBitPin P2(MICROBIT_ID_IO_P2, MICROBIT_PIN_P2, PIN_CAPABILITY_ALL);
MicroBitPin P8(MICROBIT_ID_IO_P8, MICROBIT_PIN_P8, PIN_CAPABILITY_STANDARD);
MicroBitPin P11(MICROBIT_ID_IO_P11, MICROBIT_PIN_P11, PIN_CAPABILITY_STANDARD);
MicroBitPin P12(MICROBIT_ID_IO_P12, MICROBIT_PIN_P12, PIN_CAPABILITY_STANDARD);
MicroBitPin P13(MICROBIT_ID_IO_P13, MICROBIT_PIN_P13, PIN_CAPABILITY_STANDARD);
MicroBitPin P14(MICROBIT_ID_IO_P14, MICROBIT_PIN_P14, PIN_CAPABILITY_STANDARD);
MicroBitPin P15(MICROBIT_ID_IO_P15, MICROBIT_PIN_P15, PIN_CAPABILITY_STANDARD);
MicroBitPin P16(MICROBIT_ID_IO_P16, MICROBIT_PIN_P16, PIN_CAPABILITY_STANDARD);

#if 0 // Kitronik motor driver board
MicroBitQuadratureDecoder qd(P1, P11);
MicroBitMotor motor(P12, P16);
#else
MicroBitQuadratureDecoder qd(P0, P1);
GenericMotor motor(P15, P16);

SoftQuadratureDecoder qdb(MICROBIT_ID_IO_P2, bus, P2, P8);
GenericMotor motorb(P13, P14);
#endif
TachoMotor tmot(12345, motor, qd);
TachoMotor tmotb(12345, motorb, qdb);

int main()
{
    char const* command = "";

    // Lego motors have series resistors on their quadrature output, which
    // appear to be used for part identification via ADC.
    // Make sure we're not fighting with that output.
    P0.getDigitalValue(PullNone);
    P1.getDigitalValue(PullNone);
    P2.getDigitalValue(PullNone);
    P8.getDigitalValue(PullNone);
    P11.getDigitalValue(PullNone);

    for (;;)
    {
        int key;
        int pstep = abs(tmot.positionP) / 32 + 1;
        int istep = abs(tmot.positionI) / 32 + 1;
        int dstep = abs(tmot.positionD) / 32 + 1;
        while ((key = serial.read(ASYNC)) != MICROBIT_NO_DATA) {
            switch (key) {
            case '0': command = "sleep";
                tmot.sleep();
                break;
            case '1': command = "coast";
                tmot.coast();
                break;
            case '2': command = "brake";
                tmot.brake();
                break;
            case '3': command = "go(85)";
                tmot.go(85);
                break;
            case '4': command = "goAt(-720)";
                tmot.goAt(-720);
                break;
            case '5': command = "goAt(-1440)";
                tmot.goAt(-1440);
                break;
            case '6': command = "goTo(0)";
                tmot.goTo(0);
                break;
            case '7': command = "goTo(-720, POSITION)";
                tmot.goTo(-720, tmot.MOTOR_POSITION);
                break;
            case '8': command = "goTo(0, POSITION)";
                tmot.goTo(0, tmot.MOTOR_POSITION);
                break;
            case '9': command = "goTo(720, POSITION)";
                tmot.goTo(720, tmot.MOTOR_POSITION);
                break;
            case 'P':
                tmot.positionP -= pstep;
                break;
            case 'p':
                tmot.positionP += pstep;
                break;
            case 'I':
                tmot.positionI -= istep;
                break;
            case 'i':
                tmot.positionI += istep;
                break;
            case 'D':
                tmot.positionD -= dstep;
                break;
            case 'd':
                tmot.positionD += dstep;
                break;
            }
        }

        int64_t target;
        int32_t tspeed;
        int8_t power;
        char const* mode;
        int64_t position = tmot.getPosition();
        int32_t speed = tmot.getSpeed();
        int32_t error, sigma, delta;
        tmot.peek(target, tspeed, power, mode);
        tmot.pidpeek(error, sigma, delta);
        printf("\033[1;1H%s\033[K\r\n"
                "position: %6d       speed: %5d      error: %6d        mode: %s  \033[K\r\n"
                "  target: %6d      target: %5d      sigma: %6d       power: %5d  \033[K\r\n"
                "   error: %6d       error: %5d      delta: %6d  \033[K\r\n"
                " tripped: %6d  \033[K\r\n"
                "\033[K\r\n"
                "    posP: %8d    posI: %8d    posD: %8d  \033[K\r\n\033[K\r\n",
                command,
                (int)position, (int)speed, (int)error, mode,
                (int)target, (int)tspeed, (int)sigma, power,
                (int)(position - target), (int)(speed - tspeed), (int)delta,
                (int)tmot.triggerPosition,
                (int)tmot.positionP, (int)tmot.positionI, (int)tmot.positionD);
        wait_ms(49);
    }
}
