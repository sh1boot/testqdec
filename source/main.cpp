#include "mbed.h"
#include "MicroBitSerial.h"
#include "MicroBitPin.h"
#include "MicroBitQDec.h"

MicroBitSerial serial(USBTX, USBRX);
#if 0 // Kitronik motor driver board
MicroBitPin P1(MICROBIT_ID_IO_P1, MICROBIT_PIN_P1, PIN_CAPABILITY_ALL);
MicroBitPin P0(MICROBIT_ID_IO_P11, MICROBIT_PIN_P11, PIN_CAPABILITY_DIGITAL);
MicroBitQDec qdec(MICROBIT_ID_IO_P1, MICROBIT_PIN_P1, MICROBIT_PIN_P11);
#else
MicroBitPin P0(MICROBIT_ID_IO_P0, MICROBIT_PIN_P0, PIN_CAPABILITY_ALL);
MicroBitPin P1(MICROBIT_ID_IO_P1, MICROBIT_PIN_P1, PIN_CAPABILITY_ALL);

MicroBitQDec qdec(MICROBIT_ID_IO_P0, P0, P1);
#endif

int main()
{
    qdec.start();
     
    int oldx = qdec.getPosition();
    for (;;)
    {
        int x = qdec.getPosition();
        int v = qdec.getSpeed();
        printf("%6d  %6d  \r", x, v);
        fflush(stdout);
        wait_ms(51);

        /* add a wee snooze here to see what happens when we don't poll */
        if (oldx < 720 && x >= 720)
            wait_ms(3000);
        oldx = x;
    }
}
