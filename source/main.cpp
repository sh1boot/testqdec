#include "mbed.h"
#include "MicroBitSerial.h"
#include "MicroBitPin.h"
#include "MicroBitQDec.h"

MicroBitSerial serial(USBTX, USBRX);
#if 0 // Kitronik motor driver board
MicroBitQDec qdec(MICROBIT_PIN_P1, BUTTON_B);
#else
MicroBitQDec qdec(MICROBIT_PIN_P0, MICROBIT_PIN_P1);
#endif

int main()
{
    qdec.start();
     
    for (;;)
    {
        int x = qdec.read();
        printf("%6d  \r", x);
        fflush(stdout);
        wait_ms(51);
    }
}
