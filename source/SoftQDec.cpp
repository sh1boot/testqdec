/*
The MIT License (MIT)

Copyright (c) 2016 British Broadcasting Corporation.
This software is provided by Lancaster University by arrangement with the BBC.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#include "mbed.h"
#include "MicroBitSystemTimer.h"
#include "SoftQDec.h"

#include <limits.h>

/**
  * Set the rate at which input pins are sampled.
  *
  * @param  The maximum interval between samples in microseconds.
  *
  * @return MICROBIT_OK on success, or MICROBIT_INVALID_PARAMETER if the configuration is invalid.
  */
int SoftQuadratureDecoder::setSamplePeriodUs(uint32_t period)
{
    return MicroBitQuadratureDecoder::setSamplePeriodUs(period);
}

/**
  * Configure the hardware to keep this instance up to date.
  *
  * Several instances can exist so long as no more than one of them is
  * attached to the hardware.  This can be a practical way to control
  * several motors with their own encoders if they run only at different
  * times.
  *
  * While the hardware is active, `poll()` must be called
  *
  * @return MICROBIT_OK on success, MICROBIT_BUSY if the hardware is already attached to another instance, or MICROBIT_INVALID_PARAMETER if the configuration is invalid.
  */
int SoftQuadratureDecoder::start()
{
    livestamp = latchstamp = system_timer_current_time_us();
    eventBus.listen(listenId, MICROBIT_PIN_EVT_RISE, this, &SoftQuadratureDecoder::onEdgeEvent, MESSAGE_BUS_LISTENER_IMMEDIATE);
    eventBus.listen(listenId, MICROBIT_PIN_EVT_FALL, this, &SoftQuadratureDecoder::onEdgeEvent, MESSAGE_BUS_LISTENER_IMMEDIATE);
    phaseA.eventOn(MICROBIT_PIN_EVENT_ON_EDGE);
    return MICROBIT_OK;
}

/**
  * Stop the hardware and make it available for use by other instances.
  */
void SoftQuadratureDecoder::stop()
{
    phaseA.eventOn(MICROBIT_PIN_EVENT_NONE);
    eventBus.ignore(listenId, MICROBIT_PIN_EVT_RISE, this, &SoftQuadratureDecoder::onEdgeEvent);
    eventBus.ignore(listenId, MICROBIT_PIN_EVT_FALL, this, &SoftQuadratureDecoder::onEdgeEvent);
}

/** Poll hardware for latest decoder movement and reset the hardware counter to zero.
  *
  * This must be called regularly to prevent the hardware from overflowing.
  * About ten times per second, or less if the attached hardware is
  * guaranteed to count more slowly than 10000 encoder counts per second.
  *
  * This call may be made from systemTick(), or a dedicated motor control ticker interrupt.
  */
void SoftQuadratureDecoder::poll()
{
    int32_t current = countstate ^ phaseB.getDigitalValue();
    position += current - (int32_t)position;
}

/**
  * Reset the position to a known value.
  *
  * This can be used to zero the counter on detection of an index or end-stop signal.
  *
  * @param The value that getPosition() should return at this encoder position.
  */
void SoftQuadratureDecoder::resetPosition(int64_t position)
{
    countstate = (position & ~3) | phaseA.getDigitalValue() * 3;
    this->position = position;
}

void SoftQuadratureDecoder::onEdgeEvent(MicroBitEvent e)
{
    int A = (e.value == MICROBIT_PIN_EVT_RISE);
    int B = phaseB.getDigitalValue();
    int32_t state = countstate;

    A = !A; // Reverse polarity -- would normally swap pins to achieve this, but there's only one safe clock pin here

    if (B == 0)
    {
        // So... the bottom two bits of state contain the old value of A.
        // We expect this to be the opposite of what A is now, but in the case
        // where we drop an interrupt because A is glitching too quickly, it's
        // possible that A is the same as it was the last time we looked.  In
        // that case the final count should be unchanged.
        //
        // If A is high then we subtract 1, if A is low we add 1, so...
        // if A was low and goes high, we subtract 1 from 0 and borrow from bit 2.
        // If A was high and goes low, we add 1 to 3 and carry to bit 2.
        // If A was low and stays low, we subtract 1 from 3 and no borrow.  Later we reassert the 0 in state[1:0] and there is no change.
        // If A was high and stays high, we add 1 to zero and no carry.  Later we reassert the 3 in state[1:0] and there is no change.
        state += 1 - 2 * A;
        livestamp = e.timestamp;
        speed = livestamp - latchstamp;
    }
    else
        latchstamp = livestamp;

    // Set the bottom two bits of the counter to two copies of A.  The bottom
    // two bits of a quadrature encoder count can be inferred from the current
    // states of its outputs.  Bit 1 is A (just like we assert here), and bit 0
    // is A eor B.  When we read the count value in poll() we exclusive-or it
    // with B to make the reading complete and up-to-date.
    //
    // We want that exclusive-or to be with the value of A seen at the last
    // interrupt, because that value is consistent with the rest of the bits of
    // the count.  That's why we duplicate it here rather than testing both
    // pins at poll().
    countstate = (state & ~3) | A * 3;
}
