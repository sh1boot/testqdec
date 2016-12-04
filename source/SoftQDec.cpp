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
    eventBus.listen(listenId, MICROBIT_PIN_EVT_RISE, this, &SoftQuadratureDecoder::onRiseEvent, MESSAGE_BUS_LISTENER_IMMEDIATE);
    eventBus.listen(listenId, MICROBIT_PIN_EVT_FALL, this, &SoftQuadratureDecoder::onFallEvent, MESSAGE_BUS_LISTENER_IMMEDIATE);
    phaseA.eventOn(MICROBIT_PIN_EVENT_ON_EDGE);
    return MICROBIT_OK;
}

/**
  * Stop the hardware and make it available for use by other instances.
  */
void SoftQuadratureDecoder::stop()
{
    phaseA.eventOn(MICROBIT_PIN_EVENT_NONE);
    eventBus.ignore(listenId, MICROBIT_PIN_EVT_RISE, this, &SoftQuadratureDecoder::onRiseEvent);
    eventBus.ignore(listenId, MICROBIT_PIN_EVT_FALL, this, &SoftQuadratureDecoder::onFallEvent);
}

int events = 0;

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
//    position += current - (int32_t)position;
    position = current;
//    position = events;
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

static inline int32_t updateCountState(int32_t state, int A, int B) {
    if (B == 0)
        state += 1 - 2 * A;
    return (state & ~3) | A * 3;
}

void SoftQuadratureDecoder::onRiseEvent(MicroBitEvent e)
{
    int A = 1;
    int B = phaseB.getDigitalValue();
    countstate = updateCountState(countstate, A, B);
    if (B == 0)
    {
        livestamp = e.timestamp;
        speed = livestamp - latchstamp;
        events++;
    }
    else
    {
        latchstamp = livestamp;
        events -= 100;
    }
}

void SoftQuadratureDecoder::onFallEvent(MicroBitEvent e)
{
    int A = 0;
    int B = phaseB.getDigitalValue();
    countstate = updateCountState(countstate, A, B);
    if (B == 0)
    {
        livestamp = e.timestamp;
        speed = livestamp - latchstamp;
        events += 10000;
    }
    else
    {
        latchstamp = livestamp;
        events -= 1000000;
    }
}
