/*
The MIT License (MIT)

Copyright (c) 2016-2017 Simon Hosie

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
#include "MicroBitMessageBus.h"
#include "MicroBitQuadratureDecoder.h"

#include <limits.h>

#ifndef MICROBIT_SOFTQDEC_H
#define MICROBIT_SOFTQDEC_H

class SoftQuadratureDecoder : public MicroBitQuadratureDecoder
{
    MicroBitMessageBus& eventBus;
    uint32_t livestamp, latchstamp;
    int32_t countstate = 0;
    uint32_t speed;
    uint16_t listenId;

    void onEdgeEvent(MicroBitEvent e); // when phaseA changes, check B and update counter accordingly

    public:
    /**
      * Constructor.
      * Create a software abstraction of the quadrature decoder.
      *
      * @param id                 The message bus id of phaseA.
      * @param messageBus         The message bus on which to listen for edges.
      * @param phaseA             Pin connected to quadrature encoder output A
      * @param phaseB             Pin connected to quadrature encoder output B
      * @param LED                The pin for the LED to enable during each quadrature reading
      * @param LEDDelay           Number of microseconds after LED activation before sampling
      *
      * @code
      * MicroBitQuadratureDecoder qdec(QDEC_ID, QDEC_PHA, QDEC_PHB, QDEC_LED);
      * @endcode
      */
    SoftQuadratureDecoder(uint16_t id, MicroBitMessageBus& eventBus_, MicroBitPin& phaseA_, MicroBitPin& phaseB_, MicroBitPin& LED_, uint8_t LEDDelay_ = 0, uint8_t flags_ = 0)
        : MicroBitQuadratureDecoder(phaseA_, phaseB_, LED_, LEDDelay_, flags_), eventBus(eventBus_) { listenId = id; }
    SoftQuadratureDecoder(uint16_t id, MicroBitMessageBus& eventBus_, MicroBitPin& phaseA_, MicroBitPin& phaseB_, uint8_t flags_ = 0)
        : MicroBitQuadratureDecoder(phaseA_, phaseB_, flags_), eventBus(eventBus_) { listenId = id; }

    /**
      * Set the rate at which input pins are sampled.
      *
      * @param  The maximum interval between samples in microseconds.
      *
      * @return MICROBIT_OK on success, or MICROBIT_INVALID_PARAMETER if the configuration is invalid.
      */
    int setSamplePeriodUs(uint32_t period);

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
    virtual int start() override;

    /**
      * Stop the hardware and make it available for use by other instances.
      */
    virtual void stop() override;

    /** Poll hardware for latest decoder movement and reset the hardware counter to zero.
      *
      * This must be called regularly to prevent the hardware from overflowing.
      * About ten times per second, or less if the attached hardware is
      * guaranteed to count more slowly than 10000 encoder counts per second.
      *
      * This call may be made from systemTick(), or a dedicated motor control ticker interrupt.
      */
    virtual void poll() override;

    /**
      * Reset the position to a known value.
      *
      * This can be used to zero the counter on detection of an index or end-stop signal.
      *
      * @param The value that getPosition() should return at this encoder position.
      */
    virtual void resetPosition(int64_t position = 0) override;
};

#endif
