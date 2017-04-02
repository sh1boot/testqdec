Some test code for changes to the [micro:bit][] driver code, plus a software
quadrature decoder and PID control loop.

The [EV3 intelligent brick][] used to control Lego [Mindstorms][] motors is
kind of pricey, and it sets an upper limit on the number of robotics project
that can be built from a finite set of lego, since every project needs some
kind of controller.

I happened to have a [micro:bit][] on my desk when I was thinking about this
and I decided it might be a helpful to offer driver support for motors like
the Mindstorms ones on much cheaper hardware.  This is kind of the same idea
(though far less developed) as [ev3dev][], but with a [different OS][mbed OS].

Mindstorms motors are straightforward DC motors with a quadrature encoder
attached to the output so that speed and position can be monitored and motor
control adjusted accordingly.

All you need is a micro:bit and a few bits of interface hardware wired
together, like so:

![interface logic][]

Parts: [board][], [edge connector][], [driver][], [connector][]
(cheap in principle but that's all prototyping kits)

The [nRF51822][] on micro:bit has a single hardware quadrature decoder, and
establishing a [driver][microbit-dal] for that is the very first step.

Here we have test code for that driver along with some provisional next steps:
a software quadrature decoder (to monitor two motors concurrently) and some
motor control logic.  I'd intended to contribute to microbit-dal and build upon
those drivers within MicroPython once they were done, but that may no longer be
the right way forward.

[micro:bit]: http://microbit.org/
[ev3dev]: http://www.ev3dev.org/
[EV3 intelligent brick]: https://shop.lego.com/en-US/EV3-Intelligent-Brick-45500
[Mindstorms]: https://www.lego.com/en-us/mindstorms
[mbed OS]: https://www.mbed.com/en/development/mbed-os/
[microbit-dal]: https://github.com/sh1boot/microbit-dal/
[nRF51822]: https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.nrf51/dita/nrf51/pdflinks/ref_manual.html
[interface logic]: doc/interface_board.jpg
[board]: https://www.kitronik.co.uk/5613-bbc-microbit-board-only.html
[edge connector]: https://shop.pimoroni.com/products/edge-connector-breakout-board-for-bbc-micro-bit
[driver]: https://www.pololu.com/product/2130
[connector]: http://www.mindsensors.com/ev3-and-nxt/58-breadboard-connector-kit-for-nxt-or-ev3
