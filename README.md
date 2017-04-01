Some test code for changes to the [micro:bit][] driver code, plus a software
quadrature decoder and PID control loop.

Because I have some Lego and I have a micro:bit, and because Lego's [EV3
intelligent brick][] is kind of pricey, I thought it might be a good idea to
reduce the cost of controlling Lego robots by writing some basic driver support
for Lego's [Mindstorms][] motors on micro:bit.  This would allow kits to be
stretched to more projects at once, rather than just one project per
[expensive] controller.

This is kind of the same idea (but far less developed) as [ev3dev][], but
running on much cheaper hardware and a [different OS][mbed OS].  A few bits of
interface hardware can be soldered together for cheap and motors can be bought
individually as needed.  Like so:

![interface logic][]
(parts: [board][], [edge connector][], [driver][], [connector][], breadboard was a freebie with the wire)

Mindstorms motors are simple DC motors with a quadrature encoder attached to
the output so that speed and position can be monitored and the motor controlled
accordingly.  The [nRF51822][] on micro:bit has just a single hardware
quadrature decoder (previously with no software driver support).

Here we have test code for the generic QDEC driver I wrote, plus a software
quadrature decoder (to control two motors concurrently) and a bit of half-baked
motor control logic.  Although this README is likely to be out of date if I've
kept on hacking since I wrote it.

[micro:bit]: http://microbit.org/
[ev3dev]: http://www.ev3dev.org/
[EV3 intelligent brick]: https://shop.lego.com/en-US/EV3-Intelligent-Brick-45500
[Mindstorms]: https://www.lego.com/en-us/mindstorms
[mbed OS]: https://www.mbed.com/en/development/mbed-os/
[nRF51822]: https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.nrf51/dita/nrf51/pdflinks/ref_manual.html
[interface logic]: doc/interface_board.jpg
[board]: https://www.kitronik.co.uk/5613-bbc-microbit-board-only.html
[edge connector]: https://shop.pimoroni.com/products/edge-connector-breakout-board-for-bbc-micro-bit
[driver]: https://www.pololu.com/product/2130
[connector]: http://www.mindsensors.com/ev3-and-nxt/58-breadboard-connector-kit-for-nxt-or-ev3
