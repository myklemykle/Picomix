# Arduino+Pico+PWM Sample Mixer

This library implements a PWM-based audio system for Arduino on
the Raspberry Pi RP2040 which allows multiple samples
to be played simultaneously at independent volumes and speeds 
(including backwards) as a PWM signal on a stereo pair of pins.
This is very useful for music-toy applications, 
but if you just want to play a single channel of sound 
at a normal speed you have other, simpler options. =)

# Requirements

This library currently requires the arduino-pico core by Earl Philhower:
  https://github.com/earlephilhower/arduino-pico
It is not currently ported to other cores, to other MCUs than the RP2040,
or to other environments than Arduino.

# NOTICE: PWM AUDIO IS CHEAP AND NOISY!

Due to the very nature of PWM audio, the signal produced on the output pins
contains horrible amounts of supersonic noise that must be filtered out,
or else it can cause all kinds of downstream audio problems
and may even damage amplifiers and loudspeakers! 
I will not be held responsible for your loss of gear or hearing.
Chapter 3.4 of the RPi Foundation's excellent document "Hardware Design with RP2040" 
describes a good filter circuit and other strategies for addressing this:
  https://datasheets.raspberrypi.com/rp2040/hardware-design-with-rp2040.pdf
  
Furthermore, PWM audio signals are powered by the MCU's main power bus, 
and will transmit whatever noise is present on that bus, which is often quite a lot.

# Roadmap

Missing features that you or I might someday implement include:
  * Multiple stereo outputs
  * PDM output (lower noise, more cycles)
  * Output to some specific codec
  * Support for the mbed core 
  * Don't require Arduino at all


# Usage example:

~~~cpp

// TODO

~~~

# License

This library is Open Source, released under the Creative Commons
ATTRIBUTION-SHAREALIKE 4.0 INTERNATIONAL license (CC BY-SA 4.0)
  https://creativecommons.org/licenses/by-sa/4.0/
(This is the license used by the RPi foundation for the Pico SDK.)

