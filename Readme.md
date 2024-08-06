# PICOMIX: Arduino+Pico+PWM Audio Mixer

This library implements a PWM-based audio system 
for Arduino on the Raspberry Pi RP2040 
which allows multiple samples to be played simultaneously 
at independent volumes and speeds (even backwards),
all mixed down to a pair of PWM signals.

This is very useful for music-toy applications,
but if you just want to play a single channel of sound at a normal speed 
there are better options.

# Features
* Play multiple tracks of audio -- at least 24 simultanous tracks at 133mhz.
* Each track has independent speed, volume, transport and loop controls.
* Uses the RP2040's DMA controllers, PWM generators and hardware interpolators to reduce MCU usage.
* The mixer ISR (the main user of MCU) can run on either core.
* Some handy waveform-generation utilities.

# Requirements
This library currently requires the arduino-pico core by Earl Philhower:
  https://github.com/earlephilhower/arduino-pico
It is not currently ported to other cores, to other MCUs than the RP2040,
or to other environments than Arduino.

# NOTICE: PWM AUDIO IS CHEAP AND DIRTY!
Due to the nature of PWM audio, the signal produced on the output pins
contains horrible amounts of supersonic noise that must be filtered out,
or else it can cause all kinds of downstream audio problems
and may even damage amplifiers and loudspeakers!
I will not be held responsible for your loss of gear or hearing.
Chapter 3.4 of the RPi Foundation's excellent document "Hardware Design with RP2040"
describes a good filter circuit and other strategies for addressing this:
  https://datasheets.raspberrypi.com/rp2040/hardware-design-with-rp2040.pdf

Furthermore, PWM audio signals are powered by the MCU's main power bus,
and will transmit whatever audible noise is present on that bus, which is often quite a lot.

This library allows you to adjust the tradeoffs between bit-depth and sample rate.
Higher bit depth gives lower sample rates, which in turn generates more HF noise.
When using a simple single-pole RC hipass output filter, 10 or 11 bit resolution 
can produce a decent-sounding signal with a manageable amount of noise.


# Roadmap

Missing features that you or I might someday implement include:
  * Stereo. In this release the left and right PWM pins play the same mono signal.
  * Multiple stereo outputs
  * PDM output (lower noise, more cycles)
  * Output to any specific codec
  * Support the mbed core
  * Don't require Arduino at all


# Usage example:

~~~cpp

// Coming soon ...

~~~

# License

This library is Open Source, released under the Creative Commons
ATTRIBUTION-SHAREALIKE 4.0 INTERNATIONAL license (CC BY-SA 4.0)
  https://creativecommons.org/licenses/by-sa/4.0/
(This is the license used by the RPi foundation for the Pico SDK.)
