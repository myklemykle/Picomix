# PICOMIX: Arduino+Pico+PWM Audio Mixer

This library implements a PWM-based audio system 
for Arduino on the Raspberry Pi RP2040 
which allows multiple samples to be played simultaneously 
at independent volumes and speeds (even backwards),
all mixed down to a pair of PWM signals.

This is very useful for music-toy applications,
but if you just want to play a single channel of sound at a normal speed 
there are simpler options.

# Features
* Play multiple tracks of audio -- at least 24 simultanous tracks at 133mhz.
* Each track has independent speed, volume, transport and loop controls.
* Uses the RP2040's DMA controllers, PWM generators and hardware interpolators to reduce MCU usage.
* The mixer ISR (the main user of MCU) can run on either core.
* Some handy waveform-generation utilities.

# Requirements
This library currently requires [the arduino-pico core by Earl Philhower](https://github.com/earlephilhower/arduino-pico).
It is not currently ported to other cores, to other MCUs than the RP2040,
or to other environments than Arduino.

# NOTICE: PWM AUDIO IS CHEAP AND DIRTY
Due to the nature of PWM audio, the signals produced on the output pins
contain huge amounts of supersonic noise that must be filtered out,
or else it can cause all kinds of downstream audio problems
and may even damage amplifiers and loudspeakers!
I will not be held responsible for your loss of gear or hearing.
Chapter 3.4 of the RPi Foundation's excellent document ["Hardware Design with RP2040"](https://datasheets.raspberrypi.com/rp2040/hardware-design-with-rp2040.pdf)
describes a good filter circuit and other strategies for addressing this.

Furthermore, PWM audio signals are powered by the MCU's main power bus
and will transmit whatever audible noise is present on that bus, which is often quite a lot.

This library allows you to adjust the tradeoffs between bit-depth and sample rate.
Higher bit depth gives lower sample rates, which in turn generates more HF noise.
When using a simple single-pole RC lowpass output filter and running at 133mhz clock rate, 
10 or 11 bit resolution can produce a decent-sounding signal with a manageable amount of noise.

# Roadmap

Missing features that you or I might someday implement include:
  * Stereo. In this release the left and right PWM pins play the same mono signal, and raw audio files are assumed to be mono.
  * Multiple stereo outputs
  * PDM output (lower noise, more cycles)
  * Output to any specific codec
  * Support the mbed core
  * Don't require Arduino at all


# Usage example:

~~~cpp

#include "Picomix.h"
#include "LittleFS.h" // or some other file system supported by arduino-pico
 
auto &audio = PicoMix::onlyInstance();
const AUDIO_PIN 23;  // or some other GPIO pin

void setup(){

  audio.init(AUDIO_PIN);
  audio.start();

  LittleFS.begin();

  // Load a raw audio file (mono, 16-bit signed integer samples) into a track:
  auto track0 = audio.addTrack(LittleFS, "blorp.raw")
    ->setLoops(1000) // tell it to loop one thousand times
    ->setLevel(0.5)  // volume level
    ->play();
  // Now track0 == audio.trk[0]

  // Create another track with some sine waves:
  auto track1 = audio.addTrack(1, 4410) // Allocate space for 0.1 seconds of mono samples at (approximately) 44.1khz
    ->setLoops(LOOPFOREVER)   // Loop until stopped
    ->play()
    ->buf->fillWithSine(44);  // 1/10th second of (approximately) 440hz (when track speed == 1.0)
  // Now track1 == audio.trk[1]

}

void loop(){
  // You can adjust playback speeds on the fly
  audio.trk[0]->setSpeed((random(0,20) - 10) / 5.0);

  delay(1000);

  audio.trk[1]->setSpeed((random(0,10)) / 5.0);

  delay(1000);
}
~~~

# Open Source

This library is released under the Creative Commons 
ATTRIBUTION-SHAREALIKE 4.0 INTERNATIONAL open source license 
([CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/)).
This is the same license used by the RPi foundation for the Pico SDK.
