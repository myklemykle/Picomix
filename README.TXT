This Arduino library uses the DMA audio streaming stategy demonstrated by 
Van Hunter Adams (https://vanhunteradams.com/Pico/DAC/DMA_DAC.html)
to feed samples directly to the PWM subsystem of the RP2040,
instead of to a DAC, while minimizing MCU consumption. 
RP2040 hardware interpolators are also used to reduce the MCU workload.
The remaining MCU audio work can be moved to core1 if desired.

It's totally incomplete & still evolving, but I did use it 
for Pocket Integrator (https://msl.mykle.com/pi/) .

Normal PWM caveats apply: the audio signal produced will contain
horrible amounts of supersonic noise that must be filtered out, or it
can cause all kinds of downstream audio problems and potentially even damage
amplifiers and loudspeakers! I will not be held responsible for 
your loss of gear or hearing.

Chapter 3.4 of the RPi Foundation's excellent document 
"Hardware Design with RP2040" describes a good filter circuit 
and other strategies for addressing this:
	https://datasheets.raspberrypi.com/rp2040/hardware-design-with-rp2040.pdf

Much thanks to Van Hunter Adams and Jason Birch 
for providing well-documented example code online.

