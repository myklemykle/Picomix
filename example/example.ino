// Because this repo is not organized like a standard Arduino library,
// for this example to build you must add Picomix.cpp and Picomix.h to this sketch. 
// The Arduino IDE has an "add file" command that works for that,
// or you can just copy or link the files into the sketch directory.

// You also need to load the example audio file onto your Pico's flash filesystem.
// This example uses LittleFS for the filesystem. 
// It's supported by the Arduino-Pico core by Earle Philhower.
// The easy way to use it is to install his helper tool into Arduino:
//   https://arduino-pico.readthedocs.io/en/latest/fs.html#uploading-files-to-the-littlefs-file-system-on-ide-2-x-rp2040-and-rp2350
// Once that's installed, the Arduino Command Pallete will include two LittleFS tools,
// one to build the filesystem image and another to install it.


#include "Picomix.h"
#include "LittleFS.h" // or some other file system supported by arduino-pico
 
auto &audio = Picomix::onlyInstance();
#define AUDIO_PIN 23  // or some other GPIO pin

void setup(){

  audio.init(AUDIO_PIN);
  audio.start();

  LittleFS.begin();

  // Load a raw audio file (mono, 16-bit signed integer samples) into a track:
  auto *track0 = audio.addTrack(LittleFS, "blorp.raw")
    ->setLoops(1000) // tell it to loop one thousand times
    ->setLevel(0.5)  // volume level
    ->play();
  // Now track0 == audio.trk[0]

  // Create another track with some sine waves:
  auto *track1 = audio.addTrack(1, 4410) // Allocate space for 0.1 seconds of mono samples at (approximately) 44.1khz
    ->setLoops(LOOPFOREVER)   // Loop until stopped
    ->play();
    track1->buf->fillWithSine(44);  // 1/10th second of (approximately) 440hz (when track speed == 1.0)
  // Now track1 == audio.trk[1]

}

void loop(){
  // You can adjust playback speeds on the fly
  audio.trk[0]->setSpeed((random(0,20) - 10) / 5.0);

  delay(1000);

  audio.trk[1]->setSpeed((random(0,10)) / 5.0);

  delay(1000);
}

