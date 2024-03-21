// TODO: license
#ifndef __RP2040AUDIO_H
#define __RP2040AUDIO_H

#ifndef MCU_MHZ
#define MCU_MHZ 133
#endif

#define WAV_PWM_SCALE 1                             // the tradeoff btwn bit depth & sample rate. 1 = 10 bit, 2 = 11 bit ... 
                                                    // 10-bit audio has the advantage that the sample rate is up at 130khz, 
                                                    // which means that the HF noise is really getting well-suppressed. 
                                                    // With 12-bit audio, that noise is getting down into the almost-audible 
                                                    // spectrum, so output filtering is more crucial.
																										//
                                                    // TODO: PDM could improve this, if more sample resolution was needed.
#define WAV_PWM_RANGE (1024 * WAV_PWM_SCALE)
#define WAV_PWM_COUNT (WAV_PWM_RANGE - 1)  // the PWM counter's setting
#define WAV_SAMPLE_RATE (MCU_MHZ * 1000000 / WAV_PWM_RANGE) // in seconds/hz

#define SAMPLE_BUFF_CHANNELS 1 // one hopes the compiler is smart enough to remove a 1-iteration loop
#define TRANSFER_BUFF_CHANNELS 2 // because the PWM subsystem wants to deal with stereo pairs, we use 2 stereo txBufs instead of 4 mono ones.
#define AUDIO_PERIOD 1 // seconds
#define SAMPLE_BYTES 2  // bytes

// Core1 scales samples from the sample buffer into this buffer,
// while DMA transfers from this buffer to the PWM.
#define TRANSFER_WINDOW_XFERS 40 // number of 32-bit (4-byte) DMA transfers in the window
																 // NOTE: when this was 80, the resulting PWM frequency was in hearing range & faintly audible in some situations.
																 // Now I am talking about dropping the system clock, this will need to be re-checked.
#define TRANSFER_SAMPLES ( 4 / SAMPLE_BYTES ) // == 2; 32 bits is two samples per transfer
#define TRANSFER_BUFF_SAMPLES ( TRANSFER_WINDOW_XFERS * TRANSFER_SAMPLES ) // size in uint_16 samples
																																 
// IMPORTANT:
// SAMPLE_BUFF_SAMPLES must be a multiple of TRANSFER_WINDOW_XFERS, because the ISR only 
// checks for overrun once per TRANSFER_WINDOW_XFERS.  (For efficiency.)
//
//#define SAMPLE_BUFF_SAMPLES 	( TRANSFER_WINDOW_XFERS * (320 / WAV_PWM_SCALE) )
//
// that's fine for a waveform, but for noise we need a much larger buffer.
// (It's remarkable how long a white noise sample has to be before you can't detect some
// looping artifact.  Longer than 2 seconds, for sure.)
#define SAMPLE_BUFF_SAMPLES (TRANSFER_WINDOW_XFERS * 1000) 

class RP2040Audio {
public:
  static short transferBuffer[2][TRANSFER_BUFF_SAMPLES];
  static short sampleBuffer[SAMPLE_BUFF_SAMPLES];
	static volatile uint32_t iVolumeLevel; // 0-1024, or higher for clipping
	// an unused pwm slice that we can make a loop timer from:
	static unsigned char loopTriggerPWMSlice;

  RP2040Audio();
  static void __not_in_flash_func(ISR_play)();
  static void __not_in_flash_func(ISR_test)();

	// TODO: more general-purpose init() signatures for:
	// -- only one stereo pair
	// -- only one mono channel?
	// -- without loopSlice? (for not looping samples)
  void init(unsigned char ring1, unsigned char ring2, unsigned char loopSlice);  // allocate & configure PWM and DMA for two TRS ports

  // void play(short buf[], unsigned int bufLen, unsigned char port); // turn on PWM & DMA
  void play(unsigned char port);   // turn on PWM & DMA
  void pause(unsigned char port);  // halt PWM & DMA
  void pauseAll();  // halt everything
  bool isPlaying(unsigned char port);
	void fillWithNoise();
	void fillWithSine(uint count, bool positive = false);
	void fillWithSaw(uint count, bool positive = false);
	void fillWithSquare(uint count, bool positive = false);
  void tweak();  // adjust the trigger pulse. for debugging purposes only. reads from Serial.
	// TODO:
	// void sleep()
	// void wake()

private:
  static int wavDataCh[2];
  static int wavCtrlCh[2];
  static unsigned int pwmSlice[2];
  static short* bufPtr[2];
  io_rw_32* interpPtr;
  unsigned short volumeLevel = 0;
};


#endif  // __RP2040AUDIO_H
