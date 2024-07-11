// TODO: license
//
//
// WARNING: there should only ever be one instance of this object!
// TODO: implement a minimal singleton pattern here
//
#ifndef __RP2040AUDIO_H
#define __RP2040AUDIO_H

#ifndef MCU_MHZ
#define MCU_MHZ 133
#endif

#define WAV_PWM_SCALE 1                             // the tradeoff btwn bit depth & sample rate. 1 = 10 bit, 2 = 11 bit ... 
                                                    // 10-bit audio has the advantage that the PWM output rate is up at 130khz, 
                                                    // which means that the HF noise is really getting well-suppressed. 
                                                    // With 12-bit audio, that noise is getting down into the almost-audible 
                                                    // spectrum, so output filtering is more crucial.
																										//
                                                    // TODO: PDM could improve this, if more sample resolution was needed.
#define WAV_PWM_BITS (WAV_PWM_SCALE + 9)
#define WAV_PWM_RANGE (1024 * WAV_PWM_SCALE)
#define WAV_PWM_COUNT (WAV_PWM_RANGE - 1)  // the PWM counter's setting
#define WAV_SAMPLE_RATE (MCU_MHZ * 1000000 / WAV_PWM_RANGE) // in seconds/hz .  Running at 133mhz sys_clk, this comes to 129883hz .
#define PWM_SAMPLE_RATE WAV_SAMPLE_RATE // not sure what WAV means in this context, really
#define BUFF_SAMPLE_RATE 44100  				// the sample rate of the sample in our buffer.

// For adjusting a DMA timer to feed the PWM at the buffer sample rate, we need to know clocks-per-sample as a ratio
// That's 133M/44.1k , which is the same as 190000/63 (approximately 3015.873)
// However the DMA timer scales clk_sys by a coefficient, where both the numerator and denominator be 16 bit values.
// 42222/14 ~= 3015.857, i think that's the closest we can get with 16-bit ints.
// (21111/7 is the same, obv)
// I believe we'd need to change our system clock speed to get any closer to a 44.1khz sample rate.
#define PWM_DMA_TIMER_DEM 21111
#define PWM_DMA_TIMER_NUM 7

// To match that on the loop timing PWM, we need it to loop every clocks-per-window, which is TRANSFER_WINDOW_XFERS * clocks-per-sample
// However we can only express the numerator in 8 bits and the denoinator in 4!
// But the PWM counter effectively multiplies the PWM clock divider in our situation.
// Helpfully, 21111 == 227 * 93 
#define LOOP_PWM_COUNT 93 * TRANSFER_WINDOW_XFERS
#define LOOP_PWM_NUM 227
#define LOOP_PWM_DEN 7

// TODO: implement a double-buffering scheme with pure
// DMA instead of this loop-timing PWM thing.

#define SAMPLES_PER_CHANNEL 2
#define BYTES_PER_SAMPLE 2  				
#define SAMPLE_BUFF_CHANNELS 1 
#define TRANSFER_BUFF_CHANNELS 2 // because the PWM subsystem wants to deal with stereo pairs, we use 2 stereo txBufs instead of 4 mono ones.

// Core1 scales samples from the sample buffer into this buffer,
// while DMA transfers from this buffer to the PWM.
#define TRANSFER_WINDOW_XFERS 40 // number of 32-bit (4-byte) DMA transfers in the window
																 // NOTE: when this was 80, the resulting PWM frequency was in hearing range & faintly audible in some situations.
#define TRANSFER_SAMPLES ( 4 / BYTES_PER_SAMPLE ) // == 2; 32 bits is two samples per transfer
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
//#define SAMPLE_BUFF_SAMPLES (TRANSFER_WINDOW_XFERS * 2500) 
#define SAMPLE_BUFF_SAMPLES (TRANSFER_WINDOW_XFERS * 1000) 

// And that's using this much memory:
// #define SAMPLE_BUFF_BYTES SAMPLE_BUFF_SAMPLES * BYTES_PER_SAMPLE
// // aka
#define SAMPLE_BUFF_BYTES SAMPLE_BUFF_SAMPLES * sizeof(short)
#define TRANSFER_BUFF_BYTES TRANSFER_BUFF_SAMPLES * BYTES_PER_SAMPLE

#include "hardware/pwm.h"

class RP2040Audio {
public:
  short transferBuffer[2][TRANSFER_BUFF_SAMPLES];
  short sampleBuffer[SAMPLE_BUFF_SAMPLES];
	volatile uint32_t iVolumeLevel; // 0 - WAV_PWM_RANGE, or higher for clipping
	// an unused pwm slice that we can make a loop timer from:
	unsigned char loopTriggerPWMSlice;
	// is the buffer timing being tweaked at the moment?
	bool tweaking = false;

	// TODO: need to track these per-port:
	// are we looping the buffer?
	bool looping = true;
	// are we playing a sample (otherwise we are silent)
	bool playing = false;

  RP2040Audio();
	// NOTE: these ISRs will need binding to the single instance
	// TODO: singleton pattern should keep a static pointer to the single instance so that's not necessary.
  void __not_in_flash_func(ISR_play)();
  void __not_in_flash_func(ISR_test)();

	// TODO: more general-purpose init() process:
	// -- specify total number of ports (stereo pairs), could be 1, 2, more ...
	// -- then init them port by port
	// -- allocate transfer buffers by port 
	// -- loopslice will not be a thing once double buffering is go ...
  void init(unsigned char ring1, unsigned char ring2, unsigned char loopSlice);  // allocate & configure PWM and DMA for two TRS ports

  // void play(unsigned char port);   // turn on one channel PWM & DMA and start looping the buffer
  // void play();   // turn on both channels
  // void pause(unsigned char port);  // halt PWM & DMA
  // void pauseAll();  // halt everything
	//
	// // I am adding these underscores so that _pause and pause don't get mixed up when i port PI to this version
	
  void _start(unsigned char port);   // turn on one channel PWM & DMA and start looping the buffer
  void _start();   // turn on both channels
  void _stop(unsigned char port);  // halt PWM & DMA
  void _stop();  // halt everything
  void _play(unsigned char port);   // begin transferring sample to the buffer
  void _play();   // begin transferring on both channels
	void _pause(unsigned char port);  // stop transferring
	void _pause();  // stop transferring
								 
	void setLooping(bool l);
	void setSpeed(float speed);
	float getSpeed();
	void setLevel(float level);
  bool isStarted(unsigned char port);
	void fillWithNoise();
	void fillWithSine(uint count, bool positive = false);
	void fillWithSaw(uint count, bool positive = false);
	void fillWithSquare(uint count, bool positive = false);
	void fillFromRawFile(Stream &f);
  void tweak();  // adjust the trigger pulse. for debugging purposes only. reads from Serial.

	uint32_t playbackLen, playbackStart; // public until we need accessors
	uint32_t sampleLen, sampleStart;

private:
  int wavDataCh[2] = {-1, -1};  // -1 = DMA channel not assigned yet. 
  int wavCtrlCh[2] = {-1, -1};
	pwm_config pCfg[2], tCfg;
	int dmaTimer;
  unsigned int pwmSlice[2];
	// TODO: for double buffering, bufPtr[2][2]
  short* bufPtr[2];
  io_rw_32* interpPtr;
  unsigned short volumeLevel = 0;

	// *_fr means a 32-bit fractional value, where 27 bits hold the integer and the remaining 5 bits hold 32nds of an integer
#define SAMPLEBUFFCURSOR_FBITS 5 					// 1, 2, 3, 4, 5
#define SAMPLEBUFFCURSOR_SCALE  ( 1 << (SAMPLEBUFFCURSOR_FBITS - 1) )  // 1,2,4,8,16
	volatile int32_t sampleBuffCursor_fr = 0, sampleBuffInc_fr = (1 * SAMPLEBUFFCURSOR_SCALE); // fractional value: 
																																														 
	void setup_dma_channels();
	void setup_audio_pwm_slice(int channel, unsigned char pin);
	void setup_loop_pwm_slice(unsigned char loopSlice);
};

// // TODO: seperate channel objects:
// class RP2040AudioChannel {
// public:
//   short transferBuffer[TRANSFER_BUFF_SAMPLES];
//   short sampleBuffer[SAMPLE_BUFF_SAMPLES];
// 	volatile uint32_t iVolumeLevel; // 0 - WAV_PWM_RANGE, or higher for clipping
// 	//unsigned char loopTriggerPWMSlice; //share
// 	// bool tweaking = false; // share
// 	bool looping = true;
//
// private:
//   int wavDataCh;
//   int wavCtrlCh;
// 	//int dmaTimer;   // both channels one sample rate for now
//   unsigned int pwmSlice;
//   short* bufPtr;
//   unsigned short volumeLevel = 0; // TODO split
// 	size_t sampleLen; // TODO split
// 	volatile size_t sampleBuffCursor = 0;
// };

#endif  // __RP2040AUDIO_H
