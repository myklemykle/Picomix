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
#define TRANSFER_BUFF_CHANNELS 2
	// because the PWM subsystem wants to deal with stereo pairs, we use 2 stereo txBufs instead of 4 mono ones.

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
//#define SAMPLE_BUFF_SAMPLES (TRANSFER_WINDOW_XFERS * 2500)
// looping artifact.  Longer than 2 seconds, for sure.)
#define SAMPLE_BUFF_SAMPLES (TRANSFER_WINDOW_XFERS * 1000)

// And that's using this much memory:
// #define SAMPLE_BUFF_BYTES SAMPLE_BUFF_SAMPLES * BYTES_PER_SAMPLE
// // aka
#define SAMPLE_BUFF_BYTES SAMPLE_BUFF_SAMPLES * sizeof(short)
#define TRANSFER_BUFF_BYTES TRANSFER_BUFF_SAMPLES * BYTES_PER_SAMPLE

////////////////////
// test tone defs:
#define TESTTONE_OFF 0
#define TESTTONE_SINE 1
#define TESTTONE_SQUARE 2
#define TESTTONE_SAW 3
#define TESTTONE_NOISE 4
#define TESTTONE_NAMES { "off", "sine", "square", "saw", "noise" }
#define TESTTONE_COUNT 5 // including "off"


////////////////
// Audiobuffer stores channels x samples of audio
//
#include <functional>
struct AudioBuffer {
	const uint8_t resolution = BYTES_PER_SAMPLE; // bytes per a single channel's sample

	const uint8_t channels; // # of interleaved channels of samples: mono = 1, stereo = 2
	const long int samples;	// number of N-channel samples in this buffer
	int16_t *data;

	AudioBuffer(uint8_t c, long int s): channels(c), samples(s){
		data = new int16_t[c * s];
	}

	inline uint32_t byteLen(){
		return channels * samples * resolution;
	}

	uint32_t sampleStart = 0;
	uint32_t sampleLen;

	void fillWithFunction(float start, float end, const std::function<int(float)> theFunction, float repeats = 1.0);
	void fillWithFunction(float fStart, float fEnd, const std::function<int(float)> theFunction, float repeats, uint32_t sLen, uint32_t sStart=0);

	void fillWithNoise();
	void fillWithSine(uint count, bool positive = false);
	void fillWithSaw(uint count, bool positive = false);
	void fillWithSquare(uint count, bool positive = false);
	uint32_t fillFromRawFile(Stream &f);

};

#include "hardware/pwm.h"

//////////////
// The PWMStreamer sets up & manages DMA streaming
// from a memory buffer to a PWM instance.  Once this is
// running it consumes no MCU cycles.
//
struct PWMStreamer {
public:
	PWMStreamer(AudioBuffer &aB){
		tBuf = &aB;
		// point to the start of the first half of the buffer
		tBufDataPtr[0] = tBuf->data;
		// point to the start of the second half
		tBufDataPtr[1] = &(tBuf->data[tBuf->samples / 2]);
	}

  void init(unsigned char ring, unsigned char loopSlice);
  void _start();
  void _stop();
  bool isStarted();

	unsigned char loopTriggerPWMSlice; // an unused pwm slice that we can make a loop timer from:
  AudioBuffer *tBuf;
	
  int wavDataCh[2] = {-1, -1};  // -1 = DMA channel not assigned yet.
  int wavCtrlCh[2] = {-1, -1};
  unsigned int pwmSlice = 0;
  int16_t *tBufDataPtr[2]; // used by DMA control channel to reset DMA data channel
private:
	pwm_config pCfg, tCfg;
	int dmaTimer;

	void setup_dma_channels();
	void setup_audio_pwm_slice(unsigned char pin);
	void setup_loop_pwm_slice(unsigned char loopSlice);
};


////////////////////
// fp5_t implements (crudely) a 27:5 fixed-point variable.
// The bottom 5 bits hold 32nds of an integer

typedef int32_t fp5_t;
#define SAMPLEBUFFCURSOR_FBITS 5 					// 1, 2, 3, 4, 5
#define SAMPLEBUFFCURSOR_SCALE  ( 1 << (SAMPLEBUFFCURSOR_FBITS - 1) )  // 1,2,4,8,16
#define fp5toint(fp5) (fp5 / SAMPLEBUFFCURSOR_SCALE)
#define fp5tofloat(fp5) (static_cast< float >(fp5) / static_cast< float >(SAMPLEBUFFCURSOR_SCALE))
#define inttofp5(i32) (i32 * SAMPLEBUFFCURSOR_SCALE)


///////////////////
// AudioCursor plays through an AudioBuffer at an adjustable rate & level
// It handles play/pause/seek (with wraparound) and looping.

struct AudioCursor {
	AudioBuffer *buf;
	AudioCursor(AudioBuffer &b){
		buf = &b;
	};

	volatile uint32_t iVolumeLevel; // 0 - WAV_PWM_RANGE, or higher for clipping

	volatile fp5_t sampleBuffCursor_fp5 =	inttofp5(0);
	volatile fp5_t sampleBuffInc_fp5 = 		inttofp5(1); // fractional value:
																																														 //
	bool looping = true;
	int loops = -1;
	int loopCount = 0;
	bool playing = false;

	uint32_t fillFromRawFile(Stream &f);
	// I am adding these underscores so that _pause and pause don't get mixed up when i port PI to this version:
  void _play();
	void _pause();
	void setLooping(bool l);
	void setLoops(int l);
	bool _doneLooping(); // _why?
	void setSpeed(float speed);
	float getSpeed();
	void setLevel(float level);
	void advance();
	uint32_t playbackLen, playbackStart; // public until we need accessors
private:

};

class RP2040Audio {

	///////////////////////////////
	// this section implements the singleton pattern for c++:
	// https://stackoverflow.com/questions/1008019/how-do-you-implement-the-singleton-design-pattern
public:
	static RP2040Audio& onlyInstance(){
		static RP2040Audio singleGuy;
		return singleGuy;
	}
private:
	RP2040Audio() {}
	//RP2040Audio(RP2040Audio const&);    // Don't Implement
	//void operator=(RP2040Audio const&); // Don't implement

public:
	RP2040Audio(RP2040Audio const&)     = delete;
	void operator=(RP2040Audio const&)  = delete;
	///////////////////////////////

public:

  AudioBuffer transferBuffer{TRANSFER_BUFF_CHANNELS, TRANSFER_BUFF_SAMPLES};
	PWMStreamer pwm{transferBuffer};

	// RAM buffer for samples loaded from flash
  AudioBuffer sampleBuffer{1, SAMPLE_BUFF_SAMPLES};
	AudioCursor csr{sampleBuffer};

  void init(unsigned char ring, unsigned char loopSlice);  // allocate & configure one PWM instance & suporting DMA channels

	// is the buffer timing being tweaked at the moment?
	bool tweaking = false;

	// some timing data
	volatile unsigned long ISRcounter = 0;

	// NOTE: these ISRs will need binding to the single instance
	// TODO: singleton pattern should keep a static pointer to the single instance so that's not necessary.
  void ISR_play();
  void ISR_test();


								
	void fillFromRawFile(Stream &f);
  void tweak();  // adjust the trigger pulse. for debugging purposes only. reads from Serial.

private:

};

#endif  // __RP2040AUDIO_H
