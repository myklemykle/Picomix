#ifndef __RP2040AUDIO_H
#define __RP2040AUDIO_H

// TODO: license?

#include <functional>

// TODO: F_CPU instead
#ifndef MCU_MHZ
#define MCU_MHZ 133
#endif


// There's a tradeoff btwn bit depth & sample rate. 
// 10-bit audio has the advantage that the PWM output rate is up at 130khz,
// which means that the HF noise is really getting well-suppressed.
// With 12-bit audio, that noise is getting down into the almost-audible
// spectrum, so output filtering is more crucial.
#define WAV_PWM_BITS 10
#define WAV_PWM_SCALE (WAV_PWM_BITS - 9)
#define WAV_PWM_RANGE (1024 * WAV_PWM_SCALE)
#define WAV_PWM_COUNT (WAV_PWM_RANGE - 1)  // the PWM counter's setting
#define PWM_SAMPLE_RATE (MCU_MHZ * 1000000 / WAV_PWM_RANGE) // in seconds/hz .  Running at 133mhz sys_clk, this comes to 129883hz .

// To scale a DMA timer to feed samples to the PWM driver at a standard 44.1khz sample rate,
// we need to configure the ratio between sample rate & master clock speed.
// Here we presume that ratio is 133mhz/44.1khz, which simplifies to 190000/63 (approximately 3015.873)
// However, the DMA timer can only scale clk_sys by a coefficient 
// where both the numerator and denominator are 16 bit values. 
// So 190000 doesn't fit.
// However, 21111/7 ~= 3015.857 . I think that's the closest to 3015.873 we can get with a ratio of 16-bit ints.
// (To get closer you could change the clock speed and recalculate the ratio.)
#define PWM_DMA_TIMER_DEM 21111
#define PWM_DMA_TIMER_NUM 7


// The PWM subsystem is fed 2 16-bit samples per transfer:
#define SAMPLES_PER_CHANNEL 2
#define BYTES_PER_SAMPLE 2  				

// The transfer buffer is where we assemble those samples:
#define TRANSFER_BUFF_CHANNELS 2

// ISR_play() merges AudioTracks into a transfer buffer
// as DMA transfers from this buffer to the PWM.

// Number of 32-bit (4-byte) DMA transfers in the transfer buffer
// The size of this value determines the interrupt rate.
// A larger buffer means fewer interrupts, perhaps more efficient.
// OTOH, when this was set to 80 (at 133mhz), the resulting interrrupt frequency 
// injected audible noise into a circuit. Lower values keep it supersonic.
#define TRANSFER_WINDOW_XFERS 40 

#define TRANSFER_BUFF_SAMPLES ( TRANSFER_WINDOW_XFERS * TRANSFER_BUFF_CHANNELS)
#define TRANSFER_BUFF_BYTES 	( TRANSFER_BUFF_SAMPLES * BYTES_PER_SAMPLE )
																																
///// NO LONGER TRUE? TODO: test!
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
// AudioBuffer
//
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
// The PWMStreamer sets up & manages a pair of DMA channels
// that take turns streaming samples from a pair of AudioBuffers to a PWM instance.
// (The ISR rewinds the DMA channels and refills the buffers.)
//
struct PWMStreamer {
public:
	PWMStreamer(AudioBuffer &aB0, AudioBuffer &aB1){
		tBuf[0] = &aB0;
		tBuf[1] = &aB1;
		// point to the start of the first half of the buffer
		tBufDataPtr[0] = tBuf[0]->data;
		// point to the start of the second half
		tBufDataPtr[1] = tBuf[1]->data;
	}

  void init(unsigned char ring);
  void _start();
  void _stop();
  bool isStarted();

  AudioBuffer *tBuf[2];
	
  int wavDataCh[2] = {-1, -1};  // -1 = DMA channel not assigned yet.
  unsigned int pwmSlice = 0;
  int16_t *tBufDataPtr[2]; // used by DMA control channel to reset DMA data channel
private:
	pwm_config pCfg, tCfg;
	int dmaTimer;

	void setup_dma_channels();
	void setup_audio_pwm_slice(unsigned char pin);
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
// AudioTrack plays through an AudioBuffer at an adjustable rate & level
// It handles play/pause/seek (with wraparound) and looping.

struct AudioTrack {
	AudioBuffer *buf;
	AudioTrack(AudioBuffer &b){
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
	// This section implements the singleton pattern for c++:
	// https://stackoverflow.com/questions/1008019/how-do-you-implement-the-singleton-design-pattern
public:
	static RP2040Audio& onlyInstance(){
		static RP2040Audio singleGuy;
		return singleGuy;
	}
private:
	RP2040Audio() {}

public:
	RP2040Audio(RP2040Audio const&)     = delete;
	void operator=(RP2040Audio const&)  = delete;
	//
	// end singleton section
	///////////////////////////////

public:
	void start();
	void stop();

  AudioBuffer transferBuffer[2] = { 
		AudioBuffer(TRANSFER_BUFF_CHANNELS, TRANSFER_BUFF_SAMPLES / 2),
			// handle possible odd value of TRANSFER_BUFF_SAMPLES
		AudioBuffer(TRANSFER_BUFF_CHANNELS, (TRANSFER_BUFF_SAMPLES - (TRANSFER_BUFF_SAMPLES / 2)))
	};
	PWMStreamer pwm{transferBuffer[0],transferBuffer[1]};

	// RAM buffer for samples loaded from flash
  AudioBuffer sampleBuffer{1, SAMPLE_BUFF_SAMPLES};
	AudioTrack csr{sampleBuffer};

  void init(unsigned char ring);  // allocate & configure one PWM instance & suporting DMA channels

	// some performance profiling info:
	volatile unsigned long ISRcounter = 0;

	void enableISR(bool on);

	void fillFromRawFile(Stream &f);

private:
	// TODO: move to PWMStreamer?
  static void ISR_play();

};

#endif  // __RP2040AUDIO_H
