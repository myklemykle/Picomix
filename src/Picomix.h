#ifndef __PICOMIX_H
#define __PICOMIX_H

// This project is Open Source!
// License: https://creativecommons.org/licenses/by-sa/4.0/

#include <functional>
#include <FS.h>
#include "hardware/pwm.h"

//////////////////////////
// Some potentially tweakable/tuneable values.
//
// SDEBUG: send debug statements to serial port?
// Use Dbg_print(), Dbg_println(), Dbg_printf(), etc., to send debug output.
// If SDEBUG is defined, output is sent to Serial.
// If SDEBUG is undefined, all debug code is stripped from the binary.
#define SDEBUG
//
// WAV_PWM_BITS: PWM sample resolution.
// There's a tradeoff between PWM bit resolution & sample rate.
// Choosing 10-bit audio (at 133mhz clock rate) has the advantage 
// that the PWM output frequency is near 130khz,
// which means that the HF noise is easier to suppress.
// Going to 11 bits doubles the PWM resolution but halves the output frequency.
// With 12-bit audio, that noise is getting down into the almost-audible
// spectrum, so output filtering becomes crucial.  But if you know you are
// going into an amplifier or interface that includes its own HF filter,
// you could try 11 or 12 bits here. 
// (Or even more bits if you are overclocking, 
// or if you can't hear high frequencies because you are in Metallica.)
//
#define WAV_PWM_BITS 10
//
//
// PWM_DMA_TIMER_*: fine tuning of output sample rate.
// To scale a DMA timer to feed samples to the PWM driver at a standard 44.1khz sample rate,
// we need to configure the ratio between sample rate & master clock speed.
// This ratio has been calculated to get pretty close to 44.1khz when pico runs at 133mhz.
// For other clock rates or desired output frequencies you'll need to change this ratio.
//
// Here we presume that ratio is 133mhz/44.1khz, which simplifies to 190000/63 (approximately 3015.873)
// However, the DMA timer can only scale clk_sys by a coefficient
// where both the numerator and denominator are 16 bit values.
// So 190000 doesn't go, because it won't fit in a 16-bit int.
// However, 21111/7 ~= 3015.857 . I think that's the closest to 3015.873 we can get with a ratio of 16-bit ints.
//
#define PWM_DMA_TIMER_DEM 21111
#define PWM_DMA_TIMER_NUM 7
//
//
// TRANSFER_WINDOW_XFERS: Number of 32-bit (4-byte) DMA transfers in the transfer buffer
// The size of this value determines the interrupt rate.
// A larger buffer means fewer interrupts, perhaps more efficient.
// OTOH, when this was set to 80 (at 133mhz), the resulting interrrupt frequency
// injected audible noise into a circuit. Lower values keep it supersonic.
#define TRANSFER_WINDOW_XFERS 40
//
//
// PWMSTREAMER_DMA_INTERRUPT:
// RP2040 offers two IRQs that the DMA system may use to trigger an ISR.
// You can select either of them here, just in case some other library wants the other one.
#define PWMSTREAMER_DMA_INTERRUPT DMA_IRQ_0 
//#define PWMSTREAMER_DMA_INTERRUPT DMA_IRQ_1
//
//
// MAX_TRACKS: How many tracks will the ISR try to mix?
// FYI, mixing 24 tracks with the above settings 
// seems to consume about 80% of one core's cycles. 
// I don't know what the application for even that many tracks would be.
// But if you really need to push it, 
// with overclocking or a lower output sample rate
// you can probably mix even more tracks than this:
#define MAX_TRACKS 24
//
/// End user-tweakable section.
/////////////////////////////////////


////////////////////
// fp5_t implements (crudely) a 27:5 fixed-point variable.
// The bottom 5 bits hold 32nds of an integer
//
typedef int32_t fp5_t;
#define SAMPLEBUFFCURSOR_FBITS 5 					// 1, 2, 3, 4, 5
#define SAMPLEBUFFCURSOR_SCALE  ( 1 << (SAMPLEBUFFCURSOR_FBITS - 1) )  // 1,2,4,8,16
#define fp5toint(fp5) (fp5 / SAMPLEBUFFCURSOR_SCALE)
#define fp5tofloat(fp5) (static_cast< float >(fp5) / static_cast< float >(SAMPLEBUFFCURSOR_SCALE))
#define inttofp5(i32) (i32 * SAMPLEBUFFCURSOR_SCALE)


///////////////////
// Serial debugging macros:
#ifdef SDEBUG
// https://gcc.gnu.org/onlinedocs/cpp/Variadic-Macros.html 
#define Dbg_println(...) if(Serial) Serial.println(__VA_ARGS__)
#define Dbg_printf(...) if(Serial) Serial.printf(__VA_ARGS__)
#define Dbg_print(...) if(Serial) Serial.print(__VA_ARGS__)
#define Dbg_flush(X) if(Serial) Serial.flush()

#else
#define Dbg_println(...) {}
#define Dbg_printf(...) {}
#define Dbg_print(...) {}
#define Dbg_flush(X) {}
#endif


///////////////////
// PWM math:
// 
#define WAV_PWM_SCALE (WAV_PWM_BITS - 9)
#define WAV_PWM_RANGE (2 << (WAV_PWM_BITS - 1))
#define WAV_PWM_COUNT (WAV_PWM_RANGE - 1)  // the PWM counter's range is from 0 to (WAV_PWM_COUNT - 1)
#define PWM_SAMPLE_RATE (F_CPU * 1000000 / WAV_PWM_RANGE) // in seconds/hz .  
																													// Running at 133mhz sys_clk, 10 bits == 129883hz .
//
// The PWM subsystem is fed 2 16-bit samples per transfer:
#define SAMPLES_PER_CHANNEL 2
//#define BYTES_PER_SAMPLE 2
// // aka
#define BYTES_PER_SAMPLE sizeof(short)
//
// The transfer buffer is where we assemble those samples:
#define TRANSFER_BUFF_CHANNELS 2
#define TRANSFER_BUFF_SAMPLES ( TRANSFER_WINDOW_XFERS * TRANSFER_BUFF_CHANNELS)
#define TRANSFER_BUFF_BYTES 	( TRANSFER_BUFF_SAMPLES * BYTES_PER_SAMPLE )


////////////////
// AudioBuffer: storage for samples that are played by AudioTracks.
//
// NOTE: It is expressly permitted for a sample to only use part of the buffer,
// and/or for a sample to wrap around the end of the buffer.
//
struct AudioBuffer {
	const uint8_t resolution = BYTES_PER_SAMPLE; // bytes per a single channel's sample
	const uint8_t channels; // # of interleaved channels of samples: mono = 1, stereo = 2
	const long int samples;	// number of N-channel samples in this buffer
	int16_t *data;

	AudioBuffer(uint8_t c, long int s): 
		channels(c), 
		samples(s), 
		data(new int16_t[c * s])
		{
		}

	inline uint32_t byteLen(){
		return channels * samples * resolution;
	}

	uint32_t sampleStart = 0;
	uint32_t sampleLen;

	// Waveform-rendering:
	void fillWithFunction(float start, float end, const std::function<int(float)> theFunction, float repeats = 1.0);
	void fillWithFunction(float fStart, float fEnd, const std::function<int(float)> theFunction, float repeats, uint32_t sLen, uint32_t sStart=0);

	void fillWithNoise();
	void fillWithSine(uint count, bool positive = false);
	void fillWithSaw(uint count, bool positive = false);
	void fillWithSquare(uint count, bool positive = false);

	// Sample-loading:
	uint32_t fillFromRawFile(fs::FS &fs, String filename);
	uint32_t fillFromRawStream(Stream &f);
};


//////////////
// PWMStreamer: sets up & manages a pair of DMA channels
// which take turns streaming samples from a pair of AudioBuffers to a PWM instance.
// The ISR in Picomix rewinds the DMA channels and refills the buffers.
// 
//
struct PWMStreamer {
public:
	PWMStreamer(AudioBuffer &aB0, AudioBuffer &aB1){
		tBuf[0] = &aB0;
		tBuf[1] = &aB1;
		tBufDataPtr[0] = tBuf[0]->data;
		tBufDataPtr[1] = tBuf[1]->data;
	}

  void init(unsigned char ring);
  void start();
  void stop();
  bool isStarted();
	int resetIRQ();

  AudioBuffer *tBuf[2];

  int wavDataCh[2] = {-1, -1};  // -1 = DMA channel not assigned yet.
  int pwmSlice = -1;						// ditto
  int16_t *tBufDataPtr[2]; 			// This was needed by an earlier version -- remove/refactor?
	
private:
	pwm_config pCfg, tCfg;
	int dmaTimer;

	void setup_dma_channels();
	void setup_audio_pwm_slice(unsigned char pin);
};


///////////////////
// AudioTrack: plays samples from an AudioBuffer at an adjustable rate & level.
// It handles play/pause/seek (with wraparound) and looping.
// playbackStart & playbackLen allow trimming to a subset of the sample.
//
struct AudioTrack {
	AudioBuffer *buf;

	// AudioTrack can be instantiated with an existing buffer like so:
	AudioTrack(AudioBuffer &b):
		buf(&b),
		playbackStart(b.sampleStart),
		playbackLen(b.sampleLen)
		{
		};

	// Or with a pointer to a buffer like so:
	AudioTrack(AudioBuffer *b):
		buf(b),
		playbackStart(b->sampleStart),
		playbackLen(b->sampleLen)
		{
		};

	// Or it can instantiate its own new buffer like so:
	AudioTrack(uint8_t channels, long int sampleLen):
		buf(new AudioBuffer(channels, sampleLen)),
		playbackLen(sampleLen)
		{};

	volatile uint32_t iVolumeLevel; // 0 - WAV_PWM_RANGE, or higher for clipping
	volatile fp5_t sampleBuffCursor_fp5 =	inttofp5(0);
	volatile fp5_t sampleBuffInc_fp5 = 		inttofp5(1); 
	bool playing = false;
	uint32_t playbackStart = 0; 
	uint32_t playbackLen; 

  void play();
	void pause(); 
	void setLooping(bool l);
	void setLevel(float level);
	void setLoops(int l);
	void setSpeed(float speed);

	float getSpeed();
	bool isLooping();

	void advance();
	uint32_t fillFromRawStream(Stream &f);
	uint32_t fillFromRawFile(fs::FS &fs, String filename);

private:
	int loops = -1;
	int loopCount = 0;

};

class Picomix {

	///////////////////////////////
	// This section implements the singleton pattern for c++:
	// https://stackoverflow.com/questions/1008019/how-do-you-implement-the-singleton-design-pattern
public:
	static Picomix& onlyInstance(){
		static Picomix singleGuy;
		return singleGuy;
	}
private:
	Picomix() {
		for (int i=0;i<MAX_TRACKS;i++){
			trk[i] = NULL;
		}
	}
public:
	Picomix(Picomix const&)     = delete;
	void operator=(Picomix const&)  = delete;
	//
	// end singleton section
	///////////////////////////////

public:
	void start();
	void stop();

  AudioBuffer transferBuffer[2] = {
		AudioBuffer(TRANSFER_BUFF_CHANNELS, TRANSFER_BUFF_SAMPLES / 2),
		AudioBuffer(TRANSFER_BUFF_CHANNELS, (TRANSFER_BUFF_SAMPLES - (TRANSFER_BUFF_SAMPLES / 2)))
	};
	PWMStreamer pwm{transferBuffer[0],transferBuffer[1]};

	AudioTrack *trk[MAX_TRACKS];

	// some performance profiling info:
	volatile unsigned long ISRcounter = 0;

  void init(unsigned char ring);  
	void enableISR(bool on);

	AudioTrack *addTrack(uint8_t channels, long int sampleLen);
	AudioTrack *addTrack(AudioTrack *t);

private:
	// The master sample mixer:
  static void ISR_play();
};

#endif  // __PICOMIX_H
