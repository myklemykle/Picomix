// TODO: license/intro


#define SDEBUG
#include "dbg_macros.h"

#include "Arduino.h" // for Serial
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/interp.h"
#include "RP2040Audio.h"

#define iTWICE (i=0;i<2;i++)

void PWMStreamer::setup_audio_pwm_slice(unsigned char pin){
	pwmSlice = pwm_gpio_to_slice_num(pin);

	// halt
	pwm_set_enabled(pwmSlice, false);

	// initialize:
	pCfg = pwm_get_default_config();
	pwm_config_set_wrap(&pCfg, WAV_PWM_COUNT);
	pwm_init(pwmSlice, &pCfg, false);
	pwm_set_irq_enabled(pwmSlice, false);

	// line them up & adjust levels
	pwm_set_both_levels(pwmSlice, 0, 0);
	pwm_set_counter(pwmSlice, 0);
}


// ALERT: This methods assumes you've already set up the pwm slices
void PWMStreamer::setup_dma_channels(){
  dma_channel_config wavDataChConfig;
	int i;

	// Setup a DMA timer to feed samples to PWM at an adjustable rate:
	dmaTimer = dma_claim_unused_timer(true /* required */);
	dma_timer_set_fraction(dmaTimer, PWM_DMA_TIMER_NUM, PWM_DMA_TIMER_DEM);  // play back at (nearly) 44.1khz

	// get some DMA channels:
	for iTWICE {
		if (wavDataCh[i] < 0) {  // if uninitialized
			Dbg_println("getting dma");
			wavDataCh[i] = dma_claim_unused_channel(true);
			Dbg_printf("pwm dma channel %d= %d\n", i, wavDataCh[i]);
		}
	}


	for iTWICE {
		/****************************************************/
		/* Configure data DMA to copy samples from xferbuff to PWM */
		/****************************************************/
		wavDataChConfig = dma_channel_get_default_config(wavDataCh[i]);
		channel_config_set_read_increment(&wavDataChConfig, true);
		channel_config_set_write_increment(&wavDataChConfig, false);
		channel_config_set_transfer_data_size(&wavDataChConfig, DMA_SIZE_32);     // 32 bytes at a time (l & r 16-bit samples)

		int treq = dma_get_timer_dreq(dmaTimer);
		channel_config_set_dreq(&wavDataChConfig, treq);
		channel_config_set_chain_to(&wavDataChConfig, (i == 0) ? wavDataCh[1] : wavDataCh[0]);      // chain to the other data channel
		dma_channel_configure(
			wavDataCh[i],                                               // channel to config
			&wavDataChConfig,                                           // this configuration
			(void*)(PWM_BASE + PWM_CH0_CC_OFFSET + (0x14 * pwmSlice)),  // write to pwm channel (pwm structures are 0x14 bytes wide)
			tBufDataPtr[i],
			TRANSFER_BUFF_SAMPLES / 4,         // 2 samples (32 bits) per transfer, and we're only transferring half of the buffer.
																				 // TODO: deal with odd numbers here
			false                              // Don't start immediately
		);
		dma_channel_set_irq1_enabled(wavDataCh[i], true);
	}

}

void PWMStreamer::init(unsigned char ring) {
	///////////////////
	// Set up PWM 
	setup_audio_pwm_slice(ring); // Enable PWM assigned to pins { ring, ring+1 }

	/////////////////////////
	// claim and set up DMA channels
	setup_dma_channels();
}

bool PWMStreamer::isStarted() {
	if (dma_channel_is_busy(wavDataCh[0]))
		return true;
	return dma_channel_is_busy(wavDataCh[1]);
}

void PWMStreamer::_stop(){
	int i;
	// abort DMA
	for iTWICE {
		dma_channel_abort(wavDataCh[i]);
	}
	// disable pwm
	pwm_set_enabled(pwmSlice, false);

	Dbg_println("all stopped");
}

void PWMStreamer::_start() {
	/*********************************************/
	/* Stop playing audio if DMA already active. */
	/*********************************************/
	if (isStarted())
		_stop();

	// rewind pwm
	pwm_init(pwmSlice, &pCfg, false);
	pwm_set_counter(pwmSlice, 0);


	/**********************/
	/* Start WAV PWM DMA. */
	/**********************/
	dma_start_channel_mask(1 << wavDataCh[0]);
	Dbg_println("dma channels started");

	// start the signal PWM 
	pwm_set_mask_enabled((1 << pwmSlice) | pwm_hw->en);
}


//////////////////////////////////////////////////
///  AudioTrack
//////////////////////////////////////////////////

void AudioTrack::_pause(){
	playing = false;
	// Dbg_println("paused");
}

void AudioTrack::_play(){
	long c;
	if (sampleBuffInc_fp5 > 0)  {
		c = playbackStart;
	} else {
		c = min (playbackStart + playbackLen, buf->sampleLen);
	}

	sampleBuffCursor_fp5 = inttofp5(c);
	playing = true;
	loopCount = max(1, loops);
	// Dbg_println("playing");
}

void AudioTrack::setLoops(int l){
	loops = max(-1, l);
}

inline bool AudioTrack::_doneLooping(){
	if (loops < 0) return false;
	if (loopCount > 1) return false;
	return true;
}

void AudioTrack::setSpeed(float speed){
	if (speed == 0) // no. do not do this.
		return;

	sampleBuffInc_fp5 = int(inttofp5(speed));
	// Dbg_printf("rate = %f, inc = %d\n", speed, sampleBuffInc_fp5);
}

float AudioTrack::getSpeed(){
	float speed = fp5tofloat(sampleBuffInc_fp5);
	// Dbg_printf("rate = %f, inc = %d\n", speed, sampleBuffInc_fp5);
	return speed;
}

// expecting a value between 0 and 1, or higher for trouble ...
void AudioTrack::setLevel(float level){
	iVolumeLevel = max(0, level * WAV_PWM_RANGE);
}

void AudioTrack::advance(){
	// TODO: move this to the spots where start/len change ...
	int32_t playbackStart_fp5 = inttofp5(playbackStart);
	int32_t playbackEnd_fp5 = inttofp5(min((playbackStart + playbackLen), buf->sampleLen));

	sampleBuffCursor_fp5 += sampleBuffInc_fp5;

	// sampleBuffInc_fp5 may be negative
	if (sampleBuffInc_fp5 > 0)
		while (sampleBuffCursor_fp5 >= playbackEnd_fp5){
			if (_doneLooping()) {
				playing = false;
				sampleBuffCursor_fp5 = playbackStart_fp5;
				//Dbg_println("played.");
			} else {
				sampleBuffCursor_fp5 -= (playbackEnd_fp5 - playbackStart_fp5);
				loopCount--;
			}
		}

	if (sampleBuffInc_fp5 < 0)
		while (sampleBuffCursor_fp5 <= playbackStart_fp5){
			if (_doneLooping()) {
				playing = false;
				sampleBuffCursor_fp5 = playbackEnd_fp5;
				//Dbg_println(".deyalp");
			} else
				sampleBuffCursor_fp5 += (playbackEnd_fp5 - playbackStart_fp5);
				loopCount--;
		}
}



////////////////////////////////////////
// RP2040Audio object manages all the audio objects in the system,
// and defines the ISR that pumps AudioTracks into txBufs.

// This gets called once at startup to set up PWM
void RP2040Audio::init(unsigned char ring) {

	/////////////////////////
	// set up digital limiter (used by ISR)
	// interp1 will clamp signed integers to within +/- WAV_PWM_RANGE/2
	interp_config cfg = interp_default_config();
	interp_config_set_clamp(&cfg, true);
	interp_config_set_signed(&cfg, true);
	interp_set_config(interp1, 0, &cfg);

	interp1->base[0] = 0 - (WAV_PWM_RANGE / 2);;
	interp1->base[1] = (WAV_PWM_RANGE / 2) -1;

	////////////////////////
	// set up PWM streaming
	pwm.init(ring);
	
	// install ISR
	irq_set_exclusive_handler(DMA_IRQ_1, ISR_play);
}

void RP2040Audio::start(){
	enableISR(true);
	pwm._start();
}

void RP2040Audio::stop(){
	pwm._stop();
	enableISR(false);
}

void RP2040Audio::enableISR(bool on){
	if (on) {
		irq_set_enabled(DMA_IRQ_1, true);
	} else {
		irq_set_enabled(DMA_IRQ_1, false);
	}
}

//
// In our double-buffered DMA scheme, this ISR refills 
// the idle buffer with new samples and rewinds its
// DMA channel, even as the other DMA continues
// to pump the other buffer's samples through the PWM.
//
void __not_in_flash_func(RP2040Audio::ISR_play)() {
	static auto &my = onlyInstance();
	AudioBuffer *idleTxBuf;
	int idleChannel;
	short scaledSample;

	my.ISRcounter++;

	// write to the half of the buffer that we're not DMAing from ATM
	if (dma_channel_is_busy(my.pwm.wavDataCh[0])) {
		idleChannel = 1;
	} else {
		idleChannel = 0;
	}

	// reset idle channel DMA
	dma_channel_acknowledge_irq1(my.pwm.wavDataCh[idleChannel]);
	dma_channel_set_read_addr(my.pwm.wavDataCh[idleChannel], my.pwm.tBufDataPtr[idleChannel], false);

	idleTxBuf = &my.transferBuffer[idleChannel];

	// fill idle channel buffer
	for (int i = 0; i < idleTxBuf->samples; i += idleTxBuf->channels) {
		// // sanity check:
		// if (sampleBuffCursor_fp5 < 0)
		// 	Dbg_println("!preD");

		// get next sample:
		if (!my.csr.playing){
			scaledSample = WAV_PWM_RANGE / 2; // 50% == silence

		} else {

			// Since amplitude can go over max, use interpolator #1 in clamp mode
			// to hard-limit the signal.
			interp1->accum[0] =
												(short)(
													(long) (
														my.sampleBuffer.data[fp5toint(my.csr.sampleBuffCursor_fp5)]
														 * my.csr.iVolumeLevel  // scale numerator (can be from 0 to more than WAV_PWM_RANGE
													)
													/ WAV_PWM_RANGE      // scale denominator
												)
				;
			// TODO: set up interp0 to perform this add? would that even be faster?
			scaledSample = interp1->peek[0] + (WAV_PWM_RANGE / 2); // shift to positive
		}

		// put that sample in both channels
		for (int j=0; j < idleTxBuf->channels; j++)
			idleTxBuf->data[i+j] = scaledSample;

		my.csr.advance();

	}
}

/*
// TODO: ISR_test is from the PI codebase. It's broken until we get multi-port support back in ...
//
// This ISR time-scales a 1hz sample into 4 output channels at 4 rates, but not (yet) adjusting volume.
// instead of FP, we will use long ints, and shift off the 8 LSB
// This assumes that the sample buffer contains a single complete waveform.
extern volatile uint32_t sampleCursorInc[4];
//void __not_in_flash_func(RP2040Audio::ISR_test)() {
void RP2040Audio::ISR_test() {
	static long sampleBuffCursor[4] = {0,0,0,0};

	ISRcounter++;

	// reset idle channel DMA
	if (dma_channel_is_busy(pwm.wavDataCh[0])) {
		idleChannel = 1;
	} else {
		idleChannel = 0;
	}

	dma_channel_acknowledge_irq1(pwm.wavDataCh[idleChannel]);
	dma_channel_set_read_addr(pwm.wavDataCh[idleChannel], pwm.tBufDataPtr[idleChannel], false);
	
	for (uint8_t port = 0; port<1; port++) {
		for (int i = 0; i < TRANSFER_BUFF_SAMPLES; i+=TRANSFER_BUFF_CHANNELS) {
			for (uint8_t chan = 0; chan < TRANSFER_BUFF_CHANNELS; chan++){
				uint8_t pc = (port<<1)+chan;

				// copy from mono samplebuf to stereo transferbuf
				transferBuffer.data[i+chan] = sampleBuffer.data[ sampleBuffCursor[pc] / 256 ] ;
						// + (WAV_PWM_RANGE / 2);  // not shifting! we expect a positive-weighted sample in the buffer (true arg passed to fillWithSine)

				// advance cursor:
				sampleBuffCursor[pc] += sampleCursorInc[pc];
					while (sampleBuffCursor[pc] >= SAMPLE_BUFF_SAMPLES *256)
						sampleBuffCursor[pc] -= SAMPLE_BUFF_SAMPLES *256;
			}
		}
	}

}
*/

//////////
//
// These basic utils generate signals in the sampleBuffer.
// In every case it's signed values between -(WAV_PWM_RANGE/2)
// and WAV_PWM_COUNT
//
// fill buffer with value of an arbitrary function across a given range,
// repeated some number of times.
//

// this version takes a sample start & length, updating sampleStart & sampleLen
void AudioBuffer::fillWithFunction(float fStart, float fEnd, const std::function<int(float)> theFunction, float repeats, uint32_t sLen, uint32_t sStart){
	// If we had exceptions in Arduino, these would be exceptions.
	// Instead, try to cope with really weird args:
	while (sStart >= samples)
		// wrap it
		sStart -= samples;

	if (sampleStart + sLen > samples)
		// truncate it
		sLen = samples - sampleStart;

	if (sLen == 0)
		// forgetaboutit
		return;


	sampleLen = sLen;
	sampleStart = sStart;
	fillWithFunction(fStart, fEnd, theFunction, repeats);
}

// this version fills between the current values of sampleStart & sampleLen 
void AudioBuffer::fillWithFunction(float start, float end, const std::function<int(float)> theFunction, float repeats){
	float deltaX = (end - start)/sampleLen * repeats;
	float repeatLen = sampleLen / repeats;

	float loopCsr = 0;
	for (int csr = 0; csr < sampleLen; csr++){
		loopCsr += 1;
		while (loopCsr > repeatLen)
			loopCsr -= repeatLen;
		float xNow = start + (loopCsr * deltaX);
		// fill all channels:
		for (int ch = 0; ch < channels; ch++) {
			data[sampleStart + csr + ch] = theFunction(xNow);
		}
	}
}

// fill buffer with white noise (signed)
void AudioBuffer::fillWithNoise(){
	randomSeed(666);
	for(int i=0; i<(channels * samples); i++){
		data[i] = random(WAV_PWM_RANGE) - (WAV_PWM_RANGE / 2);
	}
}


// fill buffer with sine waves
void AudioBuffer::fillWithSine(uint count, bool positive){
	const float twoPI = 6.283;
	const float scale = (WAV_PWM_RANGE) / 2;
	fillWithFunction(0, twoPI, [=](float x)->int {
			return static_cast<int>( (

				sin(x) 

			* scale) + (positive ? scale : 0) );
	}, count, samples);
}

// fill buffer with square waves
void AudioBuffer::fillWithSquare(uint count, bool positive){
	const float scale = (WAV_PWM_RANGE) / 2;
	fillWithFunction(0,1, [=](float x)->int {
			return static_cast<int>( (

				(x >= 0.5 ? -1 : 1)

			* scale) + (positive ? scale : 0) );
	}, count, samples);
}

// fill buffer with sawtooth waves running negative to positive
void AudioBuffer::fillWithSaw(uint count, bool positive){
	const float scale = (WAV_PWM_RANGE) / 2;
	fillWithFunction(-1,1, [=](float x)->int {
			return static_cast<int>( (

					x

			* scale) + (positive ? scale : 0) );
	}, count, samples);
}

// load a raw PCM audio file, which you can create with sox:
//       sox foo.wav foo.raw
// assuming foo.raw contains signed 16-bit samples
uint32_t AudioBuffer::fillFromRawFile(Stream &f){
	uint32_t bc; // buffer cursor
	// loading 16-bit data 8 bits at a time ...
	uint32_t length = f.readBytes((char *)data, byteLen());
	if (length<=0){
		Dbg_println("read failure");
		return length;
	}

	if (length==byteLen()){
		Dbg_println("sample truncated");
	}

	sampleStart = 0;
	// convert it to length in samples
	sampleLen = length / resolution;

	// Now shift those (presumably) signed-16-bit samples down to our output sample width
	for (bc = sampleStart; bc<sampleLen; bc++) {
		data[bc] = data[bc] / (pow(2, (16 - WAV_PWM_BITS)));
	}

	return sampleLen;
}

uint32_t AudioTrack::fillFromRawFile(Stream &f){
	playbackLen = buf->fillFromRawFile(f);
	playbackStart = buf->sampleStart; // probably 0
	return playbackLen;
}



