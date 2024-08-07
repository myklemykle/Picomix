// Picomix -- multitrack PWM audio contraption for RP2040 by Mykle Hansen
// This project is Open Source!
// License: https://creativecommons.org/licenses/by-sa/4.0/

#include "Picomix.h"
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/interp.h"

#define iTWICE (i=0;i<2;i++)

void PWMStreamer::setup_audio_pwm_slice(unsigned char pin){
	if (pwmSlice < 0) // if not already assigned
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


void PWMStreamer::setup_dma_channels(){
  dma_channel_config wavDataChConfig;
	int i;

	if (pwmSlice < 0){
		Dbg_println("error: set up pwm before dma");
		return;
	}

	// Setup a DMA timer to feed samples to PWM at an adjustable rate:
	if (dmaTimer == -1)
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
			TRANSFER_BUFF_SAMPLES / 4,         // transfer count: 2 samples (32 bits) per transfer, and we're only transferring half of the buffer.
																				 // TODO: deal with non-even-sized tx buffers
			false                              // Don't start immediately
		);
#if (PWMSTREAMER_DMA_INTERRUPT == DMA_IRQ_1)
		dma_channel_set_irq1_enabled(wavDataCh[i], true);
#else
		dma_channel_set_irq0_enabled(wavDataCh[i], true);
#endif
	}

}

void PWMStreamer::init(unsigned char pin) {
	///////////////////
	// Set up PWM output on pin & (pin+1)
	setup_audio_pwm_slice(pin); 

	/////////////////////////
	// claim and set up a pair of DMA channels for double-buffering
	setup_dma_channels();
}

bool PWMStreamer::isStarted() {
	if (dma_channel_is_busy(wavDataCh[0]))
		return true;
	return dma_channel_is_busy(wavDataCh[1]);
}

void PWMStreamer::stop(){
	int i;
	// abort DMA
	for iTWICE {
		dma_channel_abort(wavDataCh[i]);
	}
	// disable pwm
	pwm_set_enabled(pwmSlice, false);

	Dbg_println("all stopped");
}

void PWMStreamer::start() {
	/*********************************************/
	/* Stop playing audio if DMA already active. */
	/*********************************************/
	if (isStarted())
		stop();

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

// This is an inline method the ISR can call
// to clear interrupts & reset streamer for the next interrupt.
inline int PWMStreamer::resetIRQ(){
	int idleSide;

  if (dma_channel_is_busy(wavDataCh[0])) {
    idleSide = 1;
  } else {
    idleSide = 0;
  }

	// clear interrupt
#if (PWMSTREAMER_DMA_INTERRUPT == DMA_IRQ_1)
  dma_channel_acknowledge_irq1(wavDataCh[idleSide]);
#else
  dma_channel_acknowledge_irq0(wavDataCh[idleSide]);
#endif
  // rewind idle channel DMA
  dma_channel_set_read_addr(wavDataCh[idleSide], tBufDataPtr[idleSide], false);

	return idleSide;
}


//////////////////////////////////////////////////
///  AudioTrack
//////////////////////////////////////////////////

AudioTrack *AudioTrack::pause(){
	playing = false;
	// Dbg_println("paused");
	return this;
}

AudioTrack *AudioTrack::play(){
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
	return this;
}

// setLoops(-1) to loop forever when play() is called;
// setLoops(n) to loop N times before stopping
AudioTrack *AudioTrack::setLoops(int l){
	loops = max(-1, l);
	return this;
}

inline bool AudioTrack::isLooping(){
	if (loops < 0) return true;
	if (loopCount > 1) return true;
	return false;
}

AudioTrack *AudioTrack::setSpeed(float speed){
	if (speed == 0) // no. do not do this. 
		return this;

	sampleBuffInc_fp5 = int(inttofp5(speed));
	// Dbg_printf("rate = %f, inc = %d\n", speed, sampleBuffInc_fp5);
	return this;
}

float AudioTrack::getSpeed(){
	float speed = fp5tofloat(sampleBuffInc_fp5);
	// Dbg_printf("rate = %f, inc = %d\n", speed, sampleBuffInc_fp5);
	return speed;
}

// expecting a value between 0 and 1, or higher for trouble ...
AudioTrack *AudioTrack::setLevel(float level){
	iVolumeLevel = max(0, level * WAV_PWM_RANGE);
	return this;
}

void AudioTrack::advance(){
	int32_t playbackStart_fp5 = inttofp5(playbackStart);
	int32_t playbackEnd_fp5 = inttofp5(min((playbackStart + playbackLen), buf->sampleLen));

	sampleBuffCursor_fp5 += sampleBuffInc_fp5;

	// sampleBuffInc_fp5 may be negative:
	if (sampleBuffInc_fp5 > 0)
		while (sampleBuffCursor_fp5 >= playbackEnd_fp5){
			if (! isLooping()) {
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
			if (! isLooping()) {
				playing = false;
				sampleBuffCursor_fp5 = playbackEnd_fp5;
				//Dbg_println(".deyalp");
			} else
				sampleBuffCursor_fp5 += (playbackEnd_fp5 - playbackStart_fp5);
				loopCount--;
		}
}

uint32_t AudioTrack::fillFromRawStream(Stream &f){
	bool p = playing;
	if (p)
		pause();
	playbackLen = buf->fillFromRawStream(f);
	playbackStart = buf->sampleStart; // probably 0
	if (p)
		play();
	return playbackLen;
}

uint32_t AudioTrack::fillFromRawFile(fs::FS &fs, String filename){
	bool p = playing;
	if (p)
		pause();
	playbackLen = buf->fillFromRawFile(fs, filename);
	playbackStart = buf->sampleStart; // probably 0
	if (p)
		play();
	return playbackLen;
}


////////////////////////////////////////
// Picomix object manages all the audio objects in the system,
// and defines the ISR that pumps AudioTracks into txBufs.

// This gets called once at startup to set up PWM
void Picomix::init(unsigned char ring) {

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
	irq_set_exclusive_handler(PWMSTREAMER_DMA_INTERRUPT, ISR_play);
}

AudioTrack *Picomix::addTrack(AudioTrack *t){
	for (int i=0;i<MAX_TRACKS;i++){
		if (trk[i] == NULL){
			trk[i] = t;
			return trk[i];
		}
	}
	return NULL;
}

AudioTrack *Picomix::addTrack(uint8_t channels, long int sampleLength){
	for (int i=0;i<MAX_TRACKS;i++){
		if (trk[i] == NULL){
			trk[i] = new AudioTrack(channels, sampleLength);
			return trk[i];
		}
	}
	return NULL;
}

AudioTrack *Picomix::addTrack(fs::FS &fs, String filename){
	File f = fs.open(filename, "r");
  if (!f) {
    Dbg_println("file open failed");
		return NULL;
  } else {
    Dbg_printf("%s: %d bytes\n", filename, f.size());
  }
	// presuming mono
	AudioTrack *t = addTrack(1, f.size() / 2);
	t->fillFromRawStream(f);
	f.close();
	return t;
}

void Picomix::start(){
	enableISR(true);
	pwm.start();
}

void Picomix::stop(){
	pwm.stop();
	enableISR(false);
}

void Picomix::enableISR(bool on){
	if (on) {
		irq_set_enabled(PWMSTREAMER_DMA_INTERRUPT, true);
	} else {
		irq_set_enabled(PWMSTREAMER_DMA_INTERRUPT, false);
	}
}

//
// In our double-buffered DMA scheme, this ISR refills
// the idle buffer with new samples and rewinds its
// DMA channel, even as the other DMA continues
// to pump the other buffer's samples through the PWM.
//
void __not_in_flash_func(Picomix::ISR_play)() {
	static auto &my = onlyInstance();
	AudioBuffer *idleTxBuf;
	int idleSide;

	my.ISRcounter++;

	// Acknowledge interrupt, rewind DMA & determine idle side of the double buffer
	idleSide = my.pwm.resetIRQ();

	idleTxBuf = &my.transferBuffer[idleSide];

	// fill idle channel buffer
	for (int i = 0; i < idleTxBuf->samples; i += idleTxBuf->channels) {
		long scaledSample = 0;

		for (int t=0; t<MAX_TRACKS; t++){
			if (my.trk[t] == NULL)
				continue;
			if (my.trk[t]->buf == NULL)
				continue;
			if (! my.trk[t]->playing)
				continue;

			if (my.trk[t]->iVolumeLevel == 0) {
				my.trk[t]->advance();
				continue;
			}

			scaledSample +=
													(
														my.trk[t]->buf->data[fp5toint(my.trk[t]->sampleBuffCursor_fp5)]
														 * my.trk[t]->iVolumeLevel  // scale numerator (can be from 0 to more than WAV_PWM_RANGE
													)
													/ WAV_PWM_RANGE      // scale denominator
				;
			my.trk[t]->advance();
		}

		interp1->accum[0] = (short) scaledSample; // hard-limit with interpolator

		// put that sample in both channels
		for (int j=0; j < idleTxBuf->channels; j++)
			idleTxBuf->data[i+j] = interp1->peek[0] + (WAV_PWM_RANGE / 2); // shift to positive
	}
}

//////////
//
// These basic utils generate signals in the sampleBuffer.
// In every case it's signed values between -(WAV_PWM_RANGE/2)
// and WAV_PWM_COUNT
//
// Fill buffer with value of an arbitrary function across a given range,
// repeated some number of times.
//

// This version takes a sample start & length, updating sampleStart & sampleLen
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

// This version fills in between the buffer's current values of sampleStart & sampleLen
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

// Load a raw PCM audio file of signed 16-bit samples, which you can create with sox:
//       sox foo.wav foo.raw
//
uint32_t AudioBuffer::fillFromRawFile(fs::FS &fs, String filename){
	File f = fs.open(filename, "r");
  if (!f) {
    Dbg_println("file open failed");
		return 0;
  } else {
    Dbg_printf("%s: %d bytes\n", filename, f.size());
  }
  uint32_t sampleLen = fillFromRawStream(f);
	f.close();
	return sampleLen;
}

// Fill the buffer from an input stream of signed 16-bit samples
uint32_t AudioBuffer::fillFromRawStream(Stream &f){
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

	// Now shift those signed-16-bit samples down to our output bit resolution of WAV_PWM_BITS
	for (bc = sampleStart; bc<sampleLen; bc++) {
		data[bc] = data[bc] / (pow(2, (16 - WAV_PWM_BITS)));
	}

	return sampleLen;
}

