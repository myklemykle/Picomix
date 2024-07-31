// RP2040Audio learns from two codebases:
//
// Basic PWM audio configuation:
// PicoRecPlayAudio - Raspberry Pi Pico Audio Record/Playbak/WAV File
// Copyright (C) 2022 Jason Birch
//
// DMA audio playback strategy (frees the CPU for other tasks):
// https://vanhunteradams.com/Pico/DAC/DMA_DAC.html
//
// Thanks Jason! Thanks Van!
//
//

#include "dbg_macros.h"

#include "Arduino.h" // for Serial
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/interp.h"
#include "RP2040Audio.h"

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

// db refac part 1: double the loop PWM rate ...
void PWMStreamer::setup_loop_pwm_slice(unsigned char loopSlice){

	loopTriggerPWMSlice = loopSlice;

	// triggerSlice generates an interrupt in sync with that one,
	// but is scaled to only once per TRANSFER_BUFF_SAMPLES samples.
	// It triggers a refill of the transfer buffer.  It is just
	// for IRQs & isn't connected to any pins.

	pwm_set_enabled(loopTriggerPWMSlice, false);

	// initialize:

	tCfg = pwm_get_default_config();

	// HACK: now playing back at 1/3 speed, which is close-ish to a 44.1khz sample rate
	// pwm_config_set_wrap(&tCfg, WAV_PWM_COUNT);

	// db refac: double the PWM rate
	pwm_config_set_wrap(&tCfg, WAV_PWM_COUNT / 2);

	pwm_config_set_clkdiv(&tCfg, TRANSFER_WINDOW_XFERS * 3);

	// instead:
	// pwm_config_set_wrap(&tCfg, LOOP_PWM_COUNT);
	// pwm_config_set_clkdiv_int_frac(&tCfg, LOOP_PWM_NUM, LOOP_PWM_DEN);

	pwm_init(loopTriggerPWMSlice, &tCfg, false);
	pwm_set_irq_enabled(loopTriggerPWMSlice, true);
	irq_set_enabled(PWM_IRQ_WRAP, true);

	// give the buffer writes a head start on the DMA reads. see tweak() for explanation of the magic number 28.
	pwm_set_counter(loopTriggerPWMSlice, 28);
}

// ALERT: This methods assumes you've already set up the pwm slices
void PWMStreamer::setup_dma_channels(){
  dma_channel_config wavDataChConfig, wavCtrlChConfig;

	// Setup a DMA timer to feed samples to PWM at an adjustable rate:
	dmaTimer = dma_claim_unused_timer(true /* required */);
	dma_timer_set_fraction(dmaTimer, 1, WAV_PWM_RANGE * 3);  // HACK! play back at 1/3 PWM rate
	//dma_timer_set_fraction(dmaTimer, 1, WAV_PWM_RANGE);  // play back at PWM rate (works!)
	//dma_timer_set_fraction(dmaTimer, PWM_DMA_TIMER_NUM, PWM_DMA_TIMER_DEM);  // play back at (nearly) 44.1khz

	if (wavDataCh < 0) {  // if uninitialized
		Dbg_println("getting dma");
		Dbg_flush();
		wavDataCh = dma_claim_unused_channel(true);
	}
	if (wavCtrlCh < 0) {  // if uninitialized
		Dbg_println("getting dma");
		Dbg_flush();
		wavCtrlCh = dma_claim_unused_channel(true);
	}

	// Dbg_printf("pwm dma channel %d\n", wavDataCh);
	// Dbg_printf("loop dma channel %d\n", wavCtrlCh);

	/****************************************************/
	/* Configure data DMA to copy samples from xferbuff to PWM */
	/****************************************************/
	wavDataChConfig = dma_channel_get_default_config(wavDataCh);
	channel_config_set_read_increment(&wavDataChConfig, true);
	channel_config_set_write_increment(&wavDataChConfig, false);
	channel_config_set_transfer_data_size(&wavDataChConfig, DMA_SIZE_32);     // 32 bytes at a time (l & r 16-bit samples)

	int treq = dma_get_timer_dreq(dmaTimer);
	channel_config_set_dreq(&wavDataChConfig, treq);
	channel_config_set_chain_to(&wavDataChConfig, wavCtrlCh);           // chain to the loop-control channel when finished.
	dma_channel_configure(
		wavDataCh,                                                  // channel to config
		&wavDataChConfig,                                           // this configuration
		(void*)(PWM_BASE + PWM_CH0_CC_OFFSET + (0x14 * pwmSlice)),  // write to pwm channel (pwm structures are 0x14 bytes wide)
		tBuf->data,                                        // read from here (this value will be overwritten if we start the other loop first)
		TRANSFER_BUFF_SAMPLES / 2,                         // transfer exactly (samples/2) times (cuz 2 samples per transfer)
																											 // TODO: when double-buffering, divide that by 2 again.
		false);


	// configure loop DMA channel, which resets the WAV DMA channel start address, then chains to it.
	// ( https://vanhunteradams.com/Pico/DAC/DMA_DAC.html )

	wavCtrlChConfig = dma_channel_get_default_config(wavCtrlCh);
	channel_config_set_read_increment(&wavCtrlChConfig, false);
	channel_config_set_write_increment(&wavCtrlChConfig, false);
	channel_config_set_transfer_data_size(&wavCtrlChConfig, DMA_SIZE_32);
	channel_config_set_chain_to(&wavCtrlChConfig, wavDataCh);  // chain to the wav PWM channel when finished.

	// NOTE: tBufDataPtr is 32 bits of RAM holding a pointer to our buffer data
	// that the control DMA can copy to the wave DMA's config, over and over.
	dma_channel_configure(
		wavCtrlCh,                         // Channel to be configured
		&wavCtrlChConfig,                  // The configuration we just created
		&dma_hw->ch[wavDataCh].read_addr,  // Write address (wav PWM channel read address)
		&tBufDataPtr,                      // Read address (POINTER TO A POINTER)
		1,                                 // transfer 32 bits one time.
		false                              // Don't start immediately
	);
}

void PWMStreamer::init(unsigned char ring, unsigned char loopSlice) {
	////////////////////////////
	// Set up PWM slices
	setup_audio_pwm_slice(ring); // Enable PWM assigned to pins { ring, ring+1 }
	setup_loop_pwm_slice(loopSlice);

	/////////////////////////
	// claim and set up DMA channels
	setup_dma_channels();
}

bool PWMStreamer::isStarted() {
	if (dma_channel_is_busy(wavDataCh))
		return true;

	// Problem:
	// Approx 1 time per transfer buffer rewind
	// (that's (TRANSFER_BUFF_SAMPLES/2) / WAV_SAMPLE_RATE hz)
	// the loop control DMA resets this DMA.
	// This DMA channel could be not_busy() at that moment
	// even when the channel isStarted(). If this method is
	// called during that reset window, it could get something wrong.
	//
	// That reset window could be pretty short.
	// I don't know how many cycles it takes for a chain between DMAs
	// to happen -- maybe zero! The loop DMA writes a single word
	// to rewind the data DMA, so that should only need 1 cycle.
	// But with 4 DMA channels round-robining, that's really 4 cycles, minimum.
	// If other DMA is in use it could be more.
	//
	// So the odds of hitting this reset window when we call dma_channel_is_busy()
	// are as described above: extremely low, but not impossible.
	//
	// How to do better? Check twice.
	// dma_channel_is_busy() compiles down to 9 instructions,
	// so it takes at least 9 cycles + calling overhead.
	//
	// In the unlikely event we hit the reset window with the last check,
	// I hope this next check will still spot a busy channel.

	return dma_channel_is_busy(wavDataCh);
}

void PWMStreamer::_stop(){
	// abort both data & loop DMAs
	// disable both audio PWMs and the xfer trigger PWM.
	dma_channel_abort(wavDataCh);
	dma_channel_abort(wavDataCh);
	pwm_set_enabled(pwmSlice, false);
	// // .. once everything is off, we can pause the trigger slice.
	// pwm_set_mask_enabled(pwm_hw->en & ~(1 << pwmSlice) & ~(1 << loopTriggerPWMSlice));
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

	pwm_init(loopTriggerPWMSlice, &tCfg, false);
	pwm_set_counter(loopTriggerPWMSlice, 28);

	/**********************/
	/* Start WAV PWM DMA. */
	/**********************/
	dma_start_channel_mask(1 << wavCtrlCh);
	Dbg_println("dma channels started");

	// start the signal PWM and the xfer trigger PWM
	pwm_set_mask_enabled((1 << pwmSlice) | (1 << loopTriggerPWMSlice) | pwm_hw->en);
}


//////////////////////////////////////////////////
///  AudioCursor
//////////////////////////////////////////////////

void AudioCursor::_pause(){
	playing = false;
	// Dbg_println("paused");
}

void AudioCursor::_play(){
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

void AudioCursor::setLoops(int l){
	loops = max(-1, l);
}

inline bool AudioCursor::_doneLooping(){
	if (loops < 0) return false;
	if (loopCount > 1) return false;
	return true;
}

void AudioCursor::setSpeed(float speed){
	if (speed == 0) // no. do not do this.
		return;

	sampleBuffInc_fp5 = int(inttofp5(speed));
	// Dbg_printf("rate = %f, inc = %d\n", speed, sampleBuffInc_fp5);
}

float AudioCursor::getSpeed(){
	float speed = fp5tofloat(sampleBuffInc_fp5);
	// Dbg_printf("rate = %f, inc = %d\n", speed, sampleBuffInc_fp5);
	return speed;
}

// expecting a value between 0 and 1, or higher for trouble ...
void AudioCursor::setLevel(float level){
	iVolumeLevel = max(0, level * WAV_PWM_RANGE);
}

void AudioCursor::advance(){
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
// and defines the ISR that pumps them into the txBuf.

// This gets called once at startup to set up PWM
void RP2040Audio::init(unsigned char ring, unsigned char loopSlice) {

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
	pwm.init(ring, loopSlice);
}

// This ISR sends a single stereo audio stream to two different outputs on two different PWM slices.
// init() sets up an interrupt every TRANSFER_WINDOW_XFERS output samples,
// then this ISR refills the transfer buffer with TRANSFER_BUFF_SAMPLES more samples,
// which is TRANSFER_WINDOW_XFERS * TRANSFER_BUFF_CHANNELS
void __not_in_flash_func(RP2040Audio::ISR_play)() {
	static void *halfwayThroughTxBuffer = &transferBuffer.data[transferBuffer.samples / 2]; // TO FIX: glossing over non-even buffer size for now
	int txStartIdx, txEndIdx;
	short scaledSample;

	pwm_clear_irq(pwm.loopTriggerPWMSlice);

	ISRcounter++;

	// db refac: since we're triggered twice, write to just the half of the buffer that we're not DMAing from atm.
	void *currentPosition = (void*)dma_hw->ch[pwm.wavDataCh].read_addr;

	if ( currentPosition > halfwayThroughTxBuffer ) {
		txStartIdx = transferBuffer.samples / 2;
		txEndIdx = transferBuffer.samples;
	} else {
		txStartIdx = 0;
		txEndIdx = transferBuffer.samples / 2;
	}

	// TODO: get triggered by xferDMA instead?
	// then check tBufDataPtr here, and swap it, before restarting?
	// in that case we wouldn't need the rewind DMA.

	for (int i = txStartIdx; i < txEndIdx; i+=transferBuffer.channels) {
		// // sanity check:
		// if (sampleBuffCursor_fp5 < 0)
		// 	Dbg_println("!preD");

		// get next sample:
		if (!csr.playing){
			scaledSample = WAV_PWM_RANGE / 2; // 50% == silence

		} else {

			// Since amplitude can go over max, use interpolator #1 in clamp mode
			// to hard-limit the signal.
			interp1->accum[0] =
												(short)(
													(long) (
														sampleBuffer.data[fp5toint(csr.sampleBuffCursor_fp5)]
														 * csr.iVolumeLevel  // scale numerator (can be from 0 to more than WAV_PWM_RANGE
													)
													/ WAV_PWM_RANGE      // scale denominator
												)
				;
			// TODO: set up interp0 to perform this add? would that even be faster?
			scaledSample = interp1->peek[0] + (WAV_PWM_RANGE / 2); // shift to positive
		}

		// put that sample in both channels
		for (int j=0;j<transferBuffer.channels;j++)
			transferBuffer.data[i+j] = scaledSample;

		csr.advance();

	}
}

// This ISR time-scales a 1hz sample into 4 output channels at 4 rates, but not (yet) adjusting volume.
// instead of FP, we will use long ints, and shift off the 8 LSB
// This assumes that the sample buffer contains a single complete waveform.
extern volatile uint32_t sampleCursorInc[4];
//void __not_in_flash_func(RP2040Audio::ISR_test)() {
void RP2040Audio::ISR_test() {
	static long sampleBuffCursor[4] = {0,0,0,0};
	pwm_clear_irq(pwm.loopTriggerPWMSlice);

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

uint32_t AudioCursor::fillFromRawFile(Stream &f){
	playbackLen = buf->fillFromRawFile(f);
	playbackStart = buf->sampleStart; // probably 0
	return playbackLen;
}


// PWM timing adjustment utility.
// One can use this, along with a scope, to experimentally determine the appropriate
// moment for the sample buffer refill IRQ to fire, where the buffer writes can
// race ahead of the DMA channel buffer reads without
// wrapping around and overtaking the DMA's cursor position.
// Last time I checked this, 28 was a good value with no distorition.
//
// That's the meaning of  pwm_set_counter(loopTriggerPWMSlice, 28)  in init() above.)
//
void RP2040Audio::tweak() {
	if (! tweaking) return;

	char c;

	static int position = 0;

	static int step = 50;

	if (Serial.available()) {
		c = Serial.read();
		if (c == '+') {
			// advance
			for (int x = 0; x < step; x++) {
				pwm_advance_count(0);
			}
			position += step;
			Dbg_println(position);
		} else if (c == '-') {
			// retard
			for (int x = 0; x < step; x++) {
				pwm_retard_count(0);
			}
			position -= step;
			Dbg_println(position);
		} else if (c == '*') {
			// increase step size
			step = step * 2;
			Dbg_print('*');
			Dbg_println(step);
		} else if (c == '/') {
			// decrease step size
			step = step / 2;
			Dbg_print('*');
			Dbg_println(step);
		} else if (c == '!') {
			// done tweaking!
			tweaking = false;
		} else {
			Dbg_print(c);
		}
	}
}


