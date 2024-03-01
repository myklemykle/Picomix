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

#include "config.h"

// don't compile this for Teensy:
#ifdef TARGET_RP2040

#include <Arduino.h> // for Serial
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/interp.h"
#include "pins.h"
#include "RP2040Audio.h"

// C++, why you can't just read the header file jeez ...
extern int RP2040Audio::wavDataCh[2];
extern int RP2040Audio::wavCtrlCh[2];
extern unsigned int RP2040Audio::pwmSlice[2];
extern short* RP2040Audio::bufPtr[2];


// Digital Limiter: 
// interp1 will limit signed integers to within +/- WAV_PWM_RANGE/2
void setup_interp1_clamp(){
		interp_config cfg = interp_default_config();
    interp_config_set_clamp(&cfg, true);
    interp_config_set_signed(&cfg, true);
    interp_set_config(interp1, 0, &cfg);

		interp1->base[0] = 0 - (WAV_PWM_RANGE / 2);;
    interp1->base[1] = (WAV_PWM_RANGE / 2) -1;
}

// This gets called once at startup to set up both stereo PWMs for both ports
void RP2040Audio::init() {
	/////////////////////////
	// set up interp1 for clamping (used by ISR)
	//
	setup_interp1_clamp();

  ////////////////////////////
  // Set up PWM slices

  // pwmSlice converts samples to PWM audio on gpio pins
  pwmSlice[0] = pwm_gpio_to_slice_num(ring1);
  pwmSlice[1] = pwm_gpio_to_slice_num(ring2);

  // triggerSlice generates an interrupt in sync with that one,
  // but is scaled to only once per TRANSFER_BUFF_SAMPLES samples.
  // It triggers a refill of the transfer buffer.  It is just
  // for IRQs & isn't connected to any pins.
  //
  // use slice 0 here, which corresponds to gpio 0&1 or 16&17,
  // which I happen to know I'm not using.

  // halt:
	pwm_set_enabled(pwmSlice[0], false);
	pwm_set_enabled(pwmSlice[1], false);
  pwm_set_enabled(TRIGGER_SLICE, false);


  // initialize:
  pwm_config pCfg;
  for (int i = 0; i < 2; i++) {
    pCfg = pwm_get_default_config();
    pwm_config_set_wrap(&pCfg, WAV_PWM_COUNT);
    pwm_init(pwmSlice[i], &pCfg, false);
    pwm_set_irq_enabled(pwmSlice[i], false);
  }

  pwm_config tCfg = pwm_get_default_config();
  pwm_config_set_clkdiv_int_frac(&tCfg, (int)TRANSFER_WINDOW_XFERS, 0);
  pwm_config_set_wrap(&tCfg, WAV_PWM_COUNT);
  pwm_init(TRIGGER_SLICE, &tCfg, false);
  pwm_set_irq_enabled(TRIGGER_SLICE, true);
  irq_set_enabled(PWM_IRQ_WRAP, true);


  // line them up & adjust levels
  for (int i = 0; i < 2; i++) {
		pwm_set_both_levels(pwmSlice[i], 0, 0);
    pwm_set_counter(pwmSlice[i], 0);
	}
	// give the buffer writes a head start on the DMA reads. see tweak() for explanation of the magic number 28.
  pwm_set_counter(TRIGGER_SLICE, 28);  

  // don't make them go yet. Turn on DMA first!
  //pwm_set_mask_enabled((1<<pwmSlice) | (1<<TRIGGER_SLICE) | pwm_hw->en);
}


bool RP2040Audio::isPlaying(unsigned char port) {
  if (dma_channel_is_busy(wavDataCh[port]))
		return true;

  // This is tricky: 
	// approx 1 time per transfer buffer rewind 
	// (that's (TRANSFER_BUFF_SAMPLES/2) / WAV_SAMPLE_RATE hz) 
	// the loop control DMA resets this DMA.
  // This DMA channel could be not_busy at that moment 
	// even when the channel isPlaying.
	//
	// That reset window could be pretty short. 
	// I don't know how many cycles it takes for a chain between DMAs 
	// to happen -- maybe zero! The loop DMA writes a single word
	// to rewind the data DMA, so that should only need 1 cycle.
	// But with 4 DMA channels round-robining, that's really 4 cycles, minimum.
	//
	// So the odds of hitting this reset window when we call dma_channel_is_busy()
	// are as described above: extremely low, but not impossible.
	//
  // How to do better? Check twice.
	// dma_channel_is_busy() seems to compile down to 9 instructions, 
	// so it takes at least 9 cycles.
	//
	// In the unlikely event we hit the reset window with the last check, 
	// I hope this next check will still spot a busy channel.
	
  return dma_channel_is_busy(wavDataCh[port]);
}


// TODO: why do we hear a click on pause & play?
// I guess going from 50% DC (silence) to 0% DC (logic false) & back.
// Can we let it down more gently?
// Maybe first set the pins to float, or input mode?
void RP2040Audio::pause(unsigned char port) {
  if (wavDataCh[port] >= 0 && dma_channel_is_busy(wavDataCh[port])) {
    dma_channel_abort(wavDataCh[port]);
    dma_channel_abort(wavCtrlCh[port]);
    pwm_set_enabled(pwmSlice[port], false);
  }
	Dbg_printf("paused port %d\n",port);
}

// HACK presumes exactly two ports.
// Better would be to track the number of ports there are.
void RP2040Audio::pauseAll(){
	pause(0);
	pause(1);
	// .. once everything is off, we can pause the trigger slice.
	pwm_set_enabled(TRIGGER_SLICE, false);
}


void RP2040Audio::play(unsigned char port) {
  dma_channel_config wavDataChConfig, wavCtrlChConfig;
  bufPtr[port] = &(transferBuffer[port][0]);

  if (wavDataCh[port] < 0) {  // if uninitialized
    Dbg_println("getting dma");
    Dbg_flush();
    wavDataCh[port] = dma_claim_unused_channel(true);
  }
  if (wavCtrlCh[port] < 0) {  // if uninitialized
    Dbg_println("getting dma");
    Dbg_flush();
    wavCtrlCh[port] = dma_claim_unused_channel(true);
  }

  // Dbg_printf("pwm dma channel %d\n", wavDataCh[port]);
  // Dbg_printf("loop dma channel %d\n", wavCtrlCh[port]);
  // Dbg_printf("pwm slice num %d\n", pwmSlice[port]);
  // Dbg_flush();

  /*********************************************/
  /* Stop playing audio if DMA already active. */
  /*********************************************/
  this->pause(port);

  /****************************************************/
  /* Don't start playing audio if DMA already active. */
  /****************************************************/
  if (!dma_channel_is_busy(wavDataCh[port])) {

    //////
    // configure loop DMA channel, which resets the WAV DMA channel start address, then chains to it.
    // ( https://vanhunteradams.com/Pico/DAC/DMA_DAC.html )

    wavCtrlChConfig = dma_channel_get_default_config(wavCtrlCh[port]);
    channel_config_set_read_increment(&wavCtrlChConfig, false);
    channel_config_set_write_increment(&wavCtrlChConfig, false);
    channel_config_set_transfer_data_size(&wavCtrlChConfig, DMA_SIZE_32);
    channel_config_set_chain_to(&wavCtrlChConfig, wavDataCh[port]);  // chain to the wav PWM channel when finished.
    dma_channel_configure(
      wavCtrlCh[port],                         // Channel to be configured
      &wavCtrlChConfig,                        // The configuration we just created
      &dma_hw->ch[wavDataCh[port]].read_addr,  // Write address (wav PWM channel read address)
      &bufPtr[port],                                 // Read address (POINTER TO AN ADDRESS) ... contains the address that this DMA writes to the other DMA's read-address.
      1,                                       // transfer 32 bits one time.
      false                                    // Don't start immediately
    );

    /****************************************************/
    /* Configure state machine DMA from WAV PWM memory. */
    /****************************************************/
    wavDataChConfig = dma_channel_get_default_config(wavDataCh[port]);
    channel_config_set_read_increment(&wavDataChConfig, true);
    channel_config_set_write_increment(&wavDataChConfig, false);
    channel_config_set_transfer_data_size(&wavDataChConfig, DMA_SIZE_32);     // 32 bytes at a time (l & r 16-bit samples)
    channel_config_set_dreq(&wavDataChConfig, pwm_get_dreq(pwmSlice[port]));  // let PWM cycle request transfers
    channel_config_set_chain_to(&wavDataChConfig, wavCtrlCh[port]);           // chain to the loop-control channel when finished.
    dma_channel_configure(
      wavDataCh[port],                                                  // channel to config
      &wavDataChConfig,                                                 // this configuration
      (void*)(PWM_BASE + PWM_CH0_CC_OFFSET + (0x14 * pwmSlice[port])),  // write to pwm channel (pwm structures are 0x14 bytes wide)
      transferBuffer[port],                                                   // read from here (this value will be overwritten if we start the other loop first)
      TRANSFER_BUFF_SAMPLES / 2,                                           // transfer exactly (samples/2) times (cuz 2 samples per transfer)
      false);

    /**********************/
    /* Start WAV PWM DMA. */
    /**********************/
    dma_start_channel_mask(1 << wavCtrlCh[port]);
		// Dbg_printf("dma channel %d started\n",wavCtrlCh[port]);

    // start the PWM to generate the DREQ signals for the DMA:
    pwm_set_mask_enabled((1 << pwmSlice[port]) | (1 << TRIGGER_SLICE) | pwm_hw->en);
		// Dbg_printf("pwm channels %d & %d enabled\n",pwmSlice[port],TRIGGER_SLICE);
  } else {
		// Dbg_printf("dma channel %d busy.\n",wavDataCh[port]);
	}

}


// constructor/initalizer cuz c++ is weird about this
RP2040Audio::RP2040Audio() {
  wavDataCh[0] = wavDataCh[1] = -1;
  wavCtrlCh[0] = wavCtrlCh[1] = -1;
  pwmSlice[0] = pwmSlice[1] = 0;
}

extern volatile uint32_t iVolumeLevel;

// init() sets up an interrupt every TRANSFER_WINDOW_XFERS output samples,
// then this ISR refills the transfer buffer with TRANSFER_BUFF_SAMPLES more samples,
// which is TRANSFER_WINDOW_XFERS * TRANSFER_BUFF_CHANNELS
void __not_in_flash_func(RP2040Audio::ISR_play)() { 
  static unsigned int sampleBuffCursor = 0;
  pwm_clear_irq(TRIGGER_SLICE);

  for (int i = 0; i < TRANSFER_BUFF_SAMPLES; i+=TRANSFER_BUFF_CHANNELS ) {

    // Since amplitude can go over max, use interpolator #1 in clamp mode
		// to hard-limit the signal.
    interp1->accum[0] = 
											(short)( 
													(long) (
														sampleBuffer[sampleBuffCursor++]
														 * iVolumeLevel  // scale numerator (can be from 0 to more than WAV_PWM_RANGE
														)
                         / WAV_PWM_RANGE      // scale denominator (TODO right shift here? or is the compiler smart?)
                         )
      ;
		// TODO: set up interp0 to perform this add?
		short scaledSample = interp1->peek[0] + (WAV_PWM_RANGE / 2); // shift to positive
																																 
		// put that in both channels of both outputs:
		for (int j=0;j<TRANSFER_BUFF_CHANNELS;j++) 
			transferBuffer[0][i+j] = transferBuffer[1][i+j] = scaledSample;

    if (sampleBuffCursor == SAMPLE_BUFF_SAMPLES)
      sampleBuffCursor = 0;
  }
}

// Fancier version, time-scaling a 1hz sample into 4 output channels at 4 rates, but not (yet) adjusting volume.
// instead of FP, we will use long ints, and shift off the 8 LSB 
extern volatile uint32_t sampleCursorInc[4];
void __not_in_flash_func(RP2040Audio::ISR_test)() {
  static long sampleBuffCursor[4] = {0,0,0,0};
  pwm_clear_irq(TRIGGER_SLICE);

	for (uint8_t port = 0; port<2; port++) {
		for (int i = 0; i < TRANSFER_BUFF_SAMPLES; i+=TRANSFER_BUFF_CHANNELS) {
			for (uint8_t chan = 0; chan < TRANSFER_BUFF_CHANNELS; chan++){
				uint8_t pc = (port<<1)+chan;

				// copy from mono samplebuf to stereo transferbuf
				// transferBuffer[port][i+chan] = sampleBuffer[ sampleBuffCursor[pc]] ;
				transferBuffer[port][i+chan] = sampleBuffer[ sampleBuffCursor[pc] / 256 ] ;
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
// fill buffer with white noise (signed)
void RP2040Audio::fillWithNoise(){
	randomSeed(666);
	for(int i=0; i<SAMPLE_BUFF_SAMPLES; i++){
		sampleBuffer[i] = random(WAV_PWM_RANGE) - (WAV_PWM_RANGE / 2);
	}
}

// fill buffer with sine waves
void RP2040Audio::fillWithSine(uint count, bool positive){
	const float twoPI = 6.283;
	const float scale = (WAV_PWM_RANGE) / 2;

	for (int i=0; i<SAMPLE_BUFF_SAMPLES; i+= SAMPLE_BUFF_CHANNELS){
		for(int j=0;j<SAMPLE_BUFF_CHANNELS; j++)
			sampleBuffer[i + j] = (int) (scale
					* sin( (float)i * count / (float)SAMPLE_BUFF_SAMPLES * twoPI ) 
				 ) + (positive ? scale : 0) ; // shift sample to positive? (so the ISR routine doesn't have to)
	}
}

// fill buffer with square waves
void RP2040Audio::fillWithSquare(uint count, bool positive){
	for (int i=0; i<SAMPLE_BUFF_SAMPLES; i+= SAMPLE_BUFF_CHANNELS)
		for(int j=0;j<SAMPLE_BUFF_CHANNELS; j++)
		 if ((i*count)%SAMPLE_BUFF_SAMPLES < (SAMPLE_BUFF_SAMPLES / 2)){ 
			 sampleBuffer[i + j] = positive ? WAV_PWM_RANGE : (WAV_PWM_RANGE)/ 2;
		 } else {
			 sampleBuffer[i + j] = positive ? 0 : 0 - ((WAV_PWM_RANGE) / 2);
		 }
}

// fill buffer with sawtooth waves running negative to positive
// (Still slightly buggy ...)
void RP2040Audio::fillWithSaw(uint count, bool positive){
	const float twoPI = 6.283;
	const float scale = (WAV_PWM_RANGE) / 2;

	for (int i=0; i<SAMPLE_BUFF_SAMPLES; i+= SAMPLE_BUFF_CHANNELS){
		for(int j=0;j<SAMPLE_BUFF_CHANNELS; j++)
			sampleBuffer[i + j] = (int) 

				// i 																																// 0 -> SAMPLE_BUFF_SAMPLES-1
				// i / (SAMPLE_BUFF_SAMPLES - 1) 																			// 0 -> 1
				// i * WAV_PWM_RANGE / (SAMPLE_BUFF_SAMPLES -1) 												// 0 -> WAV_PWM_RANGE
				// (i * count) * WAV_PWM_RANGE / (SAMPLE_BUFF_SAMPLES -1) 							// 0 -> count*WAV_PWM_RANGE
				(i * count * WAV_PWM_RANGE / (SAMPLE_BUFF_SAMPLES -1) ) % WAV_PWM_RANGE // 0 -> WAV_PWM_RANGE, count times
					- (positive ? 0 : (WAV_PWM_RANGE / 2)) ; // shift to 50% negative?
	}
}

// TODO: fillWithRamp, so we can ramp on & off for clickless pause/play

// PWM timing adjustment utility.
// One can use this, along with a scope, to experimentally determine the appropriate 
// moment for the sample buffer refill IRQ to fire, where the buffer writes can
// race ahead of the DMA channel buffer reads without 
// wrapping around and overtaking the DMA's cursor position.  
// Last time I checked this, 28 was a good value with no distorition.
// 
// That's the meaning of  pwm_set_counter(TRIGGER_SLICE, 28)  in init() above.)
//
void RP2040Audio::tweak() {
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
		} else {
			Dbg_print(c);
		}
	}
}


#endif // TARGET_RP2040
