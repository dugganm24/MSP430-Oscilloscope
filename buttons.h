/*
 * buttons.h
 *
 *  Created on: Aug 12, 2012, modified 9/8/2017
 *      Author: Gene Bogdanov
 *
 *  Button debouncer, calibrated for 200 samples/sec.
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

#include <stdint.h>
#include "sampling.h"

#define BUTTON_COUNT 5				// number of buttons excluding joystick directions
#define BUTTON_AND_JOYSTICK_COUNT 9 // number of buttons including joystick directions
#define BUTTON_SAMPLES_PRESSED 2	// number of samples before a button is considered pressed
#define BUTTON_SAMPLES_RELEASED 5	// number of samples before a button is considered released
// counter value indicating button pressed state
#define BUTTON_PRESSED_STATE (BUTTON_SAMPLES_RELEASED*BUTTON_SAMPLES_PRESSED)
#define BUTTON_STATE_INCREMENT (BUTTON_PRESSED_STATE/BUTTON_SAMPLES_PRESSED)
#define BUTTON_STATE_DECREMENT (BUTTON_PRESSED_STATE/BUTTON_SAMPLES_RELEASED)

#define BUTTON_SCAN_RATE 200    // [Hz] button scanning interrupt rate
#define BUTTON_INT_PRIORITY 64  // button interrupt priority (higher number is lower priority)

#define BUTTON_AUTOREPEAT_INITIAL 100   // how many samples must read pressed before autorepeat starts
#define BUTTON_AUTOREPEAT_NEXT 10       // how many samples must read pressed before the next repetition

#define JOYSTICK_UPPER_PRESS_THRESHOLD 3595     // above this ADC value, button is pressed
#define JOYSTICK_UPPER_RELEASE_THRESHOLD 3095   // below this ADC value, button is released
#define JOYSTICK_LOWER_PRESS_THRESHOLD 500      // below this ADC value, button is pressed
#define JOYSTICK_LOWER_RELEASE_THRESHOLD 1000   // above this ADC value, button is released

//#define ADC_SAMPLING_RATE 2000000   // [samples/sec] desired ADC sampling rate
#define CRYSTAL_FREQUENCY 25000000  // [Hz] crystal oscillator frequency used to calculate clock rates

extern volatile uint32_t gButtons;	// debounced button state, one per bit in the lowest bits
extern uint32_t gJoystick[2];       // joystick coordinates
extern uint32_t gADCSamplingRate;   // [Hz] actual ADC sampling rate

extern volatile bool flag;
typedef char DataType;


// initialize all button and joystick handling hardware
void ButtonInit(void);

// update the debounced button state in the global variable gButtons
// the input argument is a bitmap of raw button state from the hardware
void ButtonDebounce(uint32_t buttons);

// sample joystick and convert to button presses
void ButtonReadJoystick(void);

// autorepeat button presses if a button is held long enough
uint32_t ButtonAutoRepeat(void);

int fifo_put(DataType data);

int fifo_get(DataType *data);



#endif /* BUTTONS_H_ */
