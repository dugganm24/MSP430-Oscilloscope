/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include "buttons.h"
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "sampling.h"
#include "driverlib/timer.h"

#include <math.h>
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"

#include "audio_waveform.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/pwm.h"

#define PI 3.14159265358979f
#define NFFT 1024 // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state) + sizeof(kiss_fft_cpx)*(NFFT-1))

uint32_t gSystemClock = 120000000; // [Hz] system clock frequency
#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz
#define PWM_STEP_SIZE 500
#define AUDIO_PWM_FREQUENCY 465000

volatile uint32_t gTime = 8345; // time in hundredths of a second

float load=0;
float unload=0;
float cpu_load=0.0;

uint32_t CPULoad(void);
void DMA_Init(void);
void Timer_Init(void);
void PWM_ISR(void);
void TimerCapture(void);
void PWM_Init(void);


int sample[LCD_HORIZONTAL_MAX];
int i = 0;
int y = 0;
int prev_y = 0;
int rising = 1;
int trigger;
int voltsPerDiv = 4;
int fVoltsPerDiv[] = {0.1,0.2,0.5,1,2};
volatile int buffer[ADC_BUFFER_SIZE];
int fft = 0;
float out_db[NFFT];

int decimal = 0;
int integer = 0;

uint16_t sample_fft[NFFT];


// Kiss FFT config memory
static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE];
size_t buffer_size = KISS_FFT_CFG_SIZE;
kiss_fft_cfg cfg; // Kiss FFT config
// complex waveform and spectrum buffers
static kiss_fft_cpx in[NFFT], out[NFFT];
int i;
// init Kiss FFT

DataType button;

const char * const gVoltageScaleStr[] = {
                                         "100 mV", "200 mV", "500 mV", "1V", "2V"
};

volatile uint32_t current_count = 0;
volatile uint32_t prev_count = 0;
volatile uint32_t period = 0;
int PWMperiod = 0;

uint32_t gPWMSample = 0; // PWM sample counter
uint32_t gSamplingRateDivider = AUDIO_PWM_FREQUENCY/AUDIO_SAMPLING_RATE; // sampling rate divider

/*
 *  ======== main ========
 */


int main(void)
{
    // hardware initialization goes here
    IntMasterDisable();

    // Enable the Floating Point Unit, and permit ISRs to use it
    FPUEnable();
    FPULazyStackingEnable();

    // Initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    // configure M0PWM2, at GPIO PF2, BoosterPack 1 header C1 pin 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinConfigure(GPIO_PF3_M0PWM3);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    // configure the PWM0 peripheral, gen 1, outputs 2 and 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    // use system clock without division
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1,
                    roundf((float)gSystemClock/PWM_FREQUENCY));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                     roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,
                     roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    ADC_Init();
    ButtonInit();
    DMA_Init();
    Timer_Init();
    PWM_Init();

    unload = CPULoad();

    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font


    /* Start BIOS */
    BIOS_start();

    return (0);
}

void IntMasterEnableTask(UArg arg1, UArg arg2)
{
    IntMasterEnable();
}

void WaveformTask(void){
    while(1){
        Semaphore_pend(Waveform_sem, BIOS_WAIT_FOREVER);
        trigger = rising ? RisingTrigger(): FallingTrigger();

        if(!fft){
            for(i = 0; i < LCD_HORIZONTAL_MAX - 1; i++) {
                sample[i] = gADCBuffer[ADC_BUFFER_WRAP(trigger - LCD_HORIZONTAL_MAX/2 + i)];
            }
        }

        if(fft){
            for(i=0; i < NFFT; i++){
                sample_fft[i] = gADCBuffer[ADC_BUFFER_WRAP(i)];
            }
        }
        Semaphore_post(Processing_sem);
    }
}

void ProcessingTask(void){
    int i;
    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size);

    while(1){
        Semaphore_pend(Processing_sem, BIOS_WAIT_FOREVER);

        if(!fft){
            float fScale = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * fVoltsPerDiv[voltsPerDiv]);
            int i = 0;
            for(i=0;i<LCD_HORIZONTAL_MAX-1;i++){
                buffer[i] = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)sample[i]-ADC_OFFSET)); //calculate y value
            }
            Semaphore_post(Display_sem);
            Semaphore_post(Waveform_sem);
        }

        if(fft){
            for (i = 0; i < NFFT; i++) { // generate an input waveform
                in[i].r = sample_fft[i]; // real part of waveform
                in[i].i = 0; // imaginary part of waveform
            }
            kiss_fft(cfg, in, out); // compute FFT
            // convert first 128 bins of out[] to dB for display
            for(i=0; i < LCD_HORIZONTAL_MAX - 1; i++){
                out_db[i] = (160-(10 * log10f(out[i].r * out[i].r + out[i].i * out[i].i))) ;
            }
            Semaphore_post(Display_sem);
            Semaphore_post(Waveform_sem);
        }

    }
}

void DisplayTask(void){

    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};
    char str[50];


    while(1){
        Semaphore_pend(Display_sem, BIOS_WAIT_FOREVER);

        load = CPULoad();
        cpu_load = (1.0 - load/unload)*100; //calculate CPU utilization

        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black
        GrContextForegroundSet(&sContext, ClrBlue);

        int j=0;
        for(j = -3; j < 4; j++){
            GrLineDrawH(&sContext, 0, LCD_HORIZONTAL_MAX - 1, LCD_VERTICAL_MAX/2 + j * PIXELS_PER_DIV);
            GrLineDrawV(&sContext, LCD_VERTICAL_MAX /2 + j * PIXELS_PER_DIV, 0, LCD_HORIZONTAL_MAX-1);
        }

        if(!fft){
            GrContextForegroundSet(&sContext, ClrYellow);
            for(i = 0; i < LCD_HORIZONTAL_MAX - 1; i++) {
                if(buffer[i] >= LCD_HORIZONTAL_MAX){
                    y = LCD_HORIZONTAL_MAX - 1;
                }
                else if(buffer[i]<0){
                    y = 0;
                }
                else{
                    y = buffer[i];
                }
                GrLineDraw(&sContext, i, prev_y, i+1, y);
                prev_y = y;
            }

            GrContextForegroundSet(&sContext, ClrWhite);
            if(rising){
                GrLineDraw(&sContext, 105, 10, 115, 10);
                GrLineDraw(&sContext, 115, 10, 115, 0);
                GrLineDraw(&sContext, 115, 0, 125, 0);
                GrLineDraw(&sContext, 112, 6, 115, 2);
                GrLineDraw(&sContext, 115, 2, 118, 6);
            }

            if(!rising){
                GrLineDraw(&sContext, 105, 10, 115, 10);
                GrLineDraw(&sContext, 115, 10, 115, 0);
                GrLineDraw(&sContext, 115, 0, 125, 0);
                GrLineDraw(&sContext, 112, 3, 115, 7);
                GrLineDraw(&sContext, 115, 7, 118, 3);
            }

            if(!checkTrigger){
                GrLineDraw(&sContext, 105, 10, 115, 10);
                GrLineDraw(&sContext, 115, 10, 115, 0);
                GrLineDraw(&sContext, 115, 0, 125, 0);
            }


            GrStringDraw(&sContext, "20 us", -1, 4, 0, false);
            GrStringDraw(&sContext, gVoltageScaleStr[voltsPerDiv], -1, 50, 0, false);

            decimal = (int)(cpu_load * 10)%10;
            integer = (int)cpu_load;
            snprintf(&str, 40, "CPU Load = %d.%d%%", integer, decimal);
            GrStringDraw(&sContext, str, -1, 0, 100, false);

            uint32_t PeriodDisplay = roundf((float)gSystemClock/(PWM_FREQUENCY + PWM_STEP_SIZE*PWMperiod));

            snprintf(&str, sizeof(str), "f=%5d Hz", gSystemClock/PeriodDisplay);
            GrStringDraw(&sContext, str, -1, 0, 120, false);

            snprintf(&str, sizeof(str), "T=%4d", PeriodDisplay);
            GrStringDraw(&sContext, str, -1, 75, 120, false);

            GrFlush(&sContext);

        }

        if(fft){
            GrContextForegroundSet(&sContext, ClrYellow);
            for(i=0; i<LCD_HORIZONTAL_MAX-1; i++){
                y = out_db[i];
                GrLineDraw(&sContext, i, prev_y, i+1, y);
                prev_y = y;
            }


            GrContextForegroundSet(&sContext, ClrWhite);
            GrStringDraw(&sContext, "20 kHz", -1, 4, 0, false);
            GrStringDraw(&sContext, "20 dB", -1, 50, 0, false);
            decimal = (int)cpu_load%10;
            integer = (int)cpu_load;
            snprintf(&str, 40, "CPU Load = %d.%d%%", integer, decimal);
            GrStringDraw(&sContext, str, -1, 1, 120, false);
            GrFlush(&sContext);

        }

    }
}

void ButtonTask(void){
    while(1){

        Semaphore_pend(Button_sem, BIOS_WAIT_FOREVER);

        TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag

        // read hardware button state
        uint32_t gpio_buttons =
                (~GPIOPinRead(GPIO_PORTJ_BASE, 0xff) & (GPIO_PIN_1 | GPIO_PIN_0)) |
                (~GPIOPinRead(GPIO_PORTH_BASE, 0xff) & (GPIO_PIN_1)) << 1 |
                (~GPIOPinRead(GPIO_PORTK_BASE, 0xff) & (GPIO_PIN_6)) >> 3 |
                (~GPIOPinRead(GPIO_PORTD_BASE, 0xff) & (GPIO_PIN_4)); // EK-TM4C1294XL buttons in positions 0 and 1

        uint32_t old_buttons = gButtons;    // save previous button state
        ButtonDebounce(gpio_buttons);       // Run the button debouncer. The result is in gButtons.
        ButtonReadJoystick();               // Convert joystick state to button presses. The result is in gButtons.
        uint32_t presses = ~old_buttons & gButtons;   // detect button presses (transitions from not pressed to pressed)
        presses |= ButtonAutoRepeat();      // autorepeat presses if a button is held long enough

        static bool tic = false;
        static bool running = true;

        if (presses & 1) { // EK-TM4C1294XL button 1 pressed
            fifo_put('1');
            Mailbox_post(mailbox_button_input, &button, BIOS_WAIT_FOREVER);
        }

        if (presses & 2){
            fifo_put('2');
            Mailbox_post(mailbox_button_input, &button, BIOS_WAIT_FOREVER);
        }

        if (presses & 8){
            fifo_put('3');
            Mailbox_post(mailbox_button_input, &button, BIOS_WAIT_FOREVER);
        }

        if (presses & 4){
            fifo_put('4');
            Mailbox_post(mailbox_button_input, &button, BIOS_WAIT_FOREVER);
        }

        if (presses & 16){
            fifo_put('5');
            Mailbox_post(mailbox_button_input, &button, BIOS_WAIT_FOREVER);
        }

        if(presses & 32){
            fifo_put('6');
            Mailbox_post(mailbox_button_input, &button, BIOS_WAIT_FOREVER);
        }

        if(presses & 64){
            fifo_put('7');
            Mailbox_post(mailbox_button_input, &button, BIOS_WAIT_FOREVER);
        }

        if (running) {
            if (tic) gTime++; // increment time every other ISR call
            tic = !tic;
        }
    }
}

void UserInputTask(void){
    while(1){
        Mailbox_pend(mailbox_button_input, &button, BIOS_WAIT_FOREVER);
        while(fifo_get(&button)){
            switch(button){
            case '1':
                voltsPerDiv = (voltsPerDiv + 1) > 4 ? 4 : (voltsPerDiv + 1);
                break;
            case '2':
                voltsPerDiv = (voltsPerDiv -1) <= 0 ? 0 : (voltsPerDiv-1);
                break;
            case '3':
                rising = !rising;
                break;
            case '4':
                fft = !fft;
                break;
            case '5':
                PWMIntEnable(PWM0_BASE, PWM_INT_GEN_2);
                break;
            case '6':
                PWMperiod = PWMperiod <= -1000 ? -1000: (PWMperiod - 1);
                PWM_Init();
                break;
            case '7':
                PWMperiod = PWMperiod >= 1000 ? 1000: (PWMperiod + 1);
                PWM_Init();
                break;
            }

        }
    }
}


void Clock0Task(void){
    Semaphore_post(Button_sem);
}

uint32_t CPULoad(void){
    float i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE,TIMER_A); //start one-shot timer

    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT)) {
        i++;
    }
    return i;
}

void Timer_Init(void){
    // config GPIO PD0 as timer input T0CCP0 at BoosterPack Connector #1 pin 14
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_T0CCP0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerDisable(TIMER0_BASE, TIMER_BOTH);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    // use maximum load value
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xffff);
    // use maximum prescale value
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xff);
    TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void TimerCapture(void){
    TIMER0_ICR_R = TIMER_ICR_CAECINT;

    current_count = TimerValueGet(TIMER0_BASE, TIMER_A);
    period = (current_count - prev_count) & 0xfffff;
    prev_count = current_count;
}

void PWM_Init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1); // PG1 = M0PWM5
    GPIOPinConfigure(GPIO_PG1_M0PWM5);
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    // configure the PWM0 peripheral, gen 2, outputs 5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); // use system clock without division
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, roundf((float)gSystemClock/(AUDIO_PWM_FREQUENCY)+PWM_STEP_SIZE*PWMperiod));   // Initialize to period of 258 clock cycles
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, roundf((float)gSystemClock/(AUDIO_PWM_FREQUENCY)+PWM_STEP_SIZE*PWMperiod)*0.4f); // Initialize to 50% duty cycle
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_ZERO);    // configures PWM interrupts (every time the counter reaches 0)
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
}

void PWM_ISR(void){
    PWMGenIntClear(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_ZERO); // clear PWM interrupt flag
    // waveform sample index
    int i = (gPWMSample++) / gSamplingRateDivider;
    // write directly to the PWM compare B register
    PWM0_2_CMPB_R = 1 + gWaveform[i];
    if (i == gWaveformSize - 1) { // if at the end of the waveform array
        // disable these interrupts
        PWMIntDisable(PWM0_BASE, PWM_INT_GEN_2);
        // reset sample index so the waveform starts from the beginning
        gPWMSample = 0;
    }
}



