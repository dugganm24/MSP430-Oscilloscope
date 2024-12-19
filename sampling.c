#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "sysctl_pll.h"
#include "buttons.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/adc.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "Crystalfontz128x128_ST7735.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "sampling.h"
#include <xdc/std.h>
#include <xdc/cfg/global.h>

#include "driverlib/udma.h"
#pragma DATA_ALIGN(gDMAControlTable, 1024) // address alignment required
tDMAControlTable gDMAControlTable[64]; // uDMA control table (global)

#define ADC_BUFFER_SIZE 2048 // size must be a power of 2
// index wrapping macro
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))
// latest sample index
#define ADC_OFFSET 2048
#define ADC_SEQUENCE_0 0
//volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE]; // circular buffer
volatile uint32_t gADCErrors = 0; // number of missed ADC deadlines

volatile bool checkTrigger;

volatile bool gDMAPrimary = true;

volatile int32_t DMA_Index;

void ADC_Init(void){
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
      GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // GPIO setup for analog input AIN3
      // initialize ADC peripherals
      SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
      // ADC clock
      uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
      uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; // round up
      ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL,
      pll_divisor);
      ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL,
      pll_divisor);
      // choose ADC1 sequence 0; disable before configuring
      ADCSequenceDisable(ADC1_BASE, ADC_SEQUENCE_0);
      ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS,0);//ecify the "Always" trigger
      // in the 0th step, sample channel 3 (AIN3)
      // enable interrupt, and make it the end of sequence
      ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
      // enable the sequence. it is now sampling
      ADCSequenceEnable(ADC1_BASE, 0);
      // enable sequence 0 interrupt in the ADC1 peripheral
      //ADCIntEnable(ADC1_BASE, 0);
      //IntPrioritySet(INT_ADC1SS0, 0); //t ADC1 sequence 0 interrupt priority
      // enable ADC1 sequence 0 interrupt in int. controller
     // IntEnable(INT_ADC1SS0);

      ADCSequenceDMAEnable(ADC1_BASE, 0); // enable DMA for ADC1 sequence 0
      ADCIntEnableEx(ADC1_BASE, ADC_INT_DMA_SS0); // enable ADC1 sequence 0 DMA interrupt
}

void DMA_Init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();
    uDMAControlBaseSet(gDMAControlTable);
    // assign DMA channel 24 to ADC1 sequence 0
    uDMAChannelAssign(UDMA_CH24_ADC1_0);
    uDMAChannelAttributeDisable(UDMA_SEC_CHANNEL_ADC10, UDMA_ATTR_ALL);
    // primary DMA channel = first half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R, (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2);
    // alternate DMA channel = second half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R, (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);
    uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);
}


int RisingTrigger(void){

    checkTrigger = true;

    DMA_Index = getADCBufferIndex();

    //Step 1
    int x = DMA_Index - LCD_HORIZONTAL_MAX/2;

    //Step 2
    int x_stop = x - ADC_BUFFER_SIZE/2;
    for (; x > x_stop; x--){
        if (gADCBuffer[ADC_BUFFER_WRAP(x)] >= ADC_OFFSET &&
             gADCBuffer[ADC_BUFFER_WRAP(x-1)] < ADC_OFFSET)
            break;
    }
    //Step 3
    if (x == x_stop)
        x = DMA_Index - LCD_HORIZONTAL_MAX/2;
        checkTrigger = false;
    return x;
}

int FallingTrigger(void){
    checkTrigger = true;
    DMA_Index = getADCBufferIndex();
    //Step 1
    int x = DMA_Index - LCD_HORIZONTAL_MAX/2;

    //Step 2
    int x_stop = x - ADC_BUFFER_SIZE/2;
    for (; x > x_stop; x--){
        if (gADCBuffer[ADC_BUFFER_WRAP(x)] < ADC_OFFSET &&
             gADCBuffer[ADC_BUFFER_WRAP(x-1)] >= ADC_OFFSET)
            break;
    }
    //Step 3
    if (x == x_stop)
        x = DMA_Index - LCD_HORIZONTAL_MAX/2;
        checkTrigger = false;
    return x;
}




void ADC_ISR(void){

    ADCIntClearEx(ADC1_BASE, ADC_INT_DMA_SS0); // clear the ADC1 sequence 0 DMA interrupt flag
    // Check the primary DMA channel for end of transfer, and
    // restart if needed.
    if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT) == UDMA_MODE_STOP) {
    // restart the primary channel (same as setup)
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R, (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2);
    // DMA is currently occurring in the alternate buffer
    gDMAPrimary = false;
    }
    // Check the alternate DMA channel for end of transfer, and
    // restart if needed.
    // Also set the gDMAPrimary global.
    if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT) == UDMA_MODE_STOP) {
        // restart the primary channel (same as setup)
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R, (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);
        // DMA is currently occurring in the alternate buffer
        gDMAPrimary = true;
    }// The DMA channel may be disabled if the CPU is paused by the debugger
    if (!uDMAChannelIsEnabled(UDMA_SEC_CHANNEL_ADC10)) {
    // re-enable the DMA channel
    uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);
    }

    /*
    // clear ADC1 sequence0 interrupt flag in the ADCISC register
   ADC1_ISC_R = ADC_ISC_IN0;
    // check for ADC FIFO overflow
    if(ADC1_OSTAT_R & ADC_OSTAT_OV0) {
            gADCErrors++; // count errors
            ADC1_OSTAT_R = ADC_OSTAT_OV0; // clear overflow condition
    }
    gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1);
    // read sample from the ADC1 sequence 0 FIFO
    gADCBuffer[gADCBufferIndex] =  ADC1_SSFIFO0_R;
    */
}

int32_t getADCBufferIndex(void)
{
    int32_t index;

    IArg key = GateHwi_enter(gateHwi0);

    if (gDMAPrimary) { // DMA is currently in the primary channel
            index = ADC_BUFFER_SIZE/2 - 1 -
            uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT);
            GateHwi_leave(gateHwi0, key);
}
    else { // DMA is currently in the alternate channel
        index = ADC_BUFFER_SIZE - 1 -
         uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT);
         GateHwi_leave(gateHwi0, key);
}
    return index;
}
