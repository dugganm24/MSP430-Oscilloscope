#ifndef SAMPLING_H_
#define SAMPLING_H_

#include <stdint.h>

#define ADC_SAMPLING_RATE 2000000
#define CRYSTAL_FREQUENCY 25000000
#define PIXELS_PER_DIV 20
#define ADC_INT_PRIORITY 16
#define VIN_RANGE 3.3
#define ADC_BITS 12
#define ADC_OFFSET 2048
#define ADC_BUFFER_SIZE 2048
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))

extern volatile uint16_t gADCBuffer[];
extern volatile int32_t gADCBufferIndex;
extern volatile uint32_t gADCErrors;
extern volatile bool checkTrigger;


void ADC_Init(void);

void ADC_ISR(void);

int RisingTrigger(void);

int FallingTrigger(void);

void DMA_Init(void);

int32_t getADCBufferIndex(void);


#endif
