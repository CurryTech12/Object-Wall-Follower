// ADC1PD0SS3.c
// Runs on TM4C123
// Provide functions that initialize ADC0 SS3 to be triggered by
// software and trigger a conversion, wait for it to finish,
// and return the result.
// Daniel Valvano
// October 20, 2013
// Modified by Min He, 10/09/2022

// This file provide initialization function for two analog channels:
// PE2/AIN1 and PD0/AIN7

#include "tm4c123gh6pm.h"
#include <stdint.h>

#define ADC1_PSSI_SS1 0x0002   // start sample sequencer 1
#define ADC1_ISC_SS1  0x0002    // acknowledge sample sequencer 1 interrupt
#define ADC1_RIS_SS1  0x02
#define PE2_PE0_MASK  0x07


void ADC1_Init321(void){ 
 SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;   // 1) activate clock for Port E
	while ((SYSCTL_RCGCGPIO_R&SYSCTL_RCGCGPIO_R4)!=SYSCTL_RCGCGPIO_R4){}

	GPIO_PORTE_DIR_R &= ~PE2_PE0_MASK;      // 2) make PE2 - PE0 input
  GPIO_PORTE_AFSEL_R |= PE2_PE0_MASK;     // 3) enable alternate function on PE2 - PE0
  GPIO_PORTE_DEN_R &= ~PE2_PE0_MASK;      // 4) disable digital I/O on PE2 - PE0
  GPIO_PORTE_AMSEL_R |= PE2_PE0_MASK;     // 5) enable analog function on PE2 - PE0
  GPIO_PORTE_PCTL_R = GPIO_PORTE_PCTL_R&0xFFFFF000;

	SYSCTL_RCGC0_R |= 0x00020000;   // 6) activate ADC1
	while ((SYSCTL_RCGC0_R&0x00020000)!=0x00020000){}
         
  SYSCTL_RCGC0_R &= ~0x00000300;  // 7) configure for 125K 
  ADC1_SSPRI_R = 0x3210;          // 8) Sequencer 0 is highest priority
  ADC1_ACTSS_R &= ~0x0002;        // 9) disable sample sequencer 1
  ADC1_EMUX_R &= ~0x00F0;         // 10) seq1 is software trigger
  ADC1_SSMUX1_R = 0x0123; // 11) channel Ain3 (PE0) lsb then Ain2(PE1) next Ain1 (PE2)
  ADC1_SSCTL1_R = 0x0600;         // 12) no TS0 D0, yes IE0 END0
	ADC1_IM_R &= ~0x0002;           // 13) disable SS1 interrupts
  ADC1_ACTSS_R |= 0x0002;         // 14 enable sample sequencer 1
}


void ADC_In321(uint32_t *ain3,uint32_t *ain2,uint32_t *ain1){
  ADC1_PSSI_R = 0x0002;            // 1) initiate SS1
  while((ADC1_RIS_R&0x02)==0){};   // 2) wait for conversion done
  *ain3 = ADC1_SSFIFO1_R&0xFFF;    // 3A) read first result//PE0 Center
  *ain2 = ADC1_SSFIFO1_R&0xFFF;    // 3B) read second result//PE1 Left sensors
  *ain1 = ADC1_SSFIFO1_R&0xFFF;    // 3C) read third result//PE2 Right
  ADC1_ISC_R = 0x0002;             // 4) acknowledge completion
}
void READ_FIC_FLITER(uint32_t *AIN3,uint32_t *AIN2,uint32_t *AIN1){//FIR filiter
	static unsigned long ain3previous = 0;
	static unsigned long ain2previous = 0;
	static unsigned long ain1previous = 0;
	
	uint32_t ain3newest;
	uint32_t ain2newest;
	uint32_t ain1newest;
	
	ADC_In321(&ain3newest,&ain2newest,&ain1newest);

	*AIN3 = (ain3newest + ain3previous)/2;
	*AIN2 = (ain2newest + ain2previous)/2;
	*AIN1 = (ain1newest + ain1previous)/2;
	
	ain3previous = ain3newest; ain2previous = ain2newest; ain1previous = ain1newest;

}