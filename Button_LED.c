#include "tm4c123gh6pm.h"

#define Button_MASK 0x11
#define PCTL_MASK 0x0000FFF0
#define PCTL_BUTTON_MASK 0x000F000F
#define LED_MASK  0x0E
#define LED_BUTTON_MASK 0x1F

void LED_Init_buttonInit(void){  unsigned long volatile delay;
  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTF_AMSEL_R &= ~LED_BUTTON_MASK;      // disable analog functionality on PF4-0
  GPIO_PORTF_PCTL_R &= ~PCTL_MASK; // configure PF3-1 as GPIO
  GPIO_PORTF_DIR_R |= LED_MASK;     // make PF3-1 out (built-in LEDs)
  GPIO_PORTF_AFSEL_R &= ~LED_MASK;  // disable func make PF3-1 out (built-in LEDs)
  GPIO_PORTF_DEN_R |= LED_MASK;     // enable digital I/O  PF3-1(built-in LEDs)
  GPIO_PORTF_DATA_R &= ~LED_MASK;   // make PF3-1 low
}

void Switch_Init(void){  unsigned long volatile delay;
  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
  GPIO_PORTF_CR_R = Button_MASK;         // allow changes to PF4,0
  GPIO_PORTF_DIR_R &= ~Button_MASK;    // (c) make PF4,0 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~Button_MASK;  //     disable alt funct on PF4,0
  GPIO_PORTF_DEN_R |= Button_MASK;     //     enable digital I/O on PF4,0
  GPIO_PORTF_PCTL_R &= ~PCTL_BUTTON_MASK; //  configure PF4,0 as GPIO
  GPIO_PORTF_AMSEL_R &= ~Button_MASK;  //     disable analog functionality on PF4,0
  GPIO_PORTF_PUR_R |= Button_MASK;     //     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~Button_MASK;     // (d) PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~Button_MASK;    //     PF4,PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~Button_MASK;    //     PF4,PF0 falling edge event
  GPIO_PORTF_ICR_R = Button_MASK;      // (e) clear flags 4,0
  GPIO_PORTF_IM_R |= Button_MASK;      // (f) arm interrupt on PF4,PF0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 2
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}