// Motors.c
// Runs on TM4C123
// Use Hardware pwm for PB4 and PB5 to generate pulse-width modulated outputs.
// Daniel Valvano
// March 28, 2014
// Modified by Group 6 10/9/2024

/*
	Group 6 
	Brandon Jamjampour 
	Daniel Banuelos
	Anthony Nuth
	Anastacsia Estrella
*/
#include <stdint.h>
#include "Motors.h"
#include "tm4c123gh6pm.h"

#define PERIOD 10000				// Total PWM period
#define PLL_SYSDIV_50MHZ 7

#define PB5_PB4_MASK  0x30
#define AlTFuncPB5_PB4  0x00FF0000
#define AltFunctionDef  0x00440000

#define DIR_SLP_MASK  0xCC
#define ALTFUNC_CLEAR  0xFF00FF00

#define LEFT_ENABLE_MOTOR 0x00000004
#define RIGHT_ENABLE_MOTOR 0x00000008

// Wheel PWM connections: on PB4/M0PWM0:Left wheel, PB5/M0PWM0:Right wheel
void Motors_INIT(void){
  SYSCTL_RCGCPWM_R |= 0x01;             // 1) activate PWM0
  SYSCTL_RCGCGPIO_R |= 0x02;            // 2) activate port B: 000010
  while((SYSCTL_RCGCGPIO_R&0x02) == 0){};
	GPIO_PORTB_AFSEL_R |= PB5_PB4_MASK;           // enable alt funct on PB5/PB4
  GPIO_PORTB_PCTL_R &= ~AlTFuncPB5_PB4;     // configure PB5/PB4 as PWM0
  GPIO_PORTB_PCTL_R |= AltFunctionDef;
  GPIO_PORTB_AMSEL_R &= ~PB5_PB4_MASK;          // disable analog functionality on PB5/PB4
  GPIO_PORTB_DEN_R |= PB5_PB4_MASK;             // enable digital I/O on PB6
  GPIO_PORTB_DR8R_R |= PB5_PB4_MASK;    // enable 8 mA drive on PB5/PB4
  SYSCTL_RCC_R = 0x00100000 |           // 3) use PWM divider
    (SYSCTL_RCC_R & (~0x001E0000));   //    configure for /2 divider: PWM clock: 50Mhz/2=25MHz
  SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV; // 3) use PWM divider
  SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; //    clear PWM divider field
  SYSCTL_RCC_R |= SYSCTL_RCC_PWMDIV_2;  //    configure for /2 divider

	PWM0_1_CTL_R = 0;                     // 4) re-loading down-counting mode
	PWM0_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE|PWM_1_GENA_ACTLOAD_ZERO;   // PB4: low on LOAD, high on CMPA down
	PWM0_1_GENB_R = (PWM_1_GENB_ACTCMPBD_ONE|PWM_1_GENB_ACTLOAD_ZERO); // PB5: 0xC08: low on LOAD, high on CMPB down
  PWM0_1_LOAD_R = PERIOD - 1;           // 5) cycles needed to count down to 0
  PWM0_1_CTL_R |= 0x00000001;           // 7) start PWM1
}

// Start left wheel
void Start_L(void) {
  PWM0_ENABLE_R |= LEFT_ENABLE_MOTOR;          // PB4/M0PWM2
}

// Start right wheel
void Start_R(void) {
  PWM0_ENABLE_R |= RIGHT_ENABLE_MOTOR;          // enable PB5/M0PWM2
}

// Stop left wheel
void Stop_L(void) {
  PWM0_ENABLE_R &= ~LEFT_ENABLE_MOTOR;          // PB4/M0PWM2
}

// Stop right wheel
void Stop_R(void) {
  PWM0_ENABLE_R &= ~RIGHT_ENABLE_MOTOR;          // enable PB5/M0PWM2
}
//start both wheels
void Start_Both_Wheels(void){
	Start_R();
	Start_L();
}
//stop both wheels
void Stop_Both_Wheels(void) {
	Set_L_Speed(STOP);				// update duty cycle
	Set_R_Speed(STOP);
	Stop_L();
	Stop_R();
}
void move_foward(void){
	DIRECTION = FORWARD;
	Set_L_Speed(SPEED_10);				// update duty cycle
	Set_R_Speed(SPEED_10);
	Start_R();
	Start_L();
}
void move_backwards(void){
	DIRECTION = BACKWARD;
	Set_L_Speed(SPEED_10);				// update duty cycle
	Set_R_Speed(SPEED_10);
	Start_R();
	Start_L();
}
//wide foward left
void foward_left_turn_wallfollower(void){
	DIRECTION = FORWARD;
	Set_L_Speed(SPEED_15);				// update duty cycle
	Set_R_Speed(SPEED_10);
	Start_R();
	Start_L();
}

void foward_right_turn_wallfollower(void){
	DIRECTION = FORWARD;
	Set_L_Speed(SPEED_10);				// update duty cycle
	Set_R_Speed(SPEED_16);
	Start_R();
	Start_L();
}
void foward_left_turn_object_follower(void){
	DIRECTION = FORWARD;
	Set_L_Speed(SPEED_15);				// update duty cycle
	Set_R_Speed(SPEED_10);
	Start_R();
	Start_L();
}

void foward_right_turn_object_follower(void){
	DIRECTION = FORWARD;
	Set_L_Speed(SPEED_10);				// update duty cycle
	Set_R_Speed(SPEED_20);
	Start_R();
	Start_L();
}

//backward left
void backward_left(void){
	DIRECTION = BACKWARD;
	Set_L_Speed(SPEED_10);				// update duty cycle
	Set_R_Speed(SPEED_20);
	Start_R();
	Start_L();
}
void sharp_right(void){
	DIRECTION = FORWARD;
	Set_L_Speed(STOP);				// update duty cycle
	Set_R_Speed(SPEED_50);
	Start_R();
	Stop_L();
}
void sharp_left(void){
	DIRECTION = FORWARD;
	Set_L_Speed(SPEED_50);				// update duty cycle
	Set_R_Speed(STOP);
	Start_L();
	Stop_R();
}
//backward right
void backward_right(void){
	DIRECTION = BACKWARD;
	Set_L_Speed(SPEED_20);				// update duty cycle
	Set_R_Speed(SPEED_10);
	Start_R();
	Start_L();
}

// Set duty cycle for Left Wheel: PB4
void Set_L_Speed(uint16_t duty){
  PWM0_1_CMPA_R = duty - 1;             // 6) count value when output rises
}
// Set duty cycle for Right Wheel: PB5
void Set_R_Speed(uint16_t duty){
  PWM0_1_CMPB_R = duty - 1;             // 6) count value when output rises
}

// Initialize port B pins PB7/6-3/2 for output
// PB7/6 - PB3/2 control directions of the two motors: PB7 R-SLP,PB6 R-DIR, PB3 L-SLP, PB2 L-DIR
// Inputs: None
// Outputs: None
void Dir_Init(void){
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB; //activate B clock
	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)!= SYSCTL_RCGC2_GPIOB){} //wait for clk
	
	GPIO_PORTB_AMSEL_R &= ~DIR_SLP_MASK; //disable analog function
	GPIO_PORTB_PCTL_R &= ~ALTFUNC_CLEAR; //GPIO clear bit PCTL
	GPIO_PORTB_DIR_R |= DIR_SLP_MASK; //PB7 - 6 / PB3 - 2 output
	GPIO_PORTB_AFSEL_R &= ~DIR_SLP_MASK; //no alternate function
	GPIO_PORTB_DEN_R |= DIR_SLP_MASK; //enable digital pins PB7 - 6 / PB3 - 2
}
