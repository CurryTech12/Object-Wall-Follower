// FollowingRobot.c
// Runs TM4C123
// Main program CECS 347 project 2 - A Follwoing Robot
// by Min He, 03/17/2024
//Group 6 
//Brandon Jamjampour, Daniel Banuelos, Anthony Nuth, Anastacia Estrella
//Description: This code is the implementation of a follwoing robot with two
//separate modes.  Mode 1 is an object-following mode, where the robot will
//maintain a 20cm distance between any object in front of it.  This mode is
//indicated by a blue on-board LED.  Mode 2 is a left/right-wall-following mode,
//where the robot will follow the perimeter of a wall.  This mode is indicated
//by a green on-board LED. On-board SW2 is used to toggle between these two modes.

#include "tm4c123gh6pm.h"
#include "ADC1.h"
#include "Motors.h"
#include "PLL.h"
#include "button_led.h"

#define RED   0x02
#define BLUE  0x04
#define GREEN 0x08 
#define SW2_MASK 0x01
#define SW1_MASK 0x10

// move the following constant definitions to ADC0SS3.h
#define TOO_FAR 		(800)  // replace the zero with the ADC output value for maximum distance
#define FOLLOW_DIST (1613)  // replace the zero with the ADC output value for object following distance
#define TOO_CLOSE 	(3102)  // replace the zero with the ADC output value for minimum distance
#define super_closer (4000)

#define close       (1150)
#define far         (950)
#define CRASH       (1500) 
#define LED_OUTPUT (*((volatile unsigned long *)0x40025038))//output for leds


enum robot_modes {INACTIVE, OBJECT_FOLLOWER, WALL_FOLLOWER};//enum to go throug the modes

// Function prototypes
// external functions
extern void DisableInterrupts(void);
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // low power mode

// functions defined in this file
void System_Init(void);
void object_follower(void);
void wall_follower(void);
void Delay(uint8_t n_halfs);
void left_followerAlgo(uint32_t ahead1,uint32_t left1, uint32_t right1,uint32_t LF);
void right_followerAlgo(uint32_t ahead1, uint32_t left1, uint32_t right1, uint32_t RF);


enum robot_modes mode= INACTIVE;
uint8_t SW1_toggle_Count = 0;//flags to check the sw1
uint8_t SW2_toggle_Count = 0;//flag to check sw2

uint8_t leftflag = 0;
uint8_t rightflag = 0;

int main(void){	
	System_Init();//init the entire system function
	LED_OUTPUT = RED;//system inactive waiting for button presses
	
  while(1){
		switch (mode) {
			case OBJECT_FOLLOWER:
				object_follower();
			  break;
			case WALL_FOLLOWER:
				wall_follower();
			  break;
			default:
				WaitForInterrupt();
			  break;				
		}
  }
}

void System_Init(void){
	DisableInterrupts();
	PLL_Init();									// 16MHz system clock
	ADC1_Init321(); 	// ADC initialization for PE2 - PE0
	Dir_Init();             // initialize PB32: left direction pins, PB76: right direction pins
	Motors_INIT(); // PB4:Left wheel, PB5:Right wheel  
	LED_Init_buttonInit();//init LED
	Switch_Init();//switch init
  EnableInterrupts();	
}

void object_follower(void)
{
	Stop_Both_Wheels();
	uint8_t i;
	uint32_t ain3center;
	uint32_t ain2left;
	uint32_t ain1right;


	
	// Calibrate the sensor
	for (i=0;i<10;i++) {
		READ_FIC_FLITER(&ain3center,&ain2left,&ain1right);
	} 
	
  // wait until an obstacle is in the right distant range.
 	do {
		READ_FIC_FLITER(&ain3center,&ain2left,&ain1right);
		
	} while ((ain3center>TOO_CLOSE) || (ain3center<TOO_FAR) || (ain2left>TOO_CLOSE) || (ain2left<TOO_FAR) || (ain1right>TOO_CLOSE) || (ain1right<TOO_FAR));//if object not in distance range then do nothing busy wait
		READ_FIC_FLITER(&ain3center,&ain2left,&ain1right);
		
  while ((ain3center<TOO_CLOSE) && (ain3center>TOO_FAR) || (ain1right<TOO_CLOSE) && (ain1right>TOO_FAR) || (ain2left<TOO_CLOSE) && (ain2left>TOO_FAR)) {	//if there is obstcale in range read a value from ADC						
		
		if (ain3center>FOLLOW_DIST && ain3center > TOO_FAR) { // negative logic: too close, move back
			move_backwards();
		}
		else if (ain3center<FOLLOW_DIST && ain3center >TOO_FAR){ // negative logic: too far, move forward
			move_foward();
		}
	 else if(ain2left < FOLLOW_DIST && ain2left > TOO_FAR){
			foward_left_turn_object_follower();
		}
	 else if(ain2left > FOLLOW_DIST && ain2left < TOO_CLOSE){
		 backward_right();
	 }
	 else if(ain1right < FOLLOW_DIST && ain1right > TOO_FAR){
			foward_right_turn_object_follower();
	 }
	 else if(ain1right > FOLLOW_DIST && ain1right < TOO_CLOSE){
		 backward_left();
	 }
		else { // right distance, stop
			Stop_Both_Wheels();
		}
		READ_FIC_FLITER(&ain3center,&ain2left,&ain1right);
  }
	Stop_Both_Wheels();//when it exits make sure the wheels are stop again
}

void wall_follower(void){//each time the robot finnishes a run go back to begining of this function
	uint8_t i;
	uint32_t ahead,fwd_left,fwd_right;

	for(i = 0; i < 10; i++){
		 READ_FIC_FLITER(&ahead, &fwd_left, &fwd_right);//cablirate the sensors
	}

    // Read sensor values
    READ_FIC_FLITER(&ahead, &fwd_left, &fwd_right);//read one more value
			if(fwd_left > fwd_right){//if robot closer to left we know its left wall following 
				leftflag = 1;//set flag to left following
			}
			if(fwd_right > fwd_left){//if robot is closer to right we know its right wall following
				rightflag = 1;//set flag to right following
			}
			
	while(ahead < super_closer){//when the car isnt stopped meaning we dont put an object superclose
			if(leftflag == 1){//if flag is 1 
				left_followerAlgo(ahead,fwd_left,fwd_right,leftflag);//run the left wall follow algo
			}
			if(rightflag == 1){//if rightflag is high run righ wall follow algo
				right_followerAlgo(ahead,fwd_left,fwd_right,rightflag);
			}
			READ_FIC_FLITER(&ahead,&fwd_left,&fwd_right);//after calling this func sample again
		}
		if(ahead > super_closer){//if we stop the car this indicates we completed a wall following journey
			//reset both flags to prepare for the next following
			leftflag = 0;
			rightflag = 0;
			Stop_Both_Wheels();//then stop the car as well
		}
	}//then go back to begininning to recalibrate the sensors again for the next start
	

void GPIOPortF_Handler(void){ // called on touch of either SW1 or SW2
	for (uint32_t i = 0; i < 80000; i++){} // debounce
	Stop_Both_Wheels();
		
  if(GPIO_PORTF_RIS_R&SW2_MASK){  // SW2 touch
		SW2_toggle_Count = SW2_toggle_Count + 1;//when the button pressed inc count by 1
		if(mode != INACTIVE && SW2_toggle_Count == 1){//if odd meaning button pressed first time
			mode = WALL_FOLLOWER;
			LED_OUTPUT = GREEN;
		}
		if(mode != INACTIVE && SW2_toggle_Count == 2){//if count is 2 means button pressed a second time
			mode = OBJECT_FOLLOWER;
			LED_OUTPUT = BLUE;
			SW2_toggle_Count = 0;//reset count to zero so when next button pressed it goes to 1 and saves space
		}
    GPIO_PORTF_ICR_R = SW2_MASK;  // acknowledge flag0
  }
	
  if(GPIO_PORTF_RIS_R&SW1_MASK){  // SW1 touch
		SW1_toggle_Count = SW1_toggle_Count + 1;
		if(SW1_toggle_Count == 1){//when button pressed first time we go to object
			mode = OBJECT_FOLLOWER;
			LED_OUTPUT = BLUE;
		}
		if(SW1_toggle_Count == 2){//if pressed again go to inactive 
			mode = INACTIVE;
			LED_OUTPUT = RED;
			Stop_Both_Wheels();
			SW1_toggle_Count = 0;
			SW2_toggle_Count = 0;//reset the flag so it goes through correct sequence
		}
		GPIO_PORTF_ICR_R = SW1_MASK;  // acknowledge flag4
  }
}
//left following algo
void left_followerAlgo(uint32_t ahead1, uint32_t left1, uint32_t right1,uint32_t lf) {
	
    if (left1 > close && ahead1 < CRASH) {
        foward_right_turn_wallfollower();  // Adjust right if too close
    } else if (left1 < close && ahead1 < CRASH) {
        foward_left_turn_wallfollower();   // Adjust left if veering off
    } else if (ahead1 > CRASH && left1 > right1) {
        sharp_right();  // At a corner, need to turn right
    }
			else if(left1 < TOO_FAR && ahead1 < TOO_FAR && lf == 1){
				foward_left_turn_wallfollower();//turn left when left sees nothing and ahead sees nothing
		}
}

// Right-wall-following algorithm
void right_followerAlgo(uint32_t ahead1,uint32_t left1, uint32_t right1,uint32_t rf) {
	
    if (right1 > close && ahead1 < CRASH) {
        foward_left_turn_wallfollower();  // Adjust left if too close
    } else if (right1 < close && ahead1 < CRASH) {
        foward_right_turn_wallfollower();  // Adjust right if veering off
    } else if (ahead1 > CRASH && left1 < right1) {
        sharp_left();  // At a corner, need to turn left
		}
			else if(right1 < TOO_FAR && ahead1 < TOO_FAR && rf == 1){//turn right when we see nothing on right 
				foward_right_turn_wallfollower();
		}
}
