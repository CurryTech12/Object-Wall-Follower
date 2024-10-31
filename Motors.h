// Motors.h
// Runs on TM4C123

/*
	Group 6 
	Brandon Jamjampour 
	Daniel Banuelos
	Anthony Nuth
	Anastacsia Estrella
*/
#include <stdint.h>

#define DIRECTION (*((volatile unsigned long *)0x40005330))
#define FORWARD 		0xCC	  //11001100, both wheels move forward PB7 R-SLP,PB6 R-DIR, PB3 L-SLP, PB2 L-DIR
#define BACKWARD 		0x88	  //1000 1000, both wheels move backward sleep pin always one
#define LEFTPIVOT   0x8C    // left moving backward, right moving fordward
#define RIGHTPIVOT  0xC8    // right moving backward, left moving fordward

// duty cycles for different speeds
#define STOP 1
#define SPEED_10 1000
#define SPEED_15 1500
#define SPEED_16 1600
#define SPEED_20 2000
#define SPEED_30 3000
#define SPEED_50 5000
#define SPEED_55 5500
#define SPEED_56 5600
#define SPEED_60 6000
#define SPEED_80 8000
#define SPEED_98 9800


void PLL_Init(void);

// Wheel PWM connections: on PB6/M0PWM0:Left wheel, PB7/M0PWM0:Right wheel
void Motors_INIT(void);

// Start left wheel
void Start_L(void);

// Start right wheel
void Start_R(void);

// Stop left wheel
void Stop_L(void);

// Stop right wheel
void Stop_R(void);

//move foward
void move_foward(void);

//move backward
void move_backwards(void);

void sharp_right(void);

void sharp_left(void);

//start both wheels
void Start_Both_Wheels(void);

//stop both wheels
void Stop_Both_Wheels(void);


void backward_left(void);

void backward_right(void);

void foward_left_turn_wallfollower(void);

void foward_right_turn_wallfollower(void);

void foward_right_turn_object_follower(void);

void foward_left_turn_object_follower(void);



// Change duty cycle of left wheel: PB4
void Set_L_Speed(uint16_t duty);

// change duty cycle of right wheel: PB5
void Set_R_Speed(uint16_t duty);

//dir for pins PB7-6 and PB3-2
void Dir_Init(void);

