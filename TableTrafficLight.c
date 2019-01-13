// TableTrafficLight.c
// Author: Heather Minke and Ajem McConico
// Lab 5
// start date: 4/18/2018
// last modified date: 4/27/2018
// Runs on LM4F120 or TM4C123
// Index implementation of a Moore finite state machine to operate
// a traffic light.
// Daniel Valvano, Jonathan Valvano
// July 20, 2013

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Volume 1 Program 6.8, Example 6.4
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
   Volume 2 Program 3.1, Example 3.1

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// west red light connected to PB5
// west yellow light connected to PB4
// west green light connected to PB3

// south red light connected to PB2
// south yellow light connected to PB1
// south green light connected to PB0

// walk white light connected to PF1,2,3)
// walk warning light connected to PF1 (blinks)
// do not walk light connected to PF1

// south car detector connected to PE1 (1=car present)
// west car detector connected to PE0 (1=car present)
// walk detector connected to PE2 (1=pedestrean present)

#include "PLL.h"
#include "SysTick.h"

#define LIGHT                   (*((volatile unsigned long *)0x400050FC)) //output to PortB
#define GPIO_PORTB_OUT          (*((volatile unsigned long *)0x400050FC)) //bits 5-0
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
	
#define WALK_LIGHT							(*((volatile unsigned long *)0x40025038)) //output to PortF
#define GPIO_PORTF_OUT					(*((volatile unsigned long *)0x40025038)) //bits 3-1
#define GPIO_PORTF_DIR_R				(*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R			(*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_DEN_R				(*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_AMSEL_R			(*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R				(*((volatile unsigned long *)0x4002552C))
#define GPIO_PORTF_LOCK_R				(*((volatile unsigned long *)0x40025038))
#define GPIO_PORTF_CR_R					(*((volatile unsigned long *)0x40025038))
#define GPIO_PORTF_PUR_R				(*((volatile unsigned long *)0x40025038))

#define SENSOR                  (*((volatile unsigned long *)0x4002401C)) //input for PortE
#define GPIO_PORTE_IN           (*((volatile unsigned long *)0x4002401C)) //bits 2-0
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
	
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOE      0x00000010  //port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOB      0x00000002  //port B Clock Gating Control
#define SYSCTL_RCGC2_GPIOF			0x00000020	//port F Clock Gating Control


// Linked data structure
struct State {
  unsigned long Out; 
  unsigned long Time;  
  unsigned long Next[8];}; 
typedef const struct State STyp;
#define goS   					0
#define waitS 					1
#define allRedS   			2
#define goW		 					3
#define waitW						4
#define allRedW					5
#define walk						6
#define warning					7
#define allRedWaiting		8
STyp FSM[9]={
	//(output, how long it is going to last (value * 10ms))[where to go next]
	//(output, how long it is going to last (value * 10ms))[0, 1, 2, 3]
	//3000 = 30s, 500 = 5s, 200 = 2s
 {0x21,1000,{waitS,waitS,goS,waitS, waitS, waitS, waitS, waitS}}, 
 {0x22, 500,{allRedS,allRedS,allRedS,allRedS, allRedS, allRedS, allRedS, allRedS}}, 
 {0x24, 200,{goW,goW,goS,goW, walk, walk, walk, walk}},
 {0x0C,1000,{waitW,goW,waitW,waitW, waitW, waitW, waitW, waitW}},
 {0x14, 500,{allRedW, allRedW, allRedW, allRedW, allRedW, allRedW, allRedW, allRedW}},
 {0x24, 200,{goS, goW, goS, goS, walk, walk, walk, walk}},
 {0x24,1000,{warning, warning, warning, warning, warning, warning, warning, warning}},
 {0x24, 500,{allRedWaiting, allRedWaiting, allRedWaiting, allRedWaiting, allRedWaiting, allRedWaiting, allRedWaiting, allRedWaiting}},
 {0x24, 200,{goS, goW, goS, goS, goS, goW, goS, goS}}};
unsigned long S;  // index to the current state 
unsigned long Input; 
int main(void){ volatile unsigned long delay;
  PLL_Init();       									// 80 MHz, Program 10.1
  SysTick_Init();   									// Program 10.2
  SYSCTL_RCGC2_R |= 0x12;      				// 1) B E
  delay = SYSCTL_RCGC2_R;      				// 2) no need to unlock
	
  GPIO_PORTE_AMSEL_R &= ~0x07; 				// 3) disable analog function on PE2-0
  GPIO_PORTE_PCTL_R &= ~0x00000FFF; 	// 4) enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x07;   				// 5) inputs on PE2-0
  GPIO_PORTE_AFSEL_R &= ~0x07; 				// 6) regular function on PE2-0
  GPIO_PORTE_DEN_R |= 0x07;    				// 7) enable digital on PE2-0
	
  GPIO_PORTB_AMSEL_R &= ~0x3F; 				// 3) disable analog function on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; 	// 4) enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    				// 5) outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F; 				// 6) regular function on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;    				// 7) enable digital on PB5-0
	
	SYSCTL_RCGC2_R |= 0x00000020;				// 1) activate clock for port F
	delay = SYSCTL_RCGC2_R;							// allow time for clock to start
	GPIO_PORTF_LOCK_R = 0x4C4F434B;			// 2) unlock GPIO port F
	GPIO_PORTF_CR_R = 0x1F;							// allow changes to PF4-0
	GPIO_PORTF_AMSEL_R = 0x00;	 				// 3) disable analog function on port F
  GPIO_PORTF_PCTL_R = 0x00000000; 		// 4) enable regular GPIO
  GPIO_PORTF_DIR_R = 0x0E;	   				// 5) outputs on PF3-1
  GPIO_PORTF_AFSEL_R = 0x00;	 				// 6) disable alternate function on PF7-0
	GPIO_PORTF_PUR_R = 0x11;						// enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R = 0x1F;    				// 7) enable digital on PF4-0

  S = goS;  
  while(1){
    LIGHT = FSM[S].Out;  							// set lights
		
		if(S == walk){
			WALK_LIGHT = 0xE;
			SysTick_Wait10ms(FSM[S].Time);
		}
		else if(S == warning){
			int count = 0;
			
			while(count < 3){
				WALK_LIGHT = 0x02;
				SysTick_Wait10ms(83);
				WALK_LIGHT = 0x0;
				SysTick_Wait10ms(83);
				count++;
			}
		}
		else {
			WALK_LIGHT = 0x02;
			SysTick_Wait10ms(FSM[S].Time);
		}
    
    Input = SENSOR;     							// read sensors
    S = FSM[S].Next[Input];  
  }
}

