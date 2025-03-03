// Line Follower Sensor Test Code 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach


#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "Systick.h"



void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC; // set SysTick timer to use core clock: 50MHz
}

// Time delay using busy wait.
// This assumes 50 MHz system clock.
// Input: 16-bit interger for multiple of 10ms
void delayMs(unsigned long delay){	
	NVIC_ST_RELOAD_R = 50000*delay-1;
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it                                        
  NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // enable SysTick timer
	
	// wait for COUNT bit in control register to be raised.
	while ((NVIC_ST_CTRL_R&NVIC_ST_CTRL_COUNT)==0) {} 
  NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick timer
}

void delayUs(unsigned long delay){	
	NVIC_ST_RELOAD_R = 50*delay-1;
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it                                        
  NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // enable SysTick timer
	
	// wait for COUNT bit in control register to be raised.
	while ((NVIC_ST_CTRL_R&NVIC_ST_CTRL_COUNT)==0) {} 
  NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick timer
}

unsigned long SysTickValue(void) {
    return NVIC_ST_CURRENT_R;
}
