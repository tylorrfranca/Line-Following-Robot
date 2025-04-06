



#include "SysTick.h"
#include "PLL.h"

// Initialize SysTick timer
void SysTick_Init(void) {
    NVIC_ST_CTRL_R = 0;            // disable SysTick during setup
    NVIC_ST_RELOAD_R = 0x00FFFFFF; // maximum reload value
    NVIC_ST_CURRENT_R = 0;         // clear current value
    NVIC_ST_CTRL_R = 0x00000005;   // enable SysTick with core clock
}

// Time delay using busy wait.
// This assumes 50 MHz system clock.
// Input: 16-bit interger for multiple of 10ms
void delayMs(unsigned long delay){	
	NVIC_ST_RELOAD_R = 160000*delay-1;
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it                                        
  NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // enable SysTick timer
	
	// wait for COUNT bit in control register to be raised.
	while ((NVIC_ST_CTRL_R&NVIC_ST_CTRL_COUNT)==0) {} 
  NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick timer
}

void delayUs(unsigned long delay){	
	NVIC_ST_RELOAD_R = 16*delay-1;
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it                                        
  NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // enable SysTick timer
	
	// wait for COUNT bit in control register to be raised.
	while ((NVIC_ST_CTRL_R&NVIC_ST_CTRL_COUNT)==0) {} 
  NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick timer
}
		
// Delay seconds using busy wait.
// Assumes 16 MHz system clock.
void delaySec(unsigned long delay){	
    while(delay > 0){
        NVIC_ST_RELOAD_R = 16000000 - 1;   // 1 sec at 16 MHz = 16,000,000 cycles
        NVIC_ST_CURRENT_R = 0;                   
        NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;

        while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0) {} 

        NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;
        delay--;
    }
}
