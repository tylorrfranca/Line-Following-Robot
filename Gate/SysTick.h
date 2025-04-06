



#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "PLL.h"


// Initialize SysTick timer
void SysTick_Init(void);

// Simple delay function using busy-wait
void delayMs(unsigned long delay);

// Function to delay in microseconds for precise timing
void delayUs(unsigned long delay);

void delaySec(unsigned long delay);