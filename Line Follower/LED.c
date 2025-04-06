// Line Follower Sensor Test Code 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach


#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "LED.h"

void PortF_Init(void){
  // Enable clock for Port F
  SYSCTL_RCGC2_R |= 0x20;       // Enable clock for Port F
  volatile unsigned long delay = SYSCTL_RCGC2_R;  // Allow time for clock to stabilize
  delay = SYSCTL_RCGC2_R;       // Extra delay to ensure stability

  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // Unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x01;          // Allow changes to PF0
  
  GPIO_PORTF_DIR_R = 0x01;         // PF0 as output
  GPIO_PORTF_AFSEL_R &= ~0x01;     // Disable alternate function on PF0
  GPIO_PORTF_AMSEL_R &= ~0x01;     // Disable analog on PF0
  GPIO_PORTF_DEN_R |= 0x01;        // Enable digital on PF0
  GPIO_PORTF_DATA_R &= ~0x01;      // Initialize PF0 to 0
}