
// Line Follower Sensor Test Code 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach


#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "LED.h"

void PortF_Init(void){
  SYSCTL_RCGC2_R |= 0x20;       //Enable clock for Port B
  volatile unsigned long delay = SYSCTL_RCGC2_R;

  GPIO_PORTF_DIR_R   |= 0x1F;  //PB0,1,4 as outputs
  GPIO_PORTF_AFSEL_R &= ~0x1F;  //Disable alternate function
  GPIO_PORTF_AMSEL_R &= ~0x1F; //Disable analog
  GPIO_PORTF_DEN_R   |= 0x1F; //Enable digital
}