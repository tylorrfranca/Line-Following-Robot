
// Line Follower Sensor Test Code 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach


#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "Sensor.h"
void PortE_Init(void){
  SYSCTL_RCGC2_R |= 0x10;       //Enable clock for Port E
  volatile unsigned long delay = SYSCTL_RCGC2_R;

  GPIO_PORTE_DIR_R   &= ~0x0F;  //PE3..PE0 as inputs
  GPIO_PORTE_AFSEL_R &= ~0x0F;  //Disable alternate function
  GPIO_PORTE_AMSEL_R &= ~0x0F; //Disable analog
  GPIO_PORTE_DEN_R   |= 0x0F; //Enable digital
}

void PortD_Init(void){
  SYSCTL_RCGC2_R |= 0x08;       //Enable clock for Port D
  volatile unsigned long delay = SYSCTL_RCGC2_R;

  GPIO_PORTD_DIR_R   &= ~0x0F;  //PD3..PD0 as inputs
  GPIO_PORTD_AFSEL_R &= ~0x0F;  //Disable alternate function
  GPIO_PORTD_AMSEL_R &= ~0x0F; //Disable analog
  GPIO_PORTD_DEN_R   |= 0x0F; //Enable digital
}

void PortB_Init(void){
  SYSCTL_RCGC2_R |= 0x02;       //Enable clock for Port B
  volatile unsigned long delay = SYSCTL_RCGC2_R;

  GPIO_PORTB_DIR_R   |= 0x13;  //PB0,1,4 as outputs
	GPIO_PORTB_DATA_R  &= ~0x03;  // PB0 and PB1 low 
  GPIO_PORTB_AFSEL_R &= ~0x13;  //Disable alternate function
  GPIO_PORTB_AMSEL_R &= ~0x13; //Disable analog
  GPIO_PORTB_DEN_R   |= 0x13; //Enable digital
}