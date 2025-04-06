// Daniel Valvano
// March 28, 2014
// Modified by Min He, September 7, 2021
// Modified by Tylor Franca, March 6, 2025

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
  Program 6.7, section 6.3.2

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
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
#include <stdint.h>
#include <stdbool.h>
#include "Motors.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"

#define NVIC_APINT (*((volatile uint32_t *)0xE000ED0C))
#define PERIOD 10000				// Total PWM period
#define PLL_SYSDIV_50MHZ 7
#define PORTB_MASK 0x02     // Port B Mask value
#define SW1_MASK 0x10
#define SW_MASK 0x11
#define PWM0_1_CTL_ENABLE 0x01
#define SW  (*((volatile unsigned long *)0x40025040)) //Port F Init

#define LED_RED 0x02 //PF1
#define LED_BLUE 0x04 //PF2
#define LED_GREEN 0x08 //PF3
#define LED_ALL 0x0E //PF1 + 2 + 3

#define SW1 0x10 //PF4
#define SW2 0x01 //PF0


bool sw1_flag = false;
uint16_t curr_speed_idx = 0;
uint16_t speeds[] = {STOP, SPEED_35, SPEED_60, SPEED_80, SPEED_98};
#define NUM_OF_SPEEDS		5
	

// Wheel PWM connections: on PB4/M0PWM0:Left wheel, PB5/M0PWM0:Right wheel
void Wheels_PWM_Init(void){
  SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;             // 1) activate PWM0
  SYSCTL_RCGCGPIO_R |= PORTB_MASK;            // 2) activate port B: 000010
  while((SYSCTL_RCGCGPIO_R&0x02) == 0){};
	GPIO_PORTB_AFSEL_R |= 0x30;          
  GPIO_PORTB_PCTL_R &= ~0x00FF0000;     // configure PB4 as PWM0
  GPIO_PORTB_PCTL_R |= 0x00440000;
  GPIO_PORTB_AMSEL_R &= ~0x30;          // disable analog functionality on PB4
  GPIO_PORTB_DEN_R |= 0x30;             // enable digital I/O on PB4
  GPIO_PORTB_DR8R_R |= 0x30;    // enable 8 mA drive on PB4,5
  SYSCTL_RCC_R = SYSCTL_RCC_USEPWMDIV |           // 3) use PWM divider
    (SYSCTL_RCC_R & (~0x001E0000));   //    configure for /2 divider: PWM clock: 50Mhz/2=25MHz
  SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV; // 3) use PWM divider
  SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; //    clear PWM divider field
  SYSCTL_RCC_R |= SYSCTL_RCC_PWMDIV_2;  //    configure for /2 divider

	PWM0_1_CTL_R &= ~PWM0_1_CTL_ENABLE;                     // 4) re-loading down-counting mode
	PWM0_1_GENA_R = PWM_0_GENA_ACTCMPAD_ONE|PWM_0_GENA_ACTLOAD_ZERO;   // PB4: low on LOAD, high on CMPA down
	PWM0_1_GENB_R = (PWM_0_GENB_ACTCMPBD_ONE|PWM_0_GENB_ACTLOAD_ZERO); // PB5: 0xC08: low on LOAD, high on CMPB down
  PWM0_1_LOAD_R = PERIOD - 1;           // 5) cycles needed to count down to 0
  PWM0_1_CTL_R |= PWM0_1_CTL_ENABLE;           // 7) start PWM0
}

// Start left wheel
void Start_L(void) {
  PWM0_ENABLE_R |= 0x04;          // PB4/M0PWM2 (bit 2)
}

// Start right wheel (PB5 = M0PWM3)
void Start_R(void) {
  PWM0_ENABLE_R |= 0x08;          // PB5/M0PWM3 (bit 3)
}

// Stop left wheel (PB4 = M0PWM2)
void Stop_L(void) {
  PWM0_ENABLE_R &= ~0x04;         // PB4/M0PWM2 (bit 2)
}

// Stop right wheel (PB5 = M0PWM3)
void Stop_R(void) {
  PWM0_ENABLE_R &= ~0x08;         // PB5/M0PWM3 (bit 3)
}

void Start_Both_Wheels(void){
	Start_R();
	Start_L();
}

void Stop_Both_Wheels(void) {
	Stop_L();
	Stop_R();
}
// Set duty cycle for Left Wheel: PB4
void Set_L_Speed(uint16_t duty){
  PWM0_1_CMPA_R = duty - 1;             // 6) count value when output rises
}
// Set duty cycle for Right Wheel: PB5
void Set_R_Speed(uint16_t duty){
  PWM0_1_CMPB_R = duty - 1;             // 6) count value when output rises
}

// Initialize port B and A pins PA76 PB32 for output
// PA76 and PB32 control directions of the two motors: PA76:L/SLP,L/DIR  PB32: R/SLP,R/DIR
// Inputs: None
// Outputs: None

void Dir_Init(void){
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB; //activate B clock
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; //activate A clock
	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)!= SYSCTL_RCGC2_GPIOB){} //wait for clk
	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOA)!= SYSCTL_RCGC2_GPIOA){} //wait for clk
		
	GPIO_PORTB_AMSEL_R &= ~0x0C; //disable analog function
	GPIO_PORTB_PCTL_R &= ~0x0000FF00; //GPIO clear bit PCTL
	GPIO_PORTB_DIR_R |= 0x0C; //PB32 output
	GPIO_PORTB_AFSEL_R &= ~0x0C; //no alternate function
	GPIO_PORTB_DEN_R |= 0x0C; //enable digital pins PB32
		
	GPIO_PORTA_AMSEL_R &= ~0xC0; //disable analog function
	GPIO_PORTA_PCTL_R &= ~0xFF000000; //GPIO clear bit PCTL
	GPIO_PORTA_DIR_R |= 0xC0; //PA76 output
	GPIO_PORTA_AFSEL_R &= ~0xC0; //no alternate function
	GPIO_PORTA_DEN_R |= 0xC0; //enable digital pins PA76
}