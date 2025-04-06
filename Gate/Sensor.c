/*
Project: Line Following Robot
Sensor.c
Runs on TM4C123
Student: Tylor Franca
Class: CECS 497 - Spring 2025
Major: Computer Engineering
School: CSULB
Date: 4/6/2025

Description: Implementation of IR sensor interface with TM4C123
             Provides functions for obstacle detection and distance measurement
*/

#include "Sensor.h"
#include "Servo.h"
#include "SysTick.h"


// Global variable to track IR sensor state (updated in interrupt handler)


// Port D ISR - handles IR sensor state changes

// Initialize edge trigger interrupt for PD6 both edges
void IR_Sensor_Init(void) { 	
	if((SYSCTL_RCGCGPIO_R & SYSCTL_RCGCGPIO_R3) != SYSCTL_RCGCGPIO_R3){
			SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;		// activate clock for Port D
			while((SYSCTL_RCGCGPIO_R & SYSCTL_RCGCGPIO_R3) != SYSCTL_RCGCGPIO_R3){};
	}
	GPIO_PORTD_DIR_R &= ~0x40;															// make PD6 an input
	GPIO_PORTD_DEN_R |= 0x40;																// enable digital I/O on PD6
	GPIO_PORTD_AMSEL_R &= ~0x40;														// disable analog function for PD6
	GPIO_PORTD_PCTL_R &= ~0x0F000000;												// GPIO clear bit PCTL for PD6
	GPIO_PORTD_AFSEL_R &= ~0x40;														// no alternate function for PD6

	// Port D Interrupt
	GPIO_PORTD_IS_R &= ~0x40;																// PD6 is edge sensitive
	GPIO_PORTD_IBE_R |= 0x40;																// PD6 is both edges
	GPIO_PORTD_ICR_R |= 0x40;																// clear flag
	GPIO_PORTD_IM_R |= 0x40;																// arm interrupt on PD6
	NVIC_PRI0_R = (NVIC_PRI0_R & 0x1FFFFFFF) | 0x60000000;	// priority 3
	NVIC_EN0_R |= 0x00000008;																// enable interrupt 0 in NVIC
}
