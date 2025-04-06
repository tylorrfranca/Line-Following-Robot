// IRSensor.c: starter file for CECS346 Lab 7 - Obstacle Avoidance Sensor and Battery Power Supply.
// by Min He
// 4/3/2024

#include <stdint.h> // C99 data types
#include "tm4c123gh6pm.h"
#include <stdbool.h> // provides boolean data type
#include "FrontSensor.h"
// Sensor bit address defenition for PD6
 

// External Function Prototypes (external functions from startup.s)
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // Go to low power mode while waiting for the next interrupt



// Initialize edge trigger interrupt for PD6 both edges
void FrontSensor_Init(void) { 	
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
	//GPIO_PORTD_IEV_R &= ~0x40;																// sensitive on high to low
	GPIO_PORTD_ICR_R |= 0x40;																// clear flag
	GPIO_PORTD_IM_R |= 0x40;																// arm interrupt on PD6
	NVIC_PRI0_R = (NVIC_PRI0_R & 0x1FFFFFFF) | 0x60000000;	// priority 3
	NVIC_EN0_R &= ~0x00000008;																// disable interrupt 0 in NVIC
}

void enableSensor(void){
	NVIC_EN0_R |= 0x00000008;																// enable interrupt 0 in NVIC
}

void DisableSensor(void){
	NVIC_EN0_R &= ~0x00000008;
}
