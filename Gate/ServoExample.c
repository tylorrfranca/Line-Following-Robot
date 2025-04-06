/*
Project: Line Following Robot
ServoExample.c
Runs on TM4C123
Student: Tylor Franca
Class: CECS 497 - Spring 2025
Major: Computer Engineering
School: CSULB
Date: 4/6/2025

Description: Example program demonstrating servo motor control
             with TM4C123 microcontroller
*/

// ServoExample.c
// Example program demonstrating servo motor control
// with TM4C123 microcontroller

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode


#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "Servo.h"
#include "SysTick.h"
#include "Sensor.h"
#include "PLL.h"

bool doorOpen = false;
bool ObjectDetected = false;


int main(void) {
	DisableInterrupts();
	PLL_Init();
	SysTick_Init();
	IR_Sensor_Init();
	Servo_Init();
	EnableInterrupts();
  
	Servo_Center(0);
	Servo_Center(1);
  
	while(1){
		
		if(ObjectDetected){
				delaySec(1);
				Servo_SetPosition(0, 180);  // Servo 1 to 180 degrees
				Servo_SetPosition(1, 0);  // Servo 2 to 0 degrees
				delaySec(1);
		}
		
		if(!ObjectDetected){
			delaySec(2);
			Servo_Center(0);
			Servo_Center(1);
		}
	}
} 




void GPIOPortD_Handler(void) {
    // Check if interrupt was from PD6
    if(GPIO_PORTD_RIS_R & 0x40) {
        // Clear the interrupt flag
        GPIO_PORTD_ICR_R |= 0x40;
        
        // Update global state based on current sensor reading
        // When pin is LOW (0), an obstacle is detected
        if((GPIO_PORTD_DATA_R & 0x40) == 0) {
            ObjectDetected = true;  // Obstacle detected
        } else {
            ObjectDetected = false;  // No obstacle
        }
    }
}