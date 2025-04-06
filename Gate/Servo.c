/*
Project: Line Following Robot
Servo.c
Runs on TM4C123
Student: Tylor Franca
Class: CECS 497 - Spring 2025
Major: Computer Engineering
School: CSULB
Date: 4/6/2025

Description: Implementation for controlling servo motors with TM4C123
             Uses PWM for servo control
*/

// Servo.c
// Implementation for controlling servo motors with TM4C123
// Uses PWM for servo control

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "Servo.h"
#include "SysTick.h"
#include "PLL.h"

// PWM period for 50 Hz (20ms) which is standard for most servos
// PWM Frequency = System Clock / (PWM_DIV * Load)
// 50 = 16,000,000 / (64 * Load)
// Load = 5000
#define PWM_LOAD 5000

// PWM pulse width values for servo control
// For standard servo motors:
// 0 degrees = 1ms pulse = 5% duty cycle
// 90 degrees = 1.5ms pulse = 7.5% duty cycle
// 180 degrees = 2ms pulse = 10% duty cycle

// Final adjusted values with additional 5 more degrees of turn
#define SERVO_MIN 145     // 0.58ms pulse (0 degrees) - extreme range
#define SERVO_MID 375     // 1.5ms pulse (90 degrees) - kept the same
#define SERVO_MAX 605     // 2.42ms pulse (180 degrees) - extreme range




void Servo_Init(void) {
    // Enable clock to Port F
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    
    // Enable clock to PWM Module 1
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    
    // Wait for the PWM and GPIO to be ready
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){};
    
    // Configure PF2 and PF3 for PWM
    GPIO_PORTF_AFSEL_R |= 0x0C;           // Enable alternate function on PF2 and PF3
    GPIO_PORTF_PCTL_R &= ~0x0000FF00;     // Clear PWM bits for PF2 and PF3
    GPIO_PORTF_PCTL_R |= 0x00005500;      // PWM module 1 for PF2 and PF3
    GPIO_PORTF_DEN_R |= 0x0C;             // Enable digital I/O on PF2 and PF3
    
    // Configure PWM clock to divide by 64
    SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV;
    SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; // Clear PWM divider field
    SYSCTL_RCC_R |= SYSCTL_RCC_PWMDIV_64; // Configure PWM divider to /64
    
    // Configure PWM Module 1 Generator 3 (controls M1PWM6 and M1PWM7)
    PWM1_3_CTL_R = 0;                     // Disable PWM while configuring
    PWM1_3_GENA_R = 0x0000008C;           // M1PWM6 goes high on LOAD, low on CMPA match
    PWM1_3_GENB_R = 0x0000080C;           // M1PWM7 goes high on LOAD, low on CMPB match
    
    // Set PWM period to 20ms (50 Hz)
    PWM1_3_LOAD_R = PWM_LOAD - 1;         // Period = PWM_LOAD
    
    // Set initial duty cycle (90 degrees position)
    PWM1_3_CMPA_R = PWM_LOAD - SERVO_MID;  // Set duty cycle for M1PWM6
    PWM1_3_CMPB_R = PWM_LOAD - SERVO_MID;  // Set duty cycle for M1PWM7
    
    // Enable PWM Generator 3
    PWM1_3_CTL_R |= 0x00000001;           // Enable Generator 3
    
    // Enable PWM outputs
    PWM1_ENABLE_R |= 0xC0;                // Enable M1PWM6 and M1PWM7 outputs
}

void Servo_SetPosition(uint8_t servo, uint8_t position) {
    uint16_t pulse_width;
    
    // Constrain position to 0-180 range
    if (position > 180) position = 180;
    
    // Calculate pulse width based on position (linear interpolation)
    pulse_width = SERVO_MIN + ((SERVO_MAX - SERVO_MIN) * position) / 180;
    
    // Set duty cycle
    if (servo == 0) {
        PWM1_3_CMPA_R = PWM_LOAD - pulse_width;  // Set duty cycle for M1PWM6
    } else {
        PWM1_3_CMPB_R = PWM_LOAD - pulse_width;  // Set duty cycle for M1PWM7
    }
}

void Servo_Center(uint8_t servo) {
    Servo_SetPosition(servo, 90);
}

