// Servo.h
// Header file for controlling servo motors with TM4C123
// Uses PWM for servo control

#include <stdint.h>
#include "tm4c123gh6pm.h"

// Initialize PWM for servo motor control
// We will use PWM Module 1 Generator 1 (M1PWM6 and M1PWM7)
// These are on pins PF2 and PF3
void Servo_Init(void);

// Set servo position (0-180 degrees)
// servo: 0 for servo 1, 1 for servo 2
// position: 0-180 degrees
void Servo_SetPosition(uint8_t servo, uint8_t position);

// Move servo to 90 degrees
void Servo_Center(uint8_t servo);

void delaySec(unsigned long delay);