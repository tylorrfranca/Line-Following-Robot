// Servo.h
// Header file for controlling servo motors with TM4C123
// Uses PWM for servo control

#ifndef SERVO_H
#define SERVO_H

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

// Move servo between 0 and 90 degrees continuously
// servo: 0 for servo 1, 1 for servo 2
// cycles: number of back and forth movements (0 for continuous)
// delay_ms: delay between movements in ms
void Servo_MoveBackForth(uint8_t servo, uint32_t cycles, uint32_t delay_ms);

// Move both servos 90 degrees at the same time
// from_pos: starting position (0-180)
// to_pos: ending position (0-180)
// delay_ms: delay to wait after completing the movement
void Servo_MoveBoth90Degrees(uint8_t from_pos, uint8_t to_pos, uint32_t delay_ms);

// Move servos in opposite directions:
// Servo 1 moves from 90 to 180 degrees (counter-clockwise)
// Servo 2 moves from 90 to 0 degrees (clockwise)
// delay_ms: delay to wait after completing the movement
void Servo_MoveOppositeDirections(uint32_t delay_ms);

// Initialize IR obstacle sensor
// Configures pins for enhanced detection range:
// PE0 - Primary digital input 
// PE1, PE2 - Additional detection inputs (if available)
// PE3 - IR LED control output for active illumination
void IR_Sensor_Init(void);

// Check if an obstacle is detected
// Uses pattern recognition with digital inputs to extend range
// Returns 1 if obstacle is detected, 0 otherwise
uint8_t IR_Obstacle_Detected(void);

// Legacy function maintained for compatibility
// Always returns 0 since we're not using ADC
uint32_t Get_IR_Distance(void);

// Open and close door based on obstacle detection
// Will open door when obstacle is detected and close it when obstacle is removed
// delay_ms: delay before closing the door after obstacle is removed
void Door_Control_IR(uint32_t delay_ms);

// Precise timing delay in microseconds
void Delay_us(uint32_t us);

#endif // SERVO_H 