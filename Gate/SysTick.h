/*
Project: Line Following Robot
SysTick.h
Runs on TM4C123
Student: Tylor Franca
Class: CECS 497 - Spring 2025
Major: Computer Engineering
School: CSULB
Date: 4/6/2025

Description: Header file for SysTick timer interface with TM4C123
             Provides delay functions in microseconds, milliseconds, and seconds
*/

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "PLL.h"


// Initialize SysTick timer
void SysTick_Init(void);

// Simple delay function using busy-wait
void delayMs(unsigned long delay);

// Function to delay in microseconds for precise timing
void delayUs(unsigned long delay);

void delaySec(unsigned long delay);