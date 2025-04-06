/*
Project 2 
Motors.h
Runs on TM4C123
Starter File: CECS 347 Project 2 - Romi Car Test
Project Group Number: 4
Team Members : Ivan Martinez, Anthony Keroles, Nicolas De Greef, Tylor Franca Pires
Date: 10/30/2024
File Description : This header file contains the definitions of variables and functions 
 required to initialize the motors, PLL, PWM, the onboard LED, and 
 onboard switches
 */



// Runs on TM4C123
// Use PWM0/PB6 and PWM1/PB7 to generate pulse-width modulated outputs.
// Daniel Valvano
// March 28, 2014

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
// Modified by Min He, September 7, 2021
// Modified by Tylor Franca, March 6th, 2025

#include <stdint.h>
#include <stdbool.h>


#define LEFTDIRECTION (*((volatile unsigned long *)0x40004300))
#define RIGHTDIRECTION (*((volatile unsigned long *)0x40005030))
	
#define FORWARDLEFT        0x80  // PA7, PA6 HIGH

#define FORWARDRIGHT      0x08  // PB3, PB2 HIGH

#define LIGHT (*((volatile unsigned long *)0x40025038)) // onboard RBG LEDs are used to show car status.
#define RED 0x02
#define GREEN 0x08
#define BLUE 0x04

// duty cycles for different speeds
#define STOP 1
#define SPEED_05  500
#define SPEED_20 2000
#define SPEED_25 2500
#define SPEED_30 3000
#define SPEED_35 3500
#define SPEED_40 4000
#define SPEED_50 5000
#define SPEED_60 6000
#define SPEED_70 7000
#define SPEED_80 8000
#define SPEED_98 9800

#define LED_RED 0x02 //PF1
#define LED_BLUE 0x04 //PF2
#define LED_GREEN 0x08 //PF3
#define LED_ALL 0x0E //PF1 + 2 + 3

#define SW1 0x10 //PF4
#define SW2 0x01 //PF0

extern bool sw1_flag;

void PLL_Init(void);

// Wheel PWM connections: on PB4/M0PWM0:Left wheel, PB5/M0PWM0:Right wheel
void Wheels_PWM_Init(void);

// Start left wheel
void Start_L(void);

// Start right wheel
void Start_R(void);

// Stop left wheel
void Stop_L(void);

// Stop right wheel
void Stop_R(void);

void Start_Both_Wheels(void);

void Stop_Both_Wheels(void);


// Change duty cycle of left wheel: PB5
void Set_L_Speed(uint16_t duty);

// change duty cycle of right wheel: PB4
void Set_R_Speed(uint16_t duty);

// Initialize port E pins PE0-3 for output
// PE0-3 control directions of the two motors: PB7632:L/SLP,L/DIR,R/SLP,R/DIR
// Inputs: None
// Outputs: None
void Dir_Init(void);

void LED_CTL(uint8_t color);

// Initilize port F and arm PF4, PF0 for falling edge interrupts
void SwitchLED_Init(void);  

