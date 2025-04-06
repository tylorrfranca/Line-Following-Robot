/*
Project: Line Following Robot
Sensor.h
Runs on TM4C123
Student: Tylor Franca
Class: CECS 497 - Spring 2025
Major: Computer Engineering
School: CSULB
Date: 4/6/2025

Description: Header file for IR sensor interface with TM4C123
             Provides functions for obstacle detection and distance measurement
*/

#include <stdint.h>
#include "tm4c123gh6pm.h"


void IR_Sensor_Init(void);

uint8_t IR_Obstacle_Detected(void);

void Door_Control_IR(uint32_t delay_ms);

uint32_t Get_IR_Distance(void);