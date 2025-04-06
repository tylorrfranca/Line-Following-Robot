

#include <stdint.h>
#include "tm4c123gh6pm.h"


void IR_Sensor_Init(void);

uint8_t IR_Obstacle_Detected(void);

void Door_Control_IR(uint32_t delay_ms);

uint32_t Get_IR_Distance(void);