// Line Follower Sensor Test Code 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach


#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "Systick.h"
#include "LED.h"
#include "Motors.h"
#include "sensor.h"
#include "FrontSensor.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode
void System_Init(void);

struct State {
  int16_t leftMotor;              // left motor speed
  int16_t rightMotor;             // right motor speed
  uint32_t delay;                // Delay in ms
  const struct State *next[8];   // next state
};
typedef const struct State State_t;
bool ObjInRange = false; 

#define Straight &fsm[0]
#define SlightLeft &fsm[1]
#define Left &fsm[2]
#define HardLeft &fsm[3]
#define SlightRight &fsm[4]
#define Right &fsm[5]
#define HardRight &fsm[6]
#define Finish &fsm[7]

#define STOP 1
#define SPEED_05  500
#define SPEED_10 1000
#define SPEED_15 1500
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

#define FRONTSENSOR 		(*((volatile uint32_t *)0x40007100)) //PD6


State_t fsm[15]={
  {SPEED_25, SPEED_25,  10, {HardLeft, Left, SlightLeft, Straight, SlightRight, Right, HardRight, Finish}},  // Straight
	{SPEED_15, SPEED_25,  500, {HardLeft, Left, SlightLeft, Straight, SlightRight, Right, HardRight, Finish}},  // Slight left
	{SPEED_10, SPEED_25,  1000, {HardLeft, Left, SlightLeft, Straight, SlightRight, Right, HardRight, Finish}},  // Left
	{STOP, SPEED_25,  1000, {HardLeft, Left, SlightLeft, Straight, SlightRight, Right, HardRight, Finish}},  // Hard LEFT
	{SPEED_25, SPEED_15,  500, {HardLeft, Left, SlightLeft, Straight, SlightRight, Right, HardRight, Finish}},  // Slight right
  {SPEED_25, SPEED_10,  1000, {HardLeft, Left, SlightLeft, Straight, SlightRight, Right, HardRight, Finish}},  // Right
  {SPEED_25, STOP,  1000, {HardLeft, Left, SlightLeft, Straight, SlightRight, Right, HardRight, Finish}},  // Hard Right
//	{SPEED_05, SPEED_05,  0, {HardLeft, Left, SlightLeft, Straight, SlightRight, Right, HardRight}},  // Lost
  {STOP,     STOP,      1000, {HardLeft, Left, SlightLeft, Straight, SlightRight, Right, HardRight, Finish}},  // Finish

};

State_t *currentState = Straight;   // Initial state

int main(void){
	System_Init();
	while(1){
		if(ObjInRange){
			// Explicitly enter Finish state
			currentState = Finish;

			// Stop the motors explicitly
			Set_L_Speed(currentState->leftMotor);
			Set_R_Speed(currentState->rightMotor);
			Start_Both_Wheels(); // Ensure motors are stopped
			delayUs(currentState->delay);

			// Wait here while object is still in range
			while(ObjInRange){
					WaitForInterrupt(); // Low power mode while waiting
			}

			// Once the object is gone, reset state to Straight
			currentState = Straight;
			}
			// Step 1: Collect Sensor Data
			CollectSensorData();
		
			// Step 2: Get the Position from Sensor Data
			int position = CalculateReflectancePosition();
			
			// Step 3: Update FSM State Based on Sensor Position
			currentState = currentState->next[position];
		

			// Step 4: Set Motor Speed and Direction Based on FSM State
			Set_L_Speed(currentState->leftMotor);
			Set_R_Speed(currentState->rightMotor);
			
			// Step 5: Start the Motors
			Start_Both_Wheels();
			
			// Step 6: Delay for stability
			delayUs(currentState->delay);
	}
}

void GPIOPortD_Handler(void){ 
	for(int i = 0; i < 111000; i ++){};
	if(FRONTSENSOR == 0x40){//out of range
		ObjInRange = false;
	}
	else{
		ObjInRange = true;//in range
	}
	
	if (GPIO_PORTD_RIS_R & 0x40) {		
		GPIO_PORTD_ICR_R |= 0x40;     
	}
}


void System_Init(void){
	unsigned long volatile delay;
	PortB_Init();
	PortD_Init();
	PortE_Init();
	PortF_Init();
  PLL_Init();   // 50 MHz
	Wheels_PWM_Init();
	Dir_Init();
	FrontSensor_Init();
	enableSensor();
  RIGHTDIRECTION = FORWARDRIGHT;
  LEFTDIRECTION = FORWARDLEFT;
	currentState = Straight;
}