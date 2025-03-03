// Line Follower Sensor Test Code 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach


#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "Systick.h"
#include "LED.h"
#include "sensor.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode
void PortE_Init(void);
void PortD_Init(void);
void PortB_Init(void);
void PortF_Init(void);
void delayUs(unsigned long delay);
void delayMs(unsigned long delay);
void SysTick_Init(void);
void CollectSensorData(void);
volatile unsigned long sensorData[8];


int main(void){
	unsigned long volatile delay;
	PortB_Init();
	PortD_Init();
	PortE_Init();
	PortF_Init();
  PLL_Init();   // 50 MHz

  while(1){
		CollectSensorData(); 
  }
}


void CollectSensorData(void){
					GPIO_PORTF_DATA_R &= ~0xFF; 
		
        // Turn on EVEN sensors (PB0 high, PB1 high)
        GPIO_PORTB_DATA_R |= 0x03;  // PB0 and PB1 = 1 
        
        //Charge capacitors by setting PE3 as output and high
        GPIO_PORTE_DIR_R  |= 0x0F;  // Set PE0-3 as output
				GPIO_PORTD_DIR_R  |= 0x0F;  // Set PD0-3 as output
        GPIO_PORTE_DATA_R |= 0x0F; // Set PE0-3 HIGH
        GPIO_PORTD_DATA_R |= 0x0F; // Set PD0-3 HIGH
        
        //Wait 10 µs
        delayUs(10);
        
        //Set PE0-3 and PD0-3 as input
        GPIO_PORTE_DIR_R &= ~0x0F; // Set PE3 as input
				GPIO_PORTD_DIR_R &= ~0x0F; // Set PE3 as input
	
				delayUs(100);
	
					
				sensorData[0] = (GPIO_PORTE_DATA_R & 0x08) ? 1 : 0;
				sensorData[1] = (GPIO_PORTE_DATA_R & 0x04) ? 1 : 0;
				sensorData[2] = (GPIO_PORTE_DATA_R & 0x02) ? 1 : 0;
				sensorData[3] = (GPIO_PORTE_DATA_R & 0x01) ? 1 : 0;

				sensorData[4] = (GPIO_PORTD_DATA_R & 0x08) ? 1 : 0;
				sensorData[5] = (GPIO_PORTD_DATA_R & 0x04) ? 1 : 0;
				sensorData[6] = (GPIO_PORTD_DATA_R & 0x02) ? 1 : 0;
				sensorData[7] = (GPIO_PORTD_DATA_R & 0x01) ? 1 : 0;

					
					
//				if(sensorData[7]){ 
//						GPIO_PORTB_DATA_R |= 0x10;  // Set PB4 high if PE3 is high
//						GPIO_PORTF_DATA_R = 0x02;
//				}
//				else{
//						GPIO_PORTB_DATA_R &= ~0x10; // Set PB4 low if PE3 is low
//						GPIO_PORTF_DATA_R = 0x08;
//					}
				GPIO_PORTB_DATA_R &= ~0x03;
   			delayUs(1000);
}





