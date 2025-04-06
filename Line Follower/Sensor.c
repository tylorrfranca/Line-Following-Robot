
// Line Follower Sensor Code 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach


#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "Sensor.h"
#include "Systick.h"

volatile unsigned long sensorData[8];
volatile signed long distance;



int CalculateReflectancePosition(void) {
    int W[8] = {334, 238, 142, 48, -48, -142, -238, -334}; // Position weights
    int numerator = 0;
    int denominator = 0;

    for (int i = 0; i < 8; i++) {
        numerator += sensorData[i] * W[i];  // Sum of (bi * Wi)
        denominator += sensorData[i];       // Sum of bi
    }

    // Avoid division by zero (if no white is detected)
    if (denominator == 0) {
       return 7; 
    }

    distance = (numerator / denominator);  // Compute reflectance position

		if((distance < 47) && (distance > -47)){
			return 3; 
		}
		if((distance < 142) && (distance >= 47)){
			return 4; 
		}
		if((distance < 238) && (distance >= 142)){
			return 5; 
		}
		if((distance < 334) && (distance >= 238)){
			return 6; 
		}
		
		if((distance <= -47) && (distance > -142)){
			return 2; 
		}
		if((distance <= -142) && (distance > -238)){
			return 1; 
		}
		if((distance <= -238) && (distance > -334)){
			return 0; 
		}
		else{
			return 7; 
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
				GPIO_PORTD_DIR_R &= ~0x0F; // Set PD as input
	
				delayUs(200);
	
					
				sensorData[0] = (GPIO_PORTE_DATA_R & 0x08) ? 0 : 1;
				sensorData[1] = (GPIO_PORTE_DATA_R & 0x04) ? 0 : 1;
				sensorData[2] = (GPIO_PORTE_DATA_R & 0x02) ? 0 : 1;
				sensorData[3] = (GPIO_PORTE_DATA_R & 0x01) ? 0 : 1;

				sensorData[4] = (GPIO_PORTD_DATA_R & 0x08) ? 0 : 1;
				sensorData[5] = (GPIO_PORTD_DATA_R & 0x04) ? 0 : 1;
				sensorData[6] = (GPIO_PORTD_DATA_R & 0x02) ? 0 : 1;
				sensorData[7] = (GPIO_PORTD_DATA_R & 0x01) ? 0 : 1;

					
					
				if(sensorData[7]){ 
						GPIO_PORTF_DATA_R = 0x08;
				}
				else{
						GPIO_PORTF_DATA_R = 0x02;
					}
				GPIO_PORTB_DATA_R &= ~0x03;
   			delayUs(1000);
					
				CalculateReflectancePosition();
					
}


void PortE_Init(void){
  SYSCTL_RCGC2_R |= 0x10;       //Enable clock for Port E
  volatile unsigned long delay = SYSCTL_RCGC2_R;

  GPIO_PORTE_DIR_R   &= ~0x0F;  //PE3..PE0 as inputs
  GPIO_PORTE_AFSEL_R &= ~0x0F;  //Disable alternate function
  GPIO_PORTE_AMSEL_R &= ~0x0F; //Disable analog
  GPIO_PORTE_DEN_R   |= 0x0F; //Enable digital
}

void PortD_Init(void){
  SYSCTL_RCGC2_R |= 0x08;       //Enable clock for Port D
  volatile unsigned long delay = SYSCTL_RCGC2_R;

  GPIO_PORTD_DIR_R   &= ~0x0F;  //PD3..PD0 as inputs
	GPIO_PORTD_PCTL_R &= ~0x000000FF;  // Clear PCTL bits for PD0 and PD1
  GPIO_PORTD_AFSEL_R &= ~0x0F;  //Disable alternate function
  GPIO_PORTD_AMSEL_R &= ~0x0F; //Disable analog
  GPIO_PORTD_DEN_R   |= 0x0F; //Enable digital
}

void PortB_Init(void){
  SYSCTL_RCGC2_R |= 0x02;       //Enable clock for Port B
  volatile unsigned long delay = SYSCTL_RCGC2_R;

  GPIO_PORTB_DIR_R   |= 0x03;  //PB0,1 as outputs
	GPIO_PORTB_DATA_R  &= ~0x03;  // PB0 and PB1 low 
  GPIO_PORTB_AFSEL_R &= ~0x03;  //Disable alternate function
  GPIO_PORTB_AMSEL_R &= ~0x03; //Disable analog
  GPIO_PORTB_DEN_R   |= 0x03; //Enable digital
}