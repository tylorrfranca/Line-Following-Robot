// ServoExample.c
// Example program demonstrating servo motor control
// with TM4C123 microcontroller

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "Servo.h"

// Define system clock frequency
#define SYSTEM_FREQ 16000000  // 16 MHz

// Simple SysTick-based delay function
void SysTick_Wait(uint32_t delay) {
    NVIC_ST_RELOAD_R = delay - 1;  // number of counts to wait
    NVIC_ST_CURRENT_R = 0;         // clear current
    while((NVIC_ST_CTRL_R & 0x00010000) == 0){} // wait for count flag
}

// Delay in ms using SysTick timer
void SysTick_Delay_ms(uint32_t delay) {
    uint32_t i;
    for(i = 0; i < delay; i++) {
        SysTick_Wait(SYSTEM_FREQ/1000);  // 1ms delay
    }
}

// Initialize SysTick timer
void SysTick_Init(void) {
    NVIC_ST_CTRL_R = 0;            // disable SysTick during setup
    NVIC_ST_RELOAD_R = 0x00FFFFFF; // maximum reload value
    NVIC_ST_CURRENT_R = 0;         // clear current value
    NVIC_ST_CTRL_R = 0x00000005;   // enable SysTick with core clock
}

// Initialize SW1 on PF4
void SW1_Init(void) {
    // Enable clock to Port F (if not already enabled)
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    // Wait for clock to stabilize
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){};
    
    // Configure PF4 (SW1) as input with pull-up
    GPIO_PORTF_DIR_R &= ~0x10;          // Set PF4 as input
    GPIO_PORTF_DEN_R |= 0x10;           // Enable digital function
    GPIO_PORTF_PUR_R |= 0x10;           // Enable pull-up resistor
}

// Check if SW1 is pressed (returns 1 if pressed, 0 if not)
// SW1 is active low (returns 0 when pressed)
uint8_t SW1_Pressed(void) {
    // SW1 is on PF4 and is active low (returns 0 when pressed)
    // So we invert the result to return 1 when pressed
    return (GPIO_PORTF_DATA_R & 0x10) == 0;
}

int main(void) {
    // Always use IR sensor mode
    uint8_t mode = 1;  
    
    // Initialize system
    SysTick_Init();
    
    // Initialize switch for mode selection
    SW1_Init();
    
    // Initialize IR sensor
    IR_Sensor_Init();
    
    // Initialize servo motors
    Servo_Init();
    
    // Center both servos initially (90 degrees)
    Servo_Center(0);
    Servo_Center(1);
    
    // Wait a moment for servos to settle
    SysTick_Delay_ms(1000);
    
    // To manually override to SW1 mode, press and hold SW1 during startup
    if(SW1_Pressed()) {
        mode = 0;  // Manual mode with SW1
        
        // Wait for switch release
        while(SW1_Pressed()) {
            SysTick_Delay_ms(10);
        }
    }
    
    if(mode == 0) {
        // Manual mode - servo control with SW1
        while(1) {
            // Check if SW1 is pressed
            if(SW1_Pressed()) {
                // Move servos in opposite directions for a full 90 degrees:
                // Servo 1: 90 to 180 degrees (counter-clockwise)
                // Servo 2: 90 to 0 degrees (clockwise)
                
                // With calibrated values, these should now move a full 90 degrees
                Servo_SetPosition(0, 180);  // Servo 1 to 180 degrees 
                Servo_SetPosition(1, 0);    // Servo 2 to 0 degrees
                
                // Keep checking if the button is still pressed
                while(SW1_Pressed()) {
                    // Small delay for debounce
                    SysTick_Delay_ms(10);
                }
                
                // When the button is released, return to 90 degrees
                Servo_Center(0);
                Servo_Center(1);
            }
            
            // Small delay to prevent CPU hogging
            SysTick_Delay_ms(10);
        }
    }
    else {
        // IR sensor mode - automatic door control
        // Doorway opens when an obstacle is detected
        // Faster closing after obstacle is gone (200ms delay in function)
        Door_Control_IR(200);
    }
} 