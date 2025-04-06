// Servo.c
// Implementation for controlling servo motors with TM4C123
// Uses PWM for servo control

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "Servo.h"

// System clock frequency (should match your PLL configuration)
#define SYSFREQ 16000000  // 16 MHz

// PWM clock divider
#define PWM_DIV 64

// PWM period for 50 Hz (20ms) which is standard for most servos
// PWM Frequency = System Clock / (PWM_DIV * Load)
// 50 = 16,000,000 / (64 * Load)
// Load = 5000
#define PWM_LOAD 5000

// PWM pulse width values for servo control
// For standard servo motors:
// 0 degrees = 1ms pulse = 5% duty cycle
// 90 degrees = 1.5ms pulse = 7.5% duty cycle
// 180 degrees = 2ms pulse = 10% duty cycle

// Final adjusted values with additional 5 more degrees of turn
#define SERVO_MIN 145     // 0.58ms pulse (0 degrees) - extreme range
#define SERVO_MID 375     // 1.5ms pulse (90 degrees) - kept the same
#define SERVO_MAX 605     // 2.42ms pulse (180 degrees) - extreme range

// Counter for more reliable IR detection - updated for enhanced sensitivity
#define IR_DETECTION_THRESHOLD 2  // Lowered from 3 to 2 to increase detection range

// Global variable to track IR sensor state (updated in interrupt handler)
volatile uint8_t gIRDetected = 0;

// Port D ISR - handles IR sensor state changes
void GPIOPortD_Handler(void) {
    // Check if interrupt was from PD6
    if(GPIO_PORTD_RIS_R & 0x40) {
        // Clear the interrupt flag
        GPIO_PORTD_ICR_R |= 0x40;
        
        // Update global state based on current sensor reading
        // When pin is LOW (0), an obstacle is detected
        if((GPIO_PORTD_DATA_R & 0x40) == 0) {
            gIRDetected = 1;  // Obstacle detected
        } else {
            gIRDetected = 0;  // No obstacle
        }
    }
}

// Simple delay function using busy-wait
void Delay_ms(uint32_t ms) {
    uint32_t i;
    for (i = 0; i < ms * 16000; i++) {  // Approximate timing, may need calibration
        __asm("nop");
    }
}

// Function to delay in microseconds for precise timing
void Delay_us(uint32_t us) {
    uint32_t i;
    for (i = 0; i < us * 16; i++) {  // Approximate timing
        __asm("nop");
    }
}

void Servo_Init(void) {
    // Enable clock to Port F
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    
    // Enable clock to PWM Module 1
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    
    // Wait for the PWM and GPIO to be ready
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){};
    
    // Configure PF2 and PF3 for PWM
    GPIO_PORTF_AFSEL_R |= 0x0C;           // Enable alternate function on PF2 and PF3
    GPIO_PORTF_PCTL_R &= ~0x0000FF00;     // Clear PWM bits for PF2 and PF3
    GPIO_PORTF_PCTL_R |= 0x00005500;      // PWM module 1 for PF2 and PF3
    GPIO_PORTF_DEN_R |= 0x0C;             // Enable digital I/O on PF2 and PF3
    
    // Configure PWM clock to divide by 64
    SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV;
    SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; // Clear PWM divider field
    SYSCTL_RCC_R |= SYSCTL_RCC_PWMDIV_64; // Configure PWM divider to /64
    
    // Configure PWM Module 1 Generator 3 (controls M1PWM6 and M1PWM7)
    PWM1_3_CTL_R = 0;                     // Disable PWM while configuring
    PWM1_3_GENA_R = 0x0000008C;           // M1PWM6 goes high on LOAD, low on CMPA match
    PWM1_3_GENB_R = 0x0000080C;           // M1PWM7 goes high on LOAD, low on CMPB match
    
    // Set PWM period to 20ms (50 Hz)
    PWM1_3_LOAD_R = PWM_LOAD - 1;         // Period = PWM_LOAD
    
    // Set initial duty cycle (90 degrees position)
    PWM1_3_CMPA_R = PWM_LOAD - SERVO_MID;  // Set duty cycle for M1PWM6
    PWM1_3_CMPB_R = PWM_LOAD - SERVO_MID;  // Set duty cycle for M1PWM7
    
    // Enable PWM Generator 3
    PWM1_3_CTL_R |= 0x00000001;           // Enable Generator 3
    
    // Enable PWM outputs
    PWM1_ENABLE_R |= 0xC0;                // Enable M1PWM6 and M1PWM7 outputs
}

void Servo_SetPosition(uint8_t servo, uint8_t position) {
    uint16_t pulse_width;
    
    // Constrain position to 0-180 range
    if (position > 180) position = 180;
    
    // Calculate pulse width based on position (linear interpolation)
    pulse_width = SERVO_MIN + ((SERVO_MAX - SERVO_MIN) * position) / 180;
    
    // Set duty cycle
    if (servo == 0) {
        PWM1_3_CMPA_R = PWM_LOAD - pulse_width;  // Set duty cycle for M1PWM6
    } else {
        PWM1_3_CMPB_R = PWM_LOAD - pulse_width;  // Set duty cycle for M1PWM7
    }
}

void Servo_Center(uint8_t servo) {
    Servo_SetPosition(servo, 90);
}

void Servo_MoveBackForth(uint8_t servo, uint32_t cycles, uint32_t delay_ms) {
    uint32_t i = 0;
    
    // If cycles is 0, run continuously until the function is called again
    if (cycles == 0) {
        while (1) {
            Servo_SetPosition(servo, 0);
            Delay_ms(delay_ms);
            Servo_SetPosition(servo, 90);
            Delay_ms(delay_ms);
        }
    } else {
        for (i = 0; i < cycles; i++) {
            Servo_SetPosition(servo, 0);
            Delay_ms(delay_ms);
            Servo_SetPosition(servo, 90);
            Delay_ms(delay_ms);
        }
    }
}

void Servo_MoveBoth90Degrees(uint8_t from_pos, uint8_t to_pos, uint32_t delay_ms) {
    // Set both servos to the starting position
    Servo_SetPosition(0, from_pos);
    Servo_SetPosition(1, from_pos);
    
    // Ensure the from position is registered before moving
    Delay_ms(100);
    
    // Move both servos to the target position simultaneously
    Servo_SetPosition(0, to_pos);
    Servo_SetPosition(1, to_pos);
    
    // Wait for the specified delay time
    Delay_ms(delay_ms);
}

void Servo_MoveOppositeDirections(uint32_t delay_ms) {
    // First center both servos at 90 degrees
    Servo_SetPosition(0, 90);
    Servo_SetPosition(1, 90);
    
    // Allow servos to reach the center position
    Delay_ms(100);
    
    // Move Servo 1 counter-clockwise to 180 degrees and 
    // Servo 2 clockwise to 0 degrees simultaneously
    Servo_SetPosition(0, 180);  // Counter-clockwise
    Servo_SetPosition(1, 0);    // Clockwise
    
    // Wait for the specified delay time
    Delay_ms(delay_ms);
    
    // Return both servos to 90 degrees
    Servo_SetPosition(0, 90);
    Servo_SetPosition(1, 90);
    
    // Wait for the servos to return to center
    Delay_ms(delay_ms);
}

// Initialize edge trigger interrupt for PD6 both edges
void IR_Sensor_Init(void) { 	
	// Activate clock for Port D if not already active
	if((SYSCTL_RCGCGPIO_R & SYSCTL_RCGCGPIO_R3) != SYSCTL_RCGCGPIO_R3){
		SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;		// activate clock for Port D
		while((SYSCTL_RCGCGPIO_R & SYSCTL_RCGCGPIO_R3) != SYSCTL_RCGCGPIO_R3){};
	}
    
	// Configure PD6 as digital input for IR sensor
	GPIO_PORTD_DIR_R &= ~0x40;         // make PD6 an input (bit 6 = 0)
	GPIO_PORTD_DEN_R |= 0x40;          // enable digital I/O on PD6
	GPIO_PORTD_AMSEL_R &= ~0x40;       // disable analog function for PD6
	GPIO_PORTD_PCTL_R &= ~0x0F000000;  // GPIO clear bit PCTL for PD6
	GPIO_PORTD_AFSEL_R &= ~0x40;       // no alternate function for PD6
    
    // Enable pull-up resistor for PD6 (active low sensor)
    GPIO_PORTD_PUR_R |= 0x40;         // enable pull-up on PD6
	
	// Configure Port D Interrupt for edge detection
	GPIO_PORTD_IS_R &= ~0x40;          // PD6 is edge sensitive
	GPIO_PORTD_IBE_R |= 0x40;          // PD6 is both edges
	GPIO_PORTD_ICR_R |= 0x40;          // clear interrupt flag
	GPIO_PORTD_IM_R |= 0x40;           // arm interrupt on PD6
    
	// Configure interrupt priority
	NVIC_PRI0_R = (NVIC_PRI0_R & 0x1FFFFFFF) | 0x60000000;  // priority 3
    
    // Enable interrupt 3 in NVIC (Port D is interrupt 3)
	NVIC_EN0_R |= 0x00000008;          // enable interrupt 3 in NVIC
    
    // Allow time for sensor to stabilize
    Delay_ms(50);
}

uint8_t IR_Obstacle_Detected(void) {
    // Read from PD6 instead of PE0 for IR sensor input
    // The sensor outputs LOW (0) when an obstacle is detected
    
    // Read the sensor pin - PD6 (bit 6)
    uint8_t sensorValue = (GPIO_PORTD_DATA_R & 0x40) >> 6;
    
    // Return true (1) if obstacle detected (pin is LOW)
    // Return false (0) if no obstacle (pin is HIGH)
    return (sensorValue == 0);
}

void Door_Control_IR(uint32_t delay_ms) {
    uint8_t doorState = 0;  // 0 = closed, 1 = open
    uint8_t obstacleDetected;
    uint8_t consecutiveDetections = 0;
    uint8_t consecutiveNonDetections = 0;
    
    // Initial state - door closed (servos at center position)
    Servo_Center(0);
    Servo_Center(1);
    
    // Allow system to stabilize
    Delay_ms(100);
    
    // Reset global IR detection state
    gIRDetected = IR_Obstacle_Detected();
    
    while(1) {
        // Get current obstacle state
        obstacleDetected = gIRDetected;  // Use the global variable updated by interrupt
        
        // Manual polling as a backup - ensures we don't miss state changes
        obstacleDetected |= IR_Obstacle_Detected();
        
        // Detection counting with software debouncing
        if(obstacleDetected) {
            if(consecutiveDetections < 100) {
                consecutiveDetections++;
            }
            consecutiveNonDetections = 0;
        } else {
            consecutiveDetections = 0;
            if(consecutiveNonDetections < 100) {
                consecutiveNonDetections++;
            }
        }
        
        // Door control state machine
        if(doorState == 0) {  // Door is closed
            // Open door when obstacle detected consistently
            // Higher threshold (5) for more reliable operation with interrupt-driven approach
            if(consecutiveDetections >= 5) {
                // Add 0.2 second delay before opening the gate
                Delay_ms(200);
                
                // Open the door
                doorState = 1;
                
                // Move servos to open position
                Servo_SetPosition(0, 180);  // Servo 1 to 180 degrees
                Servo_SetPosition(1, 0);    // Servo 2 to 0 degrees
                
                // Reset counters
                consecutiveDetections = 0;
                consecutiveNonDetections = 0;
                
                // Small delay to let servos move
                Delay_ms(200);
            }
        } else {  // Door is open
            // Close door after consistent non-detection
            // Using higher threshold (15) for stability
            if(consecutiveNonDetections >= 15) {
                // Double-check to avoid premature closing
                if(!IR_Obstacle_Detected()) {
                    // Add 1 second delay before closing the door
                    Delay_ms(1000);
                    
                    // Close the door
                    Servo_Center(0);
                    Servo_Center(1);
                    doorState = 0;
                    
                    // Reset counters
                    consecutiveDetections = 0;
                    consecutiveNonDetections = 0;
                    
                    // Small delay to let servos move
                    Delay_ms(200);
                }
            }
        }
        
        // Longer polling interval is okay with interrupt-driven approach
        Delay_ms(15);
    }
}

uint32_t Get_IR_Distance(void) {
    // Dummy function that returns 0 since we're not using ADC
    // Kept for compatibility with existing code that might call this function
    return 0;
} 