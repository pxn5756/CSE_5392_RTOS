/* 
 * File Name: main.c
 * Author: Peter Nguyen
 * Description: main entry point for program
*/

//#define DEBUG //Uncomment to debug

#include <stdint.h>
#include <stdbool.h>

#include "tm4c123gh6pm.h"
#include "Uart0.h"
#include "wait.h"
#include "shell.h"

//Masked Ports
#define ALL_LEDS (*((volatile uint32_t *)(0x40025000 + 0x0E*4)))

//Bitband Alias
#define RED_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define GREEN_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))


//Mask
// PortF Mask
#define ALL_LED_MASK 14
#define GREEN_LED_MASK 8
#define BLUE_LED_MASK 4
#define RED_LED_MASK 2

void blinkLED()
{
    GREEN_LED = 1;
    waitMicrosecond(500e3);
    GREEN_LED = 0;
}

void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 4, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN
            | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable PORTF
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);
    // Configure Ports
    //Outputs
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | RED_LED_MASK;
    //Inputs
    //Setting Drive Strength
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | RED_LED_MASK; 
    //Digital Enable
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK | RED_LED_MASK;
}

void main(void)
{
    initHw();
    initUart0();
    setUart0ISR();
    putsUart0("\r\n\n======================================================\r\n");
    putsUart0("---------------- CSE 5392 Shell Nano Project --------------\r\n");
    putsUart0("======================================================\r\n");
    blinkLED(); // Visual Indicator on board for program start

    shell();
}
