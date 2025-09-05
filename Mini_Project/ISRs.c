
#include "tm4c123gh6pm.h"
#include "Uart0.h"
#include "my_stringlib.h"


uint32_t pid = 0xDEAD;

void BusFaultISR(void)
{
    char temp_str[10];
    displayUart0("Bus Fault in process ");
    displayUart0(my_itoa(pid, temp_str, BASEDEC));
    displayUart0("\r\n\n");
}

void UsageFaultISR(void)
{
    char temp_str[10];
    displayUart0("Usage Fault in process ");
    displayUart0(my_itoa(pid, temp_str, BASEDEC));
    displayUart0("\r\n\n");
}

void MPUFaultISR(void)
{
    char temp_str[10];
    displayUart0("MPU Fault in process ");
    displayUart0(my_itoa(pid, temp_str, BASEDEC));
    displayUart0("\r\n\n");
}

void HardFaultISR(void)
{
    char temp_str[10];
    displayUart0("Hard Fault in process ");
    displayUart0(my_itoa(pid, temp_str, BASEDEC));
    displayUart0("\r\n\n");  
}

void PendSVISR(void)
{
    char temp_str[10];
    displayUart0("Pendsv in process ");
    displayUart0(my_itoa(pid, temp_str, BASEDEC));
    displayUart0("\r\n\n");  

    if (M)
}