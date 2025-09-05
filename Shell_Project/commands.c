/* 
 * File Name: commands.c
 * Author: Peter Nguyen
 * Description: 
 * Contains functions that can be executed by the
 * shell program
*/

#include "Uart0.h"
#include "shell.h"
#include "my_stringlib.h"

#define RED_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))

int RunCommand(USER_DATA *data)
{   
    int error = 0;
    // There is probably a better way to do this. 
    if(isCommand(data, "reboot", 0))
    {
        reboot();
    }
    else if(isCommand(data, "ps", 0))
    {
        ps();
    }
    else if(isCommand(data, "ipcs", 0))
    {
        ipcs();
    }
    else if(isCommand(data, "kill", 1))
    {
        uint32_t pid = getFieldValue(data, 1);
        if(pid != INVALIDARG)
        {
            kill(pid);
        }
        else
        {
            error = INVALIDCMD;
        }
    }
    else if(isCommand(data, "pkill", 1))
    {
        pkill(getFieldString(data, 1));
    }
    else if(isCommand(data, "pi", 1))
    {
        if(my_strcmp("ON", getFieldString(data, 1)))
        {
            pi(true);
        }
        else if (my_strcmp("OFF", getFieldString(data, 1)))
        {
            pi(false);
        }
        else
        {
            error = INVALIDCMD;
        }
    }
    else if(isCommand(data, "preempt", 1))
    {
        if(my_strcmp("ON", getFieldString(data, 1)))
        {
            preempt(true);
        }
        else if (my_strcmp("OFF", getFieldString(data, 1)))
        {
            preempt(false);
        }
        else
        {
            error = INVALIDCMD;
        }
    }
    else if(isCommand(data, "sched", 1))
    {
        if(my_strcmp("PRIO", getFieldString(data, 1)))
        {
            sched(true);
        }
        else if(my_strcmp("RR", getFieldString(data, 1)))
        {
            sched(false);
        }
        else
        {
            error = INVALIDCMD;
        }

    }
    else if(isCommand(data, "pidof", 1))
    {
        pidof(getFieldString(data, 1));
    }
    else if(isCommand(data, "run", 1))
    {
        run(getFieldString(data, 1));
    }
    else
    {
        return INVALIDCMD;
    }

    return error;
}

// TBD: Reboots the microcontroller
void reboot(void)
{
    displayUart0("Rebooting...\r\n");
}

// Displays process thread status
void ps()
{
    displayUart0("PS called\r\n");
}

// Displays the inter-process thread status
void ipcs()
{
    displayUart0("IPCS called\r\n");
}

// Kills the thread with the matching PID
void kill(uint32_t pid)
{
    char temp_str[10];
    displayUart0(my_itoa(pid, temp_str));
    displayUart0(" killed\r\n");
    RED_LED = 0;
}

// Kills thread based on proces name
void pkill(const char name[])
{
    displayUart0(name);
    displayUart0(" killed\r\n");
}

// Turns priority inheritance on or off.
void pi(bool on)
{
    if(on)
    {
        displayUart0("pi on\r\n");
    }
    else
    {
        displayUart0("pi off\r\n");
    }
}

// Turns On or Off preemption
void preempt(bool on)
{
        if(on)
    {
        displayUart0("preempt on\r\n");
    }
    else
    {
        displayUart0("preempt off\r\n");
    }
}

// Selects Priority or Round-Robin Scheduling
void sched(bool prio_on)
{
    if(prio_on)
    {
        displayUart0("sched prio\r\n");
    }
    else
    {
        displayUart0("sched rr\r\n");
    }
}

// Displays PID of the thread
void pidof(const char name[])
{
    displayUart0(name);
    displayUart0(" launched\r\n");
}

// Runs selected program in the background
void run(const char name[])
{
    RED_LED = 1;
    displayUart0("Running task\r\n");
}
