/* 
 * File Name: shell.h
 * Author: Peter Nguyen
 * Description:
 * Header file containing defintions and function declartions
 * for the shell program. This includes command declarations.
*/

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef SHELL_H_
#define SHELL_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_CHARS 80
#define MAX_ARGS 5

// Error Codes
#define INVALIDCMD 0xBADC
#define INVALIDARG 0xBADA

typedef struct USER_DATA
{
    char buffer[MAX_CHARS + 1];
    uint8_t argCount;
    uint8_t Index[MAX_ARGS];
    char Type[MAX_ARGS];
} USER_DATA;

typedef struct
{
    char *name;
    uint8_t argCount;
}COMMAND_DATA;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

//  Shell Routines - Defined in Shell.c
void parseCommand(USER_DATA *data);
char* getFieldString(USER_DATA *data, uint8_t fieldNumber);
uint32_t getFieldValue(USER_DATA *data, uint8_t fieldNumber);
bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments); 
int RunCommand(USER_DATA *data);

void getsUart0(char *str_Buffer);
void yield();
void shell(void);

// Commands - Defined in Commands.c
void reboot(void);
void ps();
void ipcs();
void kill(uint32_t pid);
void pkill(const char name[]);
void pi(bool on);
void preempt(bool on);
void sched(bool prio_on);
void pidof(const char name[]);
void run(const char name[]);

#endif
