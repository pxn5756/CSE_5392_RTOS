/* 
 * File Name: shell.c
 * Author: Peter Nguyen
 * Description: 
 * Contains utility functions for shell execution
 * including command parsing and validation.
*/

#include "tm4c123gh6pm.h"
#include "Uart0.h"
#include "my_stringlib.h"
#include "shell.h"

//Non-printable characters (Used in getsUart)
#define BCKSPACE 8
#define DEL 127

// Defines used for argument type
#define ALPHA 'a'
#define NUMBER 'n'
#define DELIM 'd'

void parseCommand(USER_DATA *data)
{
    uint8_t arg_count = 0;
    uint8_t i = 0;
    char prev_type = DELIM;
    char c;
    for (i = 0; data->buffer[i] != '\0'; i++)
    {
        c = data->buffer[i];
        if (c >= '0' && c <= '9' && prev_type == DELIM)
        {
            data->Type[arg_count] = NUMBER;
            prev_type = data->Type[arg_count];
            data->Index[arg_count++] = i;
        }
        else if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))
        {
            if(prev_type == DELIM)
            {
                data->Type[arg_count] = ALPHA;
                prev_type = data->Type[arg_count];
                data->Index[arg_count++] = i;
            }
            // Handle Case for given string starts with a number (Not Valid)
            else if(prev_type == NUMBER)
            {
                arg_count = 0;
            }
        }
        else if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')
                || (c >= '0' && c <= '9') || c == '_'))
        {
            prev_type = DELIM;
            data->buffer[i] = '\0';
        }
        if (arg_count > MAX_ARGS)
        {
            arg_count = 0;
            break;
        }
    }
    data->argCount = arg_count;
    #ifdef DEBUG
    for (i = 0; i < data->argCount; i++)
    {
        putcUart0(data->Type[i]);
        putcUart0(',');
        putsUart0(&data->buffer[data->Index[i]]);
        putcUart0('\n');

    }
    #endif
}

// Extracts string type arguments
char* getFieldString(USER_DATA *data, uint8_t fieldNumber)
{
    if (fieldNumber < data->argCount)
    {
        return &(data->buffer[data->Index[fieldNumber]]);
    }
    else
    {
        return '\0';
    }
}

// Extracts and converts numerical arguments
uint32_t getFieldValue(USER_DATA *data, uint8_t fieldNumber)
{
    if (fieldNumber < data->argCount && data->Type[fieldNumber] == NUMBER)
    {
        return my_atoi(&data->buffer[data->Index[fieldNumber]]);
    }
    else
    {
        return INVALIDARG;
    }
}

// Validates Command and Arguments
bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments)
{
    char *command = &data->buffer[data->Index[0]];
    if (minArguments != data->argCount - 1)
    {
        return false;
    }
    return my_strcmp(command, strCommand);
}

// TBD - Hands Over control to Kernal
void yield()
{

}

// Blocking function that obtains string data from console on return
void getsUart0(char *str_Buffer)
{
    uint8_t count = 0;
    char c;
    while (1)
    {
        if(kbhitUart0())
        {
            c = getcUart0();
            if (((c == DEL) || (c == BCKSPACE)) && (count > 0))
            {
                count--;
                displayUart0(&c);
                continue;
            }
            else if (((c == DEL) || (c == BCKSPACE)) && (count == 0))
            {
                continue;
            }
            else
            {
                displayUart0(&c);
                if (c == '\r')
                {
                    str_Buffer[count] = '\0';
                    displayUart0("\r\n");
                    break;
                }
                else if (c >= ' ')
                {
                    str_Buffer[count++] = c;
                    if (count == MAX_CHARS)
                    {
                        str_Buffer[count] = '\0';
                        displayUart0("\r\n");
                        break;
                    }
                }
            }
        }
        yield();
    }
}

void shell(void)
{
    USER_DATA input_stream;
    int error = 0;
    while(1)
    {
        displayUart0("Type in Command: ");
        getsUart0(input_stream.buffer);
        parseCommand(&input_stream);
        error = RunCommand(&input_stream);
        if(error)
        {
            displayUart0("Invalid Command. Please Try again...\r\n");
        }
    }
}
