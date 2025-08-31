/*
 * project.c
 *
 *  Created on: Nov 10, 2020
 *      Author: peter
 *
 * This header contains all functions for string processing
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "my_stringlib.h"

// Reverse a string
void reverse_str(char *str, const int length)
{
    int start = 0;
    int end = length - 1;
    char temp;
    while(start < end)
    {
        temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        end--;
        start++;
    }
}
// Convert character to integer
int32_t my_atoi(const char *str)
{
    int32_t result = 0;
    int8_t sign = 1;
    int i = 0;
    if (str[0] == '-') //Sign of number
    {
        sign = -1;
        i++;
    }
    for (; str[i] != '\0'; i++)
    {
        if (!(str[i] >= '0' && str[i] <= '9')) //Is c a number
        {
            return 0;
        }
        result = result * 10 + str[i] - '0';
    }
    return sign * result;
}

// Convert integer (base 10) to character
char *my_itoa(int num, char* str)
{
    uint16_t indx = 0;
    uint16_t rem = 0;
    bool negative = false;

    // Handle 0 case
    if(num == 0)
    {
        str[indx++] = '0';
        str[indx] = '\0';
        return str;
    }

    if (num < 0)
    {
        // Make positive for conversion
        negative = true;
        num = -num;
    }

    while(num != 0 )
    {
        // Obtain remainder
        rem = num - (num/10)*10;
        str[indx++] = rem + '0';
        num = num/10;
    }

    if(negative)
    {
        str[indx++] = '-';
    }

    str[indx] = '\0';

    reverse_str(str, indx);
    return(str);
}

// Compare two strings
bool my_strcmp(char *input, const char str[])
{
    int i = 0;
    while (input[i] != '\0' || str[i] != '\0')
    {
        if (input[i] == str[i])
        {
            i++;
        }
        else
        {
            return false;
        }
    }
    return true;
}


