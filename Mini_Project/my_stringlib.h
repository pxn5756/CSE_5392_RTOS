/*
 * my_stringlib.h
 *
 *  Created on: Nov 10, 2020
 *      Author: peter
 */

#ifndef MY_STRINGLIB_H_
#define MY_STRINGLIB_H_

#include <stdint.h>
#include <stdbool.h>

#define BASEDEC 10
#define BASEHEX 16

int32_t my_atoi(const char *str);
char *my_itoa(const int num, char *str, uint16_t base);
void reverse_str(char *str, const int length);
bool my_strcmp(char *input, const char str[]);


#endif /* MY_STRINGLIB_H_ */
