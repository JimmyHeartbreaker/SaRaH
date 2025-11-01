#ifndef ARDUINO_EXT_H
#define ARDUINO_EXT_H

#include <stdio.h>
#include <stdarg.h>
#include <cmath>

#define printf(...) serialPrintf(__VA_ARGS__)



void serialPrintf(const char *format, ...);
void HAL_Error_Handler();

#endif