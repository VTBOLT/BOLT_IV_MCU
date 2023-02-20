/*
 * helper.c
 *
 *  Created on: Feb 19, 2023
 *      Author: Molly Shear
 *              Mason DiGiorgio
 */

#include "helper.h"

// Converts a number into a string of ACII chars
void convertToASCII(uint8_t* chars, uint8_t digits, int32_t num) {
  uint8_t STOP = 0;
  if (num < 0) {
    chars[0] = '-';
    STOP = 1;
  }
  num = (uint16_t)abs(num);

  for (; digits > STOP; digits--) {
    chars[digits - 1] = num % 10 + '0';
    num /= 10;
  }
}
