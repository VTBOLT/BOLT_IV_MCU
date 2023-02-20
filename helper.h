/*
 * helper.h
 *
 *  Created on: Feb 19, 2023
 *      Author: Molly Shear
 *              Mason DiGiorgio
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ti/devices/msp432e4/driverlib/driverlib.h>

#ifndef HELPER_H_
#define HELPER_H_

void convertToASCII(uint8_t* chars, uint8_t digits, int32_t num);

#endif /* HELPER_H_ */
