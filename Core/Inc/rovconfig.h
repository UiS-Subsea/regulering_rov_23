/*
 * rovconfig.h
 *
 *  Created on: Apr 15, 2024
 *      Author: halvard
 */

#ifndef INC_ROVCONFIG_H_
#define INC_ROVCONFIG_H_

#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "matrix.h"

//#define ENABLE_LOGGING
#define ENABLE_FDCAN
//#define ENABLE_IMU

#define SEND_THRUSTERDATA_INTERVALL 160
#define SEND_STATUS_INTERVALL 500

#define MAX_ROV_CONTROL_DEADTIME 1000 // if no control input within 1sec of last, shutdown motors

// Set to 250W per thruster
#define THRUSTER_MAX_NEWTON 40.0
#define THRUSTER_MIN_NEWTON -31.4

#define THRUSTER_MAX_PERCENT 83.0
#define THRUSTER_MIN_PERCENT -83.0

// You are in the dangerzone
#define THRUSTER_MAX_POWER_TOTAL 450

#define BIT_SET(var, bit)   ((var) |= (1 << (bit)))
#define BIT_CLEAR(var, bit) ((var) &= ~(1 << (bit)))
#define BIT_TOGGLE(var, bit) ((var) ^= (1 << (bit)))
#define BIT_EDIT(var, bit, boolean) ((boolean) ? BIT_SET(var, bit) : BIT_CLEAR(var, bit))


#include <stdio.h>
#include <stdarg.h>

void print(const char* format, ...);


#endif /* INC_ROVCONFIG_H_ */
