/*
 * thruster.h
 *
 *  Created on: Apr 15, 2024
 *      Author: halvard
 */

#ifndef INC_THRUSTER_H_
#define INC_THRUSTER_H_

#include "rovconfig.h"
#include "tim.h"

typedef enum
{
  NONE = -1,
  M1 = 0,
  M2,
  M3,
  M4,
  M5,
  M6,
  M7,
  M8,
  SIZE
} Motor;

/* Thruster Control */
void  thruster_init(void);
void  thruster_deinit(void);
void  thruster_set_dutycycle(Motor motor, double dutycycle);
void  thruster_set_power_percent_single(Motor motor, double percent);
void  thruster_set_power_percents(double* percents);

void  thruster_disable(void);

/* Thruster Mappings */
double thruster_map_force_to_power_single(double force);
double thruster_map_force_to_power(double* forces);
double thruster_map_force_to_percent_single(double force);
void thruster_map_force_to_percent(double* forces, double* percents);

/* Testing */
void thruster_test_dutycycle(void);
void thruster_test_identify(void);
void thruster_test_thrust(void);
void thruster_test();

/* Convertions */
void thruster_calc_tau(double* force, double* tau);
void thruster_calc_force(double* tau, double* force);
void thruster_calc_force_with_weights(double* tau, double* force, double* weights);

void thruster_calc_force_safe(double* tau, double* force);

#endif /* INC_THRUSTER_H_ */
