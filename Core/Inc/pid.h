// Original fra https://github.com/geekfactory/PID (12/02/24)
// har blitt redigert til eget bruk
#ifndef PID_H
#define PID_H

#include "rovconfig.h"

#define TICK_SECOND 170000000 // 170Mhz

//typedef struct {
//	// Input, output, Error and setpoint
//	double input; //!< Current Process Value
//	double output; //!< Corrective Output from PID Controller
//	double setpoint; //!< Controller Setpoint
//
//	double error;
//	double error_f;
//	// Tuning parameters
//	double Kp; //!< Stores the gain for the Proportional term
//	double Ki; //!< Stores the gain for the Integral term
//	double Kd; //!< Stores the gain for the Derivative term
//	// Variables for PID algorithm
//	double iterm; //!< Accumulator for integral term
//	double lastin; //!< Last input value for differential term
//	// Time related
//	uint32_t lasttime; //!< Stores the time when the control loop ran last time
//
//} pidc_t;

typedef struct {
  double Kp;
  double Ki;
  double Kd;

  double error_f;
  double iterm;

  double out_min;
  double out_max;

  uint32_t lasttime;
} pidc_t;

void pid_init(pidc_t* pid, double kp, double ki, double kd);
void pid_set_k(pidc_t* pid, double kp, double ki, double kd);
void pid_limits(pidc_t* pid, double minimum, double maximum);
double pid_compute_angle(pidc_t* pid, double setpoint, double measurement);
double pid_compute_speed(pidc_t* pid, double setpoint, double measurement);
double pid_compute(pidc_t* pid, double error);
double calc_angle_error(double setpoint, double measurement);
void pid_reset(pidc_t* pid);

#endif
