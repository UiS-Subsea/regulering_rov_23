/*
 * thruster.c
 *
 *  Created on: Mar 9, 2024
 *      Author: halvard
 */

#include "thruster.h"
#include "helpers.h"

#include "moore_penrose_pseudo_inverse.h"

#include <math.h>

static matrix_t B[6][8] = {
  {8.38670567945424050e-01, 8.38670567945424050e-01, 8.38670567945424050e-01, 8.38670567945424050e-01, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00},
  {5.44639035015027084e-01, -5.44639035015027084e-01, 5.44639035015027084e-01, -5.44639035015027084e-01, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00},
  {0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, -1.00000000000000000e+00, -1.00000000000000000e+00, -1.00000000000000000e+00, -1.00000000000000000e+00},
  {2.63181019138996386e-01, -2.63181019138996386e-01, 2.63181019138996386e-01, -2.63181019138996386e-01, 2.72698399999999985e+00, -2.77301600000000015e+00, -2.77301600000000015e+00, 2.72698399999999985e+00},
  {-4.05263230513155770e-01, -4.05263230513155770e-01, -4.05263230513155770e-01, -4.05263230513155770e-01, 3.15000099999999961e+00, 3.15000099999999961e+00, -3.14999900000000022e+00, -3.14999900000000022e+00},
  {3.25910781856666532e+00, -3.29771350215032921e+00, -3.29771241287225925e+00, 3.25910672928859535e+00, 0.00000000000000000e+00, 0.00000000000000000e+00, -0.00000000000000000e+00, 0.00000000000000000e+00},
};
matrix_t Bi[8][6]; // B moore penrose inverse
matrix_t Bt[8][6]; // B transpose
matrix_t Wi[8][8]; // Weights matrix

/*
 * Thruster Control
 * - Init/Deinit thrusters
 * - Set power of a thruster
 * - Test thrusters
 */

void thruster_init(void)
{
  // Setup matricies
  moore_penrose_get_pinv(6, 8, B, Bi);
  matrix_eye(8, Wi);
  matrix_transpose(6, 8, B, Bt);

  // Initialize thruster power to 0
  double zeros[8] = {0.0};
  thruster_set_power_percents(zeros);
  //thruster_disable();

  // Start PWM signal generation
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void thruster_deinit(void)
{
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
}

/*
 * Thruster math calculations
 * - tau calculation
 */

void thruster_calc_tau(double* force, double* tau)
{
  matrix_mul_vec(6,8,B,force,tau);
}

void thruster_calc_force_with_weights(double* tau, double* force, double* weights)
{
  // adjust weights matrix
  for(unsigned i = 0; i < 8; i++)
    Wi[i][i] = 1.0 / weights[i];

  // Bw = Wi*Bt*inv(B*Wi*Bt) => reduced to Bw = WiBt*inv(B*WiBt);

  matrix_t Bw[8][6]; // B inverse with weights
  matrix_t WiBt[8][6]; // temp WiBt
  matrix_t temp66[6][6]; // temp 6x6 matrix
  matrix_mul(8, 8, Wi, 8, 6, Bt, WiBt); // WiBt
  matrix_mul(6, 8, B, 8, 6, WiBt, temp66); // B*WiBt
  moore_penrose_get_pinv(6, 6, temp66, temp66); //inv(B*WiBt)
  matrix_mul(8, 6, WiBt, 6, 6, temp66, Bw); // WiBt*inv(B*WiBt)

  // calculate weighted force
  matrix_mul(8, 6, Bw, 6, 1, tau, force);
}

void thruster_calc_force(double* tau, double* force)
{
  matrix_mul_vec(8, 6, Bi, tau, force);
}

void thruster_calc_force_safe(double* tau, double* force)
{
  const double rescaling_factor = 0.95;
recalc:
  thruster_calc_force(tau, force);// calculate intial force

//  print("initial force = ");
//  matrix_print_vec(8, force);

  double total_power = thruster_map_force_to_power(force);
  if(total_power > THRUSTER_MAX_POWER_TOTAL)
  {
    for(unsigned i = 0; i < 6; i++)
    {
      tau[i] *= rescaling_factor;
    }
    goto recalc;
  }

  //thruster_calc_tau(force, tau);

//  print("initial tau = ");
//  matrix_print_vec(6, tau);

  // check constraints and set weights
  double weights[8];
  bool ok = true;
  for(unsigned i = 0; i < 8; i++)
  {
    double weight = 1;
    const double weight_scaling = 1;
    if(force[i] < 0)
      weight = force[i] / THRUSTER_MIN_NEWTON * weight_scaling;
    else if(force[i] > 0)
      weight = force[i] / THRUSTER_MAX_NEWTON * weight_scaling;
    else // force[i] == 0 => prevent 0 division
      weight = 0.01;

    if(weight > 1) // this means one force is not within constraints
      ok = false;

    weights[i] = weight;
  }

  // if force within constraints, we are done
  if(ok) return;

  // recalculate force with weights
  thruster_calc_force_with_weights(tau, force, weights);

//  print("weight force =  ");
//  matrix_print_vec(8, force);

  // Limit force
  for(unsigned i = 0; i < 8; i++)
  {
    force[i] = clamp(force[i],THRUSTER_MIN_NEWTON, THRUSTER_MAX_NEWTON);
  }


  //Just to make sure
  total_power = thruster_map_force_to_power(force);
  if(total_power > THRUSTER_MAX_POWER_TOTAL)
  {
    for(unsigned i = 0; i < 6; i++)
    {
      tau[i] *= rescaling_factor;
    }
    goto recalc;
  }

//  print("weights = ");
//  matrix_print_vec(8, weights);
}

/*
 * Thruster mappings
 * - Force to percent
 * - Froce to power
 */
static const double k_negative = 51.4849f / 39.9131f;
static const double force_to_power_map[]={9.693489e-02,2.400012e+00,-3.014117e+00};
static const uint8_t force_to_power_map_size = sizeof(force_to_power_map) / sizeof(double);
double thruster_map_force_to_power_single(double force)
{
  bool isneg = force < 0;
  if (isneg)
  {
    force *= -k_negative;
  }

  double power = poly_eval(force, force_to_power_map, force_to_power_map_size);
  return power;
}

double thruster_map_force_to_power(double* forces)
{
  double sum = 0;
  for(unsigned i = 0; i < 8; i++)
  {
    double pwr = thruster_map_force_to_power_single(forces[i]);
    if(pwr > 0)
      sum += pwr;
  }
  return sum;
}

static const double force_to_percent_map[]={1.253369e-08,-1.320440e-06,3.089012e-05,1.205638e-03,-8.287445e-02,3.387172e+00,7.530814e+00};
static const uint8_t force_to_percent_map_size = sizeof(force_to_percent_map) / sizeof(double);
double thruster_map_force_to_percent_single(double force)
{
  if(abs(force) < 0.4f) return 0;

  bool isneg = force < 0;
  if (isneg)
  {
    force *= -k_negative;
  }

  double percent = poly_eval(force, force_to_percent_map, force_to_percent_map_size);
  if (isneg) return -percent;
  return percent;
}

void thruster_map_force_to_percent(double* forces, double* percents)
{
  for(unsigned i = 0; i < 8; i++)
    percents[i] = thruster_map_force_to_percent_single(forces[i]);
}

void thruster_set_dutycycle(Motor motor, double dutycycle)
{
  switch(motor)
  {
  case M1:
    TIM3->CCR4 = (uint32_t)(dutycycle*TIM3->ARR); // PWM 1
    break;
  case M2:
    TIM3->CCR1 = (uint32_t)(dutycycle*TIM3->ARR); // PWM 7
    break;
  case M3:
    TIM2->CCR1 = (uint32_t)(dutycycle*TIM2->ARR); // PWM 5
    break;
  case M4:
    TIM2->CCR3 = (uint32_t)(dutycycle*TIM2->ARR); // PWM 2
    break;
  case M5:
    TIM3->CCR3 = (uint32_t)(dutycycle*TIM3->ARR); // PWM 4
    break;
  case M6:
    TIM2->CCR2 = (uint32_t)(dutycycle*TIM2->ARR); // PWM 6
    break;
  case M7:
    TIM3->CCR2 = (uint32_t)(dutycycle*TIM3->ARR); // PWM 8
    break;
  case M8:
    TIM2->CCR4 = (uint32_t)(dutycycle*TIM2->ARR); // PWM 3
    break;
  default:
    break;
  }
}

// set power of a motor, percent is [-100,100]
void  thruster_set_power_percent_single(Motor motor, double percent)
{
  // limits set in rovconfig.h
  percent = clamp(percent, THRUSTER_MIN_PERCENT, THRUSTER_MAX_PERCENT);

  double dutycycle = 0.75 + 0.002 * percent;
  //double esc_value = dutycycle * 2000.0;

  //print("Motor %i set to %.0fus = %.2f%% dc = %.1f%% percent\r\n", motor, esc_value, dutycycle,percent);
  thruster_set_dutycycle(motor, dutycycle);
}

void thruster_set_power_percents(double* percents)
{
  for(unsigned i = 0; i < 8; i++)
    thruster_set_power_percent_single(i+M1, percents[i]);
}
void  thruster_disable(void)
{
  for(unsigned i = 0; i < 8; i++)
    thruster_set_dutycycle(i+M1, 0.10);
}

void thruster_test_dutycycle(void)
{
  print("Thruster dutycycle test started!\r\n");
  for(unsigned i = 0; i < 8; i++)
  {
    double dc = (1.0+i)/10.0;
    thruster_set_dutycycle(i+M1, dc);
    print("Set thruster %d to %d\%\r\n",i+1,(uint8_t)(dc*100.0));
  }
  Test_Handler();
}

// TESTS
void thruster_test_identify(void)
{
  int mot = 0;
  while(true)
  {
    print("motornr: \r\n");
    scanf("%d",&mot);
    mot -= 1;

    double zero[8] = {0};
    thruster_set_power_percents(zero);
    if (mot >= 0 && mot < 8)
    {
      thruster_set_power_percent_single(mot,12);
    }
  }
}

void thruster_test_thrust(void)
{
    double power = 0;
    double old = 0;
    while(1)
    {
      scanf("%lf",&power);
      if(power == old) continue;

      print("setting power to %.2f\r\n",power);
      for(unsigned i = 0; i < 8; i++)
      {
        Motor mot = i + M1;
        thruster_set_power_percent_single(mot, power);
      }
      old = power;
    }
}

void thruster_test()
{
  for(uint8_t i = 0; i < 10; i++) print("\r\n");

  print("Paadrag Test:\r\n");

  matrix_t tau[6] = {100,-30,0,30,-25,-60};
  matrix_t U[8] = {0};
  matrix_t u[8] = {0};
  print("Testing power calculations\r\n");
  print("  %dN -> %.1fW, expected %.1fW\r\n",35,thruster_map_force_to_power_single(35),204.6f);
  print(" %dN -> %.1fW, expected %.1fW\r\n",-35,thruster_map_force_to_power_single(-35),299.8f);
  print("\r\n");
  print("Testing matrix calculations\r\n");
  print("  tau wanted = ");
  matrix_print_vec(6, tau);

  print("  Calculated values:\r\n");
  thruster_calc_force_safe(tau, U);
  print("  U = ");
  matrix_print_vec(8, U);
  thruster_calc_tau(U, tau);
  print("  result tau = ");
  matrix_print_vec(6, tau);
  print("  pwr = %.1f Watt\r\n",thruster_map_force_to_power(U));

  Test_Handler();
}
