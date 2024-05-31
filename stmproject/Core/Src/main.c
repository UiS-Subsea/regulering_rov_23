/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "retarget.h"
#include "helpers.h"
//#include "ICM20948.h"
#include "fdcan.h"
#include "thruster.h"
#include "pid.h"
#include "vector.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

bool motors_enabled = false;
bool linear_pids_set = false;
bool angular_pids_set = false;

double tau[6] = {0};
double U[8] = {0}; // newtons
double u[8] = {0}; // percent

// pids
pidc_t pid_vx;
pidc_t pid_vy;
pidc_t pid_vz;
pidc_t pid_tx;
pidc_t pid_ty;
pidc_t pid_tz;

// deg
static vec3 rotasjon = {0};
static vec3 rotasjon_last = {0};
static vec3 rotasjon_target = {0};
//  deg/s
static vec3 rotasjonfart = {0};
static vec3 rotasjonfart_target = {0};
// m
static vec3 posisjon = {0};
static vec3 posisjon_last = {0};
static vec3 posisjon_target = {0};
//  m/s
static vec3 fart = {0};
static vec3 fart_target = {0};

// gyro/accel data
//axises gyro = {0};
//axises accel = {0};

#define FIRST_TIME UINT32_MAX

// timekeeping
uint32_t angpos_meas_time = FIRST_TIME;
uint32_t angvel_meas_time = FIRST_TIME;
uint32_t linvel_meas_time = FIRST_TIME;
uint32_t gyro_meas_time = FIRST_TIME;
uint32_t last_controll_time = FIRST_TIME;

// Motor data
StatusByte status_byte = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  CANFD_Init();

  BIT_CLEAR(status_byte, STATUS_THRUSTER_ENABLED);
  BIT_CLEAR(status_byte, STATUS_PIDS_SET);

  RetargetInit(&huart2);

  thruster_init();
  print("- Motors Initialized\r\n");

  //thruster_test_cmd();
  //thruster_test_dutycycle();

  // SPI1 and TIM4CH3 has issues so this does not work atm...
  // TODO: Finne et alternativ som funker, enten bytte til SPI2,
  // Eller bytte fra Tim4 til en annen timer.
  //icm20948_init();

  //thruster_test();
  pid_init(&pid_vx, 0, 0, 0);
  pid_init(&pid_vy, 0, 0, 0);
  pid_init(&pid_vz, 0, 0, 0);
  pid_init(&pid_tx, 0, 0, 0);
  pid_init(&pid_ty, 0, 0, 0);
  pid_init(&pid_tz, 0, 0, 0);

  // limits from thruster_force_get_limits.m
//  pid_limits(&pid_vx, -170.2, 208.9);
//  pid_limits(&pid_vy, -206.8, 175.2);
//  pid_limits(&pid_vz, -199.8, 182.1);
//  pid_limits(&pid_tz, -177.2, 202.0);
//  pid_limits(&pid_tx, -224.5, 266.5);
//  pid_limits(&pid_ty, -249.8, 244.5);


  const double minlim = -120;
  const double maxlim = 120;
  pid_limits(&pid_vx, minlim, maxlim);
  pid_limits(&pid_vy, minlim, maxlim);
  pid_limits(&pid_vz, minlim, maxlim);
  pid_limits(&pid_tz, minlim, maxlim);
  pid_limits(&pid_tx, minlim, maxlim);
  pid_limits(&pid_ty, minlim, maxlim);


  print("- Controllers Initialized\r\n");

  // BLINK on initialized
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
  HAL_Delay(250);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
  HAL_Delay(250);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
  HAL_Delay(250);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);

  //thruster_test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t loop_timestamp = HAL_GetTick();
  while (1)
  {
    HAL_Delay(10);

    uint32_t loop_dt_ms = (HAL_GetTick() - loop_timestamp);
    loop_timestamp = HAL_GetTick();

    double loop_dt_sec = loop_dt_ms/1000.0;

    /* TESTING START */

    //print("%10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %u\r\n", gyr.x, gyr.y, gyr.z, acc.x, acc.y, acc.z, loop_dt_ms);
    //print("%f %f %f %f %f %f %u\r\n",gyr.x,gyr.y,gyr.z,acc.x,acc.y,acc.z,loop_dt_ms);
    //continue;

    //print("UPS: %f.2f\r\n", update_frequency);

    //if(HAL_GetTick() > 2000) continue;
    //icm20948_gyrotest();

    /* TESTING SLUTT */

    // Sending av data over fdcan
    static uint32_t sendthrusterdata_lasttime = 0;
    if(HAL_GetTick() - sendthrusterdata_lasttime > SEND_THRUSTERDATA_INTERVALL)
    {
      sendthrusterdata_lasttime = HAL_GetTick();

      SendThrusterData(u);
      //print("Sendt thrusterdata!\r\n");
    }

    static uint32_t sendstatus_lasttime = 0;
    if(HAL_GetTick() - sendstatus_lasttime > SEND_STATUS_INTERVALL)
    {
      sendstatus_lasttime = HAL_GetTick();
      BIT_EDIT(status_byte, STATUS_THRUSTER_ENABLED, motors_enabled);
      BIT_EDIT(status_byte, STATUS_PIDS_SET, angular_pids_set && linear_pids_set);
      SendStatusData(&status_byte);
      // TODO: Sende status
      //SendStatusData(u);
      //print("Sendt status!\r\n");
    }

    motors_enabled = angular_pids_set && linear_pids_set;
    if (!motors_enabled)
    {
      //print("Motors not enabled, set pids first\r\n");
      continue;
    }


    if(HAL_GetTick() - last_controll_time > MAX_ROV_CONTROL_DEADTIME)
    {
      // To long since last requested control, stop!
      zeromem(u, sizeof(u));
      thruster_set_power_percents(u);
      continue;
    }

    // Update states
    for(unsigned i = 0; i < 3; i++)
    {
      rotasjon_target[i] += rotasjonfart_target[i] * loop_dt_sec;
    }


    // Obtain gyro data

//#ifndef ENABLE_IMU
//    icm20948_accel_read_g(&accel);
//    icm20948_gyro_read_dps(&gyro);
//
//    // extrapolation
//    //TODO: Correct axis to frame
//    rotasjon[0] += gyro.x * loop_dt_sec;
//    rotasjon[1] += gyro.y * loop_dt_sec;
//    rotasjon[2] += gyro.z * loop_dt_sec;
//
//#endif

    // Update Pids
    //print("vx: %.2f, vxt: %.2f\r\n",fart[0],fart_target[0]);
    tau[0] = pid_compute_speed(&pid_vx,fart_target[0],fart[0]);
    tau[1] = pid_compute_speed(&pid_vy,fart_target[1],fart[1]);
    tau[2] = pid_compute_speed(&pid_vz,fart_target[2],fart[2]);
    tau[3] = pid_compute_angle(&pid_tx,rotasjon_target[0],rotasjon[0]);
    tau[4] = pid_compute_angle(&pid_ty,rotasjon_target[1],rotasjon[1]);
    tau[5] = pid_compute_angle(&pid_tz,rotasjon_target[2],rotasjon[2]);


//    print("fart_target = ");
//    matrix_print_vec(3, fart_target);
//    print("fart = ");
//    matrix_print_vec(3, fart);
//    print("tau = ");
//    matrix_print_vec(6, tau);

    // Calculate thruster power in newtons
    thruster_calc_force_safe(tau, U);
//    print("U = ");
//    matrix_print_vec(8, U);

    thruster_map_force_to_percent(U, u);
//    print("u = ");
//    matrix_print_vec(8, u);

    static double u_last[8] = {0};

    const double max_percent_per_second = 10.0; // percent
    const double max_change = max_percent_per_second * loop_dt_sec;
    for(unsigned i = 0; i < 8; i++)
    {
      // check if rapid change in pwm, if not pass
      if(abs(u[i] - u_last[i]) < max_change)
      {
        continue;
      }
      // softstart
      if(u[i] > u_last[i])
      {
        u[i] = u_last[i] + max_change;
      }
      else
      {
        u[i] = u_last[i] - max_change;
      }
    }
    // save
    memcpy(u_last, u, sizeof(u));

    // set power
    thruster_set_power_percents(u);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
float floatdata[9];
void canfd_callback(uint16_t id, void* rxdata)
{
  float* pfloat = rxdata; // assign ptrfloat to same address as rxdata(start of rxbuffer)

  switch(id)
  {
  case REG_KONTROLL:
    // currently bypassed, motors automatically enabled on pids set
    // No need to swap endianess for uint8_t[]
    uint8_t kontroll_byte = ((uint8_t*)rxdata)[4];
    bool enable_motors_requested = kontroll_byte & 0x01;
    if(enable_motors_requested && angular_pids_set && linear_pids_set)
    {
      // only enable motors if pids are set and it is requested
      motors_enabled = true;
    }
    break;
  case PING_RX:
    uint8_t* pingblock = (uint8_t*)rxdata;
    pingblock[0] += 1;
    //print("Ping recieved\r\n");
    SendDataNew(PING_TX, pingblock, sizeof(pingblock));
    break;
  case REG_POSITION_MEASURED:
    swap_endianess(pfloat, 8*sizeof(float), sizeof(float));

    double figure_of_merit = pfloat[6];
    double dt = pfloat[7]; // time since last measurement

    for(unsigned i = 0; i < 3; i++)
    {
      posisjon_last[i] = posisjon[i];
      rotasjon_last[i] = rotasjon[i];

      posisjon[i] = pfloat[i];
      rotasjon[i] = pfloat[i+3];

      if(dt > 0)
      {
        rotasjonfart[i] = (rotasjon[i] - rotasjon_last[i])/dt;
      }
    }
    break;
  case REG_VELOCITY_MEASURED:
    swap_endianess(pfloat, 4*sizeof(float), sizeof(float));

    for(unsigned i = 0; i < 3; i++)
    {
      fart[i] = (double)pfloat[i];
    }
    double distanse_til_bunnen = pfloat[3];
    break;
  case REG_PID_SET_ANGULAR_PARAMS:
    swap_endianess(pfloat, 9*sizeof(float), sizeof(float));

    angular_pids_set = true;
    pid_set_k(&pid_ty, pfloat[0], pfloat[1], pfloat[2]);// x og y er byttet om
    pid_set_k(&pid_tx, pfloat[3], pfloat[4], pfloat[5]);
    pid_set_k(&pid_tz, pfloat[6], pfloat[7], pfloat[8]);
    print("angular: ");
    matrix_print_vecf(9, floatdata);
    break;
  case REG_PID_SET_LINEAR_PARAMS:
    swap_endianess(pfloat, 9*sizeof(float), sizeof(float));

    linear_pids_set = true;
    pid_set_k(&pid_vx, pfloat[0], pfloat[1], pfloat[2]);
    pid_set_k(&pid_vy, pfloat[3], pfloat[4], pfloat[5]);
    pid_set_k(&pid_vz, pfloat[6], pfloat[7], pfloat[8]);
    print("linear: ");
    matrix_print_vecf(9, pfloat);
    break;
  case REG_ANGULAR_VELOCITY_TARGET:
    swap_endianess(pfloat, 3*sizeof(float), sizeof(float));

    last_controll_time = HAL_GetTick();
    for(uint8_t i = 0; i < 3; i++)
    {
      rotasjonfart_target[i] = pfloat[i];
    }
//    print("vinkeltarget: ");
//    matrix_print_vec(3, fart_target);

    break;
  case REG_LINEAR_VELOCITY_TARGET:
    swap_endianess(pfloat, 3*sizeof(float), sizeof(float));

    last_controll_time = HAL_GetTick();
    for(uint8_t i = 0; i < 3; i++)
    {
      fart_target[i] = (double)pfloat[i];
    }
//    print("farttarget: ");
//    matrix_print_vec(3, fart_target);

    break;
  default:
    break;
  }
}

void Test_Handler(void)
{
  print("test ferdig, stopper\r\n");
  while(1){}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  print("ERROR! system crashed\r\n");
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1); // enable led
  __disable_irq();
  thruster_deinit(); // force stop motors
  while (1){}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number, */
     print("Wrong parameters value: file %s on line %d\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
