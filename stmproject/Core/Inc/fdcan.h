/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
// 0x0 til 0x7FF (Max ID mulig) -> 2047 Id-er, (11 bits)
// 0x0 til 0x03F -> 64 Id-er
enum {
  Kommunikasjon_start = 0x000,
  Kommunikasjon_slutt = 0x03F,
  Regulering_start = 0x040,
  Regulering_slutt = 0x07F,
  Sensor_start = 0x080,
  Sensor_slutt = 0x0BF,
  Manipulator_start = 0x0C0,
  Manipulator_slutt = 0x0FF,
  Kraft_start = 0x100,
  Kraft_slutt = 0x13F,
  // For testing
  test_start = 0x200,
  test_slutt = 0x2FF
};


/*Id Liste for verdier som skal sendes ut til CANBussen*/
enum {
  // get
  REG_KONTROLL = Kommunikasjon_start + 0x0,
  REG_PID_SET_ANGULAR_PARAMS = Kommunikasjon_start + 0x1,
  REG_PID_SET_LINEAR_PARAMS = Kommunikasjon_start + 0x2,
  REG_ANGULAR_VELOCITY_TARGET = Kommunikasjon_start + 0x4,
  REG_LINEAR_VELOCITY_TARGET = Kommunikasjon_start + 0x5,

  REG_POSITION_MEASURED = Sensor_start + 0x0,
  REG_VELOCITY_MEASURED = Sensor_start + 0x1,

  PING_RX = Kommunikasjon_start + 0x9,
  PING_TX = Regulering_slutt,
  // set
  REG_SEND_THRUSTER_PAADRAG = Regulering_start + 0x0,
  REG_SEND_STATUS = Regulering_start + 0x1,
};

typedef enum  {
  STATUS_THRUSTER_ENABLED = 0,  // Bit 0
  STATUS_PIDS_SET = 1,  // Bit 1
//  STATUS_POSITION_CONTROL = 2,  // Bit 2
//  STATUS_BIT_3 = (1 << 3),  // Bit 3
//  STATUS_BIT_4 = (1 << 4),  // Bit 4
//  STATUS_BIT_5 = (1 << 5),  // Bit 5
//  STATUS_BIT_6 = (1 << 6),  // Bit 6
//  STATUS_BIT_7 = (1 << 7)   // Bit 7
} StatusByte;

extern StatusByte rov_status;

#define FD_FILTER1_START Kommunikasjon_start
#define FD_FILTER1_SLUTT Kommunikasjon_slutt
#define FD_FILTER2_START Sensor_start
#define FD_FILTER2_SLUTT Sensor_slutt

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */
typedef enum {
    uint8,
    uint16,
    float32
} DataType;

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void CANFD_Init(void);
int SendData(uint16_t adr, void* data, uint8_t size, DataType dataType);
int SendDataNew(uint16_t adr, void* data, uint8_t size);
int hexToValues(void* voidData, const uint8_t* hexData, uint8_t sizeBytes, DataType dataType);

void canfd_callback(uint16_t id, void* rxdata);
void SendThrusterData(double* thrustervalues);
void SendStatusData(void* data);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

