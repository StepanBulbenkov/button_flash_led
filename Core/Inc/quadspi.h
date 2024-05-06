/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    quadspi.h
  * @brief   This file contains all the function prototypes for
  *          the quadspi.c file
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
#ifndef __QUADSPI_H__
#define __QUADSPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern QSPI_HandleTypeDef hqspi;

/* USER CODE BEGIN Private defines */

#define cmd_write 0x32
#define cmd_read 0xEB
#define cmd_quad_en 0x35
#define cmd_quad_dis 0xF5
#define cmd_write_vcr 0x81
#define cmd_write_en 0x06
#define cmd_write_dis 0x04
#define cmd_erase_sub4k 0x20

/* USER CODE END Private defines */

void MX_QUADSPI_Init(void);

/* USER CODE BEGIN Prototypes */

void QSPI_WRITE_VCR(QSPI_HandleTypeDef* hqspi, QSPI_CommandTypeDef* cmd, uint8_t* data);
void QSPI_WRITE(QSPI_HandleTypeDef* hqspi, QSPI_CommandTypeDef* cmd, uint32_t addr,uint8_t* data, uint32_t size);
void QSPI_READ(QSPI_HandleTypeDef* hqspi, QSPI_CommandTypeDef* cmd, uint32_t addr, uint8_t* data, uint32_t size);
void QSPI_QUAD_EN(QSPI_HandleTypeDef* hqspi, QSPI_CommandTypeDef* cmd);
void QSPI_QUAD_DIS(QSPI_HandleTypeDef* hqspi, QSPI_CommandTypeDef* cmd);
void QSPI_WRITE_EN(QSPI_HandleTypeDef* hqspi, QSPI_CommandTypeDef* cmd);
void QSPI_WRITE_DIS(QSPI_HandleTypeDef* hqspi, QSPI_CommandTypeDef* cmd);
void QSPI_ERASE_SUB4K(QSPI_HandleTypeDef* hqspi, QSPI_CommandTypeDef* cmd, uint32_t addr);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __QUADSPI_H__ */

