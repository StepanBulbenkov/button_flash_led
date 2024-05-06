/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    quadspi.c
  * @brief   This file provides code for the configuration
  *          of the QUADSPI instances.
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
#include "quadspi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

QSPI_HandleTypeDef hqspi;

/* QUADSPI init function */
void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 1;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_6_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

void HAL_QSPI_MspInit(QSPI_HandleTypeDef* qspiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(qspiHandle->Instance==QUADSPI)
  {
  /* USER CODE BEGIN QUADSPI_MspInit 0 */

  /* USER CODE END QUADSPI_MspInit 0 */
    /* QUADSPI clock enable */
    __HAL_RCC_QSPI_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**QUADSPI GPIO Configuration
    PE2     ------> QUADSPI_BK1_IO2
    PB6     ------> QUADSPI_BK1_NCS
    PB2     ------> QUADSPI_CLK
    PD12     ------> QUADSPI_BK1_IO1
    PD13     ------> QUADSPI_BK1_IO3
    PD11     ------> QUADSPI_BK1_IO0
    */
    GPIO_InitStruct.Pin = QSPI_D2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = QSPI_NCS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
    HAL_GPIO_Init(QSPI_NCS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = QSPI_D1_Pin|QSPI_D3_Pin|QSPI_D0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN QUADSPI_MspInit 1 */

  /* USER CODE END QUADSPI_MspInit 1 */
  }
}

void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef* qspiHandle)
{

  if(qspiHandle->Instance==QUADSPI)
  {
  /* USER CODE BEGIN QUADSPI_MspDeInit 0 */

  /* USER CODE END QUADSPI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_QSPI_CLK_DISABLE();

    /**QUADSPI GPIO Configuration
    PE2     ------> QUADSPI_BK1_IO2
    PB6     ------> QUADSPI_BK1_NCS
    PB2     ------> QUADSPI_CLK
    PD12     ------> QUADSPI_BK1_IO1
    PD13     ------> QUADSPI_BK1_IO3
    PD11     ------> QUADSPI_BK1_IO0
    */
    HAL_GPIO_DeInit(QSPI_D2_GPIO_Port, QSPI_D2_Pin);

    HAL_GPIO_DeInit(GPIOB, QSPI_NCS_Pin|GPIO_PIN_2);

    HAL_GPIO_DeInit(GPIOD, QSPI_D1_Pin|QSPI_D3_Pin|QSPI_D0_Pin);

  /* USER CODE BEGIN QUADSPI_MspDeInit 1 */

  /* USER CODE END QUADSPI_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void QSPI_WRITE_VCR(QSPI_HandleTypeDef* hqspi, QSPI_CommandTypeDef* cmd, uint8_t* data)
{
	cmd->Instruction = cmd_write_vcr;
	cmd->AddressMode = QSPI_ADDRESS_NONE;
	cmd->InstructionMode = QSPI_INSTRUCTION_4_LINES;
	cmd->DataMode = QSPI_DATA_4_LINES;
	cmd->NbData = 1;
	if(HAL_QSPI_Command(hqspi,cmd,10) != HAL_OK) Error_Handler();
	if(HAL_QSPI_Transmit(hqspi,data,10) != HAL_OK) Error_Handler();
	cmd->DummyCycles = *data >> 4;
}

void QSPI_WRITE(QSPI_HandleTypeDef* hqspi, QSPI_CommandTypeDef* cmd, uint32_t addr,uint8_t* data, uint32_t size)
{
	cmd->Instruction = cmd_write;
	cmd->Address = addr;
	cmd->AddressMode = QSPI_ADDRESS_4_LINES;
	cmd->InstructionMode = QSPI_INSTRUCTION_4_LINES;
	cmd->DataMode = QSPI_DATA_4_LINES;
	cmd->NbData = size;
	if(HAL_QSPI_Command(hqspi,cmd,10) != HAL_OK) Error_Handler();
	if(HAL_QSPI_Transmit(hqspi,data,10) != HAL_OK) Error_Handler();
}

void QSPI_READ(QSPI_HandleTypeDef* hqspi, QSPI_CommandTypeDef* cmd, uint32_t addr, uint8_t* data, uint32_t size)
{
	cmd->Instruction = cmd_read;
	cmd->Address = addr;
	cmd->AddressMode = QSPI_ADDRESS_4_LINES;
	cmd->InstructionMode = QSPI_INSTRUCTION_4_LINES;
	cmd->DataMode = QSPI_DATA_4_LINES;
	cmd->NbData = size;
	if(HAL_QSPI_Command(hqspi,cmd,10) != HAL_OK) Error_Handler();
	if(HAL_QSPI_Receive(hqspi,data,10) != HAL_OK) Error_Handler();
}

void QSPI_QUAD_EN(QSPI_HandleTypeDef* hqspi, QSPI_CommandTypeDef* cmd)
{
	cmd->Instruction = cmd_quad_en;
	cmd->AddressMode = QSPI_ADDRESS_NONE;
	cmd->InstructionMode = QSPI_INSTRUCTION_1_LINE;
	cmd->DataMode = QSPI_DATA_NONE;
	if(HAL_QSPI_Command(hqspi,cmd,10) != HAL_OK) Error_Handler();
}

void QSPI_QUAD_DIS(QSPI_HandleTypeDef* hqspi, QSPI_CommandTypeDef* cmd)
{
	cmd->Instruction = cmd_quad_dis;
	cmd->AddressMode = QSPI_ADDRESS_NONE;
	cmd->InstructionMode = QSPI_INSTRUCTION_4_LINES;
	cmd->DataMode = QSPI_DATA_NONE;
	if(HAL_QSPI_Command(hqspi,cmd,10) != HAL_OK) Error_Handler();
}

void QSPI_WRITE_EN(QSPI_HandleTypeDef* hqspi, QSPI_CommandTypeDef* cmd)
{
	cmd->Instruction = cmd_write_en;
	cmd->AddressMode = QSPI_ADDRESS_NONE;
	cmd->InstructionMode = QSPI_INSTRUCTION_4_LINES;
	cmd->DataMode = QSPI_DATA_NONE;
	if(HAL_QSPI_Command(hqspi,cmd,10) != HAL_OK) Error_Handler();
}

void QSPI_WRITE_DIS(QSPI_HandleTypeDef* hqspi, QSPI_CommandTypeDef* cmd)
{
	cmd->Instruction = cmd_write_dis;
	cmd->AddressMode = QSPI_ADDRESS_NONE;
	cmd->InstructionMode = QSPI_INSTRUCTION_4_LINES;
	cmd->DataMode = QSPI_DATA_NONE;
	if(HAL_QSPI_Command(hqspi,cmd,10) != HAL_OK) Error_Handler();
}

void QSPI_ERASE_SUB4K(QSPI_HandleTypeDef* hqspi, QSPI_CommandTypeDef* cmd, uint32_t addr)
{
	cmd->Instruction = cmd_erase_sub4k;
	cmd->Address = addr;
	cmd->AddressMode = QSPI_ADDRESS_4_LINES;
	cmd->InstructionMode = QSPI_INSTRUCTION_4_LINES;
	cmd->DataMode = QSPI_DATA_NONE;
	if(HAL_QSPI_Command(hqspi,cmd,10) != HAL_OK) Error_Handler();
}

/* USER CODE END 1 */
