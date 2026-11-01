/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
/* EXTI15_10 是输入采集核心（对应下方 MX_GPIO_Init 内 EXTI15_10 配置，
 * 2026-10-29 自生成区挪入 USER CODE 区，CubeMX 重新生成后保留）：
 * PB12=编码器 S1 旋转（上沿）→ RotaryEncoder_IRQ_Callback 计数/测速；
 * PB10=红外 NEC（升/降沿） → IRReceiver_IRQ_Callback 记录边沿时刻；
 * PB11=悬空（中断组已使能，但 HAL_GPIO_EXTI_Callback 无对应分支，无外设）。
 * input_task 的 10ms 轮询只"消费"中断产出的计数/时间戳，不替代中断。
 * 历史：韦东山回调曾 mdelay(2) 忙等 → 中断风暴饿死 SysTick/TIM4 时基
 * （黑屏/任务冻结）；现回调已改 3ms 时间戳防抖、零忙等。
 * PB5(GPXTI5)=MPU6050 INT 遗留：EXTI9_5 未使能、无 ISR，
 * MPU6050 已定型为 I2C 轮询（100ms 节拍 + 失败离线退避 3s）。
 * CubeMX 重新生成工程时请保留：PB12=PULLUP、优先级=15。 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB15 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */
/**
  * @brief  PA8 有源蜂鸣器 GPIO 输出初始化
  * @note   有源蜂鸣器低电平驱动：SET=静音 / RESET=响。
  *         GPIO 层统一配置，取代原先"TIM1_CH1(AF_PP) + Sound_Init
  *         二次改配"的顺序依赖方案——只配一次，无初始化顺序风险
  *         （原方案依赖 TIM1 初始化在 Sound_Init 之前，CubeMX 重新生成或
  *         改动调用顺序即失效，见 tim.c HAL_TIM_MspPostInit 注释）。
  *         初始输出高电平（静音），防止上电瞬间误响。
  *         本函数在 main.c 的 USER CODE 区于 MX_GPIO_Init() 后调用；
  *         若 .ioc 中 PA8 已为 GPIO_Output，CubeMX 会在 MX_GPIO_Init 常规区
  *         生成同样配置，重复配置幂等无害。 */
void MX_GPIO_PA8_Buzzer_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   /* 初始静音（高电平） */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE END 2 */
