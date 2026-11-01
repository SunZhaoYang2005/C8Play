/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ================= 故障诊断=================
 * HardFault / BusFault / MemManage / UsageFault 统一 dump 到 USART1：
 *   1. 寄存器级直写 USART1->SR/DR——故障时 FreeRTOS/中断状态不可信，
 *      不依赖 HAL 锁、队列、互斥量或任务调度，纯裸寄存器轮询；
 *   2. 提取异常栈帧（R0-R3 / R12 / LR / PC / xPSR，Cortex-M3 进入
 *      异常时硬件自动压栈）：LR 的 bit2 指示异常前用的是 MSP 还是 PSP，
 *      据此取栈帧指针，栈帧偏移固定（[0]=R0 ...[6]=PC [7]=xPSR）；
 *   3. 用法：串口 115200 收 "[FAULT] HardFault" + PC/LR 等寄存器，
 *      PC 对照 Keil 生成的 .map 文件即可定位到具体函数。
 * 超时保护：每字符最多轮询 20 万次（约 20ms @72MHz），TX 引脚异常不死等。 */

static void fault_uart_putc(char c)
{
    uint32_t guard = 200000u;

    while ((USART1->SR & USART_SR_TXE) == 0)
    {
        if (--guard == 0)
        {
            return;
        }
    }
    USART1->DR = (uint8_t)c;
}

static void fault_uart_str(const char *s)
{
    while (*s)
    {
        fault_uart_putc(*s++);
    }
}

static void fault_uart_hex32(uint32_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    char buf[12];
    int i = 11;

    buf[11] = '\0';
    buf[--i] = 'x';
    buf[--i] = '0';
    do
    {
        buf[--i] = hex[v & 0x0Fu];
        v >>= 4;
    } while (v);
    fault_uart_str(&buf[i]);
}

/* 通用故障 dump：name 为故障名（如 "HardFault"），frame 为异常栈帧指针 */
static void fault_dump(const char *name, uint32_t *frame)
{
    fault_uart_str("\r\n[FAULT] ");
    fault_uart_str(name);
    fault_uart_str("\r\nPC=");
    fault_uart_hex32(frame[6]);      /* 故障指令地址：对照 .map 定位函数 */
    fault_uart_str(" LR=");
    fault_uart_hex32(frame[5]);      /* 返回地址（调用处，可看调用链上一级） */
    fault_uart_str(" xPSR=");
    fault_uart_hex32(frame[7]);
    fault_uart_str(" R0=");
    fault_uart_hex32(frame[0]);
    fault_uart_str(" R1=");
    fault_uart_hex32(frame[1]);
    fault_uart_str(" R2=");
    fault_uart_hex32(frame[2]);
    fault_uart_str(" R3=");
    fault_uart_hex32(frame[3]);
    fault_uart_str(" R12=");
    fault_uart_hex32(frame[4]);
    fault_uart_str("\r\n[FAULT] HALT\r\n");
}

/* 读取进入异常前的 LR（本 CMSIS 版本未导出 __get_LR；
 * ARMCC 寄存器变量语法：r14 = LR，AC5 兼容） */
static uint32_t fault_get_lr(void)
{
    register uint32_t lr __ASM("r14");
    return lr;
}

/* 根据 LR.bit2 选择异常前使用的堆栈指针，返回异常栈帧基址 */
static uint32_t *fault_get_frame(void)
{
    uint32_t lr = fault_get_lr();
    return (uint32_t *)((lr & 0x4u) ? __get_PSP() : __get_MSP());
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  __disable_irq();              /* 故障现场已损坏：关中断后不再依赖任何调度/锁 */
  fault_dump("HardFault", fault_get_frame());
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  __disable_irq();
  fault_dump("MemManage", fault_get_frame());
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  __disable_irq();
  fault_dump("BusFault", fault_get_frame());
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  __disable_irq();
  fault_dump("UsageFault", fault_get_frame());
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  /* 输入采集核心：PB12=编码器旋转、PB10=红外 NEC（PB11 悬空、无外设）。
   * 链路：本 ISR → HAL_GPIO_EXTI_IRQHandler → HAL_GPIO_EXTI_Callback
   * (driver_irq.c) → RotaryEncoder_IRQ_Callback / IRReceiver_IRQ_Callback；
   * 回调内零忙等（3ms 时间戳防抖），input_task 10ms 轮询消费计数/时刻。 */
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.（串口协议层接收）
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
