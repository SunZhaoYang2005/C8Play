/* USER CODE BEGIN Header */
#include "driver_led.h"
#include "driver_lcd.h"
#include "driver_mpu6050.h"
#include "driver_timer.h"
#include "driver_ds18b20.h"
#include "driver_dht11.h"
#include "driver_active_buzzer.h"
#include "driver_color_led.h"
#include "driver_ir_receiver.h"
#include "driver_ir_sender.h"
#include "driver_light_sensor.h"
#include "driver_ir_obstacle.h"
#include "driver_ultrasonic_sr04.h"
#include "driver_spiflash_w25q64.h"
#include "driver_rotary_encoder.h"
#include "driver_motor.h"
#include "driver_key.h"
#include "driver_uart.h"
#include "screen.h"
#include "input_event.h"
#include "app.h"
#include "save.h"    /* 存档轮询：housekeeping 落盘最高分 */
#include "proto.h"   /* 串口协议层：帧编解码 + 发送接口 */
#include "env.h"     /* 环境数据：DHT11 温湿度 + 光敏亮度 */
#include "led_anim.h"/* 全彩 LED 状态动画 */
#include "sound.h"   /* 蜂鸣器音效 */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c.h"   /* hi2c1 声明（I2C1：OLED + MPU6050 共用） */
#include "usart.h" /* huart1 声明 */
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
/* USER CODE BEGIN Variables */
/* 正式任务架构
 *  inputTask        	AboveNormal  	任务内初始化输入外设(含 MPU6050)，随后 10ms 采样所有输入 -> 事件队列
 *  gameTask         	Normal      	33ms 帧驱动：消费事件 + 状态机 + 渲染（App_Init 内任务内初始化 OLED）
 *  housekeepingTask	Low         	100ms 节拍杂务（DHT11/LED/蜂鸣器/光敏/存档；各服务内部自行分频，如 DHT11 2s、全彩 LED 闪烁 500ms）
 *  （原 CubeMX defaultTask 已废弃，重新生成工程时注意删除） */
osThreadId_t inputTaskHandle;
const osThreadAttr_t inputTask_attributes = {
  .name = "inputTask",
  /* 栈 128*4=512B：实测峰值 328B、余量 184B。
   * 历史 512B 曾溢出（静默破坏内存 → 随机卡死/复位），当时代码路径与现版
   * 不同；现以高水位实测为准 + configCHECK_FOR_STACK_OVERFLOW=2 兜底，
   * 若串口出现 [FAULT] STACK OVERFLOW: inputTask 需回调增大 */
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
osThreadId_t gameTaskHandle;
const osThreadAttr_t gameTask_attributes = {
  .name = "gameTask",
  /* 栈 128*4=512B：实测峰值 320B、余量 192B。
   * 深调用链 = 游戏 Tick → 满行消除重画 → Screen_* 局部缓冲，均已被测量覆盖 */
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t housekeepingTaskHandle;
const osThreadAttr_t housekeepingTask_attributes = {
  .name = "housekeepingTask",
  /* 栈 128*4=512B：实测峰值 280B（破纪录存档深链）
   * Save_Poll → W25Q64_Erase/Write → HAL_SPI_TransmitReceive + 回读校验）、
   * 余量 232B；日常杂务峰值 176B。*/
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* I2C1 总线互斥量：OLED 与 MPU6050 共用 I2C1 */
osMutexId_t g_i2cMutex;
/* SPI1 总线互斥量：W25Q64 独占 SPI1，但 gameTask 读存档与
 * housekeeping 写存档可能并发，互斥保护 + 优先级继承防反转 */
osMutexId_t g_spiMutex;
/* USART1 发送互斥量：Proto_Send 会被 protoTask（命令应答）
 * 与 gameTask（0x07 状态上报）并发调用，寄存器级轮询发送跨字节非原子，
 * 互斥保证两任务不会把各自帧的字节交错混发 */
osMutexId_t g_uartTxMutex;
/* 串口协议层任务与队列
 *  protoTask       Normal  USART1 中断逐字节入队 -> 帧解析 -> 命令分发/应答
 *  g_uartRxQueue   32字节队列：ISR(osMessageQueuePut)与任务解耦，字节到达不占任务时间，解析在任务内做 */
osThreadId_t protoTaskHandle;
const osThreadAttr_t protoTask_attributes = {
  .name = "protoTask",
  /* 栈 128*4=512B：实测峰值 224B、余量 288B。
   * 深调用链 = 帧解析 → App_HandleProtoFrame → Save_GetHi(SPI 读链) */
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osMessageQueueId_t g_uartRxQueue;
static uint8_t s_uartRxByte;   /* HAL_UART_Receive_IT 接收缓冲（回调取字节入队） */
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartInputTask(void *argument);
void StartGameTask(void *argument);
void StartHousekeepingTask(void *argument);
void StartProtoTask(void *argument);   

void MX_FREERTOS_Init(void);

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
	g_i2cMutex = osMutexNew(NULL);
	g_spiMutex = osMutexNew(NULL);  
	g_uartTxMutex = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  g_uartRxQueue = osMessageQueueNew(32, 1, NULL); 
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of inputTask */
  inputTaskHandle = osThreadNew(StartInputTask, NULL, &inputTask_attributes);

  /* creation of gameTask */
  gameTaskHandle = osThreadNew(StartGameTask, NULL, &gameTask_attributes);

  /* creation of housekeepingTask */
  housekeepingTaskHandle = osThreadNew(StartHousekeepingTask, NULL, &housekeepingTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  protoTaskHandle = osThreadNew(StartProtoTask, NULL, &protoTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartInputTask */

/**
  * @brief  Function implementing the inputTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartInputTask */
void StartInputTask(void *argument)
{
  /* USER CODE BEGIN StartInputTask */
  /*先初始化轮询外设（编码器/按键/红外），再初始化 MPU6050（内部osDelay让出CPU，I2C 持互斥锁，失败不阻塞）*/
  Input_Init();
  Input_InitMPU6050();

  for (;;)
  {
    Input_Scan();
    osDelay(10);
  }
  /* USER CODE END StartInputTask */
}

/* USER CODE BEGIN Header_StartGameTask */

/**
  * @brief  Function implementing the gameTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartGameTask */
void StartGameTask(void *argument)
{
  /* USER CODE BEGIN StartGameTask */
  /* 应用层初始化，随后 33ms(30fps) 帧驱动 */
  App_Init();

  for (;;)
  {
    App_Process();
    osDelay(33);
  }
  /* USER CODE END StartGameTask */
}

/* USER CODE BEGIN Header_StartHousekeepingTask */

/**
  * @brief  Function implementing the housekeepingTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartHousekeepingTask */
void StartHousekeepingTask(void *argument)
{
  /* USER CODE BEGIN StartHousekeepingTask */
  for (;;)
  {
    /* 最高分存档落盘（有脏标志才执行 擦扇区→写→回读校验，
     * 擦写 ~100ms 的阻塞操作放低优先级任务，绝不进 33ms 游戏帧循环） */
    Save_Poll();
    /* 外设服务（100ms 节拍，各服务内部计数分频）：
     * -Env_Poll：光敏 ADC 采样+滤波+OLED 对比度；DHT11 内部 2s 节拍
     * -LedAnim_Poll：全彩 LED 状态闪烁节拍（500ms 半周期）
     * -Sound_Poll：蜂鸣器 beep 播放（100ms/拍） */
    Env_Poll();
    LedAnim_Poll();
    Sound_Poll();
    osDelay(100);
  }
  /* USER CODE END StartHousekeepingTask */
}

/* USER CODE BEGIN Header_StartProtoTask */

/**
  * @brief  Function implementing the protoTask thread。
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartProtoTask */
void StartProtoTask(void *argument)
{
  /* USER CODE BEGIN StartProtoTask */
  uint8_t byte;
  ProtoFrame frame;

  Proto_Reset();
  HAL_UART_Receive_IT(&huart1, &s_uartRxByte, 1);   /* 启动接收中断 */

  for (;;)
  {
    if (osMessageQueueGet(g_uartRxQueue, &byte, NULL, osWaitForever) == osOK)
    {
      if (Proto_Feed(byte, &frame))
      {
        App_HandleProtoFrame(&frame);
      }
    }
  }
  /* USER CODE END StartProtoTask */
}

/* ----USART1 接收中断回调 ----
 * HAL_UART_Receive_IT 每收 1 字节触发：入队 + 重新启动接收。
 * 队列满（protoTask 未及时消费）时丢字节——协议有 CRC8 + 超时重传兜底，不阻塞 ISR */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    osMessageQueuePut(g_uartRxQueue, &s_uartRxByte, 0, 0);
    HAL_UART_Receive_IT(&huart1, &s_uartRxByte, 1);
  }
}

/* ----协议帧发送（proto.h 声明，通道集成实现） ----
 * 寄存器级轮询发送：与 ISR 的 HAL_UART_Receive_IT 混用同一 huart 句柄时，
 * HAL_UART_Transmit 的 __HAL_LOCK 会与中断接收的锁管理互相踩踏——
 * 发送持锁时 ISR 完成接收会误释放锁，或 Receive_IT 启动返回 HAL_BUSY，
 * 表现为"命令有动作但应答帧丢失"。直接操作 SR/DR 绕过 HAL 锁，发送路径与中断接收路径互不干扰。
 * 发送整体持 g_uartTxMutex——protoTask（应答帧）与 gameTask
 * （0x07 状态上报）可能并发调用，寄存器直写虽单字节原子，但"等 TXE→写 DR"
 * 的序列跨字节交错会把两帧字节穿插，互斥保证整帧串行。 */
uint8_t Proto_Send(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t buf[PROTO_FRAME_MAX];
    uint8_t n = Proto_Pack(buf, cmd, data, len);
    uint8_t i;
    uint8_t ok = 1;

    if (osMutexAcquire(g_uartTxMutex, osWaitForever) != osOK)
    {
        return 0;                       /* 理论不发生（forever 等待），防御性处理 */
    }
    for (i = 0; i < n; i++)
    {
        uint32_t guard = 500000;        /* 超时保护：约 40ms @72MHz，防 TX 引脚故障死等 */
        while ((USART1->SR & USART_SR_TXE) == 0)
        {
            if (--guard == 0)
            {
                ok = 0;                 /* 发送总线异常 */
                break;
            }
        }
        if (!ok)
        {
            break;
        }
        USART1->DR = buf[i];
    }
    if (ok)
    {
        uint32_t guard = 500000;
        while ((USART1->SR & USART_SR_TC) == 0)  /* 等最后一字节移出移位寄存器（发送完成） */
        {
            if (--guard == 0)
            {
                ok = 0;
                break;
            }
        }
    }
    osMutexRelease(g_uartTxMutex);
    return ok;
}

/* ================= 任务内存统计（栈高水位测量） =================
 * 用法：PC 发 0x08 命令（AA 55 00 08 CRC 0D）→ 掌机打印各任务栈用量文本：
 *   [TASK] <name> stack=<分配字节> used=<峰值使用> free=<最小剩余字节>
 * uxTaskGetStackHighWaterMark 返回任务运行以来"最小剩余栈"（单位：字，×4=字节），
 * 即最危险时刻（最深调用链/最多局部变量）剩余的空闲栈——衡量栈余量的权威指标：
 * 峰值使用 = 分配 − free，是"历史最大值"：测量时机越晚（玩遍所有模式/触发过
 * 所有调用链）越接近真实上限，调整建议以此为准。
 * 附注：xPortGetFreeHeapSize = FreeRTOS 堆剩余（8KB 堆里同时装 4 个任务栈、
 * TCB、队列、互斥量——调栈时需保证堆余量仍充足）。 */

static void mem_uart_puts(const char *s)
{
    while (*s)
    {
        uint32_t guard = 200000u;
        while ((USART1->SR & USART_SR_TXE) == 0)
        {
            if (--guard == 0)
            {
                return;
            }
        }
        USART1->DR = (uint8_t)*s++;
    }
}

static void mem_uart_dec(uint32_t v)
{
    char buf[12];
    int i = 11;

    buf[11] = '\0';
    do
    {
        buf[--i] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v);
    mem_uart_puts(&buf[i]);
}

void Mem_DumpTasks(void)
{
    static const char *names[4] = { "inputTask", "gameTask",
                                    "housekeepingTask", "protoTask" };
    /* 与 task_attributes 的 stack_size 保持一致*/
    static const uint32_t alloc[4] = { 128u * 4u, 128u * 4u, 128u * 4u, 128u * 4u };
    osThreadId_t handles[4] = { inputTaskHandle, gameTaskHandle,
                                housekeepingTaskHandle, protoTaskHandle };
    uint32_t i;

    if (osMutexAcquire(g_uartTxMutex, osWaitForever) != osOK)
    {
        return;
    }
    mem_uart_puts("\r\n[TASK-MEM] free heap=");
    mem_uart_dec((uint32_t)xPortGetFreeHeapSize());
    mem_uart_puts("B\r\n");
    for (i = 0; i < 4u; i++)
    {
        uint32_t minFreeWords = (uint32_t)uxTaskGetStackHighWaterMark(handles[i]);
        uint32_t minFree = minFreeWords * 4u;
        uint32_t used = (alloc[i] > minFree) ? (alloc[i] - minFree) : 0u;

        mem_uart_puts("[TASK] ");
        mem_uart_puts(names[i]);
        mem_uart_puts(" stack=");
        mem_uart_dec(alloc[i]);
        mem_uart_puts("B used=");
        mem_uart_dec(used);
        mem_uart_puts("B free=");
        mem_uart_dec(minFree);
        mem_uart_puts("B\r\n");
    }
    osMutexRelease(g_uartTxMutex);
}

/* 栈溢出钩子（configCHECK_FOR_STACK_OVERFLOW=2 时由 FreeRTOS 在任务切换时调用）：
 * 任务栈被踩是最高危故障（静默破坏内存 → 随机卡死/复位），
 * 钩子内任务栈已损坏，只做最简寄存器级输出后死循环，不再碰任何栈/调度 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    __disable_irq();
    mem_uart_puts("\r\n[FAULT] STACK OVERFLOW: ");
    if (pcTaskName != NULL)
    {
        mem_uart_puts(pcTaskName);
    }
    mem_uart_puts("\r\n[FAULT] HALT\r\n");
    for (;;)
    {
    }
}
