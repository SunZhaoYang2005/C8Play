/**
  ******************************************************************************
  * @file    env.c
  * @brief   环境数据服务（：
  *          1) DHT11 温湿度：housekeeping 每 2s 采样一次（单总线慢操作放低优先级任务）
  *          2) 光敏采样：ADC1_IN3(PA3) 采样 → 一阶滤波 → 光强百分比（DESKTOP 显示）
  * @note    屏亮度联动已移除（本模组对 0x81/0xD9 亮度调节实测不响应：
  *          0x81 无效果、0xD9 闪烁且效果差），仅保留 ADC 采样 + 滤波 +
  *          数值展示；Screen_SetContrast/SetPrecharge 已从 screen.c 删除
  * 接线     DHT11 数据脚（韦东山库内部配置）；光敏 = PA3(ADC1_IN3，CubeMX 已配模拟输入)
  ******************************************************************************
  */

#include "env.h"
#include "adc.h"                 /* hadc1 */
#include "main.h"
#include "driver_dht11.h"

/* ---------------- 参数 ---------------- */

#define ENV_DHT_INTERVAL     20u  /* DHT11 采样节拍：20 × 100ms = 2s */
#define ENV_LIGHT_FILTER_SHIFT  3u  /* 一阶滤波系数：新值权重 1/8（平滑光敏抖动） */

/* ---------------- 状态 ---------------- */

static int     s_temp = -32768;   /* 温度 x10（INT16_MIN = 无效，显示 "--"） */
static int     s_humi = -32768;   /* 湿度 x10 */
static uint8_t s_light = 0xFFu;   /* 光强 0~100%，0xFF = 未采样 */
static uint16_t s_adcFilter;      /* 光敏 ADC 一阶滤波值（12 位 0~4095） */
static uint8_t  s_dhtTick;        /* DHT11 节拍计数 */
static uint32_t s_pollTick;       /* 诊断：Env_Poll 心跳计数（每 100ms +1，判断 housekeeping 是否存活） */

/* ---------------- 内部函数 ---------------- */

/**
  * @brief  ADC1 单次采样 PA3（软件触发，超时 10ms；失败返回 -1）
  */
static int env_read_light_raw(void)
{
    uint16_t val;

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return -1;
    }
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1);
        return -1;
    }
    val = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return (int)val;
}

/**
  * @brief  光强百分比（0~100%）：板上光敏电路为"越暗 → 分压点电压越高 → ADC 越大"
  *         （黑暗中 raw≈4095），故用 4095-raw 反转得到"环境亮度"：暗→0%，亮→100%
  */
static uint8_t env_light_percent(uint16_t raw)
{
    uint32_t bright = 4095u - raw;      /* 反转：暗处高 ADC = 低亮度 */
    return (uint8_t)((bright * 100u) / 4095u);
}

/* ---------------- 对外接口 ---------------- */

void Env_Init(void)
{
    DHT11_Init();
    s_temp = -32768;
    s_humi = -32768;
    s_light = 0xFFu;
    s_adcFilter = 0;
    s_dhtTick = 0;
    s_pollTick = 0;
}

void Env_Poll(void)
{
    int raw = env_read_light_raw();

    s_pollTick++;   /* 诊断：housekeeping 存活心跳 */

    /* 光敏：每次(100ms)采样 + 一阶滤波 → 光强百分比（仅数值展示，不写 I2C） */
    if (raw >= 0)
    {
        /* 一阶 IIR：new = old + (raw - old)/8，等效权重 7/8 旧 + 1/8 新 */
        s_adcFilter = (uint16_t)(s_adcFilter + ((int)raw - (int)s_adcFilter) / (1 << ENV_LIGHT_FILTER_SHIFT));
        s_light = env_light_percent(s_adcFilter);
    }

    /* DHT11：2s 节拍（20 × 100ms）；采样失败保持旧值，下次重试 */
    if (++s_dhtTick >= ENV_DHT_INTERVAL)
    {
        int hum, temp;
        s_dhtTick = 0;
        if (DHT11_Read(&hum, &temp) == 0)
        {
            s_humi = hum * 10;      /* 韦东山驱动返回整数部分（DHT11 小数位恒 0） */
            s_temp = temp * 10;
        }
    }
}

int Env_GetTemp(void)
{
    return s_temp;
}

int Env_GetHumi(void)
{
    return s_humi;
}

uint8_t Env_GetLight(void)
{
    return s_light;
}

/**
  * @brief  诊断：Env_Poll 心跳计数（每 100ms +1；卡死时停住 = housekeeping 死）
  */
uint32_t Env_GetPollTick(void)
{
    return s_pollTick;
}
