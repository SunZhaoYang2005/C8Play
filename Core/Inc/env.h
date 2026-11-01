/**
  ******************************************************************************
  * @file    env.h
  * @brief   环境数据服务：DHT11 温湿度 + 光敏(ADC1_IN3=PA3) 采样
  * 设计     housekeepingTask 每 100ms 节拍调用 Env_Poll()：
  *            - 光敏：ADC1 采样(PA3) → 一阶滤波 → 光强百分比（DESKTOP 显示）
  *            - DHT11：内部计数，每 2s 采样一次（单总线慢操作，绝不进游戏帧）
  * @note    屏亮度联动已移除（本模组对 0x81/0xD9 实测不响应），光敏仅作数值展示
  * 面试点    ADC 采样 + 软件滤波（一阶 IIR）
  ******************************************************************************
  */
#ifndef __ENV_H
#define __ENV_H

#include <stdint.h>

/* 初始化：DHT11 引脚配置（App_Init 中调用，配置快不阻塞） */
void Env_Init(void);

/* housekeeping 每 100ms 调用：光敏采样+滤波；DHT11 2s 节拍采样 */
void Env_Poll(void);

/* 读取缓存：温度（℃×10，如 253=25.3℃）；采样失败返回 INT16_MIN（显示 "--"） */
int Env_GetTemp(void);
int Env_GetHumi(void);

/* 光强 0~100%（12 位 ADC 线性映射）；未采样前返回 0xFF */
uint8_t Env_GetLight(void);

/* 诊断：Env_Poll 心跳计数（每 100ms +1，卡死时停住 = housekeeping 任务死亡） */
uint32_t Env_GetPollTick(void);

#endif /* __ENV_H */
