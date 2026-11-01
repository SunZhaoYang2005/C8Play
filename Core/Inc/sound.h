/**
  ******************************************************************************
  * @file    sound.h
  * @brief   蜂鸣器音效，异步 beep 播放（有源蜂鸣器 PA8，低电平驱动）。
  *          音量 = 响的时长比例（时间调制，蜂鸣器始终满压，不欠压停振）
  * 设计     app.c 事件处调 Sound_Beep() 只登记播放计划，不阻塞；
  *          housekeepingTask 每 100ms 调 Sound_Poll() 按 tick 推进 on/off 拍
  ******************************************************************************
  */
#ifndef __SOUND_H
#define __SOUND_H

#include <stdint.h>

/* 初始化有源蜂鸣器（App_Init 调用；GPIO 配置快不阻塞） */
void Sound_Init(void);

/* 设置音量 1~100%（默认 50%）：on 拍响的时长按比例压缩，
 * 100%=原始时长，50%=响一半时间（更短促、不吵）；0 会被提升为 1 */
void Sound_SetVolume(uint8_t percent);

/* 登记一次播放：count 个"嘟"声（每个 = on 拍 + off 拍），longBeep=1 时长音
 * 例：菜单 CLICK → Sound_Beep(1,0)；破纪录 → Sound_Beep(3,0)；GAME OVER → Sound_Beep(1,1) */
void Sound_Beep(uint8_t count, uint8_t longBeep);

/* housekeeping 每 100ms 调用：推进播放计划；播放中返回 1，空闲返回 0 */
uint8_t Sound_Poll(void);

#endif /* __SOUND_H */
