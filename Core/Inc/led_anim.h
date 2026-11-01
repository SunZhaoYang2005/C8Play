/**
  ******************************************************************************
  * @file    led_anim.h
  * @brief   全彩 LED 状态动画：按掌机状态点亮对应颜色
  * 设计     app.c 在状态转移处调 LedAnim_SetState；housekeepingTask 每 100ms
  *          调 LedAnim_Poll() 驱动闪烁节拍（慢操作/节拍全在低优先级任务）
  ******************************************************************************
  */
#ifndef __LED_ANIM_H
#define __LED_ANIM_H

#include <stdint.h>

/* 掌机状态 → LED 颜色：
 *   菜单=绿 / 游戏中=蓝 / GAME OVER=红(闪) / 清档成功=白(闪) */
typedef enum {
    LED_STATE_MENU = 0,
    LED_STATE_GAME,
    LED_STATE_GAMEOVER,
    LED_STATE_CLEARED,
    LED_STATE_COUNT
} LedState;

/* 初始化全彩 LED（App_Init 调用；GPIO 配置快不阻塞） */
void LedAnim_Init(void);

/* 设置当前状态（颜色+闪烁相位重置）；非法值忽略 */
void LedAnim_SetState(LedState st);

/* housekeeping 每 100ms 调用：闪烁状态按 500ms 周期翻转亮灭 */
void LedAnim_Poll(void);

#endif /* __LED_ANIM_H */
