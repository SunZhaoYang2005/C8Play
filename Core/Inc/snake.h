/**
  ******************************************************************************
  * @file    snake.h
  * @brief   贪吃蛇
  *          数据模型：20x11 网格 + 环形数组蛇身（容量 SNAKE_MAX_LEN）
  *          128x64 屏每格 4px，游戏区 80x44 像素（居中，顶部 16px 标题/分数）
  *          接口 3 个函数 + 返回值的约定，由 app.c 接入：
  *            Snake_Init()          进入 SNAKE 模式时调用（重置蛇/食物/分数）
  *            Snake_HandleEvent()   返回 1 = 已消费 LONG_PRESS（游戏进行中，
  *                                  长按被持续加速占用，不返回菜单）
  *            Snake_Tick()          每帧调用（gameTask 33ms）：节拍移动 + 渲染
  * 按键映射  MOVE 转向(禁180°掉头) / CLICK 短按=加速一拍 / ENC_HOLD 长按=持续加速 /
  *          ENC_RELEASE 松开恢复节拍 / KEY K1=暂停/继续 / 红外 + 与 Repeat 同 CLICK/HOLD
  *          暂停态：CLICK 或 K1 继续；LONG_PRESS 返回菜单（app.c 处理）
  ******************************************************************************
  */
#ifndef __SNAKE_H
#define __SNAKE_H

#include <stdint.h>
#include "input_event.h"

/* 游戏区域（20 列 x 11 行，每格 4px；边框画在格子区外扩 1px，
 * 与俄罗斯方块同策略：格子像素与边框线不重叠，增量擦除永不破坏边框） */
#define SNAKE_COLS     20
#define SNAKE_ROWS     11

/* 蛇身环形数组容量 = 蛇长上限（20x11 网格理论满格 220，实际游玩不会达到，
 * 环形数组满后吃食物不再增长，防御性封顶） */
#define SNAKE_MAX_LEN  100

/* 重置游戏（进入 SNAKE 模式时由 app.c 调用） */
void Snake_Init(void);

/* 处理输入事件：MOVE→转向 / CLICK→加速一拍 / ENC_HOLD→持续加速 /
 * KEY→暂停 / LONG_PRESS 进行中吞掉（返回 1），暂停/结算态不消费（返回 0） */
int Snake_HandleEvent(const InputEvent *ev);

/* 每帧调用（gameTask 33ms）：节拍移动 + 增量渲染 */
void Snake_Tick(void);

/* 存档接入：分数/结算状态查询 + 最高分注入
 * app.c 在游戏结算上升沿查分数、进入游戏时注入 HI（Game Over 屏显示） */
int32_t Snake_GetScore(void);
uint8_t Snake_IsGameOver(void);
void Snake_SetHiScore(int32_t hi);

#endif /* __SNAKE_H */
