/**
  ******************************************************************************
  * @file    breakout.h
  * @brief   打砖块
  *          数据模型：20x11 格（每格 4px）+ 砖块位图 3x20 + 挡板(4格=16px) + 球
  *          球为像素坐标（每帧 33ms 平滑移动，无步进跳变）；碰撞按像素分轴判定
  *          128x64 屏：游戏区 80x44 像素居中（格子起点 (25,17)），顶部 16px HUD 行
  *          挡板/球/命 采用韦东山 N|Watch 打砖块位图样式：
  *            挡板 16x8 圆头（画屏幕底部）、球 2x2（球底贴挡板顶行）、命 7x8 心形 ×3（右上角）；
  *            输入支持编码器/MPU6050 倾斜控挡板；发球慢斜 45° 起步，反弹四档角度
  *          接口 3 个函数 + 返回值约定，由 app.c 接入（与 tetris/snake 同模式）：
  *            Breakout_Init()          进入 BREAKOUT 模式时调用（重置砖/挡板/球/分数）
  *            Breakout_HandleEvent()   返回 0 = 未消费 LONG_PRESS（app.c 返回菜单）
  *            Breakout_Tick()          每帧调用（gameTask 33ms）：像素移动 + 渲染
  * 按键映射  MOVE 挡板左右 / CLICK 发球·通关继续 / KEY K1=暂停/继续 /
  *          LONG_PRESS 返回菜单（app.c 处理）
  * 游戏流程  READY(待发球,球贴挡板) → PLAYING(球每帧平滑移动,分轴碰撞) →
  *          失球(3 球机会) → GAMEOVER；砖全消 → CLEAR 屏 → CLICK 下一关(提速)
  * 铁律     分轴碰撞：每帧先 X 后 Y，撞砖/壁只反射该轴方向；
  *          先判后动，失败无副作用（不穿模、不一次撞两砖）
  ******************************************************************************
  */
#ifndef __BREAKOUT_H
#define __BREAKOUT_H

#include <stdint.h>
#include "input_event.h"

/* 游戏区域（20 列 x 11 行，每格 4px；边框画在格子区外扩 1px，
 * 与俄罗斯方块/贪吃蛇同策略：格子像素与边框线不重叠，增量擦除永不破坏边框） */
#define BRICK_COLS     20
#define BRICK_ROWS     3       /* 砖块区行数（顶部 3 行） */
#define PADDLE_W       4       /* 挡板宽（格数）= 16px（韦东山位图加宽后适中宽度） */
#define PADDLE_ROW     10      /* 挡板所在行（游戏区最底行） */

/* 重置游戏（进入 BREAKOUT 模式 / Game Over 后 CLICK 重开） */
void Breakout_Init(void);

/* 处理输入事件：MOVE→挡板左右 / CLICK→发球·通关继续 / KEY→暂停 /
 * LONG_PRESS 一律不消费（返回 0 → app.c 返回菜单） */
int Breakout_HandleEvent(const InputEvent *ev);

/* 每帧调用（gameTask 33ms）：节拍移动 + 增量渲染 */
void Breakout_Tick(void);

/* 存档接入：分数/结算状态查询 + 最高分注入
 * app.c 在游戏结算上升沿查分数、进入游戏时注入 HI（Game Over 屏显示） */
int32_t Breakout_GetScore(void);
uint8_t Breakout_IsGameOver(void);
void Breakout_SetHiScore(int32_t hi);

#endif /* __BREAKOUT_H */
