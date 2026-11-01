/**
  ******************************************************************************
  * @file    tetris.h
  * @brief   俄罗斯方块
  *          数据模型：10x20 网格 + 4x4 当前方块；128x64 屏每格 4px，
  *          可视 15 行（board 顶部 5 行为出生缓冲，不显示；
  *          格子起点 (1,1)，游戏区 42x62 完整边框与格子不重叠）
  *          接口只有 3 个函数，由 app.c 接入：Init / HandleEvent / Tick
  ******************************************************************************
  */
#ifndef __TETRIS_H
#define __TETRIS_H

#include <stdint.h>
#include "input_event.h"

/* 游戏区域（标准俄罗斯方块棋盘） */
#define TETRIS_COLS     10
#define TETRIS_ROWS     20
#define TETRIS_TOP_HIDE 5       /* 顶部隐藏行数（出生缓冲，屏幕只显示 row>=5） */

/* 重置游戏（进入 TETRIS 模式时由 app.c 调用） */
void Tetris_Init(void);

/* 处理输入事件：MOVE→左右移动 / CLICK→旋转 / KEY→硬降（后续部分填充） */
void Tetris_HandleEvent(const InputEvent *ev);

/* 每帧调用（gameTask 33ms）：节拍下落 + 渲染（第一部分仅渲染） */
void Tetris_Tick(void);

/* 存档接入：分数/结算状态查询 + 最高分注入
 * app.c 在游戏结算上升沿查分数、进入游戏时注入 HI（Game Over 屏显示） */
int32_t Tetris_GetScore(void);
uint8_t Tetris_IsGameOver(void);
void Tetris_SetHiScore(int32_t hi);

#endif /* __TETRIS_H */
