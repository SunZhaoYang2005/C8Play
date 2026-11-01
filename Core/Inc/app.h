/**
  ******************************************************************************
  * @file    app.h
  * @brief   掌机应用层：主菜单状态机 + 模式分发
  *          由 game_task 每 33ms(30fps) 调用 App_Process()
  *          菜单骨架（滚动/进入/长按返回）
  *          接入俄罗斯方块（TETRIS）
  *          贪吃蛇/打砖块/桌面/设置/关于 + Flash 双槽存档 + 外设
  *          UART 协议层（protoTask 收帧 → 本层命令分发 + 应答 + 状态上报）
  ******************************************************************************
  */
#ifndef __APP_H
#define __APP_H

#include <stdint.h>
#include "proto.h"   /* 串口协议帧（App_HandleProtoFrame 参数） */

/* 应用状态（即主状态机节点） */
typedef enum {
    APP_STATE_MENU = 0,     /* 主菜单 */
    APP_STATE_TETRIS,       /* 俄罗斯方块 */
    APP_STATE_SNAKE,        /* 贪吃蛇 */
    APP_STATE_BREAKOUT,     /* 打砖块 */
    APP_STATE_DESKTOP,      /* 桌面模式：DHT11 温湿度 + 光敏 */
    APP_STATE_SETTINGS,     /* 设置：清档确认；音量/亮度界面预留 */
    APP_STATE_ABOUT,        /* 关于 */
    APP_STATE_COUNT
} AppState;

/* 应用层初始化（game_task 启动时调用一次） */
void App_Init(void);

/* 应用层主循环（game_task 每帧调用：收事件 -> 状态机 -> 渲染） */
void App_Process(void);

/* 串口协议命令分发入口（protoTask 收到完整合法帧时调用，protoTask 上下文）。
 * 实现命令处理 + 应答帧发送（Proto_Send） */
void App_HandleProtoFrame(const ProtoFrame *frame);

#endif /* __APP_H */
