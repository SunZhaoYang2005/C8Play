/**
  ******************************************************************************
  * @file    screen.h
  * @brief   掌机 OLED 渲染引擎（SSD1306 128x64 帧缓冲），帧缓冲 + 脏列区间局部刷新
  * 说明     所有坐标均为像素坐标（0~127, 0~63），游戏/菜单直接绘制
  ******************************************************************************
  */
#ifndef __SCREEN_H
#define __SCREEN_H

#include <stdint.h>

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64

/* 初始化屏幕（SSD1306 完整初始化序列） */
void Screen_Init(void);

/* 清空帧缓冲（不刷屏） */
void Screen_Clear(void);

/* 判定 OLED 是否处于"总线异常冻结期"：Flush 连续 ≥2 帧刷不出去返回 1。
 * 上层（app.c）据此暂停游戏推进、只重试刷屏，总线恢复后自动继续 */
int Screen_IsStalled(void);

/* 复位 I2C1 外设（HAL I2C 卡 BUSY 的标准恢复）：冻结期间每 500ms 调用，
 * 完整重建总线（DeInit→MspDeInit→Init→MspInit），总线自愈关键 */
void Screen_BusReset(void);

/* 将帧缓冲中"脏"的页/列区间刷新到 OLED（无脏区域则 0 次 I2C 传输） */
void Screen_Flush(void);

/* 画单个像素：on=1 点亮, on=0 熄灭 */
void Screen_DrawPixel(uint8_t x, uint8_t y, uint8_t on);

/* 画 8x16 字符（复用 ascii_font 字模），x/y 为字符左上角像素坐标 */
void Screen_DrawChar8x16(uint8_t x, uint8_t y, char c);

/* 画 8x16 字符串 */
void Screen_DrawString8x16(uint8_t x, uint8_t y, const char *str);

/* 画 8x16 十进制有符号数（分数/计数显示用） */
void Screen_PrintSignedVal(uint8_t x, uint8_t y, int32_t val);

/* 画 8x16 十进制有符号数（以 center_x 为水平中心自动居中，结算屏对称布局用） */
void Screen_PrintSignedValCentered(uint8_t center_x, uint8_t y, int32_t val);

/* 画 8x16 十六进制数 */
void Screen_PrintHex(uint8_t x, uint8_t y, uint32_t val);

/* 填充实心矩形（游戏方块用），on=1 填充, on=0 清除 */
void Screen_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t on);

/* 画矩形边框 */
void Screen_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t on);

/* 画列格式位图（每字节一列，bit0=顶部，高度≤8）；
 * on=1 点亮 / on=0 擦除（透明区域不动，增量擦除安全） */
void Screen_DrawBitmap(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const uint8_t *bitmap, uint8_t on);

#endif /* __SCREEN_H */
