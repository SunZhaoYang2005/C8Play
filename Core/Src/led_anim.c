/**
  ******************************************************************************
  * @file    led_anim.c
  * @brief   全彩 LED 状态动画：
  *          按掌机状态点亮对应颜色，GAME OVER / 清档成功按 500ms 周期闪烁
  * 接线     ColorLED_Set(uint32_t color)，格式 0x00RRGGBB（韦东山全彩 LED 驱动）
  * 设计     状态由 app.c 在状态转移处设置；闪烁节拍由 housekeeping 100ms 轮询驱动。
  *          常亮状态（菜单/游戏）颜色不变时不重复 Set
  ******************************************************************************
  */

#include "led_anim.h"
#include "driver_color_led.h"

/* ---------------- 参数 ---------------- */

#define LED_BLINK_HALF   5u   /* 闪烁半周期：5 × 100ms = 500ms 亮/灭（GAME OVER/清档） */

/* ---------------- 状态 ---------------- */

static LedState s_state = LED_STATE_MENU;
static uint8_t  s_phase = 0;   /* 闪烁相位：0=亮 1=灭 */
static uint8_t  s_tick = 0;    /* 半周期节拍计数 */
static uint32_t s_lastColor = 0xFFFFFFFFu;  /* 上次输出的颜色（去重：颜色不变不重配 PWM） */

/* ---------------- 内部函数 ---------------- */

static uint32_t led_color_of(LedState st)
{
    switch (st)
    {
    case LED_STATE_GAME:      return 0x0000FFu;   /* 蓝 */
    case LED_STATE_GAMEOVER:  return 0xFF0000u;   /* 红 */
    case LED_STATE_CLEARED:   return 0xFFFFFFu;   /* 白 */
    case LED_STATE_MENU:
    default:                  return 0x00FF00u;   /* 绿 */
    }
}

/* ---------------- 对外接口 ---------------- */

void LedAnim_Init(void)
{
    ColorLED_Init();
    s_state = LED_STATE_MENU;
    s_phase = 0;
    s_tick = 0;
    s_lastColor = 0xFFFFFFFFu;
    ColorLED_Set(0x00FF00u);
    s_lastColor = 0x00FF00u;
}

void LedAnim_SetState(LedState st)
{
    if (st >= LED_STATE_COUNT)
    {
        return;
    }
    if (st == s_state)
    {
        return;   /* 状态未变：不重置闪烁相位（app 每帧调用，避免打断闪烁） */
    }
    s_state = st;
    s_phase = 0;
    s_tick = 0;
}

void LedAnim_Poll(void)
{
    uint32_t color = led_color_of(s_state);

    /* 常亮状态（菜单/游戏）：固定颜色不重配（去重，零 PWM 抖动） */
    if (s_state == LED_STATE_MENU || s_state == LED_STATE_GAME)
    {
        if (color != s_lastColor)
        {
            ColorLED_Set(color);
            s_lastColor = color;
        }
        return;
    }

    /* 闪烁状态（GAME OVER / 清档）：每半周期翻转亮灭 */
    if (++s_tick >= LED_BLINK_HALF)
    {
        s_tick = 0;
        s_phase ^= 1;
    }
    color = s_phase ? 0x000000u : color;
    if (color != s_lastColor)
    {
        ColorLED_Set(color);
        s_lastColor = color;
    }
}
