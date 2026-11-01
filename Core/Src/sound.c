/**
  ******************************************************************************
  * @file    sound.c
  * @brief   有源蜂鸣器音效（PA8，低电平驱动，ActiveBuzzer_Control(on)=拉低=响）。
  *          【方案修正】曾用 TIM1 高频 PWM 降平均电压调音量，实测有源蜂鸣器
  *          内部振荡器对欠压敏感：50% 占空比即低于最低工作电压 → 停振不响。
  *          正确做法 = 时间调制：蜂鸣器始终满压工作，音量比例压缩"响的时长"
  *          （100% = 原始时长；50% = 响一半时间），响度不变但更短促、不吵。
  *          播放计划由 app.c 登记（异步），housekeeping 每 100ms 调 Sound_Poll()
  *          用 tick 计时推进：短音 = on 100ms + off 100ms；长音 = on 300ms + off 100ms
  ******************************************************************************
  */

#include "sound.h"
#include "driver_active_buzzer.h"
#include "cmsis_os.h"       /* osKernelGetTickCount：拍时长计时 */

/* ---------------- 参数 ---------------- */

#define SOUND_ON_SHORT_MS   100u  /* 短音 on 时长（100% 音量） */
#define SOUND_ON_LONG_MS    300u  /* 长音 on 时长 */
#define SOUND_OFF_MS        100u  /* 拍间 off 时长 */
#define SOUND_VOL_DEFAULT   50u   /* 默认音量 50%（on 时长减半） */

/* ---------------- 状态 ---------------- */

static uint8_t  s_beepsLeft = 0;   /* 剩余"嘟"声数 */
static uint8_t  s_longBeep = 0;    /* 1 = 长音 */
static uint8_t  s_phase = 0;       /* 0=off 拍 1=on 拍 */
static uint32_t s_phaseStart = 0;  /* 当前拍开始 tick */
static uint8_t  s_volPercent = 100;/* 音量 1~100（on 时长比例） */

/* ---------------- 内部 ---------------- */

/* 当前 on 拍的实际响时长（按音量比例压缩） */
static uint32_t sound_on_ms(void)
{
    uint32_t base = s_longBeep ? SOUND_ON_LONG_MS : SOUND_ON_SHORT_MS;
    return (base * s_volPercent) / 100u;
}

/* ---------------- 对外接口 ---------------- */

void Sound_Init(void)
{
    ActiveBuzzer_Init();        /* PA8 = GPIO 推挽输出 */
    ActiveBuzzer_Control(0);    /* 初始静音（高电平） */
    s_beepsLeft = 0;
    s_longBeep = 0;
    s_phase = 0;
    s_phaseStart = 0;
    Sound_SetVolume(SOUND_VOL_DEFAULT);
}

void Sound_SetVolume(uint8_t percent)
{
    if (percent > 100u)
    {
        percent = 100u;
    }
    if (percent < 1u)
    {
        percent = 1u;   /* 不允许 0：保证 beep 必响（仅时长缩短） */
    }
    s_volPercent = percent;
}

void Sound_Beep(uint8_t count, uint8_t longBeep)
{
    uint8_t wasIdle;

    if (count == 0u)
    {
        return;
    }
    /* 空闲判定 = 无剩余声数且当前在 off 拍完成态（on 拍中不算空闲） */
    wasIdle = (s_beepsLeft == 0u && s_phase == 0u) ? 1u : 0u;

    /* 播放中再登记：追加到当前计划（最多 5 声，防止队列无限积压） */
    s_beepsLeft += count;
    if (s_beepsLeft > 5u)
    {
        s_beepsLeft = 5u;
    }
    if (longBeep)
    {
        s_longBeep = 1;
    }
    if (wasIdle)
    {
        s_phase = 1;                    /* 空闲 → 立即开始第一声 on 拍 */
        s_phaseStart = osKernelGetTickCount();
    }
}

uint8_t Sound_Poll(void)
{
    uint32_t now = osKernelGetTickCount();
    uint32_t elapsed = now - s_phaseStart;

    if (s_beepsLeft == 0u && s_phase == 0u)
    {
        ActiveBuzzer_Control(0);
        return 0;                       /* 空闲 */
    }

    if (s_phase)
    {
        /* on 拍：响（满压），时长按音量比例压缩 */
        ActiveBuzzer_Control(1);
        if (elapsed >= sound_on_ms())
        {
            s_phase = 0;                /* on 拍完成 → 转入 off 拍 */
            s_phaseStart = now;
        }
    }
    else
    {
        /* off 拍：这一声完成；还有下一声才进入 on 拍，否则播放结束 */
        ActiveBuzzer_Control(0);
        if (elapsed >= SOUND_OFF_MS)
        {
            if (s_beepsLeft > 0u)
            {
                s_beepsLeft--;
            }
            if (s_beepsLeft > 0u)
            {
                s_phase = 1;
                s_phaseStart = now;
            }
            else
            {
                s_phase = 0;            /* 播放结束：下次 Poll 空闲退出 */
            }
        }
    }
    return 1;
}
