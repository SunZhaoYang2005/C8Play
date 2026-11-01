/**
  ******************************************************************************
  * @file    app.c
  * @brief   掌机应用层：主菜单状态机 + 模式分发
  *          输入事件统一由 input_task 采样入队，本层只消费事件不碰硬件
  *          约定：INPUT_EV_MOVE value>0 = 顺时针/右/下，value<0 = 逆时针/左/上
  *          红外键码 -> 标准语义事件翻译（app_ir_translate），菜单/游戏全部复用，游戏层无需感知红外存在
  *          渲染为增量绘制：整屏/滚动/光标三档，配合 screen.c 脏列局部刷新，静止画面 0 次 I2C 传输
  ******************************************************************************
  */

#include "app.h"
#include "screen.h"
#include "input_event.h"
#include "tetris.h"
#include "snake.h"
#include "breakout.h"
#include "save.h"        /* 三游戏最高分存档 */
#include "env.h"         /* 环境数据：DHT11 温湿度 + 光敏光强 */
#include "led_anim.h"    /* 全彩 LED 状态动画 */
#include "sound.h"       /* 蜂鸣器音效 */

extern void Mem_DumpTasks(void);   /* freertos.c：任务栈高水位打印（0x08 命令触发） */
#include "cmsis_os.h"    /* osDelay（冻结期降频重试） */
#include "stm32f1xx.h"   /* NVIC_SystemReset（冻结超时软件复位兜底） */
#include <stddef.h>   /* NULL（红外翻译未映射键的返回值） */

/* ---------------- 红外键码定义 ----------------
 * 红外在 input 层已投递为 INPUT_EV_IR（value = 遥控器键码，键码表见
 * Drivers/DshanMCU-F103/driver_ir_receiver.c 的 IRReceiver_CodeToString）。
 * 本层统一翻译成标准语义事件，菜单/俄罗斯方块/未来游戏全部复用，
 * 未映射的键（含 NEC Repeat 码 0x00，长按连发）直接忽略 */
#define IR_KEY_LEFT     0xE0u   /* 遥控器 Left */
#define IR_KEY_RIGHT    0x90u   /* 遥控器 Right */
#define IR_KEY_PLUS     0x02u   /* 遥控器 + */
#define IR_KEY_MINUS    0x98u   /* 遥控器 - */
#define IR_KEY_PLAY     0xA8u   /* 遥控器 Play（= 确认） */
#define IR_KEY_RETURN   0xC2u   /* 遥控器 Return（= 返回） */
#define IR_KEY_REPEAT   0x00u   /* NEC Repeat（按住连发 ~110ms/次） */

/* 上次翻译成功的红外键码（NEC Repeat 无"哪个键"信息，靠它重放持续语义） */
static uint8_t s_lastIrKey = 0xFF;

/* 菜单项名称（顺序与 AppState 一致，跳过 MENU 自身） */
static const char * const kMenuNames[] = {
    "TETRIS",       /* APP_STATE_TETRIS   */
    "SNAKE",        /* APP_STATE_SNAKE    */
    "BREAKOUT",     /* APP_STATE_BREAKOUT */
    "DESKTOP",      /* APP_STATE_DESKTOP  */
    "SETTINGS",     /* APP_STATE_SETTINGS */
    "ABOUT"         /* APP_STATE_ABOUT    */
};
#define MENU_COUNT  6u
#define MENU_ROWS   3u      /* 128x64 屏：标题占 0 行，菜单 8x16 行高 16，可视 3 行(16/32/48) */

/* 应用状态 */
static AppState g_state = APP_STATE_MENU;
static int8_t   g_menuIndex = 0;      /* 当前选中菜单项 0..MENU_COUNT-1 */
static uint32_t g_frameCount = 0;     /* 帧计数（供后续动画/刷新节拍用） */
static uint8_t  s_lastGoSeen = 0;     /* 当前游戏上一帧是否已 Game Over（存档破纪录只在上升沿判一次） */

/* 增量绘制状态：只重画变化区域（配合 screen.c 脏列区间局部刷新）
 * s_prevMenuTop / s_prevMenuIndex 记录上次绘制快照；
 * s_menuForceRedraw=1 表示整屏重画（首次进入 / 从模式态返回）；
 * s_lastDrawnState 用于占位页去重（模式态只画一次） */
static int8_t   s_prevMenuTop = -1;
static int8_t   s_prevMenuIndex = 0;
static uint8_t  s_menuForceRedraw = 1;
static AppState s_lastDrawnState = APP_STATE_MENU;

/* SETTINGS 清档交互状态：
 * s_settingsSel=0 光标在 NO（默认，防误触清档）/ 1 在 YES；
 * s_settingsCleared=1 表示已清档（画面显示 CLEARED! 停留） */
static uint8_t  s_settingsSel = 0;
static uint8_t  s_settingsCleared = 0;
static uint8_t  s_clearedDrawn = 0;
static uint8_t  s_lastSel = 0xFF;

/* DESKTOP 环境数值绘制快照：数值变化才刷新行，静止帧 0 传输 */
static int32_t  s_lastEnvTemp = 0x7FFFFFFF;
static int32_t  s_lastEnvHumi = 0x7FFFFFFF;
static uint8_t  s_lastEnvLight = 0xFF;

/* 总线冻结自愈状态（卡死根治）：
 * s_freezeTick   冻结开始时刻（0 = 未冻结；解冻清零）
 * s_busResetTick 上次 I2C 外设复位时刻（每 500ms 一次） */
static uint32_t s_freezeTick = 0;
static uint32_t s_busResetTick = 0;

/**
  * @brief  应用层初始化（gameTask 启动时调用一次）
  * @note   OLED 初始化放任务内（标准 RTOS 写法：慢速外设初始化
  *         不阻塞调度器启动路径），异常不影响系统启动
  */
void App_Init(void)
{
    Screen_Init();
    Save_Init();        /* 读 W25Q64 最高分存档到缓存 */
    Env_Init();         /* DHT11 引脚配置 + 环境缓存清零 */
    LedAnim_Init();     /* 全彩 LED 初始化 */
    Sound_Init();       /* 蜂鸣器初始化 */
    g_state = APP_STATE_MENU;
    g_menuIndex = 0;
    g_frameCount = 0;
}

/* ---------------- 0x07 状态上报 ----------------
 * 掌机主动向 PC 推送状态变化：切模式 / 破纪录。
 * 由 gameTask 上下文调用（本文件内均为此上下文），与 protoTask 的
 * 命令应答共用 USART1 发送互斥（freertos.c g_uartTxMutex），帧不交错。
 * 上位机收到后可即时同步界面（切模式回显 / 刷新最高分），无需轮询。 */
static void app_notify_status(uint8_t evt, uint8_t p0, uint8_t p1, uint8_t p2)
{
    uint8_t buf[4];

    buf[0] = evt;   /* PROTO_ST_EVT_MODE / PROTO_ST_EVT_HISCORE */
    buf[1] = p0;    /* 模式值 或 游戏索引（SaveGame，与读最高分顺序一致） */
    buf[2] = p1;    /* 保留 或 分数低字节 */
    buf[3] = p2;    /* 保留 或 分数高字节 */
    (void)Proto_Send(PROTO_CMD_STATUS, buf, 4);
}

/* ---------------- 存档破纪录判定 ----------------
 * 在游戏 Game Over 的"上升沿"（上一帧未结算、本帧已结算）检查一次：
 * 破纪录 → Save_Submit 更新缓存 + 置脏标志（落盘由 housekeeping 轮询），
 * 未破纪录 → 一个字都不写。s_lastGoSeen 在进入游戏时清零。 */

/**
  * @brief  结算检查：Game Over 上升沿提交存档 + LED/音效反馈
  * @param  game  游戏索引（SAVE_GAME_*）
  * @param  go    本帧是否已 Game Over
  * @param  score 本局最终分数
  * @note   LED：游戏中=蓝（每帧置状态，未变化不重置相位）；GAME OVER=红闪；
  *         音效：破纪录 3 短音，普通结束 1 长音；
  *         破纪录成功后额外推送 0x07 HISCORE 帧（game + 分数 LE）
  */
static void app_check_record(SaveGame game, uint8_t go, int32_t score)
{
    if (go && !s_lastGoSeen)
    {
        if (Save_Submit(game, score))   /* 破纪录：内部比较，未破不写 Flash */
        {
            Sound_Beep(3, 0);           /* 3 短：破纪录庆祝 */
            app_notify_status(PROTO_ST_EVT_HISCORE, (uint8_t)game,
                              (uint8_t)((uint32_t)score & 0xFFu),
                              (uint8_t)(((uint32_t)score >> 8) & 0xFFu));
        }
        else
        {
            Sound_Beep(1, 1);           /* 1 长：普通结束 */
        }
        LedAnim_SetState(LED_STATE_GAMEOVER);
    }
    else if (!go)
    {
        LedAnim_SetState(LED_STATE_GAME);   /* 游戏进行中：蓝（重开后自动恢复） */
    }
    s_lastGoSeen = go;
}

/**
  * @brief  切换到指定模式（菜单 CLICK 与串口协议 SET_MODE 共用入口）
  * @param  mode  APP_STATE_MENU=0 / TETRIS=1 / SNAKE=2 / BREAKOUT=3
  * @note   进入游戏时重置游戏状态并注入历史最高分（Game Over 屏显示 HI）；
  *         返回菜单须整屏重画（s_menuForceRedraw），避免残留占位页内容；
  *         切换成功后推送 0x07 MODE 帧，上位机回显当前模式
  */
static void app_enter_mode(uint8_t mode)
{
    g_state = (AppState)mode;
    s_menuForceRedraw = (mode == APP_STATE_MENU) ? 1 : 0;

    if (mode == APP_STATE_TETRIS)
    {
        Tetris_Init();
        Tetris_SetHiScore(Save_GetHi(SAVE_GAME_TETRIS));
        s_lastGoSeen = 0;
        LedAnim_SetState(LED_STATE_GAME);
    }
    else if (mode == APP_STATE_SNAKE)
    {
        Snake_Init();
        Snake_SetHiScore(Save_GetHi(SAVE_GAME_SNAKE));
        s_lastGoSeen = 0;
        LedAnim_SetState(LED_STATE_GAME);
    }
    else if (mode == APP_STATE_BREAKOUT)
    {
        Breakout_Init();
        Breakout_SetHiScore(Save_GetHi(SAVE_GAME_BREAKOUT));
        s_lastGoSeen = 0;
        LedAnim_SetState(LED_STATE_GAME);
    }
    else if (mode == APP_STATE_DESKTOP || mode == APP_STATE_SETTINGS ||
             mode == APP_STATE_ABOUT)
    {
        /* 桌面/设置/关于：进入时重置各自交互状态（LED 保持菜单绿） */
        s_settingsSel = 0;
        s_settingsCleared = 0;
        s_clearedDrawn = 0;
        s_lastSel = 0xFF;
        s_lastEnvTemp = 0x7FFFFFFF;   /* 强制首帧刷新环境数值 */
        s_lastEnvHumi = 0x7FFFFFFF;
        s_lastEnvLight = 0xFF;
        LedAnim_SetState(LED_STATE_MENU);
    }

    /* 模式切换成功后主动上报（含返回菜单），上位机实时回显 */
    app_notify_status(PROTO_ST_EVT_MODE, mode, 0, 0);
}

/* ---------------- 事件处理 ---------------- */

/**
  * @brief  红外键码翻译：INPUT_EV_IR -> 标准语义事件
  * @param  ev  原始事件（INPUT_EV_IR，value=遥控器键码）
  * @param  e2  翻译结果存放处（由调用方提供，生命周期覆盖调用方）
  * @retval 翻译后的事件指针；NULL = 未映射的键，调用方应忽略
  * @note   一次翻译、处处复用：菜单态能滚动/进入/返回，
  *         游戏态直接获得左右/旋转/硬降能力，游戏层不感知红外
  */
static const InputEvent *app_ir_translate(const InputEvent *ev, InputEvent *e2)
{
    switch (ev->value)
    {
    case IR_KEY_LEFT:   s_lastIrKey = (uint8_t)ev->value;
                        e2->type = INPUT_EV_MOVE;       e2->value = -1; return e2;
    case IR_KEY_RIGHT:  s_lastIrKey = (uint8_t)ev->value;
                        e2->type = INPUT_EV_MOVE;       e2->value = 1;  return e2;
    case IR_KEY_PLUS:   s_lastIrKey = (uint8_t)ev->value;
                        e2->type = INPUT_EV_CLICK;      e2->value = 1;  return e2;  /* + = 旋转/确认/加速一拍 */
    case IR_KEY_MINUS:  s_lastIrKey = (uint8_t)ev->value;
                        e2->type = INPUT_EV_KEY;        e2->value = 1;  return e2;  /* - = 硬降 */
    case IR_KEY_PLAY:   s_lastIrKey = (uint8_t)ev->value;
                        e2->type = INPUT_EV_CLICK;      e2->value = 1;  return e2;  /* Play = 确认 */
    case IR_KEY_RETURN: s_lastIrKey = (uint8_t)ev->value;
                        e2->type = INPUT_EV_LONG_PRESS; e2->value = 1;  return e2;  /* Return = 返回 */
    case IR_KEY_REPEAT: /* NEC 长按连发（~110ms/次）：重放最近按键的持续语义 */
        switch (s_lastIrKey)
        {
        case IR_KEY_LEFT:   e2->type = INPUT_EV_MOVE;     e2->value = -1; return e2;  /* 俄罗斯方块/菜单长按连续移动 */
        case IR_KEY_RIGHT:  e2->type = INPUT_EV_MOVE;     e2->value = 1;  return e2;
        case IR_KEY_PLUS:   e2->type = INPUT_EV_IR_HOLD;  e2->value = 1;  return e2;  /* 贪吃蛇持续加速 */
        default:            return NULL;   /* MINUS/PLAY/RETURN 无长按语义 */
        }
    default:            return NULL;   /* 数字键 / 未知键忽略 */
    }
}

static void app_handle_event(const InputEvent *ev)
{
    /* 串口协议 0x05 切模式：跨任意状态直接切换，
     * 事件由 protoTask 投递、gameTask 统一消费——应用状态只在
     * gameTask 上下文修改，避免跨任务竞态（protoTask 不碰 g_state） */
    if (ev->type == INPUT_EV_PROTO_SET_MODE)
    {
        app_enter_mode((uint8_t)ev->value);
        return;
    }

    /* 红外键码 -> 标准语义（一次翻译，菜单/所有游戏复用；未映射键忽略） */
    if (ev->type == INPUT_EV_IR)
    {
        InputEvent e2;   /* 翻译结果：生命周期覆盖整个函数 */
        ev = app_ir_translate(ev, &e2);
        if (ev == NULL)
        {
            return;
        }
    }

    if (g_state == APP_STATE_MENU)
    {
        /* 菜单态 */
        switch (ev->type)
        {
        case INPUT_EV_MOVE:
            if (ev->value > 0)
            {
                g_menuIndex = (int8_t)((g_menuIndex + 1) % MENU_COUNT);
            }
            else
            {
                g_menuIndex = (int8_t)((g_menuIndex - 1 + MENU_COUNT) % MENU_COUNT);
            }
            break;

        case INPUT_EV_CLICK:
            /* 进入所选模式：TETRIS=1 ... ABOUT=6 */
            Sound_Beep(1, 0);   /* 菜单确认短音（阶段3-4） */
            app_enter_mode((uint8_t)(g_menuIndex + 1));
            break;

        default:
            break;
        }
    }
    else
    {
        /* 模式态：游戏先消费自己的按键语义，LONG_PRESS 未被消费才返回菜单
         * （俄罗斯方块不消费 → 长按返回菜单；贪吃蛇进行中消费掉 → 长按=持续加速，
         *  暂停/结算态不消费 → 长按返回菜单） */
        uint8_t consumed = 0;
        if (g_state == APP_STATE_TETRIS)
        {
            Tetris_HandleEvent(ev);   /* MOVE→左右 / CLICK→旋转 / KEY→硬降（长按返回见下） */
        }
        else if (g_state == APP_STATE_SNAKE)
        {
            consumed = (uint8_t)Snake_HandleEvent(ev);   /* 返回 1 = 已消费 LONG_PRESS */
        }
        else if (g_state == APP_STATE_BREAKOUT)
        {
            consumed = (uint8_t)Breakout_HandleEvent(ev); /* 打砖块不消费 LONG_PRESS → 返回菜单 */
        }
        else if (g_state == APP_STATE_SETTINGS)
        {
            /* 设置：MOVE 翻转 NO/YES，CLICK 确认（默认 NO 防误触清档） */
            if (ev->type == INPUT_EV_MOVE)
            {
                s_settingsSel ^= 1;
            }
            else if (ev->type == INPUT_EV_CLICK && s_settingsSel)
            {
                Save_ClearAll();          /* 清三游戏最高分（置脏 → housekeeping 落盘全 0 帧） */
                s_settingsCleared = 1;
                s_clearedDrawn = 0;
                Sound_Beep(2, 0);
                LedAnim_SetState(LED_STATE_CLEARED);
            }
        }
        /* DESKTOP/ABOUT 无自定义交互：MOVE/CLICK 忽略，LONG 返回菜单（见下） */
        if (ev->type == INPUT_EV_LONG_PRESS && consumed == 0)
        {
            g_state = APP_STATE_MENU;
            s_menuForceRedraw = 1;   /* 返回菜单须整屏重画，避免残留占位页内容 */
            LedAnim_SetState(LED_STATE_MENU);
        }
    }
}

/* ---------------- 绘制 ---------------- */

/**
  * @brief  绘制主菜单（增量绘制：滚动窗口，选中项尽量保持在可视区中部）
  * @note   配合 screen.c 局部刷新，分三档：
  *         1) 整屏重画（首次 / 从模式态返回，Clear 标全屏脏）
  *         2) 窗口滚动：只重画菜单区 3 行，标题区保持不动
  *         3) 光标移动：只擦旧光标 + 画新光标（各 8 列 x 2 页）
  *         无变化时不做任何绘制，Flush 无脏可刷，0 次 I2C 传输
  */
static void app_draw_menu(void)
{
    int8_t top = (int8_t)(g_menuIndex - 1);
    int8_t i;

    if (top < 0)
    {
        top = 0;
    }
    if (top > (int8_t)(MENU_COUNT - MENU_ROWS))
    {
        top = (int8_t)(MENU_COUNT - MENU_ROWS);
    }

    if (s_menuForceRedraw)
    {
        /* 首次 / 从模式态返回：整屏重画（标题 + 菜单区） */
        Screen_Clear();
        Screen_DrawString8x16(16, 0, "PALM STATION");
        for (i = 0; i < (int8_t)MENU_ROWS; i++)
        {
            int8_t idx = (int8_t)(top + i);
            if (idx >= (int8_t)MENU_COUNT)
            {
                break;
            }
            Screen_DrawString8x16(8, (uint8_t)(16 + 16 * i), (idx == g_menuIndex) ? ">" : " ");
            Screen_DrawString8x16(16, (uint8_t)(16 + 16 * i), kMenuNames[idx]);
        }
    }
    else if (s_prevMenuTop != top)
    {
        /* 窗口滚动：只重画菜单区 3 行（标题区保持不动，不入脏） */
        for (i = 0; i < (int8_t)MENU_ROWS; i++)
        {
            int8_t idx = (int8_t)(top + i);
            if (idx >= (int8_t)MENU_COUNT)
            {
                break;
            }
            Screen_DrawString8x16(8, (uint8_t)(16 + 16 * i), (idx == g_menuIndex) ? ">" : " ");
            Screen_DrawString8x16(16, (uint8_t)(16 + 16 * i), kMenuNames[idx]);
        }
    }
    else if (s_prevMenuIndex != g_menuIndex)
    {
        /* 光标移动（窗口未滚动）：只擦旧光标 + 画新光标 */
        int8_t prevRow = s_prevMenuIndex - top;
        int8_t newRow  = g_menuIndex - top;
        Screen_DrawString8x16(8, (uint8_t)(16 + 16 * prevRow), " ");
        Screen_DrawString8x16(8, (uint8_t)(16 + 16 * newRow), ">");
    }
    /* else：画面无任何变化，不绘制 */

    s_prevMenuTop = top;
    s_prevMenuIndex = g_menuIndex;
    s_menuForceRedraw = 0;

    Screen_Flush();
}

/**
  * @brief  兜底绘制"开发中"页
  * @note   MENU/TETRIS/SNAKE/BREAKOUT/DESKTOP/SETTINGS/ABOUT
  *         均已实现，本函数仅在收到未知模式值时兜底显示，正常路径不触发
  * @note   DESKTOP/SETTINGS/ABOUT 均已实现，本函数仅作兜底
  */
static void app_draw_placeholder(void)
{
    const char *name = kMenuNames[(int)g_state - 1];

    Screen_Clear();
    Screen_DrawString8x16(16, 0, name);
    Screen_DrawString8x16(8, 32, "DEVELOPING");
    Screen_DrawString8x16(8, 48, "<LONG BACK");
    Screen_Flush();
}

/* ---------------- 模式页绘制（DESKTOP / SETTINGS / ABOUT） ---------------- */

/**
  * @brief  在值区 (56,y) 手写 8x16 数值字符（避免 PrintSignedVal 宽度不定导致后续字符错位）
  * @note   val <= -32768 → "--"；isFloat=1 时按 "整数.一位小数" 显示（val 为 x10，
  *         如 255 → "25.5"），frac 为小数位（负数自动取正）；unit 为后缀（'C'/'%'）
  */
static void app_draw_val_unit(uint8_t y, int32_t val, uint8_t isFloat, char unit, int32_t *frac)
{
    uint8_t x = 56;
    int32_t w;

    if (val <= -32768)
    {
        Screen_DrawString8x16(x, y, "--");
        return;
    }
    w = isFloat ? (val / 10) : val;
    if (w < 0)
    {
        Screen_DrawChar8x16(x, y, '-');
        x += 8;
        w = -w;
        if (frac != 0 && *frac < 0)
        {
            *frac = -*frac;
        }
    }
    if (w >= 100)
    {
        Screen_DrawChar8x16(x, y, (char)('0' + w / 100));
        x += 8;
    }
    if (w >= 10)
    {
        Screen_DrawChar8x16(x, y, (char)('0' + (w / 10) % 10));
        x += 8;
    }
    Screen_DrawChar8x16(x, y, (char)('0' + w % 10));
    x += 8;
    if (isFloat)
    {
        Screen_DrawChar8x16(x, y, '.');
        x += 8;
        Screen_DrawChar8x16(x, y, (char)('0' + (*frac % 10)));
        x += 8;
    }
    if (unit != 0)
    {
        Screen_DrawChar8x16(x, y, unit);
    }
}

/**
  * @brief  DESKTOP 桌面模式：DHT11 温湿度 + 光敏光强
  * @note   值变化才刷新（静止帧 0 I2C 传输）；"xx.xC" / "xx%" 手写字符
  */
static void app_draw_desktop(void)
{
    int32_t temp = Env_GetTemp();
    int32_t humi = Env_GetHumi();
    uint8_t l = Env_GetLight();

    if (s_lastDrawnState != APP_STATE_DESKTOP)
    {
        Screen_Clear();
        Screen_DrawString8x16(36, 0, "DESKTOP");       /* 7 字符 56px 居中 */
        Screen_DrawString8x16(8, 16, "TEMP");
        Screen_DrawString8x16(8, 32, "HUMI");
        Screen_DrawString8x16(8, 48, "LIGHT");
        s_lastDrawnState = APP_STATE_DESKTOP;
        Screen_Flush();
    }

    if (temp != s_lastEnvTemp || humi != s_lastEnvHumi || l != s_lastEnvLight)
    {
        int32_t frac = (temp <= -32768) ? 0 : (temp % 10);

        s_lastEnvTemp = temp;
        s_lastEnvHumi = humi;
        s_lastEnvLight = l;
        app_draw_val_unit(16, temp, 1, 'C', &frac);
        app_draw_val_unit(32, (humi <= -32768) ? -32768 : humi / 10, 0, '%', 0);
        app_draw_val_unit(48, (l == 0xFFu) ? -32768 : (int32_t)l, 0, '%', 0);
        Screen_Flush();
    }
}

/**
  * @brief  SETTINGS 设置模式：清档确认（NO 默认防误触）
  * @note   标题/提示进入时画一次；选择行与清档结果按需刷新
  */
static void app_draw_settings(void)
{
    if (s_lastDrawnState != APP_STATE_SETTINGS)
    {
        Screen_Clear();
        Screen_DrawString8x16(4, 0, "CLEAR SAVE?");
        Screen_DrawString8x16(4, 48, "<LONG BACK");
        s_lastSel = 0xFF;            /* 强制首帧画选择行 */
        s_lastDrawnState = APP_STATE_SETTINGS;
        Screen_Flush();
    }

    if (s_settingsCleared)
    {
        if (!s_clearedDrawn)
        {
            s_clearedDrawn = 1;
            Screen_DrawString8x16(0, 16, "                ");  /* 擦选择行 */
            Screen_DrawString8x16(32, 32, "CLEARED!");        /* 居中（8ch） */
            Screen_Flush();
        }
        return;
    }

    if (s_settingsSel != s_lastSel)
    {
        s_lastSel = s_settingsSel;
        Screen_DrawString8x16(0, 16, "                ");  /* 擦旧选择行（16 空格=128px） */
        if (s_settingsSel == 0u)
        {
            Screen_DrawString8x16(0, 16, ">NO");
            Screen_DrawString8x16(56, 16, "YES");
        }
        else
        {
            Screen_DrawString8x16(8, 16, "NO");
            Screen_DrawString8x16(48, 16, ">YES");
        }
        Screen_Flush();
    }
}

/**
  * @brief  ABOUT 关于模式：静态信息页（进入画一次，静止帧 0 传输）
  */
static void app_draw_about(void)
{
    if (s_lastDrawnState == APP_STATE_ABOUT)
    {
        return;   /* 已画过 */
    }
    Screen_Clear();
    Screen_DrawString8x16(20, 0, "C8PLAY V2.0");
    Screen_DrawString8x16(12, 16, "STM32F103C8T6");
    Screen_DrawString8x16(4, 32, "OLED ENC IR MPU");
    Screen_DrawString8x16(12, 48, "W25Q64 DHT11");
    Screen_Flush();
    s_lastDrawnState = APP_STATE_ABOUT;
}

/* ---------------- 主入口 ---------------- */

/**
  * @brief  应用层每帧处理：收齐本帧事件 -> 状态机 -> 渲染
  * @note   增量绘制：菜单每次移动只刷 16 字节量级；模式态只在进入时
  *         画一次；静止画面 0 次 I2C 传输（screen.c 脏列区间局部刷新）
  */
void App_Process(void)
{
    InputEvent ev;

    /* 0. 总线异常冻结期：暂停游戏推进（不消费事件、不跑 Tick、不绘制），
     *    丢弃冻结期间积压的输入（避免恢复时挡板/菜单瞬移），只降频重试刷屏。
     *    总线恢复（Flush 成功）后自动回到正常流程，从原状态继续——
     *    解决"画面刷不出但游戏继续跑、恢复时已输"的问题。
     *    冻结期每 ~100ms 才重试一次（osDelay 拉长周期）：总线被物理拉死时
     *    每 33ms 的无效重试只会徒耗总线，降频减少压力、加快总线自愈。
     *    升级（卡死根治）：HAL I2C 偶发失败可能让外设卡 BUSY（重试必失败、
     *    永久冻结）——每 500ms 调 Screen_BusReset() 复位 I2C 外设重建总线；
     *    冻结超过 2.5s 仍未恢复 → NVIC_SystemReset() 软件复位自愈（等效
     *    看门狗，复位后重新初始化，屏幕重绘，不再需要手动重新烧录） */
    if (Screen_IsStalled())
    {
        uint32_t now = osKernelGetTickCount();

        Input_ClearEvents();
        if (s_freezeTick == 0)
        {
            s_freezeTick = now;
            s_busResetTick = now;
        }
        if ((now - s_busResetTick) >= 500u)
        {
            s_busResetTick = now;
            Screen_BusReset();      /* 复位 I2C1 外设：清 BUSY/错误状态，总线自愈关键 */
        }
        if ((now - s_freezeTick) >= 2500u)
        {
            NVIC_SystemReset();     /* 冻结超时兜底：软件复位，重新初始化屏幕 */
        }
        Screen_Flush();
        osDelay(70);
        return;
    }
    s_freezeTick = 0;   /* 总线正常：清零冻结计时 */

    /* 1. 非阻塞收齐本帧所有事件（队列空则立即返回） */
    while (Input_GetEvent(&ev, 0))
    {
        app_handle_event(&ev);
    }

    /* 2. 渲染当前状态（增量绘制：仅变化区域入脏） */
    if (g_state == APP_STATE_MENU)
    {
        app_draw_menu();    /* 内部按"整屏/滚动/光标"三档增量绘制 */
        s_lastDrawnState = APP_STATE_MENU;   /* 更新快照：再次进入同一模式时能正确重画占位页 */
    }
    else if (g_state == APP_STATE_TETRIS)
    {
        Tetris_Tick();      /* 俄罗斯方块：每帧渲染（后续加入下落节拍） */
        app_check_record(SAVE_GAME_TETRIS, Tetris_IsGameOver(), Tetris_GetScore());
    }
    else if (g_state == APP_STATE_SNAKE)
    {
        Snake_Tick();       /* 贪吃蛇：节拍移动 + 增量渲染 */
        app_check_record(SAVE_GAME_SNAKE, Snake_IsGameOver(), Snake_GetScore());
    }
    else if (g_state == APP_STATE_BREAKOUT)
    {
        Breakout_Tick();    /* 打砖块：节拍移动 + 分轴碰撞 + 增量渲染 */
        app_check_record(SAVE_GAME_BREAKOUT, Breakout_IsGameOver(), Breakout_GetScore());
    }
    else if (g_state == APP_STATE_DESKTOP)
    {
        app_draw_desktop(); /* 环境页：数值变化才刷新（静止 0 传输） */
    }
    else if (g_state == APP_STATE_SETTINGS)
    {
        app_draw_settings();/* 设置页：选择/清档结果按需刷新 */
    }
    else if (g_state == APP_STATE_ABOUT)
    {
        app_draw_about();   /* 关于页：进入画一次，之后静止 */
    }
    else if (g_state != s_lastDrawnState)
    {
        app_draw_placeholder();   /* 兜底：未接入状态显示"开发中" */
        s_lastDrawnState = g_state;
    }
    /* 模式态内画面无变化：不绘制不刷新 */

    g_frameCount++;
}

/* ---------------- 串口协议命令分发 ----------------
 * protoTask 上下文调用：解析后的完整帧在此分发执行 + 回应答帧。
 * 应答帧格式：DATA[0]=错误码（0=OK），后跟数据 */

/**
  * @brief  协议游戏控制码 → 输入事件注入（0x06 命令）
  * @param  key  1=右 2=左 3=确认/旋转 4=硬降/加速 5=返回/暂停
  * @retval 0 = OK；PROTO_ERR_PARAM = 非法按键码
  * @note   注入的事件与编码器/红外完全等价，游戏任务无感知来源
  */
static uint8_t app_proto_game_ctrl(uint8_t key)
{
    switch (key)
    {
    case 1: Input_PostEvent(INPUT_EV_MOVE, 1);       break;   /* 右 */
    case 2: Input_PostEvent(INPUT_EV_MOVE, -1);      break;   /* 左 */
    case 3: Input_PostEvent(INPUT_EV_CLICK, 0);      break;   /* 确认/旋转 */
    case 4: Input_PostEvent(INPUT_EV_KEY, 1);        break;   /* 硬降/加速 */
    case 5: Input_PostEvent(INPUT_EV_LONG_PRESS, 0); break;   /* 返回/暂停 */
    default: return PROTO_ERR_PARAM;
    }
    return PROTO_ERR_OK;
}

void App_HandleProtoFrame(const ProtoFrame *frame)
{
    uint8_t resp[PROTO_DATA_MAX];
    uint8_t n = 1;

    switch (frame->cmd)
    {
    case PROTO_CMD_GET_INFO:   /* 读设备信息 */
        resp[0] = PROTO_ERR_OK;
        resp[1] = 'C'; resp[2] = '8'; resp[3] = 'P'; resp[4] = 'L';
        resp[5] = PROTO_FW_VERSION;      /* 固件版本 v0.4（阶段4） */
        resp[6] = PROTO_PROTO_VERSION;   /* 协议版本 v1 */
        n = 7;
        break;

    case PROTO_CMD_GET_HISCORE:   /* 读最高分：uint16 LE × 3（俄/蛇/砖） */
        resp[0] = PROTO_ERR_OK;
        resp[1] = (uint8_t)(Save_GetHi(SAVE_GAME_TETRIS) & 0xFFu);
        resp[2] = (uint8_t)(((uint32_t)Save_GetHi(SAVE_GAME_TETRIS) >> 8) & 0xFFu);
        resp[3] = (uint8_t)(Save_GetHi(SAVE_GAME_SNAKE) & 0xFFu);
        resp[4] = (uint8_t)(((uint32_t)Save_GetHi(SAVE_GAME_SNAKE) >> 8) & 0xFFu);
        resp[5] = (uint8_t)(Save_GetHi(SAVE_GAME_BREAKOUT) & 0xFFu);
        resp[6] = (uint8_t)(((uint32_t)Save_GetHi(SAVE_GAME_BREAKOUT) >> 8) & 0xFFu);
        n = 7;
        break;

    case PROTO_CMD_CLEAR_SAVE:   /* 清存档（危险命令） */
        Save_ClearAll();
        resp[0] = PROTO_ERR_OK;
        n = 1;
        break;

    case PROTO_CMD_SET_MODE:   /* 切换模式：0=菜单 1=俄 2=蛇 3=砖
                                * 不直接改应用状态：投递事件由 gameTask 统一消费（防跨任务竞态） */
        if (frame->len >= 1 && frame->data[0] <= APP_STATE_BREAKOUT)
        {
            Input_PostEvent(INPUT_EV_PROTO_SET_MODE, frame->data[0]);
            resp[0] = PROTO_ERR_OK;
        }
        else
        {
            resp[0] = PROTO_ERR_PARAM;
        }
        n = 1;
        break;

    case PROTO_CMD_GAME_CTRL:   /* 游戏控制：按键事件注入 */
        if (frame->len >= 1)
        {
            resp[0] = app_proto_game_ctrl(frame->data[0]);
        }
        else
        {
            resp[0] = PROTO_ERR_PARAM;
        }
        n = 1;
        break;

    case PROTO_CMD_MEM_INFO:   /* 任务内存统计（阶段5）：打印各任务栈高水位文本 */
        Mem_DumpTasks();       /* 文本走寄存器级 UART，持发送互斥，与应答帧不交错 */
        resp[0] = PROTO_ERR_OK;
        n = 1;
        break;

    default:
        resp[0] = PROTO_ERR_CMD;   /* 未知命令 */
        n = 1;
        break;
    }

    Proto_Send(PROTO_CMD_RESPONSE, resp, n);
}
