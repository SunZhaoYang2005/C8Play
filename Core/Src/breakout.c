/**
  ******************************************************************************
  * @file    breakout.c
  * @brief   打砖块
  *          已实现：砖块位图 3x20、挡板(4格=16px)、球 像素坐标（每帧平滑移动）、
  *                 分轴碰撞（先 X 后 Y，撞砖/壁只反射该轴，先判后动无副作用，
  *                 步长 ≤2px < 格宽 4px 逐帧判定不穿模）、
  *                 挡板命中位置决定角度档（四档：快斜±2 / 慢斜±1，永不带垂直档）、
  *                 发球慢斜 45° 起步（好接球；角度靠挡板反弹调节）、
  *                 3 球机会、通关 CLEAR 屏 + CLICK 下一关（重排砖块）、
  *                 K1 暂停、Game Over 结算 + CLICK 重开
  *          渲染：增量绘制（球/挡板擦旧画新、消砖只擦该格、分数/心形变化入脏），
  *                配合 screen.c 脏列局部刷新，静止帧 0 次 I2C 传输
  *          外观：挡板/球/命 采用韦东山 N|Watch 打砖块位图样式——
  *                挡板 16x8 圆头位图画屏幕底部、球 2x2 位图（球底贴挡板顶行）、
  *                命 7x8 心形 ×3 右上角；
  *                游戏区边框只画上/左/右三边（底边让给挡板活动区，擦挡板不破坏边框）
  * 按键映射  MOVE 挡板左右 / CLICK 发球·通关继续 / KEY K1=暂停/继续 /
  *          LONG_PRESS 不消费（返回 0 → app.c 返回菜单）
  * 铁律     分轴碰撞：每帧先 X 后 Y；撞砖/壁反射该轴方向，
  *          反射时球不移动（停在障碍前），下一帧沿反方向自然离开，
  *          永不穿模、永不重复撞同一块；挡板碰撞按"球底行 == 挡板顶行"判定
  * 屏幕映射  cell(row,col) -> 像素 (25+col*4, 17+row*4)，顶部 16px HUD 行
  * 速度      分数驱动提速：累计消砖 10 块 → 20px/s、20 块 → 30px/s（上限），
  *          消砖当下生效、跨关延续；开局 15px/s（每 2 帧动 1px）。
  *          X 步长：慢斜 ±1 / 快斜 ±2
  * 布局     游戏区 20x11 格 = 80x44 像素，边框 DrawRect(24,16,82,46)
  *          （格子从 (25,17) 起，边框线在格子区外扩 1px）
  ******************************************************************************
  */

#include "breakout.h"
#include "screen.h"
#include "cmsis_os.h"   /* osKernelGetTickCount：节拍计时 */
#include <stdlib.h>     /* rand()：发球随机斜向 */

/* ---------------- 布局常量 ---------------- */

#define GRID_X0  25      /* 游戏区格子左上角 x（格子从 25 起，边框线在 24，不重叠） */
#define GRID_Y0  17      /* 游戏区格子左上角 y（格子从 17 起，边框线在 16，不重叠） */
#define CELL     4       /* 每格像素（与俄罗斯方块/贪吃蛇一致） */
#define GRID_W   (BRICK_COLS * CELL)   /* 80 px */
#define GRID_H   ((PADDLE_ROW + 1) * CELL)   /* 11 行 * 4 = 44 px（挡板行也算） */
/* 边框画在格子区外扩 1px：DrawRect(24,16,82,46)，格子与边框线完全错开 */

#define PADDLE_Y_PX   (SCREEN_HEIGHT - 8)   /* 挡板画在屏幕底部 8px（同韦东山 g_yres-8） */
#define LIVES_ORIGIN  3                     /* 初始球数=3（右上角画 3 颗心形） */

/* 球为像素坐标（每帧平滑移动，无步进跳变） */
#define BALL_PX      2    /* 球 2x2 位图像素宽高 */
#define BALL_SLOW    1    /* 慢斜 X 步长（px/帧）：与 Y 同幅 = 45° */
#define BALL_FAST    2    /* 快斜 X 步长（px/帧）：更平 */
#define PADDLE_TOP_PX (GRID_Y0 + PADDLE_ROW * CELL)   /* 挡板行顶像素 y = 57 */

/* 挡板 16x8 圆头位图（上宽圆润、下窄收口），
 * 16px 宽与 4 格碰撞一致；可见像素在 bit0..3（画在 y=56 时可见区 y=56..59，
 * 与贴底球的底行(y=56)齐平，球挡板无缝衔接） */
static const uint8_t kPlatformImg[] = {
    0x04, 0x07, 0x05, 0x01, 0x01, 0x05, 0x0F, 0x0F,
    0x0F, 0x0F, 0x05, 0x01, 0x01, 0x07, 0x05, 0x04
};
/* 球 2x2 位图 */
static const uint8_t kBallImg[] = { 0x03, 0x03 };
/* 命 7x8 心形位图 */
static const uint8_t kLivesImg[] = { 0x06, 0x0F, 0x1F, 0x3E, 0x1F, 0x0F, 0x06 };

/* 游戏状态机 */
typedef enum {
    BO_READY    = 0,   /* 待发球：球贴挡板，MOVE 移挡板球跟随，CLICK 发球 */
    BO_PLAYING,        /* 游戏中：球按节拍自动移动 + 分轴碰撞 */
    BO_CLEAR,          /* 通关：显示 CLEAR 屏，CLICK 进入下一关 */
    BO_GAMEOVER        /* 结算：3 球用完，CLICK 重开 */
} BO_State;

/* ---------------- 数据模型 ---------------- */

static uint8_t  s_bricks[BRICK_ROWS][BRICK_COLS];  /* 砖块位图：1=存在 */
static uint8_t  s_paddleX;    /* 挡板左端格坐标 0..BRICK_COLS-PADDLE_W */
static int16_t  s_ballPx;     /* 球左上角像素 x（GRID_X0 .. GRID_X0+GRID_W-BALL_PX） */
static int16_t  s_ballPy;     /* 球左上角像素 y */
static int8_t   s_ballVxPx;   /* 每帧 X 步长：慢斜±BALL_SLOW / 快斜±BALL_FAST（px/帧） */
static int8_t   s_ballVyPx;   /* 每帧 Y 步长：±s_speed（1 起步，L3 起 2） */
static uint8_t  s_lives;      /* 剩余球数 3..0 */
static int32_t  s_score;      /* 分数（每消一块 +10） */
static uint8_t  s_level;      /* 关卡（通关 +1，提速） */
static uint8_t  s_speed;      /* 速度步长（px/帧）：1 起步，L3 起 2（上限） */
static uint8_t  s_moveDivider;/* 移动分频：L1=2（每 2 帧动 1px ≈ 15px/s 极慢开局），L2 起=1 */
static uint8_t  s_moveAccum;  /* 分频累积计数器 */
static uint8_t  s_state;      /* BO_State */
static uint8_t  s_paused;     /* 1 = 暂停（K1 进入，CLICK/K1 继续） */
static int32_t  s_hiScore;    /* 历史最高分（app.c 进入游戏时注入，Game Over 屏显示 HI） */

/* ---------------- 增量绘制快照 ---------------- */
static uint8_t  s_firstFrame = 1;        /* 整屏重画（进入/暂停恢复/换关） */
static uint8_t  s_goForceRedraw = 0;     /* Game Over 屏只画一次 */
static uint8_t  s_clearForceRedraw = 0;  /* CLEAR 屏只画一次 */
static uint8_t  s_pauseForceRedraw = 0;  /* 暂停屏只画一次 */
static uint8_t  s_lastPaddleX = 0;
static int16_t  s_lastBallPx = 0;
static int16_t  s_lastBallPy = 0;
static int32_t  s_lastScore = -1;
static uint8_t  s_lastLives = 0;

/* ---------------- 内部函数 ---------------- */

/** 重置砖块位图：3 行满砖（60 块） */
static void breakout_reset_bricks(void)
{
    uint8_t r, c;
    for (r = 0; r < BRICK_ROWS; r++)
    {
        for (c = 0; c < BRICK_COLS; c++)
        {
            s_bricks[r][c] = 1;
        }
    }
}

/** 画砖块格（on=1 点亮 / on=0 擦除） */
static void breakout_draw_brick(uint8_t row, uint8_t col, uint8_t on)
{
    Screen_FillRect((uint8_t)(GRID_X0 + col * CELL),
                    (uint8_t)(GRID_Y0 + row * CELL), CELL, CELL, on);
}

/** 画挡板（韦东山 16x8 圆头位图，画屏幕底部；on=1 点亮 / on=0 擦除） */
static void breakout_draw_paddle(uint8_t on)
{
    Screen_DrawBitmap((uint8_t)(GRID_X0 + s_paddleX * CELL),
                      PADDLE_Y_PX, 16, 8, kPlatformImg, on);
}

/** 画球（韦东山 2x2 位图，球底贴挡板顶行——无空行无缝衔接；
 *  指定像素坐标版——增量擦除必须用快照旧坐标） */
static void breakout_draw_ball_at(int16_t px, int16_t py, uint8_t on)
{
    Screen_DrawBitmap((uint8_t)px, (uint8_t)py, BALL_PX, BALL_PX, kBallImg, on);
}

/** 画球（当前位置版） */
static void breakout_draw_ball(uint8_t on)
{
    breakout_draw_ball_at(s_ballPx, s_ballPy, on);
}

/** 画 HUD（分数左上 / 命右上角 3 颗心形，韦东山样式）；只重画有变化的项 */
static void breakout_draw_hud(void)
{
    if (s_score != s_lastScore)
    {
        Screen_DrawString8x16(8, 0, "        ");   /* 擦旧分数 */
        Screen_PrintSignedVal(8, 0, s_score);
        s_lastScore = s_score;
    }
    if (s_lives != s_lastLives)
    {
        uint8_t i;
        for (i = 0; i < LIVES_ORIGIN; i++)
        {
            /* 右上角从右往左排 3 颗心（x=104/112/120，间距 8px），
             * i < lives 画心，否则画空（擦除该颗心） */
            uint8_t hx = (uint8_t)(SCREEN_WIDTH - 24u + 8u * i);
            Screen_DrawBitmap(hx, 1, 7, 8, kLivesImg, (i < s_lives) ? 1u : 0u);
        }
        s_lastLives = s_lives;
    }
}

/** 挡板反弹：垂直反射 + 命中位置决定水平方向与角度档。
 *  @note 四档角度映射（快斜/慢斜），永不给垂直档（vx=0）：
 *        格子模型里垂直球撞左右壁反射后仍垂直（-0=0），一旦垂直就永远直上直下。
 *        命中越靠边缘角度越平（快斜 ±BALL_FAST），越靠中间越陡（慢斜 ±BALL_SLOW） */
static void breakout_paddle_bounce(void)
{
    int16_t left = (int16_t)(GRID_X0 + s_paddleX * CELL);
    int16_t center = (int16_t)(s_ballPx + BALL_PX / 2);      /* 球中心 */
    int8_t off = (int8_t)((center - left) / CELL);           /* 0..PADDLE_W-1 */

    s_ballVyPx = (int8_t)-s_ballVyPx;
    if (off == 0)
    {
        s_ballVxPx = -BALL_FAST;   /* 左缘：快左斜 */
    }
    else if (off == 1)
    {
        s_ballVxPx = -BALL_SLOW;   /* 左半：慢左斜 */
    }
    else if (off == 2)
    {
        s_ballVxPx = BALL_SLOW;    /* 右半：慢右斜 */
    }
    else
    {
        s_ballVxPx = BALL_FAST;    /* 右缘：快右斜 */
    }
}

/** 球回到待发位置（贴挡板中心上方，球底贴挡板顶行；READY 跟随/失球/开局共用） */
static void breakout_place_ball_ready(void)
{
    s_ballPx = (int16_t)(GRID_X0 + s_paddleX * CELL + (PADDLE_W * CELL - BALL_PX) / 2);
    s_ballPy = (int16_t)(PADDLE_Y_PX - BALL_PX + 1);   /* 球底行 = 挡板顶行（无缝） */
    s_ballVxPx = 0;
    s_ballVyPx = 0;
}

/** 分数驱动提速：累计消砖 10 块 → 20px/s、20 块 → 30px/s（上限）。
 *  消砖当下立即生效（更新飞行中球的 Y 速度幅值，方向不变）；
 *  通关不额外提速，分数跨关延续。开局 15px/s（每 2 帧动 1px）。
 *  档位刻意平缓（15→20→30）：用户反馈 30px/s 已偏快，上限不再加档 */
static void breakout_update_speed(void)
{
    uint8_t blocks = (uint8_t)(s_score / 10u);   /* 累计消砖数 */
    uint8_t newSpeed, newDivider;

    if (blocks >= 20u)
    {
        newSpeed = 1; newDivider = 1;   /* 30px/s：每帧 1px（最平滑），上限 */
    }
    else if (blocks >= 10u)
    {
        newSpeed = 2; newDivider = 3;   /* 20px/s：每 3 帧动 2px */
    }
    else
    {
        newSpeed = 1; newDivider = 2;   /* 15px/s：每 2 帧动 1px */
    }

    if (newSpeed != s_speed || newDivider != s_moveDivider)
    {
        s_speed = newSpeed;
        s_moveDivider = newDivider;
        /* 同步飞行中球的速度幅值（保持方向） */
        if (s_ballVyPx > 0)
        {
            s_ballVyPx = (int8_t)s_speed;
        }
        else if (s_ballVyPx < 0)
        {
            s_ballVyPx = (int8_t)-s_speed;
        }
    }
}

/** 失球：扣一条命；0 命 → Game Over，否则回到待发球（球贴挡板） */
static void breakout_lose_ball(void)
{
    s_lives--;
    if (s_lives == 0)
    {
        s_state = BO_GAMEOVER;
        s_goForceRedraw = 1;
        return;
    }
    s_state = BO_READY;
    breakout_place_ball_ready();
}

/** 通关检查：砖块位图全 0 → CLEAR 态 */
static void breakout_check_clear(void)
{
    uint8_t r, c;
    for (r = 0; r < BRICK_ROWS; r++)
    {
        for (c = 0; c < BRICK_COLS; c++)
        {
            if (s_bricks[r][c])
            {
                return;
            }
        }
    }
    s_state = BO_CLEAR;
    s_clearForceRedraw = 1;
}

/**
  * @brief  核心移动：像素级分轴碰撞（先 X 后 Y），每帧（33ms）平滑移动
  * @note   铁律：先判后动。撞壁/撞砖 → 只反射该轴方向，球停在障碍前，
  *         下一帧沿反方向离开——永不穿模、永不重复撞同一块砖。
  *         步长 ≤2px < 格宽 4px，逐帧判定可靠，无跳变穿格；
  *         挡板碰撞按"球底行 == 挡板顶行"精确判定（球底贴挡板，无缝衔接）
  */
static void breakout_move(void)
{
    /* ---------- X 轴先走（vx∈{-2,-1,1,2} px/帧，慢斜±1 / 快斜±2） ---------- */
    if (s_ballVxPx != 0)
    {
        int16_t nx = (int16_t)(s_ballPx + s_ballVxPx);
        if (nx < GRID_X0)
        {
            s_ballVxPx = (int8_t)-s_ballVxPx;            /* 撞左壁：反射 */
            s_ballPx = GRID_X0;
        }
        else if (nx > (int16_t)(GRID_X0 + GRID_W - BALL_PX))
        {
            s_ballVxPx = (int8_t)-s_ballVxPx;            /* 撞右壁：反射 */
            s_ballPx = (int16_t)(GRID_X0 + GRID_W - BALL_PX);
        }
        else
        {
            int8_t row = (int8_t)((s_ballPy - GRID_Y0) / CELL);
            int8_t col = (int8_t)((nx - GRID_X0) / CELL);
            if (row < (int8_t)BRICK_ROWS && s_bricks[row][col])
            {
                s_bricks[row][col] = 0;                  /* 撞砖：消砖 + 反射 + 停 */
                breakout_draw_brick((uint8_t)row, (uint8_t)col, 0);
                s_score += 10;
                breakout_update_speed();                 /* 消砖即按分数提速 */
                s_ballVxPx = (int8_t)-s_ballVxPx;
            }
            else
            {
                s_ballPx = nx;
            }
        }
    }

    /* ---------- Y 轴再走 ---------- */
    {
        int16_t ny = (int16_t)(s_ballPy + s_ballVyPx);
        int16_t bottom;                                  /* 球底行（球占 py..py+1） */
        if (ny < GRID_Y0)
        {
            s_ballVyPx = (int8_t)-s_ballVyPx;            /* 撞顶：反射 */
            s_ballPy = GRID_Y0;
        }
        else
        {
            int8_t col = (int8_t)((s_ballPx - GRID_X0) / CELL);
            int8_t row = (int8_t)((ny - GRID_Y0) / CELL);

            if (row < (int8_t)BRICK_ROWS && s_bricks[row][col])
            {
                s_bricks[row][col] = 0;                  /* 撞砖：消砖 + 反射 + 停 */
                breakout_draw_brick((uint8_t)row, (uint8_t)col, 0);
                s_score += 10;
                breakout_update_speed();                 /* 消砖即按分数提速 */
                s_ballVyPx = (int8_t)-s_ballVyPx;
            }
            else
            {
                bottom = (int16_t)(ny + BALL_PX - 1);    /* 球底像素行 */
                if (bottom >= (int16_t)PADDLE_Y_PX &&
                    bottom <= (int16_t)(PADDLE_Y_PX + 2))
                {
                    /* 球底到达挡板顶行（3 行宽容）：判是否命中挡板。
                     * 横向判定左右各 +1px 宽容：球擦挡板边缘 1px 也算接住 */
                    int16_t left = (int16_t)(GRID_X0 + s_paddleX * CELL);
                    if (s_ballPx + BALL_PX > (int16_t)(left - 1) &&
                        s_ballPx < (int16_t)(left + PADDLE_W * CELL + 1))
                    {
                        breakout_paddle_bounce();        /* 命中：反弹 + 定方向 */
                        s_ballPy = (int16_t)(PADDLE_Y_PX - BALL_PX + 1);  /* 贴挡板顶 */
                    }
                    else
                    {
                        breakout_lose_ball();            /* 球越过挡板边缘：失球 */
                        return;
                    }
                }
                else if (bottom > (int16_t)(PADDLE_Y_PX + 2))
                {
                    breakout_lose_ball();                /* 已深过挡板行：落底 */
                    return;
                }
                else
                {
                    s_ballPy = ny;   /* 空中正常飞行 */
                }
            }
        }
    }

    breakout_check_clear();
}

/* ---------------- 渲染 ---------------- */

/** 增量绘制：球/挡板擦旧画新 + 消砖已即时擦除 + HUD 变化入脏 */
static void breakout_draw(void)
{
    if (s_firstFrame)
    {
        /* 整屏重画：三边边框 + 全部砖块 + 挡板 + 球（进入游戏 / 暂停恢复 / 换关） */
        uint8_t r, c;
        Screen_Clear();
        s_lastScore = -1;          /* 强制 HUD 首画（分数左上 + 心形右上） */
        s_lastLives = 0xFF;
        /* 边框只画上/左/右三边：底边整行是挡板活动区（画了会被擦挡板破坏）；
         * 挡板 x=25..104 与左右边框线（x=24 / x=105）不重叠，擦除安全 */
        Screen_FillRect((uint8_t)(GRID_X0 - 1), (uint8_t)(GRID_Y0 - 1),
                        (uint8_t)(GRID_W + 2), 1, 1);                 /* 顶边 */
        Screen_FillRect((uint8_t)(GRID_X0 - 1), (uint8_t)(GRID_Y0 - 1),
                        1, (uint8_t)(GRID_H + 1), 1);                 /* 左边 */
        Screen_FillRect((uint8_t)(GRID_X0 + GRID_W), (uint8_t)(GRID_Y0 - 1),
                        1, (uint8_t)(GRID_H + 1), 1);                 /* 右边 */
        for (r = 0; r < BRICK_ROWS; r++)
        {
            for (c = 0; c < BRICK_COLS; c++)
            {
                if (s_bricks[r][c])
                {
                    breakout_draw_brick(r, c, 1);
                }
            }
        }
        breakout_draw_paddle(1);
        breakout_draw_ball(1);
        s_firstFrame = 0;
    }
    else
    {
        uint8_t ballMoved   = (s_lastBallPx != s_ballPx || s_lastBallPy != s_ballPy);
        uint8_t paddleMoved = (s_lastPaddleX != s_paddleX);

        /* 球移动：擦旧（快照像素坐标）画新（当前像素坐标）（含 READY 态跟随挡板） */
        if (ballMoved)
        {
            breakout_draw_ball_at(s_lastBallPx, s_lastBallPy, 0);
            breakout_draw_ball_at(s_ballPx, s_ballPy, 1);
        }
        /* 挡板移动：擦旧（位图形状，透明区不动） */
        if (paddleMoved)
        {
            Screen_DrawBitmap((uint8_t)(GRID_X0 + s_lastPaddleX * CELL),
                              PADDLE_Y_PX, 16, 8, kPlatformImg, 0);
        }
        /* 球或挡板任一移动时最后重画挡板：球贴底（底行 y=56）与挡板顶行（y=56）
         * 共享像素行，球擦旧会破坏挡板顶行；挡板最后画可恢复（只点亮形状像素，
         * 不影响球与边框） */
        if (ballMoved || paddleMoved)
        {
            breakout_draw_paddle(1);
        }
    }

    breakout_draw_hud();

    s_lastPaddleX = s_paddleX;
    s_lastBallPx = s_ballPx;
    s_lastBallPy = s_ballPy;

    Screen_Flush();
}

/** Game Over 结算屏（整屏，只画一次）
 * 布局与俄罗斯方块/贪吃蛇统一：左右两列各自居中（中心 x=32 / x=96），
 * HI 显示 max(进入本局前的纪录, 本局分数)：破纪录时结算屏立刻展示新纪录 */
static void breakout_draw_gameover(void)
{
    int32_t hi = (s_score > s_hiScore) ? s_score : s_hiScore;

    Screen_Clear();
    Screen_DrawString8x16(28, 0, "GAME OVER");            /* 9 字符 72px，全屏居中 */
    Screen_DrawString8x16(12, 16, "SCORE");               /* 左列标签（中心 32） */
    Screen_PrintSignedValCentered(32, 32, s_score);       /* 左列数值 */
    Screen_DrawString8x16(88, 16, "HI");                  /* 右列标签（中心 96） */
    Screen_PrintSignedValCentered(96, 32, hi);            /* 右列数值 */
    Screen_DrawString8x16(12, 48, "CLICK RESTART");       /* 13 字符 104px，全屏居中 */
    Screen_Flush();
}

/** 通关屏（整屏，只画一次） */
static void breakout_draw_clear(void)
{
    Screen_Clear();
    Screen_DrawString8x16(44, 0, "CLEAR!");
    Screen_DrawString8x16(40, 16, "LEVEL");
    Screen_PrintSignedVal(44, 32, (int32_t)s_level);
    Screen_DrawString8x16(12, 48, "CLICK NEXT");
    Screen_Flush();
}

/** 暂停屏（整屏，只画一次） */
static void breakout_draw_pause(void)
{
    Screen_Clear();
    Screen_DrawString8x16(40, 0, "PAUSED");
    Screen_DrawString8x16(8, 24, "K1/CLICK RESUME");
    Screen_DrawString8x16(16, 48, "LONG= BACK");
    Screen_Flush();
}

/* ---------------- 对外接口 ---------------- */

/** 重置游戏（进入 BREAKOUT 模式 / Game Over 后 CLICK 重开） */
void Breakout_Init(void)
{
    s_level = 1;
    s_score = 0;
    s_lives = 3;
    s_paddleX = (uint8_t)((BRICK_COLS - PADDLE_W) / 2);   /* 挡板居中 = 8 */
    s_speed = 1;              /* 初始速度步长：Y 每帧 1px */
    s_moveDivider = 2;        /* L1 每 2 帧动 1px（≈15px/s，极慢好接） */
    s_moveAccum = 0;
    breakout_place_ball_ready();                          /* 球贴挡板中心上方待发 */
    s_state = BO_READY;
    s_paused = 0;

    breakout_reset_bricks();

    /* 渲染快照复位 */
    s_firstFrame = 1;
    s_goForceRedraw = 0;
    s_clearForceRedraw = 0;
    s_pauseForceRedraw = 0;
    s_lastPaddleX = s_paddleX;
    s_lastBallPx = s_ballPx;
    s_lastBallPy = s_ballPy;
    s_lastScore = -1;
    s_lastLives = 0xFF;
}

/**
  * @brief  处理输入事件
  * @retval 0 = 未消费 LONG_PRESS（app.c 返回菜单）；打砖块无长按占用需求
  */
int Breakout_HandleEvent(const InputEvent *ev)
{
    if (s_state == BO_GAMEOVER)
    {
        if (ev->type == INPUT_EV_CLICK)
        {
            Breakout_Init();   /* 结算后短按重开 */
        }
        return 0;
    }

    if (s_paused)
    {
        if (ev->type == INPUT_EV_CLICK || ev->type == INPUT_EV_KEY)
        {
            s_paused = 0;
            s_firstFrame = 1;   /* 暂停时清屏了，恢复需整屏重画 */
        }
        return 0;
    }

    switch (ev->type)
    {
    case INPUT_EV_MOVE:
    case INPUT_EV_TILT:   /* MPU6050 倾斜：右倾/左倾 = 挡板右移/左移（与编码器同语义） */
        if (ev->value > 0)
        {
            if (s_paddleX < (uint8_t)(BRICK_COLS - PADDLE_W))
            {
                s_paddleX++;
            }
        }
        else if (ev->value < 0)
        {
            if (s_paddleX > 0)
            {
                s_paddleX--;
            }
        }
        if (s_state == BO_READY)
        {
            breakout_place_ball_ready();   /* 待发球：球跟随挡板（贴中心上方） */
        }
        break;

    case INPUT_EV_CLICK:
        if (s_state == BO_READY)
        {
            /* 发球：45° 起步（X 与 Y 同幅，好接球）；角度靠挡板反弹调节 */
            s_state = BO_PLAYING;
            s_ballVxPx = (rand() & 1) ? (int8_t)s_speed : (int8_t)-s_speed;
            s_ballVyPx = (int8_t)-s_speed;
            s_moveAccum = 0;   /* 发球后立即开始移动计数 */
        }
        else if (s_state == BO_CLEAR)
        {
            /* 通关 → 下一关：重排砖块（速度由累计消砖数驱动，跨关延续） */
            s_level++;
            breakout_reset_bricks();
            s_state = BO_READY;
            breakout_place_ball_ready();
            s_moveAccum = 0;
            s_firstFrame = 1;   /* 新关整屏重画（砖块排布变了） */
        }
        break;

    case INPUT_EV_KEY:
        if (s_state == BO_READY || s_state == BO_PLAYING)
        {
            s_paused = 1;
            s_pauseForceRedraw = 1;
        }
        break;

    case INPUT_EV_LONG_PRESS:
        return 0;   /* 不消费 → app.c 返回菜单 */

    default:
        break;
    }
    return 0;
}

/** 每帧调用（gameTask 33ms）：像素级平滑移动 + 增量渲染 */
void Breakout_Tick(void)
{
    if (s_state == BO_GAMEOVER)
    {
        if (s_goForceRedraw)
        {
            breakout_draw_gameover();
            s_goForceRedraw = 0;
        }
        return;
    }

    if (s_state == BO_CLEAR)
    {
        if (s_clearForceRedraw)
        {
            breakout_draw_clear();
            s_clearForceRedraw = 0;
        }
        return;
    }

    if (s_paused)
    {
        if (s_pauseForceRedraw)
        {
            breakout_draw_pause();
            s_pauseForceRedraw = 0;
        }
        return;
    }

    if (s_state == BO_PLAYING)
    {
        /* 移动分频：L1 每 2 帧移动 1px（≈15px/s），L2 起每帧移动（30px/s+） */
        if (++s_moveAccum >= s_moveDivider)
        {
            s_moveAccum = 0;
            breakout_move();
        }
    }

    breakout_draw();
}

/* ---------------- 存档接入 ---------------- */

int32_t Breakout_GetScore(void)
{
    return s_score;
}

uint8_t Breakout_IsGameOver(void)
{
    return (s_state == BO_GAMEOVER) ? 1u : 0u;
}

void Breakout_SetHiScore(int32_t hi)
{
    s_hiScore = hi;
}
