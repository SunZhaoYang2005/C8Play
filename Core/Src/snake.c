/**
  ******************************************************************************
  * @file    snake.c
  * @brief   贪吃蛇
  *          已实现：环形数组蛇身(头进尾出 O(1))、先试算后移动、撞墙/撞身判定、
  *                 转向带锁(每步最多转一次，防快速旋转180°掉头撞脖子)、吃食物计分提速、
  *                 短按加速一拍 / 长按持续加速(ENC_HOLD) / 松开恢复节拍、
  *                 K1 暂停、Game Over 结算 + CLICK 重开
  *          渲染：增量绘制（移动帧只擦旧尾格 + 画新头格 + 食物/分数变化入脏），
  *                配合 screen.c 脏列局部刷新，静止帧 0 次 I2C 传输
  * 按键映射  MOVE 转向(带锁) / CLICK 短按=加速一拍 / ENC_HOLD 长按=持续加速(每~100ms一拍) /
  *          ENC_RELEASE 松开恢复节拍 / KEY K1=暂停/继续 / IR_HOLD 红外长按同持续加速
  *          LONG_PRESS：进行中吞掉(返回1)，暂停/结算态不消费(返回0 → app.c 返回菜单)
  * 铁律     先判定后执行：移动前完整检查撞墙+撞身，失败不修改任何状态
  * 屏幕映射  cell(row,col) -> 像素 (25+col*4, 17+row*4)，顶部 16px 为标题/分数行
  * 布局     游戏区 20x11 格 = 80x44 像素，边框 DrawRect(24,16,82,46)
  *          （格子从 (25,17) 起，边框线在格子区外扩 1px，与俄罗斯方块
  *           同策略：格子像素与边框不重叠，增量擦除永不破坏墙壁）
  ******************************************************************************
  */

#include "snake.h"
#include "screen.h"
#include "cmsis_os.h"   /* osKernelGetTickCount：节拍计时 */

/* ---------------- 布局常量 ---------------- */

#define GRID_X0  25      /* 游戏区格子左上角 x（格子从 25 起，边框线在 24，不重叠） */
#define GRID_Y0  17      /* 游戏区格子左上角 y（格子从 17 起，边框线在 16，不重叠） */
#define CELL     4       /* 每格像素（与俄罗斯方块一致） */
#define GRID_W   (SNAKE_COLS * CELL)   /* 80 px */
#define GRID_H   (SNAKE_ROWS * CELL)   /* 44 px */
/* 边框画在格子区外扩 1px：DrawRect(GRID_X0-1, GRID_Y0-1, GRID_W+2, GRID_H+2)
 * 即 (24,16,82,46)，格子区域与边框线完全错开，擦蛇格不会破坏墙 */

/* 方向 */
#define SNAKE_DIR_UP    0
#define SNAKE_DIR_RIGHT 1
#define SNAKE_DIR_DOWN  2
#define SNAKE_DIR_LEFT  3

/* ---------------- 数据模型 ---------------- */

/* 蛇身环形数组：存格子序号 cell = row*20+col（0..239，uint8_t 足够）。
 * head/tail 为环形索引，移动 = head 前进一格写新头、tail 前进一格释放旧尾（O(1)）。 */
static uint8_t s_body[SNAKE_MAX_LEN];
static uint8_t s_head;        /* 头索引（body[s_head] = 当前头格） */
static uint8_t s_tail;        /* 尾索引（body[s_tail] = 当前尾格） */
static uint8_t s_len;         /* 当前长度 3..SNAKE_MAX_LEN */
static uint8_t s_headCell;    /* 头格 cell（渲染用，避免反复索引数组） */
static uint8_t s_dir;         /* 方向 0上/1右/2下/3左 */

static uint8_t  s_food;       /* 食物格 cell */
static int32_t  s_score;      /* 分数（每食物 +10） */
static uint8_t  s_foodEaten;  /* 累计吃食物数（驱动提速） */
static uint16_t s_moveInterval; /* 移动节拍 ms：400 起步，每 5 个食物 -30，下限 120 */

static uint8_t  s_gameOver;     /* 1 = 撞墙/撞身 */
static uint8_t  s_paused;       /* 1 = 暂停（K1 进入，CLICK/K1 继续） */
static uint8_t  s_turnLock;     /* 1 = 本步已转向，移动前忽略后续转向（防快速旋转 180° 自杀） */
static uint32_t s_lastMoveTick; /* 上次移动时刻（tick=1ms，节拍用） */
static uint8_t  s_boostActive;    /* 1 = 持续加速中（屏蔽左右转向，防长按加速时误转撞墙） */
static uint32_t s_boostExpireTick; /* 加速屏蔽到期时刻（红外长按无释放事件，超时自动解除） */
static int32_t  s_hiScore;      /* 历史最高分（app.c 进入游戏时注入，Game Over 屏显示 HI） */

/* 伪随机（LCG，种子由 Init 时注入 tick 打散） */
static uint32_t s_rng = 0x12345678u;

/* ---------------- 增量绘制快照 ----------------
 * 记录"上一帧渲染了什么"，本帧只重画变化部分：
 *  - s_lastTailCell / s_lastHeadCell / s_lastFood：移动/食物变化"擦旧画新"
 *  - s_lastScore：分数变化才重画（-1 = 未画过，强制首画）
 *  - s_firstFrame：进入游戏/从暂停恢复需整屏重画（清屏后所有内容重画）
 *  - s_goForceRedraw / s_pauseForceRedraw：Game Over / 暂停屏只画一次
 * 配合 screen.c 脏列局部刷新：无变化的帧 Flush 0 次 I2C 传输 */
static uint8_t  s_firstFrame = 1;
static uint8_t  s_goForceRedraw = 0;
static uint8_t  s_pauseForceRedraw = 0;
static uint8_t  s_lastTailCell = 0;
static uint8_t  s_lastHeadCell = 0;
static uint8_t  s_lastFood = 0;
static int32_t  s_lastScore = -1;

/* ---------------- 内部函数 ---------------- */

/* 前置声明：snake_step_now 在渲染函数定义之后，需先声明才能调用 */
static void snake_draw(void);
static void snake_draw_gameover(void);

static uint32_t snake_rng(void)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return s_rng;
}

/** 蛇身是否包含该格子（遍历 body[tail..head]，共 s_len 个） */
static int snake_body_contains(uint8_t cell)
{
    uint8_t i = s_tail;
    uint8_t cnt = s_len;
    while (cnt > 0)
    {
        if (s_body[i] == cell)
        {
            return 1;
        }
        i = (uint8_t)((i + 1) % SNAKE_MAX_LEN);
        cnt--;
    }
    return 0;
}

/** 生成新食物：随机位置不与蛇身重叠；64 次随机失败后线性扫描兜底（蛇身近满） */
static void snake_spawn_food(void)
{
    uint8_t tries;
    int32_t cell;
    int32_t total = (int32_t)(SNAKE_COLS * SNAKE_ROWS);

    for (tries = 0; tries < 64u; tries++)
    {
        cell = (int32_t)(snake_rng() % (uint32_t)total);
        if (!snake_body_contains((uint8_t)cell))
        {
            s_food = (uint8_t)cell;
            return;
        }
    }
    for (cell = 0; cell < total; cell++)
    {
        if (!snake_body_contains((uint8_t)cell))
        {
            s_food = (uint8_t)cell;
            return;
        }
    }
    s_food = 0;   /* 网格全满（必胜局面），任意放置 */
}

/**
  * @brief  核心移动：先完整判定（撞墙 + 撞身），通过才"头进尾出"
  * @retval 0 = 撞死；1 = 正常移动；2 = 吃到食物（增长）
  * @note   撞身判定细节：没吃食物时尾格即将移走，允许新头占用；
  *         吃了食物时尾格不动，仍是障碍，必须查。先判后改，失败无副作用。
  */
static int snake_move(void)
{
    int16_t row = (int16_t)(s_headCell / SNAKE_COLS);
    int16_t col = (int16_t)(s_headCell % SNAKE_COLS);
    int32_t newCell;
    uint8_t i, cnt;
    int grow;

    switch (s_dir)
    {
    case SNAKE_DIR_UP:    row--; break;
    case SNAKE_DIR_DOWN:  row++; break;
    case SNAKE_DIR_LEFT:  col--; break;
    default:              col++; break;   /* SNAKE_DIR_RIGHT */
    }

    /* 撞墙 */
    if (row < 0 || row >= SNAKE_ROWS || col < 0 || col >= SNAKE_COLS)
    {
        return 0;
    }

    newCell = (int32_t)(row * SNAKE_COLS + col);
    grow = (newCell == s_food) ? 1 : 0;

    /* 撞自己：跳过"即将移走的尾格"（仅当不增长时） */
    cnt = (uint8_t)(s_len - (grow ? 0u : 1u));
    i = s_tail;
    while (cnt > 0)
    {
        if (s_body[i] == (uint8_t)newCell)
        {
            return 0;
        }
        i = (uint8_t)((i + 1) % SNAKE_MAX_LEN);
        cnt--;
    }

    /* 头进：新头写入下一槽 */
    s_head = (uint8_t)((s_head + 1) % SNAKE_MAX_LEN);
    s_body[s_head] = (uint8_t)newCell;
    s_headCell = (uint8_t)newCell;
    s_turnLock = 0;   /* 移动成功 → 解锁转向（每步最多转向一次） */

    if (grow)
    {
        /* 尾不动 = 增长；计分 + 提速（每 5 个食物 -30ms，下限 120ms）
         * 饱和写法：food/5 超过 13（即 food>=70）直接取 120ms，不进减法——
         * 原 400u - (food/5)*30u 在 food>=70 时无符号回绕成 ~65 秒/步（bug） */
        s_score += 10;
        s_foodEaten++;
        if (s_len < SNAKE_MAX_LEN)
        {
            s_len++;
        }
        if ((s_foodEaten / 5u) > 13u)
        {
            s_moveInterval = 120u;   /* 已到提速下限，饱和（不进入减法，防无符号回绕） */
        }
        else
        {
            s_moveInterval = (uint16_t)(400u - (uint16_t)(s_foodEaten / 5u) * 30u);
            if (s_moveInterval < 120u)
            {
                s_moveInterval = 120u;   /* 下限 clamp 仍需要（n=12/13 时 40/10ms → 120ms） */
            }
        }
        snake_spawn_food();
        return 2;
    }

    /* 尾出：释放旧尾格 */
    s_tail = (uint8_t)((s_tail + 1) % SNAKE_MAX_LEN);
    return 1;
}

/** 转向：±1 = 左右转 90°；返回 1 = 方向已改变，0 = 未转向（无输入/180° 防御）
 * 真正的 180° 掉头防御由调用处的转向锁承担（每步最多转一次，两步 90° 被锁挡）；
 * 本函数内的 180° 判定在绝对方向转向（如未来 TILT 直接给方向）时才有实际意义 */
static int snake_turn(int32_t val)
{
    int8_t  delta;
    uint8_t newDir;

    if (val == 0)
    {
        return 0;                       /* 防御：无方向输入不转向 */
    }
    delta = (val > 0) ? (int8_t)1 : (int8_t)-1;
    newDir = (uint8_t)((s_dir + delta + 4) % 4);   /* s_dir 参与 int 提升，无回绕 */
    if ((s_dir + 2) % 4 == newDir)
    {
        return 0;                       /* 180° 掉头防御（绝对转向场景下才有意义） */
    }
    s_dir = newDir;
    return 1;
}

/** 立即走一步（短按加速一拍 / 长按持续加速一拍）+ 重置节拍基准
 *  铁律：移动后立即渲染（snake_draw），增量快照保持同步——
 *  一帧内多个加速事件（队列积压）时，每步都"移动→渲染"成对，
 *  中间步的蛇头/尾格不会漏画或残留（修复"同帧两步断节"bug） */
static void snake_step_now(void)
{
    s_lastMoveTick = osKernelGetTickCount();
    if (s_gameOver || s_paused)
    {
        return;
    }
    if (!snake_move())
    {
        s_gameOver = 1;
        snake_draw_gameover();   /* 立即画结算屏（清掉强制重画标志，Tick 不再重复画） */
        s_goForceRedraw = 0;
        return;
    }
    snake_draw();                /* 每步立即渲染：擦尾画头 + 快照更新 */
}

/** 松开：重置节拍基准，避免"松手还憋着一跳" */
static void snake_reset_tick(void)
{
    s_lastMoveTick = osKernelGetTickCount();
}

/* ---------------- 渲染 ---------------- */

static void snake_draw_cell(uint8_t cell, uint8_t on)
{
    uint8_t col = cell % SNAKE_COLS;
    uint8_t row = cell / SNAKE_COLS;
    Screen_FillRect((uint8_t)(GRID_X0 + col * CELL),
                    (uint8_t)(GRID_Y0 + row * CELL), CELL, CELL, on);
}

/** 增量绘制：仅"尾格前移 / 食物变化 / 分数变化 / 头移动"入脏 */
static void snake_draw(void)
{
    if (s_firstFrame)
    {
        /* 整屏重画：边框 + 标题 + 全部蛇身 + 食物（进入游戏 / 暂停恢复） */
        uint8_t i = s_tail;
        uint8_t cnt = s_len;
        Screen_Clear();
        Screen_DrawString8x16(8, 0, "SNAKE");
        Screen_DrawRect((uint8_t)(GRID_X0 - 1), (uint8_t)(GRID_Y0 - 1),
                        (uint8_t)(GRID_W + 2), (uint8_t)(GRID_H + 2), 1);
        while (cnt > 0)
        {
            snake_draw_cell(s_body[i], 1);
            i = (uint8_t)((i + 1) % SNAKE_MAX_LEN);
            cnt--;
        }
        snake_draw_cell(s_food, 1);
        s_firstFrame = 0;
    }
    else
    {
        /* 尾格前移：擦旧尾（新头可能落在旧尾格，画头在最后一步，先擦后画顺序安全） */
        if (s_lastTailCell != s_body[s_tail])
        {
            snake_draw_cell(s_lastTailCell, 0);
        }
        /* 食物变化：擦旧画新 */
        if (s_lastFood != s_food)
        {
            snake_draw_cell(s_lastFood, 0);
            snake_draw_cell(s_food, 1);
        }
        /* 头移动：只画新头（旧头格仍是蛇身的一部分，绝不能擦！
         * 擦旧头会让蛇身一节节消失；蛇身缩短只来自"尾格前移"的擦除） */
        if (s_lastHeadCell != s_headCell)
        {
            snake_draw_cell(s_headCell, 1);
        }
    }

    /* 分数变化才重画（8 字符宽空格覆盖旧值后重画） */
    if (s_score != s_lastScore)
    {
        Screen_DrawString8x16(88, 0, "        ");
        Screen_PrintSignedVal(88, 0, s_score);
        s_lastScore = s_score;
    }

    s_lastTailCell = s_body[s_tail];
    s_lastHeadCell = s_headCell;
    s_lastFood = s_food;

    Screen_Flush();
}

/** Game Over 结算屏（整屏，只画一次）
 * 布局与俄罗斯方块/打砖块统一：左右两列各自居中（中心 x=32 / x=96），
 * HI 显示 max(进入本局前的纪录, 本局分数)：破纪录时结算屏立刻展示新纪录 */
static void snake_draw_gameover(void)
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

/** 暂停屏（整屏，只画一次） */
static void snake_draw_pause(void)
{
    Screen_Clear();
    Screen_DrawString8x16(40, 0, "PAUSED");
    Screen_DrawString8x16(8, 24, "K1/CLICK RESUME");
    Screen_DrawString8x16(16, 48, "LONG= BACK");
    Screen_Flush();
}

/* ---------------- 对外接口 ---------------- */

/** 重置游戏（进入 SNAKE 模式 / Game Over 后 CLICK 重开） */
void Snake_Init(void)
{
    s_len = 3;
    s_dir = SNAKE_DIR_RIGHT;

    /* 初始蛇身：row 5（11 行网格中间），col 9/10/11，头在最右 */
    s_tail = 0;
    s_head = 2;
    s_body[0] = (uint8_t)(5 * SNAKE_COLS + 9);
    s_body[1] = (uint8_t)(5 * SNAKE_COLS + 10);
    s_body[2] = (uint8_t)(5 * SNAKE_COLS + 11);
    s_headCell = s_body[2];

    s_score = 0;
    s_foodEaten = 0;
    s_moveInterval = 400u;
    s_gameOver = 0;
    s_paused = 0;
    s_turnLock = 0;
    s_boostActive = 0;
    s_lastMoveTick = osKernelGetTickCount();
    s_rng ^= osKernelGetTickCount();   /* 随机种子打散 */

    snake_spawn_food();

    /* 渲染快照复位 */
    s_firstFrame = 1;
    s_goForceRedraw = 0;
    s_pauseForceRedraw = 0;
    s_lastTailCell = s_body[s_tail];
    s_lastHeadCell = s_headCell;
    s_lastFood = s_food;
    s_lastScore = -1;
}

/**
  * @brief  处理输入事件
  * @retval 1 = 已消费 LONG_PRESS（游戏进行中：长按被持续加速占用，不返回菜单）
  *         0 = 未消费（暂停/结算态：LONG_PRESS 由 app.c 返回菜单）
  */
int Snake_HandleEvent(const InputEvent *ev)
{
    if (s_gameOver)
    {
        if (ev->type == INPUT_EV_CLICK)
        {
            Snake_Init();   /* 结算后短按重开 */
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

    /* 进行中 */
    switch (ev->type)
    {
    case INPUT_EV_MOVE:
        /* 持续加速中屏蔽左右转向（长按加速时误转容易撞墙/自撞） */
        if (!s_boostActive && !s_turnLock && snake_turn(ev->value))
        {
            s_turnLock = 1;
        }
        break;
    case INPUT_EV_CLICK:        snake_step_now(); break;     /* 短按 = 加速一拍（不屏蔽转向） */
    case INPUT_EV_ENC_HOLD:     s_boostActive = 1;
                                s_boostExpireTick = osKernelGetTickCount() + 500u;
                                snake_step_now(); break;     /* 长按 = 持续加速 */
    case INPUT_EV_IR_HOLD:      s_boostActive = 1;
                                s_boostExpireTick = osKernelGetTickCount() + 500u;
                                snake_step_now(); break;     /* 红外长按同持续加速 */
    case INPUT_EV_ENC_RELEASE:  s_boostActive = 0; snake_reset_tick(); break;   /* 松开恢复节拍 */
    case INPUT_EV_KEY:          s_paused = 1; s_pauseForceRedraw = 1; break;  /* K1 = 暂停 */
    case INPUT_EV_LONG_PRESS:   return 1;   /* 吞掉：长按=持续加速，不返回菜单 */
    default: break;
    }
    return 0;
}

/** 每帧调用（gameTask 33ms）：节拍移动 + 增量渲染 */
void Snake_Tick(void)
{
    uint32_t now;

    if (s_gameOver)
    {
        if (s_goForceRedraw)
        {
            snake_draw_gameover();
            s_goForceRedraw = 0;
        }
        return;
    }

    if (s_paused)
    {
        if (s_pauseForceRedraw)
        {
            snake_draw_pause();
            s_pauseForceRedraw = 0;
        }
        return;
    }

    now = osKernelGetTickCount();

    /* 加速屏蔽超时自动解除：红外长按无释放事件，500ms 无新加速即恢复转向 */
    if (s_boostActive && (int32_t)(now - s_boostExpireTick) >= 0)
    {
        s_boostActive = 0;
    }

    if ((now - s_lastMoveTick) >= s_moveInterval)
    {
        s_lastMoveTick = now;
        if (!snake_move())
        {
            s_gameOver = 1;
            s_goForceRedraw = 1;
        }
    }

    snake_draw();
}

/* ---------------- 存档接入 ---------------- */

int32_t Snake_GetScore(void)
{
    return s_score;
}

uint8_t Snake_IsGameOver(void)
{
    return s_gameOver;
}

void Snake_SetHiScore(int32_t hi)
{
    s_hiScore = hi;
}
