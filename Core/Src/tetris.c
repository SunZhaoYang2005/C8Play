/**
  ******************************************************************************
  * @file    tetris.c
  * @brief   俄罗斯方块
  *          已实现：数据模型、7 方块表、碰撞检测(can_place)、左右移动、
  *                 旋转(查表+墙踢)、硬降、软降(按住K1)、下落节拍(等级加速)、
  *                 锁底、消行计分升级、出块、Game Over、Ghost 落点阴影
  *          渲染：增量绘制（擦旧画新 + 按需重画面板/棋盘），
  *                配合 screen.c 脏列局部刷新，静止帧 0 次 I2C 传输
  * 按键映射  MOVE 左右 / CLICK 旋转 / KEY 点按=硬降 / K1 按住=软降 /
  *          长按返回菜单(app.c 处理)；Game Over 后任意 CLICK 重开
  * 铁律     先 tetris_can_place() 试算碰撞，通过才真正移动/旋转
  * 屏幕映射  board(row,col) -> 像素 (1+col*4, 1+(row-5)*4)，
  *          仅 row>=5 可见（顶部 5 行出生缓冲）
  * 布局     游戏区边框 DrawRect(0,0,42,62) 与格子像素不重叠
  *          （可视 15 行；增量擦除永不破坏边框）
  * 渲染     增量绘制：擦旧画新 + 消行帧整棋盘区"先擦后画"
  *          （只画不擦会留消行残影）；I2C 400kHz
  ******************************************************************************
  */

#include "tetris.h"
#include "screen.h"
#include "cmsis_os.h"   /* osKernelGetTickCount：下落节拍计时 */

/* ---------------- 数据模型 ---------------- */

/* 游戏网格：1=已锁定方块（20 行 x 10 列，全部在 RAM） */
static uint8_t board[TETRIS_ROWS][TETRIS_COLS];

/* 当前方块状态 */
static uint8_t curShape = 0;    /* 形状索引 0..6 */
static uint8_t curRot   = 0;    /* 旋转态 0..3 */
static int8_t  curX     = 3;    /* 在 board 中的列坐标 0..9 */
static int8_t  curY     = 0;    /* 在 board 中的行坐标 0..19 */
static uint8_t nextShape = 1;   /* 下一个方块（NEXT 预览） */

static int32_t score = 0;       /* 分数 */
static uint8_t level = 1;       /* 等级：决定下落速度，每 10 行升 1 级 */
static uint8_t linesCleared = 0;/* 累计消行数 */

static uint8_t  s_gameOver = 0;     /* 1 = 游戏结束（出生位被占） */
static uint32_t s_lastDropTick = 0; /* 上次下落时刻（tick=1ms，节拍用） */
static int32_t  s_hiScore = 0;      /* 历史最高分（app.c 进入游戏时注入，Game Over 屏显示 HI） */

/* ---------------- 增量绘制快照 ----------------
 * 记录"上一帧渲染了什么"，本帧只重画变化部分：
 *  - s_lastPieceXxx：上一帧方块位置，用于"擦旧画新"
 *  - s_lastGhostY：上一帧 Ghost 落点行（空心轮廓，与实块分开擦/画）
 *  - s_lastScore/Level/Lines/NextShape：面板按需重画（值变了才画）
 *  - s_boardDirty：锁底/消行后棋盘内容变化，下一帧需重画棋盘区
 *  - s_firstFrame / s_lastGameOver：整屏重画只做一次（进入 / Game Over）
 * 配合 screen.c 脏列局部刷新：无变化的帧 Flush 0 次 I2C 传输 */
static uint8_t  s_firstFrame = 1;
static uint8_t  s_boardDirty = 1;
static uint8_t  s_lastPieceShape = 0;
static uint8_t  s_lastPieceRot   = 0;
static int8_t   s_lastPieceX     = 0;
static int8_t   s_lastPieceY     = 0;
static int8_t   s_lastGhostY     = 0;   /* 上一帧 Ghost 落点行 */
static int32_t  s_lastScore      = -1;   /* -1 = 未画过，强制首画 */
static uint8_t  s_lastLevel      = 0xFF;
static uint8_t  s_lastLines      = 0xFF;
static uint8_t  s_lastNextShape  = 0xFF;
static uint8_t  s_lastGameOver   = 0;

/* 7 种方块 x 4 旋转态 x 4 行；每行低 4 位，bit3=最左列、bit0=最右列。
 * 判断第 col 列是否有方块：row & (0x08 >> col)
 * 形状顺序：I / O / T / S / Z / L / J（与输入无关，内部固定） */
static const uint8_t shapes[7][4][4] = {
    /* I */
    { {0x00, 0x0F, 0x00, 0x00},
      {0x04, 0x04, 0x04, 0x04},
      {0x00, 0x0F, 0x00, 0x00},
      {0x04, 0x04, 0x04, 0x04} },
    /* O */
    { {0x00, 0x06, 0x06, 0x00},
      {0x00, 0x06, 0x06, 0x00},
      {0x00, 0x06, 0x06, 0x00},
      {0x00, 0x06, 0x06, 0x00} },
    /* T */
    { {0x00, 0x04, 0x0E, 0x00},
      {0x00, 0x04, 0x06, 0x04},
      {0x00, 0x00, 0x0E, 0x04},
      {0x00, 0x04, 0x0C, 0x04} },
    /* S */
    { {0x00, 0x06, 0x0C, 0x00},
      {0x00, 0x04, 0x06, 0x02},
      {0x00, 0x06, 0x0C, 0x00},
      {0x00, 0x04, 0x06, 0x02} },
    /* Z */
    { {0x00, 0x0C, 0x06, 0x00},
      {0x00, 0x02, 0x06, 0x04},
      {0x00, 0x0C, 0x06, 0x00},
      {0x00, 0x02, 0x06, 0x04} },
    /* L */
    { {0x00, 0x02, 0x02, 0x06},
      {0x00, 0x00, 0x0E, 0x08},
      {0x00, 0x06, 0x02, 0x02},
      {0x00, 0x02, 0x0E, 0x00} },
    /* J */
    { {0x00, 0x04, 0x04, 0x06},
      {0x00, 0x00, 0x0E, 0x02},
      {0x00, 0x06, 0x04, 0x04},
      {0x00, 0x08, 0x0E, 0x00} },
};

/* ---------------- 游戏逻辑（第二部分） ---------------- */

/**
  * @brief  碰撞检测（全游戏铁律：先试算，通过才真正移动/旋转）
  * @param  shape/rot/x/y  拟放置的方块参数（本函数不修改任何全局状态）
  * @retval 1 = 可放置；0 = 左右/底部出界，或撞到已锁定方块
  * @note   只读不改：board 和方块表都不会被本函数修改，试算失败无副作用
  */
static int tetris_can_place(int8_t shape, int8_t rot, int8_t x, int8_t y)
{
    uint8_t r, c;
    for (r = 0; r < 4; r++)
    {
        for (c = 0; c < 4; c++)
        {
            if (shapes[shape][rot][r] & (0x08u >> c))
            {
                int8_t bx = (int8_t)(x + c);   /* 本格在 board 中的列 */
                int8_t by = (int8_t)(y + r);   /* 本格在 board 中的行 */
                if (bx < 0 || bx >= TETRIS_COLS || by >= TETRIS_ROWS)
                {
                    return 0;                  /* 左右或底部出界 */
                }
                if (by >= 0 && board[by][bx])
                {
                    return 0;                  /* 撞到已锁定方块（by<0 在板外，不查） */
                }
            }
        }
    }
    return 1;
}

/**
  * @brief  左右移动：先试算碰撞，通过才更新 curX（失败则原地不动）
  */
static void tetris_try_move(int8_t dx)
{
    if (tetris_can_place(curShape, curRot, (int8_t)(curX + dx), curY))
    {
        curX = (int8_t)(curX + dx);
    }
}

/**
  * @brief  旋转：查表切到下一旋转态；贴墙时依次尝试墙踢（x-1 / x+1）
  * @note   三种位置都失败则保持原样（被夹住不转），不会破坏棋盘
  */
static void tetris_try_rotate(void)
{
    uint8_t newRot = (uint8_t)((curRot + 1) % 4);

    if (tetris_can_place(curShape, newRot, curX, curY))
    {
        curRot = newRot;                            /* 原位可转 */
    }
    else if (tetris_can_place(curShape, newRot, (int8_t)(curX - 1), curY))
    {
        curRot = newRot;                            /* 左踢 1 格 */
        curX   = (int8_t)(curX - 1);
    }
    else if (tetris_can_place(curShape, newRot, (int8_t)(curX + 1), curY))
    {
        curRot = newRot;                            /* 右踢 1 格 */
        curX   = (int8_t)(curX + 1);
    }
}

/**
  * @brief  消行：从下往上扫满行，把整行上方所有块下移一行，顶部补空行
  *         计分：一次消 1/2/3/4 行 = 100/300/500/800 分 x 等级
  *         升级：每累计 10 行升 1 级（level 决定下落速度）
  */
static void tetris_clear_lines(void)
{
    static const uint16_t kScore[4] = { 100, 300, 500, 800 };
    uint8_t r, c, rr;
    uint8_t lines = 0;

    for (r = TETRIS_ROWS - 1; r >= TETRIS_TOP_HIDE; r--)
    {
        /* 检查第 r 行是否满 */
        uint8_t full = 1;
        for (c = 0; c < TETRIS_COLS; c++)
        {
            if (!board[r][c])
            {
                full = 0;
                break;
            }
        }
        if (full)
        {
            /* 上方所有行整体下移一行 */
            for (rr = r; rr > TETRIS_TOP_HIDE; rr--)
            {
                for (c = 0; c < TETRIS_COLS; c++)
                {
                    board[rr][c] = board[rr - 1][c];
                }
            }
            /* 顶部补一行空（TETRIS_TOP_HIDE 是可视区第 0 行） */
            for (c = 0; c < TETRIS_COLS; c++)
            {
                board[TETRIS_TOP_HIDE][c] = 0;
            }
            lines++;
            r++;   /* 原地重查：下移后本行来了新内容，可能又是满行 */
        }
    }

    if (lines > 0)
    {
        score += (int32_t)kScore[lines - 1] * (int32_t)level;
        linesCleared += lines;
        if (linesCleared / 10 >= level)    /* 每 10 行升 1 级 */
        {
            level++;
        }
    }
}

/**
  * @brief  出下一个方块：当前 = next，next 简易随机（tick 取模）
  * @note   出生位（curX=3, curY=0）已被占用 = Game Over
  */
static void tetris_spawn_next(void)
{
    curShape  = nextShape;
    nextShape = (uint8_t)(osKernelGetTickCount() % 7);   /* 简易随机，够用 */
    curRot    = 0;
    curX      = 3;
    curY      = 0;

    if (!tetris_can_place(curShape, curRot, curX, curY))
    {
        s_gameOver = 1;   /* 出生即被占：顶到天了 */
    }
}

/**
  * @brief  锁底：当前方块写入 board -> 消行计分 -> 出下一个方块
  * @note   由"下落触底"或"硬降"调用；锁底后 s_gameOver 可能被置 1；
  *         s_boardDirty=1 通知渲染下一帧重画棋盘区（锁入的格子要显示）
  */
static void tetris_lock_piece(void)
{
    uint8_t r, c;
    for (r = 0; r < 4; r++)
    {
        for (c = 0; c < 4; c++)
        {
            if (shapes[curShape][curRot][r] & (0x08u >> c))
            {
                int8_t by = (int8_t)(curY + r);
                int8_t bx = (int8_t)(curX + c);
                if (by >= 0)              /* 防御：只写板内格 */
                {
                    board[by][bx] = 1;
                }
            }
        }
    }
    s_boardDirty = 1;                    /* 棋盘内容变了（含消行），需重画 */
    tetris_clear_lines();
    tetris_spawn_next();
}

/**
  * @brief  硬降：一路下落到底并立即锁底（KEY 触发）
  */
static void tetris_hard_drop(void)
{
    while (tetris_can_place(curShape, curRot, curX, (int8_t)(curY + 1)))
    {
        curY++;
    }
    tetris_lock_piece();
    s_lastDropTick = osKernelGetTickCount();   /* 新方块重新计时 */
}

/**
  * @brief  下落节拍：等级 1 为 500ms，每升 1 级 -50ms，下限 100ms
  * @note   configTICK_RATE_HZ=1000，tick 差值即毫秒
  */
static uint32_t tetris_drop_interval(void)
{
    int32_t t = 500 - (int32_t)(level - 1) * 50;
    if (t < 100)
    {
        t = 100;
    }
    return (uint32_t)t;
}

/* ---------------- 渲染（增量绘制：擦旧画新 + 按需重画） ---------------- */

/**
  * @brief  画一个格子（4x4 像素）：board 坐标 (col,row) -> 屏幕像素
  *         布局：可视 15 行，格子起点 (1,1)，游戏区边框 (0,0,42,62)
  *         与格子像素不重叠（增量擦除永不破坏边框）
  * @note   出生缓冲区的行（row < TETRIS_TOP_HIDE）不显示
  */
static void tetris_draw_cell(int8_t col, int8_t row, uint8_t on)
{
    if (row < TETRIS_TOP_HIDE)
    {
        return;
    }
    Screen_FillRect((uint8_t)(1 + col * 4), (uint8_t)(1 + (row - TETRIS_TOP_HIDE) * 4), 4, 4, on);
}

/**
  * @brief  画所有已锁定在 board 上的方块（从第 TETRIS_TOP_HIDE 行开始扫）
  * @note   只画不擦：适用于首帧（已 Clear）等"已知内容全新"的场景；
  *         消行后必须用 tetris_redraw_board（先擦后画）
  */
static void tetris_draw_board(void)
{
    uint8_t r, c;
    for (r = TETRIS_TOP_HIDE; r < TETRIS_ROWS; r++)
    {
        for (c = 0; c < TETRIS_COLS; c++)
        {
            if (board[r][c])
            {
                tetris_draw_cell((int8_t)c, (int8_t)r, 1);
            }
        }
    }
}

/**
  * @brief  刷新整个棋盘格子区：先擦除格子区（40x60px，边框内侧），
  *         再全量重画 board==1 的格子
  * @note   只画不擦是消行残影的根因：消行/整列下移后，board==0
  *         的位置必须显式擦除，否则旧格子像素残留在屏幕上
  *         （"满行不消除"即此现象）。I2C 400kHz 下重画 40 字节约 1ms，锁底帧内可承受
  */
static void tetris_redraw_board(void)
{
    Screen_FillRect(1, 1, 40, 60, 0);   /* 擦整个格子区（不碰边框） */
    tetris_draw_board();
}

/**
  * @brief  按参数画/擦一个方块（增量绘制核心：擦旧画新共用）
  * @param  shape/rot/x/y  方块状态（快照或当前值）
  * @param  on  1=画实心格，0=擦除（画空格覆盖旧像素）
  */
static void tetris_draw_piece_ex(uint8_t shape, uint8_t rot, int8_t x, int8_t y, uint8_t on)
{
    uint8_t r, c;
    for (r = 0; r < 4; r++)
    {
        for (c = 0; c < 4; c++)
        {
            if (shapes[shape][rot][r] & (0x08u >> c))
            {
                tetris_draw_cell((int8_t)(x + c), (int8_t)(y + r), on);
            }
        }
    }
}

/**
  * @brief  画当前方块
  */
static void tetris_draw_piece(void)
{
    tetris_draw_piece_ex(curShape, curRot, curX, curY, 1);
}

/**
  * @brief  擦上一帧的方块（用快照位置画空格）
  * @note   方块与 board 永不重叠（can_place 保证），擦旧不会误伤棋盘；
  *         锁底帧的格子由 s_boardDirty 触发的棋盘重画恢复
  */
static void tetris_erase_piece(void)
{
    tetris_draw_piece_ex(s_lastPieceShape, s_lastPieceRot, s_lastPieceX, s_lastPieceY, 0);
}

/* ---------------- Ghost 落点阴影 ----------------
 * 显示当前方块将落到哪一行：用 can_place 从 curY 一路试算到最后可放位置，
 * 画成"空心 4x4 格子"（单色 OLED 下与实心块区分）。落点只读不写 board，
 * 与实块/棋盘永不重叠（can_place 保证），可安全擦旧画新。 */

/**
  * @brief  计算 Ghost 落点行：当前方块从 curY 一直下落到最后可放位置
  * @retval 落点行号（board 坐标）
  */
static int8_t tetris_ghost_y(void)
{
    int8_t gy = curY;
    while (tetris_can_place(curShape, curRot, curX, (int8_t)(gy + 1)))
    {
        gy++;
    }
    return gy;
}

/**
  * @brief  画/擦一个 Ghost 空心格（4x4 边框，与实心格区分）
  * @note   空心格内部不填充；与实块同位置时由实块覆盖（先画 ghost 再画实块）
  */
static void tetris_draw_ghost_cell(int8_t col, int8_t row, uint8_t on)
{
    if (row < TETRIS_TOP_HIDE)
    {
        return;
    }
    Screen_DrawRect((uint8_t)(1 + col * 4), (uint8_t)(1 + (row - TETRIS_TOP_HIDE) * 4), 4, 4, on);
}

/**
  * @brief  按参数画/擦整个 Ghost（快照与当前值共用）
  */
static void tetris_draw_ghost_ex(uint8_t shape, uint8_t rot, int8_t x, int8_t y, uint8_t on)
{
    uint8_t r, c;
    for (r = 0; r < 4; r++)
    {
        for (c = 0; c < 4; c++)
        {
            if (shapes[shape][rot][r] & (0x08u >> c))
            {
                tetris_draw_ghost_cell((int8_t)(x + c), (int8_t)(y + r), on);
            }
        }
    }
}

/**
  * @brief  擦上一帧的 Ghost（快照位置画 0 边框）
  */
static void tetris_erase_ghost(void)
{
    tetris_draw_ghost_ex(s_lastPieceShape, s_lastPieceRot, s_lastPieceX, s_lastGhostY, 0);
}

/**
  * @brief  画当前 Ghost（重新计算落点）
  */
static void tetris_draw_ghost(void)
{
    tetris_draw_ghost_ex(curShape, curRot, curX, tetris_ghost_y(), 1);
}

/**
  * @brief  画右侧信息面板（增量绘制：值变了才重画，force=1 全画）
  *         布局：行0 SCORE|LEVEL 行1 数值|数值 行2 NEXT|LINES 行3 预览|数值
  * @note   标题行/边框只在 force（整屏重画）时画；
  *         数值/预览项与快照比对，变了才擦旧画新（每项 2 列内，I2C 开销极小）；
  *         无变化的帧整段跳过，不给 Flush 增加脏列
  */
static void tetris_draw_panel(uint8_t force)
{
    uint8_t r, c;

    /* 标题行与预览边框：只在整屏重画时画
     * 面板两栏：SCORE/LINES 栏 x=46，LEVEL/LINES 栏 x=88
     * （游戏区含边框占 0..41，面板从 46 起） */
    if (force)
    {
        Screen_DrawString8x16(46, 0, "SCORE");
        Screen_DrawString8x16(88, 0, "LEVEL");
        Screen_DrawString8x16(46, 32, "NEXT");
        Screen_DrawString8x16(88, 32, "LINES");
        Screen_DrawRect(46, 48, 16, 16, 1);
    }

    /* SCORE 数值（5 字符宽 40px，擦旧后重画，避免新旧数字叠加） */
    if (force || score != s_lastScore)
    {
        Screen_FillRect(46, 16, 40, 16, 0);
        Screen_PrintSignedVal(46, 16, score);
        s_lastScore = score;
    }

    /* LEVEL 数值 */
    if (force || level != s_lastLevel)
    {
        Screen_FillRect(88, 16, 24, 16, 0);
        Screen_PrintSignedVal(88, 16, (int32_t)level);
        s_lastLevel = level;
    }

    /* NEXT 预览（4x4 格 x 3px = 12x12px，落在 16x16 边框内圈居中，
     * 擦除区 12px 不碰边框 -> 边框永不破坏） */
    if (force || nextShape != s_lastNextShape)
    {
        Screen_FillRect(48, 50, 12, 12, 0);
        for (r = 0; r < 4; r++)
        {
            for (c = 0; c < 4; c++)
            {
                if (shapes[nextShape][0][r] & (0x08u >> c))
                {
                    Screen_FillRect((uint8_t)(48 + c * 3), (uint8_t)(50 + r * 3), 3, 3, 1);
                }
            }
        }
        s_lastNextShape = nextShape;
    }

    /* LINES 数值 */
    if (force || linesCleared != s_lastLines)
    {
        Screen_FillRect(88, 48, 24, 16, 0);
        Screen_PrintSignedVal(88, 48, (int32_t)linesCleared);
        s_lastLines = linesCleared;
    }
}

/* ---------------- 对外接口 ---------------- */

/**
  * @brief  重置游戏（进入 TETRIS 模式时由 app.c 调用，Game Over 重开也走这里）
  */
void Tetris_Init(void)
{
    uint8_t r, c;
    for (r = 0; r < TETRIS_ROWS; r++)
    {
        for (c = 0; c < TETRIS_COLS; c++)
        {
            board[r][c] = 0;
        }
    }
    curShape  = 0;      /* 固定 I 方块开局，便于验证 */
    curRot    = 0;
    curX      = 3;
    curY      = 0;
    nextShape = 1;
    score     = 0;
    level     = 1;
    linesCleared = 0;
    s_gameOver = 0;
    s_lastDropTick = osKernelGetTickCount();

    /* 重置增量绘制快照：整屏重画一次，面板全部强制首画 */
    s_firstFrame     = 1;
    s_boardDirty     = 1;
    s_lastPieceShape = 0;
    s_lastPieceRot   = 0;
    s_lastPieceX     = 0;
    s_lastPieceY     = 0;
    s_lastGhostY     = 0;
    s_lastScore      = -1;
    s_lastLevel      = 0xFF;
    s_lastLines      = 0xFF;
    s_lastNextShape  = 0xFF;
    s_lastGameOver   = 0;
}

/**
  * @brief  处理输入事件：
  *         MOVE 左右 / CLICK 旋转 / KEY 点按=硬降 / K1 按住=软降 /
  *         Game Over 后 CLICK 重开
  * @note   LONG_PRESS 返回菜单由 app.c 统一处理，本函数不消费
  */
void Tetris_HandleEvent(const InputEvent *ev)
{
    /* Game Over 态：CLICK 重开；LONG_PRESS 返回菜单由 app.c 统一处理
     * （本函数不消费长按，直接 return 放行给应用层） */
    if (s_gameOver)
    {
        if (ev->type == INPUT_EV_CLICK)
        {
            Tetris_Init();
        }
        return;
    }

    switch (ev->type)
    {
    case INPUT_EV_MOVE:
        /* 编码器：value>0 右移 1 格，value<0 左移 1 格 */
        tetris_try_move((int8_t)((ev->value > 0) ? 1 : -1));
        break;

    case INPUT_EV_CLICK:
        tetris_try_rotate();          /* 短按 = 旋转 */
        break;

    case INPUT_EV_KEY:
        tetris_hard_drop();           /* K1 快速点按(<300ms) = 硬降 */
        break;

    case INPUT_EV_KEY_HOLD:
        /* K1 按住（input 层每 100ms 投递一次）= 软降：加速下落一格。
         * 与节拍触底一致：落到底则锁底（消行+出块）；
         * 每格都重置节拍基准，软降期间以 100ms 节拍取代正常下落节拍 */
        if (tetris_can_place(curShape, curRot, curX, (int8_t)(curY + 1)))
        {
            curY++;
        }
        else
        {
            tetris_lock_piece();
        }
        s_lastDropTick = osKernelGetTickCount();
        break;

    case INPUT_EV_KEY_RELEASE:
        /* K1 松开：软降结束。重置节拍基准，避免松开瞬间新方块立即下落
         * （快速点按也会先收到 KEY_RELEASE，重置无害——硬降已重置过） */
        s_lastDropTick = osKernelGetTickCount();
        break;

    default:
        /* 不直接消费的类型：INPUT_EV_IR / INPUT_EV_TILT / INPUT_EV_LONG_PRESS。
         * 注意：红外遥控器在 app.c 层已翻译为标准语义（←/→=移动、+=旋转、
         * -=硬降、Return=返回菜单），进到这里的是翻译后事件，红外有效；
         * 真正未接入的是 TILT（体感倾斜，input_event 已产生事件但无映射） */
        break;
    }
}

/**
  * @brief  每帧调用：下落节拍 + 增量渲染
  * @note   流程：Game Over 只画一次结算 -> 节拍到点尝试下移一行，
  *         触底则锁底(消行+出块) -> 增量渲染：
  *         首帧整屏重画；平时"擦旧方块 + 棋盘按脏重画 + 画新方块 +
  *         面板按需重画"，Flush 只发脏列。
  *         性能：方块移动帧仅刷 2~4 列 x 2 页（<2ms I2C）；
  *         静止帧（方块未动、无消行）0 次 I2C 传输；
  *         锁底/消行帧重画棋盘区 40 字节（约 4ms），都在 33ms 帧预算内
  */
void Tetris_Tick(void)
{
    uint32_t now = osKernelGetTickCount();
    uint8_t pieceChanged;   /* 本帧方块是否变化（增量绘制擦旧画新的开关） */

    /* Game Over：整屏结算画面只画一次，之后静止 0 传输，等 CLICK 重开
     * 布局与贪吃蛇/打砖块统一：左右两列各自居中（中心 x=32 / x=96），
     * 8x16 字模高 16px，行距 0/16/32/48。
     * HI 显示 max(进入本局前的纪录, 本局分数)：破纪录时结算屏立刻展示新纪录 */
    if (s_gameOver)
    {
        if (!s_lastGameOver)
        {
            int32_t hi = (score > s_hiScore) ? score : s_hiScore;

            Screen_Clear();
            Screen_DrawString8x16(28, 0, "GAME OVER");            /* 9 字符 72px，全屏居中 */
            Screen_DrawString8x16(12, 16, "SCORE");               /* 左列标签（中心 32） */
            Screen_PrintSignedValCentered(32, 32, score);         /* 左列数值 */
            Screen_DrawString8x16(88, 16, "HI");                  /* 右列标签（中心 96） */
            Screen_PrintSignedValCentered(96, 32, hi);            /* 右列数值 */
            Screen_Flush();
            s_lastGameOver = 1;
        }
        return;
    }
    s_lastGameOver = 0;

    /* 下落节拍：距上次下落 >= 间隔，且下方可放才下移一行 */
    if ((now - s_lastDropTick) >= tetris_drop_interval())
    {
        if (tetris_can_place(curShape, curRot, curX, (int8_t)(curY + 1)))
        {
            curY++;                          /* 下方空：下移一行 */
        }
        else
        {
            tetris_lock_piece();             /* 触底：锁底+消行+出块（置 s_boardDirty） */
        }
        s_lastDropTick = now;                /* 重设节拍（新方块重新计时） */
    }

    /* 刚在本帧锁底触发 Game Over：不渲染（新块与 board 重叠），
     * 下一帧进入结算分支整屏重画 */
    if (s_gameOver)
    {
        return;
    }

    /* ===== 增量渲染 ===== */
    if (s_firstFrame)
    {
        /* 整屏重画：棋盘 + 当前块 + 全部面板 + 游戏区完整边框
         * （进入游戏/重开时一次）
         * 布局：可视 15 行，格子起点 (1,1)，边框 (0,0,42,62)，
         * 边框与格子像素不重叠 -> 增量擦除永不破坏边框 */
        Screen_Clear();
        tetris_draw_board();
        tetris_draw_ghost();           /* Ghost 先画，实块后画可覆盖贴底重叠 */
        tetris_draw_piece();
        tetris_draw_panel(1);
        Screen_DrawRect(0, 0, 42, 62, 1);
        Screen_Flush();
        s_firstFrame = 0;
    }
    else
    {
        /* 方块位置/形态是否有变化（静止帧整段跳过 -> 0 绘制 0 传输） */
        pieceChanged = (s_lastPieceShape != curShape || s_lastPieceRot != curRot ||
                        s_lastPieceX != curX || s_lastPieceY != curY);
        if (pieceChanged)
        {
            tetris_erase_ghost();        /* 擦旧 Ghost */
            tetris_erase_piece();          /* 擦旧方块（快照位置画空格） */
        }

        /* 棋盘变化（锁底/消行）时：先擦整个格子区再全量重画。
         * 只画不擦 -> 消行后残影不消失（"满行不消除"） */
        if (s_boardDirty)
        {
            tetris_redraw_board();
            s_boardDirty = 0;
        }

        if (pieceChanged)
        {
            tetris_draw_ghost();         /* 画新 Ghost（先画，实块后画覆盖重叠） */
            tetris_draw_piece();           /* 画新方块（在 board 之上） */
        }

        /* 面板按需重画（内部与快照比对，没变就不画） */
        tetris_draw_panel(0);

        Screen_Flush();      /* 只发脏列：本帧变化区域 */
    }

    /* 记录快照：下一帧以此为"旧状态"做擦旧画新 */
    s_lastPieceShape = curShape;
    s_lastPieceRot   = curRot;
    s_lastPieceX     = curX;
    s_lastPieceY     = curY;
    s_lastGhostY     = tetris_ghost_y();   /* Ghost 落点行快照 */
}

/* ---------------- 存档接入 ---------------- */

int32_t Tetris_GetScore(void)
{
    return score;
}

uint8_t Tetris_IsGameOver(void)
{
    return s_gameOver;
}

void Tetris_SetHiScore(int32_t hi)
{
    s_hiScore = hi;
}
