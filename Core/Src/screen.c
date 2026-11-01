/**
  ******************************************************************************
  * @file    screen.c
  * @brief   掌机 OLED 渲染引擎（SSD1306 128x64 I2C 帧缓冲）
  *          内存帧缓冲(8页x128列) -> 脏列区间局部刷新(页地址模式定位后写)
  *          字模复用 ascii_font.c 的 8x16 列扫描字模
  * 接线     I2C1: PB6=SCL PB7=SDA，OLED 地址 0x3C(7bit)
  ******************************************************************************
  */

#include "screen.h"
#include "main.h"
#include "i2c.h"
#include "cmsis_os.h"

/* I2C1 总线互斥量（freertos.c RTOS_MUTEX 区创建）：OLED 与 MPU6050 共用 I2C1 */
extern osMutexId_t g_i2cMutex;

/* 复用韦东山字模：128 个字符 x 16 字节（8 列宽 x 16 行高，每字节一列，上8字节为上半部分） */
extern const uint8_t ascii_font[128][16];

/* OLED I2C 地址（7bit 0x3C 左移 1 位 = 8bit 0x78） */
#define OLED_ADDR8   (0x3C << 1)
#define OLED_CTRL_CMD   0x00    /* I2C control byte: 命令 */
#define OLED_CTRL_DATA  0x40    /* I2C control byte: 数据 */

/* 帧缓冲：page(0~7) x col(0~127)，每个 bit 对应一列中的 8 个像素，bit0 为顶部 */
static uint8_t g_fb[SCREEN_HEIGHT / 8][SCREEN_WIDTH];

/* 局部刷新（脏列区间）：每页记录"被改过"的列区间 [lo, hi]。
 * lo > hi 表示该页干净；所有绘制都汇聚到 Screen_DrawPixel，
 * 在那里统一打脏标记，Flush 只刷脏页的脏列，省去整屏 1KB 传输。 */
static uint8_t s_dirty_lo[8] = { 255, 255, 255, 255, 255, 255, 255, 255 };
static uint8_t s_dirty_hi[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

/* Flush 连续失败帧数：整帧刷完=清零；任一页失败=累加。
 * ≥2 帧刷不出去 → 判定总线被外部拖死（Screen_IsStalled 返回 1），
 * 上层暂停游戏推进、只重试刷屏，避免"画面刷不出但游戏继续跑"导致
 * 总线恢复时游戏已输（球已掉光） */
static uint8_t s_flushFailCnt = 0;

/* 将某页的某列标记为脏（合并进已有区间） */
static void mark_dirty(uint8_t page, uint8_t x)
{
    if (x < s_dirty_lo[page])
    {
        s_dirty_lo[page] = x;
    }
    if (x > s_dirty_hi[page])
    {
        s_dirty_hi[page] = x;
    }
}

/* 发送命令序列（整包单事务发送）
 * SSD1306 双字节命令【0x81 对比度 / 0xD5 时钟 / 0xA8 复用率等】的参数必须与命令
 * 处于同一 I2C 事务：逐字节分开发送时，参数字节会被屏当作独立命令解析，导致对比度等设置失效*/
static int oled_write_cmds(const uint8_t *cmds, uint8_t len)
{
    if (len == 0u)
    {
        return 0;
    }
    return (int)HAL_I2C_Mem_Write(&hi2c1, OLED_ADDR8, OLED_CTRL_CMD, I2C_MEMADD_SIZE_8BIT,
                                  (uint8_t *)cmds, len, 20);
}

/**
  * @brief  SSD1306 初始化（128x64，页地址模式）
  */
void Screen_Init(void)
{
    static const uint8_t init_cmds[] = {
        0xAE,             /* 关闭显示 */
        0xD5, 0x80,       /* 时钟分频 */
        0xA8, 0x3F,       /* 复用率 63 */
        0xD3, 0x00,       /* 显示偏移 0 */
        0x40,             /* 起始行 0 */
        0x8D, 0x14,       /* 电荷泵开 */
        0x20, 0x02,       /* 页地址模式 */
        0xA1,             /* 段重映射（若显示左右反，改 0xA0） */
        0xC8,             /* COM 扫描方向（若显示上下反，改 0xC0） */
        0xDA, 0x12,       /* COM 引脚配置 */
        0x81, 0x7F,       /* 对比度 */
        0xD9, 0xF1,       /* 预充电周期 */
        0xDB, 0x40,       /* VCOMH */
        0xA4,             /* 从 RAM 恢复显示 */
        0xA6,             /* 正常显示（非反色） */
        0xAF              /* 打开显示 */
    };

    oled_write_cmds(init_cmds, sizeof(init_cmds));
    Screen_Clear();
    Screen_Flush();
}

/**
  * @brief  清空帧缓冲
  */
void Screen_Clear(void)
{
    uint8_t page, col;
    for (page = 0; page < SCREEN_HEIGHT / 8; page++)
    {
        for (col = 0; col < SCREEN_WIDTH; col++)
        {
            g_fb[page][col] = 0;
        }
    }
    /* 清屏 = 全屏区域都变了，标全屏脏，确保下次 Flush 整屏刷新（否则屏幕残留旧内容） */
    for (page = 0; page < SCREEN_HEIGHT / 8; page++)
    {
        s_dirty_lo[page] = 0;
        s_dirty_hi[page] = SCREEN_WIDTH - 1;
    }
}

/**
  * @brief  局部刷新：只把"脏"的页/列区间写到 OLED（页地址模式定位后写 lo..hi）
  * @note   I2C1 与 MPU6050 共用，逐页加互斥锁，避免与 inputTask 的 MPU6050
  *         读取竞争（HAL I2C 非线程安全）；列地址自动递增，写满 128 列才翻页，
  *         局部写不会串页。干净页直接跳过，无 I2C 操作。
  */
void Screen_Flush(void)
{
    uint8_t page, lo, hi, cnt;
    uint8_t pos_cmds[3];
    uint8_t fail = 0;

    for (page = 0; page < SCREEN_HEIGHT / 8; page++)
    {
        lo = s_dirty_lo[page];
        hi = s_dirty_hi[page];
        if (lo > hi)
        {
            continue;   /* 本页无脏区域，跳过 */
        }

        if (osMutexAcquire(g_i2cMutex, osWaitForever) != osOK)
        {
            continue;   /* 理论上不会发生（forever 等待），防御性处理 */
        }
        pos_cmds[0] = 0xB0 | page;            /* 设置页地址 */
        pos_cmds[1] = 0x00 | (lo & 0x0F);     /* 列地址低 4 位 */
        pos_cmds[2] = 0x10 | (lo >> 4);       /* 列地址高 4 位 */
        cnt = hi - lo + 1;
        if (oled_write_cmds(pos_cmds, 3) != 0 ||
            HAL_I2C_Mem_Write(&hi2c1, OLED_ADDR8, OLED_CTRL_DATA, I2C_MEMADD_SIZE_8BIT,
                              &g_fb[page][lo], cnt, 30) != HAL_OK)
        {
            /* 传输失败（总线被外部拉死/中断打断）：保留脏区不清（下帧重刷自愈），
             * 并中止本帧剩余页——一帧最多付出 40ms 代价；若继续刷满 8 页
             * 每页 30ms，总线死时画面会冻住近 1 秒（旧版 8x100ms≈800ms 冻结数秒）。
             * 注：I2C 配置 400kHz（i2c.c 快速模式），128 字节页实际约 3ms，
             * 30ms 超时余量充足（10 倍）；整屏 8 页 ≈ 24ms，
             * 仅换关/首次重画时一帧拉长，之后增量刷新帧时间正常 */
            osMutexRelease(g_i2cMutex);
            fail = 1;
            break;
        }
        osMutexRelease(g_i2cMutex);

        s_dirty_lo[page] = 255;   /* 本页已刷新，清脏 */
        s_dirty_hi[page] = 0;
    }

    /* 整帧刷完 = 总线正常（清零）；有页失败 = 累加（≥2 判定冻结，上层暂停游戏） */
    if (fail)
    {
        if (s_flushFailCnt < 200u)
        {
            s_flushFailCnt++;
        }
    }
    else
    {
        s_flushFailCnt = 0;
    }
}

/**
  * @brief  判定 OLED 是否处于总线异常冻结期
  * @retval 1 = Flush 连续 ≥2 帧失败（总线被拖死，上层应暂停游戏只重刷屏）
  *         0 = 正常
  */
int Screen_IsStalled(void)
{
    return (s_flushFailCnt >= 2u) ? 1 : 0;
}

/**
  * @brief  复位 I2C1 外设（HAL I2C 卡 BUSY/错误状态的标准恢复手段）
  * @note   HAL I2C 偶发传输失败可能让外设卡在 BUSY（后续重试必失败、
  *         表现为"永久冻结"）——DeInit 走 MspDeInit（GPIO 复位）→ Init 走
  *         MspInit（GPIO 重配 + 外设初始化），完整重建总线；
  *         由 app.c 冻结分支每 500ms 调用一次，总线自愈关键。
  *         初始化失败不阻塞：下一轮冻结重试会再次复位
  */
void Screen_BusReset(void)
{
    if (HAL_I2C_DeInit(&hi2c1) == HAL_OK)
    {
        (void)HAL_I2C_Init(&hi2c1);
    }
}

/**
  * @brief  画单个像素
  */
void Screen_DrawPixel(uint8_t x, uint8_t y, uint8_t on)
{
    if (x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT)
    {
        return;
    }
    if (on)
    {
        g_fb[y >> 3][x] |= (uint8_t)(1u << (y & 7));
    }
    else
    {
        g_fb[y >> 3][x] &= (uint8_t)~(1u << (y & 7));
    }
    /* 所有绘制 API 最终都汇聚到这里，统一打脏标记（仅记录，不触发 I2C） */
    mark_dirty(y >> 3, x);
}

/**
  * @brief  画 8x16 字符（列扫描字模：每字节一列，上半 8 字节 + 下半 8 字节）
  */
void Screen_DrawChar8x16(uint8_t x, uint8_t y, char c)
{
    uint8_t col, row;
    uint8_t hi, lo;
    uint8_t idx;

    if (x > SCREEN_WIDTH - 8 || y > SCREEN_HEIGHT - 16)
    {
        return;
    }

    idx = (uint8_t)c;
    if (idx >= 128)
    {
        idx = 32;   /* 越界字符回退为空格 */
    }
    for (col = 0; col < 8; col++)
    {
        hi = ascii_font[idx][col];      /* 上半部分（高 8 行） */
        lo = ascii_font[idx][col + 8];  /* 下半部分（低 8 行） */
        for (row = 0; row < 8; row++)
        {
            /* 完整覆盖：字模为 1 的像素点亮、为 0 的像素清除。
             * 若只写"1"不写"0"，画空格/覆盖文字时旧像素残留在帧缓冲，
             * 屏幕会出现字体重叠。每个像素都经 DrawPixel 写入，自动打脏。 */
            Screen_DrawPixel(x + col, y + row,     (hi & (1u << row)) ? 1 : 0);
            Screen_DrawPixel(x + col, y + row + 8, (lo & (1u << row)) ? 1 : 0);
        }
    }
}

/**
  * @brief  画 8x16 字符串（超出屏幕宽度自动截断）
  */
void Screen_DrawString8x16(uint8_t x, uint8_t y, const char *str)
{
    uint8_t cur_x = x;
    while (*str != '\0')
    {
        if (cur_x > SCREEN_WIDTH - 8)
        {
            break;
        }
        Screen_DrawChar8x16(cur_x, y, *str);
        cur_x += 8;
        str++;
    }
}

/**
  * @brief  画 8x16 十进制有符号数
  */
void Screen_PrintSignedVal(uint8_t x, uint8_t y, int32_t val)
{
    char buf[12];
    int32_t i = 11;
    uint32_t v;
    uint8_t neg = 0;

    buf[11] = '\0';
    if (val < 0)
    {
        neg = 1;
        v = (uint32_t)(-val);
    }
    else
    {
        v = (uint32_t)val;
    }

    do
    {
        buf[--i] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v != 0u);

    if (neg)
    {
        buf[--i] = '-';
    }

    Screen_DrawString8x16(x, y, &buf[i]);
}

/**
  * @brief  画 8x16 十进制有符号数，以 center_x 为水平中心自动居中
  * @note   计算字符宽度（8px/字符，含负号）后左移起始 x；
  *         超出左/右边界时钳位，避免 uint8 溢出画到屏幕外
  */
void Screen_PrintSignedValCentered(uint8_t center_x, uint8_t y, int32_t val)
{
    uint8_t n = (val < 0) ? 1u : 0u;
    int32_t v = (val < 0) ? -val : val;
    int16_t x;

    do
    {
        n++;
        v /= 10;
    } while (v != 0);

    x = (int16_t)center_x - (int16_t)(n * 8) / 2;
    if (x < 0)
    {
        x = 0;
    }
    Screen_PrintSignedVal((uint8_t)x, y, val);
}

/**
  * @brief  画 8x16 十六进制数
  */
void Screen_PrintHex(uint8_t x, uint8_t y, uint32_t val)
{
    static const char hex_digit[] = "0123456789ABCDEF";
    char buf[12];
    int32_t i = 11;
    uint32_t v = val;

    buf[11] = '\0';
    if (v == 0u)
    {
        buf[--i] = '0';
    }
    while (v != 0u)
    {
        buf[--i] = hex_digit[v & 0xFu];
        v >>= 4u;
    }
    buf[--i] = 'x';
    buf[--i] = '0';

    Screen_DrawString8x16(x, y, &buf[i]);
}

/**
  * @brief  填充实心矩形
  */
void Screen_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t on)
{
    uint8_t i, j;
    for (i = 0; i < w; i++)
    {
        for (j = 0; j < h; j++)
        {
            Screen_DrawPixel(x + i, y + j, on);
        }
    }
}

/**
  * @brief  画矩形边框
  */
void Screen_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t on)
{
    uint8_t i;
    for (i = 0; i < w; i++)
    {
        Screen_DrawPixel(x + i, y, on);
        Screen_DrawPixel(x + i, y + h - 1, on);
    }
    for (i = 0; i < h; i++)
    {
        Screen_DrawPixel(x, y + i, on);
        Screen_DrawPixel(x + w - 1, y + i, on);
    }
}

/**
  * @brief  画列格式位图（每字节一列，bit0=顶部像素，高度≤8）
  * @note   bitmap 中 bit=0 的像素为透明区域，on=1 只点亮、on=0 只擦除（增量擦除安全，
  *         不会破坏同区域内的边框/文字等既有内容）；逐像素经 DrawPixel 自动打脏
  */
void Screen_DrawBitmap(uint8_t x, uint8_t y, uint8_t w, uint8_t h, const uint8_t *bitmap, uint8_t on)
{
    uint8_t col, row;
    for (col = 0; col < w; col++)
    {
        uint8_t bits = bitmap[col];
        for (row = 0; row < h; row++)
        {
            if (bits & (1u << row))
            {
                Screen_DrawPixel((uint8_t)(x + col), (uint8_t)(y + row), on);
            }
        }
    }
}
