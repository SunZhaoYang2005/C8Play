/**
  ******************************************************************************
  * @file    save.c
  * @brief   存档服务：三游戏最高分持久化到 W25Q64
  *          存储布局 v2（11 字节 × 双槽，W25Q64 扇区 0/1）：
  *            [0..1] magic "CP"（0x43 0x50）
  *            [2]    版本号 0x02（v1 单槽格式作废）
  *            [3]    序列号（uint8 递增，回绕用环形差比较）
  *            [4..5] 俄罗斯方块高分（uint16 小端）
  *            [6..7] 贪吃蛇高分（uint16 小端）
  *            [8..9] 打砖块高分（uint16 小端）
  *            [10]   XOR 校验（byte0..byte9 逐字节异或）
  *          双槽设计（v2，修复 v1"复位后归零"）：
  *            - 读：两个槽都读 + 校验，取序列号更新的有效帧
  *            - 写：先擦"较旧"的槽，写新帧（seq+1），最新槽始终完整保留
  *            - 掉电保护：擦除窗口掉电最多毁旧槽，重启回退到另一槽旧纪录
  *            - 磨损均衡：两扇区交替擦写，寿命 ×2
  *          写入流程（只在破纪录时由 housekeeping 触发）：
  *            选旧槽 → 擦扇区(4096) → 写 11 字节 → 回读逐字节比对
  *          失败处理：保留脏标志下轮重试，连续 3 次失败放弃
  ******************************************************************************
  */

#include "save.h"
#include "driver_spiflash_w25q64.h"
#include "cmsis_os.h"   /* osMutexAcquire/Release：W25Q64 访问互斥 */

/* ---------------- 存储格式 ---------------- */

#define SAVE_MAGIC0       0x43u   /* 'C' */
#define SAVE_MAGIC1       0x50u   /* 'P' */
#define SAVE_VERSION      0x02u   /* v2：双槽 + 序列号（v1 单槽 10 字节作废） */
#define SAVE_SECTOR_SIZE  4096u   /* 擦除粒度 = 1 扇区 */
#define SAVE_DATA_SIZE    11u     /* 有效数据 11 字节 */
#define SAVE_HI_MAX       65535u  /* uint16 存储上限（超出截断） */
#define SAVE_MAX_RETRY    3u      /* 连续写失败放弃次数 */
#define SAVE_SLOT_A       0u      /* 槽 A：扇区 0 起始（偏移 0） */
#define SAVE_SLOT_B       4096u   /* 槽 B：扇区 1 起始（偏移 4096） */

/* SPI 总线互斥量（freertos.c 创建：gameTask 读存档 vs housekeeping 写存档并发保护） */
extern osMutexId_t g_spiMutex;

/* ---------------- 状态 ---------------- */

static int32_t s_hi[SAVE_GAME_COUNT];  /* 最高分缓存（内存） */
static uint8_t s_dirty;                /* 1 = 有破纪录待落盘 */
static uint8_t s_failCount;            /* 连续写失败次数 */
static uint8_t s_seq;                  /* 当前最新有效帧的序列号 */

/* ---------------- 内部函数 ---------------- */

/**
  * @brief  打包存档帧：magic + 版本 + 序列号 + 3 个 uint16 高分 + XOR 校验
  * @param  buf  11 字节输出（调用方提供）
  * @param  seq  本帧序列号
  */
static void save_pack(uint8_t *buf, uint8_t seq)
{
    uint8_t i;

    buf[0] = SAVE_MAGIC0;
    buf[1] = SAVE_MAGIC1;
    buf[2] = SAVE_VERSION;
    buf[3] = seq;
    buf[4] = (uint8_t)(s_hi[SAVE_GAME_TETRIS]    & 0xFFu);
    buf[5] = (uint8_t)((s_hi[SAVE_GAME_TETRIS]    >> 8) & 0xFFu);
    buf[6] = (uint8_t)(s_hi[SAVE_GAME_SNAKE]    & 0xFFu);
    buf[7] = (uint8_t)((s_hi[SAVE_GAME_SNAKE]    >> 8) & 0xFFu);
    buf[8] = (uint8_t)(s_hi[SAVE_GAME_BREAKOUT] & 0xFFu);
    buf[9] = (uint8_t)((s_hi[SAVE_GAME_BREAKOUT] >> 8) & 0xFFu);

    buf[10] = 0;
    for (i = 0; i < 10u; i++)
    {
        buf[10] ^= buf[i];
    }
}

/**
  * @brief  校验一帧数据是否合法（magic + 版本 + XOR）
  * @retval 1 = 合法
  */
static int save_validate(const uint8_t *buf)
{
    uint8_t i, x = 0;

    if (buf[0] != SAVE_MAGIC0 || buf[1] != SAVE_MAGIC1 || buf[2] != SAVE_VERSION)
    {
        return 0;
    }
    for (i = 0; i < 10u; i++)
    {
        x ^= buf[i];
    }
    return (buf[10] == x) ? 1 : 0;
}

/**
  * @brief  序列号环形比较：a 是否比 b 新（uint8 回绕安全，同 TCP 序号比较）
  * @retval 1 = a 新；0 = b 新或相等
  */
static int save_seq_newer(uint8_t a, uint8_t b)
{
    return ((uint8_t)(a - b) < 128u) ? 1 : 0;
}

/**
  * @brief  读一个槽并校验（持锁调用）
  * @retval 1 = 读到有效帧；0 = 无效/读失败
  */
static int save_read_slot(uint32_t addr, uint8_t *buf, uint8_t *seq)
{
    int r = W25Q64_Read(addr, buf, SAVE_DATA_SIZE);
    if (r != (int)SAVE_DATA_SIZE || !save_validate(buf))
    {
        return 0;
    }
    *seq = buf[3];
    return 1;
}

/**
  * @brief  从双槽选择最新有效帧，载入缓存与状态
  * @retval 1 = 载入有效帧（rawOut 已填充）；0 = 两槽皆无效（无存档）
  */
static int save_load_best(uint8_t *rawOut)
{
    uint8_t bufA[SAVE_DATA_SIZE], bufB[SAVE_DATA_SIZE];
    uint8_t seqA = 0, seqB = 0;
    int okA, okB;
    uint8_t useB;
    uint8_t i;

    okA = save_read_slot(SAVE_SLOT_A, bufA, &seqA);
    okB = save_read_slot(SAVE_SLOT_B, bufB, &seqB);

    if (!okA && !okB)
    {
        s_seq = 0;
        return 0;   /* 双槽皆空/损坏：按无存档处理 */
    }

    /* 都有效 → 取序列号新的；只有一槽有效 → 用之 */
    if (okA && okB)
    {
        useB = save_seq_newer(seqB, seqA) ? 1u : 0u;
    }
    else
    {
        useB = okB ? 1u : 0u;
    }

    if (useB)
    {
        for (i = 0; i < SAVE_DATA_SIZE; i++) { rawOut[i] = bufB[i]; }
        s_seq = seqB;
    }
    else
    {
        for (i = 0; i < SAVE_DATA_SIZE; i++) { rawOut[i] = bufA[i]; }
        s_seq = seqA;
    }
    return 1;
}

/* ---------------- 对外接口 ---------------- */

/**
  * @brief  初始化：读双槽 → 选最新有效帧 → 载入缓存
  * @note   gameTask App_Init 中调用（读 22 字节微秒级，不阻塞帧循环）；
  *         两槽都无效 / 校验失败 / 空 Flash 一律按无存档处理（分数 0）
  */
void Save_Init(void)
{
    uint8_t raw[SAVE_DATA_SIZE];
    uint8_t g;

    for (g = 0; g < SAVE_GAME_COUNT; g++)
    {
        s_hi[g] = 0;
    }
    s_dirty = 0;
    s_failCount = 0;

    W25Q64_Init();

    osMutexAcquire(g_spiMutex, osWaitForever);
    if (!save_load_best(raw))
    {
        osMutexRelease(g_spiMutex);
        return;   /* 无存档 / 损坏：缓存保持 0 */
    }
    osMutexRelease(g_spiMutex);

    s_hi[SAVE_GAME_TETRIS]    = (int32_t)(raw[4] | ((uint16_t)raw[5] << 8));
    s_hi[SAVE_GAME_SNAKE]     = (int32_t)(raw[6] | ((uint16_t)raw[7] << 8));
    s_hi[SAVE_GAME_BREAKOUT]  = (int32_t)(raw[8] | ((uint16_t)raw[9] << 8));
}

/**
  * @brief  读某游戏最高分（内存缓存）
  */
int32_t Save_GetHi(SaveGame game)
{
    if (game >= SAVE_GAME_COUNT)
    {
        return 0;
    }
    return s_hi[game];
}

/**
  * @brief  提交新分数：破纪录则更新缓存 + 置脏标志
  * @retval 1 = 破纪录（待 housekeeping 落盘）；0 = 未破纪录（不写 Flash）
  */
int Save_Submit(SaveGame game, int32_t score)
{
    if (game >= SAVE_GAME_COUNT)
    {
        return 0;
    }
    if (score <= s_hi[game])
    {
        return 0;   /* 未破纪录：一个字都不写（省擦写寿命） */
    }
    if (score > (int32_t)SAVE_HI_MAX)
    {
        score = (int32_t)SAVE_HI_MAX;
    }
    s_hi[game] = score;
    s_dirty = 1;
    return 1;
}

/**
  * @brief  清空全部最高分（协议命令 0x03）
  * @note   缓存清零 + 置脏标志，housekeeping 下一轮轮询会把全 0 新帧
  *         （序列号 +1）写入较旧槽——复用现有落盘机制，无需单独写 Flash
  */
void Save_ClearAll(void)
{
    uint8_t g;

    for (g = 0; g < SAVE_GAME_COUNT; g++)
    {
        s_hi[g] = 0;
    }
    s_dirty = 1;
}

/**
  * @brief  housekeepingTask 每 100ms（节拍）轮询：有脏标志才执行 选旧槽→擦→写→回读校验
  * @note   双槽策略：先擦"较旧"的槽再写新帧，最新槽始终完整；
  *         擦除窗口掉电最多毁旧槽，重启自动回退另一槽旧纪录（不归零）；
  *         实际只在破纪录/清档后各执行一次（脏标志置位时），正常轮询开销几乎为零
  */
void Save_Poll(void)
{
    uint8_t buf[SAVE_DATA_SIZE];
    uint8_t check[SAVE_DATA_SIZE];
    uint8_t bufA[SAVE_DATA_SIZE], bufB[SAVE_DATA_SIZE];
    uint8_t seqA = 0, seqB = 0;
    int okA, okB;
    uint8_t slotIdx;
    uint32_t slotAddr;
    uint8_t newSeq;
    int r;

    if (!s_dirty)
    {
        return;
    }
    if (s_failCount >= SAVE_MAX_RETRY)
    {
        s_dirty = 0;      /* 多次失败：放弃本次存档（缓存仍在内存，运行期有效） */
        s_failCount = 0;
        return;
    }

    /* 选写目标槽：写"较旧"的槽，保留最新槽作回退（掉电保护核心） */
    osMutexAcquire(g_spiMutex, osWaitForever);
    okA = save_read_slot(SAVE_SLOT_A, bufA, &seqA);
    okB = save_read_slot(SAVE_SLOT_B, bufB, &seqB);

    if (!okA && !okB)
    {
        slotIdx = 0;                        /* 两槽皆空：写槽 A */
    }
    else if (okA && !okB)
    {
        slotIdx = 1;                        /* 只有 A 有效：写空的 B */
    }
    else if (!okA && okB)
    {
        slotIdx = 0;                        /* 只有 B 有效：写空的 A */
    }
    else
    {
        slotIdx = save_seq_newer(seqA, seqB) ? 1u : 0u;  /* 都有效：写较旧的 */
    }
    slotAddr = slotIdx ? SAVE_SLOT_B : SAVE_SLOT_A;
    newSeq = (uint8_t)(s_seq + 1);

    save_pack(buf, newSeq);

    r = W25Q64_Erase(slotAddr, SAVE_SECTOR_SIZE);
    if (r < 0)
    {
        osMutexRelease(g_spiMutex);
        s_failCount++;
        return;
    }
    r = W25Q64_Write(slotAddr, buf, SAVE_DATA_SIZE);
    if (r < 0)
    {
        osMutexRelease(g_spiMutex);
        s_failCount++;
        return;
    }
    r = W25Q64_Read(slotAddr, check, SAVE_DATA_SIZE);
    osMutexRelease(g_spiMutex);

    /* 回读校验：长度 + 逐字节比对（含 XOR 与序列号，可发现数据位翻转） */
    if (r != (int)SAVE_DATA_SIZE ||
        check[0] != buf[0] || check[1] != buf[1] || check[2] != buf[2] ||
        check[3] != buf[3] || check[4] != buf[4] || check[5] != buf[5] ||
        check[6] != buf[6] || check[7] != buf[7] || check[8] != buf[8] ||
        check[9] != buf[9] || check[10] != buf[10])
    {
        s_failCount++;
        return;
    }

    s_seq = newSeq;
    s_dirty = 0;
    s_failCount = 0;
}
