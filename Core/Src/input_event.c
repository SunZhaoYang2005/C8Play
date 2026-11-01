/**
  ******************************************************************************
  * @file    input_event.c
  * @brief   掌机统一输入抽象实现
  *          输入源：旋转编码器(PB0/PB12 + 按键PB1)、板载K1(PB14)、
  *                 红外接收(PB10, NEC)、MPU6050(I2C1, 倾斜检测)
  *          编码器旋转：计数变化->MOVE，带 40ms 反向防抖（机械回弹产生的
  *                     反向脉冲直接丢弃，游戏层不会"自己往回转"）
  *          编码器按键：短按(<300ms)=CLICK(确认/加速一拍)，
 *                     按住>=300ms=ENC_HOLD(每100ms持续加速)，松开=ENC_RELEASE；
 *                     按住>800ms仍投递一次LONG_PRESS(返回菜单，由游戏层决定是否消费)
  *          K1：点按(<300ms)=KEY(硬降)，按住>=300ms=KEY_HOLD(每100ms软降)，
  *               松开=KEY_RELEASE
  *          所有事件经 FreeRTOS 消息队列分发
  ******************************************************************************
  */

#include "input_event.h"
#include "main.h"
#include "cmsis_os.h"

#include "driver_rotary_encoder.h"
#include "driver_key.h"
#include "driver_ir_receiver.h"
#include "driver_mpu6050.h"

/* driver_mpu6050.h 未导出 ReadRegister，此处补充声明 */
extern int MPU6050_ReadRegister(uint8_t reg, uint8_t *pdata);
#include "i2c.h"   /* hi2c1：OLED 与 MPU6050 共用 I2C1 */

/* I2C1 总线互斥量（freertos.c RTOS_MUTEX 区创建）：与 OLED 共用，读取需持锁 */
extern osMutexId_t g_i2cMutex;

/* 事件队列 */
static osMessageQueueId_t g_inputQueue;

/* 编码器状态 */
static int32_t g_lastCnt;        /* 上次采样计数值 */
static uint8_t g_encKeyPressed;  /* 编码器按键是否处于按下状态（稳定电平驱动） */
static uint32_t g_encKeyPressTick; /* 按下时刻(tick) */
static uint8_t g_encKeyLongFired;  /* LONG_PRESS 是否已触发 */
static uint8_t g_encHoldFired;     /* 1 = 已进入持续模式（ENC_HOLD 重复投递中） */
static uint32_t g_encLastHoldTick; /* 上次投递 ENC_HOLD 的时刻（100ms 节拍） */
static uint32_t g_encKeyReleaseTick; /* 上次确认松开的时刻(tick)：回弹锁基准 */
static uint32_t g_encKeyRawTick;     /* 原始电平首次为 1 的时刻：按键扰动窗口基准 */
static uint8_t  g_encKeyPrevRaw;     /* 编码器按键上一帧原始电平（沿预检用） */
static uint32_t g_encKeyRawReleaseTick; /* 原始松开沿时刻(tick)：旋转块吸收窗口基准
                                         *（松开瞬间弹片回弹的假旋转与松开同帧发生，
                                         * 而消抖后的 ReleaseTick 在按键块才更新、
                                         * 旋转块先执行看不到——用原始沿提前记录） */
static int8_t  g_lastMoveDir;     /* 上次采样/投递方向（+1/-1） */
static int8_t  g_pendingDir;      /* 旋转待确认方向（0=无；机械回弹吸收用） */
static int32_t g_pendingBase;     /* 进入待确认时的基准计数值（净位移判定） */
static uint32_t g_pendingTick;    /* 进入/更新待确认的时刻(tick) */
static uint8_t  g_pendingReversed; /* 待确认期间是否经历过方向反转（回弹标记） */
static uint32_t g_lastMoveTick;    /* 上次 MOVE 投递时刻：静止兜底的"停手确认"基准 */

/* 红外状态（重复码间隔过滤） */
static uint8_t  s_irLastKey;      /* 上次有效按键码（重复码回放用） */
static uint32_t s_irLastTick;     /* 上次有效按键时刻(tick) */

/* 板载按键 K1 状态 */
static uint8_t  g_k1Pressed;      /* 1 = 正在按下 */
static uint32_t g_k1PressTick;    /* 按下时刻(tick)，用于 300ms 长短判定 */
static uint8_t  g_k1HoldFired;    /* 1 = 已进入软降模式（KEY_HOLD 重复投递中） */
static uint32_t g_k1LastHoldTick; /* 上次投递 KEY_HOLD 的时刻（100ms 节拍） */

/* MPU6050 状态 */
static uint32_t g_mpuNextScan;   /* 下次 MPU6050 扫描时刻（tick），初始 0 = 上电立即首读 */
static uint8_t  g_mpuFailCnt;    /* 连续读取失败计数（≥3 降频，避免超时反复拖慢帧） */

/* 向队列投递事件（非阻塞，队列满则丢弃） */
static void input_post(InputEventType type, int32_t value)
{
    InputEvent ev;
    ev.type = (uint8_t)type;
    ev.value = value;
    osMessageQueuePut(g_inputQueue, &ev, 0, 0);
}

/**
  * @brief  MPU6050 唤醒：写 PWR_MGMT_1=0x00 并读回验证
  * @note   韦东山 MPU6050_Init() 不检查 I2C 写失败、无条件返回 0：
  *         上电早期芯片未稳定时配置写不进去，芯片停留在睡眠态(加速度全 0)。
  *         这里"写唤醒 + 读回确认"，确保传感器真正退出睡眠。
  * @retval 0 成功, -1 失败
  */
static int mpu6050_wakeup(void)
{
    uint8_t wd[2] = {0x6B, 0x00};   /* PWR_MGMT_1 = 0x00：解除休眠(SLEEP=0) */
    uint8_t pwr = 0xFF;
    int ret = -1;

    /* I2C1 与 OLED 共用：唤醒序列整体持互斥锁（HAL I2C 非线程安全）。
     * 任务内延时用 osDelay（让出 CPU），不用 HAL_Delay（忙等占满 CPU）。 */
    if (osMutexAcquire(g_i2cMutex, osWaitForever) == osOK)
    {
        if (HAL_I2C_Master_Transmit(&hi2c1, 0xD0, wd, 2, 100) == HAL_OK)
        {
            osDelay(10);   /* 等唤醒生效 */
            if (HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x6B, I2C_MEMADD_SIZE_8BIT,
                                 &pwr, 1, 100) == HAL_OK)
            {
                ret = (pwr == 0x00) ? 0 : -1;
            }
        }
        osMutexRelease(g_i2cMutex);
    }
    return ret;
}

/**
  * @brief  MPU6050 初始化：唤醒 + 量程验证（任务内调用，标准 RTOS 写法）
  * @note   由 inputTask 启动时调用：延时用 osDelay（让出 CPU，不忙等），
  *         I2C 传输持 g_i2cMutex（与 OLED 刷新互斥）。
  *         唤醒或量程验证失败均静默继续（不影响其他输入设备）。
  */
void Input_InitMPU6050(void)
{
    int i;

    /* 上电稳定延时 + 唤醒读回验证 + 重试（最多 5 次） */
    osDelay(100);
    for (i = 0; i < 5; i++)
    {
        if (mpu6050_wakeup() == 0)
        {
            break;
        }
        osDelay(100);
    }
    /* 量程读回验证：ACCEL_CONFIG 必须 == 0x18(±16g)。
     * 韦东山 Init 不检查写失败，若量程停留在默认 ±2g，
     * 本文件倾斜阈值 ±700(≈20°) 会按错误灵敏度失效，故写后必读回。
     * I2C 传输持互斥锁，避免与 OLED 刷新并发。 */
    for (i = 0; i < 5; i++)
    {
        uint8_t cfg = 0;
        if (osMutexAcquire(g_i2cMutex, osWaitForever) == osOK)
        {
            MPU6050_Init();
            osMutexRelease(g_i2cMutex);
        }
        if (osMutexAcquire(g_i2cMutex, osWaitForever) == osOK)
        {
            if (MPU6050_ReadRegister(0x1C, &cfg) == 0 && cfg == 0x18)
            {
                osMutexRelease(g_i2cMutex);
                break;      /* 量程 ±16g 确认生效 */
            }
            osMutexRelease(g_i2cMutex);
        }
        osDelay(100);
    }
}

/**
  * @brief  初始化输入外设 + 创建事件队列
  * @note   MPU6050 由 Input_InitMPU6050() 单独初始化（inputTask 内调用）
  */
void Input_Init(void)
{
    g_inputQueue = osMessageQueueNew(16, sizeof(InputEvent), NULL);
    RotaryEncoder_Init();
    Key_Init();
    IRReceiver_Init();

    g_lastCnt = 0;
    g_encKeyPressed = 0;
    g_encKeyLongFired = 0;
    g_encHoldFired = 0;
    g_encLastHoldTick = 0;
    g_encKeyReleaseTick = 0;
    g_encKeyRawTick = 0;
    g_encKeyPrevRaw = 0;
    g_encKeyRawReleaseTick = 0;
    g_lastMoveDir = 0;
    g_pendingDir = 0;
    g_pendingBase = 0;
    g_pendingTick = 0;
    g_pendingReversed = 0;
    g_lastMoveTick = 0;
    s_irLastKey = 0;
    s_irLastTick = 0;
    g_k1Pressed = 0;
    g_k1HoldFired = 0;
    g_k1LastHoldTick = 0;
    g_mpuNextScan = 0;
    g_mpuFailCnt = 0;
}

/**
  * @brief  周期采样所有输入（建议 10ms 调用一次）
  */
void Input_Scan(void)
{
    int32_t cnt, speed, key;
    int k1;
    uint8_t dev, data;
    uint32_t now;
    InputEvent ev;

    now = osKernelGetTickCount();

    /* ---------- 1. 旋转编码器：计数变化 -> MOVE ---------- */
    /* 注：此统一为"顺时针=正"，游戏层无需感知物理方向 */
    RotaryEncoder_Read(&cnt, &speed, &key);

    /* ---------- 1.5 编码器按键原始电平沿预检 ----------
     * 旋转块先于按键块执行：松开瞬间弹片回弹带动的假旋转若
     * 与松开同帧发生，旋转块看到的 g_encKeyReleaseTick 仍是旧值（按键块
     * 本帧末才更新），吸收窗口失效 → "长按松开后偶尔多走一格"。
     * 这里在 RotaryEncoder_Read 之后（key 已赋值）记录原始电平沿时刻，
     * 供旋转块同帧生效：按下沿 = g_encKeyRawTick（与按键块记录同值，
     * 更早可用）；松开沿 = g_encKeyRawReleaseTick（按键块不记录原始松开沿） */
    if (key != g_encKeyPrevRaw)
    {
        if (key == 1)
        {
            g_encKeyRawTick = now;
        }
        else
        {
            g_encKeyRawReleaseTick = now;
        }
        g_encKeyPrevRaw = key;
    }

    if (cnt != g_lastCnt)
    {
        int32_t delta = (int32_t)(g_lastCnt - cnt);
        int8_t  dir = (delta > 0) ? 1 : -1;

        /* 按下原始沿起 100ms 内 / 原始松开沿起 50ms 内：EC11 弹片
         * 按下与松开瞬间都会带动 S1/S2 产生一次假旋转（通常反向一格），
         * 全部吸收（更新基准不投递）。松开沿由第 0 段预检同帧记录 */
        if ((now - g_encKeyRawTick) < 100u ||
            (now - g_encKeyRawReleaseTick) < 50u)
        {
            g_lastCnt = cnt;
            g_pendingDir = 0;
            g_pendingReversed = 0;
        }
        else if (g_pendingDir != 0)
        {
            if (dir == g_pendingDir)
            {
                /* 待确认方向连续同向：确认，投递基准到当前的累计位移 */
                ev.type = (uint8_t)INPUT_EV_MOVE;
                ev.value = (int32_t)(g_pendingBase - cnt);
                osMessageQueuePut(g_inputQueue, &ev, 0, 0);
                g_lastMoveTick = now;
                g_pendingDir = 0;
                g_pendingReversed = 0;
                g_lastMoveDir = dir;
            }
            else
            {
                /* 待确认中又反转：典型回弹，方向跟随最新、基准保持，
                 * 标记"经历过反转"——静止兜底不再投递（回弹归位丢弃） */
                g_pendingDir = dir;
                g_pendingTick = now;
                g_pendingReversed = 1;
            }
        }
        else if (dir != g_lastMoveDir)
        {
            /* 方向反转：进入待确认（不投递），记基准计数值 */
            g_pendingDir = dir;
            g_pendingBase = g_lastCnt;
            g_pendingTick = now;
            g_pendingReversed = 0;
        }
        else
        {
            /* 与上次采样方向一致：直接投递本次位移 */
            ev.type = (uint8_t)INPUT_EV_MOVE;
            ev.value = delta;
            osMessageQueuePut(g_inputQueue, &ev, 0, 0);
            g_lastMoveTick = now;
            g_lastMoveDir = dir;
        }
        g_lastCnt = cnt;
    }
    else if (g_pendingDir != 0 && (now - g_pendingTick) >= 20u)
    {
        /* 待确认后静止 20ms：
         * 未经历过反转、净位移非零、且距上次投递 >=300ms（独立单格旋转）
         * 才投递；距上次投递 <300ms = 旋转中/停手瞬间的偶发反向沿
         * （S1 反弹沿 + S2 已变导致的方向误判），吸收不投——这是
         * "连续旋转偶尔回退一下"（快/慢速统一覆盖）的根源。
         * 真实反向微调若在连续旋转后 300ms 内只转一格会被吸收（罕见）；
         * 连续反向旋转则经"同向确认"正常投递，不受影响。
         * 经历过反转（回弹）或净零 = 回弹归位，丢弃 */
        int32_t net = (int32_t)(g_pendingBase - cnt);
        if (g_pendingReversed == 0 && net != 0 && (now - g_lastMoveTick) >= 300u)
        {
            ev.type = (uint8_t)INPUT_EV_MOVE;
            ev.value = net;
            osMessageQueuePut(g_inputQueue, &ev, 0, 0);
            g_lastMoveTick = now;
            g_lastMoveDir = (net > 0) ? 1 : -1;
        }
        g_pendingDir = 0;
        g_pendingReversed = 0;
    }

    /* ---------- 2. 编码器按键：短按 CLICK / 按住 ENC_HOLD / 800ms LONG_PRESS ----------
     * 语义：按下 300ms 内松开 -> CLICK（确认/加速一拍）；
     *       按住 >=300ms 进入持续模式，之后每 100ms 投递 ENC_HOLD（持续加速）；
     *       松开投递 ENC_RELEASE（持续加速结束，恢复节拍基准）；
     *       按住 >800ms 仍投递一次 LONG_PRESS（返回菜单；游戏进行中由游戏层
     *       消费掉，暂停/结算态才真正返回菜单）。俄罗斯方块不识别新事件，
     *       行为不变（CLICK=旋转、LONG_PRESS=返回菜单）。 */
    /* 编码器按键（20ms 稳定消抖 + 回弹锁）：
     * 原始 key 按下/松开瞬间有机械抖动，甚至半接触（弹片接触不良持续
     * 低电平），会把一次物理按键拆成多次状态翻转，或让"短按"被误判成
     * 800ms 长按退回菜单。稳定电平需连续 2 次采样（20ms）一致才翻转；
     * 确认松开后 50ms 内再次稳定按下视为弹片回弹，忽略。 */
    {
        static uint8_t s_encKeyStable = 0;    /* 消抖后的稳定电平 */
        static uint8_t s_encKeyStableCnt = 0; /* 连续不一致计数 */
        uint8_t keyRaw = (key == 1) ? 1 : 0;

        if (keyRaw == 1 && s_encKeyStable == 0 && g_encKeyPressed == 0)
        {
            /* 原始电平首次为 1（消抖前）：记录物理按下时刻，供旋转块
             * 的"按键机械扰动窗口"（100ms 内吸收假旋转）使用 */
            g_encKeyRawTick = now;
        }

        if (keyRaw != s_encKeyStable)
        {
            if (++s_encKeyStableCnt >= 2)
            {
                s_encKeyStable = keyRaw;
                s_encKeyStableCnt = 0;
            }
        }
        else
        {
            s_encKeyStableCnt = 0;
        }

        if (s_encKeyStable == 1 && g_encKeyPressed == 0)
        {
            /* 刚按下（回弹锁：松开确认后 50ms 内的稳定按下 = 弹片回弹） */
            if ((now - g_encKeyReleaseTick) >= 50u)
            {
                g_encKeyPressed = 1;
                g_encKeyPressTick = now;
                g_encKeyLongFired = 0;
                g_encHoldFired = 0;
                g_encLastHoldTick = now;
            }
            /* <50ms 内：回弹，保持松开状态 */
        }
        else if (s_encKeyStable == 0 && g_encKeyPressed == 1)
        {
            /* 松开：若从未触发过长按/持续模式则为短按 */
            if (g_encKeyLongFired == 0)
            {
                input_post(INPUT_EV_CLICK, 1);
            }
            input_post(INPUT_EV_ENC_RELEASE, 1);   /* 持续加速结束（即使没持续也无害） */
            g_encKeyPressed = 0;
            g_encKeyReleaseTick = now;
        }
        else if (g_encKeyPressed == 1 && g_encKeyLongFired == 0 &&
                 (now - g_encKeyPressTick) > 800u)
        {
            /* 长按 800ms 触发一次 LONG_PRESS（返回菜单；游戏层决定是否消费）。
             * 必须排在 ENC_HOLD 重复分支之前：否则 holdFired=1 后每 100ms 分支
             * 总是先匹配，LONG_PRESS 永远轮不到。 */
            g_encKeyLongFired = 1;
            input_post(INPUT_EV_LONG_PRESS, 1);
        }
        else if (g_encKeyPressed == 1 && g_encHoldFired == 0 &&
                 (now - g_encKeyPressTick) > 300u)
        {
            /* 按住超过 300ms：进入持续模式，立即投递一次 */
            g_encHoldFired = 1;
            g_encLastHoldTick = now;
            input_post(INPUT_EV_ENC_HOLD, 1);
        }
        else if (g_encKeyPressed == 1 && g_encHoldFired == 1 &&
                 (now - g_encLastHoldTick) >= 100u)
        {
            /* 持续模式中：每 100ms 投递一次（10ms 采样周期，用时刻差判定） */
            g_encLastHoldTick = now;
            input_post(INPUT_EV_ENC_HOLD, 1);
        }
    }

    /* ---------- 3. 板载按键 K1：点按=KEY(硬降) / 按住=KEY_HOLD(软降) ----------
     * 语义：按下 300ms 内松开 -> INPUT_EV_KEY（快速点按，游戏里=硬降）；
     *       按住超过 300ms 进入软降模式，之后每 100ms 投递一次 KEY_HOLD
     *       （游戏里=加速下落一格）；松开投递 KEY_RELEASE（软降结束）。
     * 这样"点按落底"与"按住持续加速"共用 K1，不增加硬件按键。 */
    k1 = Key_Read();
    if (k1 == 1 && g_k1Pressed == 0)
    {
        /* 刚按下：记录时刻，等待 300ms 长短判定 */
        g_k1Pressed = 1;
        g_k1PressTick = now;
        g_k1HoldFired = 0;
        g_k1LastHoldTick = now;
    }
    else if (k1 == 0 && g_k1Pressed == 1)
    {
        /* 松开：若从未进入软降，则为快速点按（短按语义） */
        if (g_k1HoldFired == 0)
        {
            input_post(INPUT_EV_KEY, 1);
        }
        input_post(INPUT_EV_KEY_RELEASE, 1);   /* 软降结束（即使没软降也无害） */
        g_k1Pressed = 0;
    }
    else if (g_k1Pressed == 1 && g_k1HoldFired == 0 &&
             (now - g_k1PressTick) > 300u)
    {
        /* 按住超过 300ms：进入软降模式，立即投递一次 */
        g_k1HoldFired = 1;
        g_k1LastHoldTick = now;
        input_post(INPUT_EV_KEY_HOLD, 1);
    }
    else if (g_k1Pressed == 1 && g_k1HoldFired == 1 &&
             (now - g_k1LastHoldTick) >= 100u)
    {
        /* 软降中：每 100ms 投递一次（10ms 采样周期，故用时刻差判定） */
        g_k1LastHoldTick = now;
        input_post(INPUT_EV_KEY_HOLD, 1);
    }

    /* ---------- 4. 红外遥控：收到按键码 -> IR ---------- */
    if (IRReceiver_Read(&dev, &data) == 0)
    {
        if (data != 0)
        {
            /* 有效按键：记录为"上次按键"，供重复码回放 */
            s_irLastKey = data;
            s_irLastTick = now;
            input_post(INPUT_EV_IR, (int32_t)data);
        }
        else
        {
            /* 重复码（data==0，NEC 长按自动重复帧）：游戏用它实现
             * 长按连续移动/加速。只有距上次有效按键 >=100ms（重复帧
             * 间隔约 110ms）才是真实长按重复，回放上次按键码；
             * <100ms 的是松手杂波误判的伪重复码，丢弃
             * （否则"点一次按键触发两次旋转"） */
            if (s_irLastKey != 0 && (now - s_irLastTick) >= 100u)
            {
                s_irLastTick = now;
                input_post(INPUT_EV_IR, (int32_t)s_irLastKey);
            }
        }
    }

    /* ---------- 5. MPU6050 倾斜：节拍式扫描（默认 100ms，连续失败离线退避 3s） ----------
     * 读到非零档位即持续投递（挡板持续移动，回正停止）；
     * 连续 3 次读失败 → 进入离线退避：3 秒才探活一次，其余时间完全不碰 I2C
     * （零锁竞争、零卡顿）；读失败超时仅 5ms
     * 曾排查"晃动卡死"现方案已定型：轮询持锁 + 连续失败离线退避 3s + 探活失败仅锁总线 5ms，
     * MPU6050异常不再拖死 OLED 刷屏；退避期间倾斜事件静默*/
    now = osKernelGetTickCount();
    if ((int32_t)(now - g_mpuNextScan) >= 0)
    {
        int16_t ax, ay, az, gx, gy, gz;
        int32_t tilt = 0;
        int ok = 0;

        /* I2C1 与 OLED 共用：读取持互斥锁，避免与 Screen_Flush 竞争 */
        if (osMutexAcquire(g_i2cMutex, osWaitForever) == osOK)
        {
            ok = (MPU6050_ReadData(&ax, &ay, &az, &gx, &gy, &gz) == 0);
            osMutexRelease(g_i2cMutex);
        }

        if (ok)
        {
            g_mpuFailCnt = 0;
            /* ±16g 量程下 2048 LSB/g；阈值 ±350 ≈ 0.17g ≈ 10° 倾斜（灵敏控板） */
            if (ax > 350)
            {
                tilt = 1;
            }
            else if (ax < -350)
            {
                tilt = -1;
            }

            if (tilt != 0)
            {
                input_post(INPUT_EV_TILT, tilt);
            }
            g_mpuNextScan = now + 100u;
        }
        else
        {
            g_mpuFailCnt++;
            /* 连续失败 ≥3：进入离线退避——3 秒才探活一次（每次探活失败仅锁 5ms），
             * 其余时间完全不碰 I2C 总线；晃动导致传感器/接线异常把总线拖死时，
             * 此举把传感器侧占用降到最低，给 OLED 刷屏留出恢复窗口 */
            g_mpuNextScan = now + (g_mpuFailCnt >= 3u ? 3000u : 100u);
        }
    }
}

/**
  * @brief  从事件队列取事件
  * @param  ev            事件输出
  * @param  timeout_tick  超时(tick)，osWaitForever 表示一直等待
  * @retval 1 成功取到事件, 0 超时
  */
int Input_GetEvent(InputEvent *ev, uint32_t timeout_tick)
{
    if (osMessageQueueGet(g_inputQueue, ev, NULL, timeout_tick) == osOK)
    {
        return 1;
    }
    return 0;
}

/**
  * @brief  投递一个事件到队列（串口协议游戏控制命令注入）
  * @note   与物理输入等价：游戏任务只认事件，不关心来源（编码器/红外/串口）。
  *         队列满时丢弃（协议侧有应答与重传兜底，不阻塞调用方）
  */
void Input_PostEvent(uint8_t type, int32_t value)
{
    InputEvent ev;

    ev.type = type;
    ev.value = value;
    osMessageQueuePut(g_inputQueue, &ev, 0, 0);
}

/**
  * @brief  丢弃队列中全部积压事件
  * @note   冻结暂停期调用：冻结期间 inputTask 仍会把倾斜/按键事件投递入队，
  *         若恢复后一次性消费，挡板/菜单会瞬移；冻结期间的输入本就看不见，直接丢弃
  */
void Input_ClearEvents(void)
{
    osMessageQueueReset(g_inputQueue);
}
