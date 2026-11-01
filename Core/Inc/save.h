/**
  ******************************************************************************
  * @file    save.h
  * @brief   存档服务：三游戏最高分持久化到 W25Q64
  *          存储布局 v2（11 字节 × 双槽，W25Q64 扇区 0/1 各一份）：
  *            [magic "CP" 2B][版本 0x02 1B][序列号 1B][tetris 高分 2B]
  *            [snake 高分 2B][breakout 高分 2B][XOR 校验 1B]
  *          设计要点（面试点）：
  *            1. 只在破纪录时写（写前先比大小，没破纪录 0 次写入）
  *            2. 慢操作移出游戏帧：破纪录只置脏标志，housekeepingTask(100ms 节拍)
  *               轮询 Save_Poll() 执行 擦扇区→写→回读校验，不阻塞 33ms 帧循环
  *            3. 读入带校验：magic/版本/XOR 任一不符按无存档处理（分数 0）
  *            4. 【v2 掉电保护】双槽交替写 + 序列号：
  *               写前先擦"较旧"的槽，保留最新槽作完整备份；
  *               任意时刻掉电/复位最多毁掉一个槽，重启回退到另一槽的旧纪录，
  *               不会因擦除窗口掉电导致全丢（修复 v1 单槽"复位后归零"）
  *            5. 【v2 磨损均衡】两扇区交替擦写，寿命 ×2；
  *               序列号 uint8 回绕用"环形差 <128 判新"（同 TCP 序号比较）
  *          并发：所有 W25Q64 访问持 g_spiMutex（freertos.c 创建，防 gameTask
  *               读存档与 housekeeping 写存档并发冲突）
  ******************************************************************************
  */
#ifndef __SAVE_H
#define __SAVE_H

#include <stdint.h>

/* 存档游戏索引（与菜单/AppState 的游戏顺序一致：俄罗斯方块=0 贪吃蛇=1 打砖块=2） */
typedef enum {
    SAVE_GAME_TETRIS = 0,
    SAVE_GAME_SNAKE,
    SAVE_GAME_BREAKOUT,
    SAVE_GAME_COUNT
} SaveGame;

/* 初始化：读 W25Q64 → 校验 magic/版本/XOR → 载入缓存（gameTask App_Init 调用一次） */
void Save_Init(void);

/* 读某游戏最高分（内存缓存，0 = 无存档 / 数据损坏） */
int32_t Save_GetHi(SaveGame game);

/* 提交新分数：破纪录则更新缓存并置脏标志（返回 1），否则不写（返回 0）。
 * 实际落盘由 housekeeping 轮询 Save_Poll() 完成 */
int Save_Submit(SaveGame game, int32_t score);

/* 清空全部最高分（阶段4：协议命令 0x03）：
 * 缓存清零 + 置脏标志，下一轮 housekeeping 写入全 0 新帧（序列号 +1） */
void Save_ClearAll(void);

/* housekeepingTask 每 100ms（节拍）调用：有脏标志才执行 擦扇区→写→回读校验；
 * 失败保留脏标志下轮重试，连续 3 次失败放弃（避免死芯片阻塞系统） */
void Save_Poll(void);

#endif /* __SAVE_H */
