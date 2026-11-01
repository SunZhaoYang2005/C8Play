/**
  ******************************************************************************
  * @file    proto.h
  * @brief   串口通信协议层：帧格式 + 命令集
  *          帧格式（UART，115200 8N1）：
  *            [0..1] 帧头 0xAA 0x55（跳变明显，接收端识别帧开始）
  *            [2]    LEN 数据域长度（0..16）
  *            [3]    CMD 命令码
  *            [4..]  DATA[LEN]
  *            [4+LEN]     CRC8（覆盖 LEN..DATA 末位，多项式 0x07 初值 0）
  *            [5+LEN]     帧尾 0x0D（LEN 之外的第二重边界保险）
  *          设计要点（面试点）：
  *            1. 帧头+长度+命令+数据+校验+帧尾 = 简化 CAN 报文结构：
  *               帧头/命令 ↔ CAN 仲裁段+ID，数据域 ↔ 数据场，CRC8 ↔ CRC15，
  *               应答+超时重传 ↔ ACK 槽 —— 迁到 CAN 只需换收发层
  *            2. 接收用逐字节状态机（Proto_Feed），不依赖 空闲中断/定时器，
  *               任意字节流都能自同步（丢字节后靠 AA 55 重新对齐）
  *            3. 变长数据域（LEN 控制），命令集可扩展
  ******************************************************************************
  */
#ifndef __PROTO_H
#define __PROTO_H

#include <stdint.h>

/* ---------------- 帧常量 ---------------- */
#define PROTO_HEAD0       0xAAu   /* 帧头第 1 字节 */
#define PROTO_HEAD1       0x55u   /* 帧头第 2 字节 */
#define PROTO_TAIL        0x0Du   /* 帧尾 */
#define PROTO_DATA_MAX    16u     /* 数据域最大长度 */
#define PROTO_FRAME_MAX   (2u + 1u + 1u + PROTO_DATA_MAX + 1u + 1u)  /* 22 字节 */

/* ---------------- 命令集 ----------------
 * 上行（PC → 掌机） */
#define PROTO_CMD_GET_INFO     0x01u   /* 读设备信息：型号/固件版本/协议版本 */
#define PROTO_CMD_GET_HISCORE  0x02u   /* 读最高分：三游戏高分 */
#define PROTO_CMD_CLEAR_SAVE   0x03u   /* 清存档（危险命令，联调验证用） */
#define PROTO_CMD_SET_MODE     0x05u   /* 切换模式：0=菜单 1=俄 2=蛇 3=砖 */
#define PROTO_CMD_GAME_CTRL    0x06u   /* 游戏控制：按键事件注入 */
#define PROTO_CMD_MEM_INFO     0x08u   /* 任务内存统计：触发掌机打印各任务栈高水位文本（调试） */
/* 下行（掌机 → PC） */
#define PROTO_CMD_RESPONSE     0x81u   /* 命令应答：错误码 + 数据 */
#define PROTO_CMD_STATUS       0x07u   /* 状态上报（下行，掌机主动推送）：DATA[0]=事件类型，见 PROTO_ST_EVT_* */

/* ---------------- 应答错误码 ---------------- */
#define PROTO_ERR_OK      0x00u
#define PROTO_ERR_CMD     0x01u   /* 未知命令 */
#define PROTO_ERR_PARAM   0x02u   /* 参数错误 */
#define PROTO_ERR_EXEC    0x03u   /* 执行失败 */

/* ---------------- 0x07 状态上报事件类型（DATA[0]） ----------------
 * 掌机主动推送（gameTask 上下文调 Proto_Send，与应答帧同锁串行）：
 *   MODE   ：DATA[1]=目标模式（AppState 值：0菜单/1俄/2蛇/3砖）
 *   HISCORE：DATA[1]=游戏索引（0俄/1蛇/2砖，与读最高分顺序一致），
 *            DATA[2..3]=新纪录分数 uint16 LE（上位机可即时刷新界面） */
#define PROTO_ST_EVT_MODE     0x01u
#define PROTO_ST_EVT_HISCORE  0x02u

/* ---------------- 设备信息（0x01 应答） ---------------- */
#define PROTO_FW_VERSION     0x04u   /* 固件版本 v0.4（阶段4） */
#define PROTO_PROTO_VERSION  0x01u   /* 协议版本 v1 */

/* ---------------- 解析结果 ---------------- */
typedef struct {
    uint8_t cmd;                    /* 命令码 */
    uint8_t len;                    /* 数据域长度 */
    uint8_t data[PROTO_DATA_MAX];   /* 数据域 */
} ProtoFrame;

/* 计算 CRC8：多项式 0x07、初值 0、LSB-first、覆盖 [buf, buf+len) */
uint8_t Proto_CalcCrc8(const uint8_t *buf, uint8_t len);

/* 打包一帧到 out（长度 = PROTO_HEAD0/1 + LEN + CMD + DATA + CRC8 + TAIL） */
uint8_t Proto_Pack(uint8_t *out, uint8_t cmd, const uint8_t *data, uint8_t len);

/* 复位解析状态机（初始化 / 超时强制对齐时调用） */
void Proto_Reset(void);

/* 逐字节喂入解析状态机：返回 1 = 收到完整合法帧（填入 frame）；0 = 继续收 */
uint8_t Proto_Feed(uint8_t byte, ProtoFrame *frame);

/* 发送一帧（通道集成层实现于 freertos.c Proto_Send：寄存器级轮询直写
 * USART1->SR/DR，绕过 HAL 锁——HAL_UART_Transmit 与中断接收共用句柄
 * 时锁管理互相踩踏会导致应答帧丢失；发送整体持 g_uartTxMutex，与
 * protoTask 应答/ gameTask 状态上报并发安全）
 * 返回 1 = 发送成功；0 = 失败（总线忙/超时） */
uint8_t Proto_Send(uint8_t cmd, const uint8_t *data, uint8_t len);

#endif /* __PROTO_H */
