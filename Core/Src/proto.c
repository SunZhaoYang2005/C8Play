/**
  ******************************************************************************
  * @file    proto.c
  * @brief   串口通信协议层：帧打包 + 逐字节解析状态机 + CRC8
  *          帧格式：AA 55 | LEN | CMD | DATA[LEN] | CRC8 | 0D
  *          解析状态机不依赖 空闲中断/定时器/环形缓冲长度，丢字节后
  *          靠帧头 AA 55 重新自同步；CRC8 覆盖 LEN..DATA 末位。
  ******************************************************************************
  */

#include "proto.h"

/* ---------------- 解析状态机状态 ---------------- */
enum {
    ST_IDLE = 0,   /* 等帧头 AA */
    ST_HEAD1,      /* 已收 AA，等 55 */
    ST_LEN,        /* 已收 AA 55，收 LEN */
    ST_CMD,        /* 收 CMD */
    ST_DATA,       /* 收 DATA[LEN] */
    ST_CRC,        /* 收 CRC8 校验字节 */
    ST_TAIL,       /* 收帧尾 0D */
};

static uint8_t s_state;              /* 当前状态 */
static uint8_t s_len;                /* 期望数据域长度 */
static uint8_t s_idx;                /* 已收数据字节数 */
static uint8_t s_cmd;                /* 命令码 */
static uint8_t s_data[PROTO_DATA_MAX];
static uint8_t s_crc;                /* 接收端累计 CRC（LEN^CMD^DATA...） */

/* ---------------- CRC8 ----------------
 * 多项式 0x07（x^8+x^2+x+1），初值 0，LSB-first，无查表（省 Flash/兼容性好）。
 * 与 Python 上位机黄金帧同算法，可交叉验证 */
uint8_t Proto_CalcCrc8(const uint8_t *buf, uint8_t len)
{
    uint8_t crc = 0;

    while (len--)
    {
        uint8_t i;
        crc ^= *buf++;
        for (i = 0; i < 8u; i++)
        {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* 流式 CRC 更新：逐字节喂入，与 Proto_CalcCrc8 全程计算等价 */
static uint8_t proto_crc_update(uint8_t crc, uint8_t byte)
{
    uint8_t i;

    crc ^= byte;
    for (i = 0; i < 8u; i++)
    {
        crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* ---------------- 打包 ---------------- */
uint8_t Proto_Pack(uint8_t *out, uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t crc;
    uint8_t i;

    if (len > PROTO_DATA_MAX)
    {
        len = PROTO_DATA_MAX;
    }

    out[0] = PROTO_HEAD0;
    out[1] = PROTO_HEAD1;
    out[2] = len;
    out[3] = cmd;
    for (i = 0; i < len; i++)
    {
        out[4 + i] = data[i];
    }

    crc = Proto_CalcCrc8(&out[2], (uint8_t)(2u + len));   /* LEN + CMD + DATA */
    out[4 + len] = crc;
    out[5 + len] = PROTO_TAIL;
    return (uint8_t)(6u + len);
}

/* ---------------- 解析状态机 ---------------- */

void Proto_Reset(void)
{
    s_state = ST_IDLE;
    s_len = 0;
    s_idx = 0;
}

uint8_t Proto_Feed(uint8_t byte, ProtoFrame *frame)
{
    switch (s_state)
    {
    case ST_IDLE:
        if (byte == PROTO_HEAD0)
        {
            s_state = ST_HEAD1;
        }
        break;

    case ST_HEAD1:
        if (byte == PROTO_HEAD1)
        {
            s_state = ST_LEN;
            s_crc = 0;
        }
        else if (byte != PROTO_HEAD0)
        {
            s_state = ST_IDLE;   /* 连续 AA AA 合法：前一 AA 视为噪声，继续等 55 */
        }
        break;

    case ST_LEN:
        if (byte > PROTO_DATA_MAX)
        {
            s_state = ST_IDLE;   /* 长度非法：放弃本帧 */
            break;
        }
        s_len = byte;
        s_idx = 0;
        s_crc = proto_crc_update(0, byte);   /* CRC 覆盖 LEN 起（流式更新） */
        s_state = ST_CMD;
        break;

    case ST_CMD:
        s_cmd = byte;
        s_crc = proto_crc_update(s_crc, byte);
        s_state = (s_len == 0) ? ST_CRC : ST_DATA;
        break;

    case ST_DATA:
        s_data[s_idx] = byte;
        s_crc = proto_crc_update(s_crc, byte);
        s_idx++;
        if (s_idx >= s_len)
        {
            s_state = ST_CRC;
        }
        break;

    case ST_CRC:
        if (byte != s_crc)
        {
            s_state = ST_IDLE;   /* 校验失败：丢弃（可加错误计数） */
            break;
        }
        s_state = ST_TAIL;
        break;

    case ST_TAIL:
        if (byte == PROTO_TAIL)
        {
            uint8_t i;
            frame->cmd = s_cmd;
            frame->len = s_len;
            for (i = 0; i < s_len; i++)
            {
                frame->data[i] = s_data[i];
            }
            s_state = ST_IDLE;
            return 1;            /* 完整合法帧 */
        }
        s_state = ST_IDLE;       /* 帧尾不符：丢弃 */
        break;

    default:
        s_state = ST_IDLE;
        break;
    }
    return 0;
}
