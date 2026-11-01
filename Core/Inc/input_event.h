/**
  ******************************************************************************
  * @file    input_event.h
  * @brief   掌机统一输入抽象：编码器 / 板载按键 / 红外遥控 / MPU6050 倾斜
  *          全部转换为标准事件，经消息队列分发给游戏任务
  *          由 input_task 周期调用 Input_Scan() 采样，游戏任务调用 Input_GetEvent()
  ******************************************************************************
  */
#ifndef __INPUT_EVENT_H
#define __INPUT_EVENT_H

#include <stdint.h>

/* 事件类型 */
typedef enum {
    INPUT_EV_NONE = 0,      /* 无事件 */
    INPUT_EV_MOVE,          /* 编码器旋转：value>0 顺时针(右/下), value<0 逆时针(左/上) */
    INPUT_EV_CLICK,         /* 编码器按键短按（确认/进入） */
    INPUT_EV_LONG_PRESS,    /* 编码器按键长按 >800ms（返回/退出） */
    INPUT_EV_IR,            /* 红外遥控：value=按键码 */
    INPUT_EV_TILT,          /* MPU6050 倾斜：超出阈值才投递 value=±1（-1 左倾 / +1 右倾），水平不投递 */
    INPUT_EV_KEY,           /* 板载按键 K1：value=1 快速点按（<300ms，硬降） */
    INPUT_EV_KEY_HOLD,      /* K1 按住 >=300ms 后每 100ms 投递（软降：持续加速下落） */
    INPUT_EV_KEY_RELEASE,   /* K1 松开（软降结束，重置节拍基准） */
    INPUT_EV_ENC_HOLD,      /* 编码器按键按住 >=300ms 后每 100ms 投递（贪吃蛇持续加速） */
    INPUT_EV_ENC_RELEASE,   /* 编码器按键松开（持续加速结束，重置节拍基准） */
    INPUT_EV_IR_HOLD,       /* 红外 NEC Repeat（长按连发 ~110ms/次，持续加速） */
    INPUT_EV_PROTO_SET_MODE /* 串口协议 0x05 切模式（value=目标模式），
                             * 由 gameTask 统一消费，避免 protoTask 直接改应用状态 */
} InputEventType;

/* 统一输入事件 */
typedef struct {
    uint8_t  type;
    int32_t  value;
} InputEvent;

/* 初始化所有输入外设 + 创建事件队列（在 input_task 创建时调用一次） */
void Input_Init(void);

/* MPU6050 初始化：唤醒 + 量程 ±16g 验证（标准 RTOS 写法：
 * 在 inputTask 任务内调用，延时用 osDelay 让出 CPU，
 * I2C 传输持 g_i2cMutex 与 OLED 刷新互斥，失败不阻塞系统启动） */
void Input_InitMPU6050(void);

/* 周期采样所有输入并投递事件（建议 10ms 周期，由 input_task 调用） */
void Input_Scan(void);

/* 从队列取事件：timeout_tick 传 osWaitForever 阻塞等待；返回 1 成功 0 超时 */
int Input_GetEvent(InputEvent *ev, uint32_t timeout_tick);

/* 投递一个事件到队列（串口协议游戏控制命令注入用；与物理输入等价，protoTask上下文调用，线程安全）*/
void Input_PostEvent(uint8_t type, int32_t value);

/* 丢弃队列中全部积压事件（冻结暂停期调用：
 * 总线异常期间倾斜/按键事件持续入队，恢复时一次性消费会造成挡板/菜单瞬移） */
void Input_ClearEvents(void);

#endif /* __INPUT_EVENT_H */
