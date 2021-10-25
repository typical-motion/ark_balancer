#ifndef __H_PROTOCOL
#define __H_PROTOCOL

#include <inttypes.h>
#include <stdlib.h>
#define __packed __attribute__((packed))

/**
 * 帧头
 **/
typedef struct
{
    uint16_t start;     //标识符
    uint8_t id;         //消息代号
    uint16_t length;    //消息长度
} __packed FrameHeader; //帧头

/**
 * 通用回音数据包 
 * device: 当发送者为下位机时为 0, 上位机时为 1
 * @id 0xA0
 * @from 下位机, 上位机
 **/
typedef struct
{
    uint8_t device;   // 来源设备 ID
    uint8_t data[32]; // 数据
} __packed PacketGenericEcho;

/**
 * 陀螺仪反馈数据包 
 * @id 0xA1
 * @from 下位机
 **/
typedef struct
{
    float orientationW, orientationX, orientationY, orientationZ;
    uint16_t accelerationX, accelerationY, accelerationZ;
} __packed PacketIMUSensorFeedback;

/**
 * 数据帧
 **/
typedef struct
{
    FrameHeader Header; //帧头
    union
    {
        PacketIMUSensorFeedback IMUSensorFeedback;
        PacketGenericEcho genericEcho;
    } __packed Body;
    uint32_t CRC32; //CRC16校验码
} __packed Frame;   //整个数据帧
#endif