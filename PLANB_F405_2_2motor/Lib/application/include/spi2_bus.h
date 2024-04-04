#ifndef __SPI2_BUS_H__
#define __SPI2_BUS_H__

#include "spi2_typedef.h"
#include <stdint.h>

typedef struct 
{

    uint8_t start[2]; // 包头FF和FE
    uint8_t leg_id;   // 腿ID
    uint8_t motor_id; // 电机ID  0,1,2,3 ...   0xBB 表示向所有电机广播（此时无返回）
    uint8_t mode;     // 关节模式选择

    int16_t T;   // 期望关节的输出力矩（电机本身的力矩）x256, 7 + 8 描述
    int16_t W;   // 期望关节速度 （电机本身的速度） x128,       8 + 7描述
    int32_t Pos; // 期望关节位置 x 16384/6.2832, 14位编码器（主控0点修正，电机关节还是以编码器0点为准）

    int16_t K_P; // 关节刚度系数 x2048  4+11 描述
    int16_t K_W; // 关节速度系数 x1024  5+10 描述

    int32_t SumCheck; // 四位和校验

}__attribute__((__packed__)) A1PackageSpiTx; // 21

typedef struct
{
    uint8_t start[2];
    uint8_t leg_id;
    uint8_t motor_id;

    uint8_t mode;
    int8_t Temp;    // 电机当前平均温度
    uint8_t MError; // 电机错误 标识
    // 7
    int16_t T; // 当前实际电机输出力矩       7 + 8 描述
    int16_t W; // 当前实际电机速度（高速）   8 + 7 描述

    int16_t Acc; // 电机转子加速度       15+0 描述  惯量较小
    int32_t Pos; // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）

    int32_t SumCheck; // 四位和校验

}__attribute__((__packed__)) A1PackageSpiRx; // 21位一个电机

typedef union 
{
    A1PackageSpiTx a1msg;
    uint8_t u8[21];
}A1msgTxTransform;

typedef union 
{
    A1PackageSpiRx a1msg;
    uint8_t u8[21];
}A1msgRxTransform;

extern A1PackageSpiTx motor_tx[4][3];
extern A1PackageSpiRx motor_rx[4][3];

void GetMotorMsg(uint8_t *input);

#endif
