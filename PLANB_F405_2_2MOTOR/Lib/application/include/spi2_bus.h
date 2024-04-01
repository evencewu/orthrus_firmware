#ifndef __SPI2_BUS_H__
#define __SPI2_BUS_H__

#include "spi2_typedef.h"
#include <stdint.h>

typedef struct
{
    uint8_t start[2];        //包头FF和FE
    uint8_t motorID;  // 电机ID  0,1,2,3 ...   0xBB 表示向所有电机广播（此时无返回）
    uint8_t mode;      // 关节模式选择

    int16_t T;     // 期望关节的输出力矩（电机本身的力矩）x256, 7 + 8 描述
    int16_t W;     // 期望关节速度 （电机本身的速度） x128,       8 + 7描述
    int32_t Pos; // 期望关节位置 x 16384/6.2832, 14位编码器（主控0点修正，电机关节还是以编码器0点为准）

    int16_t K_P; // 关节刚度系数 x2048  4+11 描述
    int16_t K_W; // 关节速度系数 x1024  5+10 描述

    int32_t SumCheck;
}A1PackageSpiTx;

A1PackageSpiTx motor[12];

#endif
