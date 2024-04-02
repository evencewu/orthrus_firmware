#ifndef __DATE_DEF__
#define __DATE_DEF__

#include <stdint.h>
/*
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

    int32_t SumCheck;  //四位和校验

}A1PackageSpiRx;//20
typedef struct
{
    uint8_t start[2];
    uint8_t LegID;

    uint8_t MotorID_0;
    int16_t T0;      // 当前实际电机输出力矩       7 + 8 描述
    int16_t W0; // 当前实际电机速度（高速）   8 + 7 描述
    int16_t Acc0;    // 电机转子加速度       15+0 描述  惯量较小
    int32_t Pos0;  // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
    int32_t SumCheck0;  //四位和校验

    uint8_t MotorID_1;
    int16_t T1;      // 当前实际电机输出力矩       7 + 8 描述
    int16_t W1; // 当前实际电机速度（高速）   8 + 7 描述
    int16_t Acc1;    // 电机转子加速度       15+0 描述  惯量较小
    int32_t Pos1;  // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
    int32_t SumCheck1;  //四位和校验

    uint8_t MotorID_2;
    int16_t T2;      // 当前实际电机输出力矩       7 + 8 描述
    int16_t W2; // 当前实际电机速度（高速）   8 + 7 描述
    int16_t Acc2;    // 电机转子加速度       15+0 描述  惯量较小
    int32_t Pos2;  // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
    int32_t SumCheck2;  //四位和校验
	
}A1PackageSpiTx;//48位一条腿

*/
typedef struct
{

    uint8_t start[2];        //包头FF和FE
    uint8_t LegID;      // 腿ID
    uint8_t motorID;  // 电机ID  0,1,2,3 ...   0xBB 表示向所有电机广播（此时无返回）
    uint8_t mode;      // 关节模式选择

    int16_t T;     // 期望关节的输出力矩（电机本身的力矩）x256, 7 + 8 描述
    int16_t W;     // 期望关节速度 （电机本身的速度） x128,       8 + 7描述
    int32_t Pos; // 期望关节位置 x 16384/6.2832, 14位编码器（主控0点修正，电机关节还是以编码器0点为准）

    int16_t K_P; // 关节刚度系数 x2048  4+11 描述
    int16_t K_W; // 关节速度系数 x1024  5+10 描述

    int32_t SumCheck;  //四位和校验

}A1PackageSpiRx;//20


typedef struct
{
    uint8_t start[2];
    uint8_t LegID;
    uint8_t MotorID;


    int16_t T;      // 当前实际电机输出力矩       7 + 8 描述
    int16_t W; // 当前实际电机速度（高速）   8 + 7 描述
    int16_t Acc;    // 电机转子加速度       15+0 描述  惯量较小
    int32_t Pos;  // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
  
    int32_t SumCheck;  //四位和校验
	
}A1PackageSpiTx;//16位一个电机

#endif
