#ifndef __SPI_DEAL__
#define __SPI_DEAL__

#include "data_def.h"

typedef __packed struct
{
    int8_t start[2];
    int8_t motorID;
	int8_t mode;
    uint8_t LegID;      // 腿ID

	int16_t T;
	int16_t W;
	int32_t Pos;

    int16_t K_P; // 关节刚度系数 x2048  4+11 描述
    int16_t K_W; // 关节速度系数 x1024  5+10 描述

    int32_t SumCheck;  //四位和校验

} modf_temp_buf;

void send_A1msgToEcat(int leg_id);
void EcatChat_Init(void);
static void sbus_to_rc_spi(volatile const uint8_t *sbus_buf, A1PackageSpiRx *modf_buf);
 void DateCheck_DateModfy(A1PackageSpiRx *modf_buf);

#endif


