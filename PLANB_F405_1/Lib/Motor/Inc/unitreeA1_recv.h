#ifndef __UNITREEA1_RECV_H__
#define __UNITREEA1_RECV_H__

#include "unitreeA1_msg.h"
#include "unitreeA1_cmd.h"
#include "bsp_dma.h"

#define SBUS_RX_BUF_NUM 128u  //实际接收128位数据

#define RC_FRAME_LENGTH 78u   //目标接收78数据

typedef __packed struct
{
    uint8_t motorID;
    uint8_t mode;
    int8_t Temp;     // 电机当前平均温度
	
    uint8_t MError;  // 电机错误 标识

	uint16_t T;
	uint16_t W;
	uint16_t Acc;
	uint32_t Pos;

} A1_buf;//17

typedef struct MotorCan 
{
	motor_send_t cmd_leg_confirm;
    motor_send_t cmd_leg_temp;
    uint16_t Checksum;
	uint16_t sum_cheak_temp;
}MotorCan;

typedef struct Leg
{
    MotorCan motor_can[3];
	A1_buf a1_buf;
}Leg;

extern Leg leg[4];

void remote_control_init(void);

void unitreeA1_rx(int leg_id);
static void sbus_to_rc(volatile const uint8_t *sbus_buf, A1_buf *a1_buf);

void sum_cheak();

void SetMotorMsg(int motor_id);


#endif
