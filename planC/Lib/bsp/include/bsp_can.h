#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f4xx.h"
#include "can.h"

typedef struct
{
	uint8_t Data[8];
} CAN_Message;

typedef enum
{
	//    L_00_A1Date_W_T_Pos    =0x200,
	//    L_01_A1Date_W_T_Pos    =0x201,
	//    R_00_A1Date_W_T_Pos    =0x202,
	//    R_01_A1Date_W_T_Pos    =0x203,
	//    INS_Date_00            =0x204,
	//	INS_Date_01            =0x205,
	//	INS_accel              =0x206,

	// 左后腿发送数据can包
	LB_00_A1Date_T_P = 0x40,
	LB_00_A1Date_W_Acc = 0x41,
	LB_01_A1Date_T_P = 0x42,
	LB_01_A1Date_W_Acc = 0x43,
	LB_02_A1Date_T_P = 0x44,
	LB_02_A1Date_W_Acc = 0x45,

	// 左前腿发送数据can包
	LF_00_A1Date_T_P = 0x46,
	LF_00_A1Date_W_Acc = 0x47,
	LF_01_A1Date_T_P = 0x48,
	LF_01_A1Date_W_Acc = 0x49,
	LF_02_A1Date_T_P = 0x4A,
	LF_02_A1Date_W_Acc = 0x4B,
} can_msg_id1;

// 陀螺仪数据解算用到
typedef union
{
	uint8_t c[4];
	float f;
	uint32_t d;
} fchange_t;

typedef struct
{
	float pit_speed;
	float angle_z;
	float yaw_speed;
	float yaw_speed_last;
	float pit_speed_last;
} gyro_param;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern gyro_param gyro_pitch;

void CAN_InitArgument(void);
void CAN_Enable(CAN_HandleTypeDef *hcan);
void CANFilter_Enable(CAN_HandleTypeDef *hcan);
void gyro_data_handle(fchange_t *ptrr, float *gyro, uint8_t RxData[]);
void CAN_send_data(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t data[]);
void Float_to_Byte(float a, float b, unsigned char byte[]);

#endif


