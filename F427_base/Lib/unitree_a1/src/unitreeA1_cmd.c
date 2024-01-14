#include "motor_msg.h"
#include <string.h>
#include <stdio.h>
#include "usart.h"
#include "unitreeA1_cmd.h"

motor_send_t cmd_left_forward;       // 左前腿修改电机数据体
motor_send_t cmd_left_back;          // 左后腿修改电机数据体

motor_send_t cmd_right_forward;      // 右前腿修改电机数据体
motor_send_t cmd_right_back;         // 右后腿修改电机数据体

motor_recv_t Date_left_forward;      // 左前腿电机接收数据体
motor_recv_t id00_left_date_forward; // 左前腿00号电机接收数据体
motor_recv_t id01_left_date_forward; // 左前腿01号电机接收数据体
motor_recv_t id02_left_date_forward; // 左前腿02号电机接收数据体

motor_recv_t Date_left_back;         // 左前腿电机接收数据体
motor_recv_t id00_left_date_back; // 左前腿00号电机接收数据体
motor_recv_t id01_left_date_back; // 左前腿01号电机接收数据体
motor_recv_t id02_left_date_back; // 左前腿02号电机接收数据体

motor_recv_t Date_right_forward;        // 右后腿电机接收数据体
motor_recv_t id00_right_date_forward;   // 右后腿00号电机接收数据体
motor_recv_t id01_right_date_forward;   // 右后腿01号电机接收数据体
motor_recv_t id02_right_date_forward;   // 右后腿02号电机接收数据体

motor_recv_t Date_right_back;        // 右后腿电机接收数据体
motor_recv_t id00_right_date_back;   // 右后腿00号电机接收数据体
motor_recv_t id01_right_date_back;   // 右后腿01号电机接收数据体
motor_recv_t id02_right_date_back;   // 右后腿02号电机接收数据体

// CRC校验位的代码
uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    uint32_t bits;
    uint32_t i;
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}

void modfiy_cmd(motor_send_t *send,uint8_t id, float Pos, float KP, float KW)
{

    send->hex_len = 34;

    send->mode = 10;
	send->id = id;
    send->K_P = KP;
    send->K_W = KW;
    send->Pos = 6.2832*9.1*2*Pos;
    send->W = 0;
    send->T = 0.0;
}


void unitreeA1_rxtx(UART_HandleTypeDef *huart)
{

if (huart == &huart1)
    {
        uint8_t Date[78];

        /*—————————————————————————————————————————左前腿代码范围————————————————————————————————————————————————*/
        uint8_t A1cmd_left_forward[34];

        // 此处为左腿电机结构体//
        cmd_left_forward.motor_send_data.head.start[0] = 0xFE;
        cmd_left_forward.motor_send_data.head.start[1] = 0xEE;
        cmd_left_forward.motor_send_data.head.motorID = cmd_left_forward.id;
        cmd_left_forward.motor_send_data.head.reserved = 0x00;

        cmd_left_forward.motor_send_data.Mdata.mode = cmd_left_forward.mode;
        cmd_left_forward.motor_send_data.Mdata.ModifyBit = 0xFF;
        cmd_left_forward.motor_send_data.Mdata.ReadBit = 0x00;
        cmd_left_forward.motor_send_data.Mdata.reserved = 0x00;
        cmd_left_forward.motor_send_data.Mdata.Modify.F = 0;
        cmd_left_forward.motor_send_data.Mdata.T = cmd_left_forward.T * 256;
        cmd_left_forward.motor_send_data.Mdata.W = cmd_left_forward.W * 128;
        cmd_left_forward.motor_send_data.Mdata.Pos = (int)((cmd_left_forward.Pos / 6.2832f) * 16384.0f);
        cmd_left_forward.motor_send_data.Mdata.K_P = cmd_left_forward.K_P * 2048;
        cmd_left_forward.motor_send_data.Mdata.K_W = cmd_left_forward.K_W * 1024;
        cmd_left_forward.motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
        cmd_left_forward.motor_send_data.Mdata.LowHzMotorCmdByte = 0;
        cmd_left_forward.motor_send_data.Mdata.Res[0] = cmd_left_forward.Res;

        cmd_left_forward.motor_send_data.CRCdata.u32 = crc32_core((uint32_t *)(&cmd_left_forward.motor_send_data), 7);

        memcpy(A1cmd_left_forward, &cmd_left_forward.motor_send_data, 34);



        HAL_UART_Transmit_DMA(&huart1, A1cmd_left_forward, 34);
        HAL_Delay(10);
        HAL_UART_Receive_DMA(&huart1, Date, 78);



        Date_left_forward.motor_recv_data.head.motorID = Date[2];
        Date_left_forward.motor_recv_data.Mdata.MError = Date[7];
        Date_left_forward.motor_recv_data.Mdata.T = Date[12] << 8 | Date[13];
        Date_left_forward.motor_recv_data.Mdata.Pos = Date[30] << 24 | Date[31] << 16 | Date[32] << 8 | Date[33];

        Date_left_forward.motor_id = Date_left_forward.motor_recv_data.head.motorID;
        Date_left_forward.MError = Date_left_forward.motor_recv_data.Mdata.MError;
        Date_left_forward.T = Date_left_forward.motor_recv_data.Mdata.T / 256;
        Date_left_forward.Pos = (int)((Date_left_forward.motor_recv_data.Mdata.Pos / 16384.0f) * 6.2832f);

        if (Date_left_forward.motor_id == 0x00)
        {
            id00_left_date_forward.motor_id = Date_left_forward.motor_id;
            id00_left_date_forward.MError = Date_left_forward.MError;
            id00_left_date_forward.T = Date_left_forward.T;
            id00_left_date_forward.Pos = Date_left_forward.Pos;
        }

        if (Date_left_forward.motor_id == 0x01)
        {
            id01_left_date_forward.motor_id = Date_left_forward.motor_id;
            id01_left_date_forward.MError = Date_left_forward.MError;
            id01_left_date_forward.T = Date_left_forward.T;
            id01_left_date_forward.Pos = Date_left_forward.Pos;
        }

        if (Date_left_forward.motor_id == 0x02)
        {
            id02_left_date_forward.motor_id = Date_left_forward.motor_id;
            id02_left_date_forward.MError = Date_left_forward.MError;
            id02_left_date_forward.T = Date_left_forward.T;
            id02_left_date_forward.Pos = Date_left_forward.Pos;
        }
		
    }

if (huart == &huart2)
    {
        uint8_t Date[78];

        /*—————————————————————————————————————————左后腿代码范围————————————————————————————————————————————————*/
        uint8_t A1cmd_left_back[34];

        // 此处为左腿电机结构体//
        cmd_left_back.motor_send_data.head.start[0] = 0xFE;
        cmd_left_back.motor_send_data.head.start[1] = 0xEE;
        cmd_left_back.motor_send_data.head.motorID = cmd_left_back.id;
        cmd_left_back.motor_send_data.head.reserved = 0x00;

        cmd_left_back.motor_send_data.Mdata.mode = cmd_left_back.mode;
        cmd_left_back.motor_send_data.Mdata.ModifyBit = 0xFF;
        cmd_left_back.motor_send_data.Mdata.ReadBit = 0x00;
        cmd_left_back.motor_send_data.Mdata.reserved = 0x00;
        cmd_left_back.motor_send_data.Mdata.Modify.F = 0;
        cmd_left_back.motor_send_data.Mdata.T = cmd_left_back.T * 256;
        cmd_left_back.motor_send_data.Mdata.W = cmd_left_back.W * 128;
        cmd_left_back.motor_send_data.Mdata.Pos = (int)((cmd_left_back.Pos / 6.2832f) * 16384.0f);
        cmd_left_back.motor_send_data.Mdata.K_P = cmd_left_back.K_P * 2048;
        cmd_left_back.motor_send_data.Mdata.K_W = cmd_left_back.K_W * 1024;
        cmd_left_back.motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
        cmd_left_back.motor_send_data.Mdata.LowHzMotorCmdByte = 0;
        cmd_left_back.motor_send_data.Mdata.Res[0] = cmd_left_back.Res;

        cmd_left_back.motor_send_data.CRCdata.u32 = crc32_core((uint32_t *)(&cmd_left_back.motor_send_data), 7);

        memcpy(A1cmd_left_back, &cmd_left_back.motor_send_data, 34);



        HAL_UART_Transmit_DMA(&huart2, A1cmd_left_back, 34);
        HAL_Delay(10);
        HAL_UART_Receive_DMA(&huart2, Date, 78);



        Date_left_back.motor_recv_data.head.motorID = Date[2];
        Date_left_back.motor_recv_data.Mdata.MError = Date[7];
        Date_left_back.motor_recv_data.Mdata.T = Date[12] << 8 | Date[13];
        Date_left_back.motor_recv_data.Mdata.Pos = Date[30] << 24 | Date[31] << 16 | Date[32] << 8 | Date[33];

        Date_left_back.motor_id = Date_left_back.motor_recv_data.head.motorID;
        Date_left_back.MError = Date_left_back.motor_recv_data.Mdata.MError;
        Date_left_back.T = Date_left_back.motor_recv_data.Mdata.T / 256;
        Date_left_back.Pos = (int)((Date_left_back.motor_recv_data.Mdata.Pos / 16384.0f) * 6.2832f);

        if (Date_left_back.motor_id == 0x00)
        {
            id00_left_date_back.motor_id = Date_left_back.motor_id;
            id00_left_date_back.MError = Date_left_back.MError;
            id00_left_date_back.T = Date_left_back.T;
            id00_left_date_back.Pos = Date_left_back.Pos;
        }

        if (Date_left_back.motor_id == 0x01)
        {
            id01_left_date_back.motor_id = Date_left_back.motor_id;
            id01_left_date_back.MError = Date_left_back.MError;
            id01_left_date_back.T = Date_left_back.T;
            id01_left_date_back.Pos = Date_left_back.Pos;
        }

        if (Date_left_back.motor_id == 0x02)
        {
            id02_left_date_back.motor_id = Date_left_back.motor_id;
            id02_left_date_back.MError = Date_left_back.MError;
            id02_left_date_back.T = Date_left_back.T;
            id02_left_date_back.Pos = Date_left_back.Pos;
        }
		
    }

if (huart == &huart6)
    {
        uint8_t Date[78];

        /*—————————————————————————————————————————右前腿代码范围————————————————————————————————————————————————————————*/
        uint8_t A1cmd_right_forward[34];

        // 此处为右腿一号电机结构体//
        cmd_right_forward.motor_send_data.head.start[0] = 0xFE;
        cmd_right_forward.motor_send_data.head.start[1] = 0xEE;
        cmd_right_forward.motor_send_data.head.motorID = cmd_right_forward.id;
        cmd_right_forward.motor_send_data.head.reserved = 0x00;

        cmd_right_forward.motor_send_data.Mdata.mode = cmd_right_forward.mode;
        cmd_right_forward.motor_send_data.Mdata.ModifyBit = 0xFF;
        cmd_right_forward.motor_send_data.Mdata.ReadBit = 0x00;
        cmd_right_forward.motor_send_data.Mdata.reserved = 0x00;
        cmd_right_forward.motor_send_data.Mdata.Modify.F = 0;
        cmd_right_forward.motor_send_data.Mdata.T = cmd_right_forward.T * 256;
        cmd_right_forward.motor_send_data.Mdata.W = cmd_right_forward.W * 128;
        cmd_right_forward.motor_send_data.Mdata.Pos = (int)((cmd_right_forward.Pos / 6.2832f) * 16384.0f);
        cmd_right_forward.motor_send_data.Mdata.K_P = cmd_right_forward.K_P * 2048;
        cmd_right_forward.motor_send_data.Mdata.K_W = cmd_right_forward.K_W * 1024;
        cmd_right_forward.motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
        cmd_right_forward.motor_send_data.Mdata.LowHzMotorCmdByte = 0;
        cmd_right_forward.motor_send_data.Mdata.Res[0] = cmd_right_forward.Res;

        cmd_right_forward.motor_send_data.CRCdata.u32 = crc32_core((uint32_t *)(&cmd_right_forward.motor_send_data), 7);

        memcpy(A1cmd_right_forward, &cmd_right_forward.motor_send_data, 34);

        HAL_UART_Transmit_DMA(&huart6, A1cmd_right_forward, 34);
        HAL_Delay(10);
        HAL_UART_Receive_DMA(&huart6, Date, 78);
		
        Date_right_forward.motor_recv_data.head.motorID = Date[2];
        Date_right_forward.motor_recv_data.Mdata.MError = Date[7];
        Date_right_forward.motor_recv_data.Mdata.T = Date[12] << 8 | Date[13];
        Date_right_forward.motor_recv_data.Mdata.Pos = Date[30] << 24 | Date[31] << 16 | Date[32] << 8 | Date[33];

        Date_right_forward.motor_id = Date_right_forward.motor_recv_data.head.motorID;
        Date_right_forward.MError = Date_right_forward.motor_recv_data.Mdata.MError;
        Date_right_forward.T = Date_right_forward.motor_recv_data.Mdata.T / 256;
        Date_right_forward.Pos = (int)((Date_right_forward.motor_recv_data.Mdata.Pos / 16384.0f) * 6.2832f);

        if (Date_right_forward.motor_id == 0x00)
        {
            id00_right_date_forward.motor_id = Date_right_forward.motor_id;
            id00_right_date_forward.MError = Date_right_forward.MError;
            id00_right_date_forward.T = Date_right_forward.T;
            id00_right_date_forward.Pos = Date_right_forward.Pos;
            
        }

        if (Date_right_forward.motor_id == 0x01)
        {
            id01_right_date_forward.motor_id = Date_right_forward.motor_id;
            id01_right_date_forward.MError = Date_right_forward.MError;
            id01_right_date_forward.T = Date_right_forward.T;
            id01_right_date_forward.Pos = Date_right_forward.Pos;
            
        }

        if (Date_right_forward.motor_id == 0x02)
        {
            id02_right_date_forward.motor_id = Date_right_forward.motor_id;
            id02_right_date_forward.MError = Date_right_forward.MError;
            id02_right_date_forward.T = Date_right_forward.T;
            id02_right_date_forward.Pos = Date_right_forward.Pos;
            
        }
    }


if (huart == &huart3)
    {
        uint8_t Date[78];

        /*—————————————————————————————————————————右后腿代码范围————————————————————————————————————————————————————————*/
        uint8_t A1cmd_right_back[34];

        // 此处为右腿一号电机结构体//
        cmd_right_back.motor_send_data.head.start[0] = 0xFE;
        cmd_right_back.motor_send_data.head.start[1] = 0xEE;
        cmd_right_back.motor_send_data.head.motorID = cmd_right_back.id;
        cmd_right_back.motor_send_data.head.reserved = 0x00;

        cmd_right_back.motor_send_data.Mdata.mode = cmd_right_back.mode;
        cmd_right_back.motor_send_data.Mdata.ModifyBit = 0xFF;
        cmd_right_back.motor_send_data.Mdata.ReadBit = 0x00;
        cmd_right_back.motor_send_data.Mdata.reserved = 0x00;
        cmd_right_back.motor_send_data.Mdata.Modify.F = 0;
        cmd_right_back.motor_send_data.Mdata.T = cmd_right_back.T * 256;
        cmd_right_back.motor_send_data.Mdata.W = cmd_right_back.W * 128;
        cmd_right_back.motor_send_data.Mdata.Pos = (int)((cmd_right_back.Pos / 6.2832f) * 16384.0f);
        cmd_right_back.motor_send_data.Mdata.K_P = cmd_right_back.K_P * 2048;
        cmd_right_back.motor_send_data.Mdata.K_W = cmd_right_back.K_W * 1024;
        cmd_right_back.motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
        cmd_right_back.motor_send_data.Mdata.LowHzMotorCmdByte = 0;
        cmd_right_back.motor_send_data.Mdata.Res[0] = cmd_right_back.Res;

        cmd_right_back.motor_send_data.CRCdata.u32 = crc32_core((uint32_t *)(&cmd_right_back.motor_send_data), 7);

        memcpy(A1cmd_right_back, &cmd_right_back.motor_send_data, 34);

        HAL_UART_Transmit_DMA(&huart3, A1cmd_right_back, 34);
        HAL_Delay(10);
        HAL_UART_Receive_DMA(&huart3, Date, 78);
		
        Date_right_back.motor_recv_data.head.motorID = Date[2];
        Date_right_back.motor_recv_data.Mdata.MError = Date[7];
        Date_right_back.motor_recv_data.Mdata.T = Date[12] << 8 | Date[13];
        Date_right_back.motor_recv_data.Mdata.Pos = Date[30] << 24 | Date[31] << 16 | Date[32] << 8 | Date[33];

        Date_right_back.motor_id = Date_right_back.motor_recv_data.head.motorID;
        Date_right_back.MError = Date_right_back.motor_recv_data.Mdata.MError;
        Date_right_back.T = Date_right_back.motor_recv_data.Mdata.T / 256;
        Date_right_back.Pos = (int)((Date_right_back.motor_recv_data.Mdata.Pos / 16384.0f) * 6.2832f);

        if (Date_right_back.motor_id == 0x00)
        {
            id00_right_date_back.motor_id = Date_right_back.motor_id;
            id00_right_date_back.MError = Date_right_back.MError;
            id00_right_date_back.T = Date_right_back.T;
            id00_right_date_back.Pos = Date_right_back.Pos;
        }

        if (Date_right_back.motor_id == 0x01)
        {
            id01_right_date_back.motor_id = Date_right_back.motor_id;
            id01_right_date_back.MError = Date_right_back.MError;
            id01_right_date_back.T = Date_right_back.T;
            id01_right_date_back.Pos = Date_right_back.Pos;
        }

        if (Date_right_back.motor_id == 0x02)
        {
            id02_right_date_back.motor_id = Date_right_back.motor_id;
            id02_right_date_back.MError = Date_right_back.MError;
            id02_right_date_back.T = Date_right_back.T;
            id02_right_date_back.Pos = Date_right_back.Pos;
            
        }
    }
}

