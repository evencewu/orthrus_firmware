#include "main.h"
#include <string.h>
#include <stdio.h>
#include "usart.h"
#include "unitreeA1_cmd.h"
#include "unitreeA1_msg.h"

#define LF 0
#define LB 1
#define RF 2
#define RB 3

motor_send_t cmd_leg[4];
motor_recv_t data_leg[4];
/*
void DataSet(int leg_id)
{
    cmd_leg[leg_id].hex_len = 34;

    cmd_leg[leg_id].id = 2;

    cmd_leg[leg_id].mode = 10;
    cmd_leg[leg_id].K_P = 0;
    cmd_leg[leg_id].K_W = 0;
    cmd_leg[leg_id].Pos = 0;
    cmd_leg[leg_id].W = 0;
    cmd_leg[leg_id].T = 0;
}
*/
void unitreeA1_tx(int leg_id)
{
    /*—————————————————————————————————————————左前腿代码范围————————————————————————————————————————————————*/
    uint8_t A1cmd[34];


    // 此处为左腿电机结构体//
    cmd_leg[leg_id].motor_send_data.head.start[0] = 0xFE;
    cmd_leg[leg_id].motor_send_data.head.start[1] = 0xEE;
    cmd_leg[leg_id].motor_send_data.head.motorID = cmd_leg[leg_id].id;
    cmd_leg[leg_id].motor_send_data.head.reserved = 0x00;

    cmd_leg[leg_id].motor_send_data.Mdata.mode = cmd_leg[leg_id].mode;
    cmd_leg[leg_id].motor_send_data.Mdata.ModifyBit = 0xFF;
    cmd_leg[leg_id].motor_send_data.Mdata.ReadBit = 0x00;
    cmd_leg[leg_id].motor_send_data.Mdata.reserved = 0x00;
    cmd_leg[leg_id].motor_send_data.Mdata.Modify.F = 0;
    cmd_leg[leg_id].motor_send_data.Mdata.T = (float)cmd_leg[leg_id].T * 256;
    cmd_leg[leg_id].motor_send_data.Mdata.W = (float)cmd_leg[leg_id].W * 128;
    cmd_leg[leg_id].motor_send_data.Mdata.Pos = (int)(((float)cmd_leg[leg_id].Pos / 6.2832f) * 16384.0f);
    cmd_leg[leg_id].motor_send_data.Mdata.K_P = (float)cmd_leg[leg_id].K_P * 2048;
    cmd_leg[leg_id].motor_send_data.Mdata.K_W = (float)cmd_leg[leg_id].K_W * 1024;
    cmd_leg[leg_id].motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
    cmd_leg[leg_id].motor_send_data.Mdata.LowHzMotorCmdByte = 0;
    cmd_leg[leg_id].motor_send_data.Mdata.Res[0] = cmd_leg[leg_id].Res;

    cmd_leg[leg_id].motor_send_data.CRCdata.u32 = crc32_core((uint32_t *)(&cmd_leg[leg_id].motor_send_data), 7);

    memcpy(A1cmd, &cmd_leg[leg_id].motor_send_data, 34);

    switch (leg_id)
    {
    case 0:
        HAL_UART_Transmit(&huart1, A1cmd, 34, 0x01);
        break;
    case 1:
        HAL_UART_Transmit(&huart2, A1cmd, 34, 0x01);
        break;
    case 2:
        HAL_UART_Transmit(&huart6, A1cmd, 34, 0x01);
        break;
    case 3:
        HAL_UART_Transmit(&huart3, A1cmd, 34, 0x01);
        break;
    }
}

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
