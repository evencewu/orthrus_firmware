#include "bsp_can.h"
#include "RC.h"
#include "unitreeA1_cmd.h"

fchange_t data4bytes;
gyro_param gyro_pitch = {0};

extern motor_recv_t data_leg[2];
extern motor_send_t cmd_leg_temp[2][3];
extern motor_recv_t data_motor[2][3];

uint32_t can1_rx_stdid;
uint8_t can1_rx_data[8];

extern Leg leg[2];

void CAN_InitArgument(void)
{
    CAN_Enable(&hcan1);
    CAN_Enable(&hcan2);
}

void CAN_Enable(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_Start(hcan);                                             // 对can进行激活
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 使能can接收中断
    CANFilter_Enable(hcan);                                          // 使能滤波器
}

void CANFilter_Enable(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter1;
    CAN_FilterTypeDef filter2;
    if (hcan->Instance == CAN1)
    {
        filter1.FilterActivation = ENABLE;
        filter1.FilterBank = 0U;
        filter1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        filter1.FilterIdHigh = 0x0000;
        filter1.FilterIdLow = 0x0000;
        filter1.FilterMaskIdHigh = 0x0000;
        filter1.FilterMaskIdLow = 0x0000;
        filter1.FilterMode = CAN_FILTERMODE_IDMASK;
        filter1.FilterScale = CAN_FILTERSCALE_32BIT;
        filter1.SlaveStartFilterBank = 14;

        HAL_CAN_ConfigFilter(&hcan1, &filter1);
    }
    if (hcan->Instance == CAN2)
    {
        filter2.FilterActivation = ENABLE;
        filter2.FilterBank = 14;
        filter2.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        filter2.FilterIdHigh = 0x0000;
        filter2.FilterIdLow = 0x0000;
        filter2.FilterMaskIdHigh = 0x0000;
        filter2.FilterMaskIdLow = 0x0000;
        filter2.FilterMode = CAN_FILTERMODE_IDMASK;
        filter2.FilterScale = CAN_FILTERSCALE_32BIT;
        filter2.SlaveStartFilterBank = 14;

        HAL_CAN_ConfigFilter(&hcan2, &filter2);
    }
}

union FloatConverter
{
    uint8_t bytes[4];
    float floatValue;
}converter;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Message can1_rx_message, can2_rx_message;
    CAN_RxHeaderTypeDef Can1RxHeader, Can2RxHeader;
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Can1RxHeader, can1_rx_message.Data);

        can1_rx_stdid = Can1RxHeader.StdId;
        can1_rx_data[0] = can1_rx_message.Data[0];
        can1_rx_data[1] = can1_rx_message.Data[1];
        can1_rx_data[2] = can1_rx_message.Data[2];
        can1_rx_data[3] = can1_rx_message.Data[3];
        can1_rx_data[4] = can1_rx_message.Data[4];
        can1_rx_data[5] = can1_rx_message.Data[5];
        can1_rx_data[6] = can1_rx_message.Data[6];
        can1_rx_data[7] = can1_rx_message.Data[7];
        if (Can1RxHeader.StdId >= 0x20 && Can1RxHeader.StdId <= 0x32)
        {
            int flag_num = (int)(Can1RxHeader.StdId - 0x20);
            if (flag_num % 3 == 0)
            {
                converter.bytes[0] = can1_rx_data[0];
                converter.bytes[1] = can1_rx_data[1];
                converter.bytes[2] = can1_rx_data[2];
                converter.bytes[3] = can1_rx_data[3];

                leg[flag_num / 9].motor_can[(flag_num % 9) / 3].cmd_leg_temp.K_P = converter.floatValue;

                converter.bytes[0] = can1_rx_data[4];
                converter.bytes[1] = can1_rx_data[5];
                converter.bytes[2] = can1_rx_data[6];
                converter.bytes[3] = can1_rx_data[7];

                leg[flag_num / 9].motor_can[(flag_num % 9) / 3].cmd_leg_temp.K_W = converter.floatValue;
            }
            else if (flag_num % 3 == 1)
            {
                converter.bytes[0] = can1_rx_data[0];
                converter.bytes[1] = can1_rx_data[1];
                converter.bytes[2] = can1_rx_data[2];
                converter.bytes[3] = can1_rx_data[3];

                leg[flag_num / 9].motor_can[(flag_num % 9) / 3].cmd_leg_temp.W = converter.floatValue;

                converter.bytes[0] = can1_rx_data[4];
                converter.bytes[1] = can1_rx_data[5];
                converter.bytes[2] = can1_rx_data[6];
                converter.bytes[3] = can1_rx_data[7];

                leg[flag_num / 9].motor_can[(flag_num % 9) / 3].cmd_leg_temp.T = converter.floatValue;
            }
            else if (flag_num % 3 == 2)
            {
                converter.bytes[0] = can1_rx_data[0];
                converter.bytes[1] = can1_rx_data[1];
                converter.bytes[2] = can1_rx_data[2];
                converter.bytes[3] = can1_rx_data[3];

                leg[flag_num / 9].motor_can[(flag_num % 9) / 3].cmd_leg_temp.Pos = converter.floatValue;
                leg[flag_num / 9].motor_can[(flag_num % 9) / 3].cmd_leg_temp.mode = (uint32_t)can1_rx_message.Data[4];

                leg[flag_num / 9].motor_can[(flag_num % 9) / 3].Checksum = (can1_rx_message.Data[6] | can1_rx_message.Data[7] << 8);
            }
        }
    }
    if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &Can2RxHeader, can2_rx_message.Data);
        switch (Can2RxHeader.StdId)
        {
        }
    }
}

uint8_t send_succeed = 0;
/*CAN发送1个4个元素的uint8_t数组，共32位(从float类型转换而来)*/
void CAN_send_data(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t data[])
{
    CAN_TxHeaderTypeDef tx_message;
    uint8_t can_send_data[8];
    uint32_t send_mail_box;
    tx_message.StdId = id;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    can_send_data[0] = data[0];
    can_send_data[1] = data[1];
    can_send_data[2] = data[2];
    can_send_data[3] = data[3];
    can_send_data[4] = data[4];
    can_send_data[5] = data[5];
    can_send_data[6] = data[6];
    can_send_data[7] = data[7];

    // HAL_CAN_AddTxMessage(hcan, &tx_message, can_send_data, &send_mail_box);
    if (HAL_CAN_AddTxMessage(hcan, &tx_message, can_send_data, &send_mail_box) == HAL_OK)
    {
        send_succeed = 1;
    }
    else
    {
        send_succeed = 2;
    }
}
