#include "main.h"
#include "spi.h"
#include "string.h"

#include "unitreeA1_msg.h"
#include "unitreeA1_recv.h"
#include "unitreeA1_cmd.h"

#include "data_def.h"
#include "spi_deal.h"

A1msgTxTransform a1_msg_tx_transform;
A1msgRxTransform a1_msg_rx_transform;

A1PackageSpiTx spi_tx_data[4][3];
A1PackageSpiRx spi_rx_data[4][3];

//
volatile uint8_t RxComplete = 0;

extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

A1PackageSpiRx RXA1cmd;

extern motor_recv_t data_motor[4][3];
extern motor_recv_t data_leg[4];
extern motor_send_t cmd_leg[4];

void DateCheck_DateModfy(A1PackageSpiRx *modf_buf)
{
    if (modf_buf->SumCheck == modf_buf->start[0] + modf_buf->start[1] + modf_buf->LegID + modf_buf->motorID + modf_buf->mode + modf_buf->T + modf_buf->W + modf_buf->Pos + modf_buf->K_P + modf_buf->K_W)
    {
        if (modf_buf->LegID == 0)
        {
            uint8_t leg_id = 0;
            cmd_leg[leg_id].id = modf_buf->motorID;
            cmd_leg[leg_id].mode = modf_buf->mode;
            cmd_leg[leg_id].K_P = modf_buf->K_P;
            cmd_leg[leg_id].K_W = modf_buf->K_W;
            cmd_leg[leg_id].Pos = modf_buf->Pos;
            cmd_leg[leg_id].W = modf_buf->W;
            cmd_leg[leg_id].T = modf_buf->T;
        }
        if (modf_buf->LegID == 1)
        {
            uint8_t leg_id = 1;
            cmd_leg[leg_id].id = modf_buf->motorID;
            cmd_leg[leg_id].mode = modf_buf->mode;
            cmd_leg[leg_id].K_P = modf_buf->K_P;
            cmd_leg[leg_id].K_W = modf_buf->K_W;
            cmd_leg[leg_id].Pos = modf_buf->Pos;
            cmd_leg[leg_id].W = modf_buf->W;
            cmd_leg[leg_id].T = modf_buf->T;
        }

        if (modf_buf->LegID == 2)
        {
            uint8_t leg_id = 2;
            cmd_leg[leg_id].id = modf_buf->motorID;
            cmd_leg[leg_id].mode = modf_buf->mode;
            cmd_leg[leg_id].K_P = modf_buf->K_P;
            cmd_leg[leg_id].K_W = modf_buf->K_W;
            cmd_leg[leg_id].Pos = modf_buf->Pos;
            cmd_leg[leg_id].W = modf_buf->W;
            cmd_leg[leg_id].T = modf_buf->T;
        }

        if (modf_buf->LegID == 3)
        {
            uint8_t leg_id = 3;
            cmd_leg[leg_id].id = modf_buf->motorID;
            cmd_leg[leg_id].mode = modf_buf->mode;
            cmd_leg[leg_id].K_P = modf_buf->K_P;
            cmd_leg[leg_id].K_W = modf_buf->K_W;
            cmd_leg[leg_id].Pos = modf_buf->Pos;
            cmd_leg[leg_id].W = modf_buf->W;
            cmd_leg[leg_id].T = modf_buf->T;
        }
    }
}
void SPI_RXdate(A1PackageSpiRx *modf_buf, uint8_t *spi_sbus_buf)
{
    modf_buf->start[0] = spi_sbus_buf[0];
    modf_buf->start[1] = spi_sbus_buf[1];
    modf_buf->LegID = spi_sbus_buf[2];
    modf_buf->motorID = spi_sbus_buf[3];
    modf_buf->mode = spi_sbus_buf[4];
    modf_buf->T = spi_sbus_buf[5] << 8 | spi_sbus_buf[6];
    modf_buf->W = spi_sbus_buf[7] << 8 | spi_sbus_buf[8];
    modf_buf->Pos = spi_sbus_buf[9] << 24 | spi_sbus_buf[10] << 16 | spi_sbus_buf[11] << 8 | spi_sbus_buf[12];
    modf_buf->K_P = spi_sbus_buf[13] << 8 | spi_sbus_buf[14];
    modf_buf->K_W = spi_sbus_buf[15] << 8 | spi_sbus_buf[16];
    modf_buf->SumCheck = spi_sbus_buf[17] << 24 | spi_sbus_buf[18] << 16 | spi_sbus_buf[19] << 8 | spi_sbus_buf[20];
    DateCheck_DateModfy(modf_buf);
}

// 拉底片选
void ecat_NS_L(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}
// 拉高片选
void ecat_NS_H(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

/// @brief 主从同步读写
/// @param
void MASTER_Synchro(void)
{
    uint8_t txbety = 0xD2;
    uint8_t rxbety = 0x00;

    do
    {
        if (HAL_SPI_TransmitReceive_IT(&hspi2, &txbety, &rxbety, 1) != HAL_OK)
        {
            Error_Handler();
        }

        while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
        {
        }
    } while (rxbety != txbety);
}

uint8_t spi2_tx_original_data[41];
uint8_t spi2_rx_original_data[41];
uint8_t spi2_tx_data_u8[21];
uint8_t spi2_rx_data_u8[21];

/// @brief SPI 数据交换
/// @param leg_id 对应腿的ID
void SPI_TRANSMIT(int leg_id, int motor_id)
{
    ecat_NS_L();
    MASTER_Synchro();

    memcpy(spi2_tx_data_u8, &spi_tx_data[leg_id][motor_id], 21);
    MotorTxDeal(spi2_tx_data_u8, spi2_tx_original_data);
    if (HAL_SPI_TransmitReceive_IT(&hspi2, spi2_tx_original_data, spi2_rx_original_data, 41) != HAL_OK)
    {
        Error_Handler();
    }

    for (int i = 0; i < 21; i++)
    {
        if (spi2_rx_original_data[i] == 0xD2 && spi2_rx_original_data[i + 1] == 0xFE)
        {
            for (int j = 0; j < 21; j++)
            {
                spi2_rx_data_u8[j] = spi2_rx_original_data[i + j];
            }
            SpiMotorRxArchive(spi2_rx_data_u8);
        }
    }

    while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
    {
    }

    ecat_NS_H();
}

/// @brief 处理21字节uint8列表，将电机数据归档到usart发送数据列表
/// @param rx_recv_data
void SpiMotorRxArchive(uint8_t *rx_recv_data)
{
    memcpy(&a1_msg_rx_transform.u8[0], rx_recv_data, 21);
    int leg_id = a1_msg_rx_transform.a1msg.LegID;
    int motor_id = a1_msg_rx_transform.a1msg.motorID;
    uint32_t sum = a1_msg_rx_transform.a1msg.start[0] + a1_msg_rx_transform.a1msg.start[1] + a1_msg_rx_transform.a1msg.LegID + a1_msg_rx_transform.a1msg.motorID + a1_msg_rx_transform.a1msg.mode + a1_msg_rx_transform.a1msg.Pos + a1_msg_rx_transform.a1msg.T + a1_msg_rx_transform.a1msg.W + a1_msg_rx_transform.a1msg.K_P + a1_msg_rx_transform.a1msg.K_W;

    if (a1_msg_rx_transform.a1msg.SumCheck == sum)
    {
        if (leg_id < 4 && leg_id >= 0 && motor_id < 4 && motor_id >= 0)
        {
            memcpy(&spi_rx_data[leg_id][motor_id], &a1_msg_rx_transform.a1msg.start[0], 21);
        }
    }
}

/// @brief 21字节数据转为双倍41字节数据
/// @param data motor spi 的21字节数据
/// @param original_data 被发送出去的41字节数据
void MotorTxDeal(uint8_t *data, uint8_t *original_data)
{
    for (int i = 0; i < 20; i++)
    {
        *(original_data + i) = *(data + i + 1);
    }

    for (int i = 0; i < 21; i++)
    {
        *(original_data + i + 20) = *(data + i);
    }

    *(original_data) = 0xFE;
    *(original_data + 20) = 0xD2;
    *(original_data + 21) = 0xFE;
}

int usart_motor_send_flag = 0;
// 定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        if (usart_motor_send_flag < 5)
        {
            usart_motor_send_flag++;
        }
        else
        {
            usart_motor_send_flag = 0;
        }

        // if (usart_motor_send_flag == 1)
        //{
        //     HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
        // }
        
        //unitreeA1_enable(usart_motor_send_flag%2,usart_motor_send_flag/2);

        unitreeA1_tx(usart_motor_send_flag%2,usart_motor_send_flag/2);
        //unitreeA1_tx(usart_motor_send_flag%2,usart_motor_send_flag/2);
        //unitreeA1_tx(0);
    }
}