#include "main.h"
#include "spi.h"
#include "unitreeA1_msg.h"
#include "unitreeA1_recv.h"
#include "unitreeA1_cmd.h"
#include "data_def.h"
#include "string.h"
#include "spi_deal.h"


volatile uint8_t RxComplete = 0;

extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

A1PackageSpiTx A1date[4][3];
A1PackageSpiRx RXA1cmd;

extern motor_recv_t data_motor[4][3];
extern motor_recv_t data_leg[4];
extern motor_send_t cmd_leg[4];


void DateCheck_DateModfy (A1PackageSpiRx *modf_buf)
{
    if(modf_buf->SumCheck == modf_buf->start[0] + modf_buf->start[1] + modf_buf->LegID + modf_buf->motorID +modf_buf->mode + modf_buf->T + modf_buf->W + modf_buf->Pos + modf_buf->K_P + modf_buf->K_W)
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
void SPI_RXdate(A1PackageSpiRx *modf_buf,uint8_t *spi_sbus_buf)
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


void ecat_NS_L (void)
{
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);

}

void ecat_NS_H (void)
{
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
}


void sendToEcat(int leg_id)
{   
    ecat_NS_L();
    uint8_t rxdate[21];
    uint8_t txdate[21];//= {1,1,1,1,1,1,1,1,11,1,1,1,1,1,1,1,1};
    memcpy(txdate ,&A1date[leg_id][data_leg[leg_id].motor_recv_data.head.motorID],21);
    HAL_SPI_TransmitReceive(&hspi2,txdate,rxdate,21,0x10);

    SPI_RXdate(&RXA1cmd,rxdate);
    ecat_NS_H();
}

void SPI_TRANSMIT(int leg_id)
{   
    ecat_NS_L();
    MASTER_Synchro();
    uint8_t txdate[21]= {54,11,91,13,87,55,1,11,11,98,1,1,1,1,1,1,1};
    //memcpy(txdate ,&A1date[leg_id][data_leg[leg_id].motor_recv_data.head.motorID],21);

    if(HAL_SPI_Transmit_IT(&hspi2,txdate,21) != HAL_OK)
    {
        Error_Handler();
    }
    while ( HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
    {}
    ecat_NS_H();
}

void MASTER_Synchro(void)
{
    uint8_t txbety = 0xD2;
    uint8_t rxbety = 0x00;

    do
    {
        if (HAL_SPI_TransmitReceive_IT(&hspi2,&txbety,&rxbety,1) != HAL_OK)
        {
            Error_Handler();
        }

    while ( HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
    {}
    } while (rxbety != txbety);
}

void SPI_RECEIVE(void)
{   ecat_NS_L();
    MASTER_Synchro();
    uint8_t rxdate[21];
    HAL_SPI_Receive_IT(&hspi2,rxdate,21);
    SPI_RXdate(&RXA1cmd,rxdate);
    ecat_NS_H();

}
