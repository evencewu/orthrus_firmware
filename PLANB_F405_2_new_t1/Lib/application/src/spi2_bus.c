#include "spi2_bus.h"
#include <string.h>

A1PackageSpiTx motor_tx[4][3];
A1PackageSpiRx motor_rx[4][3];

uint8_t motor_original_rx[41];
uint8_t motor_original_tx[41];
uint8_t motormsg_get[21];

uint8_t spi2_flag;
uint8_t call_flag;

A1msgTxTransform tx_transform;
A1msgRxTransform rx_transform;

void GetMotorMsg(uint8_t *input)
{
    memcpy(&rx_transform.u8[0], input, 21);
    memcpy(&motor_rx[rx_transform.a1msg.leg_id][rx_transform.a1msg.motor_id], &rx_transform.a1msg.start[0], 21);
}

void PreparMotorMsg(A1PackageSpiTx motor_tx_pack, uint8_t *motor_original_tx)
{
    uint8_t motor_tx_msg[21];
    memcpy(motor_tx_msg, &motor_tx_pack, 21);

    for (int i = 0; i < 20; i++)
    {
        *(motor_original_tx + i) = motor_tx_msg[i + 1];
    }

    for (int i = 0; i < 21; i++)
    {
        *(motor_original_tx + i + 20) = motor_tx_msg[i];
    }
}

void ecat_spi_motor(int leg_id, int motor_id)
{
    PreparMotorMsg(motor_tx[leg_id][motor_id], motor_original_tx);

    spi2_flag = 0;

    if (call_flag == 0)
    {
        spi2_w_cmd(0xD2);
        call_flag = 1;
    }

    spi2_flag = spi2_r_cmd();

    // spi
    if (spi2_flag == 0xD2)
    {
        for (int i = 0; i < 41; i++)
        {
            motor_original_rx[i] = spi2_wr_cmd(motor_original_tx[i]);
        }

        for (int i = 0; i < 21; i++)
        {
            if (motor_original_rx[i] == 0xD2 && motor_original_rx[i + 1] == 0xFE)
            {
                for (int j = 0; j < 21; j++)
                {
                    motormsg_get[j] = motor_original_rx[i + j];
                }
                GetMotorMsg(&motormsg_get[0]);
            }
        }

        call_flag = 0;
    }
}

void MotorTxDataInit()
{
    for (int i = 0; i < 12; i++)
    {
        motor_tx[i / 3][i % 3].leg_id = i / 3;
        motor_tx[i / 3][i % 3].motor_id = i % 3;
    }
}