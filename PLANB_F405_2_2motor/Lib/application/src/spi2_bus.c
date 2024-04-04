#include "spi2_bus.h"

A1PackageSpiTx motor_tx[4][3];
A1PackageSpiRx motor_rx[4][3];

A1msgTxTransform tx_transform;
A1msgRxTransform rx_transform;

void GetMotorMsg(uint8_t *input)
{
    memcpy(&rx_transform.u8[0],input,21);
    memcpy(&motor_rx[rx_transform.a1msg.leg_id][rx_transform.a1msg.motor_id],&rx_transform.a1msg.start[0],21); 
}

void PreparMotorMsg(A1PackageSpiTx motor_tx_pack,uint8_t *motor_original_tx)
{
    uint8_t motor_tx_msg[21];
    memcpy(motor_tx_msg,&motor_tx_pack,21);

    for (int i = 0; i < 20; i++)
    {
        *(motor_original_tx + i) = motor_tx_msg[i + 1];
    }

    for (int i = 0; i < 21; i++)
    {
        *(motor_original_tx + i + 20) = motor_tx_msg[i];
    }
}