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

void PreparMotorMsg(uint8_t *motormsg_set,uint8_t *motormsg_tx)
{
    for (int i = 0; i < 20; i++)
    {
        *(motormsg_tx + i) = *(motormsg_set + i + 1);
    }

    for (int i = 0; i < 21; i++)
    {
        *(motormsg_tx + i + 20) = *(motormsg_set + i);
    }
}