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