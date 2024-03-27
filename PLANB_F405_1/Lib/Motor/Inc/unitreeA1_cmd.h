#ifndef __UNITREEA1_CMD__
#define __UNITREEA1_CMD__

#include "unitreeA1_msg.h"
#include "usart.h"

extern motor_send_t cmd_leg[4];
extern motor_recv_t data_motor[4][3];
extern motor_recv_t data_leg[4];

void DataSet(int leg_id);
void unitreeA1_tx(int leg_id);

extern uint32_t crc32_core(uint32_t *ptr, uint32_t len);

#endif
