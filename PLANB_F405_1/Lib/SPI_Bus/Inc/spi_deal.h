#ifndef __SPI_DEAL__
#define __SPI_DEAL__

#include "data_def.h"

extern uint8_t spi_sbus_buf[21];

extern A1msgTxTransform a1_msg_tx_transform;
extern A1msgRxTransform a1_msg_rx_transform;

extern A1PackageSpiTx spi_tx_data[4][3];
extern A1PackageSpiRx spi_rx_data[4][3];

// void EcatChat_Init(void);
// static void sbus_to_rc_spi(volatile const uint8_t *sbus_buf, A1PackageSpiRx *modf_buf);
void DateCheck_DateModfy(A1PackageSpiRx *modf_buf);
void SPI_RXdate(A1PackageSpiRx *modf_buf, uint8_t *spi_sbus_buf);
void ecat_NS_L (void);
void ecat_NS_H (void);
void sendToEcat(int leg_id);


void MASTER_Synchro(void);
void SPI_TRANSMIT(int leg_id,int motor_id);

void MotorTxDeal(uint8_t *array1, uint8_t *array3);
void SpiMotorRxArchive(uint8_t *rx_recv_data);

#endif
