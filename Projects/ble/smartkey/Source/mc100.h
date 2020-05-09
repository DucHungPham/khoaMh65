#ifndef MC100_H
#define MC100_H


/******************************************************************************
 * INCLUDES
 */
#include "hal_types.h"

uint8 RF_Init(void);
void RF_TX(void);
void RF_TX_PKT(void);
void RF_TX_250K(void);
void RF_RX(void);
void RF_RX_250K(void);
void RF_Idle(void);
void RF_Sleep(void);
void RF_Wakeup(void);
void RF_WriteFIFO(uint8 *TxRxFIFO);
void RF_ResetWriteFIFO(void);
uint8 RF_ReadFIFO(uint8 *TxRxFIFO);
void RF_ResetReadFIFO(void);
void SPI_WriteReg(uint8 RF_RegAddr_Temp,uint8 RF_RegData_Temp);
void SPI_WriteBit(uint8 SPI_BUF);
uint8 SPI_ReadReg(uint8 RF_RegAddr_Temp);
uint8 SPI_ReadBit(void);

   
#endif
