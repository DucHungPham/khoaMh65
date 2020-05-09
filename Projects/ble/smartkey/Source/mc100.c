/******************************************************************************
 * INCLUDES
 */
#include <ioCC2541.h>
#include "mc100.h"


/******************************************************************************
 * DEFINES
 */

//------------------------------
#define	RX_DataLengthMax_Value 	4
//------------------------------
#define RF_ChannelValue 0
//------------------------------
#define	SyncWord_Addr1 	72
#define	SyncWord_Data1 	0xAA

#define	SyncWord_Addr2 	73
#define	SyncWord_Data2 	0xBB

#define	SyncWord_Addr3 	78             	       	
#define	SyncWord_Data3 	0xCC

#define	SyncWord_Addr4 	79             	
#define	SyncWord_Data4 	0xDD
//------------------------------
#define	RF_CLK         	       	P0_1
#define	RF_CSn         	       	P0_0
#define	RF_MOSI_MISO   	       	P0_6

#define	RF_MOSI_MISO_DDR       	P0DIR
//------------------------------
#define	RF_MOSI_Input  	       	1
#define	RF_MOSI_Output 	0



static uint8 SPI_CKPHA_Flag =0;
   
/******************************************************************************
 * FUNCTION PROTOTYPES
 */


/******************************************************************************
 * LOCAL VARIABLES
 */
//static uint8 acc_initialized = FALSE;

static const uint8 RF_InitTab[35]=
{
       	17,0x3A,
       	18,0x0C,       	       	
       	34,0x08,
       	45,0x00,
       	52,0x19,
       	53,0x40,
       	64,0x78,       	       
       	72,0x55,
       	73,0x55,
       	74,0x55,
       	75,0x55,
       	76,0xAA,
       	77,0x3E,
       	78,0x57,
       	79,0xE4,
       	80,0xF8,
       	81,0x47,       	       	
       	0xFF
};
/******************************************************************************
 * FUNCTIONS
 */
void delay_us(uint8 t){
  while(t--)
    for(unsigned short i=0;i<32;i++)asm("NOP"); 
}


uint8 RF_Init(void)
{
  
  uint8 RF_InitIndex,RF_RegAddr,RF_RegData,RF_Init_RegData_Temp;
  // GPIO init
  
  P0DIR |= 0x43;
    
  SPI_CKPHA_Flag=0; 
  SPI_WriteReg(30,0x00); 
  SPI_CKPHA_Flag=1;      	

  SPI_WriteReg(94,0x80);    	
  
  for(RF_InitIndex=0;RF_InitIndex<100;RF_InitIndex++)
  {
          RF_RegAddr=RF_InitTab[RF_InitIndex*2];
          if(0xFF==RF_RegAddr)
          {
                  break; 	       	       	
          }
          RF_RegData=RF_InitTab[RF_InitIndex*2+1];
          RF_Init_RegData_Temp=RF_RegData;
          
          SPI_WriteReg(RF_RegAddr,RF_RegData);
          
          SPI_ReadReg(RF_RegAddr);
          
          if(RF_Init_RegData_Temp!=RF_RegData)
          {
                  return 0;
          }
  }
  SPI_WriteReg(46,0x09);
  return 1;
}
//------------------------------
void RF_TX(void)
{
       	SPI_WriteReg(15,RF_ChannelValue);
       	
       	SPI_WriteReg(14,0x01);
}
//------------------------------
void RF_TX_PKT(void)
{
       uint8 tmp;
  do
       	{
       	       	tmp =SPI_ReadReg(97);  	       	
       	}
       	while(0x40!=(tmp&0x40));
}
//------------------------------
void RF_TX_250K(void)
{
       	SPI_WriteReg(64,0x68);

       	SPI_WriteReg(106,0x68);

       	SPI_WriteReg(107,0xC3);

       	SPI_WriteReg(108,0x16);

       	SPI_WriteReg(109,0x7C);

       	SPI_WriteReg(110,0x6E);

       	SPI_WriteReg(111,0xA1);
}

//------------------------------
void RF_RX(void)
{
       	SPI_WriteReg(14,0x00);
       	
       	SPI_WriteReg(15,0x80+RF_ChannelValue);
}
//------------------------------
void RF_RX_250K(void)
{
       	SPI_WriteReg(64,0x68);

       	SPI_WriteReg(106,0x68);

       	SPI_WriteReg(107,0xC3);

       	SPI_WriteReg(108,0x7A);

       	SPI_WriteReg(109,0x89);

       	SPI_WriteReg(110,0xC1);

       	SPI_WriteReg(111,0x97);
}
//------------------------------
void RF_Idle(void)
{
       	SPI_WriteReg(14,0);
       	
       	SPI_WriteReg(15,0);       	
}
//------------------------------
void RF_Sleep(void)
{
  uint8 tmp;     	
  tmp=SPI_ReadReg(70);
       	
       	tmp=tmp|0x40;
       	
       	SPI_WriteReg(70,tmp);
}
//------------------------------
void RF_Wakeup(void)
{
       	RF_CSn=0;
       	
       	//Delay_100us(40);
       	
       	RF_CSn=1;
}
//------------------------------
void RF_WriteFIFO(uint8 *TxRxFIFO)
{
  uint8 tmp;     	
  RF_CSn=0;
       	SPI_WriteBit(100);
       	
       	for(tmp=0;tmp<=TxRxFIFO[0];tmp++)
       	{
       	       	SPI_WriteBit(TxRxFIFO[tmp]);
       	}
       	RF_CSn=1;
}
//------------------------------
void RF_ResetWriteFIFO(void)
{
       	SPI_WriteReg(104,0x80);   	
}
//------------------------------
uint8 RF_ReadFIFO(uint8 *TxRxFIFO)
{
       	uint8 tmp,Error_Flag=0;
       	
       	tmp = SPI_ReadReg(96);
       	
       	if(0==(tmp&0x80))
       	{
       	       	RF_CSn=0;
       	       	delay_us(4);
       	       	SPI_WriteBit(100|0x80);
       	       	delay_us(4);
       	       	TxRxFIFO[0]=SPI_ReadBit();
       	       	if((TxRxFIFO[0]<=RX_DataLengthMax_Value)&&(TxRxFIFO[0]!=0))
       	       	{
       	       	       	for(tmp=1;tmp<=TxRxFIFO[0];tmp++)
       	       	       	{
       	       	       	       	TxRxFIFO[tmp]=SPI_ReadBit();
       	       	       	}
                        
       	       	}
       	       	else 
       	       	{
       	       	       	Error_Flag=1;
       	       	}
       	       	RF_CSn=1;
       	}
       	else 
       	{
       	       	Error_Flag=1;
       	}
        return Error_Flag;
}
//------------------------------
void RF_ResetReadFIFO(void)
{
       	SPI_WriteReg(105,0x80);
}
//------------------------------
void SPI_WriteReg(uint8 RF_RegAddr_Temp,uint8 RF_RegData_Temp)
{
       	RF_CSn=0;
       	SPI_WriteBit(RF_RegAddr_Temp);
       	delay_us(2);
       	SPI_WriteBit(RF_RegData_Temp);
       	RF_CSn=1;
}
//------------------------------
void SPI_WriteBit(uint8 SPI_BUF_Temp)
{
       	#if RF_MOSI_Output
       	       	RF_MOSI_MISO_DDR &= ~0x40;
       	#else 
       	       	RF_MOSI_MISO_DDR |= 0x40;
       	#endif
       	uint8 tmp;
       	for(tmp=0;tmp<8;tmp++)
       	{
       	       	if(0==SPI_CKPHA_Flag)
       	       	{
       	       	       	RF_CLK=1;
       	       	}
       	       	else
       	       	{
       	       	       	RF_CLK=0;
       	       	}
delay_us(1);
       	       	if(0x80==(SPI_BUF_Temp&0x80))
       	       	{
       	       	       	RF_MOSI_MISO=1;
       	       	}
       	       	else 
       	       	{
       	       	       	RF_MOSI_MISO=0;
       	       	}
       	       	delay_us(2);
       	       	

       	       	if(0==SPI_CKPHA_Flag)
       	       	{
       	       	       	RF_CLK=0;
       	       	}
       	       	else
       	       	{
       	       	       	RF_CLK=1;
       	       	}
       	       	SPI_BUF_Temp=SPI_BUF_Temp<<1;
                delay_us(1);
       	}
       	RF_CLK=0;
}
//------------------------------
uint8 SPI_ReadReg(uint8 RF_RegAddr_Temp)
{
       	uint8 tmp;
  RF_CSn=0;
       	delay_us(2);
       	SPI_WriteBit(0x80|RF_RegAddr_Temp);
       	delay_us(2);
       tmp=SPI_ReadBit();
       	RF_CSn=1;

       	#if RF_MOSI_Output
       	       	RF_MOSI_MISO_DDR &= ~0x40;
       	#else 
       	       	RF_MOSI_MISO_DDR |= 0x40;
       	#endif
                return tmp;
}
//------------------------------
uint8 SPI_ReadBit(void)
{
       	#if RF_MOSI_Output
       	       	RF_MOSI_MISO_DDR &= ~0x40;
       	#else 
       	       	RF_MOSI_MISO_DDR |= 0x40;
       	#endif	
uint8 tmp,dataRead=0;
       	for(tmp=0;tmp<8;tmp++)
       	{
       	       	if(0==SPI_CKPHA_Flag)
       	       	{
       	       	       	RF_CLK=1;
       	       	       	delay_us(1);
       	       	       	RF_CLK=0;
       	       	}
       	       	else
       	       	{
       	       	       	RF_CLK=0;
       	       	       	delay_us(1);
       	       	       	RF_CLK=1;
       	       	}

       	       	delay_us(2);
       	       	dataRead=dataRead<<1;
       	       	if(1==RF_MOSI_MISO)
       	       	{
       	       	       	dataRead++;
       	       	}
       	}
       	RF_CLK=0;
       	return dataRead;
}
//------------------------------

//------------------------------
void RF_SetSyncWord(void)
{
       	SPI_WriteReg(SyncWord_Addr1,SyncWord_Data1);
       	SPI_WriteReg(SyncWord_Addr2,SyncWord_Data2);
       	SPI_WriteReg(SyncWord_Addr3,SyncWord_Data3);
       	SPI_WriteReg(SyncWord_Addr4,SyncWord_Data4);
}