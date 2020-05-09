/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_drivers.h"
#include "hal_adc.h"
#include "hal_key.h"
#include "osal.h"

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

#define HAL_KEY_DEBOUNCE_VALUE  10

/* CPU port interrupt */

/* swStand is at P0.7 */
#define swStand_bit    BV(7)
#define swStand P0_7
/* swStart is at P0.3 */
#define swStart_bit    BV(3)
#define swStart P0_3

/* 358out is at P1.5 */
#define sig358_bit     BV(5)
#define sig358 P1_5

/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/


/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/
static uint8 halKeySavedKeys;     /* used to store previous key state in polling mode */
static halKeyCBack_t pHalKeyProcessFunction;
static uint8 HalKeyConfigured;
bool Hal_KeyIntEnable;            /* interrupt enable/disable flag */

bool mode_chek = md_ckIDE;
unsigned char cntCyc = 0, cntOff = 0; 
/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/
void halProcessKeyInterrupt(void);

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/


/**************************************************************************************************
 * @fn      HalKeyInit
 *
 * @brief   Initilize Key Service
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalKeyInit( void )
{

  halKeySavedKeys = 0;  // Initialize previous key to 0.
  /* Initialize GPIO */

  /* Initialize callback function */
  pHalKeyProcessFunction  = NULL;

  /* Start with key is not configured */
  HalKeyConfigured = FALSE;

  /* Rising/Falling edge configuratinn */
  // Rising Default
  // PICTL &= 0xfa; 
  
  /* enable CPU interrupt */
  //IEN1 |= 0x20;//P0
  IEN2 |= 0x10;//P1
}


/**************************************************************************************************
 * @fn      HalKeyConfig
 *
 * @brief   Configure the Key serivce
 *
 * @param   interruptEnable - TRUE/FALSE, enable/disable interrupt
 *          cback - pointer to the CallBack function
 *
 * @return  None
 **************************************************************************************************/
void HalKeyConfig (halKeyCBack_t cback)
{
  /* Register the callback fucntion */
  pHalKeyProcessFunction = cback;

  /* enable interrupt generation at port */  
  //P0IEN |=0x88;//pin7 pin3
  P1IEN |=0x20;//pin5    

  /* Do this only after the hal_key is configured - to work with sleep stuff */
  if (HalKeyConfigured == TRUE)
  {
    //osal_stop_timerEx(Hal_TaskID, HAL_KEY_EVENT);  /* Cancel polling if active */
  }
  
  /* Key now is configured */
  HalKeyConfigured = TRUE;
}


/**************************************************************************************************
 * @fn      HalKeyRead
 *
 * @brief   Read the current value of a key
 *
 * @param   None
 *
 * @return  keys - current keys status
 **************************************************************************************************/
uint8 HalKeyRead ( void )
{
  uint8 keys = 0;


  return keys;
}


/**************************************************************************************************
 * @fn      HalKeyPoll
 *
 * @brief   Called by hal_driver to poll the keys
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalKeyPoll (void)
{
  uint8 keys = 0;
 if(swStart)
  keys |= swStart_bit;
 if(swStand)
  keys |= swStand_bit;
 if(sig358)
   keys |= sig358;
  /* Store the current keys for comparation next time */
  halKeySavedKeys = keys;

  /* Invoke Callback if new keys were depressed */
  if (pHalKeyProcessFunction)
  {
    (pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
  }
}

void kfDetect(uint8 kf){
  //uint8 keys = 0xbb;
  if (pHalKeyProcessFunction)
  {
    
    (pHalKeyProcessFunction) (kf, HAL_KEY_STATE_NORMAL);
  }
}
/**************************************************************************************************
 * @fn      halProcessKeyInterrupt
 *
 * @brief   Checks to see if it's a valid key interrupt, saves interrupt driven key states for
 *          processing by HalKeyRead(), and debounces keys by scheduling HalKeyRead() 25ms later.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halProcessKeyInterrupt (void)
{
  bool valid=FALSE;

  //if(P0_7) P0_6 =~P0_6;
  //if(P0_3) P0_5 =~P0_5;
  
  if (valid)
  {
    //osal_start_timerEx (Hal_TaskID, HAL_KEY_EVENT, HAL_KEY_DEBOUNCE_VALUE);
  }
}

/**************************************************************************************************
 * @fn      HalKeyEnterSleep
 *
 * @brief  - Get called to enter sleep mode
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void HalKeyEnterSleep ( void )
{
}

/**************************************************************************************************
 * @fn      HalKeyExitSleep
 *
 * @brief   - Get called when sleep is over
 *
 * @param
 *
 * @return  - return saved keys
 **************************************************************************************************/
uint8 HalKeyExitSleep ( void )
{
  /* Wake up and read keys */
  return ( HalKeyRead () );
}

/***************************************************************************************************
 *                                    INTERRUPT SERVICE ROUTINE
 ***************************************************************************************************/

/**************************************************************************************************
 * @fn      halKeyPort0Isr
 *
 * @brief   Port0 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halKeyPort0Isr, P0INT_VECTOR )
{
  HAL_ENTER_ISR();

  if (P0IFG & 0x88) //bit 7,3
  {
    //P0_6 =~P0_6;
    //osal_start_timerEx (Hal_TaskID, HAL_KEY_EVENT, HAL_KEY_DEBOUNCE_VALUE);
  }
  /*
    Clear the CPU interrupt flag for Port_0
    PxIFG has to be cleared before PxIF
  */
  P0IFG = 0; 
  P0IF = 0; 

  CLEAR_SLEEP_MODE();

  HAL_EXIT_ISR();

  return;
}

/**************************************************************************************************
 * @fn      halKeyPort1Isr
 *
 * @brief   Port1 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halKeyPort1Isr, P1INT_VECTOR )
{
  HAL_ENTER_ISR();

  if (P1IFG & 0x20) //bit 5
  {
    //P0_6 =~P0_6;
    if (mode_chek == md_ckIDE) {
      cntCyc = 0;
      cntOff = 0;
      //P0_6 =1;
      mode_chek = md_ckStart;
      // đảo sườn
      PICTL |=0x04;//Falling edge on input gives interrupt
      osal_start_timerEx (Hal_TaskID, HAL_KEYFOB_EVENT, 10);
    }else  if (mode_chek == md_ckStart){
      cntOff = 0;
      //P0_6 =0;
    }
    
  }
  
  P1IFG = 0; 
  P1IF = 0; 

  CLEAR_SLEEP_MODE();

  HAL_EXIT_ISR();

  return;
}

/**************************************************************************************************
**************************************************************************************************/
