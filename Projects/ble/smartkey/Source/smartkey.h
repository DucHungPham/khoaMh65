#ifndef smartkey_h
#define smartkey_h

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */


// Key Fob Task Events
#define SKP_START_DEVICE_EVT                              0x0001
#define SKP_RF2G4_READ_EVT                                0x0002
#define SKP_TOGGLE_BUZZER_EVT                             0x0004
#define SKP_ADV_IN_CONNECTION_EVT                         0x0008
#define SKP_POWERON_TIMEOUT_EVT                           0x0010
#define SKP_TimeTick_TIMEOUT_EVT                          0x0020
#define SKP_TogOut_TIMEOUT_EVT                          0x0040

typedef union {
  unsigned char byte;
  struct{
    unsigned char b0 : 1;
    unsigned char b1 : 1;
    unsigned char b2 : 1;
    unsigned char b3 : 1;
    unsigned char b4 : 1;
    unsigned char b5 : 1;
    unsigned char b6 : 1;
    unsigned char b7 : 1;
  } bits;
}regType;
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void KeyFobApp_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 KeyFobApp_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* KEYFOBDEMO_H */
