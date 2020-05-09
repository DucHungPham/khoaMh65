/******************************************************************************

 @file  keyfobdemo.c

 @brief Key Fob Demo Application.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
mapping
____CC2541___
9 - 358vcc - P11
5 - 358sig - P15
11 - buz - P10
12 - in1 - P07
13 - rl1 - P06
14 - rl2 - P05
15 - out2 - P04
16 - in2 - P03

___PortOut___
1 - DVDD
2 - GND
3 - RST
4 - P17
5678 - P20..23

 ******************************************************************************

 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_key.h"
#include "buzzer.h"
#include "mc100.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"
#include "rfservice.h"
#include "osal_snv.h"
#include "smartkey.h"


/*********************************************************************
 * MACROS
 */

#define beepOn()                                                        \
{                                                                       \
 osal_start_timerEx( keyfobapp_TaskID, SKP_TOGGLE_BUZZER_EVT, 200);     \
}                                                                                                                                          

/*********************************************************************
 * CONSTANTS
 */

// Delay between power-up and starting advertising (in ms)
#define STARTDELAY                            500

//Time tick time out (in ms)
#define sysTick        10

// Number of beeps before buzzer stops by itself
#define BUZZER_MAX_BEEPS                      10

// Buzzer beep tone frequency for "High Alert" (in Hz)
#define BUZZER_ALERT_HIGH_FREQ                2000//4096

// Buzzer beep tone frequency for "Low Alert" (in Hz)
#define BUZZER_ALERT_LOW_FREQ                 250


// How often (in ms) to read the accelerometer
#define RF2G4_READ_PERIOD                     100//50


//GAP Peripheral Role desired connection parameters

// Use limited discoverable mode to advertise for 30.72s, and then stop, or
// use general discoverable mode to advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
//#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         5


// buzzer_state values
#define BUZZER_OFF                            0
#define BUZZER_ON                             1
#define BUZZER_ALERT                          2  
#define BUZZER_OFF_DELAY                      200
#define BUZZER_TOG_DELAY                      500
  
// keyfobAlertState values
#define ALERT_STATE_OFF                       0
#define ALERT_STATE_LOW                       1
#define ALERT_STATE_HIGH                      2

// keyRF
// xem bcomdef.h(BLE_NVID_CUST_START) & osal_snv.h
#define BUF_LEN 5
#define SNV_ID_APP 0x80

// Status
#define _Open	1
#define tOut_Open	3500 //1->33=>35

#define _Ide		0
#define tOut_Ide	0

#define _rCheck	2
#define tOut_rCheck 300 //150

#define _Alert		3
#define tOut_Alert  1800
#define add_Alert 0x08

#define _rAlert		4
#define tOut_rAlert	6000

//mapping
#define OutSign                          P0_6
#define OutMain                          P0_5
#define OutBtn                           P0_4//
#define InBtn                            P0_3//
#define InStn                            P0_7
#define Vcc358                           P1_1
#define InSKey                           P1_5

#define _adCt 0x3F
#define _adMd 0x40

#define _adsSk 0x41
#define _adsX 0x10
#define _adsY 0x11

#define Status			RegStatus.byte
#define reAlertOn 		RegStatus.bits.b7
#define accEna 		RegStatus.bits.b6
#define vibrateOn 	RegStatus.bits.b5
#define antenSkip 	RegStatus.bits.b4
#define mode_chek 	RegStatus.bits.b3
#define keyfob 			RegStatus.bits.b2
#define keyUpd 		RegStatus.bits.b1
#define bitPwOn		RegStatus.bits.b0

#define voice1	20
#define voice2	25

#define INVALID_CONNHANDLE                    0xFFFF

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static unsigned int timeOut = 0, timeTick = 0,tmp16=0;
static unsigned char  lostDetect,mtState = _Ide, mtOldState = _Ide,isSw=0; // che do trong qua trinh hoat dong cua xe
static uint16 buzDelayOn =500,buzFeq = BUZZER_ALERT_LOW_FREQ;
volatile regType RegStatus;

static uint8 keyfobapp_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

static uint8 keyfobAlertState;

//key
static uint8 buf[BUF_LEN] ={'O','K','i','e','f'};

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 deviceName[] =
{
  // complete name
  0x0b,   // length of first data structure (11 bytes excluding length byte)
  0x09,   // AD Type = Complete local name
  'S',   // 'K'
  'm',   // 'e'
  'a',   // 'y'
  'r',   // 'f'
  't',   // 'o'
  ' ',   // 'b'
  'R',   // 'd'
  'F',   // 'e'
  'v',   // 'm'
  '1',   // 'o'
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  0x02,   // length of first data structure (2 bytes excluding length byte)
  GAP_ADTYPE_FLAGS,   // AD Type = Flags
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x07,   // length of second data structure (7 bytes excluding length byte)
  GAP_ADTYPE_16BIT_MORE,   // list of 16-bit UUID's available, but not complete list
  LO_UINT16( LINK_LOSS_SERV_UUID ),        // Link Loss Service (Proximity Profile)
  HI_UINT16( LINK_LOSS_SERV_UUID ),
  LO_UINT16( IMMEDIATE_ALERT_SERV_UUID ),  // Immediate Alert Service (Proximity / Find Me Profile)
  HI_UINT16( IMMEDIATE_ALERT_SERV_UUID ),
  LO_UINT16( TX_PWR_LEVEL_SERV_UUID ),     // Tx Power Level Service (Proximity Profile)
  HI_UINT16( TX_PWR_LEVEL_SERV_UUID )
};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "SmartRF Smartkey";//"TI BLE Keyfob"
  
// Buzzer state
static uint8 buzzer_state = BUZZER_OFF;
static uint8 buzzer_beep_count = 0;

// Accelerometer Profile Parameters
static uint8 rfEnabler = FALSE;
//static uint8 rfCnt = 0;

#define outStop 0
#define outStart 1
#define outAccept 2
#define outToggle 3

static uint8 key[9] = {0,5,7,5,4,9,2,7,4};
static uint8 keyCnt=0,mdOut=outStop;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void keyfobapp_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void keyfobapp_StopAlert( void );
static void keyfobapp_HandleKeys( uint8 shift, uint8 keys );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void rfEnablerChangeCB( void );


/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t keyFob_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                // When a valid RSSI is read from controller
};


// GAP Bond Manager Callbacks
static gapBondCBs_t keyFob_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Accelerometer Profile Callbacks
static rfCBs_t keyFob_RFCBs =
{
  rfEnablerChangeCB,          // Called when Enabler attribute changes
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
void setState(unsigned char stt, unsigned int _tOut);
void beep(uint8 delay, unsigned char rep);
void delay_x10ms(unsigned int t);
void beepOff(void);

/*********************************************************************
 * @fn      KeyFobApp_Init
 *
 * @brief   Initialization function for the Key Fob App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void KeyFobApp_Init( uint8 task_id )
{
  keyfobapp_TaskID = task_id;
    
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
    uint8 initial_advertising_enable = FALSE;
    
    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;
  
    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
   
    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
  
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( deviceName ), deviceName );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
  
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }
  
  // Set the GAP Attributes
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Setup the GAP Bond Manager
  {
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
  RF_AddService( GATT_ALL_SERVICES );      // Accelerometer Profile


  keyfobAlertState = ALERT_STATE_OFF;

  // make sure buzzer is off
  buzzerStop();

  // makes sure outpyt are off
  

  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.

  /*
9 - 358vcc - P11
5 - 358sig - P15
11 - buz - P10
12 - in1 - P07
13 - rl1 - P06
14 - rl2 - P05
15 - out2 - P04
16 - in2 - P03

___PortOut___
1 - DVDD
2 - GND
3 - RST
4 - P17
5678 - P20..23
*/
  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0x01; // Configure Port 1 as GPIO, except P1.0 for peripheral function for buzzer
  P2SEL = 0; // Configure Port 2 as GPIO

  //P1INP = 0x20;// kiểm tra lại
  //P2INP = 0x40;
  
  P0DIR = 0x70;
  P1DIR = 0x03; // 
  P2DIR = 0x00; // 

  P0 = 0; // All pins on port 0 to low 
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( keyfobapp_TaskID );//đăng ký task nhận Mes khi phím đc nhấn
  
  // Setup a delayed profile startup
  osal_start_timerEx( keyfobapp_TaskID, SKP_START_DEVICE_EVT, STARTDELAY );
}

/*********************************************************************
 * @fn      KeyFobApp_ProcessEvent
 *
 * @brief   Key Fob Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 KeyFobApp_ProcessEvent( uint8 task_id, uint16 events )
{
  // Detected Smartkey - evenMsg
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( keyfobapp_TaskID )) != NULL )
    {
      keyfobapp_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  //-----------------------------
  
  // Start device
  if ( events & SKP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &keyFob_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &keyFob_BondMgrCBs );

    // Start the Accelerometer Profile
    VOID RF_RegisterAppCBs( &keyFob_RFCBs );
         
    osal_pwrmgr_device( PWRMGR_ALWAYS_ON ); // To keep the LED on continuously.
    osal_start_timerEx( keyfobapp_TaskID, SKP_POWERON_TIMEOUT_EVT, 1000 );
    return ( events ^ SKP_START_DEVICE_EVT );
  }
  //-----------------------------
  
  // Power On
  if ( events & SKP_POWERON_TIMEOUT_EVT )
  {
    //osal_pwrmgr_device( PWRMGR_BATTERY ); // Revert to battery mode after LED off
    if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }

      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
    }
        
    osal_start_timerEx( keyfobapp_TaskID, SKP_TimeTick_TIMEOUT_EVT,sysTick);
    
    beep(10,2);
    
    // start App
    //setState(_Ide, 0);
    
    setState(_rCheck, 400); // timout 4s cho thoi gian mo may, check tin hieu chia
    mtOldState = _Ide;
                
    return ( events ^ SKP_POWERON_TIMEOUT_EVT );
  }
  //-----------------------------
  
  // App sysTick
  if ( events & SKP_TimeTick_TIMEOUT_EVT){
    
    // test buf
    /*
   bStatus_t status = osal_snv_read(SNV_ID_APP, BUF_LEN, (uint8 *)buf);
  if(status != SUCCESS)
  {
    //Write first time to initialize SNV ID
    osal_snv_write(SNV_ID_APP, BUF_LEN, (uint8 *)buf);
  }
    */
    if ((timeOut != 0) && ( timeTick > timeOut)) {

      switch (mtState) {
      case _Open:
      //timeTick = 0; //open->open
      //tmp16 = 40;// thoi gian lay mau gia tri goc
        break;
      case _Alert:
        beepOff();
        bitPwOn = 0;
        setState(_rCheck, tOut_rCheck);
        break;
      case _rAlert:
        beepOff();
        reAlertOn = 0;
        beep(30, 1);
        vibrateOn = 1;
        setState(_Ide, tOut_Ide);
        //enaDetect =0;
        tmp16 = timeTick + 40;
        break;
      case _rCheck:
        // xoa canh bao
        //>>if (READ_EEPROM(add_Alert) == 0xcc) WRITE_EEPROM(add_Alert, 0);
        if (reAlertOn) {	///???reAlertOn
            setState(_rAlert, tOut_rAlert);

            buzzer_state = BUZZER_ALERT;
            buzFeq = BUZZER_ALERT_LOW_FREQ;
            buzzerStart(buzFeq);
            osal_start_timerEx( keyfobapp_TaskID, SKP_TOGGLE_BUZZER_EVT,BUZZER_TOG_DELAY);

        } else if (mtOldState == _Ide) {
            // bao anten hong
            beep(10, 5);
            bitPwOn = 1;
            setState(_Open, tOut_Open);
            tmp16 = timeTick + 40;

        } else {
            reAlertOn = 0;
            beep(30, 1);
            setState(_Ide, tOut_Ide);
            //enaDetect =0;
            vibrateOn = 1; // kiem tra
            tmp16 = timeTick + 40;
        }
        break;
      }
    }
    //----------------------
    // kiểm tra chân chống
    if (timeTick > tmp16){
      tmp16 = timeTick + 40;
      if((mtState == _Open) ){//||(mtState == _rCheck)
        // kiem tra da chong
        if (InStn) {
          //beep(10,1);
          isSw++;
          if (isSw > 44) {
            isSw = 0;
            vibrateOn = 1; // bat che do chong rung
            // lay vi tri chinh xac tai thoi diem da chong, truoc khi chuyen sang che do chong rung
            //acYOld = (signed char)buf[3]; acXOld = (signed char)buf[1]; // co the bo vi o che do rCheck van lay gia tri
            bitPwOn = 0;
            setState(_rCheck, tOut_rCheck);
            //tmp16 = timeTick + 40;///----
          }
          else if (isSw > 25) { //0.4*20=8s
            if (isSw % 2 == 0) beep(10,1);       
          }
        }    
      }
    }
    //----------------------
    // Thuc thi lenh dieu khien
    if (bitPwOn) {OutMain = 1;}
    else {OutMain = 0;}
    //----------------------
    timeTick++;
    osal_start_timerEx( keyfobapp_TaskID, SKP_TimeTick_TIMEOUT_EVT,sysTick);
    return ( events ^ SKP_TimeTick_TIMEOUT_EVT);
  }
  //-----------------------------
  
  // Check RF recive
  if ( events & SKP_RF2G4_READ_EVT )
  {
    bStatus_t status = RF_GetParameter( RF_ENABLER, &rfEnabler );

    if (status == SUCCESS)
    {
      if ( rfEnabler )
      {
        uint8 tmp;
        tmp = SPI_ReadReg(97);
        if(0x40==(tmp&0x40))
        {
           tmp = RF_ReadFIFO(buf);
           RF_ResetReadFIFO();
           RF_RX();
           if((0==tmp)){
             //HalLedBlink (OutSign, 2, 50, 100);
             RF_SetParameter(RF_REC, 5, buf);
              //if(TX_RX_DataLength[4]) L_Rx1 =0;
              //else L_Rx1 =1;
             //P1_1 =~P1_1;
           };
        }   
        osal_start_timerEx( keyfobapp_TaskID, SKP_RF2G4_READ_EVT, RF2G4_READ_PERIOD );
        // Read accelerometer data
      }
      else
      {
        // Stop the acceleromter
        osal_stop_timerEx( keyfobapp_TaskID, SKP_RF2G4_READ_EVT);
      }
    }
    else
    {
        //??
    }
    return (events ^ SKP_RF2G4_READ_EVT);
  }
  //-----------------------------
  // toggle out
  if ( events & SKP_TogOut_TIMEOUT_EVT )
  {     
    switch (mdOut){
      case outStart:
        OutBtn = 0;
        osal_start_timerEx( keyfobapp_TaskID, SKP_TogOut_TIMEOUT_EVT,500);
        mdOut = outAccept;
        break;
      case outAccept:
        OutBtn = 1;
        osal_start_timerEx( keyfobapp_TaskID, SKP_TogOut_TIMEOUT_EVT,200);
        mdOut = outToggle;
        buzzer_beep_count = key[0]*2;
        keyCnt =1;
        break;
      case outToggle:
        if ( buzzer_beep_count == 0 )
        {   
          OutBtn =0;
          if(keyCnt<9){
             buzzer_beep_count = key[keyCnt]*2;
             osal_start_timerEx( keyfobapp_TaskID, SKP_TogOut_TIMEOUT_EVT,5200);
             keyCnt++;
          }else{
            OutBtn = 0;
            mdOut = outStop;
            osal_stop_timerEx(keyfobapp_TaskID, SKP_TogOut_TIMEOUT_EVT);
          }
          
        }else {
          OutBtn = ~OutBtn;  
          osal_start_timerEx( keyfobapp_TaskID, SKP_TogOut_TIMEOUT_EVT,120);
          buzzer_beep_count--;
        }
        break;
      case outStop:
        OutBtn = 1;
        osal_start_timerEx( keyfobapp_TaskID, SKP_TogOut_TIMEOUT_EVT,5500);
        mdOut = outStart;
        break;
    }
    
    return (events ^ SKP_TogOut_TIMEOUT_EVT);
  }
  //-----------------------------
  
  
  // toggle buzzer
  if ( events & SKP_TOGGLE_BUZZER_EVT )
  {
    // if this event was triggered while buzzer is on, turn it off, increment beep_count,
    // check whether max has been reached, and if not set the OSAL timer for next event to
    // turn buzzer back on.
    
    if ( buzzer_state == BUZZER_ON )
    {
      buzzerStop();
      buzzer_state = BUZZER_OFF;
      
      if(buzzer_beep_count ==0){       
        osal_stop_timerEx(keyfobapp_TaskID, SKP_TOGGLE_BUZZER_EVT);//có cần k?
      } else{
        buzzer_beep_count--;
        osal_start_timerEx( keyfobapp_TaskID, SKP_TOGGLE_BUZZER_EVT, BUZZER_OFF_DELAY);
      }
      
      #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_BATTERY );
      #endif

    }else if(buzzer_state == BUZZER_OFF){
      //osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
      buzzer_state = BUZZER_ON;
      buzzerStart( BUZZER_ALERT_LOW_FREQ );
      osal_start_timerEx( keyfobapp_TaskID, SKP_TOGGLE_BUZZER_EVT, buzDelayOn);
    }else if(buzzer_state == BUZZER_ALERT){
      if(buzFeq == BUZZER_ALERT_LOW_FREQ )
        buzFeq = BUZZER_ALERT_HIGH_FREQ;
      else
        buzFeq = BUZZER_ALERT_LOW_FREQ;
      buzzerStart(buzFeq);
      osal_start_timerEx( keyfobapp_TaskID, SKP_TOGGLE_BUZZER_EVT,BUZZER_TOG_DELAY);
    }
    
    
    /*
    else if ( keyfobAlertState != ALERT_STATE_OFF )
    {
      // if this event was triggered while the buzzer is off then turn it on if appropriate
      keyfobapp_PerformAlert();
    }
*/
    //osal_start_timerEx( keyfobapp_TaskID, SKP_TOGGLE_BUZZER_EVT, 1000 );
    return (events ^ SKP_TOGGLE_BUZZER_EVT);
  }
  //-----------------------------

#if defined ( PLUS_BROADCASTER )
  if ( events & SKP_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );
  }
#endif

  // Discard unknown events
  return 0;
}

//-------------------------------
void beepOff(void)                                                       
{                                                                       
  buzzerStop();                                                         
  buzzer_state = BUZZER_OFF;                                                   
  osal_stop_timerEx( keyfobapp_TaskID, SKP_TOGGLE_BUZZER_EVT);          
}   
void setState(unsigned char stt, unsigned int _tOut) {
  mtOldState = mtState;
  mtState = stt;
  timeTick = 0;
  //tmp16 = 40;
  timeOut = _tOut;
}

void beep(uint8 delay, unsigned char rep) {
  if(rep==0) return;
  buzDelayOn = delay*10;
  buzzer_beep_count = rep -1;
  osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
  buzzerStart( BUZZER_ALERT_LOW_FREQ );
  buzzer_state = BUZZER_ON;
  osal_start_timerEx( keyfobapp_TaskID, SKP_TOGGLE_BUZZER_EVT, buzDelayOn );
}

void setOut(uint8 rep){
  buzzer_beep_count = rep*2;
  osal_start_timerEx( keyfobapp_TaskID, SKP_TogOut_TIMEOUT_EVT, 200 );
}
//-------------------------------

/*********************************************************************
 * @fn      keyfobapp_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void keyfobapp_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  uint8 kf=0;
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      //keyfobapp_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      //keyfob update

        //disable intr 358

      // test time out  chan detect 125khz?
      // co chia -> tao TimeOut
      kf = ((keyChange_t *)pMsg)->keys;
      //beep(10,1);
      if (kf == 0xaa) {
        //beep(10,1);
        
        switch (mtState) {
        case _Open:
          lostDetect = 0;
          timeTick = 0; //open->open
          tmp16 = 40;// thoi gian lay mau gia tri goc
          break;

        case _rCheck:
          if (timeTick > 30) {
            timeTick = 0;
            //Xoa trang thai canh bao khi ve reCheck
            //>>if (READ_EEPROM(add_Alert) == 0xcc) WRITE_EEPROM(add_Alert, 0);

            if ((mtOldState == _Alert)) {
              //beepOn(); 
              beep(10,1);
              if (reAlertOn)reAlertOn = 0;
            }
            else if ((mtOldState == _Ide) || (mtOldState == _rAlert)) {
              bitPwOn = 1;
              setState(_Open, tOut_Open);
              //>>if (READ_EEPROM(_adMd))WRITE_EEPROM(_adMd, 0);
              tmp16 = timeTick + 40;
              beep(10, 1);
            }
            else if ((mtOldState == _Open)) {
              //beepOn();TMR2ON = 0; beep(10,1);// nhac quen chia
              if (reAlertOn)reAlertOn = 0;
            }
          }
          break;

        case _Alert:
          if (timeTick > 200) {
            // xoa canh bao khi dang bao
            beepOff();
            //>>if (READ_EEPROM(add_Alert) == 0xcc) WRITE_EEPROM(add_Alert, 0);
            bitPwOn = 1;
            setState(_Open, tOut_Open);

            tmp16 = timeTick + 40;
          }
          //timeTick =0;
          break;

        case _rAlert:
          beepOff();
          setState(_rCheck, tOut_rCheck);
          break;
        case _Ide:
          // Bo mo may nhanh
          //setState(_rCheck,tOut_rCheck);
          break;
        }
        
        
      }
      // mat chia
      else if(kf ==0xbb){
        //beep(10,2);
        
        switch (mtState) {

        case _Open:
          /// Da chong canh, ? mat chia => uu tien ?
          // vision moi lan nhan nut de, -> check chia
          if ((lostDetect == 1) && (timeTick < 1500) && (timeTick > 700)) // check lan 2 <=> = 20 -> alert 20s -> den smart key sang!  =>      alert =18s
          {
            //>> if (READ_EEPROM(add_Alert) == 0x00) WRITE_EEPROM(add_Alert, 0xcc);           
            setState(_Alert, tOut_Alert);
            buzzer_state = BUZZER_ALERT;
            buzFeq = BUZZER_ALERT_LOW_FREQ;
            buzzerStart(buzFeq);
            osal_start_timerEx( keyfobapp_TaskID, SKP_TOGGLE_BUZZER_EVT,BUZZER_TOG_DELAY);
            lostDetect = 0;
          }
          else {
            lostDetect = 1;//lostDetect++;
            //beep(10,2);
          }
          timeTick = 0;
          tmp16 = 40;
          break;
        case _rCheck:
          //if(timeTick >30){
          timeTick = 0;
          beep(10, 2);
          if (mtOldState == _Alert) {reAlertOn = 1;}
          break;
        }
        
    }
      //enable intr 358
     break;
  }
}

/*********************************************************************
 * @fn      keyfobapp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void keyfobapp_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;

  (void)shift;  // Intentionally unreferenced parameter

  if ( keys & _SwStart )
  {
   P0_6 =~P0_6;
  }

  if ( keys & _SwStand )
  {
    P0_6 =~P0_6;
  }

  if ( keys & _Sig358 )
  {
    P0_6 =~P0_6;
  }
}

/*********************************************************************
 * @fn      keyfobapp_StopAlert
 *
 * @brief   Stops an alert
 *
 * @param   none
 *
 * @return  none
 */
void keyfobapp_StopAlert( void )
{

  keyfobAlertState = ALERT_STATE_OFF;

  buzzerStop();
  buzzer_state = BUZZER_OFF;
  //HalLedSet( (OutMain | OutSign), HAL_LED_MODE_OFF );


  #if defined ( POWER_SAVING )
    osal_pwrmgr_device( PWRMGR_BATTERY );
  #endif
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  uint16 connHandle = INVALID_CONNHANDLE;
  uint8 valFalse = FALSE;

  if ( gapProfileState != newState )
  {
    switch( newState )
    {
    case GAPROLE_STARTED:
      {
        // Set the system ID from the bd addr
        //uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
        //GAPRole_GetParameter(GAPROLE_BD_ADDR, systemId);
/*
        // shift three bytes up
        systemId[7] = systemId[5];
        systemId[6] = systemId[4];
        systemId[5] = systemId[3];

        // set middle bytes to zero
        systemId[4] = 0;
        systemId[3] = 0;

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
        */
      }
      break;

    //if the state changed to connected, initially assume that keyfob is in range
    case GAPROLE_ADVERTISING:
      {
        // Visual feedback that we are advertising.
        //HalLedSet( OutSign, HAL_LED_MODE_ON );
        //osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
      }
      break;
      
    //if the state changed to connected, initially assume that keyfob is in range      
    case GAPROLE_CONNECTED:
      {
        
        GAPRole_GetParameter( GAPROLE_CONNHANDLE, &connHandle );

        #if defined ( PLUS_BROADCASTER )
          osal_start_timerEx( keyfobapp_TaskID, SKP_ADV_IN_CONNECTION_EVT, ADV_IN_CONN_WAIT );
        #endif
          
        // Turn off LED that shows we're advertising
        //HalLedSet( OutSign, HAL_LED_MODE_OFF );
        
      }
      break;

    case GAPROLE_WAITING:
      {
                
        // Change attribute value of Accelerometer Enable to FALSE
        RF_SetParameter(RF_ENABLER, sizeof(valFalse), &valFalse);
        // Stop the acceleromter
        rfEnablerChangeCB(); // SetParameter does not trigger the callback
        
        // Turn off LED that shows we're advertising
        //HalLedSet( OutSign, HAL_LED_MODE_OFF );
        
        // Stop alert if it was active
        if( keyfobAlertState != ALERT_STATE_OFF )
        {
          keyfobapp_StopAlert();
        }
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        
        // Change attribute value of Accelerometer Enable to FALSE
        RF_SetParameter(RF_ENABLER, sizeof(valFalse), &valFalse);
        // Stop the acceleromter
        rfEnablerChangeCB(); // SetParameter does not trigger the callback
        
       
      }
      break;

    default:
      // do nothing
      break;
    }
  }

  gapProfileState = newState;
}

/*********************************************************************
 * @fn      accelEnablerChangeCB
 *
 * @brief   Called by the Accelerometer Profile when the Enabler Attribute
 *          is changed.
 *
 * @param   none
 *
 * @return  none
 */
static void rfEnablerChangeCB( void )
{
  bStatus_t status = RF_GetParameter( RF_ENABLER, &rfEnabler );

  if (status == SUCCESS){
    if (rfEnabler)
    {
      // Initialize accelerometer
      //accInit();
      //osal_snv_read(SNV_ID_APP, BUF_LEN, (uint8 *)buf);
      //rfCnt = buf[0];
      //RF_SetParameter(RF_REC, sizeof ( int8 ), &rfCnt);
      //VOID RF_GetParameter(RF_REC,&rfCnt);
      
      uint8 res=0;
      res = RF_Init();
      if(res) {
        //HalLedBlink (OutSign, 3, 50, 100);
        
        RF_ResetReadFIFO();
        RF_RX();
        RF_SetParameter(RF_REC, 4, buf);
      }     
      
      // Setup timer for accelerometer task
      osal_start_timerEx( keyfobapp_TaskID, SKP_RF2G4_READ_EVT, RF2G4_READ_PERIOD );
    } else
    {
      // Stop the acceleromter
      //accStop();
      //buf[0]=rfCnt;
      //VOID osal_snv_write(SNV_ID_APP, BUF_LEN, (uint8 *)buf);
      osal_stop_timerEx( keyfobapp_TaskID, SKP_RF2G4_READ_EVT);
    }
  } else
  {
    //??
  }
  
  status = RF_GetParameter( RF_REC, buf);
  if (status == SUCCESS){
    //buzzerStart( BUZZER_ALERT_LOW_FREQ );
    //buzzer_state = BUZZER_ON;
    //osal_start_timerEx( keyfobapp_TaskID, SKP_TOGGLE_BUZZER_EVT, 200 );
    switch (buf[0]){
        case 'a':
          //P0_5 =0;P0_6 =0;
          keyCnt=0;
          //OutBtn=1;
          osal_start_timerEx( keyfobapp_TaskID, SKP_TogOut_TIMEOUT_EVT,10);
        break;
        case 'b':
          P0_5 =0;P0_6 =1;
           
        break;
        case 'c':
          P0_5 =1;P0_6 =1;
         
        break;
    }
  }
}
/*********************************************************************
*********************************************************************/
