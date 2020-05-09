/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "linkdb.h"
#include "OSAL.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "rfservice.h"

#include "hal_board_cfg.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        6//8

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// RF Service UUID
CONST uint8 rfServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(RF_SERVICE_UUID), HI_UINT16(RF_SERVICE_UUID)
};

// RF Enabler UUID
CONST uint8 rfEnablerUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(RF_ENABLER_UUID), HI_UINT16(RF_ENABLER_UUID)
};

// RF rec UUID
CONST uint8 rfRecUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(RF_REC_UUID), HI_UINT16(RF_REC_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// con trỏ hàm callback của app
static rfCBs_t *rf_AppCBs = NULL;


/*********************************************************************
 * Profile Attributes - variables
 */

// Accelerometer Service attribute
static CONST gattAttrType_t rfService = { ATT_BT_UUID_SIZE, rfServUUID };


// Enabler Characteristic Properties
static uint8 rfEnabledCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Enabler Characteristic Value
static uint8 rfEnabled = 0;

// Enabler Characteristic user description
//static uint8 rfEnabledUserDesc[11] = "RF Enable";


// RF rec Characteristic Properties
static uint8 rfRecCharProps = GATT_PROP_READ |GATT_PROP_NOTIFY|GATT_PROP_WRITE;

// RF rec Characteristic Value
static uint8 rfRec[5] = {0x12,0x34,0x45,0x67,0x87};

// RF rec Characteristic user description
//static uint8 rfRecUserDesc[11] = "RF Recive";

// RF rec Characteristic Configs
// cho phép người dùng config(bật notify)
static gattCharCfg_t *rfRecConfig;


/*********************************************************************
 * Profile Attributes - Table
 */
static gattAttribute_t rfAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // Accelerometer Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                   /* permissions */
    0,                                  /* handle */
    (uint8 *)&rfService                /* pValue */
  },

//---------Rf enable  
    // Enabler Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &rfEnabledCharProps 
    },

      // Enable Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, rfEnablerUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0,
        &rfEnabled 
      },
/*
      // Enable User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        rfEnabledUserDesc 
      },
      */
//------------ Rf recive
    // Rec Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &rfRecCharProps
    },

      // rec Char Value
      { 
        { ATT_BT_UUID_SIZE,rfRecUUID },
        GATT_PERMIT_READ| GATT_PERMIT_WRITE, 
        0,
        rfRec 
      },
	  // rf rec Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&rfRecConfig 
      },
      // rec Range User Description
      /*
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0,
        rfRecUserDesc 
      },
      */
  
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t rf_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                   uint8 *pValue, uint8 *pLen, uint16 offset,
                                   uint8 maxLen, uint8 method );
static bStatus_t rf_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint8 len, uint16 offset,
                                    uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
//  RF Service Callbacks
//  khai báo biến tên rfCBs(con trỏ) kiểu gattServiceCBs_t được gán =rf_ReadAttrCB,rf_WriteAttrCB,
//  theo cấu trúc kiểu gattServiceCBs_t
CONST gattServiceCBs_t  rfCBs =
{
  rf_ReadAttrCB,  // Read callback function pointer
  rf_WriteAttrCB, // Write callback function pointer
  NULL               // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Accel_AddService
 *
 * @brief   Initializes the Accelerometer service by
 *          registering GATT attributes with the GATT server. Only
 *          call this function once.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t RF_AddService( uint32 services )
{
  uint8 status = SUCCESS;
  size_t allocSize = sizeof(gattCharCfg_t) * linkDBNumConns;

  // Allocate Client Characteristic Configuration tables
  // câp phát bộ nhớ cho biến rfRecConfig - thực hiện khi client tác động
  rfRecConfig = (gattCharCfg_t *)osal_mem_alloc( allocSize );
  if ( rfRecConfig == NULL )
  {     
    return ( bleMemAllocError );
  }
  
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, rfRecConfig );
 
  if ( services & RF_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    // đăng ký (gán giá trị khởi tạo) 1 sevice với bảng attributes 
    status = GATTServApp_RegisterService( rfAttrTbl, 
                                          GATT_NUM_ATTRS( rfAttrTbl ),// sô phần từ
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &rfCBs );// callback
  }

  return ( status );
}

/*********************************************************************
 * @fn      Accel_RegisterAppCBs
 *
 * @brief   Does the profile initialization.  Only call this function
 *          once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
// hàm tạo đăng ký cho AppCallback
bStatus_t RF_RegisterAppCBs( rfCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    rf_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}


/*********************************************************************
 * @fn      Accel_SetParameter
 *
 * @brief   Set an Accelerometer Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t RF_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case RF_ENABLER:
      if ( len == sizeof ( uint8 ) ) 
      {
        rfEnabled = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    case RF_REC:
      if ( len == 5 ) 
      {
        /*
        rfRec[0] = *((uint8*)value);
        rfRec[1] = *((uint8*)value+1);
        rfRec[2] = *((uint8*)value+2);
        rfRec[3] = *((uint8*)value+3);
        */
        
        VOID osal_memcpy(rfRec,value, 5);
         // See if Notification has been enabled
         // kiểm tra giá trị config , đồng thời cập nhật giá trị set rfRec đến client "rf_ReadAttrCB"
        // goi lại hàm rf_ReadAttrCB để cập nhật nếu config là notify
        GATTServApp_ProcessCharCfg( rfRecConfig, rfRec,
                                    FALSE, rfAttrTbl, GATT_NUM_ATTRS( rfAttrTbl ),
                                    INVALID_TASK_ID, rf_ReadAttrCB );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      Accel_GetParameter
 *
 * @brief   Get an Accelerometer Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t RF_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case RF_ENABLER:
      *((uint8*)value) = rfEnabled;
      break;
      
    case RF_REC:
      /*
      *((uint8*)value) = rfRec[0];
      *((uint8*)value+1) = rfRec[1];
      *((uint8*)value+2) = rfRec[2];
      *((uint8*)value+3) = rfRec[3];
      */
      VOID osal_memcpy(value, rfRec, 5);
      break;
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          accel_ReadAttr
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message 
 *
 * @return      SUCCESS, blePending or Failure
 */
// đc gọi khi client gửi lệnh đọc - "ReadAttrCB"
// công việc là gán giá trị cần đọc *pAttr->pValue (dựa theo UUID) cho pValue
// GATT sẽ truyền giá trị đó đến client
static bStatus_t rf_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                   uint8 *pValue, uint8 *pLen, uint16 offset,
                                   uint8 maxLen, uint8 method )
{
  uint16 uuid;
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {    
    // 16-bit UUID
    uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those types for reads

      case RF_ENABLER_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;
      case RF_REC_UUID:
        *pLen = 5;
        
        //pValue[0] = *pAttr->pValue;
        //pValue[1] = *(pAttr->pValue+1);
        //pValue[2] = *(pAttr->pValue+2);
        VOID osal_memcpy(pValue, pAttr->pValue, *pLen);
        break;
      
      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }


  return ( status );
}

/*********************************************************************
 * @fn      accel_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message 
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t rf_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                    uint8 *pValue, uint8 len, uint16 offset,
                                    uint8 method )
{
  bStatus_t status = SUCCESS;
  uint8 notify = 0xFF; // biến cho biết trạng thái khi client ghi giá trị enable =1-> gọi appCb

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case RF_ENABLER_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          /*
          if ( len > 1 )
            status = ATT_ERR_INVALID_VALUE_SIZE;
          else if ( pValue[0] != FALSE && pValue[0] != TRUE ) // kiểm tra giá trị chỉ đc phép TRUE or FALSE
            status = ATT_ERR_INVALID_VALUE;
          */
          
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          
          *pCurValue = pValue[0];
          notify = RF_ENABLER;  // chỉ là 1 giá trj bất kỳ miễn khác giá trị đầu      
        }
             
        break;
      case RF_REC_UUID: 
        if ( offset == 0 )
        {
          if ( len != 5 )
            status = ATT_ERR_INVALID_VALUE_SIZE;       
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          
          //*pCurValue = pValue[0];
          //*(pCurValue+1) = pValue[1];
          //*(pCurValue+2) = pValue[2];
          VOID osal_memcpy(pCurValue,pValue,5);
          //if(*pCurValue == 'a') P0_5 =1;
          //else P1_1 =0;
          /*
          switch (*pCurValue){
            case 'a':
              P0_5 =0;P0_6 =0;             
            break;
            case 'b':
              P0_5 =0;P0_6 =1;
            break;
            case 'c':
              P0_5 =1;P0_6 =1;
            break;
            case 'd':
              P0_5 =1;P0_6 =0;
            break;
          }
          */
          notify =6;
        }
        break;
      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;      
          
      default:
          // Should never get here!
          status = ATT_ERR_ATTR_NOT_FOUND;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }  

  // If an attribute changed then callback function to notify application of change
  if ( (notify != 0xFF) && rf_AppCBs && rf_AppCBs->pfnRfEnabler )// nếu đã khai báo
    rf_AppCBs->pfnRfEnabler();  
  
  return ( status );
}


/*********************************************************************
*********************************************************************/
