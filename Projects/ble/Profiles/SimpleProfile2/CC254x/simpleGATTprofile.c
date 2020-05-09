/******************************************************************************

 @file  simpleGATTprofile.c

 @brief This file contains the Simple GATT profile sample GATT service profile
        for use with the BLE sample application.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2010-2019, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: ble_sdk_1.5.0.16
 Release Date: 2019-04-18 08:53:32
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "simpleGATTprofile.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        17

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// User information Service UUID: 0xFFA0
CONST uint8 UserInfServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(USERINFOR_SERV_UUID), HI_UINT16(USERINFOR_SERV_UUID)
};

// User Command UUID: 0xFFA1
CONST uint8 UserCommandUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(UserCommand_UUID), HI_UINT16(UserCommand_UUID)
};

// User Notify UUID: 0xFFA2
CONST uint8 UserNotifyUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(UserNotify_UUID), HI_UINT16(UserNotify_UUID)
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

static UserInfCBs_t *UserInf_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// User infor Service attribute
static CONST gattAttrType_t UserInfService = { ATT_BT_UUID_SIZE, UserInfServUUID };


// UserCommnad Properties
static uint8 UserCommandProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 UserCommandVal[UserCommand_LEN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0};
static uint8 UserCommandDesp[8] = "Command\0"; // + null

// UserNotify Properties
static uint8 UserNotifyProps = GATT_PROP_READ |GATT_PROP_NOTIFY| GATT_PROP_WRITE;
static uint8 UserNotifyVal[UserNotify_LEN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0};
static uint8 UserNotifyDesp[7] = "Notify\0"; 
// UserName Characteristic Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t *UserNotifyConfig;


/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t SmartKeyAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // User Infor Service Declaration
  // 
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&UserInfService            /* pValue */
  },

   // Characteristic User Command Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &UserCommandProps 
    },

      // Add Characteristic  User Command Value
      { 
        { ATT_BT_UUID_SIZE, UserCommandUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        UserCommandVal 
      },
                
      
      // Characteristic Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        UserCommandDesp 
      },      
      
      
   // Characteristic User Notify Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &UserNotifyProps 
    },

      // Add Characteristic  User Notify Value
      { 
        { ATT_BT_UUID_SIZE, UserNotifyUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        UserNotifyVal 
      },
      
      // Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&UserNotifyConfig 
      }, 
      // Characteristic Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        UserNotifyDesp 
      },  
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t UserInf_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                           uint8 *pValue, uint8 *pLen, uint16 offset,
                                           uint8 maxLen, uint8 method );
static bStatus_t UserInf_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint8 len, uint16 offset,
                                            uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t UserInfCBs =
{
  UserInf_ReadAttrCB,  // Read callback function pointer
  UserInf_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      UserInf_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t UserInfo_AddService( uint32 services )
{
  uint8 status;
  
  // Allocate Client Characteristic Configuration table
  UserNotifyConfig = (gattCharCfg_t *)osal_mem_alloc( sizeof(gattCharCfg_t) *
                                                              linkDBNumConns );
  if ( UserNotifyConfig == NULL )
  {     
    return ( bleMemAllocError );
  }
  
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, UserNotifyConfig );
  
  if ( services & USERINFOR_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( SmartKeyAttrTbl, 
                                          GATT_NUM_ATTRS( SmartKeyAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &UserInfCBs );
  }
  else
  {
    status = SUCCESS;
  }
  
  return ( status );
}

/*********************************************************************
 * @fn      UserInf_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t UserInf_RegisterAppCBs( UserInfCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    UserInf_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      UserInf_SetParameter
 *
 * @brief   Set a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t UserInf_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    
    case UserNotify:
      if ( len == UserNotify_LEN ) 
      {
        VOID memcpy( UserNotifyVal, value, UserNotify_LEN );
        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( UserNotifyConfig, UserNotifyVal, FALSE,
                                    SmartKeyAttrTbl, GATT_NUM_ATTRS( SmartKeyAttrTbl),
                                    INVALID_TASK_ID, UserInf_ReadAttrCB );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    case UserCommand:
      if ( len == UserCommand_LEN ) 
      {
        VOID memcpy( UserCommand, value, UserCommand_LEN );       
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
 * @fn      UserInf_GetParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t UserInf_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {      
    case UserCommand:
      VOID memcpy( value, UserCommandVal, UserCommand_LEN );
      break;
      case UserNotify:
      VOID memcpy( value, UserNotifyVal, UserNotify_LEN );
      break; 
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          UserInf_ReadAttrCB
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
static bStatus_t UserInf_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                           uint8 *pValue, uint8 *pLen, uint16 offset,
                                           uint8 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      // characteristics 1 and 2 have read permissions
      // characteritisc 3 does not have read permissions; therefore it is not
      //   included here
      // characteristic 4 does not have read permissions, but because it
      //   can be sent as a notification, it is included here
    /*  
    case UserInf_CHAR1_UUID:
      case UserInf_CHAR2_UUID:
      case UserInf_CHAR4_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;
*/
      
    case UserNotify_UUID:
        *pLen = UserNotify_LEN;
        VOID memcpy( pValue, pAttr->pValue, UserNotify_LEN );
        break;
    case UserCommand_UUID:
        *pLen = UserCommand_LEN;
        VOID memcpy( pValue, pAttr->pValue, UserCommand_LEN );
        break;
        
      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
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
 * @fn      UserInf_WriteAttrCB
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
static bStatus_t UserInf_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint8 len, uint16 offset,
                                            uint8 method )
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  
  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      /*
      case UserInf_CHAR1_UUID:
      case UserInf_CHAR3_UUID:

        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != 1 )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
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

          if( pAttr->pValue == &UserInfChar1 )
          {
            notifyApp = UserInf_CHAR1;        
          }
          else
          {
            notifyApp = UserInf_CHAR3;           
          }
        }
             
        break;
*/
    case UserCommand_UUID:
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;        
          //*pCurValue = pValue[0];
          VOID memcpy( pCurValue, pValue, len );
        }
        break;  
    case UserNotify_UUID:
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;        
          //*pCurValue = pValue[0];
          VOID memcpy( pCurValue, pValue, len );
        }
        break;
      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;
        
      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && UserInf_AppCBs && UserInf_AppCBs->pfnUserInfChange )
  {
    UserInf_AppCBs->pfnUserInfChange( notifyApp );  
  }
  
  return ( status );
}

/*********************************************************************
*********************************************************************/
