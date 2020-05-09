
#ifndef RF2G4_H
#define RF2G4_H

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

// Profile Parameters - danh sach đối số
#define RF_ENABLER                 0  // RW uint8 - Profile Attribute value
#define RF_REC                  1  // RW int16 - Profile Attribute value
  
// Profile UUIDs - của service đc tạo
#define RF_ENABLER_UUID          0xFFA1
#define RF_REC_UUID              0xFFA2

  
// Accelerometer Service UUID
#define RF_SERVICE_UUID            0xFFA0
  

// Accelerometer Profile Services bit fields
#define RF_SERVICE                 0x00000001

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */
// Callback when the device has been started.  Callback event to 
// the ask for a battery check.
// định nghĩa kiểu con trỏ hàm có tên "rfEnabler_t"
typedef void (*rfEnabler_t)(void); 

// định nghĩa kiểu "rfCBs_t" trong đó bao gồm 1 con trỏ hàm kiểu "rfEnabler_t"
// có tên "pfnRfEnabler"
typedef struct
{
  rfEnabler_t     pfnRfEnabler;  // Called when Enabler attribute changes
} rfCBs_t; 

/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * Accel_AddService - Initializes the Accelerometer service by registering 
 *          GATT attributes with the GATT server. Only call this function once.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */
extern bStatus_t RF_AddService(uint32 services);

/*
 * Accel_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t RF_RegisterAppCBs(rfCBs_t *appCallbacks);


/*
 * Accel_SetParameter - Set an Accelerometer Profile parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t RF_SetParameter(uint8 param, uint8 len, void *value);
  
/*
 * Accel_GetParameter - Get an Accelerometer Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t RF_GetParameter(uint8 param, void *value);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ACCELEROMETER_H */
