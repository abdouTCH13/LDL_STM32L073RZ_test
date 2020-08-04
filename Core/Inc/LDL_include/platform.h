/*
 * platform.h
 *
 *  Created on: Jun 19, 2020
 *      Author: user
 */

#ifndef INC_LDL_INCLUDE_PLATFORM_H_
#define INC_LDL_INCLUDE_PLATFORM_H_



#endif /* INC_LDL_INCLUDE_PLATFORM_H_ */

/* timing settings */

/* +/- error as PPM
 *
 * Assume a ceramic resonator has ~0.5% error, so
 *
 * 16M * 0.005 = 80000
 *
 * 80000 / 16 = 5000
 *
 * */
//#define XTAL_PPM 5000

/* optimisations ******************************************************/

/* the atmega328p is short on RAM, you probably shouldn't increase this */
#define LDL_MAX_PACKET UINT8_MAX
/* optionally constrain to one radio */
//#define LDL_ENABLE_SX1272
//#define LDL_ENABLE_SX1276

/* optionally constrain to one or more regions */
//#define LDL_ENABLE_EU_863_870
//#define LDL_ENABLE_EU_433
//#define LDL_ENABLE_US_902_928
//#define LDL_ENABLE_AU_915_928

/* define to keep the RX buffer in mac state rather than on the stack */
//#define LDL_ENABLE_STATIC_RX_BUFFER

/* time in milliseconds until channels become available after device reset
 *
 * this is a safety feature for devices stuck in a transmit-reset loop, perhaps
 * due to an end-of-life battery .
 *
 *  */
#define LDL_STARTUP_DELAY 0UL

/* optionally disable these event callbacks */
/*
#define LDL_DISABLE_CHECK
#define LDL_DISABLE_DEVICE_TIME
#define LDL_DISABLE_MAC_RESET_EVENT
#define LDL_DISABLE_CHIP_ERROR_EVENT
#define LDL_DISABLE_DOWNSTREAM_EVENT
#define LDL_DISABLE_TX_BEGIN_EVENT
#define LDL_DISABLE_TX_COMPLETE_EVENT
#define LDL_DISABLE_SLOT_EVENT
#define LDL_DISABLE_JOIN_TIMEOUT_EVENT
#define LDL_DISABLE_JOIN_COMPLETE_EVENT
#define LDL_DISABLE_DATA_TIMEOUT_EVENT
#define LDL_DISABLE_DATA_COMPLETE_EVENT
#define LDL_DISABLE_DATA_CONFIRMED_EVENT
#define LDL_DISABLE_RX_EVENT

 do not change *****************************************************

#define LDL_DISABLE_SESSION_UPDATE
#define LDL_DISABLE_POINTONE
#define LDL_ENABLE_STATIC_RX_BUFFER
#define LDL_DISABLE_CHECK
#define LDL_DISABLE_DEVICE_TIME
#define LDL_DISABLE_FULL_CHANNEL_CONFIG
#define LDL_DISABLE_CMD_DL_CHANNEL

#define LDL_ENABLE_AVR
#define LDL_LITTLE_ENDIAN
#define LDL_SYSTEM_ENTER_CRITICAL(APP) ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
#define LDL_SYSTEM_LEAVE_CRITICAL(APP) }

#endif
*/
#define LDL_DISABLE_POINTONE


#define LDL_SYSTEM_ENTER_CRITICAL(APP) volatile uint32_t primask = __get_PRIMASK(); __disable_irq();
#define LDL_SYSTEM_LEAVE_CRITICAL(APP) __set_PRIMASK(primask);


#include "debug.h"
#define LDL_INFO(APP,...) do{printf("Info : ");printf(__VA_ARGS__);printf("\r\n");}while(0);
#define LDL_ERROR(APP,...) do{printf("Error : ");printf(__VA_ARGS__);printf("\r\n");}while(0);
#define LDL_DEBUG(APP,...) do{printf("Debug : ");printf(__VA_ARGS__);printf("\r\n");}while(0);
