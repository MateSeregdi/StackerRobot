/**************************************************************************************************
* Copyright © 2024 ams-OSRAM AG                                                                   *
* All rights are reserved.                                                                        *
*                                                                                                 *
* FOR FULL LICENSE TEXT SEE LICENSES-MIT.TXT                                                      *
*                                                                                                 *
**************************************************************************************************/

/** @file This is the tmf8829 arduino uno driver example console application. 
 */

#ifndef TMF8829_APP_H
#define TMF8829_APP_H

// ---------------------------------------------- includes ----------------------------------------
#include "tmf8829_shim.h"
#include "tmf8829.h"       // for TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_* defines
#include "tof_results.h"   // for TofResults struct
// ---------------------------------------------- functions ---------------------------------------

/** @brief Arduino setup function is only called once at startup. Do all the HW initialisation stuff here.
 * @param logLevelIdx ...  the log level index to be used (0..8 -> see logLevels array in tmf8829_app.cpp)
 * @param  baudrate ... for the serial input the baudrate
 * @param  i2cClockSpeedInHz ... the i2c frequency
 */
void setupFn( uint8_t logLevelIdx, uint32_t baudrate, uint32_t i2cClockSpeedInHz );
/** @brief Arduino main loop function, is executed cyclic.

 * @return 1 if wants to be called again
 * @return 0 if program should terminate
 */
int8_t loopFn( );

/** @brief Arduino terminate function is only called once when exit key 'q' is pressed. Write a message and wait for shutdown of arduino.
 */
void terminateFn( );

/** @brief Enable the device, download firmware, and start measuring automatically.
 *  Call this once after setupFn() to begin readings without needing serial commands.
 */
void autoStartFn( );

/** @brief Stop measurement, apply a new resolution preset, then restart measuring.
 *  @param cfgCmd  one of the TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_* defines, e.g.:
 *    CMD_LOAD_CFG_8X8            CMD_LOAD_CFG_8X8_LONG_RANGE   CMD_LOAD_CFG_8X8_HIGH_ACCURACY
 *    CMD_LOAD_CFG_16X16          CMD_LOAD_CFG_16X16_HIGH_ACCURACY
 *    CMD_LOAD_CFG_32X32          CMD_LOAD_CFG_32X32_HIGH_ACCURACY
 *    CMD_LOAD_CFG_48X32          CMD_LOAD_CFG_48X32_HIGH_ACCURACY
 */
void setResolutionFn( int cfgCmd );

/** @brief Stop measurement and fully disable the device. */
void disableFn( );

#endif // TMF8829_APP_H
