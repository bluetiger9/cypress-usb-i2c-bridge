/**
 * @file cs3240.h
 *
 * @brief Cypress CY3240 I2C to USB Bridge
 *
 * Define constant packets, addresses, lengths, etc. for writing to theDefine constant packets, addresses, lengths, etc. for writing to the
 * CY3240 USB HID to I2C converter
 * 
 * @ingroup CY3240
 *
 * @owner  Kevin Kirkup (kevin.kirkup@gmail.com)
 * @author Kevin Kirkup (kevin.kirkup@gmail.com)
 */
#ifndef INCLUSION_GUARD_CS3240_H
#define INCLUSION_GUARD_CS3240_H

#ifdef __cplusplus
extern "C" {
#endif

//////////////////////////////////////////////////////////////////////
/// @name Includes
//@{

#include <stdint.h>
#include "cy3240_types.h"

//@} End of Includes

//////////////////////////////////////////////////////////////////////
/// @name Defines
//@{

// The product and vendor id for the Cypress USB-I2C bridge
#define CY3240_VID (0x04B4)
#define CY3240_PID (0xF147)

/* Debug configuration */
#if DEBUG
#define DBG(x) x

#else
#define DBG(x)

#endif // DEBUG

#define TRUE   (1)
#define FALSE  (0)

/* Result Macros */
#define CY3240_SUCCESS(s)  ((s == CY3240_ERROR_OK) ? TRUE : FALSE)
#define CY3240_FAILURE(s)  ((s != CY3240_ERROR_OK) ? TRUE : FALSE)

//@} End of Defines

//////////////////////////////////////////////////////////////////////
/// @name Methods
//@{

//-----------------------------------------------------------------------------
/**
 *  Method to restart the CY3240 bridge chip
 *
 *  @param handle [in] the handle to the bridge controller
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_restart(
        void *handle
        );

//-----------------------------------------------------------------------------
/**
 *  Method to reconfigure the settings for the CY3240 bridge chip
 *
 *  @param handle [in] the handle to the bridge controller
 *  @param power  [in] the power mode
 *  @param bus    [in] the bus mode
 *  @param clock  [in] the clock speed
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_reconfigure(
        void *handle,
        Cy3240_Power_t power,
        Cy3240_Bus_t bus,
        Cy3240_I2C_ClockSpeed_t clock
        );

//-----------------------------------------------------------------------------
/**
 *  Method to reinitialize the CY3240 bridge chip
 *
 *  @param handle [in] the handle to the bridge controller
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_reinit(
        void *handle
        );

//-----------------------------------------------------------------------------
/**
 *  Method to write data to the CY3240
 *
 *  @param handle [in] the handle to the bridge controller
 *  @param address [in] the I2C Address of the device to read
 *  @param pData   [in] the data read from the device
 *  @param pLength [in] the length of the read data
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_write(
        void *handle,
        uint8_t address,
        const uint8_t* const pData,
        uint16_t* const pLength
        );

//-----------------------------------------------------------------------------
/**
 *  Method to read data from the CY3240
 *
 *  @param handle  [in] the handle to the bridge controller
 *  @param address [in] the I2C Address of the device to read
 *  @param pData   [in] the data read from the device
 *  @param pLength [in] the length of the read data
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_read(
        void *handle,
        uint8_t address,
        uint8_t* const pData,
        uint16_t* const pLength
        );

//-----------------------------------------------------------------------------
/**
 *  Method to open the CY3240
 *
 *  @param handle [in] the handle to the bridge controller
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_open(
        void *handle
        );

//-----------------------------------------------------------------------------
/**
 *  Method to close the CY3240 usb device
 *
 *  @param handle [in] the handle to the bridge controller
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_close(
        void *handle
        );

//-----------------------------------------------------------------------------
/**
 *  Factory method to initialize the Cy3240 data structure and internal state
 *
 *  @param pCy3240      [out] the data structure to initialize
 *  @param iface_number [in] the interface number of the device
 *  @param timeout      [in] the timeout for tx and rx
 *  @param power        [in] the power mode to use
 *  @param bus          [in] the bus type to use
 *  @param clock        [in] the I2C clock rate to use
 *  @returns Cy3240_Error_t
 */
//-----------------------------------------------------------------------------
Cy3240_Error_t
cy3240_factory(
        void **pHandle,
        int iface_number,
        int timeout,
        Cy3240_Power_t power,
        Cy3240_Bus_t bus,
        Cy3240_I2C_ClockSpeed_t clock
        );

//@} End of Methods

#ifdef __cplusplus
}
#endif

#endif // INCLUSION_GUARD_CS3240_H
