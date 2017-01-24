/**
 * @file unittest
 *
 * @brief Unit test common defines
 *
 * Unit test common defines
 *
 * @ingroup 
 *
 * @owner  Kevin S Kirkup (kevin.kirkup@gmail.com)
 * @author Kevin S Kirkup (kevin.kirkup@gmail.com)
 */
#ifndef INCLUSION_GUARD_UNITTEST_H
#define INCLUSION_GUARD_UNITTEST_H

#ifdef __cplusplus
extern "C" {
#endif

//////////////////////////////////////////////////////////////////////
/// @name Includes
//@{

#include "config.h"
#include "cy3240.h"
#include "cy3240_types.h"
#include "cy3240_private_types.h"
#include "cy3240_util.h"
#include "cy3240_packet.h"
#include "AceUnitData.h"

//@} End of Includes

//////////////////////////////////////////////////////////////////////
/// @name Defines
//@{

#define MY_ADDRESS             (0)
#define SEND_BUFFER_SIZE       (128)
#define RECEIVE_BUFFER_SIZE    (128)

//@} End of Defines

//////////////////////////////////////////////////////////////////////
/// @name Data
//@{

/**
 * The component that will be used for the unit tests.
 */
extern Cy3240_t* pMyData;

// The sending buffer
extern uint8_t SEND_BUFFER[SEND_BUFFER_SIZE];
extern uint8_t RECEIVE_BUFFER[SEND_BUFFER_SIZE];

//@} End of Data

//////////////////////////////////////////////////////////////////////
/// @name Methods
//@{

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID init
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
int
testGenericInit(
        void
        );

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID close
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
int
testGenericClose(
        hid_device *const hidif
        );

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID read
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
int
testGenericRead(
        hid_device* const hidif,
        unsigned int const ep,
        char* const bytes,
        unsigned int const size,
        unsigned int const timeout
        );

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID write
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
int
testGenericWrite(
        hid_device* const hidif,
        unsigned int const ep,
        const char* bytes,
        unsigned int const size,
        unsigned int const timeout
        );

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID clean
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
int
testGenericCleanup(
        void
        );

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID delete interface
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
void
testGenericDeleteIf(
        hid_device **const hidif
        );

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID Force Open
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
hid_device * 
testOpen(
        unsigned short vendor_id, 
		unsigned short product_id, 
		const wchar_t *serial_number
        );

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the new HID Interface
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
hid_device *
testGenericNewHidInterface(
        void
        );

//@} End of Methods

//////////////////////////////////////////////////////////////////////
/// @name Methods
//@{

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID init
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
int
testInit(
        void
        );

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID read
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
int
testRead(
        hid_device* const hidif,
        unsigned int const ep,
        char* const bytes,
        unsigned int const size,
        unsigned int const timeout
        );

//-----------------------------------------------------------------------------
/**
 *  Substitute method for the HID write
 *
 *  @see hid.h
 *  @returns int
 */
//-----------------------------------------------------------------------------
int
testWrite(
        hid_device* const hidif,
        unsigned int const ep,
        const char* bytes,
        unsigned int const size,
        unsigned int const timeout
        );

//@} End of Methods

#ifdef __cplusplus
}
#endif

#endif // INCLUSION_GUARD_UNITTEST_H
